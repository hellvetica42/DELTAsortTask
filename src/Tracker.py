import time
from multiprocessing.connection import Connection
from multiprocessing import Queue
import json
import numpy as np

import sys
sys.path.append(".")
from CameraCalibration.CameraMapping import CameraMapping
from CameraCalibration.ConveyorTracker import ConveyorTracker
from DDS.Cloud2DPublisher import Cloud2DPublisher

class TrackerProcess:
    def __init__(self):
        self.prev_points = []
        self.tracks : dict[int, ConveyorTracker] = {}
        self.max_track_id = 0
        self.max_distance = 50

        with open('Config/robot_calibration.json', 'r') as f:
            self.config = json.load(f)
            self.config = {k: np.array(v, dtype=np.float64) for k,v in self.config.items()}
            #self.config['pos_dst'] = self.config['pos_dst'] + self.config['offset']

        self.cloud_publisher = Cloud2DPublisher("TRACKS")

    def similarity_metric(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def match_point(self, point, track_pool):
        tracker_pts = [np.array(self.tracks[track_id].get_current_state()['position']) for track_id in track_pool]
        distances = [self.similarity_metric(point, tracker_pt) for tracker_pt in tracker_pts]
        min_distance = min(distances)
        min_index = distances.index(min_distance)
        track_key = list(track_pool)[min_index]
        return track_key, min_distance

    def get_all_positions(self):
        positions = []
        for track_id in self.tracks:
            positions.append(self.tracks[track_id].get_current_state()['position'])
        return np.array(positions)


    def update_tracks(self, new_points):
        if new_points is None:
            for t in self.tracks:
                self.tracks[t].update(None)
            return

        for t in self.tracks:
            self.tracks[t].matched = False

        def create_new_tracker(point):
            self.tracks[self.max_track_id] = ConveyorTracker(0.038, 0.05, 0.1, self.max_track_id)
            self.tracks[self.max_track_id].update(np.array(point))
            matched_tracks.append(self.max_track_id)
            self.max_track_id += 1


        matched_tracks = []
        if len(self.tracks) == 0:
            for point in new_points:
                #self.tracks[self.track_id] = Track(self.track_id, point, time.time())
                create_new_tracker(point)
        else:
            track_pool = list(self.tracks.keys())
            for point in new_points:
                if len(track_pool) > 0:
                    matched_track_id, distance = self.match_point(point, track_pool)
                else:
                    distance = self.max_distance

                if distance < self.max_distance:
                    self.tracks[matched_track_id].update(np.array(point))
                    matched_tracks.append(matched_track_id)
                    track_pool.remove(matched_track_id)
                else:
                    create_new_tracker(point)

        # Update all non matched tracks so that kalman predicts them
        for track_id in self.tracks:
            if track_id not in matched_tracks:
                self.tracks[track_id].update(None)

        self.prev_points = new_points

    def cleanup_tracks(self, age_threshold=7.0):
        to_pop = []
        for track_id, track in self.tracks.items():
            if time.time() - track.last_seen > age_threshold:
                to_pop.append(track_id)
        for track_id in to_pop:
            self.tracks.pop(track_id)
            print("Removed track", track_id)

    def run(pipe_parent : Connection, pipe_child : Connection):
        tracker = TrackerProcess()

        previous_tracks = []

        while True:
            pipe_parent.poll(timeout=None)
            msg = pipe_parent.recv()
            detections_px = np.array([])
            if msg[0] == "EXIT":
                pipe_child.send(("EXIT", None))
                break
            elif msg[0] == "DETECTIONS":
                detections = msg[1]
                detections_px = detections['detection_centers']


            if detections_px.shape[0] > 0:

                detections_uv = CameraMapping.map_from_pixels_to_uv(detections_px, tracker.config["frame_size"])
                detections_mm = CameraMapping.apply_homography(detections_uv, tracker.config["homography"])

                tracker.update_tracks(detections_mm)
            else:
                tracker.update_tracks(None)

                
            for key in tracker.tracks:
                if key not in previous_tracks:
                    previous_tracks.append(key)
                    #print("New track", key)

            pipe_child.send(("TRACK", tracker.tracks))

            #gui_queue.put(("UPDATE_TRACKER_POINTS", tracker.get_all_positions()))
            tracker.cloud_publisher.publish(tracker.get_all_positions())

            
            tracker.cleanup_tracks()

            

            