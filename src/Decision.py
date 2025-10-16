import time
import numpy as np
from multiprocessing.connection import Connection
from multiprocessing import Queue
from CameraCalibration.ConveyorTracker import ConveyorTracker as Tracker
from DDS.Cloud2DPublisher import Cloud2DPublisher
import threading
from threading import Event, Lock
from CameraCalibration.CameraMapping import CameraMapping


tracker_lock = Lock()
end_event = Event()
class DecisionProcess:
    def __init__(self, pipe_parent, pipe_child):
        self.pipe_parent : Connection = pipe_parent
        self.pipe_child : Connection = pipe_child
        self.tracks : dict[int, Tracker] = {}

        self.pick_height = -615
        self.hover_height = -600
        self.current_id = None
        self.home_point = np.array([0,0,-350])
        self.dropoff_point = np.array([-180,-180,-400])


        self.MIN_LIMIT = [-200,-200,-600]
        self.MAX_LIMIT = [300,300,-350]
        self.MAX_RADIUS = 380

        self.travel_velocity = 1400
        self.pipe_child.send(("MOVE_ROBOT", self.home_point, self.travel_velocity))
        self.track_log = {}

        self.pick_point_publisher = Cloud2DPublisher("PICK_POINTS")

    def get_robot_position(self):
        self.pipe_child.send(("GET_POSITION", None))
        msg = self.pipe_child.recv()
        if msg[0] == "POSITION":
            return msg[1]
        else:
            raise Exception("Invalid response from Delta process")


    def update_decision(self):
        pick_point = (0,0,self.hover_height)

        with tracker_lock:
            if len(self.tracks) > 0:
                pt = self.tracks[min(self.tracks.keys())].get_current_state()['position']
                pick_point = (pt[0], pt[1], self.hover_height)
            pass

        #pick_point = [-200, 0, self.hover_height]
        self.pipe_child.send(("MOVE_ROBOT", pick_point, self.travel_velocity))
        self.pick_point_publisher.publish(np.array([pick_point]))

        time.sleep(2)

        with tracker_lock:
            if len(self.tracks) > 0:
                pt = self.tracks[min(self.tracks.keys())].get_current_state()['position']
                pick_point = (pt[0], pt[1], self.hover_height)
            pass

        #pick_point = np.array([200, 0, self.hover_height])
        self.pipe_child.send(("MOVE_ROBOT", pick_point, self.travel_velocity))
        self.pick_point_publisher.publish(np.array([pick_point]))
        time.sleep(2)




    def run(pipe_parent, pipe_child):
        decision = DecisionProcess(pipe_parent, pipe_child)

        def poll_tracks():
            while not end_event.is_set():
                time.sleep(0.01)
                with tracker_lock:
                    msg = pipe_parent.recv()
                    if msg is None:
                        break
                    if msg[0] == "EXIT":
                        break
                    elif msg[0] == "TRACK":
                        decision.tracks = msg[1]
        
        threading.Thread(target=poll_tracks).start()

        while True:

            start_time = time.time()
            decision.update_decision()
            elapsed_time = time.time() - start_time
            sleep_time = max(0, (1/30) - elapsed_time)
            time.sleep(sleep_time)
