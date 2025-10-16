import time
import numpy as np
import random

import sys
sys.path.append(".")
from DDS.Cloud2DPublisher import Cloud2DPublisher
from DDS.B64ImagePublisher import B64ImagePublisher

class DummyDetectorProcess:
    def __init__(self, pipe_parent, pipe_child):
        self.pipe_parent = pipe_parent
        self.pipe_child = pipe_child

        self.min_start_x, self.max_start_x = 0, 848
        self.start_y = 0
        self.max_y = 480

        self.move_vector = np.array([0, 150])

        self.timer = time.time()
        self.fps = 8

        self.detections = {}
        self.detection_id = 0

        self.cloud_publisher = Cloud2DPublisher("DETECTIONS")
        self.image_publisher = B64ImagePublisher("IMG")

    def create_new_detection(self):
        x = float(np.random.randint(self.min_start_x, self.max_start_x))
        y = float(self.start_y)
        self.detections[self.detection_id] = np.array([x, y])
        self.detection_id += 1

    def update_detections(self):
        to_remove_i = []
        for track_id in self.detections:
            self.detections[track_id] += self.move_vector * (1/self.fps)

            if self.detections[track_id][1] > self.max_y:
                to_remove_i.append(track_id)

        for i in to_remove_i:
            del self.detections[i]

        self.cloud_publisher.publish(self.get_detection_centers())

    def get_detection_centers(self):
        return np.array(list(self.detections.values()))


    def run(pipe_parent, pipe_child):
        detector = DummyDetectorProcess(pipe_parent, pipe_child)

        while True:
            if pipe_parent.poll():
                msg = pipe_parent.recv()
                if msg[0] == "EXIT":
                    break

            if time.time() - detector.timer > 1.0 / detector.fps:
                detector.timer = time.time()
                #detector.image_publisher.publish((np.random.random((848, 480))*255).astype(int))

                # add new detection
                if random.random() < 0.1 and len(detector.detections) < 3:
                    detector.create_new_detection()

                detector.update_detections()
                msg = {
                    "detection_centers": np.array(list(detector.detections.values())),
                    "track_ids": list(detector.detections.keys()),
                    "timestamp": time.time()
                }
                pipe_child.send(("DETECTIONS", msg))

            

            