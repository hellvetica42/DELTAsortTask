from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence

from cyclonedds.core import Listener, Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.util import duration

import numpy as np
import cv2
import base64

from DDS.DDSTypes import Cloud2D, B64Image 

def cloud2d_to_numpy(cloud):
    if not isinstance(cloud, Cloud2D):
        raise TypeError("Expected Cloud2D type")
    
    points = cloud.points
    return np.array([[p.x, p.y] for p in points], dtype=np.float64)


class DetectionsListener(Listener):
    def __init__(self, callback_fn):
        super().__init__()
        self.detections = np.array([])
        self.callback_fn = callback_fn

    def on_data_available(self, reader):
        samples = reader.read()
        if len(samples) == 0:
            return

        self.detections = cloud2d_to_numpy(samples[0])
        self.callback_fn(self.detections)

class TrackerListener(Listener):
    def __init__(self, callback_fn):
        super().__init__()
        self.tracks = np.array([])
        self.callback_fn = callback_fn

    def on_data_available(self, reader):
        samples = reader.read()
        if len(samples) == 0:
            return

        self.tracks = cloud2d_to_numpy(samples[0])
        self.callback_fn(self.tracks)

class PickPointListener(Listener):
    def __init__(self, callback_fn):
        super().__init__()
        self.pick_points = np.array([])
        self.callback_fn = callback_fn

    def on_data_available(self, reader):
        samples = reader.read()
        if len(samples) == 0:
            return

        self.pick_points = cloud2d_to_numpy(samples[0])
        self.callback_fn(self.pick_points)

class TargetPointsListener(Listener):
    def __init__(self, callback_fn):
        super().__init__()
        self.target_points = np.array([])
        self.callback_fn = callback_fn

    def on_data_available(self, reader):
        samples = reader.read()
        if len(samples) == 0:
            return

        self.target_points = cloud2d_to_numpy(samples[0])
        self.callback_fn(self.target_points)

class DeltaPositionListener(Listener):
    def __init__(self, callback_fn):
        super().__init__()
        self.target_points = np.array([])
        self.callback_fn = callback_fn

    def on_data_available(self, reader):
        samples = reader.read()
        if len(samples) == 0:
            return

        self.target_points = cloud2d_to_numpy(samples[0])
        self.callback_fn(self.target_points)

class B64ImageListener(Listener):
    def __init__(self, callback_fn):
        super().__init__()
        self.callback_fn = callback_fn

    def on_data_available(self, reader):
        samples = reader.read()
        if len(samples) == 0:
            return

        b64_img = samples[0].img
        img_bytes = base64.b64decode(b64_img)
        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
        image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        self.callback_fn(image)


    
class GUIReceiver:
    def __init__(self, 
        on_detections_update,
        on_tracks_update,
        on_pick_points_update,
        on_target_points_update,
        on_delta_position_update,
        on_image_update,
    ):
        self.participant = DomainParticipant()
        self.detections_topic = Topic(self.participant, "DETECTIONS", Cloud2D)
        self.tracker_topic = Topic(self.participant, "TRACKS", Cloud2D)
        self.pick_points_topic = Topic(self.participant, "PICK_POINTS", Cloud2D)
        self.target_points_topic = Topic(self.participant, "TARGET_POINTS", Cloud2D)
        self.delta_position_topic = Topic(self.participant, "DELTA_POSITION", Cloud2D)
        self.b64_image_topic = Topic(self.participant, "IMG", B64Image)

        self.detections_listener = DetectionsListener(on_detections_update)
        self.tracker_listener = TrackerListener(on_tracks_update)
        self.pick_points_listener = PickPointListener(on_pick_points_update)
        self.target_points_listener = TargetPointsListener(on_target_points_update)
        self.delta_position_listener = DeltaPositionListener(on_delta_position_update)
        self.b64_image_listener = B64ImageListener(on_image_update)

        qos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(1),
            Policy.Deadline(duration(seconds=2))
        )

        self.detections_reader = DataReader(self.participant, self.detections_topic, qos=qos, listener=self.detections_listener)
        self.tracker_reader = DataReader(self.participant, self.tracker_topic, qos=qos, listener=self.tracker_listener)
        self.pick_points_reader = DataReader(self.participant, self.pick_points_topic, qos=qos, listener=self.pick_points_listener)
        self.target_points_reader = DataReader(self.participant, self.target_points_topic, qos=qos, listener=self.target_points_listener)
        self.delta_position_reader = DataReader(self.participant, self.delta_position_topic, qos=qos, listener=self.delta_position_listener)
        self.b64_image_reader = DataReader(self.participant, self.b64_image_topic, qos=qos, listener=self.b64_image_listener)