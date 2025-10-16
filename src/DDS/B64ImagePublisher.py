from cyclonedds.core import Qos, Policy
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.util import duration
from cyclonedds.domain import DomainParticipant

from DDS.DDSTypes import B64Image

import cv2
import base64
import time

class B64ImagePublisher:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.participant = DomainParticipant()

        self.topic = Topic(self.participant, self.topic_name, B64Image)

        qos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(1),
            Policy.Deadline(duration(seconds=2))
        )

        self.writer = DataWriter(self.participant, self.topic, qos=qos)
        self.timer = time.time()
        self.rate_limit = 15 #fps

    def publish(self, np_img):
        if time.time() - self.timer > 1/self.rate_limit:
            self.timer = time.time()
            success, encoded_image = cv2.imencode(".jpg", np_img)
            if not success:
                raise ValueError("Image encoding failed")

            b64_str = base64.b64encode(encoded_image.tobytes()).decode('utf-8')
            img = B64Image(img=b64_str)
            self.writer.write(img)
