from cyclonedds.core import Qos, Policy
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.util import duration
from cyclonedds.domain import DomainParticipant

from DDS.DDSTypes import Cloud2D, Point2D

class Cloud2DPublisher:
    def __init__(self, topic_name):

        self.topic_name = topic_name
        self.participant = DomainParticipant()

        self.topic = Topic(self.participant, self.topic_name, Cloud2D)

        qos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(1),
            Policy.Deadline(duration(seconds=2))
        )

        self.writer = DataWriter(self.participant, self.topic, qos=qos)

    def publish(self, points):
        if points.shape[0] == 0:
            return
        pts = []
        for p in points:
            pts.append(Point2D(x=float(p[0]), y=float(p[1])))

        cloud = Cloud2D(points=pts)
        self.writer.write(cloud)


