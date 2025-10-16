from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence

@dataclass
class Point2D(IdlStruct):
    x: float
    y: float

@dataclass
class Cloud2D(IdlStruct):
    points: sequence[Point2D]

@dataclass 
class B64Image(IdlStruct):
    img: str

@dataclass
class RobotCommand(IdlStruct):
    x: float
    y: float
    z: float
    velocity: float

@dataclass
class BBOX2D(IdlStruct):
    x: float
    y: float
    w: float
    h: float
    cls: int
    conf: float

@dataclass
class BBOX2DArray(IdlStruct):
    bboxes: sequence[BBOX2D]