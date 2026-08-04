from __future__ import annotations
from dataclasses import dataclass
import random

from .kinematics import Position2D


@dataclass
class SensorConfidence:
    gps: float = 98.0
    ins: float = 95.0
    compass: float = 100.0
    camera: float = 88.0
    gps_lost: bool = False


def step_sensor_confidence(sc: SensorConfidence, rng: random.Random) -> None:
    if not sc.gps_lost and rng.random() < 0.0015:
        sc.gps_lost = True
    elif sc.gps_lost and rng.random() < 0.01:
        sc.gps_lost = False

    if sc.gps_lost:
        sc.gps = max(5.0, min(40.0, sc.gps + rng.uniform(-8, -2)))
        sc.ins = max(80.0, min(99.0, sc.ins + rng.uniform(0, 2)))
    else:
        sc.gps = max(90.0, min(100.0, sc.gps + rng.uniform(-1, 1.2)))
        sc.ins = max(90.0, min(99.0, sc.ins + rng.uniform(-1, 1)))

    sc.compass = max(95.0, min(100.0, sc.compass + rng.uniform(-0.5, 0.5)))
    sc.camera = max(70.0, min(95.0, sc.camera + rng.uniform(-1.5, 1.5)))


def fuse_position(gps_pos: Position2D, ins_pos: Position2D, sc: SensorConfidence) -> Position2D:
    total = sc.gps + sc.ins
    if total <= 0:
        return ins_pos
    w_gps = sc.gps / total
    w_ins = sc.ins / total
    return Position2D(
        x=w_gps * gps_pos.x + w_ins * ins_pos.x,
        y=w_gps * gps_pos.y + w_ins * ins_pos.y,
    )