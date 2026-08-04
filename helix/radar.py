from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import math
import random

from .kinematics import Position2D


@dataclass
class RadarContact:
    name: str
    bearing_deg: float
    level: str
    ttl_seconds: float


def bearing_relative(ownship_heading_deg: float, absolute_bearing_deg: float) -> float:
    return (absolute_bearing_deg - ownship_heading_deg) % 360.0


def maybe_spawn_contact(rng: random.Random, spawn_probability: float = 0.003) -> Optional[RadarContact]:
    if rng.random() >= spawn_probability:
        return None
    letter = chr(ord("A") + rng.randint(0, 4))
    return RadarContact(
        name=f"EMITTER {letter}",
        bearing_deg=rng.uniform(0, 360),
        level="HIGH" if rng.random() < 0.4 else "LOW",
        ttl_seconds=rng.uniform(8, 18),
    )


def step_contact(contact: RadarContact, dt: float) -> bool:
    contact.ttl_seconds -= dt
    return contact.ttl_seconds <= 0