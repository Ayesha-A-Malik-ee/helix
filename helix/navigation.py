from __future__ import annotations
from dataclasses import dataclass
from typing import List
import math

from .kinematics import Position2D


@dataclass
class Waypoint:
    x: float
    y: float
    label: str = ""


def distance_m(a: Position2D, b) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def bearing_to(a: Position2D, b) -> float:
    dx = b.x - a.x
    dy = b.y - a.y
    return math.degrees(math.atan2(dx, dy)) % 360.0


def cross_track_error_m(pos: Position2D, wp_from: Waypoint, wp_to: Waypoint) -> float:
    rx, ry = wp_to.x - wp_from.x, wp_to.y - wp_from.y
    px, py = pos.x - wp_from.x, pos.y - wp_from.y
    route_len = math.hypot(rx, ry)
    if route_len == 0:
        return 0.0
    cross = abs(rx * py - ry * px)
    return cross / route_len


def eta_seconds(distance_m_: float, ground_speed_kmh: float) -> float:
    speed_ms = ground_speed_kmh / 3.6
    if speed_ms <= 0.01:
        return float("inf")
    return distance_m_ / speed_ms


@dataclass
class RouteManager:
    waypoints: List[Waypoint]
    current_idx: int = 0
    capture_radius_m: float = 60.0

    @property
    def current(self) -> Waypoint:
        return self.waypoints[self.current_idx]

    @property
    def previous(self) -> Waypoint:
        prev_idx = (self.current_idx - 1) % len(self.waypoints)
        return self.waypoints[prev_idx]

    def update(self, pos: Position2D) -> bool:
        if distance_m(pos, self.current) <= self.capture_radius_m:
            self.current_idx = (self.current_idx + 1) % len(self.waypoints)
            return True
        return False
