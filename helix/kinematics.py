from __future__ import annotations
from dataclasses import dataclass
import math


def wrap_deg(angle: float) -> float:
    return angle % 360.0


def shortest_angle_diff(target: float, current: float) -> float:
    diff = (target - current + 540.0) % 360.0 - 180.0
    return diff


@dataclass
class AttitudeState:
    heading_deg: float = 214.0
    target_heading_deg: float = 214.0
    pitch_deg: float = 12.0
    roll_deg: float = -18.0
    yaw_rate_gain: float = 0.6


def update_heading(att: AttitudeState, dt: float) -> None:
    diff = shortest_angle_diff(att.target_heading_deg, att.heading_deg)
    att.heading_deg = wrap_deg(att.heading_deg + diff * att.yaw_rate_gain * dt)


def update_attitude_oscillation(att: AttitudeState, t: float) -> None:
    att.pitch_deg = 12.0 + 6.0 * math.sin(0.5 * t)
    att.roll_deg = -18.0 + 10.0 * math.sin(0.33 * t)


@dataclass
class Position2D:
    x: float = 0.0
    y: float = 0.0


def integrate_position(pos: Position2D, heading_deg: float, ground_speed_kmh: float, dt: float) -> None:
    v_ms = ground_speed_kmh / 3.6
    heading_rad = math.radians(heading_deg)
    pos.x += v_ms * math.sin(heading_rad) * dt
    pos.y += v_ms * math.cos(heading_rad) * dt


def altitude_wander(base_alt_m: float, t: float, amplitude_m: float = 40.0) -> float:
    return base_alt_m + amplitude_m * math.sin(0.07 * t)