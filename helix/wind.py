from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass
class Wind:
    direction_deg: float = 260.0
    speed_kmh: float = 18.0


def wind_correction_angle(true_airspeed_kmh: float, wind: Wind, desired_track_deg: float) -> float:
    if true_airspeed_kmh <= 0:
        return 0.0
    angle_diff = math.radians(wind.direction_deg - desired_track_deg)
    ratio = (wind.speed_kmh * math.sin(angle_diff)) / true_airspeed_kmh
    ratio = max(-1.0, min(1.0, ratio))
    return math.degrees(math.asin(ratio))


def ground_speed(true_airspeed_kmh: float, wind: Wind, desired_track_deg: float) -> float:
    wca = wind_correction_angle(true_airspeed_kmh, wind, desired_track_deg)
    along_track_wind = -wind.speed_kmh * math.cos(math.radians(wind.direction_deg - desired_track_deg))
    return true_airspeed_kmh * math.cos(math.radians(wca)) + along_track_wind


def wind_drift_component(wind: Wind, heading_deg: float) -> float:
    return wind.speed_kmh * math.cos(math.radians(wind.direction_deg - heading_deg))


def step_wind(wind: Wind, t: float) -> None:
    wind.direction_deg = (wind.direction_deg + 0.3 * math.sin(0.15 * t)) % 360.0
    wind.speed_kmh = max(2.0, min(45.0, 18.0 + 6.0 * math.sin(0.25 * t)))