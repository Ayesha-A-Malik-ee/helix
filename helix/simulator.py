from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional
import math
import random

from .kinematics import AttitudeState, Position2D, update_heading, update_attitude_oscillation, integrate_position, altitude_wander
from .wind import Wind, step_wind, wind_drift_component
from .navigation import Waypoint, RouteManager, distance_m, bearing_to, cross_track_error_m, eta_seconds
from .fuel import FuelState, step_fuel, estimated_endurance_seconds
from .sensors import SensorConfidence, step_sensor_confidence, fuse_position
from .radar import RadarContact, maybe_spawn_contact, step_contact, bearing_relative
from .mission import MissionState
from .alerts import AlertLog


@dataclass
class Telemetry:
    t: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    altitude_m: float
    airspeed_kmh: float
    ground_speed_kmh: float
    fuel_fraction: float
    endurance_seconds: float
    position: Position2D
    current_waypoint_label: str
    distance_to_wp_m: float
    eta_seconds: float
    cross_track_error_m: float
    wind_dir_deg: float
    wind_speed_kmh: float
    sensors: SensorConfidence
    mission_phase: str
    radar_contact: Optional[RadarContact]


class HelixSimulator:
    def __init__(self, seed: Optional[int] = None):
        self.rng = random.Random(seed)
        self.t = 0.0

        self.attitude = AttitudeState()
        self.true_pos = Position2D(x=0.0, y=0.0)
        self.airspeed_kmh = 178.0

        self.wind = Wind()
        self.fuel = FuelState()
        self.sensors = SensorConfidence()
        self.mission = MissionState()
        self.alerts = AlertLog()

        self.route = RouteManager(waypoints=[
            Waypoint(600, 1400, "A"),
            Waypoint(1800, 2200, "B"),
            Waypoint(2600, 900, "C"),
            Waypoint(1400, -300, "D"),
        ])

        self.radar_contact: Optional[RadarContact] = None
        self.trail: List[Position2D] = []

        self.alerts.push("System initialized.", "info")
        self.alerts.push("GPS accuracy improved.", "info")

    def step(self, dt: float) -> Telemetry:
        self.t += dt

        target = self.route.current
        desired_bearing = bearing_to(self.true_pos, target)
        if self.rng.random() < 0.004:
            self.attitude.target_heading_deg = (desired_bearing + self.rng.uniform(-25, 25)) % 360
        else:
            self.attitude.target_heading_deg = desired_bearing
        update_heading(self.attitude, dt)
        update_attitude_oscillation(self.attitude, self.t)

        step_wind(self.wind, self.t)
        drift = wind_drift_component(self.wind, self.attitude.heading_deg)
        ground_speed_kmh = max(0.0, self.airspeed_kmh - drift)

        integrate_position(self.true_pos, self.attitude.heading_deg, ground_speed_kmh, dt)
        self.trail.append(Position2D(self.true_pos.x, self.true_pos.y))
        if len(self.trail) > 500:
            self.trail.pop(0)

        altitude_m = altitude_wander(452.0, self.t)

        captured = self.route.update(self.true_pos)
        if captured:
            self.alerts.push(f"Waypoint {self.route.previous.label} reached.", "info")
        dist = distance_m(self.true_pos, self.route.current)
        eta = eta_seconds(dist, ground_speed_kmh)
        xte = cross_track_error_m(self.true_pos, self.route.previous, self.route.current)

        step_fuel(self.fuel, self.airspeed_kmh, dt)
        endurance = estimated_endurance_seconds(self.fuel, self.airspeed_kmh)
        if self.fuel.fraction < 0.15 and self.rng.random() < 0.01:
            self.alerts.push("Low fuel.", "crit")

        was_lost = self.sensors.gps_lost
        step_sensor_confidence(self.sensors, self.rng)
        if self.sensors.gps_lost and not was_lost:
            self.alerts.push("GPS signal degraded. INS taking over.", "warn")
        elif was_lost and not self.sensors.gps_lost:
            self.alerts.push("GPS accuracy improved.", "info")

        if self.radar_contact is None:
            new_contact = maybe_spawn_contact(self.rng)
            if new_contact:
                self.radar_contact = new_contact
                level = "crit" if new_contact.level == "HIGH" else "warn"
                self.alerts.push(
                    f"{new_contact.name} detected, bearing "
                    f"{new_contact.bearing_deg:.0f} deg.", level
                )
        else:
            if step_contact(self.radar_contact, dt):
                self.alerts.push(f"{self.radar_contact.name} contact lost.", "info")
                self.radar_contact = None

        if self.rng.random() < 0.0012:
            self.alerts.push("Crosswind increasing.", "warn")
        if self.rng.random() < 0.0008:
            self.alerts.push("Communications degraded.", "warn")

        return Telemetry(
            t=self.t,
            heading_deg=self.attitude.heading_deg,
            pitch_deg=self.attitude.pitch_deg,
            roll_deg=self.attitude.roll_deg,
            altitude_m=altitude_m,
            airspeed_kmh=self.airspeed_kmh,
            ground_speed_kmh=ground_speed_kmh,
            fuel_fraction=self.fuel.fraction,
            endurance_seconds=endurance,
            position=Position2D(self.true_pos.x, self.true_pos.y),
            current_waypoint_label=self.route.current.label,
            distance_to_wp_m=dist,
            eta_seconds=eta,
            cross_track_error_m=xte,
            wind_dir_deg=self.wind.direction_deg,
            wind_speed_kmh=self.wind.speed_kmh,
            sensors=self.sensors,
            mission_phase=self.mission.phase,
            radar_contact=self.radar_contact,
        )