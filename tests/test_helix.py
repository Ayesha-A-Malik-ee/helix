import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from helix.fuel import FuelState, fuel_flow_kg_per_hr, step_fuel
from helix.kinematics import (
    AttitudeState,
    Position2D,
    integrate_position,
    shortest_angle_diff,
    update_heading,
    wrap_deg,
)
from helix.navigation import (
    Waypoint,
    bearing_to,
    cross_track_error_m,
    distance_m,
    eta_seconds,
)
from helix.simulator import HelixSimulator
from helix.wind import Wind, ground_speed, wind_correction_angle


class TestKinematics(unittest.TestCase):
    def test_wrap_deg(self):
        self.assertAlmostEqual(wrap_deg(370), 10)
        self.assertAlmostEqual(wrap_deg(-10), 350)

    def test_shortest_angle_diff(self):
        self.assertAlmostEqual(shortest_angle_diff(10, 350), 20)

    def test_heading_converges(self):
        att = AttitudeState(heading_deg=0, target_heading_deg=90, yaw_rate_gain=1.0)
        for _ in range(500):
            update_heading(att, dt=0.1)
        self.assertAlmostEqual(att.heading_deg, 90, delta=1.0)

    def test_position_integration_moves_north_at_heading_zero(self):
        pos = Position2D(0, 0)
        integrate_position(pos, heading_deg=0, ground_speed_kmh=36, dt=10)
        self.assertAlmostEqual(pos.x, 0, delta=1e-6)
        self.assertAlmostEqual(pos.y, 100, delta=1e-6)

    def test_position_integration_moves_east_at_heading_90(self):
        pos = Position2D(0, 0)
        integrate_position(pos, heading_deg=90, ground_speed_kmh=36, dt=10)
        self.assertAlmostEqual(pos.x, 100, delta=1e-6)
        self.assertAlmostEqual(pos.y, 0, delta=1e-6)


class TestWind(unittest.TestCase):
    def test_headwind_reduces_ground_speed(self):
        wind = Wind(direction_deg=0, speed_kmh=20)
        gs = ground_speed(true_airspeed_kmh=100, wind=wind, desired_track_deg=0)
        self.assertLess(gs, 100)

    def test_tailwind_increases_ground_speed(self):
        wind = Wind(direction_deg=180, speed_kmh=20)
        gs = ground_speed(true_airspeed_kmh=100, wind=wind, desired_track_deg=0)
        self.assertGreater(gs, 100)

    def test_crosswind_needs_correction_angle(self):
        wind = Wind(direction_deg=90, speed_kmh=20)
        wca = wind_correction_angle(true_airspeed_kmh=100, wind=wind, desired_track_deg=0)
        self.assertGreater(abs(wca), 0)


class TestNavigation(unittest.TestCase):
    def test_distance_and_bearing(self):
        a = Position2D(0, 0)
        b = Waypoint(0, 100, "X")
        self.assertAlmostEqual(distance_m(a, b), 100)
        self.assertAlmostEqual(bearing_to(a, b), 0)

    def test_bearing_east(self):
        a = Position2D(0, 0)
        b = Waypoint(100, 0, "X")
        self.assertAlmostEqual(bearing_to(a, b), 90)

    def test_cross_track_error_zero_on_line(self):
        wp_from = Waypoint(0, 0, "A")
        wp_to = Waypoint(0, 100, "B")
        pos = Position2D(0, 50)
        self.assertAlmostEqual(cross_track_error_m(pos, wp_from, wp_to), 0, delta=1e-6)

    def test_cross_track_error_offset(self):
        wp_from = Waypoint(0, 0, "A")
        wp_to = Waypoint(0, 100, "B")
        pos = Position2D(10, 50)
        self.assertAlmostEqual(cross_track_error_m(pos, wp_from, wp_to), 10, delta=1e-6)

    def test_eta(self):
        self.assertAlmostEqual(eta_seconds(1000, 36), 100, delta=1e-6)


class TestFuel(unittest.TestCase):
    def test_fuel_depletes(self):
        fuel = FuelState(fraction=1.0)
        start = fuel.fraction
        step_fuel(fuel, airspeed_kmh=150, dt_seconds=3600)
        self.assertLess(fuel.fraction, start)

    def test_higher_speed_burns_more_fuel(self):
        slow = fuel_flow_kg_per_hr(FuelState(), airspeed_kmh=100)
        fast = fuel_flow_kg_per_hr(FuelState(), airspeed_kmh=200)
        self.assertGreater(fast, slow)


class TestSimulatorIntegration(unittest.TestCase):
    def test_runs_without_error(self):
        sim = HelixSimulator(seed=1)
        for _ in range(200):
            tel = sim.step(0.5)
        self.assertGreaterEqual(tel.t, 100)
        self.assertTrue(0.0 <= tel.fuel_fraction <= 1.0)
        self.assertTrue(0.0 <= tel.sensors.gps <= 100.0)


if __name__ == "__main__":
    unittest.main()