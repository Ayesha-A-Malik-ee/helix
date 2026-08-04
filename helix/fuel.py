from __future__ import annotations
from dataclasses import dataclass


@dataclass
class FuelState:
    fraction: float = 0.83
    capacity_kg: float = 400.0
    base_burn_kg_per_hr: float = 90.0
    speed_burn_coeff: float = 0.35


def fuel_flow_kg_per_hr(fuel: FuelState, airspeed_kmh: float) -> float:
    return fuel.base_burn_kg_per_hr + fuel.speed_burn_coeff * airspeed_kmh


def step_fuel(fuel: FuelState, airspeed_kmh: float, dt_seconds: float) -> None:
    ff = fuel_flow_kg_per_hr(fuel, airspeed_kmh)
    burned_kg = ff * (dt_seconds / 3600.0)
    fuel.fraction = max(0.0, fuel.fraction - burned_kg / fuel.capacity_kg)


def estimated_endurance_seconds(fuel: FuelState, airspeed_kmh: float) -> float:
    ff = fuel_flow_kg_per_hr(fuel, airspeed_kmh)
    if ff <= 0:
        return float("inf")
    remaining_kg = fuel.fraction * fuel.capacity_kg
    return (remaining_kg / ff) * 3600.0
