from __future__ import annotations
import argparse
import dataclasses
import json
import sys
import time

from helix.simulator import HelixSimulator, Telemetry


def telemetry_to_dict(tel: Telemetry) -> dict:
    return dataclasses.asdict(tel)


def format_line(tel: Telemetry) -> str:
    radar = "CLEAR"
    if tel.radar_contact:
        radar = f"{tel.radar_contact.name} ({tel.radar_contact.level}) brg {tel.radar_contact.bearing_deg:5.1f}"
    
    return (
        f"t={tel.t:7.1f}s  HDG={tel.heading_deg:6.1f}  "
        f"ALT={tel.altitude_m:6.1f}m  GS={tel.ground_speed_kmh:6.1f}km/h  "
        f"FUEL={tel.fuel_fraction*100:5.1f}%  WP={tel.current_waypoint_label} "
        f"DIST={tel.distance_to_wp_m:7.1f}m  XTE={tel.cross_track_error_m:5.1f}m  "
        f"GPS={tel.sensors.gps:5.1f}%  RADAR={radar}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ticks", type=int, default=0)
    parser.add_argument("--dt", type=float, default=0.5)
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--json", type=str, default=None)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    sim = HelixSimulator(seed=args.seed)
    json_fh = open(args.json, "w") if args.json else None

    tick = 0
    try:
        while args.ticks == 0 or tick < args.ticks:
            tel = sim.step(args.dt)

            if not args.quiet:
                print(format_line(tel))

            if json_fh:
                json_fh.write(json.dumps(telemetry_to_dict(tel), default=str) + "\n")

            for alert in list(sim.alerts.items)[:1]:
                pass

            tick += 1
            if args.rate > 0:
                time.sleep(1.0 / args.rate)
                
    except KeyboardInterrupt:
        print("\nSimulation stopped.")
    finally:
        if json_fh:
            json_fh.close()


if __name__ == "__main__":
    sys.exit(main() or 0)