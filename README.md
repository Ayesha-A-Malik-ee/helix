# Helix — Mission Systems Simulator (Python logic)

Helix is a simulated helicopter **mission systems console** — the kind of
multi-function display (MFD) an avionics/HMI engineer might prototype: a
moving map, EO/IR camera feed, artificial horizon, radar warning indicator,
fuel and sensor-fusion panels, and a mission timeline.

This repository is the **simulation logic**, written in plain Python with
no external dependencies, that drives that console. It mirrors the logic
used in the browser (HTML/Canvas/JS) version of the display, so it can be
read, tested, and reused independently of any UI — as a CLI telemetry
stream, a backend for a different frontend, or just a reference for the
math.

![Helix demo](helix.gif)

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for
details.

> **Scope note:** this is a mission-display / HMI simulation, not a real
> flight-dynamics or targeting system. Attitude, position, and sensor
> behavior are lightweight kinematic and statistical approximations meant
> to *look and behave* like a real avionics system, not to fly a real
> aircraft. Radar contacts are generic ("Emitter A") rather than modeling
> any specific weapon or sensor system.

---

## Quick start

```bash
git clone <this repo>
cd helix-python
python3 main.py
```

That starts a live console readout, ticking twice a second by default.
Press `Ctrl+C` to stop.

Useful flags:

```bash
python3 main.py --ticks 200            # run 200 steps then exit (no live wait)
python3 main.py --dt 1.0 --rate 10     # 1 simulated second per tick, 10 ticks/sec wall clock
python3 main.py --seed 42              # deterministic run (same random events every time)
python3 main.py --json telemetry.jsonl # also write one JSON object per line
python3 main.py --quiet --json out.jsonl --ticks 1000   # headless data logging
```

Run the test suite:

```bash
python3 -m unittest discover tests -v
# or, if you have pytest installed:
python3 -m pytest tests/ -v
```

---

## Project layout

```
helix-python/
├── main.py                 # CLI entry point: runs the sim, prints/exports telemetry
├── requirements.txt
├── helix/
│   ├── kinematics.py        # heading control, attitude, position integration
│   ├── wind.py               # wind triangle (ground speed, wind correction angle)
│   ├── navigation.py         # waypoints, distance/bearing, cross-track error, ETA
│   ├── fuel.py                # fuel flow & endurance model
│   ├── sensors.py             # sensor confidence + GPS/INS fusion
│   ├── radar.py                # radar warning contact generation & bearing
│   ├── mission.py              # mission phase state machine
│   ├── alerts.py                # ECAM-style timestamped alert log
│   └── simulator.py             # HelixSimulator — wires every module together
└── tests/
    └── test_helix.py             # unit tests for every physics/logic function
```

Each module is self-contained and independently testable — `simulator.py`
is the only file that imports the others and steps them together each
frame, the same separation of concerns as the individual panels in the
console UI.

---

## Architecture: the simulation step

Every call to `HelixSimulator.step(dt)` performs, in order:

1. **Heading control** — steer toward the current waypoint's bearing (with
   occasional random re-targeting for visual interest).
2. **Wind update** — slowly wander the ambient wind direction/speed.
3. **Ground speed** — apply the wind's along-heading component to airspeed.
4. **Position integration** — dead-reckon the new position from heading and
   ground speed.
5. **Altitude** — wander around a base altitude (simulated altitude-hold).
6. **Navigation** — recompute distance, ETA, and cross-track error to the
   active waypoint; advance to the next waypoint if within capture radius.
7. **Fuel** — burn fuel proportional to airspeed; recompute endurance.
8. **Sensors** — random-walk each sensor's confidence; simulate GPS dropout
   and INS takeover.
9. **Radar** — spawn/age simulated "emitter" contacts.
10. **Alerts** — log any of the above as a timestamped message.

It returns a `Telemetry` snapshot — a plain dataclass with everything a UI
panel would need to render that frame.

---

## Physics & math used

Everything below is implemented in the module named in parentheses, with
the same explanation as an inline docstring next to the code.

### 1. Heading control — first-order lag turn model (`kinematics.py`)

```
dHeading/dt = k * (target_heading - heading)
```

The aircraft turns toward a commanded heading at a rate proportional to
how far off it currently is — the same style of proportional control used
in a simple autopilot heading-hold mode. It approaches the target
asymptotically rather than snapping to it. Heading being circular (0° = 360°)
is handled with a **shortest-signed-angle** helper so a turn from 350° to
10° is computed as +20°, not −340°.

### 2. Attitude oscillation — simple harmonic motion (`kinematics.py`)

```
pitch(t) = pitch0 + A_p * sin(w_p * t)
roll(t)  = roll0  + A_r * sin(w_r * t)
```

Approximates the small constant pitch/roll corrections a helicopter makes
around a trim attitude in forward flight, using independent low-frequency
sinusoids rather than a full 6-degree-of-freedom equations-of-motion solve.

### 3. Position integration — dead reckoning (`kinematics.py`)

```
v_x = V * sin(heading)     (East component)
v_y = V * cos(heading)     (North component)
x(t+dt) = x(t) + v_x * dt
y(t+dt) = y(t) + v_y * dt
```

This is exactly how an Inertial Navigation System propagates position
between GPS fixes: integrate a velocity vector forward in time. Because
heading is measured clockwise from North (a compass bearing), `sin` gives
the East component and `cos` gives the North component — the axes are
swapped relative to standard math-convention (`x=cos, y=sin`) angles.

### 4. Wind triangle (`wind.py`)

Standard aviation navigation math for how wind affects an aircraft's track:

```
WCA = asin( (Wind_speed * sin(wind_dir - desired_track)) / TAS )
GS  = TAS * cos(WCA) - Wind_speed * cos(wind_dir - desired_track)
```

- **WCA (Wind Correction Angle):** how far off the desired track the nose
  must point to cancel crosswind drift, derived from the perpendicular
  ("crosswind") component of the wind vector.
- **GS (Ground Speed):** the along-track component of true airspeed after
  correcting for WCA, plus/minus the along-track component of the wind
  itself (a headwind subtracts, a tailwind adds) — the standard
  law-of-cosines solution to the wind triangle.
- `wind.direction_deg` uses the **meteorological convention**: the
  direction the wind is blowing *from*.

### 5. Navigation geometry (`navigation.py`)

Distance and bearing between two points, using plane (Euclidean) geometry
rather than great-circle navigation — the correct simplification for a
local mission area of a few kilometers where Earth curvature is
negligible:

```
distance = hypot(dx, dy)
bearing  = atan2(dx, dy) mod 360        (compass convention, clockwise from North)
```

**Cross-track error (XTE)** — perpendicular distance from the current
position to the planned route line between the previous and next
waypoint, via the standard 2D vector-projection / cross-product formula:

```
r = wp_to - wp_from
p = pos - wp_from
XTE = | r_x * p_y - r_y * p_x | / |r|
```

**ETA:**

```
ETA = distance / ground_speed
```

### 6. Fuel model (`fuel.py`)

A linear fuel-flow approximation, standard in early-stage performance
planning before consulting a full engine power chart:

```
FuelFlow(kg/hr) = base_burn + k * airspeed
burned_kg = FuelFlow * (dt / 3600)
endurance = (fraction * capacity_kg) / FuelFlow   [hours]
```

Burn rate rises roughly linearly with airspeed because engine power (and
thus fuel flow) scales with drag, which is well approximated as linear
over a limited cruise speed band.

### 7. Sensor fusion — complementary filter (`sensors.py`)

Blends a GPS position estimate (noisy but absolute) with an INS estimate
(smooth but drifting) weighted by each source's confidence:

```
w_gps = conf_gps / (conf_gps + conf_ins)
w_ins = conf_ins / (conf_gps + conf_ins)
fused = w_gps * gps_pos + w_ins * ins_pos
```

This is the same family of technique used in real avionics (a
confidence-weighted blend of sensors), simplified from a full Kalman
filter by skipping covariance propagation — appropriate for a mission
display rather than a certified navigation system. When GPS confidence
collapses (simulated signal loss), `w_gps → 0` and the estimate falls back
smoothly to INS-only, avoiding a hard-cutover glitch.

Sensor confidence itself evolves as a **bounded random walk** — a
stochastic process, not a physical sensor-noise model — chosen because it
produces the same qualitative behavior a real health-monitor page shows:
mostly-steady confidence with occasional dips.

### 8. Radar contact bearing (`radar.py`)

```
relative_bearing = (absolute_bearing - own_heading) mod 360
```

Converts a contact's true (absolute) bearing to a heading-relative bearing,
as shown on a heading-up radar display. Contact spawn timing uses a
**Bernoulli trial per simulation step** (constant per-step probability of
an arrival), a discrete approximation of a Poisson process — the standard
way to model sporadic, memoryless events like radar detections.

### 9. Altitude wander (`kinematics.py`)

```
altitude(t) = base_altitude + amplitude * sin(0.07 * t)
```

A slow sinusoid around a base altitude, representing imperfect
altitude-hold rather than a perfectly flat line.

---

## Extending this

- **Swap dead reckoning for real lat/lon:** replace the local ENU
  `Position2D` with a proper geodesic model (e.g. haversine distance /
  bearing) if you want the mission area to span more than a few kilometers.
- **Replace the complementary filter with a Kalman filter:** `sensors.py`
  is the natural place to add covariance propagation if you want a more
  rigorous fusion model.
- **Feed a real frontend:** `main.py --json out.jsonl` streams one
  telemetry snapshot per line — pipe or tail that file into any UI (the
  browser console, a Grafana dashboard, etc.) instead of printing to the
  terminal.

---
