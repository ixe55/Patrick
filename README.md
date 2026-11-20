# Patrick

Patrick is a 2-DOF ball-on-plate balancing platform. A camera tracks the ball,
a controller works out the plate tilt needed to hold it in place (or move it
along a path), and servos tilt the plate to match.

Built with support from the Sears think[box] Embedded AI Microgrant at Case
Western Reserve University.

## Repo layout

- `sim/` – Julia simulation: plate/ball dynamics, LQR design, closed-loop tests
- `vision/` – ball detection and state estimation *(coming soon)*
- `hardware/` – Raspberry Pi 5 control loop, servo drivers, mechanical/electrical design *(coming soon)*

## Status

Simulation and controller design in progress.
