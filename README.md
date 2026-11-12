# ROS 2 Robot Fleet with a Safety Layer

A multi-robot ROS 2 / Nav2 fleet in simulation, guarded by a camera-based safety layer: robots stop when a person enters the protective zone and slow down when the camera sees something the detector was not trained on. The velocity gate is written in C++ and fails safe.

## Why it matters

Mobile robots are moving into Italian warehouses, airports and hospitals, and the EU Machinery Regulation (from 20 January 2027) adds requirements for machine-learning safety functions. Integrators need engineers who can combine ROS 2 navigation, perception and conservative safety logic.

**Relevant employers in Italy:** Comau, Alba Robot, Cyberwave, Oversonic, Teoresi, Fogliani

## What it does

- ROS 2 Jazzy + Nav2 + Gazebo in Docker (native arm64 on Apple Silicon); view in Foxglove
- Three-robot fleet with waypoint missions
- `safety_monitor` (Python): person detection, protective-zone check, OOD energy score
- `speed_gate` (C++, rclcpp): scales Nav2 velocity commands; stops on a stale or missing safety signal
- Hysteresis state machine so flickering detections cannot cause lurching
- Metrics: stop latency, missions completed, OOD detection AUROC

## Results

| Metric | Value |
|---|---|
| Single robot, 5-goal loop through the gate | **5/5 reached, 44.4 s sim time** (week 1) |
| Three-robot fleet, 14 goals in parallel | **14/14 reached, makespan 58.8 s sim, 14.3 goals/min** (week 2) |
| Closest robot-to-robot approach / near misses (< 0.5 m) | **0.49 m / 1** (week 2) |
| Stop latency after a person enters the zone (ms) | TBD |
| False stops per hour | TBD |
| OOD detection AUROC | TBD |

*Filled in as the roadmap progresses.*

## Quickstart

Requires [uv](https://docs.astral.sh/uv/) and Docker Desktop.

```bash
make setup              # Python ML side
make test
make docker-build       # ROS 2 image (first build downloads several GB)
make sim                # simulation + Foxglove on ws://localhost:8765
make missions           # headless 5-goal run, JSON report in results/
make sim-fleet          # three robots + Foxglove
make fleet              # headless three-robot run, fleet report in results/
```

### Viewing in Foxglove

With `make sim` running, open [Foxglove](https://foxglove.dev/download) and choose
**Open connection → Foxglove WebSocket → `ws://localhost:8765`**. Add a **3D** panel and
enable `/map`, `/scan`, `/tf`, `/robot_description` and `/plan`. Send a goal with the panel's
**Publish → Pose** tool on `/goal_pose`, or stop `make sim` and run `make missions` instead (the
bridge is served there too) to watch the five-goal loop. Add a **Plot** panel on `/safety/speed_scale.data`
to watch the safety scale.

The bridge (foxglove_bridge 3.x) speaks only the `foxglove.sdk.v1` protocol, so use a current
Foxglove release. One simulated robot uses about 1.1 GB of RAM and 1.8 CPU cores.

## Roadmap

See [docs/ROADMAP.md](docs/ROADMAP.md).

## In italiano

Flotta di robot mobili in ROS 2 / Nav2 con un livello di sicurezza basato su visione: i robot si fermano quando una persona entra nella zona protetta e rallentano quando la telecamera vede qualcosa di sconosciuto. Il gate di velocità è scritto in C++ ed è fail-safe. Collegato al nuovo Regolamento Macchine UE.

## Licence

Code: MIT, see [LICENSE](LICENSE). Datasets and third-party assets keep their own licences;
see [data/README.md](data/README.md).
