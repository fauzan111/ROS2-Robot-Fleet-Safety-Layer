# ROS 2 Robot Fleet with a Safety Layer

A multi-robot ROS 2 / Nav2 fleet in simulation, guarded by a safety layer: each robot's camera recognises people and slows the robot early, a 360° lidar near field stops it for anything unknown in its protective zone, and a degraded camera image (fog, darkness, dirt) slows it down. The velocity gate is written in C++ and fails safe. Every number below is measured against simulation ground truth.

## Why it matters

Mobile robots are moving into Italian warehouses, airports and hospitals, and the EU Machinery Regulation (from 20 January 2027) adds requirements for machine-learning safety functions. Integrators need engineers who can combine ROS 2 navigation, perception and conservative safety logic.

**Relevant employers in Italy:** Comau, Alba Robot, Cyberwave, Oversonic, Teoresi, Fogliani

## What it does

- ROS 2 Jazzy + Nav2 + Gazebo in Docker (native arm64 on Apple Silicon); view in Foxglove
- Three-robot fleet with waypoint missions
- `safety_monitor` (Python, one per robot): YOLOv4-tiny person detection (OpenCV DNN) + depth
  localisation, stop zone (1.0 m) and warning zone (2.0 m), lidar near field (returns not on
  the static map), out-of-distribution frame check; any sensor going quiet means STOP
- `speed_gate` (C++, rclcpp): scales Nav2 velocity commands; stops on a stale or missing safety signal
- Hysteresis state machine so flickering detections cannot cause lurching
- Ground-truth evaluation: scripted walking people + Gazebo poses -> stop latency, missed
  entries, false stops, recall by distance (`make eval-crossing`, `make eval-false-stops`)

## Results

| Metric | Value |
|---|---|
| Single robot, 5-goal loop through the gate | **5/5 reached, 44.4 s sim time** (week 1) |
| Three-robot fleet, 14 goals in parallel (2 runs) | **14/14** (makespan 58.8 s sim) and **13/14** (robot1 wedged after a close pass, goal cancelled at 120 s) (week 2) |
| Closest robot-to-robot approach / near misses (< 0.5 m) | **0.49 m / 1** and **0.42 m / 2** (week 2) |
| Fault injection: robot2's safety monitor crashed (SIGKILL) mid-mission | **gate zeroed in 0.50 s (timeout 0.5 s), robot halted in 0.78 s; robot1/robot3 kept driving (0.75 m, 1.17 m in 5 s); resumed 2.8 s after restart** |
| Camera alone: true stop-zone entries missed (crossing scenario) | **4 of 13**: a 60° camera sees the full 0.8 m-wide zone only beyond ~0.76 m (week 3) |
| Camera + lidar near field: entries missed | **0 of 16** (week 3) |
| Stop latency after a person enters the stop zone | **command 0.13 s, robot halted 0.46 s** (medians, n=3) (week 3) |
| False stops | **0 in 24.5 robot-minutes** (no people) and 1 in 3 robot-minutes (with people) (week 3) |
| Camera recall for a person ahead, by distance | 0.5-1.0 m 7/56, 1.0-1.5 m 24/117, 1.5-2.0 m 12/14, 2.0-2.5 m 36/59 (week 3) |
| OOD camera check (AUROC, 6 corruptions) | **image statistics + Mahalanobis 0.93**, MobileNetV2 features 0.80, ImageNet energy score 0.45 (worse than chance) (week 3) |
| Deployed OOD threshold | 0% clean frames flagged; fog/dark/noise 100%, smudge 60%, blur and overexposure 12% (week 3) |

Sample sizes are small (minutes of simulation), so these are engineering measurements, not
safety claims.

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
make fault             # crash robot2's safety monitor mid-mission, score the fail-safe
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

**Whole fleet in one view** (`make sim-fleet` or `make fleet`): set the 3D panel's display
frame to `world` and enable `/tf`, `/fleet/robot1/scan` … `/fleet/robot3/scan` and
`/fleet/robot1/map`. For the robot models add one **URDF** custom layer per robot with topic
`/robotN/robot_description` and frame prefix `robotN/`. `fleet_tf_relay` merges each robot's
own TF tree into the global `/tf` as `robotN/<frame>`; the robots' trees are untouched.

## Roadmap

See [docs/ROADMAP.md](docs/ROADMAP.md).

## In italiano

Flotta di robot mobili in ROS 2 / Nav2 con un livello di sicurezza basato su visione: i robot si fermano quando una persona entra nella zona protetta e rallentano quando la telecamera vede qualcosa di sconosciuto. Il gate di velocità è scritto in C++ ed è fail-safe. Collegato al nuovo Regolamento Macchine UE.

## Credits

- Walking person: "actor" by Mingfei on [Gazebo Fuel](https://app.gazebosim.org/Mingfei/fuel/models/actor), CC BY 4.0
- Person detector: YOLOv4-tiny (COCO) weights from [AlexeyAB/darknet](https://github.com/AlexeyAB/darknet)
- Robot and world: `nav2_minimal_tb3_sim` (Apache-2.0)

## Licence

Code: MIT, see [LICENSE](LICENSE). Datasets and third-party assets keep their own licences;
see [data/README.md](data/README.md).
