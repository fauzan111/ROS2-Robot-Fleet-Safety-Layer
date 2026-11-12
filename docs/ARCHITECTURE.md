# Architecture

```
                 camera/image_raw                      safety/speed_scale (Float32)
 Gazebo robot ───────────────────► safety_monitor ───────────────────────────┐
     ▲                          (Python: detector, zone, OOD)                │
     │                                                                       ▼
     │  cmd_vel_out (Twist)                            cmd_vel_in       ┌─────────────┐
     └───────────────────────────── speed_gate ◄───────────────────────│    Nav2     │
                                  (C++, fail-safe)                      └─────────────┘
```

## Design decisions

- **The gate is separate from the detector.** The C++ `speed_gate` is small and easy to
  review; the ML part (`safety_monitor`) can change without touching the component that
  actually moves the robot.
- **Fail-safe by default.** No speed scale yet, or a stale one (older than 0.5 s), means the
  robot stops. A crashed monitor never leaves a robot moving.
- **Hysteresis.** Leaving STOP needs the zone to be clear for 1 s (`fleet_ml.zone`), so a
  flickering detection cannot make the robot lurch.
- **OOD means slow.** If the camera sees something unlike its training data (fog, a dirty lens,
  an unusual object), the detector may be blind, so the robot slows to 30%.

## Regulation context

The EU Machinery Regulation (EU) 2023/1230 applies from 20 January 2027. Safety components
with self-evolving (machine learning) behaviour need third-party conformity assessment, and
risk assessment must cover behaviour after the machine is placed on the market. This project
is a learning demonstrator, **not** a certified safety function; real deployments need a
certified safety PLC and laser scanners (ISO 13849 / IEC 61508).

## Single choke point (topic wiring)

The simulated robot's bridge is launched with `cmd_vel` remapped to `cmd_vel_safe`, and only
`speed_gate` publishes `cmd_vel_safe`. Every Nav2 node that commands motion publishes `cmd_vel`,
so all of them pass through the gate:

```
Nav2 (collision_monitor, docking_server, ...) --/cmd_vel--> speed_gate --/cmd_vel_safe--> robot
```

Why not just rewire one Nav2 output? Because Nav2 has more than one velocity publisher. During
development, `ros2 topic info -v /cmd_vel` showed `docking_server` publishing straight to the
robot, which would have bypassed a gate placed only after the collision monitor. Guarding the
robot's *input* removes that whole class of bypass.

## Fleet (week 2)

`sim_fleet.launch.py` reads `config/fleet.yaml` and starts, per robot, everything above inside
the robot's namespace, plus one shared Gazebo world, one `/clock` bridge and a `fleet_monitor`:

```
/robot1:  Nav2 --cmd_vel--> speed_gate --cmd_vel_safe--> ros_gz_bridge --> Gazebo /robot1/cmd_vel
                                ^ safety/speed_scale (safety_monitor)
/robot2:  ... same chain ...
/robot3:  ... same chain ...
fleet_monitor: /robotN/odom -> separation, near misses, distance -> results/fleet_monitor.json
```

- **One safety chain per robot.** All names are relative, so each gate listens only to its own
  robot's monitor. A person next to robot1 should not stop robot3 across the hall, and a
  crashed monitor should stop only its own robot (fault-injection test planned for week 4).
- **The choke point is in the bridge config, not a remap.** `config/tb3_bridge.yaml` maps ROS
  `cmd_vel_safe` to the Gazebo wheels' `cmd_vel`; nothing else reaches the wheels. That is
  easier to audit than a launch-file remap.
- **Separate TF trees.** Each robot's frames live on `/robotN/tf` (Nav2's multi-robot pattern),
  so robots never fight over `map`/`odom`/`base_link`. The cost: a single Foxglove 3D view
  needs a relay that prefixes frames (`robot1/base_link`) into one tree.
- **Near misses use ground-truth-like odometry.** Gazebo's diff-drive odometry starts at zero
  at the spawn pose; `fleet_monitor` transforms it to the world frame with each spawn pose and
  counts a near miss below 0.5 m centre-to-centre, with 0.1 m hysteresis so one close pass is
  one event.
