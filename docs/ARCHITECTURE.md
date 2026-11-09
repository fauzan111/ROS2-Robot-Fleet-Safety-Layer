# Architecture

```
                 camera/image_raw                     /safety/speed_scale (Float32)
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
