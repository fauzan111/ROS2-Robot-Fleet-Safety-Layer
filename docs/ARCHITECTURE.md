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

## Topic remapping

Nav2 publishes `cmd_vel`. Remap it into the gate and the gate output to the robot:
`ros2 run speed_gate speed_gate --ros-args -r cmd_vel_in:=/cmd_vel_nav -r cmd_vel_out:=/cmd_vel`
