# Roadmap: ROS 2 Robot Fleet with a Safety Layer

## Week 1: one robot

- [x] Docker image builds; TurtleBot navigates in Gazebo with Nav2
- [x] Foxglove shows map, robot and laser (camera moves to week 3, with the detector)
- [x] speed_gate in the loop (pass-through monitor)

### Week 1 notes

- Single choke point: the robot only obeys `/cmd_vel_safe`, published only by `speed_gate`.
  Found and closed a bypass: Nav2's `docking_server` published straight to `/cmd_vel`.
- AMCL starts at the spawn pose via `fleet_bringup/config/nav2_overrides.yaml`.
- First mission: 5/5 goals, 44.4 s sim time.
- Foxglove bridge checked with a protocol client: 83 topics; `/scan` 5 Hz, `/odom` 28 Hz,
  `/tf` 46 Hz, `/safety/speed_scale` 20 Hz; `/map` and `/robot_description` latched.
  foxglove_bridge 3.x accepts only the `foxglove.sdk.v1` subprotocol.
- Resources for one robot: about 1.1 GB RAM and 1.8 cores, so three robots fit in 7.7 GB.

## Week 2: fleet

- [ ] Three robots with namespaces, waypoint missions
- [ ] Mission metrics logged (completed, time, distance)

## Week 3: safety layer

- [ ] Bridge the camera to ROS and show it in Foxglove
- [ ] Person detector on the camera; zone check in the map frame
- [ ] OOD score and slow mode
- [ ] Measure stop latency and false stops

## Week 4: polish

- [ ] Fault injection: kill safety_monitor and show the fail-safe stop
- [ ] README results, 2-minute video, LinkedIn post (robotica mobile, sicurezza)

## Definition of done

- [ ] Results table filled in the README
- [ ] 2-minute demo video linked
- [ ] Tests and lint green
- [ ] LinkedIn post (Italian and English)
