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

- [x] Three robots with namespaces, waypoint missions
- [x] Mission metrics logged (completed, time, distance, near misses)
- [x] Foxglove fleet view (relay each robot's namespaced TF into one tree)

### Week 2 notes

- `make fleet`: 14/14 goals, makespan 58.8 s sim time, 14.3 goals/min; 1 near miss
  (robot1-robot3, closest 0.49 m) where robot3's north-south route crosses robot1's.
  | Robot | Goals | Mission time (sim s) | Distance (m) |
  |---|---|---|---|
  | robot1 | 5/5 | 50.6 | 7.4 |
  | robot2 | 5/5 | 58.8 | 7.0 |
  | robot3 | 4/4 | 42.9 | 8.7 |
- Distance comes from wheel odometry. A goal counts as reached within 0.25 m, so a robot can
  drive slightly less than the straight-line sum of its waypoints.
- Each robot has its own safety chain in its namespace (gate + monitor on
  `/robotN/safety/speed_scale`), designed so a failure in one chain stops only that robot
  (demonstrated by `make fault`, see below).
- Nav2 params start from upstream `nav2_multirobot_params_all.yaml` (DWB controller,
  namespaced scan topics). Two fixes were needed: `route_server` has no section there but the
  Jazzy bringup starts it, and the collision monitor read an absolute
  `/local_costmap/published_footprint`.
- Resources: the whole fleet uses about 1.7 GB RAM and 2.6 cores, only 0.6 GB and 0.8 cores
  more than one robot with MPPI in week 1, because DWB samples far fewer trajectories.

### Week 2 follow-ups: fleet view and fault injection

- Second `make fleet` run: 13/14. robot3 passed robot1 at 0.42 m (t = 13.9 s) near the
  south-west pillar; robot1 then failed to make progress 15 times, Nav2's spin and backup
  recoveries failed, and its `south_west` goal was cancelled at the 120 s limit (it finished
  every later goal). Robots only see each other as laser obstacles, with no coordination.
  Candidate fixes: one-way traffic lanes via keepout/speed filter masks, or a simple
  reservation of the pillar gaps by the fleet layer.

- `make fault` (target robot2): SIGKILL its safety_monitor once all robots have driven for 1 s.
  Gate output zero after 0.50 s (heartbeat timeout 0.5 s), robot halted after 0.78 s, robot1
  and robot3 drove 0.75 m and 1.17 m during the 5 s outage, robot2 resumed 2.8 s after the
  monitor restart (mostly Python start-up). All checks passed.
- The test refuses to score a run where not every robot is driving. Its first run did exactly
  that: robot1's mission runner hung forever in `waitUntilNav2Active()` because its DDS
  discovery of `amcl/get_state` got stuck under start-up load. `mission_runner` now waits with
  a 60 s bound and rebuilds the navigator node to redo discovery (3 attempts).
- Foxglove fleet view: `fleet_tf_relay` republishes `/robotN/tf` on `/tf` as `robotN/<frame>`
  under `world`, plus `/fleet/robotN/{scan,plan,map}`. Checked as a Foxglove client: `/tf`
  56 Hz with all three robots, three scans at 5 Hz, map and robot models latched; and
  `tf2_echo world robotN/base_link` resolves for every robot.

## Week 3: safety layer

- [ ] Bridge the camera to ROS and show it in Foxglove (the waffle's depth camera topic is
      not namespaced in the upstream model, so each robot needs its own camera topic)
- [ ] Person detector on the camera; zone check in the map frame
- [ ] OOD score and slow mode
- [ ] Measure stop latency and false stops

## Week 4: polish

- [x] Fault injection: kill safety_monitor and show the fail-safe stop (done in week 2)
- [ ] README results, 2-minute video, LinkedIn post (robotica mobile, sicurezza)

## Definition of done

- [ ] Results table filled in the README
- [ ] 2-minute demo video linked
- [ ] Tests and lint green
- [ ] LinkedIn post (Italian and English)
