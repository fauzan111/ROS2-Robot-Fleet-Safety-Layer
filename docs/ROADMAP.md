# Roadmap: ROS 2 Robot Fleet with a Safety Layer

## Week 1: one robot

- [ ] Docker image builds; TurtleBot navigates in Gazebo with Nav2
- [ ] Foxglove shows map, robot, laser and camera
- [ ] speed_gate in the loop (pass-through monitor)

## Week 2: fleet

- [ ] Three robots with namespaces, waypoint missions
- [ ] Mission metrics logged (completed, time, distance)

## Week 3: safety layer

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
