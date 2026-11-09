#!/bin/bash
# Run inside the container: start the simulation, wait for Nav2, run the mission list.
# Watchdog: if the simulation process dies, stop immediately and show its log.
MISSIONS=${1:-/ws/src/fleet_bringup/config/missions_one_robot.yaml}
SIM_LOG=/ws/results/sim.log

ros2 launch fleet_bringup sim_one_robot.launch.py > "$SIM_LOG" 2>&1 &
SIM_PID=$!
ros2 run fleet_bringup mission_runner --ros-args -p use_sim_time:=true -p missions_file:="$MISSIONS" &
MISSION_PID=$!
trap 'kill $MISSION_PID $SIM_PID 2>/dev/null; wait 2>/dev/null' EXIT

while kill -0 $MISSION_PID 2>/dev/null; do
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERROR: simulation exited before the missions finished. Last lines of $SIM_LOG:"
    tail -20 "$SIM_LOG"
    exit 1
  fi
  sleep 2
done
wait $MISSION_PID
