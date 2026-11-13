#!/bin/bash
# Run inside the container: start the fleet, drive the missions, crash one robot's
# safety_monitor mid-mission and score the fail-safe (results/fault_injection_<ts>.json).
# Usage: run_fault.sh [target robot, default robot2] [fleet file]
TARGET=${1:-robot2}
FLEET=${2:-/ws/src/fleet_bringup/config/fleet.yaml}
RESULTS=/ws/results
SIM_LOG=$RESULTS/sim_fault.log
mkdir -p "$RESULTS"

# No people: this test isolates the gate's reaction to a dead monitor from perception stops.
ros2 launch fleet_bringup sim_fleet.launch.py fleet_file:="$FLEET" people_file:=none > "$SIM_LOG" 2>&1 &
SIM_PID=$!
ROBOTS=$(python3 -c "import sys, yaml; print(' '.join(yaml.safe_load(open(sys.argv[1]))['robots']))" "$FLEET")
PIDS=()
for r in $ROBOTS; do
  ros2 run fleet_bringup mission_runner --ros-args -r __ns:=/"$r" \
    -p use_sim_time:=true -p missions_file:="$FLEET" > "$RESULTS/fault_mission_$r.log" 2>&1 &
  PIDS+=($!)
done
trap 'kill "${PIDS[@]}" $SIM_PID 2>/dev/null; wait 2>/dev/null' EXIT

ros2 run fleet_bringup fault_injection --ros-args -p use_sim_time:=true \
  -p fleet_file:="$FLEET" -p target:="$TARGET" &
FAULT_PID=$!
while kill -0 $FAULT_PID 2>/dev/null; do
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERROR: simulation exited during the fault test. Last lines of $SIM_LOG:"
    tail -20 "$SIM_LOG"
    exit 1
  fi
  sleep 1
done
wait $FAULT_PID
grep -E "process has died|safety scale stale" "$SIM_LOG" | head -5
