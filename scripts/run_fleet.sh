#!/bin/bash
# Run inside the container: start the fleet simulation, run every robot's missions in
# parallel, then merge the results into results/fleet_<timestamp>.json.
# Watchdog: if the simulation process dies, stop immediately and show its log.
FLEET=${1:-/ws/src/fleet_bringup/config/fleet.yaml}
RESULTS=/ws/results
SIM_LOG=$RESULTS/sim_fleet.log
START=$(date +%s)
mkdir -p "$RESULTS"
rm -f "$RESULTS/fleet_monitor.json"

ros2 launch fleet_bringup sim_fleet.launch.py fleet_file:="$FLEET" > "$SIM_LOG" 2>&1 &
SIM_PID=$!

ROBOTS=$(python3 -c "import sys, yaml; print(' '.join(yaml.safe_load(open(sys.argv[1]))['robots']))" "$FLEET")
PIDS=()
for r in $ROBOTS; do
  ros2 run fleet_bringup mission_runner --ros-args -r __ns:=/"$r" \
    -p use_sim_time:=true -p missions_file:="$FLEET" > "$RESULTS/mission_$r.log" 2>&1 &
  PIDS+=($!)
done
trap 'kill "${PIDS[@]}" $SIM_PID 2>/dev/null; wait 2>/dev/null' EXIT
echo "fleet: $ROBOTS (logs in $RESULTS/mission_<robot>.log, $SIM_LOG)"

any_running() { for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && return 0; done; return 1; }
while any_running; do
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERROR: simulation exited before the missions finished. Last lines of $SIM_LOG:"
    tail -20 "$SIM_LOG"
    exit 1
  fi
  sleep 2
done

sleep 2  # let fleet_monitor write its final snapshot
for r in $ROBOTS; do tail -1 "$RESULTS/mission_$r.log"; done
ros2 run fleet_bringup fleet_report "$RESULTS" --since "$START"
