#!/bin/bash
# Run inside the container: a fleet scenario with the safety recorder, then score it.
# Usage: run_eval.sh <tag> <fleet file> <people file | none>
TAG=${1:-crossing}
FLEET=${2:-/ws/src/fleet_bringup/config/crossing.yaml}
PEOPLE=${3:-/ws/src/fleet_bringup/config/people_crossing.yaml}
RESULTS=/ws/results
SIM_LOG=$RESULTS/sim_eval_$TAG.log
mkdir -p "$RESULTS"

ros2 launch fleet_bringup sim_fleet.launch.py fleet_file:="$FLEET" people_file:="$PEOPLE" \
  > "$SIM_LOG" 2>&1 &
SIM_PID=$!
ros2 run fleet_bringup safety_recorder --ros-args -p use_sim_time:=true \
  -p fleet_file:="$FLEET" -p tag:="$TAG" &
REC_PID=$!
ROBOTS=$(python3 -c "import sys, yaml; print(' '.join(yaml.safe_load(open(sys.argv[1]))['robots']))" "$FLEET")
PIDS=()
for r in $ROBOTS; do
  ros2 run fleet_bringup mission_runner --ros-args -r __ns:=/"$r" \
    -p use_sim_time:=true -p missions_file:="$FLEET" > "$RESULTS/eval_${TAG}_mission_$r.log" 2>&1 &
  PIDS+=($!)
done
trap 'kill "${PIDS[@]}" $REC_PID $SIM_PID 2>/dev/null; wait 2>/dev/null' EXIT

any_running() { for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && return 0; done; return 1; }
while any_running; do
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERROR: simulation exited during the run. Last lines of $SIM_LOG:"
    tail -20 "$SIM_LOG"
    exit 1
  fi
  sleep 2
done
# Stop the recorder. `ros2 run` does not reliably pass SIGINT on (the first run hung here),
# so signal the node process itself; its log is line-buffered, so nothing is lost.
pkill -INT -f "safety_recorder --ros-args" ; sleep 2; pkill -KILL -f "safety_recorder --ros-args"
kill $REC_PID 2>/dev/null
for r in $ROBOTS; do tail -1 "$RESULTS/eval_${TAG}_mission_$r.log"; done
ros2 run fleet_bringup safety_report "$RESULTS/safety_log_$TAG.jsonl" --people "$PEOPLE"
