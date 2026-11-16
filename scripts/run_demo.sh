#!/bin/bash
# Run inside the container: the demo scenario, recorded to results/demo_bag (sim-time stamps).
#   robot1 meets people crossing its corridor, robot2's safety_monitor is killed and restarted,
#   robot3's camera is fogged. Render the video on the host with `make render-demo`.
FLEET=/ws/src/fleet_bringup/config/demo.yaml
PEOPLE=/ws/src/fleet_bringup/config/people_crossing.yaml
RESULTS=/ws/results
BAG=$RESULTS/demo_bag
SIM_LOG=$RESULTS/sim_demo.log
mkdir -p "$RESULTS"; rm -rf "$BAG"

ros2 launch fleet_bringup sim_fleet.launch.py fleet_file:="$FLEET" people_file:="$PEOPLE" \
  demo:=true foxglove:=false > "$SIM_LOG" 2>&1 &
SIM_PID=$!
ROBOTS="robot1 robot2 robot3"
TOPICS=(/demo/overhead /demo/event /clock /robot1/safety/annotated /robot3/safety/annotated)
for r in $ROBOTS; do
  TOPICS+=(/$r/ground_truth/pose /$r/safety/state /$r/safety/speed_scale /$r/cmd_vel_safe
           /$r/safety/ood_score /$r/safety/persons)
done
ros2 bag record --use-sim-time -o "$BAG" "${TOPICS[@]}" > "$RESULTS/demo_bag.log" 2>&1 &
BAG_PID=$!
ros2 run fleet_bringup demo_director --ros-args -p use_sim_time:=true -p fleet_file:="$FLEET" \
  > "$RESULTS/demo_director.log" 2>&1 &
DIR_PID=$!
PIDS=()
for r in $ROBOTS; do
  ros2 run fleet_bringup mission_runner --ros-args -r __ns:=/"$r" \
    -p use_sim_time:=true -p missions_file:="$FLEET" > "$RESULTS/demo_mission_$r.log" 2>&1 &
  PIDS+=($!)
done
# Cleanup must not hang: an unbounded `wait` blocked on a child that ignored SIGTERM and
# left a finished run's container burning CPU under the next test.
trap 'kill "${PIDS[@]}" $DIR_PID $SIM_PID 2>/dev/null; sleep 3; kill -9 "${PIDS[@]}" $DIR_PID $SIM_PID 2>/dev/null' EXIT

any_running() { for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && return 0; done; return 1; }
while any_running; do
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERROR: simulation exited. Last lines of $SIM_LOG:"; tail -20 "$SIM_LOG"; exit 1
  fi
  sleep 2
done
sleep 3   # a few seconds of everyone parked at the end
# Stop the recorder and let it finish writing (about 1 GB; killing it early leaves a bag
# without its index and metadata). SIGTERM, not SIGINT: bash starts background jobs with
# SIGINT ignored, so the recorder never sees an interrupt.
kill -TERM $BAG_PID
for _ in $(seq 60); do kill -0 $BAG_PID 2>/dev/null || break; sleep 1; done
for r in $ROBOTS; do tail -1 "$RESULTS/demo_mission_$r.log"; done
grep "demo event" "$RESULTS/demo_director.log"
ros2 bag info "$BAG" | head -40
