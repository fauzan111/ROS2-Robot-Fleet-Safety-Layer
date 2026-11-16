.PHONY: setup test lint fmt

setup:
	uv sync

test:
	uv run pytest -q

lint:
	uv run ruff check .

fmt:
	uv run ruff format . && uv run ruff check --fix .

.PHONY: docker-build docker-up ros-shell colcon sim missions sim-fleet fleet fault eval-crossing eval-false-stops demo render-demo

docker-build:
	docker compose build

docker-up:
	docker compose up

ros-shell:
	docker compose run --rm ros bash

colcon:
	docker compose run --rm ros bash -c "cd /ws && colcon build --symlink-install"

sim:
	docker compose run --rm --service-ports ros ros2 launch fleet_bringup sim_one_robot.launch.py

missions:
	docker compose run --rm --service-ports ros /ws/scripts/run_missions.sh

sim-fleet:
	docker compose run --rm --service-ports ros ros2 launch fleet_bringup sim_fleet.launch.py

fleet:
	docker compose run --rm --service-ports ros /ws/scripts/run_fleet.sh

fault:                  # straight demo lanes: with perception on, robots must be driving when robot2's monitor dies
	docker compose run --rm --service-ports ros /ws/scripts/run_fault.sh robot2 \
	  /ws/src/fleet_bringup/config/demo.yaml

eval-crossing:          # stop latency: one robot, two people crossing its corridor
	docker compose run --rm --service-ports ros /ws/scripts/run_eval.sh crossing

eval-false-stops:       # three robots, no people: every stop is a false stop
	docker compose run --rm --service-ports ros /ws/scripts/run_eval.sh nopeople \
	  /ws/src/fleet_bringup/config/fleet.yaml none

demo:                   # record the demo scenario (people, a killed monitor, fog) to results/demo_bag
	docker compose run --rm ros /ws/scripts/run_demo.sh

render-demo:            # bag -> docs/media/demo.mp4 + demo.gif (runs on the host, uv)
	uv run python scripts/render_demo.py results/demo_bag --out docs/media
