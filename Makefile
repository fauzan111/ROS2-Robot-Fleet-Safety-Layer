.PHONY: setup test lint fmt

setup:
	uv sync

test:
	uv run pytest -q

lint:
	uv run ruff check .

fmt:
	uv run ruff format . && uv run ruff check --fix .

.PHONY: docker-build docker-up ros-shell colcon

docker-build:
	docker compose build

docker-up:
	docker compose up

ros-shell:
	docker compose run --rm ros bash

colcon:
	docker compose run --rm ros bash -c "cd /ws && colcon build --symlink-install"
