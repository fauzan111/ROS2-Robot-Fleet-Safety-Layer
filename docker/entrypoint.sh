#!/bin/bash
# Source ROS 2 and, if built, this workspace for every command (interactive or not).
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
