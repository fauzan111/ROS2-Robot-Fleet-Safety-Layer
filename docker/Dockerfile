# ROS 2 Jazzy + Nav2 + Gazebo (headless). Native arm64 images run on Apple Silicon.
FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-foxglove-bridge \
      python3-colcon-common-extensions \
    && apt-get clean
WORKDIR /ws
COPY ros2_ws/src /ws/src
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
