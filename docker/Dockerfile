# ROS 2 Jazzy + Nav2 + Gazebo (headless). Native arm64 images run on Apple Silicon.
FROM ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-foxglove-bridge \
      python3-colcon-common-extensions \
    && apt-get clean
# Perception: OpenCV (DNN module runs the detector) and cv_bridge, no pip packages needed.
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl ros-jazzy-cv-bridge python3-opencv \
    && apt-get clean
# Walking person for the simulation: Gazebo Fuel "actor" by Mingfei, version 1, CC BY 4.0.
# Baked into the image (pinned by checksum) so runs don't depend on the network.
RUN mkdir -p /opt/fleet_models/actor \
    && curl -fsSL -o /opt/fleet_models/actor/walk.dae \
       https://fuel.gazebosim.org/1.0/Mingfei/models/actor/1/files/meshes/walk.dae \
    && echo "49af0df3a319d1cb8ca2cebf02dbd00f625e5d5bec820bc5e109925b18b65c6e  /opt/fleet_models/actor/walk.dae" \
       | sha256sum -c -
WORKDIR /ws
COPY ros2_ws/src /ws/src
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
