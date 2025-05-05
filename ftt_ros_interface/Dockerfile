FROM ros:jazzy

# install requirements
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-ruamel.yaml \
    ros-jazzy-tf2-ros \
    ros-jazzy-cv-bridge \
    ros-jazzy-rosbridge-server \
    ros-jazzy-image-transport \
    python3-opencv \
    libcurl4-openssl-dev \
    python3-requests \
    python3-pil \
    libopencv-dev \
    libyaml-cpp-dev \
    git \
  && rm -rf /var/lib/apt/lists/*

# make the directory for the ROS code (to be mounted as volume)
RUN mkdir -p /ros2/src/ftt_ros_interface
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# compile and launch ros node
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash; cd /ros2; colcon build --symlink-install; source /ros2/install/setup.bash; ros2 launch ftt_ros_interface ftt_ros.launch.xml"]
