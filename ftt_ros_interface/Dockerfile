FROM ros:noetic

# install requirements
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    python3-ruamel.yaml \
    ros-noetic-industrial-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-cv-bridge \
    ros-noetic-rosbridge-server \
    ros-noetic-image-transport \
    python3-opencv \
    libopencv-dev \
    libyaml-cpp-dev \
    git \
  && python3 -m pip install -U \
    requests \
    Pillow \
  && rm -rf /var/lib/apt/lists/*

# make the directory for the ROS code (to be mounted as volume)
RUN mkdir -p /ros/src/ftt_ros_interface

# compile and launch ros node
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash; cd /ros; catkin_make; source /ros/devel/setup.bash; roslaunch ftt_ros_interface ftt_ros.launch"]
