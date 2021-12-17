FROM ros:melodic

# install requirements
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-pip \
    python-ruamel.yaml \
    ros-melodic-industrial-msgs \
    ros-melodic-tf2-ros \
    ros-melodic-cv-bridge \
    ros-melodic-rosbridge-server \
  && python -m pip install -U \
    requests \
    Pillow \
  && rm -rf /var/lib/apt/lists/*

# make the directory for the ROS code (to be mounted as volume)
RUN mkdir -p /ros/src/ftt_ros_interface

# compile and launch ros node
CMD ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash; cd /ros; catkin_make; source /ros/devel/setup.bash; roslaunch ftt_ros_interface ftt_ros.launch"]
