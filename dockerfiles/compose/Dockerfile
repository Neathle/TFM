FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# ROS INSTALL 
RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget git ros-noetic-catkin python3-catkin-tools python-is-python3 -y

# CRAZYS INSTALL
RUN apt-get install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink -y
RUN apt-get install ros-noetic-octomap-mapping ros-noetic-control-toolbox ros-noetic-mavros -y
RUN apt-get install python3-vcstool protobuf-compiler libgoogle-glog-dev python3-wstool -y

# INITIALIZE ROS
RUN rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN source /opt/ros/noetic/setup.bash

