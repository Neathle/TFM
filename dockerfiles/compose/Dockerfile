FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# ROS INSTALL 
RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget git ros-noetic-catkin python3-catkin-tools python-is-python3 python3-pip -y
RUN apt-get install libopencv-dev python3-opencv -y

RUN apt-get install ros-noetic-vision-opencv -y

# INITIALIZE ROS
RUN rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN source /opt/ros/noetic/setup.bash

