FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# ROS INSTALL 
RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget git curl python-is-python3 python3-pip -y

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install ros-noetic-desktop-full -y
RUN apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# INITIALIZE ROS
RUN source /opt/ros/noetic/setup.bash
RUN rosdep fix-permissions && rosdep init
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN apt-get install ros-noetic-perception -y

CMD [ "bash" ]


