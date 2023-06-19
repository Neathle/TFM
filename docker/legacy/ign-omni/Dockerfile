ARG FROM_IMAGE=ros:galactic-ros-base

# multi-stage for caching
FROM $FROM_IMAGE
SHELL ["/bin/bash", "-c"]

VOLUME ign-omni-volume:/ign_omni/src
WORKDIR /ign_omni/src
ENV DEBIAN_FRONTEND noninteractive
# COPY ./keyboard /etc/default/keyboard

RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget python3-pip lsb-release gnupg curl -y

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && apt-get install python3-vcstool python3-colcon-common-extensions git libfreeimage-dev -y
RUN apt-get install ignition-fortress -y



