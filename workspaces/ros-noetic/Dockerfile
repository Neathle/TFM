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

# CREATE A ROS WORKSPACE
VOLUME ros-noetic-volume:/home/ros/catkin_ws/src
WORKDIR /home/ros/catkin_ws
RUN catkin init

WORKDIR /home/ros/catkin_ws/src
RUN git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
RUN git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git

WORKDIR /home/ros/catkin_ws/
RUN rosdep install --from-paths src -i
RUN rosdep fix-permissions && rosdep update

RUN catkin config  --extend /opt/ros/noetic
RUN catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release -j $(expr `nproc` / 2)

RUN echo "source /home/ros/catkin_ws/devel/setup.bash" >> /root/.bashrc

# INSTALL KEYBOARD CONTROL
WORKDIR /home/ros
RUN apt-get install kmod python3-pip -y
RUN git clone https://github.com/devbharat/python-uinput
WORKDIR /home/ros/python-uinput
RUN python3 -m pip install pygame
RUN python3 setup.py build && python3 setup.py install

