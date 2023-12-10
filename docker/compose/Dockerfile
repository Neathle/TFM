FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

ARG USER
ARG UID
ARG GID

ENV USERNAME ${USER}
RUN useradd -M $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid ${UID} $USERNAME && \
        groupmod --gid ${GID} $USERNAME

# cleanup
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# INSTALL DEPENDENCIES
RUN apt-get update && apt-get install wget git python-is-python3 python3-pip -y
RUN apt-get update && apt-get install -y \
      libsqlite3-dev \
      libpcl-dev \
      git \
      cmake \
      libyaml-cpp-dev \
      software-properties-common \
      pkg-config \ 
      wget \
      libsuitesparse-dev

# OPENCV INSTALL
WORKDIR /workspace
RUN git clone https://github.com/opencv/opencv_contrib.git && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv_contrib && \
    git checkout tags/4.2.0 && \
    cd /workspace && \
    cd opencv && \
    git checkout tags/4.2.0 && \
    mkdir build && \
    cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=/workspace/opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspace && \
    rm -rf opencv opencv_contrib

# ROS INSTALL 

RUN apt-get install ros-noetic-perception -y
RUN apt-get install libcanberra-gtk-module libcanberra-gtk3-module -y
RUN apt-get install gdb -y

RUN apt-get install tree iputils-ping -y

ENV SHELL /usr/bin/bash
RUN setcap cap_sys_ptrace=eip /usr/bin/gdb
ENV NO_AT_BRIDGE 1

# INITIALIZE ROS
RUN rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# TIAGO INSTALL
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git wget ipython3 python3-rosinstall \
        python-is-python3 ros-noetic-actionlib-tools ros-noetic-moveit-commander

WORKDIR /tiago_ws
RUN chmod agu+rwx /tiago_ws

RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall
RUN rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall

RUN rosdep fix-permissions && rosdep update
RUN rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

RUN apt-get update && apt-get install -y python3-catkin-tools 

RUN git clone https://github.com/ros-drivers/mocap_optitrack/ -b master src/mocap_optitrack

RUN source /opt/ros/noetic/setup.bash && \
    catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc`)

# install net utils
RUN apt-get update && apt-get install net-tools nano traceroute -y

# install realsense
RUN apt-get update && apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-description curl -y
RUN curl -o /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
RUN curl -o /etc/udev/rules.d/99-realsense-d4xx-mipi-dfu.rules https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-d4xx-mipi-dfu.rules


# install other requirements
RUN apt-get update && apt-get install -y ros-noetic-navigation ros-noetic-tf2-tools ros-noetic-tf2-sensor-msgs \
                                         ros-noetic-mrpt-navigation imagemagick
RUN pip install bagpy
RUN add-apt-repository ppa:joseluisblancoc/mrpt-stable
RUN apt install libmrpt-dev mrpt-apps -y
RUN pip install evo --upgrade --no-binary evo

# clang-format
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key| apt-key add -
RUN apt-get update && apt-get install -y clang-format


# cleanup
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*


CMD [ "bash" ]
