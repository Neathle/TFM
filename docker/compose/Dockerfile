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

# INSTALL DEPENDENCIES
RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget git python-is-python3 python3-pip -y
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

CMD [ "bash" ]
