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

# ROS INSTALL 
RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget git python-is-python3 python3-pip -y
RUN apt-get install libopencv-dev python3-opencv -y

RUN apt-get install ros-noetic-perception -y
RUN apt-get install libcanberra-gtk-module libcanberra-gtk3-module -y
RUN apt-get install gdb -y

RUN apt-get install tree iputils-ping -y


# INITIALIZE ROS
RUN rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

CMD [ "bash" ]
