FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash", "-c"]

VOLUME ros-noetic-volume:/home/ros
WORKDIR /home/ros/

RUN apt-get update && apt-get install apt-utils -y
RUN apt-get install wget git ros-melodic-catkin python3-catkin-tools -y

RUN mkdir tiago_public_ws
WORKDIR /home/ros/tiago_public_ws
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/kinetic-devel/tiago_public-melodic.rosinstall
RUN rosinstall src /opt/ros/melodic tiago_public-melodic.rosinstall

RUN rosdep fix-permissions \
 && rosdep update

RUN rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller pal_moveit_capabilities pal_moveit_plugins pal_loc_measure pal_map_manager"

RUN source /opt/ros/melodic/setup.bash
RUN catkin config  --extend /opt/ros/melodic
RUN catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)

RUN echo "source /home/ros/tiago_public_ws/devel/setup.bash" >> /root/.bashrc

