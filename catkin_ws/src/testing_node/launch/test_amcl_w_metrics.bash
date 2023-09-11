#!/bin/bash
#roscore must be launched previously to this script
echo "start amcl testing"
source /opt/ros/noetic/setup.bash
source /home/fer/catkin_ws/devel/setup.bash
scenario=("pare" "rx2")
for i in {0..1}
do
	test_num=1
	mkdir /home/fer/.ros/"${scenario[i]}_depth"
	for j in {1..15}
	do
		roslaunch run_amcl test_amcl_w_metrics_virtual_laser.launch scenario:=${scenario[i]} file_num:=$test_num
		((test_num=test_num+1))
	done 
done

roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/ scenario:=pare
roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare_depth/ scenario:=pare
roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/rx2/ scenario:=rx2
roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/rx2_depth/ scenario:=rx2
