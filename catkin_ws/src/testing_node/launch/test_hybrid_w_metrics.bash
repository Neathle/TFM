#!/bin/bash
#roscore must be launched previously to this script
echo "############ Start Hybrid AMCL Testing ############ "
source /opt/ros/noetic/setup.bash
source /home/fer/catkin_ws/devel/setup.bash
scenario=("pare" "rx2")
delay=("2.176" "2.277")
thresh=("bin" "bin_inv")

##Scenario##
#for i in {0..1}  
#do
#	##Thresh##
#	for j in {0..1}
#	do
#		echo scenario: ${scenario[i]} 
#		echo thresh type:  ${thresh[j]}
#		num_m=$(find  /home/fer/catkin_ws/src/amcl_hybrid/detector/markers/${scenario[i]}/${thresh[j]}/ -mindepth 1 -type d | wc -l)
#		echo number of marker folders:  $num_m
#		##Marker Num##
#		for k in $(seq 1 $num_m)  
#		do 
#			test_num=1
#			mkdir -p /home/fer/.ros/"${scenario[i]}"/"${thresh[j]}"/m$k/
#			echo /home/fer/.ros/"${scenario[i]}"/"${thresh[j]}"/m$k/
#			##Test Repeat##
#			for n in {0..11} #15 times {1..15}
#			do
#				echo test num: $test_num 
#				roslaunch amcl_hybrid amcl_diff.launch scenario:=${scenario[i]} file_num:=$test_num thresh:=${thresh[j]} delay:=${delay[i]} num_markers:=$k
#				((test_num=test_num+1))
#			done
#		done 
#	done
#	echo " "
#done

roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin/m1/ scenario:=pare
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin/m2/ scenario:=pare
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin/m3/ scenario:=pare
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin/m4/ scenario:=pare
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin_inv/m1/ scenario:=pare
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin_inv/m2/ scenario:=pare
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/pare/bin_inv/m3/ scenario:=pare
#
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/rx2/bin/m1/        scenario:=rx2
#roslaunch testing_node compute_metrics.launch data_path:=/home/fer/.ros/rx2/bin_inv/m1/ scenario:=rx2


