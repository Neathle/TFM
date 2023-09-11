#!/bin/bash
echo "Taking rviz screenshot and killing nodes"
sleep 260
my_date=$(date +"%Y-%m-%d_%H:%M:%S")
import -w root -crop 1500x890+2350+120 test_amcl_${my_date}.png
#import -window root odom$2_path_$1_${my_date}.png
rosnode kill -a 