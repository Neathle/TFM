#include <iostream>
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_name");
  ros::NodeHandle n;
  std::cout << "Hello World \n";
  ros::spinOnce();
  return 0;
}