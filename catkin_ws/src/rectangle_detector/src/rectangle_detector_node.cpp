#include "rectangle_detector_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_detector");

    RectangleDetectorNode node;

    ros::spin();
}