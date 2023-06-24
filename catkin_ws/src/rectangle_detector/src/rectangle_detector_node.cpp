#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "rectangle_detector_node.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received Image");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Find edges
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150, 3, false);

    // Create an instance of the FastLineDetector class
    cv::Ptr<cv::ximgproc::FastLineDetector> lsd = cv::ximgproc::createFastLineDetector();

    // Detect the lines
    std::vector<cv::Vec4f> lines;
    cv::Mat mask = cv::Mat::ones( image.size(), CV_8UC1 );
    lsd->detect(edges, lines);
    
    ROS_INFO("Found %lu lines", lines.size());

    cv::Mat cdst = cv::Mat::zeros(edges.size(), CV_8UC1);
    cv::RNG rng(12345);
    cv::Scalar color = cv::Scalar(255);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4f l = lines[i];
        // cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 1, cv::LINE_AA);
    }

    // Find polygons in the cdst image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cdst, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    ROS_INFO("Found %lu contours", contours.size());

    // Find polygons with four corners
    std::vector<std::vector<cv::Point>> polygons;
    // get number of pixels in cdst
    int cdst_area = cdst.rows * cdst.cols;
    for (const auto& contour : contours)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.02, true);
        if (approx.size() == 4 && cv::isContourConvex(approx) && cv::contourArea(approx)/cdst_area > 0.001 && cv::contourArea(approx)/cdst_area < 0.4)
            polygons.push_back(approx);
    }

    // Draw polygons
    for (const auto& polygon : polygons)
    {
        // Draw corners
        for (const auto& point : polygon)
            cv::circle(image, point, 3, cv::Scalar(0, 255, 0), -1);

        // Draw edges
        for (size_t i = 0; i < polygon.size(); i++)
            cv::line(image, polygon[i], polygon[(i + 1) % polygon.size()], cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("Image", image);
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_detector");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("/image_publisher/image_raw", 1, imageCallback);

    cv::namedWindow("Image");

    while (ros::ok())
        ros::spinOnce();

    return 0;
}

