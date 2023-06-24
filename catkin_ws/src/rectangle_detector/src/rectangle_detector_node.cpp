#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "rectangle_detector_node.hpp"

ros::Publisher im_pub_lines;
ros::Publisher im_pub_rectangles;
ros::Publisher im_pub_contours;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received Image");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Create an instance of the FastLineDetector class
    cv::Ptr<cv::ximgproc::FastLineDetector> lsd = cv::ximgproc::createFastLineDetector();

    // Detect the lines
    std::vector<cv::Vec4f> lines;
    cv::Mat mask = cv::Mat::ones( image.size(), CV_8UC1 );
    lsd->detect(gray, lines);
    elongate_lines(lines, 1.1);
    
    ROS_INFO("Found %lu lines", lines.size());

    cv::Mat cdst = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::RNG rng(12345);
    cv::Scalar color = cv::Scalar(255);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4f l = lines[i];
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 1, cv::LINE_AA);
    }
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    // cv::morphologyEx(cdst, cdst, cv::MORPH_CLOSE, kernel);

    // kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // cv::erode(cdst, cdst, kernel);

    // Find polygons in the cdst image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cdst, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    ROS_INFO("Found %lu contours", contours.size());

    // Draw contours
    cv::Mat drawing = cv::Mat::zeros(cdst.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::drawContours(drawing, contours, (int)i, color, 1, cv::LINE_AA);
    }


    // Find polygons with four corners
    std::vector<std::vector<cv::Point>> polygons;
    int cdst_area = cdst.rows * cdst.cols;
    for (const auto& contour : contours)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.1, true);
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

    // Publish images
    cv_bridge::CvImage cv_image_lines;
    cv_image_lines.header = msg->header;
    cv_image_lines.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image_lines.image = cdst;
    im_pub_lines.publish(cv_image_lines.toImageMsg());

    cv_bridge::CvImage cv_image_rectangles;
    cv_image_rectangles.header = msg->header;
    cv_image_rectangles.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image_rectangles.image = image;
    im_pub_rectangles.publish(cv_image_rectangles.toImageMsg());

    cv_bridge::CvImage cv_image_contours;
    cv_image_contours.header = msg->header;
    cv_image_contours.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image_contours.image = drawing;
    im_pub_contours.publish(cv_image_contours.toImageMsg());

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_detector");
    ros::NodeHandle nh;

    ros::Subscriber im_sub = nh.subscribe("/image_publisher/image_raw", 1, imageCallback);
    im_pub_lines = nh.advertise<sensor_msgs::Image>("/rectangle_detector/lines", 1);
    im_pub_rectangles = nh.advertise<sensor_msgs::Image>("/rectangle_detector/rectangles", 1);
    im_pub_contours = nh.advertise<sensor_msgs::Image>("/rectangle_detector/contours", 1);

    ros::spin();
}

void elongate_lines(std::vector<cv::Vec4f>& lines, float factor)
{
    for (cv::Vec4f &line : lines)
    {
        float x1 = line[0];
        float y1 = line[1];
        float x2 = line[2];
        float y2 = line[3];

        float dx = x2 - x1;
        float dy = y2 - y1;

        float extension = (factor - 1) / 2;

        line[0] = x1 - dx * extension;
        line[1] = y1 - dy * extension;
        line[2] = x2 + dx * extension;
        line[3] = y2 + dy * extension;
    }
}