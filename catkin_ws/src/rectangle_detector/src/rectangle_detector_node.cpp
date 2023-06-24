#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <numeric>
#include "rectangle_detector_node.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received Image");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Extract lines from the image using FLD
    std::vector<cv::Vec4f> lines = extractLines(gray);
    ROS_INFO("Extracted %lu lines", lines.size());

    // Create a black image
    cv::Mat linesImg = cv::Mat::zeros(gray.size(), CV_8UC1);

    // Paint the lines on the black image in random colors
    cv::Scalar color(255);
    for (const cv::Vec4f &line : lines)
    {
        cv::line(linesImg, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, 2, cv::LINE_AA);
    }

    // Find all the intersection points between the lines
    std::vector<cv::Point2i> intersections = findIntersections(lines);
    ROS_INFO("Found %lu intersections", intersections.size());

    // Draw the intersection points on the image
    for (const cv::Point2i &intersection : intersections)
    {
        cv::circle(linesImg, intersection, 2, cv::Scalar(255), 2);
    }

    // Create a graph where nodes are intersection points and edges are lines that join them
    std::vector<std::vector<bool>> graph = createGraph(linesImg, intersections);

    // Create a black image
    cv::Mat graphImg = cv::Mat::zeros(gray.size(), CV_8UC3);

    // Draw valid lines
    for (int i = 0; i < graph.size(); i++)
    {
        for (int j = i + 1; j < graph.size(); j++)
        {
            if (graph[i][j])
            {
                cv::line(graphImg, intersections[i], intersections[j], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
    }
    for (const cv::Point2i &intersection : intersections)
    {
        cv::circle(graphImg, intersection, 2, cv::Scalar(255, 255, 255), 2);
    }

    // Extract all possible trapezoids from the graph

    std::vector<cv::Vec8i> trapezoids = findTrapezoids(intersections, graph, 0.001 * gray.rows * gray.cols, 0.3 * gray.rows * gray.cols);
    ROS_INFO("Found %lu trapezoids", trapezoids.size());

    // Draw the trapezoids on the image
    image = reduceSaturationBrightness(image, 0.5f, 0.5f);
    for (const auto &trapezoid : trapezoids)
    {
        color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::line(image, cv::Point(trapezoid[0], trapezoid[1]), cv::Point(trapezoid[2], trapezoid[3]), color, 1, cv::LINE_AA);
        cv::line(image, cv::Point(trapezoid[2], trapezoid[3]), cv::Point(trapezoid[4], trapezoid[5]), color, 1, cv::LINE_AA);
        cv::line(image, cv::Point(trapezoid[4], trapezoid[5]), cv::Point(trapezoid[6], trapezoid[7]), color, 1, cv::LINE_AA);
        cv::line(image, cv::Point(trapezoid[6], trapezoid[7]), cv::Point(trapezoid[0], trapezoid[1]), color, 1, cv::LINE_AA);
    }

    // Publish images
    cv_bridge::CvImage cv_image_lines;
    cv_image_lines.header = msg->header;
    cv_image_lines.encoding = sensor_msgs::image_encodings::MONO8;
    cv_image_lines.image = linesImg;
    im_pub_lines.publish(cv_image_lines.toImageMsg());

    cv_bridge::CvImage cv_image_rectangles;
    cv_image_rectangles.header = msg->header;
    cv_image_rectangles.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image_rectangles.image = image;
    im_pub_rectangles.publish(cv_image_rectangles.toImageMsg());

    cv_bridge::CvImage cv_image_graph;
    cv_image_graph.header = msg->header;
    cv_image_graph.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image_graph.image = graphImg;
    im_pub_graph.publish(cv_image_graph.toImageMsg());

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_detector");
    ros::NodeHandle nh;

    ros::Subscriber im_sub = nh.subscribe("/image_publisher/image_raw", 1, imageCallback);
    im_pub_lines = nh.advertise<sensor_msgs::Image>("/rectangle_detector/lines", 1);
    im_pub_rectangles = nh.advertise<sensor_msgs::Image>("/rectangle_detector/rectangles", 1);
    im_pub_graph = nh.advertise<sensor_msgs::Image>("/rectangle_detector/graph", 1);

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

// Function to check if a line is sufficiently white
bool isWhite(cv::Mat &img, cv::Point2i p1, cv::Point2i p2)
{
    cv::LineIterator it(img, p1, p2);
    int whiteCount = 0;
    for (int i = 0; i < it.count; i++, ++it)
    {
        if (img.at<uchar>(it.pos()) > 0)
        {
            whiteCount++;
        }
    }
    return (whiteCount >= it.count * 0.95);
}

// Function to extract lines from an image using FLD
std::vector<cv::Vec4f> extractLines(cv::Mat &img)
{
    // Create a black image
    cv::Mat linesImg = cv::Mat::zeros(img.size(), CV_8UC1);

    // Extract lines from the image using FLD
    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector();
    std::vector<cv::Vec4f> lines;
    fld->detect(img, lines);

    // Paint the lines on the black image
    for (const cv::Vec4f &line : lines)
    {
        cv::line(linesImg, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), 1);
    }

    elongate_lines(lines, 1.2);

    return lines;
}

// Function to check if two line segments intersect
bool intersect(cv::Vec4f line1, cv::Vec4f line2, cv::Point2i &int_pt)
{
    cv::Point2f p = cv::Point2f(line1[0], line1[1]);
    cv::Point2f q = cv::Point2f(line2[0], line2[1]);
    cv::Point2f s = cv::Point2f(line2[2] - line2[0], line2[3] - line2[1]);
    cv::Point2f r = cv::Point2f(line1[2] - line1[0], line1[3] - line1[1]);

    float rxs = r.x * s.y - r.y * s.x;
    float qpxr = (q.x - p.x) * r.y - (q.y - p.y) * r.x;
    if (rxs == 0)
        return false;
    
    float t = ((q.x - p.x) * s.y - (q.y - p.y) * s.x) / rxs;
    float u = ((q.x - p.x) * r.y - (q.y - p.y) * r.x) / rxs;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
    {
        int_pt.x = std::round(p.x + t * r.x);
        int_pt.y = std::round(p.y + t * r.y);
        return true;
    }

    return false;
}

std::vector<cv::Point2i> nonMaximumSuppression(const std::vector<cv::Point2i>& intersections, float threshold) 
{
    std::vector<cv::Point2i> filteredIntersections;
    std::vector<bool> suppressed(intersections.size(), false);
    for (int i = 0; i < intersections.size(); ++i) {
        if (suppressed[i]) continue;

        filteredIntersections.push_back(intersections[i]);
        for (int j = i + 1; j < intersections.size(); ++j) {
            if (cv::norm(intersections[i] - intersections[j]) < threshold) {
                suppressed[j] = true;
            }
        }
    }
    return filteredIntersections;
}

// Function to find all the intersection points between a set of lines
std::vector<cv::Point2i> findIntersections(std::vector<cv::Vec4f> &lines)
{
    std::vector<cv::Point2i> intersections;
    for (size_t i = 0; i < lines.size(); i++)
    {
        for (size_t j = i + 1; j < lines.size(); j++)
        {
            cv::Point2i intersection;
            if (intersect(lines[i], lines[j], intersection))
                intersections.push_back(intersection);
        }
    }

    return nonMaximumSuppression(intersections, 1);
}

// Function to create a graph where nodes are intersection points and edges are lines that join them
std::vector<std::vector<bool>> createGraph(cv::Mat &linesImg, std::vector<cv::Point2i> &intersections)
{
    std::vector<std::vector<bool>> adjacencyMatrix(intersections.size(), std::vector<bool>(intersections.size(), false));
    for (int i = 0; i < intersections.size(); ++i) {
        for (int j = i + 1; j < intersections.size(); ++j) {
            if (isWhite(linesImg, intersections[i], intersections[j])) {
                adjacencyMatrix[i][j] = true;
                adjacencyMatrix[j][i] = true;
            }
        }
    }

    //Number of valid lines
    ROS_INFO("Number of valid lines: %d", 
        std::accumulate(adjacencyMatrix.begin(), adjacencyMatrix.end(), 0, [](int sum, const std::vector<bool>& v) {
            return sum + std::accumulate(v.begin(), v.end(), 0);
        }) / 2
    );
    return adjacencyMatrix;
}

// Function to calculate the area of a trapezoid
int trapezoid_area(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d) 
{
    return std::abs((a.x * b.y - a.y * b.x) + (b.x * c.y - b.y * c.x) + (c.x * d.y - c.y * d.x) + (d.x * a.y - d.y * a.x)) / 2;
}

// Function to extract all possible trapezoids from a graph
std::vector<cv::Vec8i> findTrapezoids(const std::vector<cv::Point2i>& intersections, const std::vector<std::vector<bool>>& adjacencyMatrix, int minArea, int maxArea) 
{
    std::vector<cv::Vec8i> trapezoids;
    for (int i = 0; i < intersections.size(); ++i) {
        for (int j = i + 1; j < intersections.size(); ++j) {
            if (!adjacencyMatrix[i][j]) continue;

            for (int k = j + 1; k < intersections.size(); ++k) {
                if (!adjacencyMatrix[j][k]) continue;

                for (int l = k + 1; l < intersections.size(); ++l) {
                    if (!adjacencyMatrix[k][l] || !adjacencyMatrix[l][i]) continue;
                    
                    int trapezoidArea = trapezoid_area(intersections[i], intersections[j], intersections[k], intersections[l]);
                    if (trapezoidArea < minArea || trapezoidArea > maxArea) continue;

                    trapezoids.push_back(cv::Vec8i(intersections[i].x, intersections[i].y,
                                                    intersections[j].x, intersections[j].y,
                                                    intersections[k].x, intersections[k].y,
                                                    intersections[l].x, intersections[l].y));
                }
            }
        }
    }
    return trapezoids;
}

cv::Mat reduceSaturationBrightness(const cv::Mat& image, float saturationScale, float valueScale) 
{
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsvImage, channels);

    channels[1] *= saturationScale;
    channels[2] *= valueScale;

    cv::merge(channels, hsvImage);
    cv::Mat result;
    cv::cvtColor(hsvImage, result, cv::COLOR_HSV2BGR);
    return result;
}