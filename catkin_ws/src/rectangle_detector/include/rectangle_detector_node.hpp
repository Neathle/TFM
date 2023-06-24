#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher im_pub_lines;
ros::Publisher im_pub_rectangles;
ros::Publisher im_pub_graph;
cv::RNG rng(0xFFFFFFFF);


void elongate_lines(std::vector<cv::Vec4f>& lines, float factor);
bool isWhite(cv::Mat &img, cv::Point2f p1, cv::Point2f p2);
std::vector<cv::Vec4f> extractLines(cv::Mat &img);
bool intersect(cv::Vec4f line1, cv::Vec4f line2, cv::Point2i &int_pt);
std::vector<cv::Point2i> nonMaximumSuppression(const std::vector<cv::Point2i>& intersections, float threshold);
std::vector<cv::Point2i> findIntersections(std::vector<cv::Vec4f> &lines);
std::vector<std::vector<bool>> createGraph(cv::Mat &linesImg, std::vector<cv::Point2i> &intersections);
int trapezoid_area(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d);
std::vector<cv::Vec8i> findTrapezoids(const std::vector<cv::Point2i>& intersections, const std::vector<std::vector<bool>>& adjacencyMatrix, int minArea, int maxArea);
cv::Mat reduceSaturationBrightness(const cv::Mat& image, float saturationScale, float valueScale);