#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <numeric>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <detector/marker.h>
#include <detector/messagedet.h>
#include <detector/num_markers.h>

class RectangleDetectorNode
{
private:
    ros::Publisher im_pub_lines_;
    ros::Publisher im_pub_rectangles_;
    ros::Publisher im_pub_graph_;
    ros::Publisher detections_pub_;
    ros::Publisher detections_num_pub_;
    ros::NodeHandle nh_;
    ros::Subscriber im_sub_;

    std::string image_topic_ = "/image_publisher/image_raw";
    bool intersections_detector_mode_ = false;
    bool draw_graph_ = true;
    bool draw_trapezoids_ = true;
    float elongation_factor_ = 1.2;
    float rects_min_area_factor_ = 0.001;
    float rects_max_area_factor_ = 0.3;
    float compactness_threshold_ = 0.5;
    float length_diff_threshold_ = 0.1;
    int nms_threshold_ = 1;
    float angle_threshold_ = 0.1;
    int lineImg_drawing_width_ = 2;
    int rectImg_drawing_width_ = 2;
    float whiteness_threshold_ = 0.95;
    int max_num_trapezoids_ = 50;
    ros::Time image_stamp_; 
    ros::Time last_execution_ = ros::Time::now();
    double execution_period_ = 0.1;

    cv::RNG rng_ = cv::RNG(0xFFFFFFFF);
    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
        8, //length_threshold
        1.41421356f, //distance_threshold
        50, //canny_th1
        150, //canny_th2
        3, //canny_aperture_size
        false //do_merge
    );
    std::vector<std::vector<bool>> adjacencyMatrix_;

    void imageCallbackTrapezoids(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackIntersections(const sensor_msgs::ImageConstPtr& msg);

    void elongateLines();
    bool isWhiteLine(cv::Point2f p1, cv::Point2f p2);
    bool intersect(cv::Vec4f line1, cv::Vec4f line2, cv::Point2i &int_pt);
    bool trapezoidLineTest(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d);
    bool trapezoidAreaTest(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d, float min_area, float max_area);
    bool trapezoidAnglesTest(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d);
    bool isConvexTrapezoid(cv::Vec8i trapezoid); //TODO: implement
    void nonMaximumSuppressionPoints();
    void reduceSaturationBrightness(float saturationScale, float valueScale);

    void drawGraph();
    void drawTrapezoids();

    void extractLines();
    void findIntersections();
    void createGraph();
    void extractTrapezoids();
    void publishDetections();

public:
    std::vector<cv::Vec4f> lines;
    std::vector<cv::Point2i> intersections;
    std::vector<cv::Vec8i> trapezoids;

    cv::Mat image;
    cv::Mat gray;
    cv::Mat linesImg;
    cv::Mat graphImg;

    
    RectangleDetectorNode();
};

