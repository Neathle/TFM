#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <numeric>


class RectangleDetectorNode
{
private:
    ros::Publisher im_pub_lines_;
    ros::Publisher im_pub_rectangles_;
    ros::Publisher im_pub_graph_;
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

    cv::RNG rng_ = cv::RNG(0xFFFFFFFF);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8, 100);
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
    void nonMaximumSuppression();
    void reduceSaturationBrightness(float saturationScale, float valueScale);

    void drawGraph();
    void drawTrapezoids();

    void extractLines();
    void findIntersections();
    void createGraph();
    void extractTrapezoids();

public:
    std::vector<cv::Vec4f> lines;
    std::vector<cv::Point2i> intersections;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<cv::Vec8i> trapezoids;

    cv::Mat image;
    cv::Mat gray;
    // cv::Mat grayFloat;
    // cv::Mat mask;
    cv::Mat linesImg;
    cv::Mat graphImg;


    RectangleDetectorNode();
};

//constructor
RectangleDetectorNode::RectangleDetectorNode()
{

    //receive params
    nh_.getParam("rectangle_detector/image_topic", image_topic_);
    nh_.getParam("rectangle_detector/intersections_detector_mode", intersections_detector_mode_);
    nh_.getParam("rectangle_detector/elongation_factor", elongation_factor_);
    nh_.getParam("rectangle_detector/rects_min_area_factor", rects_min_area_factor_);
    nh_.getParam("rectangle_detector/rects_max_area_factor", rects_max_area_factor_);
    nh_.getParam("rectangle_detector/compactness_threshold", compactness_threshold_);
    nh_.getParam("rectangle_detector/length_diff_threshold", length_diff_threshold_);
    nh_.getParam("rectangle_detector/nms_threshold", nms_threshold_);
    nh_.getParam("rectangle_detector/lineImg_drawing_width", lineImg_drawing_width_);
    nh_.getParam("rectangle_detector/rectImg_drawing_width", rectImg_drawing_width_);
    nh_.getParam("rectangle_detector/draw_graph", draw_graph_);
    nh_.getParam("rectangle_detector/draw_trapezoids", draw_trapezoids_);
    nh_.getParam("rectangle_detector/whiteness_threshold", whiteness_threshold_);
    nh_.getParam("rectangle_detector/angle_threshold", angle_threshold_);

    //initialize publishers and subscribers
    im_pub_lines_ = nh_.advertise<sensor_msgs::Image>("/rectangle_detector/lines", 1);
    im_pub_graph_ = nh_.advertise<sensor_msgs::Image>("/rectangle_detector/graph", 1);

    if (intersections_detector_mode_) {
        im_sub_ = nh_.subscribe(image_topic_, 1, &RectangleDetectorNode::imageCallbackIntersections, this);
    }
    else {
        im_pub_rectangles_ = nh_.advertise<sensor_msgs::Image>("/rectangle_detector/rectangles", 1);
        im_sub_ = nh_.subscribe(image_topic_, 1, &RectangleDetectorNode::imageCallbackTrapezoids, this);
    }
}

// Callback for intersections detector mode
void RectangleDetectorNode::imageCallbackIntersections(const sensor_msgs::ImageConstPtr& msg)
{
    //convert image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        image = cv_ptr->image;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        linesImg = cv::Mat::zeros(gray.size(), CV_8UC1);

        ROS_INFO("Received Image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    extractLines();
    findIntersections();

    //publish images
    im_pub_lines_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", linesImg).toImageMsg());
}

// Callback for trapezoids detection mode
void RectangleDetectorNode::imageCallbackTrapezoids(const sensor_msgs::ImageConstPtr& msg)
{
    //convert image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        image = cv_ptr->image;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        linesImg = cv::Mat::zeros(gray.size(), CV_8UC1);
        graphImg = cv::Mat::zeros(gray.size(), CV_8UC3);

        ROS_INFO("Received Image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    extractLines();
    findIntersections();
    // createGraph();
    // extractTrapezoids();

    //publish images
    im_pub_lines_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", linesImg).toImageMsg());

    if (draw_graph_)
        im_pub_graph_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", graphImg).toImageMsg());
    
    if (draw_trapezoids_)
        im_pub_rectangles_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
}

// Function to increase the length of the lines
void RectangleDetectorNode::elongateLines()
{
    for (cv::Vec4f &line : lines)
    {
        float x1 = line[0];
        float y1 = line[1];
        float x2 = line[2];
        float y2 = line[3];

        float dx = x2 - x1;
        float dy = y2 - y1;

        float extension = (elongation_factor_ - 1) / 2;

        line[0] = x1 - dx * extension;
        line[1] = y1 - dy * extension;
        line[2] = x2 + dx * extension;
        line[3] = y2 + dy * extension;
    }
}

// Function to check if a line is sufficiently white
bool RectangleDetectorNode::isWhiteLine(cv::Point2f p1, cv::Point2f p2)
{
    cv::LineIterator it(linesImg, p1, p2);
    int whiteCount = 0;
    for (int i = 0; i < it.count; i++, ++it)
    {
        if (linesImg.at<uchar>(it.pos()) > 64)
        {
            whiteCount++;
        }
    }
    return (whiteCount >= it.count * whiteness_threshold_);
}

// Function to check if two line segments intersect
bool RectangleDetectorNode::intersect(cv::Vec4f line1, cv::Vec4f line2, cv::Point2i &int_pt)
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

// Function to check if the difference between the shortest line and the longest is less than a threshold
bool RectangleDetectorNode::trapezoidLineTest(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d) 
{
    float ab = cv::norm(a - b);
    float bc = cv::norm(b - c);
    float cd = cv::norm(c - d);
    float da = cv::norm(d - a);

    float min_length = std::min(std::min(ab, bc), std::min(cd, da));
    float max_length = std::max(std::max(ab, bc), std::max(cd, da));

    if (min_length / max_length < length_diff_threshold_) return false;

    return true;
}

//https://fisherzachary.github.io/public/r-output.html
// Function to calculate the area of a trapezoid
bool RectangleDetectorNode::trapezoidAreaTest(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d, float min_area, float max_area) 
{
    float trapezoid_area = std::abs((a.x * b.y - a.y * b.x) + (b.x * c.y - b.y * c.x) + (c.x * d.y - c.y * d.x) + (d.x * a.y - d.y * a.x)) / 2.0;
    // ROS_INFO("Trapezoid area %f, for %d, %d, %d, %d, %d, %d, %d, %d", trapezoid_area, a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y);
    if (trapezoid_area < min_area || trapezoid_area > max_area) return false;
    // find the perimeter
    float perimeter = cv::norm(a - b) + cv::norm(b - c) + cv::norm(c - d) + cv::norm(d - a);

    //Polsby-Popper test
    float pp = 4.0 * CV_PI * trapezoid_area / (perimeter * perimeter);
    // ROS_INFO("Polsby-Popper test %f, Area %f, Perimeter %f", pp, trapezoid_area, perimeter);
    if (pp < compactness_threshold_) return false;
    
    return true;
}

// dot product
float dot(cv::Point2i a, cv::Point2i b)
{
    return a.x * b.x + a.y * b.y;
}

// cross product
float cross(cv::Point2i a, cv::Point2i b)
{
    return a.x * b.y - a.y * b.x;
}

// Function to check if the angles of a trapezoid are sufficiently close to 90 degrees
bool RectangleDetectorNode::trapezoidAnglesTest(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d) 
{
    // atan2(vectorA.y, vectorA.x) - atan2(vectorB.y,  vectorB.x)
    float angle1 = std::atan2(b.y - a.y, b.x - a.x) - std::atan2(c.y - b.y, c.x - b.x);
    if (angle1 < 0) angle1 += 2 * CV_PI;
    angle1 = std::abs(angle1);
    // if angle is not between pi/2-angle_threshold_ and pi/2+angle_threshold_ then return false
    if (angle1 < CV_PI / 2 - angle_threshold_ || angle1 > CV_PI / 2 + angle_threshold_) return false;

    float angle2 = std::atan2(c.y - b.y, c.x - b.x) - std::atan2(d.y - c.y, d.x - c.x);
    if (angle2 < 0) angle2 += 2 * CV_PI;
    angle2 = std::abs(angle2);
    if (angle2 < CV_PI / 2 - angle_threshold_ || angle2 > CV_PI / 2 + angle_threshold_) return false;

    float angle3 = std::atan2(d.y - c.y, d.x - c.x) - std::atan2(a.y - d.y, a.x - d.x);
    if (angle3 < 0) angle3 += 2 * CV_PI;
    angle3 = std::abs(angle3);
    if (angle3 < CV_PI / 2 - angle_threshold_ || angle3 > CV_PI / 2 + angle_threshold_) return false;
    
    // ROS_INFO("Rectangle angles %f %f %f", angle1, angle2, angle3);
    return true;
}

// Function to remove close intersections
void RectangleDetectorNode::nonMaximumSuppression() 
{
    std::vector<cv::Point2i> filteredIntersections;
    std::vector<bool> suppressed(intersections.size(), false);
    for (int i = 0; i < intersections.size(); ++i) {
        if (suppressed[i]) continue;

        filteredIntersections.push_back(intersections[i]);
        std::vector<cv::Point2i> closeIntersections;
        for (int j = i + 1; j < intersections.size(); ++j) {
            if (cv::norm(intersections[i] - intersections[j]) < nms_threshold_) {
                suppressed[j] = true;
                closeIntersections.push_back(intersections[j]);
            }
        }

        if (closeIntersections.size() > 0) {
            cv::Point2i avgIntersection = std::accumulate(closeIntersections.begin(), closeIntersections.end(), intersections[i]);
            avgIntersection.x /= closeIntersections.size() + 1;
            avgIntersection.y /= closeIntersections.size() + 1;
            filteredIntersections.back() = avgIntersection;
        }
    }

    intersections = filteredIntersections;
}

// Function to reduce the saturation and brightness of an image
void RectangleDetectorNode::reduceSaturationBrightness(float saturationScale, float valueScale)
{
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsvImage, channels);

    channels[1] *= saturationScale;
    channels[2] *= valueScale;

    cv::merge(channels, hsvImage);
    cv::cvtColor(hsvImage, image, cv::COLOR_HSV2BGR);
}

// Function to draw the Graph
void RectangleDetectorNode::drawGraph()
{
    for (int i = 0; i < adjacencyMatrix_.size(); i++)
    {
        for (int j = i + 1; j < adjacencyMatrix_.size(); j++)
        {
            if (adjacencyMatrix_[i][j])
            {
                cv::line(graphImg, intersections[i], intersections[j], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
    }

    for (const cv::Point2i &intersection : intersections)
    {
        cv::circle(graphImg, intersection, 2, cv::Scalar(255, 255, 255), -1);
    }
}

// Function to draw the trapezoids
void RectangleDetectorNode::drawTrapezoids()
{   
    reduceSaturationBrightness(0.5f, 0.5f);
    cv::Scalar color;
    for (const auto &trapezoid : trapezoids)
    {
        color = cv::Scalar(rng_.uniform(0, 255), rng_.uniform(0, 255), rng_.uniform(0, 255));
        cv::line(image, cv::Point(trapezoid[0], trapezoid[1]), cv::Point(trapezoid[2], trapezoid[3]), color, rectImg_drawing_width_, cv::LINE_AA);
        cv::line(image, cv::Point(trapezoid[2], trapezoid[3]), cv::Point(trapezoid[4], trapezoid[5]), color, rectImg_drawing_width_, cv::LINE_AA);
        cv::line(image, cv::Point(trapezoid[4], trapezoid[5]), cv::Point(trapezoid[6], trapezoid[7]), color, rectImg_drawing_width_, cv::LINE_AA);
        cv::line(image, cv::Point(trapezoid[6], trapezoid[7]), cv::Point(trapezoid[0], trapezoid[1]), color, rectImg_drawing_width_, cv::LINE_AA);
    }
}


// Function to extract lines from an image using FLD
void RectangleDetectorNode::extractLines()
{
    // Remove noise by blurring with a Gaussian filter
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    // Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    cv::Sobel(gray, grad_x, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    cv::Sobel(gray, grad_y, CV_8U, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, linesImg);

    // linesImg.convertTo(mask, CV_32F);
    // gray.convertTo(grayFloat, CV_32F);
    // mask.setTo(cv::Scalar(255.0));

    // cv::multiply(mask,grayFloat,grayFloat);
    // grayFloat = grayFloat / 255.0;
    // grayFloat.convertTo(linesImg, CV_8UC1);
}

// Function to find all the intersection points between a set of lines
void RectangleDetectorNode::findIntersections()
{
    // orb->detect(gray,keypoints,linesImg);
    // orb->compute(gray,keypoints,descriptors);
    orb->detectAndCompute(gray,linesImg,keypoints,descriptors);
    cv::cvtColor(gray,graphImg,cv::COLOR_GRAY2BGR);
    cv::drawKeypoints(graphImg, keypoints, graphImg, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    ROS_INFO("Found %lu intersections", keypoints.size());
}

// Function to create a graph where nodes are intersection points and edges are lines that join them
void RectangleDetectorNode::createGraph()
{
    adjacencyMatrix_.clear();
    adjacencyMatrix_.resize(intersections.size(), std::vector<bool>(intersections.size(), false));
    
    for (int i = 0; i < intersections.size(); ++i) {
        for (int j = i + 1; j < intersections.size(); ++j) {
            if (isWhiteLine(intersections[i], intersections[j])) {
                adjacencyMatrix_[i][j] = true;
                adjacencyMatrix_[j][i] = true;
            }
        }
    }
    
    if (draw_graph_)
        drawGraph();

    // ROS_INFO("Number of valid lines: %d", 
    //     std::accumulate(adjacencyMatrix_.begin(), adjacencyMatrix_.end(), 0, [](int sum, const std::vector<bool>& v) {
    //         return sum + std::accumulate(v.begin(), v.end(), 0);
    //     }) / 2
    // );
}

// Function to extract all possible trapezoids from a graph
void RectangleDetectorNode::extractTrapezoids()
{
    trapezoids.clear();
    int min_area = rects_min_area_factor_ * image.cols * image.rows;
    int max_area = rects_max_area_factor_ * image.cols * image.rows;

    for (int i = 0; i < intersections.size(); ++i) {
        for (int j = i + 1; j < intersections.size(); ++j) {
            if (!adjacencyMatrix_[i][j]) continue;

            for (int k = j + 1; k < intersections.size(); ++k) {
                if (!adjacencyMatrix_[j][k]) continue;

                for (int l = k + 1; l < intersections.size(); ++l) {
                    if (!adjacencyMatrix_[k][l] || !adjacencyMatrix_[l][i]) continue;
                    
                    if (!trapezoidLineTest(intersections[i], intersections[j], intersections[k], intersections[l])) continue;
                    if (!trapezoidAreaTest(intersections[i], intersections[j], intersections[k], intersections[l], min_area, max_area)) continue;
                    if (!trapezoidAnglesTest(intersections[i], intersections[j], intersections[k], intersections[l])) continue;
                    // if (!cv::isContourConvex(std::vector<cv::Point2i>{intersections[i], intersections[j], intersections[k], intersections[l]})) continue;
                    trapezoids.emplace_back(cv::Vec8i(intersections[i].x, intersections[i].y,
                                                    intersections[j].x, intersections[j].y,
                                                    intersections[k].x, intersections[k].y,
                                                    intersections[l].x, intersections[l].y));
                }
            }
        }
    }

    if (draw_trapezoids_)
        drawTrapezoids();
    
    ROS_INFO("Found %lu trapezoids", trapezoids.size());
}