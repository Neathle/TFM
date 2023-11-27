#include "rectangle_detector_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_detector");

    RectangleDetectorNode node;

    ros::spin();
}

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
    nh_.getParam("rectangle_detector/execution_period", execution_period_);

    //initialize publishers and subscribers
    im_pub_lines_ = nh_.advertise<sensor_msgs::Image>("/rectangle_detector/lines", 1);
    im_pub_graph_ = nh_.advertise<sensor_msgs::Image>("/rectangle_detector/graph", 1);
    detections_pub_ = nh_.advertise<detector::messagedet>("/rectangle_detector/detections", 10);
    detections_num_pub_ = nh_.advertise<detector::num_markers>("/rectangle_detector/num_detections", 10);

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
    ros::Time now = ros::Time::now();
    if ((now - last_execution_).toSec() < execution_period_) return;
    last_execution_ = now;

    //convert image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        image = cv_ptr->image;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        linesImg = cv::Mat::zeros(gray.size(), CV_8UC1);

        image_stamp_ = msg->header.stamp;

        // ROS_INFO("Received Image");
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
    ros::Time now = ros::Time::now();
    if ((now - last_execution_).toSec() < execution_period_) return;
    last_execution_ = now;
    
    //convert image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        image = cv_ptr->image;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        linesImg = cv::Mat::zeros(gray.size(), CV_8UC1);
        graphImg = cv::Mat::zeros(gray.size(), CV_8UC3);

        // ROS_INFO("Received Image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    extractLines();
    findIntersections();
    createGraph();
    extractTrapezoids();
    publishDetections();

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

    // Check if line AB is mostly horizontal or vertical
    if (std::abs(std::atan2(b.y - a.y, b.x - a.x)) < angle_threshold_ 
     || std::abs(std::atan2(b.y - a.y, b.x - a.x) - CV_PI) < angle_threshold_)
        return true;

    // Check if line BC is mostly horizontal or vertical
    if (std::abs(std::atan2(c.y - b.y, c.x - b.x)) < angle_threshold_ 
     || std::abs(std::atan2(c.y - b.y, c.x - b.x) - CV_PI) < angle_threshold_)
        return true;

    // Check if line CD is mostly horizontal or vertical
    if (std::abs(std::atan2(d.y - c.y, d.x - c.x)) < angle_threshold_ 
     || std::abs(std::atan2(d.y - c.y, d.x - c.x) - CV_PI) < angle_threshold_)
        return true;

    // Check if line DA is mostly horizontal or vertical
    if (std::abs(std::atan2(a.y - d.y, a.x - d.x)) < angle_threshold_ 
     || std::abs(std::atan2(a.y - d.y, a.x - d.x) - CV_PI) < angle_threshold_)
        return true;
    
    // In fo linesare mostly horizontal nor vertical, the test fails
    return false;
}


// Function to remove close intersections
void RectangleDetectorNode::nonMaximumSuppressionPoints() 
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
    cv::normalize(gray, gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    lines.clear();

    fld->detect(gray, lines);

    elongateLines();

    // Paint the lines on the black image
    for (const cv::Vec4f &line : lines)
    {
        cv::line(linesImg, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), lineImg_drawing_width_, cv::LINE_AA);
    }

    // Gaussian blur
    cv::GaussianBlur(linesImg, linesImg, cv::Size(5,5), 0, 0);

    // ROS_INFO("Extracted %lu lines", lines.size());
}


// Function to find all the intersection points between a set of lines
void RectangleDetectorNode::findIntersections()
{
    intersections.clear();
    for (size_t i = 0; i < lines.size(); i++)
    {
        for (size_t j = i + 1; j < lines.size(); j++)
        {
            cv::Point2i intersection;
            if (intersect(lines[i], lines[j], intersection))
                intersections.push_back(intersection);
        }
    }

    if (nms_threshold_ > 1)
        nonMaximumSuppressionPoints();

    // ROS_INFO("Found %lu intersections", intersections.size());
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
                    if(trapezoids.size() >= 50) goto endloop;
                }
            }
        }
    }
    endloop:
    if (draw_trapezoids_)
        drawTrapezoids();
    
    // ROS_INFO("Found %lu trapezoids", trapezoids.size());
}

void RectangleDetectorNode::publishDetections()
{
    // Header header
    // marker[] DetectedMarkers
    
    // geometry_msgs/Point32[] Corners
    // std_msgs/UInt8 map
    // std_msgs/UInt8 sector
    // std_msgs/UInt8 ID

    // Header header
    // std_msgs/UInt32 number

    detector::messagedet msg;
    detector::num_markers msg_num;

    msg.header.stamp = image_stamp_;
    msg.header.frame_id = "xtion_rgb_optical_frame";
    msg.header.seq = 0;
    msg_num.header.frame_id = "xtion_rgb_optical_frame";
    msg_num.header.seq = 0;
    msg_num.header.stamp = image_stamp_;

    msg_num.number.data = trapezoids.size();

    if (!trapezoids.size()) return;

    ROS_INFO("Found %lu trapezoids", (long unsigned int)msg_num.number.data);

    for(const auto& trapezoid : trapezoids)
    {
        detector::marker marker;

        marker.map.data = 0;
        marker.sector.data = 0;
        marker.ID.data  = 0;
        marker.Corners.resize(4);

        for(int j = 0; j < 4; j++)
        {
            geometry_msgs::Point32 corner;
            corner.x = trapezoid[j*2];
            corner.y = trapezoid[j*2 + 1];
            corner.z = 0.0;
            
            marker.Corners[j] = corner;
        }

        //TODO: Match rectangles to the map

        msg.DetectedMarkers.push_back(marker);
    }

    // // Print the complete message to ROS info after the loop
    // std::stringstream ss;
    // ss << "Complete message: number=" << (long unsigned int)msg_num.number.data
    //    << ", markers: [";
    // for(const auto& marker : msg.DetectedMarkers)
    // {
    //     ss << "map=" << (int)marker.map.data
    //        << ", sector=" << (int)marker.sector.data
    //        << ", ID=" << (int)marker.ID.data
    //        << ", corners=";
    //     for(const auto& corner : marker.Corners)
    //     {
    //         ss << "(" << corner.x << ", " << corner.y << ", " << corner.z << "), ";
    //     }
    //     ss << "; ";
    // }
    // ss << "]";
    // ROS_INFO_STREAM(ss.str());

    detections_pub_.publish(msg);
    detections_num_pub_.publish(msg_num);

}
