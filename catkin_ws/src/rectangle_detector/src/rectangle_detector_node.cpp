#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "rectangle_detector_node.hpp"

using namespace cv;
using namespace std;

// void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
// {
//     imageSize = Size(msg->width, msg->height);
//     cameraMatrix = Mat(3, 3, CV_64F);
//     for (int i = 0; i < 9; i++)
//         cameraMatrix.at<double>(i / 3, i % 3) = msg->K[i];
//     distCoeffs = Mat(1, 5, CV_64F);
//     for (int i = 0; i < 5; i++)
//         distCoeffs.at<double>(i) = msg->D[i];
// }

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received Image");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat image = cv_ptr->image;

    // // Undistort the image
    // Mat undistorted;
    // undistort(image, undistorted, cameraMatrix, distCoeffs);

    // Convert to grayscale
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // Find edges
    Mat edges;
    Canny(gray, edges, 50, 150, 3, false);
    // imshow("Image", edges);
    // waitKey();
    // Create an instance of the LineSegmentDetector class
    cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();

    // Detect the lines
    vector<Vec4i> lines;
    lsd->detect(edges, lines);
    
    ROS_INFO("Found %lu lines", lines.size());
    // vector<vector<Vec2f>> segmented_lines = segment_by_angle_kmeans(lines);
    // vector<Point> intersections = segmented_intersections(segmented_lines);

    Mat cdst = Mat::zeros(edges.size(), CV_8UC1);
    cv::RNG rng(12345);
    cv::Scalar color = cv::COLOR_WHITE;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        // cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), color, 1, LINE_AA);
    }

    // Find polygons in the cdst image
    vector<vector<Point>> contours;
    findContours(cdst, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    ROS_INFO("Found %lu contours", contours.size());

    // Find polygons with four corners
    vector<vector<Point>> polygons;
    // get number of pixels in cdst
    int cdst_area = cdst.rows * cdst.cols;
    for (const auto& contour : contours)
    {
        vector<Point> approx;
        approxPolyDP(contour, approx, arcLength(contour, true) * 0.02, true);
        if (approx.size() == 4 && isContourConvex(approx) && contourArea(approx)/cdst_area > 0.001 && contourArea(approx)/cdst_area < 0.4)
            polygons.push_back(approx);
    }

    // Draw polygons
    for (const auto& polygon : polygons)
    {
        // Draw corners
        for (const auto& point : polygon)
            circle(image, point, 3, Scalar(0, 255, 0), -1);

        // Draw edges
        for (size_t i = 0; i < polygon.size(); i++)
            line(image, polygon[i], polygon[(i + 1) % polygon.size()], Scalar(0, 255, 0), 1);
    }
    
    


    // Group line segments into sets of four that form a rectangle
    // vector<vector<Point>> rectangles;
    // rectangles.push_back(intersections);
    // for (size_t i = 0; i < segmented_lines.size(); i++) {
    //     ROS_INFO("Processing line %lu", i);
    //     for (size_t j = i + 1; j < segmented_lines.size(); j++)
    //         for (size_t k = j + 1; k < segmented_lines.size(); k++)
    //             for (size_t l = k + 1; l < segmented_lines.size(); l++)
    //             {   
                    
    //                 vector<Vec2f> lines_subset = {segmented_lines[i], segmented_lines[j], segmented_lines [k], segmented_lines[l]};
                    
                    

    //                 rectangles.push_back(intersections);
    //             }}

    // Draw rectangles
    // for (const auto& rectangle : rectangles)
    // {
    //     // Draw corners
    //     for (const auto& point : rectangle)
    //         circle(image, point, 3, Scalar(0, 255, 0), -1);

    //     // // Draw edges
    //     // for (size_t i = 0; i < rectangle.size(); i++)
    //     //     line(image, rectangle[i], rectangle[(i + 1) % rectangle.size()], Scalar(0, 255, 0), 1);
    // }

    imshow("Image", image);
    waitKey(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectangle_detector");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("/image_publisher/image_raw", 1, imageCallback);
    // ros::Subscriber sub2 = nh.subscribe("/camera/camera_info", 1, cameraInfoCallback);

    namedWindow("Image");

    while (ros::ok())
        ros::spinOnce();

    return 0;
}

Point intersection(Vec2f line1, Vec2f line2)
{
    // Find the intersection of two lines given in Hesse normal form
    float rho1 = line1[0], theta1 = line1[1];
    float rho2 = line2[0], theta2 = line2[1];
    Mat A = (Mat_<float>(2, 2) << cos(theta1), sin(theta1), cos(theta2), sin(theta2));
    Mat b = (Mat_<float>(2, 1) << rho1, rho2);
    Mat x;
    solve(A, b, x, DECOMP_LU);
    Point pt(round(x.at<float>(0)), round(x.at<float>(1)));
    return pt;
}

vector<Point> segmented_intersections(vector<vector<Vec2f>> lines)
{
    // Find the intersections between groups of lines
    vector<Point> intersections;
    for (size_t i = 0; i < lines.size() - 1; i++)
        for (size_t j = i + 1; j < lines.size(); j++)
            for (const auto& line1 : lines[i])
                for (const auto& line2 : lines[j])
                    intersections.push_back(intersection(line1, line2));
    return intersections;
}

vector<vector<Vec2f>> segment_by_angle_kmeans(vector<Vec2f> lines, int k, TermCriteria criteria, int attempts, int flags)
{
    // Group lines based on angle with k-means
    vector<float> angles;
    for (const auto& line : lines)
        angles.push_back(line[1]);
    Mat pts(angles.size(), 2, CV_32F);
    for (size_t i = 0; i < angles.size(); i++)
    {
        pts.at<float>(i, 0) = cos(2 * angles[i]);
        pts.at<float>(i, 1) = sin(2 * angles[i]);
    }
    Mat labels, centers;
    kmeans(pts, k, labels, criteria, attempts, flags, centers);

    // Segment lines based on their k-means label
    map<int, vector<Vec2f>> segmented;
    for (size_t i = 0; i < lines.size(); i++)
        segmented[labels.at<int>(i)].push_back(lines[i]);
    vector<vector<Vec2f>> result;
    for (const auto& item : segmented)
        result.push_back(item.second);
    return result;
}