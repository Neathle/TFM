#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>

static const std::string OPENCV_WINDOW = "RGB Image";

class FindArucoMarker{
    public: bool ok_info_camera = false;
	private: float sizeMarker = 0.5;
    public: cv::Mat camera_matrix;
    public: cv::Mat dist_coeffs;
    public: float width_image, height_image;
    public: std::vector<int> ids;
	public: std::vector<std::vector<cv::Point2f> > corners;
    public: cv::Mat cloud_image;
    public: bool findMark(cv::Mat imageRGB); // devuelve true si ha encontrado mínimo un marker
};