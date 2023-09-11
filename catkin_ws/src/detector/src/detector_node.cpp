#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include "detector/arucoMarker.h"
#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/CameraInfo.h>
#include <detector/marker.h>
#include <detector/messagedet.h>
#include <detector/marcador.h>
#include "detector/pixels_corners.h"
#include "detector/pixels_cloud.h"
#include "detector/num_markers.h"

static const std::string OPENCV_WINDOW_2 = "Cloud Image";

class Detecter
{
    public: ros::NodeHandle nh_;
    private: ros::NodeHandle private_nh_;
    private: ros::Subscriber camera_Info_;
    private: ros::Publisher detected_markers_, pub_num_markers_;
    private: ros::Subscriber sub_pixels_cloud;
    private: image_transport::ImageTransport it_;
    private: image_transport::Subscriber image_sub_;
    private: image_transport::Publisher image_pub_;
    private: XmlRpc::XmlRpcValue marker_list;
    private: std::vector<int> IDs,sectors,maps;
    private: FindArucoMarker findArucoMarker; // arucoMarker.h
    private: FindArucoMarker *ptr_aruco_ = &findArucoMarker;
    public: Detecter();
    public: ~Detecter();
    private: void imageCb(const sensor_msgs::ImageConstPtr& msg);
    private: void infoCameraCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);// read camera infomation topic
    private: void pixelsCloaudCallBack(const detector::pixels_cloud::ConstPtr& msg);// show the projection of pointscloud in the image
};

int main(int argc, char **argv){
    ros::init(argc, argv, "DetectorNode");
	Detecter ic;
	ros::spin();
    return 0;
}

Detecter::Detecter() : it_(nh_), private_nh_("~")// constructor
    {
        // Subscrive to input video feed and publish output video feed
        std::string camera_image_input, camera_image_output, camera_info, amcl_pixels_clouds;
        private_nh_.getParam("camera_image_input", camera_image_input);
        private_nh_.getParam("camera_image_output", camera_image_output);
        private_nh_.getParam("camera_info", camera_info);
        private_nh_.getParam("amcl_pixels_clouds", amcl_pixels_clouds);
        private_nh_.getParam("marker_positions", marker_list);

        image_sub_ = it_.subscribe(camera_image_input, 1, &Detecter::imageCb, this);
        image_pub_ = it_.advertise(camera_image_output, 1);
        camera_Info_ = nh_.subscribe(camera_info, 1, &Detecter::infoCameraCallback, this);
        sub_pixels_cloud = nh_.subscribe(amcl_pixels_clouds, 1, &Detecter::pixelsCloaudCallBack, this);
        detected_markers_ = nh_.advertise<detector::messagedet>("detected_markers", 1);
        pub_num_markers_ = nh_.advertise<detector::num_markers>("num_detecte_markers", 10);
        
        for(int i=0;i<marker_list.size();i++){
            IDs.push_back(marker_list[i]["ID"]);
            sectors.push_back(marker_list[i]["sector"]);
            maps.push_back(marker_list[i]["map"]);
        }
        
        cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(OPENCV_WINDOW_2, cv::WINDOW_AUTOSIZE);
        std::cout << "waiting for info camera topic ..." << std::endl;
    }

Detecter::~Detecter()// destructor
{
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW_2);
}

void Detecter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    detector::num_markers  msg_num_markers;
    if(ptr_aruco_->ok_info_camera){
        //it will enter if it find at least one aruco marker
        if(findArucoMarker.findMark(cv_ptr->image)){
            detector::messagedet msg_det;
            for(int i=0; i<ptr_aruco_->ids.size(); i++){//ids.size = numbers of detected markers 
                detector::marker detected;
                for(int j=0; j<4; j++){
                    geometry_msgs::Point32 corner;
                    corner.x=(float)ptr_aruco_->corners[i][j].x;
                    corner.y=(float)ptr_aruco_->corners[i][j].y;
                    corner.z=0.0;
                    detected.Corners.push_back(corner);
                }
                for(int j = 0; j<marker_list.size(); j++){
                    if(ptr_aruco_->ids[i] == IDs[j]){
                        detected.ID.data=uint8_t(IDs[j]);
                        detected.map.data=uint8_t(sectors[i]); // it is not neccesary this info
                        detected.sector.data=uint8_t(maps[i]);
                        msg_det.DetectedMarkers.push_back(detected);
                    }
                }
            }
            msg_det.header.frame_id="camera_link";
            msg_det.header.stamp=ros::Time::now();
            if(msg_det.DetectedMarkers.size()>0){
                detected_markers_.publish(msg_det); //publish topic with detected marker
            }
            msg_num_markers.number.data = uint8_t(msg_det.DetectedMarkers.size());
               
        } else {
            msg_num_markers.number.data = uint8_t(0);
        }
        pub_num_markers_.publish(msg_num_markers);
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg()); // publish topic with image
    }
}

void Detecter::infoCameraCallback(const sensor_msgs::CameraInfoConstPtr& cam_info){
    if(!ptr_aruco_->ok_info_camera){   
        ptr_aruco_->width_image = cam_info->width;
        ptr_aruco_->height_image = cam_info->height;
        ptr_aruco_->ok_info_camera = true;
        std::cout << "cameraInfo received -> width: " << ptr_aruco_->width_image << " , height: " << ptr_aruco_->height_image << " (in Pixels)" << std::endl;
    }
}

void Detecter::pixelsCloaudCallBack(const detector::pixels_cloud::ConstPtr &msg){

    if(ptr_aruco_->ok_info_camera){
        detector::pixels_corners corners;
        std::cout << "size pixels cloud: " << msg->pixels_cloud.size();
        for(int i=0; i<msg->pixels_cloud.size(); i++){
            corners = msg->pixels_cloud[i];
            cv::Scalar color;
            for(int j=0; j<corners.pixels_corners.size(); j++){
                int corner_w = int(corners.pixels_corners[j].x);
                int corner_h = int(corners.pixels_corners[j].y);
                if(corner_w > 0  &&  corner_w < ptr_aruco_->width_image  &&  corner_h > 0  &&  corner_h < ptr_aruco_->height_image){
                    if(j==0){
                        color = cv::Scalar(0, 255, 0); // green circle corner 0
                    } else if (j==1){
                        color = cv::Scalar(255, 0, 0); // blue circle corner 1
                    } else if (j==2){
                        color = cv::Scalar(0, 0, 255); // red circle corner 2
                    } else {
                        color = cv::Scalar(0, 255, 255); // yellow circle corner 3
                    }
                    cv::circle(ptr_aruco_->cloud_image, {corner_w, corner_h}, 1, color, 1, LINE_4);
                }
            }
        }
        cv::putText(ptr_aruco_->cloud_image, ("Num of particles is " + std::to_string(msg->pixels_cloud.size())), {10, 30}, 2, 1, cv::Scalar(0, 0, 255));
        cv::imshow(OPENCV_WINDOW_2, ptr_aruco_->cloud_image);//show image
        cv::waitKey(3);
    }
}