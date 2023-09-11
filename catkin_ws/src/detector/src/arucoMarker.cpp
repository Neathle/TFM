#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>
#include <detector/arucoMarker.h>

bool FindArucoMarker::findMark(cv::Mat imageRGB){
    //std::cout << "findMark" << std::endl;
    bool ok_detected_markers = false;
	std::vector<std::vector<cv::Point2f> > corners_aux; // frame ( width, height)
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(imageRGB, dictionary, corners_aux, this->ids);
    this->corners = corners_aux;		
    //if at least one marker detected
    if (ids.size() > 0){
        cv::aruco::drawDetectedMarkers(imageRGB, corners_aux, this->ids);
        // change of frame from left_top_corner to center camera
        std::cout << "####### Detected Markers #######" << std::endl;
        for(int numMarkers=0; numMarkers<ids.size(); numMarkers++){
            //std::cout << corners_aux[numMarkers] << std::endl;
            std::cout << "Marker " << numMarkers  << " -> id: " << ids[numMarkers] << std::endl;  
            for (int j=0; j<4; j++){
                // red rectangle corner 0
                if(j==1){
                    cv::circle(imageRGB, {corners_aux[numMarkers][j].x, corners_aux[numMarkers][j].y}, 2, cv::Scalar(255, 0, 0), 2);// blue circle corner 1
                } else if(j==2){
                    cv::circle(imageRGB, {corners_aux[numMarkers][j].x, corners_aux[numMarkers][j].y}, 2, cv::Scalar(0, 0, 255), 2);// red circle corner 2
                } else if(j==3){
                    cv::circle(imageRGB, {corners_aux[numMarkers][j].x, corners_aux[numMarkers][j].y}, 2, cv::Scalar(0, 255, 255), 2);// yellow circle corner 3
                }
            }
            std::cout << "" << std::endl;
        }
        ok_detected_markers = true;
    }else{
        std::cout << "no markers detected" << std::endl;
    }
    // Update GUI Window
    /*cv::circle(imageRGB, {this->width_image/2, this->height_image/2}, 4, cv::Scalar(255, 0, 0), 8);
    cv::arrowedLine(imageRGB,{this->width_image/2, this->height_image/2}, {this->width_image/4, this->height_image/2},cv::Scalar(0, 0, 255)); // X frame
    cv::arrowedLine(imageRGB,{this->width_image/2, this->height_image/2}, {this->width_image/2, 3*(this->height_image/4)},cv::Scalar(0, 255, 0)); // Y frame
    cv::putText(imageRGB, "X", {(this->width_image/2)-50, (this->height_image/2)-10}, 1, 1, cv::Scalar(0, 0, 255));
    cv::putText(imageRGB, "y", {(this->width_image/2)+10, (this->height_image/2)+50}, 1, 1, cv::Scalar(0, 255, 0));*/
    cv::imshow(OPENCV_WINDOW, imageRGB);//show image
    imageRGB.copyTo(this->cloud_image);
    cv::waitKey(3);
    return ok_detected_markers;
}	