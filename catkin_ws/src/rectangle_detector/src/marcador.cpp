#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/assign/list_of.hpp>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <functional>
#include <signal.h>
#include <detector/marcador.h>

void Marcador::MarkerPoints(std::vector<cv::Point2f> markerPoints){// save the 4 corners of the marker
    this->puntosMarca= markerPoints;
}
void Marcador::addPoint(cv::Point2f point) { puntosMarca.push_back(point); }
void Marcador::setPoint(cv::Point2f point, int index) { puntosMarca.at(index) = point; }
void Marcador::setContourIdx(unsigned int contourIdx) { this->contourIdx = contourIdx; }
void Marcador::setArea(double area) { this->area = area; }
void Marcador::setAngle(double angle) { this->angle= angle; }
void Marcador::setRotatedRect(cv::RotatedRect rect) { this->rect = rect; }
cv::RotatedRect Marcador::getRotatedRect(void){
    return this->rect;
}

std::vector<cv::Point2f> Marcador::getMarkerPoints(void) { return this->puntosMarca; }
int Marcador:: getNumberOfPoints(){
    return puntosMarca.size();
}
cv::Point2f Marcador::getPoint(int index){
    return this->puntosMarca.at(index);
}

std::vector<cv::Point2f> Marcador::getAllPoints(){
    return this->puntosMarca;
}
 void Marcador::setMapId (int map){// save the map id del marker finding
     this->mapID = map;
 }
 void Marcador::setSectorId(int sector){// save the Sect id del marker finding
     this->sectorID = sector;
 }
void Marcador::setMarkerId (int ID){// save the Marker id del marker finding
   this->markerID=ID;
}
void Marcador::setWidthId (int width){// save the Marker id del marker finding
   this->widthID=width;
}
void Marcador::setHeightId (int height){// save the Marker id del marker finding
   this->heightID=height;
}
int Marcador::getMarkerID(void){
    return this->markerID;
}
int Marcador::getSectorID(void){
    return this->sectorID;
}
int Marcador::getMapID(void){
    return this->mapID;
}
int Marcador::getWidthID(void){
    return this->widthID;
}
int Marcador::getHeightID(void){
    return this->heightID;
}

std::vector<geometry_msgs::Point> Marcador::getPoseWorld(void){
    return this->PositionCorners3d;
}

void Marcador::setPoseWorld(std::vector<geometry_msgs::Point> Position){
    this->PositionCorners3d=Position;
}

void Marcador::setTransformCorners(std::vector<geometry_msgs::Transform> CornersTr){
    this->ToCorners=CornersTr;
}
void Marcador::setCorner(geometry_msgs::Point Corner){
    this->PositionCorners3d.push_back(Corner);
}



std::vector<geometry_msgs::Transform> Marcador::getTransformCorners (void){
    return this->ToCorners;
}

void Marcador::setRelativePose(std::vector<geometry_msgs::Point> Relativa){
    this->ReltoCam=Relativa;

}
std::vector<geometry_msgs::Point> Marcador::getRelativeCoordinates (void){
    return this->ReltoCam;
}







