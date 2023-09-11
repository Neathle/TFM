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
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
using namespace std;

class Marcador{
    private:
        int mapID;
        int sectorID;
        int markerID;
        int widthID;
        int heightID;
        std::vector<cv::Point2f> puntosMarca;
        int contourIdx;
        double area;
        double angle;
        cv::RotatedRect rect;
        std::vector<geometry_msgs::Point> PositionCorners3d;
        std::vector<geometry_msgs::Transform> ToCorners;
        std::vector<geometry_msgs::Point> ReltoCam;

    public:
        void MarkerPoints(std::vector<cv::Point2f> markerPoints);
        void addPoint(cv::Point2f point);
        void setPoint(cv::Point2f point, int index);
        void setContourIdx(unsigned int contourIdx);
        void setArea(double area);
        void setAngle(double angle);
        std::vector<cv::Point2f> getMarkerPoints(void);
        void setRotatedRect (cv::RotatedRect rect);
        cv::Point2f getPoint(int index);
        int getNumberOfPoints(void);
        std::vector<cv::Point2f> getAllPoints();
        void setMapId (int map);
        void setSectorId(int sector);
        void setMarkerId (int ID);
        void setWidthId (int width);
        void setHeightId (int height);
        cv::RotatedRect getRotatedRect(void);
        int getMapID(void);
        int getSectorID(void);
        int getMarkerID(void);
        int getWidthID(void);
        int getHeightID(void);
        void setPoseWorld(std::vector<geometry_msgs::Point> Position);
        std::vector<geometry_msgs::Point> getPoseWorld (void);
        void setTransformCorners(std::vector<geometry_msgs::Transform> CornersTr);
        std::vector<geometry_msgs::Transform> getTransformCorners (void);
        void setRelativePose(std::vector<geometry_msgs::Point> Relativa);
        std::vector<geometry_msgs::Point> getRelativeCoordinates(void);
        void setCorner(geometry_msgs::Point Corner);

};
