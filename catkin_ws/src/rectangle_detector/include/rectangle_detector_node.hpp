#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

Mat cameraMatrix, distCoeffs;
Size imageSize;

Point intersection(Vec2f line1, Vec2f line2);
vector<Point> segmented_intersections(vector<vector<Vec2f>> lines);
vector<vector<Vec2f>> segment_by_angle_kmeans(vector<Vec2f> lines, int k = 2,
                                               TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0),
                                               int attempts = 10,
                                               int flags = KMEANS_RANDOM_CENTERS);