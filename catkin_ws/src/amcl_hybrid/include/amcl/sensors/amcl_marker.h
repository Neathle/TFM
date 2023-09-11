/*
* 
*/

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc:  Camera model for AMCL
// Author: Patricia Javierre
// Date: 17 Aug 2003
// CVS: $Id: amcl_laser.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_MARKER_H
#define AMCL_MARKER_H

#include <vector>
#include "amcl_sensor.h"
#include "../map/map.h"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <detector/marcador.h>
#include <opencv2/ccalib/omnidir.hpp>
#include <amcl_hybrid/pixels_cloud.h>
//#include <amcl_hybrid/fail_marker.h>
#include "ros/ros.h"

using namespace std;

//double height_pos_camera_link_;

namespace amcl
{

typedef enum
{
  MARKER_MODEL_LIKELIHOOD
} marker_model_t;

// Laser sensor data
class AMCLMarkerData : public AMCLSensorData
{
  public:
    AMCLMarkerData () {markers_obs;};
    virtual ~AMCLMarkerData() {markers_obs.clear();};
  // vector of detected Markers
  public: std::vector<Marcador> markers_obs;
  //public: double range_max;
 // public: double (*ranges)[2];
};


// Marker sensor model
class AMCLMarker : public AMCLSensor
{
  // Default constructor
  public: AMCLMarker(int simulation);

  public: virtual ~AMCLMarker();

  public: void SetModelLikelihoodField(double z_hit,
                                       double z_rand,
                                       double sigma_hit,
                                       double landa,
                                       double marker_coeff);


  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Determine the probability for the given pose
  private: static double ObservationLikelihood(AMCLMarkerData *data,
                                              pf_sample_set_t* set);
  private:std::vector<geometry_msgs::Point> CalculateRelativePose (Marcador Marca, geometry_msgs::Pose CamaraMundo);
  private: void LoadCameraInfo(void);
  //#%
  private: bool okSetupCameraInfo = true;
  public: void LoadCameraInfo2(const sensor_msgs::CameraInfoConstPtr& cam_info);

  private: std::vector<cv::Point2d> projectPoints(std::vector<geometry_msgs::Point> cam_center_coord);
  private:std::vector<float> calculateError(std::vector<cv::Point2f> projection_detected, std::vector<cv::Point2d> projection_map);
  public: marker_model_t model_type;

  // Current data timestamp
  private: double time;
  public: float  marker_width, num_cam,marker_height,image_width,image_height, height_center_camera;

  // Pixels cloud
  public: amcl_hybrid::pixels_cloud send_pixels_cloud;
  public: amcl_hybrid::pixels_corners send_pixels_corners;

  // Pub_Fail_Marker
  public: ros::Publisher pub_coeff_marker, pub_marker_error;

  // The marker map
  public: std::vector<Marcador> map;

  //Camera parameters
  public:std::vector<geometry_msgs::TransformStamped> tf_cameras;
  public:image_geometry::PinholeCameraModel pin_model;
  public:sensor_msgs::CameraInfo cam_inf_ed;
  private: cv::Mat camMatrix, distCoeff;
  double xi;

  //temp data that is kept before observations are integrated to each particle (requried for beam skipping)
  private: int max_samples;
  private: int max_obs;
  public: std::vector<Marcador> temp_obs;
  public: Mat image_filter;
  public: int simulation;

  // Marker model params
  //
  // Mixture params for the components of the model; must sum to 1
  private: double z_hit;
  private: double z_short;
  private: double z_max;
  private: double z_rand;
  private: double marker_coeff;
  //
  // Stddev of Gaussian model for marker hits.
  private: double sigma_hit;
  // Decay rate of exponential model for short readings.
  private: double lambda_short;
  // Threshold for outlier rejection (unused)
  private: double chi_outlier;
   //Landa for exponential model of marker hits.
  private:double landa;
};


}

#endif
