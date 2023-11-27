/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/sensors/amcl_marker.h"
#include "detector/messagedet.h"
#include "detector/marker.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <amcl_hybrid/pose_error.h>
#include <amcl_hybrid/pixels_cloud.h>
#include <amcl_hybrid/coeff_sensor.h>
#include <amcl_hybrid/marker_error.h>

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl_hybrid/AMCLConfig.h"

// Allows AMCL to run from bag file
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

// TODO: Replace with PoseWithCovarianceStamped
//  Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double normalize(double z)
{
  return atan2(sin(z), cos(z));
}

static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0)
    d2 *= -1.0;
  if (fabs(d1) < fabs(d2))
    return (d1);
  else
    return (d2);
}

static const std::string scan_topic_ = "scan";

class AmclNode
{
public:
  AmclNode();
  ~AmclNode();

  /**
   * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
   */
  void runFromBag(const std::string& in_bag_fn);

  int process();
  void savePoseToServer();

private:
  tf::TransformBroadcaster* tfb_;

  // Use a child class to get access to tf2::Buffer class inside of tf_
  struct TransformListenerWrapper : public tf::TransformListener
  {
    inline std::shared_ptr<tf2_ros::Buffer> getBuffer()
    {
      return tf2_buffer_ptr_;
    }  // cambiado por mi: https://github.com/moriarty/navigation/commit/ae060c92a423783c45ef35005ec443e5736c6689
  };

  TransformListenerWrapper* tf_;

  bool sent_first_transform_;

  tf::Transform latest_tf_;
  bool latest_tf_valid_;

  // Pose-generating function used to uniformly distribute particles over
  // the map
  static pf_vector_t uniformPoseGenerator(void* arg);

#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int> > free_space_indices;
#endif
  // Callbacks
  bool globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool nomotionUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool setMapCallback(nav_msgs::SetMap::Request& req, nav_msgs::SetMap::Response& res);

  void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

  void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
  void freeMapDependentMemory();
  map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
  void updatePoseFromServer();
  void applyInitialPose();

  double getYaw(tf::Pose& t);

  // parameter for what odom to use
  std::string odom_frame_id_;

  // paramater to store latest odom pose
  pf_vector_t latest_odom_pose_scan;
  pf_vector_t latest_odom_pose_camera;
  tf::Stamped<tf::Pose> latest_odom_pose_;

  // parameter for what base to use
  std::string base_frame_id_;
  std::string global_frame_id_;
  std::string camera_topic_;
  std::string camera_link_;
  std::string topic_output_video;
  std::string topic_marker;

  bool use_map_topic_;
  bool first_map_only_;

  ros::Duration gui_publish_period;
  ros::Time save_pose_last_time;
  ros::Duration save_pose_period;

  geometry_msgs::PoseWithCovarianceStamped last_published_pose;

  map_t* map_;
  char* mapdata;
  int sx, sy;
  double resolution;

  message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
  ros::Subscriber initial_pose_sub_;
  std::vector<AMCLLaser*> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;

  // Particle filter
  pf_t* pf_;
  double pf_err_, pf_z_;
  bool pf_init_;
  pf_vector_t pf_odom_pose_;
  // pf_vector_t pf_odom_pose_scan;
  double d_thresh_, a_thresh_;
  int resample_interval_;
  int resample_count_cam;
  int resample_count_scan;
  double laser_min_range_;
  double laser_max_range_;

  // Nomotion update control
  bool m_force_update_cam;
  bool m_force_update_scan;  // used to temporarily let amcl update samples even when no motion occurs...
  bool updated_scan;         // used to know what update has occurred last
  bool updated_camera;
  bool pf_init_scan;
  bool pf_init_cam;

  AMCLOdom* odom_;
  AMCLLaser* laser_;
  AMCLMarker* marker_;  // puntero de un objeto de tipo AMCLMarker

  ros::Duration cloud_pub_interval;
  ros::Time last_cloud_pub_time;

  // For slowing play-back when reading directly from a bag file
  ros::WallDuration bag_scan_period_;

  void requestMap();

  // Helper to get odometric pose from transform system
  bool getOdomPose(tf::Stamped<tf::Pose>& pose, double& x, double& y, double& yaw, const ros::Time& t,
                   const std::string& f);

  // time for tolerance on the published transform,
  // basically defines how long a map->odom transform is good for
  ros::Duration transform_tolerance_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pose_pub_;
  ros::Publisher particlecloud_pub_;
  ros::ServiceServer global_loc_srv_;
  ros::ServiceServer nomotion_update_srv_;  // to let amcl update samples without requiring motion
  ros::ServiceServer set_map_srv_;
  ros::Subscriber initial_pose_sub_old_;
  ros::Subscriber map_sub_;
  //%# Publica la proyección de cada punto de la nube sobre la imagen para visualizarlo
  ros::Publisher pixels_particles_image_pub_;

  amcl_hyp_t* initial_pose_hyp_;
  bool first_map_received_;
  bool first_reconfigure_call_;

  boost::recursive_mutex configuration_mutex_;
  dynamic_reconfigure::Server<amcl::AMCLConfig>* dsrv_;
  amcl::AMCLConfig default_config_;
  ros::Timer check_laser_timer_;

  int max_beams_, min_particles_, max_particles_;
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
  double alpha_slow_, alpha_fast_;
  double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  // beam skip related params
  bool do_beamskip_;
  double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
  double laser_likelihood_max_dist_;
  odom_model_t odom_model_type_;
  double init_pose_[3];
  double init_cov_[3];
  laser_model_t laser_model_type_;
  marker_model_t marker_model_type_;
  bool tf_broadcast_;

  void reconfigureCB(amcl::AMCLConfig& config, uint32_t level);

  ros::Time last_laser_received_ts_;
  ros::Duration laser_check_interval_;
  void checkLaserReceived(const ros::TimerEvent& event);

  // For Camera PF
  bool marker_update;
  geometry_msgs::Pose EstimatedPose, Cam1, Cam2, Cam3;
  ros::Publisher publicar_marker, publicar_mapa, error_pub, path_pub_r, path_pub_out, yaw_odom, yaw_amcl;
  ros::Publisher pub_route_calc;
  // #% Añado info_Camera subscriber
  ros::Subscriber detector_subs, corners_subs, ground_truth_subs, real_odom_subs, info_Camera;
  float marker_width, num_cam, marker_height, image_width, ground_truth_x_, ground_truth_y_, ground_truth_yaw_,
      image_height;
  double height_pos_camera_link_;
  // visualization_msgs::Marker pub_map;
  visualization_msgs::MarkerArray marker_array;
  tf::TransformBroadcaster br_marker;
  Mat imagen_filter;
  std::vector<Marcador> marker_map;
  std::vector<geometry_msgs::TransformStamped> tf_cameras;
  std::string frame_to_camera_;
  // Functions
  void LoadMapMarkers(std::vector<int> maps, std::vector<int> sectors, std::vector<int> IDs,
                      std::vector<geometry_msgs::Pose> Centros, std::vector<double> Widths,
                      std::vector<double> Heights);
  void loadTFCameras(std::vector<geometry_msgs::Pose> pose_cameras);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void detectionCallback(const detector::messagedet::ConstPtr& msg);
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void realOdomCallback(const geometry_msgs::PoseStamped& msg);
  // #% Añadimos el Callback para infoCamera setup
  bool setupCameraInfo = false;
  void setupCameraCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);

  std::vector<geometry_msgs::Point> CalculateRelativePose(Marcador Marca, geometry_msgs::Pose CamaraMundo);
  // Prob related parameters
  double marker_z_hit, marker_z_rand, marker_sigma_hit, marker_landa;
  message_filters::Subscriber<detector::messagedet>* marker_detection_sub_;
  tf::MessageFilter<detector::messagedet>* marker_detection_filter_;
  nav_msgs::Path reference, output;
  geometry_msgs::Pose ground_truth;
  geometry_msgs::PoseStamped real_odom;

  // Fusion coefficients
  double marker_coeff;
  double laser_coeff;

  // Simulation or real (for camera)
  int simulation;
  tf::Pose pose_ini;
};

std::vector<std::pair<int, int> > AmclNode::free_space_indices;

#define USAGE "USAGE: amcl"

boost::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}

// ######################## main #########################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);
  ros::Rate r(10);
  // Make our node available to sigintHandler
  // amcl_node_ptr.reset(new AmclNode());
  AmclNode node;
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return (0);
}

AmclNode::AmclNode()
  : sent_first_transform_(false)
  , latest_tf_valid_(false)
  , map_(NULL)
  , pf_(NULL)
  , resample_count_cam(0)
  , resample_count_scan(0)
  , odom_(NULL)
  , laser_(NULL)
  , marker_(NULL)
  , private_nh_("~")
  , initial_pose_hyp_(NULL)
  , first_map_received_(false)
  , first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);
  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0 / tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0 / tmp);

  private_nh_.param("camera_coeff", marker_coeff, 0.5);
  private_nh_.param("laser_coeff", laser_coeff,
                    0.5);  //%# esto estaba mal, antes de poner "laser_coeff" ponia "camera_coeff"
  cout << "COEFF SENSORES-> laser_coeff: " << laser_coeff << "; camera_coeff: " << marker_coeff << endl;

  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);
  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);

  private_nh_.param("do_beamskip", do_beamskip_, false);
  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);

  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_short", z_short_, 0.1);
  private_nh_.param("laser_z_max", z_max_, 0.05);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
  std::string tmp_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if (tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if (tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if (tmp_model_type == "likelihood_field_prob")
  {
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  }
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model", tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }
  std::string tmp_marker_model_type;
  private_nh_.param("marker_model_type", tmp_marker_model_type, std::string("observation_likelihood"));
  if (tmp_marker_model_type == "observation_likelihood")
  {
    marker_model_type_ = MARKER_MODEL_LIKELIHOOD;
  }
  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if (tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if (tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if (tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if (tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model", tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }
  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI / 6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_footprint"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("camera_topic", camera_topic_, std::string("/camera_info"));
  private_nh_.param("resample_interval", resample_interval_, 2);
  private_nh_.param("height_pos_camera_link", height_pos_camera_link_, -0.0001);
  private_nh_.param("camera_link", camera_link_, std::string("xtion_rgb_optical_link"));
  private_nh_.param("topic_output_video", topic_output_video, std::string("/image_converter/output_video"));
  private_nh_.param("topic_marker", topic_marker, std::string("/detected_markers"));
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  private_nh_.param("tf_broadcast", tf_broadcast_, true);

  transform_tolerance_.fromSec(tmp_tol);

  {
    double bag_scan_period;
    private_nh_.param("bag_scan_period", bag_scan_period, -1.0);
    bag_scan_period_.fromSec(bag_scan_period);
  }
  updatePoseFromServer();

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  global_loc_srv_ = nh_.advertiseService("global_localization", &AmclNode::globalLocalizationCallback, this);
  nomotion_update_srv_ = nh_.advertiseService("request_nomotion_update", &AmclNode::nomotionUpdateCallback, this);
  // #% defino el publicador de pixeles de la imagen
  pixels_particles_image_pub_ = nh_.advertise<amcl_hybrid::pixels_cloud>("pixels_clouds", 1);

  set_map_srv_ = nh_.advertiseService("set_map", &AmclNode::setMapCallback, this);

  // ######  descomento esto para que funcione el laser ################################

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100);

  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived, this, _1));
  // ##############################   END  #########################################33

  if (use_map_topic_)
  {
    map_sub_ = nh_.subscribe(global_frame_id_, 1, &AmclNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  }
  else
  {
    requestMap();
  }

  /* dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);*/

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_, boost::bind(&AmclNode::checkLaserReceived, this, _1));

  // cout<<"llego a camara"<<endl;

  // For Camera PF
  XmlRpc::XmlRpcValue marker_list, camera_list;
  // float image_width_;
  private_nh_.getParam("/amcl_hybrid/IMAGE_WIDTH", image_width);
  private_nh_.getParam("/amcl_hybrid/IMAGE_HEIGHT", image_height);
  cout << "image_height: " << image_height << endl;
  // marker_->image_width=image_width_;
  private_nh_.getParam("/amcl_hybrid/MARKER_HEIGHT", marker_height);
  private_nh_.getParam("/amcl_hybrid/MARKER_WIDTH", marker_width);
  private_nh_.getParam("/amcl_hybrid/NUM_CAM", num_cam);
  private_nh_.getParam("/amcl_hybrid/marker_positions", marker_list);
  private_nh_.getParam("/amcl_hybrid/camera_positions", camera_list);
  private_nh_.getParam("/amcl_hybrid/marker_z_hit", marker_z_hit);
  private_nh_.getParam("/amcl_hybrid/marker_z_rand", marker_z_rand);
  private_nh_.getParam("/amcl_hybrid/marker_sigma_hit", marker_sigma_hit);
  private_nh_.getParam("/amcl_hybrid/marker_landa", marker_landa);
  cout << "landa: " << marker_landa << endl;
  private_nh_.getParam("/amcl_hybrid/simulation", simulation);
  marker_ = new AMCLMarker(simulation);
  marker_->simulation = simulation;
  // cout<<"marker_landa"<<marker_landa<<endl;
  // Reading mapfile
  std::vector<geometry_msgs::Pose> Centros;
  std::vector<int> IDs, sectors, maps;
  std::vector<double> widths, heights;
  for (int i = 0; i < marker_list.size(); i++)
  {
    tf::Matrix3x3 orientation;
    tf::Quaternion Quat;
    geometry_msgs::Pose temp_pose;
    temp_pose.position.x = marker_list[i]["x"];
    temp_pose.position.y = marker_list[i]["y"];
    temp_pose.position.z = marker_list[i]["z"];
    double roll = marker_list[i]["roll"];
    double pitch = marker_list[i]["pitch"];
    double yaw = marker_list[i]["yaw"];
    orientation.setRPY(float(roll), float(pitch), float(yaw));
    orientation.getRotation(Quat);
    temp_pose.orientation.x = double(Quat.x());
    temp_pose.orientation.y = double(Quat.y());
    temp_pose.orientation.z = double(Quat.z());
    temp_pose.orientation.w = double(Quat.w());
    Centros.push_back(temp_pose);
    IDs.push_back(marker_list[i]["ID"]);
    sectors.push_back(marker_list[i]["sector"]);
    maps.push_back(marker_list[i]["map"]);
    widths.push_back(marker_list[i]["width"]);
    heights.push_back(marker_list[i]["height"]);
    /*cout << "marker " << marker_list[i]["ID"];
    cout << "  centro-> x:" << temp_pose.position.x << "   y:" << temp_pose.position.y << "   z:" <<
    temp_pose.position.z; cout << "   roll:" << roll << "  pitch:" << pitch << "  yaw:" << yaw << endl;*/
  }
  // cout<<"centros"<<Centros.size()<<endl;
  // Reading camerafile
  std::vector<geometry_msgs::Pose> cameras;
  for (int i = 0; i < camera_list.size(); i++)
  {
    tf::Matrix3x3 orientation;
    tf::Quaternion Quat;
    geometry_msgs::Pose temp_pose;
    temp_pose.position.x = camera_list[i]["x"];
    temp_pose.position.y = camera_list[i]["y"];
    temp_pose.position.z = camera_list[i]["z"];
    double roll = camera_list[i]["roll"];
    double pitch = camera_list[i]["pitch"];
    double yaw = camera_list[i]["yaw"];
    orientation.setRPY(float(roll), float(pitch), float(yaw));
    orientation.getRotation(Quat);
    temp_pose.orientation.x = double(Quat.x());
    temp_pose.orientation.y = double(Quat.y());
    temp_pose.orientation.z = double(Quat.z());
    temp_pose.orientation.w = double(Quat.w());
    cameras.push_back(temp_pose);
  }
  cout << "Loaded marker.yaml file" << endl;
  tf::Quaternion quat;
  /*quat=tf::createQuaternionFromRPY(0,0,1.57);
  cout<<"quaternion initial pose"<<endl;
  cout<<quat.getX()<<endl;
  cout<<quat.getY()<<endl;
  cout<<quat.getZ()<<endl;
  cout<<quat.getW()<<endl;
  waitKey();*/
  // cout<<"cam"<<cameras.size()<<endl;
  this->loadTFCameras(cameras);
  this->LoadMapMarkers(maps, sectors, IDs, Centros, widths, heights);
  this->publicar_marker = nh_.advertise<visualization_msgs::MarkerArray>("marker_positions", 15, true);
  this->publicar_marker.publish(this->marker_array);
  std::cout << "Publish marker_array" << std::endl;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Rate rate_height(10.0);
  while (height_pos_camera_link_ == -0.0001)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer.lookupTransform(base_frame_id_, camera_link_, ros::Time(0));
      height_pos_camera_link_ = transform_stamped.transform.translation.z;
    }
    catch (tf2::TransformException& exception)
    {
      ROS_WARN("%s", exception.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate_height.sleep();
  }
  cout << "height_pos_camera_link_: " << height_pos_camera_link_ << endl;
  // #% execute callback to camera info
  // this->pub_route_calc=nh_.advertise<visualization_msgs::Marker> ("route_amcl")

  this->info_Camera =
      private_nh_.subscribe<sensor_msgs::CameraInfo>(camera_topic_, 1, &AmclNode::setupCameraCallback, this);
  this->detector_subs =
      private_nh_.subscribe<sensor_msgs::Image>(topic_output_video, 1, &AmclNode::imageCallback, this);
  marker_detection_sub_ = new message_filters::Subscriber<detector::messagedet>(nh_, topic_marker, 1);
  marker_detection_filter_ =
      new tf::MessageFilter<detector::messagedet>(*marker_detection_sub_, *tf_, odom_frame_id_, 100);
  marker_detection_filter_->registerCallback(boost::bind(&AmclNode::detectionCallback, this, _1));
  // ground_truth_subs=nh_.subscribe("/Doris/ground_truth/state",1, &AmclNode::groundTruthCallback,this);
  if (simulation == 0)
  {
    real_odom_subs = nh_.subscribe(odom_frame_id_, 1, &AmclNode::realOdomCallback, this);
    path_pub_r = nh_.advertise<nav_msgs::Path>("reference_path", 1);
    yaw_amcl = nh_.advertise<std_msgs::Float64>("amcl_yaw", 1);
    yaw_odom = nh_.advertise<std_msgs::Float64>("odom_yaw", 1);
  }
  m_force_update_cam = false;
  m_force_update_scan = false;
  updated_scan = false;
  updated_camera = false;

  // this->corners_subs=private_nh_.subscribe<detector::messagedet>("/DetectorNode/detection",1,&AmclNode::detectionCallback,this);
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
  error_pub = nh_.advertise<amcl_hybrid::pose_error>("amcl_error", 1);
  path_pub_out = nh_.advertise<nav_msgs::Path>("output_path", 1);
  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  // Configuration
  // cout<<"constructorOK"<<endl;
}

void AmclNode::reconfigureCB(AMCLConfig& config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  // we don't want to do anything on the first call
  // which corresponds to startup
  if (first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if (config.restore_defaults)
  {
    config = default_config_;
    // avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;

  gui_publish_period = ros::Duration(1.0 / config.gui_publish_rate);
  save_pose_period = ros::Duration(1.0 / config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  max_beams_ = config.laser_max_beams;
  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if (config.laser_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if (config.laser_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if (config.laser_model_type == "likelihood_field_prob")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;

  if (config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if (config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if (config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if (config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;

  if (config.min_particles > config.max_particles)
  {
    ROS_WARN(
        "You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;
  tf_broadcast_ = config.tf_broadcast;

  do_beamskip_ = config.do_beamskip;
  beam_skip_distance_ = config.beam_skip_distance;
  beam_skip_threshold_ = config.beam_skip_threshold;

  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, (void*)map_);
  pf_err_ = config.kld_err;
  pf_z_ = config.kld_z;
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6 * 0 + 0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6 * 1 + 1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6 * 5 + 5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_scan = false;
  pf_init_cam = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if (laser_model_type_ == LASER_MODEL_BEAM)
  {
    ROS_INFO("Initializing model BEAM LASER; this can take some time on large maps...");
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, 0.0);
    ROS_INFO("Done initializing model BEAM LASER with probabilities.");
  }
  else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
  {
    ROS_INFO("Initializing likelihood field model PROB LASER; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, do_beamskip_,
                                        beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model PROB LASER with probabilities.");
  }
  else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD)
  {
    ROS_INFO("Initializing likelihood field model LASER; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, laser_coeff);
    ROS_INFO("Done initializing likelihood field model.");
    laser_->pub_coeff_laser = nh_.advertise<amcl_hybrid::coeff_sensor>("coeff_sensor", 10);
  }

  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;

  delete laser_scan_filter_;
  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived, this, _1));

  // Markers
  delete marker_;
  marker_ = new AMCLMarker(simulation);
  ROS_ASSERT(marker_);
  if (marker_model_type_ == MARKER_MODEL_LIKELIHOOD)
  {
    ROS_INFO("Initializing marker filter EN reconfigureCB...");
    marker_->SetModelLikelihoodField(marker_z_hit, marker_z_rand, marker_sigma_hit, marker_landa, marker_coeff);
    ROS_INFO("Done initializing marker filter");
    // cout<<marker_map.size()<<endl;
    marker_->map = marker_map;
    // cout<<tf_cameras.size()<<endl;
    marker_->tf_cameras = tf_cameras;
    marker_->num_cam = num_cam;
    marker_->image_width = image_width;
    marker_->image_height = image_height;
    marker_->height_center_camera = height_pos_camera_link_;
    marker_->pub_coeff_marker = nh_.advertise<amcl_hybrid::coeff_sensor>("coeff_sensor", 10);
    marker_->pub_marker_error = nh_.advertise<amcl_hybrid::marker_error>("error_marker", 10);
  }

  delete marker_detection_filter_;
  marker_detection_filter_ =
      new tf::MessageFilter<detector::messagedet>(*marker_detection_sub_, *tf_, odom_frame_id_, 100);
  marker_detection_filter_->registerCallback(boost::bind(&AmclNode::detectionCallback, this, _1));
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
}

void AmclNode::runFromBag(const std::string& in_bag_fn)
{
  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("tf"));
  std::string scan_topic_name = odom_frame_id_;  // TODO determine what topic this actually is from ROS
  topics.push_back(scan_topic_name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
  ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);

  // Sleep for a second to let all subscribers connect
  ros::WallDuration(1.0).sleep();

  ros::WallTime start(ros::WallTime::now());

  // Wait for map
  while (ros::ok())
  {
    {
      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
      if (map_)
      {
        ROS_INFO("Map is ready");
        break;
      }
    }
    ROS_INFO("Waiting for map...");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
  }

  BOOST_FOREACH (rosbag::MessageInstance const msg, view)
  {
    if (!ros::ok())
    {
      break;
    }

    // Process any ros messages or callbacks at this point
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL)
    {
      tf_pub.publish(msg);
      for (size_t ii = 0; ii < tf_msg->transforms.size(); ++ii)
      {
        // tf_->getBuffer().setTransform(tf_msg->transforms[ii], "rosbag_authority");// cambiado por mi:
        // https://github.com/moriarty/navigation/commit/ae060c92a423783c45ef35005ec443e5736c6689
        tf_->getBuffer()->setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (base_scan != NULL)
    {
      laser_pub.publish(msg);
      laser_scan_filter_->add(base_scan);
      if (bag_scan_period_ > ros::WallDuration(0))
      {
        bag_scan_period_.sleep();
      }
      continue;
    }

    ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
  }

  bag.close();

  double runtime = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Bag complete, took %.1f seconds to process, shutting down", runtime);

  const geometry_msgs::Quaternion& q(last_published_pose.pose.pose.orientation);
  double yaw, pitch, roll;
  tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw, pitch, roll);
  ROS_INFO("Final location %.3f, %.3f, %.3f with stamp=%f", last_published_pose.pose.pose.position.x,
           last_published_pose.pose.pose.position.y, yaw, last_published_pose.header.stamp.toSec());
  ros::shutdown();
}

void AmclNode::savePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf::Pose map_pose;
  map_pose = latest_tf_.inverse() * latest_odom_pose_;

  double yaw, pitch, roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y());

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx", last_published_pose.pose.covariance[6 * 0 + 0]);
  private_nh_.setParam("initial_cov_yy", last_published_pose.pose.covariance[6 * 1 + 1]);
  private_nh_.setParam("initial_cov_aa", last_published_pose.pose.covariance[6 * 5 + 5]);
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
  if (!std::isnan(tmp_pos))
    init_pose_[0] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
  if (!std::isnan(tmp_pos))
    init_pose_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
  if (!std::isnan(tmp_pos))
    init_pose_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");
  private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
  if (!std::isnan(tmp_pos))
    init_cov_[0] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
  if (!std::isnan(tmp_pos))
    init_cov_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
  if (!std::isnan(tmp_pos))
    init_cov_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");
}

void AmclNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if (d > laser_check_interval_)
  {
    ROS_WARN(
        "No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is "
        "being published on the %s topic.",
        d.toSec(), ros::names::resolve(scan_topic_).c_str());
  }
}

void AmclNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while (!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage(resp.map);
}

void AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (first_map_only_ && first_map_received_)
  {
    return;
  }

  handleMapMessage(*msg);

  first_map_received_ = true;
}

void AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n", msg.info.width, msg.info.height, msg.info.resolution);

  if (msg.header.frame_id != global_frame_id_)
    ROS_WARN(
        "Frame_id of map received:'%s' doesn't match global_frame_id:'%s;'. This could cause issues with reading "
        "published topics",
        msg.header.frame_id.c_str(), global_frame_id_.c_str());

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for (int i = 0; i < map_->size_x; i++)
    for (int j = 0; j < map_->size_y; j++)
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i, j));
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, (void*)map_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  updatePoseFromServer();
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  pf_init_scan = false;
  pf_init_cam = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if (laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, 0.0);
  else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
  {
    ROS_INFO("Initializing likelihood field model LASER; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, do_beamskip_,
                                        beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model LASER.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model LASER; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, laser_coeff);
    ROS_INFO("Done initializing likelihood field model LASER.");
    laser_->pub_coeff_laser = nh_.advertise<amcl_hybrid::coeff_sensor>("coeff_sensor", 10);
  }
  // Markers
  delete marker_;
  marker_ = new AMCLMarker(simulation);
  ROS_ASSERT(marker_);
  if (marker_model_type_ == MARKER_MODEL_LIKELIHOOD)
  {
    ROS_INFO("Initializing likelihood field model MARKER, EN handleMapMessage");
    marker_->SetModelLikelihoodField(marker_z_hit, marker_z_rand, marker_sigma_hit, marker_landa, marker_coeff);
    ROS_INFO("Done initializing likelihood field model MARKER.");
    marker_->map = marker_map;
    marker_->tf_cameras = tf_cameras;
    marker_->num_cam = num_cam;
    marker_->image_width = image_width;
    marker_->image_height = image_height;
    marker_->simulation = simulation;
    marker_->height_center_camera = height_pos_camera_link_;
    marker_->pub_coeff_marker = nh_.advertise<amcl_hybrid::coeff_sensor>("coeff_sensor", 10);
    marker_->pub_marker_error = nh_.advertise<amcl_hybrid::marker_error>("error_marker", 10);
  }
  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();
}

void AmclNode::freeMapDependentMemory()
{
  if (map_ != NULL)
  {
    map_free(map_);
    map_ = NULL;
  }
  if (pf_ != NULL)
  {
    pf_free(pf_);
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
  delete marker_;
  marker_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t* AmclNode::convertMap(const nav_msgs::OccupancyGrid& map_msg)
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);
  ROS_ASSERT(map->cells);
  for (int i = 0; i < map->size_x * map->size_y; i++)
  {
    if (map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if (map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

AmclNode::~AmclNode()
{
  delete dsrv_;
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
  delete tfb_;
  delete tf_;
  delete marker_detection_filter_;
  delete marker_detection_sub_;
  // TODO: delete everything allocated in constructor
}

bool AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose, double& x, double& y, double& yaw, const ros::Time& t,
                           const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), t, f);
  // cout<<"odom frame id"<<odom_frame_id_<<endl;
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch (tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  // cout<<"odom x"<<x<<endl;
  // cout<<"odom y"<<y<<endl;
  double pitch, roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  // cout<<yaw<<pitch<<roll<<endl;

  return true;
}

pf_vector_t AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for (;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

bool AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (map_ == NULL)
  {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, (void*)map_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  pf_init_scan = false;
  pf_init_cam = false;
  return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool AmclNode::nomotionUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  std::cout << "no motion" << std::endl;
  m_force_update_scan = true;
  m_force_update_cam = true;
  // ROS_INFO("Requesting no-motion update");
  return true;
}

bool AmclNode::setMapCallback(nav_msgs::SetMap::Request& req, nav_msgs::SetMap::Response& res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  res.success = true;
  return true;
}

void AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  last_laser_received_ts_ = ros::Time::now();
  if (map_ == NULL || laser_coeff == 0.0)
  {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), ros::Time(),
                                laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR(
          "Couldn't transform from %s to %s, "
          "even though the message notifier is in use",
          laser_scan->header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f", laser_pose_v.v[0], laser_pose_v.v[1],
              laser_pose_v.v[2]);
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  }
  else
  {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2], laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  pf_vector_t delta_update = pf_vector_zero();

  if (pf_init_scan)
  {
    // Compute change in pose since last filter actualization (could be camera)
    // delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // Change in pose since last laser actualization
    delta_update.v[0] = pose.v[0] - latest_odom_pose_scan.v[0];
    delta_update.v[1] = pose.v[1] - latest_odom_pose_scan.v[1];
    delta_update.v[2] = angle_diff(pose.v[2], latest_odom_pose_scan.v[2]);
    // See if we should update the filter
    bool update = fabs(delta_update.v[0]) > d_thresh_ || fabs(delta_update.v[1]) > d_thresh_ ||
                  fabs(delta_update.v[2]) > a_thresh_;
    update = update || m_force_update_scan;
    m_force_update_scan = false;

    // Set the laser update flags
    if (update)
      for (unsigned int i = 0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if (!pf_init_scan)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    latest_odom_pose_scan = pose;

    // Filter is now initialized
    pf_init_scan = true;

    // Should update sensor data
    for (unsigned int i = 0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_scan = 0;
  }
  // If the robot has moved, update the filter
  else if (pf_init_scan && lasers_update_[laser_index])
  {
    // printf("pose\n");
    // pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    std::cout << "Update odom with LASER sensor" << std::endl;
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    // this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if (lasers_update_[laser_index])
  {
    AMCLLaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp, laser_scan->header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp, laser_scan->header.frame_id);
    try
    {
      tf_->transformQuaternion(base_frame_id_, min_q, min_q);
      tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s", e.what());
      return;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

    ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if (laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if (laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
    else
      range_min = laser_scan->range_min;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for (int i = 0; i < ldata.range_count; i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if (laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min + (i * angle_increment);
    }
    // lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    updated_scan = lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    ;
    // std::cout<<"Updated pointcloud with LASER\n"<<std::endl;

    lasers_update_[laser_index] = false;

    latest_odom_pose_scan = pose;
    pf_odom_pose_ = pose;

    // Resample the particles
    // if(!(++resample_count_scan % resample_interval_))
    //{
    pf_update_resample(pf_);
    resampled = true;
    // }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    // #% Lo he quitado para que no me llene de valores el terminal
    // ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    if (!m_force_update_scan)
    {
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for (int i = 0; i < set->sample_count; i++)
      {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                 tf::Vector3(set->samples[i].pose.v[0], set->samples[i].pose.v[1], 0)),
                        cloud_msg.poses[i]);
      }
      particlecloud_pub_.publish(cloud_msg);
    }
  }

  if (resampled || force_publication)
  {
    if (!resampled)
    {
      // re-compute the cluster statistics
      pf_cluster_stats(pf_, pf_->sets);
    }
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for (int hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if (max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: X: %.3f Y:%.3f Yaw:%.3f", hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1], hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
      puts("");
      pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
      puts("");
      */

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          // p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6 * i + j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];

      /*
      printf("cov:\n");
      for(int i=0; i<6; i++)
      {
      for(int j=0; j<6; j++)
      printf("%6.3f ", p.covariance[6*i+j]);
      puts("");
      }
      */
      amcl_hybrid::pose_error p_error;

      float error_x = p.pose.pose.position.x - ground_truth_x_;
      float error_y = p.pose.pose.position.y - ground_truth_y_;

      p_error.vec_error.data.push_back(p.pose.pose.position.x - ground_truth_x_);
      p_error.vec_error.data.push_back(p.pose.pose.position.y - ground_truth_y_);
      p_error.vec_error.data.push_back(sqrt((error_x * error_x) + (error_y * error_y)));
      p_error.vec_error.data.push_back(hyps[max_weight_hyp].pf_pose_mean.v[2] - ground_truth_yaw_);
      p_error.num_markers.data = 0;

      p_error.header.stamp = ros::Time::now();
      error_pub.publish(p_error);

      pose_pub_.publish(p);
      std::cout << "publish pose, error LASER" << std::endl;
      last_published_pose = p;
      std::cout << "## new pose with LASER -> ";
      std::cout << " X: " << hyps[max_weight_hyp].pf_pose_mean.v[0];
      std::cout << " Y: " << hyps[max_weight_hyp].pf_pose_mean.v[1];
      std::cout << " Yaw: " << hyps[max_weight_hyp].pf_pose_mean.v[2] << std::endl;

      /*ROS_DEBUG("New pose: X: %6.3f Y:%6.3f Yaw: %6.3f",
                  hyps[max_weight_hyp].pf_pose_mean.v[0],
                  hyps[max_weight_hyp].pf_pose_mean.v[1],
                  hyps[max_weight_hyp].pf_pose_mean.v[2]);*/

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(
            tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
            tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0], hyps[max_weight_hyp].pf_pose_mean.v[1], 0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), laser_scan->header.stamp, base_frame_id_);
        this->tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
      }
      catch (tf::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      if (tf_broadcast_ == true)
      {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id_,
                                            odom_frame_id_);
        this->tfb_->sendTransform(tmp_tf_stamped);
        sent_first_transform_ = true;
      }
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if (latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if ((save_pose_period.toSec() > 0.0) && (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }

  geometry_msgs::PoseStamped pose_g, pose_o;
  pose_g.pose = ground_truth;
  pose_g.header.stamp = ros::Time::now();
  if (simulation == 1)
  {
    reference.poses.push_back(pose_g);
  }

  pose_o.pose = last_published_pose.pose.pose;
  pose_o.header.stamp = ros::Time::now();
  output.poses.push_back(pose_o);
  reference.header.frame_id = "map";
  output.header.frame_id = "map";
  if (simulation == 0)
  {
    reference.poses.push_back(real_odom);
    path_pub_r.publish(reference);
  }
  path_pub_out.publish(output);
}

double AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw, pitch, roll);
  return yaw;
}

void AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // cout<<"he recibido initial pose"<<endl;
  // waitKey();
  handleInitialPoseMessage(*msg);
}

void AmclNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if (msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if (tf_->resolve(msg.header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg.header.frame_id.c_str(), global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tf_->waitForTransform(base_frame_id_, msg.header.stamp, base_frame_id_, now, odom_frame_id_, ros::Duration(0.5));
    tf_->lookupTransform(base_frame_id_, msg.header.stamp, base_frame_id_, now, odom_frame_id_, tx_odom);
  }
  catch (tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if (sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

  // Transform into the global frame
  pose_ini = pose_new;
  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f", ros::Time::now().toSec(), pose_new.getOrigin().x(),
           pose_new.getOrigin().y(), getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  std::cout << "initpose" << std::endl;
  if (initial_pose_hyp_ != NULL && map_ != NULL)
  {
    std::cout << "initpose2" << std::endl;
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;
    pf_init_scan = false;
    pf_init_cam = false;
    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}

/* %#
    Carga las características del mapa para que se visualize en rviz
    Calcula la posición de los corner de cada marca, sabiendo el centro de este que está almacenado en el archivo yaml.
   Son almacenados para que posteriormente sean utilizados
 */
void AmclNode::LoadMapMarkers(std::vector<int> maps, std::vector<int> sectors, std::vector<int> IDs,
                              std::vector<geometry_msgs::Pose> Centros, std::vector<double> Widths,
                              std::vector<double> Heights)
{
  // Show marker in RVIZ
  visualization_msgs::Marker pub_map, pub_arrow, pub_line, pub_corner, pub_text;
  int id_marker = 0;

  pub_map.header.frame_id = "map";
  pub_map.header.stamp = ros::Time();
  pub_map.pose.orientation.w = 1.0;
  pub_map.scale.x = 0.05;
  pub_map.scale.y = 0.05;
  pub_map.scale.z = 0.05;
  pub_map.ns = "centers";
  pub_map.type = visualization_msgs::Marker::SPHERE;
  pub_map.action = visualization_msgs::Marker::ADD;
  pub_map.color.r = 0.3412;
  pub_map.color.g = 0.1373;
  pub_map.color.b = 0.3922;
  pub_map.color.a = 1.0;
  pub_map.pose.orientation.w = 1.0;
  pub_map.lifetime = ros::Duration();

  pub_arrow.header.frame_id = "map";
  pub_arrow.header.stamp = ros::Time();
  pub_arrow.type = visualization_msgs::Marker::ARROW;
  pub_arrow.action = visualization_msgs::Marker::ADD;
  pub_arrow.scale.x = 0.2;
  pub_arrow.scale.y = 0.02;
  pub_arrow.scale.z = 0.02;
  pub_arrow.color.r = 1.0;
  pub_arrow.color.a = 1.0;
  pub_arrow.ns = "arrows";
  pub_arrow.lifetime = ros::Duration();

  pub_line.header.frame_id = "map";
  pub_line.header.stamp = ros::Time();
  pub_line.type = visualization_msgs::Marker::LINE_LIST;
  pub_line.action = visualization_msgs::Marker::ADD;
  pub_line.scale.x = 0.03;
  pub_line.scale.y = 0.03;
  pub_line.scale.z = 0.03;
  pub_line.color.g = 1.0;
  pub_line.color.a = 1.0;
  pub_line.ns = "lines";
  pub_line.id = id_marker++;
  pub_line.lifetime = ros::Duration();

  pub_corner.header.frame_id = "map";
  pub_corner.header.stamp = ros::Time();
  pub_corner.type = visualization_msgs::Marker::SPHERE_LIST;
  pub_corner.action = visualization_msgs::Marker::ADD;
  pub_corner.pose.orientation.w = 1.0;
  pub_corner.scale.x = 0.1;
  pub_corner.scale.y = 0.1;
  pub_corner.scale.z = 0.1;
  pub_corner.ns = "corners";
  pub_corner.id = id_marker++;
  pub_corner.lifetime = ros::Duration();

  pub_text.header.frame_id = "map";
  pub_text.header.stamp = ros::Time();
  pub_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  pub_text.action = visualization_msgs::Marker::ADD;
  pub_text.pose.orientation.w = 1.0;
  pub_text.scale.x = 0.2;
  pub_text.scale.y = 0.2;
  pub_text.scale.z = 0.2;
  pub_text.color.b = 1.0;
  pub_text.color.a = 1.0;
  pub_text.ns = "text";
  pub_text.lifetime = ros::Duration();

  for (int i = 0; i < Centros.size(); i++)
  {
    Marcador Marker;
    geometry_msgs::Pose marker_pose = Centros[i];
    std::cout << "Centro " << i << std::endl;
    std::cout << "    Pos XYZ -> (" << marker_pose.position.x << " ," << marker_pose.position.y << " ,"
              << marker_pose.position.z << " )" << std::endl;
    std::cout << "    Ori XYZW-> (" << marker_pose.orientation.x << " ," << marker_pose.orientation.y << " ,"
              << marker_pose.orientation.z << " ," << marker_pose.orientation.w << " )" << std::endl;
    geometry_msgs::TransformStamped tf_marker;
    tf_marker.header.frame_id = "ground_plane__link";
    tf_marker.child_frame_id = "Marca" + std::to_string(i);
    tf_marker.transform.translation.x = marker_pose.position.x;
    tf_marker.transform.translation.y = marker_pose.position.y;
    tf_marker.transform.translation.z = marker_pose.position.z;
    tf_marker.transform.rotation = marker_pose.orientation;

    pub_map.id = id_marker++;
    pub_map.pose.position = marker_pose.position;

    pub_arrow.id = id_marker++;
    pub_arrow.pose.position = marker_pose.position;
    pub_arrow.pose.orientation = marker_pose.orientation;

    pub_text.id = id_marker++;
    pub_text.pose.position = marker_pose.position;
    pub_text.pose.position.z = marker_pose.position.z + (marker_height * 1.5);
    pub_text.text = "Id:" + std::to_string(IDs[i]);

    geometry_msgs::Pose pos_aux, pos_aux_first, pos_aux_last;
    bool first_point = true;
    for (int j = 0; j < 4; j++)
    {
      geometry_msgs::PointStamped relative_corner;
      relative_corner.point.x = 0;
      std_msgs::ColorRGBA colors_rgba;
      colors_rgba.a = 1.0;
      if (j == 0)
      {  // corner cero -> top/left cuadrant
        relative_corner.point.y = -Widths[i] / 2;
        relative_corner.point.z = Heights[i] / 2;
        colors_rgba.g = 1.0;
      }
      else if (j == 1)
      {  // corner one -> top/right cuadrant
        relative_corner.point.y = Widths[i] / 2;
        relative_corner.point.z = Heights[i] / 2;
        colors_rgba.b = 1.0;
      }
      else if (j == 2)
      {  // corner two -> bottom/right cuadrant
        relative_corner.point.y = Widths[i] / 2;
        relative_corner.point.z = -Heights[i] / 2;
        colors_rgba.r = 1.0;
      }
      else
      {  // corner three -> bottom/left cuadrant
        relative_corner.point.y = -Widths[i] / 2;
        relative_corner.point.z = -Heights[i] / 2;
        colors_rgba.r = 1.0;
        colors_rgba.g = 1.0;
      }
      geometry_msgs::PointStamped global_corner;
      tf2::doTransform(relative_corner, global_corner, tf_marker);
      std::cout << "    Corner " << j << ":" << std::endl;
      std::cout << "          Pos XYZ -> (" << global_corner.point.x << " ," << global_corner.point.y << " ,"
                << global_corner.point.z << " )" << std::endl;
      // Ahora guarda el valor de cada corner en 3D
      Marker.setCorner(global_corner.point);
      if (first_point)
      {
        pos_aux.position = global_corner.point;
        pos_aux_first.position = global_corner.point;
        first_point = false;
      }
      else
      {
        pub_line.points.push_back(pos_aux.position);
        pub_line.points.push_back(global_corner.point);
        pos_aux.position = global_corner.point;
        pos_aux_last.position = global_corner.point;
      }
      pub_corner.points.push_back(global_corner.point);
      pub_corner.colors.push_back(colors_rgba);
    }
    pub_line.points.push_back(pos_aux_last.position);
    pub_line.points.push_back(pos_aux_first.position);
    // cout<<IDs[i]<<endl;
    Marker.setMarkerId(IDs[i]);
    Marker.setSectorId(sectors[i]);
    Marker.setMapId(maps[i]);
    Marker.setWidthId(Widths[i]);
    Marker.setHeightId(Heights[i]);
    this->marker_map.push_back(Marker);
    this->marker_array.markers.push_back(pub_map);
    this->marker_array.markers.push_back(pub_arrow);
    this->marker_array.markers.push_back(pub_text);
  }
  this->marker_array.markers.push_back(pub_line);
  this->marker_array.markers.push_back(pub_corner);

  // TODO: Create a KDtree from the points in the map

  std::cout << "End LoadMaps" << std::endl;
}
// ##### le entra la posición y la orientación de la camra
void AmclNode::loadTFCameras(std::vector<geometry_msgs::Pose> pose_cameras)
{
  // cout<<pose_cameras.size()<<endl;
  for (int i = 0; i < pose_cameras.size(); i++)
  {  // solo existe una cámara
    tf::Vector3 Trasl(pose_cameras[i].position.x, pose_cameras[i].position.y, pose_cameras[i].position.z);
    geometry_msgs::TransformStamped inv_tf_cam_st, tf_cam_st;
    tf::Quaternion QuatT(pose_cameras[i].orientation.x, pose_cameras[i].orientation.y, pose_cameras[i].orientation.z,
                         pose_cameras[i].orientation.w);
    tf::Transform tf_cam, inv_tfcam;
    tf_cam.setOrigin(Trasl);
    tf_cam.setRotation(QuatT);
    inv_tfcam = tf_cam.inverse();
    transformTFToMsg(tf_cam, tf_cam_st.transform);
    transformTFToMsg(inv_tfcam, inv_tf_cam_st.transform);
    inv_tf_cam_st.header.frame_id = camera_link_;
    // inv_tf_cam_st.child_frame_id="Vam"+to_string(i);
    inv_tf_cam_st.child_frame_id = "cam_RGB";
    this->tf_cameras.push_back(inv_tf_cam_st);
    // this->marker_->tf_cameras.push_back(inv_tf_cam_st);
    this->br_marker.sendTransform(inv_tf_cam_st);
  }
  // cout<<"cams in object"<<this->marker_->tf_cameras.size()<<endl;
}

void AmclNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  this->marker_->image_filter = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  // cout<<"Callback"<<endl;
  // imshow("Callback",this->marker_->image_filter);
  waitKey(30);
}

// Include all functions in callback
void AmclNode::detectionCallback(const detector::messagedet::ConstPtr& msg)
{
  if (map_ == NULL || !setupCameraInfo || marker_coeff == 0.0)
  {  // si el mapa está vacío o no se ha recibo todavía la info de la cámara, se sale
    return;
  }

  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  std::vector<Marcador> observation;
  // un bucle por cada marker encontrado
  // cout << "Num the markers is: " << msg->DetectedMarkers.size() << endl;//lo hace bien
  for (int i = 0; i < msg->DetectedMarkers.size(); i++)
  {                                    // detectedMarker es una array donde cada fila es una marca encontrada
    Marcador marker;                   // definimos un objeto de tipo Marcador
    std::vector<cv::Point2f> corners;  // definimos un objeto de tipo vecor de puntos 2D
    marker.setMapId(int(msg->DetectedMarkers[i].map.data));  // insertamos el mapa en el  atributo setMapId del marker
    marker.setSectorId(int(msg->DetectedMarkers[i].sector.data));  // insetamos el Id del sector
    marker.setMarkerId(int(msg->DetectedMarkers[i].ID.data));      // insertamos el Id de la marca
    for (int j = 0; j < 4; j++)
    {  // sacamos del mensaje cada corner y lo añadimos a un vector de una fila y 8 valores ( 2 por cada corner)
      cv::Point2f corner;
      corner.x = msg->DetectedMarkers[i].Corners[j].x;
      corner.y = msg->DetectedMarkers[i].Corners[j].y;
      corners.push_back(corner);  // se ponen en cola
    }
    marker.MarkerPoints(corners);   // #% get save the four corners of the marker pueden estar referenciados
    observation.push_back(marker);  // push in quete the four corner of each corner ( is a vector of marcardor type)
  }
  // observation.clear();
  // cout<<"detected"<<observation.size()<<endl;
  // tf_broadcast_=true;
  // cout<<"tf_broadcast"<<tf_broadcast_<<endl;
  // cout<<"base_frame_id"<<base_frame_id_<<endl;
  // cout<<"odom frame"<<odom_frame_id_<<endl;
  if (frame_to_camera_ != msg->header.frame_id)
  {                                      // entra una sola vez
    marker_ = new AMCLMarker(*marker_);  // marker_ es un objeto de tipo AMCLMarker ,*marker_ -> el comando * de un
                                         // puntero hace que un objeto (marker_) este referenciado al objeto que apunta
    marker_update = true;
    frame_to_camera_ = msg->header.frame_id;
    marker_->simulation = simulation;
    marker_->image_height = image_height;
    marker_->image_width = image_width;
    marker_->height_center_camera = height_pos_camera_link_;
    marker_->pub_coeff_marker = nh_.advertise<amcl_hybrid::coeff_sensor>("coeff_sensor", 10);
    marker_->pub_marker_error = nh_.advertise<amcl_hybrid::marker_error>("error_marker", 10);
  }
  pf_vector_t pose;

  // bool force_publication = false;
  //  cout<<"odom"<<getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
  // msg.header.stamp, base_frame_id_)<<endl;

  if (!(getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2], msg->header.stamp, base_frame_id_)))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with camera info");
    return;
  }
  pf_vector_t delta = pf_vector_zero();
  pf_vector_t delta_update = pf_vector_zero();
  if (pf_init_cam)
  {
    // cout<<"in pf_init"<<endl;
    // cout<<"update"<<update<<endl;
    //  Compute change in pose
    // delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    // Change in position since last filter actualization(laser or camera)
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // Change in position since last filter actualization
    delta_update.v[0] = pose.v[0] - latest_odom_pose_camera.v[0];
    delta_update.v[1] = pose.v[1] - latest_odom_pose_camera.v[1];
    delta_update.v[2] = angle_diff(pose.v[2], latest_odom_pose_camera.v[2]);
    // See if we should update the filter
    bool update = fabs(delta_update.v[0]) > d_thresh_ || fabs(delta_update.v[1]) > d_thresh_ ||
                  fabs(delta_update.v[2]) > a_thresh_;
    // cout<<"update"<<update<<endl;
    update = update || m_force_update_cam;
    m_force_update_cam = false;

    // Set the laser update flags
    if (update)
    {
      // cout<<"entra en update"<<endl;
      marker_update = true;
    }
    // waitKey();
  }
  bool force_publication = false;

  if (!pf_init_cam)  // Update data if the robot has moved
  {
    // cout<<"not init"<<endl;
    //  Pose at last filter update
    pf_odom_pose_ = pose;
    latest_odom_pose_camera = pose;
    // Filter is now initialized
    pf_init_cam = true;

    // Should update sensor data
    marker_update = true;

    force_publication = true;

    resample_count_cam = 0;
  }
  // If the robot has moved update the filter
  else if (pf_init_cam && marker_update)
  {
    std::cout << "Update odom with MARKER sensor" << std::endl;
    AMCLOdomData odata;
    odata.pose = pose;
    odata.delta = delta;
    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);
  }
  bool resampled = false;
  if (marker_update)
  {
    // cout<<"actualizar markers"<<endl;
    // waitKey();
    AMCLMarkerData mdata;
    mdata.sensor = this->marker_;
    mdata.markers_obs = observation;
    marker_->model_type = marker_model_type_;
    marker_->image_width = image_width;
    marker_->num_cam = num_cam;
    marker_->image_height = image_height;
    marker_->height_center_camera = height_pos_camera_link_;
    marker_->pub_coeff_marker = nh_.advertise<amcl_hybrid::coeff_sensor>("coeff_sensor", 10);
    marker_->pub_marker_error = nh_.advertise<amcl_hybrid::marker_error>("error_marker", 10);
    // cout<<"cuantos"<<observation.size()<<endl;
    // cout<<global_frame_id_<<endl;
    // cout<<odom_frame_id_<<endl;
    // Update filter with marker data

    if (!observation.empty())
    {  // entra si el vector de marcas encontradas es distinto a 0
      // cout<<"Received markers"<<endl;
      // #% esta es la que hace la actualización de los pesos de particulas segun las markers
      // marker_->UpdateSensor(pf_,(AMCLSensorData*) &mdata);
      updated_camera = marker_->UpdateSensor(pf_, (AMCLSensorData*)&mdata);
      ;
      // #% aquí publicaría la nube de pixeles
      pixels_particles_image_pub_.publish(marker_->send_pixels_cloud);
      // std::cout << "Updated pointcloud with MARKERS\n" << std::endl;
    }
    latest_odom_pose_camera = pose;
    pf_odom_pose_ = pose;
    marker_update = false;
    // cout<<"numcam"<<num_cam<<endl;
    // cout<<"image_width"<<image_width<<endl;

    if (!(++resample_count_cam % resample_interval_))
      //{
      // cout<<"resample"<<endl;
      pf_update_resample(pf_);
    resampled = true;
    //}

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    // ROS_INFO("Updated Marker");
    // #% Lo he quitado para que no me llene de valores el terminal
    // ROS_INFO("Num samples: %d\n", set->sample_count);
    // ROS_INFO("Publish pixels_cloud");

    // Publish resulting particle cloud
    if (!m_force_update_cam)
    {
      // cout<<"publishparticles"<<endl;
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for (int i = 0; i < set->sample_count; i++)
      {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                 tf::Vector3(set->samples[i].pose.v[0], set->samples[i].pose.v[1], 0)),
                        cloud_msg.poses[i]);
      }
      // cout<<"Publicacion de la nube de particulas"<<endl;
      particlecloud_pub_.publish(cloud_msg);
    }
  }
  if (resampled || force_publication)
  {
    if (!resampled)
    {
      // re-compute the cluster statistics
      pf_cluster_stats(pf_, pf_->sets);
    }
    // read hypotheses
    double max_weight = 0.0;
    double max_weight_hyp = -1;
    std::vector<amcl_hyp_t> marker_hyps;
    marker_hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for (int hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;

      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      marker_hyps[hyp_count].weight = weight;
      marker_hyps[hyp_count].pf_pose_mean = pose_mean;
      marker_hyps[hyp_count].pf_pose_cov = pose_cov;

      if (marker_hyps[hyp_count].weight > max_weight)
      {
        max_weight = marker_hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }
    // cout<<"max weight"<<max_weight<<endl;
    if (max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f", marker_hyps[max_weight_hyp].pf_pose_mean.v[0],
                marker_hyps[max_weight_hyp].pf_pose_mean.v[1], marker_hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // Message for pose
      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = msg->header.stamp;
      // Copy in the pose with the maximum weigh
      p.pose.pose.position.x = marker_hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = marker_hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(marker_hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 2; j++)
        {
          p.pose.covariance[6 * i + j] = set->cov.m[i][j];
        }
      }
      p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];

      // cout<<"publicacion de la pose"<<endl;
      pose_pub_.publish(p);

      // Publishing error with gazebo's ground_truth
      if (simulation == 1)
      {
        amcl_hybrid::pose_error p_error;

        float error_x = p.pose.pose.position.x - ground_truth_x_;
        float error_y = p.pose.pose.position.y - ground_truth_y_;

        p_error.vec_error.data.push_back(p.pose.pose.position.x - ground_truth_x_);
        p_error.vec_error.data.push_back(p.pose.pose.position.y - ground_truth_y_);
        p_error.vec_error.data.push_back(sqrt((error_x * error_x) + (error_y * error_y)));
        p_error.vec_error.data.push_back(marker_hyps[max_weight_hyp].pf_pose_mean.v[2] - ground_truth_yaw_);
        p_error.num_markers.data = int(observation.size());
        p_error.header.stamp = ros::Time::now();
        error_pub.publish(p_error);
        std::cout << "publish pose, error Marker" << std::endl;
      }

      // yaw_out.header.stamp=ros::Time::now();
      if (simulation == 0)
      {
        std_msgs::Float64 yaw_out;
        yaw_out.data = marker_hyps[max_weight_hyp].pf_pose_mean.v[2];
        yaw_amcl.publish(yaw_out);
      }
      last_published_pose = p;

      // ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
      std::cout << "## new pose with Marker -> ";
      std::cout << " X: " << marker_hyps[max_weight_hyp].pf_pose_mean.v[0];
      std::cout << " Y: " << marker_hyps[max_weight_hyp].pf_pose_mean.v[1];
      std::cout << " Yaw: " << marker_hyps[max_weight_hyp].pf_pose_mean.v[2] << std::endl;

      // tf from map to odom frame
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(marker_hyps[max_weight_hyp].pf_pose_mean.v[2]),
                             tf::Vector3(marker_hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         marker_hyps[max_weight_hyp].pf_pose_mean.v[1], 0.0));
        // cout<<base_frame_id_<<endl; //TODO: Revisar por Paloma
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), ros::Time(0), base_frame_id_);
        // cout<<odom_frame_id_<<endl;
        this->tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
        // cout<<"publica tf base_link->odom"<<endl;
      }
      catch (tf::TransformException e)
      {
        std::cout << "Failed to subtract base to odom transform " << e.what() << std::endl;
        waitKey();
        return;
      }
      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;
      // cout<<"tf_broadcast2"<<tf_broadcast_<<endl;
      if (tf_broadcast_ == true)
      {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (msg->header.stamp + transform_tolerance_);
        // cout<<"entro en publicar tf"<<endl;
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), msg->header.stamp, global_frame_id_, odom_frame_id_);
        // cout<<"Publicar tf odom ->map"<<endl;
        this->tfb_->sendTransform(tmp_tf_stamped);
        sent_first_transform_ = true;
      }
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if (latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (msg->header.stamp + transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }
    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if ((save_pose_period.toSec() > 0.0) && (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }
  geometry_msgs::PoseStamped pose_g, pose_o;
  pose_g.pose = ground_truth;
  pose_g.header.stamp = ros::Time::now();
  if (simulation == 1)
  {
    reference.poses.push_back(pose_g);
  }

  pose_o.pose = last_published_pose.pose.pose;
  pose_o.header.stamp = ros::Time::now();
  output.poses.push_back(pose_o);
  reference.header.frame_id = "map";
  output.header.frame_id = "map";
  if (simulation == 0)
  {
    reference.poses.push_back(real_odom);
    path_pub_r.publish(reference);
  }
  path_pub_out.publish(output);
}

void AmclNode::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ground_truth = msg->pose.pose;
  ground_truth_x_ = msg->pose.pose.position.x;
  ground_truth_y_ = msg->pose.pose.position.y;
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  ground_truth_yaw_ = tf::getYaw(pose.getRotation());
}

void AmclNode::realOdomCallback(const geometry_msgs::PoseStamped& msg)
{
  real_odom = msg;
  real_odom.pose.position.x = real_odom.pose.position.x + pose_ini.getOrigin().x();
  real_odom.pose.position.y = real_odom.pose.position.y + pose_ini.getOrigin().y();

  tf::Pose pose;
  tf::poseMsgToTF(msg.pose, pose);
  float real_odom_yaw = tf::getYaw(pose.getRotation());
  std_msgs::Float64 yaw;
  // yaw.header.stamp=ros::Time::now();
  yaw.data = real_odom_yaw;
  yaw_odom.publish(yaw);
}

//%#
void AmclNode::setupCameraCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if (!setupCameraInfo)
  {  // solo entra una vez
    marker_->LoadCameraInfo2(cam_info);
    setupCameraInfo = true;
    std::cout << "\nUpdated CameraInfo with cameraInfo topic" << std::endl;
    image_width = marker_->cam_inf_ed.width;
    image_height = marker_->cam_inf_ed.height;
    std::cout << "#### camera info ####" << std::endl;
    std::cout << "- Width image: " << image_width << std::endl;
    std::cout << "- Height image: " << image_height << std::endl;
    std::cout << "- Height camera: " << height_pos_camera_link_ << std::endl;
  }
}