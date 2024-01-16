#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <amcl/odometry/odometry.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <random>

constexpr size_t N = 20;


static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x - vec2.x, 2) + std::pow(vec1.y - vec2.y, 2) + std::pow(vec1.z - vec2.z, 2));
}

/*
 * \brief Check that a link exists and has a geometry collision.
 * \param link The link
 * \return true if the link has a collision element with geometry
 */
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr& link)
{
  if (!link)
  {
    ROS_ERROR("Link pointer is null.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name
                             << " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name
                             << " does not have collision geometry description. Add collision geometry description for "
                                "link to urdf.");
    return false;
  }
  return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr& link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have sphere geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius)
{
  if (isCylinder(wheel_link))
  {
    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }
  else if (isSphere(wheel_link))
  {
    wheel_radius = (static_cast<urdf::Sphere*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }

  ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder or sphere!");
  return false;
}

class IsaacOdomPublisher
{
public:
  IsaacOdomPublisher(): distribution(0.0, std::sqrt(0.0001))
  {
    nh.param("left_wheel_names", left_wheel_names, std::vector<std::string>(1, "wheel_left_joint"));
    nh.param("right_wheel_names", right_wheel_names,  std::vector<std::string>(1, "wheel_right_joint"));
    nh.param("wheel_radius", wheel_radius, 0.0985);
    nh.param("wheel_separation", wheel_separation, 0.4044);
    nh.param("publish_rate", publish_rate, 50.0);
    nh.param("pose_covariance_diagonal", pose_covariance_diagonal, std::vector<double>{0.001, 0.001, 0.001, 0.001, 0.001, 0.01});
    nh.param("twist_covariance_diagonal", twist_covariance_diagonal, std::vector<double>{0.001, 0.001, 0.001, 0.001, 0.001, 0.01});
    nh.param("base_frame_id", base_frame_id, std::string("base_footprint"));
    nh.param("velocity_rolling_window_size", velocity_rolling_window_size, 10);

    publish_period_ = ros::Duration(1.0 / publish_rate);
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !nh.getParam("wheel_separation", wheel_separation);
    bool lookup_wheel_radius = !nh.getParam("wheel_radius", wheel_radius);

    if (!setOdomParamsFromUrdf(nh, left_wheel_names[0], right_wheel_names[0], lookup_wheel_separation,
                               lookup_wheel_radius))
    {
      return;
    }
    odometry_.setWheelParams(wheel_separation, wheel_radius, wheel_radius);
    ROS_INFO_STREAM_NAMED(name_, "Odometry params : wheel separation " << wheel_separation << ", left wheel radius "
                                                                       << wheel_radius << ", right wheel radius "
                                                                       << wheel_radius);

    setOdomPubFields(nh, nh);

    ros::Time time = ros::Time::now();
    last_state_publish_time_ = time;
    time_previous_ = time;
    
    odometry_.init(time);
  }

  void tfCallback()
  {
    double roll, pitch, yaw, left_pos, right_pos;
    try {
        listener.lookupTransform(base_frame_id, "wheel_left_link", ros::Time(0), transform);
        tf::Quaternion left_rot = transform.getRotation();
        tf::Matrix3x3 l(left_rot);
        l.getRPY(roll, left_pos, yaw);
        double old_left_pos = left_pos;
        if(yaw > M_PI/2){
            if(left_pos > 0) {
              left_pos = M_PI - left_pos;
            } else {
              left_pos = -M_PI - left_pos;
            }
        }
        left_pos = unwrap(left_pos, true) + distribution(generator);

        // ROS_WARN_STREAM_NAMED(name_, "old_left_pos: " << old_left_pos << ", left_pos: " << left_pos << ", yaw:" << yaw);

        listener.lookupTransform(base_frame_id, "wheel_right_link", ros::Time(0), transform);
        tf::Quaternion right_rot = transform.getRotation();
        tf::Matrix3x3 r(right_rot);
        r.getRPY(roll, right_pos, yaw);
        double old_right_pos = right_pos;
        if(yaw > M_PI/2){
            if(right_pos > 0) {
              right_pos = M_PI - right_pos;
            } else {
              right_pos = -M_PI - right_pos;
            }
        }
        right_pos = unwrap(right_pos, false) + distribution(generator);

        // ROS_WARN_STREAM_NAMED(name_,  "old_right_pos: " << old_right_pos << ", right_pos: " << right_pos << ", yaw:" << yaw);

        update(transform.stamp_ , publish_period_, left_pos, right_pos);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_STREAM_NAMED(name_, "Error when calculating odometry: \n" << ex.what());
    }
    
  }

  bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh, const std::string& left_wheel_name,
                             const std::string& right_wheel_name, bool lookup_wheel_separation,
                             bool lookup_wheel_radius)
  {
    if (!(lookup_wheel_separation || lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str = "";
    if (!res || !root_nh.getParam(model_param_name, robot_model_str))
    {
      ROS_ERROR_NAMED(name_, "Robot description couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    urdf::JointConstSharedPtr left_wheel_joint(model->getJoint(left_wheel_name));
    urdf::JointConstSharedPtr right_wheel_joint(model->getJoint(right_wheel_name));

    if (!left_wheel_joint)
    {
      ROS_ERROR_STREAM_NAMED(name_, left_wheel_name << " couldn't be retrieved from model description");
      return false;
    }

    if (!right_wheel_joint)
    {
      ROS_ERROR_STREAM_NAMED(name_, right_wheel_name << " couldn't be retrieved from model description");
      return false;
    }

    if (lookup_wheel_separation)
    {
      // Get wheel separation
      ROS_INFO_STREAM("left wheel to origin: " << left_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                                               << left_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                                               << left_wheel_joint->parent_to_joint_origin_transform.position.z);
      ROS_INFO_STREAM("right wheel to origin: " << right_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                                                << right_wheel_joint->parent_to_joint_origin_transform.position.y
                                                << ", "
                                                << right_wheel_joint->parent_to_joint_origin_transform.position.z);

      wheel_separation = euclideanOfVectors(left_wheel_joint->parent_to_joint_origin_transform.position,
                                            right_wheel_joint->parent_to_joint_origin_transform.position);
    }

    if (lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << left_wheel_name << " wheel radius");
        return false;
      }
    }

    return true;
  }

  void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = base_frame_id;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = { static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0., 0.,
                                        static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0., 0., 0.,
                                        static_cast<double>(pose_cov_list[2]), 0., 0., 0., 0., 0., 0.,
                                        static_cast<double>(pose_cov_list[3]), 0., 0., 0., 0., 0., 0.,
                                        static_cast<double>(pose_cov_list[4]), 0., 0., 0., 0., 0., 0.,
                                        static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y = 0;
    odom_pub_->msg_.twist.twist.linear.z = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = { static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
                                         static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
                                         static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
                                         static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
                                         static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
                                         static_cast<double>(twist_cov_list[5]) };
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  }

  void update(const ros::Time& time, const ros::Duration& period, double left_pos, double right_pos)
  {

    // Estimate linear and angular velocity using joint information
    odometry_.update(left_pos, right_pos, time);

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinear();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    time_previous_ = time;
  }

  double unwrap(double new_angle, bool is_left) {
    double& offset = is_left ? offset_left : offset_right;
    double& prev_pos = is_left ? previous_left_pos : previous_right_pos;
    double diff = new_angle - prev_pos;

    if (diff > M_PI) {
        offset -= 2.0 * M_PI;
    } else if (diff < -M_PI) {
        offset += 2.0 * M_PI;
    }

    prev_pos = new_angle;
    return new_angle + offset;
}

private:
  ros::NodeHandle nh;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
  tf::TransformListener listener;

  std::vector<std::string> left_wheel_names, right_wheel_names;
  double wheel_radius, wheel_separation;
  double publish_rate;
  std::vector<double> pose_covariance_diagonal, twist_covariance_diagonal;
  std::string base_frame_id;
  int velocity_rolling_window_size;
  std::string robot_urdf_path;

  tf::StampedTransform transform;

  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  ros::Time time_previous_;
  diff_drive_controller::Odometry odometry_;
  std::string name_ = "IsaacOdomPublisher";

  
  std::array<double, N> unwrap_left = {0};
  std::array<double, N> unwrap_right = {0};
  double offset_left = 0.0;
  double offset_right = 0.0;
  double previous_left_pos = 0.0;
  double previous_right_pos = 0.0;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "isaac_odom_publisher");
  IsaacOdomPublisher isaacOdomPublisher;
  ros::Rate rate(10.0);
  while (ros::ok())
  {
      isaacOdomPublisher.tfCallback();
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}
