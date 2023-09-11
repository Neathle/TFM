#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
#include <chrono>

/**NODE that suscribes to topic with amcl pose and publisth the data as path to visualize in Rviz*/

/*GLOBAL VARIABLES*/
float x_amcl, y_amcl, theta_amcl;

/*FUNCTION DEFINITIONS*/
void poseCallback_amcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
	x_amcl = msg -> pose.pose.position.x;
	y_amcl = msg -> pose.pose.position.y;
	tf::Quaternion q( msg->pose.pose.orientation.x,
					  msg->pose.pose.orientation.y,
					  msg->pose.pose.orientation.z,
					  msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	theta_amcl = yaw;
}

/*MAIN PROGRAM*/
int main(int argc, char **argv)
{
	/*Initialize*/
	ros::init(argc, argv, "run_amcl");
	ros::NodeHandle nh;
	
	/*Declare publishers and suscriber*/
	ros::Subscriber sub_amcl        = nh.subscribe("/amcl_pose", 1000, poseCallback_amcl);
	ros::Publisher  path_pub_amcl   = nh.advertise<nav_msgs::Path>("trajectory_amcl",1000);
		
	/*Get current time in variable*/
	ros::Time current_time;
	
	/*Declare path messages*/
	nav_msgs::Path amcl_path;
	
	/*Node frequency*/
	ros::Rate r(10.0);
	
	while(nh.ok()){
		ros::spinOnce();  /*check for incoming messages*/
		current_time = ros::Time::now(); /*get current time*/
		
		/*transform theta to quaternion for odom msg*/
		geometry_msgs::Quaternion g_truth_quat = tf::createQuaternionMsgFromYaw(theta_amcl);
		
		/*create path msg w/amcl_pose information*/
		geometry_msgs::PoseStamped amcl_pose_stamped;
		amcl_pose_stamped.pose.position.x  = x_amcl;
		amcl_pose_stamped.pose.position.y  = y_amcl;
		amcl_pose_stamped.pose.orientation = g_truth_quat;
		amcl_pose_stamped.header.stamp     = current_time;
		amcl_pose_stamped.header.frame_id  = "map";
		amcl_path.poses.push_back(amcl_pose_stamped);
		amcl_path.header.stamp = current_time;
		amcl_path.header.frame_id = "map";
		path_pub_amcl.publish(amcl_path);
				
		r.sleep();
	}
}