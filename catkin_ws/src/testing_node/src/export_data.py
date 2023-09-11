#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import math
#Script that reads from topics /amcl_pose and /gtruth_pose and exports data to txt file_amcl for posterior metrics processing. 

class ExportPosesTxt:
    
    def __init__ (self):
        rospy.init_node ('testing_node' , anonymous=True)
        rospy.Subscriber("/amcl_pose"   , PoseWithCovarianceStamped, self.callback_amcl)
        rospy.Subscriber("/g_truth/Path", Path, self.callback_gt)
        rospy.Subscriber("/odom"        , Odometry, self.callback_odom)
        
        self.file_name = rospy.get_param('/testing_node_export_data/file_name')
        self.scenario = rospy.get_param('/testing_node_export_data/scenario')
                
        self.file_amcl = open('/home/fer/.ros/'+self.scenario+'/data_amcl_'+self.file_name+'.txt', 'w') #create txt file_amcl to export data from callback funct 
        self.file_amcl.write("Seq,TimeStamp,Xamcl,Yamcl,Thacml,Xamcl_cov,Yamcl_cov,Thamcl_cov\n")
        
        self.file_gt = open('/home/fer/.ros/'+self.scenario+'/data_gdtr_'+self.file_name+'.txt', 'w')
        self.file_gt.write("Seq,TimeStamp,Xgt,Ygt,Thgt\n")
        
        self.file_odom = open('/home/fer/.ros/'+self.scenario+'/data_odom_'+self.file_name+'.txt', 'w')
        self.file_odom.write("Seq,TimeStamp,Xodom,Yodom,Thodm,Xodom_cov,Yodom_cov,Thodom_cov\n")
    
        rospy.spin()
        self.file_amcl.close()
        self.file_odom.close()
        self.file_gt.close()

    def callback_amcl(self, amcl_pose):
        self.file_amcl.write(str(amcl_pose.header.seq)+",")
        self.file_amcl.write(str(amcl_pose.header.stamp)+",")
        self.file_amcl.write(str(amcl_pose.pose.pose.position.x)+",")
        self.file_amcl.write(str(amcl_pose.pose.pose.position.y)+",")

        quat_amcl = (amcl_pose.pose.pose.orientation.x, amcl_pose.pose.pose.orientation.y, amcl_pose.pose.pose.orientation.z, amcl_pose.pose.pose.orientation.w)
        euler = euler_from_quaternion(quat_amcl) 
        th_amcl = (euler[2] * 180)/math.pi #aqui lo convierto a grados, pero es mejor dejarlo en radianes
        self.file_amcl.write(str(th_amcl)+",")
        
        self.file_amcl.write(str(amcl_pose.pose.covariance[0]) +",")
        self.file_amcl.write(str(amcl_pose.pose.covariance[7]) +",")
        self.file_amcl.write(str(amcl_pose.pose.covariance[35])+"\n")
        
        print( "amcl pose:", amcl_pose.pose.pose.position.x, amcl_pose.pose.pose.position.y, th_amcl )
        print( "amcl cov:", amcl_pose.pose.covariance[0], amcl_pose.pose.covariance[7], amcl_pose.pose.covariance[35] )
        
    def callback_gt(self, gt_pose):
        self.file_gt.write(str(gt_pose.header.seq)+",")
        self.file_gt.write(str(gt_pose.header.stamp)+",")
        self.file_gt.write(str(gt_pose.poses[-1].pose.position.x)+",")
        self.file_gt.write(str(gt_pose.poses[-1].pose.position.y)+",")
       
        quat_gt = (gt_pose.poses[-1].pose.orientation.x, gt_pose.poses[-1].pose.orientation.y, gt_pose.poses[-1].pose.orientation.z, gt_pose.poses[-1].pose.orientation.w)
        euler = euler_from_quaternion(quat_gt) 
        th_gt = (euler[2] * 180)/math.pi
        self.file_gt.write(str(th_gt)+"\n")
        
        print( "gtruth pose:", gt_pose.poses[-1].pose.position.x, gt_pose.poses[-1].pose.position.y, th_gt )

    def callback_odom(self, odom_pose):
        self.file_odom.write(str(odom_pose.header.seq)+",")
        self.file_odom.write(str(odom_pose.header.stamp)+",")
        self.file_odom.write(str(odom_pose.pose.pose.position.x)+",")
        self.file_odom.write(str(odom_pose.pose.pose.position.y)+",")
       
        quat_odom = (odom_pose.pose.pose.orientation.x, odom_pose.pose.pose.orientation.y, odom_pose.pose.pose.orientation.z, odom_pose.pose.pose.orientation.w)
        euler = euler_from_quaternion(quat_odom) 
        th_odom = (euler[2] * 180)/math.pi
        self.file_odom.write(str(th_odom)+",")
        
        self.file_odom.write(str(odom_pose.pose.covariance[0]) +",")
        self.file_odom.write(str(odom_pose.pose.covariance[7]) +",")
        self.file_odom.write(str(odom_pose.pose.covariance[35])+"\n")
        
        print( "odom pose:", odom_pose.pose.pose.position.x, odom_pose.pose.pose.position.y, th_odom )

    
if __name__ == '__main__':
    export_data = ExportPosesTxt()
