#!/usr/bin/env python
import rospy
import roslib
import rosbag
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import tf2_ros
import tf 
import tf2_geometry_msgs 
import geometry_msgs.msg
from std_msgs.msg import ColorRGBA
import os
import time
import yaml
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Camera calibration parameters, taken from dataset
cx = 157.3245865
cy = 120.0802295
fx  = 286.441384
fy  = 271.36999
    

def pixel_to_point_in_space(pixel, pt_x, pt_y, cx, cy, fx, fy):
    #Function to obtain x,y,z position of pixels in space. Equations taken from dataset webpage    
    h = pt_y
    w = pt_x
    depth = pixel * (1/278.905687) # read each single pixel in image
    x = depth
    z = (cx- h) * x/fx 
    y = (cy- w) * x/fy 
    point = [x*10, y*10, z*10]
    return point


def create_marker(x,y,z, t, frame, color):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = t
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0
    # Set the scale of the marker
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    # Set the color
    marker.color = color
    # Set the pose of the marker, primero convierto y luego paso aquí los datos convertidos 
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    return marker 
 
 
def create_marker_text(x,y,z, t, frame, color, text):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = t
    marker.type=Marker.TEXT_VIEW_FACING
    marker.id=0
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    marker.color = color
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.text=text
    return marker


def create_static_tf(x,y,z,quat,time,frame, child_frame):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = time
    static_transformStamped.header.frame_id = frame
    static_transformStamped.child_frame_id = child_frame
    static_transformStamped.transform.translation.x = x
    static_transformStamped.transform.translation.y = y
    static_transformStamped.transform.translation.z = z
    #quat = tf.transformations.quaternion_from_euler(0,0,yaw)
    static_transformStamped.transform.rotation.x = quat.x
    static_transformStamped.transform.rotation.y = quat.y
    static_transformStamped.transform.rotation.z = quat.z
    static_transformStamped.transform.rotation.w= quat.w
    return static_transformStamped


def create_TFmsg(x, y, z, q, frame, child_frame):
    trans = geometry_msgs.msg.TransformStamped()
    trans.header.frame_id = frame
    trans.child_frame_id = child_frame
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    #q = tf.transformations.quaternion_from_euler(0,0,theta)
    trans.transform.rotation.x  = q.x
    trans.transform.rotation.y  = q.y
    trans.transform.rotation.z  = q.z
    trans.transform.rotation.w = q.w
    return trans


def sortCorners(corners):
    sorted_corner = [list(), list(), list(), list()]
    M = cv2.moments(corners) #find rectangle centroid, (it can be an irregular rectangle so we want the centroid instead of the center) https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    w_center = cx
    h_center = cy             
    center= [cx, cy]  
    #clasify corners according to their position compared to the centroid point
    for corner in corners:
        if corner[0][0] < w_center and corner[0][1] < h_center:   
            sorted_corner[0] = corner[0]
        elif corner[0][0] > w_center and corner[0][1] < h_center: 
            sorted_corner[1] = corner[0]
        elif corner[0][0] > w_center and corner[0][1] > h_center: 
            sorted_corner[2] = corner[0]
        elif corner[0][0] < w_center and corner[0][1] > h_center: 
            sorted_corner[3] = corner[0]
        else:
            print("Error of Cuadrant or a corner equal to centroid point-irregular polygons is best to avoid")
            return 0
    return sorted_corner, center
    

def main(args):
    #1. Parameters SETUP
    input_data_folder           =  rospy.get_param('marker_map_creator/input_data_folder')#seq, tstamp, points 
    output_map_path            =  rospy.get_param('marker_map_creator/output_map_path')
    input_gtruth_bag            =  rospy.get_param('marker_map_creator/input_gtruth_bag')
    input_depth_bag            =  rospy.get_param('marker_map_creator/input_depth_bag')
    input_depth_img_topic =  rospy.get_param('marker_map_creator/input_depth_img_topic')
    bridge = CvBridge()
    
    #2. Pre-procesing: Read and parse data from text file
    files = os.listdir(input_data_folder)
    filenames = []
    markers_data = []
    for file in files:
        if file.endswith( '.txt'):
            filenames.append(file) #append in list all txt files located in the given path
    num_files = len(filenames)
    print("############################")
    print("#### File Pre-procesing ####")
    print("############################")
    print( "num_files is: ", int(num_files))
    for file in range(num_files):
        with open (input_data_folder+filenames[file], 'r') as myfile:
            data = myfile.readlines()
            seq      = int(data[0].translate({ord(i): None for i in '\n'}))
            tstamp = data[1].translate({ord(i): None for i in '\n'})
            tstamp = rospy.Time(int( tstamp[:10]), int(tstamp[10:]) )
            pt1 = data[2].translate({ord(i): None for i in '[]'}).split() #get point data, remove unwantes chars, and split
            pt1 = list(map(int, pt1))
            pt2 = data[4].translate({ord(i): None for i in '[]'}).split()
            pt2 = list(map(int, pt2))
            pt3 = data[6].translate({ord(i): None for i in '[]'}).split()
            pt3 = list(map(int, pt3))
            pt4 = data[8].translate({ord(i): None for i in '[]'}).split()
            pt4 = list(map(int, pt4))
            marker = {'seq':seq, 'tstamp':tstamp, 'pt1':pt1, 'pt2':pt2, 'pt3':pt3, 'pt4':pt4 }
            markers_data.append(marker)
            print ("\nseq is ", seq, "\ntstamp is ", tstamp, "\npt1:", pt1, " pt2:", pt2, " pt3:", pt3, " pt4:", pt4)
            
    #3. ROS configuration   
    rospy.init_node('marker_map_creator', anonymous=True)    
    
    publishers = {}
    publishers_list = []
    for i in range(0,len(markers_data)):
        publishers["marker1_pub_"+str(i)]   = rospy.Publisher("/mk1_"+str(i), Marker,  queue_size = 2, latch=True)
        publishers["marker2_pub_"+str(i)]   = rospy.Publisher("/mk2_"+str(i), Marker,  queue_size = 2, latch=True)
        publishers["marker3_pub_"+str(i)]   = rospy.Publisher("/mk3_"+str(i), Marker,  queue_size = 2, latch=True)
        publishers["marker4_pub_"+str(i)]   = rospy.Publisher("/mk4_"+str(i), Marker,  queue_size = 2, latch=True)
        publishers["markerID_pub_"+str(i)] = rospy.Publisher("/mkID"+str(i),Marker, queue_size = 2, latch=True)
        publishers["markerIDm_pub_"+str(i)] = rospy.Publisher("/mkIDm"+str(i),Marker, queue_size = 2, latch=True)
        publishers["br_rgbd_"+str(i)]  = tf2_ros.StaticTransformBroadcaster()
        publishers["br_base_"+str(i)] = tf2_ros.StaticTransformBroadcaster()
        listener = tf.TransformListener()
        publishers_list.append(publishers)
    
    rate = rospy.Rate(1) # 3 Hz
    time = rospy.Time.now()
    
    #4. Extract depth img from bagfile
    markers_center_xyz = []
    markers_data_rviz    = []
    depth_bag = rosbag.Bag(input_depth_bag)
    for i in range(0,len(markers_data)):
        print("############################")
        print("Marker's ID",str(i),"Points Procesing")
        print("############################")
        
        for topic, msg, t in depth_bag.read_messages(topics=[input_depth_img_topic]):
            if msg.header.stamp == markers_data[i]['tstamp']:
                print("EXTRACTING DEPTH_IMG: Tstamp marker: ", markers_data[i]['tstamp'], "matches img_depth: ",  msg.header.stamp)
                print("seq:", markers_data[i]['seq'], "pt1:", markers_data[i]['pt1'], " pt2:", markers_data[i]['pt2'], " pt3:", markers_data[i]['pt3'], " pt4:", markers_data[i]['pt4']) 
                
                #4.1 Convert ros img msg to cv_img
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8") 
                
                #5. Compute img center
                pt1 = markers_data[i]['pt1']
                pt2 = markers_data[i]['pt2'] 
                pt3 = markers_data[i]['pt3']
                pt4 = markers_data[i]['pt4']      
                corners = np.array([pt1, pt2, pt3, pt4], np.int32)
                corners = corners.reshape((-1,1,2))
                sorted_corners, center = sortCorners(corners) #hay que calcular el centro ya que el mapa de marcas usa el centro del rectangulo
                print("center:", [center[0], center[1]])
                
                 #6. Draw square and points in the img, export image 
                cv2.polylines(cv_image,[corners],True,(255,0,0)) #draw square, blue bgr
                cv2.circle(cv_image, (pt1[0], pt1[1]), radius=0, color=(0, 255, 0), thickness=5) #draw corners, green
                cv2.circle(cv_image, (pt2[0], pt2[1]), radius=0, color=(255, 0, 0), thickness=5) #blue
                cv2.circle(cv_image, (pt3[0], pt3[1]), radius=0, color=(0, 255, 255), thickness=5) #yellow
                cv2.circle(cv_image, (pt4[0], pt4[1]), radius=0, color=(0, 0, 255), thickness=5) #red           
                path = output_map_path
                rectangle_export = cv2.imwrite(path+'sortcorners_img_'+str(i)+'.bmp',cv_image) #TODO exporta fotos
                
                #7. Get pixel values and compute position in space XYZ  
                center_pixel = cv_image[center[0], center[1]][0]       
                center_xyz = pixel_to_point_in_space(center_pixel, center[0], center[1], cx, cy, fx, fy)
                print("center_pixel: ", center_pixel, "center xyz points are: ", center_xyz)
                
                pt1_pixel = cv_image[pt1[0], pt1[1]][0]       
                pt1_xyz = pixel_to_point_in_space(pt1_pixel, pt1[0], pt1[1], cx, cy, fx, fy)
                print("pt1_pixel: ", pt1_pixel, "pt1 xyz points are: ", pt1_xyz)
                
                pt2_pixel = cv_image[pt2[0], pt2[1]][0]  
                pt2_xyz = pixel_to_point_in_space(pt2_pixel, pt2[0], pt2[1], cx, cy, fx, fy)
                print("pt2_pixel: ", pt2_pixel, "pt2 xyz points are: ",pt2_xyz)
                
                pt3_pixel = cv_image[pt3[0], pt3[1]][0]  
                pt3_xyz = pixel_to_point_in_space(pt3_pixel, pt3[0], pt3[1], cx, cy, fx, fy)
                print("pt3_pixel: ", pt3_pixel, "pt3 xyz points are: ",pt3_xyz)
                
                pt4_pixel = cv_image[pt4[0], pt4[1]][0]  
                pt4_xyz = pixel_to_point_in_space(pt4_pixel, pt4[0], pt4[1], cx, cy, fx, fy)
                print("pt4_pixel: ", pt4_pixel, "pt4 xyz points are: ",pt4_xyz)
                
                #8. Get position and rotation of the fixed frame of the RGBD camera 
                for topic, msg, t in depth_bag.read_messages(topics=["/tf"]):
                    msg = msg.transforms
                    rgbd_tf = create_TFmsg(msg[0].transform.translation.x, msg[0].transform.translation.y, msg[0].transform.translation.z, msg[0].transform.rotation, msg[0].header.frame_id, msg[0].child_frame_id)
                    yaw= euler_from_quaternion([rgbd_tf.transform.rotation.x, rgbd_tf.transform.rotation.y, rgbd_tf.transform.rotation.z, rgbd_tf.transform.rotation.w])
                    print("EXTRACTING RGBD FRAME: RGBD1 pose in this moment:", rgbd_tf.header.frame_id, rgbd_tf.child_frame_id, rgbd_tf.transform.translation.x, rgbd_tf.transform.translation.y, rgbd_tf.transform.translation.z, "rotation: ", yaw)               
                    break #this tf is static so with just reading the 1rst msg is enough
                    
                #9. Extract pose of base_link in the tstamp where the depth_img was taken
                gtruth_bag = rosbag.Bag(input_gtruth_bag)
                for topic, gt_msg, t in gtruth_bag.read_messages(topics=["tf"]):
                    tf_msg = gt_msg.transforms
                    #9.1 Find the closest tstamp in the gtruth data to the tstamp when the img was taken
                    if tf_msg[0].header.stamp > markers_data[i]['tstamp']: 
                        print("gtruth tstamp: ", tf_msg[0].header.stamp, " close to img_depth: ", markers_data[i]['tstamp'])
                        gtruth_tf = create_TFmsg( tf_msg[0].transform.translation.x,  tf_msg[0].transform.translation.y,  tf_msg[0].transform.translation.z,  tf_msg[0].transform.rotation,  tf_msg[0].header.frame_id,  tf_msg[0].child_frame_id)
                        yaw= euler_from_quaternion([gtruth_tf.transform.rotation.x, gtruth_tf.transform.rotation.y, gtruth_tf.transform.rotation.z, gtruth_tf.transform.rotation.w])
                        print("EXTRACTING BASE_LINK FRAME AND POSE: base_link(gtruth) pose in this moment:", gtruth_tf.header.frame_id, gtruth_tf.child_frame_id, gtruth_tf.transform.translation.x, gtruth_tf.transform.translation.y, gtruth_tf.transform.translation.z, "rotation: ", yaw)
                       
                        #10. Convert points to marker msg 
                        pt1_pub      = create_marker(pt1_xyz[0], pt1_xyz[1], pt1_xyz[2], time, rgbd_tf.child_frame_id[1:]+str(i), ColorRGBA(1, 0, 0, 0.5)) #red   #rgbd_tf.child_frame_id[1:] fix to take '/' from beggining of tf
                        pt2_pub      = create_marker(pt2_xyz[0], pt2_xyz[1], pt2_xyz[2], time, rgbd_tf.child_frame_id[1:]+str(i), ColorRGBA(0, 1, 0, 0.5))#green
                        pt3_pub      = create_marker(pt3_xyz[0], pt3_xyz[1], pt3_xyz[2], time, rgbd_tf.child_frame_id[1:]+str(i), ColorRGBA(0, 0, 1, 0.5))#blue
                        pt4_pub      = create_marker(pt4_xyz[0], pt4_xyz[1], pt4_xyz[2], time, rgbd_tf.child_frame_id[1:]+str(i), ColorRGBA(1, 1, 0, 0.5))#yellow
                        ptID_pub    = create_marker_text(center_xyz[0]+0.3, center_xyz[1]+0.2, center_xyz[2], time, rgbd_tf.child_frame_id[1:]+str(i), ColorRGBA(1, 0, 0, 5), "ID"+str(i))#red
                        ptIDm_pub = create_marker(center_xyz[0], center_xyz[1], center_xyz[2], time, rgbd_tf.child_frame_id[1:]+str(i), ColorRGBA(1, 0, 0, 5))#red
                        
                        tf_rgb1   = create_static_tf(  rgbd_tf.transform.translation.x,   rgbd_tf.transform.translation.y,   rgbd_tf.transform.translation.z ,  rgbd_tf.transform.rotation, time,   rgbd_tf.header.frame_id+str(i),   rgbd_tf.child_frame_id+str(i))
                        tf_gtruth = create_static_tf(gtruth_tf.transform.translation.x, gtruth_tf.transform.translation.y, gtruth_tf.transform.translation.z ,gtruth_tf.transform.rotation, time, gtruth_tf.header.frame_id, gtruth_tf.child_frame_id+str(i))
                              
                        #10. Append all data in a list of dictionaries 
                        marker_rviz = {'tf_rgb':tf_rgb1, 'tf_gtruth':tf_gtruth, 'ptID_pub':ptID_pub, 'ptIDm_pub':ptIDm_pub, 'pt1_pub':pt1_pub, 'pt2_pub':pt2_pub, 'pt3_pub':pt3_pub, 'pt4_pub':pt4_pub}
                        markers_data_rviz.append(marker_rviz)  
                        #print("el marker del punto 1 es ", pt1_pub.header.frame_id, pt1_pub.pose.position.x, pt1_pub.pose.position.y, pt1_pub.pose.position.z, pt1_pub.pose.orientation.x, pt1_pub.pose.orientation.y,pt1_pub.pose.orientation.z,pt1_pub.pose.orientation.w)                                           
                        markers_center_xyz.append(center_xyz)                        
                        break
                gtruth_bag.close()
    print("markers_data len is", len(markers_data_rviz)) 
    depth_bag.close()

    
    centers_tfmap_list = []
    export_yaml_file = True 
    #12. Start spinning node 
    while not rospy.is_shutdown():     
        for i in range(0,len(markers_data)):
            publishers_list[i]["marker1_pub_"+str(i)].publish(markers_data_rviz[i]["pt1_pub"]) ###pt1_pub por markers_data_rviz
            publishers_list[i]["marker2_pub_"+str(i)].publish(markers_data_rviz[i]["pt2_pub"])    
            publishers_list[i]["marker3_pub_"+str(i)].publish(markers_data_rviz[i]["pt3_pub"])    
            publishers_list[i]["marker4_pub_"+str(i)].publish(markers_data_rviz[i]["pt4_pub"])    
            publishers_list[i]["markerID_pub_"+str(i)].publish(markers_data_rviz[i]["ptID_pub"])
            publishers_list[i]["markerIDm_pub_"+str(i)].publish(markers_data_rviz[i]["ptIDm_pub"])             
            publishers_list[i]["br_rgbd_"+str(i)] .sendTransform(markers_data_rviz[i]["tf_rgb"])  
            publishers_list[i]["br_base_"+str(i)].sendTransform(markers_data_rviz[i]["tf_gtruth"]) 
            try:
                #13. Compute the transform to world for center point 
                (trans,rot) = listener.lookupTransform(markers_data_rviz[i]["tf_rgb"].child_frame_id, markers_data_rviz[i]["tf_rgb"].header.frame_id ,rospy.Time(0))
                p1 = geometry_msgs.msg.PoseStamped() #Convert points to pose msg to do the frame transformation  
                p1.header.frame_id = markers_data_rviz[i]["tf_rgb"].child_frame_id
                p1.pose.position.x = markers_center_xyz[i][0]
                p1.pose.position.y = markers_center_xyz[i][1]
                p1.pose.position.z = markers_center_xyz[i][2]
                #p_in_base = listener.transformPose( markers_data_rviz[i]["tf_rgb"].header.frame_id, p1)#Aqui se transforma el TF a map    
                p_in_base = listener.transformPose( "map", p1)#Aqui se transforma el TF a map    
                p_in_base_rpy = euler_from_quaternion([p_in_base.pose.orientation.x, p_in_base.pose.orientation.y, p_in_base.pose.orientation.z, p_in_base.pose.orientation.w])
                print("the trans listened is \nx: ", p_in_base.pose.position.x, " y: ", p_in_base.pose.position.y, " z: ", p_in_base.pose.position.z, "roll, pitch, yaw: ", p_in_base_rpy)                
                centers_tfmap = {'x': float(p_in_base.pose.position.x), 'y': float(p_in_base.pose.position.y), 'z': float(p_in_base.pose.position.z), 'roll':p_in_base_rpy[0], 'pitch':p_in_base_rpy[1], 'yaw': p_in_base_rpy[2], 'ID': i}
                if export_yaml_file == True:
                    centers_tfmap_list.append(centers_tfmap) #data for yaml map file 
                    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        #14. Create and export data to yaml file to create markers map 
        if export_yaml_file == True and len(centers_tfmap_list)==len(markers_data):
            with open(output_map_path+'markers_map.yml', 'w') as f:
                yaml.dump({"marker_positions": centers_tfmap_list}, f, default_flow_style=False, sort_keys=False)
                export_yaml_file = False #variable to export yaml file only once
                print("Markers map published in: ", output_map_path)
        rate.sleep()


if __name__ == "__main__":
    main(sys.argv)
