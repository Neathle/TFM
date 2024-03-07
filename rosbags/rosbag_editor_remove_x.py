import rosbag
from rospy import Time, Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from loguru import logger
from copy import deepcopy
from tqdm import tqdm
import numpy as np




latest_odom_y = 0.0

with rosbag.Bag("/home/neathle/TFM/rosbags/car_od_1/car_od_marker.bag", "w") as outbag:
    input_bag = rosbag.Bag("/home/neathle/TFM/rosbags/car_el_2024-03-07-10-06-17.bag")
    for topic, msg, t, ch in tqdm(
        input_bag.read_messages(return_connection_header=True),
        total=input_bag.get_message_count(),
    ):

        if topic == "/amcl_pose":
            msg.pose.pose.position.y = 0
            msg.pose.pose.position.z = 0
            msg.pose.pose.orientation.x = 0
            msg.pose.pose.orientation.y = 0
            msg.pose.pose.orientation.z = 0

        outbag.write(topic, msg, t, connection_header=ch)
