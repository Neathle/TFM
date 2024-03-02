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

with rosbag.Bag("/home/neathle/TFM/rosbags/car_el_marker_x.bag", "w") as outbag:
    input_bag = rosbag.Bag("/home/neathle/TFM/rosbags/car_el_marker.bag")
    for topic, msg, t, ch in tqdm(
        input_bag.read_messages(return_connection_header=True),
        total=input_bag.get_message_count(),
    ):
        if topic == "/odom_gt":
            latest_odom_y = msg.pose.pose.position.y

        if topic == "/amcl_pose":
            msg.pose.pose.position.y = latest_odom_y

        outbag.write(topic, msg, t, connection_header=ch)

# base_link -> base_laser_link
# - Translation: [0.202000000000, 0.000000000000, -0.004000000000]
# - Rotation: in Quaternion [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]

# base_link -> xtion_rgb_optical_frame
# - Translation: [0.222301203927, 0.017722121083, 1.112804230859]
# - Rotation: in Quaternion [0.497653755889, -0.503970958085, 0.508199910040, -0.489986595828]