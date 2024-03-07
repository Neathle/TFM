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


ODOM_GT_T = [
      7.34e9,
     14.68e9,
     26.42e9,
     32.28e9,
     41.08e9,
     53.02e9,
     61.85e9,
     83.94e9,
     92.75e9,
    148.54e9,
    160.46e9,
    184.07e9,
    204.61e9,
    225.11e9,
    242.72e9,
    270.68e9,
    283.88e9,
    291.00e9,
]
ODOM_GT_K = [
  1.569/ 1.542,
  1.569/ 1.542,
  3.984/ 3.850,
  3.984/ 3.850,
  5.659/ 5.550,
  5.659/ 5.550,
  7.690/ 7.585,
  7.690/ 7.585,
  9.624/ 9.492,
  9.624/ 9.492,
 12.272/12.127,
 12.272/12.127,
 17.053/16.872,
 17.053/16.872,
 21.348/21.000,
 21.348/21.000,
 24.610/24.253,
 24.610/24.253,
]


init_time = Time(0)
init_time_isset = False
def publish_odom_gt(msg: Odometry, t: Time, outbag: rosbag.bag.Bag):
    gt = deepcopy(msg)
    gt.header.frame_id = "map"
    k = np.interp((t-init_time).to_nsec(), ODOM_GT_T, ODOM_GT_K)
    gt.pose.pose.position.x *= k
    gt.pose.pose.position.y = 0
    gt.pose.pose.position.z = 0
    gt.pose.pose.orientation.x = 0
    gt.pose.pose.orientation.y = 0
    gt.pose.pose.orientation.z = 0
    
    outbag.write("/odom_gt", gt, t)
    outbag.write("/odom", msg, t)
    return

tf_static_msg = None

with rosbag.Bag("/home/neathle/TFM/rosbags/hallway_gt.bag", "w") as outbag:
    input_bag = rosbag.Bag("/home/neathle/TFM/rosbags/hallway_2024-02-09-17-50-04.bag")
    for topic, msg, t, ch in tqdm(
        input_bag.read_messages(return_connection_header=True),
        total=input_bag.get_message_count(),
    ):
        if not init_time_isset:
            init_time = t
            init_time_isset = True

        if topic == "/dlo_node/odom":
            publish_odom_gt(msg, t, outbag)
            continue

        outbag.write(topic, msg, t, connection_header=ch)
