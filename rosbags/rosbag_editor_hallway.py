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
  1.649/ 1.542,
  1.649/ 1.542,
  4.064/ 3.850,
  4.064/ 3.850,
  5.739/ 5.550,
  5.739/ 5.550,
  7.770/ 7.585,
  7.770/ 7.585,
  9.704/ 9.492,
  9.704/ 9.492,
 12.352/12.127,
 12.352/12.127,
 17.133/16.872,
 17.133/16.872,
 21.428/21.000,
 21.428/21.000,
 24.690/24.253,
 24.690/24.253,
]


init_time = Time(0)
init_time_isset = False
def publish_odom_gt(msg: Odometry, t: Time, outbag: rosbag.bag.Bag):
    gt = deepcopy(msg)
    gt.header.frame_id = "odom_gt"
    k = np.interp((t-init_time).to_nsec(), ODOM_GT_T, ODOM_GT_K)
    gt.pose.pose.position.x *= k
    gt.pose.pose.position.y /= 5*k
    
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

# base_link -> base_laser_link
# - Translation: [0.202000000000, 0.000000000000, -0.004000000000]
# - Rotation: in Quaternion [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]

# base_link -> xtion_rgb_optical_frame
# - Translation: [0.222301203927, 0.017722121083, 1.112804230859]
# - Rotation: in Quaternion [0.497653755889, -0.503970958085, 0.508199910040, -0.489986595828]