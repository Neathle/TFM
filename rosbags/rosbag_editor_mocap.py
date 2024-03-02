import rosbag
from rospy import Time, Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from loguru import logger
from copy import deepcopy
from tqdm import tqdm


# topics:      /base_imu                         4191 msgs    : sensor_msgs/Imu
#              /mobile_base_controller/odom     21259 msgs    : nav_msgs/Odometry
#              /scan                             6378 msgs    : sensor_msgs/LaserScan
#              /tf                              57058 msgs    : tf2_msgs/TFMessage      (2 connections)
#              /tf_static                           3 msgs    : tf2_msgs/TFMessage      (3 connections)
#              /xtion/depth_registered/points    3836 msgs    : sensor_msgs/PointCloud2
#              /xtion/rgb/image_rect_color      12775 msgs    : sensor_msgs/Image

previous_time: Time

def create_and_save_pose_msg(
    transform: TransformStamped, t: Time, outbag: rosbag.bag.Bag
):
    # Create a new PoseStamped message
    pose = PoseStamped()
    pose.header.stamp = transform.header.stamp
    pose.header.frame_id = "world"  # mocap reference

    # Copy the position from the tf message
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z

    # Copy the orientation from the tf message
    pose.pose.orientation.x = transform.transform.rotation.x
    pose.pose.orientation.y = transform.transform.rotation.y
    pose.pose.orientation.z = transform.transform.rotation.z
    pose.pose.orientation.w = transform.transform.rotation.w

    outbag.write("/mocap_pose", pose, t)


def modify_tf_message(msg: TFMessage, t: Time, outbag: rosbag.bag.Bag):
    filtered_msg = deepcopy(msg)
    filtered_msg.transforms = []
    for transform in msg.transforms:
        # logger.info(transform.header.frame_id + " -> " + transform.child_frame_id)
        if transform.child_frame_id == "Robot_1/base_link":
            transform.header.stamp += Duration(secs=94, nsecs=240000000)
            create_and_save_pose_msg(transform, t, outbag)
        if transform.child_frame_id != "Robot_2/base_link":
        # and not (
        #     "arm_" in transform.child_frame_id or "gripper_" in transform.child_frame_id or
        # ):
            filtered_msg.transforms.append(transform)  
        
        # if tf_static_msg:
        #     outbag.write("/tf", tf_static_msg, t + Duration(nsecs=1))

    return filtered_msg


def modify_tf_static_message(msg: TFMessage):
    filtered_msg = deepcopy(msg)
    filtered_msg.transforms = []
    for transform in msg.transforms:
        if transform.child_frame_id == "Robot_1/base_footprint":
            logger.info(transform.header.frame_id + " -> " + transform.child_frame_id)
            return None
        if transform.header.frame_id == "Robot_1/base_footprint":
            logger.info(transform.header.frame_id + " -> " + transform.child_frame_id)
            return None

        if transform.child_frame_id == "base_link":
            filtered_msg.transforms.append(transform)

    # Map -> World
    tm = TransformStamped()
    tm.header.stamp = msg.transforms[0].header.stamp
    tm.header.frame_id = "map"
    tm.child_frame_id = "world"
    tm.transform.translation.x = 0
    tm.transform.translation.y = 0
    tm.transform.translation.z = 0
    tm.transform.rotation.x = 0
    tm.transform.rotation.y = 0
    tm.transform.rotation.z = 0.7071068
    tm.transform.rotation.w = 0.7071068
    filtered_msg.transforms.append(tm)

    # base_link -> base_laser_link
    tm1 = TransformStamped()
    tm1.header.stamp = msg.transforms[0].header.stamp
    tm1.header.frame_id = "base_link"
    tm1.child_frame_id = "base_laser_link"
    tm1.transform.translation.x = 0.202
    tm1.transform.translation.y = 0
    tm1.transform.translation.z = -0.004
    tm1.transform.rotation.x = 0
    tm1.transform.rotation.y = 0
    tm1.transform.rotation.z = 0
    tm1.transform.rotation.w = 1
    filtered_msg.transforms.append(tm1)


    # base_link -> xtion_rgb_optical_frame
    tm2 = TransformStamped()
    tm2.header.stamp = msg.transforms[0].header.stamp
    tm2.header.frame_id = "base_link"
    tm2.child_frame_id = "xtion_rgb_optical_frame"
    tm2.transform.translation.x = 0.222301203927
    tm2.transform.translation.y = 0.017722121083
    tm2.transform.translation.z = 1.112804230859
    tm2.transform.rotation.x = 0.497653755889
    tm2.transform.rotation.y = -0.503970958085
    tm2.transform.rotation.z = 0.508199910040
    tm2.transform.rotation.w = -0.489986595828
    filtered_msg.transforms.append(tm2)

    return filtered_msg


def publish_odom_frame(msg: Odometry, t: Time, outbag: rosbag.bag.Bag):
    tm = TransformStamped()
    tf = TFMessage()

    tm.header.stamp = t + Duration(secs=94, nsecs=240000000)
    tm.header.frame_id = "odom"
    tm.child_frame_id = "base_footprint"
    tm.transform.translation.x = msg.pose.pose.position.x
    tm.transform.translation.y = msg.pose.pose.position.y
    tm.transform.translation.z = msg.pose.pose.position.z
    tm.transform.rotation.x = msg.pose.pose.orientation.x
    tm.transform.rotation.y = msg.pose.pose.orientation.y
    tm.transform.rotation.z = msg.pose.pose.orientation.z
    tm.transform.rotation.w = msg.pose.pose.orientation.w

    tf.transforms.append(tm)
    outbag.write("/tf", tf, t)
    return

tf_static_msg = None

with rosbag.Bag("/home/neathle/TFM/rosbags/mocap.bag", "w") as outbag:
    input_bag = rosbag.Bag("/home/neathle/TFM/rosbags/2023-09-06-17-32-12.bag")
    for topic, msg, t, ch in tqdm(
        input_bag.read_messages(return_connection_header=True),
        total=input_bag.get_message_count(),
    ):
        if topic == "/tf":
            msg = modify_tf_message(msg, t, outbag)

        elif topic == "/tf_static":
            msg = modify_tf_static_message(msg)
            tf_static_msg = msg
            if not msg:
                continue

        elif topic == "/mobile_base_controller/odom":
            publish_odom_frame(msg, t, outbag)

        outbag.write(topic, msg, t, connection_header=ch)

# base_link -> base_laser_link
# - Translation: [0.202000000000, 0.000000000000, -0.004000000000]
# - Rotation: in Quaternion [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]

# base_link -> xtion_rgb_optical_frame
# - Translation: [0.222301203927, 0.017722121083, 1.112804230859]
# - Rotation: in Quaternion [0.497653755889, -0.503970958085, 0.508199910040, -0.489986595828]