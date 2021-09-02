from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
import tf
import rospy
import numpy as np
from std_msgs.msg import Header


def callback_odom(data):
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    quaternion = (qx, qy, qz, qw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    v = data.twist.twist.linear.x
    w = data.twist.twist.angular.z
    got_new_data = True
    return x, y, yaw, v, w, got_new_data
