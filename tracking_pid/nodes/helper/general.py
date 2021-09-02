
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import tf
import math
import rospy


def get_goal(goal_list, idx):
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "/map"
    goal.pose.position.x = goal_list[idx][0]
    goal.pose.position.y = goal_list[idx][1]
    goal.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(
        0, 0, goal_list[idx][2])
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    return goal


def distance_to_goal(x_c, y_c, x_g, y_g):
    return math.sqrt((x_c - x_g)**2 + (y_c - y_g)**2)


def dy(distance, m):
    return m*dx(distance, m)

def dx(distance, m):
    return math.sqrt(distance/(m**2+1))

def get_second_point(point_a, point_c):
    distance = 1
    m = (point_c[-1] - point_a[-1]) / (point_c[0] - point_a[0])
    p1 = (point_a[0]+dx(distance,m), point_a[1]+dy(distance,m))
    p2 = (point_a[0]-dx(distance,m), point_a[1]-dy(distance,m))
    if distance_to_goal(p1[0],p1[1],point_a[0],point_a[1]) + distance_to_goal(p1[0],p1[1],point_c[0],point_c[1]) == distance_to_goal(point_a[0], point_a[1],point_c[0], point_c[1]):
        return p1
    else:
        return p2 

def generate_path(i, obj):
    if obj.path_is == 'turn':
        obj.point_b = get_second_point(obj.section[-1], obj.waypoints[i + 2])
        section_1 = list((obj.section[0]))
        section_2 = list(obj.section[1])
        section_3 = list(obj.point_b)
        obj.section = []
        obj.section.append(section_1)
        obj.section.append(section_2)
        obj.section.append(section_3)
    elif obj.path_is == 'straight' and i != 0:
        obj.section = list(obj.section)
        obj.section[0] = obj.point_b

    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    for wp in obj.section:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        msg.poses.append(pose)
    return msg, obj.section, obj.point_b

def get_path_type(section):
    first_point = section[0]
    second_point = section[-1]
    x1 = first_point[0]
    y1 = first_point[1]
    x2 = second_point[0]
    y2 = second_point[1]
    dist = distance_to_goal(x1,y1,x2,y2)
    if dist >= 3:
        return "straight"
    else:
        return "turn"