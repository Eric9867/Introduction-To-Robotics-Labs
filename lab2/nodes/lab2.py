#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def callback(odom_data):
    global cur_pose
    point=odom_data.pose.pose.position
    quart=odom_data.pose.pose.orientation
    theta=get_yaw_from_quaternion(quart)
    cur_pose = (point.x, point.y, theta)
    # rospy.loginfo(cur_pose)

rospy.init_node("lab02")
# rospy.init_node("motor_node")
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10) #10Hz
odom_subscriber=rospy.Subscriber('odom', Odometry, callback,queue_size=1)

rospy.sleep(2)
# max v = 0.26
v = 0.1
# max angular z 1.82
omega = 0.15

def main():
    # part1()
    part2()

def part1():
    # rotate to angle arctan(1/4)
    theta = math.atan(1/4)
    t_rot = theta / omega
    t_start = rospy.get_rostime().secs
    print(theta)
    print(t_rot)
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_rot:
        twist=Twist()
        twist.linear.x=0
        twist.angular.z=omega
        cmd_pub.publish(twist)
        rate.sleep()

    dist = math.sqrt(2**2 + 0.5**2)
    t_trans = dist / v
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=0
        cmd_pub.publish(twist)
        rate.sleep()

    # rotate from arctan(1/4) to 3/4 pi
    theta = 3/4 * math.pi - math.atan(1/4)
    t_rot = theta / omega
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_rot:
        twist=Twist()
        twist.linear.x=0
        twist.angular.z=omega
        cmd_pub.publish(twist)
        rate.sleep()

def part2():


    theta = -cur_pose[2]
    rot_mat = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    print(cur_pose)
    pos_init = np.array([cur_pose[0], cur_pose[1]])
    pos = np.array([cur_pose[0], cur_pose[1]])
    pos_q = np.matmul(rot_mat, pos - pos_init)
    print(pos_q)

    
    ### A
    dist = 1 
    t_trans = dist / v
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=0
        cmd_pub.publish(twist)
        rate.sleep()

    print(cur_pose)
    pos = np.array([cur_pose[0], cur_pose[1]])
    pos_q = np.matmul(rot_mat, pos - pos_init)
    print(pos_q)
    print("Error:" + str(np.linalg.norm(pos_q - np.array([1,0]))))
    
    ### B
    theta = math.pi/2
    t_rot = theta / omega
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_rot:
        twist=Twist()
        twist.linear.x=0
        twist.angular.z=omega
        cmd_pub.publish(twist)
        rate.sleep()

    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=0
        cmd_pub.publish(twist)
        rate.sleep()

    pos = np.array([cur_pose[0], cur_pose[1]])
    pos_q = np.matmul(rot_mat, pos - pos_init)
    print(pos_q)
    print("Error:"+ str(np.linalg.norm(pos_q - np.array([1,1]))))

    ### C
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_rot:
        twist=Twist()
        twist.linear.x=0
        twist.angular.z=omega
        cmd_pub.publish(twist)
        rate.sleep()

    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=0
        cmd_pub.publish(twist)
        rate.sleep()

    pos = np.array([cur_pose[0], cur_pose[1]])
    pos_q = np.matmul(rot_mat, pos - pos_init)
    print(pos_q)
    print("Error:"+ str(np.linalg.norm(pos_q - np.array([0,1]))))

    ### D
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_rot:
        twist=Twist()
        twist.linear.x=0
        twist.angular.z=omega
        cmd_pub.publish(twist)
        rate.sleep()

    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=0
        cmd_pub.publish(twist)
        rate.sleep()

    pos = np.array([cur_pose[0], cur_pose[1]])
    pos_q = np.matmul(rot_mat, pos - pos_init)
    print(pos_q)
    print("Error:" + str(np.linalg.norm(pos_q - np.array([0,0]))))


def part3():
    r = 0.50
    omega = v/r
    dist1 = 1.5
    dist2 = pi/2 * r #while turning

    t_trans = dist1 / v
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=0
        cmd_pub.publish(twist)
        rate.sleep()

    t_trans = dist2 / v
    t_start = rospy.get_rostime().secs
    while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t_trans:
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=omega
        cmd_pub.publish(twist)
        rate.sleep()
    

if __name__ == "__main__":
    main()
