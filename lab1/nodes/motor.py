#!/usr/bin/env python
import sys
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    rospy.init_node("motor_node")
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10) #10Hz
    time0 = rospy.get_rostime().secs


    while not rospy.is_shutdown() and rospy.get_rostime().secs - time0 < 3:
        twist=Twist()
        twist.linear.x=-0.2
        twist.angular.z=0.0
        cmd_pub.publish(twist)
        rate.sleep()

    # time0 = rospy.get_rostime().secs
    # while not rospy.is_shutdown() and rospy.get_rostime().secs - time0 < 5:
    #     twist=Twist()
    #     twist.linear.x=0.0
    #     twist.angular.z=6.1/5
    #     cmd_pub.publish(twist)
    #     rate.sleep()


if __name__ == "__main__":
    main()
