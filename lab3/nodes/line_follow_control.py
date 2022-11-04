#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from std_msgs.msg import Float64



class Controller(object):
    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )
        self.state_sub = rospy.Subscriber(
            "state", Float64, self.state_callback, queue_size=1
        )
        self.rate = rospy.Rate(30) #10Hz
        self.dt = 1/30
        #odom_subscriber=rospy.Subscriber('odom', Odometry, callback,queue_size=1)
        

        self.max_v = 0.26
        self.v = 0.1 #0.26
        # max angular z 1.82
        self.omega_max = 1.82
        self.omega_0 = 0.3

        # controller parameters
        self.desired_x = 320
        
        # just barely works
        # self.ku = 0.02/2
        # self.kp = self.ku * 0.6
        # self.ki = self.ku
        # self.kd = self.ku / 4
        
        # good performance on slalom
        # self.kp = 0.006
        # self.ki = 0.006
        # self.kd = 0.001

        # # better performance on slalom
        # self.kp = 0.0065
        # self.ki = 0.008
        # self.kd = 0.001

        # for variable speed -- sharp turn track
        # self.kp = 0.009
        # self.ki = 0.012
        # self.kd = 0.0005

        # FINAL VALUES FOR RACE
        self.kp = 0.007
        self.ki = 0.010
        self.kd = 0.001
        # a bit jittery but took home the dub


        self.error_x = 0
        self.error_integral = 0 # freq = 30 hz
        self.error_derivative = 0
        rospy.sleep(2)

    # def camera_callback(self, msg):
    #     """Callback for line index."""
    #     #self.prev_line_x = self.cur_line_x
    #     #print(msg)
    #     error_prev = self.error_x
    #     self.error_x =self.desired_x- msg.data
    #     self.error_integral = self.error_x * self.dt
    #     self.error_derivative = (self.error_derivative + (self.error_x - error_prev) / self.dt) / 2 # divide by dt
    #     return

    def state_callback(self, msg):
        e = 0.05
        self.checkpoints=[61,122,244,305]
        self.checkpoints_reached = [0,0,0,0]

        for i in range(self.checkpoints):
            if(msg<(self.checkpoints[i]+e) and msg>(self.checkpoints[i]-e) and self.checkpoints_reached == 0):
                at_checkpoint = 1
                self.checkpoints_reached[i] = 1
                ros.sleep(2)

        
    def camera_callback(self, msg):

        """Callback for line index."""
        #self.prev_line_x = self.cur_line_x
        #print(sg)
        error_prev = self.error_x
        self.error_x =self.desired_x- msg.data
       
        if (abs(self.error_x) > 100):
            self.v = 0.5*self.v # or you could proportionally decrease the velocity based on self.error_x
        else:
            self.v=self.max_v

        self.error_integral = self.error_x * self.dt
        self.error_derivative = (self.error_x - error_prev) / self.dt # divide by dt
        return

    def follow_the_line(self):
        while not rospy.is_shutdown():
            omega = \
                self.kp * self.error_x + \
                self.ki * self.error_integral + \
                self.kd * self.error_derivative
            
            twist=Twist()
            twist.linear.x=self.v
            twist.angular.z=omega
            self.cmd_pub.publish(twist)
            self.rate.sleep()

    def pause(t):
        t_start = rospy.get_rostime()
        # SET V = 0??
        while not rospy.is_shutdown() and rospy.get_rostime().secs - t_start < t:
            rate.sleep()
        



if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line()
