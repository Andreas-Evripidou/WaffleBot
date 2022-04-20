#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion


class Maze:

    def odom_callback(self, odom_data):
        orientation_x = odom_data.pose.pose.orientation.x
        orientation_y = odom_data.pose.pose.orientation.y
        orientation_z = odom_data.pose.pose.orientation.z
        orientation_w = odom_data.pose.pose.orientation.w

        position_x = odom_data.pose.pose.position.x
        position_y = odom_data.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')

        self.x = position_x
        self.y = position_y
        self.theta_z = yaw


    def scan_callback(self, scan_data):
        # From the front of the robot, obtain a 20 degree 
        # arc of scan data either side of the x-axis 
        left_arc = scan_data.ranges[0:91]
        right_arc = scan_data.ranges[-90:]
        # combine the "left_arc" and "right_arc" data arrays, flip them so that 
        # the data is arranged from left (-20 degrees) to right (+20 degrees)
        # then convert to a numpy array
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        # Obtain the minimum distance measurement within the "front_arc" array
        # to indicate when we are getting close to something up ahead:
        self.min_distance = self.front_arc.min()

        # Advanced:
        # Create another numpy array which represents the angles 
        # (in degrees) associated with each of the data-points in 
        # the "front_arc" array above:
        arc_angles = np.arange(-90, 91)
        # determine the angle at which the minimum distance value is located
        # in front oself.arc_anglesf the robot:
        self.object_angle = arc_angles[np.argmin(self.front_arc)]



    def __init__(self):

        self.min_distance = 100
        self.front_arc = np.empty(180)

        self.node_name = "maze_navigator"

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        
        self.vel_cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        print("stopping the robot")
        self.pub.publish(Twist())
        self.ctrl_c = True
    

    def main_loop(self):

        while not self.ctrl_c:

            right_side = self.front_arc[120:150]
            front_side = self.front_arc[75:105]
            min_right_side = np.amin(right_side)
            min_front_side = np.amin(front_side)

            if (min_front_side < 0.3):
                self.vel_cmd.angular.z = 0
                self.vel_cmd.linear.x = 0
                self.pub.publish(self.vel_cmd)
                self.vel_cmd.angular.z = 0.2
                self.pub.publish(self.vel_cmd)
            else:        
                if(min_right_side > 0.3 and min_right_side < 0.4):
                    self.vel_cmd.angular.z = 0
                    self.vel_cmd.linear.x = 0.1
                    self.pub.publish(self.vel_cmd)
                elif(min_right_side < 0.3):
                    self.vel_cmd.linear.x = 0.05
                    self.vel_cmd.angular.z = 0.2
                    self.pub.publish(self.vel_cmd)
                elif(min_right_side > 0.4):
                    self.vel_cmd.linear.x = 0.05
                    self.vel_cmd.angular.z = -0.2
                    self.pub.publish(self.vel_cmd)

            
    
                
            self.pub.publish(self.vel_cmd)
        
            #print (self.front_arc[180])
            self.rate.sleep()
        
        
    
if __name__ == '__main__':
    maze_instance = Maze()
    try:
        maze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass