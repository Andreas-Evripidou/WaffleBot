#!/usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import numpy as np
from tf.transformations import euler_from_quaternion

# Import some image processing modules:
#import cv2
#from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from com2009_msgs.msg import  SearchFeedback, SearchResult, SearchAction
from nav_msgs.msg import Odometry

# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move

# Import some other useful Python Modules
from math import sqrt, pow


class FindBox(object):
    feedback = SearchFeedback() 
    result = SearchResult()


    def scan_callback(self, scan_data):
        # From the front of the robot, obtain a 20 degree 
        # arc of scan data either side of the x-axis 
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        # combine the "left_arc" and "right_arc" data arrays, flip them so that 
        # the data is arranged from left (-20 degrees) to right (+20 degrees)
        # then convert to a numpy array
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        # Obtain the minimum distance measurement within the "front_arc" array
        # to indicate when we are getting close to something up ahead:
        self.min_distance = front_arc.min()

        # Advanced:
        # Create another numpy array which represents the angles 
        # (in degrees) associated with each of the data-points in 
        # the "front_arc" array above:
        arc_angles = np.arange(-20, 21)
        # determine the angle at which the minimum distance value is located
        # in front of the robot:
        self.object_angle = arc_angles[np.argmin(front_arc)]

    def odom_callback(self,odom_data):
         # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can 
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        # If this is the first time that this callback_function has run, then 
        # obtain a "reference position" (used to determine how far the robot has moved
        # during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):

        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.actionserver = actionlib.SimpleActionServer("/find_box_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.startup = True

        #define the robot pose variables and set them all to zero to start with:
        # variables to use for the "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.robot_controller = Tb3Move()

    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity > 0.26:
            print("Invalid forward velocity. Maximum velocity is 0.26 m/s.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"\n#####\n"
            f"The 'find_box_action_server' has been called.\n"
            f"Goal: Move at a speed of {goal.fwd_velocity} m/s\n\n"
            f"Commencing the action...\n"
            f"#####\n")
    
        self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)

        while self.min_distance > goal.approach_distance : 

            self.robot_controller.publish()
            
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the search.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break

            self.feedback.current_distance_travelled = sqrt(pow(self.x - self.x0,2) + pow((self.y-self.y0),2))
            self.actionserver.publish_feedback(self.feedback)

        if success:
            self.robot_controller.stop()
            rospy.loginfo("Object found sucessfully.")
            self.result.total_distance_travelled = sqrt(pow(self.x - self.x0,2) + pow((self.y-self.y0),2))
            self.result.closest_object_distance = self.min_distance
            self.result.closest_object_angle = self.object_angle
            self.actionserver.set_succeeded(self.result)

            
if __name__ == '__main__':
    rospy.init_node("find_box_action_server")
    FindBox()
    rospy.spin()
