#!/usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
from com2009_msgs.msg import  SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan
from tb3 import Tb3Move,Tb3Odometry
import numpy as np


class FindBox(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        # getting left side point
        self.left_arc_min = np.array(left_arc).min()
        # getting right side point
        self.right_arc_min=np.array(right_arc).min()
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        # getting front side point
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]


    def __init__(self):

        self.actionserver = actionlib.SimpleActionServer("/find_box_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.arc_angles = np.arange(-20, 21)
        self.min_distance = 0
        self.left_arc_min = 0
        self.right_arc_min = 0

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
       

        # Get the current robot odometry:
        self.posx0 = self.robot_odom.posx
        self.posy0 = self.robot_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        
        while success :
        
        
         while self.min_distance > goal.approach_distance:
             self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
             self.robot_controller.publish()
             # check if there has been a request to cancel the action mid-way through:
             if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the search.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                break
                # exit the loop:
         # if there is an object to the left side is too close , then turn right
         while self.left_arc_min <= goal.approach_distance: 
             self.robot_controller.set_move_cmd(0.0, -0.5)
             self.robot_controller.publish()
             
             
         # if there is an object to the left side is too close, then turn left
         while self.right_arc_min<=goal.approach_distance:
             self.robot_controller.set_move_cmd(0.0,0.5)
             self.robot_controller.publish()

        # if there is an object to the left and right side is too close, then 180 degree to the back
         while self.right_arc_min<=0.85 and self.left_arc_min <= 0.85: 
             self.robot_controller.set_move_cmd(0.0,1.5)
             self.robot_controller.publish()
            
if __name__ == '__main__':
    rospy.init_node("find_box_action_server")
    FindBox()
    rospy.spin()