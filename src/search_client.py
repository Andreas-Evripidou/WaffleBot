#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from com2009_msgs.msg import  SearchFeedback, SearchGoal, SearchAction

class SearchClient():
   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance_travelled = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance: {self.distance_travelled:.1f} meters. ")

    def callback(self, odom_data):
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

    def __init__(self):
        self.distance_travelled = 0
        self.action_complete = False

        node_name = "find_box_action_client"
        action_server_name = "/find_box_action_server"
        
        rospy.init_node(node_name)

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback)

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0

        self.vel_cmd = Twist()

        self.rate = rospy.Rate(1)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    SearchAction)
        self.client.wait_for_server()

        self.ctrl_c = False

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            print(f"RESULT: {self.distance_travelled} m travelled")
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0


    def send_goal(self, velocity, distance):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = distance
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        while not self.ctrl_c:
            self.send_goal(velocity = 0.26, distance = 0.5)
            self.client.wait_for_result()
            result = self.client.get_result()
            self.action_complete = True

            temp_z = self.theta_z
            turn_speed = 1

            if (result.closest_object_angle <= 0):
                while (abs(self.theta_z - temp_z) < 1.2):
                    self.vel_cmd.angular.z = -turn_speed
                    self.pub.publish(self.vel_cmd)
                
            else:
                while (abs(self.theta_z - temp_z) < 1.2):
                    self.vel_cmd.angular.z = turn_speed
                    self.pub.publish(self.vel_cmd)
            self.vel_cmd.angular.z = 0
            self.pub.publish(self.vel_cmd)
        
            print(f"RESULT: Action State = {self.client.get_state()}")

if __name__ == '__main__':
    action_instance = SearchClient()
    try:
        action_instance.main()
    except rospy.ROSInterruptException:
        pass