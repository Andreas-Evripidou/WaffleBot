#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist

from com2009_msgs.msg import  SearchFeedback, SearchGoal, SearchAction

class SearchClient():
   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance_travelled = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance: {self.distance_travelled:.1f} meters. ")

    def __init__(self):
        self.distance_travelled = 0
        self.action_complete = False

        node_name = "find_box_action_client"
        action_server_name = "/find_box_action_server"
        
        rospy.init_node(node_name)

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.vel_cmd = Twist()

        self.rate = rospy.Rate(1)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            print(f"RESULT: {self.distance_travelled} image(s) saved.")

    def send_goal(self, velocity, distance):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = distance
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(velocity = 0.2, distance = 0.5)
        self.client.wait_for_result()
        print(self.client.get_result)
        self.action_complete = True

        


        self.vel_cmd.linear.x = -0.1
        self.pub.publish(self.vel_cmd)
        
        print(f"RESULT: Action State = {self.client.get_state()}")

if __name__ == '__main__':
    action_instance = SearchClient()
    try:
        action_instance.main()
    except rospy.ROSInterruptException:
        pass