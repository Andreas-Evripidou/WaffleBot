#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist

from com2009_msgs.msg import SearchGoal, SearchAction,SearchFeedback

class SearchClient(object):
   
    def feedback_callback(self, feedback_data:SearchFeedback):
        self.distance_travelled = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance: {self.distance_travelled:.1f} meters. ")
        if self.x < 100:
            self.x += 1
        else:
            self.x = 0


    def __init__(self):
        self.action_complete = False
        
        rospy.init_node("find_box_action_client")

        self.rate = rospy.Rate(1)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient("/find_box_action_server", 
                    SearchAction)
        self.client.wait_for_server()
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.distance = 0.0

        self.x = 0

    def shutdown_ops(self):
         if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(velocity = 0.26, approach = 0.75)
        StartTime = rospy.get_rostime()
        print("the robot will now move for 90 seconds...")
        while self.client.get_state() < 2:
            if rospy.get_rostime().secs - StartTime.secs > 90 :
                rospy.logwarn("Cancelling goal now...")
                self.client.cancel_goal()
                rospy.logwarn("Goal Cancelled")
                rospy.loginfo('90 seconds have elapsed, stopping the robot...')
                break

            self.rate.sleep()
        
        self.ac1tion_complete = True

if __name__ == '__main__':
    ac_object = SearchClient()
    try:
        ac_object.main()
    except rospy.ROSInterruptException:
        pass