#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rospy,math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Circle:

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

        self.true_x = self.x - self.x0
        self.true_y = self.y - self.y0
        self.true_z = self.theta_z - self.theta_z0
        
        if self.counter > 10:
            self.counter = 0
            print("x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.true_x, self.true_y, (self.theta_z * 180/math.pi)-(self.theta_z0 * 180/math.pi)))
            #print("x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.x, self.y, (self.theta_z * 180/math.pi)))
            #print("first lap: ",self.second_lap, "Second lab: ", self.final_lap)
        else:
            self.counter+=1

        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):

        self.node_name = "figure-eight"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

        self.rate = rospy.Rate(10) # hz

        self.counter = 0

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.true_x = 0
        self.true_y = 0
        
        self.vel_cmd = Twist()

        self.startup = True

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)

        self.ctrl_c = True

    def main_loop(self):
        #tick counters dut to high rospy rate
        self.sm = 0 #tick counter for second lap
        self.am = 0 #tick counter for first lap
        
        #Keeping track of the current lap
        self.final_lap = False
        self.second_lap = False
        
        while not self.ctrl_c:
            # specify the radius of the circle:
            path_rad = 0.5 # m
            lin_vel = 0.2 # m/s
           
            #If no laps are completed
            if  not self.second_lap:
                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = lin_vel / path_rad # rad/s
                if math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2) < 0.05 and math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2) > 0 :
                    self.am +=1
                if self.am > 5 :
                    self.second_lap = True

            #If the first laps are completed
            if  self.second_lap:  
                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = - (lin_vel / path_rad) # rad/s
                if math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2) < 0.05 and math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2) > 0 :
                    self.sm +=1 
                #Make sure to not stop early
                if self.sm > 5 :
                    self.final_lap = True

            #If both laps are completed
            if self.final_lap:
                self.ctrl_c = True
            
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass