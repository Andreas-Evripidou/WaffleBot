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
        
        if self.counter > 10:
            self.counter = 0
            print ("X Y Z")
            print("x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.x,self.y,(self.theta_z * 180/math.pi)))
            print(self.second_lap)
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

        self.rate = rospy.Rate(1) # hz

        self.counter = 0

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel_cmd = Twist()

        self.startup = True

        #self.second_lap = False

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
        while not self.ctrl_c:
            self.laps = 0
            # specify the radius of the circle:
            path_rad = 0.5 # m
            # linear velocity must be below 0.26m/s:
            lin_vel = 0.2 # m/s

            self.vel_cmd.linear.x = lin_vel
            self.vel_cmd.angular.z = lin_vel / path_rad # rad/s

            #if  self.theta_z > -0.100 and self.theta_z < - 0.90:
            #    self.vel_cmd.angular.z = 0
            #    self.vel_cmd.linear = 0
            #    self.second_lap == True
            
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass