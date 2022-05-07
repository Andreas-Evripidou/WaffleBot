#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from tf.transformations import euler_from_quaternion
import time
# Import some other modules from within this package
from tb3 import Tb3Move, Tb3Odometry
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import some package for mapping
import roslaunch
from pathlib import Path


class Maze:

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

    def camera_callback(self, img_data):  
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape

        # print(f"Obtained an image of height {height}px and width {width}px.")

        crop_width = width - 400
        crop_height = 100
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height + 500, crop_y0:crop_y0+crop_width]

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
            
        index = 0
        while index < 4 :
            mask = cv2.inRange(hsv_img, self.lower_threshold[index], 
                self.upper_threshold[index]) 

            m = cv2.moments(mask)
            
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(hsv_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                self.colour_of_found_item = index
                print("Colour found: ", self.colours[self.colour_of_found_item]) 
            index += 1



    def __init__(self):

        self.min_distance = 100
        self.front_arc = np.empty(180)

        # SLAM file path
        self.map_path = Path.home().joinpath("catkin_ws/src/team46/maps/task5_map")

        self.node_name = "maze_navigator"
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        # Scan 
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")

        # Camera
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cvbridge_interface = CvBridge()

        self.rate = rospy.Rate(10) # hz

        # Initialize the Tb3Move class
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.start = True

        # Initialize the target colour
        self.colours = ["Yellow", "Red", "Green", "Blue"]
        self.lower_threshold = [(26,193,100), (0, 185, 100),  (57,150,100), (115, 220, 100)]
        self.upper_threshold = [(40,255,255), (10, 255, 255), (63, 255, 255), (130, 255, 255)]
        self.colour_of_found_item = -1

        # Initialize the m00_min
        self.m00 = 0
        self.m00_min = 30000000


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
        #node = roslaunch.core.Node(package="map_server",
         #                   node_type="map_saver",
         #                   args=f"-f {self.map_path}")

        #process = self.launch.launch(node)
        self.ctrl_c = True
    

    def main_loop(self):

        while not self.ctrl_c:

            front_sensor = np.amin(self.front_arc[75:105])
            right_sensor = np.amin(self.front_arc[130:145])
            left_sensor = np.amin(self.front_arc[35:75])  

            # Starting
            # print("Starting...", self.start)
            if self.start:
                time.sleep(1)
                right_sensor = np.amin(self.front_arc[130:140])

                # While there is no wall in the right direction
                while right_sensor > 0.35:
                    right_sensor = np.amin(self.front_arc[130:140])
                    self.robot_controller.set_move_cmd(0.05,1)
                    self.robot_controller.publish()
                self.start = False
            
            # If there is a blob is detected 
            if self.colour_of_found_item > -1 and front_sensor < 0.5:
                print("Colour found: ", self.colours[self.colour_of_found_item])
                self.robot_controller.set_move_cmd( -0.023, -0.8)
                self.robot_controller.publish()
                time.sleep(3.5)
                self.colour_of_found_item = -1

            # If there is a wall in front of the robot    
            elif (front_sensor < 0.35):
                self.robot_controller.set_move_cmd(0,1.0)

            # If there is a wall in the right and front direction
            elif ( front_sensor < 0.4 and right_sensor < 0.4):
                print("Wall found in front and right direction")
                self.robot_controller.set_move_cmd(0, 0.7)

            # If there is a wall in the left and front direction
            elif ( front_sensor < 0.4 and left_sensor < 0.5):
                print("Wall found in front and left direction")
                self.robot_controller.set_move_cmd(0, -0.7)
            else:  

                # If there is no wall in the right direction
                if (right_sensor < 0.3):
                    self.robot_controller.set_move_cmd(0.15, 0.5) 

                # Find the wall in the left direction
                elif (left_sensor < 0.3):
                    self.robot_controller.set_move_cmd(0.15, -0.7)

                # If there is not wall in the right direction
                elif (right_sensor > 0.4):
                    self.robot_controller.set_move_cmd(0.2, -0.7)
                
                # Else, move forward
                else:
                    self.robot_controller.set_move_cmd(0.25,0.0)

            # if (right_sensor < 0.4):
            #     self.wall_found = True
            #     self.x = self.robot_odom.posx
            #     self.y = self.robot_odom.posy

            # if math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2) < 0.05 and math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2) > 0:

            
            self.robot_controller.publish()

            node = roslaunch.core.Node(package="map_server",
                            node_type="map_saver",
                            args=f"-f {self.map_path}")

            process = self.launch.launch(node)

            



            self.rate.sleep()
        
        
    
if __name__ == '__main__':
    maze_instance = Maze()
    try:
        maze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass