#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
# Import path module
from pathlib import Path
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

# Import module for accepting arguments from launch file 
import argparse
# Import String module
from std_msgs.msg import String

# Import path module





class Explore:

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

    def save_image(self, img, img_name):

        full_image_path = self.base_image_path.joinpath(f"{img_name}.jpg")
        cv2.imwrite(str(full_image_path), img)

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

        if not self.initial_photo:
            img_name = "the_beacon"
            self.save_image(cv_img, img_name)
            
            
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
                self.waiting_for_image = True
                
                if not self.target_photo_taken and self.cy > 600 and self.cy < 1100:
                    # self.waiting_for_image = False
                    print(f"Obtained an image of height {height}px and width {width}px.")
                    img_name = "the_beacon"
                    self.save_image(cv_img, img_name)

                    if self.target_colour == self.colours[index]:
                        self.target_photo_taken = True

            index += 1


    def __init__(self):

        self.min_distance = 100
        self.front_arc = np.empty(180)

        self.node_name = "area_navigator"

        # Scan 
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.init_node(self.node_name, anonymous=True)
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")

        # Camera
        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw",
            Image, self.camera_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cvbridge_interface = CvBridge()

        self.rate = rospy.Rate(30) # hz

        # Initialize the Tb3Move class
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.start = True
        self.initial_photo = False

        # Initialize the target colour
        self.colours = ["red", "yellow", "green", "blue"]
        self.lower_threshold = [ (0, 185, 100), (26,193,100), (57,150,100), (115, 220, 100)]
        self.upper_threshold = [ (10, 255, 255), (40,255,255), (63, 255, 255), (130, 255, 255)]
        self.colour_of_found_item = -1

        # Initialize the m00_min
        self.m00 = 0
        self.m00_min = 2000000
        self.min_wall_dist = 0.37

        # Saving pictures
        self.waiting_for_image = False
        self.target_photo_taken = False
        self.base_image_path = Path("/home/student/catkin_ws/src/team46/snaps")
        # self.base_image_path.mkdir(parents=True, exist_ok=True)

        # Map 
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument("-target_colour", metavar="COL", type=String,
            default="red", 
            help="The name of a colour (for example)")
        
        # For retriving the argument
        self.args = cli.parse_args(rospy.myargv()[1:])
        self.target_colour = self.args.target_colour.data

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        print("stopping the robot")
        self.robot_controller.set_move_cmd(0, 0)
        self.robot_controller.publish()
        
        self.ctrl_c = True
    

    def main_loop(self):

        while not self.ctrl_c:

            front_sensor = self.front_arc[80:100]
            front_sensor[ front_sensor == 0] = 20
            front_sensor = np.amin(front_sensor)
 
            
            front_right_sensor = self.front_arc[120:145]
            front_right_sensor[ front_right_sensor == 0] = 20
            front_right_sensor = np.amin(front_right_sensor) 
            

            front_left_sensor = self.front_arc[50:85]
            front_left_sensor[front_left_sensor == 0] = 20
            front_left_sensor = np.amin(front_left_sensor)
                
            right_sensor = self.front_arc[170:180]
            right_sensor[right_sensor == 0] = 20
            right_sensor = np.amin(right_sensor)
            

            # Starting
            if self.start:
                time.sleep(1)
                right_sensor = np.amin(self.front_arc[160:170])
                # While there is no wall in the right direction
                while  right_sensor > self.min_wall_dist: #right_sensor > self.min_wall_dist + 1:
                    right_sensor = np.amin(self.front_arc[160:170])
                    self.robot_controller.set_move_cmd(0.05,1)
                    self.robot_controller.publish()
                self.start = False
            
            # If a blob is detected 
            elif front_sensor < self.min_wall_dist - 0.15:
                print("polla konta se tixo" , front_sensor)
                self.robot_controller.set_move_cmd(-0.08, 0)

            # A blob was detected
            elif self.colour_of_found_item > -1 and front_sensor < 0.7:
                print("vlepo blob")
                if self.cy < 600:
                    self.robot_controller.set_move_cmd(0, 0.4)

                elif self.cy > 1100:
                    self.robot_controller.set_move_cmd(0, -0.4)       

                else:
                    self.robot_controller.set_move_cmd(0, 0)
                    self.robot_controller.publish()
                    time.sleep(0.1)
                    self.robot_controller.set_move_cmd(-0.05, 1.8)
                    self.robot_controller.publish()
                    time.sleep(1)
                    
                self.colour_of_found_item = -1

            # If there is a wall in front of the robot    
            elif ( front_sensor < self.min_wall_dist):
                print("wall mprosta mou", front_sensor)
                self.robot_controller.set_move_cmd(0, 0)
                self.robot_controller.publish()
                self.robot_controller.set_move_cmd(0 ,1.2)

            else:  

                # If there is wall in the right direction
                if (front_right_sensor < self.min_wall_dist -0.04 ):
                    print("There is something on my right ", front_right_sensor)
                    self.robot_controller.set_move_cmd(0.2, 0.4) 

                # Find the wall in the left direction
                elif ( front_left_sensor < self.min_wall_dist ):
                    print("There is something on my left", front_left_sensor)
                    self.robot_controller.set_move_cmd(0.2, -0.6)

                # If there is not wall in the right direction
                elif (front_right_sensor > self.min_wall_dist + 0.5):
                    print("I need a wall on the right ", front_right_sensor )
                    self.robot_controller.set_move_cmd(0.15, -1.2)
            
            self.robot_controller.publish()   

            self.rate.sleep()
        
        
    
if __name__ == '__main__':
    explore_instance = Explore()
    try:
        explore_instance.main_loop()
    except rospy.ROSInterruptException:
        pass