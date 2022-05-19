#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import time
import numpy as np

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move, Tb3Odometry

#Import laserscan for wall following algorithm
from sensor_msgs.msg import LaserScan

class colour_search(object):

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
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        # Variables for retriving lightdar data
        self.min_distance = 100
        self.front_arc = np.empty(180)

        # For taking images
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cvbridge_interface = CvBridge()
        
        # For using Tb3 topic
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_controller.stop()

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        # Setting he colors and thresholds for detecting the beacons
        self.colours = ["Turqoise", "Yellow", "Red", "Green", "Purple", "Blue"]
        self.lower_threshold = [(78, 160, 100),   (26,193,100), (0, 185, 100),  (57,150,100), (148, 250, 100), (115, 220, 100)]
        self.upper_threshold = [(95, 255, 255),  (40,255,255), (10, 255, 255), (63, 255, 255), (153, 275, 255), (130, 255, 255)]
        self.m00 = 0
        self.m00_min = 600000
        
        # For making sure we run the code only at the start
        self.target_found = False
        self.target_colour = -1
        self.seeing_beacon = False
        self.ready_to_beaconing = False
        self.beaconing_message_published = False

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):  
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape

        # Cropping the image to only analyse a part of it
        crop_width = width - 400
        crop_height = 100
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height + 500, crop_y0:crop_y0+crop_width]

        # Filering the cropped images to hsv colors
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        
        # If the target is found and the target colour is not set.
        if self.target_found and self.target_colour == -1: 
            # Cycle all the colours to find the target one
            index = 0
            while self.target_colour == -1 and index < 6 :
                mask = cv2.inRange(hsv_img, self.lower_threshold[index], self.upper_threshold[index]) 
                m = cv2.moments(mask)
                self.m00 = m["m00"]
                self.cy = m["m10"] / (m["m00"] + 1e-5)

                # If the color is found
                if self.m00 > self.m00_min:
                    cv2.circle(hsv_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                    self.target_colour = index
                index += 1

        # Else if the robot is ready to beaconing
        elif self.ready_to_beaconing:
            colour_index = self.target_colour
            mask = cv2.inRange(hsv_img, self.lower_threshold[colour_index], self.upper_threshold[colour_index]) 

            m = cv2.moments(mask)
                
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)
            self.cz = m['m01']/(m['m00']+1e-5) 

            if self.m00 > self.m00_min:
                    cv2.circle(hsv_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                    self.seeing_beacon = True
            else:
                self.seeing_beacon = False


    def main(self):
        while not self.ctrl_c:
            # Calculating the distance of each sensor
            min_right_side = np.amin(self.front_arc[115:150])
            min_left_side = np.amin(self.front_arc[50:85]) 
            min_front_side = np.amin(self.front_arc[75:105])

            if not self.target_found :
                # Wait 1 second to initialise all the sensors
                time.sleep(1)

                # Turn 90 degrees right
                self.robot_controller.set_move_cmd(0, -1)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.stop()
                time.sleep(0.5)

                # Find target color (see camera_callback())
                self.target_found = True

                # Turn 90 degrees left
                self.robot_controller.set_move_cmd(0, 1)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.stop()
                time.sleep(0.5)
                print("SEARCH INITIATED: The target beacon colour is", self.colours[self.target_colour],"\b.") 
                
                #Go a bit forward to get out of the box
                self.robot_controller.set_move_cmd(0.2, 0)
                self.robot_controller.publish()
                time.sleep(2.5)
                
                # If there is a right wall at the start turn right 
                # and move forward
                start_right_sensor = np.amin(self.front_arc[175:180]) 
                if start_right_sensor > 0.65:
                    self.robot_controller.set_move_cmd(0, -1.82)
                    self.robot_controller.publish()
                    time.sleep(1)
                    self.robot_controller.set_move_cmd(0.26, 0)
                    self.robot_controller.publish()
                    time.sleep(2.5)

                self.ready_to_beaconing = True            
            # If the beacond is in the field of view 
            if self.seeing_beacon:
                # if the beacon and a wall is in front of the robot
                if self.cz > 265 and min_front_side < 0.5:
                    print("BEACONING COMPLETE: The robot has now stopped.")
                    self.ctrl_c = True      

                # Else if the robot is to close to a wall  
                elif (min_front_side < 0.35):
                    self.robot_controller.set_move_cmd(-0.15, -0.8)
                
                # Else if there is a wall at the right of the robot
                elif(min_right_side < 0.45):
                    self.robot_controller.set_move_cmd(0.14, 0.9)
                
                # Else if there is a wall at the left of the robot    
                elif(min_left_side < 0.4):
                    self.robot_controller.set_move_cmd(0.08, -0.7)

                # Else if the beacon is in the midle of the robots view
                elif self.cy >= 650 and self.cy <= 800:
                    if not self.beaconing_message_published:
                        self.beaconing_message_published = True
                        print("TARGET DETECTED: Beaconing initiated.")
                    self.robot_controller.set_move_cmd(0.26, 0)
                    self.robot_controller.publish()
                # Else if the beacon is too far left
                elif self.cy < 650:
                    if not self.beaconing_message_published:
                        self.robot_controller.stop()
                        self.beaconing_message_published = True
                        print("TARGET DETECTED: Beaconing initiated.")    
                    self.robot_controller.set_move_cmd(0.26, 0.4)
                    self.robot_controller.publish()
                # Else if the beacon is too far right
                elif self.cy > 800:
                    if not self.beaconing_message_published:
                        self.beaconing_message_published = True
                        print("TARGET DETECTED: Beaconing initiated.")
                    self.robot_controller.set_move_cmd(0.26, -0.4)
            # Else if ther is a wall in front 
            elif (min_front_side < 0.75) and min_left_side > 0.4:
                self.robot_controller.set_move_cmd(0.2, 1.5)  

            # Else if there is a wall in the right
            elif(min_right_side > 0.5 and min_right_side < 0.6):
                self.robot_controller.set_move_cmd(0.26, 0.1)

            # Else if there is a wall in the right side
            elif(min_right_side < 0.5):
                self.robot_controller.set_move_cmd(0.26, 1.1)

            elif(min_right_side > 0.6):
                self.robot_controller.set_move_cmd(0.26, -0.8)

            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
