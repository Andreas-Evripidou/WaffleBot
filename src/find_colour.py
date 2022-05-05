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
from tf.transformations import euler_from_quaternion

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

        self.min_distance = 100
        self.front_arc = np.empty(180)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cvbridge_interface = CvBridge()
        

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = self.robot_odom.yaw
        
        # For making sure we run the code only at the start
        self.target_found = False
        self.target_colour = -1
        self.seeing_beacon = False
        self.ready_to_beaconing = False

        self.m00 = 0
        self.m00_min = 10000

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

        # print(f"Obtained an image of height {height}px and width {width}px.")

<<<<<<< HEAD
        crop_width = width - 400
        crop_height = 100
=======
        crop_width = width - 700
        crop_height = 50
>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height + 500, crop_y0:crop_y0+crop_width]

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        # Colours Thresholds       Turquoise         Red       Yellow           Green           Purple             Blue
<<<<<<< HEAD
        lower_threshold = [(78, 160, 100),  (0, 190, 100), (26,98,100),   (57,150,100), (148, 250, 100), (115, 220, 100)]
        upper_threshold = [(95, 255, 255),  (10, 255, 255),(34,251,255), (63, 255, 255), (153, 275, 255), (130, 255, 255)]
=======
        lower_threshold = [(86, 150, 100),  (0, 190, 100), (26,98,100),   (57,150,100), (148, 250, 100), (115, 220, 100)]
        upper_threshold = [(93, 250, 255),  (10, 255, 255),(34,251,255), (63, 255, 255), (153, 275, 255), (130, 255, 255)]
>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a

        if self.target_found and self.target_colour == -1:
            
            index = 0
            while self.target_found and index < 6 :
                mask = cv2.inRange(hsv_img, lower_threshold[index], upper_threshold[index]) 

                m = cv2.moments(mask)
                
                self.m00 = m["m00"]
                self.cy = m["m10"] / (m["m00"] + 1e-5)

                if self.m00 > self.m00_min:
                    cv2.circle(hsv_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                    self.target_colour = index
                index += 1
            
        elif self.ready_to_beaconing:
            colour_index = self.target_colour
            mask = cv2.inRange(hsv_img, lower_threshold[colour_index], upper_threshold[colour_index]) 

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
            
            if not self.target_found :
                print("I am initializing")
<<<<<<< HEAD
                time.sleep(0.8)
=======
                time.sleep(0.5)
>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a

                # Turn right
                print("I am turning right")
                self.robot_controller.set_move_cmd(0, -1.3)
                self.robot_controller.publish()
                time.sleep(1.2)
                self.robot_controller.set_move_cmd(0, 0)
                self.robot_controller.publish()
                
                time.sleep(0.3)
                # Find target color
                print("I am finding the target colour")
                self.target_found = True

                # Turn left
                print("I am turning left")
                self.robot_controller.set_move_cmd(0, 1.3)
                self.robot_controller.publish()
                time.sleep(1.2)
                self.robot_controller.set_move_cmd(0, 0)
                self.robot_controller.publish() 
                time.sleep(0.5)
                print("Colour found: ", self.target_colour) 
                
                self.robot_controller.set_move_cmd(0.26, 0)
                self.robot_controller.publish()
                time.sleep(2)

<<<<<<< HEAD
                self.robot_controller.set_move_cmd(0, -1.82)
                self.robot_controller.publish()
                time.sleep(1)
                self.robot_controller.set_move_cmd(0.26, 0)
                self.robot_controller.publish()
                time.sleep(0.5)
=======
                self.robot_controller.set_move_cmd(0, -1.57)
                self.robot_controller.publish()
                time.sleep(1.2)
                self.robot_controller.set_move_cmd(0.26, 0)
                self.robot_controller.publish()

>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a

                self.ready_to_beaconing = True
                print ("kamno beaconing")

            

            min_right_side = np.amin(self.front_arc[115:150])
            min_left_side = np.amin(self.front_arc[50:85]) 
            min_front_side = np.amin(self.front_arc[75:105])

            if self.seeing_beacon:
<<<<<<< HEAD
                if self.cz > 265 and self.cy > 600 and self.cy < 800 and min_front_side < 0.5:
=======
                if min_front_side < 0.40 and self.seeing_beacon:
>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a
                    self.robot_controller.set_move_cmd(0, 0)
                    print("We fucking did it")
                    self.ctrl_c = True        
                else:
<<<<<<< HEAD
                    if(min_right_side < 0.45):
                        print ("deksi tixos")
                        self.robot_controller.set_move_cmd(0.14, 0.9)
                        
                    elif(min_left_side < 0.4):
                        print ("arister tixos")
                        self.robot_controller.set_move_cmd(0.08, -0.7)
                    else:
                        if self.cy >= 600 and self.cy <= 800:
                            self.robot_controller.set_move_cmd(0.26, 0)
                            self.robot_controller.publish()

                        elif self.cy < 600:
                            self.robot_controller.set_move_cmd(0.26, 0.4)
                            self.robot_controller.publish()
                        elif self.cy > 800:
                            self.robot_controller.set_move_cmd(0.26, -0.4)
                
                    
=======
                    print ("vlepo sto tzai pao")
                    #vriski to kentro damesa
                    if self.cy >= 300 and self.cy <= 400:
                        self.robot_controller.set_move_cmd(0.26, 0)
                        self.robot_controller.publish()
                    if self.cy < 300:
                        self.robot_controller.set_move_cmd(0.26, 0.30)
                        self.robot_controller.publish()
                    elif self.cy > 400:
                        self.robot_controller.set_move_cmd(0.26, -0.30)
                        self.robot_controller.publish()
>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a


                    #self.robot_controller.set_move_cmd(0.26, 0)
                    #self.robot_controller.publish()

<<<<<<< HEAD
            elif (min_front_side < 0.7):
                self.robot_controller.set_move_cmd(0.3, 1.6)

            else:        
                if(min_right_side > 0.5 and min_right_side < 0.6):
                    self.robot_controller.set_move_cmd(0.26, 0.1)
                elif(min_right_side < 0.5):
                    self.robot_controller.set_move_cmd(0.26, 1.2)
                elif(min_right_side > 0.6):
                    self.robot_controller.set_move_cmd(0.26, -0.8)
            self.robot_controller.publish()
=======
            elif (min_front_side < 0.8):
                self.robot_controller.set_move_cmd(0, 0)
                self.robot_controller.publish()
                self.robot_controller.set_move_cmd(0.04, 1.0)
                self.robot_controller.publish()

            else:        
                if(min_right_side > 0.5 and min_right_side < 0.6):
                    self.robot_controller.set_move_cmd(0.26, 0)
                    self.robot_controller.publish()
                elif(min_right_side < 0.5):
                    self.robot_controller.set_move_cmd(0.26, 0.8)
                    self.robot_controller.publish()
                elif(min_right_side > 0.6):
                    self.robot_controller.set_move_cmd(0.26, -0.8)
                    self.robot_controller.publish()
>>>>>>> f3c5eb86a7dcfd9a4597a570d40d7f0df68e727a






            self.rate.sleep()
            
if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
