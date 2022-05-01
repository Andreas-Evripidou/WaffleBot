#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import time

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move, Tb3Odometry

class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(1.0, self.turn_vel_fast)
        self.robot_controller.publish()

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

        crop_width = width - 400
        crop_height = 400
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        if self.target_found and self.target_colour == -1:
            # Colours Thresholds       Turquoise         Red       Yellow           Green           Purple             Blue
            lower_threshold = [(86, 150, 100),(0, 190, 100), (26,98,100),   (57,150,100), (148, 250, 100), (118, 215, 100)]
            upper_threshold = [(93, 250, 255),  (10, 255, 255),(34,251,255), (63, 255, 255), (153, 275, 255), (123, 253, 255)]
            
            index = 0
            while self.target_found and self.target_colour == -1 :
                mask = cv2.inRange(hsv_img, lower_threshold[index], upper_threshold[index]) 

                m = cv2.moments(mask)
                
                self.m00 = m["m00"]
                self.cy = m["m10"] / (m["m00"] + 1e-5)

                if self.m00 > self.m00_min:
                    cv2.circle(hsv_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                    self.target_colour = index
                    print("empika dames")
                index += 1


    #     #Blue pillar bounds
    #     lower = (115, 224, 100)
    #     upper = (130, 255, 255)

    #     #Cyan pillar bounds
    #     #lower = (85, 158, 100)
    #     #upper = (92, 255, 255)

    #     #Green pillar bounds
    #     #lower = (50, 170, 100)
    #     #upper = (65, 255, 255)

    #     #Red pillar bounds
    #     #lower = (-2, 195, 100)
    #     #upper = (6, 255, 255)

    #     mask = cv2.inRange(hsv_img, lower, upper)
    #     res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

    #     m = cv2.moments(mask)
    #     self.m00 = m['m00']
    #     self.cy = m['m10'] / (m['m00'] + 1e-5)

    #     if self.m00 > self.m00_min:
    #         cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
    #     cv2.imshow('cropped image', crop_img)
    #     cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            
            if not self.target_found :
                print("I am initializing")
                time.sleep(1)

                # Turn right
                print("I am turning right")
                self.robot_controller.set_move_cmd(0, -1.57)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.set_move_cmd(0, 0)
                self.robot_controller.publish()
                
                # Find target color
                print("I am finding the target colour")
                time.sleep(0.5)
                self.target_found = True

                # Turn left
                print("I am turning left")
                self.robot_controller.set_move_cmd(0, 1.57)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.set_move_cmd(0, 0)
                self.robot_controller.publish()  

            # print(self.robot_odom.posx)
            print("Colour found: ", self.target_colour)
            self.rate.sleep()
            
if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass

                # print("gurizo deksia")
                # while abs(self.robot_odom.yaw - self.theta_z0) < 90:
                #     self.robot_controller.set_move_cmd(0, self.turn_vel_fast)
                #     self.robot_controller.publish()
                # self.robot_controller.set_move_cmd(0,0)
                # self.robot_controller.publish()

                # print("gurizo aristera")
                # while self.robot_odom.yaw > self.y0:
                #     self.robot_controller.set_move_cmd(0,1.0)
                #     self.robot_controller.publish()
                # self.robot_controller.set_move_cmd(0,0)
                # self.robot_controller.publish()
