#!/usr/bin/env python3

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

node_name = "object_detection_node"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(5)

base_image_path = Path("/home/student/myrosdata/week6_images")
base_image_path.mkdir(parents=True, exist_ok=True)

cvbridge_interface = CvBridge()

waiting_for_image = True

def show_and_save_image(img, img_name):
    full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

    cv2.imshow(img_name, img)
    cv2.waitKey(0)

    cv2.imwrite(str(full_image_path), img)
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes")

def camera_cb(img_data):
    global waiting_for_image  
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True:
        height, width, channels = cv_img.shape

        print(f"Obtained an image of height {height}px and width {width}px.")

        crop_width = width - 400
        crop_height = 400
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        blue_lower_threshold = (118, 215, 100)
        blue_upper_threshold = (123, 253, 255)
        blue_mask = cv2.inRange(hsv_img, blue_lower_threshold, blue_upper_threshold)

        if blue_mask.any():
            print ("eivramen to")

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)

while waiting_for_image:
    rate.sleep()

cv2.destroyAllWindows()