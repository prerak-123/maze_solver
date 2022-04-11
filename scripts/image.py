#!/usr/bin/env python

import rospy
from sensor_msgs.msg import  Image
import cv2
import numpy as np

def show(rgb_data):
    img_rgb_left = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)[:,:,0:3]
    
    print(img_rgb_left)
    
    cv2.imshow("Img", img_rgb_left)
    
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    
    rospy.init_node('Image', anonymous= True)
    rospy.sleep(1)
    get_rgb_image_left = rospy.Subscriber('/maze_solver/camera/image_topic', Image, show)
    rospy.spin()