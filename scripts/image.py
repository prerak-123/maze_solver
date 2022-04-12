#!/usr/bin/env python

import rospy
from sensor_msgs.msg import  Image
from std_msgs.msg import Bool
import cv2
import numpy as np

def publish(rgb_data):
    img_rgb_left = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)[:,:,0:3]
    
    img = cv2.cvtColor(img_rgb_left, cv2.COLOR_BGR2HSV)[200:500, 500:1000, :]
    
    mask = cv2.inRange(img, lowerb=(88, 250, 250), upperb=(92, 255, 255))
    
    count_0 = 0
    count_1 = 0
    for i in mask:
        for j in i:
            if j == 0:
                count_0 += 1
            else:
                count_1 += 1
    
    print(count_1)
    if(count_1 > 600):
        image_publisher = rospy.Publisher('maze_solver/yellow_wall', Bool, queue_size=10)
        
        msg = Bool()
        msg.data = True

        image_publisher.publish(msg)
    else:
        image_publisher = rospy.Publisher('maze_solver/yellow_wall', Bool, queue_size=10)
        
        msg = Bool()
        msg.data = False

        image_publisher.publish(msg)

if __name__ == "__main__":
    
    rospy.init_node('Image', anonymous= True)
    rospy.sleep(1)
    get_rgb_image_left = rospy.Subscriber('/maze_solver/camera/image_topic', Image, publish)
    rospy.spin()