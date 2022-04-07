#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    try:
        rospy.init_node('move_robot', anonymous=True)
        while not rospy.is_shutdown():
            speed = 0.5
            velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            vel_msg = Twist()
    
            vel_msg.linear.x = speed
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
    
    except rospy.ROSInterruptException: pass

