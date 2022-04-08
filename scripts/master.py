#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

speed = 0.2
omega = 0.2
p_detected = False
password = 'XXXXX'

#def Arm_Control:


#When spawned, detect the nearest wall and start moving with that wall to the right
#def Nearest_Wall():


if __name__ == '__main__':
    try:
        rospy.init_node('move_robot', anonymous=True)
        while not rospy.is_shutdown():
            velocity_publisher = rospy.Publisher('maze_solver/cmd_vel', Twist, queue_size=10)
            vel_msg = Twist()
            vel_msg.linear.x = speed
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = omega
            velocity_publisher.publish(vel_msg)

            sensor_l_subs = rospy.Subscriber('maze_solver/sensor_l', float)
            sensor_f_subs = rospy.Subscriber('maze_solver/sensor_f', float)
            sensor_r_subs = rospy.Subscriber('maze_solver/sensor_r', float)
    
    except rospy.ROSInterruptException: pass

