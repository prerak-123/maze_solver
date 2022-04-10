#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

p_detected = False
password = 'XXXXX'
distances = {
    'left': 0,
    'right': 0,
    'front': 0
}

pose = {
    "x": 0,
    "y": 0,
    "z": 0,
    "time_sec": 0
}

def sensor_l(data):
    distances['left'] = data.range
    
def sensor_f(data):
    distances['front'] = data.range
    
def sensor_r(data):
    distances['right'] = data.range
    
def odom(data):
    pose["x"] = data.pose.pose.position.x
    pose["y"] = data.pose.pose.position.y
    pose["z"] = data.pose.pose.position.z

def time(data):
    pose["time_sec"] = data.clock.secs
    
def publish_velocity(l_x, l_y, l_z, a_x, a_y, a_z):
    rate = rospy.Rate(20)
    velocity_publisher = rospy.Publisher('maze_solver/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = l_x
    vel_msg.linear.y = l_y
    vel_msg.linear.z = l_z
    vel_msg.angular.x = a_x
    vel_msg.angular.y = a_y
    vel_msg.angular.z = a_z
    
    velocity_publisher.publish(vel_msg)
    
    rate.sleep()
    return
#def Arm_Control:


#When spawned, detect the nearest wall and start moving with that wall to the right
#def Nearest_Wall():

def listner():
    rospy.Subscriber('maze_solver/sensor_l', Range, sensor_l)
    rospy.Subscriber('maze_solver/sensor_f', Range, sensor_f)
    rospy.Subscriber('maze_solver/sensor_r', Range, sensor_r)
    rospy.Subscriber('/odom', Odometry, odom)
    rospy.Subscriber('/clock', Clock, time)

    
    
if __name__ == '__main__':
    try:
        rospy.init_node('master', anonymous=True)
        while(not rospy.is_shutdown()):
            listner()
            a_z = 0
            l_x = 0.5
            if(distances["front"] < 1.1): 
                l_x = 0
            
            publish_velocity(l_x, 0, 0, 0, 0, a_z)    

    except rospy.ROSInterruptException: pass

