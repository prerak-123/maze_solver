#!/usr/bin/env python

from math import copysign, pi
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

p_detected = False
password = 'XXXXX'
difference_front_right = 0.965609133244 - 0.802687704563 #Front measures 0.96 => After rotation Right measures 0.80

distance_right_wall = 0.6

speed = 0.2
err = speed*speed / 2


def old_range_to_new(angle):
    if angle >= 0:
        return angle
    else:
        return angle + 2 * pi

distances = {
    'left': 0,
    'right': 0,
    'front': 0
}

pose = {
    "x": 0,
    "y": 0,
    "z": 0,
    "angle": 0,
    "new_angle": 0,
    "time_sec": 0,
    "yellow_wall": False
}



def next_angle_ACW(current_angle):
    if(pi/2 < current_angle and current_angle <= pi):
        return current_angle - 3*pi/2
    
    else:
        return current_angle + pi/2 

def next_angle_CW(current_angle):
    if(-1*pi <= current_angle and current_angle <= -0.5*pi):
        return current_angle + 3*pi/2
    
    else:
        return current_angle - pi/2
    
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
    
    orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    
    pose["angle"] = yaw
    pose["new_angle"] = old_range_to_new(yaw)


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
   
def turn_left():
    a_z = 0.1
    final_angle = next_angle_ACW(pose["angle"])
    publish_velocity(0,0,0,0,0,a_z)
    while(not(abs(pose["angle"] - final_angle) <= 0.01)):
        publish_velocity(0,0,0,0,0,a_z)
    
    publish_velocity(0,0,0,0,0,0)
    return 

def turn_right():
    a_z = -0.1
    final_angle = next_angle_CW(pose["angle"])
    publish_velocity(0,0,0,0,0,a_z)
    while(not(abs(pose["angle"] - final_angle) <= 0.01)):
        publish_velocity(0,0,0,0,0,a_z)
    
    publish_velocity(0,0,0,0,0,0)
    return   

def right_arc(radius):
    v = 0.04
    w = -1 * v / radius
    publish_velocity(v,0,0,0,0,w)

    final_angle = next_angle_CW(pose["angle"])
    while(not(abs(pose["angle"] - final_angle) <= 0.03)):
        publish_velocity(v,0,0,0,0,w)
    
    distance = 0
    
    t_0 = rospy.Time.now().to_sec()
    
    publish_velocity(v, 0, 0, 0 ,0 ,0)
    
    while(distance <= 0.5):
        t = rospy.Time.now().to_sec()
        distance = (t-t_0)*v
        publish_velocity(v, 0, 0, 0 ,0 ,0)

    publish_velocity(0, 0, 0, 0 ,0 ,0)
    return

def Reach_Wall():
    w = -1 * copysign(0.04, pose["angle"])
    while not abs(pose["angle"]) < 0.05 :
        publish_velocity(0, 0, 0, 0, 0, w)
    if(distances["left"] < distances["right"]):
        turn_left()
    else:
        turn_right()
    v = copysign(0.04, distances["front"] - err - difference_front_right - distance_right_wall)
    while not abs(distances["front"] - err - difference_front_right - distance_right_wall) < 0.01:
        publish_velocity(v, 0, 0, 0, 0, 0)
    publish_velocity(0, 0, 0, 0, 0, 0)
    turn_left()
    
def Arm_Control():
    return


def listner():
    rospy.Subscriber('maze_solver/sensor_l', Range, sensor_l)
    rospy.Subscriber('maze_solver/sensor_f', Range, sensor_f)
    rospy.Subscriber('maze_solver/sensor_r', Range, sensor_r)
    rospy.Subscriber('/odom', Odometry, odom)
    rospy.Subscriber('/clock', Clock, time)
    rospy.Subscriber('/maze_solver/yellow_wall', Bool, wall_type )
    
def wall_type(data):
    pose["yellow_wall"] = data.data

    
    
if __name__ == '__main__':
    try:
        rospy.init_node('master', anonymous=True)
        listner()
        prev_wall_distance = distances["right"]
        rospy.sleep(1)
        listner
        Reach_Wall()
        while(not rospy.is_shutdown()):
            rospy.sleep(1)
            listner()
            
            if(distances["right"] >= distance_right_wall + 1):
                publish_velocity(0,0,0,0,0,0)
                right_arc(distance_right_wall - err)
                continue
            
            prev_wall_distance = distances["right"]
                   
            if(distances["front"] - err - difference_front_right <= distance_right_wall):
                if(pose["yellow_wall"]):
                    print("Solved!")
                    break
                publish_velocity(0,0,0,0,0,0)
                turn_left()
                continue

            publish_velocity(speed, 0, 0, 0, 0, -0.05*(distances["right"] - distance_right_wall))
            
    except rospy.ROSInterruptException: pass

