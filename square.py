#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CompressedImage

from time import sleep
import math


#permet au turtlebot d'effectuer une trajectoire en carré, avec feedback du turtlebot.

def imu_callback(msg):
    rospy.loginfo("From IMU, linear acceleration = %s angular acceleration = %s",
                  str(msg.linear_acceleration.x),
                  str(msg.angular_velocity.x))
                  # str(msg.linear_acceleration.y), "]")

def raspicam_callback(msg):
    rospy.loginfo("From Raspicam, format = %s",
                  str(msg.format))
                  # str(msg.linear_acceleration.y), "]")

def listener():
    # rospy.init_node('mydemo1', anonymous=True)
    rospy.Subscriber("/imu", Imu, imu_callback)
#    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, raspicam_callback)

def segment(velocity_publisher, length=1, speed=0.1):
    vel_msg = Twist()
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    vel_msg.linear.x = speed

    print("On avance")
    vel_msg.linear.x = speed
    velocity_publisher.publish(vel_msg)

    duration = length / speed
    
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < duration:
        velocity_publisher.publish(vel_msg)
        sleep(0.1)

    print("Et on s'arrete")
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)


def rotation(velocity_publisher, deg=90, speed=1):
    vel_msg = Twist()
    
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    print("On tourne")
    vel_msg.angular.z = speed
    velocity_publisher.publish(vel_msg)

    turn_duration = math.pi/2
    
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < turn_duration:
        velocity_publisher.publish(vel_msg)
        sleep(0.1)

    print("Et on s'arrete")
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def move():
    # Starts a new node
    rospy.init_node('carre_augustin', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener()
    
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    #speed = input("Input your speed:")
    #distance = input("Type your distance:")
    #isForward = input("Foward?: ")#True or False

    speed = 0.5
    distance = 0.01
    isForward = True
    
    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(distance)
    else:
        vel_msg.linear.x = -abs(distance)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    #while not rospy.is_shutdown():

    #Setting the current time for distance calculus
    #t0 = rospy.Time.now().to_sec()
    #current_distance = 0
    
    #Loop to move the turtle in an specified distance
    #while(current_distance < distance):
        #Publish the velocity
        #velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        #t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        #current_distance= speed*(t1-t0)
        
    #After the loop, stops the robot
    #vel_msg.linear.x = 0
    #Force the robot to stop

    for i in range(4):
        segment(velocity_publisher)
        rotation(velocity_publisher)
    
    

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
