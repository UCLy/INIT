#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CompressedImage

#permet au turtlebot d'avancer ou de reculer avec un input de l'utilisateur pour définir la distance.
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
    
def move():
    # Starts a new node
    rospy.init_node('square', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
   #listener()
    
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed:")
    distance = input("Type your distance:")
    isForward = input("Foward?: ")#True or False

    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

"""
rospy.init_node('square', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
for a in range(4):
    current_distance = 0
    distance = 50
    vel_msg.linear.x = 2
    t0 = rospy.Time.now().to_sec()

    while(current_distance < distance):
            velocity_publisher.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            current_distance= speed*(t1-t0)
            
    current_distance = 0
    vel_msg.linear.y = 2
    vel_msg.linear.x = 0
    t0 = rospy.Time.now().to_sec()

    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)
vel_msg.linear.y = 0
velocity_publisher.publish(vel_msg) """




if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
