#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    vel_msg = Twist()
	vel_msg.linear.x = 5
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
    if 0.01 < msg.ranges[0] < msg.range_max:
        print("I see a hand at", msg.ranges[0], "m !")
		vel_msg.linear.x = 0
	    pub.publish(vel_msg)
		sub.unregister()
	else:
		print(msg.range_min, "<", msg.ranges[0],"<",msg.range_max)
    pub.publish(vel_msg)

rospy.init_node('laser_data')
sub = rospy.Subscriber('scan', LaserScan, callback)

rospy.spin()
