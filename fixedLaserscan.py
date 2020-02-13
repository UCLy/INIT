#! /usr/bin/env python

import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def stop():
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()

	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	pub.publish(vel_msg)
	sub.unregister()


def callback(msg):
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()

	vel_msg.linear.x = 4
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
    if 0.01 < msg.ranges[0] < msg.range_max:
		print("I see a wall at", msg.ranges[0], "m !")
		stop()
		return()
	else:
		print(msg.range_min, "<", msg.ranges[0], "<", msg.range_max)
	pub.publish(vel_msg)
	return()

rospy.init_node('laser_data')
for timer in range(21):
	sub = rospy.Subscriber('scan', LaserScan, callback)
	time.sleep(0.33)
	print(Timer)
	pass

stop()
rospy.spin()
