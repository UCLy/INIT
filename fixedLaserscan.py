#! /usr/bin/env python

import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#permet au turtlebot de détecter et s
def callback(msg):
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()

	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	if 0.01 > msg.ranges[0] or msg.ranges[0] > msg.range_max:
		print("I see a wall at", msg.ranges[0], "m !")
	else:
		print(msg.range_min, "<", msg.ranges[0], "<", msg.range_max)
		vel_msg.linear.x = 4
	pub.publish(vel_msg)

"""
	print('===============================================')
	print('s1 [0]')
	print msg.ranges[0]

	print('s2 [90]')
	print msg.ranges[90]

	print('s3 [180]')
	print msg.ranges[180]

	print('s4 [270]')
	print msg.ranges[270]

	print ('s5 [359]')
	print msg.ranges[359]
"""

rospy.init_node('laser_data')
Timer = 0
while Timer < 6:
	sub = rospy.Subscriber('scan', LaserScan, callback)
	Timer = Timer +1
	time.sleep(1)
	print(Timer)

rospy.spin()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
vel_msg.linear.x = 0
pub.publish(vel_msg)
sub.unregister()
