#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# def spawn():
#     rospy.wait_for_service('circle')
#     tout_droit = rospy.ServiceProxy('tout_droit', ToutDroit)

#permet au turtlebot d'effectuer une trajectoire en cercle avec un timer.

def rosace():
	timer =0
	rospy.init_node('circle', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	vel_msg = Twist()

	vel_msg.linear.x = 2
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = -1.5
	rate = rospy.Rate(10)

	while not rospy.is_shutdown() and timer < 20 :
		velocity_publisher.publish(vel_msg)
		rate.sleep()
		time.sleep(1)
		timer += 1
	vel_msg.linear.x = 0
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)

rosace()




