# !/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TurtleBot:

    def __init__(self):
        # Message to inform user
        rospy.loginfo("Press CTRL+c to stop TurtleBot")

        # Keys CNTL + c will stop script
        rospy.on_shutdown(self.shutdown)

        # ControlTurtleBot is the name of the node sent to the #master
        rospy.init_node('TurtleBot', anonymous=False)

        # Initialisation du noeud qui publie
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.start_position = None
        self.current_position = None

    def enact(self, linear_speed=0.0, angular_speed=0.0, duration=1.0):
        self.laser_subscriber = rospy.Subscriber('scan', LaserScan, self.getTurtleBotPos)
        self.current_position = rospy.Subscriber('odom', Odometry, None)

        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0

        # Setting the current time for duration calculus
        t0 = float(rospy.Time.now().to_sec())

        # Loop to move the turtle during a specific duration
        while float(rospy.Time.now().to_sec()) - t0 < duration:
            # Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        # Publish the null velocity to force the robot to stop
        self.velocity_publisher.publish(vel_msg)

    def getTurtleBotPos(self, data):
        print(data)
        print("getTurtleBotPos : " + str(data.ranges[0]))

    def shutdown(self):
        # You can stop turtlebot by publishing an empty Twist
        # message
        rospy.loginfo("Stopping TurtleBot")

        self.velocity_publisher.publish(Twist())
        # Give TurtleBot time to stop
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        lx_speed = float(input("Input the linear speed (cell /sec): "))
        az_speed = float(input("Input the angular speed (rad /sec): "))
        d = float(input("Input the duration (sec): "))
        turtle.enact(lx_speed, az_speed, d)


    except rospy.ROSInterruptException:
        pass
