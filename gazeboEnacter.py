#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class GazeboEnacter:

    def __init__(self):
        """ Creating our node, publisher and subscriber """
        rospy.init_node('gazebo_enacter', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/turtle1/pose', Odometry, self.callback)
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

    def callback(self, data):
        """ Callback function implementing the pose value received """
        self.odom = data
        self.odom.x = round(self.odom.x, 4)
        self.odom.y = round(self.odom.y, 4)

    def move(self, linear_speed=0.0, angular_speed=0.0, duration=1.0):
        """ Enacting a movement and returning the outcome """
        vel_msg = Twist()

        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_speed

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
        # print(Position x=% 2.1f, y=% 2.1f" % (self.pose.x, self.pose.y))

        # return outcome 1 if position is against the wall
        if 0.1 < self.odom.x < 10.9 and 0.1 < self.odom.y < 10.9:  # ici on doit mettre la partie de Mr thollin
            return 0
        else:
            return 1

    def outcome(self, action):
        """ Enacting an action and returning the outcome """
        if action == 0:
            # move forward
            return self.move(linear_speed=1)
        elif action == 1:
            # rotate left
            return self.move(linear_speed=0.5, angular_speed=1)
        elif action == 2:
            # rotate right
            return self.move(linear_speed=0.5, angular_speed=-1)
        else:
            return 0


if __name__ == '__main__':
    """ Main """
    try:
        x = GazeboEnacter()
        choice = input("Type 0 to enter values, or 1 to enter interactions: ")
        if choice == 0:
            lx_speed = float(input("Input the linear speed (cell /sec): "))
            az_speed = float(input("Input the angular speed (rad /sec): "))
            d = float(input("Input the duration (sec): "))
            outcome = x.move(lx_speed, az_speed, d)
            print("Outcome:% 1d" % outcome)
        elif choice == 1:
            interaction = 0
            while interaction < 3:
                interaction = input("Type 0 to move forward, 1 to rotate left, 2 to rotate right, or 3 to stop: ")
                if interaction == 0:
                    outcome = x.move(linear_speed=1)
                    print("Outcome:% 1d" % outcome)
                elif interaction == 1:
                    outcome = x.move(linear_speed=0.5, angular_speed=1)
                    print("Outcome:% 1d" % outcome)
                elif interaction == 2:
                    outcome = x.move(linear_speed=0.5, angular_speed=-1)
                    print("Outcome:% 1d" % outcome)

    except rospy.ROSInterruptException:
        pass
