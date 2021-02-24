#!/usr/bin/env python
import rospy
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class GazeboEnacter:

    def __init__(self):
        """ Creation of the GazeboEnacter object """

        # Message to inform user
        rospy.loginfo("Press CTRL+c to stop TurtleBot")

        # Keys CNTL + c will stop script
        rospy.on_shutdown(self.shutdown)

        """ Creating our node, publisher and subscriber """
        rospy.init_node('gazebo_enacter', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/turtle1/pose', Odometry, self.callback)

        self.odom = Odometry()
        self.rate = rospy.Rate(10)
        self.numpyArray = numpy.full(360, numpy.inf)

    def callback(self, data):
        """ Callback function implementing the pose value received """
        print(data)
        self.odom = data
        self.odom.x = round(self.odom.x, 4)
        self.odom.y = round(self.odom.y, 4)

    def getTurtleBotPos(self, data):
        """ Callback function that collects information from LaserScan

            These informations are stored in the variable data and we take
            the ranges array where are stored all the distances between the
            robot and the obstacle at 360° and at a maximum distance of 3.5 metres.
        """
        self.numpyArray = numpy.array(data.ranges)
        # print("Valeurs brut : " + str(data.ranges[0]))
        # print("Original type: " + str(type(data.ranges[0])))
        # print("Tableau numpy : " + str(numpyArray[0]))
        # print("Numpy type: " + str(type(numpyArray)))
        # print("distObstacleDevant : " + str(data.ranges[180]))

    def move(self, safe_dist=0.4, linear_speed=0.0, angular_speed=0.0, duration=5.0):
        """ Enacting a movement and returning the outcome """
        self.laser_subscriber = rospy.Subscriber('scan', LaserScan, self.getTurtleBotPos)
        self.current_position = rospy.Subscriber('odom', Odometry, None)
        vel_msg = Twist()
        warning = False

        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_speed

        # Setting the current time for duration calculus
        t0 = float(rospy.Time.now().to_sec())

        # Loop to move the turtle during a specific duration

        # while float(rospy.Time.now().to_sec()) - t0 < duration:
        #     # Publish the velocity
        #     self.velocity_publisher.publish(vel_msg)
        #     self.rate.sleep()

        while float(rospy.Time.now().to_sec()) - t0 < duration:
            # print(str(self.numpyArray[0]))
            if self.numpyArray[
                0] < safe_dist:  # or self.numpyArray[90] < safe_dist or self.numpyArray[270] < safe_dist:
                # print("diego")
                warning = True
                self.velocity_publisher.publish(Twist())
                # break
            else:
                # print("dora")
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
        # print("c fini")

        # After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        # Publish the null velocity to force the robot to stop
        self.velocity_publisher.publish(vel_msg)

        # return outcome 1 if the robot is close to a wall
        if warning:
            return 1
        else:
            return 0

    # def updatePosition(self, data):

    def outcome(self, action):
        """ Enacting an action and returning the outcome """
        if action == 0:
            # move forward
            return self.move(linear_speed=0.2)
        elif action == 1:
            # rotate left
            return self.move(safe_dist=0.2, linear_speed=0, angular_speed=0.3)
        elif action == 2:
            # rotate right
            return self.move(safe_dist=0.2, linear_speed=0, angular_speed=-0.3)
        else:
            return 0

    def shutdown(self):
        # You can stop turtlebot by publishing an empty Twist message
        rospy.loginfo("Stopping TurtleBot")

        self.velocity_publisher.publish(Twist())
        # Give TurtleBot time to stop
        rospy.sleep(1)
        return 0


if __name__ == '__main__':
    """ Main """
    try:
        x = GazeboEnacter()
        lx_speed = float(input("Input the linear speed (cell /sec): "))
        az_speed = float(input("Input the angular speed (rad /sec): "))
        d = float(input("Input the duration (sec): "))
        x.move(lx_speed, az_speed, d)
        # choice = input("Type 0 to enter values, or 1 to enter interactions: ")
        # if choice == 0:
        #     lx_speed = float(input("Input the linear speed (cell /sec): "))
        #     az_speed = float(input("Input the angular speed (rad /sec): "))
        #     d = float(input("Input the duration (sec): "))
        #     outcome = x.move(lx_speed, az_speed, d)
        #     print("Outcome:% 1d" % outcome)
        # elif choice == 1:
        #     interaction = 0
        #     while interaction < 3:
        #         interaction = input("Type 0 to move forward, 1 to rotate left, 2 to rotate right, or 3 to stop: ")
        #         if interaction == 0:
        #             outcome = x.move(linear_speed=1)
        #             print("Outcome:% 1d" % outcome)
        #         elif interaction == 1:
        #             outcome = x.move(linear_speed=0.5, angular_speed=1)
        #             print("Outcome:% 1d" % outcome)
        #         elif interaction == 2:
        #             outcome = x.move(linear_speed=0.5, angular_speed=-1)
        #             print("Outcome:% 1d" % outcome)

    except rospy.ROSInterruptException:
        pass
