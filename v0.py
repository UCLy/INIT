#!/usr/bin/env python
# Execute as a python script
# Set linear and angular values of TurtleBot's speed and turning.
import rospy      # Needed to create a ROS node
from geometry_msgs.msg import Twist    # Message that moves base

class turtleBotForward():

  def __init__(self):
    # Message to screen
    rospy.loginfo("Press CTRL+c to stop TurtleBot")

    # Keys CNTL + c will stop script
    rospy.on_shutdown(self.shutdown)

    # ControlTurtleBot is the name of the node sent to the #master
    rospy.init_node('turtleBotForward', anonymous=False)

    self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    # TurtleBot will receive the message 10 times per second.
    rate = rospy.Rate(10);
    # 10 Hz is fine as long as the processing does not exceed
    #   1/10 second.

    # Twist is geometry_msgs for linear and angular velocity
    vel_info = Twist()
    # (backwards)
    vel_info.linear.x = 2
    # Modify this value to change speed
    # Turn at 0 radians/s
    vel_info.angular.z = 0
    # Modify this value to cause rotation rad/s

    # Loop and TurtleBot will move until you type CNTL+c
    while not rospy.is_shutdown():
      # publish Twist values to TurtleBot node /cmd_vel_mux
      self.velocity_publisher.publish(vel_info)
      # wait for 0.1 seconds (10 HZ) and publish again
      rate.sleep()


  def shutdown(self):
    # You can stop turtlebot by publishing an empty Twist
    # message
    rospy.loginfo("Stopping TurtleBot")

    self.velocity_publisher.publish(Twist())
    # Give TurtleBot time to stop
    rospy.sleep(1)

if __name__ == '__main__':
  try:
    turtle = turtleBotForward()
    turtle.move()
  except:
    rospy.loginfo("End of the trip for TurtleBot")

