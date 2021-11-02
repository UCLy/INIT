# !/usr/bin/env python

# !/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
SALUT !!!
Control Your TurtleBot3!
---------------------------
Moving around:
        z
   q    s    d
        x    o	
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop
o : Noeud avanceObstacle
CTRL-C to quit
"""

e = """
Communications Failed
"""

# Declaration des variables initialisees a None : Objet Python qui exprime l'absence de valeur
start_position = None
current_position = None
arret = None


def distance(pos1, pos2):
    if pos1 is None or pos2 is None:
        return 0
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)


def maj_distance(msg):
    global start_position, current_position
    current_position = msg.pose.pose.position

    if start_position is None:
        start_position = current_position


def callback(msg):
    global arret
    arret = msg.ranges[0]


def laser(dist):
    if dist is None:
        return 0
    else:
        return dist


def etalonnage(velocity_publisher):
    global start_position, current_position, arret
    rospy.Subscriber('odom', Odometry, maj_distance)
    sub = rospy.Subscriber('scan', LaserScan, callback)

    vel_msg = Twist()
    vel_msg.linear.x = 1
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    operations_par_seconde = 50.
    rate = rospy.Rate(operations_par_seconde)

    t = 0
    while not rospy.is_shutdown() and distance(start_position, current_position) < 0.5 and t == 0:

        # La variable test contient l appel de la fonction laser avec la variable arret en parametre
        test = laser(arret)
        if test > 0.5:
            vel_msg.linear.x = 1
            print("0")
        if test < 0.5:
            vel_msg.linear.x = 0
            print("1")
            t = t + 1
        velocity_publisher.publish(vel_msg)
        rate.sleep()

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while 1:
            key = getKey()

            if key == 'z':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'q':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'o':
                # call function etalonnage
                # pub est une variable dans laquelle est stockee des donnees publiee par le topic cmd_vel
                etalonnage(pub)
                status = status + 1
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    # call function etalonnage
    etalonnage()