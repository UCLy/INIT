#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#Le ce noeud "avanceObstacle.py" permet d'avancer de 20 cm, et de s'arrêter à la rencontre d'un obsctacle.

# def spawn():
#     rospy.wait_for_service('circle')
#     tout_droit = rospy.ServiceProxy('tout_droit', ToutDroit)


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

def etalonnage(velocity_publisher): # etalonnage function
    global start_position, current_position, arret
    #pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
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
    while not rospy.is_shutdown() and distance(start_position, current_position) < 0.5 and t==0 :
        #print "dans la fonction globale, distance =", distance(start_position, current_position),"\n"
        #print "LaserScan =", laser(arret)
        test = laser(arret)
        if test > 0.5 :
            vel_msg.linear.x = 1
            print("0")
        if test < 0.5 :
            #print "dans la fonction globale, distance =", distance(start_position, current_position)
            vel_msg.linear.x = 0
            print("1")
            t = t + 1
        velocity_publisher.publish(vel_msg)
        rate.sleep()
    
    #vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #print("Finish")
    

    


if __name__=="__main__":
    #rospy.init_node('etalonnage')
    etalonnage()
