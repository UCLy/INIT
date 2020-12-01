#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

arret = None

"""La fonction callback recupere les donnees de la structure de donnees LaserScan
    via la variable arret elle est declaree comme variable global afin qu il soit utilisees
    dans n importe quelle fonction du Noeud"""


def callback(msg):
    global arret
    print("je marche")
    arret = msg.ranges[0]


"""La fonction laser retourne les donnees de la variable
    qui lui sera passee en parametre"""


def laser(dist):
    if dist is None:
        return 0
    else:
        return dist


def obstacle(dist_arret):
    rospy.Subscriber('odom', Odometry)
    sub = rospy.Subscriber('scan', LaserScan, callback)
    return_value = 0
    test = laser(arret)
    print(arret)
    print(test)
    if test > dist_arret:
        print("0")
    elif test <= dist_arret:
        print("1")
        return_value = 1
    return return_value


obstacle(0.5)