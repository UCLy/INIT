#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#Le noeud avanceObstacle.py permet d avancer de 20 cm et de s arreter a la rencontre d un obsctacle.

# def spawn():
#     rospy.wait_for_service('circle')
#     tout_droit = rospy.ServiceProxy('tout_droit', ToutDroit)

# Declaration des variables initialisees a None : Objet Python qui exprime l absence de valeur
start_position = None
current_position = None
arret = None

"""La fonction distance calcul la distance parcourue
  entre la position de depart (pos1) et la position courante (pos2)"""
def distance(pos1, pos2):
    if pos1 is None or pos2 is None:
        return 0
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

"""La fonction majdistance recupere les donnees de la structure de donnees Odometry
    precisement le champ pose.pose.position startposition et currentposition sont declarees
    comme variable global afin qu ils soient utilisees dans n importe quelle fonction du Noeud"""
def maj_distance(msg):
    global start_position, current_position
    current_position = msg.pose.pose.position

    if start_position is None:
        start_position = current_position

"""La fonction callback recupere les donnees de la structure de donnees LaserScan
    via la variable arret elle est declaree comme variable global afin qu il soit utilisees
    dans n importe quelle fonction du Noeud"""
def callback(msg):
    global arret 
    arret = msg.ranges[0]

"""La fonction laser retourne les donnees de la variable
    qui lui sera passee en parametre"""
def laser(dist):
    if dist is None:
        return 0
    else:
        return dist

"""La fonction etalonnage avec velocitypublisher en parametre pour recuperer les donnees
    de la structure de donnees Twist via un autre noeud dans lequel cette structure de donnees sera importee"""
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
    
"""La variable t permet d ajouter une condition de sortie de la boucle while
    en l incrementant de 1 dans le cas ou la valeur du LaserScan est inferieur a 0.5"""
    t = 0
    while not rospy.is_shutdown() and distance(start_position, current_position) < 0.5 and t==0 :
        
        # La variable test contient l appel de la fonction laser avec la variable arret en parametre
        test = laser(arret)
        if test > 0.5 :
            vel_msg.linear.x = 1
            print("0")
        if test < 0.5 :
            vel_msg.linear.x = 0
            print("1")
            t = t + 1
        velocity_publisher.publish(vel_msg)
        rate.sleep()
    
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #print("Finish")
    

    


if __name__=="__main__":
    # call function etalonnage
    etalonnage()
