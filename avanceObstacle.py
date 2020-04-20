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

# Déclaration des variables initialisées à None (Objet Python qui exprime l'absence de valeur)
start_position = None
current_position = None
arret = None

""" La fonction distance calcul la distance parcourue
  entre la position de départ (pos1) et la position courante (pos2) """
def distance(pos1, pos2):
    if pos1 is None or pos2 is None:
        return 0
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

""" La fonction maj_distance récupére les données de la structure de données Odometry,
    précisement le champ pose.pose.position, start_position et current_position sont déclarées
    comme variable global afin qu'ils soient utilisées dans n'importe quelle fonction du Noeud """
def maj_distance(msg):
    global start_position, current_position
    current_position = msg.pose.pose.position

    if start_position is None:
        start_position = current_position

""" La fonction callback récupére les données de la structure de données LaserScan
    via la variable arret, elle est déclarée comme variable global afin qu'il soit utilisées
    dans n'importe quelle fonction du Noeud """
def callback(msg):
    global arret 
    arret = msg.ranges[0]

""" La fonction laser retourne, les données de la variable
    qui lui sera passée en paramètre """
def laser(dist):
    if dist is None:
        return 0
    else:
        return dist

""" La fonction etalonnage avec velocity_publisher en paramètre pour récupérer les données
    de la structure de données Twist, via un autre noeud dans lequel cette structure de données sera importée """
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
    
""" La variable t permet d'ajouter une condition de sortie de la boucle while
    en l'incrémentant de 1, dans le cas où la valeur du LaserScan est inférieur à 0.5 """
    t = 0
    while not rospy.is_shutdown() and distance(start_position, current_position) < 0.5 and t==0 :
        
        # La variable test contient l'appel de la fonction laser avec la variable arret en paramètre
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
