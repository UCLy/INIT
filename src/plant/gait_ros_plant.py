#!/usr/bin/env python

# Copyright 2019 Dynamic Object Language Labs Inc.
#
# This software is licensed under the terms of the
# Apache License, Version 2.0 which can be found in
# the file LICENSE at the root of this distribution.

import argparse
import sys
import time
from multiprocessing import Process, Queue
from pprint import pprint
import traceback
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

try:
    import plant
except ImportError as error:
    # Output expected ImportErrors.
    print(error.__class__.__name__ + ": " + error.message)
    print('Ensure PYTHONPATH has plant.py')
except Exception as exception:
    # Output unexpected Exceptions.
    print(exception, False)
    print(exception.__class__.__name__ + ": " + exception.message)

# Globals
rmq = None
init = None
PI = 3.1415926535897
scan_in_front = None
total_interactions = 0

class RosActionClient:

    def __init__(self, server_name, msg_type):
        self.server_name = server_name
        self.client = actionlib.SimpleActionClient(server_name, msg_type)
        # Waits until the action server has started up and started
        # listening for goals.
        print 'Waiting for action server:', server_name, ;
        print '- done'
        self.client.wait_for_server()

    def translate_goal_state(self, state):
        lookup = {0: 'pending',
                  1: 'active',
                  2: 'prempted',  # failed for plant
                  3: 'succeeded',
                  4: 'aborted',
                  5: 'rejected',
                  6: 'prempting',
                  7: 'recalling',
                  8: 'recalled',  # cancelled for plant
                  9: 'lost'}  # If we see this, programming bug in our code using simpleactionclient
        return lookup[state]

    def send_goal(self, command):
        print 'sending command', command, 'to', self.server_name

        # Sends the goal to the action server.
        self.client.send_goal(command)

        # Waits for the server to finish performing the action.
        print 'Waiting for result'
        self.client.wait_for_result()

        # Prints out the result of executing the action
        cmd_result = self.client.get_result()
        cmd_state = self.translate_goal_state(self.client.get_state())
        print self.server_name, 'command result:', cmd_result
        print self.server_name, 'command state:', cmd_state
        return {'result': cmd_result, 'state': cmd_state}


def ros_to_rmq_state(state):
    lookup = {'pending': 'started',
              'active': 'started',
              'prempted': 'failed',
              'succeeded': 'success',
              'aborted': 'failed',
              'rejected': 'failed',
              'prempting': 'failed',
              'recalling': 'cancelled',
              'recalled': 'cancelled',
              'lost': 'failed'}

    return lookup[state]

class Init:
    def __init__(self, to_rmq, from_rmq):
        self.to_rmq = to_rmq
        self.from_rmq = from_rmq
        self.laser_callback = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.move_forward_action = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.left_rotate_action = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.right_rotate_action = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def laser_callback(self, msg):
        global scan_in_front
        global scan_right
        global scan_left
        scan_in_front = msg.ranges[0]
        scan_right = msg.ranges[-90]
        scan_left = msg.ranges[90]

    def touch_in_front(self):
        table = [0]
        table.append(scan_in_front)
        if scan_in_front < 0.7:
            table[0] = 1
        print table
        return table

    def touch_right(self):
        table = [0]
        table.append(scan_right)
        if scan_right < 0.7:
            table[0] = 1
        print table
        return table

    def touch_left(self):
        table = [0]
        table.append(scan_left)
        if scan_left < 0.7:
            table[0] = 1
        print table
        return table

    def move_forward(self):
        table = [0]

        rate = rospy.Rate(1)
        vel_msg = Twist()

        #input
        speed = 0.20
        distance = 1
        forward = 1

        #we won't use this
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #forward or backwards
        if forward:
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)

        current_distance = 0

        while(current_distance < distance) and scan_in_front > 0.7:
            self.move_forward_action.publish(vel_msg)
            current_distance += speed
            print(current_distance, speed)
            print("scan_in_front: ", scan_in_front)
            rate.sleep()

        vel_msg.linear.x = 0
        self.move_forward_action.publish(vel_msg)

        if scan_in_front < 0.7:
            table[0] = 1
            print "Obstacle detected"
        print "Interaction completed"
        return table

    def left_rotate(self):
        table = [1]

        rate = rospy.Rate(1)
        vel_msg = Twist()

        #input
        speed = 30
        angle = 90
        clockwise = 0

        #converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #we won't use this
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        #clockwise or counterclockwise
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)

        current_angle = 0

        while (current_angle < relative_angle):
            self.left_rotate_action.publish(vel_msg)
            current_angle += angular_speed
            print(current_angle, relative_angle)
            rate.sleep()

        vel_msg.angular.z = 0
        self.left_rotate_action.publish(vel_msg)
        print "Rotation completed"
        return table

    def right_rotate(self):
        table = [1]

        rate = rospy.Rate(1)
        vel_msg = Twist()

        #input
        speed = 30
        angle = 90
        clockwise = 1

        #from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #we won't use this
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        #clockwise or counterclockwise
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)

        current_angle = 0

        while (current_angle < relative_angle):
            self.right_rotate_action.publish(vel_msg)
            current_angle += angular_speed
            print(current_angle, relative_angle)
            rate.sleep()
        
        vel_msg.angular.z = 0
        self.right_rotate_action.publish(vel_msg)
        print "Rotation completed"
        return table


class Rmq:
    """
    Class to interface RMQ plant messaging with Ardu Copter
    """

    def __init__(self, plantid, exchange, host, port, to_rmq, from_rmq):
        self.plant = plant.Plant(plantid, exchange, host, port)
        self.to_rmq = to_rmq
        self.from_rmq = from_rmq
        self.done = False
        self.last_rmq_call_back = time.time()

    def move_forward(self, msg):
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('move_forward_observation', init.move_forward())]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def left_rotate(self, msg):
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('left_rotate_observation', init.left_rotate())]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def right_rotate(self, msg):
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('right_rotate_observation', init.right_rotate())]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def touch_in_front(self, msg):
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('touch_in_front_observation', init.touch_in_front())]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def touch_right(self, msg):
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('touch_right_observation', init.touch_right())]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def touch_left(self, msg):
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('touch_left_observation', init.touch_left())]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def dispatch_func(self, msg, rkey_):
        if "number-of-actions" in msg:
            self.number_of_actions
        if 'function-digit' in msg:
            self.handle_fn(msg)
        else:
            print('unhandled message')
            pprint(msg)

    def handle_fn(self, msg):
        self.perform(msg)

    def perform(self, msg):
        print('Got message', msg)
        fn_digit = msg['function-digit']
        print('Got message', 'fn_digit: ', fn_digit)
        if fn_digit == '0':
            self.touch_in_front(msg)
        elif fn_digit == '1':
            self.touch_right(msg)
        elif fn_digit == '2':
            self.touch_left(msg)
        elif fn_digit == '3':
            self.move_forward(msg)
        elif fn_digit == '4':
            self.right_rotate(msg)
        elif fn_digit == '5':
            self.left_rotate(msg)
        else:
            print('RMQ Unknown function', msg['function-digit'])
            self.plant.failed(msg, "Unknown function for cps ros plant" + msg['function-digit'])

    def number_of_actions(self, msg):
        table = [6]
        self.plant.started(msg)
        myobservations = [self.plant.make_observation('number_of_actions_observation', table)]
        print('Publishing result observation: ', myobservations)
        self.plant.observations(msg, myobservations)
        self.plant.finished(msg)

    def subscribe_and_wait(self):
        self.plant.wait_for_messages(self.dispatch_func)

    def shutdown(self):
        self.done = True
        print('RMQ Shut down')
        self.plant.close()

def main(args):
    global rmq
    global init

    from_rmq = Queue()
    to_rmq = Queue()

    rmq = Rmq(args.plantid, args.exchange, args.host, args.port, to_rmq, from_rmq)
    rospy.init_node('GAIT_ROS_plant')
    init = Init(to_rmq, from_rmq)

    try:
        rmq.subscribe_and_wait()
    except Exception as e:
        traceback.print_exc()
        print('Ignoring exception as we are shutting down', e.__class__.__name__ + ": " + e.message)

    print('Done test_plant main')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test Plant')
    parser.add_argument('--host', default='localhost', help='RMQ host')
    parser.add_argument('-p', '--port', default=5672, help='RMQ Port', type=int)
    parser.add_argument('-e', '--exchange', default='test', help='RMQ Exchange')
    parser.add_argument('--plantid', default="init-1", help='default plant id')

    args = parser.parse_args()
    pprint(args)
    main(args)
    sys.exit(0)
