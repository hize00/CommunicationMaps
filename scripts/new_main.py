#!/usr/bin/env python

from copy import deepcopy
import math
import os
import pickle
import random
import sys
import threading

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Vector3, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool

import communication
from environment import Environment
import exploration_strategies
from GPmodel import GPmodel
import utils
from utils import conv_to_hash, eucl_dist
from strategy.msg import GoalWithBackup, SignalData, RobotInfo, AllInfo, SignalMappingAction, \
                               SignalMappingGoal, SignalMappingFeedback, SignalMappingResult
from strategy.srv import GetSignalData, GetSignalDataResponse

TIME_STUCK = 3.0
TIME_AGAINST_WALL = 5.0
SPEED = 0.45 # TODO It should depend on the settings of the planner.
TIME_TOL_PERC = 0.04 # TODO It should depend on the settings of the planner and velocity.
REPLAN_RATE = 10 # Frequency rate at which the leader is checking again the plan

#TURTLEBOT CUSTOM
MIN_SCAN_ANGLE_RAD_FRONT = -30.0*3.14/180.0
MAX_SCAN_ANGLE_RAD_FRONT = 30.0*3.14/180.0

#first random, max_var offices had 2
MIN_FRONT_RANGE_DIST = 1.5 # TODO It should depend on the settings of the planner.

MAX_NUM_ERRORS = 40

PATH_DISC = 1 #m

#TODO NON C'È LA COMMUNICATION, PER ORA (i robot non comunicano tra loro, non ci sono errori o problemi di comunicazione)

class GenericRobot(object):
    def __init__(self, robot_id, sim, seed, map_filename, duration, teammates_id, is_leader,
                 n_robots, errors_filename, log_filename, comm_dataset_filename, client_topic = 'move_base'):

        self.robot_id = robot_id
        self.sim = sim
        self.seed = seed
        random.seed(seed + robot_id)
        self.map_filename = map_filename
        self.teammates_id = teammates_id
        self.is_leader = is_leader
        self.n_robots = n_robots
        self.tol_dist = 2.5
        self.errors_filename = errors_filename
        self.error_count = 0

        #for ending the mission
        self.duration = rospy.Duration(duration)
        self.mission_start_time = rospy.Time.now()

        # for handling motion
        self.client_topic = client_topic
        self.client_motion = actionlib.SimpleActionClient(self.client_topic, MoveBaseAction)
        self.client_motion.wait_for_server()
        rospy.logdebug('initialized action exec')
        self.clear_costmap_service = rospy.ServiceProxy('move_base_node/clear_costmaps', Empty)

        # for logging
        self.log_filename = log_filename
        log_file = open(log_filename, "w")
        log_file.close()
        rospy.Timer(rospy.Duration(10), self.distance_logger_callback)

        self.comm_dataset_filename = comm_dataset_filename
        log_dataset_file = open(comm_dataset_filename, "w")
        log_dataset_file.close()

'''
 # estimated position
        rospy.Subscriber('/%s/pose' % robot_id, geometry_msgs.msg.PoseStamped, handle_robot_pose, robot_id)
        self.x = 0.0
        self.y = 0.0
        self.last_x = None
        self.last_y = None
        self.traveled_dist = 0.0
'''


if __name__ == '__main__':
    rospy.init('robot')

    #attributi del robot
    robot_id = int(rospy.get_param('~id'))
    sim = int(rospy.get_param('/sim'))
    seed = int(rospy.get_param('/seed'))
    map_filename = rospy.get_param('/map_filename')
    disc = float(rospy.get_param('/disc'))
    disc_method = rospy.get_param('/disc_method')
    duration = float(rospy.get_param('/duration'))
    env_filename = rospy.get_param('/env_filename')
    teammates_id = str(rospy.get_param('~teammates_id'))
    is_leader = int(rospy.get_param('~is_leader'))
    n_robots = int(rospy.get_param('/n_robots'))
    tiling = int(rospy.get_param('/tiling'))
    log_folder = rospy.get_param('/log_folder')
    errors_filename = log_folder + 'errors.log'
    print "Logging possible errors to: " + error_filename

    '''
     #for handling tf listener
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tf.StampedTransform()
            listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0), transform)
            x = transform.getOrigin().x()
            y = transform.getOrigin().y();
            cout << "Current position: (" << x << "," << y << ")" << endl;
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    '''






