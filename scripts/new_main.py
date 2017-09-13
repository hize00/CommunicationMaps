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

#TODO NON C'e LA COMMUNICATION, PER ORA (i robot non comunicano tra loro, non ci sono errori o problemi di comunicazione)

class GenericRobot(object):
    def __init__(self, robot_id, sim, seed, map_filename, duration, teammates_id, is_leader,
                 comm_range, resize_factor,
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

        # for communication
        self.comm_range = comm_range
        self.comm_module = communication.CommunicationModule(sim, seed, robot_id, n_robots, comm_range, ref_dist,
                                                             map_filename, resize_factor)

        # for logging
        self.log_filename = log_filename
        log_file = open(log_filename, "w")
        log_file.close()

        self.comm_dataset_filename = comm_dataset_filename
        log_dataset_file = open(comm_dataset_filename, "w")
        log_dataset_file.close()


'''
        # estimated position TODO in TF e non in acml
        #rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.x = 0.0
        self.y = 0.0
        self.last_x = None
        self.last_y = None
        self.traveled_dist = 0.0

        # other robots' estimated position - cur robot position remains 0.0 here
        self.other_robots_pos = [(0.0, 0.0) for _ in xrange(n_robots)]
        self.last_robots_polling_pos = [None for _ in xrange(n_robots)]

        for i in xrange(n_robots):
            if i == robot_id: continue
            s = "def a_" + str(i) + "(self, msg): self.other_robots_pos[" + str(i) + "] = (msg.pose.pose.position.x, msg.pose.pose.position.y)"
            exec (s)
            exec ("setattr(GenericRobot, 'callback_pos_teammate" + str(i) + "', a_" + str(i) + ")")
            exec ("rospy.Subscriber('/robot_" + str(i) + "/amcl_pose', PoseWithCovarianceStamped, self.callback_pos_teammate" + str(i) + ", queue_size = 100)")

        self.lock_info = threading.Lock()

        # recovery
        self.last_feedback_pose = None
        self.stuck = False
        self.last_motion_time = None
        self.pub_motion_rec = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
        self.front_range = 0.0


        # for maintaining and selectively publishing signal data
        self.robot_data_list = []
        for r in xrange(n_robots):
            self.robot_data_list.append([])

        rospy.Service('/robot_' + str(self.robot_id) + '/get_signal_data', GetSignalData, self.handle_get_signal_data)

        # subscribe to all the other robots - will never call mine
        self.get_signal_data_service_proxies = []
        for r in xrange(n_robots):
            service_name = '/robot_' + str(r) + '/get_signal_data'
            rospy.wait_for_service(service_name)
            self.get_signal_data_service_proxies.append(rospy.ServiceProxy(service_name, GetSignalData))
'''




class Leader(GenericRobot):
    def __init__(self, robot_id, sim, seed, map_filename, disc, disc_method, duration, env_filename, teammates_id,
                 n_robots, tiling, error_filename, log_filename, comm_dataset_filename,
                 resize_factor,comm_range, communication_model):

        rospy.loginfo(str(robot_id) + ' - Leader - starting!')
        if (not (os.path.exists(env_filename))):
            print "Creating new environment."
            f = open(env_filename, "wb")
            self.env = Environment(map_filename, disc_method, disc, resize_factor, comm_range,communication_model)
            pickle.dump(self.env, f)
            f.close()
        else:
            f = open(env_filename, "rb")
            self.env = pickle.load(f)
            f.close()

        super(Leader, self).__init__(seed, robot_id, True, sim, map_filename, duration, log_filename,
                                     comm_dataset_filename, teammates_id, n_robots, errors_filename,
                                     resize_factor,comm_range)

        print 'created environment variable'
        rospy.loginfo(str(robot_id) + ' - Created environment variable')

        self.comm_map = GPmodel(self.env.dimX, self.env.dimY, tiling,self.log_filename)
        self.comm_maps = []  # for logging













class Follower(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename, duration, log_filename, comm_dataset_filename,
                 teammates_id, n_robots, env_filename, resize_factor, errors_filename):
        rospy.loginfo(str(robot_id) + ' - Follower - starting!')
        # Load Environment for follower to filter readings.
        environment_not_loaded = True
        while environment_not_loaded:
            try:
                f = open(env_filename, "rb")
                self.env = pickle.load(f)
                f.close()
                environment_not_loaded = False
            except:  # TODO specific exception.
                rospy.logerr(str(robot_id) + " - Follower - Environment not loaded yet.")
                rospy.sleep(1)

        super(Follower, self).__init__(seed, robot_id, False, sim, comm_range, map_filename , duration, log_filename,
                                       comm_dataset_filename, teammates_id, n_robots, resize_factor, errors_filename)

#TODO PEZZO DA AGGIUNGERE

        print 'created environment variable'

        rospy.loginfo(str(robot_id) + ' - Follower - created environment variable!')

        self._as.start()





if __name__ == '__main__':
    rospy.init_node('robot')

    #attributi del robot
    robot_id = int(rospy.get_param('~id'))
    sim = int(rospy.get_param('/sim'))
    seed = int(rospy.get_param('/seed'))
    map_filename = rospy.get_param('/map_filename')
    disc = float(rospy.get_param('/disc'))
    disc_method = rospy.get_param('/disc_method')
    duration = float(rospy.get_param('/duration'))
    env_filename = rospy.get_param('/env_filename')
    env_filename = env_filename.replace(".dat","_")
    teammates_id_temp = str(rospy.get_param('~teammates_id'))
    is_leader = int(rospy.get_param('~is_leader'))
    n_robots = int(rospy.get_param('/n_robots'))
    tiling = int(rospy.get_param('/tiling'))
    log_folder = rospy.get_param('/log_folder')

    comm_range = float(rospy.get_param('/range'))
    communication_model = rospy.get_param('/communication_model', "")
    resize_factor = float(rospy.get_param('/resize_factor'))

    teammates_id_temp = teammates_id_temp.split('-')
    teammates_id = map(lambda x: int(x), teammates_id_temp)

    env = (map_filename.split('/')[-1]).split('.')[0]
    log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + str(n_robots) + '.log'
    comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + str(n_robots) + '.dat'

    errors_filename = log_folder + 'errors.log'
    print "Logging possible errors to: " + errors_filename

    lead = Leader(robot_id, sim, seed, map_filename, disc, disc_method, duration, env_filename, teammates_id,
                  n_robots, tiling, errors_filename, log_filename, comm_dataset_filename,
                  resize_factor,comm_range, communication_model)
