#!/usr/bin/env python

from copy import deepcopy
import math
import os
import pickle
import random
import sys
import threading

import rospy
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

class Configuration():
    def __init__(self, conf_id, robots):
        self.conf_id = conf_id
        self.robots = robots


#GenericRobot is a generic robot in a generic configuration
class GenericRobot(object):
    def __init__(self, seed, robot_id, is_leader, sim, comm_range, map_filename, polling_signal_period, duration,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, configuration, ref_dist, strategy, resize_factor, errors_filename, client_topic='move_base'):
        self.robot_id = robot_id
        self.is_leader = is_leader
        self.map_filename = map_filename
        self.polling_signal_period = polling_signal_period
        self.teammates_id = teammates_id
        self.configuration = configuration
        self.n_robots = n_robots
        self.strategy = strategy #STRATEGY = RANDOM

        self.errors_filename = errors_filename
        self.error_count = 0
        self.sim = sim

        # for communication
        self.comm_range = comm_range
        self.comm_module = communication.CommunicationModule(sim, seed, robot_id, n_robots, comm_range, ref_dist, map_filename, resize_factor)

        # for ending the mission
        self.duration = rospy.Duration(duration)
        self.mission_start_time = rospy.Time.now()

        # for handling motion
        self.client_topic = client_topic
        self.client_motion = actionlib.SimpleActionClient(self.client_topic, MoveBaseAction)
        self.stop_when_comm = False  # for stopping the robot when in comm with teammate
        self.teammate_comm_regained = False
        self.client_motion.wait_for_server()
        rospy.logdebug('initialized action exec')
        self.clear_costmap_service = rospy.ServiceProxy('move_base_node/clear_costmaps', Empty)

        # estimated position
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
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
            exec(s)
            exec("setattr(GenericRobot, 'callback_pos_teammate" + str(i) + "', a_" + str(i) +")")
            exec("rospy.Subscriber('/robot_" + str(i) + "/amcl_pose', PoseWithCovarianceStamped, self.callback_pos_teammate" + str(i) + ", queue_size = 100)")

        self.lock_info = threading.Lock()

        """ TODO RECOVERY
        #recovery
        self.last_feedback_pose = None
        self.stuck = False
        self.last_motion_time = None
        self.pub_motion_rec = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
        self.front_range = 0.0
        """

        # for maintaining and publishing all the info known by the robot w.r.t the other robots (destination and path)
        self.robot_info_list = []
        for r in xrange(n_robots):
            robot_info = RobotInfo()
            robot_info.robot_id = -1
            self.robot_info_list.append(robot_info)

        self.pub_all_info = rospy.Publisher('all_info', AllInfo, queue_size=10)
        rospy.Timer(rospy.Duration(0.25), self.pub_all_info_callback)

        rospy.sleep(rospy.Duration(1))

        for r in range(n_robots):
            if r == robot_id: continue
            rospy.Subscriber('/robot_' + str(r) + '/all_info', AllInfo, self.update_all_info_callback)

        # for maintaining and selectively publishing signal data
        self.robot_data_list = []
        for r in xrange(n_robots):
            self.robot_data_list.append([])

        rospy.Service('/robot_' + str(self.robot_id) + '/get_signal_data', GetSignalData,
                      self.handle_get_signal_data)
        # subscribe to all the other robots - will never call mine
        self.get_signal_data_service_proxies = []
        for r in xrange(n_robots):
            service_name = '/robot_' + str(r) + '/get_signal_data'
            rospy.wait_for_service(service_name)
            self.get_signal_data_service_proxies.append(rospy.ServiceProxy(service_name, GetSignalData))

        # ask if new signal data is available each second
        rospy.Timer(rospy.Duration(1), self.ask_new_signal_data_callback)

        # for logging
        self.log_filename = log_filename
        log_file = open(log_filename, "w")
        log_file.close()
        rospy.Timer(rospy.Duration(10), self.distance_logger_callback)

        self.comm_dataset_filename = comm_dataset_filename
        log_dataset_file = open(comm_dataset_filename, "w")
        log_dataset_file.close()

        self.first_plan = True  # for coordinating multiple teams at the beginning

    def ask_new_signal_data_callback(self, event):
        #ask to each other robot in communication...
        for other_robot_id in xrange(self.n_robots):
            if other_robot_id == self.robot_id or not(self.comm_module.can_communicate(other_robot_id)): continue
            #if it holds more recent signal data w.r.t. my most recent - of each other robot
            for other_robot_id_2 in xrange(self.n_robots):
                if other_robot_id_2 == self.robot_id: continue
                last_timestep = 0 if len(self.robot_data_list[other_robot_id_2]) == 0 else self.robot_data_list[other_robot_id_2][-1].timestep
                try: #could become disconnected on real robots
                    resp = self.get_signal_data_service_proxies[other_robot_id](last_timestep, other_robot_id_2)
                    self.robot_data_list[other_robot_id_2] += resp.data
                except rospy.ServiceException, e:
                    rospy.loginfo(str(self.robot_id) + ' - failed service call while calling robot ' + str(other_robot_id))
                    break

                if(not(self.comm_module.can_communicate(other_robot_id))): break

    def handle_get_signal_data(self, req):
        return GetSignalDataResponse(filter(lambda x: x.timestep > req.timestep, self.robot_data_list[req.robot_id]))



