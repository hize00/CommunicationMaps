#!/usr/bin/env python

from copy import deepcopy
import math
import os
import pickle
import random
import sys
import threading
import time

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Vector3, PoseWithCovarianceStamped, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool, Int8, Float32

import communication
from environment import Environment
import exploration_strategies
from GPmodel import GPmodel
import utils
from utils import conv_to_hash, eucl_dist, get_index_from_coord
from strategy.msg import SignalData, SignalMappingAction, SignalMappingGoal, SignalMappingFeedback, \
                        SignalMappingResult, Plan, RobotData
from strategy.srv import GetSignalData, GetSignalDataResponse

TIME_STUCK = 3.0
TIME_AGAINST_WALL = 2.5
SPEED = 0.45 # TODO It should depend on the settings of the planner.
TIME_TOL_PERC = 0.04 # TODO It should depend on the settings of the planner and velocity.
REPLAN_RATE = 10 # Frequency rate at which the leader is checking again the plan

#TURTLEBOT CUSTOM
MIN_SCAN_ANGLE_RAD_FRONT = -30.0*3.14/180.0
MAX_SCAN_ANGLE_RAD_FRONT = 30.0*3.14/180.0

MIN_FRONT_RANGE_DIST = 1.5 # TODO It should depend on the settings of the planner.
MAX_NUM_ERRORS = 300
MAX_TIMEOUT_EXPIRED = 5

WALL_DIST = 6 #in pixels: offices = 6, open = 14
PATH_DISC = 1 #m

class GenericRobot(object):
    def __init__(self, seed, robot_id, is_leader, sim, comm_range, map_filename,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, resize_factor,
                 errors_filename, polling_freq, selection_policy, client_topic='move_base'):

        self.robot_id = robot_id
        self.sim = sim
        self.seed = seed
        random.seed(seed + robot_id)
        self.map_filename = map_filename
        self.teammates_id = teammates_id
        self.is_leader = is_leader
        self.n_robots = n_robots
        self.selection_policy = selection_policy

        self.errors_filename = errors_filename
        self.error_count = 0

        # for ending the mission
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
        rospy.Timer(rospy.Duration(10), self.distance_logger_callback)

        self.comm_dataset_filename = comm_dataset_filename
        log_dataset_file = open(comm_dataset_filename, "w")
        log_dataset_file.close()

        self.polling_freq = polling_freq
        rospy.Timer(rospy.Duration(self.polling_freq), self.polling_callback)

        # recovery
        self.last_feedback_pose = None
        self.stuck = False
        self.last_motion_time = None
        self.pub_motion_rec = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
        self.front_range = 0.0

        #estimated position
        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.1), self.tf_callback)
        self.pub_my_pose = rospy.Publisher("updated_pose", Point, queue_size=100)

        rospy.Subscriber('/robot_' + str(self.robot_id) + '/updated_pose', Point, self.pose_callback)
        self.x = 0.0
        self.y = 0.0
        self.last_x = None
        self.last_y = None
        self.traveled_dist = 0.0

        self.robots_pos = [(0.0, 0.0) for _ in xrange(self.n_robots)]

        for i in xrange(self.n_robots):
            if i == robot_id: continue
            s = "def a_" + str(i) + "(self, msg): self.robots_pos[" + str(i) + "] = (msg.x, msg.y)"
            exec s
            exec ("setattr(GenericRobot, 'pos_teammate" + str(i) + "', a_" + str(i) + ")")
            exec ("rospy.Subscriber('/robot_" + str(i) + "/updated_pose', Point, self.pos_teammate" + str(i) + ", queue_size = 100)")

        #stuff for navigation
        self.plans = []
        self.starting_pose = True
        self.alone = False
        self.fixed_wall_poses = []
        self.problematic_poses = []
        self.distance = -1
        self.timer = None
        self.timeout_expired_count = 0
        self.lock_data = threading.Lock()

        env_name = (os.path.splitext(self.map_filename)[0]).split("/")[-1]
        self.info_filename = '/home/andrea/catkin_ws/src/strategy/log/' + \
                             "info_" + str(self.n_robots) + '_' + str(env_name) + \
                             "_" + self.selection_policy + '.txt'

        self.robot_dict = dict()

        for i in xrange(self.n_robots): #every robot has its own dict with all important information to share
            self.robot_dict[i] = dict([('id', self.robot_id), ('arrived_nominal_dest', False),
                                       ('timestep', -1), ('teammate', -1),
                                       ('execute_plan_state', -1), ('time_expired', False)])

        self.myself = self.robot_dict[self.robot_id] #to directly access to my dictionary

        if self.is_leader:
            self.plans_folder = '/home/andrea/catkin_ws/src/strategy/plans/'

        self.replan_rate = REPLAN_RATE

        # useful topics
        self.pub_robot_data = rospy.Publisher('robot_data', RobotData, queue_size= 10)

        for i in xrange(self.n_robots):
            if i == robot_id: continue
            rospy.Subscriber('/robot_' + str(i) + '/robot_data', RobotData, self.robot_data_callback)

        rospy.Timer(rospy.Duration(0.1), self.publish_stuff)

    def tf_callback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', rospy.get_namespace() + 'base_link', rospy.Time(0))
            self.robots_pos[self.robot_id] = (trans[0],trans[1])
            self.pub_my_pose.publish(Point(trans[0],trans[1],0.0))
        except Exception as e:
            pass

    def pose_callback(self, msg):
        self.x = self.robots_pos[self.robot_id][0]
        self.y = self.robots_pos[self.robot_id][1]

        if self.last_x is not None:
            self.traveled_dist += utils.eucl_dist((self.x, self.y),(self.last_x, self.last_y))

        self.last_x = self.x
        self.last_y = self.y

    def robot_data_callback(self, msg):
        for i in xrange(self.n_robots):
            if i == msg.id:
                self.robot_dict[i]['id'] =  msg.id
                self.robot_dict[i]['arrived_nominal_dest'] = msg.arrived_nominal_dest
                self.robot_dict[i]['timestep'] =  msg.timestep
                self.robot_dict[i]['teammate'] =  msg.teammate
                self.robot_dict[i]['execute_plan_state'] = msg.execute_plan_state
                self.robot_dict[i]['time_expired'] = msg.time_expired

    def publish_stuff(self, event):
        robot_data = RobotData()
        robot_data.id = self.robot_id
        robot_data.arrived_nominal_dest = self.myself['arrived_nominal_dest']
        robot_data.timestep = self.myself['timestep']
        robot_data.teammate = self.myself['teammate']
        robot_data.execute_plan_state = self.myself['execute_plan_state']
        robot_data.time_expired = self.myself['time_expired']
        self.pub_robot_data.publish(robot_data)

    def scan_callback(self, scan):
        min_index = int(math.ceil((MIN_SCAN_ANGLE_RAD_FRONT - scan.angle_min) / scan.angle_increment))
        max_index = int(math.floor((MAX_SCAN_ANGLE_RAD_FRONT - scan.angle_min) / scan.angle_increment))
        self.front_range = min(scan.ranges[min_index: max_index])

    def file_writer(self, file):
        mode = 'a' if os.path.exists(file) else 'w'
        f = open(file, mode)
        f.write(str(self.seed) + " ---> " + self.map_filename + " ---> " + self.selection_policy + " ---> " +
                (str(self.n_robots) if file == self.errors_filename else str(self.robot_id)) +
                 ((" ---> errors: " + str(self.error_count) + " --- timeout_expired: " + str(self.timeout_expired_count) + '\n')
                  if file == self.info_filename else
                  (" --> too many errors\n" if self.error_count == MAX_NUM_ERRORS else " --> too many timeouts expired\n")))
        f.close()

    def distance_logger_callback(self, event):
        f = open(self.log_filename, "a")
        f.write('D ' + str((rospy.Time.now() - self.mission_start_time).secs) + ' ' + str(self.traveled_dist) + '\n')
        f.close()

    def new_data_writer(self, timestep, x, y, teammate_x, teammate_y, strength, c_alg):
        new_data = SignalData()
        new_data.signal_strength = strength
        new_data.my_pos.pose.position.x = x
        new_data.my_pos.pose.position.y = y
        new_data.teammate_pos.pose.position.x = teammate_x
        new_data.teammate_pos.pose.position.y = teammate_y
        new_data.timestep = timestep

        f = open(self.comm_dataset_filename, "a")
        f.write(str(new_data.timestep) +
                ' ' + str(new_data.my_pos.pose.position.x) + ' ' + str(new_data.my_pos.pose.position.y) +
                ' ' + str(new_data.teammate_pos.pose.position.x) + ' ' + str(new_data.teammate_pos.pose.position.y) +
                ' ' + str(new_data.signal_strength) +
                (' C\n' if c_alg else '\n')) # writing 'C' if data is found by using Pairing TSP algorithm
        f.close()

    def polling_callback(self, event):
        for robot in range(self.n_robots):
            if robot == self.robot_id: continue
            if not self.comm_module.can_communicate(robot): continue
            if(utils.eucl_dist((self.robots_pos[self.robot_id][0], self.robots_pos[self.robot_id][1]),
                               (self.robots_pos[robot][0],self.robots_pos[robot][1])) > self.comm_range): continue

            self.new_data_writer(int((rospy.Time.now() - self.mission_start_time).secs),
                                 self.robots_pos[self.robot_id][0], self.robots_pos[self.robot_id][1],
                                 self.robots_pos[robot][0], self.robots_pos[robot][1],
                                 self.comm_module.get_signal_strength(robot, safe = False), False)

    def feedback_motion_cb(self, feedback):
        if (self.last_feedback_pose is not None and abs(
                feedback.base_position.pose.position.x - self.last_feedback_pose[0]) <= 1e-3 and
                abs(feedback.base_position.pose.position.y - self.last_feedback_pose[1]) <= 1e-3):
            if (rospy.Time.now() - self.last_motion_time) > rospy.Duration(TIME_STUCK):
                self.error_count += 1
                if self.error_count == MAX_NUM_ERRORS:
                    self.file_writer(self.errors_filename)
                    if self.sim:
                        rospy.loginfo(str(self.robot_id) + ' - Mission aborted, too many errors.')
                        os.system("pkill -f ros")
                    else:
                        self.client_motion.cancel_goal()

                self.stuck = True
            else:
                self.stuck = False
        else:
            self.last_motion_time = rospy.Time.now()
            self.stuck = False

        if self.stuck:
            self.client_motion.cancel_goal()

        self.last_feedback_pose = (feedback.base_position.pose.position.x, feedback.base_position.pose.position.y)

    def bump_fwd(self):
        for i in range(10):
            msg = Twist(Vector3(0.7, 0, 0), Vector3(0, 0, 0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.1))

    def bump_bkw(self):
        for i in range(10):
            msg = Twist(Vector3(-0.7, 0, 0), Vector3(0, 0, 0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.1))

    def motion_recovery(self):
        if self.sim:
            #simpy move forward the robot
            start = rospy.Time.now()
            while self.front_range < MIN_FRONT_RANGE_DIST:
                msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.0))
                self.pub_motion_rec.publish(msg)
                rospy.sleep(rospy.Duration(0.2))

                if (rospy.Time.now() - start) > rospy.Duration(TIME_AGAINST_WALL):  # against wall
                    self.bump_bkw()
                    start = rospy.Time.now()

            self.bump_fwd()
        else:
            self.clear_costmap_service()
            msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.2))
            self.clear_costmap_service()

    def check_time_expired(self):
        distance_coverage_time = self.distance / SPEED

        if (rospy.Time.now() - self.timer).secs > rospy.Duration(60 + int(2 * distance_coverage_time)).secs:
            rospy.loginfo(str(self.robot_id) + ' - robot cannot reach its destination in time')
            self.myself['time_expired'] = True

        return self.myself['time_expired']

    def fix_pose(self, old_pos, fix):
        found = False
        timeout = time.time() + 5  #5 seconds
        fix_count = 0
        max_fix_count = 5
        pos = []
        while not found:
            pos = list(old_pos)
            pos[0] = round(random.uniform(pos[0] - fix, pos[0] + fix), 2)
            pos[1] = round(random.uniform(pos[1] - fix, pos[1] + fix), 2)
            pos = tuple(pos)

            p = list(pos)  # conversion for all_free()
            p[0] = self.comm_module.I - int(pos[1] / resize_factor)
            p[1] = int(pos[0] / resize_factor)

            if self.env.all_free(p[0], p[1], self.comm_module.I, self.comm_module.J, WALL_DIST) and \
                    communication.numObstaclesBetweenRobots(self.comm_module.im_array,
                                                                self.comm_module.I,
                                                                old_pos, pos, resize_factor) == 0:
                found = True
            else:
                if time.time() >= timeout:
                    fix += 0.5
                    fix_count += 1
                    if fix_count == max_fix_count:
                        self.myself['time_expired'] = True
                        rospy.loginfo(str(self.robot_id) + ' - WALL_DIST too large: I cannot find a fix position.')
                        return None

        return pos

    def go_to_pose(self, pos):
        if self.starting_pose:
            rospy.loginfo(str(self.robot_id) + ' - moving to starting position ' + str(pos))
        elif self.alone:
            rospy.loginfo(str(self.robot_id) + ' - moving alone to final position ' + str(pos))
        else:
            rospy.loginfo(str(self.robot_id) + ' - moving to ' + str(pos))

        success = False
        old_pos = pos
        already_found = False
        already_tried = False
        fixing_pose = False
        fix = 1
        loop_count = 0
        fixing_attempts = 0
        max_attempts = 3
        radius = 1.5

        self.timer = rospy.Time.now()

        while not success:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = '/map'
            goal.target_pose.pose.position = Point(pos[0], pos[1], 0.000)
            goal.target_pose.pose.orientation.w = 1
            self.last_feedback_pose = None
            self.last_motion_time = rospy.Time.now()

            # Start moving
            self.client_motion.send_goal(goal, feedback_cb = self.feedback_motion_cb)
            self.client_motion.wait_for_result()
            state = self.client_motion.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(str(self.robot_id) + ' - position reached')
                success = True
                self.myself['arrived_nominal_dest'] = True

                if fixing_pose and pos not in self.fixed_wall_poses:
                    self.fixed_wall_poses.append(pos)

            elif state == GoalStatus.PREEMPTED:
                if loop_count == 0:
                    rospy.loginfo(str(self.robot_id) + ' - preempted, using recovery')

                    if pos not in self.problematic_poses:
                        self.problematic_poses.append(pos)
                    else:
                        for pose in self.fixed_wall_poses:
                            pos = list(pos)
                            if (pose[0] - pos[0])**2 + (pose[1] - pos[1])**2 <= radius**2: #if a pose is in a circle centered in pos
                                rospy.loginfo(str(self.robot_id) +
                                              ' - moving to the fixed position already found: ' + str(pose))
                                pos[0] = pose[0]
                                pos[1] = pose[1]
                                already_found = True
                            pos = tuple(pos)

                if loop_count >= 4:
                    if not already_found:
                        fixing_pose = True

                        if not already_tried:
                            already_tried = True
                            rospy.loginfo(str(self.robot_id) + ' - preempted, trying to fix the goal after too many recoveries')
                            if pos in self.fixed_wall_poses:
                                self.fixed_wall_poses.remove(pos) # removing a fixed position that is not working anymore

                        pos = self.fix_pose(old_pos, fix)

                        if pos:
                            rospy.loginfo(str(self.robot_id) + ' - moving to new fixed goal ' + str(pos))
                            loop_count = 1
                            fixing_attempts += 1
                            if fixing_attempts == max_attempts: #increasing the radius of fixing area
                                fix += 0.5
                                fixing_attempts = 0
                        else:
                            break

                    else:
                        if loop_count == 7:
                            already_found = False
                            loop_count = 4

                self.clear_costmap_service()
                self.motion_recovery()
                self.clear_costmap_service()

                loop_count += 1

                if self.myself['time_expired']:
                    success = True
                else:
                    success = self.check_time_expired()
            elif state == GoalStatus.ABORTED:
                rospy.logerr(str(self.robot_id) + " - motion aborted by the server! Trying to recover")
                self.clear_costmap_service()
                self.motion_recovery()
                self.clear_costmap_service()
                success = False
            else:
                rospy.loginfo(str(self.robot_id) + ' - state: ' + str(state))
                break

    def reset_stuff(self):
        self.myself['arrived_nominal_dest'] = False
        self.myself['timestep'] = -1
        self.myself['teammate'] = -1
        self.myself['time_expired'] = False
        self.distance = -1
        self.last_feedback_pose = None
        self.last_motion_time = None

    def distance_to_cover(self, plan):
        start = (self.robots_pos[self.robot_id][0],self.robots_pos[self.robot_id][1])
        destination = (plan.my_dest.position.x, plan.my_dest.position.y)
        start_index = utils.get_index_from_coord(self.env, self.env.get_closest_cell(start, True))
        destination_index = utils.get_index_from_coord(self.env, self.env.get_closest_cell(destination, True))
        self.distance = self.env.sps_lengths[start_index][destination_index] #shortest paths lengths from start to dest

    def set_plans_info(self):
        if self.plans:
            plan_count = 1
            for plan in self.plans:
                rospy.loginfo(str(self.robot_id) + ' - plan: ' + str(plan_count) + '/' + str(len(self.plans)))
                plan_count += 1

                self.reset_stuff()

                self.myself['teammate'] = plan.comm_robot_id
                self.myself['timestep'] = plan.timestep

                if self.robot_id == self.myself['teammate'] and not self.starting_pose:
                    # for the last plan, if I have to move to the last destination
                    self.alone = True

                self.distance_to_cover(plan)
                self.go_to_pose((plan.my_dest.position.x, plan.my_dest.position.y))

                if not self.starting_pose and not self.alone:
                    self.lock_data.acquire()
                    self.check_signal_strength()
                    self.lock_data.release()

                if self.starting_pose:
                    self.starting_pose = False

        else:
            rospy.loginfo(str(self.robot_id) + ' - no plans to follow')

        self.myself['execute_plan_state'] = 2

    def check_signal_strength(self):
        my_teammate = self.robot_dict[self.myself['teammate']]

        rate = rospy.Rate(10 * self.replan_rate)
        success = False
        while not success:
            if (self.myself['timestep'] == my_teammate['timestep'] and
                my_teammate['id'] == self.myself['teammate'] and
                self.robot_id == my_teammate['teammate']) and \
                    (my_teammate['arrived_nominal_dest'] or my_teammate['time_expired']):
                success = True
            rate.sleep()

        if not my_teammate['time_expired'] and not self.myself['time_expired']:
            rospy.loginfo(str(self.robot_id) + ' - calculating the signal strength with teammate ' + str(self.myself['teammate']))

            self.new_data_writer(int((rospy.Time.now() - self.mission_start_time).secs),
                                 self.robots_pos[self.robot_id][0], self.robots_pos[self.robot_id][1],
                                 self.robots_pos[self.myself['teammate']][0], self.robots_pos[self.myself['teammate']][1],
                                 self.comm_module.get_signal_strength(self.myself['teammate'], safe = False), True)
        else:
            self.timeout_expired_count += 1
            if self.timeout_expired_count == MAX_TIMEOUT_EXPIRED:
                rospy.loginfo(str(self.robot_id) + ' - Mission aborted, too many timeouts expired.')
                self.file_writer(self.errors_filename)
                os.system("pkill -f ros")
            else:
                rospy.loginfo(str(self.robot_id) + ' - time is expired, changing destination')

        rospy.sleep(rospy.Duration(0.2))

    def end_exploration(self):
        rospy.loginfo(str(self.robot_id) + ' - Arrived at final destination.')
        rospy.loginfo(str(self.robot_id) + ' - self.errors = ' + str(self.error_count) +
                      ', self.timeout_expired = ' + str(self.timeout_expired_count))

        self.file_writer(self.info_filename)

        all_arrived = False
        while not all_arrived:
            if self.is_leader:
                if all(self.robot_dict[id]['execute_plan_state'] == 2 for id in xrange(self.n_robots)):
                    all_arrived = True
            else:
                rospy.sleep(rospy.Duration(10))

        rospy.loginfo(str(self.robot_id) + ' - All robots have arrived at final destinations. Sending shutdown.')
        os.system("pkill -f ros")
        rospy.sleep(rospy.Duration(2))

    # -1: plan, 0: leader sets plans
    # 1: leader/followers follow plans waiting for their teammate
    # 2: completed
    def execute_plan(self):
        r = rospy.Rate(self.replan_rate)

        while not rospy.is_shutdown():
            if self.myself['execute_plan_state'] == -1:
                if self.is_leader:
                    self.parse_plans_file()
                else:
                    self.myself['execute_plan_state'] = 0

            elif self.myself['execute_plan_state'] == 0:
                if self.is_leader:
                    self.assign_plans_to_robots()
                else:
                    self.check_leader_state()

            elif self.myself['execute_plan_state'] == 1:
                self.set_plans_info()

            elif self.myself['execute_plan_state'] == 2: #plan completed
                self.end_exploration()

            r.sleep()


class Leader(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename,
                 disc_method, disc, log_filename, teammates_id, n_robots, ref_dist,
                 env_filename, comm_dataset_filename, resize_factor, tiling, errors_filename,
                 communication_model, polling_freq, selection_policy):

        rospy.loginfo(str(robot_id) + ' - Leader - starting!')
        if not (os.path.exists(env_filename)):
            print "Creating new environment."
            f = open(env_filename, "wb")
            self.env = Environment(map_filename, disc_method, disc, resize_factor, comm_range,communication_model)
            pickle.dump(self.env, f)
            f.close()
        else:
            f = open(env_filename, "rb")
            self.env = pickle.load(f)
            f.close()

        super(Leader, self).__init__(seed, robot_id, True, sim, comm_range, map_filename,
                                     log_filename, comm_dataset_filename, teammates_id, n_robots,
                                     ref_dist, resize_factor, errors_filename, polling_freq, selection_policy)

        rospy.loginfo(str(self.robot_id) + ' - Created environment variable')


        self.comm_map = GPmodel(self.env.dimX, self.env.dimY, comm_range, tiling, self.comm_module.comm_model, self.log_filename)
        self.comm_maps = []

        #for sending commands to the follower
        #the name of the server is the name of the robot itself
        self.clients_signal = {}
        for teammate_id in teammates_id:
            self.clients_signal[teammate_id] = actionlib.SimpleActionClient('/robot_' + str(teammate_id) + '/main_robot', SignalMappingAction)
            rospy.loginfo(str(self.robot_id) + ' - Leader - waiting for follower server ' + str(teammate_id))
            self.clients_signal[teammate_id].wait_for_server()
            rospy.loginfo(str(self.robot_id) + ' - Done.')

    def parse_plans_file(self):
        rospy.loginfo(str(self.robot_id) + ' - Leader - planning')

        coord = []
        robot_moving = []
        robot_plan = []
        timetable = []
        reading_coords = 0
        reading_RM = 0
        reading_TT = 0
        env_name = (os.path.splitext(map_filename)[0]).split("/")[-1]

        with open(self.plans_folder + 'solution_plan_' + str(self.n_robots) + '_robots_' +
                  env_name + '_' + selection_policy + '.txt', 'r') as file:
            data = file.readlines()
            for line in data:
                words = line.split()
                if len(words) > 0:
                    if words[0] == "N_ROBOTS:":
                        N_ROBOTS = int(words[1])
                        line_lenght = N_ROBOTS * 2 + N_ROBOTS - 1
                    elif words[0] == "COORDINATES_LIST:":
                        reading_coords = 1
                        continue
                    if reading_coords == 1:
                        if words[0] != ';':
                            for i in range(0, line_lenght):
                                if words[i] != '|':
                                    coord.append(int(words[i]))
                        else:
                            reading_coords = 0
                    elif words[0] == "ROBOT_MOVING:":
                        reading_RM = 1
                        continue
                    if reading_RM == 1:
                        min_list_RM = []
                        if words[0] != ';':
                            for i in range(0, N_ROBOTS):
                                min_list_RM.append(int(words[i]))
                            robot_moving.append(min_list_RM)
                        else:
                            reading_RM = 0
                    elif words[0] == "TIMETABLE:":
                        reading_TT = 1
                        continue
                    if reading_TT == 1:
                        min_list_T = []
                        if words[0] != ';':
                            for i in range(0, N_ROBOTS):
                                min_list_T.append(float(words[i]))
                            timetable.append(min_list_T)
                        else:
                            reading_TT = 0

        file.close()

        # grouping coordinates by (x,y)
        coord = [coord[i:i + 2] for i in range(0, len(coord), 2)]  # group x and y of a single robot

        # converting from pixels to meters
        for c in coord:
            pos = c
            c[0] = float(resize_factor * pos[0])
            c[1] = float(self.env.dimY - resize_factor * pos[1])

        coord = [tuple(l) for l in coord]
        coord = [coord[i:i + N_ROBOTS] for i in range(0, len(coord), N_ROBOTS)]  # creating a plan of coordinates

        # creating the plan
        count_config = 0
        for config in robot_moving:
            count = 0
            first_robot = -1
            for robot in config:
                if (count_config == (len(robot_moving) - 1) and robot != 0) or (count_config == 0 and robot == 0) :
                    # adding the starting/final positions of each robot to the plan: [my_self,(pose),(pose), my_self, timestep]
                    my_self = count
                    robot_plan.append(my_self)
                    position = count
                    robot_plan.append(coord[count_config][position])
                    robot_plan.append(coord[count_config][position])
                    robot_plan.append(my_self)
                    robot_plan.append(timetable[count_config][position])
                else:
                    if robot != 0 and first_robot == -1:
                        first_robot = count
                        robot_plan.append(first_robot)
                        position = count
                        robot_plan.append(coord[count_config][position])
                    elif robot != 0 and first_robot != -1:
                        second_robot = count
                        position = count
                        robot_plan.append(coord[count_config][position])
                        robot_plan.append(second_robot)
                        robot_plan.append(timetable[count_config][position])

                        # assigning a specular plan to the communication teammate
                        robot_plan.append(second_robot)
                        robot_plan.append(coord[count_config][position])
                        robot_plan.append(coord[count_config][first_robot])
                        robot_plan.append(first_robot)
                        robot_plan.append(timetable[count_config][position])

                count += 1
            count_config += 1

        # grouping plan elements: [my_id, (my_coords),(teammate_coords), teammate_id, timestep]
        plan_elements = 5 #my_id is used only for grouping the robots plans at the end of the calculation
        robot_plan = [robot_plan[i:i + plan_elements] for i in range(0, len(robot_plan), plan_elements)]

        # grouping plan elements: [(my_id, (((my_coords), (teammate_coords)), teammate_id, timestep)]
        plans = []
        for plan in robot_plan:
            my_id = []
            coordinates = []
            msgs = []
            my_id.append(plan[0])
            coordinates.append(plan[1])
            coordinates.append(plan[2])
            msgs.append(tuple(coordinates))
            msgs.append(plan[3])
            msgs.append(plan[4]) #timestep
            my_id.append(tuple(msgs))
            plans.append(tuple(my_id))

        plans = tuple(plans)

        # grouping plan elements by robot_id
        robot_ids = set(map(lambda x: x[0], plans))
        plan_id = [[y[1] for y in plans if y[0] == x] for x in robot_ids]
        plan_id = tuple([tuple(l) for l in plan_id])  # plans = (plan_robot_0, plan_robot_1,...,plan_robot_n)

        self.plans = plan_id

        self.myself['execute_plan_state'] = 0

    def assign_plans_to_robots(self):
        rospy.loginfo(str(self.robot_id) + ' - assigning plans to each robot')
        plans_leader = []
        clients_messages = []
        plan_index = 0
        for robots_plans in self.plans:
            for robot in xrange(self.n_robots):
                if robot == plan_index:
                    #rospy.loginfo(str(robot) + ' - PLAN: ' + str(robots_plans))

                    plans_robot = []
                    for plan in robots_plans:
                        points = plan[0]

                        plan_robot = Plan()

                        # my_destination
                        plan_robot.my_dest = Pose()
                        plan_robot.my_dest.position.x = points[0][0]
                        plan_robot.my_dest.position.y = points[0][1]

                        # teammate_destination
                        plan_robot.teammate_dest = Pose()
                        plan_robot.teammate_dest.position.x = points[1][0]
                        plan_robot.teammate_dest.position.y = points[1][1]

                        # teammate_id
                        plan_robot.comm_robot_id = Int8
                        plan_robot.comm_robot_id = plan[1]

                        # timestep
                        plan_robot.timestep = Float32
                        plan_robot.timestep = plan[2]

                        plans_robot.append(plan_robot)

                    if robot in self.teammates_id:
                        goal = SignalMappingGoal(plans_robot = plans_robot)
                        clients_messages.append((robot, goal))
                    else:
                        plans_leader = plans_robot

            plan_index += 1

        goal_threads = []

        for (robot, goal) in clients_messages:
            t = threading.Thread(target=self.send_and_wait_goal, args=(robot, goal))
            rospy.sleep(rospy.Duration(1.0))
            t.start()
            goal_threads.append(t)

        for t in goal_threads:
            t.join()

        self.plans = plans_leader
        self.myself['execute_plan_state'] = 1

    def send_and_wait_goal(self, teammate_id, goal):
        rospy.loginfo(str(self.robot_id) + ' - Leader - sending a new goal for follower ' + str(teammate_id))
        self.clients_signal[teammate_id].send_goal(goal)

        self.clients_signal[teammate_id].wait_for_result()
        rospy.loginfo(str(self.robot_id) + ' - Leader - received the result of ' + str(teammate_id))


class Follower(GenericRobot):
    _result   = SignalMappingResult()

    def __init__(self, seed, robot_id, sim, comm_range, map_filename,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                 resize_factor, errors_filename, polling_freq, selection_policy):

        rospy.loginfo(str(robot_id) + ' - Follower - starting!')

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

        super(Follower, self).__init__(seed, robot_id, False, sim, comm_range, map_filename,
                                       log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist,
                                       resize_factor, errors_filename, polling_freq, selection_policy)

        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(self._action_name, SignalMappingAction,
                                                execute_cb=self.execute_callback, auto_start=False)

        rospy.loginfo(str(self.robot_id) + ' - Follower - created environment variable!')

        self._as.start()

    def execute_callback(self,goal):
        rospy.loginfo(str(self.robot_id) + ' - Follower - received new goal ')

        success = False
        for plan in goal.plans_robot:
            success = True
            self.plans.append(plan)

        if not success:
            rospy.loginfo(str(self.robot_id) + ' - Follower failed last point.')

        rospy.loginfo(str(self.robot_id) + ' - Follower - action succeded')

        self._as.set_succeeded(self._result)

    def check_leader_state(self):
        leader_id = self.teammates_id[0]
        if self.robot_dict[leader_id]['execute_plan_state'] != 1:
            while not self.robot_dict[leader_id]['execute_plan_state'] == 1:
                pass

        self.myself['execute_plan_state'] = 1


if __name__ == '__main__':
    rospy.init_node('robot')

    robot_id = int(rospy.get_param('~id'))
    sim = int(rospy.get_param('/sim'))
    seed = int(rospy.get_param('/seed'))
    ref_dist = int(rospy.get_param('/ref_dist'))
    map_filename = rospy.get_param('/map_filename')
    disc = float(rospy.get_param('/disc'))
    comm_range = float(rospy.get_param('/range'))
    disc_method = rospy.get_param('/disc_method')
    env_filename = rospy.get_param('/env_filename')

    communication_model = rospy.get_param('/communication_model', "")
    if communication_model is "":
        env_filename = env_filename.replace(".dat","_" + str(int(comm_range)) + ".dat")
    else:
        env_filename = env_filename.replace(".dat","_" + str(int(comm_range)) + '_' + communication_model + ".dat")
    print env_filename

    teammates_id_temp = str(rospy.get_param('~teammates_id'))
    is_leader = int(rospy.get_param('~is_leader'))
    n_robots = int(rospy.get_param('/n_robots'))
    resize_factor = float(rospy.get_param('/resize_factor'))
    tiling = int(rospy.get_param('/tiling'))
    log_folder = rospy.get_param('/log_folder')
    polling_freq = rospy.get_param('/polling_freq')
    selection_policy = rospy.get_param('/selection_pol')

    temmates_id_temp = teammates_id_temp.split('-')
    teammates_id = map(lambda x: int(x), temmates_id_temp)

    env = (map_filename.split('/')[-1]).split('.')[0]
    if communication_model is "":
        log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + \
                       str(n_robots) + '_' + str(int(comm_range)) + '_' + selection_policy + '.log'
    else:
        log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + \
                       str(n_robots) + '_' + str(int(comm_range)) + '_' + communication_model + '_' + selection_policy + '.log'
    if communication_model is "":
        comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + \
                                '_' + str(n_robots) + '_' + str(int(comm_range)) + '_' + selection_policy + '.dat'
    else:
        comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + \
                                '_' + str(n_robots) + '_' + str(int(comm_range)) + '_' + communication_model + \
                                '_' + selection_policy +'.dat'

    errors_filename = log_folder + 'errors_' + str(n_robots) + '_' + str(env) + '_' + str(selection_policy) + '.log'
    print "Logging possible errors to: " + errors_filename

    if is_leader:
        lead = Leader(seed, robot_id, sim, comm_range, map_filename,
                      disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename,
                      comm_dataset_filename, resize_factor, tiling, errors_filename, communication_model,
                      polling_freq, selection_policy)
        lead.execute_plan()

    else:
        foll = Follower(seed, robot_id, sim, comm_range, map_filename, log_filename,
                        comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                        resize_factor, errors_filename, polling_freq, selection_policy)
        foll.execute_plan()
