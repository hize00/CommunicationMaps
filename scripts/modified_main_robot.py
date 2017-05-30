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

#NOTE: TIMEOUT IN TODO
#NOTE: MYSTRATEGY IN TODO

########################################################################################################################################################################
"""
#A configuration is a set Ci = <r1 = vertex1, r2 = vertex2, ..., rN = vertexN, timestampI>
class Configuration():
    def __init__(self, conf_id, robots, timestamp):
        self.conf_id = conf_id
        self.robots = robots
        self.timestamp = timestamp
"""

########################################################################################################################################################################

class StrategyParams(object):
    def __init__(self, samples_pairs_greedy=None, samples_leader_multi2=None, samples_follower_multi2_single=None,
                 mindist_vertices_multi2=None, mindist_vertices_maxvar=None):
        self.samples_pairs_greedy = samples_pairs_greedy
        self.samples_leader_multi2 = samples_leader_multi2
        self.samples_follower_multi2_single = samples_follower_multi2_single
        self.mindist_vertices_multi2 = mindist_vertices_multi2
        self.mindist_vertices_maxvar = mindist_vertices_maxvar

###############################################################################################################################

#GenericRobot is a generic robot in a generic configuration
class GenericRobot(object):
    def __init__(self, seed, robot_id, is_leader, sim, comm_range, map_filename, polling_signal_period, duration,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, """configuration""", ref_dist, strategy, resize_factor, errors_filename, client_topic='move_base'):
        #self.configuration = configuration
        self.seed = seed
        random.seed(seed + robot_id)
        self.robot_id = robot_id
        self.is_leader = is_leader
        self.teammates_id = teammates_id
        self.n_robots = n_robots
        self.polling_signal_period = polling_signal_period
        self.strategy_error_log = strategy
        self.map_filename = map_filename
        self.strategy = strategy
        self.sim = sim

        self.tol_dist = 2.5
        self.errors_filename = errors_filename
        self.error_count = 0

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


        #recovery
        self.last_feedback_pose = None
        self.stuck = False
        self.last_motion_time = None
        self.pub_motion_rec = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
        self.front_range = 0.0

        #TODO MYSTRATEGY (main_robot 130)

        # for polling signal strength
        # each time a new target is available, reset the list and start polling by setting polling signal to true
        # old
        # self.signal_data = []
        rospy.Timer(rospy.Duration(polling_signal_period), self.polling_signal_callback)
        self.iam_moving = False
        # from 4th max variance of grass, 4 robots, before was 0.09
        rospy.Timer(rospy.Duration(0.05), self.monitor_stop_motion_callback)


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

    def pub_all_info_callback(self, event):
        #if(self.strategy != 'multi2' and self.my_path is not None and self.teammate_path is not None and not(self.path_inserted_in_info)):

        self.robot_info_list[self.robot_id].path_leader = map(lambda x: Point(x.pose.position.x, x.pose.position.y, 0.0), self.my_path)
        self.robot_info_list[self.robot_id].path_follower = map(lambda x: Point(x.pose.position.x, x.pose.position.y, 0.0), self.teammate_path)
        self.robot_info_list[self.robot_id].timestep = int((rospy.Time.now() - self.mission_start_time).secs)
        self.robot_info_list[self.robot_id].timestep_path = int((rospy.Time.now() - self.mission_start_time).secs)
        rospy.loginfo(str(self.robot_id) + " inserting new paths in info to share")
        self.path_inserted_in_info = True

        all_info = AllInfo()
        all_info.all_info = self.robot_info_list
        all_info.sender = self.robot_id
        self.pub_all_info.publish(all_info)

    def fill_cur_destinations(self, dest_leader, dest_follower):
        if (self.robot_info_list[self.robot_id].robot_id == -1):
            self.robot_info_list[self.robot_id].robot_id = self.robot_id
            if (self.my_path is None or self.teammate_path is None):
                # the follower will always have empty lists
                self.robot_info_list[self.robot_id].path_leader = []
                self.robot_info_list[self.robot_id].path_follower = []

        self.robot_info_list[self.robot_id].timestep = int((rospy.Time.now() - self.mission_start_time).secs)
        self.robot_info_list[self.robot_id].dest_leader = Point(dest_leader[0], dest_leader[1], 0)
        self.robot_info_list[self.robot_id].dest_follower = Point(dest_follower[0], dest_follower[1], 0)

    def fill_signal_data(self, datum):
        self.robot_data_list[self.robot_id].append(datum)

    def update_all_info_callback(self, msg):
        if(self.sim and not(self.comm_module.can_communicate(msg.sender))):
            #rospy.loginfo(str(self.robot_id) + " cannot be updated with info received by " + str(msg.sender))
            return

        self.lock_info.acquire()
        for robot_info in msg.all_info:
            if robot_info.robot_id == -1 or robot_info.robot_id == self.robot_id:
                # -1 means invalid
                continue

            if (self.robot_info_list[robot_info.robot_id] is None or
                        robot_info.timestep > self.robot_info_list[robot_info.robot_id].timestep or
                        robot_info.timestep_path > self.robot_info_list[robot_info.robot_id].timestep_path):
                self.robot_info_list[robot_info.robot_id] = robot_info

            if (not (self.is_leader) and robot_info.robot_id in self.teammates_id and
                    (robot_info.dest_follower.x < -99999 or self.robot_info_list[
                        self.robot_id].dest_follower.x < -99999)):
                # print "Setting planning in follower!"
                self.robot_info_list[self.robot_id] = robot_info
                self.robot_info_list[self.robot_id].robot_id = self.robot_id
                self.robot_info_list[self.robot_id].path_leader = []
                self.robot_info_list[self.robot_id].path_follower = []

        self.lock_info.release()

    def distance_logger_callback(self, event):
        f = open(self.log_filename, "a")
        f.write('D ' + str((rospy.Time.now() - self.mission_start_time).secs) + ' ' + str(self.traveled_dist) + '\n')
        f.close()

        if((rospy.Time.now() - self.mission_start_time) >= self.duration):
            rospy.loginfo("Sending shutdown...")
            os.system("pkill -f ros")

    def monitor_stop_motion_callback(self, event):
        if (not (self.iam_moving)): return
        # If I am still moving towards first goal after 5 seconds (teammate path will be arrived meanwhile, otherwise
        # no path is arrived because teammate was already at goal)
        if (self.moving_nominal_dest and not (self.completed) and (rospy.Time.now() - self.time_start_path) >= rospy.Duration(3)):
            if ((rospy.Time.now() - self.time_start_path) > rospy.Duration(max(self.my_time_to_dest, self.teammate_time_to_dest))):
                self.path_timeout_elapsed = True
                self.client_motion.cancel_goal()
        #First part is also used in the first and final step of multi2 by followers - stop when comm always false for leaders
        elif ((self.stop_when_comm)):
            if (self.comm_module.can_communicate(self.teammates_id[0], for_stopping=True)):
                self.teammate_comm_regained = True
                self.client_motion.cancel_goal()

    def info_teammate_callback(self, event):
        if(self.moving_nominal_dest):
            self.pub_time_to_dest.publish(Float32(self.my_time_to_dest))

        self.pub_state.publish(Bool(self.arrived_nominal_dest))

    def reset_stuff(self):
        self.last_feedback_pose = None
        self.last_motion_time = None
        self.stop_when_comm = False
        self.teammate_comm_regained = False
        self.iam_moving = False

        self.path_timeout_elapsed = False
        self.my_path = None
        self.teammate_path = None
        self.my_time_to_dest = 0.0
        self.teammate_time_to_dest = 0.0
        self.time_start_path = None
        self.first_estimate_time = False
        self.first_estimate_teammate_path = False
        self.path_inserted_in_info = False
        self.moving_nominal_dest = False
        self.arrived_nominal_dest = False
        self.teammate_arrived_nominal_dest = False

    def compute_dist(self, poses):
        return sum(map(lambda x: utils.eucl_dist((x[0].pose.position.x, x[0].pose.position.y),(x[1].pose.position.x, x[1].pose.position.y)),
                       zip(poses[:-1], poses[1:])))

    def state_callback(self, msg):
        self.teammate_arrived_nominal_dest = msg.data

    def extrapolate_waypoints(self, poses):
        meters = PATH_DISC
        if(len(poses)==0): return []

        waypoints = [poses[0]]
        dist = 0.0
        for i in xrange(1,len(poses)):
            dist += utils.eucl_dist((poses[i].pose.position.x, poses[i].pose.position.y),
                                    (poses[i-1].pose.position.x, poses[i-1].pose.position.y))
            if(dist >= meters):
                waypoints.append(poses[i])
                meters += PATH_DISC

        return waypoints

    def teammate_path_callback(self, msg):
        if(not(self.moving_nominal_dest)): return #not needed

        if (self.first_estimate_teammate_path):
            self.teammate_path = self.extrapolate_waypoints(msg.poses)

            self.first_estimate_teammate_path = False

    def path_callback(self, msg):
        if(not(self.moving_nominal_dest)): return #not needed

        #cannot update estimated time -
        if (self.first_estimate_time):
            self.my_path = self.extrapolate_waypoints(msg.poses)
            self.my_time_to_dest = self.compute_dist(msg.poses)/SPEED
            self.my_time_to_dest = self.my_time_to_dest + TIME_TOL_PERC*self.my_time_to_dest

            self.first_estimate_time = False
            rospy.loginfo('Time to dest robot ' + str(self.robot_id) + ' = ' + str(self.my_time_to_dest))

    def teammate_time_path_callback(self, msg):
        if(not(self.moving_nominal_dest)): return
        #callback means I can actually communicate
        self.teammate_time_to_dest = float(msg.data)

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if(self.last_x is not None):
            self.traveled_dist += utils.eucl_dist((self.x, self.y),(self.last_x, self.last_y))

        self.last_x = self.x
        self.last_y = self.y

    def scan_callback(self, scan):
        min_index = int(math.ceil((MIN_SCAN_ANGLE_RAD_FRONT - scan.angle_min)/scan.angle_increment))
        max_index = int(math.floor((MAX_SCAN_ANGLE_RAD_FRONT - scan.angle_min)/scan.angle_increment))
        self.front_range = min(scan.ranges[min_index: max_index])

    def poll_signal(self):
        # Add myself and then other robot.
        # my_pos receiver.
        # teammate_pos sender.

        for other_robot_id in xrange(self.n_robots):
            if other_robot_id == self.robot_id: continue

            if (not (self.comm_module.can_communicate(other_robot_id))): continue

            # consider to call a service on real robots that, if succeds, returns the current teammate pos.

            # if both did not move do not make measurement
            if self.last_robots_polling_pos[other_robot_id] is not None:
                if (abs(self.x - self.last_robots_polling_pos[other_robot_id][0][0]) <= 1 and abs(self.y - self.last_robots_polling_pos[other_robot_id][0][1]) <= 1 and
                            abs(self.other_robots_pos[other_robot_id][0] - self.last_robots_polling_pos[other_robot_id][1][0]) <= 1 and
                            abs(self.other_robots_pos[other_robot_id][1] - self.last_robots_polling_pos[other_robot_id][1][1]) <= 1):
                    continue

                # Add for our specific scenarios, so we process data ONLY when x2,y2 of neighbor robot exists in selected_locations of x1,y1 pppph
                # Reportedly, this is not a good place to start.. we should change at a different location
                if hasattr(self.env, 'selected_locations'):
                    # Finding free_positions for teammate and myself.
                    teammate_x, teammate_y = self.env.get_closest_cell(
                        (self.last_robots_polling_pos[other_robot_id][1][0],
                         self.last_robots_polling_pos[other_robot_id][1][1]), True)
                    my_x, my_y = self.env.get_closest_cell((self.x, self.y), True)

                    # Finding candidate locations according to communication model.
                    candidate_location_hash_of_teammate = conv_to_hash(teammate_x, teammate_y)
                    candidate_location_hash_of_myself = conv_to_hash(my_x, my_y)
                    teammate_as_source_candidate_locations = self.env.selected_locations.get(candidate_location_hash_of_teammate, 0)
                    myself_as_source_candidate_locations = self.env.selected_locations.get(candidate_location_hash_of_myself, 0)

                    # Update environment with new selected locations in case it is needed.
                    if teammate_as_source_candidate_locations == 0:
                        self.env.update_selected_locations((teammate_x, teammate_y))
                        teammate_as_source_candidate_locations = self.env.selected_locations.get(candidate_location_hash_of_teammate, 0)
                    if myself_as_source_candidate_locations == 0:
                        self.env.update_selected_locations((my_x, my_y))
                        myself_as_source_candidate_locations = self.env.selected_locations.get(candidate_location_hash_of_myself, 0)

                    found_close_to_candidate_location = False

                    # Checking if current pose of the robot (self) is close to a candidate location in a list, considering the teammate as source
                    # or if the current pose of the teammate is close to a candidate location in a list, considering self as source.
                    if (len(filter(lambda dest: eucl_dist(dest, (self.x, self.y)) <= 1.0,teammate_as_source_candidate_locations)) > 0
                        or len(filter(lambda dest: eucl_dist(dest, (teammate_x, teammate_y)) <= 1.0,myself_as_source_candidate_locations)) > 0):  # TODO constant, maybe consistent with other constants in the project.
                        found_close_to_candidate_location = True

                    # If it is not close to a location that can be selected, then don't include measurement.
                    if not found_close_to_candidate_location:
                        continue

            # if I am not interested because I know it is too far
            if (utils.eucl_dist((self.x, self.y), self.other_robots_pos[other_robot_id]) > self.comm_range): continue

            # maybe not enough to be sure on real robots
            new_data = SignalData()
            new_data.signal_strength = self.comm_module.get_signal_strength(other_robot_id, safe=False)
            if new_data.signal_strength < self.comm_module.comm_model.CUTOFF:
                return  # means communication was interrupted while retrieving signal strength - also with real robots!
            new_data.my_pos.pose.position.x = self.x
            new_data.my_pos.pose.position.y = self.y
            new_data.teammate_pos.pose.position.x = self.other_robots_pos[other_robot_id][0]
            new_data.teammate_pos.pose.position.y = self.other_robots_pos[other_robot_id][1]
            self.last_robots_polling_pos[other_robot_id] = ((self.x, self.y), self.other_robots_pos[other_robot_id])
            new_data.timestep = int((rospy.Time.now() - self.mission_start_time).secs)

            self.fill_signal_data(new_data)

            f = open(comm_dataset_filename, "a")
            f.write(str(new_data.timestep) + ' ' + str(new_data.my_pos.pose.position.x) + ' ' + str(new_data.my_pos.pose.position.y) +
                    ' ' + str(new_data.teammate_pos.pose.position.x) + ' ' + str(new_data.teammate_pos.pose.position.y) +
                    ' ' + str(new_data.signal_strength) + '\n')

            f.close()

    def polling_signal_callback(self, event):
        self.poll_signal()

    def feedback_motion_cb(self, feedback):
        if (self.last_feedback_pose is not None and abs(feedback.base_position.pose.position.x - self.last_feedback_pose[0]) <= 1e-3 and
            abs(feedback.base_position.pose.position.y - self.last_feedback_pose[1]) <= 1e-3):
            if (rospy.Time.now() - self.last_motion_time) > rospy.Duration(TIME_STUCK):
                self.error_count += 1
                if (self.error_count == MAX_NUM_ERRORS):
                    mode = 'a' if os.path.exists(self.errors_filename) else 'w'
                    f = open(self.errors_filename, mode)
                    f.write(str(self.seed) + "--->" + self.map_filename + "--->" + str(self.n_robots) + "--->" + self.strategy_error_log + "\n")
                    f.close()
                    if self.sim:
                        # sim too biased by nav errors!
                        os.system("pkill -f ros")
                    else:
                        self.client_motion.cancel_goal()

                self.stuck = True
            else:
                self.stuck = False
        else:
            self.last_motion_time = rospy.Time.now()
            self.stuck = False

        # rospy.loginfo('Stuck: ' + str(self.stuck))
        if (self.stuck):
            # rospy.loginfo('Sending cancel goal.')
            self.client_motion.cancel_goal()

        self.last_feedback_pose = (feedback.base_position.pose.position.x, feedback.base_position.pose.position.y)

    def bump_fwd(self):
        for i in range(10):
            msg = Twist(Vector3(0.7,0,0), Vector3(0,0,0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.1))

    def bump_bkw(self):
        for i in range(10):
            msg = Twist(Vector3(-0.7,0,0), Vector3(0,0,0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.1))


    def motion_recovery(self):
        if self.sim:
            #simpy move forward the robot
            #rospy.loginfo(str(self.robot_id) + ' in motion recovery.')
            start = rospy.Time.now()
            while(self.front_range < MIN_FRONT_RANGE_DIST):
                #rospy.loginfo(str(self.robot_id) + ' rotating to avoid obstacle')
                msg = Twist(Vector3(0,0,0), Vector3(0,0,1.0))
                self.pub_motion_rec.publish(msg)
                rospy.sleep(rospy.Duration(0.2))

                if((rospy.Time.now() - start) > rospy.Duration(TIME_AGAINST_WALL)): #against wall
                    self.bump_bkw()
                    start = rospy.Time.now()

            #rospy.loginfo(str(self.robot_id) + ' now the space is free. Sending x speed.')
            self.bump_fwd()
        else:
            msg = Twist(Vector3(0,0,0), Vector3(0,0,1.0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.2))
            self.clear_costmap_service()


    def send_to(self, target, timeout=0): #TODO TIMEOUT
        rospy.loginfo(str(self.robot_id) + ' moving to ' + str(target))
        success = False

        while (not (success)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = '/map'
            goal.target_pose.pose.position.x = target[0]
            goal.target_pose.pose.position.y = target[1]
            goal.target_pose.pose.orientation.w = 1
            self.last_feedback_pose = None
            self.last_motion_time = rospy.Time.now()
            self.client_motion.send_goal(goal, feedback_cb=self.feedback_motion_cb)
            self.iam_moving = True
            self.client_motion.wait_for_result()
            self.iam_moving = False
            state = self.client_motion.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(str(self.robot_id) + " destination reached.")
                success = True
                if self.moving_nominal_dest:
                    rospy.loginfo(str(self.robot_id) + " setting arrived nominal dest to true.")
                    self.arrived_nominal_dest = True
            else:
                if (abs(self.x - target[0]) < self.tol_dist and abs(self.y - target[1]) < self.tol_dist):
                    rospy.loginfo(str(self.robot_id) + " navigation failed, but waypoint was reached.")
                    success = True
                    if self.moving_nominal_dest:
                        self.arrived_nominal_dest = True

                else:
                    if state == GoalStatus.PREEMPTED:
                        # If not able to reach destination in time...
                        if (self.moving_nominal_dest and self.path_timeout_elapsed):
                            rospy.loginfo(str(self.robot_id) + " stopping motion as not able to reach dest in time.")
                            success = True

                        # If trying to reconnect with teammate through backup plan...
                        elif (self.stop_when_comm and self.teammate_comm_regained):
                            rospy.loginfo(str(self.robot_id) + " stopping motion as communication was re-established and signal is enough")
                            success = True
                        else:
                            rospy.logerr(str(self.robot_id) + " navigation failed, using motion recovery")
                            self.clear_costmap_service()
                            self.motion_recovery()
                            self.clear_costmap_service()
                            success = False

                    elif state == GoalStatus.ABORTED:
                        if (self.path_timeout_elapsed and self.moving_nominal_dest):
                            rospy.loginfo(str(self.robot_id) + " stopping motion AFTER ABORTED as not able to reach dest in time.")
                            success = True

                        else:
                            # in multi2, when trying to reach the backup assume that will always succeed
                            rospy.logerr(str(self.robot_id) + " motion aborted by the server!!! Trying moving backward.")
                            if self.sim:
                                self.bump_bkw()
                            else:
                                self.clear_costmap_service()
                                self.motion_recovery()
                                self.clear_costmap_service()
                            success = False
                            # else:
                            #    return False
                            # Aborted: It cancelled

        if (self.n_robots > 4):
            self.poll_signal()

        return True

######################################################################################################################################################################

#How to move a robot randomly
class Random(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration,
                 disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename,
                 comm_dataset_filename, strategy, resize_factor, tiling, errors_filename):
        rospy.loginfo(str(robot_id) + ' - Random - starting!')

        if (not (os.path.exists(env_filename))):
            f = open(env_filename, "wb")
            self.env = Environment(map_filename, disc_method, disc, resize_factor, comm_range)
            pickle.dump(self.env, f)
            f.close()
        else:
            f = open(env_filename, "rb")
            self.env = pickle.load(f)
            f.close()

        super(Random, self).__init__(seed, robot_id, True, sim, comm_range, map_filename, polling_signal_period,
                                     duration, log_filename, comm_dataset_filename, teammates_id, n_robots,
                                     ref_dist, strategy, resize_factor, errors_filename)

        self.replan_rate = REPLAN_RATE
        self.arrived_to_random_pos = True

    def explore_comm_maps(self):
        r = rospy.Rate(self.replan_rate)
        while not rospy.is_shutdown():
            if (self.arrived_to_random_pos):
                self.arrived_to_random_pos = False
                new_dest = random.choice(self.env.free_positions)
                rospy.loginfo(str(self.robot_id) + ' chosen new dest: ' + str(new_dest))
                t = threading.Thread(target=self.send_to_light, args=(new_dest,))
                t.start()

            r.sleep()

    def send_to_light(self, target):
        rospy.loginfo(str(self.robot_id) + ' moving to ' + str(target))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.pose.position.x = target[0]
        goal.target_pose.pose.position.y = target[1]
        goal.target_pose.pose.orientation.w = 1
        self.client_motion.send_goal(goal, feedback_cb=self.feedback_motion_cb)
        self.client_motion.wait_for_result()
        state = self.client_motion.get_state()
        rospy.loginfo(str(self.robot_id) + ' stopped motion with state ' + str(state))
        if (state == GoalStatus.PREEMPTED):
            self.clear_costmap_service()
            self.motion_recovery()
            self.clear_costmap_service()
        elif (state == GoalStatus.ABORTED):
            if self.sim:
                self.bump_bkw()
            else:
                self.clear_costmap_service()
                self.motion_recovery()
                self.clear_costmap_service()

        self.arrived_to_random_pos = True

######################################################################################################################################################################

#How the leader manages the strategy and the followers
class Leader(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename, polling_signal_period,
                 duration, disc_method, disc, log_filename, teammates_id, n_robots, ref_dist,
                 env_filename, comm_dataset_filename, strategy, strategyParams, resize_factor, tiling, errors_filename,
                 communication_model):
        rospy.loginfo(str(robot_id) + ' - Leader - starting!')
        if communication_model is "":
            self.filter_locations = False
        else:
            self.filter_locations = True
        if (not (os.path.exists(env_filename))):
            print "Creating new environment."
            f = open(env_filename, "wb")
            self.env = Environment(map_filename, disc_method, disc, resize_factor, comm_range,
                                   communication_model)
            pickle.dump(self.env, f)
            f.close()
        else:
            f = open(env_filename, "rb")
            self.env = pickle.load(f)
            f.close()

        super(Leader, self).__init__(seed, robot_id, True, sim, comm_range, map_filename, polling_signal_period,
                                     duration, log_filename, comm_dataset_filename, teammates_id, n_robots,
                                     ref_dist, strategy, resize_factor, errors_filename)

        print 'created environment variable'
        rospy.loginfo(str(robot_id) + ' - Created environment variable')

        self.comm_map = GPmodel(self.env.dimX, self.env.dimY, comm_range, tiling, self.comm_module.comm_model,self.log_filename)
        self.comm_maps = []  # for logging

        # for sending commands to the follower
        # the name of the server is the name of the robot itself
        self.clients_signal = {}
        for teammate_id in teammates_id:
            self.clients_signal[teammate_id] = actionlib.SimpleActionClient(
                '/robot_' + str(teammate_id) + '/main_robot', SignalMappingAction)
            rospy.loginfo(str(self.robot_id) + ' - Leader - waiting for follower server ' + str(teammate_id))
            self.clients_signal[teammate_id].wait_for_server()
            rospy.loginfo(str(self.robot_id) + ' - Done.')

        self.strategyParams = strategyParams

        # old
        # self.all_signal_data = []
        # self.signal_data_follower = []

        #TODO EXPLORATION STRATEGY (DA METTERE self.exploration_strategy = exploration_strategies.my_strategy)
        """
        if (self.strategy == 'max_variance'):
            self.exploration_strategy = exploration_strategies.max_variance_strategy
        elif (self.strategy == 'multi2'):
            self.exploration_strategy = exploration_strategies.multi2_strategy
        """

        self.exploration_strategy = exploration_strategies.dora_strategy

        self.backup_strategy = exploration_strategies.backup_safe

        self.connect_first_attempt = False
        # dest = random.choice(self.env.free_positions)
        # self.send_follower_to((17.0,10.0), (self.x, self.y-2))

        self.replan_rate = REPLAN_RATE

        # -1: plan, 0: plan_set, 1: leader reached, 2: follower reached, 3: all reached. - for pair
        # -1: plan, 0: plan_set, 1-2 leaders/followers reached safe, 3: all paths sent, 4: completed - for multi2
        self.explore_comm_maps_state = -1

    def extract_signal_data(self):
        all_signal_data = []
        for data in self.robot_data_list:
            all_signal_data += data

        return all_signal_data

    #TODO MYSTRATEGY = dora_strategy
    def explore_comm_maps_dora(self):










    #TODO send_myself_to_dora(self,dest_leader)

    def send_myself_to_dora(self,dest_leader):


    #TODO def send_followers_to_MYSTRATEGY(self, plans, dest_leader): guarda main_robot 877

    def send_followers_to_dora(self,plans,dest_leader):


    def send_and_wait_goal(self, teammate_id, goal):


######################################################################################################################################################################

#How the follower behaves
class Follower(GenericRobot):
    # Feedback and result used to inform the leader about the gathered data
    _feedback = SignalMappingFeedback()
    _result = SignalMappingResult()

    def __init__(self, seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration, log_filename, comm_dataset_filename,
                 teammates_id, n_robots, ref_dist, env_filename, strategy, resize_factor, errors_filename):
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

        super(Follower, self).__init__(seed, robot_id, False, sim, comm_range, map_filename, polling_signal_period,
                                       duration, log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist,
                                       strategy, resize_factor, errors_filename)

        # for SignalMapping action commanded by the Leader
        self._action_name = rospy.get_name()

        #TODO TO MODIFY FOR MYSTRATEGY ###################  FATTO (riga sotto)
        """ 
        if (self.strategy != 'multi2'):
            self._as = actionlib.SimpleActionServer(self._action_name, SignalMappingAction,
                                                    execute_cb=self.execute_cb_pair, auto_start=False)
        else:
            self.first_safe_pos_add = False
            self._as = actionlib.SimpleActionServer(self._action_name, SignalMappingAction, execute_cb=self.execute_cb_multi2, auto_start=False)
        """

        self.as = actionlib.SimpleActionClient(self._action_name, SignalMappingAction, execute_cb = self.execute_cb_dora, auto_start=FALSE)

        print 'created environment variable'

        rospy.loginfo(str(robot_id) + ' - Follower - created environment variable!')

        self._as.start()

    #TODO  def execute_cb_MYSTRATEGY(self, goal): guarda main_robot 1182

    def execute_cb_dora(self,goal):
        rospy.loginfo(str(self.robot_id) + ' - Follower - has received a new goal.')
        self.reset_stuff()

        # use this trick to discriminate between first safe position and the final one
        self.first_safe_pos_add = (goal.double_goals[0].backup_leader.pose.position.x > 1.5)
        first_safe_pos_norm = (goal.double_goals[0].backup_leader.pose.position.x > 0.0 and
                               goal.double_goals[0].backup_leader.pose.position.x < 1.5)
        path_action = (goal.double_goals[0].backup_leader.pose.position.x < 0.0)

        rospy.loginfo(str(self.robot_id) + ' - Follower first safe_pos_add' + str(self.first_safe_pos_add))
        rospy.loginfo(str(self.robot_id) + ' - Follower first safe_pos_norm' + str(first_safe_pos_norm))
        rospy.loginfo(str(self.robot_id) + ' - Follower path action' + str(path_action))

        for destination in goal.double_goals:
            # each destination also includes the one of the leader - will be always the same in this strategy
            self.fill_cur_destinations((destination.target_leader.pose.position.x, destination.target_leader.pose.position.y),(-1.0, -1.0))
            success = self.send_to((destination.target_follower.pose.position.x, destination.target_follower.pose.position.y))
            # meanwhile, it will poll

        if (not (success)):
            rospy.loginfo(str(self.robot_id) + ' - Follower failed last point.')

        if (not (self.comm_module.can_communicate(self.teammates_id[0]))):
            if (path_action):
                rospy.loginfo(str(self.robot_id) + ' - Follower executing backup.')
                backup_dest = (goal.double_goals[-1].backup_follower.pose.position.x,
                               goal.double_goals[-1].backup_follower.pose.position.y)
                self.stop_when_comm = True
                self.send_to(backup_dest)
                # no need to fill cur destinations as follower destinations are ignored by leaders in this strategy

                # once it gets there, it will surely communicate - this is just to be safe
                if (not (self.comm_module.can_communicate(self.teammates_id[0]))):
                    backup_dest = (goal.double_goals[-1].target_leader.pose.position.x,goal.double_goals[-1].target_leader.pose.position.y)
                    self.send_to(backup_dest)

            elif (first_safe_pos_norm):
                while (not (self.comm_module.can_communicate(self.teammates_id[0]))):
                    rospy.loginfo(str(self.robot_id) + ' - Follower NORM waiting for the leader to communicate')
                    rospy.sleep(1)

            else:  # first_safe_pos_add -> should arrive here only when leader is also arrived!
                print "safe add has arrived, teammate_arrived_ value = " + str(self.teammate_arrived_nominal_dest)
                while (not (self.comm_module.can_communicate(self.teammates_id[0]))):
                    rospy.loginfo(str(self.robot_id) + ' - Follower ADD waiting for the leader to communicate')
                    rospy.sleep(1)

        self._result.data = []  # old self._feedback.data
        rospy.loginfo(str(self.robot_id) + ' - Follower - action succeded')

        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('robot')

    robot_id  = int(rospy.get_param('~id'))
    sim = int(rospy.get_param('/sim'))
    seed = int(rospy.get_param('/seed'))
    ref_dist = int(rospy.get_param('/ref_dist'))
    map_filename = rospy.get_param('/map_filename')
    disc = float(rospy.get_param('/disc'))
    comm_range = float(rospy.get_param('/range'))
    polling_signal_period = float(rospy.get_param('/polling_signal_period'))
    disc_method = rospy.get_param('/disc_method')
    duration = float(rospy.get_param('/duration'))
    env_filename = rospy.get_param('/env_filename')
    # Parameter for communication model to filter locations.
    communication_model = rospy.get_param('/communication_model', "")
    if communication_model is "":
        env_filename = env_filename.replace(".dat",
            "_" + str(int(comm_range)) + ".dat")
    else:
        env_filename = env_filename.replace(".dat",
            "_" + str(int(comm_range)) + '_' + communication_model + ".dat")
    print env_filename
    teammates_id_temp = str(rospy.get_param('~teammates_id'))
    is_leader = int(rospy.get_param('~is_leader'))
    n_robots = int(rospy.get_param('/n_robots'))
    resize_factor = float(rospy.get_param('/resize_factor'))
    tiling = int(rospy.get_param('/tiling'))
    log_folder = rospy.get_param('/log_folder')


    # strategies params
    strategy = rospy.get_param('/strategy')


    if (strategy == 'max_variance'):
        samples_pairs_greedy = int(rospy.get_param('/samples_pairs_greedy'))
        mindist_vertices_maxvar = int(rospy.get_param('/mindist_vertices_maxvar'))
        strategyParams = StrategyParams(samples_pairs_greedy=samples_pairs_greedy,
                                        mindist_vertices_maxvar=mindist_vertices_maxvar)
    elif('dora' in strategy):
        #TODO IMPLEMENTARE COSA FARE MYSTRATEGY
    elif(strategy==random):
        pass
    else:
        print "Strategy unknown! Exiting..."
        exit(1)


    temmates_id_temp = teammates_id_temp.split('-')
    teammates_id = map(lambda x: int(x), temmates_id_temp)

    env = (map_filename.split('/')[-1]).split('.')[0]
    if communication_model is "":
        log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + \
                       str(n_robots) + '_' + strategy + '_' + str(int(comm_range)) + '.log'
    else:
        log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + \
                       str(n_robots) + '_' + strategy + '_' + str(int(comm_range)) + '_' + communication_model + '.log'
    if communication_model is "":
        comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + \
                                '_' + str(n_robots) + '_' + strategy + '_' + str(int(comm_range)) + '.dat'
    else:
        comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + \
                                '_' + str(n_robots) + '_' + strategy + '_' + str(
            int(comm_range)) + '_' + communication_model + '.dat'

    errors_filename = log_folder + 'errors.log'
    print "Loggin possible errors to: " + errors_filename

    if strategy != "random":

        if((len(teammates_id) > 1) and not(is_leader)):
            print "Error! Only leader can have more than one teammate."
            exit(1)

        elif(len(teammates_id) > 1 and 'multi2' not in strategy):
            print "Error! Only strategy multi2 supports more than 1 teammate."
            exit(1)

        if is_leader:
            l = Leader(seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration,
                       disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename,
                       comm_dataset_filename, strategy, strategyParams, resize_factor, tiling, errors_filename,
                       communication_model)
            l.explore_comm_maps()
        else:
            f = Follower(seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration,
                         log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                         strategy, resize_factor, errors_filename)
            rospy.spin()
    else:
        r = Random(seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration, 
                   disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename, 
                   comm_dataset_filename, strategy, resize_factor, tiling, errors_filename)
        r.explore_comm_maps()
