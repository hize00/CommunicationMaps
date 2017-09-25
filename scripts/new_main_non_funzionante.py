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
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool

import communication
import exploration_strategies
from environment import Environment
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


class GenericRobot(object):
    def __init__(self, seed, robot_id, is_leader, sim, comm_range, map_filename, duration,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, resize_factor,
                 errors_filename, client_topic='move_base'):

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
        self.goal_sent = False
        self.client_motion = actionlib.SimpleActionClient(self.client_topic, MoveBaseAction)
        self.client_motion.wait_for_server()
        rospy.logdebug('initialized action exec')
        self.clear_costmap_service = rospy.ServiceProxy('move_base_node/clear_costmaps', Empty)

        # for communication
        self.comm_range = comm_range
        self.comm_module = communication.CommunicationModule(sim, seed, robot_id, n_robots, comm_range, ref_dist,
                                                             map_filename, resize_factor)

        # estimated position
        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.1), self.tf_callback)

        # for logging
        self.log_filename = log_filename
        log_file = open(log_filename, "w")
        log_file.close()
        rospy.Timer(rospy.Duration(10), self.distance_logger)

        self.comm_dataset_filename = comm_dataset_filename
        log_dataset_file = open(comm_dataset_filename, "w")
        log_dataset_file.close()

        # for estimating the path length
        rospy.Subscriber('move_base_node/NavfnROS/plan', Path, self.path_callback)
        self.pub_time_to_dest = rospy.Publisher('time_to_dest', Float32, queue_size=10)
        self.my_path = None
        self.my_time_to_dest = None
        rospy.Timer(rospy.Duration(1), self.info_teammate_callback)

        rospy.Subscriber('/robot_' + str(self.teammates_id[0]) + '/time_to_dest', Float32,self.teammate_time_path_callback)
        self.teammate_time_to_dest = None

        # to update path and time only at the begininning with the navfnros data
        self.first_estimate_time = False
        self.first_estimate_teammate_path = False
        self.path_inserted_in_info = False

        # start (joint) path time
        self.time_start_path = None

        # for the robotic pair "state machine"
        self.moving_nominal_dest = False
        self.arrived_nominal_dest = False
        self.teammate_arrived_nominal_dest = False
        self.path_timeout_elapsed = False
        self.completed = False

        # (reduced) state publisher: 0 = not arrived to nominal dest, 1 = arrived to nominal dest
        self.pub_state = rospy.Publisher('expl_state', Bool, queue_size=10)
        rospy.Subscriber('/robot_' + str(self.teammates_id[0]) + '/expl_state', Bool, self.state_callback)

        self.iam_moving = False

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

        rospy.Service('/robot_' + str(self.robot_id) + '/get_signal_data', GetSignalData, self.handle_get_signal_data)
        # subscribe to all the other robots - will never call mine
        self.get_signal_data_service_proxies = []
        for r in xrange(n_robots):
            service_name = '/robot_' + str(r) + '/get_signal_data'
            rospy.wait_for_service(service_name)
            self.get_signal_data_service_proxies.append(rospy.ServiceProxy(service_name, GetSignalData))


        self.pub_my_pose = rospy.Publisher("/updated_pose", Point, queue_size=100)

        self.robots_pos = [(0.0, 0.0) for _ in xrange(n_robots)]

        for i in xrange(n_robots):
            if i == robot_id: continue
            s = "def a_" + str(i) + "(self, msg): self.robots_pos[" + str(i) + "] = (msg.x, msg.y)"
            exec (s)
            exec ("setattr(GenericRobot, 'pos_teammate" + str(i) + "', a_" + str(i) + ")")
            exec ("rospy.Subscriber('/robot_" + str(i) + "/updated_pose', Point, self.pos_teammate" + str(i) + ", queue_size = 100)")

        self.lock_info = threading.Lock()

    def tf_callback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', rospy.get_namespace() + 'base_link', rospy.Time(0))
            self.robots_pos[self.robot_id] = (trans[0],trans[1])
            pub_my_pose.publish(Point(trans[0],trans[1],0))
        except Exception as e:
            pass

    def path_callback(self, msg):
        if (not (self.moving_nominal_dest)): return  # not needed

        # cannot update estimated time -
        if (self.first_estimate_time):
            self.my_path = self.extrapolate_waypoints(msg.poses)
            self.my_time_to_dest = self.compute_dist(msg.poses) / SPEED
            self.my_time_to_dest = self.my_time_to_dest + TIME_TOL_PERC * self.my_time_to_dest

            self.first_estimate_time = False
            rospy.loginfo('Time to dest robot ' + str(self.robot_id) + ' = ' + str(self.my_time_to_dest))

    def info_teammate_callback(self, event):
        if (self.moving_nominal_dest):
            self.pub_time_to_dest.publish(Float32(self.my_time_to_dest))

        self.pub_state.publish(Bool(self.arrived_nominal_dest))

    def teammate_time_path_callback(self, msg):
        if (not (self.moving_nominal_dest)): return

        self.teammate_time_to_dest = float(msg.data)

    def state_callback(self, msg):
        self.teammate_arrived_nominal_dest = msg.data

    def handle_get_signal_data(self, req):
        return GetSignalDataResponse(filter(lambda x: x.timestep > req.timestep, self.robot_data_list[req.robot_id]))

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


    def pub_all_info_callback(self, event):
        if (self.my_path is not None and self.teammate_path is not None and not (self.path_inserted_in_info)):
            self.robot_info_list[self.robot_id].path_leader = map(
                lambda x: Point(x.pose.position.x, x.pose.position.y, 0.0), self.my_path)
            self.robot_info_list[self.robot_id].path_follower = map(
                lambda x: Point(x.pose.position.x, x.pose.position.y, 0.0), self.teammate_path)
            self.robot_info_list[self.robot_id].timestep = int((rospy.Time.now() - self.mission_start_time).secs)
            self.robot_info_list[self.robot_id].timestep_path = int((rospy.Time.now() - self.mission_start_time).secs)
            rospy.loginfo(str(self.robot_id) + " inserting new paths in info to share")
            self.path_inserted_in_info = True

        all_info = AllInfo()
        all_info.all_info = self.robot_info_list
        all_info.sender = self.robot_id
        self.pub_all_info.publish(all_info)

    def update_all_info_callback(self, msg):
        if (self.sim and not (self.comm_module.can_communicate(msg.sender))):
            # rospy.loginfo(str(self.robot_id) + " cannot be updated with info received by " + str(msg.sender))
            return

        self.lock_info.acquire()
        for robot_info in msg.all_info:
            if robot_info.robot_id == -1 or robot_info.robot_id == self.robot_id:
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

    def fill_cur_destinations(self, dest_leader, dest_follower):
        if (self.robot_info_list[self.robot_id].robot_id == -1):
            self.robot_info_list[self.robot_id].robot_id = self.robot_id
            if self.my_path is None or self.teammate_path is None:
                # the follower will always have empty lists
                self.robot_info_list[self.robot_id].path_leader = []
                self.robot_info_list[self.robot_id].path_follower = []

        self.robot_info_list[self.robot_id].timestep = int((rospy.Time.now() - self.mission_start_time).secs)
        self.robot_info_list[self.robot_id].dest_leader = Point(position_lead[0], position_lead[1], 0)
        self.robot_info_list[self.robot_id].dest_follower = Point(position_foll[0], position_foll[1], 0)

    def distance_logger(self, event):
        f = open(self.log_filename, "a")
        f.write('D ' + str((rospy.Time.now() - self.mission_start_time).secs) + '\n') # + str(self.traveled_dist)

        f.close()

        if (rospy.Time.now() - self.mission_start_time) >= self.duration:
            rospy.loginfo("Sending shutdown...")
            os.system("pkill -f ros")

    def go_to_pose(self, pos):
        rospy.loginfo("Robot " + str(robot_id) + " goes to" +str(pos))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(pos[0], pos[1], 0.000)
        goal.target_pose.pose.orientation.w = 1

        # Start moving
        self.client_motion.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.client_motion.wait_for_result(rospy.Duration(60))
        state = self.client_motion.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            pass
        else:
            self.client_motion.cancel_goal()

    def send_to(self, target, timeout=0):
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
            self.client_motion.send_goal(goal)
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
                if state == GoalStatus.ABORTED:
                    rospy.logerr(str(self.robot_id) + " motion aborted by the server!!! ")
                    success = False

        return True



class Leader(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename,
                 duration, disc_method, disc, log_filename, teammates_id, n_robots, ref_dist,
                 env_filename, comm_dataset_filename, resize_factor, tiling, errors_filename,
                 communication_model):

        rospy.loginfo(str(robot_id) + ' - Leader - starting!')
        if communication_model is "":
            self.filter_locations = False
        else:
            self.filter_locations = True
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
                                     duration, log_filename, comm_dataset_filename, teammates_id, n_robots,
                                     ref_dist, resize_factor, errors_filename)

        print 'created environment variable'
        rospy.loginfo(str(robot_id) + ' - Created environment variable')


        self.comm_map = GPmodel(self.env.dimX, self.env.dimY, comm_range, tiling, self.comm_module.comm_model, self.log_filename)
        self.comm_maps = [] #for logging

        #for sending commands to the follower
        #the name of the server is the name of the robot itself
        self.clients_signal = {}
        for teammate_id in teammates_id:
            self.clients_signal[teammate_id] = actionlib.SimpleActionClient('/robot_' + str(teammate_id) + '/main_robot', SignalMappingAction)
            rospy.loginfo(str(self.robot_id) + ' - Leader - waiting for follower server ' + str(teammate_id))
            self.clients_signal[teammate_id].wait_for_server()
            rospy.loginfo(str(self.robot_id) + ' - Done.')

        self.replan_rate = REPLAN_RATE

        # -1: plan, 0: plan_set, 1: leader reached, 2: follower reached, 3: all reached. - for pair
        # -1: plan, 0: plan_set, 1-2 leaders/followers reached safe, 3: all paths sent, 4: completed - for multi2
        self.explore_comm_maps_state = -1

        self.exploration_strategy = exploration_strategies.random_strategy

    def explore_comm_maps_random(self):
        r = rospy.Rate(self.replan_rate)
        while not rospy.is_shutdown():
            new_dest = random.choice(self.env.free_positions)
            rospy.loginfo(str(self.robot_id) + ' chosen new dest: ' + str(new_dest))
            t = threading.Thread(target=self.send_to, args=(new_dest,))
            t.start()

        r.sleep()

    def explore_comm_maps_multi2(self):
        r = rospy.Rate(self.replan_rate)
        while not rospy.is_shutdown():
            if self.explore_comm_maps_state == -1:
                self.reset_stuff()

                rospy.loginfo(str(self.robot_id) + ' planning')

                dest_leader = None
                while (dest_leader is None):
                    self.lock_info.acquire()
                    robot_info_list_copy = deepcopy(self.robot_info_list)
                    self.lock_info.release()
                    self.fill_cur_destinations((self.robots_pos[self.robot_id][0],self.robots_pos[self.robot_id][1]), (
                    -999999.0, -999999.0))  # this value tells the other leaders to wait planning.
                    dest_leader, dest_followers_start, paths_followers = self.exploration_strategy((self.robots_pos[self.robot_id][0],self.robots_pos[self.robot_id][1]),
                                                                                                   self.other_robots_pos,
                                                                                                   self.env,
                                                                                                   self.comm_map,
                                                                                                   robot_info_list_copy,
                                                                                                   self.robot_id,
                                                                                                   self.teammates_id,
                                                                                                   self.strategyParams,
                                                                                                   self.filter_locations)

                    rospy.sleep(rospy.Duration(8))

                rospy.loginfo(str(self.robot_id) + ' - Leader - has decided its dest:')
                rospy.loginfo(dest_leader)

                rospy.loginfo(str(self.robot_id) + ' - Leader - has decided followers start vertices:')
                rospy.loginfo(dest_followers_start)

                rospy.loginfo(str(self.robot_id) + ' - Leader - has decided followers paths:')
                rospy.loginfo(paths_followers)

                self.fill_cur_destinations(dest_leader, (-1.0, -1.0))  # followers are not considered in this strategy
                rospy.sleep(rospy.Duration(2.0))
                self.explore_comm_maps_state = 0
                t1 = threading.Thread(target=self.send_myself_to_multi2, args=(dest_leader,))
                t1.start()
                t2 = threading.Thread(target=self.send_followers_to_multi2, args=(dest_followers_start, dest_leader))
                t2.start()

            elif self.explore_comm_maps_state == 2:
                # all are arrived, can now send paths
                self.explore_comm_maps_state = 3
                t3 = threading.Thread(target=self.send_followers_to_multi2, args=(paths_followers, dest_leader))
                t3.start()

            elif self.explore_comm_maps_state == 4:
                # be sure to receive the reconnection signal data (in case of disconnection)
                rospy.sleep(rospy.Duration(2))

                all_signal_data = self.extract_signal_data()
                print str(self.robot_id) + ' - creating new model with a total of ' + str(len(all_signal_data))
                self.comm_map.update_model(all_signal_data)

                self.log_plan()

                self.explore_comm_maps_state = -1

            r.sleep()

    def send_myself_to_pos(self, dest_leader):
        self.send_to(dest_leader)
        self.explore_comm_maps_state += 1
        rospy.loginfo(str(self.robot_id) + ' - Leader - has arrived to its destination.')

    def send_followers_to_pos(self, plans, dest_leader):
        clients_messages = []
        for plan in plans:
            print "plan:"
            print plan
            teammate_id = plan[0]
            points = plan[1]
            backup = plan[2]
            double_goals = []
            print points
            for point in points:
                double_goal = GoalWithBackup()
                double_goal.target_follower = PoseStamped()
                double_goal.target_follower.pose.position.x = point[0]
                double_goal.target_follower.pose.position.y = point[1]

                double_goal.target_leader = PoseStamped()
                double_goal.target_leader.pose.position.x = dest_leader[0]
                double_goal.target_leader.pose.position.y = dest_leader[1]

                double_goals.append(double_goal)

            goal = SignalMappingGoal(double_goals=double_goals)
            clients_messages.append((teammate_id, goal))

        goal_threads = []

        for (teammate_id, goal) in clients_messages:
            t = threading.Thread(target=self.send_and_wait_goal, args=(teammate_id, goal))
            t.start()
            goal_threads.append(t)

        for t in goal_threads:
            t.join()

        self.explore_comm_maps_state += 1
class Follower(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename, duration,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                 resize_factor, errors_filename):

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

        super(Follower, self).__init__(seed, robot_id, False, sim, comm_range, map_filename, duration,
                                       log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist,
                                       resize_factor, errors_filename)

        self._action_name = rospy.get_name()

        self._as = actionlib.SimpleActionServer(self._action_name, SignalMappingAction, auto_start=False) #execute_cb=self.execute_cb_dora,

        print 'created environment variable'

        rospy.loginfo(str(robot_id) + ' - Follower - created environment variable!')

        self._as.start()

    def execute_cb_multi2(self, goal):
        rospy.loginfo(str(self.robot_id) + ' - Follower - has received a new goal.')
        self.reset_stuff()

        rospy.loginfo(str(self.robot_id) + ' - Follower path action' + str(path_action))

        for destination in goal.double_goals:
            # each destination also includes the one of the leader - will be always the same in this strategy
            self.fill_cur_destinations(
                (destination.target_leader.pose.position.x, destination.target_leader.pose.position.y),
                (-1.0, -1.0))
            success = self.send_to((destination.target_follower.pose.position.x, destination.target_follower.pose.position.y))
            # meanwhile, it will poll

        if (not (success)):
            rospy.loginfo(str(self.robot_id) + ' - Follower failed last point.')

        if (not (self.comm_module.can_communicate(self.teammates_id[0]))):
            if (first_safe_pos_norm):
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

    robot_id = int(rospy.get_param('~id'))
    sim = int(rospy.get_param('/sim'))
    seed = int(rospy.get_param('/seed'))
    ref_dist = int(rospy.get_param('/ref_dist'))
    map_filename = rospy.get_param('/map_filename')
    disc = float(rospy.get_param('/disc'))
    comm_range = float(rospy.get_param('/range'))
    disc_method = rospy.get_param('/disc_method')
    duration = float(rospy.get_param('/duration'))
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

    temmates_id_temp = teammates_id_temp.split('-')
    teammates_id = map(lambda x: int(x), temmates_id_temp)

    env = (map_filename.split('/')[-1]).split('.')[0]
    if communication_model is "":
        log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + \
                       str(n_robots) + '_' + str(int(comm_range)) + '.log'
    else:
        log_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + '_' + \
                       str(n_robots) + '_' + str(int(comm_range)) + '_' + communication_model + '.log'
    if communication_model is "":
        comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + \
                                '_' + str(n_robots) + '_' + str(int(comm_range)) + '.dat'
    else:
        comm_dataset_filename = log_folder + str(seed) + '_' + env + '_' + str(robot_id) + \
                                '_' + str(n_robots) + '_' + str(int(comm_range)) + '_' + communication_model + '.dat'

    errors_filename = log_folder + 'errors.log'
    print "Logging possible errors to: " + errors_filename

    position_lead = [11.50,12.50]
    position_foll = [31,13]

    if is_leader:
        lead = Leader(seed, robot_id, sim, comm_range, map_filename, duration,
                      disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename,
                      comm_dataset_filename, resize_factor, tiling, errors_filename, communication_model)
        #lead.go_to_pose(position)
        lead.explore_comm_maps()

    else:
        foll = Follower(seed, robot_id, sim, comm_range, map_filename, duration, log_filename,
                        comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                        resize_factor, errors_filename)
        #foll.go_to_pose(position)
        rospy.spin()


