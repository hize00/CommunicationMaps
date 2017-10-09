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
from geometry_msgs.msg import Point, Twist, Vector3, PoseWithCovarianceStamped, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool

import communication
from environment import Environment
import exploration_strategies
from GPmodel import GPmodel
import utils
from utils import conv_to_hash, eucl_dist
from strategy.msg import SignalData, RobotInfo, AllInfo, SignalMappingAction, SignalMappingGoal, \
                         SignalMappingFeedback, SignalMappingResult, GoalWithBackup, GoalDest, Plan
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

        # for logging
        self.log_filename = log_filename
        log_file = open(log_filename, "w")
        log_file.close()
        rospy.Timer(rospy.Duration(10), self.distance_logger)

        self.comm_dataset_filename = comm_dataset_filename
        log_dataset_file = open(comm_dataset_filename, "w")
        log_dataset_file.close()

        #estimated position
        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.1), self.tf_callback)
        self.robots_pos = [(0.0, 0.0) for _ in xrange(n_robots)]

        self.pub_my_pose = rospy.Publisher("/updated_pose", Point, queue_size=100)

        for i in xrange(n_robots):
            if i == robot_id: continue
            s = "def a_" + str(i) + "(self, msg): self.robots_pos[" + str(i) + "] = (msg.x, msg.y)"
            exec (s)
            exec ("setattr(GenericRobot, 'pos_teammate" + str(i) + "', a_" + str(i) + ")")
            exec ("rospy.Subscriber('/robot_" + str(i) + "/updated_pose', Point, self.pos_teammate" + str(i) + ", queue_size = 100)")

        #plan for navigation
        self.plans = []
        self.teammate_arrived_nominal_dest = False
        self.arrived_nominal_dest = False
        self.teammate_moving_nominal_dest = False

        # (reduced) state publisher: 0 = not arrived to nominal dest, 1 = arrived to nominal dest
        self.pub_state = rospy.Publisher('expl_state', Bool, queue_size=10)
        rospy.Subscriber('/robot_' + str(self.teammates_id[0]) + '/expl_state', Bool, self.state_callback)

        # teammate movement state publisher: 0 = not moving, 1 = moving
        self.pub_movement = rospy.Publisher('expl_movement_state', Bool, queue_size=10)
        rospy.Subscriber('/robot_' + str(self.teammates_id[0]) + '/expl_movement_state', Bool, self.movement_state_callback)

        # -1: plan, 0: plan_set, 1: leader-follower reached, 2: plan finished
        self.replan_rate = REPLAN_RATE
        self.execute_plan_state = -1


    def tf_callback(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', rospy.get_namespace() + 'base_link', rospy.Time(0))
            self.robots_pos[self.robot_id] = (trans[0],trans[1])
            pub_my_pose.publish(Point(trans[0],trans[1],0.0))
        except Exception as e:
            pass

    def distance_logger(self, event):
        f = open(self.log_filename, "a")
        f.write('D ' + str((rospy.Time.now() - self.mission_start_time).secs) + '\n') # + str(self.traveled_dist)

        f.close()

        if (rospy.Time.now() - self.mission_start_time) >= self.duration:
            rospy.loginfo("Sending shutdown...")
            os.system("pkill -f ros")

    def state_callback(self, msg):
        self.teammate_arrived_nominal_dest = msg.data

    def movement_state_callback(self, msg):
        self.teammate_moving_nominal_dest = msg.data

    def reset_stuff(self):
        self.arrived_nominal_dest = False
        self.teammate_moving_nominal_dest = False
        self.teammate_arrived_nominal_dest = False

    def go_to_pose(self, pos):
        rospy.loginfo(str(robot_id) + ' - moving to ' + str(pos))
        success = False
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(pos[0], pos[1], 0.000)
        goal.target_pose.pose.orientation.w = 1

        self.teammate_moving_nominal_dest = True
        self.pub_state.publish(Bool(self.arrived_nominal_dest))
        self.pub_movement.publish(Bool(self.teammate_moving_nominal_dest))

        # Start moving
        self.client_motion.send_goal(goal)

        success = self.client_motion.wait_for_result()
        state = self.client_motion.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo(str(robot_id) + ' - position reached ')
            self.arrived_nominal_dest = True
            self.pub_state.publish(Bool(self.arrived_nominal_dest))
            #self.teammate_moving_nominal_dest = False
            #self.pub_movement.publish(Bool(self.teammate_moving_nominal_dest))
        else:
            self.client_motion.cancel_goal()

    def move_robot(self):
        for plan in self.plans:
            self.reset_stuff()
            if self.is_leader:
                self.go_to_pose(plan[0][0])
                # self.go_to_pose((plan.first_robot_dest.position.x, plan.first_robot_dest.position.y))
            else:
                self.go_to_pose((plan.second_robot_dest.position.x,plan.second_robot_dest.position.y))

            r = rospy.Rate(1)
            while not self.teammate_arrived_nominal_dest:
                rospy.loginfo(str(robot_id) + ' - waiting for my teammate')
                r.sleep()


        while self.teammate_moving_nominal_dest:
            rospy.loginfo(str(robot_id) + ' - waiting for my teammate to reach the final destination')
            r.sleep

        rospy.sleep(rospy.Duration(5.0))
        self.execute_plan_state = 2



    # -1: plan, 0: plan_set
    # 1: leader/follower arrived and they have to wait for their teammate
    #2: completed
    def execute_plan(self):
        r = rospy.Rate(self.replan_rate)
        while not rospy.is_shutdown():
            if self.execute_plan_state == -1:
                #leader calculate plan
                if self.is_leader:
                    self.calculate_plan()
                else:
                    self.execute_plan_state = 0
            elif self.execute_plan_state == 0:
                #leader sends plans to follower
                if self.is_leader:
                    self.send_plans_to_foll(self.plans)
                else:
                    rospy.sleep(rospy.Duration(10.0))
                    self.execute_plan_state = 1
            elif self.execute_plan_state == 1:
                #follower has received plan, robots can move
                self.move_robot()
            elif self.execute_plan_state == 2:
                #plan completed
                rospy.loginfo('Exploration completed! Shutting down...')
                break

        r.sleep()




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

    def calculate_plan(self):
        self.plans = ((((11,12),(25,18)), self.teammates_id,10),(((13,15),(31,13)),self.teammates_id,12),
                     (((14,16),(25,18)),self.teammates_id,11))
        rospy.loginfo(str(self.robot_id) + ' - Leader - planning')

        self.execute_plan_state = 0

    def send_plans_to_foll(self,plans):
        clients_messages = []
        for plan in plans:
            points = plan[0]
            plans_follower = []

            plan_follower = Plan()

            #first_robot_dest
            plan_follower.first_robot_dest = Pose()
            plan_follower.first_robot_dest.position.x = points[0][0]
            plan_follower.first_robot_dest.position.y = points[0][1]
            #print 'LEADER_DEST: ' + str(plan_follower.leader_dest)

            #second_robot_dest
            plan_follower.second_robot_dest = Pose()
            plan_follower.second_robot_dest.position.x = points[1][0]
            plan_follower.second_robot_dest.position.y = points[1][1]
            #print 'FOLLOWER_DEST: ' + str(plan_follower.follower_dest)

            #teammate_id
            plan_follower.teammate_id = Float32
            plan_follower.teammate_id = plan[1][0]
            #print 'TEAMMATE_ID: ' + str(plan_follower.teammate_id)

            #timestep
            plan_follower.timestep = Float32
            plan_follower.timestep = plan[2]
            #print 'TIMESTEP: ' + str(plan_follower.timestep)

            plans_follower.append(plan_follower)
            goal = SignalMappingGoal(plans_follower=plans_follower)
            clients_messages.append((plan_follower.teammate_id,goal))

        goal_threads = []

        for (plan_follower.teammate_id, goal) in clients_messages:
            t = threading.Thread(target=self.send_and_wait_goal, args=(plan_follower.teammate_id, goal))
            rospy.sleep(rospy.Duration(2.0))
            t.start()
            goal_threads.append(t)

        for t in goal_threads:
            t.join()

        self.execute_plan_state = 1


    def send_and_wait_goal(self, teammate_id, goal):
        rospy.loginfo(str(self.robot_id) + ' - Leader - sending a new goal for follower ' + str(teammate_id))
        self.clients_signal[teammate_id].send_goal(goal)

        self.clients_signal[teammate_id].wait_for_result()
        rospy.loginfo(str(self.robot_id) + ' - Leader - has received the result of ' + str(teammate_id))




class Follower(GenericRobot):
    _result   = SignalMappingResult()

    def __init__(self, seed, robot_id, sim, comm_range, map_filename, duration,
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                 resize_factor, errors_filename):

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

        super(Follower, self).__init__(seed, robot_id, False, sim, comm_range, map_filename, duration,
                                       log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist,
                                       resize_factor, errors_filename)

        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(self._action_name, SignalMappingAction,
                                                execute_cb=self.execute_callback, auto_start=False)

        print 'created environment variable'

        rospy.loginfo(str(robot_id) + ' - Follower - created environment variable!')

        self._as.start()

    def execute_callback(self,goal):
        rospy.loginfo(str(self.robot_id) + ' - Follower - has received new goal ')

        for plan in goal.plans_follower:
            success = True
            self.plans.append(plan)

        if not success:
            rospy.loginfo(str(self.robot_id) + ' - Follower failed last point.')

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


    if is_leader:
        lead = Leader(seed, robot_id, sim, comm_range, map_filename, duration,
                      disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename,
                      comm_dataset_filename, resize_factor, tiling, errors_filename, communication_model)
        lead.execute_plan()

    else:
        foll = Follower(seed, robot_id, sim, comm_range, map_filename, duration, log_filename,
                        comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                        resize_factor, errors_filename)
        foll.execute_plan()
        rospy.spin()



