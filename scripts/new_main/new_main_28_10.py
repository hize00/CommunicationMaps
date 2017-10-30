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
                         SignalMappingFeedback, SignalMappingResult, Plan
from strategy.srv import GetSignalData, GetSignalDataResponse

TIME_STUCK = 10.0
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

        # recovery
        self.last_feedback_pose = None
        self.stuck = False
        self.last_motion_time = None
        self.pub_motion_rec = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
        self.front_range = 0.0

        self.iam_moving = False

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
        self.moving_nominal_dest = False
        self.arrived_final_dest = False
        self.signal_strengths = []

        # (reduced) state publisher: 0 = not arrived to nominal dest, 1 = arrived to nominal dest
        self.pub_state = rospy.Publisher('expl_state', Bool, queue_size=10)

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

    def reset_stuff(self):
        self.arrived_nominal_dest = False
        self.teammate_arrived_nominal_dest = False
        self.last_feedback_pose = None
        self.last_motion_time = None
        self.moving_nominal_dest = False
        self.iam_moving = False

    def scan_callback(self, scan):
        min_index = int(math.ceil((MIN_SCAN_ANGLE_RAD_FRONT - scan.angle_min) / scan.angle_increment))
        max_index = int(math.floor((MAX_SCAN_ANGLE_RAD_FRONT - scan.angle_min) / scan.angle_increment))
        self.front_range = min(scan.ranges[min_index: max_index])

    def feedback_motion_cb(self, feedback):
        if (self.last_feedback_pose is not None and abs(
                    feedback.base_position.pose.position.x - self.last_feedback_pose[0]) <= 1e-3 and
                    abs(feedback.base_position.pose.position.y - self.last_feedback_pose[1]) <= 1e-3):
            if (rospy.Time.now() - self.last_motion_time) > rospy.Duration(TIME_STUCK):
                self.error_count += 1
                if self.error_count == MAX_NUM_ERRORS:
                    mode = 'a' if os.path.exists(self.errors_filename) else 'w'
                    f = open(self.errors_filename, mode)
                    f.write(str(self.seed) + "--->" + self.map_filename + "--->" + str(
                        self.n_robots) + "--->" + self.strategy_error_log + "\n")
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

        if self.stuck:
            rospy.loginfo(str(self.robot_id) + ' - STUCK, sending cancel goal.')
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
            msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.0))
            self.pub_motion_rec.publish(msg)
            rospy.sleep(rospy.Duration(0.2))
            self.clear_costmap_service()

    def go_to_pose(self, pos):
        rospy.loginfo(str(robot_id) + ' - moving to ' + str(pos))
        success = False

        while not success:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = '/map'
            goal.target_pose.pose.position = Point(pos[0], pos[1], 0.000)
            goal.target_pose.pose.orientation.w = 1
            self.last_feedback_pose = None
            self.last_motion_time = rospy.Time.now()

            # Start moving
            self.iam_moving = True
            self.client_motion.send_goal(goal, feedback_cb = self.feedback_motion_cb)
            self.iam_moving = False
            self.client_motion.wait_for_result()
            state = self.client_motion.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(str(robot_id) + ' - position reached ')
                self.arrived_nominal_dest = True
                success = True
            elif state == GoalStatus.PREEMPTED:
                rospy.loginfo(str(robot_id) + ' - preempted, using recovery')
                self.arrived_nominal_dest = False
                self.clear_costmap_service()
                self.motion_recovery()
                self.clear_costmap_service()
            else:
                break

        self.pub_state.publish(Bool(self.arrived_nominal_dest))

    def move_robot(self):
        if self.plans: #if plan exists (it is not an empty tuple)
            starting_position = True
            alone = False
            for plan in self.plans:
                self.reset_stuff()
                self.moving_nominal_dest = True

                if starting_position:
                    rospy.loginfo(str(self.robot_id) + ' - going to starting position')
                else: #I set my communication teammate (that changes according to the plan)
                    if self.is_leader:
                        self.teammates_id[0] = plan[1]
                    else:
                        self.teammates_id[0] = plan.comm_robot_id

                    if self.robot_id == self.teammates_id[0]: #if I am my own communication teammate
                        alone = True
                        rospy.loginfo(str(self.robot_id) + ' - going alone to final destination')
                    else: #I need to know where my teammate is
                        rospy.Subscriber('/robot_' + str(self.teammates_id[0]) + '/expl_state', Bool, self.state_callback)

                if self.is_leader:
                    self.go_to_pose(plan[0][0])
                else:
                    self.go_to_pose((plan.first_robot_dest.position.x,plan.first_robot_dest.position.y))

                r = rospy.Rate(0.5)
                while not self.teammate_arrived_nominal_dest and not starting_position and not alone:
                    rospy.loginfo(str(robot_id) + ' - waiting for my teammate ' + str(self.teammates_id[0]))
                    r.sleep()

                if not starting_position and not alone: #in starting position and alone robots have not a teammate
                    self.check_signal_strength()

                starting_position = False

        else:
            rospy.loginfo(str(self.robot_id) + ' - no plans to follow')

        self.execute_plan_state = 2

    def check_signal_strength(self):
        rospy.loginfo(str(self.robot_id) + ' - calculating signal strenght with teammate ' + str(self.teammates_id[0]))
        signal_strength = self.comm_module.get_signal_strength(self.teammates_id[0])
        self.signal_strengths.append(signal_strength)

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
                    self.send_plans_to_foll()
                else:
                    rospy.sleep(rospy.Duration(10))  # I have to wait while leader is sending goals to followers
                    self.execute_plan_state = 1
            elif self.execute_plan_state == 1:
                #follower has received plan, robots can move
                if self.is_leader:
                    self.plans = self.plans[self.robot_id] #leader plans are the ones at index self.robot_id
                self.move_robot()
            elif self.execute_plan_state == 2:
                #plan completed
                rospy.loginfo(str(robot_id) + ' - exploration completed! Shutting down...')
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
        rospy.loginfo(str(self.robot_id) + ' - Leader - planning')
        #self.parse_plans_file()

        self.plans = (((((14.0, 12.0), (14.0, 12.0)), 0), (((13.0, 15.0), (31.0, 13.0)), 1),(((14.0, 16.0), (25.0, 18.0)), 1)),
            ((((25.0,18.0), (25.0,18.0)), 1), (((31.0, 13.0), (13.0, 15.0)), 0),(((25.0, 18.0), (14.0, 16.0)), 0)))

        self.execute_plan_state = 0

    def parse_plans_file(self):
        coord = []
        robot_moving = []
        robot_plan = []

        reading_coords = 0
        reading_RM = 0

        with open('/home/andrea/catkin_ws/src/strategy/data/solution_plan_2_robots.txt', 'r') as file:
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

        file.close()

        # grouping coordinates by (x,y)
        coord = [coord[i:i + 2] for i in range(0, len(coord), 2)]  # group x and y of a single robot

        # converting from pixels to meters
        for c in coord:
            c[0] = float(self.env.dimX - resize_factor * c[0])
            c[1] = float(resize_factor * c[1])

        coord = [tuple(l) for l in coord]

        coord = [coord[i:i + N_ROBOTS] for i in range(0, len(coord), N_ROBOTS)]  # create a plan of coordinates

        # creating the plan
        count_config = 0
        for config in robot_moving:
            count = 0
            first_robot = -1
            for robot in config:
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

                    # assigning a reflected plan to the communication teammate
                    robot_plan.append(second_robot)
                    robot_plan.append(coord[count_config][position])
                    robot_plan.append(coord[count_config][first_robot])
                    robot_plan.append(first_robot)
                elif count_config == 0 and robot == 0:
                    #I add the starting positions of each robot to plan: [my_self,(starting_pose),(starting_pose),my_self]
                    my_self = count
                    robot_plan.append(my_self)
                    position = count
                    robot_plan.append(coord[count_config][position])
                    robot_plan.append(coord[count_config][position])
                    robot_plan.append(my_self)

                count += 1
            count_config += 1

        # grouping plan elements: [my_id, (my_coords),(teammate_coords),communication_teammate]
        robot_plan = [robot_plan[i:i + 4] for i in range(0, len(robot_plan), 4)]

        # deleting last (incomplete) plan if last robot_moving row has only one robot to move
        for plan in robot_plan:
            if len(plan) < 4:  # 4 = number of elements in a plan
                robot_plan.pop(-1)
                final_dest = plan

        final_dest.append(final_dest[1])
        final_dest.append(final_dest[0])

        robot_plan.append(final_dest) #adding to the complete plan the plan of the robot that has to go to final destination

        # grouping plan elements: [(my_id, (((my_coords), (teammate_coords)), communication_teammate)]
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
            my_id.append(tuple(msgs))
            plans.append(tuple(my_id))

        plans = tuple(plans)

        # grouping plan elements by robot_id
        robot_ids = set(map(lambda x: x[0], plans))
        plan_id = [[y[1] for y in plans if y[0] == x] for x in robot_ids]

        if len(robot_ids) < N_ROBOTS:
            for i in xrange(N_ROBOTS):
                if i not in robot_ids:
                    plan_id.insert(i, ())  # if a robot never moves, his plan will be empty

        plan_id = tuple([tuple(l) for l in plan_id])  # plans = (plan_robot_0, plan_robot_1,...,plan_robot_n)

        self.plans = plan_id

    def send_plans_to_foll(self):
        rospy.loginfo(str(robot_id) + ' - sending plans to other robots')
        clients_messages = []
        plan_index = 0
        for robots_plans in self.plans:
            for teammate_id in teammates_id:
                if teammate_id == plan_index:
                    for plan in robots_plans:
                        points = plan[0]
                        plans_follower = []

                        plan_follower = Plan()

                        #first_robot_dest
                        plan_follower.first_robot_dest = Pose()
                        plan_follower.first_robot_dest.position.x = points[0][0]
                        plan_follower.first_robot_dest.position.y = points[0][1]

                        #second_robot_dest
                        plan_follower.second_robot_dest = Pose()
                        plan_follower.second_robot_dest.position.x = points[1][0]
                        plan_follower.second_robot_dest.position.y = points[1][1]

                        #first_robot_id
                        plan_follower.comm_robot_id = Float32
                        plan_follower.comm_robot_id = plan[1]

                        #timestep
                        #plan_follower.timestep = Float32
                        #plan_follower.timestep = plan[2]
                        #print 'TIMESTEP: ' + str(plan_follower.timestep)

                        plans_follower.append(plan_follower)
                        goal = SignalMappingGoal(plans_follower=plans_follower)
                        clients_messages.append((teammate_id,goal))
            plan_index += 1

        goal_threads = []

        for (teammate_id, goal) in clients_messages:
            t = threading.Thread(target=self.send_and_wait_goal, args=(teammate_id, goal))
            rospy.sleep(rospy.Duration(1.0))
            t.start()
            goal_threads.append(t)

        for t in goal_threads:
            t.join()

        self.execute_plan_state = 1

    def send_and_wait_goal(self, teammate_id, goal):
        #rospy.loginfo(str(self.robot_id) + ' - Leader - sending a new goal for follower ' + str(teammate_id))
        self.clients_signal[teammate_id].send_goal(goal)

        self.clients_signal[teammate_id].wait_for_result()
        #rospy.loginfo(str(self.robot_id) + ' - Leader - has received the result of ' + str(teammate_id))




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