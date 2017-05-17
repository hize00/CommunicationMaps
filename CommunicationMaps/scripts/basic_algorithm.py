from copy import deepcopy
import math
import os
import pickle
import random
import sys
import threading

#import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Vector3, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Bool

#import communication
#from environment import Environment
#import exploration_strategies
#from GPmodel import GPmodel
#import utils
#from utils import conv_to_hash, eucl_dist
#from strategy.msg import GoalWithBackup, SignalData, RobotInfo, AllInfo, SignalMappingAction, \
                               #SignalMappingGoal, SignalMappingFeedback, SignalMappingResult
#from strategy.srv import GetSignalData, GetSignalDataResponse

N_ROBOTS=4  

map_file = "/home/andrea/CommunicationMaps/envs/tutorial.yaml"
world_file = "/home/andrea/CommunicationMaps/envs/2_offices.world"


"""
class Cell:
	def __init__(self, robot, vertex):
		self.robot = robot
		self.vertex = vertex

class Configuration:
	def __init__(self)
"""

