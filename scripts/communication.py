#!/usr/bin/env python
"""
  Communication module.
  
  To make the module work with a real WiFi adapter, aircrack-ng tools 
  are necessary.
  
  sudo apt-get install aircrack-ng
  
  Then, before using this, it is necessary to set the network interface
  in monitor mode:
  
  sudo airmon-ng wlan0
  
  Note that the module assumes that the network is ad-hoc.
"""

import csv # Process CSV file.
import math
import os # For path.
import time # Timer.
import xml.etree.ElementTree as ET

import numpy as np
import cv2

import rospkg # To get ROS paths.
import rospy
from nav_msgs.msg import Odometry

from wifi_node.msg import WifiStrength
import utils

TIMEOUT = 1.0 # Timeout (sec) before stopping waiting for the signal strength message.
PIXELS_PER_WALL = 4 # TODO parameter.
TH_STOP = 5

rospack = rospkg.RosPack()

# Path to file containing MAC addresses associated to robots.
MAC_TABLE_PATH = os.path.join(rospack.get_path('strategy'), 'data/mac_table_tplink.csv') # TODO parameter.

class CommModel():
    """Class for storing parameters related to communication model,
        reading from XML file.
    """
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        self.REF_SIGNAL = float(root.find("ref_signal").text)
        self.CUTOFF = float(root.find("cutoff").text)
        self.MAX_SIGNAL = float(root.find("max_signal").text)
        self.MAX_WALL = float(root.find("max_wall").text)
        self.CUTOFF_SAFE = float(root.find("cutoff_safe").text)
        self.VAR_NOISE = float(root.find("var_noise").text)
        self.PATH_LOSS = float(root.find("path_loss").text)
        self.WALL_ATT = float(root.find("wall_att").text)

        #USED BY EVALUATOR
        self.COMM_RANGE = float(root.find("comm_range").text)
        self.MIN_DIST = float(root.find("min_dist").text)
        self.REF_DIST = float(root.find("ref_dist").text)

        print "Created communication model instance with parameters:"
        print "REF_SIGNAL:", self.REF_SIGNAL
        print "CUTOFF:", self.CUTOFF
        print "MAX_SIGNAL:", self.MAX_SIGNAL
        print "MAX_WALL:", self.MAX_WALL
        print "CUTOFF_SAFE:", self.CUTOFF_SAFE
        print "VAR_NOISE:", self.VAR_NOISE
        print "PATH_LOSS:", self.PATH_LOSS
        print "WALL_ATT:", self.WALL_ATT
        print "COMM_RANGE:", self.COMM_RANGE
        print "MIN_DIST:", self.MIN_DIST
        print "REF_DIST:", self.REF_DIST
        
def numObstaclesBetweenRobots(im_array, I, my_pos, teammate_pos, 
    resize_factor):
    """Calculate number of obstacles between robots.

    A wall is considered to have an arbitrary number of pixels.

    Args:
        im_array (numpy.array): environment image.
        I (int): width in pixels.
        my_pos (tuple of float): x, y of robot making a query (meters).
        teammate_pos (tuple of float): x, y of other robot (meters).
        resize_factor (float): size of a cell (meter).
    Returns:
        counter (int): number of obstacles encountered.
    """
    x1 = I - int(my_pos[1]/resize_factor)
    y1 = int(my_pos[0]/resize_factor)

    x2 = I - int(teammate_pos[1]/resize_factor)
    y2 = int(teammate_pos[0]/resize_factor)

    # Bresenham algorithm.
    counter = 0
    angle = math.atan2(y2-y1, x2-x1) 

    # Setup initial conditions
    dx = x2 - x1
    dy = y2 - y1
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2: 
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
    # Iterate over bounding box generating points between start and end
    y = y1
    counter = 0
    for x in xrange(x1, x2 + 1):
        if is_steep:
            if(im_array[y, x] == 0): 
                    counter += 1
        else:
            if(im_array[x, y] == 0):
                counter += 1
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
        
    return int(counter/PIXELS_PER_WALL)
    
def mac_table(mac_table_path):
    """Read MAC table to be used to properly filter robot, MAC address.
    """
    with open(mac_table_path, 'r') as mac_table_file:
        # Open the CSV file.
        reader = csv.reader(mac_table_file) 
        
        # Skip the comment line.
        next(reader, None) 
        
        # Populate the dictionary mac_table key=robot_id,value=MAC_address.
        mac_table = {}
        for row in reader:
            mac_table[row[1]] = int(row[0])
        
        return mac_table
        

class CommunicationModule(object):
    def __init__(self, sim, seed, robot_id, n_robots, comm_range, ref_dist=None, map_filename=None, resize_factor=None):
        self.robot_id = robot_id
        self.n_robots = n_robots
        self.comm_range = comm_range
        self.ref_dist = ref_dist
        self.resize_factor = resize_factor

        self.comm_model = CommModel(rospy.get_param('/comm_model_filename'))

        if(map_filename is not None):
            im = cv2.imread(map_filename)    
            self.im_array = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
            self.I = int(np.size(self.im_array,0))
            self.J = int(np.size(self.im_array,1))

        if sim:
            np.random.seed(seed + robot_id)
            self.get_signal_strength = self.get_sim_signal_strength
            self.can_communicate = self.can_communicate_sim
            
            self.positions = [(0,0) for _ in xrange(n_robots)]

            for i in xrange(n_robots):
                s = "def a_" + str(i) + "(self, msg): self.positions[" + str(i) + "] = (msg.pose.pose.position.x, msg.pose.pose.position.y)"
                exec(s)
                exec("setattr(CommunicationModule, 'callback_pos_" + str(i) + "', a_" + str(i) +")")
                exec("rospy.Subscriber('/robot_" + str(i) + "/base_pose_ground_truth', Odometry, self.callback_pos_" + str(i) + ", queue_size = 100)")

        else:
            self.mac_table = mac_table(MAC_TABLE_PATH)
            self.get_signal_strength = self.get_real_signal_strength
            self.can_communicate = self.can_communicate_real

        #self.time_last_not_comm = None

    def get_sim_signal_strength(self, teammate_id, safe=True):
        dist = utils.eucl_dist(self.positions[self.robot_id], self.positions[teammate_id])

        num_obstacles = numObstaclesBetweenRobots(self.im_array, self.I, self.positions[self.robot_id], self.positions[teammate_id], self.resize_factor)
        num_obstacles = max(num_obstacles, numObstaclesBetweenRobots(self.im_array, self.I, self.positions[teammate_id], 
                                           self.positions[self.robot_id], self.resize_factor))

        #print num_obstacles
        
        nominal = self.comm_model.REF_SIGNAL - 10*self.comm_model.PATH_LOSS*math.log10(dist/self.ref_dist) - min(self.comm_model.MAX_WALL, num_obstacles)*self.comm_model.WALL_ATT
        
        if(safe):
            return nominal

        noise = np.random.normal(0, self.comm_model.VAR_NOISE)

        return max(self.comm_model.CUTOFF, nominal + noise)

    def get_real_signal_strength(self, teammate_id, safe=True): #safe here is not relevant
        message_from_teammate = False
        start_time = rospy.Time.now() # Timer for checking the elapsed time.

        while not message_from_teammate:
            try:
								
                wifi_strength_msg = rospy.wait_for_message('wifi_chatter', WifiStrength, TIMEOUT)
                if self.mac_table[wifi_strength_msg.src] == teammate_id:
                    message_from_teammate = True
            except:
                rospy.logerr("Wifi message not received.")
            
            if rospy.Time.now() - start_time > rospy.Duration(TIMEOUT):
                break

        if message_from_teammate:    
            return wifi_strength_msg.signal
        else:
            return self.comm_model.CUTOFF

    def can_communicate_sim(self, teammate_id, for_stopping=False):
        threshold = self.comm_model.CUTOFF_SAFE
        if(for_stopping):
            threshold += TH_STOP

        if(self.get_sim_signal_strength(teammate_id) > threshold):
            return True
        else:
            return False

    def can_communicate_real(self, teammate_id, for_stopping=True):
        #TODO CHANGE HERE WITH SERVICE!
        threshold = self.comm_model.CUTOFF_SAFE
        if(for_stopping):
            threshold += TH_STOP

        if(self.get_real_signal_strength(teammate_id) > threshold):
            return True
        else:
            return False
        
