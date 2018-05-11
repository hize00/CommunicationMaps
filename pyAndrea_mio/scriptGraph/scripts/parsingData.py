#!/usr/bin/env python
import math
import random
import cv2
import numpy as np
from igraph import *
import matplotlib.pyplot as plt
import sys
sys.path.append('/home/andrea/catkin_ws/src/strategy/scripts')
import simulated_models, environment
from communication import CommModel, numObstaclesBetweenRobots

import gflags
import os
import yaml
import random
import time


#CONSTANTS
gflags.DEFINE_string('exp_name', 'provaC', 'name of the file to be opened')
gflags.DEFINE_string('phys_graph', 'offices_phys_uniform_grid.graphml', 'file containing the physical graph')
gflags.DEFINE_integer('n_robots', 4, 'number of robots of the experiment')

gflags.DEFINE_string("communication_model_path", "/home/andrea/catkin_ws/src/strategy/data/comm_model_50.xml",
    "Path to the XML file containing communication model parameters.")

VELOCITY = 1
RANGE_DISTANCE = 40
GRID_DISCRETIZATION = 11

#maximum recursion depth
sys.setrecursionlimit(100000)

def read_environment(environment_yaml_path):
    with open(environment_yaml_path, 'r') as environment_yaml_file:
        try:
            environment_yaml = yaml.load(environment_yaml_file)
            environment_image_path = os.path.join(os.path.dirname(environment_yaml_path),environment_yaml["image"])
            environment_image = cv2.imread(environment_image_path)
            if len(environment_image.shape) > 2:
                environment_image = cv2.cvtColor(environment_image,cv2.COLOR_BGR2GRAY)
            return environment_image
        except yaml.YAMLError as exc:
            print(exc)
            return None

def all_free(ii, jj, I, J, border=None):
	if(im_array[ii][jj] == 0): return False

	if border is None: return True

	for k in xrange(ii - border, ii + border + 1):
		if k < 0 or k >= I: return False
		for w in xrange(jj - border, jj + border + 1):
			if w < 0 or w >= J: return False
			if (im_array[k][w] == 0): return False

	return True

if __name__ == '__main__':

	os.chdir("/home/andrea/catkin_ws/src/strategy/")

	#for gflags
	argv = gflags.FLAGS(sys.argv)

	communication_model_path = gflags.FLAGS.communication_model_path
	comm_model = CommModel(communication_model_path)
	env_name = (os.path.splitext(gflags.FLAGS.phys_graph)[0]).split("_")[0]
	environment_yaml_path = os.getcwd() + '/envs/' + env_name + '.yaml'
	im_array = read_environment(environment_yaml_path)
	image_array = im_array

	resize_factor = 0.1
	dimX = np.size(im_array, 1) * resize_factor
	dimY = np.size(im_array, 0) * resize_factor
	I = np.size(im_array, 0)
	J = np.size(im_array, 1)

	os.chdir("/home/andrea/Desktop/pyAndrea_mio/scriptGraph/scripts")

	#PARSE file with points of the graph
	POINTS = []
	readingPoints = 0
	with open('../data/' + gflags.FLAGS.exp_name + '.exp', 'r') as f1:
		data = f1.readlines()
		for line in data:
			words = line.split()
			if len(words) > 0:
				if words[0] == "GRAPH_POINTS":
					readingPoints = 1
					continue
				if readingPoints == 1:
					if words[0] != ';':
						POINTS.append(int(words[0]))
					else:
						readingPoints = 0
	f1.close()	         		

	print POINTS
	print "\n"
	for i in range(0, len(POINTS)):
		print "New ID of vertex " + str(POINTS[i]) + " is: " + str(i)
	print "\n"

	#compute distancese between points
	G_E = Graph()
	phys_graph_path = '../data/' + gflags.FLAGS.phys_graph
	#read the graphml file
	G_E = read(phys_graph_path, format='graphml')

	for i in range(0,len(POINTS)):
		print "x_coord of node " + str(i) + ": " + str(G_E.vs[POINTS[i]]['x_coord'])
		print "y_coord of node " + str(i) + ": " + str(G_E.vs[POINTS[i]]['y_coord'])

	dijsktra_list = G_E.shortest_paths_dijkstra(source=POINTS,target=POINTS)
	print "Distance List:"
	print dijsktra_list

	#STARTING POS: random for now
	START_POS = []
	possible_values = list(range(0, len(POINTS)))
	for i in range(0, gflags.FLAGS.n_robots):
		allowed_values = possible_values
		random_start_pos = random.choice(allowed_values)
		START_POS.append(random_start_pos)
		allowed_values.remove(random_start_pos)
	print "Starting POS:"
	print START_POS

	def euclidean_distance(x1, y1, x2, y2):
		return math.sqrt((x1-x2)**2 + (y1-y2)**2)/GRID_DISCRETIZATION

	#coordinates conversion
	for i in range(0, len(POINTS)):
		coord = i
		G_E.vs[POINTS[i]]['x_coord'] = float(resize_factor * G_E.vs[POINTS[coord]]['x_coord'])
		G_E.vs[POINTS[i]]['y_coord'] = float(dimY - resize_factor * G_E.vs[POINTS[coord]]['y_coord'])
		#print "parsing: " + str(G_E.vs[POINTS[i]]['x_coord']),(G_E.vs[POINTS[i]]['y_coord'])

	#fare PTExp: points of the graph which lies within RANGE_DISTANCE
	POINTS_TO_EXPLORE = []
	for i in range(0,len(POINTS)):
		for j in range(i+1,len(POINTS)):
			if euclidean_distance(G_E.vs[POINTS[i]]['x_coord'], G_E.vs[POINTS[i]]['y_coord'],
								  G_E.vs[POINTS[j]]['x_coord'], G_E.vs[POINTS[j]]['y_coord']) <= RANGE_DISTANCE and \
					numObstaclesBetweenRobots(image_array, I, (G_E.vs[POINTS[i]]['x_coord'], G_E.vs[POINTS[i]]['y_coord']),
											  (G_E.vs[POINTS[j]]['x_coord'], G_E.vs[POINTS[j]]['y_coord']), resize_factor) != 0:
				POINTS_TO_EXPLORE.append([i , j])

	print "P to explore"
	print POINTS_TO_EXPLORE


	#FINAL FILE
	f2 = open('../data/' + gflags.FLAGS.exp_name + '_parsed' + '.dat', 'w')
	f2.write('INPUT IGRAPH FILE: phys_graph' + gflags.FLAGS.phys_graph + '\n\n')
	f2.write('RANGE_DISTANCE = ' + str(RANGE_DISTANCE) + '\n' + ';' + '\n\n')
	f2.write('GRID_DISCRETIZATION = ' + str(GRID_DISCRETIZATION) + '\n' + ';' + '\n\n')
	f2.write('N_ROBOTS = ' + str(gflags.FLAGS.n_robots) + '\n' + ';' + '\n\n')
	f2.write('START =\n')
	for i in range(0, len(START_POS)):
		f2.write(str(START_POS[i]) + '\n')
	f2.write(';' + '\n\n')
	f2.write('VERTEXES = ' + str(len(POINTS)) + '\n' + ';' + '\n\n')
	f2.write('VELOCITY = ' + str(VELOCITY) + '\n' + ';' + '\n\n')
	f2.write('POINTS_TO_EXPLORE =\n')
	for i in range(0,len(POINTS_TO_EXPLORE)):
		f2.write(str(POINTS_TO_EXPLORE[i][0]) + ' ' + str(POINTS_TO_EXPLORE[i][1]) + '\n')
	f2.write(';' + '\n\n')
	f2.write('DISTANCE_MATRIX =\n')
	# write on the file as matrix
	# for i in range(0,len(dijsktra_list)):
	# 	for j in range(0,len(dijsktra_list)):
	#  		f2.write(str(dijsktra_list[i][j]) + ' ')
	# 	f2.write("\n")

	# write on the file as pair of points with distances
	for i in range(0,len(dijsktra_list)):
		for j in range(0,len(dijsktra_list)):
			f2.write(str(i) + ' ' + str(j) + '  ' + str(dijsktra_list[i][j]) + '\n')
	f2.write(';')

	f2.close()

	print "\n\n.dat File successfully parsed and written"