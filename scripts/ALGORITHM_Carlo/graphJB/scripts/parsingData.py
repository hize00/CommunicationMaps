import math
import random
import cv2
import numpy as np
from igraph import *
import matplotlib.pyplot as plt
import sys
import gflags

#CONSTANTS
gflags.DEFINE_string('exp_name', 'provaC', 'name of the file to be opened')
gflags.DEFINE_string('phys_graph', 'offices_phys_uniform_grid.graphml', 'file containing the physical graph')
gflags.DEFINE_integer('n_robots', 5, 'number of robots of the experiment')
VELOCITY = 1
RANGE_DISTANCE = 15
GRID_DISCRETIZATION = 11

#for gflags
argv = gflags.FLAGS(sys.argv)

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

#sort points read from f1
print POINTS
sortedPOINTS = sorted(POINTS)
print sortedPOINTS
print "\n"
for i in range(0, len(sortedPOINTS)):
	print "New ID of vertex " + str(sortedPOINTS[i]) + " is: " + str(i)
print "\n"



#compute distancese between points
G_E = Graph()
phys_graph_path = '../data/' + gflags.FLAGS.phys_graph
#read the graphml file
G_E = read(phys_graph_path, format='graphml')

#get coordinates of points of graph
for i in range(0,len(sortedPOINTS)):
	print "x_coord of node " + str(i) + ": " + str(G_E.vs[sortedPOINTS[i]]['x_coord'])
	print "y_coord of node " + str(i) + ": " + str(G_E.vs[sortedPOINTS[i]]['y_coord'])
dijsktra_list = G_E.shortest_paths_dijkstra(source=sortedPOINTS,target=sortedPOINTS)
print "Distance List:"
print dijsktra_list

#STARTING POS: random for now
START_POS = []
possible_values = list(range(0, len(sortedPOINTS)))
for i in range(0, gflags.FLAGS.n_robots):
	allowed_values = possible_values
	random_start_pos = random.choice(allowed_values)
	START_POS.append(random_start_pos)
	allowed_values.remove(random_start_pos)
print "Starting POS:"
print START_POS


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)/GRID_DISCRETIZATION

#fare PTExp: points of the graph which lies within RANGE_DISTANCE
POINTS_TO_EXPLORE = []
for i in range(0,len(sortedPOINTS)):
	for j in range(i+1,len(sortedPOINTS)):
		if euclidean_distance(G_E.vs[sortedPOINTS[i]]['x_coord'], G_E.vs[sortedPOINTS[i]]['y_coord'], G_E.vs[sortedPOINTS[j]]['x_coord'], G_E.vs[sortedPOINTS[j]]['y_coord']) <= RANGE_DISTANCE:
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
f2.write('VERTEXES = ' + str(len(sortedPOINTS)) + '\n' + ';' + '\n\n')
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