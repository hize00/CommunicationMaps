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
gflags.DEFINE_string('output_name', 'output.txt', 'file containing the solution computed by the algorithm')
gflags.DEFINE_string('point_selection_policy', 'grid', 'policy for selecting points in an environment') #click,grid,voronoi

#for gflags
argv = gflags.FLAGS(sys.argv)

#-------------------------------------------------------------------
#---------------PARSING
POINTS = []
CONFIGURATION = []
R_MOVING = []
TIMETABLE = []

readingPoints = 0
readingConf = 0
readingRM = 0
readingTT = 0

#PARSE file .exp with original ID of the graph
#POINTS[0] is the original ID -for example 197- of node 0 of "output.txt", POINTS[1] is the original ID of node 1 of "output.txt", etc
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

#parsing output.txt
with open('../data/' + gflags.FLAGS.output_name, 'r') as f2:
	data = f2.readlines()
	for line in data:
		words = line.split()
		if len(words) > 0:
			if words[0] == "N_ROBOTS:":
				N_ROBOTS = int(words[1])

			elif words[0] == "CONFIGURATIONS:":
				readingConf = 1
				continue
			if readingConf == 1:
				minilistC = []
				if words[0] != ';':
					for i in range(0,N_ROBOTS):
						minilistC.append(int(words[i]))
					CONFIGURATION.append(minilistC)
				else:
					readingConf = 0

			elif words[0] == "ROBOT_MOVING:":
				readingRM = 1
				continue
			if readingRM == 1:
				minilistRM = []
				if words[0] != ';':
					for i in range(0,N_ROBOTS):
						minilistRM.append(int(words[i]))
					R_MOVING.append(minilistRM)
				else:
					readingRM = 0

			elif words[0] == "TIMETABLE:":
				readingTT = 1
				continue
			if readingTT == 1:
				minilistT = []
				if words[0] != ';':
					for i in range(0,N_ROBOTS):
						minilistT.append(float(words[i]))
					TIMETABLE.append(minilistT)
				else:
					readingTT = 0
f2.close()

print N_ROBOTS
print CONFIGURATION
print R_MOVING
print TIMETABLE
print POINTS

#-------------END PARSING
#--------------------------------------------------------------

#read the graphml file
G_E = Graph()
phys_graph_path = '../data/' + gflags.FLAGS.phys_graph
G_E = read(phys_graph_path, format='graphml')

# #get coordinates of points of graph
# for i in range(0,len(POINTS)):
# 	print "x_coord of node " + str(i) + ": " + str(G_E.vs[POINTS[i]]['x_coord'])
# 	print "y_coord of node " + str(i) + ": " + str(G_E.vs[POINTS[i]]['y_coord'])

COORDINATES_LIST = []

#TRASFORMA la sequenza dei punti delle configurazioni in sequenza di coordinate x e y
for i in range(0,len(CONFIGURATION)):
	minilistXY = []
	for j in range(0,N_ROBOTS):
		x = G_E.vs[POINTS[CONFIGURATION[i][j]]]['x_coord']
		#minilistXY.append(int(x))
		y = G_E.vs[POINTS[CONFIGURATION[i][j]]]['y_coord']
		minilistXY.append([int(x), int(y)])
	COORDINATES_LIST.append(minilistXY)

print COORDINATES_LIST

#writing solution plan.txt
env_name = (os.path.splitext(gflags.FLAGS.phys_graph)[0]).split("_")[0]
with open('solution_plan_' + str(N_ROBOTS) + '_robots' + '_' + env_name + '_' + gflags.FLAGS.point_selection_policy + '.txt', 'w') as f3:
	f3.write("N_ROBOTS: " + str(N_ROBOTS) + "\n;\n")
	
	#writing CONFIGURATIONS
	f3.write("\nCONFIGURATIONS:\n")
	for i in range (0,len(CONFIGURATION)):
		for j in range (0, N_ROBOTS):
			f3.write(str(CONFIGURATION[i][j]) + " ")
		f3.write("\n")
	f3.write(";\n")

	#writing COORDINATES_LIST
	f3.write("\nCOORDINATES_LIST:\n")
	for i in range (0,len(COORDINATES_LIST)):
		for j in range (0, N_ROBOTS):
			f3.write(str(COORDINATES_LIST[i][j][0]) + " " + str(COORDINATES_LIST[i][j][1]) + " | " )
		f3.write("\n")
	f3.write(";\n")

	# writing ROBOT_MOVING
	f3.write("\nROBOT_MOVING:\n")
	for i in range(0, len(R_MOVING)):
		for j in range(0, N_ROBOTS):
			f3.write(str(R_MOVING[i][j]) + " ")
		f3.write("\n")
	f3.write(";\n")
	
	#writing TIMETABLE
	f3.write("\nTIMETABLE:\n")
	for i in range (0,len(TIMETABLE)):
		for j in range (0, N_ROBOTS):
			f3.write(str(TIMETABLE[i][j]) + " ")
		f3.write("\n")
	f3.write(";\n")
f3.close()