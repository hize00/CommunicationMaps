import math
import time
import numpy as np
import sys
import itertools
from operator import *
from igraph import *
from gurobipy import *

#start the time
start_time = time.time()

file_to_open = sys.argv[1]
obj_fun = str(sys.argv[2])
SORTIING = str(sys.argv[3])

# Parse argument
if len(sys.argv) < 3:
    print("Usage: python filename.py datafile.dat obj_fun('time' or 'distance') sorting_type('cardinaliy', 'heuristic' or 'objective')")
    exit(1)

#returns N_ROBOTS, N_VERTEX, ROBOT_VELOCITY, POINTS_TO_EXPLORE and DISTANCE_MATRIX
def parsing_file(datfile):
	P_EXPLORE = []
	START_POS = []
	readingstart=0
	readingpoints=0
	readingmatrix=0
	with open(file_to_open, "r") as f:
		data = f.readlines()
		for line in data:
			words = line.split()
			if len(words)>0:

				if words[0]=='N_ROBOTS':
					N_ROBOTS = int(words[2])
					continue

				elif words[0]=='START':
					readingstart = 1
					continue

				elif readingstart:
					minilist2 = []
					if words[0]!=';':
						START_POS.append(int(words[0]))
					else:
						readingstart = 0

				elif words[0]=='VERTEXES':
					N_VERT = int(words[2])
					continue

				elif words[0]=='VELOCITY':
					R_VELOCITY = int(words[2])
					continue
				
				elif words[0]=='POINTS_TO_EXPLORE':
					readingpoints = 1
					continue

				elif readingpoints:
						minilist = []
						if words[0]!=';':
							minilist.append(int(words[0]))
							minilist.append(int(words[1]))
							P_EXPLORE.append(minilist)
						else:
							readingpoints = 0

				elif words[0]=='DISTANCE_MATRIX':
					readingmatrix = 1
					distance_matrix =  np.zeros((N_VERT, N_VERT))
					continue

				elif readingmatrix==1:
						if words[0]!=';':
							distance_matrix[int(words[0])][int(words[1])] = int(words[2])
						else:
							readingmatrix = 0
				
		return N_ROBOTS, START_POS, N_VERT, R_VELOCITY, P_EXPLORE, distance_matrix

		
N_ROBOTS, STARTING_POS, N_VERTEXES, ROBOT_VELOCITY, POINTS_TO_EXPLORE, distance_matrix = parsing_file(file_to_open)
if len(STARTING_POS) != 3:
	exit(1)

#time matrix initialization
time_matrix =  np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,N_VERTEXES):
	for j in range(0,N_VERTEXES):
		time_matrix[i][j] = distance_matrix[i][j] / ROBOT_VELOCITY



#ROAD MATRIX. Being a completely connected graph, the 0 are only on the diagonal
adjacency_road = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range (0,N_VERTEXES):
	for j in range (0,N_VERTEXES):
		if j!=i:
			adjacency_road[i][j] = 1
		

n_to_explore = len(POINTS_TO_EXPLORE)
#RADIO MATRIX. Values are 1 for points where we need to measure connection
adjacency_radio = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,n_to_explore):
	adjacency_radio[POINTS_TO_EXPLORE[i][0]][POINTS_TO_EXPLORE[i][1]] = 1
	adjacency_radio[POINTS_TO_EXPLORE[i][1]][POINTS_TO_EXPLORE[i][0]] = 1

g = Graph()
g.add_vertices(N_VERTEXES)

EDGES = []
for i in range(0,N_VERTEXES-1):
	for j in range(i+1,N_VERTEXES):
		EDGES.append((i,j))

g.add_edges(EDGES)
gEdgeList = g.get_edgelist()

if obj_fun == "time":
	for i in range(0,len(gEdgeList)):
		g.es[i]["weight"] = time_matrix[gEdgeList[i][0]][gEdgeList[i][1]]
elif obj_fun == "distance":
	for i in range(0,len(gEdgeList)):
		g.es[i]["weight"] = distance_matrix[gEdgeList[i][0]][gEdgeList[i][1]]


#LIST of the shortest distances between vertexes
shortest_list = g.shortest_paths_dijkstra( weights="weight" )

#LIST to MATRIX conversion
shortest_matrix = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0, N_VERTEXES):
	for j in range(0, N_VERTEXES):
		shortest_matrix[i][j] = shortest_list[i][j]

# |---------------------------|
# |DATA AND PARAMETERS - END  |
# |---------------------------|


# |---------------------------|
# |ALGORITHM - START          |
# |---------------------------|

V3 = []
z = 0
for i in range(0, N_VERTEXES):
	for j in range(i, N_VERTEXES):
		for k in range(j, N_VERTEXES):
			if (i!=j) and (j!=k):
				if adjacency_radio[i][j]==1 or adjacency_radio[j][k]==1 or adjacency_radio[i][k]==1:
					V3.append([z, [i,j,k]])
					z = z+1
#accessing values of V3: [list index] | [vertex_id or pair] | [if pair, selects if first/second/third point]
#example1: V3[1][1][1] accesses 2nd element of list, pair of point, second point of the pair
#example2: V3[2][0] accesses 3rd element of list and its vertex_id.

#return minumum distance and ID COMBINATION of config2 after moving there from config1
def compute_distance(configuration, vertex3, i):
	values = []
	d1 = shortest_matrix[configuration[0]][vertex3[i][1][0]] + shortest_matrix[configuration[1]][vertex3[i][1][1]] + shortest_matrix[configuration[2]][vertex3[i][1][2]]
	values.append(d1)
	d2 = shortest_matrix[configuration[0]][vertex3[i][1][0]] + shortest_matrix[configuration[1]][vertex3[i][1][2]] + shortest_matrix[configuration[2]][vertex3[i][1][1]]
	values.append(d2)
	d3 = shortest_matrix[configuration[0]][vertex3[i][1][2]] + shortest_matrix[configuration[1]][vertex3[i][1][0]] + shortest_matrix[configuration[2]][vertex3[i][1][1]]
	values.append(d3)
	d4 = shortest_matrix[configuration[0]][vertex3[i][1][2]] + shortest_matrix[configuration[1]][vertex3[i][1][1]] + shortest_matrix[configuration[2]][vertex3[i][1][0]]
	values.append(d4)
	d5 = shortest_matrix[configuration[0]][vertex3[i][1][1]] + shortest_matrix[configuration[1]][vertex3[i][1][0]] + shortest_matrix[configuration[2]][vertex3[i][1][2]]
	values.append(d5)
	d6 = shortest_matrix[configuration[0]][vertex3[i][1][1]] + shortest_matrix[configuration[1]][vertex3[i][1][2]] + shortest_matrix[configuration[2]][vertex3[i][1][0]]
	values.append(d6)
	min_index, min_value = min(enumerate(values), key=operator.itemgetter(1))
	return min_index, min_value


#return minumum time and ID COMBINATION of config2 after moving there from config1
def compute_times(configuration, vertex3, i):
	values = []
	t1 = max(shortest_matrix[configuration[0]][vertex3[i][1][0]], shortest_matrix[configuration[1]][vertex3[i][1][1]], shortest_matrix[configuration[2]][vertex3[i][1][2]])
	values.append(t1)
	t2 = max(shortest_matrix[configuration[0]][vertex3[i][1][0]], shortest_matrix[configuration[1]][vertex3[i][1][2]], shortest_matrix[configuration[2]][vertex3[i][1][1]])
	values.append(t2)
	t3 = max(shortest_matrix[configuration[0]][vertex3[i][1][2]], shortest_matrix[configuration[1]][vertex3[i][1][0]], shortest_matrix[configuration[2]][vertex3[i][1][1]])
	values.append(t3)
	t4 = max(shortest_matrix[configuration[0]][vertex3[i][1][2]], shortest_matrix[configuration[1]][vertex3[i][1][1]], shortest_matrix[configuration[2]][vertex3[i][1][0]])
	values.append(t4)
	t5 = max(shortest_matrix[configuration[0]][vertex3[i][1][1]], shortest_matrix[configuration[1]][vertex3[i][1][0]], shortest_matrix[configuration[2]][vertex3[i][1][2]])
	values.append(t5)
	t6 = max(shortest_matrix[configuration[0]][vertex3[i][1][1]], shortest_matrix[configuration[1]][vertex3[i][1][2]], shortest_matrix[configuration[2]][vertex3[i][1][0]])
	values.append(t6)
	min_index, min_value = min(enumerate(values), key=operator.itemgetter(1))
	return min_index, min_value


#initialization
CONFIGURATIONS = []
CONFIGURATIONS.append(STARTING_POS)
TIMETABLE = []
TIMETABLE.append([0]*N_ROBOTS)
MEASURING_ROBOTS = []
MEASURING_ROBOTS.append([0]*N_ROBOTS)
MOVING_ROBOTS = []
MOVING_ROBOTS.append([0]*N_ROBOTS)
#POINTS_TO_EXPLORE already computed
EXPLORED_POINT = []

#ID COMBINATIONS: 0: 0,1,2 | 1: 0,2,1 | 2: 2,0,1 | 3: 2,1,0 | 4: 1,0,2 | 5:1,2,0
def combination(V3item, index):
	combinations = []
	combinations.append([V3item[index][1][0], V3item[index][1][1], V3item[index][1][2]])
	combinations.append([V3item[index][1][0], V3item[index][1][2], V3item[index][1][1]])
	combinations.append([V3item[index][1][2], V3item[index][1][0], V3item[index][1][1]])
	combinations.append([V3item[index][1][2], V3item[index][1][1], V3item[index][1][0]])
	combinations.append([V3item[index][1][1], V3item[index][1][0], V3item[index][1][2]])
	combinations.append([V3item[index][1][1], V3item[index][1][2], V3item[index][1][0]])
	return combinations

#returns points explored given a configuration
def exploring(configuration):
	explored = []
	for k in range(0, len(POINTS_TO_EXPLORE)):
		if set(POINTS_TO_EXPLORE[k]).issubset(configuration):
			explored.append(POINTS_TO_EXPLORE[k])
	return explored 

#returns the robots who moved from config1 to config2
def robots_moving(config1, config2):
	r_mov = [0]*len(config1)
	for i in range(0, len(config1)):
		if config1[i]!=config2[i]:
			r_mov[i]=1
	return r_mov

#returns the robots who measure the connection between points
def robots_measuring(configuration, p_explored):
	r_meas = [0]*len(configuration)
	for i in range(0,len(configuration)):
		for j in range(0,len(p_explored)):
			if (configuration[i]==p_explored[j][0]) or (configuration[i]==p_explored[j][1]):
				r_meas[i] = 1
				continue
	return r_meas

#compute the timetables for distance and time
#all parameters have the same length
def compute_timetable(config1, config2, r_measuring, r_moving, timetable):
	tt = deepcopy(timetable)
	t_measure = 0
	#update time/distance after moving robots
	for i in range(0,len(config1)):
		if r_moving[i]==1:
			tt[i] = tt[i] + shortest_matrix[config1[i]][config2[i]]

	if obj_fun == "time":
		#get the max time for robots who are measuring
		for i in range(0,len(config1)):
			if r_measuring[i]==1:
				if tt[i] > t_measure:
					t_measure = tt[i]
		#update the time for robots who are measuring
		for i in range(0,len(config1)):
			if r_measuring[i]==1:
				tt[i] = t_measure
		return tt, t_measure

	elif obj_fun == "distance":
		t_measure = max(tt)
		return tt, t_measure
	

#compute the possible moves from a configuration of V3
#return list with [[Cini] [C end] [timetable after move] [max T needed for the move] [points explored] [measuring robots] [moving robots]]
def compute_moves(configuration, timetable):
	moves = []

	if obj_fun == "time":
			#t_matrix = T[index_configuration_in_V3][i]
			for i in range(0,len(V3)):
				index_combination, t_matrix = compute_times(configuration, V3, i)
				config2_combinations = combination(V3, i)
				config2 = config2_combinations[index_combination]
				p_explored = exploring(config2)
				r_measuring = robots_measuring(config2, p_explored)
				r_moving = robots_moving(configuration, config2)
				timetable_move, maxT_move = compute_timetable(configuration, config2, r_measuring, r_moving, timetable)
				moves.append([configuration, config2, timetable_move, maxT_move, p_explored, r_measuring, r_moving])

	elif obj_fun == "distance":
			for i in range(0,len(V3)):
				index_combination, t_matrix = compute_distance(configuration, V3, i)
				config2_combinations = combination(V3, i)
				config2 = config2_combinations[index_combination]
				p_explored = exploring(config2)
				r_measuring = robots_measuring(config2, p_explored)
				r_moving = robots_moving(configuration, config2)
				timetable_move, maxT_move = compute_timetable(configuration, config2, r_measuring, r_moving, timetable)
				moves.append([configuration, config2, timetable_move, maxT_move, p_explored, r_measuring, r_moving])
	return moves


#sort by objective
def sort_objective(moveList):
	sort = []
	sort = sorted(moveList, key=lambda x: (x[3], max(x[2])))
	#filter the list to remove the elements that dont explore any point
	filtered = [item for item in sort if len(item[4]) > 0]
	return filtered

#sort first by cardinality of p_explored, then by time or distance
def sort_cardinality(moveList):
	sortPTE = []
	sortPTE = sorted(moveList, key=lambda x: (len(x[4]), -x[3], -max(x[2])), reverse=True)
	#filter the list to remove the elements that dont explore any point
	filtered = [item for item in sortPTE if len(item[4]) > 0]
	return filtered

def sort_heuristic(moveList):
	#sort and filter the list by cardinality
	sortPTE = sort_cardinality(moveList)
	sortH = []
	#sort by time/number points explored
	sortH = sorted(sortPTE, key=lambda x: (x[3]/len(x[4]), max(x[2])))
	return sortH

#choose greedy move after sorting 
def choose(moveList, sorting_type):
	S = []
	if sorting_type == "objective":
		S = sort_objective(moveList)
	elif sorting_type == "cardinality":
		S = sort_cardinality(moveList)
	elif sorting_type == "heuristic":
		S = sort_heuristic(moveList)
	chosen = S[0]
	#removing explored points from POINTS_TO_EXPLORE
	EXPLORED_POINT.append(chosen[4])
	for i in range(0,len(chosen[4])):
		POINTS_TO_EXPLORE.remove(chosen[4][i])
	return chosen



#FIRST MOVE
move_list = compute_moves(CONFIGURATIONS[-1], TIMETABLE[-1])
if SORTIING == "cardinality":
	mC1 = choose(move_list, "cardinality")
elif SORTIING == "heuristic":
	mC1 = choose(move_list, "heuristic")
elif SORTIING == "objective":
	mC1 = choose(move_list, "objective")

CONFIGURATIONS.append(mC1[1])
TIMETABLE.append(mC1[2])
MEASURING_ROBOTS.append(mC1[5])
MOVING_ROBOTS.append(mC1[6])

move_list = []

#check moves until we explored every point
while len(POINTS_TO_EXPLORE) > 0 :
	move_list = compute_moves(CONFIGURATIONS[-1], TIMETABLE[-1])
	if SORTIING == "cardinality":
		move_chosen = choose(move_list, "cardinality")
	elif SORTIING == "heuristic":
		move_chosen = choose(move_list, "heuristic")
	elif SORTIING == "objective":	
		move_chosen = choose(move_list, "objective")
	#append configuration of move_chosen
	CONFIGURATIONS.append(move_chosen[1])
	#append timetable of move_chosen
	TIMETABLE.append(move_chosen[2])
	#append measuring robots of move_chosen
	MEASURING_ROBOTS.append(move_chosen[5])
	#append moving robots of move_chosen
	MOVING_ROBOTS.append(move_chosen[6])

#LAST MOVE
tt_final = [0]*N_ROBOTS
r_moving_final = [0]*N_ROBOTS
r_measuring_final = [0]*N_ROBOTS

for i in range(0,N_ROBOTS):
	tt_final[i] = TIMETABLE[-1][i] + shortest_matrix[CONFIGURATIONS[-1][i]][STARTING_POS[i]]
	if CONFIGURATIONS[-1][i] != STARTING_POS[i]:
		r_moving_final[i] = 1

CONFIGURATIONS.append(STARTING_POS)
TIMETABLE.append(tt_final)
MEASURING_ROBOTS.append(r_measuring_final)
MOVING_ROBOTS.append(r_moving_final)


T_FINAL = max(TIMETABLE[-1])
print "DATFILE: " + str(file_to_open)
print "Kumar3 for " + str(obj_fun) + "/ " + SORTIING + " ended computation at: " + str(T_FINAL)
print("KUMAR 3 EXECUTION TIME: %s seconds\n" % (time.time() - start_time))
print "-------------------------------------------------------------------"