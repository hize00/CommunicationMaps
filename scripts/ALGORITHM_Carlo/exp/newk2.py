import math
import time
import numpy as np
import sys
import itertools
from igraph import *
from gurobipy import *

sys.setrecursionlimit(1000000)
#start the time
start_time = time.time()

file_to_open = sys.argv[1]
obj_fun = str(sys.argv[2])

# Parse argument
if len(sys.argv) < 3:
    print("Usage: python filename.py datafile.dat obj_fun('time' or 'distance')")
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
if len(STARTING_POS)>2:
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

V2 = []
z=0
for i in range (0,N_VERTEXES):
	for j in range (i,N_VERTEXES):
		if adjacency_radio[i][j] == 1:
			V2.append((z,[i,j]))
			z=z+1
#adding START_POS
V2.append((z, [STARTING_POS[0], STARTING_POS[1]]))
#accessing values of V2: [list index] | [vertex_id or pair] | [if pair, selects if first or second point]
#example1: V2[1][1][1] accesses 2nd element of list, pair of point, second point of the pair
#example2: V2[2][0] accesses 3rd element of list and vertex_id.
length = len(V2)

"""DISTANCE COST MATRIX"""
def distance_cost(adjacency_road, adjacency_radio):
	A2 = np.zeros((length, length))
	D2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				#the cost is the MIN between d(ii')+d(jj') OR d(ij')+d(ji')
				D2[i][j]=min( (shortest_matrix[V2[i][1][0]][V2[j][1][0]] + shortest_matrix[V2[i][1][1]][V2[j][1][1]]) , (shortest_matrix[V2[i][1][0]][V2[j][1][1]]+shortest_matrix[V2[i][1][1]][V2[j][1][0]]))
	#symmetric wrt diagonal				
	return D2

"""TIME COST MATRIX"""
def time_cost(adjacency_road, adjacency_radio):
	A2 = np.zeros((length, length))
	T2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]: #control only on different nodes
				A2[i][j]=1
				#the cost is the MIN between the MAX of t(ii')+t(jj') OR t(ij')+t(ji') -- OK
				T2[i][j]=min(max(shortest_matrix[V2[i][1][0]][V2[j][1][0]], shortest_matrix[V2[i][1][1]][V2[j][1][1]]) , max(shortest_matrix[V2[i][1][0]][V2[j][1][1]],shortest_matrix[V2[i][1][1]][V2[j][1][0]]))
	#symmetric wrt diagonal				
	return T2


""" GUROBI TSP"""

# Callback - use lazy constraints to eliminate sub-tours
def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = tuplelist((i,j) for i,j in model._vars.keys() if vals[i,j] > 0.5)
        # find the shortest cycle in the selected edge list
        tour = subtour(selected)
        if len(tour) < n:
            # add subtour elimination constraint for every pair of cities in tour
            model.cbLazy(quicksum(model._vars[i,j]
                                  for i,j in itertools.combinations(tour, 2))
                         <= len(tour)-1)


# Given a tuplelist of edges, find the shortest subtour
def subtour(edges):
    unvisited = list(range(n))
    cycle = range(n+1) # initial length has 1 more city
    while unvisited: # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i,j in edges.select(current,'*') if j in unvisited]
        if len(cycle) > len(thiscycle):
            cycle = thiscycle
    return cycle


# Cost matrixes (distance/time) between each pair of points
D = distance_cost(adjacency_road,adjacency_radio)
T = time_cost(adjacency_road,adjacency_radio)
matrix_length = int(math.sqrt(T.size))

#Dictionaries (distance/time)
dict_dist = {}
dict_time = {}

dict_time = {(i,j) : T[i][j] 
    for i in range(matrix_length) for j in range(i)}

dict_dist = {(i,j) : D[i][j] 
    for i in range(matrix_length) for j in range(i)}

m = Model()

if obj_fun == "time":
	vars = m.addVars(dict_time.keys(), obj=dict_time, vtype=GRB.BINARY, name='e')
	for i,j in vars.keys():
		vars[j,i] = vars[i,j] # edge in opposite direction
elif obj_fun == "distance":
	vars = m.addVars(dict_dist.keys(), obj=dict_dist, vtype=GRB.BINARY, name='e')
	for i,j in vars.keys():
		vars[j,i] = vars[i,j] # edge in opposite direction
else: 
	print "Error in calling the file. Usage: file.py datafile.dat obj_fun('time' or 'distance')"
	sys.exit()

n = len(POINTS_TO_EXPLORE)+1
# Add degree-2 constraint
m.addConstrs(vars.sum(i,'*') == 2 for i in range(n))

# Optimize model
m._vars = vars
#dont print information
m.Params.LogToConsole = 0
m.Params.OutputFlag = 0

m.Params.lazyConstraints = 1
m.optimize(subtourelim)

vals = m.getAttr('x', vars)
selected = tuplelist((i,j) for i,j in vals.keys() if vals[i,j] > 0.5)

tour = subtour(selected)
assert len(tour) == n


#accessing values of V2: [list index] | [vertex_id or pair] | [if pair, selects if first or second point]
#example1: V2[1][1][1] accesses 2nd element of list, pair of point, second point of the pair
#example2: V2[2][0] accesses 3rd element of list and vertex_id.
tour_list = []
tour_list.append([ V2[tour[0]][1][0], V2[tour[0]][1][1] ])

if obj_fun == "distance":
	t = 0
	for i in range(1,len(tour)):
		t1 = D[tour[i-1]][tour[i]] 
		t = t + t1
		tour_list.append([ V2[tour[i]][1][0], V2[tour[i]][1][1] ])

#transforming the tour planned by GUROBI into a succession of points in tour_list (not in order of robots)
elif obj_fun == "time":
	t = 0
	for i in range(1,len(tour)):
		t1 = T[tour[i-1]][tour[i]] 
		t = t + t1
		tour_list.append([ V2[tour[i]][1][0], V2[tour[i]][1][1] ])

#getting tour_index of STARTING_POS
for i in range(0,len(tour_list)):
	if tour_list[i][0]==STARTING_POS[0] and tour_list[i][1]==STARTING_POS[1]:
		fAp_index = i 


#list to order the tour points to visit
CONFIGURATIONS = []
CONFIGURATIONS.append([STARTING_POS[0], STARTING_POS[1]])

if obj_fun == "time":
	#check that fAp_index is not the last element, otherwise starts from beginning
	if fAp_index < len(tour_list)-1:
	#second half of the tour_list: from [fAp_index] to end
		#the element [fAp_index] is STARTING_POS, therefore the loop starts at [fAp_index+1]
		for i in range(fAp_index,len(tour_list)-1):
			if max(shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][0]] , shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][1]]) <= max(shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][1]] , shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][0]]):
				CONFIGURATIONS.append([tour_list[i+1][0] , tour_list[i+1][1]])
			else:
				CONFIGURATIONS.append([tour_list[i+1][1] , tour_list[i+1][0]])
	
	#check between last element and first element of tour
	if max(shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[0][0]] , shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[0][1]]) <= max(shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[0][1]] , shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[0][0]]):
			CONFIGURATIONS.append([tour_list[0][0] , tour_list[0][1]])
	else:
			CONFIGURATIONS.append([tour_list[0][1] , tour_list[0][0]])

	#first half of the tour_list: from 0 to [fAp_index-1]
	for i in range(0,fAp_index-1):
		if max(shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][0]] , shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][1]]) <= max(shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][1]] , shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][0]]):
			CONFIGURATIONS.append([tour_list[i+1][0] , tour_list[i+1][1]])
		else:
			CONFIGURATIONS.append([tour_list[i+1][1] , tour_list[i+1][0]])

elif obj_fun == "distance":
	#check that fAp_index is not the last element
	if fAp_index < len(tour_list)-1:
	#second half of the tour_list: from [fAp_index] to end
		for i in range(fAp_index,len(tour_list)-1):
			if shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][0]] + shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][1]] <= shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][1]] + shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][0]]:
				CONFIGURATIONS.append([tour_list[i+1][0] , tour_list[i+1][1]])
			else:
				CONFIGURATIONS.append([tour_list[i+1][1] , tour_list[i+1][0]])
	
	#check between last element and first element of tour
	if shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[0][0]] + shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[0][1]] <= shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[0][1]] + shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[0][0]]:
			CONFIGURATIONS.append([tour_list[0][0] , tour_list[0][1]])
	else:
			CONFIGURATIONS.append([tour_list[0][1] , tour_list[0][0]])

	#first half of the tour_list: from 0 to [fAp_index]
	for i in range(0,fAp_index-1):
		if shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][0]] + shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][1]] <= shortest_matrix[CONFIGURATIONS[-1][0]][tour_list[i+1][1]] + shortest_matrix[CONFIGURATIONS[-1][1]][tour_list[i+1][0]]:
			CONFIGURATIONS.append([tour_list[i+1][0] , tour_list[i+1][1]])
		else:
			CONFIGURATIONS.append([tour_list[i+1][1] , tour_list[i+1][0]])


CONFIGURATIONS.append(STARTING_POS)


TIMETABLE = []
TIMETABLE.append([0]*N_ROBOTS)
tt = [0]*N_ROBOTS
ttab = [0]*N_ROBOTS
if obj_fun == "time":
	for i in range(0,len(CONFIGURATIONS)-1):
		t_bottleneck = max(shortest_matrix[CONFIGURATIONS[i][0]][CONFIGURATIONS[i+1][0]], shortest_matrix[CONFIGURATIONS[i][1]][CONFIGURATIONS[i+1][1]])
		ttab[0] = TIMETABLE[-1][0] + t_bottleneck
		ttab[1] = TIMETABLE[-1][1] + t_bottleneck
		tt = deepcopy(ttab)
		TIMETABLE.append(tt)
	
elif obj_fun == "distance":
	for i in range(0,len(CONFIGURATIONS)-1):
		ttab[0] = TIMETABLE[-1][0] + shortest_matrix[CONFIGURATIONS[i][0]][CONFIGURATIONS[i+1][0]]
		ttab[1] = TIMETABLE[-1][1] + shortest_matrix[CONFIGURATIONS[i][1]][CONFIGURATIONS[i+1][1]]
		tt = deepcopy(ttab)
		TIMETABLE.append(tt)

print "DATFILE : " + str(file_to_open)

env = file_to_open[0:6]
print "ENVIRONMENT : " + str(env)

print "ALGORITHM : KUMAR2"

if file_to_open[14] == "_":
	RANGE = file_to_open[11:14] #se 100 [11:15] se 1000
else:
	RANGE = RANGE = file_to_open[11:15]
print "RANGE : " + str(RANGE)

print "STARTING_POS : " + str(STARTING_POS[0])

print "N_ROBOTS : " + str(N_ROBOTS)

if obj_fun == "time":
	print"KUMAR2 T : " + str(max(TIMETABLE[-1]))
elif obj_fun == "distance":
	sumdist = TIMETABLE[-1][0] + TIMETABLE[-1][1]
	print"KUMAR2 sumD : " + str(sumdist)

print "CONFIGURATIONS :"
for i in range(0,len(CONFIGURATIONS)):
	miniC = ""
	for j in range(0, N_ROBOTS):
		miniC = miniC + str(CONFIGURATIONS[i][j]) + " "
	print miniC
print ";\n"
print "TIMETABLE : "
for i in range(0,len(TIMETABLE)):
	miniT = ""
	for j in range(0, N_ROBOTS):
		miniT = miniT + str(TIMETABLE[i][j]) + " "
	print miniT
print ";"


print("EXECUTION TIME KUMAR2: %s seconds\n" % (time.time() - start_time))
print "-------------------------------------------------------------------"