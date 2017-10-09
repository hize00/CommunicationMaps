import math
import time
import numpy as np
import sys
import itertools
from igraph import *
from gurobipy import *

#start the time
start_time = time.time()

#VERSION WITH INDEX OF GRAPH FROM 0 TO n_vertex-1
#IT ALSO PARSES FILES .dat
file_to_open = sys.argv[1]
obj_fun = str(sys.argv[2])

# Parse argument
if len(sys.argv) < 3:
    print("Usage: python filename.py datafile.dat obj_fun('time' or 'distance')")
    exit(1)

print "\n------------PARSING THE FILE------------\n"
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
			#print words
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

print "\n---> NUMBER OF ROBOTS = " + str(N_ROBOTS)
print "\n---> STARTING POSITIONS "
print STARTING_POS
print "\n---> NUMBER OF VERTEXES = " + str(N_VERTEXES)
print "\n---> ROBOT VELOCITY = " + str(ROBOT_VELOCITY)
print "\n---> POINTS TO EXPLORE "
print POINTS_TO_EXPLORE
print "\n"

#time matrix initialization
time_matrix =  np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,N_VERTEXES):
	for j in range(0,N_VERTEXES):
		time_matrix[i][j] = distance_matrix[i][j] / ROBOT_VELOCITY

print "DISTANCE Matrix"
print distance_matrix
print "\nTIME Matrix"
print time_matrix


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

print "\nRoadmap adjacency MATRIX"
print adjacency_road
print"\nRadiomap MATRIX - Pairs to be measured"
print adjacency_radio


g = Graph()
#indexing will go from 0 to N_VERTEXES-1 
g.add_vertices(N_VERTEXES)

EDGES = []
for i in range(0,N_VERTEXES-1):
	for j in range(i+1,N_VERTEXES):
		EDGES.append((i,j))

g.add_edges(EDGES)
gEdgeList = g.get_edgelist()
print "\nEDGES LIST"
print g.get_edgelist()

if obj_fun == "time":
	for i in range(0,len(gEdgeList)):
		g.es[i]["weight"] = time_matrix[gEdgeList[i][0]][gEdgeList[i][1]]
elif obj_fun == "distance":
	for i in range(0,len(gEdgeList)):
		g.es[i]["weight"] = distance_matrix[gEdgeList[i][0]][gEdgeList[i][1]]

print "\nGRAPH\n"
print g
for i in range (0, len(EDGES)):
	print g.es[i].attributes()

#LIST of the shortest distances between vertexes
shortest_list = g.shortest_paths_dijkstra( weights="weight" )
print "\nSHORTEST PATHS LIST - " + str(obj_fun)
print shortest_list
#LIST to MATRIX conversion
shortest_matrix = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0, N_VERTEXES):
	for j in range(0, N_VERTEXES):
		shortest_matrix[i][j] = shortest_list[i][j]

print "\nSHORTEST PATHS MATRIX - " + str(obj_fun)
print shortest_matrix



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
print "\nV2: each nodes represents a link between two points. Configurations not ordered"
print V2
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
				#TODO: old matrix
				D2[i][j]=min( (shortest_matrix[V2[i][1][0]][V2[j][1][0]] + shortest_matrix[V2[i][1][1]][V2[j][1][1]]) , (shortest_matrix[V2[i][1][0]][V2[j][1][1]]+shortest_matrix[V2[i][1][1]][V2[j][1][0]]))
	
	#symmetric wrt diagonal				
	print "\nCOST MATRIX for DISTANCES"
	print D2
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
	print "\nCOST MATRIX for TIME"
	print T2
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

print "\n\n\n|------------GUROBI TSP COMPUTATION------------|\n"
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

n = len(POINTS_TO_EXPLORE)
# Add degree-2 constraint
m.addConstrs(vars.sum(i,'*') == 2 for i in range(n))

# Optimize model
m._vars = vars
m.Params.lazyConstraints = 1
m.optimize(subtourelim)

vals = m.getAttr('x', vars)
selected = tuplelist((i,j) for i,j in vals.keys() if vals[i,j] > 0.5)

tour = subtour(selected)
assert len(tour) == n

print('')
print('Optimal tour: %s' % str(tour))
print('Optimal cost: %g' % m.objVal)
print('')
print "|------------GUROBI TSP COMPUTATION ENDED------------|\n\n\n"


#accessing values of V2: [list index] | [vertex_id or pair] | [if pair, selects if first or second point]
#example1: V2[1][1][1] accesses 2nd element of list, pair of point, second point of the pair
#example2: V2[2][0] accesses 3rd element of list and vertex_id.
tour_list = []
tour_list.append([ V2[tour[0]][1][0], V2[tour[0]][1][1] ])

#obsolete
if obj_fun == "distance":
	t = 0
	for i in range(1,len(tour)):
		print "\ntour iteration number " + str(i-1)
		t1 = D[tour[i-1]][tour[i]] 
		print "iteration distance: " + str(t1) 
		t = t + t1
		tour_list.append([ V2[tour[i]][1][0], V2[tour[i]][1][1] ])
		print tour_list

#transforming the tour planned by GUROBI into a succession of points in tour_list (not in order of robots)
elif obj_fun == "time":
	t = 0
	for i in range(1,len(tour)):
		print "\ntour iteration number " + str(i-1)
		t1 = T[tour[i-1]][tour[i]] 
		print "iteration time: " + str(t1) 
		t = t + t1
		tour_list.append([ V2[tour[i]][1][0], V2[tour[i]][1][1] ])
		print tour_list

#list to order the tour points to visit
tour_configuration = []

#first append:  node of the GUROBI tour closest to STARTING_POS
firstAppend = 9999999999999999999
if obj_fun == "time":	
	for i in range(0,len(tour)):
		t1 = max(shortest_matrix[STARTING_POS[0]][tour_list[i][0]] , shortest_matrix[STARTING_POS[1]][tour_list[i][1]])
		t2 = max(shortest_matrix[STARTING_POS[0]][tour_list[i][1]] , shortest_matrix[STARTING_POS[1]][tour_list[i][0]])
		stfAp = min(t1, t2)
		if stfAp < firstAppend:
			firstAppend = stfAp
			#index of tour_list of the closest point to STARTING_POS
			fAp_index = i

elif obj_fun == "distance":
	for i in range(0,len(tour)):
		t1 = shortest_matrix[STARTING_POS[0]][tour_list[i][0]] + shortest_matrix[STARTING_POS[1]][tour_list[i][1]]
		t2 = shortest_matrix[STARTING_POS[0]][tour_list[i][1]] + shortest_matrix[STARTING_POS[1]][tour_list[i][0]]
		stfAp = min(t1, t2)
		if stfAp < firstAppend:
			firstAppend = stfAp
			#index of tour_list of the closest point to STARTING_POS
			fAp_index = i

if obj_fun == "time":
	t1 = max(shortest_matrix[STARTING_POS[0]][tour_list[fAp_index][0]] , shortest_matrix[STARTING_POS[1]][tour_list[fAp_index][1]])
	t2 = max(shortest_matrix[STARTING_POS[0]][tour_list[fAp_index][1]] , shortest_matrix[STARTING_POS[1]][tour_list[fAp_index][0]])
elif obj_fun == "distance":
	t1 = shortest_matrix[STARTING_POS[0]][tour_list[fAp_index][0]] + shortest_matrix[STARTING_POS[1]][tour_list[fAp_index][1]]
	t2 = shortest_matrix[STARTING_POS[0]][tour_list[fAp_index][1]] + shortest_matrix[STARTING_POS[1]][tour_list[fAp_index][0]]
if t1<=t2:
	tour_configuration.append([ tour_list[fAp_index][0], tour_list[fAp_index][1] ])
else:
	tour_configuration.append([ tour_list[fAp_index][1], tour_list[fAp_index][0] ])


print "\n\n\n----------------------------------------------------"
print "RECAP"
print "STARTING POSITION: " + str(STARTING_POS)
print "time needed for completing the tour: " + str(t)

CONFIGURATIONS = []
CONFIGURATIONS.append(STARTING_POS)

#computation of the starting position of the GUROBI tour. It has already been computed with fAp_index
stp = 0
if (STARTING_POS[0]==tour_configuration[0][0] and STARTING_POS[1]==tour_configuration[0][1]) or (STARTING_POS[0]==tour_configuration[0][1] and STARTING_POS[1]==tour_configuration[0][0]):
	stp = 1
if stp == 1:
	stpL = []
	stpL.append(-1)
	stpL.append(-1)
	stpL[0] = STARTING_POS[0]
	stpL[1] = STARTING_POS[1]
	print "GUROBI TOUR STARTS FROM: " + str(STARTING_POS[0]) + " " + str(STARTING_POS[1])

else:
	stpL = []
	stpL.append(-1)
	stpL.append(-1)
	stpL[0] = tour_configuration[0][0]
	stpL[1] = tour_configuration[0][1]
	print "GUROBI TOUR STARTS FROM " + str(stpL[0]) + " " + str(stpL[1])


if obj_fun == "time":
	#check that fAp_index is not the last element, otherwise it has been already added
	if fAp_index < len(tour_list)-1:
	#second half of the tour_list: from [fAp_index] to end
		#the element [fAp_index] has been already taken, therefore the loop starts at [fAp_index+1]
		for i in range(fAp_index+1,len(tour_list)):
			if max(shortest_matrix[tour_configuration[-1][0]][tour_list[i][0]] , shortest_matrix[tour_configuration[-1][1]][tour_list[i][1]]) <= max(shortest_matrix[tour_configuration[-1][0]][tour_list[i][1]] , shortest_matrix[tour_configuration[-1][1]][tour_list[i][0]]):
				tour_configuration.append([tour_list[i][0] , tour_list[i][1]])
			else:
				tour_configuration.append([tour_list[i][1] , tour_list[i][0]])
	#first half of the tour_list: from 0 to [fAp_index]
	for i in range(0,fAp_index):
		if max(shortest_matrix[tour_configuration[-1][0]][tour_list[i][0]] , shortest_matrix[tour_configuration[-1][1]][tour_list[i][1]]) <= max(shortest_matrix[tour_configuration[-1][0]][tour_list[i][1]] , shortest_matrix[tour_configuration[-1][1]][tour_list[i][0]]):
			tour_configuration.append([tour_list[i][0] , tour_list[i][1]])
		else:
			tour_configuration.append([tour_list[i][1] , tour_list[i][0]])

elif obj_fun == "distance":
	#check that fAp_index is not the last element, otherwise it has been already added
	if fAp_index < len(tour_list)-1:
	#second half of the tour_list: from [fAp_index] to end
		#the element [fAp_index] has been already taken, therefore the loop starts at [fAp_index+1]
		for i in range(fAp_index+1,len(tour_list)):
			if shortest_matrix[tour_configuration[-1][0]][tour_list[i][0]] + shortest_matrix[tour_configuration[-1][1]][tour_list[i][1]] <= shortest_matrix[tour_configuration[-1][0]][tour_list[i][1]] + shortest_matrix[tour_configuration[-1][1]][tour_list[i][0]]:
				tour_configuration.append([tour_list[i][0] , tour_list[i][1]])
			else:
				tour_configuration.append([tour_list[i][1] , tour_list[i][0]])
	#first half of the tour_list: from 0 to [fAp_index]
	for i in range(0,fAp_index):
		if shortest_matrix[tour_configuration[-1][0]][tour_list[i][0]] + shortest_matrix[tour_configuration[-1][1]][tour_list[i][1]] <= shortest_matrix[tour_configuration[-1][0]][tour_list[i][1]] + shortest_matrix[tour_configuration[-1][1]][tour_list[i][0]]:
			tour_configuration.append([tour_list[i][0] , tour_list[i][1]])
		else:
			tour_configuration.append([tour_list[i][1] , tour_list[i][0]])


#print "\nFINAL CONFIGURATION TOUR LIST:"
#print tour_configuration
for i in range(0,len(tour_configuration)):
	CONFIGURATIONS.append([tour_configuration[i][0], tour_configuration[i][1]])
CONFIGURATIONS.append([STARTING_POS[0], STARTING_POS[1]])
print "\nFINAL CONFIGURATION LIST:"
print CONFIGURATIONS

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
	print "\n TIMETABLE - " + str(obj_fun)
	print TIMETABLE

elif obj_fun == "distance":
	for i in range(0,len(CONFIGURATIONS)-1):
		ttab[0] = TIMETABLE[-1][0] + shortest_matrix[CONFIGURATIONS[i][0]][CONFIGURATIONS[i+1][0]]
		ttab[1] = TIMETABLE[-1][1] + shortest_matrix[CONFIGURATIONS[i][1]][CONFIGURATIONS[i+1][1]]
		tt = deepcopy(ttab)
		TIMETABLE.append(tt)
	print "\n TIMETABLE - " + str(obj_fun)
	print TIMETABLE



#final move
#print "\nFINAL MOVE: robots will move from " + str(tour_configuration[-1]) + " back to " + str(STARTING_POS)
print"\nKUMAR2 last timestamp is " + str(max(TIMETABLE[-1]))

print("\n\n---EXECUTION TIME: %s seconds ---\n" % (time.time() - start_time))