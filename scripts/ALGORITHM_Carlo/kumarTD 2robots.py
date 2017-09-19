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
#print "ACCESSING STRUCTURE EXAMPLE:"
# print gEdgeList[1][1]
# print time_matrix[gEdgeList[1][0]][gEdgeList[1][1]]
# print time_matrix[0][2]
for i in range(0,len(gEdgeList)):
	g.es[i]["weight"] = time_matrix[gEdgeList[i][0]][gEdgeList[i][1]]

print "\nGRAPH\n"
print g
for i in range (0, len(EDGES)):
	print g.es[i].attributes()

#LIST of the shortest distances between vertexes
shortest_time_list = g.shortest_paths_dijkstra( weights="weight" )
print "\nSHORTEST PATHS LIST - TIME"
print shortest_time_list
#LIST to MATRIX conversion
shortest_time_matrix = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0, N_VERTEXES):
	for j in range(0, N_VERTEXES):
		shortest_time_matrix[i][j] = shortest_time_list[i][j]

print "\nSHORTEST PATHS MATRIX - TIME"
print shortest_time_matrix



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
print "\nV2: each nodes represents a link between two points"
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
				D2[i][j]=min( (distance_matrix[V2[i][1][0]][V2[j][1][0]] + distance_matrix[V2[i][1][1]][V2[j][1][1]]) , (distance_matrix[V2[i][1][0]][V2[j][1][1]]+distance_matrix[V2[i][1][1]][V2[j][1][0]]))
	
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
				T2[i][j]=min(max(shortest_time_matrix[V2[i][1][0]][V2[j][1][0]], shortest_time_matrix[V2[i][1][1]][V2[j][1][1]]) , max(shortest_time_matrix[V2[i][1][0]][V2[j][1][1]],shortest_time_matrix[V2[i][1][1]][V2[j][1][0]]))
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


# Parse argument
if len(sys.argv) < 3:
    print("Usage: filename.py datafile.dat obj_fun('time' or 'distance')")
    exit(1)


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

#first value of configuration is robot1. 2nd value is robot2.
tour_configuration = []
#starting configuration: first node of the GUROBI tour
tour_configuration.append( [ V2[tour[0]][1][0], V2[tour[0]][1][1] ] )

if obj_fun == "distance":
	for i in range(1,len(tour)):
		if ( D[V2[tour[i-1]][1][0]][V2[tour[i]][1][0]] + D[V2[tour[i-1]][1][1]][V2[tour[i]][1][1]] ) <  ( D[V2[tour[i-1]][1][0]][V2[tour[i]][1][1]] + D[V2[tour[i-1]][1][1]][V2[tour[i]][1][0]] ) :
			tour_configuration.append( [ V2[tour[i]][1][0], V2[tour[i]][1][1] ] )
		else:
			tour_configuration.append( [ V2[tour[i]][1][1], V2[tour[i]][1][0] ] )

#transforming the tour planned by GUROBI into a succession of proper tour_configuration
elif obj_fun == "time":
	t = 0
	for i in range(1,len(tour)):
		print "\ntour iteration number " + str(i-1)
		t1 = T[tour[i-1]][tour[i]] 
		print "iteration time: " + str(t1) 
		t = t + t1
		
		if max(shortest_time_matrix[tour_configuration[i-1][0]][V2[tour[i]][1][0]] , shortest_time_matrix[tour_configuration[i-1][1]][V2[tour[i]][1][1]]) <= max(shortest_time_matrix[tour_configuration[i-1][0]][V2[tour[i]][1][1]] , shortest_time_matrix[tour_configuration[i-1][1]][V2[tour[i]][1][0]]):
			tour_configuration.append([ V2[tour[i]][1][0], V2[tour[i]][1][1] ])
		else:
			tour_configuration.append( [ V2[tour[i]][1][1], V2[tour[i]][1][0] ] )

		print tour_configuration

print "\n\n\n----------------------------------------------------"
print "RECAP"
print "STARTING POSITION: " + str(STARTING_POS)
print "time needed for completing the tour: " + str(t)

#computation of the starting position of the GUROBI tour. It's the closest point to STARTING_POS
stp = 0
for i in range(0, len(tour_configuration)):
	if (STARTING_POS[0]==tour_configuration[i][0] and STARTING_POS[1]==tour_configuration[i][1]):
		stp = 1
		stp_index = i
if stp == 1:
	print "TOUR STARTS FROM: " + str(STARTING_POS[0]) + " " + str(STARTING_POS[1])

else:
	stpL = []
	stpL.append(-1)
	stpL.append(-1)
	min_distance = 999999999
	for i in range(0, len(tour_configuration)):
		stpDistance = min(max(shortest_time_matrix[STARTING_POS[0]][tour_configuration[i][0]] , shortest_time_matrix[STARTING_POS[1]][tour_configuration[i][1]]), max(shortest_time_matrix[STARTING_POS[0]][tour_configuration[i][1]] , shortest_time_matrix[STARTING_POS[1]][tour_configuration[i][0]]))
		if stpDistance < min_distance:
			min_distance = stpDistance
			stpL[0] = tour_configuration[i][0]
			stpL[1] = tour_configuration[i][1]
			stp_index = i
	print "TOUR STARTS FROM " + str(stpL[0]) + " " + str(stpL[1]) 

#ordering tour configuration list 
#stp_index is the index from where the tour starts
CONFIGURATIONS = []
tour_configuration_sorted = []
CONFIGURATIONS.append(STARTING_POS)
for i in range(stp_index,len(tour_configuration)):
	CONFIGURATIONS.append(tour_configuration[i])
	tour_configuration_sorted.append(tour_configuration[i])
for i in range(0,stp_index):
	CONFIGURATIONS.append(tour_configuration[i])
	tour_configuration_sorted.append(tour_configuration[i])



print "\nFINAL CONFIGURATION TOUR LIST:"
print tour_configuration_sorted

print "\nFINAL CONFIGURATION LIST:"
print CONFIGURATIONS

#final move
print "\nFINAL MOVE: robots will move from " + str(tour_configuration[-1+stp_index]) + " back to " + str(STARTING_POS)
#adding the time from STARTING_POS to FIRST
tS = min(max(shortest_time_matrix[STARTING_POS[0]][tour_configuration[stp_index][0]] , shortest_time_matrix[STARTING_POS[1]][tour_configuration[stp_index][1]]), max(shortest_time_matrix[STARTING_POS[0]][tour_configuration[stp_index][1]] , shortest_time_matrix[STARTING_POS[1]][tour_configuration[stp_index][0]]))
print "\ntime for moving from STARTING_POS to first point of the tour " 
print str(STARTING_POS) + " ---> " + str(tour_configuration[stp_index])+ " : " + str(tS)
#adding the time from LAST to STARTING_POS
tF = min(max(shortest_time_matrix[STARTING_POS[0]][tour_configuration[-1+stp_index][0]] , shortest_time_matrix[STARTING_POS[1]][tour_configuration[-1+stp_index][1]]), max(shortest_time_matrix[STARTING_POS[0]][tour_configuration[-1+stp_index][1]] , shortest_time_matrix[STARTING_POS[1]][tour_configuration[-1+stp_index][0]]))
print "time for moving from last point of the tour to STARTING_POS " 
print str(tour_configuration[-1+stp_index]) + " ---> " + str(STARTING_POS)+ " : " + str(tF)

tEND = t + tS + tF
print "\n\nFINAL TIMESTAMP: " + str(tEND)

print("\n\n\n---EXECUTION TIME: %s seconds ---\n" % (time.time() - start_time))