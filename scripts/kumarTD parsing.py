import math
import numpy as np
import sys
import itertools
from gurobipy import *

# |---------------------------|
# |DATA AND PARAMETERS - START|
# |---------------------------|

#VERSION WITH INDEX OF GRAPH FROM 0 TO n_vertex-1
#IT ALSO PARSES FILES .dat

file_to_open = sys.argv[1]
obj_fun = str(sys.argv[2])


#RITORNA N_VERTEX, ROBOT_VELOCITY, POINTS_TO_EXPLORE e DISTANCE_MATRIX
def parsing_file(datfile):
	P_EXPLORE = []
	readingpoints=0
	readingmatrix=0
	with open(file_to_open, "r") as f:
		data = f.readlines()
		for line in data:
			words = line.split()
			#print words
			if len(words)>0:
				if words[0]=='VERTEXES':
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
				
		return N_VERT, R_VELOCITY, P_EXPLORE, distance_matrix


		
N_VERTEXES, ROBOT_VELOCITY, POINTS_TO_EXPLORE, distance_matrix = parsing_file(file_to_open)

print "\n---> NUMBER OF VERTEXES = " + str(N_VERTEXES)
print "\n---> ROBOT VELOCITY = " + str(ROBOT_VELOCITY)
print "\n---> POINTS TO EXPLORE "
print POINTS_TO_EXPLORE
print "\n"

#popolamento matrice tempo
time_matrix =  np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,N_VERTEXES):
	for j in range(0,N_VERTEXES):
		time_matrix[i][j] = distance_matrix[i][j] / ROBOT_VELOCITY

print "DISTANCE Matrix"
print distance_matrix
print "\nTIME Matrix"
print time_matrix


#MATRICE ROAD DI ADIACENZA. Essendo un grafo completamente connesso gli unici 0 sono sulla diagonale
adjacency_road = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range (0,N_VERTEXES):
	for j in range (0,N_VERTEXES):
		if j!=i:
			adjacency_road[i][j] = 1
		

n_to_explore = len(POINTS_TO_EXPLORE)
#MATRICE RADIO DEI PUNTI IN CUI MISURARE CONNESSIONE. 1 SE BISOGNA MISURARE
adjacency_radio = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,n_to_explore):
	adjacency_radio[POINTS_TO_EXPLORE[i][0]][POINTS_TO_EXPLORE[i][1]] = 1
	adjacency_radio[POINTS_TO_EXPLORE[i][1]][POINTS_TO_EXPLORE[i][0]] = 1

print "\nRoadmap adjacency MATRIX"
print adjacency_road
print"\nRadiomap MATRIX - Pairs to be measured"
print adjacency_radio

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
print "\n V2: each v represents the links between two points"
print V2
	#valori: indice lista / accesso a id_vertice o coppia / quale tra i due punti
	#print V2[1][1][1] #stampa 4
length = len(V2)

"""DISTANCE COST MATRIX"""
def distance_cost(adjacency_road, adjacency_radio):
	
	A2 = np.zeros((length, length))
	D2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				#il costo e' il minimo tra somma di (i-i',j-j') OR (i-j',j-i')
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
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				#il costo e' il minimo tra i max di (i-i',j-j') OR (i-j',j-i')
				T2[i][j]=min(max(time_matrix[V2[i][1][0]][V2[j][1][0]], time_matrix[V2[i][1][1]][V2[j][1][1]]) , max(time_matrix[V2[i][1][0]][V2[j][1][1]],time_matrix[V2[i][1][1]][V2[j][1][0]]))
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
matrix_length = int(math.sqrt(D.size))


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

n = N_VERTEXES
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

