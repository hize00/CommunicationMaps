#######TIPO DI COMMENTO#######

""" COMMENTO A FUNZIONE"""
### codice da implementare
# commento a riga di codice

from copy import deepcopy
import math
import numpy as np
import time

#######OPERAZIONI STUPIDE SU DIZIONARI#########
"""
if '2' in C0:
	print "OK"

for k, v in C0.iteritems():
    print "robot "+str(k)+" is in vertex "+ str(v)

print "\nrobot 1 is in "+str(C0.get('1'))

#eliminare un elemento
del C0['4']

#cambiare key / inserire nuovo elemento
C0.update({'5':9})

#stampa in ordine di key
for key in sorted(C0):
    print "%s: %s" % (key, C0[key])
"""    
#################################################

N_VERTEXES = 11
#CONFIGURAZIONE INIZIALE -- DA FARE BENE con parsing ecc
#I NOMI DEI VERTEXES VANNO DA 0 A N_VERTEXES-1
vertexes = [(46,31),(34,19),(55,54),(46,16),(57,42),(27,16),(13,40),(2,8),(51,35),(48,51),(43,35)]
C0 = {1:vertexes[8],
	  2:vertexes[6],
	  3:vertexes[7],
	  4:vertexes[1],
	  5:vertexes[9]
	  }

#list of timestamp and correlated configurations
OUTPUT = []

#distance matrix initialization
distances = np.zeros((N_VERTEXES, N_VERTEXES))

""" euclidean distance between two vertexes"""
def vertex_dist(index_1, index_2):
	sub_x = math.pow((vertexes[index_1][0] - vertexes[index_2][0]), 2)
	sub_y = math.pow((vertexes[index_1][1] - vertexes[index_2][1]), 2)
	return math.sqrt(sub_x + sub_y)

"""distance matrix. Each vertex has the distances between itself and other nodes. In this way I compute the distances here and can access them by accessing the matrix """
for i in range(0,N_VERTEXES):
	for j in range(0,N_VERTEXES):
		distances[i][j] = vertex_dist(i,j)

print distances
print "\n"

"""torna la minima distanza del vertice a cui e' piu' vicino row_index"""
def minRowDistance(row_index):
	minimum = 999999999
	for i in range(0,N_VERTEXES):
		if i != row_index:
			if distances[row_index][i] < minimum:
				minimum = distances[row_index][i]
	return minimum


"""torna la colonna della matrice in cui c'e' la minima distanza della riga row_index"""
def minColIndex(row_index):
	minimum = 999999999
	for i in range(0,N_VERTEXES):
		if i != row_index:
			if distances[row_index][i] < minimum:
				minimum = distances[row_index][i]
				minCol = i
	return minCol

""" R(C): ritorna i robots presenti in configurazione C in ordine di ID """
def robotsInConfiguration(configuration):
	return sorted(configuration.keys())

""" C(r): vertex in which robot r is in configuration C. Ritorna numero vertice.
    Se r non e' presente ritorna None """
def robotVertex(robot,configuration):
	if robot in configuration.keys():
		for i in range(0,len(vertexes)):
			if vertexes[i]==configuration.get(robot):
				return i


""" adiacenza tra due vertici. Ritorna 1 se la distanza tra di loro e' minima """
def adjacency(vertex1, vertex2):
	#if vertex1 connected to vertex 2 by minimum distance
	if distances[vertex1][vertex2]==minRowDistance(vertex1):
		return 1
	else:
		return 0

""" ritorna una lista di vertici che rappresentano lo SP da vertex1 a vertex2 """
def shortestPath(vertex1, vertex2):
	SP_list = [vertex1]
	j = vertex1
	while(j!=vertex2):
		newNode = minColIndex(j)
		SP_list.append(newNode)
		j = newNode
	return SP_list

""" move robot in an adjacent vertex """
def adjacentMove(robot,configuration,newVertex):
	newConfiguration = deepcopy(configuration)
	newConfiguration.update({robot:vertexes[newVertex]})
	return newConfiguration	



""" move robot from configuration (if exists) to newVertex in newConfiguration """
def moveRobot(robot,configuration,newVertex):
	if robot not in configuration.keys():
		print "Error: " + str(robot) + " not in configuration "+str(configuration)
		return configuration

	else:
		path_vertexes = []
		temporaryConfigurations = []
		start_time = time.time()
		#if robot is in a vertex adjacent to newVertex it moves there directly
		if (adjacency(robotVertex(robot,configuration),newVertex)==1):	
			newConfiguration=adjacentMove(robot,configuration,newVertex)
		else:
			#compute SHORTEST PATH between the two vertexes. Return a list of vertexes
			path_vertexes = shortestPath(robotVertex(robot,configuration), newVertex)
			#temporary configurations needed when moving between non-adjacent vertexes		
			temporaryConfigurations.append(configuration)

			#move robot in intermediate configurations (each step is an adjacent move)
			for i in range(1,len(path_vertexes)):
				#adjacentMove from robotVertex and first vertexes on the list
				temporaryConfigurations.append(adjacentMove(robot,robotVertex(robot,temporaryConfigurations[i-1]),path_vertexes[i]))
		
			newConfiguration = deepcopy(temporaryConfigurations[-1])

		end_time = time.time()
		TIME = end_time - start_time

		OUTPUT.append((newConfiguration,TIME))
		return OUTPUT

		###controllare funzione TIME
		###nome CONFIGURAZIONE



moveRobot(3,C0,5)
print OUTPUT