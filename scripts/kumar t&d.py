import math
import numpy as np




POINTS_TO_EXPLORE = [(1,5),(3,5),(3,4),(1,3),(1,4)]
N_ROBOT = 2
N_VERTEXES = 5
ROBOT_VELOCITY = 1
#GRAPH BUILT BY ME
#robot velocity = 2.3 m/s


#popolamento manuale matrice distanze
distance_matrix =  np.zeros((N_VERTEXES, N_VERTEXES))
distance_matrix[0][0] = 0
distance_matrix[0][1] = 14
distance_matrix[0][2] = 27
distance_matrix[0][3] = 35
distance_matrix[0][4] = 40
distance_matrix[1][0] = 14
distance_matrix[1][1] = 0
distance_matrix[1][2] = 17
distance_matrix[1][3] = 24
distance_matrix[1][4] = 42
distance_matrix[2][0] = 27
distance_matrix[2][1] = 17
distance_matrix[2][2] = 0
distance_matrix[2][3] = 21
distance_matrix[2][4] = 38
distance_matrix[3][0] = 35
distance_matrix[3][1] = 24
distance_matrix[3][2] = 21
distance_matrix[3][3] = 0
distance_matrix[3][4] = 19
distance_matrix[4][0] = 40
distance_matrix[4][1] = 42
distance_matrix[4][2] = 38
distance_matrix[4][3] = 19
distance_matrix[4][4] = 0

#popolamento matrice tempo
time_matrix =  np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,N_VERTEXES):
	for j in range(0,N_VERTEXES):
		time_matrix[i][j] = distance_matrix[i][j] / ROBOT_VELOCITY

print "DISTANCE Matrix"
print distance_matrix
print "\nTIME Matrix"
print time_matrix



#GRAFO DELLA MAPPA. Grafo completamente connesso
roadmap_graph = {1: [2,3,4,5],
         2: [1,3,4,5],
         3: [1,2,4,5],
         4: [1,2,3,5],
         5: [1,2,3,4]}

adjacency_road = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range (0,N_VERTEXES):
	for j in range (0,N_VERTEXES):
		if (j+1 in roadmap_graph[i+1]):
			adjacency_road[i][j] = 1
		

#GRAFO DEI PUNTI IN CUI MISURARE CONNESSIONE
radiomap_graph = {1: [3, 4, 5],
         2: [],
         3: [1, 4, 5],
         4: [1, 3],
         5: [1, 3]}

adjacency_radio = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range (0,N_VERTEXES):
	for j in range (0,N_VERTEXES):
		if (j+1 in radiomap_graph[i+1]):
			adjacency_radio[i][j] = 1

print "\nRoadmap adjacency MATRIX"
print adjacency_road
print"\nRadiomap MATRIX - Pairs to be measured"
print adjacency_radio

V2 = []
z=0
for i in range (0,N_VERTEXES):
	for j in range (i,N_VERTEXES):
		if adjacency_radio[i][j] == 1:
			z=z+1
			V2.append((z,[i+1,j+1]))
print "\n V2: each v represents the links between two points"
print V2
	#valori: indice lista / accesso a id_vertice o coppia / quale tra i due punti
	#print V2[1][1][1] #stampa 4
length = len(V2)

"""OPTIMAL PLAN FOR 2 ROBOTS"""
def distance_cost(roadmap_graph, adjacency_road, radiomap_graph, adjacency_radio):
	
	A2 = np.zeros((length, length))
	D2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				#il costo e' il minimo tra somma di (i-i',j-j') OR (i-j',j-i')
				D2[i][j]=min( (distance_matrix[V2[i][1][0]-1][V2[j][1][0]-1] + distance_matrix[V2[i][1][1]-1][V2[j][1][1]-1]) , (distance_matrix[V2[i][1][0]-1][V2[j][1][1]-1]+distance_matrix[V2[i][1][1]-1][V2[j][1][0]-1]))
	
	#symmetric wrt diagonal				
	print "\nCOST MATRIX for DISTANCES"
	print D2
	return D2


def time_cost(roadmap_graph, adjacency_road, radiomap_graph, adjacency_radio):
	A2 = np.zeros((length, length))
	T2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				#il costo e' il minimo tra i max di (i-i',j-j') OR (i-j',j-i')
				T2[i][j]=min(max(time_matrix[V2[i][1][0]-1][V2[j][1][0]-1], time_matrix[V2[i][1][1]-1][V2[j][1][1]-1]) , max(time_matrix[V2[i][1][0]-1][V2[j][1][1]-1],time_matrix[V2[i][1][1]-1][V2[j][1][0]-1]))
	#symmetric wrt diagonal				
	print "\nCOST MATRIX for TIME"
	print T2
	return T2



D = distance_cost(roadmap_graph,adjacency_road,radiomap_graph,adjacency_radio)
T = time_cost(roadmap_graph,adjacency_road,radiomap_graph,adjacency_radio)

#TODO: G2
"""
il grafo G2 sara' un grafo completamente connesso con tanti vertici quanto la dimensione della matrice D o T (5 in questo caso)
I pesi degli archi sono i valori contenuti nella matrice D o T
per trovare il piano ottimo bisogna risolvere il TSP su G2
"""



