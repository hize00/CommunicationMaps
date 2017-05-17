import math
import numpy as np

"""
#GRAPH BUILT PREVIOUSLY
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
distance_matrix[3][0] = 27
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
		time_matrix[i][j] = distance_matrix[i][j] / 2.3

print "DISTANCE Matrix"
print distance_matrix
print "\nTIME Matrix"
print time_matrix
"""


POINTS_TO_EXPLORE = [(1,5),(3,5),(3,4),(1,3),(1,4)]
N_ROBOT = 2
N_VERTEXES = 5


#GRAFO DELLA MAPPA
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

print "Roadmap adjacency MATRIX"
print adjacency_road
print"\nRadiomap MATRIX - Pairs to be measured"
print adjacency_radio



"""OPTIMAL PLAN FOR 2 ROBOTS"""
def cost_graph(roadmap_graph, adjacency_road, radiomap_graph, adjacency_radio):
	V2 = []
	z=0
	for i in range (0,N_VERTEXES):
		for j in range (i,N_VERTEXES):
			if adjacency_radio[i][j] == 1:
				z=z+1
				V2.append((z,[i+1,j+1]))
	print "\n V2: each v represents the links between two points"
	print V2
	#valori: indice lista / acesso a id_vertice o coppia / quale tra i due punti
	#print V2[1][1][1] #stampa 3

	length = len(V2)
	A2 = np.zeros((length, length))
	C2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				#il costo e' il minimo tra somma di (i-i',j-j') OR (i-j',j-i')
				#C2[i][j]=min((number_of_moves(roadmap_graph,V2[i][1][0],V2[j][1][0])+number_of_moves(roadmap_graph,V2[i][1][1],V2[j][1][1])) , (number_of_moves(roadmap_graph,V2[i][1][0],V2[j][1][1])+number_of_moves(roadmap_graph,V2[i][1][1],V2[j][1][0])))
				
	print "\n COST MATRIX"
	#it's symetric wrt diagonal
	print C2
	return C2

C = cost_graph(roadmap_graph,adjacency_road,radiomap_graph,adjacency_radio)

#TODO: G2
"""
il grafo G2 sara' un grafo completamente connesso con tanti vertici quanto la dimensione della matrice C (4 in questo caso)
I pesi degli archi sono i valori contenuti nella matrice C
per trovare il piano ottimo bisogna risolvere il TSP su G2
"""



