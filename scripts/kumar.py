import math
import numpy as np

POINTS_TO_EXPLORE = [(1,5),(2,6),(2,3),(3,4)]
N_ROBOT = 2
N_VERTEXES = 6

#GRAFO DELLA MAPPA
roadmap_graph = {1: [2],
         2: [1, 3],
         3: [2, 4, 5],
         4: [3],
         5: [3, 6],
         6: [5]}

adjacency_road = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range (0,N_VERTEXES):
	for j in range (0,N_VERTEXES):
		if (j+1 in roadmap_graph[i+1]):
			adjacency_road[i][j] = 1

#GRAFO DEI PUNTI IN CUI MISURARE CONNESSIONE
radiomap_graph = {1: [5],
         2: [3, 6],
         3: [2, 4],
         4: [3],
         5: [1],
         6: [2]}

adjacency_radio = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range (0,N_VERTEXES):
	for j in range (0,N_VERTEXES):
		if (j+1 in radiomap_graph[i+1]):
			adjacency_radio[i][j] = 1

print adjacency_road
print adjacency_radio

def find_shortest_path(graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        if not graph.has_key(start):
            return None
        shortest = None
        for node in graph[start]:
            if node not in path:
                newpath = find_shortest_path(graph, node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest

def number_of_moves(graph, vertex1, vertex2):
	if find_shortest_path(graph, vertex1, vertex2) == None:
		count = 9999999
	else : count = len(find_shortest_path(graph, vertex1, vertex2))
	return count

print number_of_moves(roadmap_graph,1,4)

"""OPTIMAL PLAN FOR 2 ROBOTS"""
def optimal_plan2(roadmap_graph, adjacency_road, radiomap_graph, adjacency_radio):
	V2 = []
	z=0
	for i in range (0,int(math.ceil(N_VERTEXES/2))):
		for j in range (0,N_VERTEXES):
			if adjacency_radio[i][j] == 1:
				z=z+1
				V2.append((z,(i+1,j+1)))
	print "\n V2"
	print V2
	length = len(V2)
	A2 = np.zeros((length, length))
	C2 = np.zeros((length, length))
	for i in range (0,length):
		for j in range (0,length):
			if V2[i][0]!=V2[j][0]:
				A2[i][j]=1
				C2[i][j]=number_of_moves(roadmap_graph,i+1,j+1)
	print "\n COST MATRIX"
	print C2
	#TODO compute minimum cost open path on G2 s.t. each node in V2 is traversed only once

optimal_plan2(roadmap_graph, adjacency_road, radiomap_graph, adjacency_radio)
