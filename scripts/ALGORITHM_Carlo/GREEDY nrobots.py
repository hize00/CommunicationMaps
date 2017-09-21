import math
import time
import numpy as np
import sys
import itertools
from igraph import *
from operator import itemgetter
from gurobipy import *
from copy import deepcopy

#start the time
start_time = time.time()
# |---------------------------|
# |DATA AND PARAMETERS - START|
# |---------------------------|

#VERSION WITH INDEX OF GRAPH FROM 0 TO n_vertex-1
#IT ALSO PARSES FILES .dat

file_to_open = sys.argv[1]
obj_fun = str(sys.argv[2])


#returns N_ROBOTS, STARTING_POS, N_VERTEX, ROBOT_VELOCITY, POINTS_TO_EXPLORE and DISTANCE_MATRIX
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

		
n_to_explore = len(POINTS_TO_EXPLORE)
#RADIO MATRIX. Values are 1 for points where we need to measure connection
adjacency_radio = np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,n_to_explore):
	adjacency_radio[POINTS_TO_EXPLORE[i][0]][POINTS_TO_EXPLORE[i][1]] = 1
	adjacency_radio[POINTS_TO_EXPLORE[i][1]][POINTS_TO_EXPLORE[i][0]] = 1


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

#Generic state of the tree. Parameters: idNumber,configuration, timeTable, depth level, points to visit, father, children
class State(object):

	def __init__(self, idNumber, configuration, timeTable, depth_level, points_to_visit, father, children):
		self.idNumber = idNumber
		self.configuration = configuration
		self.timeTable = timeTable
		self.depth_level = depth_level
		self.points_to_visit = points_to_visit
		self.father = father
		self.children = children

	def infoState(self):
		print "\n"
		print "---> INFO ABOUT STATE " + str(self.idNumber)
		print "| ID NUMBER |"
		print self.idNumber
		print "| ROBOT CONFIGURATION |"
		print self.configuration
		print "| TIMETABLE |"
		print self.timeTable
		print "| DEPTH LEVEL |"
		print self.depth_level
		print "| POINTS TO VISIT |"
		print self.points_to_visit
		print "| FATHER |"
		print self.father
		print "| CHILDREN |"
		gC = self.children
		for i in range (0, len(gC)):
			print gC[i].getidNumber()
		print "\n"

	def getidNumber(self):
		return self.idNumber
	def setidNumber(self,idn):
		self.idNumber = idn

	def getConfiguration(self):
		return self.configuration
	def setConfiguration(self, config):
		self.configuration = config

	def getTimeTable(self):
		return self.timeTable
	def setTimeTable(self, ttable):
		self.timeTable = ttable

	def getDepthLevel(self):
		return self.depth_level
	def setDepthLevel(self, dL):
		self.depth_level = dL

	def getPointsToVisit(self):
		return self.points_to_visit
	def setPointsToVisit(self, ptv):
		self.points_to_visit = ptv

	def getFather(self):
		return self.father
	def setFather(self, fath):
		self.father = fath

	def getChildren(self):
		return self.children
	def addChild(self, obj):
		self.children.append(obj)

#configuration List. STARTING_POS will be appended by root
CONFIGURATIONS = []

#list of created states
STATES = []

#timeTable initialized at 0
timeTable_ini =  np.zeros((N_ROBOTS, 1))

#ROOT State. Id & dept_level = 0, STARTING_POS & POINTS_TO_EXPLORE are given by input, Children list is empty
root = State(0, STARTING_POS, timeTable_ini, 0, POINTS_TO_EXPLORE, None, [])
STATES.append(root)

root.infoState()

#return the maximum identification number of the already created states.
def maxId(stateList):
	maxNumber = 0
	for i in range (0, len(stateList)):
		if stateList[i].getidNumber() > maxNumber:
			maxNumber = stateList[i].getidNumber()
	return maxNumber


#return Moves sorted by time: [ [t][C_ini][C_end] [moving robots] [visited pair] ]
def computeMoves(configuration, points_to_visit):
	moves = []
	sortedMoves = []
	movingRobots = [0] * len(configuration) #1 for robots who are moving
	pair = [None, None]
	#for each couple of point in -configuration- I need to compute all possible moves to reach -points_to_visit-
	for i in range(0,len(configuration)-1):
		for j in range(i+1,len(configuration)):
			for k in range(0,len(points_to_visit)):
				#t is the time
				t = min( max(shortest_time_matrix[configuration[i]][points_to_visit[k][0]], shortest_time_matrix[configuration[j]][points_to_visit[k][1]]), max(shortest_time_matrix[configuration[i]][points_to_visit[k][1]], shortest_time_matrix[configuration[j]][points_to_visit[k][0]]))
				if (max(shortest_time_matrix[configuration[i]][points_to_visit[k][0]], shortest_time_matrix[configuration[j]][points_to_visit[k][1]])) <= max(shortest_time_matrix[configuration[i]][points_to_visit[k][1]], shortest_time_matrix[configuration[j]][points_to_visit[k][0]]):
					movingRobots = [0] * len(configuration)
					cM = list(configuration)
					cM[i] = points_to_visit[k][0]
					cM[j] = points_to_visit[k][1]
					movingRobots[i] = 1
					movingRobots[j] = 1
					pair[0] = points_to_visit[k][0]
					pair[1] = points_to_visit[k][1]
					#-----CAREFUL WHAT TO APPEND: []: LIST----[[]]: NESTED LIST
					moves.append((t,configuration,cM[:], movingRobots,[pair[:]]))

				else:
					movingRobots = [0] * len(configuration)
					cM = list(configuration)
					cM[i] = points_to_visit[k][1]
					cM[j] = points_to_visit[k][0]
					movingRobots[i] = 1
					movingRobots[j] = 1
					pair[0] = points_to_visit[k][0]
					pair[1] = points_to_visit[k][1]
					moves.append((t,configuration,cM[:], movingRobots, [pair[:]]))


	sortedMoves = sorted(moves, key=itemgetter(0))
	return sortedMoves

###ESPANDI: given a State return the list calculated by computeMoves
def EXPAND(state):
	LEx = []
	print"\nSorted moves of state " + str(state.getidNumber())
	LEx = computeMoves(state.getConfiguration(), state.getPointsToVisit())
	print LEx
	return LEx


###CHOOSE: given a State and a heuristic chooses the move to make, so it returns an instance of the List calculated by computeMoves
def CHOOSE(state, heuristic):
	if heuristic == "greedy":
		move = GREEDY(state)
		#updateTimeTable(move)
		return move
	else:
		print "heuristic doesn't exist"

###GREEDY HEURISTIC: pure greedy choice
def GREEDY(state):
	LGe = []
	LGe = EXPAND(state)
	chosen = LGe[0]
	return chosen


#given a state and a move updates the timetable (for the child)
def updateTimeTable(state, move):
	tt = state.getTimeTable()
	#move[3] contains the robots moving
	for i in range(0,len(move[3])):
		#if 1 update the table
		if move[3][i] == 1:
			#move[0] contains the time
			tt[i][0] = tt[i][0] + move[0]
	return tt

#given a state and a move updates the pointovisit (for the child)
def updatePointToVisit(state, move):
	ppp = state.getPointsToVisit()
	#move[4] contains the pair visited
	pToDelete = move[4]
	#delete pToDelete
	ppp = [x for x in ppp if x not in pToDelete]
	return ppp

###GENERATE: 
def GENERATE(state, heuristic):
	m = CHOOSE(state, heuristic)
	ttable = updateTimeTable(state, m)
	ptv = updatePointToVisit(state, m)
	#m[2] is the configuration after a move
	s = State(maxId(STATES)+1, m[2], ttable, state.getDepthLevel()+1, ptv, state, [])
	#STATES.append(s)
	state.addChild(s)
	return s


#------------------------------------------------------
#greedy deterministica

state = deepcopy(root)
for i in range(0, len(POINTS_TO_EXPLORE)):
	print "\n---> iteration " + str(i)
	s = GENERATE(state, "greedy")
	STATES.append(s)
	s.infoState()
	state = deepcopy(s)
#------------------------------------------------------

for i in range(0,len(STATES)):
	print "\n|--- STEP "+str(i)+"---|"
	print "Id State: " + str(STATES[i].getidNumber())
	print "Configuration: " + str(STATES[i].getConfiguration())
	CONFIGURATIONS.append(STATES[i].getConfiguration())
	print "Time Table:\n" + str(STATES[i].getTimeTable())

#time in which robot complete exploration
maxTF = 0
for i in range(0,len(STARTING_POS)):
	if shortest_time_matrix[CONFIGURATIONS[-1][i]][STARTING_POS[i]] > maxTF:
		maxTF = shortest_time_matrix[CONFIGURATIONS[-1][i]][STARTING_POS[i]] 
tF = int(max(STATES[-1].getTimeTable())) + maxTF

print "\n\n\n----------------------------------------------------"
print "RECAP"
print "STARTING POSITION: " + str(STARTING_POS)
print "\nFINAL CONFIGURATION LIST"
print CONFIGURATIONS
print "\nTIME ELAPSED"
print int(max(STATES[-1].getTimeTable()))
print "\nFINAL MOVE: robots will go from " + str(STATES[-1].getConfiguration()) + " back to " + str(STARTING_POS)
print "\ntime for moving from last point of the tour to STARTING_POS"
print str(CONFIGURATIONS[-1]) + " ---> " + str(STARTING_POS) + " : " + str(maxTF)
print "\n\nFINAL TIMESTAMP: " + str(tF)



print("\n\n\n---EXECUTION TIME: %s seconds ---\n" % (time.time() - start_time))