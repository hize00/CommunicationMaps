import math
import random
import time
import numpy as np
import sys
import itertools
from igraph import *
from operator import itemgetter
from copy import deepcopy

sys.setrecursionlimit(1000000)
#start the time
start_time = time.time()

file_to_open = sys.argv[1]
obj_fun = str(sys.argv[2])
# Parse argument
if len(sys.argv) < 3:
    print("Usage: python filename.py datafile.dat obj_fun('time' or 'distance')")
    exit(1)

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

#time matrix initialization
time_matrix =  np.zeros((N_VERTEXES, N_VERTEXES))
for i in range(0,N_VERTEXES):
	for j in range(0,N_VERTEXES):
		time_matrix[i][j] = distance_matrix[i][j] / ROBOT_VELOCITY

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

#Generic state of the tree. Parameters: idNumber,configuration, timeTable, robots moving, depth level, points to visit, father, children
class State(object):

	def __init__(self, idNumber, configuration, timeTable, robotMoving, depth_level, points_to_visit, father, children):
		self.idNumber = idNumber
		self.configuration = configuration
		self.timeTable = timeTable
		self.robotMoving = robotMoving
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
		print "| ROBOT MOVING |"
		print self.robotMoving
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

	def getRobotMoving(self):
		return self.robotMoving
	def setRobotMoving(self, rmoving):
		self.robotMoving = rmoving

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
robotMoving_ini =  [0]*N_ROBOTS

#ROOT State. Id , robots moving & depth_level = 0, STARTING_POS & POINTS_TO_EXPLORE are given by input, Children list is empty
root = State(0, STARTING_POS, timeTable_ini, robotMoving_ini, 0, POINTS_TO_EXPLORE, None, [])
STATES.append(root)

#root.infoState()

#return the maximum identification number of the already created states.
def maxId(stateList):
	maxNumber = 0
	for i in range (0, len(stateList)):
		if stateList[i].getidNumber() > maxNumber:
			maxNumber = stateList[i].getidNumber()
	return maxNumber

#return Moves by time or distance: [ [t][C_ini][C_end] [moving robots] [visited pair] ]
def computeMoves(configuration, points_to_visit, timetable):
	moves = []
	sortedMoves = []
	movingRobots = [0] * len(configuration) #1 for robots who are moving
	pair = [None, None]
	if obj_fun == "time":
		#for each couple of point in -configuration- I need to compute all possible moves to reach -points_to_visit-
		for i in range(0,len(configuration)-1):
			for j in range(i+1,len(configuration)):
				for k in range(0,len(points_to_visit)):
					#t is the time
					t = min( max(shortest_matrix[configuration[i]][points_to_visit[k][0]], shortest_matrix[configuration[j]][points_to_visit[k][1]]), max(shortest_matrix[configuration[i]][points_to_visit[k][1]], shortest_matrix[configuration[j]][points_to_visit[k][0]]))
					if (max(shortest_matrix[configuration[i]][points_to_visit[k][0]], shortest_matrix[configuration[j]][points_to_visit[k][1]])) <= max(shortest_matrix[configuration[i]][points_to_visit[k][1]], shortest_matrix[configuration[j]][points_to_visit[k][0]]):
						movingRobots = [0] * len(configuration)
						cM = list(configuration)
						cM[i] = points_to_visit[k][0]
						cM[j] = points_to_visit[k][1]
						movingRobots[i] = 1
						movingRobots[j] = 1
						pair[0] = points_to_visit[k][0]
						pair[1] = points_to_visit[k][1]
						t1 = shortest_matrix[configuration[i]][points_to_visit[k][0]]
						t2 = shortest_matrix[configuration[j]][points_to_visit[k][1]]
						t = max(timetable[i][0]+t1, timetable[j][0]+t2)
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
						t1 = shortest_matrix[configuration[i]][points_to_visit[k][1]]
						t2 = shortest_matrix[configuration[j]][points_to_visit[k][0]]
						t = max(timetable[i][0]+t1, timetable[j][0]+t2)
						moves.append((t,configuration,cM[:], movingRobots, [pair[:]]))

	elif obj_fun == "distance":
		#for each couple of point in -configuration- I need to compute all possible moves to reach -points_to_visit-
		for i in range(0,len(configuration)-1):
			for j in range(i+1,len(configuration)):
				for k in range(0,len(points_to_visit)):
					#t is the distance
					t = min( (shortest_matrix[configuration[i]][points_to_visit[k][0]] + shortest_matrix[configuration[j]][points_to_visit[k][1]]), (shortest_matrix[configuration[i]][points_to_visit[k][1]] + shortest_matrix[configuration[j]][points_to_visit[k][0]]))
					if (shortest_matrix[configuration[i]][points_to_visit[k][0]] + shortest_matrix[configuration[j]][points_to_visit[k][1]]) <= (shortest_matrix[configuration[i]][points_to_visit[k][1]] + shortest_matrix[configuration[j]][points_to_visit[k][0]]):
						movingRobots = [0] * len(configuration)
						cM = list(configuration)
						cM[i] = points_to_visit[k][0]
						cM[j] = points_to_visit[k][1]
						movingRobots[i] = 1
						movingRobots[j] = 1
						pair[0] = points_to_visit[k][0]
						pair[1] = points_to_visit[k][1]
						t1 = shortest_matrix[configuration[i]][points_to_visit[k][0]]
						t2 = shortest_matrix[configuration[j]][points_to_visit[k][1]]
						t = (timetable[i][0]+t1) + (timetable[j][0]+t2)
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
						t1 = shortest_matrix[configuration[i]][points_to_visit[k][1]]
						t2 = shortest_matrix[configuration[j]][points_to_visit[k][0]]
						t = (timetable[i][0]+t1) + (timetable[j][0]+t2)
						moves.append((t,configuration,cM[:], movingRobots, [pair[:]]))

	return moves

###EXPAND: given a State returns the list calculated by computeMoves
def EXPAND(state):
	LEx = []
	LEx = computeMoves(state.getConfiguration(), state.getPointsToVisit(), state.getTimeTable())
	return LEx

###CHOOSE: given a State and a heuristic chooses the move to make, so it returns an instance of the List calculated by computeMoves
def CHOOSE(state, heuristic):
	if heuristic == "greedy":
		move, move_index = GREEDY(state)
		return move
	else:
		print "heuristic doesn't exist"

def GREEDY(state):
	#list with moves computed by EXPAND
	LGe = []
	LGe = EXPAND(state)
	sList = []
	sList = sorted(LGe, key=itemgetter(0))
	greedy_index = 0
	chosen = sList[greedy_index]
	return chosen, greedy_index

#given a state and a move updates the timetable (for the child)
def updateTimeTable(state, move):
	tt = state.getTimeTable()
	if obj_fun == "time":
		#move[3] contains the robots moving
		for i in range(0,len(move[3])):
			#if 1 update the table
			if move[3][i] == 1:
				tt[i][0] = move[0]

	elif obj_fun == "distance":
		#move[3] contains the robots moving
		for i in range(0,len(move[3])):
			#if 1 update the table
			if move[3][i] == 1:
				tt[i][0] = tt[i][0] + shortest_matrix[move[1][i]][move[2][i]]

	return tt


#given a state and a move updates the pointovisit (for the child)
def updatePointToVisit(state, move):
	ppp = state.getPointsToVisit()
	#move[4] contains the pair visited
	pToDelete = move[4]
	#delete pToDelete
	ppp = [x for x in ppp if x not in pToDelete]
	return ppp

###GENERATE: given a state creates the child
def GENERATE(state, heuristic):
	m = CHOOSE(state, heuristic)
	ttable = updateTimeTable(state, m)
	ptv = updatePointToVisit(state, m)
	#m[2] is the configuration after a move, m[3] robots moving
	s = State(maxId(STATES)+1, m[2], ttable, m[3], state.getDepthLevel()+1, ptv, state, [])
	state.addChild(s)
	return s


#------------------------------------------------------
#greedy deterministica
state = deepcopy(root)
for i in range(0, len(POINTS_TO_EXPLORE)):
	s = GENERATE(state, "greedy")
	STATES.append(s)
	state = deepcopy(s)

for i in range(0,len(STATES)):
	CONFIGURATIONS.append(STATES[i].getConfiguration())
	
if obj_fun == "time":
	#time in which robot complete exploration
	maxTF = 0
	for i in range(0,len(STARTING_POS)):
		if shortest_matrix[CONFIGURATIONS[-1][i]][STARTING_POS[i]] > maxTF:
			maxTF = shortest_matrix[CONFIGURATIONS[-1][i]][STARTING_POS[i]]
			maxTF_index = i
	greedy_time = float(STATES[-1].getTimeTable()[maxTF_index]) + maxTF

elif obj_fun == "distance":
	#distance in which robot complete exploration
	greedy_time = 0
	for i in range(0, N_ROBOTS):
		greedy_time = greedy_time + STATES[-1].getTimeTable()[i] + shortest_matrix[CONFIGURATIONS[-1][i]][STARTING_POS[i]]

# |---------------------------|
# |		 HBSS ALGORITHM       |
# |---------------------------|

HB_CONFIGURATIONS = []
HB_CONFIGURATIONS_LIST = []
HB_STATES = []
HB_STATES_LIST = []
TIME_HBSEARCH = []
HB_TIME = []
HB_TIME_LIST = []
HB_MOVING = []
HB_MOVING_LIST = []


def HBBS(state0, heuristic, bias, bound, initialBest):
	resultS, resultT = HBBS_Search(state0, heuristic, bias, 0, bound)
	return resultS, resultT
	
def HBBS_Search(state, heuristic, bias, depth, bound):
	scores = []
	if state.getDepthLevel() < bound:
		#ordered child based on heuristic
		scores = SORT_heuristic(state, heuristic)
		length = len(scores)
		rank = [-1]*length
		weight = [-1]*length
		probability = [-1]*length
		total_weight = 0
		for i in range(0, len(scores)):
			rank[i] = i+1
			weight[i] = BIAS_FUNCTION(bias, rank[i])
			total_weight = total_weight + weight[i]
		for i in range(0, len(scores)):
			probability[i] = weight[i]/total_weight
		#probability selection
		child_index = np.random.choice(range(0, length), None, False, probability)
		#selected move according to BIAS and probability
		m = scores[child_index]
		ttable = updateTimeTable(state, m)
		ptv = updatePointToVisit(state, m)
		#create a child
		child = State(maxId(HB_STATES)+1, m[2], ttable, m[3], state.getDepthLevel()+1, ptv, state, [])
		state.addChild(child)
		HB_STATES.append(child)
		tt = deepcopy(child.getTimeTable())
		TIME_HBSEARCH.append(tt)
		depth = depth + 1
		HBBS_Search(child, heuristic, bias, depth, bound)

	return HB_STATES, TIME_HBSEARCH

#sort the moves computed previously according to the heuristic
def SORT_heuristic(state, heuristic):
	sortedMoves = []
	if heuristic == "GREEDY":
		sortedMoves = SORT_Greedy(state)
		return sortedMoves
	else:
		print "Heuristic doesn't exist"
		return

#Greedy sorting
def SORT_Greedy(state):
	LSG = []
	#all possible moves
	moves = EXPAND(state)
	LSG = sorted(moves, key=itemgetter(0))
	return LSG


#compute the BIAS
def BIAS_FUNCTION(bias, rank):
	w = 0
	if bias == "LOG":
		w = 1/math.log10(rank+1)
	elif bias == "EXP":
		w = math.exp(-rank)
	elif bias == "POLY1":
		w = math.pow(rank, -1)
	elif bias == "POLY2":
		w = math.pow(rank, -2)
	elif bias == "POLY3":
		w = math.pow(rank, -3)
	elif bias == "POLY4":
		w = math.pow(rank, -4)
	elif bias == "POLY5":
		w = math.pow(rank, -5)
	elif bias == "POLY6":
		w = math.pow(rank, -6)
	elif bias == "POLY7":
		w = math.pow(rank, -7)
	elif bias == "POLY9":
		w = math.pow(rank, -9)
	else:
		print "Unknown BIAS function"
	return w
	

TIMES = []

#run HBBS procedure N_ITERATIONS times
N_ITERATIONS = 200
for j in range (0, N_ITERATIONS):
	#empty the lists
	HB_STATES = [] 
	TIME_HBSEARCH = []
	HB_CONFIGURATIONS = []  
	HB_MOVING = []
	HBT = []
	HB_TIME = []
	HB_STATES.append(root)
	HB_STATES, HBT = HBBS(root, "GREEDY", "POLY9", len(POINTS_TO_EXPLORE), greedy_time)
	HBT.insert(0, np.zeros((N_ROBOTS, 1)))
	for i in range(0,len(HB_STATES)):
		HB_CONFIGURATIONS.append(HB_STATES[i].getConfiguration())
		HB_MOVING.append(HB_STATES[i].getRobotMoving())
		HB_TIME.append(HBT[i])
	T_exploration = 0
	#find the maximum time or distance in which the HBSS computation ends and add it to TIMES list
	if obj_fun == "time":
		for i in range(0,len(STARTING_POS)):
			if shortest_matrix[HB_CONFIGURATIONS[-1][i]][STARTING_POS[i]] + HB_STATES[-1].getTimeTable()[i] > T_exploration:
				T_exploration = shortest_matrix[HB_CONFIGURATIONS[-1][i]][STARTING_POS[i]] + HB_STATES[-1].getTimeTable()[i]
				T_exploration_index = i
		T_final =  float(T_exploration)
		TIMES.append(T_final)
	elif obj_fun == "distance":
		sumtime = 0
		for i in range(0,len(STARTING_POS)):
			sumtime = sumtime + shortest_matrix[HB_CONFIGURATIONS[-1][i]][STARTING_POS[i]] + HB_STATES[-1].getTimeTable()[i]
		TIMES.append(int(sumtime))


	#set timetable of root to 0
	tt = np.full((N_ROBOTS,1), 0)
	root.setTimeTable(tt)
	#list of list with all computations
	HB_CONFIGURATIONS_LIST.append(HB_CONFIGURATIONS)
	HB_MOVING_LIST.append(HB_MOVING)
	HB_STATES_LIST.append(HB_STATES)
	HB_TIME_LIST.append(HB_TIME)

bestTime = min(TIMES)

bestIndex = TIMES.index(bestTime)
#append STARTING POS
HB_CONFIGURATIONS_LIST[bestIndex].append(STARTING_POS)

#flatting timetable arraylist computed by HB_TIME_LIST
T_LIST = []
flat = []
for i in range(0, len(HB_TIME_LIST[bestIndex])):
	T_LIST.append(HB_TIME_LIST[bestIndex][i].tolist())
for i in range(0,len(T_LIST)):
	flattened = [val for sublist in T_LIST[i] for val in sublist]
	flat.append(flattened)

#adding the final robots that are moving and the final times
final_robot_moving = [0]*N_ROBOTS
final_times = [0]*N_ROBOTS
for i in range(0,len(STARTING_POS)):
	#at [-1] the configuration = START POS, at [-2] there's the 'real' last configuration
	if HB_CONFIGURATIONS_LIST[bestIndex][-2][i] != STARTING_POS[i]:
		final_robot_moving[i] = 1
	final_times[i] = flat[-1][i] + shortest_matrix[HB_CONFIGURATIONS_LIST[bestIndex][-2][i]][STARTING_POS[i]] 
HB_MOVING_LIST[bestIndex].append(final_robot_moving) 
flat.append(final_times)

print "DATFILE : " + str(file_to_open)

env = file_to_open[0:6]
print "ENVIRONMENT : " + str(env)

print "ALGORITHM : HBSS - TAU 9"

if file_to_open[0] == "b":
	if file_to_open[14] == '.':
		RANGE = file_to_open[11:14] #se 100 [11:15] se 1000
	else:
		RANGE = RANGE = file_to_open[11:15]
elif file_to_open[0] == "o":
	if file_to_open[15] == '.':
		RANGE = file_to_open[12:15]
	else:
		RANGE = RANGE = file_to_open[12:16]
		
print "RANGE : " + str(RANGE)

print "STARTING_POS : " + str(STARTING_POS[0])

print "N_ROBOTS : " + str(N_ROBOTS)

print "GREEDY SOLUTION : " + str(greedy_time)

if obj_fun == "time":
	o = "T"
else:
	o = "D"

print "COMPUTATIONS " + o + " :"
for i in range(0, len(TIMES)):
	print TIMES[i]
print ";"

if obj_fun == "time":
	print "HBBS bestT : " + str(bestTime)
elif obj_fun == "distance":
	print "HBBS sumD : " + str(bestTime)

print "BEST_CONFIGURATIONS : "
for i in range(0,len(HB_CONFIGURATIONS_LIST[bestIndex])):
	miniC = ""
	for j in range(0, N_ROBOTS):
		miniC = miniC + str(HB_CONFIGURATIONS_LIST[bestIndex][i][j]) + " "
	print miniC
print ";"
print "ROBOT MOVING BEST:"
for i in range(0,len(HB_MOVING_LIST[bestIndex])):
	miniRM = ""
	for j in range(0, N_ROBOTS):
		miniRM = miniRM + str(HB_MOVING_LIST[bestIndex][i][j]) + " "
	print miniRM
print ";"
print "TIMETABLE BEST: "
for i in range(0,len(flat)):
	miniT = ""
	for j in range(0, N_ROBOTS):
		miniT = miniT + str(flat[i][j]) + " "
	print miniT
print ";"

print("EXECUTION TIME HBSS: %s seconds \n" % (time.time() - start_time))
print "-------------------------------------------------------------------"
