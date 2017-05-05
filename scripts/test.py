from copy import deepcopy

#######OPERAZIONI STUPIDE SU DIZIONARI#########
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
#################################################

#CONFIGURAZIONE INIZIALE -- DA FARE BENE con parsing ecc
C0 = {'1':2,
	  '2':5,
	  '3':7,
	  '4':10
	  }
print C0

#temporary configurations needed when moving between non-adjacent vertexes
temporaryConfigurations = {}

#list of timestamp and correlated configurations
OUTPUT = {}

### R(C): ritorna i robots presenti in configurazione C
def robotsInConfiguration(configuration):
	return sorted(configuration.keys())
#print "\nC0 has ", robotsInConfiguration(C0)

### C(r): vertex in which robot r is in configuration C. Se r non e' presente ritorna None 
def robotVertex(robot,configuration):
	if robot in configuration.keys():
		return configuration.get(robot)


### adiacenza tra due vertici
def adjacency(vertex1, vertex2):
	#if vertex1 directly connected to vertex 2:
		#return 1
	#else: return 0

#move robot in an adjacent vertex
def adjacentMove(robot,configuration,newVertex):
	#start time
	newConfiguration = deepcopy(configuration)
	newConfiguration.update({robot:newVertex})
	#quando robot a destinazione -> end time
	return newConfiguration	#e tempo


#move robot from configuration (if exists) to newVertex in newConfiguration
def moveRobot(robot,configuration,newVertex):
	if robot not in configuration.keys():
		print "Error: " + str(robot) + " not in configuration "+str(configuration)
	else:
		if (adjacency(robotVertex(robot,configuration),newVertex)==1):	#if robot is in a vertex adjacent to newVertex it moves there directly
			newConfiguration=adjacentMove(robot,configuration,newVertex)
		else:
			#compute SHORTEST PATH between the two vertexes. Return a list of nodes
			#path_nodes = shortestPath(robotVertex(robot,configuration), newVertex)

			#move robot in intermediate configurations (each step is an adjacent move)
			for i in path_nodes:
				#adjacentMove from robotVertex and first node on the list
				adjacentMove(robotVertex(robot,configuration),path_nodes[i])
				#adjusting indexes
				robotVertex(robot,configuration) = path_nodes[i]



		

		return #OUTPUT(i) quindi timestamp e configurazione

