import random
import numpy as np
import sys

#CREATE DAT FILES READING FROM 'provaC_parsed.dat' or 'provaC_bw_open_parsed.dat'

#SETTING PARAMETERS
MIN_ROBOTS = 2
MAX_ROBOTS = 10
NUMBER_OF_EXP = 20
POINTS = 44
RANGE = 100
START_POS = []

#START_POS = 0
START_POS = 42

for i in range(MIN_ROBOTS, MAX_ROBOTS+1):
	readingpoints = 0
	readingmatrix = 0
	
	with open("provaC_parsed.dat", "r") as f, open("offices1_" + str(i) +"r_" + str(RANGE) + ".dat" , "w") as out:
	#with open("provaC_bwopen_parsed.dat", "r") as f, open("bwopen0_" + str(i) +"r_" + str(RANGE) + ".dat" , "w") as out:
			data = f.readlines()
			for line in data:
				words = line.split()
				if len(words)>0:

					if words[0]=='RANGE_DISTANCE':
						RANGE_DISTANCE = RANGE
						out.write("RANGE_DISTANCE = " + str(RANGE_DISTANCE) + "\n;\n")

						out.write("\nN_ROBOTS = " + str(i) + "\n;\n")

						out.write("\nSTART =" + "\n")
						for j in range(0, i):
							out.write(str(START_POS)+"\n")
						out.write(";\n")

					elif words[0]=='VERTEXES':
						N_VERT = int(words[2])
						out.write("\nVERTEXES = " + str(N_VERT) + "\n;\n")
						continue

					elif words[0]=='VELOCITY':
						R_VELOCITY = int(words[2])
						out.write("\nVELOCITY = " + str(R_VELOCITY) + "\n;\n")
						continue
					
					elif words[0]=='POINTS_TO_EXPLORE':
						readingpoints = 1
						out.write("\nPOINTS_TO_EXPLORE = " + "\n")
						continue

					elif readingpoints:
						minilist = []
						if words[0]!=';':
							minilist.append(int(words[0]))
							minilist.append(int(words[1]))
							out.write(str(minilist[0]) + " " + str(minilist[1]) + "\n")
						else:
							readingpoints = 0
							out.write(";\n")

					elif words[0]=='DISTANCE_MATRIX':
						readingmatrix = 1
						out.write("\nDISTANCE_MATRIX = " + "\n")
						distance_matrix =  np.zeros((N_VERT, N_VERT))
						continue

					elif readingmatrix==1:
						if words[0]!=';':
							out.write(str(words[0]) + " " + str(words[1]) + "  " + str(words[2]) + "\n")
						else:
							readingmatrix = 0
							out.write(";\n")

	f.close()
	out.close()
