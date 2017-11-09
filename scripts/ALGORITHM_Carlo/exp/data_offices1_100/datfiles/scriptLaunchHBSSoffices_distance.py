
import os
import sys
import datetime
import multiprocessing

ENVIRONMENT = 'offices1_'

RANGE = 100

MIN_ROBOT = 2
MAX_ROBOT = 10
ALGORITHM = ['HBSS']
TAU_LIST = [3,5,7,9]
OBJECTIVE = 'distance'

LAUNCH_DAT_FILES = []
#LAUNCH_PY_FILES = []

for i in range(MIN_ROBOT, MAX_ROBOT+1):
	name_of_file = ENVIRONMENT + str(i) + "r_" + str(RANGE) + ".dat"
	LAUNCH_DAT_FILES.append(name_of_file)

print LAUNCH_DAT_FILES
#LAUNCH_DAT_FILES from 2r to 10r

for i in range(0,len(LAUNCH_DAT_FILES)):
	#tau 3,5,7,9
	for tau in TAU_LIST:
		os.system("python " + "HBSS" + str(tau) + ".py" + " " + LAUNCH_DAT_FILES[i] + " " + OBJECTIVE + " > resultsD_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(i+2) + "r.txt" )
		#print("python " + "HBSS" + str(tau) + ".py" + " " + LAUNCH_DAT_FILES[i] + " " + OBJECTIVE + " > resultsD_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(i+2) + "r.txt" )


#"python HBSS3 filedati distance > resultsD_hbss_bwopen0_100_tau3_4r.txt"