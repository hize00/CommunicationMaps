
import os
import sys
import datetime
import multiprocessing

ENVIRONMENT = 'bwopen0_'
#ENVIRONMENT = 'offices1'

RANGE = 1000

MIN_ROBOT = 2
MAX_ROBOT = 10
ALGORITHM = ['HBSS']
OBJECTIVE = 'time'

LAUNCH_DAT_FILES = []
#LAUNCH_PY_FILES = []

for i in range(MIN_ROBOT, MAX_ROBOT+1):
	name_of_file = ENVIRONMENT + str(i) + "r_" + str(RANGE) + ".dat"
	LAUNCH_DAT_FILES.append(name_of_file)

print LAUNCH_DAT_FILES
#LAUNCH_DAT_FILES from 2r to 10r

for i in range(0,len(LAUNCH_DAT_FILES)):
	#tau from 1 to 7
	for tau in range(1,8):
		os.system("python " + "HBSS" + str(tau) + ".py" + " " + LAUNCH_DAT_FILES[i] + " " + OBJECTIVE + " > resultsT_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(i+2) + "r.txt" )
		#print("python " + "HBSS" + str(tau) + ".py" + " " + LAUNCH_DAT_FILES[i] + " " + OBJECTIVE + " > resultsT_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(i+2) + ".txt" )


#"python HBSS3 filedati time > resultsT_hbss_bwopen0_100_tau3_4r.txt"
