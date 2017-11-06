
import os
import sys
import datetime
import multiprocessing

ENVIRONMENT = 'bwopen0_'
#ENVIRONMENT = 'offices1'

RANGE = 1000

MIN_ROBOT = 9
MAX_ROBOT = 10
ALGORITHM = ['HBSS']
OBJECTIVE = 'time'
TAU_LIST = [5,7]
LAUNCH_DAT_FILES = []
#LAUNCH_PY_FILES = []

name_of_file = ENVIRONMENT + "10r_" + str(RANGE) + ".dat"
LAUNCH_DAT_FILES.append(name_of_file)

###REMAKE OF EXPERIMENTS FOR TIME 9 & 10 r
print LAUNCH_DAT_FILES

for tau in TAU_LIST:
	os.system("python " + "HBSS" + str(tau) + ".py" + " " + LAUNCH_DAT_FILES + " " + OBJECTIVE + " > resultsT_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_10r.txt" )
		#print("python " + "HBSS" + str(tau) + ".py" + " " + LAUNCH_DAT_FILES[i] + " " + OBJECTIVE + " > resultsT_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(i+2) + ".txt" )


#"python HBSS3 filedati time > resultsT_hbss_bwopen0_100_tau3_4r.txt"
