
import os
import sys
import datetime
import multiprocessing

ENVIRONMENT = 'offices1_'

RANGE = 500

MIN_ROBOT = 2
MAX_ROBOT = 10
nrob = 5
ALGORITHM = ['HBSS']
TAU_LIST = [3,5,7,9]
OBJECTIVE = 'distance'

LAUNCH_DAT_FILES = []
#LAUNCH_PY_FILES = []

name_of_file = ENVIRONMENT + str(nrob) + "r_" + str(RANGE) + ".dat"


for tau in TAU_LIST:
		os.system("python " + "HBSS" + str(tau) + ".py" + " " + name_of_file + " " + OBJECTIVE + " > resultsD_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(nrob) + "r.txt" )
		#print("python " + "HBSS" + str(tau) + ".py" + " " + name_of_file + " " + OBJECTIVE + " > resultsD_hbss_" + ENVIRONMENT + str(RANGE) + "_tau" + str(tau) + "_" + str(nrob) + "r.txt" )


#"python HBSS3 filedati distance > resultsD_hbss_bwopen0_100_tau3_4r.txt"
