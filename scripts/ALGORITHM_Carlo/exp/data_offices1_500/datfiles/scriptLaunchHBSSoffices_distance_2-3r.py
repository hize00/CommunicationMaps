
import os
import sys
import datetime
import multiprocessing

ENVIRONMENT = 'offices1_'

RANGE = 500

MIN_ROBOT = 2
MAX_ROBOT = 10
nrob_list = [2,3]
ALGORITHM = ['HBSS']
tau = 5
OBJECTIVE = 'distance'


os.system("python HBSS5.py offices1_2r_500.dat distance > resultsD_hbss_offices1_500_tau5_2r.txt")
os.system("python HBSS5.py offices1_3r_500.dat distance > resultsD_hbss_offices1_500_tau5_3r.txt")

#print "python HBSS5.py offices1_2r_500.dat distance > resultsD_hbss_offices1_500_tau5_2r.txt"
#print "python HBSS5.py offices1_3r_500.dat distance > resultsD_hbss_offices1_500_tau5_3r.txt"

#"python HBSS3 filedati distance > resultsD_hbss_bwopen0_100_tau3_4r.txt"
