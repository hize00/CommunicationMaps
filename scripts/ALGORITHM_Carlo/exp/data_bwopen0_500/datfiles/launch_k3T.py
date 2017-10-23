
import os
import sys
import gflags
import datetime
import multiprocessing
from joblib import Parallel, delayed


#gflags.DEFINE_string('env_name', 'offices1_', 'environment name')
gflags.DEFINE_string('env_name', 'bwopen0_', 'environment name')
gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')

MIN_ROBOTS = 2
MAX_ROBOTS = 10
RANGE = 500

comm_discr_types = ['range']


obj_f = 'time'
sorting = ['cardinality' , 'heuristic', 'objective']
alg = 'k3.py'
dat = gflags.FLAGS.env_name + '3r_' + str(RANGE) + '.dat'

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    #print 'Running experiments on environment ', gflags.FLAGS.env_name
    subdir = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    mydir = os.path.join(os.getcwd(), 'logs', subdir)

    file_to_open = "resultsT_k3_"+ gflags.FLAGS.env_name + "_" + str(RANGE) +".txt"
    file = open(file_to_open, "w")


    d = str(dat)
    o = str(obj_f)
    a = str(alg)
    for sort in sorting:
        s = str(sort)
        os.system("python " + a +' '+ d +' '+ o +' '+ s)


    print "MAP: " + gflags.FLAGS.env_name + "\n" + "DATE: " + subdir 
    file.close()


