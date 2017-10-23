
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
RANGE = 250

comm_discr_types = ['range']


dat = gflags.FLAGS.env_name + '2r_' + str(RANGE) + '.dat'
obj_f = 'time'
sorting = ['cardinality' , 'heuristic', 'objective']
alg = 'k2.py'

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    #print 'Running experiments on environment ', gflags.FLAGS.env_name
    subdir = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    mydir = os.path.join(os.getcwd(), 'logs', subdir)

    file_to_open = "resultsD_k2_"+ gflags.FLAGS.env_name + "_" + str(RANGE) + ".txt"
    file = open(file_to_open, "w")

    d = str(dat)
    o = str(obj_f)
    a = str(alg)
    os.system("python " + a +' '+ d +' '+ o )

    print "MAP: " + gflags.FLAGS.env_name + "\n" + "DATE: " + subdir 
    file.close()


