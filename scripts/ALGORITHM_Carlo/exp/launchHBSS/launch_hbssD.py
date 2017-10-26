
import os
import sys
import gflags
import datetime
import multiprocessing
from joblib import Parallel, delayed


#gflags.DEFINE_string('env_name', 'offices1_', 'environment name')
gflags.DEFINE_string('env_name', 'bwopen0_', 'environment name')
gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')

tau_to_compute = sys.argv[1]
n_robot = sys.argv[2]

RANGE = 100

name_of_file = gflags.FLAGS.env_name + str(n_robot) + "r_" + str(RANGE) + ".dat"

obj_f = 'distance'
alg = 'HBSS' + str(tau_to_compute) + '.py'

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    #print 'Running experiments on environment ', gflags.FLAGS.env_name
    subdir = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    mydir = os.path.join(os.getcwd(), 'logs', subdir)

    d = str(name_of_file)
    o = str(obj_f)
    a = str(alg)
    os.system("python " + a +' '+ d +' '+ o )


    print "MAP: " + gflags.FLAGS.env_name + "\n" + "DATE: " + subdir 


