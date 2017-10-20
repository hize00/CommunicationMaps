
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
RANGE = 100

comm_discr_types = ['range']

datfiles = []
for i in range(MIN_ROBOTS, MAX_ROBOTS+1):
    name_of_file = gflags.FLAGS.env_name + str(i) + "r_" + str(RANGE) + ".dat"
    datfiles.append(name_of_file)


obj_f = 'distance'
sorting = ['cardinality' , 'heuristic', 'objective']
alg = 'HBSS.py'

if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    #print 'Running experiments on environment ', gflags.FLAGS.env_name
    subdir = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    mydir = os.path.join(os.getcwd(), 'logs', subdir)

    file_to_open = "resultsD_HBSS_"+ gflags.FLAGS.env_name + "_" + str(RANGE) + ".txt"
    file = open(file_to_open, "w")

    for dat in datfiles:
        d = str(dat)
        o = str(obj_f)
        a = str(alg)
        if a == 'k3.py':
            for sort in sorting:
                s = str(sort)
                os.system("python " + a +' '+ d +' '+ o +' '+ s)
        else:
            os.system("python " + a +' '+ d +' '+ o )


    print "MAP: " + gflags.FLAGS.env_name + "\n" + "DATE: " + subdir 
    file.close()


