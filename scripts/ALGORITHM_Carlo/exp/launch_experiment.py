
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
N_EXP = 20

comm_discr_types = ['range']

datfiles = []
for i in range(MIN_ROBOTS, MAX_ROBOTS+1):
    for j in range(0,N_EXP):
        name_of_file = gflags.FLAGS.env_name + str(i) + "r_" + str(RANGE) + "_" + str(j) + ".dat"
        datfiles.append(name_of_file)


obj_functions = ['time', 'distance']
algorithms = ['k2.py', 'k3.py', 'HBSS.py']
sorting = ['cardinality' , 'heuristic', 'objective']


if __name__ == "__main__":
    """
        To be launched from the project main folder.
    """
    #print 'Running experiments on environment ', gflags.FLAGS.env_name
    subdir = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    mydir = os.path.join(os.getcwd(), 'logs', subdir)
    #os.mkdir(mydir)
    #save_path = mydir
    #name_of_file = gflags.FLAGS.env_name + "_" + subdir
    #completeName = os.path.join(save_path, name_of_file+".txt")         
    #file = open(completeName, "w")
    


    #file_to_open = "resultsT_"+ gflags.FLAGS.env_name + "_" + str(RANGE) +".txt"
    #file = open(file_to_open, "w")
    #obj_f = obj_functions[0]
    file_to_open = "resultsD_"+ gflags.FLAGS.env_name + "_" + str(RANGE) + ".txt"
    file = open(file_to_open, "w")
    obj_f = obj_functions[1]

    for dat in datfiles:
        d = str(dat)
        o = str(obj_f)
        for alg in algorithms:
            a = str(alg)
            if a == 'k3.py':
                for sort in sorting:
                    s = str(sort)
                    os.system("python " + a +' '+ d +' '+ o +' '+ s)
            else:
                os.system("python " + a +' '+ d +' '+ o )


    print "MAP: " + gflags.FLAGS.env_name + "\n" + "DATE: " + subdir 
    file.close()


