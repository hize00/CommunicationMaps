
import os
import sys
import gflags
import datetime
import multiprocessing
from joblib import Parallel, delayed


gflags.DEFINE_string('env_name', 'offices', 'environment name')
gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')

comm_discr_types = ['range']

datfiles = ['offices0_15r_11gd_19v_2robots_0.dat', 'offices0_15r_11gd_19v_2robots_1.dat', 'offices0_15r_11gd_19v_2robots_2.dat', 
			'offices0_15r_11gd_19v_2robots_3.dat', 'offices0_15r_11gd_19v_2robots_4.dat', 'offices0_15r_11gd_19v_3robots_0.dat',
			'offices0_15r_11gd_19v_3robots_1.dat', 'offices0_15r_11gd_19v_3robots_2.dat', 'offices0_15r_11gd_19v_3robots_3.dat',
			'offices0_15r_11gd_19v_3robots_4.dat']
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
    file = open("results.txt", "w")

    for dat in datfiles:
    	d = str(dat)
    	for obj_f in obj_functions:
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


