import os
import time

nrobots_all = ["2"]
methods = ["multi2-2","max_variance","random"]
runs = range(5)
envs = ["open"] #open

def num_errors():
    f = open("log/errors.log", "r")
    lines = f.readlines()
    f.close()
    num = len(lines)
    return num

for nrobots in nrobots_all:
    for method in methods:
        for run in runs:
            for env in envs:
                errors_before = num_errors()
                print "Errors before: ", errors_before
                command = "roslaunch strategy " + nrobots + "_" + env + "_" + method + "_" + str(run) + ".launch"
                start = time.time()
                os.system(command)
                elapsed = time.time() - start
                errors_after = num_errors()
                print "Errors after: ", errors_after
                if elapsed < 1780 and errors_before  == errors_after:
                    print "BUG AT ", nrobots, method, run, env
                    exit(1) 
                time.sleep(25)

#DONE:
#50
#2 robots
#offices DONE
#open

#4 robots
#offices DONE
#open DONE

#6 robots
#offices
#open

#PARAMS: dists: 25- 20- 15, multi2 samples: 3, 2, 1        
