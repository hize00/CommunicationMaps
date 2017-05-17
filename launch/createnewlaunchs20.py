'''
Created on Apr 22, 2016

@author: banfi
'''

def modify_instance(rob, alg, env, run):
    
    fname = str(rob) + "_" + env + "_" + alg + "_" + str(run) + ".launch"
    f = open(fname, "r")
    lines = f.readlines()
    f.close()
    newlaunch = ""
    for i in range(len(lines)):
        cur_string = lines[i]
        if "comm_model_filename" in cur_string or "range" in cur_string:
            cur_string = cur_string.replace("50", "20")       
        newlaunch += cur_string
    
    fnamenew = str(rob) + "_" + env + "_" + alg + "_" + str(run) + "_20.launch"    
    f = open(fnamenew, "w")
    f.write(newlaunch)
    f.close()
    
    
if __name__ == '__main__':
    robots = [2,4,6]
    algorithms = {}
    algorithms[2] = ["max_variance", "multi2-2", "random"]
    algorithms[4] = ["max_variance", "multi2-2", "multi2-4", "random"]
    algorithms[6] = ["max_variance", "multi2-3", "multi2-6", "random"]
    environments = ["open", "offices"]
    runs = 5

    for rob in robots:
        for alg in algorithms[rob]:
            for env in environments:
                for run in range(runs):
                    modify_instance(rob, alg, env, run)
