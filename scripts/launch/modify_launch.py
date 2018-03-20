'''
Created on Apr 22, 2016

@author: banfi
'''
import os

def change_name(rob, alg, env, run):
    fname = str(rob) + "_" + env + "_" + alg + "_" + str(run) + "_20.launch"
    fname2 = str(rob) + "_" + env + "_" + alg + "_" + str(run) + "_15.launch"
    os.system("mv " + fname + " " + fname2)

def modify_instance(rob, alg, env, run):
    
    fname = str(rob) + "_" + env + "_" + alg + "_" + str(run) + ".launch"
    f = open(fname, "r")
    lines = f.readlines()
    f.close()
    newlaunch = ""
    for i in range(len(lines)):
        cur_line = lines[i]
        if "polling_signal_period" in cur_line:
            #cur_line = cur_line.replace('<param name="samples_follower_multi2_single" value="2" />','<param name="samples_follower_multi2_single" value="1" />')
            #cur_line = cur_line.replace('<param name="mindist_vertices_multi2" value="25" />','<param name="mindist_vertices_multi2" value="10" />')
            #cur_line = cur_line.replace('<param name="mindist_vertices_maxvar" value="25" />','<param name="mindist_vertices_maxvar" value="20" />')
            cur_line = cur_line.replace('<param name="polling_signal_period" value="10"/>','<param name="polling_signal_period" value="15"/>')
            #<param name="samples_follower_multi2_single" value="3" />
        
        newlaunch += cur_line

        #if i == 10:
            #newlaunch += '  <param name="comm_model_filename" value="$(find strategy)/data/comm_model_50.xml" /> \n'
            #newlaunch += '    <param name="samples_pairs_greedy" value="10000" />\n'
            #newlaunch += '    <param name="samples_leader_multi2" value="100" />\n'
            #newlaunch += '    <param name="samples_follower_multi2_single" value="3" />\n' if rob < 6 else '    <param name="samples_follower_multi2_single" value="1" />\n'
            #if rob < 6:
            #    newlaunch += '    <param name="mindist_vertices_multi2" value="25" /> <param name="mindist_vertices_maxvar" value="25" />\n'
            #else:
            #    if env == "offices" and alg == "max_variance":
            #        newlaunch += '    <param name="mindist_vertices_multi2" value="25" /> <param name="mindist_vertices_maxvar" value="25" />\n'
            #    else:
            #        newlaunch += '    <param name="mindist_vertices_multi2" value="15" /> <param name="mindist_vertices_maxvar" value="15" />\n'

        
    f = open(fname, "w")
    f.write(newlaunch)
    f.close()
    
    
if __name__ == '__main__':
    robots = [6]
    algorithms = {}
    algorithms[2] = ["random"]
    algorithms[4] = ["random"]
    algorithms[6] = ["random"]
    environments = ["offices","open"]
    runs = 5

    for rob in robots:
        for alg in algorithms[rob]:
            for env in environments:
                for run in range(runs):
                    #change_name(rob, alg, env, run)                 
                    modify_instance(rob, alg, env, run)
