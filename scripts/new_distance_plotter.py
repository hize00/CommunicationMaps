import numpy as np
import os
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
from matplotlib import cm


ENVIRONMENT = 'offices'
NROBOTS = 4
NUM_RUNS = 3
RANGE = 50

GRANULARITY = 300 #secs
MISSION_DURATION = 5830 #secs

plot_format = {'graph': ['b--s', 'AC']}


def parse_logfile(filename):
    distances = []
    f = open(filename, "r")
    lines = f.readlines()
    for line in lines:
        s = line.split()
        if s[0] == 'D':
            distances.append((float(s[1]), float(s[2])))

    if abs(distances[-1][1] - distances[int(3*len(distances)/4.0)][1]) <= 0.1:
        print "WARNING: " + filename

    return distances

def plot_values(x_vals, y, yerr, ylabel, filename):
    fig, ax = plt.subplots()

    for key in plot_format.keys():
        plt.errorbar(x_vals, y, yerr, fmt=plot_format[key][0],label=plot_format[key][1], markersize=10, elinewidth=2)

    plt.legend(fontsize=20, loc=2)
    plt.xlim(x_vals[0]-0.5, x_vals[-1] + 0.5)
    plt.ylim(0,1000)
    plt.ylabel(ylabel, fontsize=22)
    plt.tick_params(labelsize=20)
    plt.xlabel("Time (minutes)", fontsize=22)
    
    os.environ['PATH'] = os.environ['PATH'] + 'YOURLATEXPATH'
    matplotlib.rcParams['ps.useafm'] = True
    matplotlib.rcParams['pdf.use14corefonts'] = True
    matplotlib.rcParams['text.usetex'] = True
    fig.savefig(filename, bbox_inches='tight')
    

def plot(distances):

    x = range(int(GRANULARITY/60.0), int(MISSION_DURATION/60.0 + 1), 5)

    distances_avg = []
    distances_yerr = []

    for stamp in range(len(x)):
        cur_distances_avg = 0.0
        cur_distances_values = []

        for run in distances.keys():
            cur_distances_avg += distances[run][stamp]
            cur_distances_values.append(distances[run][stamp])

        cur_distances_avg = cur_distances_avg/len(distances.keys())

        distances_avg.append(cur_distances_avg)
        distances_yerr.append(np.std(cur_distances_values))

    plot_values(x, distances_avg, distances_yerr, "Avg. Distance", os.getcwd() + '/figs/DISTANCE_' +
                str(NROBOTS) + '_' + ENVIRONMENT + '_' + str(RANGE) + '.pdf')

def retrieve_closest_dist(data, secs):
    if data[-1][0] <= secs: return data[-1][1]
    closest = data[0]
    for i in xrange(1,len(data)):
        #print data[i]
        if data[i][0] <= secs and data[i + 1][0] > secs:
            closest = data[i]
            return closest[1] # the distance

    return closest[1]

if __name__ == '__main__':

    os.chdir("/home/andrea/catkin_ws/src/strategy/")

    runs = range(NUM_RUNS)

    average_dist = {}

    for run in runs:
        robot_distances = {}

        for robot in range(NROBOTS):
            log_filename = os.getcwd() + '/log/' + str(run) + '_' \
                           + ENVIRONMENT + '_' + str(robot) + '_' + str(NROBOTS) + '_' + str(RANGE) + '.log'
            robot_distances[robot] =  parse_logfile(log_filename)

        average_dist[run] = []

        for secs in xrange(GRANULARITY, MISSION_DURATION + 1, GRANULARITY):
            avg_dist_cur_time = 0.0
            for robot in range(NROBOTS):
                avg_dist_cur_time += retrieve_closest_dist(robot_distances[robot], secs)

            average_dist[run].append(avg_dist_cur_time/NROBOTS)

    plot(average_dist)