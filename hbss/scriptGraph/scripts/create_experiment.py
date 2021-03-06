'''
Created on Aug 4, 2017

@author: banfi
'''
import gflags
import matplotlib.pyplot as plt
import pylab as pl
import matplotlib.cm as cm
import numpy as np
import time
import sys

from utils import get_graphs_and_image_from_files

gflags.DEFINE_string('exp_name', 'prova3', 'name of the experiment to be written as .exp file')
gflags.DEFINE_string('phys_graph', 'offices_phys_uniform_grid.graphml', 'file containing the physical graph')
gflags.DEFINE_integer('n_robots', 2, 'number of robots of the experiment')

start_config = []
goal_config = []
start = True
G_E = None
im_array = None

def plot_plan(config):
    color = 'bo' if start else 'ro'

    for robot in range(len(config)):
        ax.plot([G_E.vs[config[robot]]['x_coord']],[G_E.vs[config[robot]]['y_coord']], color, markersize = 16)
        ax.annotate(str(robot), xy=(G_E.vs[config[robot]]['x_coord'], G_E.vs[config[robot]]['y_coord']), 
                    xytext=(G_E.vs[config[robot]]['x_coord'] - 6, G_E.vs[config[robot]]['y_coord'] + 8), color='w')

    ax.set_xlim([0, np.size(im_array,1)])
    ax.set_ylim([np.size(im_array,0), 0])

    pl.draw()

def get_closest_vertex(xd, yd):
    return min(range(len(G_E.vs)), key=lambda x: (G_E.vs[x]['y_coord'] - yd)**2 + (G_E.vs[x]['x_coord'] - xd)**2)

def write_exp_file():
    f = open('../data/' + gflags.FLAGS.exp_name + '.exp', 'w')
    f.write('phys_graph ' + gflags.FLAGS.phys_graph + '\n')
    start_string = ""
    for v in start_config:
        start_string += str(v) + ' '
    goal_string = ""
    for v in goal_config:
        goal_string += str(v) + ' '
    f.write('start ' + start_string + '\n')
    f.write('goal ' + goal_string + '\n')
    f.close()
    exit(1)

def onclick(event):
    global start
    global start_config
    global goal_config

    x = event.xdata
    y = event.ydata
    vertex_id = get_closest_vertex(x, y)
    if start:
        start_config.append(vertex_id)
        plot_plan(start_config)
        if len(start_config) == gflags.FLAGS.n_robots:
            start = False
    else:
        goal_config.append(vertex_id)
        plot_plan(goal_config)
        if len(goal_config) == gflags.FLAGS.n_robots:
            time.sleep(2)
            write_exp_file()

if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    G_E, im_array = get_graphs_and_image_from_files(gflags.FLAGS.phys_graph)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(im_array, cmap=cm.Greys_r)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    


