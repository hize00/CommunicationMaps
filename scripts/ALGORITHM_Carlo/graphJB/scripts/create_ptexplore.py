import gflags
import matplotlib.pyplot as plt
import pylab as pl
import matplotlib.cm as cm
import numpy as np
import time
import sys

from utils import get_graphs_and_image_from_files

gflags.DEFINE_string('exp_name', 'provaC', 'name of the experiment to be written as .exp file')
gflags.DEFINE_string('phys_graph', 'offices_phys_uniform_grid.graphml', 'file containing the physical graph')

goal_config = []
start = True
G_E = None
im_array = None


def plot_plan(config):
    color = 'bo' 

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

    goal_string = ""
    for v in goal_config:
        goal_string += str(v) + '\n'
    f.write('GRAPH_POINTS \n' + goal_string + ';' + '\n')
    f.close()

def onclick(event):
    global start
    global goal_config

    x = event.xdata
    y = event.ydata
    vertex_id = get_closest_vertex(x, y)
    
    goal_config.append(vertex_id)
    plot_plan(goal_config)
    write_exp_file()

def press(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'q':
        plt.close()

if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    G_E, im_array = get_graphs_and_image_from_files(gflags.FLAGS.phys_graph)


    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(im_array, cmap=cm.Greys_r)
    #cid = fig.canvas.mpl_connect('button_press_event', onclick)
    cid = fig.canvas.mpl_connect('key_press_event', press)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # data_x = [566, 511]
    # data_y = [258, 269]
    # ax.plot(data_x, data_y, 'or')

    plt.show()

    


