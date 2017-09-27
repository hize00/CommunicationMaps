'''
Created on Aug 4, 2017

@author: banfi
'''

from igraph import *
import matplotlib.cm as cm
import numpy as np
import cv2

def plot_plan(G_E, im_array, config, ax, color='bo'):
    ax.imshow(im_array, cmap=cm.Greys_r)

    for robot in range(len(config)):
        ax.plot([G_E.vs[config[robot]]['x_coord']],[G_E.vs[config[robot]]['y_coord']], color, markersize = 16)
        ax.annotate(str(robot), xy=(G_E.vs[config[robot]]['x_coord'], G_E.vs[config[robot]]['y_coord']), 
                    xytext=(G_E.vs[config[robot]]['x_coord'] - 6, G_E.vs[config[robot]]['y_coord'] + 8), color='w')


    ax.set_xlim([0, np.size(im_array,1)])
    ax.set_ylim([np.size(im_array,0), 0])

def get_graphs_and_image_from_files(phys_graph_file):
    G_E = None
    im_array = None

    phys_graph_path = '../data/' + phys_graph_file
    G_E = read(phys_graph_path, format='graphml')

    image_path = '../envs/' + phys_graph_file.split('_')[0] + '.png'
    im = cv2.imread(image_path)
    im_array = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


    if G_E is None or im_array is None:
        print 'Error while reading graphs! Aborting.'
        exit(1)

    return G_E, im_array

def get_graphs_and_image_from_exp(exp_file):
    f = open(exp_file, 'r')
    lines = f.readlines()

    G_E = None
    im_array = None

    for line in lines:
        s = line.split()
        if s[0] == 'phys_graph':
            phys_graph_path = '../data/' + s[1]
            G_E = read(phys_graph_path, format='graphml')

            image_path = '../envs/' + s[1].split('_')[0] + '.png'
            im = cv2.imread(image_path)
            im_array = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    f.close()

    if G_E is None or im_array is None:
        print 'Error while reading graphs! Aborting.'
        exit(1)

    return G_E, im_array
