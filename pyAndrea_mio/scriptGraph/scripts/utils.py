'''
Created on Aug 4, 2017

@author: banfi
'''

from igraph import *
import matplotlib.cm as cm
import numpy as np
import cv2
import math

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

def conv_to_hash(x, y):
    return (int(round(x*1000)), int(round(y*1000)))

def get_index_from_coord(env, c):
    return env.coord_index_map[conv_to_hash(*c)]

def eucl_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def directLinePossibleBresenham(start, end, im_array):
    #print start
    #print end
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        
        if(im_array[coord] == 0): return False
        
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    return True


 
