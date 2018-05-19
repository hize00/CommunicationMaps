import math
import cv2
import numpy as np
from igraph import *
import matplotlib.pyplot as plt
import sys

import gflags

gflags.DEFINE_string('file_path', '../envs/open.png', 'png file path')

gflags.DEFINE_string('phys_discr_type', 'uniform_grid', 'environment discretization - physical')
gflags.DEFINE_integer('cell_size', 7, 'pixels making 1 grid cell '
                                      '(only for uniform grid discretization)') #11 offices, 7 open

gflags.DEFINE_string('output_phys', '../data/open_phys', 'physical graph output')

gflags.DEFINE_bool('debug', True, 'debug mode active')

def is_grid_cell(im_array, i, j, rows, cols):
    for k in range(i, i + gflags.FLAGS.cell_size):
        if k >= rows: return False

        for w in range(j, j + gflags.FLAGS.cell_size):
            if w >= cols: return False

            if im_array[k][w] == 0: return False

    return True

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

def create_phys_graph_grid():
    print 'Creating grid physical graph...'
    im = cv2.imread(gflags.FLAGS.file_path)
    im_array = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    
    rows = np.size(im_array,0)
    cols = np.size(im_array,1)

    G_E = Graph(directed=False)
    curr_id = 0
    bu = gflags.FLAGS.cell_size/2
    for i in range(0, rows, gflags.FLAGS.cell_size):
        for j in range(0, cols, gflags.FLAGS.cell_size):
            if is_grid_cell(im_array, i,j, rows, cols):
                G_E.add_vertex()
                G_E.vs[curr_id]['y_coord'] = i + bu
                G_E.vs[curr_id]['x_coord'] = j + bu
                curr_id += 1

    neighbors = []
    for vertex_id in range(curr_id):

        curr_x = G_E.vs[vertex_id]['x_coord']
        curr_y = G_E.vs[vertex_id]['y_coord']

        up = G_E.vs.select(x_coord_eq=curr_x, y_coord_eq=(curr_y + gflags.FLAGS.cell_size))
        if len(up):
            neighbors.append((vertex_id, up[0].index))

        right = G_E.vs.select(x_coord_eq=(curr_x + gflags.FLAGS.cell_size), y_coord_eq=curr_y)
        if len(right):
            neighbors.append((vertex_id, right[0].index))

    G_E.add_edges(neighbors)
    print 'Done. Number of vertices: ', len(G_E.vs)

    if gflags.FLAGS.debug:
        plt.imshow(im_array)
        for edge in G_E.es:
            v1 = G_E.vs[edge.source]
            v2 = G_E.vs[edge.target]
            plt.plot([v1['x_coord'] ,v2['x_coord']],[v1['y_coord'], v2['y_coord']],'b')
        plt.show()

    return G_E,im_array



if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    if gflags.FLAGS.phys_discr_type == 'uniform_grid':
        G_E, im_array = create_phys_graph_grid()
    else:
        print 'Error! Physical discretization not supported!'
        exit(1)

    G_E.write(gflags.FLAGS.output_phys + '_' + gflags.FLAGS.phys_discr_type + '.graphml', format='graphml')




