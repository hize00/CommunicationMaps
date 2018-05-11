import gflags
import matplotlib.pyplot as plt
import pylab as pl
import matplotlib.cm as cm
import numpy as np
import time
import sys

import cv2
from igraph import *
from scipy.spatial import Voronoi, voronoi_plot_2d

#from shapely.geometry import LineString, MultiPolygon, MultiPoint, Point
#from skimage.morphology import medial_axis
#from skimage import img_as_bool, io, color, morphology
#from rdp import rdp

from utils import get_graphs_and_image_from_files, eucl_dist


gflags.DEFINE_string('exp_name', 'provaC', 'name of the experiment to be written as .exp file')
gflags.DEFINE_string('phys_graph', 'offices_phys_uniform_grid.graphml', 'file containing the physical graph')

gflags.DEFINE_string('file_path', '../envs/offices.png', 'png file path')
gflags.DEFINE_integer('sel_grid_size', 12, 'pixels making 1 grid cell'
                      'for points selection')#offices = 12
                                             #open = 10

gflags.DEFINE_string('point_selection_policy', 'grid',
                     'policy for selecting points in an environment') #click,grid,voronoi

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

def is_grid_cell(im_array, i, j, rows, cols):
    for k in range(i, i + gflags.FLAGS.sel_grid_size):
        if k >= rows: return False

        for w in range(j, j + gflags.FLAGS.sel_grid_size):
            if w >= cols: return False

            if im_array[k][w] == 0: return False

    return True

def all_free(ii, jj, I, J, border=None):
	if(im_array[ii][jj] == 0): return False

	if border is None: return True

	for k in xrange(ii - border, ii + border + 1):
		if k < 0 or k >= I: return False
		for w in xrange(jj - border, jj + border + 1):
			if w < 0 or w >= J: return False
			if (im_array[k][w] == 0): return False

	return True

def environment_discretization():
    print 'Creating grid physical graph with a different cell size for points selection ...'

    #im = cv2.imread(gflags.FLAGS.file_path)
    #im_array = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

    rows = np.size(im_array, 0)
    cols = np.size(im_array, 1)

    graph = Graph(directed=False)
    curr_id = 0
    bu = gflags.FLAGS.sel_grid_size / 2

    for i in range(0, rows, gflags.FLAGS.sel_grid_size):
        for j in range(0, cols, gflags.FLAGS.sel_grid_size):
            if gflags.FLAGS.point_selection_policy != 'voronoi':
                if is_grid_cell(im_array, i, j, rows, cols):
                    graph.add_vertex()
                    graph.vs[curr_id]['y_coord'] = i + bu
                    graph.vs[curr_id]['x_coord'] = j + bu
                    curr_id += 1
            else:
                if not is_grid_cell(im_array, i, j, rows, cols):
                    graph.add_vertex()
                    graph.vs[curr_id]['y_coord'] = i + bu
                    graph.vs[curr_id]['x_coord'] = j + bu
                    curr_id += 1

    print 'Done. Number of vertices: ', len(graph.vs)

    return graph.vs

def grid_points_selection(I,J):
    global start
    global goal_config

    graph = environment_discretization()

    env_name = (os.path.splitext(gflags.FLAGS.file_path)[0]).split("/")[-1]
    if env_name == 'offices':
        wall_dist = 6
        coeff = 4
    elif env_name == 'open':
        wall_dist = 13  # open = 9,10,11,12
        coeff = 5
    else:
        wall_dist = 0  # keeping all the points
        coeff = 0

    points = []
    for i in xrange(0, len(graph)):
        points.append(graph[i]) #creating a list of vertices of the graph (for legibility)

    #removing points that are too close to each other or too close to an obstacle
    too_close = []
    for p1 in points:
        if p1 in too_close: continue
        if not all_free(int(p1['y_coord']), int(p1['x_coord']), I, J, wall_dist):
            too_close.append(p1)
        else:
            for p2 in points:
                if p1 == p2 or p2 in too_close: continue
                x1 = p1['x_coord']
                y1 = p1['y_coord']
                x2 = p2['x_coord']
                y2 = p2['y_coord']
                if eucl_dist((x1, y1), (x2, y2)) < coeff * gflags.FLAGS.sel_grid_size:
                    too_close.append(p2)

    points = [x for x in points if x not in too_close]

    for point in points:
        x1 = point['x_coord']
        y1 = point['y_coord']
        vertex_id = get_closest_vertex(x1, y1)
        if vertex_id not in goal_config:
            goal_config.append(vertex_id)

    write_exp_file()
    plot_plan(goal_config)

    print 'Done. Number of points: ' + str(len(goal_config))

def voronoi_points_selection(I,J):
    global start
    global goal_config

    graph = environment_discretization()

    graph_points = []
    for vertex in graph:
        x = vertex['x_coord']
        y = vertex['y_coord']
        graph_points.append((x, y))

    env_name = (os.path.splitext(gflags.FLAGS.file_path)[0]).split("/")[-1]
    if env_name == 'offices':
        wall_dist = 7
        coeff = 3
    elif env_name == 'open':
        wall_dist = 13
        coeff =  2.5 #or 3, according to the number of points I want
    else:
        wall_dist = 0  # keeping all the points
        coeff = 0

    # calculating Voronoi vertices
    voronoi = Voronoi(graph_points) # , qhull_options='Qbb Qc Qx')
    #voronoi_plot_2d(voronoi)

    voronoi_points = []
    for vertex in voronoi.vertices:
        voronoi_points.append((int(vertex[0]), int(vertex[1])))

    # removing points that are close to an obstacle
    to_remove = []
    for point in voronoi_points:
        if not all_free(point[1], point[0], I, J, wall_dist):
            to_remove.append(point)

    points = [x for x in voronoi_points if x not in to_remove]

    #keeping max_min distance points from the obstacles
    too_close = []
    for p1 in points:
        if p1 in too_close: continue
        min_dist = eucl_dist((p1[0], p1[1]), (to_remove[0][0], to_remove[0][1]))
        min_coords = to_remove[0]

        for p2 in to_remove:
            dist = eucl_dist((p1[0],p1[1]), (p2[0],p2[1]))
            if dist < min_dist:
                min_dist = dist
                min_coords = p2

        for p3 in points:
            if p1 == p3 or p3 in too_close: continue
            if eucl_dist((p3[0], p3[1]), (min_coords[0], min_coords[1])) < min_dist:
                too_close.append(p3)

    points = [x for x in points if x not in too_close]

    # removing points that too close to each other
    cluster = []
    for p1 in points:
        if p1 in cluster: continue
        for p2 in points:
            if p1 == p2: continue
            if eucl_dist((p1[0], p1[1]),(p2[0], p2[1])) < coeff * gflags.FLAGS.sel_grid_size:
                if p2 not in too_close:
                    cluster.append(p2)

    points = [x for x in points if x not in cluster]

    for point in points:
        vertex_id = get_closest_vertex(point[0], point[1])
        if vertex_id not in goal_config:
            goal_config.append(vertex_id)

    write_exp_file()
    plot_plan(goal_config)

    print 'Done. Number of points: ' + str(len(goal_config))


if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    G_E, im_array = get_graphs_and_image_from_files(gflags.FLAGS.phys_graph)

    I = np.size(im_array, 0)
    J = np.size(im_array, 1)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(im_array, cmap=cm.Greys_r)

    if gflags.FLAGS.point_selection_policy == 'click':
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
    elif gflags.FLAGS.point_selection_policy == 'grid':
        grid_points_selection(I,J)
    elif gflags.FLAGS.point_selection_policy == 'voronoi':
        voronoi_points_selection(I,J)
    else:
        print 'Point selection policy not valid.'

    cid = fig.canvas.mpl_connect('key_press_event', press)

    # data_x = [566, 511]
    # data_y = [258, 269]
    # ax.plot(data_x, data_y, 'or')

    plt.show()