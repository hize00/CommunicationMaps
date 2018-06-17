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

from skimage import img_as_bool, io, color, morphology, util

from utils import get_graphs_and_image_from_files, eucl_dist


gflags.DEFINE_string('exp_name', 'provaC', 'name of the experiment to be written as .exp file')
gflags.DEFINE_string('phys_graph', 'offices_phys_uniform_grid.graphml', 'file containing the physical graph')
gflags.DEFINE_string('file_path', '../envs/offices.png', 'png file path')
gflags.DEFINE_string('point_selection_policy', 'grid', 'policy for selecting points in an environment') #click,grid,voronoi

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

    if all_free(int(G_E.vs[vertex_id]['y_coord']), int(G_E.vs[vertex_id]['x_coord']), I, J, wall_dist):
        goal_config.append(vertex_id)
        plot_plan(goal_config)
        write_exp_file()
    else:
        print "Point too close to an obstacle."

def press(event):
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'q':
        plt.close()

def is_grid_cell(im_array, i, j, rows, cols):
    for k in range(i, i + sel_grid_size):
        if k >= rows: return False

        for w in range(j, j + sel_grid_size):
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
    rows = np.size(im_array, 0)
    cols = np.size(im_array, 1)

    graph = Graph(directed=False)
    curr_id = 0
    bu = sel_grid_size / 2

    for i in range(0, rows, sel_grid_size):
        for j in range(0, cols, sel_grid_size):
            if not is_grid_cell(im_array, i, j, rows, cols):
                graph.add_vertex()
                graph.vs[curr_id]['y_coord'] = i + bu
                graph.vs[curr_id]['x_coord'] = j + bu
                curr_id += 1

    return graph.vs

def check_vertex_dist_from_obstacle(vertex_id):
    #fixing the vertices that are too close to an obstacle
    if env_name == 'open' and (G_E.vs[vertex_id]['x_coord'] < 60 or G_E.vs[vertex_id]['x_coord'] > 700):
        return None #bad discretization of the environment

    if not all_free(int(G_E.vs[vertex_id]['y_coord']), int(G_E.vs[vertex_id]['x_coord']), I, J, wall_dist):
        for vertex in G_E.vs:
            if vertex == vertex_id: continue
            x1 = G_E.vs[vertex_id]['x_coord']
            y1 = G_E.vs[vertex_id]['y_coord']
            x2 = vertex['x_coord']
            y2 = vertex['y_coord']
            if eucl_dist((x1, y1), (x2, y2)) <= sel_grid_size and all_free(int(y2), int(x2), I, J, wall_dist):
                vertex_id = get_closest_vertex(x2, y2)
                return vertex_id

        return None #if no fixing vertex is found, no vertex will be added in the goal set

    return vertex_id

def grid_points_selection():
    global start
    global goal_config

    points = []
    for i in xrange(0, len(G_E.vs)):
        points.append(G_E.vs[i]) #creating a list of vertices of the graph (for legibility)

    #removing points that are too close to each other or too close to an obstacle
    too_close = []
    for p1 in points:
        if p1 in too_close: continue
        x1 = p1['x_coord']
        y1 = p1['y_coord']
        if not all_free(int(y1), int(x1), I, J, wall_dist):
            too_close.append(p1)
        else:
            for p2 in points:
                if p1 == p2 or p2 in too_close: continue
                x2 = p2['x_coord']
                y2 = p2['y_coord']
                if eucl_dist((x1, y1), (x2, y2)) < coeff * sel_grid_size:
                    too_close.append(p2)

    points = [x for x in points if x not in too_close]

    for point in points:
        x1 = point['x_coord']
        y1 = point['y_coord']
        vertex_id = get_closest_vertex(x1, y1)
        vertex_id = check_vertex_dist_from_obstacle(vertex_id)
        if vertex_id and vertex_id not in goal_config:
            goal_config.append(vertex_id)

    write_exp_file()
    plot_plan(goal_config)

    print 'Done. Number of points: ' + str(len(goal_config))

def draw_voronoi_from_image():
    image = img_as_bool(color.rgb2gray(io.imread(gflags.FLAGS.file_path)))
    out = morphology.medial_axis(image)
    #out = morphology.skeletonize(image)

    f, (ax0, ax1) = plt.subplots(1,2)
    ax0.imshow(image, cmap='gray', interpolation='nearest')
    ax0.set_axis_off()

    #cmap = plt.get_cmap('Reds')
    ax1.imshow(out,  cmap='binary', interpolation='nearest')
    ax1.contour(image, [0.5], colors='r')
    ax1.set_axis_off()
    plt.subplots_adjust(wspace=0)

    plt.savefig('voronoi_' + env_name, format='eps', dpi=1000, bbox_inches='tight')

def draw_voronoi_from_points(graph_points):
    voronoi_points = Voronoi(graph_points)
    voronoi_plot_2d(voronoi_points)
    plt.axis('off')
    plt.savefig('voronoi_vertexes_' + env_name, format='eps', dpi=1000, bbox_inches='tight')
    #plt.close('all')

    points = []
    for v in voronoi_points.vertices:
        if all_free(int(v[1]), int(v[0]), I, J, 5):
            points.append(v)

    vertices = []
    for point in points:
        vertex_id = get_closest_vertex(point[0], point[1])
        vertices.append(vertex_id)

    plot_plan(vertices)

def voronoi_points_selection():
    global start
    global goal_config

    graph = environment_discretization()

    graph_points = []
    for vertex in graph:
        x = vertex['x_coord']
        y = vertex['y_coord']
        graph_points.append((x,y))


    vertices = []
    for vertex in G_E.vs:
        x = vertex['x_coord']
        y = vertex['y_coord']
        vertices.append((x, y))

    #Calculating Voronoi skeleton by keeping max_min distance points from the obstacles
    too_close = []
    for p1 in vertices:
        if p1 in too_close: continue
        min_dist = eucl_dist((p1[0], p1[1]), (graph_points[0][0], graph_points[0][1]))
        min_coords = graph_points[0]

        for p2 in graph_points:
            dist = eucl_dist((p1[0],p1[1]), (p2[0],p2[1]))
            if dist < min_dist:
                min_dist = dist
                min_coords = p2

        for p3 in vertices:
            if p1 == p3 or p3 in too_close: continue
            if eucl_dist((p3[0], p3[1]), (min_coords[0], min_coords[1])) < min_dist:
                too_close.append(p3)

    points = [x for x in vertices if x not in too_close]

    # removing points that are too close to an obstacle
    to_remove = []
    for point in points:
        if not all_free(int(point[1]), int(point[0]), I, J, wall_dist):
            to_remove.append(point)

    points = [x for x in points if x not in to_remove]

    # removing points that too close to each other
    cluster = []
    for p1 in points:
        if p1 in cluster: continue
        for p2 in points:
            if p1 == p2: continue
            if eucl_dist((p1[0], p1[1]), (p2[0], p2[1])) < coeff * sel_grid_size:
                if p2 not in too_close:
                    cluster.append(p2)

    points = [x for x in points if x not in cluster]

    for point in points:
        vertex_id = get_closest_vertex(point[0], point[1])
        vertex_id = check_vertex_dist_from_obstacle(vertex_id)
        if vertex_id and vertex_id not in goal_config:
            goal_config.append(vertex_id)

    write_exp_file()
    plot_plan(goal_config)

    print 'Done. Number of points: ' + str(len(goal_config))

    #draw_voronoi_from_image()
    #draw_voronoi_from_points(graph_points)

if __name__ == "__main__":
    argv = gflags.FLAGS(sys.argv)
    G_E, im_array = get_graphs_and_image_from_files(gflags.FLAGS.phys_graph)
    env_name = (os.path.splitext(gflags.FLAGS.file_path)[0]).split("/")[-1]

    if env_name == 'offices':
        sel_grid_size = 11 #sel_grid_size = pixels making 1 grid cell (same as cell size in create_graph_from_png.py)

        if gflags.FLAGS.point_selection_policy == 'click':
            wall_dist = 6
        elif gflags.FLAGS.point_selection_policy == 'grid':
            wall_dist = 10
            coeff = 4
        elif gflags.FLAGS.point_selection_policy == 'voronoi':
            wall_dist = 10
            coeff = 3
        else:
            print 'Unknown selection policy'
    elif env_name == 'open':
        sel_grid_size = 7

        if gflags.FLAGS.point_selection_policy == 'click':
            wall_dist = 12
        elif gflags.FLAGS.point_selection_policy == 'grid':
            wall_dist = 18
            coeff = 4
        elif gflags.FLAGS.point_selection_policy == 'voronoi':
            wall_dist = 18
            coeff = 2
        else:
            print 'Unknown selection policy'
    else:
        print "Unknown environment"

    I = np.size(im_array, 0)
    J = np.size(im_array, 1)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(im_array, cmap=cm.Greys_r)

    if gflags.FLAGS.point_selection_policy == 'click':
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
    elif gflags.FLAGS.point_selection_policy == 'grid':
        grid_points_selection()
    elif gflags.FLAGS.point_selection_policy == 'voronoi':
        voronoi_points_selection()
    else:
        print 'Point selection policy not valid.'

    cid = fig.canvas.mpl_connect('key_press_event', press)

    # data_x = [566, 511]
    # data_y = [258, 269]
    # ax.plot(data_x, data_y, 'or')

    plt.show()