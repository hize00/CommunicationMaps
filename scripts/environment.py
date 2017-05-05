import math
import os
import pickle
import random
import time

import cv2
from igraph import *
import matplotlib.pyplot as plt
import numpy as np
import rospy

import utils
from utils import conv_to_hash, eucl_dist
from communication import numObstaclesBetweenRobots
import simulated_models

WALL_DIST = 6 #in pixels

class Environment(object):
    def __init__(self, map_filename, disc_method, disc, resize_factor, 
        comm_range, simulated_comm_model="", initialize=False):
        """Constructor of class Environment.
        
        Args:
            map_filename (str): path to environment image.
            disc_method (str): how environment is discretized {grid}.
            disc (int): value to distance the locations that can be
                selected (pixels).
            resize_factor (float): size of a cell (meters).
            comm_range (float): range of communication (meters).
            simulated_comm_model (str): communication model to build
                a priori communication map based on environment.
            initialize (bool): if True, precompute offline selected
                locations according to simulated_comm_model.
        """
        im = cv2.imread(map_filename)    
        self.im_array = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)

        self.dimX = np.size(self.im_array,1)*resize_factor
        self.dimY = np.size(self.im_array,0)*resize_factor

        self.I = np.size(self.im_array,0) # Height.
        self.J = np.size(self.im_array,1) # Width.

        self.disc = disc
        self.resize_factor = resize_factor

        #candidate positions to have the robots sensing (x,y) in meters.
        self.free_positions = []

        # with an additional set of points needed to fastly compute a 
        # path (in meters).
        self.path_positions = []

        #the graph in which computing the path
        self.G_E = Graph(directed=False)
        self.coord_index_map = {}

        #the graph for checking line-of-sight comm
        self.G_C = Graph(directed=False)

        #shortest paths lengths
        self.sps_lengths = None

        if disc_method == 'grid':
            self.discretize_grid()

        rospy.loginfo('Computing shortest path lengths...')
        self.sps_lengths = self.G_E.shortest_paths_dijkstra()
        for i in xrange(len(self.sps_lengths)):
            for j in xrange(len(self.sps_lengths[i])):
                self.sps_lengths[i][j] = self.sps_lengths[i][j]*self.disc*self.resize_factor

        self.longest_path_length = float(max(max(self.sps_lengths, key=lambda x: max(x))))

        rospy.loginfo('Longest path length is ' + str(self.longest_path_length))

        #print self.sps_lengths[10][11]
        rospy.loginfo('Done.')

        rospy.loginfo('Computing safe graph...')
        comm_edge_list = []
        for v1 in self.G_C.vs:
            p1 = self.grid_map_cells[utils.conv_to_hash(*v1["coord"])]
            for v2 in filter(lambda x: x.index > v1.index, self.G_C.vs):
                if(eucl_dist(v1["coord"], v2["coord"]) <= (comm_range/3.0) or 
                  (eucl_dist(v1["coord"], v2["coord"]) <= (comm_range/2.0) and 
                   utils.directLinePossibleBresenham(p1, self.grid_map_cells[utils.conv_to_hash(*v2["coord"])], self.im_array))):

                    comm_edge_list.append((v1.index, v2.index))

        rospy.loginfo('Done.')
    
        self.G_C.add_edges(comm_edge_list)
        
        # Selected locations where key is referring to the source
        # location in pixels, and the value is a list referring
        # to locations where it is worth to go. The list contains
        # locations x,y in pixels.
        rospy.loginfo("A priori communication model is {}".format(
            simulated_comm_model))
        if simulated_comm_model is not "":
            # TODO Only one place to properly define filenames!
            environment_w_model_path = os.path.splitext(map_filename)[0] + '_' + str(int(comm_range)) + '_' + simulated_comm_model  + '.dat'
            self.simulated_comm_model = simulated_comm_model
            # Generate the selectable locations according to source.
            self.selected_locations = {}

            if initialize:
                current_processed_location = 0
                for candidate_location in self.free_positions:
                    self.update_selected_locations(candidate_location)
                    print 'current_processed location ', current_processed_location
                    current_processed_location += 1
                    f = open(environment_w_model_path, "wb")
                    pickle.dump(self, f)
                    f.close()

    def update_selected_locations(self, candidate_location):
        """Update selected locations according to comm model.
        
        Args:
            candidate_location (tuple of float): x,y of fixed robot,
                should be already from free_positions.
        """
        candidate_location_hash = conv_to_hash(
            candidate_location[0], candidate_location[1])
        if candidate_location_hash not in self.selected_locations:
                
            pixel_cell = self.grid_map_cells[candidate_location_hash]
            print "fixed_robot ", pixel_cell
            # Generate communication map from one of the models.
            start = time.time()
            simulated_communication_map = (
                simulated_models.generate_communication_model(
                    self.im_array, self.simulated_comm_model,
                    pixel_cell, self.resize_factor))
            end = time.time() - start
            print "finished calculating communication map ", end
            # Find selected locations according to slope 
            # and change in slope.
            start = time.time()
            slope_map = simulated_models.calculate_slope(self.im_array,
                simulated_communication_map)
            end = time.time() - start
            print "finished calculating slope map", end
            change_slope_map = simulated_models.calculate_slope_change(
                self.im_array, slope_map)
            start = time.time()
            selected_locations = simulated_models.select_locations(
                self.im_array, simulated_communication_map, 
                change_slope_map, self.resize_factor)
            end = time.time() - start
            print "finished calculating change slope map", end
            
            start = time.time()
            # Change locations to locations that are close to
            # cells.
            actual_selected_locations = []
            actual_selected_locations_append = actual_selected_locations.append
            offset_y_pixel = self.I*self.resize_factor
            resize_factor = self.resize_factor
            for i in xrange(len(selected_locations)):
                
                x = selected_locations[i][0]*resize_factor
                y = (offset_y_pixel 
                    - selected_locations[i][1]*resize_factor)
                closest_cell = self.get_closest_cell(
                    (x, y), True) # Locations only for measurements!
                if closest_cell not in actual_selected_locations:
                    actual_selected_locations_append(closest_cell)

            # TODO check all these transformations!!
            # TODO define a function that transforms from pixel to meters and viceversa.
            self.selected_locations[candidate_location_hash] = actual_selected_locations
            end = time.time() - start
            print "finished calculating locations", end
            print actual_selected_locations
            print 'Number of locations selected is ', len(actual_selected_locations)

    def discretize_grid(self):
        # Dictionary that contains actual locations where the robot can
        # be sent to sense, where key is a hash from x,y in meters
        # and value is i (row), j (column) in pixel.
        self.grid_map_cells = {}
      
        #self.im_array_copy = np.copy(self.im_array)

        rows = int(self.I/self.disc)
        cols = int(self.J/self.disc)

        cur_index = 0

        # Add locations in free space uniformly spaced over the grid.
        for i in xrange(rows):
            ii = int(i*self.disc + math.floor(self.disc/2.0))
            if(ii >= self.I): continue
            for j in xrange(cols):
                jj = int(j*self.disc + math.floor(self.disc/2.0))
                if(jj >= self.J): continue
                if(self.all_free(ii, jj, self.I, self.J, WALL_DIST)):
                    #self.im_array_copy[ii][jj] = 0
                    x = (j*self.disc + self.disc/2.0)*self.resize_factor
                    y = self.I*self.resize_factor - (i*self.disc + self.disc/2.0)*self.resize_factor
                    self.free_positions.append((x, y))
                    self.path_positions.append((x, y))
                    self.grid_map_cells[utils.conv_to_hash(x, y)] = (ii,jj)
                    
                    #G_E and G_C have same indexes for same coordinates
                    self.G_E.add_vertex()
                    self.G_E.vs[cur_index]["coord"] = (x,y)

                    self.G_C.add_vertex()
                    self.G_C.vs[cur_index]["coord"] = (x,y)

                    self.coord_index_map[utils.conv_to_hash(x, y)] = cur_index
                    cur_index += 1

        for i in xrange(rows):
            ii = int(i*self.disc + math.floor(self.disc/2.0))
            if(ii >= self.I): continue
            for j in xrange(cols):
                jj = int(j*self.disc + math.floor(self.disc/2.0))
                if(jj >= self.J): continue       
                if(not(self.all_free(ii, jj, self.I, self.J, WALL_DIST)) and self.all_free(ii, jj, self.I, self.J, WALL_DIST/2)):
                    #self.im_array_copy[ii][jj] = 0
                    x = (j*self.disc + self.disc/2.0)*self.resize_factor
                    y = self.I*self.resize_factor - (i*self.disc + self.disc/2.0)*self.resize_factor
                    self.path_positions.append((x, y))
                    self.grid_map_cells[utils.conv_to_hash(x, y)] = (ii,jj)
                    self.G_E.add_vertex()
                    self.G_E.vs[cur_index]["coord"] = (x,y)
                    self.coord_index_map[utils.conv_to_hash(x, y)] = cur_index
                    cur_index += 1



        #plt.imshow(self.im_array_copy)
        #plt.show()
        rospy.loginfo('Environment discretized in ' + str(len(self.free_positions)) + ' candidate cells, and ' + str(len(self.path_positions)) + ' for path planning.')

        
        #add edges to obtain the grid
        #does not need to check the presence of obstacles if grid is fine-grained enough
        for vertex in self.G_E.vs:
            neighbors = filter(lambda x: not(x["coord"] == vertex["coord"]) 
                               and eucl_dist(x["coord"], vertex["coord"]) <= self.disc*self.resize_factor + 2*self.resize_factor, self.G_E.vs)

            self.G_E.add_edges(map(lambda x: (vertex.index, x.index), neighbors))       

        #for edge in self.G_E.es:
        #    p1 = self.G_E.vs[edge.source]
        #    p2 = self.G_E.vs[edge.target]
        #    plt.plot([p1["coord"][0],p2["coord"][0]],[p1["coord"][1],p2["coord"][1]],'k')
            
        #plt.show()
        #print "SP"
        #print self.G_E.shortest_paths_dijkstra(source=324, target=323)[0][0]

    def all_free(self, ii, jj, I, J, border=None):
        if(self.im_array[ii][jj] == 0): return False

        if border is None: return True

        for k in xrange(ii - border, ii + border + 1):
            if k < 0 or k >= I: return False
            for w in xrange(jj - border, jj + border + 1):
                if w < 0 or w >= J: return False
                if (self.im_array[k][w] == 0): return False

        return True

    def get_closest_cell(self, cur_position, free=False):
        if(free):
            closest_path_pos = min(self.path_positions, key=lambda x: eucl_dist(x, cur_position))
            if(closest_path_pos not in self.free_positions):
                return min(filter(lambda x: numObstaclesBetweenRobots(self.im_array, self.I, closest_path_pos, x, self.resize_factor) == 0, # TODO actual path!
                                            self.free_positions), 
                        #self.free_positions, 
                           key=lambda x: eucl_dist(x, closest_path_pos))
            else:
                return closest_path_pos
        else:
            return min(self.path_positions, key=lambda x: eucl_dist(x, cur_position))

    def plot_free_positions(self):
        """Plot in an image free_positions.
        
        Plot an image with environment and locations in free_positions,
        namely those where robots can go to sample.
        """
        test_im = self.im_array.copy()
        print len(self.free_positions)

        for x, y in self.free_positions:
            i, j = self.grid_map_cells[utils.conv_to_hash(x, y)] # row, col
            test_im[i, j] = 64
        cv2.imshow("test", test_im)
        cv2.waitKey()
        
    def plot_selected_locations(self):
        """Plot in an image free_positions.
        
        Plot an image with environment and locations in free_positions,
        namely those where robots can go to sample.
        """
        test_im = self.im_array.copy()
        candidate_location = random.choice(self.free_positions)
        self.update_selected_locations(candidate_location)
        candidate_location_hash = conv_to_hash(
            candidate_location[0], candidate_location[1])

        i, j = self.grid_map_cells[candidate_location_hash] # row, col
        test_im[i, j] = 64
        cv2.imshow("test", test_im)
        cv2.waitKey()

        for x, y in self.selected_locations[candidate_location_hash]:
            i, j = self.grid_map_cells[utils.conv_to_hash(x, y)] # row, col
            test_im[i, j] = 64
        cv2.imshow("test", test_im)
        cv2.waitKey()
        return i, j