import GPy
import numpy as np
import time
import random

import communication
from utils import conv_to_hash, eucl_dist

class GPmodel(object):
    def __init__(self, dimX, dimY, comm_range, tiling, comm_model=None, log_filename=None):
        self.comm_range = comm_range

        if comm_model is None:
            self.CUTOFF = -93
            self.MAX_SIGNAL = -10
        else:
            self.CUTOFF = int(comm_model.CUTOFF)
            self.MAX_SIGNAL = int(comm_model.MAX_SIGNAL)

        #the model will always be based on tiles, this is useful for prediction
        self.tiling = tiling
        self.GP_dict = {} #contains different GPs associated with the tiling of the environment
        self.means_dict = {}
        
        self.dimX = dimX
        self.dimY = dimY
        self.log_filename = log_filename

    def update_model(self, all_signal_data):
        start_glob = time.time()

        self.GP_dict = {}
        self.means_dict = {}

        #transform data in a more convenient format
        data_short = map(lambda d: [d.my_pos.pose.position.x, d.my_pos.pose.position.y, 
                                    d.teammate_pos.pose.position.x, d.teammate_pos.pose.position.y,
                                    d.signal_strength], all_signal_data)

        tiles_data = {}

        if(self.tiling):
            x_c = self.comm_range

            while(x_c >= self.dimX):
                x_c = x_c/2.0
            while(x_c < self.dimX):
                y_c = self.comm_range
                while(y_c >= self.dimY):
                    y_c = y_c/2.0
                while(y_c < self.dimY):
                    tiles_data[(x_c, y_c)] = []
                    y_c += self.comm_range               
                x_c += self.comm_range

            print "Creating new models with tiles:"
            print tiles_data.keys()

            #associate data to tiles
            for datum in data_short:
                for (x_c, y_c) in tiles_data.keys():
                    if(abs(datum[0] - x_c) <= self.comm_range and abs(datum[1] - y_c) <= self.comm_range and
                       abs(datum[2] - x_c) <= self.comm_range and abs(datum[3] - y_c) <= self.comm_range):
                        tiles_data[(x_c, y_c)].append(datum)
        else:
            #know there will be a single tile
            tiles_data[(self.dimX/2.0, self.dimY/2.0)] = []            
            tiles_data[tiles_data.keys()[0]] = data_short
            #print "Model centered in:"
            #print tiles_data.keys()

        data_len_log = ''
        data_time_log = ''
        for key in tiles_data.keys():
            start_single = time.time()
            #train a GP in the region centered in key
            cur_data = tiles_data[key]
            data_len_log += str(len(cur_data)) + ' '

            if(len(cur_data) == 0):
                self.GP_dict[key] = None
                data_time_log += str(time.time() - start_single) + ' '
                continue

            curX = map(lambda x: np.array(x[0:4]), cur_data)
            curX = np.array(curX)
            #print curX

            curY = map(lambda x: x[4], cur_data)
            curY = np.array(curY)
            curY = np.atleast_2d(curY).T
            mean = np.mean(curY)
            self.means_dict[key] = mean
            curY = curY - mean 

            #train new GP
            print "Training a new GP..."
            kernel = GPy.kern.RBF(input_dim=4, variance=1., lengthscale=1.) #SE
            m = GPy.models.GPRegression(curX, curY, kernel)
            m.optimize()
            print "Done."
            self.GP_dict[key] = m
            data_time_log += str(time.time() - start_single) + ' '

        glob_time = time.time() - start_glob

        if(self.log_filename is None): return

        self.logger = open(self.log_filename, "a")
    
        #format: T ALL_DATA_SIZE GLOB_TIME N_TILES TILES_SIZES... TILES_TIMES...
        self.logger.write('T ' + str(len(data_short)) + ' ' + str(glob_time) + ' ' + str(len(self.GP_dict.keys())) +
                          ' ' + data_len_log + data_time_log + '\n')
        self.logger.close()

    def get_average_variance_comm_region(self, center, env):
        """Calculate average of variance centered in center over comm_range.

        center (tuple of float): x, y position, in meters.
        env (Environment): environment used.
        """

        """minX = max(0, center[0] - self.comm_range)
        maxX = min(self.dimX, center[0] + self.comm_range)

        minY = max(0, center[1] - self.comm_range)
        maxY = min(self.dimY, center[1] + self.comm_range)

        X = np.arange(minX, maxX, 2.0)
        Y = np.arange(minY, maxY, 2.0)"""

        data = []


        for x, y in env.free_positions:
            if eucl_dist(center, (x,y)) <= self.comm_range:
                data.append((center[0], center[1], x, y))
                data.append((x, y, center[0], center[1]))        
        """
        for x in X:
            for y in Y:
                # Only for free cell and within communication range
                # calculate variance.
                
                ii, jj, = grid_map_cells[conv_to_hash(x, y)]
                if(env.all_free(ii, jj, env.I, env.J) and 
                    eucl_dist(center, (x,y)) <= self.comm_range):
                    data.append((center[0], center[1], x, y))
                    data.append((x, y, center[0], center[1]))
        """

        variances = map(lambda x: x[1], self.predict(data))
        return np.mean(variances)

    def predict(self, data):
        if(self.tiling):
            return self.predict_tiling(data)
        else:
            return self.predict_plain(data)

    def predict_plain(self, data):
        if(len(self.GP_dict.keys()) == 0):
            return [(float(random.randint(self.CUTOFF, self.MAX_SIGNAL)), float('inf')) for _ in xrange(len(data))]

        np_data = map(lambda x: np.array(x), data)
        key = self.GP_dict.keys()[0]
        YPred, YVar = self.GP_dict[key].predict(np.atleast_2d(np.array(np_data)))
        YPred = np.transpose(YPred)[0] + self.means_dict[key]
        YVar = np.transpose(YVar)[0]
        return zip(YPred, YVar)

    def predict_tiling(self, data):
        #create double mapping GP -> data to predict and datum -> GP predicting.
        GP_data_dict = {}
        GP_last_indexes_dict = {}
        for key in self.GP_dict.keys():
            GP_data_dict[key] = []
            GP_last_indexes_dict[key] = -1

        datum_GPs_dict = {}
        
        for datum in data:
            datum_GPs_dict[datum] = []
            np_datum = np.array(datum)
            for key in filter(lambda k: self.GP_dict[k] is not None and 
                                               abs(datum[0] - k[0]) <= self.comm_range and abs(datum[1] - k[1]) <= self.comm_range and
                                               abs(datum[2] - k[0]) <= self.comm_range and abs(datum[3] - k[1]) <= self.comm_range, self.GP_dict.keys()): 
                new_index = GP_last_indexes_dict[key] + 1
                GP_last_indexes_dict[key] = new_index

                #this datum is predicted by GP 'key', and the result will be at index new_index
                datum_GPs_dict[datum].append((key, new_index))
                GP_data_dict[key].append(np_datum)

        #do all batch predictions
        GP_predictions = {}
        for key in self.GP_dict.keys():
            if(len(GP_data_dict[key]) == 0):
                continue

            YPred, YVar = self.GP_dict[key].predict(np.atleast_2d(np.array(GP_data_dict[key])))
            YPred = np.transpose(YPred)[0] + self.means_dict[key]
            YVar = np.transpose(YVar)[0]
            GP_predictions[key] = (YPred, YVar)
        
        #retrieve best prediction for each input
        predictions = []
        for datum in data:
            best_var = float('inf')
            for (GPkey, index) in datum_GPs_dict[datum]:
                pred = GP_predictions[GPkey][0][index]
                var = GP_predictions[GPkey][1][index]
                if var < best_var:
                    best_pred = pred
                    best_var = var

            if(not(best_var < float('inf'))):
                best_pred = float(random.randint(self.CUTOFF, self.MAX_SIGNAL))

            predictions.append((best_pred, best_var))

        return predictions

