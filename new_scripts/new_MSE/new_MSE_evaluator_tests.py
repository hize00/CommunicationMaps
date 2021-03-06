#!/usr/bin/python
# -*- coding: utf-8 -*-

"""This module is for plotting predicted map from GP with variance.

It creates in figs directory the following:
- MSE over time.
- RMSE over time.
- Predictive Variance over time.
- Predictive STD over time.
- 95% confidence over time.
- Communication map every timestep for a given fixed location (size of
 the plot depends on the communication range).
- Predictive STD map every timestep for a given fixed location (size of
 the plot depends on the communication range).

Run:
- from expl_comm_maps/strategy
python new_MSE_evaluator.py --plot_communication_map
python new_MSE_evaluator.py --task=evaluate
python new_MSE_evaluator.py --task=plot

TODO Change in such a way that directory can be specified.
"""

from copy import deepcopy
import math
import os
import pickle
import random
import time
import sys

import cv2
import GPy
import gflags
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.cbook import get_sample_data
from matplotlib._png import read_png
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from pylab import ogrid, gca, show
from sklearn.metrics import mean_squared_error
import yaml
import scipy.stats

from communication import CommModel, numObstaclesBetweenRobots
import environment
from strategy.msg import SignalData
from GPmodel import GPmodel
import utils

from sklearn import preprocessing

# Parameters in common, defining environment, number of robots used and number of repetitions.
gflags.DEFINE_string("environment", "offices",
    ("Environment to be loaded "
    "(it opens the yaml file to read resolution and image)."))

gflags.DEFINE_integer("num_robots", 2, "Number of robots used in the experiment.")
gflags.DEFINE_integer("num_runs", 5, "Number of repetitions for an experiment.")
gflags.DEFINE_bool("is_simulation", True, "True if simulation data; False if robot")

# Parameters for simulation.
gflags.DEFINE_integer("test_set_size", 10000, "Size of test set to check error in simulation.")
gflags.DEFINE_string("test_type", "normalized", "Test type for creating the test set") #random, discarded_set ,normalized

#gflags.DEFINE_integer("comm_range_exp", 15, "Communication range for creating the test set.")

#MUST BE COHERENT!!!
# Parameters for communication model.
gflags.DEFINE_string("communication_model_path", "data/comm_model_50.xml",
    "Path to the XML file containing communication model parameters.")

# Parameters for plotting.
gflags.DEFINE_integer("granularity", 23350, "Granularity of the mission (seconds) to plot every granularity.")
gflags.DEFINE_integer("mission_duration", 23350, "Mission duration (seconds).")

# FIXED POINT FROM WHERE TO PLOT THE COMM MAP
gflags.DEFINE_bool("plot_communication_map", False, "If True, plot and save communication map in figure.")
gflags.DEFINE_float("fixed_robot_x", 33.158, "x-coordinate for source (meter).")#56.0
gflags.DEFINE_float("fixed_robot_y", 17.129, "y-coordinate for source (meter).")#24.0

gflags.DEFINE_string("task", "evaluate", "Script task {evaluate, plot}.")
gflags.DEFINE_string("log_folder", "/home/andrea/catkin_ws/src/strategy/log/", "Root of log folder.")

# Point selection policy
gflags.DEFINE_string('point_selection_policy', 'grid', 'policy for selecting points in an environment') #click,grid,voronoi

# Data sets to be plotted
sets = ["complete", "pre_processing", "filtered"]

# Plot Parameters ('b--s', 'k-.*', 'g:o', 'r-^')
plot_format = {'complete': ['b--s', 'Complete'], 'pre_processing': ['g:o', 'Pre-processing'],
               'filtered': ['r-^', 'Pairing TSP']}

FONTSIZE = 16

def create_test_set(im_array, comm_model, test_set_size, normalized, resize_factor=0.1):
    def all_free(ii, jj, I, J, border=None):
        if(im_array[ii][jj] == 0): return False

        if border is None: return True

        for k in xrange(ii - border, ii + border + 1):
            if k < 0 or k >= I: return False
            for w in xrange(jj - border, jj + border + 1):
                if w < 0 or w >= J: return False
                if (im_array[k][w] == 0): return False

        return True

    XTest = []
    YTest = []

    dimX = np.size(im_array,1)*resize_factor
    dimY = np.size(im_array,0)*resize_factor
    
    I = np.size(im_array,0)
    J = np.size(im_array,1)

    items = 0
    while items < test_set_size:
        while True:
            x1 = dimX*random.random()
            y1 = dimY*random.random()
            i1 = I - int(y1/resize_factor)
            j1 = int(x1/resize_factor)
            if i1 >= I or j1 >= J: continue
            if all_free(i1,j1,I,J, environment.WALL_DIST):
                break
        
        while True:
            x2 = dimX*random.random()
            y2 = dimY*random.random()
            if (utils.eucl_dist((x1,y1),(x2,y2)) < comm_model.MIN_DIST
                or utils.eucl_dist((x1,y1),(x2,y2)) > comm_model.COMM_RANGE): #gflags.FLAGS.comm_range_exp):
                continue

            i2 = I - int(y2/resize_factor)
            j2 = int(x2/resize_factor)
            if i2 >= I or j2 >= J: continue
            if all_free(i2,j2,I,J, environment.WALL_DIST):
                break        

        num_obstacles = numObstaclesBetweenRobots(im_array, I, (x1,y1), (x2,y2), resize_factor)
        signal_strength = (comm_model.REF_SIGNAL
            - 10*comm_model.PATH_LOSS*math.log10(
                utils.eucl_dist((x1,y1),(x2,y2))/comm_model.REF_DIST)
            - min(comm_model.MAX_WALL,
                num_obstacles)*comm_model.WALL_ATT)

        noise = np.random.normal(0, comm_model.VAR_NOISE)
        signal_strength += noise

        #otherwise the robot has not measured it
        if signal_strength < comm_model.CUTOFF_SAFE: continue

        XTest.append((x1,y1,x2,y2))
        YTest.append(signal_strength)

        items += 1

    if normalized:
        print "Normalizing the test set..."
        YTest = [YTest]
        YTest_norm = preprocessing.normalize(YTest)
        YTest = YTest_norm[0]

    return dimX, dimY, XTest, YTest

def parse_dataset(filename, filter, normalized):
    data_list = []
    f = open(filename, "r")
    lines = f.readlines()
    signals = []
    for line in lines:
        s = line.split()
        if (filter and s[-1] == 'C') or not filter:
            data_list.append(s)
            if normalized:
                signals.append(s[5])

    if normalized:
        signals = [signals]
        signals_norm = preprocessing.normalize(signals)
        signals = signals_norm[0]

        for i in xrange(0, len(data_list)):
            data_list[i][5] = signals[i]

    return data_list

def create_dataset(data_list, set):
    if set == "pre_processing":
        to_remove = []
        for d1 in data_list:
            for d2 in data_list:
                if d1 != d2 and d1[-1] != 'C' and \
                        utils.eucl_dist((float(d1[1]), float(d1[2])), (float(d2[1]), float(d2[2]))) <= 2.0 and \
                        utils.eucl_dist((float(d1[3]), float(d1[4])), (float(d2[3]), float(d2[4]))) <= 2.0:
                    to_remove.append(d1)
                    break

        data = [x for x in data_list if x not in to_remove]
    else:
        data = data_list

    dataset = []
    for s in data:
        new_data = SignalData()
        new_data.timestep = float(s[0])
        new_data.my_pos.pose.position.x = float(s[1])
        new_data.my_pos.pose.position.y = float(s[2])
        new_data.teammate_pos.pose.position.x = float(s[3])
        new_data.teammate_pos.pose.position.y = float(s[4])
        new_data.signal_strength = float(s[5])
        dataset.append(new_data)

    return dataset

def create_test(im_array, data_list, resize_factor=0.1):
    XTest = []
    YTest = []

    dimX = np.size(im_array, 1) * resize_factor
    dimY = np.size(im_array, 0) * resize_factor

    test_set = []
    for d1 in data_list:
        for d2 in data_list:
            if d1 != d2 and d1[-1] != 'C' and \
                    utils.eucl_dist((float(d1[1]), float(d1[2])), (float(d2[1]), float(d2[2]))) <= 2.0 and \
                    utils.eucl_dist((float(d1[3]), float(d1[4])), (float(d2[3]), float(d2[4]))) <= 2.0:
                test_set.append(d1)
                break

    print "Test Set length: " + str(len(test_set))

    for s in test_set:
        XTest.append((float(s[1]), float(s[2]), float(s[3]), float(s[4])))
        YTest.append(float(s[5]))

    return dimX, dimY, XTest, YTest

def specular(img):
    for j in xrange(img.shape[1]):
        for i in xrange(0,img.shape[0]/2):
            tmp = np.copy(img[img.shape[0] -1 -i][j])
            img[img.shape[0] -1 -i][j] = img[i][j]
            img[i][j] = tmp
    return img

def plot_prediction_from_xy_center_3d(environment_image, center,comm_map, dimX, dimY, comm_model, resize_factor=0.1,
    plot_variance=False, all_signal_data=None):
    """Plot predictions from Gaussian Process in a map.

    Args:
        environment_image (numpy.array): Grayscale environment image.
        center (tuple of float): (x,y) of fixed source location.
        comm_map (GPy.model): Gaussian Process model trained.
        dimX (float): X dimension in meters.
        dimY (float): Y dimension in meters.
        comm_model: data structure containing parameters for
            communication model.
        resize_factor (float): cell size in meter.
        plot_variance (bool): seconds for total mission.
        all_signal_data (list): List of signal readings.
    """

    # Get predicted values.    
    X, Y, Z, V = get_prediction_plot(comm_map, center, dimX, dimY, comm_model, resize_factor)

    # Preparing the figure with axes.
    img = deepcopy(environment_image)
    if len(img.shape) < 3:
        img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    if np.amax(img) > 1.0:
        img = img/255
    center_px = (int(center[0]/resize_factor), int(center[1]/resize_factor))
    cv2.circle(img, center_px, 5, (1,0,0))

    x, y = ogrid[0:img.shape[0], 0:img.shape[1]]

    fig_comm = plt.figure()
    ax_comm = fig_comm.gca(projection='3d')
    ax_comm.auto_scale_xyz([0, img.shape[0]], [0, img.shape[1]],[-100, 0])
    #ax_var.set_zlim([0.0, 100.0])
    ax_comm.set_zlabel('Signal strength (dBm)', fontsize=FONTSIZE)
    ax_comm.plot_surface(y,x, -100, rstride=1, cstride=1, facecolors=img)

    surf = ax_comm.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha=0.4)
    fig_comm.colorbar(surf, shrink=0.5, aspect=5)

    # colorbar = fig_comm.colorbar(surf, shrink=0.5, aspect=5)
    # tick_locator = ticker.MaxNLocator(nbins=9)
    # colorbar.locator = tick_locator
    # colorbar.update_ticks()

    ax_comm.view_init(elev=46)
    ax_comm.set_xlabel('X (pixels)', fontsize=FONTSIZE)
    ax_comm.set_ylabel('Y (pixels)', fontsize=FONTSIZE)

    # plt.tight_layout()

    #if all_signal_data is not None:
    #    X_points, Y_points, Z_points = get_scatter_plot(all_signal_data, dimX, dimY, center, resize_factor)
    #    ax_comm.autoscale(enable=False)
    #    ax_comm.scatter(X_points, Y_points, Z_points)


    if plot_variance:
        fig_var = plt.figure()
        ax_var = fig_var.gca(projection='3d')
        ax_var.auto_scale_xyz([0, img.shape[0]], [0, img.shape[1]], [-10, 100])
        #ax_var.set_zlim([0.0, 100.0])
        ax_var.set_zlabel('STD Signal strength (dBm)', fontsize=FONTSIZE)
        ax_var.plot_surface(y,x, 0, rstride=1, cstride=1, facecolors=img)

        surf = ax_var.plot_surface(X, Y, np.sqrt(V), rstride=1, cstride=1,
                                   cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha=0.4)
        fig_var.colorbar(surf, shrink=0.5, aspect=5)

        ax_var.view_init(elev=46)
        ax_var.set_xlabel('X (pixels)', fontsize=FONTSIZE)
        ax_var.set_ylabel('Y (pixels)', fontsize=FONTSIZE)

        # plt.tight_layout()

        return [fig_comm, fig_var]
    else:
        return [fig_comm]

def get_prediction_plot(comm_map, center, dimX, dimY, comm_model, resize_factor=0.1):
    #print center
    center_px = (center[0]/resize_factor, center[1]/resize_factor)
    comm_range = comm_model.COMM_RANGE
    X = np.arange(max(0, int(center_px[0] - comm_range/resize_factor)), min(int(center_px[0] + comm_range/resize_factor),
            int(math.floor(dimX/resize_factor))), 5)
    Y = np.arange(max(0, int(center_px[1] - comm_range/resize_factor)), min(int(center_px[1] + comm_range/resize_factor),
            int(math.floor(dimY/resize_factor))), 5)   
    X, Y = np.meshgrid(X, Y)

    Z = np.zeros(np.shape(X))
    V = np.zeros(np.shape(X))
    for i in xrange(np.shape(X)[0]):
        for j in xrange(np.shape(X)[1]):
            datum = (X[i][j]*resize_factor, Y[i][j]*resize_factor, center[0], center[1])
            predictions = comm_map.predict([datum])
            Z[i][j] = predictions[0][0]
            V[i][j] = predictions[0][1]

    return X,Y,Z,V

def get_scatter_plot(data, dimX, dimY, center, resize_factor=0.1):
    X = []
    Y = []
    Z = []
    for d in data:
        if np.linalg.norm(np.array(center) - np.array([d.teammate_pos.pose.position.x,
                d.teammate_pos.pose.position.y])) < 1.0:
            X.append(d.my_pos.pose.position.x/resize_factor)
            Y.append(d.my_pos.pose.position.y/resize_factor)
            Z.append(d.signal_strength)

    return X, Y, Z

def plot_values(x_vals, y, yerr, ylabel, no_complete, filename):
    fig, ax = plt.subplots()

    # ticks on x axis
    # ax.set_xticks(np.arange(start = 0, stop= gflags.FLAGS.mission_duration,
    #                        step = gflags.FLAGS.mission_duration/360))
    # ticks on y axis
    # plt.locator_params(nbins = 8, axis = 'y')
    # ax.tick_params(axis='y', which='major', pad= 8)

    for key in plot_format.keys():
        #if key == "complete": continue
        if no_complete and key == "complete": continue #not plotting "complete" set if it was too heavy
        plt.errorbar(x_vals, y[key], yerr[key], fmt=plot_format[key][0],label=plot_format[key][1], markersize=10, elinewidth=2)

    loc_legend = 2 if "TIME" in filename else 1
    plt.legend(fontsize=20,loc=loc_legend)
    plt.xlim(x_vals[0]-0.5, x_vals[-1] + 0.5)
    plt.ylabel(ylabel, fontsize=22)
    #if(not('Var' in ylabel) and ):
    #plt.ylim(0,50)
    plt.tick_params(labelsize=20)
    plt.xlabel("Time (minutes)", fontsize=22)
    """if "TIME" not in filename:
        if "VAR" in filename:
            if "4" in filename:
                spacing = 10
            else:
                spacing = 20
        else:
            if "4" in filename:
                spacing = 20
            else:
                spacing = 25
        plt.yticks(np.arange(0, max(map(lambda w: y[w][0] + yerr[w][0] + 25, yerr.keys())), spacing))
        plt.grid()"""
    
    os.environ['PATH'] = os.environ['PATH'] + 'YOURLATEXPATH'
    matplotlib.rcParams['ps.useafm'] = True
    matplotlib.rcParams['pdf.use14corefonts'] = True
    matplotlib.rcParams['text.usetex'] = True
    fig.savefig(filename, bbox_inches='tight')

def plot(environment, num_robots, comm_model_path,granularity, mission_duration):
    """Plot graphs about MSE and traveled distance.

    Args:
        environment (str): name of environment used for saving data.
        num_robots (int): number of robots.
        comm_model_path (str): path to the XML file containing communication 
            model parameters.
        granularity (int): second for each epoch to plot data.
        mission_duration (int): seconds for total mission.
    """
    comm_model = CommModel(comm_model_path)

    sel_pol = gflags.FLAGS.point_selection_policy

    f = open(gflags.FLAGS.log_folder + str(num_robots) + '_' + environment + \
             '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.dat', "rb")
    errors, variances_all, times_all = pickle.load(f)
    f.close()

    x = range(granularity, mission_duration + 1, granularity)
    x = map(lambda x: x/60.0, x)

    no_complete = False

    mse_avg = {}
    rmse_avg = {}
    mse_yerr = {}
    rmse_yerr = {}

    var_avg = {}
    rvar_avg = {}
    var_yerr = {}
    rvar_yerr = {}
    conf_avg = {}
    conf_yerr = {}

    times_avg = {}
    times_yerr = {}

    for set in sets:
        #if set == "complete": continue

        if set == "complete" and len(errors[set]) == 0: #if "complete" is empty
            no_complete = True
            continue

        #print "Set: " + set
        mse_avg[set] = []
        rmse_avg[set] = []
        mse_yerr[set] = []
        rmse_yerr[set] = []

        var_avg[set] = []
        rvar_avg[set] = []
        var_yerr[set] = []
        rvar_yerr[set] = []
        conf_avg[set] = []
        conf_yerr[set] = []

        times_avg[set] = []
        times_yerr[set] = []

        for stamp in range(len(x)):
            cur_mse_avg = 0.0
            cur_rmse_avg = 0.0
            cur_mse_values = []
            cur_rmse_values = []

            cur_var_avg = 0.0
            cur_rvar_avg = 0.0
            cur_conf_avg = 0.0
            cur_var_values = []
            cur_rvar_values = []
            cur_conf_values = []

            cur_times_avg = 0.0
            cur_times_values = []

            for run in errors[set].keys():
                cur_mse_avg += errors[set][run][stamp][0]
                cur_rmse_avg += errors[set][run][stamp][1]
                cur_mse_values.append(errors[set][run][stamp][0])
                cur_rmse_values.append(errors[set][run][stamp][1])

                cur_var_avg += variances_all[set][run][stamp][0]
                cur_rvar_avg += variances_all[set][run][stamp][2]
                cur_conf_avg += variances_all[set][run][stamp][4]
                cur_var_values.append(variances_all[set][run][stamp][0])
                cur_rvar_values.append(variances_all[set][run][stamp][2])
                cur_conf_values.append(variances_all[set][run][stamp][4])

                cur_times_avg += times_all[set][run][stamp]
                cur_times_values.append(times_all[set][run][stamp])

            cur_mse_avg = cur_mse_avg / len(errors[set].keys())
            cur_rmse_avg = cur_rmse_avg / len(errors[set].keys())
            cur_var_avg = cur_var_avg / len(errors[set].keys())
            cur_conf_avg = cur_conf_avg / len(errors[set].keys())
            cur_rvar_avg = cur_rvar_avg / len(errors[set].keys())
            cur_times_avg = cur_times_avg / len(errors[set].keys())

            mse_avg[set].append(cur_mse_avg)
            rmse_avg[set].append(cur_rmse_avg)
            mse_yerr[set].append(np.std(cur_mse_values))
            rmse_yerr[set].append(np.std(cur_rmse_values))

            var_avg[set].append(cur_var_avg)
            rvar_avg[set].append(cur_rvar_avg)
            conf_avg[set].append(cur_conf_avg)
            var_yerr[set].append(np.std(cur_var_values))
            rvar_yerr[set].append(np.std(cur_rvar_values))
            conf_yerr[set].append(np.std(cur_conf_values))

            times_avg[set].append(cur_times_avg)
            times_yerr[set].append(np.std(cur_times_values))

    plot_values(x, rmse_avg, rmse_yerr, "RMSE", no_complete, os.getcwd() + '/figs/RMSE_' + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.pdf')
    plot_values(x, mse_avg, mse_yerr, "MSE", no_complete, os.getcwd() + '/figs/MSE_' + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.pdf')
    plot_values(x, conf_avg, conf_yerr, "95% Confidence Width",no_complete, os.getcwd() + '/figs/95CONF_' + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.pdf')
    plot_values(x, var_avg, var_yerr, "Pred. Variance", no_complete, os.getcwd() + '/figs/VAR_' + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.pdf')
    plot_values(x, rvar_avg, rvar_yerr, "Pred. Std. Dev.", no_complete, os.getcwd() + '/figs/STDEV_' + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.pdf')
    plot_values(x, times_avg, times_yerr, "GP Training Time", no_complete, os.getcwd() + '/figs/TIME_' + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + '_' + sel_pol + '.pdf')

def read_environment(environment_yaml_path):
    """Read environment yaml file to get the figure and the resolution.

    Args:
        environment_yaml_path (str): Path to yaml file from map_saver in ROS
            that contains environment and resolution..

    Return:
        environment_image (numpy.ndarray): Grayscale image.
        resolution (float): size of a cell, in meter typically.
    """
    with open(environment_yaml_path, 'r') as environment_yaml_file:
        try:
            environment_yaml = yaml.load(environment_yaml_file)
            resolution = environment_yaml["resolution"]
            environment_image_path = os.path.join(os.path.dirname(environment_yaml_path),environment_yaml["image"])
            environment_image = cv2.imread(environment_image_path)
            if len(environment_image.shape) > 2:
                environment_image = cv2.cvtColor(environment_image,cv2.COLOR_BGR2GRAY)
            return environment_image, resolution
        except yaml.YAMLError as exc:
            print(exc)
            return None

def evaluate(environment, num_robots, num_runs, is_simulation,
    comm_model_path, granularity, mission_duration,
    plot_comm_map, fixed_robot, test_set_size, log_folder):
    """Create pickle files containing data to be plotted.

    It processes the dat files containing the collected data from the robots.
    It outputs a pickle file, containing errors, variances, and GP comm_map,
    according to run, environment, robot.

    Args:
        environment (str): name of environment used for saving data.
        num_robots (int): number of robots.
        num_runs (int): number of repeated experiments.
        is_simulation(bool): True, if data generated from simulation.
        comm_model_path (str): path to the XML file containing communication 
            model parameters.
        granularity (int): second for each epoch to plot data.
        mission_duration (int): seconds for total mission.
        plot_comm_map (bool): If True, plot communication map.
        fixed_robot (tuple of float): x,y of robot in meters.
        filter (bool): If True, consider ony information found by Pairing TSP algorithm
        test_set_size (int): number of samples in the test set
        log_folder (str): folder where logs are saved.
    """
    log_folder = os.path.expanduser(log_folder)
    
    # Reading of the communication parameters necessary to produce correct test set.
    comm_model = CommModel(comm_model_path)

    # Reading environment image.
    environment_yaml_path = os.getcwd() + '/envs/' + environment + '.yaml'

    sel_pol = gflags.FLAGS.point_selection_policy

    im_array, resolution = read_environment(environment_yaml_path)
    im_array = specular(im_array)

    test_type = gflags.FLAGS.test_type
    
    # Generation of test set.
    if is_simulation:
        if test_type != "discarded_set":
            random.seed(0)
            np.random.seed(0)
            dimX, dimY, XTest, YTest = create_test_set(im_array, comm_model, test_set_size,
                                                       False if test_type == "random" else True, resolution)

    runs = range(num_runs)

    errors = {}
    variances_all = {}
    times_all = {}

    for set in sets:
        #if set == "complete": continue

        print "Set: ", set

        filter = True if set == "filtered" else False

        errors[set] = {}
        variances_all[set] = {}
        times_all[set] = {}

        for run in runs:
            if run != 0 and plot_comm_map: break

            parsed = []
            if test_type == "discarded_set" and set == "filtered":
                parsed_test_set = []

            for robot in range(num_robots):
                dataset_filename = log_folder + str(run) + '_' + environment + \
                                   '_' + str(robot) + '_' + str(num_robots) + \
                                   '_' + str(int(comm_model.COMM_RANGE)) + \
                                   '_' + sel_pol + '.dat'
                parsed += parse_dataset(dataset_filename, filter, True if test_type == "normalized" else False)

                if test_type == "discarded_set" and set == "filtered":
                    parsed_test_set += parse_dataset(dataset_filename, False, False)

            all_signal_data = create_dataset(parsed, set)

            if test_type == "discarded_set":
                dimX, dimY, XTest, YTest = create_test(im_array, parsed if set != "filtered" else parsed_test_set, resolution)

            print "Set length: " + str(len(all_signal_data))

            if not plot_comm_map and \
                    len(all_signal_data) >= 10000: #with 5 runs and 10k samples for run, 64GB of RAM/swap memory are not enough
                print "Too many samples: the GP training would be too heavy. Discarding this set."
                break

            errors[set][run] = []
            variances_all[set][run] = []
            times_all[set][run] = []

            all_secs = range(granularity, mission_duration + 1, granularity)
            for secs in all_secs:
                cur_signal_data = []
                for datum in all_signal_data:
                    if datum.timestep <= secs:
                        cur_signal_data.append(datum)

                print "Run: " + str(run) + " - number of data: ", len(cur_signal_data)

                start = time.time()
                comm_map = GPmodel(dimX, dimY, comm_model.COMM_RANGE, False)
                comm_map.update_model(cur_signal_data)
                end = time.time()

                predictions_all = comm_map.predict(XTest)
                predictions = map(lambda x: x[0], predictions_all)
                variances = map(lambda x: x[1], predictions_all)
                std_devs = map(lambda x: math.sqrt(x), variances)
                conf_95 = map(lambda x: 1.96 * x, std_devs)

                errors[set][run].append((mean_squared_error(YTest, predictions), math.sqrt(mean_squared_error(YTest, predictions))))
                variances_all[set][run].append((np.mean(variances), np.std(variances), np.mean(std_devs), np.std(std_devs),np.mean(conf_95), np.std(conf_95)))
                times_all[set][run].append(end - start)

                if plot_comm_map:
                    print "Drawing the Communication map..."

                    if set == "complete":
                        extension = '.png'
                    elif set == "pre_processing":
                        extension = '_P.png'
                    else:
                        extension = '_C.png'

                    communication_figures = plot_prediction_from_xy_center_3d(im_array, fixed_robot, comm_map, dimX, dimY,
                                                                              comm_model, resolution, True, cur_signal_data)
                    communication_map_figure_filename = os.getcwd() + '/figs/COMM_MAP_' + str(num_robots) + \
                                                        '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) + \
                                                        '_' + str(run) + '_' + str(secs) + '_' + sel_pol + extension
                    communication_figures[0].savefig(communication_map_figure_filename, bbox_inches='tight')
                    print "Done."
                    if len(communication_figures) > 1:
                        print "Drawing the Variance map..."
                        communication_map_figure_filename = os.getcwd() + '/figs/COMM_MAP_' + str(num_robots) + \
                                                            '_'  + environment + '_' + str(int(comm_model.COMM_RANGE)) + \
                                                            '_' + str(run) + '_' + str(secs) + '_' + sel_pol + \
                                                            '_' + 'VAR' + extension
                        communication_figures[1].savefig(communication_map_figure_filename, bbox_inches='tight')
                        print "Done."

    print '----------------------------------------------------------------------------'
    print errors
    print '----------------------------------------------------------------------------'
    print variances_all
    print '----------------------------------------------------------------------------'
    print times_all
    print '----------------------------------------------------------------------------'

    f = open(log_folder + str(num_robots) + '_' + environment + '_' + str(int(comm_model.COMM_RANGE)) +
             '_' + gflags.FLAGS.point_selection_policy +'.dat', "wb")
    pickle.dump((errors, variances_all, times_all), f)
    f.close()

if __name__ == '__main__':

    os.chdir("/home/andrea/catkin_ws/src/strategy/")

    # Parsing of gflags.
    try:
        sys.argv = gflags.FLAGS(sys.argv)  # parse flags
    except gflags.FlagsError, e:
        print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0],gflags.FLAGS)
        sys.exit(1)
    if gflags.FLAGS.task == 'evaluate':
        evaluate(gflags.FLAGS.environment, gflags.FLAGS.num_robots,
            gflags.FLAGS.num_runs, gflags.FLAGS.is_simulation,
            gflags.FLAGS.communication_model_path,
            gflags.FLAGS.granularity, gflags.FLAGS.mission_duration,
            gflags.FLAGS.plot_communication_map,
            (gflags.FLAGS.fixed_robot_x, gflags.FLAGS.fixed_robot_y),
            gflags.FLAGS.test_set_size,
            gflags.FLAGS.log_folder)
    elif gflags.FLAGS.task == 'plot':
        plot(gflags.FLAGS.environment, gflags.FLAGS.num_robots,
            gflags.FLAGS.communication_model_path,
            gflags.FLAGS.granularity, gflags.FLAGS.mission_duration)