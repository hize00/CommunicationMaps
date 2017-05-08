"""Script to plot communication map on top of results.

First run simulated_models.py. Then run this script on the results.


To run:
e.g.,
python figure.py --results_path=../sim_waf_results.txt --environment_path=../amoco_hall.png

Result, if save_figure is enabled, is saved in the same directory as the 
environment.

"""
import os
import sys

import cv2
import gflags
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib._png import read_png
import numpy as np

import simulated_models


XY_AXES_LABELS = ["Width (Pixels)", "Height (Pixels)"]
RSSI_LABEL = "Predicted RSSI (dB)"
VAR_LABEL = "Predictive Variance (dB)"
#f=open('expt_val_results.txt','r')
#f=open('expt_val_results1.txt','r')
#f=open('expt_val_results2.txt','r')
#f=open('sim_distance_results.txt','r')
#f=open('../sim_waf_results.txt','r')
#f=open('sim_mwm_results.txt','r')
#f=open('sim_itu_results.txt','r')
#f=open('waf_sim_results.txt','r')
#f=open('swearingen_3a_itu_results.txt','r')
#f=open('swearingen_3a_mwm_results.txt','r')
#f=open('swearingen_3a_waf_results.txt','r')
#f=open('swearingen_3a_dist_results.txt','r')


def plot_communication_map(X, Y, Z, labels=None, image=None, 
    bound_value=simulated_models.DEFAULT_BASEFIG_Z_AXIS_VALUE,
    ground_truth_x=None, ground_truth_y=None, ground_truth_rssi=None):
    """Plot communication map.

    Args:
        X (numpy.ndarray): X coordinates..
        Y (numpy.ndarray): Y coordinates.
        Z (numpy.ndarray): Z (signal) coordinates.
        labels (list of str): labels for x, y, z axes.
        image (numpy.ndarray): environment
        ground_truth_x (numpy.ndarray): x coordinates of ground truth.
        ground_truth_y (numpy.ndarray): y coordinates of ground truth.
        ground_truth_rssi (numpy.ndarray): rssi of ground truth.

    Returns:
        fig: Figure with the plot.
    """

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    if labels is not None:
        ax.set_xlabel(labels[0])
        ax.set_ylabel(labels[1])
        ax.set_zlabel(labels[2])
    #ax.set_zlim(simulated_models.DEFAULT_RSSI_NO_COMMUNICATION, 0)
    #ax.set_zlim(simulated_models.DEFAULT_BASEFIG_Z_AXIS_VALUE, 0)
    ax.set_zlim(bottom=bound_value, top=max(0, np.amax(Z)))

    surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, 
        linewidth=0, antialiased=False, alpha=0.2)

    #surf=ax.contour(X,Y,Z,cmap=cm.coolwarm)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    if image is not None:
        if len(image.shape) < 3:
            image = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
        if np.amax(image) > 1.0:
            image = image/255
        ax.plot_surface(X,Y, 
            bound_value, 
            #simulated_models.DEFAULT_RSSI_NO_COMMUNICATION,
            rstride=5, cstride=5, facecolors=image)
    if (ground_truth_x is not None and ground_truth_y is not None
        and ground_truth_rssi is not None):
            ax.scatter(ground_truth_x,ground_truth_y,ground_truth_rssi)
    return fig

if __name__ == "__main__":
    gflags.DEFINE_string("observed_values_path", "processed_values_pixels",
        "File containing observed values from real experiments.")
    gflags.DEFINE_string("results_path", "swearingen_3a_dist_results.txt", 
        "Path that contains results from simulated_models.py.")
    gflags.DEFINE_string("environment_path", "swearingen_3a.pgm",
        "Image with the environment.")
    gflags.DEFINE_bool("save_figure", True, "If True, it saves the figure.")
    gflags.DEFINE_string("figure_information", "rssi", 
        "Information in the figure {rssi, var}.") 

    # Parsing of gflags.
    try:
        sys.argv = gflags.FLAGS(sys.argv)  # parse flags
    except gflags.FlagsError, e:
        print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], gflags.FLAGS)
        sys.exit(1)
    
    with open(gflags.FLAGS.results_path, 'r') as predicted_file, \
        open(gflags.FLAGS.observed_values_path, 'r') as observed_file:
        # Read environment image.
        image = cv2.imread(gflags.FLAGS.environment_path)
        if len(image.shape) > 2:
            # Convert to greyscale.
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        height, width = image.shape

        # Setting X and Y for plotting.
        X = np.arange(0, width)
        Y = np.arange(0, height)
        X, Y = np.meshgrid(X, Y)
        axes_labels = XY_AXES_LABELS

        # Initialize matrix that contains RSSI or variance.
        if gflags.FLAGS.figure_information == "rssi":
            bound_value = simulated_models.DEFAULT_BASEFIG_Z_AXIS_VALUE
            ground_truth_x = []
            ground_truth_y = []
            ground_truth_rssi = []
            for line in observed_file:
                i, j, rssi = map(float, line.split(',', 3))
                ground_truth_x.append(i)
                ground_truth_y.append(j)
                ground_truth_rssi.append(rssi)
            ground_truth_x = np.array(ground_truth_x)
            ground_truth_y = np.array(ground_truth_y)
            ground_truth_rssi = np.array(ground_truth_rssi)
            
            axes_labels.append(RSSI_LABEL)
        elif gflags.FLAGS.figure_information == "var":
            bound_value = 0
            ground_truth_x = None
            ground_truth_y = None
            ground_truth_rssi = None
            
            axes_labels.append(VAR_LABEL)
        Z = np.full(np.shape(X), 
            #simulated_models.DEFAULT_RSSI_NO_COMMUNICATION)
            bound_value)
        a = np.shape(X)[0]
        b = np.shape(X)[1]
        i = 0
        for line in predicted_file:
            if i<3:
                # Skip first three lines that contains some comments.
                i=i+1
                continue;
            a = line.split(',',3)
            i = float(a[0]) # x coordinate.
            j = float(a[1]) # y coordinate.
            #print a[2]
            k = float(a[2]) # RSSI.
            Z[j][i]=k
            
        

        # Plot figure.
        fig = plot_communication_map(X, Y, Z, 
            labels=axes_labels, image=image,
            bound_value=bound_value,
            ground_truth_x=ground_truth_x,
            ground_truth_y=ground_truth_y,
            ground_truth_rssi=ground_truth_rssi)
        if gflags.FLAGS.save_figure:
            fig.savefig(
                "_".join([os.path.splitext(gflags.FLAGS.environment_path)[0],
                    os.path.splitext(
                        os.path.basename(gflags.FLAGS.results_path))[0]])
                + '.png', bbox_inches='tight')
        plt.show()

