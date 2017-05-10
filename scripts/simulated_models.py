"""
To run:

Amoco_hall
#x1,y1=320,97
#x1,y1=405,679 -- before translations
x1,y1=405,241 # after translations
#scale is a constant for converting pixels to values.



Swearingen3a.pgm:
- itu (with current default values it works)
python refactored_simulated_models.py

- waf (with current default values)
python refactored_simulated_models.py --communication_model waf

- mwm (with current default values)
python refactored_simulated_models.py --communication_model mwm

- dist (with current default values)
python refactored_simulated_models.py --communication_model dist
"""

import math
import os
from random import randrange
import random
import sys

import cv2
import gflags
import numpy as np
import scipy
from scipy.optimize import curve_fit

import GPy

TXT_EXTENSION = ".txt"
RESULTS_STRING = "results"
ERRORS_STRING = "errors"
ITU_STRING = "itu"
WAF_STRING = "waf"
MWM_STRING = "mwm"
DISTANCE_STRING = "dist"
DEFAULT_RSSI_NO_COMMUNICATION = -100
DEFAULT_MAX_WALLS_WAF = 4
DEFAULT_BASEFIG_Z_AXIS_VALUE=-130
RSSI_MIN = 10

# ITU constants for efficiency.
L0 = 20*math.log10(2400)

# Distance model constants.
WIFI_FREQUENCY = 2.4 * math.pow(10, 9) # Frequency of WiFi.
C_LIGHT = 3.0 * math.pow(10, 8) # Light constant.
WIFI_WAVELENGTH = C_LIGHT / WIFI_FREQUENCY
GL = math.sqrt(1)
PI = math.pi
CONSTANT_FOR_DISTANCE_MODEL = (GL*WIFI_WAVELENGTH) / PI

# Function for math operations for efficiency.
cos = math.cos
sin = math.sin
log = np.log10
ln = np.log
sqrt = math.sqrt

"""
Utilities
"""
def get_image_all_positions(image_path):
    """Return the image with pixel values.
    """
    environment = cv2.imread(image_path)
    try:
        environment = cv2.cvtColor(environment, cv2.COLOR_BGR2GRAY)
    except:
        print "Already greyscale."
    height = environment.shape[0]
    width = environment.shape[1] 
    
    return environment, width, height

def eucl_update_distance(x1,y1,x2,y2, scale):
    diff_x = (x1-x2) * scale
    diff_y = (y1-y2) * scale
    return sqrt(diff_x*diff_x + diff_y*diff_y)

def directLinePossibleBresenham(x1,y1,x2,y2, im_array, width):
    """
    TODO: check if possible to read width and height from im_array.
    """
    # Setup initial conditions
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
    
    for x in xrange(x1, x2 + 1):
        if is_steep:
            
            #if(im_array[width*x+y][0] == 0): return False;
            if(im_array[x, y] == 0): return False;
        else:
            #if(im_array[width*y+x][0] == 0): return False;
            if(im_array[y, x] == 0): return False;

        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    return True

def numObstaclesBetweenRobots(x1, y1, x2, y2, im_array, width, scale):
    #if(directLinePossibleBresenham(x1,y1,x2,y2, im_array, width)):
    #    return 0

    #this is the method of MRESim
    counter = 0 
    angle = math.atan2(y2 - y1, x2 - x1) 

    # Setup initial conditions
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
    already_seen_wall = False
    counter = 0
    for x in xrange(x1, x2 + 1):
        if is_steep:
            
            #if(im_array[width*x+y][0] == 0): return False;
            if(im_array[x, y] == 0): 
                if not already_seen_wall:
                    already_seen_wall = True
                    counter += 1
            else:
                already_seen_wall = False
        else:
            #if(im_array[width*y+x][0] == 0): return False;
            if(im_array[y, x] == 0):
                if not already_seen_wall:
                    already_seen_wall = True
                    counter += 1
            else:
                already_seen_wall = False
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    return counter

    """
    distance = int(eucl_update_distance(x1,y1,x2,y2,1)) # no scale because it is considered the distance in pixels.
    i = 0
    lastX = None
    lastY = None


    already_seen_wall = False
    while (i <= distance):
        currX = x1 + (int)(cos(angle) * i)
        currY = y1 + (int)(sin(angle) * i)

        if(lastX is not None and lastX == currX and lastY == currY):
            pass
        else:
            #if(im_array[currX][currY] == 0): 
            #if(im_array[width*currY+currX][0] == 0): 
            #if(im_array[width*currY+currX] == 0):
            if im_array[currY, currX] == 0:
                if not already_seen_wall:
                    already_seen_wall = True
                    counter += 1

            else:
                already_seen_wall = False
                
        lastX = currX
        lastY = currY
        i += 1
    return int(counter/3)
    """
    
def process_dataset_for_optimization(dataset_path, x1, y1, offset_y):
    f=open(dataset_path, 'r')
    x=[] # Locations (fixed robot, moving robot).
    y=[] # RSSI.

    for line in f:
        x_1=[]
        y_1=[]
        x_1.append(x1)
        x_1.append(y1)
        a=line.split(',',3)
        x_1.append(round(float(a[0])))
        if offset_y:
            x_1.append(height-round(float(a[1])))
        else:
            x_1.append(round(float(a[1])))

        x.append(x_1)
        y.append(round(float(a[2])))

    print 'y is', y
    x=scipy.array(x)
    x=np.transpose(x)
    y=scipy.array(y)

    f.close()
    return x, y

#######################################################################

"""
Communication models.
"""
def ITUmodelRSSI(x1,y1,x2,y2, im_array, width, scale):
    """ITU
    """
    #print x1,y1,x2,y2
    d = eucl_update_distance(x1,y1,x2,y2,scale)
    n_obstacles = numObstaclesBetweenRobots(x1,y1,x2,y2, im_array, width, scale);
    if d==0:
        rss=0;
    else:
        #http://www.comlab.hut.fi/opetus/333/2004_2005_slides/Path_loss_models
        #http://www.radioeng.cz/fulltexts/2003/03_04_42_49.pdf
        #loss=20*math.log10(2400)+10*2.8*math.log10(d)+(n_obstacles*4+11)-28
        loss = L0 + 28*math.log10(d) + n_obstacles*4 - 17
        rss=10-loss

    return rss


def WAFmodelRSSI(x1,y1,x2,y2, im_array, width, scale, loss=-57.58, n=1.53, waf=3.1):
    """WAF
    
    P(d) = P(d_0) - 10 * n * log(d/d_0) - nW*WAF if nW<C
    P(d) = P(d_0) - 10 * n * log(d/d_0) - C*WAF if nW>=C
    In the original paper, determined values:
     WAF=3.1dBm
     C=4
    Table 2 in http://research.microsoft.com/en-us/um/people/bahl/papers/PDF/infocom2000.pdf
     P_{d0} = 57.58, 56,95, 64.94
     n      = 1.53, 1.45, 1.76
     with d_{0} as 1 metre 
     Picked MAAX_WALL=4 as suggested in the above paper here.
    """

    d = eucl_update_distance(x1,y1,x2,y2, scale)
    n_obstacles = numObstaclesBetweenRobots(x1,y1,x2,y2, im_array, width, scale);
    max_walls = DEFAULT_MAX_WALLS_WAF
    #if n_obstacles<4:
    if n_obstacles<max_walls:
        #print 'd is ', d
        if d==0:
            rss=0;
        else:
            #rss= loss - 10 * n * math.log10(d/1) - n_obstacles * waf;
            rss = loss - 10 * n * log(d/1) - n_obstacles * waf;
    else:
        rss = loss - 10 * n * log(d/1) - max_walls * waf;
    return rss

def make_waf_optimize(resolution):
    def waf_optimize(x, loss, n, waf):
        """Optimization for WAF"""
        n_obstacles = 1 # TODO Add numObstacles.
        res = resolution
        d = []
        for i in xrange(0, x.shape[1]):
            d.append(eucl_update_distance(x[0,i],x[1,i],x[2,i],x[3,i], 
                res))
        d = scipy.array(d)
        return loss-10*n*log(d/1) - n_obstacles * waf
    return waf_optimize



def MWMmodelRSSI(x1,y1,x2,y2, im_array, width, scale, L_0=40.0, L_1=5.9, L_2=8.0, n=2.0):
    """MWM.
    
    Default values for parameters from the paper.
    """

    d=eucl_update_distance(x1,y1,x2,y2, scale);
    if d!=0:
        rss= 10*n*log(math.fabs(d));
    else:
        rss=0;
        return rss
    rss = rss+L_0;

    n_obstacles=numObstaclesBetweenRobots(x1,y1,x2,y2, im_array, width, scale)
    rss = rss + (n_obstacles/2) *L_1 + (n_obstacles/2) * L_2;

    rss = 0 - rss;
    return rss

def make_mwm_optimize(resolution):
    """Optimization for MWM that takes constant input.
    
    Args:
        resolution (float): size of a cell in meters.
    """
    def mwm_optimize(x, L_0, L_1, L_2, n):
        """Optimization for MWM.
        TODO Clean, possibly merging model above, checking if im_array is possible.
        """
        d = []
        res = resolution
        for i in xrange(0, x.shape[1]):
            d.append(eucl_update_distance(x[0,i],x[1,i],x[2,i],x[3,i],
                res))
        d = scipy.array(d)

        n_obstacles=1
        rss = 10*n*log(d) +L_0 + (n_obstacles/2) *L_1 + (n_obstacles/2) * L_2;

        rss = 0 - rss;
        return rss
    return mwm_optimize

def distance_modelRSSI(x1,y1,x2,y2, scale):
    """
    """
    d = eucl_update_distance(x1,y1,x2,y2, scale)
    # If the distance is very small, 2 robots can NOT stay so close
    # Assuming the radius of each robot is: 15cm.
    if d < 0.001:
        return 0;
    
    temp = CONSTANT_FOR_DISTANCE_MODEL / d
    temp = temp * temp / 16
    rssi = 10 * log(temp)
    #print d,rssi
    return rssi



#######################################################################

def generate_communication_model(environment_image, communication_model,
    fixed_robot, resolution):
    """Generate communication model according to environment.
    
    Args:
        environment_image (numpy.array): environment image in grayscale.
        communication_model (str): {itu, waf, mwm, dist}.
        fixed_robot (tuple of int): y, x of source location.
        resolution (float): size of a cell in meters.
    Return:
        simulated_communication_map (numpy.array): RSSI values for
            fixed_robot according to communication_model.
    """
    y1, x1 = fixed_robot
    width = environment_image.shape[1]
    simulated_communication_map = np.full(
        environment_image.shape[0:2] + (1,),
            DEFAULT_RSSI_NO_COMMUNICATION)
    #print simulated_communication_map.shape
    for i in xrange(0, environment_image.shape[1]): # Columns.
        for j in xrange(0, environment_image.shape[0]): # Rows.
            if(environment_image[j,i] != 0):
                if communication_model == ITU_STRING:
                    rssi = ITUmodelRSSI(x1, y1, i, j, environment_image, width, resolution)
                elif communication_model == WAF_STRING:
                    rssi = WAFmodelRSSI(x1, y1, i, j, environment_image, width, resolution)#, loss, n, waf)
                elif communication_model == MWM_STRING:
                    rssi = MWMmodelRSSI(x1, y1, i, j, environment_image, width, resolution)#, L_0, L_1, L_2, n)
                elif communication_model == DISTANCE_STRING:
                    rssi = distance_modelRSSI(x1,y1,i,j, resolution)
                simulated_communication_map[j,i] = rssi
    return simulated_communication_map

def calculate_slope(environment_image, communication_map):
    """Generate slope map of communication map.
    
    Args:
        environment_image (numpy.array): environment image in grayscale.
        communication_map (numpy.array): matrix containing RSSI from 
            given source.
    Returns:
        slope_map (numpy.array): Slope from RSSI communication_map.
    """
    slope_map = np.zeros(
        communication_map.shape[0:2] + (1,))
    width, height = environment_image.shape[1], environment_image.shape[0]
    fabs_function = math.fabs
    neighbor_xrange = xrange(-1, 2)
    for i in xrange(0, width): # Columns.
        for j in xrange(0, height): # Rows.
            if(environment_image[j,i] != 0):
                #pixel_ij_neighbor_rssi = []
                pixel_ij_neighbor_rssi = 0.0
                num_neighbors = 0
                # Calculate the mean of the neighbors.
                for k in neighbor_xrange: # Columns.
                    column_neighbor_id = i+k
                    for l in neighbor_xrange: # Rows.
                        row_neighbor_id = j+l
                        if (column_neighbor_id >= 0
                            and column_neighbor_id < width
                            and row_neighbor_id >= 0 
                            and row_neighbor_id < height):
                            #pixel_ij_neighbor_rssi.append(
                            #    communication_map[j+l][i+k])
                            pixel_ij_neighbor_rssi += communication_map[j+l][i+k]
                            num_neighbors += 1
                slope_map[j,i] = fabs_function(communication_map[j,i] 
                    - pixel_ij_neighbor_rssi/num_neighbors)
                    #- np.mean(pixel_ij_neighbor_rssi))

    return slope_map

def calculate_slope_change(environment_image, slope_map):
    """Generate change in slope map from slope_map.

    Args:
        environment_image (numpy.array): environment image in grayscale.
        slope_map (numpy.array): matrix containing slope from 
            communication_map.
    Returns:
        change_slope_map (numpy.array): Change in slope from slope map
            from RSSI communication_map.

    ..TODO: Merge calculate_slope and calculate_slope_change
    """
    change_slope_map = np.zeros(
        slope_map.shape[0:2] + (1,))
    width, height = environment_image.shape[1], environment_image.shape[0]
    fabs_function = math.fabs
    neighbor_xrange = xrange(-1, 2)
    for i in xrange(0, width): # Columns.
        for j in xrange(0, height): # Rows.
            if(environment_image[j,i] != 0):
                #pixel_ij_neighbor_rssi = []
                pixel_ij_neighbor_rssi = 0.0
                num_neighbors = 0
                # Calculate the mean of the neighbors.
                for k in neighbor_xrange: # Columns.
                    column_neighbor_id = i+k
                    for l in neighbor_xrange: # Rows.
                        row_neighbor_id = j+l
                        if (column_neighbor_id >= 0 
                            and column_neighbor_id < width
                            and row_neighbor_id >= 0 
                            and row_neighbor_id < height):
                            #pixel_ij_neighbor_rssi.append(
                            #    slope_map[j+l][i+k])
                            pixel_ij_neighbor_rssi += slope_map[j+l][i+k]
                            num_neighbors += 1
                change_slope_map[j,i] = fabs_function(slope_map[j,i] 
                    - pixel_ij_neighbor_rssi/num_neighbors)
                    #- np.mean(pixel_ij_neighbor_rssi))

    return change_slope_map

def select_locations(environment_image, communication_map,
    change_slope_map, resolution):
    """Generate selected locations from change_slope_map.
    
    Args:
        environment_image (numpy.array): environment image in grayscale.
        communication_map (numpy.array): RSSI map.
        change_slope_map (numpy.array): matrix containing changing_slope
            from communication_map.
        resolution (float): size of a cell (meter)
    Returns:
        selected_locations (numpy.array): Some of the locations
            from RSSI communication_map.

    """

    selected_locations=[]
    append_to_selected_locations = selected_locations.append
    for i in xrange(0, change_slope_map.shape[1]): # Columns.
        for j in xrange(0, change_slope_map.shape[0]): # Rows
            if(environment_image[j,i] != 0):
                if (communication_map[j,i] != 
                        DEFAULT_RSSI_NO_COMMUNICATION 
                    and change_slope_map[j,i] != 0 
                    and change_slope_map[j,i] > RSSI_MIN):
                    #print 'change_slope_map[j,i]',i, j, change_slope_map[j,i]
                    add_ji = True
                    for x,y in selected_locations:
                         if eucl_update_distance(x, y, i, j, resolution) < 0.3: # TODO constant.
                             #print 'reached111'
                             add_ji = False
                             break
                    if add_ji:
                        if [i,j] not in selected_locations:
                            #print 'adding' ,i, j
                            append_to_selected_locations([i,j]) # column, row.
    return selected_locations

###########################################

"""
Main
"""
if __name__ == "__main__":

    gflags.DEFINE_integer("fixed_robot_x", 361, "X coordinates in pixel of the fixed robot.")
    gflags.DEFINE_integer("fixed_robot_y", 682, "X coordinates in pixel of the fixed robot.")
    gflags.DEFINE_float("scale", 0.1, "Dimension of a cell in m.")
    gflags.DEFINE_string("environment", "swearingen_3a.pgm", "Path to the image file containing the environment (grayscale).")
    gflags.DEFINE_string("communication_model", ITU_STRING, "Communication model to use to calculate the predicted RSSI in environment {itu, waf, mwm, dist}.")
    gflags.DEFINE_string("dataset_path", 'processed_values_pixels', "Path to the dataset containing x,y,RSSI of moving robot from experiments.")
    gflags.DEFINE_bool("offset_y", True, "Offset y needed for transforming correctly reference frame of the map from the robot and the image reference frame.")
    # Parsing of gflags.
    try:
        sys.argv = gflags.FLAGS(sys.argv)  # parse flags
    except gflags.FlagsError, e:
        print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], gflags.FLAGS)
        sys.exit(1)
    

    all_positions, width, height = get_image_all_positions(gflags.FLAGS.environment)

    # Files where RSSI and errors are going to be saved.
    environment_sim_rssi_filename = "_".join([os.path.splitext(gflags.FLAGS.environment)[0], gflags.FLAGS.communication_model, RESULTS_STRING]) + TXT_EXTENSION
    environment_sim_error_filename = "_".join([os.path.splitext(gflags.FLAGS.environment)[0], gflags.FLAGS.communication_model, ERRORS_STRING]) + TXT_EXTENSION

    x1, y1 = gflags.FLAGS.fixed_robot_x, gflags.FLAGS.fixed_robot_y

    # Run optimization if needed.
    if gflags.FLAGS.communication_model == WAF_STRING:
        x, y = process_dataset_for_optimization(gflags.FLAGS.dataset_path, x1, y1, gflags.FLAGS.offset_y) # TODO clean repetition
        fitParams, fitCovariances = curve_fit(waf_optimize, x, y, absolute_sigma=True, sigma=1)#, p0=[-20, 1.53, 3.1], bounds=([-100,0,0], [0,4,5])) # TOFIX why bounds don't work?
        loss=fitParams[0]
        n=fitParams[1]
        waf=fitParams[2]
    elif gflags.FLAGS.communication_model == MWM_STRING:
        x, y = process_dataset_for_optimization(gflags.FLAGS.dataset_path, x1, y1, gflags.FLAGS.offset_y)
        fitParams, fitCovariances = curve_fit(make_mwm_optimize(gflags.FLAGS.scale), x, y, absolute_sigma=True, sigma=2)#, bounds=([0,0,0,0], [200,10,10, 10]))
        L_0 = fitParams[0]
        L_1 = fitParams[1]
        L_2 = fitParams[2]
        n = fitParams[3]

    # Writing of predicted values in results file.
    f1=open(environment_sim_rssi_filename, 'w')
    f1.write('#Fixed Robot Location: {} {}\n'.format(x1, y1))
    f1.write('#Moving Robot \n')
    f1.write('#x1,y1,RSSI')
    for i in xrange(0, width):
        for j in xrange(0, height):
            #if(all_positions[width*j+i][0] != 0):
            #if(all_positions[width*j+i] != 0):
            if(all_positions[j][i] != 0):
                if gflags.FLAGS.communication_model == ITU_STRING:
                    #rssi=ITUmodelRSSI(x1, y1, i, (j), all_positions, width, gflags.FLAGS.scale)
                    rssi = ITUmodelRSSI(x1, y1, i, j, all_positions, width, gflags.FLAGS.scale)
                elif gflags.FLAGS.communication_model == WAF_STRING:
                    rssi = WAFmodelRSSI(x1, y1, i, j, all_positions, width, gflags.FLAGS.scale)#, loss, n, waf)
                elif gflags.FLAGS.communication_model == MWM_STRING:
                    rssi = MWMmodelRSSI(x1, y1, i, j, all_positions, width, gflags.FLAGS.scale)#, L_0, L_1, L_2, n)
                elif gflags.FLAGS.communication_model == DISTANCE_STRING:
                    rssi = distance_modelRSSI(x1,y1,i,j, gflags.FLAGS.scale)
            else:
                rssi = DEFAULT_RSSI_NO_COMMUNICATION
            f1.write(''+str(i)+','+str(j)+','+str(rssi)+'\n')
    f1.close()


    f1 = open(environment_sim_rssi_filename,'r')
    f2 = open(gflags.FLAGS.dataset_path,'r')

    dict={}
    #Processing for each value in prediction is painsttakingly high
    #So, we create a dictionary for each "KNOWN" observed value.
    for line in f2:
        a=line.split(',',3)
        b=int(round(float(a[0]))) # Columns (x).
        if gflags.FLAGS.offset_y:
            c = height - int(round(float(a[1]))) # Row (y).
        else:
            c = int(round(float(a[1])))
        d=str(b)+','+str(c)
        #print d, a[2]
        if dict.get(d):
            dict[d].append(a[2])
        else:
            dict.setdefault(d,[])
            dict[d].append(int(a[2]))

    '''
    print dict
    for keys in dict.keys():
        print keys+'::'+dict.get(keys)+'\n'
    '''

    #Process for each line in the predicted Result
    i=0;
    count=0;
    g=[]
    per=[]

    dict1={}

    for line in f1:
        f=[]
        h=[]
        if i<3:
            i=i+1;
            continue
        a=line.split(',',3)
        b=int(round(float(a[0]))) # Column (x).
        c=int(round(float(a[1]))) # Row (y).
        d=str(b)+','+str(c)
        if dict.get(d):
            #print "Need to work for", d, "with ", count, "values"
            count=count+1
            e=dict.get(d)
            for item in e:
                if float(a[2].strip()) == DEFAULT_RSSI_NO_COMMUNICATION:
                    print "High value!!!", b, c
                f.append(math.fabs(float(item)-float(a[2])))
                h.append(math.fabs( 100* ((float(item)-float(a[2]))/float(item))) )
            #print '%error is',min(f)
            dict1[d]=min(f)
            g.append(min(f))
            per.append(min(h))
        #else:
        #    print 'Supervised Value does not exist for',d
    # TODO Clean prints!
    #print 'All errors are:'
    #print g
    #print 'min error is', min(g), 'max error is', max(g)

    f=open(environment_sim_error_filename,'w')
    print 'Errors at each pixel is::'
    for keys in dict1.keys():
        #print keys, dict1.get(keys)
        f.write(keys)
        f.write(',')
        f.write(str(dict1.get(keys)))
        f.write('\n')

    f.write('min error is')
    f.write(str(min(g)))
    f.write('\tmax error is')
    f.write(str(max(g)))
    f.write('\tmean error is')
    f.write(str(np.mean(g)))
    f.write('\tStandard Deviation is')
    f.write(str(np.std(g)))

    f.write('\nmin %error is')
    f.write(str(min(per)))
    f.write('\tmax %error is')
    f.write(str(max(per)))
    f.write('\tmean %error is')
    f.write(str(np.mean(per)))
    f.write('\tStandard Deviation in %error is')
    f.write(str(np.std(per)))
    if gflags.FLAGS.communication_model == WAF_STRING:
        f.write('\nvalues from optimization are:')
        f.write('loss, n, waf:')
        f.write(str(loss))
        f.write(',')
        f.write(str(n))
        f.write(',')
        f.write(str(waf))
    elif gflags.FLAGS.communication_model == MWM_STRING:
        f.write('\nvalues from optimization are:')
        f.write('L_0, L_1, L_2, n:')
        f.write(str(L_0))
        f.write(',')
        f.write(str(L_1))
        f.write(',')
        f.write(str(L_2))
        f.write(',')
        f.write(str(n))

    f.close()
    f1.close()
    f2.close()
