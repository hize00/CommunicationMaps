#!/usr/bin/env python

import sys
import random
import numpy as np
import os
import time

coord = []
robot_moving = []
robot_plan = []
timetable = []

reading_coords = 0
reading_RM = 0
reading_TT = 0

with open('/home/andrea/Desktop/parser/solution_plan_4_robots.txt', 'r') as file:
    data = file.readlines()
    for line in data:
        words = line.split()
        if len(words) > 0:
            if words[0] == "N_ROBOTS:":
                N_ROBOTS = int(words[1])
                line_lenght = N_ROBOTS * 2 + N_ROBOTS - 1
            elif words[0] == "COORDINATES_LIST:":
                reading_coords = 1
                continue
            if reading_coords == 1:
                if words[0] != ';':
                    for i in range(0, line_lenght):
                        if words[i] != '|':
                            coord.append(int(words[i]))
                else:
                    reading_coords = 0
            elif words[0] == "ROBOT_MOVING:":
                reading_RM = 1
                continue
            if reading_RM == 1:
                min_list_RM = []
                if words[0] != ';':
                    for i in range(0, N_ROBOTS):
                        min_list_RM.append(int(words[i]))
                    robot_moving.append(min_list_RM)
                else:
                    reading_RM = 0
            elif words[0] == "TIMETABLE:":
                reading_TT = 1
                continue
            if reading_TT == 1:
                min_list_T = []
                if words[0] != ';':
                    for i in range(0, N_ROBOTS):
                        min_list_T.append(float(words[i]))
                    timetable.append(min_list_T)
                else:
                    reading_TT = 0

file.close()

# grouping coordinates by (x,y)
coord = [coord[i:i + 2] for i in range(0, len(coord), 2)]  # group x and y of a single robot

# converting from pixels to meters
for c in coord:
    pos = c
    c[0] = float(79.7 - 0.1 * pos[0])
    c[1] = float(0.1 * pos[1])

coord = [tuple(l) for l in coord]

coord = [coord[i:i + N_ROBOTS] for i in range(0, len(coord), N_ROBOTS)]  # create a plan of coordinates

#assigning a timestamp to a robot everytime it has to move and creating the plan
count_config = 0
for config in robot_moving:
    count = 0
    first_robot = -1
    for robot in config:
        if (count_config == (len(robot_moving) - 1) and robot !=0) or (count_config == 0 and robot == 0) :
            # I add the starting/final positions of each robot to plan: [my_self,(pose),(pose),my_self]
            my_self = count
            robot_plan.append(my_self)
            position = count
            robot_plan.append(coord[count_config][position])
            robot_plan.append(coord[count_config][position])
            robot_plan.append(my_self)
            robot_plan.append(timetable[count_config][position])
        else:
            if robot != 0 and first_robot == -1:
                first_robot = count
                robot_plan.append(first_robot)
                position = count
                robot_plan.append(coord[count_config][position])
            elif robot != 0 and first_robot != -1:
                second_robot = count
                position = count
                robot_plan.append(coord[count_config][position])
                robot_plan.append(second_robot)
                robot_plan.append(timetable[count_config][position])

                # assigning a reflected plan to the communication teammate
                robot_plan.append(second_robot)
                robot_plan.append(coord[count_config][position])
                robot_plan.append(coord[count_config][first_robot])
                robot_plan.append(first_robot)
                robot_plan.append(timetable[count_config][position])
        count += 1
    count_config += 1

# grouping plan elements: [my_id, (my_coords),(teammate_coords),communication_teammate,timestamp]
plan_elements = 5
robot_plan = [robot_plan[i:i + plan_elements] for i in range(0, len(robot_plan), plan_elements)]

# grouping plan elements: [(my_id, (((my_coords), (teammate_coords)), communication_teammate, timestamp)]
plans = []
for plan in robot_plan:
    my_id = []
    coordinates = []
    msgs = []
    my_id.append(plan[0])
    coordinates.append(plan[1])
    coordinates.append(plan[2])
    msgs.append(tuple(coordinates))
    msgs.append(plan[3])
    msgs.append(plan[4])
    my_id.append(tuple(msgs))
    plans.append(tuple(my_id))

plans = tuple(plans)

# grouping plan elements by robot_id
robot_ids = set(map(lambda x: x[0], plans))
plan_id = [[y[1] for y in plans if y[0] == x] for x in robot_ids]

#if len(robot_ids) < N_ROBOTS:
#    for i in xrange(N_ROBOTS):
#        if i not in robot_ids:
#            plan_id.insert(i, ())  # if a robot never moves, his plan will be empty

plan_id = tuple([tuple(l) for l in plan_id])  # plans = (plan_robot_0, plan_robot_1,...,plan_robot_n)

###################################################################################################


filename = '/home/andrea/Desktop/parser/0_open_0_4_50.dat'
carlo = False
f = open(filename, "r")
lines = f.readlines()
count = 0
tot = 0
for line in lines:
    s = line.split()
    tot += 1
    if (carlo and s[-1] == 'C') or not carlo:
        #print s
        count +=1
#print "Count: " + str(count)
#print "Tot: " + str(tot)

###################################################################################################

num_runs = 5
procs = 3
runs = range(num_runs)
#print "Runs: " + str(runs)
#print "range(proc): " + str(range(procs))
proc = 1

for p in range(procs):
    if p == proc:
        runs = np.array_split(runs, procs)[p]
        #print runs

###################################################################################################

#print os.getcwd()

os.chdir('/home/andrea/catkin_ws/src/strategy/')

#print os.getcwd()

log_dir = os.getcwd() + '/log/'

#print log_dir

#print "Run: " + str(5) + ", log_dir: " + str(log_dir)

###################################################################################################

old_pos = (42.0, 18.0)

pos = old_pos
pos = list(pos)

#print pos

new_pos = list(old_pos)
#print new_pos

###################################################################################################

count = 0
fix = 0
max_fix = 3

timeout = time.time() #+2
found = False
#print time.time()
while not found:
    num = round(random.uniform(-10 - 0.5, 10+ 0.5), 2)
    numb = round(random.uniform(-20- 0.5, 20+ 0.5), 2)
    count += 1
    if time.time() >= timeout:
        fix += 1
        #print "fix: " + str(fix) + ", time.time(): "+ str(time.time())
        if fix == max_fix:
            found = True
            #print found

#print count

######################################################################################################

#original
def parse_dataset(filename):
    dataset = []
    filter = gflags.FLAGS.filter_dat
    f = open(filename, "r")
    lines = f.readlines()
    for line in lines:
        s = line.split()
        if (filter and s[-1] == 'C') or not filter:
            new_data = SignalData()
            new_data.timestep = float(s[0])
            new_data.my_pos.pose.position.x = float(s[1])
            new_data.my_pos.pose.position.y = float(s[2])
            new_data.teammate_pos.pose.position.x = float(s[3])
            new_data.teammate_pos.pose.position.y = float(s[4])
            new_data.signal_strength = float(s[5])

            dataset.append(new_data)

    return dataset

#scarto

def parse_dataset(filename):
    dataset = []
    filter = gflags.FLAGS.filter_dat
    f = open(filename, "r")
    lines = f.readlines()
    scarto = False
    for line in lines:
        s = line.split()
        if (filter and s[-1] == 'C') or not filter:
            if s[-1] != 'C':
                if not scarto:
                    scarto = True
                    new_data = SignalData()
                    new_data.timestep = float(s[0])
                    new_data.my_pos.pose.position.x = float(s[1])
                    new_data.my_pos.pose.position.y = float(s[2])
                    new_data.teammate_pos.pose.position.x = float(s[3])
                    new_data.teammate_pos.pose.position.y = float(s[4])
                    new_data.signal_strength = float(s[5])

                    dataset.append(new_data)
                else:
                    scarto = False
            else:
                new_data = SignalData()
                new_data.timestep = float(s[0])
                new_data.my_pos.pose.position.x = float(s[1])
                new_data.my_pos.pose.position.y = float(s[2])
                new_data.teammate_pos.pose.position.x = float(s[3])
                new_data.teammate_pos.pose.position.y = float(s[4])
                new_data.signal_strength = float(s[5])

                dataset.append(new_data)

    return dataset

###############################################################################################################
"""
    global start
    global goal_config

    graph = environment_discretization()

    points = []
    for vertex in graph:
        x = vertex['x_coord']
        y = vertex['y_coord']
        points.append((x, -y))

    voronoi = Voronoi(points, qhull_options='Qbb Qc Qx')
    lines = [
        LineString(voronoi.vertices[line])
        for line in voronoi.ridge_vertices if -1 not in line
    ]
    # print lines

    voronoi_plot_2d(voronoi)

    voronoi_points = []
    for i, reg in enumerate(voronoi.regions):
        voronoi_points += points[i]

    voronoi_points = [voronoi_points[i:i + 2] for i in range(0, len(voronoi_points), 2)]

    for point in voronoi_points:
        vertex_id = get_closest_vertex(point[0], point[1])
        goal_config.append(vertex_id)
        write_exp_file()

    print 'Done.'

    plot_plan(goal_config)

##############################################################################################

    image = img_as_bool(color.rgb2gray(io.imread(gflags.FLAGS.file_path)))
    out = morphology.medial_axis(image)
    out = out.astype(int)

    f, (ax0, ax1) = plt.subplots(1, 2)
    ax0.imshow(image, cmap='gray', interpolation='nearest')
    ax1.imshow(out, cmap='gray', interpolation='nearest')
    plt.show()

##############################################################################################


    graph = Graph(directed=False)

    for i in range(0, len(final_points)):
            graph.add_vertex()
            graph.vs[i]['y_coord'] = final_points[i][0]
            graph.vs[i]['x_coord'] = final_points[i][1]

    neighbors = []
    for vertex_id in graph.vs:

        curr_x = vertex_id['x_coord']
        curr_y = vertex_id['y_coord']

        up = graph.vs.select(x_coord_eq=curr_x, y_coord_eq=(curr_y + gflags.FLAGS.sel_grid_size))
        if len(up):
            neighbors.append((vertex_id, up[0].index))

        right = graph.vs.select(x_coord_eq=(curr_x + gflags.FLAGS.sel_grid_size), y_coord_eq=curr_y)
        if len(right):
            neighbors.append((vertex_id, right[0].index))

    graph.add_edges(neighbors)

    too_close = []
    for neighbor in neighbors:
        too_close.append((neighbor[0]['y_coord'], neighbor[0]['x_coord']))

    final_points = [x for x in final_points if x not in too_close]

################################################################################################

    cluster = []
    neighbors = []
    for p1 in final_points:
        count = 0
        for p2 in final_points:
            if p1 == p2: continue
            if eucl_dist((p1[0], p1[1]), (p2[0], p2[1])) <= 2 * gflags.FLAGS.sel_grid_size:
                count += 1
                neighbors.append(p1)

        if count != 0 and p1 not in neighbors:
            cluster.append(p1)

    final_points = [x for x in final_points if x not in cluster]

##############################################################################################

    final_points = rdp(final_points, epsilon=0.5)

####################################################################################################

    too_close = []
    for p1 in final_points:
        min_dist = eucl_dist((p1[0], p1[1]), (to_remove[0][0], to_remove[0][1]))
        min_coords = to_remove[0]
        for p2 in to_remove:
            if eucl_dist((p1[0],p1[1]), (p2[0],p2[1])) < min_dist:
                min_dist = eucl_dist((p1[0],p1[1]), (p2[0],p2[1]))
                min_coords = p2

        for p3 in final_points:
            if p1 == p3: continue
            if eucl_dist((p3[0], p3[1]), (min_coords[0], min_coords[1])) < min_dist:
                if p3 not in too_close:
                    too_close.append(p3)

    final_points = [x for x in final_points if x not in too_close]


"""


####################################################################################################

string = 'offices_phys_uniform_grid.graphml'
env =     env_name = (os.path.splitext(string)[0]).split("_")[0]
print env

####################################################################################################

def create_new_test_set(im_array, comm_model, runs , log_folder, sel_pol, num_robots, resize_factor=0.1):
    environment = gflags.FLAGS.environment

    XTest = []
    YTest = []

    dimX = np.size(im_array, 1) * resize_factor
    dimY = np.size(im_array, 0) * resize_factor

    parsed = []
    for run in runs:
        for robot in range(num_robots):
            dataset_filename = log_folder + str(run) + '_' + environment + \
                               '_' + str(robot) + '_' + str(num_robots) + \
                               '_' + str(int(comm_model.COMM_RANGE)) + \
                               '_' + sel_pol + '.dat'
            parsed += parse_dataset(dataset_filename, False)

    print str(len(parsed))

    test_data = []
    for d1 in parsed:
        for d2 in parsed:
            if d1 == d2: continue
            if d1[-1] != 'C' and \
                    utils.eucl_dist((float(d1[1]), float(d1[2])), (float(d2[1]), float(d2[2]))) <= 2.0 and \
                    utils.eucl_dist((float(d1[3]), float(d1[4])), (float(d2[3]), float(d2[4]))) <= 2.0:
                test_data.append(d1)
                break

    print "TEST SET LENGTH: " + str(len(test_data))

    for s in test_data:
        XTest.append((float(s[1]), float(s[2]), float(s[3]), float(s[4])))
        YTest.append(float(s[5]))

    return dimX, dimY, XTest, YTest






































































































