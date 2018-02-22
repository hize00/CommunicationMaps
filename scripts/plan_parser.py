#!/usr/bin/env python

import sys
import random
import numpy as np

coord = []
robot_moving = []
robot_plan = []
timetable = []

reading_coords = 0
reading_RM = 0
reading_TT = 0

with open('/home/andrea/catkin_ws/src/strategy/data/solution_plan_2_robots.txt', 'r') as file:
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
    #c[0] = float(self.env.dimX - resize_factor * pos[0])
    #c[1] = float(resize_factor * pos[1])

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
robot_plan = [robot_plan[i:i + 5] for i in range(0, len(robot_plan), 5)]

# deleting last (incomplete) plan if last robot_moving row has only one robot to move
final_dest = ()
for plan in robot_plan:
    #print plan
    if len(plan) < 5: #if only one robot or all robots have to go to final destination
        robot_plan.pop(-1)
        final_dest = plan

if final_dest:
    final_dest.append(final_dest[1])
    final_dest.append(final_dest[0])

    robot_plan.append(final_dest) #adding to the complete plan the plan of the robot that has to go to final destination

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

#for i in xrange(10):
#    random_update = random.randint(0, 4)
#    print random_update

#print plan_id

filename = '/home/andrea/Desktop/parser/0_offices_0_4_50.dat'
carlo = True
f = open(filename, "r")
lines = f.readlines()
prev_time = 0
count = 0
for line in lines:
    s = line.split()
    if (carlo and s[-1] == 'C') or (not carlo):
        count += 1
        print s

print count

num_runs = 5
proc = 3
runs = range(num_runs)

for p in range(proc):
    runs = np.array_split(runs, 3)[p]
    #print run









