#!/usr/bin/env python

import sys

coord = []
robot_moving = []
robot_plan = []

resize_factor = 0.1
reading_coords = 0
reading_RM = 0

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

file.close()

#grouping coordinates by (x,y)
coord = [coord[i:i + 2] for i in range(0, len(coord), 2)]  # group x and y of a single robot


#converting from pixels to meters
for c in coord:
    c[0] = int(79.7 - resize_factor * c[0]) #79.7 = self.env.dimX
    c[1] = int(resize_factor * c[1])

coord = [tuple(l) for l in coord]

coord = [coord[i:i + N_ROBOTS] for i in
                    range(0, len(coord), N_ROBOTS)]  # create a plan of coordinates

print coord

#creating the plan
count_config = 0
for config in robot_moving:
    count = 0
    first_robot = -1
    my_self = -1
    for robot in config:
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

            # assigning a reflected plan to the communication teammate
            robot_plan.append(second_robot)
            robot_plan.append(coord[count_config][position])
            robot_plan.append(coord[count_config][first_robot])
            robot_plan.append(first_robot)
        elif count_config == 0 and robot == 0:
            #I add the starting positions of each robot to plan: [my_self,(starting_pose),(starting_pose),my_self]
            my_self = count
            robot_plan.append(my_self)
            position = count
            robot_plan.append(coord[count_config][position])
            robot_plan.append(coord[count_config][position])
            robot_plan.append(my_self)
        count += 1
    count_config += 1


#grouping plan elements: [my_id, (my_coords),(teammate_coords),communication_teammate]
robot_plan = [robot_plan[i:i + 4] for i in range(0, len(robot_plan), 4)]

#deleting last (incomplete) plan if last robot_moving row has only one robot to move
for plan in robot_plan:
    if len(plan) < 4:  # 4 = number of elements in a plan
        robot_plan.pop(-1)
        final_dest = plan

final_dest.append(final_dest[1])
final_dest.append(final_dest[0])

robot_plan.append(final_dest)


#grouping plan elements: [(my_id, (((my_coords), (teammate_coords)), communication_teammate)]
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
    my_id.append(tuple(msgs))
    plans.append(tuple(my_id))

plans = tuple(plans)

# grouping plan elements by robot_id
robot_ids = set(map(lambda x: x[0], plans))
plan_id = [[y[1] for y in plans if y[0] == x] for x in robot_ids]


if len(robot_ids) < N_ROBOTS:
    for i in xrange(N_ROBOTS):
        if i not in robot_ids:
            plan_id.insert(i, ())  #if a robot never moves, his plan will be empty

plan_id = tuple([tuple(l) for l in plan_id])  #plans = (plan_robot_0, plan_robot_1,...,plan_robot_n)

