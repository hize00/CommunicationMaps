#!/usr/bin/env python

import sys

coord = []
plan_coords = []
plan_coordinates = []
robot_moving = []
id_robot_moving = []
robots = []
plans_id_robot_moving = []
plan = []

reading_coords = 0
reading_RM = 0


with open('../data/solution_plan_2_robots.txt', 'r') as file:
    data = file.readlines()
    for line in data:
        words = line.split()
        if len(words) > 0:
            if words[0] == "N_ROBOTS:":
                N_ROBOTS = int(words[1])
                line_lenght = N_ROBOTS*2 + N_ROBOTS-1
            elif words[0] == "COORDINATES_LIST:":
                reading_coords = 1
                continue
            if reading_coords == 1:
                coordinate = []
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

#coordinates
coords = [coord[i:i+2] for i in range(0, len(coord), 2)] #group x and y of a single robot
nested_tuple_coords = [tuple(l) for l in coords]

plan_coordinates = [nested_tuple_coords[i:i+N_ROBOTS] for i in range(0, len(nested_tuple_coords), N_ROBOTS)] #create a plan of coordinates

#creating a tuples of coordinates
tuple_plan_coordinates = tuple(plan_coordinates)
nested_tuple_plan_coordinates = [tuple(l) for l in tuple_plan_coordinates]

#robot moving
for config in robot_moving:
    print config
    count = 0
    position = 0
    count_moving = 0
    for j in config:
        if j != 0:
            id_robot_moving.append(count)
            position = count
            count_moving += 1
        count += 1

    if count_moving != 0 and count_moving < 2 and position == N_ROBOTS-1:
        id_robot_moving.insert(-1, -1)
    elif count_moving != 0 and count_moving < 2 and position != N_ROBOTS:
        id_robot_moving.append(-1)


plans_id_robot_moving = [id_robot_moving[i:i+2] for i in range(0, len(id_robot_moving), 2)]
tuple_id__robot_moving = [tuple(l) for l in plans_id_robot_moving]


#in the first configuration no robot is moving
tuple_id__robot_moving.insert(0,(-1,-1))
print tuple_id__robot_moving


#Generating the complete plan
plan = zip(nested_tuple_plan_coordinates,tuple_id__robot_moving)
tuple_plan = tuple(plan)

#stampa il piano corretto e completo (anche i robot che non si muovono)
