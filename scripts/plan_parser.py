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



with open('../data/solutionPlan.txt', 'r') as file:
    data = file.readlines()
    for line in data:
        words = line.split()
        if len(words) > 0:
            if words[0] == "N_ROBOTS:":
                N_ROBOTS = int(words[1])
                line_len = N_ROBOTS*2 + N_ROBOTS-1
            elif words[0] == "COORDINATES_LIST:":
                reading_coords = 1
                continue
            if reading_coords == 1:
                coordinate = []
                if words[0] != ';':
                    for i in range(0, line_len):
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

plan_coordinates = [coords[i:i+N_ROBOTS] for i in range(0, len(coords), N_ROBOTS)] #create a plan of coordinates

#robot moving
for config in robot_moving:
    count = 0
    for j in config:
        if j != 0:
            id_robot_moving.append(count)
        count +=1

plans_id_robot_moving = [id_robot_moving[i:i+2] for i in range(0, len(id_robot_moving), 2)]

#in the first configuration no robot is moving
plans_id_robot_moving.insert(0,[0,0])

#Gernerate the complete plan
plan = zip(plan_coordinates,plans_id_robot_moving)

print plan #stampa il piano corretto e completo (anche i robot che non si muovono)
