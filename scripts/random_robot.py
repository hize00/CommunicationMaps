#!/usr/bin/python

import random


class GenericRobot(object):
    def __init__(self, isLeader, idRobot):
        self.isLeader = isLeader
        self.idRobot = idRobot

class Leader(GenericRobot):


class Follower(GenericRobot):


if __name__ == '__main__':

    isLeader = 0
    idRobot = [1,2,3,4,5,6,7,8,9,10]

    robot_set = []

    #per ora creo tre configurazioni
    configuration1 = [] #configurazione singola di robot
    configuration2 = [] #configurazione singola di robot
    configuration3 = [] #configurazione singola di robot
    configuration_set = [] #insieme delle configurazioni

    for id in idRobot:
        robot_set.append([isLeader, id])

    random.shuffle(robot_set)

    robot_set[0][0] = 1 #il primo robot del set diventa leader

    #creo random le configurazioni di robot
    for robot in robot_set:
        x = random.random()
        if x>0.0 and x<0.3:
           configuration1.append(robot)
        elif x>0.3 and x<0.6:
            configuration2.append(robot)
        else:
           configuration3.append(robot)


    configuration_set.append(configuration1)
    configuration_set.append(configuration2)
    configuration_set.append(configuration3)

    for configuration in configuration_set:
        print configuration

    #il leader decide le destinazioni degli altri robot



