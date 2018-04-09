#!/usr/bin/env python
# Game of life functions
import serial
import socket
import time
import random
import copy
import math
import numpy as np

import ur_interface_cmds as ic
import ur_waypoints as wp
import lego_moves as lm
import file_decoder as fd
import assembly as ass
import disassembly as dis
import reassembly as rea

def game_of_life(c,ser_ee,nbricks=10,iter=100):
    bricks,model = initialise_grid(nbricks)

    print "------------ initial state ------------"
    print "x",","," y"
    for i in range(0,len(bricks)):
        print bricks[i]['x'],', ',bricks[i]['y']

    que,opt = ass.sort_bricks_ass(copy.deepcopy(bricks),copy.deepcopy(model))
    print "\nAssembly sort output: ",opt
    print "-------------- queue --------------"
    #print "x",","," y",","," p",",","  r",","," ex",","," ey"
    print "x\ty\tp\tr\tex\tey"
    for i in range(0,len(que)):
        print que[i]['x'],'\t',que[i]['y'],'\t',que[i]['p'],'\t',que[i]['r'],'\t',que[i]['ye'],'\t',que[i]['xe']
    lm.assemble(c,ser_ee,que)

    # ------------------------take photo here----------------------------

    for i in range(0,iter):
        time.sleep(20)
        new_bricks,new_model = update_grid(bricks,copy.deepcopy(model))
        dis_list, ass_list = rea.reassemble(bricks,new_bricks,copy.deepcopy(model),copy.deepcopy(new_model))
        print "-------------- new state --------------"
        for i in range(0,len(new_bricks)):
            print new_bricks[i]['x'],', ',new_bricks[i]['y']

        #raw_input("Continue?")

        que,opt = dis.sort_bricks_dis(dis_list,copy.deepcopy(model))
        print "\nDisassembly sort output: ",opt
        print "-------------- queue --------------"
        #print "x",","," y",","," p",",","  r",","," ex",","," ey"
        print "x\ty\tp\tr\tex\tey"
        for i in range(0,len(que)):
            print que[i]['x'],'\t',que[i]['y'],'\t',que[i]['p'],'\t',que[i]['r'],'\t',que[i]['ye'],'\t',que[i]['xe']
        lm.disassemble(c,ser_ee,que)

        que,opt = ass.sort_bricks_ass(ass_list,copy.deepcopy(new_model))
        print "\nAssembly sort output: ",opt
        print "-------------- queue --------------"
        #print "x",","," y",","," p",",","  r",","," ex",","," ey"
        print "x\ty\tp\tr\tex\tey"
        for i in range(0,len(que)):
            print que[i]['x'],'\t',que[i]['y'],'\t',que[i]['p'],'\t',que[i]['r'],'\t',que[i]['ye'],'\t',que[i]['xe']
        lm.assemble(c,ser_ee,que)

        # ------------------------take photo here----------------------------
        
        if len(new_bricks) == 0:
            print 'No bricks remaining'
            break
        bricks = copy.deepcopy(new_bricks)
        model = copy.deepcopy(new_model)
    return

def initialise_grid(nbricks):
    # generate random list
    bricks = []
    i = 0
    while i < nbricks:
        x = 2*random.randint(0,15)
        y = 2*random.randint(0,7)
        flag = 0
        for j in range(0,len(bricks)):
            if x == bricks[j]['x'] and y == bricks[j]['y']:
                flag = 1
                break
        if flag == 0:
            bricks.append({'x':x,'y':y,'z':0,'r':0,'p':0,'xe':0,'ye':0,'b':1})
            i+=1

    # populate model with bricks
    model = []
    model.append(copy.deepcopy(fd.grid_space))
    for i in range(0,len(bricks)):
        model[0][bricks[i]['y']][bricks[i]['x']] = i+1
        model[0][bricks[i]['y']][bricks[i]['x']+1] = i+1
        model[0][bricks[i]['y']+1][bricks[i]['x']] = i+1
        model[0][bricks[i]['y']+1][bricks[i]['x']+1] = i+1

    return bricks, model


def update_grid(bricks,model):
    # all states in next time step
    new_bricks = []
    for y in range(0,16,2):
        for x in range(0,32,2):
            state = 0
            for i in range(0,len(bricks)):
                if x == bricks[i]['x'] and y == bricks[i]['y']:
                    state = 1
                    break
            next_state = update_state(x,y,model,state)
            if next_state == 1:
                new_bricks.append({'x':x,'y':y,'z':0,'r':0,'p':0,'xe':0,'ye':0,'b':1})
 
    # populate new_model with bricks
    new_model = []
    new_model.append(copy.deepcopy(fd.grid_space))
    for i in range(0,len(new_bricks)):
        new_model[0][new_bricks[i]['y']][new_bricks[i]['x']] = i+1
        new_model[0][new_bricks[i]['y']][new_bricks[i]['x']+1] = i+1
        new_model[0][new_bricks[i]['y']+1][new_bricks[i]['x']] = i+1
        new_model[0][new_bricks[i]['y']+1][new_bricks[i]['x']+1] = i+1

    return new_bricks,new_model


def update_state(x,y,model,state):
    neighbours = 0
    if x > 0 and y > 0:
        if model[0][y-1][x-1] != 0:
            neighbours += 1
    if x > 0:
        if model[0][y][x-1] != 0:
            neighbours += 1
    if x > 0 and y < 14:
        if model[0][y+2][x-1] != 0:
            neighbours += 1
    if y < 14:
        if model[0][y+2][x] != 0:
            neighbours += 1
    if x < 30 and y < 14:
        if model[0][y+2][x+2] != 0:
            neighbours += 1
    if x < 30:
        if model[0][y][x+2] != 0:
            neighbours += 1
    if x < 30 and y > 0:
        if model[0][y-1][x+2] != 0:
            neighbours += 1
    if y > 0:
        if model[0][y-1][x] != 0:
            neighbours += 1

    next_state = 0
    if state == 0 and neighbours == 3:
        next_state = 1
    if state == 1 and (neighbours == 2 or neighbours == 3):
        next_state = 1

    return next_state