#!/usr/bin/env python
# Functions for reading lego structure file, decoding and sorting
import serial
import socket
import time
import random
import copy
import math
import numpy as np
import itertools

import file_decoder as fd

# Values for binary representation of space around a brick in the model
#       0  1  2  3
#   11  b  b  b  b  4
#   10  b  b  b  b  4
#       9  8  7  6

b0 = 1
b1 = 2
b2 = 4
b3 = 8
b4 = 16
b5 = 32
b6 = 64
b7 = 128
b8 = 256
b9 = 512
b10 = 1024
b11 = 2048

place_masks = [771,390,204]
# [case][subcase][mask, xe, ye]
case_masks = [[[4095,0,0]],
              [[1023,1,0],[4047,-1,0]],
              [[975,0,0]],
              [[3327,0,1],[3519,0,-1],[3903,0,-1],[4083,0,1],[4086,0,1],[4092,0,1]],
              [[255,1,1],[447,1,1],[831,1,1],[1011,1,-1],[1014,1,-1],[1020,1,-1],[3279,-1,1],[3471,-1,1],[3855,-1,1],[4035,-1,-1],[4038,-1,-1],[4044,-1,-1]],
              [[3324,0,0],[3510,0,0],[3891,0,0]],
              [[252,1,0],[438,1,0],[819,1,0],[3276,-1,0],[3462,-1,0],[3843,-1,0]],
              [[204,0,0],[390,0,0],[771,0,0]]]

# master function for sorting brick list into a disassemble que, quick sort(not always optimal)
# -identify separate layers
# -sort layers into sub-groups (pickable and non-pickable)
# -after removing pickable, sort non-pickable into sub-groups (pickable and non-pickable)
# -repeat until all pickable
# -brute force sub_groups
# -update picking method
def sort_bricks_ass(bricks,model):
    build_que = list(bricks)

    # separate layers
    layers = [0]
    for i in range(0,len(bricks)-1):
        if bricks[i]['z'] != bricks[i+1]['z']:
            layers.append(i+1)
    layers.append(len(bricks))

    # sort each layer into sub-groups, label end of each sub-group
    sub_groups = []
    for i in range(0,len(layers)-1):
        #sub_groups.append([0])
        #print "layer ", i,": "
        sub_group,build_que[layers[i]:layers[i+1]] = copy.deepcopy(sort_layer_ass(bricks[layers[i]:layers[i+1]],model))
        #for j in range(0,len(sub_group)):
        sub_groups.append(sub_group)

    # optimise picking order of each sub-group
    for i in range(0,len(layers)-1):
        for j in range(0,len(sub_groups[i])-1):
            updated_model = copy.deepcopy(model)
            updated_model = fd.update_model(build_que[layers[i]+sub_groups[i][j+1]:layers[i]+sub_groups[i][-1]],updated_model)
            build_que[layers[i]+sub_groups[i][j]:layers[i]+sub_groups[i][j+1]] = list(brute_force_ass(build_que[layers[i]+sub_groups[i][j]:layers[i]+sub_groups[i][j+1]],updated_model))

    # update picking method
    build_que = list(list_placing(build_que,model))

    return build_que

# master function for brute force sort
# -identify separate layers
# -brute force each layer
# -update picking method
def bf_sort_bricks_ass(bricks,model):
    build_que = list(bricks)

    # separate layers
    layers = [0]
    for i in range(0,len(bricks)-1):
        if bricks[i]['z'] != bricks[i+1]['z']:
            layers.append(i+1)
    layers.append(len(bricks))

    # optimise picking order of each layer
    for i in range(0,len(layers)-1):
        build_que[layers[i]:layers[i+1]] = list(brute_force_ass(build_que[layers[i]:layers[i+1]],model))

    # update picking method
    build_que = list(list_placing(build_que,model))

    return build_que

def sort_layer_ass(sub_bricks,model):
    placeable = 0
    groups = [len(sub_bricks)]
    updated_model = copy.deepcopy(model)
    build_que = copy.deepcopy(sub_bricks)

    #print 'start: '
    #for i in range(0,len(build_que)):
    #            print build_que[i]['x']+build_que[i]['px'],', ',build_que[i]['y']+build_que[i]['py']

    while placeable == 0:
        group, build_que[0:groups[0]], placeable = sort_placeable_ass(build_que[0:groups[0]],updated_model)
        groups.insert(0,group)
        updated_model = fd.update_model(build_que[groups[0]:groups[1]],updated_model)

        print 'placeable: '
        for i in range(groups[0],groups[-1]):
                if build_que[i]['r'] == 0 or build_que[i]['r'] == 180:
                    print build_que[i]['x'],', ',build_que[i]['y']+build_que[i]['p']
                else:
                    print build_que[i]['x']+build_que[i]['p'],', ',build_que[i]['y']
        print 'not placeable: '
        for i in range(0,groups[0]):
                if build_que[i]['r'] == 0 or build_que[i]['r'] == 180:
                    print build_que[i]['x'],', ',build_que[i]['y']+build_que[i]['p']
                else:
                    print build_que[i]['x']+build_que[i]['p'],', ',build_que[i]['y']
        print '\n'

    return groups,build_que

# re-orders a brick list into a valid picking order, label end of each sub-group
def sort_placeable_ass(sub_bricks,model):
    sub_que = copy.deepcopy(sub_bricks)
    place = len(sub_bricks)-1
    nplace = 0
    placeable = 0
    for i in range(0,len(sub_bricks)):                          # sort bricks into pickable and non-pickable, boundary given by result of 'pick'
        constraints = fd.brick_constraints(sub_bricks[i],model)
        #for j in range(0,len(pick_masks)):
        if constraints&place_masks[1] == 0:                      # if constraints allow a certain pick...
            sub_que[place] = copy.deepcopy(sub_bricks[i])
            place -= 1
        elif constraints&place_masks[0] == 0:
            sub_que[place] = copy.deepcopy(sub_bricks[i])
            sub_que[place]['p'] = 0
            place -= 1
        elif constraints&place_masks[2] == 0:
            sub_que[place] = copy.deepcopy(sub_bricks[i])
            sub_que[place]['p'] = 2
            place -= 1

        else:                                                   # if not pickable...
            sub_que[nplace] = copy.deepcopy(sub_bricks[i])
            nplace += 1
    if place != len(sub_bricks)-1:                                               # if no pickable bricks, can't disassemble so return error
        sub_groups = place+1
    else:
        return 'no picks'
    if nplace == 0:                              # if any unpickable bricks, remove pickable from the model, re-sort list of non-pickable 
        placeable = 1
    #print 'pickable: '
    #for i in range(0,sub_groups):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    #print 'not pickable: '
    #for i in range(sub_groups,len(sub_que)):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    return sub_groups, sub_que, placeable

# Computes optimal place order of list by evaluating the cost associated with each possible order
def brute_force_ass(bricks,model):
    cases = math.factorial(len(bricks))
    print "number of permutations: ",cases
    queue = list(bricks)
    min_cost = list_cost(bricks,model)
    for i in range(1,cases):
        #tic = time.time()
        b_list = generate_list(bricks,i)
        #toc = time.time()-tic
        #tic2 = time.time()
        cost = list_cost(b_list,model)
        #toc2 = time.time()-tic2
        if cost < min_cost:
            queue = list(b_list)
            min_cost = cost
        #print cost
        #print "time = ", toc, toc2
    print "min_cost = ", min_cost
    return queue

# re-order the list based on index
def generate_list(bricks,i):
    #tic = time.time()
    b_list = list(bricks)
    #toc = time.time()-tic
    order = list(itertools.permutations(range(0,len(bricks))))[i]
    #tic3 = time.time()
    for j in range(0,len(bricks)):
        b_list[j] = dict(bricks[order[j]])
    #toc3 = time.time()-tic3
    #print toc, toc3
    return b_list

# generate the cost of a list of bricks
def list_cost(bricks,model):
    cost = 0
    #tic1 = time.time()
    updated_model = copy.deepcopy(model)
    #tic2 = time.time()
    k = len(bricks)
    #tic3=0
    #tic4=0
    #tic5=0
    #tic6=0
    for i in range(0,k):
        #tic3 += time.time()
        constraints = fd.brick_constraints(bricks[k-1-i],updated_model)
        #tic4 += time.time()
        cost += brick_cost(constraints)
        #tic5 += time.time()
        updated_model = fd.update_model(bricks[k-1-i:k-i],updated_model)
        #tic6 += time.time()
    #print "times: ",tic2-tic1,tic4-tic3,tic5-tic4,tic6-tic5
    return cost

# use constraints to estimate the cost of a specific action
def brick_cost(constraints):
    # -------------- case 0 -------------- #
    for i in range(0,len(case_masks[0])):
        if constraints & case_masks[0][i][0] == 0:
            return 0
    # -------------- case 1 -------------- #
    for i in range(0,len(case_masks[1])):
        if constraints & case_masks[1][i][0] == 0:
            return 0.2
    # -------------- case 3 -------------- #
    for i in range(0,len(case_masks[3])):
        if constraints & case_masks[3][i][0] == 0:
            return 0.2
    # -------------- case 4 -------------- #
    for i in range(0,len(case_masks[4])):
        if constraints & case_masks[4][i][0] == 0:
            return 0.4
    # -------------- case 2 -------------- #
    for i in range(0,len(case_masks[2])):
        if constraints & case_masks[2][i][0] == 0:
            return 1
    # -------------- case 5 -------------- #
    for i in range(0,len(case_masks[5])):
        if constraints & case_masks[5][i][0] == 0:
            return 1
    # -------------- case 6 -------------- #
    for i in range(0,len(case_masks[6])):
        if constraints & case_masks[6][i][0] == 0:
            return 1.2
    # -------------- case 7 -------------- #
    for i in range(0,len(case_masks[7])):
        if constraints & case_masks[7][i][0] == 0:
            return 2
    return 5

# generate the cost of a list of bricks
def list_placing(bricks,model):
    updated_model = list(model)
    k = len(bricks)
    for i in range(0,k):
        constraints = fd.brick_constraints(bricks[k-1-i],updated_model)
        bricks[k-1-i] = dict(update_placing(bricks[k-1-i],constraints))
        updated_model = fd.update_model(bricks[k-1-i:k-i],updated_model)
    return bricks

# use constraints to redefine placing method
def update_placing(brick,constraints):
    # -------------- case 7 -------------- #
    if constraints & case_masks[7][0][0] == 0:
        brick['p']=2
        brick['xe']=0
        brick['ye']=0
    elif constraints & case_masks[7][1][0] == 0:
        brick['p']=1
        brick['xe']=0
        brick['ye']=0
    elif constraints & case_masks[7][2][0] == 0:
        brick['p']=0
        brick['xe']=0
        brick['ye']=0

    # -------------- case 6 -------------- #
    if constraints & case_masks[6][0][0] == 0:
        brick['p']=2
        brick['xe']=-1
        brick['ye']=0
    elif constraints & case_masks[6][1][0] == 0:
        brick['p']=1
        brick['xe']=-1
        brick['ye']=0
    elif constraints & case_masks[6][2][0] == 0:
        brick['p']=0
        brick['xe']=-1
        brick['ye']=0
    elif constraints & case_masks[6][3][0] == 0:
        brick['p']=2
        brick['xe']=1
        brick['ye']=0
    elif constraints & case_masks[6][4][0] == 0:
        brick['p']=1
        brick['xe']=1
        brick['ye']=0
    elif constraints & case_masks[6][5][0] == 0:
        brick['p']=0
        brick['xe']=1
        brick['ye']=0

    # -------------- case 5 -------------- #
    if constraints & case_masks[5][0][0] == 0:
        brick['p']=0
        brick['xe']=0
        brick['ye']=0
    elif constraints & case_masks[5][1][0] == 0:
        brick['p']=1
        brick['xe']=0
        brick['ye']=0
    elif constraints & case_masks[5][2][0] == 0:
        brick['p']=2
        brick['xe']=0
        brick['ye']=0

    # -------------- case 2 -------------- #
    if constraints & case_masks[2][0][0] == 0:
        brick['p']=1
        brick['xe']=0
        brick['ye']=0

    # -------------- case 4 -------------- #
    if constraints & case_masks[4][0][0] == 0:
        brick['p']=2
        brick['xe']=1
        brick['ye']=-1
    elif constraints & case_masks[4][1][0] == 0:
        brick['p']=1
        brick['xe']=1
        brick['ye']=-1
    elif constraints & case_masks[4][2][0] == 0:
        brick['p']=0
        brick['xe']=1
        brick['ye']=-1
    elif constraints & case_masks[4][3][0] == 0:
        brick['p']=0
        brick['xe']=1
        brick['ye']=1
    elif constraints & case_masks[4][4][0] == 0:
        brick['p']=1
        brick['xe']=1
        brick['ye']=1
    elif constraints & case_masks[4][5][0] == 0:
        brick['p']=2
        brick['xe']=1
        brick['ye']=1
    elif constraints & case_masks[4][6][0] == 0:
        brick['p']=2
        brick['xe']=-1
        brick['ye']=-1
    elif constraints & case_masks[4][7][0] == 0:
        brick['p']=1
        brick['xe']=-1
        brick['ye']=-1
    elif constraints & case_masks[4][8][0] == 0:
        brick['p']=0
        brick['xe']=-1
        brick['ye']=-1
    elif constraints & case_masks[4][9][0] == 0:
        brick['p']=0
        brick['xe']=-1
        brick['ye']=1
    elif constraints & case_masks[4][10][0] == 0:
        brick['p']=1
        brick['xe']=-1
        brick['ye']=1
    elif constraints & case_masks[4][11][0] == 0:
        brick['p']=2
        brick['xe']=-1
        brick['ye']=1

    # -------------- case 3 -------------- #
    if constraints & case_masks[3][0][0] == 0:
        brick['p']=2
        brick['xe']=0
        brick['ye']=-1
    elif constraints & case_masks[3][1][0] == 0:
        brick['p']=1
        brick['xe']=0
        brick['ye']=-1
    elif constraints & case_masks[3][2][0] == 0:
        brick['p']=0
        brick['xe']=0
        brick['ye']=-1
    elif constraints & case_masks[3][3][0] == 0:
        brick['p']=0
        brick['xe']=0
        brick['ye']=1
    elif constraints & case_masks[3][4][0] == 0:
        brick['p']=1
        brick['xe']=0
        brick['ye']=1
    elif constraints & case_masks[3][5][0] == 0:
        brick['p']=2
        brick['xe']=0
        brick['ye']=1

    # -------------- case 1 -------------- #
    if constraints & case_masks[1][0][0] == 0:
        brick['p']=1
        brick['xe']=1
        brick['ye']=0
    elif constraints & case_masks[1][1][0] == 0:
        brick['p']=1
        brick['xe']=-1
        brick['ye']=0

    # -------------- case 0 -------------- #
    if constraints & case_masks[0][0][0] == 0:
        brick['p']=1
        brick['xe']=0
        brick['ye']=0


    return brick
