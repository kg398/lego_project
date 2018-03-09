#!/usr/bin/env python
# Functions for reading lego structure file, decoding and sorting
import serial
import socket
import time
import random
import copy
import math
import numpy as np

import file_decoder as fd

# Values for binary representation of space around a brick in the model
#       0  1  2  3
#   11  b  b  b  b  4
#   10  b  b  b  b  4
#       9  8  7  6

# mask free neighbouring spaces, separated by brick type
# [mask,p,r(0),r(90)]
pick_masks = [[[207,2,180,0],[783,0,180,0],[963,0,0,180],[972,2,0,180],[399,1,180,0],[966,1,0,180]],    # standard picks
              [[51,0,0],[204,0,0]],                                                                     # 2x2 picks
              [[15,3,180,0],[960,3,0,180]]]                                                             # other pick

# master function for sorting brick list into a disassemble que, quick sort(not always optimal)
# -identify separate layers
# -sort layers into sub-groups (pickable and pickable after previous groups picked)
# -reverse layer order (disassemble last layer first
# -find best picking method for each brick
def sort_bricks_dis(bricks,model):
    build_que = copy.deepcopy(bricks)
    layers = [0]
    for i in range(0,len(bricks)-1):                            # separate layers
        if bricks[i]['z'] != bricks[i+1]['z']:
            layers.append(i+1)
    layers.append(len(bricks))

    sub_groups = []
    for i in range(0,len(layers)-1):                            # sort each layer into sub-groups, label end of each sub-group
        sub_groups.append([0])
        #print "layer ", i,": "
        sub_group,build_que[layers[i]:layers[i+1]] = copy.deepcopy(sort_layer_dis(bricks[layers[i]:layers[i+1]],model))
        if sub_group == 'n':
            return build_que[layers[i]:layers[i+1]],'n'
        for j in range(0,len(sub_group)):
            sub_groups.append(sub_group)

    new_que = []
    for i in range(0,len(layers)-1):                            # reverse layer order
        for j in range(layers[len(layers)-2-i],layers[len(layers)-1-i]):
            new_que.append(build_que[j])

    build_que = list(list_picking(new_que,model))               # update picking method

    return build_que,'y'

def sort_layer_dis(sub_bricks,model):
    pickable = 0
    groups = [0]
    updated_model = copy.deepcopy(model)
    build_que = copy.deepcopy(sub_bricks)

    #print 'start: '
    #for i in range(0,len(build_que)):
    #            print build_que[i]['x']+build_que[i]['px'],', ',build_que[i]['y']+build_que[i]['py']

    while pickable == 0:
        group, build_que[groups[-1]:len(sub_bricks)], pickable = sort_pickable_dis(build_que[groups[-1]:len(sub_bricks)],updated_model)
        if group == 0:
            return 'n',build_que[groups[-1]:len(sub_bricks)]
        groups.append(groups[-1]+group)
        updated_model = fd.update_model(build_que[groups[-2]:groups[-1]],updated_model)

        #print 'pickable: '
        #for i in range(groups[-2],groups[-1]):
        #        print build_que[i]['x']+build_que[i]['px'],', ',build_que[i]['y']+build_que[i]['py']
        #print 'not pickable: '
        #for i in range(groups[-1],len(sub_bricks)):
        #        print build_que[i]['x']+build_que[i]['px'],', ',build_que[i]['y']+build_que[i]['py']

    return groups,build_que

# re-orders a brick list into a valid picking order, label end of each sub-group
def sort_pickable_dis(sub_bricks,model):
    sub_que = copy.deepcopy(sub_bricks)
    pick = 0
    npick = len(sub_bricks)-1
    pickable = 0
    for i in range(0,len(sub_bricks)):                              # sort bricks into pickable and non-pickable, boundary given by result of 'pick'
        constraints = fd.brick_constraints(sub_bricks[i],model)
        if sub_bricks[i]['b']==0:
            flag = 0
            # -----------------comment out for flexible assembly-------------------
            for j in range(0,len(pick_masks[2])):                   # if constraints allow a tool pick...
                if constraints&pick_masks[2][j][0] == 0:           
                    sub_que[pick] = copy.deepcopy(sub_bricks[i])
                    flag = 1
            # ---------------------------------------------------------------------

            for j in range(0,len(pick_masks[0])):                   # if constraints allow a normal pick...
                if constraints&pick_masks[0][j][0] == 0:            
                    sub_que[pick] = copy.deepcopy(sub_bricks[i])
                    flag = 1

            if flag == 1:
                pick += 1
            else:                                                   # if not pickable...
                sub_que[npick] = copy.deepcopy(sub_bricks[i])
                npick -= 1

        elif sub_bricks[i]['b']==1:
            flag = 0
            for j in range(0,len(pick_masks[1])):                   # if constraints allow a 2x2 pick...
                if constraints&pick_masks[1][j][0] == 0:         
                    sub_que[pick] = copy.deepcopy(sub_bricks[i])
                    pick += 1
                    flag = 1

            if flag == 1:                           
                pick += 1
            else:                                                   # if not pickable...
                sub_que[npick] = copy.deepcopy(sub_bricks[i])
                npick -= 1

    sub_groups = pick

    if npick == len(sub_bricks)-1: 
        pickable = 1
    #print 'pickable: '
    #for i in range(0,sub_groups):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    #print 'not pickable: '
    #for i in range(sub_groups,len(sub_que)):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    return sub_groups, sub_que, pickable

# update picking method for optimised disassembly queue
def list_picking(bricks,model):
    updated_model = list(model)
    P = 1
    k = len(bricks)
    for i in range(0,k):
        constraints = fd.brick_constraints(bricks[i],updated_model)
        bricks[i] = dict(update_placing(bricks[i],constraints))
        updated_model = fd.update_model(bricks[i:i+1],updated_model)
    return bricks

# update picking method for brick given immediate constraints
def update_placing(brick,constraints):
    if brick['b']==0:
        for j in range(0,len(pick_masks[2])):
            if constraints&pick_masks[2][j][0] == 0:                      # if constraints allow a tool pick...
                brick['p'] = pick_masks[2][j][1]
                if brick['r'] == 0 or brick['r'] == 180:
                    brick['r'] = pick_masks[2][j][2]
                if brick['r'] == 90 or brick['r'] == 270:
                    brick['r'] = 90 + pick_masks[2][j][3]

        for j in range(0,len(pick_masks[0])):
            if constraints&pick_masks[0][j][0] == 0:                      # if constraints allow a normal pick...
                brick['p'] = pick_masks[0][j][1]
                if brick['r'] == 0 or brick['r'] == 180:
                    brick['r'] = pick_masks[0][j][2]
                if brick['r'] == 90 or brick['r'] == 270:
                    brick['r'] = 90 + pick_masks[0][j][3]

    elif brick['b']==1:
        for j in range(0,len(pick_masks[1])):
            if constraints&pick_masks[1][j][0] == 0:                      # if constraints allow a 2x2 pick...
                brick['p'] = pick_masks[1][j][1]
                if brick['r'] == 0 or brick['r'] == 180:
                    brick['r'] = pick_masks[1][j][2]
                if brick['r'] == 90 or brick['r'] == 270:
                    brick['r'] = 90 + pick_masks[1][j][3]

    return brick

