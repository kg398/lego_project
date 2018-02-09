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

pick_masks = [207,399,783,963,966,972]

# master function for sorting brick list into a disassemble que, quick sort(not always optimal)
# -identify separate layers
# -sort layers into sub-groups (pickable and non-pickable)
# -after removing pickable, sort non-pickable into sub-groups (pickable and non-pickable)
# -repeat until all pickable
# -optimise sub-groups for reliability
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
        for j in range(0,len(sub_group)):
            sub_groups.append(sub_group)

    new_que = []
    for i in range(0,len(layers)-1):                            # reverse layer order
        for j in range(layers[len(layers)-2-i],layers[len(layers)-1-i]):
            new_que.append(build_que[j])

    #for i in range(0,len(layers)-1):                            # optimise picking order of each sub-group
    #    for j in range(0,len(sub_groups[i])-1):
    #        build_que[layers[i]+sub_groups[i][j]:layers[i]+sub_groups[i][j+1]] = optimise_picking(build_que[layers[i]+sub_groups[i][j]:layers[i]+sub_groups[i][j+1]])
    return new_que

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
    for i in range(0,len(sub_bricks)):                          # sort bricks into pickable and non-pickable, boundary given by result of 'pick'
        constraints = fd.brick_constraints(sub_bricks[i],model)
        #for j in range(0,len(pick_masks)):
        if constraints&pick_masks[1] == 0:                      # if constraints allow a certain pick...
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            if sub_que[pick]['r'] == 0:
                sub_que[pick]['r'] = sub_que[pick]['r'] + 180
            pick += 1
        elif constraints&pick_masks[4] == 0:
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            if sub_que[pick]['r'] == 90:
                sub_que[pick]['r'] = sub_que[pick]['r'] + 180
            pick += 1
        elif constraints&pick_masks[0] == 0:
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            if sub_que[pick]['r'] == 0:
                sub_que[pick]['r'] = sub_que[pick]['r'] + 180
            sub_que[pick]['p'] = 2
            pick += 1
        elif constraints&pick_masks[2] == 0:
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            if sub_que[pick]['r'] == 0:
                sub_que[pick]['r'] = sub_que[pick]['r'] + 180
            sub_que[pick]['p'] = 0
            pick += 1
        elif constraints&pick_masks[3] == 0:
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            if sub_que[pick]['r'] == 90:
                sub_que[pick]['r'] = sub_que[pick]['r'] + 180
            sub_que[pick]['p'] = 0
            pick += 1
        elif constraints&pick_masks[5] == 0:
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            if sub_que[pick]['r'] == 90:
                sub_que[pick]['r'] = sub_que[pick]['r'] + 180
            sub_que[pick]['p'] = 2
            pick += 1

        else:                                                   # if not pickable...
            sub_que[npick] = copy.deepcopy(sub_bricks[i])
            npick -= 1
    if pick != 0:                                               # if no pickable bricks, can't disassemble so return error
        sub_groups = pick
    else:
        return 'no picks'
    if npick == len(sub_bricks)-1:                              # if any unpickable bricks, remove pickable from the model, re-sort list of non-pickable 
        pickable = 1
    #print 'pickable: '
    #for i in range(0,sub_groups):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    #print 'not pickable: '
    #for i in range(sub_groups,len(sub_que)):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    return sub_groups, sub_que, pickable

