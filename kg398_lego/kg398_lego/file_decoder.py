#!/usr/bin/env python
# Functions for reading lego structure file, decoding and sorting
import serial
import socket
import time
import random
import copy
import math
import numpy as np

# Values for binary representation of space around a brick in the model
bit0 = 1
bit1 = 2
bit2 = 4
bit3 = 8
bit4 = 16
bit5 = 32
bit6 = 64
bit7 = 128
bit8 = 256
bit9 = 512
bit10 = 1024
bit11 = 2048

# empty grid layer, x = 32, y = 16, z = arbitrary
grid_space = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
              [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]

# Reads file 'name' and decodes into a model array
# Returns model array
def import_file(name):
    f = open(name,"r")
    file_string = f.read()
    f.close()
    height = int(file_string[0])                                # intial value gives number of layers
    model = [copy.deepcopy(grid_space)]
    for i in range(0,height-1):                                 # empty model array of corresponding height is created
        model.append(copy.deepcopy(grid_space))
    for z in range(0,height):                                   # model array is populated with bricks given by structure file
        for y in range(0,16):
            for x in range(0,32):
                model[z][y][x] = int(file_string[3+x+33*y+(33*16+1)*z]) #3 start characters, 33 characters/line(y), 33*16+1 characters/layer(z)
    return model

# Reads model array and decodes into a list of bricks
# Brick info stored as a dictionary: {'x','y','z','r','px','py'}, locatiion in space (x,y,z,r), picking coords (px,py,z,r)
# Returns list of bricks
def decode_file(model):
    bricks = []
    for z in range(0,len(model)):
        for y in range(0,16):
            for x in range(0,32):
                if model[z][y][x]!=0:                           # find brick by comparing corners
                    if model[z][y+3][x+1] == model[z][y][x]:    # vertical brick, 'r' = 0 and default picking location in centre of brick
                        bricks.append({'x':x,'y':y,'z':z,'r':0,'px':x,'py':y+1})
                    if model[z][y+1][x+3] == model[z][y][x]:    # horizontal brick, 'r' = 90 and default picking location in centre of brick
                        bricks.append({'x':x,'y':y,'z':z,'r':90,'px':x+1,'py':y})
    return bricks

# master function for sorting brick list into an assemble que
# -identify separate layers
# -sort layers into sub-groups (placeable and non-placeable)
# -after removing placeable, sort non-placeable into sub-groups (placeable and non-placeable)
# -repeat until all placeable
# -optimise sub-groups for reliability
def sort_bricks_as(bricks):
    build_que = copy.deepcopy(bricks)
    return build_que


# master function for sorting brick list into a disassemble que, quick sort(not always optimal)
# -identify separate layers
# -sort layers into sub-groups (pickable and non-pickable)
# -after removing pickable, sort non-pickable into sub-groups (pickable and non-pickable)
# -repeat until all pickable
# -optimise sub-groups for reliability
def sort_bricks_dis(bricks,model):
    build_que = copy.deepcopy(bricks)                           # copy structure
    layers = [0]
    for i in range(0,len(bricks)-1):                            # parse list for all bricks in same layers, store start positon of each layer in list
        if bricks[i]['z'] != bricks[i+1]['z']:
            layers.append(i+1)
    layers.append(len(bricks))                                  # add final position for end of last layer

    sub_groups = [[0]]
    for i in range(0,len(layers)-1):                            # sort each layer into sub-groups, label end of each sub-group
        print "layer ", i,": "
        sub_group,build_que[layers[i]:layers[i+1]] = copy.deepcopy(sort_pickable(bricks[layers[i]:layers[i+1]],model))
        for j in range(0,len(sub_group)):
            sub_groups[i].append(sub_group[j])

    for i in range(0,len(layers)-1):                            # optimise picking order of each sub-group
        for j in range(0,len(sub_groups[i]):
            build_que[layers[i]+sub_groups[j]:layers[i]+sub_groups[j+1]] = optimise_picking(build_que[layers[i]+sub_groups[j]:layers[i]+sub_groups[j+1]])
    return build_que

# re-orders a brick list into a valid picking order, label end of each sub-group
def sort_pickable(sub_bricks,model):
    sub_que = copy.deepcopy(sub_bricks)
    sub_groups = [[]]
    pick = 0
    npick = len(sub_bricks)-1
    for i in range(0,len(sub_bricks)):                          # sort bricks into pickable and non-pickable, boundary given by result of 'pick'
        constraints = brick_constraints(sub_bricks[i],model)
        if constraints: #meet requirements:
            sub_que[pick] = copy.deepcopy(sub_bricks[i])
            pick += 1
        else:
            sub_que[npick] = copy.deepcopy(sub_bricks[i])
            npick -= 1
    if pick != 0:                                               # if no pickable bricks, can't disassemble so return error
        sub_groups.append(pick)
    else:
        return 'no picks'
    if npick != len(sub_bricks)-1:                              # if any unpickable bricks, remove pickable from the model, re-sort list of non-pickable 
        updated_model = update_model(sub_que[0:pick],model)
        sub_sub_groups, sub_que[npick:len(sub_bricks)-1] = sort_pickable(sub_que[npick:len(sub_bricks)-1],updated_model)
        sub_groups.append(sub_sub_groups)
    return sub_groups, sub_que

# optimises picking order of list, assumes any order is valid
def optimise_picking(sub_bricks):
    sub_que = copy.deepcopy(sub_bricks)
    print sub_que
    return sub_que

# compute binary value of the surrounding bits of a brick
def brick_constraints(brick,model):
    constraints = 0
    if brick['r'] = 0 or brick['r'] = 180:
        if brick['x'] < 30:
            if model[brick['z']][brick['y']][brick['x']+2] != 0:
                constraints += bit0
            if model[brick['z']][brick['y']+1][brick['x']+2] != 0:
                constraints += bit1
            if model[brick['z']][brick['y']+2][brick['x']+2] != 0:
                constraints += bit2
            if model[brick['z']][brick['y']+3][brick['x']+2] != 0:
                constraints += bit3
        if brick['y'] < 12:
            if model[brick['z']][brick['y']+4][brick['x']+1] != 0:
                constraints += bit4
            if model[brick['z']][brick['y']+4][brick['x']] != 0:
                constraints += bit5
        if brick['x'] > 0:
            if model[brick['z']][brick['y']+3][brick['x']-1] != 0:
                constraints += bit6
            if model[brick['z']][brick['y']+2][brick['x']-1] != 0:
                constraints += bit7
            if model[brick['z']][brick['y']+1][brick['x']-1] != 0:
                constraints += bit8
            if model[brick['z']][brick['y']][brick['x']-1] != 0:
                constraints += bit9
        if brick['y'] > 0:
            if model[brick['z']][brick['y']-1][brick['x']] != 0:
                constraints += bit10
            if model[brick['z']][brick['y']-1][brick['x']+1] != 0:
                constraints += bit11
    elif brick['r'] = 90 or brick['r'] = 270:
        if brick['y'] > 0:
            if model[brick['z']][brick['y']-1][brick['x']] != 0:
                constraints += bit0
            if model[brick['z']][brick['y']-1][brick['x']+1] != 0:
                constraints += bit1
            if model[brick['z']][brick['y']-1][brick['x']+2] != 0:
                constraints += bit2
            if model[brick['z']][brick['y']-1][brick['x']+3] != 0:
                constraints += bit3
        if brick['x'] < 28:
            if model[brick['z']][brick['y']][brick['x']+4] != 0:
                constraints += bit4
            if model[brick['z']][brick['y']+1][brick['x']+4] != 0:
                constraints += bit5
        if brick['y'] < 14:
            if model[brick['z']][brick['y']+2][brick['x']+3] != 0:
                constraints += bit6
            if model[brick['z']][brick['y']+2][brick['x']+2] != 0:
                constraints += bit7
            if model[brick['z']][brick['y']+2][brick['x']+1] != 0:
                constraints += bit8
            if model[brick['z']][brick['y']+2][brick['x']] != 0:
                constraints += bit9
        if brick['x'] > 0:
            if model[brick['z']][brick['y']+1][brick['x']-1] != 0:
                constraints += bit10
            if model[brick['z']][brick['y']][brick['x']-1] != 0:
                constraints += bit11
    return constraints

# remove pickable bricks from a model to sort remaining bricks
def update_model(pickable,model):
    updated_model = copy.deepcopy(model)
    for i in range(0,len(pickable)):
        updated_model[pickable['z']][picakble['y']][pickable['x']] = 0
        updated_model[pickable['z']][picakble['y']+1][pickable['x']] = 0
        updated_model[pickable['z']][picakble['y']][pickable['x']+1] = 0
        updated_model[pickable['z']][picakble['y']+1][pickable['x']+1] = 0
        if pickable['r'] = 0 or pickable['r'] = 180:
            updated_model[pickable['z']][picakble['y']+2][pickable['x']] = 0
            updated_model[pickable['z']][picakble['y']+3][pickable['x']] = 0
            updated_model[pickable['z']][picakble['y']+2][pickable['x']+1] = 0
            updated_model[pickable['z']][picakble['y']+3][pickable['x']+1] = 0
        if pickable['r'] = 90 or pickable['r'] = 270:
            updated_model[pickable['z']][picakble['y']][pickable['x']+2] = 0
            updated_model[pickable['z']][picakble['y']+1][pickable['x']+2] = 0
            updated_model[pickable['z']][picakble['y']][pickable['x']+3] = 0
            updated_model[pickable['z']][picakble['y']+1][pickable['x']+3] = 0
    return updated_model