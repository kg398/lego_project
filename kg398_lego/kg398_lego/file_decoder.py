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
#       0  1  2  3
#   11  b  b  b  b  4
#   10  b  b  b  b  4
#       9  8  7  6
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
    if file_string[1] != '\n':
        height = 10*int(file_string[0])+int(file_string[1])               # intial value gives number of layers
        k = 1
    else:
        height = int(file_string[0])                    
        k = 0
    model = [copy.deepcopy(grid_space)]
    for i in range(0,height-1):                                 # empty model array of corresponding height is created
        model.append(copy.deepcopy(grid_space))
    for z in range(0,height):                                   # model array is populated with bricks given by structure file
        for y in range(0,16):
            for x in range(0,32):
                model[z][y][x] = int(file_string[3+k+x+33*y+(33*16+1)*z]) #3 start characters, 33 characters/line(y), 33*16+1 characters/layer(z)
    return model

# Reads model array and decodes into a list of bricks
# Brick info stored as a dictionary: {'x','y','z','r','p','xe','ye','b'}, locatiion in space (x,y,z,r), relative picking coord (p), relative placing offsets (xe,ye), brick type (b)
# Returns list of bricks
def decode_file(model):
    bricks = []
    for z in range(0,len(model)):
        for y in range(0,16):
            for x in range(0,32):
                if model[z][y][x]!=0:                           # find brick by comparing corners
                    # locate top left corner
                    y0 = 0
                    x0 = 0
                    n = 0
                    while y-n > 0:
                        if model[z][y-n][x] != model[z][y][x]:
                            y0 = y-n+1
                            break
                        n+=1
                    n = 0
                    while x-n > 0:
                        if model[z][y0][x-n] != model[z][y0][x]:
                            x0 = x-n+1
                            break
                        n+=1

                    flag = 0
                    for i in range(0,len(bricks)):              # compare with already located bricks
                        if bricks[i]['x'] == x0 and bricks[i]['y'] == y0 and bricks[i]['z'] == z:
                            flag = 1
                            break

                    # compare with opposite conrner for brick info
                    if x0 < 31 and y0 < 13 and flag == 0:
                        if model[z][y0+3][x0+1] == model[z][y0][x0]:    # vertical brick, 'r' = 0 and default picking location in centre of brick
                            bricks.append({'x':x0,'y':y0,'z':z,'r':0,'p':1,'xe':0,'ye':0,'b':0})
                            flag = 1
                    if x0 < 29 and y0 < 15 and flag == 0:
                        if model[z][y0+1][x0+3] == model[z][y0][x0]:    # horizontal brick, 'r' = 90 and default picking location in centre of brick
                            bricks.append({'x':x0,'y':y0,'z':z,'r':90,'p':1,'xe':0,'ye':0,'b':0})
                            flag = 1
                    if x0 < 31 and y0 < 15 and flag == 0:
                        if model[z][y0+1][x0+1] == model[z][y0][x0]:    # 2x2 brick, default picking location in centre of brick
                            bricks.append({'x':x0,'y':y0,'z':z,'r':0,'p':0,'xe':0,'ye':0,'b':1})

    return bricks

# optimises picking order of list, assumes any order is valid
def optimise_picking(sub_bricks):
    sub_que = copy.deepcopy(sub_bricks)
    print sub_que
    return sub_que

# compute binary value of the surrounding bits of a brick
def brick_constraints(brick,model):
    constraints = 0
    if brick['b'] == 0:
        if brick['r'] == 0 or brick['r'] == 180:
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
        elif brick['r'] == 90 or brick['r'] == 270:
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
    if brick['b'] == 1:
        if brick['y'] > 0:
            if model[brick['z']][brick['y']-1][brick['x']] != 0:
                constraints += bit0
            if model[brick['z']][brick['y']-1][brick['x']+1] != 0:
                constraints += bit1
        if brick['x'] < 30:
            if model[brick['z']][brick['y']][brick['x']+2] != 0:
                constraints += bit2
            if model[brick['z']][brick['y']+1][brick['x']+2] != 0:
                constraints += bit3
        if brick['y'] < 14:
            if model[brick['z']][brick['y']+2][brick['x']+1] != 0:
                constraints += bit4
            if model[brick['z']][brick['y']+2][brick['x']] != 0:
                constraints += bit5
        if brick['x'] > 0:
            if model[brick['z']][brick['y']+1][brick['x']-1] != 0:
                constraints += bit6
            if model[brick['z']][brick['y']][brick['x']-1] != 0:
                constraints += bit7
    return constraints

# remove pickable bricks from a model to sort remaining bricks
def update_model(pickable,updated_model):
    #updated_model = copy.deepcopy(model)
    for i in range(0,len(pickable)):
        updated_model[pickable[i]['z']][pickable[i]['y']][pickable[i]['x']] = 0
        updated_model[pickable[i]['z']][pickable[i]['y']+1][pickable[i]['x']] = 0
        updated_model[pickable[i]['z']][pickable[i]['y']][pickable[i]['x']+1] = 0
        updated_model[pickable[i]['z']][pickable[i]['y']+1][pickable[i]['x']+1] = 0
        if pickable[i]['b']==0 and (pickable[i]['r'] == 0 or pickable[i]['r'] == 180):
            updated_model[pickable[i]['z']][pickable[i]['y']+2][pickable[i]['x']] = 0
            updated_model[pickable[i]['z']][pickable[i]['y']+3][pickable[i]['x']] = 0
            updated_model[pickable[i]['z']][pickable[i]['y']+2][pickable[i]['x']+1] = 0
            updated_model[pickable[i]['z']][pickable[i]['y']+3][pickable[i]['x']+1] = 0
        elif pickable[i]['b']==0 and (pickable[i]['r'] == 90 or pickable[i]['r'] == 270):
            updated_model[pickable[i]['z']][pickable[i]['y']][pickable[i]['x']+2] = 0
            updated_model[pickable[i]['z']][pickable[i]['y']+1][pickable[i]['x']+2] = 0
            updated_model[pickable[i]['z']][pickable[i]['y']][pickable[i]['x']+3] = 0
            updated_model[pickable[i]['z']][pickable[i]['y']+1][pickable[i]['x']+3] = 0
    return updated_model

def match_bricks(brick1,brick2):
    if brick1['x']==brick2['x'] and brick1['y']==brick2['y'] and brick1['z']==brick2['z'] and brick1['r']%180==brick2['r']%180:
        return 1
    return 0