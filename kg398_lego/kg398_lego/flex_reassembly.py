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
import disassembly as dis

# replace masks
diamond = 3+256+512+1024+2048
square = 15+16+32+1024+2048

def sort_bricks_flex(bricks,model):
    build_que = list(bricks)

    # separate layers
    layers = [0]
    for i in range(0,len(bricks)-1):
        if bricks[i]['z'] != bricks[i+1]['z']:
            layers.append(i+1)
    layers.append(len(bricks))

    # find non-disassemblable
    sub_bricks = []
    remaining_bricks = []
    for i in range(0,len(layers)-1):
        temp_bricks,opt = dis.sort_bricks_dis(bricks[layers[i]:layers[i+1]],copy.deepcopy(model))
        if opt == 'n':
            sub_bricks.append(temp_bricks)
        else:
            sub_bricks.append([])
        for j in range(layers[i],layers[i+1]):
            flag = 0
            for k in range(0,len(temp_bricks)):
                if fd.match_bricks(bricks[j],temp_bricks[k]) == 1:
                    flag = 1
            if flag == 0:
                remaining_bricks.append(bricks[j])
    print "bricks", sub_bricks

    # get replacement bricks for flexible assembly
    updated_model = fd.update_model(remaining_bricks,copy.deepcopy(model))
    changed_model = copy.deepcopy(model)
    for i in range(0,len(sub_bricks)):
        if len(sub_bricks[i])!=0:
            replaced,replacement = get_replaceable(sub_bricks[i],updated_model)
            for j in range(layers[i],layers[i+1]):
                if fd.match_bricks(bricks[j],replaced) == 1:
                    changed_model = fd.update_model([replaced],copy.deepcopy(changed_model))
                    changed_model = change_model(replacement,copy.deepcopy(changed_model))
                    build_que[j] = replacement

    return build_que, changed_model, 'y'

# find brick to be replaced by 2x2 brick
def get_replaceable(sub_bricks,updated_model):
    if len(sub_bricks) == 4:
        for i in range(0,len(sub_bricks)):
            constraints = fd.brick_constraints(sub_bricks[i],updated_model)
            if constraints&diamond == 0:
                replacement = dict(sub_bricks[i])
                replacement['b']=1
                return sub_bricks[i],replacement

    if len(sub_bricks) == 2:
        for i in range(0,len(sub_bricks)):
            constraints = fd.brick_constraints(sub_bricks[i],updated_model)
            if constraints&square == 0:
                if sub_bricks[i]['r'] == 0:
                    replacement = dict(sub_bricks[i])
                    replacement['b']=1
                    replacement['y']+=2
                elif sub_bricks[i]['r'] == 90:
                    replacement = dict(sub_bricks[i])
                    replacement['b']=1
                return sub_bricks[i],replacement

def change_model(replace,changed_model):
    changed_model[replace['z']][replace['y']][replace['x']] = 10
    changed_model[replace['z']][replace['y']+1][replace['x']] = 10
    changed_model[replace['z']][replace['y']][replace['x']+1] = 10
    changed_model[replace['z']][replace['y']+1][replace['x']+1] = 10
    #if replace['r'] == 0 or replace['r'] == 180:
    #    changed_model[replace['z']][replace['y']+2][replace['x']] = 0
    #    changed_model[replace['z']][replace['y']+3][replace['x']] = 0
    #    changed_model[replace['z']][replace['y']+2][replace['x']+1] = 0
    #    changed_model[replace['z']][replace['y']+3][replace['x']+1] = 0
    #if replace['r'] == 90 or replace['r'] == 270:
    #    changed_model[replace['z']][replace['y']][replace['x']+2] = 0
    #    changed_model[replace['z']][replace['y']+1][replace['x']+2] = 0
    #    changed_model[replace['z']][replace['y']][replace['x']+3] = 0
    #    changed_model[replace['z']][replace['y']+1][replace['x']+3] = 0
    return changed_model
