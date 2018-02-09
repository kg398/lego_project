#!/usr/bin/env python
# Functions for comparing lego structure files, decoding and sorting
import serial
import socket
import time
import random
import copy
import math
import numpy as np
import itertools

import file_decoder as fd

# quick easy bad method
# plan to remove all bricks above lowest difference
def reassemble(bricks_old,bricks_new):

    # find old bricks not existant in new structure, remove all bricks above
    lowest = bricks_old[-1]
    for i in range(0,len(bricks_old)):
        flag = 0
        for j in range(0,len(bricks_new)):
            if bricks_old[i]['x']==bricks_new[j]['x'] and bricks_old[i]['y']==bricks_new[j]['y'] and bricks_old[i]['z']==bricks_new[j]['z'] and bricks_old[i]['r']==bricks_new[j]['r']:
                flag = 1
        if flag == 0: #brick doesn't exist in old structure
            if bricks_new[i]['z']<lowest['z']:
                lowest = bricks_old[i]

    index = len(bricks_old)
    for i in range(0,len(bricks_old)):
        if bricks_old[i]['z'] == lowest['z']+1:
            index = i
            break
    dis_list = list(bricks_old[index:])
    dis_list.insert(0,lowest)

    # find new bricks not existant in old structure, build all bricks above
    lowest = bricks_new[-1]
    for i in range(0,len(bricks_new)):
        flag = 0
        for j in range(0,len(bricks_old)):
            if bricks_new[i]['x']==bricks_old[j]['x'] and bricks_new[i]['y']==bricks_old[j]['y'] and bricks_new[i]['z']==bricks_old[j]['z'] and bricks_new[i]['r']==bricks_old[j]['r']:
                flag = 1
        if flag == 0: #brick doesn't exist in old structure
            if bricks_new[i]['z']<lowest['z']:
                lowest = bricks_new[i]

    index = len(bricks_new)
    for i in range(0,len(bricks_new)):
        if bricks_new[i]['z'] == lowest['z']+1:
            index = i
            break
    ass_list = list(bricks_new[index:])
    ass_list.insert(0,lowest)

    return dis_list, ass_list