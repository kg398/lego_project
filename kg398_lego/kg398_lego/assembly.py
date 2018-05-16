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

# 2x4 bricks
place_masks = [771,390,204]
# [case][subcase][mask,p , xe, ye, cost]
case_masks = [[[4095,1,0,0,0]],#0
              [[1023,1,1,0,0.2],[4047,1,-1,0,0.2]],#1
              [[3327,2,0,-1,0.2],[3519,1,0,-1,0.2],[3903,0,0,-1,0.2],[4083,0,0,1,0.2],[4086,1,0,1,0.2],[4092,2,0,1,0.2]],#3
              [[255,2,1,-1,0.4],[447,1,1,-1,0.4],[831,0,1,-1,0.4],[1011,0,1,1,0.4],[1014,1,1,1,0.4],[1020,2,1,1,0.4],[3279,2,-1,-1,0.4],[3471,1,-1,-1,0.4],[3855,0,-1,-1,0.4],[4035,0,-1,1,0.4],[4038,1,-1,1,0.4],[4044,2,-1,1,0.4]],#4
              [[975,1,0,0,1]],#2
              [[3324,2,0,0,1],[3510,1,0,0,1],[3891,0,0,0,1]],#5
              [[3135,3,0,-1,1.1],[4080,3,0,1,1.1]],#tool 0
              [[63,3,1,-1,1.2],[1008,3,1,1,1.2],[4032,3,-1,1,1.2],[3087,3,-1,-1,1.2]],#tool 1
              [[207,2,0,-1,1.2],[399,1,0,-1,1.2],[783,0,0,-1,1.2],[963,0,0,1,1.2],[966,1,0,1,1.2],[972,2,0,1,1.2]],#7
              [[252,2,1,0,1.2],[438,1,1,0,1.2],[819,0,1,0,1.2],[3276,2,-1,0,1.2],[3462,1,-1,0,1.2],[3843,0,-1,0,1.2]],#6
              [[15,3,0,-1,1.5],[960,3,0,1,1.5]],#tool 2
              [[204,2,0,0,2],[390,1,0,0,2],[771,0,0,0,2]],#8
              [[0,3,0,0,5]]]#tool 3

# 2x2 bricks
place_masks1 = [252,243,207,63]#[204,51] swap for looser constraints
# [case][subcase][mask,p , xe, ye, r, cost]
case_masks1 = [[[255,0,0,0,0,0]],#0
               [[63,0,1,0,90,0.2],[207,0,0,-1,0,0.2],[243,0,-1,0,90,0.2],[252,0,0,1,0,0.2]],#1
               #[[51,0,0,0,90,1],[204,0,0,0,0,1]],#2 uncomment for looser constraints
               [[15,3,1,-1,0,1.2],[60,3,1,1,0,1.2],[240,3,-1,1,0,1.2],[195,3,-1,-1,0,1.2]],#tool 0
               ]#[[3,3,0,-1,0,2],[12,3,1,0,0,2],[48,3,0,1,0,2],[192,3,-1,0,0,2]],#tool 1
               #[[0,3,0,0,0,5]]]#tool 2

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
    opt = 'y'
    sub_groups = []
    for i in range(0,len(layers)-1):
        #sub_groups.append([0])
        #print "layer ", i,": "
        sub_group,build_que[layers[i]:layers[i+1]],opt = copy.deepcopy(sort_layer_ass(bricks[layers[i]:layers[i+1]],model))

        # only for gol
        if opt == 'n':
            return build_que[layers[i]:layers[i+1]],'n'

        sub_groups.append(sub_group)

    # optimise picking order of each sub-group
    for i in range(0,len(layers)-1):
        for j in range(0,len(sub_groups[i])-1):
            updated_model = copy.deepcopy(model)
            updated_model = fd.update_model(build_que[layers[i]+sub_groups[i][j+1]:layers[i]+sub_groups[i][-1]],updated_model)
            build_que[layers[i]+sub_groups[i][j]:layers[i]+sub_groups[i][j+1]] = list(rnd_force_ass(build_que[layers[i]+sub_groups[i][j]:layers[i]+sub_groups[i][j+1]],updated_model))

    # update picking method
    build_que = list(list_placing(build_que,model))

    return build_que,opt#,p #temp delete p

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
        if group == len(build_que[0:groups[0]]): # if all bricks are unplaceable by std method
            # additional tool sort for gol reassembly
            group, build_que[0:groups[0]], placeable = sort_placeable_tool(build_que[0:groups[0]],updated_model)
            if group == len(build_que[0:groups[0]]): # if all bricks are unplaceable by tool method (only true if some cases are removed)
                return group,build_que[0:groups[0]],'n'

            # std tool reassembly
            #groups.insert(0,0)
            #return groups,build_que,'n'

            # reassembly without tool
            #return 'n',build_que[0:groups[0]]
        groups.insert(0,group)
        updated_model = fd.update_model(build_que[groups[0]:groups[1]],updated_model)

        #print 'placeable: '
        #for i in range(groups[0],groups[-1]):
        #        if build_que[i]['r'] == 0 or build_que[i]['r'] == 180:
        #            print build_que[i]['x'],', ',build_que[i]['y']+build_que[i]['p']
        #        else:
        #            print build_que[i]['x']+build_que[i]['p'],', ',build_que[i]['y']
        #print 'not placeable: '
        #for i in range(0,groups[0]):
        #        if build_que[i]['r'] == 0 or build_que[i]['r'] == 180:
        #            print build_que[i]['x'],', ',build_que[i]['y']+build_que[i]['p']
        #        else:
        #            print build_que[i]['x']+build_que[i]['p'],', ',build_que[i]['y']
        #print '\n'

    return groups,build_que,'y'

# re-orders a brick list into a valid picking order, label end of each sub-group
def sort_placeable_ass(sub_bricks,model):
    sub_que = copy.deepcopy(sub_bricks)
    place = len(sub_bricks)-1
    nplace = 0
    placeable = 0
    for i in range(0,len(sub_bricks)):                              # sort bricks into pickable and non-pickable, boundary given by result of 'pick'
        constraints = fd.brick_constraints(sub_bricks[i],model)
        if sub_bricks[i]['b']==0:
            if constraints&place_masks[1] == 0:                     # if constraints allow a certain pick...
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

        elif sub_bricks[i]['b']==1:
            if constraints&place_masks1[3] == 0:                    # if constraints allow a certain pick...
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                sub_que[place]['p'] = 0
                place -= 1
            elif constraints&place_masks1[2] == 0:
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                sub_que[place]['p'] = 0
                place -= 1
            elif constraints&place_masks1[1] == 0:
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                sub_que[place]['p'] = 0
                place -= 1
            elif constraints&place_masks1[0] == 0:
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                sub_que[place]['p'] = 0
                place -= 1

            else:                                                   # if not pickable...
                sub_que[nplace] = copy.deepcopy(sub_bricks[i])
                nplace += 1

    #if place != len(sub_bricks)-1:                                               # if no pickable bricks, can't disassemble so return error
    sub_groups = place+1
    #else:
    #    return 'no picks'
    if nplace == 0:                              # if any unpickable bricks, remove pickable from the model, re-sort list of non-pickable 
        placeable = 1
    #print 'pickable: '
    #for i in range(0,sub_groups):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    #print 'not pickable: '
    #for i in range(sub_groups,len(sub_que)):
    #            print sub_que[i]['x']+sub_que[i]['px'],', ',sub_que[i]['y']+sub_que[i]['py']
    return sub_groups, sub_que, placeable

# re-orders a brick list into a valid picking order, label end of each sub-group
# only for game of life (all 2x2 bricks)
def sort_placeable_tool(sub_bricks,model):
    sub_que = copy.deepcopy(sub_bricks)
    place = len(sub_bricks)-1
    nplace = 0
    placeable = 0
    for i in range(0,len(sub_bricks)):                              # sort bricks into pickable and non-pickable, boundary given by result of 'pick'
        constraints = fd.brick_constraints(sub_bricks[i],model)

        if sub_bricks[i]['b']==1:
            if constraints&case_masks1[2][0][0] == 0:                    # if constraints allow a certain pick..., change 2 -> 3 for looser constraints
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                place -= 1
            elif constraints&case_masks1[2][1][0] == 0:
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                place -= 1
            elif constraints&case_masks1[2][2][0] == 0:                    # if constraints allow a certain pick...
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                place -= 1
            elif constraints&case_masks1[2][3][0] == 0:
                sub_que[place] = copy.deepcopy(sub_bricks[i])
                place -= 1

            else:                                                   # if not pickable...
                sub_que[nplace] = copy.deepcopy(sub_bricks[i])
                nplace += 1

    #if place != len(sub_bricks)-1:                                               # if no pickable bricks, can't disassemble so return error
    sub_groups = place+1
    #else:
    #    return 'no picks'
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
    order = range(0,len(bricks))
    for i in range(1,cases):
        #tic = time.time()
        order = generate_order(order,i)
        b_list = generate_list(bricks,order)
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

# Computes optimal place order of list by evaluating the cost associated with each possible order
def rnd_force_ass(bricks,model):
    permutations = math.factorial(len(bricks))
    if len(bricks) < 6:
        cases = permutations
    else:
        cases = 200
    print "number of permutations: ",cases
    queue = list(bricks)
    min_cost = list_cost(bricks,model)
    order = range(0,len(bricks))
    for i in range(1,cases):
        #tic = time.time()
        order = generate_order(order,i)
        #toc = time.time()
        b_list = generate_list(bricks,order)
        #toc2 = time.time()
        #tic2 = time.time()
        cost = list_cost(b_list,model)
        #toc2 = time.time()-tic2
        if cost < min_cost:
            queue = list(b_list)
            min_cost = cost
        #print cost
        #print "time = ", toc-tic, toc2-toc
    print "min_cost = ", min_cost
    return queue

# generate list order randomly if large, or specific permutation if small
def generate_order(order,i):
    if len(order) < 6:
        #tic = time.time()
        order = list(itertools.permutations(range(0,len(order))))[i]
        #toc = time.time()
        #print "rnd time = ", toc-tic
    else:
        #tic = time.time()
        random.shuffle(order)
        #toc = time.time()
        #print "perm time = ", toc-tic
    return order

# re-order the list based on order
def generate_list(bricks,order):
    #tic = time.time()
    b_list = list(bricks)
    #toc = time.time()-tic
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
        cost += brick_cost(bricks[k-1-i],constraints)
        #tic5 += time.time()
        updated_model = fd.update_model(bricks[k-1-i:k-i],updated_model)
        #tic6 += time.time()
    #print "times: ",tic2-tic1,tic4-tic3,tic5-tic4,tic6-tic5
    return cost

# use constraints to estimate the cost of a specific action
def brick_cost(brick,constraints):
    if brick['b']==0:
        for i in range(0,9):
            for j in range(0,len(case_masks[i])):
                if constraints & case_masks[i][j][0] == 0:
                    return case_masks[i][j][4]
    
    elif brick['b']==1:
        for i in range(0,3):
            for j in range(0,len(case_masks1[i])):
                if constraints & case_masks1[i][j][0] == 0:
                    return case_masks1[i][j][5]

    return 5

# generate the cost of a list of bricks and update placing method
def list_placing(bricks,model):
    updated_model = list(model)
    P = 1
    k = len(bricks)
    for i in range(0,k):
        constraints = fd.brick_constraints(bricks[k-1-i],updated_model)
        bricks[k-1-i] = dict(update_placing(bricks[k-1-i],constraints)) #temp dict(update_placing) once deleted p
        #P = P*p
        updated_model = fd.update_model(bricks[k-1-i:k-i],updated_model)
    return bricks#,P #temp delete p

# 2x4 bricks
# [case][subcase][mask,p , xe, ye]
#case_masks = [[[4095,1,0,0]],#0
#              [[1023,1,1,0],[4047,1,-1,0]],#1
#              [[3327,2,0,-1],[3519,1,0,-1],[3903,0,0,-1],[4083,0,0,1],[4086,1,0,1],[4092,2,0,1]],#3
#              [[255,2,1,-1],[447,1,1,-1],[831,0,1,-1],[1011,0,1,1],[1014,1,1,1],[1020,2,1,1],[3279,2,-1,-1],[3471,1,-1,-1],[3855,0,-1,-1],[4035,0,-1,1],[4038,1,-1,1],[4044,2,-1,1]],#4
#              [[975,1,0,0]],#2
#              [[3324,2,0,0],[3510,1,0,0],[3891,0,0,0]],#5
#              [[207,2,0,-1],[399,1,0,-1],[783,0,0,-1],[963,0,0,1],[966,1,0,1],[972,2,0,1]],#7
#              [[252,2,1,0],[438,1,1,0],[819,0,1,0],[3276,2,-1,0],[3462,1,-1,0],[3843,0,-1,0]],#6
#              [[204,2,0,0],[390,1,0,0],[771,0,0]]]#8

# 2x2 bricks
# [case][subcase][mask,p , xe, ye, r]
#case_masks1 = [[[255,0,0,0,0]],
#               [[63,0,1,0,90],[207,0,0,-1,0],[243,0,-1,0,90],[252,0,0,1,0]],
#               [[51,0,0,0,0],[204,0,0,0,90]]]
# use constraints to redefine placing method
#Prob = [1,0.99,0.99,0.98,0.91,0.91,0.9,0.9,0.83]
#Prob1 = [1,0.99,0.91]

def update_placing(brick,constraints):
    p = 0
    if brick['b']==0:
        l = len(case_masks)-1
        for i in range(0,l+1):
            for j in range(0,len(case_masks[l-i])):
                if constraints & case_masks[l-i][j][0] == 0:
                    brick['p']=case_masks[l-i][j][1]
                    brick['xe']=case_masks[l-i][j][2]
                    brick['ye']=case_masks[l-i][j][3]
                    #p = Prob[8-i]


    elif brick['b']==1:
        l = len(case_masks1) - 1
        for i in range(0,l+1):
            for j in range(0,len(case_masks1[l-i])):
                if constraints & case_masks1[l-i][j][0] == 0:
                    brick['p']=case_masks1[l-i][j][1]
                    brick['xe']=case_masks1[l-i][j][2]
                    brick['ye']=case_masks1[l-i][j][3]
                    brick['r']=case_masks1[l-i][j][4]
                    #p = Prob[2-i]

    return brick#,p #temp delete p
