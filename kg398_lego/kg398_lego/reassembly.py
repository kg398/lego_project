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
import assembly as ass
import disassembly as dis

# quick easy bad method
# plan to remove all bricks above lowest difference
def reassemble(bricks_old,bricks_new,model_old,model_new):
    dis_list = []
    updated_model = copy.deepcopy(model_old)

    # find old bricks not existant in new structure, remove all bricks above
    lowest = bricks_old[-1]
    for i in range(0,len(bricks_old)):
        flag = 0
        for j in range(0,len(bricks_new)):
            if fd.match_bricks(bricks_old[i],bricks_new[j]) == 1:
                flag = 1
                break
        if flag == 0: #brick doesn't exist in new structure
            dis_list.append(bricks_old[i]) 
            if bricks_old[i]['z']<lowest['z']:
                lowest = bricks_old[i]

    # find lowest new brick
    for i in range(0,len(bricks_new)):
        flag = 0
        for j in range(0,len(bricks_old)):
            if fd.match_bricks(bricks_new[i],bricks_old[j]) == 1:
                flag = 1
                break
        if flag == 0: #brick doesn't exist in old structure
            if bricks_new[i]['z']<lowest['z']:
                lowest = bricks_new[i]

    # find min no. bricks to safely disasseble
    for i in range(0,len(dis_list)):
        constraints = fd.brick_constraints(dis_list[i],copy.deepcopy(updated_model))
        if pickable(dis_list[i]['b'],constraints) != 1:                  # if not pickable: find neighbouring bricks to remove
            remove_bricks = critical_neighbours(constraints,dis_list[i],updated_model)
            for j in range(0,len(remove_bricks)):
                dis_list.append(remove_bricks[j])
            dis_list = copy.deepcopy(remove_duplicates(dis_list))
            updated_model = fd.update_model(remove_bricks,updated_model)
    
            #above_bricks = critical_above

    # remove layers above lowest discrepancy
    index = len(bricks_old)
    for i in range(0,len(bricks_old)):
        if bricks_old[i]['z'] == lowest['z']+1:
            index = i
            break
    dis_list = dis_list + list(bricks_old[index:])
    if len(dis_list)>0:
        dis_list = copy.deepcopy(remove_duplicates(dis_list))
    
    # update bricks_old list
    bricks_intermediate = []
    for i in range(0,len(bricks_old)):
        flag = 0
        for j in range(0,len(dis_list)):
            if fd.match_bricks(bricks_old[i],dis_list[j]) == 1:
                flag = 1
        if flag == 0:
            bricks_intermediate.append(bricks_old[i])

    # find new bricks not existant in intermediate structure, build all
    #lowest = bricks_new[-1]
    ass_list = []
    for i in range(0,len(bricks_new)):
        flag = 0
        for j in range(0,len(bricks_intermediate)):
            if fd.match_bricks(bricks_new[i],bricks_intermediate[j]) == 1:
                flag = 1
        if flag == 0: #brick doesn't exist in old structure
            ass_list.append(bricks_new[i])
            #if bricks_new[i]['z']<lowest['z']:
            #    lowest = bricks_new[i]

    # only for gol
    # if not assemblable, sort will return bricks which cannot be assembled, and 'n', otherwise sorted list and 'y'
    blocking_bricks = []
    que,opt = ass.sort_bricks_ass(ass_list,copy.deepcopy(model_new))
    if opt == 'n' and len(que) != 0:
        for i in range(0,len(que)):
            bb = find_brick(que[i]['x'],que[i]['y']-1,que[i]['z'],model_new)
            if bb != None:
                blocking_bricks.append(bb)
            bb = find_brick(que[i]['x']+2,que[i]['y'],que[i]['z'],model_new)
            if bb != None:
                blocking_bricks.append(bb)
            bb = find_brick(que[i]['x'],que[i]['y']+2,que[i]['z'],model_new)
            if bb != None:
                blocking_bricks.append(bb)
            bb = find_brick(que[i]['x']-1,que[i]['y'],que[i]['z'],model_new)
            if bb != None:
                blocking_bricks.append(bb)
        blocking_bricks = list(remove_duplicates(blocking_bricks))

        # check if 1 brick can be removed to fix assembly
        ass_list.append(blocking_bricks[0])
        for i in range(0,len(blocking_bricks)):
            ass_list[-1] = blocking_bricks[i]
            que,opt = ass.sort_bricks_ass(ass_list,copy.deepcopy(model_new))
            if opt == 'y':
                dis_list.append(blocking_bricks[i])
                return dis_list, ass_list

        # check if 2 bricks can be removed to fix assembly
        ass_list[-1] = blocking_bricks[0]
        ass_list.append(blocking_bricks[1])
        for i in range(0,len(blocking_bricks)-1):
            ass_list[-2] = blocking_bricks[i]
            for j in range(i+1,len(blocking_bricks)):
                ass_list[-1] = blocking_bricks[j]
                que,opt = ass.sort_bricks_ass(ass_list,copy.deepcopy(model_new))
                if opt == 'y':
                    dis_list.append(blocking_bricks[i])
                    dis_list.append(blocking_bricks[j])
                    return dis_list, ass_list
            
        return 'help'


    #index = len(bricks_new)
    #for i in range(0,len(bricks_new)):
    #    if bricks_new[i]['z'] == lowest['z']+1:
    #        index = i
    #        break
    #ass_list = list(bricks_new[index:])
    #ass_list.insert(0,lowest)

    return dis_list, ass_list

# pickable if constraints pass all masks
def pickable(b,constraints):
    if b == 0:
        for i in range(0,len(dis.pick_masks[0])):
            if constraints&dis.pick_masks[0][i][0] == 0:                      # if constraints allow a certain pick...
                return 1
        # comment out loop for flex
        for i in range(0,len(dis.pick_masks[2])):
            if constraints&dis.pick_masks[2][i][0] == 0:                      # if constraints allow a certain pick...
                return 1
    elif b == 1:
        for i in range(0,len(dis.pick_masks[1])):
            if constraints&dis.pick_masks[1][i][0] == 0:                      # if constraints allow a certain pick...
                return 1
        # comment out loop for flex
        for i in range(0,len(dis.pick_masks[3])):
            if constraints&dis.pick_masks[3][i][0] == 0:                      # if constraints allow a certain pick...
                return 1
    return 0

# if not pickable: find min no. of bricks to remove to make pickable
def critical_neighbours(constraints,brick,updated_model):
    remove = 0
    remove_bricks = []

    if brick['b'] == 0:
        remove = constraints&dis.pick_masks[0][0][0]
        remove_bricks = list(find_bricks(remove,brick,updated_model))
        for i in range(1,len(dis.pick_masks[0])):
            remove = constraints&dis.pick_masks[0][i][0]
            bricks = find_bricks(remove,brick,updated_model)
            if len(bricks)<len(remove_bricks):
                remove_bricks = []
                remove_bricks = copy.deepcopy(bricks)

        for i in range(0,len(dis.pick_masks[2])):
            remove = constraints&dis.pick_masks[2][i][0]
            bricks = find_bricks(remove,brick,updated_model)
            if len(bricks)<len(remove_bricks):
                remove_bricks = []
                remove_bricks = copy.deepcopy(bricks)

    if brick['b'] == 1:
        remove = constraints&dis.pick_masks[1][0][0]
        remove_bricks = list(find_bricks(remove,brick,updated_model))
        for i in range(1,len(dis.pick_masks[1])):
            remove = constraints&dis.pick_masks[1][i][0]
            bricks = find_bricks(remove,brick,updated_model)
            if len(bricks)<len(remove_bricks):
                remove_bricks = []
                remove_bricks = copy.deepcopy(bricks)

        for i in range(0,len(dis.pick_masks[3])):
            remove = constraints&dis.pick_masks[3][i][0]
            bricks = find_bricks(remove,brick,updated_model)
            if len(bricks)<len(remove_bricks):
                remove_bricks = []
                remove_bricks = copy.deepcopy(bricks)

    return remove_bricks

# find bricks contributing to the 'remove' part of constraints
def find_bricks(remove,brick,updated_model):
    remove_bricks = [brick]
    if remove_bricks[0]['b']==0:
        # -------------- 0 -------------- #
        if remove&1 == 1:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']+2,brick['y'],brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x'],brick['y']-1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 1 -------------- #
        if remove&2 == 2:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']+2,brick['y']+1,brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x']+1,brick['y']-1,brick['z'],updated_model)

            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 2 -------------- #
        if remove&4 == 4:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']+2,brick['y']+2,brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x']+2,brick['y']-1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 3 -------------- #
        if remove&8 == 8:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']+2,brick['y']+3,brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x']+3,brick['y']-1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 6 -------------- #
        if remove&64 == 64:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']-1,brick['y']+3,brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x']+3,brick['y']+2,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 7 -------------- #
        if remove&128 == 128:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']-1,brick['y']+2,brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x']+2,brick['y']+2,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 8 -------------- #
        if remove&256 == 256:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']-1,brick['y']+1,brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x']+1,brick['y']+2,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 9 -------------- #
        if remove&512 == 512:
            if brick['r'] == 0:
                remove_brick = find_brick(brick['x']-1,brick['y'],brick['z'],updated_model)
            else:
                remove_brick = find_brick(brick['x'],brick['y']+2,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)



    elif remove_bricks[0]['b'] == 1:
        # -------------- 0 -------------- #
        if remove&1 == 1:
            remove_brick = find_brick(brick['x'],brick['y']+1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 1 -------------- #
        if remove&2 == 2:
            remove_brick = find_brick(brick['x']+1,brick['y']+1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 2 -------------- #
        if remove&4 == 4:
            remove_brick = find_brick(brick['x']+2,brick['y'],brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 3 -------------- #
        if remove&8 == 8:
            remove_brick = find_brick(brick['x']+2,brick['y']+1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 4 -------------- #
        if remove&16 == 16:
            remove_brick = find_brick(brick['x']+1,brick['y']+2,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 5 -------------- #
        if remove&32 == 32:
            remove_brick = find_brick(brick['x'],brick['y']+2,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 6 -------------- #
        if remove&64 == 64:
            remove_brick = find_brick(brick['x']-1,brick['y']+1,brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)

        # -------------- 7 -------------- #
        if remove&128 == 128:
            remove_brick = find_brick(brick['x']-1,brick['y'],brick['z'],updated_model)
            if remove_brick != None:
                remove_bricks.append(remove_brick)


    opt = 'n'
    while opt == 'n':
        remove_bricks = copy.deepcopy(remove_duplicates(remove_bricks))
        new_list,opt = dis.sort_bricks_dis(remove_bricks,updated_model)
        if opt == 'n':
            for i in range(0,len(new_list)):
                r_bricks = list(find_bricks(remove,brick,updated_model))
                for j in range(0,len(r_bricks)):
                    remove_bricks.append(r_bricks[j])

    return remove_bricks

# given a constraint location, find the contributing brick
def find_brick(x,y,z,updated_model):
    if y >= 0 and y <= 15 and x >= 0 and x <= 31:
        if updated_model[z][y][x] == 0:
            return
    else:
        return
    # locate top left corner
    y0 = 0
    x0 = 0
    n = 0
    while y-n > 0:
        if updated_model[z][y-n][x] != updated_model[z][y][x]:
            y0 = y-n+1
            break
        n+=1
    n = 0
    while x-n > 0:
        if updated_model[z][y0][x-n] != updated_model[z][y0][x]:
            x0 = x-n+1
            break
        n+=1

    # compare with opposite conrner for brick info
    if x0 < 31 and y0 < 13:
        if updated_model[z][y0+3][x0+1] == updated_model[z][y0][x0]:    # vertical brick, 'r' = 0 and default picking location in centre of brick
            brick = {'x':x0,'y':y0,'z':z,'r':0,'p':1,'xe':0,'ye':0,'b':0}
            return brick
    if x0 < 29 and y0 < 15:
        if updated_model[z][y0+1][x0+3] == updated_model[z][y0][x0]:    # horizontal brick, 'r' = 90 and default picking location in centre of brick
            brick = {'x':x0,'y':y0,'z':z,'r':90,'p':1,'xe':0,'ye':0,'b':0}
            return brick
    if x0 < 31 and y0 < 15:
        if updated_model[z][y0+1][x0+1] == updated_model[z][y0][x0]:    # 2x2 brick, default picking location in centre of brick
            brick = {'x':x0,'y':y0,'z':z,'r':0,'p':0,'xe':0,'ye':0,'b':1}
            return brick

    return

# remove all duplicates from a list
def remove_duplicates(alist):
    brick_list = [alist[0]]
    for i in range(1,len(alist)):
        flag = 0
        for j in range(0,len(brick_list)):
            if fd.match_bricks(alist[i],brick_list[j]) == 1:
                flag = 1
        if flag == 0:
            brick_list.append(alist[i])
    return brick_list