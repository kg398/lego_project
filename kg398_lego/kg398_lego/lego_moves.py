#!/usr/bin/env python
# Motion-planning functions for use with kg398_lego tasks
import serial
import socket
import time
import random
import copy
import math
import numpy as np

import ur_interface_cmds as ic
import ur_waypoints as wp

# Assemble a build que
def assemble(c,ser_ee,bricks):
    delay = 0
    n = 0
    grip = 0
    for i in range(0,len(bricks)):
        # stop after 0,20,40 etc bricks to fill hoppers 
        #if i % 20 == 0:
        #    tic = time.time()
        #    ipt = raw_input("Fill hopper and press enter to continue")
        #    toc = time.time()
        #    delay+=toc-tic

        # pick brick(s) from feed if none in grip (grip = 0)
        flag = 1
        if bricks[i]['b']==0:                                      # if 2x4 brick use hopper 0
            if bricks[i]['p']==3:                                  # if tool placing method, pick 2 bricks
                if grip == 0:
                    feed_pick(c,ser_ee)
                feed_pick(c,ser_ee,stack=2)
                flag = 2
                bricks[i]['p']=1
            elif (bricks[i]['r'] == 0 or bricks[i]['r'] == 180) and grip == 0:
                feed_pick(c,ser_ee,X=bricks[i]['p'])
            elif grip == 0:
                feed_pick(c,ser_ee,X=2-bricks[i]['p'])
        elif bricks[i]['b']==1:                                     # if 2x2 brick use hopper 1
            if grip == 0:
                feed_pick(c,ser_ee,X=2,H=1)
            if bricks[i]['p'] == 3:
                feed_pick(c,ser_ee,X=2,H=1,stack=2)
                flag = 2
                bricks[i]['p']=0

        # home waypoint
        msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)

        # grid placing
        if bricks[i]['b']==0:
            if bricks[i]['r'] == 0 or bricks[i]['r'] == 180:
                grid_place(c,ser_ee,bricks[i]['x'],bricks[i]['y']+bricks[i]['p'],bricks[i]['z'],bricks[i]['r'],XE=-bricks[i]['ye'],YE=bricks[i]['xe'],stack=flag)
            else:
                grid_place(c,ser_ee,bricks[i]['x']+bricks[i]['p'],bricks[i]['y'],bricks[i]['z'],bricks[i]['r'],XE=bricks[i]['xe'],YE=bricks[i]['ye'],stack=flag)
        elif bricks[i]['b']==1:
            grid_place(c,ser_ee,bricks[i]['x'],bricks[i]['y'],bricks[i]['z'],bricks[i]['r'],XE=bricks[i]['xe'],YE=bricks[i]['ye'],stack=flag)

        grip = 0
        # stowing excess bricks
        if i < len(bricks)-1 and flag == 2:
           if bricks[i]['b'] == bricks[i+1]['b'] and (bricks[i+1]['p'] == 1 or bricks[i+1]['p'] == 3):   # if tool placing, don't stow brick if next place can use it
            grip = 1

        if grip == 0 and flag == 2:
            msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)
            feed_place(c,ser_ee,H=bricks[i]['b'])

        # home waypoint
        msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)

    return delay


# Disassemble a build que
def disassemble(c,ser_ee,bricks):
    delay = 0
    grip = 0
    for i in range(0,len(bricks)):                              # pick brick, place in feed system, move up, repeat for whole list
        # stop after 0,20,40 etc bricks to empty hoppers 
        #if i % 20 == 0:
        #    tic = time.time()
        #    ipt = raw_input("Empty hopper and press enter to continue")
        #    toc = time.time()
        #    delay+=toc-tic

        # pick brick if using tool disassembly
        flag = 1
        if bricks[i]['p'] == 3 and bricks[i]['b'] == 0:
            if grip == 0:
                feed_pick(c,ser_ee)
                msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)
            flag = 2
            bricks[i]['p'] = 1
        elif bricks[i]['p'] == 3 and bricks[i]['b'] == 1:
            if grip == 0:
                feed_pick(c,ser_ee,X=2,H=1)
                msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)
            flag = 2
            bricks[i]['p'] = 0


        # pick brick from grid
        if bricks[i]['r'] == 0 or bricks[i]['r'] == 180:
            grid_pick(c,ser_ee,bricks[i]['x'],bricks[i]['y']+bricks[i]['p'],bricks[i]['z'],bricks[i]['r'],stack=flag)
        else:
            grid_pick(c,ser_ee,bricks[i]['x']+bricks[i]['p'],bricks[i]['y'],bricks[i]['z'],bricks[i]['r'],stack=flag)
        
        # home waypoint
        msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)

        # stow bricks
        if flag == 2:
            feed_place2(c,ser_ee,sH=bricks[i]['b'])

        grip = 0
        # if tool picking next, don't stow brick
        if i < len(bricks)-1 and (flag == 2 or bricks[i]['p'] == 1):
            if bricks[i+1]['p'] == 3 and bricks[i]['b'] == bricks[i+1]['b']:
                grip = 1

        # stow excess bricks
        if grip == 0:
            feed_place(c,ser_ee,H=bricks[i]['b'])

        # home waypoint
        msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2,Speed=1.05)
 
    return

# Pick from feed system
def feed_pick(c,ser_ee,X=1,H=0,stack=1):
    ic.socket_send(c,sCMD=300)                                  # select lego tcp (tool centre point)
    if H==0:                                                    # waypoints for hopper and brick position
        if X == 0:
            demand_Pose = copy.deepcopy(wp.Hopper0_feed0)
        elif X == 1:
            demand_Pose = copy.deepcopy(wp.Hopper0_feed1)
        else:
            demand_Pose = copy.deepcopy(wp.Hopper0_feed2)
    else:
        if X == 0:
            demand_Pose = copy.deepcopy(wp.Hopper1_feed0)
        elif X == 1:
            demand_Pose = copy.deepcopy(wp.Hopper1_feed1)
        else:
            demand_Pose = copy.deepcopy(wp.Hopper1_feed2)

    if stack == 1:                                              # normal picking
        demand_Pose['z'] = demand_Pose['z']+20
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        demand_Pose['z'] = demand_Pose['z']-20
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8) 

        ic.super_serial_send(ser_ee,"G",49) 
    
        demand_Pose['z'] = demand_Pose['z']+40
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4) 

    elif stack == 2:                                            # run after normal picking to pick a second brick
        demand_Pose['z'] = demand_Pose['z']+40
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4) 

        demand_Pose['z'] = demand_Pose['z']-40+8
        print demand_Pose
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8,Speed=0.05)

        demand_Pose['y'] = demand_Pose['y']+0.5
        demand_Pose['x'] = demand_Pose['x']+0.5
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)
        
        demand_Pose['y'] = demand_Pose['y']+0.5
        demand_Pose['x'] = demand_Pose['x']+0.5
        demand_Pose['z'] = demand_Pose['z']+5
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8,Speed=0.001) 

        demand_Pose['y'] = demand_Pose['y']-1
        demand_Pose['x'] = demand_Pose['x']-1
        demand_Pose['z'] = demand_Pose['z']+35-8
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8,Speed=0.05) 

    return


# function for disassembling a stack of 2 into the feed system using the feed system
def feed_place2(c,ser_ee,sH=0):
    ic.socket_send(c,sCMD=300)                              # select lego tcp (tool centre point)

    # separate stack by placing in pick position
    demand_Pose = copy.deepcopy(wp.Hopper0_feed1)

    demand_Pose['z'] = demand_Pose['z']+38
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)         # move above brick

    demand_Pose['z'] = demand_Pose['z']-20
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8) 

    demand_Pose = dict(smooth_rotate(c,R=15))
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)         # rotate grabber

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {'x':current_Pose[0],'y':current_Pose[1],'z':current_Pose[2]+80,'rx':current_Pose[3],'ry':current_Pose[4],'rz':current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    feed_place(c,ser_ee,H=sH)

    demand_Pose = copy.deepcopy(wp.Hopper0_feed1)

    demand_Pose['z'] = demand_Pose['z']+28.5
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)         # move above brick

    demand_Pose['z'] = demand_Pose['z']-20
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8) 

    ic.super_serial_send(ser_ee,"G",49) 

    demand_Pose['x'] = wp.Hopper0_feed0['x']
    demand_Pose['y'] = wp.Hopper0_feed0['y']
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    demand_Pose = dict(smooth_rotate(c,R=30))
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)         # rotate grabber

    demand_Pose['z'] = demand_Pose['z']+80
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    #feed_place(c,ser_ee,H=sH)
    return


# Place in feed system
def feed_place(c,ser_ee,H=0):
    ic.socket_send(c,sCMD=300)                           # select lego tcp (tool centre point)

    if H==0:
        demand_Joints = copy.deepcopy(wp.Hopper0_stow_wp_joints)
        demand_Pose = copy.deepcopy(wp.Hopper0_stow)
    else:
        demand_Joints = copy.deepcopy(wp.Hopper1_stow_wp_joints)
        demand_Pose = copy.deepcopy(wp.Hopper1_stow)

    msg = ic.safe_ur_move(c,Pose=dict(demand_Joints),CMD=2,Speed=1.05)

    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8) 

    ic.super_serial_send(ser_ee,"G",51) 

    demand_Pose['x'] = demand_Pose['x']+3
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose = dict(smooth_rotate(c,R=15))
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    demand_Pose['x'] = demand_Pose['x']-3
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose['y'] = demand_Pose['y']-20
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    msg = ic.safe_ur_move(c,Pose=dict(demand_Joints),CMD=2,Speed=1.05)
    return

# Pick from a grid location
def grid_pick(c,ser_ee,x,y,z,r,stack=1):
    ic.socket_send(c,sCMD=300)                                          # select lego tcp (tool centre point)

    demand_Joints = dict(grid_pos(c,x,y,z+1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=1.05)        # move above brick

    #ic.super_serial_send(ser_ee,"G",51)

    demand_Joints = dict(grid_pos(c,x,y,z-0.1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5)

    demand_Joints = dict(grid_pos(c,x,y,z-1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.1)

    if stack == 1:
        ic.super_serial_send(ser_ee,"G",49)                             # close grabber

    ic.socket_send(c,sCMD=299+stack) 
    demand_Pose = dict(smooth_rotate(c,R=15))                 
    #print "demand_Pose: ",demand_Pose
    #ipt = raw_input("continue?")
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)                     # rotate grabber

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {'x':current_Pose[0],'y':current_Pose[1],'z':current_Pose[2]+20,'rx':current_Pose[3],'ry':current_Pose[4],'rz':current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    demand_Joints = dict(grid_pos(c,x,y,z+1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5)     # move back up

# Place in a grid location
def grid_place(c,ser_ee,x,y,z,r,XE=0,YE=0,stack=1):
    ic.socket_send(c,sCMD=300)                                      # select lego tcp

    alpha = 0.3
    XE = alpha*XE
    YE = alpha*YE
    
    demand_Joints = dict(grid_pos(c,x+XE,y+YE,z+1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=1.05)    # move above location

    demand_Joints = dict(grid_pos(c,x,y,z-0.5+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5)

    demand_Joints = dict(grid_pos(c,x,y,z-1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.1)

    if stack == 1:                                                  # if normal placing, release brick
        ic.super_serial_send(ser_ee,"G",51)                         # open grabber

        ic.socket_send(c,sCMD=299)  
        demand_Pose = dict(smooth_rotate(c,R=3))
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)             # rotate grabber
        ic.socket_send(c,sCMD=300)  

    elif stack == 2:                                                # if tool placing, pick top brick
        demand_Pose = dict(smooth_rotate(c,R=15))
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)             # rotate grabber

    demand_Joints = dict(grid_pos(c,x,y,z+1+stack,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)               # move back up
 
    return

# Converts grid position to robot co-ordinates
# Uses linear interpolation of calibration points in ur_waypoints
# Returns demand pose in joint space
def grid_pos(c,x,y,z,r):
    # x:0-31
    # y:0-15
    # z:0-31
    # r:0/90
    nx = 30
    ny = 12
    nz = 9

    # dx = {"x": (wp.grid_30_1['x']-wp.grid_0_1['x'])/nx, "y": (wp.grid_30_1['y']-wp.grid_0_1['y'])/nx, "z": (wp.grid_30_1['z']-wp.grid_0_1['z'])/nx, "rx": 0, "ry": 0, "rz": 0}
    dy1 = {"x": (wp.grid_0_13['x']-wp.grid_0_1['x'])/ny, "y": (wp.grid_0_13['y']-wp.grid_0_1['y'])/ny, "z": (wp.grid_0_13['z']-wp.grid_0_1['z'])/ny, "rx": 0, "ry": 0, "rz": 0}
    dy2 = {"x": (wp.grid_30_13['x']-wp.grid_30_1['x'])/ny, "y": (wp.grid_30_13['y']-wp.grid_30_1['y'])/ny, "z": (wp.grid_30_13['z']-wp.grid_30_1['z'])/ny, "rx": 0, "ry": 0, "rz": 0}
    dz = {"x": (wp.grid_30_1_10['x']-wp.grid_30_1['x'])/nz, "y": (wp.grid_30_1_10['y']-wp.grid_30_1['y'])/nz, "z": (wp.grid_30_1_10['z']-wp.grid_30_1['z'])/nz, "rx": 0, "ry": 0, "rz": 0}

    y1_Pose = {"x": wp.grid_0_1['x'] + (y-1)*dy1['x'] + z*dz['x'], 
                 "y": wp.grid_0_1['y'] + (y-1)*dy1['y'] + z*dz['y'], 
                 "z": wp.grid_0_1['z'] + (y-1)*dy1['z'] + z*dz['z'], 
                 "rx": wp.grid_0_1['rx'], "ry": wp.grid_0_1['ry'], "rz": wp.grid_0_1['rz']}

    y2_Pose = {"x": wp.grid_30_1['x'] + (y-1)*dy2['x'] + z*dz['x'], 
                 "y": wp.grid_30_1['y'] + (y-1)*dy2['y'] + z*dz['y'], 
                 "z": wp.grid_30_1['z'] + (y-1)*dy2['z'] + z*dz['z'], 
                 "rx": wp.grid_30_1['rx'], "ry": wp.grid_0_1['ry'], "rz": wp.grid_0_1['rz']}

    grid_Pose = {"x": ((nx-x)*y1_Pose['x'] + x*y2_Pose['x'])/nx, 
                 "y": ((nx-x)*y1_Pose['y'] + x*y2_Pose['y'])/nx, 
                 "z": ((nx-x)*y1_Pose['z'] + x*y2_Pose['z'])/nx, 
                 "rx": wp.grid_0_1['rx'], "ry": wp.grid_0_1['ry'], "rz": wp.grid_0_1['rz']}


    ic.socket_send(c,sCMD=300)

    calculated_Joints = ic.get_ur_position(c,10,gPose=dict(grid_Pose))      # uses UR inverse kinematics solver to get joint positions, then add rotation for brick orienation
    grid_Joints = {"x": calculated_Joints[0], "y": calculated_Joints[1], "z": calculated_Joints[2], "rx": calculated_Joints[3], "ry": calculated_Joints[4], "rz": calculated_Joints[5]+r}
    if calculated_Joints[5] > 270:
        grid_Joints["rz"] = calculated_Joints[5]+r-360

    return grid_Joints

# Calculates a demand pose to give a rotation about desired axis
# Returns demand_Pose
def smooth_rotate(c,R=15):
    # axis-angle combination formula
    #current_Pose = ic.get_ur_position(c,1)
    #T1 = math.pi*math.sqrt(current_Pose[3]*current_Pose[3]+current_Pose[4]*current_Pose[4]+current_Pose[5]*current_Pose[5])/180
    #R1 = np.array([current_Pose[3]*math.pi/(180*T1),
    #               current_Pose[4]*math.pi/(180*T1),
    #               current_Pose[5]*math.pi/(180*T1)])


    #if r == 0:
    #    T2 = math.pi*R/180
    #    dx = (wp.grid_30_13['x']-wp.grid_30_1['x'])*math.pi/180
    #    dy = (wp.grid_30_13['y']-wp.grid_30_1['y'])*math.pi/180
    #    dz = (wp.grid_30_13['z']-wp.grid_30_1['z'])*math.pi/180
    #    norm = math.sqrt(dx*dx+dy*dy+dz*dz)
    #    R2 = np.array([dx/norm,
    #                   dy/norm,
    #                   dz/norm])

    #T3 = 2*math.acos(math.cos(T1/2)*math.cos(T2/2)-math.sin(T1/2)*math.sin(T2/2)*np.dot(R1,R2))
    #R3 = (math.sin(T2/2)*math.cos(T1/2)*R2+math.cos(T2/2)*math.sin(T1/2)*R1+math.sin(T2/2)*math.sin(T1/2)*np.cross(R2,R2))/math.sin(T3/2)
    #return {"x": current_Pose[0], "y": current_Pose[1], "z": current_Pose[2], "rx": T3*R3[0]*180/math.pi, "ry": T3*R3[1]*180/math.pi, "rz": T3*R3[2]*180/math.pi}

    # Rotation axis in end-effector co-ordinate system
    R2 = [0,-1,0]
    T2 = R

    trans = {'x':0,'y':0,'z':0,'rx':T2*R2[0],'ry':T2*R2[1],'rz':T2*R2[2]}   # pose decribing transformation (no translation, rotation about defined axis)
    trans_pose = ic.get_ur_position(c,11,trans)                             # returns current pose transformed by 'trans'
        
    return {"x": trans_pose[0], "y": trans_pose[1], "z": trans_pose[2], "rx": trans_pose[3], "ry": trans_pose[4], "rz": trans_pose[5]}