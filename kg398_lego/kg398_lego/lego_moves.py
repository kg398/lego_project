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
    for i in range(0,len(bricks)):                              # get brick from feed system, place in demand position, move up, repeat for whole list
        feed_pick(c,ser_ee)                 

        grid_place(c,ser_ee,bricks[i]['px'],bricks[i]['py'],bricks[i]['z'],bricks[i]['r'])

        msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
    return

# Disassemble a build que
def disassemble(c,ser_ee,bricks):
    for i in range(0,len(bricks)):                              # pick brick, place in feed system, move up, repeat for whole list
        grid_pick(c,ser_ee,bricks[len(bricks)-i-1]['px'],bricks[len(bricks)-i-1]['py'],bricks[len(bricks)-i-1]['z'],bricks[len(bricks)-i-1]['r'])
        
        feed_place(c,ser_ee)
        
        msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
    return

# Pick from feed system
def feed_pick(c,ser_ee):
    grid_pick(c,ser_ee,30,1,0,0)
    return

# Place in feed system
def feed_place(c,ser_ee):
    grid_place(c,ser_ee,30,1,0,0)
    return

# Pick from a grid location
def grid_pick(c,ser_ee,x,y,z,r):
    ic.socket_send(c,sCMD=300)                                  # select lego tcp (tool centre point)

    demand_Joints = dict(grid_pos(c,x,y,z+2,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)           # move above brick

    #ic.super_serial_send(ser_ee,"G",51)

    demand_Joints = dict(grid_pos(c,x,y,z+0.3,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5)

    demand_Joints = dict(grid_pos(c,x,y,z,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.1) # move down to brick slowly

    ic.super_serial_send(ser_ee,"G",49)                         # close grabber


    demand_Pose = dict(smooth_rotate(c,r,R=20))                 
    #print "demand_Pose: ",demand_Pose
    #ipt = raw_input("continue?")
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)             # rotate grabber to separate brick

    demand_Joints = dict(grid_pos(c,x,y,z+2,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5) # move back up

# Place in a grid location
def grid_place(c,ser_ee,x,y,z,r):
    ic.socket_send(c,sCMD=300)                                  # select lego tcp
    
    demand_Joints = dict(grid_pos(c,x,y,z+2,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)           # move above location

    demand_Joints = dict(grid_pos(c,x,y,z+0.3,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5)

    demand_Joints = dict(grid_pos(c,x,y,z,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.1) # press down slowly

    ic.super_serial_send(ser_ee,"G",51)                         # open grabber

    demand_Joints = dict(grid_pos(c,x,y,z+2,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)           # move back up
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
def smooth_rotate(c,r,R=20):
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



# Test function for picking multiple bricks simulateously, by changing lego tcp, axis of smooth rotate is altered
def stack_pick(c,ser_ee,x,y,z,r,stack=1):
    demand_Joints = dict(grid_pos(c,x,y,z+1,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    ic.socket_send(c,sCMD=299+stack)

    ic.super_serial_send(ser_ee,"G",51)

    demand_Joints = dict(grid_pos(c,x,y,z,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.1)

    ic.super_serial_send(ser_ee,"G",49)

    demand_Pose = dict(smooth_rotate(c,r,R=20))
    #print "demand_Pose: ",demand_Pose
    #ipt = raw_input("continue?")
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)

    demand_Joints = dict(grid_pos(c,x,y,z+1,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.5)