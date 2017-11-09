#!/usr/bin/env python
import serial
import socket
import time
import random
import copy
import math

import ur_interface_cmds as ic
import ur_waypoints as wp

def grid_pick(c,ser_ee,x,y,z,r):
    ic.socket_send(c,sCMD=300)

    demand_Joints = dict(grid_pos(c,x,y,z,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    ic.super_serial_send(ser_ee,"G",51)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x": current_Pose[0], "y": current_Pose[1], "z": current_Pose[2]-12, "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4,Speed=0.05)

    ic.super_serial_send(ser_ee,"G",49)

    current_Joints = ic.get_ur_position(c,3)
    demand_Joints = {"x": current_Joints[0], "y": current_Joints[1], "z": current_Joints[2], "rx": current_Joints[3], "ry": current_Joints[4]+30, "rz": current_Joints[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2,Speed=0.05)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x": current_Pose[0], "y": current_Pose[1], "z": current_Pose[2]+14, "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4,Speed=0.001)

def grid_place(c,ser_ee,x,y,z,r):
    ic.socket_send(c,sCMD=300)
    
    demand_Joints = dict(grid_pos(c,x,y,z,r))
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x": current_Pose[0], "y": current_Pose[1], "z": current_Pose[2]-12, "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4,Speed=0.001)

    ic.super_serial_send(ser_ee,"G",51)

    demand_Pose['z'] = demand_Pose['z']+14
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4,Speed=0.05)
    return

def grid_pos(c,x,y,z,r):
    # x:0-31
    # y:0-15
    # z:0-31
    # r:0/90
    nx = 30
    ny = 12

    dx = {"x": (wp.grid_30_1['x']-wp.grid_0_1['x'])/nx, "y": (wp.grid_30_1['y']-wp.grid_0_1['y'])/nx, "z": (wp.grid_30_1['z']-wp.grid_0_1['z'])/nx, "rx": 0, "ry": 0, "rz": 0}
    dy = {"x": (wp.grid_30_13['x']-wp.grid_30_1['x'])/ny, "y": (wp.grid_30_13['y']-wp.grid_30_1['y'])/ny, "z": (wp.grid_30_13['z']-wp.grid_30_1['z'])/ny, "rx": 0, "ry": 0, "rz": 0}
    dz = {"x": 0, "y": 0, "z": 9.5, "rx": 0, "ry": 0, "rz": 0}

    grid_Pose = {"x": wp.grid_0_1['x'] + x*dx['x'] + (y-1)*dy['x'] + z*dz['x'], 
                 "y": wp.grid_0_1['y'] + x*dx['y'] + (y-1)*dy['y'] + z*dz['y'], 
                 "z": wp.grid_0_1['z'] + x*dx['z'] + (y-1)*dy['z'] + z*dz['z']+10, 
                 "rx": wp.grid_0_1['rx'], "ry": wp.grid_0_1['ry'], "rz": wp.grid_0_1['rz']}

    print 'pose: ',grid_Pose

    ic.socket_send(c,sCMD=300)

    calculated_Joints = ic.get_ur_position(c,10,gPose=dict(grid_Pose))
    grid_Joints = {"x": calculated_Joints[0], "y": calculated_Joints[1], "z": calculated_Joints[2], "rx": calculated_Joints[3], "ry": calculated_Joints[4], "rz": calculated_Joints[5]+r}

    return grid_Joints