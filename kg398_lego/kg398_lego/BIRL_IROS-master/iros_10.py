#!/usr/bin/env python
# Scripts for iros challenge 10: open a bottle with a safety locking cap
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt
import numpy as np

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

lid_joints = [{"x": 72.60, "y": -69.01, "z": 81.04, "rx": -94.34, "ry": -91.74, "rz": 30.52},
              {"x": 74.46, "y": -70.66, "z": 84.36, "rx": -100.22, "ry": -96.86, "rz": -18.75},
              {"x": 74.62, "y": -73.17, "z": 89.18, "rx": -108.25, "ry": -97.33, "rz": -61.22},
              {"x": 73.19, "y": -75.42, "z": 93.32, "rx": -114.90, "ry": -93.47, "rz": -106.65},
              {"x": 71.04, "y": -76.16, "z": 94.48, "rx": -115.95, "ry": -87.62, "rz": -149.86}]

def begin(c,ser_ee):

    # parameters
    bottle_radius = 25
    bottle_height = 70
    act_bottle = 35

    # Parameters to pass to the function
    bx_1 = -80
    by_1 = -590

    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_bottle
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Set tool to iros_10
    ic.socket_send(c,sCMD=210)

    # Goto XY position for the bottle
    demand_Pose = dict(iw.home)
    demand_Pose["x"] = bx_1
    demand_Pose["y"] = by_1
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)
    #stored_Joints = ic.get_ur_position(c,3)

    # Twist lid
    #tcp_rotate(c)
    
    for i in range(0,2):
        for j in range(0,5):
            msg = ic.safe_ur_move(c,Pose=dict(lid_joints[j]),Speed=0.15,CMD=2)
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2]+30,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Raise
    '''current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2]+30,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Repeat
    # Goto XY position for the bottle
    demand_Joints = {"x":stored_Joints[0],"y":stored_Joints[1],"z":stored_Joints[2],"rx":stored_Joints[3],"ry":stored_Joints[4],"rz":stored_Joints[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    # Twist lid
    tcp_rotate(c)'''
    
    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)
 
    
    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(iw.home_joints),CMD=2) 

    demand_Pose = dict(iw.home)
    demand_Pose["x"] = bx_1 + bottle_radius/1.414
    demand_Pose["y"] = by_1 - bottle_radius/1.414
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Grasp lid
    demand_Pose["z"]=bottle_height
    demand_Grip["servo"]=30
    msg = ic.safe_move(c,ser_ee,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    time.sleep(0.5)

    # Lift up
    demand_Pose = dict(iw.home)
    demand_Pose["x"] = bx_1 + bottle_radius/1.414
    demand_Pose["y"] = by_1 - bottle_radius/1.414
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Set tool to iros_0
    ic.socket_send(c,sCMD=200)

    # Home
    msg = ic.safe_ur_move(c,Pose=dict(iw.home_joints),CMD=2)

    # Release bottle lid
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Home
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"

def tcp_rotate(c):
     # Cartesian rotation matrices to match grabbing_joints rotation
    x_rot = np.matrix([[ 1.0, 0.0, 0.0],
             [ 0.0, math.cos(math.pi-0.001), -math.sin(math.pi-0.001)],
             [ 0.0, math.sin(math.pi-0.001), math.cos(math.pi-0.001)]]) # x_rot[rows][columns]
    y_rot = np.matrix([[ math.cos(0.0), 0.0, -math.sin(0.0)],
             [ 0.0, 1.0, 0.0],
             [ math.sin(0.0), 0.0, math.cos(0.0)]]) # y_rot[rows][columns]
    z_rot = np.matrix([[ math.cos(-7*math.pi/4), -math.sin(-7*math.pi/4), 0.0],
             [ math.sin(-7*math.pi/4), math.cos(-7*math.pi/4), 0.0],
             [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

    # Create rotation matrix for current position
    R=z_rot*y_rot*x_rot

    # Cartesian to axis-angle
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
    multi = 1 / (2 * math.sin(theta))
    rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
    ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
    rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
    #print rx, ry, rz
    #inp = raw_input("Continue?")

    # Move to drawing waypoint
    #msg = ic.safe_ur_move(c,Pose=dict(uw.naughts_crosses_cam_joints),CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,Speed=1.5,CMD=8)

    # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
    #x_rot = np.matrix([[ 1.0, 0.0, 0.0],
    #         [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
    #         [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]

    # Move down until oject is reached
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":40,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    print "sending force_move................................................"
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),Speed=0.05,CMD=5)

    time.sleep(0.5)

    current_Pose = ic.get_ur_position(c,1)

    for i in range(0,4):
        z_rot = np.matrix([[ math.cos(math.pi/2), -math.sin(math.pi/2), 0.0],
                 [ math.sin(math.pi/2), math.cos(math.pi/2), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

        # Cartesian rotation matrix of desired orientation
        R=z_rot*R

        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        #print rx, ry, rz
        #inp = raw_input("Continue?")

        # Rotate around tool centre point defined by tcp_2
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=demand_Pose,Speed=1.5,CMD=8)
