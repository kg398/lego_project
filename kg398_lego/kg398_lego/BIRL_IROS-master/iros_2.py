#!/usr/bin/env python
# Scripts for iros challenge 2: lay out silverware
#                               re-stow silverware
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

#Joint positions for the picking ( just above)
pick_f = {"x": 77.82, "y": -86.76, "z": 105.13, "rx": -102.76, "ry": -92.04, "rz":-58.57}
pick_k = {"x": 74.00, "y": -86.55, "z": 107.18, "rx": -105.20, "ry": -92.41, "rz":-62.37}
pick_s = {"x": 85.44, "y": -85.50, "z": 105.73, "rx": -104.36, "ry": -91.28, "rz": -50.97}


#Joint positions for the placing
place_f = {"x": 35.87, "y": -46.09, "z": 48.92, "rx": -95.08, "ry": -93.37, "rz": -98.40}
place_k = {"x": 77.82, "y": -86.76, "z": 105.13, "rx": -102.76, "ry": -92.04, "rz": -58.57}
place_s = {"x": 26.63, "y": -77.99, "z": 95.92, "rx": -109.28, "ry": -90.67, "rz": -189.17}

pick_height = -14.0
place_height = -17.0

act_f = 70
act_k = 70
act_s = 75

def begin(c,ser_ee):
    # pick up fork
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_f
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    ipt = raw_input("continue")

    # Go to just above fork place_height
    msg = ic.safe_move(c,ser_ee,Pose=dict(pick_f),CMD=2)
    #ipt = raw_input("continue")
    #Go down in the z to pick up the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0]+12, "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Close the gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    ipt = raw_input("continue")
    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(pick_f),CMD=2)

    #ipt = raw_input("continue")
    # Go to place the fork
    msg = ic.safe_ur_move(c,Pose=dict(place_f),CMD=2)
    #ipt = raw_input("continue")
    # Lower the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = place_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)
    #ipt = raw_input("continue")
    # Raise arm
    msg = ic.safe_ur_move(c,Pose=dict(place_f),CMD=2)
    test = raw_input("Go on to spoon?")
###############################################################################
    # pick up spoon
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_s
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Go to just above spoon place_height
    msg = ic.safe_ur_move(c,Pose=dict(pick_s),CMD=2)

    #Go down in the z to pick up the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0]+10, "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Close the gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(pick_s),CMD=2)

    # Go to place the fork
    msg = ic.safe_ur_move(c,Pose=dict(place_s),CMD=2)

    # Lower the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = place_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise arm
    msg = ic.safe_ur_move(c,Pose=dict(place_s),CMD=2)
    test = raw_input("Go on to knife?")
###############################################################################
    # pick up knife
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_k
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Go to just above spoon place_height
    demand_Pose = dict(pick_k)
    demand_Pose["x"] = demand_Pose["x"]  + 20
    msg = ic.safe_ur_move(c,Pose=dict(pick_k),CMD=2)

    # Go to low ish and then mostly close gripper
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0]-2, "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height +15
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=50
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Move sideways
    demand_Pose = ic.get_ur_position(c,1)
    demand_Pose["x"] = demand_Pose["x"] 
    msg = ic.safe_ur_move(c,Pose=dict(pick_k),CMD=2)

    #Go down in the z to pick up the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0]-2, "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height -2
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)


    # Close the gripper
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(pick_k),CMD=2)

    # Go to place the fork
    msg = ic.safe_ur_move(c,Pose=dict(place_k),CMD=2)

    # Lower the fork
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = place_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise arm
    msg = ic.safe_ur_move(c,Pose=dict(place_k),CMD=2)
    test = raw_input("Go on to retrieve fork?")
###############################################################################
    # pick up fork
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_f
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Go abouve place of fork
    msg = ic.safe_ur_move(c,Pose=dict(place_f),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = place_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Pick up fork
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Lift and move above the tray for fokr
    msg = ic.safe_ur_move(c,Pose=dict(place_f),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(pick_f),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(pick_k),CMD=2)

    test = raw_input("Go on to retrieve spoon?")
###############################################################################
    # pick up spoon
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_s
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Go abouve place of spoon
    msg = ic.safe_ur_move(c,Pose=dict(place_s),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = place_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Pick up fork
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Lift and move above the tray for fokr
    msg = ic.safe_ur_move(c,Pose=dict(place_s),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(pick_s),CMD=2)
    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(pick_s),CMD=2)

    test = raw_input("Go on to retrieve knife?")
###############################################################################
    # pick up knife
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_k
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Go abouve place of knife
    msg = ic.safe_ur_move(c,Pose=dict(place_k),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = place_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Pick up fork
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Lift and move above the tray for fokr
    msg = ic.safe_ur_move(c,Pose=dict(place_k),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(pick_k),CMD=2)

    # Lower
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    demand_Pose["z"] = pick_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    time.sleep(1)

    # Raise
    msg = ic.safe_ur_move(c,Pose=dict(pick_k),CMD=2)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"
