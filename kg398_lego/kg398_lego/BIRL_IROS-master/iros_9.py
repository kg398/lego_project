#!/usr/bin/env python
# Scripts for iros challenge 9: pick up a straw
#                               insert straw into to-go cup
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

# parameters
act_straw = 75
cup1_offset = 30    # How close to go to the centre of the cup to get the straw
straw_height = 200
cup2_height = 150

cup_1 = [-400, -400]      # Location of cup 1
cup_2 = [-500, -400]      # Location of cup 2

def begin(c,ser_ee):
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_straw
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    ic.socket_send(c,sCMD=201)

    # Go to waypoint to get correct orientation
    demand_Pose = dict(iw.grabbing_joints_waypoint)
    demand_Pose["x"]=cup_1[0]
    msg = ic.safe_move(c,ser_ee,Pose=demand,Grip=demand_Grip,CMD=2)

    # Keep same joint orientation and just move in x, y and z
    #Move to straw height_mug
    demand_Pose["z"]=straw_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Move to straw location
    demand_Pose["y"]=cup_1[1] + cup1_offset
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    #Grasp straw
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Lift Straw
    demand_Pose["z"]=straw_height +200
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Move Straw
    demand_Pose["x"]=cup_2[0]
    demand_Pose["y"]=cup_2[1]
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Lower straw into cup
    demand_Pose["z"]=cup2_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Release straw
    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    # Move up
    demand_Pose["z"]=cup2_height
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Return home
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)
