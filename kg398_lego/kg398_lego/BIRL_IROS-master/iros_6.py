#!/usr/bin/env python
# Scripts for iros challenge 6: tear off a sheet of paper towel
#                               tear off another sheet of paper towel
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

# Pre defined variables
centre = [-400, -400]
radius = 100
height = 300
act_paper = 78
move = 200

def begin(c,ser_ee):
    # Go To home position
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=act_paper
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Go to kitchen paper location
    ori = [0, 45, 90, 135, 180]
    dx = r*math.cos(ori)* 180/math.pi
    dy = r*math.sin(ori)* 180/math.pi
    move_dx = r*math.cos(ori)* 180/math.pi
    move_dy = r*math.sin(ori)* 180/math.pi

    for i in range(0, len(ori)+1):
        # Home
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Set rotations
        current_Joints = ic.get_ur_position(c,3)
    	demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]-ori+135}
        msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

        # Move to abouve the point
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose["x"] = centre[0] + dx[i]
        demand_Pose["y"] = centre[1] + dy[i]
        demand_Pose["z"] = height + 50
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

        # Move down
        demand_Pose["z"] = height
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

        # Grip
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)

        # Tear
        demand_Pose["x"] = centre[0] + move_dx[i]
        demand_Pose["y"] = centre[1] + move_dy[i]
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

        # Release
        demand_Grip["servo"]=120
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        # Move up
        demand_Pose["z"] = height + 50
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)
