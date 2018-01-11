#!/usr/bin/env python
# Scripts for iros challenge 8: pick up a hammer
#                               use this to drive 5 nails into foam board
# ic.serial_send(ser_ee,"H",var)  0-127
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

hammer_waypoint_joints_1 = {"x": 86.74, "y": -44.87, "z": 36.82, "rx": 5.5, "ry": -6.23, "rz": 151.96}
hammer_waypoint_joints_2 = {"x": 84.80, "y": -32.29, "z": 56.16, "rx": -14.12, "ry": -6.75, "rz": -157.72}
hammer_waypoint_joints_3 = {"x": 78.21, "y": -33.61, "z": 58.62, "rx": -20.08, "ry": -13.29, "rz": -152.85}


hammer_waypoint_joints_4 = {"x": 85.48, "y": -47.16, "z": 62.60, "rx": -11.79, "ry": -6.10, "rz": -191.35}

nail_1 = {"x": 77.03, "y": -66.42, "z": 87.15, "rx": -21.23, "ry": 11.14, "rz": -193.37}
nail_2 = {"x": 81.80, "y": -66.29, "z": 86.95, "rx": -21.00, "ry": 15.92, "rz": -193.52}
nail_3 = {"x": 86.75, "y": -65.66, "z": 86.02, "rx": -20.61, "ry": 20.85, "rz": -193.60}
nail_4 = {"x": 92.09, "y": -64.41, "z": 84.16, "rx": -19.93, "ry": 26.18, "rz": -193.64}
nail_6 = {"x": 102.70, "y": -59.97, "z": 77.36, "rx": -17.48, "ry": 36.78, "rz": -193.68}
nail_5 = {"x": 96.90, "y": -62.75, "z": 81.65, "rx": -19.04, "ry": 30.99, "rz": -193.67}
nail_x = 5
nail_down =80

def begin(c,ser_ee):
    # Home
    msg = ic.safe_ur_move(c,Pose=dict(iw.home_joints),CMD=2)
    ic.serial_send(ser_ee,"H",100)

    # Move towards hammer using waypoints
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_1),CMD=2)
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_2),CMD=2)
    time.sleep(1)
    raw = raw_input("continue?")
    msg = ic.safe_ur_move(c,Pose=dict(hammer_waypoint_joints_3),CMD=2, Speed = 0.1)

    # Close hammer servo
    ic.serial_send(ser_ee,"H",10)
    time.sleep(2)
    raw = raw_input("continue?")

    msg = ic.safe_ur_move(c,Pose=dict(nail_1),CMD=2, Speed = 0.2)
    updown(c)

    msg = ic.safe_ur_move(c,Pose=dict(nail_2),CMD=2, Speed = 0.2)
    updown(c)

    msg = ic.safe_ur_move(c,Pose=dict(nail_3),CMD=2, Speed = 0.2)
    updown(c)

    msg = ic.safe_ur_move(c,Pose=dict(nail_4),CMD=2, Speed = 0.2)
    updown(c)

    msg = ic.safe_ur_move(c,Pose=dict(nail_5),CMD=2, Speed = 0.2)
    updown(c)

    msg = ic.safe_ur_move(c,Pose=dict(nail_6),CMD=2, Speed = 0.2)
    updown(c)

    # Release hammer
    ic.serial_send(ser_ee,"H",100)

    # GO HOme
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"


def updown(c):
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    demand_Pose["z"]= demand_Pose["z"] - nail_down
    msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD =4, Speed = 0.2)

    time.sleep(1)

    demand_Pose["z"]= demand_Pose["z"] + nail_down
    msg = ic.safe_ur_move(c,Pose= dict(demand_Pose), CMD =4, Speed = 0.2)
