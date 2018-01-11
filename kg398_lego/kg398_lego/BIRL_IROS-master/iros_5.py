import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

# Pre Determined stuff
act_usb=78
pos_u = [-400, -400]            # Location to go to for the USB
height_u = 80

pos_light = [-450, -400]        #
height_light = 60               #
act_light= 20

usb1_down = {"x":67.21,"y":-107.33,"z":135.22,"rx":-127.32,"ry":-118.51,"rz":41.32}
usb1_up =  {"x":67.21,"y":-110.09,"z":131.14,"rx":-120.49,"ry":-118.49,"rz":41.29}

usb_joints_waypoint = {"x":-44.27,"y":-96.15,"z":104.97,"rx":-34.63,"ry":-46.86,"rz":-207.27}

def begin(c,ser_ee):
    obj = raw_input('Light or USB? (l/u)?')
    if (obj == "u"):
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_usb
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Set tool to iros_1
        ic.socket_send(c,sCMD=201)

        # Close Gripper
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        msg = ic.safe_ur_move(c,Pose=dict(usb1_down),CMD=2, Speed = 0.2)

        # Close Gripper
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)

        raw= raw_input("wait")

        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        demand_Pose["z"] = demand_Pose["z"] + 30
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,Speed=0.2,CMD=4)	
        
        time.sleep(2)
 
        msg = ic.safe_ur_move(c,Pose=dict(usb1_down),CMD=2, Speed = 0.2)
        raw= raw_input("wati")

        # Open Gripper
        demand_Grip["servo"]=120
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)



        #Pull out
        demand_Pose["z"] = height_u + 50
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        time.sleep(5)

        #Push in and release
        demand_Pose["z"] = height_u - 5
        demand_Grip["servo"]=120
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Speed=0.25,Grip=demand_Grip,CMD=4)

    else:
        # Home position
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_light
        msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

        # Set tool to iros_1
        ic.socket_send(c,sCMD=201)

        # Goto position of light
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":pos_light[0], "y":pos_light[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Lower to light
        demand_Pose["z"] = height_light
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Close Gripper
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee,demand_Grip)

        time.sleep(0.5)

        # Pull out
        demand_Pose["z"] =  height_light + 50
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Push in and release
        demand_Pose["z"] =  height_light  - 5
        demand_Grip["servo"]=120
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Speed=0.25,Grip=demand_Grip,CMD=4)

    # Raise
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2]+80, "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    # Set tool to iros_0
    ic.socket_send(c,sCMD=200)

    # Home position
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    print ".....................Done......................"        
