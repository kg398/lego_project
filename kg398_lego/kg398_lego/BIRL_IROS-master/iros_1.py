#!/usr/bin/env python
# Scripts for iros challenge 1: pick up a mug and place on saucer
#                               pick up saucer with mug on
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt
import os

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

import iros_vision_tools as ivt
import iros_vision_functions as ivfunc

PATH_TO_TASK_IMAGES = "task_images"

saucer_waypoint1_joints = {"x": -1.38, "y": -101.75, "z": 118.72, "rx": -105.93, "ry": -31.30, "rz": -47.55}
saucer_waypoint2 = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 50.04, "ry": 122.78, "rz": -82.80}

def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    #object grasping parameters
    act_mug=76
    act_saucer=83
    height_mug=20.0
    height_saucer=5.0
    radius_mug=30.0
    radius_saucer=52.0
    zoff = -23.5
    
    task_img_1 = ivt.capture_pic(CAMERA,3)
    cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_1.jpg'), task_img_1)
    crop_task_img_1 = ivt.crop_out(task_img_1, crop_points)
    table_circles = ivfunc.cup_saucer2(crop_task_img_1, show=True)
    print table_circles
    print "CROP_POINTS: ", crop_points
    print "P1: ", p1
    print "INVERSE", inverse

    m_circle = table_circles["mug"]["circle"]
    s_circle = table_circles["saucer"]["circle"]
    print "m_circle: ", m_circle
    print "s_circle: ", s_circle
    mp = [m_circle[0], m_circle[1]]
    mx,my = ivt.pix3world(p1, inverse, mp)
    mx = mx[0,0]
    my = my[0,0]
    
    sp = [s_circle[0], s_circle[1]]
    sx,sy = ivt.pix3world(p1, inverse, sp)
    sx = sx[0,0]
    sy = sy[0,0]
    
    #vision stuff: get mug and saucer position
    # mug and saucer centre positions
    #mx,my,sx,sy = mug_saucer_pos

    print "MX: ", mx
    print "MY: ", my
    
    # Home
    demand_Grip = dict(iw.ee_home)
    demand_Grip["act"]=60
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    # Set tool to iros_1
    ic.socket_send(c,sCMD=201)
    
    demand_Pose = dict(iw.home)
    demand_Pose["x"]=mx + radius_mug/1.41421
    demand_Pose["y"]=my - radius_mug/1.41421
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_mug+zoff
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=40
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    
    demand_Grip["act"]=act_mug
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    
    demand_Grip["servo"]=80
    msg = ic.end_effector_move(ser_ee,demand_Grip)
    
    demand_Grip["servo"]=30
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    time.sleep(1)

    demand_Pose["z"]=height_mug+height_saucer+40+zoff
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["x"]=sx + radius_mug/1.41421
    demand_Pose["y"]=sy - radius_mug/1.41421
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_mug+height_saucer+10+zoff
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Grip["servo"]=120
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    demand_Pose["z"]=120+zoff
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    #motion stuff: pick saucer
    demand_Grip["act"]=act_saucer
    msg = ic.end_effector_move(ser_ee,demand_Grip)

    demand_Joints = dict(saucer_waypoint1_joints)
    msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":sx, "y":sy+radius_saucer+30, "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+40+zoff
    demand_Grip["servo"]=60
    demand_Grip["tilt"]=28
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    time.sleep(0.2)

    demand_Pose["y"]=sy+radius_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+zoff
    demand_Grip["servo"]=30
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    time.sleep(0.5)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":saucer_waypoint2["rx"], "ry":saucer_waypoint2["ry"], "rz":saucer_waypoint2["rz"]}
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+50+zoff
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    x=-400#float(raw_input("x: "))
    y=-400#float(raw_input("y: "))

    demand_Pose["x"]=x
    demand_Pose["y"]=y+radius_saucer
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+zoff
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    demand_Pose["z"]=height_saucer+zoff
    demand_Grip["servo"]=60
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=height_saucer+40+zoff
    demand_Grip["servo"]=120
    msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=current_Pose[2]+80
    msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

    ic.socket_send(c,sCMD=200)

    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)
   
print ".....................Done......................"
