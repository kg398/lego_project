#!/usr/bin/env python
# Scripts for iros challenge 7: store 6 wooden blocks onto correct pegs
import time
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt
import numpy as np
import os

import iros_interface_cmds as ic
import iros_waypoints as iw
#import vision_copy as vc

import iros_vision_tools as ivt
import iros_vision_functions as ivfunc
PATH_TO_TASK_IMAGES = "task_images"

# Pre defined parameters
x = -300
y = -300

peg_loc =[[x, y],[x + 100, y],[x + 200, y],[x + 300, y],[x + 400, y]]
height_pick = -18
height_place = 30

act_objects= [30, 23, 25, 22, 35, 35]

circle_way = {"x": 104.95, "y": -91.30, "z": 106.71, "rx": -102.23, "ry": -96.60, "rz": -226.57}
rect_way = {"x": 102.90, "y": -103.98, "z": 118.74, "rx": -103.45, "ry": -89.90, "rz": -119.40}
tri_way = {"x": 91.38, "y": -106.72, "z": 119.31, "rx": -99.18, "ry": -89.54, "rz": -134.62}
square_way = {"x": 80.12, "y": -109.44, "z": 123.23, "rx": -100.73, "ry": 90.03, "rz": -143.64}
pent_way = {"x": 80.12, "y": -109.44, "z": 123.23, "rx": -100.73, "ry": 90.03, "rz": -143.64}
def begin(c,ser_ee,p1,inverse,CAMERA,crop_points):
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)

    ic.socket_send(c,sCMD=207)

    for i in range(0,6):
        # Vision - Extract the list of shapes
        task_img_7 = ivt.capture_pic(CAMERA,3)
        cv2.imwrite(os.path.join(PATH_TO_TASK_IMAGES, 'task_img_7'+str(i)+'.jpg'), task_img_7)
        crop_task_img_7 = ivt.crop_out(task_img_7, crop_points)

        shape_list = ivfunc.extract_shape_list(crop_task_img_7, threshold=120, show=True)

        if len(shape_list)==0:
            print "No more shapes left"
            print "Expected "+str(5-i)+" more shapes"
            continue
        piece = shape_list[0]

        print "SHAPE INFO: "
        print "SHAPE ID:   ", piece['shape']
        px1 = piece['point1'][0][0]
        py1 = piece['point1'][0][1]
        px2 = piece['point2'][0][0]
        py2 = piece['point2'][0][1]

        pix = [px1, py1]
        x1,y1 = ivt.pix3world(p1, inverse, pix)
        x1 = x1[0,0]
        y1 = y1[0,0]

        pix2 = [px2, py2]
        x2,y2 = ivt.pix3world(p1, inverse, pix2)
        x2 = x2[0,0]
        y2 = y2[0,0]

        print "X1, Y1, X2, Y2: ", x1, y1, x2, y2

        params = [piece['shape'], x1, y1, x2, y2]
        x_n, y_n, orient = get_grasping_coords([x1, y1], [x2, y2])

        print "X_N, Y_N, ORIENT: "
        print x_n, y_n, orient

        # Get paramters and put into the following data structure
        # Get paramters and put into the following data structure
        #params = [0, x_pos, y_pos, ori]     # where the first item is the number that states the object type: 0 = cirlce, 1 = rect, 2 = triangle etc.

        ## Set orientation
        current_Joints = ic.get_ur_position(c,3)
    	demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]-orient+135}
        msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

        # Go to X,Y centre of the location
	current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":x1,"y":y1,"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        demand_Grip = dict(iw.ee_home)
        demand_Grip["act"]=act_objects[params[0]]-5
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        # Move down to Grasp
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose["z"]=height_pick
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        ## Partially close Gripper
        demand_Grip["servo"]=40
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        ## Close Actuator
        demand_Grip["act"]=act_objects[params[0]]
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        ## Close Gripper
        demand_Grip["servo"]=30
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        time.sleep(0.5)

        # Pick up object
        demand_Pose["z"] = height_pick + 90
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

	if(params[0] == 0):
		# Move to peg holder
		location = peg_loc[params[0]]
		#demand_Pose["x"] = location[0]
		#demand_Pose["y"] = location[1]
		msg = ic.safe_ur_move(c,Pose=dict(circle_way),CMD=2, Speed = 0.4)

	if(params[0] == 1):
		# Move to peg holder
		location = peg_loc[params[0]]
		#demand_Pose["x"] = location[0]
		#demand_Pose["y"] = location[1]
		msg = ic.safe_ur_move(c,Pose=dict(rect_way),CMD=2, Speed = 0.4)

	if(params[0] == 2):
		# Move to peg holder
		location = peg_loc[params[0]]
		#demand_Pose["x"] = location[0]
		#demand_Pose["y"] = location[1]
		msg = ic.safe_ur_move(c,Pose=dict(tri_way),CMD=2, Speed = 0.1)
	if(params[0] == 3):
		# Move to peg holder
		location = peg_loc[params[0]]
		#demand_Pose["x"] = location[0]
		#demand_Pose["y"] = location[1]
		msg = ic.safe_ur_move(c,Pose=dict(square_way),CMD=2, Speed = 0.4)
	if(params[0] == 4):
		# Move to peg holder
		location = peg_loc[params[0]]
		#demand_Pose["x"] = location[0]
		#demand_Pose["y"] = location[1]
		msg = ic.safe_ur_move(c,Pose=dict(pent_way),CMD=2, Speed = 0.4)
        '''
        # Rotate to zero wrist orientation
        current_Joints = ic.get_ur_position(c,3)
    	demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":0}
        msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

        # Home rotation
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = dict(iw.home)
        demand_Pose["x"]=current_Pose[0]
        demand_Pose["y"]=current_Pose[1]
        demand_Pose["z"]=current_Pose[2]
        msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)

        # Rotate to zero orientation
        current_Joints = ic.get_ur_position(c,3)
    	demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]-45}
        msg = ic.safe_ur_move(c,Pose=demand_Joints,CMD=2)

	#test = raw_input("sasdf")
        '''
        # Lower
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":height_place,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}

        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,Speed=0.2,CMD=4)

        # Release
        demand_Grip["servo"]= 120
        msg = ic.end_effector_move(ser_ee, demand_Grip)

        # Raise
        demand_Pose["z"] = height_place + 70
        msg = ic.safe_move(c,ser_ee,Pose=demand_Pose,Grip=demand_Grip,CMD=4)

        msg = ic.safe_ur_move(c,Pose=dict(iw.home_joints),CMD=2)

    ic.socket_send(c,sCMD=200)

    # Raise from peg
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),Grip=demand_Grip,CMD=2)

    print ".....................Done......................"

def get_grasping_coords(p_centre,p_edge):
    ori = math.atan2(p_centre[1]-p_edge[1],p_centre[0]-p_edge[0])*180.0/math.pi
    print "ori: ",ori
    ori = ori-180
    if ori<-180:
        ori=360+ori
    x = p_edge[0]
    y = p_edge[1]
    return float(x), float(y), ori
