#!/usr/bin/env python
# End effector example:
# Send cmd code (G for grabber servo, A for DC motor actuator, V for vacuum pump)
# Then value:
# G: open 0-120 closed
# A: open 0-90 closed
# V: grab g/r release
import serial
import socket
import time
import random
import copy
import math

import cv2
import imutils
from matplotlib import pyplot as plt
import os

import iros_interface_cmds as ic
import iros_waypoints as iw
import iros_1 as i1
import iros_2 as i2
import iros_3 as i3
import iros_4 as i4
import iros_5 as i5
import iros_6 as i6
import iros_7 as i7
import iros_8 as i8
import iros_9 as i9
import iros_10 as i10
#import vision_copy as vc
import iros_vision_functions as ivfunc
import iros_vision_tools as ivt

ROTATION = 3

def initialize():
    #HOST = "169.254.103.235" # The remote host
    HOST = "192.168.1.105" # The remote host
    HOST = "169.254.187.178"
    PORT = 30000 # The same port as used by the server

    print ".......................Starting Program......................."
    print ""

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT)) # Bind to the port
    s.listen(5) # Now wait for client connection.
    c, addr = s.accept() # Establish connection with client.

    print "Connected to UR"
    print ""
   
    ser_ee = serial.Serial('/dev/ttyACM0',9600)  # open serial port
    while ser_ee.isOpen()==False:
        print "Waiting for serial"
    print ser_ee.name, ": ",ser_ee.readline()         # check which port was really used
    print "Ready"
    return c, ser_ee

def main():
    c, ser_ee = initialize()
    # loop
    print c.recv(1024)
    inp = raw_input("Continue?")
    msg = ic.safe_move(c,ser_ee,Pose=dict(iw.home_joints),CMD=2)
    
    cali_img = cv2.imread("cali_img.jpg")
    circles_sorted, crop_points = ivt.run_calibration(cali_img, adjust=False)
    cali_circles_init = circles_sorted-circles_sorted[0][0]
    cali_circles=[]
    for circ in cali_circles_init[0]:
        cali_circles.append([circ[0], circ[1]])

    print "CALI_CIRCLES", cali_circles

    p1, inverse = ivt.pix3world_cal(cali_circles[0],cali_circles[2], cali_circles[1])
    
    CAMERA = 1
    ivt.check_camera()
    cam_check = raw_input("Change Camera?: " )
    if cam_check == "yes":
        print "Current camera is "+str(CAMERA)
        CAMERA = raw_input("Which Camera to use?: ")
        CAMERA = int(CAMERA)
    
    while True:
        task = raw_input("task: ")
        if task == "wp":
            msg = ic.safe_ur_move(c,Pose=dict(i2.pick_k),CMD=2)
            #current_Pose = ic.get_ur_position(c,1)
            #demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":i1.saucer_waypoint2["rx"], "ry":i1.saucer_waypoint2["ry"], "rz":i1.saucer_waypoint2["rz"]}
            #msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=4)
        if task == "1":
            print "Begin challenge 1..."
            i1.begin(c,ser_ee,p1,inverse,CAMERA,crop_points)
        if task == "2":
            print "Begin challenge 2..."
            i2.begin(c,ser_ee)
        if task == "3":
            print "Begin challenge 3..."
            i3.begin(c,ser_ee,p1,inverse,CAMERA,crop_points)
        if task == "4":
            print "Begin challenge 4..."
            i4.begin(c,ser_ee,p1,inverse,CAMERA,crop_points)
        if task == "5":
            print "Begin challenge 5..."
            i5.begin(c,ser_ee)
        if task == "6":
            print "Begin challenge 6..."
            i6.begin(c,ser_ee)
        if task == "7":
            print "Begin challenge 7..."
            i7.begin(c,ser_ee,p1,inverse,CAMERA,crop_points)
        if task == "8":
            print "Begin challenge 8..."
            i8.begin(c,ser_ee)
        if task == "9":
            print "Begin challenge 9..."
            i9.begin(c,ser_ee)
        if task == "10":
            print "Begin challenge 10..."
            i10.begin(c,ser_ee)
        if task == "s":
            print "Begin challenge s.."
            ic.serial_send(ser_ee,"H",100)
            time.sleep(1)
            ic.serial_send(ser_ee,"H",20)           
        if task == "pose":
            current_Pose, current_Grip = ic.get_position(c,ser_ee,CMD=1)
            print "current pose: ", current_Pose
            print "current grip: ", current_Grip
        if task == "joints":
            current_Joints, current_Grip = ic.get_position(c,ser_ee,CMD=3)
            print "current joints: ", current_Joints
            print "current grip: ", current_Grip
        if task == "grab":
            demand_Grip = dict(iw.ee_home)
            demand_Grip["act"] = int(raw_input("act: "))
            demand_Grip["servo"] = int(raw_input("servo: "))
            demand_Grip["tilt"] = int(raw_input("tilt: "))
            msg = ic.safe_move(c,ser_ee,Grip=demand_Grip, CMD=0)
            
        if task == "calibrate":
            while True:
                ready = raw_input("Ready?: ")
                if ready == "yes":
                    cali_img = ivt.capture_pic(CAMERA,ROTATION)
                    circles_sorted, crop_points = ivt.run_calibration(cali_img)
                    cali_circles_init = circles_sorted-circles_sorted[0][0]
                    cali_circles=[]
                    for circ in cali_circles_init[0]:
                        cali_circles.append([circ[0], circ[1]])

                    print cali_circles

                    p1, inverse = ivt.pix3world_cal(cali_circles[0],cali_circles[2], cali_circles[1])
                    cv2.imwrite("cali_img.jpg",cali_img)
                    break
        

if __name__ == '__main__': main()
