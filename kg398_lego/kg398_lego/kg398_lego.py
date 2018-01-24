#!/usr/bin/env python
# main functon for running scripts and testing lego project
import serial
import socket
import time
import random
import copy
import math
import numpy as np

import ur_interface_cmds as ic
import ur_waypoints as wp
import lego_moves as lm
import file_decoder as fd

def initialize():
    #HOST = "169.254.103.235" # The remote host
    HOST = "192.168.1.105" # The remote host
    HOST = "129.169.80.9"
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
   
    ser_ee = serial.Serial("COM28",9600)  # open serial port
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
    msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
    
    while True:
        task = raw_input("task: ")
        if task == "cal":
            ipt = raw_input("Continue?")
            lm.grid_pick(c,ser_ee,0,1,0,0)
            lm.grid_place(c,ser_ee,0,1,0,0)
            ipt = raw_input("Continue?")
            lm.grid_pick(c,ser_ee,30,1,0,0)
            lm.grid_place(c,ser_ee,30,1,0,0)
            ipt = raw_input("Continue?")
            lm.grid_pick(c,ser_ee,30,13,0,0)
            lm.grid_place(c,ser_ee,30,13,0,0)
            ipt = raw_input("Continue?")
            lm.grid_pick(c,ser_ee,0,13,0,0)
            lm.grid_place(c,ser_ee,0,13,0,0)
            ipt = raw_input("Continue?")
            msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
        if task == "stack":
            msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
            time.sleep(3)
            lm.stack_pick(c,ser_ee,0,1,2,0,stack=3)
            lm.grid_place(c,ser_ee,30,1,2,0)
        if task == "rot":
            msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
            ic.socket_send(c,sCMD=300)
            current_Pose = ic.get_ur_position(c,1)
            demand_Pose = {"x":current_Pose[0], "y":current_Pose[1], "z":current_Pose[2], "rx":current_Pose[3], "ry":current_Pose[4], "rz":current_Pose[5]}
            print "current_Pose: ",demand_Pose
            demand_Pose = dict(lm.smooth_rotate(c,0,R=20))
            print "demand_Pose: ",demand_Pose
            ipt = raw_input("continue?")
            msg = ic.safe_ur_move(c,Pose=demand_Pose,CMD=8)
#        if task == "lego":
#            #time.sleep(5)
#            ipt = raw_input("Open?(y/n)")
#            if ipt == "y":
#                ic.super_serial_send(ser_ee,"G",51)
#            for i in range(0,3):
#                for j in range(0,4):
#                    for k in range(0,2):
#                        lm.grid_pick(c,ser_ee,j*8+3,k*6+1,4,0)
#                        lm.grid_place(c,ser_ee,j*8+3,k*6+7,4,0)
#                    msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
#                for j in range(0,4):
#                    for k in range(0,2):
#                        lm.grid_pick(c,ser_ee,27-j*8,13-k*6,4,0)
#                        lm.grid_place(c,ser_ee,27-j*8,7-k*6,4,0)
#                    msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
        if task == "lego":
            ic.socket_send(c,sCMD=300)
            R = int(raw_input("angle:"))
            lm.grid_pick(c,ser_ee,1,1,5,R)
            lm.grid_place(c,ser_ee,1,1,5,R)
        if task == "pt":
            #time.sleep(5)
            ipt = raw_input("Open?(y/n)")
            if ipt == "y":
                ic.super_serial_send(ser_ee,"G",51)
            for i in range(0,4):
                for j in range(0,4):
                    for k in range(0,2):
                        lm.grid_pick(c,ser_ee,((j+k)*8+3)%32,k*6+1,1,0)
                        lm.grid_place(c,ser_ee,((j+k+1)*8+3)%32,k*6+7,1,0)
                    msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
                for j in range(0,4):
                    lm.grid_pick(c,ser_ee,j*8+3,13,1,0)
                    lm.grid_place(c,ser_ee,((j+1)*8+3)%32,1,1,0)
                msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
        if task == "wp":
            msg = ic.safe_ur_move(c,Pose=dict(wp.grid_0_1_joints),CMD=2)
        if task == "pose":
            current_Pose = ic.get_ur_position(c,1)
            print "current pose: ", current_Pose
        if task == "joints":
            current_Joints, current_Grip = ic.get_ur_position(c,3)
            print "current joints: ", current_Joints

        if task == "file":
            ipt = raw_input("Open?(y/n)")
            if ipt == "y":
                ic.super_serial_send(ser_ee,"G",51)
            model = fd.import_file("example.txt")
            bricks = fd.decode_file(model)
            que = fd.sort_bricks_dis(bricks,model)
            #print bricks
            #ipt = raw_input("continue?")
            #lm.assemble(c,ser_ee,bricks)
            #ipt = raw_input("continue?")
            #lm.disassemble(c,ser_ee,bricks)
        if task == "i":
            print ser_ee.readline()
            
if __name__ == '__main__': main()