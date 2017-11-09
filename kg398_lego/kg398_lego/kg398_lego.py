#!/usr/bin/env python
import serial
import socket
import time
import random
import copy
import math

import ur_interface_cmds as ic
import ur_waypoints as wp
import lego_moves as lm

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
        if task == "lego":
            for i in range(0,3):
                lm.grid_pick(c,ser_ee,7+i*4,7,0,0)
                lm.grid_place(c,ser_ee,11+i*4,7,0,0)
            msg = ic.safe_ur_move(c,Pose=dict(wp.home_joints),CMD=2)
        if task == "wp":
            msg = ic.safe_ur_move(c,Pose=dict(wp.grid_0_1_joints),CMD=2)
        if task == "pose":
            current_Pose = ic.get_ur_position(c,1)
            print "current pose: ", current_Pose
        if task == "joints":
            current_Joints, current_Grip = ic.get_ur_position(c,3)
            print "current joints: ", current_Joints
        if task == "0":
            ic.serial_send(ser_ee,"G",48)
            while True:
                ipt = ser_ee.readline()
                print ipt
                if ipt == "done\r\n":
                    break
        if task == "1":
            ic.serial_send(ser_ee,"G",49)
            while True:
                ipt = ser_ee.readline()
                print ipt
                if ipt == "done\r\n":
                    break
        if task == "2":
            ic.serial_send(ser_ee,"G",50)
            while True:
                ipt = ser_ee.readline()
                print ipt
                if ipt == "done\r\n":
                    break
        if task == "i":
            print ser_ee.readline()
            
if __name__ == '__main__': main()