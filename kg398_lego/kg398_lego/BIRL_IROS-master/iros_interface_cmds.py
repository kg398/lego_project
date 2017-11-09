#!/usr/bin/env python
# Grasping strategy for each classified object
import socket
import serial
import time
import math
import copy

import iros_waypoints as iw

# Socket CMDs
# Send (Pose, Command, Variable) to the UR5 using the socket connection c
# Returns string recieved from the UR
# List of CMDs to UR5:
# 0 - moves to Pose, Variable = max joint speed, UR returns string "complted_pose_move" upon completion 
# 1 - queries UR for current Pose, Variable = n.a., UR returns string "p[current_Pose]"
# 2 - moves to Joint positions, Variable = max joint speed, UR returns string "complted_joint_move" upon completion 
# 3 - queries UR for current Joint values, Variable = n.a., UR returns string "[current_Joints]"
# 4 - safe move to Pose-if any joints change by more than the threshold angle(2rad) Pose is rejected, if using safe_ur_move, move is subsampled until not rejected, Variable = max joint speed, UR returns string "no_safe_move_found" upon rejection, "complted_pose_move" upon completion 
# 5 - moves to new z position unless z force is exceeded, Variable = max joint speed, UR returns string final z position if force exceeded, else returns "0" upon completion
# 6 - queries UR for current Force, Variable = n.a., UR returns string "p[forces]"
# 7 - queries UR for current Joint torques, Variable = n.a., UR returns string "[torques]"
# 8 - moves linearly to Pose, Variable = max joint speed, UR returns string "complted_linear_move" upon completion
#
#
# 100 - set tool centre point to tcp_1 (only z offset from end), Variable = n.a., UR returns string "tool_1_selected"
# 101 - set tool centre point to tcp_2 (centre used for vector rotations), Variable = n.a., UR returns string "tool_2_selected"
def socket_send(c, sPose=dict(iw.home), sSpeed = 0.75, sCMD = 0):
    sendPose = dict(sPose)
    msg = "Failed"
    try:
        # Send formatted CMD
        c.send("("+str(sendPose["x"])+","+str(sendPose["y"])+","+str(sendPose["z"])+","+str(sendPose["rx"])+","+str(sendPose["ry"])+","+str(sendPose["rz"])+","+str(sCMD)+","+str(sSpeed)+")");
        print "Sent ur move"
        # Wait for reply
        msg=c.recv(1024)
        print msg
        print ""
    except socket.error as socketerror:
        print ".......................Some kind of error :(......................."
    # Return reply
    return msg

def serial_send(ser_ee,id,var):
    # Serial CMDs
    #print "Sending end effector move"
    ser_ee.flush
    # Set Actuator position, min = 0, max = 80
    ser_ee.write(id + chr(var) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    return

# Safe Move CMDs
# Send socket and serial CMDs
# Returns reply from UR
def safe_move(c,ser_ee,Pose=dict(iw.home),Speed=0.75,Grip=dict(iw.ee_home),CMD=4):
    # Socket CMDs
    print "Sending ur move"
    msg = safe_ur_move(c,dict(Pose),CMD,Speed=Speed)

    # Serial CMDs
    print "Sending end effector move"
    end_effector_move(ser_ee,dict(Grip))
    return msg

#
#
#
def end_effector_move(ser_ee,Grip):
    # Serial CMDs
    serial_send(ser_ee,"A",Grip["act"])
    ipt = ser_ee.readline()
    #print "sw state = ",ipt
    ipt = ser_ee.readline()
    #print "Timeout = ",ipt

    serial_send(ser_ee,"G",Grip["servo"])

    serial_send(ser_ee,"T",Grip["tilt"])
    return

# Safe Move CMDs
# Send socket CMDs without Serial CMDs
# Returns reply from UR
def safe_ur_move(c,Pose,CMD,Speed=0.75):
    # Socket CMDs
    sendPose = dict(Pose)
    if CMD == 4:
        # Safe move
        demand_Pose = dict(sendPose)
        #print "demand_Pose: ", demand_Pose
        msg = "no_safe_move_found"
        n = 1           # Number of steps to divide the remaining move into
        alpha = 1.0     # Interpolation factor
        # Subsample pose until steps are small enough
        # e.g. demand_Pose rejected -> move split into 2 steps -> first step accepted -> second step accepted -> returns "completed_safe_move"
        #                                                                             -> second step rejected -> remaining n steps split into n+1 steps etc...
        #                                                      -> first step rejected -> move split into 3 steps etc...
        while msg == "no_safe_move_found":
            current_Pose = get_ur_position(c,CMD=1)
            #print "current_pose: ", current_Pose
            Pose2 = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
            alpha = 1.0/n
            for i in range(1,n+1):
                #print n
                # Interpolate from current position to demand position in n steps
                i_Pose = interpolate_pose(demand_Pose,Pose2,alpha*i)
                #print i_Pose
                # Send new demand_Pose
                msg = socket_send(c,sPose=i_Pose,sSpeed=0.75,sCMD=4)
                # If rejected, increment number of steps and restart for loop
                if msg == "no_safe_move_found":
                    n = n+1
                    time.sleep(0.5)
                    break
                # If accepted, decrement number of steps and continue for loop
                else:
                    n = n-1
    else:
        # Non-safe move
        msg = socket_send(c,sPose=sendPose,sSpeed=Speed,sCMD=CMD)

    return msg

# Query CMDs
# Send socket and serial CMDs
# Returns decoded replies: [current robot position], [actuator angle, servo pos, tilt pos, switch state, vac state]
def get_position(c,ser_ee,Pose=dict(iw.home),Speed=0.75,CMD=1):
    # Socket CMDs
    current_position = get_ur_position(c,CMD,dict(Pose),Speed)

    # Serial CMDs
    current_grip = get_ee_position(ser_ee)

    return current_position, current_grip 

#
#
#
def get_ur_position(c,CMD,gPose=dict(iw.home),gSpeed=0.75):
    # Initialize variables
    sendPose = dict(gPose)
    msg = "0"

    # Socket CMDs
    msg = socket_send(c, sPose=sendPose, sSpeed=gSpeed, sCMD=CMD)
    #print "recieved: ",msg

    # Decode Pose or Joints from UR
    current_position = [0,0,0,0,0,0]
    data_start = 0
    data_end = 0
    n = 0
    x = 0
    while x < len(msg):
        if msg[x]=="," or msg[x]=="]" or msg[x]=="e":
            data_end = x
            current_position[n] = float(msg[data_start:data_end])
            if msg[x]=="e":
                current_position[n] = current_position[n]*math.pow(10,float(msg[x+1:x+4]))
                #print "e", msg[x+1:x+4]
                #print "e", int(msg[x+1:x+4])
                if n < 5:
                    x = x+5
                    data_start = x
                else:
                    break
            n=n+1
        if msg[x]=="[" or msg[x]==",":
            data_start = x+1
        x = x+1

    # Convert x,y,z values to mm, convert rotations to degrees
    for x in range(0,3):
        if CMD==1 or CMD==8: current_position[x] = current_position[x]*1000.0
        if CMD==3: current_position[x] = current_position[x]*180.0/math.pi
        current_position[x+3] = current_position[x+3]*180.0/math.pi

    #print "decoded msg: ",current_position
    return current_position

#
#
#
def get_ee_position(ser_ee):
    # Initialize variables
    ipta = "0"
    iptb = "0"
    iptc = "0"

    # Serial CMDs
    # Query end effector arduino state
    ser_ee.write("R" + "0" + "\n")

    ipta = ser_ee.readline() #actuator angle
    iptb = ser_ee.readline() #grabber servo pos
    iptc = ser_ee.readline() #tilt servo pos
    iptd = ser_ee.readline() #grabber switch state
    #print ipt1a
    #print ipt1b
    #print ipt1c
    #print ipt1d

    # Decode Serial Data
    ipta = ipta[0:len(ipta)-2]
    iptb = iptb[0:len(iptb)-2]
    iptc = iptc[0:len(iptc)-2]
    iptd = iptd[0:len(iptd)-2]

    return [int(ipta)+300, int(iptb), int(iptc), int(iptd)] 


# Query CMDs
# Send socket CMD=6
# Unused as force now sent as a vector and decoded by get_position()
def get_force(c):
    msg = socket_send(c, sPose=dict(iw.home), sCMD=6)
    #print "force: ", msg
    return msg

# Query CMDs
# Send socket CMD=7
# Unused as torque now sent as a vector and decoded by get_position()
def get_torque(c):
    msg = socket_send(c, sPose=dict(iw.home), sCMD=7)
    #print "torque: ", msg
    return msg

# Finds new pose linearly interpotlated from Pose2 to Pose1
# Returns new pose
def interpolate_pose(Pose1, Pose2, alpha):
    i_Pose = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}
    i_Pose1 = dict(Pose1)
    i_Pose2 = dict(Pose2)
    i_Pose["x"] = alpha*i_Pose1["x"] + (1.0-alpha)*i_Pose2["x"]
    i_Pose["y"] = alpha*i_Pose1["y"] + (1.0-alpha)*i_Pose2["y"]
    i_Pose["z"] = alpha*i_Pose1["z"] + (1.0-alpha)*i_Pose2["z"]
    i_Pose["rx"] = alpha*i_Pose1["rx"] + (1.0-alpha)*i_Pose2["rx"]
    i_Pose["ry"] = alpha*i_Pose1["ry"] + (1.0-alpha)*i_Pose2["ry"]
    i_Pose["rz"] = alpha*i_Pose1["rz"] + (1.0-alpha)*i_Pose2["rz"]
    return i_Pose

# Alternate end effector arduino CMD
# Returns switch state(0=open, 1=closed), data(timed out if !=0)
def ee_pinch(ser_ee, act):
    ser_ee.flush
    print "Sending end effector move"
    # Set Pinch position, min = 0, max = 80
    ser_ee.write("P" + chr(act) + "\n")
    # Wait for end effector arduino to finish 
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    # Red additional data
    msg = ser_ee.readline()
    msg = int(msg[0:len(msg)-2])
    timeout = ser_ee.readline()
    timeout = int(timeout[0:len(timeout)-2])
    return msg, timeout
