#!/usr/bin/env python
# Waypoints for ur joints and poses
import math

# Pose = [x, y, z, rx, ry, rz]
# Joints = [base, shoulder, elbow, wrist 1, wrist 2, wrist 3]
# home position, arbitrary
home = {"x": 90.0, "y": -500.0, "z": 100.0, "rx": 0.0, "ry": 180.0, "rz": 0.0}
home_joints = {"x": 87.61, "y": -87.40, "z": 100.79, "rx": -103.37, "ry": -89.70, "rz": -2.26}

# Lego calibration points
grid_0_1 = {"x": -100.29, "y": -356.57, "z": 7.37, "rx": 0.0765*180/math.pi, "ry": 3.1408*180/math.pi, "rz": 0.0001*180/math.pi}
grid_0_1_joints = {"x": 57.92, "y": -76.53, "z": 142.74, "rx": -155.19, "ry": -89.90, "rz": -28.97}

grid_30_1 = {"x": 138.81, "y": -368.54, "z": 6.30, "rx": 0.0765*180/math.pi, "ry": 3.1408*180/math.pi, "rz": 0.0001*180/math.pi}
grid_30_1_joints = {"x": 95.31, "y": -73.53, "z": 138.44, "rx": -154.89, "ry": -89.90, "rz": 8.43}

grid_30_13 = {"x": 134.78, "y": -464.03, "z": 6.79, "rx": 0.0765*180/math.pi, "ry": 3.1408*180/math.pi, "rz": 0.0001*180/math.pi}
grid_30_13_joints = {"x": 93.82, "y": -65.83, "z": 124.45, "rx": -148.60, "ry": -89.87, "rz": 6.93}

grid_0_13 = {"x": -105.30, "y": -453.14, "z": 6.37, "rx": 0.0765*180/math.pi, "ry": 3.1408*180/math.pi, "rz": 0.0001*180/math.pi}
grid_0_13_joints = {"x": 64.00, "y": -67.47, "z": 127.91, "rx": -150.60, "ry": -89.88, "rz": -22.90}

grid_30_1_10 = {"x": 139.21, "y": -367.78, "z": 92.28, "rx": 0.0764*180/math.pi, "ry": 3.1407*180/math.pi, "rz": 0.0000*180/math.pi}
grid_30_1_10_joints = {"x": 95.48, "y": -87.63, "z": 134.48, "rx": -136.84, "ry": -89.80, "rz": 8.54}

