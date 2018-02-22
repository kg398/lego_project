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

Hopper0_feed0 = {"x": 373.00, "y": -457.94, "z": 9, "rx": 0.0*180/math.pi, "ry": 3.1415*180/math.pi, "rz": 0.0000*180/math.pi}
Hopper0_feed0_joints = {"x": 118.98, "y": -56.42, "z": 105.84, "rx": -139.41, "ry": -89.83, "rz": 29.26}

Hopper0_feed1 = {"x": 373.06, "y": -465.99, "z": 9, "rx": 0.0*180/math.pi, "ry": 3.1415*180/math.pi, "rz": 0.0000*180/math.pi}
Hopper0_feed1_joints = {"x": 118.61, "y": -55.71, "z": 104.74, "rx": -139.01, "ry": -89.82, "rz": 28.90}

Hopper0_feed2 = {"x": 373.86, "y": -473.69, "z": 9, "rx": 0.0*180/math.pi, "ry": 3.1407*180/math.pi, "rz": 0.0000*180/math.pi}
Hopper0_feed2_joints = {"x": 118.26, "y": -55.29, "z": 103.64, "rx": -138.33, "ry": -89.82, "rz": 28.58}

Hopper0_stow = {"x": 394.19, "y": 407.25, "z": 461.49, "rx": 0.0566*180/math.pi, "ry": 3.0581*180/math.pi, "rz": 0.7245*180/math.pi}
Hopper0_stow_joints = {"x": 208.78, "y": -85.12, "z": 78.26, "rx": -97.11, "ry": -66.75, "rz": 123.85}

Hopper0_stow_wp = {"x": 394.19, "y": 234.57, "z": 461.49, "rx": 0.0566*180/math.pi, "ry": 3.0581*180/math.pi, "rz": 0.7245*180/math.pi}
Hopper0_stow_wp_joints = {"x": 188.78, "y": -98.01, "z": 88.56, "rx": -85.35, "ry": -63.44, "rz": 102.04}

Hopper1_feed0 = {"x": 409.74, "y": -459.57, "z": 9, "rx": 0.0*180/math.pi, "ry": 3.1415*180/math.pi, "rz": 0.0000*180/math.pi}
Hopper1_feed0_joints = {"x": 121.94, "y": -54.07, "z": 101.25, "rx": -137.17, "ry": -89.81, "rz": 32.22}

Hopper1_feed1 = {"x": 409.45, "y": -467.41, "z": 9, "rx": 0.0*180/math.pi, "ry": 3.1415*180/math.pi, "rz": 0.0000*180/math.pi}
Hopper1_feed1_joints = {"x": 121.53, "y": -53.52, "z": 100.22, "rx": -136.70, "ry": -89.81, "rz": 31.80}

Hopper1_feed2 = {"x": 409.24, "y": -475.78, "z": 9, "rx": 0.0*180/math.pi, "ry": 3.1407*180/math.pi, "rz": 0.0000*180/math.pi}
Hopper1_feed2_joints = {"x": 121.11, "y": -52.75, "z": 99.13, "rx": -136.39, "ry": -89.80, "rz": 31.38}

Hopper1_stow = {"x": 431.41, "y": 407.29, "z": 461.49, "rx": 0.0566*180/math.pi, "ry": 3.0581*180/math.pi, "rz": 0.7245*180/math.pi}
Hopper1_stow_joints = {"x": 206.92, "y": -81.10, "z": 73.53, "rx": -95.61, "ry": -66.31, "rz": 121.86}

Hopper1_stow_wp = {"x": 431.41, "y": 264.10, "z": 461.49, "rx": 0.0566*180/math.pi, "ry": 3.0581*180/math.pi, "rz": 0.7245*180/math.pi}
Hopper1_stow_wp_joints = {"x": 191.66, "y": -91.69, "z": 83.17, "rx": -87.69, "ry": -63.71, "rz": 105.25}
