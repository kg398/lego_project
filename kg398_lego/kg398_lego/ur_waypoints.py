#!/usr/bin/env python
# Waypoints for ur joints and poses
import math

# Pose = [x, y, z, rx, ry, rz]
# Joints = [base, shoulder, elbow, wrist 1, wrist 2, wrist 3]
home = {"x": 90.0, "y": -500.0, "z": 100.0, "rx": 0.0, "ry": 180.0, "rz": 0.0}
home_joints = {"x": 87.61, "y": -87.40, "z": 100.79, "rx": -103.37, "ry": -89.70, "rz": -2.26}

ee_home = {"act": 80, "servo": 120, "tilt": 58}

grabbing_joints_waypoint = {"x": -27.33, "y": -83.04, "z": 105.47, "rx": -21.80, "ry": -27.13, "rz": -225.00}

grid_0_1 = {"x": -100.23, "y": -356.71, "z": 10.47, "rx": 0.0802*180/math.pi, "ry": 3.1344*180/math.pi, "rz": 0.0002*180/math.pi}
grid_0_1_joints = {"x": 57.76, "y": -77.16, "z": 142.67, "rx": -155.51, "ry": -89.59, "rz": -28.98}
grid_30_1 = {"x": 139.41, "y": -368.45, "z": 9.46, "rx": 0.0802*180/math.pi, "ry": 3.1344*180/math.pi, "rz": 0.0002*180/math.pi}
grid_30_1_joints = {"x": 95.37, "y": -74.15, "z": 138.42, "rx": -154.29, "ry": -89.55, "rz": 8.62}
grid_30_13 = {"x": 134.80, "y": -464.40, "z": 10.05, "rx": 0.0802*180/math.pi, "ry": 3.1344*180/math.pi, "rz": 0.0002*180/math.pi}
grid_30_13_joints = {"x": 93.73, "y": -66.27, "z": 124.36, "rx": -148.10, "ry": -89.52, "rz": 6.97}

