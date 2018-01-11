#!/usr/bin/env python
# Waypoints for ur joints and poses

# Pose = [x, y, z, rx, ry, rz]
# Joints = [base, shoulder, elbow, wrist 1, wrist 2, wrist 3]
home = {"x": 90.0, "y": -500.0, "z": 100.0, "rx": 0.0, "ry": 180.0, "rz": 0.0}
home_joints = {"x": 87.61, "y": -87.40, "z": 100.79, "rx": -103.37, "ry": -89.70, "rz": -2.26}

ee_home = {"act": 80, "servo": 120, "tilt": 58}

grabbing_joints_waypoint = {"x": -27.33, "y": -83.04, "z": 105.47, "rx": -21.80, "ry": -27.13, "rz": -225.00}
