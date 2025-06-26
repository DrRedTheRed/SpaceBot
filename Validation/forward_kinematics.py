# Irregular DH table -> xyz & rpy of the end effector

import numpy as np
from math import sin, cos, pi

# input DH parameters
# if left origin
alpha_l = np.array([0, pi/2, -pi/2, 0, 0, pi/2, pi/2])
a_l = np.array([0, 0, 0, 400, -400, 0, 0]) / 1000
d_l = np.array([120, -100, -100, -100, -100, 100, 120]) / 1000
theta_origin_l = np.array([0, pi/2, 0, 0, 0, -pi/2, 0])

# if right origin
alpha_r = np.array([0, pi/2, -pi/2, 0, 0, pi/2, pi/2])
a_r = np.array([0, 0, 0, 400, -400, 0, 0]) / 1000
d_r = np.array([120, -100, 100, 100, 100, 100, 120]) / 1000
theta_origin_r = np.array([0, pi/2, 0, 0, 0, -pi/2, 0])

# homogenous transformation matrix
def getT(alpha, a, d, theta):
    T = np.array([[cos(theta), -sin(theta), 0, a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                [0, 0, 0, 1]])
    for i in range(3):
        for j in range(3):
            if abs(T[i,j]) < 1e-8:
                T[i,j] = 0
    return T
    

# XYZ euler angles (roll, pitch, yaw)
def getRPY(T):
    T02_restricted = np.clip(T[0,2], -1, 1) # avoid nan from arcsin
    rpy = np.zeros(3)
    rpy[0] = np.arctan2(-T[1,2],T[2,2])/pi*180
    rpy[1] = np.arcsin(T02_restricted)/pi*180
    rpy[2] = np.arctan2(-T[0,1],T[0,0])/pi*180
    for i in range(3):
        if abs(rpy[i]) < 1e-8:
            rpy[i] = 0
    # print("RPY(in degrees):", rpy)
    return rpy

# xyz position
# theta_offset is the offset of the joint angles from the origin
def getXYZ(alpha, a, d, theta_origin, theta_offset):
    T = np.eye(4)
    for i in range(7):
        T_i = getT(alpha[i], a[i], d[i], theta_origin[i] + theta_offset[i])
        T = T @ T_i
    xyz = T[:3, 3]
    # print("XYZ(in meters):", xyz)
    return xyz, T

# forward kinematics
def FK(theta_offset, direction):
    if direction == 'L':
        xyz, T_whole = getXYZ(alpha_l, a_l, d_l, theta_origin_l, theta_offset)
        rpy = getRPY(T_whole)
        return xyz, rpy, T_whole
    elif direction == 'R':
        xyz, T_whole = getXYZ(alpha_r, a_r, d_r, theta_origin_r, theta_offset)
        rpy = getRPY(T_whole)
        return xyz, rpy, T_whole

if __name__ == "__main__":
    theta_offset = input("Enter joint angles in degrees: ")
    theta_offset = np.array([float(x) for x in theta_offset.split()]) / 180 * pi
    direction = input("Direction(L/R): ")
    xyz, rpy, T_whole = FK(theta_offset, direction)
    xyz_str, rpy_str, T_str = np.array2string(xyz, precision=4, suppress_small=True), np.array2string(rpy, precision=4, suppress_small=True), np.array2string(T_whole, precision=4, suppress_small=True)
    print(f"XYZ: {xyz_str}")
    print(f"RPY: {rpy_str}")
    print(f"T:\n{T_str}")