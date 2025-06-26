import numpy as np
from scipy.spatial.transform import Rotation
import numpy as np
from numpy import cos, sin, arcsin, arccos, arctan2, sqrt, pi
import math
#通过枚举x1来解出剩下的所有角
def calc_angle2and6(r, x, y, z, x1):
    a1 = r[0, 2] * sin(x1) - r[1, 2] * cos(x1) #cos的系数
    b1 = -r[2, 2] #sin的系数
    a2 = x * sin(x1) - y * cos(x1) #cos系数2
    b2 = 120 - z #sin系数2
    a = a2 - 120 * a1 #acosx + bsinx = -300
    b = b2 - 120 * b1
    # print(a1, b1, a2, b2, a, b)
    # x2_1 = arcsin(-300 / sqrt(a ** 2 + b ** 2)) - arctan2(a, b)
    x2_1 = arctan2(-300, sqrt(a ** 2 + b ** 2 - 300 ** 2)) - arctan2(a, b)
    x2_2 = arctan2(-300, -sqrt(a ** 2 + b ** 2 - 300 ** 2)) - arctan2(a, b)
    # print(a1 * np.cos(x2_1) + b1 * sin(x2_1))
    x6_1 = -arcsin(a1 * cos(x2_1) + b1 * sin(x2_1))
    x6_3 = np.pi - x6_1
    x6_2 = -arcsin(a1 * cos(x2_2) + b1 * sin(x2_2))
    x6_4 = np.pi - x6_2
    return x2_1, x2_2, x6_1, x6_2, x6_3, x6_4

def calc_angle7(r, x1, x2, x6):
    s = (r[0, 1] * cos(x2) *sin(x1) - r[1, 1] * cos(x1)*cos(x2) - r[2,1] * sin(x2)) / cos(x6) # 2024.7.4 wzh改
    c = -((r[0, 0] * cos(x2) *sin(x1) - r[1, 0] * cos(x1)*cos(x2) - r[2,0] * sin(x2))) / cos(x6) # 2024.7.4 wzh改
    print("s, c:", s, c)
    x7 = arctan2(s, c)
    return x7

def calc_angle345(r, x1, x6, x7):
    m = -r[0, 0] * cos(x1) - r[1, 0] * sin(x1)
    b = cos(x7) * sin(x6)
    a = -sin(x7)
    print("x345:", arcsin(m / sqrt(a ** 2 + b ** 2)) - arctan2(a, b))
    return arcsin(m / sqrt(a ** 2 + b ** 2)) - arctan2(a, b)

# 2024.7.5 wzh改
# def calc_angle345(r, x1, x6, x7):
#     k1 = -r[0, 1] * cos(x1) - r[1, 1] * sin(x1)
#     k2 = -r[0, 0] * cos(x1) - r[1, 0] * sin(x1)
#     s345 = (cos(x7) * k2 - sin(x7) * k1) / sin(x6)
#     c345 = -sin(x7) * k2 - cos(x7) * k1
#     print("x345:", arctan2(s345, c345))
#     return arctan2(s345, c345)

def calc_angle34(x, y, z, x1, x2, x6, x345):
    #400*cos((pi*x3)/180) - 400*cos((pi*(x3 + x4))/180) = (120*cos(x345 + x6))/2 - (120*cos((x345 - x6)))/2 + 100*sin(x345)
    m1 = (120*cos(x345)*cos(x6) - 100*sin(x345) + (z - 120) * cos(x2) + x * sin(x1) * sin(x2) - y * cos(x1) * sin(x2)) / 400
    #400*sin((pi*x3)/180) - 400*sin((pi*(x3 + x4))/180) = (120*sin(x345 + x6))/2 + (120*sin((x345 - x6)))/2 + 100*cos(x345)
    m2 = (120*sin(x345)*cos(x6) + 100*cos(x345) - x * cos(x1) - y * sin(x1) - 100) / 400
    # print("m1, m2:", m1, m2)
    # #(2x3+x4) / 2
    # gamma = arctan2(m1, -m2)
    # print("gamma:", gamma)
    # x4_1 = 0
    # x4_2 = 0
    # if(sin(gamma) > 1e-10):
    #     x4_1 = 2 * arcsin(m1 / 2 / sin(gamma))
    #     x4_2 = 2 * np.pi - x4_1
    # else:
    #     x4_1 = 2 * arcsin(-m2 / 2 / cos(gamma))
    #     x4_2 = 2 * np.pi - x4_1
    # x3_1 = gamma - x4_1 / 2
    # x3_2 = gamma - x4_2 / 2
    x4_1 = arccos(1 - (m1 ** 2 + m2 ** 2) / 2)
    x4_2 = -x4_1
    x3_1 = arctan2(((1+cos(x4_1))*m1 + sin(x4_1)*m2)/(2*sin(x4_1)), ((1-cos(x4_1))*m1 - sin(x4_1)*m2)/(2*(1-cos(x4_1))))
    x3_2 = arctan2(((1+cos(x4_2))*m1 + sin(x4_2)*m2)/(2*sin(x4_2)), ((1-cos(x4_2))*m1 - sin(x4_2)*m2)/(2*(1-cos(x4_2))))
    x5_1 = x345 - x3_1 - x4_1
    x5_2 = x345 - x3_2 - x4_2
    print("x3_1, x3_2:", x3_1, x3_2)
    print("x4_1, x4_2:", x4_1, x4_2)
    return x3_1, x4_1, x5_1, x3_2, x4_2, x5_2

def solve(T_target, x1):
    x = T_target[0, 3]
    y = T_target[1, 3]
    z = T_target[2, 3]
    r = T_target[0:3, 0:3]
    print("xyz:", x, y, z)
    #先计算x2
    x2_1, x2_2, x6_1, x6_2, x6_3, x6_4 = calc_angle2and6(r, x, y, z, x1) #x2_1和x6_1，3对应
    print("angle2:", x2_1, x2_2)
    print("angle6:", x6_1, x6_2, x6_3, x6_4)
    x7_1 = calc_angle7(r, x1, x2_1, x6_1)
    x7_2 = calc_angle7(r, x1, x2_2, x6_2)
    x7_3 = calc_angle7(r, x1, x2_1, x6_3)
    x7_4 = calc_angle7(r, x1, x2_2, x6_4)
    print("angle7:", x7_1, x7_2, x7_3, x7_4)
    x345_1 = calc_angle345(r, x1, x6_1, x7_1)
    x345_2 = calc_angle345(r, x1, x6_2, x7_2)
    x345_3 = calc_angle345(r, x1, x6_3, x7_3)
    x345_4 = calc_angle345(r, x1, x6_4, x7_4)
    print("angle345:", x345_1, x345_2, x345_3, x345_4)
    x3_1, x4_1, x5_1, x3_5, x4_5, x5_5 = calc_angle34(x, y, z, x1, x2_1, x6_1, x345_1)
    x3_2, x4_2, x5_2, x3_6, x4_6, x5_6 = calc_angle34(x, y, z, x1, x2_2, x6_2, x345_2)
    x3_3, x4_3, x5_3, x3_7, x4_7, x5_7 = calc_angle34(x, y, z, x1, x2_1, x6_3, x345_3)
    x3_4, x4_4, x5_4, x3_8, x4_8, x5_8 = calc_angle34(x, y, z, x1, x2_2, x6_4, x345_4)
    print("angle3:", x3_1, x3_2, x3_3, x3_4, x3_5, x3_6, x3_7, x3_8)
    print("angle4:", x4_1, x4_2, x4_3, x4_4, x4_5, x4_6, x4_7, x4_8)
    print("angle5:", x5_1, x5_2, x5_3, x5_4, x5_5, x5_6, x5_7, x5_8)
    ans1 = [x1, x2_1, x3_1, x4_1, x5_1, x6_1, x7_1]
    ans2 = [x1, x2_1, x3_5, x4_5, x5_5, x6_1, x7_1]
    ans3 = [x1, x2_1, x3_3, x4_3, x5_3, x6_3, x7_3]
    ans4 = [x1, x2_1, x3_7, x4_7, x5_7, x6_3, x7_3]
    ans5 = [x1, x2_2, x3_2, x4_2, x5_2, x6_2, x7_2]
    ans6 = [x1, x2_2, x3_6, x4_6, x5_6, x6_2, x7_2]
    ans7 = [x1, x2_2, x3_4, x4_4, x5_4, x6_4, x7_4]
    ans8 = [x1, x2_2, x3_8, x4_8, x5_8, x6_4, x7_4]
    return [ans1, ans2, ans3, ans4, ans5, ans6, ans7, ans8]

def euler_to_rotation_matrix(roll, pitch, yaw):
    roll_rad = roll
    pitch_rad = pitch
    yaw_rad = yaw

    # 计算旋转矩阵
    rotation_matrix_roll = np.array([[1, 0, 0],
                                      [0, np.cos(roll_rad), -np.sin(roll_rad)],
                                      [0, np.sin(roll_rad), np.cos(roll_rad)]])

    rotation_matrix_pitch = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                                       [0, 1, 0],
                                       [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])

    rotation_matrix_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                                     [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                                     [0, 0, 1]])

    # 将三个旋转矩阵相乘得到最终的旋转矩阵
    rotation_matrix = np.dot(rotation_matrix_roll, np.dot(rotation_matrix_pitch, rotation_matrix_yaw))

    return rotation_matrix

def inv(pos, x1):# "pos" is a list
    # euler_verse = np.array([[0, -1, 0, 0],
    #                         [0, 0, -1, 0],
    #                         [1, 0, 0, 0],
    #                         [0, 0, 0, 1]])
    euler_verse = np.array([[0, 0, 1, 0],
                            [-1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, 0, 1]])
    # converse = np.array([[0, -1, 0, 0],
    #                      [1, 0, 0, 0],
    #                      [0, 0, 1, 0],
    #                      [0, 0, 0, 1]])
    converse = np.array([[0, 1, 0, 0],
                         [-1, 0, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    P = np.array([pos[0], pos[1], pos[2]]).T
    rotateM = euler_to_rotation_matrix(pos[3], pos[4], pos[5])
    print(np.linalg.inv(euler_verse))
    # T = np.dot(converse, np.vstack((np.hstack((rotateM, P[:, None])), np.array([0, 0, 0, 1])))) #得到最终的矩阵
    # T = np.dot(T, euler_verse)
    T = np.vstack((np.hstack((rotateM, P[:, None])), np.array([0, 0, 0, 1]))) #得到最终的矩阵
    print(T)
    answer = solve(T, x1)
    return answer

# print(euler_to_rotation_matrix(0, np.pi / 2, 0))

# answer = inv([0, 600, 0, np.pi, 0, -np.pi/2], 0)
# answer = inv([0, 300, 0, np.pi, 0, -np.pi/2], 0)
# -----第二步 L_base-----
# answer = inv([0, 600, 20, np.pi, 0, -np.pi/2], 0)
answer = inv([120, 400, -100, np.pi/2, 0, np.pi/2, ], 2/3*pi+4.92659305-2*pi)

# q_m2_0 = np.array([q1[6], -q1[5], 2*q1[4], 2*q1[3], 2*q1[2], -q1[1], q1[0]]) \
#     + np.array([0, -pi, 0, 0, 0, 0, 0]) \
#     + np.array([0, -1.080839000541168, 1.974018405462387, -1.5407918249714194, -2.7083660730988255, 2.060753653048625, 0]) \
#     - np.flip(q1)
q_m2_0 = np.array([0, 1.0141970087846741, 1.1675742481274056, 1.54079182497142, -0.4332265804909674, -2.1273956448051194, 3.141592653589793])
for ans in answer:
    #控制范围在[-pi, pi]之间
    for i in range(len(ans)):
        if(ans[i] > np.pi): ans[i] -= np.pi * 2
        if(ans[i] < -np.pi): ans[i] += np.pi * 2
    ans[1] = -ans[1]
    ans[2] = -ans[2]
    ans[3] = -ans[3]
    ans[5] = -ans[5]
    ans[6] = -ans[6]
    sum = 0
    for i in range(len(ans)):
        sum = sum + abs(q_m2_0[i] - ans[i])
    print(ans, sum)

l1 = np.array([0, -2.790686821121354, 0.6441645843738765, 1.2424883197633365, 0.31585657734231576, -3.141592653589793, 2.790686821121354])
l2 = np.array([0, -2.790686821121354, 0.6942503159010794, 1.2252700800130993, 0.34872406911928155, -3.141592653589793, 2.790686821121354])
print(l2-l1)