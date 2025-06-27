import numpy as np
from scipy.spatial.transform import Rotation
import numpy as np
from numpy import cos, sin, arcsin, arccos, arctan2, sqrt, pi
import math

#通过枚举x1来解出剩下的所有角
def calc_angle2and6(r, x, y, z, x1, base_dir):
    a1 = -r[0, 2] * cos(x1) - r[1, 2] * sin(x1) #cos的系数
    b1 = -r[2, 2] #sin的系数
    a2 = -x * cos(x1) - y * sin(x1) #cos系数2
    b2 = 120 - z #sin系数2
    if base_dir == "R":
        a = a2 + 120 * a1 #acosx + bsinx = 300
        b = b2 + 120 * b1
        x2_1 = arctan2(300, sqrt(a ** 2 + b ** 2 - 300 ** 2)) - arctan2(a, b)
        x2_2 = arctan2(300, -sqrt(a ** 2 + b ** 2 - 300 ** 2)) - arctan2(a, b)
    else:
        a = a2 - 120 * a1 #acosx + bsinx = -300
        b = b2 - 120 * b1
        x2_1 = arctan2(-300, sqrt(a ** 2 + b ** 2 - 300 ** 2)) - arctan2(a, b)
        x2_2 = arctan2(-300, -sqrt(a ** 2 + b ** 2 - 300 ** 2)) - arctan2(a, b)
    x6_1 = -arcsin(a1 * cos(x2_1) + b1 * sin(x2_1))
    x6_3 = np.pi - x6_1
    x6_2 = -arcsin(a1 * cos(x2_2) + b1 * sin(x2_2))
    x6_4 = np.pi - x6_2
    return x2_1, x2_2, x6_1, x6_2, x6_3, x6_4

def calc_angle7(r, x1, x2, x6):
    s = (- r[0, 1] * cos(x2) *cos(x1) - r[1, 1] * sin(x1)*cos(x2) - r[2,1] * sin(x2)) / cos(x6) # 2024.7.4 wzh改
    c = -((- r[0, 0] * cos(x2) *cos(x1) - r[1, 0] * sin(x1)*cos(x2) - r[2,0] * sin(x2))) / cos(x6) # 2024.7.4 wzh改
    print("s, c:", s, c)
    x7 = arctan2(s, c)
    return x7

def calc_angle345(r, x1,x2, x6, x7):
    """
    根据变换矩阵方程求解角度345的和
    T7^2(3,1) = s7*s345 - c7*s6*c345 = s1*r11 - c1*r21
    T7^2(3,2) = c7*c345 + s7*s6*s345 = s1*r12 - c1*r22
    """
    # 计算右侧已知量
    rhs1 = sin(x1) * r[0, 0] - cos(x1) * r[1, 0]  # s1*r11 - c1*r21
    rhs2 = sin(x1) * r[0, 1] - cos(x1) * r[1, 1]  # s1*r12 - c1*r22
    
    # 系数矩阵和求解
    # s345 * s7 + c345 * (-c7*s6) = rhs1
    # s345 * (s7*s6) + c345 * c7 = rhs2
    A = np.array([[-cos(x7) * sin(x6), sin(x7)],
                  [sin(x7) * sin(x6), cos(x7)]])
    b = np.array([rhs1, rhs2])
    
    solution = np.linalg.solve(A, b)
    s345 = solution[0]
    c345 = solution[1]
    print("s345：",s345)
    print("c345：",c345)
    x345 = arctan2(s345, c345)
    print("345:",x345)
    return x345

def calc_angle34(x, y, z, x1, x2, x6, x345):
    m1 = (120*cos(x345)*cos(x6) - 100*sin(x345) + (z - 120) * cos(x2) - x * cos(x1) * sin(x2) - y * sin(x1) * sin(x2)) / 400 #c3-c34
    m2 = -(-120*sin(x345)*cos(x6) - 100*cos(x345) + x * sin(x1) - y * cos(x1) + 100) / 400 #s3-s34
    gamma = arctan2(m1, -m2)
    print("gamma:", gamma)
    x4_1 = 0
    x4_2 = 0
    if(sin(gamma) > 1e-10):
        x4_1 = 2 * arcsin(m1 / 2 / sin(gamma))
        x4_2 = 2 * np.pi - x4_1
    else:
        x4_1 = 2 * arcsin(-m2 / 2 / cos(gamma))
        x4_2 = 2 * np.pi - x4_1
    x3_1 = gamma - x4_1 / 2
    x3_2 = gamma - x4_2 / 2

    # x4_1 = arccos(1 - (m1 ** 2 + m2 ** 2) / 2)
    # x4_2 = -x4_1
    # x3_1 = arctan2(((1+cos(x4_1))*m1 + sin(x4_1)*m2)/(2*sin(x4_1)), ((1-cos(x4_1))*m1 - sin(x4_1)*m2)/(2*(1-cos(x4_1))))
    # x3_2 = arctan2(((1+cos(x4_2))*m1 + sin(x4_2)*m2)/(2*sin(x4_2)), ((1-cos(x4_2))*m1 - sin(x4_2)*m2)/(2*(1-cos(x4_2))))

    x5_1 = x345 - x3_1 - x4_1
    x5_2 = x345 - x3_2 - x4_2
    print("x3_1, x3_2:", x3_1, x3_2)
    print("x4_1, x4_2:", x4_1, x4_2)
    return x3_1, x4_1, x5_1, x3_2, x4_2, x5_2

def solve(T_target, x1, base_dir):
    x = T_target[0, 3]
    y = T_target[1, 3]
    z = T_target[2, 3]
    r = T_target[0:3, 0:3]
    print("xyz:", x, y, z)
    #先计算x2
    x2_1, x2_2, x6_1, x6_2, x6_3, x6_4 = calc_angle2and6(r, x, y, z, x1, base_dir=base_dir) #x2_1和x6_1，3对应
    print("angle2:", x2_1, x2_2)
    print("angle6:", x6_1, x6_2, x6_3, x6_4)
    x7_1 = calc_angle7(r, x1, x2_1, x6_1)
    x7_2 = calc_angle7(r, x1, x2_2, x6_2)
    x7_3 = calc_angle7(r, x1, x2_1, x6_3)
    x7_4 = calc_angle7(r, x1, x2_2, x6_4)
    print("angle7:", x7_1, x7_2, x7_3, x7_4)
    x345_1 = calc_angle345(r, x1, x2_1, x6_1, x7_1)
    x345_2 = calc_angle345(r, x1, x2_2, x6_2, x7_2)
    x345_3 = calc_angle345(r, x1, x2_1, x6_3, x7_3)
    x345_4 = calc_angle345(r, x1, x2_2, x6_4, x7_4)
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

def inv(pos, x1, base_dir):# "pos" is a list
    P = np.array([pos[0], pos[1], pos[2]]).T
    rotateM = euler_to_rotation_matrix(pos[3], pos[4], pos[5])
    T = np.vstack((np.hstack((rotateM, P[:, None])), np.array([0, 0, 0, 1]))) #得到最终的矩阵
    print(T)
    answer = solve(T, x1, base_dir)
    return answer

def read_inputs_from_file(filename):
    """从txt文件中读取输入参数"""
    with open(filename, 'r', encoding='utf-8') as file:
        lines = file.readlines()
    # 去除每行的换行符和空白字符
    lines = [line.strip() for line in lines if line.strip()]
    if len(lines) < 4:
        raise ValueError("文件中的行数不足，需要至少4行输入")
    # 第1行：末端执行器XYZ位置
    end_xyz = np.array([float(x) for x in lines[0].split()])
    # 第2行：末端执行器RPY角度
    end_rpy = np.array([float(x) for x in lines[1].split()])
    # 将角度转换为弧度
    end_rpy = end_rpy * np.pi / 180
    # 第3行：方向
    base_dir = lines[2]
    # 第4行：初始关节角度x1
    x1_input = lines[3]
        
    return end_xyz, end_rpy, base_dir, x1_input

if __name__ == '__main__':
    # 用户交互输入
    # end_xyz = input("End-effector XYZ position (in meters): ")
    # end_xyz = np.array([float(x) for x in end_xyz.split()])
    
    # end_rpy = input("End-effector RPY angles (in degrees): ")
    # end_rpy = np.array([float(x) for x in end_rpy.split()])
    # # 将角度转换为弧度
    # end_rpy = end_rpy * np.pi / 180
    # base_dir = input("Direction (L/R): ")
    # x1_input = input("Initial joint angle x1 (in degrees): ")

    # 因为太烦了，搞成了从文件读取输入
    filename = "input.txt"  # 你的txt文件名
    end_xyz, end_rpy, base_dir, x1_input = read_inputs_from_file(filename)
    x1 = float(x1_input) * np.pi / 180  # 转换为弧度
    
    # 组合位置和姿态信息
    pos = [end_xyz[0]*1000, end_xyz[1]*1000, end_xyz[2]*1000, end_rpy[0], end_rpy[1], end_rpy[2]]
    
    # 计算逆运动学解
    answer = inv(pos, x1, base_dir)
    
    # 参考值（用于比较）
    q_m2_0 = np.array([0, 1.0141970087846741, 1.1675742481274056, 1.54079182497142, -0.4332265804909674, -2.1273956448051194, 3.141592653589793])
    
    print("==================")
    print("Calculated joint angles solutions (in degrees):")
    
    for i, ans in enumerate(answer):
        # 控制范围在[-pi, pi]之间
        for j in range(len(ans)):
            if(ans[j] > np.pi): 
                ans[j] -= np.pi * 2
            if(ans[j] < -np.pi): 
                ans[j] += np.pi * 2
        
        # 符号调整
        # ans[1] = -ans[1]
        # ans[2] = -ans[2]
        # ans[3] = -ans[3]
        # ans[5] = -ans[5]
        # ans[6] = -ans[6]
        
        # 计算与参考值的差异
        sum_diff = 0
        for j in range(len(ans)):
            sum_diff = sum_diff + abs(q_m2_0[j] - ans[j])
        
        # 转换为角度制输出
        ans_degrees = [angle * 180 / np.pi for angle in ans]
        
        print(f"Solution {i+1}: {ans_degrees}")
        # print(f"Difference sum from reference: {sum_diff:.6f}")
        print()
