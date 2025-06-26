# Before running this example, make sure you have installed the following package.
# pip install coppeliasim-zmqremoteapi-client numpy
# You can find more information about ZeroMQ remote API 
# in the file path <Coppeliasim_install_path>\programming\zmqRemoteApi
# or on https://github.com/CoppeliaRobotics/zmqRemoteApi
#
# You can get more API about coppleliasim on https://coppeliarobotics.com/helpFiles/en/apiFunctions.htm

import time
import numpy as np
from math import pi
from math import *
from numpy import arctan, arccos
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from trajplanning import change_base_LtoR, change_base_RtoL, sexticCurvePlanning, sexticCurveExecute, sexticCurvePlanningPoint, straightTrajPlaning
print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

# Run a simulation in stepping mode:
client.setStepping(True)
sim.startSimulation()

# Get object handle
joint = np.zeros(7)
joint[0] = L_joint1 = sim.getObject('./L_Joint1')
joint[1] = L_joint2 = sim.getObject('./L_Joint2')
joint[2] = L_joint3 = sim.getObject('./L_Joint3')
joint[3] = joint4 = sim.getObject('./Joint4')
joint[4] = R_joint3 = sim.getObject('./R_Joint3')
joint[5] = R_joint2 = sim.getObject('./R_Joint2')
joint[6] = R_joint1 = sim.getObject('./R_Joint1')

matrix_l = []
matrix_r = []
for i in range(16, 30):
    matrix_l.append(sim.getObjectMatrix(i, i + 1))
for i in range(16, 30):
    matrix_r.append(sim.getObjectMatrix(i + 1, i))

graph0 = sim.getObject('/Graph')
velocityStreamHandles = [0, 0, 0, 0, 0, 0, 0]
velocityStreamHandles[0] = sim.addGraphStream(graph0, 'L_joint1 velocity', 'deg/s', 0, [1, 0, 0])
velocityStreamHandles[1] = sim.addGraphStream(graph0, 'L_joint2 velocity', 'deg/s', 0, [0, 1, 0])
velocityStreamHandles[2] = sim.addGraphStream(graph0, 'L_joint3 velocity', 'deg/s', 0, [0, 0, 1])
velocityStreamHandles[3] = sim.addGraphStream(graph0, 'joint4 velocity', 'deg/s', 0, [1, 1, 0])
velocityStreamHandles[4] = sim.addGraphStream(graph0, 'R_joint1 velocity', 'deg/s', 0, [1, 0, 1])
velocityStreamHandles[5] = sim.addGraphStream(graph0, 'R_joint2 velocity', 'deg/s', 0, [0, 1, 1])
velocityStreamHandles[6] = sim.addGraphStream(graph0, 'R_joint3 velocity', 'deg/s', 0, [1, 1, 1])

# 低通滤波器函数用于减少绘制角速度曲线的毛刺
def low_pass_filter(new_value, prev_filtered_value, alpha):
    return alpha * new_value + (1 - alpha) * prev_filtered_value

# 初始化滤波器的历史值
filtered_velocities = [0 for _ in range(7)]  # 假设有7个关节
alpha = 0.05  # 滤波系数，需要根据实际情况调整

def set_joint_positions(sim, q, base):
    if (base == 'B'):
        # 以B为固定点 
        sim.setJointPosition(R_joint1, q[0])
        sim.setJointPosition(R_joint2, q[1])
        sim.setJointPosition(R_joint3, q[2])
        sim.setJointPosition(joint4,   q[3])
        sim.setJointPosition(L_joint3, q[4])
        sim.setJointPosition(L_joint2, q[5])
        sim.setJointPosition(L_joint1, q[6])
    if (base == 'A'):
        # 以A为固定点 
        sim.setJointPosition(L_joint1, q[0])
        sim.setJointPosition(L_joint2, q[1])
        sim.setJointPosition(L_joint3, q[2])
        sim.setJointPosition(joint4,   q[3])
        sim.setJointPosition(R_joint3, q[4])
        sim.setJointPosition(R_joint2, q[5])
        sim.setJointPosition(R_joint1, q[6])

# 关节角通过IK_L_base.py和IK_R_base.py计算
# q_test = np.array([0, -1.0471975511965979, -2.707988850562399, 1.539541238295402, 1.9731450413227956, 2.094395102393195, -6.123233995736766e-17]) #L_base, 600
q0 = np.array([0, 0, 0, 0, 0, 0, 0]) #q0

q1 = np.array([
    (120) / 180 * np.pi,
    0,
    -(2 * np.arcsin(150 * np.sqrt(3) / 400)) / 2,
    -(2 * np.arcsin(150 * np.sqrt(3) / 400)),
    (2 * np.arcsin(150 * np.sqrt(3) / 400)) / 2,
    0,
    -(120) / 180 * np.pi,
])

q1_1 = np.array([0.066, 0.056978523455432084, -0.12154285835185606, 0.05, -0.1715480681505778, -0.05697852345543221, -0.066])

q1_2 = q1 + np.array([0, 0.03364145, 0.00063271, 0.00126542, -0.00063271, -0.03364145, 0])

q2 = np.array([4.92659305, 0, -0.80382838, 0.69382108, -1.68080363, 1.35659225, 0])

q_m2_0 = np.array([2.0943951023931953, -6.217248937900877e-15, -0.6691134093180293, 1.4151688734507095, 0.7460554641326766, 6.156016597943509e-15, -2.0943951023931953])

q_m2_3 = q2

print(q1)
print(q2)

# 使用 sexticCurvePlanning 函数计算路径规划
k_1_1 = sexticCurvePlanning(q0, q1_1, 1)
k_1_2 = sexticCurvePlanning(q1_1, q1_2, 3)
k_1_3 = sexticCurvePlanning(q1_2, q1, 1)

k_2_0 = sexticCurvePlanning(-np.flip(q1), q_m2_0, 1)
k_2_1 = sexticCurvePlanning(q_m2_0, q_m2_3, 3)
k_2_4 = sexticCurvePlanning(q_m2_3, q2, 1)


flag_turn = 1
while (t := sim.getSimulationTime()) < 20:
    # 获取当前速度
    current_velocities = [sim.getJointVelocity(joint) for joint in [L_joint1, L_joint2, L_joint3, joint4, R_joint1, R_joint2, R_joint3]]

    # 对每个关节的速度进行低通滤波
    for i in range(7):
        filtered_velocities[i] = low_pass_filter(current_velocities[i], filtered_velocities[i], alpha)

    # 使用滤波后的速度值更新图表
    for i, vel in enumerate(filtered_velocities):
        sim.setGraphStreamValue(graph0, velocityStreamHandles[i], 180 * vel / pi)

    if t < 5:
        if flag_turn:
            change_base_LtoR(sim)
            flag_turn = 0
            
            pos_now = np.zeros(7)
            for i in range(0, 7):
                pos_now[i] = sim.getJointPosition(joint[i])
            for i in range(16, 30):
                sim.setObjectMatrix(i + 1, i, matrix_r[i - 16])
                if i % 2 == 1:      # is joint
                    sim.setJointPosition(i, -pos_now[(i - 17) // 2])
        if t < 1:
            q = sexticCurveExecute(k_1_1, t)
        elif t < 4:
            q = sexticCurveExecute(k_1_2, t-1)
        else:
            q = sexticCurveExecute(k_1_3, t-4)
        set_joint_positions(sim, q, "B")

    elif t < 6:
        q = q1
        set_joint_positions(sim, q, "B")

    elif t < 11:
        if not flag_turn:
            change_base_RtoL(sim)
            flag_turn = 1

            pos_now = np.zeros(7)
            for i in range(0, 7):
                pos_now[i] = sim.getJointPosition(joint[i])
            for i in range(16, 30):
                sim.setObjectMatrix(i + 1, i, matrix_r[i - 16])
                if i % 2 == 1:      # is joint
                    sim.setJointPosition(i, -pos_now[(i - 17) // 2])
        if t < 7:
            startPosition = np.array([0, -600, 0, pi, 0, -pi/2])
            endPosition = np.array([0, -600, 20, pi, 0, -pi/2])
            startX1 = 2/3*pi
            endX1 = 2/3*pi
            tim = 1
            q = straightTrajPlaning(startPosition, startX1, endPosition, endX1, t-6, tim)

        elif t < 10:
            q = sexticCurveExecute(k_2_1, t-7)

        else: 
            q = sexticCurveExecute(k_2_4, t-10)
        
        set_joint_positions(sim, q, "A")

    message = f'Simulation time: {t:.2f} s'
    print(message)
    sim.addLog(sim.verbosity_scriptinfos, message)
    client.step()  # triggers next simulation step
    # time.sleep(0.01)
time.sleep(1)


# Change the parent
# sim.setObjectParent(int objectHandle, int parentObjectHandle, bool keepInPlace)

# Stop simulation
sim.stopSimulation()

# Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

print('Program ended')


