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
from traj_planning import change_base_LtoR, change_base_RtoL, sexticCurvePlanning, sexticCurveExecute
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

# 低通滤波器函数用于削掉尖刺
def threshold_filter(new_value, threshold, prev_value):
    if abs(new_value - prev_value) > threshold:
        # 异常，丢弃
        return 0
    else:
        return new_value

# 初始化滤波器的历史值
filtered_velocities = [0 for _ in range(7)]  # 假设有7个关节
threshold_dv = 0.5  # 理想低通滤波阈值，需要根据实际情况调整

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

# 关节角通过IK_numerical.py计算。你需要首先基本知道q0，q1，q2的大概值，然后再迭代出好值。

# 由于仿真坐标系和实际坐标系不同，关节角需要转换。具体而言：L_base的关节角2,3,4,6,7需要取反，R_base的关节角2,4,5,6,7需要取反。
# 以下是原来的参数

# 以下是修改后的参数

q0 = np.array([0, 0, 0, 0, 0, 0, 0]) # 起始位置

q1_1 = np.array([0.06671612,0.06671593,0.02499931,0.05000519,-0.02500588,-0.06671593,-0.06671612]) # 抬起点A

q1_2 = np.array([2.09343291, 0.0333106, -0.68832498, -1.41516858, 0.72684361, -0.0333106, -2.09343291]) # 落点上方B

q1 = np.array([2.09447475, 0.01664359, -0.71656178, -1.41390317, 0.69734138, -0.0166436, -2.09447475]) # 以右基座为基准的目标位置C（第一次挪动）

q_m2_0 = np.array([2.09439510, 0, -0.66911340, 1.41516887, 0.74605546, 0, -2.09439510]) # 抬起点D

q_m2_3 = np.array([2*np.pi-1.4103532, 0.00371712, -0.80441153, 0.71944925, -1.67872323, 1.41031052, 0.02326346]) # 落点上方E

q2 = np.array([4.92659305, 0, -0.80382838, 0.69382108, -1.68080363, 1.35659225, 0]) # 以左基座为基准的目标位置F（第二次挪动）

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
while (t := sim.getSimulationTime()) < 15:
    # 获取当前速度
    current_velocities = [sim.getJointVelocity(joint) for joint in [L_joint1, L_joint2, L_joint3, joint4, R_joint1, R_joint2, R_joint3]]

    # 对每个关节的速度进行低通滤波
    for i in range(7):
        filtered_velocities[i] = threshold_filter(current_velocities[i], threshold_dv, filtered_velocities[i])
        # filtered_velocities[i] = current_velocities[i]  # 你也可以直接使用当前速度值，取消滤波试试看。你会看到一个爆大的、由基座转换产生的尖刺

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
            q = sexticCurveExecute(k_2_0, t-6)

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


