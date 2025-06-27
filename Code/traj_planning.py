import numpy as np
from numpy import pi

def sexticCurvePlanning(startPosition, endPosition, time, startvel=np.array([0,0,0,0,0,0,0]), endvel=np.array([0,0,0,0,0,0,0])):
    timeMatrix = np.matrix([
        [         0,           0,             0,          0,        0,   0,   1],
        [   time**6,     time**5,       time**4,    time**3,  time**2,time,   1],
        [         0,           0,             0,          0,        0,   1,   0],
        [ 6*time**5,   5*time**4,     4*time**3,  3*time**2,   2*time,   1,   0],
        [         0,           0,             0,          0,        2,   0,   0],
        [30*time**4,  20*time**3,    12*time**2,     6*time,        2,   0,   0],
        [         0,           0,             0,          6,        0,   0,   0]
    ])
    invTimeMatrix = np.linalg.inv(timeMatrix)
    kArray = []
    for i in range(len(startPosition)):
        X = np.matrix([startPosition[i], endPosition[i],startvel[i],endvel[i],0,0,0]).T
        k = np.dot(invTimeMatrix, X)
        kArray.append(k)
    return kArray

def sexticCurveExecute(kArray, time):
    timeVector = np.matrix([time**6, time**5, time**4, time**3, time**2, time, 1]).T
    jointPositions = []
    for i in range(7):
        jointPosition = np.dot(kArray[i].T, timeVector)
        jointPositions.append(jointPosition[0, 0])
    return np.array(jointPositions)

def change_base_LtoR(sim):
    # 改变父子关系，从左变成右
    # Get object handle
    l_base = sim.getObject('./L_Base')
    l_joint1 = sim.getObject('./L_Joint1')
    l_link1 = sim.getObject('./L_Link1')
    l_joint2 = sim.getObject('./L_Joint2')
    l_link2 = sim.getObject('./L_Link2')
    l_joint3 = sim.getObject('./L_Joint3')
    l_link3 = sim.getObject('./L_Link3')

    joint4 = sim.getObject('./Joint4')

    r_base = sim.getObject('./R_Base')
    r_joint1 = sim.getObject('./R_Joint1')
    r_link1 = sim.getObject('./R_Link1')
    r_joint2 = sim.getObject('./R_Joint2')
    r_link2 = sim.getObject('./R_Link2')
    r_joint3 = sim.getObject('./R_Joint3')
    r_link3 = sim.getObject('./R_Link3')

    sim.setObjectParent(r_base, -1, 1)
    sim.setObjectParent(r_joint1, r_base, 1)
    sim.setObjectParent(r_link1, r_joint1, 1)
    sim.setObjectParent(r_joint2, r_link1, 1)
    sim.setObjectParent(r_link2, r_joint2, 1)
    sim.setObjectParent(r_joint3, r_link2, 1)
    sim.setObjectParent(r_link3, r_joint3, 1)

    sim.setObjectParent(joint4, r_link3, 1)

    sim.setObjectParent(l_link3, joint4, 1)
    sim.setObjectParent(l_joint3, l_link3, 1)
    sim.setObjectParent(l_link2, l_joint3, 1)
    sim.setObjectParent(l_joint2, l_link2, 1)
    sim.setObjectParent(l_link1, l_joint2, 1)
    sim.setObjectParent(l_joint1, l_link1, 1)
    sim.setObjectParent(l_base, l_joint1, 1)
    print("This is R_Base")

def change_base_RtoL(sim):
    # 改变父子关系，从右变成左
    # Get object handle
    r_base = sim.getObject('./R_Base')
    r_joint1 = sim.getObject('./R_Joint1')
    r_link1 = sim.getObject('./R_Link1')
    r_joint2 = sim.getObject('./R_Joint2')
    r_link2 = sim.getObject('./R_Link2')
    r_joint3 = sim.getObject('./R_Joint3')
    r_link3 = sim.getObject('./R_Link3')

    joint4 = sim.getObject('./Joint4')

    l_base = sim.getObject('./L_Base')
    l_joint1 = sim.getObject('./L_Joint1')
    l_link1 = sim.getObject('./L_Link1')
    l_joint2 = sim.getObject('./L_Joint2')
    l_link2 = sim.getObject('./L_Link2')
    l_joint3 = sim.getObject('./L_Joint3')
    l_link3 = sim.getObject('./L_Link3')

    sim.setObjectParent(l_base, -1, 1)
    sim.setObjectParent(l_joint1, l_base, 1)
    sim.setObjectParent(l_link1, l_joint1, 1)
    sim.setObjectParent(l_joint2, l_link1, 1)
    sim.setObjectParent(l_link2, l_joint2, 1)
    sim.setObjectParent(l_joint3, l_link2, 1)
    sim.setObjectParent(l_link3, l_joint3, 1)

    sim.setObjectParent(joint4, l_link3, 1)

    sim.setObjectParent(r_link3, joint4, 1)
    sim.setObjectParent(r_joint3, r_link3, 1)
    sim.setObjectParent(r_link2, r_joint3, 1)
    sim.setObjectParent(r_joint2, r_link2, 1)
    sim.setObjectParent(r_link1, r_joint2, 1)
    sim.setObjectParent(r_joint1, r_link1, 1)
    sim.setObjectParent(r_base, r_joint1, 1)
    print("This is L_Base")
