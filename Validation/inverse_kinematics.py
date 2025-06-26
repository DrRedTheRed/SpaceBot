from forward_kinematics import FK
import numpy as np
from scipy.spatial.transform import Rotation as R

# numerical jacobian
def jacobian(theta, direction, eps=1e-6):
    # calculate current end-effector position and orientation
    J = np.zeros((6, 7))
    if direction == 'L':
        T0 = FK(theta, 'L')[2]
    elif direction == 'R':
        T0 = FK(theta, 'R')[2]
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    # make a minor change to each joint angle
    for i in range(7):
        d_theta = theta.copy()
        d_theta[i] += eps
        if direction == 'L':
            Ti = FK(d_theta, 'L')[2]
        elif direction == 'R':
            Ti = FK(d_theta, 'R')[2]
        # end-effector pos and ori after the change
        pi = Ti[:3, 3]
        Ri = Ti[:3, :3]
        # calculate derivative of position
        dp = (pi - p0) / eps
        # calculate derivative of orientation
        rpy_0 = R.from_matrix(R0).as_euler('xyz', degrees=True)
        rpy_i = R.from_matrix(Ri).as_euler('xyz', degrees=True)
        dr = (rpy_i - rpy_0) / eps
        J[:, i] = np.concatenate((dp, dr))
    return J

def xyz_and_rpy_to_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', rpy, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T

def euler_error(R_target, R_current):
    rpy_target = R.from_matrix(R_target).as_euler('xyz', degrees=True)
    rpy_current = R.from_matrix(R_current).as_euler('xyz', degrees=True)
    error = rpy_target - rpy_current
    # normalize the error to be within -pi to pi
    error = (error + 180) % 360 - 180
    return error

# inverse kinematics using numerical jacobian
def IK(end_xyz, end_rpy, direction, max_iter=1000, tol=1e-6):
    theta = np.zeros(7) # initial guess for joint angles
    end_T = xyz_and_rpy_to_T(end_xyz, end_rpy)
    for i in range(max_iter):
        if direction == 'L':
            T = FK(theta, 'L')[2]
        elif direction == 'R':
            T = FK(theta, 'R')[2]
        pos_err = end_T[:3, 3] - T[:3, 3] # position error matrix
        rot_err = euler_error(end_T[:3, :3], T[:3, :3]) # orientation error matrix

        delta_x = np.concatenate((pos_err, rot_err)) # error matrix
        if np.linalg.norm(delta_x) < tol:
            return theta
        J = jacobian(theta, direction)
        d_theta = np.linalg.pinv(J) @ delta_x # calculate d_theta using pseudoinverse
        theta += d_theta
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
    print("Error: IK did not converge within the maximum iterations.")
    return theta

if __name__ == '__main__':
    end_xyz = input("End-effector XYZ position (in meters): ")
    end_xyz = np.array([float(x) for x in end_xyz.split()])
    end_rpy = input("End-effector RPY angles (in degrees): ")
    end_rpy = np.array([float(x) for x in end_rpy.split()])
    direction = input("Direction (L/R): ")
    theta = IK(end_xyz, end_rpy, direction)
    print("Calculated joint angles (in degrees):", theta * 180 / np.pi)