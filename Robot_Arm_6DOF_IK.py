import numpy as np
from scipy.spatial.transform import Rotation

class ik_6dof:
    def __init__(self, dh_table):
        self.dh_table = np.array(dh_table, dtype=float)

    # Create HTM for each transformation using dh parameters

    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),               d],
            [0,              0,                           0,                           1]
        ])

    # Forward kinematics

    def fk(self, theta_list):
        T = np.identity(4)
        Ts = []
        for i, row in enumerate(self.dh_table):
            theta = theta_list[i]
            d, a, alpha = row[1:]
            T = T @ self.dh_matrix(theta, d, a, alpha)
            Ts.append(T.copy())
        return Ts

    # Find error between current and desired orientation
    
    def orientation_error(self, R_current, R_target):
        R_err = R_current.T @ R_target
        rotvec = Rotation.from_matrix(R_err).as_rotvec()
        return R_current @ rotvec

    # Create a jacobian matrix

    def compute_jacobian(self, Ts):
        J = np.zeros((6,6))
        O_n = Ts[-1][:3,3]

        O_prev = np.zeros(3)
        Z_prev = np.array([0,0,1])

        for i in range(6):
            if i > 0:
                O_prev = Ts[i-1][:3,3]
                Z_prev = Ts[i-1][:3,2]

            J[:3,i] = np.cross(Z_prev, O_n - O_prev)
            J[3:,i] = Z_prev

        return J

    # Check if the target can be reached

    def is_reachable(self, target_pos):
        total_length = sum(abs(row[2]) for row in self.dh_table)
        return np.linalg.norm(target_pos) <= total_length

    # Numerically solve the inverse kinematics

    def solve(self, target_pos, target_quat):
        if not self.is_reachable(target_pos):
            raise ValueError("Target is out of reach")
        
        theta = np.array([0, np.pi/6, np.pi/3, 0, 0, 0])
        tol = 1e-4
        max_iters = 300
        damping = 0.01
        alpha = 0.5  

        R_target = Rotation.from_quat(target_quat).as_matrix()

        for _ in range(max_iters):
            Ts = self.fk(theta)
            T_end = Ts[-1]
            pos_current = T_end[:3,3]
            R_current = T_end[:3,:3]

            e_pos = target_pos - pos_current
            e_ori = self.orientation_error(R_current, R_target)
            e = np.concatenate((e_pos, e_ori))

            if np.linalg.norm(e) < tol:
                break

            J = self.compute_jacobian(Ts)

            JT = J.T
            delta_theta = JT @ np.linalg.inv(J @ JT + damping**2*np.eye(6)) @ e

            theta += alpha * delta_theta  

        return theta, Ts, np.linalg.norm(e)

