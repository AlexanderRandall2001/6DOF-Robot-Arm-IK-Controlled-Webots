import numpy as np
from scipy.spatial.transform import Rotation

class fk_6dof:
    def __init__(self, dh_table):
        self.dh_table = dh_table

    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def fk(self):
        T = np.identity(4)
        joints = []

        for row in self.dh_table:
            theta, d, a, alpha = row
            Ti = self.dh_matrix(theta, d, a, alpha)
            T = T @ Ti
            joints.append(T[0:3, 3].copy())
        
        quaternion = Rotation.from_matrix(T[0:3, 0:3]).as_quat()
        return joints, quaternion

