import numpy as np

class jacobian_generator:
    def __init__(self, joints: dict):
        self.joints = joints
        self.q = [0] * 6  # q[0] for joint 1, q[1] for joint 2, etc.
        self.number_of_joints = len(joints)

    def transformation_matrix(self, key, joint: str):
        string = joint.split(',')
        rotation_plane = string[0]
        orientation = string[1]
        length = float(string[2])

        idx = key - 1  # key 1 -> q[0], key 2 -> q[1], etc.
        q = self.q[idx]

        if rotation_plane == 'xy':
            if orientation == 'z':
                return np.array([[np.cos(q), -np.sin(q), 0, 0],
                                 [np.sin(q), np.cos(q), 0, 0],
                                 [0, 0, 1, length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return np.array([[np.cos(q), -np.sin(q), 0, 0],
                                 [np.sin(q), np.cos(q), 0, length],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return np.array([[np.cos(q), -np.sin(q), 0, length],
                                 [np.sin(q), np.cos(q), 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
        elif rotation_plane == 'xz':
            if orientation == 'z':
                return np.array([[np.cos(q), 0, np.sin(q), 0],
                                 [0, 1, 0, 0],
                                 [-np.sin(q), 0, np.cos(q), length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return np.array([[np.cos(q), 0, np.sin(q), 0],
                                 [0, 1, 0, length],
                                 [-np.sin(q), 0, np.cos(q), 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return np.array([[np.cos(q), 0, np.sin(q), length],
                                 [0, 1, 0, 0],
                                 [-np.sin(q), 0, np.cos(q), 0],
                                 [0, 0, 0, 1]])
        elif rotation_plane == 'yz':
            if orientation == 'z':
                return np.array([[1, 0, 0, 0],
                                 [0, np.cos(q), -np.sin(q), 0],
                                 [0, np.sin(q), np.cos(q), length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return np.array([[1, 0, 0, 0],
                                 [0, np.cos(q), -np.sin(q), length],
                                 [0, np.sin(q), np.cos(q), 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return np.array([[1, 0, 0, length],
                                 [0, np.cos(q), -np.sin(q), 0],
                                 [0, np.sin(q), np.cos(q), 0],
                                 [0, 0, 0, 1]])
