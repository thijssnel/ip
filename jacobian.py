import numpy as np
import sympy as sp
"""
this class generates the jacobian matrix for a given industrial robot
provide the class with a dictionary of joints in the following manner
{
rank(int) : 'rotation plane (xy, xz, yz, ee), orientation(x,y,z), length (float)'
}
rank(int) = the order of the joint in the kinematic chain from the base starting from 0

rotation plane(xy, xz, yz, ee) = the plane in which the joint rotates or translates, use ee for end effector

orientation(x,y,z) = the axis of rotation or translation of the joint, 

length(float) = the length between current joint and the previous joint
"""
class industrial_robot:
    def __init__(self, joints: dict):
        self.joints = joints
        self.q = sp.symbols(f'q:{len(joints)}')
        self.number_of_joints = len(joints)
        self.jacobian = self.jacobian_generator(self.q) 

    def transformation_matrix(self, key, joint: str):
        string = joint.split(',')
        rotation_plane = string[0]
        orientation = string[1]
        length = float(string[2])

        idx = key
        q = self.q[idx]
        s, c = sp.sin(q), sp.cos(q)

        if rotation_plane == 'ee':
            if orientation == 'z':
                return sp.Matrix([[1, 0, 0, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return sp.Matrix([[1, 0, 0, 0],
                                 [0, 1, 0, length],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return sp.Matrix([[1, 0, 0, length],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])

        elif rotation_plane == 'xy':
            if orientation == 'z':
                return sp.Matrix([[c, -s, 0, 0],
                                 [s, c, 0, 0],
                                 [0, 0, 1, length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return sp.Matrix([[c, -s, 0, 0],
                                 [s, c, 0, length],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return sp.Matrix([[c, -s, 0, length],
                                 [s, c, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
            
        elif rotation_plane == 'xz':
            if orientation == 'z':
                return sp.Matrix([[c, 0, s, 0],
                                 [0, 1, 0, 0],
                                 [-s, 0, c, length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return sp.Matrix([[c, 0, s, 0],
                                 [0, 1, 0, length],
                                 [-s, 0, c, 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return sp.Matrix([[c, 0, s, length],
                                 [0, 1, 0, 0],
                                 [-s, 0, c, 0],
                                 [0, 0, 0, 1]])
            
        elif rotation_plane == 'yz':
            if orientation == 'z':
                return sp.Matrix([[1, 0, 0, 0],
                                 [0, c, -s, 0],
                                 [0, s, c, length],
                                 [0, 0, 0, 1]])
            elif orientation == 'y':
                return sp.Matrix([[1, 0, 0, 0],
                                 [0, c, -s, length],
                                 [0, s, c, 0],
                                 [0, 0, 0, 1]])
            elif orientation == 'x':
                return sp.Matrix([[1, 0, 0, length],
                                 [0, c, -s, 0],
                                 [0, s, c, 0],
                                 [0, 0, 0, 1]])
            
    def total_transformation_matrix(self):
        T = np.eye(4)
        for key, joint in self.joints.items():
            T = T * self.transformation_matrix(key, joint)
        return T
    
    def jacobian_generator(self, q: list):
        T = self.total_transformation_matrix()

        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        R = T[:3,:3]

        J = sp.zeros(6, self.number_of_joints)
        for i in range(self.number_of_joints):
            J[0, i] = sp.diff(x, self.q[i])
            J[1, i] = sp.diff(y, self.q[i])
            J[2, i] = sp.diff(z, self.q[i])

                    # Angular velocity part (from rotation derivative)
            dR_dqi = R.diff(self.q[i])
            w_i = sp.Matrix([
                dR_dqi[2,1] - dR_dqi[1,2],
                dR_dqi[0,2] - dR_dqi[2,0],
                dR_dqi[1,0] - dR_dqi[0,1]
            ]) / 2
            J[3:, i] = w_i
        return sp.simplify(J)
    

        
    

