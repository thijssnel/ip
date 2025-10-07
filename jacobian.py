import numpy as np
import pandas as pd
"""
this class generates the jacobian matrix for a given industrial robot
provide the class with a dictionary of joints in the following manner
{
rank(int) : 'type of joint(revolute, prismatic), orientation(x,y,z), length (float)'
}
rank(int) = the order of the joint in the kinematic chain from the base starting from 0

orientation(x,y,z) = the axis of rotation or translation of the joint, 

length(float) = the length of the link following the joint
"""

class jacobian_generator:
    def __init__(self, joints: dict):
        self.joints = joints
        self.number_of_joints = len(joints)


    def orientation(self, joint: str):
        for i in range(self.number_of_joints):
            if self.joints[i].split(',')[1] == joint:
                return i
            

        

        




