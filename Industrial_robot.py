          
import numpy as np
import sympy as sp
# import time
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class IndustrialRobot:
    def __init__(self, joints:dict):
        # joints: dict mapping index-> specification string
        self.joint_dict = joints
        self.number_of_joints = len(joints)

        # create symbolic joint variables q1..qN (inclusive)
        self.joint_symbols = []
        for i in range(1, self.number_of_joints + 1):
            s = sp.symbols(f"q{i}")
            setattr(self, f"q{i}_sym", s)
            self.joint_symbols.append(s)
            # initialise numeric value attributes
            setattr(self, f"q{i}_val", 0.0)
        self.q_rest = np.zeros(self.number_of_joints)

        # containers for lambdified functions
        self.matrix_funcs = []   # functions that return 4x4 matrices
        self.J_funcs = []        # functions that return Jacobians (6 x N)
        # build symbolic transforms and jacobians and lambdify them
        self.total_transformation_matrix()


    def transformation_matrix(self, key, joint: str):
        # parse joint specification string
        string = joint.split(',')
        rotation_plane = string[0].strip()
        orientation = string[1].strip()
        length = float(string[2])

        # update max length
        setattr(self,'max_length',self.max_length+length)

        # note: use the symbol corresponding to joint index `key+1`
        q = getattr(self, f'q{key+1}_sym')
        s, c = sp.sin(q), sp.cos(q)

        T = None
        # Transformation matrix selection 
        if rotation_plane == 'ee':
            if orientation == 'z':
                T = sp.Matrix([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, length],
                               [0, 0, 0, 1]])
            elif orientation == 'y':
                T = sp.Matrix([[1, 0, 0, 0],
                               [0, 1, 0, length],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])
            elif orientation == 'x':
                T = sp.Matrix([[1, 0, 0, length],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]])

        elif rotation_plane == 'xy':
            if orientation == 'z':
                T = sp.Matrix([[c, -s, 0, 0],
                               [s,  c, 0, 0],
                               [0,  0, 1, length],
                               [0,  0, 0, 1]])
            elif orientation == 'y':
                T = sp.Matrix([[c, -s, 0, 0],
                               [s,  c, 0, length],
                               [0,  0, 1, 0],
                               [0,  0, 0, 1]])
            elif orientation == 'x':
                T = sp.Matrix([[c, -s, 0, length],
                               [s,  c, 0, 0],
                               [0,  0, 1, 0],
                               [0,  0, 0, 1]])

        elif rotation_plane == 'xz':
            if orientation == 'z':
                T = sp.Matrix([[c, 0, s, 0],
                               [0, 1, 0, 0],
                               [-s, 0, c, length],
                               [0, 0, 0, 1]])
            elif orientation == 'y':
                T = sp.Matrix([[c, 0, s, 0],
                               [0, 1, 0, length],
                               [-s, 0, c, 0],
                               [0, 0, 0, 1]])
            elif orientation == 'x':
                T = sp.Matrix([[c, 0, s, length],
                               [0, 1, 0, 0],
                               [-s, 0, c, 0],
                               [0, 0, 0, 1]])

        elif rotation_plane == 'yz':
            if orientation == 'z':
                T = sp.Matrix([[1, 0, 0, 0],
                               [0, c, -s, 0],
                               [0, s,  c, length],
                               [0, 0, 0, 1]])
            elif orientation == 'y':
                T = sp.Matrix([[1, 0, 0, 0],
                               [0, c, -s, length],
                               [0, s,  c, 0],
                               [0, 0, 0, 1]])
            elif orientation == 'x':
                T = sp.Matrix([[1, 0, 0, length],
                               [0, c, -s, 0],
                               [0, s,  c, 0],
                               [0, 0, 0, 1]])
        return T

    def total_transformation_matrix(self):
        #makes frame for total transformation matrix
        T = sp.eye(4)

        #updates throughout the process
        self.max_length = 0

        # Build transforms and Jacobians symbolically for each link, lambdify them
        for key, joint in self.joint_dict.items():
            # multiply transform for joint `key`
            T = T * self.transformation_matrix(key, joint)
            setattr(self, f"matrix_b{key + 1}", T)

            # position and rotation submatrix
            x, y, z = T[0,3], T[1,3], T[2,3]
            R = T[:3,:3]

            # Jacobian: 6 x N (N = number_of_joints)
            J = sp.zeros(6, self.number_of_joints)
            # compute jacobian for cartisian position
            for i in range(self.number_of_joints):
                sym = getattr(self, f"q{i+1}_sym")
                J[0,i] = sp.diff(x, sym)
                J[1,i] = sp.diff(y, sym)
                J[2,i] = sp.diff(z, sym)

                # rotation matrix derivative
                dR_dq = R.diff(sym)
                # angular velocity approximation (from rotation matrix derivative)
                J[3:,i] = sp.Matrix([dR_dq[2,1] - dR_dq[1,2],
                                     dR_dq[0,2] - dR_dq[2,0],
                                     dR_dq[1,0] - dR_dq[0,1]])/2
            
            # store symbolic jacobian for every joint
            setattr(self, f"jacobian_b{key+1}", J)

        # lambdify all previously stored symbolic matrices & jacobians
        for i in range(1, self.number_of_joints+1):
            M_sym = getattr(self, f"matrix_b{i}")
            J_sym = getattr(self, f"jacobian_b{i}")
            # lambdify: functions accept len(self.joint_symbols) positional args
            self.matrix_funcs.append(sp.lambdify(self.joint_symbols, M_sym, 'numpy'))
            self.J_funcs.append(sp.lambdify(self.joint_symbols, J_sym, 'numpy'))


    def get_joint_dict(self):
        # mapping from sympy symbols to numeric values (kept for compatibility)
        return {self.joint_symbols[i]: getattr(self, f"q{i+1}_val") for i in range(self.number_of_joints)}

    def get_xyz(self,matrix):
        return matrix[0,3], matrix[1,3], matrix[2,3]

    def get_joint_position_car(self):
        # get all the angle values for every joint
        q_vals = [getattr(self, f"q{i+1}_val") for i in range(self.number_of_joints)]
        q_args = tuple(q_vals)  # for passing into lambdified functions

        # Prepare storage for the X,Y,Z,phi,theta,psi coordinates of each joint
        self.q_positions = np.zeros((self.number_of_joints, 6), dtype=float)

        for i in range(self.number_of_joints):
            # Call the matrix function for joint i
            M = np.array(self.matrix_funcs[i](*q_args), dtype=float)

            # extract position entries
            X, Y, Z = M[0,3], M[1,3], M[2,3]

            # extract rotation matrix entries
            r11, r12, r13 = M[0,0], M[0,1], M[0,2]
            r21, r22, r23 = M[1,0], M[1,1], M[1,2]
            r31, r32, r33 = M[2,0], M[2,1], M[2,2]

            # Calculate euler angles from rotation matrix
            theta = np.arctan2(-r31, np.sqrt(r11*r11 + r21*r21))   # pitch
            psi   = np.arctan2(r21, r11)                           # yaw
            phi   = np.arctan2(r32, r33)                           # roll

            # store in array and object attributes    
            self.q_positions[i] = np.array([X, Y, Z, theta, psi, phi], dtype=float)
            setattr(self, f"q{i+1}_car", self.q_positions[i].copy())

    def update_angles(self, angles, rad=True):
        # fill q1..qN numeric values; angles length must match number_of_joints
        if rad:
            for i in range(self.number_of_joints
                           ):
                setattr(self,f"q{i+1}_val", float(angles[i]))
        else:
            for i in range(self.number_of_joints):
                setattr(self, f"q{i+1}_val", float(np.deg2rad(angles[i])))


    def plot_robot(self):
        # obtain and update joint positions
        self.get_joint_position_car()
        x = self.q_positions[:,0]
        y = self.q_positions[:,1]
        z = self.q_positions[:,2]

        # Make figure in a 3d space if objects are detected plot them as well
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        if hasattr(self,'detected_objects'):
            [ax.plot([p1[0],p2[0]], [p1[1],p2[1]],[p1[2],p2[2]],color = 'red',linewidth=8,label='obstacle') for p1, p2 in self.detected_objects]

        ax.plot(x, y, z, '-o', color = 'blue', linewidth=4, markersize=6,label='robot')
        ax.bar3d((x[0]-0.5)*self.max_length/3,(y[0]-0.5)*self.max_length/3,(z[0]),(1)*self.max_length/3,(1)*self.max_length/3,-z[0],'blue', alpha=0.7)
        ax.set_title("Industrial Robot Configuration")
        ax.set_xlabel('X-axis'); ax.set_ylabel('Y-axis'); ax.set_zlabel('Z-axis')
        ax.grid(True)
        ax.set_xlim(-self.max_length, self.max_length); ax.set_ylim(-self.max_length, self.max_length); ax.set_zlim(0, self.max_length)
        plt.show()
    
    import numpy as np

    def update_objects(self, objects: list,correction=[0,0,0],direction='x'):
        self.detected_objects = [make_object_line(obj[0], obj[1], obj[2],forward_axis=direction) for obj in objects]
        self.detected_objects = [(p1+correction, p2+correction) for p1, p2 in self.detected_objects]
        self.objects_updated = True
        
    
    def get_to_destination(self, destination: tuple, dt: float, attractive_coff=1,posture_coff=0.1,ground_coff=0.0000003, repelent_joint=0.00001,repelent_object=0.0001,range_repelent=0,ground_range = 0.05,z_ground=0,lam=0.001, time=10, verbose = False, reset = False):
        #standard range_repelent if not specified
        if range_repelent == 0:
            range_repelent=self.max_length/5

        #setup variables
        self.destination = np.array(destination, dtype=float)
        steps = int(time / dt)
        arrival_time = time
        final_error = None
        collision_cost = 0.0
        smoothness_cost = 0.0

        #checks for path storage
        if not hasattr(self, 'path'):
            self.path = {}
        self.path_rad ={}
        if reset:
            for i in range(6):
                setattr(self, f"q{i+1}_val", 0.0)

        # small epsilon to avoid divide-by-zero
        eps = 1e-12
        

        for step in range(steps):
            # get current joint positions and save to path
            self.get_joint_position_car()
            self.path[step] = {j: getattr(self, f"q{j+1}_car") for j in range(self.number_of_joints)}
            self.path_rad[step] = [getattr(self,f'q{i+1}_val') for i in range(6)]

            # evaluate current forward kinematics (vectorized per-joint)
            q_vals = [getattr(self, f"q{i+1}_val") for i in range(self.number_of_joints)]
            q_args = tuple(q_vals)

            # positions array (N x 3)
            positions = self.q_positions[:, :3].copy()

            # iterate joints from end-effector downwards (only needed if you want hierarchical updates)
            # here we compute for the end effector (last joint) and then apply repulsion for intermediate ones
            for j in range(self.number_of_joints, 0, -1):
                idx = j - 1
                p = positions[idx]    # position of joint j (x,y,z)
                # SPEEDS is a 3-vector (we only use translation for collision/attraction)
                speeds = np.zeros(3, dtype=float)

                # ---- GROUND REPULSION ----
                g_force = ground_repulsion(
                    p,
                    z_ground=z_ground,
                    influence=ground_range,
                    k_ground=ground_coff
                )

                # Penalize ground collision
                if p[2] < z_ground:
                    collision_cost += (1.0 / (abs(p[2] - z_ground) + eps))**2 * dt

                speeds += g_force

                # ---- OBJECT REPEL ----
                if idx > 0:
                    #begin end of link segment
                    pA1 = positions[idx]
                    pA2 = positions[idx-1]

                    # checks for objects to repel from for every joint except the base
                    if hasattr(self, 'detected_objects') and idx == 6:
                        p1_p2_objects = [
                            segment_distance(pA1, pA2, i[0], i[1])
                            for i in self.detected_objects
                        ]

                        p1_p2_objects = [
                            i for i in p1_p2_objects
                            if i[0] < (self.max_length * 0.2)
                        ]

                    
                        # calculate repulsive forces from objects
                        for dist, cA, cB in p1_p2_objects:
                            dist = np.clip(dist, eps, range_repelent)

                            direction = (cA - cB) / dist

                            force_mag = (
                                repelent_object
                                * (1/dist - 1/range_repelent)
                                * (1/dist**2)
                            )

                            speeds += force_mag * direction


                    # ---- LINK REPEL ----
                    for k in range(idx-2):
                        pB1 = positions[k]
                        pB2 = positions[k+1]

                        dist, cA, cB = segment_distance(pA1, pA2, pB1, pB2)

                        if dist < range_repelent:
                            # collision penalty
                            collision_cost += (1.0 / (dist + eps))**2 * dt

                            # direction away from the closest point on the other link
                            direction = cA - cB
                            norm = np.linalg.norm(direction)

                            direction /= norm

                            # repulsive velocity (scale with soft boundary)
                            strength = repelent_joint * (1.0/dist - 1.0/range_repelent)*1/dist**2

                            speeds += strength * direction

                # ---- ATTRACTION TO DESTINATION ----
                #end-effector only
                if j == self.number_of_joints:
                    speeds += attractive_coff * (self.destination[:3] - p)


                # If speeds is effectively zero, skip jacobian invert
                if np.linalg.norm(speeds) < eps:
                    continue

                # Weight matrix for damped least squares for biased joint updates
                W = np.diag([5, 4, 3, 2, 1, 0.5, 0.2])

                # Evaluate Jacobian for link j (6 x N)
                J_full = np.array(self.J_funcs[idx](*q_args), dtype=float)   # shape (6, N)
                Jv = J_full[:3, :]   # translational part (3 x N)

                # pseudoinverse mapping from cartesian velocity -> joint velocities
                JT = Jv.T
                A = JT @ Jv + lam * W
                dq_task = np.linalg.solve(A, JT @ speeds)     

                # Null-space projector
                J_pinv = np.linalg.solve(A, JT)
                N = np.eye(self.number_of_joints) - J_pinv @ Jv

                # Posture bias (pull toward rest pose)
                q_vals = np.array(q_vals)
                dq_posture = -posture_coff * (q_vals - self.q_rest)
                dq = dq_task + N @ dq_posture
  
                # We only apply updates to joints 0..idx-1 (joints upstream of current link)
                # Project dq to affect only the first idx joints:
                smoothness_cost += np.sum(dq**2) * dt
                dq_to_apply = dq.copy()
                # zeros for joints beyond idx-1
                dq_to_apply[idx:] = 0.0

                # update numeric joint values
                for k in range(self.number_of_joints):
                    val = getattr(self, f"q{k+1}_val")
                    setattr(self, f"q{k+1}_val", val + dq_to_apply[k] * dt)


            # check endpoint close to destination
            if np.allclose(self.q_positions[-1,:3], self.destination[:3], atol=1e-3):
                print(f"Destination reached in {step*dt} seconds.")
                break
        arrival_time = step * dt
        final_error = np.linalg.norm(self.q_positions[-1,:3] - self.destination[:3])

        score = (1*arrival_time + 
                1000*final_error + 
                100*collision_cost + 
                0.1*smoothness_cost)
        if verbose:
            print('arrival_time:', arrival_time)
            print('final_error:', final_error)
            print('collision_cost:', collision_cost)
            print('smoothness_cost:', smoothness_cost)
            print('total score:', score)
        return score, {
        "arrival_time": arrival_time,
        "final_error": final_error,
        "collision_cost": collision_cost,
        "smoothness_cost": smoothness_cost
    }


################### functions #####################
def get_distance(a,b):
    return np.linalg.norm(np.array(a[:3]) - np.array(b[:3]))



def segment_distance(p1, p2, p3, p4, eps=1e-12):
    """
    Returns (dist, closest_point_on_seg1, closest_point_on_seg2, s, t)
    where s,t are parameters in [0,1].
    """
    u = p2 - p1
    v = p4 - p3
    w = p1 - p3
    a = np.dot(u,u)
    b = np.dot(u,v)
    c = np.dot(v,v)
    d = np.dot(u,w)
    e = np.dot(v,w)
    D = a*c - b*b
    s = 0.0
    t = 0.0

    if D < eps:
        # nearly parallel
        s = 0.0
        t = (b>c and d/b) or (e/c)
    else:
        s = (b*e - c*d) / D
        t = (a*e - b*d) / D

    s = np.clip(s, 0.0, 1.0)
    t = np.clip(t, 0.0, 1.0)

    c1 = p1 + s * u
    c2 = p3 + t * v
    dist = np.linalg.norm(c1 - c2)
    return dist, c1, c2

import numpy as np

import numpy as np

def make_object_line(
    distance,
    offset_y,
    offset_z,
    length=1.0,
    forward_axis='x'
):
    ay = np.deg2rad(offset_y)
    az = np.deg2rad(offset_z)

    ty = np.tan(ay)
    tz = np.tan(az)

    denom = np.sqrt(1.0 + tz*tz + ty*ty)

    # Camera-frame direction (Y forward)
    ux =  1 / denom
    uy = ty / denom
    uz = tz / denom

    dir_cam = np.array([ux, uy, uz])

    # Proper axis rotation (not swapping!)
    if forward_axis == 'x':
        dir_world = dir_cam

    elif forward_axis == 'y':
        dir_world = np.array([dir_cam[1], -dir_cam[0], dir_cam[2]])

    elif forward_axis == 'z':
        dir_world = np.array([dir_cam[0], dir_cam[2], -dir_cam[1]])

    else:
        raise ValueError("forward_axis must be 'x', 'y', or 'z'")

    # Start point
    p0 = dir_world * distance

    # End point
    p1 = p0 + dir_world * length

    return p0, p1

def ground_repulsion(
    p,
    z_ground=0.0,
    influence=0.1,
    k_ground=0.002,
    eps=1e-6
):
    """
    Repulsive velocity from a ground plane z = z_ground
    """
    dist = p[2] - z_ground

    # Only act if within influence distance
    if dist >= influence:
        return np.zeros(3)

    # Clamp distance to avoid singularity
    d = np.clip(dist, eps, influence)

    # Upward normal
    n = np.array([0.0, 0.0, 1.0])

    force_mag = k_ground * (1.0 / d - 1.0 / influence) * (1.0 / d**2)
    return force_mag * n

