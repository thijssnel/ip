import numpy as np
import sympy as sp
import time
from matplotlib.animation import FuncAnimation, PillowWriter


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class industrial_robot:
    def __init__(self, joints: dict):
        start_total = time.perf_counter()
        print("Initializing industrial_robot...")

        t0 = time.perf_counter()
        self.joints = joints
        print(f"[init] Loaded joint definitions in {time.perf_counter() - t0:.6f}s")

        t1 = time.perf_counter()
        self.q = {'joint': sp.symbols(f'q:{len(joints)}'),
                  'value': [0,0,0,0,0,0,0]}

        print(f"[init] Created symbolic joint variables in {time.perf_counter() - t1:.6f}s")

        t2 = time.perf_counter()
        self.number_of_joints = len(joints)
        print(f"[init] Counted {self.number_of_joints} joints in {time.perf_counter() - t2:.6f}s")

        self.transform = self.total_transformation_matrix()

        t3 = time.perf_counter()
        self.jacobian = self.jacobian_generator()
        print(f"[init] Generated Jacobian in {time.perf_counter() - t3:.6f}s")
        self.get_position__car()
        print(f"[init] Total initialization time: {time.perf_counter() - start_total:.6f}s\n")

    def transformation_matrix(self, key, joint: str):
        # start_trans = time.perf_counter()

        string = joint.split(',')
        rotation_plane = string[0].strip()
        orientation = string[1].strip()
        length = float(string[2])

        idx = key
        q = self.q['joint'][idx]
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

        # elapsed = time.perf_counter() - start_trans
        # print(f"[transformation_matrix] Joint {key} ({joint}) computed in {elapsed:.6f}s")
        return T

    def total_transformation_matrix(self):
        # start_total = time.perf_counter()

        T = sp.eye(4)
        for key, joint in self.joints.items():
            # t0 = time.perf_counter()
            T = T * self.transformation_matrix(key, joint)
            if key == 0:
                self.matrix_b1 = T
            elif key == 1:
                self.matrix_12 = T
            elif key == 2:
                self.matrix_23 = T
            elif key == 3:
                self.matrix_34 = T
            elif key == 4:
                self.matrix_45 = T
            elif key == 5:
                self.matrix_56 = T
            # print(f"  [total_T] Multiplied joint {key} in {time.perf_counter() - t0:.6f}s")

        # elapsed = time.perf_counter() - start_total
        # print(f"[total_transformation_matrix] Completed in {elapsed:.6f}s\n")
        return T
    
    def new_cor(self,angles, rad:bool=True):
        if rad:
            for i in range(self.number_of_joints-1):
                self.q['value'][i] = np.deg2rad(angles[i])
        else:
            for i in range(self.number_of_joints-1):
                self.q['value'][i] = angles[i]

    def jacobian_generator(self):
        # start_jac = time.perf_counter()
        # print("[jacobian_generator] Starting Jacobian computation...")

        T = self.total_transformation_matrix()

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        R = T[:3, :3]

        J = sp.zeros(6, self.number_of_joints)

        for i in range(self.number_of_joints):
            # t0 = time.perf_counter()
            J[0, i] = sp.diff(x, self.q['joint'][i])
            J[1, i] = sp.diff(y, self.q['joint'][i])
            J[2, i] = sp.diff(z, self.q['joint'][i])

            # Angular velocity part (from rotation derivative)
            dR_dqi = R.diff(self.q['joint'][i])
            w_i = sp.Matrix([
                dR_dqi[2, 1] - dR_dqi[1, 2],
                dR_dqi[0, 2] - dR_dqi[2, 0],
                dR_dqi[1, 0] - dR_dqi[0, 1]
            ]) / 2
            J[3:, i] = w_i
            # print(f"  [jacobian_generator] Partial Jacobian for joint {i} computed in {time.perf_counter() - t0:.6f}s")

        # t1 = time.perf_counter()
        # print(f"[jacobian_generator] Simplification done in {time.perf_counter() - t1:.6f}s")

        # total_elapsed = time.perf_counter() - start_jac
        # print(f"[jacobian_generator] Completed in {total_elapsed:.6f}s\n")
        return J
    def get_xyz(self,matrix):
        return matrix[0,3], matrix[1,3], matrix[2,3]
    
    def get_position__car(self):
        # Substitution dictionary for symbolic variables
        sub_dict = {
            self.q['joint'][i]: self.q['value'][i]
            for i in range(self.number_of_joints)
        }

        # Evaluate transformation matrices numerically
        self.q1_car = self.get_xyz(self.matrix_b1.evalf(subs=sub_dict))
        self.q2_car = self.get_xyz(self.matrix_12.evalf(subs=sub_dict))
        self.q3_car = self.get_xyz(self.matrix_23.evalf(subs=sub_dict))
        self.q4_car = self.get_xyz(self.matrix_34.evalf(subs=sub_dict))
        self.q5_car = self.get_xyz(self.matrix_45.evalf(subs=sub_dict))
        self.q6_car = self.get_xyz(self.matrix_56.evalf(subs=sub_dict))
        self.ee_car = self.get_xyz(self.transform.evalf(subs=sub_dict))
        return[self.q1_car, self.q2_car, self.q3_car, self.q4_car, self.q5_car, self.q6_car, self.ee_car]


        
    def plot_robot(self,positions=None):
        # Initialize coordinate containers
        coordinates = {'x': [], 'y': [], 'z': []}
        if positions == None:
            self.get_position__car()
            for q in [self.q1_car, self.q2_car, self.q3_car, self.q4_car, self.q5_car, self.q6_car, self.ee_car]:
                coordinates['x'].append(float(q[0]))
                coordinates['y'].append(float(q[1]))
                coordinates['z'].append(float(q[2]))
        elif type(positions) == dict:
            for q in positions.values:
                coordinates['x'].append(float(q[0]))
                coordinates['y'].append(float(q[1]))
                coordinates['z'].append(float(q[2]))

            


        # Create 3D figure (interactive)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Collect all joint coordinates in order


        # Convert to arrays for plotting
        x, y, z = np.array(coordinates['x']), np.array(coordinates['y']), np.array(coordinates['z'])

        # Plot line connecting joints + scatter markers
        ax.plot(x, y, z, '-o', color='blue', linewidth=4, markersize=6)
        ax.bar3d(self.q1_car[0]-0.5, self.q1_car[1]-0.5, self.q1_car[2],1,1,-self.q1_car[2], zsort= 'average')

        # Labels and title
        ax.set_title("Industrial Robot Configuration")
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.grid(True)

        # Make aspect ratio equal
        max_range = np.ptp([x, y, z])
        ax.set_xlim(- max_range/2, max_range/2)
        ax.set_ylim(- max_range/2, max_range/2)
        ax.set_zlim(- max_range/2, max_range/2)

        plt.show()

    def get_distance(self,a,b):
        return ((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)**0.5
    
    def get_to_destination(self, destination: tuple, dt: float, attractive_coff=10,repelent_joint=1, range_repelent=1, time=30):
        self.destination = destination
        self.dt = dt
        self.path = {}
        frame_idx = 0

        while destination != self.ee_car:
            # Calculate position error
            joint_pos = self.get_position__car()


            
            error = attractive_coff * (np.array(destination) - np.array(self.ee_car))

            for a in range(self.number_of_joints):
                for b in range(a + 1, self.number_of_joints):
                    r_vec = np.array(joint_pos[a]) - np.array(joint_pos[b])
                    dist = self.get_distance(joint_pos[a],joint_pos[b])
                    if dist <= range_repelent:  # vermijd deling door 0
                        # vectoriële afstoting
                        repelent_force = repelent_joint * (1/dist - 1/range_repelent) * (r_vec / dist**3)
                        error += repelent_force

            # Compute Jacobian numerically
            sub_dict = {self.q['joint'][i]: self.q['value'][i] for i in range(self.number_of_joints)}
            J_numeric = np.array(self.jacobian.evalf(subs=sub_dict)).astype(np.float64)

            # Calculate joint velocities using pseudo-inverse of Jacobian
            try:
                J_pseudo_inv = np.linalg.pinv(J_numeric[:3, :])  # Use only position part
                joint_velocities = J_pseudo_inv @ (error)
            except np.linalg.LinAlgError:
                print("Jacobian is singular, cannot compute inverse.")
                break

            # Update joint angles
            for i in range(self.number_of_joints):
                self.q['value'][i] += joint_velocities[i] * self.dt

            # Update end-effector position


            self.path[frame_idx] = {0:self.q1_car,
                                    1:self.q2_car,
                                    2:self.q3_car,
                                    3:self.q4_car,
                                    4:self.q5_car,
                                    5:self.q6_car,
                                    6:self.ee_car}

            # Optional: Print current position and error
            print(f"Current EE Position: {self.ee_car}, Error: {error}")
            frame_idx += 1
            print(frame_idx)
            if destination[0] + 0.00001>self.ee_car[0]>destination[0] - 0.00001 and destination[1] + 0.00001>self.ee_car[1]>destination[1] - 0.00001 and destination[2] + 0.00001>self.ee_car[2]>destination[2] - 0.00001:
                break
            elif frame_idx> 1/self.dt*time:
                break
    
    def make_gif(self, filename="robot_motion.gif"):
        frames = len(self.path)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ln, = ax.plot([],[],[], '-o', color='blue', linewidth=4, markersize=6)

        ax.set_title("Industrial Robot Configuration")
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.grid(True)

        # Make aspect ratio equal
        max_range = 6
        ax.set_xlim(- max_range/2, max_range/2)
        ax.set_ylim(- max_range/2, max_range/2)
        ax.set_zlim(- max_range/2, max_range/2)



        def update(i):
            x=[self.path[i][x][0] for x in range(len(self.path[i]))]
            y=[self.path[i][y][1] for y in range(len(self.path[i]))]
            z=[self.path[i][z][2] for z in range(len(self.path[i]))]
            ln.set_data(x, y)
            ln.set_3d_properties(z)
            return ln
        
        ani = FuncAnimation(fig, update, frames=frames, interval=self.dt*1000)
        writer = PillowWriter(fps=30/self.dt)
        ani.save(filename, writer=writer)
        plt.close(fig)




        


       