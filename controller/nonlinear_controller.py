import numpy as np
from scipy.spatial.transform import Rotation

class NonLinearPDController:
    def __init__(self):
        # hover control gains
        self.Kp = np.diag([10, 20, 120])
        self.Kd = np.diag([8.0 , 6.0, 25])
        # altitude control gains
        self.K_r = np.diag([2500, 2500, 400])
        self.K_w =  np.diag([60, 60, 50])
        m = 0.030  # weight (in kg) with 5 vicon markers (each is about 0.25g)
        g = 9.81  # gravitational constant
        I = np.array([[1.43e-5, 0, 0],
                      [0, 1.43e-5, 0],
                      [0, 0, 2.89e-5]])  # inertial tensor in m^2 kg
        L = 0.046  # arm length in m
        self.rotor_speed_min = 0
        self.rotor_speed_max = 2500
        self.mass = m
        self.inertia = I
        self.invI = np.linalg.inv(I)
        self.g = g
        self.arm_length = L
        self.k_thrust = 2.3e-08
        self.k_drag = 7.8e-11

    def control(self, des_state, state):
        '''
        :param desired state: pos, vel, acc, yaw, yaw_dot
        :param current state: pos, vel, euler, omega
        :return:
        '''
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        error_pos = state.get('x') - des_state.get('x')
        error_vel = state.get('v') - des_state.get('x_dot')

        error_pos = np.array(error_pos).reshape(3,1)
        error_vel = np.array(error_vel).reshape(3,1)

        # page 29
        rdd_des = np.array(des_state.get('x_ddot')).reshape(3,1) - np.matmul(self.Kd, error_vel) - np.matmul(self.Kp, error_pos)
        
        F_des = (self.mass * rdd_des) + np.array([0, 0, self.mass * self.g]).reshape(3,1)

        R = Rotation.as_matrix(Rotation.from_quat(state.get('q')))
        b3 = R[:,2]
        u1 = b3.T @ F_des

        b3_des = F_des / np.linalg.norm(F_des)
        a_Psi = np.array([np.cos(des_state.get('yaw')), np.sin(des_state.get('yaw',)), 0]).reshape(3,1)
        b2_des = np.cross(b3_des, a_Psi, axis=0) / np.linalg.norm(np.cross(b3_des, a_Psi, axis=0))
        b1_des = np.cross(b2_des, b3_des, axis=0)

        R_des = np.hstack((b1_des, b2_des, b3_des))

        temp = 0.5 * ((R_des.T @ R) - (R.T @ R_des))

        e_R = np.array([-temp[1,2], temp[0,2], -temp[0,1]]).reshape(3,1)
        
        u2 = self.inertia @ (-self.K_r @ e_R - self.K_w @ np.array(state.get('w')).reshape(3,1))

        gama = self.k_drag / self.k_thrust
        Len = self.arm_length
        cof_temp = np.array(
            [1, 1, 1, 1, 0, Len, 0, -Len, -Len, 0, Len, 0, gama, -gama, gama, -gama]).reshape(4, 4)

        u = np.array([u1, u2[0], u2[1], u2[2]])
        F_i = np.matmul(np.linalg.inv(cof_temp), u)

        for i in range(4):
            if F_i[i] < 0:
                F_i[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = np.sqrt(F_i[i] / self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max


        cmd_thrust = u1
        cmd_moment[0] = u2[0]
        cmd_moment[1] = u2[1]
        cmd_moment[2] = u2[2]

        cmd_q = Rotation.as_quat(Rotation.from_matrix(R_des))

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                        'cmd_thrust': cmd_thrust,
                        'cmd_moment': cmd_moment,
                        'cmd_q' : cmd_q,
                        }
        return control_input