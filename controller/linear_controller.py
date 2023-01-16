import numpy as np
from scipy.spatial.transform import Rotation

class LinearPDController:
    def __init__(self):
        # hover control gains
        self.Kp = np.diag([8, 8, 90])
        self.Kd = np.diag([5.5, 5.5, 25])
        # angular control gains
        self.Kp_t = np.diag([2700, 2700, 100])
        self.Kd_t = np.diag([200, 200, 100])
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
        rdd_des = np.squeeze(rdd_des)

        # Desired roll, pitch and yaw (in rad).
        # page 30 Equation 8
        psi_des = des_state.get('yaw')
        phi_des = (np.sin(psi_des) * rdd_des[0] - rdd_des[1] * np.cos(psi_des)) / self.g
        theta_des = (np.cos(psi_des) * rdd_des[0] + rdd_des[1] * np.sin(psi_des)) / self.g

        # calculate u1 (thrust)
        # page 30 Equation 8
        u1 = (rdd_des[2] + self.g) * self.mass
        quat = state['q']
        omega = state['w']
        rotation = Rotation.from_quat(quat)
        angle = rotation.as_rotvec()
        angle = np.array(angle)

        # page 31
        error_angle = np.matmul(self.Kp_t, np.array([phi_des, theta_des, psi_des]) - angle) +\
            np.matmul(self.Kd_t, np.array([0,0, des_state['yaw_dot']]) - omega)
        u2 = self.inertia @ error_angle

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


        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment}
        return control_input