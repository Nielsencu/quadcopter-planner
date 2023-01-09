import numpy as np
import cvxpy as cp
import control
from envs.Quadrotor import Quadrotor
import math 
from trajectory import *



class MPCController:
    def __init__(self):
        
        self.Q = np.diag([80, 80, 100, 80, 80, 100, 50, 50, 50, 50, 50, 50]) #state cost
        self.R = np.diag([50, 80, 80, 80]) #control cost
        #self.P, self.K = self.get_terminal_cost(self.Ad, self.Bd, self.Q, self.R)#terminal cost
        self.N = 3 #horizon/ number of look-ahead time-steps
   
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
        self.dt = 0.01
        
    def pack_state_as_dict(self, pos, vel, acc, yaw, yaw_dot):
        return {'x' : pos, 'x_dot' : vel, 'x_ddot' : acc, 'yaw' : yaw, 'yaw_dot' : yaw_dot}
   
    def mpccontrol(self, des_state, state): #A_obs, B_obs, , x0
       s = 12 #number of states: 
       #s = {x, y, z, dx, dy, dz, psi, theta, phi, psi_dot, phi_dot, theta_dot}
       u = 4 #number of inputs
       
       x_error = state.get('x') - des_state.get('x')
       
       x0 = state2x(self,(TUDTrajectory.getDesState(self, t= 0)))
      
       n, m = self.Q.shape[0], self.R.shape[0]
       x_cp = cp.Variable((n, self.N+1))
       u_cp = cp.Variable((m, self.N))
       #u_target = np.array([self.mass*self.g, 0, 0, 0])
       cost = 0.
       constraints = []
       x_target = []
       
       for i in range(self.N):
           # if k == self.N: -----> terminal cost with terminal set
           #     cost += cp.quad_form(x[:, self.N]-x_ref_k, self.P)
           t = i*self.dt
           x_target = (state2x(self, TUDTrajectory.getDesState(self, t)))
           
           if i > 0:
               cost += cp.quad_form(x_cp[:,i] - x_target, self.Q) #add state cost #state.get(all) - des_state.gett(all)
           
           cost += cp.quad_form(u_cp[:,i], self.R) #add control cost '- u_target'
           
           constraints += [[x_cp[:,i]] == Quadrotor().Ad@x_cp[:,i] + Quadrotor().Bd@u_cp[:,i]]
           
           constraints += [Quadrotor().Hx@x_cp[:,i] <= Quadrotor().h1[Quadrotor().Hu1.shape[0]:]]
           constraints += [Quadrotor().Hu1@x_cp] <= Quadrotor().h1[:Quadrotor().Hu1.shape[0]]
           #constraints += [A_obs@x_cp[:,i+1] <= B_obs] #Obstacle avoidance
    
       constraints += [x_cp[:,0] == x0] #Initial state
       cost += self.Q*(x_cp[0,self.N] - x_target)**2
       
       prob = cp.Problem(cp.Minimize(cost), constraints)
       prob.solve()
       x = x_cp.value
       u = u_cp.value
       status = prob.status
       
       cmd_motor_speeds = np.zeros((4,))
       cmd_thrust = 0
       cmd_moment = np.zeros((3,))
       # cmd_q = np.zeros((4,))

       F = (np.linalg.inv(self.to_TM) @ u).astype(float)

       for i in range(4):
           if F[i] < 0:
               F[i] = 0
               cmd_motor_speeds[i] = self.rotor_speed_min
           cmd_motor_speeds[i] = np.sqrt(F[i] / self.k_thrust)
           if cmd_motor_speeds[i] > self.rotor_speed_max:
               cmd_motor_speeds[i] = self.rotor_speed_max

       cmd_thrust = u[0]  # thrust
       cmd_moment[0] = u[1]  # moment about p
       cmd_moment[1] = u[2]  # moment about q
       cmd_moment[2] = u[3]  # moment about r

       control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                        'cmd_thrust': cmd_thrust,
                        'cmd_moment': cmd_moment}
       
       return control_input, x_error
       
   


def state2x(self, state):
        x = state.get('x')#.flatten()
        v = state.get('v')#.flatten()
        try:
            q = state.get('q')#.flatten()
            w = state.get('w')#.flatten()
            euler_ang = self.euler_from_quaternion(q[0], q[1], q[2], q[3])
        except:
            euler_ang = np.zeros(3)
            euler_ang[2] = state.get('yaw')
            w = np.zeros(3)
            w[2] = state.get('yaw_dot')

        x_init = np.block([x, v, euler_ang, w])
        return x_init


def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return np.array([roll_x, pitch_y, yaw_z])  # in radians
    

    