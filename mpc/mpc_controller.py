import numpy as np
import cvxpy as cp
import control


class MPCController:
    def __init__(self):
        
        self.Q = np.diag([80, 80, 100, 80, 80, 100, 50, 50, 50, 50, 50, 50]) #state cost
        self.R = np.diag([50, 80, 80, 80]) #control cost
        self.P, self.K = self.get_terminal_cost(self.Ad, self.Bd, self.Q, self.R)#terminal cost
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
   
    def control(self, des_state, state, x0, A, B, A_obs, B_obs):
       
       x_error = state.get('x') - des_state.get('x')
       des_state = x_target
       
       n, m = self.Q.shape[0], self.R.shape[0]
       x_cp = cp.Variable((N + 1, n))
       u_cp = cp.Variable((N, m))
       
       cost = 0.
       constraints = []
       
       constraints += [x_cp[:,0] == x0.flatten()] #Initial state
       
       for i in range(N+1):
           cost += cp.quad_form(x_cp[:,i] - x_target, self.Q) #add state cost
           cost += cp.quad_form(u_cp[:,i] - u_target, self.R) #add control cost
           
           constraints += [A_obs @ x_cp[:2,i] <= B_obs.flatten()] #Obstacle avoidance
           constraints += [x[:,i]] == A@x_cp[:,i] + B@x_cp[:,i]
           # constraints += [x_cp[6,i] <= 0.5]
           # constraints += [x_cp[7,i] <= 0.5]
           # constraints += [x_cp[6,i] >= -0.5]
           # constraints += [x_cp[7,i] >= -0.5]
           # constraints += [u_cp[:,i] >= -0.07*30]
           # constraints += [u_cp[:,i] <= 0.07*30]
           
       cost += cp.quad_form((x_cp[:,N]-x_target), self.P) #add terminal cost
       
       prob = cp.Problem(cvx.Minimize(cost), constraints)
       prob.solve()
       x = x_cp.value
       u = u_cp.value
       status = prob.status
       
       return x, u, status, x_error