import gym
from linear_controller import LinearPDController
from nonlinear_controller import NonLinearPDController
#from mpc_controller import MPCController
from envs.Quadrotor import Quadrotor
from trajectory import *
from matplotlib import pyplot as plt
from collections import deque
import numpy as np

"""
Please run this file by running python main.py, you can change the trajectory or controller by setting traj and controller variable respectively.
"""

if __name__ == "__main__":
    env = gym.make('Quadrotor-v0')
    freq = 100 
    dt = 1/freq
    t = 0
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    abs_err = lambda x1, x2 : np.abs(x1-x2)
    sqnorm_err = lambda x1,x2 : (x1-x2)**2

    END_SIM_TIME = 30
    STABLE_THRESHOLD = 30
    ERR_STABLE_THRESHOLD = 0.001

    CIRCLE = 0
    DIAMOND = 1
    TUD = 2
    HOVER = 3

    NONLINEAR = 0
    LINEAR = 1

    traj = DIAMOND
    controller = NONLINEAR
    if traj == CIRCLE:
        T=9
        trajectory = CircleTrajectory(dt, radius=5, end_z=2.5, T=T)
    elif traj == DIAMOND:
        T=7
        trajectory = DiamondTrajectory(dt, end_x=1, T=T)
    elif traj == TUD:
        trajectory = TUDTrajectory(dt)
        ax.set_xlim(-0.1,0.1)
    elif traj == HOVER:
        T=3
        trajectory = HoverTrajectory(dt, hoverHeight=5, T=T)

    ctrl_type = "Non-linear Controller"

    if controller == NONLINEAR:
        controller = NonLinearPDController()
    elif controller == LINEAR:
        controller = LinearPDController()
        ctrl_type = "Linear Controller"
    elif controller == MPC:
        controller = MPCController()
    
    ax.set_title(f"Following {trajectory.getName()} trajectory with {ctrl_type}")

    start_pos = trajectory.getDesState(t)['x']
    end_pos = trajectory.getDesState(END_SIM_TIME)['x']
    cur_state = env.reset(position=start_pos)
    print(f"Starting state : {cur_state}, Ending state : {end_pos}")

    plan_trajectory = {'x' : [], 'y' : [], 'z' : []}
    real_trajectory = {'x' : [], 'y' : [], 'z' : []}

    w_history = []
    stable_count = 0
    time_to_complete = END_SIM_TIME
    while t < END_SIM_TIME:
        des_state = trajectory.getDesState(t)
        ctrl_var = controller.control(des_state, cur_state)
        action = ctrl_var['cmd_motor_speeds']
        cur_state, reward, done, info = env.step(action)
        print(f"Desired state: {des_state}")
        print(f"Current state: {cur_state}")
        print(f"Executed action {action}")
        print("--------------------------")

        plan_trajectory['x'].append(des_state['x'][0])
        plan_trajectory['y'].append(des_state['x'][1])
        plan_trajectory['z'].append(des_state['x'][2])

        cur_x = cur_state['x'][0]
        cur_y = cur_state['x'][1]
        cur_z = cur_state['x'][2]

        real_trajectory['x'].append(cur_x)
        real_trajectory['y'].append(cur_y)
        real_trajectory['z'].append(cur_z)

        end_pos_err = np.sum(sqnorm_err(end_pos, np.array([cur_x, cur_y, cur_z])))

        if end_pos_err < ERR_STABLE_THRESHOLD:
            stable_count +=1
        if stable_count > STABLE_THRESHOLD:
            time_to_complete = t
            break

        w_history.append(cur_state['w'])

        t += dt

    pos_err_tracking = 0
    calc_err = sqnorm_err
    err_t = []
    battery_spent = 0
    for i in range(len(plan_trajectory['x'])):
        x_err = calc_err(plan_trajectory['x'][i], real_trajectory['x'][i])
        y_err = calc_err(plan_trajectory['y'][i], real_trajectory['y'][i])
        z_err = calc_err(plan_trajectory['z'][i], real_trajectory['z'][i])
        tot_err = x_err + y_err + z_err
        err_t.append((tot_err, i, plan_trajectory['x'][i], plan_trajectory['y'][i], plan_trajectory['z'][i]))
        pos_err_tracking += tot_err
        if 0:
            print(f"Norm error for x is {calc_err(plan_trajectory['x'][i], real_trajectory['x'][i])}")
            print(f"Norm error for y is {calc_err(plan_trajectory['y'][i], real_trajectory['y'][i])}")
            print(f"Norm error for z is {calc_err(plan_trajectory['z'][i], real_trajectory['z'][i])}")
    err_t.sort()
    largest_errors = err_t[-10:]
    for tup in largest_errors:
        print(f"Big error spotted at time t = {tup[1]} with error {tup[0]} and with coordinate in planned trajectory : {tup[2], tup[3], tup[4]}")
    print(f"Simulation of {END_SIM_TIME}s, following {trajectory.getName()} trajectory with {ctrl_type}")
    print(f"(5.1) Pos error tracking (Unnormalized): {pos_err_tracking}, Normalized (divided with dt) : {pos_err_tracking*dt}" )
    print(f"(5.2) Total time spent finishing task : {time_to_complete}" )
    print(f"(5.3) Total battery storage spent finishing task : {sum(sum(w**2) for w in w_history)}" )
    ax.plot(plan_trajectory['x'], plan_trajectory['y'], plan_trajectory['z'], c='#0000FF')
    ax.plot(real_trajectory['x'], real_trajectory['y'], real_trajectory['z'], c='#FF0000')
    plt.show()

