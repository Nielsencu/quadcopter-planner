import numpy as np
import math

def tj_from_line(start_pos, end_pos, time_ttl, t_c):
    v_max = (end_pos - start_pos)*2 / time_ttl
    if t_c >= 0 and t_c < time_ttl/2:
        vel = v_max*t_c / (time_ttl/2)
        pos = start_pos + t_c*vel/2
    else:
        vel = v_max*(time_ttl - t_c) / (time_ttl/2)
        pos = end_pos - (time_ttl-t_c) * vel/2
    return pos

class Trajectory:
    def __init__(self, dt):
        self.t = 0
        self.dt = dt

    def getName(self):
        return self.name

    def pack_state_as_dict(self, pos, vel, acc, yaw, yaw_dot):
        return {'x' : pos, 'x_dot' : vel, 'x_ddot' : acc, 'yaw' : yaw, 'yaw_dot' : yaw_dot}
                # 3 + 3 + 3 + 1 + 1


class HoverTrajectory(Trajectory):
    def __init__(self, dt, hoverHeight=2, T=5):
        super().__init__(dt)
        self.hoverHeight = hoverHeight
        self.T = T
        self.name = 'Hover'
    
    def getDesState(self, t):
        self.t = t
        start_pos = np.array([0,0,0])
        end_pos = np.array([0,0,self.hoverHeight])            
        if self.t > self.T:
            vel = np.array([0,0,0])
            acc = np.array([0,0,0])
            return self.pack_state_as_dict(end_pos, vel, acc, 0,0)
        pos= tj_from_line(start_pos, end_pos, self.T, self.t)

        nextPos= np.array(tj_from_line(start_pos, end_pos, self.T, self.t+self.dt))
        nextNextPos= np.array(tj_from_line(start_pos, end_pos, self.T, self.t+2*self.dt))

        vel = (nextPos - pos) / self.dt
        nextVel = (nextNextPos - nextPos) / self.dt

        acc = (nextVel - vel) / self.dt

        vel = np.clip(vel, -1, 1) # Limit max vel
        acc = np.clip(acc, -1, 1) # Limit max acc
        return self.pack_state_as_dict(pos, vel, acc, 0,0)

class DiamondTrajectory(Trajectory):
    def __init__(self, dt, end_x=1, T=12):
        super().__init__(dt)
        self.end_x = end_x
        self.T=T
        self.name = 'Diamond'

    def getDesState(self, t):
        self.t = t
        # 0, sqrt 2, sqrt 2 -> 0, 0, 2 sqrt 2 -> 0, -sqrt2, sqrt 2, 0, 0, 0
        start_pos = np.zeros(3)
        end_pos = np.zeros(3)
        waypoints = np.array([np.zeros(3), 
            np.array([self.end_x*1/4, np.sqrt(2), np.sqrt(2)]),
            np.array([self.end_x*2/4, 0, 2*np.sqrt(2)]),
            np.array([self.end_x*3/4, -np.sqrt(2), np.sqrt(2)]),
            np.array([self.end_x, 0, 0]),
        ])
        if t < self.T / 4:
            end_pos = waypoints[1]
            t_c = self.t
        elif t < self.T / 2:
            start_pos = waypoints[1]
            end_pos = waypoints[2]
            t_c = self.t - self.T/4
        elif t < 3 * self.T / 4:
            start_pos = waypoints[2]
            end_pos = waypoints[3]
            t_c = self.t - self.T/2
        elif t < self.T:
            start_pos = waypoints[3]
            end_pos = waypoints[4]
            t_c = self.t - self.T*3/4
        else:
            start_pos = waypoints[4]
            end_pos = waypoints[4]
            t_c = self.t - self.T
        pos = tj_from_line(start_pos, end_pos, self.T/4, t_c)

        nextPos = np.array(tj_from_line(start_pos, end_pos, self.T/4, t_c+self.dt))
        nextNextPos = np.array(tj_from_line(start_pos, end_pos, self.T/4, t_c+2*self.dt))

        vel = (nextPos - pos) / self.dt
        nextVel = (nextNextPos - nextPos) / self.dt

        acc = (nextVel - vel) / self.dt

        vel = np.clip(vel, -2, 2) # Limit max vel
        acc = np.clip(acc, -2, 2) # Limit max acc
        return self.pack_state_as_dict(pos, vel, acc, 0,0)

class CircleTrajectory(Trajectory):
    def __init__(self, dt, radius=5, end_z=2.5, T=12):
        super().__init__(dt)
        self.T = T
        self.radius = radius
        self.end_z = end_z
        self.name = 'Circle'

    def pos_from_angle(self, a):
        return np.array([self.radius*math.cos(a), self.radius*math.sin(a), self.end_z*a/(2*np.pi)])

    def get_vel(self, t):
        angle1 = tj_from_line(0, 2*np.pi, self.T, t)
        pos1 = self.pos_from_angle(angle1)
        angle2 = tj_from_line(0, 2*np.pi, self.T, t+self.dt)
        vel = (self.pos_from_angle(angle2) - pos1)/self.dt
        return vel

    def getDesState(self, t):
        self.t = t
        if self.t > self.T:
            pos = [self.radius, 0, 2.5]
            vel = [0,0,0]
            acc = [0,0,0]
        else:
            angle = tj_from_line(0, 2*np.pi, self.T, self.t)
            pos = self.pos_from_angle(angle)
            vel = self.get_vel(self.t)
            acc = (self.get_vel(self.t+self.dt) - vel) / self.dt
        return self.pack_state_as_dict(pos, vel, acc, 0, 0)


class TUDTrajectory(Trajectory):
    def __init__(self, dt, T=12):
        super().__init__(dt)
        self.T=T
        self.name = 'TUD'

    def pos_from_angle(self, radius, a):
        return np.array([0, radius*math.cos(a), radius*math.sin(a)])

    def get_vel(self, radius, t):
        angle1 = tj_from_line(0, np.pi, self.T, t)
        pos1 = self.pos_from_angle(radius, angle1)
        angle2 = tj_from_line(0, np.pi, self.T, t+self.dt)
        vel = (self.pos_from_angle(radius, angle2) - pos1)/self.dt
        return vel

    def getDesState(self, t):
        self.t=t
        waypoints = [
            [0,0,0],
            [0,0,7],
            [0,-2,7],
            [0,3,7],
            [0,3,2], # semicircle U
            [0,7,2],
            [0,7,7],
            [0,8.5,7], # semicircle U
            [0,8.5,0], 
            [0,8,0],
            [0,8,7],
        ]
        
        trajPeriod = [3] * 10
        trajPeriod[0] = 3.3 # Rise to top mid of T [0,0] -> [0,7]
        trajPeriod[1] = 1.0 # Move to left of T [0,7] -> [-2,7]
        trajPeriod[2] = 2.5 # Move to top left of U [-2,7] -> [3,7]
        trajPeriod[3] = 2 # Move to left starting pos of bottom semicircle of U [3,7] -> [3,2]
        trajPeriod[4] = 4 # U semicircle duration [3,2] -> [7,2]
        trajPeriod[5] = 2.5 # Move to top right of U [7,2] -> [7,7]
        trajPeriod[6] = 1.5 # Move to to top left starting pos of semicirlce of D [7,7] -> [8.5,7]
        trajPeriod[7] = 5 # D semicircle duration [8.5, 7] -> [8.5,0]
        trajPeriod[8] = 1 # Move to Left [8.5,0] -> [8.0,0]
        trajPeriod[9] = 1.5 # Move vertically up back to top left of D [8,0] -> [8,7]
        timeForEachPoint = [0]
        acc = 0
        for period in trajPeriod:
            acc += period
            timeForEachPoint.append(acc)
        
        trajNo = -1
        for (index, t_traj) in enumerate(timeForEachPoint):
            if t < t_traj:
                trajNo = index
                t_start = timeForEachPoint[index-1]
                break
        
        if self.t < timeForEachPoint[-1]:
            T = timeForEachPoint[trajNo] - timeForEachPoint[trajNo-1]
            t_c = (self.t - t_start) # 5.5 - 4
            if trajNo == 5 or trajNo == 8:
                self.T = T
                if trajNo == 5:
                    center = [0,5,2]
                    start_pos = 0
                    end_pos = np.pi
                    radius = 2
                    calcPos = lambda center, posOut : np.array(center) - posOut
                else:
                    center = [0,8,3.5]
                    start_pos = 0.5 * np.pi
                    end_pos = 1.5 * np.pi
                    radius = 3.5
                    calcPos = lambda center, posOut : np.array([0, center[1]-posOut[1], center[2] + posOut[2]])

                angle = tj_from_line(start_pos, end_pos, self.T, t_c)
                nextAngle = tj_from_line(start_pos, end_pos, self.T, t_c+self.dt)
                nextNextAngle = tj_from_line(start_pos, end_pos, self.T, t_c+2*self.dt)

                pos = calcPos(center, self.pos_from_angle(radius, angle))
                nextPos = calcPos(center, self.pos_from_angle(radius, nextAngle))
                nextNextPos = calcPos(center, self.pos_from_angle(radius, nextNextAngle))

                vel = (nextPos - pos) / self.dt
                nextVel = (nextNextPos - nextPos) / self.dt

                acc = (nextVel - vel) / self.dt

                vel = np.clip(vel, -1, 1) # Limit max vel
                acc = np.clip(acc, -1, 1) # Limit max acc
                return self.pack_state_as_dict(pos, vel, acc, 0, 0)
            start_pos = waypoints[trajNo-1]
            end_pos = waypoints[trajNo]
        else:
            return self.pack_state_as_dict(np.array(waypoints[-1]), np.zeros(3), np.zeros(3), 0, 0)
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)
        

        pos = tj_from_line(start_pos, end_pos, T, t_c)

        nextPos = np.array(tj_from_line(start_pos, end_pos, T, t_c+self.dt))
        nextNextPos = np.array(tj_from_line(start_pos, end_pos, T, t_c+2*self.dt))

        vel = (nextPos - pos) / self.dt
        nextVel = (nextNextPos - nextPos) / self.dt

        acc = (nextVel - vel) / self.dt

        vel = np.clip(vel, -1, 1) # Limit max vel
        acc = np.clip(acc, -1, 1) # Limit max acc
        return self.pack_state_as_dict(pos, vel, acc, 0, 0)