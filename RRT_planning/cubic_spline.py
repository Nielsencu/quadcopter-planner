import numpy as np
import scipy

def divideTimeFromPath(path, T):
    totalWaypoints = len(path)
    distanceList = []
    for i in range(totalWaypoints-1):
        distance = np.linalg.norm(path[totalWaypoints-1-i, :] - path[totalWaypoints-i-2, :])
        distanceList.append(distance)
        
    tArr = np.zeros(totalWaypoints)
    for i in range(totalWaypoints-1):
        tArr[i] = sum(distanceList[0:i]) / sum(distanceList) * T
    tArr[totalWaypoints-1] = T
    return tArr

def getCubicSpline(path, T):
    tArr = divideTimeFromPath(path, T)
    tResampled = np.linspace(0, T, int(T/0.01))
    
    def getPosVelAccFromCubicSpline(arr):
        f = scipy.interpolate.CubicSpline(tArr, arr, bc_type='clamped')
        pos = f(tResampled)
        vel = f(tResampled, 1)
        acc = f(tResampled, 2)
        return pos, vel, acc
    
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    z = [point[2] for point in path]
    
    xPos, xVel, xAcc = getPosVelAccFromCubicSpline(x)
    yPos, yVel, yAcc = getPosVelAccFromCubicSpline(y)
    zPos, zVel, zAcc = getPosVelAccFromCubicSpline(z)
    
    pos = np.vstack((xPos, yPos, zPos)).T
    vel = np.vstack((xVel, yVel, zVel)).T
    acc = np.vstack((xAcc, yAcc, zAcc)).T
    
    return pos, vel, acc
    
        