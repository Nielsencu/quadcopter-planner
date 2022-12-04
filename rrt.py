import numpy as np

class Point3D:
    def __init__(self, x, y, z):
        self.x=x
        self.y=y
        self.z=z

class Configuration:
    def __init__(self, x, y, z, yaw=0):
        self.pos = Point3D(x,y,z)
        self.yaw = yaw

class GridMap:
    def __init__(self, width, height, depth):
        self.map=np.array([[[0 for _ in range(depth)] for _ in range(width)] for _ in range(height)])
        
    def getMap(self):
        return self.map
        
    def flagAsOccupied(self, point: Point3D):
        self.map[point.x][point.y][point.z] = 1
        
    def addObstacles(self, point : Point3D, radius=1):
        for i in range(1, radius+1):
            for x_inc in range(-i, i+1):
                point.x += x_inc
                self.flagAsOccupied(point)
                point.x -= x_inc
                for y_inc in range(-i, i+1):
                    point.y += y_inc
                    self.flagAsOccupied(point)
                    point.y -= y_inc
                    for z_inc in range(-i, i+1):
                        point.z += z_inc
                        self.flagAsOccupied(point)
                        point.z -= z_inc
                
class RRTPlanner:
    def __init__(self, map : GridMap):
        self.map = map
        
    def getTrajectory(self, start: Configuration, end: Configuration):
        ...