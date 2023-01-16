import numpy as np
import time
from typing import List, Set
import math
from dataclasses import dataclass

CONNECTOR_LENGTH = 5
@dataclass
class Point3D:
    x : int
    y : int
    z : int

    def __str__(self):
        return f'{self.x, self.y, self.z}'
        
    def getL2(self, p1) -> float:
        return (p1.x - self.x) ** 2 + (p1.y - self.y) ** 2 + (p1.z - self.z) ** 2
    
    def getMag(self):
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5
    
    def __sub__(self,other):
        return Point3D(self.x-other.x, self.y-other.y, self.z-other.z)
    
    def __add__(self,other):
        return Point3D(self.x+other.x, self.y+other.y, self.z+other.z)
    
    def __truediv__(self, other):
        return Point3D(self.x / other, self.y / other, self.z / other)
    
    def __rtruediv__(self, other):
        return Point3D(other / self.x, other / self.y, other / self.z)

class Configuration:
    def __init__(self, x, y, z, yaw=0.):
        self.pos = Point3D(x,y,z)
        self.yaw = yaw

    def __hash__(self):
        return hash((self.pos.x, self.pos.y, self.pos.z, self.yaw))

    def __str__(self):
        return f'Position {self.pos} yaw {self.yaw}'

    def getError(self, other):
        return self.pos.getL2(other.pos) + min(abs(self.yaw - other.yaw), 2 * math.pi * abs(self.yaw - other.yaw)) ** 2

    @staticmethod
    def getRandomConfiguration(lowBound, highBound):
        pos = np.random.randint(low=lowBound,high=highBound, size=3) 
        yaw = np.random.randint(low=-math.pi, high=math.pi) # Sample yaw from -pi to pi
        return Configuration(pos[0], pos[1], pos[2], yaw)
        
class RobotModel:
    def __init__(self, radius=5):
        self.radius = radius
        
    def getDiscretizedRepresentation(self) -> List[Point3D]:
        """
        Returns discrete points representing the robot model (could be modelled as sphere)
        """
        # TODO: Model Robot, should perform collision checking using this robot model as well
        return []

class GridMap:
    def __init__(self, xMax, yMax, zMax):
        self.map=np.array([[[0 for _ in range(xMax)] for _ in range(yMax)] for _ in range(zMax)])
        self.xMax = xMax
        self.yMax = yMax
        self.zMax = zMax
        
    def getMap(self):
        return self.map
    
    def isOutOfBounds(self, point: Point3D):
        return not(0 <= point.x < self.xMax and 0 <= point.y < self.yMax and 0 <= point.z < self.zMax)
    
    def checkCollision(self, point: Point3D) -> bool:
        return self.map[point.x, point.y, point.z] == 1
        
    def flagAsOccupied(self, point: Point3D):
        self.map[point.x, point.y, point.z] = 1
        
    def addObstacles(self, x, y, z, radius=1):
        point = Point3D(x,y,z)
        x,y,z = point.x, point.y, point.z
        original = Point3D(x,y,z)
        max_l2 = radius**2
        for x_inc in range(-radius, radius+1):
            point.x = x + x_inc
            if point.getL2(original) < max_l2:
                self.flagAsOccupied(point)
            for y_inc in range(-radius, radius+1):
                point.y = y + y_inc
                if point.getL2(original) < max_l2:
                    self.flagAsOccupied(point)
                for z_inc in range(-radius, radius+1):
                    point.z = z + z_inc
                    if point.getL2(original) < max_l2:
                        self.flagAsOccupied(point)

    def addRectangle(self, x, y, z, width=1, height=1, depth=1):
        for x_inc in range(x - int(width/2), x + int(width/2)):
            for y_inc in range(y - int(height/2), y + int(height/2)):
                for z_inc in range(z - int(depth/2), z + + int(depth/2)):
                    point = Point3D(x_inc, y_inc, z_inc)
                    self.flagAsOccupied(point) 