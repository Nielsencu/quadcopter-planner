# -*- coding: utf-8 -*-
"""
Created on Wed Jan  4 15:24:25 2023

@author: Bas
"""

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
    def __init__(self, x, y, z, cost =0, yaw=0):
        self.pos = Point3D(x,y,z)
        self.yaw = yaw
        self.cost = cost
        self.parent = None
        

    @staticmethod
    def getRandomConfiguration():
        # Sample X,Y,Z space TODO:Change hardcoding of low and high 
        pos = np.random.randint(low=0,high=99, size=3) 
        yaw = np.random.randint(low=-math.pi, high=math.pi) # Sample yaw from -pi to pi
        cost = 0 
        return Configuration(pos[0], pos[1], pos[2], cost, yaw)
        
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
                
class RRTStarPlanner:
    # TODO: Extend Vanilla RRT and Implement RRTStar with rewiring
    def __init__(self):
        ...

class InformedRRTStarPlanner:
    def __init__(self):
        ...
class RRTPlanner:
    def __init__(self, map : GridMap, maxTimeTaken=2, maxNodesExpanded=10000):
        self.map = map
        self.maxTimeTaken = maxTimeTaken
        # TODO: Bug when error margin is 75, won't find path
        self.startErrorMargin = 3 * (5**2)
        self.goalErrorMargin = 3 * (5**2)
        self.maxNodesExpanded = maxNodesExpanded
        print(f'Initialized RRT Planner with\nstartErrorMargin {self.startErrorMargin}\ngoalErrorMargin {self.goalErrorMargin}\nmaxTimeTaken {self.maxTimeTaken}\nmaxNodesExpanded {self.maxNodesExpanded}')
        
    def steering(self, q1 : Configuration, q2: Configuration) -> List[Configuration]:
        v = q2.pos - q1.pos
        vMag = v.getMag()
        u = v / vMag
        start = q1.pos
        discretizedLine = []
        DISCRETIZATION = 10
        for l in np.linspace(0,1,DISCRETIZATION,endpoint=True):
            segmentLength = min(vMag, CONNECTOR_LENGTH)
            x = start.x  + int(l * segmentLength * u.x)
            y = start.y + int(l * segmentLength * u.y)
            z = start.z + int(l * segmentLength * u.z)
            configuration = Configuration(x,y,z)
            if self.map.isOutOfBounds(configuration.pos) or self.map.checkCollision(configuration.pos):
                break
            discretizedLine.append(configuration)
        return discretizedLine
    
    def getLowestCostNeighbor(self, vertices: Set[Configuration], q: Configuration) -> Configuration:
        lowestcostvertex = None
        #TODO: Find maximum of minimum distance according to map size instead o inf
        minDist = 5 # radius
        for vertex in vertices:
            #TODO: Implement closest neighbor with yaw, currently only with L2 distance of xyz
            # check neighbors within radius of the new point and check which one has 
            #the shortest total cost and choose that one as new vertex
            #This means you will have to keep track of the total cost to get to each vertex
            # This can maybe be done in the similar way of the .parent but then .cost
            dist = vertex.pos.getL2(q.pos)
            minCost = math.inf
            if dist < minDist and dist != 0 and vertex.cost < minCost:
                minCost = vertex.cost
                lowestcostvertex = vertex
   
        return lowestcostvertex
        
    def getTrajectory(self, start: Configuration, goal: Configuration, ax=None) -> List[Configuration]:
        vertices = set()
        timeStart = time.time()
        nodesCount = 0
        vertices.add(start)
        start.cost = 0
        while time.time() - timeStart < self.maxTimeTaken and nodesCount < self.maxNodesExpanded:
            q = Configuration.getRandomConfiguration()
            nodesCount +=1
            if self.map.checkCollision(q.pos):
                continue
            qPrime = self.getLowestCostNeighbor(vertices, q)
            segment = self.steering(qPrime, q)
            if not segment:
                continue
            q = segment[-1]
            vertices.add(q)
            if ax is not None:
                # TODO: Real-time plotting? Use blit to cache background and avoid redrawing
                x = np.array([point.pos.x for point in segment])
                y = np.array([point.pos.y for point in segment])
                z = np.array([point.pos.z for point in segment])
                ax.plot(x, y, z, '-b')
            q.parent = qPrime
            q.cost = qPrime.cost +  qPrime.pos.getL2(q.pos)
            
            #here comes the rewiring
            minDist = math.inf
            for vertex in vertices:
                 dist = vertex.pos.getL2(q.pos)
                 if dist < minDist and dist != 0 and q.cost + dist < vertex.cost:
                    vertex.parent = q
                    
            if q.pos.getL2(goal.pos) > self.goalErrorMargin:
                continue
            path = []
            while q.parent != None and q.parent.pos != start.pos:
                path.append(q)
                q = q.parent
            path.append(q)
            if q.pos.getL2(start.pos)  < self.startErrorMargin:
                print(f'Number of nodes expanded {nodesCount} and time taken {time.time() - timeStart}')
                return path
        print("Failed to find path")
        return []
    
    #possible_vertices = ("f",[possible_vertices])