import numpy as np
import time
import typing

class LinkedList:
    def __init__(self, value):
        self.value = value
        self.parent = None

class Point3D:
    def __init__(self, x, y, z):
        self.x=x
        self.y=y
        self.z=z
        
    def getL2(self, p1) -> float:
        return (p1.x - self.x) ** 2 + (p1.y - self.y) ** 2 + (p1.z - self.z) ** 2

class Configuration:
    def __init__(self, x, y, z, yaw=0):
        self.pos = Point3D(x,y,z)
        self.yaw = yaw
        
class RobotModel:
    def __init__(self, radius=5):
        self.radius = radius
        
    def getDiscretizedRepresentation(self) -> list[Point3D]:
        """
        Returns discrete points representing the robot model (could be modelled as sphere)
        """
        return []

class GridMap:
    def __init__(self, width, height, depth):
        self.map=np.array([[[0 for _ in range(depth)] for _ in range(width)] for _ in range(height)])
        
    def getMap(self):
        return self.map
    
    def checkCollision(self, point: Point3D) -> bool:
        return self.map[point.x][point.y][point.z] == 1
        
    def flagAsOccupied(self, point: Point3D):
        self.map[point.x][point.y][point.z] = 1
        
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
                
class RRTPlanner:
    def __init__(self, map : GridMap, maxTimeTaken=1, goalErrorMargin=1, maxNodesExpanded=100000):
        self.map = map
        self.GoalErrorMargin = goalErrorMargin
        self.maxTimeTaken = maxTimeTaken
        self.maxNodesExpanded = maxNodesExpanded
        
    def steering(self, q1 : Configuration, q2: Configuration) -> list[Configuration]:
        #TODO: Implement steering between two configurations
        return []
    
    def getClosestNeighbor(self, vertices: set[Configuration], q: Configuration) -> Configuration:
        #TODO: Implement closest neighbor with yaw, currently only with L2 distance of xyz
        closestVertex = None
        #TODO: Find maximum minimum distance according to map size
        minDist = 100000
        for vertex in vertices:
            dist = vertex.pos.getL2(q.pos)
            if dist < minDist:
                minDist = dist
                closestVertex=vertex
        return closestVertex
        
    def getTrajectory(self, start: Configuration, goal: Configuration) -> list[Configuration]:
        #TODO: Use linked list class as vertex, to efficiently find the trajectory from start to goal once found explained below
        vertices = set()
        start = time.time()
        nodesCount = 0
        while time.time() - start < self.maxTimeTaken and nodesCount < self.maxNodesExpanded:
            pos = np.random.random_integers(low=0,high=100, size=(3,1)) # Sample X,Y,Z space TODO:Change hardcoding of low and high 
            yaw = np.random.random_integers(low=0, high=180) # Sample yaw from 0 to pi
            q = Configuration(pos[0], pos[1], pos[2], yaw)
            qPrime = self.getClosestNeighbor(vertices, q)
            nodesCount +=1
            for conf in self.steering(qPrime, q):
                pos = conf.pos
                if self.map.checkCollision(pos):
                    hasCollision = True
                    break
            if hasCollision:
                continue
            vertices.add(q)
            #TODO: In the pseudocode given, add edge to the set but we can just connect two linked list together
            if q.pos.getL2(goal.pos) <= self.errorMargin:
                break
        #TODO: Idea is as long as goal is found, trace back the linked list and return that trajectory of configurations
        return []