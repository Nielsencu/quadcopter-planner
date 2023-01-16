from RRT_algorithms.rrt_max import RRTPlanner, GridMap, Configuration
import matplotlib.pyplot as plt
import numpy as np
import math

bm = 1 #benchmark

if __name__ == "__main__":
    grid = GridMap(100,100,100)
    grid.addObstacles(25,25,25, radius=10)
    grid.addObstacles(70,70,70, radius=5)
    grid.addObstacles(50,50,50, radius=10)
    startConfig = Configuration(0,0,0)
    planner = RRTPlanner(grid)
    goalConfig = Configuration(80,80,80, np.pi)
    ax = plt.axes(projection='3d')
    x, y, z = grid.getMap().nonzero()
    ax.scatter(x,y,z,c=z, alpha=1)
    nodes_countinitreg = 0
    shortestpathlistreg = np.empty([bm,5000])
    for i in range(bm):
        traj,nodescount,shortestpath = planner.getTrajectory(startConfig, goalConfig, ax=ax)
        if ax is not None:
            y = np.array([point.pos.y for point in traj])
            x = np.array([point.pos.x for point in traj])
            z = np.array([point.pos.z for point in traj])
            ax.plot(x, y, z, '-r')
            #print(f'Trajectory generated is:\n{[conf.pos for conf in traj]}')
            plt.show()
            if nodescount != math.inf:
                for j in range(len(nodescount)):
                    #nodescountlistreg[i,j] = nodescount[j]
                    shortestpathlistreg[i,j] = shortestpath[j]
                    if len(nodescount) > nodes_countinitreg:
                        nodes_countinitreg = len(nodescount)
                        #print(nodescount_initreg)
                                     
    nodescountreg = range(1,nodes_countinitreg)   

    shortestpathlistreg = shortestpathlistreg[:,1:nodes_countinitreg]


from rrt_star1_max import RRTPlanner, GridMap, Configuration
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    grid = GridMap(100,100,100)
    grid.addObstacles(25,25,25, radius=10)
    grid.addObstacles(70,70,70, radius=5)
    grid.addObstacles(50,50,50, radius=10)
    startConfig = Configuration(0,0,0)
    planner = RRTPlanner(grid)
    goalConfig = Configuration(80,80,80, np.pi)
    ax = plt.axes(projection='3d')
    x, y, z = grid.getMap().nonzero()
    ax.scatter(x,y,z,c=z, alpha=1)
    nodes_countinitstar = 0
    shortestpathliststar = np.empty([bm,5000])
    for i in range(bm):
        traj,nodescount,shortestpath = planner.getTrajectory(startConfig, goalConfig, ax=ax)
        if ax is not None:
            y = np.array([point.pos.y for point in traj])
            x = np.array([point.pos.x for point in traj])
            z = np.array([point.pos.z for point in traj])
            ax.plot(x, y, z, '-r')
            #print(f'Trajectory generated is:\n{[conf.pos for conf in traj]}')
            plt.show()
            if nodescount != math.inf:
                for j in range(len(nodescount)):
                    #nodescountlistreg[i,j] = nodescount[j]
                    shortestpathliststar[i,j] = shortestpath[j]
                    if len(nodescount) > nodes_countinitstar:
                        nodes_countinitstar = len(nodescount)
                                          
    nodescountstar = range(1,nodes_countinitstar)   
    shortestpathliststar = shortestpathliststar[:,1:nodes_countinitstar]
    
    
from RRT_algorithms.informed_rrt_star import RRTPlanner, GridMap, Configuration
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    grid = GridMap(100,100,100)
    grid.addObstacles(25,25,25, radius=10)
    grid.addObstacles(70,70,70, radius=5)
    grid.addObstacles(50,50,50, radius=10)
    startConfig = Configuration(0,0,0)
    planner = RRTPlanner(grid)
    goalConfig = Configuration(80,80,80, np.pi)
    ax = plt.axes(projection='3d')
    x, y, z = grid.getMap().nonzero()
    ax.scatter(x,y,z,c=z, alpha=1)
    #nodescountlistinf = []
    #shortestpathlistinf = []
    nodes_countinitinf = 0
    shortestpathlistinf = np.empty([bm,5000])
    for i in range(bm):
        traj,nodescount,shortestpath = planner.getTrajectory(startConfig, goalConfig, ax=ax)
        if ax is not None:
            y = np.array([point.pos.y for point in traj])
            x = np.array([point.pos.x for point in traj])
            z = np.array([point.pos.z for point in traj])
            ax.plot(x, y, z, '-r')
            #print(f'Trajectory generated is:\n{[conf.pos for conf in traj]}')
            plt.show()
            if nodescount != math.inf:
                for j in range(len(nodescount)):
                    #nodescountlistreg[i,j] = nodescount[j]
                    shortestpathlistinf[i,j] = shortestpath[j]
                    if len(nodescount) > nodes_countinitinf:
                        nodes_countinitinf = len(nodescount)
                        #print(nodes_countinitinf)
                        
    nodescountinf = range(1,nodes_countinitinf)   
    shortestpathlistinf = shortestpathlistinf[:,1:nodes_countinitinf]
    
shortestpathlistreg = np.mean(shortestpathlistreg, axis = 0)    
shortestpathliststar = np.mean(shortestpathliststar, axis = 0)  
shortestpathlistinf = np.mean(shortestpathlistinf, axis = 0)  

#print(nodescountlistreg[1:], shortestpathlistreg[1:])
#print(nodescountliststar, shortestpathliststar)   
#print(nodescountlistinf[0,1:], shortestpathlistinf[0,1:])
print(shortestpathlistinf)

plt.plot(nodescountreg, shortestpathlistreg ,'r')
plt.plot(nodescountstar, shortestpathliststar, 'g')
plt.plot(nodescountinf, shortestpathlistinf, 'b')
plt.xlim(0,10)
plt.ylim(0,30)
plt.show()