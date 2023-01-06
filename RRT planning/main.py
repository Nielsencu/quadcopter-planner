from graphrrt import RRTPlanner, GridMap, Configuration
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    grid = GridMap(100,100,100)
    grid.addObstacles(25,25,25, radius=10)
    grid.addObstacles(70,70,70, radius=10)
    grid.addObstacles(50,50,50, radius=10)
    startConfig = Configuration(0,0,0)
    planner = RRTPlanner(grid)
    goalConfig = Configuration(80,80,80, np.pi)
    ax = plt.axes(projection='3d')
    x, y, z = grid.getMap().nonzero()
    ax.scatter(x,y,z,c=z, alpha=1)
    traj = planner.getTrajectory(startConfig, goalConfig, ax=ax)
    if ax is not None:
        y = np.array([point.pos.y for point in traj])
        x = np.array([point.pos.x for point in traj])
        z = np.array([point.pos.z for point in traj])
        ax.plot(x, y, z, '-r')
    #print(f'Trajectory generated is:\n{[conf.pos for conf in traj]}')
    plt.show()