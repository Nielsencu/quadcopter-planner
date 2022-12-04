from rrt import RRTPlanner, GridMap, Configuration, Point3D
import matplotlib.pyplot as plt

if __name__ == "__main__":
    grid = GridMap(100,100,100)
    grid.addObstacles(Point3D(10,10,10), radius=5)
    startConfig = Configuration(0,0,0)
    planner = RRTPlanner(grid)
    goalConfig = Configuration(80,80,80)
    planner.getTrajectory(startConfig, goalConfig)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    z,x,y = grid.getMap().nonzero()
    ax.scatter(x,y,z,c=z, alpha=1)
    plt.show()