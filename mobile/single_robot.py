import numpy as np
import matplotlib.pyplot as plt
from utils.Robot import Robot
from utils.GoalPlanner import GoalPlanner

goal_x = 0
goal_y = 0
goal_theta = 90

robot = Robot(-3, -3, 0)
planner = GoalPlanner(goal_x, goal_y, goal_theta, robot)
planner.setParameter(3, 8, -1.5)
planner.setDirection()

total_time = 10
dt = 0.01

x_paths = []
y_paths = []

if(planner.mode == "FORWARD"): robot.drawRobot(color = 'g')
elif(planner.mode == "REVERSE"): robot.drawRobot(color = 'r')

for t in np.arange(0, total_time, dt):
        
    v, w = planner.calculateVelocity()
    robot.updatePoseByVelocity(v, w, dt)

    x_paths.append(robot.x)
    y_paths.append(robot.y)

robot = Robot(goal_x, goal_y, goal_theta)
robot.drawRobot(color='b')

plt.plot(x_paths, y_paths)

plt.xlabel('x Position')
plt.ylabel('y Position')
plt.title('Single-Robot Trajectory')
plt.axis('equal')
plt.xlim(-8, 8)
plt.ylim(-8, 8)
plt.grid()
plt.show()