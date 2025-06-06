import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def updatePoseByVelocity(self, v, w, dt):
        self.x += v * np.cos(np.deg2rad(self.theta)) * dt
        self.y += v * np.sin(np.deg2rad(self.theta)) * dt
        self.theta += np.rad2deg(w) * dt

    def drawRobot(self, line_length = 1, color = 'r'):

        dx = line_length * np.cos(np.deg2rad(self.theta))
        dy = line_length * np.sin(np.deg2rad(self.theta))

        plt.plot(self.x, self.y, 'o', markersize = 15, color=color, alpha = 0.3, zorder=5)
        plt.arrow(self.x, self.y, dx, dy, head_width=0, head_length=0, fc=color, ec=color,linewidth=1.5, zorder=5)
        