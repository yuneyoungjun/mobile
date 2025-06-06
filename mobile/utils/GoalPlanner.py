import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from Robot import Robot

class GoalPlanner:
    def __init__(self, desX, desY, desTheta, robot : Robot):
        self.goal_x = desX
        self.goal_y = desY
        self.goal_theta = desTheta
        self.robot = robot

        self.K_rho = 0
        self.K_alpha = 0
        self.K_beta = 0

        self.mode = "FORWARD"

    def setParameter(self, k_r, k_a, k_b):
        self.K_rho = k_r
        self.K_alpha = k_a
        self.K_beta = k_b

    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi
    
    def setDirection(self):
        dx = self.goal_x - self.robot.x  # 북쪽 차이
        dy = self.goal_y - self.robot.y  # 서쪽 차이

        # 주의: arctan2(y, x)에서 y는 서쪽, x는 북쪽이 됨
        self.alpha = self.saturationRad(np.arctan2(dy, dx) - np.deg2rad(self.robot.theta))

        if abs(self.alpha) > np.pi / 2:
            self.mode = "REVERSE"
        else:
            self.mode = "FORWARD"

    def calculateVelocity(self):

        dx = self.goal_x - self.robot.x  # 북쪽 차이
        dy = self.goal_y - self.robot.y  # 서쪽 차이

        path_theta = np.arctan2(dy, dx)  # (y=서쪽, x=북쪽)
        if self.mode == "REVERSE":
            path_theta = self.saturationRad(path_theta + np.pi)

        rho = np.hypot(dx, dy)
        alpha = self.saturationRad(path_theta - np.deg2rad(self.robot.theta))
        beta = self.saturationRad(np.deg2rad(self.goal_theta) - path_theta)

        v = self.K_rho * rho
        w = self.K_alpha * alpha + self.K_beta * beta

        if self.mode == "REVERSE":
            v = -v

        if rho < 0.05:
            v = 0
            heading_error = self.saturationRad(np.deg2rad(self.goal_theta) - np.deg2rad(self.robot.theta))
            w = 1.0 * heading_error
            if heading_error<0.1*np.pi/180:
                w=0.0

        return v, w
