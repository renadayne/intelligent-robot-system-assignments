import math
import numpy as np
import matplotlib.pyplot as plt

class AckermanRobot:
    def __init__(self, L, r, dT=0.1):
        self.L = L
        self.r = r
        self.dT = dT
        self.v = 0
        self.w = 0
        self.R = 0
        self.px = 0
        self.py = 0
        self.theta = 0
        self.tracker = np.array([[self.v, self.w, self.R, self.px, self.py, self.theta]])

    def setVelocity(self, vw, phi):
        self.R = 0 if math.tan(phi) == 0 else abs(self.L/math.tan(phi))
        self.v = vw*self.r/2
        self.w = self.v*math.tan(phi)/self.L

    def setDeltaT(self, dT):
        self.dT = dT

    def appendTracker(self):
        self.tracker = np.append(self.tracker,[[self.v, self.w, self.R, self.px, self.py, self.theta]], axis=0)

    def nextPos(self):
        self.px = self.px + self.v*self.dT*math.cos((self.theta + self.theta + self.w*self.dT)/2)
        self.py = self.py + self.v*self.dT*math.sin((self.theta + self.theta + self.w*self.dT)/2)
        self.theta = self.theta + self.w*self.dT
        self.appendTracker()

    def setPath(self, path):
        self.path = path
        self.path_idx = 0

    def followPath(self):
        if self.path_idx < len(self.path):
            x, y = self.path[self.path_idx]
            dx = x - self.px
            dy = y - self.py
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.path_idx += 1
                return
            phi = math.atan2(dy, dx) - self.theta
            vw = math.sqrt(dx*dx + dy*dy)
            self.setVelocity(vw, phi)
            self.nextPos()

    def run(self):
        while self.path_idx < len(self.path):
            self.followPath()

        plt.plot(self.tracker[:,3], self.tracker[:,4], '-b')
        plt.axis('equal')
        plt.show()

robot = AckermanRobot(0.5, 0.1, 0.05)
path = [(0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]
robot.setPath(path)
robot.run()
