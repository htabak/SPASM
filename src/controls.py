import queue
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import pi4
import trajectory
import gate
import chemical
import controls

##-----------ROBOT CLASS-----------##
class robot:
    def __init__ (self, ts=0.01):
        self.xeta = [0,0,0]
        self.dim = [3,3]
        self.cmdQueue = queue.Queue()
        self.sim = simulation(self, 80, 50)
        self.ts = ts

        #Pool Map
        self.poolPoints = np.array([[]]);

    def move (self, type, vAvg, x, y):
        cur_xeta = self.xeta
        if type == "Arc":
            new_xeta = trajectory.getArc(x,y,0,vAvg,0)[0]
        elif type == "Linear":
            new_xeta = trajectory.getLine(x,y,0,vAvg,0)[0]

        new_xeta[0:np.size(new_xeta,0),2] = new_xeta[0:np.size(new_xeta,0),2] + cur_xeta[2]
        new_xeta[0:np.size(new_xeta,0),0:2] = trajectory.rotate(new_xeta[0:np.size(new_xeta,0),0:2],cur_xeta[2])
        new_xeta[0:np.size(new_xeta,0),0:2] = trajectory.translate(new_xeta[0:np.size(new_xeta,0),0:2],cur_xeta[0:2])
        
        for i in range(np.size(new_xeta,0)):
            self.xeta[0:2] = new_xeta[i,0:2]#trajectory.translate(new_xeta[i,0:2],cur_xeta[0:2])
            self.xeta[2] = new_xeta[i,2]
            time.sleep(self.ts)

    def localize (self):
        pass

    def getDistances (self):
        pass

    def reset (self):
        pass

##-----------SIMULATION CLASS-----------##
class simulation:
    def __init__ (self, rbt, ts, frames):
        self.rbt = rbt
        #Set up figure and plots
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1,1,1)
        self.ax1.set_xlim([-0.5,3]); self.ax1.set_ylim([-1.5,1.5]);
        
        #Robot polygon
        self.rbtPoints = np.array([[-0.0635,-0.2064],
                                   [-0.0635,-0.2064+0.1016],
                                   [0.0635,-0.2064+0.1016],
                                   [0.0635,-0.2064],
                                   [0.0505,-0.2064-0.0125],
                                   [0.0351,-0.2064-0.0220],
                                   [0.0180,-0.2064-0.0278],
                                   [0,-0.2064-0.0298],
                                   [-0.0180,-0.2064-0.0278],
                                   [-0.0351,-0.2064-0.0220],
                                   [-0.0505,-0.2064-0.0125],
                                   [-0.0635,-0.2064],
                                   [-0.1524,-0.2064],
                                   [-0.1905,-0.1683],
                                   [-0.1905,0.1683],
                                   [-0.1524,0.2064],
                                   [0.1524,0.2064],
                                   [0.1905,0.1683],
                                   [0.1905,-0.1683],
                                   [0.1524,-0.2064],
                                   [0.0635,-0.2064]]);
        self.rbtFrame = np.array([[0.07,0.015],
                                  [0.1,0],
                                  [0.07,-0.015],
                                  [0.1,0],
                                  [0,0],
                                  [0,0.1],
                                  [-0.015,0.07],
                                  [0,0.1],
                                  [0.015,0.07]])*1.5;
        self.rbtGraphic = [self.ax1.fill(self.rbtPoints[3:np.shape(self.rbtPoints)[0],0],
                                         self.rbtPoints[3:np.shape(self.rbtPoints)[0],1],
                                         facecolor='grey')[0],
                           self.ax1.plot(self.rbtPoints[3:np.shape(self.rbtPoints)[0],0],
                                         self.rbtPoints[3:np.shape(self.rbtPoints)[0],1],
                                         'k')[0],
                           self.ax1.plot(self.rbtFrame[4:np.shape(self.rbtFrame)[0],0],
                                         self.rbtFrame[4:np.shape(self.rbtFrame)[0],1],
                                         'b',linewidth=3)[0],
                           self.ax1.plot(self.rbtFrame[0:5,0],
                                         self.rbtFrame[0:5,1],
                                         'r',linewidth=3)[0]];

        #Pool Map
        self.baseFrame = np.array([[0.07,0.015],
                                  [0.1,0],
                                  [0.07,-0.015],
                                  [0.1,0],
                                  [0,0],
                                  [0,0.1],
                                  [-0.015,0.07],
                                  [0,0.1],
                                  [0.015,0.07]])*1.5;
        self.poolGrahic = [self.ax1.plot(self.baseFrame[4:np.shape(self.baseFrame)[0],0],
                                         self.baseFrame[4:np.shape(self.baseFrame)[0],1],
                                         'b',linewidth=3)[0],
                           self.ax1.plot(self.baseFrame[0:5,0],
                                         self.baseFrame[0:5,1],
                                         'r',linewidth=3)[0]];

        self.ax1.set_aspect('equal')
        self.ax1.grid()
        self.rbtAnimation = FuncAnimation(self.fig, self.nextFrame, ts*frames, interval=ts)
    
    def nextFrame(self,j):   
        self.cur_rbtPoints = trajectory.rotate(self.rbtPoints,self.rbt.xeta[2])
        self.cur_rbtPoints = trajectory.translate(self.cur_rbtPoints, self.rbt.xeta[0:2])
        self.cur_rbtFrame = trajectory.rotate(self.rbtFrame,self.rbt.xeta[2])
        self.cur_rbtFrame = trajectory.translate(self.cur_rbtFrame, self.rbt.xeta[0:2])
        #print(self.cur_rbtFrame[4,0:2])
        self.rbtGraphic[0].xy = self.cur_rbtPoints
        self.rbtGraphic[1].set_data(self.cur_rbtPoints[0:np.shape(self.rbtPoints)[0],0],
                                    self.cur_rbtPoints[0:np.shape(self.rbtPoints)[0],1])
        self.rbtGraphic[2].set_data(self.cur_rbtFrame[4:np.shape(self.rbtFrame)[0],0],
                                    self.cur_rbtFrame[4:np.shape(self.rbtFrame)[0],1])
        self.rbtGraphic[3].set_data(self.cur_rbtFrame[0:5,0],
                                    self.cur_rbtFrame[0:5,1])
        return [self.rbtGraphic]

    def show(self):
        plt.show()

    def isOpen(self):
        return plt.get_fignums()


##-----------MAIN-----------##
if __name__ == '__main__':
    SPASM = robot()
    SPASM.sim.show()