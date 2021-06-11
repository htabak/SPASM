import queue
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import pi4
import trajectory
#import gate
#import chemical
import particleFilter

##-----------ROBOT CLASS-----------##
class robot:
    def __init__ (self, zeta=[0,0,0], ts=0.01, N = 10, lite=False, DRAW_SENSORS=False):
        self.IS_LITE = lite
        self.zeta = zeta
        self.dim = [3,3]
        self.ts = ts
        #Pool Map
        self.poolPoints = np.array([[-1.5,-1.5],
                                    [-1.5,1.5],
                                    [1.5,1.5],
                                    [1.5,-1.5],
                                    [-1.5,-1.5]]);
        self.sensors = [ultrasonic(self,0.1715,-0.1874,-np.pi/4),
                        ultrasonic(self,-0.1715,-0.1874,-3*np.pi/4),
                        ultrasonic(self,-0.1715,0.1874,3*np.pi/4),
                        ultrasonic(self,0.1715,0.1874,np.pi/4)]
        self.snsrData = [-1,-1,-1,-1]
        if not lite:
            self.DRAW_SENSORS = DRAW_SENSORS      
            self.cmdQueue = queue.Queue()
            self.PF_params = [N, 1, 0]   #[NumberOfParticles, PoolScale, IndexOfBestParticle]
            self.particles = np.array([[robot(zeta=[np.random.uniform(-1.45,1.45),
                                                    np.random.uniform(-1.45,1.45),
                                                    self.zeta[2]], lite=True), 1] for i in range(N)])
            particleFilter.update(self)       
            self.sim = simulation(self, 80, 50)

    def move (self, type, vAvg, x, y):
        cur_zeta = self.zeta
        if type == "Arc":
            new_zeta = trajectory.getArc(x,y,0,vAvg,0,ts=self.ts)[0]
        elif type == "Linear":
            new_zeta = trajectory.getLine(x,y,0,vAvg,0,ts=self.ts)[0]

        trajLen = np.size(new_zeta,0)
        new_zeta[0:trajLen,2] = new_zeta[0:trajLen,2] + cur_zeta[2]
        new_zeta[0:trajLen,0:2] = trajectory.rotate(new_zeta[0:trajLen,0:2],cur_zeta[2])
        new_zeta[0:trajLen,0:2] = trajectory.translate(new_zeta[0:trajLen,0:2],cur_zeta[0:2])
        
        for i in range(trajLen): 
            delta = new_zeta[i,0:2] - self.zeta[0:2];
            #particleFilter.update(self)
            self.zeta[0:2] = new_zeta[i,0:2]#trajectory.translate(new_zeta[i,0:2],cur_zeta[0:2])
            self.zeta[2] = new_zeta[i,2]
     
            for j in range(self.PF_params[0]):
                self.particles[j,0].zeta[0:2] = self.particles[j,0].zeta[0:2] + delta[0:2]
                self.particles[j,0].zeta[2] = new_zeta[i,2]

            time.sleep(self.ts)
            if not self.sim.isOpen(): break

    def updateDistances (self):
        self.snsrData = [self.sensors[0].getRange(),self.sensors[1].getRange(),
                         self.sensors[2].getRange(),self.sensors[3].getRange()]

    def reset (self):
        pass

##-----------ULTRASONIC SENSOR CLASS-----------##
class ultrasonic:
    def __init__ (self, rbt, snsr_x, snsr_y, snsr_angle):
        self.rbt = rbt
        self.ray = np.array([[snsr_x,snsr_y],
                             [snsr_x+10*np.cos(snsr_angle),snsr_y+10*np.sin(snsr_angle)]])
        self.snsr_zeta = [snsr_x,snsr_y,snsr_angle]
        self.getRange()

    def getRange (self):
        ray = np.copy(self.ray)
        ray[1,0:2] = [self.snsr_zeta[0]+10*np.cos(self.snsr_zeta[2]),self.snsr_zeta[1]+10*np.sin(self.snsr_zeta[2])]
        a = trajectory.toWorldFrame(ray,self.rbt.zeta)
        c1 = trajectory.getIntersect(a[0,0:2],a[1,0:2],self.rbt.poolPoints[0,0:2],self.rbt.poolPoints[1,0:2])
        c2 = trajectory.getIntersect(a[0,0:2],a[1,0:2],self.rbt.poolPoints[1,0:2],self.rbt.poolPoints[2,0:2])
        c3 = trajectory.getIntersect(a[0,0:2],a[1,0:2],self.rbt.poolPoints[2,0:2],self.rbt.poolPoints[3,0:2])
        c4 = trajectory.getIntersect(a[0,0:2],a[1,0:2],self.rbt.poolPoints[3,0:2],self.rbt.poolPoints[4,0:2])
        c = trajectory.toRobotFrame(np.array([[c1[0],c1[1]],
                                              [c2[0],c2[1]],
                                              [c3[0],c3[1]],
                                              [c4[0],c4[1]]]), 
                                    self.rbt.zeta)
        b = np.array([]);
        for i in range(4):
            if round(abs(np.arctan2(c[i,1]-self.snsr_zeta[1],c[i,0]-self.snsr_zeta[0])/self.snsr_zeta[2]),2) == 1:
                #print(round(abs(np.arctan2(c[i,1]-self.snsr_zeta[1],c[i,0]-self.snsr_zeta[0])/self.snsr_zeta[2]),3))
                if np.shape(b)[0] == 0: b = np.array([c[i,0:2]]) 
                else: b = np.append(b,np.array([c[i,0:2]]),axis=0)
        b_len = np.size(b,0)
        if b_len == 0: 
            #if not self.rbt.IS_LITE: pi4.cout(-1,"Could not read sensor data")
            #for i in range(4): print(round(abs(np.arctan2(c[i,1]-self.snsr_zeta[1],c[i,0]-self.snsr_zeta[0])/self.snsr_zeta[2]),3))
            return -1
        elif b_len == 1:
            ray[1,0:2] = b
            min = trajectory.distance(ray[0,0],ray[0,1],ray[1,0],ray[1,1])
        else:
            ray[1,0:2] = b[0,0:2]
            min = trajectory.distance(ray[0,0],ray[0,1],ray[1,0],ray[1,1])
            for i in range(1,b_len):
                d = trajectory.distance(ray[0,0],ray[0,1],b[i,0],b[i,1])
                if d < min:
                    ray[1,0:2] = b[i,0:2]
                    min = d
        if not self.rbt.IS_LITE and (ray[1,0] < -3 or ray[1,0] > 3 or ray[1,1] < -3 or ray[1,1] > 3): 
            pi4.cout(-1,"Problematic sensor reading")
            #input("Press Enter to continue...")
            return -1
        self.ray = ray
        return min


##-----------SIMULATION CLASS-----------##
class simulation:
    def __init__ (self, rbt, ts, frames):
        self.rbt = rbt
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1,1,1)  
        #Pool Background Graphics
        self.poolGraphic = [self.ax1.fill(self.rbt.poolPoints[0:np.shape(self.rbt.poolPoints)[0],0],
                                         self.rbt.poolPoints[0:np.shape(self.rbt.poolPoints)[0],1],
                                         color='#8EE1FF')[0]]
        #Sensor Graphics
        if self.rbt.DRAW_SENSORS:
            snsr_ray = np.array([trajectory.toWorldFrame(self.rbt.sensors[i].ray,self.rbt.zeta) for i in range(4)])
            self.snsrGraphic = [self.ax1.plot(snsr_ray[0,0:2,0],
                                              snsr_ray[0,0:2,1],
                                              'b--')[0],
                                self.ax1.plot(snsr_ray[1,0:2,0],
                                              snsr_ray[1,0:2,1],
                                              'b--')[0],
                                self.ax1.plot(snsr_ray[2,0:2,0],
                                              snsr_ray[2,0:2,1],
                                              'b--')[0],
                                self.ax1.plot(snsr_ray[3,0:2,0],
                                              snsr_ray[3,0:2,1],
                                              'b--')[0],
                                self.ax1.plot(snsr_ray[0,1,0],
                                              snsr_ray[0,1,1],
                                              'bo')[0],
                                self.ax1.plot(snsr_ray[1,1,0],
                                              snsr_ray[1,1,1],
                                              'bo')[0],
                                self.ax1.plot(snsr_ray[2,1,0],
                                              snsr_ray[2,1,1],
                                              'bo')[0],
                                self.ax1.plot(snsr_ray[3,1,0],
                                              snsr_ray[3,1,1],
                                              'bo')[0]]
        #Robot Graphics
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
        cur_rbtPoints = trajectory.toWorldFrame(self.rbtPoints,self.rbt.zeta)
        cur_rbtFrame = trajectory.toWorldFrame(self.rbtFrame,self.rbt.zeta)
        self.rbtGraphic = [self.ax1.fill(cur_rbtPoints[3:np.shape(self.rbtPoints)[0],0],
                                         cur_rbtPoints[3:np.shape(self.rbtPoints)[0],1],
                                         facecolor='grey')[0],
                           self.ax1.plot(cur_rbtPoints[0:np.shape(self.rbtPoints)[0],0],
                                         cur_rbtPoints[0:np.shape(self.rbtPoints)[0],1],
                                         'k')[0],
                           self.ax1.plot(cur_rbtFrame[4:np.shape(self.rbtFrame)[0],0],
                                         cur_rbtFrame[4:np.shape(self.rbtFrame)[0],1],
                                         'b',linewidth=3)[0],
                           self.ax1.plot(cur_rbtFrame[0:5,0],
                                         cur_rbtFrame[0:5,1],
                                         'r',linewidth=3)[0]];
        #Particle Filter Graphics
        self.PFPoints = np.array([[self.rbt.particles[i,0].zeta[0],self.rbt.particles[i,0].zeta[1]] for i in range(self.rbt.PF_params[0])])
        self.PFGrapic = [self.ax1.plot(self.PFPoints[0:self.rbt.PF_params[0],0],self.PFPoints[0:self.rbt.PF_params[0],1],
                                       'ow')[0]]
        #Pool Foreground Graphics
        self.baseFrame = np.array([[0.07,0.015],
                                  [0.1,0],
                                  [0.07,-0.015],
                                  [0.1,0],
                                  [0,0],
                                  [0,0.1],
                                  [-0.015,0.07],
                                  [0,0.1],
                                  [0.015,0.07]])*1.5;
        self.poolGraphic.append([self.ax1.plot(self.rbt.poolPoints[0:np.shape(self.rbt.poolPoints)[0],0],
                                              self.rbt.poolPoints[0:np.shape(self.rbt.poolPoints)[0],1],
                                              'k',linewidth=2)[0],
                                self.ax1.plot(self.baseFrame[4:np.shape(self.baseFrame)[0],0],
                                              self.baseFrame[4:np.shape(self.baseFrame)[0],1],
                                              'b',linewidth=3)[0],
                                self.ax1.plot(self.baseFrame[0:5,0],
                                              self.baseFrame[0:5,1],
                                              'r',linewidth=3)[0]]);
        #Plot Details
        self.ax1.set_xlim([-1.7,1.7]); self.ax1.set_ylim([-1.7,1.7]);
        #self.ax1.set_xlim([-3,3]); self.ax1.set_ylim([-3,3]);
        self.ax1.set_aspect('equal')
        self.ax1.grid()
        self.rbtAnimation = FuncAnimation(self.fig, self.nextFrame, ts*frames, interval=ts)
    
    def nextFrame(self,j):  
        if self.rbt.DRAW_SENSORS:
            snsr_ray = np.array([trajectory.toWorldFrame(self.rbt.sensors[i].ray,self.rbt.zeta) for i in range(4)])
        cur_rbtPoints = trajectory.toWorldFrame(self.rbtPoints,self.rbt.zeta)
        cur_rbtFrame = trajectory.toWorldFrame(self.rbtFrame,self.rbt.zeta)
        self.PFPoints = np.array([[self.rbt.particles[i,0].zeta[0],self.rbt.particles[i,0].zeta[1]] for i in range(self.rbt.PF_params[0])])
        #Update Graphics
        if self.rbt.DRAW_SENSORS:
            for i in range(4): 
                self.snsrGraphic[i].set_data(snsr_ray[i,0:2,0],snsr_ray[i,0:2,1]) 
                self.snsrGraphic[i+4].set_data(snsr_ray[i,1,0],snsr_ray[i,1,1]) 
        cur_rbtPointsLen = np.shape(cur_rbtPoints)[0]
        cur_rbtFrameLen = np.shape(cur_rbtFrame)[0]
        self.rbtGraphic[0].xy = cur_rbtPoints[3:cur_rbtPointsLen,0:2]
        self.rbtGraphic[1].set_data(cur_rbtPoints[0:cur_rbtPointsLen,0],
                                    cur_rbtPoints[0:cur_rbtPointsLen,1])
        self.rbtGraphic[2].set_data(cur_rbtFrame[4:cur_rbtFrameLen,0],
                                    cur_rbtFrame[4:cur_rbtFrameLen,1])
        self.rbtGraphic[3].set_data(cur_rbtFrame[0:5,0],
                                    cur_rbtFrame[0:5,1])
        self.PFGrapic[0].set_data(self.PFPoints[0:self.rbt.PF_params[0],0],
                               self.PFPoints[0:self.rbt.PF_params[0],1])

        return [self.rbtGraphic]

    def show(self):
        plt.show()

    def isOpen(self):
        return plt.get_fignums()


##-----------MAIN-----------##
if __name__ == '__main__':
    SPASM = robot(DRAW_SENSORS = True)
    SPASM.sim.show()