import numpy as np
import threading
import time

import controls
import particleFilter

msgTypes = {
    -1: "ERROR",
    0: "PI4",
    1: "ROBOT",
    2: "GATE",
    3: "CHEMICAL",
    4: "FILTER",
}

##-----------RUN THREADS-----------##
class pi4Thread (threading.Thread):
    def __init__(self, threadID, name, rbt):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rbt = rbt
    def run(self):
        cout(0,"Starting " + self.name)
        if self.threadID == 1: #robot thread
            self.robotThread()
        elif self.threadID == 2: #console thread
            self.sensorThread()
        cout(0,"Exiting " + self.name)

    def robotThread(self):
        self.rbt.move("Linear", 0.3, -1, 0)
        self.rbt.move("Arc", -np.pi/8, 0, -np.pi/2)
        while self.rbt.sim.isOpen():
            self.rbt.move("Arc", -np.pi/3, 1, -2*np.pi)
            self.rbt.move("Linear", 0.3, 2, 0)
            self.rbt.move("Arc", np.pi/8, 0, np.pi/2)
            self.rbt.move("Arc", np.pi/3, 1, 2*np.pi)
            self.rbt.move("Linear", 0.3, -2, 0)
            self.rbt.move("Arc", -np.pi/8, 0, -np.pi/2)
            time.sleep(1)

    def sensorThread(self):
        while self.rbt.sim.isOpen():
            particleFilter.update(self.rbt)
            particleFilter.resample(self.rbt)
            #pi4.cout(1,"Sensor Readings: [" + str(snsrData[0]) + ", " + str(snsrData[1]) + ", " + str(snsrData[2]) + ", " + str(snsrData[3]) + "]")
            time.sleep(self.rbt.ts)
   
##-----------HELPER FUNCTIONS-----------##         
def cout(type, msg):
    print("<" + time.strftime("%H:%M:%S",time.localtime()) + "> [" + msgTypes[type] + "]: " + msg)


##-----------MAIN-----------##
if __name__ == '__main__':
    SPASM = controls.robot(DRAW_SENSORS = True);
    robotThreads = [pi4Thread(1, "Robot-Thread", SPASM),
                    pi4Thread(2, "Sensor-Thread", SPASM)];
    try:
        [robotThreads[i].start() for i in range(np.size(robotThreads))]
        cout(0,"Starting Simulation-Thread") 
        SPASM.sim.show()   
    except:
        cout(-1, "One or more threads failed to start")
    cout(0,"Animation figure closed")
    cout(0,"Exiting Simulation-Thread")
    try:
        [robotThreads[i].join() for i in range(np.size(robotThreads))]
    except:
        cout(-1, "Failed to join one or more threads")