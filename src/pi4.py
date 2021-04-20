import numpy as np
import threading
import time

import controls

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
        prnt(0,"Starting " + self.name)
        if self.threadID == 1: #robot thread
            self.robotThread()
        elif self.threadID == 2: #console thread
            self.consoleThread()
        prnt(0,"Exiting " + self.name)

    def robotThread(self):
        while self.rbt.sim.isOpen():
            #self.rbt.move("Linear", 0.5, 1, 1)
            self.rbt.move("Arc", -np.pi/2, 1.25, -2*np.pi)
            time.sleep(1)

    def consoleThread(self):
        pass
   
##-----------HELPER FUNCTIONS-----------##         
def prnt(type, msg):
    print("<" + time.strftime("%H:%M:%S",time.localtime()) + "> [" + msgTypes[type] + "]: " + msg)


##-----------MAIN-----------##
if __name__ == '__main__':
    SPASM = controls.robot();
    robotThreads = [pi4Thread(1, "Robot-Thread", SPASM)];
    try:
        [robotThreads[i].start() for i in range(np.size(robotThreads))]
        prnt(0,"Starting Simulation-Thread") 
        SPASM.sim.show()
        prnt(0,"Animation figure closed")
        prnt(0,"Exiting Simulation-Thread")
    except:
        prnt(-1, "One or more threads failed to start")
