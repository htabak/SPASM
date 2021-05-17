import numpy as np
from scipy.stats import norm

MIN_POOL_SCALE = 0.005
MIN_WEIGHT = 0.1
EXPAND_RATE = 1.05
NARROW_RATE = 0.9
PF_TYPE = ["SENSOR","POSITION"][0] #Pick Particle Filter Type

sensorNoise = lambda: np.random.random()*(-1 if np.random.random() < 0.5 else 1)/1000

def update(rbt):
    global PF_TYPE

    rbt.particles[0:rbt.PF_params[0],1] = np.ones((1,rbt.PF_params[0]))  #Reset particle weights
    maxWeight = 0
    
    rbt.snsrData = [rbt.sensors[0].getRange(),rbt.sensors[1].getRange(),
                    rbt.sensors[2].getRange(),rbt.sensors[3].getRange()]

    if PF_TYPE == "SENSOR":
        for i in range(rbt.PF_params[0]):
            snsrData = [rbt.particles[i,0].sensors[0].getRange(),rbt.particles[i,0].sensors[1].getRange(),
                        rbt.particles[i,0].sensors[2].getRange(),rbt.particles[i,0].sensors[3].getRange()]
            rbt.particles[i,1] *= norm.pdf(snsrData[0], rbt.snsrData[0])
            rbt.particles[i,1] *= norm.pdf(snsrData[1], rbt.snsrData[1])
            rbt.particles[i,1] *= norm.pdf(snsrData[2], rbt.snsrData[2])
            rbt.particles[i,1] *= norm.pdf(snsrData[3], rbt.snsrData[3])
            if rbt.particles[i,1] > maxWeight:
                maxWeight = rbt.particles[i,1]
                rbt.PF_params[2] = i
    if PF_TYPE == "POSITION":
        for j in range(rbt.PF_params[0]):
            rbt.particles[j,1] *= norm.pdf(rbt.particles[j,0].zeta[0], rbt.zeta[0])
            rbt.particles[j,1] *= norm.pdf(rbt.particles[j,0].zeta[1], rbt.zeta[1])
            if rbt.particles[j,1] > maxWeight:
                maxWeight = rbt.particles[j,1]
                rbt.PF_params[2] = j
    
    rbt.particles[0:rbt.PF_params[0],1] = rbt.particles[0:rbt.PF_params[0],1] + np.ones((1,rbt.PF_params[0]))*1.e-300  #avoid round-off to zero
    rbt.particles[0:rbt.PF_params[0],1] = rbt.particles[0:rbt.PF_params[0],1] / sum(rbt.particles[0:rbt.PF_params[0],1]) #Normalize weights


def resample(rbt):
    global MIN_POOL_SCALE, MIN_WEIGHT, EXPAND_RATE, NARROW_RATE

    if rbt.PF_params[1] <= 1 and rbt.particles[rbt.PF_params[2],1] <= MIN_WEIGHT:
        rbt.PF_params[1] *= EXPAND_RATE
    elif rbt.PF_params[1] > MIN_POOL_SCALE:
        rbt.PF_params[1] *= NARROW_RATE
    #print(rbt.particles[rbt.PF_params[2],1])

    sampleOrigin = rbt.particles[rbt.PF_params[2],0].zeta[0:2]
    xlims = [sampleOrigin[0] - 1.45*rbt.PF_params[1], sampleOrigin[0] + 1.45*rbt.PF_params[1]]
    ylims = [sampleOrigin[1] - 1.45*rbt.PF_params[1], sampleOrigin[1] + 1.45*rbt.PF_params[1]]

    for i in range(rbt.PF_params[0]): rbt.particles[i,0].zeta = [np.random.uniform(xlims[0],xlims[1]),
                                                      np.random.uniform(ylims[0],ylims[1]),
                                                      rbt.zeta[2]]
