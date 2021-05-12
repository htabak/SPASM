import numpy as np
import scipy.stats

global VARIANCE
[VARIANCE] = [50]
sensorNoise = lambda: np.random.random()*(-1 if np.random.random() < 0.5 else 1)/1000

# Source: http://ros-developer.com/2019/04/10/parcticle-filter-explained-with-python-code-from-scratch/
def update(rbt):
    global VARIANCE
    rbt.particles[0:rbt.N,1] = np.ones((1,rbt.N))  #Reset particle weights
    for i in range(4):
        rbt.snsrData[i] = rbt.sensors[i].getRange()
        for j in range(rbt.N):
            rbt.particles[j,1] *= scipy.stats.norm(rbt.particles[j,0].sensors[i].getRange(), VARIANCE).pdf(rbt.snsrData[i])

    rbt.particles[0:rbt.N,1] = rbt.particles[0:rbt.N,1] + np.ones((1,rbt.N))*1.e-300  #avoid round-off to zero

    #print(np.array([[rbt.particles[i,0].zeta[0:2], rbt.particles[i,1]] for i in range(rbt.N)]))
    rbt.particles[0:rbt.N,1] = rbt.particles[0:rbt.N,1] / sum(rbt.particles[0:rbt.N,1]) #Normalize weights
    #print(np.array([[rbt.particles[i,0].zeta[0:2], rbt.particles[i,1]] for i in range(rbt.N)]))
    #print()

def resample(rbt):
    pass

def systematic_resample(rbt):
    N = rbt.N
    weights = rbt.particles[0:rbt.N,1]
    positions = (np.arange(N) + np.random.random()) / N

    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N and j<N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes

def resample_from_index(rbt, indexes):
    rbt.particles[:,0] = rbt.particles[indexes,0]
    rbt.particles[:,1] = rbt.particles[indexes,1]
    rbt.particles[0:rbt.N,1] /= np.sum(rbt.particles[0:rbt.N,1])