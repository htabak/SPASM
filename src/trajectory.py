import numpy as np

##-----------TRAJECTORY FUNCTIONS-----------##
def getArc(R,theta,w0,wAvg,wf,a0=0,af=0,ts=0.01):
    if theta == 0 or wAvg == 0: return np.array([[0,0,0]]),0,0
    N = int((theta/wAvg)/ts + 1)
    if N <= 0: return np.array([[0,0,0]]),0,0

    t = np.linspace(0,N*ts,N)
    A = quanticSolve(N*ts,0,theta,w0,wf,a0,af) 
    th = A[0] + A[1]*t + A[2]*np.power(t,2) + A[3]*np.power(t,3) + A[4]*np.power(t,4) + A[5]*np.power(t,5)
    if theta < 0: xeta = np.array([[R*(1 - np.cos(-th[i])),R*np.sin(-th[i]),th[i]] for i in range(np.size(th))])
    else: xeta = np.array([[R*(np.cos(th[i]) - 1),R*np.sin(th[i]),th[i]] for i in range(np.size(th))])

    w = 0; dw = 0;
    #w = A[1] + 2*A[2]*t + 3*A[3]*np.power(t,2) + 4*A[4]*np.power(t,3) + 5*A[5]*np.power(t,4)
    #dw = 2*A[2] + 6*A[3]*t + 12*A[4]*np.power(t,2) + 20*A[5]*np.power(t,3)
    return xeta, w, dw

def getLine(x,y,v0,vAvg,vf,a0=0,af=0,ts=0.01):
    if (y == 0 and x == 0) or vAvg == 0: return np.array([[0,0,0]]),0,0

    theta = np.arctan2(y,x)
    if theta < np.pi/2: xeta = getArc(0,theta-np.pi/2,0,-np.pi/8,0)[0]
    else: xeta = getArc(0,theta-np.pi/2,0,np.pi/8,0)[0]

    d = np.sqrt(np.power(x,2) + np.power(y,2))
    N = int((d/vAvg)/ts + 1)
    if N <= 0: return np.array([[0,0,0]]),0,0

    t = np.linspace(0,N*ts,N)
    A = quanticSolve(N*ts,0,d,v0,vf,a0,af) 
    s = A[0] + A[1]*t + A[2]*np.power(t,2) + A[3]*np.power(t,3) + A[4]*np.power(t,4) + A[5]*np.power(t,5)

    xeta = np.append(xeta,np.array([[s[i]*np.cos(theta),s[i]*np.sin(theta),theta-np.pi/2] for i in range(np.size(s))]), axis=0)

    ds = 0; dds = 0;
    #ds = A[1] + 2*A[2]*t + 3*A[3]*np.power(t,2) + 4*A[4]*np.power(t,3) + 5*A[5]*np.power(t,4)
    #dds = 2*A[2] + 6*A[3]*t + 12*A[4]*np.power(t,2) + 20*A[5]*np.power(t,3)
    return xeta, ds, dds    

##-----------LINEAR SOLVER FUNCTIONS-----------##
def quanticSolve(tf,x0,xf,v0,vf,a0,af):
    A = np.array([[1,0,0,0,0,0],
                  [1,tf,np.power(tf,2),np.power(tf,3),np.power(tf,4),np.power(tf,5)],
                  [0,1,0,0,0,0],
                  [0,1,2*tf,3*np.power(tf,2),4*np.power(tf,3),5*np.power(tf,4)],
                  [0,0,2,0,0,0],
                  [0,0,2,6*tf,12*np.power(tf,2),20*np.power(tf,3)]])
    A = np.dot(np.linalg.inv(A),([x0,xf,v0,vf,a0,af]))
    return A

##-----------HELPER FUNCTIONS-----------##
def reflect(path, axis):
	if axis < 0 or axis > 1: return -1
	new_path = np.copy(path)
	for i in range(np.shape(path)[0]): new_path[i,axis] = -new_path[i,axis]
	return new_path

def delay(path, index_delay):
	return np.roll(path, -index_delay, 0)

def translate(path, P = [0,0]):
	return path + P

def rotate(path, gamma):
	Rz = np.array([[np.cos(gamma),-np.sin(gamma)],
				   [np.sin(gamma),np.cos(gamma)]])
	new_path = np.copy(path)
	for i in range(np.shape(path)[0]):
		new_path[i,0:2] = np.transpose(np.dot(Rz, np.transpose(path[i,0:2])))
	return new_path