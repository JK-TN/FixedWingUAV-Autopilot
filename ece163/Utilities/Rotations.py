"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to perform rotations by producing rotation matricies.
"""
import math
#import MatrixMath as np
from . import MatrixMath as np

def dcm2Euler(dcm):
    pv = -dcm[0][2]
    if pv<-1:#check for max -+1 bound
        pv = -1
    elif pv>1:
        pv = 1
    else:
        pass
    #calculate euler angles
    pitch  = math.asin(pv)
    roll = math.atan2(dcm[1][2], dcm[2][2])
    yaw = math.atan2(dcm[0][1],dcm[0][0])
    return yaw, pitch, roll

def euler2DCM(yaw, pitch, roll):
    dcm = [[0,0,0],[0,0,0],[0,0,0]]#use yaw pitch and roll to calculate direction cosine matrix
    dcm[0][0] = math.cos(pitch)*math.cos(yaw)
    dcm[0][1] = math.cos(pitch)*math.sin(yaw)
    dcm[0][2] = -math.sin(pitch)

    dcm[1][0] = math.sin(roll)*math.sin(pitch)*math.cos(yaw)-math.cos(roll)*math.sin(yaw)
    dcm[1][1] = math.sin(roll)*math.sin(pitch)*math.sin(yaw)+math.cos(roll)*math.cos(yaw)
    dcm[1][2] = math.sin(roll)*math.cos(pitch)

    dcm[2][0] = math.cos(roll)*math.sin(pitch)*math.cos(yaw)+math.sin(roll)*math.sin(yaw)
    dcm[2][1] = math.cos(roll)*math.sin(pitch)*math.sin(yaw)-math.sin(roll)*math.cos(yaw)
    dcm[2][2] = math.cos(roll)*math.cos(pitch)
    return dcm

def ned2enu(points):
    dcm = euler2DCM(math.pi/2,0,math.pi)#use euler rotations to create DCM for NED -> ENU
    point_ENU = np.multiply(dcm, np.transpose(points))
    return np.transpose(point_ENU)

# Lazy Test Harness -- test the code with known good examples
#if __name__ == "__main__":
#    import MatrixMath
#    print('hello')
#    dcm = euler2DCM(2.356,0.7854,1.571)
#    print(dcm)
#    print(dcm2Euler(dcm))
#    pt = [[1,0,2],[7.5,-3,-19.2]]
#    print(ned2enu(pt))

# HW1 Q4 code
#if __name__ == "__main__":
    #import MatrixMath
    #print("Calculating Part A")
    #e0 = [[0],[0],[0]]
    #w0 = [[1],[1],[1]]
    #m = [[1,0,0],[0,1,0],[0,0,1]]
    #dt = 0.1
    #a =np.add(e0,np.scalarMultiply(dt,np.multiply(m,w0)))
    #print(a)

    #print("Calculating Part B")
    #dcm0 = euler2DCM(0,0,0)
    #wx = [[0,-1,1],[1,0,-1],[-1,1,0]]
    #delta = np.scalarMultiply(-dt,np.multiply(wx,dcm0))
    #dcm1 = np.add(dcm0,delta)
    #y,p,r = dcm2Euler(dcm1)
    #b = [[r],[p],[y]]
    #print(b)

    #print("Calculating Part C")
    #s1 = math.sin(math.sqrt(3)*0.1)/math.sqrt(3)
    #s2 = (1-math.cos(math.sqrt(3)*0.1))/3
    #wx2 = np.multiply(wx,wx)
    #I = [[1,0,0],[0,1,0],[0,0,1]]
    #p1 = np.scalarMultiply(-s1,wx)
    #p2 = np.scalarMultiply(s2,wx2)
    #yb,pb,rb = dcm2Euler(np.multiply(np.add(I,np.add(p1,p2)),dcm0))
    #c = [[rb],[pb],[yb]]
    #print(c)

    #print("Calculating Part D")
    #e = [[0],[0],[0]]
    #dt = 0.001
    #for i in range(100):
        #m1 = [[1,math.sin(e[0][0])*math.tan(e[1][0]),math.cos(e[0][0])*math.tan(e[1][0])],[0,math.cos(e[0][0]),-math.sin([0][0])],[0,math.sin(e[0][0])/math.cos(e[1][0]),math.cos(e[0][0])/math.cos(e[1][0])]]
        #e = np.add(e, np.scalarMultiply(dt, np.multiply(m1, w0)))
    #print(e)
    #print("Compared with A")
    #print(np.subtract(e,a))
    #print("Compared with B")
    #print(np.subtract(e,b))
    #print("Compared with C")
    #print(np.subtract(e,c))