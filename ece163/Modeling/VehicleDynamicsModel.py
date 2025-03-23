"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to perform forward integration of vehicle state using derivation methods to step the vehicle through time.
"""

import math
from ctypes.wintypes import SMALL_RECT

from fontTools.misc.psOperators import ps_string

from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel:
    def __init__(self,dT=VPC.dT):
        
        #Setting timestep and initializing objects
        self.dT = dT
        self.state = States.vehicleState()
        self.dot = States.vehicleState()

        #pull DCM of state
        r = self.state.R
        #Find Determinant
        det = r[0][0] * ((r[1][1] * r[2][2]) - (r[1][2] * r[2][1])) - r[0][1] * (
                (r[1][0] * r[2][2]) - (r[1][2] * r[2][1])) + r[0][2] * ((r[1][0] * r[2][1]) - (r[1][1] * r[2][0]))
        
        #if DCM is not a valid rotation matrix use euler angles to solve for DCM
        if abs(abs(det) - 1) > 0.001:
            self.state.R = Rotations.euler2DCM(self.state.yaw, self.state.pitch, self.state.roll)
        #otherwise update euler based on rotation matrix
        else:
            self.state.yaw, self.state.pitch, self.state.roll = Rotations.dcm2Euler(r)


    def reset(self):
        
        #reinitialize state and derivative objects
        self.state = States.vehicleState()
        self.dot = States.vehicleState()

        #pull DCM of state
        r = self.state.R
        #Find Determinant
        det = r[0][0] * ((r[1][1] * r[2][2]) - (r[1][2] * r[2][1])) - r[0][1] * (
                (r[1][0] * r[2][2]) - (r[1][2] * r[2][1])) + r[0][2] * ((r[1][0] * r[2][1]) - (r[1][1] * r[2][0]))
        
        #if DCM is not a valid rotation matrix use euler angles to solve for DCM
        if abs(abs(det) - 1) > 0.001:
            self.state.R = Rotations.euler2DCM(self.state.yaw, self.state.pitch, self.state.roll)
        #otherwise update euler based on rotation matrix
        else:
            self.state.yaw, self.state.pitch, self.state.roll = Rotations.dcm2Euler(r)

    def getVehicleState(self):
        #return current state
        return self.state

    def setVehicleState(self,state):
        #updating current state
        self.state = state

    def getVehicleDerivative(self):
        #returning current derivative
        return self.dot

    def setVehicleDerivative(self,dot):
        #updating current derivative
        self.dot = dot

    def Update(self,forcesMoments):
        #find new derivative and update it
        self.dot = self.derivative(self.getVehicleState(),forcesMoments)
        
        #integrate forward one timestep and update current state
        self.state = self.IntegrateState(self.dT,self.getVehicleState(),self.getVehicleDerivative())


    def derivative(self,state,forcesMoments):
        
        #derivative state storage
        ds = States.vehicleState()

        #placing variables into workable vectors
        v = [[state.u],[state.v],[state.w]]
        eul = [[state.roll],[state.pitch],[state.yaw]]
        p = [[state.p],[state.q],[state.r]]
        fr = [[forcesMoments.Fx],[forcesMoments.Fy],[forcesMoments.Fz]]
        m = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]]

        #calculate derivative of Pn Pe and Pd then storing
        dP = MatrixMath.multiply(MatrixMath.transpose(state.R),v)
        ds.pn = dP[0][0]
        ds.pe = dP[1][0]
        ds.pd = dP[2][0]

        #calculate derivative of veocities and store
        skw = MatrixMath.skew(p[0][0],p[1][0],p[2][0])
        f = MatrixMath.scalarMultiply(1/VPC.mass,fr)
        bdy = MatrixMath.multiply(MatrixMath.scalarMultiply(-1,skw),v)
        dv = MatrixMath.add(bdy,f)
        ds.u = dv[0][0]
        ds.v = dv[1][0]
        ds.w = dv[2][0]
        
        #Derivaive matrx for deriving euler angles
        rde = [[1,math.sin(eul[0][0])*math.tan(eul[1][0]), math.cos(eul[0][0])*math.tan(eul[1][0])],
               [0,math.cos(eul[0][0]),-math.sin(eul[0][0])],
               [0,math.sin(eul[0][0])/math.cos(eul[1][0]),math.cos(eul[0][0])/math.cos(eul[1][0])]]

        #find derivative of euler angles and store
        de = MatrixMath.multiply(rde,p)
        ds.roll = de[0][0]
        ds.pitch = de[1][0]
        ds.yaw = de[2][0]

        #calculate and store derivative of DCM
        ds.R = MatrixMath.multiply(MatrixMath.scalarMultiply(-1,skw),state.R)

        #calculating inertial moment matrix
        j = [[VPC.Jxx,0,-VPC.Jxz],[0,VPC.Jyy,0],[-VPC.Jxz,0,VPC.Jzz]]
        jp = [[VPC.Jzz,0,VPC.Jxz],[0,(VPC.Jxx*VPC.Jzz-(VPC.Jxz**2))/VPC.Jyy,0],[VPC.Jxz,0,VPC.Jxx]]
        jin = MatrixMath.scalarMultiply(1/(VPC.Jxx*VPC.Jzz-(VPC.Jxz**2)),jp)

        #calculate derivative of rotation rates
        dw = MatrixMath.multiply(MatrixMath.scalarMultiply(-1,skw),MatrixMath.multiply(j,p))
        dw = MatrixMath.add(m,dw)
        dw = MatrixMath.multiply(jin,dw)

        #store derivative of rotation rates
        ds.p = dw[0][0]
        ds.q = dw[1][0]
        ds.r = dw[2][0]

        #return state derivative
        return ds

    def ForwardEuler(self,dT,state,dot):

        #storage class for forward euler
        sA = States.vehicleState()

        #update p v and w by forward euler integration
        sA.pn = state.pn + dot.pn*dT
        sA.pe = state.pe + dot.pe*dT
        sA.pd = state.pd + dot.pd*dT
        sA.u = state.u + dot.u*dT
        sA.v = state.v + dot.v*dT
        sA.w = state.w + dot.w*dT
        sA.p = state.p + dot.p*dT
        sA.q = state.q + dot.q*dT
        sA.r = state.r + dot.r*dT

        #return timestepped state
        return sA

    def Rexp(self,dT,state,dot):

        #place values into workable vectors
        w0 = [[state.p],[state.q],[state.r]]
        w1 = [[dot.p],[dot.q],[dot.r]]
        w1 = MatrixMath.scalarMultiply(dT/2,w1)

        #calculate w for skew symmetric matrix
        wn = MatrixMath.add(w0,w1)

        #find skew matrix
        crss = MatrixMath.skew(wn[0][0],wn[1][0],wn[2][0])

        #find magnitude of w
        mag = math.hypot(wn[0][0],wn[1][0],wn[2][0])

        #if magnitude is less than 0.2 use taylor expansion approximation coefficients
        if mag <= 0.2:
            s1 = dT-(((dT**3)*(mag**2))/6)+((dT**5)*(mag**4))/120
            s2 = ((dT**2)/2) - (((dT**4) * (mag**2)) / 24) + ((dT**6) * (mag**4)) / 720
            
        #otherwise use given coefficients
        else:
            s1 = math.sin(mag*dT)/mag
            s2 = (1-math.cos(mag*dT))/(mag**2)

        #identity matrix
        i = [[1,0,0],[0,1,0],[0,0,1]]

        #Calculating matrix exponential piece meal
        p1 =MatrixMath.scalarMultiply(-s1,crss)
        p2 = MatrixMath.scalarMultiply(s2,MatrixMath.multiply(crss,crss))
        rexp = MatrixMath.add(MatrixMath.add(i,p1),p2)

        #return matrix exponential
        return rexp

    def IntegrateState(self,dT,state,dot):
        
        #too lazy to type "state"
        s = state

        #find matrix exponential
        rExp = self.Rexp(dT,s,dot)
        
        #find timestepped DXM
        rN = MatrixMath.multiply(rExp,s.R)

        #timestep most variables by forward euler
        fe = self.ForwardEuler(dT,s,dot)
        
        #storage object for timestepped values
        sI = States.vehicleState()

        #update with new timestepped variables
        sI.pn = fe.pn
        sI.pe = fe.pe
        sI.pd =  fe.pd
        sI.u = fe.u
        sI.v = fe.v
        sI.w =  fe.w
        sI.p = fe.p
        sI.q = fe.q
        sI.r =  fe.r
        sI.R = rN

        #copying some currently unknown variables and timestepping chi
        sI.alpha = s.alpha
        sI.beta = s.beta
        sI.Va  = s.Va
        sI.chi = math.atan2(dot.pe,dot.pn)

        #timestep and store yaw pitch and roll
        sI.yaw,sI.pitch,sI.roll = Rotations.dcm2Euler(sI.R)

        #return new fully timestepped state
        return sI
