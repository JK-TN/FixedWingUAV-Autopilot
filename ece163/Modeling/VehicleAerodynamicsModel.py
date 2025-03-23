"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to perform to calculate and enforce forces and moments on the aircraft.
"""
import math

from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel:
    def __init__(self,initialSpeed=VPC.InitialSpeed,initialHeight=VPC.InitialDownPosition):
        #Setting initial constants
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight

        #Vehicle dynamic model for state and dot storage/handling
        self.dModel = VehicleDynamicsModel.VehicleDynamicsModel()

        #Wind model for storage and handling of wind
        self.wModel = WindModel.WindModel()

    def reset(self):
        #reset initial values
        self.initialSpeed = VPC.InitialSpeed
        self.initialHeight = VPC.InitialDownPosition
        self.dModel = VehicleDynamicsModel.VehicleDynamicsModel()
        self.wModel = WindModel.WindModel()

    def getVehicleDynamicsModel(self):
        #return dynamic model
        return self.dModel

    def getVehicleState(self):
        #return state of dynamic model
        return self.dModel.state

    def getWindModel(self):
        #return wind model
        return self.wModel

    def setVehicleState(self,state):
        #Update state with a provided state
        self.dModel.state = state

    def setWindModel(self,windModel):
        #setting wind model
        self.wModel = windModel

    def CalculateAirspeed(self,state,wind):

        #rotation matrix for inertial to body frame
        dcm = state.R
        #gust handling calculations
        Ws = math.hypot(wind.Wn,wind.We,wind.Wd)

        Xw = math.atan2(wind.We,wind.Wn)


        if math.isclose(Ws, 0):
            Gammaw = 0
        else:
            Gammaw = -math.asin(wind.Wd/Ws)

        #Rotation matrix for going from gust frame to static wind frame
        Rgx = [[math.cos(Xw)*math.cos(Gammaw),math.sin(Xw)*math.cos(Gammaw),-math.sin(Gammaw)],
               [-math.sin(Xw),math.cos(Xw),0],
               [math.cos(Xw)*math.sin(Gammaw),math.sin(Xw)*math.sin(Gammaw),math.cos(Gammaw)]]

        #static and Gust winds in vector form
        sw = [[wind.Wn],[wind.We],[wind.Wd]]
        gw = [[wind.Wu], [wind.Wv], [wind.Ww]]

        #transforming winds into body frame and combining
        Wb = MatrixMath.multiply(dcm,MatrixMath.add(sw,MatrixMath.multiply(MatrixMath.transpose(Rgx),gw)))

        #UAV velocities in vector form
        v = [[state.u],[state.v],[state.w]]

        #remove wind speed from UAV speed
        net = MatrixMath.subtract(v,Wb)

        # update Va and alpha
        Va = math.hypot(net[0][0], net[1][0], net[2][0])
        alpha = math.atan2(net[2][0], net[0][0])

        # update beta based on Va, if Va = 0 beta = 0 else use regular formula
        if math.isclose(Va, 0):
            beta = 0
        else:
            beta = math.asin(net[1][0] / Va)

        return Va, alpha, beta
    
    def Update(self,controls):
        #store current state
        state = self.getVehicleState()

        #calculate/update forces given controls and current state
        forces = self.updateForces(state,controls,self.wModel.getWind())

        #update models
        self.dModel.Update(forces)
        self.wModel.Update()

    def gravityForces(self,state):
        #store current DCM
        dcm = self.dModel.state.R

        #place gravitational forces in vector form Fx, Fy, Fz
        fGi = [[0],[0],[VPC.g0*VPC.mass]]

        #multiply gravitational forces by DCM
        fGb = MatrixMath.multiply(dcm,fGi)

        #Create object to store forces and moments and store our calculated values
        fM = Inputs.forcesMoments()
        fM.Fx = fGb[0][0]
        fM.Fy = fGb[1][0]
        fM.Fz = fGb[2][0]

        #return gravitational forces in body frame
        return fM
        

    def CalculateCoeff_alpha(self,alpha):

        #calculate sigma for blending function
        sig = (1+math.exp(-VPC.M*(alpha-VPC.alpha0))+math.exp(VPC.M*(alpha+VPC.alpha0)))/((1+math.exp(-VPC.M*(alpha-VPC.alpha0)))*(1+math.exp(VPC.M*(alpha+VPC.alpha0))))

        #Cl and CD in pre stall region (attached)
        C_La = VPC.CL0 + VPC.CLalpha*alpha
        C_Da = VPC.CDp + ((C_La**2)/(math.pi*VPC.e*VPC.AR))

        #CL and CD in post stall region (seperated)
        C_Ls = 2*math.sin(alpha)*math.cos(alpha)
        C_Ds = 2*(math.sin(alpha)**2)

        #blending function for previous attached and seperated equations
        C_L = (1-sig)*C_La + (sig)*C_Ls
        C_D = (1-sig)*C_Da + (sig)*C_Ds

        #Pitch Moment coefficient calculation
        C_M = VPC.CM0 + VPC.CMalpha*alpha

        #return calculated values
        return C_L,C_D,C_M

    def aeroForces(self,state):
        #object to store force moments
        aF = Inputs.forcesMoments()

        #leading coefficient for all formulas
        leading_c = 0.5*VPC.rho*(state.Va**2)*VPC.S

        #Dealing with airspeed 0 in rotation rate normalization
        if state.Va == 0:
            norm = 0
        else:
            norm = 1/(2*state.Va)

        #calculate beginning terms for the expansion of My, Fx, Fz
        cL,cD,cM = self.CalculateCoeff_alpha(state.alpha)

        #calculating forces of lift, drag, and y axis
        Fd = leading_c*(cD+VPC.CDq*VPC.c*norm*state.q)
        Fl = leading_c*(cL+VPC.CLq*VPC.c*norm*state.q)
        Fy = leading_c*(VPC.CY0+VPC.CYbeta*state.beta+VPC.CYp*VPC.b*norm*state.p+VPC.CYr*VPC.b*norm*state.r)

        #calculating moments on the body axis
        Mx = leading_c*VPC.b*(VPC.Cl0+VPC.Clbeta*state.beta+VPC.Clp*VPC.b*norm*state.p+VPC.Clr*VPC.b*norm*state.r)
        My = leading_c*VPC.c*(cM+VPC.CMq*VPC.c*norm*state.q)
        Mz = leading_c*VPC.b*(VPC.Cn0+VPC.Cnbeta*state.beta+VPC.Cnp*VPC.b*norm*state.p+VPC.Cnr*VPC.b*norm*state.r)

        #adjusting lift and drag into forces on the body frame
        aF.Fx = -Fd*math.cos(state.alpha)+Fl*math.sin(state.alpha)
        aF.Fz = -Fl*math.cos(state.alpha)-Fd*math.sin(state.alpha)

        #store all other forces and moments
        aF.Fy = Fy
        aF.Mx = Mx
        aF.My = My
        aF.Mz = Mz

        return aF

    def CalculatePropForces(self,Va,Throttle):
        #calculate a, b, and c to find omega
        a = (VPC.rho*(VPC.D_prop**5)*VPC.C_Q0)/(4*math.pi**2)
        b = ((VPC.rho*(VPC.D_prop**4)*VPC.C_Q1*Va)/(2*math.pi)) + ((VPC.KQ*VPC.KQ)/VPC.R_motor)
        c = (VPC.rho*(VPC.D_prop**3)*VPC.C_Q2*(Va**2))-(VPC.KQ*((Throttle*VPC.V_max)/VPC.R_motor))+(VPC.KQ*VPC.i0)

        #calculating omega
        try:
            omega = (-b+math.sqrt((b**2)-4*a*c))/(2*a)
        except ValueError:
            omega = 100

        #use omega to find J
        J = (2*math.pi*Va)/(omega*VPC.D_prop)

        #Find cT and cQ from J
        cT = VPC.C_T0 + VPC.C_T1*J + VPC.C_T2*(J**2)
        cQ = VPC.C_Q0 + VPC.C_Q1*J + VPC.C_Q2*(J**2)

        #use derived values to find force and moment of the propeller
        Fp = (VPC.rho*(omega**2)*(VPC.D_prop**4)*cT)/(4*math.pi**2)

        Mp = -((VPC.rho*(omega**2)*(VPC.D_prop**5)*cQ)/(4*math.pi**2))
        return (Fp,Mp)

    def controlForces(self,state,controls):

        #Calculate propeller forces and moments
        Fp, Mp = self.CalculatePropForces(state.Va,controls.Throttle)

        #Storage for forces and moments
        cF = Inputs.forcesMoments()

        #leading coefficient for all force and moment calculations
        leading_c = 0.5 * VPC.rho * (state.Va ** 2) * VPC.S

        # calculating forces of lift, drag, and y axis based on elevator input
        Fl = leading_c*(VPC.CLdeltaE*controls.Elevator)
        Fd = leading_c*(VPC.CDdeltaE*controls.Elevator)
        Fy = leading_c*(VPC.CYdeltaA*controls.Aileron+VPC.CYdeltaR*controls.Rudder)

        #calculaying moments in x y z of body frame based on aileron and rudder movement
        Mx = leading_c*VPC.b*(VPC.CldeltaA*controls.Aileron+VPC.CldeltaR*controls.Rudder)
        My = leading_c*VPC.c*(VPC.CMdeltaE*controls.Elevator)
        Mz = leading_c*VPC.b*(VPC.CndeltaA*controls.Aileron+VPC.CndeltaR*controls.Rudder)

        # adjusting lift and drag into forces on the body frame
        cF.Fx = Fp-Fd*math.cos(state.alpha)+Fl*math.sin(state.alpha)
        cF.Fz = -Fl*math.cos(state.alpha)-Fd*math.sin(state.alpha)

        # store all other forces and moments
        cF.Fy = Fy
        cF.Mx = Mx+Mp
        cF.My = My
        cF.Mz = Mz

        return cF


    def updateForces(self,state,controls,wind=None):

        #Calculate Va, alpha, and beta based on current state and wind
        Va,alpha,beta = self.CalculateAirspeed(state,wind)

        #state object to update beta alpha and Va
        ns = state
        ns.alpha = alpha
        ns.beta = beta
        ns.Va = Va

        #calculate forces
        af = self.aeroForces(ns)
        cf = self.controlForces(ns,controls)
        gf = self.gravityForces(ns)

        #object to store force and moments
        F = Inputs.forcesMoments()

        #add all force and moment values
        F.Fx = af.Fx + cf.Fx + gf.Fx
        F.Fy = af.Fy + cf.Fy + gf.Fy
        F.Fz = af.Fz + cf.Fz + gf.Fz
        F.Mx = af.Mx + cf.Mx + gf.Mx
        F.My = af.My + cf.My + gf.My
        F.Mz = af.Mz + cf.Mz + gf.Mz

        return F

    
