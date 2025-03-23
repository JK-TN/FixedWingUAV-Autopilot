"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to perform closed loop control of the aircraft.
"""
import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleEstimator as VehicleEstimator
import ece163.Sensors.SensorsModel as SensorsModel

class PIDControl:
    def __init__(self,dT=VPC.dT,kp=0,kd=0,ki=0,trim=0,lowLimit=0,highLimit=0):
        #containers for holding PID values
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.acc = 0
        self.perr = 0

    def Update(self,command=0,current=0,derivative=0):
        #error between command and real value
        err = command-current

        #hold onto accumulator
        safety = self.acc

        #update accumulator
        self.acc = self.acc + self.dT*(err + self.perr)/2

        #find output of P I and D blocks
        I = self.ki*self.acc
        D = derivative*self.kd
        P = self.kp*err

        #sum PID with trim
        u = P+I-D+self.trim

        #saturation check
        if(u>self.highLimit):
            u = self.highLimit

            #dont update accumulator if saturated
            self.acc = safety
        elif(u<self.lowLimit):
            u = self.lowLimit
            self.acc = safety

        #update prev error
        self.perr = err

        #return output
        return u

    def resetIntegrator(self):
        #reset accumulator and previous error
        self.acc = 0
        self.perr = 0
        return

    def setPIDGains(self,dT=VPC.dT,kp=0,kd=0,ki=0,trim=0,lowLimit=0,highLimit=0):
        #set values and storages for PID
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.acc = 0
        self.perr = 0
        return

class PIControl:
    def __init__(self,dT=VPC.dT,kp=0,ki=0,trim=0,lowLimit=0,highLimit=0):
        #init PI variables and stores
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.acc = 0
        self.perr = 0

    def Update(self,command=0,current=0):

        #error of control and actual value
        err = command - current

        #hold onto accumulator in case of saturation
        safety = self.acc

        #update accumulator
        self.acc = self.acc + self.dT * (err + self.perr) / 2

        #find I and P outputs
        I = self.ki * self.acc
        P = self.kp * err

        #sum P and I with trim
        u = P + I + self.trim

        #saturation check
        if (u > self.highLimit):
            u = self.highLimit
            self.acc = safety
        elif (u < self.lowLimit):
            u = self.lowLimit
            self.acc = safety

        #update previous error
        self.perr = err

        #return output
        return u

    def resetIntegrator(self):
        #reset accumulator and previous error
        self.acc = 0
        self.perr = 0
        return

    def setPIGains(self,dT=VPC.dT,kp=0,ki=0,trim=0,lowLimit=0,highLimit=0):
        #set variables and containers for PI
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.acc = 0
        self.perr = 0
        return

class PDControl:
    def __init__(self,kp=0,kd=0,trim=0,lowLimit=0,highLimit=0):
        #initialize variables and stores for PD
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

    def Update(self,command=0,current=0,derivative=0):

        #error of control vs actual value
        err = command - current

        #find P and D outputs
        D = derivative * self.kd
        P = self.kp * err

        #sum PD with trim
        u = P - D + self.trim

        #saturation check
        if (u > self.highLimit):
            u = self.highLimit
        elif (u < self.lowLimit):
            u = self.lowLimit

        #return output
        return u

    def setPDGains(self,kp=0,kd=0,trim=0,lowLimit=0,highLimit=0):

        #set values and containers for PD
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

class VehicleClosedLoopControl:
    def __init__(self,dT=VPC.dT,rudderControlSource='SIDESLIP',useSensors=False,useEstimator=False):
        #init variables and stores for objects that hold info
        self.useSensors = useSensors
        self.useEstimator = useEstimator
        self.dT = dT
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.Gain = Controls.controlGains()
        self.Tctrl = Inputs.controlInputs()
        self.Octrl = Inputs.controlInputs()
        self.mode = Controls.AltitudeStates.HOLDING
        if self.useSensors:
            self.sensorsModel = SensorsModel.SensorsModel(aeroModel=self.VAM)

        if self.useEstimator:
            self.vehicleEstimator = VehicleEstimator.VehicleEstimator(dT=self.dT,sensorsModel=self.sensorsModel)

        #init PI controllers
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()

        #init PD controllers
        self.elevatorFromPitch = PDControl()

        #init PID controllers
        self.aileronFromRoll = PIDControl()

    def UpdateControlCommands(self,referenceCommands,state):
        #upper and lower threshold for altitude control
        threshU = referenceCommands.commandedAltitude +VPC.altitudeHoldZone
        threshL = referenceCommands.commandedAltitude -VPC.altitudeHoldZone

        #chi storage for modification
        chi = state.chi

        #storage for returning
        store = Inputs.controlInputs()

        #increment/decrement chi based on error compared to pi
        if referenceCommands.commandedCourse-chi >= math.pi:
            chi = chi+2*math.pi
        elif referenceCommands.commandedCourse-chi <= -math.pi:
            chi = chi - 2*math.pi

        #if in descending mode
        if self.mode == Controls.AltitudeStates.DESCENDING:

            #if no longer need to descend
            if -state.pd < threshU and -state.pd > threshL:
                #reset pitch-altitude integrator
                self.pitchFromAltitude.resetIntegrator()
                #redo function call in holding mode
                self.mode = Controls.AltitudeStates.HOLDING
                return self.UpdateControlCommands(referenceCommands,state)

            else:
                #otherwise use throttle and pitch command of descending mode
                store.Throttle = VPC.minControls.Throttle
                pitchCmd = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed,state.Va)

        #if in holding mode
        elif self.mode == Controls.AltitudeStates.HOLDING:

            #if need to descend
            if -state.pd > threshU :
                #reset pitch-airspeed integrator
                self.pitchFromAirspeed.resetIntegrator()
                #redo function call in descending mode
                self.mode = Controls.AltitudeStates.DESCENDING
                return self.UpdateControlCommands(referenceCommands,state)

            #if need to climb
            elif -state.pd < threshL:
                # reset pitch-airspeed integrator
                self.pitchFromAirspeed.resetIntegrator()
                # redo function call in climbing mode
                self.mode = Controls.AltitudeStates.CLIMBING
                return self.UpdateControlCommands(referenceCommands,state)

            else:
                #if in holding mode use holding pitch command and throttle
                store.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed,state.Va)
                pitchCmd = self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, -state.pd)

        #if in climbing mode
        elif self.mode == Controls.AltitudeStates.CLIMBING:

            #if no longer need to climb
            if -state.pd < threshU and -state.pd > threshL:
                #reset pitch-altitude integrator
                self.pitchFromAltitude.resetIntegrator()

                #redo function call in holding mode
                self.mode = Controls.AltitudeStates.HOLDING
                return self.UpdateControlCommands(referenceCommands,state)

            else:
                #if in climbing still use climbing throttle and pitch command
                store.Throttle = VPC.maxControls.Throttle
                pitchCmd = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)

        #find roll command
        rollCmd = self.rollFromCourse.Update(referenceCommands.commandedCourse,chi)

        #find and store control surface commands
        store.Aileron = self.aileronFromRoll.Update(rollCmd,state.roll,state.p)
        store.Rudder = self.rudderFromSideslip.Update(0,state.beta)
        store.Elevator = self.elevatorFromPitch.Update(pitchCmd,state.pitch,state.q)

        #update reference commands
        referenceCommands.commandedPitch = pitchCmd
        referenceCommands.commandedRoll = rollCmd

        #return the control surface commands
        #return the control surface commands
        return store

    def getControlGains(self):
        return self.Gain

    def getSensorsModel(self):
        if self.useSensors:
            return self.sensorsModel
        else:
            return SensorsModel.SensorsModel()

    def getTrimInputs(self):
        return self.Tctrl

    def getVehicleAerodynamicsModel(self):
        return self.VAM

    def getVehicleControlSurfaces(self):
        return self.Octrl

    def getvehicleEstimator(self):
        if self.useEstimator:
            return self.vehicleEstimator
        else:
            return VehicleEstimator.VehicleEstimator()

    def getVehicleState(self):
        return self.VAM.getVehicleState()

    def reset(self):
        #reset aerodynamics model and all integrators
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        if self.useSensors:
            self.sensorsModel.reset()
        if self.useEstimator:
            self.vehicleEstimator.reset()
        return

    def setControlGains(self,controlGains=Controls.controlGains()):
        #rename controlGains for ease of use
        cG = controlGains

        #renaming trim input for ease of use
        T = self.getTrimInputs()

        #update gain
        self.Gain = cG

        #update PI controllers with K, and trim
        self.rollFromCourse.setPIGains(self.dT,cG.kp_course,cG.ki_course,0,math.radians(-VPC.bankAngleLimit),math.radians(VPC.bankAngleLimit))
        self.rudderFromSideslip.setPIGains(self.dT,cG.kp_sideslip,cG.ki_sideslip,T.Rudder,VPC.minControls.Rudder,VPC.maxControls.Rudder)
        self.throttleFromAirspeed.setPIGains(self.dT,cG.kp_SpeedfromThrottle,cG.ki_SpeedfromThrottle,T.Throttle,VPC.minControls.Throttle,VPC.maxControls.Throttle)
        self.pitchFromAltitude.setPIGains(self.dT,cG.kp_altitude,cG.ki_altitude,0,math.radians(-VPC.pitchAngleLimit),math.radians(VPC.pitchAngleLimit))
        self.pitchFromAirspeed.setPIGains(self.dT,cG.kp_SpeedfromElevator,cG.ki_SpeedfromElevator,0,math.radians(-VPC.pitchAngleLimit),math.radians(VPC.pitchAngleLimit))

        # update PD controllers with K, and trim
        self.elevatorFromPitch.setPDGains(cG.kp_pitch,cG.kd_pitch,T.Elevator,VPC.minControls.Elevator,VPC.maxControls.Elevator)

        # update PID controllers with K, and trim
        self.aileronFromRoll.setPIDGains(self.dT,cG.kp_roll,cG.kd_roll,cG.ki_roll,T.Aileron,VPC.minControls.Aileron,VPC.maxControls.Aileron)
        return

    def setTrimInputs(self,trimInputs=Inputs.controlInputs(Throttle=0.5,Aileron=0,Elevator=0,Rudder=0)):
        #set trim inputs
        self.Tctrl.Rudder = trimInputs.Rudder
        self.Tctrl.Aileron = trimInputs.Aileron
        self.Tctrl.Elevator = trimInputs.Elevator
        self.Tctrl.Throttle = trimInputs.Throttle
        return

    def setVehicleState(self,state):
        self.VAM.setVehicleState(state)
        return


#need one version of update for test and another for use in Chapter6.py
    def update(self,referenceCommands=Controls.referenceCommands()):
        #Derive control command from reference commands and state
        self.Octrl= self.UpdateControlCommands(referenceCommands,self.VAM.getVehicleState())
        if self.useEstimator:
            self.Octrl = self.UpdateControlCommands(referenceCommands,state=self.vehicleEstimator.state)
        #update model with commands
        self.VAM.Update(self.Octrl)
        if self.useSensors:
            self.sensorsModel.update(self.VAM.dModel.state,self.VAM.dModel.dot)

        if self.useEstimator:
            self.vehicleEstimator.Update()
        return

    def Update(self,referenceCommands=Controls.referenceCommands()):
        # Derive control command from reference commands and state
        self.Octrl = self.UpdateControlCommands(referenceCommands,self.VAM.getVehicleState())

        # update model with commands
        self.VAM.Update(self.Octrl)
        if self.useSensors:
            self.sensorsModel.update(self.VAM.dModel.state,self.VAM.dModel.dot)
        return