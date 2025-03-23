"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to perform estimation of vehicle state based on sensor data.
"""
import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Modeling.VehicleDynamicsModel import VehicleDynamicsModel
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as MM
from ..Utilities import Rotations as R

class LowPassFilter:
    def __init__(self,dT=VPC.dT,cutoff=1):
        #init all relevant variables
        self.dT = dT
        self.cutoff = cutoff
        self.yk = 0
        self.a = 2*math.pi*cutoff

    def reset(self):
        #reset storage variable
        self.yk = 0

    def update(self,input):
        #use exponent form to update low pass filter
        exp = math.exp(-self.a*self.dT)
        yk1 = exp*self.yk + (1-exp)*input
        self.yk = yk1
        return yk1


class VehicleEstimator:
    def __init__(self,dT=VPC.dT,gains=Controls.VehicleEstimatorGains(),sensorsModel=SensorsModel.SensorsModel()):
        self.dT = dT
        #init relevant storage containers
        self.sensors = sensorsModel
        self.gains = gains
        #set to initial values
        self.state = States.vehicleState(pd=VPC.InitialDownPosition)
        self.state.R = [[1,0,0],[0,1,0],[0,0,1]]
        self.state.Va = VPC.InitialSpeed
        self.LPF = LowPassFilter(self.dT,cutoff=gains.lowPassCutoff_h)

        #init biases
        self.estimatedGyroBias=[[0],[0],[0]]
        self.estimatedPitotBias = 0
        self.estimatedChiBias = 0
        self.estimatedAscentRate = 0
        self.estimatedAltitudeGPSBias = 0


    def reset(self):
        #reset states, low pass filters and biases
        self.state = States.vehicleState(pd=VPC.InitialDownPosition)
        self.state.R = [[1,0,0],[0,1,0],[0,0,1]]
        self.state.Va = VPC.InitialSpeed
        self.LPF.reset()
        self.estimatedGyroBias = [[0], [0], [0]]
        self.estimatedPitotBias = 0
        self.estimatedChiBias = 0
        self.estimatedAscentRate = 0
        self.estimatedAltitudeGPSBias = 0

    def getEstimatedState(self):
        #return state
        return self.state

    def getEstimatorGains(self):
        #return gains
        return self.gains

    def setEstimatedState(self,estimatedState=States.vehicleState()):
        #set state given state
        self.state = estimatedState
        return

    def setEstimatorBiases(self,estimatedGyroBias=[[0],[0],[0]],estimatedPitotBias=0,estimatedChiBias=0,estimatedAscentRate=0,estimatedAltitudeGPSBias=0):
        #set biases given bises
        self.estimatedGyroBias = estimatedGyroBias
        self.estimatedPitotBias = estimatedPitotBias
        self.estimatedChiBias = estimatedChiBias
        self.estimatedAscentRate = estimatedAscentRate
        self.estimatedAltitudeGPSBias = estimatedAltitudeGPSBias
        return

    def setEstimatorGains(self,gains=Controls.VehicleEstimatorGains()):
        #setting gains based on given gains
        self.gains = gains
        return

    def estimateAttitude(self,sensorData=Sensors.vehicleSensors(),estimatedState=States.vehicleState()):
        #find norm of accel
        norm_A = math.hypot(sensorData.accel_x,sensorData.accel_y,sensorData.accel_z)
        #derive normallized magnetometer and accelorometer data
        ai_hat = [[0/math.hypot(0,0,-VPC.g0)],[0/math.hypot(0,0,-VPC.g0)],[-VPC.g0/math.hypot(0,0,-VPC.g0)]]
        hi_hat = [[VSC.magfield[0][0]/math.hypot(VSC.magfield[0][0],VSC.magfield[1][0],VSC.magfield[2][0])],
                  [VSC.magfield[1][0]/math.hypot(VSC.magfield[0][0],VSC.magfield[1][0],VSC.magfield[2][0])],
                  [VSC.magfield[2][0]/math.hypot(VSC.magfield[0][0],VSC.magfield[1][0],VSC.magfield[2][0])]]
        #store bias
        b_hat = self.estimatedGyroBias

        #if only affected by gravity
        if 0.9*VPC.g0 < norm_A  and norm_A< 1.1*VPC.g0:
            #finding cross of accel and magnetometer with 0 handling
            if math.hypot(sensorData.accel_x,sensorData.accel_y,sensorData.accel_z) != 0:
                weA = MM.crossProduct([[sensorData.accel_x/math.hypot(sensorData.accel_x,sensorData.accel_y,sensorData.accel_z)],
                                        [sensorData.accel_y/math.hypot(sensorData.accel_x,sensorData.accel_y,sensorData.accel_z)],
                                        [sensorData.accel_z/math.hypot(sensorData.accel_x,sensorData.accel_y,sensorData.accel_z)]],
                                        MM.multiply(estimatedState.R,ai_hat))
            else:
                weA = MM.crossProduct([[0],[0],[0]],MM.multiply(estimatedState.R,ai_hat))

            if math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z) != 0:
                weB = MM.crossProduct(
                    [[sensorData.mag_x / math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z)],
                        [sensorData.mag_y / math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z)],
                        [sensorData.mag_z / math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z)]],
                        MM.multiply(estimatedState.R, hi_hat))
            else:
                weB = MM.crossProduct([[0],[0],[0]],MM.multiply(estimatedState.R, hi_hat))
            #calculate biases
            bdA = MM.scalarMultiply(-self.gains.Ki_acc,weA)
            bdB = MM.scalarMultiply(-self.gains.Ki_mag,weB)
            bd = MM.add(bdA,bdB)
            b_hat = MM.add(b_hat,MM.scalarMultiply(self.dT,bd))
            #remove biases from gyro data
            w_hat = MM.subtract([[sensorData.gyro_x], [sensorData.gyro_y], [sensorData.gyro_z]], b_hat)

            #calculate with gains
            gA = MM.scalarMultiply(self.gains.Kp_acc,weA)
            gB = MM.scalarMultiply(self.gains.Kp_mag,weB)
            #update vehicle state for skew calculation
            state = States.vehicleState(p=w_hat[0][0] + gA[0][0] + gB[0][0],
                                        q=w_hat[1][0] + gA[1][0] + gB[1][0],
                                        r=w_hat[2][0] + gA[2][0] + gB[2][0])


        else:
            #do the same as above but ignore acceleration
            if math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z) != 0:
                weB = MM.crossProduct(
                    [[sensorData.mag_x / math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z)],
                        [sensorData.mag_y / math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z)],
                        [sensorData.mag_z / math.hypot(sensorData.mag_x, sensorData.mag_y, sensorData.mag_z)]],
                        MM.multiply(estimatedState.R, hi_hat))
            else:
                weB = MM.crossProduct([[0],[0],[0]],MM.multiply(estimatedState.R, hi_hat))

            bdB = MM.scalarMultiply(-self.gains.Ki_mag,weB)
            b_hat = MM.add(b_hat,MM.scalarMultiply(self.dT,bdB))
            w_hat = MM.subtract([[sensorData.gyro_x], [sensorData.gyro_y], [sensorData.gyro_z]], b_hat)


            gB = MM.scalarMultiply(self.gains.Kp_mag,weB)
            state = States.vehicleState(p=w_hat[0][0] + gB[0][0],
                                        q=w_hat[1][0] + gB[1][0],
                                        r=w_hat[2][0] + gB[2][0])
        #calculate R and return values
        dot = States.vehicleState(p = 0,q=0,r=0)
        V = VehicleDynamicsModel()
        R_new = MM.multiply(V.Rexp(self.dT,state,dot),estimatedState.R)
        return b_hat,w_hat,R_new

    def estimateAltitude(self,sensorData=Sensors.vehicleSensors(),estimatedState=States.vehicleState()):
        #set variables
        bh_gps= self.estimatedAltitudeGPSBias
        bh_dot =self.estimatedAscentRate
        h_hat =-estimatedState.pd

        #calculate z accel and height (with LPF)
        aup = MM.add(MM.multiply(MM.transpose(estimatedState.R),[[sensorData.accel_x],[sensorData.accel_y],[sensorData.accel_z]]),[[0],[0],[VPC.g0]])
        h_baro =-(sensorData.baro-VSC.Pground)/(VPC.rho*VPC.g0)
        hLPF = self.LPF.update(h_baro)

        #integrate bias and h without gps
        bh_dot2 = self.gains.Ki_h*(hLPF-h_hat)
        bh_dot = bh_dot+(bh_dot2*self.dT)
        he_dot = (aup[2][0]*self.dT) + bh_dot
        h_hat = h_hat + ((self.gains.Kp_h * (hLPF - h_hat) + he_dot) * self.dT)

        if self.sensors.updateTicks%self.sensors.gpsTickUpdate == 1:
            #if gps then integrate gps bias and calculate gps height
            bh_gpsdot = -self.gains.Ki_h_gps*(sensorData.gps_alt-h_hat)
            bh_gps = (bh_gpsdot*self.dT) +bh_gps
            h_dot = self.gains.Kp_h_gps*(sensorData.gps_alt-h_hat)+h_hat-bh_gps
            h_hat = h_hat+h_dot*self.dT

        else:
            #if no gps remove gps bias
            h_hat = h_hat-bh_gps
        return h_hat,he_dot,bh_gps

    def estimateCourse(self,sensorData=Sensors.vehicleSensors(),estimatedState=States.vehicleState()):
        #set variables
        b_hatp = self.estimatedChiBias
        chi_hat = estimatedState.chi
        yaw_dot = (1/math.cos(estimatedState.pitch))*(estimatedState.q*math.sin(estimatedState.roll)+estimatedState.r*math.cos(estimatedState.roll))

        if self.sensors.updateTicks  % self.sensors.gpsTickUpdate == 1:
            #if gps integrate bias and course
            chiE = sensorData.gps_cog-chi_hat
            '''if chiE > math.pi:
                chiE = math.pi
            elif chiE < -math.pi:
                chiE = -math.pi'''
            b_dot = -self.gains.Ki_chi*(chiE)
            b_hat = b_hatp + (b_dot*self.dT)
            chi_dot = self.gains.Kp_chi*(chiE)-b_hat
            chi_hat = chi_hat+(chi_dot*self.dT)+(yaw_dot*self.dT)
            b_hatp = b_hat
        else:
            #if no gps remove bias and integrate course
            chi_dot = yaw_dot-b_hatp
            chi_hat = chi_hat+(chi_dot*self.dT)

        '''if chi_hat > math.pi:
            chi_hat = math.pi
        elif chi_hat < -math.pi:
            chi_hat = -math.pi'''

        #return bias and course
        return b_hatp,chi_hat

    def estimateAirspeed(self,sensorData=Sensors.vehicleSensors(),estimatedState=States.vehicleState()):
        #set variables
        b_hat = self.estimatedPitotBias
        Va_hat = estimatedState.Va

        #find x accel without g
        ax = MM.add([[sensorData.accel_x],[sensorData.accel_y],[sensorData.accel_z]],MM.multiply(estimatedState.R,[[0],[0],[VPC.g0]]))
        #airspeed from pitot
        Va_p = math.sqrt(2*abs(sensorData.pitot)/VPC.rho)

        #integrate bias and airspeed estimate
        b_dot = -self.gains.Ki_Va*(Va_p-Va_hat)
        b_hat = b_hat + b_dot*self.dT
        Va_dot = ax[0][0]-b_hat+self.gains.Kp_Va*(Va_p-Va_hat)
        Va_hat = Va_hat + Va_dot*self.dT
        #return bias and airspeed
        return b_hat,Va_hat

    def Update(self):
        #storage for states
        s = States.vehicleState()
        #update state estimates
        bGyro,w,Rn = self.estimateAttitude(self.sensors.sensorsNoisy,self.state)
        h,bAscent,bGPS = self.estimateAltitude(self.sensors.sensorsNoisy, self.state)
        bPitot,Va = self.estimateAirspeed(self.sensors.sensorsNoisy, self.state)
        bChi,Chi = self.estimateCourse(self.sensors.sensorsNoisy, self.state)

        #gather biases and estimates then update
        s.pn = self.sensors.sensorsNoisy.gps_n
        s.pe = self.sensors.sensorsNoisy.gps_e
        s.R = Rn
        s.yaw,s.pitch,s.roll = R.dcm2Euler(Rn)
        s.p = w[0][0]
        s.q = w[1][0]
        s.r = w[2][0]
        s.pd = -h
        s.Va = Va
        s.chi = Chi
        self.setEstimatedState(s)
        self.setEstimatorBiases(bGyro,bPitot,bChi,bAscent,bGPS)
        return
