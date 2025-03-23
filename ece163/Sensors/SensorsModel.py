"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to model sensors on the aircraft and their associated noises/drifts.
"""
import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ..Utilities import Rotations
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

class GaussMarkov:
    def __init__(self,dT=VPC.dT,tau=1000000.0,eta=0.0):
        #initailizing storage variables
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0.0

    def reset(self):
        #reset storage variables
        self.dT = VPC.dT
        self.tau = 1000000.0
        self.eta = 0.0
        self.v = 0.0

    def update(self,vnoise=None):
        #if no noise use guassian distribution
        if vnoise == None:
            noise = random.gauss(0.0,self.eta)
            #otherwise use provided noise
        else:
            noise = vnoise
        #calculate and save new state
        v = math.exp(-self.dT/self.tau)*self.v+noise
        self.v = v
        #return state
        return v

class GaussMarkovXYZ:
    def __init__(self,dT=VPC.dT,tauX=1000000.0,etaX=0.0,tauY=None,etaY=None,tauZ=None,etaZ=None):

        #if else logic in cases where y and z are empty
        #y copies x and z copies y
        if tauY is None:
            tY = tauX
        else:
            tY = tauY
        if etaY is None:
            eY = etaX
        else:
            eY = etaY

        if tauZ is None:
            tZ = tY
        else:
            tZ = tauZ
        if etaZ is None:
            eZ = eY
        else:
            eZ = etaZ

        #create storage gauss markov for xyz axes
        self.gmX = GaussMarkov(dT,tauX,etaX)
        self.gmY = GaussMarkov(dT,tY,eY)
        self.gmZ = GaussMarkov(dT,tZ,eZ)

    def reset(self):
        #reset all three axes GaussMarkov
        self.gmX.reset()
        self.gmY.reset()
        self.gmZ.reset()
    def update(self,vXnoise=None,vYnoise=None,vZnoise=None):
        # update all three axes GaussMarkov
        vX = self.gmX.update(vXnoise)
        vY = self.gmY.update(vYnoise)
        vZ = self.gmZ.update(vZnoise)
        #return updated values
        return vX,vY,vZ

class SensorsModel:
    def __init__(self,aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(),taugyro=VSC.gyro_tau,etagyro=VSC.gyro_eta,tauGPS=VSC.GPS_tau,etaGPSHorizontal=VSC.GPS_etaHorizontal,etaGPSVertical=VSC.GPS_etaVertical,gpsUpdateHz=VSC.GPS_rate):
        #storage variables and containers for sensor model
        self.taugyro = taugyro
        self.etagyro = etagyro
        self.tauGPS = tauGPS
        self.etaGPSHorizontal = etaGPSHorizontal
        self.etaGPSVertical = etaGPSVertical
        self.gpsUpdateHz = gpsUpdateHz
        self.aeroModel = aeroModel
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()

        self.gyroGM = GaussMarkovXYZ(VPC.dT,taugyro,etagyro)
        self.gpsGM = GaussMarkovXYZ(VPC.dT,tauGPS,etaGPSHorizontal,tauGPS,etaGPSHorizontal,tauGPS,etaGPSVertical)

        #time handling for GPS and sensor model
        self.dT = VPC.dT
        self.updateTicks = 0
        self.gpsTickUpdate = (1/gpsUpdateHz)/VPC.dT

    def initializeSigmas(self,gyroSigma=0.002617993877991494, accelSigma=0.24525000000000002,
                        magSigma=25.0, baroSigma=10.0, pitotSigma=2.0, gpsSigmaHorizontal=0.4,
                        gpsSigmaVertical=0.7, gpsSigmaSOG=0.05, gpsSigmaCOG=0.002):
        #storage for return value
        sS = Sensors.vehicleSensors()
        #set sigmas given input
        sS.gyro_x = gyroSigma
        sS.gyro_y = gyroSigma
        sS.gyro_z = gyroSigma
        sS.accel_x = accelSigma
        sS.accel_y = accelSigma
        sS.accel_z = accelSigma
        sS.mag_x = magSigma
        sS.mag_y = magSigma
        sS.mag_z = magSigma
        sS.baro = baroSigma
        sS.pitot = pitotSigma
        sS.gps_n = gpsSigmaHorizontal
        sS.gps_e = gpsSigmaHorizontal
        sS.gps_alt = gpsSigmaVertical
        sS.gps_sog = gpsSigmaSOG
        sS.gps_cog = gpsSigmaCOG
        return sS

    def initializeBiases(self, gyroBias=VSC.gyro_bias , accelBias=VSC.accel_bias, magBias=VSC.mag_bias,baroBias=VSC.baro_bias, pitotBias=VSC.pitot_bias):
        #storage for return value
        sB = Sensors.vehicleSensors()
        #initializing biases with uniform distribution of given value
        sB.gyro_x = random.uniform(-gyroBias,gyroBias)
        sB.gyro_y = random.uniform(-gyroBias,gyroBias)
        sB.gyro_z = random.uniform(-gyroBias,gyroBias)
        sB.accel_x = random.uniform(-accelBias,accelBias)
        sB.accel_y = random.uniform(-accelBias,accelBias)
        sB.accel_z = random.uniform(-accelBias,accelBias)
        sB.mag_x = random.uniform(-magBias,magBias)
        sB.mag_y = random.uniform(-magBias,magBias)
        sB.mag_z = random.uniform(-magBias,magBias)
        sB.baro = random.uniform(-baroBias,baroBias)
        sB.pitot = random.uniform(-pitotBias,pitotBias)
        sB.gps_n = 0
        sB.gps_e = 0
        sB.gps_alt = 0
        sB.gps_sog = 0
        sB.gps_cog = 0
        return sB

    def  reset(self):
        #reset relevant containers
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorsSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()

        self.gyroGM = GaussMarkovXYZ(VPC.dT, self.taugyro, self.etagyro)
        self.gpsGM = GaussMarkovXYZ(VPC.dT, self.tauGPS, self.etaGPSHorizontal, self.tauGPS, self.etaGPSHorizontal, self.tauGPS, self.etaGPSVertical)

        self.updateTicks = 0
        self.gpsTickUpdate = (1 / self.gpsUpdateHz) * VPC.dT

    def getSensorsNoisy(self):
        #return noisy sensor values
        return self.sensorsNoisy

    def getSensorsTrue(self):
        #return true sensor values
        return self.sensorsTrue

    def setSensorsNoisy(self,sensorsNoisy=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                        accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                        gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0)):
        #set noisy sensors with provided values
        self.sensorsNoisy = sensorsNoisy

    def setSensorsTrue(self,sensorsTrue=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                        accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0, baro=0.0, pitot=0.0,
                        gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0, gps_cog=0.0)):
        # set true sensors with provided values
        self.sensorsTrue = sensorsTrue

    def update(self,state=None,dot=None):
        #if else cases for manually or automatically passing the function state and dot
        if state != None:
            s = state
        else:
            s = self.aeroModel.dModel.state

        if dot != None:
            d = dot
        else:
            d = self.aeroModel.dModel.dot

        #update and store sensors true and noisy
        self.sensorsTrue = self.updateSensorsTrue(self.sensorsTrue,s,d)
        self.sensorsNoisy = self.updateSensorsNoisy(self.sensorsTrue,self.sensorsNoisy,self.sensorsBiases,self.sensorsSigmas)
        #increment tick
        self.updateTicks += 1

    def updateAccelsTrue(self,state,dot):
        #storing variables in matrix friendly form
        v_dot = [[dot.u],[dot.v],[dot.w]]
        skw = MatrixMath.skew(state.p,state.q,state.r)
        v = [[state.u],[state.v],[state.w]]

        #calculate true accels
        g = MatrixMath.multiply(Rotations.euler2DCM(state.yaw,state.pitch,state.roll),[[0],[0],[VPC.g0]])
        vSkw = MatrixMath.multiply(skw,v)
        a = MatrixMath.add(v_dot,MatrixMath.subtract(vSkw,g))

        #return true accels
        return a[0][0], a[1][0], a[2][0]

    def updateGyrosTrue(self,state):
        #return true gyro readings
        return state.p,state.q,state.r

    def updateMagsTrue(self,state):
        M = VSC.magfield
        #calculate mag readings
        Mt = MatrixMath.multiply(Rotations.euler2DCM(state.yaw,state.pitch,state.roll),M)

        #return readings
        return Mt[0][0],Mt[1][0],Mt[2][0]

    def updateGPSTrue(self,state,dot):
        #calculate and return gps readings
        return state.pn,state.pe,-state.pd,math.hypot(dot.pn,dot.pe),math.atan2(dot.pe,dot.pn)

    def updatePressureSensorsTrue(self,state):
        #calculate barometer and pitot pressure readings
        Pb = VSC.Pground+(VPC.rho*VPC.g0*state.pd)
        Pp = 0.5*VPC.rho*(state.Va)**2

        #return readings
        return Pb,Pp

    def updateSensorsTrue(self,prevTrueSensors,state,dot):
        #storage for return
        sO = Sensors.vehicleSensors()
        #update true sensor readings
        sO.gyro_x,sO.gyro_y,sO.gyro_z = self.updateGyrosTrue(state)
        sO.accel_x,sO.accel_y,sO.accel_z = self.updateAccelsTrue(state,dot)
        sO.mag_x,sO.mag_y,sO.mag_z = self.updateMagsTrue(state)
        sO.baro,sO.pitot = self.updatePressureSensorsTrue(state)

        #for gps check if valid update time
        if self.updateTicks%self.gpsUpdateHz == 0:
            sO.gps_n,sO.gps_e,sO.gps_alt,sO.gps_sog,sO.gps_cog = self.updateGPSTrue(state,dot)
        #if not then return past values
        else:
            sO.gps_n = prevTrueSensors.gps_n
            sO.gps_e = prevTrueSensors.gps_e
            sO.gps_alt = prevTrueSensors.gps_alt
            sO.gps_sog = prevTrueSensors.gps_sog
            sO.gps_cog = prevTrueSensors.gps_cog

        #return updated sensors
        return sO

    def updateSensorsNoisy(self,trueSensors=Sensors.vehicleSensors(),noisySensors=Sensors.vehicleSensors(),
                            sensorBiases=Sensors.vehicleSensors(),sensorSigmas=Sensors.vehicleSensors()):
        #storage for returning
        Sn = Sensors.vehicleSensors()

        #add noise, bias, and drift to respective sensor readings
        Sn.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x  + self.gyroGM.gmX.v + random.gauss(0,sensorSigmas.gyro_x)
        Sn.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + self.gyroGM.gmY.v + random.gauss(0,sensorSigmas.gyro_y)
        Sn.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + self.gyroGM.gmZ.v + random.gauss(0,sensorSigmas.gyro_z)
        Sn.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0,sensorSigmas.accel_x)
        Sn.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0,sensorSigmas.accel_y)
        Sn.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0,sensorSigmas.accel_z)
        Sn.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x)
        Sn.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y)
        Sn.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z)
        Sn.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0, sensorSigmas.baro)
        Sn.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0, sensorSigmas.pitot)

        #only add new noise to gps if valid update time
        if self.updateTicks%self.gpsUpdateHz == 0:
            Sn.gps_n = trueSensors.gps_n + sensorBiases.gps_n + self.gpsGM.gmX.v + random.gauss(0, sensorSigmas.gps_n)
            Sn.gps_e = trueSensors.gps_e + sensorBiases.gps_e + self.gpsGM.gmY.v + random.gauss(0, sensorSigmas.gps_e)
            Sn.gps_alt = trueSensors.gps_alt + sensorBiases.gps_alt + self.gpsGM.gmZ.v + random.gauss(0, sensorSigmas.gps_alt)
            Sn.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog  + random.gauss(0, sensorSigmas.gps_sog)

            #Trap if ground speed is 0
            try:
                Sn.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog  + random.gauss(0, sensorSigmas.gps_cog*(VPC.InitialSpeed/trueSensors.gps_sog))
            except:
                pass

            #limit to +- pi
            if Sn.gps_cog > math.pi:
                Sn.gps_cog = math.pi
            elif Sn.gps_cog < -math.pi:
                Sn.gps_cog = -math.pi

        #otherwise old values are preserved
        else:
            Sn.gps_n = noisySensors.gps_n
            Sn.gps_e = noisySensors.gps_e
            Sn.gps_alt = noisySensors.gps_alt
            Sn.gps_sog = noisySensors.gps_sog
            Sn.gps_cog = noisySensors.gps_cog

        #return noisy sensor readings
        return Sn
