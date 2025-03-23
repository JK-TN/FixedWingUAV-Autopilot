"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to calculate controller gains, frequencies, and damping.
"""
import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations


def computeGains(tuningParameters=Controls.controlTuning(),linearizedModel=Linearized.transferFunctions()):
    #storage object
    store = Controls.controlGains()

    #Calculatin K for lateral autopilot from wn and Zeta
    store.kp_roll = (tuningParameters.Wn_roll**2)/linearizedModel.a_phi2
    store.kd_roll = ((2*tuningParameters.Zeta_roll*tuningParameters.Wn_roll)-linearizedModel.a_phi1)/linearizedModel.a_phi2
    store.ki_roll = 0.001
    store.kp_sideslip = (2*tuningParameters.Zeta_sideslip*tuningParameters.Wn_sideslip-linearizedModel.a_beta1)/linearizedModel.a_beta2
    store.ki_sideslip = (tuningParameters.Wn_sideslip**2)/linearizedModel.a_beta2
    store.kp_course = (2*tuningParameters.Zeta_course*tuningParameters.Wn_course*linearizedModel.Va_trim)/VPC.g0
    store.ki_course =(linearizedModel.Va_trim*(tuningParameters.Wn_course**2))/VPC.g0

    #Calculating K for longitudinal autopilot from wn to Zeta
    store.kp_pitch = (tuningParameters.Wn_pitch**2-linearizedModel.a_theta2)/linearizedModel.a_theta3
    store.kd_pitch = (tuningParameters.Wn_pitch*2*tuningParameters.Zeta_pitch-linearizedModel.a_theta1)/linearizedModel.a_theta3
    # calculating DC Ktheta
    K_thetaDC = (store.kp_pitch * linearizedModel.a_theta3) / (linearizedModel.a_theta2 + store.kp_pitch * linearizedModel.a_theta3)
    store.ki_altitude = (tuningParameters.Wn_altitude**2)/(K_thetaDC*linearizedModel.Va_trim)
    store.kp_altitude = (2*tuningParameters.Zeta_altitude*tuningParameters.Wn_altitude)/(K_thetaDC*linearizedModel.Va_trim)
    store.kp_SpeedfromThrottle = (2*tuningParameters.Zeta_SpeedfromThrottle*tuningParameters.Wn_SpeedfromThrottle-linearizedModel.a_V1)/linearizedModel.a_V2
    store.ki_SpeedfromThrottle = (tuningParameters.Wn_SpeedfromThrottle**2)/linearizedModel.a_V2
    store.ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator**2)/(K_thetaDC*VPC.g0)
    store.kp_SpeedfromElevator = (linearizedModel.a_V1-2*tuningParameters.Zeta_SpeedfromElevator*tuningParameters.Wn_SpeedfromElevator)/(K_thetaDC*VPC.g0)
    return store

def computeTuningParameters(controlGains=Controls.controlGains(),linearizedModel=Linearized.transferFunctions()):
    #Storage object
    store = Controls.controlTuning()

    #try in case of math error
    try:
        #calculating Wn and Zeta of lateral autopilot from K values
        store.Wn_roll = math.sqrt(controlGains.kp_roll*linearizedModel.a_phi2)
        store.Zeta_roll = (linearizedModel.a_phi1+controlGains.kd_roll*linearizedModel.a_phi2)/(2*store.Wn_roll)
        store.Wn_course = math.sqrt((controlGains.ki_course*VPC.g0)/linearizedModel.Va_trim)
        store.Zeta_course = ((VPC.g0*controlGains.kp_course)/linearizedModel.Va_trim)/(2*store.Wn_course)
        store.Wn_sideslip = math.sqrt(linearizedModel.a_beta2*controlGains.ki_sideslip)
        store.Zeta_sideslip = (linearizedModel.a_beta1+linearizedModel.a_beta2*controlGains.kp_sideslip)/(2*store.Wn_sideslip)

        # calculating Wn and Zeta of longitudinal autopilot from K values
        store.Wn_pitch = math.sqrt(linearizedModel.a_theta2+controlGains.kp_pitch*linearizedModel.a_theta3)
        store.Zeta_pitch = (linearizedModel.a_theta1+controlGains.kd_pitch*linearizedModel.a_theta3)/(2*store.Wn_pitch)

        #calculating DC Ktheta
        K_thetaDC = (controlGains.kp_pitch*linearizedModel.a_theta3)/(linearizedModel.a_theta2+controlGains.kp_pitch*linearizedModel.a_theta3)

        store.Wn_altitude = math.sqrt(K_thetaDC*linearizedModel.Va_trim*controlGains.ki_altitude)
        store.Zeta_altitude = (K_thetaDC*linearizedModel.Va_trim*controlGains.kp_altitude)/(2*store.Wn_altitude)
        store.Wn_SpeedfromThrottle = math.sqrt(controlGains.ki_SpeedfromThrottle*linearizedModel.a_V2)
        store.Zeta_SpeedfromThrottle = (controlGains.kp_SpeedfromThrottle*linearizedModel.a_V2+linearizedModel.a_V1)/(2*store.Wn_SpeedfromThrottle)
        store.Wn_SpeedfromElevator = math.sqrt(-K_thetaDC*VPC.g0*controlGains.ki_SpeedfromElevator)
        store.Zeta_SpeedfromElevator = (linearizedModel.a_V1-K_thetaDC*VPC.g0*controlGains.kp_SpeedfromElevator)/(2*store.Wn_SpeedfromElevator)
        return store

    except:
        #if math error return all zeros
        store = Controls.controlTuning()
        return store
