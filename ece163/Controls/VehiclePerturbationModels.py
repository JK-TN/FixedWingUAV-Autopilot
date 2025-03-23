"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains functions to perform to calculate transfer functions and thrust derivatives.
"""

import math

from matplotlib.backend_bases import NavigationToolbar2

from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

def CreateTransferFunction(trimState,trimInputs):

    #storage object for transfer function variables
    output = Linearized.transferFunctions()

    #calculating partial derivatives
    dTdt = dThrust_dThrottle(trimState.Va,trimInputs.Throttle)
    dTdVa = dThrust_dVa(trimState.Va,trimInputs.Throttle)

    #filling in trim variables
    output.Va_trim = trimState.Va
    output.alpha_trim = trimState.alpha
    output.beta_trim = trimState.beta
    output.theta_trim = trimState.pitch
    output.phi_trim =  trimState.roll

    #gamma is pitch with alpha subtracted from it
    output.gamma_trim = trimState.pitch-trimState.alpha

    #calculating roll angle coefficients
    output.a_phi1 = -0.5*VPC.rho*(trimState.Va**2)*VPC.S*VPC.b*VPC.Cpp*(VPC.b/(2*trimState.Va))
    output.a_phi2 = 0.5*VPC.rho*(trimState.Va**2)*VPC.S*VPC.b*VPC.CpdeltaA

    #calculating sideslip coefficients
    output.a_beta1 = -((VPC.rho*trimState.Va*VPC.S)/(2*VPC.mass))*VPC.CYbeta
    output.a_beta2 = ((VPC.rho*trimState.Va*VPC.S)/(2*VPC.mass))*VPC.CYdeltaR

    #calculating pitch angle coefficients
    output.a_theta1 = -((VPC.rho*(trimState.Va**2)*VPC.c*VPC.S)/(2*VPC.Jyy))*VPC.CMq*(VPC.c/(2*trimState.Va))
    output.a_theta2 = -((VPC.rho*(trimState.Va**2)*VPC.c*VPC.S)/(2*VPC.Jyy))*VPC.CMalpha
    output.a_theta3 = ((VPC.rho*(trimState.Va**2)*VPC.c*VPC.S)/(2*VPC.Jyy))*VPC.CMdeltaE

    #Calculating airspeed coefficients
    output.a_V1 = ((VPC.rho*output.Va_trim*VPC.S)/VPC.mass)*(VPC.CD0+VPC.CDalpha*output.alpha_trim+VPC.CDdeltaE*trimInputs.Elevator)-(1/VPC.mass)*dTdVa
    output.a_V2 = (1/VPC.mass)*dTdt
    output.a_V3 = VPC.g0*math.cos(output.theta_trim-output.alpha_trim)

    return output

def dThrust_dThrottle(Va,Throttle,epsilon=0.01):

    #Aerodynamics model to gain access to CalculatePropForces()
    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()

    #calculate propeller force with and without epsilon
    F,M = VAM.CalculatePropForces(Va,Throttle)
    Fe,Me = VAM.CalculatePropForces(Va,Throttle+epsilon)

    #return partial derivative of thrust with respect to throttle
    return (Fe-F)/epsilon

def dThrust_dVa(Va,Throttle,epsilon=0.5):

    # Aerodynamics model to gain access to CalculatePropForces()
    VAM = VehicleAerodynamicsModel.VehicleAerodynamicsModel()

    # calculate propeller force with and without epsilon
    F, M = VAM.CalculatePropForces(Va, Throttle)
    Fe, Me = VAM.CalculatePropForces(Va + epsilon, Throttle)

    # return partial derivative of thrust with respect to airspeed
    return (Fe - F) / epsilon