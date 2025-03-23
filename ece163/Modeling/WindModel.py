"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tools to perform to calculate and enforce wind models in the simulation.
"""


import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel:
    def __init__(self,dT = VPC.dT,Va=VPC.InitialSpeed,drydenParameters=VPC.DrydenNoWind):
        #storage of parameters and variables
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParameters

        #Wind state object
        self.wState = States.windState()

        #Variables to hold X over iterations
        self.xU = [[0]]
        self.xV = [[0],[0]]
        self.xW = [[0],[0]]


        #Coefficient Storage
        self.phiU = [[0]]
        self.gammaU = [[0]]
        self.hU = [[0]]
        self.phiV = [[0,0],[0,0]]
        self.gammaV = [[0],[0]]
        self.hV = [[0,0]]
        self.phiW = [[0,0],[0,0]]
        self.gammaW = [[0],[0]]
        self.hW = [[0,0]]

        #Call CreateDrydenTransferFns() to populate above variables
        self.CreateDrydenTransferFns(self.dT,self.Va,self.drydenParameters)

    def reset(self):
        #reset all containers and variables
        self.dT = VPC.dT
        self.Va = VPC.InitialSpeed
        self.drydenParameters = VPC.DrydenNoWind
        self.wState = States.windState()

        self.xU = [[0]]
        self.xV = [[0],[0]]
        self.xW = [[0],[0]]

        self.phiU = [[0]]
        self.gammaU = [[0]]
        self.hU = [[0]]
        self.phiV = [[0,0],[0,0]]
        self.gammaV = [[0],[0]]
        self.hV = [[0,0]]
        self.phiW = [[0,0],[0,0]]
        self.gammaW = [[0],[0]]
        self.hW = [[0,0]]
        self.CreateDrydenTransferFns(self.dT,self.Va,self.drydenParameters)

    def getWind(self):
        #return wind state
        return self.wState

    def setWind(self,windState):
        #store provided wind state
        self.wState = windState

    def setWindModelParameters(self,Wn=0,We=0,Wd=0,drydenParameters=VPC.DrydenNoWind):

        #update coefficients bades on Va and dryden parameters
        self.CreateDrydenTransferFns(self.dT, self.Va, drydenParameters)

        #store static wind
        self.wState.Wn = Wn
        self.wState.We = We
        self.wState.Wd = Wd

    def getDrydenTransferFns(self):
        #return dryden coefficients
        return self.phiU, self.gammaU, self.hU, self.phiV, self.gammaV, self.hV, self.phiW, self.gammaW, self.hW

    def CreateDrydenTransferFns(self,dT,Va,drydenParameters):
        if Va<=0:
            raise ArithmeticError("Va must not be negative or zero")
        elif drydenParameters==VPC.DrydenNoWind:
            self.phiU = [[1]]
            self.phiV = [[1,0],[0,1]]
            self.phiW = [[1,0],[0,1]]

            self.gammaU = [[0]]
            self.gammaV = [[0],[0]]
            self.gammaW = [[0],[0]]

            self.hU = [[1]]
            self.hV = [[1,1]]
            self.hW = [[1,1]]

        else:
            self.phiU = [[math.exp(-(Va*dT)/drydenParameters.Lu)]]
            self.gammaU = [[(drydenParameters.Lu/Va)*(1-math.exp(-(Va*dT)/drydenParameters.Lu))]]
            self.hU = [[drydenParameters.sigmau*math.sqrt((2*Va)/(math.pi*drydenParameters.Lu))]]

            mPhiv = [[1-((Va*dT)/drydenParameters.Lv),-((Va/drydenParameters.Lv)**2)*dT],[dT,1+((Va*dT)/drydenParameters.Lv)]]
            mPhiw = [[1-((Va*dT)/drydenParameters.Lw),-((Va/drydenParameters.Lw)**2)*dT],[dT,1+((Va*dT)/drydenParameters.Lw)]]

            mGammav = [[dT],[(((drydenParameters.Lv/Va)**2)*(math.exp((Va*dT)/drydenParameters.Lv)-1))-((dT*drydenParameters.Lv)/Va)]]
            mGammaw = [[dT],[(((drydenParameters.Lw/Va)**2)*(math.exp((Va*dT)/drydenParameters.Lw)-1))-((dT*drydenParameters.Lw)/Va)]]

            mHv = [[1,Va/(math.sqrt(3)*drydenParameters.Lv)]]
            mHw = [[1, Va / (math.sqrt(3) * drydenParameters.Lw)]]

            self.phiV = MatrixMath.scalarMultiply(math.exp(-(Va*dT)/drydenParameters.Lv),mPhiv)
            self.gammaV = MatrixMath.scalarMultiply(math.exp(-(Va * dT) / drydenParameters.Lv), mGammav)
            self.hV = MatrixMath.scalarMultiply(drydenParameters.sigmav*math.sqrt((3*Va)/(math.pi*drydenParameters.Lv)),mHv)

            self.phiW = MatrixMath.scalarMultiply(math.exp(-(Va*dT)/drydenParameters.Lw),mPhiw)
            self.gammaW = MatrixMath.scalarMultiply(math.exp(-(Va*dT)/drydenParameters.Lw),mGammaw)
            self.hW = MatrixMath.scalarMultiply(drydenParameters.sigmaw * math.sqrt((3 * Va)/(math.pi*drydenParameters.Lw)),mHw)

    def Update(self,uu=None,uv=None,uw=None):

        #if no values are given use random gaussian
        if uu is None:
            uu = random.gauss(0, 1)
        else:
            uu = uu
        if uv is None:
            uv = random.gauss(0, 1)
        else:
            uv = uv
        if uw is None:
            uw = random.gauss(0, 1)
        else:
            uw = uw

        #compute new X and gust for u, v, and w
        newxu = MatrixMath.add(MatrixMath.multiply(self.phiU,self.xU),MatrixMath.scalarMultiply(uu,self.gammaU))
        self.wState.Wu = MatrixMath.multiply(self.hU,newxu)[0][0]
        self.xU = newxu

        newxv = MatrixMath.add(MatrixMath.multiply(self.phiV,self.xV),MatrixMath.scalarMultiply(uv,self.gammaV))
        self.wState.Wv = MatrixMath.multiply(self.hV,newxv)[0][0]
        self.xV = newxv

        newxw = MatrixMath.add(MatrixMath.multiply(self.phiW,self.xW),MatrixMath.scalarMultiply(uw,self.gammaW))
        self.wState.Ww = MatrixMath.multiply(self.hW,newxw)[0][0]
        self.xW = newxw