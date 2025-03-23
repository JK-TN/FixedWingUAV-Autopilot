"""This file is a test harness for the module VehicleAerodynamicsModel.
Author: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tests for gravityForces, CalculateCoeff_alpha, aeroForces, CalculatePropForces,controlForces, updateForces functions. From simple to more complex tests.
It is meant to be run from the Test harnesses directory of the repo with:

python ./TestHarnesses/testChapter4.py (from the root directory) -or-
python testChapter4.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import random
import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Modeling.WindModel as WM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Constants.VehiclePhysicalConstants as VPC

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-4)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(len(a[0]))]
	return all(el_close)




failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean




print("Beginning testing of VAM.gravityForces()")

cur_test = "Gravity Forces"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.pitch = 30*math.pi/180
testState.roll = 20*math.pi/180
testState.yaw = 10*math.pi/180
rF = testVAM.gravityForces(testState)
result = [[rF.Fx],[rF.Fy],[rF.Fz]]
expected = [[0],[0],[107.91]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of VAM.CalculateCoeff_alpha()")

cur_test = "Alpha Test 1"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testalpha = 1
cL,cD,cM = testVAM.CalculateCoeff_alpha(testalpha)
result = [[cL],[cD],[cM]]
expected = [[0.90929],[1.4161],[-2.7265]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "Alpha Test 2"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testalpha = -2
cL,cD,cM = testVAM.CalculateCoeff_alpha(testalpha)
result = [[cL],[cD],[cM]]
expected = [[0.7568],[1.6536],[5.4935]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)




print("Beginning testing of VAM.aeroForces()")

cur_test = "aeroForces Test 1: Pitch Rate"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.q = 2
testState.Va = 5
testState.alpha = 1
aF = testVAM.aeroForces(testState)
result = [[aF.Fx],[aF.Fy],[aF.Fz],[aF.Mx],[aF.My],[aF.Mz]]
expected = [[2.2157],[0],[-16.096],[0],[-6.9190],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


cur_test = "aeroForces Test 2: Roll Rate"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.p = 3
testState.Va = 5
testState.alpha = 1
aF = testVAM.aeroForces(testState)
result = [[aF.Fx],[aF.Fy],[aF.Fz],[aF.Mx],[aF.My],[aF.Mz]]
expected = [[0],[0],[-14.6733],[-11.1848],[-4.5152],[1.5132]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "aeroForces Test 3: Yaw Rate"

testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.r = 5
testState.Va = 2
testState.alpha = 2
aF = testVAM.aeroForces(testState)
result = [[aF.Fx],[aF.Fy],[aF.Fz],[aF.Mx],[aF.My],[aF.Mz]]
expected = [[0],[0],[-2.5369],[3.6551],[-1.4484],[-1.3889]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)




print("Beginning testing of VAM.CalculatePropForces()")

cur_test = "propForces Test 1: No Throttle"

testVAM = VAM.VehicleAerodynamicsModel()
testThrottle = 0
Va = 5
Fp,Mp = testVAM.CalculatePropForces(Va,testThrottle)
result = [[Fp],[Mp]]
expected = [[-0.8805],[0.06925]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "propForces Test 2: Throttle"

testVAM = VAM.VehicleAerodynamicsModel()
testThrottle = 3
Va = 5
Fp,Mp = testVAM.CalculatePropForces(Va,testThrottle)
result = [[Fp],[Mp]]
expected = [[656.0817],[-19.6125]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of VAM.controlForces()")

cur_test = "controlForces Test 1: Ailerons"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()

testControl.Aileron = 1
testControl.Elevator = 0
testControl.Rudder = 0
testControl.Throttle = 1
cF1 = testVAM.controlForces(testState,testControl)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[84.5695],[0],[0],[-2.4012],[0],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "controlForces Test 2: Elevator"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()

testControl.Aileron = 0
testControl.Elevator = 1
testControl.Rudder = 0
testControl.Throttle = 1
cF1 = testVAM.controlForces(testState,testControl)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[84.5695],[0],[0],[-2.4012],[0],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



cur_test = "controlForces Test 3: Rudder"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()

testControl.Aileron = 0
testControl.Elevator = 0
testControl.Rudder = 1
testControl.Throttle = 1
cF1 = testVAM.controlForces(testState,testControl)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[84.5695],[0],[0],[-2.4012],[0],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


cur_test = "controlForces Test 4: All"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()

testControl.Aileron = 0.1
testControl.Elevator = 0.2
testControl.Rudder = 0.5
testControl.Throttle = 3
cF1 = testVAM.controlForces(testState,testControl)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[674.8216],[0],[0],[-19.1609],[0],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of VAM.updateForces()")

cur_test = "updateForces Test 1: Ailerons"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()
testWM = WM.WindModel()
testControl.Aileron = 1
testControl.Elevator = 0
testControl.Rudder = 0
testControl.Throttle = 1
testState.p = 0
testState.q = 0
testState.r = 0
testState.u = 5
testState.v = 0
testState.w = 5
cF1 = testVAM.updateForces(testState,testControl,testWM.wState)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[75.0760],[1.3078],[83.2492],[6.0256],[-7.0829],[-0.5554]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


cur_test = "updateForces Test 2: Elevator"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()

testControl.Aileron = 0
testControl.Elevator = 1
testControl.Rudder = 0
testControl.Throttle = 1
testState.p = 0
testState.q = 0
testState.r = 0
testState.u = 5
testState.v = 0
testState.w = 5
cF1 = testVAM.updateForces(testState,testControl,WM.WindModel().wState)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[76.5125],[0],[81.4798],[-2.5580],[-10.3619],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "updateForces Test 3: Rudder"

testVAM = VAM.VehicleAerodynamicsModel()
testControl = Inputs.controlInputs
testState = States.vehicleState()

testControl.Aileron = 0
testControl.Elevator = 0
testControl.Rudder = 1
testControl.Throttle = 1
testState.p = 0
testState.q = 0
testState.r = 0
testState.u = 5
testState.v = 0
testState.w = 5
cF1 = testVAM.updateForces(testState,testControl,WM.WindModel().wState)
result = [[cF1.Fx],[cF1.Fy],[cF1.Fz],[cF1.Mx],[cF1.My],[cF1.Mz]]
expected = [[75.0760],[3.3131],[83.2492],[-2.4368],[-7.0829],[-3.4839]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of VAM.CalculateAirspeed()")

cur_test = "CalculateAirspeed Test"

testState = States.vehicleState()
testWM = States.windState
testState.roll = 45*math.pi/180
testState.pitch = 72*math.pi/180
testState.yaw = 2*math.pi/180
testState.u = 2
testState.v = 15
testState.w = 0
testWM.Wn = 2
testWM.We = 2
testWM.Wd = 0
testWM.Wu = 3
testWM.Wv = 5
testWM.Ww = -4
Va,alpha,beta = testVAM.CalculateAirspeed(testState,testWM)
result = [[Va],[alpha],[beta]]
expected = [[8.4806],[1.2309],[1.0468]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of WM.CreateDrydenTransferFns()")

cur_test = "CreateDrydenTransferFns Test 1: Nominal Conditions"

testWM = WM.WindModel()
testWM.CreateDrydenTransferFns(VPC.dT,VPC.InitialSpeed,VPC.DrydenLowAltitudeLight)
result = [[testWM.phiU[0][0]],[testWM.gammaU[0][0]],[testWM.hU[0][0]]]
expected = [[0.9987],[0.0099],[0.2990]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "CreateDrydenTransferFns Test 2: No Wind"

testWM = WM.WindModel()
testWM.CreateDrydenTransferFns(VPC.dT,VPC.InitialSpeed,VPC.DrydenNoWind)
result = [[testWM.phiU[0][0]],[testWM.gammaU[0][0]],[testWM.hU[0][0]]]
expected = [[1],[0],[1]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


cur_test = "CreateDrydenTransferFns Test 3: Airspeed 0"

testWM = WM.WindModel()
try:
	testWM.CreateDrydenTransferFns(VPC.dT,0,VPC.DrydenLowAltitudeLight)
except ArithmeticError:
	print("passed!")
	evaluateTest(cur_test, True)
except:
	print("failed :(")
	evaluateTest(cur_test, False)




print("Beginning testing of WM.Update()")

cur_test = "WM.Update Test 1: No noise"

testWM = WM.WindModel()
testWM.CreateDrydenTransferFns(VPC.dT,VPC.InitialSpeed,VPC.DrydenLowAltitudeLight)
testWM.Update(0, 0, 0)
result = [[testWM.wState.Wu],[testWM.wState.Wv],[testWM.wState.Ww]]
expected = [[0],[0],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "WM.Update Test 2: Some noise"

testWM = WM.WindModel()
testWM.CreateDrydenTransferFns(VPC.dT,VPC.InitialSpeed,VPC.DrydenLowAltitudeLight)
for i in range(1000):
	testWM.Update(-1.32, 0.523, 1.232)
result = [[testWM.wState.Wu],[testWM.wState.Wv],[testWM.wState.Ww]]
expected = [[-2.2529],[0.8631],[0.7004]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]