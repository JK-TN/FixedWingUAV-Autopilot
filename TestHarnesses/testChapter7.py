"""This file is a test harness for the module VehicleClosedLooopControl and VehicleControlGains.
Author: Timothy Nguyen (tilonguy@ucsc.edu)
It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter6.py (from the root directory) -or-
python testChapter6.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehiclePerturbationModels module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Controls.VehicleControlGains as VCG
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Controls as Controls
import ece163.Containers.Sensors as Sensors
import ece163.Sensors.SensorsModel as SensorsModel
import ece163.Constants.VehiclePhysicalConstants as VPC
from ece163.Containers import Linearized

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-2)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(len(a))]
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


#%% PUT A TEST HERE?
print("Beginning testing of GaussMarkovXYZ()")

cur_test = "GaussMarkovXYZ() test1: X Filled"

testG = SensorsModel.GaussMarkovXYZ(VPC.dT,tauX=1000000.0,etaX=0.0,tauY=None,etaY=None,tauZ=None,etaZ=None )

r = [[0],[0],[0]]
r[0][0],r[1][0],r[2][0] = testG.update(1,2,3)
expected = [[1],[2],[3]]

if compareVectors(r,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "GaussMarkovXYZ() tes2: XY Filled"

testG = SensorsModel.GaussMarkovXYZ(VPC.dT,tauX=1000000.0,etaX=0.0,tauY=200,etaY=1,tauZ=None,etaZ=None )

r = [[0],[0],[0]]
r[0][0],r[1][0],r[2][0] = testG.update(2,1,5)
expected = [[2],[1],[5]]

if compareVectors(r,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "GaussMarkovXYZ() test3: XYZ Filled"

testG = SensorsModel.GaussMarkovXYZ(VPC.dT,tauX=1000000.0,etaX=0.0,tauY=2000000,etaY=3,tauZ=1500000,etaZ=2 )

r = [[0],[0],[0]]
r[0][0],r[1][0],r[2][0] = testG.update(2.5,8,2)
expected = [[2.5],[8],[2]]

if compareVectors(r,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of initializeSigmas()")

cur_test = "initializeSigmas() test1"

testSM = SensorsModel.SensorsModel()
result = testSM.initializeSigmas(2,1,5,7,9,2,1,3,6)

expected = Sensors.vehicleSensors(gyro_x=2, gyro_y=2, gyro_z=2, accel_x=1, accel_y=1, accel_z=1, mag_x=5, mag_y=5, mag_z=5, baro=7, pitot=9, gps_n=2, gps_e=2, gps_alt=1, gps_sog=3, gps_cog=6)

if result==expected:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of updateAccelsTrue()")

cur_test = "updateAccelsTrue() test1"

testSM = SensorsModel.SensorsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.u = 1
testState.v = 2
testState.w = 3
testState.p = 0.5
testState.q = 0.25
testState.r = 0
testDot.u = 0
testDot.v = 4
testDot.w = 1
result = testSM.updateAccelsTrue(testState,testDot)
expected = (0.75, 2.5, -8.06)

if result==expected:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Beginning testing of updateGyrosTrue()")

cur_test = "updateGyrosTrue() test1"

testSM = SensorsModel.SensorsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.p = 0.5
testState.q = 0.25
testState.r = 0
result = testSM.updateGyrosTrue(testState)
expected = (0.5,0.25,0)

if result==expected:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of updateMagsTrue()")

cur_test = "updateMagsTrue() test1"

testSM = SensorsModel.SensorsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.yaw = 0.5
testState.pitch = 0.25
testState.roll = 0
result = testSM.updateMagsTrue(testState)
print(result)
expected = (11551.140567271741, -6267.327515043595, 45704.954253825774)

if result==expected:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of updateGPSTrue()")

cur_test = "updateGPSTrue() test1"

testSM = SensorsModel.SensorsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.pn = 75.2
testState.pe = 3
testState.pd = 500
result = testSM.updateGPSTrue(testState,testDot)
expected = (75.2, 3, -500, 0.0, 0.0)

if result==expected:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of updateSensorsTrue()")

cur_test = "updateGPSTrue() test1"

testSM = SensorsModel.SensorsModel()
pstS = Sensors.vehicleSensors(0,0,0,0,0,0,0,0,0,0,0,12,51,20,2,18)
testState = States.vehicleState(1,45,2,5,7,2,1,7,2,7,2,1)
testDot = States.vehicleState(6,1,5,2,4,1,3,8,6,4,1,6)
result = testSM.updateSensorsTrue(pstS,testState,testDot)
print(result)
expected = Sensors.vehicleSensors(gyro_x=7, gyro_y=2, gyro_z=1, accel_x=5.445038533431321, accel_y=-11.72496473733351, accel_z=43.077730914847216, mag_x=-14595.77329136595, mag_y=45177.14671912568, mag_z=-2764.037314444227, baro=101349.882084, pitot=49.45980000000001, gps_n=1, gps_e=45, gps_alt=-2, gps_sog=6.082762530298219, gps_cog=0.16514867741462683)

if result==expected:
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