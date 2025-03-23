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
from ece163.Containers import Linearized

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-3)

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
print("Beginning testing of PIDControl.update()")

cur_test = "PIDControl.update() test1: No change"

testPID = VCLC.PIDControl()
testPID.kd = 2
testPID.ki = 0.1
testPID.kp = 10
testPID.highLimit = 50
testPID.lowLimit = 0

result = [[testPID.Update(1,1,3)]]
expected = [[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "PIDControl.update() test2: Iteration"

testPID = VCLC.PIDControl()
testPID.kd = 2
testPID.ki = 0.1
testPID.kp = 10
testPID.highLimit = 50
testPID.lowLimit = 0
testPID.Update(10,4,1)
testPID.Update(10,4,1)
testPID.Update(10,4,1)
testPID.Update(10,4,1)

result = [[testPID.Update(10,4,1)]]
expected = [[50]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of PIControl.update()")

cur_test = "PIControl.update() test1: Nominal Diff"

testPI = VCLC.PIControl()
testPI.ki = 0.1
testPI.kp = 10
testPI.highLimit = 200
testPI.lowLimit = -50

result = [[testPI.Update(20,5)]]
expected = [[150.0075]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "PIControl.update() test2: No Diff"

testPI = VCLC.PIControl()
testPI.ki = 0.1
testPI.kp = 10
testPI.highLimit = 200
testPI.lowLimit = -50

result = [[testPI.Update(5,5)]]
expected = [[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of PDControl.update()")

cur_test = "PDControl.update() test1: Nominal Diff"

testPD = VCLC.PDControl()
testPD.kd = 15
testPD.kp = 10
testPD.highLimit = 200
testPD.lowLimit = -50

result = [[testPD.Update(20,5,4)]]
expected = [[90]]
if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "PDControl.update() test2: Sequences"

testPD = VCLC.PDControl()
testPD.kd = 15
testPD.kp = 10
testPD.highLimit = 200
testPD.lowLimit = -50
testPD.Update(5,3,1)
testPD.Update(3,2,5)
testPD.Update(20,5,3)
testPD.Update(20,5,4)

result = [[testPD.Update(14,5,4)]]
expected = [[30]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Beginning testing of VehicleClosedLoopControl.UpdateControlCommands()")

cur_test = "VehicleClosedLoopControl.UpdateControlCommands() test1: Holding"
testV = VCLC.VehicleClosedLoopControl()
testState = States.vehicleState()
testCmd = Controls.referenceCommands()
testCmd.commandedAltitude = 15
testCmd.commandedAirspeed = 10
testCmd.commandedPitch = 0
testCmd.commandedRoll = 0
testCmd.commandedCourse = 0
testState.u  = 5
testState.pd = -10
testV.setControlGains(Controls.controlGains(1,4,3,2,1,6,4,3,7,3,8,3,4,2,9))
res = testV.UpdateControlCommands(testCmd,testState)
result = [[res.Throttle],[res.Aileron],[res.Elevator],[res.Rudder]]
expected = [[1],[0],[0.43633],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


cur_test = "VehicleClosedLoopControl.UpdateControlCommands() test2: Descending"
testV = VCLC.VehicleClosedLoopControl()
testV.mode = Controls.AltitudeStates.DESCENDING
testState = States.vehicleState()
testCmd = Controls.referenceCommands()
testCmd.commandedAltitude = 0
testCmd.commandedAirspeed = 10
testCmd.commandedPitch = 1
testCmd.commandedRoll = 0.5
testCmd.commandedCourse = 0.5
testState.u  = 5
testState.pd = -10
testV.setControlGains(Controls.controlGains(1,4,3,2,1,6,4,3,7,3,8,3,4,2,9))
res = testV.UpdateControlCommands(testCmd,testState)
result = [[res.Throttle],[res.Aileron],[res.Elevator],[res.Rudder]]
expected = [[1],[0.43633],[-0.43633],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

cur_test = "VehicleClosedLoopControl.UpdateControlCommands() test3: Climbing"
testV = VCLC.VehicleClosedLoopControl()
testV.mode = Controls.AltitudeStates.CLIMBING
testState = States.vehicleState()
testCmd = Controls.referenceCommands()
testCmd.commandedAltitude = 100
testCmd.commandedAirspeed = 10
testCmd.commandedPitch = 0
testCmd.commandedRoll = 0
testCmd.commandedCourse = 0.5
testState.u  = 10
testState.pd = -10
testV.setControlGains(Controls.controlGains(1,4,3,2,1,6,4,3,7,3,8,3,4,2,9))
res = testV.UpdateControlCommands(testCmd,testState)
result = [[res.Throttle],[res.Aileron],[res.Elevator],[res.Rudder]]
expected = [[1],[0.43633],[0.43633],[0]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Beginning testing of VehicleControlGains.computeGains()")

cur_test = "VehicleControlGains.computeGains() test"
testTune = Controls.controlTuning(3,6,2,7,3,2,7,0,3,1,2,6,4,3)
testLinear = Linearized.transferFunctions(3,2,1,5,7,3,6,2,8,4,2,6,4,3,2,7)

if VCG.computeGains(testTune,testLinear) == Controls.controlGains(4.5, 15.0, 0.001,1.0, 2.25, 8.5626, 1.2232, 10.75, -0.5, 2.2790, 3.4186, 10.5, 2.0, -2.4393, -1.8585):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of VehicleControlGains.computeTuningParameters()")

cur_test = "VehicleControlGains.computeTuningParameters() test"
testGain = Controls.controlGains(4.5, 15.0, 0.001,1.0, 2.25, 8.5626, 1.2232, 10.75, -0.5, 2.2790, 3.4186, 10.5, 2.0, -2.4393, -1.8585)
testLinear = Linearized.transferFunctions(3,2,1,5,7,3,6,2,8,4,2,6,4,3,2,7)

if VCG.computeTuningParameters(testGain,testLinear) == Controls.controlTuning(3,6,2,7,3,2,7,0,3,1,2,6,4,3):
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