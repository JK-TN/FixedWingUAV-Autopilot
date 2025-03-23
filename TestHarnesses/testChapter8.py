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
import ece163.Controls.VehicleEstimator as VehicleEstimator

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
print("Beginning testing of estimateAttitude()")

cur_test = "estimateAttitude() test"

testG = VehicleEstimator.VehicleEstimator()
testG.setEstimatorGains(Controls.VehicleEstimatorGains(1,6,2,8,2,8,3,5,2,6,1,2,5))
testG.setEstimatorBiases([[2],[3],[5]],3,2,8,2)
testS = Sensors.vehicleSensors(5,2,6,1,7,2,0,3,7,2,1,4,6,81,3,7)
testE = States.vehicleState(8,2,6,9,3,2,4,0,3,1,0,3,)

a,b,c = testG.estimateAttitude(testS,testE)
if compareVectors(a,[[2.0164], [3.0291], [4.9874]]) and compareVectors(b,[[2.9835], [-1.0291], [1.0125]]):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of estimateAltitude()")

cur_test = "estimateAltitude() test"

testG = VehicleEstimator.VehicleEstimator()
testG.setEstimatorGains(Controls.VehicleEstimatorGains(1,6,2,8,2,8,3,5,2,6,1,2,5))
testG.setEstimatorBiases([[2],[3],[5]],3,2,8,2)
testS = Sensors.vehicleSensors(5,2,6,1,7,2,0,3,7,2,1,4,6,81,3,7)
testE = States.vehicleState(8,2,6,9,3,2,4,0,3,1,0,3,)

a,b,c = testG.estimateAltitude(testS,testE)
if a == -7.7943182144936785 and b==8.568178550632183 and c== 2:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



print("Beginning testing of estimateCourse()")

cur_test = "estimateCourse() test"

testG = VehicleEstimator.VehicleEstimator()
testG.setEstimatorGains(Controls.VehicleEstimatorGains(1,6,2,8,2,8,3,5,2,6,1,2,5))
testG.setEstimatorBiases([[2],[3],[5]],3,2,8,2)
testS = Sensors.vehicleSensors(5,2,6,1,7,2,0,3,7,2,1,4,6,81,3,7)
testE = States.vehicleState(8,2,6,9,3,2,4,0,3,1,0,3,)

a,b = testG.estimateCourse(testS,testE)
if a == 2 and b==-2.6796418958055876:
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


print("Beginning testing of estimateAirspeed()")

cur_test = "estimateAirspeed() test"

testG = VehicleEstimator.VehicleEstimator()
testG.setEstimatorGains(Controls.VehicleEstimatorGains(1,6,2,8,2,8,3,5,2,6,1,2,5))
testG.setEstimatorBiases([[2],[3],[5]],3,2,8,2)
testS = Sensors.vehicleSensors(5,2,6,1,7,2,0,3,7,2,1,4,6,81,3,7)
testE = States.vehicleState(8,2,6,9,3,2,4,0,3,1,0,3,)

a,b = testG.estimateAirspeed(testS,testE)
if a == 3.0843955785067125 and b==9.168142288007315:
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