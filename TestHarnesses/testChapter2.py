"""
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tests for euler2DCM, dcm2Euler, NED2ENU, and getNewPoints functions. From simple to more complex tests.
"""

"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-1)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-0], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



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


#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%%  

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""
cur_test = "Euler2dcm test 2"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(math.pi, 0, 0)
orig_vec = [[1],[3],[-2]]
expected_vec = [[-1],[-3],[-2]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "Euler2dcm test 3"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(-90*math.pi/180, 135*math.pi/180, 420*math.pi/180)
orig_vec = [[1],[-1],[1]]
expected_vec = [[-1.414],[0.5],[-0.866]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%% dcm2Euler():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "dcm2Euler test 1"
dcm = [[0,1,0],[1,0,0],[0,0,-1]]
expected_vec = [[90*math.pi/180],[0],[math.pi]]
yaw,pitch,roll = Rotations.dcm2Euler(dcm)
actual_vec = [[yaw],[pitch],[roll]]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "dcm2Euler test 2"
dcm = [[-0.5,0.5,-0.707],[-0.5,0.5,0.707],[0.707,0.707,0]]
expected_vec = [[135*math.pi/180],[45*math.pi/180],[90*math.pi/180]]
yaw,pitch,roll = Rotations.dcm2Euler(dcm)
actual_vec = [[yaw],[pitch],[roll]]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%% ned2enu():
print("Beginning testing of Rotations.ned2enu()")

cur_test = "ned2enu test 1"
input = [[0,1,2]]
expected_vec = [[1,0,-2]]
actual_vec = Rotations.ned2enu(input)
if not evaluateTest(cur_test, compareVectors(mm.transpose(expected_vec), mm.transpose(actual_vec)) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "ned2enu test 2"
input = [[7.5,-3,-19.2],[1,4,-9.8]]
expected_vec = [[-3,7.5,19.2],[4,1,9.8]]
actual_vec = Rotations.ned2enu(input)
if not evaluateTest(cur_test, compareVectors(mm.transpose(expected_vec), mm.transpose(actual_vec)) ):
	print(f"{expected_vec} != {actual_vec}")


#%% getNewPoints():
print("Beginning testing of VehicleGeometry.getNewPoints()")

cur_test = "getNewPoints test 1"
expected_vec = [[2,3.5,1]]
v = VG.VehicleGeometry()
actual_vec = [VG.VehicleGeometry.getNewPoints(v,2.5,2,-1,0,0,0)[0]]
if not evaluateTest(cur_test, compareVectors(mm.transpose(expected_vec), mm.transpose(actual_vec)) ):
	print(f"{expected_vec} != {actual_vec}")

cur_test = "getNewPoints test 2"
expected_vec = [[2.707,2.5,1.707]]
v = VG.VehicleGeometry()
actual_vec = [VG.VehicleGeometry.getNewPoints(v,2.5,2,-1,90*math.pi/180,45*math.pi/180,math.pi)[0]]
if not evaluateTest(cur_test, compareVectors(mm.transpose(expected_vec), mm.transpose(actual_vec)) ):
	print(f"{expected_vec} != {actual_vec}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]