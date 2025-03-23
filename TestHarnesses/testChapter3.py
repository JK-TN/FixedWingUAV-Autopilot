"""This file is a test harness for the module VehicleDynamicsModel.
Autor: Timothy Nguyen (tilonguy@ucsc.edu)
This file contains tests for IntegrateState,ForwardEuler,Rexp, and derivative functions. From simple to more complex tests.
It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter3.py (from the root directory) -or-
python testChapter3.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
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


#%% Derivative():
print("Beginning testing of VDM.Derivative(), subtest of [pe,pn,pd]")

cur_test = "Derivative test p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.R = Rotations.euler2DCM(0.0,testState.pitch,0.0)
testState.u = 10
testDot = testVDM.derivative(testState, testFm)

print("With a velocity of u = 10 m/s, and pitch = 30deg:\n")
resultPdot = [[testDot.pn],[testDot.pe],[testDot.pd]]
expectedPdot = [[10*math.sqrt(3)/2],[0],[-10/2]]

if compareVectors(resultPdot,expectedPdot):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)



#%%  

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

cur_test = "Derivative test 2 p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 40*math.pi/180
testState.roll = 40*math.pi/180
testState.R = Rotations.euler2DCM(0.0,testState.pitch,testState.roll)
testState.u = 10
testState.v = 2
testState.w = 4
testDot = testVDM.derivative(testState, testFm)

print("More velocities and angles:\n")
resultPdot = [[testDot.pn],[testDot.pe],[testDot.pd]]
expectedPdot = [[10.456411759547265],[-1.039061552508201],[-3.0957719885193242]]

if compareVectors(resultPdot,expectedPdot):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of [u,v,w]")

cur_test = "Derivative test 3 -skew * v + accel"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 40*math.pi/180
testState.roll = 40*math.pi/180
testState.yaw = 57.2*math.pi/180
testState.R = Rotations.euler2DCM(testState.yaw,testState.pitch,testState.roll)
testState.u = 2
testState.v = 1
testState.w = 6
testFm.Fx = 1
testFm.Fy = 3
testFm.Fz = 2
testDot = testVDM.derivative(testState, testFm)

resultPdot = [[testDot.u],[testDot.v],[testDot.w]]
expectedPdot = [[0.09090909090909091],[0.2727272727272727],[0.18181818181818182]]

if compareVectors(resultPdot,expectedPdot):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of [roll,pitch,yaw]")

cur_test = "Derivative test 4 Rm*p"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 70*math.pi/180
testState.roll = 20*math.pi/180
testState.yaw = 67.5*math.pi/180
testState.R = Rotations.euler2DCM(testState.yaw,testState.pitch,testState.roll)
testState.p = 0
testState.q = 2.5
testState.r = 16
testDot = testVDM.derivative(testState, testFm)

resultPdot = [[testDot.roll],[testDot.pitch],[testDot.yaw]]
expectedPdot = [[43.657779661363456],[-3.123090741245928],[46.459638711273946]]

if compareVectors(resultPdot,expectedPdot):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of R")

cur_test = "Derivative test 5 -[wx]*R"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.roll = 12*math.pi/180
testState.yaw = 45*math.pi/180
testState.R = Rotations.euler2DCM(testState.yaw,testState.pitch,testState.roll)
testState.p = 1
testState.q = 4
testState.r = 0.005
testDot = testVDM.derivative(testState, testFm)

resultPdot1 = [[testDot.R[0][0]],[testDot.R[1][0]],[testDot.R[2][0]]]
resultPdot2 = [[testDot.R[0][1]],[testDot.R[1][1]],[testDot.R[2][1]]]
resultPdot3 = [[testDot.R[0][2]],[testDot.R[1][2]],[testDot.R[2][2]]]
expectedPdot1 = [[-1.9744634034124828],[0.4897813050268322],[3.0676366610308046]]
expectedPdot2 = [[-0.7914207236760931],[0.19574977209643532],[1.6843270580703535]]
expectedPdot3 = [[-3.3875023995151365],[0.849600670886274],[-2.1800568059919554]]

if compareVectors(resultPdot1,expectedPdot1) and compareVectors(resultPdot2,expectedPdot2) and compareVectors(resultPdot3,expectedPdot3):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)


cur_test = "Derivative test 6 J^-1(M-[wx]*J*p)"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testFm.Mx = 2
testFm.My = 1
testFm.Mz = -10
testState.p = 1
testState.q = 2.5
testState.r = 7
testDot = testVDM.derivative(testState, testFm)

resultPdot = [[testDot.p],[testDot.q],[testDot.r]]
print(resultPdot)
expectedPdot = [[-11.640931691669781],[11.73691629955947],[-8.121130287479842]]

if compareVectors(resultPdot,expectedPdot):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

#note: IntegrateState uses forardEuler() and Rexp() as helpers so they will be tested in here too
print("Beginning testing of VDM.IntegrateState(), subtest of [pe,pn,pd]")

cur_test = "Integral test forward euler p + dp/dt*dT"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.pn = 0
testState.pe = 1
testState.pd = 2.5
testDot.pn = 2
testDot.pe = 0.5
testDot.pd = 0
testnew = testVDM.IntegrateState(testVDM.dT,testState, testDot)

result = [[testnew.pn],[testnew.pe],[testnew.pd]]
expected = [[0.02],[1.005],[2.5]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of [u,v,w]")

cur_test = "Integral test forward euler v + dv/dt*dT"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.u = 1.7
testState.v = 1.2
testState.w = 3
testDot.u = 3
testDot.v = -4
testDot.w = 2.3
testnew = testVDM.IntegrateState(testVDM.dT,testState, testDot)

result = [[testnew.u],[testnew.v],[testnew.w]]
expected = [[1.73],[1.16],[3.023]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of [p,q,r]")

cur_test = "Integral test forward euler w + dw/dt*dT"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.p = 19
testState.q = 0
testState.r = -7
testDot.p = 24
testDot.q = 0.001
testDot.r = 0
testnew = testVDM.IntegrateState(testVDM.dT,testState, testDot)

result = [[testnew.p],[testnew.q],[testnew.r]]
expected = [[19.24],[0.00001],[-7]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of [roll,pitch,yaw]")

cur_test = "Integral test attitude propogation by matrix exponential dcm2Euler(Rt+0.01)"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.p = 2
testState.q = 0
testState.r = -1
testState.roll = 15*math.pi/180
testState.pitch = 20*math.pi/180
testState.yaw = 80*math.pi/180
testDot.p = 3
testDot.q = -2
testDot.r = 1
testnew = testVDM.IntegrateState(testVDM.dT,testState, testDot)

result = [[testnew.roll],[testnew.pitch],[testnew.yaw]]
expected = [[0.020150165010728943],[2.504481197539728e-07],[-0.009950334157247378]]

if compareVectors(result,expected):
	print("passed!")
	evaluateTest(cur_test,True)
else:
	print("failed :(")
	evaluateTest(cur_test, False)

print("Subtest of R")

cur_test = "Integral test  Rexp*R"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testDot = States.vehicleState()
testState.p = 1
testState.q = 2
testState.r = -3
testState.roll = 72*math.pi/180
testState.pitch = 45*math.pi/180
testState.yaw = 0*math.pi/180
testDot.p = 4
testDot.q = -0.2
testDot.r = 2
testnew = testVDM.IntegrateState(testVDM.dT,testState, testDot)

resultPdot1 = [[testnew.R[0][0]],[testnew.R[1][0]],[testnew.R[2][0]]]
resultPdot2 = [[testnew.R[0][1]],[testnew.R[1][1]],[testnew.R[2][1]]]
resultPdot3 = [[testnew.R[0][2]],[testnew.R[1][2]],[testnew.R[2][2]]]
print(testnew.R)
expectedPdot1 = [[0.9993532702804189],[0.029994972656848085],[0.019832871580958738]]
expectedPdot2 = [[-0.029791098403913384],[0.999501033119154],[-0.010496439855118]]
expectedPdot3 = [[-0.020137816061336303],[0.009898808466613337],[0.9997482092783267]]

if compareVectors(resultPdot1,expectedPdot1) and compareVectors(resultPdot2,expectedPdot2) and compareVectors(resultPdot3,expectedPdot3):
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