'''
Author: Timothy Nguyen (tilonguy@ucsc.edu)
Date: March 2025
This file is the simulator that runs the path following simulation
'''

from . import Simulate
from ..Controls import VehicleClosedLoopControl
from .. import Pthfollowing
from ..Containers.Controls import intendedPath
from ece163.Controls.VehicleTrim import VehicleTrim
from ..Constants import VehiclePhysicalConstants

class PathFollowingSimulate(Simulate.Simulate):
	def __init__(self):
		super().__init__()
		self.inputNames.extend(['mode', 'oX', 'oY','oZ','Course','R','Direction'])
		self.underlyingModel = Pthfollowing.Pathfollow()

		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.vehicle.getVehicleState, 'state',
									['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta', 'chi']))
		# self.dT = 1/50

		self.referenceInput = intendedPath()

	def getVehicleState(self):
		#returns vehicle state
		return self.underlyingModel.vehicle.getVehicleState()

	def takeStep(self, referenceInput=None):
		#updated path follower with new user inputs and increments timestep
		self.time += VehiclePhysicalConstants.dT
		if referenceInput is None:
			referenceInput = self.referenceInput
		self.underlyingModel.update(referenceInput)
		self.recordData([referenceInput.mode, referenceInput.oX, referenceInput.oY, referenceInput.oZ, referenceInput.Course, referenceInput.R, referenceInput.Direction])
		return

	def reset(self):
		#reset relevant models
		self.time = 0
		self.underlyingModel.reset()
		self.takenData.clear()