'''
Author: Timothy Nguyen (tilonguy@ucsc.edu)
Date: March 2025
This file is a widget for path following inputs to set the path
'''
from math import radians

import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets

import ece163.Constants.VehiclePhysicalConstants as VehiclePhysicalConstants
from . import doubleInputWithLabel
from .. import Pthfollowing
from ..Controls import VehicleTrim
import sys
import os
import pickle

import math
import threading

defaultPathParameters = [('Origin X', 0),('Origin Y', 0),('Origin Z', 100), ('Course', 0), ('R', 50),('Direction', 1),]
defaultPathFileName = 'VehiclePath_Data.pickle'


class vehiclePathWidget(QtWidgets.QWidget):
	PathSignal = QtCore.pyqtSignal(tuple)
	def __init__(self, guiControls, callBackOnSuccesfulPath=None, parent=None):
		"""
		widget to calculate path within a gui

		:param guiControls: this must be the main gui to allow access to stuff like reset
		:param callBackOnSuccesfulPath: called if trim was succesful with arguments of the path parameters
		"""
		super().__init__(parent)
		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)
		self.guiControls = guiControls
		self.callBack = callBackOnSuccesfulPath

		self.PathInstance = Pthfollowing.Pathfollow()

		try:
			with open(os.path.join(sys.path[0], defaultPathFileName), 'rb') as f:
				self.currentPathState = pickle.load(f)
		except FileNotFoundError:
			self.currentPathState = self.PathInstance.vehicle.getVehicleState()
		valueInputsBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(valueInputsBox)
		#input box for path parameters
		self.modeSelect = 0
		self.PathInputsDict = dict()
		for name, initValue in defaultPathParameters:
			newControl = doubleInputWithLabel.doubleInputWithLabel(name, initValue)
			valueInputsBox.addWidget(newControl)
			self.PathInputsDict[name] = newControl

		#set to default button
		PathControlsBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(PathControlsBox)
		self.setDefault = QtWidgets.QPushButton('Set to Default')
		PathControlsBox.addWidget(self.setDefault)
		self.setDefault.clicked.connect(self.default)

		#button to generate the path
		self.calcPathButton = QtWidgets.QPushButton("Generate Path")
		self.calcPathButton.clicked.connect(self.PathButtonResponse)
		PathControlsBox.addWidget(self.calcPathButton)

		#button to save path in file
		self.savePathButton = QtWidgets.QPushButton("Save Path")
		self.savePathButton.clicked.connect(self.savePathResponse)
		PathControlsBox.addWidget(self.savePathButton)

		#label when no path is generated
		self.PathStatus = QtWidgets.QLabel("No Path Calculated")
		PathControlsBox.addWidget(self.PathStatus)
		PathControlsBox.addStretch()

		#checkboxes layout
		modeColumn = QtWidgets.QVBoxLayout()
		modeSelectionBox = QtWidgets.QHBoxLayout()
		self.usedLayout.addLayout(modeSelectionBox)
		modeSelectionBox.addLayout(modeColumn)

		#checkboxes for selection of path following mode
		self.mode = list()
		self.steadyButtonsGroup = QtWidgets.QButtonGroup()
		for index, (name, value) in enumerate(VehiclePhysicalConstants.PathFollowingModes):
			newRadio = QtWidgets.QRadioButton(name)
			self.mode.append(value)
			modeColumn.addWidget(newRadio)
			self.steadyButtonsGroup.addButton(newRadio, index)

		self.steadyButtonsGroup.button(0).setChecked(True)

		#apply mode button
		applyModeButton = QtWidgets.QPushButton("Apply Mode")
		applyModeButton.clicked.connect(self.applyMode)
		self.usedLayout.addWidget(applyModeButton)
		self.usedLayout.addStretch()

		self.PathSignal.connect(self.PathCalculated)
		return

	def default(self):
		"""
		sets the inputs back to default
		"""
		for name, value in defaultPathParameters:
			self.PathInputsDict[name].setValue(value)
		return

	def PathButtonResponse(self):
		"""
		disables the gui and starts the thread which actually calculates trim to ensure the gui doesn't stall
		"""
		self.PathStatus.setText("Calculating Path")
		# self.guiControls.ResetSimulation()
		# self.resetSimulationActions()
		self.setDisabled(True)
		mode = self.modeSelect
		oX = self.PathInputsDict['Origin X'].getValue()
		oY = self.PathInputsDict['Origin Y'].getValue()
		oZ = self.PathInputsDict['Origin Z'].getValue()
		Course = radians(self.PathInputsDict['Course'].getValue())
		R = abs(self.PathInputsDict['R'].getValue())
		Dir = self.PathInputsDict['Direction'].getValue()
		threading.Thread(target=self.calculatePath, name='Calc Path', args=(mode,oX,oY,oZ,Course,R,Dir), daemon=True).start()
		#self.PrimCalcComplete()
		return

	def calculatePath(self, mode,oX,oY,oZ,Course,R,Dir):
		"""
		calculates path and emits a signal to the gui

		:param mode: passed to computePath
		:param oX: passed to computePath
		:param oY: passed to computePath
		:param oZ: passed to computePath
		:param Course: passed to computePath
		:param R: passed to computePath
		:param Dir: passed to computePath
		:return:
		"""
		#compute the path points then emit signal
		pathSucceeded = self.PathInstance.computePath(mode,oX,oY,oZ,Course,R,Dir)
		self.PathSignal.emit((pathSucceeded,mode,oX,oY,oZ,Course,R,Dir))
		return

	def PathCalculated(self, parameters):
		"""
		enables the gui and prints a status message.
		updates the inputs and uses the callback to generate path plot

		:param parameters: all the returns from the thread
		"""
		self.setDisabled(False)
		self.PathStatus.setText("Path calculations Complete")
		pathSucceeded,mode,oX,oY,oZ,Course,R,Dir = parameters
		if not pathSucceeded:
			self.PathStatus.setText("Path parameters given are not possible")
			return

		self.currentPathState = self.PathInstance.vehicle.getVehicleState()
		parametersDict = dict()
		parametersDict['Mode'] = mode
		parametersDict['Origin X'] = oX
		parametersDict['Origin Y'] = oY
		parametersDict['Origin Z'] = oZ
		parametersDict['Course'] = Course
		parametersDict['R'] = R
		parametersDict['Direction'] = Dir
		self.callBack(pathSucceeded)
		return

	def savePathResponse(self):
		"""
		calls the path with the path so we export the pathstate and inputs
		"""
		PathExportPath = os.path.join(sys.path[0], defaultPathFileName)
		with open(PathExportPath, 'wb') as f:
			pickle.dump(self.currentPathState, f)
		return


	def applyMode(self):
		"""
		Updates selected mode

		"""
		modeWanted = self.mode[self.steadyButtonsGroup.checkedId()]
		#print(modeWanted)
		self.modeSelect = modeWanted
		return