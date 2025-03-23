'''
Author: Timothy Nguyen (tilonguy@ucsc.edu)
Date: March 2025
This file is the the wrapper that runs the path following simulation
'''

import math
import sys

import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface 
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue
import ece163.Simulation.FinalProjectSimulate
import ece163.Display.DataExport
import ece163.Display.doubleInputWithLabel
import ece163.Constants.VehiclePhysicalConstants as VehiclePhysicalConstants
import ece163.Display.WindControl as WindControl
from ece163.Display.vehicleTrimWidget import vehicleTrimWidget
from ece163.Display.vehiclePathWidget import vehiclePathWidget
from ece163.Display.controlGainsWidget import controlGainsWidget
from ece163.Containers.Controls import intendedPath
from ece163.Display.ReferenceControlWidget import ReferenceControlWidget

from ece163.Containers.Controls import referenceCommands
from ece163.Utilities.Joystick import Joystick
import ece163.Constants.JoystickConstants as JSC


stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'alpha', 'beta']

positionRange = 200

defaultTrimParameters = [('Airspeed', VehiclePhysicalConstants.InitialSpeed), ('Climb Angle', 0), ('Turn Radius', math.inf)]

class PathFollowing(baseInterface.baseInterface):

	def __init__(self, parent=None):
		self.simulateInstance = ece163.Simulation.FinalProjectSimulate.PathFollowingSimulate()
		super().__init__(parent)
		self.setWindowTitle("ECE163 Final Project: PathFollowing")
		stateplotElements = [[x] for x in stateNamesofInterest]
		stateplotElements.append(['Va', 'Vg'])
		statetitleNames = list(stateNamesofInterest)
		statetitleNames.append('Va & Vg')
		legends = [False] * len(stateNamesofInterest) + [True]
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(5, 3, stateplotElements, titles=statetitleNames, useLegends=legends)
		self.outPutTabs.addTab(self.stateGrid, "States")

		ControlPlotNames = ['Course', 'Speed', 'Height', 'Pitch', 'Roll']
		controlplotElements = [['Reference', 'Actual'] for x in ControlPlotNames]
		trimPlotNames = ['Throttle', 'Aileron', 'Elevator', 'Rudder']
		controlplotElements.extend([['Trim', 'Actual'] for x in trimPlotNames])
		ControlPlotNames.extend(trimPlotNames)

		self.controlResponseGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(3, 4, controlplotElements, titles=ControlPlotNames, useLegends=True)
		self.afterUpdateDefList.append(self.updateControlResponsePlots)


		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.exportWidget = ece163.Display.DataExport.DataExport(self.simulateInstance, 'FinalProject')
		self.outPutTabs.addTab(self.exportWidget, "Export Data")


		#add path widget here
		self.trimCalcWidget = vehicleTrimWidget(self, self.trimCalcComplete)
		self.PathCalcWidget = vehiclePathWidget(self, self.PathCalcComplete)

		# self.inputTabs.

		self.gainCalcWidget = controlGainsWidget(self, self.gainCalcComplete, parent=self)
		self.gainCalcWidget.createLinearizedModels(self.trimCalcWidget.currentTrimState, self.trimCalcWidget.currentTrimControls)

		# self.gainCalcWidget.createLinearizedModels(self.trimCalcWidget.currentTrimState, self.trimCalcWidget.currentTrimControls)

		self.windControl = WindControl.WindControl(self.simulateInstance.underlyingModel.vehicle.getVehicleAerodynamicsModel())
		self.inputTabs.addTab(self.windControl, WindControl.widgetName)

		#Initialize the controller and a structure for its commands
		self.joystick = Joystick()

		#For convenience, if a controller is active, go directly to the wind tab
		if self.joystick.active:
			self.inputTabs.setCurrentIndex(1)

		self.simulateInstance.underlyingModel.vehicle.setControlGains(self.gainCalcWidget.curGains)
		self.simulateInstance.underlyingModel.vehicle.setTrimInputs(self.trimCalcWidget.currentTrimControls)
		# self.playButton.setDisabled(True)

		# we wil handle the tab ordering at the end

		stateTab = self.outPutTabs.widget(2)
		stateText = self.outPutTabs.tabText(2)

		self.outPutTabs.removeTab(2)

		exportTab = self.outPutTabs.widget(2)
		exportText = self.outPutTabs.tabText(2)

		#we need a big enough window/tab for path widget
		self.outPutTabs.addTab(self.trimCalcWidget, "Trim")
		self.outPutTabs.addTab(self.PathCalcWidget, "Path Input")
		self.outPutTabs.addTab(self.gainCalcWidget, "Gains")
		self.outPutTabs.addTab(stateTab, stateText)
		self.outPutTabs.addTab(self.controlResponseGrid, "Control Response")
		self.outPutTabs.addTab(exportTab, exportText)

		self.outPutTabs.setCurrentIndex(4)

		self.showMaximized()

		####Simulation Update code###
		# # Updates the simulation when tab is being changed
		self.outPutTabs.currentChanged.connect(self.newTabClicked)
		self.outPutTabs.setCurrentIndex(0)
		self.plotWidgets = [self.stateGrid, self.controlResponseGrid]
		# Default for all graphs to be turned off
		self.updatePlotsOn()
		self.updatePlotsOff()
		# Overwrite simulationTimedThread function with modified sliderChangeResponse
		self.simulationTimedThread.timeout.connect(self.UpdateSimulationPlots)

		return


	def updateStatePlots(self, newState):
		self.updatePlotsOff()
		stateList = list()
		for key in stateNamesofInterest:
			newVal = getattr(newState, key)
			if key in ['yaw', 'pitch', 'roll', 'p', 'q', 'r', 'alpha', 'beta']:
				newVal = math.degrees(newVal)
			stateList.append([newVal])
		stateList.append([newState.Va, math.hypot(newState.u, newState.v, newState.w)])

		self.stateGrid.addNewAllData(stateList, [self.simulateInstance.time]*(len(stateNamesofInterest) + 1))
		self.updatePlotsOn()
		return

	def getVehicleState(self):
		return self.simulateInstance.underlyingModel.vehicle.getVehicleState()

	def runUpdate(self):
		#If wanting to use the controller for the reference input, modify here
		if self.joystick.active:
			#joysticks are for the bourgeoisie
			print("Joystick Incompatible With This Simulation")
		try:
			#update user input data
			pathData = intendedPath(self.PathCalcWidget.modeSelect,self.PathCalcWidget.PathInputsDict['Origin X'].getValue(),
									self.PathCalcWidget.PathInputsDict['Origin Y'].getValue(),-self.PathCalcWidget.PathInputsDict['Origin Z'].getValue(),
									math.radians(self.PathCalcWidget.PathInputsDict['Course'].getValue()),abs(self.PathCalcWidget.PathInputsDict['R'].getValue()),
									self.PathCalcWidget.PathInputsDict['Direction'].getValue())
		except:
			#recycle old inputs if a field is left blank
			pathData = self.simulateInstance.underlyingModel.path_input

		#pass user input to simulation model
		self.simulateInstance.takeStep(pathData)
		return

	def resetSimulationActions(self):
		self.simulateInstance.reset()
		self.stateGrid.clearDataPointsAll()
		self.vehicleInstance.reset(self.simulateInstance.underlyingModel.vehicle.getVehicleState())
		self.updateNumericStateBox(self.simulateInstance.underlyingModel.vehicle.getVehicleState())
		self.vehicleInstance.removeAllAribtraryLines()
		self.controlResponseGrid.clearDataPointsAll()

	def trimCalcComplete(self, **kwargs):
		"""
		if we have valid trim conditions we calculate the linear model
		"""
		self.gainCalcWidget.createLinearizedModels(self.trimCalcWidget.currentTrimState, self.trimCalcWidget.currentTrimControls)
		self.simulateInstance.underlyingModel.vehicle.setTrimInputs(self.trimCalcWidget.currentTrimControls)
		return

	def PathCalcComplete(self,points,  **kwargs):
		#used to generate the ideal path and plot it on the simulator
		self.vehicleInstance.removeAllAribtraryLines()
		self.vehicleInstance.addAribtraryLine(points, (1, 0, 1, 1))

		return

	def gainCalcComplete(self):
		self.simulateInstance.underlyingModel.vehicle.setControlGains(self.gainCalcWidget.curGains)
		self.simulateInstance.underlyingModel.vehicle.setTrimInputs(self.trimCalcWidget.currentTrimControls)
		print(self.simulateInstance.underlyingModel.vehicle.getControlGains())

	def updateControlResponsePlots(self):
		self.updatePlotsOff()
		inputToGrid = list()

		#Update the commanded commands appropriately if a controller is active
		#Commanded = self.referenceControl.currentReference


		vehicleState = self.simulateInstance.getVehicleState()
		#inputToGrid.append([math.degrees(Commanded.commandedCourse), math.degrees(vehicleState.chi)])  # Course
		#inputToGrid.append([Commanded.commandedAirspeed, vehicleState.Va])  # Speed
		#inputToGrid.append([Commanded.commandedAltitude, -vehicleState.pd])  # Height
		#inputToGrid.append([math.degrees(x) for x in [Commanded.commandedPitch, vehicleState.pitch]])  # pitch
		#inputToGrid.append([math.degrees(x) for x in [Commanded.commandedRoll, vehicleState.roll]])  # pitch
		ActualControl = self.simulateInstance.underlyingModel.vehicle.getVehicleControlSurfaces()
		trimSettings = self.trimCalcWidget.currentTrimControls
		inputToGrid.append([trimSettings.Throttle, ActualControl.Throttle])  # Throttle
		inputToGrid.append([math.degrees(x) for x in [trimSettings.Aileron, ActualControl.Aileron]])  # Throttle
		inputToGrid.append([math.degrees(x) for x in [trimSettings.Elevator, ActualControl.Elevator]])  # Throttle
		inputToGrid.append([math.degrees(x) for x in [trimSettings.Rudder, ActualControl.Rudder]])  # Throttle
		# print(inputToGrid)
		self.controlResponseGrid.addNewAllData(inputToGrid, [self.simulateInstance.time]*len(inputToGrid))
		self.updatePlotsOn()

		return

	def UpdateSimulationPlots(self):
		currentWidget = self.outPutTabs.currentWidget()
		# Ensure that that the timer is only enabled for states, sensors, and control response widgets
		if (currentWidget in self.plotWidgets):
			# self.runUpdate()
			self.updatePlotsOn()
			self.updatePlotsOff()
		return

	def newTabClicked(self):
		self.updatePlotsOn()
		self.updatePlotsOff()
		return

	# toggles the state grid widget
	def togglestateGridPlot(self, toggleIn):
		self.stateGrid.setUpdatesEnabled(toggleIn)
		return

	# toggles the control response widget
	def togglecontrolResponsePlot(self, toggleIn):
		self.controlResponseGrid.setUpdatesEnabled(toggleIn)
		return

	# Turns on all simulation plots
	def updatePlotsOn(self):
		# print("Turning on plot update")
		self.togglestateGridPlot(True)
		self.togglecontrolResponsePlot(True)
		return

	# Turns off all simulation plots
	def updatePlotsOff(self):
		# print("Turning off plot update")
		self.togglestateGridPlot(False)
		self.togglecontrolResponsePlot(False)
		return

sys._excepthook = sys.excepthook

def my_exception_hook(exctype, value, tracevalue):
	# Print the error and traceback
	import traceback
	with open("LastCrash.txt", 'w') as f:
		traceback.print_exception(exctype, value, tracevalue, file=f)
		# traceback.print_tb(tracevalue, file=f)
	print(exctype, value, tracevalue)
	# Call the normal Exception hook after
	sys._excepthook(exctype, value, tracevalue)
	sys.exit(0)

# Set the exception hook to our wrapping function
sys.excepthook = my_exception_hook

qtApp = QtWidgets.QApplication(sys.argv)
ourWindow = PathFollowing()
ourWindow.show()
qtApp.exec()