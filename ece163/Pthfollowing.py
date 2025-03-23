'''
Authors: Neili Hu (nehu@ucsc.edu), Timothy Nguyen (tilonguy@ucsc.edu), Gowrav (gbukkapa@ucsc.edu)
Date: March 2025
This file holds classes relevant to path following and generation for straight line and circular paths.

Function Credits:
__init__() -Neili Hu (nehu@ucsc.edu)
circular() -Neili Hu (nehu@ucsc.edu)
straight() - Gowrav (gbukkapa@ucsc.edu)
computePath() -Timothy Nguyen (tilonguy@ucsc.edu)
generateIdealLine() -Timothy Nguyen (tilonguy@ucsc.edu)
generateIdealCircle() -Timothy Nguyen (tilonguy@ucsc.edu)
getcommands() -Neili Hu (nehu@ucsc.edu)
getstate() -Neili Hu (nehu@ucsc.edu)
setcommands() -Neili Hu (nehu@ucsc.edu)
setstate() -Neili Hu (nehu@ucsc.edu)
reset() -Neili Hu (nehu@ucsc.edu)
update() -Neili Hu (nehu@ucsc.edu),Timothy Nguyen (tilonguy@ucsc.edu)
'''


import math
import sys
import ece163.Utilities.MatrixMath as MM
import ece163.Utilities.Rotations as Rotations
import ece163.Containers as Containers
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Controls.VehicleClosedLoopControl as VehicleClosedLoopControl


'''
algorihtm processes
(1)pathfollow class generate commands first.
(2)VehicleClosedLoopControl.update() run once.
(3)pathfollow get new generated state from VehicleClosedLoopControl.update()
(3.1)#maybe get extra error state form somewhere else
(4)use new state to generate commands
(5)back to step (2)

'''
class Pathfollow:
    def __init__(self, dt = VPC.dT):
        self.dt = dt
        self.path_input = Containers.Controls.intendedPath()
        self.vehicle = VehicleClosedLoopControl.VehicleClosedLoopControl(useSensors=False, useEstimator=False)
        gain = Containers.Controls.controlGains(kp_roll = 76.40372075088878,
                                                kd_roll = 1.2023741345878758,
                                                ki_roll = 0.001,
                                                kp_sideslip = 0.021431120776235192,
                                                ki_sideslip = 1.0099685422355764,
                                                kp_course = 4.841997961264016,
                                                ki_course = 2.2999490316004074,
                                                kp_pitch = -274.14559648856346,
                                                kd_pitch = -4.837820587299192,
                                                kp_altitude = 0.07272688647866368,
                                                ki_altitude = 0.040403825821479826,
                                                kp_SpeedfromThrottle = 0.13300933812934967,
                                                ki_SpeedfromThrottle = 0.08858391531047775,
                                                kp_SpeedfromElevator = -0.15460400930452495,
                                                ki_SpeedfromElevator = -0.10296591697624827)
        self.vehicle.setControlGains(controlGains = gain)
        self.commands = Containers.Controls.referenceCommands() # store last command, may be removed later

    def circular(self, state = Containers.States.vehicleState(), path = Containers.Controls.intendedPath()):
        #path.ox,oy,oz is n,e,d position
        res = Containers.Controls.referenceCommands()
        temp = math.atan2(state.pe-path.oY, state.pn-path.oX)

        if temp - state.chi < -math.pi: #modify to within [-pi,pi]
            temp = temp + 2*math.pi
        elif temp - state.chi > math.pi:
            temp = temp - 2*math.pi

        kd = 1.1
        d = math.hypot(state.pn-path.oX, state.pe-path.oY)
        res.commandedCourse = temp + path.Direction*(math.pi/2+math.atan(kd*(d-path.R)/path.R))
        res.commandedAltitude = -path.oZ
        res.commandedAirspeed = VPC.InitialSpeed

        #pass forward for continuity
        res.commandedPitch = self.commands.commandedPitch
        res.commandedRoll = self.commands.commandedRoll

        return res
    
    def straight(self, state = Containers.States.vehicleState(), path = Containers.Controls.intendedPath()):
        #Container for closed loop commands
        res = Containers.Controls.referenceCommands()

        #store what doesn't need to be changed
        res.commandedAltitude = -path.oZ
        res.commandedAirspeed = VPC.InitialSpeed

        #pass forward for continuity
        res.commandedPitch = self.commands.commandedPitch
        res.commandedRoll = self.commands.commandedRoll

        # store course
        qCourse = path.Course

        # Rotation matrix based on course
        rotM = [[math.cos(qCourse), math.sin(qCourse), 0], [-math.sin(qCourse), math.cos(qCourse), 0], [0, 0, 1]]

        # rotate error from path into path frame
        ep = MM.multiply(rotM, MM.subtract([[state.pn], [state.pe], [state.pd]], [[path.oX], [path.oY], [path.oZ]]))

        # Calculate cross-track error
        crosstrack_error = ep[1][0]

        # Base gain of 0.05 is kept for compatibility, just adding error-based adjustment
        k_py = 0.05
        if abs(crosstrack_error) > 10.0:  # For large errors, limit the gain's effect
            crosstrack_error = 10.0 * (crosstrack_error / abs(crosstrack_error))
            
        # Adding a small component to account for current course error
        chi_error = qCourse - state.chi
        # Normalize the angle to [-pi, pi]
        while chi_error > math.pi:
            chi_error -= 2.0 * math.pi
        while chi_error < -math.pi:
            chi_error += 2.0 * math.pi

        # Small gain for heading correction
        k_chi = 0.01  

        # generate required course from error
        res.commandedCourse = (qCourse - math.atan(k_py * crosstrack_error) + k_chi * chi_error)

        return res

    def computePath(self,mode=0,oX=0,oY=0,oZ=-VPC.InitialSpeed,Course=0,R=0,Dir=1):
        #use this to generate path lines dependent on whether circular or line following is selected
        if(mode == 0):
            points =self.generateIdealLine(oX,oY,oZ,Course)
        elif(mode==1):
            points = self.generateIdealCircle(oX,oY,oZ,R)

        return points

    def generateIdealLine(self,oX,oY,oZ,Course):
        #store origin
        points = [[oY,oX,oZ]]

        #no end point input for ease of programming, therefore we put an endpoint really far away with a given course
        uV = [[100000],[0],[0]]

        #rotate endpoint by course
        rotM = [[math.cos(Course-(math.pi/2)),math.sin(Course-(math.pi/2)),0],[-math.sin(Course-(math.pi/2)),math.cos(Course-(math.pi/2)),0],[0,0,1]]
        nV = MM.multiply(rotM,uV)

        #store endpoint
        points.append([nV[0][0]+oY,nV[1][0]+oX,nV[2][0]+oZ])

        #return
        return points

    def generateIdealCircle(self,oX,oY,oZ,R):
        #start with a point R away from origin
        points = [[oY,oX+R,oZ]]

        #starting angle
        psi = math.pi/2

        #zero catching
        if R <50:
            R=50

        #iterate based on circumfrence of circular path
        for i in range(int(R*math.pi*2/VPC.dT)):
            #increment over time step
            psi += VPC.dT
            #store points
            points.append([R * math.cos(psi)+oY, R * math.sin(psi)+oX, oZ])

        return points

    def getcommands(self):
        #return commands
        return self.commands
    
    def getstate(self):
        #return state
        return self.vehicle.getVehicleState()
    
    def setcommands(self, commands = Containers.Controls.referenceCommands()):
        #sets commands based on command input
        self.commands = commands

    def setstate(self, state = Containers.States.vehicleState()):
        #sets vehicle state based on given state
        self.vehicle.setVehicleState(state)

    def reset(self):
        self.vehicle.reset()
        self.commands = Containers.Controls.referenceCommands()
        self.path_input = Containers.Controls.intendedPath()
        #reset path generator


    def update(self, path = Containers.Controls.intendedPath()):
        #take in user input
        inp = path

        #Saturating direction command
        if inp.Direction >=0:
            inp.Direction = 1
        else:
            inp.Direction = -1

        #setting minimum circle size
        if inp.R <50:
            inp.R = 50

        #store path variables for accessibility
        self.path_input = inp

        if inp.mode == 1:#if circle path mode run circle path follow program
            self.commands = self.circular(self.vehicle.getVehicleState(), inp)
        elif inp.mode == 0:#if line path mode run line path follow program
            self.commands = self.straight(self.vehicle.getVehicleState(), inp)

        #update closed loop control with resulting commands
        self.vehicle.update(referenceCommands=self.commands)
