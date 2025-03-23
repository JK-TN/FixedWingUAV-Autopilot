# FixedWingUAV-Autopilot
This repository holds the files to control a fixed-wing UAV through successive loop closure. Several modules allow the testing of separate parts of the simulator and autopilot.<br /><br /><br />
<ins>Chapter2.py:</ins> Translations and rotations of aircraft. Used for testing rotation from inertial frame to body frame.<br /><br />
<ins>Chapter3.py:</ins> Forces and moments acting on aircraft. Used for testing aircraft dynamics.<br /><br />
<ins>Chapter4.py:</ins> Throttle and control surfaces. Used for testing aircraft aerodynamics (specifically gravity and lift and drag forces).<br /><br />
<ins>Chapter5.py:</ins> Trim, steady, and gust winds. Used for testing aircraft trim conditions and models of steady and gust winds.<br /><br />
<ins>Chapter6.py:</ins> Closed loop control gains. Used to test UAV autopilot and tune gains using known state values.<br /><br />
<ins>Chapter7.py:</ins> Used to test and verify sensor noise models for correct noise, drift, and bias.<br /><br />
<ins>Chapter8.py:</ins> Used to incorporate sensor estimator onto autopilot model.<br /><br />
<ins>FinalProject.py</ins> Adds path following into simulator to follow straight line and circular paths.<br /><br /><br />

## How to use:
1. Clone this repository onto your current device.
2. CD into the folder "FixedWingUAV-Autopilot".
3. Type the following command into your command prompt: "py <Specific module you want to run>.py".
4. Play around with the simulator.<br />

## Notes:
FinalProject.py does not accept command inputs from joystick control.
