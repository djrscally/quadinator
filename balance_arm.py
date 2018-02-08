"""\
First test of the code. Objectives:
	1) Have an arm with a single motor at either end come to a level balance from any starting position.
	2) Maybe something else? Not sure yet.

This will require...
	1) Get the current roll angle (I'll use roll instead of pitch, for no particular reason)
	2) Run a PD loop to close the error
"""

from quadcopter import quadcopter
import sys

quad = quadcopter()

pitch_setpoint = 0.0
roll_setpoint = 0.0
z_accel_setpoint = 1.0

max_z_error = 1.0 # we'll just normalise to 1G maybe.

while True:
	current_roll_error = quad.get_rotational_error(pitch_setpoint, roll_setpoint, z_accel_setpoint)[1] # get the roll error
	z_accel_error = quad.get_acceleration_error(0.0,0.0,z_accel_setpoint)[2]
	
	both_throttles = 
	