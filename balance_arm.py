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

desired_roll_angle = 0.0
max_roll_angle

while True:
	current_roll_angle = quad.get_pitch_roll()[0] # 0 to get pitch, roll comes second
	current_error = desired_roll_angle - current_roll_angle
	# error as percentage of possible error

	# tailor that into a directional throttle gradient somehow
