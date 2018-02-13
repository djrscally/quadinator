"""\
Main file for quadcopter control project. 
# YO YO YOU YO
Some other details down here, I suppose.

"""

import sys
from Adafruit_ADXL345 import ADXL345 # Accelerometer
from sensors.hmc5883l.hmc5883l import hmc5883l # Magnetic Compass (orientation)
from sensors.L3G4200D.L3G4200D import L3G4200D # Gyroscope (rotational acceleration)from Adafruit_ADXL345 import ADXL345 # Accelerometer
from Adafruit_BMP import BMP085 # Temp and Pressure Sensor (Coarse reading altimeter and...fuck knows what use temperature might be.)
from Adafruit_PCA9685 import PCA9685 # PWM Pi Hat, Controlling Motors and any servos I might include.
import math
import time

class quadcopter:

	def __init__(self):

		# Initialise the sensors
		self.gyro = L3G4200D()
		self.accel = ADXL345()
		self.barometer = BMP085.BMP085()
		self.compass = hmc5883l(gauss=4.7, declination = (-2,5))
		self.motors = PCA9685()

		# Set the PWM frequency to 50Hz for the motors
		self.motors.set_pwm_freq(50)

		# Map so I can refer to the motors alphabetically but still get the right PWM channel
		self.motor_map = {'A':0, 'B':1, 'C':2, 'D':3}

		# Min/Max pulse width for the PWM, read from the data sheet of the ESC
		self.min_pulse = 205
		self.max_pulse = 410
		self.pulse_range = self.max_pulse - self.min_pulse

		return None

	def _calibrate_min_max_single_throttle(self, channel):
		"""\
		Runs through the initialisation of the ESCs. Possibly configurable depending on the ESC
		but for now I'll just hardcode in the HobbyKing ones (*spits*).
		"""
		# also need to configure the frequency using PCA9685.set_pwm_freq()...again, read the datasheet. 
		# This is the first bit that can be threaded realistically.
		self.motors.set_pwm(channel, 0, 410)
		print("Plug in the battery now")
		time.sleep(5)
		self.motors.set_pwm(channel, 0, 205)
			
		return None # because I'm a placeholder.
		
	def _calibrate_min_max_all_throttles(self, channels):
		"""\
		Initialises all of the ESCs at once.
		"""
		return None
		

	def report_status(self):
		""" \
		No Actual function, really. Just going to exist \
		so that I can check all the sensors are working \
		whilst I'm building this thing.                 \
		"""

		print(\
		"Current Temperature: " + str(self.barometer.read_temperature())+\
		"\nCurrent Pressure: " + str(self.barometer.read_pressure())+\
		"\nAltitude: " + str(self.barometer.read_altitude())+\
		"\nOrientation: " + str(self.get_orientation())+\
		"\nPitch and Roll: : " + str(self.get_pitch_roll())+\
		"\nRotational Error: " + str(self.get_rotational_error(0.0, -45.0, 0.0))+\
		"\nAcceleration: " + str(self.get_acceleration())+\
		"\nAcceleration Error: " + str(self.get_acceleration_error(0.0, 0.0, 0.0))\
		)

	def get_rotational_error(self, x_targ, y_targ, z_targ):
		"""\
		Feeder function for PID controller.                         \
		"Manual" Inputs: desired x,y,z orientation.                 \
		Sensor Inputs: observed x,y,z orientation from HMC5883L     \
		Outputs: x,y,z error.                                       \

		Basically will just calculate how far the quad is from the  \
		desired orientation.
		"""
		obs_x, obs_y = self.get_pitch_roll()
		return (x_targ - obs_x, y_targ - obs_y)#, z_targ - obs_z)

	def get_acceleration_error(self, x_targ, y_targ, z_targ):
		"""\
		Calculates the error value for acceleration across all
		three axes.
		"""
		obs_x, obs_y, obs_z = self.get_acceleration()
		return (x_targ - obs_x, y_targ - obs_y, z_targ - obs_z)

	def get_acceleration(self):
		"""\
		Fetches the current x,y,z acceleration from the ADXL345. Converted to Gs
		"""

		x, y, z = self.accel.read()

		# constant from Range/2**9 where Range is the 2G sensitivity range of the sensor.
		c = 0.00390625

		x = x*c
		y = y*c
		z = z*c

		return (x, y, z)

	def get_pitch_roll(self):
		"""\
		Converts the acceleration readings (raw in LSB) from the Accelerometer
		into a G value, and calculates pitch and roll from these.

		The constant 0.00390625 is given from Range/2**9, where Range is the 2G 
		sensitivity range of the ADXL345, and 9 is the 10bit (left shifted) resolution.
		These values and the constant should be recalculated if sensitivity is amended.

		Ideally, I should probably put those in the class rather than here.

		Positive pitch is up. Positive roll is to the right (with the headers
		on the left).

		"""

		x, y, z = self.accel.read()

		c = 0.00390625

		x = x*c
		y = y*c
		z = z*c

		# Calculations for pitch and roll. Multiplied by 57.2958 to get
		# the output in degrees instead of radians.

		pitch = math.atan2(y,math.sqrt((x**2) + (z**2)))*57.2958
		roll = math.atan2(-x,z)*57.2958

		return (pitch, roll)

	def set_throttle(self, motor, throttle_setting):
		"""\
		Now then. I need a configurable min/max pulse width so I can pass a 0-1 values
		in the throttle variable I think. Really need to read the datasheet for the
		ESC to figure out what that should be though!

		"motor" variable will be A, B, C or D. In this configuration (top is direction
		of flight):

				A        B
				 \      /
				  \    /
				   \  /
				    \/
				    /\
				   /  \
			  	  /    \
				 /	    \
				C	     D
		"""
		pulse_width = self.min_pulse + (throttle_setting * self.pulse_range)
		self.motors.set_pwm(self.motor_map[motor], 0, int(pulse_width))

		return None 

	def run_motor(self):
		"""\
		For now, just run the fucking motor
		"""
		while True:
			rot_error = self.get_rotational_error(0.0,0.0,0.0)[1] / 180.0
			self.set_throttle('A', rot_error)
			s2lf.set_throttle('B', rot_error)
			sys.stdout.write('\r' + str(self.get_pitch_roll()[1]))
			sys.stdout.flush()
			
		return None


quad = quadcopter()
quad._calibrate_min_max_single_throttle(1)
while True:
	sys.stdout.write('\r' + str(quad.get_acceleration()))
	sys.stdout.flush()
	time.sleep(0.5)

"""

quad = quadcopter()
#quad.report_status()
quad.set_throttle('A', 0.0)
print("Plug er in")
time.sleep(5)
quad.run_motor()
"""
