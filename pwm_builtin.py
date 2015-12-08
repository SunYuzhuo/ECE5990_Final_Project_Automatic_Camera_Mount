#!/usr/bin/python

import smbus
import math
import time
import RPi.GPIO as GPIO 

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

LOW = 0.02 
motor_x = 0.0015
motor_y = 0.0015
motor_z = 0.0015

GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)    # x axis
GPIO.setup(17, GPIO.OUT)    # y axis
GPIO.setup(22, GPIO.OUT)    # z axis
px = GPIO.PWM(16, 50)
py = GPIO.PWM(17, 50)
pz = GPIO.PWM(22, 50)

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def get_z_rotation(x,y,z):
    radians = math.atan2(y,z)
    #radians = math.atan((math.sqrt(x*x+y*y)/z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

print "gyro data"
print "---------"



class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value
		#print "Error: ", self.error

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setpoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

def get_current_perf(axis):
	accel_xout = read_word_2c(0x3b)
	accel_yout = read_word_2c(0x3d)
	accel_zout = read_word_2c(0x3f)
	accel_xout_scaled = accel_xout / 16384.0
	accel_yout_scaled = accel_yout / 16384.0
	accel_zout_scaled = accel_zout / 16384.0

	if axis == "x":
		value = round(get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), 2)
		print "x rotation:  " , value
		return value
	elif axis == "y":
		value = round(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), 2)
		print "y rotation: " , value
		return value
	elif axis == "z":
		value = round(get_z_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), 2)
		print "z rotation: " , value
		return value
	else: 
		return False

def set_pwm_output(axis, angle):
	global LOW
	global motor_x
	global motor_y
	global motor_z
	
	if axis == "x":
		value = pidx.update(angle)
		value =deadzone(value)
		val = float(value/90)/1000
		motor_x = motor_x + val
		motor_x = saturation(motor_x)
		duty_x = 100 * motor_x / 0.02
		#print "motor_x: ", motor_x
		f = 1 / (motor_x + LOW)
		px.ChangeFrequency(f)
		px.start(duty_x)
	if axis == "y":
		value = pidy.update(angle)
		print "PID value: ", value
		value = deadzone(value)
		val = float(value)/90/1000
		motor_y = motor_y + val
		motor_y = saturation(motor_y)
		f = 1 / (motor_y + LOW)
		duty_y = round(100 * motor_y * f, 2)
		if duty_y < 6.5:
			duty_y = 6.5
                        motor_y = motor_y - val
		print "Duty_y: " , duty_y
		py.ChangeFrequency(f)
		py.start(duty_y)

	if axis == "z":
		value = pidz.update(angle)
		value =deadzone(value)
		val = float(value/90)/1000
		motor_z = motor_z - val
		motor_z = saturation(motor_z)
		duty_z = 100 * motor_z / 0.02
		#print "motor_z: ", motor_z
		f = 1 / (motor_z + LOW)
		pz.ChangeFrequency(f)
		pz.start(duty_z)

def saturation(value):
	if value > 0.0023: # the maximum angle is 160 degree
		value = 0.0023
	if value < 0.0007: # the minmum angle is 20 degree
		value = 0.0007
	return value

def deadzone(value):
	if value < 1 and value > -1:
		value = 0
	return value

start_time = time.time()
while time.time()-start_time < 2:
	duty_x = 100 * motor_x / 0.02
	px.start(duty_x)

	duty_y = 100 * motor_y / 0.02
	py.start(duty_y)

	duty_z = 100 * motor_z / 0.02
	#print duty_z
	pz.start(duty_z)

pidx = PID(0.2, 0.02, 0.01)
pidy = PID(0.21, 0.022, 0.01)
pidz = PID(0.2, 0, 0)

cmd = None
#while cmd != "y":
	#cmd = raw_input("Set the initial performance? y/n\n")

x_init = get_current_perf("x")
y_init = get_current_perf("y")
z_init = get_current_perf("z")

pidx.setpoint(x_init)
pidy.setpoint(y_init)
pidz.setpoint(z_init)

while True: 
	time.sleep(0.11)  # 10 Hz
	angle_x = get_current_perf("x")
	angle_y = get_current_perf("y")
	#angle_z = get_current_perf("z")
	set_pwm_output("x", angle_x)
	set_pwm_output("y", angle_y)
	#set_pwm_output("z", angle_z)

GPIO.cleanup()

