
import serial

class MotorControl:

<<<<<<< HEAD
	def __init__(self, prefix, ports=[], baud=115200):
=======
	def __init__(self, prefix='/dev/ttyACM', ports=[], baud=9600):
>>>>>>> 4ec647cc1562bfdf4d98500d2b1d98927ea1b65f
		'''
		Connect the motor control at serial port defined as:
		prefix: The port prefix, e.g. /dev/ttyUSB, /dev/ttyACM, etc.
		ports: A list of port numbers (as a suffix) to attempt to connect to.
		baud: Baud...
		'''
		if not ports:
			ports = range(3)

		success = False
		for port in ports:
			dev = prefix + str(port)
			try:
				self.ser = serial.Serial(dev, baud, timeout=2)
			except:
				print "Failed to open:", dev, "Attempting next port..."
			else:
				print "Succesfully opened:", dev
				break


	def setLeftMotor(self, v):
		''' setLeftMotor(velocity)
		Takes a velocity valued between -100 and 100 to send to the left motor.
		'''
		# The motors are "servo controlled". The values to the Arduino need to
		# be between 30 and 150:
		# 30 = full speed backwards
		# 90 = no speed
		# 150 = full speed forwards

		v = max( min(v,100), -100)
		vel = int( float(v)/60 + 90 )
		self.ser.write('L' + str(vel))
		self.ser.flush()
		#self.ser.close()

	def setRightMotor(self, v):
		''' setLeftMotor(velocity)
		Takes a velocity valued between -100 and 100 to send to the left motor.
		'''
		v = max( min(v,100), -100)
		vel = int (float(v)/60 + 90 )
		self.ser.write('R' + str(vel))
		self.ser.flush()
		#self.ser.close()

	def setMotor(self, vL, vR):
		self.setLeftMotor(vL)
		self.setRightMotor(vR)

