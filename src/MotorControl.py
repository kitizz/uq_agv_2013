
import serial

class MotorControl:

	def __init__(self, port, baud=9600):
		self.ser = serial.Serial(port, baud, timeout=2)
		self.ser.open()

	def setLeftMotor(self, v):
		''' setLeftMotor(velocity)
		Takes a velocity valued between -100 and 100 to send to the left motor.
		'''
		# The motors are "servo controlled". The values to the Arduino need to
		# be between 30 and 150:
		# 30 = full speed backwards
		# 90 = no speed
		# 150 = full speed forwards
		vel = int( float(v)/60 + 90 )
		self.ser.write('L' + str(v))

	def setRightMotor(self, v):
		''' setLeftMotor(velocity)
		Takes a velocity valued between -100 and 100 to send to the left motor.
		'''
		vel = int (float(v)/60 + 90 )
		self.ser.write('R' + str(v))

	def setMotor(self, vL, vR):
		for i in range(10):
			self.ser.write('L' + str(vL))
			self.ser.write('R' + str(vR))

	
