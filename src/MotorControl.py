
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
		ser.write('l' + str(vel))

	def setRightMotor(self, v):
		''' setLeftMotor(velocity)
		Takes a velocity valued between -100 and 100 to send to the left motor.
		'''
		vel = int (float(v)/60 + 90 )
		ser.write('r' + str(vel))

	