
import serial

class MotorControl:

	def __init__(self, prefix='/dev/ttyACM', ports=[], baud=9600):
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
		v = max( min(v,100), 0)
		# print "v", v
		try:
			if not self.ser.isOpen():
				self.ser.open()
			speed = 50 - int((v))
			speed = max( min(speed,60), 40)
			self.ser.write('R' + str(speed)+'\n')
			self.ser.flush()
		except:
			print "Failed to write to serial"
		else:
			print "Serial already open"
			#self.ser.close()

	def setRightMotor(self, v):
		''' setLeftMotor(velocity)
		Takes a velocity valued between -100 and 100 to send to the left motor.
		'''
		v = max( min(v,100), 0)
		# print "v", v
		try:
			if not self.ser.isOpen():
				self.ser.open()
			speed = 50 + int((v))
			speed = max( min(speed,60), 40)
			self.ser.write('L' + str(speed)+'\n')
			self.ser.flush()
		except:
			print "Failed to write to serial"
		else:
			print "Serial already open"
			#self.ser.close()

	def setMotor(self, vL, vR):
		self.setLeftMotor(vL)
		# time.sleep(0.01)
		self.setRightMotor(vR)


	def setMotorSpeeds(self, vL, vR):
		self.motorSpeedL = vL
		self.motorSpeedR = vR

	def getMotorSpeeds(self):
		return (self.motorSpeedL,self.motorSpeedR)

