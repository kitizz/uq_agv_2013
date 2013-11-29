import NMEA
import serial.serialwin32 as serial
from SimpleXMLRPCServer import SimpleXMLRPCServer
from threading import Thread

SERIALPORT = 9
BAUDRATE = 4800
SERVERPORT = 54321

class NMEAServer(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.daemon = True
		self.NMEA = NMEA.NMEA()
		self.ser = serial.Serial(SERIALPORT, BAUDRATE)

def run(self):
	while(self.ser.isOpen()):
		try:
			line = self.ser.readline()
		except:
			pass
			if(line):
				self.NMEA.handle_line(line)

def join(self):
	self.ser.close()
	return True


def GetLatLong(self):
	return (self.NMEA.lat, self.NMEA.lon)
def GetInView(self):
	return self.NMEA.in_view


def RunServer():
	MyXMLServer = SimpleXMLRPCServer(('localhost', SERVERPORT))
	MyNMEAServer = NMEAServer()
	MyNMEAServer.start()
	MyXMLServer.register_instance(MyNMEAServer)
	MyXMLServer.serve_forever()