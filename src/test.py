# file: rfcomm-client.py
# auth: Albert Huang <albert@csail.mit.edu>
# desc: simple demonstration of a client application that uses RFCOMM sockets
#       intended for use with rfcomm-server
#
# $Id: rfcomm-client.py 424 2006-08-24 03:35:54Z albert $

from bluetooth import *
import sys

myadd = ""

print "performing inquiry..."

nearby_devices = discover_devices(lookup_names = True)

print "found %d devices" % len(nearby_devices)

for name, addr in nearby_devices:
	 myadd = name
	 print " %s - %s" % (addr, name)

import bluetooth

bd_addr = myadd #itade address

port = 1
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))
print 'Connected'
sock.settimeout(1.0)
sock.send("x")
print 'Sent data'

data = sock.recv(1)
print 'received [%s]'%data

sock.close()