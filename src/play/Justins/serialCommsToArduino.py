# import serial
import time

serial = serial.Serial('/dev/tty.usbserial', 115200)
time.sleep(2)
serial.write('L25')
