#!/usr/bin/env python

from MotorControl import *

m = MotorControl('/dev/ttyACM', range(4), 9600)
m.setLeftMotor(0)
m.setRightMotor(0)
m.ser.close()

