//-----------------------------------------------------------
//
// Differential motor control using serial communications
//
//
// Author: Jared Page
//
// ----------------------------------------------------------

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#define ARDUINO_WAIT_TIME 2000

#include <stdio.h>
#include <stdlib.h>

class MotorControl
{
	public:
		//INitialize Serial communication with the given COM port
		MotorControl();
		~MotorControl();
		void setLeftMotor(int vel);
		void setRightMotor(int vel);
		int isConnected();
		void testMotors();

	private:
		bool connected;

};

#endif