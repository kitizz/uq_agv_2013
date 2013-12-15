//-----------------------------------------------------------
//
// Differential motor control using serial communications
//
//
// Author: Jared Page
//
// ----------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include "MotorControl.h"

int fd1;
int fd2;

char *buff, *buffer, *bufptr;
int wr,rd,nbytes,tries;

MotorControl::MotorControl()
{
	this->connected = false;
    fd1 = open("dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd1 == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
    }
    else
    {
        fcntl(fd1,F_SETFL,0);
        printf("Port 1 has been successfully opened and %d is the file description \n",fd1);
    }
}

MotorControl::~MotorControl()
{
//Deconstructor
}


void MotorControl::setRightMotor(int Speed)
{
	//Send command to the right motor
	printf("Sending to right motor\n");
}

void MotorControl::setLeftMotor(int Speed)
{
	//Send command to the left motor
	printf("Sending to left motor\n");
}

int MotorControl::isConnected()
{
	return this->connected;
}

void MotorControl::testMotors()
{
    printf("Motor Test Control Starting\n");
    MotorControl mc;  	
    // To create an 'instance' of the class, simply treat it like you would
    //  a structure.  (An instance is simply when you create an actual object
    //  from the class, as opposed to having the definition of the class)
    mc.setLeftMotor(100); 
    mc.setRightMotor(100);
    // To call functions in the class, you put the name of the instance,
    //  a period, and then the function name.
    bool conn;
    conn<< mc.isConnected();
    printf("Serial Connected %s \n",conn ? "true" : "false");
    // See above note.
}