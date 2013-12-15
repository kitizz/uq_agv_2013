//-----------------------------------------------------------
//
// Grass lane detection
//
//
// Author: Jared Page
//
// ----------------------------------------------------------

#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#define ARDUINO_WAIT_TIME 2000

#include <stdio.h>
#include <stdlib.h>

class LaneDetector
{
	public:
		//INitialize Serial communication with the given COM port
		LaneDetector();
		~LaneDetector();
		void findLines();
		void updateImage(int image);
		void showImage();

	private:
		int size;
		int image;
		int houghPImg; 
		int houghImg;

};

#endif