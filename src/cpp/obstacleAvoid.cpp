//-----------------------------------------------------------
//
// Obstacle detection and avoidance test
//
//
// Author: Jared Page
//
// ----------------------------------------------------------
//My files
#include "MotorControl.h"
#include "LaneDetector.h"
#include "GPS.h"
//OpenCV files
// #include "/opt/ros/fuerte/include/opencv2/core/core.hpp"
// #include <opencv2/core/core.hpp>
// #include "opencv2/core/core.hpp"
// #include "opencv2/flann/miniflann.hpp"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/photo/photo.hpp"
// #include "opencv2/video/video.hpp"
// #include "opencv2/features2d/features2d.hpp"
// #include "opencv2/objdetect/objdetect.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/ml/ml.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/contrib/contrib.hpp"
// #include "opencv2/core/core_c.h"
// #include "opencv2/highgui/highgui_c.h"
// #include "opencv2/imgproc/imgproc_c.h"
//Core files
#include <iostream>
#include <stdio.h>

int main()
{
    printf("Motor control starting\n");
	MotorControl mc; 
	printf("GPS starting up\n");
	GPS gps;
	printf("Lane detection starting\n");
	LaneDetector ld;

	while(1)
	{

	    // // To create an 'instance' of the class, simply treat it like you would
	    // //  a structure.  (An instance is simply when you create an actual object
	    // //  from the class, as opposed to having the definition of the class)
	    // mc.setLeftMotor(100); 
	    // // To call functions in the class, you put the name of the instance,
	    // //  a period, and then the function name.
	    // bool conn;
	    // conn<< mc.IsConnected();
	    // printf("Serial Connected %s \n",conn ? "true" : "false");
	    // // See above note.
	}
}