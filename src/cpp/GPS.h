//-----------------------------------------------------------
//
// Grass lane detection
//
//
// Author: Jared Page
//
// ----------------------------------------------------------

#ifndef GPS_H
#define GPS_H

#define ARDUINO_WAIT_TIME 2000

#include <stdio.h>
#include <stdlib.h>

class GPS
{
	public:
		//INitialize Serial communication with the given COM port
		GPS();
		~GPS();
		void openPort();
		void readFromPort();
		void handleData();
		int getSpeed();
		int getHeadingT();
		int getHeadingM();
		int getLatitude();
		int getLongitude();
		int getLatitudeD();
		int getLongitudeD();
		int getTime();


	private:
		
        //self.portlist = []
        //self.list_serial_ports();
        
        bool serial_open;
        //self.queue = Queue()
        
        //VTG STRINGS
        int speedkhm;
        int speedknots;
        int headingM;
        int headingT;
        
        //GGA STRINGS
        int time;
        int latitude;
        int longitude;
        int latitude_D;
        int longitude_D;
        int quality;
        int satelliteno;
        int HDOP;
        int altitude;
        int HOGAWGS84E;
        int time_since_DGPS_update;
        int DGPS_reference_ID;
        
       // int lock = threading.Lock()

};

#endif