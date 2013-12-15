#CC = g++
#CFLAGS = -c -Wall
CXX = g++.lib -lpthread -ldl -lm -std=gnu++0x
CXXFLAGS = =I/user/local/include/opencv2 -g3 -Wall -c
LIBS = -L/user
CFLAGS = $SHELL(pkg-config --cflags opencv)
LIBS = $SHELL(pkg-config --libs opencv)



all: build

#INCLUDEPATH += "/opt/ros/fuerte/include"

build: obstacleAvoid.o MotorControl.o LaneDetector.o GPS.o
	g++ GPS.cpp LaneDetector.cpp MotorControl.cpp obstacleAvoid.cpp -o main 

# build: obstacleAvoid.o MotorControl.o LaneDetector.o GPS.o
	# g++ GPS.o LaneDetector.o MotorControl.o obstacleAvoid.o -o main 

# obstacleAvoid.o: obstacleAvoid.cpp
# 	$(CFLAGS) $(LIBS) obstacleAvoid.cpp -o obstacleAvoid.o

# MotorControl.o: MotorControl.cpp
# 	$(CFLAGS) $(LIBS) MotorControl.cpp -o MotorControl.o

# LaneDetector.o: LaneDetector.cpp
# 	$(CFLAGS) $(LIBS) LaneDetector.cpp -o LaneDetector.o

# GPS.o: GPS.cpp
# 	$(CFLAGS) $(LIBS) GPS.cpp -o GPS.o