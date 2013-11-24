
from Ros2Python import *
import thread
import time

# r2p = None

def rosLoop():
	rospy.init_node('Ros2Python', anonymous=True)

	try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down liveObstacleDetection module"

def doControl():
	''' Do the actual control stuff'''
	r2p.obstacleDetector.showDetected(r2p.mask)


def controlLoop(r2p, loop_time):
	while True:
		now = time.time()
		doControl(r2p)
		elapsed = (time.time() - now)
		# Should be good enough
		time.sleep( loop_time - elapsed )


def main():
	r2p = Ros2Python()

	# Start the ROS loop
	thread.start_new_thread(rosLoop, ())
	# Start control loop @ 10Hz
	thread.start_new_thread(controlLoop, (r2p, 0.1))

	

if __name__ == '__main__':
	main()