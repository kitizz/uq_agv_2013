# Generate some object patterns as save as JSON format
import json
import numpy as np

from obstacle import *

def main():
	obs = []
	# Generate a list of Obstacles in JSON dictionary format
	r = 10
	for thetaDeg in range(0, 360, 10):
		theta = np.deg2rad(thetaDeg)
		x = r*np.cos(theta)
		y = r*np.sin(theta)
		obs.append(Obstacle(x, y, radius=0.75).toJsonObject())

	jsonOb = {'map': {'obstacle': obs}}

	# print jsonOb
	F = open('testDump.json', 'w')
	json.dump(jsonOb, F, indent=4, separators=(',', ': '))
	F.close()

if __name__ == '__main__':
	main()