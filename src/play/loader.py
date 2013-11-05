import json
from obstacle import *

def loadObstacles(filePath):
	F = open(filePath, 'r')
	jsonObj = json.load(F)
	
	# JSON hierarchy is {'map': 'obstacle': [list]}
	# Check that this is the case, return empty list if not
	if not jsonObj.has_key('map'):
		print 'No key, "map"'
		return []

	m = jsonObj['map']
	if not m.has_key('obstacle'):
		print 'Map has no key, "obstacle"'
		return []
	
	# Get the list of objects from the JSON file
	objList = m['obstacle']
	# Init the list of obstacles
	obsList = []

	for objDict in objList:
		# Create the obstacle from the info and add to list
		obsList.append( Obstacle.fromJsonObject(objDict) )

	return obsList

def main():
	# Test that we're reading in the json and creating objects OK
	filePath = 'testDump.json'
	obsList = loadObstacles(filePath)

	print obsList

if __name__ == '__main__':
	main()
