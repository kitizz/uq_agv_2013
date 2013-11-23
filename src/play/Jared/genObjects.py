# Generate some object patterns as save as JSON format
import json
import math
import random
from obstacle import *

def main(map):
	obs = []
	for x in range(1,35):
		obs.append(Obstacle(random.randint(0,map.getHeight()), y=random.randint(0,map.getWidth()), radius=20).toJsonObject())

	jsonOb={'map': {'obstacle': obs}}
	
	print jsonOb
	F = open('testDump.json', 'w')
	json.dump(jsonOb, F, indent=4, separators=(',', ': '))
	F.close()

if __name__ == '__main__':
	main()