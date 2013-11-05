# Generate some object patterns as save as JSON format
import json

from obstacle import *

def main():
	obs = []
	for x in range(1, 10):
		obs.append(Obstacle(x, y=0.1, radius=0.5).toJsonObject())

	jsonOb={'map': {'obstacle': obs}}
	
	# print jsonOb
	F = open('testDump.json', 'w')
	json.dump(jsonOb, F, indent=4, separators=(',', ': '))
	F.close()

if __name__ == '__main__':
	main()