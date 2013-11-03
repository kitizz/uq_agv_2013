# Generate some object patterns as save as JSON format
import json

class obstacle:
	''' All dimensions in metres'''

	def __init__(self, x=0, y=0, radius=1):
		self.x = x;
		self.y = y;
		self.radius = radius;

	def toJsonObject(self):
		return {"obstacle":{"x": self.x, "y": self.y}}

def main():
	obs = []
	for x in range(1, 10):
		obs.append(obstacle(x, y=0.1, radius=0.5))

	jsonOb=[]
	for ob in obs:
		jsonOb.append(ob.toJsonObject())

	print json.dumps(jsonOb, indent=4, separators=(',', ': '))

if __name__ == '__main__':
	main()