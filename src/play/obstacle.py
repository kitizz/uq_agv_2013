
class Obstacle:
	''' All dimensions in metres'''

	def __init__(self, x=0, y=0, radius=1):
		self.x = x;
		self.y = y;
		self.radius = radius;

	def toJsonObject(self):
		# Export the useful object information
		return {"x": self.x, "y": self.y, 'radius': self.radius}

	@staticmethod
	def fromJsonObject(d):
		# From a JSON dictionary object, create an Obstacle object
		if d.has_key('x') and d.has_key('y') and d.has_key('radius'):
			obs = Obstacle(d['x'], d['y'], d['radius'])
			return obs
		return None