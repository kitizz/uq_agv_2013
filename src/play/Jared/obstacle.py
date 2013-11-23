import cv2

class Obstacle:
	''' All dimensions in metres'''

	def __init__(self, x=0, y=0, radius=1):
		self.x = x;
		self.y = y;
		self.radius = radius;

	def toJsonObject(self):
		# Export the useful object information
		return {"x": self.x, "y": self.y, 'radius': self.radius}

	def place(self,map): 
		cv2.circle(map,(self.x,self.y),self.radius-10,(255,0,0),thickness=-1)
		#map[500,500] = (0,0,255)
		#map[:,0:0.5*1000] = (255,0,0)      # (B, G, R)
		#map[:,0.5*1000:1000] = (0,255,0)
		return 1

	@staticmethod
	def fromJsonObject(d):
		# From a JSON dictionary object, create an Obstacle object
		if d.has_key('x') and d.has_key('y') and d.has_key('radius'):
			obs = Obstacle(d['x'], d['y'], d['radius'])
			return obs
		return None