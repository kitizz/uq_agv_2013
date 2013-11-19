import math

class LatLongConvert():
	def __init__(self):
		self.R = 6378.1 	# Radius of the earth in km

	def dir2latlong(self, lat0, long0, dist, bearing):
		lat0rad = math.radians(lat0)
		long0rad = math.radians(long0)

		bearing = math.radians(bearing)

		newlatrad = math.asin(math.sin(lat0rad)*math.cos(dist/self.R) +
				     math.cos(lat0rad)*math.sin(dist/self.R)*math.cos(bearing))

		newlongrad = long0rad + math.atan2(math.sin(bearing)*math.sin(dist/self.R)*math.cos(lat0rad),
		             math.cos(dist/self.R)-math.sin(lat0rad)*math.sin(newlatrad))

		return math.degrees(newlatrad), math.degrees(newlongrad)

bla = LatLongConvert()
print bla.dir2latlong(52.20472, 0.14056, 15, 90)