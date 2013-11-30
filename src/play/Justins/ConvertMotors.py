def convert(speed, dire):

	m1 = 50 + (speed + dire)
	m2 = 50 + (speed - dire)

	return m1, m2


print convert(0, 0)
print convert(50, 0)
print convert(-50, 0)
print convert (0, -50)
print convert (0, 50)