import numpy as np
import math
EARTH_RADIUOS = 6371000

p = np.array([3,0])
J = np.array([[math.cos(1.57), -1*math.sin(1.57)],[math.sin(1.57), math.cos(1.57)]])
n = J.dot(p)

phi1 = math.radians(25.653319)

latitude2  = 25.653319  + (n[0] / EARTH_RADIUOS) * (180 / math.pi)
longitude2 = -100.291318 + (n[1] / EARTH_RADIUOS) * (180 / math.pi) / math.cos(phi1)# * math.pi/180)

print(latitude2,longitude2)

