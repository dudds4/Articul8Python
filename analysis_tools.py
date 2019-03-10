from quaternion import *
import math

def getKneeAngle(shankQuat, thighQuat):
	angle = quatToRPY(relativeRotation(shankQuat, thighQuat))[1]
	angle *= (180 / math.pi)
	return (angle if angle < 180 else 360 - angle)
	
