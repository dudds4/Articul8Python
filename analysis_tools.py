from quaternion import *

def getKneeAngle(shankQuat, thighQuat)
	angle = quatToRPY(relativeRotation(shankQuat, thighQuat))[0]
	
