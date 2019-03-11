from quaternion import *
import math

def getAnkleFlexion(shankQuat):
	shankGrav = quatToGravity(shankQuat)
	gravX = shankGrav[0]
	gravY = shankGrav[1]
	return 90 - math.acos(-gravX / math.sqrt(gravX*gravX+gravY*gravY))*180/math.pi;

def getAnkleAbduction(shankQuat):
	shankGrav = quatToGravity(shankQuat)
	gravX = shankGrav[0]
	gravZ = shankGrav[2]
	return math.acos(-gravX / math.sqrt(gravX*gravX+gravZ*gravZ))*180/math.pi;

def getKneeAngle(shankQuat, thighQuat):
	angle = quatToRPY(relativeRotation(shankQuat, thighQuat))[1]
	angle *= (180 / math.pi)
	return (angle if angle < 180 else 360 - angle)
	
