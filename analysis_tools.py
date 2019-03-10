from quaternion import *
import math

def getKneeAngle(shankQuat, thighQuat):
	angle = quatToRPY(relativeRotation(shankQuat, thighQuat))[1]
	# if(angle > math.pi):
	# 	angle -= math.pi 
	angle *= (180 / math.pi)
	return angle
	
