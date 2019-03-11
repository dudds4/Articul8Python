
from msg_defs import *
from articul8_comm import *
from quaternion import *
import numpy as np
import math

VIBRATE_THRESHOLD_DIST = 0.087 # 5 deg in rad

class BodyState:

    # Uses one imuMsg and one rpy set from each band
    def __init__(self, imuMsgs, rpyAngles):

        if (len(rpyAngles) != len(imuMsgs)):
            print('Error: Wrong number of rpyAngles')
            return

        self.numBands = len(imuMsgs)
        self.imuMsgs = imuMsgs
        self.rpyAngles = rpyAngles

    # TODO: Determine most accurate distance metric
    def dist(self, otherBodyState):
        sumsq = 0
        for rpy, otherRpy in zip(self.rpyAngles, otherBodyState.rpyAngles):
            sumsq += sum([(otherAngle - angle)**2 for angle, otherAngle in zip(rpy, otherRpy)])
        return sumsq

    def errorToLraMsgs(self, otherBodyState):

        lraMsgs = []

        if (otherBodyState.numBands != self.numBands):
            print('Wrong number of bands!')
            for band in range(self.numBands):
                intensities = [0] * numLRAs[band]
                lraMsg = LRACmdMsg(intensities).toBytes()
                lraMsgs.append(lraMsg)
            return lraMsgs

        else:
            for band in range(self.numBands):

                thisGrav = quatToGravity(self.imuMsgs[band].quat)
                magThisGrav = np.linalg.norm(thisGrav)

                otherGrav = quatToGravity(otherBodyState.imuMsgs[band].quat)
                magOtherGrav = np.linalg.norm(otherGrav)

                if (magOtherGrav == 0 or magThisGrav == 0):
                    continue

                normDot = np.dot(thisGrav, otherGrav) / (magOtherGrav * magThisGrav)
                dist = math.acos(np.clip(normDot, -1, 1))

                intensities = [0] * numLRAs[band]

                if (dist >= VIBRATE_THRESHOLD_DIST):
                    z_dist = otherGrav[2] - thisGrav[2]
                    y_dist = -otherGrav[1] + thisGrav[1]

                    mag = math.sqrt(y_dist**2 + z_dist**2)
                    angle = math.atan2(y_dist, z_dist)
                    if (angle < 0):
                        angle += 2*math.pi

                    # Linearly interpolate magnitude between two adjacent LRAs
                    angleSegment = 2*math.pi/numLRAs[band]

                    idx1 = 0
                    while ((idx1+1) * angleSegment < angle):
                        idx1 += 1

                    idx2 = (idx1 + 1) % numLRAs[band]

                    portion2 = (angle - idx1 * angleSegment) / angleSegment
                    portion1 = 1 - portion2

                    intensities[idx1] = mag * portion1
                    intensities[idx2] = mag * portion2
                    intensities = [int(min(127, 600*elem)) for elem in intensities]

                lraMsg = LRACmdMsg(intensities).toBytes()
                lraMsgs.append(lraMsg)

            return lraMsgs

    @staticmethod
    def fromIMU(imuMsgs, baselineImuMsgs):

        # Get local RPY relative to baseline quaternion
        rpyAngles = []

        for blImuMsg, imuMsg in zip(baselineImuMsgs, imuMsgs):
            blQuat = Quat(blImuMsg.quat)
            quat = Quat(imuMsg.quat)
            globalQuatDiff = blQuat * quat.conj()
            localQuatDiff = [globalQuatDiff[0]] + quat.conj().rotate(globalQuatDiff[1:])

            rpyAngles.append(quatTo3Angle(localQuatDiff))

        return BodyState(imuMsgs.copy(), rpyAngles)

class Movement:
    def __init__(self):
        self.baseImuMsgs = None
        self.stateVector = []

    def update(self, imuMsgs):
        if self.baseImuMsgs is None:
            self.baseImuMsgs = imuMsgs.copy()

        self.stateVector.append(BodyState.fromIMU(imuMsgs, self.baseImuMsgs))

    def getNearestState(self, bodyState):
        minDist = math.inf
        nearestState = None
        nearestIdx = None

        for i, state in enumerate(self.stateVector):
            dist = bodyState.dist(state)
            if (dist < minDist):
                nearestState = state
                minDist = dist
                nearestIdx = i

        return nearestState

    def getLraMsgs(self, currIMU, baselineIMU):
        currBodyState = BodyState.fromIMU(currIMU, baselineIMU)
        nearestState = self.getNearestState(currBodyState)
        return currBodyState.errorToLraMsgs(nearestState)

