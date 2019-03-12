
from lra_helper import *
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

    # TODO: Determine best distance metric for selection of target state
    def dist(self, otherBodyState):
        sumsq = 0
        for rpy, otherRpy in zip(self.rpyAngles, otherBodyState.rpyAngles):
            sumsq += sum([(otherAngle - angle)**2 for angle, otherAngle in zip(rpy, otherRpy)])
        return sumsq

    # Correct yaw only
    def errorToLraMsgs(self, otherBodyState):

        if (otherBodyState.numBands != self.numBands):
            print('Wrong number of bands!')
            return None

        lraMsgs = []

        for band in range(self.numBands):

            intensities = [0] * numLRAs[band]

            rpyDiff = [self.rpyAngles[band][i] - otherBodyState.rpyAngles[band][i] for i in range(3)]
            rollDiff = rpyDiff[0]
            pitchDiff = rpyDiff[1]
            yawDiff = rpyDiff[2]

            if (abs(yawDiff) > abs(rollDiff) and abs(yawDiff) > abs(pitchDiff)):
                mag = 6 * abs(yawDiff) * 180 / math.pi
                angle = math.pi/2
                if (yawDiff < 0):
                    angle = 3*math.pi/2

                lraInterpolation = interpolateAngle(angle, band)

                for lra in lraInterpolation:
                    intensities[lra['idx']] = int(min(127, mag * lra['portion']))

                lraMsg = LRACmdMsg(False, intensities).toBytes()
                lraMsgs.append(lraMsg)

            elif (abs(pitchDiff) > abs(rollDiff)):
                mag = 8 * abs(pitchDiff) * 180 / math.pi
                angle = 0
                if (pitchDiff < 0):
                    angle = math.pi

                lraInterpolation = interpolateAngle(angle, band)

                for lra in lraInterpolation:
                    intensities[lra['idx']] = int(min(127, mag * lra['portion']))

                lraMsg = LRACmdMsg(False, intensities).toBytes()
                lraMsgs.append(lraMsg)


            elif (abs(rollDiff) * 180 / math.pi > 5):
                sign = 1.0
                if (rollDiff < 0):
                    sign = -1.0

                lraMsg = LRACmdMsg(True, sign*0.8).toBytes()
                lraMsgs.append(lraMsg)

            else:
                lraMsg = LRACmdMsg(False, intensities).toBytes()
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
        self.lastIdx = 0

    def update(self, imuMsgs):
        if self.baseImuMsgs is None:
            self.baseImuMsgs = imuMsgs.copy()

        self.stateVector.append(BodyState.fromIMU(imuMsgs, self.baseImuMsgs))

    def getNearestState(self, bodyState):
        minDist = math.inf
        nearestState = None
        nearestIdx = None
        
        nStates = len(self.stateVector)
        nChecks = nStates/4

        statesToCheck = []
        checkStart = self.lastIdx

        if(checkStart + nChecks < nStates):
            statesToCheck = self.stateVector[checkStart:(checkStart+nChecks)]
        else:
            statesToCheck = self.stateVector[checkStart:]
            nLeft = nChecks - len(statesToCheck)
            statesToCheck += self.stateVector[:nLeft]

        for i, state in enumerate(statesToCheck):
            dist = bodyState.dist(state)
            if (dist < minDist):
                nearestState = state
                minDist = dist
                nearestIdx = i

        self.lastIdx = (checkStart + nearestIdx) % nStates
        # print(self.lastIdx)
        return nearestState

    def reset(self):
        self.lastIdx = 0

    def getLraMsgs(self, currIMU, baselineIMU):
        currBodyState = BodyState.fromIMU(currIMU, baselineIMU)
        nearestState = self.getNearestState(currBodyState)
        return currBodyState.errorToLraMsgs(nearestState)

