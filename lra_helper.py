
from articul8_comm import numLRAs
import math

# Linearly interpolate magnitude between two adjacent LRAs
def interpolateAngle(angle, bandId):

    angleSegment = 2*math.pi/numLRAs[bandId]

    idx1 = 0
    while ((idx1+1) * angleSegment < angle):
        idx1 += 1

    idx2 = (idx1 + 1) % numLRAs[bandId]

    portion2 = (angle - idx1 * angleSegment) / angleSegment
    portion1 = 1 - portion2

    return [{"idx": idx1, "portion": portion1}, {"idx": idx2, "portion": portion2}]