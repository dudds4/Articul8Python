
import math

def quatConj(q):
    conj = [0,0,0,0]
    conj[0] = q[0]
    conj[1] = -q[1]
    conj[2] = -q[2]
    conj[3] = -q[3]
    return conj

def quatProduct(q1, q2):
    prod = [0,0,0,0]
    prod[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    prod[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    prod[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    prod[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return prod

def quatDist(q1, q2):
    dotProduct = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]

    # Sometimes dotProduct is very slightly over 1
    dotProduct = min(dotProduct, 1)

    return 2*math.acos(dotProduct)

def quatToGravity(quat):
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    
    gx = 2 * (x*z - w*y);
    gy = 2 * (w*x + y*z);
    gz = w*w - x*x - y*y + z*z;

    return [gx, gy, gz]


def quatNormSq(q):
    return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]

def quatNorm(q):
    return math.sqrt(quatNormSq(q))

def quatNormalize(q):
    norm = quatNorm(q)
    return [x/norm for x in q]

def quatInv(q):
    conj = quatConj(q)
    normsq = quatNormSq(q)
    return ([a / normsq for a in conj])

def rotateVector(v, q):
    qInv = quatConj(q)
    qv = [0] + v
    quatProduct(quatProduct(q, qv), qInv)

# not actually sure that this is correct..
# my logic was that if we have a zero quat,
def relativeRotation(q1, q2):
    n = quatNormSq(q1)
    invQ1 = [val / n for val in quatConj(q1)]
    return invQ1*q2

# check below link to see how to get one of r/p/y
# I just use this method but apply to all three simultaneously
# https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
def quatToRPY(q):
    w2 = q[0]*q[0]
    mags = [math.sqrt(w2 + a*a) for a in q[1:4]]
    return [2 * math.acos(q[0] / a) for a in mags]

def quatByUnitAxes(q, idx):
    if(idx == 0):
        return [-1*q[1], q[0], q[3], -1*q[2]]
    if(idx == 1):
        return [-1*q[2], -1*q[3], q[0], q[1]]
    return [-1*q[3], q[2], -1*q[1], q[0]]

def vectorDot(v1, v2):
    return sum( [a*b for a,b in zip(v1,v2)] )

def vectorNorm(v):
    return math.sqrt(sum( [a*a for a in v] ))

def angleBetween(v1, v2):
    cosangle = vectorDot(v1, v2) / (vectorNorm(v1) * vectorNorm(v2))
    return math.acos(cosangle)

def quatCompInAxis(q, axisIdx):
    axis = [0, 0, 0]
    axis[axisIdx] = 1

    normIdx = (axisIdx + 1)%3
    br = quatByUnitAxes(q, normIdx)
    c = br
    c[axisIdx] = 0
    return angleBetween(br, c)

def quatTo3Angle(q):
    angles = [0, 0, 0]

    num = 2*(q[0]*q[1] + q[2]*q[3])
    den = 1 - 2*(q[1]*q[1] + q[2]*q[1])
    # print(num)
    # print(den)
    angles[0] = math.atan(num/den)

    num = 2*(q[0]*q[2] - q[1]*q[3])
    angles[1] = math.asin(num)
    # print(num)

    num = 2*(q[0]*q[3] + q[1]*q[2])
    den = 1 - 2*(q[2]*q[2] + q[3]*q[3])
    angles[2] = math.atan(num/den)
    # print(num)
    # print(den)

    # for idx in range(0,3):
    #     axisIdx = axes[idx]
    #     angle = quatCompInAxis(q, axisIdx)
    #     angles[axisIdx] = angle
    #     axis = [0, 0, 0]
    #     sinAngle = math.sin(angle/2)
    #     axis[axisIdx] = sinAngle
    #     qComp = [math.cos(angle/2)] + axis
    #     qCompInv = quatInv(qComp)
    #     q = quatProduct(q, qCompInv)

    # print(q)
    return angles