
import math

class Quat:
    def __init__(self, w, x=None, y=None, z=None):
        if(y is None and z is None and hasattr(w, '__len__') and len(w) == 3):
            a = w
            theta = x
            self.w = math.cos(theta/2)
            s = math.sin(theta/2)
            self.x = a[0]*s
            self.y = a[1]*s
            self.z = a[2]*s
        elif(hasattr(w, '__len__') and len(w) == 4):
            self.w = w[0]
            self.x = w[1]
            self.y = w[2]
            self.z = w[3]
        else:
            self.w = w
            self.x = x
            self.y = y
            self.z = z

    def __add__(self, other):
        return Quat(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Quat(self.w - other.w, self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        if(type(other) == type(1) or type(other) == type(1.2)):
            return Quat(self.w*other, self.x*other, self.y*other, self.z*other)

        prod0 = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        prod1 = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        prod2 = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        prod3 = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quat(prod0, prod1, prod2, prod3)

    def __truediv__(self, f):
        return Quat(self.w/f, self.x/f, self.y/f, self.z/f)

    def __getitem__(self, index):
        if(index == 0):
            return self.w
        if(index == 1):
            return self.x
        if(index == 2):
            return self.y
        if(index == 3):
            return self.z

        b = 0 if index.start is None else index.start
        e = 4 if index.stop is None else index.stop
        return [self[i] for i in range(b, e)]

    def __str__(self):
        return ("[{}, {}, {}, {}]".format(self.w, self.x, self.y, self.z))

    def conj(self):
        n = -1
        return Quat(self.w, n*self.x, n*self.y, n*self.z)

    def normSquared(self):
        q = self
        return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]

    def norm(self):
        return math.sqrt(self.normSquared())

    def normalize(self):
        self = self / self.norm()
        return self

    def inv(self):
        return self.conj() / self.norm()

    def rotate(self, vec):
        vQuat = Quat(0, vec[0], vec[1], vec[2])
        r = self * vQuat * self.inv()
        return [r[i] for i in range(1,4)]

    def round(self, decimalPlaces):
        self.w = round(self.w, decimalPlaces)
        self.x = round(self.x, decimalPlaces)
        self.y = round(self.y, decimalPlaces)
        self.z = round(self.z, decimalPlaces)
        return self

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

# not actually sure that this is correct..
# my logic was that if we have a zero quat,
def relativeRotation(q1, q2):
    return q1.inv() * q2

# check below link to see how to get one of r/p/y
# I just use this method but apply to all three simultaneously
# https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
def quatToRPY(q):
    w2 = q[0]*q[0]
    mags = [math.sqrt(w2 + a*a) for a in q[1:4]]
    return [2 * math.acos(q[0] / a) for a in mags]

# def quatByUnitAxes(q, idx):
#     if(idx == 0):
#         return [-1*q[1], q[0], q[3], -1*q[2]]
#     if(idx == 1):
#         return [-1*q[2], -1*q[3], q[0], q[1]]
#     return [-1*q[3], q[2], -1*q[1], q[0]]

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
    angles[0] = math.atan2(num,den)

    num = 2*(q[0]*q[2] - q[1]*q[3])
    angles[1] = math.asin(num)
    # print(num)

    num = 2*(q[0]*q[3] + q[1]*q[2])
    den = 1 - 2*(q[2]*q[2] + q[3]*q[3])
    angles[2] = math.atan2(num,den)
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
