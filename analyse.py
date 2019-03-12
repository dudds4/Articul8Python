from analysis_tools import *
from articul8_comm import *
from articul8_logs import *
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def readIMUData(path="."):
    if not(os.path.isdir(path)):
        print('{} is not a valid path'.format(path))
        return

    # open the latest two log files
    nFiles = len(os.listdir(path))
    files = glob.glob(os.path.relpath(path)+"/*.log")
    files.sort(key=os.path.getmtime)
    files = files[::-1]
    files = files[0:nFiles]
    print(files)

    imuMsgLists = []

    for i in range(nFiles):
        ser_openLog(files[i], i)
        imuMsgLists.append([])

    failed = False
    while(not failed):
        for i in range(0, nFiles):
            packet = ser_getLogPacket(i)
            failed = failed or packet == None

            if (packet is not None and packet[POS_DATA] == IMU_DATA_MSG):
                parsed_message = IMUDataMsg.fromBytes(packet)
                imuMsgLists[i].append(parsed_message)

    return imuMsgLists


def plotQuaternia(imuDataSets, title=''):
    line_types = ['-', '--', '-.', ':', 'o', 's']

    for i, imuDataSet in enumerate(imuDataSets):
        j = range(len(imuDataSet))
        w = [msg.quat[0] for msg in imuDataSet]
        x = [msg.quat[1] for msg in imuDataSet]
        y = [msg.quat[2] for msg in imuDataSet]
        z = [msg.quat[3] for msg in imuDataSet]

        plt.plot(j, w, 'r'+line_types[i])
        plt.plot(j, x, 'b'+line_types[i])
        plt.plot(j, y, 'g'+line_types[i])
        plt.plot(j, z, 'k'+line_types[i])

    plt.legend(['w','x','y','z'])
    plt.title(title)
    plt.show()

def plotKneeAngles(imuDataSets, title=''):
    if (len(imuDataSets) != 2):
        print('Knee angle requires exactly two datasets')
        return

    n = min([len(imuDataSet) for imuDataSet in imuDataSets])
    print('{} data points'.format(n))

    shankIMU = [Quat(s.quat) for s in imuDataSets[0][:n]]
    thighIMU = [Quat(s.quat) for s in imuDataSets[1][:n]]

    kneeAngles = [getKneeAngle(s, t) for s,t in zip(shankIMU, thighIMU)]

    t = range(len(kneeAngles))
    plt.plot(t, kneeAngles)
    plt.title(title+' Knee Angles')
    plt.show()


def plotJointAngles(imuDataSets, title=''):

    n = min([len(imuDataSet) for imuDataSet in imuDataSets])
    print('{} data points'.format(n))

    names = ['shank', 'thigh']
    line_types = ['-', '--']
    for i, imuDataSet in enumerate(imuDataSets):

        imuDataSet = imuDataSet[:n]
        initialQuat = Quat(imuDataSet[0].quat)

        globalQuats = [Quat(IMUSample.quat) for IMUSample in imuDataSet]
        globalQuatDiffs = [initialQuat * quat.conj() for quat in globalQuats]
        localQuatDiff = [ [globalQuatDiff[0]] + globalQuat.conj().rotate(globalQuatDiff[1:]) for globalQuat, globalQuatDiff in zip(globalQuats, globalQuatDiffs)]

        roll =  [180/math.pi*quatTo3Angle(q)[0] for q in localQuatDiff]
        pitch = [180/math.pi*quatTo3Angle(q)[1] for q in localQuatDiff]
        yaw  =  [180/math.pi*quatTo3Angle(q)[2] for q in localQuatDiff]

        t = range(len(roll))
        plt.plot(t, roll, 'r'+line_types[i])
        plt.plot(t, pitch, 'b'+line_types[i])
        plt.plot(t, yaw, 'g'+line_types[i])

    plt.legend(['roll (x) (ext rot) '+names[0],'pitch (y) (abduction) '+names[0],'yaw (z) (flexion) '+names[0],
                'roll (x) (ext rot) '+names[1],'pitch (y) (abduction) '+names[1],'yaw (z) (flexion) '+names[1]])
    plt.title(title)
    plt.show()

def plotGravity(imuDataSets, title=''):
    line_types = ['-', '--', '-.', ':', 'o', 's']

    for i, imuDataSet in enumerate(imuDataSets):
        j = range(len(imuDataSet))
        x = [quatToGravity(msg.quat)[0] for msg in imuDataSet]
        y = [quatToGravity(msg.quat)[1] for msg in imuDataSet]
        z = [quatToGravity(msg.quat)[2] for msg in imuDataSet]

        plt.plot(j, x, 'r'+line_types[i])
        plt.plot(j, y, 'g'+line_types[i])
        plt.plot(j, z, 'b'+line_types[i])

    plt.legend(['x','y','z'])
    plt.title(title)
    plt.show()

# def getGravity(qI):

#     data[0] = (qI[1] * qI[3] - qI[0] * qI[2]) / 16384;
#     data[1] = (qI[0] * qI[1] + qI[2] * qI[3]) / 16384;
#     data[2] = (qI[0] * qI[0] - qI[1] * qI[1] - qI[2] * qI[2] + qI[3] * qI[3]) / (2 * 16384);

anklePosition = [0, 0, 0]
kneePosition = [0, 0, 0]
hipPosition = [0, 0, 0]
# rot = Quat([1, 0, 0], math.pi)

def turnToX(v):
    x = v[0]
    y = v[1]
    m = math.sqrt(x*x+y*y)
    th = math.atan2(y,x)

    if(th > math.pi / 2):
        th = th - math.pi

    if(th < -math.pi / 2):
        th = th + math.pi

    q = Quat([0, 0, 1], -th)
    return q

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def testGravity(imuDataSets):

    N = min(len(imuDataSets[0]), len(imuDataSets[1]))
    shankImus = imuDataSets[0][0:N]
    thighImus = imuDataSets[1][0:N]

    # quats = [Quat(imu.quat) for imu in imus]
    # gravities = [quatToGravity(quat) for quat in quats]
    # rotated = [ q.rotate(v) for q,v in zip(quats, gravities) ]

    firstShank = Quat(shankImus[0].quat)
    firstThigh = Quat(thighImus[0].quat)

    rotQs = turnToX(firstShank.rotate([1, 0, 0]))
    rotQt = turnToX(rotQs.rotate(firstThigh.rotate([1, 0, 0])))
    rotQt = Quat([0, 0, 1], math.pi) * (rotQt)

    shankQuats = [Quat(x.quat) for x in shankImus]
    thighQuats = [Quat(x.quat) for x in thighImus]

    kneePositions = [  rotQs.rotate(sq.rotate([1, 0, 0])) for sq in shankQuats]
    kneeHips = [  rotQs.rotate(tq.rotate([1, 0, 0])) for tq in thighQuats]
    kneeHips = [ rotQt.rotate(kh) for kh in kneeHips ]

    kneePositions = np.array(kneePositions)
    kneeHips = np.array(kneeHips)
    hipPositions = np.add(kneePositions, kneeHips)

    s = range(N)

    fig = plt.figure()
    ax = fig.gca(projection="3d")
    ax.view_init(azim=30)
    ax.set_aspect('equal')
    
    ax.scatter(kneePositions[:,0], kneePositions[:,1], kneePositions[:,2] )
    ax.scatter(hipPositions[:,0], hipPositions[:,1], hipPositions[:,2] )
    
    ax.scatter(kneePositions[0,0], kneePositions[0,1], kneePositions[0,2], marker="x", s=128)
    ax.scatter(hipPositions[0,0], hipPositions[0,1], hipPositions[0,2], marker="x", s=128)

    set_axes_equal(ax)

    plt.show()

    # i = 0
    # while(i < N):
    #     shankQuat = Quat(shankImus[i].quat)
    #     thighQuat = Quat(thighImus[i].quat)
        
    #     kneePosition = shankQuat.rotate([1, 0, 0])
    #     kneeHip = thighQuat.rotate([1, 0, 0])

    #     rotQ = turnToX(kneePosition)
    #     kneePosition = rotQ.rotate(kneePosition)
    #     kneeHip = rotQ.rotate(kneeHip)

    #     kneeHip = [round(a, 3) for a in kneeHip]
    #     hipPosition = [round(a+b,3) for a,b in zip(kneePosition, kneeHip)]
    #     kneePosition = [round(a,3) for a in kneePosition]

    #     print("{}, {}".format(kneePosition, hipPosition))

    #     i += 5


def main():
    loadCSerial()

    if(len(sys.argv) < 2):
        print("Please supply a folder containing log files")
    else:
        for i in range(1, len(sys.argv)):   
            imuDataSets = readIMUData(sys.argv[i])
            # plotGravity(imuDataSets, sys.argv[i])
            # plotKneeAngles(imuDataSets, sys.argv[i])
            # plotQuaternia(imuDataSets)
            # plotJointAngles(imuDataSets, sys.argv[i])
            testGravity(imuDataSets)

if __name__ == '__main__':
    main()