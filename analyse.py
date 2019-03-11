from analysis_tools import *
from articul8_comm import *
from articul8_logs import *

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


def plotQuaternia(imuDataSets):
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
    plt.show()

def plotKneeAngles(imuDataSets, title=''):
    if (len(imuDataSets) != 2):
        print('Knee angle requires exactly two datasets')
        return

    n = min([len(imuDataSet) for imuDataSet in imuDataSets])
    print('{} data points'.format(n))

    shankIMU = imuDataSets[0][:n]
    thighIMU = imuDataSets[1][:n]

    kneeAngles = [getKneeAngle(s.quat, t.quat) for s,t in zip(shankIMU, thighIMU)]

    t = range(len(kneeAngles))
    plt.plot(t, kneeAngles)
    plt.title(title+' Knee Angles')
    plt.show()


def plotJointAngles(imuDataSets, title=''):

    n = min([len(imuDataSet) for imuDataSet in imuDataSets])
    print('{} data points'.format(n))

    names = ['right shank', 'right thigh']
    for imuDataSet in imuDataSets:

        imuDataSet = imuDataSet[:n]
        initialQuat = imuDataSet[0].quat

        globalQuatDiff = [quatProduct(initialQuat, quatConj(IMUSample.quat)) for IMUSample in imuDataSet]
        localQuatDiff = [[globalQuat[0]] + (rotateVector(globalQuat[1:], quatConj(initialQuat))[1:]) for globalQuat in globalQuatDiff]

        roll =  [180/math.pi*quatTo3Angle(q)[0] for q in localQuatDiff]
        pitch = [180/math.pi*quatTo3Angle(q)[1] for q in localQuatDiff]
        yaw  =  [180/math.pi*quatTo3Angle(q)[2] for q in localQuatDiff]

        t = range(len(roll))
        plt.plot(t, roll)
        plt.plot(t, pitch)
        plt.plot(t, yaw)

        plt.legend(['roll (x) (ext rot)','pitch (y) (abduction)','yaw (z) (flexion)'])
        plt.title(title)
        plt.show()

def main():
    loadCSerial()

    if(len(sys.argv) < 2):
        print("Please supply a folder containing log files")
    else:
        for i in range(1, len(sys.argv)):
            imuDataSets = readIMUData(sys.argv[i])
            plotKneeAngles(imuDataSets, sys.argv[i])

if __name__ == '__main__':
    main()