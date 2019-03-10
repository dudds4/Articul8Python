from analysis_tools import *
from articul8_comm import *
from articul8_logs import *

def plotKneeAngle(path="."):
    if not(os.path.isdir(path)):
        print('{} is not a valid path'.format(path))
        return

    # open the latest two log files
    nFiles = 2
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

    thighIMU = imuMsgLists[1]
    shankIMU = imuMsgLists[0]

    n = min(len(thighIMU), len(shankIMU))
    thighIMU = thighIMU[:n]
    shankIMU = shankIMU[:n]

    print(len(thighIMU))
    print(len(shankIMU))

    kneeAngles = [getKneeAngle(s.quat, t.quat) for s,t in zip(shankIMU, thighIMU)]

    # print(len(kneeAngles))
    # print(kneeAngles)
    t = range(len(kneeAngles))
    plt.plot(t, kneeAngles)

    # line_types = ['-', '--', '-.', ':', 'o', 's']
    # for i in range(nFiles):
    #     j = range(len(imuMsgLists[i]))
    #     w = [msg.quat[0] for msg in imuMsgLists[i]]
    #     x = [msg.quat[1] for msg in imuMsgLists[i]]
    #     y = [msg.quat[2] for msg in imuMsgLists[i]]
    #     z = [msg.quat[3] for msg in imuMsgLists[i]]
    #     plt.plot(j, w, 'r'+line_types[i])
    #     plt.plot(j, x, 'b'+line_types[i])
    #     plt.plot(j, y, 'g'+line_types[i])
    #     plt.plot(j, z, 'k'+line_types[i])

    # plt.legend(['w','x','y','z'])
    plt.show()

def main():
    loadCSerial()

    if(len(sys.argv) != 2):
        print("Please supply a folder containing log files")
    else:
        plotKneeAngle(sys.argv[1])

if __name__ == '__main__':
    main()