from msg_defs import *
from articul8_comm import *
import sys
import os
import glob
import matplotlib.pyplot as plt


def printImuData(imuData, tag):
    ss = "Quat: {} {} {} {}, Accel: {}, Time: {}".format(                   \
        imuData.quat[0], imuData.quat[1], imuData.quat[2], imuData.quat[3], \
        imuData.xAccel, imuData.time)

    print(tag + ss)

def plotLog(path="."):
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

    line_types = ['-', '--', '-.', ':', 'o', 's']
    for i in range(nFiles):
        j = range(len(imuMsgLists[i]))
        w = [msg.quat[0] for msg in imuMsgLists[i]]
        x = [msg.quat[1] for msg in imuMsgLists[i]]
        y = [msg.quat[2] for msg in imuMsgLists[i]]
        z = [msg.quat[3] for msg in imuMsgLists[i]]
        plt.plot(j, w, 'r'+line_types[i])
        plt.plot(j, x, 'b'+line_types[i])
        plt.plot(j, y, 'g'+line_types[i])
        plt.plot(j, z, 'k'+line_types[i])

    plt.legend(['w','x','y','z'])
    plt.show()

def exampleReadLog(nFiles):

    # open the latest two log files
    files = glob.glob("*.log")

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
            if(not failed):
                imuData = IMUDataMsg.fromBytes(packet)
                print("device {}. ".format(i) + str(imuData))

            if (packet is not None and packet[POS_DATA] == IMU_DATA_MSG):
                parsed_message = IMUDataMsg.fromBytes(packet)
                imuMsgLists[i].append(parsed_message)