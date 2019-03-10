from articul8_comm import *
from msg_defs import *
from workers import *
from threadman import *

import sys
import time
import string
import signal
import platform
import os
import glob
import matplotlib.pyplot as plt

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        print('Cleaning up')
        setThreadFlag(False)
        while(checkThreadCount() > 0):
            print('{} threads remaining'.format(checkThreadCount()))
            time.sleep(1)

        ser_cleanup();
        sys.exit(0)

def exampleWriteLog():
    ser_startLogging()

    for i in range(0, 100):
        m = StreamMsg(20)
        ser_write(m.toBytes())
        time.sleep(0.02)

    ser_stopLogging()

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

def main():

    loadCSerial()
    signal.signal(signal.SIGINT, signal_handler)

    if (len(sys.argv) > 1):
        time.sleep(0.5)
        try:
            nFiles = int(sys.argv[1])
            exampleReadLog(nFiles)
            return
        except:
            pass

        try:
            plotLog(sys.argv[1])
            return
        except:
            pass

    tries = 0
    for i, port in enumerate(ports):
        print("Opening port {}".format(port))
        while(not ser_isOpen(i) and tries < 5):
            ser_open(port, i)
            time.sleep(0.2)
            tries += 1

    failed = False
    for i, port in enumerate(ports):
        if(not ser_isOpen(i)):
            failed = True

    if(failed):
        print("Couldn't open all ports...")
        ser_cleanup()
        sys.exit()
        
    print("Starting bt thread")
    btThread = startThread(bluetoothWorker)

    print('Starting keep alive thread')
    keepAliveThread = startThread(keepAliveWorker)

    print('Starting TCP server thread')
    tcpServerThread = startThread(tcpServerWorker)

    print('Starting LRA control thread')
    lraControlThread = startThread(lraControlWorker)

    portIdxs = [i for i in range(len(ports))]
    while(checkThreadFlag() and len(portIdxs)):
        
        msg = None
        while(checkThreadFlag() and (msg is None or ser_getFrequency(i) < 30)):
            time.sleep(0.5)
            print(ser_getFrequency(i))
            msg = ser_getLastPacket(i)

        if(checkThreadFlag()):
            ser_startLogging(i)
            portIdxs = portIdxs[1:]

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
