from articul8_comm import *
from articul8_logs import *
from msg_defs import *
from workers import *
from threadman import *

import math
import sys
import time
import string
import signal
import platform
import os
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt

firstImuData = [None] * len(ports)

def angleDifference(quat1, quat2):
    quat1 = Quat(quat1)
    quat2 = Quat(quat2)
    return 180*2*math.acos(np.clip((quat1*quat2.inv())[0], -1, 1))/math.pi

def orientationChangeWorker():
    f = open('lastOrientationTest.txt', 'w')

    while(checkThreadFlag()):
        t = time.time()

        for i, port in enumerate(ports):

            if(not ser_isOpen(i)):
                ser_open(port)

            msg = ser_getLastPacket(i)

            if(msg == None):
                time.sleep(0.01)
                continue

            parsed_message = None
            if (msg[POS_DATA] == IMU_DATA_MSG):
                parsed_message = IMUDataMsg.fromBytes(msg)

                if (firstImuData[i] is None):
                    firstImuData[i] = parsed_message

                outputStr = '{},{},{}\n'.format(i, parsed_message.time, angleDifference(firstImuData[i].quat, parsed_message.quat))
                f.write(outputStr)
                print(outputStr)

        time.sleep(1)

    print("Orientation Change Worker Quiting...")
    f.close()
    threadQuit()

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        print('Cleaning up')
        setThreadFlag(False)
        while(checkThreadCount() > 0):
            print('{} threads remaining'.format(checkThreadCount()))
            time.sleep(1)

        ser_cleanup();
        sys.exit(0)

def plotLogs(filename):
    f = open(filename, 'r')
    c_f = csv.reader(f, delimiter=',')
    data = np.array(list(c_f), dtype=float)
    f.close()
    # Shift time to start at 0 and angles to +/- 180 deg
    data[np.where(data[:,2] > 180)] -= 360

    data0 = data[np.where(data[:,0] == 0)]
    data0[:,1] -= data0[0][1]
    data0[np.where(data0[:,1] < 0),1] += 65536

    data1 = data[np.where(data[:,0] == 1)]
    data1[:,1] -= data1[0][1]
    data1[np.where(data1[:,1] < 0),1] += 65536

    plt.plot(data0[:,1], data0[:,2])
    plt.plot(data1[:,1], data1[:,2])
    plt.plot([np.min(data0[:,1]), np.max(data0[:,1])],[90, 90], 'k--')
    plt.plot()
    plt.show()

def main():

    if len(sys.argv) > 1:
        filename = sys.argv[1]
        plotLogs(filename)
        return

    loadCSerial()
    signal.signal(signal.SIGINT, signal_handler)

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

    print('Starting orientation change thread')
    orientationChangeThread = startThread(orientationChangeWorker)

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
