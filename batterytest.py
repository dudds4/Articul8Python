from articul8_comm import *
from articul8_logs import *
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

def spinThreadWorker():
    spinMessage = LRACmdMsg(True, 0.3).toBytes()

    while(checkThreadFlag()):
        for i, port in enumerate(ports):
            sendSerial(spinMessage, i)

        time.sleep(1)

    print("Spin Worker Quiting...")
    threadQuit()

def batteryCheckWorker():
    while(checkThreadFlag()):
        print(time.localtime())

        for i, port in enumerate(ports):
            sendSerial(BatteryReportMsg.toBytes(), i)

        time.sleep(2)

    print("Battery Check Worker Quiting...")
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

def main():

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

    print('Starting spin thread')
    spinThread = startThread(spinThreadWorker)

    print('Starting battery check')
    batteryCheckThread = startThread(batteryCheckWorker)

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
