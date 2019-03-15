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

def commandWorker():

    while(checkThreadFlag()):

        x = input("Please enter command: ")
        msg = None

        if (x == 'b'):
            msg = BatteryReportMsg.toBytes()

        elif (x == 'c'):
            msg = CalibrateMsg(0).toBytes()

        elif (x == 'o'):
            msg = OffsetMsg.toBytes()

        elif (x == 'spin'):
            msg = LRACmdMsg(True, 0.8).toBytes()

        elif (x == 'stop'):
            msg = LRACmdMsg(False, [0]*8).toBytes()

        else:
            continue

        x = None

        for i, port in enumerate(ports):

            if(not ser_isOpen(i)):
                ser_open(port)

            ser_write(msg, i)

        time.sleep(0.2)

    print("Command Worker Quiting...")
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

    print('Starting command thread')
    commandThread = startThread(commandWorker)

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
