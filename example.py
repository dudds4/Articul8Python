from articul8_comm import *
from msg_defs import *

import sys

import atexit
import time
import socket

import random
import string
import threading
import signal

STARTING_STREAM = 0
STREAMING = 1
state = STARTING_STREAM

fsmState = 0
fsmCounter = 0

imuDataAvailable = 0
latestImuData = ''

threadFlag = True
threadFlagLock = threading.Lock()
threadCount = 0

def checkThreadCount():
    global threadCount
    global threadFlagLock
    threadFlagLock.acquire()
    x = threadCount
    threadFlagLock.release()
    return x

def startThread(x):
    global threadCount
    global threadFlagLock
    thread = threading.Thread(target=x)
    thread.start()

    threadFlagLock.acquire()
    threadCount+=1
    threadFlagLock.release()

    return thread    

def threadQuit():
    global threadCount
    global threadFlagLock

    threadFlagLock.acquire()
    threadCount -= 1
    threadFlagLock.release()

def setThreadFlag(x):
    global threadFlag
    global threadFlagLock
    
    threadFlagLock.acquire()
    threadFlag = x
    threadFlagLock.release()


def checkThreadFlag():
    global threadFlag
    global threadFlagLock
    
    threadFlagLock.acquire()
    x = threadFlag
    threadFlagLock.release()
    return x

def bluetoothWorker():

    global imuDataAvailable
    global latestImuData

    MAX_NUM_ERRORS = 4

    while(checkThreadFlag()):
        msg = ser_getLastPacket()
        if(msg == None):
            time.sleep(0.01)
            continue

        parsed_message = None
        if (msg[POS_DATA] == chr(IMU_DATA_MSG)):
            parsed_message = IMUDataMsg.fromBytes(msg)
            if (imuDataAvailable == False):
                latestImuData = msg[2:18]
                imuDataAvailable = True

        elif (msg[POS_DATA] == chr(ACK_MSG)):
            parsed_message = ACKMsg.fromBytes(msg)

        elif (msg[POS_DATA] == chr(STANDBY_MSG)):
            parsed_message = StandbyMsg.fromBytes(msg)

        else:
            # print("Invalid Data Type")
            continue

        # print("Received: {}".format(parsed_message))

    print("Bluetooth Worker Quiting...")
    threadQuit()

port = "COM10"
def keepAliveWorker():
    # Send periodic messages to continue streaming 

    while(checkThreadFlag()):
        print("Sending stream message")
        msg = StreamMsg(20)    # Set IMU streaming period (in ms)
        if(not ser_isOpen()):
            ser_open(port)

        ser_write(msg.toBytes())
        time.sleep(1)

    print("Keep Alive Worker Quiting...")
    threadQuit()

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        print('Cleaning up')
        setThreadFlag(False)
        while(checkThreadCount() > 0):
            time.sleep(1)

        ser_cleanup();
        sys.exit(0)

def main():
    loadCSerial()
    signal.signal(signal.SIGINT, signal_handler)
    
    i = 0
    while(not ser_isOpen() and i < 5):
        ser_open(port)

    if(not ser_isOpen()):
        print("Couldn't open COM port")
        sys.exit()

    print("Starting bt thread")
    btThread = startThread(bluetoothWorker)

    print('Starting keep alive thread')
    keepAliveThread = startThread(keepAliveWorker)

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()