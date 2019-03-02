from articul8_comm import *
from msg_defs import *

import sys

import atexit
import time
import socket

import random
import string
import threading

STARTING_STREAM = 0
STREAMING = 1
state = STARTING_STREAM

fsmState = 0
fsmCounter = 0

imuDataAvailable = 0
latestImuData = ''

def bluetoothWorker():

    global imuDataAvailable
    global latestImuData

    MAX_NUM_ERRORS = 4

    while(True):
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

port = "COM10"
def keepAliveWorker():
    # Send periodic messages to continue streaming 

    while(True):
        print("Sending stream message")
        msg = StreamMsg(20)    # Set IMU streaming period (in ms)
        if(not ser_isOpen()):
        	ser_open(port)

        ser_write(msg.toBytes())
        time.sleep(1)

def main():
    loadCSerial()

    i = 0
    while(not ser_isOpen() and i < 5):
        ser_open(port)

    if(not ser_isOpen()):
        print("Couldn't open COM port")
        sys.exit()

    print("Starting bt thread")
    btThread = threading.Thread(target=bluetoothWorker)
    btThread.start()

    print('Starting keep alive thread')
    keepAliveThread = threading.Thread(target=keepAliveWorker)
    keepAliveThread.start()

    while(True):
        time.sleep(1)

if __name__ == '__main__':
    main()