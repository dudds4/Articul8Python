from articul8_comm import *
from articul8_logs import *
from msg_defs import *
from threadman import *

import math
import socket
import numpy as np
import sys
import time
import string
import signal
import platform
import os
import glob
import matplotlib.pyplot as plt

tcpConnection = None
tcpLock = threading.Lock()

serialLocks = [threading.Lock() for i in range(len(ports))]

def sendSerial(msg, i):
    global serialLocks
    serialLocks[i].acquire()

    try:
        if(not ser_isOpen(i)):
            ser_open(ports[i])

        ser_write(msg, i)

    except:
        pass

    serialLocks[i].release()

def sendTCP(msg):
    global tcpConnection, tcpLock
    tcpLock.acquire()

    try:

        if(tcpConnection is not None):
            tcpConnection.send(msg)

    except:
        pass

    tcpLock.release()

# For now if you close the GUI you have to restart python
def tcpServerWorker():
    global tcpConnection, exercising, recording, recordedMovement, baselineImuData

    tcpConnection = None

    time.sleep(1)
    HOST = ''               # Symbolic name meaning all available interfaces
    PORT = 5432             # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(2)
    s.settimeout(0.2)
    tcpConnection = None

    while (checkThreadFlag()):

        while(tcpConnection == None and checkThreadFlag()):
            try:
                tcpConnection, addr = s.accept()
                tcpConnection.settimeout(1)
                print('TCP server connected on: {}'.format(addr))

            except Exception:
                pass

        try:
            connectionFailed = False
            sendCounter = 0

            while (not(connectionFailed) and checkThreadFlag()):
                # Check if any commands are being sent
                try:
                    cmd = tcpConnection.recv(PACKET_SIZE)
                    if(cmd is None):
                        continue

                    if (cmd[POS_SOP] == SOP and cmd[POS_DATA] == GUI_CONTROL_MSG):

                        if (cmd[POS_DATA+1] == START_RECORDING):
                            print('Starting recording')
                            ser_startRecording()

                        elif (cmd[POS_DATA+1] == STOP_RECORDING):
                            print('Stopping recording')
                            ser_stopRecording()

                        elif (cmd[POS_DATA+1] == START_EXERCISE):
                            print('Starting exercise')
                            ser_startExercise()

                        elif (cmd[POS_DATA+1] == STOP_EXERCISE):
                            print('Stopping exercise')
                            ser_stopExercise()

                        elif (cmd[POS_DATA+1] == REPORT_OFFSETS):
                            print('Requesting offsets')
                            for i, port in enumerate(ports):
                                sendSerial(OffsetMsg.toBytes(), i)

                        elif (cmd[POS_DATA+1] == CALIBRATE):
                            print('Calibrating')
                            msg = CalibrateMsg(cmd[POS_DATA+1])
                            for i, port in enumerate(ports):
                                sendSerial(msg.toBytes(), i)

                        elif (cmd[POS_DATA+1] == PRINT_BATTERY):
                            print('Requesting battery report')
                            for i, port in enumerate(ports):
                                sendSerial(BatteryReportMsg.toBytes(), i)

                        else:
                            print("Received invalid command: {}".format(cmd))
                            connectionFailed = True

                except Exception as e:
                    if (str(e) != 'timed out'):
                        connectionFailed = True
        finally:
            if tcpConnection is not None:
                print('Closing socket connection')
                tcpConnection.close()
            else:
                print('No socket connection')

            tcpConnection = None
            print('Restarting TCP server')

    print('TCP Server Worker Quitting')
    threadQuit()

def bluetoothWorker():
    global tcpConnection

    while(checkThreadFlag()):

        for i, port in enumerate(ports):

            if(not ser_isOpen(i)):
                ser_open(port)

            msg = ser_getLastPacket(i)

            if (msg is not None):

                if (msg[POS_DATA] == IMU_DATA_MSG and tcpConnection is not None):

                    parsed_message = IMUDataMsg.fromBytes(msg)
                    sendTCP(buildQuatMsgTCP(int(i/2), (i+1)%2, parsed_message.quat))
                    time.sleep(0.01)

                elif (msg[POS_DATA] == OFFSET_REPORT_MSG):

                    parsed_message = OffsetMsg.fromBytes(msg)
                    print("{} from Port {}".format(parsed_message, i))

                elif (msg[POS_DATA] == BATTERY_REPORT_MSG):

                    parsed_message = BatteryReportMsg.fromBytes(msg)
                    print("{} from Port {}".format(parsed_message, i))

                else:
                    continue


            lraMsg = ser_getLraPacket(i)

            if (lraMsg is not None and tcpConnection is not None):

                sendTCP(buildLRAMsgTCP(int(i/2), (i+1)%2, lraMsg))
                time.sleep(0.01)

        time.sleep(0.002)

    print("Bluetooth Worker Quiting...")
    threadQuit()

def keepAliveWorker():
    global ports
    # Send periodic messages to continue streaming 
    while(checkThreadFlag()):
        # print("Sending stream message")
        msg = StreamMsg(20)    # Set IMU streaming period (in ms)
        
        for i, port in enumerate(ports):
            sendSerial(msg.toBytes(), i)
        
        time.sleep(1)

    print("Keep Alive Worker Quiting...")
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

    print('Starting TCP server thread')
    tcpServerThread = startThread(tcpServerWorker)

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
