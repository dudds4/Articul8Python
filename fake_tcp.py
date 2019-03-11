from msg_defs import *
from articul8_comm import *
import sys
import os
import glob
import matplotlib.pyplot as plt
import time
import math
import socket
import struct

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

tcpConnection = None
s = None

def tcpSetup(nRetries):
    global tcpConnection, s
    
    HOST = ''               # Symbolic name meaning all available interfaces
    PORT = 5432             # Arbitrary non-privileged port

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(2)
    s.settimeout(0.2)    

    i = 0
    while(tcpConnection == None and i < nRetries):
        try:
            tcpConnection, addr = s.accept()
            tcpConnection.settimeout(1)
            print('TCP server connected on: {}'.format(addr))

        except Exception:
            pass

        i += 1

    return (tcpConnection is not None)

SOP = 253
QUAT_MSG = 100
MSG_SIZE = 24

def buildQuatMsg(left, upper, q):
    l = 1 if left else 0
    u = 1 if upper else 0
    msg = struct.pack('B', SOP) + struct.pack('B', QUAT_MSG) +  struct.pack('B', l) + struct.pack('B', u)

    for i in range(4):
        msg = msg + struct.pack('f', q[i])
    
    while(len(msg) < MSG_SIZE):
        msg = msg + struct.pack('B', 0)

    return msg

def clockOutOverTcp(imuMsgLists):
    nIMUs = len(imuMsgLists)
    nMsgs = len(imuMsgLists[0])
    for i in range(1,nIMUs):
        nMsgs = min(nMsgs, len(imuMsgLists[i]))

    global tcpConnection, s
    time.sleep(1)
    
    print("Trying to connect to TCP")
    success = tcpSetup(100)

    if(not success):
        print("TCP Quitting... couldn't connect to client")
        return

    print("Succeeded")

    try:
        connectionFailed = False
        sendCounter = 0
        lastTime = imuMsgLists[0][0].time
        while ((not connectionFailed) and (sendCounter < nMsgs)):

            nextTime = 0
            for i in range(2):
                imuMsg = imuMsgLists[i][sendCounter]
                left = False
                upper = False if i == 0 else True

                msg = buildQuatMsg(left, upper, imuMsg.quat)
                tcpConnection.send(msg)
                # print(msg)
                nextTime = max(nextTime, imuMsg.time)

            sendCounter += 1
            sTime = (nextTime - lastTime)/1000.0
            time.sleep(sTime)
            lastTime = nextTime

    except Exception as e:
        print("Exceptioned in tcp")
        print(e)
        pass

    finally:
        if tcpConnection is not None:
            print('Closing socket connection')
            tcpConnection.close()
        else:
            print('No socket connection')

def main():
    loadCSerial()

    if(len(sys.argv) != 2):
        print("Please supply exactly 1 folder containing log files")
    else:
        imuDataSets = readIMUData(sys.argv[1])
        clockOutOverTcp(imuDataSets)



if __name__ == '__main__':
    main()