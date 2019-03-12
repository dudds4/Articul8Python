from articul8_comm import *
from msg_defs import *
from threadman import *
from quaternion import *
from movement import *

import sys
import time
import math
import socket
import numpy as np

tcpConnection = None
tcpLock = threading.Lock()

serialLocks = [threading.Lock() for i in range(len(ports))]

exercising = False
baselineImuData = [None] * len(ports)

recording = False
recordedMovement = Movement()

latestImuData = [None] * len(ports)

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
                            if not(recording):
                                recording = True
                                recordedMovement = Movement()
                                print("Starting Recording!")

                        elif (cmd[POS_DATA+1] == STOP_RECORDING):
                            if (recording):
                                recording = False
                                print("Ending Recording!")
                                print("Recording size: {}".format(len(recordedMovement.stateVector)))

                        elif (cmd[POS_DATA+1] == START_EXERCISE):
                            if not(exercising):
                                exercising = True
                                baselineImuData = latestImuData.copy()
                                print("Starting Exercise!")

                        elif (cmd[POS_DATA+1] == STOP_EXERCISE):
                            if (exercising):
                                exercising = False
                                print("Ending Exercise!")

                        elif (cmd[POS_DATA+1] == PRINT_RECORDING):
                            print(recordedMovement)

                        elif (cmd[POS_DATA+1] == REPORT_OFFSETS):
                            for i, port in enumerate(ports):
                                sendSerial(OffsetMsg.toBytes(), i)

                        elif (cmd[POS_DATA+1] == CALIBRATE):
                            msg = CalibrateMsg(cmd[POS_DATA+1])
                            for i, port in enumerate(ports):
                                sendSerial(msg.toBytes(), i)

                        elif (cmd[POS_DATA+1] == PRINT_BATTERY):
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
    global tcpConnection, latestImuData, recording, recordedMovement

    sendCounters = [0] * len(ports)
    recvCounters = [0] * len(ports)
    latestImuData = [None] * len(ports)

    while(checkThreadFlag()):

        for i, port in enumerate(ports):

            if(not ser_isOpen(i)):
                ser_open(port)

            msg = ser_getLastPacket(i)

            if(msg == None):
                time.sleep(0.001)
                continue

            parsed_message = None
            if (msg[POS_DATA] == IMU_DATA_MSG):
                parsed_message = IMUDataMsg.fromBytes(msg)
                latestImuData[i] = parsed_message

                if (recording):
                    if None in latestImuData:
                        print("latestImuData contained None in Port {}".format(latestImuData.find(None)))
                    else:
                        recordedMovement.update(latestImuData)

                if (tcpConnection is not None):
                    sendTCP(buildQuatMsgTCP(int(i / 2), i % 2, parsed_message.quat))

                    sendCounters[i] += 1
                    if sendCounters[i] >= 100:
                        print('Sent {} IMU Packets from Port {}'.format(sendCounters[i], i))
                        sendCounters[i] = 0
                else:
                    pass

                recvCounters[i] += 1
                if recvCounters[i] >= 100:
                    print('Got {} IMU Packets from Port {}'.format(recvCounters[i], i))
                    recvCounters[i] = 0

            elif (msg[POS_DATA] == OFFSET_REPORT_MSG):
                parsed_message = OffsetMsg.fromBytes(msg)
                print("{} from Port {}".format(parsed_message, i))

            elif (msg[POS_DATA] == BATTERY_REPORT_MSG):
                parsed_message = BatteryReportMsg.fromBytes(msg)
                print("{} from Port {}".format(parsed_message, i))

            else:
                continue

        time.sleep(0.015)

    print("Bluetooth Worker Quiting...")
    threadQuit()


LRA_WORKER_PERIOD = 0.5
def testLraSpinWorker():
    global tcpConnection

    count = 0
    while(checkThreadFlag()):
        # newLraMsg = LRACmdMsg(True, 0.5).toBytes()
        if (count < 10):
            newLraMsg = LRACmdMsg(True, -0.5).toBytes()
        elif (count < 20):
            newLraMsg = LRACmdMsg(False, [0]*numLRAs[i]).toBytes()
        elif (count < 30):
            newLraMsg = LRACmdMsg(True, 0.5).toBytes()
        else:
            newLraMsg = LRACmdMsg(False, [0]*numLRAs[i]).toBytes()

        print("Writing... {}".format(count))
        for i, port in enumerate(ports):
            sendSerial(newLraMsg, i)

        if (tcpConnection is not None):
            sendTCP(buildLRAMsgTCP(int(i/2), i%2, newLraMsg))

        time.sleep(LRA_WORKER_PERIOD)
        count += 1

    print("LRA Command Worker Quiting...")
    threadQuit()


def lraControlWorker():
    global tcpConnection, latestImuData, baselineImuData, recordedMovement, recording, exercising

    offLraMsgs  = [LRACmdMsg(False, [0] * numLRAs[i]).toBytes() for i in range(len(ports))]
    lastLraMsgs = [None] * len(ports)

    while(checkThreadFlag()):

        newLraMsgs = offLraMsgs

        if (exercising and not(recording) and not(None in latestImuData)):
            newLraMsgs = recordedMovement.getLraMsgs(latestImuData, baselineImuData)

        for i, port in enumerate(ports):

            if (newLraMsgs[i] != lastLraMsgs[i]):
                sendSerial(newLraMsgs[i], i)

                lastLraMsgs[i] = newLraMsgs[i]

                # TODO: Send all LRAs over TCP
                if (tcpConnection is not None):
                    sendTCP(buildLRAMsgTCP(int(i/2), i%2, newLraMsgs[i]))

        time.sleep(LRA_WORKER_PERIOD)

    print("LRA Command Worker Quiting...")
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
