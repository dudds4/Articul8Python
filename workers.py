from articul8_comm import *
from msg_defs import *
from threadman import *
from quaternion import *

import sys
import time
import math
import socket
import numpy as np

tcpConnection = None
tcpLock = threading.Lock()

exercising = False
recording = False
recordedMovement = []

latestImuData = [None] * len(ports)

NUM_LRAS = 8
LRA_WORKER_PERIOD = 0.33
MIN_REC_MOVEMENT = 5
VIBRATE_THRESHOLD_DIST = 0.087 # 5 deg in rad

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
    global tcpConnection, exercising, recording, recordedMovement

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
                            recordedMovement = []
                            print("Starting Recording!")

                    elif (cmd[POS_DATA+1] == STOP_RECORDING):
                        if (recording):
                            recording = False
                            print("Ending Recording!")
                            print("Recording size: {}".format(len(recordedMovement)))

                    elif (cmd[POS_DATA+1] == START_EXERCISE):
                        if not(exercising):
                            exercising = True
                            print("Starting Exercise!")

                    elif (cmd[POS_DATA+1] == STOP_EXERCISE):
                        if (exercising):
                            exercising = False
                            print("Ending Exercise!")

                    elif (cmd[POS_DATA+1] == PRINT_RECORDING):
                        print(recordedMovement)

                    elif (cmd[POS_DATA+1] == REPORT_OFFSETS):
                        for i, port in enumerate(ports):
                            if(not ser_isOpen()):
                                ser_open(port)

                            ser_write(OffsetMsg.toBytes(), i)

                    elif (cmd[POS_DATA+1] == CALIBRATE_ACCEL or cmd[POS_DATA+1] == CALIBRATE_GYRO):
                        msg = CalibrateMsg(cmd[POS_DATA+1])
                        for i, port in enumerate(ports):
                            if(not ser_isOpen()):
                                ser_open(port)

                            ser_write(msg.toBytes())

                    elif (cmd[POS_DATA+1] == PRINT_BATTERY):
                        for i, port in enumerate(ports):
                            if(not ser_isOpen()):
                                ser_open(port)

                            ser_write(BatteryReportMsg.toBytes(), i)

                    else:
                        print("Received invalid command: {}".format(cmd))
                        connectionFailed = True

            except Exception as e:
                pass
    finally:
        if tcpConnection is not None:
            print('Closing socket connection')
            tcpConnection.close()
        else:
            print('No socket connection')

        tcpConnection = None

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
                        recordedMovement.append(latestImuData.copy())

                # TODO: Send all latestImuData over TCP
                if (tcpConnection is not None and i == 0):
                    sendTCP(msg)

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


def testLrasWorker():
    global tcpConnection

    lraId = 0

    while(checkThreadFlag()):
        intensities = [int(0)] * NUM_LRAS
        intensities[lraId] = 127
        newLraMsg = LRACmdMsg(intensities).toBytes()

        for i, port in enumerate(ports):
            if(not ser_isOpen(i)):
                ser_open(port)

            ser_write(newLraMsg, i)

        if (tcpConnection is not None):
            sendTCP(newLraMsg)

        time.sleep(LRA_WORKER_PERIOD * 3)

        lraId += 1
        if (lraId >= NUM_LRAS):
            lraId = 0

    print("LRA Command Worker Quiting...")
    threadQuit()


def lraControlWorker():
    global tcpConnection, latestImuData, recordedMovement, recording, exercising

    lastLraMsgs = [None] * len(ports)

    while(checkThreadFlag()):

        for i, port in enumerate(ports):

            intensities = [0] * NUM_LRAS

            if (exercising and not(recording) and latestImuData[i] is not None):

                nearestMag = None
                nearestAngle = None
                minDist = math.inf

                imuGrav = quatToGravity(latestImuData[i].quat)
                magImuGrav = np.linalg.norm(imuGrav)

                for movementPoint in recordedMovement:

                    mvGrav = quatToGravity(movementPoint[i].quat)
                    magMvGrav = np.linalg.norm(mvGrav)

                    if (magMvGrav == 0 or magImuGrav == 0):
                        continue

                    normDot = np.dot(imuGrav, mvGrav) / (magMvGrav * magImuGrav)
                    dist = math.acos(np.clip(normDot, -1, 1))

                    if (dist >= VIBRATE_THRESHOLD_DIST and dist < minDist):
                        minDist = dist
                        z_dist = mvGrav[2] - imuGrav[2]
                        y_dist = -mvGrav[1] + imuGrav[1]

                        nearestMag = math.sqrt(y_dist**2 + z_dist**2)
                        nearestAngle = math.atan2(y_dist, z_dist)
                        if (nearestAngle < 0):
                            nearestAngle += 2*math.pi

                if (nearestAngle is not None and nearestMag is not None):
                    # Linearly interpolate magnitude between two adjacent LRAs
                    angleSegment = 2*math.pi/NUM_LRAS

                    idx1 = 0
                    while ((idx1+1) * angleSegment < nearestAngle):
                        idx1 += 1

                    idx2 = (idx1 + 1) % NUM_LRAS

                    portion2 = (nearestAngle - idx1 * angleSegment) / angleSegment
                    portion1 = 1 - portion2

                    intensities[idx1] = nearestMag * portion1
                    intensities[idx2] = nearestMag * portion2
                    intensities = [int(min(127, 600*elem)) for elem in intensities]

            newLraMsg = LRACmdMsg(intensities).toBytes()

            if (newLraMsg != lastLraMsgs[i]):

                if(not ser_isOpen(i)):
                    ser_open(port)

                ser_write(newLraMsg, i)
                lastLraMsgs[i] = newLraMsg

                # TODO: Send all LRAs over TCP
                if (tcpConnection is not None and i == 0):
                    sendTCP(newLraMsg)

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
            if(not ser_isOpen(i)):
                ser_open(port)

            ser_write(msg.toBytes(), i)
        
        time.sleep(1)

    print("Keep Alive Worker Quiting...")
    threadQuit()
