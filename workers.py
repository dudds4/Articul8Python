from articul8_comm import *
from msg_defs import *
from threadman import *

import sys
import time
import math
import socket

tcpConnection = None

exercising = False
recording = False
recordedMovement = []

imuDataAvailable = 0
latestImuData = ''

LRA_WORKER_PERIOD = 0.3
MIN_REC_MOVEMENT = 5
VIBRATE_THRESHOLD_DIST = 0.087 # 5 deg in rad

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
    tcpConnection, addr = s.accept()
    tcpConnection.setblocking(1)
    print('TCP server connected on: {}'.format(addr))

    try:
        connectionFailed = False
        sendCounter = 0

        while (not(connectionFailed) and checkThreadFlag()):
            # Send an empty string - does nothing unless connection is broken, in which case
            # it throws an exception and the connection is set to None
            # tcpConnection.send('')

            # Check if any commands are being sent
            try:
                cmd = tcpConnection.recv(PACKET_SIZE)

                if (ord(cmd[POS_SOP]) == SOP and ord(cmd[POS_DATA]) == GUI_CONTROL_MSG):

                    if (cmd[POS_DATA+1] == chr(START_RECORDING)):
                        if not(recording):
                            recording = True
                            recordedMovement = []
                            print("Starting Recording!")

                    elif (cmd[POS_DATA+1] == chr(STOP_RECORDING)):
                        if (recording):
                            recording = False
                            print("Ending Recording!")
                            print("Recording size: {}".format(len(recordedMovement)))

                    elif (cmd[POS_DATA+1] == chr(START_EXERCISE)):
                        if not(exercising):
                            exercising = True
                            print("Starting Exercise!")

                    elif (cmd[POS_DATA+1] == chr(STOP_EXERCISE)):
                        if (exercising):
                            exercising = False
                            print("Ending Exercise!")

                    elif (cmd[POS_DATA+1] == chr(PRINT_RECORDING)):
                        print(recordedMovement)

                    else:
                        print("Received invalid command: {}".format(cmd))
                        connectionFailed = True
            except Exception:
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

    MAX_NUM_ERRORS = 4
    sendCounter = 0
    recvCounter = 0

    while(checkThreadFlag()):
        msg = ser_getLastPacket()
        if(msg == None):
            time.sleep(0.01)
            continue

        parsed_message = None
        if (msg[POS_DATA] == IMU_DATA_MSG):
            parsed_message = IMUDataMsg.fromBytes(msg)

            if (recording):
                recordedMovement.append(parsed_message)

            if (tcpConnection is not None):
                # Only send over 1 in 2 so Processing doesn't get overwhelmed
                if (sendCounter % 2 == 0):
                    tcpConnection.send(msg)
    
                latestImuData = parsed_message
                sendCounter += 1
                if sendCounter >= 50:
                    print('Sent {} IMU Packets'.format(sendCounter))
                    sendCounter = 0

            recvCounter += 1
            if recvCounter >= 50:
                print('Got {} IMU Packets'.format(recvCounter))
                recvCounter = 0

        elif (msg[POS_DATA] == ACK_MSG):
            parsed_message = ACKMsg.fromBytes(msg)

        elif (msg[POS_DATA] == STANDBY_MSG):
            parsed_message = StandbyMsg.fromBytes(msg)

        else:
            # print("Invalid Data Type")
            continue

        # print("Received: {}".format(parsed_message))

    print("Bluetooth Worker Quiting...")
    threadQuit()

def lraControlWorker():
    global tcpConnection, latestImuData

    onIntensities = [int(127)]*8
    onMsgBytes = LRACmdMsg(onIntensities).toBytes()

    offIntensities = [int(0)]*8
    offMsgBytes = LRACmdMsg(offIntensities).toBytes()

    lastLraMsg = offMsgBytes

    while(checkThreadFlag()):

        newLraMsg = offMsgBytes

        if (exercising and not(recording) and latestImuData != None
            and len(recordedMovement) >= MIN_REC_MOVEMENT):

            minDist = VIBRATE_THRESHOLD_DIST + 1
            imuGrav = quatToGravity(latestImuData.quat)
            magImuGrav = math.sqrt(sum([elem*elem for elem in imuGrav]))

            for movementPoint in recordedMovement:

                mvGrav = quatToGravity(movementPoint.quat)
                magMvGrav = math.sqrt(sum([elem*elem for elem in mvGrav]))

                if (magMvGrav != 0 and magImuGrav != 0):
                    normDot = (imuGrav[0]*mvGrav[0] + imuGrav[1]*mvGrav[1] + imuGrav[2]*mvGrav[2]) / (magMvGrav * magImuGrav)
                    dist = math.acos(normDot)
                else:
                    dist = 0

                if (dist >= VIBRATE_THRESHOLD_DIST and dist < minDist):
                    minDist = dist
                    z_dist = mvGrav[2] - imuGrav[2]
                    y_dist = mvGrav[1] - imuGrav[1]

                    mag = math.sqrt(y_dist**2 + z_dist**2)
                    angle = math.atan2(y_dist, z_dist)
                    if (angle < 0):
                        angle += 2*math.pi

                    idx1 = 0
                    while ((idx1+1) * math.pi/4 < angle):
                        idx1 += 1

                    idx2 = (idx1 + 1) % 8
                    portion2 = (angle - idx1 * math.pi/4) / (math.pi / 4)
                    portion1 = 1 - portion2

                    if (portion1 > 1 or portion1 < 0):
                        print("portioning error")
                        print("Angle {} idx1 {} portion1 {}".format(angle, idx1, portion1))

                    intensities = [0] * 8
                    intensities[idx1] = mag * portion1
                    intensities[idx2] = mag * portion2
                    intensities = [int(min(127, 600*elem)) for elem in intensities]

                    newLraMsg = LRACmdMsg(intensities).toBytes()

        if (newLraMsg != lastLraMsg):
            articulate_board.write(newLraMsg)
            if (tcpConnection is not None):
                tcpConnection.send(newLraMsg)
                tcpConnection.send(newLraMsg) # Twice for good measure
            lastLraMsg = newLraMsg

            time.sleep(LRA_WORKER_PERIOD)

    print("LRA Command Worker Quiting...")
    threadQuit()

def keepAliveWorker():
    # Send periodic messages to continue streaming 
    while(checkThreadFlag()):
        print("Sending stream message")
        msg = StreamMsg(15)    # Set IMU streaming period (in ms)
        if(not ser_isOpen()):
            ser_open(port)

        ser_write(msg.toBytes())
        time.sleep(1)

    print("Keep Alive Worker Quiting...")
    threadQuit()
