# packet related defs

PACKET_SIZE = 24
PACKET_OVERHEAD = 2
SOP = 253
POS_SOP = 0
POS_DATA = 1
POS_CHECKSUM = (PACKET_SIZE-1)

# message related defines
MSG_OVERHEAD = 1

NONE_MSG = 0
ACK_MSG = 1
STANDBY_MSG = 2
STATE_CHANGE_MSG = 3
IMU_DATA_MSG = 4
LRA_CONTROL_MSG = 5
GUI_CONTROL_MSG = 6
CALIBRATE_MSG = 7
OFFSET_REPORT_MSG = 8
BATTERY_REPORT_MSG = 9
NUM_MSG_TYPES = 10

# streaming states

DEFAULT_STATE = 0
IMU_STREAMING_STATE = 1
INVALID_STATE = 2
NUM_BOARD_STATES = 3

START_RECORDING = 0
STOP_RECORDING = 1
START_EXERCISE = 2
STOP_EXERCISE = 3
PRINT_RECORDING = 4
CALIBRATE_ACCEL = 5
CALIBRATE_GYRO = 6
REPORT_OFFSETS = 7
PRINT_BATTERY = 8

import struct
import binascii

AOK = 'AOK'
ERR = 'ERR'

def wrapDataInPacket(data):

    padLen = PACKET_SIZE - PACKET_OVERHEAD - len(data)
    if(padLen < 0):
        return None

    checksum = ((sum([x for x in data])) % 256)

    msg = struct.pack('B', SOP) + data

    # print(str(msg))

    # print('{} {} {}'.format(padLen, len(data), len(str(data))))
    # msg = struct.pack('B', 0) + msg
    for i in range(padLen):
        msg += struct.pack('B', 0)

    msg += struct.pack('B', checksum)

    # print(len(msg))

    if(len(msg) != PACKET_SIZE):
        print("wtf?{}".format(len(msg)))

    # msg = str(msg)
    # r = struct.struct.unpack('40p', msg)[0].decode()
    # print(r)
    # print("STR:")
    # print(str(r))
    # x = str(msg.decode())
    return msg

class StreamMsg:
    def __init__(self, p, ok=True):
        self.period = int(p)
        self.isOk = ok

    def toBytes(self):
        data = struct.pack('B', STATE_CHANGE_MSG) + struct.pack('<B', IMU_STREAMING_STATE) + struct.pack('<i', self.period)
        return wrapDataInPacket(data)

    def fromBytes(cls, bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(STREAM_MSG)):
            return None

        period = struct.unpack('<i', bytes[1:5].encode())
        isOk = bytes[5:7] == AOK

        return StreamMsg(period, isOk)

class CalibrateMsg:
    def __init__(self, cal_type):
        self.cal_type = int(cal_type)

    def toBytes(self):
        data = struct.pack('B', CALIBRATE_MSG) + struct.pack('<i', self.cal_type)
        return wrapDataInPacket(data)

class LRACmdMsg:
    def __init__(self, intensities):
        # TODO: Check values are integers <127
        self.intensities = intensities

    def toBytes(self):
        data = struct.pack('B', LRA_CONTROL_MSG)
        for i in range(8):
            data += struct.pack('B', int(self.intensities[i]))
        return wrapDataInPacket(data)

    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(LRA_CONTROL_MSG)):
            return None

        new_intensities = []
        for i in range(4):
            val = struct.unpack('B', bytes[1+i:2+i])
            new_intensities.append(val[0])

        return LRACmdMsg(new_intensities)

class ACKMsg:
    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(ACK_MSG)):
            return None

        return ACKMsg()

    def __str__(self):
        return "ACK"

class StandbyMsg:
    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != chr(ACK_MSG)):
            return None

        return StandbyMsg()

    @staticmethod
    def toBytes():
        data = struct.pack('B', STANDBY_MSG)
        return wrapDataInPacket(data)

    def __str__(self):
        return "STANDBY"

class OffsetMsg:
    def __init__(self, accelData, gyroData):
        self.accelData = accelData
        self.gyroData = gyroData

    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != OFFSET_REPORT_MSG):
            return None

        accelData = []
        for i in range(3):
            val = struct.unpack('h', bytes[1+2*i:3+2*i])
            accelData.append(val[0])

        gyroData = []
        for i in range(3):
            val = struct.unpack('h', bytes[7+2*i:9+2*i])
            gyroData.append(val[0])

        return OffsetMsg(accelData, gyroData)

    # To request a report, send empty offset report message to the board
    @staticmethod
    def toBytes():
        data = struct.pack('B', OFFSET_REPORT_MSG)
        return wrapDataInPacket(data)

    def __str__(self):
        return "IMU Offset Update: Accel {}, Gyro {}".format(self.accelData, self.gyroData)

class IMUDataMsg:
    def __init__(self, quat, accel, timestamp):
        self.quat = quat
        self.xAccel = accel
        self.time = timestamp

    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != IMU_DATA_MSG):
            return None

        quatLen = 4
        quat = [0 for x in range(quatLen)]

        for i in range(quatLen):
            val = struct.unpack('f', bytes[1+4*i:5+4*i])
            quat[i] = val[0]

        z = struct.pack('B', 0)

        xAccelBytes = bytes[17:19]
        xAccel = struct.unpack('<i', xAccelBytes + 2*z)[0]

        timestampBytes = bytes[19:22]
        timestamp = struct.unpack('<i', timestampBytes + z)[0]

        return IMUDataMsg(quat, xAccel, timestamp)

    def __str__(self):
        return "quat: {}, xAccel: {}, time: {}".format(str(self.quat), self.xAccel, self.time)

class BatteryReportMsg:
    def __init__(self, battLevel):
        self.battLevel = battLevel

    @staticmethod
    def fromBytes(bytes):
        bytes = bytes[POS_DATA:POS_CHECKSUM]

        if(bytes[0] != BATTERY_REPORT_MSG):
            return None

        battLevel = struct.unpack('h', bytes[1:3])[0]

        return BatteryReportMsg(battLevel)

    @staticmethod
    def toBytes():
        data = struct.pack('B', BATTERY_REPORT_MSG)
        return wrapDataInPacket(data)

    def __str__(self):
        return "BATTERY: {} mV".format(self.battLevel)
