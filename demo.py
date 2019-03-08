from articul8_comm import *
from msg_defs import *
from workers import *
from threadman import *

import sys
import time
import string
import signal
import platform

port = "COM7"
if (platform.system().lower() == 'darwin'):
    port = "/dev/cu.Articul8Board3-SerialPo"

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        print('Cleaning up')
        setThreadFlag(False)
        while(checkThreadCount() > 0):
            print('{} threads remaining'.format(checkThreadCount()))
            time.sleep(1)

        ser_cleanup();
        sys.exit(0)

def exampleWriteLog():
    ser_startLogging()

    for i in range(0, 100):
        m = StreamMsg(20)
        ser_write(m.toBytes())
        time.sleep(0.02)

    ser_stopLogging()

def exampleReadLog():
    ser_openLog("articul8Log.Mar_08_17.15.58_201.csv")
    packet = 1
    while(packet != None):
        packet = ser_getLogPacket()
        if(packet != None):
            print(packet)                

def main():
    loadCSerial()
    signal.signal(signal.SIGINT, signal_handler)
    
    i = 0
    while(not ser_isOpen() and i < 5):
        ser_open(port)
        i += 1
        time.sleep(0.2)

    if(not ser_isOpen()):
        print("Couldn't open serial port")
        sys.exit()
        
    print("Starting bt thread")
    btThread = startThread(bluetoothWorker)

    print('Starting keep alive thread')
    keepAliveThread = startThread(keepAliveWorker)

    print('Starting TCP server thread')
    tcpServerThread = startThread(tcpServerWorker)

    print('Starting LRA control thread')
    lraControlThread = startThread(lraControlWorker)

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
