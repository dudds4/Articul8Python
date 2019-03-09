from articul8_comm import *
from msg_defs import *
from workers import *
from threadman import *

import sys
import time
import string
import signal
import platform
import os
import glob

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

    # open the latest two log files
    files = glob.glob("*.log")
    files.sort(key=os.path.getmtime)
    files = files[::-1]
    nFiles = 2
    files = files[0:nFiles]
    print(files)

    for i in range(0, nFiles):
        ser_openLog(files[i], i)

    failed = False
    while(not failed):
        for i in range(0, nFiles):
            packet = ser_getLogPacket(i)
            print("device {}: {}".format(i, packet))
            failed = failed or packet == None

def main():
    loadCSerial()
    signal.signal(signal.SIGINT, signal_handler)

    # # uncomment to see example of log reading...    
    # time.sleep(0.5)
    # exampleReadLog()
    # return

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
        else:
            ser_startLogging(i)

    if(failed):
        print("Coundn't open all ports...")
        ser_cleanup()
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
