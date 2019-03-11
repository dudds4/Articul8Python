from articul8_comm import *
from articul8_logs import *
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
import matplotlib.pyplot as plt

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

    if (len(sys.argv) > 1):
        nFiles = -1

        try:
            nFiles = int(sys.argv[1])
        except:
            pass

        if(nFiles > 0):
            exampleReadLog(nFiles)
            return

        try:
            plotLog(sys.argv[1])
        except:
            pass
        
        return

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

    print('Starting LRA control thread')
    lraControlThread = startThread(lraControlWorker)

    portIdxs = [i for i in range(len(ports))]
    while(checkThreadFlag() and len(portIdxs)):

        msg = None
        while(checkThreadFlag() and (msg is None or ser_getFrequency(portIdxs[0]) < 30)):
            time.sleep(0.5)
            msg = ser_getLastPacket(portIdxs[0])

        if(checkThreadFlag()):
            ser_startLogging(portIdxs[0])
            portIdxs = portIdxs[1:]

    while(checkThreadCount() > 0):
        time.sleep(1)

if __name__ == '__main__':
    main()
