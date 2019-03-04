
import threading

threadFlag = True
threadFlagLock = threading.Lock()
threadCount = 0

def checkThreadCount():
    global threadCount, threadFlagLock
    threadFlagLock.acquire()
    x = threadCount
    threadFlagLock.release()
    return x

def startThread(x):
    global threadCount, threadFlagLock
    thread = threading.Thread(target=x)
    thread.start()

    threadFlagLock.acquire()
    threadCount+=1
    threadFlagLock.release()

    return thread    

def threadQuit():
    global threadCount, threadFlagLock
    threadFlagLock.acquire()
    threadCount -= 1
    threadFlagLock.release()

def setThreadFlag(x):
    global threadFlag, threadFlagLock
    threadFlagLock.acquire()
    threadFlag = x
    threadFlagLock.release()


def checkThreadFlag():
    global threadFlag, threadFlagLock
    threadFlagLock.acquire()
    x = threadFlag
    threadFlagLock.release()
    return x
