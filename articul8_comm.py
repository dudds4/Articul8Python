
from msg_defs import *
from ctypes import *

import os
import sys
import platform

global articul8
initialized = False

portNicknames = ["thigh", "shank"]

numLRAs = [8, 6]
ports = ["COM6", "COM5"]
if (platform.system().lower() == 'darwin'):
    ports = ["/dev/cu.Articul8Board3-SerialPo", "/dev/cu.Articul8Board1-SerialPo"]

# Opens the given serial port at the given index and looks for devices
def ser_open(port, portId=0):
	global articul8
	if(portId == None):
		portId = 0

	try:
		articul8.ser_open( port.encode('utf-8'), len(port), portId)
	except Exception as ex:
		print(ex)

# Returns a boolean that indicates whether the port is open
def ser_isOpen(portId=0):
	global articul8

	if(articul8.ser_isOpen(portId) == 0):
		return False
	else:
		return True

def ser_close(portId=0):
	global articul8
	articul8.ser_close(portId)

def ser_write(msg, portId=0):
	global articul8	
	# articul8.ser_send(msg.encode('utf-8'), len(msg))
	articul8.ser_send(msg, len(msg), portId)

def ser_getLastPacket(portId=0):
	global articul8
	flag = articul8.ser_newPacketAvailable(portId)

	c_ubyte_p = POINTER(c_ubyte)

	if flag != 0:
		p = articul8.ser_getLastPacket(portId)

		result = struct.pack('B', cast(p, c_ubyte_p).contents.value)
		for i in range(1, PACKET_SIZE):
			byte = cast(p+i, c_ubyte_p).contents.value
			result += struct.pack('B', byte)

	else:
		result = None

	return result

def ser_startLogging(portId=0):
	global articul8
	articul8.ser_startLogging(portId)

def ser_stopLogging(portId=0):
	global articul8
	articul8.ser_stopLogging(portId)

def ser_openLog(filename, portId=0):
	global articul8
	articul8.ser_openLog(filename.encode('utf-8'), portId)

def ser_getLogPacket(portId=0):
	global articul8
	flag = articul8.ser_logPacketAvailable(portId)

	c_ubyte_p = POINTER(c_ubyte)

	if flag != 0:
		p = articul8.ser_getLogPacket(portId)

		result = struct.pack('B', cast(p, c_ubyte_p).contents.value)
		for i in range(1, PACKET_SIZE):
			byte = cast(p+i, c_ubyte_p).contents.value
			result += struct.pack('B', byte)

	else:
		result = None

	return result	

def ser_getFrequency(portId):
	global articul8
	return articul8.ser_getFrequency(portId)


# Loads the library from the c lib
def loadCSerial():
	global articul8

	loadSucceeded  = False
	# is_64bits = sys.maxsize > 2**32
	sysOS = platform.system().lower()

	dir_path = os.path.dirname(os.path.realpath(__file__))
	lpath_base = dir_path + '/build'

	if(sysOS == 'darwin'):
		librarypath = lpath_base + "/libarticul8.dylib"
	else:
		librarypath = lpath_base + "/libarticul8.dll"
	# 	if(is_64bits):
	# 		librarypath = lpath_base + '/win64/libfx_plan_stack.dll'
	# 	else:
	# 		librarypath = lpath_base + '/win/libfx_plan_stack.dll'
	# else:
	# 	if(is_64bits):
	# 		librarypath = lpath_base + '/unix64/libfx_plan_stack.so'
	# 	else:
	# 		librarypath = lpath_base + '/unix/libfx_plan_stack.so'	

	try:
		print("loading... " + librarypath)
		articul8 = cdll.LoadLibrary(librarypath)
	except OSError as arg:
		print( "\n\nThere was a problem loading the library\n {0}\n".format(arg))
	else:
		loadSucceeded  = True

	if(loadSucceeded  != True):
		return False

	print("Loaded!")
	initialized = True
	articul8.ser_setup()

	# set arg types
	articul8.ser_open.argtypes = [c_char_p, c_int, c_uint]
	articul8.ser_send.argtypes = [c_char_p, c_int, c_uint]
	articul8.ser_getLastPacket.restype = c_void_p
	articul8.ser_getLogPacket.restype = c_void_p
	articul8.ser_getFrequency.restype = c_double

	return True

def ser_cleanup():
	global articul8

	for portId in range(0,4):
		if(ser_isOpen(portId)):
			ser_close(portId)

	articul8.ser_cleanup()

def list_to_int_arr(l):
	c_arr = (c_int * len(l))(*l)
	c_len = c_int(len(l))
	return c_arr, c_len

def list_to_bool_arr(l):
	c_arr = (c_bool * len(l))(*l)
	c_len = c_int(len(l))
	return c_arr, c_len
