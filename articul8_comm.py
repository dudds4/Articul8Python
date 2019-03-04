
from msg_defs import *
from ctypes import *

import os
import sys
import platform

global articul8
initialized = False

# Opens the given serial port at the given index and looks for devices
def ser_open(port):
	global articul8
	try:
		articul8.ser_open( port.encode('utf-8'), len(port) )
	except Exception as ex:
		print(ex)

# Returns a boolean that indicates whether the port is open
def ser_isOpen():
	global articul8
	if(articul8.ser_isOpen() == 0):
		return False
	else:
		return True

def ser_close():
	global articul8
	articul8.ser_close()

def ser_write(msg):
	global articul8	
	# articul8.ser_send(msg.encode('utf-8'), len(msg))
	articul8.ser_send(msg, len(msg))

def ser_getLastPacket():
	global articul8
	flag = articul8.ser_newPacketAvailable()

	c_ubyte_p = POINTER(c_ubyte)

	if flag != 0:
		p = articul8.ser_getLastPacket()

		result = struct.pack('B', cast(p, c_ubyte_p).contents.value)
		for i in range(1, PACKET_SIZE):
			byte = cast(p+i, c_ubyte_p).contents.value
			result += struct.pack('B', byte)

	else:
		result = None

	return result

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
	articul8.ser_open.argtypes = [c_char_p, c_int]
	articul8.ser_send.argtypes = [c_char_p, c_int]
	# articul8.ser_getLastPacket.restype = POINTER(c_char)
	articul8.ser_getLastPacket.restype = c_void_p

	return True

def ser_cleanup():
	global articul8

	if(ser_isOpen()):
		ser_close()

	articul8.ser_cleanup()

def list_to_int_arr(l):
	c_arr = (c_int * len(l))(*l)
	c_len = c_int(len(l))
	return c_arr, c_len

def list_to_bool_arr(l):
	c_arr = (c_bool * len(l))(*l)
	c_len = c_int(len(l))
	return c_arr, c_len
