#include "wrapper.h"
#include "serialman.h"
#include <thread>
#include <atomic>
#include <iostream>

SerialMan *ser = nullptr;
std::thread *serThread = nullptr;

#define BAUD_RATE 9600

void ser_setup() {
	if(!ser)
		ser = new SerialMan();

	serThread = new std::thread(&SerialMan::runTask, ser);
}

void ser_cleanup() {
	std::cout << "In C++ Cleanup\n";
	ser_close();
	
	if(ser)
		ser->quitPeriodicTask();

	if(serThread)
	{
		serThread->join();
		delete serThread;
	}

	serThread = nullptr;

	if(ser) delete ser;
	ser = nullptr;
}

void ser_open(const char* portName, int n) {
	char* tmp[n+1];
	
	memcpy(tmp, portName, n);
	tmp[n] = 0;
	std::string s = (const char*) tmp;

	if(ser)
	{
		if(ser->isOpen())
			ser->close();

		try {
			ser->setPort(portName, BAUD_RATE);
			ser->open();
			ser->clear();			
		} catch (const std::exception &e) {
			std::cerr << e.what() << std::endl;
		}
	}	
}

uint8_t ser_isOpen() {
	if(ser)
		return ser->isOpen();

	return false;
}

void ser_close() { 
	if(ser && ser->isOpen())
		ser->close();
}

std::vector<uint8_t> static_msg;
uint8_t ser_send(const uint8_t* msg, unsigned n) { 
	if(!msg) return 1;

	// copy the data in
	static_msg.resize(n);
	uint8_t* dst = static_msg.data();
	memcpy(dst, msg, n);

	// queue the message
	if(ser && ser->isOpen())
		ser->sendMsg(static_msg);

	return 0;
}

std::atomic<unsigned long> lastId = 0;

uint8_t ser_newPacketAvailable()
{
	// id of 0 means we haven't received a packet yet
	unsigned long id = ser->getLastPacketId();
	uint8_t result = (id != lastId);

	return result;
}

uint8_t packet[PACKET_SIZE];
uint8_t* ser_getLastPacket()
{
	unsigned long id = ser->getLastPacketId();
	ser->getLastPacket(packet, PACKET_SIZE);
	lastId = id;
	return packet;
}
