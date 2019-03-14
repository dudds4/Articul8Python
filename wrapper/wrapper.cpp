#include "wrapper.h"
#include "serialman.h"
#include "recording_man.h"
#include "logger.h"
#include <thread>
#include <atomic>
#include <iostream>

#define NUM_PORTS 2

SerialMan *ser[NUM_PORTS] = {nullptr};
std::thread *serThread[NUM_PORTS] = {nullptr};
LogReader logReader[NUM_PORTS];
RecordingMan recording_man;

#define BAUD_RATE 9600

void ser_setup() {

	for(int port = 0; port < NUM_PORTS; ++port)
	{
		if(!ser[port])
			ser[port] = new SerialMan();

		if(!serThread[port])
			serThread[port] = new std::thread(&SerialMan::runTask, ser[port]);		
	}
}

void ser_cleanup() {

	std::cout << "In C++ Cleanup\n";

	for(int port = 0; port < NUM_PORTS; ++port)
	{
		ser_close(port);
		
		if(ser[port])
			ser[port]->quitPeriodicTask();

		if(serThread[port])
		{
			serThread[port]->join();
			delete serThread[port];
		}

		serThread[port] = nullptr;

		if(ser[port]) delete ser[port];
		ser[port] = nullptr;
	}
}

void ser_open(const char* portName, int n, unsigned port, int nLras) {
	if(port >= NUM_PORTS) 
		return;

	char* tmp[n+1];
	
	memcpy(tmp, portName, n);
	tmp[n] = 0;
	std::string s = (const char*) tmp;

	if(ser[port])
	{
		if(ser[port]->isOpen())
			ser[port]->close();

		try {
			ser[port]->setPort(portName, BAUD_RATE, port, nLras);
			ser[port]->setRecordingMan(&recording_man);
			ser[port]->open();
			ser[port]->clear();			
		} catch (const std::exception &e) {
			std::cerr << e.what() << std::endl;
		}
	}	
}

uint8_t ser_isOpen(unsigned port) {
	if(port >= NUM_PORTS) 
		return false;

	if(ser[port])
		return ser[port]->isOpen();

	return false;
}

void ser_close(unsigned port) { 
	if(port >= NUM_PORTS) 
			return;

	if(ser[port] && ser[port]->isOpen())
		ser[port]->close();
}

std::vector<uint8_t> static_msg;
uint8_t ser_send(const uint8_t* msg, unsigned n, unsigned port) {
	if(port >= NUM_PORTS) 
			return -1;

	if(!msg) 
		return 1;

	// copy the data in
	static_msg.resize(n);
	uint8_t* dst = static_msg.data();
	memcpy(dst, msg, n);

	// queue the message
	if(ser[port] && ser[port]->isOpen())
		ser[port]->sendMsg(static_msg);

	return 0;
}

std::atomic<unsigned long> lastId = 0;

uint8_t ser_newPacketAvailable(unsigned port)
{
	if(port >= NUM_PORTS) 
		return 0;

	// id of 0 means we haven't received a packet yet
	unsigned long id = ser[port]->getLastPacketId();
	uint8_t isAvailable = (id != lastId);

	return isAvailable;
}

uint8_t packet[PACKET_SIZE];
uint8_t lraPacket[PACKET_SIZE];

uint8_t* ser_getLastPacket(unsigned port)
{
	if(port >= NUM_PORTS) 
			return packet;

	unsigned long id = ser[port]->getLastPacketId();
	ser[port]->getLastPacket(packet, PACKET_SIZE);
	lastId = id;
	return packet;
}

uint8_t* ser_getLraPacket(unsigned port)
{
	if(port >= NUM_PORTS)
			return lraPacket;

	ser[port]->getLraPacket(lraPacket, PACKET_SIZE);
	return lraPacket;
}

void ser_startLogging(unsigned port)
{
	if(port >= NUM_PORTS) 
			return;

	std::cout << "Starting logging for port " << port << "\n";
	ser[port]->setLogging(true);
}

void ser_stopLogging(unsigned port)
{
	if(port >= NUM_PORTS) 
			return;

	ser[port]->setLogging(false);
}

void ser_openLog(const char* filename, unsigned port)
{
	if(port >= NUM_PORTS) 
			return;

	logReader[port].openLog(filename);
}

uint8_t ser_logPacketAvailable(unsigned port)
{
	if(port >= NUM_PORTS) 
			return false;

	return logReader[port].isOpen() && !logReader[port].isDone();
}

uint8_t* ser_getLogPacket(unsigned port)
{
	if(port >= NUM_PORTS) 
			return packet;

	logReader[port].readPacket(packet, PACKET_SIZE);
	return packet;
}

double ser_getFrequency(unsigned port)
{
	if(port >= NUM_PORTS) 
			return -1;

	return ser[port]->getFrequency();
}

bool ser_startRecording()
{
	if (recording_man.m_recording || recording_man.m_exercising) {
		return false;
	}
	recording_man.startRecording();
	for (int i = 0; i < NUM_PORTS; i++) {
		ser[i]->startRecording();
	}
	return recording_man.m_recording;
}

bool ser_stopRecording()
{
	for (int i = 0; i < NUM_PORTS; i++) {
		ser[i]->stopRecording();
	}
	recording_man.stopRecording();

	return !recording_man.m_recording;
}

bool ser_startExercise()
{
	if (recording_man.m_recording || recording_man.m_exercising) {
		return false;
	}
	recording_man.startExercise();
	for (int i = 0; i < NUM_PORTS; i++) {
		ser[i]->startExercise();
	}
	return recording_man.m_exercising;
}

bool ser_stopExercise()
{
	for (int i = 0; i < NUM_PORTS; i++) {
		ser[i]->stopExercise();
	}
	recording_man.stopExercise();

	return !recording_man.m_exercising;
}