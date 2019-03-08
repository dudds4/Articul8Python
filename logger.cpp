#include "logger.h"

#include <sstream>
#include <ctime>
#include <chrono>
#include <algorithm>
#include "msg_defs.h"
#include <iostream>

std::string generateFileName();

#define ASSERT_OPEN(f) do { if(!f.is_open()) return; } while(0)

void LogWriter::writePacket(const void* p, unsigned nb)
{
	const char* packet = static_cast<const char*>(p);
	ASSERT_OPEN(fout);
	nb = std::min(nb, (unsigned)PACKET_SIZE);
	int padding = PACKET_SIZE - nb;

	fout.write(packet, nb);

	if(padding)
	{
		char zeros[padding] = {0};
		fout.write(zeros, padding);		
	}
}


std::string LogWriter::getFilename()
{
	return m_filename;
}

void LogWriter::writeHeader()
{
	ASSERT_OPEN(fout);

	unsigned fileLength = 0;
	unsigned packetSize = PACKET_SIZE;
	fout.write((char*)&fileLength, sizeof(unsigned));
	fout.write((char*)&packetSize, sizeof(unsigned));
}

void LogWriter::openLog()
{
	closeLog();
	std::string fname = generateFileName();
	fout.open(generateFileName(), std::ios::binary);
	if(fout.is_open())
	{
		m_filename = fname;
		writeHeader();	
	}
}

void LogWriter::closeLog()
{
	if(fout.is_open())
		fout.close();

	m_filename = "";
}

void LogReader::readPacket(void* d, unsigned nb)
{
	char* data = static_cast<char*>(d);
	ASSERT_OPEN(fin);
	nb = std::min(nb, packetSize);

	fin.read(data, nb);
	// for(int i = 0; i < nb; ++i)
	// 	fin >> data[i];

	// skip any bytes for this packet that weren't read
	int padding = packetSize - nb;

	// std::cout << "packet size: " << packetSize << ", padding: " << padding << std::endl;
	if(padding > 0)
	{
		char grbg[padding];

		fin.read(grbg, padding);
		// for(int i = 0; i < padding; ++i)
		// 	fin >> grbg[i];
	}
}

bool LogReader::isOpen()
{
	return fin.is_open();
}


bool LogReader::isDone()
{
	// return (!(fin));
	return fin.is_open() && fin.eof();
}

void LogReader::readHeader()
{
	ASSERT_OPEN(fin);

	unsigned fileLength;

	fin.read((char*)&fileLength, sizeof(unsigned));
	fin.read((char*)&packetSize, sizeof(unsigned));
}

void LogReader::openLog(const char* filename)
{
	closeLog();
	fin.open(filename, std::ios::binary);
	readHeader();
}

void LogReader::closeLog()
{		
	if(fin.is_open())
		fin.close();

	packetSize = 0;
}

std::string generateFileName()
{
	std::stringstream ss;

	auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

	std::string cNow = std::ctime(&now);
	std::string filetime = cNow.substr(4,4+15);

	if(filetime[4] == ' ')
		filetime[4] = '0';

	std::replace(filetime.begin(), filetime.end(), ' ', '_');
	std::replace(filetime.begin(), filetime.end(), ':', '.');

	ss << "articul8Log." << filetime << ".csv";
	return ss.str();
}
