#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>

struct LogWriter
{
	~LogWriter();

	void writePacket(const void* packet, unsigned nb);
	void openLog(std::string tag="");
	void closeLog();
	std::string getFilename();

private:
	std::ofstream fout;
	std::string m_filename;
	void writeHeader();
};

struct LogReader
{
	~LogReader();

	void readPacket(void* data, unsigned nb);
	bool isOpen();
	bool isDone();
	void openLog(const char* filename);
	void closeLog();

private:
	std::ifstream fin;
	unsigned packetSize = 0;
	void readHeader();
};

#endif