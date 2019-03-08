#include "logger.h"
#include "msg_defs.h"

#include <ctime>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>

void fillBufferNoEscape(uint8_t* buf, unsigned nb)
{
	// picking a range that has (i think)
	// no special characters
	for(int i = 0; i < nb; ++i)
			buf[i] = rand() % 20 + 100;	
}

void fillBuffer(uint8_t* buf, unsigned nb)
{

	for(int i = 0; i < nb; ++i)
			buf[i] = rand() % 127;	
}

bool fileSize()
{
	LogWriter w;
	w.openLog();
	auto fname = w.getFilename();

	const int NPACKETS=10;
	uint8_t dataOut[NPACKETS][PACKET_SIZE];
	fillBufferNoEscape((uint8_t*)dataOut, NPACKETS*PACKET_SIZE);

	for(int i = 0; i < NPACKETS; ++i)
		w.writePacket((uint8_t*)&dataOut[i][0], PACKET_SIZE);

	w.closeLog();

	std::ifstream file(fname.c_str(), std::ios::binary | std::ios::ate);
	unsigned fileSize = file.tellg();
	unsigned expectedSize = NPACKETS*PACKET_SIZE + sizeof(unsigned)*2; 
	if(fileSize != expectedSize) 
	{
		std::cout 	<< "expected size: " << expectedSize 
					<< ", actual: " << fileSize << std::endl;

		return false;
	}
	return true;
}

bool fileConsistency()
{
	LogWriter w;
	LogReader r;

	w.openLog();
	std::string filename = w.getFilename();

	const int NPACKETS = 10;
	unsigned char dataOut[NPACKETS][PACKET_SIZE];
	unsigned char dataIn[NPACKETS][PACKET_SIZE];

	// generate random data
	fillBuffer((uint8_t*)dataOut, NPACKETS*PACKET_SIZE);

	// write out
	for(int i = 0; i < NPACKETS; ++i)
		w.writePacket((uint8_t*)&dataOut[i][0], PACKET_SIZE);

	w.closeLog();

	r.openLog(filename.c_str());

	if(!r.isOpen())
	{
		std::cout << "couldnt open file\n";
		return false;
	}

	// read in
	int i;
	for(i = 0; i < NPACKETS && !r.isDone(); ++i)
		r.readPacket((uint8_t*)&dataIn[i][0], PACKET_SIZE);

	if(i != NPACKETS) 
	{
			std::cout << "wrong num packets " << i << "\n";	
			return false;
	}

	// compare data
	unsigned nb = NPACKETS*PACKET_SIZE*sizeof(unsigned char);
	bool same =  memcmp(dataOut, dataIn, nb) == 0;
	if(!same)
		std::cout << "similarity failed\n";

	return same;
}

bool fileCreation()
{
	LogWriter w;
	w.openLog();
	std::string filename = w.getFilename();

	w.closeLog();

	LogReader r;
	r.openLog(filename.c_str());

	bool worked = r.isOpen();
	r.closeLog();

	return worked;
}


int main()
{
	srand(time(NULL));

	std::cout << (fileCreation() ? "passed" : "failed") << " file creation\n";
	std::cout << (fileSize() ? "passed" : "failed") << " file size\n";
	std::cout << (fileConsistency() ? "passed" : "failed") << " file consistency\n";

	return 0;

}