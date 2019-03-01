#ifndef SERIALMAN_H
#define SERIALMAN_H

#include "quickqueue.h"
#include "serial/serial.h"
#include <iostream>
#include "periodic.h"
#include "circular_buffer.h"
#include <string>
#include <mutex>

#define LINBUFSIZE 1024

using serial::Serial;

typedef std::vector<uint8_t> Msg;

#define GUARD(x) std::lock_guard lk(x)

struct SerialMan : Periodic<SerialMan>
{
	SerialMan() = delete;
	SerialMan(std::string port, int baud) 
	{
		serial = new Serial(port, baud);
		this->taskPeriod = 10;
	}

	void open() { GUARD(myMutex); serial->open(); }
	void close() { GUARD(myMutex); serial->close(); std::cout << "closed\n"; }

    bool wakeFromLongSleep() { return serial->isOpen(); }
    bool goToLongSleep() { return !serial->isOpen(); }

    void handleParseResult(int result)
    {
		switch(result) {
			case BUFFER_SUCCESS:
		      // Populate buffer with first complete BT packet
		      if (cb.readPacket(lastpacket))
		      {
		      	// do something more
		        std::cout << "Received BT packet\n";
		      }
		      break;

		    case BUFFER_FAILED_CHECKSUM:
		    	std::cout << "Bt failed checksum\n";
		    	break;    			    	
		}

    }

    void checkIncomingSerial()
    {
    	size_t nb = serial->available();
    	std::cout << "checking bytes available " << nb << "\n";

    	while(nb)
    	{
			std::cout << "received msg" << std::endl;

        	nb = std::min(nb, (size_t)LINBUFSIZE);
	    	size_t nRead = serial->read(linbuf, nb);

	    	if(cb.getSpace() < nRead) { std::cout << "Overwriting circ buffer\n"; }

    		cb.write(linbuf, nRead);


    		int result = cb.findPacket();
    		handleParseResult(result);
    		nb = serial->available();
    	}    	
    }

	void checkOutgoingSerial()
	{
		GUARD(myMutex);

		const int MAX_AT_ONCE = 3;

		int i = 0;
		while(i < MAX_AT_ONCE && outgoing.size())
		{
			Msg m = outgoing.getNext();
			serial->write(m);
		}
	}

	void sendMsg(const Msg& m) 
	{
		GUARD(myMutex);
		outgoing.push_back(m);
	}

    void periodicTask() 
    {
    	checkIncomingSerial();
    	checkOutgoingSerial();
    }

private:
	Serial* serial;
	CircularBuffer<2*LINBUFSIZE> cb;
	uint8_t linbuf[LINBUFSIZE];
	uint8_t lastpacket[PACKET_SIZE];
	QuickQueue<Msg> outgoing;
	std::mutex myMutex;
};

#endif