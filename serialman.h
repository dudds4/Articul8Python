#ifndef SERIALMAN_H
#define SERIALMAN_H

#include "quickqueue.h"
#include "serial/serial.h"
#include <iostream>
#include "periodic.h"
#include "circular_buffer.h"
#include <string>
#include <mutex>
#include <chrono>

#define LINBUFSIZE 1024

using serial::Serial;

typedef std::vector<uint8_t> Msg;

#define GUARD(x) std::lock_guard lk(x)

struct SerialMan : Periodic<SerialMan>
{
	SerialMan() 
	{
		serial = new Serial();
	}

	SerialMan(std::string port, int baud) 
	{
		serial = new Serial(port, baud);
	}

	void setPort(std::string port, int baud)
	{
		serial->setPort(port);
		serial->setBaudrate(baud);
	}

	bool isOpen()  const { GUARD(myMutex); return serial->isOpen(); }
	
	void open() { 
		GUARD(myMutex); 
		serial->open();
		if(serial->isOpen())
			wakeCV.notify_all();
	}
	
	void close() { GUARD(myMutex); serial->close(); std::cout << "closed\n"; }

	void clear() {
		GUARD(myMutex);
		serial->flushInput();
		serial->flushOutput();
	}

    bool wakeFromLongSleep() { return serial->isOpen(); }
    bool goToLongSleep() { return !serial->isOpen(); }

    void handleParseResult(int result)
    {
		switch(result) {
			case BUFFER_SUCCESS:
		      // Populate buffer with first complete BT packet
		      if (cb.readPacket(lastpacket))
		      {
		      	packetId++;
		      	
                static auto lastReceived = std::chrono::system_clock::now();
                auto received = std::chrono::system_clock::now();

		        auto diff = std::chrono::duration_cast<std::chrono::milliseconds> 
		                            (received - lastReceived);

	            lastReceived = received;

                static double period = 1000;
                const double alpha = 0.95;

                period = period*alpha + (1-alpha)*diff.count();
                double freq = 1000.0 / period;
                
                static int printCount = 0;
                if(printCount++ > freq)
                {
	                std::cout << "Msrd Stream Freq: " << freq << std::endl;
                	printCount = 0;
                }

		      	// do something more
		        // std::cout << "Received BT packet " << packetId << "\n";
		      }
		      break;

		    case BUFFER_FAILED_CHECKSUM:
		    	// std::cout << "Bt failed checksum\n";
		    	break;    			    	
		}

    }

    void checkIncomingSerial()
    {
    	size_t nb = serial->available();
    	// std::cout << "checking bytes available " << nb << "\n";

    	while(nb)
    	{
        	nb = std::min(nb, (size_t)LINBUFSIZE-1);
	    	size_t nRead = serial->read(linbuf, nb);

	    	// linbuf[nRead] = 0;
	    	// std::string received = (char*)linbuf;
			// std::cout << "received msg: " << received << std::endl;

	    	// bool clear = false;
	    	if(cb.getSpace() < nRead) 
	    	{ 
	    		std::cout << "Overwriting circ buffer\n";
	    		// clear = true;
	    	}

    		cb.write(linbuf, nRead);
    		int result = cb.findPacket();
    		handleParseResult(result);

    		// conditionally clear the circ buffer
    		// if(clear) { cb.read(linbuf, cb.getSize()); }

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

	unsigned long getLastPacketId() const { return packetId; }

	void getLastPacket(uint8_t* dst, uint16_t max)  const
	{
		uint16_t nb = std::min(max, (uint16_t)PACKET_SIZE);
		memcpy(dst, lastpacket, nb);
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

	unsigned long packetId = 0;

	uint8_t lastpacket[PACKET_SIZE];
	QuickQueue<Msg> outgoing;
	mutable std::mutex myMutex;
};

#endif