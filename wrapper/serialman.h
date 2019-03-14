#ifndef SERIALMAN_H
#define SERIALMAN_H

#include "recording_man.h"
#include "logger.h"
#include "quickqueue.h"
#include "serial/serial.h"
#include <iostream>
#include "periodic.h"
#include "circular_buffer.h"
#include <string>
#include <mutex>
#include <chrono>
#include "leg_state.h"
#include "helper_3dmath.h"
#include "lra_packet_gen.h"

#define LINBUFSIZE 64

using serial::Serial;

typedef std::vector<uint8_t> Msg;

#define GUARD(x) std::lock_guard lk(x)

#define LIKELY(condition) __builtin_expect(static_cast<bool>(condition), 1)

struct SerialMan : Periodic<SerialMan>
{
	SerialMan() 
	{
		serial = new Serial();
	}

	SerialMan(std::string port, int baud, int sid) 
	{
		serial = new Serial(port, baud);
		serial_id = sid;
	}

	void setPort(std::string port, int baud, int sid, int nLras)
	{
		serial->setPort(port);
		serial->setBaudrate(baud);
		setBand(sid, nLras);
	}

	void setRecordingMan(RecordingMan* recMan) {
		recordingMan = recMan;
	}

	int m_nLras = 0;
	bool m_exercising = false;
	LraPacketGenerator lraPacketGenerator;

	// void setExercising(bool exercising) { m_exercising = exercising; }
	
	void setBand(int idx, int nLras) 
	{ 
		serial_id = idx;
		m_nLras = nLras; 
		lraPacketGenerator = LraPacketGenerator(nLras); 
	}

	void setLogging(bool log) {
		if(logging == log) return;
		logging = log;
		
		if(logging)
		{
			std::string portName = serial->getPort();
			std::string tag;
			const int MAX_L = 10;
			if(portName.length() < MAX_L)
				tag = portName;
			else
				tag = portName.substr(portName.length()-MAX_L, portName.length());

			logWrite.openLog(tag);
		}
		else
			logWrite.closeLog();
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
		      	// predict yes plz
		      	if(LIKELY(lastpacket[POS_TYPE] == IMU_DATA))
		      	{
		      		recordingMan->receivedQuat((float*)&lastpacket[POS_DATA], serial_id);
		      		
		      		if(!recording)
		      		{
		      			float asdf[6];

		      			// LegState l(asdf);

			      		lraPacketGenerator.generatePacket(
			      			hotpathPacket,
			      			// l,
			      			recordingMan->getLatestStateDiff(),
			      			serial_id);

			      		if(m_exercising)
			      		{
			      			// std::cout << "sending packet" << std::endl;

				        	GUARD(myMutex);
				        	serial->write(hotpathPacket, PACKET_SIZE);

				        	// for(int j = 0; j < PACKET_SIZE; ++j)
				        	// 	std::cout << (int)hotpathPacket[j] << " ";

				        	// std::cout << std::endl;
				        	
			      		}
		      		}
		      	}

     //            if(lastpacket[POS_TYPE] == 10)
     //            {
					// GUARD(myMutex);
					// serial->write(lastpacket, PACKET_SIZE);
     //            	std::cout << "Received a test packet\n";

     //            	for(int i = 0; i < PACKET_SIZE; ++i)
     //            		std::cout << (int) lastpacket[i] << " ";
     //            	std::cout << std::endl;
     //            }

		      	packetId++;
	      		computeFrequency();

                if(logging)
                	logWrite.writePacket(lastpacket, PACKET_SIZE);

		      	// do something more
		        // std::cout << "Received BT packet " << packetId << "\n";

                // this is just for round trip testing
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
	    		static int printC = 0;
	    		if(printC++ > 50)
	    		{
		    		std::cout << "Overwriting circ buffer (50th time)\n";
		    		printC = 0;
	    		}
	    		// clear = true;
	    	}

    		cb.write(linbuf, nRead);
    		int result;

    		do {
			
			 	result = cb.findPacket();
    			handleParseResult(result);
    		
    		} while(result == BUFFER_SUCCESS);

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

    double getFrequency()
    {
    	return 1000.0 / period;
    }

    bool startRecording() {
    	if (!m_exercising) {
    		recording = true;
    	}
    	return recording;
    }

    bool stopRecording() {
    	recording = false;
    	return !recording;
    }

    bool startExercise() {
    	if (!recording) {
    		m_exercising = true;
    	}

    	return m_exercising;
    }

    bool stopExercise() {
    	m_exercising = false;
    	return !m_exercising;
    }

private:
	Serial* serial;
	CircularBuffer<2*LINBUFSIZE> cb;
	uint8_t linbuf[LINBUFSIZE];

	unsigned long packetId = 0;

	uint8_t lastpacket[PACKET_SIZE];
	uint8_t hotpathPacket[PACKET_SIZE];
	QuickQueue<Msg> outgoing;
	mutable std::mutex myMutex;
	bool logging = false;
	LogWriter logWrite;

	double period = 1000;
	int printCount = 0;

	RecordingMan* recordingMan;
	int serial_id = -1;
	bool recording = false;

	std::chrono::time_point<std::chrono::system_clock> lastReceived = std::chrono::system_clock::now();      

	void computeFrequency()
	{
        auto received = std::chrono::system_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds> 
                            (received - lastReceived);

        lastReceived = received;

        const double alpha = 0.95;

        period = period*alpha + (1-alpha)*diff.count();
        double freq = 1000.0 / period;
        
        if(printCount++ > freq*2)
        {
            std::cout << "Msrd Stream Freq: " << freq << std::endl;
        	printCount = 0;
        }		
	}          
};



#endif