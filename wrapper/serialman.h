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

#define LINBUFSIZE 64

using serial::Serial;

typedef std::vector<uint8_t> Msg;

#define GUARD(x) std::lock_guard lk(x)


#define IS_GREATEST(a, b, c) ( (a) > (b) && (a) > (c) )
#define PI 3.14159265f

struct LraPacketGenerator
{
	LraPacketGenerator(int nLras=0) {
		NUM_LRAS = nLras;
		bounds = new float[NUM_LRAS];

		float step = 2 * PI / NUM_LRAS;
		for(int i = 0; i < NUM_LRAS; ++i)
			bounds[i] = step*i;
	}

	void interpolateAngle(float angle, float mag, uint8_t* intensities)
	{
		int lowerInd, upperInd = 1;
		while(angle > bounds[upperInd] && upperInd < NUM_LRAS)
			upperInd++;

		lowerInd = upperInd - 1;
		if(upperInd == NUM_LRAS) upperInd = 0;

		if(lowerInd < 0 || lowerInd >= NUM_LRAS || upperInd < 0 || upperInd >= NUM_LRAS)
			return;

		for(int i = 0; i < NUM_LRAS; ++i)
			intensities[i] = 0;

		const float step = 2*PI/NUM_LRAS;
		float lowerDist = angle - bounds[lowerInd];
		float upperDist = step - lowerDist;
		float s = lowerDist + upperDist;

		intensities[lowerInd] = mag * upperDist / s;
		intensities[upperInd] = mag * lowerDist / s;		
	}

	void lraRotatePacket(uint8_t* packet, int intensity)
	{
		packet[0] = SOP;
		packet[1] = LRA_CONTROL;
		packet[2] = LRA_SPIN;
		packet[3] = intensity;
		
		memset(packet + 4, 0, 16);

		packet[POS_CHECKSUM] = LRA_CONTROL + LRA_SPIN + intensity;
	}

	void lraRawPacket(uint8_t* packet, float angle, float mag)
	{
		packet[0] = SOP;
		packet[1] = LRA_CONTROL;
		packet[2] = LRA_NO_SPIN;

		interpolateAngle(angle, mag, packet+3);

		packet[POS_CHECKSUM] = 0;
		
		for(int i = 1; i < NUM_LRAS+4; ++i)
		{
			packet[POS_CHECKSUM] += packet[i];
		}
	}

void generateLraPacket(uint8_t* packet, const std::vector<LegState>& motion, unsigned idx, const LegState& current, int boardIdx)
{
	LegState diff = current.getDiff(motion.at(idx));
	    
	uint8_t intensities[NUM_LRAS];

	float yd, rd, pd;

	if(boardIdx == 0)
	{
		rd = (diff.rpyAngles[0]);
		pd = (diff.rpyAngles[1]);
		yd = (diff.rpyAngles[2]);
	}
	else
	{
		rd = (diff.rpyAngles[0]);
		pd = (diff.rpyAngles[1]);		
		yd = (diff.rpyAngles[2]);
	}
    
    if(IS_GREATEST(abs(yd), abs(rd), abs(pd)))
    {
    	float mag = 6 * abs(yd) * 180 / PI;
    	float angle = yd > 0 ? PI / 2 : 3*PI/2;
    	lraRawPacket(packet, angle, mag);
    }
    else if(abs(pd) > abs(rd))
    {
    	float mag = 8 * abs(rd) * 180 / PI;
    	float angle = pd > 0 ? 0 : PI;
    	lraRawPacket(packet, angle, mag);
    }
    else if(abs(rd) > (12*PI/180))
    {
    	float mag = rd > 0 ? 1 : -1;
    	lraRotatePacket(packet, mag*0.8);
    }
    else
    {
    	lraRawPacket(packet, 0, 0);
    }
}

	~LraPacketGenerator() { delete[] bounds; }

	float* bounds;
	int NUM_LRAS;
};


// void interpolateAngle(float angle, float mag, uint8_t* intensities)
// {
// 	int lowerInd, upperInd = 1;
// 	while(m_angle > bounds[upperInd] && upperInd < NUM_LRAS)
// 		upperInd++;

// 	lowerInd = upperInd - 1;
// 	if(upperInd == NUM_LRAS) upperInd = 0;

// 	if(lowerInd < 0 || lowerInd >= NUM_LRAS || upperInd < 0 || upperInd >= NUM_LRAS)
// 		return;

// 	for(int i = 0; i < NUM_LRAS; ++i)
// 		intensities[i] = 0;
// 	// memset(intensities, 0, sizeof(int) * NUM_LRAS);

// 	const float step = 2*PI/NUM_LRAS;
// 	float lowerDist = m_angle - bounds[lowerInd];
// 	float upperDist = step - lowerDist;

// 	intensities[lowerInd] = m_intensity * upperDist / step;
// 	intensities[upperInd] = m_intensity * lowerDist / step;
// }


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

	void setExercising(bool exercising) { m_exercising = exercising; }
	
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
		        std::vector<LegState> motion;
		        LegState currentState = LegState({0, 0, 0, 0, 0, 0});
		        int lastStateIdx;

		        // hot path
		      	if(motion.size() && lastpacket[POS_TYPE] == IMU_DATA)
		      	{
		      		int currentStateIdx = getCurrentLegState(motion, currentState, lastStateIdx);
		      		lraPacketGenerator.generateLraPacket(hotpathPacket, motion, currentStateIdx, currentState, serial_id);
		      		
		      		if(m_exercising)
		      		{
			        	GUARD(myMutex);
			        	serial->write(hotpathPacket, PACKET_SIZE);
		      		}

			        lastStateIdx = currentStateIdx;
		      	}


		      	if (recording && lastpacket[POS_TYPE] == IMU_DATA && recordingMan != nullptr)
		      	{
		      		recordingMan->recievedQuat(Quaternion::fromImuPacket(lastpacket), serial_id);
		      	}
                if(lastpacket[POS_TYPE] == 10)
                {
					GUARD(myMutex);
					serial->write(lastpacket, PACKET_SIZE);
                	std::cout << "Received a test packet\n";

                	for(int i = 0; i < PACKET_SIZE; ++i)
                		std::cout << (int) lastpacket[i] << " ";
                	std::cout << std::endl;
                }

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
    	if (!exercising) {
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
    		exercising = true;
    	}
    	return exercising;
    }

    bool stopExercise() {
    	exercising = false;
    	return !exercising;
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
	bool exercising = false;

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