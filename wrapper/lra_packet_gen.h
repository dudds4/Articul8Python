
#ifndef LRA_PACKET_GEN
#define LRA_PACKET_GEN

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

	bool lraRotatePacket(uint8_t* packet, float intensity)
	{
		if(cacheIsSpin && intensity == cacheMag) 
			return false;

		packet[0] = SOP;
		packet[1] = LRA_CONTROL;
		packet[2] = LRA_SPIN;

		memcpy(packet + 3, &intensity, sizeof(float));		
		memset(packet + 7, 0, 16);
		
		packet[POS_CHECKSUM] = 0;		
		for(int i = 1; i < 15; ++i)
		{
			packet[POS_CHECKSUM] += packet[i];
		}

		cacheIsSpin = true;
		cacheMag = intensity;
		return true;
	}

	#define DIST_SQR_THRESHOLD 20

	bool lraRawPacket(uint8_t* packet, float angle, float mag)
	{
		if(!cacheIsSpin)
		{
			float distSqr = (mag*mag + cacheMag*cacheMag - 2*mag*cacheMag*cos(angle-cacheAngle));
			if(distSqr < DIST_SQR_THRESHOLD)
				return false;
		}
		
		packet[0] = SOP;
		packet[1] = LRA_CONTROL;
		packet[2] = LRA_NO_SPIN;

		interpolateAngle(angle, mag, packet+3);

		packet[POS_CHECKSUM] = 0;
		
		for(int i = 1; i < 15; ++i)
		{
			packet[POS_CHECKSUM] += packet[i];
		}

		cacheIsSpin = false;
		cacheMag = mag;
		cacheAngle = angle;
		return true;
	}

#define THIGH_IDX 0
bool generatePacket(uint8_t* packet, const LegState& diff, int boardIdx)
{	    
	uint8_t intensities[NUM_LRAS];

	float yd, rd, pd;

	if(boardIdx == THIGH_IDX)
	{
		rd = (diff.rpyAngles[0]);
		pd = (diff.rpyAngles[1]);
		yd = (diff.rpyAngles[2]);
	}
	else
	{
		rd = (diff.rpyAngles[3]);
		pd = (diff.rpyAngles[4]);		
		yd = (diff.rpyAngles[5]);
	}
    
    if(abs(yd) > abs(pd) && abs(yd) > abs(rd))
    {
    	float mag = 6 * abs(yd) * 180 / PI;
    	float angle = yd > 0 ? PI / 2 : 3*PI/2;
    	return lraRawPacket(packet, angle, mag);
    }
    else if(abs(pd) > abs(rd))
    {
    	float mag = 8 * abs(rd) * 180 / PI;
    	float angle = pd > 0 ? 0 : PI;
    	return lraRawPacket(packet, angle, mag);
    }
    else if(abs(rd) > (16*PI/180))
    {
    	float mag = rd > 0 ? 1 : -1;
    	return lraRotatePacket(packet, mag*0.8);
    }
    else
    {
    	return lraRawPacket(packet, 0, 0);
    }
}

	~LraPacketGenerator() { delete[] bounds; }

	float* bounds;
	int NUM_LRAS;

	float cacheMag;
	float cacheAngle;
	int cacheIsSpin;
};


#endif