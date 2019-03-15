#ifndef LEG_STATE_H
#define LEG_STATE_H

#include <math.h>
#include "helper_3dmath.h"

#define BANDS_PER_LEG 2
#define STATES_PER_BAND 3

#define NUM_STATES (BANDS_PER_LEG * STATES_PER_BAND)

void saveRPYAngles(float* angles, Quaternion q) {

 	float num = 2*(q.w*q.x + q.y*q.z);
    float den = 1 - 2*(q.x*q.x + q.y*q.x);
    angles[0] = atan2(num,den);

    num = 2*(q.w*q.y - q.x*q.z);
    angles[1] = asin(num);

    num = 2*(q.w*q.z + q.x*q.y);
    den = 1 - 2*(q.y*q.y + q.z*q.z);
    angles[2] = atan2(num,den);
}

struct LegState {

	float rpyAngles[NUM_STATES];

	LegState(float shankR, float shankP, float shankY, 
			 float thighR, float thighP, float thighY) {
		rpyAngles[0] = shankR;
		rpyAngles[1] = shankP;
		rpyAngles[2] = shankY;
		rpyAngles[3] = thighR;
		rpyAngles[4] = thighP;
		rpyAngles[5] = thighY;
	}

	LegState(float _rpyAngles[NUM_STATES]) {
		for (int i = 0; i < NUM_STATES; i++) {
			rpyAngles[i] = _rpyAngles[i];
		}
	}

	inline float dist(const LegState& otherLegState) const {
		// float sumsq = 0;
		// for (int i = 0; i < NUM_STATES; i++) {
		// 	sumsq += (rpyAngles[i] - otherLegState.rpyAngles[i])*(rpyAngles[i] - otherLegState.rpyAngles[i]);
		// }
		// return sumsq;
		return (rpyAngles[2] - otherLegState.rpyAngles[2])*(rpyAngles[2] - otherLegState.rpyAngles[2]);
	}

	LegState avg(const LegState& other) const {
		
		float f[6];
		for(int i = 0; i < 6; ++i)
		{
			f[i] = (this->rpyAngles[i] + other.rpyAngles[i])/2;
		}

		return LegState(f);
	}

	void computeRPYAngles(float* angles, const Quaternion& current, const Quaternion& initial)
	{
		Quaternion globalDiff = initial.getProduct(current.getConjugate());
		
		VectorFloat coords(globalDiff.x, globalDiff.y, globalDiff.z);
		coords.rotate(current.getConjugate());
		
		Quaternion localDiff = Quaternion(globalDiff.w, coords.x, coords.y, coords.z);
		
		saveRPYAngles(angles, localDiff);
	}

	LegState(	const Quaternion& currShank, const Quaternion& currThigh,
				const Quaternion& initialShank, const Quaternion& initialThigh, bool verbose=false) {

		computeRPYAngles(this->rpyAngles, currThigh, initialThigh);
		computeRPYAngles(this->rpyAngles+3, currShank, initialShank);

	}

	inline LegState getDiff(const LegState& b) const
	{

		float r[NUM_STATES];
		for(int i = 0; i < NUM_STATES; ++i)
		{
			r[i] = this->rpyAngles[i] - b.rpyAngles[i];
		}

		return LegState(r);
	}


};

#endif