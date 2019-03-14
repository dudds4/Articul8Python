#ifndef LEG_STATE_H
#define LEG_STATE_H

#include <math.h>

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
			rpyAngles[i] = _rpyAngles[i]
		}
	}

	float dist(const LegState& otherLegState) {
		float sumsq = 0;
		for (int i = 0; i < NUM_STATES; i++) {
			sumsq += (rpyAngles[i] - otherLegState.rpyAngles[i])*(rpyAngles[i] - otherLegState.rpyAngles[i]);
		}
		return sumsq;
	}

	static LegState fromIMU(const Quaternion& currShank, Quaternion currThigh,
							Quaternion initialShank, Quaternion initialThigh) {

		float rpyAngles[NUM_STATES];

		Quaternion globalShankDiff = initialShank.getProduct(currShank.getConjugate());
		VectorFloat shankCoords(globalShankDiff.x, globalShankDiff.y, globalShankDiff.z);
		shankCoords.rotate(currShank.getConjugate());

		Quaternion localShankDiff = Quaternion(globalShankDiff.w, currShank.x, currShank.y, currShank.z);

		Quaternion globalThighDiff = initialThigh.getProduct(currThigh.getConjugate());
		VectorFloat thighCoords(globalThighDiff.x, globalThighDiff.y, globalThighDiff.z);
		thighCoords.rotate(currThigh.getConjugate());

		Quaternion localThighDiff = Quaternion(globalThighDiff.w, thighCoords.x, thighCoords.y, thighCoords.z);

		saveRPYAngles(rpyAngles,   localShankDiff);
		saveRPYAngles(rpyAngles+3, localThighDiff);

		return LegState(rpyAngles);
	}


	// david's code
	float r1, p1, y1;
	float r2, p2, y2;

	inline LegState getDiff(const LegState& b)
	{
		return {
			.r1 = this->r1 - b.r1,
			.p1 = this->p1 - b.p1,
			.y1 = this->y1 - b.y1,

			.r2 = this->r2 - b.r2,
			.p2 = this->p2 - b.p2,
			.y2 = this->y2 - b.y2,
		}
	}

	inline float getDist(const LegState& a, const LegState& b)
	{
		LegState diff = getDiff(b);

		return 	diff.r1*diff.r1 + diff.p1*diff.p1 + diff.y1*diff.y1 +
				diff.r2*diff.r2 + diff.p2*diff.p2 + diff.y2*diff.y2;
	}


};

#endif