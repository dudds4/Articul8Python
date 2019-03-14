#ifndef RECORDING_MAN_H
#define RECORDING_MAN_H

#include <vector>
#include "leg_state.h"
#include "quaternion.h"

#define GUARD(x) std::lock_guard lk(x)

struct RecordingMan {

	std::vector<LegState> recording;
	Quaternion initialQuats[2];
	Quaternion latestQuats[2];
	bool recievedQuat[2];
	mutable std::mutex myMutex;

	RecordingMan() {
		recording = std::vector<LegState>();
		recievedQuat[0] = false;
		recievedQuat[1] = false;
	}

	void beginRecording() {
		recording = std::vector<LegState>();
		recievedQuat[0] = false;
		recievedQuat[1] = false;
	}

	void recievedQuat(const Quaternion& q, int idx) {
		if (!recievedQuat[idx]) {
			initialQuats[idx] = q;
		}
		recievedQuat[idx] = true;
		latestQuats[idx] = q;

		if (recievedQuat[0] && recievedQuat[1]) {
			GUARD(myMutex);
			recording.insert(LegState.fromIMU(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]));
		}
	}
};

#endif