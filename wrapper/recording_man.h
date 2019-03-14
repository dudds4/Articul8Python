#ifndef RECORDING_MAN_H
#define RECORDING_MAN_H

#include <vector>
#include <mutex>
#include "leg_state.h"
#include "helper_3dmath.h"

#define GUARD(x) std::lock_guard lk(x)

struct RecordingMan {

	std::vector<LegState> recording;
	Quaternion initialQuats[2];
	Quaternion latestQuats[2];
	bool quatsRecieved[2];
	mutable std::mutex myMutex;

	RecordingMan() {
		recording = std::vector<LegState>();
		quatsRecieved[0] = false;
		quatsRecieved[1] = false;
	}

	void beginRecording() {
		recording = std::vector<LegState>();
		quatsRecieved[0] = false;
		quatsRecieved[1] = false;
	}

	void recievedQuat(const Quaternion& q, int idx) {
		if (!quatsRecieved[idx]) {
			initialQuats[idx] = q;
		}
		quatsRecieved[idx] = true;
		latestQuats[idx] = q;

		if (quatsRecieved[0] && quatsRecieved[1]) {
			GUARD(myMutex);
			recording.push_back(LegState::fromIMU(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]));
		}
	}
};

#endif