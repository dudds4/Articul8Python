#ifndef RECORDING_MAN_H
#define RECORDING_MAN_H

#include <vector>
#include <mutex>
#include "leg_state.h"
#include "helper_3dmath.h"
#include <iostream>

#define GUARD(x) std::lock_guard lk(x)

struct RecordingMan {

	std::vector<LegState> recording;
	
	Quaternion initialQuats[2];
	Quaternion latestQuats[2];

	bool quatsRecieved[2];
	mutable std::mutex myMutex;
	bool isRecording = false;

	RecordingMan() {
		recording = std::vector<LegState>();
		quatsRecieved[0] = false;
		quatsRecieved[1] = false;
	}

	void beginRecording() {
		GUARD(myMutex);
		recording.clear();
		quatsRecieved[0] = false;
		quatsRecieved[1] = false;

		isRecording = true;
	}
	void stopRecording() {
		isRecording = false;

		// let's condense this bad boy
		// GUARD(myMutex);
		// for(int i = recording.size()-2; i >= 1; --i)
		// {

		// }

	}

	void receivedQuat(float* quat, int idx)
	{
		// return;
		latestQuats[idx] = Quaternion(quat);

		if (!quatsRecieved[idx]) {
			initialQuats[idx] = latestQuats[idx];
		}

		quatsRecieved[idx] = true;

		if (isRecording && quatsRecieved[0] && quatsRecieved[1]) {
			GUARD(myMutex);
			recording.emplace_back(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]);
		}		
	}

	// void recievedQuat(const Quaternion& q, int idx) {
	// 	if (!quatsRecieved[idx]) {
	// 		initialQuats[idx] = q;
	// 	}

	// 	quatsRecieved[idx] = true;
	// 	latestQuats[idx] = q;

	// 	if (isRecording && quatsRecieved[0] && quatsRecieved[1]) {
	// 		GUARD(myMutex);

	// 		recording.emplace_back(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]);
	// 		// recording.push_back(LegState::fromIMU(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]));
	// 	}
	// }

	int cachedLatestDiffIdx = 0;

	// only to be called while not recording, so no synchronization needed
	LegState getLatestStateDiff()
	{

		int nChecks, s;
		{
			GUARD(myMutex);

			nChecks = recording.size() / 2;
			s = recording.size();
			
			if(!s)
			{
				// std::cout << "recording is empty" << std::endl;
				return LegState(0, 0, 0, 0, 0, 0);
			}
		}

		LegState current = LegState(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]);

		float minDist = current.dist(recording.at(cachedLatestDiffIdx));
		int minIdx = cachedLatestDiffIdx;

		float tmp;
		int i;

		for(i = cachedLatestDiffIdx+1; i < nChecks && i < s; ++i)
		{
			tmp = current.dist(recording.at(i));
			
			if(tmp < minDist)
			{
				minDist = tmp;
				minIdx = i;
			}
		}

		// check if we need to loop around
		if(i < nChecks)
		{
			nChecks -= i;
			for(i = 0; i < nChecks; ++i)
			{
				tmp = current.dist(recording.at(i));
				
				if(tmp < minDist)
				{
					minDist = tmp;
					minIdx = i;
				}			
			}
		}

		cachedLatestDiffIdx = minIdx;
		auto r = current.getDiff(recording.at(minIdx));
		// auto r = recording.at(minIdx).getDiff(current);
		
		// for(int i = 0; i < 6; ++i)
		// 	std::cout << r.rpyAngles[i] << " ";
		// std::cout << "\n";


		return r; 


	}
};



#endif