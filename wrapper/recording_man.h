#ifndef RECORDING_MAN_H
#define RECORDING_MAN_H

#include <vector>
#include <mutex>
#include "leg_state.h"
#include "helper_3dmath.h"
#include <iostream>

#define GUARD(x) std::lock_guard lk(x)

#ifndef LIKELY
#define LIKELY(condition) __builtin_expect(static_cast<bool>(condition), 1)
#endif

Quaternion zeroExtRot(float goalRoll, float goalPitch, float goalYaw, const Quaternion& q);
Quaternion ThreeAxis(const VectorFloat &v, float angle);

struct RecordingMan {

	std::vector<LegState> recording;
	
	Quaternion initialQuats[2];
	Quaternion latestQuats[2];

	Quaternion initialExerciseQuats[2];
	bool gotInitialExercise[2];

	bool quatsRecieved[2];
	mutable std::mutex myMutex;
	bool m_recording = false;
	bool m_exercising = false;

	int cachedLatestDiffIdx = 0;

	RecordingMan() {
		recording = std::vector<LegState>();
		quatsRecieved[0] = false;
		quatsRecieved[1] = false;
	}

	void startRecording() {
		if (m_exercising || m_recording) {
			return;
		}
		GUARD(myMutex);
		recording.clear();
		quatsRecieved[0] = false;
		quatsRecieved[1] = false;

		m_recording = true;
	}
	void stopRecording() {
		m_recording = false;
	}

	void startExercise() {
		if (m_exercising || m_recording) {
			return;
		}
		gotInitialExercise[0] = false;
		gotInitialExercise[1] = false;

		m_exercising = true;

		cachedLatestDiffIdx = 0;
	}

	void stopExercise() {
		m_exercising = false;
	}

	void receivedQuat(float* quat, int idx)
	{
		// return;
		latestQuats[idx] = Quaternion(quat);

		if (!quatsRecieved[idx]) {
			initialQuats[idx] = latestQuats[idx];
		}

		if (m_exercising && !gotInitialExercise[idx]) {
			initialExerciseQuats[idx] = latestQuats[idx];
			gotInitialExercise[idx] = true;
		}

		quatsRecieved[idx] = true;

		if (m_recording && quatsRecieved[0] && quatsRecieved[1]) {
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

	// 	if (m_recording && quatsRecieved[0] && quatsRecieved[1]) {
	// 		GUARD(myMutex);

	// 		recording.emplace_back(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]);
	// 		// recording.push_back(LegState::fromIMU(latestQuats[1], latestQuats[0], initialQuats[1], initialQuats[0]));
	// 	}
	// }

	// only to be called while not recording, so no synchronization needed
	LegState getLatestStateDiff()
	{

		int nChecks, s;
		{
			GUARD(myMutex);

			nChecks = recording.size() / 4;
			s = recording.size();
			
			if(!s)
			{
				// std::cout << "recording is empty" << std::endl;
				return LegState(0, 0, 0, 0, 0, 0);
			}
		}

		if (!gotInitialExercise[0] || !gotInitialExercise[1]) {
			return LegState(0, 0, 0, 0, 0, 0);
		}

		LegState current = LegState(latestQuats[1], latestQuats[0], initialExerciseQuats[1], initialExerciseQuats[0]);

		// std::cout << "Curr Roll: " << current.rpyAngles[0];
		// std::cout << ", Curr Pitch: " << current.rpyAngles[1];
		// std::cout << ", Curr Yaw: " << current.rpyAngles[2] << std::endl;

		float minDist = current.dist(recording.at(cachedLatestDiffIdx));
		int minIdx = cachedLatestDiffIdx;

		float tmp;
		int i;

		for(i = cachedLatestDiffIdx+1; i < cachedLatestDiffIdx + nChecks && i < s; ++i)
		{
			tmp = current.dist(recording.at(i));
			
			if(tmp < minDist)
			{
				minDist = tmp;
				minIdx = i;
			}
		}

		// check if we need to loop around
		if(i >= s)
		{
			nChecks -= (i-cachedLatestDiffIdx-1);
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

		// std::cout << "Min Idx: " << minIdx << " / " << s << std::endl;

		if (minIdx < cachedLatestDiffIdx) {

			std::cout << "Did a rep!" << std::endl;

			initialExerciseQuats[0] = latestQuats[0];
			initialExerciseQuats[1] = latestQuats[1];
			
			// Multiply initialExerciseQuats by roll quaternion inverse

			// {
			// 	initialExerciseQuats[0] = zeroExtRot(
			// 								recording.at(minIdx).rpyAngles[0],
			// 								current.rpyAngles[1],
			// 								current.rpyAngles[2],
			// 								latestQuats[0]
			// 	);
				
			// 	initialExerciseQuats[1] = zeroExtRot(
			// 								recording.at(minIdx).rpyAngles[3],
			// 								current.rpyAngles[4],
			// 								current.rpyAngles[5],
			// 								latestQuats[1]
			// 	);

			// 	// initialExerciseQuats[0] = q.getProduct(latestQuats[0]);

			// 	// q = 	Quaternion(xaxis, recording.at(minIdx).rpyAngles[3]).getProduct(
			// 	// 				Quaternion(yaxis, current.rpyAngles[4]).getProduct(
			// 	// 					Quaternion(zaxis, current.rpyAngles[5])

			// 	// 					));

			// 	// initialExerciseQuats[1] = q.getProduct(latestQuats[1]);		

			// }

			// LegState test = LegState(latestQuats[1], latestQuats[0], initialExerciseQuats[1], initialExerciseQuats[0]);

			// LegState goalState(
			// 	recording.at(minIdx).rpyAngles[0],
			// 	current.rpyAngles[1],
			// 	current.rpyAngles[2],
			// 	recording.at(minIdx).rpyAngles[3],
			// 	current.rpyAngles[4],
			// 	current.rpyAngles[5]
			// 	);

			// test = test.getDiff(goalState);

			// float threshold = 0.001;
			// if (abs(test.rpyAngles[0]) < threshold) {
			// 	std::cout << "Thigh is GOOD" << std::endl;
			// }
			// if (abs(test.rpyAngles[3]) < threshold) {
			// 	std::cout << "Shank is GOOD" << std::endl;
			// }
		}

		cachedLatestDiffIdx = minIdx;
		auto r = current.getDiff(recording.at(minIdx));
		// auto r = recording.at(minIdx).getDiff(current);

		return r; 
	}
};


Quaternion zeroExtRot(float goalRoll, float goalPitch, float goalYaw, const Quaternion& qCurrent)
{
	VectorFloat xAxis(1, 0, 0);
	VectorFloat yAxis(0, 1, 0);
	VectorFloat zAxis(0, 0, 1);

	xAxis.rotate(&qCurrent);
	yAxis.rotate(&qCurrent);
	zAxis.rotate(&qCurrent);

	auto qGlobalDiff = 	ThreeAxis(xAxis, goalRoll).getProduct(
						ThreeAxis(yAxis, goalPitch).getProduct(
						ThreeAxis(zAxis, goalYaw)
						));

	return qGlobalDiff.getProduct(qCurrent);
}

Quaternion ThreeAxis(const VectorFloat &v, float angle)
{
    float s = sin(angle/2) / v.getMagnitude();
    return Quaternion(cos(angle/2), s*v.x, s*v.y, s*v.z);
}

#endif