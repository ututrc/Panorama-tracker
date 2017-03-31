#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <sstream>
#include <iomanip>
#define ERROR_TAG "timer_not_found_err"
#define MAX_TIMERS 32

using namespace std;
/*
A class that can be used to stop and start timers 
which can be used for performance profiling.
*/
class DebugTimer{
public:
	struct TimerResult{
		string tag;
		float milliseconds;
		string toString(){
			stringstream ss;
			ss << std::setprecision(2) << tag << ": " << milliseconds << "ms";
			return ss.str();
		}
	};

	struct RunningTimer{
		string tag;
		chrono::time_point<chrono::system_clock, chrono::system_clock::duration> start;
	};

	DebugTimer();
	void StartTimer(string tag);
	void StopTimer(string tag);
	void ClearAll();
	vector<TimerResult> GetResults();
private:
	vector<TimerResult> results;
	vector<RunningTimer> startedTimers;
	void findRunningTimer(string tag, RunningTimer &timer);
};