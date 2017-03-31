#include "DebugTimer.h"

DebugTimer::DebugTimer(){

}

void DebugTimer::StartTimer(std::string tag){
	if (startedTimers.size() < MAX_TIMERS){
		//Assumes that no timer with the same name exists
		//i.e. usually means that timers should be cleared after each frame
		auto start = std::chrono::high_resolution_clock::now();
		RunningTimer tr;
		tr.tag = tag;
		tr.start = start;
		startedTimers.push_back(tr);
	}
}

void DebugTimer::StopTimer(std::string tag){
	if (startedTimers.size() < MAX_TIMERS){
		RunningTimer rt;
		findRunningTimer(tag, rt);
		if (rt.tag == ERROR_TAG) return;

		auto end = std::chrono::high_resolution_clock::now();
		TimerResult tr;
		tr.tag = tag;
		tr.milliseconds = chrono::duration_cast<chrono::duration<double>>(end - rt.start).count() * 1000;
		results.push_back(tr);
	}
}

void DebugTimer::ClearAll(){
	results.clear();
	startedTimers.clear();
}

vector<DebugTimer::TimerResult> DebugTimer::GetResults(){
	return results;
}

void DebugTimer::findRunningTimer(string tag, RunningTimer &timer){
	for (int i = 0; i < startedTimers.size(); i++){
		if (startedTimers.at(i).tag == tag){
			timer = startedTimers.at(i);
			return;
		}
	}
	//If not found, return a -1 timer
	RunningTimer rt;
	rt.tag = ERROR_TAG;
	timer = rt;
}

