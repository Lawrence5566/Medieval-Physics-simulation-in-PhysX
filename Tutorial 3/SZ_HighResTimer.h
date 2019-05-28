
#ifndef aHighResTimerFILE
#define aHighResTimerFILE
#include <iostream>
#include <chrono>

//Written by Olivier Szymanezyk oszymanezyk@lincoln.ac.uk
class SZ_HighResTimer
{
private:
	typedef std::chrono::high_resolution_clock Clock;
	std::chrono::steady_clock::time_point startChrono;

public:
	SZ_HighResTimer();
	void resetChronoTimer();
	float getChronoTime();
};
#endif
