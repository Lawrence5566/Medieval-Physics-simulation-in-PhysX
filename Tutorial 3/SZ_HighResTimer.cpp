#include "SZ_HighResTimer.h"

SZ_HighResTimer::SZ_HighResTimer()
{
}
void SZ_HighResTimer::resetChronoTimer()
{
	startChrono = Clock::now();
}
float SZ_HighResTimer::getChronoTime()
{
	std::chrono::steady_clock::time_point now = Clock::now();
	auto timeDiff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - startChrono).count();
	return (float)timeDiff/1000000000; //convert to seconds
}