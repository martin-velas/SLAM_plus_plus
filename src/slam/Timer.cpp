/*
								+----------------------------------+
								|                                  |
								| ***   Multi-platform timer   *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2006  |
								|                                  |
								|            Timer.cpp             |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam/Timer.cpp
 *	@brief multi-platform timer
 *	@author -tHE SWINe-
 *	@date 2006
 *
 *	@date 2006-08-23
 *
 *	passed code revision
 *
 *	removed some unnecessary \#defines for linux
 *	added higher precision timer for linux (instead of clock(), gettimeofday() can be used)
 *
 *	@date 2007-05-10
 *
 *	added conditional \#include <windows.h> to make it easier to include
 *
 *	@date 2008-04-09
 *
 *	added GetTickCount method, redesigned timer code. should cope nicely with counter overflow
 *	(previous version of timer with QueryPerformanceCounter had some trouble on core duo cpu's)
 *
 *	@date 2008-08-02
 *
 *	cleared up code around n_MaxIntValue() a bit, removed MAX_VALUE_FUNCTION and MAX_VALUE_CONST
 *	and moved all the stuff to Integer.h
 *
 *	@date 2008-08-08
 *
 *	added \#ifdef for windows 64
 *
 *	@date 2009-05-04
 *
 *	fixed mixed windows / linux line endings
 *
 */

#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h>
#else // _WIN32 || _WIN64
#include <unistd.h>
#include <sys/time.h>
#endif // _WIN32 || _WIN64
#include <time.h>
#include "slam/Debug.h"
#include "slam/Timer.h"

/*
 *								=== CDeltaTimer ===
 */

/*
 *	CDeltaTimer::CDeltaTimer()
 *		- default constructor
 */
CDeltaTimer::CDeltaTimer()
{
	//__FuncGuard("CDeltaTimer::CDeltaTimer");

#if defined(TIMER_USE_QPC)
	LARGE_INTEGER t_freq;
	QueryPerformanceFrequency(&t_freq);
	_ASSERTE(t_freq.QuadPart < INT64_MAX);
	m_n_freq = t_freq.QuadPart;
	// determine QPC frequency

#ifdef _DEBUG
	LARGE_INTEGER t_tmp = {0};
	_ASSERTE(n_MaxIntValue(t_tmp.QuadPart) == INT64_MAX);
	// make sure max value of QPC's counter is INT64_MAX
#endif // _DEBUG
#elif defined(TIMER_USE_GETTICKCOUNT)
	m_n_freq = 1000; // one milisecond

	_ASSERTE(n_MaxIntValue(GetTickCount()) == UINT32_MAX);
	// make sure max value of GetTickCount's counter is UINT32_MAX
#elif defined(TIMER_USE_CLOCKGETTIME)
	m_n_freq = 1000000000; // one nanosecond

#ifdef _DEBUG
	timespec t_tmp_time = {0, 0};
	//_ASSERTE(n_MaxIntValue(t_tmp_time.tv_sec) < UINT64_MAX / 1000000000);
	//_ASSERTE(n_MaxIntValue(t_tmp_time.tv_sec) * 1000000000 <= UINT64_MAX - 999999999); // these are broken on 64-bit OS'
	// make sure we fit into int64_t
#endif // _DEBUG
#elif defined(TIMER_USE_GETTIMEOFDAY)
	m_n_freq = 1000000; // one microsecond

#ifdef _DEBUG
	timeval t_tmp_time = {0, 0};
	//_ASSERTE(n_MaxIntValue(t_tmp_time.tv_sec) < UINT64_MAX / 1000000);
	//_ASSERTE(n_MaxIntValue(t_tmp_time.tv_sec) * 1000000 <= UINT64_MAX - 999999); // these are broken on 64-bit OS'
	// make sure we fit into int64_t
#endif // _DEBUG
#else // clock
	m_n_freq = CLOCKS_PER_SEC;
#endif
	Reset();
}

/*
 *	int64_t CDeltaTimer::n_Frequency() const
 *		- returns timer frequency (inverse of smallest time step)
 */
int64_t CDeltaTimer::n_Frequency() const
{
	//__FuncGuard("CDeltaTimer::n_Frequency");

	return m_n_freq;
}

/*
 *	static inline int64_t CDeltaTimer::n_SampleTimer()
 *		- returns time counter sample
 */
inline int64_t CDeltaTimer::n_SampleTimer()
{
	//__FuncGuard("CDeltaTimer::n_SampleTimer");

#if defined(TIMER_USE_QPC)
	LARGE_INTEGER t_time;
	QueryPerformanceCounter(&t_time);
	_ASSERTE(t_time.QuadPart <= INT64_MAX); // it can equal
	return t_time.QuadPart;
#elif defined(TIMER_USE_GETTICKCOUNT)
	return GetTickCount();
#elif defined(TIMER_USE_CLOCKGETTIME)
	timespec t_tmp_time;
    clock_gettime(CLOCK_MONOTONIC, &t_tmp_time);
	// could use CLOCK_PROCESS_CPUTIME_ID for higher precision, but that has problems
	// with systems with more CPUs (when the process migrates, a different timer with
	// a different value might be sampled)

	_ASSERTE((t_tmp_time.tv_sec >= 0 && t_tmp_time.tv_nsec >= 0) ||
		t_tmp_time.tv_sec > t_tmp_time.tv_nsec / 1000000000 ||
		(t_tmp_time.tv_sec >= t_tmp_time.tv_nsec / 1000000000 &&
		!(t_tmp_time.tv_nsec % 1000000000))); // make sure that the time will be positive
	// otherwise madness ensues

	return (t_tmp_time.tv_sec * int64_t(1000000000) + t_tmp_time.tv_nsec) & INT64_MAX; // make sure this does not overflow to negative values; implement by dropping the top bit as the value might have already overflown before the and-ing and doing a modulo with a non-power-of-two would give a bad result
	// returns values in [0, INT64_MAX] (inclusive)
#elif defined(TIMER_USE_GETTIMEOFDAY)
	timeval t_tmp_time;
    gettimeofday(&t_tmp_time, NULL);

	_ASSERTE((t_tmp_time.tv_sec >= 0 && t_tmp_time.tv_usec >= 0) ||
		t_tmp_time.tv_sec > t_tmp_time.tv_usec / 1000000 ||
		(t_tmp_time.tv_sec >= t_tmp_time.tv_nsec / 1000000 &&
		!(t_tmp_time.tv_nsec % 1000000))); // make sure that the time will be positive
	// otherwise madness ensues

	return (t_tmp_time.tv_sec * int64_t(1000000) + t_tmp_time.tv_usec) & INT64_MAX; // make sure this does not overflow to negative values; implement by dropping the top bit as the value might have already overflown before the and-ing and doing a modulo with a non-power-of-two would give a bad result
	// returns values in [0, INT64_MAX] (inclusive)
#else // clock
	return clock();
#endif
}

/*
 *	void CDeltaTimer::Reset()
 *		- resets timer (sets time to zero)
 */
void CDeltaTimer::Reset()
{
	//__FuncGuard("CDeltaTimer::ResetTimer");

	//m_f_time = 0;
	m_n_time = n_SampleTimer();
}

/*
 *	double CDeltaTimer::f_Time() const
 *		- returns time in seconds
 *		- should cope nicely with counter overflows
 */
double CDeltaTimer::f_Time() const
{
	//__FuncGuard("CDeltaTimer::f_Time");

	const int64_t n_cur_time = n_SampleTimer(); // this will be remembered, will not be 
	// determine current time

	double f_time = 0;

	int64_t n_delta_time;
	if(n_cur_time >= m_n_time)
		n_delta_time = n_cur_time - m_n_time;
	else {
#if defined(TIMER_USE_QPC)
		const int64_t n_max_time_value = INT64_MAX;
#elif defined(TIMER_USE_GETTICKCOUNT)
		const int64_t n_max_time_value = UINT32_MAX;
#elif defined(TIMER_USE_CLOCKGETTIME)
		const timespec t_tmp_time = {0, 0};
		const int64_t n_max_secs = n_MaxIntValue(t_tmp_time.tv_sec);
		const int64_t n_max_time_value = (n_max_secs > INT64_MAX / 1000000000 ||
			n_max_secs * 1000000000 > INT64_MAX - 999999999)? INT64_MAX :
			n_max_secs * 1000000000 + 999999999;
		// the samples are in [0, INT64_MAX] (inclusive) or less if t_tmp_time.tv_sec is a 32-bit number
#elif defined(TIMER_USE_GETTIMEOFDAY)
		const timeval t_tmp_time = {0, 0};
		const int64_t n_max_secs = n_MaxIntValue(t_tmp_time.tv_sec);
		const int64_t n_max_time_value = (n_max_secs > INT64_MAX / 1000000 ||
			n_max_secs * 1000000 > INT64_MAX - 999999)? INT64_MAX :
			n_max_secs * 1000000 + 999999;
		// the samples are in [0, INT64_MAX] (inclusive) or less if t_tmp_time.tv_sec is a 32-bit number
#else // TIMER_USE_QPC
		const int64_t n_max_time_value = CMaxIntValue<clock_t>::result();
#endif // TIMER_USE_QPC
		// determine maximal time value, based on used counter

		n_delta_time = n_max_time_value - m_n_time;
		// time until overflow

		if(n_delta_time <= INT64_MAX - n_cur_time)
			n_delta_time += n_cur_time;
		else {
			f_time = double(n_cur_time) / m_n_freq;
			// adding n_cur_time would cause another overflow ... so add it this way
		}
		// time after overflow
	}
	// calculate delta time

	f_time += double(n_delta_time) / m_n_freq;
	m_n_time = n_cur_time;
	// integrate time

	return f_time;
}

/*
 *								=== ~CDeltaTimer ===
 */
