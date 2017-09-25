/*
								+----------------------------------+
								|                                  |
								| ***   Multi-platform timer   *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2006  |
								|                                  |
								|             Timer.h              |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __TIMER_INCLUDED
#define __TIMER_INCLUDED

/**
 *	@file include/slam/Timer.h
 *	@brief multi-platform timer
 *	@author -tHE SWINe-
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
 *	@date 2010-01-08
 *
 *	added the PRItime and PRItimeparams macros for easy printfing of time values
 *
 *	@date 2010-10-29
 *
 *	Unified windows detection macro to "#if defined(_WIN32) || defined(_WIN64)".
 *
 *	Added CTimer::Reset() function in place of CTimer::ResetTimer() (the old function
 *	is kept for backward compatibility).
 *
 *	@date 2013-01-13
 *
 *	Added new and more precise TIMER_USE_CLOCKGETTIME for linux.
 *
 */

/**
 *	@def PRItime
 *
 *	@brief printf macro for printing time in the "hh:mm:ss.ss" format
 *
 *	This macro is used similar to standard PRI64 macro, it just defines
 *	proper formatting string. It should be used in conjunction with the
 *	PRItimeparams macro, as follows:
 *	@code
 *	double f_time = 123456;
 *	printf("elapsed time: " PRItime "\n", PRItimeparams(f_time));
 *	@endcode
 */
#define PRItime "%s%02d:%02d:%05.2f"

/**
 *	@def PRItimeprecise
 *
 *	@brief printf macro for printing time in the "hh:mm:ss.sssss" format
 *
 *	This macro is used similar to standard PRI64 macro, it just defines
 *	proper formatting string. It should be used in conjunction with the
 *	PRItimeparams macro, as follows:
 *	@code
 *	double f_time = 123456;
 *	printf("elapsed time: " PRItimeprecise "\n", PRItimeparams(f_time));
 *	@endcode
 */
#define PRItimeprecise "%s%02d:%02d:%08.5f"

/**
 *	@def PRItimeparams
 *
 *	@brief splits it's argument to integer hours, integer minutes
 *		and floating-point seconds
 *
 *	Calculates sign and three values in hours, minutes and seconds from given time.
 *	These values are separated using colons, and are intended to be used as
 *	function parameters. It could be used as:
 *
 *	@code
 *	void PrintTime3(const char *sign, int hh, int mm, float ss)
 *	{
 *		printf("time is: %s%02d:%02d:%05.2f ...\n", sign, hh, mm, ss);
 *	}
 *
 *	void PrintTime(float seconds)
 *	{
 *		PrintTime3(PRItimeparams(seconds));
 *	}
 *	@endcode
 *
 *	It's however intended to be used along with PRItime. Refer to PRItime
 *	macro documentation for more details.
 *
 *	@param[in] f is time in seconds to be split into hours, minutes, seconds
 */
#define PRItimeparams(f) (((f) >= 0)? "" : "-"), int(f) / 3600, \
	(((f) >= 0)? 1 : -1) * (int(f) / 60 - 60 * (int(f) / 3600)), \
	(((f) >= 0)? 1 : -1) * ((f) - 60 * (int(f) / 60))

/**
 *	@def TIMER_ALLOW_QPC
 *	@brief this macro enables use of QueryPerformanceCounter() function (win32-only)
 */
#define TIMER_ALLOW_QPC

/**
 *	@def TIMER_ALLOW_GETTICKCOUNT
 *	@brief this macro enables use of GetTickCount() function (win32-only)
 */
#define TIMER_ALLOW_GETTICKCOUNT

#if !defined(_WIN32) && !defined(_WIN64)
#include <unistd.h>
#if defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0 && defined(_POSIX_MONOTONIC_CLOCK)

/**
 *	@def TIMER_ALLOW_CLOCKGETTIME
 *	@brief this macro enables use of clock_gettime() function (unix-only)
 *	@note On some systems, this may need linking with librt (add "-lrt").
 */
#define TIMER_ALLOW_CLOCKGETTIME

#endif // _POSIX_TIMERS && _POSIX_TIMERS > 0 && _POSIX_MONOTONIC_CLOCK
#endif // !_WIN32 && !_WIN64

/**
 *	@def TIMER_ALLOW_GETTIMEOFDAY
 *	@brief this macro enables use of gettimeofday() function (unix-only)
 *	@note The time of gettimeofday() is possibly drifting when time synchronization takes place.
 *		This will only be used where TIMER_ALLOW_CLOCKGETTIME is not available.
 */
#define TIMER_ALLOW_GETTIMEOFDAY

/**
 *	@def TIMER_METHOD_OVERRIDE
 *	@brief this macro disables automatic timer method selection
 *
 *	One of following macros must be defined as well to choose timer method:
 *		TIMER_USE_QPC				(win32-only, resolution less than 1 usec, depends on CPU)
 *		TIMER_USE_GETTICKCOUNT		(win32-only, resolution 1 msec)
 *		TIMER_USE_CLOCKGETTIME		(unix-only, resolution 1 usec)
 *		TIMER_USE_GETTIMEOFDAY		(unix-only, resolution 1 usec, possibly drifting when time synchronization takes place)
 *		TIMER_USE_CLOCK				(default fallback, resolution 1 msec or less, depends on os)
 */
//#define TIMER_METHOD_OVERRIDE

#ifndef TIMER_METHOD_OVERRIDE
#if defined(_WIN32) || defined(_WIN64)

#if defined(TIMER_ALLOW_QPC)
/**
 *	@def TIMER_USE_QPC
 *	@brief selected timer method
 */
#define TIMER_USE_QPC
#elif defined(TIMER_ALLOW_GETTICKCOUNT)
/**
 *	@def TIMER_USE_GETTICKCOUNT
 *	@brief selected timer method
 */
#define TIMER_USE_GETTICKCOUNT
#else // TIMER_ALLOW_QPC
/**
 *	@def TIMER_USE_CLOCK
 *	@brief selected timer method
 */
#define TIMER_USE_CLOCK
#endif // TIMER_ALLOW_QPC

#else // _WIN32 || _WIN64

#if defined(TIMER_ALLOW_CLOCKGETTIME)
/**
 *	@def TIMER_USE_CLOCKGETTIME
 *	@brief selected timer method
 */
#define TIMER_USE_CLOCKGETTIME
#elif defined(TIMER_ALLOW_GETTIMEOFDAY)
/**
 *	@def TIMER_USE_GETTIMEOFDAY
 *	@brief selected timer method
 */
#define TIMER_USE_GETTIMEOFDAY
#else // TIMER_ALLOW_CLOCKGETTIME
/**
 *	@def TIMER_USE_CLOCK
 *	@brief selected timer method
 */
#define TIMER_USE_CLOCK
#endif // TIMER_ALLOW_CLOCKGETTIME

#endif // _WIN32 || _WIN64
#endif // !TIMER_METHOD_OVERRIDE

#include "Integer.h"

/**
 *	@brief a simple delta timer class
 */
class CDeltaTimer {
	mutable int64_t m_n_freq, m_n_time;

public:
	/**
	 *	@brief default constructor
	 */
	CDeltaTimer();

	/**
	 *	@brief resets timer (starts counting time from now)
	 */
	void Reset();

	/**
	 *	@brief gets time in seconds
	 *	@return Returns time since creation of this object, since
	 *		the last call to Reset() or since the last call to
	 *		f_Time(), whichever occurred last, in seconds.
	 *	@note This should cope nicely with counter overflows.
	 */
	double f_Time() const;

	/**
	 *	@brief gets timer frequency
	 *	@return Returns timer frequency (inverse of the smallest time step).
	 */
	int64_t n_Frequency() const;

protected:
	/**
	 *	@brief gets raw timer sample
	 *	@return Returns raw timer sample, semantic of the value depends on selected timer method.
	 */
	static inline int64_t n_SampleTimer();
};

/**
 *	@brief a simple timer class
 */
class CTimer {
	mutable CDeltaTimer m_timer;
	mutable double m_f_time;

public:
	/**
	 *	@brief default constructor
	 */
	inline CTimer()
		:m_f_time(0)
	{}

	/**
	 *	@brief resets timer (sets time to zero)
	 */
	inline void Reset()
	{
		m_f_time = 0;
		m_timer.Reset();
	}

	/**
	 *	@brief resets timer (sets time to zero)
	 *	@deprecated This function is deprecated in favor of Reset().
	 */
	inline void ResetTimer()
	{
		Reset();
	}

	/**
	 *	@brief gets time in seconds
	 *	@return Returns time since creation of this object or since
	 *		the last call to ResetTimer(), in seconds.
	 *	@note This should cope nicely with counter overflows.
	 */
	inline double f_Time() const
	{
		m_f_time += m_timer.f_Time();
		return m_f_time;
	}

	/**
	 *	@brief gets timer frequency
	 *	@return Returns timer frequency (inverse of the smallest time step).
	 */
	inline int64_t n_Frequency() const
	{
		return m_timer.n_Frequency();
	}
};

/**
 *	@brief timer sampler object
 *
 *	We are intersting in sampling a timer and measuring performance of pieces of code.
 *	In our application, those are usually structures as some total time + breakdown of
 *	the individual stages of the algorithm (possibly even in a tree structure).
 *
 *	The general requirements are:
 *		- need to use a single timer sample for subsequent events
 *		- need to be able to easily disable it, without causing a mess in the code
 *		- don't want crazy macros that would mess the code up
 *
 *	Possible problems:
 *		- time of some phase is not calculated correctly (e.g. the phase was split
 *		  and incorrect timer samples are being subtracted)
 *		- total time is not summed up correctly
 *		- thanks to code branching, time is not calculated correctly
 *		- thanks to premature exit (return, break), time is not calculated correctly
 *
 *	This can be used in analogous fashion to Matlab's tic() and toc(), without using
 *	the sissy names.
 *
 *	This would be used as follows:
 *	@code
 *	CTimer timer; // a global timer object (don't want to construct many of those, want to share it)
 *	double f_time_a = 0, f_time_b = 0; // time spent in different parts of the algorithm
 *	double f_total_time; // total time (we want to represent it explicitly to avoid later mistakes)
 *
 *	CTimerSampler tic(timer); // timing starts here
 *	AlgorithmPartA();
 *	tic.Accum_DiffSample(f_time_a); // like a "toc" and a next "tic"
 *	AlgorithmPartB();
 *	tic.Accum_DiffSample(f_time_b); // like a "toc"
 *	tic.Accum_CumTime_LastSample(f_total_time); // does not imply a sample (the last sample is used)
 *	@endcode
 *
 *	To measure in a tree structure, one would need a stackable timer sampler (problems with
 *	threading and many additional issues with correct timing). Instead, one can use more timer
 *	sampler objects in a stacked fashion (the measurements of inner phases will not fit perfectly
 *	with the outer phase, thanks to double timer samples - no good way around that). Could
 *	add functions like Fork() and Join() to support explicit hierarchy of the timed stages.
 *
 *	It is also easy to make a void sampler class, which will not touch the referenced variables,
 *	effectively optimizing all the timing code away (but the variables themselves must remain
 *	in place, see CVoidTimerSampler).
 *
 *	This solves correct summation of the total time and algorithm stages.
 *
 *	Problems with the total time and branching / early exit could be solved by passing a reference
 *	to the total time variable in the constructor, and the destructor would accumulate the cummulative
 *	time automatically. This assumes that the scope of the sampler is correctly limited and that
 *	the last sample is properly sampled (cumtime does not imply sampling). Probably causes more
 *	problems than it solves.
 *
 *	Another approach (possibly more efficient) would be to declare the timed periods
 *	as global objects. That could look like this:
 *
 *	@code
 *	CTimer timer;
 *	CTimerSampler<CMakeTypelist(part_a, part_b)> algorithm_time;
 *
 *	algorithm_time.start();
 *	AlgorithmPartA();
 *	algorithm_time.sample(part_a);
 *	AlgorithmPartB();
 *	algorithm_time.sample(part_b);
 *	@endcode
 *
 *	This would allow for completely disabling the timing, including the data.
 */
class CTimerSampler {
public:
	typedef double _TySample; /**< @brief time sample data type */

protected:
	CTimer &m_r_timer; /**< @brief reference to the shared timer object */
	double m_f_last_sample_time; /**< @brief last time a sample was taken */
	// hot

	double m_f_start_time; /**< @brief time of the constructor being called */
	// cold

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_timer is timer object to be used by this sampler
	 */
	inline CTimerSampler(CTimer &r_timer)
		:m_r_timer(r_timer), m_f_last_sample_time(r_timer.f_Time())
	{
		m_f_start_time = m_f_last_sample_time;
	}

	/**
	 *	@brief accumulates differential sample
	 *
	 *	This increments the argument by the amount of time that passed since
	 *	the last call to Accum_DiffSample() or since the constructor in case
	 *	Accum_DiffSample() was not called before.
	 *
	 *	@param[in,out] r_f_time_accum is time accumulation variable
	 */
	inline void Accum_DiffSample(_TySample &r_f_time_accum)
	{
		double f_time = m_r_timer.f_Time();
		_ASSERTE(f_time >= m_f_last_sample_time); // we do not own the timer, make sure noone reset it since
		r_f_time_accum += f_time - m_f_last_sample_time;
		m_f_last_sample_time = f_time;
	}

	/**
	 *	@brief accumulates cummulative sample
	 *
	 *	This increments the argument by the amount of time that passed since
	 *	the constructor.
	 *
	 *	@param[in,out] r_f_time_accum is time accumulation variable
	 *
	 *	@note This doesn't imply timer sample, instead time when Accum_DiffSample()
	 *		was called the last time is used (if it was never called, this function
	 *		has no effect).
	 */
	inline void Accum_CumTime_LastSample(_TySample &r_f_time_accum) const // does not modify the timer
	{
		r_f_time_accum += m_f_last_sample_time - m_f_start_time;
	}
};

/**
 *	@brief dummy no-op timer sampler
 */
class CVoidTimerSampler {
public:
	/**
	 *	@brief (dummy) time sample data type
	 */
	struct _TySample {
		/**
		 *	@brief default constructor
		 */
		inline _TySample()
		{}

		/**
		 *	@brief initialization constructor (has no effect)
		 *	@param[in] s is time sample (ignored)
		 */
		inline _TySample(double UNUSED(s))
		{}

		/**
		 *	@brief conversion to double
		 *	@return Returns 0 (always).
		 */
		inline operator double() const
		{
			return .0;
		}

		/**
		 *	@brief copy-operator (has no effect)
		 *	@param[in] s is time sample (ignored)
		 */
		inline void operator =(double UNUSED(s)) const
		{}

		/**
		 *	@brief copy-operator (has no effect)
		 *	@param[in] s is time sample (ignored)
		 */
		inline void operator =(_TySample UNUSED(s)) const
		{}

		/**
		 *	@brief addition operator (has no effect)
		 *	@param[in] s is time sample (ignored)
		 */
		inline void operator +=(double UNUSED(s)) const
		{}

		/**
		 *	@brief addition operator (has no effect)
		 *	@param[in] s is time sample (ignored)
		 */
		inline void operator +=(_TySample UNUSED(s)) const
		{}
	};

public:
	/**
	 *	@brief default constructor (has no effect)
	 *	@param[in] r_timer is timer object to be used by this sampler (unused)
	 */
	inline CVoidTimerSampler(CTimer &UNUSED(r_timer))
	{}

	/**
	 *	@brief accumulates differential sample (has no effect)
	 *	@param[in,out] r_f_time_accum is time accumulation variable (unused)
	 */
	inline void Accum_DiffSample(_TySample &UNUSED(r_f_time_accum))
	{}

	/**
	 *	@brief accumulates cummulative sample (has no effect)
	 *	@param[in,out] r_f_time_accum is time accumulation variable (unused)
	 */
	inline void Accum_CumTime_LastSample(_TySample &UNUSED(r_f_time_accum)) const
	{}

	/**
	 *	@brief accumulates differential sample (has no effect)
	 *	@param[in,out] r_f_time_accum is time accumulation variable (unused)
	 */
	inline void Accum_DiffSample(double &UNUSED(r_f_time_accum)) // use double, in case the samples are in some structures
	{}

	/**
	 *	@brief accumulates cummulative sample (has no effect)
	 *	@param[in,out] r_f_time_accum is time accumulation variable (unused)
	 */
	inline void Accum_CumTime_LastSample(double &UNUSED(r_f_time_accum)) const // use double, in case the samples are in some structures
	{}
};

/**
 *	@brief timer sampler traits
 *
 *	This chooses time sampler type based on whether the timing is supposed to be enabled
 *	or disabled. That allows simple specification of timing or zero-cost impostor, based
 *	on a template parameter.
 *
 *	@param[in] _b_enable_timing is timing enable flag
 */
template <bool _b_enable_timing>
class CTimerSamplerTraits {
public:
	/**
	 *	@brief configuration stored as enum
	 */
	enum {
		b_enable_timing = _b_enable_timing /**< @brief timing enabled flag */
	};

	typedef CVoidTimerSampler _TyTimerSampler; /**< @brief timer sampler type */
	typedef CVoidTimerSampler _TyTimerSamplerRef; /**< @brief timer sampler type reference (also a void type, want to avoid having a (non-void!) reference to a void type) */
	typedef _TyTimerSampler::_TySample _TySample; /**< @brief time sample data type (dummy) */
};

/**
 *	@brief timer sampler traits (specialization for timing enabled)
 */
template <>
class CTimerSamplerTraits<true> {
public:
	/**
	 *	@brief configuration stored as enum
	 */
	enum {
		b_enable_timing = true /**< @brief timing enabled flag */
	};

	typedef CTimerSampler _TyTimerSampler; /**< @brief timer sampler type */
	typedef CTimerSampler &_TyTimerSamplerRef; /**< @brief timer sampler type reference (for passing the sampler to functions) */
	typedef _TyTimerSampler::_TySample _TySample; /**< @brief time sample data type */
};

#endif // !__TIMER_INCLUDED
