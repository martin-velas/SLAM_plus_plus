/*
								+--------------------------------+
								|                                |
								|  ***   Simple threading   ***  |
								|                                |
								|  Copyright © -tHE SWINe- 2008  |
								|                                |
								|            Thread.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __LAME_THREADS_INCLUDED
#define __LAME_THREADS_INCLUDED

/**
 *	@file Thread.h
 *	@brief simple multithreading primitives
 *	@date 2008
 *	@author -tHE SWINe-
 *
 *	@date 2008-12-21
 *
 *	removed some warning under g++, note this only works on BSD, linux gives error 12
 *	on pthread_create, whatever that one means. t_odo. (guess i'm not returning something)
 *
 *	@date 2009-06-01
 *
 *	added CMutex class for primitive thread synchronisation
 *
 *	@date 2009-07-07
 *
 *	added CMutex copy-constructor and copy-operator to protected functions (mutex instances
 *	shouldn't be copied. use pointers / references)
 *
 *	@date 2009-11-12
 *
 *	added CThread::Suspend() and CThread::Resume()
 *
 *	changed CThread::Run() in linux version (simplified it)
 *
 *	@todo - debug linux version
 *	http://www.linuxquestions.org/questions/programming-9/resume-and-suspend-pthreads-184535/
 *
 *	@date 2010-02-19
 *
 *	added CCurrentThreadHandle class, it currently doesn't implement any functions
 *	@todo - need to debug SuspendThread() under linux first, then implement it in CCurrentThreadHandle
 *	@todo - implement some unified thread priority control functions
 *	@todo - think about NUMA
 *
 *	@date 2010-10-25
 *
 *	Added Unused.h, decorated some function parameters as UNUSED().
 *
 *	@date 2010-10-29
 *
 *	Unified windows detection macro to "\#if defined(_WIN32) || defined(_WIN64)".
 *
 *	@date 2010-11-03
 *
 *	Added CSemaphore.
 *	Reformatted comments so they can be parsed by doxygen.
 *
 *	@date 2010-11-10
 *
 *	Added CProducerConsumerQueue.
 *
 *	@date 2011-11-01
 *
 *	Fixed a typo in CCurrentThreadHandle::Set_*Priority().
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2013-08-18
 *
 *	Added CSemaphore::TimedWait(), CSemaphore::n_Value() and Signal(int, int&),
 *	all of them remain mostly untested (seem to work on windows).
 *
 *	@date 2014-11-11
 *
 *	The thread destructor now explicitly kills the thread, avoiding possible orphaned threads.
 *
 */

#include "Unused.h"
#include "Integer.h"
#include <vector>
#include <functional> // std::equal_to

/**
 *	@brief virtual class for thread-attached runable object
 */
class CRunable {
public:
	/**
	 *	@brief this function is called from within the thread; thread code comes here
	 */
	virtual void Run() = 0;
};

#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h>
#else // _WIN32 || _WIN64
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <unistd.h>
#include <sys/types.h> // syscall(SYS_gettid);
#include <sys/syscall.h> // syscall(SYS_gettid);
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
#include <pthread_np.h> // not on Mac
#endif // __FreeBSD__ || __NetBSD__ || __OpenBSD__
#ifdef __FreeBSD__
#include <sys/param.h>
#include <sys/cpuset.h> // not on Mac
#endif // __FreeBSD__
#ifdef __APPLE__
//#include <cpuset.h> // not on Mac
#include <sched.h> // yes
#include <sys/param.h> // yes
#endif // __APPLE__
#endif // _WIN32 || _WIN64
#include <stdexcept>

/**
 *	@brief simple thread class
 */
class CThread {
protected:
	CRunable *m_p_runable;

#if defined(_WIN32) || defined(_WIN64)
	mutable HANDLE m_h_thread;
#else // _WIN32 || _WIN64
	pthread_t m_t_thread; // thread
	pthread_mutex_t m_t_running_mutex, m_t_suspend_mutex;
	pthread_cond_t m_t_suspend_cond;
	bool m_b_running; // thread state
#endif // _WIN32 || _WIN64

	class CEmptyRunable : public CRunable {
		virtual void Run() {}
	};
	static CEmptyRunable empty_runable;

public:
	/**
	 *	@brief default constructor; attaches empty runable object to a thread
	 */
	CThread();

	/**
	 *	@brief constructor, attaches runable object to a thread
	 *	@param[in] r_runable is reference to the runable object to be executed in the thread
	 */
	CThread(CRunable &r_runable);

	/**
	 *	@brief destructor; kills the thread forcibly
	 */
	~CThread();

	/**
	 *	@brief attaches a new runable, the thread must not be running
	 *	@param[in] r_runable is reference to the runable object to be executed in the thread
	 *	@return Returns true on success, false on failure.
	 */
	bool AttachRunable(CRunable &r_runable);

	/**
	 *	@brief gets current runable object
	 *	@return Returns reference to current runable object.
	 */
	CRunable &r_Runable();

	/**
	 *	@brief gets current runable object
	 *	@return Returns const reference to current runable object.
	 */
	const CRunable &r_Runable() const;

	/**
	 *	@brief starts the thread and returns immediately
	 *	@return Returns true on success, false on failure.
	 *	@note This fails in case the thread is already running.
	 */
	bool Start();

	/**
	 *	@brief starts the thread and waits untill it finishes
	 *	@return Returns true on success, false on failure.
	 *	@note This fails in case the thread is already running. Also note this
	 *		function is not thread-safe (the thread object doesn't change
	 *		it's state to running while in this function, and it can't be stopped)
	 */
	bool Run();

	/**
	 *	@brief suspends the thread (must be called from within the thread!)
	 *
	 *	@important Must be called from within the thread!
	 *
	 *	Suspends the thread undefinitely. Use Resume() to resume thread execution.
	 *		Suspended thread may be killed using Stop(); but waiting for suspended
	 *		thread to finish causes deadlock.
	 *
	 *	@return Returns true on success, false on failure.
	 *	@return Always returns false in case thread is not running.
	 *	@return Always returns true in case thread is running and is already suspended.
	 */
	bool Suspend();

	/**
	 *	@brief resumes the thread
	 *
	 *	Resumes execution of thread, previously stopped using Suspend().
	 *		Resuming running thread has no effect.
	 *
	 *	@return Returns true on success, false on failure.
	 *	@return Always returns false in case thread is not running.
	 *	@return Always returns true in case thread is running, and is not suspended.
	 *
	 *	@note For windows programmers - thread, executing Sleep() is woken up as well.
	 */
	bool Resume();

	/**
	 *	@brief gets current thread state
	 *	@return Returns true if thread is running, otherwise returns false.
	 */
	bool b_IsRunning() const;

	/**
	 *	@brief waits for the thread to end, if b_force_kill is set, it's ended forcibly
	 *	@param[in] b_force_kill specifies whether to forcibly end the thread (true),
	 *		or whether to wait until the thread finishes (false; default behavior)
	 *	@return Returns true if thread is stopped, returns false if thread didn't stop in given time, or on failure.
	 */
	bool Stop(bool b_force_kill = false);

	/**
	 *	@brief utility function; gets number of logical CPUs on the system
	 *	@return Returns number of logical CPUs, in case CPU count cannot be determined, returns -1.
	 */
	static size_t n_CPU_Num();

protected:
#if defined(_WIN32) || defined(_WIN64)
	static unsigned long __stdcall _run(void *p_arg); // run for windows
#else // _WIN32 || _WIN64
	static void *_run(void *p_arg); // run for linux
#endif // _WIN32 || _WIN64
	CThread(const CThread &UNUSED(r_thread)) {} /**< @brief can't copy threads this way, use pointers instead */
	const CThread &operator =(const CThread &UNUSED(r_thread)) { return r_thread; } /**< @brief can't copy threads this way, use pointers instead */
};

/**
 *	@brief virtual class for thread-attached runable object, containing it's thread
 */
class CRunable_Thread : public CRunable {
protected:
	CThread m_thread; /**< @brief thread for the runable */

public:
	/**
	 *	@brief constructor; attaches this runable to the thread it contains
	 */
	inline CRunable_Thread()
	{
		m_thread.AttachRunable(*this);
	}

	/**
	 *	@brief simple stert function; starts the associated thread
	 *	@return Returns true on success, false on failure. If the thread is already running, fails.
	 *	@note This can be overriden by the inheriting classes (add run parameters, ...).
	 */
	inline bool Start()
	{
		return m_thread.Start();
	}

	/**
	 *	@brief simple stop function; waits for the associated thread to finish
	 *	@return Returns true on success, false on failure. If the thread is not running, succeeds.
	 *	@note This can be overriden by the inheriting classes (add error checking, ...).
	 */
	inline bool WaitForFinish()
	{
		return m_thread.Stop(false);
	}
};

/**
 *	@brief virtual class for thread-attached runable object, containing it's thread, supports shallow copy
 *
 *	These objects can be e.g. stored in std::vector and the task parameters can be distributed
 *	via the copy constructor. This must be used with caution, as the thread itself is not copied.
 *	Therefore copying over a running runable might have unexpected consequences.
 *
 *	@remark Use only if you know what you are doing.
 */
class CRunable_Thread_ShallowCopy : public CRunable {
protected:
	CThread m_thread; /**< @brief thread for the runable */

public:
	/**
	 *	@brief constructor; attaches this runable to the thread it contains
	 */
	inline CRunable_Thread_ShallowCopy()
	{
		m_thread.AttachRunable(*this);
	}

	/**
	 *	@brief copy-constructor; attaches this runable to the thread it contains
	 *	@param[in] r_other is unused (the thread is not copied)
	 */
	inline CRunable_Thread_ShallowCopy(const CRunable_Thread_ShallowCopy UNUSED(&r_other))
	{
		m_thread.AttachRunable(*this);
	}

	/**
	 *	@brief copy-operator; attaches this runable to the thread it contains
	 *	@param[in] r_other is unused (the thread is not copied)
	 *	@return Returns reference to this.
	 */
	inline CRunable_Thread_ShallowCopy &operator =(const CRunable_Thread_ShallowCopy UNUSED(&r_other))
	{
		return *this;
	}

	/**
	 *	@brief simple stert function; starts the associated thread
	 *	@return Returns true on success, false on failure. If the thread is already running, fails.
	 *	@note This can be overriden by the inheriting classes (add run parameters, ...).
	 */
	inline bool Start()
	{
		return m_thread.Start();
	}

	/**
	 *	@brief simple stop function; waits for the associated thread to finish
	 *	@return Returns true on success, false on failure. If the thread is not running, succeeds.
	 *	@note This can be overriden by the inheriting classes (add error checking, ...).
	 */
	inline bool WaitForFinish()
	{
		return m_thread.Stop(false);
	}
};

/**
 *	@brief wrapper for current thread handle
 *
 *	Allows users to execute operations on current thread, without having
 *		it's CThread (such as process first thread).
 */
class CCurrentThreadHandle {
protected:
#if defined(_WIN32) || defined(_WIN64)
	mutable HANDLE m_h_thread;
#else // _WIN32 || _WIN64
	pthread_t m_t_thread; // thread
#endif // _WIN32 || _WIN64

public:
	/**
	 *	@brief default constructor
	 */
	CCurrentThreadHandle()
	{
#if defined(_WIN32) || defined(_WIN64)
		DuplicateHandle(GetCurrentProcess(), GetCurrentThread(),
			GetCurrentProcess(), &m_h_thread, 0, TRUE, DUPLICATE_SAME_ACCESS);
		// get thread handle (must duplicate, handle returned by
		// GetCurrentThread() can't be used to do some things)
#else // _WIN32 || _WIN64
		m_t_thread = pthread_self();
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief destructor
	 */
	~CCurrentThreadHandle()
	{
#if defined(_WIN32) || defined(_WIN64)
		CloseHandle(m_h_thread);
		// must return duplicated thread handle
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief gets current thread id, where the meaning is undefined and may not even be an integer
	 *	@return Returns current thread id.
	 *	@note This cannot be safely compared using the == operator, the meaning is system dependent.
	 *		However, on some systems, this can (and does) work quite well.
	 */
	static int n_Get_OpaqueId()
	{
#if defined(_WIN32) || defined(_WIN64)
		return GetCurrentThreadId();
#else // _WIN32 || _WIN64
		return (long)pthread_self(); // cast to long required for FreeBSD
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief gets current thread id (can be used as means of thread identification) as an integer
	 *	@return Returns current thread id.
	 *	@note Actual range of values of thread id's is system-dependent.
	 */
	static int n_Get_Id()
	{
#if defined(SYS_gettid)
		return syscall(SYS_gettid);
#elif defined(__NR_gettid)
		return syscall(__NR_gettid);
#else // SYS_gettid
		return n_Get_OpaqueId();
#endif // SYS_gettid
	}

	bool Set_AffinityMask32(uint32_t n_mask)
	{
#if defined(_WIN32) || defined(_WIN64)
		return SetThreadAffinityMask(m_h_thread, n_mask) != 0;
#else // _WIN32 || _WIN64
#if defined(__FreeBSD__)
		cpuset_t t_cpuset;
		CPU_ZERO(&t_cpuset);
		for(int i = 0; i < 32; ++ i) {
			if((n_mask >> i) & 1)
				CPU_SET(i, &t_cpuset); // todo - try to get rid of this
		}
		return !pthread_setaffinity_np(m_t_thread, sizeof(cpuset_t), &t_cpuset);
#elif defined(__NetBSD__)
		cpuset_t *cmask = cpuset_create();
		cpuset_zero(cmask);
		for(int i = 0; i < 32; ++ i) {
			if((n_mask >> i) & 1)
				cpuset_set(bitnum, cmask);
		}
		bool b_result = !pthread_setaffinity_np(m_t_thread, sizeof(cpuset_t), &t_cpuset);
		cpuset_destroy(cmask);
		return b_result;
#elif defined(__APPLE__)
		return true;
		// apparently this can't be done on OS X (see Thread Affinity API and thread_policy_set())
		// silently ignore this to not make existing software fail
#else // __FreeBSD__
		cpu_set_t t_cpuset;
		CPU_ZERO(&t_cpuset);
		for(int i = 0; i < 32; ++ i) {
			if((n_mask >> i) & 1)
				CPU_SET(i, &t_cpuset); // todo - try to get rid of this
		}
		return !pthread_setaffinity_np(m_t_thread, sizeof(cpu_set_t), &t_cpuset);
#endif // __FreeBSD__
#endif // _WIN32 || _WIN64
	}

	bool Set_HighPriority()
	{
#if defined(_WIN32) || defined(_WIN64)
		return SetThreadPriority(m_h_thread, THREAD_PRIORITY_HIGHEST) != 0;
#else // _WIN32 || _WIN64
		int n_policy;
		struct sched_param t_param;
		if(pthread_getschedparam(m_t_thread, &n_policy, &t_param))
			return false;
		// get scheduling policy for the thread

		int n_prio = sched_get_priority_max(n_policy);
		if(n_prio == -1)
			return false;
		// get maximal priority for that policy

#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__APPLE__)
		return true;
		// This function is missing on some platforms: glibc 2.3.6, MacOS X 10.3, FreeBSD 6.0,
		// NetBSD 3.0, OpenBSD 3.8, AIX 5.1, HP-UX 11, IRIX 6.5, OSF/1 5.1, Solaris 9, Cygwin, mingw, BeOS.
#else //  __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __APPLE__
		return !pthread_setschedprio(m_t_thread, n_prio - 1); // set priority one lower
		// set minimal priority
#endif // __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __APPLE__
#endif // _WIN32 || _WIN64
	}

	bool Set_LowPriority()
	{
#if defined(_WIN32) || defined(_WIN64)
		return SetThreadPriority(m_h_thread, THREAD_PRIORITY_LOWEST) != 0;
#else // _WIN32 || _WIN64
		int n_policy;
		struct sched_param t_param;
		if(pthread_getschedparam(m_t_thread, &n_policy, &t_param))
			return false;
		// get scheduling policy for the thread

		int n_prio = sched_get_priority_min(n_policy);
		if(n_prio == -1)
			return false;
		// get minimal priority for that policy

#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__APPLE__)
		return true;
		// This function is missing on some platforms: glibc 2.3.6, MacOS X 10.3, FreeBSD 6.0,
		// NetBSD 3.0, OpenBSD 3.8, AIX 5.1, HP-UX 11, IRIX 6.5, OSF/1 5.1, Solaris 9, Cygwin, mingw, BeOS.
#else //  __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __APPLE__
		return !pthread_setschedprio(m_t_thread, n_prio + 1); // set priority one higher
		// set minimal priority
#endif // __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __APPLE__
#endif // _WIN32 || _WIN64
	}

	bool Set_NormalPriority()
	{
#if defined(_WIN32) || defined(_WIN64)
		return SetThreadPriority(m_h_thread, THREAD_PRIORITY_NORMAL) != 0;
#else // _WIN32 || _WIN64
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__APPLE__)
		return true;
		// This function is missing on some platforms: glibc 2.3.6, MacOS X 10.3, FreeBSD 6.0,
		// NetBSD 3.0, OpenBSD 3.8, AIX 5.1, HP-UX 11, IRIX 6.5, OSF/1 5.1, Solaris 9, Cygwin, mingw, BeOS.
#else //  __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __APPLE__
		return !pthread_setschedprio(m_t_thread, 0); // set default priority (fixme?)
		// set minimal priority
#endif // __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __APPLE__
#endif // _WIN32 || _WIN64
	}

protected:
	CCurrentThreadHandle(const CCurrentThreadHandle &UNUSED(r_cth)) {} /**< @brief do not copy thread handles this way. use references / pointers */
	void operator =(const CCurrentThreadHandle &UNUSED(r_cth)) {} /**< @brief do not copy thread handles this way. use references / pointers */
};

#ifdef _OPENMP
#include <omp.h>

/**
 *	@def __MUTEX_USE_OPENMP_LOCK
 *	@brief if defined, enables the use of OpenMP lock instead of windows mutex (where available)
 */
//#define __MUTEX_USE_OPENMP_LOCK
#endif // _OPENMP

/**
 *	@brief simple mutex class
 */
class CMutex {
protected:
#if defined(_WIN32) || defined(_WIN64)
#ifdef __MUTEX_USE_OPENMP_LOCK
	omp_lock_t m_t_lock;
#else // __MUTEX_USE_OPENMP_LOCK
	HANDLE m_h_mutex;
	//bool m_b_status;
#endif // __MUTEX_USE_OPENMP_LOCK
#else // _WIN32 || _WIN64
	pthread_mutex_t m_t_mutex;
	bool m_b_status;
#endif // _WIN32 || _WIN64

public:
	/**
	 *	@brief default constructor; creates a new mutex
	 *	@note The mutex is initially unlocked.
	 *	@note It is advised to call b_Status() to see if mutex
	 *		was really created (might fail due to OS resource limits).
	 */
	CMutex();

	/**
	 *	@brief destructor; deletes mutex
	 */
	~CMutex();

	/**
	 *	@brief gets constructor result
	 *	@return Returns true if the mutex was successfuly created and can be used.
	 *	@note This doesn't reflect actual mutex state (locked / unlocked).
	 */
	bool b_Status() const;

	/**
	 *	@brief attempts to lock the mutex. in case mutex is already locked,
	 *		the calling thread is suspended until mutex owner calls Unlock()
	 *	@return Returns true on success, false on failure (doesn't reflect whether
	 *		the thread had to wait, or not. might fail because mutex was
	 *		deleted by another thread, ... shouldn't really happen).
	 */
	bool Lock();

	/**
	 *	@brief attempts to lock the mutex. in case mutex is already locked, returns immediately
	 *	@return Returns true in case mutex was successfuly locked, false in case the mutex
	 *		is already locked, or on failure (shouldn't really happen).
	 */
	bool TryLock();

	/**
	 *	@brief unlocks the mutex
	 *	@return Returns true on success, false on failure (such as mutex was locked by different
	 *		thread, and therefore cannot be unlocked, or if the mutex wasn't locked).
	 */
	bool Unlock();

protected:
	CMutex(const CMutex &UNUSED(r_mutex)) {} /**< @brief can't copy mutexes this way, use pointers */
	const CMutex &operator =(const CMutex &UNUSED(r_mutex)) { return *this; } /**< @brief can't copy mutexes this way, use pointers */
};

/**
 *	@brief simple semaphore class
 *	@note This hasn't been tested on linux / bsd yet.
 */
class CSemaphore {
protected:
#if defined(_WIN32) || defined(_WIN64)
	HANDLE m_h_semaphore;
#else // _WIN32 || _WIN64
	sem_t m_t_semaphore;
	bool m_b_status;
#endif // _WIN32 || _WIN64

public:
	/**
	 *	@brief default constructor; creates a new semaphore
	 *	@param[in] n_initial_count is the initial semaphore value
	 *	@note It is advised to call b_Status() to see if the semaphore
	 *		was really created (might fail due to OS resource limits).
	 */
	CSemaphore(int n_initial_count);

	/**
	 *	@brief destructor
	 */
	~CSemaphore();

	/**
	 *	@brief gets constructor result
	 *	@return Returns true if the semaphore was successfuly created and can be used, otherwise returns false.
	 *	@note This doesn't reflect semaphore state (signaled / unsignaled).
	 */
	bool b_Status() const;

	/**
	 *	@brief gets the current value of the semaphore
	 *	@return Returns the value of the semaphore on success, or INT_MAX on failure.
	 *	@note In Windows, this relies on some internal functionality. Where possible, this
	 *		should be replaced by Signal(int, int&) function (but that changes semaphore value).
	 */
	int n_Value() const;

	/**
	 *	@brief attempts to enter the semaphore. in case it's value is zero,
	 *		the calling thread is suspended until someone calls Signal()
	 *	@return Returns true on success, false on failure (doesn't reflect whether
	 *		the thread had to wait, or not. might fail because the semaphore was
	 *		deleted by another thread, ... shouldn't really happen).
	 */
	bool Wait();

	/**
	 *	@brief attempts to enter the semaphore. in case it's value is zero,
	 *		the calling thread is suspended until someone calls Signal()
	 *		or until the specified time elapses
	 *
	 *	@param[out] r_b_locked is set if the semaphore was successfully locked,
	 *		or cleared if the timeout elapsed
	 *	@param[in] n_wait_miliseconds is time to wait, in miliseconds
	 *
	 *	@return Returns true on success, false on failure (doesn't reflect whether
	 *		the thread had to wait, or not. might fail because the semaphore was
	 *		deleted by another thread, ... shouldn't really happen).
	 */
	inline bool TimedWait(bool &r_b_locked, int n_wait_miliseconds)
	{
		return TimedWait(r_b_locked, n_wait_miliseconds / 1000, n_wait_miliseconds % 1000);
	}

	/**
	 *	@brief attempts to enter the semaphore. in case it's value is zero,
	 *		the calling thread is suspended until someone calls Signal()
	 *		or until the specified time elapses
	 *
	 *	@param[out] r_b_locked is set if the semaphore was successfully locked,
	 *		or cleared if the timeout elapsed
	 *	@param[in] n_wait_seconds is time to wait, in seconds
	 *	@param[in] n_wait_nanoseconds is time to wait, in nanoseconds
	 *
	 *	@return Returns true on success, false on failure (doesn't reflect whether
	 *		the thread had to wait, or not. might fail because the semaphore was
	 *		deleted by another thread, ... shouldn't really happen).
	 */
	bool TimedWait(bool &r_b_locked, int n_wait_seconds, long n_wait_nanoseconds);

	/**
	 *	@brief attempts to enter the semaphore, in case it's value is zero (would block), returns immediately
	 *	@return Returns true in case mutex was successfuly locked,
	 *		otherwise returns false; also returns false on failure (shouldn't really happen).
	 */
	bool TryWait();

	/**
	 *	@brief Unlocks the semaphore, increments it's value by 1, returns immediately
	 *	@return Returns true on success, false on failure (such as the semaphore being deleted by another thread).
	 */
	bool Signal();

	/**
	 *	@brief Unlocks the semaphore, increments it's value by a given count, returns immediately
	 *	@param[in] n_signal_num is number of pending threads to signal (must be nonzero positive)
	 *	@return Returns true on success, false on failure (such as the semaphore being deleted by another thread).
	 */
	bool Signal(int n_signal_num);

	/**
	 *	@brief Unlocks the semaphore, increments it's value by a given count, returns immediately
	 *
	 *	@param[in] n_signal_num is number of pending threads to signal (must be nonzero positive)
	 *	@param[out] r_n_previous_value is filled with the semaphore value before the operation
	 *
	 *	@return Returns true on success, false on failure (such as the semaphore being deleted by another thread).
	 */
	bool Signal(int n_signal_num, int &r_n_previous_value);

protected:
	CSemaphore(const CSemaphore &UNUSED(r_sema)) {} /*<< @brief can't copy semaphores this way, use pointers */
	const CSemaphore &operator =(const CSemaphore &UNUSED(r_sema)) { return *this; } /*<< @brief can't copy semaphores this way, use pointers */
};

/**
 *	@brief a simple thread-safe queue for producer-consumer scenarios
 *
 *	The correct usage is as follows:
 *	@code
 *	void Producer(CProducerConsumerQueue<TWorkItem> &r_queue)
 *	{
 *		TWorkItem t_wi;
 *		while(GetMoreWork(&t_wi))
 *			r_queue.Put(t_wi); // mind some error-checking here
 *		r_queue.Signal_Finished(); // and here
 *	}
 *
 *	void Consumer(CProducerConsumerQueue<TWorkItem> &r_queue)
 *	{
 *		TWorkItem t_wi;
 *		while(r_queue.Get(t_wi))
 *			ProcessWorkItem(t_wi);
 *		if(!r_queue.b_Finished())
 *			Error(); // there are still data in the queue, but Get() failed for some reason ...
 *	}
 *	@endcode
 *
 *	@tparam TWorkItem is data type of a single unit of work
 *	@note There may be multiple producers or consumers, all variants of Get() and Put() are both thread-safe.
 */
template <class TWorkItem>
class CProducerConsumerQueue {
protected:
	CSemaphore m_full_count_sema, m_free_count_sema;
	CMutex m_buffer_access_mutex;

	size_t m_n_free_index; // index of first free work-item
	size_t m_n_full_index; // index of first full work-item
	std::vector<TWorkItem> m_queue_data;
	size_t m_n_queue_size;

	bool m_b_finished;

public:
	/**
	 *	@brief default constructor; initializes the queue
	 *	@param[in] n_queue_size is size of the queue, in work-items
	 *	@note This constructor may fail (due to OS limits, or low free memory),
	 *		it is therefore reccommended to call b_Status() afterwards.
	 */
	inline CProducerConsumerQueue(size_t n_queue_size)
		:m_full_count_sema(0), m_free_count_sema(int(n_queue_size)), m_n_free_index(0), m_n_full_index(0), m_b_finished(false)
	{
		_ASSERTE(n_queue_size < INT_MAX); // CSemaphore constructor
		try {
			_ASSERTE(n_queue_size < SIZE_MAX);
			m_queue_data.resize(n_queue_size + 1); // +1
			m_n_queue_size = m_queue_data.size();
		} catch(std::bad_alloc&) {
			m_n_queue_size = 0;
		}
	}

	/**
	 *	@brief gets constructor result
	 *	@return Returns true if constructor succeeded and the queue is ready to be used, otherwise returns false.
	 *	@note This doesn't reflect any kind of queue state (eg. empty / full / finished queue).
	 */
	inline bool b_Status() const
	{
		_ASSERTE(m_n_queue_size == m_queue_data.size());
		return m_n_queue_size && m_full_count_sema.b_Status() &&
			m_free_count_sema.b_Status() && m_buffer_access_mutex.b_Status();
	}

	/**
	 *	@brief gets queue size
	 *	@return returns queue size, in work-items
	 */
	inline size_t n_Size() const
	{
		return m_n_queue_size - 1;
	}

	/**
	 *	@brief gets number of items, currently in the queue
	 *	@return Returns number of items, currently in the queue.
	 *	@note This function is not thread-safe, and the result may thus be inaccurate.
	 */
	inline size_t n_EnqueuedItem_Num() const
	{
		size_t n_head = m_n_full_index, n_tail = m_n_free_index;
		return (n_tail >= n_head)? n_tail - n_head : n_tail + m_n_queue_size - n_head; // @t_odo - debug this
		// queue item num:
		//
		//		0123456789
		// tail * 
		// head * 0 = tail - head
		// tail  *
		// head * 1 = tail - head
		// tail *
		// head          * 1 = tail + 10 - head = 0+10-9
		// tail  *
		// head         * 3 = tail + 10 - head = 1+10-8
		// tail  *
		// head   * 9 = tail + 10 - head = 1 + 10 - 2
	}

	/**
	 *	@brief determines whether is the queue empty
	 *	@return Returns true if the queue is empty, otherwise returns false.
	 *	@note This does not signal consumers there will be no more data, b_Finished() is used to do that.
	 *	@note This function always returns immediately.
	 */
	inline bool b_Empty() const
	{
		return m_n_full_index == m_n_free_index;
	}

	/**
	 *	@brief determines whether is the queue full
	 *	@return Returns true if the queue is full, otherwise returns false.
	 *	@note This function always returns immediately.
	 */
	inline bool b_Full() const
	{
		return m_n_full_index == n_NextIndex(m_n_free_index); // f_ixme? yes, this is correct (full = head!)
	}

	/**
	 *	@brief signals the producer is done generating data
	 *	@return Returns true on success, false on failure (shouldn't really happen,
	 *		provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note This function should only be called by the producer, and only once.
	 */
	inline bool Signal_Finished()
	{
		m_b_finished = true;
		return m_full_count_sema.Signal();
	}

	/**
	 *	@brief determines whether the finished flag was raised by the producer
	 *		(there will be no more data once the queue is empty)
	 *	@return Returns true in case the producer raised the finished flag (by calling Signal_Finished()),
	 *		otherwise returns false.
	 *	@note There still may be data in the queue which must be processed. The correct way to use this queue
	 *		is to call Get() until it fails, then calling b_Finished(). If it returns true, consumer can quit.
	 *		If, on the other hand, it returns false, there was an error.
	 */
	inline bool b_Finished() const
	{
		return b_Empty() && m_b_finished;
	}

	/**
	 *	@brief puts a single work-item to the queue
	 *	@param[in] t_wi is the work-item
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *	@note Returns immediately if the queue is not full, or blocks until a consumer calls Get() and makes some space.
	 */
	inline bool Put(TWorkItem t_wi)
	{
		if(!m_free_count_sema.Wait())
			return false;
		// enter the "empty" semaphore

		return Naked_Put(t_wi);
	}

	/**
	 *	@brief tries to put a single work-item to the queue, in the specified time
	 *
	 *	@param[out] r_b_put_work_item is the result of the put (set if put, cleared if timed out)
	 *	@param[in] t_wi is the work-item
	 *	@param[in] n_timeout_msec is time to put the item in the queue, in milliseconds
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note Returns immediately if the queue is not full, or blocks until either a consumer calls Get()
	 *		and makes some space, or until the timeout is exceeded.
	 */
	inline bool TimedPut(bool &r_b_put_work_item, TWorkItem t_wi, int n_timeout_msec)
	{
		if(!m_free_count_sema.TimedWait(r_b_put_work_item, n_timeout_msec))
			return false;
		if(!r_b_put_work_item)
			return true;
		// enter the "empty" semaphore

		return Naked_Put(t_wi);
	}

	/**
	 *	@brief tries to put a single work-item to the queue
	 *	@param[in] t_wi is the work-item
	 *	@return Returns true in case the work-item was successfuly enqueued, false in case the queue was full,
	 *		or on failure (shouldn't really happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note Always returns immediately.
	 */
	inline bool TryPut(TWorkItem t_wi)
	{
		if(!m_free_count_sema.TryWait())
			return false;
		// enter the "empty" semaphore

		return Naked_Put(t_wi);
	}

	/**
	 *	@brief gets a single work-item from the queue
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *	@return Returns true on success, false if there's no more work (call b_Finished() to determine the cause)
	 *		or on failure (shouldn't really happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note Returns immediately if the queue is not empty, or blocks until a producer calls Put() or Signal_Finished().
	 */
	inline bool Get(TWorkItem &r_t_wi)
	{
		if(!m_full_count_sema.Wait())
			return false;
		// enter the "fill" semaphore

		return Naked_Get(r_t_wi);
	}

	/**
	 *	@brief tries to get a single work-item from the queue, in the specified time
	 *
	 *	@param[out] r_b_received_work_item is the result of the get (set if got one, cleared if timed out)
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *	@param[in] n_timeout_msec is time to put the item in the queue, in milliseconds
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note Returns immediately if the queue is not empty, or blocks until either
	 *		a producer calls Put() or Signal_Finished(), or until the timeout is exceeded.
	 */
	inline bool TimedGet(bool &r_b_received_work_item, TWorkItem &r_t_wi, int n_timeout_msec)
	{
		if(!m_full_count_sema.TimedWait(r_b_received_work_item, n_timeout_msec))
			return false;
		if(!r_b_received_work_item)
			return true;
		// enter the "fill" semaphore

		return Naked_Get(r_t_wi);
	}

	/**
	 *	@brief tries to get a single work-item from the queue
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *	@return Returns true in case work-item was successfuly obtained, false in case the queue was empty,
	 *		if there's no more work (call b_Finished() to determine the cause), or on failure (shouldn't
	 *		really happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note Always returns immediately.
	 */
	inline bool TryGet(TWorkItem &r_t_wi)
	{
		if(!m_full_count_sema.TryWait())
			return false;
		// enter the "fill" semaphore

		return Naked_Get(r_t_wi);
	}

protected:
	/**
	 *	@brief puts a single work-item to the queue
	 *
	 *	@param[in] t_wi is the work-item
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note This requires waiting for the m_full_count_sema mutex before this is called.
	 *	@note Always returns immediately.
	 */
	bool Naked_Put(TWorkItem t_wi)
	{
		//m_free_count_sema.Wait(); // this is called just before calling this function
		// enter the "empty" semaphore

		if(!m_buffer_access_mutex.Lock())
			return false;
		// lock buffer access mutex

		_ASSERTE(!b_Full()); // must not be full; if this triggers, someone forgot to wait for m_free_count_sema
		_ASSERTE(m_n_free_index >= 0 && m_n_free_index < m_queue_data.size());
		m_queue_data[m_n_free_index] = t_wi; // if this throws, we lose the mutex, this takes assumptions about the user types
		m_n_free_index = n_NextIndex(m_n_free_index);
		// add work-item to the queue

		return m_buffer_access_mutex.Unlock() && m_full_count_sema.Signal();
		// unlock buffer access mutex and signal the "fill" semaphore
	}

	/**
	 *	@brief gets a single work-item from the queue
	 *
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *
	 *	@return Returns true in case work-item was successfuly obtained, false in case there's no
	 *		more work (call b_Finished() to determine the cause), or on failure (shouldn't really
	 *		happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *
	 *	@note This requires waiting for the m_full_count_sema mutex before this is called.
	 *	@note Always returns immediately.
	 */
	bool Naked_Get(TWorkItem &r_t_wi)
	{
		//m_full_count_sema.Wait(); // this is called just before calling this function
		// enter the "fill" semaphore

		if(!m_buffer_access_mutex.Lock())
			return false;
		// lock buffer access mutex

		if(m_b_finished && b_Empty()) {
			m_buffer_access_mutex.Unlock();
			m_full_count_sema.Signal();
			// signal the "fill" semaphore in case there is more consumers accessing the queue to free the next one
			return false;
		}
		// handle finished state

		_ASSERTE(!b_Empty()); // must not be empty; if this triggers, someone forgot to wait for m_full_count_sema
		_ASSERTE(m_n_full_index >= 0 && m_n_full_index < m_queue_data.size());
		r_t_wi = m_queue_data[m_n_full_index]; // if this throws, we lose the mutex, this takes assumptions about the user types
		m_n_full_index = n_NextIndex(m_n_full_index);
		// get work-item from the queue

		return m_buffer_access_mutex.Unlock() && m_free_count_sema.Signal();
		// unlock buffer access mutex and signal the "empty" semaphore
	}

	/**
	 *	@brief indexing function for a circular buffer
	 *	@param[in] n_index is index of element in the queue
	 *	@return Returns index of the next element in the queue.
	 */
	inline size_t n_NextIndex(size_t n_index) const
	{
		_ASSERTE(n_index < SIZE_MAX);
		return (n_index + 1) % m_n_queue_size;
	}

private:
	CProducerConsumerQueue(const CProducerConsumerQueue &r_other); // no-copy
	CProducerConsumerQueue &operator =(const CProducerConsumerQueue &r_other); // no-copy
};


/**
 *	@brief default adjacency predicate
 *	@tparam T is a type of an item, must have operator ++ and ==
 */
template <class T>
class CDefaultAdjacencyPredicate {
public:
	/**
	 *	@brief adjacency predicate
	 *
	 *	@param[in] a is the first item
	 *	@param[in] b is the second item
	 *
	 *	@return Returns true if (a, b) forms adjacent ordered pair, otherwise returns false.
	 */
	inline bool operator ()(T a, T b) const
	{
		return ++ a == b;
	}
};

/**
 *	@brief ordered queue, relying on adjacency predicate only
 *
 *	This is a fixed-size constructor-allocated queue, which orders the sequence.
 *	It requires the sequence to be contiguous, and it requires to have an increment
 *	and comparison operations defined on the type it works with. Notably, it does
 *	not require less-than comparison, making it suitable for ordering modulo
 *	sequences, etc.
 *
 *	@tparam T is a type of item to be queued
 *	@tparam TIndex is a type of item counter, used to identify the next item in order
 *		(by default the same as T)
 *	@tparam CAdjacencyPredicate is adjacency predicate (if and only if (a, b) forms an
 *		adjacent ordered pair, then <tt>CAdjacencyPredicate()(a, b)</tt> returns true)
 *	@tparam CEqualityPredicate is equality predicate, used to test values of T and TIndex
 *		for equality (in this order)
 */
template <class T, class TIndex = T,
	class CAdjacencyPredicate = CDefaultAdjacencyPredicate<T>,
	class CEqualityPredicate = std::equal_to<T> >
class CAdjacentContiguousOrderedQueue {
protected:
	std::vector<T> m_deque_forest; /**< @brief forest of double-ended queues */
	std::vector<size_t> m_deque_sizes; /**< @brief list of double-ended queue sizes */
    TIndex m_t_next; /**< @brief value of the next item to be popped */
	std::vector<T> m_splice_temp; /**< @brief temporary buffer for splicing */
	size_t m_n_max_size; /**< @brief queue size limit (then it would reallocate and Push() would potentially throw std::bad_alloc) */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] n_queue_size is queue size, in elements
	 *	@param[in] t_first_value is the first value of the order (the first value to t_Pop())
	 *
	 *	@note This constructor does not throw, if there was not enough memory for the queue,
	 *		its n_Capacity() will be null.
	 */
    inline CAdjacentContiguousOrderedQueue(size_t n_queue_size,
		TIndex t_first_value = TIndex())
        :m_t_next(t_first_value), m_n_max_size(n_queue_size)
    {
		try {
			m_deque_forest.reserve(n_queue_size);
			m_deque_sizes.reserve(n_queue_size);
			if(n_queue_size > 1) // otherwise don't reserve anything
				m_splice_temp.reserve(n_queue_size - 1); // one less suffices
		} catch(std::bad_alloc&) {
			m_n_max_size = 0; // marks error
		}
	}

	/**
	 *	@brief gets number of items that can be pushed into the queue
	 *	@return Returns number of items that can be pushed into the queue.
	 */
	inline size_t n_Capacity() const
	{
		return m_n_max_size;
	}

	/**
	 *	@brief gets number of deques
	 *	@return Returns number of deques (double ended queues).
	 *	@note This is more of a performance indicator, as t_Pop() runs in O(N) in n_Deque_Num().
	 */
	inline size_t n_Deque_Num() const
	{
		return m_deque_sizes.size();
	}

	/**
	 *	@brief gets number of items, currently in the queue
	 *	@return Returns number of items, currently in the queue.
	 */
	inline size_t n_Size() const
	{
		return m_deque_forest.size();
	}

	/**
	 *	@brief determines whether is the queue empty
	 *	@return Returns true if the queue is empty, otherwise returns false.
	 */
	inline bool b_Empty() const
	{
		return m_deque_forest.empty();
	}

	/**
	 *	@brief determines whether is the queue full
	 *	@return Returns true if the queue is full, otherwise returns false.
	 */
	inline bool b_Full() const
	{
		return m_deque_forest.size() == m_n_max_size;
	}

	/**
	 *	@brief gets the next index to be popped
	 *	@return Returns the next index in the adjacent order to be popped.
	 */
	inline TIndex t_Next() const
	{
		return m_t_next;
	}

	/**
	 *	@brief pushes an item to the queue
	 *	@param[in] t_item is item to be inserted in this queue
	 *	@return Returns true on success, false on failure (the queue is full).
	 *	@note Filling the queue with elements, none of which is equal to t_Next()
	 *		will render the queue useless once it is filled - it will be unable
	 *		to t_Pop() and at the same time unable to Push().
	 */
    inline bool Push(T t_item)
	{
		size_t n_increment; // unused
		return Push(t_item, n_increment);
	}

	/**
	 *	@brief pushes an item to the queue
	 *
	 *	@param[in] t_item is item to be inserted in this queue
	 *	@param[out] r_n_poppable_increment is change in the number of items
	 *		that can be popped (see n_Poppable_Num())
	 *
	 *	@return Returns true on success, false on failure (the queue is full).
	 *
	 *	@note Filling the queue with elements, none of which is equal to t_Next()
	 *		will render the queue useless once it is filled - it will be unable
	 *		to t_Pop() and at the same time unable to Push().
	 */
    bool Push(T t_item, size_t &r_n_poppable_increment)
    {
		if(b_Full())
			return false;
		// can't push in a full queue (would realloc std::vectors and potentially
		// run out of memory, otherwise the below algorithm will fare well)

		size_t n_prev_queue_begin, n_next_queue_end, n_prev_queue, n_next_queue;
		if(Find_Adjacent(t_item, n_prev_queue_begin, n_next_queue_end, n_prev_queue, n_next_queue)) {
			if(n_next_queue != size_t(-1) && n_prev_queue != size_t(-1)) {
				if(CEqualityPredicate()(m_deque_forest[n_next_queue_end - m_deque_sizes[n_next_queue]], m_t_next))
					r_n_poppable_increment = m_deque_sizes[n_prev_queue] + 1; // added so many to the top size
				else if(CEqualityPredicate()(t_item, m_t_next))
					r_n_poppable_increment = m_deque_sizes[n_next_queue] + m_deque_sizes[n_prev_queue] + 1; // added the next poppable to the joined queue
				else
					r_n_poppable_increment = 0; // no change
				// calculate poppable increment

				Splice(t_item, n_prev_queue_begin, n_next_queue_end, n_prev_queue, n_next_queue);
				// this element joins two previously disconnected deques
			} else if(n_next_queue != size_t(-1)) {
				if(CEqualityPredicate()(m_deque_forest[n_next_queue_end - m_deque_sizes[n_next_queue]], m_t_next))
					r_n_poppable_increment = 1; // added 1 to the queue, connected to this queue
				else if(CEqualityPredicate()(t_item, m_t_next))
					r_n_poppable_increment = m_deque_sizes[n_next_queue] + 1; // added the next poppable to the existing queue
				else
					r_n_poppable_increment = 0; // no change
				// calculate poppable increment

				_ASSERTE(m_deque_forest.capacity() > m_deque_forest.size()); // so that the below insert won't_item allocate / throw std::bad_alloc
				m_deque_forest.insert(m_deque_forest.begin() + n_next_queue_end, 1, t_item);
				++ m_deque_sizes[n_next_queue];
				// add at the end of a deque
			} else /*if(n_prev_queue != size_t(-1))*/ {
				_ASSERTE(n_prev_queue != size_t(-1));
				_ASSERTE(!CEqualityPredicate()(m_deque_forest[n_prev_queue_begin], m_t_next)); // prepending the begin, cannot be possibly the next poppable
				if(CEqualityPredicate()(t_item, m_t_next))
					r_n_poppable_increment = m_deque_sizes[n_prev_queue] + 1; // added the next poppable to the existing queue
				else
					r_n_poppable_increment = 0;
				// calculate poppable increment

				_ASSERTE(m_deque_forest.capacity() > m_deque_forest.size()); // so that the below insert won't_item allocate / throw std::bad_alloc
				m_deque_forest.insert(m_deque_forest.begin() + n_prev_queue_begin, 1, t_item);
				++ m_deque_sizes[n_prev_queue];
				// add at the front of a deque
			}
		} else {
			r_n_poppable_increment = (CEqualityPredicate()(t_item, m_t_next))? 1 : 0; // not connected to anything
			_ASSERTE(m_deque_forest.capacity() > m_deque_forest.size()); // so that the below insert won't_item allocate / throw std::bad_alloc
			m_deque_forest.push_back(t_item);
			_ASSERTE(m_deque_sizes.capacity() > m_deque_sizes.size()); // so that the below insert won't_item allocate / throw std::bad_alloc
			m_deque_sizes.push_back(1);
			// add as a new deque with a single item
		}

#ifdef _DEBUG
		_ASSERTE(!m_deque_sizes.size() == !m_deque_forest.size());
		_ASSERTE(m_deque_sizes.size() <= m_deque_forest.size());
		_ASSERTE(m_deque_forest.size() <= m_n_max_size);
		for(size_t i = 0, n = m_deque_sizes.size(), m = m_deque_forest.size(), b = 0; i < n; ++ i) {
			size_t e = b + m_deque_sizes[i];
			_ASSERTE(b < e);
			_ASSERTE(e <= m_deque_forest.size());

			for(++ b; b < e; ++ b)
				_ASSERTE(b_Is_Prev(m_deque_forest[b - 1], m_deque_forest[b]));
			// make sure that the queue is sorted

			_ASSERTE(i + 1 < n || e == m);
			// make sure the queues cover all of the m_deque_forest

			_ASSERTE(b == e);
		}
#endif // _DEBUG
		// integrity check

		return true;
    }

	/**
	 *	@brief gets the number of poppable items
	 *
	 *	@return Returns the number of items that can be popped from the queue, or 0 if none.
	 *
	 *	@note This is a different value than n_Size(), although n_Size() is an upper bound to this.
	 *	@note This takes up to O(n) to calculate.
	 */
	size_t n_Poppable_Num() const
	{
		for(size_t i = 0, n = m_deque_sizes.size(), b = 0; i < n; ++ i) {
			if(CEqualityPredicate()(m_deque_forest[b], m_t_next))
				return m_deque_sizes[i];
			b += m_deque_sizes[i];
		}
		return 0;
	}

	/**
	 *	@brief pops the next item in order from the queue
	 *
	 *	@return Returns value of the next item in order.
	 *
	 *	@note This function throws std::runtime_error if the
	 *		next item in order is not present in the queue.
	 *	@note This takes up to O(n).
	 */
    inline T t_Pop() // throw(std::runtime_error)
    {
		for(size_t i = 0, n = m_deque_sizes.size(), b = 0; i < n; ++ i) {
			if(CEqualityPredicate()(m_deque_forest[b], m_t_next)) { // the m_t_next item can be found at the beginning of one of the deques
				++ m_t_next;
				T value = m_deque_forest[b];
				m_deque_forest.erase(m_deque_forest.begin() + b);
				if(!(-- m_deque_sizes[i]))
					m_deque_sizes.erase(m_deque_sizes.begin() + i);
				return value;
			}
			b += m_deque_sizes[i];
		}
		_ASSERTE(!n_Poppable_Num()); // there is nothing to pop, n_Poppable_Num() must be 0
        throw std::runtime_error("no next element"); // this is programmer's fault, should not call this when n_Poppable_Num() is 0
		//return m_deque_forest.front(); // avoid T having to have a default constructor (but this can't be executed)
    }

protected:
	/**
	 *	@brief finds deques with the first or the last item adjacent to a new item
	 *
	 *	@param[in] t_item is item to be inserted in this queue (but not inserted in this function)
	 *	@param[out] r_n_prev_queue_begin is zero-based index of the first item of
	 *		the second deque (points in m_deque_forest, only valid if n_prev_queue != size_t(-1))
	 *	@param[out] r_n_next_queue_end is zero-based index of one past the last item of the first
	 *		deque (points in m_deque_forest, only valid if n_next_queue != size_t(-1))
	 *	@param[out] r_n_prev_queue is zero-based indec of the second queue (will end up at the end
	 *		of the spliced deque), or size_t(-1) if none found (points in m_deque_sizes)
	 *	@param[out] r_n_next_queue is zero-based indec of the first queue (will end up at the
	 *		beginning of the spliced deque), or size_t(-1) if none found (points in m_deque_sizes)
	 */
	bool Find_Adjacent(T t_item, size_t &r_n_prev_queue_begin, size_t &r_n_next_queue_end,
		size_t &r_n_prev_queue, size_t &r_n_next_queue) const
	{
		_ASSERTE(!m_deque_sizes.size() == !m_deque_forest.size());
		_ASSERTE(m_deque_sizes.size() <= m_deque_forest.size());
		_ASSERTE(m_deque_forest.size() < m_n_max_size);
		r_n_prev_queue = size_t(-1);
		r_n_next_queue = size_t(-1);
		for(size_t i = 0, n = m_deque_sizes.size(), m = m_deque_forest.size(), b = 0, e; i < n; ++ i, b = e) {
			e = b + m_deque_sizes[i];
			_ASSERTE(b < e && e <= m);
			if(b_Is_Prev(t_item, m_deque_forest[b])) {
				_ASSERTE(r_n_prev_queue == size_t(-1));
				r_n_prev_queue = i;
				r_n_prev_queue_begin = b;

				_ASSERTE(r_n_next_queue == size_t(-1));
				for(++ i, b = e; i < n; ++ i, b = e) {
					e = b + m_deque_sizes[i];
					_ASSERTE(b < e && e <= m);
					if(b_Is_Next(t_item, m_deque_forest[e - 1])) {
						r_n_next_queue = i;
						r_n_next_queue_end = e;
						break;
					}
				}
				_ASSERTE(r_n_next_queue != size_t(-1) || b == m); // not sure why check, but yes
				_ASSERTE(r_n_next_queue != r_n_prev_queue); // cannot be simulatenously before and after the same deque
				return true;
			}
			if(b_Is_Next(t_item, m_deque_forest[e - 1])) {
				_ASSERTE(r_n_next_queue == size_t(-1));
				r_n_next_queue = i;
				r_n_next_queue_end = e;

				_ASSERTE(r_n_prev_queue == size_t(-1));
				for(++ i, b = e; i < n; ++ i, b = e) {
					e = b + m_deque_sizes[i];
					_ASSERTE(b < e && e <= m);
					if(b_Is_Prev(t_item, m_deque_forest[b])) {
						r_n_prev_queue = i;
						r_n_prev_queue_begin = b;
						break;
					}
				}
				_ASSERTE(r_n_next_queue != r_n_prev_queue); // cannot be simulatenously before and after the same deque
				return true;
			}
		}
		_ASSERTE(r_n_next_queue == r_n_prev_queue && r_n_prev_queue == size_t(-1)); // is nowhere
		// find queues where the new item could be added

		return false;
		// no adjacent
	}

	/**
	 *	@brief splíces two deques and inserts a new element between them
	 *
	 *	@param[in] t_item is item that joins two deques
	 *	@param[in] n_prev_queue_begin is zero-based index of the first item of
	 *		the second deque (points in m_deque_forest, only valid if n_prev_queue != size_t(-1))
	 *	@param[in] n_next_queue_end is zero-based index of one past the last item of the first
	 *		deque (points in m_deque_forest, only valid if n_next_queue != size_t(-1))
	 *	@param[in] n_prev_queue is zero-based indec of the second queue (will end up at the end
	 *		of the spliced deque), or size_t(-1) if none found (points in m_deque_sizes)
	 *	@param[in] n_next_queue is zero-based indec of the first queue (will end up at the
	 *		beginning of the spliced deque), or size_t(-1) if none found (points in m_deque_sizes)
	 */
	void Splice(T t_item, size_t n_prev_queue_begin,
		size_t n_next_queue_end, size_t n_prev_queue, size_t n_next_queue)
	{
		_ASSERTE(n_prev_queue < m_deque_sizes.size());
		_ASSERTE(n_next_queue < m_deque_sizes.size());
		_ASSERTE(n_prev_queue_begin < m_deque_forest.size());
		_ASSERTE(n_next_queue_end <= m_deque_forest.size());
		// make sure the indices are valid

		_ASSERTE(b_Is_Next(t_item, m_deque_forest[n_next_queue_end - 1]));
		_ASSERTE(b_Is_Prev(t_item, m_deque_forest[n_prev_queue_begin]));
		// make sure those are indeed both adjacent

		if(n_next_queue_end != n_prev_queue_begin) {
			const size_t n_prev_begin = n_prev_queue_begin;
			const size_t n_prev_size = m_deque_sizes[n_prev_queue];
			const size_t n_prev_end = n_prev_begin + n_prev_size;

			_ASSERTE(m_splice_temp.capacity() >= n_prev_size + 1); // so that the next resize won't_item allocate / throw std::bad_alloc
			m_splice_temp.resize(n_prev_size + 1);
			m_splice_temp.front() = t_item; // !!
			std::copy(m_deque_forest.begin() + n_prev_begin,
				m_deque_forest.begin() + n_prev_end, m_splice_temp.begin() + 1);
			m_deque_forest.erase(m_deque_forest.begin() + n_prev_begin,
				m_deque_forest.begin() + n_prev_end);
			m_deque_sizes.erase(m_deque_sizes.begin() + n_prev_queue);
			// move the previous deque to the temporary splice buffer

			if(n_next_queue > n_prev_queue) {
				-- n_next_queue;
				n_next_queue_end -= n_prev_size;
			}
			// fixup indices

			_ASSERTE(m_deque_forest.capacity() >= m_deque_forest.size() + m_splice_temp.size()); // so that the next resize won't_item allocate / throw std::bad_alloc
			m_deque_forest.insert(m_deque_forest.begin() + n_next_queue_end,
				m_splice_temp.begin(), m_splice_temp.end());
			m_splice_temp.clear();
			m_deque_sizes[n_next_queue] += n_prev_size + 1; // update the size
			// this element joins two previously disconnected queues
		} else {
			m_deque_forest.insert(m_deque_forest.begin() + n_next_queue_end, 1, t_item);
			m_deque_sizes[n_next_queue] += m_deque_sizes[n_prev_queue] + 1; // update the size
			_ASSERTE(n_prev_queue > n_next_queue); // this way the erase below needs to shift less elements
			m_deque_sizes.erase(m_deque_sizes.begin() + n_prev_queue);
			// the queues are already next to each other in the buffer, no need for shuffling
		}
	}

	/**
	 *	@brief a is previous to b predicate
	 *
	 *	@param[in] a is the first item to be compared
	 *	@param[in] b is the second item to be compared
	 *
	 *	@return Returns true if (a, b) makes an adjacent ordered pair, otherwise returns false.
	 */
	static inline bool b_Is_Prev(T a, T b)
	{
		return CAdjacencyPredicate()(a, b);
	}

	/**
	 *	@brief a is next to b predicate
	 *
	 *	@param[in] a is the first item to be compared
	 *	@param[in] b is the second item to be compared
	 *
	 *	@return Returns true if (b, a) makes an adjacent ordered pair, otherwise returns false.
	 */
	static inline bool b_Is_Next(T a, T b)
	{
		return b_Is_Prev(b, a);
	}
};

/**
 *	@brief an ordered thread-safe queue for producer-consumer scenarios
 *
 *	This is used for ordering the work produced by several producers,
 *	so that it arrives in a ascending id order to a consumer, typical
 *	in parallel stream processing.
 *
 *	Care must be taken to limit the number of work items in circulation
 *	to avoid situation where the queue is filled with items, but not
 *	including the next item to be consumed. This will result in a deadlock
 *	as the consumer will wait for this item forever, while the queue is
 *	full and it is impossible to enqueue the item it is waiting for.
 *	The simplest way to limit this is to use another (unordered)
 *	producer-consumer queue, through which the "empty" work-items are
 *	returned to be sent for further processing. Alternately, it is possible
 *	to use a semaphore (if it is not required to recycle the work-items).
 *	The deadlock is detected using a debug assertion only, since it is so
 *	simple to avoid it.
 *
 *	In case the producer and the consumer are in the same thread:
 *	@code
 *	void Producer_Consumer(CProducerConsumerQueue<std::pair<TWorkItem, int> > &r_work_queue,
 *		COrderedProducerConsumerQueue<TWorkItem, int> &r_ordered_queue)
 *	{
 *		_ASSERTE(r_work_queue.n_Size() == r_ordering_queue.n_Size());
 *		// to avoid deadlock
 *
 *		int n_work_id = 0, n_consumed_id = 0;
 *		for(int n_enqueued = 0;; ++ n_work_id) {
 *			TWorkItem t_wi;
 *			if(!ReadStream(&t_wi))
 *				break; // finished
 *			r_work_queue.Put(std::make_pair(t_wi, n_work_id)); // mind some error-checking here
 *			++ n_enqueued;
 *			// send work to be done
 *
 *			TWorkItem t_processed;
 *			if(!r_ordered_queue.TryGet(t_processed)) // will not block, allows quickly filling the work queue at the beginning
 *				continue; // no work returned, push more work to do
 *			// see if finished work is coming back
 *
 *			WriteStream(t_processed); // write output stream; the order of work-items is preserved
 *			++ n_consumed_id; // note that this can safely overflow as long as the queue is shorter than the number of values of n_consumed_id
 *			-- n_enqueued; // one just came back
 *		}
 *		r_queue.Signal_Finished(); // mind some error-checking here
 *		// read input stream and send work items for processing
 *
 *		for(; n_consumed_id != n_work_id; ++ n_consumed_id) {
 *			TWorkItem t_processed;
 *			r_ordered_queue.Get(t_processed);
 *			WriteStream(t_processed); // write output stream; the order of work-items is preserved
 *		}
 *		// process the rest of the data left in the queue
 *
 *		Join_AllWorkers(); // join the worker threads (will not block, as the workers are finished now)
 *	}
 *
 *	void Worker(CProducerConsumerQueue<std::pair<TWorkItem, int> > &r_work_queue,
 *		COrderedProducerConsumerQueue<TWorkItem, int> &r_ordered_queue)
 *	{
 *		std::pair<TWorkItem, int> t_wi;
 *		while(r_work_queue.Get(t_wi)) { // get work item and its id
 *			ProcessWorkItem(t_wi.first); // do the work
 *			// the work items are processed in random order, depending on the speed
 *			// of processing of different work-items in different threads
 *
 *			r_ordered_queue.Put(t_wi.first, t_wi.second); // send the finished work back
 *		}
 *		if(!r_queue.b_Finished())
 *			Error(); // there are still data in the queue, but Get() failed for some reason ...
 *	}
 *	@endcode
 *
 *	If the producer and the consumer are running in separate threads, the problem
 *	is slightly more complicated:
 *	@code
 *	void Producer(CProducerConsumerQueue<std::pair<TWorkItem, int> > &r_work_queue,
 *		CProducerConsumerQueue<TWorkItem> &r_return_queue,
 *		COrderedProducerConsumerQueue<TWorkItem, int> &r_ordered_queue)
 *	{
 *		_ASSERTE(r_work_queue.n_Size() == r_ordering_queue.n_Size() &&
 *			r_return_queue.n_Size() == r_work_queue.n_Size());
 *		// to avoid deadlock
 *
 *		for(int i = 0; i < r_work_queue.n_Size(); ++ i)
 *			r_return_queue.Put(TWorkItem());
 *		// will return queue with empty work-items (or return_semaphore.Signal(r_work_queue.n_Size()))
 *
 *		for(int n_work_id = 0;; ++ n_work_id) {
 *			TWorkItem t_wi;
 *			r_return_queue.Get(t_wi); // or return_semaphore.Wait();
 *			if(!ReadStream(&t_wi))
 *				break; // finished
 *			r_work_queue.Put(std::make_pair(t_wi, n_work_id)); // mind some error-checking here
 *		}
 *		// read input stream and send work items for processing
 *		// note that this gets complicated if producer and consumer are
 *		// running in the same thread (the same loop)
 *
 *		r_queue.Signal_Finished(); // mind some error-checking here
 *		Join_AllWorkers(); // wait for the workers to finish processing
 *		r_ordered_queue.Signal_Finished(); // mind some error-checking here
 *	}
 *
 *	void Worker(CProducerConsumerQueue<std::pair<TWorkItem, int> > &r_work_queue,
 *		COrderedProducerConsumerQueue<TWorkItem, int> &r_ordered_queue)
 *	{
 *		std::pair<TWorkItem, int> t_wi;
 *		while(r_work_queue.Get(t_wi)) { // get work item and its id
 *			ProcessWorkItem(t_wi.first); // do the work
 *			// the work items are processed in random order, depending on the speed
 *			// of processing of different work-items in different threads
 *
 *			r_ordered_queue.Put(t_wi.first, t_wi.second); // send the finished work back
 *		}
 *		if(!r_queue.b_Finished())
 *			Error(); // there are still data in the queue, but Get() failed for some reason ...
 *	}
 *
 *	void Consumer(CProducerConsumerQueue<TWorkItem> &r_return_queue,
 *		COrderedProducerConsumerQueue<TWorkItem, int> &r_ordered_queue)
 *	{
 *		TWorkItem t_wi;
 *		while(r_ordered_queue.Get(t_wi)) {
 *			WriteStream(t_wi);
 *			// the work items are arriving in the same order
 *			// as they were read from the input stream by the producer
 *
 *			r_return_queue.Put(t_wi); // or return_semaphore.Signal();
 *		}
 *		// write output stream; the order is preserved
 *
 *		if(!r_ordered_queue.b_Finished())
 *			Error(); // there are still data in the queue, but Get() failed for some reason ...
 *	}
 *	@endcode
 *
 *	@tparam TWorkItem is data type of a single unit of work
 *	@tparam TIndex is data type for work item numbering
 *
 *	@note There may be multiple producers or consumers, all variants of Get() and Put() are both thread-safe.
 */
template <class TWorkItem, class TIndex = int>
class COrderedProducerConsumerQueue {
public:
	typedef std::pair<TIndex, TWorkItem> TIndex_WorkItem; /**< @brief index / work-item pair */

	/**
	 *	@brief adjacency predicate
	 */
	struct CIsAdjacent {
		/**
		 *	@copydoc CDefaultAdjacencyPredicate::operator()
		 */
		inline bool operator ()(const TIndex_WorkItem &a, const TIndex_WorkItem &b) const
		{
			TIndex _a = a.first;
			return ++ _a == b.first;
		}
	};

	/**
	 *	@brief equality predicate
	 */
	struct CIsEqual {
		/**
		 *	@brief equality predicate
		 *
		 *	@param[in] a is the first item
		 *	@param[in] b is the second item
		 *
		 *	@return Returns true if a equals b, otherwise returns false.
		 */
		inline bool operator ()(const TIndex_WorkItem &a, TIndex b) const
		{
			return a.first == b;
		}
	};

	typedef CAdjacentContiguousOrderedQueue<TIndex_WorkItem,
		TIndex, CIsAdjacent, CIsEqual> COrderedQueue; /**< @brief queue type */

protected:
	CSemaphore m_free_count_sema; /**< @brief semaphore, guarding the number of items to Put() */
	CSemaphore m_full_count_sema; /**< @brief semaphore, guarding the number of items to Get() */
	CMutex m_queue_access_mutex; /**< @brief data access mutex */

	COrderedQueue m_queue; /**< @brief work item storage and sorting queue */

	bool m_b_finished; /**< @brief finished flag */

public:
	/**
	 *	@brief default constructor; initializes the queue
	 *
	 *	@param[in] n_queue_size is size of the queue, in work-items
	 *	@param[in] n_first_id is id of the first work-item (the ids must be increasing and contiguous)
	 *
	 *	@note This constructor may fail (due to OS limits, or low free memory),
	 *		it is therefore reccommended to call b_Status() afterwards.
	 */
	inline COrderedProducerConsumerQueue(size_t n_queue_size, TIndex n_first_id = TIndex())
		:m_queue(n_queue_size, n_first_id), m_full_count_sema(0),
		m_free_count_sema(int(n_queue_size)), m_b_finished(false)
	{
		_ASSERTE(n_queue_size < INT_MAX); // CSemaphore constructor
		_ASSERTE(n_queue_size > 0); // useless, also b_Status() would become ambiguous
	}

	/**
	 *	@brief gets constructor result
	 *	@return Returns true if constructor succeeded and the queue is ready to be used, otherwise returns false.
	 *	@note This doesn't reflect any kind of queue state (eg. empty / full / finished queue).
	 */
	inline bool b_Status() const
	{
		return m_queue.n_Capacity() > 0 && m_queue_access_mutex.b_Status() &&
			m_free_count_sema.b_Status() && m_full_count_sema.b_Status();
	}

	/**
	 *	@brief gets queue size
	 *	@return returns queue size, in work-items
	 */
	inline size_t n_Size() const
	{
		return m_queue.n_Capacity();
	}

	/**
	 *	@brief gets number of items, currently in the queue
	 *	@return Returns number of items, currently in the queue.
	 *	@note This function is not thread-safe, and the result may thus be inaccurate.
	 */
	inline size_t n_EnqueuedItem_Num() const
	{
		return m_queue.n_Size();
	}

	/**
	 *	@brief determines whether is the queue empty
	 *	@return Returns true if the queue is empty, otherwise returns false.
	 *	@note This does not signal consumers there will be no more data, b_Finished() is used to do that.
	 *	@note This function always returns immediately.
	 */
	inline bool b_Empty() const
	{
		return m_queue.b_Empty();
	}

	/**
	 *	@brief determines whether is the queue full
	 *	@return Returns true if the queue is full, otherwise returns false.
	 *	@note This function always returns immediately.
	 */
	inline bool b_Full() const
	{
		return m_queue.b_Full();
	}

	/**
	 *	@brief signals the producer is done generating data
	 *	@return Returns true on success, false on failure (shouldn't really happen,
	 *		provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note This function should only be called by the producer, and only once.
	 */
	inline bool Signal_Finished()
	{
		m_b_finished = true;
		return m_full_count_sema.Signal(); // let the next waiting thread in
	}

	/**
	 *	@brief determines whether the finished flag was raised by the producer
	 *		(there will be no more data once the queue is empty)
	 *	@return Returns true in case the producer raised the finished flag (by calling Signal_Finished()),
	 *		otherwise returns false.
	 *	@note There still may be data in the queue which must be processed. The correct way to use this queue
	 *		is to call Get() until it fails, then calling b_Finished(). If it returns true, consumer can quit.
	 *		If, on the other hand, it returns false, there was an error.
	 */
	inline bool b_Finished() const
	{
		return b_Empty() && m_b_finished;
	}

	/**
	 *	@brief puts a single work-item to the queue
	 *
	 *	@param[in] t_wi is the work-item
	 *	@param[in] n_id is work-item id
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note Returns immediately if the queue is not full, or blocks until a consumer calls Get() and makes some space.
	 *	@note Thia could potentially cause a deadlock, if the queue was filled by items, not including the item
	 *		that is the next in the Get() order. This is checked by a debug assertion, and is a caller's response
	 *		to correctly initialize size of this queue to the number of work items in circulation.
	 */
	bool Put(TWorkItem t_wi, TIndex n_id)
	{
		if(!m_free_count_sema.Wait())
			return false;
		// wait for free semaphore

		return Naked_Put(t_wi, n_id);
	}

	/**
	 *	@brief tries to put a single work-item to the queue, in the specified time
	 *
	 *	@param[out] r_b_put_work_item is the result of the put (set if put, cleared if timed out)
	 *	@param[in] t_wi is the work-item
	 *	@param[in] n_timeout_msec is time to put the item in the queue, in milliseconds
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note Returns immediately if the queue is not full, or blocks until either a consumer calls Get()
	 *		and makes some space, or until the timeout is exceeded.
	 */
	bool TimedPut(bool &r_b_put_work_item, TWorkItem t_wi, int n_timeout_msec)
	{
		if(!m_free_count_sema.TimedWait(r_b_put_work_item, n_timeout_msec))
			return false;
		if(!r_b_put_work_item)
			return true;
		// enter the "empty" semaphore

		return Naked_Put(t_wi);
	}

	/**
	 *	@brief tries to put a single work-item to the queue
	 *	@param[in] t_wi is the work-item
	 *	@return Returns true in case the work-item was successfuly enqueued, false in case the queue was full,
	 *		or on failure (shouldn't really happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note Always returns immediately.
	 */
	inline bool TryPut(TWorkItem t_wi)
	{
		if(!m_free_count_sema.TryWait())
			return false;
		// enter the "empty" semaphore

		return Naked_Put(t_wi);
	}

	/**
	 *	@brief gets a single work-item from the queue
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *	@return Returns true on success, false if there's no more work (call b_Finished() to determine the cause)
	 *		or on failure (shouldn't really happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note Returns immediately if the queue is not empty, or blocks until a producer calls Put() or Signal_Finished().
	 */
	inline bool Get(TWorkItem &r_t_wi)
	{
		if(!m_full_count_sema.Wait())
			return false;
		// wait for the poppable items

		return Naked_Get(r_t_wi);
	}

	/**
	 *	@brief tries to get a single work-item from the queue, in the specified time
	 *
	 *	@param[out] r_b_received_work_item is the result of the get (set if got one, cleared if timed out)
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *	@param[in] n_timeout_msec is time to put the item in the queue, in milliseconds
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note Returns immediately if the queue is not empty, or blocks until either
	 *		a producer calls Put() or Signal_Finished(), or until the timeout is exceeded.
	 */
	inline bool TimedGet(bool &r_b_received_work_item, TWorkItem &r_t_wi, int n_timeout_msec)
	{
		if(!m_full_count_sema.TimedWait(r_b_received_work_item, n_timeout_msec))
			return false;
		if(!r_b_received_work_item)
			return true;
		// enter the "fill" semaphore

		return Naked_Get(r_t_wi);
	}

	/**
	 *	@brief tries to get a single work-item from the queue
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *	@return Returns true in case work-item was successfuly obtained, false in case the queue was empty,
	 *		if there's no more work (call b_Finished() to determine the cause), or on failure (shouldn't
	 *		really happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *	@note Always returns immediately.
	 */
	bool TryGet(TWorkItem &r_t_wi)
	{
		if(!m_full_count_sema.TryWait())
			return false;
		// wait for the poppable items

		return Naked_Get(r_t_wi);
	}

protected:
	/**
	 *	@brief puts a single work-item to the queue
	 *
	 *	@param[in] t_wi is the work-item
	 *
	 *	@return Returns true on success, false on failure (shouldn't really happen, provided the queue
	 *		was successfuly initialized and hasn't been deleted).
	 *
	 *	@note This requires waiting for the m_full_count_sema mutex before this is called.
	 *	@note Always returns immediately.
	 */
	bool Naked_Put(TWorkItem t_wi, TIndex n_id)
	{
		//m_free_count_sema.Wait(); // this is called outside, just before calling this function
		// wait for free semaphore

		if(!m_queue_access_mutex.Lock())
			return false;

		_ASSERTE(!m_queue.b_Full());
		_ASSERTE(m_queue.n_Size() + 1 < m_queue.n_Capacity() ||
			m_queue.n_Poppable_Num() > 0 || n_id == m_queue.t_Next()); // make this a function of queue? no need.
		// avoid the last free space deadlock
		// could we wait instead? would result in another deadlock, unless the semaphores
		// implement fair policy, which is os dependent. anyway, this is a programmer's fault
		// for not limiting the number of work-items in circulation, otherwise it would not happen
		// note that the implementation of TryPut() behaves completely the same

		size_t n_poppable_increment;
		bool b_result = m_queue.Push(TIndex_WorkItem(n_id, t_wi), n_poppable_increment);
		_ASSERTE(b_result); // only fails when completely full, but that cannot happen, thanks to m_free_count_sema
		// put the item in the queue in order to be sorted

		return m_queue_access_mutex.Unlock() &&
			(!n_poppable_increment || m_full_count_sema.Signal(n_poppable_increment));
		// signal the new poppable items (only if there is something to signal)
	}

	/**
	 *	@brief gets a single work-item from the queue
	 *
	 *	@param[out] r_t_wi is place for the work-item to be written
	 *
	 *	@return Returns true in case work-item was successfuly obtained, false in case there's no
	 *		more work (call b_Finished() to determine the cause), or on failure (shouldn't really
	 *		happen, provided the queue was successfuly initialized and hasn't been deleted).
	 *
	 *	@note This requires waiting for the m_full_count_sema mutex before this is called.
	 *	@note Always returns immediately.
	 */
	bool Naked_Get(TWorkItem &r_t_wi) // should we return the counter? it could be useful in situations where multiple threads read from the queue, but that does not make sense, as an ordinary unordered p-c queue would be used and the work item sorting would be done at a later stage. if there are any applications for it, it is easy to provide new interface.
	{
		//m_full_count_sema.Wait(); // this is called outside, just before calling this function
		// wait for the poppable items

		if(!m_queue_access_mutex.Lock())
			return false;

		if(m_b_finished && b_Empty()) {
			m_queue_access_mutex.Unlock(); // unlock mutex
			m_full_count_sema.Signal();
			// signal the "fill" semaphore in case there is more consumers accessing the queue to free the next one
			return false;
		}
		// handle finished state

		try {
			TIndex_WorkItem t_wic = m_queue.t_Pop();
			r_t_wi = t_wic.second; // if this throws, we lose the mutex, this takes assumptions about the user types
		} catch(std::runtime_error&) {
			_ASSERTE(0); // if m_full_count_sema let us in, there must be something to Pop()
			// if this triggers, there is an internal implementation error, or the mutex is broken
		}

		return m_queue_access_mutex.Unlock() &&
			m_free_count_sema.Signal();
		// signal the free semaphore
	}

private:
	COrderedProducerConsumerQueue(const COrderedProducerConsumerQueue &r_other); // no-copy
	COrderedProducerConsumerQueue &operator =(const COrderedProducerConsumerQueue &r_other); // no-copy
};

// todo - add an .inl, migrate code

#endif // !__LAME_THREADS_INCLUDED
