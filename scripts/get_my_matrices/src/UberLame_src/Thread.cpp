/*
								+--------------------------------+
								|                                |
								|  ***   Simple threading   ***  |
								|                                |
								|  Copyright © -tHE SWINe- 2008  |
								|                                |
								|           Thread.cpp           |
								|                                |
								+--------------------------------+
*/

/**
 *	@file Thread.cpp
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
 */

#include "NewFix.h"
#include "CallStack.h"
#include <time.h>

#if !defined(_DONT_USE_STACK_GUARD) && !defined(_MULTITHREADED_STACK_GUARD)
#error "call stack doesn't support multithreaded applications in this ÜberLame version" // #define _DONT_USE_STACK_GUARD
#endif // !_DONT_USE_STACK_GUARD && !_MULTITHREADED_STACK_GUARD

#include "Thread.h"

#if defined(_WIN32) || defined(_WIN64)

/*
 *								=== CThread (win32) ===
 */

CThread::CEmptyRunable CThread::empty_runable;

CThread::CThread()
	:m_p_runable(&empty_runable), m_h_thread(0)
{}

CThread::CThread(CRunable &r_runable)
	:m_p_runable(&r_runable), m_h_thread(0)
{}

CThread::~CThread()
{
	Stop(true);
	//if(m_h_thread)
	//	CloseHandle(m_h_thread); // Stop() does that
}

bool CThread::AttachRunable(CRunable &r_runable)
{
	if(b_IsRunning())
		return false;
	m_p_runable = &r_runable;
	return true;
}

CRunable &CThread::r_Runable()
{
	return *m_p_runable;
}

const CRunable &CThread::r_Runable() const
{
	return *m_p_runable;
}

bool CThread::Start()
{
	if(b_IsRunning())
		return false;
	if(!(m_h_thread = CreateThread(0, 0, _run, this, 0, 0)))
		return false;
	return true;
}

bool CThread::Run()
{
#ifdef __THREAD_SAFE_RUN
	if(!Start())
		return false;
	return Stop();
	// this is thread safe, but requires thread
#else // __THREAD_SAFE_RUN
	if(b_IsRunning())
		return false;
	m_p_runable->Run();
	return true;
	// this simply invokes thread's runable
#endif // __THREAD_SAFE_RUN
}

bool CThread::Suspend()
{
	if(!b_IsRunning())
		return false;
	return SuspendThread(m_h_thread) != -1;
}

bool CThread::Resume()
{
	if(!b_IsRunning())
		return false;
	for(;;) {
		int n = ResumeThread(m_h_thread);
		if(n == -1)
			return false; // failed
		else if(n <= 1)
			return true; // resumed
	}
}

bool CThread::b_IsRunning() const
{
	if(!m_h_thread)
		return false;
	if(!WaitForSingleObject(m_h_thread, 0)) {
		// in case thread is not running anymore,
		// WaitForSingleObject will return 0

		CloseHandle(m_h_thread);
		m_h_thread = 0;
		// close thread handle
		return false;
	}
	return true;
}

bool CThread::Stop(bool b_force_kill)
{
	if(!b_IsRunning())
		return true;
	// not running anymore

	if(!b_force_kill) {
		if(WaitForSingleObject(m_h_thread, INFINITE))
			return false;
	} else {
		if(!TerminateThread(m_h_thread, 0))
			return false;
		// kill thread
	}
	// either waits or terminates

	CloseHandle(m_h_thread);
	m_h_thread = 0;
	// destroy thread handle

	return true;
}

size_t CThread::n_CPU_Num()
{
	SYSTEM_INFO t_sys_info;
	GetSystemInfo(&t_sys_info);
	return t_sys_info.dwNumberOfProcessors;
}

unsigned long __stdcall CThread::_run(void *p_arg)
{
	CThread *p_this = (CThread*)p_arg;
	// this

	p_this->m_p_runable->Run();
	// run the thread

	return 0;
}

/*
 *								=== ~CThread (win32) ===
 */

#else // _WIN32 || _WIN64

#include <errno.h>
#include <limits.h>

/*
 *								=== CThread (linux) ===
 */

CThread::CEmptyRunable CThread::empty_runable;

CThread::CThread()
	:m_p_runable(&empty_runable), m_b_running(false), m_t_thread(0)
{
	pthread_mutex_init(&m_t_running_mutex, NULL);
	pthread_mutex_init(&m_t_suspend_mutex, NULL);
	pthread_cond_init(&m_t_suspend_cond, NULL);
}

CThread::CThread(CRunable &r_runable)
	:m_p_runable(&r_runable), m_b_running(false), m_t_thread(0)
{
	pthread_mutex_init(&m_t_running_mutex, NULL);
	pthread_mutex_init(&m_t_suspend_mutex, NULL);
	pthread_cond_init(&m_t_suspend_cond, NULL);
}

CThread::~CThread()
{
	Stop(true);
	pthread_mutex_destroy(&m_t_running_mutex);
	pthread_mutex_destroy(&m_t_suspend_mutex);
	pthread_cond_destroy(&m_t_suspend_cond);
}

bool CThread::AttachRunable(CRunable &r_runable)
{
	if(pthread_mutex_lock(&m_t_running_mutex))
		return false;
	// lock mutex, we will read m_b_running

	if(m_b_running) {
		pthread_mutex_unlock(&m_t_running_mutex);
		return false;
	}
	// thread is running, can't attach other runable

	m_p_runable = &r_runable;

	pthread_mutex_unlock(&m_t_running_mutex);
	// unlock

	return true;
}

CRunable &CThread::r_Runable()
{
	return *m_p_runable;
}

const CRunable &CThread::r_Runable() const
{
	return *m_p_runable;
}

//static int running_threads = 0;

bool CThread::Start()
{
	if(pthread_mutex_lock(&m_t_running_mutex))
		return false;
	// lock mutex, we will change m_b_running

	if(m_b_running) {
		pthread_mutex_unlock(&m_t_running_mutex);
		return false;
	}
	// thread is running, can't start

	pthread_attr_t t_attr;
	pthread_attr_init(&t_attr);
	pthread_attr_setdetachstate(&t_attr, PTHREAD_CREATE_JOINABLE);
	//int err;
	if((/*err =*/ pthread_create(&m_t_thread, &t_attr, _run, (void*)this))) {
		//fprintf("stderr: thread create: %d, running thread num: %d\n", err, running_threads);
		pthread_attr_destroy(&t_attr);
		pthread_mutex_unlock(&m_t_running_mutex); // don't forget!
		return false;
	}
	pthread_attr_destroy(&t_attr);
	// create joinable thread
	//++ running_threads; // debug

	m_b_running = true;
	pthread_mutex_unlock(&m_t_running_mutex);
	// set running to true and unlock

	return true;
}

bool CThread::Run()
{
#ifdef __THREAD_SAFE_RUN
	if(!Start())
		return false;
	return Stop();
	// this is thread safe, but requires thread
#else // __THREAD_SAFE_RUN
	if(b_IsRunning())
		return false;
	m_p_runable->Run();
	return true;
	// this simply invokes thread's runable
#endif // __THREAD_SAFE_RUN

/*	if(pthread_mutex_lock(&m_t_running_mutex))
		return false;
	// lock mutex, we will change m_b_running

	if(m_b_running) {
		pthread_mutex_unlock(&m_t_running_mutex);
		return false;
	}
	// thread is running, can't start

	m_b_running = true;
	pthread_mutex_unlock(&m_t_running_mutex);
	// set running to true and unlock

	m_p_runable->Run();
	// this simply invokes thread's runable

	int UNUSED(n_result) = pthread_mutex_lock(&m_t_running_mutex);
	_ASSERTE(!n_result); // this shouldn't fail
	m_b_running = false;
	pthread_mutex_unlock(&m_t_running_mutex);
	// not running anymore

	return true;*/
}

bool CThread::Suspend()
{
	if(!b_IsRunning())
		return false;
	_ASSERTE(pthread_self() == m_t_thread); // must be called from within the thread // this was pthread_current
	pthread_mutex_lock(&m_t_suspend_mutex);
	pthread_cond_wait(&m_t_suspend_cond, &m_t_suspend_mutex);
	pthread_mutex_unlock(&m_t_suspend_mutex);
	return true;
}

bool CThread::Resume()
{
	if(!b_IsRunning())
		return false;
	pthread_mutex_lock(&m_t_suspend_mutex);
	pthread_cond_signal(&m_t_suspend_cond);
	pthread_mutex_unlock(&m_t_suspend_mutex);
	return true;
}

bool CThread::b_IsRunning() const
{
	return m_b_running;
}

bool CThread::Stop(bool b_force_kill)
{
	if(pthread_mutex_lock(&m_t_running_mutex))
		return false;
	// lock mutex, we will change m_b_running

	if(!m_b_running) {
		if(m_t_thread) {
			/*int n =*/ pthread_join(m_t_thread, NULL);
			m_t_thread = 0; // !!
		}
		/*if(n)
			fprintf(stderr, "error: thread join: %d\n", n);*/
		//-- running_threads; // debug
		pthread_mutex_unlock(&m_t_running_mutex);
		return true;
	}
	// not running anymore

	if(!b_force_kill) {
		pthread_mutex_unlock(&m_t_running_mutex);
		// unlock mutex, the thread will set m_b_running by itself

		if(pthread_join(m_t_thread, NULL))
			return false;
		m_t_thread = 0; // !!
		//-- running_threads; // debug
		// join the thread (this helps with running out of memory)

		// don't assert m_b_running now! it can be running again by now!
	} else {
		if(pthread_cancel(m_t_thread)) {
			pthread_mutex_unlock(&m_t_running_mutex); // don't forget
			return false;
		}
		// kill thread

		/*int n =*/ pthread_join(m_t_thread, NULL); // not sure wheter this is useful here (after cancel)
		m_t_thread = 0; // !!
		/*if(n)
			fprintf(stderr, "error: thread join: %d\n", n);*/
		//-- running_threads; // debug
		// join the thread (this helps with running out of memory)

		m_b_running = false;
		pthread_mutex_unlock(&m_t_running_mutex);
		// set m_b_running 'manually'
	}
	// either waits or terminates

	return true;
}

#include <unistd.h>
#include <sys/sysctl.h>

#if defined(CTL_HW) && defined(HW_NCPU)

static size_t n_CPU_Num() // BSD
{
	int ncpus;
	size_t len = sizeof(ncpus);
	int mib[2] = {CTL_HW, HW_NCPU};
	if(sysctl(&mib[0], 2, &ncpus, &len, NULL, 0) != 0)
		return -1;
	return ncpus;
}

#elif defined(_SC_NPROCESSORS_ONLN)

static size_t n_CPU_Num() // Solaris
{
	return sysconf(_SC_NPROCESSORS_ONLN);
}

#elif defined(_SC_NPROCESSORS_CONF)

static size_t n_CPU_Num() // Linux
{
	return sysconf(_SC_NPROCESSORS_CONF);
}

#else

static size_t n_CPU_Num() // failed
{
	return size_t(-1);
}

#endif

size_t CThread::n_CPU_Num()
{
	return ::n_CPU_Num();
}

void *CThread::_run(void *p_arg)
{
	CThread *p_this = (CThread*)p_arg;
	// this

	p_this->m_p_runable->Run();
	// run the thread

	int UNUSED(n_result) = pthread_mutex_lock(&p_this->m_t_running_mutex);
	_ASSERTE(!n_result); // this shouldn't fail ... there's no way how to handle failure here
	p_this->m_b_running = false;
	pthread_mutex_unlock(&p_this->m_t_running_mutex);
	// not running anymore

	pthread_exit(NULL);
	// exit
}

/*
 *								=== ~CThread (linux) ===
 */

#endif // _WIN32 || _WIN64

#if defined(_WIN32) || defined(_WIN64)

/*
 *								=== CMutex (win32) ===
 */

CMutex::CMutex()
	//:m_b_status(false)
{
#ifdef __MUTEX_USE_OPENMP_LOCK
	omp_init_lock(&m_t_lock);
#else // __MUTEX_USE_OPENMP_LOCK
#ifdef _WIN32_WCE
	m_h_mutex = CreateMutexW(NULL, false, NULL); // there is no CreateMutexA in WinCE
#else // _WIN32_WCE
	m_h_mutex = CreateMutex(NULL, false, NULL);
#endif // _WIN32_WCE
#endif // __MUTEX_USE_OPENMP_LOCK
}

CMutex::~CMutex()
{
#ifdef __MUTEX_USE_OPENMP_LOCK
	omp_destroy_lock(&m_t_lock);
#else // __MUTEX_USE_OPENMP_LOCK
	if(m_h_mutex != NULL)
		CloseHandle(m_h_mutex);
#endif // __MUTEX_USE_OPENMP_LOCK
}

bool CMutex::b_Status() const
{
#ifdef __MUTEX_USE_OPENMP_LOCK
	return true; // ??
#else // __MUTEX_USE_OPENMP_LOCK
	return m_h_mutex != NULL;
#endif // __MUTEX_USE_OPENMP_LOCK
}

bool CMutex::Lock()
{
#ifdef __MUTEX_USE_OPENMP_LOCK
	omp_set_lock(&m_t_lock);
	return true;
#else // __MUTEX_USE_OPENMP_LOCK
	switch(WaitForSingleObject(m_h_mutex, INFINITE)) {
	case WAIT_OBJECT_0:
		//m_b_status = true;
		return true;
	//case WAIT_TIMEOUT: // won't happen with INFINITE
	case WAIT_FAILED:
	default:
		return false;
	case WAIT_ABANDONED: // data, accessed inside mutex are probably dammaged. fail.
		/*if(!m_b_status) { // mutex was properly unlocked (windows sets ABANDONED even to unlocked mutexes)
			m_b_status = true;
			return true;
		}*/
		return false;
	}
#endif // __MUTEX_USE_OPENMP_LOCK
}

bool CMutex::TryLock()
{
#ifdef __MUTEX_USE_OPENMP_LOCK
	return omp_test_lock(&m_t_lock) != 0; // for a simple lock, the omp_test_lock function returns a nonzero value if the lock is successfully set
#else // __MUTEX_USE_OPENMP_LOCK
	switch(WaitForSingleObject(m_h_mutex, 0)) {
	case WAIT_OBJECT_0:
		//m_b_status = true;
		return true; // we got the mutex
	case WAIT_TIMEOUT:
		return false; // timeout
	case WAIT_FAILED:
	default:
		return false;
	case WAIT_ABANDONED: // data, accessed inside mutex are probably dammaged. fail.
		/*if(!m_b_status) { // mutex was properly unlocked (windows sets ABANDONED even to unlocked mutexes)
			m_b_status = true;
			return true;
		}*/
		return false;
	}
#endif // __MUTEX_USE_OPENMP_LOCK
}

bool CMutex::Unlock()
{
#ifdef __MUTEX_USE_OPENMP_LOCK
	omp_unset_lock(&m_t_lock);
	return true;
#else // __MUTEX_USE_OPENMP_LOCK
	//m_b_status = false;
	return ReleaseMutex(m_h_mutex) != 0;
#endif // __MUTEX_USE_OPENMP_LOCK
}

/*
 *								=== ~CMutex (win32) ===
 */

#else // WIN32 || _WIN64

/*
 *								=== CMutex (linux) ===
 */

CMutex::CMutex()
{
	m_b_status = !pthread_mutex_init(&m_t_mutex, NULL);
}

CMutex::~CMutex()
{
	if(m_b_status)
		pthread_mutex_destroy(&m_t_mutex);
}

bool CMutex::b_Status() const
{
	return m_b_status;
}

bool CMutex::Lock()
{
	return !pthread_mutex_lock(&m_t_mutex);
}

bool CMutex::TryLock()
{
	return !pthread_mutex_trylock(&m_t_mutex);
}

bool CMutex::Unlock()
{
	return !pthread_mutex_unlock(&m_t_mutex);
}

/*
 *								=== ~CMutex (linux) ===
 */

#endif // WIN32 || _WIN64

/*
 *								=== CSemaphore ===
 */

CSemaphore::CSemaphore(int n_initial_count)
{
#if defined(_WIN32) || defined(_WIN64)
#ifdef _WIN32_WCE
	m_h_semaphore = CreateSemaphoreW(NULL, n_initial_count, INT_MAX, NULL); // there is no CreateSemaphoreA in WinCE
#else // _WIN32_WCE
	m_h_semaphore = CreateSemaphore(NULL, n_initial_count, INT_MAX, NULL);
#endif // _WIN32_WCE
#else // _WIN32 || _WIN64
	m_b_status = sem_init(&m_t_semaphore, 0, n_initial_count) != -1;
#endif // _WIN32 || _WIN64
}

CSemaphore::~CSemaphore()
{
#if defined(_WIN32) || defined(_WIN64)
	CloseHandle(m_h_semaphore);
#else // _WIN32 || _WIN64
	sem_destroy(&m_t_semaphore);
#endif // _WIN32 || _WIN64
}

bool CSemaphore::b_Status() const
{
#if defined(_WIN32) || defined(_WIN64)
	return m_h_semaphore != NULL;
#else // _WIN32 || _WIN64
	return m_b_status;
#endif // _WIN32 || _WIN64
}

bool CSemaphore::Wait()
{
#if defined(_WIN32) || defined(_WIN64)
	switch(WaitForSingleObject(m_h_semaphore, INFINITE)) {
	case WAIT_OBJECT_0:
		return true;
	//case WAIT_TIMEOUT: // won't happen with INFINITE
	case WAIT_FAILED:
	case WAIT_ABANDONED: // data, accessed inside mutex are probably dammaged. fail.
	default:
		return false;
	}
#else // _WIN32 || _WIN64
	return !sem_wait(&m_t_semaphore);
#endif // _WIN32 || _WIN64
}

bool CSemaphore::TryWait()
{
#if defined(_WIN32) || defined(_WIN64)
	return WaitForSingleObject(m_h_semaphore, 0) == WAIT_OBJECT_0; // zero-second time-out interval
#else // _WIN32 || _WIN64
	return !sem_trywait(&m_t_semaphore);
#endif // _WIN32 || _WIN64
}

bool CSemaphore::TimedWait(bool &r_b_locked, int n_wait_seconds, long n_wait_nanoseconds)
{
#if defined(_WIN32) || defined(_WIN64)
	switch(WaitForSingleObject(m_h_semaphore, n_wait_seconds * 1000 + n_wait_nanoseconds / 1000000)) {
	case WAIT_OBJECT_0:
		r_b_locked = true;
		// successfully locked before timeout
		return true;
	case WAIT_TIMEOUT:
		r_b_locked = false;
		// timed out
		return true;
	case WAIT_FAILED:
	case WAIT_ABANDONED: // data, accessed inside mutex are probably dammaged. fail.
	default:
		r_b_locked = false;
		return false;
	}
#else // _WIN32 || _WIN64
#ifdef __APPLE__
	uint64_t n_useconds = uint64_t(n_wait_seconds) * 1000000;
	n_useconds += n_wait_nanoseconds / 1000;
	// convert to useconds

	int n_multiplier = 1000000 / CLOCKS_PER_SEC;
	if(!n_multiplier) {
		n_multiplier = 1; // overly precise clock()
		n_useconds *= CLOCKS_PER_SEC / 1000000; // scale the time we wait instead
	}
	// calculate clock() multipler

	int n_prev_clock;
	n_useconds += (n_prev_clock = clock()) * n_multiplier;
	// see when

	for(;;) {
		if(!sem_trywait(&m_t_semaphore)) {
			r_b_locked = true;
			return true;
			// successfully locked before timeout
		} else {
			switch(errno) {
			case EINTR: // interrupt from a signal
			case ETIMEDOUT:
				{
					int n_cur_clock;
					if((n_cur_clock = clock()) * n_multiplier < n_useconds && n_cur_clock > n_prev_clock)
						continue; // if there is still time *and* the clock did not overflow, try again
				}
				r_b_locked = false;
				// timed out, but no error occured
				return true;
			default:
			case EDEADLK:
			case EINVAL: // invalid semaphore handle
				r_b_locked = false;
				return false;
			}
		}
		// apple does not have timed wait, just use trywait
	}
	// this is a shitty implementation as the waiting thread loses fairness and
	// may never get in if there are many other threads waiting without timeout.
	// we need to wait and see what apple comes up with.
#else // __APPLE__
	n_wait_seconds += (n_wait_nanoseconds / (1000 * long(1000000)));
	n_wait_nanoseconds %= (1000 * long(1000000));
	// sem_timedwait requires that n_wait_nanoseconds is less than 1000 000 000

	struct timespec t_timeout;
	t_timeout.tv_sec = n_wait_seconds;
	t_timeout.tv_nsec = n_wait_nanoseconds;
	if(!sem_timedwait(&m_t_semaphore, &t_timeout)) {
		r_b_locked = true;
		return true;
		// successfully locked before timeout
	} else {
		switch(errno) {
		case ETIMEDOUT:
			r_b_locked = false;
			// timed out, but no error occured
			return true;
		default:
		case EINTR: // interrupt from a signal
		case EDEADLK:
		case EINVAL: // invalid semaphore handle
			r_b_locked = false;
			return false;
		}
	}
#endif // __APPLE__
#endif // _WIN32 || _WIN64
}

bool CSemaphore::Signal()
{
#if defined(_WIN32) || defined(_WIN64)
	return ReleaseSemaphore(m_h_semaphore, 1, NULL) != 0; // increase count by one
#else // _WIN32 || _WIN64
	return !sem_post(&m_t_semaphore);
#endif // _WIN32 || _WIN64
}

bool CSemaphore::Signal(int n_signal_num)
{
	_ASSERTE(n_signal_num > 0); // MSDN says: The value must be greater than zero.
#if defined(_WIN32) || defined(_WIN64)
	return ReleaseSemaphore(m_h_semaphore, n_signal_num, NULL) != 0; // increase count by n_signal_num
#else // _WIN32 || _WIN64
	int n_result = 0;
	for(int i = 0; i < n_signal_num; ++ i)
		n_result |= sem_post(&m_t_semaphore); // todo 
	return !n_result;
#endif // _WIN32 || _WIN64
}

bool CSemaphore::Signal(int n_signal_num, int &r_n_previous_value)
{
	_ASSERTE(n_signal_num > 0); // MSDN says: The value must be greater than zero.
#if defined(_WIN32) || defined(_WIN64)
	LONG n_prev_value;
	bool b_result = ReleaseSemaphore(m_h_semaphore, n_signal_num, &n_prev_value) != 0; // increase count by n_signal_num
	r_n_previous_value = int(n_prev_value);
	return b_result;
#else // _WIN32 || _WIN64
	{
		if(!sem_getvalue(&m_t_semaphore, &r_n_previous_value)) // pretty simple
			return false;
		int n_result = 0;
		for(int i = 0; i < n_signal_num; ++ i)
			n_result |= sem_post(&m_t_semaphore);
		return !n_result;
	}
	// todo - this is all neat, but not atomic. are there any applications where it matters?
	// note this could be implemented by a call to semop()
#endif // _WIN32 || _WIN64
}

#if defined(_WIN32) || defined(_WIN64)

typedef LONG NTSTATUS;

typedef NTSTATUS (NTAPI *_NtQuerySemaphore) (
    HANDLE SemaphoreHandle, 
    DWORD SemaphoreInformationClass, /* Would be SEMAPHORE_INFORMATION_CLASS */
    PVOID SemaphoreInformation,      /* but this is to much to dump here     */
    ULONG SemaphoreInformationLength, 
    PULONG ReturnLength OPTIONAL
); 

typedef struct _SEMAPHORE_BASIC_INFORMATION {   
    ULONG CurrentCount; 
    ULONG MaximumCount;
} SEMAPHORE_BASIC_INFORMATION;

static _NtQuerySemaphore NtQuerySemaphore = 0; // not completely sure if it is safe to cache this

#endif // _WIN32 || _WIN64

int CSemaphore::n_Value() const
{
#if defined(_WIN32) || defined(_WIN64)
	if(!NtQuerySemaphore && !(NtQuerySemaphore =
	   (_NtQuerySemaphore)GetProcAddress(GetModuleHandleA("ntdll.dll"), "NtQuerySemaphore")))
		return INT_MAX;
	SEMAPHORE_BASIC_INFORMATION t_sema_info;
	NTSTATUS n_result = NtQuerySemaphore(m_h_semaphore, 0,
		&t_sema_info, sizeof(SEMAPHORE_BASIC_INFORMATION), 0);
	if(n_result == ERROR_SUCCESS)
		return t_sema_info.CurrentCount;
	else
		return INT_MAX;
#else // _WIN32 || _WIN64
	int n_value;
	if(!sem_getvalue(const_cast<sem_t*>(&m_t_semaphore), &n_value)) // pretty simple
		return n_value;
	else
		return INT_MAX;
#endif // _WIN32 || _WIN64
}

/*
 *								=== ~CSemaphore ===
 */
