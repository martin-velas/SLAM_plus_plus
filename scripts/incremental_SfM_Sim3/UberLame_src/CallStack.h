/*
								+--------------------------------+
								|                                |
								|   ***  Call stack guard  ***   |
								|                                |
								|  Copyright © -tHE SWINe- 2005  |
								|                                |
								|          CallStack.h           |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __CALL_STACK_MONITOR_INCLUDED
#define __CALL_STACK_MONITOR_INCLUDED

/**
 *	@file CallStack.h
 *	@author -tHE SWINe-
 *	@date 2005
 *	@brief simple software-enabled call stac guard
 *
 *	This is used to provide information about program flow for debugging purposes.
 *	A variant of assert() macro called _ASSERTE() is provided here, which shows
 *	call history when assert fails, giving much more information about how exactly
 *	the failure occured.
 *
 *	@date 2006-05-16
 *
 *	added the p_s_Format() function, fixed some logging errors
 *	added couple of _ASSERTE-s to CCallStackGuard::file_printf
 *
 *	@date 2007-02-05
 *
 *	fixed pointer / iterator type mismatch in CCallStackGuard::~CCallStackGuard()
 *	removed handcrafted loops over std::vector, using std::for_each instead
 *	fixed 'the' typo in code history
 *
 *	@date 2007-03-06
 *
 *	removed __declspec(dllexport)
 *	fixed error arround separator concatenating in call stack dump
 *
 *	@date 2007-10-29
 *
 *	enhanced linux compatibility
 *
 *	@date 2007-11-12
 *
 *	reformat (added line breaks where lines were longer than 100 characters)
 *
 *	@date 2008-02-06
 *
 *	slightly improved assertion for console apps (needs no memory allocation)
 *	changed message formatting a bit
 *
 *	@date 2008-04-20
 *
 *	added a couple of \#ifdefs which disable compilation of CCallStackGuard
 *	in cases it's not needed
 *
 *	@date 2008-08-08
 *
 *	added \#ifdef for windows 64
 *
 *	@date 2009-05-23
 *
 *	added check for NewFix.h inclusion (todo - maybe remove it, it might be rather annoying)
 *
 *	@date 2009-06-01
 *
 *	rewritten call-stack compile logic, added _USE_STACK_GUARD_IN_RELEASE macro
 *
 *	renamed CCallStackGuard to CCallStackMonitor
 *
 *	removed p_s_Format() functions, use stl_ut::Format() from StlUtils.h instead
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 *	checking NewFix presence under MSVC 6.0 only
 *
 *	@todo this implementation can work with single-threaded code only. think about
 *		multithreaded call stack (call stack contains pairs of function name and thread
 *		identification, is protected by mutex, abort prints thread id)
 *
 *	@date 2010-10-29
 *
 *	Unified windows detection macro to "\#if defined(_WIN32) || defined(_WIN64)".
 *
 *	@date 2012-06-11
 *
 *	Unified x64 detection macros.
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	Added the _MULTITHREADED_STACK_GUARD (disabled by default) macro, enabling the use of
 *	call stack even in multithreaded scenarios. Note that using this may cause some performance
 *	drop (in debug), as the call stack is now protected by a mutex.
 *
 */

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200

#ifndef __ULAME_SUPPRESS_NEW_FIX_PRESENCE_TEST
#ifndef __ULAME_SUPPRESS_NEW_FIX
#ifndef __ULAME_FIX_NEW_STD_NOTHROW
#pragma message("warning: NewFix.h not included")
#include "NewFix.h"
#endif // __ULAME_FIX_NEW_STD_NOTHROW
#endif // __ULAME_SUPPRESS_NEW_FIX
#endif // __ULAME_SUPPRESS_NEW_FIX_PRESENCE_TEST
// CallStack.h is usualy (or should be) the first file included.
// this makes sure NewFix.h is included as well.

#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200
// this fix is only needed in MSVC 6.0

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200
#pragma warning(disable: 4786)
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200
// disable MSVC warning identifier was truncated to 255 characters in debug info

#include <vector>
#include <string>

/**
 *	@def _FORCE_ASSERTIONS
 *
 *	@brief force assertions in release
 *
 *	If defined, assertions are compiled in release code as well.
 */
//#define _FORCE_ASSERTIONS

/**
 *	@def _USE_STACK_GUARD_IN_RELEASE
 *
 *	@brief force call stack guard in release
 *
 *	If defined, call stack guard is compiled in release code as well
 *		(note this exposes program function names and may considerably
 *		ease reverse-engineering).
 */
//#define _USE_STACK_GUARD_IN_RELEASE

/**
 *	@def _DONT_USE_STACK_GUARD
 *
 *	@brief disables call stack guard
 *
 *	Disables using call stack in both debug and release. The <tt>__FuncGuard()</tt> macro
 *		will be defined empty, and <tt>_ASSERTE()</tt> is not going to display call stack
 *		on failure.
 *
 *	@note This is higher priority than _USE_STACK_GUARD_IN_RELEASE macro.
 */
//#define _DONT_USE_STACK_GUARD

/**
 *	@def _MULTITHREADED_STACK_GUARD
 *	@brief enables the use of call stack even with multiple threads
 */
//#define _MULTITHREADED_STACK_GUARD

/**
 *	@def ABORT_HERE
 *	@brief aborts the execution of the program, highlighting the line of execution in the debugger
 *		or terminating the program due to unhandled exception in case it is not being debugged
 *	@note This has the same effect as CCallStackMonitor::Abort() except that the debugger stops at the line
 *		where this macro is put rather than inside the implementation of CCallStackMonitor::Abort().
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(__ARM__) && !defined(_WIN32_WCE)
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
void __debugbreak(void); // from intrin.h, don't want to include the whole file
#define ABORT_HERE do { __debugbreak(); /* x64 debug break */ } while(0)
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#ifdef _OPENMP
inline void MSVC_OpenMPAbortWrapper() { __asm { int 3 }; /* msvc user breakpoint */ }
#define ABORT_HERE do { MSVC_OpenMPAbortWrapper(); } while(0) /* msvc OpenMP regions cannot contain inline assembly */
#else // _OPENMP
#define ABORT_HERE do { __asm { int 3 }; /* msvc user breakpoint */ } while(0)
#endif // _OPENMP
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _MSC_VER && !__MWERKS__ && !__ARM__ && !_WIN32_WCE
#include <signal.h>
#define ABORT_HERE do { raise(SIGTRAP); /* unix solution */ } while(0)
#endif // _MSC_VER && !__MWERKS__ && !__ARM__ && !_WIN32_WCE

#if defined(_DEBUG) || defined(_FORCE_ASSERTIONS)
/**
 *	@brief set when the <tt>_ASSERTE()</tt> macro is not empty
 */
#define __HAVE_ASSERTE__ 1
#else // _DEBUG || _FORCE_ASSERTIONS
#define __HAVE_ASSERTE__ 0
#endif // _DEBUG || _FORCE_ASSERTIONS

#if ((defined(_DEBUG) || defined(_USE_STACK_GUARD_IN_RELEASE)) && !defined(_DONT_USE_STACK_GUARD))
/**
 *	@brief set when the <tt>__FuncGuard()</tt> macro is not empty
 */
#define __HAVE_FUNCGUARD__ 1
#else // (_DEBUG || _USE_STACK_GUARD_IN_RELEASE) && !_DONT_USE_STACK_GUARD
#define __HAVE_FUNCGUARD__ 0
#endif // (_DEBUG || _USE_STACK_GUARD_IN_RELEASE) && !_DONT_USE_STACK_GUARD
// a bit complicated logic here ...

#if __HAVE_ASSERTE__ || __HAVE_FUNCGUARD__

/**
 *	@brief global call stack monitor class
 *
 *	Helper class for function flow monitoring and debugging. Manages stack
 *		of function names in order they are called, it can create string,
 *		containing list of those function names. (that is particularily
 *		useful for debugging, for example assertion macros provided here
 *		show call hierarchy).
 *
 *	@note It is necessary to put <tt>__FuncGuard("function_name");</tt> as
 *		the first line to each function (aparts from getters / setters
 *		and other "small" functions) for call stack to collect it's
 *		name once the function is entered.
 */
class CCallStackMonitor {
private:
#if __HAVE_FUNCGUARD__
/*#ifdef _MULTITHREADED_STACK_GUARD
	static CMutex m_mutex; // can't be member, must not include Thread.h before defining _ASSERTE, or things will go wrong
#endif // _MULTITHREADED_STACK_GUARD*/
	static std::vector<std::pair<int, const char*> > m_call_stack; /**< @brief stack of thread ids and function names */
#endif // __HAVE_FUNCGUARD__

public:
#if __HAVE_FUNCGUARD__

	/**
	 *	@brief function entry notification
	 *
	 *	Default constructor. Adds pointer to function name on the top of the stack.
	 *
	 *	@param[in] p_s_function_name is name of function to be added at the top
	 *		of the stack. It is expected this will always be static string, so it
	 *		doesn't have to be deleted / doesn't get deleted.
	 *
	 *	@note This should never be used directly in the program, only trough the
	 *		<tt>__FuncGuard()</tt> macro.
	 */
	CCallStackMonitor(const char *p_s_function_name);

	/**
	 *	@brief function return notification
	 *
	 *	Destructor, removes last function name from the top of the stack (doesn't
	 *		really check if name on top of the stack is the same, that was passed
	 *		to constructor, it is relied upon correct usage of <tt>__FuncGuard()</tt> macro).
	 */
	~CCallStackMonitor();

	/**
	 *	@brief gets current call stack
	 *
	 *	@return Returns current state of call stack (vector, containing list of
	 *		thread ids and function names).
	 */
	static const std::vector<std::pair<int, const char*> > &Get();

	/**
	 *	@brief gets current call stack
	 *
	 *	Copies string, containing function names, separated by p_s_separator to r_s_stack.
	 *
	 *	@param[out] r_s_stack is destination for call stack string (original contents are lost)
	 *	@param[in] p_s_separator is function name separator (default " -> ")
	 *
	 *	@return Returns true on success, false on failure (not enough memory for the string).
	 *
	 *	@note Any of p_s_separator, p_s_pro or p_s_epi can be null.
	 */
	static bool Get(std::string &r_s_stack, const char *p_s_separator = " -> ");

#endif // __HAVE_FUNCGUARD__

	/**
	 *	@brief assert handler
	 *
	 *	Stops program execution. This is usually called when <tt>_ASSERTE()</tt> fails,
	 *		but it can be called by user code as well.
	 *
	 *	@note This uses interrupt 3 "user breakpoint" in MSVC or SIGTRAP elsewhere.
	 */
	static void Abort();

#if __HAVE_ASSERTE__
#if (defined(_WIN32) || defined(_WIN64)) && !defined(_CONSOLE)

	/**
	 *	@brief assert handler
	 *
	 *	Function, called when assertion failed. Displays information (in windows message box)
	 *		about current state in which the error occured, and calls Abort().
	 *
	 *	@param[in] p_s_expr is expression that failed (filled-in by preprocessor)
	 *	@param[in] n_line is line in source code expression lies on (filled-in by preprocessor)
	 *	@param[in] p_s_file is filename of source code (filled-in by preprocessor)
	 *	@param[in] b_abort is abort flag (if set, program execution will be aborted)
	 *
	 *	@note This is usually called in <tt>_ASSERTE()</tt> macro expansion, but it can be called
	 *		by user code as well. None of the parameters may be null, with exception of n_line.
	 */
	static void AssertHandler(const char *p_s_expr, int n_line, const char *p_s_file, bool b_abort = true);

#else // (_WIN32 || _WIN64) && !_CONSOLE

	/**
	 *	@brief assert handler
	 *
	 *	Function, called when assertion failed. Displays information (prints to stderr)
	 *		about current state in which the error occured, and calls Abort().
	 *
	 *	@param[in] p_s_expr is expression that failed (filled-in by preprocessor)
	 *	@param[in] n_line is line in source code expression lies on (filled-in by preprocessor)
	 *	@param[in] p_s_file is filename of source code (filled-in by preprocessor)
	 *	@param[in] b_abort is abort flag (if set, program execution will be aborted)
	 *
	 *	@note This is usually called in <tt>_ASSERTE()</tt> macro expansion, but it can be called
	 *		by user code as well. None of the parameters may be null, with exception of n_line.
	 */
	static void AssertHandler(const char *p_s_expr, int n_line, const char *p_s_file, bool b_abort = true);

#endif // (_WIN32 || _WIN64) && !_CONSOLE
#endif // __HAVE_ASSERTE__
};

#endif // __HAVE_ASSERTE__ || __HAVE_FUNCGUARD__

#if __HAVE_FUNCGUARD__
/**
 *	@brief function guard
 *
 *	This should be inserted as the first line of every function, with possible exception
 *		of very short functions which do not contain <tt>_ASSERTE()</tt> and do not call
 *		other functions. This just creates instance of CCallStackMonitor, pushing function
 *		name on the top of the stack once function is called. When function returns,
 *		CCallStackMonitor::~CCallStackMonitor() is called, removing function name from the
 *		stack.
 *
 *	@param[in] p_s_func_name is function name in static null-terminated string.
 */
#define __FuncGuard(p_s_func_name) CCallStackMonitor __call_stack_guard__##__LINE__##__(p_s_func_name)
#else // __HAVE_FUNCGUARD__
#define __FuncGuard(p_s_func_name) do { /* empty */ } while(false)
#endif // __HAVE_FUNCGUARD__
// defines __FuncGuard macro

#ifdef _ASSERTE
#undef _ASSERTE
#endif // _ASSERTE
// undef _ASSERTE if defined

/**
 *	@def _ASSERTE
 *	@brief expression assertion macro
 *
 *	This macro behaves just like common assert() macro, with the exception it shows
 *		call stack trace when assertion fails.
 *
 *	@param[in] b_exp is (boolean) expression to be asserted
 */
#if __HAVE_ASSERTE__
#define _ASSERTE(b_exp) \
	do { \
		if(!(b_exp)) { \
			CCallStackMonitor::AssertHandler(#b_exp, __LINE__, __FILE__, false); \
			ABORT_HERE; /* stop with the debugger at the context rather than in CallStack.cpp */ \
		} \
	} while(false)
#else // __HAVE_ASSERTE__
#define _ASSERTE(b_exp) do { /* empty */ } while(false)
#endif // __HAVE_ASSERTE__
// defines _ASSERTE macro

#endif // !__CALL_STACK_MONITOR_INCLUDED
