/*
								+--------------------------------+
								|                                |
								|   ***  Call stack guard  ***   |
								|                                |
								|  Copyright © -tHE SWINe- 2005  |
								|                                |
								|         CallStack.cpp          |
								|                                |
								+--------------------------------+
*/

/**
 *	@file CallStack.cpp
 *	@date 2005
 *	@author -tHE SWINe-
 *	@brief call stack guard
 *
 *	@date 2006-05-16
 *
 *	passed code revision
 *
 *	added the p_s_Format() function, fixed some logging errors
 *	added couple of _ASSERTE-s to CCallStackGuard::file_printf
 *
 *	@date 2007-02-05
 *
 *	passed code revision
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
 */

#include "NewFix.h"

#ifndef __ULAME_SUPPRESS_NEW_FIX_PRESENCE_TEST
#ifndef __ULAME_SUPPRESS_NEW_FIX
extern void Please_Include_UberLame_src_NewFix_cpp();
static class CTest_NewFix_Presence {
public:
	CTest_NewFix_Presence()
	{
		Please_Include_UberLame_src_NewFix_cpp();
		// this is used only to test presence of NewFix.cpp in the project
	}
} __test_new_fix_presence;
#endif // __ULAME_SUPPRESS_NEW_FIX
#endif // __ULAME_SUPPRESS_NEW_FIX_PRESENCE_TEST
// callstack.h is usualy (or should be) included in any UberLame project
// make sure NewFix.cpp is included as well

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif // _WIN32 || _WIN64
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h> // exit()
#include <vector>
#include <algorithm>
#include "CallStack.h"
#include "StlUtils.h"
#ifdef _MULTITHREADED_STACK_GUARD
#include "Thread.h"
#endif // _MULTITHREADED_STACK_GUARD

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/*
 *								=== CCallStackMonitor ===
 */

#if __HAVE_FUNCGUARD__

std::vector<std::pair<int, const char*> > CCallStackMonitor::m_call_stack;

#ifdef _MULTITHREADED_STACK_GUARD

class CCallStackMonitor_Mutex {
public:
	static CMutex m_mutex;
};

CMutex CCallStackMonitor_Mutex::m_mutex;
#define m_mutex CCallStackMonitor_Mutex::m_mutex
// can't be CCallStackMonitor member, must not include Thread.h
// in CallStack.h before defining _ASSERTE, or things will go wrong

#endif // _MULTITHREADED_STACK_GUARD

CCallStackMonitor::CCallStackMonitor(const char *p_s_function_name)
{
#ifdef _MULTITHREADED_STACK_GUARD
	if(!m_mutex.b_Status())
		return; // this happens if other static objects are initialized before the mutex is
	if(!m_mutex.Lock() || !stl_ut::Resize_Add_1More(m_call_stack,
	   std::make_pair(CCurrentThreadHandle::n_Get_Id(), p_s_function_name)) ||
	   !m_mutex.Unlock())
		Abort();
#else // _MULTITHREADED_STACK_GUARD
	if(!stl_ut::Resize_Add_1More(m_call_stack,
	   std::make_pair(0, p_s_function_name)))
		Abort();
#endif // _MULTITHREADED_STACK_GUARD
}

class CFindById {
protected:
	int m_n_id;

public:
	inline CFindById(int n_id)
		:m_n_id(n_id)
	{}

	inline bool operator ()(const std::pair<int, const char*> &r_t_cs_item) const
	{
		return r_t_cs_item.first == m_n_id;
	}
};

CCallStackMonitor::~CCallStackMonitor()
{
#ifdef _MULTITHREADED_STACK_GUARD
	if(!m_mutex.b_Status())
		return; // this happens if other static objects are initialized before the mutex is
	if(!m_mutex.Lock())
		Abort();
	int n_id = CCurrentThreadHandle::n_Get_Id();
	std::vector<std::pair<int, const char*> >::reverse_iterator p_cs_item_it =
		std::find_if(m_call_stack.rbegin(), m_call_stack.rend(), CFindById(n_id));
	if(p_cs_item_it == m_call_stack.rend())
		Abort();
	m_call_stack.erase(p_cs_item_it.base() - 1);
	if(!m_mutex.Unlock())
		Abort();
#else // _MULTITHREADED_STACK_GUARD
	m_call_stack.erase(m_call_stack.end() - 1);
#endif // _MULTITHREADED_STACK_GUARD

#ifdef _DEBUG
	if(m_call_stack.empty()) {
		std::vector<std::pair<int, const char*> > empty_vec;
		m_call_stack.swap(empty_vec);
	}
	// attempt to completely deallocate vector not to hinder memory leak debugging
#endif // _DEBUG
}

const std::vector<std::pair<int, const char*> > &CCallStackMonitor::Get()
{
	return m_call_stack;
}

bool CCallStackMonitor::Get(std::string &r_s_stack, const char *p_s_separator)
{
	r_s_stack.erase();
	// erase

#ifdef _MULTITHREADED_STACK_GUARD
	if(!m_mutex.Lock())
		Abort();
	int n_id = CCurrentThreadHandle::n_Get_Id();
#endif // _MULTITHREADED_STACK_GUARD

	for(size_t i = 0, n = m_call_stack.size(); i < n; ++ i) {
#ifdef _MULTITHREADED_STACK_GUARD
		if(m_call_stack[i].first != n_id)
			continue;
		// ignore records for the other threads
#endif // _MULTITHREADED_STACK_GUARD

		if(!r_s_stack.empty() && p_s_separator && !stl_ut::AppendCStr(r_s_stack, p_s_separator))
			return false;
		// appends separator

		if(!stl_ut::AppendCStr(r_s_stack, m_call_stack[i].second) ||
		   !stl_ut::AppendCStr(r_s_stack, "()")) {
#ifdef _MULTITHREADED_STACK_GUARD
			if(!m_mutex.Unlock())
				Abort();
#endif // _MULTITHREADED_STACK_GUARD
			return false;
		}
		// appends function name
	}
	// concatenates function names

#ifdef _MULTITHREADED_STACK_GUARD
	if(!m_mutex.Unlock())
		Abort();
#endif // _MULTITHREADED_STACK_GUARD

	return true;
}

#endif // __HAVE_FUNCGUARD__

#if __HAVE_ASSERTE__ || __HAVE_FUNCGUARD__

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(__ARM__) && !defined(_WIN32_WCE)
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#include <intrin.h>
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _MSC_VER && !__MWERKS__ && !__ARM__ && !_WIN32_WCE
#include <signal.h>
#endif // _MSC_VER && !__MWERKS__ && !__ARM__ && !_WIN32_WCE

void CCallStackMonitor::Abort()
{
#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(__ARM__) && !defined(_WIN32_WCE)
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	//abort(); // CRT function (__asm is not supported in msvc/x64)
	__debugbreak();
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	__asm {
		int 3
	}
	// MSVC "user breakpoint"
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _MSC_VER && !__MWERKS__ && !__ARM__ && !_WIN32_WCE
	raise(SIGTRAP);
#endif // _MSC_VER && !__MWERKS__ && !__ARM__ && !_WIN32_WCE

	//exit(-1); // no need for this now, have all paths covered
	// universal solution
}

#endif // __HAVE_ASSERTE__ || __HAVE_FUNCGUARD__

#if __HAVE_ASSERTE__
#if (defined(_WIN32) || defined(_WIN64)) && !defined(_CONSOLE)

void CCallStackMonitor::AssertHandler(const char *p_s_expr,
	int n_line, const char *p_s_file, bool b_abort /*= true*/)
{
#if __HAVE_FUNCGUARD__
	std::string s_message, s_call_stack;
#ifdef _MULTITHREADED_STACK_GUARD
	int n_id = CCurrentThreadHandle::n_Get_Id();
	if(stl_ut::Format(s_message, "expression: '%s'\nfile: '%s'\nline: %d\n\ncall stack (thread 0x%04x):\n",
	   p_s_expr, p_s_file, n_line, n_id) &&
#else // _MULTITHREADED_STACK_GUARD
	if(stl_ut::Format(s_message, "expression: '%s'\nfile: '%s'\nline: %d\n\ncall stack:\n",
	   p_s_expr, p_s_file, n_line) &&
#endif // _MULTITHREADED_STACK_GUARD
	   Get(s_call_stack, " -> ") && stl_ut::Append(s_message, s_call_stack))
		MessageBoxA(0, s_message.c_str(), "Assertion failed", MB_OK | MB_ICONERROR);
	else
		MessageBoxA(0, "<Not enough memory>", "Assertion failed", MB_OK | MB_ICONERROR);
#else // __HAVE_FUNCGUARD__
	std::string s_message;
	if(stl_ut::Format(s_message, "expression: '%s'\nfile: '%s'\nline: %d",
	   p_s_expr, p_s_file, n_line))
		MessageBoxA(0, s_message.c_str(), "Assertion failed", MB_OK | MB_ICONERROR);
	else
		MessageBoxA(0, "<Not enough memory>", "Assertion failed", MB_OK | MB_ICONERROR);
#endif // __HAVE_FUNCGUARD__
	// formats and displays the message

	if(b_abort)
		Abort();
	// abort
}

#else // _WIN32 || _WIN64

void CCallStackMonitor::AssertHandler(const char *p_s_expr,
	int n_line, const char *p_s_file, bool b_abort /*= true*/)
{
	fprintf(stderr, "=== Assertion failed ===\n"
		"expression: '%s'\nfile: '%s'\nline: %d\n",
		p_s_expr, p_s_file, n_line);
	// prints message header

#if __HAVE_FUNCGUARD__
#ifdef _MULTITHREADED_STACK_GUARD
	if(!m_mutex.Lock())
		Abort();
	int n_id = CCurrentThreadHandle::n_Get_Id();
#endif // _MULTITHREADED_STACK_GUARD

#ifdef _MULTITHREADED_STACK_GUARD
	fprintf(stderr, "call stack (thread 0x%04x): ", n_id);
#else // _MULTITHREADED_STACK_GUARD
	fprintf(stderr, "call stack: ");
#endif // _MULTITHREADED_STACK_GUARD
	for(size_t i = 0, n = m_call_stack.size(), m = 0; i < n; ++ i) {
#ifdef _MULTITHREADED_STACK_GUARD
		if(m_call_stack[i].first != n_id)
			continue;
#endif // _MULTITHREADED_STACK_GUARD

		fprintf(stderr, (m)? " -> %s" : "%s", m_call_stack[i].second);

		++ m;
	}
	fprintf(stderr, "\n");
	// prints call stack contents

#ifdef _MULTITHREADED_STACK_GUARD
	if(!m_mutex.Unlock())
		Abort();
#endif // _MULTITHREADED_STACK_GUARD
#endif // __HAVE_FUNCGUARD__

	if(b_abort)
		Abort();
	// abort
}

#endif // _WIN32 || _WIN64
#endif // __HAVE_ASSERTE__

/*
 *								=== ~CCallStackMonitor ===
 */
