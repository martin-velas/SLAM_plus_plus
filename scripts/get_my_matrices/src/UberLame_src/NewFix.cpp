/*
								+----------------------------------+
								|                                  |
								|    ***  Operator new fix  ***    |
								|                                  |
								|   Copyright © -tHE SWINe- 2009   |
								|                                  |
								|            NewFix.cpp            |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file NewFix.cpp
 *	@date 2009
 *	@author -tHE SWINe-
 *	@brief operator new fix
 *
 *	this file patches operator new behavior in msvc so it throws std::bad_alloc
 *	in case it runs out of memory, instead of returning null pointer.
 *
 *	as a consequence of this patch, operator new(std::nothrow) throws exception
 *	as well, so it's proper version is supplied here as well (note this file needs
 *	to be included before using new(std::nothrow) to take effect)
 *
 *	=== Important ===
 *
 *	note in case this is used with MFC, or in case this is linked with DLL of the
 *	C runtime library (Debug Multithreaded DLL or Multithreaded DLL) then the
 *	new handler is effectively installed into the runtime DLL, which means that any
 *	DLL file that loads into the process address space that also links against
 *	a matching version of the runtime library DLL will be affected by this handler
 *	(new will throw an exception on failure).
 *
 *	note problems could arise when STL was using new(std::nothrow), but the SGI
 *	implementation does not (get one at http://www.sgi.com/tech/stl/download.html).
 *	MSVC 6.0 Hewlett-Packard uses new(std::nothrow), but only at a few places.
 *
 *	note this is based on article:
 *	James Hebben, "Don't Let Memory Allocation Failures Crash Your Legacy STL Application",
 *	http://msdn.microsoft.com/en-us/magazine/cc164087.aspx
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 *	NewFix is now compiled under MSVC 6.0 only (can give link-time warning about
 *	no public symbols in NewFix.obj under newer / non-msvc compilers)
 *
 */

#include "NewFix.h"
#include "CallStack.h"
#include "Integer.h"

#include <new>
#if defined(_MSC_VER) && !defined(__MWERKS__)
#include <new.h>
#endif // MSVC

#ifndef __ULAME_SUPPRESS_NEW_FIX

void Please_Include_UberLame_src_NewFix_cpp()
{
	// this is used only to test presence of NewFix.cpp in the project
}

#endif // __ULAME_SUPPRESS_NEW_FIX

#ifdef __ULAME_FIX_NEW_STD_NOTHROW

void *__cdecl operator new(size_t n_size, const std::nothrow_t&) throw()
{
	uint8_t *p;
	try {
		p = new uint8_t[n_size];
	} catch(std::bad_alloc) {
		p = 0;
	}
	return p;
}

#endif // __ULAME_FIX_NEW_STD_NOTHROW

#ifdef __ULAME_FIX_NEW

#pragma warning(disable: 4073)
// disable warning 4073 (initializers put in library initialization area)

#pragma init_seg(lib)

namespace {

static class CNewHandlerFixup {
protected:
	_PNH m_old_new_handler;

public:
	CNewHandlerFixup() 
	{
		m_old_new_handler = _set_new_handler(new_handler);
	}

	~CNewHandlerFixup() 
	{
		_set_new_handler(m_old_new_handler);
	}

protected:
	static int __cdecl new_handler(size_t)
	{
		throw std::bad_alloc();
		return 0;
	}
} __NewHandler__;

}

#endif // __ULAME_FIX_NEW

/*
 *	-end-of-file-
 */
