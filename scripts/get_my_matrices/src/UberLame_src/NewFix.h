/*
								+----------------------------------+
								|                                  |
								|    ***  Operator new fix  ***    |
								|                                  |
								|   Copyright © -tHE SWINe- 2009   |
								|                                  |
								|             NewFix.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __OPERATOR_NEW_EXCEPTION_HANDLING_COMPATIBILITY_FIX_INCLUDED
#define __OPERATOR_NEW_EXCEPTION_HANDLING_COMPATIBILITY_FIX_INCLUDED

/**
 *	@file NewFix.h
 *	@author -tHE SWINe-
 *	@date 2009
 *	@brief operator new fix for MSVC
 *
 *	@date 2009-09-13
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
 *	@date 2010-10-18
 *
 *	Apparently, g++ doesn't know new.h (added \#ifdef).
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 */

#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER > 1200
/**
 *	@def __ULAME_SUPPRESS_NEW_FIX
 *
 *	@brief this fix is only needed in MSVC 6.0
 */
#define __ULAME_SUPPRESS_NEW_FIX
#endif // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200

#include <new>

#ifndef __ULAME_SUPPRESS_NEW_FIX

/**
 *	@def __ULAME_FIX_NEW_STD_NOTHROW
 *	@brief If defined, operator new(std::nothrow) is fixed in such
 *		  a way it doesn't throw std::bad_alloc on failure.
 *	@note This file needs to be included prior using
 *		  new(std::nothrow) to have any effect.
 */
#define __ULAME_FIX_NEW_STD_NOTHROW

/**
 *	@def __ULAME_FIX_NEW
 *	@brief if defined, a new operator new failure handler is set, which
 *		  guarantees new throws std::bad_alloc instead of returning null pointer
 *	@note This only works in windows.
 */
#define __ULAME_FIX_NEW

#ifdef __ULAME_FIX_NEW_STD_NOTHROW

/**
 *	@brief patched operator new (std::nothrow)
 *
 *	@param[in] n_size is required allocation size in bytes
 *
 *	@return Returns pointer ot allocated block of memory, or null on failure.
 */
void *__cdecl operator new(size_t n_size, const std::nothrow_t&) throw();

/**
 *	@brief operator delete for the patched operator new (std::nothrow)
 *
 *	@param[in] p_mem is pointer to memory to be deleted
 */
inline void __cdecl operator delete(void *p_mem, const std::nothrow_t&) throw() { delete p_mem; }

#endif // __ULAME_FIX_NEW_STD_NOTHROW

#endif // !__ULAME_SUPPRESS_NEW_FIX

#endif // !__OPERATOR_NEW_EXCEPTION_HANDLING_COMPATIBILITY_FIX_INCLUDED

