/*
								+-----------------------------------+
								|                                   |
								| ***  Unused macro definition  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2010  |
								|                                   |
								|             Unused.h              |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __UNUSED_INCLUDED
#define __UNUSED_INCLUDED

/**
 *	@file include/slam/Unused.h
 *	@brief The UNUSED() macro definition
 *	@author -tHE SWINe-
 *	@date 2010-11-25
 *
 *	@date 2014-01-31
 *
 *	Added UNUSED implementation for Visual Studio (presumably works
 *	since 2003 and up, definitely does work in 2008).
 */

#ifndef UNUSED
/**
 *	@def UNUSED
 *	@brief marks function argument	 / variable as deliberately unused
 *	@param x is parameter to be marked as unused
 *	@note This is especially useful for template programming or defining common interface classes
 *		where functions having unused parameters are pretty common cause of numerous g++ warnings.
 */
#if defined(__GNUC__)
#define UNUSED(x) x __attribute__((unused))
#define UNUSED_VAR(x) UNUSED(x)
#elif defined(__LCLINT__)
#define UNUSED(x) /*@unused@*/ x
#define UNUSED_VAR(x) UNUSED(x)
#elif defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
#define UNUSED(x) x
// the below macro causes problems with intellisense, let's disable it
//#define UNUSED(x) __pragma(warning(suppress: 4100 4101)) x
// 4100 is unreferenced argument and 4101 is unreferenced local variable in msvc (does not work in VC 6.0)

#define UNUSED_VAR(x) __pragma(warning(suppress: 4101)) x
// might not throw IS off as much
#else // __GNUC__
#define UNUSED(x) x
#define UNUSED_VAR(x) UNUSED(x)
#endif // __GNUC__
#endif // UNUSED

#endif // !__UNUSED_INCLUDED
