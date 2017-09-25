/*
								+-----------------------------------+
								|                                   |
								| ***  Unused macro definition  *** |
								|                                   |
								|   Copyright  © -tHE SWINe- 2010   |
								|                                   |
								|             Unused.h              |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __UNUSED_INCLUDED
#define __UNUSED_INCLUDED

/**
 *	@file Unused.h
 *	@brief The UNUSED() macro definition
 *	@author -tHE SWINe-
 *	@date 2010-11-25
 *
 *	@date 2012-06-07
 *
 *	Moved documentation for the UNUSED inside the block so that doxygen can find it.
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 *	@date 2014-01-31
 *
 *	Added UNUSED implementation for Visual Studio (presumably works
 *	since 2003 and up, definitely does work in 2008).
 *
 */

/**
 *	@def __UNUSED_BREAKS_INTELLISENSE
 *	@brief if defined, <tt>UNUSED(x)</tt> evaluates to <tt>x</tt> in Visual Studio,
 *		where it breaks the Intellisense (in 2008 it does); has no effect on other compilers
 */
#define __UNUSED_BREAKS_INTELLISENSE

#ifndef UNUSED
/**
 *	@def UNUSED
 *	@brief marks function argument / variable as deliberately unused
 *	@param x is parameter to be marked as unused
 *	@note This is especially useful for template programming or defining common interface classes
 *		where functions having unused parameters are pretty common cause of numerous g++ warnings.
 */
#if defined(__GNUC__)
#define UNUSED(x) x __attribute__((unused))
#elif defined(__LCLINT__)
#define UNUSED(x) /*@unused@*/ x
#elif defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
#ifdef __UNUSED_BREAKS_INTELLISENSE
#define UNUSED(x) x
#else // __UNUSED_BREAKS_INTELLISENSE
#define UNUSED(x) __pragma(warning(suppress: 4100 4101)) x
// 4100 is unreferenced argument and 4101 is unreferenced local variable in msvc (does not work in VC 6.0)
#endif // __UNUSED_BREAKS_INTELLISENSE
#else // __GNUC__
#define UNUSED(x) x
#endif // __GNUC__
#endif // UNUSED

#endif // !__UNUSED_INCLUDED
