/*
								+--------------------------------+
								|                                |
								| ***  Inline min/max funcs  *** |
								|                                |
								|  Copyright © -tHE SWINe- 2008  |
								|                                |
								|            MinMax.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __INLINED_MIN_MAX_TEMPLATES_INCLUDED
#define __INLINED_MIN_MAX_TEMPLATES_INCLUDED

/**
 *	@file MinMax.h
 *	@author -tHE SWINe-
 *	@date 2008
 *	@brief minimum / maximum functions
 *
 *	@date 2009-04-23
 *
 *	added \#define NOMINMAX to avoid min / max macro definition
 *	in case windows.h is included after this file.
 *
 *	changed min / max comparation sign from ">" to "<", so prerequisities for use are the
 *	same as for std::min / std::max (for _Ty to be less-than-comparable)
 *
 *	there are min, max functions in std algorithm header, but they are suboptimal.
 *	when compiling the following code with MSVC6 for release, optimize for speed,
 *	inline expansion enabled:
 *
 *	{
 *		int a, b;
 *		scanf("%d%d", &a, &b);
 *		// a, b (not constant, can't optimize)
 *
 *		int c = std::min(a, b);
 *		int d = min(a, b);
 *		// test
 *
 *		printf("%d %d", c, d);
 *		// do not optimize away c and d
 *	}
 *
 *	then std::min gives this code:
 *
 *	  00018	8b 44 24 0c	 mov	 eax, DWORD PTR _b$[esp+20]
 *	  0001c	8b 4c 24 10	 mov	 ecx, DWORD PTR _a$[esp+20]
 *	  00020	83 c4 0c	 add	 esp, 12			; 0000000cH
 *	  00023	3b c1		 cmp	 eax, ecx
 *	  00025	8d 54 24 00	 lea	 edx, DWORD PTR _b$[esp+8]
 *	  00029	7c 04		 jl	 SHORT $L4883
 *	  0002b	8d 54 24 04	 lea	 edx, DWORD PTR _a$[esp+8]
 *	$L4883:
 *	  0002f	8b 12		 mov	 edx, DWORD PTR [edx]
 *
 *	while ÜberLame min gives a bit shorter code:
 *
 *	  00022	8b 44 24 14	 mov	 eax, DWORD PTR _a$[esp+84]
 *	  00026	8b 54 24 18	 mov	 edx, DWORD PTR _b$[esp+84]
 *	  0002a	83 c4 0c	 add	 esp, 12			; 0000000cH
 *	  0002d	3b c2		 cmp	 eax, edx
 *	  0002f	8b c8		 mov	 ecx, eax
 *	  00031	7c 02		 jl	 SHORT $L51125
 *	  00033	8b ca		 mov	 ecx, edx
 *	$L51125:
 *
 *	this is because std::min uses references, while ÜberLame min doesn't. on the other
 *	hand, when comparing more complex objects (not primitive types), std::min code
 *	works better. that's why Überlame code is specialized for use with primitive types
 *	and for use with user-defined types. that makes ÜberLame version at least as fast
 *	(if not faster) as std version in about any situation.
 *
 *	added specialized minmax functions with implicit type-cast, allowing compiler
 *	to make some optimalizations that cannot be done when using std::min / std::max
 *	and casting simpler data type before entering the function. this functionality
 *	can be disabled using DISABLE_EXTENDED_MINMAX_WITH_CONVERSION (support just
 *	implicit conversions for int / float, int / double or float / double arguments),
 *	or using DISABLE_MINMAX_WITH_CONVERSION to disable this completely.
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 */

#ifndef NOMINMAX
/**
 *	@def NOMINMAX
 *
 *	@brief avoid min, max macro generation in windows.h (MSVC)
 */
#define NOMINMAX
#endif

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
// in case min max was defined already, undefine it

/*
 *								=== inlined min, max ===
 */

#define __Specialized_MinMax(_Ty) \
	/** \
	 *	@brief minimum of two values \
	 * \
	 *	@return Returns less of a, b. \
	 */ \
	static inline _Ty min(_Ty a, _Ty b) \
	{ \
		return (a < b)? a : b; \
	} \
	/** \
	 *	@brief maximum of two values \
	 * \
	 *	@return Returns greater of a, b. \
	 */ \
	static inline _Ty max(_Ty a, _Ty b) \
	{ \
		return (a < b)? b : a; \
	}
// macro, defining specialized min / max for a single primitive type

#define __Specialized_MinMax_Conversion(_HiTy,_LoTy) \
	/** \
	 *	@brief minimum of two values \
	 * \
	 *	@return Returns less of a, b. \
	 */ \
	static inline _HiTy min(_HiTy a, _LoTy b) \
	{ \
		return (a < (_HiTy)(b))? a : b; \
	} \
	/** \
	 *	@brief maximum of two values \
	 * \
	 *	@return Returns greater of a, b. \
	 */ \
	static inline _HiTy max(_HiTy a, _LoTy b) \
	{ \
		return (a < (_HiTy)(b))? b : a; \
	} \
	/** \
	 *	@brief minimum of two values \
	 * \
	 *	@return Returns less of a, b. \
	 */ \
	static inline _HiTy min(_LoTy a, _HiTy b) \
	{ \
		return ((_HiTy)(a) < b)? a : b; \
	} \
	/** \
	 *	@brief maximum of two values \
	 * \
	 *	@return Returns greater of a, b. \
	 */ \
	static inline _HiTy max(_LoTy a, _HiTy b) \
	{ \
		return ((_HiTy)(a) < b)? b : a; \
	}
// macro, defining specialized min / max for two primitive types with conversion
// (can correctly calculate min(int float) without explicit conversion)

__Specialized_MinMax(unsigned char)
__Specialized_MinMax(char)
__Specialized_MinMax(unsigned short)
__Specialized_MinMax(short)
__Specialized_MinMax(unsigned int)
__Specialized_MinMax(int)
__Specialized_MinMax(unsigned long)
__Specialized_MinMax(long)
__Specialized_MinMax(float)
__Specialized_MinMax(double)
// basic C / C++ types

#if !defined(DISABLE_EXTENDED_MINMAX_WITH_CONVERSION) && !defined(DISABLE_MINMAX_WITH_CONVERSION)
__Specialized_MinMax_Conversion(double, float)
__Specialized_MinMax_Conversion(double, long)
__Specialized_MinMax_Conversion(double, unsigned long)
__Specialized_MinMax_Conversion(double, int)
__Specialized_MinMax_Conversion(double, unsigned int)
__Specialized_MinMax_Conversion(double, short)
__Specialized_MinMax_Conversion(double, unsigned short)
__Specialized_MinMax_Conversion(double, char)
__Specialized_MinMax_Conversion(double, unsigned char)

__Specialized_MinMax_Conversion(float, long)
__Specialized_MinMax_Conversion(float, unsigned long)
__Specialized_MinMax_Conversion(float, int)
__Specialized_MinMax_Conversion(float, unsigned int)
__Specialized_MinMax_Conversion(float, short)
__Specialized_MinMax_Conversion(float, unsigned short)
__Specialized_MinMax_Conversion(float, char)
__Specialized_MinMax_Conversion(float, unsigned char)

//__Specialized_MinMax_Conversion(long, unsigned long) // this may be unsafe
__Specialized_MinMax_Conversion(long, int)
//__Specialized_MinMax_Conversion(long, unsigned int) // this may be unsafe
__Specialized_MinMax_Conversion(long, short)
__Specialized_MinMax_Conversion(long, unsigned short)
__Specialized_MinMax_Conversion(long, char)
__Specialized_MinMax_Conversion(long, unsigned char)

//__Specialized_MinMax_Conversion(unsigned long, int) // this may be unsafe
__Specialized_MinMax_Conversion(unsigned long, unsigned int)
__Specialized_MinMax_Conversion(unsigned long, short)
__Specialized_MinMax_Conversion(unsigned long, unsigned short)
__Specialized_MinMax_Conversion(unsigned long, char)
__Specialized_MinMax_Conversion(unsigned long, unsigned char)

//__Specialized_MinMax_Conversion(int, unsigned int) // this may be unsafe
__Specialized_MinMax_Conversion(int, short)
__Specialized_MinMax_Conversion(int, unsigned short)
__Specialized_MinMax_Conversion(int, char)
__Specialized_MinMax_Conversion(int, unsigned char)

__Specialized_MinMax_Conversion(unsigned int, short)
__Specialized_MinMax_Conversion(unsigned int, unsigned short)
__Specialized_MinMax_Conversion(unsigned int, char)
__Specialized_MinMax_Conversion(unsigned int, unsigned char)

//__Specialized_MinMax_Conversion(short, unsigned short) // this may be unsafe
__Specialized_MinMax_Conversion(short, char)
__Specialized_MinMax_Conversion(short, unsigned char)

__Specialized_MinMax_Conversion(unsigned short, char)
__Specialized_MinMax_Conversion(unsigned short, unsigned char)
#elif !defined(DISABLE_MINMAX_WITH_CONVERSION)
__Specialized_MinMax_Conversion(float, int)
__Specialized_MinMax_Conversion(double, int)
__Specialized_MinMax_Conversion(double, float)
#endif // !DISABLE_EXTENDED_MINMAX_WITH_CONVERSION && !DISABLE_MINMAX_WITH_CONVERSION
// basic C / C++ types with automatic conversion (may be evil, hence the #disable)
// in case you wish to use only basic types, #define DISABLE_EXTENDED_MINMAX_WITH_CONVERSION
// in case you don't wish to use this feature at all, #define DISABLE_MINMAX_WITH_CONVERSION

#if defined(_MSC_VER) && !defined(__MWERKS__) && (_MSC_VER <= 1300) // up to MSVC60
#define ENABLE_MINMAX_ON_STDINTS
#endif
// MSVC 6.0 can use ENABLE_MINMAX_ON_STDINTS, it has it's own integer types "[unsigned] __int<nn>"

#ifdef ENABLE_MINMAX_ON_STDINTS
#include "Integer.h"

__Specialized_MinMax(int8_t)
__Specialized_MinMax(uint8_t)
__Specialized_MinMax(int16_t)
__Specialized_MinMax(uint16_t)
__Specialized_MinMax(int32_t)
__Specialized_MinMax(uint32_t)
__Specialized_MinMax(int64_t)
__Specialized_MinMax(uint64_t)
// standard integer types (do not define minmax with conversion here)
#endif // ENABLE_MINMAX_ON_STDINTS
// note this may confuse some compilers as int8_t may be defined as char
// therefore effectively re-defining functions we already have

/**
 *	@brief universal min template, using references (faster for classes, slower for primitive types)
 *
 *	@return Returns a if a < b returns true, otherwise returns b.
 */
template <class _Ty>
static inline const _Ty &min(const _Ty &a, const _Ty &b)
{
	return (a < b)? a : b;
}

/**
 *	@brief universal max template, using references (faster for classes, slower for primitive types)
 *
 *	@return Returns b if a < b returns true, otherwise returns a.
 */
template <class _Ty>
static inline const _Ty &max(const _Ty &a, const _Ty &b)
{
	return (a < b)? b : a;
}

/*
 *								=== ~inlined min, max ===
 */

#endif // !__INLINED_MIN_MAX_TEMPLATES_INCLUDED
