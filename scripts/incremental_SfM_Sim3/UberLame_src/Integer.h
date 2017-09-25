/*
								+--------------------------------+
								|                                |
								| *** Portable integer types *** |
								|                                |
								|  Copyright © -tHE SWINe- 2008  |
								|                                |
								|            Integer.h           |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __INTEGER_TYPES_INCLUDED
#define __INTEGER_TYPES_INCLUDED

/**
 *	@file Integer.h
 *	@author -tHE SWINe-
 *	@date 2008
 *	@brief standard integer types
 *
 *	@note This was never tested on compilers other than MSVC or g++, in case you are using another
 *		compiler, make sure either inttypes.h is included, or that the integer types really have
 *		size and signedness they are supposed to.
 *
 *	@date 2008-08-02
 *
 *	added some template-based code for determining signedness and maximal value of integer types
 *
 *	@date 2008-08-08
 *
 *	added \#ifdef for windows 64
 *
 *	@date 2009-11-12
 *
 *	added macro SIZE_MAX for maximal size of size_t type (assumes it is unsigned)
 *
 *	@date 2010-10-18
 *
 *	Added some common "bit hack" functions: b_Is_POT(), n_Make_POT(), n_Make_Lower_POT(),
 *	n_Align_Up(), n_Align_Up_POT(), n_SetBit_Num() and n_Log2().
 *
 *	@date 2010-10-29
 *
 *	Unified windows detection macro to "\#if defined(_WIN32) || defined(_WIN64)".
 *
 *	@date 2010-11-25
 *
 *	Added compile-time versions of the common "bit hack" functions: b_Is_POT_Static(), n_Make_POT_Static(),
 *	n_Make_Lower_POT_Static(), n_Align_Up_Static(), n_Align_Up_POT_Static(), n_SetBit_Num_Static()
 *	and n_Log2_Static()
 *
 *	@date 2010-11-28
 *
 *	Note for linux builds - the size_t type is not declared in this header, and must be declared
 *	before including this header. Including "new" or "sys/types.h" should do.
 *
 *	@date 2011-07-18
 *
 *	Changed definitions of signed integer types int8_t, int16_t and int32_t (added explicit 'signed').
 *
 *	@date 2011-11-01
 *
 *	Changed some 64-bit integer constants to two-part 32-bit constants to avoid some g++ complaints.
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2012-06-21
 *
 *	Added the n_RightFill_Ones(), n_LeadingZero_Num() and n_Mask() functions.
 *
 *	@date 2013-01-13
 *
 *	Added the PRIsize and PRIdiff macros for printing size_t and ptrdiff_t integer types.
 *
 *	@date 2014-07-16
 *
 *	Added the n_Power_Static() and f_Power_Static() macros for calculating integer powers
 *	of integer base (both positive and negative), at compile time.
 *
 */

#include "CallStack.h"
#include "Unused.h"
#include <stddef.h> // needed by Ubuntu
#include <limits.h>

#if !defined(_WIN32) && !defined(_WIN64)
#include <sys/types.h>
#endif // !_WIN32 && !_WIN64
// for linux builds

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#elif defined(__sun__)
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#if defined(__STDC__)
typedef char int8_t; /**< 8 bit signed integer */
typedef unsigned char uint8_t; /**< 8 bit unsigned integer */
typedef short int16_t; /**< 16 bit signed integer */
typedef unsigned short uint16_t; /**< 16 bit unsigned integer */
typedef int int32_t; /**< 32 bit signed integer */
typedef unsigned int uint32_t; /**< 32 bit unsigned integer */
#if defined(__arch64__)
typedef long int64_t; /**< 64 bit signed integer */
typedef unsigned long uint64_t; /**< 64 bit unsigned integer */
#else // __arch64__
typedef long long int int64_t; /**< 64 bit signed integer */
typedef unsigned long long int uint64_t; /**< 64 bit unsigned integer */
#endif // __arch64__
#endif // __STDC__
#elif defined(__VMS)
#include <inttypes.h>
#elif defined(__SCO__) || defined(__USLC__)
#include <stdint.h>
#elif defined(__UNIXOS2__) || defined(__SOL64__)
typedef char int8_t; /**< 8 bit signed integer */
typedef unsigned char uint8_t; /**< 8 bit unsigned integer */
typedef short int16_t; /**< 16 bit signed integer */
typedef unsigned short uint16_t; /**< 16 bit unsigned integer */
typedef long int int32_t; /**< 32 bit signed integer */
typedef unsigned long int uint32_t; /**< 32 bit unsigned integer */
typedef long long int int64_t; /**< 64 bit signed integer */
typedef unsigned long long int uint64_t; /**< 64 bit unsigned integer */
#elif defined(_WIN32) && defined(__GNUC__) // MinGW
#include <stdint.h>
#elif defined(_WIN32) || defined(_WIN64) // MSVC
typedef signed __int8 int8_t; /**< 8 bit signed integer */
typedef signed __int16 int16_t; /**< 8 bit unsigned integer */
typedef signed __int32 int32_t; /**< 16 bit signed integer */
typedef signed __int64 int64_t; /**< 16 bit unsigned integer */
typedef unsigned __int8 uint8_t; /**< 32 bit signed integer */
typedef unsigned __int16 uint16_t; /**< 32 bit unsigned integer */
typedef unsigned __int32 uint32_t; /**< 64 bit signed integer */
typedef unsigned __int64 uint64_t; /**< 64 bit unsigned integer */
#else // __STDC_VERSION__ && __STDC_VERSION__ >= 199901L
#define __STDC_FORMAT_MACROS
#include <inttypes.h> // fallback
#endif // __STDC_VERSION__ && __STDC_VERSION__ >= 199901L
// define int types

#ifndef UINT8_MAX
/**
 *	@brief maximal value of 8 bit unsigned integer (255)
 */
#define UINT8_MAX  (uint8_t(-1))
#endif

#ifndef UINT16_MAX
/**
 *	@brief maximal value of 16 bit unsigned integer (65535)
 */
#define UINT16_MAX (uint16_t(-1))
#endif

#ifndef UINT32_MAX
/**
 *	@brief maximal value of 32 bit unsigned integer (4294967295)
 */
#define UINT32_MAX (uint32_t(-1))
#endif

#ifndef UINT64_MAX
/**
 *	@brief maximal value of 64 bit unsigned integer (18446744073709551615)
 */
#define UINT64_MAX (uint64_t(-1))
#endif

#ifndef INT8_MAX
/**
 *	@brief maximal value of 8 bit signed integer (127)
 */
#define INT8_MAX  (int8_t(UINT8_MAX / 2))
#endif

#ifndef INT16_MAX
/**
 *	@brief maximal value of 16 bit signed integer (32767)
 */
#define INT16_MAX (int16_t(UINT16_MAX / 2))
#endif

#ifndef INT32_MAX
/**
 *	@brief maximal value of 32 bit signed integer (2147483647)
 */
#define INT32_MAX (int32_t(UINT32_MAX / 2))
#endif

#ifndef INT64_MAX
/**
 *	@brief maximal value of 64 bit signed integer (9223372036854775807)
 */
#define INT64_MAX (int64_t(UINT64_MAX / 2))
#endif

#ifndef INT8_MIN
/**
 *	@brief minimal value of 8 bit signed integer (-128)
 */
#define INT8_MIN  (int8_t(-INT8_MAX - 1))
#endif

#ifndef INT16_MIN
/**
 *	@brief minimal value of 16 bit signed integer (-32768)
 */
#define INT16_MIN (int16_t(-INT16_MAX - 1))
#endif

#ifndef INT32_MIN
/**
 *	@brief minimal value of 32 bit signed integer (-2147483648)
 */
#define INT32_MIN (int32_t(-INT32_MAX - 1))
#endif

#ifndef INT64_MIN
/**
 *	@brief minimal value of 64 bit signed integer (-9223372036854775808)
 */
#define INT64_MIN (int64_t(-INT64_MAX - 1))
#endif

#ifndef SIZE_MAX
/**
 *	@brief maximal value of type size_t (same as UINT32_MAX or UINT64_MAX)
 */
#define SIZE_MAX  (size_t(-1))
#endif
// calc limits

/**
 *	@def PRIu64
 *	@brief printf format strings for unsigned 64-bit integer
 */
/**
 *	@def PRI64
 *	@brief printf format strings for signed 64-bit integer
 */
/**
 *	@def PRIx64
 *	@brief printf format strings for lowercase hexadecimal unsigned 64-bit integer
 */
/**
 *	@def PRIX64
 *	@brief printf format strings for uppercase hexadecimal signed 64-bit integer
 */
/**
 *	@def _PRIu64
 *	@brief printf format strings for unsigned 64-bit integer, without the percent character
 */
/**
 *	@def _PRI64
 *	@brief printf format strings for signed 64-bit integer, without the percent character
 */
/**
 *	@def _PRIx64
 *	@brief printf format strings for lowercase hexadecimal unsigned 64-bit integer, without the percent character
 */
/**
 *	@def _PRIX64
 *	@brief printf format strings for uppercase hexadecimal signed 64-bit integer, without the percent character
 */
#if defined(_MSC_VER) && !defined(__MWERKS__)
#ifndef PRIu64
#define PRIu64 "%I64u"
#endif // !PRIu64
#ifndef PRI64
#define PRI64 "%I64d"
#endif // !PRI64
#ifndef PRIx64
#define PRIx64 "%I64x"
#endif // !PRIx64
#ifndef PRIX64
#define PRIX64 "%I64X"
#endif // !PRIX64
#ifndef _PRIu64
#define _PRIu64 "I64u"
#endif // !_PRIu64
#ifndef _PRI64
#define _PRI64 "I64d"
#endif // !_PRI64
#ifndef _PRIx64
#define _PRIx64 "I64x"
#endif // !_PRIx64
#ifndef _PRIX64
#define _PRIX64 "I64X"
#endif // !_PRIX64
#else // _MSC_VER && !__MWERKS__
#ifndef PRIu64
#define PRIu64 "%llu"
#endif // !PRIu64
#ifndef PRI64
#define PRI64 "%ll"
#endif // !PRI64
#ifndef PRIx64
#define PRIx64 "%llx"
#endif // !PRIx64
#ifndef PRIX64
#define PRIX64 "%llX"
#endif // !PRIX64
#ifndef _PRIu64
#define _PRIu64 "llu"
#endif // !_PRIu64
#ifndef _PRI64
#define _PRI64 "ll"
#endif // !_PRI64
#ifndef _PRIx64
#define _PRIx64 "llx"
#endif // !_PRIx64
#ifndef _PRIX64
#define _PRIX64 "llX"
#endif // !_PRIX64
#endif // _MSC_VER && !__MWERKS__
// printf format strings for 64-bit integers

/**
 *	@def PRId64
 *	@copydoc PRI64
 */
#ifndef PRId64
#define PRId64 PRI64
#endif // !PRId64

/**
 *	@def _PRId64
 *	@copydoc _PRI64
 */
#ifndef _PRId64
#define _PRId64 _PRI64
#endif // !_PRId64

/**
 *	@def PRIsize
 *	@brief printf format strings for size_t
 */
/**
 *	@def _PRIsize
 *	@brief printf format strings for size_t, without the percent character
 */
#ifndef PRIsize
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#define PRIsize "%I64u"
#define _PRIsize "I64u"
// %u doesn't work correctly if compiling for x64
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#define PRIsize "%u"
#define _PRIsize "u"
// %u is ok for x86
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _MSC_VER && !__MWERKS__
#define PRIsize "%zd"
#define _PRIsize "zd" // was zd but want to use it for x as well
#endif // _MSC_VER && !__MWERKS__
#endif // !PRIsize

/**
 *	@def PRIsizex
 *	@brief printf format strings for size_t and hexadecimal
 */
/**
 *	@def _PRIsizex
 *	@brief printf format strings for size_t and hexadecimal, without the percent character
 */
#ifndef PRIsizex
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#define PRIsizex "%I64x"
#define _PRIsizex "I64x"
// %x doesn't work correctly if compiling for x64
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#define PRIsizex "%x"
#define _PRIsizex "x"
// %x is ok for x86
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _MSC_VER && !__MWERKS__
#define PRIsizex "%zx"
#define _PRIsizex "zx"
#endif // _MSC_VER && !__MWERKS__
#endif // !PRIsizex

/**
 *	@def PRIdiff
 *	@brief printf format strings for ptrdiff_t
 */
/**
 *	@def _PRIdiff
 *	@brief printf format strings for ptrdiff_t, without the percent character
 */
#ifndef PRIdiff
#if defined(_MSC_VER) && !defined(__MWERKS__)
#define PRIdiff "%ld"
#define _PRIdiff "ld"
#else // _MSC_VER && !__MWERKS__
#define PRIdiff "%td"
#define _PRIdiff "td"
#endif // _MSC_VER && !__MWERKS__
#endif // !PRIsize

/*
 *								=== templates for maximal value of integer types ===
 */

/**
 *	@brief data type signedness predicate
 *
 *	Decides wheter a type is signed, or not. Value of CIsSigned<_Ty>::result
 *		is true if type _Ty is signed, or false if it's unsigned.
 *
 *	@tparam _Ty is type tested for being signed
 */
template <class _Ty>
struct CIsSigned {
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = _Ty(-1) < 1 /**< contains true if type _Ty is signed, or false if it's unsigned */
	};
	// should be '_Ty(-1) < 0', using 1 to avoid g++ warning 'unsigned comparison < 0 is always false'
};

/**
 *	@brief integer type predicate
 *
 *	Decides wheter a type is integer, or floating point. Value of CIsInteger<_Ty>::result
 *		is set to true if _Ty is integer type, otherwise it's false.
 *
 *	@tparam _Ty is type tested for being integer
 */
template <class _Ty>
struct CIsInteger {
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = _Ty(.5) == 0 /**< is set to true if _Ty is integer type, otherwise it's false */
	};
};

/**
 *	@brief maximal integer value template
 *
 *	Determines maximal value of integer type _Ty, result is returned by CMaxIntValue<_Ty>::result().
 *
 *	@tparam _Ty is type to determine maximal value of
 *
 *	@note _Ty must be integer type, it won't work with float or double.
 */
template <class _Ty>
struct CMaxIntValue {
private:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		n_bit_num = 8 * sizeof(_Ty),
		b_unsigned = (CIsSigned<_Ty>::result)? 0 : 1
	};

/**
 *	@brief this calculates maximal value of _Ty as 64-bit unsigned integer
 *	@note this may not be used outside CMaxIntValue
 */
#define _Ty_max (((uint64_t(1) << (n_bit_num - 1)) - 1) + b_unsigned * (uint64_t(1) << (n_bit_num - 1)))

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400 // MSVC 6.0
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		n_result_hi = uint32_t(_Ty_max >> 32),
		n_result_lo = uint32_t(_Ty_max & 0xffffffff)
	};
	// msvc doesn't allow static const initialization so we keep it as hi-lo
	// enum values (note this relies on enum values being 32-bit integers)

public:
	/**
	 *	@brief gets maximal value of type _Ty
	 *
	 *	@return Returns maximal value of type _Ty.
	 *
	 *	@note Result is uint64_t, in case larger types are implemented by your compiler,
	 *		this will not work for them.
	 *	@note This doesn't carry-out any calculations, it is just not possible to store
	 *		64-bit number using enum.
	 */
	static inline uint64_t result()
	{
		_ASSERTE(CIsInteger<_Ty>::result); // only works for integers
		return (uint64_t(n_result_hi) << 32) | uint32_t(n_result_lo);
	}
#else // _MSC_VER && !__MWERKS__ &&_MSC_VER < 1400
	static const uint64_t n_result = _Ty_max; // g++ allows this (preferable)

public:
	/**
	 *	@brief gets maximal value of type _Ty
	 *
	 *	@return Returns maximal value of type _Ty.
	 *
	 *	@note Result is uint64_t, in case larger types are implemented by your compiler,
	 *		this will not work for them.
	 *	@note This doesn't carry-out any calculations, it is just not possible to store
	 *		64-bit number using enum.
	 */
	static inline uint64_t result()
	{
		_ASSERTE(CIsInteger<_Ty>::result); // only works for integers
		return n_result;
	}
#endif // _MSC_VER && !__MWERKS__ &&_MSC_VER < 1400

#undef _Ty_max
};

/**
 *	@brief gets maximal value of type _Ty
 *	@tparam _Ty is integer data type
 *	@param[in] x is parameter of type _Ty (the value is ignored here)
 *	@return Returns maximal value of type _Ty.
 *	@note Result is uint64_t, in case larger types are implemented by your compiler,
 *		this will not work for them.
 */
template <class _Ty>
inline uint64_t n_MaxIntValue(_Ty UNUSED(x))
{
	return CMaxIntValue<_Ty>::result();
}

/*
 *								=== ~templates for maximal value of integer types ===
 */

/*
 *								=== bit hacks ===
 */

/**
 *	@brief hides inside implementation of helper classes for the fast bit hacks
 */
namespace ul_bithacks {

/**
 *	@def MSVC_OMMIT
 *	@param[in] x is a declaration / statement
 *	@brief ommits the argument when compiled on MSVC, otherwise resolves to the argument
 */
/**
 *	@def MSVC_OMMIT_ARG
 *	@param[in] x is a declaration / statement
 *	@brief ommits the argument when compiled on MSVC, otherwise resolves to a comma,followed by the argument
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(MSVC_OMMIT)
#define MSVC_OMMIT(x) 
#define MSVC_OMMIT_ARG(x) 
#elif !defined(MSVC_OMMIT) // _MSC_VER && !__MWERKS__ && !MSVC_OMMIT
#define MSVC_OMMIT(x) x
#define MSVC_OMMIT_ARG(x) , x
#endif // _MSC_VER && !__MWERKS__ && !MSVC_OMMIT

/**
 *	@def MSVC6_OMMIT
 *	@param[in] x is a declaration / statement
 *	@brief ommits the argument when compiled on MSVC, otherwise resolves to the argument
 */
/**
 *	@def MSVC6_OMMIT_ARG
 *	@param[in] x is a declaration / statement
 *	@brief ommits the argument when compiled on MSVC, otherwise resolves to a comma,followed by the argument
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200 && !defined(MSVC6_OMMIT)
#define MSVC6_OMMIT(x) 
#define MSVC6_OMMIT_ARG(x) 
#elif !defined(MSVC6_OMMIT) // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200 && !MSVC6_OMMIT
#define MSVC6_OMMIT(x) x
#define MSVC6_OMMIT_ARG(x) , x
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200 && !MSVC6_OMMIT

/**
 *	@def IS_OMMIT
 *	@param[in] x is a declaration / statement
 *	@brief ommits the argument when compiled on compilers
 *		with intellisense support, otherwise resolves to the argument
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(IS_OMMIT)
#define IS_OMMIT(x) 
#elif !defined(IS_OMMIT) // _MSC_VER && !__MWERKS__ && !IS_OMMIT
#define IS_OMMIT(x) x
#endif // _MSC_VER && !__MWERKS__ && !IS_OMMIT

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
/**
 *	@brief simple template for calculating power of two
 *
 *	MSVC60 doesn't handle partial template specializations, so we need to hide _Ty parameter to CMakePOT
 *	in order to be able to declare full specialization of CShifter (otherwise gives infinite template recursion error).
 *
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 */
template <class _Ty>
class CMakePOT {
public:
	/**
	 *	@brief shift series expansion template
	 *	@tparam n_shift is amount to shift the number
	 */
	template <int n_shift>
	class CShifter {
	public:
		/**
		 *	@brief calculates shift series
		 *	@param[in] n_x is input number
		 *	@return Returns n_x or-ed together with all power-of-two fractions of n_shift down to zero.
		 */
		static inline _Ty n_Or_Shift(_Ty n_x)
		{
			n_x = CShifter<n_shift / 2>::n_Or_Shift(n_x);
			return n_x | (n_x >> n_shift);
		}
	};

	/**
	 *	@brief shift series template specialization (recursion terminator)
	 */
	template <>
	class CShifter<0> {
	public:
		/**
		 *	@brief identity function
		 *	@param[in] n_x is input number
		 *	@return Returns n_x.
		 */
		static inline _Ty n_Or_Shift(_Ty n_x)
		{
			return n_x;
		}
	};

	/**
	 *	@brief calculates power of two greater or equal to the argument
	 *
	 *	@param[in] n_x is input number
	 *
	 *	@return Returns power of two greater or equal to n_x. In case such number is not representable by given type, returns null.
	 *
	 *	@note In case _Ty is signed and n_x is greater than the largest power of two, representable by this type (can be set to zero by masking-out the sign bit).
	 */
	static inline _Ty n_Make_POT(_Ty n_x)
	{
		return CShifter<sizeof(_Ty) * 8 / 2>::n_Or_Shift(n_x - 1) + 1;
	}

	/**
	 *	@brief fill ones right of the highest one that is set
	 *
	 *	@param[in] n_x is input number
	 *
	 *	@return Returns n_x which is right-filled with ones.
	 */
	static inline _Ty n_RightFill_Ones(_Ty n_x)
	{
		return CShifter<sizeof(_Ty) * 8 / 2>::n_Or_Shift(n_x);
	}

	/**
	 *	@brief calculates power of two below or equal to the argument
	 *
	 *	@param[in] n_x is input number
	 *
	 *	@return Returns power of two below or equal to n_x.
	 */
	static inline _Ty n_Make_Lower_POT(_Ty n_x)
	{
		return CShifter<sizeof(_Ty) * 8 / 2>::n_Or_Shift(n_x >> 1) + 1; // this is very similar to n_Make_POT(), we just divide the number by two and add one in order to get lower power of two
	}
};
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
// this is the right way to go

/**
 *	@brief shift series expansion template
 *
 *	@tparam _Ty is integer data type
 *	@tparam n_shift is amount to shift the number
 */
template <class _Ty, int n_shift>
class CShifter {
public:
	/**
	 *	@brief calculates shift series
	 *	@param[in] n_x is input number
	 *	@return Returns n_x or-ed together with all power-of-two fractions of n_shift down to zero.
	 */
	static inline _Ty n_Or_Shift(_Ty n_x)
	{
		n_x = CShifter<_Ty, n_shift / 2>::n_Or_Shift(n_x);
		return n_x | (n_x >> n_shift);
	}
};

/**
 *	@brief shift series template specialization (recursion terminator)
 *	@tparam _Ty is integer data type
 */
template <class _Ty>
class CShifter<_Ty, 0> {
public:
	/**
	 *	@brief identity function
	 *	@param[in] n_x is input number
	 *	@return Returns n_x.
	 */
	static inline _Ty n_Or_Shift(_Ty n_x)
	{
		return n_x;
	}
};
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
/**
 *	@brief static shift series expansion template
 *
 *	MSVC60 doesn't handle partial template specializations, so we need to hide the n_x parameter to CMakePOT
 *	in order to be able to declare full specialization of CShifter_Static.
 *
 *	@tparam n_x is the input value to shift
 */
template <const size_t n_x>
class CMakePOT_Static {
public:
	/**
	 *	@brief static shift series expansion template
	 *	@tparam n_shift is amount to shift n_x
	 */
	template <const int n_shift>
	class CShifter_Static {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			/**
			 *	@brief equals n_x or-ed together with all power-of-two fractions of n_shift down to zero
			 */
			result = CShifter_Static<n_shift / 2>::result | (CShifter_Static<n_shift / 2>::result >> n_shift)
		};
	};

	/**
	 *	@brief static shift series template specialization (recursion terminator)
	 */
	template <>
	class CShifter_Static<0> {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			/**
			 *	@brief equals n_x
			 */
			result = n_x
		};
	};
};
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
// this is the right way to go

/**
 *	@brief static shift series expansion template
 *	@tparam n_x is the input value to shift
 *	@tparam n_shift is amount to shift n_x
 */
template <const size_t n_x, const int n_shift>
class CShifter_Static {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		/**
		 *	@brief equals n_x or-ed together with all power-of-two fractions of n_shift down to zero
		 */
		result = CShifter_Static<n_x, n_shift / 2>::result | (CShifter_Static<n_x, n_shift / 2>::result >> n_shift)
	};
};

/**
 *	@brief static shift series template specialization (recursion terminator)
 *	@tparam n_x is the input value to shift
 */
template <const size_t n_x>
class CShifter_Static<n_x, 0> {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		/**
		 *	@brief equals n_x
		 */
		result = n_x
	};
};
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400

/**
 *	@brief simple template for counting number of bits set in a number
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 */
template <class _Ty>
class CBitCounter {
public:
	/**
	 *	@brief template for calculating repeating byte patterns of variable lengths
	 *	@tparam n_value32 is value of the pattern to be repeated
	 */
	template <const unsigned int n_value32>
	class CIntRepeater {
	protected:
		/**
		 *	@brief intermediates, stored as enum
		 */
		enum {
			n_repeat_num = (sizeof(_Ty) + 3) / 4 /**< @brief number of repeats required to fill all of _Ty (round up) */
		};

		/**
		 *	@brief pattern shifter
		 *
		 *	@tparam n_repeat_num isn umber of times to repeat the pattern
		 *	@tparam GppDummy is dummy parameter (g++ compatibility hack)
		 */
		template <const int n_repeat_num MSVC_OMMIT_ARG(class GppDummy)>
		class CShifter {
		public:
			/**
			 *	@brief gets the repeated value
			 *	@return Returns _Ty, filled with the n_value32 pattern.
			 */
			static inline _Ty n_Value()
			{
				return (_Ty(n_value32) << ((n_repeat_num - 1) * 32)) |
					CShifter<n_repeat_num - 1 MSVC_OMMIT_ARG(GppDummy)>::n_Value();
			}
		};

		/**
		 *	@brief pattern shifter (specialization for a single / the last repeat)
		 *	@tparam GppDummy is dummy parameter (g++ compatibility hack)
		 */
		template <MSVC_OMMIT(class GppDummy)>
		class CShifter<1 MSVC_OMMIT_ARG(GppDummy)> {
		public:
			/**
			 *	@brief gets the repeated value
			 *	@return Returns _Ty, filled with the n_value32 pattern.
			 */
			static inline _Ty n_Value()
			{
				return _Ty(n_value32);
			}
		};

	public:
		/**
		 *	@brief gets the repeated value
		 *	@return Returns _Ty, filled with the n_value32 pattern.
		 */
		static inline _Ty n_Value()
		{
			return CShifter<n_repeat_num MSVC_OMMIT_ARG(void)>::n_Value();
		}
	};

	/**
	 *	@brief calculates number of bits set, in parallel
	 *
	 *	@param[in] n_x is input value, which must not be negative
	 *
	 *	@return Returns number of bits set in n_x.
	 */
	static inline uint8_t n_SetBit_Num(_Ty n_x)
	{
		_ASSERTE(sizeof(_Ty) * 8 < 256); // the last summation happens in a single byte; this wont work for int256_t or larger (will have to split the final product (the last line of this function) to four smaller products)
		n_x -= (n_x >> 1) & CIntRepeater<0x55555555U>::n_Value(); // sums even + odd (wow), all bits are still significant
		n_x = (n_x & CIntRepeater<0x33333333U>::n_Value()) +
			((n_x >> 2) & CIntRepeater<0x33333333U>::n_Value()); // sums pairs, only three bits in each nibble are now significant - can add without masking
		n_x = (n_x + (n_x >> 4)) & CIntRepeater<0x0f0f0f0fU>::n_Value(); // sums nibbles, using common mask; only every other nibble is now significant
		return uint8_t((n_x * CIntRepeater<0x01010101U>::n_Value()) >> (sizeof(_Ty) * 8 - 8)); // can sum everything using multiplication, result is guaranteed not to overflow a single byte
	}
};

#if 0 // specialization not required anymore
/**
 *	@brief simple template for counting number of bits set in an int64_t number
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 */
template <>
class CBitCounter<int64_t> {
public:
	/**
	 *	@brief calculates number of bits set, in parallel
	 *	@param[in] n_x is input value, which must not be negative
	 *	@return Returns number of bits set in n_x.
	 */
	static inline unsigned int n_SetBit_Num(int64_t n_x)
	{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
		n_x -= (n_x >> 1) & 0x5555555555555555; // sums even + odd (wow), all bits are still significant
		n_x = (n_x & 0x3333333333333333) + ((n_x >> 2) & 0x3333333333333333); // sums pairs, only three bits in each nibble are now significant - can add without masking
		n_x = (n_x + (n_x >> 4)) & 0x0f0f0f0f0f0f0f0f; // sums nibbles, using common mask; only every other nibble is now significant
		return (unsigned int)((n_x * 0x0101010101010101) >> 56); // can sum everything using multiplication, result is guaranteed not to overflow a single byte
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		const int64_t n_fives = 0x55555555 | (int64_t(0x55555555) << 32);
		const int64_t n_threes = 0x33333333 | (int64_t(0x33333333) << 32);
		const int64_t n_zerofs = 0x0f0f0f0f | (int64_t(0x0f0f0f0f) << 32);
		const int64_t n_zeroones = 0x01010101 | (int64_t(0x01010101) << 32);
		// it would be better to use LL suffixes, but g++ is yappin about it

		n_x -= (n_x >> 1) & n_fives; // sums even + odd (wow), all bits are still significant
		n_x = (n_x & n_threes) + ((n_x >> 2) & n_threes); // sums pairs, only three bits in each nibble are now significant - can add without masking
		n_x = (n_x + (n_x >> 4)) & n_zerofs; // sums nibbles, using common mask; only every other nibble is now significant
		return (unsigned int)((n_x * n_zeroones) >> 56); // can sum everything using multiplication, result is guaranteed not to overflow a single byte
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	}
};

/**
 *	@brief simple template for counting number of bits set in an uint64_t number
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 */
template <>
class CBitCounter<uint64_t> {
public:
	/**
	 *	@brief calculates number of bits set, in parallel
	 *	@param[in] n_x is input value
	 *	@return Returns number of bits set in n_x.
	 */
	static inline unsigned int n_SetBit_Num(uint64_t n_x)
	{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
		n_x -= (n_x >> 1) & 0x5555555555555555U; // sums even + odd (wow), all bits are still significant
		n_x = (n_x & 0x3333333333333333U) + ((n_x >> 2) & 0x3333333333333333U); // sums pairs, only three bits in each nibble are now significant - can add without masking
		n_x = (n_x + (n_x >> 4)) & 0x0f0f0f0f0f0f0f0fU; // sums nibbles, using common mask; only every other nibble is now significant
		return (unsigned int)((n_x * 0x0101010101010101U) >> 56); // can sum everything using multiplication, result is guaranteed not to overflow a single byte
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		const uint64_t n_fives = 0x55555555U | (int64_t(0x55555555U) << 32);
		const uint64_t n_threes = 0x33333333U | (int64_t(0x33333333U) << 32);
		const uint64_t n_zerofs = 0x0f0f0f0fU | (int64_t(0x0f0f0f0fU) << 32);
		const uint64_t n_zeroones = 0x01010101U | (int64_t(0x01010101U) << 32);
		// it would be better to use ULL suffixes, but g++ is yappin about it

		n_x -= (n_x >> 1) & n_fives; // sums even + odd (wow), all bits are still significant
		n_x = (n_x & n_threes) + ((n_x >> 2) & n_threes); // sums pairs, only three bits in each nibble are now significant - can add without masking
		n_x = (n_x + (n_x >> 4)) & n_zerofs; // sums nibbles, using common mask; only every other nibble is now significant
		return (unsigned int)((n_x * n_zeroones) >> 56); // can sum everything using multiplication, result is guaranteed not to overflow a single byte
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	}
};
#endif // 0

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400

/**
 *	@brief simple compile-time bit counter template
 *	@tparam x is an input value where the bits should be counted
 */
template <const size_t x>
class CBitCounter_Static {
public:
	/**
	 *	@brief simple compile-time bit counter template
	 *	@tparam n is the number of remaining bits to count in x
	 */
	template <const size_t n>
	class CBitCounter2_Static {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			result = (x & 1) + CBitCounter_Static<x >> 1>::CBitCounter2_Static<n - 1>::result /**< @brief contains the sum of bits set in x */
		};
	};

	/**
	 *	@brief simple compile-time bit counter template
	 */
	template <>
	class CBitCounter2_Static<0> {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			result = 0; /**< @brief contains the sum of bits set in x */
		};
	};
};

#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
// this is the right way to go

/**
 *	@brief simple compile-time bit counter template
 *
 *	@tparam x is an input value where the bits should be counted
 *	@tparam n is the number of remaining bits to count in x
 */
template <const size_t x, const size_t n>
class CBitCounter_Static {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = (x & 1) + CBitCounter_Static<x / 2, n - 1>::result /**< @brief contains the sum of bits set in x */
	};
};

/**
 *	@brief simple compile-time bit counter template
 *	@tparam x is an input value where the bits should be counted
 */
template <const size_t x>
class CBitCounter_Static<x, 0> {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = 0 /**< @brief contains the sum of bits set in x */
	};
};

#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400

/**
 *	@brief simple compile-time base 2 logarithm template
 *	@tparam n is the number of remaining bits to count in x
 */
template <const size_t n>
class CLog2_Static {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = CLog2_Static<n / 2>::result + 1 /**< @brief result = log<sub>2</sub>(n) */
	};
};

/**
 *	@brief simple compile-time base 2 logarithm template
 */
template <>
class CLog2_Static<1> {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = 0 /**< @brief result = log<sub>2</sub>(n) */
	};
};

/**
 *	@brief simple compile-time base 2 logarithm template
 */
template <>
class CLog2_Static<0> {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		result = 0 /**< @brief result = log<sub>2</sub>(n) */
	};
};

/**
 *	@brief compile-time power helper (only positive powers)
 *
 *	@tparam n_base is value of the base
 *	@tparam n_power is power (can be both positive or negative)
 */
template <const int n_base, const unsigned int n_power> // C++03 can't have double template args, n_base must be an integer
class CUIntPower {
protected:
	/**
	 *	@brief msvc6 workarround for partial template specialization
	 *
	 *	@tparam n_pow is power (can be both positive or negative)
	 *	@tparam _GppDummy is g++ compatibility workarround (avoids full specialization inside a template scope)
	 */
	template <const unsigned int n_pow MSVC_OMMIT_ARG(class _GppDummy)>
	class CRecurseSpec {
	protected:
		/**
		 *	@brief intermediate values stored as enum
		 */
		enum {
			n_next_power = n_pow >> 1, /**< @brief next recursion power value */
			n_next_base = n_base * (n_next_power != 0) * n_base, /**< @brief next recursion base power */
			n_cur_multipler = (n_pow & 1)? n_base : 1 /**< @brief value of the current multiplier */
		};

	public:
		/**
		 *	@brief result stored as enum
		 */
		enum {
			n_result = n_cur_multipler * CUIntPower<n_next_base, n_next_power>::n_result /**< @brief value of the result */
		};
	};

	/**
	 *	@brief msvc6 workarround for partial template specialization (specialization for 0th power)
	 */
	template <MSVC_OMMIT(class _GppDummy)>
	class CRecurseSpec<0 MSVC_OMMIT_ARG(_GppDummy)> {
	public:
		/**
		 *	@brief result stored as enum
		 */
		enum {
			n_result = 1 /**< @brief value of the result */
		};
	};

public:
	/**
	 *	@brief result stored as enum
	 */
	enum {
		n_result = CRecurseSpec<n_power MSVC_OMMIT_ARG(void)>::n_result /**< @brief value of the result */
	};
};

/**
 *	@brief compile-time power function
 *
 *	@tparam n_base is value of the base
 *	@tparam n_power is power (can be both positive or negative)
 *	@tparam T is data type to store the result (default double)
 */
template <const int n_base, const int n_power, class T = double> // C++03 can't have double template args, n_base must be an integer
class CPower {
#if 0
public: // msvc 6.0 requires public here
	/**
	 *	@brief msvc6 workarround for partial template specialization
	 *
	 *	@tparam n_pow is power (can be both positive or negative)
	 *	@tparam _GppDummy is g++ compatibility workarround (avoids full specialization inside a template scope)
	 */
	template <const int n_pow MSVC_OMMIT_ARG(class _GppDummy)>
	class CPowerHelper {
	protected:
		/**
		 *	@brief intermediate values stored as enum
		 */
		enum {
			b_positive = n_pow > 0, /**< @brief positive power flag */
			n_next = n_pow + ((b_positive)? -1 : 1) /**< @brief next recursion power value */
		};

	public:
		/**
		 *	@brief gets the power
		 *	@return Returns the power.
		 */
		static inline T f_Result()
		{
			return (b_positive)? CPowerHelper<n_next MSVC_OMMIT_ARG(_GppDummy)>::f_Result() * n_base :
				CPowerHelper<n_next MSVC_OMMIT_ARG(_GppDummy)>::f_Result() / n_base;
		}
	};

	/**
	 *	@brief msvc6 workarround for partial template specialization (specialization for 0th power)
	 */
	template <MSVC_OMMIT(class _GppDummy)>
	class CPowerHelper<0 MSVC_OMMIT_ARG(_GppDummy)> {
	public:
		/**
		 *	@brief gets the power
		 *	@return Returns the power.
		 */
		static inline T f_Result()
		{
			return 1;
		}
	};
#endif // 0

public:
	/**
	 *	@brief gets the power
	 *	@return Returns the power.
	 */
	static inline T f_Result()
	{
		enum {
			n_abs_power = (n_power < 0)? -n_power : n_power,
			n_abs_quarter_power = n_abs_power / 4, // divide the power to increase range
			n_abs_remainder_power = n_abs_power - 3 * n_abs_quarter_power,
			n_abs_quarter_result = CUIntPower<n_base, n_abs_quarter_power>::n_result,
			n_abs_remainder_result = CUIntPower<n_base, n_abs_remainder_power>::n_result
		};
		T f = (T)n_abs_quarter_result * (T)n_abs_quarter_result *
			(T)n_abs_quarter_result * (T)n_abs_remainder_result;
		return (n_power < 0)? 1 / f : f; // handle negative powers here
	}
};

} // ~ul_bithacks

/**
 *	@def n_Power_Static
 *
 *	@brief calculates integer power at compile-time
 *
 *	@param[in] n_base is base
 *	@param[in] n_power is power (negative powers will yield zero result, as it is integer)
 *
 *	@return Returns specified power of the specified base.
 */
#define n_Power_Static(n_base,n_power) (ul_bithacks::CPower<n_base, n_power, int>::f_Result())

/**
 *	@def f_Power_Static
 *
 *	@brief calculates floating-point power at compile-time
 *
 *	@param[in] n_base is base
 *	@param[in] n_power is power
 *
 *	@return Returns specified power of the specified base.
 */
#define f_Power_Static(n_base,n_power) (ul_bithacks::CPower<n_base, n_power, double>::f_Result())

/**
 *	@brief determines whether a number is power of two, or not
 *	@tparam _Ty is integer data type
 *	@param[in] n_x is number being tested. note it must be positive
 *	@return Returns true if n_x is power of two, otherwise returns false.
 */
template <class _Ty>
inline bool b_Is_POT(_Ty n_x)
{
	return !(n_x & (n_x - 1));
}

/**
 *	@brief determines whether a number is power of two, or not, can be evaluated at compile-time
 *	@param[in] n_x is number being tested. note it must be positive
 *	@return Returns true if n_x is power of two, otherwise returns false.
 */
#define b_Is_POT_Static(n_x) (!((n_x) & ((n_x) - 1)))

/**
 *	@brief calculates power of two greater or equal to the argument
 *
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *
 *	@param[in] n_x is input number, which must not be negative
 *
 *	@return Returns power of two greater or equal to n_x.
 *
 *	@note In case _Ty is unsigned and n_x is greater than the largest power of two,
 *		representable by the given type, returns null.
 *	@note In case _Ty is signed and n_x is greater than the largest power of two,
 *		representable by this type, returns the maximum negative value representable
 *		by this type (can be set to zero by masking-out the sign bit).
 */
template <class _Ty>
inline _Ty n_Make_POT(_Ty n_x)
{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
	return ul_bithacks::CMakePOT<_Ty>::n_Make_POT(n_x);
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
	return ul_bithacks::CShifter<_Ty, sizeof(_Ty) * 8 / 2>::n_Or_Shift(n_x - 1) + 1;
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
}

/**
 *	@brief calculates power of two greater or equal to the argument, at compile-time
 *	@param[in] n_x is input number, which must not be negative
 *	@return Returns power of two below or equal to n_x.
 *	@note Overflow handling may be compiler dependent (depends on whether enums
 *		are signed or unsigned), may return 0 or maximum negative value, respectively.
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
#define n_Make_POT_Static(n_x) (ul_bithacks::CMakePOT_Static<(n_x) - 1>::CShifter_Static<sizeof(n_x) * 8 / 2>::result + 1)
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
#define n_Make_POT_Static(n_x) (ul_bithacks::CShifter_Static<(n_x) - 1, sizeof(n_x) * 8 / 2>::result + 1)
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400

/**
 *	@brief calculates power of two below or equal to the argument
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *	@param[in] n_x is input number, which must not be zero or negative
 *	@return Returns power of two below or equal to n_x.
 */
template <class _Ty>
inline _Ty n_Make_Lower_POT(_Ty n_x)
{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
	return (n_x)? ul_bithacks::CMakePOT<_Ty>::n_Make_Lower_POT(n_x) : 0;
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
	return (n_x)? ul_bithacks::CShifter<_Ty, sizeof(_Ty) * 8 / 2>::n_Or_Shift(n_x >> 1) + 1 : 0; // this is very similar to n_Make_POT(), we just divide the number by two and add one in order to get lower power of two
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
}

/**
 *	@brief calculates power of two below or equal to the argument, at compile-time
 *	@param[in] n_x is input number, which must not be zero or negative
 *	@return Returns power of two below or equal to n_x.
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
#define n_Make_Lower_POT_Static(n_x) ((n_x)? (ul_bithacks::CMakePOT_Static<(n_x) >> 1>::CShifter_Static<sizeof(n_x) * 8 / 2>::result + 1) : 1)
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
#define n_Make_Lower_POT_Static(n_x) ((n_x)? (ul_bithacks::CShifter_Static<(n_x) / 2, sizeof(n_x) * 8 / 2>::result + 1) : 1)
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400

/**
 *	@brief aligns number up to the next multiple of given alignment
 *
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *
 *	@param[in] n_x is value to be aligned
 *	@param[in] n_alignment is alignment
 *
 *	@return Returns such y, so that y >= n_x and y < n_x + n_alignment and y % n_alignment = 0.
 *
 *	@note This function uses the modulo operator; for power of two alignments,
 *		it might be more effective to use n_Align_Up_POT().
 */
template <class _Ty>
inline _Ty n_Align_Up(_Ty n_x, _Ty n_alignment)
{
	_ASSERTE(n_alignment != 0); // would cause division by zero
	n_x += n_alignment - 1;
	return n_x - n_x % n_alignment;
}

/**
 *	@brief aligns number up to the next multiple of given alignment, can be evaluated at compile-time
 *
 *	@param[in] n_x is value to be aligned
 *	@param[in] n_alignment is alignment
 *
 *	@return Returns such y, so that y >= n_x and y < n_x + n_alignment and y % n_alignment = 0.
 */
#define n_Align_Up_Static(n_x,n_alignment) (((n_x) + (n_alignment) - 1) - ((n_x) + (n_alignment) - 1) % (n_alignment))

/**
 *	@brief aligns number up to the next multiple of given power-of-two alignment
 *
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *
 *	@param[in] n_x is value to be aligned
 *	@param[in] n_pot_alignment is alignment, it must be power of two
 *
 *	@return Returns such y, so that y >= n_x and y < n_x + n_alignment and y % n_alignment = 0.
 */
template <class _Ty>
inline _Ty n_Align_Up_POT(_Ty n_x, _Ty n_pot_alignment)
{
	_ASSERTE(b_Is_POT(n_pot_alignment)); // alignment must be power of two
	-- n_pot_alignment;
	return (n_x + n_pot_alignment) & ~n_pot_alignment;
}

/**
 *	@brief aligns number up to the next multiple of given (power-of-two) alignment, can be evaluated at compile-time
 *
 *	@param[in] n_x is value to be aligned
 *	@param[in] n_alignment is alignment
 *
 *	@return Returns such y, so that y >= n_x and y < n_x + n_alignment and y % n_alignment = 0.
 *
 *	@note As this is evaluated at compile-time and the speed is of little interest, this just calls
 *		n_Align_Up_Static() macro and the power-of-two requirement is therefore relaxed.
 */
#define n_Align_Up_POT_Static(n_x,n_alignment) (n_Align_Up_Static((n_x), (n_alignment)))

/**
 *	@brief calculates number of bits set, in parallel
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *	@param[in] n_x is input value, which must not be negative
 *	@return Returns number of bits set in n_x.
 */
template <class _Ty>
inline unsigned int n_SetBit_Num(_Ty n_x)
{
	return ul_bithacks::CBitCounter<_Ty>::n_SetBit_Num(n_x);
}

/**
 *	@brief calculates number of bits set in a given number at compile-time
 *	@param[in] n_x is input value, which must not be negative
 *	@return Returns number of bits set in n_x.
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 */
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
#define n_SetBit_Num_Static(n_x) (ul_bithacks::CBitCounter_Static<(n_x)>::CBitCounter2_Static<sizeof(n_x) * 8>::result)
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
#define n_SetBit_Num_Static(n_x) (ul_bithacks::CBitCounter_Static<(n_x), sizeof(n_x) * 8>::result)
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400

/**
 *	@brief calculates base two logarithm of a given number (round down)
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *	@param[in] n_x is input value, which must not be negative
 *	@return Returns floor(log2(n_x)).
 *	@note On range of 1 to 10^9, <tt>n_Log2(x)</tt> returns the same values
 *		as <tt>int(floor(log(double(x)) / log(2.0)))</tt>.
 */
template <class _Ty>
inline unsigned int n_Log2(_Ty n_x)
{
	if(!n_x)
		return 0;
	n_x = n_Make_Lower_POT(n_x) - 1; // set all bits right of highest set bit in n_x
	return n_SetBit_Num(n_x); // count bits set
}

/**
 *	@brief calculates base two logarithm of a given number (round down) at compile-time
 *	@param[in] n_x is input value, which must not be negative
 *	@return Returns floor(log2(n_x)).
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 *	@note On range of 1 to 10^9, <tt>n_Log2_Static(x)</tt> returns the same values
 *		as <tt>int(floor(log(double(x)) / log(2.0)))</tt>.
 */
#define n_Log2_Static(n_x) (ul_bithacks::CLog2_Static<(n_x)>::result)

/**
 *	@brief calculates base two logarithm of a given number (round up)
 *
 *	@tparam _Ty is integer data type; this works for both signed and unsigned types
 *
 *	@param[in] n_x is input value, which must not be negative
 *
 *	@return Returns ceil(log2(n_x)).
 *
 *	@note This does not return the number of bits required to store the number
 *		(it underestimates the powers of two by 1 bit), üse n_Bit_Width() instead.
 *	@note On range of 1 to 0.5 * 10^9, <tt>n_Log2_Ceil(x)</tt> returns the same values
 *		as <tt>int(ceil(log(double(x)) / log(2.0)))</tt>.
 */
template <class _Ty>
inline unsigned int n_Log2_Ceil(_Ty n_x)
{
	if(!n_x)
		return 0;
	n_x = n_Make_POT(n_x) - 1; // set all bits right of highest set bit in n_x
	return n_SetBit_Num(n_x); // count bits set
}

/**
 *	@brief calculates base two logarithm of a given number (round up) at compile-time
 *
 *	@param[in] n_x is input value, which must not be negative
 *
 *	@return Returns floor(log2(n_x)).
 *
 *	@note This does not return the number of bits required to store the number
 *		(it underestimates the powers of two by 1 bit), üse n_Bit_Width_Static() instead.
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 *	@note On range of 1 to 0.5 * 10^9, <tt>n_Log2_Ceil_Static(x)</tt> returns the same values
 *		as <tt>int(ceil(log(double(x)) / log(2.0)))</tt>.
 */
#define n_Log2_Ceil_Static(n_x) (n_Log2_Static(n_Make_POT_Static(n_x)))

/**
 *	@brief fill ones right of the highest one that is set
 *	@param[in] n_x is input number
 *	@return Returns n_x which is right-filled with ones.
 */
template <class _Ty>
inline _Ty n_RightFill_Ones(_Ty n_x)
{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
	return ul_bithacks::CMakePOT<_Ty>::n_RightFill_Ones(n_x);
#else // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
	return ul_bithacks::CShifter<_Ty, sizeof(_Ty) * 8 / 2>::n_Or_Shift(n_x);
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
}

/**
 *	@brief fill ones right of the highest one that is set at compile-time
 *	@param[in] n_x is input number
 *	@return Returns n_x which is right-filled with ones.
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 */
#define n_RightFill_Ones_Static(n_x) (n_Make_POT_Static((n_x) + 1) - 1)

/**
 *	@brief calculates mask with a specified right-most bits set
 *	@tparam _Ty is integer data type
 *	@param[in] n_bit_num is number of bits to set (must be less than or equal to 8 * sizeof(_Ty))
 *	@return Returns mask with n_bit_num right-most bits set.
 */
template <class _Ty>
inline _Ty n_Mask(_Ty n_bit_num)
{
	return (n_bit_num > 0)? (_Ty(1) << (n_bit_num - 1)) |
		((n_bit_num > 1)? ((_Ty(1) << (n_bit_num - 1)) - 1) : 0) : 0;
}

/**
 *	@brief calculates mask with a specified right-most bits set at compile-time
 *	@param[in] n_bit_num is number of bits to set (must be less than or equal to 8 * sizeof(int))
 *	@return Returns mask with n_bit_num right-most bits set.
 */
#define n_Mask_Static(n_bit_num) \
	((n_bit_num > 0)? ((1 << ((n_bit_num) - 1)) | \
	((n_bit_num > 1)? ((1 << ((n_bit_num) - 1)) - 1) : 0)) : 0)

/**
 *	@brief calculates number of leading zeros
 *	@tparam _Ty is integer data type
 *	@param[in] n_x is the input number
 *	@return Returns number of leading zeros in n_x.
 */
template <class _Ty>
inline unsigned int n_LeadingZero_Num(_Ty n_x)
{
	return n_SetBit_Num(_Ty(~n_RightFill_Ones(n_x)));
}

/**
 *	@brief calculates number of leading zeros at compile-time
 *	@param[in] n_x is the input number
 *	@return Returns number of leading zeros in n_x.
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 */
#define n_LeadingZero_Num_Static(n_x) (n_SetBit_Num_Static(~n_RightFill_Ones_Static(n_x)))

/**
 *	@brief calculates number of trailing zeros
 *	@tparam _Ty is integer data type
 *	@param[in] n_x is the input number
 *	@return Returns number of trailing zeros in n_x.
 */
template <class _Ty>
inline unsigned int n_TrailingZero_Num(_Ty n_x)
{
	return n_SetBit_Num(_Ty(((n_x - _Ty(1)) | n_x) ^ n_x));
	// need to watch the type to avoid getting casted to integer
	// and counting more than is there (e.g. when _Ty is uint8_t)
}

/**
 *	@brief calculates number of leading zeros at compile-time
 *	@param[in] n_x is the input number
 *	@return Returns number of leading zeros in n_x.
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 */
#define n_TrailingZero_Num_Static(n_x) ((n_x)? n_SetBit_Num_Static((((n_x) - 1) | (n_x)) ^ (n_x)) : 8 * sizeof(n_x))
// handle zero explicitly otherwise it will cause an underflow, potentially
// setting higher bits (we don't know the type of n_x so we can't cast it)

/**
 *	@brief calculates the number of bits the nonzero bits in a number span over
 *	@tparam _Ty is integer data type
 *	@param[in] n_x is the input number
 *	@return Returns the bandwidth of n_x.
 *	@note This is equivalent to shifting right until the LSB is one and then
 *		taking bit width, with the exception that bandwidth of zero is zero.
 */
template <class _Ty>
inline unsigned int n_Bit_Bandwidth(_Ty n_x)
{
	if(!n_x)
		return 0; // otherwise trailing zeros are all ones and this returns sizeof(_Ty) * 8
	_Ty n_trailing_zeros = _Ty(((n_x - _Ty(1)) | n_x) ^ n_x);
	_Ty n_right_fill = n_RightFill_Ones(n_x);
	_Ty n_band_width = n_right_fill ^ n_trailing_zeros;
	return n_SetBit_Num(n_band_width);
}

/**
 *	@brief calculates the number of bits the nonzero bits in a number span over at compile-time
 *	@param[in] n_x is the input number
 *	@return Returns the bandwidth of n_x.
 *	@note This is equivalent to shifting right until the LSB is one and then
 *		taking bit width, with the exception that bandwidth of zero is zero.
 */
#define n_Bit_Bandwidth_Static(n_x) ((n_x)? n_SetBit_Num_Static( \
	n_RightFill_Ones_Static(n_x) ^ (((n_x - 1) | n_x) ^ n_x)) : 0)

/**
 *	@brief calculates the number of bits needed to store the input number
 *	@tparam _Ty is integer data type
 *	@param[in] n_x is the input number
 *	@return Returns number of bits required to store n_x.
 *	@note Bit width of zero is one bit, not zero.
 */
template <class _Ty>
inline unsigned int n_Bit_Width(_Ty n_x)
{
	return (n_x)? n_SetBit_Num(n_RightFill_Ones(n_x)) : 1;
}

/**
 *	@brief calculates the number of bits needed to store the input number, at compile-time
 *	@param[in] n_x is the input number
 *	@return Returns number of bits required to store n_x.
 *	@note This uses templates and enums, and may therefore have limited range of input
 *		values (some compilers limit maximum size of enum to 31 or 32 bits).
 */
#define n_Bit_Width_Static(n_x) ((n_x)? n_SetBit_Num_Static(n_RightFill_Ones_Static(n_x)) : 1)

#ifdef __BIT_HACKS_UNIT_TESTS
#include <math.h> // log()
#endif // __BIT_HACKS_UNIT_TESTS

namespace ul_bithacks {

#ifdef __BIT_HACKS_UNIT_TESTS
/**
 *	@brief tests bit hacks
 *
 *	Test with e.g.:
 *
 *	@code
 *	printf("char: "); ul_bithacks::UnitTests<char>(INT8_MAX);
 *	printf("unsigned char: "); ul_bithacks::UnitTests<unsigned char>(UINT8_MAX);
 *	printf("short: "); ul_bithacks::UnitTests<short>(INT16_MAX);
 *	printf("unsigned short: "); ul_bithacks::UnitTests<unsigned short>(UINT16_MAX);
 *	printf("int: "); ul_bithacks::UnitTests<int>(INT_MAX);
 *	printf("unsigned int: "); ul_bithacks::UnitTests<unsigned int>(UINT_MAX);
 *	@endcode
 *
 *	Note that this code is not included here to avoid instantiation
 *	of this function in code that does not need it.
 *
 *	@tparam Int is integer data type to test with
 *	@param[in] n_max is maximum value to test with (default maximum value of the given type)
 *	@return Returns true on success, false on failute.
 */
template <class Int>
static bool UnitTests(Int n_max = CMaxIntValue<Int>::result())
{
	size_t n_fail_num = 0;
	for(Int i = 0; i < n_max; ++ i) {
		bool b_is_pot = b_Is_POT(i);
		bool b_is_pot_ref = false;
		if(!i) // need to handle zero explicitly
			b_is_pot_ref = true;
		else {
			for(Int j = 1; j <= i && j != 0; j <<= 1) {
				if(i == j) {
					b_is_pot_ref = true;
					break;
				}
			}
			// search for powers of two by shifting a single one
		}
		if(b_is_pot != b_is_pot_ref) {
			printf("is_pot %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("is_pot ");
	for(Int i = 0; i < n_max; ++ i) {
		Int n_pot = n_Make_Lower_POT(i); // next greater or equal
		if(!b_Is_POT(n_pot) || // not por
		   n_pot > i || // not lower
		   (n_pot && n_pot < n_max / 2 && n_pot * 2 <= i)) { // lower, but there is a larger lower pot
			printf("lpot %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("lpot ");
	for(Int i = 0; i < n_max; ++ i) {
		if(b_Is_POT(i - 1)) { // i actually needs to be 1 greater than the POT for this to fail
			Int n_lpot = i - 1;//n_Make_Lower_POT(i);
			if(i > n_lpot && n_lpot > n_max / 2 && Int(n_lpot * 2) <= 0)
				break;
			// if signed, the below code will fail as the highest pot will be negative
			// if unsigned, the below code will fail as the highest pot will underflow to 0 / small number
		}
		Int n_pot = n_Make_POT(i); // next greater or equal
		if(!b_Is_POT(n_pot) || // not por
		   n_pot < i || // not greater or equal
		   (n_pot && n_pot / 2 >= i)) { // greater or equal, but there is a smaller greater or equal pot
			printf("pot %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("pot ");
	for(Int i = 1; i < n_max; ++ i) {
		for(Int j = 0; j < n_max / i; ++ j) {
			Int n_aligned = n_Align_Up(j, i);
			if(n_aligned % i != 0 || // is not aligned
			   n_aligned < j || // is not aligned up
			   (n_aligned != 0 && n_aligned - i >= j)) { // is not the next aligned
				printf("align %d %d\n", int(j), int(i));
				i = n_max - 1; // break out of the outer loop
				++ n_fail_num;
				break;
			}
			if(b_Is_POT(i) && n_Align_Up_POT(j, i) != n_aligned) {
				printf("align_POT %d %d\n", int(j), int(i));
				i = n_max - 1; // break out of the outer loop
				++ n_fail_num;
				break;
			}
		}
	}
	printf("align align_POT ");
	for(Int i = 0; i < n_max; ++ i) {
		int n_bits = n_SetBit_Num(i);
		int n_bits_ref = 0;
		for(Int j = i; j; j >>= 1)
			n_bits_ref += j & 1;
		if(n_bits != n_bits_ref) {
			printf("popcnt %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("popcnt ");
	for(Int i = 0, n = 0;; ++ n, i = (i << 1) | 1) {
		Int n_mask = n_Mask(n);
		if(n_mask != i) {
			printf("mask %d\n", int(n));
			++ n_fail_num;
			break;
		}
		if(i == Int(-1))
			break;
	}
	printf("mask ");
	for(Int i = 0; i < n_max; ++ i) {
		int n_width = (int)n_Bit_Width(i);
		if(n_width <= 0 || // no number can be stored with 0 bits
		   (n_Mask((Int)n_width) & i) != i || // does not fit
		   (~n_Mask((Int)n_width) & i) != 0 || // does not fit
		   (n_width > 1 && (n_Mask((Int)n_width - 1) & i) == i)) { // fits even smaller
			printf("width %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("width ");
	for(Int i = 0; i < n_max; ++ i) {
		int n_bandwidth = (int)n_Bit_Bandwidth(i);
		Int n = i;
		if(!(n & 0xffff))
			n >>= 16;
		if(!(n & 0xff))
			n >>= 8;
		if(!(n & 0xf))
			n >>= 4;
		if(!(n & 0x3))
			n >>= 2;
		while(n && !(n & 1)) // in case it is wider than 32 bits (has fewer loops for 32 bits this way, functional albeit suboptimal for 64 bits - but it is not expected to test full-range of the 64-bit numbers)
			n >>= 1;
		_ASSERTE(!i || n);
		// shift the value all the way to the right so that LSB is one

		int n_bandwidth_ref = (n)? n_Bit_Width(n) : 0;
		// calculate bandwidth

		if(n_bandwidth != n_bandwidth_ref) {
			printf("bandwidth %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("bandwidth ");
	for(Int i = 0; i < n_max; ++ i) {
		int n_lzcnt = (int)n_LeadingZero_Num(i);
		int n_lzcnt_naive = 8 * sizeof(Int);
		for(int j = 0; j < 8 * sizeof(Int); ++ j) {
			if(i >> j == 1) {
				n_lzcnt_naive = 8 * sizeof(Int) - j - 1;
				break;
			}
		}
		if(n_lzcnt != n_lzcnt_naive) { // fits even smaller
			printf("lzcnt %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("lzcnt ");
	for(Int i = 0; i < n_max; ++ i) {
		int n_tzcnt = (int)n_TrailingZero_Num(i);
		int n_tzcnt_naive;
		uint64_t v = i; // must be unsigned otherwise would shift indefinitely
		if(v) {
			v = (v ^ (v - 1)) >> 1;  // set v's trailing 0s to 1s and zero rest
			for(n_tzcnt_naive = 0; v; ++ n_tzcnt_naive)
				v >>= 1;
		} else
			n_tzcnt_naive = 8 * sizeof(Int);
		if(n_tzcnt != n_tzcnt_naive) {
			printf("tzcnt %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("tzcnt ");
	for(Int i = 0; i < n_max; ++ i) {
		int n_width = (int)n_Bit_Width(i);
		Int n_rfo = n_RightFill_Ones(i);
		if((i && n_Mask((Int)n_width) != n_rfo) || (!i && n_rfo != 0)) { // fits even smaller
			printf("rfo %d\n", int(i));
			++ n_fail_num;
			break;
		}
	}
	printf("rfo ");
	for(Int i = 1; i < n_max; ++ i) {
		Int n_log2floor = (i)? Int(floor(log(double(i)) / log(2.0))) : 0;
		if(n_log2floor != n_Log2(i)) {
			if(i < 1e+9) { // otherwise acceptable
				printf("log %d\n", int(i));
				++ n_fail_num;
			} else
				printf("log %d (acceptable)\n", int(i));
			break;
		}
	}
	printf("log ");
	for(Int i = 0; i < n_max; ++ i) {
		Int n_log2ceil = (i)? Int(ceil(log(double(i)) / log(2.0))) : 0;
		if(n_log2ceil != n_Log2_Ceil(i)) {
			if(i < 0.5e+9) { // otherwise acceptable
				printf("log_ceil %d\n", int(i));
				++ n_fail_num;
			} else
				printf("log_ceil %d (acceptable)\n", int(i));
			break;
		}
	}
	printf("log_ceil ");
	printf(": done (" PRIsize " fails)\n", n_fail_num);

	return !n_fail_num;
}

/**
 *	@brief tests templated static bit hacks
 */
static bool Static_UnitTests()
{
	// todo - test more of the macros

	double ten_0 = f_Power_Static(10, 0); _ASSERTE(ten_0 == 1.0);
	double ten_1 = f_Power_Static(10, 1); _ASSERTE(ten_1 == 10.0);
	double ten_2 = f_Power_Static(10, 2); _ASSERTE(ten_2 == 100.0);
	double ten_3 = f_Power_Static(10, 3); _ASSERTE(ten_3 == 1000.0);
	double ten_4 = f_Power_Static(10, 4); _ASSERTE(ten_4 == 10000.0);
	double ten_5 = f_Power_Static(10, 5); _ASSERTE(ten_5 == 100000.0);
	double ten_m1 = f_Power_Static(10, -1); _ASSERTE(ten_m1 == 1.0 / 10);
	double ten_m2 = f_Power_Static(10, -2); _ASSERTE(ten_m2 == 1.0 / 100);
	double ten_m3 = f_Power_Static(10, -3); _ASSERTE(ten_m3 == 1.0 / 1000);
	double ten_m4 = f_Power_Static(10, -4); _ASSERTE(ten_m4 == 1.0 / 10000);
	double ten_m5 = f_Power_Static(10, -5); _ASSERTE(ten_m5 == 1.0 / 100000);
	double two_0 = f_Power_Static(2, 0); _ASSERTE(two_0 == 1.0);
	double two_1 = f_Power_Static(2, 1); _ASSERTE(two_1 == 2.0);
	double two_2 = f_Power_Static(2, 2); _ASSERTE(two_2 == 4.0);
	double two_3 = f_Power_Static(2, 3); _ASSERTE(two_3 == 8.0);
	double two_4 = f_Power_Static(2, 4); _ASSERTE(two_4 == 16.0);
	double two_5 = f_Power_Static(2, 5); _ASSERTE(two_5 == 32.0);
	double two_m1 = f_Power_Static(2, -1); _ASSERTE(two_m1 == 1.0 / 2);
	double two_m2 = f_Power_Static(2, -2); _ASSERTE(two_m2 == 1.0 / 4);
	double two_m3 = f_Power_Static(2, -3); _ASSERTE(two_m3 == 1.0 / 8);
	double two_m4 = f_Power_Static(2, -4); _ASSERTE(two_m4 == 1.0 / 16);
	double two_m5 = f_Power_Static(2, -5); _ASSERTE(two_m5 == 1.0 / 32);
}

#endif // __BIT_HACKS_UNIT_TESTS

} // ~ul_bithacks

/*
 *								=== ~bit hacks ===
 */

#endif // !__INTEGER_TYPES_INCLUDED
