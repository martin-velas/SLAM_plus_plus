/*
								+----------------------------------+
								|                                  |
								|  ***   Unit prefix format   ***  |
								|                                  |
								|   Copyright © -tHE SWINe- 2016   |
								|                                  |
								|          FormatPrefix.h          |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __UNIT_PREFIX_FORMAT_INCLUDED
#define __UNIT_PREFIX_FORMAT_INCLUDED

/**
 *	@file FormatPrefix.h
 *	@brief unit prefix format
 *	@date 2016
 *	@author -tHE SWINe-
 */

#include <utility> // std::pair
#include <math.h> // fabs()


#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200

/**
 *	@brief converts 64-bit integer to double
 *	@param[in] x is 64-bit value to be converted
 *	@return Returns x converted to double.
 *	@note This is required in msvc60 to avoid "<tt>C2520: conversion
 *		from unsigned __int64 to double not implemented</tt>".
 */
inline double f_uint64_to_double(uint64_t x)
{
	if(x <= INT64_MAX)
		return double(int64_t(x));
	else /*if(x / 2 <= INT64_MAX)*/ {
		_ASSERTE(x / 2 <= INT64_MAX);
		return double(int64_t(x / 2)) * 2;
	}
}

#else // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

/**
 *	@brief converts 64-bit integer to double
 *	@param[in] x is 64-bit value to be converted
 *	@return Returns x converted to double.
 */
inline double f_uint64_to_double(uint64_t x)
{
	return double(x);
}

#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

template <class T>
inline double f_MkDbl(T x)
{
	return double(x);
}

inline double f_MkDbl(uint64_t x)
{
	return f_uint64_to_double(x);
}

/**
 *	@def PRIsizeB
 *
 *	@brief simple macro for printing file sizes in user-readable format
 *
 *	This macro is used in conjunction with PRIsizeBparams.
 */
#define PRIsizeB "%.*f %s"

/**
 *	@def PRIsizeBparams
 *
 *	@brief simple macro for expanding size in bytes to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIsizeB for printing sizes. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeBparams(256));
 *		printf("%d = " PRIsizeB "B\n", 1000, PRIsizeBparams(1000));
 *		printf("%d = " PRIsizeB "B\n", 1024 + 256, PRIsizeBparams(1024 + 256));
 *		printf("%d = " PRIsizeB "B\n", 1000000, PRIsizeBparams(1000000));
 *		printf("%d = " PRIsizeB "B\n", 1048576, PRIsizeBparams(1048576));
 *	}
 *	@endcode
 *
 *	Produces the following output:
 *
 *	@code
 *	256 = 256 B
 *	1000 = 1000 B
 *	1280 = 1.25 kB
 *  1000000 = 976.56 kB
 *	1048576 = 1.00 MB
 *	@endcode
 *
 *	More extended version of this macro is PRIsizeBparamsExt.
 *
 *	@param[in] n_size is size in bytes
 *
 *	@note To format the units with the "i" (e.g. "1.25 kiB"), use \ref PRIsizeiBparams.
 */
#define PRIsizeBparams(n_size) (((n_size) < 1024)? 0 : 2), (((n_size) < 1024)? int(n_size) : ((n_size) < 1048576)? \
	f_MkDbl(n_size) / 1024.0 : ((n_size) < 1073741824)? f_MkDbl(n_size) / 1048576.0 : ((n_size) < 1099511627776)? \
	f_MkDbl(n_size) / 1073741824.0 : f_MkDbl(n_size) / 1099511627776.0), (((n_size) < 1024)? "" : ((n_size) < 1048576)? \
	"k" : ((n_size) < 1073741824)? "M" : ((n_size) < 1099511627776)? "G" : "T")

/**
 *	@def PRIsizeBparamsExt
 *
 *	@brief simple macro for expanding size in bytes to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIsizeB for printing sizes. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeBparamsExt(256, 1, 2, false));
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeBparamsExt(256, 0, 2, true));
 *		printf("%d = " PRIsizeB "B\n", 1000, PRIsizeBparamsExt(1000, 0, 2, true));
 *		printf("%d = " PRIsizeB "B\n", 1000000, PRIsizeBparamsExt(1000000, 0, 3, false));
 *		printf("%d = " PRIsizeB "B\n", 1048576, PRIsizeBparamsExt(1048576, 0, 3, false));
 *	}
 *	@endcode
 *
 *	Produces the following output:
 *
 *	@code
 *	256 = 256.0 B
 *	256 = 0.26 kB
 *	1000 = 0.98 kB
 *	1000000 = 976.563 kB
 *	1048576 = 1.000 MB
 *	@endcode
 *
 *	@param[in] n_size is size in bytes
 *	@param[in] n_bytes_dec_num is number of fractional digits when printing size in bytes
 *	@param[in] n_dec_num is number of fractional digits when printing size not in bytes
 *	@param[in] b_force_kB is flag, controlling wheter to allow output in bytes (false),
 *		or to force output in kilobytes (true); does not have any effect if n_size >= 1024
 *
 *	@note To format the units with the "i" (e.g. "1.25 kiB"), use \ref PRIsizeiBparamsExt.
 */
#define PRIsizeBparamsExt(n_size,n_bytes_dec_num,n_dec_num,b_force_kB) (((n_size) < 1024 && !b_force_kB)? \
	n_bytes_dec_num : n_dec_num), (((n_size) < 1024 && !b_force_kB)? int(n_size) : ((n_size) < 1048576)? \
	f_MkDbl(n_size) / 1024.0 : ((n_size) < 1073741824)? f_MkDbl(n_size) / 1048576.0 : ((n_size) < 1099511627776)? \
	f_MkDbl(n_size) / 1073741824.0 : f_MkDbl(n_size) / 1099511627776.0), (((n_size) < 1024 && !b_force_kB)? \
	"" : ((n_size) < 1048576)? "k" : ((n_size) < 1073741824)? "M" : ((n_size) < 1099511627776)? "G" : "T")

/**
 *	@def PRIsizeiBparams
 *
 *	@brief simple macro for expanding size in bytes to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIsizeB for printing sizes. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeiBparams(256));
 *		printf("%d = " PRIsizeB "B\n", 1000, PRIsizeiBparams(1000));
 *		printf("%d = " PRIsizeB "B\n", 1024 + 256, PRIsizeiBparams(1024 + 256));
 *		printf("%d = " PRIsizeB "B\n", 1000000, PRIsizeiBparams(1000000));
 *		printf("%d = " PRIsizeB "B\n", 1048576, PRIsizeiBparams(1048576));
 *	}
 *	@endcode
 *
 *	Produces the following output:
 *
 *	@code
 *	256 = 256 B
 *	1000 = 1000 B
 *	1280 = 1.25 kiB
 *  1000000 = 976.56 kiB
 *	1048576 = 1.00 MiB
 *	@endcode
 *
 *	More extended version of this macro is PRIsizeBparamsExt.
 *
 *	@param[in] n_size is size in bytes
 */
#define PRIsizeiBparams(n_size) (((n_size) < 1024)? 0 : 2), (((n_size) < 1024)? int(n_size) : ((n_size) < 1048576)? \
	f_MkDbl(n_size) / 1024.0 : ((n_size) < 1073741824)? f_MkDbl(n_size) / 1048576.0 : ((n_size) < 1099511627776)? \
	f_MkDbl(n_size) / 1073741824.0 : f_MkDbl(n_size) / 1099511627776.0), (((n_size) < 1024)? "" : ((n_size) < 1048576)? \
	"ki" : ((n_size) < 1073741824)? "Mi" : ((n_size) < 1099511627776)? "Gi" : "Ti")

/**
 *	@def PRIsizeiBparamsExt
 *
 *	@brief simple macro for expanding size in bytes to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIsizeB for printing sizes. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeiBparamsExt(256, 1, 2, false));
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeiBparamsExt(256, 0, 2, true));
 *		printf("%d = " PRIsizeB "B\n", 1000, PRIsizeiBparamsExt(1000, 0, 2, true));
 *		printf("%d = " PRIsizeB "B\n", 1000000, PRIsizeiBparamsExt(1000000, 0, 3, false));
 *		printf("%d = " PRIsizeB "B\n", 1048576, PRIsizeiBparamsExt(1048576, 0, 3, false));
 *	}
 *	@endcode
 *
 *	Produces the following output:
 *
 *	@code
 *	256 = 256.0 B
 *	256 = 0.26 kiB
 *	1000 = 0.98 kiB
 *	1000000 = 976.563 kiB
 *	1048576 = 1.000 MiB
 *	@endcode
 *
 *	More extended version of this macro is PRIsizeBparamsExt.
 *
 *	@param[in] n_size is size in bytes
 *	@param[in] n_bytes_dec_num is number of fractional digits when printing size in bytes
 *	@param[in] n_dec_num is number of fractional digits when printing size not in bytes
 *	@param[in] b_force_kiB is flag, controlling wheter to allow output in bytes (false),
 *		or to force output in kilobytes (true); does not have any effect if n_size >= 1024
 */
#define PRIsizeiBparamsExt(n_size,n_bytes_dec_num,n_dec_num,b_force_kiB) (((n_size) < 1024 && !b_force_kiB)? \
	n_bytes_dec_num : n_dec_num), (((n_size) < 1024 && !b_force_kiB)? int(n_size) : ((n_size) < 1048576)? \
	f_MkDbl(n_size) / 1024.0 : ((n_size) < 1073741824)? f_MkDbl(n_size) / 1048576.0 : ((n_size) < 1099511627776)? \
	f_MkDbl(n_size) / 1073741824.0 : f_MkDbl(n_size) / 1099511627776.0), (((n_size) < 1024 && !b_force_kiB)? \
	"" : ((n_size) < 1048576)? "ki" : ((n_size) < 1073741824)? "Mi" : ((n_size) < 1099511627776)? "Gi" : "Ti")

/**
 *	@brief unit prefix format functions
 */
namespace upf {

/**
 *	@brief prescales the value and chooses the corresponding metric prefix
 *
 *	@param[in] x is input value
 *	@param[in] b_avoid_prefixless is unit prefix preference flag for values in the <tt>[1, 1000)</tt>
 *		range (if set, the prefix will be "k", if cleared the prefix will be empty)
 *
 *	@return Returns a pair of a prescaled value x and a null-terminated
 *		string containing the corresponding unit prefix.
 */
inline std::pair<double, const char*> t_MetricValue_Prescale_Prefix(double x,
	bool b_avoid_prefixless = false)
{
	typedef const char *P; // todo - in msvc6 this does not compile with "(const char*)" for some reason
	if(!b_avoid_prefixless && x == 0)
		return std::make_pair(x, (P)""); // - (-)
	double s = fabs(x);
	if(s < 1e-12)
		return std::make_pair(x * 1e15, (P)"f"); // f (femto)
	if(s < 1e-9)
		return std::make_pair(x * 1e12, (P)"p"); // p (pico)
	if(s < 1e-6)
		return std::make_pair(x * 1e9, (P)"n"); // n (nano)
	if(s < 1e-3)
		return std::make_pair(x * 1e6, (P)"u"); // u (micro)
	if(s < 1.0)
		return std::make_pair(x * 1e3, (P)"m"); // m (mili)
	if(!b_avoid_prefixless && s < 1e3)
		return std::make_pair(x, (P)""); // - (-)
	if(s < 1e6)
		return std::make_pair(x * 1e-3, (P)"k"); // k (kilo)
	if(s < 1e9)
		return std::make_pair(x * 1e-6, (P)"M"); // M (mega)
	if(s < 1e12)
		return std::make_pair(x * 1e-9, (P)"G"); // G (giga)
	return std::make_pair(x * 1e-12, (P)"T"); // T (tera)
}

/**
 *	@brief prescales the value and chooses the corresponding metric prefix
 *
 *	@tparam T is input data type
 *
 *	@param[in] x is input value
 *	@param[in] b_avoid_prefixless is unit prefix preference flag for values in the <tt>[1, 1000)</tt>
 *		range (if set, the prefix will be "k", if cleared the prefix will be empty)
 *
 *	@return Returns prescaled value x so that it stays in <tt>[1, 1000)</tt> or <tt>(-1000, 1]</tt>.
 */
template <class T>
inline double f_MetricValue_Prescale(T x, bool b_avoid_prefixless = false)
{
	return t_MetricValue_Prescale_Prefix(f_MkDbl(x), b_avoid_prefixless).first;
	// so that the same branches are used¨for both scaler / prefix
}

/**
 *	@brief prescales the value and chooses the corresponding metric prefix
 *
 *	@tparam T is input data type
 *
 *	@param[in] x is input value
 *	@param[in] b_avoid_prefixless is unit prefix preference flag for values in the <tt>[1, 1000)</tt>
 *		range (if set, the prefix will be "k", if cleared the prefix will be empty)
 *
 *	@return Returns a null-terminated string containing the unit prefix.
 */
template <class T>
inline const char *p_s_MetricValue_Prefix(T x, bool b_avoid_prefixless = false)
{
	return t_MetricValue_Prescale_Prefix(f_MkDbl(x), b_avoid_prefixless).second;
	// so that the same branches are used for both scaler / prefix
}

} // ~upf

/**
 *	@def PRIvalueMP
 *
 *	@brief simple macro for printing values with metric prefix
 *
 *	This macro is used in conjunction with PRIvalueMPparams.
 */
#define PRIvalueMP "%.*f %s"

/**
 *	@def PRIvalueMPns
 *
 *	@brief simple macro for printing values with metric prefix
 *		but without space between the value and the prefix
 *
 *	This macro is used in conjunction with PRIvalueMPparams.
 */
#define PRIvalueMPns "%.*f%s"

/**
 *	@def PRIvalueMPparams
 *
 *	@brief simple macro for expanding size in metric elements to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIvalueMP for printing values. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIvalueMP "\n", 256, PRIvalueMPparams(256));
 *		printf("%d = " PRIvalueMP "\n", 1000, PRIvalueMPparams(1000));
 *		printf("%d = " PRIvalueMP "\n", 1024 + 256, PRIvalueMPparams(1024 + 256));
 *		printf("%d = " PRIvalueMP "\n", 1000000, PRIvalueMPparams(1000000));
 *		printf("%d = " PRIvalueMP "\n", 1048576, PRIvalueMPparams(1048576));
 *	}
 *	@endcode
 *
 *	Produces the following output:
 *
 *	@code
 *	256 = 256.00
 *	1000 = 1.00 k
 *	1280 = 1.28 k
 *	1000000 = 1.00 M
 *	1048576 = 1.05 M
 *	@endcode
 *
 *	More extended version of this macro is PRIvalueMPparamsExt.
 *
 *	@param[in] v is value
 */
#define PRIvalueMPparams(v) 2, upf::f_MetricValue_Prescale(v), upf::p_s_MetricValue_Prefix(v)

/**
 *	@def PRIvalueMPparams
 *
 *	@brief simple macro for expanding size in metric elements to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIvalueMP for printing values. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIvalueMP "\n", 256, PRIvalueMPparamsExt(256, 0, 2, false));
 *		printf("%d = " PRIvalueMP "\n", 256, PRIvalueMPparamsExt(256, 2, 2, true));
 *		printf("%d = " PRIvalueMP "\n", 1024, PRIvalueMPparamsExt(1024, 3, 3, false));
 *		printf("%d = " PRIvalueMP "\n", 1000000, PRIvalueMPparamsExt(1000000, 3, 3, false));
 *		printf("%d = " PRIvalueMP "\n", 1048576, PRIvalueMPparamsExt(1048576, 3, 3, false));
 *	}
 *	@endcode
 *
 *	Produces the following output:
 *
 *	@code
 *	256 = 256
 *	256 = 0.26 k
 *	1024 = 1.024 k
 *	1000000 = 1.000 M
 *	1048576 = 1.049 M
 *	@endcode
 *
 *	@param[in] v is value
 *	@param[in] prefixLessDecNum is number of decimals for numbers without a prefix
 *	@param[in] decNum is number of decimals for numbers with prefix
 *	@param[in] avoidprefixLess is flag for avoiding prefix-less notation
 *		(numbers in range <tt>1 <= v < 1000</tt> are then formatted using the kilo prefix)
 */
#define PRIvalueMPparamsExt(v,prefixLessDecNum,decNum,avoidprefixLess) \
	(((v == 0 || (fabs(f_MkDbl(v)) >= 1.0 && fabs(f_MkDbl(v)) < 1e3)) && !(avoidprefixLess))? \
	(prefixLessDecNum) : (decNum)), upf::f_MetricValue_Prescale((v), (avoidprefixLess)), \
	upf::p_s_MetricValue_Prefix((v), (avoidprefixLess))

#endif // __UNIT_PREFIX_FORMAT_INCLUDED
