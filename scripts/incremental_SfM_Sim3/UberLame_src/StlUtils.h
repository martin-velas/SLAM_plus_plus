/*
								+----------------------------------+
								|                                  |
								|     ***  STL utilities  ***      |
								|                                  |
								|   Copyright © -tHE SWINe- 2008   |
								|                                  |
								|            StlUtils.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __STL_UTILS_INCLUDED
#define __STL_UTILS_INCLUDED

/**
 *	@file StlUtils.h
 *	@date 2008
 *	@author -tHE SWINe-
 *	@brief STL utilities
 *
 *	@date 2008-12-22
 *
 *	added typename keyword in Reserve_NMore() so it now compiles with g++
 *
 *	@date 2009-01-13
 *
 *	added Format() function and StlUtils.cpp to keep it's body (it's not template)
 *
 *	@date 2009-03-24
 *
 *	added Assign and Append, accepting std::string as second argument,
 *	should allow assigning and appending strings with different character
 *	types as well (no conversion or error handling though).
 *
 *	(there was just const char* before; it was possible to use these versions
 *	to append std::string to std::string using c_str() function, but it
 *	would expectably yield worse performance)
 *
 *	@date 2009-05-04
 *
 *	stl_ut::Swap() was removed in favor of std::swap
 *
 *	added __STL_UTILS_ENABLE_EXCEPTIONS macro, controlling between exception-
 *	based and null-pointer based errors on operator new()
 *
 *	written documentation comments
 *
 *	@date 2009-10-11
 *
 *	added Resize_To_N(), Resize_Add_1More() and Resize_Add_NMore() functions
 *	to help avoiding unhandled std::bad_alloc in container::resize()
 *
 *	added AssignWCStr(), AppendWCStr() and FormatW() to support wide character
 *	strings, can disable them by defining __STL_UTILS_NO_WIDE_STRINGS macro
 *
 *	added __STL_UT_CATCH_BAD_ALLOC macro, containing catch(std::bad_alloc&)
 *	(in case __STL_UTILS_ENABLE_EXCEPTIONS is not defined)
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 *	@date 2010-02-12
 *
 *	added __stl_ut_try, __stl_ut_catch and __stl_ut_catch_bad_alloc aliases
 *
 *	@date 2010-08-09
 *
 *	fixed error in stl_ut::Resize_Add_NMore(), which didn't resize container
 *	in case it had sufficient capacity (regardless of size)
 *
 *	@date 2011-07-18
 *
 *	Added the stl_ut::Split() function for splitting a string to a list
 *	of substrings by a separator.
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2013-12-10
 *
 *	Decided to add TrimSpace() and ReadFile() functions. These are not quite versatile,
 *	as TrimSpace() only handles us-english whitespaces and not unicode, and ReadFile()
 *	will run into problems when using large files, but they are still useful for most
 *	of the applications.
 *
 */

/**
 *	@def __STL_UTILS_NO_WIDE_STRINGS
 *	@brief If defined, AssignWCStr(), AppendWCStr() and FormatW() won't be
 *		compiled (this may be needed in environments where wchar_t.h is not
 *		available and/or wcslen() or vsnwprintf() not defined).
 */
//#define __STL_UTILS_NO_WIDE_STRINGS

/*#if !defined(_WIN32) && !defined(_WIN64) && !defined(__STL_UTILS_NO_WIDE_STRINGS)
#define __STL_UTILS_NO_WIDE_STRINGS // unix has no vsnwsprintf but it has vswprintf
#endif // !_WIN32 && !_WIN64 && !__STL_UTILS_NO_WIDE_STRINGS*/

/**
 *	@def __STL_UTILS_ENABLE_EXCEPTIONS
 *	@brief if defined, it is expected that std::container::reserve() throws
 *		std::bad_alloc once it runs out of memory
 *
 *	The other model would be not increasing storage capacity,
 *	detected by comparing std::container::capacity() with requested
 *	number. This happens when operator new() returns 0 when there's
 *	not enough memory (patched versions of STL for MSVC 6.0 and older).
 */
#define __STL_UTILS_ENABLE_EXCEPTIONS

#ifdef __STL_UTILS_ENABLE_EXCEPTIONS
#define __STL_UT_TRY try
#define __STL_UT_CATCH(CException) catch(CException)
#else // __STL_UTILS_ENABLE_EXCEPTIONS
#define __STL_UT_TRY if(true)
#define __STL_UT_CATCH(CException) if(false)
#endif // __STL_UTILS_ENABLE_EXCEPTIONS

#define __STL_UT_CATCH_BAD_ALLOC __STL_UT_CATCH(std::bad_alloc&)

#define __stl_ut_try __STL_UT_TRY
#define __stl_ut_catch __STL_UT_CATCH
#define __stl_ut_catch_bad_alloc __STL_UT_CATCH_BAD_ALLOC
// allow small letters here

#include <string>
#include <string.h>
#include <vector>
#include <utility>
#include <algorithm> // lower_bound
#include "Unused.h"

/**
 *	@brief STL utilities
 */
namespace stl_ut {

/**
 *	@brief reserves space for N elements in a container
 *
 *	@tparam CContainer is container type
 *
 *	@param[in,out] r_vec is container to reserve space in
 *	@param[in] n is number of elements to reserve space for (n elements, not for n more)
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer>
inline bool Reserve_N(CContainer &r_vec, size_t n) throw()
{
	__STL_UT_TRY {
		r_vec.reserve(n);
		return r_vec.capacity() >= n;
		// reserve and check
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// not enough memory
	}
}

/**
 *	@brief resizes a container to N elements
 *
 *	@tparam CContainer is container type
 *
 *	@param[in,out] r_vec is container to be resized
 *	@param[in] n is number of elements to allocate space for (n elements, not for n more)
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer>
inline bool Resize_To_N(CContainer &r_vec, size_t n) throw()
{
	__STL_UT_TRY {
		r_vec.resize(n);
		return true;
		// just resize
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// not enough memory
	}
}

/**
 *	@brief resizes a container to N elements
 *
 *	@tparam CContainer is container type
 *	@tparam CElem is initializer type (may be different from element type)
 *
 *	@param[in,out] r_vec is container to be resized
 *	@param[in] n is number of elements to allocate space for (n elements, not for n more)
 *	@param[in] r_t_initializer is initializer value for newly added elements
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer, class CElem>
inline bool Resize_To_N(CContainer &r_vec, size_t n, const CElem &r_t_initializer) throw()
{
	__STL_UT_TRY {
		r_vec.resize(n, r_t_initializer);
		return true;
		// just resize
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// not enough memory
	}
}

/**
 *	@brief reserves space for 1 more element in a container
 *	@tparam CContainer is container type
 *	@param[in,out] r_vec is container to reserve space in
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer>
inline bool Reserve_1More(CContainer &r_vec) throw()
{
	__STL_UT_TRY {
		if(r_vec.capacity() == r_vec.size()) {
			r_vec.reserve((r_vec.size() <= r_vec.max_size() / 2)?
				(r_vec.size() * 2) | 1 : r_vec.max_size());
			// watch out for requested capacity overflow
			return r_vec.capacity() > r_vec.size();
		}
		return true;
	} __STL_UT_CATCH_BAD_ALLOC {
		// not enough memory
		return r_vec.size() < r_vec.max_size() && Reserve_N(r_vec, r_vec.size() + 1);
		// try allocating exactly one more (in case there's some memory left,
		// but not enough for twice as long vector)
	}
}

/**
 *	@brief allocates space for 1 more element in a container
 *	@tparam CContainer is container type
 *	@param[in,out] r_vec is container to allocate space in
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer>
inline bool Resize_Add_1More(CContainer &r_vec) throw()
{
	if(r_vec.size() == r_vec.max_size())
		return false;
	__STL_UT_TRY {
		/*if(r_vec.capacity() == r_vec.size()) {
			r_vec.reserve((r_vec.size() <= r_vec.max_size() / 2)?
				(r_vec.size() * 2) | 1 : r_vec.max_size());
			// watch out for requested capacity overflow
		}*/
		r_vec.resize(r_vec.size() + 1);
		return true;
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// not enough memory
		//return r_vec.size() < r_vec.max_size() && Resize_To_N(r_vec, r_vec.size() + 1);
		// try allocating exactly one more (in case there's some memory left,
		// but not enough for twice as long vector)
	}
}

/**
 *	@brief allocates space for 1 more element in a container
 *
 *	@tparam CContainer is container type
 *	@tparam CElem is initializer type (may be different from element type)
 *
 *	@param[in,out] r_vec is container to reserve space in
 *	@param[in] r_t_initializer is initializer value for newly added element
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer, class CElem>
inline bool Resize_Add_1More(CContainer &r_vec, const CElem &r_t_initializer) throw()
{
	__STL_UT_TRY {
		/*if(r_vec.capacity() == r_vec.size()) {
			r_vec.reserve((r_vec.size() <= r_vec.max_size() / 2)?
				(r_vec.size() * 2) | 1 : r_vec.max_size());
			// watch out for requested capacity overflow
		}
		r_vec.resize(r_vec.size() + 1, r_t_initializer);*/
		r_vec.push_back(r_t_initializer); // this already contains code above (sgi version of stl)
		return true;
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// not enough memory
		/*return r_vec.size() < r_vec.max_size() &&
			Resize_To_N(r_vec, r_vec.size() + 1, r_t_initializer);*/ // will not succeed
		// try allocating exactly one more (in case there's some memory left,
		// but not enough for twice as long vector)
	}
}

/**
 *	@brief reserves space for N more element in a container
 *
 *	@tparam CContainer is container type
 *
 *	@param[in,out] r_vec is container to reserve space in
 *	@param[in] n is number of elements to allocate space for (n more elements, not for n)
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer>
inline bool Reserve_NMore(CContainer &r_vec, size_t n) throw()
{
	const typename CContainer::size_type n_max = r_vec.max_size();
	// get maximal size of container

	if(r_vec.size() > n_max - n)
		return false;
	// see if n items can be inserted

	typename CContainer::size_type n_min_new_capa = r_vec.size() + n;
#if 1
	return Reserve_N(r_vec, n_min_new_capa);
#else // 1
	if(r_vec.capacity() < n_min_new_capa) {
#if 0
		typename CContainer::size_type n_new_capa = r_vec.capacity();
		while(n_new_capa < n_min_new_capa) {
			if(n_new_capa >= n_max / 2) {
				n_new_capa = n_max;
				break;
			}
			// watch out for overflow

			n_new_capa *= 2;
			n_new_capa |= 1;
		}
		// calculate new capacity, scale by factor of two
#else // 0
		typename CContainer::size_type n_new_capa = (r_vec.capacity() * 2) | 1;

		if(n_new_capa < n_min_new_capa)
			n_new_capa = n_min_new_capa;
		else if(n_new_capa > n_max)
			n_new_capa = n_max;
		// scale by factor of two, do it more simple (sufficient in most cases)
#endif // 0

		__STL_UT_TRY {
			r_vec.reserve(n_new_capa);
			return r_vec.capacity() >= n_min_new_capa;
			// reserve and check
		} __STL_UT_CATCH_BAD_ALLOC {
			// not enough memory
			return Reserve_N(r_vec, n_min_new_capa);
		}
	}
	return true;
#endif // 1
}

/**
 *	@brief allocates space for N more element in a container
 *
 *	@tparam CContainer is container type
 *
 *	@param[in,out] r_vec is container to allocate space in
 *	@param[in] n is number of elements to allocate space for (n more elements, not for n)
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer>
inline bool Resize_Add_NMore(CContainer &r_vec, size_t n) throw()
{
	const typename CContainer::size_type n_max = r_vec.max_size();
	// get maximal size of container

	if(r_vec.size() > n_max - n)
		return false;
	// see if n items can be inserted

	typename CContainer::size_type n_min_new_capa = r_vec.size() + n;
	/*if(r_vec.capacity() < n_min_new_capa) {
		typename CContainer::size_type n_new_capa = (r_vec.capacity() * 2) | 1;

		if(n_new_capa < n_min_new_capa)
			n_new_capa = n_min_new_capa;
		else if(n_new_capa > n_max)
			n_new_capa = n_max;
		// scale by factor of two, do it more simple (sufficient in most cases)

		__STL_UT_TRY {
			r_vec.reserve(n_new_capa);
			r_vec.resize(n_min_new_capa);
			return true;
			// reserve and check
		} __STL_UT_CATCH_BAD_ALLOC {
			// not enough memory
			//return Resize_To_N(r_vec, n_min_new_capa); // just fall trough
		}
	}*/
	return Resize_To_N(r_vec, n_min_new_capa); // just very simple function, safe to call here
}

/**
 *	@brief allocates space for N more element in a container
 *
 *	@tparam CContainer is container type
 *	@tparam CElem is initializer type (may be different from element type)
 *
 *	@param[in,out] r_vec is container to allocate space in
 *	@param[in] n is number of elements to allocate space for (n more elements, not for n)
 *	@param[in] r_t_initializer is initializer value for newly added elements
 *
 *	@return Returns true on success, false in case there's not enough memory.
 */
template <class CContainer, class CElem>
inline bool Resize_Add_NMore(CContainer &r_vec, size_t n, const CElem &r_t_initializer) throw()
{
	const typename CContainer::size_type n_max = r_vec.max_size();
	// get maximal size of container

	if(r_vec.size() > n_max - n)
		return false;
	// see if n items can be inserted

	typename CContainer::size_type n_min_new_capa = r_vec.size() + n;
	/*if(r_vec.capacity() < n_min_new_capa) {
		typename CContainer::size_type n_new_capa = (r_vec.capacity() * 2) | 1;

		if(n_new_capa < n_min_new_capa)
			n_new_capa = n_min_new_capa;
		else if(n_new_capa > n_max)
			n_new_capa = n_max;
		// scale by factor of two, do it more simple (sufficient in most cases)

		__STL_UT_TRY {
			r_vec.reserve(n_new_capa);
			r_vec.resize(n_min_new_capa, r_t_initializer);
			return true;
			// reserve and check
		} __STL_UT_CATCH_BAD_ALLOC {
			// not enough memory
			//return Resize_To_N(r_vec, n_min_new_capa, r_t_initializer); // just fall trough
		}
	}*/
	return Resize_To_N(r_vec, n_min_new_capa, r_t_initializer); // just very simple function, safe to call here
}

/**
 *	@brief assigns one std::string to other, permitting character type conversions
 *
 *	@tparam CDestString is destination string type
 *	@tparam CSrcString is source string type
 *
 *	@param[out] r_s_dest is destination string (overwritten by the source string upon successful return)
 *	@param[in] r_s_src is source string
 *
 *	@return Returns true on success, false on failure.
 */
template <class CDestString, class CSrcString>
bool Assign(CDestString &r_s_dest, const CSrcString &r_s_src) throw()
{
	r_s_dest.erase(); // avoid copying in reallocation
	if(r_s_src.length() >= r_s_dest.max_size() || !Reserve_N(r_s_dest, r_s_src.length() + 1))
		return false;
	// make sure there's space in the string, also for the terminating null so that c_str() won't throw

	//r_s_dest.assign(r_s_src);
	__STL_UT_TRY {
		r_s_dest.insert(r_s_dest.begin(), r_s_src.begin(), r_s_src.end()); // permits character conversions
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// might still throw, e.g. if allocating one extra character for null-termination for c_str()
	}
	// assign

	return true;
}

/**
 *	@brief assigns null-terminated "C" string to a std::string
 *
 *	@tparam CDestString is destination string type
 *	@tparam CChar is "C" string character type
 *
 *	@param[out] r_s_dest is destination string (overwritten by the source string upon successful return)
 *	@param[in] p_s_src is source null-terminated "C" string
 *
 *	@return Returns true on success, false on failure.
 */
template <class CDestString, class CChar>
bool AssignCStr(CDestString &r_s_dest, CChar *p_s_src) throw()
{
	r_s_dest.erase(); // avoid copying in reallocation
	size_t n = strlen(p_s_src);
	if(n >= r_s_dest.max_size() || !Reserve_N(r_s_dest, n + 1))
		return false;
	// make sure there's space in the string, also for the terminating null so that c_str() won't throw

	//r_s_dest.assign(p_s_src);
	__STL_UT_TRY {
		r_s_dest.insert(r_s_dest.end(), p_s_src, p_s_src + n); // permits character conversions
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// might still throw, e.g. if allocating one extra character for null-termination for c_str()
	}
	// append

	return true;
}

/**
 *	@brief appends one std::string to other, permitting character type conversions
 *
 *	@tparam CDestString is destination string type
 *	@tparam CSrcString is source string type
 *
 *	@param[out] r_s_dest is destination string (appended with source string upon successful return)
 *	@param[in] r_s_src is source string
 *
 *	@return Returns true on success, false on failure.
 */
template <class CDestString, class CSrcString>
bool Append(CDestString &r_s_dest, const CSrcString &r_s_src) throw()
{
	if(r_s_src.length() >= r_s_dest.max_size() - r_s_dest.length() ||
	   !Reserve_N(r_s_dest, r_s_dest.length() + r_s_src.length() + 1))
		return false;
	// make sure there's space in the string, also for the terminating null so that c_str() won't throw

	//r_s_dest.append(r_s_src);
	__STL_UT_TRY {
		r_s_dest.insert(r_s_dest.end(), r_s_src.begin(), r_s_src.end()); // permits character conversions
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// might still throw, e.g. if allocating one extra character for null-termination for c_str()
	}
	// append

	return true;
}

/**
 *	@brief appends null-terminated "C" string to std::string, permitting character type conversions
 *
 *	@tparam CDestString is destination string type
 *	@tparam CChar is "C" string character type
 *
 *	@param[out] r_s_dest is destination string (appended with source string upon successful return)
 *	@param[in] p_s_src is source null-terminated "C" string
 *
 *	@return Returns true on success, false on failure.
 */
template <class CDestString, class CChar>
bool AppendCStr(CDestString &r_s_dest, CChar *p_s_src) throw()
{
	size_t n = strlen(p_s_src);
	if(n >= r_s_dest.max_size() - r_s_dest.length() || !Reserve_N(r_s_dest, n + r_s_dest.length() + 1))
		return false;
	// make sure there's space in the string, also for the terminating null so that c_str() won't throw

	//r_s_dest.append(p_s_src);
	__STL_UT_TRY {
		r_s_dest.insert(r_s_dest.end(), p_s_src, p_s_src + n); // permits character conversions
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// might still throw, e.g. if allocating one extra character for null-termination for c_str()
	}
	// append

	return true;
}

#include <string>

/**
 *	@brief equivalent of standard "C" library sprintf function, working with std::string output
 *
 *	@param[out] r_s_result is reference to the output string
 *	@param[in] p_s_fmt is standard printf format string
 *	@param[in] ... is additional arguments, corresponding to the format string
 *
 *	@return Returns true on success, false on failure (not enough memory).
 *
 *	@note This is safe against output buffer overrun, but is still susceptible to ill-formated
 *		format string (p_s_fmt) and bad arguments (not enough of them / not corresponding
 *		to tags in the format string). but as the first one being run-time error and the
 *		latter one compile-time error, this brings enought improvement.
 */
bool Format(std::string &r_s_result, const char *p_s_fmt, ...);

/**
 *	@brief equivalent of standard "C" library sprintf function, working with std::string output
 *
 *	@param[out] p_s_result is pointer to a buffer to store the output string
 *	@param[in] n_buffer_size is size of the output buffer, in bytes
 *	@param[in] p_s_fmt is standard printf format string
 *	@param[in] ... is additional arguments, corresponding to the format string
 *
 *	@return Returns true on success, false on failure (not enough memory).
 *
 *	@note This is safe against output buffer overrun, but is still susceptible to ill-formated
 *		format string (p_s_fmt) and bad arguments (not enough of them / not corresponding
 *		to tags in the format string). but as the first one being run-time error and the
 *		latter one compile-time error, this brings enought improvement.
 */
bool Format(char *p_s_result, size_t n_buffer_size, const char *p_s_fmt, ...);

/**
 *	@brief reads line form a file
 *
 *	@param[out] r_s_line is output string, containing one line read from a file
 *	@param[in] p_fr is pointer to a file
 *
 *	@return Returns true on success, false on failure (not enough memory / input error).
 *
 *	@note In case file is at it's end, output lines are empty, but the function still succeeds.
 *	@note Output lines may contain carriage-return character(s), for example if the file
 *		is opened for binary reading. Line-feed character marks end of line and is never
 *		included.
 */
bool ReadLine(std::string &r_s_line, FILE *p_fr);

/**
 *	@brief reads entire file into a string
 *
 *	@param[out] r_s_output is string to contain the file
 *	@param[in] p_s_filename is filename
 *
 *	@return Returns true on success, false on failure (not enough memory / input error).
 */
bool ReadFile(std::string &r_s_output, const char *p_s_filename);

/**
 *	@brief removes heading and trailing whitespace from a line
 *
 *	@param[in,out] r_s_string is string to have whitespace removed
 */
void TrimSpace(std::string &r_s_string);

/**
 *	@brief splits a string by a separator
 *
 *	@param[out] r_s_dest is destination vector for substrings
 *	@param[in] r_s_string is the input string
 *	@param[in] p_s_separator is the separator
 *	@param[in] n_thresh is minimal output string threshold
 *		(only strings longer than threshold are stored in r_s_dest)
 *
 *	@return Returns true on success, false on failure (not enough memory).
 */
bool Split(std::vector<std::string> &r_s_dest, const std::string &r_s_string,
	const char *p_s_separator, int n_thresh = -1);

/**
 *	@brief splits a string by several single-char separators
 *
 *	@param[out] r_s_dest is destination vector for substrings
 *	@param[in] r_s_string is the input string
 *	@param[in] p_s_separators is null-terminated string, containing the separators
 *	@param[in] n_thresh is minimal output string threshold
 *		(only strings longer than threshold are stored in r_s_dest)
 *
 *	@return Returns true on success, false on failure (not enough memory).
 */
bool SplitByMultiple(std::vector<std::string> &r_s_dest, const std::string &r_s_string,
	const char *p_s_separators, int n_thresh = -1);

#ifndef __STL_UTILS_NO_WIDE_STRINGS

#include <wchar.h>

/**
 *	@brief assigns wide null-terminated "C" string to a std::string
 *
 *	@tparam CDestString is destination string type
 *	@tparam CChar is "C" string character type
 *
 *	@param[out] r_s_dest is destination string (overwritten by the source string upon successful return)
 *	@param[in] p_s_src is source null-terminated "C" string
 *
 *	@return Returns true on success, false on failure.
 */
template <class CDestString, class CChar>
bool AssignWCStr(CDestString &r_s_dest, CChar *p_s_src) throw()
{
	r_s_dest.erase(); // avoid copying in reallocation
	size_t n = wcslen(p_s_src);
	if(n >= r_s_dest.max_size() || !Reserve_N(r_s_dest, n + 1))
		return false;
	// make sure there's space in the string, also for the terminating null so that c_str() won't throw

	//r_s_dest.assign(p_s_src);
	__STL_UT_TRY {
		r_s_dest.insert(r_s_dest.end(), p_s_src, p_s_src + n); // permits character conversions
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// might still throw, e.g. if allocating one extra character for null-termination for c_str()
	}
	// append

	return true;
}

/**
 *	@brief appends null-terminated "C" string to std::string, permitting character type conversions
 *
 *	@tparam CDestString is destination string type
 *	@tparam CChar is "C" string character type
 *
 *	@param[out] r_s_dest is destination string (appended with source string upon successful return)
 *	@param[in] p_s_src is source null-terminated "C" string
 *
 *	@return Returns true on success, false on failure.
 */
template <class CDestString, class CChar>
bool AppendWCStr(CDestString &r_s_dest, CChar *p_s_src) throw()
{
	size_t n = wcslen(p_s_src);
	if(n >= r_s_dest.max_size() - r_s_dest.length() || !Reserve_N(r_s_dest, n + r_s_dest.length() + 1))
		return false;
	// make sure there's space in the string, also for the terminating null so that c_str() won't throw

	//r_s_dest.append(p_s_src);
	__STL_UT_TRY {
		r_s_dest.insert(r_s_dest.end(), p_s_src, p_s_src + n); // permits character conversions
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// might still throw, e.g. if allocating one extra character for null-termination for c_str()
	}
	// append

	return true;
}

/**
 *	@brief equivalent of standard "C" library sprintf function, working with std::string output
 *
 *	@param[out] r_s_result is reference to the output string
 *	@param[in] p_s_fmt is standard printf format string
 *	@param[in] ... is additional arguments, corresponding to the format string
 *
 *	@return Returns true on success, false on failure (not enough memory).
 *
 *	@note This is safe against output buffer overrun, but is still susceptible to ill-formated
 *		format string (p_s_fmt) and bad arguments (not enough of them / not corresponding
 *		to tags in the format string). but as the first one being run-time error and the
 *		latter one compile-time error, this brings enought improvement.
 */
bool FormatW(std::basic_string<wchar_t> &r_s_result, const wchar_t *p_s_fmt, ...);

/**
 *	@brief equivalent of standard "C" library sprintf function, working with std::string output
 *
 *	@param[out] p_s_result is pointer to a buffer to store the output string
 *	@param[in] n_buffer_size is size of the output buffer, in bytes
 *	@param[in] p_s_fmt is standard printf format string
 *	@param[in] ... is additional arguments, corresponding to the format string
 *
 *	@return Returns true on success, false on failure (not enough memory).
 *
 *	@note This is safe against output buffer overrun, but is still susceptible to ill-formated
 *		format string (p_s_fmt) and bad arguments (not enough of them / not corresponding
 *		to tags in the format string). but as the first one being run-time error and the
 *		latter one compile-time error, this brings enought improvement.
 */
bool FormatW(wchar_t *p_s_result, size_t n_buffer_size, const wchar_t *p_s_fmt, ...);

#endif // __STL_UTILS_NO_WIDE_STRINGS

/**
 *	@brief function onjects that scalar-deletes pointers
 *	@tparam CType is data type being pointed to
 */
template <class CType>
struct CDefaultDelete {
	/**
	 *	@brief function operator; deletes a given pointer
	 *	@param[in] p_pointer is pointer to be deleted
	 */
	inline void operator ()(CType *p_pointer) const
	{
		if(p_pointer)
			delete p_pointer;
	}
};

/**
 *	@brief an empty class
 */
class CNoIface {};

/**
 *	@brief unique pointer class
 *
 *	@tparam TType is scalar type to be pointed to (not the pointer type)
 *	@tparam TDestructor is edstructor type; must implement function operator,
 *		taking the pointer (default CDefaultDelete)
 */
template <class TType, class TDestructor = CDefaultDelete<TType>, class CExtraInterface = CNoIface>
class CUniquePtr : public CExtraInterface {
public:
	typedef TType CType; /**< @brief scalar type (not pointer) */
	typedef TDestructor CDestructor; /**< @brief destructor type */

protected:
	CType *m_p_pointer; /**< @brief the managed pointer */

public:
	/**
	 *	@brief default constructor; sets pointer to a specified value
	 *	@param[in] p_pointer is pointer to become managed (default null pointer)
	 *	@note The specified pointer will be deleted automatically.
	 */
	explicit inline CUniquePtr(CType *p_pointer = 0)
		:m_p_pointer(p_pointer)
	{}

	/**
	 *	@brief copy-constructor
	 *	@param[in,out] r_other is unique pointer to copy from (it will lose ownership of the pointer)
	 */
#if defined(_MSC_VER) && !defined(__MWERKS__)
	inline CUniquePtr(/*const*/ CUniquePtr &r_other)
		:m_p_pointer(/*const_cast<CUniquePtr&>*/(r_other).p_YieldOwnership())
#else // _MSC_VER) && !__MWERKS__
	inline CUniquePtr(const CUniquePtr &r_other) // g++ won't call this if other is a remporary object (can't convert it to a non-const reference)
		:m_p_pointer(const_cast<CUniquePtr&>(r_other).p_YieldOwnership())
#endif // _MSC_VER) && !__MWERKS__
	{}

	/**
	 *	@brief destructor; deletes the currently owned pointer, if any
	 */
	inline ~CUniquePtr()
	{
		CDestructor()(m_p_pointer);
	}

	/**
	 *	@brief copy operator; deletes the currently owned pointer, if any
	 *	@param[in,out] r_other is unique pointer to copy from (it will lose ownership of the pointer)
	 *	@return Returns reference to this.
	 */
#if defined(_MSC_VER) && !defined(__MWERKS__)
	inline CUniquePtr &operator =(/*const*/ CUniquePtr &r_other)
	{
		CDestructor()(m_p_pointer);//Destroy();
		m_p_pointer = /*const_cast<CUniquePtr&>*/(r_other).p_YieldOwnership();
		return *this;
	}
#else // _MSC_VER) && !__MWERKS__
	inline CUniquePtr &operator =(const CUniquePtr &r_other) // g++ won't call this if other is a remporary object (can't convert it to a non-const reference)
	{
		CDestructor()(m_p_pointer);//Destroy();
		m_p_pointer = const_cast<CUniquePtr&>(r_other).p_YieldOwnership();
		return *this;
	}
#endif // _MSC_VER) && !__MWERKS__

	/**
	 *	@brief deletes the currently owned pointer, if any
	 */
	inline void Destroy()
	{
		CDestructor()(m_p_pointer);//this->~CUniquePtr();
		m_p_pointer = 0;
	}

	/**
	 *	@brief yields ownership of the pointer, which is to be no longer managed
	 *	@return Returns the currently owned pointer. This object owns no pointer upon return.
	 *	@note This would be usually called "Release" but in some contexts that refers to
	 *		destroying an object and the meaning would be misleading.
	 */
	inline CType *p_YieldOwnership()
	{
		CType *p_result = m_p_pointer;
		m_p_pointer = 0;
		return p_result;
	}

	/**
	 *	@brief conversion to pointer
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline CType *p_Get() const
	{
		return m_p_pointer;
	}

	/**
	 *	@brief conversion to bool
	 *	@return Returns true if the owned pointer is not null, otherwise returns false.
	 */
	inline operator bool() const
	{
		return m_p_pointer != 0;
	}

	/**
	 *	@brief const indirection operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline const CType &operator *() const
	{
		return *m_p_pointer;
	}

	/**
	 *	@brief indirection operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline CType &operator *()
	{
		return *m_p_pointer;
	}

	/**
	 *	@brief const member access operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline const CType *operator ->() const
	{
		return m_p_pointer;
	}

	/**
	 *	@brief member access operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline CType *operator ->()
	{
		return m_p_pointer;
	}

	/**
	 *	@brief swaps two unique handles
	 *	@param[in,out] r_other is pointer to swap with
	 */
	inline void Swap(CUniquePtr &r_other)
	{
		std::swap(r_other.m_p_pointer, m_p_pointer);
	}
};

/**
 *	@brief function onjects that deletes pointers stored in a container object (e.g. in a vector)
 *	@tparam CContainerDelete is data type of the container, must contain pointers
 */
template <class CContainerType>
struct CContainerDelete {
	struct CMyDelete {
		template <class CValueType>
		inline void operator ()(CValueType *p_pointer) const
		{
			if(p_pointer)
				delete p_pointer;
		}
	};

	/**
	 *	@brief function operator; deletes a given pointer
	 *	@param[in] p_container is pointer to a container with objects to be deleted (the container itself is not deleted)
	 */
	inline void operator ()(CContainerType *p_container) const
	{
		std::for_each(p_container->begin(), p_container->end(), CMyDelete());
		p_container->clear(); // !!
	}
};

/**
 *	@brief guard object which deletes pointers from a container upon destruction
 *	@tparam CContainerType is container type, e.g. <tt>std::vector<int*></tt>
 */
template <class CContainerType>
class CPtrContainer_Guard {
public:
	typedef CContainerType CType;
	typedef CContainerDelete<CContainerType> CDestructor;

protected:
	typedef CUniquePtr<CType, CDestructor> _TyPtr; /**< @brief unique pointer type */

	_TyPtr m_ptr;

public:
	/**
	 *	@brief default constructor; binds the container object
	 */
	CPtrContainer_Guard(CContainerType &r_container)
		:m_ptr(&r_container)
	{}

	/**
	 *	@brief deletes the currently owned pointer, if any
	 */
	inline void Destroy()
	{
		m_ptr.Destroy();
	}

	/**
	 *	@brief yields ownership of the pointer, which is to be no longer managed
	 *	@return Returns the currently owned pointer. This object owns no pointer upon return.
	 *	@note This would be usually called "Release" but in some contexts that refers to
	 *		destroying an object and the meaning would be misleading.
	 */
	inline CType *p_YieldOwnership()
	{
		return m_ptr.p_YieldOwnership();
	}

	/**
	 *	@brief conversion to pointer
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline CType *p_Get() const
	{
		return m_ptr.p_Get();
	}

	/**
	 *	@brief conversion to bool
	 *	@return Returns true if the owned pointer is not null, otherwise returns false.
	 */
	inline operator bool() const
	{
		return (bool)m_ptr;
	}

	/**
	 *	@brief const indirection operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline const CType &operator *() const
	{
		return *m_ptr;
	}

	/**
	 *	@brief indirection operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline CType &operator *()
	{
		return *m_ptr;
	}

	/**
	 *	@brief const member access operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline const CType *operator ->() const
	{
		return m_ptr.p_Get();
	}

	/**
	 *	@brief member access operator
	 *	@return Returns the currently owned pointer (may be a null pointer).
	 *	@note This does not yield ownership of the pointer (it stays managed).
	 */
	inline CType *operator ->()
	{
		return m_ptr.p_Get();
	}

	/**
	 *	@brief swaps two unique handles
	 *	@param[in,out] r_other is pointer to swap with
	 */
	inline void Swap(CPtrContainer_Guard<CContainerType> &r_other)
	{
		r_other.m_ptr.Swap(m_ptr);
	}

protected:
	CPtrContainer_Guard(const CPtrContainer_Guard<CContainerType> &r_other); // no-copy
	CPtrContainer_Guard &operator =(const CPtrContainer_Guard<CContainerType> &r_other); // no-copy
};

/**
 *	@brief exclusive prefix sum function
 *
 *	@tparam CForwardIterator is forward iterator type
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sequence
 */
template <class CForwardIterator>
void ExclusiveScan(CForwardIterator p_begin_it, CForwardIterator p_end_it)
{
	_ASSERTE(p_begin_it <= p_end_it);
	typename std::iterator_traits<CForwardIterator>::value_type n_sum = 0; // msvc6 would have to use _Val_type() hack to infer the value type (easy hack) // t_odo - figure out how to get that from CRandomIterator also in case it i
	for(; p_begin_it != p_end_it; ++ p_begin_it) {
		typename std::iterator_traits<CForwardIterator>::value_type n = *p_begin_it;
		*p_begin_it = n_sum;
		n_sum += n;
	}
}

/**
 *	@brief exclusive prefix sum function
 *
 *	@tparam CConstForwardIterator is constant forward iterator type
 *	@tparam CForwardIterator is forward iterator type
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sequence
 *	@param[in] p_dest_it is iterator pointing to the first element of the result
 */
template <class CConstForwardIterator, class CForwardIterator>
void ExclusiveScan(CConstForwardIterator p_begin_it,
	CConstForwardIterator p_end_it, CForwardIterator p_dest_it)
{
	_ASSERTE(p_begin_it <= p_end_it);
	typename std::iterator_traits<CForwardIterator>::value_type n_sum = 0; // msvc6 would have to use _Val_type() hack to infer the value type (easy hack) // t_odo - figure out how to get that from CRandomIterator also in case it i
	for(; p_begin_it != p_end_it; ++ p_begin_it, ++ p_dest_it) {
		typename std::iterator_traits<CForwardIterator>::value_type n = *p_begin_it;
		*p_dest_it = n_sum;
		n_sum += n;
	}
}

/**
 *	@brief inclusive prefix sum function
 *
 *	@tparam CForwardIterator is forward iterator type
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sequence
 */
template <class CForwardIterator>
void InclusiveScan(CForwardIterator p_begin_it, CForwardIterator p_end_it)
{
	_ASSERTE(p_begin_it <= p_end_it);
	if(p_begin_it == p_end_it)
		return;
	typename std::iterator_traits<CForwardIterator>::value_type n_prev = *p_begin_it; // msvc6 would have to use _Val_type() hack to infer the value type (easy hack)
	for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it)
		n_prev = *p_begin_it += n_prev;
}

/**
 *	@brief inclusive prefix sum function
 *
 *	@tparam CConstForwardIterator is constant forward iterator type
 *	@tparam CForwardIterator is forward iterator type
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sequence
 *	@param[in] p_dest_it is iterator pointing to the first element of the result
 */
template <class CConstForwardIterator, class CForwardIterator>
void InclusiveScan(CConstForwardIterator p_begin_it,
	CConstForwardIterator p_end_it, CForwardIterator p_dest_it)
{
	_ASSERTE(p_begin_it <= p_end_it);
	if(p_begin_it == p_end_it)
		return;
	typename std::iterator_traits<CForwardIterator>::value_type n_prev = *p_begin_it; // msvc6 would have to use _Val_type() hack to infer the value type (easy hack)
	for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it, ++ p_dest_it)
		n_prev = *p_dest_it = *p_begin_it + n_prev;
}

/**
 *	@brief generates a sequence of increasing values
 *
 *	@tparam CForwardIterator is forward iterator type
 *	@tparam CValue is value type (must be convertible to type the iterators
 *		point to, and must be incrementable)
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sequence
 *	@param[in] t_first is value of the first element of the sequence
 */
template <class CForwardIterator, class CValue>
void IOTA(CForwardIterator p_begin_it, CForwardIterator p_end_it, CValue t_first)
{
	_ASSERTE(p_begin_it <= p_end_it);
	for(; p_begin_it != p_end_it; ++ p_begin_it, ++ t_first)
		*p_begin_it = t_first;
}

/**
 *	@brief generates a sequence of increasing values, starting at zero
 *
 *	@tparam CForwardIterator is forward iterator type
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sequence
 */
template <class CForwardIterator>
void IOTA(CForwardIterator p_begin_it, CForwardIterator p_end_it)
{
	_ASSERTE(p_begin_it <= p_end_it);
	typename std::iterator_traits<CForwardIterator>::value_type v = 0;
	for(; p_begin_it != p_end_it; ++ p_begin_it, ++ v)
		*p_begin_it = v;
}

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
 *	@brief comparison object for std::pair, comparing only the first member
 */
struct CCompareFirst {
	/**
	 *	@brief function operator; performs the comparison
	 *
	 *	@tparam CPairA is specialization of std::pair on the left
	 *	@tparam CPairB is specialization of std::pair on the right
	 *
	 *	@param[in] a is the first pair to compare
	 *	@param[in] b is the second pair to compare
	 *
	 *	@return Returns true if the first pair is ordered before the second pair.
	 */
	template <class CPairA, class CPairB>
	inline bool operator ()(const CPairA &a, const CPairB &b) const
	{
		return a.first < b.first;
	}
};

/**
 *	@brief comparison object for std::pair, comparing only the second member
 */
struct CCompareSecond {
	/**
	 *	@brief function operator; performs the comparison
	 *
	 *	@tparam CPairA is specialization of std::pair on the left
	 *	@tparam CPairB is specialization of std::pair on the right
	 *
	 *	@param[in] a is the first pair to compare
	 *	@param[in] b is the second pair to compare
	 *
	 *	@return Returns true if the first pair is ordered before the second pair.
	 */
	template <class CPairA, class CPairB>
	inline bool operator ()(const CPairA &a, const CPairB &b) const
	{
		return a.second < b.second;
	}
};

/**
 *	@brief comparison object for std::pair, implementing reverse lexicographical comparison
 */
struct CCompareSecond_then_First {
	/**
	 *	@brief function operator; performs the comparison
	 *
	 *	@tparam CPairA is specialization of std::pair on the left
	 *	@tparam CPairB is specialization of std::pair on the right
	 *
	 *	@param[in] a is the first pair to compare
	 *	@param[in] b is the second pair to compare
	 *
	 *	@return Returns true if the first pair is ordered before the second pair.
	 */
	template <class CPairA, class CPairB>
	inline bool operator ()(const CPairA &a, const CPairB &b) const
	{
		return a.second < b.second ||
			(a.second == b.second && a.first < b.first);
	}
};

/**
 *	@brief indirect comparison operator; compares values pointed to by indices
 *
 *	This can be used to sort several arrays at once, e.g.:
 *	@code
 *	std::vector<int> x, y, z; // some arrays to be sorted (of the same size)
 *	const size_t n = x.size(); // size of the arrays
 *	_ASSERTE(y.size() == n && z.size() == n);
 *
 *	std::vector<size_t> perm(n);
 *	stl_ut::IOTA(perm.begin(), perm.end(), size_t(0)); // make an identity permutation
 *	std::sort(perm.begin(), perm.end(), CCompare_Indirect<int>(x, n));
 *	// build permutation that sorts x
 *
 *	std::vector<int> xs(n), ys(n), zs(n);
 *	for(size_t i = 0; i < n; ++ i) {
 *		xs[perm[i]] = x[i];
 *		ys[perm[i]] = y[i];
 *		zs[perm[i]] = z[i];
 *	}
 *	// permute the arrays simultaneously, xs is now sorted
 *	@endcode
 *
 *	@tparam A is key type
 *	@tparam CCompA is comparison operator (by default std::less<A>)
 */
template <class A, class CCompA = std::less<A> >
class CCompare_Indirect {
protected:
	const A *m_p_key; /**< @brief pointer to the keys */
	CCompA m_comp_a; /**< @brief comparison operator instance */
#ifdef _DEBUG
	const size_t m_n_key_count; /**< @brief number of keys in the array (debug only) */
#endif // _DEBUG

public:
	/**
	 *	@brief default constructor; sets up the comparison operator
	 *
	 *	@param[in] p_key is pointer to the keys to be compared
	 *	@param[in] n_key_num is number of keys in the array (for debugging)
	 *	@param[in] comp_a is instance of the comparison operator
	 */
	CCompare_Indirect(const A *p_key, size_t UNUSED(n_key_num), CCompA comp_a = CCompA())
		:m_p_key(p_key), m_comp_a(comp_a)
#ifdef _DEBUG
		, m_n_key_count(n_key_num)
#endif // _DEBUG
	{}

	/**
	 *	@brief indirect comparison operator
	 *
	 *	@param[in] x is zero-based index of the first key to be compared
	 *	@param[in] y is zero-based index of the second key to be compared
	 *
	 *	@return Returns the result of comparison of the keys selected by the two indices.
	 */
	bool operator ()(size_t x, size_t y) const
	{
		_ASSERTE(x < m_n_key_count && y < m_n_key_count);
		return m_comp_a(m_p_key[x], m_p_key[y]);
	}
};

/**
 *	@brief indirect comparison operator; compares values pointed to by indices
 *
 *	@tparam A is primary key type
 *	@tparam B is secondary key type
 *	@tparam CCompA is primary kay comparison operator (by default std::less<A>)
 *	@tparam CCompB is secondary key comparison operator (by default std::less<B>)
 */
template <class A, class B, class CCompA = std::less<A>, class CCompB = std::less<B> >
class CComparePair_Indirect {
protected:
	const A *m_p_key_a; /**< @brief pointer to the primary keys */
	const B *m_p_key_b; /**< @brief pointer to the secondary keys */
	CCompA m_comp_a; /**< @brief primary comparison operator instance */
	CCompB m_comp_b; /**< @brief secondary comparison operator instance */
#ifdef _DEBUG
	const size_t m_n_key_count; /**< @brief number of keys in the array (debug only) */
#endif // _DEBUG

public:
	/**
	 *	@brief default constructor; sets up the comparison operator
	 *
	 *	@param[in] p_key_a is pointer to the primary keys to be compared
	 *	@param[in] p_key_b is pointer to the secondary keys to be compared
	 *	@param[in] n_key_num is number of keys in the arrays (for debugging)
	 *	@param[in] comp_a is instance of the primary key comparison operator
	 *	@param[in] comp_b is instance of the secondary key comparison operator
	 */
	CComparePair_Indirect(const A *p_key_a, const B *p_key_b, size_t UNUSED(n_key_num),
		CCompA comp_a = CCompA(), CCompB comp_b = CCompB())
		:m_p_key_a(p_key_a), m_p_key_b(p_key_b), m_comp_a(comp_a), m_comp_b(comp_b)
#ifdef _DEBUG
		, m_n_key_count(n_key_num)
#endif // _DEBUG
	{}

	/**
	 *	@brief indirect comparison operator
	 *
	 *	@param[in] x is zero-based index of the first key pair to be compared
	 *	@param[in] y is zero-based index of the second key pair to be compared
	 *
	 *	@return Returns the result of comparison of the keys selected by the two indices.
	 */
	bool operator ()(size_t x, size_t y) const
	{
		_ASSERTE(x < m_n_key_count && y < m_n_key_count);
		return m_comp_a(m_p_key_a[x], m_p_key_a[y]) ||
			(!m_comp_a(m_p_key_a[y], m_p_key_a[x]) && m_comp_b(m_p_key_b[x], m_p_key_b[y]));
	}
};

/**
 *	@brief indirect comparison operator; compares values pointed to by indices
 *
 *	@tparam A is primary key type
 *	@tparam B is secondary key type
 *	@tparam C is tertiary key type
 *	@tparam CCompA is primary kay comparison operator (by default std::less<A>)
 *	@tparam CCompB is secondary key comparison operator (by default std::less<B>)
 *	@tparam CCompC is tertiary key comparison operator (by default std::less<C>)
 */
template <class A, class B, class C, class CCompA = std::less<A>,
	class CCompB = std::less<B>, class CCompC = std::less<C> >
class CCompareTriplet_Indirect {
protected:
	const A *m_p_key_a; /**< @brief pointer to the primary keys */
	const B *m_p_key_b; /**< @brief pointer to the secondary keys */
	const C *m_p_key_c; /**< @brief pointer to the tertiary keys */
	CCompA m_comp_a; /**< @brief primary comparison operator instance */
	CCompB m_comp_b; /**< @brief secondary comparison operator instance */
	CCompC m_comp_c; /**< @brief tertiary comparison operator instance */
#ifdef _DEBUG
	const size_t m_n_key_count; /**< @brief number of keys in the array (debug only) */
#endif // _DEBUG

public:
	/**
	 *	@brief default constructor; sets up the comparison operator
	 *
	 *	@param[in] p_key_a is pointer to the primary keys to be compared
	 *	@param[in] p_key_b is pointer to the secondary keys to be compared
	 *	@param[in] p_key_c is pointer to the tertiary keys to be compared
	 *	@param[in] n_key_num is number of keys in the arrays (for debugging)
	 *	@param[in] comp_a is instance of the primary key comparison operator
	 *	@param[in] comp_b is instance of the secondary key comparison operator
	 *	@param[in] comp_c is instance of the tertiary key comparison operator
	 */
	CCompareTriplet_Indirect(const A *p_key_a, const B *p_key_b,
		const C *p_key_c, size_t UNUSED(n_key_num), CCompA comp_a = CCompA(),
		CCompB comp_b = CCompB(), CCompC comp_c = CCompC())
		:m_p_key_a(p_key_a), m_p_key_b(p_key_b), m_p_key_c(p_key_c),
		m_comp_a(comp_a), m_comp_b(comp_b), m_comp_c(comp_c)
#ifdef _DEBUG
		, m_n_key_count(n_key_num)
#endif // _DEBUG
	{}

	/**
	 *	@brief indirect comparison operator
	 *
	 *	@param[in] x is zero-based index of the first key triplet to be compared
	 *	@param[in] y is zero-based index of the second key triplet to be compared
	 *
	 *	@return Returns the result of comparison of the keys selected by the two indices.
	 */
	bool operator ()(size_t x, size_t y) const
	{
		_ASSERTE(x < m_n_key_count && y < m_n_key_count);
		return m_comp_a(m_p_key_a[x], m_p_key_a[y]) || (!m_comp_a(m_p_key_a[y], m_p_key_a[x]) && 
			(m_comp_b(m_p_key_b[x], m_p_key_b[y]) || (!m_comp_b(m_p_key_b[y], m_p_key_b[x]) &&
			m_comp_c(m_p_key_c[x], m_p_key_c[y]))));
	}
};

/**
 *	@brief searches for an item in a sorted container
 *
 *	@tparam CIterator is iterator type
 *	@tparam CValue is needle value type
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sorted sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sorted sequence
 *	@param[in] r_value is value to be searched for
 *
 *	@return Returns pair of found flag and iterator. In case the item was found, first equals
 *		true and second points to element storing the item. Otherwise, first equals false and
 *		second equals p_end_it.
 */
template <class CIterator, class CValue>
std::pair<bool, CIterator> t_Sorted_Find(CIterator p_begin_it, CIterator p_end_it, const CValue &r_value)
{
	CIterator p_it = std::lower_bound(p_begin_it, p_end_it, r_value);
	if(p_it == p_end_it)
		return std::make_pair(false, p_end_it);
	else
		return std::make_pair(bool(*p_it == r_value), p_it);
}

/**
 *	@brief searches for an item in a sorted container
 *
 *	@tparam CIterator is iterator type
 *	@tparam CValue is needle value type
 *	@tparam CLessThanComparison is less than comparison predicate
 *
 *	@param[in] p_begin_it is iterator pointing to the first element of the sorted sequence
 *	@param[in] p_end_it is iterator pointing to one past the last element of the sorted sequence
 *	@param[in] r_value is value to be searched for
 *	@param[in] comparator is predicate instance
 *
 *	@return Returns pair of found flag and iterator. In case the item was found, first equals
 *		true and second points to element storing the item. Otherwise, first equals false and
 *		second equals p_end_it.
 */
template <class CIterator, class CValue, class CLessThanComparison>
std::pair<bool, CIterator> t_Sorted_Find(CIterator p_begin_it, CIterator p_end_it,
	const CValue &r_value, CLessThanComparison comparator)
{
	CIterator p_it = std::lower_bound(p_begin_it, p_end_it, r_value, comparator);
	if(p_it == p_end_it)
		return std::make_pair(false, p_end_it);
	else
		return std::make_pair(!comparator(*p_it, r_value) && !comparator(r_value, *p_it), p_it); // note that this uses the comparator to check for equality
}

/**
 *	@brief inserts an item in a sorted container, without making duplicates
 *
 *	@tparam CContainer is container type (e.g. a specialization of std::vector)
 *	@tparam CValue is needle value type
 *
 *	@param[in,out] r_container is reference to the container (must be sorted)
 *	@param[in] r_value is value to be inserted
 *
 *	@return Returns pair of inserted flag and iterator. In case the item was found, it was not
 *		inserted and first equals false. In case the item was not found, it was inserted and
 *		first equals true. Second always contains iterator to an element with the value.
 */
template <class CContainer, class CValue>
std::pair<bool, typename CContainer::iterator> t_Unique_Insert(CContainer &r_container, const CValue &r_value) // throw(std::bad_alloc)
{
	typename CContainer::iterator p_it = std::lower_bound(r_container.begin(), r_container.end(), r_value);
	if(p_it == r_container.end() || *p_it != r_value)
		return std::make_pair(true, r_container.insert(p_it, r_value)); // return where it was inserted
	else
		return std::make_pair(false, p_it); // return where it is
}

/**
 *	@brief inserts an item in a sorted container, without making duplicates
 *
 *	@tparam CContainer is container type (e.g. a specialization of std::vector)
 *	@tparam CValue is needle value type
 *	@tparam CLessThanComparison is less than comparison predicate
 *
 *	@param[in,out] r_container is reference to the container (must be sorted)
 *	@param[in] r_value is value to be inserted
 *	@param[in] comparator is predicate instance
 *
 *	@return Returns pair of inserted flag and iterator. In case the item was found, it was not
 *		inserted and first equals false. In case the item was not found, it was inserted and
 *		first equals true. Second always contains iterator to an element with the value.
 */
template <class CContainer, class CValue, class CLessThanComparison>
std::pair<bool, typename CContainer::iterator> t_Unique_Insert(CContainer &r_container,
	const CValue &r_value, CLessThanComparison comparator) // throw(std::bad_alloc)
{
	typename CContainer::iterator p_it = std::lower_bound(r_container.begin(), r_container.end(), r_value, comparator);
	if(p_it == r_container.end() || comparator(*p_it, r_value) || comparator(r_value, *p_it))// note that this uses the comparator to check for equality
		return std::make_pair(true, r_container.insert(p_it, r_value)); // return where it was inserted
	else
		return std::make_pair(false, p_it); // return where it is
}

/**
 *	@brief determines whether a set of elements is sorted, permits repeated elements
 *
 *	@tparam _TyConstIter is vertex index const iterator type
 *
 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
 *
 *	@return Returns true if the set is sorted in ascending
 *		order, otherwise returns false.
 */
template <class _TyConstIter, class _TyComparator>
static bool b_IsWeaklySorted(_TyConstIter p_begin_it, _TyConstIter p_end_it, _TyComparator comp)
{
	//typedef std::iterator_traits<_TyConstIter>::value_type T;
	// unused, dont want to copy the objects in case they are large

	_ASSERTE(p_end_it >= p_begin_it);
	if(p_begin_it == p_end_it)
		return true;
	// empty set is sorted

	_TyConstIter p_prev_it = p_begin_it;
	for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it) {
		if(comp(*p_begin_it, *p_prev_it))
			return false; // not sorted, or contains repeating elements
		p_prev_it = p_begin_it;
	}
	return true;
}

template <class V0, class V1>
class CRefPair; // forward declaration

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200

/**
 *	@brief namespace with msvc 6.0 specific workarrounds
 */
namespace msvc6 {

/**
 *	@brief pair of values
 *
 *	msvc6 is unable to convert CRefPair to std::pair, saying the conversion is ambiguous,
 *	so this inherits from std::pair and adds a contructor taking CRefPair, which fixes
 *	the problems. The std::pair constructor with type conversion is likely the culprit.
 *
 *	@tparam V0 is the first value type
 *	@tparam V1 is the second value type
 */
template <class V0, class V1>
class pair_ : public ::std::pair<V0, V1> {
public:
	typedef ::std::pair<V0, V1> _TyPair; /**< @brief the std::pair type */

public:
	/**
	 *	@brief default constructor; initialize the values with defaults
	 */
	pair_()
		:_TyPair()
	{}

	/**
	 *	@brief default constructor; initialize the values with defaults
	 */
	pair_(const V0 &_V1, const V1 &_V2)
		:_TyPair(_V1, _V2)
	{}

	/**
	 *	@brief copy-constructor, with possible type conversion
	 *
	 *	@iparam U is the first value type of the pair to convert from
	 *	@iparam V is the second value type of the pair to convert from
	 *
	 *	@param[in] p is another pair
	 */
	template <class U, class V>
	pair_(const pair_<U, V> &p)
		:_TyPair(p)
	{}

	/**
	 *	@brief constructor from reference pair
	 *	@param[in] p is a reference pair
	 */
	inline pair_(const CRefPair<V0, V1> &p);
};

}; // ~msvc6

#else // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

/**
 *	@brief pair ref chaining constructs
 */
namespace pr_chain {

template <class X>
class CMakeRef {
public:
	typedef X& _TyResult;
};

template <class V0, class V1>
class CMakeRef<CRefPair<V0, V1> > {
public:
	typedef CRefPair<V0, V1> _TyResult;
};

template <class X>
class CMakeValue {
public:
	typedef X _TyResult;
};

template <class V0, class V1>
class CMakeValue<CRefPair<V0, V1> > {
public:
	typedef std::pair<V0, V1> _TyResult;
};

template <class X>
class CStripRef;

template <class X>
class CStripRef<X&> {
public:
	typedef X _TyResult;
};

template <class V0, class V1>
class CStripRef<CRefPair<V0, V1> > {
public:
	typedef CRefPair<V0, V1> _TyResult;
};

} // ~pr_chain

#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

/**
 *	@brief a pair of references; overrides copy semantics of std::pair
 *
 *	@tparam V0 is the first value type (not a reference)
 *	@tparam V1 is the second value type (not a reference)
 */
template <class V0, class V1>
class CRefPair {
public:
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200
	typedef V0 _TyValue0; /**< the first value type */
	typedef V1 _TyValue1; /**< the second value type */
	typedef msvc6::pair_<V0, V1> _TyValuePair; /**< pair of the values */
	typedef V0 &_TyReference0; /**< the first value type */
	typedef V1 &_TyReference1; /**< the second value type */
#else // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200
	typedef typename pr_chain::CMakeValue<V0>::_TyResult _TyValue0; /**< the first value type */
	typedef typename pr_chain::CMakeValue<V1>::_TyResult _TyValue1; /**< the second value type */
	typedef std::pair<_TyValue0, _TyValue1> _TyValuePair; /**< pair of the values */
	typedef typename pr_chain::CMakeRef<V0>::_TyResult _TyReference0; /**< the first value type */
	typedef typename pr_chain::CMakeRef<V1>::_TyResult _TyReference1; /**< the second value type */
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

	typedef _TyValue0 first_type; /**< the first value type alias */
	typedef _TyValue1 second_type; /**< the second value type alias */

public:
    _TyReference0 first; /**< reference to the first value */
    _TyReference1 second; /**< reference to the second value */

public:
	/**
	 *	@brief default constructor; binds two values to this reference pair
	 *
	 *	@param[in] r_v0 is reference to the first value
	 *	@param[in] r_v1 is reference to the second value
	 */
    CRefPair(_TyReference0 r_v0, _TyReference1 r_v1)
        :first(r_v0), second(r_v1)
    {
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200
		CValidate::MSVC60_DOESNT_SUPPORT_REF_PAIR_CHAINING(r_v0);
		CValidate::MSVC60_DOESNT_SUPPORT_REF_PAIR_CHAINING(r_v1);
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200
	}

	/**
	 *	@brief swaps values pointed to by the two reference pairs
	 *	@param[in,out] r_other is the other reference pair to swap with
	 */
    void Swap(CRefPair &r_other)
    {
        std::swap(first, r_other.first);
        std::swap(second, r_other.second);
    }

	/**
	 *	@brief converts this reference pair to a value pair
	 *	@return Returns a pair of the copy of the values pointed to by this reference pair.
	 */
    operator _TyValuePair() const // both g++ and msvc sort requires this (to get a pivot)
    {
        return _TyValuePair((_TyValue0)first, (_TyValue1)second);
    }

	/**
	 *	@brief assigns a value pair
	 *	@param[in] v is pair of values to be assigned to the place pointed to by this reference pair
	 *	@return Returns reference to this.
	 */
    CRefPair &operator =(_TyValuePair v) // both g++ and msvc sort requires this (for insertion sort)
    {
        first = v.first;
        second = v.second;
        return *this;
    }

	/**
	 *	@brief assigns values for a reference pair
	 *	@param[in] v is pair of values to be assigned to the place pointed to by this reference pair
	 *	@return Returns reference to this.
	 */
    CRefPair &operator =(const CRefPair &r_other) // required by g++ (for _GLIBCXX_MOVE)
    {
        first = r_other.first;
        second = r_other.second;
        return *this;
    }

	bool operator <(const CRefPair &r_other) const
	{
		return (_TyValuePair)(*this) < (_TyValuePair)r_other; // use default operator
	}

protected:
	class CValidate {
	private:
		template <class A, class B>
		static void MSVC60_DOESNT_SUPPORT_REF_PAIR_CHAINING(CRefPair<A, B> pair);

	public:
		template <class X>
		static void MSVC60_DOESNT_SUPPORT_REF_PAIR_CHAINING(const X &UNUSED(r_scalar))
		{}
	};
};

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200

template <class V0, class V1>
inline msvc6::pair_<V0, V1>::pair_(const CRefPair<V0, V1> &p)
	:_TyPair(p.first, p.second)
{}

#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

/**
 *	@brief a simple zip iterator
 *
 *	You can do e.g.:
 *	@code
 *	int pa[] = {1, 2, 4, 4, 2};
 *	int pb[] = {4, 3, 2, 1, 0};
 *	std::vector<int> a(pa, pa + 5), b(pb, pb + 5);
 *
 *	for(int i = 0, n = a.size(); i < n; ++ i)
 *		printf("%d ", a[i]);
 *	printf("\n");
 *	for(int i = 0, n = b.size(); i < n; ++ i)
 *		printf("%d ", b[i]);
 *	printf("\n");
 *
 *	printf("sort\n");
 *
 *	std::sort(stl_ut::p_Make_PairIterator(a.begin(), b.begin()),
 *		stl_ut::p_Make_PairIterator(a.end(), b.end()));
 *
 *	for(int i = 0, n = a.size(); i < n; ++ i)
 *		printf("%d ", a[i]);
 *	printf("\n");
 *	for(int i = 0, n = b.size(); i < n; ++ i)
 *		printf("%d ", b[i]);
 *	printf("\n");
 *	@endcode
 *
 *	@tparam[in] It0 is the first iterator type
 *	@tparam[in] It1 is the second iterator type
 */
template <class It0, class It1,
#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER > 1200
	class Val0 = typename std::iterator_traits<It0>::value_type,
	class Val1 = typename std::iterator_traits<It1>::value_type,
	class Ref0 = typename std::iterator_traits<It0>::reference,
	class Ref1 = typename std::iterator_traits<It1>::reference>
#else // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200
	class Val0, class Val1, class Ref0 = Val0&, class Ref1 = Val1&>
#endif // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200
class CPairIterator : public std::random_access_iterator_tag {
public:
    typedef Val0 value_type0;
    typedef Val1 value_type1;
#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER > 1200
	typedef CRefPair<typename pr_chain::CStripRef<Ref0>::_TyResult,
		typename pr_chain::CStripRef<Ref1>::_TyResult> reference;
#else
    typedef CRefPair<value_type0, value_type1> reference;
#endif
    typedef typename reference::_TyValuePair value_type;
    typedef reference *pointer; // not so sure about this, probably can't be implemented in a meaningful way, won't be able to overload ->

#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER > 1200
    typedef typename std::iterator_traits<It0>::difference_type difference_type;
    typedef /*typename std::iterator_traits<It0>::distance_type*/difference_type distance_type; // no distance_type in g++, only in msvc
    typedef typename std::iterator_traits<It0>::iterator_category iterator_category;
#else // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200
	typedef ptrdiff_t difference_type;
    typedef ptrdiff_t distance_type;
#endif // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200
    // keep the iterator traits happy

protected:
    It0 m_it0; /**< @brief the first iterator value */
    It1 m_it1; /**< @brief the second iterator value */

public:
    CPairIterator(const CPairIterator &r_other)
        :m_it0(r_other.m_it0), m_it1(r_other.m_it1)
    {}

    CPairIterator(It0 it0 = It0(), It1 it1 = It1())
        :m_it0(it0), m_it1(it1)
    {}

    reference operator *()
    {
        return reference(*m_it0, *m_it1); // explicit cast needed to correctly support chaining
    }

    value_type operator *() const
    {
        return value_type(*const_cast<const It0&>(m_it0), *const_cast<const It1&>(m_it1)); // cast to make sure we get the right thing
    }

    difference_type operator -(const CPairIterator &other) const
    {
        _ASSERTE(m_it0 - other.m_it0 == m_it1 - other.m_it1);
        // the iterators always need to have the same position
        // (incomplete check but the best we can do without having also begin / end in either vector)

        return m_it0 - other.m_it0;
    }

    bool operator ==(const CPairIterator &other) const
    {
        _ASSERTE(m_it0 - other.m_it0 == m_it1 - other.m_it1);
        return m_it0 == other.m_it0;
    }

    bool operator !=(const CPairIterator &other) const
    {
        return !(*this == other);
    }

    bool operator <(const CPairIterator &other) const
    {
        _ASSERTE(m_it0 - other.m_it0 == m_it1 - other.m_it1);
        return m_it0 < other.m_it0;
    }

    bool operator >=(const CPairIterator &other) const
    {
        return !(*this < other);
    }

    bool operator <=(const CPairIterator &other) const
    {
        return !(other < *this);
    }

    bool operator >(const CPairIterator &other) const
    {
        return other < *this;
    }

    CPairIterator operator +(distance_type d) const
    {
        return CPairIterator(m_it0 + d, m_it1 + d);
    }

    CPairIterator operator -(distance_type d) const
    {
        return *this + -d;
    }

    CPairIterator &operator +=(distance_type d)
    {
        return *this = *this + d;
    }

    CPairIterator &operator -=(distance_type d)
    {
        return *this = *this + -d;
    }

    CPairIterator &operator ++()
    {
        return *this += 1;
    }

    CPairIterator &operator --()
    {
        return *this += -1;
    }

    CPairIterator operator ++(int) // msvc sort actually needs this, g++ does not
    {
        CPairIterator old = *this;
        ++ (*this);
        return old;
    }

    CPairIterator operator --(int)
    {
        CPairIterator old = *this;
        -- (*this);
        return old;
    }
};

#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER > 1200

/**
 *	@brief make a pair iterator
 *
 *	@tparam It0 is type of the first iterator to form a pair
 *	@tparam It1 is type of the second iterator to form a pair
 *
 *	@param[in] it0 is value of the first iterator to form a pair
 *	@param[in] it1 is value of the second iterator to form a pair
 *
 *	@return Returns a pair iterator.
 */
template <class It0, class It1>
inline CPairIterator<It0, It1> p_Make_PairIterator(It0 it0, It1 it1)
{
    return CPairIterator<It0, It1>(it0, it1);
}

#else // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200

/**
 *	@brief make a pair iterator, version for msvc60
 *
 *	@tparam It0 is type of the first iterator to form a pair
 *	@tparam It1 is type of the second iterator to form a pair
 *	@tparam V0 is type of the value the first iterator points to
 *	@tparam V1 is type of the value the second iterator points to
 *
 *	@param[in] it0 is value of the first iterator to form a pair
 *	@param[in] it1 is value of the second iterator to form a pair
 *	@param[in] v0 is dummy type inference pointer
 *	@param[in] v1 is dummy type inference pointer
 *
 *	@return Returns a pair iterator.
 */
template <class It0, class It1, class V0, class V1>
inline CPairIterator<It0, It1, V0, V1, V0&, V1&> t_Make_PairIterator_msvc6(It0 it0, It1 it1, V0 *UNUSED(v0), V1 *UNUSED(v1))
{
    return CPairIterator<It0, It1, V0, V1, V0&, V1&>(it0, it1);
}

/**
 *	@def p_Make_PairIterator
 *	@brief make a pair iterator
 *
 *	@tparam It0 is type of the first iterator to form a pair
 *	@tparam It1 is type of the second iterator to form a pair
 *
 *	@param[in] it0 is value of the first iterator to form a pair
 *	@param[in] it1 is value of the second iterator to form a pair
 *
 *	@return Returns a pair iterator.
 */
#define p_Make_PairIterator(it0,it1) t_Make_PairIterator_msvc6(it0, it1, std::_Val_type(it0), std::_Val_type(it1))

#endif // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200

/**
 *	@brief determines whether a set of elements is sorted and does not contain duplicate elements
 *
 *	@tparam _TyConstIter is vertex index const iterator type
 *	@tparam _TyComparator is comparator object
 *
 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
 *
 *	@return Returns true if the set is sorted in ascending
 *		order and does not contain duplicate elements, otherwise returns false.
 */
template <class _TyConstIter, class _TyComparator>
static bool b_IsStrictlySorted(_TyConstIter p_begin_it, _TyConstIter p_end_it, _TyComparator comp)
{
	//typedef std::iterator_traits<_TyConstIter>::value_type T;
	// unused, dont want to copy the objects in case they are large

	_ASSERTE(p_end_it >= p_begin_it);
	if(p_begin_it == p_end_it)
		return true;
	// empty set is sorted

	_TyConstIter p_prev_it = p_begin_it;
	for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it) {
		if(!comp(*p_prev_it, *p_begin_it))
			return false; // not sorted, or contains repeating elements
		p_prev_it = p_begin_it;
	}
	return true;
}

/**
 *	@brief determines whether a set of elements is sorted, permits repeated elements
 *
 *	@tparam _TyConstIter is vertex index const iterator type
 *
 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
 *
 *	@return Returns true if the set is sorted in ascending
 *		order, otherwise returns false.
 */
template <class _TyConstIter>
static bool b_IsWeaklySorted(_TyConstIter p_begin_it, _TyConstIter p_end_it)
{
	return b_IsWeaklySorted(p_begin_it, p_end_it,
		std::less<typename std::iterator_traits<_TyConstIter>::value_type>()); // msvc6 would have to use _Val_type() hack to infer the value type (easy hack)
}

/**
 *	@brief determines whether a set of elements is sorted and does not contain duplicate elements
 *
 *	@tparam _TyConstIter is vertex index const iterator type
 *
 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
 *
 *	@return Returns true if the set is sorted in ascending
 *		order and does not contain duplicate elements, otherwise returns false.
 */
template <class _TyConstIter>
static bool b_IsStrictlySorted(_TyConstIter p_begin_it, _TyConstIter p_end_it)
{
	return b_IsStrictlySorted(p_begin_it, p_end_it,
		std::less<typename std::iterator_traits<_TyConstIter>::value_type>()); // msvc6 would have to use _Val_type() hack to infer the value type (easy hack)
}

} // ~stl_ut

namespace std {

/**
 *	@brief swaps two unique pointers
 *
 *	@param[in,out] r_a is the first pointer to swap
 *	@param[in,out] r_b is the second pointer to swap
 */
template <class TType, class TDestructor>
inline void swap(stl_ut::CUniquePtr<TType, TDestructor> &r_a,
	stl_ut::CUniquePtr<TType, TDestructor> &r_b)
{
	r_a.Swap(r_b);
}

/**
 *	@brief swaps two reference pairs
 *
 *	@param[in,out] r_a is the first reference pair to swap
 *	@param[in,out] r_b is the second reference pair to swap
 */
template <class V0, class V1>
inline void swap(stl_ut::CRefPair<V0, V1> &a, stl_ut::CRefPair<V0, V1> &b)
{
    a.Swap(b);
}

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200

// msvc6 workarround of absence of iterator_traits and so of value_type
template <class I0, class I1, class V0, class V1, class R0, class R1>
typename stl_ut::CPairIterator<I0, I1, V0, V1, R0, R1>::value_type *_Val_type(stl_ut::CPairIterator<I0, I1, V0, V1, R0, R1> pair_it)
{
	return (typename stl_ut::CPairIterator<I0, I1, V0, V1, R0, R1>::value_type*)(0);
}

#endif // _MSC_VER && !__MWERKS__ && _MSC_VER <= 1200

template <class V0, class V1>
inline bool operator <(std::pair<V0, V1> a, stl_ut::CRefPair<V0, V1> b) // required by both g++ and msvc
{
	return a < MSVC6_OMMIT(typename) stl_ut::CRefPair<V0, V1>::_TyValuePair(b); // default pairwise lexicographical comparison
}

template <class V0, class V1>
inline bool operator <(stl_ut::CRefPair<V0, V1> a, std::pair<V0, V1> b) // required by both g++ and msvc
{
    return MSVC6_OMMIT(typename) stl_ut::CRefPair<V0, V1>::_TyValuePair(a) < b; // default pairwise lexicographical comparison
}

template <class V0, class V1>
inline bool operator <(stl_ut::CRefPair<V0, V1> a, stl_ut::CRefPair<V0, V1> b) // required by both g++ and msvc
{
    return MSVC6_OMMIT(typename) stl_ut::CRefPair<V0, V1>::_TyValuePair(a) <
		MSVC6_OMMIT(typename) stl_ut::CRefPair<V0, V1>::_TyValuePair(b); // default pairwise lexicographical comparison
}

} // ~std

#endif // !__STL_UTILS_INCLUDED
