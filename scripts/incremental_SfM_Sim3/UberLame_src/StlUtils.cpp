/*
								+----------------------------------+
								|                                  |
								|     ***  STL utilities  ***      |
								|                                  |
								|   Copyright © -tHE SWINe- 2008   |
								|                                  |
								|           StlUtils.cpp           |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file StlUtils.cpp
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
 */

#include "NewFix.h"

#include "CallStack.h"
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif // _WIN32 || _WIN64
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include "StlUtils.h"
#include "Integer.h"

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/*
 *								=== STL Utilities ===
 */

namespace stl_ut {

bool Format(std::string &r_s_result, const char *p_s_fmt, ...)
{
	va_list ap;

	size_t n_size = strlen(p_s_fmt) + 1;
	if(!Resize_To_N(r_s_result, n_size))
		return false;
	// alloc str as long as formatting str (should be enough in some cases)

	for(;;) {
		va_start(ap, p_s_fmt);
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER >= 1400
		int n = _vsnprintf_s(&r_s_result[0], n_size * sizeof(char), _TRUNCATE, p_s_fmt, ap);
#else // _MSC_VER >= 1400
		int n = _vsnprintf(&r_s_result[0], n_size - 1, p_s_fmt, ap);
		// maximum characters to write, not the size of buffer
#endif // _MSC_VER >= 1400
#else // _MSC_VER && !__MWERKS__
		int n = vsnprintf(&r_s_result[0], n_size * sizeof(char), p_s_fmt, ap);
#endif // _MSC_VER && !__MWERKS__
		va_end(ap);
		// try to sprintf

		if(n >= 0 && unsigned(n) < n_size) {
#if defined(_MSC_VER) && !defined(__MWERKS__)
			Resize_To_N(r_s_result, n);
#else // _MSC_VER && !__MWERKS__
			Resize_To_N(r_s_result, strlen(r_s_result.data())); // doesn't need c_str(), terminating zero is already there
#endif // _MSC_VER && !__MWERKS__
			_ASSERTE(r_s_result.length() == strlen(r_s_result.data()));
			return true;
		}
		// see if we made it

		if(n >= 0) // glibc 2.1
			n_size = n + 1; // precisely what is needed
		else // glibc 2.0, msvc
			n_size *= 2;

		if(!Resize_To_N(r_s_result, n_size))
			return false;
		// realloc to try again
	}
}

bool Format(char *p_s_result, size_t n_buffer_size, const char *p_s_fmt, ...)
{
	_ASSERTE(n_buffer_size / sizeof(char) > 0);

	if(!(n_buffer_size / sizeof(char)))
		return false;
	p_s_result[0] = 0; // in case the sprintf fails and writes nothing

	va_list ap;
	va_start(ap, p_s_fmt);
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER >= 1400
	int n = _vsnprintf_s(p_s_result, n_buffer_size, _TRUNCATE, p_s_fmt, ap);
#else // _MSC_VER >= 1400
	int n = _vsnprintf(p_s_result, n_buffer_size / sizeof(char) - 1, p_s_fmt, ap);
	// maximum characters to write, not the size of buffer
#endif // _MSC_VER >= 1400
#else // _MSC_VER && !__MWERKS__
	int n = vsnprintf(p_s_result, n_buffer_size, p_s_fmt, ap);
#endif // _MSC_VER && !__MWERKS__
	va_end(ap);
	// try to sprintf

	if(n >= 0 && unsigned(n) < n_buffer_size / sizeof(char)) {
		_ASSERTE(p_s_result[n] == 0);
		return true;
	}
	// see if we made it

	p_s_result[n_buffer_size / sizeof(char) - 1] = 0;
	// make sure it is null-terminated

	return false;
}

bool ReadLine(std::string &r_s_line, FILE *p_fr)
{
	r_s_line.erase();
	try {
		for(int c = fgetc(p_fr); c != '\n' && c != EOF; c = fgetc(p_fr)) {
			/*if(ferror(p_fr)) // if some other reading error happens, fgetc() also returns EOF
				return false;*/
			r_s_line += c; // some string implementations don't have push_back()
		}
	} catch(std::bad_alloc&) {
		return false;
	}
	// read line

	return !ferror(p_fr);
}

bool ReadFile(std::string &r_s_output, const char *p_s_filename)
{
	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	int64_t n_length64; // to work even in 32-bit builds
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(_fseeki64(p_fr, 0, SEEK_END) ||
	   (n_length64 = _ftelli64(p_fr)) < 0 ||
	   _fseeki64(p_fr, 0, SEEK_SET) ||
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(fseek(p_fr, 0, SEEK_END) ||
	   (n_length64 = (int64_t)ftell(p_fr)) < 0 ||
	   fseek(p_fr, 0, SEEK_SET) ||
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	   (uint64_t)n_length64 / sizeof(char) > r_s_output.max_size()) { // fail on >4GB files in 32-bit builds
		fclose(p_fr);
		return false;
	}
	size_t n_length = size_t(n_length64 / sizeof(char)); // division by sizeof(char) is no-op, just to be correct below
	if(n_length >= r_s_output.max_size() || !stl_ut::Resize_To_N(r_s_output, n_length + 1)) { // one more so that c_str() won't fail or double the buffer
		fclose(p_fr);
		return false;
	}
	if(n_length && fread(&r_s_output[0], sizeof(char), n_length, p_fr) != n_length) {
		fclose(p_fr);
		return false;
	}
	fclose(p_fr);
	return true;
}

void TrimSpace(std::string &r_s_string)
{
	size_t b = 0, e = r_s_string.length();
	while(e > 0 && isspace(uint8_t(r_s_string[e - 1])))
		-- e;
	while(b < e && isspace(uint8_t(r_s_string[b])))
		++ b;
	r_s_string.erase(e);
	r_s_string.erase(0, b);
}

bool Split(std::vector<std::string> &r_s_dest, const std::string &r_s_string,
	const char *p_s_separator, int n_thresh)
{
	r_s_dest.clear();
	const size_t n_separator_skip = strlen(p_s_separator);
	try {
		size_t n_pos = 0;
		size_t n_next_pos;
		while((n_next_pos = r_s_string.find(p_s_separator, n_pos)) != std::string::npos) {
			if(n_thresh < 0 || n_next_pos - n_pos > unsigned(n_thresh)) {
				r_s_dest.resize(r_s_dest.size() + 1);
				std::string &r_s_new = r_s_dest.back();
				r_s_new.insert(r_s_new.begin(), r_s_string.begin() + n_pos,
					r_s_string.begin() + n_next_pos);
			}
			n_pos = n_next_pos + n_separator_skip; // skip over the separator
		}
		if(n_thresh < 0 || r_s_string.length() - n_pos > unsigned(n_thresh)) {
			r_s_dest.resize(r_s_dest.size() + 1);
			std::string &r_s_new = r_s_dest.back();
			r_s_new.insert(r_s_new.begin(), r_s_string.begin() + n_pos,
				r_s_string.end());
		}
	} catch(std::bad_alloc&) {
		return false;
	}

	return true;
}

bool SplitByMultiple(std::vector<std::string> &r_s_dest, const std::string &r_s_string,
	const char *p_s_separator, int n_thresh)
{
	r_s_dest.clear();
	const size_t n_separator_skip = 1;
	try {
		size_t n_pos = 0;
		size_t n_next_pos;
		while((n_next_pos = r_s_string.find_first_of(p_s_separator, n_pos)) != std::string::npos) {
			if(n_thresh < 0 || n_next_pos - n_pos > unsigned(n_thresh)) {
				r_s_dest.resize(r_s_dest.size() + 1);
				std::string &r_s_new = r_s_dest.back();
				r_s_new.insert(r_s_new.begin(), r_s_string.begin() + n_pos,
					r_s_string.begin() + n_next_pos);
			}
			n_pos = n_next_pos + n_separator_skip; // skip over the separator
		}
		if(n_thresh < 0 || r_s_string.length() - n_pos > unsigned(n_thresh)) {
			r_s_dest.resize(r_s_dest.size() + 1);
			std::string &r_s_new = r_s_dest.back();
			r_s_new.insert(r_s_new.begin(), r_s_string.begin() + n_pos,
				r_s_string.end());
		}
	} catch(std::bad_alloc&) {
		return false;
	}

	return true;
}

#ifndef __STL_UTILS_NO_WIDE_STRINGS

/*
 *	bool stl_ut::FormatW(std::basic_string<wchar_t> &r_s_result, const wchar_t *p_s_fmt, ...)
 *		- unicode equivalent of standard "c" library sprintf function,
 *		  working with std::string output
 *		- returns true on success, false on failure (not enough memory)
 *		- note this is safe against output buffer overrun, but is
 *		  still susceptible to ill-formated format string (p_s_fmt)
 *		  and bad arguments (not enough of them / not corresponding
 *		  to tags in the format string). but as the first one being run-time
 *		  error and the latter one compile-time error, this brings
 *		  enought improvement.
 */
bool FormatW(std::basic_string<wchar_t> &r_s_result, const wchar_t *p_s_fmt, ...)
{
	va_list ap;

	size_t n_size = wcslen(p_s_fmt) + 1;
	if(!Resize_To_N(r_s_result, n_size))
		return false;
	// alloc str as long as formatting str (should be enough in some cases)

	for(;;) {
		va_start(ap, p_s_fmt);
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER >= 1400
		int n = _vsnwprintf_s(&r_s_result[0], n_size, _TRUNCATE, p_s_fmt, ap);
#else // _MSC_VER >= 1400
		int n = _vsnwprintf(&r_s_result[0], n_size / sizeof(wchar_t) - 1, p_s_fmt, ap);
		// maximum characters to write, not the size of buffer
#endif // _MSC_VER >= 1400
#else // _MSC_VER && !__MWERKS__
#if defined(_WIN32) || defined(_WIN64)
		int n = vsnwprintf(&r_s_result[0], n_size, p_s_fmt, ap);
#else // _WIN32 || _WIN64
		int n = vswprintf(&r_s_result[0], n_size, p_s_fmt, ap); // unix
#endif // _WIN32 || _WIN64
#endif // _MSC_VER && !__MWERKS__
		va_end(ap);
		// try to sprintf

		if(n >= 0 && unsigned(n) < n_size) {
#if defined(_MSC_VER) && !defined(__MWERKS__)
			Resize_To_N(r_s_result, n);
#else // _MSC_VER && !__MWERKS__
			Resize_To_N(r_s_result, wcslen(r_s_result.data())); // doesn't need c_str(), terminating zero is already there
#endif // _MSC_VER && !__MWERKS__
			_ASSERTE(r_s_result.length() == wcslen(r_s_result.data()));
			return true;
		}
		// see if we made it

		if(n >= 0) // glibc 2.1
			n_size = n + 1; // precisely what is needed
		else // glibc 2.0, msvc
			n_size *= 2;

		if(!Resize_To_N(r_s_result, n_size))
			return false;
		// realloc to try again
	}
}

bool FormatW(wchar_t *p_s_result, size_t n_buffer_size, const wchar_t *p_s_fmt, ...)
{
	_ASSERTE(n_buffer_size / sizeof(wchar_t) > 0);

	if(!(n_buffer_size / sizeof(wchar_t)))
		return false;
	p_s_result[0] = 0; // in case the sprintf fails and writes nothing

	va_list ap;
	va_start(ap, p_s_fmt);
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER >= 1400
	int n = _vsnwprintf_s(p_s_result, n_buffer_size, _TRUNCATE, p_s_fmt, ap);
#else // _MSC_VER >= 1400
	int n = _vsnwprintf(p_s_result, n_buffer_size / sizeof(wchar_t) - 1, p_s_fmt, ap);
	// maximum characters to write, not the size of buffer
#endif // _MSC_VER >= 1400
#else // _MSC_VER && !__MWERKS__
#if defined(_WIN32) || defined(_WIN64)
	int n = vsnwprintf(p_s_result, n_buffer_size, p_s_fmt, ap);
#else // _WIN32 || _WIN64
	int n = vswprintf(p_s_result, n_buffer_size, p_s_fmt, ap); // unix
#endif // _WIN32 || _WIN64
#endif // _MSC_VER && !__MWERKS__
	va_end(ap);
	// try to sprintf

	if(n >= 0 && unsigned(n) < n_buffer_size / sizeof(wchar_t)) {
		_ASSERTE(p_s_result[n] == 0);
		return true;
	}
	// see if we made it

	p_s_result[n_buffer_size / sizeof(wchar_t) - 1] = 0;
	// make sure it is null-terminated

	return false;
}

#endif // __STL_UTILS_NO_WIDE_STRINGS

}

/*
 *								=== ~STL Utilities ===
 */
