/*
								+----------------------------------+
								|                                  |
								|  ***   Type name function   ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2017  |
								|                                  |
								|            TypeName.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __TYPE_NAME_INCLUDED
#define __TYPE_NAME_INCLUDED

/**
 *	@file include/slam/TypeName.h
 *	@brief type name retrieval
 *	@author -tHE SWINe-
 *	@date 2017
 */

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER <= 1200
#include <typeinfo>
#endif // _MSC_VER) && !__MWERKS__ && _MSC_VER <= 1200
#include <string.h>
#include <string>
// for s_TypeName()

/**
 *	@brief namespace with internal type name implementation and helper objects
 */
namespace tn_detail {

/**
 *	@brief gets user readable (demangled) type name
 *
 *	This is using the __PRETTY_FUNCTION__ hack and requires some string processing.
 *	Rather than writing some const string class, this is simply using std::string
 *	with a bit larger runtime cost. It's ok here, it is only intended for debugging.
 *
 *	This is needed as a separate function returning <tt>const char*</tt> rather
 *	than as ::s_TypeName() for some versions of g++ which otherwise also describe
 *	the type of std::string although it is not a template parameter but a typedef.
 *
 *	@tparam T is type to get the name of
 *	@return Returns the type name of <tt>T</tt>, possibly with some trailing characters.
 *	@note It is preferred to use \ref ::s_TypeName().
 */
template <class T>
const char *p_s_PrettyFunName() // do not rename it, will break the code for MSVC
{
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER <= 1200
	return typeid(T).name(); // returns mangled names in newer compilers!
#else // _MSC_VER <= 1200
	const char *p = __FUNCSIG__;
	return strstr(p, "p_s_PrettyFunName<") + 18/*strlen("p_s_PrettyFunName<")*/;
#endif // _MSC_VER <= 1200
#else // _MSC_VER) && !__MWERKS__
	const char *p = __PRETTY_FUNCTION__, *w;
	if((w = strstr(p, "T = ")))
		return w + 4;
	else if((w = strstr(p, "= ")))
		return w + 2;
	else if((w = strstr(p, "=")))
		return w + 1;
	else
		return p; // not sure how to trim it, return the whole thing
#endif // _MSC_VER) && !__MWERKS__
}

} // ~tn_detail

/**
 *	@brief gets user readable (demangled) type name
 *
 *	This is using the __PRETTY_FUNCTION__ hack and requires some string processing.
 *	Rather than writing some const string class, this is simply using std::string
 *	with a bit larger runtime cost. It's ok here, as it is only intended for debugging.
 *
 *	@tparam T is type to get the name of
 *	@return Returns the type name of <tt>T</tt>.
 *	@note This function throws std::bad_alloc.
 */
template <class T>
std::string s_TypeName() // throw(std::bad_alloc)
{
	const char *p = tn_detail::p_s_PrettyFunName<T>();
#if defined(_MSC_VER) && !defined(__MWERKS__)
#if _MSC_VER <= 1200
	return p; // done
#else // _MSC_VER <= 1200
	std::string str = p;
	str.erase(str.find_last_of(">"), std::string::npos);
	return str;
#endif // _MSC_VER <= 1200
#else // _MSC_VER) && !__MWERKS__
	std::string str = p;
	str.erase(str.end() - 1);
	return str;
#endif // _MSC_VER) && !__MWERKS__
}

#endif // !__TYPE_NAME_INCLUDED
