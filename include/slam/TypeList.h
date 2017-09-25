/*
								+----------------------------------+
								|                                  |
								|  ***   Typelist templates   ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2012  |
								|                                  |
								|            TypeList.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __TYPE_LIST_INCLUDED
#define __TYPE_LIST_INCLUDED

/**
 *	@file include/slam/TypeList.h
 *	@brief typelist template
 *	@author -tHE SWINe-
 *	@date 2012
 *
 *	@date 2012-09-09
 *
 *	Added missing recursion in CUniqueList (there was a bug, as a result, only
 *	the first element of the list could have been removed, which was missed in the
 *	test case).
 *
 *	@date 2013-02-07
 *
 *	Added runtime operations on typelists, such as CTypelistForEach,
 *	CTypelistSelect and CTypelistBFind.
 *
 *	@date 2013-07-19
 *
 *	Changed implementation of CSortTypelistRun, where the comparison order was swapped
 *	to detect pairs that violate ordering. The earlier version lead to infinite template
 *	recursion on sequences containing multiple elements of the same type.
 *
 */

/**
 *	@def __TYPE_LIST_OMIT_OPS
 *	@brief if defined, type list operation classes are omitted
 *	@note This forced in MSVC 60 or older as it does not support partial template specialization
 *		(causes C2989: "template class has already been defined as a non-template class").
 */
//#define __TYPE_LIST_OMIT_OPS

#if !defined(__TYPE_LIST_OMIT_OPS) && defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER < 1400
#define __TYPE_LIST_OMIT_OPS
#endif // !__TYPE_LIST_OMIT_OPS && _MSC_VER && !__MWERKS__ && _MSC_VER < 1400
// force __TYPE_LIST_OMIT_OPS for earlier than MSVC 2008

#include "Integer.h" // _ASSERTE(), UNUSED()
#include "TypeName.h"

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1900
#define TYPELIST_H_PUSHED_WARNING_STATE
#pragma warning(push)
#pragma warning(disable: 4503)
#endif // _MSC_VER) && !__MWERKS__ && _MSC_VER >= 1900
// get rid of VS 2015 warning C4503: '__LINE__Var': decorated name length exceeded, name was truncated

/**
 *	@brief namespace with internal typelists implementation and helper objects
 */
namespace tl_detail {

// currently empty; some objects were migrated to another ns; the ns itself was retained

} // ~tl_detail

/**
 *	@brief typelist template
 *
 *	@tparam CHead is head type
 *	@tparam CTail is tail type (another CTypelist or CTypelistEnd)
 *
 *	@note An empty typelist is represented by CTypelistEnd, not by CTypelist<CTypelistEnd, CTypelistEnd>.
 */
template <class CHead, class CTail>
class CTypelist {
public:
	typedef CHead _TyHead; /**< @brief head type */
	typedef CTail _TyTail; /**< @brief tail type (another CTypelist or CTypelistEnd) */
};

/**
 *	@brief typelist end marker type, also an empty typelist all by itself
 */
class CTypelistEnd {};

/**
 *	@def MakeTypelist1
 *	@brief makes a typelist
 *	@param[in] x is the item of typelist
 *	@return Returns declaration of a typelist containing the entered type.
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist1(x) CTypelist<x, CTypelistEnd>

/**
 *	@def MakeTypelist2
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist2(x,y) CTypelist<x, MakeTypelist1(y) >

/**
 *	@def MakeTypelist3
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *	@param[in] z is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist3(x,y,z) CTypelist<x, MakeTypelist2(y, z) >

/**
 *	@def MakeTypelist4
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *	@param[in] z is the item of typelist
 *	@param[in] w is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist4(x,y,z,w) CTypelist<x, MakeTypelist3(y, z, w) >

/**
 *	@def MakeTypelist5
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *	@param[in] z is the item of typelist
 *	@param[in] w is the item of typelist
 *	@param[in] s is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist5(x,y,z,w,s) CTypelist<x, MakeTypelist4(y, z, w, s) >

/**
 *	@def MakeTypelist6
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *	@param[in] z is the item of typelist
 *	@param[in] w is the item of typelist
 *	@param[in] s is the item of typelist
 *	@param[in] t is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist6(x,y,z,w,s,t) CTypelist<x, MakeTypelist5(y, z, w, s, t) >

/**
 *	@def MakeTypelist7
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *	@param[in] z is the item of typelist
 *	@param[in] w is the item of typelist
 *	@param[in] s is the item of typelist
 *	@param[in] t is the item of typelist
 *	@param[in] p is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist7(x,y,z,w,s,t,p) CTypelist<x, MakeTypelist6(y, z, w, s, t, p) >

/**
 *	@def MakeTypelist8
 *	@brief makes a typelist
 *
 *	@param[in] x is the item of typelist
 *	@param[in] y is the item of typelist
 *	@param[in] z is the item of typelist
 *	@param[in] w is the item of typelist
 *	@param[in] s is the item of typelist
 *	@param[in] t is the item of typelist
 *	@param[in] p is the item of typelist
 *	@param[in] q is the item of typelist
 *
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *
 *	@note This version is not safe in case any of the items on the typelist is a template with
 *		multiple parameters (the template syntax oblivious preprocesor will split the template
 *		parameters as additional types). Use MakeTypelist() or MakeTypelist_Safe() instead.
 */
#define MakeTypelist8(x,y,z,w,s,t,p,q) CTypelist<x, MakeTypelist7(y, z, w, s, t, p, q) >

/**
 *	@def MakeTypelist9
 *	@param[in] r is the item of typelist
 *	@copydoc MakeTypelist8
 */
#define MakeTypelist9(x,y,z,w,s,t,p,q,r) CTypelist<x, MakeTypelist8(y, z, w, s, t, p, q, r) >

/**
 *	@def MakeTypelist10
 *	@param[in] m is the item of typelist
 *	@copydoc MakeTypelist9
 */
#define MakeTypelist10(x,y,z,w,s,t,p,q,r,m) \
	CTypelist<x, MakeTypelist9(y, z, w, s, t, p, q, r, m) >

/**
 *	@def MakeTypelist11
 *	@param[in] n is the item of typelist
 *	@copydoc MakeTypelist10
 */
#define MakeTypelist11(x,y,z,w,s,t,p,q,r,m,n) \
	CTypelist<x, MakeTypelist10(y, z, w, s, t, p, q, r, m, n) >

/**
 *	@def MakeTypelist12
 *	@param[in] u is the item of typelist
 *	@copydoc MakeTypelist11
 */
#define MakeTypelist12(x,y,z,w,s,t,p,q,r,m,n,u) \
	CTypelist<x, MakeTypelist11(y, z, w, s, t, p, q, r, m, n, u) >

/**
 *	@def MakeTypelist13
 *	@param[in] v is the item of typelist
 *	@copydoc MakeTypelist12
 */
#define MakeTypelist13(x,y,z,w,s,t,p,q,r,m,n,u,v) \
	CTypelist<x, MakeTypelist12(y, z, w, s, t, p, q, r, m, n, u, v) >

/**
 *	@def MakeTypelist14
 *	@param[in] a is the item of typelist
 *	@copydoc MakeTypelist13
 */
#define MakeTypelist14(x,y,z,w,s,t,p,q,r,m,n,u,v,a) \
	CTypelist<x, MakeTypelist13(y, z, w, s, t, p, q, r, m, n, u, v, a) >

/**
 *	@def MakeTypelist15
 *	@param[in] b is the item of typelist
 *	@copydoc MakeTypelist14
 */
#define MakeTypelist15(x,y,z,w,s,t,p,q,r,m,n,u,v,a,b) \
	CTypelist<x, MakeTypelist14(y, z, w, s, t, p, q, r, m, n, u, v, a, b) >

/**
 *	@def MakeTypelist16
 *	@param[in] c is the item of typelist
 *	@copydoc MakeTypelist15
 */
#define MakeTypelist16(x,y,z,w,s,t,p,q,r,m,n,u,v,a,b,c) \
	CTypelist<x, MakeTypelist15(y, z, w, s, t, p, q, r, m, n, u, v, a, b, c) >

#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER >= 1400

/**
 *	@brief safe typelist constructor
 *	@tparam _TyFunction is a function pointer type (unrelated to typelist contents,
 *		merely a mechanism of resolving the types themselves using partial specialization)
 */
template <class _TyFunction>
struct CSafeTypelistCtor;

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 */
template <class x>
struct CSafeTypelistCtor<void (*)(x)> {
	typedef MakeTypelist1(x) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 */
template <class x, class y>
struct CSafeTypelistCtor<void (*)(x, y)> {
	typedef MakeTypelist2(x, y) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 */
template <class x, class y, class z>
struct CSafeTypelistCtor<void (*)(x, y, z)> {
	typedef MakeTypelist3(x, y, z) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 */
template <class x, class y, class z, class w>
struct CSafeTypelistCtor<void (*)(x, y, z, w)> {
	typedef MakeTypelist4(x, y, z, w) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 */
template <class x, class y, class z, class w, class s>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s)> {
	typedef MakeTypelist5(x, y, z, w, s) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t)> {
	typedef MakeTypelist6(x, y, z, w, s, t) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p)> {
	typedef MakeTypelist7(x, y, z, w, s, t, p) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p, class q>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q)> {
	typedef MakeTypelist8(x, y, z, w, s, t, p, q) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p, class q, class r>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r)> {
	typedef MakeTypelist9(x, y, z, w, s, t, p, q, r) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 */
template <class x, class y, class z, class w, class s,
	class t, class p, class q, class r, class m>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m)> {
	typedef MakeTypelist10(x, y, z, w, s, t, p, q, r, m) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 *	@tparam n is the item of typelist
 */
template <class x, class y, class z, class w, class s,
	class t, class p, class q, class r, class m, class n>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m, n)> {
	typedef MakeTypelist11(x, y, z, w, s, t, p, q, r, m, n) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 *	@tparam n is the item of typelist
 *	@tparam u is the item of typelist
 */
template <class x, class y, class z, class w, class s,
	class t, class p, class q, class r, class m, class n, class u>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m, n, u)> {
	typedef MakeTypelist12(x, y, z, w, s, t, p, q, r, m, n, u) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 *	@tparam n is the item of typelist
 *	@tparam u is the item of typelist
 *	@tparam v is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p, class q,
	class r, class m, class n, class u, class v>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m, n, u, v)> {
	typedef MakeTypelist13(x, y, z, w, s, t, p, q, r, m, n, u, v) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 *	@tparam n is the item of typelist
 *	@tparam u is the item of typelist
 *	@tparam v is the item of typelist
 *	@tparam a is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p, class q,
	class r, class m, class n, class u, class v, class a>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m, n, u, v, a)> {
	typedef MakeTypelist14(x, y, z, w, s, t, p, q, r, m, n, u, v, a) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 *	@tparam n is the item of typelist
 *	@tparam u is the item of typelist
 *	@tparam v is the item of typelist
 *	@tparam a is the item of typelist
 *	@tparam b is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p, class q,
	class r, class m, class n, class u, class v, class a, class b>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m, n, u, v, a, b)> {
	typedef MakeTypelist15(x, y, z, w, s, t, p, q, r, m, n, u, v, a, b) _Ty; /**< @brief resulting typelist */
};

/**
 *	@brief safe typelist constructor
 *
 *	@tparam x is the item of typelist
 *	@tparam y is the item of typelist
 *	@tparam z is the item of typelist
 *	@tparam w is the item of typelist
 *	@tparam s is the item of typelist
 *	@tparam t is the item of typelist
 *	@tparam p is the item of typelist
 *	@tparam q is the item of typelist
 *	@tparam r is the item of typelist
 *	@tparam m is the item of typelist
 *	@tparam n is the item of typelist
 *	@tparam u is the item of typelist
 *	@tparam v is the item of typelist
 *	@tparam a is the item of typelist
 *	@tparam b is the item of typelist
 *	@tparam c is the item of typelist
 */
template <class x, class y, class z, class w, class s, class t, class p, class q,
	class r, class m, class n, class u, class v, class a, class b, class c>
struct CSafeTypelistCtor<void (*)(x, y, z, w, s, t, p, q, r, m, n, u, v, a, b, c)> {
	typedef MakeTypelist16(x, y, z, w, s, t, p, q, r, m, n, u, v, a, b, c) _Ty; /**< @brief resulting typelist */
};

/**
 *	@def MakeTypelist
 *	@brief makes a typelist (version with variable number of arguments)
 *	@param[in] ... is the list of items of typelist
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *	@note This only compiles on compilers, supporting variadic macros.
 */
#define MakeTypelist(...) CSafeTypelistCtor<void (*)(__VA_ARGS__)>::_Ty

/**
 *	@def MakeTypelist_Safe
 *	@brief makes a typelist (version with variable number of arguments)
 *	@param[in] args is the list of items of typelist
 *	@return Returns declaration of a typelist containing all the entered types, in the given order.
 *	@note This is used with double parentheses, e.g. MakeTypelist_Safe((char, short, int)).
 */
#define MakeTypelist_Safe(args) CSafeTypelistCtor<void (*)args>::_Ty

#endif // !_MSC_VER || __MWERKS__ || _MSC_VER >= 1400

#ifndef __TYPE_LIST_OMIT_OPS

/**
 *	@brief template for comparing two data types
 *
 *	@tparam _TyA is the first type
 *	@tparam _TyB is the second (different) type
 */
template <class _TyA, class _TyB>
class CEqualType {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = false /**< @brief result of comparison (the types are different) */
	};
};

/**
 *	@brief template for comparing two data types
 *		(specialization for case where the types equal)
 *
 *	@tparam _TyA is the first type (the same as the second)
 */
template <class _TyA>
class CEqualType<_TyA, _TyA> {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = true /**< @brief result of comparison (the types are equal) */
	};
};

/**
 *	@brief determines whether a typelist is empty
 *	@tparam _TyList is a typelist
 */
template <class _TyList>
class CTypelistEmpty {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = CEqualType<_TyList, CTypelistEnd>::b_result /**< @brief emptiness of the list (the result) */
	};
};

/**
 *	@brief calculates length of typelist (result in member enum n_result)
 *	@tparam _TyList is a typelist
 */
template <class _TyList>
class CTypelistLength {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		n_result = 1 + CTypelistLength<typename _TyList::_TyTail>::n_result /**< @brief length of the list (the result) */
	};
};

/**
 *	@brief calculates length of typelist (specialization for empty list
 *		(end of the list); result in member enum n_result)
 */
template <>
class CTypelistLength<CTypelistEnd> {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		n_result = 0 /**< @brief length of the list (the result) */
	};
};

/**
 *	@brief picks a typelist item, selected by an index
 *
 *	@tparam CList is a typelist
 *	@tparam n_index is a zero-based index of an item to be returned
 *
 *	@note This handles out of bound array access by returning the CTypelistEnd type.
 *	@note The index must be non-negative, otherwise a very
 *		long template recursion occurs (this is not checked).
 */
template <class CList, const int n_index> // note that index must be positive
class CTypelistItemAt {
public:
	typedef typename CTypelistItemAt<typename CList::_TyTail,
		n_index - 1>::_TyResult _TyResult; /**< @brief type of the selected item */
};

/**
 *	@brief picks a typelist item, selected by an index (specialization for a hit)
 *
 *	@tparam CList is a typelist
 *
 *	@note This handles out of bound array access by returning the CTypelistEnd type.
 *	@note The index must be non-negative, otherwise a very
 *		long template recursion occurs (this is not checked).
 */
template <class CList>
class CTypelistItemAt<CList, 0> {
public:
	typedef typename CList::_TyHead _TyResult; /**< @brief type of the selected item */
};

/**
 *	@brief picks a typelist item, selected by an index
 *		(specialization for out of bounds access by >1)
 *
 *	@tparam CList is a typelist
 *
 *	@note This handles out of bound array access by returning the CTypelistEnd type.
 *	@note The index must be non-negative, otherwise a very
 *		long template recursion occurs (this is not checked).
 */
template <const int n_index>
class CTypelistItemAt<CTypelistEnd, n_index> { // out of bounds by >1
public:
	typedef CTypelistEnd _TyResult; /**< @brief type of the selected item */
};

/**
 *	@brief picks a typelist item, selected by an index
 *		(specialization for out of bounds access by 1)
 *
 *	@note This handles out of bound array access by returning the CTypelistEnd type.
 *	@note The index must be non-negative, otherwise a very
 *		long template recursion occurs (this is not checked).
 */
template <>
class CTypelistItemAt<CTypelistEnd, 0> { // out of bounds by 1
public:
	typedef CTypelistEnd _TyResult; /**< @brief type of the selected item */
};

/**
 *	@brief template function enable helper
 *
 *	@tparam b_enable is enable flag (if set, this class defines type T to be
 *		used in template function declaration)
 *	@tparam CType is data type of the result, if enabled (default void)
 *
 *	@note This will only work for template functions where SFINAE applies.
 *		The template parameter may not be explicit but can be derived from arguments as well.
 */
template <bool b_enable, class CType = void>
class CEnableIf {
public:
	typedef CType T; /**< @brief result type */
};

/**
 *	@brief template function enable helper (specialization for function disabled)
 *	@tparam CType is data type of the result, if enabled (default void)
 */
template <class CType>
class CEnableIf<false, CType> {};

/**
 *	@brief selects one of two types, based on compile-time constant boolean condition
 *
 *	@tparam CFirstType is the first type
 *	@tparam CSecondType is the first type
 *	@tparam b_select_first_type is the value of the conditiom
 */
template <class CFirstType, class CSecondType, const bool b_select_first_type>
class CChooseType {
public:
	typedef CFirstType _TyResult; /**< @brief the selected type */
};

/**
 *	@brief selects one of two types, based on compile-time constant boolean condition
 *		(specialization for the condition being false)
 *
 *	@tparam CFirstType is the first type
 *	@tparam CSecondType is the first type
 */
template <class CFirstType, class CSecondType>
class CChooseType<CFirstType, CSecondType, false> {
public:
	typedef CSecondType _TyResult; /**< @brief the selected type */
};

/**
 *	@brief conditionally inherits from a type
 *
 *	@tparam CBase is the base type
 *	@tparam b_inherit is inheritance flag (if set, this will be derived from CBase)
 */
template <class CBase, const bool b_inherit>
class CInheritIf {};

/**
 *	@brief conditionally inherits from a type (specialization for inheritance)
 *	@tparam CBase is the base type
 */
template <class CBase>
class CInheritIf<CBase, true> : public CBase {};

/**
 *	@brief finds type in a typelist
 *
 *	@tparam CList is typelist type
 *	@tparam CItemType is type of the item being searched
 *	@tparam _n_index is index of the item in the list
 *		(being incremented in template expansions - do not change)
 */
template <class CList, class CItemType, const int _n_index = 0>
class CFindTypelistItem {
protected:
	typedef CEqualType<typename CList::_TyHead, CItemType> CCompareHead; /**< @brief head comparison class */
	typedef CFindTypelistItem<typename CList::_TyTail, CItemType, _n_index + 1> CSearchTail; /**< @brief tail search class */

public:
	/**
	 *	@brief results, stored as enums
	 */
	enum {
		b_result = CCompareHead::b_result || CSearchTail::b_result, /**< @brief search result (true if found, otherwise false) */
		n_index = (CCompareHead::b_result)? _n_index : CSearchTail::n_index /**< @brief index of the first occurence of the type in the list (or -1 if not found) */
	};
};

/**
 *	@brief finds type in a typelist (specialization for the end of the list, or an empty list)
 *
 *	@tparam CItemType is type of the item being searched
 *	@tparam _n_index is index of the item in the list
 *		(being incremented in template expansions - do not change)
 */
template <class CItemType, const int _n_index>
class CFindTypelistItem<CTypelistEnd, CItemType, _n_index> {
public:
	/**
	 *	@brief results, stored as enums
	 */
	enum {
		b_result = false, /**< @brief search result (true if found, otherwise false) */
		n_index = -1 /**< @brief index of the first occurence of the type in the list (or -1 if not found) */
	};
};

/**
 *	@brief finds type in a typelist
 *
 *	@tparam CList is typelist type
 *	@tparam CReference is type of the item being searched
 *	@tparam CPredicate is predicate getting the reference type and the item type (in this order)
 *	@tparam _n_index is index of the item in the list
 *		(being incremented in template expansions - do not change)
 */
template <class CList, class CReference, template <class, class> class CPredicate, const int _n_index = 0>
class CFindTypelistItem_If {
protected:
	typedef CPredicate<typename CList::_TyHead, CReference> CCompareHead; /**< @brief head comparison class */
	typedef CFindTypelistItem_If<typename CList::_TyTail, CReference, CPredicate, _n_index + 1> CSearchTail; /**< @brief tail search class */

public:
	/**
	 *	@brief results, stored as enums
	 */
	enum {
		b_result = CCompareHead::b_result || CSearchTail::b_result, /**< @brief search result (true if found, otherwise false) */
		n_index = (CCompareHead::b_result)? _n_index : CSearchTail::n_index /**< @brief index of the first occurence of the type in the list (or -1 if not found) */
	};
};

/**
 *	@brief finds type in a typelist (specialization for the end of the list, or an empty list)
 *
 *	@tparam CItemType is type of the item being searched
 *	@tparam _n_index is index of the item in the list
 *		(being incremented in template expansions - do not change)
 */
template <class CReference, template <class, class> class CPredicate, const int _n_index>
class CFindTypelistItem_If<CTypelistEnd, CReference, CPredicate, _n_index> {
public:
	/**
	 *	@brief results, stored as enums
	 */
	enum {
		b_result = false, /**< @brief search result (true if found, otherwise false) */
		n_index = -1 /**< @brief index of the first occurence of the type in the list (or -1 if not found) */
	};
};

/**
 *	@brief removes duplicate types from a typelist
 *
 *	@tparam CList is typelist type
 */
template <class CList>
class CUniqueTypelist {
protected:
	/*template <class CList, const int n_num_to_strip>
	class CListStripper {
	public:
		typedef typename CListStripper<typename CList::_TyTail, // don't bother with unique, will be stripped anyway as n_num_to_strip > 1
			n_num_to_strip - 1>::_TyResult _TyResult;
	};

	template <class CList>
	class CListStripper<CList, 1> {
	public:
		typedef typename CUniqueList<typename CList::_TyTail>::_TyList _TyUniqueRestOfTheList;
		typedef typename CListStripper<_TyUniqueRestOfTheList, 0>::_TyResult _TyResult;
	};

	template <class CList>
	class CListStripper<CList, 0> {
	public:
		typedef typename CUniqueList<typename CList::_TyTail>::_TyList _TyUniqueRestOfTheList;
		typedef CTypelist<typename CList::_TyHead, _TyUniqueRestOfTheList> _TyResult;
	};

	template <const int n_num_to_strip>
	class CListStripper<CTypelistEnd, n_num_to_strip> {
	public:
		typedef CTypelistEnd _TyResult;
	};*/
	// less elegant solution

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *
	 *	@tparam b_select_first_type is the value of the conditiom
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <const bool b_select_first_type, class CFirstType, class CSecondType>
	class CSelectType {
	public:
		typedef CFirstType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *		(specialization for the condition being false)
	 *
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <class CFirstType, class CSecondType>
	class CSelectType<false, CFirstType, CSecondType> {
	public:
		typedef CSecondType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief intermediates, stored as enums
	 */
	enum {
		b_head_is_unique = !CFindTypelistItem<typename CList::_TyTail,
			typename CList::_TyHead>::b_result /**< @brief decides whether head is unique */
	};

	typedef typename CUniqueTypelist<typename CList::_TyTail>::_TyResult _TyUniqueRestOfTheList; /**< @brief tail of the original list, containing only unique types */

public:
	typedef typename CSelectType<b_head_is_unique,
		CTypelist<typename CList::_TyHead, _TyUniqueRestOfTheList>,
		_TyUniqueRestOfTheList>::_TyResult _TyResult; /**< @brief resulting list, containing only unique types */
};

/**
 *	@brief removes duplicate types from a typelist
 *		(specialization for the end of the list, or an empty list)
 */
template <>
class CUniqueTypelist<CTypelistEnd> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief return CTypelistEnd, empty list is only CTypelistEnd */
};

/**
 *	@brief transforms typelist entries by a template functor
 *
 *	@tparam CList is typelist type
 *	@tparam CTransform is a template that performs typelist entries transformation
 */
template <class CList, template <class> class CTransform>
class CTransformTypelist {
public:
	typedef CTypelist<typename CTransform<typename CList::_TyHead>::_TyResult,
		typename CTransformTypelist<typename CList::_TyTail, CTransform>::_TyResult> _TyResult; /**< @brief resulting transformed list */
};

/**
 *	@brief transforms typelist entries by a template functor
 *		(specialization for the end of the list, or an empty list)
 *	@tparam CTransform is a template that performs typelist entries transformation
 */
template <template <class> class CTransform>
class CTransformTypelist<CTypelistEnd, CTransform> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief resulting transformed list */
};

/**
 *	@brief filters a typelist
 *
 *	Keeps all typelist items _Ty for which CPredicate<_Ty, CReference>::b_result != false.
 *
 *	@tparam CList is typelist type
 *	@tparam CReference is reference value
 *	@tparam CPredicate is a predicate, deciding whether to keep the types
 */
template <class CList, class CReference, template <class, class> class CPredicate>
class CFilterTypelist {
protected:
	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *
	 *	@tparam b_select_first_type is the value of the conditiom
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <const bool b_select_first_type, class CFirstType, class CSecondType>
	class CSelectType {
	public:
		typedef CFirstType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *		(specialization for the condition being false)
	 *
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <class CFirstType, class CSecondType>
	class CSelectType<false, CFirstType, CSecondType> {
	public:
		typedef CSecondType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		b_head_matches_reference = CPredicate<typename CList::_TyHead, CReference>::b_result /**< @brief result of predicate for the head of the list */
	};

	typedef typename CFilterTypelist<typename CList::_TyTail, CReference, CPredicate>::_TyResult _TyFilteredTail; /**< @brief filtered tail of the original typelist */

public:
	typedef typename CSelectType<b_head_matches_reference,
		CTypelist<typename CList::_TyHead, _TyFilteredTail>, _TyFilteredTail>::_TyResult _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief filters a typelist (specialization for the end of the list (or an empty list))
 *
 *	Keeps all typelist items _Ty for which CPredicate<_Ty, CReference>::b_result != false.
 *
 *	@tparam CReference is reference value
 *	@tparam CPredicate is a predicate, deciding whether to keep the types
 */
template <class CReference, template <class, class> class CPredicate>
class CFilterTypelist<CTypelistEnd, CReference, CPredicate> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief resulting typelist */ // return CTypelistEnd, empty list is only CTypelistEnd
};

/**
 *	@brief filters a typelist
 *
 *	Keeps all typelist items _Ty for which CPredicate<_Ty>::b_result != false.
 *
 *	@tparam CList is typelist type
 *	@tparam CPredicate is a predicate, deciding whether to keep the types
 */
template <class CList, template <class> class CPredicate>
class CFilterTypelist1 {
protected:
	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *
	 *	@tparam b_select_first_type is the value of the conditiom
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <const bool b_select_first_type, class CFirstType, class CSecondType>
	class CSelectType {
	public:
		typedef CFirstType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *		(specialization for the condition being false)
	 *
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <class CFirstType, class CSecondType>
	class CSelectType<false, CFirstType, CSecondType> {
	public:
		typedef CSecondType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		b_head_matches_reference = CPredicate<typename CList::_TyHead>::b_result /**< @brief result of predicate for the head of the list */
	};

	typedef typename CFilterTypelist1<typename CList::_TyTail, CPredicate>::_TyResult _TyFilteredTail; /**< @brief filtered tail of the original typelist */

public:
	typedef typename CSelectType<b_head_matches_reference,
		CTypelist<typename CList::_TyHead, _TyFilteredTail>, _TyFilteredTail>::_TyResult _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief filters a typelist (specialization for the end of the list (or an empty list))
 *	Keeps all typelist items _Ty for which CPredicate<_Ty>::b_result != false.
 *	@tparam CPredicate is a predicate, deciding whether to keep the types
 */
template <template <class> class CPredicate>
class CFilterTypelist1<CTypelistEnd, CPredicate> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief resulting typelist */ // return CTypelistEnd, empty list is only CTypelistEnd
};

/**
 *	@brief filters a typelist, with predicate context
 *
 *	Keeps all typelist items _Ty for which CPredicate<CPredContext, _Ty, CReference>::b_result != false.
 *
 *	@tparam CList is typelist type
 *	@tparam CReference is reference value
 *	@tparam CPredicate is a predicate, deciding whether to keep the types
 *	@tparam CPredContext is predicate context
 */
template <class CList, class CReference, template <class, class, class> class CPredicate, class CPredContext>
class CFilterTypelist2 {
protected:
	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *
	 *	@tparam b_select_first_type is the value of the conditiom
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <const bool b_select_first_type, class CFirstType, class CSecondType>
	class CSelectType {
	public:
		typedef CFirstType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *		(specialization for the condition being false)
	 *
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <class CFirstType, class CSecondType>
	class CSelectType<false, CFirstType, CSecondType> {
	public:
		typedef CSecondType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		b_head_matches_reference = CPredicate<CPredContext, typename CList::_TyHead, CReference>::b_result /**< @brief result of predicate for the head of the list */
	};

	typedef typename CFilterTypelist2<typename CList::_TyTail, CReference, CPredicate, CPredContext>::_TyResult _TyFilteredTail; /**< @brief filtered tail of the original typelist */

public:
	typedef typename CSelectType<b_head_matches_reference,
		CTypelist<typename CList::_TyHead, _TyFilteredTail>, _TyFilteredTail>::_TyResult _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief filters a typelist, with predicate context (specialization for the end of the list (or an empty list))
 *
 *	Keeps all typelist items _Ty for which CPredicate<_Ty, CReference, CPredContext>::b_result != false.
 *
 *	@tparam CReference is reference value
 *	@tparam CPredicate is a predicate, deciding whether to keep the types
 *	@tparam CPredContext is predicate context
 */
template <class CReference, template <class, class, class> class CPredicate, class CPredContext>
class CFilterTypelist2<CTypelistEnd, CReference, CPredicate, CPredContext> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief resulting typelist */ // return CTypelistEnd, empty list is only CTypelistEnd
};

/**
 *	@brief performs one bubble-sort run on a typelist
 *
 *	@tparam CList is a typelist to be sorted
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *
 *	@note This should not be directly used, use CSortTypelist instead.
 */
template <class CList, template <class, class> class CComparisonOp>
class CSortTypelistRun {
protected:
	typedef typename CList::_TyHead _TyFirst; /**< @brief the first element of the list */
	typedef typename CList::_TyTail::_TyHead _TySecond; /**< @brief the second element of the list */
	typedef typename CList::_TyTail::_TyTail _TyRestOfList; /**< @brief the rest of the list, without the first two elements */

	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		b_cmp = !CComparisonOp<_TySecond, _TyFirst>::b_result /**< @brief result of the comparison operation on the two first items of the list */
		// note the comparison was inverted and the order was swapped in order to handle
		// equal items in the sorted sequence (before these would be swapped indefinitely)
	};

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *
	 *	@tparam b_select_first_type is the value of the conditiom
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <const bool b_select_first_type, class CFirstType, class CSecondType>
	class CSelectType {
	public:
		typedef CFirstType _TyResult; /**< @brief the selected type */
	};

	/**
	 *	@brief selects one of two types, based on compile-time constant boolean condition
	 *		(specialization for the condition being false)
	 *
	 *	@tparam CFirstType is the first type
	 *	@tparam CSecondType is the first type
	 */
	template <class CFirstType, class CSecondType>
	class CSelectType<false, CFirstType, CSecondType> {
	public:
		typedef CSecondType _TyResult; /**< @brief the selected type */
	};

	typedef typename CSelectType<b_cmp, _TyFirst, _TySecond>::_TyResult _TyNewFirst; /**< @brief a new first element, based on swap */
	typedef typename CSelectType<!b_cmp, _TyFirst, _TySecond>::_TyResult _TyNewSecond; /**< @brief a new second element, based on swap */
	typedef CTypelist<_TyNewSecond, _TyRestOfList> _TyNewTail; /**< @brief a new tail with the new second element attached */
	typedef CSortTypelistRun<_TyNewTail, CComparisonOp> CTailSort; /**< @brief sort for the new tail */

public:
	/**
	 *	@brief results, stored as enum
	 */
	enum {
		b_swapped = !b_cmp || CTailSort::b_swapped /**< @brief typelist modification flag (if set, more sorting runs are required) */
	};

	typedef CTypelist<_TyNewFirst, typename CTailSort::_TyResult> _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief performs one bubble-sort run on a typelist (a specialization for a
 *		typelist with a single item or the end of a list with odd number of items)
 *
 *	@tparam CLastItem is the last odd item in the typelist to be sorted
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *
 *	@note This should not be directly used, use CSortTypelist instead.
 */
template <class CLastItem, template <class, class> class CComparisonOp>
class CSortTypelistRun<CTypelist<CLastItem, CTypelistEnd>, CComparisonOp> {
public:
	/**
	 *	@brief results, stored as enum
	 */
	enum {
		b_swapped = false /**< @brief typelist modification flag (if set, more sorting runs are required) */
	};

	typedef CTypelist<CLastItem, CTypelistEnd> _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief performs one bubble-sort run on a typelist
 *		(a specialization for an empty typelist or the end of the list)
 *
 *	@tparam CList is a typelist to be sorted
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *
 *	@note This should not be directly used, use CSortTypelist instead.
 */
template <template <class, class> class CComparisonOp>
class CSortTypelistRun<CTypelistEnd, CComparisonOp> {
public:
	/**
	 *	@brief results, stored as enum
	 */
	enum {
		b_swapped = false /**< @brief typelist modification flag (if set, more sorting runs are required) */
	};

	typedef CTypelistEnd _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief sorts a typelist
 *
 *	@tparam CList is a typelist to be sorted
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *	@tparam b_iterate is an iteration flag (should not be set by the caller)
 */
template <class CList, template <class, class> class CComparisonOp, const bool b_iterate = true>
class CSortTypelist {
protected:
	typedef CSortTypelistRun<CList, CComparisonOp> CSortRun; /**< @brief one sort run on the input list */
	typedef typename CSortRun::_TyResult _TySortedList; /**< @brief the (partially) sorted input list */

	/**
	 *	@brief results, stored as enum
	 */
	enum {
		b_repeat = CSortRun::b_swapped /**< @brief typelist modification flag (if set, more sorting runs are required) */
	};

public:
	typedef typename CSortTypelist<_TySortedList, CComparisonOp, b_repeat>::_TyResult _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief sorts a typelist (specialization for the last sort iteration)
 *
 *	@tparam CList is a typelist to be sorted
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 */
template <class CList, template <class, class> class CComparisonOp>
class CSortTypelist<CList, CComparisonOp, false> {
public:
	typedef CList _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief concatenates two typelists
 *
 *	@tparam CFirstList is the first typelist, or CTypelistEnd for an empty list
 *	@tparam CSecondList is the second typelist, or CTypelistEnd for an empty list
 */
template <class CFirstList, class CSecondList>
class CConcatTypelist {
protected:
	typedef typename CFirstList::_TyHead THead; /**< @brief head of the first typelist */
	typedef typename CFirstList::_TyTail TRestOfTheList; /**< @brief the rest of the first typelist */
	typedef typename CConcatTypelist<TRestOfTheList, CSecondList>::_TyResult TConcatRestOfTheList; /**< @brief the rest of the first typelist with the second list appended */

public:
	typedef CTypelist<THead, TConcatRestOfTheList> _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief concatenates two typelists (specialization for the end of the list (or an empty list))
 *	@tparam CSecondList is the second typelist, or CTypelistEnd for an empty list
 */
template <class CSecondList>
class CConcatTypelist<CTypelistEnd, CSecondList> {
public:
	typedef CSecondList _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief concatenates two typelists (specialization for the empty second list)
 *	@tparam CFirstList is the first typelist, or CTypelistEnd for an empty list
 */
template <class CFirstList>
class CConcatTypelist<CFirstList, CTypelistEnd> {
public:
	typedef CFirstList _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief concatenates two typelists (specialization for both lists being empty)
 */
template <>
class CConcatTypelist<CTypelistEnd, CTypelistEnd> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief resulting typelist */
};

/**
 *	@brief splits a typelist in two halves
 *
 *	@tparam CList is input typelist
 *	@tparam n_split is length of the first half of the split (must be
 *		nonnegative and less or equal to the list length)
 */
template <class CList, const int n_split>
class CSplitTypelist {
public:
	typedef CTypelist<typename CList::_TyHead,
		typename CSplitTypelist<typename CList::_TyTail, n_split - 1>::_TyFirst> _TyFirst; /**< @brief the first half of the split */
	typedef typename CSplitTypelist<typename CList::_TyTail, n_split - 1>::_TySecond _TySecond; /**< @brief the second half of the split */
};

/**
 *	@brief splits a typelist in two halves (specialization)
 *	@tparam CList is input typelist
 */
template <class CTypeList>
class CSplitTypelist<CTypeList, 0> {
public:
	typedef CTypelistEnd _TyFirst; /**< @brief the first half of the split */
	typedef CTypeList _TySecond; /**< @brief the second half of the split */
};

/**
 *	@brief gets a span of a typelist
 *
 *	@tparam CList is input typelist
 *	@tparam n_begin is zero-based index of the first item in the resulting list
 *	@tparam n_end is zero-based index of one past the last item in the resulting list
 */
template <class CList, const int n_begin, const int n_end>
class CTypelistSpan {
public:
	typedef typename CSplitTypelist<typename CSplitTypelist<CList, n_begin>::_TySecond,
		n_end - n_begin>::_TyFirst _TyResult; /**< @brief the result */
	// this first cuts begin and then end, it takes O(n_begin) + O(n_end - n_begin) of recursions
	// the other split order would be O(n_end) + O(n_begin) which is clearly more costly
};

/**
 *	@brief gets a typelist without an item at a specified position
 *
 *	@tparam CList is input typelist
 *	@tparam n_index is zero-based index of the item to be erased
 */
template <class CList, const int n_index>
class CTypelistErase {
protected:
	typedef CSplitTypelist<CList, n_index> _TySplit; /**< @brief first split of the list */
	typedef typename _TySplit::_TyFirst _TyHead; /**< @brief the list before the erased item */
	typedef typename CSplitTypelist<typename _TySplit::_TySecond, 1>::_TySecond _TyTail; /**< @brief the list after the erased item */

public:
	typedef typename CConcatTypelist<_TyHead, _TyTail>::_TyResult _TyResult; /**< @brief the result */
};

/**
 *	@brief concatenates a list of typelists
 *	@tparam CListOfLists is a list of typelists (a 2D jagged list) to be concatenated
 */
template <class CListOfLists>
class CConcatListOfTypelists {
protected:
	typedef typename CListOfLists::_TyHead CFirstList; /**< @brief the first list */
	typedef typename CListOfLists::_TyTail CRestOfLists; /**< @brief the rest of the lists */

	// it may be better if we could have some divide and conquer - split
	// the list to two lists and concatenate them, then concatenate the results

	// for n list with m elements each, this takes O(mn^2 / 2)
	// to split it in two first, it would take O(n / 2) for the split and then O(mn^2 / 8) + O(mn / 2)
	// not so sure if that is better and also how the compiler would deal with it

public:
	typedef typename CConcatTypelist<CFirstList, typename
		CConcatListOfTypelists<CRestOfLists>::_TyResult>::_TyResult _TyResult; /**< @brief the concatenated list */
};

/**
 *	@brief concatenates a list of typelists (specialization for a list, containing a single list)
 *	@tparam CSingleList is a single typelist to be concatenated
 */
template <class CSingleList>
class CConcatListOfTypelists<CTypelist<CSingleList, CTypelistEnd> > {
public:
	typedef CSingleList _TyResult; /**< @brief the concatenated list */
};

/**
 *	@brief concatenates a list of typelists (specialization for an empty list)
 */
template <>
class CConcatListOfTypelists<CTypelistEnd> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief the concatenated list */
};

/**
 *	@brief calculates cartesian product of a scalar on left side and a type list on right side
 *
 *	@tparam _TLeft is a scalar type
 *	@tparam _TRight is a type list, containing a set of scalars
 *	@tparam CScalarProduct is a template that creates ordered pair from two scalars
 *		passed to it in its parameters (result expected in member type _TyResult)
 */
template <class _TLeft, class _TRight, template <class, class> class CScalarProduct>
class CLeftProductTypelist {
protected:
	typedef typename _TRight::_TyHead THead; /**< @brief head of the right set */
	typedef typename CScalarProduct<_TLeft, THead>::_TyResult TNewHead; /**< @brief an ordered pair (_TLeft, THead) */
	typedef typename _TRight::_TyTail TTail;  /**< @brief the rest of the right set */
	typedef typename CLeftProductTypelist<_TLeft, TTail, CScalarProduct>::_TyResult TNewTail; /**< producto of _TLeft and the rest of the right list */

public:
	typedef typename CConcatTypelist<CTypelist<TNewHead, CTypelistEnd>, TNewTail>::_TyResult _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief calculates cartesian product of a scalar on left side and a type list
 *		 on right side (specialization for empty typelist (or the end of the list))
 *
 *	@tparam _TLeft is a scalar type
 *	@tparam CScalarProduct is a template that creates ordered pair from two scalars
 *		passed to it in its parameters (result expected in member type _TyResult)
 */
template <class _TLeft, template <class, class> class CScalarProduct>
class CLeftProductTypelist<_TLeft, CTypelistEnd, CScalarProduct> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief calculates cartesian product of two type lists
 *
 *	@tparam _TLeft is the first type list, containing a set of scalars
 *	@tparam _TRight is the second type list, containing a set of scalars
 *	@tparam CScalarProduct is a template that creates ordered pair from two scalars
 *		passed to it in its parameters (result expected in member type _TyResult)
 */
template <class _TLeft, class _TRight, template <class, class> class CScalarProduct>
class CCarthesianProductTypelist {
public:
	typedef typename _TLeft::_TyHead THead; /**< @brief head of the left set */
	typedef typename CLeftProductTypelist<THead, _TRight, CScalarProduct>::_TyResult TNewHead; /**< @brief a product of THead and the right typelist */
	typedef typename _TLeft::_TyTail TTail; /**< @brief the rest of the left set */
	typedef typename CCarthesianProductTypelist<TTail, _TRight, CScalarProduct>::_TyResult TNewTail; /**< @brief the rest of the carthesian product */

public:
	typedef typename CConcatTypelist<TNewHead, TNewTail>::_TyResult _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief calculates cartesian product of two type lists (specialization
 *		for empty typelist (end of the list))
 *
 *	@tparam _TRight is the second type list, containing a set of scalars
 *	@tparam CScalarProduct is a template that creates ordered pair from two scalars
 *		passed to it in its parameters (result expected in member type _TyResult)
 */
template <class _TRight, template <class, class> class CScalarProduct>
class CCarthesianProductTypelist<CTypelistEnd, _TRight, CScalarProduct> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief applies an binary op in elementwise manner to two typelists of equal length
 *
 *	@tparam CLeftList is left-side typelist
 *	@tparam CRightList is right-side typelist
 *	@tparam CBinaryOp is a binary operation template,
 *		taking two types and storing result in public member type _TyResult
 *
 *	@note Both typelists must be of equal length, otherwise compile error(s) occur.
 */
template <class CLeftList, class CRightList, template <class, class> class CBinaryOp>
class CElementwiseBinaryOpTypelist {
protected:
	typedef typename CLeftList::_TyHead _TyLeftHead; /**< @brief left-side typelist head */
	typedef typename CRightList::_TyHead _TyRightHead; /**< @brief right-side typelist head */
	typedef typename CLeftList::_TyTail _TyLeftTail; /**< @brief left-side typelist tail */
	typedef typename CRightList::_TyTail _TyRightTail; /**< @brief right-side typelist tail */
	typedef typename CBinaryOp<_TyLeftHead, _TyRightHead>::_TyResult _TyNewHead; /**< @brief result of the binary operation on the heads */
	typedef typename CElementwiseBinaryOpTypelist<_TyLeftTail,
		_TyRightTail, CBinaryOp>::_TyResult _TyNewTail; /**< @brief result of the binary operation on the tails */

public:
	typedef CTypelist<_TyNewHead, _TyNewTail> _TyResult; /**< @brief the resulting list */
};

/**
 *	@brief applies an binary op in elementwise manner to two typelists of equal length
 *		(specialization for the end of the lists, or for empty lists)
 *	@tparam CBinaryOp is a binary operation template,
 *		taking two types and storing result in public member type _TyResult
 *	@note Both typelists must be of equal length, otherwise compile error(s) occur.
 */
template <template <class, class> class CBinaryOp>
class CElementwiseBinaryOpTypelist<CTypelistEnd, CTypelistEnd, CBinaryOp> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief the resulting list */
};

/**
 *	@brief generates a list of products of a typelist in elementwise manner
 *
 *	For example, if the typelist contains numbers and the binary operation is addition,
 *	it generates a list of all the (elementwise) integer multiples of these numbers,
 *	up to a given limit.
 *
 *	@tparam CList is the input typelist
 *	@tparam CBinaryOp is a binary operation template,
 *		taking two types and storing result in public member type _TyResult
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *	@tparam CLimit is a limit, uased in conjunction with CComparisonOp to limit
 *		template recursion (more products will only be generated if the last
 *		set of products still contains elements under limit)
 *	@tparam CAccumList is the input typelist (used to accumulate products from template
 *		recursion; should not be set by the caller)
 *	@tparam b_expand is a parameter, controlling template recursion
 *		(should not be set by the caller)
 *
 *	@note The result is not sorted and may contain multiple occurences of the same types.
 *	@note This will result in infinite recursion in case the input list contains elements
 *		that, when binary operation is applied to them, will never exceed the limit CLimit
 *		(this will typically happen for a binaty op that produces identity for a given value,
 *		e.g. a zero as an input list element and addition as binary op, will never reach any
 *		given positive limit). Note that this could be solved by looking only for *new*
 *		values, but that would make the compilation much slower.
 */
template <class CList, template <class, class> class CBinaryOp,
	template <class, class> class CComparisonOp, class CLimit,
	class CAccumList = CList, const bool b_expand = true>
class CElementwiseProductTypelist {
protected:
	typedef typename CElementwiseBinaryOpTypelist<CAccumList,
		CList, CBinaryOp>::_TyResult _TyCombined; /**< @brief one iteration of binary operation on the accumulated list */
	typedef typename CFilterTypelist<_TyCombined, CLimit,
		CComparisonOp>::_TyResult _TyUnderLimit; /**< @brief filtered combined list with only types that passed the comparison with the limit type */

	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		b_stop = !CEqualType<_TyUnderLimit, CTypelistEnd>::b_result /**< @brief stopping flag for template recursion @note This will fail if the list contains a value for which the binary operation is identity. */
	};

	typedef typename CElementwiseProductTypelist<CList, CBinaryOp,
		CComparisonOp, CLimit, _TyCombined, b_stop>::_TyResult _TyHighest; /**< @brief results from the next recursion(s) */
	typedef typename CFilterTypelist<_TyHighest, CLimit,
		CComparisonOp>::_TyResult _TyHighestUnderLimit; /**< @brief filtered results from the next recursion(s) with only types that passed the comparison with the limit type */

public:
	typedef typename CConcatTypelist<CAccumList, _TyHighestUnderLimit>::_TyResult _TyResult; /**< @brief the resulting typelist @note This list may contain multiple occurences of the same types. */
};

/**
 *	@brief generates a list of products of a typelist in elementwise manner
 *		(specialization for recursion termination)
 *
 *	@tparam CList is the input typelist
 *	@tparam CBinaryOp is a binary operation template,
 *		taking two types and storing result in public member type _TyResult
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *	@tparam CLimit is a limit, uased in conjunction with CComparisonOp to limit
 *		template recursion (more products will only be generated if the last
 *		set of products still contains elements under limit)
 *	@tparam CAccumList is the input typelist (used to accumulate products from template
 *		recursion; should not be set by the caller)
 */
template <class CList, template <class, class> class CBinaryOp,
	template <class, class> class CComparisonOp, class CLimit, class CAccumList>
class CElementwiseProductTypelist<CList, CBinaryOp, CComparisonOp, CLimit, CAccumList, false> {
public:
	typedef CAccumList _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief generates list of results of chains of binary
 *		combinations of elements in a list, under a given limit
 *
 *	This takes elements in the list and makes binary combinations in the
 *	every-with-every manner. E.g. if the binary operation is addition
 *	and the list contains numbers, this generates all possible sums of the
 *	original numbers, under a given limit.
 *	Another use would be generating all possible permutations from a set
 *	(if the binary operation would be concatenation and the limit would be
 *	desired size of the permutations).
 *
 *	@tparam CList is the input typelist
 *	@tparam CBinaryOp is a binary operation template,
 *		taking two types and storing result in public member type _TyResult
 *	@tparam CComparisonOp is a binary comparison template,
 *		taking two types and storing result in public enum b_result
 *	@tparam CLimit is a limit, uased in conjunction with CComparisonOp to limit
 *		template recursion
 *
 *	@note This will result in infinite recursion in case the input list contains elements
 *		that, when binary operation is applied to them, will never exceed the limit CLimit
 *		(this will typically happen for a binaty op that produces identity for a given value,
 *		e.g. a zero as an input list element and addition as binary op, will never reach any
 *		given positive limit). Note that this could be solved by looking only for *new*
 *		values, but that would make the compilation much slower.
 */
template <class CList, template <class, class> class CBinaryOp,
	template <class, class> class CComparisonOp, class CLimit>
class CBinaryCombinationTypelist {
protected:
	typedef typename CUniqueTypelist<typename CElementwiseProductTypelist<CList,
		CBinaryOp, CComparisonOp, CLimit>::_TyResult>::_TyResult _TySizeList2; /**< @brief products of types from CList, all the way up to CLimit */
	typedef typename CCarthesianProductTypelist<_TySizeList2,
		_TySizeList2, CBinaryOp>::_TyResult _TyAddList; /**< @brief typelist of all possible products (calculated only once, this one is expensive) */
	typedef typename CFilterTypelist<_TyAddList, CLimit,
		CComparisonOp>::_TyResult _TyMultiplyList; /**< @brief the original list, with all the products added */

public:
	typedef typename CUniqueTypelist<typename CConcatTypelist<CList,
		_TyMultiplyList>::_TyResult>::_TyResult _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief iterates typelist entries and passes them to a template functor
 *
 *	@tparam CList is typelist type
 *	@tparam CFunctor is a class that performs operation
 *		on typelist entries in its template function operator
 *
 *	@note This executes operations at run time.
 */
template <class CList, class CFunctor>
class CTypelistForEach {
public:
	/**
	 *	@brief iterates all elements of the typelist
	 *	@param[in] f is a functor that operates on the typelist entriex
	 *	@return Returns the functor after iterating all the elements.
	 */
	static inline CFunctor Run(CFunctor f)
	{
#if defined(_MSC_VER) && !defined(__MWERKS__)
		f.operator ()<typename CList::_TyHead>(); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
		f.template operator ()<typename CList::_TyHead>();
#endif // _MSC_VER && !__MWERKS__
		// call the function operator

		return CTypelistForEach<typename CList::_TyTail, CFunctor>::Run(f);
	}
};

/**
 *	@brief calculates typelist reduction using a binary function
 *
 *	The functor is applied to pairs of (typelist item, tail reduction)
 *	and at the end to a pair of the last two typelist items. Lists with
 *	a single item in them are left unchanged.
 *
 *	@tparam CList is input typelist
 *	@tparam CFunctor is binary functor
 */
template <class CList, template <class, class> class CFunctor>
class CTypelistReduce {
protected:
	typedef typename CList::_TyHead _TyHead; /**< @brief the current list item */
	typedef typename CList::_TyTail _TyTail; /**< @brief list tail */
	typedef typename CTypelistReduce<_TyTail,
		CFunctor>::_TyResult _TyTailReduction; /**< @brief reduction of the tail */

public:
	typedef typename CFunctor<_TyHead, _TyTailReduction>::_TyResult _TyResult; /**< @brief reduction result */
};

/**
 *	@brief calculates typelist reduction using a binary function
 *		(specialization for lists of length 1 or the end of the list)
 */
template <class CLastItem, template <class, class> class CFunctor>
class CTypelistReduce<CTypelist<CLastItem, CTypelistEnd>, CFunctor> {
public:
	typedef CLastItem _TyResult; /**< @brief reduction result */
};

/**
 *	@brief calculates typelist reduction using a binary function (specialization for empty list)
 */
template <template <class, class> class CFunctor>
class CTypelistReduce<CTypelistEnd, CFunctor> {
public:
	typedef CTypelistEnd _TyResult; /**< @brief reduction result */
};

/**
 *	@brief calculates typelist reduction using a binary function
 *
 *	The functor is applied to pairs of (typelist item, tail reduction)
 *	and at the end to a pair of (typelist item, initial value).
 *
 *	@tparam CList is input typelist
 *	@tparam CFunctor is binary functor
 *	@tparam CInitialValue is initial reduction result type
 *
 *	@note This generates the same result as CTypelistReduce using the same
 *		functor, but executed on a list with CInitialValue appended at the end:
 *		CTypelistReduce<CConcatTypelist<CList, MakeTypelist(CInitialValue)>::_TyResult, CFunctor>.
 */
template <class CList, template <class, class> class CFunctor, class CInitialValue>
class CTypelistReduce2 {
protected:
	typedef typename CList::_TyHead _TyHead; /**< @brief the current list item */
	typedef typename CList::_TyTail _TyTail; /**< @brief list tail */
	typedef typename CTypelistReduce2<_TyTail, CFunctor,
		CInitialValue>::_TyResult _TyTailReduction; /**< @brief reduction of the tail */

public:
	typedef typename CFunctor<_TyHead, _TyTailReduction>::_TyResult _TyResult; /**< @brief reduction result */
};

/**
 *	@brief calculates typelist reduction using a binary function
 *		(specialization for an empty list or the end of the list)
 */
template <template <class, class> class CFunctor, class CInitialValue>
class CTypelistReduce2<CTypelistEnd, CFunctor, CInitialValue> {
public:
	typedef CInitialValue _TyResult; /**< @brief reduction result */
};

/**
 *	@brief iterates typelist entries and passes them to a template functor
 *		(specialization for the end of the list, or an empty list)
 *
 *	@tparam CFunctor is a class that performs operation
 *		on typelist entries in its template function operator
 *
 *	@note This executes operations at run time.
 */
template <class CFunctor>
class CTypelistForEach<CTypelistEnd, CFunctor> {
public:
	/**
	 *	@brief iterates all elements of the typelist
	 *	@param[in] f is a functor that operates on the typelist entriex
	 *	@return Returns the functor after iterating all the elements.
	 */
	static inline CFunctor Run(CFunctor f)
	{
		return f; // nothing else to iterate
	}
};

namespace tl_detail {

/**
 *	@brief static assertion helper
 *	@brief b_expression is expression being asserted
 */
template <const bool b_expression>
class CStaticAssert {
public:
	typedef void DECISION_TREE_REQUIRES_SORTED_TYPELISTS; /**< @brief static assertion tag */
};

/**
 *	@brief static assertion helper (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

} // ~tl_detail

/**
 *	@brief decision tree skeleton template; note that this is only dependent
 *		on the typelist and so can be reused with different contexts
 *
 *	@tparam CList is typelist of sorted CCTSize specializations (must be sorted)
 *	@tparam CComparator is compile-time pivot to run-time reference value comparator template
 *	@tparam CReference is reference value type
 */
template <class CList, template <class> class CComparator /*= fbs_ut::CRuntimeCompareScalar*/,
	class CReference /*= size_t*/>
class CMakeDecisionTreeSkeleton {
public:
#ifdef _DEBUG
	typedef typename CUniqueTypelist<CList>::_TyResult _TyUniqueList;
	enum {
		b_is_unique = CEqualType<_TyUniqueList, CList>::b_result
	};
	typedef typename tl_detail::CStaticAssert<b_is_unique>::DECISION_TREE_REQUIRES_SORTED_TYPELISTS _TyAssert0; // this is an internal error mostly, forgotten to unique the typelist somewhere
#endif // _DEBUG

	/**
	 *	@brief makes a branch of a decission tree from a sorted typelist
	 *
	 *	@tparam n_begin is a (zero based) index of the starting element of CList
	 *		this branch decides over
	 *	@tparam n_length is number of elements of CList this branch decides over
	 */
	template <const int n_begin, const int n_length>
	class CSkeleton {
	public:
#if 0
		/**
		 *	@brief decision tree parameters stored as enums
		 */
		enum {
			n_half = (n_length + 1) / 2, /**< @brief half of the length (round up, comparison is less than) */
			n_pivot = n_begin + n_half, /**< @brief zero-based index of the pivot element */
			n_rest = n_length - n_half, /**< @brief the remaining half of the length for the right child */
			b_leaf = false /**< @brief leaf flag */
		};

		typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyPivot; /**< @brief pivot type */

		typedef CSkeleton<n_begin, n_half> _TyLeft; /**< left subtree */
		typedef CSkeleton<n_pivot, n_rest> _TyRight; /**< right subtree (has the pivot) */

		static inline bool b_Left_of_Pivot(const CReference reference_value) // returns the result of reference_value < _TyPivot
		{
			return !CComparator<_TyPivot>::b_LessThan(reference_value) &&
			   !CComparator<_TyPivot>::b_Equal(reference_value);
		}
#else // 0
		/**
		 *	@brief decision tree parameters stored as enums
		 */
		enum {
			n_half = (n_length + 1) / 2, /**< @brief half of the length */
			n_pivot = n_begin + n_half - 1, /**< @brief zero-based index of the pivot element */
			n_rest = n_length - n_half, /**< @brief the remaining half of the length for the right child */
			b_leaf = false /**< @brief leaf flag */
		};

		typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyPivot; /**< @brief pivot type */

		typedef CSkeleton<n_begin + n_half, n_rest> _TyLeft; /**< left subtree */
		typedef CSkeleton<n_begin, n_half> _TyRight; /**< right subtree (has the pivot) */

		static inline bool b_Left_of_Pivot(const CReference reference_value) // returns the result of reference_value > _TyPivot
		{
			return CComparator<_TyPivot>::b_LessThan(reference_value);
			// this is actually reference greater than pivot
			// but that's ok, the left/right halves were swapped
		}
#endif // 0
	};

	/**
	 *	@brief makes a branch of a decission tree from a sorted typelist
	 *		(specialization for leafs)
	 *	@tparam n_begin is a (zero based) index of the starting element of CList
	 *		this branch decides over
	 */
	template <const int n_begin>
	class CSkeleton<n_begin, 1> {
	public:
		/**
		 *	@brief decision tree parameters stored as enums
		 */
		enum {
			n_pivot = n_begin, /**< @brief zero-based index of the pivot element */
			b_leaf = true /**< @brief leaf flag */
		};

		typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyPivot; /**< @brief pivot type */

		static inline bool b_Equals_Pivot(const CReference reference_value) // returns the result of reference_value == _TyPivot
		{
			return CComparator<_TyPivot>::b_Equal(reference_value);
		}
	};

	typedef CSkeleton<0, CTypelistLength<CList>::n_result> _TyRoot; /**< @brief root of the decition tree */

public:
	static inline void Debug()
	{
		printf("void CMakeDecisionTreeSkeleton::Debug(%s reference_value)\n{\n",
			s_TypeName<CReference>().c_str());
		Debug<_TyRoot>(1);
		printf("}\n");
	}

protected:
	template <class CNode>
	static inline typename CEnableIf<CNode::b_leaf>::T Debug(size_t n_indent)
	{
		printf("%*s_ASSERT(b_Equals_Pivot<%s>(reference_value)); // reached leaf\n",
			n_indent * 4, "", s_TypeName<typename CNode::_TyPivot>().c_str());
	}

	template <class CNode>
	static inline typename CEnableIf<!CNode::b_leaf>::T Debug(size_t n_indent)
	{
		printf("%*sif(b_Left_of_Pivot<%s>(reference_value)) {\n", n_indent * 4, "",
			s_TypeName<typename CNode::_TyPivot>().c_str());
		Debug<typename CNode::_TyLeft>(n_indent + 1);
		printf("%*s} else {\n", n_indent * 4, "");
		Debug<typename CNode::_TyRight>(n_indent + 1);
		printf("%*s}\n", n_indent * 4, "");
	}
};

/**
 *	@brief selects a typelist item by an index using a binary tree
 *		(at runtime) and calls a functor on the selected type
 *
 *	@tparam CList is a typelist from which an item is being selected
 *	@tparam CFunctor is a functor with template function operator to process the selected item
 *
 *	@note This executes operations at run time.
 */
template <class CList, class CFunctor>
class CTypelistItemSelect {
protected:
	/**
	 *	@brief makes a branch of a decision tree from a sorted typelist
	 *
	 *	@tparam n_begin is a (zero based) index of the starting element of CList
	 *		this branch decides over
	 *	@tparam n_length is number of elements of CList this branch decides over
	 */
	template <const int n_begin, const int n_length>
	class CDecisionTree {
	public:
		/**
		 *	@brief decision indices stored as enums
		 */
		enum {
			n_half = (n_length + 1) / 2, /**< @brief half of the length (round up, comparison is less than) */
			n_pivot = n_begin + n_half, /**< @brief zero-based index of the pivot element */
			n_rest = n_length - n_half /**< @brief the remaining half of the length for the right child */
		};

		/**
		 *	@brief selects a typelist item by an index using a binary tree
		 *		(at runtime) and calls a functor on the selected type
		 *
		 *	@param[in] n_index is a zero-based index of item to be selected from the list
		 *	@param[in] f is a functor with template function operator to process the selected item
		 *
		 *	@return Returns the functor after processing the selected item.
		 */
		static inline CFunctor Select(const size_t n_index, CFunctor f)
		{
			if(n_index < n_pivot) {
				return CDecisionTree</*_CList, _CFunctor,*/ n_begin, n_half>::Select(n_index, f);
			} else {
				return CDecisionTree</*_CList, _CFunctor,*/ n_pivot, n_rest>::Select(n_index, f);
			}
		}
	};

	/**
	 *	@brief makes a branch of a decision tree from a sorted typelist
	 *		(specialization for leafs)
	 *
	 *	@tparam n_begin is a (zero based) index of the starting element of CList
	 *		this branch decides over
	 */
	template <const int n_begin>
	class CDecisionTree<n_begin, 1> {
	public:
		typedef typename CTypelistItemAt<CList, n_begin>::_TyResult _TySelected; /**< @brief selected type */

		/**
		 *	@brief selects a typelist item by an index using a binary tree
		 *		(at runtime) and calls a functor on the selected type
		 *
		 *	@param[in] n_index is a zero-based index of item to be selected from the list
		 *	@param[in] f is a functor with template function operator to process the selected item
		 *
		 *	@return Returns the functor after processing the selected item.
		 */
		static inline CFunctor Select(const size_t UNUSED(n_index), CFunctor f)
		{
			_ASSERTE(n_index == n_begin);
#if defined(_MSC_VER) && !defined(__MWERKS__)
			f.operator ()<_TySelected>(); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
			f.template operator ()<_TySelected>();
#endif // _MSC_VER && !__MWERKS__
			// execute the operation on the selected item

			return f;
		}
	};

public:
	/**
	 *	@brief typelist length stored as enum
	 */
	enum {
		n_list_length = CTypelistLength<CList>::n_result /**< @brief length of the list */
	};

	/**
	 *	@brief selects a typelist item by an index using a binary tree
	 *		(at runtime) and calls a functor on the selected type
	 *
	 *	@param[in] n_index is a zero-based index of item to be selected from the list
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 */
	static inline CFunctor Select(size_t n_index, CFunctor f)
	{
		_ASSERTE(n_index < n_list_length); // makes sure the index is valid
		return CDecisionTree</*CList, CFunctor,*/ 0, n_list_length>::Select(n_index, f);
	}
};

/**
 *	@brief predicate inverse type
 *	@tparam CPredicate is a binary predicate type
 */
template <template <class, class> class CPredicate>
class CPredicateInversion {
public:
	/**
	 *	@brief inverted predicate type
	 *
	 *	@tparam A is the first predicate argument
	 *	@tparam B is the second predicate argument
	 *
	 *	@note This needs to be inside a container since it is an incomplete type and can't
	 *		be passed as a template argument otherwise.
	 */
	template <class A, class B>
	class CInverse {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			b_result = !CPredicate<A, B>::b_result /**< @brief inverted predicate result */
		};
	};
};

/**
 *	@brief predicate for presence of an item in a typelist
 *
 *	@tparam CItemType is an item
 *	@tparam CList is a typelist
 */
template <class CItemType, class CList>
class CTypelistItemPresencePredicate { // unfortunately CFindTypelistItem cannot be directly used as a predicate
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = CFindTypelistItem<CList, CItemType>::b_result /**< @brief predicate result; true if the item was found in the typelist, false otherwise */
	};
};

/**
 *	@brief calculates typelist difference; constructs a typelist with items in the first list
 *		which are not present in the second list
 *
 *	@tparam CListA is the first list
 *	@tparam CListB is the second list
 */
template <class CListA, class CListB>
class CTypelistDifference {
protected:
	//typedef typename CPredicateInversion<CTypelistItemPresencePredicate>::CInverse CPredicate; // cant typedef an incomplete type (in C++11 this could be solved by the using keyword)

public:
	typedef typename CFilterTypelist<CListA, CListB, /*typename*/ // not a type name, it is incomplete!
		CPredicateInversion<CTypelistItemPresencePredicate>::CInverse>::_TyResult _TyResult; /**< @brief the resulting typelist difference */
};

/**
 *	@brief calculates typelist intersection; constructs a typelist with items in the first list
 *		which are also present in the second list
 *
 *	@tparam CListA is the first list
 *	@tparam CListB is the second list
 */
template <class CListA, class CListB>
class CTypelistIntersection {
public:
	typedef typename CFilterTypelist<CListA, CListB, /*typename*/ // not a type name, it is incomplete!
		CTypelistItemPresencePredicate>::_TyResult _TyResult; /**< @brief the resulting typelist intersection */
};

#include <typeinfo>

/**
 *	@brief finds an item in a sorted typelist by a reference value using binary search
 *		(at runtime) and calls a functor on the type found
 *
 *	In this template, CComparator is a class, parametrized by a typelist item
 *	that implements functions b_Equal(CReference) and b_LessThan(CReference)
 *	which compare the typelist item (on the left) to the reference value (on the right)
 *	at runtime.
 *
 *	@tparam CList is a typelist from which an item is being selected
 *	@tparam CComparator is a predicate for comparing two typelist items
 *	@tparam CReference is type of a reference value
 *	@tparam CFunctor is a functor with template function operator to process the selected item
 *
 *	@note This executes operations at run time.
 */
template <class CList, template <class> class CComparator,
	class CReference, class CFunctor>
class CTypelistItemBFind {
public:
	typedef typename CMakeDecisionTreeSkeleton<CList, CComparator, CReference>::_TyRoot _TySkeleton; /**< @brief decision tree skeleton */

	/**
	 *	@brief finds an item in a sorted typelist by a reference value using binary search
	 *		(at runtime) and calls a functor on the selected type, in case the item was not
	 *		found in the list, calls the functor with CTypelistEnd
	 *
	 *	@param[in] reference_value is reference value (a needle) to be found
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 */
	template <class CNode>
	static inline typename CEnableIf<CNode::b_leaf, CFunctor>::T Find(const CReference reference_value, CFunctor f)
	{
		if(CNode::b_Equals_Pivot(reference_value)) { // if "_TyPivot == reference_value"
#if defined(_MSC_VER) && !defined(__MWERKS__)
			f.operator ()<typename CNode::_TyPivot>(); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
			f.template operator ()<typename CNode::_TyPivot>();
#endif // _MSC_VER && !__MWERKS__
			// execute the operation on the selected item
		} else {
#if defined(_MSC_VER) && !defined(__MWERKS__)
			f.operator ()<CTypelistEnd>(); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
			f.template operator ()<CTypelistEnd>();
#endif // _MSC_VER && !__MWERKS__
			// not found, execute the operation on CTypelistEnd
		}

		return f;
	}

	/**
	 *	@brief finds an item in a sorted typelist by a reference value using binary search
	 *		(at runtime) and calls a functor on the selected type, assumes that the reference
	 *		value is always present in the list (generates one level shorter decision tree).
	 *
	 *	@param[in] reference_value is reference value (a needle) to be found
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 */
	template <class CNode>
	static inline typename CEnableIf<CNode::b_leaf, CFunctor>::T FindExisting(const CReference reference_value, CFunctor f)
	{
		_ASSERTE(CNode::b_Equals_Pivot(reference_value)); // if "_TyPivot == reference_value"
#if defined(_MSC_VER) && !defined(__MWERKS__)
		f.operator ()<typename CNode::_TyPivot>(); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
		f.template operator ()<typename CNode::_TyPivot>();
#endif // _MSC_VER && !__MWERKS__
		// execute the operation on the selected item

		return f;
	}

	/**
	 *	@brief finds an item in a sorted typelist by a reference value using binary search
	 *		(at runtime) and calls a functor on the selected type, in case the item was not
	 *		found in the list, calls the functor with CTypelistEnd
	 *
	 *	@param[in] reference_value is reference value (a needle) to be found
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 */
	template <class CNode>
	static inline typename CEnableIf<!CNode::b_leaf, CFunctor>::T Find(const CReference reference_value, CFunctor f)
	{
		if(CNode::b_Left_of_Pivot(reference_value)) // compares "reference_value < _TyPivot"
			return Find<typename CNode::_TyLeft>(reference_value, f);
		else
			return Find<typename CNode::_TyRight>(reference_value, f); // contains pivot
	}

	/**
	 *	@brief finds an item in a sorted typelist by a reference value using binary search
	 *		(at runtime) and calls a functor on the selected type, assumes that the reference
	 *		value is always present in the list (generates one level shorter decision tree).
	 *
	 *	@param[in] reference_value is reference value (a needle) to be found
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 */
	template <class CNode>
	static inline typename CEnableIf<!CNode::b_leaf, CFunctor>::T FindExisting(const CReference reference_value, CFunctor f)
	{
		if(CNode::b_Left_of_Pivot(reference_value)) // compares "reference_value < _TyPivot"
			return FindExisting<typename CNode::_TyLeft>(reference_value, f);
		else
			return FindExisting<typename CNode::_TyRight>(reference_value, f); // contains pivot
	}

public:
	/**
	 *	@brief finds an item in a sorted typelist by a reference value using binary search
	 *		(at runtime) and calls a functor on the selected type
	 *
	 *	In case the item was not found in the list, calls the functor with CTypelistEnd.
	 *
	 *	Note that the decision tree may not be optimal as there is no knowing whether
	 *	there may be more reference values between two pivot values (e.g. if an integer
	 *	is greater than 0 and smaller than 2, it is certainly equal to 1 and there is no
	 *	need to check, but due to generality, this template does not have this information
	 *	so it always has to check).
	 *
	 *	@param[in] reference_value is reference value (a needle) to be found
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 *
	 *	@note This doesn't check if the given list is actually sorted.
	 */
	static inline CFunctor Find(const CReference reference_value, CFunctor f)
	{
		return Find<_TySkeleton>(reference_value, f);
	}

	/**
	 *	@brief finds an item in a sorted typelist by a reference value using binary search
	 *		(at runtime) and calls a functor on the selected type
	 *
	 *	This version assumes that the reference value is always present in the list
	 *	(generates one level shorter decision tree).
	 *
	 *	@param[in] reference_value is reference value (a needle) to be found
	 *	@param[in] f is a functor with template function operator to process the selected item
	 *
	 *	@return Returns the functor after processing the selected item.
	 *
	 *	@note This doesn't check if the given list is actually sorted.
	 */
	static inline CFunctor FindExisting(const CReference reference_value, CFunctor f)
	{
		return FindExisting<_TySkeleton>(reference_value, f);
	}
};

/**
 *	@brief binary search result, to be used with CTypelistItemBFind::Find()
 */
class CBinarySearchResult {
protected:
	bool m_b_found; /**< @brief found flag (raised if the specified item is found) */

public:
	/**
	 *	@brief default constructor; clears the found flag
	 */
	CBinarySearchResult()
		:m_b_found(false)
	{}

	/**
	 *	@brief function operator; raises the found flag if the type is not CTypelistEnd
	 *	@tparam CType is pivot type, passed by CTypelistItemBFind
	 */
	template <class CType>
	void operator ()()
	{
		m_b_found = !CEqualType<CType, CTypelistEnd>::b_result; // or could use function specialization to do the same
	}

	/**
	 *	@brief conversion to bool
	 *	@return Returns the result of a found flag.
	 */
	inline operator bool() const
	{
		return m_b_found;
	}
};

/**
 *	@brief binary search result, to be used with CTypelistItemBFind::Find()
 *	@tparam CList is the typelist being searched through
 */
template <class CList>
class CBinarySearchResultWithIndex {
protected:
	bool m_b_found; /**< @brief found flag (raised if the specified item is found) */
	size_t m_n_index; /**< @brief index of the item found */

public:
	/**
	 *	@brief default constructor; clears the found flag
	 */
	CBinarySearchResultWithIndex()
		:m_b_found(false), m_n_index(size_t(-1))
	{}

	/**
	 *	@brief function operator; raises the found flag and calculates the index if the type is found
	 *	@tparam CType is pivot type, passed by CTypelistItemBFind
	 */
	template <class CType>
	void operator ()()
	{
		typedef CFindTypelistItem<CList, CType> CFindResult;
		m_b_found = CFindResult::b_result;
		m_n_index = CFindResult::n_index;
	}

	/**
	 *	@brief gets index of the item
	 *	@return Returns zero-based index of the item in the typelist, or -1 in case the item was not found.
	 */
	inline size_t n_Index() const
	{
		return m_n_index;
	}

	/**
	 *	@brief conversion to bool
	 *	@return Returns the result of a found flag.
	 */
	inline operator bool() const
	{
		return m_b_found;
	}
};

/**
 *	@brief sequence construction template
 *
 *	This constructs a sequence with a given number of elements.
 *	Each element is constructed from an integer. The integer, associated
 *	with the first element can be adjusted (zero by default), the step
 *	by which the next ones increase can also be specified.
 *
 *	@tparam n_element_num is number of elements of the sequence (must be zero or greater)
 *	@tparam n_first_element is value associated with the first element
 *	@tparam n_step is value of the increment
 *	@tparam CTypeConstructor is construction wrapper (see e.g. \ref fbs_ut::CCTSize_Constructor)
 *
 *	@note See also \ref fbs_ut::CTypelistIOTA.
 */
template <template <const int> class CTypeConstructor,
	const int n_element_num, const int n_first_element = 0, const int n_step = 1>
struct CTypelistIOTA {
protected:
	/**
	 *	@brief sequence construction template
	 *
	 *	@tparam n_remaining_num is the number of additional elements to generate
	 *	@tparam n_head_value is value associated with the first element
	 */
	template <const int n_remaining_num, const int n_head_value>
	struct CIOTA {
		typedef typename CTypeConstructor<n_head_value>::_TyResult CHead; /**< @brief type of the first element of the list */
		typedef typename CIOTA<n_remaining_num - 1, n_head_value + n_step>::_TyResult CTail; /**< @brief the rest of the list */
		typedef CTypelist<CHead, CTail> _TyResult; /**< @brief the resulting list */
	};

	/**
	 *	@brief sequence construction template (specialization for zero-length sequences)
	 *	@tparam n_head_value is value associated with the first element
	 */
	template <const int n_head_value>
	struct CIOTA<0, n_head_value> {
		typedef CTypelistEnd _TyResult; /**< @brief the resulting list */
	};

public:
	typedef typename CIOTA<n_element_num, n_first_element>::_TyResult _TyResult; /**< @brief the resulting list */
};

#endif // !__TYPE_LIST_OMIT_OPS

#ifdef TYPELIST_H_PUSHED_WARNING_STATE
#pragma warning(pop)
#endif // TYPELIST_H_PUSHED_WARNING_STATE

#endif // !__TYPE_LIST_INCLUDED
