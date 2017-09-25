/*
								+-----------------------------------+
								|                                   |
								| ***  _TySelf type declarator  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2014  |
								|                                   |
								|              Self.h               |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SELF_TYPE_DECLARATION_INCLUDED
#define __SELF_TYPE_DECLARATION_INCLUDED

/**
 *	@file include/slam/Self.h
 *	@brief _TySelf type declarator and utilities for consistency checking
 *	@author -tHE SWINe-
 *	@date 2014-01-16
 *
 *	@date 2014-03-27
 *
 *	Rewritten using a more elegant staric assert method. Otherwise remains the same.
 */

//#include "slam/Integer.h" // included from slam/TypeList.h
//#include "slam/Unused.h" // included from slam/TypeList.h
#include "slam/TypeList.h"

/**
 *	@brief namespace for checking the _TySelf type consistency
 */
namespace self_detail {

#if 0
/**
 *	@brief compile-time assertion (_TySelf must be declared the same as the type of class)
 *
 *	@tparam _TySelf is reported self type
 *	@tparam _TyDecltypeThis is type of <tt>*this</tt>
 */
template <class _TySelf, class _TyDecltypeThis>
class CSELF_TYPE_MUST_BE_THE_SAME_AS_CLASS_TYPE;

/**
 *	@brief compile-time assertion (specialization for assertion passing)
 *	@tparam _TySelf is reported self type (same as type of <tt>*this</tt>)
 */
template <class _TySelf>
class CSELF_TYPE_MUST_BE_THE_SAME_AS_CLASS_TYPE<_TySelf, _TySelf> {};

/**
 *	@brief compile-time assertion (_TySelf must be declared the same as the type of class)
 *	@tparam b_check is the asserted value
 */
template <bool b_check>
class CSELF_TYPE_MUST_BE_THE_SAME_AS_CLASS_TYPE2;

/**
 *	@brief compile-time assertion (specialization for assertion passing)
 */
template <>
class CSELF_TYPE_MUST_BE_THE_SAME_AS_CLASS_TYPE2<true> {};

/**
 *	@brief static assertion helper type
 *	@tparam n_size is size of object being used as assertion message
 *		(if it's a incomplete type, compiler will display object name in error output)
 */
template <const size_t n_size>
class CStaticAssert {};

/**
 *	@brief helper function for self-check, this is used to derive type of this
 *		in absence of <tt>decltype()</tt> in older versions of C++
 *
 *	@tparam _TyA is reported self type
 *	@tparam _TyB is type of <tt>*this</tt>
 *
 *	@param[in] p_this is pointer to the class (unused, only for deducing the type)
 */
template <class _TyA, class _TyB>
inline void __self_check_helper(_TyB *UNUSED(p_this))
{
	typedef CStaticAssert<sizeof(CSELF_TYPE_MUST_BE_THE_SAME_AS_CLASS_TYPE<_TyA, _TyB>)> _TyAssert;
	// make sure that the type reported as self and type of *this is the same
}

/**
 *	@def __SELF_CHECK
 *	@brief declares the body of __self_check() function
 */
#define __SELF_CHECK \
	/** checks the consistency of _TySelf type (calling it has no effect) */ \
	inline void __self_check() \
	{ \
		self_detail::__self_check_helper<_TySelf>(this); \
	}

/**
 *	@def DECLARE_SELF
 *	@brief declares _TySelf type and adds code to make sure that it is indeed a correct one
 *	@param[in] Type is type of the enclosing class
 */
#define DECLARE_SELF(Type) \
	typedef Type _TySelf; /**< @brief type of this class */ \
	__SELF_CHECK \
	/** @brief a part of check of _TySelf type for templates */ \
	enum { __static_self_check_token = __LINE__ /**< @brief pseudounique self-check token */ }; \
	typedef self_detail::CStaticAssert<sizeof(self_detail::CSELF_TYPE_MUST_BE_THE_SAME_AS_CLASS_TYPE2< \
		__static_self_check_token == _TySelf::__static_self_check_token>)> \
		__static_self_check /**< @brief check of _TySelf type for templates */
#else // 0

/**
 *	@brief static assertion helper type
 *	@tparam b_expression is the expression being assertede
 */
template <const bool b_expression>
class CStaticAssert {
public:
	typedef void SelfType_MustBeTheSameAs_ClassType; /**< @brief assertion tag for self type check */
};

/**
 *	@brief static assertion helper type (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {
};

/**
 *	@brief helper function for self-check, this is used to derive type of this
 *		in absence of <tt>decltype()</tt> in older versions of C++
 *
 *	@tparam _TyA is reported self type
 *	@tparam _TyB is type of <tt>*this</tt>
 *
 *	@param[in] p_this is pointer to the class (unused, only for deducing the type)
 */
template <class _TyA, class _TyB>
inline void __self_check_helper(_TyB *UNUSED(p_this))
{
	typedef CStaticAssert<(CEqualType<_TyA, _TyB>::b_result)> _TyAssert;
	// make sure that the type reported as self and type of *this is the same
}

/**
 *	@def __SELF_CHECK
 *	@brief declares the body of __self_check() function
 */
#define __SELF_CHECK \
	/** checks the consistency of _TySelf type (calling it has no effect) */ \
	inline void __self_check() \
	{ \
		self_detail::__self_check_helper<_TySelf>(this); \
	}

/**
 *	@def DECLARE_SELF
 *	@brief declares _TySelf type and adds code to make sure that it is indeed a correct one
 *	@param[in] Type is type of the enclosing class
 */
#define DECLARE_SELF(Type) \
	typedef Type _TySelf; /**< @brief type of this class */ \
	__SELF_CHECK \
	/** @brief a part of check of _TySelf type for templates */ \
	enum { __static_self_check_token = __LINE__ /**< @brief pseudounique self-check token */ }; \
	typedef self_detail::CStaticAssert<__static_self_check_token == \
		_TySelf::__static_self_check_token>::SelfType_MustBeTheSameAs_ClassType \
		__static_self_check /**< @brief check of _TySelf type for templates */

#endif // 0

} // ~self_detail

#endif // !__SELF_TYPE_DECLARATION_INCLUDED
