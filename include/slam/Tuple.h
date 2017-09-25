/*
								+----------------------------------+
								|                                  |
								|    ***   Tuple template   ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|             Tuple.h              |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __TUPLE_INCLUDED
#define __TUPLE_INCLUDED

/**
 *	@file include/slam/Tuple.h
 *	@brief tuple template
 *	@author -tHE SWINe-
 *	@date 2014
 *
 *	@todo - document, add support for tuples of >8 items, add more functionality
 */

#include "slam/TypeList.h"

/**
 *	@brief hidden tuple implementation details
 */
namespace tuple_detail {

/**
 *	@brief base tuple class
 *	@tparam CList is list of tuple element types
 */
template <class CList>
class CTupleBase {
public:
	typedef CList _TyList; /**< @brief list of tuple element types */

	/**
	 *	@brief parameters, stored as enum
	 */
	enum {
		n_length = CTypelistLength<CList>::n_result /**< @brief number of tuple elements */
	};

	typedef typename _TyList::_TyHead THead; /**< @brief type of the first tuple element */
	typedef CTupleBase<typename _TyList::_TyTail> TTail; /**< @brief tuple of all elements, except the first one */

	/**
	 *	@brief element type trait
	 *	@tparam n_index is zero-based element index
	 */
	template <const int n_index>
	struct TElem {
		typedef typename CTypelistItemAt<CList, n_index>::_TyResult Type; /**< @brief element type */
	};

protected:
	THead m_head; /**< @brief value of the first tuple element */
	TTail m_tail; /**< @brief values of the rest of the tuple elements */

public:
	/**
	 *	@brief default constructor
	 */
	inline CTupleBase()
		:m_head(THead())
	{}

	/**
	 *	@brief copy-constructor
	 *	@param[in] r_other is the tuple to copy from
	 */
	inline CTupleBase(const CTupleBase &r_other)
		:m_head(r_other.m_head), m_tail(r_other.m_tail)
	{}

	/**
	 *	@brief constructor
	 *
	 *	@param[in] t_head is value of the first tuple element
	 *	@param[in] r_tail is values of the rest of the tuple elements
	 */
	inline CTupleBase(const THead &t_head, const TTail &r_tail)
		:m_head(t_head), m_tail(r_tail)
	{}

	/**
	 *	@brief copy operator
	 *	@param[in] r_other is the tuple to copy from
	 *	@return Returns reference to this.
	 */
	CTupleBase &operator =(const CTupleBase &r_other)
	{
		m_head = r_other.m_head;
		m_tail = r_other.m_tail; // recurse
		return *this;
	}

	/**
	 *	@brief reverse chaining operator
	 *	@tparam CType is type of the new element
	 *	@param[in] t_value is value of the new element
	 *	@return Returns this with the new item prepended.
	 */
	template <class CType>
	CTupleBase<CTypelist<CType, _TyList> > Prepend(const CType &t_value)
	{
		return CTupleBase<CTypelist<CType, _TyList> >(t_value, *this);
	}

	/**
	 *	@brief item access operator
	 *	@tparam n_index is zero-based index of the tuple item
	 *	@return Returns reference to the specified tuple item.
	 */
	template <const int n_index>
	typename CTypelistItemAt<CList, n_index>::_TyResult &Get() // note that in retrospect, this is not so great, thanks to the ugly g++ "tuple.template Get<X>()" syntax, a global function might work better
	{
		return _Get(fbs_ut::CCTSize<n_index>());
	}

	/**
	 *	@brief item access operator
	 *	@tparam n_index is zero-based index of the tuple item
	 *	@return Returns const reference to the specified tuple item.
	 */
	template <const int n_index>
	typename CTypelistItemAt<CList, n_index>::_TyResult Get() const
	{
		return _Get(fbs_ut::CCTSize<n_index>());
	}

	/**
	 *	@brief equality operator
	 *	@param[in] r_other is other tuple to compare with
	 *	@return Returns true if values of all the tuple items compare equal to those in r_other, otherwise returns false.
	 */
	bool operator ==(const CTupleBase &r_other) const
	{
		return m_head == r_other.m_head && m_tail == r_other.m_tail; // recurse
	}

	/**
	 *	@brief lexicographical order less-than comparison operator
	 *	@param[in] r_other is other tuple to compare with
	 *	@return Returns true if values of all the tuple items compare less than those in r_other,
	 *		in lexicographical order, otherwise returns false.
	 */
	bool operator <(const CTupleBase &r_other) const
	{
		return m_head < r_other.m_head || (m_head == r_other.m_head &&
			m_tail < r_other.m_tail); // recurse
	}

	/**
	 *	@brief swaps two tuples
	 *	@param[in] r_other is reference to the other tuple to swap with
	 */
	void Swap(CTupleBase &r_other)
	{
		std::swap(m_head, r_other.m_head);
		m_tail.Swap(r_other.m_tail);
	}

protected:
	/**
	 *	@brief item access operator (specialization for access in a deeper recursion)
	 *	@tparam n_index is zero-based index of the tuple item, with respect to the current template recursion
	 *	@param[in] s is tag for tag-based specialization (unused)
	 *	@return Returns reference to the specified tuple item.
	 */
	template <const int n_index>
	typename CTypelistItemAt<CList, n_index>::_TyResult
		&_Get(fbs_ut::CCTSize<n_index> UNUSED(s))
	{
		return m_tail.template Get<n_index - 1>(); // m_tail._Get() is protected
	}

	/**
	 *	@brief item access operator (specialization for access in this recursion)
	 *	@tparam n_index is zero-based index of the tuple item, with respect to the current template recursion
	 *	@param[in] s is tag for tag-based specialization (unused)
	 *	@return Returns reference to the specified tuple item.
	 */
	typename CTypelistItemAt<CList, 0>::_TyResult
		&_Get(fbs_ut::CCTSize<0> UNUSED(s))
	{
		return m_head;
	}

	/**
	 *	@brief item access operator (specialization for access in a deeper recursion)
	 *	@tparam n_index is zero-based index of the tuple item, with respect to the current template recursion
	 *	@param[in] s is tag for tag-based specialization (unused)
	 *	@return Returns const reference to the specified tuple item.
	 */
	template <const int n_index>
	typename CTypelistItemAt<CList, n_index>::_TyResult
		_Get(fbs_ut::CCTSize<n_index> UNUSED(s)) const
	{
		return m_tail.template Get<n_index - 1>(); // m_tail._Get() is protected
	}

	/**
	 *	@brief item access operator (specialization for access in this recursion)
	 *	@tparam n_index is zero-based index of the tuple item, with respect to the current template recursion
	 *	@param[in] s is tag for tag-based specialization (unused)
	 *	@return Returns const reference to the specified tuple item.
	 */
	typename CTypelistItemAt<CList, 0>::_TyResult
		_Get(fbs_ut::CCTSize<0> UNUSED(s)) const
	{
		return m_head;
	}
};

/**
 *	@brief base tuple class (specialization for an empty tuple or the last template recursion)
 */
template <>
class CTupleBase<CTypelistEnd> {
public:
	typedef CTypelistEnd _TyList; /**< @brief list of tuple element types */

	/**
	 *	@brief parameters, stored as enum
	 */
	enum {
		n_length = 0 /**< @brief number of tuple elements */
	};

public:
	/**
	 *	@brief default constructor
	 */
	inline CTupleBase()
	{}

	/**
	 *	@brief copy-constructor
	 *	@param[in] r_other is other tuple to copy from (unused)
	 */
	inline CTupleBase(const CTupleBase &UNUSED(r_other))
	{}

	/**
	 *	@brief reverse chaining operator
	 *	@tparam CType is type of the new element
	 *	@param[in] t_value is value of the new element
	 *	@return Returns this with the new item prepended.
	 */
	template <class CType>
	CTupleBase<CTypelist<CType, _TyList> > Prepend(const CType &t_value)
	{
		return CTupleBase<CTypelist<CType, _TyList> >(t_value, *this);
	}

	// no Get() here, as there are no elements

	/**
	 *	@brief copy operator
	 *	@param[in] r_other is the tuple to copy from (unused)
	 *	@return Returns reference to this.
	 */
	CTupleBase &operator =(const CTupleBase &UNUSED(r_other))
	{
		// nothing to copy here
		return *this;
	}

	/**
	 *	@brief equality operator
	 *	@param[in] r_other is other tuple to compare with (unused)
	 *	@return Returns true if values of all the tuple items compare equal to those in r_other, otherwise returns false.
	 */
	bool operator ==(const CTupleBase &UNUSED(r_other)) const
	{
		return true; // empty equals empty
	}

	/**
	 *	@brief lexicographical order less-than comparison operator
	 *	@param[in] r_other is other tuple to compare with (unused)
	 *	@return Returns true if values of all the tuple items compare less than those in r_other,
	 *		in lexicographical order, otherwise returns false.
	 */
	bool operator <(const CTupleBase &UNUSED(r_other)) const
	{
		return false; // empty equals empty
	}

	/**
	 *	@brief swaps two tuples
	 *	@param[in] r_other is reference to the other tuple to swap with (unused)
	 */
	void Swap(CTupleBase &UNUSED(r_other))
	{
		// nothing to swap
	}
};

/**
 *	@brief static assertion helper
 *	@brief b_expression is expression being asserted
 */
template <bool b_expression>
class CStaticAssert {
public:
	typedef void TUPLES_OF_MORE_THAN_8_ELEMENTS_NOT_SUPPORTED; /**< @brief static assertion token; tuples are currently limited by the available constructor specializations */
};

/**
 *	@brief static assertion helper (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

typedef CTupleBase<CTypelistEnd> CEmptyTupleBase; /**< @brief empty base tuple instantiation */

} // ~tuple_detail

/**
 *	@brief tuple implementation
 *	@note The namespace is only a hack to make Doxygen not show the long type names in the class list.
 */
namespace spp_tuple { // not "tuple" to avoid collision with std::tuple

/**
 *	@brief tuple class
 *	@tparam CList is list of tuple element types
 */
template <class CList>
class CTuple : public tuple_detail::CTupleBase<CList> {
protected:
	/**
	 *	@brief parameters, stored as enum
	 */
	enum {
		n_arg_num = CTypelistLength<CList>::n_result, /**< @brief tuple size */
		b_is_too_large = n_arg_num > 8 /**< @brief tuple size check */ // todo - write code to generate tuples up to 16 long
	};

	typedef typename tuple_detail::CStaticAssert<!b_is_too_large>::TUPLES_OF_MORE_THAN_8_ELEMENTS_NOT_SUPPORTED CAssert0; /**< @brief tuple size static assertion */
};

/**
 *	@brief tuple class (specialization for tuples with a single item)
 *	@tparam Type0 is type of the first item
 */
template <class Type0>
class CTuple<MakeTypelist1(Type0)> :
	public tuple_detail::CTupleBase<MakeTypelist1(Type0)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist1(Type0)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *	@param[in] a0 is value of the first item
	 */
	inline CTuple(const Type0 &a0);
};

/**
 *	@brief tuple class (specialization for tuples with 2 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 */
template <class Type0, class Type1>
class CTuple<MakeTypelist2(Type0, Type1)> :
	public tuple_detail::CTupleBase<MakeTypelist2(Type0, Type1)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist2(Type0, Type1)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1);
};

/**
 *	@brief tuple class (specialization for tuples with 3 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 *	@tparam Type2 is type of the third item
 */
template <class Type0, class Type1, class Type2>
class CTuple<MakeTypelist3(Type0, Type1, Type2)> :
	public tuple_detail::CTupleBase<MakeTypelist3(Type0, Type1, Type2)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist3(Type0, Type1, Type2)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 *	@param[in] a2 is value of the third item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2);
};

/**
 *	@brief tuple class (specialization for tuples with 4 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 *	@tparam Type2 is type of the third item
 *	@tparam Type3 is type of the 4th item
 */
template <class Type0, class Type1, class Type2, class Type3>
class CTuple<MakeTypelist4(Type0, Type1, Type2, Type3)> :
	public tuple_detail::CTupleBase<MakeTypelist4(Type0, Type1, Type2, Type3)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist4(Type0, Type1, Type2, Type3)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 *	@param[in] a2 is value of the third item
	 *	@param[in] a3 is value of the 4th item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2, const Type3 &a3);
};

/**
 *	@brief tuple class (specialization for tuples with 5 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 *	@tparam Type2 is type of the third item
 *	@tparam Type3 is type of the 4th item
 *	@tparam Type4 is type of the 5th item
 */
template <class Type0, class Type1, class Type2, class Type3, class Type4>
class CTuple<MakeTypelist5(Type0, Type1, Type2, Type3, Type4)> :
	public tuple_detail::CTupleBase<MakeTypelist5(Type0, Type1, Type2, Type3, Type4)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist5(Type0, Type1, Type2, Type3, Type4)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 *	@param[in] a2 is value of the third item
	 *	@param[in] a3 is value of the 4th item
	 *	@param[in] a4 is value of the 5th item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1,
		const Type2 &a2, const Type3 &a3, const Type4 &a4);
};

/**
 *	@brief tuple class (specialization for tuples with 6 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 *	@tparam Type2 is type of the third item
 *	@tparam Type3 is type of the 4th item
 *	@tparam Type4 is type of the 5th item
 *	@tparam Type5 is type of the 6th item
 */
template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5>
class CTuple<MakeTypelist6(Type0, Type1, Type2, Type3, Type4, Type5)> :
	public tuple_detail::CTupleBase<MakeTypelist6(Type0, Type1, Type2, Type3, Type4, Type5)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist6(Type0, Type1, Type2, Type3, Type4, Type5)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 *	@param[in] a2 is value of the third item
	 *	@param[in] a3 is value of the 4th item
	 *	@param[in] a4 is value of the 5th item
	 *	@param[in] a5 is value of the 6th item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2,
		const Type3 &a3, const Type4 &a4, const Type5 &a5);
};

/**
 *	@brief tuple class (specialization for tuples with 7 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 *	@tparam Type2 is type of the third item
 *	@tparam Type3 is type of the 4th item
 *	@tparam Type4 is type of the 5th item
 *	@tparam Type5 is type of the 6th item
 *	@tparam Type6 is type of the 7th item
 */
template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5, class Type6>
class CTuple<MakeTypelist7(Type0, Type1, Type2, Type3, Type4, Type5, Type6)> :
	public tuple_detail::CTupleBase<MakeTypelist7(Type0, Type1, Type2, Type3, Type4, Type5, Type6)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist7(Type0, Type1, Type2, Type3, Type4, Type5, Type6)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 *	@param[in] a2 is value of the third item
	 *	@param[in] a3 is value of the 4th item
	 *	@param[in] a4 is value of the 5th item
	 *	@param[in] a5 is value of the 6th item
	 *	@param[in] a6 is value of the 7th item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2,
		const Type3 &a3, const Type4 &a4, const Type5 &a5, const Type6 &a6);
};

/**
 *	@brief tuple class (specialization for tuples with 8 items)
 *
 *	@tparam Type0 is type of the first item
 *	@tparam Type1 is type of the second item
 *	@tparam Type2 is type of the third item
 *	@tparam Type3 is type of the 4th item
 *	@tparam Type4 is type of the 5th item
 *	@tparam Type5 is type of the 6th item
 *	@tparam Type6 is type of the 7th item
 *	@tparam Type7 is type of the 8th item
 */
template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5, class Type6, class Type7>
class CTuple<MakeTypelist8(Type0, Type1, Type2, Type3, Type4, Type5, Type6, Type7)> :
	public tuple_detail::CTupleBase<MakeTypelist8(Type0, Type1, Type2, Type3, Type4, Type5, Type6, Type7)> {
protected:
	typedef tuple_detail::CTupleBase<MakeTypelist8(Type0, Type1, Type2, Type3, Type4, Type5, Type6, Type7)> _TyBase; /**< @brief base class type */

public:
	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase()
	 */
	inline CTuple()
	{}

	/**
	 *	@copydoc tuple_detail::CTupleBase::CTupleBase(const CTupleBase&)
	 */
	inline CTuple(const _TyBase &r_other)
		:_TyBase(r_other)
	{}

	/**
	 *	@brief constructor; initializes the tuple from item values
	 *
	 *	@param[in] a0 is value of the first item
	 *	@param[in] a1 is value of the second item
	 *	@param[in] a2 is value of the third item
	 *	@param[in] a3 is value of the 4th item
	 *	@param[in] a4 is value of the 5th item
	 *	@param[in] a5 is value of the 6th item
	 *	@param[in] a6 is value of the 7th item
	 *	@param[in] a7 is value of the 8th item
	 */
	inline CTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2, const Type3 &a3,
		const Type4 &a4, const Type5 &a5, const Type6 &a6, const Type7 &a7);
};

/**
 *	@brief utility function; builds a tuple from values
 *	@param[in] a0 is value of the first item
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0>
inline CTuple<MakeTypelist1(Type0)> MakeTuple(const Type0 &a0)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1>
inline CTuple<MakeTypelist2(Type0, Type1)> MakeTuple(const Type0 &a0, const Type1 &a1)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a1).Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *	@param[in] a2 is value of the third item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1, class Type2>
inline CTuple<MakeTypelist3(Type0, Type1, Type2)> MakeTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a2).Prepend(a1).Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *	@param[in] a2 is value of the third item
 *	@param[in] a3 is value of the 4th item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1, class Type2, class Type3>
inline CTuple<MakeTypelist4(Type0, Type1, Type2, Type3)>
	MakeTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2, const Type3 &a3)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a3).Prepend(a2).Prepend(a1).Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *	@param[in] a2 is value of the third item
 *	@param[in] a3 is value of the 4th item
 *	@param[in] a4 is value of the 5th item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1, class Type2, class Type3, class Type4>
inline CTuple<MakeTypelist5(Type0, Type1, Type2, Type3, Type4)>
	MakeTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2, const Type3 &a3, const Type4 &a4)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a4).Prepend(a3).
		Prepend(a2).Prepend(a1).Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *	@param[in] a2 is value of the third item
 *	@param[in] a3 is value of the 4th item
 *	@param[in] a4 is value of the 5th item
 *	@param[in] a5 is value of the 6th item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5>
inline CTuple<MakeTypelist6(Type0, Type1, Type2, Type3, Type4, Type5)>
	MakeTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2,
	const Type3 &a3, const Type4 &a4, const Type5 &a5)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a5).Prepend(a4).
		Prepend(a3).Prepend(a2).Prepend(a1).Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *	@param[in] a2 is value of the third item
 *	@param[in] a3 is value of the 4th item
 *	@param[in] a4 is value of the 5th item
 *	@param[in] a5 is value of the 6th item
 *	@param[in] a6 is value of the 7th item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1, class Type2, class Type3,
	class Type4, class Type5, class Type6>
inline CTuple<MakeTypelist7(Type0, Type1, Type2, Type3, Type4, Type5, Type6)>
	MakeTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2, const Type3 &a3,
	const Type4 &a4, const Type5 &a5, const Type6 &a6)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a6).Prepend(a5).
		Prepend(a4).Prepend(a3).Prepend(a2).Prepend(a1).Prepend(a0);
}

/**
 *	@brief utility function; builds a tuple from values
 *
 *	@param[in] a0 is value of the first item
 *	@param[in] a1 is value of the second item
 *	@param[in] a2 is value of the third item
 *	@param[in] a3 is value of the 4th item
 *	@param[in] a4 is value of the 5th item
 *	@param[in] a5 is value of the 6th item
 *	@param[in] a6 is value of the 7th item
 *	@param[in] a7 is value of the 8th item
 *
 *	@return Returns the tuple initialized from the argument values.
 */
template <class Type0, class Type1, class Type2, class Type3,
	class Type4, class Type5, class Type6, class Type7>
inline CTuple<MakeTypelist8(Type0, Type1, Type2, Type3, Type4, Type5, Type6, Type7)>
	MakeTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2, const Type3 &a3,
	const Type4 &a4, const Type5 &a5, const Type6 &a6, const Type7 &a7)
{
	return tuple_detail::CEmptyTupleBase().Prepend(a7).Prepend(a6).
		Prepend(a5).Prepend(a4).Prepend(a3).Prepend(a2).Prepend(a1).Prepend(a0);
}

template <class Type0>
inline CTuple<MakeTypelist1(Type0)>::CTuple(const Type0 &a0)
	:_TyBase(MakeTuple(a0))
{}

template <class Type0, class Type1>
inline CTuple<MakeTypelist2(Type0, Type1)>::CTuple(const Type0 &a0, const Type1 &a1)
	:_TyBase(MakeTuple(a0, a1))
{}

template <class Type0, class Type1, class Type2>
inline CTuple<MakeTypelist3(Type0, Type1, Type2)>::CTuple(const Type0 &a0, const Type1 &a1, const Type2 &a2)
	:_TyBase(MakeTuple(a0, a1, a2))
{}

template <class Type0, class Type1, class Type2, class Type3>
inline CTuple<MakeTypelist4(Type0, Type1, Type2, Type3)>::CTuple(const Type0 &a0,
	const Type1 &a1, const Type2 &a2, const Type3 &a3)
	:_TyBase(MakeTuple(a0, a1, a2, a3))
{}

template <class Type0, class Type1, class Type2, class Type3, class Type4>
inline CTuple<MakeTypelist5(Type0, Type1, Type2, Type3, Type4)>::CTuple(const Type0 &a0,
	const Type1 &a1, const Type2 &a2, const Type3 &a3, const Type4 &a4)
	:_TyBase(MakeTuple(a0, a1, a2, a3, a4))
{}

template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5>
inline CTuple<MakeTypelist6(Type0, Type1, Type2, Type3, Type4, Type5)>::CTuple(const Type0 &a0,
	const Type1 &a1, const Type2 &a2, const Type3 &a3, const Type4 &a4, const Type5 &a5)
	:_TyBase(MakeTuple(a0, a1, a2, a3, a4, a5))
{}

template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5, class Type6>
inline CTuple<MakeTypelist7(Type0, Type1, Type2, Type3, Type4, Type5, Type6)>::CTuple(const Type0 &a0,
	const Type1 &a1, const Type2 &a2, const Type3 &a3, const Type4 &a4, const Type5 &a5, const Type6 &a6)
	:_TyBase(MakeTuple(a0, a1, a2, a3, a4, a5, a6))
{}

template <class Type0, class Type1, class Type2, class Type3, class Type4, class Type5, class Type6, class Type7>
inline CTuple<MakeTypelist8(Type0, Type1, Type2, Type3, Type4, Type5, Type6, Type7)>::CTuple(const Type0 &a0,
	const Type1 &a1, const Type2 &a2, const Type3 &a3, const Type4 &a4, const Type5 &a5, const Type6 &a6, const Type7 &a7)
	:_TyBase(MakeTuple(a0, a1, a2, a3, a4, a5, a6, a7))
{}

/**
 *	@brief gets type of a tuple element
 *
 *	@tparam _TyTuple is instanitation of CTuple
 *	@tparam n_index is zero-based index of the element
 */
template <class _TyTuple, const int n_index>
class CTupleElement {
public:
	typedef typename CTypelistItemAt<typename _TyTuple::_TyList, n_index>::_TyResult _TyResult; /**< @brief type of the selected tuple element */
};

} // ~spp_tuple

using namespace spp_tuple; // the namespace is only a hack to make Doxygen not show the long type names in the class list

// inject a specialization of swap() in the std namespace
namespace std {

/**
 *	@brief swaps two tuples
 *
 *	@tparam CList is list of items
 *
 *	@param[in] r_a is the first tuple
 *	@param[in] r_b is the second tuple
 */
template <class CList>
void swap(CTuple<CList> &r_a, CTuple<CList> &r_b)
{
	r_a.Swap(r_b);
}

/**
 *	@brief swaps two tuples
 *
 *	@tparam CList is list of items
 *
 *	@param[in] r_a is the first tuple
 *	@param[in] r_b is the second tuple
 */
template <class CList>
void swap(tuple_detail::CTupleBase<CList> &r_a, tuple_detail::CTupleBase<CList> &r_b) // not sure if this is needed but better be safe
{
	r_a.Swap(r_b);
}

}

#endif // !__TUPLE_INCLUDED
