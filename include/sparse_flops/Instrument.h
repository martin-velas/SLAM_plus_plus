/*
								+-----------------------------------+
								|                                   |
								|***FLOP counting instrumentation***|
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2016  |
								|                                   |
								|           Instrument.h            |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __FLOP_COUNTING_SCALAR_INCLUDED
#define __FLOP_COUNTING_SCALAR_INCLUDED

/**
 *	@file include/sparse_flops/Instrument.h
 *	@brief FLOP counting instrumentation of scalar types
 *	@date 2016
 *	@author -tHE SWINe-
 */

#include "slam/Integer.h"
#include <math.h>

/**
 *	@brief wrapper around numeric type which counts operations
 *
 *	@tparam CBaseSclar is base scalar type
 *
 *	@note This does not implement conversion operator to the base type, to avoid errors.
 *	@note The in-place operations (e.g. a +=) with base type on the left are not implemented
 *		for the same reason (would most likely lead to not counting some operations).
 *	@note The counters are implemented using OpenMP atomics so this is thread-safe to a
 *		certain degree.
 *	@note The new math functions introduced in C++11 are not wrapped.
 */
template <class CBaseSclar>
class CFLOPCountingScalar {
public:
	typedef CBaseSclar _TyBase; /**< @brief base type */

	typedef void (CFLOPCountingScalar::*Boolean)() const; /**< @brief true value for the safe bool idiom */

	typedef size_t _TyCount; /**< @brief instruction counter data type */

protected:
	_TyBase m_f_value; /**< @brief value */

	static _TyCount m_n_add_num; /**< @brief counter for addition operations */
	static _TyCount m_n_mul_num; /**< @brief counter for multiplication operations */
	static _TyCount m_n_div_num; /**< @brief counter for division operations */
	static _TyCount m_n_trcd_num; /**< @brief counter for transcendental operations */
	static _TyCount m_n_cmp_num; /**< @brief counter for comparison operations */

public:
	/**
	 *	@brief resets the values of all the counters
	 */
	static void Reset_Counters()
	{
		#pragma omp atomic
		m_n_add_num ^= m_n_add_num;
		#pragma omp atomic
		m_n_mul_num ^= m_n_mul_num;
		#pragma omp atomic
		m_n_div_num ^= m_n_div_num;
		#pragma omp atomic
		m_n_trcd_num ^= m_n_trcd_num;
		#pragma omp atomic
		m_n_cmp_num ^= m_n_cmp_num;
	}

	/**
	 *	@brief gets the addition operation counter value
	 *	@return Returns the number of addition operations
	 *		since the last call to \ref Reset_Counters().
	 */
	static inline _TyCount n_Add_Num()
	{
		return m_n_add_num;
	}

	/**
	 *	@brief gets the multiplication operation counter value
	 *	@return Returns the number of multiplication operations
	 *		since the last call to \ref Reset_Counters().
	 */
	static inline _TyCount n_Multiply_Num()
	{
		return m_n_mul_num;
	}

	/**
	 *	@brief gets the division operation counter value
	 *	@return Returns the number of division operations
	 *		since the last call to \ref Reset_Counters().
	 */
	static inline _TyCount n_Divide_Num()
	{
		return m_n_div_num;
	}

	/**
	 *	@brief gets the transcendental operation counter value
	 *	@return Returns the number of transcendental operations
	 *		since the last call to \ref Reset_Counters().
	 */
	static inline _TyCount n_Transcendental_Num()
	{
		return m_n_trcd_num;
	}

	/**
	 *	@brief gets the comparison operation counter value
	 *	@return Returns the number of comparison operations
	 *		since the last call to \ref Reset_Counters().
	 */
	static inline _TyCount n_Comparison_Num()
	{
		return m_n_cmp_num;
	}

	/**
	 *	@brief gets the sum of all operation counter values
	 *	@return Returns the number of (all types of) operations
	 *		since the last call to \ref Reset_Counters().
	 *	@note This sum is equally weighted.
	 */
	static inline _TyCount n_FLOP_Num()
	{
		return m_n_add_num + m_n_mul_num + m_n_div_num + m_n_trcd_num + m_n_cmp_num;
	}

	/**
	 *	@brief default constructor; has no effect
	 */
	CFLOPCountingScalar()
	{}

	/**
	 *	@brief constructor; initializes the value
	 *	@param[in] f_value is value to initialize to
	 */
	CFLOPCountingScalar(_TyBase f_value)
		:m_f_value(f_value)
	{}

	/**
	 *	@brief constructor; initializes to value obtained by a transcendental operation(s)
	 *
	 *	@param[in] f_value is value to initialize to
	 *	@param[in] n_transcendent_operation_num is number of
	 *		transcendent operations it took to obtain the value
	 */
	CFLOPCountingScalar(_TyBase f_value, _TyCount n_transcendent_operation_num)
		:m_f_value(f_value)
	{
		#pragma omp atomic
		m_n_trcd_num += n_transcendent_operation_num;
	}

	/**
	 *	@brief gets the value
	 *	@return Returns the stored value.
	 *	@note Automatic conversion operator to \ref _TyBase is not
	 *		implemented as it would make the debugging much harder.
	 */
	_TyBase f_Value() const
	{
		return m_f_value;
	}

	/**
	 *	@brief gets the value
	 *	@return Returns a reference the stored value.
	 *	@note Automatic conversion operator to \ref _TyBase is not
	 *		implemented as it would make the debugging much harder.
	 */
	_TyBase &f_Value()
	{
		return m_f_value;
	}

	/**
	 *	@brief unary minus operator
	 *	@return Returns the negative value of this.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar operator -() const
	{
		#pragma omp atomic
		++ m_n_add_num;
		return CFLOPCountingScalar(-m_f_value);
	}

	/**
	 *	@brief addition operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the sum of the two values.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar operator +(_TyBase f_x) const
	{
		#pragma omp atomic
		++ m_n_add_num;
		return CFLOPCountingScalar(m_f_value + f_x);
	}

	/**
	 *	@brief subtraction operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the difference of the two values.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar operator -(_TyBase f_x) const
	{
		#pragma omp atomic
		++ m_n_add_num;
		return CFLOPCountingScalar(m_f_value - f_x);
	}

	/**
	 *	@brief multiplication operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the product of the two values.
	 *	@note This increments the multiplication counter.
	 */
	CFLOPCountingScalar operator *(_TyBase f_x) const
	{
		#pragma omp atomic
		++ m_n_mul_num;
		return CFLOPCountingScalar(m_f_value * f_x);
	}

	/**
	 *	@brief division operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the ratio of the two values.
	 *	@note This increments the division counter.
	 */
	CFLOPCountingScalar operator /(_TyBase f_x) const
	{
		#pragma omp atomic
		++ m_n_div_num;
		return CFLOPCountingScalar(m_f_value / f_x);
	}

	/**
	 *	@brief inplace addition operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar &operator +=(_TyBase f_x)
	{
		#pragma omp atomic
		++ m_n_add_num;
		m_f_value += f_x;
		return *this;
	}

	/**
	 *	@brief inplace subtraction operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar &operator -=(_TyBase f_x)
	{
		#pragma omp atomic
		++ m_n_add_num;
		m_f_value -= f_x;
		return *this;
	}

	/**
	 *	@brief inplace multiplication operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the multiplication counter.
	 */
	CFLOPCountingScalar &operator *=(_TyBase f_x)
	{
		#pragma omp atomic
		++ m_n_mul_num;
		m_f_value *= f_x;
		return *this;
	}

	/**
	 *	@brief inplace division operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the division counter.
	 */
	CFLOPCountingScalar &operator /=(_TyBase f_x)
	{
		#pragma omp atomic
		++ m_n_div_num;
		m_f_value /= f_x;
		return *this;
	}

	/**
	 *	@brief addition operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the sum of the two values.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar operator +(CFLOPCountingScalar f_x) const
	{
		#pragma omp atomic
		++ m_n_add_num;
		return CFLOPCountingScalar(m_f_value + f_x.m_f_value);
	}

	/**
	 *	@brief subtraction operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the difference of the two values.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar operator -(CFLOPCountingScalar f_x) const
	{
		#pragma omp atomic
		++ m_n_add_num;
		return CFLOPCountingScalar(m_f_value - f_x.m_f_value);
	}

	/**
	 *	@brief multiplication operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the product of the two values.
	 *	@note This increments the multiplication counter.
	 */
	CFLOPCountingScalar operator *(CFLOPCountingScalar f_x) const
	{
		#pragma omp atomic
		++ m_n_mul_num;
		return CFLOPCountingScalar(m_f_value * f_x.m_f_value);
	}

	/**
	 *	@brief division operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns the ratio of the two values.
	 *	@note This increments the division counter.
	 */
	CFLOPCountingScalar operator /(CFLOPCountingScalar f_x) const
	{
		#pragma omp atomic
		++ m_n_div_num;
		return CFLOPCountingScalar(m_f_value / f_x.m_f_value);
	}

	/**
	 *	@brief inplace addition operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar &operator +=(CFLOPCountingScalar f_x)
	{
		#pragma omp atomic
		++ m_n_add_num;
		m_f_value += f_x.m_f_value;
		return *this;
	}

	/**
	 *	@brief inplace subtraction operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the addition counter.
	 */
	CFLOPCountingScalar &operator -=(CFLOPCountingScalar f_x)
	{
		#pragma omp atomic
		++ m_n_add_num;
		m_f_value -= f_x.m_f_value;
		return *this;
	}

	/**
	 *	@brief inplace multiplication operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the multiplication counter.
	 */
	CFLOPCountingScalar &operator *=(CFLOPCountingScalar f_x)
	{
		#pragma omp atomic
		++ m_n_mul_num;
		m_f_value *= f_x.m_f_value;
		return *this;
	}

	/**
	 *	@brief inplace division operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns reference to this.
	 *	@note This increments the division counter.
	 */
	CFLOPCountingScalar &operator /=(CFLOPCountingScalar f_x)
	{
		#pragma omp atomic
		++ m_n_div_num;
		m_f_value /= f_x.m_f_value;
		return *this;
	}

	/**
	 *	@brief unary negation operator
	 *	@return Returns true if this equals zero, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator !() const
	{
		#pragma omp atomic
		++ m_n_cmp_num;
		return !m_f_value;
	}

	/**
	 *	@brief conversion to bool
	 *
	 *	@return Returns nonzero (not 1) if this does not equal to zero, otherwise returns null.
	 *
	 *	@note This uses the safe bool idiom to avoid mixing expansions in unsafe arithmetic expressions.
	 *	@note This increments the comparison counter.
	 */
	operator Boolean() const
	{
		#pragma omp atomic
		++ m_n_cmp_num;
		return (m_f_value)? &CFLOPCountingScalar::True_Value : 0;
	}

	/**
	 *	@brief less-than operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this equals zero, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator <(CFLOPCountingScalar f_x)
	{
		#pragma omp atomic
		++ m_n_cmp_num;
		return m_f_value < f_x.m_f_value;
	}

	/**
	 *	@brief greater-than operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is greater than \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator >(CFLOPCountingScalar f_x)
	{
		return CFLOPCountingScalar(f_x) < m_f_value;
	}

	/**
	 *	@brief equal-to operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator ==(CFLOPCountingScalar f_x)
	{
		#pragma omp atomic
		++ m_n_cmp_num;
		return m_f_value == f_x.m_f_value;
	}

	/**
	 *	@brief not-equal-to operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is not equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator !=(CFLOPCountingScalar f_x)
	{
		return !(*this == f_x);
	}

	/**
	 *	@brief less-than or equal operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is less than or equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator <=(CFLOPCountingScalar f_x)
	{
		return !(*this > f_x);
	}

	/**
	 *	@brief greater-than or equal operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is greater than or equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator >=(CFLOPCountingScalar f_x)
	{
		return !(*this < f_x);
	}

	/**
	 *	@brief less-than operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this equals zero, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator <(_TyBase f_x)
	{
		#pragma omp atomic
		++ m_n_cmp_num;
		return m_f_value < f_x;
	}

	/**
	 *	@brief greater-than operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is greater than \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator >(_TyBase f_x)
	{
		return CFLOPCountingScalar(f_x) < m_f_value;
	}

	/**
	 *	@brief equal-to operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator ==(_TyBase f_x)
	{
		#pragma omp atomic
		++ m_n_cmp_num;
		return m_f_value == f_x;
	}

	/**
	 *	@brief not-equal-to operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is not equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator !=(_TyBase f_x)
	{
		return !(*this == f_x);
	}

	/**
	 *	@brief less-than or equal operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is less than or equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator <=(_TyBase f_x)
	{
		return !(*this > f_x);
	}

	/**
	 *	@brief greater-than or equal operator
	 *	@param[in] f_x is the value on the right side
	 *	@return Returns true if this is greater than or equal to \ref f_x, otherwise returns false.
	 *	@note This increments the comparison counter.
	 */
	bool operator >=(_TyBase f_x)
	{
		return !(*this < f_x);
	}

protected:
	/**
	 *	@brief value of true for the safe bool idiom
	 */
	void True_Value() const {}
};

/**
 *	@brief addition operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns the sum of the two values.
 *
 *	@note This increments the addition counter.
 */
template <class CBaseSclar>
inline CFLOPCountingScalar<CBaseSclar> operator +(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return CFLOPCountingScalar<CBaseSclar>(f_x) + f_y;
}

/**
 *	@brief subtraction operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns the difference of the two values.
 *
 *	@note This increments the addition counter.
 */
template <class CBaseSclar>
inline CFLOPCountingScalar<CBaseSclar> operator -(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return CFLOPCountingScalar<CBaseSclar>(f_x) - f_y;
}

/**
 *	@brief multiplication operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns the product of the two values.
 *
 *	@note This increments the multiplication counter.
 */
template <class CBaseSclar>
inline CFLOPCountingScalar<CBaseSclar> operator *(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return CFLOPCountingScalar<CBaseSclar>(f_x) * f_y;
}

/**
 *	@brief division operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns the ratio of the two values.
 *
 *	@note This increments the division counter.
 */
template <class CBaseSclar>
inline CFLOPCountingScalar<CBaseSclar> operator /(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return CFLOPCountingScalar<CBaseSclar>(f_x) / f_y;
}

/**
 *	@brief greater-than operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns true if this is greater than \ref f_x, otherwise returns false.
 *	@note This increments the comparison counter.
 */
template <class CBaseSclar>
inline bool operator >(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return f_y < f_x;
}

/**
 *	@brief equal-to operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns true if this is equal to \ref f_x, otherwise returns false.
 *	@note This increments the comparison counter.
 */
template <class CBaseSclar>
inline bool operator ==(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return f_y == f_x;
}

/**
 *	@brief not-equal-to operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns true if this is not equal to \ref f_x, otherwise returns false.
 *	@note This increments the comparison counter.
 */
template <class CBaseSclar>
inline bool operator !=(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return f_y != f_x;
}

/**
 *	@brief less-than or equal operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns true if this is less than or equal to \ref f_x, otherwise returns false.
 *	@note This increments the comparison counter.
 */
template <class CBaseSclar>
inline bool operator <=(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return f_y >= f_x;
}

/**
 *	@brief greater-than or equal operator
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is the value on the left side
 *	@param[in] f_y is the value on the right side
 *
 *	@return Returns true if this is greater than or equal to \ref f_x, otherwise returns false.
 *	@note This increments the comparison counter.
 */
template <class CBaseSclar>
inline bool operator >=(CBaseSclar f_x, CFLOPCountingScalar<CBaseSclar> f_y)
{
	return f_y <= f_x;
}

template <class CBaseSclar>
typename CFLOPCountingScalar<CBaseSclar>::_TyCount CFLOPCountingScalar<CBaseSclar>::m_n_add_num = 0;
template <class CBaseSclar>
typename CFLOPCountingScalar<CBaseSclar>::_TyCount CFLOPCountingScalar<CBaseSclar>::m_n_mul_num = 0;
template <class CBaseSclar>
typename CFLOPCountingScalar<CBaseSclar>::_TyCount CFLOPCountingScalar<CBaseSclar>::m_n_div_num = 0;
template <class CBaseSclar>
typename CFLOPCountingScalar<CBaseSclar>::_TyCount CFLOPCountingScalar<CBaseSclar>::m_n_trcd_num = 0;
template <class CBaseSclar>
typename CFLOPCountingScalar<CBaseSclar>::_TyCount CFLOPCountingScalar<CBaseSclar>::m_n_cmp_num = 0;
// values of the counters

/**
 *	@brief (integer) absolute value function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value to take absolute value of
 *	@return Returns absolute value of \ref f_x.
 *	@note This increments (perhaps not entirely correctly) the transcendental operation counter.
 */
template <class CBaseSclar>
CFLOPCountingScalar<CBaseSclar> abs(CFLOPCountingScalar<CBaseSclar> f_x)
{
	return CFLOPCountingScalar<CBaseSclar>((CBaseSclar)abs(int(f_x.f_Value())), 1);
}

/**
 *	@brief (integer) absolute value function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value to take absolute value of
 *	@return Returns absolute value of \ref f_x.
 *	@note This increments (perhaps not entirely correctly) the transcendental operation counter.
 */
template <class CBaseSclar>
CFLOPCountingScalar<CBaseSclar> labs(CFLOPCountingScalar<CBaseSclar> f_x)
{
	return CFLOPCountingScalar<CBaseSclar>((CBaseSclar)labs(long(f_x.f_Value())), 1);
}

/**
 *	@def DECLARE_UNARY_TRANSCENDENTAL_OP
 *	@brief declares unary transcendental operation
 *	@tparam opname is operation name (e.g. sin)
 */
#define DECLARE_UNARY_TRANSCENDENTAL_OP(opname) \
	template <class CBaseSclar> \
	CFLOPCountingScalar<CBaseSclar> opname(CFLOPCountingScalar<CBaseSclar> f_x) \
	{ \
		return CFLOPCountingScalar<CBaseSclar>((CBaseSclar)opname(double(f_x.f_Value())), 1); /* did one transcendental op */ \
	}

/**
 *	@def DECLARE_BINARY_TRANSCENDENTAL_OP
 *	@brief declares binary transcendental operation
 *	@tparam opname is operation name (e.g. atan2)
 */
#define DECLARE_BINARY_TRANSCENDENTAL_OP(opname) \
	template <class CBaseSclar> \
	CFLOPCountingScalar<CBaseSclar> opname(CFLOPCountingScalar<CBaseSclar> f_x, CFLOPCountingScalar<CBaseSclar> f_y) \
	{ \
		return CFLOPCountingScalar<CBaseSclar>((CBaseSclar)opname(double(f_x.f_Value()), double(f_y.f_Value())), 1); /* did one transcendental op */ \
	}

/**
 *	@brief absolute value function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value to take absolute value of
 *	@return Returns absolute value of \ref f_x.
 *	@note This increments (perhaps not entirely correctly) the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(fabs)

/**
 *	@brief arc-sine function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns arc-sine of \ref f_x, expressed in radians.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(asin)

/**
 *	@brief arc-cosine function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns arc-cosine of \ref f_x, expressed in radians.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(acos)

/**
 *	@brief arc-tangent function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns arc-tangent of \ref f_x, expressed in radians.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(atan)

/**
 *	@brief binary arc-tangent function
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is value of the first input argument
 *	@param[in] f_y is value of the second input argument
 *
 *	@return Returns arc-tangent of \ref f_x / \ref f_y, expressed in radians.
 *
 *	@note This increments the transcendental operation counter.
 */
DECLARE_BINARY_TRANSCENDENTAL_OP(atan2)

/**
 *	@brief cosine function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument, expressed in radians
 *	@return Returns cosine of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(cos)

/**
 *	@brief hyperbolic cosine function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument, expressed in radians
 *	@return Returns hyperbolic cosine of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(cosh)

/**
 *	@brief base-e exponential function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns natural exponent of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(exp)

/**
 *	@brief floating-point modulo function
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is value of the numerator
 *	@param[in] f_y is value of the denominator
 *
 *	@return Returns modulo of \ref f_x / \ref f_y.
 *
 *	@note This increments the transcendental operation counter.
 */
DECLARE_BINARY_TRANSCENDENTAL_OP(fmod)

/**
 *	@brief base-e logarithm function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns natural logarithm of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(log)

/**
 *	@brief base-10 logarithm function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns base-10 logarithm of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(log10)

/**
 *	@brief raise-to-power function
 *
 *	@tparam CBaseSclar is a scalar type
 *
 *	@param[in] f_x is value of the base
 *	@param[in] f_y is value of the exponent
 *
 *	@return Returns \ref f_x to the power of \ref f_y.
 *
 *	@note This increments the transcendental operation counter.
 */
DECLARE_BINARY_TRANSCENDENTAL_OP(pow)

/**
 *	@brief sine function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument, expressed in radians
 *	@return Returns sine of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(sin)

/**
 *	@brief hyperbolic sine function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument, expressed in radians
 *	@return Returns hyperbolic sine of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(sinh)

/**
 *	@brief tangent function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument, expressed in radians
 *	@return Returns tangent of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(tan)

/**
 *	@brief hyperbolic tangent function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument, expressed in radians
 *	@return Returns hyperbolic tangent of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(tanh)

/**
 *	@brief square root function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns square root of \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(sqrt)

/**
 *	@brief round up function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns the closest integer greater than or equal to \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(ceil)

/**
 *	@brief round down function
 *	@tparam CBaseSclar is a scalar type
 *	@param[in] f_x is value of the input argument
 *	@return Returns the closest integer smaller than or equal to \ref f_x.
 *	@note This increments the transcendental operation counter.
 */
DECLARE_UNARY_TRANSCENDENTAL_OP(floor)

typedef CFLOPCountingScalar<float> CFLOPCountingFloat; /**< @brief FLOP-counting float */
typedef CFLOPCountingScalar<double> CFLOPCountingDouble; /**< @brief FLOP-counting double */

/**
 *	@page countingflops Counting FLOPs in Sparse Operations
 *
 *	This example shows how to make use of \ref CFLOPCountingDouble and \ref CTSparse
 *	to count floating point operations (FLOPs) in arbitrary sparse operations. We begin
 *	by including the two necessary files:
 *
 *	@code
 *	#include "sparse_flops/cts.hpp"
 *	#include "sparse_flops/Instrument.h"
 *	@endcode
 *
 *	Now it is possible to typedef a flavor of templated CSparse which will count FLOPs:
 *
 *	@code
 *	typedef CTSparse<CFLOPCountingDouble> CFLOPCountingSparse;
 *	@endcode
 *
 *	A nice thing about this is that \ref CFLOPCountingDouble is a thin wrapper around
 *	`double` and pointers to the two can be converted (so called type punning). This also
 *	allows us to convert between ordinary sparse matrices \ref cs and `CFLOPCountingSparse`:
 *
 *	@code
 *	cs *A = cs_spalloc(...);
 *	CFLOPCountingSparse *A_instrumented = CFLOPCountingSparse::p_FromSparse(A); // there
 *
 *	CFLOPCountingSparse *B_instrumented = CFLOPCountingSparse::spalloc(...);
 *	cs *B = CFLOPCountingSparse::p_ToSparse(B_instrumented); // and back
 *	@endcode
 *
 *	Here, the functions \ref CTSparse::p_FromSparse() and \ref CTSparse::p_ToSparse()
 *	perform the necessary checks to make sure that the conversion is safe, otherwise
 *	such code fails to compile. Note that the pointers `A` and `A_instrumented`
 *	point to the same memory location (equally as `B` and `B_instrumented` do) and
 *	no new memory is allocated.
 *
 *	Now suppose we want to count the real number of FLOPs in a sparse Cholesky
 *	factorization:
 *
 *	@code
 *	size_t n_Chol_FLOP_Num(const cs *A, int order = CFLOPCountingSparse::order_AMD_Chol)
 *	{
 *		CFLOPCountingSparse::css *S = CFLOPCountingSparse::schol(order,
 *			CFLOPCountingSparse::p_FromSparse(A)); // calls AMD
 *
 *		size_t n_before_chol = CFLOPCountingDouble::n_FLOP_Num();
 *
 *		CFLOPCountingSparse::csn *N = CFLOPCountingSparse::chol(CFLOPCountingSparse::p_FromSparse(A), S);
 *
 *		size_t n_flops = CFLOPCountingDouble::n_FLOP_Num() - n_before_chol;
 *
 *	    CFLOPCountingSparse::sfree(S);
 *	    CFLOPCountingSparse::nfree(N);
 *
 *		return n_flops;
 *	}
 *	@endcode
 *
 *	The first line performs symbolic factorization using \ref CTSparse::schol(). Then,
 *	the counters of floating point operations are sampled, using \ref CFLOPCountingScalar::n_FLOP_Num()
 *	and the numeric Cholesky factorization is performed using \ref CTSparse::chol().
 *	After that, the difference in the number of FLOPs is taken. Alternatively, one can call
 *	\ref CFLOPCountingScalar::Reset_Counters() before and then directly read out the number of operations
 *	using \ref CFLOPCountingScalar::n_FLOP_Num().
 *
 *	This has the advantage that it actually calculates the factorization in the process, so it is
 *	fairly easy to instrument existing code this way and it is possible to count FLOPs in iterative
 *	code where the stopping condition depends on the computed values.
 *
 *	The full code of the example follows:
 *
 *	@code
 *	#include "sparse_flops/cts.hpp"
 *	#include "sparse_flops/Instrument.h"
 *
 *	typedef CTSparse<CFLOPCountingDouble> CFLOPCountingSparse;
 *
 *	cs *p_AllocFull(csi m, csi n, double f_value = 1.0)
 *	{
 *		if(n && m > LONG_MAX / n)
 *			return 0; // would overflow below
 *		cs *p_matrix = cs_spalloc(m, n, m * n, 1, 0);
 *		csi n_off = 0;
 *		for(csi i = 0; i < n; ++ i) {
 *			p_matrix->p[i] = n_off;
 *			for(csi j = 0; j < m; ++ j, ++ n_off) {
 *				p_matrix->i[n_off] = j;
 *				p_matrix->x[n_off] = f_value;
 *			}
 *		}
 *		p_matrix->p[n] = n_off;
 *		return p_matrix;
 *	}
 *
 *	cs *p_AllocLower(csi m, csi n, double f_value = 1.0)
 *	{
 *		if(n && m > LONG_MAX / n)
 *			return 0; // would overflow below
 *		size_t n_nnz = std::min(m, n) * (std::min(m, n) - 1) / 2 + std::min(m, n) + // the square triangular section
 *			(m - std::min(m, n)) * n; // the bottom side if the matrix is narrow (completely filled)
 *		cs *p_matrix = cs_spalloc(m, n, n_nnz, 1, 0);
 *		csi n_off = 0;
 *		for(csi i = 0; i < n; ++ i) {
 *			p_matrix->p[i] = n_off;
 *			for(csi j = i; j < m; ++ j, ++ n_off) {
 *				p_matrix->i[n_off] = j;
 *				p_matrix->x[n_off] = f_value;
 *			}
 *		}
 *		p_matrix->p[n] = n_off;
 *		_ASSERTE(n_off == n_nnz);
 *		return p_matrix;
 *	}
 *
 *	size_t n_GEMM_FLOP_Num(const cs *A, const cs *B)
 *	{
 *		size_t n_before = CFLOPCountingDouble::n_FLOP_Num();
 *
 *		cs *p_result = CFLOPCountingSparse::p_ToSparse(CFLOPCountingSparse::multiply(
 *			CFLOPCountingSparse::p_FromSparse(A), CFLOPCountingSparse::p_FromSparse(B)));
 *		cs_spfree(p_result);
 *
 *		return CFLOPCountingDouble::n_FLOP_Num() - n_before;
 *	}
 *
 *	size_t n_GAXPY_FLOP_Num(const cs *A, const double *x, double *y)
 *	{
 *		size_t n_before = CFLOPCountingDouble::n_FLOP_Num();
 *
 *		CFLOPCountingSparse::gaxpy(CFLOPCountingSparse::p_FromSparse(A),
 *			(CFLOPCountingDouble*)x, (CFLOPCountingDouble*)y);
 *
 *		return CFLOPCountingDouble::n_FLOP_Num() - n_before;
 *	}
 *
 *	size_t n_TRSV_FLOP_Num(const cs *L, double *x)
 *	{
 *		size_t n_before = CFLOPCountingDouble::n_FLOP_Num();
 *
 *		CFLOPCountingSparse::lsolve(CFLOPCountingSparse::p_FromSparse(L), (CFLOPCountingDouble*)x);
 *
 *		return CFLOPCountingDouble::n_FLOP_Num() - n_before;
 *	}
 *
 *	size_t n_Chol_FLOP_Num(const cs *A, int order = CFLOPCountingSparse::order_AMD_Chol)
 *	{
 *		CFLOPCountingSparse::css *S = CFLOPCountingSparse::schol(order,
 *			CFLOPCountingSparse::p_FromSparse(A)); // calls AMD
 *
 *		size_t n_before_chol = CFLOPCountingDouble::n_FLOP_Num();
 *
 *		CFLOPCountingSparse::csn *N = CFLOPCountingSparse::chol(CFLOPCountingSparse::p_FromSparse(A), S);
 *
 *		size_t n_flops = CFLOPCountingDouble::n_FLOP_Num() - n_before_chol;
 *
 *		CFLOPCountingSparse::sfree(S);
 *		CFLOPCountingSparse::nfree(N);
 *
 *		return n_flops;
 *	}
 *
 *	size_t n_LU_FLOP_Num(const cs *A, int order = CFLOPCountingSparse::order_AMD_LU)
 *	{
 *		CFLOPCountingSparse::css *S = CFLOPCountingSparse::sqr(order,
 *			CFLOPCountingSparse::p_FromSparse(A), 0); // calls AMD
 *
 *		size_t n_before_chol = CFLOPCountingDouble::n_FLOP_Num();
 *
 *		CFLOPCountingSparse::csn *N = CFLOPCountingSparse::lu(CFLOPCountingSparse::p_FromSparse(A), S, 1e-3);
 *
 *		size_t n_flops = CFLOPCountingDouble::n_FLOP_Num() - n_before_chol;
 *
 *		CFLOPCountingSparse::sfree(S);
 *		CFLOPCountingSparse::nfree(N);
 *
 *		return n_flops;
 *	}
 *
 *	void Test_SparseOpsCost()
 *	{
 *		cs *A = p_AllocFull(100, 100);
 *
 *		printf("counting FLOPs in GEMM of two 100 x 100 matrices\n");
 *		size_t n_GEMM_cost = n_GEMM_FLOP_Num(A, A);
 *		size_t n_GEMM_cost_GT = 100 * 100 * (100 * 2 - 1); // the leading addition is saved
 *		printf("\tground truth FLOPs: " PRIsize "\n", n_GEMM_cost_GT);
 *		printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_GEMM_cost,
 *			(n_GEMM_cost == n_GEMM_cost_GT)? "pass" : "FAIL");
 *
 *		printf("\ncounting FLOPs in GAXPY of a 100 x 100 matrix and a 100 x 1 vector\n");
 *		double x[100] = {0}, y[100] = {0};
 *		size_t n_GAXPY_cost = n_GAXPY_FLOP_Num(A, x, y);
 *		size_t n_GAXPY_cost_GT = 100 * 100 * 2;
 *		printf("\tground truth FLOPs: " PRIsize "\n", n_GAXPY_cost_GT);
 *		printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_GAXPY_cost,
 *			(n_GAXPY_cost == n_GAXPY_cost_GT)? "pass" : "FAIL");
 *
 *		for(int i = 0; i < 100; ++ i)
 *			A->x[i * 100 + i] = 10.0;
 *		// make the diagonal a bit larger in order for the matrix to be positive definite
 *
 *		printf("\ncounting FLOPs in Cholesky of a 100 x 100 matrix\n");
 *		size_t n_Chol_cost = n_Chol_FLOP_Num(A, CFLOPCountingSparse::order_Natural);
 *		size_t n_Chol_cost_GT = 100 * 100 * 100 / 3 + (100 * (100 - 1)) / 2 + 100; // O(n^3/3 + nnz)
 *		printf("\tground truth FLOPs: " PRIsize "\n", n_Chol_cost_GT);
 *		printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_Chol_cost,
 *			(n_Chol_cost == n_Chol_cost_GT)? "pass" :
 *			(fabs(double(n_Chol_cost - n_Chol_cost_GT) / n_Chol_cost_GT) < 1e-3)?
 *			"pass within 0.1 %" : "FAIL"); // up to 0.1% discrepancy allowed
 *
 *		cs *L = p_AllocLower(100, 100);
 *		// get a triangular matrix
 *
 *		printf("\ncounting FLOPs in TRSV of a 100 x 100 lower-triangular matrix and a 100 x 1 vector\n");
 *		size_t n_TRSV_cost = n_TRSV_FLOP_Num(L, x);
 *		size_t n_TRSV_cost_GT = 100 * 100 / 2 * 2;
 *		printf("\tground truth FLOPs: " PRIsize "\n", n_TRSV_cost_GT);
 *		printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_TRSV_cost,
 *			(n_TRSV_cost == n_TRSV_cost_GT)? "pass" : "FAIL");
 *
 *		cs_spfree(A);
 *		cs_spfree(L);
 *	}
 *	@endcode
 *
 *	Note that here, FLOPs is a plural of FLOP. It does not refer to floating point
 *	operations per second (FLOPS with capital `S').
 *
 */

#endif // !__FLOP_COUNTING_SCALAR_INCLUDED
