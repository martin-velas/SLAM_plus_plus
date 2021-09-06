/*
								+----------------------------------+
								|                                  |
								|  ***  Polynomial templates  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|           Polynomial.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __POLYNOMIAL_FUNCTIONS_INCLUDED
#define __POLYNOMIAL_FUNCTIONS_INCLUDED

/**
 *	@file geometry/Polynomial.h
 *	@brief polynomial functions with (numerical) inverse, analytic derivatives and least squares fitting
 *	@author -tHE SWINe-
 *	@date 2016
 */

#include "Eigen/Core"
#include "Eigen/LU"
#include "slam/Integer.h"
#include <vector>
#include <algorithm>
#include <numeric> // accumulate
#include "geometry/RobustLoss.h"

/**
 *	@brief namespace containing helper functions for efficient polynomial derivatives evaluation
 */
namespace polynomial_detail { // todo - move this below

template <class CScalar>
class CArgAdaptor {
protected:
	CScalar m_f_arg;

public:
	CArgAdaptor(CScalar f_arg)
		:m_f_arg(f_arg)
	{}

	inline CScalar f_Arg() const
	{
		return m_f_arg;
	}

	inline CScalar f_SquaredArg() const
	{
		return m_f_arg * m_f_arg;
	}
};

template <class CScalar>
class CSquaredArgAdaptor {
protected:
	CScalar m_f_squared_arg;

public:
	CSquaredArgAdaptor(CScalar f_squared_arg)
		:m_f_squared_arg(f_squared_arg)
	{}

	inline CScalar f_Arg() const
	{
		return CScalar(sqrt(double(m_f_squared_arg)));
	}

	inline CScalar f_SquaredArg() const
	{
		return m_f_squared_arg;
	}
};

template <class CScalar>
inline CArgAdaptor<CScalar> PlainArg(CScalar f_arg)
{
	return CArgAdaptor<CScalar>(f_arg);
}

template <class CScalar>
inline CSquaredArgAdaptor<CScalar> SquaredArg(CScalar f_squared_arg)
{
	return CSquaredArgAdaptor<CScalar>(f_squared_arg);
}

/**
 *	@brief calculates integer power of a number
 *
 *	@tparam n_power is (signed) power to raise the base to
 *
 *	@param[in] x is value of the base
 *	@param[in] f_factor is the initial value (default 1.0 for zero-th power)
 *
 *	@return Returns \f$\text{f\_factor} \cdot x^{\text{n\_power}}\f$.
 *
 *	@note In case n_power is zero then this just returns f_factor (and likely inlines, yielding a no-op).
 */
template <int n_power, class CScalar>
static CScalar f_Power(CScalar x, CScalar f_factor = 1)
{
	if(n_power < 0) // compile-time branch (handle only negative powers; zero handled easier without recursion)
		return f_factor / f_Power<-n_power>(x);
	// handle negative powers correctly (can occur in derivatives computation)

	for(unsigned int i = n_power; i; i >>= 1, x *= x) { // unroll
		if(i & 1)
			f_factor *= x;
	}
	return f_factor;
}

template <class CScalar, class CVector, int n_first_multiplier,
	int n_multiplier_step, int n_first_coeff_power, int n_power_step, class CLazyArg>
inline CVector v_VecKernel(CLazyArg t_arg)
{
	CVector v_deriv;
	{
		enum {
			b_need_first_power = (((n_first_coeff_power >= 0)? n_first_coeff_power :
				-n_first_coeff_power) & 1) != 0 || (n_power_step & 1) != 0,
			n_first_pow = (b_need_first_power)? n_first_coeff_power : n_first_coeff_power / 2,
			n_pow_step = (b_need_first_power)? n_power_step : n_power_step / 2,
			n_low_pow = (n_first_pow < n_pow_step)? n_first_pow : n_pow_step,
			n_pow_diff = (n_first_pow + n_pow_step - n_low_pow) - n_low_pow,
			n_coeff_num = CVector::RowsAtCompileTime
		};
		// may save a sqrt(), known at compile-time

		CScalar r = (b_need_first_power)? t_arg.f_Arg() : t_arg.f_SquaredArg();
		// base

		CScalar f_low = f_Power<n_low_pow>(r);
		CScalar f_high = f_Power<n_pow_diff>(r, f_low);
		// low and high power (may be equal)

		CScalar f_power = (n_first_pow < n_pow_step)? f_low : f_high;
		CScalar f_step = (n_first_pow < n_pow_step)? f_high : f_low;
		// get the first coeff power and power step

		int n_multiplier = n_first_multiplier;
		for(int i = 0; i < n_coeff_num; ++ i, f_power *= f_step, n_multiplier += n_multiplier_step) // unroll
			v_deriv(i) = n_multiplier * f_power; // correctly handles the 0th power derivatives to be 0
		// fill the vector
	}

	return v_deriv;
}

template <class CScalar, int n_first_multiplier, int n_multiplier_step,
	int n_first_coeff_power, int n_power_step, class CVector, class CLazyArg>
inline CScalar f_DotKernel(const CVector &r_weights, CLazyArg t_arg)
{
	CScalar f_dot = 0;
	{
		enum {
			b_need_first_power = (((n_first_coeff_power >= 0)? n_first_coeff_power :
				-n_first_coeff_power) & 1) != 0 || (n_power_step & 1) != 0,
			n_first_pow = (b_need_first_power)? n_first_coeff_power : n_first_coeff_power / 2,
			n_pow_step = (b_need_first_power)? n_power_step : n_power_step / 2,
			n_low_pow = (n_first_pow < n_pow_step)? n_first_pow : n_pow_step,
			n_pow_diff = (n_first_pow + n_pow_step - n_low_pow) - n_low_pow,
			n_coeff_num = CVector::RowsAtCompileTime
		};
		// may save a sqrt(), known at compile-time

		CScalar r = (b_need_first_power)? t_arg.f_Arg() : t_arg.f_SquaredArg();
		// base

		CScalar f_low = f_Power<n_low_pow>(r);
		CScalar f_high = f_Power<n_pow_diff>(r, f_low);
		// low and high power (may be equal)

		CScalar f_power = (n_first_pow < n_pow_step)? f_low : f_high;
		CScalar f_step = (n_first_pow < n_pow_step)? f_high : f_low;
		// get the first coeff power and power step

		int n_multiplier = n_first_multiplier;
		for(int i = 0; i < n_coeff_num; ++ i, f_power *= f_step, n_multiplier += n_multiplier_step) // unroll
			f_dot += n_multiplier * r_weights(i) * f_power; // correctly handles the 0th power derivatives to be 0
		// accumulate the dot product
	}

	return f_dot;
}

/**
 *	@brief static assertion template
 *	@tparam b_expression is the expression being asserted
 */
template <bool b_expression>
class CStaticAssert {
public:
	typedef void POLYNOMIAL_POWER_STEP_MUST_BE_POSITIVE; /**< @brief static assertion tag */
};

/**
 *	@brief static assertion template (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

/**
 *	@brief a simple pass-through data adaptor
 */
struct TPassThrough {
	/**
	 *	@brief data conversion operation
	 *	@tparam X is data type (should be a 2D vector)
	 *	@param[in] x is input datum
	 *	@return Returns the value of the input.
	 */
	template <class X>
	inline X operator ()(X x) const
	{
		return x;
	}
};

} // ~polynomial_detail

/**
 *	@brief a simple polynomial class
 *
 *	@tparam n_coeff_num is number of distortion coeffifient
 *	@tparam n_first_coeff_power is power of the first coefficient
 *	@tparam n_power_step is difference of powers of adjancent coefficients
 *	@tparam CScalar is scalar data type for the evaluation of the polynomial
 */
template <const int n_coeff_num, const int n_first_coeff_power = 1,
	const int n_power_step = 1, class CScalar = double>
class CPolynomial { // t_odo - implement the 0th term correctly, if needed
public:
	/**
	 *	@brief model parameters, stored as enum
	 */
	enum {
		coeff_Num = n_coeff_num, /**< @brief number of coefficients */
		firstCoeff_Power = n_first_coeff_power, /**< @brief only even powers */
		power_Step = (n_coeff_num)? n_power_step : 0 /**< @brief only even or only odd powers */
	};

	typedef CScalar _TyScalar; /**< @brief distortion paramteters */
	typedef Eigen::Matrix<_TyScalar, coeff_Num, 1> _TyVector; /**< @brief a vector type of the same dimension as the polynomial */
	typedef Eigen::Matrix<_TyScalar, coeff_Num, coeff_Num> _TyMatrix; /**< @brief a square matrix type of the same dimension as the polynomial */
	typedef _TyVector _TyParams; /**< @brief distortion paramteters */

	typedef Eigen::Matrix<_TyScalar, 2, 1> Vector2; /**< @brief 2D vector type (used in least squares fitting) */
	typedef Eigen::Matrix<_TyScalar, 2, 1, Eigen::DontAlign> Vector2_u; /**< @brief unaligned 2D vector type (used in least squares fitting) */

	/**
	 *	@brief a simple data adaptor that performs scalar cast
	 */
	struct TCastAdaptor {
		/**
		 *	@brief data conversion operation
		 *	@tparam X is data type (should be a 2D vector)
		 *	@param[in] x is input datum
		 *	@return Returns the value of the input, with the type of vector
		 *		components cast to match the scalar type used by this polynomial.
		 */
		template <class X>
		inline Vector2 operator ()(X x) const
		{
			return x.template cast<_TyScalar>();
		}
	};

protected:
	typedef typename polynomial_detail::CStaticAssert<!(power_Step < 0)>::POLYNOMIAL_POWER_STEP_MUST_BE_POSITIVE CAssert0; /**< @brief static assertion about poly powers */

protected:
	_TyParams m_v_coeffs; /**< @brief vector of coefficients */

public:
	/**
	 *	@brief default constructor; initializes the polynomial coefficients
	 *	@param[in] r_parameters is vector of the polynomial coefficients (default a null vector)
	 */
	inline CPolynomial(const _TyParams &r_parameters = _TyParams::Zero()) // must be a reference, since it can't be aligned in msvc
		:m_v_coeffs(r_parameters)
	{}

	/**
	 *	@brief constructor; initializes the only polynomial coefficient
	 *	@param[in] f_parameter is value of the only polynomial coefficient
	 */
	inline CPolynomial(_TyScalar f_parameter)
	{
		m_v_coeffs(0) = f_parameter;
		_ASSERTE(n_coeff_num == 1);
	}

	/**
	 *	@brief access to distortion parameters
	 *	@return Returns reference to the parameter vector.
	 */
	_TyParams &v_Parameters()
	{
		return m_v_coeffs;
	}

	/**
	 *	@brief access to distortion parameters
	 *	@return Returns const reference to the parameter vector.
	 */
	const _TyParams &v_Parameters() const
	{
		return m_v_coeffs;
	}

	/**
	 *	@brief evaluates the polynomial
	 *	@param[in] f_x is value of the polynomial argument
	 *	@return Returns the value of the polynomial, at the specified argument.
	 */
	_TyScalar operator ()(_TyScalar f_x) const
	{
		return m_v_coeffs.dot(v_PolyWeights(f_x));
	}

	/**
	 *	@brief calculates a vector of polynomial weights, useful for evaluating the polynomial
	 *
	 *	Calculates \f$W\f$ for which \f$f(x) = \text{dot}(W, K)\f$, where
	 *	\f$f(x)\f$ is this polynomial and \f$K\f$ are the polynomial coefficients.
	 *
	 *	@param[in] f_x is value of the polynomial argument
	 *	@return Returns a vector of argument powers, matching the polynomial coefficients.
	 *	@note This is coincidentially the same as \f$D\f$ in the derivative w.r.t. the coefficients
	 *		\f$\frac{d f(x)}{d K} = \text{dot}(D, K)\f$ where \f$f(x)\f$ is this polynomial
	 *		and \f$K\f$ are the coefficients.
	 */
	inline _TyParams v_PolyWeights(_TyScalar f_x) const
	{
		return polynomial_detail::v_VecKernel<_TyScalar, _TyParams, 1, 0, firstCoeff_Power, power_Step>(
			polynomial_detail::PlainArg(f_x));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p, [K0, K1, K2]) = [x^2, x^4, x^6]
		// f(x) = dot([x^2, x^4, x^6], [K0, K1, K2])
	}

	/**
	 *	@brief calculates a vector of polynomial weights, useful for evaluating the polynomial
	 *
	 *	Calculates \f$W\f$ for which \f$f(\sqrt{x^2}) = \text{dot}(W, K)\f$, where
	 *	\f$f(x)\f$ is this polynomial and \f$K\f$ are the polynomial coefficients.
	 *
	 *	@param[in] f_x2 is value of the polynomial argument squared
	 *
	 *	@return Returns a vector of argument powers, matching the polynomial coefficients.
	 *
	 *	@note This is coincidentially the same as \f$D\f$ in the derivative w.r.t. the coefficients
	 *		\f$\frac{d f(x)}{d K} = \text{dot}(D, K)\f$ where \f$f(x)\f$ is this polynomial
	 *		and \f$K\f$ are the coefficients.
	 *	@note This is the same as v_PolyWeights() but optimized for squared input (as e.g. in the
	 *		case of 2D distortion functions).
	 */
	inline _TyParams v_PolyWeights_SquaredInput(_TyScalar f_x2) const
	{
		return polynomial_detail::v_VecKernel<_TyScalar, _TyParams, 1, 0, firstCoeff_Power, power_Step>(
			polynomial_detail::SquaredArg(f_x2));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p, [K0, K1, K2]) = [x^2, x^4, x^6]
		// f(x) = dot([x^2, x^4, x^6], [K0, K1, K2])
	}

	/**
	 *	@brief calculates derivative of the polynomial w.r.t. the argument
	 *
	 *	Calculates \f$D\f$ for which \f$\frac{d x\cdot f(x)}{d x} = \text{dot}(D, K)\f$, where
	 *	\f$f(x)\f$ is this polynomial and \f$K\f$ are the polynomial coefficients.
	 *
	 *	@param[in] f_x is value of the polynomial argument
	 *	@return Returns a vector of derivative components, matching the polynomial coefficients.
	 *		The derivative equals dot product of the returned vector and the polynomial coefficients.
	 *	@note This is a derivative with respect to the argument, rather than with
	 *		respect to the weights.
	 */
	inline _TyParams v_Derivative_PolyWeights(_TyScalar f_x) const
	{
		return polynomial_detail::v_VecKernel<_TyScalar, _TyParams,
			firstCoeff_Power, power_Step, firstCoeff_Power - 1, power_Step>(
			polynomial_detail::PlainArg(f_x));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p, x) = 2*K0*x+4*K1*x^3+6*K2*x^5 = dot([2x, 4x^3, 6x^5], [K0, K1, K2])
		// and this is returned as a vector [2x, 4x^3, 6x^5]
		// so jacobian(p, x) = dot(v_PolyWeights_Derivative(), v_Parameters());
	}

	/**
	 *	@brief calculates derivative of the polynomial w.r.t. the argument
	 *
	 *	Calculates \f$D\f$ for which \f$\frac{d x\cdot f(\sqrt{x^2})}{d x} = \text{dot}(D, K)\f$,
	 *	where \f$f(x)\f$ is this polynomial and \f$K\f$ are the polynomial coefficients.
	 *
	 *	@param[in] f_x2 is value of the polynomial argument squared
	 *
	 *	@return Returns a vector of derivative components, matching the polynomial coefficients.
	 *		The derivative equals dot product of the returned vector and the polynomial coefficients.
	 *
	 *	@note This is a derivative with respect to the argument, rather than with
	 *		respect to the weights.
	 *	@note This is the same as v_Derivative_PolyWeights() but optimized for squared input
	 *		(as e.g. in the case of 2D distortion functions).
	 */
	inline _TyParams v_Derivative_PolyWeights_SquaredInput(_TyScalar f_x2) const
	{
		return polynomial_detail::v_VecKernel<_TyScalar, _TyParams,
			firstCoeff_Power, power_Step, firstCoeff_Power - 1, power_Step>(
			polynomial_detail::SquaredArg(f_x2));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p, x) = 2*K0*x+4*K1*x^3+6*K2*x^5 = dot([2x, 4x^3, 6x^5], [K0, K1, K2])
		// and this is returned as a vector [2x, 4x^3, 6x^5]
		// so jacobian(p, x) = dot(v_PolyWeights_Derivative(), v_Parameters());
	}

	/**
	 *	@brief calculates derivative of the polynomial times the argument w.r.t. the argument
	 *
	 *	Calculates \f$D\f$ for which \f$\frac{d x\cdot f(x)}{d x} = \text{dot}(D, K)\f$, where
	 *	\f$f(x)\f$ is this polynomial and \f$K\f$ are the polynomial coefficients.
	 *
	 *	@param[in] f_x is value of the polynomial argument
	 *	@return Returns a vector of derivative components, matching the polynomial coefficients.
	 *		The derivative equals dot product of the returned vector and the polynomial coefficients.
	 *	@note This is a derivative with respect to the argument, rather than with
	 *		respect to the weights.
	 */
	inline _TyParams v_Poly_times_Arg_Derivative_PolyWeights(_TyScalar f_x) const
	{
		return polynomial_detail::v_VecKernel<_TyScalar, _TyParams,
			firstCoeff_Power + 1, power_Step, firstCoeff_Power, power_Step>(
			polynomial_detail::PlainArg(f_x));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p * x, x) = 3*K0*x^2+5*K1*x^4+7*K2*x^6 = dot([3x^2, 5x^4, 7x^6], [K0, K1, K2])
		// and this is returned as a vector [3x^2, 5x^4, 7x^6]
		// so jacobian(p * x, x) = dot(v_Poly_times_Arg_Derivative_PolyWeights(x), v_Parameters());
	}

	/**
	 *	@brief calculates derivative of the polynomial times the argument w.r.t. the argument
	 *
	 *	Calculates \f$D\f$ for which \f$\frac{d \sqrt{x^2}\cdot f(\sqrt{x^2})}{d x} = \text{dot}(D, K)\f$,
	 *	where \f$f(x)\f$ is this polynomial and \f$K\f$ are the polynomial coefficients.
	 *
	 *	@param[in] f_x2 is value of the polynomial argument squared
	 *
	 *	@return Returns a vector of derivative components, matching the polynomial coefficients.
	 *		The derivative equals dot product of the returned vector and the polynomial coefficients.
	 *
	 *	@note This is a derivative with respect to the argument, rather than with
	 *		respect to the weights.
	 *	@note This is the same as v_Derivative_PolyWeights() but optimized for squared input
	 *		(as e.g. in the case of 2D distortion functions).
	 */
	inline _TyParams v_Poly_times_Arg_Derivative_PolyWeights_SquaredInput(_TyScalar f_x2) const
	{
		return polynomial_detail::v_VecKernel<_TyScalar, _TyParams,
			firstCoeff_Power + 1, power_Step, firstCoeff_Power, power_Step>(
			polynomial_detail::SquaredArg(f_x2));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p * x, x) = 3*K0*x^2+5*K1*x^4+7*K2*x^6 = dot([3x^2, 5x^4, 7x^6], [K0, K1, K2])
		// and this is returned as a vector [3x^2, 5x^4, 7x^6]
		// so jacobian(p * x, x) = dot(v_Poly_times_Arg_Derivative_PolyWeights(x), v_Parameters());
	}

	/**
	 *	@brief calculates derivative of the polynomial w.r.t. the argument
	 *	@return Returns the (scalar) value of the derivative \f$\frac{d f(x)}{d x}\f$
	 *		where \f$f(x)\f$ is this polynomial.
	 *	@note This function returns <tt>this->v_PolyWeights_Derivative(f_x).dot(this->v_Parameters())</tt>.
	 */
	inline _TyScalar f_Derivative(_TyScalar f_x) const
	{
		return polynomial_detail::f_DotKernel<_TyScalar,
			firstCoeff_Power, power_Step, firstCoeff_Power - 1, power_Step>(
			m_v_coeffs, polynomial_detail::PlainArg(f_x));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p, x) = 2*K0*x+4*K1*x^3+6*K2*x^5
	}

	/**
	 *	@brief calculates derivative of the polynomial w.r.t. the argument
	 *
	 *	@return Returns the (scalar) value of the derivative \f$\frac{d f(\sqrt{x^2})}{d x}\f$
	 *		where \f$f(x)\f$ is this polynomial.
	 *
	 *	@note This function returns <tt>this->v_PolyWeights_Derivative_SquaredInput(f_x2).dot(this->v_Parameters())</tt>.
	 *	@note This is the same as f_Derivative() but optimized for squared input
	 *		(as e.g. in the case of 2D distortion functions).
	 */
	inline _TyScalar f_Derivative_SquaredInput(_TyScalar f_x2) const
	{
		return polynomial_detail::f_DotKernel<_TyScalar,
			firstCoeff_Power, power_Step, firstCoeff_Power - 1, power_Step>(
			m_v_coeffs, polynomial_detail::SquaredArg(f_x2));
		// p = K0*x^2+K1*x^4+K2*x^6
		// jacobian(p, x) = 2*K0*x+4*K1*x^3+6*K2*x^5
	}

	/**
	 *	@brief calculates coefficients for this polynomial as a linear
	 *		least squares fit to a set of observations
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 */
	inline void LeastSquares_Fit(const std::vector<Vector2_u> &r_obs)
	{
		LeastSquares_Fit(r_obs, polynomial_detail::TPassThrough());
	}

	/**
	 *	@brief calculates coefficients for this polynomial as a weighted linear
	 *		least squares fit to set of observations
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] r_weights is a vector of observation weights
	 */
	inline void LeastSquares_Fit(const std::vector<Vector2_u> &r_obs,
		const std::vector<_TyScalar> &r_weights)
	{
		LeastSquares_Fit(r_obs, polynomial_detail::TPassThrough(), r_weights);
	}

	/**
	 *	@brief calculates coefficients for this polynomial as a linear
	 *		least squares fit to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 */
	template <class CObservations, class CObsAdaptor>
	void LeastSquares_Fit(const CObservations &r_obs, CObsAdaptor adaptor)
	{
		_ASSERTE(!r_obs.empty()); // otherwise will produce NaNs (as sort of can be expected)

		_TyVector v_rhs = _TyVector::Zero();
		_TyMatrix hessian = _TyMatrix::Zero();

		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it) {
			Vector2 v_obs = adaptor(*p_obs_it);
			const _TyScalar x = v_obs(0), y = v_obs(1);
			// get an observation

			//_TyVector v_derivative = v_Derivative_PolyWeights(x); // calculate derivative in x
			_TyVector v_derivative = v_PolyWeights(x); // calculate derivative w.r.t. poly coeffs (not w.r.t. x)!
			//_TyScalar r = y - (*this)(x); // calculate residual in x

			v_rhs += v_derivative * y; // rhs = A^Ty
			hessian += v_derivative * v_derivative.transpose(); // h = A^TA
			// accumulate rhs and the hessian
		}

		Eigen::PartialPivLU<_TyMatrix> LU(hessian);
		m_v_coeffs = LU.solve(v_rhs); // beta = (A^TA)^(-1)A^Ty
		// solve the linear system using LU decomposition
	}

	/**
	 *	@brief calculates coefficients for this polynomial as a weighted linear
	 *		least squares fit to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] r_weights is a vector of observation weights
	 */
	template <class CObservations, class CObsAdaptor>
	void LeastSquares_Fit(const CObservations &r_obs, CObsAdaptor adaptor,
		const std::vector<_TyScalar> &r_weights)
	{
		_ASSERTE(!r_obs.empty()); // otherwise will produce NaNs (as sort of can be expected)
		_ASSERTE(r_weights.size() == r_obs.size());

		_TyVector v_rhs = _TyVector::Zero();
		_TyMatrix hessian = _TyMatrix::Zero();

		typename std::vector<_TyScalar>::const_iterator p_weight_it = r_weights.begin();
		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it, ++ p_weight_it) {
			Vector2 v_obs = adaptor(*p_obs_it);
			const _TyScalar x = v_obs(0), y = v_obs(1);
			// get an observation

			const _TyScalar w = *p_weight_it;
			// get weight

			//_TyVector v_derivative = v_Derivative_PolyWeights(x); // calculate derivative in x
			_TyVector v_derivative = v_PolyWeights(x); // calculate derivative w.r.t. poly coeffs (not w.r.t. x)!
			//_TyScalar r = y - (*this)(x); // calculate residual in x

			v_rhs += v_derivative * w * y; // rhs = A^TWy
			hessian += v_derivative * w * v_derivative.transpose(); // h = A^TWA
			// accumulate rhs and the hessian
		}

		Eigen::PartialPivLU<_TyMatrix> LU(hessian);
		m_v_coeffs = LU.solve(v_rhs); // beta = (A^TWA)^(-1)A^TWy
		// solve the linear system using LU decomposition
	}

	/**
	 *	@brief calculates the error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@return Returns the RMSE error.
	 */
	_TyScalar f_LeastSquares_Error(const std::vector<Vector2_u> &r_obs)
	{
		return f_LeastSquares_Error(r_obs, polynomial_detail::TPassThrough());
	}

	/**
	 *	@brief calculates the error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *
	 *	@return Returns the RMSE error.
	 */
	template <class CObservations, class CObsAdaptor>
	_TyScalar f_LeastSquares_Error(const CObservations &r_obs, CObsAdaptor adaptor)
	{
		if(r_obs.empty())
			return 0;
		_TyScalar f_error = 0;
		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it) {
			Vector2 v_obs = adaptor(*p_obs_it); // get an observation
			_TyScalar r = v_obs(1) - (*this)(v_obs(0)); // calculate residual in x
			f_error += r * r;
		}
		return sqrt(f_error / r_obs.size());
	}

	/**
	 *	@brief calculates the error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] r_weights is a vector of observation weights
	 *
	 *	@return Returns the RMSE error.
	 */
	_TyScalar f_LeastSquares_Error(const std::vector<Vector2_u> &r_obs,
		const std::vector<_TyScalar> &r_weights)
	{
		return f_LeastSquares_Error(r_obs, polynomial_detail::TPassThrough(), r_weights);
	}

	/**
	 *	@brief calculates the weighted error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] r_weights is a vector of observation weights
	 *
	 *	@return Returns the weighted RMSE error.
	 */
	template <class CObservations, class CObsAdaptor>
	_TyScalar f_LeastSquares_Error(const CObservations &r_obs, CObsAdaptor adaptor,
		const std::vector<_TyScalar> &r_weights)
	{
		_ASSERTE(r_obs.size() == r_weights.size());
		if(r_obs.empty())
			return 0;
		_TyScalar f_error = 0, f_weight_sum = 0;
		typename std::vector<_TyScalar>::const_iterator p_weight_it = r_weights.begin();
		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it, ++ p_weight_it) {
			Vector2 v_obs = adaptor(*p_obs_it); // get an observation
			const _TyScalar w = *p_weight_it; // get weight
			_TyScalar r = v_obs(1) - (*this)(v_obs(0)); // calculate residual in x
			f_error += r * r * w;
			f_weight_sum += w;
		}
		return sqrt(f_error / f_weight_sum);
	}

	/**
	 *	@brief refines the coefficients of this polynomial using iteratively
	 *		reweigzted least squares fit to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] n_max_iteration_num is maximum number of NLS iterations (default 25)
	 *	@param[in] f_dx_thresh is minimum update norm threshold to continue iteration
	 *		(default \f$10^{-6}\f$)
	 *	@param[in] f_nls_damping is NLS damping parameter (default 0)
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@note This NLS method requires initialization, e.g. using
	 *		least squares or an initial guess (dead reckoning).
	 */
	template <class CObservations, class CScoreFunction>
	inline void Robust_Fit_Update(const CObservations &r_obs,
		CScoreFunction score_function, int n_max_iteration_num = 25,
		_TyScalar f_dx_thresh = _TyScalar(1e-6), _TyScalar f_nls_damping = _TyScalar(0),
		_TyScalar f_mad_scale = _TyScalar(1.4826)) // throw(std::bad_alloc)
	{
		Robust_Fit_Update(r_obs, polynomial_detail::TPassThrough(), score_function,
			n_max_iteration_num, f_dx_thresh, f_nls_damping, f_mad_scale);
	}

	/**
	 *	@brief refines the coefficients of this polynomial using iteratively
	 *		reweigzted least squares fit to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] r_weights is a vector of observation weights (multiplicative with robust weights)
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] n_max_iteration_num is maximum number of NLS iterations (default 25)
	 *	@param[in] f_dx_thresh is minimum update norm threshold to continue iteration
	 *		(default \f$10^{-6}\f$)
	 *	@param[in] f_nls_damping is NLS damping parameter (default 0)
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@note This NLS method requires initialization, e.g. using
	 *		least squares or an initial guess (dead reckoning).
	 */
	template <class CObservations, class CScoreFunction>
	inline void Robust_Fit_Update(const CObservations &r_obs, const std::vector<_TyScalar> &r_weights,
		CScoreFunction score_function, int n_max_iteration_num = 25,
		_TyScalar f_dx_thresh = _TyScalar(1e-6), _TyScalar f_nls_damping = _TyScalar(0),
		_TyScalar f_mad_scale = _TyScalar(1.4826)) // throw(std::bad_alloc)
	{
		Robust_Fit_Update(r_obs, polynomial_detail::TPassThrough(), r_weights, score_function,
			n_max_iteration_num, f_dx_thresh, f_nls_damping, f_mad_scale);
	}

	/**
	 *	@brief refines the coefficients of this polynomial using iteratively
	 *		reweigzted least squares fit to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] n_max_iteration_num is maximum number of NLS iterations (default 25)
	 *	@param[in] f_dx_thresh is minimum update norm threshold to continue iteration
	 *		(default \f$10^{-6}\f$)
	 *	@param[in] f_nls_damping is NLS damping parameter (default 0)
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@note This NLS method requires initialization, e.g. using
	 *		least squares or an initial guess (dead reckoning).
	 */
	template <class CObservations, class CObsAdaptor, class CScoreFunction>
	void Robust_Fit_Update(const CObservations &r_obs, CObsAdaptor adaptor,
		CScoreFunction score_function, int n_max_iteration_num = 25,
		_TyScalar f_dx_thresh = _TyScalar(1e-6), _TyScalar f_nls_damping = _TyScalar(0),
		_TyScalar f_mad_scale = _TyScalar(1.4826)) // throw(std::bad_alloc)
	{
		_ASSERTE(!r_obs.empty()); // otherwise will fail

		/*for(int i = 1; i < n_coeff_num; ++ i)
			m_v_coeffs(i) = 0;
		m_v_coeffs(0) = .5f;*/
		// can try to reset the polynomial but we'll assume we already made a linear fit as a starting point

		std::vector<_TyScalar> MAD;
		for(int n_iter = 0; n_iter < n_max_iteration_num; ++ n_iter) {
			const _TyScalar f_MAD = f_MedianAbsoluteDeviation(r_obs, adaptor, MAD);
			// calculate MAD (median absolute deviation)

			const _TyScalar s = (f_mad_scale)? f_MAD * f_mad_scale : _TyScalar(1);
			// calculate scaling (the constant gets the median to match normal distribution in some special cases)
			// note that this sometimes makes it less robusit in case the scale is known a priori

			_TyVector v_rhs = _TyVector::Zero();
			_TyMatrix hessian = _TyMatrix::Zero();
			hessian.diagonal().setConstant(f_nls_damping);

			for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
			   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it) {
				Vector2 v_obs = adaptor(*p_obs_it);
				// get a measurement

				//_TyVector v_derivative = v_Derivative_PolyWeights(v_obs(0)); // calculate derivative in x
				_TyVector v_derivative = v_PolyWeights(v_obs(0)); // calculate derivative w.r.t. poly coeffs (not w.r.t. x)!
				// calculate derivative in x (note that the derivative w.r.t. the error is negative)

				_TyScalar f_err = v_obs(1) - m_v_coeffs.dot(v_derivative);//(*this)(v_obs(0)); // reuse calculation
				// calculate an error (not squared!)

				_TyScalar w = (f_err)? score_function(fabs(f_err / s)) : 1; // otherwise the weight is NaN
				// calculate the robust weight vector (change 1)
				// note that if the error was multidimensional, norm of error is used both to calculate s, and to calculate the MAD
				// it is not clear how to best mix multiple robust functions. calculate s for each separately? probably. ceres doesnt seem to calculate s at all
				// there is an issue with the edges not having indices and therefore calculating s in parallel is difficult
				// another issue is that the calculate jacobians function needs to be modified if a robust function is used
				// if the robust function is a part of the solver, then there is a new "robust solver", which takes care of all the issues with the parameters and would be fast but less versatile and requires new types of solvers (or a new solver parameter)
				// if the robust function is a part of the edge, then it needs a new interface to get the s, not difficult to implement but requires to have "different edges" for robust functions. there is an issue where to put the functions (must be somewhere, must not change address)
				// is it possible to make a robust wrapper above the edge? no, would have to duplicate constructors.

				// for least squares, w = 1

				v_rhs += v_derivative * w * f_err; // the rhs is weighted (change 2)
				hessian += v_derivative * w * v_derivative.transpose();
				// accumulate JTWJ (change 3)
			}
			// build the information matrix and the residual vector

			Eigen::FullPivLU<_TyMatrix> LU(hessian);
			_TyVector v_dx = LU.solve(v_rhs);
			// solve the linear system using LU decomposition

			m_v_coeffs += v_dx;
			_TyScalar f_norm = v_dx.norm();
			if(f_norm < f_dx_thresh)
				break;
			// update the solution, provide early exit
		}
		// NLS iterations
	}

	/**
	 *	@brief refines the coefficients of this polynomial using iteratively
	 *		reweigzted least squares fit to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] r_weights is a vector of observation weights (multiplicative with robust weights)
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] n_max_iteration_num is maximum number of NLS iterations (default 25)
	 *	@param[in] f_dx_thresh is minimum update norm threshold to continue iteration
	 *		(default \f$10^{-6}\f$)
	 *	@param[in] f_nls_damping is NLS damping parameter (default 0)
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@note This NLS method requires initialization, e.g. using
	 *		least squares or an initial guess (dead reckoning).
	 */
	template <class CObservations, class CObsAdaptor, class CScoreFunction>
	void Robust_Fit_Update(const CObservations &r_obs, CObsAdaptor adaptor,
		const std::vector<_TyScalar> &r_weights, CScoreFunction score_function, int n_max_iteration_num = 25,
		_TyScalar f_dx_thresh = _TyScalar(1e-6), _TyScalar f_nls_damping = _TyScalar(0),
		_TyScalar f_mad_scale = _TyScalar(1.4826)) // throw(std::bad_alloc)
	{
		_ASSERTE(!r_obs.empty()); // otherwise will fail
		_ASSERTE(r_weights.size() == r_obs.size());

		/*for(int i = 1; i < n_coeff_num; ++ i)
			m_v_coeffs(i) = 0;
		m_v_coeffs(0) = .5f;*/
		// can try to reset the polynomial but we'll assume we already made a linear fit as a starting point

		const _TyScalar f_weight_sum = std::accumulate(r_weights.begin(), r_weights.end(), _TyScalar(0));
		// sum the weights

		std::vector<std::pair<_TyScalar, _TyScalar> > MAD;
		for(int n_iter = 0; n_iter < n_max_iteration_num; ++ n_iter) {
			//const _TyScalar f_MAD = f_MedianAbsoluteDeviation(r_obs, adaptor, MAD); // should this be weighted too?
			const _TyScalar f_MAD = f_MedianAbsoluteDeviation(r_obs, adaptor, r_weights, f_weight_sum, MAD); // yes
			// calculate MAD (median absolute deviation)

			const _TyScalar s = (f_mad_scale)? f_MAD * f_mad_scale : _TyScalar(1);
			// calculate scaling (the constant gets the median to match normal distribution in some special cases)
			// note that this sometimes makes it less robusit in case the scale is known a priori

			_TyVector v_rhs = _TyVector::Zero();
			_TyMatrix hessian = _TyMatrix::Zero();
			hessian.diagonal().setConstant(f_nls_damping);

			typename std::vector<_TyScalar>::const_iterator p_weight_it = r_weights.begin();
			for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
			   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it, ++ p_weight_it) {
				Vector2 v_obs = adaptor(*p_obs_it);
				// get a measurement

				_TyScalar f_obs_w = *p_weight_it;
				// get the corresponding weight

				//_TyVector v_derivative = v_Derivative_PolyWeights(v_obs(0)); // calculate derivative in x
				_TyVector v_derivative = v_PolyWeights(v_obs(0)); // calculate derivative w.r.t. poly coeffs (not w.r.t. x)!
				// calculate derivative in x (note that the derivative w.r.t. the error is negative)

				_TyScalar f_err = v_obs(1) - m_v_coeffs.dot(v_derivative);//(*this)(v_obs(0)); // reuse calculation
				// calculate an error (not squared!)

				_TyScalar w = (f_err)? score_function(fabs(f_err / s)) : 1; // otherwise the weight is NaN
				// calculate the robust weight vector (change 1)
				// note that if the error was multidimensional, norm of error is used both to calculate s, and to calculate the MAD
				// it is not clear how to best mix multiple robust functions. calculate s for each separately? probably. ceres doesnt seem to calculate s at all
				// there is an issue with the edges not having indices and therefore calculating s in parallel is difficult
				// another issue is that the calculate jacobians function needs to be modified if a robust function is used
				// if the robust function is a part of the solver, then there is a new "robust solver", which takes care of all the issues with the parameters and would be fast but less versatile and requires new types of solvers (or a new solver parameter)
				// if the robust function is a part of the edge, then it needs a new interface to get the s, not difficult to implement but requires to have "different edges" for robust functions. there is an issue where to put the functions (must be somewhere, must not change address)
				// is it possible to make a robust wrapper above the edge? no, would have to duplicate constructors.

				// for least squares, w = 1

				w *= f_obs_w;
				// combine with observation weights

				v_rhs += v_derivative * w * f_err; // the rhs is weighted (change 2)
				hessian += v_derivative * w * v_derivative.transpose();
				// accumulate JTWJ (change 3)
			}
			// build the information matrix and the residual vector

			Eigen::FullPivLU<_TyMatrix> LU(hessian);
			_TyVector v_dx = LU.solve(v_rhs);
			// solve the linear system using LU decomposition

			m_v_coeffs += v_dx;
			_TyScalar f_norm = v_dx.norm();
			if(f_norm < f_dx_thresh)
				break;
			// update the solution, provide early exit
		}
		// NLS iterations
	}

	/**
	 *	@brief calculates the robustified error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@return Returns the RMSE error weighted by robust weights.
	 */
	template <class CScoreFunction>
	_TyScalar f_LeastSquares_RobustError(const std::vector<Vector2_u> &r_obs,
		CScoreFunction score_function, _TyScalar f_mad_scale = _TyScalar(1.4826))
	{
		return f_LeastSquares_RobustError(r_obs, polynomial_detail::TPassThrough(),
			score_function, f_mad_scale);
	}

	/**
	 *	@brief calculates the robustified error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@return Returns the RMSE error weighted by robust weights.
	 */
	template <class CObservations, class CObsAdaptor, class CScoreFunction>
	_TyScalar f_LeastSquares_RobustError(const CObservations &r_obs, CObsAdaptor adaptor,
		CScoreFunction score_function, _TyScalar f_mad_scale = _TyScalar(1.4826))
	{
		_TyScalar f_error = 0, f_weight_sum = 0;

		std::vector<_TyScalar> MAD;
		const _TyScalar f_MAD = f_MedianAbsoluteDeviation(r_obs, adaptor, MAD);
		const _TyScalar s = (f_mad_scale)? f_MAD * f_mad_scale : _TyScalar(1);
		// calculate MAD (median absolute deviation)

		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it) {
			Vector2 v_obs = adaptor(*p_obs_it);
			const _TyScalar x = v_obs(0), y = v_obs(1);
			// get an observation

			_TyScalar f_err = y - (*this)(x); // calculate residual in x

			_TyScalar w = (f_err)? score_function(fabs(f_err / s)) : 1; // otherwise the weight is NaN
			// calculate the robust weight vector

			f_error += f_err * f_err * w;
			f_weight_sum += w;
		}

		return sqrt(f_error / f_weight_sum);
	}

	/**
	 *	@brief calculates the robustified error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] r_weights is a vector of observation weights (multiplicative with robust weights)
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@return Returns the RMSE error weighted by robust weights.
	 */
	template <class CScoreFunction>
	_TyScalar f_LeastSquares_RobustError(const std::vector<Vector2_u> &r_obs,
		const std::vector<_TyScalar> &r_weights, CScoreFunction score_function,
		_TyScalar f_mad_scale = _TyScalar(1.4826))
	{
		return f_LeastSquares_RobustError(r_obs, polynomial_detail::TPassThrough(),
			r_weights, score_function, f_mad_scale);
	}

	/**
	 *	@brief calculates the robustified error of the fitted model (this
	 *		polynomial) compared to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *	@tparam CScoreFunction is robust kernel type, e.g. \ref CHuberLoss
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] r_weights is a vector of observation weights (multiplicative with robust weights)
	 *	@param[in] score_function is robust kernel instance
	 *	@param[in] f_mad_scale is MAD scale (default 1.4826)
	 *
	 *	@return Returns the RMSE error weighted by robust weights.
	 */
	template <class CObservations, class CObsAdaptor, class CScoreFunction>
	_TyScalar f_LeastSquares_RobustError(const CObservations &r_obs, CObsAdaptor adaptor,
		const std::vector<_TyScalar> &r_weights, CScoreFunction score_function,
		_TyScalar f_mad_scale = _TyScalar(1.4826))
	{
		_TyScalar f_error = 0, f_weight_sum = 0;

		std::vector<std::pair<_TyScalar, _TyScalar> > MAD;
		//const _TyScalar f_MAD = f_MedianAbsoluteDeviation(r_obs, adaptor, MAD); // should this be weighted too?
		const _TyScalar f_MAD = f_MedianAbsoluteDeviation(r_obs, adaptor, r_weights, f_weight_sum, MAD); // yes
		const _TyScalar s = (f_mad_scale)? f_MAD * f_mad_scale : _TyScalar(1);
		// calculate MAD (median absolute deviation)

		typename std::vector<_TyScalar>::const_iterator p_weight_it = r_weights.begin();
		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it, ++ p_weight_it) {
			Vector2 v_obs = adaptor(*p_obs_it);
			const _TyScalar x = v_obs(0), y = v_obs(1);
			// get an observation

			const _TyScalar f_weight = *p_weight_it;
			// get explicit weight

			_TyScalar f_err = y - (*this)(x); // calculate residual in x

			_TyScalar w = (f_err)? score_function(fabs(f_err / s)) : 1; // otherwise the weight is NaN
			// calculate the robust weight vector

			f_error += f_err * f_err * (w * f_weight);
			f_weight_sum += (w * f_weight);
		}

		return sqrt(f_error / f_weight_sum);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	/**
	 *	@brief calculates median absolute deviation of the fitted
	 *		model (this polynomial) compared to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] r_workspace is a workspace vector to store the weights (note that the order is undefined)
	 *
	 *	@return Returns (unscaled) MAD of the model with respect to the observations.
	 */
	template <class CObservations, class CObsAdaptor>
	_TyScalar f_MedianAbsoluteDeviation(const CObservations &r_obs, CObsAdaptor adaptor,
		std::vector<_TyScalar> &r_workspace)
	{
		_ASSERTE(!r_obs.empty()); // otherwise will fail

		std::vector<_TyScalar> &MAD = r_workspace; // rename
		MAD.clear(); // but reuse the buffer
		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it) {
			Vector2 v_obs = adaptor(*p_obs_it);
			// get a measurement

			_TyScalar f_err = v_obs(1) - (*this)(v_obs(0));
			// calculate an error (not squared!)

			MAD.push_back(fabs(f_err));
		}
		std::nth_element(MAD.begin(), MAD.begin() + (MAD.size() / 2), MAD.end());
		const _TyScalar f_MAD = MAD[MAD.size() / 2];
		// calculate MAD (median absolute deviation)

		return f_MAD;
	}

	/**
	 *	@brief calculates weighted median absolute deviation of the fitted
	 *		model (this polynomial) compared to a set of observations
	 *
	 *	@tparam CObservations is type of observations container (STL-like, forward-iterable)
	 *	@tparam CWeights is type of weights container (STL-like, forward-iterable)
	 *	@tparam CObsAdaptor is type of observation adaptor that converts the observations
	 *		from the stored data type to \ref Vector2, such as e.g. \ref TCastAdaptor
	 *
	 *	@param[in] r_obs is a vector of observations (pairs of
	 *		\f$\left(x, f(x)\right)\f$, where \f$f(x)\f$ is this polynomial)
	 *	@param[in] adaptor is observation adaptor instance
	 *	@param[in] r_weights is a vector of weights corresponding to the observations
	 *	@param[in] f_weight_sum is precomputed sum of the weights in <tt>r_weights</tt>
	 *	@param[in] r_workspace is a workspace vector to store the deviation, weight pairs
	 *
	 *	@return Returns (unscaled) MAD of the model with respect to the observations.
	 */
	template <class CObservations, class CWeights, class CObsAdaptor>
	_TyScalar f_MedianAbsoluteDeviation(const CObservations &r_obs, CObsAdaptor adaptor,
		const CWeights &r_weights, _TyScalar f_weight_sum, std::vector<std::pair<_TyScalar, _TyScalar> > &r_workspace)
	{
		_ASSERTE(!r_obs.empty()); // otherwise will fail
		_ASSERTE(r_weights.size() == r_obs.size());
		_ASSERTE(fabs(std::accumulate(r_weights.begin(), r_weights.end(),
			_TyScalar(0)) - f_weight_sum) < 1e-10); // this should match

		std::vector<std::pair<_TyScalar, _TyScalar> > &MAD = r_workspace; // rename
		MAD.clear(); // but reuse the buffer
		typename CWeights::const_iterator p_weight_it = r_weights.begin();
		for(typename CObservations::const_iterator p_obs_it = r_obs.begin(),
		   p_end_it = r_obs.end(); p_obs_it != p_end_it; ++ p_obs_it, ++ p_weight_it) {
			Vector2 v_obs = adaptor(*p_obs_it);
			// get a measurement

			_TyScalar f_err = v_obs(1) - (*this)(v_obs(0));
			// calculate an error (not squared!)

			_ASSERTE(*p_weight_it >= 0);
			MAD.push_back(std::make_pair(_TyScalar(fabs(f_err)), _TyScalar(*p_weight_it)));
			// push both the error and the associated weight
		}
		// calculate absolute deviations

		std::sort(MAD.begin(), MAD.end()); // could only sort by the error, not by weight (does not matter)
		_TyScalar f_half_weight = f_weight_sum / 2;
		for(size_t i = 0, n = MAD.size(); i < n; ++ i) {
			_TyScalar f_weight = MAD[i].second;
			if(f_half_weight < f_weight) // not equal, to avoid zero-weight entries
				return MAD[i].first;
			f_half_weight -= f_weight;
		}
		// use the weights to aid median selection

		return MAD.back().first; // fallback
	}
};

// for a distortion model, we use p_undistorted = p_distorted * (1 + polynomial(|p_distorted|)) (assuming center of distortion in the origin, easily shifted by subtracting another center before and then adding it back after)
// to go back, one can use p_distorted = p_undistorted / (1 + polynomial(|p_distorted|))
// at the same time, |p_undistorted| = |p_distorted| * (1 + polynomial(|p_distorted|))
// |p_undistorted| = |p_distorted| + |p_distorted| * polynomial(|p_distorted|)
// |p_distorted| = |p_undistorted| / (1 + polynomial(|p_distorted|)
// |p_undistorted| - |p_distorted| * polynomial(|p_distorted|) = |p_distorted|
// polynomial(|p_distorted|) = |p_undistorted| / |p_distorted| - 1
// |p_distorted| = inv_polynomial(|p_undistorted| / |p_distorted| - 1)
// so inverting the polynomial does not give an inverse function by itself, still need to use numerical solution

// if it was |p_undistorted| = |p_distorted| * polynomial(|p_distorted|)
// |p_undistorted| / |p_distorted| = polynomial(|p_distorted|)
// inv_polynomial(|p_undistorted| / |p_distorted|) = |p_distorted|
// so this still does not help, even if we could invert 1 + polynomial(x)

// p_undistorted = p_distorted * (1 + k * |p_distorted|)
// |p_undistorted| = |p_distorted| * (1 + k * |p_distorted|)
// |p_undistorted| = |p_distorted| + k * |p_distorted|^2
// |p_distorted| + k * |p_distorted|^2 - |p_undistorted| = 0
// 1 * |p_distorted| + k * |p_distorted|^2 - |p_undistorted| = 0 // ax + bx^2 + c = 0, x = |p_distorted|, a = 1, b = k, c = -|p_undistorted|
// so |p_distorted| = (-k +- sqrt(k*k + 4*|p_undistorted|))/-2 // here the value of sqrt is always greater than k
// so |p_distorted| = (k + sqrt(k*k + 4*|p_undistorted|))/2 // subtracting it would yield a negative root, which does not correspond to a norm of a vector

// so need to solve one degree higher polynomial being equal to |p_undistorted|, setting the lowest
// coeff to 1 and keeping the rest; this presents an issue with regards to the above template as it
// might require setting a few coeffs to zero in order to do that, but it would work. additionally,
// the correct root needs to be selected, which is nontrivial (we used to choose the one closest to
// the original point).

// alternatively, one can fit another polynomial to the inverse function and just use that (this is
// likely faster if there are many points to undistort and the polynomial is held constant; can use
// it easily in e.g. a shader. on the other hand, it is less precise and there is an issue with the
// polynomials generally not being invertible functions (still, can invert the first lobe but it is
// not guaranteed it will cover the argument space sufficiently well))

#endif // !__POLYNOMIAL_FUNCTIONS_INCLUDED
