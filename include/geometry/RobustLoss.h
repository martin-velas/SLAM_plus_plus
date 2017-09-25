/*
								+----------------------------------+
								|                                  |
								|    ***  Robust functions  ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|           RobustLoss.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __ROBUST_LOSS_FUNCTIONS_INCLUDED
#define __ROBUST_LOSS_FUNCTIONS_INCLUDED

/**
 *	@file geometry/RobustLoss.h
 *	@brief robust loss functions
 *	@author -tHE SWINe-
 *	@date 2016
 *
 *	Robust score is derivative of the robust loss. E.g. for ordinary least
 *	squares, the loss is \f$\frac{u^2}{2}\f$, the score is then \f$u\f$.
 *
 *	The weight is then calculated by diviging the score by the error. This
 *	can be directly used in IRLS for robust estimation.
 *
 *	Note that some other frameworks calculate the score function as well as the first
 *	and the second derivative, however only the first derivative is used (to weight the
 *	Jacobians and errors and to weight \f$\chi^2\f$ to make robust \f$\chi^2\f$).
 *
 *	Typically, \f$\chi^2\f$ of the constraints (divided by a scaling parameter such as
 *	MAD) is employed as the argument of these functions as it is comparable among
 *	different types of constraints.
 *
 *	@todo The derivatives are mostly untested; write unit tests and compare to analytical
 *		differencing, draw some plots to verify visually too.
 */

#include "slam/BlockMatrixBase.h" // DimensionCheck

// note - the classes are all templates to make it possible to use dual numbers in them,
// if ever needed, or to evaluate the weights with lower precision and save some space
// (this is mostly simple and numerically robust code)

/** \addtogroup robust_group
 *	@{
 */

/**
 *	@brief Huber loss function (score \f$\psi(r) = \left\{\begin{array}{ll}1 & \text{if}\; |r| \leq a \\ a \; \text{sign}(u) & \textit{otherwise}\end{array}\right.\f$), a monotonically increasing loss function
 */
template <class CScalar = double>
class CHuberLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param;

public:
	CHuberLoss(_TyScalar f_param = 1.345)
		:m_f_param(f_param)
	{}

	/**
	 *	@brief evaluates the loss function and its derivatives
	 *	@tparam CVectorType is Eigen vector type to hold the result; use this
	 *		to specify the desired number of derivatives (up to three)
	 *	@param[in] f_error is absolute value of the error to be weighted
	 *	@return Returns a vector containing the value of the loss function
	 *		in the first element and its derivatives in the consecutive elements.
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		if(n_dim > 0)
			v_result(0) = (f_error <= m_f_param)? .5 * f_error * f_error : m_f_param * (f_error - .5 * m_f_param);
		if(n_dim > 1)
			v_result(1) = (f_error <= m_f_param)? f_error : m_f_param /** sign(f_error)*/;
		if(n_dim > 2)
			v_result(2) = (f_error <= m_f_param)? 1 : 0;
		return v_result;
	}

	/**
	 *	@brief evaluates the weight function
	 *	@param[in] f_error is absolute value of the error to be weighted
	 *	@return Returns the value of the first derivative of the score
	 *		function divided by the error (the weight).
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		return (f_error <= m_f_param)? 1 : m_f_param / f_error; // this is Huber weight (w)
	}
};

/**
 *	@brief Cauchy loss function (score \f$\psi(r) = \displaystyle\frac{r}{1 + \left(\frac{r}{a}\right)^2}\f$), a soft redescending loss function
 */
template <class CScalar = double>
class CCauchyLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param;

public:
	CCauchyLoss(_TyScalar f_param = 2.385) // t_odo - figure out the parameter for 95% efficiency
		:m_f_param(f_param)
	{}

	/**
	 *	@copydoc CHuberLoss::v_Loss
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		if(n_dim > 0)
			v_result(0) = .5 * m_f_param * m_f_param * log(1 + f_error * f_error / (m_f_param * m_f_param));
		if(n_dim > 1)
			v_result(1) = f_error * m_f_param * m_f_param / (m_f_param * m_f_param + f_error * f_error);
		if(n_dim > 2) {
			v_result(2) = m_f_param * m_f_param * (m_f_param * m_f_param - f_error * f_error) /
				((m_f_param * m_f_param + f_error * f_error) * (m_f_param * m_f_param + f_error * f_error));
		}
		return v_result;
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		return m_f_param * m_f_param / (m_f_param * m_f_param + f_error * f_error);
		//return 1 / (1 + f_error * f_error / (m_f_param * m_f_param)); // the same but more division
	}
};

/**
 *	@brief Tukey biweight loss function (score \f$\psi(r) = \left\{\begin{array}{ll}r \left(1 - \left(\frac{r}{a}\right)^2\right)^2 & \text{if}\; |r| \leq a \\ 0 & \textit{otherwise}\end{array}\right.\f$), a hard redescending loss function
 */
template <class CScalar = double>
class CTukeyBiweightLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param;

public:
	CTukeyBiweightLoss(_TyScalar f_param = 4.685)
		:m_f_param(f_param)
	{}

	/**
	 *	@copydoc CHuberLoss::v_Loss
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		const _TyScalar a = m_f_param, u = f_error; // just rename
		if(n_dim > 0) {
			v_result(0) = (u <= a)? a * a * (1 -
				(1 - u * u / (a * a)) * (1 - u * u / (a * a)) * (1 - u * u / (a * a))) / 6 : a * a / 6;
		}
		if(n_dim > 1)
			v_result(1) = (u <= a)? (a * a - u * u) * (a * a - u * u) * u / (a * a * a * a) : 0;
		if(n_dim > 2)
			v_result(2) = (u <= a)? (a * a - u * u) * (a * a - 5.0 * u * u) / (a * a * a * a) : 0;
		return v_result;
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		const _TyScalar a = m_f_param, u = f_error; // just rename
		return (u <= a)? (1 - (u * u) / (a * a)) * (1 - (u * u) / (a * a)) : 0;
	}
};

/**
 *	@brief Hampel loss function (score \f$\psi(r) = \left\{\begin{array}{ll}r & \text{if}\; |r| < a \\ a\,\text{sign}(r) & \text{if}\; a \leq |r| < b \\ a \displaystyle\frac{c\,\text{sign}(r) - r}{c - b} & \text{if}\; b \leq |r| < c \\ 0 & \textit{otherwise}\end{array}\right.\f$), a hard redescending loss function
 */
template <class CScalar = double>
class CHampelLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param_lo;
	_TyScalar m_f_param_mid;
	_TyScalar m_f_param_hi;

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] f_param_lo is the first parameter value (\f$a\f$)
	 *	@param[in] f_param_mid is the second parameter value (\f$b\f$)
	 *	@param[in] f_param_hi is the third parameter value (\f$c\f$)
	 *
	 *	@note It is reccommended to set \f$c = 2a + b\f$.
	 *	@note Setting \f$a = 2\f$, \f$b = 4\f$ and \f$c = 8\f$  leads to 99% efficiency (likely
	 *		being slightly less robust to outliers in the end).
	 *	@note Setting \f$a = 1.5k\f$, \f$b = 3.5k\f$ and \f$c = 8k\f$ with \f$k = 0.902\f$ leads
	 *		to 95% efficiency (with a slightly different ratio of \f$a:b:c\f$ than the defaults).
	 *	@note The defaults \f$(1.393, 2.787, 5.573)\f$ were set by optimization and are likely not
	 *		listed in any of the existing publications (or at least not exactly).
	 */
	CHampelLoss(_TyScalar f_param_lo = 1.393, _TyScalar f_param_mid = 2.787, _TyScalar f_param_hi = 5.573)
		:m_f_param_lo(f_param_lo), m_f_param_mid(f_param_mid), m_f_param_hi(f_param_hi)
	{}

	/**
	 *	@copydoc CHuberLoss::v_Loss
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		const _TyScalar a = m_f_param_lo, b = m_f_param_mid, c = m_f_param_hi, u = f_error; // just rename
		if(n_dim > 0) {
			v_result(0) = (f_error < a)? .5 * u * u :
						  (f_error < b)? a * (/*fabs*/(u) - .5 * a * a) :
						  (f_error < c)? a * (c * /*fabs*/(u) - .5 * u * u) / (c - b) - 7.0 / 6 * a * a :
										 a * (b + c - a);
		}
		if(n_dim > 1) {
			v_result(1) = (f_error < a)? u :
						  (f_error < b)? a /** sign(u)*/ :
						  (f_error < c)? a * (c /** sign(u)*/ - u) / (c - b) :
										 0;
		}
		if(n_dim > 2) {
			v_result(2) = (f_error < a)? 1 :
						  (f_error < b)? 0 :
						  (f_error < c)? a / (b - c) :
										 0;
		}
		return v_result;
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		const _TyScalar a = m_f_param_lo, b = m_f_param_mid, c = m_f_param_hi, u = f_error; // just rename
		return (f_error < a)? 1 :
			   (f_error < b)? a / /*fabs*/(f_error) :
			   (f_error < c)? a * (c / /*fabs*/(f_error) - 1) / (c - b) :
							  0;
	}
};

/**
 *	@brief Logistic loss function (score \f$\psi(r) = a \tanh \left(\frac{r}{a}\right)\f$), a monotonically increasing loss function
 */
template <class CScalar = double>
class CLogisticLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param;

public:
	CLogisticLoss(_TyScalar f_param = 1.205) // t_odo - figure out the parameter for 95% efficiency
		:m_f_param(f_param)
	{}

	/**
	 *	@copydoc CHuberLoss::v_Loss
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		if(n_dim > 0)
			v_result(0) = m_f_param * m_f_param * log(cosh(f_error / m_f_param)); // not sure about this
		if(n_dim > 1)
			v_result(1) = m_f_param * tanh(f_error / m_f_param);
		if(n_dim > 2)
			v_result(2) = 1 / cosh(f_error / m_f_param) * cosh(f_error / m_f_param);
		// log is natural logarithm; note that cosh is not periodic but rather hyperbolic
		return v_result;
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		return m_f_param / f_error * tanh(f_error / m_f_param);
	}
};

/**
 *	@brief "fair" loss function (score \f$\psi(r) = \displaystyle\frac{r}{1 + \frac{|r|}{a}}\f$), a monotonically increasing loss function
 */
template <class CScalar = double>
class CFairLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param;

public:
	CFairLoss(_TyScalar f_param = 1.400) // t_odo - figure out the parameter for 95% efficiency
		:m_f_param(f_param)
	{}

	/**
	 *	@copydoc CHuberLoss::v_Loss
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		if(n_dim > 0)
			v_result(0) = m_f_param * (f_error - m_f_param * log(m_f_param + /*fabs*/(f_error))); // not sure about this at all
		if(n_dim > 1)
			v_result(1) = f_error / (1 + /*fabs*/(f_error) / m_f_param);
		if(n_dim > 2)
			v_result(2) = m_f_param * m_f_param / ((/*fabs*/(f_error) + m_f_param) * (/*fabs*/(f_error) + m_f_param));
		return v_result;
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		return 1 / (1 + /*fabs*/(f_error) / m_f_param);
	}
};

/**
 *	@brief Welsch loss function (score \f$\psi(r) = r e^{-\left(\raisebox{.4ex}{\small $r$\hspace{-.25ex}}/\raisebox{-.4ex}{\small $a$}\right)^2}\f$), a soft redescending loss function
 */
template <class CScalar = double>
class CWelschLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

protected:
	_TyScalar m_f_param;

public:
	CWelschLoss(_TyScalar f_param = 2.985) // t_odo - figure out the parameter for 95% efficiency
		:m_f_param(f_param)
	{}

	/**
	 *	@copydoc CHuberLoss::v_Loss
	 */
	template <class CVectorType>
	CVectorType v_Loss(_TyScalar f_error) const
	{
		enum {
			n_dim = (CVectorType::RowsAtCompileTime > 3)? 3 : CVectorType::RowsAtCompileTime
		};
		CVectorType v_result;
		DimensionCheck<Eigen::Matrix<double, n_dim, 1> >(v_result);

		_ASSERTE(f_error >= 0);
		if(n_dim > 0)
			v_result(0) = m_f_param * (f_error - m_f_param * log(m_f_param + /*fabs*/(f_error))); // not sure about this at all
		if(n_dim > 1)
			v_result(1) = f_error * exp(-f_error * f_error / (m_f_param * m_f_param));
		if(n_dim > 2)
			v_result(2) = m_f_param * m_f_param / ((/*fabs*/(f_error) + m_f_param) * (/*fabs*/(f_error) + m_f_param));
		return v_result;
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	_TyScalar operator ()(_TyScalar f_error) const
	{
		_ASSERTE(f_error >= 0);
		return exp(-f_error * f_error / (m_f_param * m_f_param));
	}
};

/*

for further reference, see https://www.mathworks.com/help/stats/robustfit.html
also see https://cran.r-project.org/web/packages/robustbase/vignettes/psi_functions.pdf

function type / name									scale		present in SLAM++
-------------------------------------------------------------------------------------
'ols'		Ordinary least squares						None		yes
'andrews'	w = (abs(r)<pi) .* sin(r) ./ r				1.339		no
'bisquare' 	w = (abs(r)<1) .* (1 - r.^2).^2				4.685		yes (CTukeyBiweightLoss)
'cauchy'	w = 1 ./ (1 + r.^2)							2.385		yes (CCauchyLoss)
'fair'		w = 1 ./ (1 + abs(r))						1.400		yes (CFairLoss)
'huber'		w = 1 ./ max(1, abs(r))						1.345		yes (CHuberLoss)
'logistic'	w = tanh(r) ./ r							1.205		yes (CLogisticLoss)
'talwar'	w = 1 * (abs(r)<1)							2.795		no
'welsch'	w = exp(-(r.^2))							2.985		yes (CWelschLoss)
'hampel'	w = ((abs(r) < 4) .* (2 - (abs(r) < 2))) .*	1.393		yes (CHampelLoss)
			(1 ./ max(1, abs(r)) - (abs(r) >= 2) ./ 4)

*/

typedef CHuberLoss<double> CHuberLossd; /**< @brief Huber robust loss function */
typedef CTukeyBiweightLoss<double> CTukeyBiweightLossd; /**< @brief Tukey robust loss function */
typedef CHampelLoss<double> CHampelLossd; /**< @brief Hampel robust loss function */
typedef CCauchyLoss<double> CCauchyLossd; /**< @brief Cauchy robust loss function */
typedef CLogisticLoss<double> CLogisticLossd; /**< @brief logistic robust loss function */
typedef CFairLoss<double> CFairLossd; /**< @brief "fair" robust loss function */
typedef CWelschLoss<double> CWelschLossd; /**< @brief Welsch robust loss function */

#ifdef __ROBUST_LOSS_POLYNOMIAL_TESTS
// some tests of the above functions

#include <time.h>
#include "geometry/Polynomial.h"

/**
 *	@brief namespace containing some tests of the robust estimation on polynomials
 */
namespace polynomial_tests {

/**
 *	@brief concatenates rand() to 64-bit uniformly distributed numbers
 *	@return Returns a 64-bit uniformly distributed pseudo-random number.
 */
uint64_t ran64()
{
	enum {
		n_rand_max_1_half = uint32_t(RAND_MAX) / 2 + (uint32_t(RAND_MAX) & 1),
		n_rand_max_1_half_1 = n_rand_max_1_half | (n_rand_max_1_half >> 1),
		n_rand_max_1_half_2 = n_rand_max_1_half_1 | (n_rand_max_1_half_1 >> 2),
		n_rand_max_1_half_4 = n_rand_max_1_half_2 | (n_rand_max_1_half_2 >> 4),
		n_rand_max_1_half_8 = n_rand_max_1_half_4 | (n_rand_max_1_half_4 >> 8),
		n_rand_max_1_half_16 = n_rand_max_1_half_8 | (n_rand_max_1_half_8 >> 16),

		n_lower_max = (RAND_MAX >= UINT32_MAX)? UINT32_MAX :
			(b_Is_POT_Static(uint32_t(RAND_MAX) + 1))? RAND_MAX :
			n_rand_max_1_half_16/*n_Make_Lower_POT_Static(uint32_t(RAND_MAX) + 1) - 1*/, // avoid integer overflow warnings
		b_need_looping = RAND_MAX < UINT64_MAX && RAND_MAX != n_lower_max,
		n_bit_width = n_SetBit_Num_Static(n_lower_max),
		n_pass_num = (64 + n_bit_width - 1) / n_bit_width
	};
	_ASSERTE(RAND_MAX >= UINT32_MAX || b_Is_POT_Static(uint32_t(RAND_MAX) + 1) ||
		n_rand_max_1_half_16 == n_Make_Lower_POT_Static(uint32_t(RAND_MAX) + 1) - 1);

	uint64_t n_result = 0;
	for(int i = 0; i < int(n_pass_num); ++ i) {
		uint64_t n_rand = rand();
		while(b_need_looping && n_rand > n_lower_max) // only loops if n_lower_max != RAND_MAX
			n_rand = rand();
		n_result <<= n_bit_width;
		n_result |= n_rand;
	}
	return n_result;
}

/**
 *	@brief generates uniformly distributed random numbers in [0, 1] with full mantissa
 *	@return Returns an uniformly distributed random number in [0, 1].
 */
double ranf()
{
	uint64_t n_rand = ran64() >> 10; // msvc 6.0 does not implement conversion from uint64_t to double
	while(n_rand > (uint64_t(1) << 53)) // generates 54-bit numbers, but we're interested only in 1 + 53 zeroes or below
		n_rand = ran64() >> 10; // can try again, the samples are independent
	return double(int64_t(n_rand)) * (1.0 / 9007199254740992.0); // n_rand is [0, 9007199254740992] inclusive
}

/**
 *	@brief generates normally distributed random numbers with zero mean and unit standard deviation
 *	@return Returns a normally distributed random number.
 */
double rann()
{
	static double f_buffered_rand = 0;
	static bool b_have_buffered_rand = false;
	if(b_have_buffered_rand) {
		b_have_buffered_rand = false;
		return f_buffered_rand;
	}
	// this generates two nümbers and stores one for later; get the stored number

	double x1, x2, w;
	do {
		x1 = 2.0 * ranf() - 1.0;
		x2 = 2.0 * ranf() - 1.0;
		w = x1 * x1 + x2 * x2;
	} while(w >= 1.0);
	w = sqrt((-2.0 * log(w)) / w); // Box-Muller transform
	f_buffered_rand = x1 * w;
	b_have_buffered_rand = true;
	return x2 * w; // return the other random number immediately
}

/**
 *	@brief evaluates relative estimator efficiency
 *	@tparam CLossFun is loss function type
 *	@param[in] loss_fun is loss function to be used for the estimator
 *	@return Returns the relative estimator efficiency (compared to ordinary LS).
 */
template <class CLossFun>
double Polynomial_REETest(CLossFun loss_fun)
{
	double f_avg_sqerr_ols = 0;
	double f_avg_sqerr_rob = 0;
	std::vector<Eigen::Matrix<double, 2, 1, Eigen::DontAlign> > samples_noisy; // reuse storage
#ifdef _DEBUG
	for(size_t n_pass = 0; n_pass < 1000; ++ n_pass) {
#else // _DEBUG
	for(size_t n_pass = 0; n_pass < 1000000; ++ n_pass) {
#endif // _DEBUG
		Eigen::Vector2d v_line(rann(), rann());
		// ranom line with Gaussian distribution

		samples_noisy.clear();
		for(int i = -50; i < 50; ++ i) {
			double y = v_line(0) + v_line(1) * i;
			double yn = y + rann();
			samples_noisy.push_back(Eigen::Vector2d(double(i), yn));
		}
		// generate samples

		CPolynomial<2, 0, 1> line;
		line.LeastSquares_Fit(samples_noisy);

		double f_err_ols = (v_line - line.v_Parameters()).squaredNorm(); // line.f_LeastSquares_Error(samples_noisy);

		line.Robust_Fit_Update(samples_noisy, loss_fun);
		//line.Robust_Fit_Update(samples_noisy, loss_fun, 25, 1e-6, .0, .0); // disable the use of MAD // underestimates the efficiency

		double f_err_rob = (v_line - line.v_Parameters()).squaredNorm(); // line_rob.f_LeastSquares_Error(samples_noisy);

		f_avg_sqerr_ols += f_err_ols;
		f_avg_sqerr_rob += f_err_rob;
	}

	double f_efficiency = f_avg_sqerr_ols / f_avg_sqerr_rob;

	printf("relative estimator efficiency (compared to ordinary LS) is %.2f%% for %s\n",
		f_efficiency * 100, s_TypeName<CLossFun>().c_str()); // should be about 95%

	return f_efficiency;
}

/**
 *	@brief factory for loss function with just a single parameter
 *	@tparam CLossFun is loss function type
 */
template <class CLossFun>
class CUnaryLossFactory {
public:
	/**
	 *	@brief makes a robust function
	 *	@param[in] f_k is robust function argument
	 *	@return Returns loss function instance, initialized with the given argument.
	 */
	CLossFun operator ()(double f_k) const
	{
		return CLossFun(f_k);
	}
};

/**
 *	@brief factory for loss function with three parameters
 *	@tparam CLossFun is loss function type
 */
template <class CLossFun>
class CTernaryLossFactory {
protected:
	double m_f_a; /**< @brief relative scale of the first loss function parameter */
	double m_f_b; /**< @brief relative scale of the second loss function parameter */
	double m_f_c; /**< @brief relative scale of the third loss function parameter */

public:
	/**
	 *	@brief default constructor; sets relative scales of the parameters
	 *
	 *	@param[in] f_a relative scale of the first loss function parameter
	 *	@param[in] f_b relative scale of the second loss function parameter
	 *	@param[in] f_c relative scale of the third loss function parameter
	 */
	CTernaryLossFactory(double f_a, double f_b, double f_c)
		:m_f_a(f_a), m_f_b(f_b), m_f_c(f_c)
	{}

	/**
	 *	@brief makes a robust function
	 *	@param[in] f_k is robust function argument
	 *	@return Returns loss function instance, initialized with the given
	 *		argument and the scales passed to the constructor.
	 */
	CLossFun operator ()(double f_k) const
	{
		return CLossFun(m_f_a * f_k, m_f_b * f_k, m_f_c * f_k);
	}
};

/**
 *	@brief tunes loss function argument for a given efficiency
 *
 *	@tparam CLossFunFactory is a loss function factory type (e.g. \ref CUnaryLossFactory)
 *
 *	@param[in] loss_factory is loss function factory instance
 *	@param[in] f_min is minimum viable loss function argument
 *	@param[in] f_max is maximum viable loss function argument
 *	@param[in] f_target_ree is target relative estimator efficiency
 *	@param[in] b_use_interpolation_search is interpolation search flag (if set,
 *		interpolation search is used; otherwise binary search is used)
 *	@param[in] n_seed is random seed to use throughout this function
 *
 *	@return Returns the loss function argument that attains the given
 *		target relative estimator efficiency.
 */
template <class CLossFunFactory>
double f_Tune_LossFun_REE(CLossFunFactory loss_factory, double f_min = .5, double f_max = 1,
	double f_target_ree = .95, bool b_use_interpolation_search = false, int n_seed = 153456)
{
	// could use one-sided search to inizialize min and max, but would be slightly slower

	printf("debug: trying with seed 0x%08x\n", n_seed);
	srand(n_seed);
	double f_ree_lo = Polynomial_REETest(loss_factory(f_min));
	srand(n_seed);
	double f_ree_hi = Polynomial_REETest(loss_factory(f_max));
	bool b_proportional = (f_max - f_min) * (f_ree_hi - f_ree_lo) > 0;
	double f_mid;
	for(int i = 0; i < 15; ++ i) {
		double t = (b_use_interpolation_search)?
			(f_target_ree - f_ree_lo) / (f_ree_hi - f_ree_lo) : .5;
		if(b_use_interpolation_search && t >= 0 && t <= 1) // if interpolation and not extrapolating
			t = t * .5 + .25; // halfway between interpolation and binary (interpolation tends to undershoot)
		f_mid = f_min + t * (f_max - f_min);

		srand(n_seed);
		double f_ree_mid = Polynomial_REETest(loss_factory(f_mid));

		if(fabs(f_ree_mid - f_target_ree) < 5e-5)
			break;
		// early exit to limit the number of iterations

		if((!b_proportional && f_ree_mid > f_target_ree) ||
		   (b_proportional && f_ree_mid < f_target_ree)) { // jump depending on whether it is proportional or inversely proportional
			f_ree_lo = f_ree_mid;
			f_min = f_mid;
			//printf("l "); // debug
		} else {
			f_ree_hi = f_ree_mid;
			f_max = f_mid;
			//printf("h "); // debug
		}
	}

	if(!b_use_interpolation_search)
		f_mid = (f_min + f_max) / 2;
	printf("lo: %g, hi: %g, mid: %g\n", f_min, f_max, f_mid);
	//printf("ended up with %.5f %.5f %.5f\n", 2 * f_mid, 4 * f_mid, 8 * f_mid);

	return f_mid;
}

/**
 *	@brief pseudo-random seed
 *
 *	@param[in] i is iteration number
 *	@param[in] n_base_seed is base pseudo-random seed
 *
 *	@return Returns different values of seeds, depending on the iteration, base seed and time.
 */
int n_RandSeed(int i, int n_base_seed = 123456)
{
	return (((n_base_seed << i) | (n_base_seed >> (32 - i))) ^ uint32_t(time(0)) ^ // note that this has 1 second resolution
		(i ^ (i << 4) ^ (i << 8) ^ (i << 12) ^ (i << 16) ^ (i << 20) ^ (i << 24) ^ (i << 28)));
}

/**
 *	@brief performs some tests on the robust kernels, prints to stdout
 */
void Run()
{
	const int n = 10;
	double f_Hampel_k = 0;
	for(int i = 0; i < n; ++ i)
		f_Hampel_k += f_Tune_LossFun_REE(CTernaryLossFactory<CHampelLossd>(2, 4, 8), .5, .8, .95, true, n_RandSeed(i));
	f_Hampel_k /= n;
	printf("for 95%% efficient Hampel, use %.3f %.3f %.3f\n", 2 * f_Hampel_k, 4 * f_Hampel_k, 8 * f_Hampel_k);

	double f_Huber_a = 0;
	for(int i = 0; i < n; ++ i)
		f_Huber_a += f_Tune_LossFun_REE(CUnaryLossFactory<CHuberLossd>(), .1, 5, .95, true, n_RandSeed(i));
	f_Huber_a /= n;
	printf("for 95%% efficient Huber, use %.3f\n", f_Huber_a);

	double f_Welsch_a = 0;
	for(int i = 0; i < n; ++ i)
		f_Welsch_a += f_Tune_LossFun_REE(CUnaryLossFactory<CWelschLoss<double> >(), .1, 5, .95, true, n_RandSeed(i));
	f_Welsch_a /= n;
	printf("for 95%% efficient Welsch, use %.3f\n", f_Welsch_a);

	Polynomial_REETest(CHuberLossd());
	Polynomial_REETest(CCauchyLoss<double>());
	Polynomial_REETest(CTukeyBiweightLossd());
	printf("for %g %g %g : ", 1.393, 2.787, 5.573); Polynomial_REETest(CHampelLossd(1.393, 2.787, 5.573));
	double f = 0.901608;
	printf("for %g %g %g : ", 1.5 * f, 3.5 * f, 8 * f); Polynomial_REETest(CHampelLossd(1.5 * f, 3.5 * f, 8 * f)); f /= 1.1;
	f = 1;
	printf("for %g %g %g : ", 2 * f, 4 * f, 8 * f); Polynomial_REETest(CHampelLossd(2 * f, 4 * f, 8 * f)); f /= 1.1;
	printf("for %g %g %g : ", 2 * f, 4 * f, 8 * f); Polynomial_REETest(CHampelLossd(2 * f, 4 * f, 8 * f)); f /= 1.1;
	printf("for %g %g %g : ", 2 * f, 4 * f, 8 * f); Polynomial_REETest(CHampelLossd(2 * f, 4 * f, 8 * f)); f /= 1.1;
	printf("for %g %g %g : ", 2 * f, 4 * f, 8 * f); Polynomial_REETest(CHampelLossd(2 * f, 4 * f, 8 * f)); f /= 1.1;
	Polynomial_REETest(CLogisticLoss<double>());
	Polynomial_REETest(CFairLoss<double>());
	Polynomial_REETest(CWelschLoss<double>());
}

} // ~polynomial_tests

#endif // __ROBUST_LOSS_POLYNOMIAL_TESTS

/** @} */ // end of group

#endif // !__ROBUST_LOSS_FUNCTIONS_INCLUDED
