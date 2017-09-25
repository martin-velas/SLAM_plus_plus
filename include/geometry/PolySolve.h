/*
								+----------------------------------+
								|                                  |
								|   ***  Polynomial solvers  ***   |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|           PolySolve.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __POLYNOMIAL_SOLVERS_INCLUDED
#define __POLYNOMIAL_SOLVERS_INCLUDED

/**
 *	@file geometry/PolySolve.h
 *	@brief basic closed-form polynomial solvers
 *	@author -tHE SWINe-
 *	@date 2015
 */

#include "slam/Integer.h"
#include <stdlib.h>
#include <math.h>

/**
 *	@brief quadratic function class
 */
template <class T = double>
class CQuadraticFunction {
protected:
	const T a; /**< @brief the 2nd order coefficient */
	const T b; /**< @brief the 1st order coefficient */
	const T c; /**< @brief 0th order coefficient */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] _a is the 2nd order coefficient
	 *	@param[in] _b is the 1st order coefficient
	 *	@param[in] _c is 0th order coefficient
	 */
	inline CQuadraticFunction(T _a, T _b, T _c)
		:a(_a), b(_b), c(_c)
	{}

	/**
	 *	@brief evaluates the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$ax^2 + bx + c\f$.
	 */
	inline T operator ()(T f_x) const
	{
		T f_x2 = f_x * f_x;
		return f_x2 * a + f_x * b + c;
	}

	/**
	 *	@brief evaluates the derivative of the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$2ax + b\f$.
	 *	@note The second derivative equals \f$2a\f$, all the following derivatives are zero.
	 */
	inline T f_Derivative(T f_x) const
	{
		return 2 * a * f_x + b;
	}
};

/**
 *	@brief cubic function class
 */
template <class T = double>
class CCubicFunction {
protected:
	const T a; /**< @brief 3rd order coefficient */
	const T b; /**< @brief the 2nd order coefficient */
	const T c; /**< @brief the 1st order coefficient */
	const T d; /**< @brief 0th order coefficient */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] _a is 3rd order coefficient
	 *	@param[in] _b is the 2nd order coefficient
	 *	@param[in] _c is the 1st order coefficient
	 *	@param[in] _d is 0th order coefficient
	 */
	inline CCubicFunction(T _a, T _b, T _c, T _d)
		:a(_a), b(_b), c(_c), d(_d)
	{}

	/**
	 *	@brief evaluates the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$ax^3 + bx^2 + cx + d\f$.
	 */
	inline T operator ()(T f_x) const
	{
		T f_x2 = f_x * f_x;
		T f_x3 = f_x * f_x2;
		return f_x3 * a + f_x2 * b + f_x * c + d;
	}

	/**
	 *	@brief evaluates the derivative of the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$3ax^2+2bx+c\f$.
	 */
	inline T f_Derivative(T f_x) const
	{
		T f_x2 = f_x * f_x;
		return 3 * a * f_x2 + 2 * b * f_x + c;
	}

	/**
	 *	@brief evaluates the derivative of the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$6ax+2b\f$.
	 *	@note The third derivative equals \f$6a\f$, all the following derivatives are zero.
	 */
	inline T f_SecondDerivative(T f_x) const
	{
		return 6 * a * f_x + 2 * b;
	}
};

/**
 *	@brief quartic function class
 */
template <class T = double>
class CQuarticFunction {
protected:
	const T a; /**< @brief 4th order coefficient */
	const T b; /**< @brief 3rd order coefficient */
	const T c; /**< @brief the 2nd order coefficient */
	const T d; /**< @brief the 1st order coefficient */
	const T e; /**< @brief 0th order coefficient */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] _a is 4th order coefficient
	 *	@param[in] _b is 3rd order coefficient
	 *	@param[in] _c is the 2nd order coefficient
	 *	@param[in] _d is the 1st order coefficient
	 *	@param[in] _e is 0th order coefficient
	 */
	inline CQuarticFunction(T _a, T _b, T _c, T _d, T _e)
		:a(_a), b(_b), c(_c), d(_d), e(_e)
	{}

	/**
	 *	@brief evaluates the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$ax^4 + bx^3 + cx^2 + dx + e\f$.
	 */
	inline T operator ()(T f_x) const
	{
		T f_x2 = f_x * f_x;
		T f_x3 = f_x * f_x2;
		T f_x4 = f_x2 * f_x2;
		return f_x4 * a + f_x3 * b + f_x2 * c + f_x * d + e;
	}

	/**
	 *	@brief evaluates the derivative of the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$4ax^3+3bx^2+2cx+d\f$.
	 */
	inline T f_Derivative(T f_x) const
	{
		T f_x2 = f_x * f_x;
		T f_x3 = f_x * f_x2;
		return 4 * a * f_x3 + 3 * b * f_x2 + 2 * c * f_x + d;
	}

	/**
	 *	@brief evaluates the derivative of the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$12ax^2+6bx+2c\f$.
	 */
	inline T f_SecondDerivative(T f_x) const
	{
		T f_x2 = f_x * f_x;
		return 12 * a * f_x2 + 6 * b * f_x + 2 * c;
	}

	/**
	 *	@brief evaluates the derivative of the function for a given argument
	 *	@param[in] f_x is value of the argument \f$x\f$
	 *	@return Returns value of \f$24ax+6b\f$.
	 *	@note The fourth derivative equals \f$24a\f$, all the following derivatives are zero.
	 */
	inline T f_ThirdDerivative(T f_x) const
	{
		return 24 * a * f_x + 6 * b;
	}
};

/**
 *	@brief a simple quadratic equation solver
 *
 *	With double-precision floating-point, this reaches 1e-12 worst-case and 1e-15 average
 *	precision of the roots (the value of the function in the roots). The roots can be however
 *	quite far from the true roots, up to 1e-10 worst-case and 1e-18 average absolute difference
 *	for cases when two roots exist. If only a single root exists, the worst-case precision is
 *	1e-13 and average-case precision is 1e-18.
 *
 *	With single-precision floating-point, this reaches 1e-3 worst-case and 1e-7 average
 *	precision of the roots (the value of the function in the roots). The roots can be however
 *	quite far from the true roots, up to 1e-1 worst-case and 1e-10 average absolute difference
 *	for cases when two roots exist. If only a single root exists, the worst-case precision is
 *	1e+2 (!) and average-case precision is 1e-2. Do not use single-precision floating point,
 *	except if pressed by time.
 *
 *	All the precision measurements are scaled by the maximum absolute coefficient value.
 *
 *	@tparam T is data type of the arguments (default double)
 *	@tparam b_sort_roots is root sorting flag (if set, the roots are
 *		given in ascending (not absolute) value; default true)
 *	@tparam n_2nd_order_coeff_log10_thresh is base 10 logarithm of threshold
 *		on the first coefficient (if below threshold, the equation is a linear one; default -6)
 *	@tparam n_zero_discriminant_log10_thresh is base 10 logarithm of threshold
 *		on the discriminant (if below negative threshold, the equation does not
 *		have a real root, if below threshold, the equation has just a single solution; default -6)
 */
template <class T = double, const bool b_sort_roots = true,
	const int n_2nd_order_coeff_log10_thresh = -6,
	const int n_zero_discriminant_log10_thresh = -6>
class CQuadraticEq : public CQuadraticFunction<T> {
protected:
	T p_real_root[2]; /**< @brief list of the roots (real parts) */
	//T p_im_root[2]; // imaginary part of the roots
	size_t n_real_root_num; /**< @brief number of real roots */

public:
	/**
	 *	@brief default constructor; solves for roots of \f$ax^2 + bx + c = 0\f$
	 *
	 *	This finds roots of the given equation. It tends to find two identical roots instead of one, rather
	 *	than missing one of two different roots - the number of roots found is therefore orientational,
	 *	as the roots might have the same value.
	 *
	 *	@param[in] _a is the 2nd order coefficient
	 *	@param[in] _b is the 1st order coefficient
	 *	@param[in] _c is 0th order coefficient
	 */
	CQuadraticEq(T _a, T _b, T _c) // ax2 + bx + c = 0
		:CQuadraticFunction<T>(_a, _b, _c)
	{
		T _aa = fabs(_a);
		if(_aa < f_Power_Static(10, n_2nd_order_coeff_log10_thresh)) { // otherwise division by a yields large numbers, this is then more precise
			p_real_root[0] = -_c / _b;
			//p_im_root[0] = 0;
			n_real_root_num = 1;
			return;
		}
		// a simple linear equation

		if(_aa < 1) { // do not divide always, that makes it worse
			_b /= _a;
			_c /= _a;
			_a = 1;

			// could copy the code here and optimize away division by _a (optimizing compiler might do it for us)
		}
		// improve numerical stability if the coeffs are very small

		const double f_thresh = f_Power_Static(10, n_zero_discriminant_log10_thresh);
		double f_disc = _b * _b - 4 * _a * _c;
		if(f_disc < -f_thresh)
			n_real_root_num = 0; // only two complex roots
		else if(f_disc <= f_thresh) {
			p_real_root[0] = T(-_b / (2 * _a));
			n_real_root_num = 1;
		} else {
			f_disc = sqrt(f_disc);
#if defined(_POSIX_VERSION)
			f_disc = copysign(f_disc, -_b); // save the branch (posix-compliant)
#elif defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
			f_disc = _copysign(f_disc, -_b); // save the branch (newer VS compilers)
#else // _POSIX_VERSION || (_MSC_VER && !__MWERKS__ && _MSC_VER > 1200)
			f_disc = (_b > 0)? -f_disc : f_disc; // not exactly equal to copysign in negative zero, but will do the job here
#endif // _POSIX_VERSION || (_MSC_VER && !__MWERKS__ && _MSC_VER > 1200)
			p_real_root[0] = T((-_b + f_disc) / (2 * _a));
			p_real_root[1] = T((2 * _c) / (-_b + f_disc));
			//p_im_root[0] = 0;
			//p_im_root[1] = 0;

			n_real_root_num = 2;

			if(b_sort_roots && p_real_root[0] > p_real_root[1]) // more adequate root sorting, given this root computation scheme
				std::swap(p_real_root[0], p_real_root[1]);
		}
	}

	/**
	 *	@brief gets number of real roots
	 *	@return Returns number of real roots (0 to 2).
	 */
	size_t n_RealRoot_Num() const
	{
		_ASSERTE(n_real_root_num >= 0);
		return n_real_root_num;
	}

	/**
	 *	@brief gets value of a real root
	 *	@param[in] n_index is zero-based index of the root
	 *	@return Returns value of the specified root.
	 */
	T f_RealRoot(size_t n_index) const
	{
		_ASSERTE(n_index < 2 && n_index < n_real_root_num);
		return p_real_root[n_index];
	}
};

/**
 *	@brief a simple cubic equation solver
 *
 *	With double-precision floating-point, this reaches 1e-10 worst-case and 1e-14 average
 *	precision of the roots (the value of the function in the roots). The roots can be however
 *	quite far from the true roots, the following list contains absolute differences from the
 *	real roots:
 *
 *	- three roots: up to 1e-10 worst-case, 1e-19 average
 *	- one double + one root: 1e-5 (!) worst-case, 1e-12 average
 *	- two roots: 1e-13 worst-case, 1e-18 average
 *	- one double root: 1e-3 (!) worst-case, 1e-9 average
 *	- a single root: ? worst-case, ? average
 *
 *	With single-precision floating-point, this reaches 1e-2 worst-case and 1e-6 average
 *	precision of the roots (the value of the function in the roots). The roots can be however
 *	quite far from the true roots, the following list contains absolute differences from the
 *	real roots:
 *
 *	- three roots: up to 1e-3 worst-case, 1e-11 average
 *	- one double + one root: 1e-1 (!) worst-case, 1e-8 average
 *	- two roots: 1e+0 (!) worst-case, 1e-11 average
 *	- one double root: 1e-0 (!) worst-case, 1e-7 average
 *	- a single root: 1e+1 (!) worst-case, 1e-6 average
 *
 *	Do not use single-precision floating point, except maybe if really pressed by time.
 *
 *	All the precision measurements are scaled by the maximum absolute coefficient value.
 *
 *	@tparam T is data type of the arguments (default double)
 *	@tparam b_sort_roots is root sorting flag (if set, the roots are
 *		given in ascending (not absolute) value; default true)
 *	@tparam n_3rd_order_coeff_log10_thresh is base 10 logarithm of threshold
 *		on the first coefficient (if below threshold, the equation is a quadratic one; default -6)
 *	@tparam n_second_root_precision_abs_log10_thresh is base 10 logarithm of absolute
 *		threshold on the second root precision (to decide whether the second root is a real one; default -6)
 *	@tparam n_second_root_precision_rel_log10_thresh is base 10 logarithm of relative
 *		threshold (w.r.t. error of the first root) on the second root precision (to decide
 *		whether the second root is a real one; default 1)
 *	@tparam n_quad_zero_discriminant_log10_thresh is base 10 logarithm of threshold
 *		on the quadratic equation discriminant (applies if the cubic equation is degenerate; default -6)
 */
template <class T = double, bool b_sort_roots = true,
	const int n_3rd_order_coeff_log10_thresh = -6,
	const int n_second_root_precision_abs_log10_thresh = -6,
	const int n_second_root_precision_rel_log10_thresh = 1,
	const int n_quad_zero_discriminant_log10_thresh = -6>
class CCubicEq : public CCubicFunction<T> {
protected:
	T p_real_root[3]; /**< @brief list of the roots (real parts) */
	//T p_im_root[3]; // imaginary part of the roots
	size_t n_real_root_num; /**< @brief number of real roots */

public:
	/**
	 *	@brief default constructor; solves for roots of \f$ax^3 + bx^2 + cx + d = 0\f$
	 *
	 *	This finds roots of the given equation. It tends to find two identical roots instead of one, rather
	 *	than missing one of two different roots - the number of roots found is therefore orientational,
	 *	as the roots might have the same value.
	 *
	 *	@param[in] _a is 3rd order coefficient
	 *	@param[in] _b is the 2nd order coefficient
	 *	@param[in] _c is the 1st order coefficient
	 *	@param[in] _d is 0th order coefficient
	 */
	CCubicEq(T _a, T _b, T _c, T _d)
		:CCubicFunction<T>(_a, _b, _c, _d)
	{
		if(fabs(_a) < f_Power_Static(10, n_3rd_order_coeff_log10_thresh)) { // otherwise division by a yields large numbers, this is then more precise
			/*if(fabs(_b) == 0) { // or a simple linear equation // not needed, implemented inside CQuadraticEq
				p_real_root[0] = -_d / _c;
				//p_im_root[0] = 0;
				n_real_root_num = 1;
			} else {*/
				CQuadraticEq<T, b_sort_roots, -12, n_quad_zero_discriminant_log10_thresh> eq2(_b, _c, _d);
				n_real_root_num = eq2.n_RealRoot_Num();
				for(unsigned int i = 0; i < n_real_root_num; ++ i) {
					p_real_root[i] = eq2.f_RealRoot(i);
					//p_im_root[i] = f_ImagRoot(i);
				}
			//}
			return;
		}
		// the highest power is multiplied by 0, it is a simple quadratic equation

		if(fabs(_d) == 0) {
			CQuadraticEq<T, b_sort_roots, -12, n_quad_zero_discriminant_log10_thresh> eq2(_a, _b, _c);
			n_real_root_num = eq2.n_RealRoot_Num();
			for(unsigned int i = 0; i < n_real_root_num; ++ i) {
				p_real_root[i] = eq2.f_RealRoot(i);
				//p_im_root[i] = f_ImagRoot(i);
			}

			p_real_root[n_real_root_num] = 0; // x = 0 is also a solution!
			++ n_real_root_num;
			if(b_sort_roots)
				std::sort(p_real_root, p_real_root + n_real_root_num); // !! the number of roots here may be 1 - 3
			// this needs to be sorted manually, as cos() is periodic and there is no good insight into the order

			return;
		}
		// the constant is 0, divide the whole equation by x to get a simple quadratic equation

		double q = (3 * _c * _a - _b * _b) / (9 * _a * _a); // this is actually very precise; more precise than the other q
		_b /= _a;
		_c /= _a;
		_d /= _a;
		//double q = (3 * _c - _b * _b) / 9; // less precise
		double r = (-27 * _d + _b * (9 * _c - 2 * _b * _b)) / 54;
		double disc = q * q * q + r * r; // can be converted to a multiple of a power of a, the sign and zero can then be determined precisely
		
		//p_im_root[0] = 0;
		// the first root is always real

		const double f_root_off = -_b / 3; // the root offset
		if(disc > 0) { // no thresh here
			double s = r + sqrt(disc), t = r - sqrt(disc);
			s = (s < 0)? -pow(-s, 1.0 / 3) : pow(s, 1.0 / 3);
			t = (t < 0)? -pow(-t, 1.0 / 3) : pow(t, 1.0 / 3);
			p_real_root[0] = T(f_root_off + (s + t));
			p_real_root[1] = T(f_root_off - (s + t) / 2);
			double f_im = sqrt(3.0) * (s - t) / 2;
			if(fabs(f_im) > 0 && fabs((*this)(p_real_root[1])) > std::max(
			   f_Power_Static(10, n_second_root_precision_abs_log10_thresh),
			   f_Power_Static(10, n_second_root_precision_rel_log10_thresh) *
			   fabs((*this)(p_real_root[0])))) { // only reject if the second root is very imprecise
			//if(fabs(f_im) > 0) { // this is missing the second root sometimes
			/*if(fabs(p_real_root[0] - p_real_root[1]) < 1e-10 &&
			   fabs((*this)(p_real_root[1])) > 1e-6) {*/ // this is letting complex roots through
				//_ASSERTE(!b_sort_roots); // sorting not implemented for complex roots
				//p_real_root[2] = p_real_root[1]; // complex conjugate of it
				//p_im_root[1] = -T(f_im);
				//p_im_root[2] = T(f_im);
				n_real_root_num = 1; // the other two are complex conjugates
			} else {
				n_real_root_num = 2; // the other root is double
				if(b_sort_roots) // compile-time constant
					cswap(p_real_root[0], p_real_root[1]); // sort roots (would have to compare s + t > 0 anyway, nothing saved)
			}

			return;
		}
		// one root real, two are complex

		//p_im_root[1] = 0;
		//p_im_root[2] = 0;
		// the remaining options are all real

		if(/*fabs(disc) == 0*/disc == 0) { // no thresh, more roots are better than missing a root
			bool b_r_neg = r < 0;
			int i = (b_sort_roots)? ((b_r_neg)? 1 : 0) : 0; // reuses comparison from the above line (maybe)
			double r13 = (b_r_neg)? -pow(-r, 1.0 / 3) : pow(r, 1.0 / 3);
			p_real_root[1 - i] = T(f_root_off + 2 * r13);
			p_real_root[i] = T(f_root_off - r13);
			_ASSERTE(!b_sort_roots || p_real_root[0] <= p_real_root[1]); // make sure the roots are sorted
			n_real_root_num = 2; // the second root is a double one
			return;
		}
		// all roots real, at least two are equal

		// this last part is really imprecise, try to use cardano's method instead
		// of the trigonometric one, that should improve it somewhat
		// the other parts are generally about 50 bits precise, which is quite ok

		q = -q;
		double f_sqrt_q = sqrt(q);
		double f_mag = 2 * /*sqrt(q)*/f_sqrt_q;
		double f_arg = r / (q * f_sqrt_q)/*sqrt(q * q * q)*/; // slightly more precise

		if(f_arg >= 1.0) { // does not help much, generally the cos does a good job, loses one bit of precision or so
			// theta = 0
			p_real_root[1] = T(f_root_off + f_mag); // cos(0) = 1
			p_real_root[0] = T(f_root_off - f_mag / 2); // cos(2 * pi / 3) = -.5 // note that f_mag is always nonnegative, the roots are therefore trivially sorted
			//p_real_root[2] = T(f_root_off - f_mag / 2); // cos(4 * pi / 3) = -.5
			n_real_root_num = 2; // !!
		} else if(f_arg <= -1.0) { // does not help much, generally the cos does a good job, loses one bit of precision or so
			// theta = pi / 3
			p_real_root[1] = T(f_root_off + f_mag / 2); // cos(pi / 3) = .5
			p_real_root[0] = T(f_root_off - f_mag); // cos(3 * pi / 3) = -1 // note that f_mag is always nonnegative, the roots are therefore trivially sorted
			//p_real_root[2] = T(f_root_off + f_mag / 2); // cos(5 * pi / 3) = .5
			n_real_root_num = 2; // !!
		} else {
			// three cosines: on average 49.39 bits, 51x 54 bits answer, worst case 24 bits (not counting the 5 bits missed root)
			// phasors: on average 50.555 bits, 62x 54 bits answer, worst case 24 bits (not counting the 5 bits missed root)
			// phasors + tan trick: 50.235 bits, 71x 54 bits answer, worst case 24 bits (not counting the 5 bits missed root)
#if 1
			double ax = /*max(-1.0, min(1.0,*/ f_arg/*))*/;
			_ASSERTE(ax >= -1.0 && ax <= 1.0); // limited above

#define __CUBIC_EQ_USE_TANGENT_TRICK 0
#if !__CUBIC_EQ_USE_TANGENT_TRICK
			ax = cos(acos(ax) / 3);
#endif // !__CUBIC_EQ_USE_TANGENT_TRICK

			double ay = sqrt(1.0 - ax * ax);
			// coordinates of the phasor, corresponding to the first root

#if __CUBIC_EQ_USE_TANGENT_TRICK
			{
				//double sx = ay * tan(M_PI_2 - acos(ax) / 3); // 50.225 bits
				_ASSERTE(ax > -1.0 && ax < 1.0); // must not equal, otherwise sx will break stuff
				double sx = ay / tan(acos(ax) / 3); // 50.235 bits
				//double sx = ay * cos(acos(ax) / 3) / sin(acos(ax) / 3); // 50.225 bits
				// horrific, this is actually one bit more precise on AMD CPU, both vs 2008 and msvc 6.0

				const double sy = ay; // just rename
				double f_renorm = sqrt(sx * sx + sy * sy);
				ax = sx / f_renorm;
				ay = sy / f_renorm;
			}
#endif // __CUBIC_EQ_USE_TANGENT_TRICK

			//double bx = -ay, by = ax;
			// a perpendicular phasor, rotated CCW

			// shift by (2 pi / 3) corresponds to (-.5, .5 * sqrt(3))

			const double m0 = -.5, m1 = .5 * sqrt(3.0);
			// avoid using trigonometric functions, just use vector composition (less complicated transcendental operations required)

			//double cx = ax * m0 + bx * m1;
			double cx = ax * m0 - ay * m1; // equals the above
			//double cy = ay * m0 + by * m1; // unused
			// calculate a phasor, rotated by 120 degrees CCW

			//double dx = ax * m0 - bx * m1;
			double dx = ax * m0 + ay * m1; // equals the above
			//double dy = ay * m0 - by * m1; // unused
			// calculate a phasor, rotated by 240 degrees CCW

			p_real_root[0] = f_root_off + f_mag * ax;
			p_real_root[1] = f_root_off + f_mag * cx;
			p_real_root[2] = f_root_off + f_mag * dx;

			// this is actually astonishingly precise, todo - mirror this to the simple cubic MPA

			n_real_root_num = 3; // note that there still can be equal roots since cos is periodic

			if(b_sort_roots) {
				cswap(p_real_root[0], p_real_root[1]);
				cswap(p_real_root[1], p_real_root[2]);
				cswap(p_real_root[0], p_real_root[1]);
			}
			// this needs to be sorted manually, as cos() is periodic and there is no good insight into the order
#else // 1
			const double f_pi__ = M_PI;//3.141592653589793238462643383279502884; // more decimals do not gain any extra precision (at least in VS2008)
			double f_theta = acos(/*max(-1.0, min(1.0, */f_arg/*))*/) / 3;
			//double f_err_0 = (t_Expansion(f_mag) * cos(f_theta) - (f_mag * cos(f_theta))).f_Approximate();
			//double f_err_1 = ((t_Expansion(f_root_off) + (f_mag * cos(f_theta))) - (f_root_off + f_mag * cos(f_theta))).f_Approximate();
			p_real_root[0] = T(f_root_off + f_mag * cos(f_theta));
			//double f_err_6 = ((t_Expansion(f_theta) + (2 * M_PI / 3)) - (f_theta + 2 * M_PI / 3)).f_Approximate();
			//double f_err_2 = (t_Expansion(f_mag) * cos(f_theta + 2 * M_PI / 3) - (f_mag * cos(f_theta + 2 * M_PI / 3))).f_Approximate();
			//double f_err_3 = ((t_Expansion(f_root_off) + (f_mag * cos(f_theta + 2 * f_pi__ / 3))) - (f_root_off + f_mag * cos(f_theta + 2 * M_PI / 3))).f_Approximate();
			p_real_root[1] = T(f_root_off + f_mag * cos(f_theta + 2 * f_pi__ / 3));
			//double f_err_7 = ((t_Expansion(f_theta) + (4 * M_PI / 3)) - (f_theta + 4 * M_PI / 3)).f_Approximate();
			//double f_err_4 = (t_Expansion(f_mag) * cos(f_theta + 4 * M_PI / 3) - (f_mag * cos(f_theta + 4 * M_PI / 3))).f_Approximate();
			//double f_err_5 = ((t_Expansion(f_root_off) + (f_mag * cos(f_theta + 4 * f_pi__ / 3))) - (f_root_off + f_mag * cos(f_theta + 4 * M_PI / 3))).f_Approximate();
			p_real_root[2] = T(f_root_off + f_mag * cos(f_theta + 4 * f_pi__ / 3));

			n_real_root_num = 3; // note that there still can be equal roots since cos is periodic

			if(b_sort_roots) {
				cswap(p_real_root[0], p_real_root[1]);
				cswap(p_real_root[1], p_real_root[2]);
				cswap(p_real_root[0], p_real_root[1]);
			}
			// this needs to be sorted manually, as cos() is periodic and there is no good insight into the order
#endif // 1
		}
		// the only option left is that all roots are real and unequal (to get here, q < 0)
		// this is actually a cube root of a (complex) number?
	}

	/**
	 *	@brief gets number of real roots
	 *	@return Returns number of real roots (0 to 3).
	 */
	size_t n_RealRoot_Num() const
	{
		_ASSERTE(n_real_root_num >= 0);
		return n_real_root_num;
	}

	/**
	 *	@brief gets value of a real root
	 *	@param[in] n_index is zero-based index of the root
	 *	@return Returns value of the specified root.
	 */
	T f_RealRoot(size_t n_index) const
	{
		_ASSERTE(n_index < 3 && n_index < n_real_root_num);
		return p_real_root[n_index];
	}

protected:
	/**
	 *	@brief conditional swap, used for sorting the roots
	 *
	 *	@param[in,out] r_a is the smaller value on output
	 *	@param[in,out] r_b is the greater value on output
	 */
	static inline void cswap(T &r_a, T &r_b)
	{
		if(r_a > r_b)
			std::swap(r_a, r_b);
	}
};

/**
 *	@brief quartic equation solver
 */
template <class T = double, bool b_sort_roots = true,
	const int n_4th_order_coeff_log10_thresh = -6,
	const int n_depressed_1st_order_coeff_log10_thresh = -6, // -6 does not catch some cases, but hurts 2-root solution precision; solved by adding n_zero_discriminant_log10_thresh2
	const int n_zero_discriminant_log10_thresh = -6,
	const int n_zero_discriminant_log10_thresh2 = -3, // -6 rejects too much
	const int n_cubic_3rd_order_coeff_log10_thresh = -6,
	const int n_cubic_second_root_precision_abs_log10_thresh = -6,
	const int n_cubic_second_root_precision_rel_log10_thresh = 1,
	const int n_quad_zero_discriminant_log10_thresh = -5> // lower than above
class CQuarticEq : public CQuarticFunction<T> {
protected:
	T p_real_root[4]; /**< @brief list of the roots (real parts) */
	//T p_im_root[4]; // imaginary part of the roots
	size_t n_real_root_num; /**< @brief number of real roots */

public:
	/**
	 *	@brief default constructor; solves for roots of \f$ax^4 + bx^3 + cx^2 + dx + e = 0\f$
	 *
	 *	This finds roots of the given equation. It tends to find two identical roots instead of one, rather
	 *	than missing one of two different roots - the number of roots found is therefore orientational,
	 *	as the roots might have the same value.
	 *
	 *	@param[in] _a is 4th order coefficient
	 *	@param[in] _b is 3rd order coefficient
	 *	@param[in] _c is the 2nd order coefficient
	 *	@param[in] _d is the 1st order coefficient
	 *	@param[in] _e is constant coefficient
	 */
	CQuarticEq(T _a, T _b, T _c, T _d, T _e)
		:CQuarticFunction<T>(_a, _b, _c, _d, _e), n_real_root_num(0)
	{
		if(fabs(_a) < f_Power_Static(10, n_4th_order_coeff_log10_thresh)) { // otherwise division by a yields large numbers, this is then more precise
			CCubicEq<T, b_sort_roots, n_cubic_3rd_order_coeff_log10_thresh,
				n_cubic_second_root_precision_abs_log10_thresh,
				n_cubic_second_root_precision_rel_log10_thresh,
				n_quad_zero_discriminant_log10_thresh> eq3(_b, _c, _d, _e);
			n_real_root_num = eq3.n_RealRoot_Num();
			for(unsigned int i = 0; i < n_real_root_num; ++ i) {
				p_real_root[i] = eq3.f_RealRoot(i);
				//p_im_root[i] = eq3.f_ImagRoot(i);
			}
			return;
		}
		// the highest power is multiplied by 0, it is a cubic equation

		if(fabs(_e) == 0) {
			CCubicEq<T, b_sort_roots, n_cubic_3rd_order_coeff_log10_thresh,
				n_cubic_second_root_precision_abs_log10_thresh,
				n_cubic_second_root_precision_rel_log10_thresh,
				n_quad_zero_discriminant_log10_thresh> eq3(_a, _b, _c, _d);
			n_real_root_num = eq3.n_RealRoot_Num();
			for(unsigned int i = 0; i < n_real_root_num; ++ i) {
				p_real_root[i] = eq3.f_RealRoot(i);
				//p_im_root[i] = eq3.f_ImagRoot(i);
			}

			p_real_root[n_real_root_num] = 0; // x = 0 is also a solution!
			++ n_real_root_num;
			if(b_sort_roots)
				std::sort(p_real_root, p_real_root + n_real_root_num);
			// this needs to be sorted manually, as cos() is periodic and there is no good insight into the order

			return;
		}
		// the constant is 0, divide the whole equation by x to get a cubic equation

		if(fabs(_b) == 0 && fabs(_d) == 0) { // this possibly needs to be done with a threshold
			CQuadraticEq<T, b_sort_roots, -6, n_quad_zero_discriminant_log10_thresh> eq2(_a, _c, _e);
			for(int i = 0, n = int(eq2.n_RealRoot_Num()); i < n; ++ i) {
				T y = eq2.f_RealRoot(i);
				if(y < 0) {
					if(y > -f_Power_Static(10, n_zero_discriminant_log10_thresh)) // only slightly negative
						p_real_root[n_real_root_num ++] = 0;
					continue; // this root would be imaginary
				}
				y = sqrt(y);
				if(y > 0)
					p_real_root[n_real_root_num ++] = -y; // generate the roots in approximately sorted order, avoid negative zeros
				p_real_root[n_real_root_num ++] = y;
			}
			// solve a biquadratic equation a*x^4 + c*x^2 + e = 0
		} else {
#if 0 // this first branch seems to be slightly less precise in the worst case
			_b /= _a;
			_c /= _a;
			_d /= _a;
			_e /= _a;
			// normalize

			T f_roots_offset = _b / 4;
			T f_alpha = _c - 3.0 / 8 * _b * _b; // p // ok
			T f_beta = _b * _b * _b / 8 - _b * _c / 2 + _d; // q // ok
			T f_gamma = -3.0 / 256 * _b * _b * _b * _b + _e - 64.0 / 256 * _d * _b + 16.0 / 256 * _b * _b * _c; // r
#else // 0
			const T a4 = _a, a3 = _b, a2 = _c, a1 = _d, a0 = _e; // just rename, no normalization
			T f_roots_offset = a3 / (4 * a4);
			T f_alpha = (8 * a2 * a4 - 3 * a3 * a3) / (8 * a4 * a4);
			T f_beta = (a3 * a3 * a3 - 4 * a2 * a3 * a4 + 8 * a1 * a4 * a4) / (8 * a4 * a4 * a4);
			T f_gamma = (-3 * a3 * a3 * a3 * a3 + 256 * a0 * a4 * a4 * a4 -
				64 * a1 * a3 * a4 * a4 + 16 * a2 * a3 * a3 * a4) / (256 * a4 * a4 * a4 * a4);
#endif // 0
			// convert to a depressed quartic u^4 + f_alpha*u^2 + f_beta*u + f_gamma = 0 (where x = u - f_roots_offset)

			if(fabs(f_beta) < f_Power_Static(10, n_depressed_1st_order_coeff_log10_thresh)) {
				CQuadraticEq<T, b_sort_roots, -6, n_quad_zero_discriminant_log10_thresh> eq2(1, f_alpha, f_gamma);
				for(int i = 0, n = int(eq2.n_RealRoot_Num()); i < n; ++ i) {
					T y = eq2.f_RealRoot(i);
					if(y < 0) {
						if(y > -f_Power_Static(10, n_zero_discriminant_log10_thresh)) // only slightly negative
							p_real_root[n_real_root_num ++] = 0 - f_roots_offset;
						continue; // this root would be imaginary
					}
					y = sqrt(y);
					p_real_root[n_real_root_num ++] = -y - f_roots_offset;
					if(y > 0)
						p_real_root[n_real_root_num ++] = y - f_roots_offset; // generate the roots in approximately sorted order
				}
				// solve a depressed quartic u^4 + f_alpha*u^2 + f_gamma = 0
			} else {
				T f_cub_a = 1, f_cub_b = 5.0 / 2 * f_alpha, f_cub_c = 2 * f_alpha * f_alpha - f_gamma,
					f_cub_d = (f_alpha * f_alpha * f_alpha - f_alpha * f_gamma) / 2 - f_beta * f_beta / 8;
				// form a cubic, which gets the parameter y for Ferrari's method

				CCubicEq<T, b_sort_roots, n_cubic_3rd_order_coeff_log10_thresh,
					n_cubic_second_root_precision_abs_log10_thresh,
					n_cubic_second_root_precision_rel_log10_thresh,
					n_quad_zero_discriminant_log10_thresh> eq3(f_cub_a, f_cub_b, f_cub_c, f_cub_d);
				for(int i = 0, n = int(eq3.n_RealRoot_Num()); i < n; ++ i) {
					const T y = eq3.f_RealRoot(i);
					_ASSERTE(fabs(f_alpha + 2 * y) > 0); // if this triggers, we have apparently missed to detect a biquadratic equation above (would cause a division by zero later on)
					// get a root of the cubic equation

					T f_a2y = f_alpha + 2 * y;
					if(f_a2y < 0)
						continue; // the root would be an imaginary one
					f_a2y = (T)sqrt(f_a2y);
					// get square root of alpha + 2y

					{
						T f_det0 = -(3 * f_alpha + 2 * y + 2 * f_beta / f_a2y);
						if(f_det0 >= 0) {
							double f_det_sqrt = sqrt(f_det0);
							p_real_root[n_real_root_num ++] = (f_a2y + f_det_sqrt) / 2 - f_roots_offset;
							if(f_det_sqrt > 0) // otherwise they are the same
								p_real_root[n_real_root_num ++] = (f_a2y - f_det_sqrt) / 2 - f_roots_offset;
						} else if(f_det0 > -f_Power_Static(10, n_zero_discriminant_log10_thresh2)) { // only slightly negative
							double f_det_sqrt = 0;
							p_real_root[n_real_root_num ++] = (f_a2y + f_det_sqrt) / 2 - f_roots_offset;
						}
					}
					{
						T f_det1 = -(3 * f_alpha + 2 * y - 2 * f_beta / f_a2y);
						if(f_det1 >= 0) {
							double f_det_sqrt = sqrt(f_det1);
							p_real_root[n_real_root_num ++] = (-f_a2y + f_det_sqrt) / 2 - f_roots_offset;
							if(f_det_sqrt > 0) // otherwise they are the same
								p_real_root[n_real_root_num ++] = (-f_a2y - f_det_sqrt) / 2 - f_roots_offset;
						} else if(f_det1 > -f_Power_Static(10, n_zero_discriminant_log10_thresh2)) { // only slightly negative
							double f_det_sqrt = 0;
							p_real_root[n_real_root_num ++] = (-f_a2y + f_det_sqrt) / 2 - f_roots_offset;
						}
					}

					break; // the same results are obtained for any value of y
				}
				// find roots using Ferrari's method
			}
		}

		if(b_sort_roots)
			std::sort(p_real_root, p_real_root + n_real_root_num);
		// sort the roots explicitly (we could use an ordered merge above,
		// but that would convolute the code somewhat)
	}

	/**
	 *	@brief gets number of real roots
	 *	@return Returns number of real roots (0 to 3).
	 */
	size_t n_RealRoot_Num() const
	{
		_ASSERTE(n_real_root_num >= 0);
		return n_real_root_num;
	}

	/**
	 *	@brief gets value of a real root
	 *	@param[in] n_index is zero-based index of the root
	 *	@return Returns value of the specified root.
	 */
	T f_RealRoot(size_t n_index) const
	{
		_ASSERTE(n_index < 4 && n_index < n_real_root_num);
		return p_real_root[n_index];
	}
};

#endif // '__POLYNOMIAL_SOLVERS_INCLUDED
