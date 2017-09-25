/*
								+----------------------------------+
								|                                  |
								|   ***   Distortion model   ***   |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|        DistortionModel.h         |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __DISTORTION_MODEL_INCLUDED
#define __DISTORTION_MODEL_INCLUDED

/**
 *	@file geometry/DistortionModel.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief basic camera distortion models
 */

#include "geometry/Polynomial.h"

/**
 *	@brief distortion model internal helpers
 */
namespace dmod_detail {

/**
 *	@brief a simple comparison function for vectors
 */
struct TCompareFirstElem {
	template <class CVector>
	bool operator ()(const CVector &r_a, const CVector &r_b) const
	{
		return r_a(0) < r_b(0);
	}

	template <class CVector>
	bool operator ()(const CVector &r_a, double f_b) const
	{
		return r_a(0) < f_b;
	}

	template <class CVector>
	bool operator ()(double f_a, const CVector &r_b) const
	{
		return f_a < r_b(0);
	}
};

} // ~dmod_detail

/**
 *	@brief a simple polynomial class
 *
 *	While the Brown distortion model has the form \f$x_d = x_u (1 + K_1 r^2 + K_2 r^4 + \ldots)\f$
 *	where \f$r = \text{norm}((x_u - x_c)^2 + (y_u - y_c)^2)\f$, this model is subtly different by
 *	handling the center more precisely \f$x_d = (x_u - x_c) (1 + K_1 r^2 + K_2 r^4 + \ldots) + x_c\f$
 *	(note the \f$x_c\f$ subtracted on the left and added on the right). While the result is similar,
 *	the directions of distortion in this model are always passing through the centre of projection
 *	(while in Browns, they only pass through the centre of projection approximately, if the distortion
 *	and excentricity is very small). This model is therefore more suitable for the fisheye lenses.
 *
 *	If needed, another template parameter could be added, changing this behavior to vanilla Brown.
 *
 *	@tparam n_coeff_num is number of distortion coeffifient
 *	@tparam n_first_coeff_power is power of the first coefficient
 *	@tparam n_power_step is difference of powers of adjancent coefficients
 *	@tparam CScalar is scalar data type for the evaluation of the polynomial
 */
template <const int n_coeff_num, const int n_first_coeff_power = 1,
	const int n_power_step = 1, class CScalar = double>
class CRadialDistortionModel {
public:
	/**
	 *	@brief model parameters, stored as enum
	 */
	enum {
		coeff_Num = n_coeff_num, /**< @brief number of coefficients */
		firstCoeff_Power = n_first_coeff_power, /**< @brief only even powers */
		power_Step = (n_coeff_num)? n_power_step : 0 /**< @brief only even ot only odd powers */

		// note that e.g. DNeg uses three odd powers for their radial distortion polynomials
	};

	typedef CScalar _TyScalar; /**< @brief distortion paramteters */
	typedef Eigen::Matrix<_TyScalar, coeff_Num, 1> _TyVector; /**< @brief a vector type of the same dimension as the polynomial */
	typedef Eigen::Matrix<_TyScalar, coeff_Num, coeff_Num> _TyMatrix; /**< @brief a square matrix type of the same dimension as the polynomial */
	typedef _TyVector _TyParams; /**< @brief distortion paramteters */
	typedef CPolynomial<n_coeff_num, n_first_coeff_power, n_power_step, CScalar> _TyPolynomial;

	typedef Eigen::Matrix<_TyScalar, 2, 1> Vector2; /**< @brief 2D vector type (used in least squares fitting) */
	typedef Eigen::Matrix<_TyScalar, 2, 1, Eigen::DontAlign> Vector2_u; /**< @brief unaligned 2D vector type (used in least squares fitting) */

protected:
	Vector2 m_v_center; /**< @brief position of the center of distortion */
	_TyPolynomial m_poly; /**< @brief radial distortion polynomial */

public:
	/**
	 *	@brief default constructor; copies the distortion parameters
	 *
	 *	@param[in] r_v_center is position of the center of distortion (default (0, 0))
	 *	@param[in] r_v_parameters is vector of coefficients for the radial distortion polynomial (default a null vector)
	 */
	inline CRadialDistortionModel(const Vector2 &r_v_center = Vector2::Zero(),
		const _TyParams &r_v_parameters = _TyParams::Zero()) // must be a reference, since it can't be aligned in msvc
		:m_v_center(r_v_center), m_poly(r_v_parameters)
	{}

	/**
	 *	@brief constructor for a polynomial with just a single argument
	 *
	 *	@param[in] r_v_center is position of the center of distortion
	 *	@param[in] f_parameter0 is the first and only argument of the radial distortion polynomial
	 */
	//template <class CParamInitializer>
	// *	@tparam CParamInitializer is radial distortion polynomial initializer argument (e.g. double)
	inline CRadialDistortionModel(const Vector2 &r_v_center, _TyScalar f_parameter0)
		:m_v_center(r_v_center), m_poly(f_parameter0)
	{}

	/**
	 *	@brief access to distortion parameters
	 *	@return Returns reference to the parameter vector.
	 */
	_TyParams &v_Parameters()
	{
		return m_poly.v_Parameters();
	}

	/**
	 *	@brief access to distortion parameters
	 *	@return Returns const reference to the parameter vector.
	 */
	const _TyParams &v_Parameters() const
	{
		return m_poly.v_Parameters();
	}

	/**
	 *	@brief access to distortion center
	 *	@return Returns reference to the distortion center position.
	 */
	Vector2 &v_Center()
	{
		return m_v_center;
	}

	/**
	 *	@brief access to distortion center
	 *	@return Returns const reference to the distortion center position.
	 */
	const Vector2 &v_Center() const
	{
		return m_v_center;
	}

	/**
	 *	@brief applies the distortion to a 2D point
	 *	@param[in] r_v_point is an (undistorted) point 
	 *	@return Returns the corresponding (distorted) point.
	 *	@note Note that this is fast, this should bring the undistorted re-projected
	 *		coordinates to distorted coordinates, in the bundler inner loop.
	 */
	Vector2 v_Apply(const Vector2 &r_v_point) const
	{
		Vector2 v_point = r_v_point - m_v_center;
		return v_point * (1 + m_poly.v_Parameters().dot(v_PolyWeights(v_point))) + m_v_center;
	}

	/**
	 *	@brief applies inverse distortion to a 2D point
	 *
	 *	@param[in] r_v_point is a (distorted) point
	 *
	 *	@return Returns the corresponding (undistorted) point.
	 *
	 *	@note This uses a numerical Newton-like method to find the inverse point. This does
	 *		not always converge to the same point that maps back to the distorted one (the
	 *		distortion polynomial often has multiple different roots).
	 *	@note Note that this is slow (compared to \ref v_Apply), this should be used in the
	 *		vision frontend for camera pose initialization using P3P which is needed only
	 *		for a small subset of the points (the ones observed by the camera being added).
	 */
	Vector2 v_UnApply(const Vector2 &r_v_point) const
	{
		Vector2 v_point = r_v_point - m_v_center;
		_TyScalar f_r = v_point.norm();
		if(!f_r)
			return r_v_point/*v_point + m_v_center*/;
		// a point exactly in the center is not affected

		Vector2 v_unit_dir = v_point / f_r;
		// calculate unit direction

		// todo - can use analytic cubic / quartic solvers for specific cases of small polynomials

		// todo - see if it is easy or more precise to solve this using eigenvalues
		{
			_TyParams v_weight = v_PolyWeights(v_point);
			// calculate the polynomial weight vector

			Vector2 v_initial_guess = v_point * (1 - v_Parameters().dot(v_weight)) + m_v_center; // SSE
			// take an initial guess (just changed the sign of the dot product)

			Vector2 v_solution = v_initial_guess;
			Vector2 v_prev_solution;
			_TyScalar f_prev_error;
			_TyScalar f_damping = 1;
			int n_fail_num = 0;

			for(int i = 0; i < 15; ++ i) {
				//Eigen::Vector2d v_error = r_v_point - v_Apply(v_solution);
				//_TyScalar f_error_r = r_v_point.norm() - v_Apply(v_solution).norm();
				//_TyScalar f_error_r = r_v_point.dot(v_unit_dir) - v_Apply(v_solution).dot(v_unit_dir); // sign-preserving
				_TyScalar f_error_r = (r_v_point - v_Apply(v_solution)).dot(v_unit_dir); // sign-preserving
				// error of the guess

				if(i && fabs(f_prev_error) < fabs(f_error_r)) {
					f_damping *= 2;
					if(++ n_fail_num > 10)
						return v_prev_solution;
					-- i; // repeat until we have fails left
					v_solution = v_prev_solution; // return to the previous step
					f_error_r = f_prev_error;
				} else {
					f_damping = std::max(1.0, f_damping / 2);
					v_prev_solution = v_solution;
					f_prev_error = f_error_r;
				}
				// convergence check

				//v_solution += v_error / (1 + m_k.dot(v_PolyWeights_times_R_Derivative(v_solution))); // it is derivative of x * w(x), so all the powers are one higher than in v_PolyWeights()
				// note that this is not quite correct but it kind of works; I'm formulating
				// the problem as a function of x, but it is in fact a function of r
				// this diverges if the distortion is large :-/

				_TyScalar f_delta_r = f_error_r / std::max(1e-10, f_damping * (1 + // avoid division by zero
					v_Parameters().dot(v_Poly_times_R_Derivative_PolyWeights(v_solution)))); // it is derivative of x * w(x), so all the powers are one higher than in v_PolyWeights()
				//f_delta_r = (f_error_r > 0)? -fabs(f_delta_r) : fabs(f_delta_r);
				//f_delta_r = -_copysign(f_delta_r, v_error.dot(v_unit_dir));
				v_solution += v_unit_dir * f_delta_r; // offset by the calculated delta r
				// this is formulating the problem as a function of r, which is supposed to be correct
				// this works as long as the polynomial coefficients are positive
			}
			// successive approximation of the solution using damped Newton iteration

			return v_solution;
		}
	}

	/**
	 *	@brief applies inverse distortion to a 2D point
	 *
	 *	@tparam CScalar2 is scalar type for the lookup table (e.g. <tt>float</tt> if the inverse
	 *		lookup is performed in graphics hardware)
	 *
	 *	@param[in] r_v_point is a (distorted) point
	 *	@param[out] r_lookup is the generated lookup table
	 *	@param[out] r_f_lookup_max is the argument value corresponding to the last lookup table entry
	 *
	 *	@return Returns the corresponding (undistorted) point.
	 *
	 *	@note This uses a lookup from a precomputed table and if the table has sufficient range,
	 *		then this will never diverge.
	 *	@note Note that this is relatively fast (save for precomputing the table) but the expected
	 *		precision depends on the size of the lookup (for a lookup of 1024 elements, the error
	 *		is about three orders of magnitude lower than the magnitude of the corresponding point
	 *		displacement).
	 */
	template <class CScalar2>
	Vector2 v_UnApply(const Vector2 &r_v_point,
		const std::vector<CScalar2> &r_lookup, _TyScalar f_lookup_max) const
	{
		Vector2 v_point = r_v_point - m_v_center;
		_TyScalar f_r = v_point.norm();
		if(!f_r)
			return r_v_point/*v_point + m_v_center*/;
		// a point exactly in the center is not affected

		Vector2 v_unit_dir = v_point / f_r;
		// calculate unit direction

		_ASSERTE(!r_lookup.empty());
		_TyScalar f_lookup = f_r / f_lookup_max * (r_lookup.size() - 1);
		size_t n_lookup = size_t(f_lookup);
		_TyScalar f_frac = f_lookup - n_lookup;
		// calculate lookup index

		_TyScalar f_inv_poly;
		if(n_lookup + 1 >= r_lookup.size())
			f_inv_poly = r_lookup.back();
		else {
			CScalar2 a = r_lookup[n_lookup], b = r_lookup[n_lookup + 1];
			f_inv_poly = _TyScalar(a + (b - a) * f_frac);
		}
		// sample the lookup table

		return v_point * (1 + f_inv_poly) + m_v_center;
	}

	/**
	 *	@brief fits an approximate analytical inverse distortion model using least squares
	 *
	 *	This rarely works, due to the polynomials generally not having a guaranteed inverse.
	 *	Often, the inverted polynomials involve fractional powers of the argument. It would
	 *	be better to take a low power of the argument and fit a polynomial of a higher degree
	 *	to that instead.
	 *
	 *	@param[in] f_max_radial_arg is maximum radial argument (e.g. for a 1024 x 768 image, this is 1280)
	 *	@param[in] f_radial_arg_step is radial argument step (controls how many samples are taken; set this
	 *		to about 1/100 of <tt>f_max_radial_arg</tt>)
	 *
	 *	@return Returns the inverse distortion estimate.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	CRadialDistortionModel t_Inverse(_TyScalar f_max_radial_arg,
		_TyScalar f_radial_arg_step) const // throw(std::bad_alloc)
	{
		std::vector<Vector2_u> poly_samples;
		for(_TyScalar u = 0; u < f_max_radial_arg; u += f_radial_arg_step) { // u is the undistorted radius
			_TyScalar qu = m_poly(u); // poly distorts the image, we go from undistorted to distorted
			_TyScalar d = u * (1 + qu); // d is the distorted radius
			_TyScalar qd = _TyScalar(1.0) / (1 + qu) - 1; // qd is the inverse polynomial m_poly'(d)
			/*
				d = u(1 + p(u)) // distort
				u = d(1 + p'(d)) // undistort
				d / (1 + p(u)) = d(1 + p'(d))
				1 / (1 + p(u)) = (1 + p'(d))
				1 / (1 + p(u)) - 1 = p'(d) // the undistiortion polynomial
			*/

			if(!poly_samples.empty() && poly_samples.back()(0) > d)
				break;
			// only up to the first inflection point otherwise not invertible below

			poly_samples.push_back(Vector2_u(d, qd));
		}
		// sample the radial distortion polynomial

		_TyPolynomial inv_poly;
		inv_poly.LeastSquares_Fit(poly_samples);
		inv_poly.Robust_Fit_Update(poly_samples, CHuberLoss<>()); // essential

		return CRadialDistortionModel(m_v_center, inv_poly.v_Parameters());
	}

	/**
	 *	@brief builds an inverse polynomial value lookup table for an inverse distortion model
	 *
	 *	@tparam CScalar2 is scalar type for the lookup table (e.g. <tt>float</tt> if the inverse
	 *		lookup is performed in graphics hardware)
	 *
	 *	@param[out] r_lookup is the generated lookup table
	 *	@param[out] r_f_lookup_max is the argument value corresponding to the last lookup table entry
	 *	@param[in] n_lookup_size is size of the lookup table (e.g. 1024 entries)
	 *	@param[in] f_max_radial_arg is maximum radial argument (e.g. for a 1024 x 768 image, this is 1280)
	 *	@param[in] f_radial_arg_step is radial argument step (controls how many samples are taken; set this
	 *		to about 1/100 of <tt>f_max_radial_arg</tt>)
	 *
	 *	@return Returns the inverse distortion estimate.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CScalar2>
	void Inverse_PolynomialLookup(std::vector<CScalar2> &r_lookup,
		_TyScalar &r_f_lookup_max, size_t n_lookup_size,
		_TyScalar f_max_radial_arg, _TyScalar f_radial_arg_step) // throw(std::bad_alloc)
	{
		std::vector<Vector2_u> poly_samples;
		for(_TyScalar u = 0; u < f_max_radial_arg; u += f_radial_arg_step) { // u is the undistorted radius
			_TyScalar qu = m_poly(u); // poly distorts the image, we go from undistorted to distorted
			_TyScalar d = u * (1 + qu); // d is the distorted radius
			_TyScalar qd = _TyScalar(1.0) / (1 + qu) - 1; // qd is the inverse polynomial m_poly'(d)
			/*
				d = u(1 + p(u)) // distort
				u = d(1 + p'(d)) // undistort
				d / (1 + p(u)) = d(1 + p'(d))
				1 / (1 + p(u)) = (1 + p'(d))
				1 / (1 + p(u)) - 1 = p'(d) // the undistiortion polynomial
			*/

			if(!poly_samples.empty() && poly_samples.back()(0) > d)
				break;
			// only up to the first inflection point otherwise not invertible below

			poly_samples.push_back(Vector2_u(d, qd));
		}
		// sample the radial distortion polynomial

		const _TyScalar f_lookup_max = poly_samples.back()(0);
		r_f_lookup_max = f_lookup_max; // this is needed for the lookup
		r_lookup.resize(n_lookup_size);
		for(size_t i = 0; i < n_lookup_size; ++ i) {
			_TyScalar f = _TyScalar(i) / (n_lookup_size - 1) * r_f_lookup_max; // this needs to go all the way to r_f_lookup_max
			typename std::vector<Vector2_u>::const_iterator p_it =
				std::lower_bound(poly_samples.begin(), poly_samples.end(),
				f/*Vector2_u(f, f)*/, dmod_detail::TCompareFirstElem());
			_ASSERTE(p_it >= poly_samples.begin() && p_it < poly_samples.end());
			_ASSERTE((*p_it)(0) >= f);
			if((*p_it)(0) == f || p_it == poly_samples.begin())
				r_lookup[i] = (CScalar2)((*p_it)(1)); // discrete sample
			else {
				Vector2_u a = *(-- p_it);
				Vector2_u b = *(++ p_it);
				_ASSERTE(f > a(0) && f <= b(0));
				_TyScalar t = (f - a(0)) / (b(0) - a(0));
				r_lookup[i] = (CScalar2)(a(1) + t * (b(1) - a(1))); // lerp two samples
			}
		}
		// build an inverse polynomial lookup table
	}

	/**
	 *	@brief evaluates radial distortion polynomial weights for a 
	 *
	 */
	inline _TyVector v_PolyWeights(const Vector2 &r_v_point) const // note that this does not (un)apply the distortion center in any way
	{
		return m_poly.v_PolyWeights_SquaredInput(r_v_point.squaredNorm());
	}

	inline _TyVector v_Derivative_PolyWeights(const Vector2 &r_v_point) const // note that this does not (un)apply the distortion center in any way
	{
		return m_poly.v_Derivative_PolyWeights_SquaredInput(r_v_point.squaredNorm());
	}

	inline _TyVector v_Poly_times_R_Derivative_PolyWeights(const Vector2 &r_v_point) const // note that this does not (un)apply the distortion center in any way
	{
		return m_poly.v_Poly_times_Arg_Derivative_PolyWeights_SquaredInput(r_v_point.squaredNorm());
	}

	_TyPolynomial &r_BasePolynomial() // can be used to fit the polynomial to some observations but that isnt the same as fitting the distortion model
	{
		return m_poly;
	}

	const _TyPolynomial &r_BasePolynomial() const
	{
		return m_poly;
	}

	Eigen::Matrix<_TyScalar, 2, 2> t_Jacobian_Center(const Vector2 &r_v_point) const
	{
		Vector2 v_point = r_v_point - m_v_center;
		const double xcan = v_point(0), ycan = v_point(1);
		const double Jxcan = -1, Jycan = -1;
		const double Jrx = (fabs(ycan) < 1e-10)? 1 : (fabs(xcan) < 1e-10)? 0 :
			1 / (sqrt(xcan * xcan + ycan * ycan)) * xcan;
		const double Jry = (fabs(xcan) < 1e-10)? 1 : (fabs(ycan) < 1e-10)? 0 :
			1 / (sqrt(xcan * xcan + ycan * ycan)) * ycan;
		const double r = v_point.norm();
		//_TyVector Jq = m_poly.v_Derivative_PolyWeights(r); // unused here
		const double Jqr = m_poly.f_Derivative(r);
		const double Jxuq = xcan, Jyuq = ycan;
		const double Jqq = 1;
		const double q = m_poly(r);
		const double Jprodcxx = 1 * (1 + q) + xcan * (Jqq * Jqr * Jrx);
		const double Jprodcyx = 0 * (1 + q) + xcan * (Jqq * Jqr * Jry);
		const double Jprodcxy = 0 * (1 + q) + ycan * (Jqq * Jqr * Jrx);
		const double Jprodcyy = 1 * (1 + q) + ycan * (Jqq * Jqr * Jry);

		Eigen::Matrix<_TyScalar, 2, 2> J;
		J << Jprodcxx * Jxcan + 1, Jprodcxy * Jxcan, // d xu / d xc, d yu / d xc
			 Jprodcyx * Jycan, Jprodcyy * Jycan + 1; // d xu / d yc, d yu / d yc
		// columns = distorted x and distorted y
		// rows = center x and center y

		return J;
	}

	Eigen::Matrix<_TyScalar, n_coeff_num, 2> t_Jacobian_PolyCoeffs(const Vector2 &r_v_point)
	{
		Vector2 v_point = r_v_point - m_v_center;
		const double xcan = v_point(0), ycan = v_point(1);
		/*const double Jxcan = -1, Jycan = -1;
		const double Jrx = (fabs(ycan) < 1e-9)? 1 : (fabs(xcan) < 1e-9)? 0 :
			1 / (sqrt(xcan * xcan + ycan * ycan)) * xcan;
		const double Jry = (fabs(xcan) < 1e-9)? 1 : (fabs(ycan) < 1e-9)? 0 :
			1 / (sqrt(xcan * xcan + ycan * ycan)) * ycan;
		const double r = v_point.norm();*/ // unused here
		_TyVector Jq = m_poly.v_PolyWeights_SquaredInput/*v_PolyWeights_Derivative_SquaredInput*/(v_point.squaredNorm()); // this was doing something else; some confusion with respect to which the derivatives are (here we need with respect to the coeffs rather than with respect to the argument)
		//const double Jqr = m_poly.f_PolyWeights_Derivative_wrt_Arg(r); // unused here
		const double Jxuq = xcan, Jyuq = ycan;
		/*const double Jqq = 1;
		const double q = m_poly(r);
		const double Jprodcxx = 1 * (1 + q) + xcan * (Jqq * Jqr * Jrx);
		const double Jprodcyx = 0 * (1 + q) + xcan * (Jqq * Jqr * Jry);
		const double Jprodcxy = 0 * (1 + q) + ycan * (Jqq * Jqr * Jrx);
		const double Jprodcyy = 1 * (1 + q) + ycan * (Jqq * Jqr * Jry);*/ // unused here

		Eigen::Matrix<_TyScalar, n_coeff_num, 2> J;
		J.col(0) = Jxuq * Jq; // d xu / d K_i
		J.col(1) = Jyuq * Jq; // d yu / d K_i
		// columns = distorted x and distorted y
		// rows = polynomial coefficients

		return J;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*

the jacobians were calculated using the following Matlab code

syms xd yd xc yc nx ny lx0 ly0 real % const inputs
syms r r2 r4 r8 xu yu diff_x diff_y f_dot x1 y1 f_error real % intermediates
syms K0 K1 K2 real

% (xcan, ycan) = a(xd, yd)
xcan = xd - xc;
ycan = yd - yc;
Jxcan = jacobian(xcan, xc) % d xcan / d xc
Jycan = jacobian(ycan, yc) % d ycan / d yc
% the other combinations are zero

ccode(Jxcan)
ccode(Jycan)

r = sqrt(xcan^2 + ycan^2);
syms r_ xcan_ ycan_ real
r_ = sqrt(xcan_^2 + ycan_^2);
Jrx = simple(jacobian(r_, xcan_)) % d r / d xcan
Jry = simple(jacobian(r_, ycan_)) % d r / d ycan
ccode(Jrx)
ccode(Jry)
Jrx = subs(Jrx, xcan_, xcan);
Jrx = subs(Jrx, ycan_, ycan);
Jry = subs(Jry, xcan_, xcan);
Jry = subs(Jry, ycan_, ycan);

r2 = r * r;
r4 = r2 * r2;
r8 = r4 * r4;
q = K0 * r2 + K1 * r4 + K2 * r8;
syms r__ r2_ r4_ r8_ q_ real
r2_ = r__ * r__;
r4_ = r2_ * r2_;
r8_ = r4_ * r4_;
q_ = K0 * r2_ + K1 * r4_ + K2 * r8_;
Jq = simple(jacobian(q_, [K0 K1 K2])) % d q / d Ki
ccode(Jq)
Jq = subs(Jq, r__, r);
Jqr = simple(jacobian(q_, r__)) % d q / d r
ccode(Jqr)
Jqr = subs(Jqr, r__, r);

xu = xcan * (1 + q) + xc; % note the + xc term at the end
syms xu_ q_ real
xu_ = xcan * (1 + q_) + xc;
Jxuq = simple(jacobian(xu_, q_)) % d xu / d q
ccode(Jxuq)
Jxuq = subs(Jxuq, q_, q);
yu = ycan * (1 + q) + yc; % note the + yc term at the end
syms yu_ real
yu_ = ycan * (1 + q_) + yc;
Jyuq = simple(jacobian(yu_, q_)) % d yu / d q
ccode(Jyuq)
Jyuq = subs(Jyuq, q_, q);
% (xu, yu) = f(.)
syms q__ real
Jqq = jacobian(1 + q__, q__) % trivial
ccode(Jqq)

% d(1 + q) / dxcan
% q = poly(norm([xcan ycan]))

syms xcan__ ycan__ real
Jprodcxx = jacobian(xcan__, xcan__) * (1 + q) + xcan * (Jqq * Jqr * Jrx) % d xu / d xcan = xcan'*(1+q) + xcan * d (1+q)/d xcan
Jprodcyx = jacobian(xcan__, ycan__) * (1 + q) + xcan * (Jqq * Jqr * Jry) % d xu / d ycan = 0 * (1+q) + xcan * d (1+q)/d ycan
Jprodcx = [Jprodcxx * Jxcan + 1 Jprodcyx * Jycan] % d xcan / d xc -> d xu / d xc % plus one to account for the + xc term at the end
Jprodcxy = jacobian(ycan__, xcan__) * (1 + q) + ycan * (Jqq * Jqr * Jrx) % d yu / d ycan = ycan'*(1+q) + ycan * d (1+q)/d ycan
Jprodcyy = jacobian(ycan__, ycan__) * (1 + q) + ycan * (Jqq * Jqr * Jry) % d yu / d xcan = 0 * (1+q) + ycan * d (1+q)/d ycan
Jprodcy = [Jprodcxy * Jxcan Jprodcyy * Jycan + 1] % d xcan / d xc -> d yu / d xc % plus one to account for the + yc term at the end
JprodKx = Jxuq * Jq;
JprodKy = Jyuq * Jq;

Jxuc = simple(jacobian(xu, [xc yc]))
JxuK = simple(jacobian(xu, [K0 K1 K2]))
Jyuc = simple(jacobian(yu, [xc yc]))
JyuK = simple(jacobian(yu, [K0 K1 K2]))
%Jyu = simple(jacobian(yu, [K0 K1 K2]));
%Ju = [Jxu; Jyu] % 2x3 jacobian
% obtain f'(.) = d K_i / d x_u and d K_i / d y_u 

Jdiffxc = simple(simple(Jprodcx - Jxuc))
JdiffxK = simple(JprodKx - JxuK)
Jdiffyc = simple(simple(Jprodcy - Jyuc))
JdiffyK = simple(JprodKy - JyuK)

*/

#if 0

/**
 *	@brief a simple unit test
 */
void Test_RadialDM()
{
	Eigen::Vector2d c(-10, 50);
	//CRadialDistortionModel<1> distortion(c, 1e-14); // 1 + k * r^2 // analytical always less precise than the initial guess, but overall precise enough
	//CRadialDistortionModel<1> distortion(c, -1e-14); // 1 + k * r^2 // analytical always less precise than the initial guess, but overall precise enough
	//CRadialDistortionModel<1> distortion(c, 1e-13); // 1 + k * r^2 // analytical usually less precise than the initial guess, but overall precise enough
	//CRadialDistortionModel<1> distortion(c, -1e-13); // 1 + k * r^2 // analytical usually less precise than the initial guess, but overall precise enough
	//CRadialDistortionModel<1> distortion(c, 1e-12); // 1 + k * r^2 // analytical usually more precise than the initial guess, overall precise enough
	//CRadialDistortionModel<1> distortion(c, -1e-12); // 1 + k * r^2 // analytical usually more precise than the initial guess, overall precise enough
	//CRadialDistortionModel<1> distortion(c, 1e-11); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, -1e-11); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, 1e-10); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, -1e-10); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, 1e-9); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, -1e-9); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, 1e-8); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, -1e-8); // 1 + k * r^2 // converges
	CRadialDistortionModel<1> distortion(c, 1e-7); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, 1e-6); // 1 + k * r^2 // converges
	//CRadialDistortionModel<1> distortion(c, 1e-5); // 1 + k * r^2 // numerical mostly converges, analytical converges well
	//CRadialDistortionModel<1> distortion(c, -1e-5); // 1 + k * r^2 // numerical diverges, analytical converges well
	//CRadialDistortionModel<2, 2, 2> distortion(c, Eigen::Vector2d(1e-8, 3e-15)); // 1 + k0 * r^2 + k1 * r^4 // converges
	//CRadialDistortionModel<2, 2, 2> distortion(c, Eigen::Vector2d(1e-7, -3e-14)); // 1 + k0 * r^2 + k1 * r^4 // converges
	//CRadialDistortionModel<2, 2, 2> distortion(c, Eigen::Vector2d(1e-7, -3e-13)); // 1 + k0 * r^2 + k1 * r^4 // mostly converges
	//CRadialDistortionModel<2, 2, 2> distortion(c, Eigen::Vector2d(1e-7, 3e-13)); // 1 + k0 * r^2 + k1 * r^4 // converges
	//CRadialDistortionModel<2, 1, 2> distortion(c, Eigen::Vector2d(1e-5, 5e-18)); // 1 + k0 * r + k1 * r^3 // a single 1e-13 error

	CRadialDistortionModel<1> dinv;
	//CRadialDistortionModel<2, 2, 2> dinv;
	//CRadialDistortionModel<2, 1, 2> dinv;

	dinv = distortion.t_Inverse(sqrt(1024.0 * 1024 + 768 * 768), 1);

	double lookup_scale;
	std::vector<double> lookup;
	distortion.Inverse_PolynomialLookup(lookup, lookup_scale, 1024, sqrt(1024.0 * 1024 + 768 * 768), 1);

	for(int i = 0; i < 100; ++ i) {
		Eigen::Vector2d v_point = Eigen::Vector2d(1024 * (double(rand()) / RAND_MAX * 2 - 1),
			768 * (double(rand()) / RAND_MAX * 2 - 1));
		Eigen::Vector2d v_undistorted = distortion.v_Apply(v_point);
		Eigen::Vector2d v_distorted = distortion.v_UnApply(v_undistorted);
		Eigen::Vector2d v_distorted2 = dinv.v_Apply(v_undistorted);
		Eigen::Vector2d v_distorted3 = distortion.v_UnApply(v_undistorted, lookup, lookup_scale);
		double f_error = (v_point - v_distorted).norm();
		double f_error2 = (v_point - v_distorted2).norm();
		double f_error3 = (v_point - v_distorted3).norm();
		double f_amount = (v_undistorted - v_distorted).norm(); // v_distorted calculated from v_undistorted precisely
		if(f_error > 1e-10 || f_error2 > 1e-10 || f_error3 > 1e-10) {
			printf("2D point (%f, %f) distortion error: %g (anal: %g, lookup: %g, distortion amount %g)\n",
				v_point(0), v_point(1), f_error, f_error2, f_error3, f_amount);
		}
		//_ASSERTE(f_error < 1e-5);
		// try with some random points
	}
	// test the distortion inverse
}

#endif // 0

#endif // !__DISTORTION_MODEL_INCLUDED
