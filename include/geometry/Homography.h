/*
								+----------------------------------+
								|                                  |
								|  ***  Homography estimator  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|           Homography.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __HOMOGRAPHY_ESTIMATION_INCLUDED
#define __HOMOGRAPHY_ESTIMATION_INCLUDED

/**
 *	@file geometry/Homography.h
 *	@brief homography estimation
 *	@author -tHE SWINe-
 *	@date 2013
 *
 *	This is a basic homography estimation class. The arguments are points
 *	in "image" space (I) and points in "template" space (T). The equation
 *	is \f$I = HT\f$, making the arguments destination space, source space
 *	as maintained throughout the rest of the geometry module.
 *
 *	On AMD Opteron 2360 SE (Windows, VS2008, x64), it takes (all times in
 *	milliseconds):
 *
 *	point pairs                 |     4 |    10 |    20 |    50 |   100 |    200 |     500 |     1000
 *	----------------------------|-------|-------|-------|-------|-------|--------|---------|----------
 *	DLT using SVD               | 0.055 | 0.096 | 0.145 | 0.426 | 1.265 |  6.131 |  70.305 |  281.119
 *	DLT using Eigs              | 0.064 | 0.063 | 0.064 | 0.077 | 0.100 |  0.144 |   0.281 |    0.514
 *	one NLS iter / transfer     | 0.008 | 0.011 | 0.018 | 0.036 | 0.066 |  0.125 |   0.304 |    0.596
 *	one NLS iter / sym transfer | 0.012 | 0.021 | 0.034 | 0.075 | 0.144 |  0.280 |   0.685 |    1.359
 *	one NLS iter / Sampson      | 0.014 | 0.026 | 0.046 | 0.105 | 0.202 |  0.398 |   0.960 |    1.871
 *	one NLS iter / reprojection | 0.026 | 0.053 | 0.122 | 1.252 | 5.281 | 25.347 | 214.517 | 1088.837
 *	one NLS iter / reproj. + SC | 0.018 | 0.027 | 0.043 | 0.090 | 0.167 |  0.329 |   0.814 |    1.653
 *
 *	On (mobile) Corei7 2620M (Windows, VS 2008, x64), it takes (all times
 *	in milliseconds):
 *
 *	point pairs                 |     4 |    10 |    20 |    50 |   100 |    200 |     500 |     1000
 *	----------------------------|-------|-------|-------|-------|-------|--------|---------|----------
 *	DLT using SVD               | 0.032 | 0.050 | 0.075 | 0.200 | 0.602 |  2.328 |  20.638 |   75.749
 *	DLT using Eigs              | 0.039 | 0.037 | 0.039 | 0.046 | 0.059 |  0.087 |   0.169 |    0.306
 *	one NLS iter / transfer     | 0.004 | 0.007 | 0.010 | 0.021 | 0.040 |  0.077 |   0.194 |    0.385
 *	one NLS iter / sym transfer | 0.007 | 0.012 | 0.018 | 0.039 | 0.074 |  0.143 |   0.358 |    0.718
 *	one NLS iter / Sampson      | 0.008 | 0.015 | 0.026 | 0.058 | 0.113 |  0.222 |   0.556 |    1.127
 *	one NLS iter / reprojection | 0.013 | 0.028 | 0.066 | 0.495 | 1.458 |  6.214 |  96.576 |  577.126
 *	one NLS iter / reproj. + SC | 0.009 | 0.015 | 0.023 | 0.048 | 0.091 |  0.178 |   0.509 |    0.988
 *
 *	Note that the above NLS times do not involve any outlier rejection.
 *
 */

/** \addtogroup geom
 *	@{
 */

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "slam/Unused.h"
#include "slam/BlockMatrixBase.h" // DimensionCheck()

/**
 *	@def SYM_TRANSFER_USE_CHAIN_RULE
 *	@brief if defined, the symmetric transfer error objective will use chain rule to get the
 *		Jacobians; otherwise end-to-end Jacobian is used (the chain rule should be faster)
 */
#define SYM_TRANSFER_USE_CHAIN_RULE

/**
 *	@def TEST_SYM_TRANSFER_JAC
 *	@brief if defined, the symmetric transfer error objective Jacobian is computed both
 *		using chain rule and end-to-end and compated every time (only use if debugging)
 */
//#define TEST_SYM_TRANSFER_JAC

/**
 *	@def TEST_SAMPSON_JAC
 *	@brief if defined, the Sampson error objective error and Jacobian are compared
 *		to those computed using Homest package (only use if debugging)
 */
//#define TEST_SAMPSON_JAC

/**
 *	@def TEST_HOMOGRAPHY_SC_SOLVER
 *	@brief if defined, the Schur complement solver in \ref CHomography::ImproveHomography_Reprojection()
 *		is compared to the general one (only use if debugging)
 */
//#define TEST_HOMOGRAPHY_SC_SOLVER

/**
 *	@brief simple homography estimation functionality
 *
 *	This can calculate Homography using DLT and then minimize one of four nonlinear objective
 *	functions using Levenberg-Marquardt. The fastest are transfer errors (unsymmetric being
 *	slightly faster than symmetric). Reprojection error is slightly slower. Sampson error is
 *	the slowest, even despite reprojection error solver having much larger state (it also
 *	optimizes the points). Still, the difference in the costs is quite small.
 *
 *	@todo Add robust optimization function variants, to be able to handle outliers.
 */
class CHomography {
public:
	typedef Eigen::Vector2d _TyPoint2D; /**< @brief 2D point data type */
	typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> _TyPoint2D_u; /**< @brief unaligned 2D point data type */

	class CTransferErrorObjective;
	class CSymTransferErrorObjective;
	class CSampsonErrorObjective;
	class CReprojectionErrorObjective;
	// forward declarations

	typedef CSymTransferErrorObjective CDefaultErrorObjective; /**< @brief the default objective function */

protected:
	/**
	 *	@brief helper object for enabling functions for a particular objective function
	 *
	 *	@tparam CCurObjective is the type of objective function being tested
	 *	@tparam CRefObjective is the type of objective function for which the implementation is intended
	 *	@tparam CRetVal is type of return value (default <tt>void</tt>)
	 */
	template <class CCurObjective, class CRefObjective, class CRetVal = void>
	class CEnableFor {};

	/**
	 *	@brief helper object for enabling functions for a particular objective
	 *		function (specialization for enable)
	 *
	 *	@tparam CCurRefObjective is the type of both the objective function being tested
	 *		and for which the implementation is intended
	 *	@tparam CRetVal is type of return value (default <tt>void</tt>)
	 */
	template <class CCurRefObjective, class CRetVal>
	class CEnableFor<CCurRefObjective, CCurRefObjective, CRetVal> {
	public:
		typedef CRetVal T; /**< @brief the conditionally declared return value type */
	};

	/**
	 *	@brief helper object for enabling functions for all but a particular objective function
	 *
	 *	@tparam CCurObjective is the type of objective function being tested
	 *	@tparam CRefObjective is the type of objective function for which the implementation is intended
	 *	@tparam CRetVal is type of return value (default <tt>void</tt>)
	 */
	template <class CCurObjective, class CRefObjective, class CRetVal = void>
	class CEnableNotFor {
	public:
		typedef CRetVal T; /**< @brief the conditionally declared return value type */
	};

	/**
	 *	@brief helper object for enabling functions for all but a particular objective
	 *		function (specialization for disable)
	 *
	 *	@tparam CCurRefObjective is the type of both the objective function being tested
	 *		and for which the implementation is intended
	 *	@tparam CRetVal is type of return value (default <tt>void</tt>)
	 */
	template <class CCurRefObjective, class CRetVal>
	class CEnableNotFor<CCurRefObjective, CCurRefObjective, CRetVal> {};

public:
	/**
	 *	@brief calculate homography using DLT and NLS improvement, no outlier rejection scheme applied here
	 *
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] b_normalize is normalization flag (DLT requires the points to be normalized, unless they already are)
	 *	@param[in] b_use_eigenvectors is eigendecomposition computation flag (if set, eigenvectors are used rather
	 *		than singular vectors to estimate the DLT homography; using eigenvalues is faster, less precise,
	 *		uses less memory and potentially does not make any dynamic allocations in the DLT part of the algorithm)
	 *	@param[in] n_max_NLS_iteration_num is the maximum number of NLS improvement iterations
	 *	@param[in] b_verbose is verbose flag (if set, staistics of NLS iterations will be printed
	 *		to stdout; disabled by default)
	 *
	 *	@return Returns a homography \f$H\f$ such that \f$I_i = HT_i\f$ where \f$I\f$ is the vector
	 *		of image points and \f$T\f$ is the vector of template points.
	 *
	 *	This uses the default objective for NLS refinement.
	 */
	static inline Eigen::Matrix3d t_Homography(const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template, bool b_normalize = true,
		bool b_use_eigenvectors = false, int n_max_NLS_iteration_num = 100, bool b_verbose = false) // throw(std::bad_alloc)
	{
		return t_Homography_Int<CDefaultErrorObjective>(r_points_image, r_points_template,
			b_normalize, b_use_eigenvectors, n_max_NLS_iteration_num, b_verbose);
	}

	/**
	 *	@brief calculate homography using DLT and NLS improvement, no outlier rejection scheme applied here
	 *
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] b_normalize is normalization flag (DLT requires the points to be normalized, unless they already are)
	 *	@param[in] b_use_eigenvectors is eigendecomposition computation flag (if set, eigenvectors are used rather
	 *		than singular vectors to estimate the DLT homography; using eigenvalues is faster, less precise,
	 *		uses less memory and potentially does not make any dynamic allocations in the DLT part of the algorithm)
	 *	@param[in] n_max_NLS_iteration_num is the maximum number of NLS improvement iterations
	 *	@param[in] b_verbose is verbose flag (if set, staistics of NLS iterations will be printed
	 *		to stdout; disabled by default)
	 *
	 *	@return Returns a homography \f$H\f$ such that \f$I_i = HT_i\f$ where \f$I\f$ is the vector
	 *		of image points and \f$T\f$ is the vector of template points.
	 */
	template <class CObjective>
	static inline Eigen::Matrix3d t_Homography(const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template, bool b_normalize = true,
		bool b_use_eigenvectors = false, int n_max_NLS_iteration_num = 100, bool b_verbose = false)
	{
		return t_Homography_Int<CObjective>(r_points_image, r_points_template, b_normalize,
			b_use_eigenvectors, n_max_NLS_iteration_num, b_verbose);
	}

	/**
	 *	@brief normalizes a sets of points and returns the corresponding transformaton matrix
	 *	@param[in,out] r_points are 2D points to be normalized
	 *	@return Returns the transform that has been applied to the points (not its inverse).
	 */
	static Eigen::Matrix3d t_NormalizePoints(std::vector<_TyPoint2D_u> &r_points)
	{
		_TyPoint2D v_center(0, 0);
		const size_t n_point_num = r_points.size();
		for(size_t i = 0; i < n_point_num; ++ i)
			v_center += r_points[i];
		v_center /= double(n_point_num);
		// calculate average position

		_TyPoint2D v_scale(0, 0);
		for(size_t i = 0; i < n_point_num; ++ i) {
			r_points[i] -= v_center;
			v_scale += r_points[i].array().abs().matrix();
		}
		// shift the points in both lists to zero respective means

		if(v_scale(0) == 0)
			v_scale(0) = 1;
		if(v_scale(1) == 0)
			v_scale(1) = 1;
		// in case all the points are on plane in any dimension,
		// do not apply scaling, to make the transform invertible

		v_scale = (double(n_point_num) / v_scale.array()).matrix();
		for(size_t i = 0; i < n_point_num; ++ i)
			r_points[i].array() *= v_scale.array();
		// scale the points to have average absolute coordinate 1 (separate in x and in y)

		v_center.array() *= v_scale.array(); // translation comes before scaling
		Eigen::Matrix3d t_transform;
		t_transform << v_scale(0),          0, -v_center(0),
								0, v_scale(1), -v_center(1),
								0,          0,            1;
		// calculate matrix of the transformation that we just performed on the points

		return t_transform.inverse();
	}

	/**
	 *	@brief estimate homography using DLT
	 *
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] b_use_eigenvectors is eigendecomposition computation flag (if set, eigenvectors are used rather
	 *		than singular vectors to estimate the DLT homography; using eigenvalues is faster, less precise,
	 *		uses less memory and potentially does not make any dynamic allocations in the DLT part of the algorithm)
	 *
	 *	@return Returns a homography \f$H\f$ such that \f$I_i = HT_i\f$ where \f$I\f$ is the vector
	 *		of image points and \f$T\f$ is the vector of template points.
	 *
	 *	@note This requires normalized points.
	 */
	static Eigen::Matrix3d t_DLT_Homography(const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template, bool b_use_eigenvectors = false) // throw(std::bad_alloc)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		Eigen::Matrix3d H_DLT;
		if(!b_use_eigenvectors) {
			typedef Eigen::Matrix<double, Eigen::Dynamic, 9, Eigen::RowMajor> _TyDLTMatrix;
			_TyDLTMatrix A(2 * n_point_num, 9);
			for(size_t i = 0; i < n_point_num; ++ i) {
				_TyPoint2D q = r_points_template[i], x = r_points_image[i]; // q is Hartley's x, x is x'
				A(2 * i + 0, 0) = 0;
				A(2 * i + 0, 1) = 0; // 0\tr
				A(2 * i + 0, 2) = 0;
				A(2 * i + 0, 3) = -1 * q(0);
				A(2 * i + 0, 4) = -1 * q(1); // -w'x\tr
				A(2 * i + 0, 5) = -1 * 1;
				A(2 * i + 0, 6) = x(1) * q(0);
				A(2 * i + 0, 7) = x(1) * q(1); // y'x\tr
				A(2 * i + 0, 8) = x(1) * 1;

				A(2 * i + 1, 0) = 1 * q(0);
				A(2 * i + 1, 1) = 1 * q(1); // w'x\tr
				A(2 * i + 1, 2) = 1 * 1;
				A(2 * i + 1, 3) = 0;
				A(2 * i + 1, 4) = 0; // 0\tr
				A(2 * i + 1, 5) = 0;
				A(2 * i + 1, 6) = -x(0) * q(0);
				A(2 * i + 1, 7) = -x(0) * q(1); // -x'x\tr
				A(2 * i + 1, 8) = -x(0) * 1;
			}
			// assemble two equations per vertex correspondence (given by cross(qH, x) = vec3(0, 0, 0))

			Eigen::JacobiSVD<_TyDLTMatrix> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
			// decompose using SVD

			_ASSERTE(svd.rank() <= 9 && 9 <= INT_MAX);
			int n_last = std::min(8, int(svd.rank())); // not all versions of Eigen have this (but it was added at some point in 3.2)
			const Eigen::JacobiSVD<_TyDLTMatrix>::MatrixVType &V = svd.matrixV();

			for(int i = 0; i < 9; ++ i)
				H_DLT(i / 3, i % 3) = V(i, n_last);
			// get hessian as the last singular vector in V
		} else {
			Eigen::Matrix<double, 9, 9> ATA;
			ATA.setZero();
			for(size_t i = 0; i < n_point_num; ++ i) {
				_TyPoint2D q = r_points_template[i], x = r_points_image[i];
				Eigen::Matrix<double, 2, 9> Ai;
				Ai << 0, 0, 0, q(0), q(1), 1, -x(1) * q(0), -x(1) * q(1), -x(1), // 0\tr, w'x\tr, -y'x\tr (flipped row sign)
					q(0), q(1), 1, 0, 0, 0, -x(0) * q(0), -x(0) * q(1), -x(0); // w'x\tr, 0\tr, -x'x\tr
				ATA.noalias() += Ai.transpose() * Ai;
			}
			// assemble two equations directly in the squared form

			Eigen::JacobiSVD<Eigen::Matrix<double, 9, 9> > svd(ATA, Eigen::ComputeFullU | Eigen::ComputeFullV);
			// could use self-adjoing eigendecomposition here instead of SVD

			_ASSERTE(svd.rank() <= 9 && 9 <= INT_MAX);
			int n_last = std::min(8, int(svd.rank())); // not all versions of Eigen have this (but it was added at some point in 3.2)
			const Eigen::JacobiSVD<Eigen::Matrix<double, 9, 9> >::MatrixVType &V = svd.matrixV();

			for(int i = 0; i < 9; ++ i)
				H_DLT(i / 3, i % 3) = V(i, n_last);
			// get hessian as the last singular vector in V
		}

		return t_Normalize(H_DLT);
	}

	/**
	 *	@brief improves homography using NLS (no outlier rejection attempted)
	 *
	 *	@param[in,out] r_t_homography is the estimated homography \f$H\f$ such that \f$I_i = HT_i\f$
	 *		where \f$I\f$ is the vector of image points and \f$T\f$ is the vector of template points
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] n_max_LM_iteration_num is the maximum number of NLS iterations
	 *	@param[in] b_verbose is verbose flag (if set, staistics of NLS iterations will be printed
	 *		to stdout; disabled by default)
	 *
	 *	@note This minimizes the symmetric transfer error.
	 */
	static inline void ImproveHomography(Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		int n_max_LM_iteration_num = 100, bool b_verbose = false)
	{
		/*r_t_homography += Eigen::Matrix3d::Random() * .1;
		b_verbose = true;
		// debug - see if the minimizer works*/

		ImproveHomography<CDefaultErrorObjective>(r_t_homography,
			r_points_image, r_points_template, n_max_LM_iteration_num, b_verbose);

		/*ImproveHomography<CTransferErrorObjective>(r_t_homography,
			r_points_image, r_points_template, n_max_LM_iteration_num, b_verbose);
		ImproveHomography<CSymTransferErrorObjective>(r_t_homography,
			r_points_image, r_points_template, n_max_LM_iteration_num, b_verbose);*/
		/*ImproveHomography<CSampsonErrorObjective>(r_t_homography,
			r_points_image, r_points_template, n_max_LM_iteration_num, b_verbose);
		ImproveHomography_Reprojection(r_t_homography,
			r_points_image, r_points_template, 0, n_max_LM_iteration_num, b_verbose);*/
	}

	/**
	 *	@brief improves homography using NLS (no outlier rejection attempted)
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in,out] r_t_homography is the estimated homography \f$H\f$ such that \f$I_i = HT_i\f$
	 *		where \f$I\f$ is the vector of image points and \f$T\f$ is the vector of template points
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] n_max_LM_iteration_num is the maximum number of NLS iterations
	 *	@param[in] b_verbose is verbose flag (if set, staistics of NLS iterations will be printed
	 *		to stdout; disabled by default)
	 *	@param[in] n_max_failed_step_repeat_num is maximum number of bad LM steps that do not count
	 *		towards the maximum LM iteration number
	 *	@param[in] f_dx_thresh is the stopping condition (threshold on update vector; default 1e-20)
	 *
	 *	@note To use reprojection error as the objective function, use
	 *		\ref ImproveHomography_Reprojection(). It requires ä different optimizer state
	 *		than maintained here.
	 */
	template <class CObjective>
	static typename CEnableNotFor<CObjective, CReprojectionErrorObjective>::T
		ImproveHomography(Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		int n_max_LM_iteration_num = 100, bool b_verbose = false,
		int n_max_failed_step_repeat_num = 0, double f_dx_thresh = 1e-20)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		Eigen::Matrix3d H = r_t_homography; // initial guess
		double f_n = 2, f_alpha; // damping
		double f_prev_Chi2 = f_Chi2Error<CObjective>(H, r_points_image, r_points_template);
		// calculate the initial RMSE

		if(b_verbose) {
			double f_initial_RMSE = sqrt(f_prev_Chi2 / n_point_num);
			printf("initial residual is %g\n", f_initial_RMSE);
		}
		// print norm of updates to see the convergence

		for(int n_pass = 0; n_pass < n_max_LM_iteration_num; ++ n_pass) {
			Eigen::Matrix<double, 9, 9> t_Hessian = Eigen::Matrix<double, 9, 9>::Zero();
			Eigen::Matrix<double, 9, 1> v_rhs = Eigen::Matrix<double, 9, 1>::Zero();
			// prepare the Hessian and the right-hand side

			CObjective objective(H, true); // will do Jacobians
			// let the objective precompute constants dependent only on H

			for(size_t i = 0; i < n_point_num; ++ i) {
				_TyPoint2D v_image = r_points_image[i], v_template = r_points_template[i];
				typename CObjective::_TyJacobian J;
				typename CObjective::_TyResidual v_error = objective.v_Error_Jacobian(J, v_image, v_template);
				t_Hessian += J * J.transpose();
				v_rhs += J * v_error;
			}

			if(!n_pass) {
				double f_abs_max = t_Hessian.diagonal().array().abs().maxCoeff();
				f_alpha = f_abs_max * 1e-3;
			}
			// heuristic for choosing damping

			t_Hessian += Eigen::Matrix<double, 9, 9>::Identity() * f_alpha;
			// apply the damping

			Eigen::PartialPivLU<Eigen::Matrix<double, 9, 9> > LU(t_Hessian);
			Eigen::Matrix<double, 9, 1> v_dx = LU.solve(v_rhs);
			// decompose

			Eigen::Matrix3d Hnew = H;
			for(int i = 0; i < 9; ++ i)
				Hnew(i / 3, i % 3) += v_dx(i);
			// update the homography

			double f_Chi2 = f_Chi2Error<CObjective>(Hnew, r_points_image, r_points_template);

			if(b_verbose) {
				double f_RMSE = sqrt(f_Chi2 / n_point_num);
				printf("dx norm is %g, residual is %g, damping is %g\n", v_dx.norm(), f_RMSE, f_alpha);
			}
			// print norm of updates to see the convergence

			double f_gain = (f_prev_Chi2 - f_Chi2) / v_dx.dot(f_alpha * v_dx + v_rhs);
			if(f_gain > 0) { // comparison to NaN does not enter
				_ASSERTE(v_dx.norm() >= 0); // if NaN, the comparison would fail
				f_alpha *= std::max(1 / 3.0, 1.0 - pow((2 * f_gain - 1), 3));
				f_n = 2;
				H = Hnew; // step taken
				f_prev_Chi2 = f_Chi2;
			} else {
				f_alpha *= f_n;
				f_n *= 2; // step not taken
				if(n_max_failed_step_repeat_num) {
					-- n_max_failed_step_repeat_num;
					-- n_pass;
					// try again
				}
			}
			// aftermath

			if(!(v_dx.norm() > f_dx_thresh)) // use a not, so that comparison to NaN would enter
				break; // avoid division by zero in the next step
			// put it here, take advantage of the last solve if it decreased the objective
		}
		// use LM

		r_t_homography = t_Normalize(H);
		// normalize, copy back
	}

	/**
	 *	@brief improves homography using NLS, minimizes reprojection error (no outlier rejection attempted)
	 *
	 *	@param[in,out] r_t_homography is the estimated homography \f$H\f$ such that \f$I_i = HT_i\f$
	 *		where \f$I\f$ is the vector of image points and \f$T\f$ is the vector of template points
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[out] p_points_template_opt is an optional vector of the optimized positions of the
	 *		2D points in the source space (dubbed template; can be null)
	 *	@param[in] n_max_LM_iteration_num is the maximum number of NLS iterations
	 *	@param[in] b_verbose is verbose flag (if set, staistics of NLS iterations will be printed
	 *		to stdout; disabled by default)
	 *	@param[in] n_max_failed_step_repeat_num is maximum number of bad LM steps that do not count
	 *		towards the maximum LM iteration number
	 *	@param[in] f_dx_thresh is the stopping condition (threshold on update vector; default 1e-20)
	 *	@param[in] b_use_Schur is Schur complement flag (use Schur complement for linear solving,
	 *		as the system matrix is bipartite (clique) graph)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This minimizes reprojection error.
	 *	@note Note that p_points_template_opt is owerwritten (out only, not in).
	 *	@note Using Schur complement for solving yields linear memory complexity in the number
	 *		of points (otherwise the complexity is quadratic as the sparsity is not taken advantage
	 *		of in any way).
	 */
	static void ImproveHomography_Reprojection(Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		std::vector<_TyPoint2D_u> *p_points_template_opt = 0,
		int n_max_LM_iteration_num = 100, bool b_verbose = false,
		int n_max_failed_step_repeat_num = 0, double f_dx_thresh = 1e-20,
		bool b_use_Schur = true) // throw(std::bad_alloc)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		const size_t n_state_size = 9 + 2 * n_point_num; // 9 for homography and 2 for each optimized template point (image points are not optimized, those can be recovered from the optimized template and the optimized homography)
		Eigen::VectorXd v_rhs(n_state_size), v_dx(n_state_size), v_points_opt_template(2 * n_point_num),
			v_new_points_opt_template(2 * n_point_num);
		Eigen::MatrixXd t_Hessian;
		if(!b_use_Schur)
			t_Hessian.resize(n_state_size, n_state_size); // t_odo - see if it is any sparse (it is; this could be solved using SC, as it is an arrow matrix with 9x9 A, 9x2p U and 2px2p D consisting of 2x2 block diagonal)
		Eigen::Matrix<double, 9, Eigen::Dynamic> t_Hessian_top_rows;
		Eigen::Matrix<double, 2, Eigen::Dynamic> t_Hessian_block_diag;
		if(b_use_Schur) {
			t_Hessian_top_rows.resize(9, n_state_size); // stores A and U
			t_Hessian_block_diag.resize(2, 2 * n_point_num); // stores compact D
		}
		// allocate matrices (the solver reuses memory between iterations)

		Eigen::Matrix3d H = r_t_homography; // initial guess
		double f_n = 2, f_alpha; // damping
		double f_prev_Chi2 = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			_TyPoint2D v_template = r_points_template[i];
			v_points_opt_template.segment<2>(2 * i) = v_template;
			// store this

			Eigen::Vector3d v_repr = H * Eigen::Vector3d(v_template(0), v_template(1), 1);
			Eigen::Vector2d v_error = r_points_image[i] - v_repr.head<2>() / v_repr(2);
			// initially, the error in the template is zero and the error in the image
			// is the unsymetric transfer error (because template is its own prior)

			f_prev_Chi2 += v_error.squaredNorm();
		}
		// calculate the initial RMSE, copy the template points aside

		if(b_verbose) {
			double f_initial_RMSE = sqrt(f_prev_Chi2 / n_point_num);
			printf("initial residual is %g\n", f_initial_RMSE);
		}
		// print norm of updates to see the convergence

		for(int n_pass = 0; n_pass < n_max_LM_iteration_num; ++ n_pass) {
			if(b_use_Schur) {
				t_Hessian_top_rows.leftCols<9>().setZero(); // only this part is overwritten multiple times
				//t_Hessian_block_diag.setZero();
			} else
				t_Hessian.setZero();
			v_rhs.setZero();
			// prepare the Hessian and the right-hand side

			for(size_t i = 0; i < n_point_num; ++ i) {
				_TyPoint2D v_image = r_points_image[i], v_template = r_points_template[i],
					v_opt_template = v_points_opt_template.segment<2>(2 * i);

				CReprojectionErrorObjective::_TyJacobian_H J_H;
				CReprojectionErrorObjective::_TyJacobian_T J_T;
#if 1 // the newer code using the reusable functions
				Eigen::Vector4d v_error = CReprojectionErrorObjective::v_Error_Jacobians(J_H, J_T,
					v_image, v_template, v_opt_template, H);

				if(b_use_Schur) {
					t_Hessian_top_rows.leftCols<9>() += J_H/*.leftCols<2>()*/ * J_H/*.leftCols<2>()*/.transpose();
					t_Hessian_top_rows.middleCols<2>(9 + 2 * i) /*+*/= J_H/*.leftCols<2>()*/ *
						J_T.leftCols<2>().transpose(); // !!
					t_Hessian_block_diag.middleCols<2>(2 * i) /*+*/= J_T * J_T.transpose(); // only written once
					// accumulate to separate SC matrices to save memory (and cache)
				} else {
					t_Hessian.topLeftCorner<9, 9>() += J_H/*.leftCols<2>()*/ * J_H/*.leftCols<2>()*/.transpose();
					t_Hessian.block<2, 2>(9 + 2 * i, 9 + 2 * i) += J_T * J_T.transpose();
					t_Hessian.middleCols<2>(9 + 2 * i).topRows<9>() += J_H/*.leftCols<2>()*/ *
						J_T.leftCols<2>().transpose(); // !!
				}

				v_rhs.head<9>() += J_H/*.leftCols<2>()*/ * v_error.head<2>();
				v_rhs.segment<2>(9 + 2 * i) += J_T * v_error;
#else // 1
				Eigen::Vector4d v_error;
				{ // the older code where the error is swapped as (template, image) rather than (image, template)
					Eigen::Vector3d v_repr = H * Eigen::Vector3d(v_opt_template(0), v_opt_template(1), 1);
					v_error.head<2>() = v_template - v_opt_template;
					v_error.tail<2>() = v_image - v_repr.head<2>() / v_repr(2);

					const double /*Imx = v_image(0), Imy = v_image(1), Tpx = v_template(0),
						Tpy = v_template(1),*/ TpOx = v_opt_template(0), TpOy = v_opt_template(1),
						h11 = H(0, 0), h12 = H(0, 1), h13 = H(0, 2), h21 = H(1, 0), h22 = H(1, 1),
						h23 = H(1, 2), h31 = H(2, 0), h32 = H(2, 1), h33 = H(2, 2);

					//J_H.leftCols<2>().setZero(); // those are zero

					double f_temp0 = -v_repr(0); _ASSERTE(fabs(-f_temp0 - (h11*TpOx+h12*TpOy+h13)) < 1e-10);
					double f_temp1 = -v_repr(1); _ASSERTE(fabs(-f_temp1 - (h21*TpOx+h22*TpOy+h23)) < 1e-10);
					double f_temp2 = v_repr(2); _ASSERTE(fabs(f_temp2 - (h31*TpOx+h32*TpOy+h33)) < 1e-10);
					double f_temp3 = f_temp2 * f_temp2; _ASSERTE(fabs(f_temp3 - pow(h31*TpOx+h32*TpOy+h33,2.0)) < 1e-10);
					J_H(0, 0/*2*/) = TpOx / f_temp2;
					J_H(1, 0/*2*/) = TpOy / f_temp2;
					J_H(2, 0/*2*/) = 1 / f_temp2;
					J_H(3, 0/*2*/) = 0.0;
					J_H(4, 0/*2*/) = 0.0;
					J_H(5, 0/*2*/) = 0.0;
					J_H(6, 0/*2*/) = f_temp0 / f_temp3 * TpOx;
					J_H(7, 0/*2*/) = f_temp0 / f_temp3 * TpOy;
					J_H(8, 0/*2*/) = f_temp0 / f_temp3;

					J_H(0, 1/*3*/) = 0.0;
					J_H(1, 1/*3*/) = 0.0;
					J_H(2, 1/*3*/) = 0.0;
					J_H(3, 1/*3*/) = TpOx / f_temp2;
					J_H(4, 1/*3*/) = TpOy / f_temp2;
					J_H(5, 1/*3*/) = 1 / f_temp2;
					J_H(6, 1/*3*/) = f_temp1 / f_temp3 * TpOx;
					J_H(7, 1/*3*/) = f_temp1 / f_temp3 * TpOy;
					J_H(8, 1/*3*/) = f_temp1 / f_temp3;

					J_T.leftCols<2>().setIdentity();
					J_T(0, 2) = h11 / f_temp2 + f_temp0 / f_temp3 * h31;
					J_T(1, 2) = h12 / f_temp2 + f_temp0 / f_temp3 * h32;
					J_T(0, 3) = h21 / f_temp2 + f_temp1 / f_temp3 * h31;
					J_T(1, 3) = h22 / f_temp2 + f_temp1 / f_temp3 * h32;
				}

				_ASSERTE(!b_use_Schur); // not implemented
				t_Hessian.topLeftCorner<9, 9>() += J_H/*.rightCols<2>()*/ * J_H/*.rightCols<2>()*/.transpose();
				t_Hessian.block<2, 2>(9 + 2 * i, 9 + 2 * i) += J_T * J_T.transpose();
				t_Hessian.middleCols<2>(9 + 2 * i).topRows<9>() += J_H/*.rightCols<2>()*/ *
					J_T.rightCols<2>().transpose(); // !!

				v_rhs.head<9>() += J_H/*.rightCols<2>()*/ * v_error.tail<2>();
				v_rhs.segment<2>(9 + 2 * i) += J_T * v_error;
#endif // 1
			}

#if defined(_DEBUG) && defined(TEST_HOMOGRAPHY_SC_SOLVER)
			if(b_use_Schur) {
				t_Hessian.setZero(n_state_size, n_state_size);
				t_Hessian.topRows<9>() = t_Hessian_top_rows;
				for(size_t i = 0; i < n_point_num; ++ i) // could run in parallel
					t_Hessian.block<2, 2>(9 + 2 * i, 9 + 2 * i) = t_Hessian_block_diag.middleCols<2>(2 * i);
				// duplicate to the full matrix before doing inplace changes (e.g. to D)
			}
#endif // _DEBUG && TEST_HOMOGRAPHY_SC_SOLVER

			if(!n_pass) {
				double f_abs_max;
				if(b_use_Schur) {
					f_abs_max = t_Hessian_top_rows.leftCols<9>().diagonal().array().abs().maxCoeff(); // max of the 9x9 block
					for(size_t i = 0; i < n_point_num; ++ i) {
						f_abs_max = std::max(f_abs_max, t_Hessian_block_diag.middleCols<2>(2 *
							i).diagonal().array().abs().maxCoeff()); // max of each block on the diagonal
					}
#ifdef TEST_HOMOGRAPHY_SC_SOLVER
					_ASSERTE(f_abs_max == t_Hessian.diagonal().array().abs().maxCoeff()); // full matrix duplicated in debug; needs to be exactly the same (will get in trouble with NaNs)
#endif // TEST_HOMOGRAPHY_SC_SOLVER
				} else
					f_abs_max = t_Hessian.diagonal().array().abs().maxCoeff(); // max of the diagonal in the full matrix					
				f_alpha = f_abs_max * 1e-3;
			}
			// heuristic for choosing damping

			if(b_use_Schur) { // use SC for solving
#if defined(_DEBUG) && defined(TEST_HOMOGRAPHY_SC_SOLVER)
				t_Hessian.diagonal().array() += f_alpha;
#endif // _DEBUG && TEST_HOMOGRAPHY_SC_SOLVER
				t_Hessian_top_rows.leftCols<9>().diagonal().array() += f_alpha;
				// apply the damping

				Eigen::Matrix<double, 9, 9> A = t_Hessian_top_rows.leftCols<9>();//t_Hessian.topLeftCorner<9, 9>();
				Eigen::Matrix<double, 9, Eigen::Dynamic> U = t_Hessian_top_rows.rightCols(2 * n_point_num);//t_Hessian.topRows<9>().rightCols(2 * n_point_num);
				//Eigen::MatrixXd D = t_Hessian.bottomRightCorner(2 * n_point_num, 2 * n_point_num);
				// todo - replace those by block expressions or at least maps

				/*Eigen::Matrix<double, 9, 9> dA = A - t_Hessian.topLeftCorner<9, 9>();
				_ASSERTE(dA.norm() < 1e-10);
				Eigen::Matrix<double, 9, Eigen::Dynamic> dU = U - t_Hessian.topRows<9>().rightCols(2 * n_point_num);
				_ASSERTE(dU.norm() < 1e-10);*/ // make sure we got that right

				Eigen::Matrix<double, 9, Eigen::Dynamic> UDinv(9, 2 * n_point_num);

				Eigen::Matrix<double, 2, Eigen::Dynamic> &Dinv = t_Hessian_block_diag; // rename, invert D inplace
				//Eigen::MatrixXd &Dinv = D; // rename; D never needed below
				for(size_t i = 0; i < n_point_num; ++ i) { // could run in parallel
					//Dinv.block<2, 2>(2 * i, 2 * i) = Dinv.block<2, 2>(2 * i, 2 * i).inverse();
					Eigen::Matrix2d Dii = Dinv.middleCols<2>(2 * i);
					Dii.diagonal().array() += f_alpha; // apply damping only now
					Dinv.middleCols<2>(2 * i) = Dii.inverse();
					UDinv.middleCols<2>(2 * i) = U.middleCols<2>(2 * i) * Dinv.middleCols<2>(2 * i);//Dinv.block<2, 2>(2 * i, 2 * i);
				}
				// block diagonal inverse

				/*double f_UDinv_err = (UDinv - t_Hessian.topRows<9>().rightCols(2 * n_point_num) *
					t_Hessian.bottomRightCorner(2 * n_point_num, 2 * n_point_num).inverse()).norm();
				_ASSERTE(f_UDinv_err <= 1e-10);*/ // make sure we got that right

				Eigen::Matrix<double, 9, 9> SC = A - /*U * Dinv*/UDinv * U.transpose(); // note that U * Dinv is still slow due to Dinv (and D) being full; we need a tridiagonal matrix; alternatively, could slice U into blocks and multiply it by blocks of D
				// calculate SC

				Eigen::PartialPivLU<Eigen::Matrix<double, 9, 9> > LU(SC);
				v_dx.head<9>() = LU.solve(v_rhs.head<9>() - /*U * Dinv*/UDinv * v_rhs.tail(2 * n_point_num));
				v_dx.tail(2 * n_point_num) = /*Dinv **/ (v_rhs.tail(2 * n_point_num) - U.transpose() * v_dx.head<9>());
				for(size_t i = 0; i < n_point_num; ++ i) // could run in parallel
					v_dx.segment<2>(9 + 2 * i).transpose() *= Dinv.middleCols<2>(2 * i).transpose();// * v_dx.segment<2>(9 + 2 * i);
				// solve using SC (first solves for homography and then for the points)

#if defined(_DEBUG) && defined(TEST_HOMOGRAPHY_SC_SOLVER)
				t_Hessian.triangularView<Eigen::StrictlyLower>() =
					t_Hessian.triangularView<Eigen::StrictlyUpper>().transpose();
				// copy the upper triangle to the lower one, we saved some computation above

				Eigen::PartialPivLU<Eigen::MatrixXd> LU_full(t_Hessian);
				Eigen::VectorXd v_dx_ref = LU_full.solve(v_rhs);
				/*double f_abs_dxh_error = (v_dx - v_dx_ref).head<9>().norm();
				double f_rel_dxh_error = f_abs_dxh_error / v_dx_ref.head<9>().norm();
				double f_abs_dxt_error = (v_dx - v_dx_ref).tail(2 * n_point_num).norm();
				double f_rel_dxt_error = f_abs_dxt_error / v_dx_ref.tail(2 * n_point_num).norm();*/ // head / tail breakdown for debugging
				double f_abs_dx_error = (v_dx - v_dx_ref).norm();
				double f_rel_dx_error = f_abs_dx_error / v_dx_ref.norm();
				_ASSERTE(!(f_abs_dx_error >= 1e-10)); // this is a lame comparison, norm of dx is usually pretty small
				_ASSERTE(!(f_rel_dx_error >= 1e-6)); // this is better (more firm)
				// decompose
#endif // _DEBUG && TEST_HOMOGRAPHY_SC_SOLVER
			} else {
				t_Hessian.diagonal().array() += f_alpha;
				// apply the damping

				t_Hessian.triangularView<Eigen::StrictlyLower>() =
					t_Hessian.triangularView<Eigen::StrictlyUpper>().transpose();
				// copy the upper triangle to the lower one, we saved some computation above

				Eigen::PartialPivLU<Eigen::MatrixXd> LU(t_Hessian);
				v_dx = LU.solve(v_rhs);
				// decompose
			}

			Eigen::Matrix3d Hnew = H;
			for(int i = 0; i < 9; ++ i)
				Hnew(i / 3, i % 3) += v_dx(i);
			v_new_points_opt_template = v_points_opt_template + v_dx.tail(2 * n_point_num);
			// update the homography and the points

			double f_Chi2 = 0;
			for(size_t i = 0; i < n_point_num; ++ i) {
				/*Eigen::Vector2d v_opt_template = v_new_points_opt_template.segment<2>(2 * i);
				Eigen::Vector3d v_repr = Hnew * Eigen::Vector3d(v_opt_template(0), v_opt_template(1), 1);
				Eigen::Vector4d v_error;
				v_error.head<2>() = r_points_template[i] - v_opt_template;
				v_error.tail<2>() = r_points_image[i] - v_repr.head<2>() / v_repr(2);*/
				f_Chi2 += CReprojectionErrorObjective::v_Error(r_points_image[i], r_points_template[i],
					v_new_points_opt_template.segment<2>(2 * i), Hnew).squaredNorm();
			}

			if(b_verbose) {
				double f_RMSE = sqrt(f_Chi2 / n_point_num);
				printf("dx norm is %g, residual is %g, damping is %g\n", v_dx.norm(), f_RMSE, f_alpha);
			}
			// print norm of updates to see the convergence

			double f_gain = (f_prev_Chi2 - f_Chi2) / v_dx.dot(f_alpha * v_dx + v_rhs);
			if(f_gain > 0) { // comparison to NaN does not enter
				_ASSERTE(v_dx.norm() >= 0); // if NaN, the comparison would fail
				f_alpha *= std::max(1 / 3.0, 1.0 - pow((2 * f_gain - 1), 3));
				f_n = 2;
				H = Hnew; // step taken
				v_new_points_opt_template.swap(v_points_opt_template); // step taken
				f_prev_Chi2 = f_Chi2;
			} else {
				f_alpha *= f_n;
				f_n *= 2; // step not taken
				if(n_max_failed_step_repeat_num) {
					-- n_max_failed_step_repeat_num;
					-- n_pass;
					// try again
				}
			}
			// aftermath

			if(!(v_dx.norm() > f_dx_thresh)) // use a not, so that comparison to NaN would enter
				break; // avoid division by zero in the next step
			// put it here, take advantage of the last solve if it decreased the objective
		}
		// use LM

		if(p_points_template_opt) {
			std::vector<_TyPoint2D_u> &r_points_image_opt = *p_points_template_opt;
			r_points_image_opt.resize(n_point_num); // !!
			for(size_t i = 0; i < n_point_num; ++ i)
				r_points_image_opt[i] = v_points_opt_template.segment<2>(2 * i);
		}
		// copy the optimized template points to the output

		r_t_homography = t_Normalize(H);
		// normalize, copy back
	}

	/**
	 *	@brief normalizes a homography to be in the cannonical form
	 *	@param[in] r_t_H is the homography to be normalized
	 *	@return Returns the given homography rescaled to unit L2-norm.
	 */
	static inline Eigen::Matrix3d t_Normalize(const Eigen::Matrix3d &r_t_H)
	{
		/*double f_norm = r_t_H.norm();
		if(fabs(r_t_H(2, 2) / f_norm) > 1e-3)
			return r_t_H / r_t_H(2, 2);
		// dominant H_22, use that

		Eigen::Matrix3d::Index maxRow, maxCol;
		r_t_H.array().abs().maxCoeff(&maxRow, &maxCol);
		double f_sign = (r_t_H(maxRow, maxCol) < 0)? -1 : 1;
		// H_22 is (near) zero, divide by norm and make sure the greatest element in the matrix is positive

		return r_t_H / (f_norm * f_sign);*/
		// this always leads to issues; two similar homographies may differ either in threshold
		// on H_22 or in the sign of the max coeff (imagine there is both 1.0 + eps and -1.0 + eps
		// present in the matrix, then the max coeff sign depends only on the sign of eps, which
		// can be arbitrarily small (although not less than 1 ULP))

		return r_t_H / r_t_H.norm();
		// this works always
	}

	/**
	 *	@brief compare two homographies and returns the normalized error
	 *
	 *	@param[in] r_t_A is the first homography to be compared
	 *	@param[in] r_t_B is the second homography to be compared
	 *
	 *	@return Returns the L2-norm of difference of the two homographies that
	 *		were first rescaled to unit L2-norm.
	 */
	static inline double f_Compare(const Eigen::Matrix3d &r_t_A, const Eigen::Matrix3d &r_t_B)
	{
		Eigen::Matrix3d t_nrm_a = t_Normalize(r_t_A), t_nrm_b = t_Normalize(r_t_B); // normalize
		return sqrt(std::min((t_nrm_a - t_nrm_b).squaredNorm(), (t_nrm_a + t_nrm_b).squaredNorm()));
		// even if normalized, the homographies may be different up to their sign
	}

	/**
	 *	@brief evaluate homography error based on a given objective function
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *
	 *	@return Returns the sum of errors (not mean, not squared).
	 */
	template <class CObjective>
	static double f_Error(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		CObjective objective(r_t_homography, false); // no Jacobians needed here
		// let the objective precompute constants dependent only on H

		double f_error = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			typename CObjective::_TyResidual v_error = objective.v_Error(r_points_image[i], r_points_template[i]);
			f_error += v_error.norm();
		}
		// calculate the error

		return f_error;
	}

	/**
	 *	@brief evaluate homography \f$\chi^2\f$ error based on a given objective function
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] r_t_cov is a covariance matrix for the points
	 *
	 *	@return Returns the sum of Mahalanobis norms of the vectorial residuals,
	 *		weighted by the given covariance.
	 */
	template <class CObjective>
	static double f_Chi2Error(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		const typename CObjective::_TyCovariance &r_t_cov /*= CObjective::_TyCovariance::Identity()*/)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		CObjective objective(r_t_homography, false); // no Jacobians needed here
		// let the objective precompute constants dependent only on H

		double f_error = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			typename CObjective::_TyResidual v_error = objective.v_Error(r_points_image[i], r_points_template[i]);
			f_error += v_error.dot(r_t_cov * v_error);
		}
		// calculate the error

		return f_error;
	}

	/**
	 *	@brief evaluate homography \f$\chi^2\f$ error based on a given objective
	 *		function (specialization for unit covariance)
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *
	 *	@return Returns the sum of Mahalanobis norms of the vectorial residuals.
	 */
	template <class CObjective>
	static double f_Chi2Error(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template/*,
		const typename CObjective::_TyCovariance &r_t_cov = CObjective::_TyCovariance::Identity()*/)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		CObjective objective(r_t_homography, false); // no Jacobians needed here
		// let the objective precompute constants dependent only on H

		double f_error = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			typename CObjective::_TyResidual v_error = objective.v_Error(r_points_image[i], r_points_template[i]);
			f_error += v_error.squaredNorm(); // specialization for identity covariance
		}
		// calculate the error

		return f_error;
	}

	/**
	 *	@brief evaluate homography root-mean-square (RMS) error based on a given objective function
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *
	 *	@return Returns the RMS error calculated using the given objective.
	 */
	template <class CObjective>
	static double f_RMSError(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template)
	{
		return sqrt(f_Chi2Error<CObjective>(r_t_homography,
			r_points_image, r_points_template) / r_points_template.size());
	}

	/**
	 *	@brief evaluate homography error based on a given objective function (version
	 *		for reprojection error)
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] r_points_template_opt is a vector of the optimized positions of the 2D points
	 *		in the source space (dubbed template)
	 *
	 *	@return Returns the sum of errors (not mean, not squared).
	 */
	template <class CObjective>
	static double f_Error(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		const std::vector<_TyPoint2D_u> &r_points_template_opt)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		_ASSERTE(r_points_template.size() == r_points_template_opt.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		CObjective objective(r_t_homography, false); // no Jacobians needed here
		// let the objective precompute constants dependent only on H

		double f_error = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			typename CObjective::_TyResidual v_error = objective.v_Error(r_points_image[i],
				r_points_template[i], r_points_template_opt[i]);
			f_error += v_error.norm();
		}
		// calculate the error

		return f_error;
	}

	/**
	 *	@brief evaluate homography \f$\chi^2\f$ error based on a given objective function (version
	 *		for reprojection error)
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] r_points_template_opt is a vector of the optimized positions of the 2D points
	 *		in the source space (dubbed template)
	 *	@param[in] r_t_cov is a covariance matrix for the points
	 *
	 *	@return Returns the sum of Mahalanobis norms of the vectorial residuals,
	 *		weighted by the given covariance.
	 */
	template <class CObjective>
	static double f_Chi2Error(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		const std::vector<_TyPoint2D_u> &r_points_template_opt,
		const typename CObjective::_TyCovariance &r_t_cov /*= CObjective::_TyCovariance::Identity()*/)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		const size_t n_point_num = r_points_template.size();
		_ASSERTE(r_points_template.size() == r_points_template_opt.size());
		// both lists must be of same size

		CObjective objective(r_t_homography, false); // no Jacobians needed here
		// let the objective precompute constants dependent only on H

		double f_error = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			typename CObjective::_TyResidual v_error = objective.v_Error(r_points_image[i],
				r_points_template[i], r_points_template_opt[i]);
			f_error += v_error.dot(r_t_cov * v_error);
		}
		// calculate the error

		return f_error;
	}

	/**
	 *	@brief evaluate homography \f$\chi^2\f$ error based on a given objective
	 *		function (specialization for unit covariance; version for reprojection error)
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] r_points_template_opt is a vector of the optimized positions of the 2D points
	 *		in the source space (dubbed template)
	 *
	 *	@return Returns the sum of Mahalanobis norms of the vectorial residuals.
	 */
	template <class CObjective>
	static double f_Chi2Error(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		const std::vector<_TyPoint2D_u> &r_points_template_opt/*,
		const typename CObjective::_TyCovariance &r_t_cov = CObjective::_TyCovariance::Identity()*/)
	{
		_ASSERTE(r_points_image.size() == r_points_template.size());
		_ASSERTE(r_points_template.size() == r_points_template_opt.size());
		const size_t n_point_num = r_points_template.size();
		// both lists must be of same size

		CObjective objective(r_t_homography, false); // no Jacobians needed here
		// let the objective precompute constants dependent only on H

		double f_error = 0;
		for(size_t i = 0; i < n_point_num; ++ i) {
			typename CObjective::_TyResidual v_error = objective.v_Error(r_points_image[i],
				r_points_template[i], r_points_template_opt[i]);
			f_error += v_error.squaredNorm(); // specialization for identity covariance
		}
		// calculate the error

		return f_error;
	}

	/**
	 *	@brief evaluate homography root-mean-square (RMS) error based on a given
	 *		objective function (version for reprojection error)
	 *
	 *	@tparam CObjective is the objective function to be minimized, can be one
	 *		of \ref CTransferErrorObjective, \ref CSymTransferErrorObjective or
	 *		\ref CSampsonErrorObjective
	 *
	 *	@param[in] r_t_homography is the homography to be evaluated (a mapping from template to image)
	 *	@param[in] r_points_image is a vector of 2D points in the destination space (dubbed image)
	 *	@param[in] r_points_template is a vector of 2D points in the source space (dubbed template)
	 *	@param[in] r_points_template_opt is a vector of the optimized positions of the 2D points
	 *		in the source space (dubbed template)
	 *
	 *	@return Returns the RMS error calculated using the given objective.
	 */
	template <class CObjective>
	static double f_RMSError(const Eigen::Matrix3d &r_t_homography,
		const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template,
		const std::vector<_TyPoint2D_u> &r_points_template_opt)
	{
		return sqrt(f_Chi2Error<CObjective>(r_t_homography,
			r_points_image, r_points_template, r_points_template_opt) / r_points_template.size());
	}

	/**
	 *	@brief asymmetric transfer error (in the destination space, "image")
	 *	@note To get asymmetric transfer error in template, swap image and template
	 *		and invert the homography. It is actually cheaper than calculating inverse
	 *		homography in each NLS iteration.
	 */
	class CTransferErrorObjective {
	public:
		enum {
			n_residual_dimension = 2
		};

		typedef Eigen::Matrix<double, n_residual_dimension, 1> _TyResidual;
		typedef Eigen::Matrix<double, n_residual_dimension, n_residual_dimension> _TyCovariance; /**< @brief covariance in image */
		typedef Eigen::Matrix<double, 9, n_residual_dimension> _TyJacobian;

	protected:
		const Eigen::Matrix3d &m_r_t_H;

	public:
		inline CTransferErrorObjective(const Eigen::Matrix3d &r_t_H, bool UNUSED(b_will_do_jacobians))
			:m_r_t_H(r_t_H)
		{}

		template <class CDerived0, class CDerived1>
		inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template) const
		{
			return v_Error(r_v_image, r_v_template, m_r_t_H);
		}

		template <class CDerived0, class CDerived1, class CDerived2>
		inline _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template) const
		{
			return v_Error_Jacobian(r_t_J, r_v_image, r_v_template, m_r_t_H);
		}

		template <class CDerived0, class CDerived1, class CDerived2>
		static inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template,
			const Eigen::MatrixBase<CDerived2> &r_t_H);

		template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
		static inline _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template,
			const Eigen::MatrixBase<CDerived3> &r_t_H);
	};

	/**
	 *	@brief symmetric transfer error
	 */
	class CSymTransferErrorObjective {
	public:
		enum {
			n_residual_dimension = 4
		};

		typedef Eigen::Matrix<double, n_residual_dimension, 1> _TyResidual;
		typedef Eigen::Matrix<double, n_residual_dimension, n_residual_dimension> _TyCovariance; /**< @brief covariance in image (top left 2x2) then template (bottom right 2x2) */
		typedef Eigen::Matrix<double, 9, n_residual_dimension> _TyJacobian;

	protected:
		const Eigen::Matrix3d &m_r_t_H;
		const Eigen::Matrix3d m_t_H_inv;
#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
		/*const*/ Eigen::Matrix<double, 9, 9> m_t_JInv; // initialized conditionally, cannot be const
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC

	public:
		inline CSymTransferErrorObjective(const Eigen::Matrix3d &r_t_H, bool b_will_do_jacobians);

		template <class CDerived0, class CDerived1>
		inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template) const
		{
			return v_Error(r_v_image, r_v_template, m_r_t_H, m_t_H_inv);
		}

		template <class CDerived0, class CDerived1, class CDerived2>
		inline _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template) const
		{
			return v_Error_Jacobian(r_t_J, r_v_image, r_v_template, m_r_t_H, m_t_H_inv
#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
				, m_t_JInv
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
				);
		}

		template <class CDerived0, class CDerived1, class CDerived2>
		static inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template,
			const Eigen::MatrixBase<CDerived2> &r_t_H)
		{
			return v_Error(r_v_image, r_v_template, r_t_H, Eigen::Matrix3d(r_t_H.inverse()));
		}

		template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
		static inline _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template,
			const Eigen::MatrixBase<CDerived3> &r_t_H)
		{
			return v_Error_Jacobian(r_t_J, r_v_image, r_v_template, r_t_H,
				Eigen::Matrix3d(r_t_H.inverse())
#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
				, t_Inverse_Jacobian(r_t_H)
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
				);
		}

		template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
		static _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template,
			const Eigen::MatrixBase<CDerived2> &r_t_H,
			const Eigen::MatrixBase<CDerived3> &r_t_H_inv);

#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
		template <class CDerived0, class CDerived1, class CDerived2,
			class CDerived3, class CDerived4, class CDerived5>
		static _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template,
			const Eigen::MatrixBase<CDerived3> &r_t_H,
			const Eigen::MatrixBase<CDerived4> &r_t_H_inv,
			const Eigen::MatrixBase<CDerived5> &r_t_JInv);
#else // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
		template <class CDerived0, class CDerived1, class CDerived2, class CDerived3, class CDerived4>
		static _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template,
			const Eigen::MatrixBase<CDerived3> &r_t_H,
			const Eigen::MatrixBase<CDerived4> &r_t_H_inv);
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC

		static Eigen::Matrix<double, 9, 9> t_Inverse_Jacobian(const Eigen::Matrix3d &r_t_H);

#if !defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
		static inline double f_Sqr(double f_x)
		{
			return f_x * f_x;
		}
#endif // !SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
	};

	/**
	 *	@brief Sampson error
	 *	@note This is described in P.D. Sampson, "Fitting Conic Sections to ''very scattered''
	 *		Data: An Iterative Refinement of the Bookstein Algorithm", CGIP, Vol. 18, Is. 1,
	 *		pp. 97-108, Jan. 1982. Also in "Multiple-View Geometry" by Hartley & Zisserman.
	 */
	class CSampsonErrorObjective {
	public:
		enum {
			n_residual_dimension = 1
		};

		typedef Eigen::Matrix<double, n_residual_dimension, 1> _TyResidual;
		typedef Eigen::Matrix<double, n_residual_dimension, n_residual_dimension> _TyCovariance;
		typedef Eigen::Matrix<double, 9, n_residual_dimension> _TyJacobian;

	protected:
		const Eigen::Matrix3d &m_r_t_H;

	public:
		inline CSampsonErrorObjective(const Eigen::Matrix3d &r_t_H, bool UNUSED(b_will_do_jacobians))
			:m_r_t_H(r_t_H)
		{}

		template <class CDerived0, class CDerived1>
		inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template) const
		{
			return v_Error(r_v_image, r_v_template, m_r_t_H);
		}

		template <class CDerived0, class CDerived1, class CDerived2>
		inline _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_v_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template) const
		{
			return v_Error_Jacobian(r_v_J, r_v_image, r_v_template, m_r_t_H);
		}

		template <class CDerived0, class CDerived1, class CDerived2>
		static _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template,
			const Eigen::MatrixBase<CDerived2> &r_t_H);

		template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
		static _TyResidual v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_v_J,
			const Eigen::MatrixBase<CDerived1> &r_v_image,
			const Eigen::MatrixBase<CDerived2> &r_v_template,
			const Eigen::MatrixBase<CDerived3> &r_t_H);

	protected:
#ifdef TEST_SAMPSON_JAC
		static void calc2DHomogSampsonErr(const double m1[2], const double m2[2], const double h[9], double err[1]);
		static void calc2DHomogSampsonErrGrads(const double m1[2], const double m2[2], const double h[9], double grads[9]);
#endif // TEST_SAMPSON_JAC
	};

	/**
	 *	@brief reprojection error objective function
	 *	@note This is subtly different than the other objective functions,
	 *		minimizing reprojection error also requires estimating perfectly
	 *		fitting points (in this case the template points) and so the optimizer
	 *		state is completely different. See also \ref ImproveHomography_Reprojection().
	 */
	class CReprojectionErrorObjective {
	public:
		enum {
			n_residual_dimension = 4
		};

		typedef Eigen::Matrix<double, n_residual_dimension, 1> _TyResidual;
		typedef Eigen::Matrix<double, n_residual_dimension, n_residual_dimension> _TyCovariance; /**< @brief covariance in image (top left 2x2) then template (bottom right 2x2) */
		typedef Eigen::Matrix<double, 9, 2/*n_residual_dimension*/> _TyJacobian_H; /**< @brief derivative of the 4D projection function w.r.t. the homography (only the left 2 columns are nonzero) */
		typedef Eigen::Matrix<double, 2, n_residual_dimension> _TyJacobian_T; /**< @brief derivative of the 4D projection function w.r.t. the optimized template point */

	protected:
		const Eigen::Matrix3d &m_r_t_H;

	public:
		inline CReprojectionErrorObjective(const Eigen::Matrix3d &r_t_H, bool UNUSED(b_will_do_jacobians))
			:m_r_t_H(r_t_H)
		{}

		template <class CDerived0, class CDerived1, class CDerived2>
		inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template,
			const Eigen::MatrixBase<CDerived2> &r_v_template_opt) const
		{
			return v_Error(r_v_image, r_v_template, r_v_template_opt, m_r_t_H);
		}

		template <class CDerived0, class CDerived1, class CDerived2,
			class CDerived3, class CDerived4>
		inline _TyResidual v_Error_Jacobians(Eigen::MatrixBase<CDerived0> &r_t_J_H,
			Eigen::MatrixBase<CDerived1> &r_t_J_T,
			const Eigen::MatrixBase<CDerived2> &r_v_image,
			const Eigen::MatrixBase<CDerived3> &r_v_template,
			const Eigen::MatrixBase<CDerived4> &r_v_template_opt) const
		{
			return v_Error_Jacobians(r_t_J_H, r_t_J_T, r_v_image, r_v_template, r_v_template_opt, m_r_t_H);
		}

		template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
		static inline _TyResidual v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
			const Eigen::MatrixBase<CDerived1> &r_v_template,
			const Eigen::MatrixBase<CDerived2> &r_v_template_opt,
			const Eigen::MatrixBase<CDerived3> &r_t_H);

		template <class CDerived0, class CDerived1, class CDerived2,
			class CDerived3, class CDerived4, class CDerived5>
		static inline _TyResidual v_Error_Jacobians(Eigen::MatrixBase<CDerived0> &r_t_J_H,
			Eigen::MatrixBase<CDerived1> &r_t_J_T,
			const Eigen::MatrixBase<CDerived2> &r_v_image,
			const Eigen::MatrixBase<CDerived3> &r_v_template,
			const Eigen::MatrixBase<CDerived4> &r_v_template_opt,
			const Eigen::MatrixBase<CDerived5> &r_t_H);
	};

protected:
	// general version that works with different objectives
	template <class CObjective>
	static typename CEnableNotFor<CObjective, CReprojectionErrorObjective, Eigen::Matrix3d>::T
		t_Homography_Int(const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template, bool b_normalize,
		bool b_use_eigenvectors, int n_max_NLS_iteration_num, bool b_verbose)
	{
		const std::vector<_TyPoint2D_u> *p_template = &r_points_template, *p_image = &r_points_image;
		std::vector<_TyPoint2D_u> temp_tpl, temp_img;

		Eigen::Matrix3d t_Np, t_Ni;
		if(b_normalize) {
			temp_tpl = r_points_template;
			temp_img = r_points_image;
			t_Np = t_NormalizePoints(temp_tpl);
			t_Ni = t_NormalizePoints(temp_img); // don't damage the input vectors
			p_template = &temp_tpl;
			p_image = &temp_img;
		} else {
			t_Np.setIdentity();
			t_Ni.setIdentity();
		}

		Eigen::Matrix3d t_H = t_DLT_Homography(*p_image, *p_template, b_use_eigenvectors);
		if(n_max_NLS_iteration_num > 0) {
			//t_H += Eigen::Matrix3d::Random() * .1; // test the optimizer with noise
			ImproveHomography<CObjective>(t_H, *p_image, *p_template, n_max_NLS_iteration_num, b_verbose);
		}
		// calculate homography using DLT

		return t_Normalize(t_Ni * t_H * Eigen::Matrix3d(t_Np.inverse())); // Eigen workaround; inverse() of 3x3 sometimes gives wrong results
		// the full estimated homography, comparable to t_H
	}

	// specialization for the CReprojectionErrorObjective
	template <class CObjective>
	static typename CEnableFor<CObjective, CReprojectionErrorObjective, Eigen::Matrix3d>::T
		t_Homography_Int(const std::vector<_TyPoint2D_u> &r_points_image,
		const std::vector<_TyPoint2D_u> &r_points_template, bool b_normalize,
		bool b_use_eigenvectors, int n_max_NLS_iteration_num, bool b_verbose)
	{
		const std::vector<_TyPoint2D_u> *p_template = &r_points_template, *p_image = &r_points_image;
		std::vector<_TyPoint2D_u> temp_tpl, temp_img;

		Eigen::Matrix3d t_Np, t_Ni;
		if(b_normalize) {
			temp_tpl = r_points_template;
			temp_img = r_points_image;
			t_Np = t_NormalizePoints(temp_tpl);
			t_Ni = t_NormalizePoints(temp_img); // don't damage the input vectors
			p_template = &temp_tpl;
			p_image = &temp_img;
		} else {
			t_Np.setIdentity();
			t_Ni.setIdentity();
		}

		Eigen::Matrix3d t_H = t_DLT_Homography(*p_image, *p_template, b_use_eigenvectors);
		if(n_max_NLS_iteration_num > 0) {
			//t_H += Eigen::Matrix3d::Random() * .1; // test the optimizer with noise
			ImproveHomography_Reprojection(t_H, *p_image, *p_template, 0, n_max_NLS_iteration_num, b_verbose);
		}
		// calculate homography using DLT

		return t_Normalize(t_Ni * t_H * Eigen::Matrix3d(t_Np.inverse())); // Eigen workaround; inverse() of 3x3 sometimes gives wrong results
		// the full estimated homography, comparable to t_H
	}
};

#include "Homography.inl"

/** @} */ // end of group

#endif // !__HOMOGRAPHY_ESTIMATION_INCLUDED
