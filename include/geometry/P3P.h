/*
								+---------------------------------+
								|                                 |
								|  ***  3-point perspective  ***  |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2016 |
								|                                 |
								|              P3P.h              |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __THREE_POINT_PERSPECTIVE_SOLVER_INCLUDED
#define __THREE_POINT_PERSPECTIVE_SOLVER_INCLUDED

/**
 *	@file geometry/P3P.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief 3-point perspective solver
 *
 *	@date 2017-07-04
 *
 *	Changed the output transformation to be from the world space to camera space,
 *	to be consistent with the rest of the geometry module. This comes at a slight
 *	extra cost for P3P (three extra dot products are needed) and slightly reduces
 *	the cost of PNP. Fast \ref CP3PSolver::Solve_Rev() function which returns the
 *	reverse transformation is available.
 *
 *	The arguments are directions (camera), positions (world). The found transform
 *	goes from world to camera, making the order of arguments destination, source.
 *	This order is maintained throughout the geometry module.
 *
 */

#include <vector>
#include <utility> // std::pair
#include <algorithm> // std::min()
#include "geometry/PolySolve.h"
#include "eigen/Eigen/Core"
#include "eigen/Eigen/Geometry"
//#include "eigen/Eigen/QR" // did not use QR finally
#include "eigen/Eigen/LU" // PNP uses LU
#include "eigen/Eigen/Eigenvalues" // PNP uses eigenvalues to solve nasty polynomials
#include <complex>

/** \addtogroup geom
 *	@{
 */

/*
 * Copyright (c) 2011, Laurent Kneip, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * p3p.cpp
 *
 * Created on: Aug 6, 2013
 * Original Author: Laurent Kneip
 * Modifications by: Karl Schwabe
 * Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences.
 * Modifications form original:
 * Using the Eigen linear algebra library instead of the TooN library that Laurent Kneip used in his original implementation.
 * Output data as a vector of poses, as opposed to the large 3x16 matrix output by original implementation by Laurent Kneip.
 *
 * Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
 * Absolute Camera Position and Orientation
 */
/**
 *	@brief perspective three point problem solver originally by Laurent Kneip and Karl Schwabe
 *	@note Improved Eigen usage and optimized.
 */
class CP3PSolver {
public:
	
	/**
	 *	@brief solve the perspective three point problem
	 *
	 *	This finds transformations from world to camera space, i.e. for points \f$\mathbf{p} = \{p_i\}\f$
	 *	in world space and directions \f$\mathbf{d} = \{d_i\}\f$ in camera space, this gives a transformation
	 *	\f$T = \vert R t \vert\f$ such that \f$d_i = T\cdot p_i\f$.
	 *
	 *	@param[in] r_t_point_directions is a \f$3\times 3\f$ matrix with normalized directions
	 *		towards the observed points (each column is a vector)
	 *	@param[in] r_t_point_positions is a \f$3\times 3\f$ matrix with corresponding 3D world
	 *		points (each column is a point)
	 *	@param[out] r_solutions is an array of up to four \f$3\times 4\f$ matrices that contains
	 *		the transformations from world space to camera space (\f$3\times 3\f$ orientation
	 *		and \f$3\times 1\f$ position)
	 *
	 *	@return Returns true on success, false on failure (in case the world points are aligned or colinear).
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This is ever so slightly more expensive than Solve_Rev(), use that one instead in e.g. RANSAC.
	 */
	static bool Solve(const Eigen::Matrix3d &r_t_point_directions,
		const Eigen::Matrix3d &r_t_point_positions,
		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > &r_solutions) // throw(std::bad_alloc)
	{
		r_solutions.clear(); // !!

		Eigen::Vector3d P1 = r_t_point_positions.leftCols<1>();
		Eigen::Vector3d P2 = r_t_point_positions.leftCols<2>().rightCols<1>();
		Eigen::Vector3d P3 = r_t_point_positions.rightCols<1>();
		// extraction of world points

		if((P2 - P1).cross(P3 - P1).norm() == 0) // Eigen likes long expressions, use it. also apparent that this is not used anywhere else
			return false;
		// verification that world points are not colinear
		// this never ever returns, due to how the floating point works (maybe in case that two of the
		// points are identical but never if they are colinear, maybe unless they are somehow axis-aligned)

		Eigen::Vector3d f1 = r_t_point_directions.leftCols<1>();
		Eigen::Vector3d f2 = r_t_point_directions.leftCols<2>().rightCols<1>();
		Eigen::Vector3d f3 = r_t_point_directions.rightCols<1>();
		// extraction of feature vectors

		Eigen::Matrix3d T;
		{
			Eigen::Vector3d e1 = f1;
			Eigen::Vector3d e3 = f1.cross(f2).normalized();
			Eigen::Vector3d e2 = e3.cross(e1);
			T.leftCols<1>() = e1;
			T.leftCols<2>().rightCols<1>() = e2;
			T.rightCols<1>() = e3;
			f3 = T.transpose() * f3;
		}
		// creation of intermediate camera frame

		if(f3(2, 0) > 0) {
			f1 = r_t_point_directions.leftCols<2>().rightCols<1>();
			f2 = r_t_point_directions.leftCols<1>();
			f3 = r_t_point_directions.rightCols<1>();
			Eigen::Vector3d e1 = f1;
			Eigen::Vector3d e3 = f1.cross(f2).normalized();
			Eigen::Vector3d e2 = e3.cross(e1);
			T.leftCols<1>() = e1;
			T.leftCols<2>().rightCols<1>() = e2;
			T.rightCols<1>() = e3;
			f3 = T.transpose() * f3;
			P1 = r_t_point_positions.leftCols<2>().rightCols<1>();
			P2 = r_t_point_positions.leftCols<1>();
			P3 = r_t_point_positions.rightCols<1>();
		}
		// reinforce that f3(2,0) > 0 for having theta in [0;pi]

		Eigen::Vector3d n1 = (P2 - P1).normalized();
		Eigen::Vector3d n3 = (n1.cross(P3 - P1)).normalized();
		Eigen::Vector3d n2 = n3.cross(n1);
		Eigen::Matrix3d N;
		N.leftCols<1>() = n1;
		N.leftCols<2>().rightCols<1>() = n2;
		N.rightCols<1>() = n3;
		// creation of intermediate world frame

		P3 = N.transpose() * (P3 - P1);
		double d_12 = (P2 - P1).norm();
		f3.head<2>() /= f3(2);
		double f_1 = f3(0), f_2 = f3(1), p_1 = P3(0), p_2 = P3(1);
		double cos_beta = f1.dot(f2);
		double b = 1 / (1 - cos_beta * cos_beta) - 1;
		if(cos_beta < 0)
			b = -sqrt(b);
		else
			b = sqrt(b);
		// extraction of known parameters

		double f_1_pw2 = f_1 * f_1, f_2_pw2 = f_2 * f_2, p_1_pw2 = p_1 * p_1, p_2_pw2 = p_2 * p_2;
		double p_1_pw3 = p_1_pw2 * p_1, p_2_pw3 = p_2_pw2 * p_2;
		double p_1_pw4 = p_1_pw3 * p_1, p_2_pw4 = p_2_pw3 * p_2;
		double d_12_pw2 = d_12 * d_12, b_pw2 = b * b;
		// definition of temporary variables for avoiding multiple computation

		Eigen::Matrix<double, 5, 1> factors;
		factors(0) = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
		factors(1) = 2 * p_2_pw3 * d_12 * b + 2 * f_2_pw2 * p_2_pw3 * d_12 * b - 2 * f_2 * p_2_pw3 * f_1 * d_12;
		factors(2) = -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 +
			f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 + 2 * p_1 * p_2_pw2 * d_12 + 2 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b -
			p_2_pw2 * p_1_pw2 * f_1_pw2 + 2 * p_1 * p_2_pw2 * f_2_pw2 * d_12 - p_2_pw2 * d_12_pw2 * b_pw2 -
			2 * p_1_pw2 * p_2_pw2;
		factors(3) = 2 * p_1_pw2 * p_2 * d_12 * b + 2 * f_2 * p_2_pw3 * f_1 * d_12 - 2 * f_2_pw2 * p_2_pw3 * d_12 * b -
			2 * p_1 * p_2 * d_12_pw2 * b;
		factors(4) = -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b + f_2_pw2 * p_2_pw2 * d_12_pw2 + 2 * p_1_pw3 * d_12 -
			p_1_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw2 * p_1_pw2 - p_1_pw4 - 2 * f_2_pw2 * p_2_pw2 * p_1 * d_12 +
			p_2_pw2 * f_1_pw2 * p_1_pw2 + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;
		// computation of the factors of 4th degree polynomial

		Eigen::Vector4d realRoots;
#if 0 // Laurent's transcendental solver, about 14 usec per P3P
		SolveQuartic(factors, realRoots);
		int n_root_num = 4; // always returns four roots, even though they might have imaginary component
#else // 0 // lame quartic solver, about 9 usec per P3P
		CQuarticEq<double> eq(factors(0), factors(1), factors(2), factors(3), factors(4));
		int n_root_num = int(eq.n_RealRoot_Num());
		for(int i = 0; i < n_root_num; ++ i)
			realRoots(i) = eq.f_RealRoot(i);
		/*static struct TDebugHist {
		protected:
			std::map<int, size_t> h;

		public:
			~TDebugHist()
			{
				for(std::map<int, size_t>::const_iterator p_hist_it = h.begin(),
				   p_end_it = h.end(); p_hist_it != p_end_it; ++ p_hist_it)
					printf("%d: %dx\n", (*p_hist_it).first, int((*p_hist_it).second));
			}

			void operator ()(int nr)
			{
				++ h[nr];
			}
		} hist;
		hist(n_root_num);*/ // this shows there are mostly two roots
#endif // 0
		// computation of the roots

		r_solutions.resize(n_root_num); // throws
		for(int i = 0; i < n_root_num; ++ i) {
			double cot_alpha = (-f_1 * p_1 / f_2 - realRoots(i) * p_2 + d_12 * b) /
				(-f_1 * realRoots(i) * p_2 / f_2 + p_1 - d_12);
			double cos_theta = realRoots(i);
			double sin_theta = sqrt(1 - cos_theta * cos_theta);
			double sin_alpha = sqrt(1 / (cot_alpha * cot_alpha + 1));
			double cos_alpha = sqrt(1 - sin_alpha * sin_alpha);
			if(cot_alpha < 0)
				cos_alpha = -cos_alpha;

			Eigen::Vector3d C;
			C(0) = d_12 * cos_alpha * (sin_alpha * b + cos_alpha);
			C(1) = cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
			C(2) = sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
			Eigen::Matrix3d R;
			R << -cos_alpha, -sin_alpha * cos_theta, -sin_alpha * sin_theta,
				  sin_alpha, -cos_alpha * cos_theta, -cos_alpha * sin_theta,
				          0,             -sin_theta,              cos_theta;
			Eigen::Matrix<double, 3, 4, Eigen::DontAlign> &solution = r_solutions[i]; // don't have to copy twice
			solution.topLeftCorner<3, 3>().noalias() = T * R * N.transpose(); // transpose (=inverse) rotation // since 2017-07-04
			solution.rightCols<1>().noalias() = -(solution.topLeftCorner<3, 3>() * (P1 + N * C)); // inverse translation (incurs an extra matrix-vector multiplication) // since 2017-07-04
		}
		// backsubstitution of each solution

		return true;
	}

	/**
	 *	@brief solve the perspective three point problem, give inverse transformation (camera to world)
	 *
	 *	This finds transformations from camera to world space, i.e. for points \f$\mathbf{p} = \{p_i\}\f$
	 *	in world space and directions \f$\mathbf{d} = \{d_i\}\f$ in camera space, this gives a transformation
	 *	\f$T = \vert R t \vert\f$ such that \f$p_i = T\cdot d_i\f$ (a mapping from lines to points).
	 *
	 *	@param[in] r_t_point_directions is a \f$3\times 3\f$ matrix with normalized directions
	 *		towards the observed points (each column is a vector)
	 *	@param[in] r_t_point_positions is a \f$3\times 3\f$ matrix with corresponding 3D world
	 *		points (each column is a point)
	 *	@param[out] r_solutions is an array of up to four \f$3\times 4\f$ matrices that contains
	 *		the transformations from camera space to the world space (\f$3\times 3\f$ orientation
	 *		and \f$3\times 1\f$ position)
	 *
	 *	@return Returns true on success, false on failure (in case the world points are aligned or colinear).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static bool Solve_Rev(const Eigen::Matrix3d &r_t_point_directions,
		const Eigen::Matrix3d &r_t_point_positions,
		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > &r_solutions) // throw(std::bad_alloc)
	{
		r_solutions.clear(); // !!

		Eigen::Vector3d P1 = r_t_point_positions.leftCols<1>();
		Eigen::Vector3d P2 = r_t_point_positions.leftCols<2>().rightCols<1>();
		Eigen::Vector3d P3 = r_t_point_positions.rightCols<1>();
		// extraction of world points

		if((P2 - P1).cross(P3 - P1).norm() == 0) // Eigen likes long expressions, use it. also apparent that this is not used anywhere else
			return false;
		// verification that world points are not colinear
		// this never ever returns, due to how the floating point works (maybe in case that two of the
		// points are identical but never if they are colinear, maybe unless they are somehow axis-aligned)

		Eigen::Vector3d f1 = r_t_point_directions.leftCols<1>();
		Eigen::Vector3d f2 = r_t_point_directions.leftCols<2>().rightCols<1>();
		Eigen::Vector3d f3 = r_t_point_directions.rightCols<1>();
		// extraction of feature vectors

		Eigen::Matrix3d T;
		{
			Eigen::Vector3d e1 = f1;
			Eigen::Vector3d e3 = f1.cross(f2);
			e3.normalize(); // !! //e3 = e3 / e3.norm();
			Eigen::Vector3d e2 = e3.cross(e1);
			T.leftCols<1>() = e1;
			T.leftCols<2>().rightCols<1>() = e2;
			T.rightCols<1>() = e3;
			f3 = T.transpose() * f3;
		}
		// creation of intermediate camera frame

		if(f3(2, 0) > 0) {
			f1 = r_t_point_directions.leftCols<2>().rightCols<1>();
			f2 = r_t_point_directions.leftCols<1>();
			f3 = r_t_point_directions.rightCols<1>();
			Eigen::Vector3d e1 = f1;
			Eigen::Vector3d e3 = f1.cross(f2);
			e3.normalize(); // = e3 / e3.norm();
			Eigen::Vector3d e2 = e3.cross(e1);
			T.leftCols<1>() = e1;
			T.leftCols<2>().rightCols<1>() = e2;
			T.rightCols<1>() = e3;
			f3 = T.transpose() * f3;
			P1 = r_t_point_positions.leftCols<2>().rightCols<1>();
			P2 = r_t_point_positions.leftCols<1>();
			P3 = r_t_point_positions.rightCols<1>();
		}
		// reinforce that f3(2,0) > 0 for having theta in [0;pi]

		Eigen::Vector3d n1 = P2 - P1;
		n1.normalize(); // = n1 / n1.norm();
		Eigen::Vector3d n3 = n1.cross(P3 - P1);
		n3.normalize(); // = n3 / n3.norm();
		Eigen::Vector3d n2 = n3.cross(n1);
		Eigen::Matrix3d N;
		N.leftCols<1>() = n1;
		N.leftCols<2>().rightCols<1>() = n2;
		N.rightCols<1>() = n3;
		// creation of intermediate world frame

		P3 = N.transpose() * (P3 - P1);
		double d_12 = (P2 - P1).norm();
		double f_1 = f3(0) / f3(2);
		double f_2 = f3(1) / f3(2);
		double p_1 = P3(0);
		double p_2 = P3(1);
		double cos_beta = f1.dot(f2);
		double b = 1 / (1 - cos_beta * cos_beta/*pow(cos_beta, 2)*/) - 1;
		if(cos_beta < 0)
			b = -sqrt(b);
		else
			b = sqrt(b);
		// extraction of known parameters

		double f_1_pw2 = f_1 * f_1;//pow(f_1, 2);
		double f_2_pw2 = f_2 * f_2;//pow(f_2, 2);
		double p_1_pw2 = p_1 * p_1;//pow(p_1, 2);
		double p_1_pw3 = p_1_pw2 * p_1;
		double p_1_pw4 = p_1_pw3 * p_1;
		double p_2_pw2 = p_2 * p_2;//pow(p_2, 2);
		double p_2_pw3 = p_2_pw2 * p_2;
		double p_2_pw4 = p_2_pw3 * p_2;
		double d_12_pw2 = d_12 * d_12;//pow(d_12, 2);
		double b_pw2 = b * b;//pow(b, 2); // calling pow() with integer power is waste of time and precision
		// definition of temporary variables for avoiding multiple computation

		Eigen::Matrix<double, 5, 1> factors;
		factors(0) = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
		factors(1) = 2 * p_2_pw3 * d_12 * b + 2 * f_2_pw2 * p_2_pw3 * d_12 * b - 2 * f_2 * p_2_pw3 * f_1 * d_12;
		factors(2) = -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 +
			f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 + 2 * p_1 * p_2_pw2 * d_12 + 2 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b -
			p_2_pw2 * p_1_pw2 * f_1_pw2 + 2 * p_1 * p_2_pw2 * f_2_pw2 * d_12 - p_2_pw2 * d_12_pw2 * b_pw2 -
			2 * p_1_pw2 * p_2_pw2;
		factors(3) = 2 * p_1_pw2 * p_2 * d_12 * b + 2 * f_2 * p_2_pw3 * f_1 * d_12 - 2 * f_2_pw2 * p_2_pw3 * d_12 * b -
			2 * p_1 * p_2 * d_12_pw2 * b;
		factors(4) = -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b + f_2_pw2 * p_2_pw2 * d_12_pw2 + 2 * p_1_pw3 * d_12 -
			p_1_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw2 * p_1_pw2 - p_1_pw4 - 2 * f_2_pw2 * p_2_pw2 * p_1 * d_12 +
			p_2_pw2 * f_1_pw2 * p_1_pw2 + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;
		// computation of the factors of 4th degree polynomial

		Eigen::Vector4d realRoots;
#if 0 // Laurent's transcendental solver, about 14 usec per P3P
		SolveQuartic(factors, realRoots);
		int n_root_num = 4; // always returns four roots, even though they might have imaginary component
#else // 0 // lame quartic solver, about 9 usec per P3P
		CQuarticEq<double> eq(factors(0), factors(1), factors(2), factors(3), factors(4));
		int n_root_num = int(eq.n_RealRoot_Num());
		for(int i = 0; i < n_root_num; ++ i)
			realRoots(i) = eq.f_RealRoot(i);
		/*static struct TDebugHist {
		protected:
			std::map<int, size_t> h;

		public:
			~TDebugHist()
			{
				for(std::map<int, size_t>::const_iterator p_hist_it = h.begin(),
				   p_end_it = h.end(); p_hist_it != p_end_it; ++ p_hist_it)
					printf("%d: %dx\n", (*p_hist_it).first, int((*p_hist_it).second));
			}

			void operator ()(int nr)
			{
				++ h[nr];
			}
		} hist;
		hist(n_root_num);*/ // this shows there are mostly two roots
#endif // 0
		// computation of the roots

		r_solutions.resize(n_root_num); // throws
		for(int i = 0; i < n_root_num; ++ i) {
			double cot_alpha = (-f_1 * p_1 / f_2 - realRoots(i) * p_2 + d_12 * b) /
				(-f_1 * realRoots(i) * p_2 / f_2 + p_1 - d_12);
			double cos_theta = realRoots(i);
			double sin_theta = sqrt(1 - cos_theta * cos_theta/*pow((double)realRoots(i), 2)*/);
			double sin_alpha = sqrt(1 / (cot_alpha * cot_alpha/*pow(cot_alpha, 2)*/ + 1));
			double cos_alpha = sqrt(1 - sin_alpha * sin_alpha/*pow(sin_alpha, 2)*/); // pow(, 2) is slow and imprecise
			if(cot_alpha < 0)
				cos_alpha = -cos_alpha;

			Eigen::Vector3d C;
			C(0) = d_12 * cos_alpha * (sin_alpha * b + cos_alpha);
			C(1) = cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
			C(2) = sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
			//C = P1 + N.transpose() * C;
			Eigen::Matrix3d R;
			R << -cos_alpha, -sin_alpha * cos_theta, -sin_alpha * sin_theta,
				  sin_alpha, -cos_alpha * cos_theta, -cos_alpha * sin_theta,
				          0,             -sin_theta,              cos_theta;
			//R = N.transpose() * R.transpose() * T;
			Eigen::Matrix<double, 3, 4, Eigen::DontAlign> &solution = r_solutions[i]; // don't have to copy twice
			solution.topLeftCorner<3, 3>()/*block<3, 3>(0, 0)*/.noalias() = N * R.transpose() * T.transpose();//R; // don't have to copy twice, can multiply without a temporary copy
			solution.rightCols<1>().noalias() = P1 + N * C;//C; // don't have to copy twice, can multiply without a temporary copy
			//p_solutions[i] = solution;
		}
		// backsubstitution of each solution

		return true;
	}

protected:
	/**
	 *	@brief solve quartic polynomial using complex numbers and transcendentals
	 *
	 *	@param[in] r_factors is a vector of factors
	 *	@param[out] r_real_roots is filled with real parts of the four roots
	 *		(some of them may have imaginary component to them)
	 *
	 *	@note This is a bit sluggish and not horribly precise. Better use
	 *		\ref CQuarticEq instead.
	 */
	static void SolveQuartic(const Eigen::Matrix<double, 5, 1> &r_factors,
		Eigen::Vector4d &r_real_roots)
	{
		double A = r_factors(0);
		double B = r_factors(1);
		double C = r_factors(2);
		double D = r_factors(3);
		double E = r_factors(4);
		double A_pw2 = A * A;
		double B_pw2 = B * B;
		double A_pw3 = A_pw2 * A;
		double B_pw3 = B_pw2 * B;
		double A_pw4 = A_pw3 * A;
		double B_pw4 = B_pw3 * B;
		double alpha = -3 * B_pw2 / (8 * A_pw2) + C / A;
		double beta = B_pw3 / (8 * A_pw3) - B * C / (2 * A_pw2) + D / A;
		double gamma = -3 * B_pw4 / (256 * A_pw4) + B_pw2 * C / (16 * A_pw3) - B * D / (4 * A_pw2) + E / A;
		double alpha_pw2 = alpha * alpha;
		double alpha_pw3 = alpha_pw2 * alpha;
		std::complex<double> P(-alpha_pw2 / 12 - gamma, 0);
		std::complex<double> Q(-alpha_pw3 / 108 + alpha * gamma / 3 - beta * beta/*pow(beta, 2)*/ / 8, 0);
		std::complex<double> R = -Q / 2.0 + sqrt(Q * Q/*pow(Q, 2.0)*/ / 4.0 + P * P * P/*pow(P, 3.0)*/ / 27.0);
		std::complex<double> U = pow(R, 1.0 / 3);
		std::complex<double> Y;

		if(U.real() == 0)
			Y = -5 * alpha / 6 - pow(Q, 1.0 / 3);
		else
			Y = -5 * alpha / 6 - P / (3.0 * U) + U;

		std::complex<double> W = sqrt(alpha + 2.0 * Y);
		r_real_roots(0) = (-B / (4.0 * A) + .5 * (W + sqrt(-(3 * alpha + 2.0 * Y + 2.0 * beta / W)))).real();
		r_real_roots(1) = (-B / (4.0 * A) + .5 * (W - sqrt(-(3 * alpha + 2.0 * Y + 2.0 * beta / W)))).real();
		r_real_roots(2) = (-B / (4.0 * A) + .5 * (-W + sqrt(-(3 * alpha + 2.0 * Y - 2.0 * beta / W)))).real();
		r_real_roots(3) = (-B / (4.0 * A) + .5 * (-W - sqrt(-(3 * alpha + 2.0 * Y - 2.0 * beta / W)))).real();

		/*
		root response precision 1e-2147483648: 2105164407 cases
		root response precision 1e-25: 879 cases
		root response precision 1e-24: 1411 cases
		root response precision 1e-23: 1065 cases
		root response precision 1e-22: 2646 cases
		root response precision 1e-21: 5054 cases
		root response precision 1e-20: 7994 cases
		root response precision 1e-19: 14458 cases
		root response precision 1e-18: 30957 cases
		root response precision 1e-17: 80655 cases
		root response precision 1e-16: 371132 cases
		root response precision 1e-15: 376273063 cases
		root response precision 1e-14: 966053874 cases
		root response precision 1e-13: 429297058 cases
		root response precision 1e-12: 69121758 cases
		root response precision 1e-11: 6470462 cases
		root response precision 1e-10: 420816 cases
		root response precision 1e-9: 17000 cases
		root response precision 1e-8: 75 cases
		root response precision 1e-4: 1 cases
		root response precision 1e-3: 8 cases
		root response precision 1e-2: 85 cases
		root response precision 1e-1: 122 cases
		root response precision 1e0: 556 cases
		root response precision 1e1: 41 cases
		root response precision 1e2: 20 cases
		root response precision 1e3: 10 cases

		solve rate 0.006196 msec/equation
		*/
		// this is not very fast, and not precise at all (it uses a lot of transcendental functions on complex numbers)

		// todo - replace by a robust version, this will not work with real numbers (lukas will do it)

		//return 0; // does not really return anything, does it?
	}
};

/**
 *	@brief perspective n-point solver based on the 2011 paper by J. Hesch
 *
 *	Joel A. Hesch and Stergios I. Roumeliotis. A Direct Least-Squares (DLS) Method for PnP.
 *	In Proc. of the Intl. Conf. on Computer Vision, Barcelona, Spain, November 6-13, 2011.
 */
class CPNPSolver_DLS {
public:
	/**
	 *	@brief solve the perspective three point problem
	 *
	 *	This finds transformations from world to camera space, i.e. for points \f$\mathbf{p} = \{p_i\}\f$
	 *	in world space and directions \f$\mathbf{d} = \{d_i\}\f$ in camera space, this gives a transformation
	 *	\f$T = \vert R t \vert\f$ such that \f$d_i = T\cdot p_i\f$.
	 *
	 *	@tparam Derived0 is derived class for the matrix type of the first argument
	 *	@tparam Derived1 is derived class for the matrix type of the second argument
	 *
	 *	@param[in] r_t_point_directions is a \f$3\times n\f$ matrix with normalized directions
	 *		towards the observed points (each column is a vector)
	 *	@param[in] r_t_point_positions is a \f$3\times n\f$ matrix with corresponding 3D world
	 *		points (each column is a point)
	 *	@param[out] r_solutions is an array of up to 27 \f$3\times 4\f$ matrices that contains
	 *		the transformations from the world space to camera space (\f$3\times 3\f$ orientation
	 *		and \f$3\times 1\f$ position)
	 *	@param[out] p_costs is an optional array of costs of the solutions (each entry corresponds
	 *		to one solution); pass null pointer if not needed
	 *
	 *	@return Always returns true.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This method relies on randomness (for Eigenvalues and elsewhere). You may get
	 *		subtly different results each time, depending on the random number generator seed.
	 *		This also depends on how well is the solution conditioned.
	 */
	template <class Derived0, class Derived1>
	static bool Solve(const Eigen::MatrixBase<Derived0> &r_t_point_directions,
		const Eigen::MatrixBase<Derived1> &r_t_point_positions,
		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > &r_solutions,
		std::vector<double> *p_costs = 0) // throw(std::bad_alloc)
	{
		r_solutions.clear(); // !!
		if(p_costs)
			p_costs->clear();

		const Eigen::Vector3d v_offset = r_t_point_positions.rowwise().mean();
		// take points mean

		Eigen::Matrix3d eR[3];
		eR[0] << 1, 0, 0, 0, 0, -1, 0, 1, 0;
		eR[1] << 0, 0, 1, 0, 1, 0, -1, 0, 0;
		eR[2] << 0, -1, 0, 1, 0, 0, 0, 0, 1;
		// make elementary axis rotations

		double f_best_cost;
		for(int n_pass = 0; n_pass < 3; ++ n_pass) {
			Derived1 t_transformed_positions = eR[n_pass] * (r_t_point_positions.colwise() - v_offset); // subtract offset from each column in r_t_point_positions
			// transform and offset the points

			std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > solutions;
			std::vector<double> costs;
			if(!FastSingleSolve(r_t_point_directions, t_transformed_positions, solutions, &costs) ||
			   solutions.empty())
				continue;
			// solve the modified problem

			double f_cost = *std::min_element(costs.begin(), costs.end());
			if(r_solutions.empty() || f_best_cost > f_cost) {
				for(size_t i = 0, n = solutions.size(); i < n; ++ i) {
					Eigen::Matrix<double, 3, 4, Eigen::DontAlign> Rt = solutions[i];
					Eigen::Matrix3d R = Rt.leftCols<3>()/*.transpose()*/;
					Eigen::Vector3d t = /*-(R **/ Rt.rightCols<1>()/*)*/;
					// invert // not since 2017-07-04

					t -= R * eR[n_pass] * v_offset;
					R *= eR[n_pass];
					// unapply the rotation and offset

					Rt.leftCols<3>() = R/*.transpose()*/;
					Rt.rightCols<1>() = /*-(R.transpose() **/ t/*)*/;
					solutions[i] = Rt;
					// invert back // not since 2017-07-04
				}
				// unapply the transformation to get the solution to the original problem

				if(p_costs)
					costs.swap(*p_costs); // swap to the output
				solutions.swap(r_solutions); // swap to the output
				f_best_cost = f_cost; // remember the best cost
			}
		}

		return true;
	}

	/**
	 *	@brief solve the perspective three point problem; does a single solution attempt and only
	 *		falls back to precise solution if it fails
	 *
	 *	This finds transformations from world to camera space, i.e. for points \f$\mathbf{p} = \{p_i\}\f$
	 *	in world space and directions \f$\mathbf{d} = \{d_i\}\f$ in camera space, this gives a transformation
	 *	\f$T = \vert R t \vert\f$ such that \f$d_i = T\cdot p_i\f$.
	 *
	 *	@tparam Derived0 is derived class for the matrix type of the first argument
	 *	@tparam Derived1 is derived class for the matrix type of the second argument
	 *
	 *	@param[in] r_t_point_directions is a \f$3\times n\f$ matrix with normalized directions
	 *		towards the observed points (each column is a vector)
	 *	@param[in] r_t_point_positions is a \f$3\times n\f$ matrix with corresponding 3D world
	 *		points (each column is a point)
	 *	@param[out] r_solutions is an array of up to 27 \f$3\times 4\f$ matrices that contains
	 *		the transformations from the world space to camera space (\f$3\times 3\f$ orientation
	 *		and \f$3\times 1\f$ position)
	 *	@param[out] p_costs is an optional array of costs of the solutions (each entry corresponds
	 *		to one solution); pass null pointer if not needed
	 *
	 *	@return Returns true on success, false on failure (in case the world points are aligned or colinear).
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This method relies on randomness (for Eigenvalues and elsewhere). You may get
	 *		subtly different results each time, depending on the random number generator seed.
	 *		This also depends on how well is the solution conditioned.
	 */
	template <class Derived0, class Derived1>
	static bool FastSolve(const Eigen::MatrixBase<Derived0> &r_t_point_directions,
		const Eigen::MatrixBase<Derived1> &r_t_point_positions,
		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > &r_solutions,
		std::vector<double> *p_costs = 0) // throw(std::bad_alloc)
	{
		if(!FastSingleSolve(r_t_point_directions, r_t_point_positions, r_solutions, p_costs) || r_solutions.empty())
			return Solve(r_t_point_directions, r_t_point_positions, r_solutions, p_costs);
		return true;
	}

	/**
	 *	@brief solve the perspective three point problem; the fastest but fails for pi rotations (Cayley singularity)
	 *
	 *	This finds transformations from world to camera space, i.e. for points \f$\mathbf{p} = \{p_i\}\f$
	 *	in world space and directions \f$\mathbf{d} = \{d_i\}\f$ in camera space, this gives a transformation
	 *	\f$T = \vert R t \vert\f$ such that \f$d_i = T\cdot p_i\f$.
	 *
	 *	@tparam Derived0 is derived class for the matrix type of the first argument
	 *	@tparam Derived1 is derived class for the matrix type of the second argument
	 *
	 *	@param[in] r_t_point_directions is a \f$3\times n\f$ matrix with normalized directions
	 *		towards the observed points (each column is a vector)
	 *	@param[in] r_t_point_positions is a \f$3\times n\f$ matrix with corresponding 3D world
	 *		points (each column is a point)
	 *	@param[out] r_solutions is an array of up to 27 \f$3\times 4\f$ matrices that contains
	 *		the transformations from the world space to camera space (\f$3\times 3\f$ orientation
	 *		and \f$3\times 1\f$ position)
	 *	@param[out] p_costs is an optional array of costs of the solutions (each entry corresponds
	 *		to one solution); pass null pointer if not needed
	 *
	 *	@return Returns true on success, false on failure (in case the world points are aligned or colinear).
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This method relies on randomness (for Eigenvalues and elsewhere). You may get
	 *		subtly different results each time, depending on the random number generator seed.
	 *		This also depends on how well is the solution conditioned.
	 */
	template <class Derived0, class Derived1>
	static bool FastSingleSolve(const Eigen::MatrixBase<Derived0> &r_t_point_directions,
		const Eigen::MatrixBase<Derived1> &r_t_point_positions,
		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > &r_solutions,
		std::vector<double> *p_costs = 0) // throw(std::bad_alloc)
	{
		r_solutions.clear(); // !!
		if(p_costs)
			p_costs->clear();

		_ASSERTE(r_t_point_directions.rows() == 3);
		_ASSERTE(r_t_point_positions.rows() == 3);
		_ASSERTE(r_t_point_directions.cols() == r_t_point_positions.cols());
		_ASSERTE(r_t_point_directions.cols() >= 3);
		// sanity check (do not require that the number of rows is compile-time
		// constant but it would be better if it is, for performance reasons)

		const size_t n = r_t_point_directions.cols();
		// the number öf points / correspondences

		const Eigen::Matrix3d H_inv = Eigen::Matrix3d::Identity() * double(n) -
			r_t_point_directions * r_t_point_directions.transpose();
		// build inverse of the coeff matrix H

		typedef const Eigen::Block<const Derived0, Derived0::RowsAtCompileTime,
			1, !(Derived0::Options & Eigen::RowMajor)> _TyConstDirCol;
		typedef const Eigen::Block<const Derived1, Derived1::RowsAtCompileTime,
			1, !(Derived1::Options & Eigen::RowMajor)> _TyConstPosCol;

		Eigen::Matrix<double, 3, 9> A = Eigen::Matrix<double, 3, 9>::Zero();
		for(size_t i = 0; i < n; ++ i) { // t_odo - factor out the column expressions, keep them as intermediates
			_TyConstDirCol &v_dir_i = r_t_point_directions.col(i);
			_TyConstPosCol &v_pos_i = r_t_point_positions.col(i);
			A.leftCols<3>() += (v_dir_i * v_dir_i(0) - Eigen::Vector3d(1.0, .0, .0)) * v_pos_i.transpose();
			A.rightCols<6>().leftCols<3>() += (v_dir_i * v_dir_i(1) - Eigen::Vector3d(.0, 1.0, .0)) * v_pos_i.transpose();
			A.rightCols<3>() += (v_dir_i * v_dir_i(2) - Eigen::Vector3d(.0, .0, 1.0)) * v_pos_i.transpose();
		}
		const Eigen::PartialPivLU<Eigen::Matrix3d> LU_H_inv(H_inv); // needed later
		A = LU_H_inv.solve(A);
		// this is really similar to N-line intersection, except that the point
		// positions and directions exist in different spaces and the rotation relating
		// those spaces is yet to be determined

		Eigen::Matrix<double, 9, 9> D = Eigen::Matrix<double, 9, 9>::Zero();
		for(size_t i = 0; i < n; ++ i) {
			Eigen::Matrix<double, 3, 9> Ap = A/* + t_LMV(r_t_point_positions.col(i))*/;
			_TyConstPosCol &v_pos_i = r_t_point_positions.col(i);
			Ap.topLeftCorner<1, 3>() += v_pos_i.transpose();
			Ap.bottomRightCorner<2, 6>().topLeftCorner<1, 3>() += v_pos_i.transpose();
			Ap.bottomRightCorner<1, 3>() += v_pos_i.transpose(); // t_odo - reformulate LMV as addition
			_TyConstDirCol &v_dir_i = r_t_point_directions.col(i);
			D += Ap.transpose() * (Eigen::Matrix3d::Identity() - v_dir_i * v_dir_i.transpose()) * Ap;
		}
		// assemble the D matrix

		// t_odo - could simplify the constant factors below
		// t_odo - mention randomness in the doc (eigs, u)
		// t_odo - mention cayley singularity (and add the "robust" version)

		Eigen::Matrix<double, 20, 1> f1coeff, f2coeff, f3coeff;
		f1coeff << 2 * (D(0, 5) - D(0, 7) + D(4, 5) - D(4, 7) + D(5, 0) + D(5, 4) + D(5, 8) - D(7, 0) - D(7, 4) - D(7, 8) +
			D(8, 5) - D(8, 7)), 6 * (D(0, 1) + D(0, 3) + D(1, 0) - D(1, 4) - D(1, 8) + D(3, 0) - D(3, 4) - D(3, 8) -
			D(4, 1) - D(4, 3) - D(8, 1) - D(8, 3)), 4 * (D(0, 6) - D(0, 2) - D(2, 0) + D(2, 4) + D(2, 8) + D(4, 2) -
			D(4, 6) + D(6, 0) - D(6, 4) - D(6, 8) + D(8, 2) - D(8, 6)) + 8 * (D(1, 5) - D(1, 7) + D(3, 5) - D(3, 7) +
			D(5, 1) + D(5, 3) - D(7, 1) - D(7, 3)), 4 * (D(0, 1) - D(0, 3) + D(1, 0) - D(1, 4) - D(1, 8) - D(3, 0) +
			D(3, 4) + D(3, 8) - D(4, 1) + D(4, 3) - D(8, 1) + D(8, 3)) + 8 * (D(2, 5) - D(2, 7) + D(5, 2) + D(5, 6) +
			D(6, 5) - D(6, 7) - D(7, 2) - D(7, 6)), 8 * (D(1, 1) - D(2, 2) - D(3, 3) + D(5, 5) + D(6, 6) - D(7, 7)), 4 *
			(D(1, 5) + D(1, 7) + D(3, 5) + D(3, 7) + D(5, 1) + D(5, 3) + D(7, 1) + D(7, 3)) + 2 * (-D(0, 6) - D(0, 2) -
			D(2, 0) + D(2, 4) - D(2, 8) + D(4, 2) + D(4, 6) - D(6, 0) + D(6, 4) - D(6, 8) - D(8, 2) - D(8, 6)), 2 *
			(D(1, 4) - D(0, 3) - D(1, 0) - D(0, 1) - D(1, 8) - D(3, 0) + D(3, 4) - D(3, 8) + D(4, 1) + D(4, 3) - D(8, 1) -
			D(8, 3)), 4 * (D(0, 8) - D(0, 0) + D(4, 4) + D(8, 0) - D(8, 8)) + 8 * (D(2, 2) + D(2, 6) + D(6, 2) +
			D(6, 6)), 4 * (D(0, 0) - D(4, 4) - D(4, 8) - D(8, 4) - D(8, 8)) + 8 * (D(5, 5) - D(5, 7) - D(7, 5) +
			D(7, 7)), 4 * (D(1, 5) - D(1, 7) - D(3, 5) + D(3, 7) + D(5, 1) - D(5, 3) - D(7, 1) + D(7, 3)) + 2 * (D(0, 2) +
			D(0, 6) + D(2, 0) + D(2, 4) + D(2, 8) + D(4, 2) + D(4, 6) + D(6, 0) + D(6, 4) + D(6, 8) + D(8, 2) +
			D(8, 6)), 4 * (-D(2, 5) + D(2, 7) - D(5, 2) + D(5, 6) + D(6, 5) - D(6, 7) + D(7, 2) - D(7, 6)) + 2 *
			(D(0, 1) + D(0, 3) + D(1, 0) + D(1, 4) + D(1, 8) + D(3, 0) + D(3, 4) + D(3, 8) + D(4, 1) + D(4, 3) + D(8, 1) +
			D(8, 3)), 4 * (D(2, 5) + D(2, 7) + D(5, 2) + D(5, 6) + D(6, 5) + D(6, 7) + D(7, 2) + D(7, 6)) + 2 * (D(1, 8) -
			D(0, 3) - D(1, 0) - D(1, 4) - D(0, 1) - D(3, 0) - D(3, 4) + D(3, 8) - D(4, 1) - D(4, 3) + D(8, 1) +
			D(8, 3)), 6 * (D(0, 5) - D(0, 7) - D(4, 5) + D(4, 7) + D(5, 0) - D(5, 4) - D(5, 8) - D(7, 0) + D(7, 4) +
			D(7, 8) - D(8, 5) + D(8, 7)), 4 * (D(1, 2) + D(1, 6) + D(2, 1) - D(2, 3) - D(3, 2) - D(3, 6) + D(6, 1) -
			D(6, 3)) + 2 * (D(0, 7) - D(0, 5) - D(4, 5) + D(4, 7) - D(5, 0) - D(5, 4) + D(5, 8) + D(7, 0) + D(7, 4) -
			D(7, 8) + D(8, 5) - D(8, 7)), 4 * (-D(1, 2) + D(1, 6) - D(2, 1) - D(2, 3) - D(3, 2) + D(3, 6) + D(6, 1) +
			D(6, 3)) + 2 * (D(0, 7) - D(0, 5) + D(4, 5) - D(4, 7) - D(5, 0) + D(5, 4) - D(5, 8) + D(7, 0) - D(7, 4) +
			D(7, 8) - D(8, 5) + D(8, 7)), 2 * (D(2, 8) - D(0, 6) - D(2, 0) - D(2, 4) - D(0, 2) - D(4, 2) - D(4, 6) -
			D(6, 0) - D(6, 4) + D(6, 8) + D(8, 2) + D(8, 6)), 4 * (D(0, 5) + D(0, 7) - D(4, 5) - D(4, 7) + D(5, 0) -
			D(5, 4) - D(5, 8) + D(7, 0) - D(7, 4) - D(7, 8) - D(8, 5) - D(8, 7)) + 8 * (D(1, 2) + D(1, 6) + D(2, 1) +
			D(2, 3) + D(3, 2) + D(3, 6) + D(6, 1) + D(6, 3)), 4 * (D(0, 4) - D(0, 0) + D(4, 0) - D(4, 4) + D(8, 8)) + 8 *
			(D(1, 1) + D(1, 3) + D(3, 1) + D(3, 3)), 6 * (D(0, 2) + D(0, 6) + D(2, 0) - D(2, 4) - D(2, 8) - D(4, 2) -
			D(4, 6) + D(6, 0) - D(6, 4) - D(6, 8) - D(8, 2) - D(8, 6)), 4 * (D(0, 0) - D(0, 4) - D(0, 8) - D(4, 0) +
			D(4, 4) + D(4, 8) - D(8, 0) + D(8, 4) + D(8, 8));
		f2coeff << 2 * (-D(0, 2) + D(0, 6) - D(2, 0) - D(2, 4) - D(2, 8) - D(4, 2) + D(4, 6) + D(6, 0) + D(6, 4) + D(6, 8) -
			D(8, 2) + D(8, 6)), 4 * (D(0, 4) - D(0, 0) + D(4, 0) - D(4, 4) + D(8, 8)) + 8 * (D(1, 1) + D(1, 3) + D(3, 1) +
			D(3, 3)), 4 * (D(0, 7) - D(0, 5) + D(4, 5) - D(4, 7) - D(5, 0) + D(5, 4) - D(5, 8) + D(7, 0) - D(7, 4) +
			D(7, 8) - D(8, 5) + D(8, 7)) + 8 * (-D(1, 2) + D(1, 6) - D(2, 1) - D(2, 3) - D(3, 2) + D(3, 6) + D(6, 1) +
			D(6, 3)), 8 * (D(1, 1) - D(2, 2) - D(3, 3) + D(5, 5) + D(6, 6) - D(7, 7)), 4 * (D(0, 3) - D(0, 1) - D(1, 0) +
			D(1, 4) - D(1, 8) + D(3, 0) - D(3, 4) + D(3, 8) + D(4, 1) - D(4, 3) - D(8, 1) + D(8, 3)) + 8 * (-D(2, 5) -
			D(2, 7) - D(5, 2) + D(5, 6) + D(6, 5) + D(6, 7) - D(7, 2) + D(7, 6)), 6 * (D(4, 5) - D(0, 7) - D(0, 5) +
			D(4, 7) - D(5, 0) + D(5, 4) - D(5, 8) - D(7, 0) + D(7, 4) - D(7, 8) - D(8, 5) - D(8, 7)), 4 * (D(0, 0) -
			D(0, 4) + D(0, 8) - D(4, 0) + D(4, 4) - D(4, 8) + D(8, 0) - D(8, 4) + D(8, 8)), 4 * (D(2, 5) + D(2, 7) +
			D(5, 2) + D(5, 6) + D(6, 5) + D(6, 7) + D(7, 2) + D(7, 6)) + 2 * (D(1, 8) - D(0, 3) - D(1, 0) - D(1, 4) -
			D(0, 1) - D(3, 0) - D(3, 4) + D(3, 8) - D(4, 1) - D(4, 3) + D(8, 1) + D(8, 3)), 4 * (-D(2, 5) + D(2, 7) -
			D(5, 2) + D(5, 6) + D(6, 5) - D(6, 7) + D(7, 2) - D(7, 6)) + 2 * (D(0, 1) + D(0, 3) + D(1, 0) + D(1, 4) +
			D(1, 8) + D(3, 0) + D(3, 4) + D(3, 8) + D(4, 1) + D(4, 3) + D(8, 1) + D(8, 3)), 4 * (-D(1, 2) + D(1, 6) -
			D(2, 1) + D(2, 3) + D(3, 2) - D(3, 6) + D(6, 1) - D(6, 3)) + 2 * (D(0, 5) + D(0, 7) + D(4, 5) + D(4, 7) +
			D(5, 0) + D(5, 4) + D(5, 8) + D(7, 0) + D(7, 4) + D(7, 8) + D(8, 5) + D(8, 7)), 4 * (-D(0, 8) - D(0, 0) +
			D(4, 4) - D(8, 0) - D(8, 8)) + 8 * (D(2, 2) - D(2, 6) - D(6, 2) + D(6, 6)), 4 * (D(0, 0) - D(4, 4) + D(4, 8) +
			D(8, 4) - D(8, 8)) + 8 * (D(5, 5) + D(5, 7) + D(7, 5) + D(7, 7)), 4 * (D(1, 5) - D(1, 7) + D(3, 5) - D(3, 7) +
			D(5, 1) + D(5, 3) - D(7, 1) - D(7, 3)) + 2 * (-D(0, 2) + D(0, 6) - D(2, 0) + D(2, 4) + D(2, 8) + D(4, 2) -
			D(4, 6) + D(6, 0) - D(6, 4) - D(6, 8) + D(8, 2) - D(8, 6)), 4 * (D(1, 5) + D(1, 7) - D(3, 5) - D(3, 7) +
			D(5, 1) - D(5, 3) + D(7, 1) - D(7, 3)) + 2 * (D(0, 2) - D(0, 6) + D(2, 0) + D(2, 4) - D(2, 8) + D(4, 2) -
			D(4, 6) - D(6, 0) - D(6, 4) + D(6, 8) - D(8, 2) + D(8, 6)), 6 * (D(0, 2) - D(0, 6) + D(2, 0) - D(2, 4) +
			D(2, 8) - D(4, 2) + D(4, 6) - D(6, 0) + D(6, 4) - D(6, 8) + D(8, 2) - D(8, 6)), 2 * (D(5, 8) - D(0, 7) -
			D(4, 5) - D(4, 7) - D(5, 0) - D(5, 4) - D(0, 5) - D(7, 0) - D(7, 4) + D(7, 8) + D(8, 5) + D(8, 7)), 4 *
			(-D(0, 6) - D(0, 2) - D(2, 0) + D(2, 4) - D(2, 8) + D(4, 2) + D(4, 6) - D(6, 0) + D(6, 4) - D(6, 8) -
			D(8, 2) - D(8, 6)) + 8 * (D(1, 5) + D(1, 7) + D(3, 5) + D(3, 7) + D(5, 1) + D(5, 3) + D(7, 1) + D(7, 3)), 6 *
			(D(1, 4) - D(0, 3) - D(1, 0) - D(0, 1) - D(1, 8) - D(3, 0) + D(3, 4) - D(3, 8) + D(4, 1) + D(4, 3) - D(8, 1) -
			D(8, 3)), 4 * (D(1, 2) + D(1, 6) + D(2, 1) + D(2, 3) + D(3, 2) + D(3, 6) + D(6, 1) + D(6, 3)) + 2 * (D(0, 5) +
			D(0, 7) - D(4, 5) - D(4, 7) + D(5, 0) - D(5, 4) - D(5, 8) + D(7, 0) - D(7, 4) - D(7, 8) - D(8, 5) -
			D(8, 7)), 2 * (D(0, 1) + D(0, 3) + D(1, 0) - D(1, 4) - D(1, 8) + D(3, 0) - D(3, 4) - D(3, 8) - D(4, 1) -
			D(4, 3) - D(8, 1) - D(8, 3));
		f3coeff << 2 * (D(0, 1) - D(0, 3) + D(1, 0) + D(1, 4) + D(1, 8) - D(3, 0) - D(3, 4) - D(3, 8) + D(4, 1) - D(4, 3) +
			D(8, 1) - D(8, 3)), 4 * (D(1, 2) + D(1, 6) + D(2, 1) + D(2, 3) + D(3, 2) + D(3, 6) + D(6, 1) + D(6, 3)) + 2 *
			(D(0, 5) + D(0, 7) - D(4, 5) - D(4, 7) + D(5, 0) - D(5, 4) - D(5, 8) + D(7, 0) - D(7, 4) - D(7, 8) - D(8, 5) -
			D(8, 7)), 8 * (D(1, 1) - D(2, 2) - D(3, 3) + D(5, 5) + D(6, 6) - D(7, 7)), 4 * (D(0, 7) - D(0, 5) - D(4, 5) +
			D(4, 7) - D(5, 0) - D(5, 4) + D(5, 8) + D(7, 0) + D(7, 4) - D(7, 8) + D(8, 5) - D(8, 7)) + 8 * (D(1, 2) +
			D(1, 6) + D(2, 1) - D(2, 3) - D(3, 2) - D(3, 6) + D(6, 1) - D(6, 3)), 4 * (D(0, 2) - D(0, 6) + D(2, 0) +
			D(2, 4) - D(2, 8) + D(4, 2) - D(4, 6) - D(6, 0) - D(6, 4) + D(6, 8) - D(8, 2) + D(8, 6)) + 8 * (D(1, 5) +
			D(1, 7) - D(3, 5) - D(3, 7) + D(5, 1) - D(5, 3) + D(7, 1) - D(7, 3)), 4 * (D(0, 0) - D(4, 4) + D(4, 8) +
			D(8, 4) - D(8, 8)) + 8 * (D(5, 5) + D(5, 7) + D(7, 5) + D(7, 7)), 2 * (D(4, 5) - D(0, 7) - D(0, 5) + D(4, 7) -
			D(5, 0) + D(5, 4) - D(5, 8) - D(7, 0) + D(7, 4) - D(7, 8) - D(8, 5) - D(8, 7)), 6 * (D(2, 8) - D(0, 6) -
			D(2, 0) - D(2, 4) - D(0, 2) - D(4, 2) - D(4, 6) - D(6, 0) - D(6, 4) + D(6, 8) + D(8, 2) + D(8, 6)), 4 *
			(D(1, 5) - D(1, 7) - D(3, 5) + D(3, 7) + D(5, 1) - D(5, 3) - D(7, 1) + D(7, 3)) + 2 * (D(0, 2) + D(0, 6) +
			D(2, 0) + D(2, 4) + D(2, 8) + D(4, 2) + D(4, 6) + D(6, 0) + D(6, 4) + D(6, 8) + D(8, 2) + D(8, 6)), 4 *
			(-D(0, 4) - D(0, 0) - D(4, 0) - D(4, 4) + D(8, 8)) + 8 * (D(1, 1) - D(1, 3) - D(3, 1) + D(3, 3)), 4 *
			(-D(1, 2) + D(1, 6) - D(2, 1) + D(2, 3) + D(3, 2) - D(3, 6) + D(6, 1) - D(6, 3)) + 2 * (D(0, 5) + D(0, 7) +
			D(4, 5) + D(4, 7) + D(5, 0) + D(5, 4) + D(5, 8) + D(7, 0) + D(7, 4) + D(7, 8) + D(8, 5) + D(8, 7)), 6 *
			(D(5, 8) - D(0, 7) - D(4, 5) - D(4, 7) - D(5, 0) - D(5, 4) - D(0, 5) - D(7, 0) - D(7, 4) + D(7, 8) + D(8, 5) +
			D(8, 7)), 4 * (D(2, 5) - D(2, 7) + D(5, 2) + D(5, 6) + D(6, 5) - D(6, 7) - D(7, 2) - D(7, 6)) + 2 * (D(0, 1) -
			D(0, 3) + D(1, 0) - D(1, 4) - D(1, 8) - D(3, 0) + D(3, 4) + D(3, 8) - D(4, 1) + D(4, 3) - D(8, 1) +
			D(8, 3)), 6 * (D(0, 3) - D(0, 1) - D(1, 0) - D(1, 4) + D(1, 8) + D(3, 0) + D(3, 4) - D(3, 8) - D(4, 1) +
			D(4, 3) + D(8, 1) - D(8, 3)), 4 * (-D(2, 5) - D(2, 7) - D(5, 2) + D(5, 6) + D(6, 5) + D(6, 7) - D(7, 2) +
			D(7, 6)) + 2 * (D(0, 3) - D(0, 1) - D(1, 0) + D(1, 4) - D(1, 8) + D(3, 0) - D(3, 4) + D(3, 8) + D(4, 1) -
			D(4, 3) - D(8, 1) + D(8, 3)), 4 * (D(0, 0) + D(0, 4) - D(0, 8) + D(4, 0) + D(4, 4) - D(4, 8) - D(8, 0) -
			D(8, 4) + D(8, 8)), 4 * (D(1, 8) - D(0, 3) - D(1, 0) - D(1, 4) - D(0, 1) - D(3, 0) - D(3, 4) + D(3, 8) -
			D(4, 1) - D(4, 3) + D(8, 1) + D(8, 3)) + 8 * (D(2, 5) + D(2, 7) + D(5, 2) + D(5, 6) + D(6, 5) + D(6, 7) +
			D(7, 2) + D(7, 6)), 4 * (D(1, 5) + D(1, 7) + D(3, 5) + D(3, 7) + D(5, 1) + D(5, 3) + D(7, 1) + D(7, 3)) + 2 *
			(-D(0, 6) - D(0, 2) - D(2, 0) + D(2, 4) - D(2, 8) + D(4, 2) + D(4, 6) - D(6, 0) + D(6, 4) - D(6, 8) -
			D(8, 2) - D(8, 6)), 4 * (D(0, 8) - D(0, 0) + D(4, 4) + D(8, 0) - D(8, 8)) + 8 * (D(2, 2) + D(2, 6) + D(6, 2) +
			D(6, 6)), 2 * (D(0, 2) + D(0, 6) + D(2, 0) - D(2, 4) - D(2, 8) - D(4, 2) - D(4, 6) + D(6, 0) - D(6, 4) -
			D(6, 8) - D(8, 2) - D(8, 6));
		// assemble the coeff vectors

		Eigen::Vector4d u = ((Eigen::Vector4d::Random() + Eigen::Vector4d::Random() +
			Eigen::Vector4d::Random() + Eigen::Vector4d::Random()) * 100);//.array().round().matrix(); // needs Eigen 3.3
		for(int i = 0; i < 4; ++ i)
			u(i) = floor(u(i) + .5);
		// random approx normally distributed vector

		Eigen::MatrixXd M = t_Cayley(f1coeff, f2coeff, f3coeff, u);
		// build the Cayley matrix

		Eigen::Matrix<double, 27, 27> Schur_M;
		{
			Eigen::PartialPivLU<Eigen::Matrix<double, 93, 93> > LU_D(M.bottomRightCorner<93, 93>());
			Schur_M = M.topLeftCorner<27, 27>() - M.topRightCorner<27, 93>() *
				LU_D.solve(M.bottomLeftCorner<93, 27>());
		}
		// compute Schur complement (74.8% dense, likely a lost cause using sparse matrices)

		Eigen::/*SelfAdjoint*/EigenSolver<Eigen::Matrix<double, 27, 27> > eig(Schur_M, true); // Schur_M not symmetric (so no self-adjoint)!
		const typename Eigen::EigenSolver<Eigen::Matrix<double, 27, 27> >::EigenvectorsType
			&V = eig.eigenvectors();
		// compute Eigen-values + vectors

		for(int i = 0; i < 27; ++ i) {
			std::complex<double> temp = V(1, i) / V(0, i);
			if(temp.imag() == 0) {
				Eigen::Matrix<std::complex<double>, 3, 1> cu;
				cu << V(9, i) / V(0, i), V(3, i) / V(0, i), temp;
				Eigen::Vector3d u = cu.real(); // should be mostly real

				Eigen::Matrix3d H = Hessian(f1coeff, f2coeff, f3coeff, u); // is this thing real? yes, also symmetric
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig1(H, false); // symmetric matrices (-> self-adjoint), not complex
				if(eig1.info() == Eigen::Success &&
				   (eig1.eigenvalues().array() > .0).all()) { // all eigenvalues must be positive
					Eigen::Matrix3d R0 = t_Cayley_to_R(u);
					Eigen::Matrix3d R = R0 * (1 / (1 + u.transpose() * u));
					// recover rotation

					Eigen::Vector3d b2 = Eigen::Vector3d::Zero();
					for(size_t j = 0; j < n; ++ j) {
						b2 += (r_t_point_directions.col(j) * r_t_point_directions.col(j).transpose() -
							Eigen::Matrix3d::Identity()) * R * r_t_point_positions.col(j);
					}
					Eigen::Vector3d t = LU_H_inv.solve(b2); // reuse decomposition calculated at the beginning (it is just negative)
					// recover translation

					bool b_points_behind_camera = ((R.bottomRows<1>() * r_t_point_positions).array() < -t(2)).any();
#ifdef _DEBUG
					bool b_points_behind_camera_dbg = false;
					for(size_t j = 0; j < n; ++ j) {
						if((R.bottomRows<1>() * r_t_point_positions.col(j) + t(2)) < 0) {
							b_points_behind_camera_dbg = true;
							break;
						}
					}
					_ASSERTE(b_points_behind_camera_dbg == b_points_behind_camera);
#endif // _DEBUG
					// check if the solution is valid

					if(!b_points_behind_camera) {
						Eigen::Matrix<double, 3, 4, Eigen::DontAlign> solution;
						solution.topLeftCorner<3, 3>() = R/*.transpose()*/;
						solution.rightCols<1>() = /*-(R.transpose() **/ t/*)*/; // return inverse pose, same as the above P3P // not since 2017-07-04
						r_solutions.push_back(solution);

						if(p_costs) {
							R0.transposeInPlace();
							const Eigen::Map<const Eigen::Matrix<double, 9, 1>,
								Eigen::AutoAlign> R0_vec(&R0(0, 0), 9, 1); // view 3x3 column-major data as a 9x1 vector
							double f_cost = (R0_vec.transpose() * D * R0_vec).norm();
							p_costs->push_back(f_cost);
							// compute cost (it should be a sum of squared residuals)

							/*double f_err_sum = 0;
							double f_rep_err_sum = 0;
							double f_rep_errB_sum = 0;
							for(size_t j = 0; j < n; ++ j) {
								_TyConstDirCol &v_dir_i = r_t_point_directions.col(j);
								_TyConstPosCol &v_pos_i = r_t_point_positions.col(j);

								Eigen::Vector3d v_dir_r = solution.topLeftCorner<3, 3>() * v_pos_i + solution.rightCols<1>();
								//Eigen::Vector3d v_dir_r = (R * v_pos_i + t).normalized();
								f_err_sum += (v_dir_r - v_dir_i).squaredNorm();
								v_dir_r /= v_dir_r(2);
								f_rep_err_sum += (v_dir_r - v_dir_i / v_dir_i(2)).squaredNorm();
								f_rep_errB_sum += (v_dir_r.template head<2>() - v_dir_i.template head<2>() / v_dir_i(2)).squaredNorm();
							}
							f_rep_err_sum = sqrt(f_rep_err_sum);
							f_rep_errB_sum = sqrt(f_rep_errB_sum);
							f_err_sum = sqrt(f_err_sum);
							++ f_err_sum;*/
							// todo - this may be flawed; the sum of errors is generally different from the cost computed above; see how it works in matlab
						}
					}
					// ...
				}
			}
		}
		// gather all algebraic solutions,
		// produce the solutions vector, possibly prune solutions that project points behind the camera

		_ASSERTE(!p_costs || p_costs->size() == r_solutions.size());
		// should be the same size

		return true;
	}

protected:
	static inline Eigen::Matrix3d t_SkewSym(const Eigen::Vector3d &v)
	{
		Eigen::Matrix3d S;
		S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
		return S;
	}

	static inline Eigen::Matrix3d t_Cayley_to_R(const Eigen::Vector3d &s)
	{
		return ((1 - s.squaredNorm()) * Eigen::Matrix3d::Identity() +
			2 * t_SkewSym(s) + 2 * (s * s.transpose())).transpose();
	}

	static Eigen::MatrixXd t_Cayley(const Eigen::Matrix<double, 20, 1> &a,
		const Eigen::Matrix<double, 20, 1> &b,
		const Eigen::Matrix<double, 20, 1> &c, const Eigen::Vector4d &u) // throw(std::bad_alloc) // Construct the Macaulay resultant matrix
	{
#ifdef _DEBUG
		Eigen::MatrixXd M(120, 120);
		M.setZero(); // !!
		M(0, 0) = u(0); M(0, 35) = a(0); M(0, 83) = b(0); M(0, 118) = c(0); M(1, 0) = u(3); M(1, 1) = u(0); M(1, 34) = a(0);
		M(1, 35) = a(9); M(1, 54) = b(0); M(1, 83) = b(9); M(1, 99) = c(0); M(1, 118) = c(9); M(2, 1) = u(3); M(2, 2) = u(0);
		M(2, 34) = a(9); M(2, 35) = a(13); M(2, 51) = a(0); M(2, 54) = b(9); M(2, 65) = b(0); M(2, 83) = b(13); M(2, 89) = c(0);
		M(2, 99) = c(9); M(2, 118) = c(13); M(3, 0) = u(2); M(3, 3) = u(0); M(3, 35) = a(10); M(3, 49) = a(0); M(3, 76) = b(0);
		M(3, 83) = b(10); M(3, 118) = c(10); M(3, 119) = c(0); M(4, 1) = u(2); M(4, 3) = u(3); M(4, 4) = u(0); M(4, 34) = a(10);
		M(4, 35) = a(4); M(4, 43) = a(0); M(4, 49) = a(9); M(4, 54) = b(10); M(4, 71) = b(0); M(4, 76) = b(9); M(4, 83) = b(4);
		M(4, 99) = c(10); M(4, 100) = c(0); M(4, 118) = c(4); M(4, 119) = c(9); M(5, 2) = u(2); M(5, 4) = u(3); M(5, 5) = u(0);
		M(5, 34) = a(4); M(5, 35) = a(11); M(5, 41) = a(0); M(5, 43) = a(9); M(5, 49) = a(13); M(5, 51) = a(10);
		M(5, 54) = b(4); M(5, 62) = b(0); M(5, 65) = b(10); M(5, 71) = b(9); M(5, 76) = b(13); M(5, 83) = b(11);
		M(5, 89) = c(10); M(5, 99) = c(4); M(5, 100) = c(9); M(5, 111) = c(0); M(5, 118) = c(11); M(5, 119) = c(13);
		M(6, 3) = u(2); M(6, 6) = u(0); M(6, 30) = a(0); M(6, 35) = a(14); M(6, 49) = a(10); M(6, 75) = b(0); M(6, 76) = b(10);
		M(6, 83) = b(14); M(6, 107) = c(0); M(6, 118) = c(14); M(6, 119) = c(10); M(7, 4) = u(2); M(7, 6) = u(3);
		M(7, 7) = u(0); M(7, 30) = a(9); M(7, 34) = a(14); M(7, 35) = a(5); M(7, 43) = a(10); M(7, 45) = a(0); M(7, 49) = a(4);
		M(7, 54) = b(14); M(7, 63) = b(0); M(7, 71) = b(10); M(7, 75) = b(9); M(7, 76) = b(4); M(7, 83) = b(5);
		M(7, 99) = c(14); M(7, 100) = c(10); M(7, 107) = c(9); M(7, 112) = c(0); M(7, 118) = c(5); M(7, 119) = c(4);
		M(8, 5) = u(2); M(8, 7) = u(3); M(8, 8) = u(0); M(8, 30) = a(13); M(8, 34) = a(5); M(8, 41) = a(10); M(8, 43) = a(4);
		M(8, 45) = a(9); M(8, 46) = a(0); M(8, 49) = a(11); M(8, 51) = a(14); M(8, 54) = b(5); M(8, 62) = b(10);
		M(8, 63) = b(9); M(8, 65) = b(14); M(8, 66) = b(0); M(8, 71) = b(4); M(8, 75) = b(13); M(8, 76) = b(11);
		M(8, 89) = c(14); M(8, 99) = c(5); M(8, 100) = c(4); M(8, 102) = c(0); M(8, 107) = c(13); M(8, 111) = c(10);
		M(8, 112) = c(9); M(8, 119) = c(11); M(9, 0) = u(1); M(9, 9) = u(0); M(9, 35) = a(8); M(9, 36) = a(0); M(9, 83) = b(8);
		M(9, 84) = b(0); M(9, 88) = c(0); M(9, 118) = c(8); M(10, 1) = u(1); M(10, 9) = u(3); M(10, 10) = u(0);
		M(10, 33) = a(0); M(10, 34) = a(8); M(10, 35) = a(3); M(10, 36) = a(9); M(10, 54) = b(8); M(10, 59) = b(0);
		M(10, 83) = b(3); M(10, 84) = b(9); M(10, 88) = c(9); M(10, 99) = c(8); M(10, 117) = c(0); M(10, 118) = c(3);
		M(11, 2) = u(1); M(11, 10) = u(3); M(11, 11) = u(0); M(11, 28) = a(0); M(11, 33) = a(9); M(11, 34) = a(3);
		M(11, 35) = a(7); M(11, 36) = a(13); M(11, 51) = a(8); M(11, 54) = b(3); M(11, 57) = b(0); M(11, 59) = b(9);
		M(11, 65) = b(8); M(11, 83) = b(7); M(11, 84) = b(13); M(11, 88) = c(13); M(11, 89) = c(8); M(11, 99) = c(3);
		M(11, 114) = c(0); M(11, 117) = c(9); M(11, 118) = c(7); M(12, 3) = u(1); M(12, 9) = u(2); M(12, 12) = u(0);
		M(12, 35) = a(2); M(12, 36) = a(10); M(12, 39) = a(0); M(12, 49) = a(8); M(12, 76) = b(8); M(12, 79) = b(0);
		M(12, 83) = b(2); M(12, 84) = b(10); M(12, 88) = c(10); M(12, 96) = c(0); M(12, 118) = c(2); M(12, 119) = c(8);
		M(13, 4) = u(1); M(13, 10) = u(2); M(13, 12) = u(3); M(13, 13) = u(0); M(13, 33) = a(10); M(13, 34) = a(2);
		M(13, 35) = a(16); M(13, 36) = a(4); M(13, 39) = a(9); M(13, 43) = a(8); M(13, 47) = a(0); M(13, 49) = a(3);
		M(13, 54) = b(2); M(13, 59) = b(10); M(13, 60) = b(0); M(13, 71) = b(8); M(13, 76) = b(3); M(13, 79) = b(9);
		M(13, 83) = b(16); M(13, 84) = b(4); M(13, 88) = c(4); M(13, 90) = c(0); M(13, 96) = c(9); M(13, 99) = c(2);
		M(13, 100) = c(8); M(13, 117) = c(10); M(13, 118) = c(16); M(13, 119) = c(3); M(14, 5) = u(1); M(14, 11) = u(2);
		M(14, 13) = u(3); M(14, 14) = u(0); M(14, 28) = a(10); M(14, 33) = a(4); M(14, 34) = a(16); M(14, 36) = a(11);
		M(14, 39) = a(13); M(14, 41) = a(8); M(14, 42) = a(0); M(14, 43) = a(3); M(14, 47) = a(9); M(14, 49) = a(7);
		M(14, 51) = a(2); M(14, 54) = b(16); M(14, 56) = b(0); M(14, 57) = b(10); M(14, 59) = b(4); M(14, 60) = b(9);
		M(14, 62) = b(8); M(14, 65) = b(2); M(14, 71) = b(3); M(14, 76) = b(7); M(14, 79) = b(13); M(14, 84) = b(11);
		M(14, 88) = c(11); M(14, 89) = c(2); M(14, 90) = c(9); M(14, 96) = c(13); M(14, 99) = c(16); M(14, 100) = c(3);
		M(14, 106) = c(0); M(14, 111) = c(8); M(14, 114) = c(10); M(14, 117) = c(4); M(14, 119) = c(7); M(15, 6) = u(1);
		M(15, 12) = u(2); M(15, 15) = u(0); M(15, 29) = a(0); M(15, 30) = a(8); M(15, 35) = a(17); M(15, 36) = a(14);
		M(15, 39) = a(10); M(15, 49) = a(2); M(15, 74) = b(0); M(15, 75) = b(8); M(15, 76) = b(2); M(15, 79) = b(10);
		M(15, 83) = b(17); M(15, 84) = b(14); M(15, 88) = c(14); M(15, 94) = c(0); M(15, 96) = c(10); M(15, 107) = c(8);
		M(15, 118) = c(17); M(15, 119) = c(2); M(16, 7) = u(1); M(16, 13) = u(2); M(16, 15) = u(3); M(16, 16) = u(0);
		M(16, 29) = a(9); M(16, 30) = a(3); M(16, 33) = a(14); M(16, 34) = a(17); M(16, 36) = a(5); M(16, 39) = a(4);
		M(16, 43) = a(2); M(16, 44) = a(0); M(16, 45) = a(8); M(16, 47) = a(10); M(16, 49) = a(16); M(16, 54) = b(17);
		M(16, 59) = b(14); M(16, 60) = b(10); M(16, 63) = b(8); M(16, 68) = b(0); M(16, 71) = b(2); M(16, 74) = b(9);
		M(16, 75) = b(3); M(16, 76) = b(16); M(16, 79) = b(4); M(16, 84) = b(5); M(16, 88) = c(5); M(16, 90) = c(10);
		M(16, 94) = c(9); M(16, 96) = c(4); M(16, 97) = c(0); M(16, 99) = c(17); M(16, 100) = c(2); M(16, 107) = c(3);
		M(16, 112) = c(8); M(16, 117) = c(14); M(16, 119) = c(16); M(17, 8) = u(1); M(17, 14) = u(2); M(17, 16) = u(3);
		M(17, 17) = u(0); M(17, 28) = a(14); M(17, 29) = a(13); M(17, 30) = a(7); M(17, 33) = a(5); M(17, 39) = a(11);
		M(17, 41) = a(2); M(17, 42) = a(10); M(17, 43) = a(16); M(17, 44) = a(9); M(17, 45) = a(3); M(17, 46) = a(8);
		M(17, 47) = a(4); M(17, 51) = a(17); M(17, 56) = b(10); M(17, 57) = b(14); M(17, 59) = b(5); M(17, 60) = b(4);
		M(17, 62) = b(2); M(17, 63) = b(3); M(17, 65) = b(17); M(17, 66) = b(8); M(17, 68) = b(9); M(17, 71) = b(16);
		M(17, 74) = b(13); M(17, 75) = b(7); M(17, 79) = b(11); M(17, 89) = c(17); M(17, 90) = c(4); M(17, 94) = c(13);
		M(17, 96) = c(11); M(17, 97) = c(9); M(17, 100) = c(16); M(17, 102) = c(8); M(17, 106) = c(10); M(17, 107) = c(7);
		M(17, 111) = c(2); M(17, 112) = c(3); M(17, 114) = c(14); M(17, 117) = c(5); M(18, 9) = u(1); M(18, 18) = u(0);
		M(18, 35) = a(12); M(18, 36) = a(8); M(18, 53) = a(0); M(18, 82) = b(0); M(18, 83) = b(12); M(18, 84) = b(8);
		M(18, 87) = c(0); M(18, 88) = c(8); M(18, 118) = c(12); M(19, 10) = u(1); M(19, 18) = u(3); M(19, 19) = u(0);
		M(19, 32) = a(0); M(19, 33) = a(8); M(19, 34) = a(12); M(19, 35) = a(18); M(19, 36) = a(3); M(19, 53) = a(9);
		M(19, 54) = b(12); M(19, 59) = b(8); M(19, 61) = b(0); M(19, 82) = b(9); M(19, 83) = b(18); M(19, 84) = b(3);
		M(19, 87) = c(9); M(19, 88) = c(3); M(19, 99) = c(12); M(19, 116) = c(0); M(19, 117) = c(8); M(19, 118) = c(18);
		M(20, 11) = u(1); M(20, 19) = u(3); M(20, 20) = u(0); M(20, 27) = a(0); M(20, 28) = a(8); M(20, 32) = a(9);
		M(20, 33) = a(3); M(20, 34) = a(18); M(20, 36) = a(7); M(20, 51) = a(12); M(20, 53) = a(13); M(20, 54) = b(18);
		M(20, 55) = b(0); M(20, 57) = b(8); M(20, 59) = b(3); M(20, 61) = b(9); M(20, 65) = b(12); M(20, 82) = b(13);
		M(20, 84) = b(7); M(20, 87) = c(13); M(20, 88) = c(7); M(20, 89) = c(12); M(20, 99) = c(18); M(20, 113) = c(0);
		M(20, 114) = c(8); M(20, 116) = c(9); M(20, 117) = c(3); M(21, 12) = u(1); M(21, 18) = u(2); M(21, 21) = u(0);
		M(21, 35) = a(1); M(21, 36) = a(2); M(21, 38) = a(0); M(21, 39) = a(8); M(21, 49) = a(12); M(21, 53) = a(10);
		M(21, 76) = b(12); M(21, 78) = b(0); M(21, 79) = b(8); M(21, 82) = b(10); M(21, 83) = b(1); M(21, 84) = b(2);
		M(21, 87) = c(10); M(21, 88) = c(2); M(21, 92) = c(0); M(21, 96) = c(8); M(21, 118) = c(1); M(21, 119) = c(12);
		M(22, 13) = u(1); M(22, 19) = u(2); M(22, 21) = u(3); M(22, 22) = u(0); M(22, 32) = a(10); M(22, 33) = a(2);
		M(22, 34) = a(1); M(22, 36) = a(16); M(22, 38) = a(9); M(22, 39) = a(3); M(22, 40) = a(0); M(22, 43) = a(12);
		M(22, 47) = a(8); M(22, 49) = a(18); M(22, 53) = a(4); M(22, 54) = b(1); M(22, 59) = b(2); M(22, 60) = b(8);
		M(22, 61) = b(10); M(22, 71) = b(12); M(22, 72) = b(0); M(22, 76) = b(18); M(22, 78) = b(9); M(22, 79) = b(3);
		M(22, 82) = b(4); M(22, 84) = b(16); M(22, 87) = c(4); M(22, 88) = c(16); M(22, 90) = c(8); M(22, 92) = c(9);
		M(22, 95) = c(0); M(22, 96) = c(3); M(22, 99) = c(1); M(22, 100) = c(12); M(22, 116) = c(10); M(22, 117) = c(2);
		M(22, 119) = c(18); M(23, 14) = u(1); M(23, 20) = u(2); M(23, 22) = u(3); M(23, 23) = u(0); M(23, 27) = a(10);
		M(23, 28) = a(2); M(23, 32) = a(4); M(23, 33) = a(16); M(23, 38) = a(13); M(23, 39) = a(7); M(23, 40) = a(9);
		M(23, 41) = a(12); M(23, 42) = a(8); M(23, 43) = a(18); M(23, 47) = a(3); M(23, 51) = a(1); M(23, 53) = a(11);
		M(23, 55) = b(10); M(23, 56) = b(8); M(23, 57) = b(2); M(23, 59) = b(16); M(23, 60) = b(3); M(23, 61) = b(4);
		M(23, 62) = b(12); M(23, 65) = b(1); M(23, 71) = b(18); M(23, 72) = b(9); M(23, 78) = b(13); M(23, 79) = b(7);
		M(23, 82) = b(11); M(23, 87) = c(11); M(23, 89) = c(1); M(23, 90) = c(3); M(23, 92) = c(13); M(23, 95) = c(9);
		M(23, 96) = c(7); M(23, 100) = c(18); M(23, 106) = c(8); M(23, 111) = c(12); M(23, 113) = c(10); M(23, 114) = c(2);
		M(23, 116) = c(4); M(23, 117) = c(16); M(24, 15) = u(1); M(24, 21) = u(2); M(24, 24) = u(0); M(24, 29) = a(8);
		M(24, 30) = a(12); M(24, 36) = a(17); M(24, 38) = a(10); M(24, 39) = a(2); M(24, 49) = a(1); M(24, 52) = a(0);
		M(24, 53) = a(14); M(24, 73) = b(0); M(24, 74) = b(8); M(24, 75) = b(12); M(24, 76) = b(1); M(24, 78) = b(10);
		M(24, 79) = b(2); M(24, 82) = b(14); M(24, 84) = b(17); M(24, 87) = c(14); M(24, 88) = c(17); M(24, 92) = c(10);
		M(24, 93) = c(0); M(24, 94) = c(8); M(24, 96) = c(2); M(24, 107) = c(12); M(24, 119) = c(1); M(25, 16) = u(1);
		M(25, 22) = u(2); M(25, 24) = u(3); M(25, 25) = u(0); M(25, 29) = a(3); M(25, 30) = a(18); M(25, 32) = a(14);
		M(25, 33) = a(17); M(25, 38) = a(4); M(25, 39) = a(16); M(25, 40) = a(10); M(25, 43) = a(1); M(25, 44) = a(8);
		M(25, 45) = a(12); M(25, 47) = a(2); M(25, 52) = a(9); M(25, 53) = a(5); M(25, 59) = b(17); M(25, 60) = b(2);
		M(25, 61) = b(14); M(25, 63) = b(12); M(25, 68) = b(8); M(25, 71) = b(1); M(25, 72) = b(10); M(25, 73) = b(9);
		M(25, 74) = b(3); M(25, 75) = b(18); M(25, 78) = b(4); M(25, 79) = b(16); M(25, 82) = b(5); M(25, 87) = c(5);
		M(25, 90) = c(2); M(25, 92) = c(4); M(25, 93) = c(9); M(25, 94) = c(3); M(25, 95) = c(10); M(25, 96) = c(16);
		M(25, 97) = c(8); M(25, 100) = c(1); M(25, 107) = c(18); M(25, 112) = c(12); M(25, 116) = c(14); M(25, 117) = c(17);
		M(26, 17) = u(1); M(26, 23) = u(2); M(26, 25) = u(3); M(26, 26) = u(0); M(26, 27) = a(14); M(26, 28) = a(17);
		M(26, 29) = a(7); M(26, 32) = a(5); M(26, 38) = a(11); M(26, 40) = a(4); M(26, 41) = a(1); M(26, 42) = a(2);
		M(26, 44) = a(3); M(26, 45) = a(18); M(26, 46) = a(12); M(26, 47) = a(16); M(26, 52) = a(13); M(26, 55) = b(14);
		M(26, 56) = b(2); M(26, 57) = b(17); M(26, 60) = b(16); M(26, 61) = b(5); M(26, 62) = b(1); M(26, 63) = b(18);
		M(26, 66) = b(12); M(26, 68) = b(3); M(26, 72) = b(4); M(26, 73) = b(13); M(26, 74) = b(7); M(26, 78) = b(11);
		M(26, 90) = c(16); M(26, 92) = c(11); M(26, 93) = c(13); M(26, 94) = c(7); M(26, 95) = c(4); M(26, 97) = c(3);
		M(26, 102) = c(12); M(26, 106) = c(2); M(26, 111) = c(1); M(26, 112) = c(18); M(26, 113) = c(14); M(26, 114) = c(17);
		M(26, 116) = c(5); M(27, 15) = u(2); M(27, 29) = a(10); M(27, 30) = a(2); M(27, 36) = a(6); M(27, 39) = a(14);
		M(27, 49) = a(17); M(27, 69) = b(8); M(27, 70) = b(0); M(27, 74) = b(10); M(27, 75) = b(2); M(27, 76) = b(17);
		M(27, 79) = b(14); M(27, 84) = b(6); M(27, 88) = c(6); M(27, 91) = c(0); M(27, 94) = c(10); M(27, 96) = c(14);
		M(27, 107) = c(2); M(27, 110) = c(8); M(27, 119) = c(17); M(28, 6) = u(2); M(28, 30) = a(10); M(28, 35) = a(6);
		M(28, 49) = a(14); M(28, 69) = b(0); M(28, 75) = b(10); M(28, 76) = b(14); M(28, 83) = b(6); M(28, 107) = c(10);
		M(28, 110) = c(0); M(28, 118) = c(6); M(28, 119) = c(14); M(29, 24) = u(2); M(29, 29) = a(2); M(29, 30) = a(1);
		M(29, 38) = a(14); M(29, 39) = a(17); M(29, 52) = a(10); M(29, 53) = a(6); M(29, 69) = b(12); M(29, 70) = b(8);
		M(29, 73) = b(10); M(29, 74) = b(2); M(29, 75) = b(1); M(29, 78) = b(14); M(29, 79) = b(17); M(29, 82) = b(6);
		M(29, 87) = c(6); M(29, 91) = c(8); M(29, 92) = c(14); M(29, 93) = c(10); M(29, 94) = c(2); M(29, 96) = c(17);
		M(29, 107) = c(1); M(29, 110) = c(12); M(30, 37) = a(17); M(30, 48) = a(6); M(30, 52) = a(1); M(30, 70) = b(19);
		M(30, 73) = b(1); M(30, 77) = b(17); M(30, 81) = b(6); M(30, 85) = c(6); M(30, 91) = c(19); M(30, 93) = c(1);
		M(30, 98) = c(17); M(31, 29) = a(1); M(31, 37) = a(14); M(31, 38) = a(17); M(31, 50) = a(6); M(31, 52) = a(2);
		M(31, 69) = b(19); M(31, 70) = b(12); M(31, 73) = b(2); M(31, 74) = b(1); M(31, 77) = b(14); M(31, 78) = b(17);
		M(31, 80) = b(6); M(31, 86) = c(6); M(31, 91) = c(12); M(31, 92) = c(17); M(31, 93) = c(2); M(31, 94) = c(1);
		M(31, 98) = c(14); M(31, 110) = c(19); M(32, 48) = a(8); M(32, 50) = a(12); M(32, 53) = a(19); M(32, 80) = b(12);
		M(32, 81) = b(8); M(32, 82) = b(19); M(32, 85) = c(8); M(32, 86) = c(12); M(32, 87) = c(19); M(33, 29) = a(14);
		M(33, 30) = a(17); M(33, 39) = a(6); M(33, 64) = b(8); M(33, 69) = b(2); M(33, 70) = b(10); M(33, 74) = b(14);
		M(33, 75) = b(17); M(33, 79) = b(6); M(33, 91) = c(10); M(33, 94) = c(14); M(33, 96) = c(6); M(33, 103) = c(8);
		M(33, 107) = c(17); M(33, 110) = c(2); M(34, 29) = a(17); M(34, 38) = a(6); M(34, 52) = a(14); M(34, 64) = b(12);
		M(34, 69) = b(1); M(34, 70) = b(2); M(34, 73) = b(14); M(34, 74) = b(17); M(34, 78) = b(6); M(34, 91) = c(2);
		M(34, 92) = c(6); M(34, 93) = c(14); M(34, 94) = c(17); M(34, 103) = c(12); M(34, 110) = c(1); M(35, 37) = a(6);
		M(35, 52) = a(17); M(35, 64) = b(19); M(35, 70) = b(1); M(35, 73) = b(17); M(35, 77) = b(6); M(35, 91) = c(1);
		M(35, 93) = c(17); M(35, 98) = c(6); M(35, 103) = c(19); M(36, 5) = u(3); M(36, 34) = a(11); M(36, 41) = a(9);
		M(36, 43) = a(13); M(36, 49) = a(15); M(36, 51) = a(4); M(36, 54) = b(11); M(36, 62) = b(9); M(36, 65) = b(4);
		M(36, 71) = b(13); M(36, 76) = b(15); M(36, 89) = c(4); M(36, 99) = c(11); M(36, 100) = c(13); M(36, 101) = c(0);
		M(36, 109) = c(10); M(36, 111) = c(9); M(36, 119) = c(15); M(37, 2) = u(3); M(37, 34) = a(13); M(37, 35) = a(15);
		M(37, 51) = a(9); M(37, 54) = b(13); M(37, 65) = b(9); M(37, 83) = b(15); M(37, 89) = c(9); M(37, 99) = c(13);
		M(37, 109) = c(0); M(37, 118) = c(15); M(38, 30) = a(14); M(38, 49) = a(6); M(38, 64) = b(0); M(38, 69) = b(10);
		M(38, 75) = b(14); M(38, 76) = b(6); M(38, 103) = c(0); M(38, 107) = c(14); M(38, 110) = c(10); M(38, 119) = c(6);
		M(39, 28) = a(13); M(39, 33) = a(15); M(39, 51) = a(7); M(39, 57) = b(13); M(39, 59) = b(15); M(39, 65) = b(7);
		M(39, 89) = c(7); M(39, 105) = c(8); M(39, 108) = c(9); M(39, 109) = c(3); M(39, 114) = c(13); M(39, 117) = c(15);
		M(40, 27) = a(13); M(40, 28) = a(7); M(40, 32) = a(15); M(40, 55) = b(13); M(40, 57) = b(7); M(40, 61) = b(15);
		M(40, 105) = c(12); M(40, 108) = c(3); M(40, 109) = c(18); M(40, 113) = c(13); M(40, 114) = c(7); M(40, 116) = c(15);
		M(41, 30) = a(6); M(41, 64) = b(10); M(41, 69) = b(14); M(41, 75) = b(6); M(41, 103) = c(10); M(41, 107) = c(6);
		M(41, 110) = c(14); M(42, 27) = a(7); M(42, 31) = a(15); M(42, 55) = b(7); M(42, 58) = b(15); M(42, 105) = c(19);
		M(42, 108) = c(18); M(42, 113) = c(7); M(42, 115) = c(15); M(43, 29) = a(6); M(43, 64) = b(2); M(43, 69) = b(17);
		M(43, 70) = b(14); M(43, 74) = b(6); M(43, 91) = c(14); M(43, 94) = c(6); M(43, 103) = c(2); M(43, 110) = c(17);
		M(44, 28) = a(15); M(44, 57) = b(15); M(44, 105) = c(3); M(44, 108) = c(13); M(44, 109) = c(7); M(44, 114) = c(15);
		M(45, 27) = a(15); M(45, 55) = b(15); M(45, 105) = c(18); M(45, 108) = c(7); M(45, 113) = c(15); M(46, 52) = a(6);
		M(46, 64) = b(1); M(46, 70) = b(17); M(46, 73) = b(6); M(46, 91) = c(17); M(46, 93) = c(6); M(46, 103) = c(1);
		M(47, 40) = a(6); M(47, 44) = a(17); M(47, 52) = a(5); M(47, 64) = b(18); M(47, 67) = b(1); M(47, 68) = b(17);
		M(47, 70) = b(16); M(47, 72) = b(6); M(47, 73) = b(5); M(47, 91) = c(16); M(47, 93) = c(5); M(47, 95) = c(6);
		M(47, 97) = c(17); M(47, 103) = c(18); M(47, 104) = c(1); M(48, 30) = a(5); M(48, 43) = a(6); M(48, 45) = a(14);
		M(48, 63) = b(14); M(48, 64) = b(9); M(48, 67) = b(10); M(48, 69) = b(4); M(48, 71) = b(6); M(48, 75) = b(5);
		M(48, 100) = c(6); M(48, 103) = c(9); M(48, 104) = c(10); M(48, 107) = c(5); M(48, 110) = c(4); M(48, 112) = c(14);
		M(49, 41) = a(11); M(49, 45) = a(15); M(49, 46) = a(13); M(49, 62) = b(11); M(49, 63) = b(15); M(49, 66) = b(13);
		M(49, 101) = c(4); M(49, 102) = c(13); M(49, 105) = c(14); M(49, 109) = c(5); M(49, 111) = c(11); M(49, 112) = c(15);
		M(50, 41) = a(15); M(50, 62) = b(15); M(50, 101) = c(13); M(50, 105) = c(4); M(50, 109) = c(11); M(50, 111) = c(15);
		M(51, 64) = b(17); M(51, 70) = b(6); M(51, 91) = c(6); M(51, 103) = c(17); M(52, 41) = a(5); M(52, 45) = a(11);
		M(52, 46) = a(4); M(52, 62) = b(5); M(52, 63) = b(11); M(52, 66) = b(4); M(52, 67) = b(13); M(52, 69) = b(15);
		M(52, 101) = c(14); M(52, 102) = c(4); M(52, 104) = c(13); M(52, 109) = c(6); M(52, 110) = c(15); M(52, 111) = c(5);
		M(52, 112) = c(11); M(53, 64) = b(14); M(53, 69) = b(6); M(53, 103) = c(14); M(53, 110) = c(6); M(54, 105) = c(13);
		M(54, 109) = c(15); M(55, 44) = a(6); M(55, 64) = b(16); M(55, 67) = b(17); M(55, 68) = b(6); M(55, 70) = b(5);
		M(55, 91) = c(5); M(55, 97) = c(6); M(55, 103) = c(16); M(55, 104) = c(17); M(56, 105) = c(7); M(56, 108) = c(15);
		M(57, 64) = b(5); M(57, 67) = b(6); M(57, 103) = c(5); M(57, 104) = c(6); M(58, 46) = a(6); M(58, 64) = b(11);
		M(58, 66) = b(6); M(58, 67) = b(5); M(58, 102) = c(6); M(58, 103) = c(11); M(58, 104) = c(5); M(59, 8) = u(3);
		M(59, 30) = a(15); M(59, 41) = a(4); M(59, 43) = a(11); M(59, 45) = a(13); M(59, 46) = a(9); M(59, 51) = a(5);
		M(59, 62) = b(4); M(59, 63) = b(13); M(59, 65) = b(5); M(59, 66) = b(9); M(59, 71) = b(11); M(59, 75) = b(15);
		M(59, 89) = c(5); M(59, 100) = c(11); M(59, 101) = c(10); M(59, 102) = c(9); M(59, 107) = c(15); M(59, 109) = c(14);
		M(59, 111) = c(4); M(59, 112) = c(13); M(60, 8) = u(2); M(60, 30) = a(11); M(60, 41) = a(14); M(60, 43) = a(5);
		M(60, 45) = a(4); M(60, 46) = a(10); M(60, 51) = a(6); M(60, 62) = b(14); M(60, 63) = b(4); M(60, 65) = b(6);
		M(60, 66) = b(10); M(60, 67) = b(9); M(60, 69) = b(13); M(60, 71) = b(5); M(60, 75) = b(11); M(60, 89) = c(6);
		M(60, 100) = c(5); M(60, 102) = c(10); M(60, 104) = c(9); M(60, 107) = c(11); M(60, 110) = c(13); M(60, 111) = c(14);
		M(60, 112) = c(4); M(61, 42) = a(15); M(61, 56) = b(15); M(61, 101) = c(7); M(61, 105) = c(16); M(61, 106) = c(15);
		M(61, 108) = c(11); M(62, 64) = b(6); M(62, 103) = c(6); M(63, 105) = c(15); M(64, 46) = a(11); M(64, 66) = b(11);
		M(64, 67) = b(15); M(64, 101) = c(5); M(64, 102) = c(11); M(64, 104) = c(15); M(64, 105) = c(6); M(65, 46) = a(5);
		M(65, 64) = b(15); M(65, 66) = b(5); M(65, 67) = b(11); M(65, 101) = c(6); M(65, 102) = c(5); M(65, 103) = c(15);
		M(65, 104) = c(11); M(66, 46) = a(15); M(66, 66) = b(15); M(66, 101) = c(11); M(66, 102) = c(15); M(66, 105) = c(5);
		M(67, 101) = c(15); M(67, 105) = c(11); M(68, 41) = a(13); M(68, 43) = a(15); M(68, 51) = a(11); M(68, 62) = b(13);
		M(68, 65) = b(11); M(68, 71) = b(15); M(68, 89) = c(11); M(68, 100) = c(15); M(68, 101) = c(9); M(68, 105) = c(10);
		M(68, 109) = c(4); M(68, 111) = c(13); M(69, 37) = a(1); M(69, 48) = a(17); M(69, 52) = a(19); M(69, 73) = b(19);
		M(69, 77) = b(1); M(69, 81) = b(17); M(69, 85) = c(17); M(69, 93) = c(19); M(69, 98) = c(1); M(70, 20) = u(1);
		M(70, 27) = a(8); M(70, 28) = a(12); M(70, 31) = a(9); M(70, 32) = a(3); M(70, 33) = a(18); M(70, 50) = a(13);
		M(70, 51) = a(19); M(70, 53) = a(7); M(70, 55) = b(8); M(70, 57) = b(12); M(70, 58) = b(9); M(70, 59) = b(18);
		M(70, 61) = b(3); M(70, 65) = b(19); M(70, 80) = b(13); M(70, 82) = b(7); M(70, 86) = c(13); M(70, 87) = c(7);
		M(70, 89) = c(19); M(70, 113) = c(8); M(70, 114) = c(12); M(70, 115) = c(9); M(70, 116) = c(3); M(70, 117) = c(18);
		M(71, 45) = a(6); M(71, 63) = b(6); M(71, 64) = b(4); M(71, 67) = b(14); M(71, 69) = b(5); M(71, 103) = c(4);
		M(71, 104) = c(14); M(71, 110) = c(5); M(71, 112) = c(6); M(72, 41) = a(6); M(72, 45) = a(5); M(72, 46) = a(14);
		M(72, 62) = b(6); M(72, 63) = b(5); M(72, 64) = b(13); M(72, 66) = b(14); M(72, 67) = b(4); M(72, 69) = b(11);
		M(72, 102) = c(14); M(72, 103) = c(13); M(72, 104) = c(4); M(72, 110) = c(11); M(72, 111) = c(6); M(72, 112) = c(5);
		M(73, 48) = a(12); M(73, 50) = a(19); M(73, 80) = b(19); M(73, 81) = b(12); M(73, 85) = c(12); M(73, 86) = c(19);
		M(74, 25) = u(2); M(74, 29) = a(16); M(74, 32) = a(6); M(74, 38) = a(5); M(74, 40) = a(14); M(74, 44) = a(2);
		M(74, 45) = a(1); M(74, 47) = a(17); M(74, 52) = a(4); M(74, 60) = b(17); M(74, 61) = b(6); M(74, 63) = b(1);
		M(74, 67) = b(12); M(74, 68) = b(2); M(74, 69) = b(18); M(74, 70) = b(3); M(74, 72) = b(14); M(74, 73) = b(4);
		M(74, 74) = b(16); M(74, 78) = b(5); M(74, 90) = c(17); M(74, 91) = c(3); M(74, 92) = c(5); M(74, 93) = c(4);
		M(74, 94) = c(16); M(74, 95) = c(14); M(74, 97) = c(2); M(74, 104) = c(12); M(74, 110) = c(18); M(74, 112) = c(1);
		M(74, 116) = c(6); M(75, 21) = u(1); M(75, 36) = a(1); M(75, 37) = a(0); M(75, 38) = a(8); M(75, 39) = a(12);
		M(75, 49) = a(19); M(75, 50) = a(10); M(75, 53) = a(2); M(75, 76) = b(19); M(75, 77) = b(0); M(75, 78) = b(8);
		M(75, 79) = b(12); M(75, 80) = b(10); M(75, 82) = b(2); M(75, 84) = b(1); M(75, 86) = c(10); M(75, 87) = c(2);
		M(75, 88) = c(1); M(75, 92) = c(8); M(75, 96) = c(12); M(75, 98) = c(0); M(75, 119) = c(19); M(76, 48) = a(19);
		M(76, 81) = b(19); M(76, 85) = c(19); M(77, 34) = a(15); M(77, 51) = a(13); M(77, 54) = b(15); M(77, 65) = b(13);
		M(77, 89) = c(13); M(77, 99) = c(15); M(77, 105) = c(0); M(77, 109) = c(9); M(78, 27) = a(16); M(78, 31) = a(11);
		M(78, 37) = a(15); M(78, 40) = a(7); M(78, 42) = a(18); M(78, 55) = b(16); M(78, 56) = b(18); M(78, 58) = b(11);
		M(78, 72) = b(7); M(78, 77) = b(15); M(78, 95) = c(7); M(78, 98) = c(15); M(78, 101) = c(19); M(78, 106) = c(18);
		M(78, 108) = c(1); M(78, 113) = c(16); M(78, 115) = c(11); M(79, 42) = a(11); M(79, 44) = a(15); M(79, 46) = a(7);
		M(79, 56) = b(11); M(79, 66) = b(7); M(79, 68) = b(15); M(79, 97) = c(15); M(79, 101) = c(16); M(79, 102) = c(7);
		M(79, 105) = c(17); M(79, 106) = c(11); M(79, 108) = c(5); M(80, 14) = u(3); M(80, 28) = a(4); M(80, 33) = a(11);
		M(80, 39) = a(15); M(80, 41) = a(3); M(80, 42) = a(9); M(80, 43) = a(7); M(80, 47) = a(13); M(80, 51) = a(16);
		M(80, 56) = b(9); M(80, 57) = b(4); M(80, 59) = b(11); M(80, 60) = b(13); M(80, 62) = b(3); M(80, 65) = b(16);
		M(80, 71) = b(7); M(80, 79) = b(15); M(80, 89) = c(16); M(80, 90) = c(13); M(80, 96) = c(15); M(80, 100) = c(7);
		M(80, 101) = c(8); M(80, 106) = c(9); M(80, 108) = c(10); M(80, 109) = c(2); M(80, 111) = c(3); M(80, 114) = c(4);
		M(80, 117) = c(11); M(81, 31) = a(2); M(81, 32) = a(1); M(81, 37) = a(3); M(81, 38) = a(18); M(81, 40) = a(12);
		M(81, 47) = a(19); M(81, 48) = a(4); M(81, 50) = a(16); M(81, 58) = b(2); M(81, 60) = b(19); M(81, 61) = b(1);
		M(81, 72) = b(12); M(81, 77) = b(3); M(81, 78) = b(18); M(81, 80) = b(16); M(81, 81) = b(4); M(81, 85) = c(4);
		M(81, 86) = c(16); M(81, 90) = c(19); M(81, 92) = c(18); M(81, 95) = c(12); M(81, 98) = c(3); M(81, 115) = c(2);
		M(81, 116) = c(1); M(82, 29) = a(5); M(82, 44) = a(14); M(82, 45) = a(17); M(82, 47) = a(6); M(82, 60) = b(6);
		M(82, 63) = b(17); M(82, 64) = b(3); M(82, 67) = b(2); M(82, 68) = b(14); M(82, 69) = b(16); M(82, 70) = b(4);
		M(82, 74) = b(5); M(82, 90) = c(6); M(82, 91) = c(4); M(82, 94) = c(5); M(82, 97) = c(14); M(82, 103) = c(3);
		M(82, 104) = c(2); M(82, 110) = c(16); M(82, 112) = c(17); M(83, 26) = u(1); M(83, 27) = a(17); M(83, 31) = a(5);
		M(83, 37) = a(11); M(83, 40) = a(16); M(83, 42) = a(1); M(83, 44) = a(18); M(83, 46) = a(19); M(83, 52) = a(7);
		M(83, 55) = b(17); M(83, 56) = b(1); M(83, 58) = b(5); M(83, 66) = b(19); M(83, 68) = b(18); M(83, 72) = b(16);
		M(83, 73) = b(7); M(83, 77) = b(11); M(83, 93) = c(7); M(83, 95) = c(16); M(83, 97) = c(18); M(83, 98) = c(11);
		M(83, 102) = c(19); M(83, 106) = c(1); M(83, 113) = c(17); M(83, 115) = c(5); M(84, 16) = u(2); M(84, 29) = a(4);
		M(84, 30) = a(16); M(84, 33) = a(6); M(84, 39) = a(5); M(84, 43) = a(17); M(84, 44) = a(10); M(84, 45) = a(2);
		M(84, 47) = a(14); M(84, 59) = b(6); M(84, 60) = b(14); M(84, 63) = b(2); M(84, 67) = b(8); M(84, 68) = b(10);
		M(84, 69) = b(3); M(84, 70) = b(9); M(84, 71) = b(17); M(84, 74) = b(4); M(84, 75) = b(16); M(84, 79) = b(5);
		M(84, 90) = c(14); M(84, 91) = c(9); M(84, 94) = c(4); M(84, 96) = c(5); M(84, 97) = c(10); M(84, 100) = c(17);
		M(84, 104) = c(8); M(84, 107) = c(16); M(84, 110) = c(3); M(84, 112) = c(2); M(84, 117) = c(6); M(85, 25) = u(1);
		M(85, 29) = a(18); M(85, 31) = a(14); M(85, 32) = a(17); M(85, 37) = a(4); M(85, 38) = a(16); M(85, 40) = a(2);
		M(85, 44) = a(12); M(85, 45) = a(19); M(85, 47) = a(1); M(85, 50) = a(5); M(85, 52) = a(3); M(85, 58) = b(14);
		M(85, 60) = b(1); M(85, 61) = b(17); M(85, 63) = b(19); M(85, 68) = b(12); M(85, 72) = b(2); M(85, 73) = b(3);
		M(85, 74) = b(18); M(85, 77) = b(4); M(85, 78) = b(16); M(85, 80) = b(5); M(85, 86) = c(5); M(85, 90) = c(1);
		M(85, 92) = c(16); M(85, 93) = c(3); M(85, 94) = c(18); M(85, 95) = c(2); M(85, 97) = c(12); M(85, 98) = c(4);
		M(85, 112) = c(19); M(85, 115) = c(14); M(85, 116) = c(17); M(86, 31) = a(17); M(86, 37) = a(16); M(86, 40) = a(1);
		M(86, 44) = a(19); M(86, 48) = a(5); M(86, 52) = a(18); M(86, 58) = b(17); M(86, 68) = b(19); M(86, 72) = b(1);
		M(86, 73) = b(18); M(86, 77) = b(16); M(86, 81) = b(5); M(86, 85) = c(5); M(86, 93) = c(18); M(86, 95) = c(1);
		M(86, 97) = c(19); M(86, 98) = c(16); M(86, 115) = c(17); M(87, 22) = u(1); M(87, 31) = a(10); M(87, 32) = a(2);
		M(87, 33) = a(1); M(87, 37) = a(9); M(87, 38) = a(3); M(87, 39) = a(18); M(87, 40) = a(8); M(87, 43) = a(19);
		M(87, 47) = a(12); M(87, 50) = a(4); M(87, 53) = a(16); M(87, 58) = b(10); M(87, 59) = b(1); M(87, 60) = b(12);
		M(87, 61) = b(2); M(87, 71) = b(19); M(87, 72) = b(8); M(87, 77) = b(9); M(87, 78) = b(3); M(87, 79) = b(18);
		M(87, 80) = b(4); M(87, 82) = b(16); M(87, 86) = c(4); M(87, 87) = c(16); M(87, 90) = c(12); M(87, 92) = c(3);
		M(87, 95) = c(8); M(87, 96) = c(18); M(87, 98) = c(9); M(87, 100) = c(19); M(87, 115) = c(10); M(87, 116) = c(2);
		M(87, 117) = c(1); M(88, 27) = a(1); M(88, 31) = a(16); M(88, 37) = a(7); M(88, 40) = a(18); M(88, 42) = a(19);
		M(88, 48) = a(11); M(88, 55) = b(1); M(88, 56) = b(19); M(88, 58) = b(16); M(88, 72) = b(18); M(88, 77) = b(7);
		M(88, 81) = b(11); M(88, 85) = c(11); M(88, 95) = c(18); M(88, 98) = c(7); M(88, 106) = c(19); M(88, 113) = c(1);
		M(88, 115) = c(16); M(89, 31) = a(6); M(89, 37) = a(5); M(89, 40) = a(17); M(89, 44) = a(1); M(89, 52) = a(16);
		M(89, 58) = b(6); M(89, 67) = b(19); M(89, 68) = b(1); M(89, 70) = b(18); M(89, 72) = b(17); M(89, 73) = b(16);
		M(89, 77) = b(5); M(89, 91) = c(18); M(89, 93) = c(16); M(89, 95) = c(17); M(89, 97) = c(1); M(89, 98) = c(5);
		M(89, 104) = c(19); M(89, 115) = c(6); M(90, 27) = a(11); M(90, 40) = a(15); M(90, 42) = a(7); M(90, 55) = b(11);
		M(90, 56) = b(7); M(90, 72) = b(15); M(90, 95) = c(15); M(90, 101) = c(18); M(90, 105) = c(1); M(90, 106) = c(7);
		M(90, 108) = c(16); M(90, 113) = c(11); M(91, 23) = u(1); M(91, 27) = a(2); M(91, 28) = a(1); M(91, 31) = a(4);
		M(91, 32) = a(16); M(91, 37) = a(13); M(91, 38) = a(7); M(91, 40) = a(3); M(91, 41) = a(19); M(91, 42) = a(12);
		M(91, 47) = a(18); M(91, 50) = a(11); M(91, 55) = b(2); M(91, 56) = b(12); M(91, 57) = b(1); M(91, 58) = b(4);
		M(91, 60) = b(18); M(91, 61) = b(16); M(91, 62) = b(19); M(91, 72) = b(3); M(91, 77) = b(13); M(91, 78) = b(7);
		M(91, 80) = b(11); M(91, 86) = c(11); M(91, 90) = c(18); M(91, 92) = c(7); M(91, 95) = c(3); M(91, 98) = c(13);
		M(91, 106) = c(12); M(91, 111) = c(19); M(91, 113) = c(2); M(91, 114) = c(1); M(91, 115) = c(4); M(91, 116) = c(16);
		M(92, 17) = u(3); M(92, 28) = a(5); M(92, 29) = a(15); M(92, 41) = a(16); M(92, 42) = a(4); M(92, 44) = a(13);
		M(92, 45) = a(7); M(92, 46) = a(3); M(92, 47) = a(11); M(92, 56) = b(4); M(92, 57) = b(5); M(92, 60) = b(11);
		M(92, 62) = b(16); M(92, 63) = b(7); M(92, 66) = b(3); M(92, 68) = b(13); M(92, 74) = b(15); M(92, 90) = c(11);
		M(92, 94) = c(15); M(92, 97) = c(13); M(92, 101) = c(2); M(92, 102) = c(3); M(92, 106) = c(4); M(92, 108) = c(14);
		M(92, 109) = c(17); M(92, 111) = c(16); M(92, 112) = c(7); M(92, 114) = c(5); M(93, 17) = u(2); M(93, 28) = a(6);
		M(93, 29) = a(11); M(93, 41) = a(17); M(93, 42) = a(14); M(93, 44) = a(4); M(93, 45) = a(16); M(93, 46) = a(2);
		M(93, 47) = a(5); M(93, 56) = b(14); M(93, 57) = b(6); M(93, 60) = b(5); M(93, 62) = b(17); M(93, 63) = b(16);
		M(93, 66) = b(2); M(93, 67) = b(3); M(93, 68) = b(4); M(93, 69) = b(7); M(93, 70) = b(13); M(93, 74) = b(11);
		M(93, 90) = c(5); M(93, 91) = c(13); M(93, 94) = c(11); M(93, 97) = c(4); M(93, 102) = c(2); M(93, 104) = c(3);
		M(93, 106) = c(14); M(93, 110) = c(7); M(93, 111) = c(17); M(93, 112) = c(16); M(93, 114) = c(6); M(94, 31) = a(1);
		M(94, 37) = a(18); M(94, 40) = a(19); M(94, 48) = a(16); M(94, 58) = b(1); M(94, 72) = b(19); M(94, 77) = b(18);
		M(94, 81) = b(16); M(94, 85) = c(16); M(94, 95) = c(19); M(94, 98) = c(18); M(94, 115) = c(1); M(95, 26) = u(3);
		M(95, 27) = a(5); M(95, 40) = a(11); M(95, 42) = a(16); M(95, 44) = a(7); M(95, 46) = a(18); M(95, 52) = a(15);
		M(95, 55) = b(5); M(95, 56) = b(16); M(95, 66) = b(18); M(95, 68) = b(7); M(95, 72) = b(11); M(95, 73) = b(15);
		M(95, 93) = c(15); M(95, 95) = c(11); M(95, 97) = c(7); M(95, 101) = c(1); M(95, 102) = c(18); M(95, 106) = c(16);
		M(95, 108) = c(17); M(95, 113) = c(5); M(96, 23) = u(3); M(96, 27) = a(4); M(96, 28) = a(16); M(96, 32) = a(11);
		M(96, 38) = a(15); M(96, 40) = a(13); M(96, 41) = a(18); M(96, 42) = a(3); M(96, 47) = a(7); M(96, 55) = b(4);
		M(96, 56) = b(3); M(96, 57) = b(16); M(96, 60) = b(7); M(96, 61) = b(11); M(96, 62) = b(18); M(96, 72) = b(13);
		M(96, 78) = b(15); M(96, 90) = c(7); M(96, 92) = c(15); M(96, 95) = c(13); M(96, 101) = c(12); M(96, 106) = c(3);
		M(96, 108) = c(2); M(96, 109) = c(1); M(96, 111) = c(18); M(96, 113) = c(4); M(96, 114) = c(16); M(96, 116) = c(11);
		M(97, 42) = a(5); M(97, 44) = a(11); M(97, 46) = a(16); M(97, 56) = b(5); M(97, 66) = b(16); M(97, 67) = b(7);
		M(97, 68) = b(11); M(97, 70) = b(15); M(97, 91) = c(15); M(97, 97) = c(11); M(97, 101) = c(17); M(97, 102) = c(16);
		M(97, 104) = c(7); M(97, 106) = c(5); M(97, 108) = c(6); M(98, 28) = a(11); M(98, 41) = a(7); M(98, 42) = a(13);
		M(98, 47) = a(15); M(98, 56) = b(13); M(98, 57) = b(11); M(98, 60) = b(15); M(98, 62) = b(7); M(98, 90) = c(15);
		M(98, 101) = c(3); M(98, 105) = c(2); M(98, 106) = c(13); M(98, 108) = c(4); M(98, 109) = c(16); M(98, 111) = c(7);
		M(98, 114) = c(11); M(99, 42) = a(6); M(99, 44) = a(5); M(99, 46) = a(17); M(99, 56) = b(6); M(99, 64) = b(7);
		M(99, 66) = b(17); M(99, 67) = b(16); M(99, 68) = b(5); M(99, 70) = b(11); M(99, 91) = c(11); M(99, 97) = c(5);
		M(99, 102) = c(17); M(99, 103) = c(7); M(99, 104) = c(16); M(99, 106) = c(6); M(100, 51) = a(15); M(100, 65) = b(15);
		M(100, 89) = c(15); M(100, 105) = c(9); M(100, 109) = c(13); M(101, 37) = a(8); M(101, 38) = a(12); M(101, 39) = a(19);
		M(101, 48) = a(10); M(101, 50) = a(2); M(101, 53) = a(1); M(101, 77) = b(8); M(101, 78) = b(12); M(101, 79) = b(19);
		M(101, 80) = b(2); M(101, 81) = b(10); M(101, 82) = b(1); M(101, 85) = c(10); M(101, 86) = c(2); M(101, 87) = c(1);
		M(101, 92) = c(12); M(101, 96) = c(19); M(101, 98) = c(8); M(102, 37) = a(12); M(102, 38) = a(19); M(102, 48) = a(2);
		M(102, 50) = a(1); M(102, 77) = b(12); M(102, 78) = b(19); M(102, 80) = b(1); M(102, 81) = b(2); M(102, 85) = c(2);
		M(102, 86) = c(1); M(102, 92) = c(19); M(102, 98) = c(12); M(103, 37) = a(19); M(103, 48) = a(1); M(103, 77) = b(19);
		M(103, 81) = b(1); M(103, 85) = c(1); M(103, 98) = c(19); M(104, 11) = u(3); M(104, 28) = a(9); M(104, 33) = a(13);
		M(104, 34) = a(7); M(104, 36) = a(15); M(104, 51) = a(3); M(104, 54) = b(7); M(104, 57) = b(9); M(104, 59) = b(13);
		M(104, 65) = b(3); M(104, 84) = b(15); M(104, 88) = c(15); M(104, 89) = c(3); M(104, 99) = c(7); M(104, 108) = c(0);
		M(104, 109) = c(8); M(104, 114) = c(9); M(104, 117) = c(13); M(105, 20) = u(3); M(105, 27) = a(9); M(105, 28) = a(3);
		M(105, 32) = a(13); M(105, 33) = a(7); M(105, 51) = a(18); M(105, 53) = a(15); M(105, 55) = b(9); M(105, 57) = b(3);
		M(105, 59) = b(7); M(105, 61) = b(13); M(105, 65) = b(18); M(105, 82) = b(15); M(105, 87) = c(15); M(105, 89) = c(18);
		M(105, 108) = c(8); M(105, 109) = c(12); M(105, 113) = c(9); M(105, 114) = c(3); M(105, 116) = c(13);
		M(105, 117) = c(7); M(106, 27) = a(3); M(106, 28) = a(18); M(106, 31) = a(13); M(106, 32) = a(7); M(106, 50) = a(15);
		M(106, 55) = b(3); M(106, 57) = b(18); M(106, 58) = b(13); M(106, 61) = b(7); M(106, 80) = b(15); M(106, 86) = c(15);
		M(106, 108) = c(12); M(106, 109) = c(19); M(106, 113) = c(3); M(106, 114) = c(18); M(106, 115) = c(13);
		M(106, 116) = c(7); M(107, 27) = a(18); M(107, 31) = a(7); M(107, 48) = a(15); M(107, 55) = b(18); M(107, 58) = b(7);
		M(107, 81) = b(15); M(107, 85) = c(15); M(107, 108) = c(19); M(107, 113) = c(18); M(107, 115) = c(7);
		M(108, 36) = a(19); M(108, 48) = a(0); M(108, 50) = a(8); M(108, 53) = a(12); M(108, 80) = b(8); M(108, 81) = b(0);
		M(108, 82) = b(12); M(108, 84) = b(19); M(108, 85) = c(0); M(108, 86) = c(8); M(108, 87) = c(12); M(108, 88) = c(19);
		M(109, 26) = u(2); M(109, 27) = a(6); M(109, 40) = a(5); M(109, 42) = a(17); M(109, 44) = a(16); M(109, 46) = a(1);
		M(109, 52) = a(11); M(109, 55) = b(6); M(109, 56) = b(17); M(109, 66) = b(1); M(109, 67) = b(18); M(109, 68) = b(16);
		M(109, 70) = b(7); M(109, 72) = b(5); M(109, 73) = b(11); M(109, 91) = c(7); M(109, 93) = c(11); M(109, 95) = c(5);
		M(109, 97) = c(16); M(109, 102) = c(1); M(109, 104) = c(18); M(109, 106) = c(17); M(109, 113) = c(6); M(110, 7) = u(2);
		M(110, 30) = a(4); M(110, 34) = a(6); M(110, 43) = a(14); M(110, 45) = a(10); M(110, 49) = a(5); M(110, 54) = b(6);
		M(110, 63) = b(10); M(110, 67) = b(0); M(110, 69) = b(9); M(110, 71) = b(14); M(110, 75) = b(4); M(110, 76) = b(5);
		M(110, 99) = c(6); M(110, 100) = c(14); M(110, 104) = c(0); M(110, 107) = c(4); M(110, 110) = c(9); M(110, 112) = c(10);
		M(110, 119) = c(5); M(111, 18) = u(1); M(111, 35) = a(19); M(111, 36) = a(12); M(111, 50) = a(0); M(111, 53) = a(8);
		M(111, 80) = b(0); M(111, 82) = b(8); M(111, 83) = b(19); M(111, 84) = b(12); M(111, 86) = c(0); M(111, 87) = c(8);
		M(111, 88) = c(12); M(111, 118) = c(19); M(112, 19) = u(1); M(112, 31) = a(0); M(112, 32) = a(8); M(112, 33) = a(12);
		M(112, 34) = a(19); M(112, 36) = a(18); M(112, 50) = a(9); M(112, 53) = a(3); M(112, 54) = b(19); M(112, 58) = b(0);
		M(112, 59) = b(12); M(112, 61) = b(8); M(112, 80) = b(9); M(112, 82) = b(3); M(112, 84) = b(18); M(112, 86) = c(9);
		M(112, 87) = c(3); M(112, 88) = c(18); M(112, 99) = c(19); M(112, 115) = c(0); M(112, 116) = c(8); M(112, 117) = c(12);
		M(113, 31) = a(8); M(113, 32) = a(12); M(113, 33) = a(19); M(113, 48) = a(9); M(113, 50) = a(3); M(113, 53) = a(18);
		M(113, 58) = b(8); M(113, 59) = b(19); M(113, 61) = b(12); M(113, 80) = b(3); M(113, 81) = b(9); M(113, 82) = b(18);
		M(113, 85) = c(9); M(113, 86) = c(3); M(113, 87) = c(18); M(113, 115) = c(8); M(113, 116) = c(12); M(113, 117) = c(19);
		M(114, 31) = a(12); M(114, 32) = a(19); M(114, 48) = a(3); M(114, 50) = a(18); M(114, 58) = b(12); M(114, 61) = b(19);
		M(114, 80) = b(18); M(114, 81) = b(3); M(114, 85) = c(3); M(114, 86) = c(18); M(114, 115) = c(12); M(114, 116) = c(19);
		M(115, 31) = a(19); M(115, 48) = a(18); M(115, 58) = b(19); M(115, 81) = b(18); M(115, 85) = c(18); M(115, 115) = c(19);
		M(116, 24) = u(1); M(116, 29) = a(12); M(116, 30) = a(19); M(116, 37) = a(10); M(116, 38) = a(2); M(116, 39) = a(1);
		M(116, 50) = a(14); M(116, 52) = a(8); M(116, 53) = a(17); M(116, 73) = b(8); M(116, 74) = b(12); M(116, 75) = b(19);
		M(116, 77) = b(10); M(116, 78) = b(2); M(116, 79) = b(1); M(116, 80) = b(14); M(116, 82) = b(17); M(116, 86) = c(14);
		M(116, 87) = c(17); M(116, 92) = c(2); M(116, 93) = c(8); M(116, 94) = c(12); M(116, 96) = c(1); M(116, 98) = c(10);
		M(116, 107) = c(19); M(117, 29) = a(19); M(117, 37) = a(2); M(117, 38) = a(1); M(117, 48) = a(14); M(117, 50) = a(17);
		M(117, 52) = a(12); M(117, 73) = b(12); M(117, 74) = b(19); M(117, 77) = b(2); M(117, 78) = b(1); M(117, 80) = b(17);
		M(117, 81) = b(14); M(117, 85) = c(14); M(117, 86) = c(17); M(117, 92) = c(1); M(117, 93) = c(12); M(117, 94) = c(19);
		M(117, 98) = c(2); M(118, 27) = a(12); M(118, 28) = a(19); M(118, 31) = a(3); M(118, 32) = a(18); M(118, 48) = a(13);
		M(118, 50) = a(7); M(118, 55) = b(12); M(118, 57) = b(19); M(118, 58) = b(3); M(118, 61) = b(18); M(118, 80) = b(7);
		M(118, 81) = b(13); M(118, 85) = c(13); M(118, 86) = c(7); M(118, 113) = c(12); M(118, 114) = c(19); M(118, 115) = c(3);
		M(118, 116) = c(18); M(119, 27) = a(19); M(119, 31) = a(18); M(119, 48) = a(7); M(119, 55) = b(19); M(119, 58) = b(18);
		M(119, 81) = b(7); M(119, 85) = c(7); M(119, 113) = c(19); M(119, 115) = c(18);
		// t_odo - use sparse matrix (it is 120x120, there are 1968 nonzeros, approx. 13.666% sparse)
#endif // _DEBUG

		static const int p[] = {0, 4, 12, 23, 31, 46, 67, 78, 99, 126, 134, 149, 170, 185, 213, 250, 271, 308, 351, 362,
			383, 410, 431, 468, 511, 538, 581, 624, 644, 656, 679, 690, 709, 718, 733, 748, 758, 776, 787, 797, 809, 821,
			828, 836, 845, 851, 856, 863, 878, 893, 905, 911, 915, 930, 934, 936, 945, 947, 951, 958, 979, 1002, 1008,
			1010, 1011, 1018, 1026, 1031, 1033, 1045, 1054, 1079, 1088, 1103, 1109, 1140, 1162, 1165, 1173, 1190, 1202,
			1230, 1254, 1274, 1299, 1330, 1364, 1382, 1416, 1434, 1453, 1465, 1499, 1527, 1558, 1570, 1591, 1619, 1634,
			1650, 1665, 1670, 1688, 1700, 1706, 1724, 1745, 1762, 1772, 1784, 1807, 1827, 1840, 1862, 1880, 1892, 1898,
			1923, 1941, 1959, 1968};
		_ASSERTE(sizeof(p) / sizeof(p[0]) - 1 == 120); // there should be 120 cols

		static const int i[] = {0, 35, 83, 118, 0, 1, 34, 35, 54, 83, 99, 118, 1, 2, 34, 35, 51, 54, 65, 83, 89, 99, 118,
			0, 3, 35, 49, 76, 83, 118, 119, 1, 3, 4, 34, 35, 43, 49, 54, 71, 76, 83, 99, 100, 118, 119, 2, 4, 5, 34, 35,
			41, 43, 49, 51, 54, 62, 65, 71, 76, 83, 89, 99, 100, 111, 118, 119, 3, 6, 30, 35, 49, 75, 76, 83, 107, 118,
			119, 4, 6, 7, 30, 34, 35, 43, 45, 49, 54, 63, 71, 75, 76, 83, 99, 100, 107, 112, 118, 119, 5, 7, 8, 30, 34,
			41, 43, 45, 46, 49, 51, 54, 62, 63, 65, 66, 71, 75, 76, 89, 99, 100, 102, 107, 111, 112, 119, 0, 9, 35, 36,
			83, 84, 88, 118, 1, 9, 10, 33, 34, 35, 36, 54, 59, 83, 84, 88, 99, 117, 118, 2, 10, 11, 28, 33, 34, 35, 36,
			51, 54, 57, 59, 65, 83, 84, 88, 89, 99, 114, 117, 118, 3, 9, 12, 35, 36, 39, 49, 76, 79, 83, 84, 88, 96, 118,
			119, 4, 10, 12, 13, 33, 34, 35, 36, 39, 43, 47, 49, 54, 59, 60, 71, 76, 79, 83, 84, 88, 90, 96, 99, 100, 117,
			118, 119, 5, 11, 13, 14, 28, 33, 34, 36, 39, 41, 42, 43, 47, 49, 51, 54, 56, 57, 59, 60, 62, 65, 71, 76, 79,
			84, 88, 89, 90, 96, 99, 100, 106, 111, 114, 117, 119, 6, 12, 15, 29, 30, 35, 36, 39, 49, 74, 75, 76, 79, 83,
			84, 88, 94, 96, 107, 118, 119, 7, 13, 15, 16, 29, 30, 33, 34, 36, 39, 43, 44, 45, 47, 49, 54, 59, 60, 63, 68,
			71, 74, 75, 76, 79, 84, 88, 90, 94, 96, 97, 99, 100, 107, 112, 117, 119, 8, 14, 16, 17, 28, 29, 30, 33, 39,
			41, 42, 43, 44, 45, 46, 47, 51, 56, 57, 59, 60, 62, 63, 65, 66, 68, 71, 74, 75, 79, 89, 90, 94, 96, 97, 100,
			102, 106, 107, 111, 112, 114, 117, 9, 18, 35, 36, 53, 82, 83, 84, 87, 88, 118, 10, 18, 19, 32, 33, 34, 35, 36,
			53, 54, 59, 61, 82, 83, 84, 87, 88, 99, 116, 117, 118, 11, 19, 20, 27, 28, 32, 33, 34, 36, 51, 53, 54, 55, 57,
			59, 61, 65, 82, 84, 87, 88, 89, 99, 113, 114, 116, 117, 12, 18, 21, 35, 36, 38, 39, 49, 53, 76, 78, 79, 82,
			83, 84, 87, 88, 92, 96, 118, 119, 13, 19, 21, 22, 32, 33, 34, 36, 38, 39, 40, 43, 47, 49, 53, 54, 59, 60, 61,
			71, 72, 76, 78, 79, 82, 84, 87, 88, 90, 92, 95, 96, 99, 100, 116, 117, 119, 14, 20, 22, 23, 27, 28, 32, 33,
			38, 39, 40, 41, 42, 43, 47, 51, 53, 55, 56, 57, 59, 60, 61, 62, 65, 71, 72, 78, 79, 82, 87, 89, 90, 92, 95,
			96, 100, 106, 111, 113, 114, 116, 117, 15, 21, 24, 29, 30, 36, 38, 39, 49, 52, 53, 73, 74, 75, 76, 78, 79, 82,
			84, 87, 88, 92, 93, 94, 96, 107, 119, 16, 22, 24, 25, 29, 30, 32, 33, 38, 39, 40, 43, 44, 45, 47, 52, 53, 59,
			60, 61, 63, 68, 71, 72, 73, 74, 75, 78, 79, 82, 87, 90, 92, 93, 94, 95, 96, 97, 100, 107, 112, 116, 117, 17,
			23, 25, 26, 27, 28, 29, 32, 38, 40, 41, 42, 44, 45, 46, 47, 52, 55, 56, 57, 60, 61, 62, 63, 66, 68, 72, 73,
			74, 78, 90, 92, 93, 94, 95, 97, 102, 106, 111, 112, 113, 114, 116, 15, 29, 30, 36, 39, 49, 69, 70, 74, 75, 76,
			79, 84, 88, 91, 94, 96, 107, 110, 119, 6, 30, 35, 49, 69, 75, 76, 83, 107, 110, 118, 119, 24, 29, 30, 38, 39,
			52, 53, 69, 70, 73, 74, 75, 78, 79, 82, 87, 91, 92, 93, 94, 96, 107, 110, 37, 48, 52, 70, 73, 77, 81, 85, 91,
			93, 98, 29, 37, 38, 50, 52, 69, 70, 73, 74, 77, 78, 80, 86, 91, 92, 93, 94, 98, 110, 48, 50, 53, 80, 81, 82,
			85, 86, 87, 29, 30, 39, 64, 69, 70, 74, 75, 79, 91, 94, 96, 103, 107, 110, 29, 38, 52, 64, 69, 70, 73, 74, 78,
			91, 92, 93, 94, 103, 110, 37, 52, 64, 70, 73, 77, 91, 93, 98, 103, 5, 34, 41, 43, 49, 51, 54, 62, 65, 71, 76,
			89, 99, 100, 101, 109, 111, 119, 2, 34, 35, 51, 54, 65, 83, 89, 99, 109, 118, 30, 49, 64, 69, 75, 76, 103,
			107, 110, 119, 28, 33, 51, 57, 59, 65, 89, 105, 108, 109, 114, 117, 27, 28, 32, 55, 57, 61, 105, 108, 109,
			113, 114, 116, 30, 64, 69, 75, 103, 107, 110, 27, 31, 55, 58, 105, 108, 113, 115, 29, 64, 69, 70, 74, 91, 94,
			103, 110, 28, 57, 105, 108, 109, 114, 27, 55, 105, 108, 113, 52, 64, 70, 73, 91, 93, 103, 40, 44, 52, 64, 67,
			68, 70, 72, 73, 91, 93, 95, 97, 103, 104, 30, 43, 45, 63, 64, 67, 69, 71, 75, 100, 103, 104, 107, 110, 112,
			41, 45, 46, 62, 63, 66, 101, 102, 105, 109, 111, 112, 41, 62, 101, 105, 109, 111, 64, 70, 91, 103, 41, 45, 46,
			62, 63, 66, 67, 69, 101, 102, 104, 109, 110, 111, 112, 64, 69, 103, 110, 105, 109, 44, 64, 67, 68, 70, 91, 97,
			103, 104, 105, 108, 64, 67, 103, 104, 46, 64, 66, 67, 102, 103, 104, 8, 30, 41, 43, 45, 46, 51, 62, 63, 65,
			66, 71, 75, 89, 100, 101, 102, 107, 109, 111, 112, 8, 30, 41, 43, 45, 46, 51, 62, 63, 65, 66, 67, 69, 71, 75,
			89, 100, 102, 104, 107, 110, 111, 112, 42, 56, 101, 105, 106, 108, 64, 103, 105, 46, 66, 67, 101, 102, 104,
			105, 46, 64, 66, 67, 101, 102, 103, 104, 46, 66, 101, 102, 105, 101, 105, 41, 43, 51, 62, 65, 71, 89, 100,
			101, 105, 109, 111, 37, 48, 52, 73, 77, 81, 85, 93, 98, 20, 27, 28, 31, 32, 33, 50, 51, 53, 55, 57, 58, 59,
			61, 65, 80, 82, 86, 87, 89, 113, 114, 115, 116, 117, 45, 63, 64, 67, 69, 103, 104, 110, 112, 41, 45, 46, 62,
			63, 64, 66, 67, 69, 102, 103, 104, 110, 111, 112, 48, 50, 80, 81, 85, 86, 25, 29, 32, 38, 40, 44, 45, 47, 52,
			60, 61, 63, 67, 68, 69, 70, 72, 73, 74, 78, 90, 91, 92, 93, 94, 95, 97, 104, 110, 112, 116, 21, 36, 37, 38,
			39, 49, 50, 53, 76, 77, 78, 79, 80, 82, 84, 86, 87, 88, 92, 96, 98, 119, 48, 81, 85, 34, 51, 54, 65, 89, 99,
			105, 109, 27, 31, 37, 40, 42, 55, 56, 58, 72, 77, 95, 98, 101, 106, 108, 113, 115, 42, 44, 46, 56, 66, 68, 97,
			101, 102, 105, 106, 108, 14, 28, 33, 39, 41, 42, 43, 47, 51, 56, 57, 59, 60, 62, 65, 71, 79, 89, 90, 96, 100,
			101, 106, 108, 109, 111, 114, 117, 31, 32, 37, 38, 40, 47, 48, 50, 58, 60, 61, 72, 77, 78, 80, 81, 85, 86, 90,
			92, 95, 98, 115, 116, 29, 44, 45, 47, 60, 63, 64, 67, 68, 69, 70, 74, 90, 91, 94, 97, 103, 104, 110, 112, 26,
			27, 31, 37, 40, 42, 44, 46, 52, 55, 56, 58, 66, 68, 72, 73, 77, 93, 95, 97, 98, 102, 106, 113, 115, 16, 29,
			30, 33, 39, 43, 44, 45, 47, 59, 60, 63, 67, 68, 69, 70, 71, 74, 75, 79, 90, 91, 94, 96, 97, 100, 104, 107,
			110, 112, 117, 25, 29, 31, 32, 37, 38, 40, 44, 45, 47, 50, 52, 58, 60, 61, 63, 68, 72, 73, 74, 77, 78, 80, 86,
			90, 92, 93, 94, 95, 97, 98, 112, 115, 116, 31, 37, 40, 44, 48, 52, 58, 68, 72, 73, 77, 81, 85, 93, 95, 97, 98,
			115, 22, 31, 32, 33, 37, 38, 39, 40, 43, 47, 50, 53, 58, 59, 60, 61, 71, 72, 77, 78, 79, 80, 82, 86, 87, 90,
			92, 95, 96, 98, 100, 115, 116, 117, 27, 31, 37, 40, 42, 48, 55, 56, 58, 72, 77, 81, 85, 95, 98, 106, 113, 115,
			31, 37, 40, 44, 52, 58, 67, 68, 70, 72, 73, 77, 91, 93, 95, 97, 98, 104, 115, 27, 40, 42, 55, 56, 72, 95, 101,
			105, 106, 108, 113, 23, 27, 28, 31, 32, 37, 38, 40, 41, 42, 47, 50, 55, 56, 57, 58, 60, 61, 62, 72, 77, 78,
			80, 86, 90, 92, 95, 98, 106, 111, 113, 114, 115, 116, 17, 28, 29, 41, 42, 44, 45, 46, 47, 56, 57, 60, 62, 63,
			66, 68, 74, 90, 94, 97, 101, 102, 106, 108, 109, 111, 112, 114, 17, 28, 29, 41, 42, 44, 45, 46, 47, 56, 57,
			60, 62, 63, 66, 67, 68, 69, 70, 74, 90, 91, 94, 97, 102, 104, 106, 110, 111, 112, 114, 31, 37, 40, 48, 58, 72,
			77, 81, 85, 95, 98, 115, 26, 27, 40, 42, 44, 46, 52, 55, 56, 66, 68, 72, 73, 93, 95, 97, 101, 102, 106, 108,
			113, 23, 27, 28, 32, 38, 40, 41, 42, 47, 55, 56, 57, 60, 61, 62, 72, 78, 90, 92, 95, 101, 106, 108, 109, 111,
			113, 114, 116, 42, 44, 46, 56, 66, 67, 68, 70, 91, 97, 101, 102, 104, 106, 108, 28, 41, 42, 47, 56, 57, 60,
			62, 90, 101, 105, 106, 108, 109, 111, 114, 42, 44, 46, 56, 64, 66, 67, 68, 70, 91, 97, 102, 103, 104, 106, 51,
			65, 89, 105, 109, 37, 38, 39, 48, 50, 53, 77, 78, 79, 80, 81, 82, 85, 86, 87, 92, 96, 98, 37, 38, 48, 50, 77,
			78, 80, 81, 85, 86, 92, 98, 37, 48, 77, 81, 85, 98, 11, 28, 33, 34, 36, 51, 54, 57, 59, 65, 84, 88, 89, 99,
			108, 109, 114, 117, 20, 27, 28, 32, 33, 51, 53, 55, 57, 59, 61, 65, 82, 87, 89, 108, 109, 113, 114, 116, 117,
			27, 28, 31, 32, 50, 55, 57, 58, 61, 80, 86, 108, 109, 113, 114, 115, 116, 27, 31, 48, 55, 58, 81, 85, 108,
			113, 115, 36, 48, 50, 53, 80, 81, 82, 84, 85, 86, 87, 88, 26, 27, 40, 42, 44, 46, 52, 55, 56, 66, 67, 68, 70,
			72, 73, 91, 93, 95, 97, 102, 104, 106, 113, 7, 30, 34, 43, 45, 49, 54, 63, 67, 69, 71, 75, 76, 99, 100, 104,
			107, 110, 112, 119, 18, 35, 36, 50, 53, 80, 82, 83, 84, 86, 87, 88, 118, 19, 31, 32, 33, 34, 36, 50, 53, 54,
			58, 59, 61, 80, 82, 84, 86, 87, 88, 99, 115, 116, 117, 31, 32, 33, 48, 50, 53, 58, 59, 61, 80, 81, 82, 85, 86,
			87, 115, 116, 117, 31, 32, 48, 50, 58, 61, 80, 81, 85, 86, 115, 116, 31, 48, 58, 81, 85, 115, 24, 29, 30, 37,
			38, 39, 50, 52, 53, 73, 74, 75, 77, 78, 79, 80, 82, 86, 87, 92, 93, 94, 96, 98, 107, 29, 37, 38, 48, 50, 52,
			73, 74, 77, 78, 80, 81, 85, 86, 92, 93, 94, 98, 27, 28, 31, 32, 48, 50, 55, 57, 58, 61, 80, 81, 85, 86, 113,
			114, 115, 116, 27, 31, 48, 55, 58, 81, 85, 113, 115};
		_ASSERTE(sizeof(i) / sizeof(i[0]) == p[sizeof(p) / sizeof(p[0]) - 1]); // make sure the number of nonzeros is right

		const double x[] = {u(0), a(0), b(0), c(0), u(3), u(0), a(0), a(9), b(0), b(9), c(0), c(9), u(3), u(0), a(9),
			a(13), a(0), b(9), b(0), b(13), c(0), c(9), c(13), u(2), u(0), a(10), a(0), b(0), b(10), c(10), c(0), u(2),
			u(3), u(0), a(10), a(4), a(0), a(9), b(10), b(0), b(9), b(4), c(10), c(0), c(4), c(9), u(2), u(3), u(0), a(4),
			a(11), a(0), a(9), a(13), a(10), b(4), b(0), b(10), b(9), b(13), b(11), c(10), c(4), c(9), c(0), c(11), c(13),
			u(2), u(0), a(0), a(14), a(10), b(0), b(10), b(14), c(0), c(14), c(10), u(2), u(3), u(0), a(9), a(14), a(5),
			a(10), a(0), a(4), b(14), b(0), b(10), b(9), b(4), b(5), c(14), c(10), c(9), c(0), c(5), c(4), u(2), u(3),
			u(0), a(13), a(5), a(10), a(4), a(9), a(0), a(11), a(14), b(5), b(10), b(9), b(14), b(0), b(4), b(13), b(11),
			c(14), c(5), c(4), c(0), c(13), c(10), c(9), c(11), u(1), u(0), a(8), a(0), b(8), b(0), c(0), c(8), u(1),
			u(3), u(0), a(0), a(8), a(3), a(9), b(8), b(0), b(3), b(9), c(9), c(8), c(0), c(3), u(1), u(3), u(0), a(0),
			a(9), a(3), a(7), a(13), a(8), b(3), b(0), b(9), b(8), b(7), b(13), c(13), c(8), c(3), c(0), c(9), c(7), u(1),
			u(2), u(0), a(2), a(10), a(0), a(8), b(8), b(0), b(2), b(10), c(10), c(0), c(2), c(8), u(1), u(2), u(3), u(0),
			a(10), a(2), a(16), a(4), a(9), a(8), a(0), a(3), b(2), b(10), b(0), b(8), b(3), b(9), b(16), b(4), c(4),
			c(0), c(9), c(2), c(8), c(10), c(16), c(3), u(1), u(2), u(3), u(0), a(10), a(4), a(16), a(11), a(13), a(8),
			a(0), a(3), a(9), a(7), a(2), b(16), b(0), b(10), b(4), b(9), b(8), b(2), b(3), b(7), b(13), b(11), c(11),
			c(2), c(9), c(13), c(16), c(3), c(0), c(8), c(10), c(4), c(7), u(1), u(2), u(0), a(0), a(8), a(17), a(14),
			a(10), a(2), b(0), b(8), b(2), b(10), b(17), b(14), c(14), c(0), c(10), c(8), c(17), c(2), u(1), u(2), u(3),
			u(0), a(9), a(3), a(14), a(17), a(5), a(4), a(2), a(0), a(8), a(10), a(16), b(17), b(14), b(10), b(8), b(0),
			b(2), b(9), b(3), b(16), b(4), b(5), c(5), c(10), c(9), c(4), c(0), c(17), c(2), c(3), c(8), c(14), c(16),
			u(1), u(2), u(3), u(0), a(14), a(13), a(7), a(5), a(11), a(2), a(10), a(16), a(9), a(3), a(8), a(4), a(17),
			b(10), b(14), b(5), b(4), b(2), b(3), b(17), b(8), b(9), b(16), b(13), b(7), b(11), c(17), c(4), c(13), c(11),
			c(9), c(16), c(8), c(10), c(7), c(2), c(3), c(14), c(5), u(1), u(0), a(12), a(8), a(0), b(0), b(12), b(8),
			c(0), c(8), c(12), u(1), u(3), u(0), a(0), a(8), a(12), a(18), a(3), a(9), b(12), b(8), b(0), b(9), b(18),
			b(3), c(9), c(3), c(12), c(0), c(8), c(18), u(1), u(3), u(0), a(0), a(8), a(9), a(3), a(18), a(7), a(12),
			a(13), b(18), b(0), b(8), b(3), b(9), b(12), b(13), b(7), c(13), c(7), c(12), c(18), c(0), c(8), c(9), c(3),
			u(1), u(2), u(0), a(1), a(2), a(0), a(8), a(12), a(10), b(12), b(0), b(8), b(10), b(1), b(2), c(10), c(2),
			c(0), c(8), c(1), c(12), u(1), u(2), u(3), u(0), a(10), a(2), a(1), a(16), a(9), a(3), a(0), a(12), a(8),
			a(18), a(4), b(1), b(2), b(8), b(10), b(12), b(0), b(18), b(9), b(3), b(4), b(16), c(4), c(16), c(8), c(9),
			c(0), c(3), c(1), c(12), c(10), c(2), c(18), u(1), u(2), u(3), u(0), a(10), a(2), a(4), a(16), a(13), a(7),
			a(9), a(12), a(8), a(18), a(3), a(1), a(11), b(10), b(8), b(2), b(16), b(3), b(4), b(12), b(1), b(18), b(9),
			b(13), b(7), b(11), c(11), c(1), c(3), c(13), c(9), c(7), c(18), c(8), c(12), c(10), c(2), c(4), c(16), u(1),
			u(2), u(0), a(8), a(12), a(17), a(10), a(2), a(1), a(0), a(14), b(0), b(8), b(12), b(1), b(10), b(2), b(14),
			b(17), c(14), c(17), c(10), c(0), c(8), c(2), c(12), c(1), u(1), u(2), u(3), u(0), a(3), a(18), a(14), a(17),
			a(4), a(16), a(10), a(1), a(8), a(12), a(2), a(9), a(5), b(17), b(2), b(14), b(12), b(8), b(1), b(10), b(9),
			b(3), b(18), b(4), b(16), b(5), c(5), c(2), c(4), c(9), c(3), c(10), c(16), c(8), c(1), c(18), c(12), c(14),
			c(17), u(1), u(2), u(3), u(0), a(14), a(17), a(7), a(5), a(11), a(4), a(1), a(2), a(3), a(18), a(12), a(16),
			a(13), b(14), b(2), b(17), b(16), b(5), b(1), b(18), b(12), b(3), b(4), b(13), b(7), b(11), c(16), c(11),
			c(13), c(7), c(4), c(3), c(12), c(2), c(1), c(18), c(14), c(17), c(5), u(2), a(10), a(2), a(6), a(14), a(17),
			b(8), b(0), b(10), b(2), b(17), b(14), b(6), c(6), c(0), c(10), c(14), c(2), c(8), c(17), u(2), a(10), a(6),
			a(14), b(0), b(10), b(14), b(6), c(10), c(0), c(6), c(14), u(2), a(2), a(1), a(14), a(17), a(10), a(6), b(12),
			b(8), b(10), b(2), b(1), b(14), b(17), b(6), c(6), c(8), c(14), c(10), c(2), c(17), c(1), c(12), a(17), a(6),
			a(1), b(19), b(1), b(17), b(6), c(6), c(19), c(1), c(17), a(1), a(14), a(17), a(6), a(2), b(19), b(12), b(2),
			b(1), b(14), b(17), b(6), c(6), c(12), c(17), c(2), c(1), c(14), c(19), a(8), a(12), a(19), b(12), b(8),
			b(19), c(8), c(12), c(19), a(14), a(17), a(6), b(8), b(2), b(10), b(14), b(17), b(6), c(10), c(14), c(6),
			c(8), c(17), c(2), a(17), a(6), a(14), b(12), b(1), b(2), b(14), b(17), b(6), c(2), c(6), c(14), c(17), c(12),
			c(1), a(6), a(17), b(19), b(1), b(17), b(6), c(1), c(17), c(6), c(19), u(3), a(11), a(9), a(13), a(15), a(4),
			b(11), b(9), b(4), b(13), b(15), c(4), c(11), c(13), c(0), c(10), c(9), c(15), u(3), a(13), a(15), a(9),
			b(13), b(9), b(15), c(9), c(13), c(0), c(15), a(14), a(6), b(0), b(10), b(14), b(6), c(0), c(14), c(10), c(6),
			a(13), a(15), a(7), b(13), b(15), b(7), c(7), c(8), c(9), c(3), c(13), c(15), a(13), a(7), a(15), b(13), b(7),
			b(15), c(12), c(3), c(18), c(13), c(7), c(15), a(6), b(10), b(14), b(6), c(10), c(6), c(14), a(7), a(15),
			b(7), b(15), c(19), c(18), c(7), c(15), a(6), b(2), b(17), b(14), b(6), c(14), c(6), c(2), c(17), a(15),
			b(15), c(3), c(13), c(7), c(15), a(15), b(15), c(18), c(7), c(15), a(6), b(1), b(17), b(6), c(17), c(6), c(1),
			a(6), a(17), a(5), b(18), b(1), b(17), b(16), b(6), b(5), c(16), c(5), c(6), c(17), c(18), c(1), a(5), a(6),
			a(14), b(14), b(9), b(10), b(4), b(6), b(5), c(6), c(9), c(10), c(5), c(4), c(14), a(11), a(15), a(13), b(11),
			b(15), b(13), c(4), c(13), c(14), c(5), c(11), c(15), a(15), b(15), c(13), c(4), c(11), c(15), b(17), b(6),
			c(6), c(17), a(5), a(11), a(4), b(5), b(11), b(4), b(13), b(15), c(14), c(4), c(13), c(6), c(15), c(5), c(11),
			b(14), b(6), c(14), c(6), c(13), c(15), a(6), b(16), b(17), b(6), b(5), c(5), c(6), c(16), c(17), c(7), c(15),
			b(5), b(6), c(5), c(6), a(6), b(11), b(6), b(5), c(6), c(11), c(5), u(3), a(15), a(4), a(11), a(13), a(9),
			a(5), b(4), b(13), b(5), b(9), b(11), b(15), c(5), c(11), c(10), c(9), c(15), c(14), c(4), c(13), u(2), a(11),
			a(14), a(5), a(4), a(10), a(6), b(14), b(4), b(6), b(10), b(9), b(13), b(5), b(11), c(6), c(5), c(10), c(9),
			c(11), c(13), c(14), c(4), a(15), b(15), c(7), c(16), c(15), c(11), b(6), c(6), c(15), a(11), b(11), b(15),
			c(5), c(11), c(15), c(6), a(5), b(15), b(5), b(11), c(6), c(5), c(15), c(11), a(15), b(15), c(11), c(15),
			c(5), c(15), c(11), a(13), a(15), a(11), b(13), b(11), b(15), c(11), c(15), c(9), c(10), c(4), c(13), a(1),
			a(17), a(19), b(19), b(1), b(17), c(17), c(19), c(1), u(1), a(8), a(12), a(9), a(3), a(18), a(13), a(19),
			a(7), b(8), b(12), b(9), b(18), b(3), b(19), b(13), b(7), c(13), c(7), c(19), c(8), c(12), c(9), c(3), c(18),
			a(6), b(6), b(4), b(14), b(5), c(4), c(14), c(5), c(6), a(6), a(5), a(14), b(6), b(5), b(13), b(14), b(4),
			b(11), c(14), c(13), c(4), c(11), c(6), c(5), a(12), a(19), b(19), b(12), c(12), c(19), u(2), a(16), a(6),
			a(5), a(14), a(2), a(1), a(17), a(4), b(17), b(6), b(1), b(12), b(2), b(18), b(3), b(14), b(4), b(16), b(5),
			c(17), c(3), c(5), c(4), c(16), c(14), c(2), c(12), c(18), c(1), c(6), u(1), a(1), a(0), a(8), a(12), a(19),
			a(10), a(2), b(19), b(0), b(8), b(12), b(10), b(2), b(1), c(10), c(2), c(1), c(8), c(12), c(0), c(19), a(19),
			b(19), c(19), a(15), a(13), b(15), b(13), c(13), c(15), c(0), c(9), a(16), a(11), a(15), a(7), a(18), b(16),
			b(18), b(11), b(7), b(15), c(7), c(15), c(19), c(18), c(1), c(16), c(11), a(11), a(15), a(7), b(11), b(7),
			b(15), c(15), c(16), c(7), c(17), c(11), c(5), u(3), a(4), a(11), a(15), a(3), a(9), a(7), a(13), a(16), b(9),
			b(4), b(11), b(13), b(3), b(16), b(7), b(15), c(16), c(13), c(15), c(7), c(8), c(9), c(10), c(2), c(3), c(4),
			c(11), a(2), a(1), a(3), a(18), a(12), a(19), a(4), a(16), b(2), b(19), b(1), b(12), b(3), b(18), b(16), b(4),
			c(4), c(16), c(19), c(18), c(12), c(3), c(2), c(1), a(5), a(14), a(17), a(6), b(6), b(17), b(3), b(2), b(14),
			b(16), b(4), b(5), c(6), c(4), c(5), c(14), c(3), c(2), c(16), c(17), u(1), a(17), a(5), a(11), a(16), a(1),
			a(18), a(19), a(7), b(17), b(1), b(5), b(19), b(18), b(16), b(7), b(11), c(7), c(16), c(18), c(11), c(19),
			c(1), c(17), c(5), u(2), a(4), a(16), a(6), a(5), a(17), a(10), a(2), a(14), b(6), b(14), b(2), b(8), b(10),
			b(3), b(9), b(17), b(4), b(16), b(5), c(14), c(9), c(4), c(5), c(10), c(17), c(8), c(16), c(3), c(2), c(6),
			u(1), a(18), a(14), a(17), a(4), a(16), a(2), a(12), a(19), a(1), a(5), a(3), b(14), b(1), b(17), b(19),
			b(12), b(2), b(3), b(18), b(4), b(16), b(5), c(5), c(1), c(16), c(3), c(18), c(2), c(12), c(4), c(19), c(14),
			c(17), a(17), a(16), a(1), a(19), a(5), a(18), b(17), b(19), b(1), b(18), b(16), b(5), c(5), c(18), c(1),
			c(19), c(16), c(17), u(1), a(10), a(2), a(1), a(9), a(3), a(18), a(8), a(19), a(12), a(4), a(16), b(10), b(1),
			b(12), b(2), b(19), b(8), b(9), b(3), b(18), b(4), b(16), c(4), c(16), c(12), c(3), c(8), c(18), c(9), c(19),
			c(10), c(2), c(1), a(1), a(16), a(7), a(18), a(19), a(11), b(1), b(19), b(16), b(18), b(7), b(11), c(11),
			c(18), c(7), c(19), c(1), c(16), a(6), a(5), a(17), a(1), a(16), b(6), b(19), b(1), b(18), b(17), b(16), b(5),
			c(18), c(16), c(17), c(1), c(5), c(19), c(6), a(11), a(15), a(7), b(11), b(7), b(15), c(15), c(18), c(1),
			c(7), c(16), c(11), u(1), a(2), a(1), a(4), a(16), a(13), a(7), a(3), a(19), a(12), a(18), a(11), b(2), b(12),
			b(1), b(4), b(18), b(16), b(19), b(3), b(13), b(7), b(11), c(11), c(18), c(7), c(3), c(13), c(12), c(19),
			c(2), c(1), c(4), c(16), u(3), a(5), a(15), a(16), a(4), a(13), a(7), a(3), a(11), b(4), b(5), b(11), b(16),
			b(7), b(3), b(13), b(15), c(11), c(15), c(13), c(2), c(3), c(4), c(14), c(17), c(16), c(7), c(5), u(2), a(6),
			a(11), a(17), a(14), a(4), a(16), a(2), a(5), b(14), b(6), b(5), b(17), b(16), b(2), b(3), b(4), b(7), b(13),
			b(11), c(5), c(13), c(11), c(4), c(2), c(3), c(14), c(7), c(17), c(16), c(6), a(1), a(18), a(19), a(16), b(1),
			b(19), b(18), b(16), c(16), c(19), c(18), c(1), u(3), a(5), a(11), a(16), a(7), a(18), a(15), b(5), b(16),
			b(18), b(7), b(11), b(15), c(15), c(11), c(7), c(1), c(18), c(16), c(17), c(5), u(3), a(4), a(16), a(11),
			a(15), a(13), a(18), a(3), a(7), b(4), b(3), b(16), b(7), b(11), b(18), b(13), b(15), c(7), c(15), c(13),
			c(12), c(3), c(2), c(1), c(18), c(4), c(16), c(11), a(5), a(11), a(16), b(5), b(16), b(7), b(11), b(15),
			c(15), c(11), c(17), c(16), c(7), c(5), c(6), a(11), a(7), a(13), a(15), b(13), b(11), b(15), b(7), c(15),
			c(3), c(2), c(13), c(4), c(16), c(7), c(11), a(6), a(5), a(17), b(6), b(7), b(17), b(16), b(5), b(11), c(11),
			c(5), c(17), c(7), c(16), c(6), a(15), b(15), c(15), c(9), c(13), a(8), a(12), a(19), a(10), a(2), a(1), b(8),
			b(12), b(19), b(2), b(10), b(1), c(10), c(2), c(1), c(12), c(19), c(8), a(12), a(19), a(2), a(1), b(12),
			b(19), b(1), b(2), c(2), c(1), c(19), c(12), a(19), a(1), b(19), b(1), c(1), c(19), u(3), a(9), a(13), a(7),
			a(15), a(3), b(7), b(9), b(13), b(3), b(15), c(15), c(3), c(7), c(0), c(8), c(9), c(13), u(3), a(9), a(3),
			a(13), a(7), a(18), a(15), b(9), b(3), b(7), b(13), b(18), b(15), c(15), c(18), c(8), c(12), c(9), c(3),
			c(13), c(7), a(3), a(18), a(13), a(7), a(15), b(3), b(18), b(13), b(7), b(15), c(15), c(12), c(19), c(3),
			c(18), c(13), c(7), a(18), a(7), a(15), b(18), b(7), b(15), c(15), c(19), c(18), c(7), a(19), a(0), a(8),
			a(12), b(8), b(0), b(12), b(19), c(0), c(8), c(12), c(19), u(2), a(6), a(5), a(17), a(16), a(1), a(11), b(6),
			b(17), b(1), b(18), b(16), b(7), b(5), b(11), c(7), c(11), c(5), c(16), c(1), c(18), c(17), c(6), u(2), a(4),
			a(6), a(14), a(10), a(5), b(6), b(10), b(0), b(9), b(14), b(4), b(5), c(6), c(14), c(0), c(4), c(9), c(10),
			c(5), u(1), a(19), a(12), a(0), a(8), b(0), b(8), b(19), b(12), c(0), c(8), c(12), c(19), u(1), a(0), a(8),
			a(12), a(19), a(18), a(9), a(3), b(19), b(0), b(12), b(8), b(9), b(3), b(18), c(9), c(3), c(18), c(19), c(0),
			c(8), c(12), a(8), a(12), a(19), a(9), a(3), a(18), b(8), b(19), b(12), b(3), b(9), b(18), c(9), c(3), c(18),
			c(8), c(12), c(19), a(12), a(19), a(3), a(18), b(12), b(19), b(18), b(3), c(3), c(18), c(12), c(19), a(19),
			a(18), b(19), b(18), c(18), c(19), u(1), a(12), a(19), a(10), a(2), a(1), a(14), a(8), a(17), b(8), b(12),
			b(19), b(10), b(2), b(1), b(14), b(17), c(14), c(17), c(2), c(8), c(12), c(1), c(10), c(19), a(19), a(2),
			a(1), a(14), a(17), a(12), b(12), b(19), b(2), b(1), b(17), b(14), c(14), c(17), c(1), c(12), c(19), c(2),
			a(12), a(19), a(3), a(18), a(13), a(7), b(12), b(19), b(3), b(18), b(7), b(13), c(13), c(7), c(12), c(19),
			c(3), c(18), a(19), a(18), a(7), b(19), b(18), b(7), c(7), c(19), c(18)};
		_ASSERTE(sizeof(x) / sizeof(x[0]) == p[sizeof(p) / sizeof(p[0]) - 1]); // make sure the number of nonzeros is right

		/*
		csc cols obtained using:

		sed "s/ //g" pnp_dls_mat.txt | awk 'BEGIN { FS = ","; row = 0; } { for(i = 1; i < NF; ++ i) { if($(i) != "0") { var = $(i); var_n = substr(var, 1, index(var, "(")); var_i = substr(var, index(var, "(") + 1, index(var, "-1)") - (index(var, "(") + 1)); print ((row + 1) * 10000 + i) " " row " " (i - 1) " " var_n (var_i - 1.0) ")"; } } ++ row; }' | sort -n > pnp_dls_triplets.txt
		cut -d " " -f 2 pnp_dls_triplets.txt | awk 'BEGIN { last_col = 0; buf = "static const int p[] = {0"; len = length(buf); cumsum = 0; } { if(last_col != $1) { last_col = $1; nstr = ", " cumsum; if(len + length(nstr) < 115) { buf = buf nstr; len += length(nstr); } else { print buf ","; buf = "\t" cumsum; len = length(buf) + 4; } } ++ cumsum; } END { print buf ", " cumsum "};"; }' > pnp_dls_cols_cumsums.txt

		csc rows obtained using:
		cut -d " " -f 3 pnp_dls_triplets.txt | awk 'BEGIN { buf = "static const int i[] = {"; len = length(buf); first = 1; } { if(first) { nstr = $1; first = 0; } else { nstr = ", " $1; } if(len + length(nstr) < 115) { buf = buf nstr; len += length(nstr); } else { print buf ","; buf = "\t" $1; len = length(buf) + 4; } } END { print buf "};"; }' > pnp_dls_rows.txt

		finally csc values obtained using:
		cut -d " " -f 4 pnp_dls_triplets.txt | awk 'BEGIN { buf = "const double x[] = {"; len = length(buf); first = 1; } { if(first) { nstr = $1; first = 0; } else { nstr = ", " $1; } if(len + length(nstr) < 115) { buf = buf nstr; len += length(nstr); } else { print buf ","; buf = "\t" $1; len = length(buf) + 4; } } END { print buf "};"; }' > pnp_dls_values.txt

		this form takes 231 lines (including spaces and comments), the packed assignment code takes 328 lines
		*/

		Eigen::MatrixXd C(120, 120);
		C.setZero();
		for(int c = 0, n = sizeof(p) / sizeof(p[0]) - 1; c < n; ++ c) {
			for(int t = p[c], e = p[c + 1]; t < e; ++ t) {
				int r = i[t];
				C(r, c) = x[t];
			}
		}
		// fill the matrix from the sparse data

#ifdef _DEBUG
		_ASSERTE(C == M.transpose()); // this is already transposed
#endif // _DEBUG

		return C;
	}

	template <class Derived0>
	static Eigen::Matrix<typename Derived0::Scalar, 3, 3> Hessian(const Eigen::Matrix<double, 20, 1> &f1coeff,
		const Eigen::Matrix<double, 20, 1> &f2coeff,
		const Eigen::Matrix<double, 20, 1> &f3coeff,
		const Eigen::MatrixBase<Derived0> &s)
	{
		// the 20D vector of monomials is
		// m = [0, s1^2 * s2, s1 * s2, s1 * s3, s2 * s3, s2^2 * s3, s2^3, ... 
		//      s1 * s3^2, s1, s3, s2, s2 * s3^2, s1^2, s3^2, s2^2, s3^3, ...
		//      s1 * s2 * s3, s1 * s2^2, s1^2 * s3, s1^3]'

		typedef double _T; // real
		typedef typename Derived0::Scalar _C; // complex or real?
		Eigen::Matrix<_C, 20, 1> Hs1;
		Hs1 << 0, _T(2) * s(0) * s(1), s(1), s(2), 0, 0, 0, s(2) * s(2), 1, 0, 0, 0, _T(2) * s(0),
			0, 0, 0, s(1) * s(2), s(1) * s(1), _T(2) * s(0) * s(2), _T(3) * s(0) * s(0);
		// deriv of m w.r.t. s1

		Eigen::Matrix<_C, 20, 1> Hs2;
		Hs2 << 0, s(0) * s(0), s(0), 0, s(2), _T(2) * s(1) * s(2), _T(3) * s(1) * s(1), 0, 0, 0, 1,
			s(2) * s(2), 0, 0, _T(2) * s(1), 0, s(0) * s(2), s(0) * _T(2) * s(1), 0, 0;
		// deriv of m w.r.t. s2

		Eigen::Matrix<_C, 20, 1> Hs3;
		Hs3 << 0, 0, 0, s(0), s(1), s(1) * s(1), 0, s(0) * _T(2) * s(2), 0, 1, 0, s(1) * _T(2) * s(2),
			0, _T(2) * s(2), 0, _T(3) * s(2) * s(2), s(0) * s(1), 0, s(0) * s(0), 0;
		// deriv of m w.r.t. s3

		Eigen::Matrix<_C, 3, 3> H;
		H << f1coeff.dot(Hs1), f1coeff.dot(Hs2), f1coeff.dot(Hs3),
			 0/*f2coeff.dot(Hs1)*/, f2coeff.dot(Hs2), f2coeff.dot(Hs3),
			 0/*f3coeff.dot(Hs1)*/, 0/*f3coeff.dot(Hs2)*/, f3coeff.dot(Hs3);
		_ASSERTE(fabs(H(0, 1) - f2coeff.dot(Hs1)) < 1e-10 * std::max(1.0, fabs(H(0, 1))));
		_ASSERTE(fabs(H(0, 2) - f3coeff.dot(Hs1)) < 1e-10 * std::max(1.0, fabs(H(0, 2))));
		_ASSERTE(fabs(H(1, 2) - f3coeff.dot(Hs2)) < 1e-10 * std::max(1.0, fabs(H(1, 2)))); // huge numbers here, need to track relative error
		H.template triangularView<Eigen::StrictlyLower>() =
			H.template triangularView<Eigen::StrictlyUpper>().transpose();
		// this is a symmetric matrix

		return H;
	}
};

/**
 *	@brief 3-point perspective solver implementation
 */
class CP3P_Solver_NewNew {
public:
	// 3D vector type is Eigen::Vector3d
	// quaternion type is Eigen::Quaterniond

	// todo - implement P3P
	static void Compute() // todo - add args as needed, add a return value (preferrably a quaternion and 3D vector pair, or a SE(3) or a Sim(3) pose)
	{
	}
};

// todo - once the above interface is finalized, fetch Gao's / Laurent's code and include it in two other classes (e.g. CP3P_Solver_Gao) with the same interface, add licenses as needed

// todo - once multiple implementations are there, compare precision / speed

/** @} */ // end of group

#endif // !__THREE_POINT_PERSPECTIVE_SOLVER_INCLUDED
