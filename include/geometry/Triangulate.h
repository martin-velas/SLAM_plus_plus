/*
								+- --------------------------------+
								|                                  |
								|  ***   Triangulation algs   ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|          Triangulate.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __TRIANGULATION_ALGORITHMS_INCLUDED
#define __TRIANGULATION_ALGORITHMS_INCLUDED

/**
 *	@file geometry/Triangulate.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief 3D point triangulation algorithms
 */

/** \addtogroup geom
 *	@{
 */

#include "Eigen/Core"
#include "Eigen/LU" // dense matrix inverse

/**
 *	@brief a precise algorithm based on algebraic solution of the triangulation problem
 */
class CAlgebraicTriangulator {
public:
	/**
	 *	@brief triangulates a 3D point, based on the (near) intersection of two rays in 3D
	 *
	 *	@param[in] r_v_u is direction of the first ray
	 *	@param[in] r_v_v is direction of the second ray
	 *	@param[in] r_v_baseline is baseline vector (the position of the origin of the second
	 *		ray, assuming the first ray passes from the origin)
	 *
	 *	@return Returns the 3D point of where the two rays meet (either intersect or just pass by).
	 */
	Eigen::Vector3d operator ()(const Eigen::Vector3d &r_v_u, const Eigen::Vector3d &r_v_v,
		const Eigen::Vector3d &r_v_baseline) const
	{
		const Eigen::Vector3d &Pu = r_v_u; // just rename
		Eigen::Vector3d Pu_prime = r_v_v + r_v_baseline;
		Eigen::Vector3d PuPu_prime = Pu_prime - Pu;
		double m00 = PuPu_prime.dot(r_v_u), m01 = -r_v_u.dot(r_v_u), m02 = r_v_u.dot(r_v_v);
		double m10 = PuPu_prime.dot(r_v_v), m11 = -m02/*r_v_v.dot(r_v_u)*/, m12 = r_v_v.dot(r_v_v);
		// as per (3.3) in Xavier Armangue's thesis
		// todo - make two 2x3 matrices with (u, v) and (-u, v) and vectorize those calculations, should be much faster

		double f_alpha, f_beta;
		{
			Eigen::Matrix2d A;
			A << m01, m02,
				 m11, m12;
			Eigen::Vector2d b(m00, m10);
			Eigen::PartialPivLU<Eigen::Matrix2d> LU(A);
			Eigen::Vector2d x = LU.solve(-b);
			f_alpha = x(0);
			f_beta = x(1);

			/*double f_first_eq = m00 + f_alpha * m01 + f_beta * m02;
			double f_second_eq = m10 + f_alpha * m11 + f_beta * m12;*/
			// debug - those should both be null
		}
		// solve (that's me being lazy)

		Eigen::Vector3d Pr = Pu + r_v_u * f_alpha;
		Eigen::Vector3d Ps = Pu_prime + r_v_v * f_beta;
		Eigen::Vector3d v_reconst = (Ps + Pr) * .5;
		// calculate the final triangulation solution

		// calculating Pu: 3 add
		// calculating the matrices: 18 mul, 12 add | 15 mul, 10 add <- 1 elem is reused: m11 = -m02
		// calculating alpha: 4 mul, 2 add, 1 div
		// calculating beta: 1 mul, 1 add, 1 div
		// calculating Pr, Ps: 6 mul, 6 add
		// calculating midpoint: 3 mul, 3 add
		// -------------------------------------
		// 27 add, 32 mul, 2 div | 31 add, 38 mul, 2 div

		return v_reconst;
	}
};

/**
 *	@brief a fast algorithm based on geometric solution of ray-plane intersection
 *
 *	This works by constructing a plane incident with the left ray and at the same time
 *	with the normal as close to the baseline as possible. This plane can then be intersected
 *	by the right ray, to yield solution on the right ray. This solution can then be
 *	perpendicularly projected onto the left ray, and a midpoint of those two is returned.
 *
 *	@note This works for general cameras and is slightly faster than the algebraic method,
 *		with a very slightly worse precision.
 */
class CRayPlaneTriangulator {
public:
	/**
	 *	@copydoc CAlgebraicTriangulator::operator ()
	 */
	Eigen::Vector3d operator ()(const Eigen::Vector3d &r_v_u, const Eigen::Vector3d &r_v_v,
		const Eigen::Vector3d &r_v_baseline) const
	{
		double f_inv_len2_v = 1 / r_v_v.squaredNorm();
		Eigen::Vector3d v_right_perp = r_v_baseline - r_v_v * (r_v_baseline.dot(r_v_v) * f_inv_len2_v); // not normalized, but works just as well
		//double f_dot = r_v_v.dot(v_left_perp); // debug - should be zero
		// baseline turned to be orthogonal to u (Gram Schmidt)

		//Plane3f t_right_plane = Plane3f(r_v_baseline, v_right_perp, plane::from_position_normal);
		const Eigen::Vector3d &v_right_plane_normal = v_right_perp; // just rename
		double f_right_plane_dist = /*-*/r_v_baseline.dot(v_right_perp); // save a negation
		// a plane going through left ray, orthogonal to the triangle given by the centers
		// of projection and the triangulated point

		Eigen::Vector3d v_reconstL;
		{
			//t_right_plane.Intersect_Ray(v_reconstL, Vector3f(0, 0, 0), r_v_u);
			// general ray-plane intersection

			double t = v_right_plane_normal.dot(r_v_u);
			v_reconstL = r_v_u * (f_right_plane_dist / /*-*/t); // save a matching negation
			// ray-plane intersection for a ray passing through origin
		}
		// intersect the plane with the ray towards the observed point from the left camera
		// gains the reconstruction in the left space

		Eigen::Vector3d v_reconstR = r_v_v * (r_v_v.dot(v_reconstL - r_v_baseline) * f_inv_len2_v) + r_v_baseline; // u dot u is reused from calculating v_left_perp ;)
		// second intersection (the closest point on v must be on the intersection of v and a line
		// going through the left point and being perpedicular to v, so this is a projection rather
		// than a second plane intersection)
		// shift the left image by baseline to be in the right space, project onto v, shift back

		Eigen::Vector3d v_reconst = (v_reconstL + v_reconstR) * .5;
		// midpoint

		// precalculation of reciprocal squared norm of u: 3 mul, 2 add, 1 div
		// denorm gramm-schmidt (reusing rsqr u): 7 mul, 5 add
		// plane construction: 3 mul, 2 add
		// ray-plane intersection: 6 mul, 2 add, 1 div (ray origin is at O)
		// second intersection (reusing rsqr u): 7 mul, 8 add
		// midpoint: 3 mul, 3 add
		// -----------------------
		// 22 add, 29 mul, 2 div, or 22 add, 27 mul, 3 div if reusing sqr u instead of rsqr u

		return v_reconst;
	}
};

/**
 *	@brief a fast algorithm for horizontally aligned cameras
 */
class CHorizAlignedTriangulator {
public:
	/**
	 *	@copydoc CAlgebraicTriangulator::operator ()
	 */
	Eigen::Vector3d operator ()(const Eigen::Vector3d &r_v_u, const Eigen::Vector3d &r_v_v,
		const Eigen::Vector3d &r_v_baseline) const
	{
		double f_disparity_x = r_v_u(0) - r_v_v(0); // the rotations are the same so those still correspond to the 2D observation
		double f_baseline_x = r_v_baseline(0);

		return r_v_u * (f_baseline_x / f_disparity_x);

		// disparity: 1 add
		// triangulation: 3 mul, 1 div
		// -----------------------
		// 1 add, 3 mul, 1 div
	}
};

/** @} */ // end of group

#endif // !__TRIANGULATION_ALGORITHMS_INCLUDED
