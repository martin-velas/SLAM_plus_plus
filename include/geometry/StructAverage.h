/*
								+----------------------------------+
								|                                  |
								|  *** 3D structure averaging ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|         StructAverage.h          |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __3D_STRUCTURE_AVERAGING_INCLUDED
#define __3D_STRUCTURE_AVERAGING_INCLUDED

/**
 *	@file geometry/StructAverage.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief rigid 3D structure average calculation from multiple observations
 */

#include <vector>
#include <algorithm> // std::min()
#include "geometry/Kabsch.h"

/** \addtogroup geom
 *	@{
 */

/**
 *	@brief calculates an average of 3D structure from multiple observations
 */
class CAverage_RigidStructure {
public:
	typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d_u; /**< @brief unaligned 3D vector type */
	// msvc requires unaligned types in std::vector in x86 builds

public:
	/**
	 *	@brief calculates average 3D structure from multiple (complete) observations
	 *
	 *	@tparam C3DPointConstIt is constatnt iterator for the input 3D points array
	 *	@tparam CStructContainer is type of the container for the average structure
	 *		(e.g. <tt>std::vector<Vector3d_u></tt>)
	 *
	 *	@param[in] p_begin_it is the iterator pointing to the first 3D observed point
	 *	@param[in] p_end_it is the iterator pointing to one past the last 3D observed point
	 *	@param[in] n_structure_size is number of points of the 3D structure
	 *	@param[out] r_avg_structure is a container for the averaged structure (the averaged
	 *		structure points are inserted there, rather than replacing the prior contents)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This assumes that the structure observations are in a contiguous range
	 *		of \f$N\f$ structure points given by the first two arguments. The structure
	 *		has a number of points \f$n\f$ given by the third argument. The number
	 *		of observations is \f$\frac{N}{n}\f$. This could easily work for incomplete
	 *		observations too, given that the correspondences are known, but the interface
	 *		would need to get a bit more complicated.
	 */
	template <class C3DPointConstIt, class CStructContainer>
	static void Calculate(C3DPointConstIt p_begin_it, C3DPointConstIt p_end_it,
		const size_t n_structure_size, CStructContainer &r_avg_structure) // throw(std::bad_alloc)
	{
		const size_t n_point_num = p_end_it - p_begin_it;
		C3DPointConstIt p_first_struct_it = p_begin_it + std::min(n_structure_size, n_point_num);

		std::vector<Vector3d_u> avg_structure(p_begin_it, p_begin_it + std::min(n_structure_size, n_point_num));
		size_t n_structure_observation_num = 1;

		Eigen::Vector3d v_center_0 = CAttitudeEstimator_Kabsch::v_Centroid(p_begin_it,
			p_begin_it + std::min(n_structure_size, n_point_num));
		// calculate the first centroid

		for(size_t i = n_structure_size, n = n_point_num; i < n; i += n_structure_size) {
			Eigen::Vector3d v_center_i = CAttitudeEstimator_Kabsch::v_Centroid(p_begin_it + i,
				p_begin_it + std::min(n_structure_size + i, n_point_num));
			// calculate the i-th centroid

			CAttitudeEstimator_Kabsch kabsch(v_center_0, v_center_i);
			for(size_t j = i, m = std::min(i + n_structure_size, n); j < m; ++ j)
				kabsch.ObservePair(*(p_begin_it + (j - i)), *(p_begin_it + j));
			Eigen::Matrix4d t_transform = kabsch.t_Estimate_Transform(false);
			// estimate the transform from the object observation i to object observation 0

			for(size_t j = i, m = std::min(i + n_structure_size, n); j < m; ++ j) {
				Eigen::Vector4d v_homog;
				v_homog.head<3>() = *(p_begin_it + j);
				v_homog(3) = 1.0;
				avg_structure[j - i] += (t_transform * v_homog).head<3>();
			}
			++ n_structure_observation_num;
			// accumulate average structure points (in the same coordinate frame)
		}

		for(size_t i = 0, n = avg_structure.size(); i < n; ++ i)
			avg_structure[i] /= double(n_structure_observation_num);
		// take an average

		v_center_0 = CAttitudeEstimator_Kabsch::v_Centroid(avg_structure.begin(), avg_structure.end());
		for(size_t i = 0, n = avg_structure.size(); i < n; ++ i) {
			avg_structure[i] -= v_center_0;
			/*printf("%f %f %f\n", avg_structure[i](0) + n_object_id % 10 - 4.5,
				avg_structure[i](1), avg_structure[i](2) - n_object_id / 10 - 2.5);*/ // verbose
		}
		// print the averaged structure points (place them behind the camera)

		r_avg_structure.insert(r_avg_structure.end(), avg_structure.begin(), avg_structure.end());
		// accumulate the averaged structure points
	}
};

/** @} */ // end of group

#endif // !__3D_STRUCTURE_AVERAGING_INCLUDED
