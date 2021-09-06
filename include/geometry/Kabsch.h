/*
								+----------------------------------+
								|                                  |
								|  ***   Kabsch's algorithm   ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|             Kabsch.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __KABSCH_RIGID_BODY_POSE_ESTIMATION_INCLUDED
#define __KABSCH_RIGID_BODY_POSE_ESTIMATION_INCLUDED

/**
 *	@file geometry/Kabsch.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief a simple, easy to use pose estimator class based on the Kabsch algorithm
 *
 *	@date 2017-07-04
 *
 *	Changed the order of arguments to match the rest of the geometry module. Renamed
 *	<tt>CKabschEstimator</tt> to \ref CAttitudeEstimator_Kabsch to make sure all the
 *	references to this code would need to be changed.
 *
 *	The arguments are target positions, source positions. The estimated transform is
 *	making the source points overlay the target ones, so that the order of arguments
 *	destination, source. This order is maintained throughout the geometry module.
 *
 */

/** \addtogroup geom
 *	@{
 */

#include "Eigen/Core"
#include "Eigen/SVD"
#include "Eigen/Geometry"
#include "slam/BlockMatrixBase.h" // DimensionCheck()

// inject the 8D vector type into Eigen namespace
namespace Eigen {

#ifndef HAVE_EIGEN_8D_DOUBLE_VECTOR
#define HAVE_EIGEN_8D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 8, 1> Vector8d; /**< @brief 8D vector type */
#endif // !HAVE_EIGEN_8D_DOUBLE_VECTOR

} // ~Eigen

/**
 *	@brief rigid body transformation estimator based on the Kabsch algorithm
 *
 *	This calculates transformation ([Rt] and optionally also s] that aligns two point
 *	clouds where the point correspondences are known.
 *
 *	The transformation estimation requires the knohledge of the point cloud centroids.
 *	To calculate them, one can use \ref v_Centroid().
 *	Afterwards, construct a new \ref CAttitudeEstimator_Kabsch::CAttitudeEstimator_Kabsch() and pass
 *	the centroids. Then use either \ref ObservePair() for a pair of two corresponding
 *	points or \ref ObserveStacked() for two stacked matrices filled with corresponding
 *	points.
 *	Finally, \ref t_Estimate_Transform() can be used to estimate the transform, based
 *	on all the data supplied.
 */
class CAttitudeEstimator_Kabsch {
protected:
	Eigen::Vector3d m_v_center_tar; /**< @brief center of the points in the target pose */
	Eigen::Vector3d m_v_center_src; /**< @brief center of the points in the initial pose */
	Eigen::Matrix3d m_t_covariance; /**< @brief covariance of the point eccentricities */
	double m_f_stddev2; /**< @brief standard deviation of the point eccentricities */

public:
	/**
	 *	@brief default constructor; sets up the body centroids
	 *
	 *	@param[in] r_v_centroid_tar is position of the centroid of the rigid body in the target pose
	 *	@param[in] r_v_centroid_src is position of the centroid of the rigid body in the initial pose
	 */
	CAttitudeEstimator_Kabsch(const Eigen::Vector3d &r_v_centroid_tar = Eigen::Vector3d::Zero(),
		const Eigen::Vector3d &r_v_centroid_src = Eigen::Vector3d::Zero())
	{
		Reset(r_v_centroid_tar, r_v_centroid_src);
	}

	/**
	 *	@brief calculates centroid of a point set
	 *
	 *	@tparam CConstIter is constant iterator class
	 *
	 *	@param[in] p_begin_it is iterator pointing to the first 3D point
	 *	@param[in] p_end_it is iterator pointing to the one past the last 3D point
	 *
	 *	@return Returns centroid of the given points.
	 */
	template <class CConstIter>
	static Eigen::Vector3d v_Centroid(CConstIter p_begin_it, CConstIter p_end_it)
	{
		if(p_begin_it == p_end_it)
			return Eigen::Vector3d::Zero();
		Eigen::Vector3d v_centroid = (*p_begin_it).template cast<double>();
		size_t n_count = p_end_it - p_begin_it;
		for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it)
			v_centroid += (*p_begin_it).template cast<double>();
		return v_centroid / double(n_count);
	}

	/**
	 *	@brief resets the estimation, sets up the body centroids
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_v_centroid_tar is position of the centroid of the rigid body in the target pose
	 *	@param[in] r_v_centroid_src is position of the centroid of the rigid body in the initial pose
	 */
	template <class Derived0, class Derived1>
	void Reset(const Eigen::MatrixBase<Derived0> &r_v_centroid_tar,
		const Eigen::MatrixBase<Derived1> &r_v_centroid_src)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_centroid_tar);
		DimensionCheck<Eigen::Vector3d>(r_v_centroid_src);

		m_v_center_tar = r_v_centroid_tar;
		m_v_center_src = r_v_centroid_src;
		m_t_covariance.setZero();
		m_f_stddev2 = 0;
	}

	/**
	 *	@brief calculates centroid of a stacked point set
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@param[in] r_v_stacked_pos is a \f$3 \times N\f$ matrix of positions of the
	 *		3D points (each column is a point)
	 *	@return Returns centroid of the given points.
	 */
	template <class Derived0>
	static Eigen::Vector3d v_CentroidStacked(const Eigen::MatrixBase<Derived0> &r_v_stacked_pos)
	{
		_ASSERTE(r_v_stacked_pos.rows() == 3);
		// dimension check

		return r_v_stacked_pos.template cast<double>().rowwise().mean().transpose();
		// and that's that
	}

	/**
	 *	@brief observation of many stacked corresponding 3D point pairs
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_v_stacked_tar is a \f$3 \times N\f$ matrix of positions of the
	 *		3D points in the target pose (each column is a point)
	 *	@param[in] r_v_stacked_src is a \f$3 \times N\f$ matrix of positions of the
	 *		3D points in the initial pose (each column is a point)
	 */
	template <class Derived0, class Derived1>
	void ObserveStacked(const Eigen::MatrixBase<Derived0> &r_v_stacked_tar,
		const Eigen::MatrixBase<Derived1> &r_v_stacked_src)
	{
		_ASSERTE(r_v_stacked_src.rows() == 3);
		_ASSERTE(r_v_stacked_tar.rows() == 3);
		_ASSERTE(r_v_stacked_tar.cols() == r_v_stacked_src.cols());
		// dimension check

		m_t_covariance.noalias() += (r_v_stacked_src.template cast<double>().colwise() -
			m_v_center_src) * (r_v_stacked_tar.template cast<double>().colwise() - m_v_center_tar).transpose();
		// accumulate the covariance matrix (always performed in double precision to avoid roundoff)

		m_f_stddev2 += (r_v_stacked_src.template cast<double>().colwise() - m_v_center_src).squaredNorm();
		// accumulate the standard deviation of one of the pose sets (needs to recalculate the difference
		// but saves allocating a new matrix for the intermediate)
	}

	/**
	 *	@brief observation of a corresponding 3D point pair
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_v_tar is position of the 3D point in the target pose
	 *	@param[in] r_v_src is position of the 3D point in the initial pose
	 */
	template <class Derived0, class Derived1>
	void ObservePair(const Eigen::MatrixBase<Derived0> &r_v_tar, const Eigen::MatrixBase<Derived1> &r_v_src)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_tar);
		DimensionCheck<Eigen::Vector3d>(r_v_src);

		Eigen::Vector3d v_vert_i = r_v_src.template cast<double>() - m_v_center_src;
		m_t_covariance.noalias() += v_vert_i * (r_v_tar.template cast<double>() - m_v_center_tar).transpose();
		// accumulate the covariance matrix (always performed in double precision to avoid roundoff)

		m_f_stddev2 += v_vert_i.squaredNorm();
		// accumulate the standard deviation of one of the pose sets
	}

	/**
	 *	@brief estimates the transform from the initial to the target pose
	 *	@param[in] b_recover_scale is scale recovery flag (if set, the scale is calculated,
	 *		otherwise it is assumed to be 1)
	 *	@return Returns the transformation matrix from the initial to the target pose.
	 */
	Eigen::Matrix4d t_Estimate_Transform(bool b_recover_scale)
	{
		return t_Estimate_Transform(m_v_center_tar, m_v_center_src,
			m_t_covariance, m_f_stddev2, b_recover_scale);
	}

	/**
	 *	@brief estimates the transform from the initial to the target pose
	 *	@param[in] b_recover_scale is scale recovery flag (if set, the scale is calculated,
	 *		otherwise it is assumed to be 1)
	 *	@return Returns the transformation from the initial to the target pose as an 8D vector
	 *		containing 3D translation, 4D quaternion rotation and 1D scale.
	 *	@note The quaternion is ordered as <tt>(x, y, z, w)</tt> whereas \ref Eigen::Quaternion
	 *		constructor takes scalar arguments in the order <tt>(w, x, y, z)</tt>.
	 */
	Eigen::Vector8d v_Estimate_Transform_Quat(bool b_recover_scale)
	{
		return v_Estimate_Transform_Quat(m_v_center_tar, m_v_center_src,
			m_t_covariance, m_f_stddev2, b_recover_scale);
	}

	/**
	 *	@brief estimates the transform from the initial to the target pose
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_centroid_tar is position of the centroid of the rigid body in the target pose
	 *	@param[in] r_v_centroid_src is position of the centroid of the rigid body in the initial pose
	 *	@param[in] r_t_covariance is covraiance matrix of the corresponding points
	 *		(relative to the centroid of each respective point set)
	 *	@param[in] b_recover_scale is scale recovery flag (if set, the scale is calculated,
	 *		otherwise it is assumed to be 1)
	 *	@return Returns the transformation matrix from the initial to the target pose.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static Eigen::Matrix4d t_Estimate_Transform(const Eigen::MatrixBase<Derived0> &r_v_centroid_tar,
		const Eigen::MatrixBase<Derived1> &r_v_centroid_src,
		const Eigen::MatrixBase<Derived2> &r_t_covariance,
		double f_pos_stddev_squared, bool b_recover_scale)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_centroid_tar);
		DimensionCheck<Eigen::Vector3d>(r_v_centroid_src);
		DimensionCheck<Eigen::Matrix3d>(r_t_covariance);

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(r_t_covariance.template cast<double>(),
			Eigen::ComputeFullU | Eigen::ComputeFullV);
		// calculate the SVD (always performed in double precision to avoid numerical instability

		Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
		double f_det = R.determinant();
		Eigen::Vector3d e(1, 1, (f_det < 0)? -1 : 1);
		// calculate determinant of V*U^T to disambiguate rotation sign

		if(f_det < 0)
			R.noalias() = svd.matrixV() * e.asDiagonal() * svd.matrixU().transpose();
		// compute the rotation part

		R = Eigen::Quaterniond(R).normalized().toRotationMatrix();
		// renormalize the rotation

		if(b_recover_scale) {
			double f_inv_scale = svd.singularValues().dot(e) / f_pos_stddev_squared;
			R *= f_inv_scale;
		}
		// calculate the scale

		Eigen::Matrix4d t_transform;
		t_transform.topLeftCorner<3, 3>() = R;
		t_transform.block<3, 1>(0, 3) = r_v_centroid_tar - R * r_v_centroid_src;
		t_transform.bottomRows<1>() << 0, 0, 0, 1;
		// put it in a single matrix

		return t_transform;
	}

	/**
	 *	@brief estimates the transform from the initial to the target pose
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_centroid_tar is position of the centroid of the rigid body in the target pose
	 *	@param[in] r_v_centroid_src is position of the centroid of the rigid body in the initial pose
	 *	@param[in] r_t_covariance is covraiance matrix of the corresponding points
	 *		(relative to the centroid of each respective point set)
	 *	@param[in] b_recover_scale is scale recovery flag (if set, the scale is calculated,
	 *		otherwise it is assumed to be 1)
	 *
	 *	@return Returns the transformation from the initial to the target pose as an 8D vector
	 *		containing 3D translation, 4D quaternion rotation and 1D scale.
	 *
	 *	@note The quaternion is ordered as <tt>(x, y, z, w)</tt> whereas \ref Eigen::Quaternion
	 *		constructor takes scalar arguments in the order <tt>(w, x, y, z)</tt>.
	 */
	template <class Derived0, class Derived1, class Derived2>
	Eigen::Vector8d v_Estimate_Transform_Quat(const Eigen::MatrixBase<Derived0> &r_v_centroid_tar,
		const Eigen::MatrixBase<Derived1> &r_v_centroid_src,
		const Eigen::MatrixBase<Derived2> &r_t_covariance,
		double f_pos_stddev_squared, bool b_recover_scale)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_centroid_tar);
		DimensionCheck<Eigen::Vector3d>(r_v_centroid_src);
		DimensionCheck<Eigen::Matrix3d>(r_t_covariance);

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(r_t_covariance.template cast<double>(),
			Eigen::ComputeFullU | Eigen::ComputeFullV);
		// calculate the SVD (always performed in double precision to avoid numerical instability

		Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
		double f_det = R.determinant();
		Eigen::Vector3d e(1, 1, (f_det < 0)? -1 : 1);
		// calculate determinant of V*U^T to disambiguate rotation sign

		if(f_det < 0)
			R.noalias() = svd.matrixV() * e.asDiagonal() * svd.matrixU().transpose();
		// compute the rotation part

		Eigen::Quaterniond rot = Eigen::Quaterniond(R).normalized();
		// renormalize the rotation

		Eigen::Vector8d v_transform;

		double f_scale;
		if(b_recover_scale) {
			f_scale = svd.singularValues().dot(e) / f_pos_stddev_squared;
			v_transform.head<3>() = r_v_centroid_tar - f_scale * rot._transformVector(r_v_centroid_src);
		} else {
			f_scale = 1;
			v_transform.head<3>() = r_v_centroid_tar - rot._transformVector(r_v_centroid_src);
		}
		// calculate the scale

		v_transform.head<7>().tail<4>() = rot.coeffs();
		v_transform(7) = f_scale;
		// put it in a single 8D trs vector

		return v_transform;
	}
};

/** @} */ // end of group

#endif // !__KABSCH_RIGID_BODY_POSE_ESTIMATION_INCLUDED
