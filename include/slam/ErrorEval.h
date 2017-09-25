/*
								+-----------------------------------+
								|                                   |
								|  ***  SLAM Error Evaluation  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|            ErrorEval.h            |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SLAM_ERROR_EVALUATOR_INCLUDED
#define __SLAM_ERROR_EVALUATOR_INCLUDED

/**
 *	@file include/slam/ErrorEval.h
 *	@brief implementation of error evaluation as in Sturm, "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012
 *	@author -tHE SWINe-
 *	@date 2015-12-20
 */

#include "slam/2DSolverBase.h"
#include "slam/3DSolverBase.h"
#include "slam/Sim3SolverBase.h"

/**
 *	@brief implementation of error evaluation as in Sturm,
 *		"A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012
 *
 *	@tparam _b_2D_problem is 2D flag (default not set - 3D)
 *	@tparam _b_use_degrees is rotational errors in degrees flag
 *		(if set will use degrees (default); if cleared will use radians)
 *	@tparam CPoseOpsImpl is class implementing operations on the poses
 *		(compose, inverse compose and inverse)
 */
template <const bool _b_2D_problem = false, const bool _b_use_degrees = true,
	class CPoseOpsImpl = typename CChooseType<C2DJacobians, C3DJacobians, _b_2D_problem>::_TyResult>
class CErrorEvaluation {
public:
	/**
	 *	@brief parameters stored as enum
	 */
	enum {
		b_2D_problem = _b_2D_problem, /**< @brief 2D flag */
		b_use_degrees = _b_use_degrees, /**< @brief degrees flag */
		n_pose_dimension = (b_2D_problem)? 3 : 6, /**< @brief vertex dimension */
		n_pose_pos_components = (b_2D_problem)? 2 : 3, /**< @brief number of position components in a pose */
		n_pose_rot_components = n_pose_dimension - n_pose_pos_components /**< @brief number of rotation components in a pose */
	};

	typedef CPoseOpsImpl _TyPoseOpsImpl; /**< @brief class implementing operations on the poses */
	typedef Eigen::Matrix<double, n_pose_dimension, 1> _TyVector; /**< @brief vector type */
	typedef Eigen::Matrix<double, n_pose_dimension, n_pose_dimension> _TyMatrix; /**< @brief matrix type */
	typedef Eigen::Matrix<double, n_pose_dimension, 1, Eigen::DontAlign> _TyVectorUnalign; /**< @brief unaligned vector type */

public:
	/**
	 *	@brief loads a set of poses from a file
	 *
	 *	@param[in] p_s_filename is null-terminated string containing input file name
	 *	@param[out] r_dest is reference to a vector of poses to be filled with the data from the file
	 *	@param[in] b_inverse_poses is pose inversion flag (if set, the poses are loaded inverted)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static bool Load_PoseSet(const char *p_s_filename,
		std::vector<_TyVectorUnalign> &r_dest, bool b_inverse_poses = false) // throw(std::bad_alloc)
	{
		r_dest.clear();
		// !!

		FILE *p_fr;
		if(!(p_fr = fopen(p_s_filename, "r")))
			return false;
		while(!feof(p_fr)) {
			_TyVectorUnalign v;
			switch((n_pose_dimension == 3)? fscanf(p_fr, "%lf %lf %lf\n", &v(0), &v(1), &v(2)) :
				fscanf(p_fr, "%lf %lf %lf %lf %lf %lf\n", &v(0), &v(1), &v(2), &v(3), &v(4), &v(5))) {
			case n_pose_dimension:
				if(b_inverse_poses)
					_TyPoseOpsImpl::Pose_Inverse(v, v);
				r_dest.push_back(v);
				break;
			case 0:
				break; // eof?
			default:
				fclose(p_fr);
				return false;
			}
		}
		if(ferror(p_fr)) {
			fclose(p_fr);
			return false;
		}
		fclose(p_fr);

		return true;
	}

	/**
	 *	@brief computes a difference of two pose sets using
	 *		different metrics, prints results to stdout
	 *
	 *	@param[in] p_s_estimated_file is null-terminated string containing name of file with estimated poses
	 *	@param[in] p_s_ground_truth_file is null-terminated string containing name of file with ground truth poses
	 *	@param[in] b_up_to_scale is scale flag (if set, scale does not matter and is estimated and matched)
	 *	@param[out] b_inverse_poses is flag to invert the pose sets before comparison
	 *
	 *	The poses are represented as 3D position + axis-angle rotation.
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note All the angular errors are reported in degrees.
	 */
	static bool Compute_AllErrors(const char *p_s_estimated_file,
		const char *p_s_ground_truth_file, bool b_up_to_scale, bool b_inverse_poses = true) // throw(std::bad_alloc)
	{
		std::vector<_TyVectorUnalign> poses_a, poses_b;
		Eigen::Vector6d v_rmse;
		return Load_PoseSet(p_s_estimated_file, poses_a) && // estimated stored as inverse for viewing
			Load_PoseSet(p_s_ground_truth_file, poses_b, b_inverse_poses) && // ground truth could be stored as inverse or direct
			Compute_AllErrors(poses_a, poses_b, b_up_to_scale, v_rmse, true);
	}

	/**
	 *	@brief computes a difference of two pose sets using different metrics
	 *
	 *	@param[in] est_vertices is a vector of estimated camera poses
	 *	@param[in] r_gt_vertices is vector of ground truth camera poses
	 *	@param[in] b_up_to_scale is scale flag (if set, scale does not
	 *		matter and is estimated and matched)
	 *	@param[out] r_v_RMSE is vector filled with RMSE errors
	 *		(translation, rotation pairs for ATE, RPE and RPE all to all)
	 *	@param[in] b_stdout_output is stdout output flag (if set,
	 *		the errors are also printed to stdout; default not set)
	 *
	 *	The poses are represented as 3D position + axis-angle rotation.
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note All the angular errors are reported in degrees.
	 */
	static bool Compute_AllErrors(std::vector<_TyVectorUnalign> est_vertices, // copy intended
		const std::vector<_TyVectorUnalign> &r_gt_vertices, bool b_up_to_scale,
		Eigen::Vector6d &r_v_RMSE, bool b_stdout_output = false) // throw(std::bad_alloc)
	{
		if(est_vertices.size() != r_gt_vertices.size()) {
			fprintf(stderr, "error: the number of poses in the estimated trajectory ("
				PRIsize ") does not correspond to the ground truth (" PRIsize ")\n",
				est_vertices.size(), r_gt_vertices.size());
			return false;
		}
		// the sizes must match

		Align_PoseSets(est_vertices, r_gt_vertices, b_up_to_scale);
		/*std::pair<_TyVector, double> t_transform = v_Align_PoseSets(r_est_vertices,
			r_gt_vertices, b_up_to_scale); // transform * est approaches gt
		// compute a rigid transformation that puts ground truth vertices close to the solution vertices

		std::vector<_TyVectorUnalign> vertices(r_est_vertices.size());
		for(size_t i = 0, n = r_est_vertices.size(); i < n; ++ i) {
			_TyVector v_aligned_pose = r_est_vertices[i];
			if(b_up_to_scale)
				v_aligned_pose.template head<3>() *= t_transform.second; // apply scale
			_TyPoseOpsImpl::Relative_to_Absolute(t_transform.first, v_aligned_pose, v_aligned_pose); // arg order! relative pose is on the left!
			vertices[i] = v_aligned_pose;
		}
		// apply transform on the estimated poses to align them with the ground truth*/

		/*FILE *p_fw;
		if((p_fw = fopen("solution_SE3_gt-camera-poses_est-aligned.txt", "w"))) {
			for(size_t i = 0, n = vertices.size(); i < n; ++ i) {
				_TyVector v_aligned_pose = vertices[i];
				v_aligned_pose = v_Invert_SE3_Pose(v_aligned_pose); // invert for the viewer
				fprintf(p_fw, "%.10f %.10f %.10f %g %g %g\n", v_aligned_pose(0), v_aligned_pose(1),
					v_aligned_pose(2), v_aligned_pose(3), v_aligned_pose(4), v_aligned_pose(5));
			}
			fclose(p_fw);
		}
		if((p_fw = fopen("solution_SE3_gt-inverse.txt", "w"))) {
			for(size_t i = 0, n = vertices.size(); i < n; ++ i) {
				_TyVector v_aligned_pose = r_gt_vertices[i];
				v_aligned_pose = v_Invert_SE3_Pose(v_aligned_pose); // invert for the viewer
				fprintf(p_fw, "%.10f %.10f %.10f %g %g %g\n", v_aligned_pose(0), v_aligned_pose(1),
					v_aligned_pose(2), v_aligned_pose(3), v_aligned_pose(4), v_aligned_pose(5));
			}
			fclose(p_fw);
		}*/
		// debug - write the output for the graph viewer

		/*t_transform.first = Eigen::Vector6d::Zero();
		t_transform.second = 1.0;*/
		// no transform, we already put the vertices into a common frame

		double f_pos_ate, f_rot_ate, f_pos2_ate, f_rot2_ate;
		double f_pos_rpe, f_rot_rpe, f_pos2_rpe, f_rot2_rpe;
		double f_pos_rpe_aa, f_rot_rpe_aa, f_pos2_rpe_aa, f_rot2_rpe_aa;
		// results of different error metrics, direct and squared

		Compute_AbsoluteTrajectoryError(est_vertices, r_gt_vertices,
			f_pos_ate, f_rot_ate, f_pos2_ate, f_rot2_ate);
		// calculate the ATEs

		Compute_RelativePoseError(est_vertices, r_gt_vertices, 1,
			f_pos_rpe, f_rot_rpe, f_pos2_rpe, f_rot2_rpe);
		// calculate the RPEs

		Compute_RelativePoseError_AllToAll(est_vertices, r_gt_vertices,
			f_pos_rpe_aa, f_rot_rpe_aa, f_pos2_rpe_aa, f_rot2_rpe_aa);
		// calculate the RPE-AAs

		if(b_stdout_output)
			printf("cumulative ATE of the aligned poses is %f, %f\n", f_pos_ate, f_rot_ate);

		const size_t n = r_gt_vertices.size();
		f_pos_ate /= n;
		f_rot_ate /= n;
		f_pos2_ate /= n;
		f_rot2_ate /= n;

		if(b_stdout_output) {
			printf("per vert. ATE of the aligned poses is %f, %f\n", f_pos_ate, f_rot_ate);
			printf("RMSE ATE of the aligned poses is %f, %f\n", sqrt(f_pos2_ate), sqrt(f_rot2_ate));

			printf("\ncumulative RPE of the aligned poses is %f, %f\n", f_pos_rpe, f_rot_rpe);
		}

		f_pos_rpe /= n;
		f_rot_rpe /= n;
		f_pos2_rpe /= n;
		f_rot2_rpe /= n;

		if(b_stdout_output) {
			printf("per vert. RPE of the aligned poses is %f, %f\n", f_pos_rpe, f_rot_rpe);
			printf("RMSE RPE of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe), sqrt(f_rot2_rpe));

			printf("\ncumulative RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa, f_rot_rpe_aa);
		}

		f_pos_rpe_aa /= n;
		f_rot_rpe_aa /= n;
		f_pos2_rpe_aa /= n;
		f_rot2_rpe_aa /= n;

		if(b_stdout_output) {
			printf("per vert. RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa, f_rot_rpe_aa);
			printf("RMSE RPE all-to-all of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_aa), sqrt(f_rot2_rpe_aa));

			printf("note that all the rotation errors are in %s\n", (b_use_degrees)? "degrees" : "radians");
		}

		r_v_RMSE << sqrt(f_pos2_ate), sqrt(f_rot2_ate), sqrt(f_pos2_rpe),
			sqrt(f_rot2_rpe), sqrt(f_pos2_rpe_aa), sqrt(f_rot2_rpe_aa);
		// put the errors in the vector

		return true;
	}

	static void Compute_RelativePoseError_AllToAll(const std::vector<_TyVectorUnalign> &r_est_vertices,
		const std::vector<_TyVectorUnalign> &r_gt_vertices, double &r_f_translation_error, double &r_f_rotation_error,
		double &r_f_squared_translation_error, double &r_f_squared_rotation_error)
	{
		r_f_translation_error = 0;
		r_f_rotation_error = 0;
		r_f_squared_translation_error = 0;
		r_f_squared_rotation_error = 0;

		_ASSERTE(r_est_vertices.size() == r_gt_vertices.size());
		for(size_t n_delta = 1, n = r_est_vertices.size(); n_delta < n; ++ n_delta) {
			double f_translation_error, f_rotation_error, f_squared_translation_error, f_squared_rotation_error;
			Compute_RelativePoseError(r_est_vertices, r_gt_vertices, n_delta, f_translation_error,
				f_rotation_error, f_squared_translation_error, f_squared_rotation_error);
			r_f_translation_error += f_translation_error;
			r_f_rotation_error += f_rotation_error;
			r_f_squared_translation_error += f_squared_translation_error;
			r_f_squared_rotation_error += f_squared_rotation_error;
		}
		// calculate RPE for all the possible deltas

		r_f_translation_error /= double(r_est_vertices.size());
		r_f_rotation_error /= double(r_est_vertices.size());
		r_f_squared_translation_error /= double(r_est_vertices.size());
		r_f_squared_rotation_error /= double(r_est_vertices.size());
		// divide by n
	}

	static void Compute_RelativePoseError(const std::vector<_TyVectorUnalign> &r_est_vertices,
		const std::vector<_TyVectorUnalign> &r_gt_vertices, size_t n_delta,
		double &r_f_translation_error, double &r_f_rotation_error,
		double &r_f_squared_translation_error, double &r_f_squared_rotation_error)
	{
		_ASSERTE(n_delta > 0); // ...

		r_f_translation_error = 0;
		r_f_rotation_error = 0;
		r_f_squared_translation_error = 0;
		r_f_squared_rotation_error = 0;

		const double f_deg_per_rad = 180 / M_PI;
		// number of degrees per one radian (for angular unit conversion)

		_ASSERTE(r_est_vertices.size() == r_gt_vertices.size());
		for(size_t i = n_delta, n = r_est_vertices.size(); i < n; ++ i) {
			_TyVector v_gt_pose_prev = r_gt_vertices[i - n_delta]; // one from n_delta vertices ago
			_TyVector v_gt_pose_cur = r_gt_vertices[i];
			_TyVector v_est_pose_prev = r_est_vertices[i - n_delta];
			_TyVector v_est_pose_cur = r_est_vertices[i];
			// get the previous and current vertices

			_TyVector v_edge_est, v_edge_gt;
			_TyPoseOpsImpl::Absolute_to_Relative(v_est_pose_cur, v_est_pose_prev, v_edge_est);
			_TyPoseOpsImpl::Absolute_to_Relative(v_gt_pose_cur, v_gt_pose_prev, v_edge_gt);
			// calculate edges between those vertices

			_TyVector v_error;
			_TyPoseOpsImpl::Absolute_to_Relative(v_edge_est, v_edge_gt, v_error);
			double f_trans_error2 = v_error.template head<n_pose_pos_components>().squaredNorm();
			r_f_translation_error += sqrt(f_trans_error2);
			r_f_squared_translation_error += f_trans_error2;
			double f_rot_error = f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm());
			if(b_use_degrees)
				f_rot_error *= f_deg_per_rad;
			r_f_rotation_error += f_rot_error;
			r_f_squared_rotation_error += f_rot_error * f_rot_error;
			// calculate the errors
		}
		// calculate RPE errors
	}

	/**
	 *	@brief computes absolute trajectory error of two pose sets
	 *
	 */
	static void Compute_AbsoluteTrajectoryError(const std::vector<_TyVectorUnalign> &r_est_vertices,
		const std::vector<_TyVectorUnalign> &r_gt_vertices, double &r_f_translation_error, double &r_f_rotation_error,
		double &r_f_squared_translation_error, double &r_f_squared_rotation_error)
	{
		r_f_translation_error = 0;
		r_f_rotation_error = 0;
		r_f_squared_translation_error = 0;
		r_f_squared_rotation_error = 0;

		const double f_deg_per_rad = 180 / M_PI;
		// number of degrees per one radian (for angular unit conversion)

		_ASSERTE(r_est_vertices.size() == r_gt_vertices.size());
		for(size_t i = 0, n = r_est_vertices.size(); i < n; ++ i) {
			_TyVector v_error;
			_TyPoseOpsImpl::Absolute_to_Relative(r_est_vertices[i], r_gt_vertices[i], v_error);
			double f_trans_error2 = v_error.template head<n_pose_pos_components>().squaredNorm();
			r_f_translation_error += sqrt(f_trans_error2);
			r_f_squared_translation_error += f_trans_error2;
			double f_rot_error = f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm());
			if(b_use_degrees)
				f_rot_error *= f_deg_per_rad;
			r_f_rotation_error += f_rot_error;
			r_f_squared_rotation_error += f_rot_error * f_rot_error;
			// calculate the errors
		}
		// calculate ATE errors
	}

	/**
	 *	@brief rigidly aligns two sets of poses
	 *
	 *	@param[in,out] r_est_vertices is a set of vertices to be aligned
	 *	@param[in] r_gt_vertices is a set of vertices to align to
	 *	@param[in] b_up_to_scale is scale flag (if set, scale does not matter
	 *		and is estimated and matched)
	 */
	static void Align_PoseSets(std::vector<_TyVectorUnalign> &r_est_vertices,
		const std::vector<_TyVectorUnalign> &r_gt_vertices, bool b_up_to_scale)
	{
		_ASSERTE(r_est_vertices.size() == r_gt_vertices.size()); // the sizes must match

		std::pair<_TyVector, double> t_transform = v_Align_PoseSets(r_est_vertices,
			r_gt_vertices, b_up_to_scale); // transform * est approaches gt
		// compute a rigid transformation that puts solution vertices close to the ground truth vertices

		for(size_t i = 0, n = r_est_vertices.size(); i < n; ++ i) {
			_TyVector v_aligned_pose = r_est_vertices[i];
			if(b_up_to_scale)
				v_aligned_pose.template head<3>() *= t_transform.second; // apply scale
			_TyPoseOpsImpl::Relative_to_Absolute(t_transform.first, v_aligned_pose, v_aligned_pose); // arg order! relative pose is on the left!
			r_est_vertices[i] = v_aligned_pose;
		}
		// apply transform on the estimated poses to align them with the ground truth
	}

	/**
	 *	@brief rigidly aligns two sets of poses
	 *
	 *	This calculates such a relative pose <tt>P</tt> and scale <tt>s</tt>, such that:
	 *
	 *	@code
	 *	_TyVector v_pose;
	 *	Relative_to_Absolute(P, r_vertices[i] * s, v_pose); // P is on the left!
	 *	double f_error = (r_tar_vertices[i].head<3>() - v_pose.head<3>()).norm();
	 *	@endcode
	 *
	 *	The error in <tt>f_error</tt> is minimized.
	 *
	 *	@param[in] r_vertices is a set of vertices to be aligned
	 *	@param[in] r_tar_vertices is a set of vertices to align to
	 *	@param[in] b_recover_scale is scale recovery flag (if set, scale is calculated otherwise it is set to 1.0)
	 *
	 *	@return Returns a relative pose that rigidly aligns the two given sets of poses.
	 *
	 *	@note This requires the two sets of poses to have the corresponding vertices stored under the same index.
	 */
	static std::pair<_TyVectorUnalign, double> v_Align_PoseSets(const std::vector<_TyVectorUnalign> &r_vertices,
		const std::vector<_TyVectorUnalign> &r_tar_vertices, bool b_recover_scale = true)
	{
		_ASSERTE(r_tar_vertices.size() == r_vertices.size());
		const size_t n = std::min(r_vertices.size(), r_tar_vertices.size());

		Eigen::Vector3d v_center_tar3 = Eigen::Vector3d::Zero(), v_center3 = Eigen::Vector3d::Zero();
		for(size_t i = 0; i < n; ++ i) {
			v_center_tar3.template head<n_pose_pos_components>() += r_tar_vertices[i].template head<n_pose_pos_components>();
			v_center3.template head<n_pose_pos_components>() += r_vertices[i].template head<n_pose_pos_components>();
		}
		v_center_tar3.template head<n_pose_pos_components>() /= double(n);
		v_center3.template head<n_pose_pos_components>() /= double(n);
		// calculate centers of positions, potentially extend to 3D

		double /*f_sd2_tar = 0,*/ f_sd2 = 0; // only one of those is really needed
		Eigen::Matrix3d t_cov = Eigen::Matrix3d::Zero();
		for(size_t i = 0; i < n; ++ i) {
			Eigen::Vector3d v_vert_i_tar = Eigen::Vector3d::Zero(), v_vert_i = Eigen::Vector3d::Zero();
			v_vert_i_tar.template head<n_pose_pos_components>() = r_tar_vertices[i].template head<n_pose_pos_components>() -
				v_center_tar3.template head<n_pose_pos_components>();
			v_vert_i.template head<n_pose_pos_components>() = r_vertices[i].template head<n_pose_pos_components>() -
				v_center3.template head<n_pose_pos_components>();
			// get both vertices, potentially extend them to 3D

			if(b_recover_scale) {
				f_sd2 += v_vert_i.squaredNorm();
				//f_sd2_tar += v_vert_i_tar.squaredNorm();
			}
			// accumulate squared standard deviation only one of those is really needed

			t_cov.noalias() += v_vert_i * v_vert_i_tar.transpose();
			// accumulate
		}
		// calculate the covariance matrix (always performed in double precision to avoid roundoff)

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(t_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
		// calculate the SVD

		Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
		double f_det = R.determinant();
		Eigen::Vector3d e(1, 1, (f_det < 0)? -1 : 1);
		// calculate determinant of V*U^T to disambiguate rotation sign

		if(f_det < 0)
			R.noalias() = svd.matrixV() * e.asDiagonal() * svd.matrixU().transpose();
		// compute the rotation part

		R = Eigen::Quaterniond(R).normalized().toRotationMatrix();
		// renormalize the rotation (not needed but gives slightly more orthogonal transformations)

		//double f_scale = (b_recover_scale)? svd.singularValues().dot(e) / f_sd2_tar : 1;
		double f_inv_scale = (b_recover_scale)? svd.singularValues().dot(e) / f_sd2 : 1; // only one of those is needed
		// calculate the scale

		Eigen::Vector3d v_translation3 = v_center_tar3 - (R * v_center3) * f_inv_scale;
		// want to align center with ground truth

		Eigen::Vector6d v_rel_pose6;
		v_rel_pose6.head<3>() = v_translation3;
		Eigen::VectorBlock<Eigen::Vector6d, 3> v_rot_part = v_rel_pose6.template tail<3>(); // g++ requires a temporary
		C3DJacobians::Quat_to_AxisAngle(Eigen::Quaterniond(R), v_rot_part);
		// make a pose that relates the two sets of poses

		_TyVectorUnalign v_rel_pose;
		if(b_2D_problem) {
			Eigen::Vector3d v_rel_pose3;
			v_rel_pose3.template head<n_pose_pos_components>() = v_rel_pose6.template head<n_pose_pos_components>(); // compiles for both SE(2) and SE(3)
			v_rel_pose3.template tail<n_pose_rot_components>()(0) = v_rel_pose6(5); // want in-plane rotation // compiles for both SE(2) and SE(3), only makes sense in SE(2)
			v_rel_pose.template head<3>() = v_rel_pose3; // compiles for both SE(2) and SE(3), only makes sense in SE(2)
		} else
			v_rel_pose.template head<n_pose_dimension>() = v_rel_pose6.template head<n_pose_dimension>(); // compiles for both SE(2) and SE(3), only makes sense in SE(3)
		// convert to the current coordinate system, pay mind to both branches actually having
		// to compile in both 2D and 3D (but only the respective branch is taken at runtime)

		return std::make_pair(v_rel_pose, f_inv_scale);
	}

	/**
	 *	@brief in SE(2) the angular errors may be outside the \f$[-2\pi, 2\pi]\f$ range and need to be clamped
	 *	@param[in] f_error_radians is angular error
	 *	@return Returns angular error modulo \f$2\pi\f$.
	 */
	static inline double f_ClampAngularError_2Pi_Abs(double f_error_radians)
	{
		if(b_2D_problem) {
			f_error_radians = fmod(f_error_radians, 2 * M_PI);
			f_error_radians = std::min(fabs(f_error_radians),
				std::min(fabs(f_error_radians - 2 * M_PI), fabs(f_error_radians + 2 * M_PI)));
		} else {
			_ASSERTE(f_error_radians >= 0 && f_error_radians <= 2 * M_PI);
			// in SE(3) this should not generally happen as the error is calculated
			// using quaternion / matrix algebra and should clamp itself
		}
		return f_error_radians;
	}

	/**
	 *	@brief runs a simple unit test
	 */
	static void Test()
	{
		const _TyVectorUnalign p_data[] = {
			_v(-0.709599, 0.184495, 3.19115, 0.0275248, 2.6636, -1.56961), _v(-0.824547, 0.227437, 3.14004, 0.117781, 2.62208, -1.55031), _v(-0.77987, 0.226588, 3.13313, 0.140778, 2.61834, -1.5444),
			_v(-0.713445, 0.188096, 3.13314, 0.160766, 2.62645, -1.52423), _v(-0.718561, 0.18813, 3.1256, 0.176283, 2.60614, -1.51154), _v(-0.663762, 0.25265, 3.14144, 0.196488, 2.58907, -1.53616),
			_v(-0.585596, 0.285537, 3.1545, 0.21356, 2.57937, -1.55484), _v(-0.598285, 0.194904, 3.144, 0.221208, 2.56852, -1.51231), _v(-0.482924, 0.0894491, 3.1297, 0.225323, 2.60039, -1.4807),
			_v(-0.461044, 0.0893305, 3.07976, 0.234396, 2.59055, -1.47089), _v(-0.392583, 0.164183, 3.01948, 0.247927, 2.58197, -1.49426), _v(-0.323704, 0.186389, 2.95679, 0.282988, 2.59548, -1.49884),
			_v(-0.378033, 0.180106, 2.89737, 0.313888, 2.58352, -1.46965), _v(-0.431318, 0.238246, 2.82175, 0.343274, 2.55151, -1.46014), _v(-0.459872, 0.285352, 2.7492, 0.373212, 2.52729, -1.45425),
			_v(-0.459518, 0.389214, 2.68519, 0.419235, 2.49506, -1.48175), _v(-0.486618, 0.490254, 2.60166, 0.483783, 2.45171, -1.49233), _v(-0.530897, 0.526776, 2.52596, 0.531064, 2.42381, -1.47044),
			_v(-0.58235, 0.59153, 2.42615, 0.589584, 2.38336, -1.45621), _v(-0.590023, 0.625101, 2.34691, 0.627043, 2.36424, -1.44246), _v(-0.565379, 0.605738, 2.28484, 0.695691, 2.3714, -1.41136),
			_v(-0.604992, 0.599334, 2.19731, 0.734179, 2.34835, -1.36636), _v(-0.705607, 0.656351, 2.08853, 0.766989, 2.27119, -1.34053), _v(-0.716506, 0.710179, 1.99537, 0.828379, 2.21018, -1.34214),
			_v(-0.63172, 0.7189, 1.95272, 0.882299, 2.20048, -1.35306), _v(-0.594594, 0.772518, 1.88187, 0.841779, 2.16104, -1.37196), _v(-0.527735, 0.746352, 1.83989, 0.912443, 2.15279, -1.36288),
			_v(-0.521512, 0.697974, 1.8495, 0.963711, 2.13764, -1.31326), _v(-0.448012, 0.707571, 1.8853, 0.978596, 2.1322, -1.31729), _v(-0.39389, 0.704314, 1.92504, 0.913355, 2.14528, -1.30861),
			_v(-0.385584, 0.700693, 1.96043, 0.912333, 2.13302, -1.28442), _v(-0.374032, 0.696335, 1.99644, 0.929093, 2.11194, -1.26819), _v(-0.337989, 0.702424, 2.03157, 0.933489, 2.10087, -1.27036),
			_v(-0.254022, 0.71216, 2.07731, 0.939661, 2.09934, -1.29063), _v(-0.187209, 0.665891, 2.12903, 0.962375, 2.1068, -1.28116), _v(-0.157749, 0.643151, 2.15482, 0.998503, 2.09976, -1.277),
			_v(-0.162739, 0.643679, 2.17696, 1.04764, 2.05672, -1.25382), _v(-0.154671, 0.65983, 2.17708, 1.0684, 2.02689, -1.24627), _v(-0.104678, 0.616005, 2.19711, 1.14697, 2.00913, -1.23359),
			_v(-0.161726, 0.642245, 2.19836, 1.21338, 1.94915, -1.21276), _v(-0.218807, 0.665564, 2.195, 1.22698, 1.90808, -1.17536), _v(-0.234282, 0.676862, 2.1986, 1.2442, 1.8717, -1.15085),
			_v(-0.219447, 0.684182, 2.21856, 1.29814, 1.82341, -1.1417), _v(-0.194466, 0.662605, 2.24739, 1.32138, 1.80258, -1.12341), _v(-0.187932, 0.634834, 2.2637, 1.33094, 1.79965, -1.10403),
			_v(-0.186662, 0.612724, 2.27114, 1.35035, 1.77585, -1.07021), _v(-0.16862, 0.653177, 2.25503, 1.38537, 1.72021, -1.06599), _v(-0.112927, 0.58347, 2.26558, 1.43704, 1.71481, -1.04327),
			_v(-0.119407, 0.572824, 2.26404, 1.44271, 1.70003, -1.00448), _v(-0.122626, 0.64591, 2.25847, 1.46025, 1.63883, -0.9968), _v(-0.121084, 0.680696, 2.26037, 1.46737, 1.60669, -0.99291),
			_v(-0.0689498, 0.699905, 2.27247, 1.49521, 1.567, -0.999378), _v(-0.0555174, 0.6791, 2.28948, 1.51906, 1.54079, -0.976978), _v(-0.0747014, 0.588206, 2.32132, 1.56706, 1.52375, -0.925404),
			_v(-0.10606, 0.660977, 2.31158, 1.59099, 1.43903, -0.914624), _v(-0.0872167, 0.661015, 2.31281, 1.60179, 1.41758, -0.900573), _v(-0.0354705, 0.575782, 2.32278, 1.66735, 1.39538, -0.880163),
			_v(-0.152767, 0.616701, 2.32697, 1.70676, 1.29165, -0.826199), _v(-0.150459, 0.613629, 2.33407, 1.71944, 1.26701, -0.820107), _v(-0.184193, 0.548745, 2.36218, 1.75746, 1.22896, -0.776249),
			_v(-0.114488, 0.567132, 2.38745, 1.74496, 1.2163, -0.78246), _v(-0.112072, 0.583288, 2.40315, 1.74632, 1.18444, -0.755977), _v(-0.119015, 0.637255, 2.39389, 1.73817, 1.14892, -0.733268),
			_v(-0.0844145, 0.669847, 2.38415, 1.74749, 1.1105, -0.729001), _v(0.0155246, 0.657627, 2.38808, 1.77154, 1.09322, -0.744426), _v(0.0964757, 0.633097, 2.39414, 1.76727, 1.11907, -0.73391),
			_v(0.186051, 0.60545, 2.40051, 1.75059, 1.15771, -0.716946), _v(0.245231, 0.566454, 2.41748, 1.75981, 1.15803, -0.70452), _v(0.304414, 0.515567, 2.434, 1.77811, 1.15131, -0.69611),
			_v(0.310276, 0.485839, 2.4533, 1.7963, 1.11646, -0.669517), _v(0.269184, 0.532032, 2.46009, 1.83699, 0.988116, -0.666424), _v(0.21631, 0.578049, 2.43617, 1.85031, 0.910347, -0.64316),
			_v(0.167065, 0.532154, 2.42316, 1.89858, 0.845404, -0.619721), _v(0.0684616, 0.435519, 2.40979, 1.96272, 0.791445, -0.560397), _v(0.0115834, 0.383996, 2.37315, 2.00739, 0.7411, -0.519129),
			_v(-0.0301493, 0.448407, 2.32558, 2.00993, 0.676521, -0.492694), _v(-0.0207977, 0.503207, 2.28676, 2.0145, 0.613641, -0.485577), _v(-0.0222401, 0.530005, 2.24509, 2.01904, 0.568867, -0.464332),
			_v(-0.137828, 0.550921, 2.18851, 2.02487, 0.50804, -0.390327), _v(-0.28858, 0.601937, 2.10805, 2.02976, 0.414276, -0.308356), _v(-0.320711, 0.633064, 2.06783, 2.03392, 0.36584, -0.287136),
			_v(-0.335984, 0.666846, 2.00786, 2.05345, 0.285614, -0.263858), _v(-0.357343, 0.6731, 1.97498, 2.06055, 0.255184, -0.241171), _v(-0.409973, 0.645304, 1.91446, 2.08785, 0.2019, -0.189292),
			_v(-0.423428, 0.573751, 1.88008, 2.12073, 0.197671, -0.151364), _v(-0.380202, 0.588206, 1.86056, 2.11408, 0.194021, -0.160418), _v(-0.313832, 0.613571, 1.84361, 2.09552, 0.219938, -0.175992),
			_v(-0.2793, 0.618622, 1.80207, 2.08022, 0.261298, -0.144573), _v(-0.208543, 0.628788, 1.78305, 2.06323, 0.313089, -0.157635), _v(-0.0974837, 0.63107, 1.76653, 2.0543, 0.376275, -0.192148),
			_v(-0.00856159, 0.639989, 1.73382, 2.04364, 0.449633, -0.214413), _v(0.0423566, 0.646858, 1.70242, 2.04174, 0.483983, -0.23046), _v(0.0956229, 0.666808, 1.66306, 2.0352, 0.509925, -0.253355),
			_v(0.202009, 0.673006, 1.59515, 2.03006, 0.593753, -0.293237), _v(0.245835, 0.664719, 1.56528, 2.02769, 0.658143, -0.300352), _v(0.356, 0.605244, 1.51448, 2.05291, 0.77001, -0.323964),
			_v(0.445916, 0.609175, 1.46933, 2.03871, 0.854013, -0.360324), _v(0.580327, 0.642554, 1.4005, 2.00732, 0.965343, -0.41909), _v(0.696621, 0.739756, 1.31305, 1.93953, 1.0431, -0.488535),
			_v(0.806497, 0.8117, 1.21753, 1.88988, 1.08312, -0.565395), _v(0.859289, 0.822082, 1.20098, 1.88759, 1.09686, -0.573371), _v(0.920206, 0.838334, 1.19638, 1.8865, 1.05538, -0.586575),
			_v(0.970465, 0.776566, 1.27179, 1.93413, 0.972105, -0.567326), _v(1.02485, 0.782193, 1.31134, 1.93266, 0.933519, -0.559188), _v(1.03425, 0.710733, 1.42005, 1.98809, 0.824116, -0.507228),
			_v(1.05426, 0.654036, 1.48141, 2.02197, 0.746313, -0.482503), _v(1.06744, 0.567765, 1.55532, 2.069, 0.668092, -0.444549), _v(1.09036, 0.503542, 1.63239, 2.1009, 0.568211, -0.421591),
			_v(1.06368, 0.470897, 1.71248, 2.12001, 0.509868, -0.36419), _v(0.965808, 0.350116, 1.82277, 2.16836, 0.378673, -0.283529), _v(0.965706, 0.357913, 1.83211, 2.16051, 0.360444, -0.244319),
			_v(1.03692, 0.386374, 1.78407, 2.1507, 0.366295, -0.251589), _v(1.03899, 0.371159, 1.7518, 2.17326, 0.333757, -0.234297), _v(1.02492, 0.362437, 1.72596, 2.19504, 0.292302, -0.223563),
			_v(0.961713, 0.354996, 1.73406, 2.21668, 0.232776, -0.187502), _v(0.904997, 0.358396, 1.74355, 2.22038, 0.165482, -0.160755), _v(0.802259, 0.332398, 1.7805, 2.22978, 0.0586454, -0.10064),
			_v(0.684593, 0.270923, 1.78302, 2.26305, -0.0350083, -0.045514), _v(0.674281, 0.283386, 1.71826, 2.28909, 0.00351916, -0.0311766), _v(0.67739, 0.314489, 1.66299, 2.30799, 0.0610927, -0.0179223),
			_v(0.697267, 0.378475, 1.61262, 2.30366, 0.138314, -0.0106), _v(0.807616, 0.523353, 1.48133, 2.2745, 0.353486, -0.0580066), _v(0.908304, 0.558805, 1.40032, 2.26988, 0.461751, -0.117895),
			_v(0.946747, 0.555279, 1.37765, 2.27847, 0.499557, -0.138936), _v(1.00662, 0.651119, 1.30244, 2.24007, 0.61673, -0.171954), _v(1.04129, 0.72426, 1.24524, 2.20478, 0.678529, -0.199705),
			_v(1.10736, 0.852193, 1.11799, 2.15693, 0.838446, -0.248901), _v(1.15985, 0.948345, 1.00602, 2.12079, 0.959951, -0.293218), _v(1.18612, 1.04212, 0.903721, 2.07204, 1.04479, -0.33383),
			_v(1.19503, 1.14576, 0.802, 2.01446, 1.13027, -0.370011), _v(1.20908, 1.15712, 0.775743, 2.00678, 1.1366, -0.385621), _v(1.25158, 1.23205, 0.650199, 1.96206, 1.18725, -0.447988),
			_v(1.29168, 1.27323, 0.541826, 1.94216, 1.23937, -0.505474), _v(1.2428, 1.32978, 0.533222, 1.93512, 1.34107, -0.483427), _v(1.21815, 1.37304, 0.478721, 1.91641, 1.4055, -0.501728),
			_v(1.20297, 1.43046, 0.34976, 1.88261, 1.49766, -0.575732), _v(1.14691, 1.49997, 0.253286, 1.83522, 1.59907, -0.628862), _v(1.11733, 1.52704, 0.230554, 1.81375, 1.63049, -0.637443),
			_v(1.05272, 1.58924, 0.128279, 1.7503, 1.69976, -0.690703), _v(0.963692, 1.65674, 0.0172406, 1.6668, 1.76627, -0.747457), _v(0.890365, 1.69801, -0.0939928, 1.5902, 1.80553, -0.806971),
			_v(1.04322, 1.62907, -0.0969555, 1.64355, 1.73564, -0.842401), _v(1.1019, 1.61529, -0.0692357, 1.64195, 1.72306, -0.835895), _v(1.16621, 1.5817, -0.0226285, 1.6649, 1.68499, -0.822333),
			_v(1.26648, 1.51717, 0.0070776, 1.68591, 1.59181, -0.829397), _v(1.32262, 1.47344, 0.0383165, 1.70752, 1.54411, -0.83051), _v(1.35731, 1.44412, 0.0830542, 1.71713, 1.50275, -0.819472),
			_v(1.42907, 1.38835, 0.122937, 1.7254, 1.41889, -0.824917), _v(1.52828, 1.30058, 0.289843, 1.78829, 1.33154, -0.765087), _v(1.59591, 1.23863, 0.341836, 1.82054, 1.28269, -0.756807),
			_v(1.66425, 1.17703, 0.435308, 1.84743, 1.22432, -0.724773), _v(1.70021, 1.15592, 0.494431, 1.85786, 1.20148, -0.704314), _v(1.76301, 1.12289, 0.551325, 1.86875, 1.16734, -0.696538),
			_v(1.80037, 1.09459, 0.638038, 1.87934, 1.12394, -0.669291), _v(1.8257, 1.04035, 0.793481, 1.90454, 1.07218, -0.61243), _v(1.87483, 0.975829, 0.890355, 1.91715, 1.01339, -0.594534),
			_v(1.9147, 0.932968, 0.965273, 1.92494, 0.980734, -0.577135), _v(1.94908, 0.855799, 1.07364, 1.94495, 0.915334, -0.555189), _v(1.94595, 0.766186, 1.24604, 1.97047, 0.821902, -0.501904),
			_v(1.93168, 0.70767, 1.39491, 1.97947, 0.727464, -0.453568), _v(1.92879, 0.608951, 1.51844, 1.99794, 0.628626, -0.426528), _v(1.94257, 0.529955, 1.61466, 2.02435, 0.580311, -0.401127),
			_v(1.94553, 0.46908, 1.70892, 2.04082, 0.525261, -0.373743), _v(1.91592, 0.438791, 1.81645, 2.03996, 0.460081, -0.334022), _v(1.87857, 0.42689, 1.92933, 2.01715, 0.363672, -0.306317),
			_v(1.84997, 0.427896, 2.01122, 2.01508, 0.332792, -0.268132), _v(1.8163, 0.382497, 2.10611, 2.02534, 0.282701, -0.229115), _v(1.789, 0.403448, 2.16851, 2.01735, 0.252197, -0.193274),
			_v(1.75399, 0.416114, 2.22648, 2.00942, 0.209206, -0.159055), _v(1.71347, 0.402252, 2.28645, 1.9968, 0.131816, -0.139894), _v(1.66921, 0.411736, 2.34459, 1.98383, 0.0687837, -0.116704),
			_v(1.59459, 0.395976, 2.42515, 1.9815, -0.00474135, -0.0810448), _v(1.52731, 0.352833, 2.48382, 1.99476, -0.0479806, -0.047026), _v(1.36908, 0.329342, 2.58145, 2.01007, -0.0957636, 0.0309687),
			_v(1.27786, 0.250142, 2.63924, 2.03282, -0.141939, 0.0653479), _v(1.20691, 0.201175, 2.68386, 2.03482, -0.201345, 0.0882042), _v(1.15761, 0.148225, 2.71289, 2.04171, -0.242873, 0.10258),
			_v(1.10899, 0.0884756, 2.73883, 2.05861, -0.26819, 0.123977), _v(0.995999, -0.0072886, 2.78476, 2.08542, -0.315959, 0.175589), _v(0.92079, -0.0276494, 2.80851, 2.08151, -0.364271, 0.206461),
			_v(0.828681, -0.0121645, 2.8285, 2.06628, -0.416183, 0.241894), _v(0.761974, 0.0175338, 2.83339, 2.05138, -0.451333, 0.269641), _v(0.697534, -0.0101506, 2.83139, 2.03883, -0.528489, 0.274575),
			_v(0.633534, -0.0492552, 2.82602, 2.03879, -0.586589, 0.287471), _v(0.546204, -0.0905113, 2.82207, 2.04784, -0.632747, 0.319809), _v(0.446942, -0.138463, 2.8198, 2.05496, -0.683163, 0.361049),
			_v(0.383247, -0.17357, 2.82247, 2.04778, -0.738192, 0.385195), _v(0.360031, -0.22397, 2.83137, 2.03719, -0.795962, 0.384699), _v(0.317604, -0.253631, 2.84404, 2.02807, -0.838702, 0.403496),
			_v(0.292306, -0.261393, 2.8547, 2.01829, -0.867121, 0.419237), _v(0.246421, -0.289419, 2.86181, 2.01688, -0.902614, 0.442185), _v(0.18735, -0.294414, 2.86526, 2.00776, -0.936326, 0.472532),
			_v(0.152961, -0.335254, 2.85943, 1.99601, -0.989361, 0.472743), _v(0.109003, -0.378251, 2.8532, 1.9784, -1.05152, 0.472677), _v(0.0679303, -0.428936, 2.84598, 1.96506, -1.10835, 0.4719),
			_v(0.018551, -0.408174, 2.85213, 1.93927, -1.14587, 0.497083), _v(0.0410439, -0.415275, 2.85766, 1.93807, -1.1498, 0.504948), _v(0.0759185, -0.442882, 2.86843, 1.94732, -1.14621, 0.51423),
			_v(0.0933737, -0.447768, 2.89501, 1.93379, -1.15901, 0.521296), _v(0.0931412, -0.43717, 2.93057, 1.91374, -1.17359, 0.53384), _v(0.143413, -0.405564, 2.96111, 1.89628, -1.16626, 0.532758),
			_v(0.240186, -0.354554, 2.9918, 1.88286, -1.13881, 0.530586), _v(0.333326, -0.345396, 3.00214, 1.88846, -1.11501, 0.528058), _v(0.418648, -0.337381, 2.99912, 1.90176, -1.08763, 0.52958),
			_v(0.506682, -0.287019, 2.99546, 1.89098, -1.07203, 0.525408), _v(0.520561, -0.266867, 2.99173, 1.87515, -1.08878, 0.530244), _v(0.549298, -0.270218, 2.98776, 1.86405, -1.10626, 0.525564),
			_v(0.603104, -0.270082, 2.98393, 1.86411, -1.10773, 0.526459), _v(0.665047, -0.268695, 2.97567, 1.86181, -1.10713, 0.520034), _v(0.701567, -0.283201, 2.97169, 1.8692, -1.10943, 0.522675),
			_v(0.708135, -0.312293, 2.97336, 1.88061, -1.11883, 0.535458), _v(0.678632, -0.318381, 2.99638, 1.87293, -1.14067, 0.558393), _v(0.643248, -0.324096, 3.0292, 1.84989, -1.17358, 0.573545),
			_v(0.591059, -0.349377, 3.06595, 1.82993, -1.21248, 0.590046), _v(0.54064, -0.393481, 3.09516, 1.81658, -1.25012, 0.601739), _v(0.476799, -0.442349, 3.12589, 1.80657, -1.2889, 0.621686),
			_v(0.361217, -0.495304, 3.14939, 1.796, -1.34152, 0.6545), _v(0.281619, -0.524153, 3.15979, 1.78141, -1.38142, 0.678594), _v(0.2213, -0.525289, 3.17259, 1.74798, -1.42347, 0.68978),
			_v(0.157722, -0.533425, 3.19018, 1.703, -1.47753, 0.691233), _v(0.129915, -0.510074, 3.21239, 1.66106, -1.51144, 0.695664), _v(0.169785, -0.510766, 3.22371, 1.64184, -1.52303, 0.685939),
			_v(0.155192, -0.518027, 3.24109, 1.6327, -1.54153, 0.697296), _v(0.0397876, -0.487807, 3.27595, 1.60573, -1.57529, 0.742111), _v(-0.0755929, -0.45945, 3.30761, 1.56141, -1.61801, 0.769799),
			_v(-0.138644, -0.452902, 3.33218, 1.54021, -1.64194, 0.791267), _v(-0.154894, -0.466784, 3.35306, 1.52926, -1.65605, 0.799962), _v(-0.197606, -0.454686, 3.37841, 1.51375, -1.66945, 0.821934),
			_v(-0.184227, -0.393779, 3.41269, 1.49168, -1.66448, 0.839152), _v(-0.139487, -0.375781, 3.43448, 1.48554, -1.65911, 0.84285), _v(-0.138409, -0.388913, 3.43986, 1.47477, -1.67526, 0.847313),
			_v(-0.195092, -0.361835, 3.44115, 1.43853, -1.70539, 0.862162), _v(-0.243684, -0.371198, 3.43922, 1.42229, -1.73249, 0.876264), _v(-0.276485, -0.388905, 3.43965, 1.39906, -1.76397, 0.877931),
			_v(-0.3571, -0.357757, 3.44774, 1.36103, -1.79671, 0.89884), _v(-0.637518, -0.205179, 3.45087, 1.26071, -1.86165, 0.971444), _v(-0.714852, -0.166481, 3.46438, 1.21772, -1.8891, 0.987923),
			_v(-0.702077, -0.212938, 3.48955, 1.21871, -1.8988, 0.990759), _v(-0.683967, -0.282962, 3.50859, 1.22163, -1.91179, 0.985249), _v(-0.74414, -0.345962, 3.50042, 1.21336, -1.94412, 0.994639),
			_v(-0.731772, -0.347116, 3.50788, 1.1887, -1.96035, 0.994149), _v(-0.74614, -0.289999, 3.50936, 1.12991, -1.98338, 0.99248), _v(-0.825519, -0.291559, 3.48782, 1.0886, -2.02417, 0.996998),
			_v(-0.794982, -0.280885, 3.49404, 1.05023, -2.03897, 0.977984), _v(-0.846366, -0.301889, 3.474, 1.04608, -2.06306, 0.997922), _v(-0.870287, -0.29794, 3.46567, 1.03779, -2.07557, 1.01759),
			_v(-0.894549, -0.2137, 3.46748, 0.997802, -2.08189, 1.0453), _v(-0.929376, -0.137656, 3.47111, 0.95119, -2.09469, 1.06695), _v(-0.955647, -0.0918643, 3.47362, 0.92243, -2.10446, 1.08618),
			_v(-0.939477, -0.026301, 3.48745, 0.90872, -2.09371, 1.11218), _v(-0.817414, 0.129383, 3.52607, 0.90134, -2.03972, 1.14993), _v(-0.771624, 0.166308, 3.53583, 0.906012, -2.02617, 1.17039),
			_v(-0.747013, 0.145233, 3.53591, 0.913664, -2.03084, 1.17907), _v(-0.79907, 0.138804, 3.51041, 0.902117, -2.05284, 1.19949), _v(-0.866757, 0.0816348, 3.47623, 0.86411, -2.10138, 1.18934),
			_v(-0.936877, 0.0148858, 3.44303, 0.842399, -2.14527, 1.18566), _v(-0.995063, -0.00281405, 3.40918, 0.819028, -2.17724, 1.19813), _v(-0.99986, 0.0530509, 3.39689, 0.806687, -2.17669, 1.22875),
			_v(-0.977887, 0.0817454, 3.40109, 0.806711, -2.17219, 1.25416), _v(-0.984685, 0.0848127, 3.40504, 0.783975, -2.18461, 1.2606), _v(-1.03349, 0.0855106, 3.40238, 0.743428, -2.20981, 1.26629),
			_v(-1.05782, 0.0996727, 3.41132, 0.712076, -2.22337, 1.27733), _v(-1.09186, 0.0696486, 3.40934, 0.695659, -2.24583, 1.28618), _v(-1.10608, 0.0357734, 3.401, 0.695798, -2.26308, 1.29616),
			_v(-1.1481, 0.0634924, 3.37193, 0.678677, -2.27519, 1.32637), _v(-1.1556, 0.0462922, 3.3507, 0.64064, -2.29894, 1.31284), _v(-1.20926, -0.00253758, 3.31683, 0.606253, -2.33797, 1.3018),
			_v(-1.24645, 0.04489, 3.28964, 0.546739, -2.35888, 1.30472), _v(-1.28817, 0.110872, 3.26352, 0.508112, -2.37207, 1.33126), _v(-1.5146, 0.234693, 3.13827, 0.428754, -2.41497, 1.40993),
			_v(-1.62191, 0.252723, 3.05965, 0.335083, -2.46108, 1.39868), _v(-1.68201, 0.242578, 2.99768, 0.26011, -2.49986, 1.37767), _v(-1.81524, 0.175804, 2.87653, 0.207617, -2.56926, 1.36888),
			_v(-1.83224, 0.250534, 2.81726, 0.165891, -2.57264, 1.39291), _v(-1.7962, 0.344557, 2.78101, 0.116176, -2.56319, 1.40179), _v(-1.78718, 0.402557, 2.73228, 0.0626919, -2.57176, 1.39452),
			_v(-1.81455, 0.446281, 2.65719, 0.0195322, -2.58871, 1.40459), _v(-1.85554, 0.448345, 2.57792, -0.0259436, -2.61844, 1.40093), _v(-1.91064, 0.495893, 2.47134, -0.109058, -2.64624, 1.40154),
			_v(-1.95207, 0.587858, 2.37384, -0.204568, -2.65818, 1.41083), _v(-2.00961, 0.638784, 2.26787, -0.28263, -2.68268, 1.42153), _v(-2.03518, 0.644944, 2.18898, -0.332884, -2.70857, 1.41672),
			_v(-2.04207, 0.695254, 2.11416, -0.374999, -2.71656, 1.4361), _v(-2.00936, 0.747831, 2.06437, -0.416524, -2.7142, 1.43947), _v(-1.9805, 0.76404, 2.01505, -0.448105, -2.72185, 1.42974),
			_v(-1.95302, 0.800158, 1.96836, -0.481867, -2.72244, 1.42587), _v(-1.92603, 0.80637, 1.91587, -0.492604, -2.73346, 1.42869), _v(-1.90656, 0.807914, 1.84563, -0.537701, -2.75679, 1.40467),
			_v(-1.89641, 0.926113, 1.71811, 0.650103, 2.72293, -1.3865), _v(-1.87538, 1.041, 1.60629, 0.744518, 2.68097, -1.3821), _v(-1.82973, 1.08987, 1.5579, 0.791728, 2.663, -1.37891),
			_v(-1.79276, 1.08883, 1.52462, 0.816001, 2.66104, -1.35803), _v(-1.76949, 1.08954, 1.4689, 0.819789, 2.64717, -1.34828), _v(-1.72932, 1.07348, 1.45168, 0.808208, 2.65412, -1.34299),
			_v(-1.68039, 1.05352, 1.44706, 0.783425, 2.66685, -1.34752), _v(-1.62763, 1.01378, 1.44422, 0.749141, 2.68775, -1.34532), _v(-1.56523, 0.934948, 1.46986, 0.661552, 2.71541, -1.36465),
			_v(-1.516, 0.836347, 1.49878, 0.576987, 2.73873, -1.36463), _v(-1.47222, 0.766266, 1.52206, 0.52892, 2.75006, -1.36264), _v(-1.4082, 0.740965, 1.56137, 0.500131, 2.75607, -1.38728),
			_v(-1.36351, 0.709546, 1.58934, 0.496373, 2.75842, -1.38847), _v(-1.32275, 0.659166, 1.61828, 0.480946, 2.76246, -1.38404), _v(-1.26315, 0.607228, 1.6653, 0.451673, 2.77768, -1.3847),
			_v(-1.18262, 0.586845, 1.71743, -0.41162, -2.76868, 1.4011), _v(-1.12094, 0.572808, 1.74952, -0.38619, -2.76001, 1.40758), _v(-1.08305, 0.549675, 1.75815, -0.374664, -2.76864, 1.41019),
			_v(-1.04583, 0.547199, 1.75822, -0.3805, -2.77273, 1.417), _v(-1.02413, 0.571345, 1.74102, 0.410909, 2.75605, -1.42378), _v(-1.0102, 0.597846, 1.72938, 0.417396, 2.72536, -1.4393),
			_v(-0.961243, 0.677933, 1.72286, 0.436809, 2.69105, -1.49315), _v(-0.922674, 0.722894, 1.72362, 0.409744, 2.66165, -1.53789), _v(-0.874062, 0.760151, 1.73444, 0.371554, 2.63289, -1.57879),
			_v(-0.827455, 0.761883, 1.75651, 0.374362, 2.62835, -1.58729), _v(-0.80517, 0.784812, 1.76149, 0.403088, 2.61069, -1.59883), _v(-0.762841, 0.770348, 1.78849, 0.405739, 2.61381, -1.6004),
			_v(-0.68147, 0.725285, 1.84138, 0.378586, 2.63641, -1.60136), _v(-0.615459, 0.694912, 1.8782, 0.344154, 2.64892, -1.60716), _v(-0.534582, 0.66401, 1.90745, 0.284478, 2.65923, -1.61924)
		};
		size_t n_data_size = sizeof(p_data) / sizeof(p_data[0]);
		// some poses from the TUM dataset

		int n_fail_num = 0;

		std::vector<_TyVectorUnalign> data(p_data, p_data + n_data_size);
		std::vector<_TyVectorUnalign> data2(n_data_size);
		for(int n_pass = 0; n_pass < 10; ++ n_pass) {
			_TyVectorUnalign v_random;
			for(int i = 0; i < n_pose_dimension; ++ i)
				v_random[i] = float(rand()) / RAND_MAX * 2 - 1;
			v_random.template head<n_pose_pos_components>() *= 100.0;
			v_random.template tail<n_pose_rot_components>() *= M_PI;
			double f_scale = .1 + float(rand()) / RAND_MAX * 2;
			// make a random transform

			/*Eigen::Quaterniond t_rot;
			C3DJacobians::AxisAngle_to_Quat(v_random.tail<n_pose_rot_components>(), t_rot);
			Eigen::Matrix3d R = t_rot.matrix();
			Eigen::Vector3d t = v_random.head<n_pose_pos_components>();*/
			// decompose to Rt to transform many points efficiently

			for(size_t i = 0; i < n_data_size; ++ i) {
				_TyVector v_transformed = data[i];
				/*v_transformed.head<3>() = R * v_transformed.head<3>() * f_scale + t;
				Eigen::Quaterniond t_pose_rot;
				C3DJacobians::AxisAngle_to_Quat(v_transformed.tail<3>(), t_pose_rot);
				t_pose_rot = t_pose_rot * t_rot;
				C3DJacobians::Quat_to_AxisAngle(t_pose_rot, v_transformed.tail<3>());
				// transform the pose*/

				v_transformed.template head<3>() *= f_scale;
				_TyPoseOpsImpl::Relative_to_Absolute(v_random, v_transformed, v_transformed);
				data2[i] = v_transformed;
			}
			// apply the transform onto the data

			Eigen::Vector6d v_rmse;
			Compute_AllErrors(data, data2, true, v_rmse, true);
			_ASSERTE(v_rmse.norm() < 1e-6); // should have null errors
			if(v_rmse.norm() >= 1e-6)
				++ n_fail_num;
		}
		// run a few tests

		if(!n_fail_num)
			printf("\ndone. all tests passed\n");
		else
			fprintf(stderr, "error: there were %d errors\n", n_fail_num);
	}

protected:
	/**
	 *	@brief helper function; assembles a pose vector
	 *
	 *	@param[in] a is value of the first translation component
	 *	@param[in] b is value of the second translation component
	 *	@param[in] c is value of the third translation component (unused in 2D)
	 *	@param[in] d is value of the first rotation component (norm of rotation used in 2D)
	 *	@param[in] e is value of the second rotation component (norm of rotation used in 2D)
	 *	@param[in] f is value of the third rotation component (norm of rotation used in 2D)
	 *
	 *	@return Returns a pose vector corresponding to the given components.
	 */
	static _TyVectorUnalign _v(double a, double b, double c, double d, double e, double f)
	{
		_TyVectorUnalign vec;
		if(n_pose_dimension == 6)
			vec << a, b, c, d, e, f;
		else /*if(n_pose_dimension == 3)*/ {
			_ASSERTE(n_pose_dimension == 3);
			vec << a, b, sqrt(d * d + e * e + f * f);
		}
		return vec;
	}
};

#endif // !__SLAM_ERROR_EVALUATOR_INCLUDED
