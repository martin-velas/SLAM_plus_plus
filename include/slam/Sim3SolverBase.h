/*
								+----------------------------------+
								|                                  |
								|      ***  Sim(3) types  ***      |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|         Sim3SolverBase.h         |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __SIM3_SOLVER_BASE_INCLUDED
#define __SIM3_SOLVER_BASE_INCLUDED

/**
 *	@file include/slam/Sim3SolverBase.h
 *	@brief Sim(3) solver utility functions
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include "slam/BlockMatrix.h" // CDeriveMatrixType
#include "slam/3DSolverBase.h" // not sure what is needed from here, maybe nothing
#include "slam/BASolverBase.h" // smart intrinsics plus
#include "eigen/Eigen/QR"
#include "geometry/DistortionModel.h"


/** \addtogroup sim3
 *	@{
 */

#ifdef HAVE_TOON
#include <TooN_msvc_patch.h> // !!
#include <TooN/sim3.h> // test TooN
#endif // HAVE_TOON

// inject the 7D, 1D and 5D types into Eigen namespace
namespace Eigen {

#ifndef HAVE_EIGEN_5D_DOUBLE_VECTOR
#define HAVE_EIGEN_5D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 5, 1> Vector5d; /**< @brief 5D vector type */
#endif // !HAVE_EIGEN_5D_DOUBLE_VECTOR
#ifndef HAVE_EIGEN_1D_DOUBLE_VECTOR
#define HAVE_EIGEN_1D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 1, 1> Vector1d; /**< @brief 1D vector type (required for inverse distance vertices) */
#endif // !HAVE_EIGEN_1D_DOUBLE_VECTOR
#ifndef HAVE_EIGEN_7D_DOUBLE_VECTOR
#define HAVE_EIGEN_7D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 7, 1> Vector7d; /**< @brief 7D vector type */
#endif // !HAVE_EIGEN_7D_DOUBLE_VECTOR
#ifndef HAVE_EIGEN_7D_DOUBLE_MATRIX
#define HAVE_EIGEN_7D_DOUBLE_MATRIX
typedef Eigen::Matrix<double, 7, 7> Matrix7d; /**< @brief 7x7 matrix type */
#endif // !HAVE_EIGEN_7D_DOUBLE_MATRIX

} // ~Eigen

/**
 *	@brief implementation of Jacobian calculations, required by Sim(3) solvers
 *
 *	This attempts to fix a few design failures in the SE(3) BA solver.
 *
 *	First, the SE(3) solver swaps the order of point, camera (as it is in the graph file)
 *	to camera, point. That introduces a lot of argument swapping and generally only makes
 *	problems. Here, the order is point, camera. This affects argument order in Project_P2C()
 *	and its variants, and also the order of arguments in the observation edge types (such
 *	as CEdgeP2CSim3G, ...).
 *
 *	Second, the SE(3) solver represents the cameras by their inverse transformation in hope
 *	to save computation. However, that is a fallacy as the inverse transformation is equally
 *	costy as the direct transformation. Therefore, this solver shall represent all the cameras
 *	by their direct transformation (the same as in the graph file). This only affects the
 *	implementation of Project_P2C().
 *
 *	To make the optimization over the distortion parameters (not implemented here yet) more
 *	stable, the distortion parameters are scaled by the focal length which in turn reduces
 *	the adjustments the solver is trying to make. This is the same as in the SE(3) solver.
 */
class CSim3Jacobians {
public:
	/**
	 *	@brief Sim(3) coordinate frame
	 */
	struct TSim3 { friend class CSim3Jacobians;
		struct from_sim3_vector_tag {}; /**< @brief tag-scheduling type for initialization from an exponent of a \f$\mathfrak{sim}(3)\f$ vector */
		struct from_tRs_vector_tag {}; /**< @brief tag-scheduling type for initialization from a translation rotation scale vector */

		static const from_sim3_vector_tag from_sim3_vector; /**< @brief tag-scheduling value for initialization from an exponent of a \f$\mathfrak{sim}(3)\f$ vector */
		static const from_tRs_vector_tag from_tRs_vector; /**< @brief tag-scheduling value for initialization from a translation rotation scale vector */

		Eigen::Vector3d v_translation; /**< @brief translation vector */
		Eigen::Quaterniond t_rotation; /**< @brief rotation as normalized quaternion */
		double f_scale; /**< @brief scale */

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/**
		 *	@brief default constructor; has no effect
		 */
		inline TSim3()
		{}

		/**
		 *	@brief constructor; initializes the pose from exponential of a \f$\mathfrak{sim}(3)\f$ vector
		 *
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *
		 *	@param[in] r_v_vec is a vector in \f$\mathfrak{sim}(3)\f$ to initialize from, obtained e.g. by a call to v_Log()
		 *	@param[in] t_tag is used for tag scheduling (value unused)
		 */
		template <class Derived0>
		inline TSim3(const Eigen::MatrixBase<Derived0> &r_v_vec, from_sim3_vector_tag UNUSED(t_tag))
		{
			//DimensionCheck<Eigen::Vector7d>(r_v_vec); // checked inside as well
			Exp(r_v_vec);
		}

		/**
		 *	@brief constructor; initializes the pose from a \f$R^7\f$ vector
		 *
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *
		 *	@param[in] r_v_vec is a vector in \f$R^7\f$ to initialize from
		 *	@param[in] t_tag is used for tag scheduling (value unused)
		 */
		template <class Derived0>
		inline TSim3(const Eigen::MatrixBase<Derived0> &r_v_vec, from_tRs_vector_tag UNUSED(t_tag))
			:v_translation(r_v_vec.template head<3>()), f_scale(r_v_vec(6))
		{
			DimensionCheck<Eigen::Vector7d>(r_v_vec);
			C3DJacobians::AxisAngle_to_Quat(r_v_vec.template tail<4>().template head<3>(), t_rotation);
		}

		/**
		 *	@brief constructor; initializes the fields in this structure
		 *
		 *	@param[in] r_v_translation is translation vector
		 *	@param[in] r_t_rotation is rotation as normalized quaternion
		 *	@param[in] _f_scale is scale
		 */
		inline TSim3(const Eigen::Vector3d &r_v_translation, const Eigen::Quaterniond &r_t_rotation, double _f_scale)
			:v_translation(r_v_translation), t_rotation(r_t_rotation), f_scale(_f_scale)
		{}

		/**
		 *	@brief gets identity pose
		 *	@return Returns an identity pose.
		 */
		static inline TSim3 t_Identity()
		{
			return TSim3(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 1.0);
		}

		/**
		 *	@brief sets this to be an identity pose
		 */
		void Identity()
		{
			v_translation.setZero();
			t_rotation.setIdentity();
			f_scale = 1;
		}

		/**
		 *	@brief inverses this pose
		 */
		void Invert()
		{
			f_scale = 1 / f_scale;
			//t_rotation = t_rotation.inverse();
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			t_rotation = t_rotation.conjugate(); // faster but only if normalized
			v_translation = -f_scale * t_rotation._transformVector(v_translation);
		}

		/**
		 *	@brief calculates logarithm of this pose
		 *	@return Returns the logarithm of this pose.
		 */
		Eigen::Vector7d v_Log() const
		{
			return CSim3Jacobians::v_Log(*this);
		}

		/**
		 *	@brief converts this pose to vectorial form
		 *	@return Returns the translation, axis-angle rotation and scale (tRs) vector representation of this pose.
		 */
		Eigen::Vector7d v_tRs() const
		{
			Eigen::Vector7d v_result;
			v_result.head<3>() = v_translation;
			Eigen::VectorBlock<Eigen::Block<Eigen::Vector7d, 4, 1>, 3> v_rot_part = v_result.tail<4>().head<3>(); // g++ requires a temporary
			C3DJacobians::Quat_to_AxisAngle(t_rotation, v_rot_part);
			v_result(6) = f_scale;
			return v_result;
		}

		/**
		 *	@brief overwrites this pose by an exponent of a 7D vector
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *	@param[in] r_v_vec is a vector in \f$\mathfrak{sim}(3)\f$, obtained e.g. by a call to v_Log()
		 *	@deprecated It is preferable to use the constructor with \ref from_sim3_vector.
		 */
		template <class Derived0>
		void Exp(const Eigen::MatrixBase<Derived0> &r_v_vec)
		{
			//DimensionCheck<Eigen::Vector7d>(r_v_vec); // checked inside CSim3Jacobians::t_Exp() as well
			*this = CSim3Jacobians::t_Exp(r_v_vec);
		}

		/**
		 *	@brief calculates composition of two poses, this pose on the left
		 *	@param[in] r_t_other is the other pose to compose
		 */
		void operator *=(const TSim3 &r_t_other)
		{
			_ASSERTE(&r_t_other != this); // r_t_other must be a different object
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			v_translation.noalias() += f_scale * t_rotation._transformVector(r_t_other.v_translation);
			t_rotation *= r_t_other.t_rotation;
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is still notmalized
			f_scale *= r_t_other.f_scale;
		}

		/**
		 *	@brief calculates relative pose \f$this = this \ominus r\_t\_other = this^{-1} \cdot r\_t\_other\f$
		 *	@param[in] r_t_other is the other pose to relate to
		 */
		void Inverse_Compose(const TSim3 &r_t_other)
		{
#ifdef _DEBUG
			TSim3 t_ref = *this;
			t_ref.Invert(); // invert
			t_ref *= r_t_other; // right multiply
			// calculate reference and compare in debug!!!
#endif // _DEBUG

			_ASSERTE(&r_t_other != this); // r_t_other must be a different object
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			_ASSERTE(fabs(r_t_other.t_rotation.norm() - 1) < 1e-5); // make sure r_t_other is notmalized

			f_scale = 1 / f_scale;
			//t_rotation = t_rotation.inverse();
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			t_rotation = t_rotation.conjugate(); // faster but only if normalized
			v_translation = f_scale * t_rotation._transformVector(r_t_other.v_translation - v_translation);
			t_rotation *= r_t_other.t_rotation;
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is still notmalized
			f_scale *= r_t_other.f_scale;
			// saves one quaternion transformation

			/*f_scale /= r_t_other.f_scale;
			Eigen::Quaterniond other_inv_rot = r_t_other.t_rotation.conjugate(); // faster but only if normalized
			v_translation.noalias() += -f_scale * t_rotation._transformVector(other_inv_rot._transformVector(r_t_other.v_translation));
			t_rotation *= other_inv_rot;*/ // the other way around

			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is still notmalized

#ifdef _DEBUG
			_ASSERTE(fabs(t_ref.f_scale - f_scale) < 1e-5);
			_ASSERTE((t_ref.v_translation - v_translation).norm() < 1e-5);
			_ASSERTE((t_ref.t_rotation.coeffs() - t_rotation.coeffs()).norm() < 1e-5);
			// check
#endif // _DEBUG
		}

		/**
		 *	@brief transforms a vector by this pose
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *	@param[in] r_v_vec is a vector to be transformed (either 3D or 4D for homogenous coordinates)
		 *	@return Returns the rotated vectot (preserves dimensionality).
		 */
		template <class Derived0>
		typename CDeriveMatrixType<Derived0>::_TyResult operator *(const Eigen::MatrixBase<Derived0> &r_v_vec) const
		{
			enum {
				n_dim = Eigen::MatrixBase<Derived0>::RowsAtCompileTime
			};
			typedef typename CChooseType<Eigen::Vector3d, Eigen::Vector4d, n_dim == 3>::_TyResult VectorType;
			DimensionCheck<VectorType>(r_v_vec); // make sure that the dimension is correct

			double w = (n_dim == 3)? 1.0 : r_v_vec.template tail<1>()(0); // homogenous coordinate
			VectorType v;
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			v.template head<3>() = f_scale * t_rotation._transformVector(r_v_vec) + w * v_translation; // calculate the transform
			if(n_dim == 4)
				v.template tail<1>()(0) = w; // pass it on

			return v;
		}

		/**
		 *	@brief transforms a vector by an inverse of this pose
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *	@param[in] r_v_vec is a vector to be transformed (either 3D or 4D for homogenous coordinates)
		 *	@return Returns the rotated vectot (preserves dimensionality).
		 */
		template <class Derived0>
		typename CDeriveMatrixType<Derived0>::_TyResult v_InvTransform(const Eigen::MatrixBase<Derived0> &r_v_vec) const
		{
			enum {
				n_dim = Eigen::MatrixBase<Derived0>::RowsAtCompileTime
			};
			typedef typename CChooseType<Eigen::Vector3d, Eigen::Vector4d, n_dim == 3>::_TyResult VectorType;
			DimensionCheck<VectorType>(r_v_vec); // make sure that the dimension is correct

			double w = (n_dim == 3)? 1.0 : r_v_vec.template tail<1>()(0); // homogenous coordinate
			VectorType v;
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			double f_inv_scale = 1 / f_scale;
			v.template head<3>() = f_inv_scale * t_rotation.conjugate()._transformVector(r_v_vec - w * v_translation); // calculate the transform
			if(n_dim == 4)
				v.template tail<1>()(0) = w; // pass it on
			// the same complexity as the forward transformation

#ifdef _DEBUG
			TSim3 t_inv = *this;
			t_inv.Invert();
			double f_error = (t_inv * r_v_vec - v).norm();
			_ASSERTE(f_error < 1e-10);
			// make sure it does what it is supposed to
#endif // _DEBUG

			return v;
		}
	};

	/**
	 *	@brief converts Sim(3) poses from absolute measurement to relative measurement
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex1 is the first vertex, in absolute coordinates
	 *	@param[in] r_v_vertex2 is the second vertex, also in absolute coordinates
	 *	@param[out] r_v_difference is filled with relative coordinates of the second vertex
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Absolute_to_Relative(const Eigen::MatrixBase<Derived0> &r_v_vertex1,
		const Eigen::MatrixBase<Derived1> &r_v_vertex2, Eigen::MatrixBase<Derived2> &r_v_difference)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex1);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex2);
		DimensionCheck<Eigen::Vector7d>(r_v_difference);

		TSim3 vert1, vert2;
		vert1.Exp(r_v_vertex1);
		vert2.Exp(r_v_vertex2);
		// exp map

		vert1.Inverse_Compose(vert2);
		// absolute to relative (vert1^{-1} * vert2)

		r_v_difference = vert1.v_Log();
		// log map
	}

	/**
	 *	@brief composes a pair of relative Sim(3) poses to yield an aboslute transformation
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex1 is the first transformation (in absolute coordinates)
	 *	@param[in] r_v_vertex2 is the second transformation (relative to the first one)
	 *	@param[out] r_v_difference is filled with relative coordinates of the second vertex
	 *	@param[out] r_t_jacobian1 is filled with the first jacobian (delta difference / delta vertex 1)
	 *	@param[out] r_t_jacobian2 is filled with the second jacobian (delta difference / delta vertex 2)
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4>
	static void Absolute_to_Relative(const Eigen::MatrixBase<Derived0> &r_v_vertex1,
		const Eigen::MatrixBase<Derived1> &r_v_vertex2, Eigen::MatrixBase<Derived2> &r_v_difference,
		Eigen::MatrixBase<Derived3> &r_t_jacobian1, Eigen::MatrixBase<Derived4> &r_t_jacobian2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex1);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex2);
		DimensionCheck<Eigen::Vector7d>(r_v_difference);
		DimensionCheck<Eigen::Matrix7d>(r_t_jacobian1);
		DimensionCheck<Eigen::Matrix7d>(r_t_jacobian2);

		TSim3 vert1, vert2;
		vert1.Exp(r_v_vertex1);
		vert2.Exp(r_v_vertex2);
		// exp map

		TSim3 diff = vert1;
		diff.Inverse_Compose(vert2);
		// compose the poses

		r_v_difference = diff.v_Log();
		// log map

		const double f_eps = 1e-9, f_scale = 1 / f_eps;
		Eigen::Vector7d v_eps = Eigen::Vector7d::Zero();
		for(int i = 0; i < 7; ++ i) {
			v_eps(i) = f_eps;
			TSim3 t_eps = t_Exp(v_eps);

			TSim3 t_shift1 = vert1;
			t_shift1 *= t_eps;
			t_shift1.Inverse_Compose(vert2);
			r_t_jacobian1.col(i) = (t_shift1.v_Log() - r_v_difference) * f_scale;

			TSim3 t_shift2 = vert2;
			t_shift2 *= t_eps;
			TSim3 t_diff = vert1; t_diff.Inverse_Compose(t_shift2);
			r_t_jacobian2.col(i) = (t_diff.v_Log() - r_v_difference) * f_scale;

			v_eps(i) = 0; // rezero
		}
		// numerical jacobian
	}

	/**
	 *	@brief composes a pair of relative Sim(3) poses to yield an aboslute transformation
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex1 is the first transformation (in absolute coordinates)
	 *	@param[in] r_v_vertex2 is the second transformation (relative to the first one)
	 *	@param[out] r_v_composition is filled with the composed transformation
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Relative_to_Absolute(const Eigen::MatrixBase<Derived0> &r_v_vertex1,
		const Eigen::MatrixBase<Derived1> &r_v_vertex2, Eigen::MatrixBase<Derived2> &r_v_composition)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex1);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex2);
		DimensionCheck<Eigen::Vector7d>(r_v_composition);

		TSim3 vert1, vert2;
		vert1.Exp(r_v_vertex1);
		vert2.Exp(r_v_vertex2);
		// exp map

		vert1 *= vert2;
		// compose the poses

		r_v_composition = vert1.v_Log();
		// log map
	}

	template <class Derived0>
	static inline Eigen::Vector3d v_InvDepth_to_XYZ(const Eigen::MatrixBase<Derived0> &r_v_position)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_position);

		Eigen::Vector3d v_pos_xyz;
		double f_depth = v_pos_xyz(2) = 1 / r_v_position(2);
		v_pos_xyz.template head<2>() = r_v_position.template head<2>() * f_depth;

		return v_pos_xyz;
	}

	template <class Derived0>
	static inline Eigen::Vector3d v_XYZ_to_InvDepth(const Eigen::MatrixBase<Derived0> &r_v_position)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_position);

		double f_inv_depth = 1 / r_v_position(2);
		Eigen::Vector3d v_inv_depth;
		v_inv_depth.template head<2>() = r_v_position.template head<2>() * f_inv_depth;
		v_inv_depth(2) = f_inv_depth;

		return v_inv_depth;
	}

	template <class Derived0>
	static inline Eigen::Vector3d v_InvDist_to_XYZ(const Eigen::MatrixBase<Derived0> &r_v_position)
	{
		DimensionCheck<Eigen::Vector4d>(r_v_position); // u v w q^{-1}

		double f_q = 1 / r_v_position(3);
		return r_v_position.template head<3>() * f_q; // [x y z] = [u v w] * q
	}

	template <class Derived0>
	static inline Eigen::Vector4d v_XYZ_to_InvDist(const Eigen::MatrixBase<Derived0> &r_v_position)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_position);
		double f_inv_q = 1 / r_v_position.norm();

		Eigen::Vector4d v_inv_dist;
		v_inv_dist.template head<3>() = r_v_position * f_inv_q;
		v_inv_dist(3) = f_inv_q;

		return v_inv_dist;
	}

	template <class Derived0>
	static inline Eigen::Vector3d v_InvDist_to_InvDepth(const Eigen::MatrixBase<Derived0> &r_v_position)
	{
		DimensionCheck<Eigen::Vector4d>(r_v_position); // u v w q^{-1}

		return v_XYZ_to_InvDepth(v_InvDist_to_XYZ(r_v_position));
	}

	template <class Derived0>
	static inline Eigen::Vector4d v_InvDepth_to_InvDist(const Eigen::MatrixBase<Derived0> &r_v_position)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_position);

		return v_XYZ_to_InvDist(v_InvDepth_to_XYZ(r_v_position));
	}

	/**
	 *	@brief composition for XYZ 3D vertices
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex1 is the vertex to be modified (xyz)
	 *	@param[in] r_v_vertex2 is the delta vector (xyz)
	 *	@param[out] r_v_composition is the result of the composition (xyz)
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Relative_to_Absolute_XYZ(const Eigen::MatrixBase<Derived0> &r_v_vertex1,
		const Eigen::MatrixBase<Derived1> &r_v_vertex2, Eigen::MatrixBase<Derived2> &r_v_composition)
	{
		r_v_composition = r_v_vertex1 + r_v_vertex2;
	}

	/**
	 *	@brief composition for inverse depth 3D vertices
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex1 is the vertex to be modified (inverse depth)
	 *	@param[in] r_v_vertex2 is the delta vector (xyz)
	 *	@param[out] r_v_composition is the result of the composition (inverse depth)
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Relative_to_Absolute_InvDepth_XYZ(const Eigen::MatrixBase<Derived0> &r_v_vertex1,
		const Eigen::MatrixBase<Derived1> &r_v_vertex2, Eigen::MatrixBase<Derived2> &r_v_composition)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex1);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex2);
		DimensionCheck<Eigen::Vector3d>(r_v_composition);

#if 0 // does not work well if adding a zero vector, e.g. in numerical differentiation
		/*Eigen::Vector3d v_pos1, v_pos2;
		double f_depth1 = v_pos1(2) = 1 / r_v_vertex1(2);
		v_pos1.template head<2>() = r_v_vertex1.template head<2>() * f_depth1;
		double f_depth2 = v_pos2(2) = 1 / r_v_vertex2(2);
		v_pos2.template head<2>() = r_v_vertex2.template head<2>() * f_depth2;
		// convert from inverse depth to xyz

		v_pos1 += v_pos2;
		// compose

		double f_inv_depth_result = 1 / v_pos1(2);
		r_v_composition.template head<2>() = v_pos1.template head<2>() * f_inv_depth_result;
		r_v_composition(2) = f_inv_depth_result;
		// convert back to inverse depth*/

		r_v_composition = v_XYZ_to_InvDepth(v_InvDepth_to_XYZ(r_v_vertex1) + v_InvDepth_to_XYZ(r_v_vertex2));
#else // 0
		/*Eigen::Vector3d v_pos1;
		double f_depth1 = v_pos1(2) = 1 / r_v_vertex1(2);
		v_pos1.template head<2>() = r_v_vertex1.template head<2>() * f_depth1;
		// convert the first vertex from inverse depth to xyz

		v_pos1 += r_v_vertex2; // the second vertex is in xyz
		// compose

		double f_inv_depth_result = 1 / v_pos1(2);
		r_v_composition.template head<2>() = v_pos1.template head<2>() * f_inv_depth_result;
		r_v_composition(2) = f_inv_depth_result;
		// convert back to inverse depth*/

		r_v_composition = v_XYZ_to_InvDepth(v_InvDepth_to_XYZ(r_v_vertex1) + r_v_vertex2);
#endif // 0
	}

	/**
	 *	@brief composition for inverse depth 3D vertices
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex1 is the vertex to be modified (inverse depth)
	 *	@param[in] r_v_vertex2 is the delta vector (xyz)
	 *	@param[out] r_v_composition is the result of the composition (inverse depth)
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Relative_to_Absolute_InvDepth_Epsilon(const Eigen::MatrixBase<Derived0> &r_v_vertex1,
		const Eigen::MatrixBase<Derived1> &r_v_vertex2, Eigen::MatrixBase<Derived2> &r_v_composition)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex1);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex2);
		DimensionCheck<Eigen::Vector3d>(r_v_composition);

		r_v_composition = r_v_vertex1 + r_v_vertex2;
	}

	/**
	 *	@brief projects an XYZ point from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static bool Project_P2C_XYZ(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const TSim3 t_camera(r_v_vertex_cam, TSim3::from_sim3_vector);
		// get a Sim(3) pose

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz

		Eigen::Vector3d x = t_camera.v_InvTransform(X); // the same cost as forward transform would have
		_ASSERTE((X - t_camera * x).norm() < 1e-10); // debug - make sure it transforms back on itself
		// transform world to camera

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d uv = K * x;
		bool b_in_front_of_camera = uv(2) > 0;
		uv.head<2>() /= uv(2); uv(2) = 1;
		// project camera to image, dehomogenize

		double r2 = (uv.template head<2>() - K.template topRightCorner<2, 1>()).squaredNorm();
		uv.template head<2>() = K.template topRightCorner<2, 1>() +
			(1 + r2 * f_k) * (uv.template head<2>() - K.template topRightCorner<2, 1>());
		// apply radial distortion // todo - make the distortion function a (template?) parameter

#ifdef DATA_UPSIDE_DOWN
		r_v_reprojection(0) = uv(0);
		r_v_reprojection(1) = -uv(1); // because of how image coordinates have orrigin in top left corner usually
#else // DATA_UPSIDE_DOWN
		r_v_reprojection = uv.template head<2>();
#endif // DATA_UPSIDE_DOWN

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[in] b_use_squared_k_normalization is flag to use normalization <tt>k_new = k / (.5 * fx * fy)</tt>
	 *		instead of <tt>k_new = k / (.5 * (fx + fy))</tt>
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static bool Project_P2C_LocalXYZ_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
			//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz

		const Eigen::Vector3d &x = X; // already in the camera
		// transform world to camera

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d uv = K * x;
		bool b_in_front_of_camera = uv(2) > 0;
		uv.head<2>() /= uv(2); uv(2) = 1;
		// project camera to image, dehomogenize

		double r2 = (uv.template head<2>() - K.template topRightCorner<2, 1>()).squaredNorm();
		uv.template head<2>() = K.template topRightCorner<2, 1>() +
			(1 + r2 * f_k) * (uv.template head<2>() - K.template topRightCorner<2, 1>());
		// apply radial distortion // todo - make the distortion function a (template?) parameter

#ifdef DATA_UPSIDE_DOWN
		r_v_reprojection(0) = uv(0);
		r_v_reprojection(1) = -uv(1); // because of how image coordinates have orrigin in top left corner usually
#else // DATA_UPSIDE_DOWN
		r_v_reprojection = uv.template head<2>();
#endif // DATA_UPSIDE_DOWN

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex, also in absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[in] b_use_squared_k_normalization is flag to use normalization <tt>k_new = k / (.5 * fx * fy)</tt>
	 *		instead of <tt>k_new = k / (.5 * (fx + fy))</tt>
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4>
	static bool Project_P2C_LocalXYZ_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived3> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const TSim3 t_camera(r_v_vertex_observing_cam, TSim3::from_sim3_vector);
		const TSim3 t_owner_camera(r_v_vertex_owner_cam, TSim3::from_sim3_vector);
		TSim3 t_diff_transform = t_camera; t_diff_transform.Inverse_Compose(t_owner_camera); // calculate direct transform as inverse composition (camera^{-1} * owner_camera)
		// get a Sim(3) pose

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz

		Eigen::Vector3d x = t_diff_transform * X; // get it from the local space of the owner camera and put it to the local space of the observing camera
		_ASSERTE((x - t_camera.v_InvTransform(t_owner_camera * X)).norm() < 1e-10); // make sure we got the difference transform right
		// transform from one camera to the other camera

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d uv = K * x;
		bool b_in_front_of_camera = uv(2) > 0;
		uv.head<2>() /= uv(2); uv(2) = 1;
		// project camera to image, dehomogenize

		double r2 = (uv.template head<2>() - K.template topRightCorner<2, 1>()).squaredNorm();
		uv.template head<2>() = K.template topRightCorner<2, 1>() +
			(1 + r2 * f_k) * (uv.template head<2>() - K.template topRightCorner<2, 1>());
		// apply radial distortion // todo - make the distortion function a (template?) parameter

#ifdef DATA_UPSIDE_DOWN
		r_v_reprojection(0) = uv(0);
		r_v_reprojection(1) = -uv(1); // because of how image coordinates have orrigin in top left corner usually
#else // DATA_UPSIDE_DOWN
		r_v_reprojection = uv.template head<2>();
#endif // DATA_UPSIDE_DOWN

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse depth point from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in absolute coordinates, inverse depth
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static bool Project_P2C_InvDepth(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth); // other dimension checks repeated later
		return Project_P2C_XYZ(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in camera local coordinates, inverse depth
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static bool Project_P2C_LocalInvDepth_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth); // other dimension checks repeated later
		return Project_P2C_LocalXYZ_Self(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_intrinsics, r_v_reprojection);
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in camera local coordinates, inverse depth
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex, also in absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4>
	static bool Project_P2C_LocalInvDepth_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived3> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth); // other dimension checks repeated later
		return Project_P2C_LocalXYZ_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth),
			r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);
	}

	/**
	 *	@brief projects an inverse distance point from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_dist is the first vertex, in absolute coordinates, inverse distance
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static bool Project_P2C_InvDist(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_dist,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector4d>(r_v_vertex_inv_dist); // other dimension checks repeated later
		return Project_P2C_XYZ(v_InvDist_to_XYZ(r_v_vertex_inv_dist), r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);
	}

	/**
	 *	@brief projects an inverse distance point in camera-local coordinates from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_dist is the first vertex, in camera local coordinates, inverse distance
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static bool Project_P2C_LocalInvDist_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_dist,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector4d>(r_v_vertex_inv_dist); // other dimension checks repeated later
		return Project_P2C_LocalXYZ_Self(v_InvDist_to_XYZ(r_v_vertex_inv_dist), r_v_intrinsics, r_v_reprojection);
	}

	/**
	 *	@brief projects an inverse distance point in camera-local coordinates from 3D to 2D camera image
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_dist is the first vertex, in camera local coordinates, inverse distance
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex, also in absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4>
	static bool Project_P2C_LocalInvDist_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_dist,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived3> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector4d>(r_v_vertex_inv_dist); // other dimension checks repeated later
		return Project_P2C_LocalXYZ_Other(v_InvDist_to_XYZ(r_v_vertex_inv_dist),
			r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);
	}

	/**
	 *	@brief projects a 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5>
	static bool Project_P2C_XYZ(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_XYZ(r_v_vertex_xyz, r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(v_delta, r_v_vertex_cam, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(r_v_vertex_xyz, v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects a 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_reprojection is the 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6>
	static bool Project_P2CI_XYZ(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1,
		Eigen::MatrixBase<Derived6> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_XYZ(r_v_vertex_xyz, r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(v_delta, r_v_vertex_cam, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(r_v_vertex_xyz, v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(r_v_vertex_xyz, r_v_vertex_cam, v_delta, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return true;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static bool Project_P2C_LocalXYZ_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection,
		Eigen::MatrixBase<Derived3> &r_t_J0)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalXYZ_Self(r_v_vertex_xyz, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Self(v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobian

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7>
	static bool Project_P2C_LocalXYZ_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived3> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(v_delta, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, v_delta, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, v_delta, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image (with intrinsics) and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta intrinsics)
	 	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4>
	static bool Project_P2CI_LocalXYZ_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection,
		Eigen::MatrixBase<Derived3> &r_t_J0,
		Eigen::MatrixBase<Derived4> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J1);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix<double, 5, 5> t_epsilon = Eigen::Matrix<double, 5, 5>::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalXYZ_Self(r_v_vertex_xyz, r_v_intrinsics, r_v_reprojection);
		//std::cout << "ref: " << r_v_reprojection.transpose() << std::endl;
		//std::cout << t_epsilon << std::endl;

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			//std::cout << t_epsilon.col(j).template head<3>().transpose() << std::endl;
			Project_P2C_LocalXYZ_Self(v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Self(r_v_vertex_xyz, v_delta, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		//std::cout << "J0:\n" << r_t_J0 << std::endl;
		//std::cout << "J1:\n" << r_t_J1 << std::endl;
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera + intrinsics image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived8 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7, class Derived8>
	static bool Project_P2CI_LocalXYZ_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2,
		Eigen::MatrixBase<Derived8> &r_t_J3)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J2);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J3);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(v_delta, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, v_delta, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, r_v_vertex_observing_cam, v_delta, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, v_delta, v_reprojected_with_delta);
			r_t_J3.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects a InvDepth point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_reprojection is the 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6>
	static bool Project_P2CI_InvDepth(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1,
		Eigen::MatrixBase<Derived6> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_XYZ(v_InvDepth_to_XYZ(r_v_vertex_xyz), r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(v_InvDepth_to_XYZ(v_delta), r_v_vertex_cam, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(v_InvDepth_to_XYZ(r_v_vertex_xyz), v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_XYZ(v_InvDepth_to_XYZ(r_v_vertex_xyz), r_v_vertex_cam, v_delta, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return true;
	}

	/**
	 *	@brief projects an InvDepth point in camera-local coordinates from 3D to 2D camera image (with intrinsics) and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta intrinsics)
		 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4>
	static bool Project_P2CI_LocalInvDepth_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection,
		Eigen::MatrixBase<Derived3> &r_t_J0,
		Eigen::MatrixBase<Derived4> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J1);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix<double, 5, 5> t_epsilon = Eigen::Matrix<double, 5, 5>::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalXYZ_Self(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_intrinsics, r_v_reprojection);
		//std::cout << "ref: " << r_v_reprojection.transpose() << std::endl;
		//std::cout << t_epsilon << std::endl;

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			//std::cout << t_epsilon.col(j).template head<3>().transpose() << std::endl;
			Project_P2C_LocalXYZ_Self(v_InvDepth_to_XYZ(v_delta), r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Self(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		//std::cout << "J0:\n" << r_t_J0 << std::endl;
		//std::cout << "J1:\n" << r_t_J1 << std::endl;
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an InvDepth point in camera-local coordinates from 3D to 2D camera + intrinsics image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived8 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7, class Derived8>
	static bool Project_P2CI_LocalInvDepth_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2,
		Eigen::MatrixBase<Derived8> &r_t_J3)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J2);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J3);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalXYZ_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(v_InvDepth_to_XYZ(v_delta), r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam, v_delta, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam, r_v_intrinsics, v_delta, v_reprojected_with_delta);
			r_t_J3.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse depth 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in absolute coordinates, inverse depth
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta inverse depth point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5>
	static bool Project_P2C_InvDepth(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_InvDepth(r_v_vertex_inv_depth, r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_InvDepth(v_delta, r_v_vertex_cam, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_InvDepth(r_v_vertex_inv_depth, v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse depth point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in camera local coordinates, inverse depth
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static bool Project_P2C_LocalInvDepth_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection,
		Eigen::MatrixBase<Derived3> &r_t_J0)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalInvDepth_Self(r_v_vertex_inv_depth, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDepth_Self(v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobian

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse depth point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in camera local coordinates, inverse depth
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7>
	static bool Project_P2C_LocalInvDepth_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived3> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalInvDepth_Other(r_v_vertex_inv_depth, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDepth_Other(v_delta, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDepth_Other(r_v_vertex_inv_depth, v_delta, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDepth_Other(r_v_vertex_inv_depth, r_v_vertex_observing_cam, r_v_intrinsics, v_delta, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse distance 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_dist is the first vertex, in absolute coordinates, inverse distance
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta inverse distance point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5>
	static bool Project_P2C_InvDist(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_dist,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector4d>(r_v_vertex_inv_dist); // 4D
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 1> >(r_t_J0); // the optimized part of the point is only 1D!
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_dist != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_InvDist(r_v_vertex_inv_dist, r_v_vertex_cam, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 1; ++ j) { // derivative only over the q coordinate
			Eigen::Vector4d v_delta = r_v_vertex_inv_dist;
			v_delta(3) += f_delta; // epsilon is added to q only
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_InvDist(v_delta, r_v_vertex_cam, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_InvDist(r_v_vertex_inv_dist, v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse distance point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_dist is the first vertex, in camera local coordinates, inverse distance
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static bool Project_P2C_LocalInvDist_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_dist,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		Eigen::MatrixBase<Derived2> &r_v_reprojection,
		Eigen::MatrixBase<Derived3> &r_t_J0)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector4d>(r_v_vertex_inv_dist);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 1> >(r_t_J0);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_dist != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalInvDist_Self(r_v_vertex_inv_dist, r_v_intrinsics, r_v_reprojection);

		for(int j = 0; j < 1; ++ j) { // derivative only over the q coordinate
			Eigen::Vector4d v_delta = r_v_vertex_inv_dist;
			v_delta(3) += f_delta; // epsilon is added to q only
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDist_Self(v_delta, r_v_intrinsics, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobian

		return b_in_front_of_camera;
	}

	/**
	 *	@brief projects an inverse distance point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_dist is the first vertex, in camera local coordinates, inverse distance
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7>
	static bool Project_P2C_LocalInvDist_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_dist,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived3> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived4> &r_v_reprojection,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector4d>(r_v_vertex_inv_dist);
		DimensionCheck<Eigen::Vector2d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 2, 1> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_dist != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		bool b_in_front_of_camera = Project_P2C_LocalInvDist_Other(r_v_vertex_inv_dist, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, r_v_reprojection);

		for(int j = 0; j < 1; ++ j) { // derivative only over the q coordinate
			Eigen::Vector4d v_delta = r_v_vertex_inv_dist;
			v_delta(3) += f_delta; // epsilon is added to q only
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDist_Other(v_delta, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDist_Other(r_v_vertex_inv_dist, v_delta, r_v_intrinsics, r_v_vertex_owner_cam, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalInvDist_Other(r_v_vertex_inv_dist, r_v_vertex_observing_cam, r_v_intrinsics, v_delta, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians

		return b_in_front_of_camera;
	}

	/**
	 *	@brief transforms an XYZ point in camera-local coordinates to other camera local coordinates
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex, also in absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the reprojected 2D coordinates of the point
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static void Project_Landmark_Local_to_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived3> &r_v_reprojection)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector3d>(r_v_reprojection);


		const TSim3 t_camera(r_v_vertex_observing_cam, TSim3::from_sim3_vector);
		const TSim3 t_owner_camera(r_v_vertex_owner_cam, TSim3::from_sim3_vector);
		TSim3 t_diff_transform = t_camera;
		t_diff_transform.Inverse_Compose(t_owner_camera); // calculate direct transform as inverse composition (camera^{-1} * owner_camera)
		// get a Sim(3) pose

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz

		r_v_reprojection = t_diff_transform * X; // get it from the local space of the owner camera and put it to the local space of the observing camera
		//_ASSERTE((r_v_reprojection - t_camera.v_InvTransform(t_owner_camera * r_v_reprojection)).norm() < 1e-10);
	}

	/**
	 *	@brief calculates the jacobians of landmark XYZ
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[out] r_v_reprojection is filled with the 3D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Project_Landmark_LocalXYZ_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		Eigen::MatrixBase<Derived1> &r_v_reprojection,
		Eigen::MatrixBase<Derived2> &r_t_J0)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector3d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);

		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix3d t_epsilon = Eigen::Matrix3d::Identity() * f_delta;

		r_v_reprojection = r_v_vertex_xyz;

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			v_delta = r_v_vertex_xyz + t_epsilon.col(j).template head<3>();
			r_t_J0.col(j) = (v_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobian
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  also in absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the transformed 3D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6>
	static void Project_Landmark_LocalXYZ_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1,
		Eigen::MatrixBase<Derived6> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector3d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		Project_Landmark_Local_to_Other(r_v_vertex_xyz, r_v_vertex_observing_cam,
				r_v_vertex_owner_cam, r_v_reprojection);

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz

			Eigen::Vector3d v_reprojected_with_delta;
			Project_Landmark_Local_to_Other(v_delta, r_v_vertex_observing_cam,
					r_v_vertex_owner_cam, v_reprojected_with_delta);

			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().

			Eigen::Vector3d v_reprojected_with_delta;
			Project_Landmark_Local_to_Other(r_v_vertex_xyz, v_delta,
					r_v_vertex_owner_cam, v_reprojected_with_delta);

			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().

			Eigen::Vector3d v_reprojected_with_delta;
			Project_Landmark_Local_to_Other(r_v_vertex_xyz, r_v_vertex_observing_cam,
					v_delta, v_reprojected_with_delta);

			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians
	}

	/**
	 *	@brief calculates the jacobians of Invlandmark
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[out] r_v_reprojection is filled with the 3D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Project_Landmark_LocalInvDepth_Self(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		Eigen::MatrixBase<Derived1> &r_v_reprojection,
		Eigen::MatrixBase<Derived2> &r_t_J0)
	{
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector3d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);

		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix3d t_epsilon = Eigen::Matrix3d::Identity() * f_delta;

		//bool b_in_front_of_camera = Project_P2C_LocalXYZ_Self(r_v_vertex_xyz, r_v_intrinsics, r_v_reprojection);
		r_v_reprojection = v_InvDepth_to_XYZ(r_v_vertex_inv_depth);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_delta_reprojection = v_InvDepth_to_XYZ(v_delta);
			r_t_J0.col(j) = (v_delta_reprojection - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobian
	}

	/**
	 *	@brief projects an invLandmark point in camera-local coordinates from 3D to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  also in absolute coordinates, Sim(3)
	 *	@param[out] r_v_reprojection is filled with the transformed 3D coordinates of the point
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6>
	static void Project_Landmark_LocalInvDepth_Other(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_vertex_owner_cam,
		Eigen::MatrixBase<Derived3> &r_v_reprojection,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1,
		Eigen::MatrixBase<Derived6> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector3d>(r_v_reprojection);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_reprojection);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_reprojection); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		Project_Landmark_Local_to_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam,
				r_v_vertex_owner_cam, r_v_reprojection);

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_InvDepth_Epsilon(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz

			Eigen::Vector3d v_reprojected_with_delta;
			Project_Landmark_Local_to_Other(v_InvDepth_to_XYZ(v_delta), r_v_vertex_observing_cam,
					r_v_vertex_owner_cam, v_reprojected_with_delta);

			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().

			Eigen::Vector3d v_reprojected_with_delta;
			Project_Landmark_Local_to_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta,
					r_v_vertex_owner_cam, v_reprojected_with_delta);

			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().

			Eigen::Vector3d v_reprojected_with_delta;
			Project_Landmark_Local_to_Other(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam,
					v_delta, v_reprojected_with_delta);

			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_reprojection) * f_scaler;
		}
		// calculate numerical jacobians
	}

	/**
	 *	@brief compute angular error
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static void Project_P2C_LocalXYZ_Self_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_expectation,
		Eigen::MatrixBase<Derived3> &r_v_error)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const Eigen::Vector3d &x = r_v_vertex_xyz; // just rename
		// get xyz

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d uv = K * x;
		uv.head<2>() /= uv(2); uv(2) = 1;

		Eigen::Vector3d x_inv;
		CRadialDistortionModel<1, 2, 2> dist(Eigen::Vector2d(f_cx, f_cy), f_k);
		x_inv.head<2>() = dist.v_UnApply(r_v_expectation);
		x_inv(2) = 1;
		// undistort the measurement

		x_inv = K.inverse() * x_inv;
		// get the corresponding view direction

		r_v_error = x.normalized().cross(x_inv.normalized());
		// compute cross product

		//r_v_error(0) = acos(x.dot(x_inv) / (x.norm() * x_inv.norm())); // todo use sqrt(x.squaredNorm() * x_inv.squaredNorm()) to save a sqrt()
		// compute angle between expected vector and actual vector
		//std::cout << r_v_expectation.transpose() << " | " << uv.transpose() << std::endl;
		//std::cout << x.normalized().transpose() << " | " << x_inv.normalized().transpose() << " |S| " << r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief compute angular error
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5>
	static void Project_P2C_LocalXYZ_Other_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		const Eigen::MatrixBase<Derived4> &r_v_expectation,
		Eigen::MatrixBase<Derived5> &r_v_error)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const TSim3 t_camera(r_v_vertex_observing_cam, TSim3::from_sim3_vector);
		const TSim3 t_owner_camera(r_v_vertex_owner_cam, TSim3::from_sim3_vector);
		TSim3 t_diff_transform = t_camera;
		t_diff_transform.Inverse_Compose(t_owner_camera); // calculate direct transform as inverse composition (camera^{-1} * owner_camera)
		// get a Sim(3) pose

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz
		Eigen::Vector3d x = t_diff_transform * X;

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d uv = K * x;
		uv.head<2>() /= uv(2); uv(2) = 1;

		Eigen::Vector3d x_inv;
		CRadialDistortionModel<1, 2, 2> dist(Eigen::Vector2d(f_cx, f_cy), f_k);
		x_inv.head<2>() = dist.v_UnApply(r_v_expectation);
		x_inv(2) = 1;
		// undistort the measurement

		x_inv = K.inverse() * x_inv;
		// get the corresponding view direction

		r_v_error = x.normalized().cross(x_inv.normalized());
		// compute cross product

		//r_v_error(0) = acos(x.dot(x_inv) / (x.norm() * x_inv.norm())); // todo use sqrt(x.squaredNorm() * x_inv.squaredNorm()) to save a sqrt()
		// compute angle between expected vector and actual vector
		//std::cout << r_v_expectation.transpose() << " | " << uv.transpose() << std::endl;
		//std::cout << x.normalized().transpose() << " | " << x_inv.normalized().transpose() << " |O| " << r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image (angle error) (with intrinsics) and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta intrinsics)
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5>
	static void Project_P2CI_LocalXYZ_Self_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_expectation,
		Eigen::MatrixBase<Derived3> &r_v_error,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 5> >(r_t_J1);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix<double, 5, 5> t_epsilon = Eigen::Matrix<double, 5, 5>::Identity() * f_delta;

		Project_P2C_LocalXYZ_Self_AngleErr(r_v_vertex_xyz, r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			//std::cout << t_epsilon.col(j).template head<3>().transpose() << std::endl;
			Project_P2C_LocalXYZ_Self_AngleErr(v_delta, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Self_AngleErr(r_v_vertex_xyz, v_delta, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera + intrinsics image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived8 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived9 is Eigen derived matrix type for the ninth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[in] r_v_expectation is the reprojected 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7, class Derived8, class Derived9>
	static void Project_P2CI_LocalXYZ_Other_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		const Eigen::MatrixBase<Derived4> &r_v_expectation,
		Eigen::MatrixBase<Derived5> &r_v_error,
		Eigen::MatrixBase<Derived6> &r_t_J0,
		Eigen::MatrixBase<Derived7> &r_t_J1,
		Eigen::MatrixBase<Derived8> &r_t_J2,
		Eigen::MatrixBase<Derived9> &r_t_J3)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 3, 5> >(r_t_J2);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J3);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_LocalXYZ_Other_AngleErr(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam,
				r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(v_delta, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(r_v_vertex_xyz, v_delta, r_v_intrinsics, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(r_v_vertex_xyz, r_v_vertex_observing_cam, v_delta, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J2.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, v_delta,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J3.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		// calculate numerical jacobians
	}

	/**
	 *	@brief projects an InvDepth point in camera-local coordinates from 3D to 2D camera image (angle error) (with intrinsics) and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, InvDepth
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta intrinsics)
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5>
	static void Project_P2CI_LocalInvDepth_Self_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_expectation,
		Eigen::MatrixBase<Derived3> &r_v_error,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 5> >(r_t_J1);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix<double, 5, 5> t_epsilon = Eigen::Matrix<double, 5, 5>::Identity() * f_delta;

		Project_P2C_LocalXYZ_Self_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			//std::cout << t_epsilon.col(j).template head<3>().transpose() << std::endl;
			Project_P2C_LocalXYZ_Self_AngleErr(v_InvDepth_to_XYZ(v_delta), r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Self_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
	}

	/**
	 *	@brief projects an InvDepth point in camera-local coordinates from 3D to 2D camera + intrinsics image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived8 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived9 is Eigen derived matrix type for the ninth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, InvDepth
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[in] r_v_expectation is the reprojected 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7, class Derived8, class Derived9>
	static void Project_P2CI_LocalInvDepth_Other_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		const Eigen::MatrixBase<Derived4> &r_v_expectation,
		Eigen::MatrixBase<Derived5> &r_v_error,
		Eigen::MatrixBase<Derived6> &r_t_J0,
		Eigen::MatrixBase<Derived7> &r_t_J1,
		Eigen::MatrixBase<Derived8> &r_t_J2,
		Eigen::MatrixBase<Derived9> &r_t_J3)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 3, 5> >(r_t_J2);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J3);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_LocalXYZ_Other_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam,
				r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(v_InvDepth_to_XYZ(v_delta), r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta, r_v_intrinsics, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam, v_delta, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_observing_cam, r_v_intrinsics, v_delta,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J3.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		// calculate numerical jacobians
	}

	/**
	 *	@brief compute angular error
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4>
	static void Project_P2C_XYZ_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_expectation,
		Eigen::MatrixBase<Derived4> &r_v_error)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const TSim3 t_camera(r_v_vertex_cam, TSim3::from_sim3_vector);
		// get a Sim(3) pose

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz

		Eigen::Vector3d x = t_camera.v_InvTransform(X); // the same cost as forward transform would have
		_ASSERTE((X - t_camera * x).norm() < 1e-10); // debug - make sure it transforms back on itself
		// transform world to camera

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d x_inv;
		CRadialDistortionModel<1, 2, 2> dist(Eigen::Vector2d(f_cx, f_cy), f_k);
		x_inv.head<2>() = dist.v_UnApply(r_v_expectation);
		x_inv(2) = 1;
		// undistort the measurement

		x_inv = K.inverse() * x_inv;
		// get the corresponding view direction

		r_v_error = x.normalized().cross(x_inv.normalized());
		//r_v_error(0) = acos(x.dot(x_inv) / (x.norm() * x_inv.norm())); // todo use sqrt(x.squaredNorm() * x_inv.squaredNorm()) to save a sqrt()

		// compute angle between expected vector and actual vector
	}

	/**
	 *	@brief projects a 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_t_error is error of edge
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6>
	static bool Project_P2C_XYZ_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_expectation,
		Eigen::MatrixBase<Derived4> &r_v_error,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_XYZ_AngleErr(r_v_vertex_xyz, r_v_vertex_cam, r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_delta, r_v_vertex_cam, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(r_v_vertex_xyz, v_delta, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		// calculate numerical jacobians

		return true;
	}

	/**
	 *	@brief projects a 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_t_error is error of edge
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6, class Derived7>
	static bool Project_P2CI_XYZ_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_expectation,
		Eigen::MatrixBase<Derived4> &r_v_error,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 3, 5> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_XYZ_AngleErr(r_v_vertex_xyz, r_v_vertex_cam, r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_delta, r_v_vertex_cam, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(r_v_vertex_xyz, v_delta, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(r_v_vertex_xyz, r_v_vertex_cam, v_delta, r_v_expectation, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		// calculate numerical jacobians

		return true;
	}

	/**
	 *	@brief projects a 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in absolute coordinates, InvDepth
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_t_error is error of edge
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6>
	static bool Project_P2C_InvDepth_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_expectation,
		Eigen::MatrixBase<Derived4> &r_v_error,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_cam, r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(v_delta), r_v_vertex_cam, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		// calculate numerical jacobians

		return true;
	}

	/**
	 *	@brief projects a 3D point to 2D camera image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the eighth matrix argument
	 *
	 *	@param[in] r_v_vertex_inv_depth is the first vertex, in absolute coordinates, InvDepth
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_t_error is error of edge
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) camera)
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5, class Derived6, class Derived7>
	static bool Project_P2CI_InvDepth_AngleErr(const Eigen::MatrixBase<Derived0> &r_v_vertex_inv_depth,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_expectation,
		Eigen::MatrixBase<Derived4> &r_v_error,
		Eigen::MatrixBase<Derived5> &r_t_J0,
		Eigen::MatrixBase<Derived6> &r_t_J1,
		Eigen::MatrixBase<Derived7> &r_t_J2)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_inv_depth);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector3d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 3, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 3, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 3, 5> >(r_t_J2);

		_ASSERTE((const void*)&r_v_vertex_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_inv_depth != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_cam, r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_inv_depth, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(v_delta), r_v_vertex_cam, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), v_delta, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector3d v_reprojected_with_delta;
			Project_P2C_XYZ_AngleErr(v_InvDepth_to_XYZ(r_v_vertex_inv_depth), r_v_vertex_cam, v_delta, r_v_expectation, v_reprojected_with_delta);
			r_t_J2.col(j) = (v_reprojected_with_delta - r_v_error) * f_scaler;
		}
		// calculate numerical jacobians

		return true;
	}

	/**
	 *	@brief compute angular error
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3>
	static void Project_P2C_LocalXYZ_Self_AngleErr2(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_expectation,
		Eigen::MatrixBase<Derived3> &r_v_error)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector2d>(r_v_error);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const Eigen::Vector3d &x = r_v_vertex_xyz; // just rename
		// get xyz

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d x_inv;
		CRadialDistortionModel<1, 2, 2> dist(Eigen::Vector2d(f_cx, f_cy), f_k);
		x_inv.head<2>() = dist.v_UnApply(r_v_expectation);
		x_inv(2) = 1;
		// undistort the measurement

		x_inv = K.inverse() * x_inv;
		// get the corresponding view direction

		x_inv.normalize();
		Eigen::Vector3d dest(0, 0, 1);
		dest.normalize();
		Eigen::Vector3d v = x_inv.cross(dest);
		double c = x_inv.dot(dest);
		double s = v.norm();

		Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
		G(0, 0) = c;
		G(1, 1) = c;
		G(1, 0) = s;
		G(0, 1) = -s;

		Eigen::Matrix3d F;
		F.col(0) = x_inv;
		Eigen::Vector3d v2 = (dest - c * x_inv) / (dest - c * x_inv).norm();
		F.col(1) = v2;
		F.col(2) = dest.cross(x_inv);

		Eigen::Matrix3d R = F * G * F.inverse();
		// finds rotation matrix that transforms vector x_inv to vector [0 0 1]
		//std::cout << R << std::endl;
		//std::cout << (R * x_inv).transpose() << std::endl;

		Eigen::Vector3d u = R * x;
		//std::cout << u.transpose() << std::endl;
		// rotate x by R2, try to get it close to [0 0 1]

		r_v_error = u.head<2>() / u(2);
		//std::cout << r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief compute angular error
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in absolute coordinates, XYZ
	 *	@param[in] r_v_vertex_cam is the second vertex, also in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *
	 *	@return Returns true if the landmark is in front of the camera, otherwise returns false.
	 *
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4, class Derived5>
	static void Project_P2C_LocalXYZ_Other_AngleErr2(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		const Eigen::MatrixBase<Derived4> &r_v_expectation,
		Eigen::MatrixBase<Derived5> &r_v_error)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector2d>(r_v_error);

		const double f_fx = r_v_intrinsics(0);
		const double f_fy = r_v_intrinsics(1);
		const double f_cx = r_v_intrinsics(2);
		const double f_cy = r_v_intrinsics(3);
		const double f_k = r_v_intrinsics(4) / (.5 * (f_fx /*note multiplication*/ * f_fy)); // SOSO: works better for mono
		//r_v_intrinsics(4) / (.5 * (f_fx + f_fy)); // rescale intrinsics// since post-ICRA2015 release the distortion is scaled by focal length in the internal representation

		const TSim3 t_camera(r_v_vertex_observing_cam, TSim3::from_sim3_vector);
		const TSim3 t_owner_camera(r_v_vertex_owner_cam, TSim3::from_sim3_vector);
		TSim3 t_diff_transform = t_camera;
		t_diff_transform.Inverse_Compose(t_owner_camera); // calculate direct transform as inverse composition (camera^{-1} * owner_camera)
		// get a Sim(3) pose

		const Eigen::Vector3d &X = r_v_vertex_xyz; // just rename
		// get xyz
		Eigen::Vector3d x = t_diff_transform * X;

		Eigen::Matrix3d K;
		K << f_fx,  0, f_cx,
			  0, f_fy, f_cy,
			  0,  0,  1;
		// construct camera intrinsics matrix

		Eigen::Vector3d x_inv;
		CRadialDistortionModel<1, 2, 2> dist(Eigen::Vector2d(f_cx, f_cy), f_k);
		x_inv.head<2>() = dist.v_UnApply(r_v_expectation);
		x_inv(2) = 1;
		// undistort the measurement

		x_inv = K.inverse() * x_inv;
		// get the corresponding view direction

		x_inv.normalize();
		Eigen::Vector3d dest(0, 0, 1);
		dest.normalize();
		Eigen::Vector3d v = x_inv.cross(dest);
		double c = x_inv.dot(dest);
		double s = v.norm();

		Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
		G(0, 0) = c;
		G(1, 1) = c;
		G(1, 0) = s;
		G(0, 1) = -s;

		Eigen::Matrix3d F;
		F.col(0) = x_inv;
		Eigen::Vector3d v2 = (dest - c * x_inv) / (dest - c * x_inv).norm();
		F.col(1) = v2;
		F.col(2) = dest.cross(x_inv);

		Eigen::Matrix3d R = F * G * F.inverse();

		Eigen::Vector3d u = R * x;

		r_v_error = u.head<2>() / u(2);
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera image (angle error) (with intrinsics) and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_expectation is the 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta intrinsics)
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5>
	static void Project_P2CI_LocalXYZ_Self_AngleErr2(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived2> &r_v_expectation,
		Eigen::MatrixBase<Derived3> &r_v_error,
		Eigen::MatrixBase<Derived4> &r_t_J0,
		Eigen::MatrixBase<Derived5> &r_t_J1)
	{
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector2d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J1);

		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix<double, 5, 5> t_epsilon = Eigen::Matrix<double, 5, 5>::Identity() * f_delta;

		Project_P2C_LocalXYZ_Self_AngleErr2(r_v_vertex_xyz, r_v_intrinsics, r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			//std::cout << t_epsilon.col(j).template head<3>().transpose() << std::endl;
			Project_P2C_LocalXYZ_Self_AngleErr2(v_delta, r_v_intrinsics, r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Self_AngleErr2(r_v_vertex_xyz, v_delta, r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
	}

	/**
	 *	@brief projects an XYZ point in camera-local coordinates from 3D to 2D camera + intrinsics image and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *	@tparam Derived5 is Eigen derived matrix type for the sixth matrix argument
	 *	@tparam Derived6 is Eigen derived matrix type for the seventh matrix argument
	 *	@tparam Derived7 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived8 is Eigen derived matrix type for the eighth matrix argument
	 *	@tparam Derived9 is Eigen derived matrix type for the ninth matrix argument
	 *
	 *	@param[in] r_v_vertex_xyz is the first vertex, in camera local coordinates, XYZ
	 *	@param[in] r_v_vertex_observing_cam is the camera observing the vertex, in absolute coordinates, Sim(3)
	 *	@param[in] r_v_intrinsics is instrinsic parameters of the observing camera
	 *	@param[in] r_v_vertex_owner_cam is the camera that owns the vertex,  alsoin absolute coordinates, Sim(3)
	 *	@param[in] r_v_expectation is the reprojected 2D coordinates of the point
	 *	@param[out] r_v_error is filled with the computed error
	 *	@param[out] r_t_J0 is filled with the first jacobian (delta reprojection / delta xyz point)
	 *	@param[out] r_t_J1 is filled with the second jacobian (delta reprojection / delta Sim(3) observing camera)
	 *	@param[out] r_t_J2 is filled with the second jacobian (delta reprojection / delta Sim(3) owner camera)
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 *	@note This has a different order of arguments, compared to CBAJacobians::Project_P2C(),
	 *		in order to avoid having to swap the order of arguments.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3,
		class Derived4, class Derived5, class Derived6, class Derived7, class Derived8, class Derived9>
	static void Project_P2CI_LocalXYZ_Other_AngleErr2(const Eigen::MatrixBase<Derived0> &r_v_vertex_xyz,
		const Eigen::MatrixBase<Derived1> &r_v_vertex_observing_cam,
		const Eigen::MatrixBase<Derived2> &r_v_intrinsics,
		const Eigen::MatrixBase<Derived3> &r_v_vertex_owner_cam,
		const Eigen::MatrixBase<Derived4> &r_v_expectation,
		Eigen::MatrixBase<Derived5> &r_v_error,
		Eigen::MatrixBase<Derived6> &r_t_J0,
		Eigen::MatrixBase<Derived7> &r_t_J1,
		Eigen::MatrixBase<Derived8> &r_t_J2,
		Eigen::MatrixBase<Derived9> &r_t_J3)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_observing_cam);
		DimensionCheck<Eigen::Vector5d>(r_v_intrinsics);
		DimensionCheck<Eigen::Vector7d>(r_v_vertex_owner_cam);
		DimensionCheck<Eigen::Vector3d>(r_v_vertex_xyz);
		DimensionCheck<Eigen::Vector2d>(r_v_expectation);
		DimensionCheck<Eigen::Vector2d>(r_v_error);
		DimensionCheck<Eigen::Matrix<double, 2, 3> >(r_t_J0);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J1);
		DimensionCheck<Eigen::Matrix<double, 2, 5> >(r_t_J2);
		DimensionCheck<Eigen::Matrix<double, 2, 7> >(r_t_J3);

		_ASSERTE((const void*)&r_v_vertex_observing_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_intrinsics != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_owner_cam != (const void*)&r_v_error);
		_ASSERTE((const void*)&r_v_vertex_xyz != (const void*)&r_v_error); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Project_P2C_InvDepth() and then the jacobians are not computed correcly

		const double f_delta = 1e-9;
		const double f_scaler = 1.0 / f_delta;
		const Eigen::Matrix7d t_epsilon = Eigen::Matrix7d::Identity() * f_delta;

		Project_P2C_LocalXYZ_Other_AngleErr2(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam,
				r_v_expectation, r_v_error);

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d v_delta;
			Relative_to_Absolute_XYZ(r_v_vertex_xyz, t_epsilon.col(j).template head<3>(), v_delta); // epsilon is in xyz
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr2(v_delta, r_v_vertex_observing_cam, r_v_intrinsics, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J0.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_observing_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr2(r_v_vertex_xyz, v_delta, r_v_intrinsics, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J1.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 5; ++ j) {
			Eigen::Vector5d v_delta;
			CBAJacobians::Relative_to_Absolute_Intrinsics(r_v_intrinsics, t_epsilon.col(j).template head<5>(), v_delta);
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr2(r_v_vertex_xyz, r_v_vertex_observing_cam, v_delta, r_v_vertex_owner_cam,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J2.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		for(int j = 0; j < 7; ++ j) {
			Eigen::Vector7d v_delta;
			Relative_to_Absolute(r_v_vertex_owner_cam, t_epsilon.col(j), v_delta); // post-multiplicative update! have to swap the first two arguments for pre-multiplicative, and equally in CVertexCamSim3::Operator_Plus() and CVertexCamSim3::Operator_Minus().
			Eigen::Vector2d v_reprojected_with_delta;
			Project_P2C_LocalXYZ_Other_AngleErr2(r_v_vertex_xyz, r_v_vertex_observing_cam, r_v_intrinsics, v_delta,
					r_v_expectation, v_reprojected_with_delta);
			r_t_J3.col(j) = (r_v_error - v_reprojected_with_delta) * f_scaler;
		}
		// calculate numerical jacobians
	}

private:
	// the rest is internal implementation

	class CSim3Tests {
	protected:
		std::vector<Eigen::Vector7d> sim3_vecs;

	public:
		void Add_TestData(const Eigen::Vector7d &r_v_vec) // throw(std::bad_alloc&)
		{
			sim3_vecs.push_back(r_v_vec);
		}

		~CSim3Tests()
		{
			const Eigen::Vector7d log_identity = Eigen::Vector7d::Zero();//Eigen::Matrix7d::Identity().row(6);
			// 0 0 0 0 0 0 0

			for(size_t i = 0, n = sim3_vecs.size(); i < n; ++ i) {
				const Eigen::Vector7d &v_a = sim3_vecs[i];
				// get a vector

				TSim3 t_pose_a;
				t_pose_a.Exp(v_a);
				// exp it

				double f_log_err = (t_pose_a.v_Log() - v_a).norm();
				_ASSERTE(f_log_err < 1e-10);
				// test log (on a is enough)

				TSim3 t_pose_a_inv = t_pose_a;
				t_pose_a_inv.Invert();
				TSim3 t_identity = t_pose_a;
				t_identity *= t_pose_a_inv;
				_ASSERTE(t_identity.v_translation.norm() < 1e-10);
				_ASSERTE(t_identity.t_rotation.vec().norm() < 1e-10);
				_ASSERTE(fabs(fabs(t_identity.t_rotation.w()) - 1) < 1e-10);
				_ASSERTE(fabs(t_identity.f_scale - 1) < 1e-10);
				_ASSERTE((t_identity.v_Log() - log_identity).norm() < 1e-10);
				// make sure that something times its inverse is identity

				TSim3 t_identity2 = t_pose_a;
				t_identity2.Inverse_Compose(t_pose_a); // a^{-1} * a
				_ASSERTE(t_identity2.v_translation.norm() < 1e-10);
				_ASSERTE(t_identity2.t_rotation.vec().norm() < 1e-10);
				_ASSERTE(fabs(fabs(t_identity2.t_rotation.w()) - 1) < 1e-10);
				_ASSERTE(fabs(t_identity2.f_scale - 1) < 1e-10);
				_ASSERTE((t_identity2.v_Log() - log_identity).norm() < 1e-10);
				// make sure that inverse of something times itself is also identity

				// todo - test 3D point transforms

				for(size_t j = 0; j < n; ++ j) {
					const Eigen::Vector7d &v_b = sim3_vecs[j];
					// get another vector

					TSim3 t_pose_b;
					t_pose_b.Exp(v_b);
					// exp it

					t_pose_b.Inverse_Compose(t_pose_a);
					// test the minus operator (debugs itself)
				}
			}
		}
	};

	static void Test_Sim3(const Eigen::Vector7d &r_v_vec)
	{
		static CSim3Tests test; // gather some Sim(3) poses to do pairwise tests on
		test.Add_TestData(r_v_vec);
	}

#ifdef HAVE_TOON

	static void Test_LnExp(const Eigen::Vector7d &r_v_vec)
	{
		TooN::Vector<7, double> toms_vec;
		(Eigen::Map<Eigen::Vector7d>((&toms_vec[0]))) = r_v_vec;
		TooN::SIM3<double> toms_pose = TooN::SIM3<double>::exp(toms_vec);
		//TooN::SE3<double> toms_pose_se3 = TooN::SE3<double>::exp(toms_vec.slice<0, 6>());
		//TooN::Vector<6, double> toms_vec_back_se3 = toms_pose_se3.ln();
		TooN::Vector<7, double> toms_vec_back = toms_pose.ln();
		Eigen::Map<Eigen::Vector7d> v_toms_vec_back(&toms_vec_back[0]);
		double f_toms_error = (v_toms_vec_back - r_v_vec).norm();

		/*std::cout << "sim3.translation = " << Eigen::Map<Eigen::Vector3d>(&toms_pose.get_translation()[0]).transpose() << std::endl;
		std::cout << "se3.translation = " << Eigen::Map<Eigen::Vector3d>(&toms_pose_se3.get_translation()[0]).transpose() << std::endl;*/

		TSim3 t_pose = t_Exp(r_v_vec);
		Eigen::Vector7d v_vec_back = v_Log(t_pose);
		double f_error = (v_vec_back - r_v_vec).norm();

		double f_scale_error = fabs(toms_pose.get_scale() - t_pose.f_scale);
		double f_pos_error = (Eigen::Map<Eigen::Vector3d>(&toms_pose.get_translation()[0]) - t_pose.v_translation).norm();
		/*Eigen::Matrix3d my_matrix = t_pose.t_rotation.toRotationMatrix();
		Eigen::Matrix3d toms_matrix = Eigen::Map<const Eigen::Matrix3d>(&toms_pose_se3.get_rotation().get_matrix()[0][0]);
		double f_rot_error_t = (my_matrix.transpose() - toms_matrix).norm();*/
		double f_rot_error = (Eigen::Map<const Eigen::Matrix3d>(&toms_pose.get_rotation().get_matrix()[0][0]).transpose() - t_pose.t_rotation.toRotationMatrix()).norm();

		printf("%15g %15g %15g %15g\n", f_toms_error, f_error, f_pos_error, f_rot_error);
	}

#endif // HAVE_TOON

	// works
	/**
	 *	@brief converts a Sim3 coordinate frame to a 7D vector
	 *	@param[in] r_t_pose is Sim3 coordinate frame
	 *	@return Returns the corresponding 7D vector containing 3D translation component, 3D axis angle rotation and 1D scale.
	 */
	static Eigen::Vector7d v_Log(const TSim3 &r_t_pose)
	{
		Eigen::Vector7d t_result;
		Eigen::VectorBlock<Eigen::Block<Eigen::Vector7d, 4, 1>, 3> v_rot_part = t_result.tail<4>().head<3>(); // g++ requires a temporary
		double f_angle = C3DJacobians::f_Quat_to_AxisAngle(r_t_pose.t_rotation, v_rot_part);
		double f_exp_scale = r_t_pose.f_scale;
		double f_scale = log(f_exp_scale);
		t_result(6) = f_scale;

		//	Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk),
		//	Ed Rosten (er258@cam.ac.uk), Gerhard Reitmayr (gr281@cam.ac.uk)

		//	All rights reserved.
		//
		//	Redistribution and use in source and binary forms, with or without
		//	modification, are permitted provided that the following conditions
		//	are met:
		//	1. Redistributions of source code must retain the above copyright
		//	   notice, this list of conditions and the following disclaimer.
		//	2. Redistributions in binary form must reproduce the above copyright
		//	   notice, this list of conditions and the following disclaimer in the
		//	   documentation and/or other materials provided with the distribution.
		//
		//	THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND OTHER CONTRIBUTORS ``AS IS''
		//	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
		//	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
		//	ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR OTHER CONTRIBUTORS BE
		//	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
		//	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
		//	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
		//	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
		//	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
		//	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
		//	POSSIBILITY OF SUCH DAMAGE.

		const double f_epsilon = 1e-6;
		double f_a, f_b, f_c;
		if(fabs(f_scale) < f_epsilon && fabs(f_angle) < f_epsilon) {
			f_a = 1 + f_scale / 2 + f_scale * f_scale / 6;
			f_b = 1.0 / 2 + f_scale / 3 - f_angle * f_angle / 24 + f_scale * f_scale / 8;
			f_c = 1.0 / 6 + f_scale / 8 - f_angle * f_angle / 120 + f_scale * f_scale / 20; // untested
		} else if(fabs(f_scale) < f_epsilon) {
			f_a = 1 + f_scale / 2 + f_scale * f_scale / 6;
			f_b = (1 - cos(f_angle)) / (f_angle * f_angle) + (sin(f_angle) - cos(f_angle) * f_angle) * f_scale /
				(f_angle * f_angle * f_angle) + (2 * sin(f_angle) * f_angle - f_angle * f_angle * cos(f_angle) -
				2 + 2 * cos(f_angle)) * f_scale * f_scale / (2 * f_angle * f_angle * f_angle * f_angle);
			f_c = (f_angle - sin(f_angle)) / (f_angle * f_angle * f_angle) - (-f_angle * f_angle - 2 + 2 *
				cos(f_angle) + 2 * sin(f_angle) * f_angle) * f_scale / (2 * f_angle * f_angle * f_angle * f_angle) -
				(-f_angle * f_angle * f_angle + 6 * cos(f_angle) * f_angle + 3 * sin(f_angle) * f_angle * f_angle -
				6 * sin(f_angle)) * f_scale * f_scale / (6 * f_angle * f_angle * f_angle * f_angle * f_angle); // untested
		} else if(fabs(f_angle) < f_epsilon) {
			f_a = (f_exp_scale - 1) / f_scale;
			f_b = (f_scale * f_exp_scale + 1 - f_exp_scale) / (f_scale * f_scale) - (6 * f_scale * f_exp_scale +
				6 - 6 * f_exp_scale + f_exp_scale * f_scale * f_scale * f_scale - 3 * f_exp_scale * f_scale *
				f_scale) * f_angle * f_angle / (6 * f_scale * f_scale * f_scale * f_scale);
			f_c = (f_exp_scale * f_scale * f_scale - 2 * f_scale * f_exp_scale + 2 * f_exp_scale - 2) / (2 *
				f_scale * f_scale * f_scale) - (f_exp_scale * f_scale * f_scale * f_scale * f_scale - 4 *
				f_exp_scale * f_scale * f_scale * f_scale + 12 * f_exp_scale * f_scale * f_scale - 24 *
				f_scale * f_exp_scale + 24 * f_exp_scale - 24) * f_angle * f_angle / (24 * f_scale * f_scale *
				f_scale * f_scale * f_scale); // mostly untested
		} else {
			double f_u = f_exp_scale * sin(f_angle), f_v = f_exp_scale * cos(f_angle);
			double f_w = 1 / (f_scale * f_scale + f_angle * f_angle);
			f_a = (f_exp_scale - 1) / f_scale;
			f_b = (f_u * f_scale + (1 - f_v) * f_angle) * f_w / f_angle;
			f_c = (f_a - ((f_v - 1) * f_scale + f_u * f_angle) * f_w) / (f_angle * f_angle);
		}
		// shamelessly ripped off of TooN

		Eigen::Vector7d &r = t_result; // shorten
		Eigen::Matrix3d t_cross;
		t_cross <<  0, -r(5),  r(4),
			     r(5),     0, -r(3),
			    -r(4),  r(3),     0;
		t_result.head<3>() = (Eigen::Matrix3d::Identity() * f_a + t_cross * f_b +
			t_cross * t_cross * f_c).householderQr().solve(r_t_pose.v_translation);
		// construct matrix form for V in exp() and solve for log()

		return t_result;
	}

	// works well
	/**
	 *	@brief converts a 7D vector to a Sim3 coordinate frame
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *	@param[in] r_v_vec is a 7D vector containing 3D translation component, 3D axis angle rotation and 1D scale
	 *	@return Returns the corresponding Sim3 coordinate frame.
	 */
	template <class Derived>
	static TSim3 t_Exp(const Eigen::MatrixBase<Derived> &r_v_vec)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vec); // must be a 7D vector

		Eigen::Vector3d t = r_v_vec.template head<3>();
		Eigen::Vector3d w = r_v_vec.template tail<4>().template head<3>();
		// avoid recomputation in case r_v_exp is an expression

		const double f_angle = w.norm();
		const double f_scale = r_v_vec(6);
		const double f_exp_scale = exp(f_scale);

		//	Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk),
		//	Ed Rosten (er258@cam.ac.uk), Gerhard Reitmayr (gr281@cam.ac.uk)

		//	All rights reserved.
		//
		//	Redistribution and use in source and binary forms, with or without
		//	modification, are permitted provided that the following conditions
		//	are met:
		//	1. Redistributions of source code must retain the above copyright
		//	   notice, this list of conditions and the following disclaimer.
		//	2. Redistributions in binary form must reproduce the above copyright
		//	   notice, this list of conditions and the following disclaimer in the
		//	   documentation and/or other materials provided with the distribution.
		//
		//	THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND OTHER CONTRIBUTORS ``AS IS''
		//	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
		//	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
		//	ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR OTHER CONTRIBUTORS BE
		//	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
		//	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
		//	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
		//	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
		//	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
		//	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
		//	POSSIBILITY OF SUCH DAMAGE.

		const double f_epsilon = 1e-6;
		double f_a, f_b, f_c;
		if(fabs(f_scale) < f_epsilon && fabs(f_angle) < f_epsilon) {
			f_a = 1 + f_scale / 2 + f_scale * f_scale / 6;
			f_b = 1.0 / 2 + f_scale / 3 - f_angle * f_angle / 24 + f_scale * f_scale / 8;
			f_c = 1.0 / 6 + f_scale / 8 - f_angle * f_angle / 120 + f_scale * f_scale / 20; // untested
		} else if(fabs(f_scale) < f_epsilon) {
			f_a = 1 + f_scale / 2 + f_scale * f_scale / 6;
			f_b = (1 - cos(f_angle)) / (f_angle * f_angle) + (sin(f_angle) - cos(f_angle) * f_angle) * f_scale /
				(f_angle * f_angle * f_angle) + (2 * sin(f_angle) * f_angle - f_angle * f_angle * cos(f_angle) -
				2 + 2 * cos(f_angle)) * f_scale * f_scale / (2 * f_angle * f_angle * f_angle * f_angle);
			f_c = (f_angle - sin(f_angle)) / (f_angle * f_angle * f_angle) - (-f_angle * f_angle - 2 + 2 *
				cos(f_angle) + 2 * sin(f_angle) * f_angle) * f_scale / (2 * f_angle * f_angle * f_angle * f_angle) -
				(-f_angle * f_angle * f_angle + 6 * cos(f_angle) * f_angle + 3 * sin(f_angle) * f_angle * f_angle -
				6 * sin(f_angle)) * f_scale * f_scale / (6 * f_angle * f_angle * f_angle * f_angle * f_angle); // untested
		} else if(fabs(f_angle) < f_epsilon) {
			f_a = (f_exp_scale - 1) / f_scale;
			f_b = (f_scale * f_exp_scale + 1 - f_exp_scale) / (f_scale * f_scale) - (6 * f_scale * f_exp_scale +
				6 - 6 * f_exp_scale + f_exp_scale * f_scale * f_scale * f_scale - 3 * f_exp_scale * f_scale *
				f_scale) * f_angle * f_angle / (6 * f_scale * f_scale * f_scale * f_scale);
			f_c = (f_exp_scale * f_scale * f_scale - 2 * f_scale * f_exp_scale + 2 * f_exp_scale - 2) / (2 *
				f_scale * f_scale * f_scale) - (f_exp_scale * f_scale * f_scale * f_scale * f_scale - 4 *
				f_exp_scale * f_scale * f_scale * f_scale + 12 * f_exp_scale * f_scale * f_scale - 24 *
				f_scale * f_exp_scale + 24 * f_exp_scale - 24) * f_angle * f_angle / (24 * f_scale * f_scale *
				f_scale * f_scale * f_scale); // mostly untested
		} else {
			double f_u = f_exp_scale * sin(f_angle), f_v = f_exp_scale * cos(f_angle);
			double f_w = 1 / (f_scale * f_scale + f_angle * f_angle);
			f_a = (f_exp_scale - 1) / f_scale;
			f_b = (f_u * f_scale + (1 - f_v) * f_angle) * f_w / f_angle; // (exp(s) * sin(theta) * s + (1 - exp(s) * cos(theta)) * theta) / (s * s + theta * theta) / theta
			f_c = (f_a - ((f_v - 1) * f_scale + f_u * f_angle) * f_w) / (f_angle * f_angle);
		}
		// shamelessly ripped off of TooN

		TSim3 t_result;
		C3DJacobians::AxisAngle_to_Quat(w, t_result.t_rotation);
		Eigen::Vector3d v_cross = w.cross(t);
		t_result.v_translation = f_a * t + f_b * v_cross + f_c * w.cross(v_cross);
		t_result.f_scale = f_exp_scale;

		return t_result;
	}

	// works well
	static Eigen::Vector7d v_Log_SE3(const TSim3 &r_t_pose) // handling the rotation as SE(3)
	{
		Eigen::Vector7d t_result;
		Eigen::VectorBlock<Eigen::Block<Eigen::Vector7d, 4, 1>, 3> v_rot_part = t_result.tail<4>().head<3>(); // g++ requires a temporary
		double f_angle = C3DJacobians::f_Quat_to_AxisAngle(r_t_pose.t_rotation, v_rot_part);
		_ASSERTE(f_angle >= 0); // make sure that the angle returned is positive
		t_result(6) = log(r_t_pose.f_scale);

#if 1
		//f_angle = std::min(f_angle, 2 * M_PI - 1e-12); // the angle should be generally below 2pi, but if it approaches, we want the limit from the left (a large positive number)
		_ASSERTE(f_angle < 2 * M_PI - 1e-3); // make sure this does not approach 2pi
		// will run into trouble if f_angle is a multiple of 2pi, but the angles of rotation
		// vectors converted from a quaternion are [-pi, pi] so that should be safe

		const double f_a = -.5, f_b = (f_angle > 1e-12)? (1 - .5 * f_angle /
			tan(.5 * f_angle)) / (f_angle * f_angle) : 1.0 / 12;
		Eigen::Vector3d v_cross = v_rot_part.cross(r_t_pose.v_translation);
		t_result.head<3>() = r_t_pose.v_translation + f_a * v_cross + f_b * v_rot_part.cross(v_cross);
		// log map
#else // 1
		//	Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk),
		//	Ed Rosten (er258@cam.ac.uk), Gerhard Reitmayr (gr281@cam.ac.uk)

		//	All rights reserved.
		//
		//	Redistribution and use in source and binary forms, with or without
		//	modification, are permitted provided that the following conditions
		//	are met:
		//	1. Redistributions of source code must retain the above copyright
		//	   notice, this list of conditions and the following disclaimer.
		//	2. Redistributions in binary form must reproduce the above copyright
		//	   notice, this list of conditions and the following disclaimer in the
		//	   documentation and/or other materials provided with the distribution.
		//
		//	THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND OTHER CONTRIBUTORS ``AS IS''
		//	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
		//	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
		//	ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR OTHER CONTRIBUTORS BE
		//	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
		//	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
		//	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
		//	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
		//	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
		//	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
		//	POSSIBILITY OF SUCH DAMAGE.

		double f_sinc_half_angle = (f_angle > 1e-5)? sin(.5 * f_angle) / f_angle : .5; // todo - modify threshold for double
		Eigen::Quaterniond t_half_rot = r_t_pose.t_rotation;
		{
			double f_identity_w = 1;
			_ASSERTE(Eigen::Quaterniond::Identity().w() == 1); // positive real expected
			if(t_half_rot.w() < 0) //if(t_half_rot.dot(Eigen::Quaterniond::Identity()) < 0)
				f_identity_w = -1.0; // make sure this has positive dot with identity
			double f_lerp = (f_angle > 1e-6)? sin(.5 * f_angle) / sin(f_angle) : .5; // http://www.wolframalpha.com/input/?i=lim+sin(x/2)/sin(x)&dataset= // this is wrong, the angle should be half of this! (changes the interpolation (non)linearity slightly)
			if(f_lerp > 1e6)
				f_lerp = 1e6; // untested
			else if(f_lerp < -1e6)
				f_lerp = -1e6; // untested
			// will run into trouble if the rotation approaches pi - then the half-rotation is arbitrary
			t_half_rot.coeffs() *= f_lerp; // identity is zero in xyz, no need to add there
			t_half_rot.w() += f_identity_w * f_lerp; // add 1 * f_lerp here
			// slerp(identity, q, .5) = q * sin((1 - .5) * angle) + identity * sin(.5 * angle)
			t_half_rot.normalize(); // in case we are near pi, the output can become denormalized (this will normalize it to full rotation though)
		}
		// make a half rotation - or simply C3DJacobians::Quat_to_AxisAngle(r_t_pose.t_rotation * .5, t_half_rot);

		Eigen::Vector3d rottrans = t_half_rot.conjugate()._transformVector(r_t_pose.v_translation);
		// rotate half way back

		if(f_angle > 1e-3) { // todo - modify threshold for double
			rottrans -= t_result.tail<4>().head<3>() *
				(r_t_pose.v_translation.dot(t_result.tail<4>().head<3>()) *
				(1 - 2 * f_sinc_half_angle) / (f_angle * f_angle));
		} else {
			rottrans -= t_result.tail<4>().head<3>() *
				(r_t_pose.v_translation.dot(t_result.tail<4>().head<3>()) / 24); // mostly untested
		}

		t_result.head<3>() = rottrans / (2 * f_sinc_half_angle);
#endif // 1

		return t_result;
	}

	// works well
	/**
	 *	@brief converts a 7D vector to a Sim3 coordinate frame but handling the rotation as SE(3)
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *	@param[in] r_v_vec is a 7D vector containing 3D translation component, 3D axis angle rotation and 1D scale
	 *	@return Returns the corresponding Sim3 coordinate frame.
	 */
	template <class Derived>
	static TSim3 t_Exp_SE3(const Eigen::MatrixBase<Derived> &r_v_vec)
	{
		DimensionCheck<Eigen::Vector7d>(r_v_vec); // must be a 7D vector

		Eigen::Vector3d t = r_v_vec.template head<3>();
		Eigen::Vector3d w = r_v_vec.template tail<4>().template head<3>();
		// avoid recomputation in case r_v_exp is an expression

		const double f_theta_sq = w.squaredNorm();
		const double f_theta = sqrt(f_theta_sq);

		TSim3 t_result;

		//	Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk),
		//	Ed Rosten (er258@cam.ac.uk), Gerhard Reitmayr (gr281@cam.ac.uk)

		//	All rights reserved.
		//
		//	Redistribution and use in source and binary forms, with or without
		//	modification, are permitted provided that the following conditions
		//	are met:
		//	1. Redistributions of source code must retain the above copyright
		//	   notice, this list of conditions and the following disclaimer.
		//	2. Redistributions in binary form must reproduce the above copyright
		//	   notice, this list of conditions and the following disclaimer in the
		//	   documentation and/or other materials provided with the distribution.
		//
		//	THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND OTHER CONTRIBUTORS ``AS IS''
		//	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
		//	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
		//	ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR OTHER CONTRIBUTORS BE
		//	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
		//	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
		//	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
		//	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
		//	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
		//	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
		//	POSSIBILITY OF SUCH DAMAGE.

		Eigen::Vector3d v_cross = w.cross(t);
		if(f_theta_sq < 1e-8)
			t_result.v_translation = t + .5 * v_cross; // mostly untested
		else if(f_theta_sq < 1e-6) { // todo - adjust this for doubles
			double f_b = 0.5 - 0.25 * f_theta_sq / 6;
			double f_c = 1.0 / 6 - f_theta_sq / 120;
			t_result.v_translation = t + f_b * v_cross + f_c * w.cross(v_cross); // untested
		} else {
			double f_inv_theta = 1 / f_theta;
			double f_a = sin(f_theta) * f_inv_theta; // sin(theta) / theta
			f_inv_theta *= f_inv_theta;
			double f_b = (1 - cos(f_theta)) * f_inv_theta; // (1 - cos(theta)) / theta^2
			double f_c = (1 - f_a) * f_inv_theta; // (1 - sin(theta) / theta) / theta^2 = (theta - sin(theta)) / theta^3
			t_result.v_translation = t + f_b * v_cross + f_c * w.cross(v_cross); // matches the A(w)t from Bullo95, the first equation on page 5
			// f_b / f_a = tan(theta / 2) / theta = (1 - cos(theta)) / (theta * sin(theta))
			// f_c / f_b = (theta - sin(theta)) / (theta * (1 - cos(theta)))
		}
		t_result.f_scale = exp(r_v_vec(6));
		C3DJacobians::AxisAngle_to_Quat(w, t_result.t_rotation); // fast, reasonably precise

		/*

		rotation of a vector r by a quaternion w q (w is real, q is imaginary part of the quat) is:

		r' = r + 2w(q � r) + 2q � (q � r)	// � is cross product, multiplications hidden

		if theta is the rotation angle, w = cos(theta / 2)

		r' = r + 2cos(theta / 2)(q � r) + 2q � (q � r)

		let a be axis-angle vector, a = q theta / sin(theta / 2), then q = a sin(theta / 2) / theta

		r' = r + 2cos(theta / 2) sin(theta / 2) / theta (a � r) + 2 * (sin(theta / 2))^2 / theta^2 a � (a � r)

		r' = r + A (a � r) + B a � (a � r)	// where A = 2cos(theta / 2) sin(theta / 2) / theta = sin(theta) / theta, B = 2 (sin(theta / 2))^2 / theta^2 = (1 - cos(theta)) / theta^2 (http://www.wolframalpha.com/input/?i=2*cos(theta+/+2)*+sin(theta+/+2)+/+theta&dataset= and http://www.wolframalpha.com/input/?i=2+*+(sin(theta+/+2))^2+/+theta^2&dataset=)

		for interpolation by a multiple t of a quaternion w q, one can do slerp of unit quaternion sign(w) 0 (identity) and w q ("one" rotation)

		omega = acos(dot(w q, sign(w) 0)) = acos(w * sign(w)) = acos(abs(w)) = theta / 2

		w' q' = sin((1 - t) omega) / sin(omega) (w q) + sin(t omega) / sin(omega) (sign(w) 0)

		w' = sin((1 - t) omega) / sin(omega) w + sin(t omega) / sin(omega) sign(w)

		w' = sin((1 - t) theta / 2) / sin(theta / 2) w + sin(t theta / 2) / sin(theta / 2) sign(w)

		w' = sin((1 - t) theta / 2) / sin(theta / 2) w + sin(t theta / 2) / sin(theta / 2)	// provided that w > 0

		q' = sin((1 - t) theta / 2) / sin(theta / 2) q

		a' = q' * theta / sin(theta / 2)

		a' = sin((1 - t) theta / 2) / sin(theta / 2) theta / sin(theta / 2) q

		q' = sin((1 - t) theta / 2) / sin(theta / 2) a sin(theta / 2) / theta

		q' = sin((1 - t) theta / 2) / theta a

		w' = sin((1 - t) theta / 2) / sin(theta / 2) cos(theta / 2) + sin(t theta / 2) / sin(theta / 2) sign(cos(theta / 2))

		r' = r + 2 * sin((1 - t) theta / 2) / sin(theta / 2) cos(theta / 2) + sin(t theta / 2) / sin(theta / 2) sign(cos(theta / 2)) * (q � r) + 2q � (q � r)

		r' = r + (2 * sin((1 - t) theta / 2) / sin(theta / 2) cos(theta / 2) + sin(t theta / 2) / sin(theta / 2) sign(cos(theta / 2)) sin((1 - t) theta / 2) / theta) * (a � r) +
			 2 * (sin((1 - t) theta / 2) / theta)^2 a � (a � r)

		 blah ...

		*/

		return t_result;
	}
};

/** @} */ // end of group

#endif // !__SIM3_SOLVER_BASE_INCLUDED
