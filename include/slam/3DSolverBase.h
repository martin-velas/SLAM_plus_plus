/*
								+----------------------------------+
								|                                  |
								| *** Base class for 3D solver *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|          3DSolverBase.h          |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __3D_SOLVER_BASE_INCLUDED
#define __3D_SOLVER_BASE_INCLUDED

/**
 *	@file include/slam/3DSolverBase.h
 *	@brief a simple base class for 3D solver, made according to 2D solver
 *	@author soso
 *	@date 2013-01-14
 *
 *	@date 2013-01-28
 *
 *	Surrounded some old stuff with ifdefs to speed up the compilation
 *	(basically vertex and edge structures based on the legacy code,
 *	which would be unused in the new code).
 *
 */

/**
 *	@page rot3d 3D Rotation Representations in SLAM++
 *
 *	This document contains specifications about rotation formats and transformation models used used in SLAM++.
 *	Only the 3D case is considered, because the 2D representation is straightforward (in 2D, using a single
 *	angle in radians to represent rotations is sufficient).
 *	We assume right-hand coordinate frame for all representations.
 *
 *	You can also look at the moules that implement the functionality described below, \ref se3 or \ref sim3.
 *
 *	\section sec_rot Rotations in Three Dimensions
 *
 *	There are several ways to represent rotations in 3D. Internally, SLAM++ uses a single representation
 *	(the \ref sec_axisangle). Unfortunately, the datasets and other software often use different representations,
 *	which are also described below.
 *
 *	\subsection sec_quats Quaternions
 *
 *	Quaternions are probably the most elegant, yet the most frowned upon means of representing a rotation in 3D.
 *	A quaternion is composed of three imaginary components which form a rotation direction vector, and a real
 *	component. When normalized, the rotation vector has the magnitude of sine of half of the angle of rotation
 *	(the <em>half-angle</em>), while the real component is the cosine of the half-angle.
 *
 *	The advantage of quaternions is that they require small number of elements to represent the rotations
 *	so they are preferable for carrying out transformations and calculating derivatives. Quaternions
 *	are used in SLAM++ internally for evaluating operations on rotations and their derivatives. However,
 *	they are <b>not</b> used to <em>represent</em> rotations (the \ref sec_axisangle is used for that),
 *	they are merely present as intermediate values.
 *
 *	\subsection sec_rotmatrix Rotation Matrix
 *
 *	Rotation matrix denoted by \f$R\f$ is a \f$3 \times 3\f$ matrix which performs rotation of a vector
 *	around arbitrary axis. Rotation matrix is composed of basic rotations (elemental rotations about one
 *	of the axes of the coordinate system) applied in certain order (see \ref sec_ypr).
 *
 *	Standard 3D point or vector rotation using rotation matrix is performed by matrix-vector multiplication:
 *
 *	\f[ v' = Rv = \left| \begin{array}{ccc}
 *	r_{1,1} & r_{1,2} & r_{1,3} \\
 *	r_{2,1} & r_{2,2} & r_{2,3} \\
 *	r_{3,1} & r_{3,2} & r_{3,3} \end{array} \right|v\f]
 *
 *	where \f$v\f$ is arbitrary 3D vector and \f$v'\f$ is the corresponding vector rotated by \f$R\f$.
 *
 *	\subsection sec_ypr Yaw, Pitch, Roll
 *
 *	A 3D point or vector can be rotated about three principal axes. Borrowing from aviation terminology, these
 *	rotations will be referred to as Yaw, Pitch and Roll which are the rotations about z, y and x axis respectively:
 *
 *	\f{eqnarray*}{ R_z(\alpha) &=& \left|\begin{array}{ccc} cos\alpha & -sin\alpha & 0 \\ sin\alpha & cos\alpha & 0\\ 0 & 0 & 1\\ \end{array}\right| \\
 *	R_y(\beta) &=& \left|\begin{array}{ccc} \beta & 0 & sin\beta \\ 0 & 1 & 0\\ -sin\beta & 0 & cos\beta\\ \end{array}\right| \\
 *	R_x(\gamma) &=& \left|\begin{array}{ccc} 1 & 0 & 0 \\ 0 & cos\gamma & -sin\gamma\\ 0 & sin\gamma & cos\gamma\\ \end{array}\right| \\
 *	R(\alpha, \beta, \gamma) &=& R_z(\alpha) \cdot R_y(\beta) \cdot R_x(\gamma) \f}
 *
 *	In SLAM++, the YPR representation is not widely used, it is only an option for input format, but is converted
 *	to internal representation still inside the parser.
 *
 *	\subsection sec_axisangle Axis-Angle Representation
 *
 *	Axis-Angle representation parametrizes rotation in three dimensional Euclidean space by a <em>single 3D
 *	vector</em> (not to be confused with <a href="https://en.wikipedia.org/wiki/Euler's_rotation_theorem"
 *	onclick="window.open(this.href); return false;">Euler axis and angle</a>, which are <em>four</em> numbers),
 *	a unit vector representing the direction of the axis of rotation, multiplied by the magnitude
 *	of the rotation in radians. This representation is related to the quaternion representation, where
 *	the imaginary part of the quaternion is also the axis of rotation, but scaled by sine of half the angle
 *	of rotation.
 *
 *	The axis-angle representation is somewhat beneficial in nonlinear optimization because, unlike
 *	quaternion, it cannot become denormalized and it forms only a single cover over SO(3), provided that
 *	the rotation angles are kept within \f$[-\frac{\pi}{2}, \frac{\pi}{2}]\f$ (the implementation in SLAM++
 *	does that). They are also better comapred to rotation matrices, since there are fewer numbers being
 *	optimized, the scale is not captured and there are no orthogonality issues.
 *
 *	\subsection sec_rotconv Rotation Formats Conversion
 *
 *	This section lists the functions for converting between the rotation representations, implemented in SLAM++.
 *
 *	* C3DJacobians::v_RotMatrix_to_AxisAngle()
 *	* C3DJacobians::t_AxisAngle_to_RotMatrix()
 *	* C3DJacobians::AxisAngle_to_Quat()
 *	* C3DJacobians::f_AxisAngle_to_Quat()
 *	* C3DJacobians::f_Quat_to_AxisAngle()
 *	* C3DJacobians::Quat_to_AxisAngle()
 *
 *	\section sec_transmodel Transformation Model
 *
 *	Transformation model which represents location of coordinate frame relative to another coordinate frame is
 *	composed of rotation \f$R\f$ and translation \f$\mathbf{t}\f$:
 *
 *	\f[T = [R|\mathbf{t}] \f]
 *
 *	The two types of transformation model are used internally in SLAM++: <b>direct</b> model which relates target
 *	coordinate frame to world coordinate frame (e.g. pose of mobile robot in world coordinate frame) and <b>inverse</b>
 *	model which relates world coordinate frame to target frame (e.g. transformation that transforms 3D points from
 *	world coordinate frame to camera coordinate frame). The former one is used internally in pose SLAM and landmark SLAM
 *	problems involving moving sensor with pose. The latter is internally used in Bundle Adjustment problems as a camera
 *	extrinsic parameters representation.
 *	The relation between direct and inverse model is the following:
 *
 *	\f[ T_{inverse} = T_{direct}^{-1} = [R|\mathbf{t}]^{-1} = [R^T|-R^T\mathbf{t}] \f]
 *
 *	\section sec_datasetsrotations Dataset Rotation Formats
 *
 *	The datasets always specify transformation in the <b>direct</b> model format.
 *	The direct model is automatically converted to inverse if needed (as is the case for camera extrinsic
 *	parameters representation). Please, keep in mind that there is no backward conversion when accessing
 *	the optimized system or when reading the <tt>solution.txt</tt> file - all the rotations there are in the
 *	internal format.
 *
 *	The dataset can specify the rotation representation by using specific tags:
 *
 *	SE(3) datasets:
 *	* EDGE3/EDGE_SE3 x y z yaw pitch roll [information matrix] - rotation input is in YPR format
 *	* EDGE3:AXISANGLE/EDGE_SE3:AXISANGLE x y z r1 r2 r3 [information matrix] - rotation input is in axis-angle format
 *
 *	Bundle Adjustment:
 *	* VERTEX_CAM/VERTEX_SCAM x y z rx ry rz rw [additional parameters] - camera pose is defined with quaternion representation, the real cmponent comes the last
 *
 *	\section fuhread Further Reading
 *
 *	Below is a list of useful links.
 *
 *	* <a href="https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions" onclick="window.open(this.href); return false;">Wikipedia - Rotation_formalisms_in_three_dimensions</a>
 *	* <a href="https://en.wikipedia.org/wiki/Rotation_matrix" onclick="window.open(this.href); return false;">Wikipedia - Rotation_matrix</a>
 *	* <a href="https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation" onclick="window.open(this.href); return false;">Wikipedia - Quaternions_and_spatial_rotation</a>
 *	* <a href="http://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm" onclick="window.open(this.href); return false;">Euclidean Space - Rotation conversions</a>
 */

//#include <math.h> // included from slam/BlockMatrix.h
//#include <float.h> // included from slam/BlockMatrix.h
#include "eigen/Eigen/Core"
#include "slam/BlockMatrix.h"
//#include "slam/Debug.h"
//#include "eigen/Eigen/Cholesky" // included from slam/BlockMatrix.h
#if 1
#include "eigen/Eigen/Geometry" // quaternions
#else // 1
#include "eigen/Eigen/src/Core/util/DisableStupidWarnings.h"
#include "eigen/Eigen/src/Geometry/RotationBase.h"
#include "eigen/Eigen/src/Geometry/Quaternion.h" // quaternions
#ifdef EIGEN_VECTORIZE_SSE
#include "eigen/Eigen/src/Geometry/arch/Geometry_SSE.h"
#endif // EIGEN_VECTORIZE_SSE
#include "eigen/Eigen/src/Core/util/ReenableStupidWarnings.h"
// works in windows but not in linux
#endif // 1

/** \addtogroup se3
 *	@{
 */

/**
 *	@brief calculates 3D jacobians of difference (u [-] v) using code exported from Matlab
 *
 *	@param[out] Ju is jacobian for the first vertex
 *	@param[out] Jv is jacobian for the second vertex
 *	@param[in] theta is rotation angle of the first vertex
 *	@param[in] omega is rotation angle of the second vertex
 *	@param[in] ux is position of the first vertex
 *	@param[in] uy is position of the first vertex
 *	@param[in] uz is position of the first vertex
 *	@param[in] ua is axis-angle rotation of the first vertex
 *	@param[in] ub is axis-angle rotation of the first vertex
 *	@param[in] uc is axis-angle rotation of the first vertex
 *	@param[in] vx is position of the second vertex
 *	@param[in] vy is position of the second vertex
 *	@param[in] vz is position of the second vertex
 *	@param[in] va is axis-angle rotation of the second vertex
 *	@param[in] vb is axis-angle rotation of the second vertex
 *	@param[in] vc is axis-angle rotation of the second vertex
 *
 *	@note The code is in 3DJacs.cpp, which may not be a part of the distribution.
 */
void Calculate3DJacobians(double Ju[6][6], double Jv[6][6], double theta, double omega,
						  double ux, double uy, double uz, double ua, double ub, double uc,
						  double vx, double vy, double vz, double va, double vb, double vc);

/**
 *	@brief calculates 3D jacobian of unitary plus (u [+] epsilon) using code exported from Matlab
 *
 *	@param[out] Ju is jacobian for the first vertex
 *	@param[in] ux is position of the first vertex
 *	@param[in] uy is position of the first vertex
 *	@param[in] uz is position of the first vertex
 *	@param[in] ua is axis-angle rotation of the first vertex
 *	@param[in] ub is axis-angle rotation of the first vertex
 *	@param[in] uc is axis-angle rotation of the first vertex
 *
 *	@note The code is in 3DJacs.cpp, which may not be a part of the distribution.
 */
void Calculate3DJacobians_Plus(double Ju[6][6], double ux, double uy, double uz, double ua, double ub, double uc);

/**
 *	@brief calculates difference using code exported from Matlab
 *
 *	@param[out] d is difference between the two poses (u [-] v)
 *	@param[in] theta is rotation angle of the first vertex
 *	@param[in] omega is rotation angle of the second vertex
 *	@param[in] ux is position of the first vertex
 *	@param[in] uy is position of the first vertex
 *	@param[in] uz is position of the first vertex
 *	@param[in] ua is axis-angle rotation of the first vertex
 *	@param[in] ub is axis-angle rotation of the first vertex
 *	@param[in] uc is axis-angle rotation of the first vertex
 *	@param[in] vx is position of the second vertex
 *	@param[in] vy is position of the second vertex
 *	@param[in] vz is position of the second vertex
 *	@param[in] va is axis-angle rotation of the second vertex
 *	@param[in] vb is axis-angle rotation of the second vertex
 *	@param[in] vc is axis-angle rotation of the second vertex
 *
 *	@note The code is in 3DJacs.cpp, which may not be a part of the distribution.
 */
void CalculateD(double d[6][1], double theta, double omega,
				double ux, double uy, double uz, double ua, double ub, double uc,
				double vx, double vy, double vz, double va, double vb, double vc);

// inject the 6D types into Eigen namespace
namespace Eigen {

#ifndef HAVE_EIGEN_6D_DOUBLE_VECTOR
#define HAVE_EIGEN_6D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 6, 1> Vector6d; /**< @brief a 6D vector */
#endif // !HAVE_EIGEN_6D_DOUBLE_VECTOR
#ifndef HAVE_EIGEN_6D_DOUBLE_MATRIX
#define HAVE_EIGEN_6D_DOUBLE_MATRIX
typedef Eigen::Matrix<double, 6, 6> Matrix6d; /**< @brief a 6x6 matrix */
#endif // !HAVE_EIGEN_6D_DOUBLE_MATRIX

} // ~Eigen

/**
 *	@brief implementation of Jacobian calculations, required by 3D solvers
 */
class C3DJacobians {
public:
	typedef Eigen::Vector6d Vector6d; /**< @brief 6D vector type */
	typedef Eigen::Matrix6d Matrix6d; /**< @brief 6x6 matrix type */

public:
	/**
	 *	@brief converts from axis angle rep to rotation matrix
	 *	@param[in] v_vec is axis-angle rotation (angle is encoded as the magnitude of the axis)
	 *	@return Returns rotation matrix, corresponding to the input.
	 *	@deprecated This function is deprecated in favor of C3DJacobians::t_AxisAngle_to_RotMatrix(). Please, do not use it.
	 */
	static Eigen::Matrix3d Operator_rot(Eigen::Vector3d v_vec)
	{
		return t_AxisAngle_to_RotMatrix(v_vec); // no need for 2nd implementation
	}

	/**
	 *	@brief converts from axis angle representation to rotation matrix
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *	@param[in] v_vec is axis-angle rotation (angle is encoded as the magnitude of the axis)
	 *	@return Returns rotation matrix, corresponding to the input.
	 *	@note Consider whether converting to a quaternion would not be faster: converting to a quaternion
	 *		and transforming a single point will be faster than converting to a rotation matrix. Only when
	 *		transforming two or more points, rotation matrix will be faster.
	 */
	template <class Derived>
	static Eigen::Matrix3d t_AxisAngle_to_RotMatrix(const Eigen::MatrixBase<Derived> &v_vec)
	{
		DimensionCheck<Eigen::Vector3d>(v_vec);

#if 1 // reuse conversion to quaternions, to avoid repeating code. recently improved numerical stability there.
		Eigen::Quaternion<double> rot;
		AxisAngle_to_Quat(v_vec, rot);
		return rot.toRotationMatrix();
#else // 1
		double f_angle = v_vec.norm();//sqrt(x*x + y*y + z*z); // SSE
		// this is really angle in radians

		if(f_angle > 0) {
			v_vec /= f_angle; // SSE
			//x = x / f_angle;
			//y = y / f_angle;
			//z = z / f_angle;
		} else
			return Eigen::Matrix3d::Identity(); // no rotation, save some hassle below
		// normalize the axis

#if 1
		double f_half_angle = f_angle * .5;
		v_vec *= sin(f_half_angle); // SSE
		Eigen::Quaternion<double> rot(cos(f_half_angle), v_vec(0), v_vec(1), v_vec(2));
		return rot.toRotationMatrix();
		// use quaternion, Eigen might provide SSE accelerated conversion to matrix
#else // 1
		double s = sin(f_angle);
		double c = cos(f_angle);
		double v = 1 - c;

		double x = v_vec(0), y = v_vec(1), z = v_vec(2);
		double xyv = x * y * v;
		double yzv = y * z * v;
		double xzv = x * z * v;

		Eigen::Matrix3d Q;
		Q << x * x * v + c, xyv - z * s, xzv + y * s,
			 xyv + z * s, y * y * v + c, yzv - x * s,
			 xzv - y * s, yzv + x * s, z * z * v + c;

		return Q;
#endif // 1
#endif // 1
	}

	/**
	 *	@brief converts from rotation matrix rep to axis angle
	 *	@param[in] Q is the rotation matrix
	 *	@return Returns axis-amgle rotation, where the angle is encoded as magnitude of the axis.
	 *	@deprecated This function is deprecated in favor of C3DJacobians::v_RotMatrix_to_AxisAngle(). Please, do not use it.
	 */
	static Eigen::Vector3d Operator_arot(const Eigen::Matrix3d &Q) // note that Eigen probably implements this somewhere
	{
		return v_RotMatrix_to_AxisAngle(Q); // no need for 2nd implementation
	}

	/**
	 *	@brief converts from a rotation matrix to axis anglerepresentation
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *	@param[in] Q is the rotation matrix
	 *	@return Returns axis-amgle rotation, where the angle is encoded as magnitude of the axis.
	 */
	template <class Derived>
	static Eigen::Vector3d v_RotMatrix_to_AxisAngle(const Eigen::MatrixBase<Derived> &Q)
	{
		DimensionCheck<Eigen::Matrix3d>(Q);

#if 1 // reuse conversion to quaternions, to avoid repeating code. recently improved numerical stability there.
		Eigen::Quaternion<double> quat(Q); // converts rotation matrix to quaternion (hopefully SSE)
		Eigen::Vector3d v_aang;
		Quat_to_AxisAngle(quat, v_aang); // reuse existing code
		return v_aang;
#else // 1
#if 1
		Eigen::Quaternion<double> quat(Q); // converts rotation matrix to quaternion (hopefully SSE)
		double f_half_angle = acos(quat.w());
		if(f_half_angle == 0)
			return Eigen::Vector3d(0, 0, 0); // handle zero angle rotation
		return quat.vec() * (2 * f_half_angle / sin(f_half_angle)); // SSE
		// need to divide by sine of half angle to get the normalized rotation axis,
		// then multiply by angle to get axis-angle
#else // 1
		const double epsilon = 1e-12;

		Eigen::Vector4d r;
		Eigen::Vector3d axis;
		double matrace = Q.trace();

		if(fabs(matrace - 3.0) <= epsilon)
			r << 0, 1, 0, 0;
		else if(fabs(matrace + 1.0) <= epsilon) {
			printf("fabs(matrace + 1.0) <= epsilon\n");

			for(int a = 0; a < 3; a ++) {
				double f_axis_a = (Q(a, a) + 1) / 2;
				axis(a) = sqrt((f_axis_a > 0)? f_axis_a : 0);
				axis(a) = (axis(a) > epsilon)? axis(a) : 0; // clamp values below epsilon
			}

			//flipping
			Eigen::Vector3d m_upper(Q(1,2), Q(0,2), Q(0,1)), signs;

			for(int a = 0; a < 3; a ++)
				signs(a) = ((m_upper(a) > epsilon)? 1 : ((m_upper(a) < -epsilon)? -1 : 0)); // same amount of comparisons, one multiplication fewer
				//signs(a) = ((m_upper(a) > 0)? 1 : ((m_upper(a) == 0)? 0 : -1)) * (fabs(m_upper(a)) > epsilon);

			Eigen::Vector3d flip; // delay creation of new objects to the latest possible moment
			if(signs.sum() >= 0) // signs(0) + signs(1) + signs(2) >= 0 // SSE
				flip << 1, 1, 1;
			else if(signs(0) != 0 && signs(1) != 0 && signs(2) != 0)
				flip = -signs;
			else {
				Eigen::Vector3d shifted(signs(2), signs(0), signs(1)); // limit the scope of the objects where possible
				for(int a = 0; a < 3; a ++)
					flip(a) = shifted(a) + (shifted(a) == 0);
			}

			//flip the axis
			//for(int a = 0; a < 3; a ++)
			//	axis(a) = axis(a) * flip(a);
			//r << axis(0), axis(1), axis(2), M_PI;
			r.head<3>() = axis.cwiseProduct(flip); // elementwise operation using .array() or .cwiseProduct(), accelerated with SSE; also no need to modify axis, it is about to be overwritten
			r(3) = M_PI;
		} else {
			double phi = acos((matrace - 1) / 2.0);
			double den = 2.0 * sin(phi);
			//axis << (Q(2, 1) - Q(1, 2)) / den, (Q(0, 2) - Q(2, 0)) / den, (Q(1, 0) - Q(0, 1)) / den;
			//r << axis(0), axis(1), axis(2), phi;
			r.head<3>() = (Eigen::Vector3d(Q(2, 1), Q(0, 2), Q(1, 0)) -
				Eigen::Vector3d(Q(1, 2), Q(2, 0), Q(0, 1))) / den; // vector ops, SSE again
			r(3) = phi;
		}

		double sum = r.head<3>().norm(); // sqrt(r(0) * r(0) + r(1) * r(1) + r(2) * r(2));
		axis = r.head<3>() * (r(3) / sum);
		//for(int a = 0; a < 3; a ++)
		//	axis(a) = r(a) * (r(3) / sum);
		// scale axis by angle

		return axis;
#endif // 1
#endif // 1
	}

	/**
	 *	@brief converts axis-angle representation to quaternion
	 *
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[in] r_axis_angle is a vector where unit direction gives axis, and magnitude gives angle in radians
	 *	@param[out] r_quat is quaternion representing the same rotation as r_axis_angle
	 */
	template <class Derived>
	static void AxisAngle_to_Quat(const Eigen::MatrixBase<Derived> &r_axis_angle, Eigen::Quaterniond &r_quat)
	{
		DimensionCheck<Eigen::Vector3d>(r_axis_angle);

		f_AxisAngle_to_Quat(r_axis_angle, r_quat); // ignore the return value, probably optimized away
		/*double f_angle = r_axis_angle.norm();
		if(f_angle < 1e-12)
			r_quat = Eigen::Quaterniond(1, 0, 0, 0); // cos(0) = 1
		else {
			//_ASSERTE(f_angle <= M_PI); // sometimes broken
			f_angle = fmod(f_angle, M_PI * 2);
			double q = (sin(f_angle * .5) / f_angle);
			r_quat = Eigen::Quaterniond(cos(f_angle * .5), r_axis_angle(0) * q,
				r_axis_angle(1) * q, r_axis_angle(2) * q);
			r_quat.normalize();
		}*/
	}

	/**
	 *	@brief converts axis-angle representation to quaternion
	 *
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[in] r_axis_angle is a vector where unit direction gives axis, and magnitude gives angle in radians
	 *	@param[out] r_quat is quaternion representing the same rotation as r_axis_angle
	 *
	 *	@return Returns rotation angle in radians. This angle is not wrapped to a specific interval
	 *		and can be used to normalize the axis of rotation. The angle is always positive.
	 */
	template <class Derived>
	static double f_AxisAngle_to_Quat(const Eigen::MatrixBase<Derived> &r_axis_angle, Eigen::Quaterniond &r_quat)
	{
		DimensionCheck<Eigen::Vector3d>(r_axis_angle);

#if 1
		double f_angle = r_axis_angle.norm();
		if(f_angle < 1e-12) { // increasing this does not help
			const double q = .5; // limit of sin(f_angle / 2) / (2 * f_angle / 2) for f_angle -> 0
			r_quat = Eigen::Quaterniond(cos(f_angle * .5), r_axis_angle(0) * q,
				r_axis_angle(1) * q, r_axis_angle(2) * q);
			r_quat.normalize(); // will be slightly denormalized since we are taking the limit here
			// use the limit
		} else {
			double f_half_angle = f_angle * .5; // no need to fmod()
			double c = cos(f_half_angle);
			double q = sin(f_half_angle) / f_angle; // sin is [0, 1], angle is about [-2pi, 2pi] but could be slightly more / less due to optimization incrementing / decrementing it
			if(c < 0) { // the same as doing fmod() on the angle, but faster and likely more numerically stable
				c = -c;
				q = -q;
			}
			r_quat = Eigen::Quaterniond(c, r_axis_angle(0) * q,
				r_axis_angle(1) * q, r_axis_angle(2) * q);
			if(c > 1 - 1e-6) // do not bother renormalizing if the values are large enough
				r_quat.normalize(); // should already be normalized unless there was a great roundoff in calculation of q; could detect that.
			_ASSERTE(fabs(r_quat.norm() - 1) < 1e-10); // make sure it stays normalized
		}
#else // 1 // the old code
		double f_angle = r_axis_angle.norm();
		if(f_angle < 1e-12) { // increasing this does not help
			r_quat = Eigen::Quaterniond(1, 0, 0, 0); // cos(0) = 1
			return 0;//M_PI * 2;
		} else {
			//_ASSERTE(f_angle <= M_PI); // sometimes broken
			double f_half_angle = fmod(f_angle, M_PI * 2) * .5;
			double q = sin(f_half_angle) / f_angle; // sin is [0, 1], angle is about [-2pi, 2pi] but could be slightly more / less due to optimization incrementing / decrementing it
			r_quat = Eigen::Quaterniond(cos(f_half_angle), r_axis_angle(0) * q,
				r_axis_angle(1) * q, r_axis_angle(2) * q);
			r_quat.normalize(); // should already be normalized unles there was a great roundoff in calculation of q; could detect that.
		}
#endif // 1

		return f_angle;
	}

	/**
	 *	@brief converts quaternion to axis-angle representation
	 *
	 *	@tparam Derived is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_quat is quaternion representing the same rotation as r_axis_angle
	 *	@param[out] r_axis_angle is a vector where unit direction gives axis, and magnitude gives angle in radians
	 */
	template <class Derived>
	static void Quat_to_AxisAngle(const Eigen::Quaterniond &r_quat, Eigen::MatrixBase<Derived> &r_axis_angle)
	{
		DimensionCheck<Eigen::Vector3d>(r_axis_angle);

		f_Quat_to_AxisAngle(r_quat, r_axis_angle); // ignore the return value, probably optimized away
		////double f_half_angle = /*(r_quat.w() <= 0)? asin(r_quat.vec().norm()) :*/ acos(r_quat.w()); // 0 .. pi
		//double f_half_angle = (r_quat.w() == 1.0)? asin(r_quat.vec().norm()) : acos(r_quat.w()); // 0 .. pi // more numerically robust. normalization of quaternions sometimes yields (eps eps eps 1.0), the above line would calculate angle of 0.0, this line calculates angle of ~eps
		//_ASSERTE(f_half_angle >= 0);
		//if(f_half_angle < 1e-12)
		//	r_axis_angle = Eigen::Vector3d(0, 0, 0); // lim(sin(x) / x) for x->0 equals 1, we're therefore multiplying a null vector by 1
		//else {
		//	double f_angle = 2 * ((r_quat.w() <= 0)? f_half_angle - M_PI : f_half_angle);
		//	r_axis_angle = r_quat.vec() * (f_angle / sin(f_half_angle));
		//}
	}

	/**
	 *	@brief converts quaternion to axis-angle representation
	 *
	 *	@tparam Derived is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_quat is quaternion representing the same rotation as r_axis_angle
	 *	@param[out] r_axis_angle is a vector where unit direction gives axis, and magnitude gives angle in radians
	 *
	 *	@return Returns rotation angle in radians (after flushing small angles to zero).
	 */
	template <class Derived>
	static double f_Quat_to_AxisAngle(const Eigen::Quaterniond &r_quat, Eigen::MatrixBase<Derived> &r_axis_angle)
	{
		DimensionCheck<Eigen::Vector3d>(r_axis_angle);

#if 1 // this uses atan() if possible, or atan2() in 2pi rotations and avoids adding M_PI

// with f_norm and asin / acos
//sphere2500.txt: 5 iterations, chi2: 727.81 (ref 728), time: 0.605334 sec <- worse
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.083739 sec
//venice871.g2o: 5 iterations, chi2: 233945658.43 (ref 234013899), time: 79.339993 sec <- this is significantly better
//sphere2500.txt: 2938 iterations, chi2: 728.31 (ref 731), time: 167.507582 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72'
//parking-garage.txt: 1563 iterations, chi2: 1.27 (ref 1), time: 13.823409 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106'

// with norm and atan2
//sphere2500.txt: 5 iterations, chi2: 727.72 (ref 728), time: 0.606549 sec <- :)
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.084157 sec
//venice871.g2o: 5 iterations, chi2: 233948937.52 (ref 234013899), time: 80.711197 sec <-  slightly worse than the previous one
//sphere2500.txt: 2542 iterations, chi2: 728.65 (ref 731), time: 129.329794 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72'
//parking-garage.txt: 1500 iterations, chi2: 1.26 (ref 1), time: 17.015005 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106'

// with norm and atan
//sphere2500.txt: 5 iterations, chi2: 727.72 (ref 728), time: 0.604325 sec <- :)
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.083413 sec
//venice871.g2o: 5 iterations, chi2: 233948935.17 (ref 234013899), time: 81.009040 sec <- slightly better than the previous one
//sphere2500.txt: 2551 iterations, chi2: 730.30 (ref 731), time: 137.754377 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72' <- worse
//parking-garage.txt: 1555 iterations, chi2: 1.26 (ref 1), time: 15.595552 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106'

// with norm and atan and atan2
//sphere2500.txt: 5 iterations, chi2: 727.72 (ref 728), time: 0.604976 sec <- :)
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.084059 sec <- :)
//venice871.g2o: 5 iterations, chi2: 233948935.17 (ref 234013899), time: 78.168804 sec <- :)
//sphere2500.txt: 2551 iterations, chi2: 730.30 (ref 731), time: 138.956700 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72' <- the same
//parking-garage.txt: 1394 iterations, chi2: 1.26 (ref 1), time: 13.186234 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106' <- :)

// with copysign (no change)
//sphere2500.txt: 5 iterations, chi2: 727.72 (ref 728), time: 0.605713 sec
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.084031 sec
//venice871.g2o: 5 iterations, chi2: 233948935.17 (ref 234013899), time: 79.507713 sec
//sphere2500.txt: 2551 iterations, chi2: 730.30 (ref 731), time: 138.474680 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72'
//parking-garage.txt: 1394 iterations, chi2: 1.26 (ref 1), time: 13.338290 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106'

		const double f_w = r_quat.w();
		const double f_abs_w = fabs(f_w), f_norm = r_quat.vec().norm();
		const double f_abs_half_angle_short = (f_abs_w > 1e-3)? atan(f_norm / f_abs_w) : atan2(f_norm, f_abs_w);
		//const double f_half_angle_short = (f_w < 0)? -f_abs_half_angle_short : f_abs_half_angle_short;
#if defined(_MSC_VER) && !defined(_MWERKS)
		const double f_half_angle_short = _copysign(f_abs_half_angle_short, f_w); // msvc
#else // _MSC_VER) && !_MWERKS
		const double f_half_angle_short = copysign(f_abs_half_angle_short, f_w); // posix
#endif // _MSC_VER) && !_MWERKS
		// the "short" rotation -pi/2, pi/2

#ifdef _DEBUG
		{
			double f_angle_ref = 2 * atan2(r_quat.vec().norm(), r_quat.w()); // this is precise but probably slower
			_ASSERTE(f_angle_ref >= 0 && f_angle_ref <= 2 * M_PI);
			if(f_angle_ref > M_PI)
				f_angle_ref -= 2 * M_PI; // keep the angle between -M_PI, M_PI
			_ASSERTE(fabs(f_half_angle_short * 2 - f_angle_ref) < 1e-10); // this must match
		}
#endif // _DEBUG
		// make sure the angle matches the reference

		_ASSERTE(f_abs_half_angle_short >= 0); // detects NaNs
		if(f_norm < 1e-12) {
			r_axis_angle = r_quat.vec() * 2.0;

			_ASSERTE(fabs(r_axis_angle.norm() - 2 * f_abs_half_angle_short) < 1e-10);
			// this presents a problem as this does not equal norm of r_axis_angle now (although sin() is *almost* linear in here)
			// although "quite" precise, it will not normalize the axis to unit length, in case someone attempts that

			//return 2 * f_abs_half_angle_short;
			return r_axis_angle.norm(); // this normalizes the axis and is very close to the angle of the input quaternion, probably a better choice
		} else {
			r_axis_angle = r_quat.vec() * (f_half_angle_short * 2 / f_norm);

#ifdef _DEBUG
			double f_angle = 2 * f_abs_half_angle_short; // what is returned
			double f_angle_ret = r_axis_angle.norm();
			_ASSERTE(fabs(f_angle_ret - fabs(f_angle)) < 1e-10); // the ret angle is always positive, since it is obtained via a norm
			Eigen::Quaterniond t_quat_back;
			double f_angle_back = f_AxisAngle_to_Quat(r_axis_angle, t_quat_back);
			double f_w = r_quat.w(), f_w_back = t_quat_back.w(), f_sign_diff = ((t_quat_back.w() * r_quat.w() < 0)? -1 : 1);
			double f_quat_error = (t_quat_back.coeffs() - r_quat.coeffs() * f_sign_diff).norm();
			double f_input_denorm = fabs(r_quat.coeffs().norm() - 1);
			_ASSERTE(f_quat_error < 1e-10 + f_input_denorm);
			_ASSERTE(fabs(f_angle_back - f_angle) < 1e-10); // is the input quaternion denormalized?
#endif // _DEBUG
			// this is somehow troubling, see if it inverts correctly

			_ASSERTE(fabs(2 * f_abs_half_angle_short - r_axis_angle.norm()) < 1e-10); // make sure this matches the norm
			return 2 * f_abs_half_angle_short; // return absolute value
		}
#elif 0 // this uses simple trig functions and avoids adding M_PI

//sphere2500.txt: 5 iterations, chi2: 727.72 (ref 728), time: 1.256919 sec
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.811704 sec
//venice871.g2o: 5 iterations, chi2: 234013929.03 (ref 234013899), time: 148.455945 sec
//sphere2500.txt: 2546 iterations, chi2: 728.02 (ref 731), time: 135.769666 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72'
//parking-garage.txt: 1534 iterations, chi2: 1.26 (ref 1), time: 96.177606 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106'

//sphere2500.txt: 5 iterations, chi2: 727.72 (ref 728), time: 0.605376 sec
//parking-garage.txt: 5 iterations, chi2: 1.46 (ref 1), time: 0.084303 sec
//venice871.g2o: 5 iterations, chi2: 234013929.03 (ref 234013899), time: 81.814012 sec
//sphere2500.txt: 2551 iterations, chi2: 728.84 (ref 731), time: 128.501534 sec, notes: 'RSS13 incremental time 9.865, chi2 727.72'
//parking-garage.txt: 1348 iterations, chi2: 1.27 (ref 1), time: 12.864370 sec, notes: 'RSS13 incremental time 3.410, chi2 1.3106'

		const double f_w = r_quat.w();
		const double f_abs_w = fabs(f_w), f_sign_w = (f_w < 0)? -1 : 1;
		const double f_abs_half_angle_short = (f_abs_w > .999)? asin(r_quat.vec().norm()) : acos(f_abs_w);
		const double f_half_angle_short = f_abs_half_angle_short * f_sign_w;
		// the "short" rotation -pi/2, pi/2

#ifdef _DEBUG
		{
			double f_angle_ref = 2 * atan2(r_quat.vec().norm(), r_quat.w()); // this is precise but probably slower
			_ASSERTE(f_angle_ref >= 0 && f_angle_ref <= 2 * M_PI);
			if(f_angle_ref > M_PI)
				f_angle_ref -= 2 * M_PI; // keep the angle between -M_PI, M_PI
			_ASSERTE(fabs(f_half_angle_short * 2 - f_angle_ref) < 1e-10); // this must match
		}
#endif // _DEBUG
		// make sure the angle matches the reference

		if(f_abs_half_angle_short < 1e-12) {
			r_axis_angle = r_quat.vec() * 2.0;

			_ASSERTE(fabs(r_axis_angle.norm() - 2 * f_abs_half_angle_short) < 1e-10);
			// this presents a problem as this does not equal norm of r_axis_angle now (although sin() is *almost* linear in here)
			// although "quite" precise, it will not normalize the axis to unit length, in case someone attempts that

			//return 2 * f_abs_half_angle_short;
			return r_axis_angle.norm(); // this normalizes the axis and is very close to the angle of the input quaternion, probably a better choice
		} else {
			r_axis_angle = r_quat.vec() * (f_half_angle_short * 2 / sin(f_abs_half_angle_short));

#ifdef _DEBUG
			double f_angle = 2 * f_abs_half_angle_short; // what is returned
			double f_angle_ret = r_axis_angle.norm();
			_ASSERTE(fabs(f_angle_ret - fabs(f_angle)) < 1e-10); // the ret angle is always positive, since it is obtained via a norm
			Eigen::Quaterniond t_quat_back;
			double f_angle_back = f_AxisAngle_to_Quat(r_axis_angle, t_quat_back);
			double f_w = r_quat.w(), f_w_back = t_quat_back.w(), f_sign_diff = ((t_quat_back.w() * r_quat.w() < 0)? -1 : 1);
			double f_quat_error = (t_quat_back.coeffs() - r_quat.coeffs() * f_sign_diff).norm();
			_ASSERTE(f_quat_error < 1e-10);
			_ASSERTE(fabs(f_angle_back - f_angle) < 1e-10);
#endif // _DEBUG
			// this is somehow troubling, see if it inverts correctly

			_ASSERTE(fabs(2 * f_abs_half_angle_short - r_axis_angle.norm()) < 1e-10); // make sure this matches the norm
			return 2 * f_abs_half_angle_short; // return absolute value
		}
#elif 0 // this uses slower atan2() if w is -1, and requires imprecise addition of M_PI
		//double f_half_angle = acos(r_quat.w()); // 0 .. pi
		// problems is w is 1 or slightly more (1+ulp occurs pretty often actually)

		//double f_half_angle = (fabs(r_quat.w()) >= 0.999)? asin(r_quat.vec().norm() *
		//	((r_quat.w() < 0)? -1 : 1)) + ((r_quat.w() < 0)? M_PI : 0) : acos(r_quat.w());
		// this uses M_PI addition which tends to be fairly imprecise

		double f_half_angle = (r_quat.w() < 0.999)? (r_quat.w() > -0.999)? acos(r_quat.w()) :
			atan2(r_quat.vec().norm(), r_quat.w()) : asin(r_quat.vec().norm());
		// this uses atan2() if w is near -1

		_ASSERTE(fabs(f_half_angle - atan2(r_quat.vec().norm(), r_quat.w())) < 1e-10); // make sure this is precise
		_ASSERTE(f_half_angle >= 0); // always positive
		// more numerically robust. normalization of quaternions sometimes yields (eps eps eps 1.0)
		// product of normalized quats sometimes yields (eps eps eps (1.0 + 1 ULP)),
		// the above line would calculate angle of 0.0, this line calculates angle of ~eps
		// note that r_quat.w() doesn't ever seem to be negative one, otherwise enable fabs()
		// note that for 1-ulp, acos() is already quite imprecise, use asin() instead for large numbers even below one

		_ASSERTE(f_half_angle >= 0); // detects NaNs
		if(f_half_angle < 1e-12) {
			//r_axis_angle = Eigen::Vector3d(0, 0, 0); // lim(sin(x) / x) for x->0 equals 1, we're therefore multiplying a null vector by 1
			//return 0;
			// that's fast

			r_axis_angle = r_quat.vec() * 2.0;// limit of (2 * f_half_angle / sin(f_half_angle)) for f_half_angle -> 0 is 2
			return 2 * f_half_angle;
			// that's hopefully more precise
		} else {
			double f_angle = 2 * f_half_angle; // 0, 2 * pi
			_ASSERTE(f_angle >= 0 && f_angle <= 2 * M_PI);
			if(f_angle > M_PI)
				f_angle -= 2 * M_PI; // try to keep the angle between -M_PI, M_PI
			r_axis_angle = r_quat.vec() * (f_angle / sin(f_half_angle));

#ifdef _DEBUG
			double f_angle_ret = r_axis_angle.norm();
			_ASSERTE(fabs(f_angle_ret - fabs(f_angle)) < 1e-10); // the ret angle is always positive, since it is obtained via a norm
			Eigen::Quaterniond t_quat_back;
			double f_angle_back = f_AxisAngle_to_Quat(r_axis_angle, t_quat_back);
			double f_w = r_quat.w(), f_w_back = t_quat_back.w(), f_sign_diff = ((t_quat_back.w() * r_quat.w() < 0)? -1 : 1);
			double f_quat_error = (t_quat_back.coeffs() - r_quat.coeffs() * f_sign_diff).norm();
			_ASSERTE(f_quat_error < 1e-10);
			_ASSERTE(fabs(f_angle_back - f_angle * f_sign_diff) < 1e-10);
#endif // _DEBUG
			// this is somehow troubling, see if it inverts correctly

			return f_angle;
		}
#else // 1 // the old code
		//double f_half_angle = /*(r_quat.w() <= 0)? asin(r_quat.vec().norm()) :*/ acos(r_quat.w()); // 0 .. pi

		double f_half_angle = (/*fabs*/(r_quat.w()) >= 1.0)? asin(r_quat.vec().norm()) : acos(r_quat.w()); // 0 .. pi
		// more numerically robust. normalization of quaternions sometimes yields (eps eps eps 1.0)
		// product of normalized quats sometimes yields (eps eps eps (1.0 + 1 ULP)),
		// the above line would calculate angle of 0.0, this line calculates angle of ~eps
		// note that r_quat.w() doesn't ever seem to be negative one, otherwise enable fabs()

		_ASSERTE(f_half_angle >= 0); // detects NaNs
		if(f_half_angle < 1e-12) {
			r_axis_angle = Eigen::Vector3d(0, 0, 0); // lim(sin(x) / x) for x->0 equals 1, we're therefore multiplying a null vector by 1
			return 0;
		} else {
			double f_angle = 2 * ((r_quat.w() <= 0)? f_half_angle - M_PI : f_half_angle);
			r_axis_angle = r_quat.vec() * (f_angle / sin(f_half_angle));
			return f_angle;
		}
#endif // 1
	}

	/**
	 *	@brief operator plus for 3D vertices
	 *
	 *	@param[in] r_t_vertex1 is the vertex to be modified
	 *	@param[in] r_t_vertex2 is the delta vector
	 *	@param[out] r_t_dest is the result of the operation
	 *
	 *	@deprecated This is deprecated in favor of C3DJacobians::Relative_to_Absolute(). Please, do not use this function.
	 */
	static void Smart_Plus(const Vector6d &r_t_vertex1, const Vector6d &r_t_vertex2, Vector6d &r_t_dest)
	{
		Relative_to_Absolute(r_t_vertex1, r_t_vertex2, r_t_dest);
		// 2nd implementation not required
	}

	/**
	 *	@brief composes a pair of relative transformations to yield an aboslute transformation
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first transformation (in absolute coordinates)
	 *	@param[in] r_t_vertex2 is the second transformation (relative to the first one)
	 *	@param[out] r_t_dest is filled with the composed transformation
	 */
	template <class Derived0, class Derived1, class Derived2>
	static inline void Relative_to_Absolute(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Vector6d>(r_t_vertex2);
		DimensionCheck<Vector6d>(r_t_dest);

#if 0
		/* TODO: make more efficient */

		//r_t_dest.resize(6, 1); // no need to resize, it is 6D vector at compile time
		//r_t_dest(0) = r_t_vertex1(0) + r_t_vertex2(0);
		//r_t_dest(1) = r_t_vertex1(1) + r_t_vertex2(1);
		//r_t_dest(2) = r_t_vertex1(2) + r_t_vertex2(2);
		r_t_dest.head<3>() = r_t_vertex1.head<3>() + r_t_vertex2.head<3>(); // accelerated using SSE

		//sum the rotations
		Eigen::Matrix3d pQ = t_AxisAngle_to_RotMatrix(r_t_vertex1.tail<3>());
		Eigen::Matrix3d dQ = t_AxisAngle_to_RotMatrix(r_t_vertex2.tail<3>());

		Eigen::Matrix3d QQ;// = pQ * dQ;
		QQ.noalias() = pQ * dQ; // multiplication without intermediate storage
		//Eigen::Vector3d axis = v_RotMatrix_to_AxisAngle(QQ);
		r_t_dest.tail<3>() = v_RotMatrix_to_AxisAngle(QQ);

		//r_t_dest(3) = axis(0);
		//r_t_dest(4) = axis(1);
		//r_t_dest(5) = axis(2);
#else
		//Eigen::Vector3d p1 = r_t_vertex1.template head<3>();
		Eigen::Quaterniond q1;
		AxisAngle_to_Quat(r_t_vertex1.template tail<3>(), q1);

		//Eigen::Vector3d p2 = r_t_vertex2.template head<3>();
		Eigen::Quaterniond q2;
		AxisAngle_to_Quat(r_t_vertex2.template tail<3>(), q2);

		r_t_dest.template head<3>() = r_t_vertex1.template head<3>() +
			q1._transformVector(r_t_vertex2.template head<3>()); // p2 rotates!
		Eigen::Vector3d v_aang;
		Quat_to_AxisAngle((q1 * q2)/*.normalized()*/, v_aang); // product doesn't denormalize
		r_t_dest.template tail<3>() = v_aang;
#endif

#if 0
#if 0
		/* TODO: make more efficient */
		//r_t_dest.resize(6, 1); // no need to resize fixed size expressions
		Eigen::Vector3d p = r_t_vertex1.head<3>();
		Eigen::Matrix3d pQ = t_AxisAngle_to_RotMatrix(r_t_vertex1.tail<3>());
		Eigen::Vector3d d = r_t_vertex2.head<3>();
		Eigen::Matrix3d dQ = t_AxisAngle_to_RotMatrix(r_t_vertex2.tail<3>());
		r_t_dest.head<3>() = p + pQ * d;

		Eigen::Matrix3d QQ;// = pQ * dQ;
		QQ.noalias() = pQ * dQ; // multiplication without intermediate storage
		r_t_dest.tail<3>() = v_RotMatrix_to_AxisAngle(QQ);
#else // 0
		Eigen::Vector3d p1 = r_t_vertex1.head<3>();
		Eigen::Quaterniond q1;
		AxisAngle_to_Quat(r_t_vertex1.tail<3>(), q1);

		Eigen::Vector3d p2 = r_t_vertex2.head<3>();
		Eigen::Quaterniond q2;
		AxisAngle_to_Quat(r_t_vertex2.tail<3>(), q2);

		r_t_dest.head<3>() = p1 + q1._transformVector(p2);
		Eigen::Vector3d v_aang;
		Quat_to_AxisAngle((q1 * q2)/*.normalized()*/, v_aang); // product doesn't denormalize
		r_t_dest.tail<3>() = v_aang;
#endif // 0
#endif // 0
	}

	/**
	 *	@brief converts xyz axis angle coordinates from absolute measurement to relative measurement
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first vertex, in absolute coordinates
	 *	@param[in] r_t_vertex2 is the second vertex, also in absolute coordinates
	 *	@param[out] r_t_dest is filled with relative coordinates of the second vertex
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Absolute_to_Relative(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Vector6d>(r_t_vertex2);
		DimensionCheck<Vector6d>(r_t_dest);

#if 0
		/* TODO: make more efficient */
		Eigen::Vector3d p1 = r_t_vertex1.template head<3>();
		Eigen::Matrix3d pQ1 = t_AxisAngle_to_RotMatrix(r_t_vertex1.template tail<3>());

		Eigen::Vector3d p2 = r_t_vertex2.template head<3>();
		Eigen::Matrix3d pQ2 = t_AxisAngle_to_RotMatrix(r_t_vertex2.template tail<3>());

		Eigen::Matrix3d pQ1_inv = pQ1.inverse();
		//Eigen::Matrix3d pQ2_inv = pQ2.inverse();

		//r_t_dest.resize(6, 1); // no need to resize fixed size expressions
		r_t_dest.head<3>() = pQ1_inv * (p2 - p1);

		Eigen::Matrix3d QQ = pQ1_inv * pQ2;
		r_t_dest.tail<3>() = v_RotMatrix_to_AxisAngle(QQ);	//TODO: is this right??
#else
		//Eigen::Vector3d p1 = r_t_vertex1.template head<3>();
		Eigen::Quaterniond q1;
		AxisAngle_to_Quat(r_t_vertex1.template tail<3>(), q1);

		//Eigen::Matrix3d pQ1 = t_AxisAngle_to_RotMatrix(r_t_vertex1.tail<3>());
		//Eigen::Matrix3d pQ1_inv = pQ1.inverse();
		// matrix not required after all

		//Eigen::Vector3d p2 = r_t_vertex2.template head<3>();
		Eigen::Quaterniond q2;
		AxisAngle_to_Quat(r_t_vertex2.template tail<3>(), q2);

		Eigen::Quaterniond q1_inv = q1.conjugate(); // the inverse rotation (also have .inverse() but that is not needed)

		r_t_dest.template head<3>() = q1_inv._transformVector(r_t_vertex2.template head<3>() -
			r_t_vertex1.template head<3>()); // this is precise enough
		//r_t_dest.head<3>() = pQ1_inv * (p2 - p1); // or can use matrix to transform position

		//Eigen::Quaterniond prod = (q1_inv * q2)/*.normalized()*/; // quaternion product does not denormalize
		//if(1 || (prod.w()) >= 0) { //printf("bloop %f\n", prod.w());
			Eigen::Vector3d v_aang;
			Quat_to_AxisAngle(q1_inv * q2, v_aang); // use quaternion to calculate the rotation
			r_t_dest.template tail<3>() = v_aang;
		//} else { //printf("bleep %f\n", prod.w());
		//	//r_t_dest.tail<3>() = v_RotMatrix_to_AxisAngle(pQ1_inv * q2); // use quaternion to calculate the rotation
		//	r_t_dest.tail<3>() = v_RotMatrix_to_AxisAngle((q1_inv * q2).toRotationMatrix()); // use quaternion to calculate the rotation
		//}
		// arot is needed, but toRotationMatrix() immediately followed by arot() can likely be optimized
#endif
	}

	/**
	 *	@brief inverts a pose
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_t_pose is a pose
	 *	@param[out] r_t_dest is filled with the inverse pose
	 *
	 *	@note This function can work inplace, r_t_pose and r_t_dest may point to the same vector.
	 */
	template <class Derived0, class Derived1>
	static void Pose_Inverse(const Eigen::MatrixBase<Derived0> &r_t_pose,
		Eigen::MatrixBase<Derived1> &r_t_dest)
	{
		DimensionCheck<Vector6d>(r_t_pose);
		DimensionCheck<Vector6d>(r_t_dest);

		Eigen::Quaterniond q;
		C3DJacobians::AxisAngle_to_Quat(r_t_pose.template tail<3>(), q);
		q = q.conjugate();
		r_t_dest.template head<3>() = -q._transformVector(r_t_pose.template head<3>());
		Eigen::VectorBlock<Derived1, 3> v_rot_part = r_t_dest.template tail<3>(); // g++ requires a temporary
		C3DJacobians::Quat_to_AxisAngle(q, v_rot_part);
	}

	/**
	 *	@brief square
	 *	@param[in] x is input argument
	 *	@return Returns x squared.
	 *	@note This is utility function to support Matlab's ccode() directly.
	 */
	static inline double sqr(double x)
	{
		return x * x;
	}

	/**
	 *	@brief conjugate
	 *	@param[in] x is input argument
	 *	@return Returns x (it is real).
	 *	@note This is utility function to support Matlab's ccode() directly.
	 */
	static inline double conjugate(double x)
	{
		return x;
	}

	/**
	 *	@brief magnitude
	 *
	 *	@param[in] x is input argument
	 *	@param[in] y is input argument
	 *
	 *	@return Returns length of the (x, y) vector.
	 *
	 *	@note This is utility function to support Matlab's ccode() directly.
	 */
	static inline double abs(double x, double y) // complex abs
	{
		return sqrt(x * x + y * y);
	}

	/**
	 *	@brief calculates skew-symmetric matrix for a given vector
	 *	@param[in] v is input vector
	 *	@return Returns skew-symmetric matrix of v.
	 */
	static Eigen::Matrix3d v_SkewSym(Eigen::Vector3d v)
	{
		Eigen::Matrix3d m;
		m << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;
		return m;
	}

	/**
	 *	@brief converts xyz axis angle coordinates from absolute measurement to relative measurement
	 *		and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first vertex, in absolute coordinates
	 *	@param[in] r_t_vertex2 is the second vertex, also in absolute coordinates
	 *	@param[out] r_t_dest is filled with relative coordinates of the second vertex
	 *	@param[out] r_t_pose3_pose1 is filled with the first jacobian
	 *	@param[out] r_t_pose3_pose2 is filled with the second jacobian
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4> // want to be able to call this with differnent dest types (sometimes generic VectorXd / MatrixXd, sometimes with Vector3d / Matrix3d)
	static void Absolute_to_Relative(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest,
		Eigen::MatrixBase<Derived3> &r_t_pose3_pose1, Eigen::MatrixBase<Derived4> &r_t_pose3_pose2)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Vector6d>(r_t_vertex2);
		DimensionCheck<Vector6d>(r_t_dest);
		DimensionCheck<Matrix6d>(r_t_pose3_pose1);
		DimensionCheck<Matrix6d>(r_t_pose3_pose2);

		_ASSERTE((const void*)&r_t_vertex1 != (const void*)&r_t_dest);
		_ASSERTE((const void*)&r_t_vertex2 != (const void*)&r_t_dest); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Absolute_to_Relative() and then the jacobians are not computed correcly

		typedef Eigen::MatrixBase<Derived3> _TyDestMatrix0;
		typedef Eigen::MatrixBase<Derived4> _TyDestMatrix1;
		// used on multiple places below

#if 0
		// use the magical analytical jacobians (magobians)

		Eigen::Vector3d p1 = r_t_vertex1.head<3>();
		Eigen::Quaterniond q1;
		double f_theta = f_AxisAngle_to_Quat(r_t_vertex1.tail<3>(), q1);
		_ASSERTE(f_theta >= 0);

		Eigen::Vector3d p2 = r_t_vertex2.head<3>();
		Eigen::Quaterniond q2;
		double f_omega = f_AxisAngle_to_Quat(r_t_vertex2.tail<3>(), q2);
		_ASSERTE(f_omega >= 0);

		Eigen::Quaterniond q1_inv = q1.conjugate(); // the inverse rotation (also have .inverse() but that is not needed)

		r_t_dest.template head<3>() = q1_inv._transformVector(p2 - p1); // this is precise enough

		Eigen::Vector3d v_aang;
		Eigen::Quaterniond qdest = (q1_inv * q2)/*.normalized()*/; // quaternion product does not denormalize
		f_Quat_to_AxisAngle(qdest, v_aang); // use quaternion to calculate the rotation
		r_t_dest.template tail<3>() = v_aang;

		double ux = r_t_vertex1(0);
		double uy = r_t_vertex1(1);
		double uz = r_t_vertex1(2);
		double ua = r_t_vertex1(3);
		double ub = r_t_vertex1(4);
		double uc = r_t_vertex1(5);
		double vx = r_t_vertex2(0);
		double vy = r_t_vertex2(1);
		double vz = r_t_vertex2(2);
		double va = r_t_vertex2(3);
		double vb = r_t_vertex2(4);
		double vc = r_t_vertex2(5);
		// just rename the variables

#if 0
		double thetaMLVar = f_theta, sintheta = f_sin_theta, costheta = f_cos_theta,
			omega = f_omega, cosomega = f_cos_omega, sinomega = f_sin_omega;
		if(omega < 0.1 || thetaMLVar < 0.1 || /*fabs(omega - M_PI) < 0.1 || fabs(thetaMLVar - M_PI) < 0.1 ||*/ omega >= 2 * M_PI - 0.1 || thetaMLVar >= 2 * M_PI - 0.1) {
			// use the numerical jacobians
			/* TODO: make more efficient */

			//lets try it according to g2o
			const double delta = 1e-9;
			const double scalar = 1.0 / (delta);


			Matrix6d Eps;// = delta * Eigen::MatrixXd::Identity(6, 6); // MatrixXd needs to allocate storage on heap ... many milliseconds lost
			Eps = Matrix6d::Identity() * delta; // faster, all memory on stack

			_TyDestMatrix0 &H1 = r_t_pose3_pose1;
			_TyDestMatrix1 &H2 = r_t_pose3_pose2;
			_ASSERTE(H1.rows() == 6 && H1.cols() == 6 && H2.rows() == 6 && H2.cols() == 6); // just make sure the shape is right
			// can actually work inplace

			//Absolute_to_Relative(r_t_vertex1, r_t_vertex2, r_t_dest);
			for(int j = 0; j < 6; ++ j) {
				Vector6d d1, p_delta;
				Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
				Absolute_to_Relative(p_delta, r_t_vertex2, d1);
				H1.col(j) = (d1 - r_t_dest) * scalar;

				Vector6d d2;
				Relative_to_Absolute(r_t_vertex2, Eps.col(j), p_delta);
				Absolute_to_Relative(r_t_vertex1, p_delta, d2);
				H2.col(j) = (d2 - r_t_dest) * scalar;
			}
			// return jacobs

			return;
		}
		// handle low angles using numerical jacobians (avoid division by zero in the analytical equations)
#endif // 0

		double Ju[6][6] = {0}, Jv[6][6] = {0};
		Calculate3DJacobians(Ju, Jv, f_theta, f_omega,
			ux, uy, uz, ua, ub, uc, vx, vy, vz, va, vb, vc);
		// todo - simplify plus, actually only interested in

		_TyDestMatrix0 &ju = r_t_pose3_pose1;
		_TyDestMatrix1 &jv = r_t_pose3_pose2;
		for(int i = 0; i < 6; ++ i) {
			for(int j = 0; j < 6; ++ j) {
				ju(i, j) = Ju[i][j];
				jv(i, j) = Jv[i][j];
			}
		}
		// get the data out

		/*FILE *p_fw = fopen("jacobs3d_anal_manifold.m", "a");
		if(p_fw) {
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, r_t_vertex1, "u = ", ";\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, r_t_vertex2, "v = ", ";\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, r_t_dest, "d_gt = ", "; % difference between u and v\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, ju, "H1_gt = ", "; % the first jacobian\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, jv, "H2_gt = ", "; % the second jacobian\n");
			fprintf(p_fw, "%s", "\n% ---\n\n");
			fclose(p_fw);
		}*/
		// get some "ground truth" for analytical jacobian debugging

#ifdef _DEBUG
		Matrix6d num_H1;
		Matrix6d num_H2;
		Matrix6d num_H1_skew;
		Matrix6d num_H2_skew;
		// can actually work inplace

		{
			const double delta = 1e-9;
			const double scalar = 1.0 / (delta);

			Matrix6d Eps;// = delta * Eigen::MatrixXd::Identity(6, 6); // MatrixXd needs to allocate storage on heap ... many milliseconds lost
			Eps = Matrix6d::Identity() * delta; // faster, all memory on stack

			//Absolute_to_Relative(r_t_vertex1, r_t_vertex2, r_t_dest); // already did that
			for(int j = 0; j < 6; ++ j) {
				{
					Vector6d d1, p_delta;
					Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
					//p_delta = r_t_vertex1 + Eps.col(j);
					Absolute_to_Relative(p_delta, r_t_vertex2, d1);
					num_H1.col(j) = (d1 - r_t_dest) * scalar;

					Vector6d d2;
					Relative_to_Absolute(r_t_vertex2, Eps.col(j), p_delta);
					//p_delta = r_t_vertex2 + Eps.col(j);
					Absolute_to_Relative(r_t_vertex1, p_delta, d2);
					num_H2.col(j) = (d2 - r_t_dest) * scalar;
				}
				// proper jacobians

				/*{
					Vector6d d1, p_delta;
					//Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
					p_delta = r_t_vertex1 + Eps.col(j);
					Absolute_to_Relative(p_delta, r_t_vertex2, d1);
					num_H1_skew.col(j) = (d1 - r_t_dest) * scalar;

					Vector6d d2;
					//Relative_to_Absolute(r_t_vertex2, Eps.col(j), p_delta);
					p_delta = r_t_vertex2 + Eps.col(j);
					Absolute_to_Relative(r_t_vertex1, p_delta, d2);
					num_H2_skew.col(j) = (d2 - r_t_dest) * scalar;
				}*/
				// jacobians missing [+] at the input (skewed)
			}
			// return jacobs
		}

		double d[6][1] = {0};
		CalculateD(d, thetaMLVar, omega,
			ux, uy, uz, ua, ub, uc,
			vx, vy, vz, va, vb, vc);
		Vector6d dv;
		for(int i = 0; i < 6; ++ i)
			dv(i, 0) = d[i][0];
		// convert to a vector

		double f_norm_vertex = (dv - r_t_dest).norm();
		_ASSERTE(f_norm_vertex < 1e-10);

		double f_norm1 = (ju - num_H1/*_skew*/).norm();
		double f_norm2 = (jv - num_H2/*_skew*/).norm();
		double f_norm1p = (ju * jup - num_H1).norm();
		double f_norm2p = (jv * jvp - num_H2).norm();
		double f_normp1 = (jup * ju - num_H1).norm();
		double f_normp2 = (jvp * jv - num_H2).norm();
		/*double f_norm1t = (ju.transpose() - num_H1).norm();
		double f_norm2t = (jv.transpose() - num_H2).norm();*/
		/*double f_norm11 = (ju.block<3, 3>(0, 0) - num_H1.block<3, 3>(0, 0)).norm();
		double f_norm12 = (ju.block<3, 3>(3, 0) - num_H1.block<3, 3>(3, 0)).norm();
		double f_norm13 = (ju.block<3, 3>(0, 3) - num_H1.block<3, 3>(0, 3)).norm();
		double f_norm14 = (ju.block<3, 3>(3, 3) - num_H1.block<3, 3>(3, 3)).norm();
		double f_norm21 = (jv.block<3, 3>(0, 0) - num_H2.block<3, 3>(0, 0)).norm();
		double f_norm22 = (jv.block<3, 3>(3, 0) - num_H2.block<3, 3>(3, 0)).norm();
		double f_norm23 = (jv.block<3, 3>(0, 3) - num_H2.block<3, 3>(0, 3)).norm();
		double f_norm24 = (jv.block<3, 3>(3, 3) - num_H2.block<3, 3>(3, 3)).norm();*/
		/*double f_norm11t = (ju.transpose().block<3, 3>(0, 0) - num_H1.block<3, 3>(0, 0)).norm();
		double f_norm12t = (ju.transpose().block<3, 3>(3, 0) - num_H1.block<3, 3>(3, 0)).norm();
		double f_norm13t = (ju.transpose().block<3, 3>(0, 3) - num_H1.block<3, 3>(0, 3)).norm();
		double f_norm14t = (ju.transpose().block<3, 3>(3, 3) - num_H1.block<3, 3>(3, 3)).norm();
		double f_norm21t = (jv.transpose().block<3, 3>(0, 0) - num_H2.block<3, 3>(0, 0)).norm();
		double f_norm22t = (jv.transpose().block<3, 3>(3, 0) - num_H2.block<3, 3>(3, 0)).norm();
		double f_norm23t = (jv.transpose().block<3, 3>(0, 3) - num_H2.block<3, 3>(0, 3)).norm();
		double f_norm24t = (jv.transpose().block<3, 3>(3, 3) - num_H2.block<3, 3>(3, 3)).norm();*/
		double f_worse = std::max(f_norm1, f_norm2); // useless, just let the debugger break after all is calculated
#endif // _DEBUG
#elif 0
		// use the new Hauke jacobians (not working at the moment)

		Eigen::Vector3d tu = r_t_vertex1.template head<3>();
		Eigen::Matrix3d Ru = t_AxisAngle_to_RotMatrix(r_t_vertex1.template tail<3>());
		Eigen::Vector3d tv = r_t_vertex2.template head<3>();
		Eigen::Matrix3d Rv = t_AxisAngle_to_RotMatrix(r_t_vertex2.template tail<3>());
		// call RotMat

		Eigen::Vector3d td = Ru.transpose() * (tv - tu);
		Eigen::Matrix3d Rd = Ru.transpose() * Rv;

		/*Matrix6d Mu, Mv, Md;

		Mu.block<3, 3>(0, 0) = -v_SkewSym(r_t_vertex1.tail<3>());
		Mu.block<3, 3>(0, 3) = -v_SkewSym(r_t_vertex1.head<3>());
		Mu.block<3, 3>(3, 0).setZero();
		Mu.block<3, 3>(3, 3) = -v_SkewSym(r_t_vertex1.tail<3>());

		Mv.block<3, 3>(0, 0) = v_SkewSym(r_t_vertex2.tail<3>());
		Mv.block<3, 3>(0, 3) = v_SkewSym(r_t_vertex2.head<3>());
		Mv.block<3, 3>(3, 0).setZero();
		Mv.block<3, 3>(3, 3) = v_SkewSym(r_t_vertex2.tail<3>());

		Md.block<3, 3>(0, 0) = Rd;
		Md.block<3, 3>(0, 3) = v_SkewSym(td) * Rd;
		Md.block<3, 3>(3, 0).setZero();
		Md.block<3, 3>(3, 3) = Rd;

		if(0) {
			std::cout << "Mu =" << Mu << std::endl;
			std::cout << "Mv =" << Mv << std::endl;
			std::cout << "Md =" << Md << std::endl;
		}
		// debug

		Matrix6d I6;
		I6.setIdentity();

		_TyDestMatrix0 &H1 = r_t_pose3_pose1;
		_TyDestMatrix1 &H2 = r_t_pose3_pose2;

		H1 = (I6 + Mu * .5) * Md;
		H2 = -(I6 + Mv * .5);*/
		// eval jacobians

		Matrix6d Mu, Mv;

		r_t_dest.template head<3>() = td;
		r_t_dest.template tail<3>() = v_RotMatrix_to_AxisAngle(Rd);
		// calculate distance

		_TyDestVector &d = r_t_dest; // rename

		Mu.template block<3, 3>(0, 0) = -v_SkewSym(d.template tail<3>());
		Mu.template block<3, 3>(0, 3) = -v_SkewSym(d.template head<3>());
		Mu.template block<3, 3>(3, 0).setZero();
		Mu.template block<3, 3>(3, 3) = -v_SkewSym(d.template tail<3>());

		Mv.template block<3, 3>(0, 0) = v_SkewSym(d.template tail<3>());
		Mv.template block<3, 3>(0, 3) = v_SkewSym(d.template head<3>());
		Mv.template block<3, 3>(3, 0).setZero();
		Mv.template block<3, 3>(3, 3) = v_SkewSym(d.template tail<3>());

		if(0) {
			std::cout << "Mu =" << Mu << std::endl;
			std::cout << "Mv =" << Mv << std::endl;
		}
		// debug

		Matrix6d I6;
		I6.setIdentity();

		_TyDestMatrix0 &H1 = r_t_pose3_pose1;
		_TyDestMatrix1 &H2 = r_t_pose3_pose2;

		H1 = -(I6 + Mu * .5);
		H2 = (I6 + Mv * .5);
		// eval jacobians
#else // 0
		// use the numerical jacobians
		/* TODO: make more efficient */

		//lets try it according to g2o
		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		//jacobians have to be computed here
		//double eps = 1.e-9; //TODO: what is this???
		//double eps_ = sqrt(eps);
		//double eps_2 = 7.45e-9;

		Matrix6d Eps;// = delta * Eigen::MatrixXd::Identity(6, 6); // MatrixXd needs to allocate storage on heap ... many milliseconds lost
		//Matrix6d H1;// = Eigen::MatrixXd::Zero(6, 6);
		//Matrix6d H2;// = Eigen::MatrixXd::Zero(6, 6);
		Eps = Matrix6d::Identity() * delta; // faster, all memory on stack
		//H1.setZero();
		//H2.setZero(); // faster, no extra memory; actually not needed at all, the below code overwrites both H1 and H2

		_TyDestMatrix0 &H1 = r_t_pose3_pose1;
		_TyDestMatrix1 &H2 = r_t_pose3_pose2;
		_ASSERTE(H1.rows() == 6 && H1.cols() == 6 && H2.rows() == 6 && H2.cols() == 6); // just make sure the shape is right
		// can actually work inplace

		//Vector6d d;
		Absolute_to_Relative(r_t_vertex1, r_t_vertex2, r_t_dest);
		//r_t_dest = d; // possibly an unnecessary copy

		for(int j = 0; j < 6; ++ j) {
			Vector6d d1, p_delta;
			Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
			Absolute_to_Relative(p_delta, r_t_vertex2, d1);
			H1.col(j) = (d1 - r_t_dest) * scalar;

			Vector6d d2;
			Relative_to_Absolute(r_t_vertex2, Eps.col(j), p_delta);
			Absolute_to_Relative(r_t_vertex1, p_delta, d2);
			H2.col(j) = (d2 - r_t_dest) * scalar;
		}
		// return jacobs

#if 0
		Eigen::Vector4d dist_4D;
		dist_4D.head<3>() = r_t_dest.head<3>(); // dx, dy, dz
		dist_4D(3) = r_t_dest.tail<3>().norm(); // a total angle of rotation
		// the 4D distance

		Eigen::Matrix<double, 4, 6> H1_4, H2_4;
		for(int j = 0; j < 6; ++ j) {
			Vector6d d1, p_delta;
			Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
			Absolute_to_Relative(p_delta, r_t_vertex2, d1);
			Eigen::Vector4d d1_4;
			d1_4.head<3>() = d1.head<3>();
			d1_4(3) = d1.tail<3>().norm();
			H1_4.col(j) = (d1_4 - dist_4D) * scalar;

			Vector6d d2;
			Relative_to_Absolute(r_t_vertex2, Eps.col(j), p_delta);
			Absolute_to_Relative(r_t_vertex1, p_delta, d2);
			Eigen::Vector4d d2_4;
			d2_4.head<3>() = d2.head<3>();
			d2_4(3) = d2.tail<3>().norm();
			H2_4.col(j) = (d2_4 - dist_4D) * scalar;
		}
		// the 4D jacobs

		Eigen::Matrix<double, 4, 6> H1_transform = /*-*/Eigen::Matrix<double, 4, 6>::Identity(),
			H2_transform = Eigen::Matrix<double, 4, 6>::Identity(); // start with identity
		{
			/*const double Aa = r_t_vertex1(3), Ab = r_t_vertex1(4), Ac = r_t_vertex1(5);
			/const double Ba = r_t_vertex2(3), Bb = r_t_vertex2(4), Bc = r_t_vertex2(5);*/
			const double invD = 1 / dist_4D(3);
			/*H1_transform(3, 3) = 1 / 2 * invD * (2*Aa-2*Ba);
			H1_transform(3, 4) = 1 / 2 * invD * (2*Ab-2*Bb);
			H1_transform(3, 5) = 1 / 2 * invD * (2*Ac-2*Bc);
			H2_transform(3, 3) = 1 / 2 * invD * (-2*Aa+2*Ba);
			H2_transform(3, 4) = 1 / 2 * invD * (-2*Ab+2*Bb);
			H2_transform(3, 5) = 1 / 2 * invD * (-2*Ac+2*Bc);*/
			const double Aa = r_t_dest(3), Ab = r_t_dest(4), Ac = r_t_dest(5);
			H1_transform(3, 3) = invD * Aa;
			H1_transform(3, 4) = invD * Ab;
			H1_transform(3, 5) = invD * Ac;
			// matlab code
		}
		// get the projection function to go from 6D jacobians to 4D jacobians // this seems to work extremely well

		double f_err_H1 = (H1_4 - H1_transform * H1).norm();
		double f_err_H2 = (H2_4 - H1_transform/*H2_transform*/ * H2).norm();
		f_err_H1 /= std::max(1.0, H1.norm());
		f_err_H2 /= std::max(1.0, H2.norm());
		fprintf(stderr, "error in jacobians is %g %g\n", f_err_H1, f_err_H2);
		// evaluate precision
#endif // 0
		// debug - projection of the jacobian to a 4D quantity

		/*FILE *p_fw = fopen("jacobs3d.m", "a");
		if(p_fw) {
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, r_t_vertex1, "u = ", ";\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, r_t_vertex2, "v = ", ";\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, r_t_dest, "d_gt = ", "; % difference between u and v\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, H1, "H1_gt = ", "; % the first jacobian\n");
			CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, H2, "H2_gt = ", "; % the second jacobian\n");
			fprintf(p_fw, "%s", "\n% ---\n\n");
			fclose(p_fw);
		}*/
		// get some "ground truth" for analytical jacobian debugging
#endif // 0
	}

	/**
	 *	@brief converts xyz axis angle coordinates from relative measurement to absolute measurement
	 *		and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first vertex, in absolute coordinates
	 *	@param[in] r_t_vertex2 is the second vertex, also in absolute coordinates
	 *	@param[out] r_t_dest is filled with relative coordinates of the second vertex
	 *	@param[out] r_t_pose3_pose1 is filled with the first jacobian
	 *	@param[out] r_t_pose3_pose2 is filled with the second jacobian
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4> // want to be able to call this with differnent dest types (sometimes generic VectorXd / MatrixXd, sometimes with Vector3d / Matrix3d)
	static void Relative_to_Absolute(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest,
		Eigen::MatrixBase<Derived3> &r_t_pose3_pose1, Eigen::MatrixBase<Derived4> &r_t_pose3_pose2)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Vector6d>(r_t_vertex2);
		DimensionCheck<Vector6d>(r_t_dest);
		DimensionCheck<Matrix6d>(r_t_pose3_pose1);
		DimensionCheck<Matrix6d>(r_t_pose3_pose2);

		_ASSERTE((const void*)&r_t_vertex1 != (const void*)&r_t_dest);
		_ASSERTE((const void*)&r_t_vertex2 != (const void*)&r_t_dest); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Absolute_to_Relative() and then the jacobians are not computed correcly

		//lets try it according to g2o
		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		//jacobians have to be computed here
		//double eps = 1.e-9; //TODO: what is this???
		//double eps_ = sqrt(eps);
		//double eps_2 = 7.45e-9;

		Matrix6d Eps;// = delta * Eigen::MatrixXd::Identity(6, 6); // MatrixXd needs to allocate storage on heap ... many milliseconds lost
		//Matrix6d H1;// = Eigen::MatrixXd::Zero(6, 6);
		//Matrix6d H2;// = Eigen::MatrixXd::Zero(6, 6);
		Eps = Matrix6d::Identity() * delta; // faster, all memory on stack
		//H1.setZero();
		//H2.setZero(); // faster, no extra memory; actually not needed at all, the below code overwrites both H1 and H2

		Eigen::MatrixBase<Derived3> &H1 = r_t_pose3_pose1;
		Eigen::MatrixBase<Derived4> &H2 = r_t_pose3_pose2;
		_ASSERTE(H1.rows() == 6 && H1.cols() == 6 && H2.rows() == 6 && H2.cols() == 6); // just make sure the shape is right
		// can actually work inplace

		//Vector6d d;
		Relative_to_Absolute(r_t_vertex1, r_t_vertex2, r_t_dest);
		//Absolute_to_Relative(r_t_vertex1, r_t_vertex2, r_t_dest);
		//r_t_dest = d; // possibly an unnecessary copy

		for(int j = 0; j < 6; ++ j) {
			Vector6d d1, p_delta;
			Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
			Relative_to_Absolute(p_delta, r_t_vertex2, d1);
			H1.col(j) = (d1 - r_t_dest) * scalar;

			Vector6d d2;
			Relative_to_Absolute(r_t_vertex2, Eps.col(j), p_delta);
			Relative_to_Absolute(r_t_vertex1, p_delta, d2);
			H2.col(j) = (d2 - r_t_dest) * scalar;
		}
		// return jacobs
	}

	/**
	 *	@brief converts xyz axis angle coordinates from absolute measurement to relative measurement
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first 6DOF vertex, in absolute coordinates
	 *	@param[in] r_t_vertex2 is the second 3DOF vertex, also in absolute coordinates
	 *	@param[out] r_t_dest is filled with relative coordinates of the second vertex
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Absolute_to_Relative_Landmark(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Eigen::Vector3d>(r_t_vertex2);
		DimensionCheck<Eigen::Vector3d>(r_t_dest);

		Eigen::Quaterniond q1;
		AxisAngle_to_Quat(r_t_vertex1.template tail<3>(), q1);
		//Eigen::Quaterniond q1_inv = q1.conjugate(); // the inverse rotation (also have .inverse() but that is not needed)
		r_t_dest = q1.conjugate()._transformVector(r_t_vertex2.template head<3>() - r_t_vertex1.template head<3>()); // this is precise enough
		//r_t_dest.head<3>() = pQ1_inv * (p2 - p1); // or can use matrix to transform position
	}

	/**
	 *	@brief composes a relative transformation and a landmark position to yield
	 *		an aboslute landmark position
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first 6DOF vertex, in absolute coordinates
	 *	@param[in] r_t_vertex2 is the second 3DOF vertex, in relative coordinates
	 *	@param[out] r_t_dest is filled with absolute coordinates of the composed vertex
	 */
	template <class Derived0, class Derived1, class Derived2>
	static void Relative_to_Absolute_Landmark(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Eigen::Vector3d>(r_t_vertex2);
		DimensionCheck<Eigen::Vector3d>(r_t_dest);

		Eigen::Quaterniond q1;
		AxisAngle_to_Quat(r_t_vertex1.template tail<3>(), q1);
		r_t_dest = r_t_vertex1.template head<3>() + q1._transformVector(r_t_vertex2.template head<3>());
	}

	/**
	 *	@brief converts xyz axis angle coordinates from absolute measurement to relative measurement
	 *		and calculates the jacobians
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam Derived3 is Eigen derived matrix type for the fourth matrix argument
	 *	@tparam Derived4 is Eigen derived matrix type for the fifth matrix argument
	 *
	 *	@param[in] r_t_vertex1 is the first vertex, in absolute coordinates
	 *	@param[in] r_t_vertex2 is the second vertex, also in absolute coordinates
	 *	@param[out] r_t_dest is filled with relative coordinates of the second vertex
	 *	@param[out] r_t_pose3_pose1 is filled with the first jacobian
	 *	@param[out] r_t_pose3_pose2 is filled with the second jacobian
	 *
	 *	@note Neither of the input parameters may be reused as output.
	 */
	template <class Derived0, class Derived1, class Derived2, class Derived3, class Derived4> // want to be able to call this with differnent dest types (sometimes generic VectorXd / MatrixXd, sometimes with Vector3d / Matrix3d)
	static void Absolute_to_Relative_Landmark(const Eigen::MatrixBase<Derived0> &r_t_vertex1,
		const Eigen::MatrixBase<Derived1> &r_t_vertex2, Eigen::MatrixBase<Derived2> &r_t_dest,
		Eigen::MatrixBase<Derived3> &r_t_pose3_pose1, Eigen::MatrixBase<Derived4> &r_t_pose3_pose2)
	{
		DimensionCheck<Vector6d>(r_t_vertex1);
		DimensionCheck<Eigen::Vector3d>(r_t_vertex2);
		DimensionCheck<Eigen::Vector3d>(r_t_dest);
		DimensionCheck<Eigen::Matrix<double, 3, 6> >(r_t_pose3_pose1);
		DimensionCheck<Eigen::Matrix3d>(r_t_pose3_pose2);

		_ASSERTE((const void*)&r_t_vertex1 != (const void*)&r_t_dest); // should not happen, they are different dimension - unless someone maps the head of it
		_ASSERTE((const void*)&r_t_vertex2 != (const void*)&r_t_dest); // cant compare addresses directly, the inputs may be maps or expressions and their type may not be convertible
		// make sure that neither vector is the same as the destination, otherwise it is overwritten
		// by the first call to Absolute_to_Relative() and then the jacobians are not computed correcly

		//lets try it according to g2o
		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		//jacobians have to be computed here
		//double eps = 1.e-9; //TODO: what is this???
		//double eps_ = sqrt(eps);
		//double eps_2 = 7.45e-9;

		Matrix6d Eps;// = delta * Eigen::MatrixXd::Identity(6, 6); // MatrixXd needs to allocate storage on heap ... many milliseconds lost
		//Matrix6d H1;// = Eigen::MatrixXd::Zero(6, 6);
		//Matrix6d H2;// = Eigen::MatrixXd::Zero(6, 6);
		Eps = Matrix6d::Identity() * delta; // faster, all memory on stack
		//H1.setZero();
		//H2.setZero(); // faster, no extra memory; actually not needed at all, the below code overwrites both H1 and H2

		Eigen::MatrixBase<Derived3> &H1 = r_t_pose3_pose1;
		Eigen::MatrixBase<Derived4> &H2 = r_t_pose3_pose2;
		// can actually work inplace

		Absolute_to_Relative_Landmark(r_t_vertex1, r_t_vertex2, r_t_dest);
		//r_t_dest = d; // possibly an unnecessary copy

		for(int j = 0; j < 6; ++ j) {
			Vector6d p_delta;
			Eigen::Vector3d d1;
			Relative_to_Absolute(r_t_vertex1, Eps.col(j), p_delta);
			Absolute_to_Relative_Landmark(p_delta, r_t_vertex2, d1);
			H1.col(j) = (d1 - r_t_dest) * scalar;
		}

		for(int j = 0; j < 3; ++ j) {
			Eigen::Vector3d p_delta, d1;
			p_delta = r_t_vertex2 + Eps.col(j).template head<3>();//col(j);
			Absolute_to_Relative_Landmark(r_t_vertex1, p_delta, d1);
			H2.col(j) = (d1 - r_t_dest) * scalar;
		}
	}

	/**
	 *	@brief SE(3) coordinate frame
	 */
	struct TSE3 {
		struct from_se3_vector_tag {}; /**< @brief tag-scheduling type for initialization from an exponent of a \f$\mathfrak{se}(3)\f$ vector */
		struct from_tR_vector_tag {}; /**< @brief tag-scheduling type for initialization from a translation rotation vector */

		static const from_se3_vector_tag from_se3_vector; /**< @brief tag-scheduling value for initialization from an exponent of a \f$\mathfrak{se}(3)\f$ vector */
		static const from_tR_vector_tag from_tR_vector; /**< @brief tag-scheduling value for initialization from a translation rotation vector */

		Eigen::Vector3d v_translation; /**< @brief translation vector */
		Eigen::Quaterniond t_rotation; /**< @brief rotation as normalized quaternion */

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/**
		 *	@brief default constructor; has no effect
		 */
		inline TSE3()
		{}

		/**
		 *	@brief constructor; initializes the pose from exponential of a \f$\mathfrak{se}(3)\f$ vector
		 *
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *
		 *	@param[in] r_v_vec is a vector in \f$\mathfrak{se}(3)\f$ to initialize from, obtained e.g. by a call to v_Log()
		 *	@param[in] t_tag is used for tag scheduling (value unused)
		 */
		template <class Derived0>
		inline TSE3(const Eigen::MatrixBase<Derived0> &r_v_vec, from_se3_vector_tag UNUSED(t_tag))
		{
			//DimensionCheck<Eigen::Vector6d>(r_v_vec); // checked inside as well
			Exp(r_v_vec);
		}

		/**
		 *	@brief constructor; initializes the pose from a \f$R^6\f$ vector
		 *
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *
		 *	@param[in] r_v_vec is a vector in \f$R^6\f$ to initialize from
		 *	@param[in] t_tag is used for tag scheduling (value unused)
		 */
		template <class Derived0>
		inline TSE3(const Eigen::MatrixBase<Derived0> &r_v_vec, from_tR_vector_tag UNUSED(t_tag))
			:v_translation(r_v_vec.template head<3>())
		{
			DimensionCheck<Eigen::Vector6d>(r_v_vec);
			C3DJacobians::AxisAngle_to_Quat(r_v_vec.template tail<3>(), t_rotation);
		}

		/**
		 *	@brief constructor; initializes the fields in this structure
		 *
		 *	@param[in] r_v_translation is translation vector
		 *	@param[in] r_t_rotation is rotation as normalized quaternion
		 */
		inline TSE3(const Eigen::Vector3d &r_v_translation, const Eigen::Quaterniond &r_t_rotation)
			:v_translation(r_v_translation), t_rotation(r_t_rotation)
		{}

		/**
		 *	@brief gets identity pose
		 *	@return Returns an identity pose.
		 */
		static inline TSE3 t_Identity()
		{
			return TSE3(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
		}

		/**
		 *	@brief sets this to be an identity pose
		 */
		void Identity()
		{
			v_translation.setZero();
			t_rotation.setIdentity();
		}

		/**
		 *	@brief inverses this pose
		 */
		void Invert()
		{
			//t_rotation = t_rotation.inverse();
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			t_rotation = t_rotation.conjugate(); // faster but only if normalized
			v_translation = t_rotation._transformVector(-v_translation);
		}

		/**
		 *	@brief calculates logarithm of this pose
		 *	@return Returns the logarithm of this pose.
		 */
		Eigen::Vector6d v_Log() const
		{
			return C3DJacobians::v_Log(*this);
		}

		/**
		 *	@brief converts this pose to vectorial form
		 *	@return Returns the translation and axis-angle rotation (tR) vector representation of this pose.
		 */
		Eigen::Vector6d v_tR() const
		{
			Eigen::Vector6d v_result;
			v_result.head<3>() = v_translation;
			Eigen::VectorBlock<Eigen::Vector6d, 3> v_rot_part = v_result.tail<3>(); // g++ requires a temporary
			C3DJacobians::Quat_to_AxisAngle(t_rotation, v_rot_part);
			return v_result;
		}

		/**
		 *	@brief overwrites this pose by an exponent of a 6D vector
		 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
		 *	@param[in] r_v_vec is a vector in sE(3), obtained e.g. by a call to v_Log()
		 *	@deprecated It is preferable to use the constructor with \ref from_se3_vector.
		 */
		template <class Derived0>
		void Exp(const Eigen::MatrixBase<Derived0> &r_v_vec)
		{
			//DimensionCheck<Eigen::Vector6d>(r_v_vec); // checked inside CSE3Jacobians::t_Exp() as well
			*this = C3DJacobians::t_Exp(r_v_vec);
		}

		/**
		 *	@brief calculates composition of two poses, this pose on the left
		 *	@param[in] r_t_other is the other pose to compose
		 */
		void operator *=(const TSE3 &r_t_other)
		{
			_ASSERTE(&r_t_other != this); // r_t_other must be a different object
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			v_translation.noalias() += t_rotation._transformVector(r_t_other.v_translation);
			t_rotation *= r_t_other.t_rotation;
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is still notmalized
		}

		/**
		 *	@brief calculates relative pose \f$this = this \ominus r\_t\_other = this^{-1} \cdot r\_t\_other\f$
		 *	@param[in] r_t_other is the other pose to relate to
		 */
		void Inverse_Compose(const TSE3 &r_t_other)
		{
#ifdef _DEBUG
			TSE3 t_ref = *this;
			t_ref.Invert(); // invert
			t_ref *= r_t_other; // right multiply
			// calculate reference and compare in debug!!!
#endif // _DEBUG

			_ASSERTE(&r_t_other != this); // r_t_other must be a different object
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			_ASSERTE(fabs(r_t_other.t_rotation.norm() - 1) < 1e-5); // make sure r_t_other is notmalized

			//t_rotation = t_rotation.inverse();
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is notmalized
			t_rotation = t_rotation.conjugate(); // faster but only if normalized
			v_translation = t_rotation._transformVector(r_t_other.v_translation - v_translation);
			t_rotation *= r_t_other.t_rotation;
			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is still notmalized
			// saves one quaternion transformation

			/*Eigen::Quaterniond other_inv_rot = r_t_other.t_rotation.conjugate(); // faster but only if normalized
			v_translation.noalias() += t_rotation._transformVector(other_inv_rot._transformVector(-r_t_other.v_translation));
			t_rotation *= other_inv_rot;*/ // the other way around

			_ASSERTE(fabs(t_rotation.norm() - 1) < 1e-5); // make sure it is still notmalized

#ifdef _DEBUG
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
			v.template head<3>() = t_rotation._transformVector(r_v_vec) + w * v_translation; // calculate the transform
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
			v.template head<3>() = t_rotation.conjugate()._transformVector(r_v_vec - w * v_translation); // calculate the transform
			if(n_dim == 4)
				v.template tail<1>()(0) = w; // pass it on
			// the same complexity as the forward transformation

#ifdef _DEBUG
			TSE3 t_inv = *this;
			t_inv.Invert();
			double f_error = (t_inv * r_v_vec - v).norm();
			_ASSERTE(f_error < 1e-10);
			// make sure it does what it is supposed to
#endif // _DEBUG

			return v;
		}
	};

	/**
	 *	@brief converts a SE(3) coordinate frame to a 6D vector via the logarithmic map to \f$\mathfrak{se}(3)\f$
	 *	@param[in] r_t_pose is a SE(3) coordinate frame
	 *	@return Returns the corresponding 6D vector containing 3D translation component and 3D axis angle rotation.
	 *	@note This representation is different from the 6D vectors with translation and rotation
	 *		as axis-angle (a vectorial representation of SE(3)) that most of the functions in this class use.
	 */
	static Eigen::Vector6d v_Log(const TSE3 &r_t_pose)
	{
		Eigen::Vector6d v_result;
		//Eigen::VectorBlock<Eigen::Block<Eigen::Vector7d, 4, 1>, 3> v_rot_part = v_result.tail<4>().head<3>(); // g++ requires a temporary
		Eigen::VectorBlock<Eigen::Vector6d, 3> v_rot_part = v_result.tail<3>(); // g++ requires a temporary
		double f_angle = C3DJacobians::f_Quat_to_AxisAngle(r_t_pose.t_rotation, v_rot_part);
		_ASSERTE(f_angle >= 0); // make sure that the angle returned is positive
		//v_result(6) = log(r_t_pose.f_scale);

		_ASSERTE(f_angle < 2 * M_PI - 1e-3); // make sure this does not approach 2pi
		// will run into trouble if f_angle is a multiple of 2pi, but the angles of rotation
		// vectors converted from a quaternion are [-pi, pi] so that should be safe

		const double f_a = -.5, f_b = (f_angle > 1e-12)? (1 - .5 * f_angle /
			tan(.5 * f_angle)) / (f_angle * f_angle) : 1.0 / 12;
		Eigen::Vector3d v_cross = v_rot_part.cross(r_t_pose.v_translation);
		v_result.head<3>() = r_t_pose.v_translation + f_a * v_cross + f_b * v_rot_part.cross(v_cross);
		// log map

		return v_result;
	}

	/**
	 *	@brief converts a 6D vector in \f$\mathfrak{se}(3)\f$ to a SE(3) coordinate frame via the exponential map
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *	@param[in] r_v_vec is a 6D vector containing 3D translation component and 3D axis angle rotation
	 *	@return Returns the corresponding SE(3) coordinate frame.
 	 *	@note This representation is different from the 6D vectors with translation and rotation
	 *		as axis-angle (a vectorial representation of SE(3)) that most of the functions in this class use.
	 */
	template <class Derived>
	static TSE3 t_Exp(const Eigen::MatrixBase<Derived> &r_v_vec)
	{
		DimensionCheck<Eigen::Vector6d>(r_v_vec); // must be a 7D vector

		Eigen::Vector3d t = r_v_vec.template head<3>();
		Eigen::Vector3d w = r_v_vec.template tail</*4>().template head<*/3>();
		// avoid recomputation in case r_v_exp is an expression

		const double f_theta_sq = w.squaredNorm();
		const double f_theta = sqrt(f_theta_sq);

		TSE3 t_result;

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
		//t_result.f_scale = exp(r_v_vec(6));
		C3DJacobians::AxisAngle_to_Quat(w, t_result.t_rotation); // fast, reasonably precise

		/*

		rotation of a vector r by a quaternion w q (w is real, q is imaginary part of the quat) is:

		r' = r + 2w(q × r) + 2q × (q × r)	// × is cross product, multiplications hidden

		if theta is the rotation angle, w = cos(theta / 2)

		r' = r + 2cos(theta / 2)(q × r) + 2q × (q × r)

		let a be axis-angle vector, a = q theta / sin(theta / 2), then q = a sin(theta / 2) / theta

		r' = r + 2cos(theta / 2) sin(theta / 2) / theta (a × r) + 2 * (sin(theta / 2))^2 / theta^2 a × (a × r)

		r' = r + A (a × r) + B a × (a × r)	// where A = 2cos(theta / 2) sin(theta / 2) / theta = sin(theta) / theta, B = 2 (sin(theta / 2))^2 / theta^2 = (1 - cos(theta)) / theta^2 (http://www.wolframalpha.com/input/?i=2*cos(theta+/+2)*+sin(theta+/+2)+/+theta&dataset= and http://www.wolframalpha.com/input/?i=2+*+(sin(theta+/+2))^2+/+theta^2&dataset=)

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

		r' = r + 2 * sin((1 - t) theta / 2) / sin(theta / 2) cos(theta / 2) + sin(t theta / 2) / sin(theta / 2) sign(cos(theta / 2)) * (q × r) + 2q × (q × r)

		r' = r + (2 * sin((1 - t) theta / 2) / sin(theta / 2) cos(theta / 2) + sin(t theta / 2) / sin(theta / 2) sign(cos(theta / 2)) sin((1 - t) theta / 2) / theta) * (a × r) +
			 2 * (sin((1 - t) theta / 2) / theta)^2 a × (a × r)

		 blah ...

		*/

		return t_result;
	}
};

/** @} */ // end of group

#endif // !__3D_SOLVER_BASE_INCLUDED
