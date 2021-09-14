/*
 * Structures.h
 *
 *  Created on: Sep 9, 2021
 *      Author: isolony
 */

#ifndef INCLUDE_MIDEND_STRUCTURES__H_
#define INCLUDE_MIDEND_STRUCTURES__H_

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <memory>

#include "Eigen/Eigen"
#include "Eigen/Core"
#include "Eigen/SVD"
#include "Eigen/Eigenvalues"
#include "Eigen/StdVector"

#include "slam/Sim3SolverBase.h" // CBAJacobians

#include "midend/rapidjson/document.h"
#include "midend/rapidjson/writer.h"
#include "midend/rapidjson/stringbuffer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "slam/LinearSolver_UberBlock.h"
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/NonlinearSolver_Lambda_LM.h"
#include "slam/SE3_Types.h"

using namespace std;
using namespace Eigen;
using namespace rapidjson;

class CTimeFrame;
class CStructure;

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIRP
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    float    phase;                     ///< rotor phase
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  typedef PointXYZIRP VelodynePoint;

  class InterpolationSE3 {
  public:
    typedef boost::shared_ptr<InterpolationSE3> Ptr;

    virtual ~InterpolationSE3() {
    }

    virtual Eigen::Affine3f estimate(const float t) const =0;
  };

  class BSplineSE3 : public InterpolationSE3 {
  public:
    BSplineSE3(const std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > &control_poses_);

    virtual ~BSplineSE3() {
    }

    virtual Eigen::Affine3f estimate(const float t) const;

  private:
    static const int DEGREE = 4;

    std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > control_poses;
    Eigen::Matrix4f C;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Omega;
  };

  class LinearInterpolationSE3 : public InterpolationSE3 {
  public:
    LinearInterpolationSE3(const Eigen::Affine3f &start_, const Eigen::Affine3f &finish_) :
      init(start_), delta(start_.inverse() * finish_) {
    }

    virtual ~LinearInterpolationSE3() {
    }

    virtual Eigen::Affine3f estimate(const float t) const;

  private:
    const Eigen::Affine3f init, delta;
  };

  typedef struct less_float_struct : std::binary_function<float,float,bool> {
    const float TOLERANCE = 1e-6;
    bool operator()(const float f1, const float f2) const
    {
        return f1 < f2 && std::abs(f2 - f1) >= TOLERANCE;
    }
  } less_float;

  class InterpolationBuffered {
  public:
    typedef std::map<float, Eigen::Affine3f, less_float, Eigen::aligned_allocator<std::pair<const float, Eigen::Affine3f> > > map_type;
    typedef boost::shared_ptr<InterpolationBuffered> Ptr;

    InterpolationBuffered(const InterpolationSE3::Ptr &method_,
        const float phase_offset_, const float phase_scale_) :
      method(method_), phase_offset(phase_offset_), phase_scale(phase_scale_) {
    }

    Eigen::Affine3f estimate(const float t);

  private:
    const InterpolationSE3::Ptr method;
    const float phase_offset, phase_scale;
    map_type interpolated_poses;
  };

  Eigen::Affine3f average_transformations(const std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > &transformations);

  BSplineSE3::BSplineSE3(const std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > &control_poses_) :
    control_poses(control_poses_) {
    C << 6,  0,  0,  0,
         5,  3, -3,  1,
         1,  3,  3, -2,
         0,  0,  0,  1;
    C *= 1.0/6.0;
    assert(control_poses.size() == DEGREE);
    Omega.push_back(Eigen::Matrix4f::Zero());
    for(int i = 1; i < DEGREE; i++) {
      Omega.push_back((control_poses[i-1].inverse() * control_poses[i]).matrix().log());
    }
  }

  Eigen::Affine3f BSplineSE3::estimate(const float t) const {
    Eigen::Vector4f B = C * Eigen::Vector4f(1, t, t*t, t*t*t);
    Eigen::Matrix4f result = control_poses[0].matrix();
    for(int i = 1; i < DEGREE; i++) {
      result = result * (B(i) * Omega[i]).exp();
    }
    Eigen::Affine3f result_t(result);
    return result_t;
  }

  Eigen::Affine3f LinearInterpolationSE3::estimate(const float portion) const {
    assert(0.0 <= portion && portion  <= 1.0);

    Eigen::Quaternionf rotation(delta.rotation());
    rotation = Eigen::Quaternionf::Identity().slerp(portion, rotation);
    Eigen::Translation3f translation(delta.translation() * portion);

    return init * (translation * rotation);
  }

  Eigen::Affine3f InterpolationBuffered::estimate(float t) {
    t = (t + phase_offset) * phase_scale;

    map_type::const_iterator it = interpolated_poses.find(t);
    if(it != interpolated_poses.end()) {
      return it->second;
    } else {
      interpolated_poses[t] = method->estimate(t);
      return interpolated_poses[t];
    }
  }

  Eigen::Affine3f average_transformations(const std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > &transformations) {
    Eigen::Vector3f translation_sum(0, 0, 0);
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    for(std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> >::const_iterator t = transformations.begin(); t < transformations.end(); t++) {

      translation_sum = translation_sum + t->translation();
      Eigen::Quaternionf Q(t->rotation());
      Eigen::Vector4f Q_vec(Q.w(), Q.x(), Q.y(), Q.z());
      A = A + Q_vec * Q_vec.transpose();
    }

    translation_sum *= 1.0 / transformations.size();

    A = A * (1.0/transformations.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver;
    solver.compute(A);
    Eigen::Vector4f evalues = solver.eigenvalues();
    int largest_vector;
    float largest_value = -INFINITY;
    for(int i = 0; i < 4; i++) {
      if(evalues(i) > largest_value) {
        largest_vector = i;
      }
    }

    Eigen::Vector4f Q_vec = solver.eigenvectors().col(largest_vector);
    Eigen::Quaternionf Q(Q_vec(0), Q_vec(1), Q_vec(2), Q_vec(3));

    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block(0, 0, 3, 3) = Q.matrix();
    matrix.block(0, 3, 3, 1) = translation_sum;

    return Eigen::Affine3f(matrix);
  }
}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIRP,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, phase, phase))

class C3DPose {
protected:
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> m_v_pose;
	// default transformation model is direct one (as in pose of object in 3D and its rotation) + scale
	bool b_pose_valid;

public:
	C3DPose(Eigen::Matrix<double, 6, 1, Eigen::DontAlign> r_pose = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>::Zero())
		:b_pose_valid(true)
	{
		m_v_pose = r_pose;
	}

	C3DPose(Eigen::Matrix4d r_Rt, bool fromRt)
			:b_pose_valid(true)
	{
		m_v_pose = RtToAa(r_Rt);
	}

	C3DPose(const C3DPose &r_other)
		:m_v_pose(r_other.m_v_pose), b_pose_valid(r_other.b_pose_valid)
	{}

	C3DPose &operator =(const C3DPose &r_other)
	{
		m_v_pose = r_other.m_v_pose;
		b_pose_valid = r_other.b_pose_valid;
		return *this;
	}

	static Eigen::Matrix4d AaToRt(Eigen::Matrix<double, 6, 1> aa)
	{
		Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
		out.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(aa.tail(3));
		out.block(0, 3, 3, 1) = aa.head(3);

		return out;
	}

	static Eigen::Matrix<double, 6, 1> RtToAa(Eigen::Matrix4d Rt)
	{
		Eigen::Matrix<double, 6, 1> out = Eigen::Matrix<double, 6, 1>::Zero();
		out.tail(3) = C3DJacobians::Operator_arot(Rt.block(0, 0, 3, 3));
		out.head(3) = Rt.block(0, 3, 3, 1);

		return out;
	}

	/**
	 *	@brief sets rotation as 3x3 rotation matrix
	 *	@param n_R rotation matrix to set
	 *	@param b_renormalize set true if rotation matrix may not have norm of sqrt(3)
	 *	@param invert set true to invert returned rotation
	 */
	void setR(Eigen::Matrix3d n_R, bool b_renormalize = true, bool invert = false) // note that this invalidates the translation component
	{
		Eigen::Matrix3d R = n_R;
		Eigen::Quaterniond quat(R);

		// normalize
		if(b_renormalize)
			quat.normalize();

		if(invert)
			quat = quat.inverse();

		Eigen::Matrix<double, 3, 1, Eigen::DontAlign> axis;
		C3DJacobians::Quat_to_AxisAngle(quat, axis);

		m_v_pose.segment<3>(3) = axis;
	}
	/**
	 *	@brief sets rotation in axis angle representation
	 *	@param r_axis rotation to set
	 *	@param invert set true to invert inserted rotation
	 */
	void setAxisAngle(Eigen::Matrix<double, 3, 1, Eigen::DontAlign> r_axis, bool invert = false) // note that this invalidates the translation component
	{
		Eigen::Matrix<double, 3, 1, Eigen::DontAlign> axis = r_axis;
		if(invert)
		{
			Eigen::Quaterniond quat;
			C3DJacobians::AxisAngle_to_Quat(axis, quat);
			quat = quat.inverse();
			C3DJacobians::Quat_to_AxisAngle(quat, axis);
		}

		m_v_pose.segment<3>(3) = axis;
	}
	/**
	 *	@brief gets rotation
	 *	@param invert set true to invert returned rotation
	 *	@return Returns rotation (3x3 submatrix of the P matrix).
	 */
	Eigen::Matrix3d r_R(bool invert = false) const
	{
		Eigen::Matrix3d R = C3DJacobians::Operator_rot(m_v_pose.segment<3>(3));
		if(invert)
			R = R.inverse().eval();

		return R;
	}
	Eigen::Matrix3f r_fR(bool invert = false) const
	{
		Eigen::Matrix3d R = C3DJacobians::Operator_rot(m_v_pose.segment<3>(3));
		if(invert)
			R = R.inverse().eval();

		Eigen::Matrix3f Rf = R.cast <float> ();

		return Rf;
	}
	/**
	 *	@brief gets rotation in axis angle representation
	 *	@param invert set true to invert returned rotation
	 *	@return Returns rotation axis angle.
	 */
	Eigen::Matrix<double, 3, 1, Eigen::DontAlign> r_AA(bool invert = false) const
	{
		if(!invert)
			return m_v_pose.segment<3>(3);
		else
		{
			Eigen::Quaterniond quat;
			C3DJacobians::AxisAngle_to_Quat(m_v_pose.segment<3>(3), quat);
			quat = quat.inverse().normalized();
			Eigen::Matrix<double, 3, 1, Eigen::DontAlign> axis;
			C3DJacobians::Quat_to_AxisAngle(quat, axis);
			return axis;
		}
	}

	/**
	 *	@brief gets translation
	 *	@param invert set true to invert returned translation
	 *	@return Returns translation (4th column of the P matrix).
	 */
	Eigen::Matrix<double, 3, 1, Eigen::DontAlign> r_C(bool invert = false) const
	{
		if(!invert)
			return m_v_pose.head<3>();
		else
		{
			Eigen::Quaterniond quat;
			C3DJacobians::AxisAngle_to_Quat(m_v_pose.segment<3>(3), quat);

			return  - (quat.inverse() * m_v_pose.head<3>());
		}
	}

	/**
	 *	@brief sets pose, representation pose + axis angles
	 *	@param data pose vector
	 *	@param invert invert pose before inserting
	 */
	void setRepresentation_6D(Eigen::Matrix<double, 6, 1, Eigen::DontAlign> data, bool invert = false)
	{
		if(!invert)
			m_v_pose.head(6) = data;
		else
		{
			Eigen::Quaterniond quat;
			C3DJacobians::AxisAngle_to_Quat(data.tail<3>(), quat);
			quat = quat.inverse().normalized();
			Eigen::Matrix<double, 6, 1, Eigen::DontAlign> rep;
			rep.head<3>() = -(quat * data.head<3>()); // set translation
			Eigen::Matrix<double, 3, 1, Eigen::DontAlign> axis;
			C3DJacobians::Quat_to_AxisAngle(quat, axis);
			rep.tail<3>() = axis;

			m_v_pose.head(6) = rep;
		}
		b_pose_valid = true;
	}

	/**
	 *	@brief gets 6D representation in pose + axis angle format
	 *	@param invert set true to invert returned pose
	 *	@return Returns camera pose as translarion + axis-angle.
	 */
	Eigen::Matrix<double, 6, 1> v_Representation_6D(bool invert = false) const
	{
		if(!invert)
			return m_v_pose.head(6);
		else
		{
			Eigen::Matrix<double, 6, 1, Eigen::DontAlign> rep;
			rep.head<3>() = r_C(true); // set translation
			rep.tail<3>() = r_AA(true);
			return rep;
		}
	}

	/**
	 *	@brief gets 7D representation,  - axis angle pose + quaternion
	 *	@param invert set true to invert returned pose
	 *	@return Returns camera pose as sim3 pose + quaternion
	 */
	Eigen::Matrix<double, 7, 1, Eigen::DontAlign> v_Representation_Quat(bool invert = false) const
	{
		Eigen::Matrix<double, 6, 1, Eigen::DontAlign> rep6d = v_Representation_6D(invert);

		// axis to quat
		Eigen::Quaterniond quat;
		Eigen::Matrix<double, 3, 1, Eigen::DontAlign> axis = rep6d.tail<3>();
		C3DJacobians::AxisAngle_to_Quat(axis, quat);

		Eigen::Matrix<double, 7, 1, Eigen::DontAlign> ret;
		ret.head<3>() = rep6d.head<3>();
		ret.tail<4>() << quat.x(), quat.y(), quat.z(), quat.w();

		return ret;
	}

	/**
	 *	@brief sets pose
	 *	@tparam T is type of pose matrix (must be a 4x4 matrix)
	 *	@param[in] n_P is new value of the P matrix
	 *	@param invert set true to invert pose before inserting
	 */
	template <class T>
	void setP(const T &n_P, bool invert = false)
	{
		// extract axis angles
		Eigen::Matrix3d R = (n_P.block(0, 0, 3, 3)).template cast<double>();
		Eigen::Matrix<double, 3, 1, Eigen::DontAlign> axis = C3DJacobians::Operator_arot(R);
		Eigen::Matrix<double, 3, 1, Eigen::DontAlign> C = n_P.block(0, 3, 3, 1);

		setAxisAngle(axis, invert);

		if(invert)
			m_v_pose.head<3>() = - (R.inverse() * C);
		else
			m_v_pose.head<3>() = C;

		//setC(C, invert);

		b_pose_valid = true;
	}
	/**
	 *	@brief set 4x4 transform matrix camera -> world (direct)
	 *	@param P inserted pose 4x4 matrix
	 */
	void setPc2w(Eigen::Matrix<double, 4, 4, Eigen::DontAlign> P)
	{
		setP(P, false);
	}
	/**
	 *	@brief set 4x4 transform matrix world -> camera (inverted)
	 *	@param P inserted pose 4x4 matrix
	 */
	void setPw2c(Eigen::Matrix<double, 4, 4, Eigen::DontAlign> P)
	{
		setP(P, true);
	}

	/**
	 *	@brief gets pose
	 *	@return Returns value of the P matrix c2w by def, w2c with invert.
	 */
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> r_P(bool invert = false) const
	{
		Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
		P.block<3, 3>(0, 0) = C3DJacobians::Operator_rot(m_v_pose.segment<3>(3));
		P.block<3, 1>(0, 3) = m_v_pose.head<3>();

		if(!invert)
			return P;
		else
			return P.inverse();
	}
	/**
	 *	@brief returns transform matrix from world to cam (inverted)
	 */
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> r_Pw2c() const
	{
		return r_P(true);
	}
	/**
	 *	@brief returns transform matrix from cam to world (direct)
	 */
	Eigen::Matrix<double, 4, 4, Eigen::DontAlign> r_Pc2w() const
	{
		return r_P(false);
	}

	/**
	 *	@brief returns true if the pose is valid
	 */
	bool poseValid()
	{
		return b_pose_valid;
	}
	/**
	 *	@brief invalidates the pose
	 */
	void Invalidate()
	{
		b_pose_valid = false;
	}
};

enum TF { tfbase, tflidarBase, tflidar0, tflidar1 };
struct MobileBase
{
private:
	C3DPose base;
	C3DPose baseToLidarBase;
	C3DPose lidarBaseToLidar0;
	C3DPose lidarBaseToLidar1;
	std::map<TF, Matrix<double, 4, 4, DontAlign> > tfMap;
public:
	MobileBase(std::string fileImuToBase, std::string fileBaseToLidar)
	{
		Matrix<double, 4, 4, DontAlign> mat0 = Matrix<double, 4, 4, DontAlign>::Identity();
		Matrix<double, 4, 4, DontAlign> mat1 = Matrix<double, 4, 4, DontAlign>::Identity();
		Matrix<double, 4, 4, DontAlign> mat2 = Matrix<double, 4, 4, DontAlign>::Identity();

		FILE *f = fopen(fileImuToBase.c_str(), "r");
		if(f != NULL)
		{
			double val;
			for(int a = 0; a < 12; ++a)
			{
				if(fscanf(f, "%lf", &val) == 1)
				{
					mat0(a / 4, a % 4) = val;
				}
				else
				{
					std::cerr << "Error loading from " << fileImuToBase << std::endl;
					break;
				}
			}
		}
		else
		{
			std::cerr << "Error opening " << fileImuToBase << std::endl;
		}
		fclose(f);

		f = fopen(fileBaseToLidar.c_str(), "r");
		if(f != NULL)
		{
			double val;
			for(int a = 0; a < 12; ++a)
			{
				if(fscanf(f, "%lf", &val) == 1)
				{
					mat1(a / 4, a % 4) = val;
				}
				else
				{
					std::cerr << "Error loading from " << fileBaseToLidar << std::endl;
					break;
				}
			}
			// mat 1
			for(int a = 0; a < 12; ++a)
			{
				if(fscanf(f, "%lf", &val) == 1)
				{
					mat2(a / 4, a % 4) = val;
				}
				else
				{
					std::cerr << "Error loading from " << fileBaseToLidar << std::endl;
					break;
				}
			}
			// mat 2
		}
		else
		{
			std::cerr << "Error opening " << fileBaseToLidar << std::endl;
		}
		fclose(f);

		base = C3DPose();
		baseToLidarBase = C3DPose(mat0, true);
		lidarBaseToLidar0 = C3DPose(mat1, true);
		lidarBaseToLidar1 = C3DPose(mat2, true);

		tfMap[tfbase] = base.r_Pc2w();
		tfMap[tflidarBase] = base.r_Pc2w() * baseToLidarBase.r_Pc2w();
		tfMap[tflidar0] = base.r_Pc2w() * baseToLidarBase.r_Pc2w() * lidarBaseToLidar0.r_Pc2w();
		tfMap[tflidar1] = base.r_Pc2w() * baseToLidarBase.r_Pc2w() * lidarBaseToLidar1.r_Pc2w();
	}

	MobileBase(const MobileBase &r_other)
		:base(r_other.base),
		 baseToLidarBase(r_other.baseToLidarBase),
		 lidarBaseToLidar0(r_other.lidarBaseToLidar0),
		 lidarBaseToLidar1(r_other.lidarBaseToLidar1),
		 tfMap(r_other.tfMap)
	{}

	MobileBase &operator =(const MobileBase &r_other)
	{
		base = r_other.base;
		baseToLidarBase = r_other.baseToLidarBase;
		lidarBaseToLidar0 = r_other.lidarBaseToLidar0;
		lidarBaseToLidar1 = r_other.lidarBaseToLidar1;
		tfMap = r_other.tfMap;
		return *this;
	}

	void Set_RtC2W(TF node, Matrix<double, 4, 4, DontAlign> Rt)
	{
		tfMap[node] = Rt;
	}
	void Set_AaC2W(TF node, Matrix<double, 6, 1> Aa)
	{
		tfMap[node] = C3DPose::AaToRt(Aa);
	}
	Matrix<double, 4, 4, DontAlign> Rt_TransformC2W(TF node)
	{
		return tfMap[node];	// c2w
	}
	Matrix<double, 4, 4, DontAlign> Rt_TransformW2C(TF node)
	{
		return tfMap[node].inverse();
	}
	Matrix<double, 4, 4, DontAlign> Rt_TransformC2W(TF node, Matrix<double, 4, 4, DontAlign> newBase)
	{
		//return newBase * tfMap[node];	// c2w
		return tfMap[node].inverse() * newBase;
	}
	Matrix<double, 4, 4, DontAlign> Rt_TransformC2W(TF from, TF to)
	{
		return tfMap[from].inverse() * tfMap[to];	// c2w
	}
	Matrix<double, 4, 4, DontAlign> Rt_TransformW2C(TF from, TF to)
	{
		return (tfMap[from].inverse() * tfMap[to]).inverse();	// c2w
	}
	VectorXd TransformPoint(Eigen::VectorXd point, TF from, TF to)
	{
		// we assume vec is 3d or 4d
		Vector4d pt = Vector4d::Ones();
		pt.head(3) = point.head(3);

		pt = Rt_TransformW2C(from, to) * pt;	// pt in coordinates of FROM to coordinates of TO

		if(point.rows() == 4)
			return pt;
		else
			return pt.head(3);
	}
	Vector3d TransformOrientation(Eigen::Vector3d point, TF from, TF to)
	{
		// we assume vec is 3d or 4d
		Vector4d pt = Vector4d::Ones();
		pt.head(3) = point.head(3);

		pt.head(3) = Rt_TransformW2C(from, to).block(0, 0, 3, 3) * pt.head(3);	// pt in coordinates of FROM to coordinates of TO

		return pt.head(3);
	}
	Eigen::Matrix<double, 7, 1, DontAlign> TransformMeasurement(Eigen::Matrix<double, 7, 1, DontAlign> measurement, TF from, TF to)
	{
		Eigen::Matrix<double, 7, 1, DontAlign> res = measurement;

		// we assume vec is 3d or 4d
		Vector4d pt = Vector4d::Ones();
		pt.head(3) = measurement.head(3);
		pt = Rt_TransformW2C(from, to) * pt;	// pt in coordinates of FROM to coordinates of TO
		res.head(3) = pt.head(3);

		pt.head(3) = measurement.segment(3, 3);
		pt.head(3) = Rt_TransformW2C(from, to).block(0, 0, 3, 3) * pt.head(3);	// pt in coordinates of FROM to coordinates of TO
		res.segment(3, 3) = pt.head(3);

		return res;
	}
	Matrix<double, 6, 1> Aa_TransformC2W(TF node)
	{
		return C3DPose::RtToAa(tfMap[node]);	// c2w
	}
	Matrix<double, 6, 1> Aa_TransformW2C(TF node)
	{
		return C3DPose::RtToAa(tfMap[node].inverse());
	}
	Matrix<double, 6, 1> Aa_TransformC2W(TF node, Matrix<double, 4, 4, DontAlign> newBase)
	{
		return C3DPose::RtToAa(Rt_TransformC2W(node, newBase));	// c2w
	}
	Matrix<double, 6, 1> Aa_TransformC2W(TF from, TF to)
	{
		return C3DPose::RtToAa(tfMap[from].inverse() * tfMap[to]);
	}
	Matrix<double, 6, 1> Aa_TransformW2C(TF from, TF to)
	{
		return C3DPose::RtToAa((tfMap[from].inverse() * tfMap[to]).inverse());
	}
};

struct IMUData
{
	std::vector<std::pair<double, C3DPose> > data;
	double min = -1;
	double max = -1;

	IMUData(std::string file)
	{
		std::ifstream read;
		read.open(file.c_str());
		std::string line;
		for (std::string line; std::getline(read, line); ) {
			if(line[0] == '{') {
				if(line[line.length()-2] == ',')
					line[line.length()-2] = ' ';
				if(line[line.length()-1] == ']')
					line[line.length()-1] = ' ';

				Document d;
				d.Parse(line.c_str());

				Quaterniond quat(d["quaternion_full"].GetArray()[3].GetDouble(),
								 d["quaternion_full"].GetArray()[0].GetDouble(),
								 d["quaternion_full"].GetArray()[1].GetDouble(),
								 d["quaternion_full"].GetArray()[2].GetDouble());
				// get quaternion

				Matrix3d R = quat.toRotationMatrix();
				Vector3d t0 = R * Vector3d(0, 0, 1);
				// get rotation matrix

				Matrix3d flip;
				flip << 0, 0, 1,
						1, 0, 0,
						0, 1, 0;
				R = flip.transpose() * R * flip;
				// swap axis

				Vector3d t1 = R * Vector3d(0, 1, 0);

				Quaterniond quat2(R);
				/*std::cout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " | ";
				std::cout << quat2.x() << " " << quat2.y() << " " << quat2.z() << " " << quat2.w() << std::endl;*/
				//std::cout << t0.transpose() << " | " << t1.transpose() << std::endl;
				Vector3d aa;
				aa = C3DJacobians::Operator_arot(R);
				//C3DJacobians::Quat_to_AxisAngle(quat2, aa);
				Matrix<double, 6, 1> pos;
				pos << 0, 0, 0, aa;
				data.push_back(std::pair<double, C3DPose>(d["timeStamp"].GetDouble(), C3DPose(pos)));

				if(min < 0)
					min = data.back().first;
			}
		}
		// read line

		max = data.back().first;
	}

	void PruneData(double timestampMin, double timestampMax)
	{
		int a = 0;
		for(; a < data.size() - 1; ++a)
		{
			if(data[a].first <= timestampMin && data[a + 1].first > timestampMin)
			{
				if(fabs(data[a].first - timestampMin) > fabs(data[a + 1].first - timestampMin))
					a = a + 1;
				// decide which node is closer

				break;
			}
		}
		// find closest start
		data.erase(data.begin(), data.begin() + a);
		// remove front

		a = 0;
		for(; a < data.size() - 1; ++a)
		{
			if(data[a].first <= timestampMax && data[a + 1].first > timestampMax)
			{
				if(fabs(data[a].first - timestampMax) > fabs(data[a + 1].first - timestampMax))
					a = a + 1;
				// decide which node is closer

				break;
			}
		}
		// find closest back
		data.erase(data.begin() + a + 1, data.end());
		// remove tail
	}

	C3DPose DataAt(double timestamp)
	{
		double rel = (timestamp - min) / (max - min);
		int idx = (data.size() * rel) >= 0 ? (data.size() * rel) : 0;

		return data[idx].second;
	}

	Eigen::Matrix4d TransformBetween(double t0, double t1)
	{
		double rel0 = (t0 - min) / (max - min);
		int idx0 = (data.size() * rel0) >= 0 ? (data.size() * rel0) : 0;

		double rel1 = (t1 - min) / (max - min);
		int idx1 = (data.size() * rel1) >= 0 ? (data.size() * rel1) : 0;

		Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
		for(int a = idx0; a <= idx1; ++a)
		{
			mat = mat * data[a].second.r_Pc2w();
		}

		return mat;
	}
};

struct TimestampData
{
	std::vector<double> data;

	TimestampData(std::string file)
	{
		FILE *f = fopen(file.c_str(), "r");
		if(f != NULL)
		{
			double val;
			while(true)
			{
				if(fscanf(f, "%lf", &val) == 1)
				{
					data.push_back(val);
					//std::cout << data.back() << std::endl;
				}
				else
				{
					//std::cerr << "Error loading from " << file << std::endl;
					break;
				}
			}
		}
		else
		{
			std::cerr << "Error opening " << file << std::endl;
		}
		fclose(f);
	}
};

/**
 *	@brief structure holding shared data for a dataset
 */
struct CInputData
{
	//std::string dataPath;
	std::string outputPath;
	std::string linesPath;
	std::string matchesPath;
	std::string matchesInternalPath;
	std::string lidarPath;

	MobileBase tf;
	IMUData IMU;
	TimestampData timestamp;

	/**
	 *	@brief constructor, load paths and data
	 *	@param fIMU file containing calibration IMU<->VeloCenter
	 *	@param fLidar file containing calibration VeloCenter<->Lidars
	 *	@param fIMUData file containing IMU data
	 *	@param fTimestamps file containing frame timestamps
	 *	@param linesPath path to line clouds
	 *	@param matchesPath path to frame matches
	 *	@param matchesInternalPath path to inter frame matches
	 *	@param lidarPath path to lidar clouds
	 *	@param outputPath path to output
	 */
	CInputData(const char *fIMU, const char *fLidar, const char *fIMUData, const char *fTimestamps, const char *pLines,
			const char *pMatches, const char *pInterMatches, const char *pLidar, const char *pOutput):
		tf(std::string(fIMU), std::string(fLidar)),
		IMU(std::string(fIMUData)),
		timestamp(std::string(fTimestamps)),
		linesPath(std::string(pLines)),
		matchesPath(std::string(pMatches)),
		matchesInternalPath(std::string(pInterMatches)),
		lidarPath(std::string(pLidar)),
		outputPath(std::string(pOutput))
	{
		IMU.PruneData(timestamp.data.front(), timestamp.data.back());
	}
	CInputData(const CInputData &r_other):
			tf(r_other.tf),
			IMU(r_other.IMU),
			timestamp(r_other.timestamp),
			linesPath(r_other.linesPath),
			matchesPath(r_other.matchesPath),
			matchesInternalPath(r_other.matchesInternalPath),
			lidarPath(r_other.lidarPath),
			outputPath(r_other.outputPath)
	{}
	CInputData &operator =(const CInputData &r_other)
	{
		tf = r_other.tf;
		IMU = r_other.IMU;
		timestamp = r_other.timestamp;
		linesPath = r_other.linesPath;
		matchesPath = r_other.matchesPath;
		matchesInternalPath = r_other.matchesInternalPath;
		lidarPath = r_other.lidarPath;
		outputPath = r_other.outputPath;

		return *this;
	}
};

class CMeasurement2
{
public:
	int lidarId;
	double phase;
	Vector3d position;
	Vector3d orientation;
	Vector3d normal;
	weak_ptr<CStructure> p_structure;	// will be assigned eventually
	weak_ptr<CTimeFrame> p_owner;

	CMeasurement2():
		lidarId(0),
		phase(0),
		position(Vector3d::Zero()),
		orientation(Vector3d::Ones()),
		normal(Vector3d(0, 0, 1)),
		p_structure(weak_ptr<CStructure>())
	{}

	CMeasurement2(int id, double d_phase, Vector3d v_position, Vector3d v_orientation, Vector3d v_normal):
		lidarId(id),
		phase(d_phase),
		position(v_position),
		orientation(v_orientation),
		normal(v_normal),
		p_structure(weak_ptr<CStructure>())
	{}

	CMeasurement2(const CMeasurement2 &r_other):
		lidarId(r_other.lidarId),
		phase(r_other.phase),
		position(r_other.position),
		orientation(r_other.orientation),
		p_structure(r_other.p_structure),
		normal(r_other.normal),
		p_owner(r_other.p_owner)
	{}
	CMeasurement2 &operator =(const CMeasurement2 &r_other)
	{
		lidarId = r_other.lidarId;
		phase = r_other.phase;
		position = r_other.position;
		orientation = r_other.orientation;
		normal = r_other.normal;
		p_structure = r_other.p_structure;
		p_owner = r_other.p_owner;

		return *this;
	}

	int n_LidarId()
	{
		return lidarId;
	}

	void SetStructurePtr(shared_ptr<CStructure> ptr)
	{
		p_structure = ptr;
	}
	shared_ptr<CStructure> p_Structure()
	{
		shared_ptr<CStructure> sp = p_structure.lock();
		return sp;
	}
	shared_ptr<CTimeFrame> p_Owner()
	{
		shared_ptr<CTimeFrame> sp = p_owner.lock();
		return sp;
	}
	void SetOwner(shared_ptr<CTimeFrame> p_frame)
	{
		p_owner = p_frame;
	}

	Matrix<double, 7, 1, DontAlign> v_Measurement()
	{
		Eigen::Matrix<double, 7, 1> res;

		res.head(3) = position;
		// pos 0

		res.segment(3, 3) = orientation;
		// ori 0

		res(6) = phase;

		return res;
	}

	friend ostream &operator<<( ostream &output, const CMeasurement2 &m ) {
		output << m.lidarId << " " << m.phase << " p: " << m.position.transpose() << " o: " <<
				m.orientation.transpose() << " n: " << m.normal.transpose();
		return output;
	}
};

class CLidar {
private:
	int lidarId;
	Matrix<double, 4, 1, DontAlign> polynome;
	std::vector<shared_ptr<CMeasurement2> > v_measurements;
	C3DPose pose;	// pose of the base node
	int systemId;
public:
	CLidar(int id):
		lidarId(id),
		polynome(Vector4d(1, 0, 0, 0)),
		systemId(-1)
	{}
	CLidar(const CLidar &r_other):
		lidarId(r_other.lidarId),
		polynome(r_other.polynome),
		v_measurements(r_other.v_measurements),
		pose(r_other.pose),
		systemId(r_other.systemId)
	{}
	CLidar &operator =(const CLidar &r_other)
	{
		lidarId = r_other.lidarId;
		polynome = r_other.polynome;
		v_measurements = r_other.v_measurements;
		pose = r_other.pose;
		systemId = r_other.systemId;

		return *this;
	}

	Matrix<double, 4, 1, DontAlign> & r_Polynome()
	{
		return polynome;
	}

	std::vector<shared_ptr<CMeasurement2> > & r_Measurements()
	{
		return v_measurements;
	}

	int n_LidarId()
	{
		return lidarId;
	}
	void SetSystemId(int id)
	{
		systemId = id;
	}
	int n_SystemId()
	{
		return systemId;
	}

	C3DPose & r_Pose()
	{
		return pose;
	}
};

class CStructure {
private:
	Eigen::Matrix<double, 4, 1, Eigen::DontAlign> m_plane; // local 3D plane
	weak_ptr<CTimeFrame> p_owner;
	int lidarId;
	int systemId;
	std::vector<weak_ptr<CMeasurement2> > m_referees;

	Vector3d v_originalPoint;
	Vector3d v_originalNormal;
public:
	CStructure():
		m_plane(Eigen::Matrix<double, 4, 1, Eigen::DontAlign>::Ones()),
		lidarId(0),
		systemId(-1)
	{}
	CStructure(int lId, Eigen::Matrix<double, 4, 1, Eigen::DontAlign> plane):
		m_plane(plane),
		lidarId(lId),
		systemId(-1)
	{}
	CStructure(const CStructure &r_other):
		m_plane(r_other.m_plane),
		p_owner(r_other.p_owner),
		lidarId(r_other.lidarId),
		systemId(r_other.systemId),
		m_referees(r_other.m_referees),
		v_originalPoint(r_other.v_originalPoint),
		v_originalNormal(r_other.v_originalNormal)
	{}
	CStructure &operator =(const CStructure &r_other)
	{
		m_plane = r_other.m_plane;
		p_owner = r_other.p_owner;
		lidarId = r_other.lidarId;
		systemId = r_other.systemId;
		m_referees = r_other.m_referees;
		v_originalPoint = r_other.v_originalPoint;
		v_originalNormal = r_other.v_originalNormal;

		return *this;
	}

	int n_LidarId()
	{
		return lidarId;
	}
	int n_SystemId()
	{
		return systemId;
	}
	void SetSystemId(int id)
	{
		systemId = id;
	}

	Eigen::Matrix<double, 4, 1, Eigen::DontAlign> & r_Plane()
	{
		return m_plane;
	}

	std::vector<weak_ptr<CMeasurement2> >& r_Referees() {
		return m_referees;
	}

	shared_ptr<CTimeFrame> p_Owner()
	{
		shared_ptr<CTimeFrame> sp = p_owner.lock();
		return sp;
	}

	void SetOwner(shared_ptr<CTimeFrame> p_frame)
	{
		p_owner = p_frame;
	}

	void AddReferee(shared_ptr<CMeasurement2> p_meas)
	{
		bool found = false;
		for(size_t i = 0, n = m_referees.size(); i < n; ++i)
		{
			shared_ptr<CMeasurement2> sp = m_referees[i].lock();

			if(sp != nullptr && sp == p_meas)
			{
				found = true;
				break;
			}
		}
		// check for duplicity

		if(!found)
			m_referees.push_back(p_meas);
	}

	shared_ptr<CMeasurement2> r_RefereeAt(int idx) {
		shared_ptr<CMeasurement2> sp = m_referees[idx].lock();
		return sp;
	}
};

class CTimeFrame {
private:
	C3DPose pose;	// pose of the base node
	CLidar lidar0;
	CLidar lidar1;
	int systemId;
	int id;

	std::vector<std::pair<int, int> > v_measurementRefs;
	std::vector<shared_ptr<CMeasurement2> > v_measurements;
	Eigen::Matrix<double, 24, 1, Eigen::DontAlign> poly;
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> bezier;
	int divider;

	double frameStart;
	double frameEnd;
public:
	CTimeFrame(int nid, double start, double end, C3DPose pos = C3DPose()):
		pose(pos),
		frameStart(start),
		frameEnd(end),
		lidar0(CLidar(0)),
		lidar1(CLidar(1)),
		systemId(-1),
		divider(0),
		id(nid)
	{
		poly << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0;
		bezier << 0, 0, 0, 0, 0, 0;
	}
	CTimeFrame(const CTimeFrame &r_other):
		id(r_other.id),
		lidar0(r_other.lidar0),
		lidar1(r_other.lidar1),
		frameStart(r_other.frameStart),
		pose(r_other.pose),
		systemId(r_other.systemId),
		frameEnd(r_other.frameEnd),
		v_measurements(r_other.v_measurements),
		v_measurementRefs(r_other.v_measurementRefs),
		divider(r_other.divider),
		poly(r_other.poly),
		bezier(r_other.bezier)
	{}
	CTimeFrame &operator =(const CTimeFrame &r_other)
	{
		id = r_other.id;
		pose = r_other.pose;
		lidar0 = r_other.lidar0;
		lidar1 = r_other.lidar1;
		frameStart = r_other.frameStart;
		frameEnd = r_other.frameEnd;
		systemId = r_other.systemId;
		v_measurements = r_other.v_measurements;
		v_measurementRefs = r_other.v_measurementRefs;
		divider = r_other.divider;
		poly = r_other.poly;
		bezier = r_other.bezier;

		return *this;
	}

	C3DPose & r_Pose()
	{
		return pose;
	}

	int n_Id()
	{
		return id;
	}
	int n_SystemId()
	{
		return systemId;
	}
	void SetSystemId(int id)
	{
		systemId = id;
	}
	double d_FrameStart()
	{
		return frameStart;
	}
	double d_FrameEnd()
	{
		return frameEnd;
	}
	int n_Divider()
	{
		return divider;
	}
	void SetDivider(int n)
	{
		divider = n;
	}

	CLidar & Lidar0()
	{
		return lidar0;
	}

	CLidar & Lidar1()
	{
		return lidar1;
	}

	CLidar & Lidar(int id)
	{
		if(id == 0)
			return Lidar0();
		else
			return Lidar1();
	}

	std::vector<std::pair<int, int> > & r_MeasurementRefs()
	{
		return v_measurementRefs;
	}
	std::vector<shared_ptr<CMeasurement2> > & r_Measurements()
	{
		return v_measurements;
	}

	Eigen::Matrix<double, 24, 1, Eigen::DontAlign> & r_Poly()
	{
		return poly;
	}

	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> & r_Bezier()
	{
		return bezier;
	}
};

class CStructureManager
{
private:
	CInputData data;
	std::vector<shared_ptr<CTimeFrame> > v_frames;
	std::vector<shared_ptr<CStructure> > v_structure;
public:
	CStructureManager(CInputData inputData):
		data(inputData)
	{}
	CStructureManager(const CStructureManager &r_other):
		data(r_other.data),
		v_frames(r_other.v_frames),
		v_structure(r_other.v_structure)
	{}
	CStructureManager &operator =(const CStructureManager &r_other)
	{
		data = r_other.data;
		v_frames = r_other.v_frames;
		v_structure = r_other.v_structure;

		return *this;
	}

	void AddFrame(int id, double subId)
	{
		double start = id < data.timestamp.data.size() ? data.timestamp.data[id] : data.timestamp.data.back();
		double end = id + 1 < data.timestamp.data.size() ? data.timestamp.data[id + 1] : data.timestamp.data.back();
		double at = start + (end - start) * subId;
		//std::cout << id << " " << start << " " << end << " " << at << std::endl;
		//double start = v_frames.size() < timestamp.data.size() ? timestamp.data[v_frames.size()] : timestamp.data.back();
		//double end = v_frames.size() + 1 < timestamp.data.size() ? timestamp.data[v_frames.size() + 1] : timestamp.data.back();
		shared_ptr<CTimeFrame> frame(new CTimeFrame(v_frames.size(), at, at/*end*/));
		//CTimeFrame frame(timestamp.data[v_frames.size()],
		//				 timestamp.data[v_frames.size() + 1]/*,
		//				 IMU.DataAt(timestamp.data[v_frames.size()])*/);
		// create frame

		v_frames.push_back(frame);
	}

	Eigen::Matrix4d GetFrameRelativeRotation(int id)
	{
		if(id >= v_frames.size() - 1)
			return Eigen::Matrix4d::Identity();

		//Eigen::Matrix4d rel = IMU.TransformBetween(v_frames[id]->d_FrameStart(), v_frames[id + 1]->d_FrameStart());

		C3DPose p0 = data.IMU.DataAt(v_frames[id]->d_FrameStart());
		C3DPose p1 = data.IMU.DataAt(v_frames[id + 1]->d_FrameStart());

		Eigen::Matrix4d rel = p0.r_Pw2c() * p1.r_Pc2w();



		return rel;
	}
	Eigen::Matrix4d GetFrameAbsoluteRotation(int id) // !!!! id of timestamp, not frame
	{
		double time = 0;
		if(id < data.timestamp.data.size())
			time = data.timestamp.data[id];
		else
			time = data.timestamp.data.back();

		C3DPose p0 = data.IMU.DataAt(time);

		Eigen::Matrix4d rel = p0.r_Pc2w();

		//std::cout << time << " " << p0.v_Representation_6D().transpose() << std::endl;

		return rel;
	}
	Eigen::Matrix4d GetFrameAbsoluteRotationByTime(double time) // !!!! id of timestamp, not frame
	{
		C3DPose p0 = data.IMU.DataAt(time);

		Eigen::Matrix4d rel = p0.r_Pc2w();

		//std::cout << time << " " << p0.v_Representation_6D().transpose() << std::endl;

		return rel;
	}

	MobileBase & r_Tf()
	{
		return data.tf;
	}

	IMUData & r_IMU()
	{
		return data.IMU;
	}

	TimestampData & r_Timestamps()
	{
		return data.timestamp;
	}

	CInputData & r_Data()
	{
		return data;
	}

	std::vector<shared_ptr<CTimeFrame> > & r_v_Frames()
	{
		return v_frames;
	}

	std::vector<shared_ptr<CStructure> > & r_v_Structure()
	{
		return v_structure;
	}
};

class CScanCorrector
{
private:
	CStructureManager manager;
	bool b_verbose;
public:
	CScanCorrector(CInputData &inputData, bool verbose):
		manager(inputData),
		b_verbose(verbose)
	{}

	Eigen::Matrix4d StolenComputeTransformationWeighted(
	    const MatrixXd &source_coresp_points,
	    const MatrixXd &target_coresp_points)
	{
		MatrixXd weights(source_coresp_points.cols(), source_coresp_points.cols());
		weights.setIdentity();
		weights = weights / source_coresp_points.cols();

		// Lets compute the translation
		// Define Column vector using definition of TPoint3D
		Vector3d centroid_0;
		MatrixXd target_points_weighted = target_coresp_points * weights;
		centroid_0 << target_points_weighted.row(0).sum(),
		  target_points_weighted.row(1).sum(), target_points_weighted.row(2).sum();

		Vector3d centroid_1;
		MatrixXd source_points_weighted = source_coresp_points * weights;
		centroid_1 << source_points_weighted.row(0).sum(),
		  source_points_weighted.row(1).sum(), source_points_weighted.row(2).sum();

		Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

		Eigen::Matrix<double, 1, Eigen::Dynamic> identity_vec =
					 Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(1, target_coresp_points.cols()); //setOnes();

		// Create matrix with repeating values in columns
		MatrixXd translate_0_mat = centroid_0 * identity_vec;
		MatrixXd translate_1_mat = centroid_1 * identity_vec;

		// Translation of source_coresp_points to the target_coresp_points (Remember this is opposite of camera movement)
		// ie if camera is moving forward, the translation of target_coresp_points to source_coresp_points is opposite
		// TPoint3D t = (centroid_1 - centroid_0);

		// Translate the point cloud 0 to the coordinates of point cloud 1
		MatrixXd target_coresp_points_translated(target_coresp_points.rows(), target_coresp_points.cols());
		MatrixXd source_coresp_points_translated(source_coresp_points.rows(), source_coresp_points.cols());

		target_coresp_points_translated = target_coresp_points - translate_0_mat;
		source_coresp_points_translated = source_coresp_points - translate_1_mat;

		// Compute the Covariance matrix of these two pointclouds moved to the origin
		// This is not properly covariance matrix as there is missing the 1/N
		// 1/N is important for computing eigenvalues(scale), not the eigenvectors(directions) - as we are interested in eigenvectors

		Matrix3d A = target_coresp_points_translated * weights * weights * source_coresp_points_translated.transpose();

		// Compute the SVD upon A = USV^t
		Eigen::JacobiSVD<Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// Compute the determinant of V*U^t - to find out in what direction the rotation is
		float det = (svd.matrixV() * svd.matrixU().transpose()).determinant();

		// Fix the right hand/left hand rotation : assuming we would like the right hand rotation
		Matrix3d E = Matrix3d::Identity();
		E(2, 2) = (det >= 0) ? 1.0f : -1.0f;

		// Compute the rotation as R = VEU^t
		// R is the rotation of point_0_translated to fit the source_coresp_points_translated
		Matrix3d R = svd.matrixV() * E * (svd.matrixU().transpose());

		transformation.block(0, 0, 3, 3) = R;
		// The translation must be computed as centroid_1 - rotated centroid_0
		transformation.block(0, 3, 3, 1) = centroid_1 - (R * centroid_0);

		return transformation;
	}

	void LoadPCL(std::string infile, pcl::PointCloud<velodyne_pointcloud::PointXYZIRP> & cloud, Eigen::Matrix4d transform)
	{
		pcl::PointCloud<velodyne_pointcloud::PointXYZIRP> tmp;
		if (pcl::io::loadPCDFile<velodyne_pointcloud::PointXYZIRP> (infile.c_str(), tmp) == -1) //* load the file
		{
			std::cout << "Couldn't read file " << infile << std::endl;
		}
		// load pcl

		pcl::transformPointCloud (tmp, cloud, transform);
		// transform pcl
	}

	void SavePCL(std::string file, pcl::PointCloud<velodyne_pointcloud::PointXYZIRP> & cloud)
	{
		pcl::PointCloud<velodyne_pointcloud::PointXYZIRP> tmp;
		pcl::io::savePCDFile(file, cloud, true);
		// save pcl
	}

	std::vector<CMeasurement2> LoadMeasurements(int id, int lidarId, std::string path, Eigen::Matrix4d transform,
			std::string file = std::string(""))
	{
		char num[10];
		sprintf(num, "%06d", id);

		if(file.length() == 0)
			path = path + std::string(num) + std::string(".") + std::to_string(lidarId) +
					std::string(".pcd.lcd");
		else
			path = file;
		// create file name

		if(b_verbose)
			std::cout << "Loading " << path << std::endl;

		std::vector<CMeasurement2> data;
		FILE *f = fopen(path.c_str(), "r");
		if(f != NULL)
		{
			int idLidar, idFrame;
			double val[16];
			while(true)
			{
				if(fscanf(f, "%lf %lf %lf "
							 "%lf %lf %lf "
							 "%lf %lf %lf "
							 "%d "
							 "%lf %lf %lf "
							 "%lf "
							 "%lf "
							 "%d",
						val, val+1, val+2, val+3, val+4, val+5, val+6, val+7, val+8,
						&idLidar, val+9, val+10, val+11, val+12, val+13, &idFrame) == 16)
				{
					CMeasurement2 m(idLidar, val[12], Vector3d(val[0], val[1], val[2]), Vector3d(val[3], val[4], val[5]),
							Vector3d(val[9], val[10], val[11]));

					//std::cout << m << std::endl;

					data.push_back(m);
				}
				else
				{
					//std::cerr << "Error loading from " << file << std::endl;
					break;
				}
			}

			fclose(f);
		}
		// try loading

		return data;
	}

	bool LoadMatches(int id, int hist, std::string path, std::string path_inter, std::vector<std::pair<int, int> > &data, size_t type = 0,
			std::string file = std::string(""))	// 0 - temporal, 1 - internal
	{
		if(type == 0)
		{
			path = path + std::to_string(hist) +
					std::string("-") + std::to_string(id) + std::string(".txt");
		} else
		{
			char num[10];
			sprintf(num, "%06d", id);
			path = path_inter + std::string(num) + std::string(".matches");
		}
		if(file.length() != 0)
			path = file;
		// create file name

		FILE *f = fopen(path.c_str(), "r");
		if(f != NULL)
		{
			if(b_verbose)
				std::cout << "Loading " << path << std::endl;
			int id0, id1;
			while(true)
			{
				if(fscanf(f, "%d %d", &id0,	&id1) == 2)
				{
					data.push_back(std::pair<int, int>(id0, id1));
					//std::cout << id0 << " " << id1 << std::endl;
				}
				else
				{
					//std::cerr << "Error loading from " << file << std::endl;
					break;
				}
			}

			fclose(f);
			return true;
		}
		// try loading
		//std::cout << data.size() << std::endl;

		return false;
	}

	void RegisterHistory(CStructureManager &manager, CTimeFrame &act, int frameId, int start, int iFrames)
	{
		int refs = 0;
		for(int a = frameId-2; a >= start; --a)
		{
			std::vector<std::pair<int, int> > matches;
			bool found = LoadMatches(frameId, a, manager.r_Data().matchesPath, manager.r_Data().matchesInternalPath, matches, 0);
			// load inter matches
			//std::cout << "ld? " << found << std::endl;

			int base0 = a - start;
			//int base1 = frameId - start;
			shared_ptr<CTimeFrame> f0 = manager.r_v_Frames()[base0 * iFrames];
			//shared_ptr<CTimeFrame> f1 = manager.r_v_Frames()[base1 * iFrames];
			// get frames
			for(int i = 0; i < matches.size(); ++i)
			{
				std::pair<int, int> ids = f0->r_MeasurementRefs()[matches[i].first];
				shared_ptr<CMeasurement2> m0 = manager.r_v_Frames()[ids.first]->r_Measurements()[ids.second];
				shared_ptr<CMeasurement2> m1 = act.r_Measurements()[matches[i].second];
				// make references

				if(m0->p_Structure() != nullptr && m1->p_Structure() == nullptr)
				{
					m0->p_Structure()->AddReferee(m1);
					m1->SetStructurePtr(m0->p_Structure());
					refs ++;
				}
				// m0 is structure
				else if(m0->p_Structure() == nullptr && m1->p_Structure() != nullptr)
				{
					m1->p_Structure()->AddReferee(m0);
					m0->SetStructurePtr(m1->p_Structure());
					refs ++;
				}
			}

			if(!found)
				break;
		}
		//std::cout << "hist: " << refs << std::endl;
	}

	inline double lerp(const double a, const double b, const double t)
	{
		return a + t * (b - a);
	}

	/**
	 *	@brief optimizes range of data
	 *	@param from id of starting frame (included)
	 *	@param to id of ending frame (included)
	 *	@param iFrames number of simulated internal frames
	 *	@param exportFrom starting id of exported frames (included)
	 *	@param exportTo ending id of exported frames (included)
	 *	@param n_iterations number of solver iterations (default 15)
	 *	@param f_threshold solver residual threshold (default 1e-3)
	 */
	void CorrectRange(int from, int to, int iFrames, int exportFrom, int exportTo,
					  int n_iterations = 15, double f_threshold = 1e-3) {
		int frameId = from;
		int historyId = frameId;

		bool mainloop = true;

		std::vector<CTimeFrame> fms;
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> residualTransform = Eigen::Matrix4d::Identity();

		while(mainloop)
		{
			if(fms.size() > 1)
				fms.erase(fms.begin() + 0);
			// keep only 2 frames

			vector<CMeasurement2> data0 = LoadMeasurements(frameId, 1, manager.r_Data().linesPath, Eigen::Matrix4d::Identity());
			vector<CMeasurement2> data1 = LoadMeasurements(frameId, 2, manager.r_Data().linesPath, Eigen::Matrix4d::Identity());

			CTimeFrame frame(0, 0, 0);

			for(int a = 0; a < data0.size(); ++a)
			{
				shared_ptr<CMeasurement2> m(new CMeasurement2(data0[a]));
				frame.r_Measurements().push_back(m);
			}
			for(int a = 0; a < data1.size(); ++a)
			{
				shared_ptr<CMeasurement2> m(new CMeasurement2(data1[a]));
				frame.r_Measurements().push_back(m);
			}

			fms.push_back(frame);

			if(data0.size() == 0 || data1.size() == 0)
				break;

			if(fms.size() > 1)
			{

				bool matches_added = false;

				std::vector<std::pair<int, int> > matches;
				bool loaded = LoadMatches(frameId, frameId - 1, manager.r_Data().matchesPath,
										  manager.r_Data().matchesInternalPath, matches);

				if(!loaded)
				{
					frameId ++;
					continue;
				}

				/*std::vector<bool> duplicities;
				for(int i = 0; i < f0->r_Measurements().size(); ++i)
					duplicities.push_back(false);*/

				Eigen::MatrixXd ptsLeft(3, matches.size());
				Eigen::MatrixXd ptsRight(3, matches.size());

				for(int b = 0; b < matches.size(); ++b)
				{
					Vector3d p0 = fms[0].r_Measurements()[matches[b].first]->position;
					p0 = manager.r_Tf().TransformPoint(p0, fms[0].r_Measurements()[matches[b].first]->lidarId == 0 ? tflidar0 : tflidar1,
							tfbase);
					// point to baseframe
					ptsLeft.col(b) = p0;

					Vector3d p1 = fms[1].r_Measurements()[matches[b].second]->position;
					p1 = manager.r_Tf().TransformPoint(p1, fms[1].r_Measurements()[matches[b].second]->lidarId == 0 ? tflidar0 : tflidar1,
							tfbase);
					// point to baseframe
					ptsRight.col(b) = p1;
				}
				Eigen::Matrix<double, 4, 4, Eigen::DontAlign> transform = StolenComputeTransformationWeighted(ptsLeft, ptsRight);
				if(b_verbose)
					std::cout << "Initial TF: \n" << transform << std::endl << std::endl;
				// estimate transform

				Eigen::Matrix<double, 4, 4, Eigen::DontAlign> fromRt = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>::Identity();
				if(manager.r_v_Frames().size() > 0)
					fromRt = manager.r_v_Frames().back()->r_Pose().r_Pc2w() * residualTransform;
				C3DPose fromPose; fromPose.setPc2w(fromRt);
				Eigen::Matrix<double, 4, 4, Eigen::DontAlign> toRt = fromRt * transform;
				C3DPose toPose; toPose.setPc2w(toRt);

				for(int a = 0; a < iFrames; ++a)
				{
					Eigen::Vector6d imm = Eigen::Vector6d::Zero();
					for(size_t i = 0; i < imm.rows(); ++i)
						imm(i) = lerp(fromPose.v_Representation_6D()(i), toPose.v_Representation_6D()(i), (1.0 / iFrames) * a);

					//std::cout << imm.transpose() << std::endl;
					manager.AddFrame(from + manager.r_v_Frames().size() / iFrames, (a % iFrames) / (double)iFrames);
					manager.r_v_Frames().back()->r_Pose().setRepresentation_6D(imm);
					// store frame
				}
				// add the frames into the system
				for(int a = 0; a < fms[0].r_Measurements().size(); ++a)
				{
					shared_ptr<CMeasurement2> m = fms[0].r_Measurements()[a];
					int id = manager.r_v_Frames().size() - iFrames + floor(m->phase / (1.0 / iFrames));
					if(id >= manager.r_v_Frames().size())
						id = manager.r_v_Frames().size() - 1;

					m->SetOwner(manager.r_v_Frames()[id]);
					manager.r_v_Frames()[id]->r_Measurements().push_back(m);
					// store measurement
					manager.r_v_Frames()[manager.r_v_Frames().size() - iFrames]->r_MeasurementRefs().push_back(
							std::pair<int, int>(id, manager.r_v_Frames()[id]->r_Measurements().size()-1));
					// store its position
				}
				// store corresponding measurements

				residualTransform = manager.r_v_Frames().back()->r_Pose().r_Pw2c() * toRt;
				//std::cout << "res >> " << residualTransform << std::endl;
				// save residual transform

				int refs = 0;
				int news = 0;
				for(int b = 0; b < matches.size(); ++b)
				{
					shared_ptr<CMeasurement2> m0 = fms[0].r_Measurements()[matches[b].first];
					shared_ptr<CMeasurement2> m1 = fms[1].r_Measurements()[matches[b].second];
					// make references

					/*if(duplicities[matches[b].first])
						continue;
					duplicities[matches[b].first] = true;*/

					if(m0->p_Structure() != nullptr && m1->p_Structure() == nullptr)
					{
						m0->p_Structure()->AddReferee(m1);
						m1->SetStructurePtr(m0->p_Structure());
						refs ++;
					}
					// m0 is structure
					else if(m0->p_Structure() == nullptr && m1->p_Structure() != nullptr)
					{
						m1->p_Structure()->AddReferee(m0);
						m0->SetStructurePtr(m1->p_Structure());
						refs ++;
					}
					if(m0->p_Structure() == nullptr && m1->p_Structure() == nullptr)
					{
						Vector4d p0 = Vector4d::Ones();
						p0.head(3) = m0->position;
						p0 = manager.r_Tf().TransformPoint(p0, m0->lidarId == 0 ? tflidar0 : tflidar1, tfbase);
						// point to baseframe

						Vector3d o0 = m0->orientation;
						o0 = manager.r_Tf().TransformOrientation(o0, m0->lidarId == 0 ? tflidar0 : tflidar1, tfbase);
						o0 = fromRt.block(0, 0, 3, 3) * o0;
						Vector3d o1 = m1->orientation;
						o1 = manager.r_Tf().TransformOrientation(o1, m1->lidarId == 0 ? tflidar0 : tflidar1, tfbase);
						o1 = toRt.block(0, 0, 3, 3) * o1;
						Vector3d n0 = o0.cross(o1);
						n0 = fromRt.inverse().block(0, 0, 3, 3) * n0;
						// compute normal from cross products of orientations

						Vector4d plane = Vector4d::Ones();
						plane.head(3) = n0.normalized();
						// normal vector
						plane(3) = - plane.head(3).transpose() * p0.head(3);
						// plane in in baseframe
						// compute and initialize plane

						std::shared_ptr<CStructure> p_str(new CStructure(2 /* baseframe */, plane));
						// create structure
						p_str->SetOwner(m0->p_Owner());
						// add structure referees
						manager.r_v_Structure().push_back(p_str);
						// add structure
						m0->SetStructurePtr(p_str);
						m1->SetStructurePtr(p_str);
						// add pointer to structure
						p_str->AddReferee(m0);
						p_str->AddReferee(m1);
						// set the links
						news ++;
					}
				}
				// build 3D structure

				std::vector<std::pair<int, int> > imatches;
				LoadMatches(frameId - 1, frameId - 1, manager.r_Data().matchesPath,
						    manager.r_Data().matchesInternalPath, imatches, 1);
				// load inter matches - for last frame

				for(int a = 0; a < imatches.size(); ++a)
				{
					shared_ptr<CMeasurement2> m0 = fms[0].r_Measurements()[imatches[a].first];
					shared_ptr<CMeasurement2> m1 = fms[0].r_Measurements()[imatches[a].second];

					if(m0->p_Structure() != nullptr)
					{
						m0->p_Structure()->AddReferee(m1);

						if(m1->p_Structure() == nullptr)
							m1->SetStructurePtr(m0->p_Structure());
					} else if(m1->p_Structure() != nullptr)
					{
						m1->p_Structure()->AddReferee(m0);

						if(m0->p_Structure() == nullptr)
							m0->SetStructurePtr(m1->p_Structure());
					}
				}
				// create inter frame references

				RegisterHistory(manager, fms[0], frameId-1, from, iFrames);
				// register history
			}

			frameId ++;
			if(frameId > to)
				break;
		}

		OptimizeOffset(manager, n_iterations, f_threshold);
		// optimize
		ExportData(manager, from, to, exportFrom, exportTo, iFrames);
		// export

		//std::cout << "done" << std::endl;
		// optimize here
	}

	float OptimizeOffset(CStructureManager &manager, int n_iterations = 15, double f_threshold = 1e-3)
	{
		typedef MakeTypelist_Safe((CVertexPose3D, CVertexPlane3D)) TVertexTypelist_SE3;
		typedef MakeTypelist_Safe((CEdgePose3D, CEdgeNormal23D, CPlaneOffsetEdge_Local, CPlaneOffsetEdge_Global)) TEdgeTypelist_SE3;
		typedef CFlatSystem<CBaseVertex, TVertexTypelist_SE3,
				CBaseEdge, TEdgeTypelist_SE3, CBasicUnaryFactorFactory,
				CBaseVertex, TVertexTypelist_SE3> CSystemType;
		typedef CLinearSolver_UberBlock<typename CSystemType::_TyHessianMatrixBlockList> CLinearSolverType; /**< @brief linear solver type */
		typedef CNonlinearSolver_Lambda_LM<CSystemType, CLinearSolverType> CBatchNonlinearSolverType;

		CSystemType m_system;
		CBatchNonlinearSolverType m_solver(m_system, TIncrementalSolveSetting(solve::nonlinear, frequency::Never(), 10, 0.01),
				TMarginalsComputationPolicy(marginals::do_not_calculate, frequency::Never()), false, CLinearSolverType(), true);
		// create optimizer

		//std::cout << manager.r_v_Frames().size() << std::endl;
		//omp_set_num_threads(1);
		std::vector<shared_ptr<CStructure> > v_p_structure;

		Matrix<double, 6, 6> info = Matrix<double, 6, 6>::Identity() * 1000;
		info.block(3, 3, 0, 0) = Matrix3d::Identity() * 1;

		Matrix<double, 6, 6> info0 = Matrix<double, 6, 6>::Identity() * 1;
		info0.block(3, 3, 0, 0) = Matrix3d::Identity() * 1;

		Eigen::Matrix<double, 1, 1> infoIMU = Eigen::Matrix<double, 1, 1>::Identity() * 10;

		m_system.r_Get_Vertex<CVertexPose3D>(0, Eigen::Matrix<double, 6, 1>::Zero());
		// base world vertex
		int systemId = 1;
		for(int a = 0; a < manager.r_v_Frames().size(); ++a)
		{
			//std::cout  << "POSE " << systemId << " " << manager.r_v_Frames()[a]->r_Pose().v_Representation_6D().transpose() << std::endl;
			m_system.r_Get_Vertex<CVertexPose3D>(systemId, manager.r_v_Frames()[a]->r_Pose().v_Representation_6D());
			manager.r_v_Frames()[a]->SetSystemId(systemId);
			systemId ++;
			// this is frame head

			if(a == 0) {
				m_system.r_Add_Edge(CEdgePose3D(0, 1, Eigen::Matrix<double, 6, 1>::Zero(), info0, m_system));
			}
			// set up connection to zero vertex

			Eigen::Matrix4d p = manager.GetFrameAbsoluteRotationByTime(manager.r_v_Frames()[a]->d_FrameStart());
			Eigen::Vector4d norm = p.inverse() * Eigen::Vector4d(0, 1, 0, 1);
			m_system.r_Add_Edge(CEdgeNormal23D(0, systemId - 1, norm.head(3), infoIMU, m_system));
			// get normal vector

			if(a > 0)
			{
				Eigen::Matrix4d rel = manager.r_v_Frames()[a - 1]->r_Pose().r_Pw2c() * manager.r_v_Frames()[a]->r_Pose().r_Pc2w();
				C3DPose relpos;
				relpos.setPc2w(rel);
				m_system.r_Add_Edge(CEdgePose3D(manager.r_v_Frames()[a - 1]->n_SystemId(), manager.r_v_Frames()[a]->n_SystemId(),
						relpos.v_Representation_6D(), info, m_system));
			}
			// add basic edges
		}
		// add poses

		m_system.r_Get_Vertex<CVertexPose3D>(-1, manager.r_Tf().Aa_TransformC2W(tflidar0));
		int l0Id = -1;//systemId;
		//systemId ++;
		m_system.r_Get_Vertex<CVertexPose3D>(-2, manager.r_Tf().Aa_TransformC2W(tflidar1));
		int l1Id = -2;//systemId;
		//systemId ++;
		// add calibration

		Matrix<double, 2, 2> info2 = Matrix<double, 2, 2>::Identity() * 10;
		info2(1, 1) = 1;

		for(int a = 0; a < manager.r_v_Structure().size(); ++a)
		{
			shared_ptr<CStructure> s = manager.r_v_Structure()[a];
			v_p_structure.push_back(s);

			m_system.r_Get_Vertex<CVertexPlane3D>(systemId, s->r_Plane());
			s->SetSystemId(systemId);

			m_system.r_Add_Edge(CPlaneOffsetEdge_Local(s->n_SystemId(), s->r_RefereeAt(0)->lidarId == 0 ? l0Id : l1Id,
					s->r_RefereeAt(0)->v_Measurement().head(6), info2, m_system));
			// t0

			for(int b = 1; b < s->r_Referees().size(); ++b)
			{
				if(s->r_RefereeAt(b)->p_Owner() == nullptr)
					continue;

				m_system.r_Add_Edge(CPlaneOffsetEdge_Global(s->n_SystemId(),
									s->r_RefereeAt(b)->p_Owner()->n_SystemId(),
									s->r_RefereeAt(0)->p_Owner()->n_SystemId(),
									s->r_RefereeAt(b)->lidarId == 0 ? l0Id : l1Id,
									s->r_RefereeAt(b)->v_Measurement().head(6), info2, m_system));
				// t1
			}

			//systemIdNeg --;
			systemId ++;
		}
		// add structure

		if(b_verbose)
			std::cout << "Optimizing " << n_iterations <<  " << " << f_threshold << std::endl;
		m_solver.Optimize(n_iterations, f_threshold);
		if(b_verbose)
			std::cout << "Done" << std::endl;
		// optimize it all

		int str_cnt = 0;
		int frame_cnt = 0;

		for(size_t i = 0, n = m_system.r_Vertex_Pool().n_Size(); i < n; ++ i) {
			//acces vertex, store to file, and upate structure
			const Eigen::Map<Eigen::VectorXd> &r_v_state = m_system.r_Vertex_Pool()[i].v_State();

			if(i == 0) {
				//std::cout << r_v_state.transpose() << std::endl;

				continue;
			}

			if(r_v_state.rows() == 6)
			{
				manager.r_v_Frames()[frame_cnt]->r_Pose().setRepresentation_6D(r_v_state); // first is constant

				frame_cnt ++;
			}
			// pose

			if(r_v_state.rows() == 4)
			{
				v_p_structure[str_cnt]->r_Plane() = r_v_state;

				str_cnt ++;
			}
			// plane
		}
		// reload data

		return m_solver.f_Chi_Squared_Error_Denorm();
	}

	void ExportData(CStructureManager &manager, int start, int end, int from, int to, bool iFrames)
	{
		int pFrom = from - start;
		int pTo = to - start;

		if(b_verbose)
			std::cout << "Export from " << from << " to " << to << std::endl;

		for(int a = pFrom; a <= pTo; ++a)
		{
			for(int c = 0; c < 2; ++c)
			{
				std::string namex;//("/mnt/ssd/isolony/dataset/GEO/lines-uncorrected/point-clouds-synced/");
				namex = manager.r_Data().lidarPath;
				char buffer [50];
				sprintf(buffer, "%06d.%d.pcd", start + a, c + 1);
				namex = namex + std::string(buffer);
				std::string outf(manager.r_Data().outputPath);
				outf = outf + std::string(buffer);

				pcl::PointCloud<velodyne_pointcloud::PointXYZIRP> pts;
				LoadPCL(namex, pts, Eigen::Matrix4d::Identity());
				//std::cout << namex << std::endl;
				// try loading pcl

				for(int b = 0; b < pts.size(); ++b)
				{
					Vector4d pt = Vector4d::Ones();
					pt << pts[b].x, pts[b].y, pts[b].z, 1;

					pt = manager.r_Tf().TransformPoint(pt, c == 0 ? tflidar0 : tflidar1, tfbase);
					// position

					double phase = pts[b].phase;
					int id = floor(phase / (1.0 / iFrames));
					if(id >= iFrames)
						id = iFrames - 1;
					Eigen::Matrix4d base_pos = manager.r_v_Frames()[a * iFrames]->r_Pose().r_Pw2c();
					Eigen::Matrix4d pos_inv = manager.r_v_Frames()[a * iFrames + id]->r_Pose().r_Pc2w();
					Eigen::Matrix<double, 6, 1> pos1 = manager.r_v_Frames()[a * iFrames + id]->r_Pose().v_Representation_6D();
					Eigen::Matrix<double, 6, 1> pos2 = manager.r_v_Frames()[a * iFrames + id + 1]->r_Pose().v_Representation_6D();
					//Eigen::Matrix<double, 6, 1> pos0 = pos1;
					Eigen::Matrix<double, 6, 1> pos;
					//if(a * iFrames + id > 0)
					//	pos0 = manager.r_v_Frames()[a * iFrames + id - 1]->r_Pose().v_Representation_6D();

					double nph = (phase - id * (1.0 / iFrames)) * iFrames;
					pos = (1.0 - nph) * pos1 + nph * pos2;


					std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > control_poses;
					for(int c = -1; c < 3; ++c)
					{
						int nid = a * iFrames + id + c;
						if(nid < 0)
							nid = 0;
						if(nid > manager.r_v_Frames().size() - 1)
							nid = manager.r_v_Frames().size() - 1;

						Eigen::Vector3d tr = manager.r_v_Frames()[nid]->r_Pose().v_Representation_6D().head(3);
						Eigen::Matrix3f R = manager.r_v_Frames()[nid]->r_Pose().r_fR();

						Eigen::Translation3f trans((float)tr(0), (float)tr(1), (float)tr(2));
						Eigen::AngleAxisf ax(R);
						Eigen::Affine3f aff = trans * ax;
						control_poses.push_back(aff);
					}
					velodyne_pointcloud::BSplineSE3 spline(control_poses);

					Eigen::Matrix4f npos = spline.estimate(nph).matrix();
					pos_inv = npos.cast <double> ();

					pt = pos_inv * pt;
					// true pose to world
					pt = base_pos * pt;
					// world to original
					pt = manager.r_Tf().TransformPoint(pt, tfbase, c == 0 ? tflidar0 : tflidar1);
					// base to lidar

					pts[b].x = pt(0);
					pts[b].y = pt(1);
					pts[b].z = pt(2);
				}

				SavePCL(outf, pts);
				//std::cout << outf << std::endl;
				// save PCL
			}
		}
		// output first half corrected
	}
};

#endif /* INCLUDE_MIDEND_STRUCTURES__H_ */
