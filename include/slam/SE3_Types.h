/**
 *	@file include/slam/SE3_Types.h
 *	@author Soso
 *	@date 2013
 *	@brief projection type for SE3
 */

#pragma once
#ifndef __SE3_TYPES_INCLUDED
#define __SE3_TYPES_INCLUDED

#include "slam/BaseTypes.h"
#include "slam/3DSolverBase.h"
#include "slam/Parser.h" // parsed types passed to constructors
#include <stdio.h>
#include <iostream>

/** \addtogroup se3
 *	@{
 */

/**
 *	@brief SE(3) pose vertex type
 */
class CVertexPose3D : public CBaseVertexImpl<CVertexPose3D, 6> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPose3D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPose3D(const Eigen::Matrix<double, 6, 1> &r_v_state)
		:CBaseVertexImpl<CVertexPose3D, 6>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		C3DJacobians::Relative_to_Absolute(m_v_state, r_v_delta.segment<6>(m_n_order), m_v_state);
		// simply reuse existing code

		/*// t_odo - use the code from 3DSolverBase.h !!! duplicates code, causes bugs !!!

		//sum the rotations
		Eigen::Matrix3d pQ = C3DJacobians::t_AxisAngle_to_RotMatrix(m_v_state.tail<3>());
		Eigen::Matrix3d dQ = C3DJacobians::t_AxisAngle_to_RotMatrix(r_v_delta.segment<3>(m_n_order + 3));

		//m_v_state(0) += r_v_delta(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		//m_v_state(1) += r_v_delta(m_n_order+1);
		//m_v_state(2) += r_v_delta(m_n_order+2);
		m_v_state.head<3>() += pQ * r_v_delta.segment<3>(m_n_order); // can select 3D segment starting at m_n_order; SSE accelerated

		Eigen::Matrix3d QQ;// = pQ * dQ;
		QQ.noalias() = pQ * dQ; // noalias says that the destination is not the same as the multiplicands and that the multiplication can be carried out without allocating intermediate storage
		//Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ);
		//m_v_state.tail(3) = axis;
		//m_v_state.tail<3>() = axis; // we have the information that it is 3 dimensional at compile time, if we put it in the <> brackets, the compiler will avoid allocating dynamic storage for the intermediate
		m_v_state.tail<3>() = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ); // also, no need for the intermediate object*/
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		//Operator_Plus(-r_v_delta); // call plus with negative delta, that should do the trick
		C3DJacobians::Relative_to_Absolute(m_v_state, -r_v_delta.segment<6>(m_n_order), m_v_state); // not sure how Eigen can wrap matrix segment and negative. want to avoid the off-chance that all of v_delta is going to be negated before calling the other function
	}
};

/**
 *	@brief SE(3) landmark vertex type
 */
class CVertexLandmark3D : public CBaseVertexImpl<CVertexLandmark3D, 3> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexLandmark3D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexLandmark3D(const Eigen::Vector3d &r_v_state)
		:CBaseVertexImpl<CVertexLandmark3D, 3>(r_v_state)
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_vertex is parsed vertex initializer
	 */
	inline CVertexLandmark3D(const CParserBase::TVertexXYZ &r_vertex)
		:CBaseVertexImpl<CVertexLandmark3D, 3>(r_vertex.m_v_position)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<3>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<3>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
class CEdgePose3D : public CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 6> {
public:
	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and an XYT edge.
	 */
	class CRelative_to_Absolute_XYZ_Initializer {
	protected:
		const Eigen::Matrix<double, 6, 1> &m_r_v_pose1; /**< @brief the first vertex */
		const Eigen::Matrix<double, 6, 1> &m_r_v_delta; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_delta is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_XYZ_Initializer(const Eigen::Matrix<double, 6, 1> &r_v_vertex1,
			const Eigen::Matrix<double, 6, 1> &r_v_delta) // just change the types, same as above
			:m_r_v_pose1(r_v_vertex1), m_r_v_delta(r_v_delta)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexPose3D() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
		{
			Eigen::Matrix<double, 6, 1> v_pose2;
			C3DJacobians::Relative_to_Absolute(m_r_v_pose1, m_r_v_delta, v_pose2); // implement your own equation here
			return CVertexPose3D(v_pose2);
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePose3D()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePose3D(const CParserBase::TEdge3D &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 6>(r_t_edge.m_n_node_0,
		r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		_ASSERTE(r_t_edge.m_n_node_0 < r_t_edge.m_n_node_1 || // either the vertices are ordered
			(r_t_edge.m_n_node_0 > r_t_edge.m_n_node_1 &&
			r_system.n_Vertex_Num() > r_t_edge.m_n_node_0)); // or they are not, but then both need to be in the system (a reversed loop closure)

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1,
			CRelative_to_Absolute_XYZ_Initializer(m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePose3D(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 6>(n_node0,
		n_node1, r_v_delta, r_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		_ASSERTE(n_node0 < n_node1 || // either the vertices are ordered
			(n_node0 > n_node1 && r_system.n_Vertex_Num() > n_node0)); // or they are not, but then both need to be in the system (a reversed loop closure)

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1,
			CRelative_to_Absolute_XYZ_Initializer(m_p_vertex0->r_v_State(), r_v_delta));
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TEdge3D &r_t_edge)
	{
		CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D),
			6>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 6, 1> &r_v_delta, const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D),
			6>::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 6, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 6, 6> &r_t_jacobian1, Eigen::Matrix<double, 6, 1> &r_v_expectation,
		Eigen::Matrix<double, 6, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

#if 0
		//C3DJacobians::Absolute_to_Relative(r_v_expectation, m_v_measurement, r_v_error); // this is slightly different, the inverse quat is on the left side, here we have it on the right.
		C3DJacobians::Relative_to_Absolute(m_v_measurement, -r_v_expectation, r_v_error); // this allows for inverse on the right but we need to manually "conjugate" the pose by flipping the sign.
		r_v_error.head<3>() = m_v_measurement.head<3>() - r_v_expectation.head<3>(); // somehow the positional error is not rotated in the code below, but in both Absolute_to_Relative() and Relative_to_Absolute() it is, we need to undo it.
		// calculate error as m_v_measurement - r_v_expectation
#else // 0
		r_v_error.head<3>() = m_v_measurement.head<3>() - r_v_expectation.head<3>();
		Eigen::Quaterniond pQ, dQ;
		C3DJacobians::AxisAngle_to_Quat(m_v_measurement.tail<3>(), pQ);
		C3DJacobians::AxisAngle_to_Quat(r_v_expectation.tail<3>(), dQ);
		Eigen::Vector3d v_aang;
		C3DJacobians::Quat_to_AxisAngle(pQ * dQ.conjugate(), v_aang);
		r_v_error.tail<3>() = v_aang;
		// same as above but saves calculation of the rotated positional difference which then gets overwritten
#endif // 0
		// both branches work, the first does some unnecessary work
		// the below branch uses rotation matrices, being slightly more expensive and also
		// different numerically than the rest of the calculus which relies on quaternions

		/*
		//const Eigen::Matrix<double, 6, 1> &p1 = m_v_measurement;
		//const Eigen::Matrix<double, 6, 1> &p2 = r_v_expectation;

		//r_v_error(0) = p1(0) - p2(0);
		//r_v_error(1) = p1(1) - p2(1);
		//r_v_error(2) = p1(2) - p2(2);
		r_v_error.head<3>() = m_v_measurement.head<3>() - r_v_expectation.head<3>();

		//sum the rotations
		Eigen::Matrix3d pQ = C3DJacobians::t_AxisAngle_to_RotMatrix(m_v_measurement.tail<3>());
		Eigen::Matrix3d dQ = C3DJacobians::t_AxisAngle_to_RotMatrix(r_v_expectation.tail<3>());
		//Eigen::Matrix3d dQ_inv = dQ.inverse();

		Eigen::Matrix3d QQ;// = pQ * dQ_inv;
		QQ.noalias() = pQ * dQ.inverse(); // eigen likes long expressions; multiplication needs .noalias() so that eigen knows it can do it without allocating intermediate storage
		//Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ);

		//r_v_error(3) = axis(0);
		//r_v_error(4) = axis(1);
		//r_v_error(5) = axis(2);
		r_v_error.tail<3>() = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ); // avoid copying around
		//TODO: fix the angle somehow? this cannot be right
		// calculates error (possibly re-calculates, if running A-SLAM)
		*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 6> p_jacobi[2];
		Eigen::Matrix<double, 6, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi[0], p_jacobi[1], v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

#if 0 // this is just a test, not really useful for anything

/**
 *	@brief SE(3) pose-pose-pose edge (a hyperedge test)
 */
class CEdgePose3D_Ternary : public CBaseEdgeImpl<CEdgePose3D_Ternary,
	MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D), 6> {
public:
	typedef CBaseEdgeImpl<CEdgePose3D_Ternary, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D), 6> _TyBase; /**< @brief base edge type */
	typedef Eigen::Matrix<double, 6, 6> Matrix6d; /**< @brief 6x6 matrix type */
	typedef Eigen::Matrix<double, 6, 1> Vector6d; /**< @brief 6D vector type */ // just shorter names

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePose3D_Ternary()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePose3D_Ternary(const CParserBase::TEdge3D &r_t_edge, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(r_t_edge.m_n_node_0,
		r_t_edge.m_n_node_1, r_t_edge.m_n_node_1/*2*/), r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1,
			CEdgePose3D::CRelative_to_Absolute_XYZ_Initializer(m_vertex_ptr.Get<0>()->v_State(), r_t_edge.m_v_delta));
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1/*2*/,
			CEdgePose3D::CRelative_to_Absolute_XYZ_Initializer(m_vertex_ptr.Get<0>()->v_State(), r_t_edge.m_v_delta));
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1/*2*/].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TEdge3D &r_t_edge)
	{
		_TyBase::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Vector6d &r_v_expectation, Vector6d &r_v_error) const
	{
		C3DJacobians::Absolute_to_Relative(m_vertex_ptr.Get<0>()->v_State(),
			m_vertex_ptr.Get<1>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>());
		r_t_jacobian_tuple.Get<2>().setIdentity(); // don't really have code that calculates this (yet)
		// calculates the expectation and the jacobians

		r_v_error.head<3>() = m_v_measurement.head<3>() - r_v_expectation.head<3>();

		Eigen::Matrix3d pQ = C3DJacobians::t_AxisAngle_to_RotMatrix(m_v_measurement.tail<3>());
		Eigen::Matrix3d dQ = C3DJacobians::t_AxisAngle_to_RotMatrix(r_v_expectation.tail<3>());
		Eigen::Matrix3d QQ;
		QQ.noalias() = pQ * dQ.inverse();
		// sum the rotations

		r_v_error.tail<3>() = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ);
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		_TyBase::_TyJacobianTuple jacobian_tuple; // note that this may not be aligned
		Vector6d v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(jacobian_tuple, v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		// note that the jacobians are not needed here; if implementing types that will be used
		// with Levenberg-Marquardt where the chi2 is frequently used, it is better to write
		// a special code to calculate the error without calculating the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

#endif // 0

/**
 *	@brief SE(3) pose-landmark edge
 */
class CEdgePoseLandmark3D : public CBaseEdgeImpl<CEdgePoseLandmark3D, MakeTypelist(CVertexPose3D, CVertexLandmark3D), 3> {
public:
	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and an XYT edge.
	 */
	class CRelative_to_Absolute_XYZ_Initializer {
	protected:
		const Eigen::Matrix<double, 6, 1> &m_r_v_pose1; /**< @brief the first vertex */
		const Eigen::Matrix<double, 3, 1> &m_r_v_delta; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_delta is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_XYZ_Initializer(const Eigen::Matrix<double, 6, 1> &r_v_vertex1,
			const Eigen::Matrix<double, 3, 1> &r_v_delta) // just change the types, same as above
			:m_r_v_pose1(r_v_vertex1), m_r_v_delta(r_v_delta)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexLandmark3D() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
		{
			Eigen::Matrix<double, 6, 1> v_pose2;
			Eigen::Matrix<double, 6, 1> relative_pose;
			relative_pose << m_r_v_delta(0), m_r_v_delta(1), m_r_v_delta(2), m_r_v_pose1(3), m_r_v_pose1(4), m_r_v_pose1(5);	//dummy rotation
			C3DJacobians::Relative_to_Absolute(m_r_v_pose1, relative_pose, v_pose2); // implement your own equation here
			return CVertexLandmark3D(v_pose2.segment<3>(0));
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePoseLandmark3D()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePoseLandmark3D(const CParserBase::TLandmark3D_XYZ &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePoseLandmark3D, MakeTypelist(CVertexPose3D, CVertexLandmark3D), 3>(r_t_edge.m_n_node_0,
		r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(r_t_edge.m_n_node_1,
			CRelative_to_Absolute_XYZ_Initializer(m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get vertices (initialize if required)

		//print me edge
		//std::cout << r_t_edge.m_n_node_0 << " to " << r_t_edge.m_n_node_1 << std::endl;
		//std::cout << r_t_edge.m_v_delta << std::endl;

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePoseLandmark3D(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePoseLandmark3D, MakeTypelist(CVertexPose3D, CVertexLandmark3D), 3>(n_node0,
		n_node1, r_v_delta, r_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(n_node1,
			CRelative_to_Absolute_XYZ_Initializer(m_p_vertex0->r_v_State(), r_v_delta));
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TLandmark3D_XYZ &r_t_edge)
	{
		CBaseEdgeImpl<CEdgePoseLandmark3D, MakeTypelist(CVertexPose3D, CVertexLandmark3D),
			3>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 3> &r_t_jacobian1, Eigen::Matrix<double, 3, 1> &r_v_expectation,
		Eigen::Matrix<double, 3, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << "expect: " << r_v_expectation(0) << " " << r_v_expectation(1) << " " << r_v_expectation(2) << " -- ";
		//std::cout << m_v_measurement(0) << " " << m_v_measurement(1) << " " << m_v_measurement(2) << std::endl;

		//const Eigen::Matrix<double, 6, 1> &p1 = m_v_measurement;
		//const Eigen::Matrix<double, 6, 1> &p2 = r_v_expectation;

		//r_v_error(0) = p1(0) - p2(0);
		//r_v_error(1) = p1(1) - p2(1);
		//r_v_error(2) = p1(2) - p2(2);
		r_v_error = m_v_measurement - r_v_expectation;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 6> p_jacobi0;
		Eigen::Matrix<double, 3, 3> p_jacobi1;
		Eigen::Matrix<double, 3, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/** @} */ // end of group

/** \addtogroup parser
 *	@{
 */

/**
 *	@brief edge traits for SE(3) solver
 */
template <class CParsedStructure>
class CSE3OnlyPoseEdgeTraits {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief it should fail on unknown edge types */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "unknown edge type occurred";
	}
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3OnlyPoseEdgeTraits<CParserBase::TEdge3D> {
public:
	typedef CEdgePose3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

#if 0
/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3OnlyPoseEdgeTraits<CParserBase::TVertex3D> { // nonsense, CParserBase::TVertex3D is not an edge, this does not belong here
public:
	typedef CIgnoreEdgeType _TyEdge; /**< @brief the edge type to construct from the parsed type */
};
#endif // 0

/**
 *	@brief edge traits for SE(3) solver
 */
template <class CParsedStructure>
class CSE3LandmarkPoseEdgeTraits {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief it should fail on unknown edge types */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "unknown edge type occurred";
	}
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdge3D> {
public:
	typedef CEdgePose3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TLandmark3D_XYZ> {
public:
	typedef CEdgePoseLandmark3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/** @} */ // end of group

/**
 *	@brief GPS edge
 */
class CEdge3DXYZ : public CBaseEdgeImpl<CEdge3DXYZ, MakeTypelist(CVertexPose3D), 3> {
public:
	typedef CBaseEdgeImpl<CEdge3DXYZ, MakeTypelist(CVertexPose3D), 3> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdge3DXYZ()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdge3DXYZ(size_t n_vertex0, const Eigen::Vector3d &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_vertex0, CInitializeNullVertex<>());
		// initialize vertex 1 manually
	}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdge3DXYZ(const CParserBase::TEdgeXYZ &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Vector3d &r_v_delta, const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma)
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 6> &r_t_jacobian0,
			Eigen::Vector3d &r_v_expectation, Eigen::Vector3d &r_v_error) const
	{
		r_v_expectation = m_p_vertex0->r_v_State().head<3>();

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		Eigen::Matrix<double, 3, 6> &H1 = r_t_jacobian0;
		// can actually work inplace

		Eigen::Matrix<double, 6, 1> p_delta;

		//for XYZ and RPY
		for(int j = 0; j < 6; ++ j) {
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);

			H1.col(j) = (p_delta.head<3>() - r_v_expectation) * scalar;
		}
		r_v_error = m_v_measurement - r_v_expectation;

		//std::cout << "exp: " << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() <<
		//		" : " << (r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error) << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;

		v_error = m_v_measurement - m_p_vertex0->r_v_State().head<3>();
		//std::cout << "ch: " << m_v_measurement.transpose() << " <> " << m_p_vertex0->r_v_State().head<3>().transpose() << std::endl;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/** @} */ // end of group

/**
 *	@brief GPS + offset edge
 */
class CEdge3DXYZOFF : public CBaseEdgeImpl<CEdge3DXYZOFF, MakeTypelist(CVertexPose3D), 3> {
protected:
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> m_v_offset;
public:
	typedef CBaseEdgeImpl<CEdge3DXYZOFF, MakeTypelist(CVertexPose3D), 3> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdge3DXYZOFF()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdge3DXYZOFF(size_t n_vertex0, const Eigen::Vector3d &r_v_delta, const Eigen::Vector3d &r_v_offset,
		const Eigen::Matrix<double, 3, 3> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma),
		 m_v_offset(r_v_offset)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_vertex0, CInitializeNullVertex<>());
		// initialize vertex 1 manually
	}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdge3DXYZOFF(const CParserBase::TEdgeXYZOFF &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma),
		 m_v_offset(r_t_edge.m_v_offset)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Vector3d &r_v_delta, const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma)
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 6> &r_t_jacobian0,
			Eigen::Vector3d &r_v_expectation, Eigen::Vector3d &r_v_error) const
	{
		Eigen::Matrix<double, 6, 1> offsetL = Eigen::Matrix<double, 6, 1>::Zero();
		offsetL.head(3) = m_v_offset;
		Eigen::Matrix<double, 6, 1> offsetW;
		C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), offsetL, offsetW);
		r_v_expectation = offsetW.head(3);
		// get expectation with offset

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		Eigen::Matrix<double, 3, 6> &H1 = r_t_jacobian0;
		// can actually work inplace

		Eigen::Matrix<double, 6, 1> p_delta;

		//for XYZ and RPY
		for(int j = 0; j < 6; ++ j) {
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Relative_to_Absolute(p_delta, offsetL, offsetW);

			H1.col(j) = (offsetW.head<3>() - r_v_expectation) * scalar;
		}
		r_v_error = m_v_measurement - r_v_expectation;

		//std::cout << "exp: " << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() <<
		//		" : " << (r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error) << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 1> offsetL = Eigen::Matrix<double, 6, 1>::Zero();
		offsetL.head(3) = m_v_offset;
		Eigen::Matrix<double, 6, 1> offsetW;
		C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), offsetL, offsetW);

		Eigen::Vector3d v_error;
		v_error = m_v_measurement - offsetW.head<3>();
		//std::cout << "ch: " << m_v_measurement.transpose() << " <> " << m_p_vertex0->r_v_State().head<3>().transpose() << std::endl;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdgeXYZ> {
public:
	typedef CEdge3DXYZ _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdgeXYZOFF> {
public:
	typedef CEdge3DXYZOFF _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *  @brief BSpline 5xpose edge
 */
class CBSplineEdge : public CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 6, 6> {

public:
  typedef CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 6, 6> _TyBase; /**< @brief base class */

public:
  __GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

  /**
   *  @brief default constructor; has no effect
   */
  inline CBSplineEdge()
  {}

  template <class CSystem>
  CBSplineEdge(const CParserBase::TEdgeSpline3D &r_t_edge, CSystem &r_system)
    :_TyBase(typename _TyBase::_TyVertexIndexTuple(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
        r_t_edge.m_n_node_2, r_t_edge.m_n_node_3, r_t_edge.m_n_node_4),
    r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
  {
    m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_2, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_3, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_4, CInitializeNullVertex<CVertexPose3D>());
    // get vertices (initialize if required)
  }

  /**
   *  @brief constructor; converts parsed edge to edge representation
   *
   *  @tparam CSystem is type of system where this edge is being stored
   *
   *  @param[in] n_node0 is (zero-based) index of the first (origin) node
   *  @param[in] n_node1 is (zero-based) index of the second (endpoint) node
   *  @param[in] r_v_delta is measurement vector
   *  @param[in] r_t_inv_sigma is the information matrix
   *  @param[in,out] r_system is reference to system (used to query edge vertices)
   */
  template <class CSystem>
  CBSplineEdge(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
      const Eigen::Matrix<double, 6, 1> &r_v_delta, const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
    :_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4), r_v_delta, r_t_inv_sigma)
  {
    m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
    m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node4, CInitializeNullVertex<CVertexPose3D>());
    // get vertices (initialize if required)
  }

  /**
   *  @brief updates the edge with a new measurement
   *
   *  @param[in] r_v_delta is new measurement vector
   *  @param[in] r_t_inv_sigma is new information matrix
   */
  /*inline void Update(const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
  {
    _TyBase::Update(r_v_delta, r_t_inv_sigma);
  }*/

  /**
   *  @brief calculates jacobians, expectation and error
   *
   *  @param[out] r_t_jacobian0 is jacobian, associated with the first vertex
   *  @param[out] r_t_jacobian1 is jacobian, associated with the second vertex
   *  @param[out] r_v_expectation is expecation vector
   *  @param[out] r_v_error is error vector
   */
  inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
      Eigen::Matrix<double, 6, 1> &r_v_expectation, Eigen::Matrix<double, 6, 1> &r_v_error) const // change dimensionality of eigen types, if required
  {
    // calculates the expectation and the jacobians
    /*std::cout << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
    std::cout << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
    std::cout << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
    std::cout << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/

    BSplineSE3 spline(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
        m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
    r_v_expectation = spline.bspline_error6D(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State());
    // expectation error

    /*std::cout << "V0: " << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
    std::cout << "V1: " << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
    std::cout << "V2: " << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
    std::cout << "V3: " << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;
    std::cout << "V4: " << m_vertex_ptr.Get<4>()->r_v_State().transpose() << std::endl;
    std::cout << "033: " << spline.estimate(0.33).matrix() << std::endl;
    std::cout << "066: " << spline.estimate(0.66).matrix() << std::endl;*/

    //std::cout << "EXP: " << r_v_expectation << std::endl;

    const double delta = 1e-3;  // smaller delta wont work ?? why? is the error function too ??
    const double scalar = 1.0 / (delta);
    Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack
    Eigen::Matrix<double, 6, 1> p_delta;

    for(size_t j = 0; j < 6; ++j) // four 1x6 jacobians
    {
      C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j), p_delta);
      BSplineSE3 spline0(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
          m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
      r_t_jacobian_tuple.Get<0>().col(j) = (spline0.bspline_error6D(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
      // J0

      C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);
      BSplineSE3 spline1(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
          m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
      r_t_jacobian_tuple.Get<1>().col(j) = (spline1.bspline_error6D(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
      // J1

      C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
      BSplineSE3 spline2(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
          p_delta, m_vertex_ptr.Get<3>()->r_v_State());
      r_t_jacobian_tuple.Get<2>().col(j) = (spline2.bspline_error6D(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
      // J2

      C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j), p_delta);
      BSplineSE3 spline3(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
          m_vertex_ptr.Get<2>()->r_v_State(), p_delta);
      r_t_jacobian_tuple.Get<3>().col(j) = (spline3.bspline_error6D(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
      // J3

      C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<4>()->r_v_State(), Eps.col(j), p_delta);
      BSplineSE3 spline4(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
          m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
      r_t_jacobian_tuple.Get<4>().col(j) = (spline4.bspline_error6D(m_v_measurement(0), p_delta) - r_v_expectation) * scalar;
      // J4
    }
    // error is given by spline function
    /*std::cout << "J0: " << r_t_jacobian_tuple.Get<0>() << std::endl << std::endl;
    std::cout << "J1: " << r_t_jacobian_tuple.Get<1>() << std::endl << std::endl;
    std::cout << "J2: " << r_t_jacobian_tuple.Get<2>() << std::endl << std::endl;
    std::cout << "J3: " << r_t_jacobian_tuple.Get<3>() << std::endl << std::endl;
    std::cout << "J4: " << r_t_jacobian_tuple.Get<4>() << std::endl << std::endl;*/

    r_v_error = - r_v_expectation;  // measurement should be zero
    // calculates error (possibly re-calculates, if running A-SLAM)
  }

  /**
   *  @brief calculates \f$\chi^2\f$ error
   *  @return Returns (unweighted) \f$\chi^2\f$ error for this edge.
   */
  inline double f_Chi_Squared_Error() const
  {
    /*std::cout << "er " <<  m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
    std::cout << "er " <<  m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
    std::cout << "er " <<  m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
    std::cout << "er " <<  m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/

    BSplineSE3 spline(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
      m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
    Eigen::Matrix<double, 6, 1> v_error = - spline.bspline_error6D(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State());
    // calculates the expectation, error and the jacobians

    return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
  }
};

// todo: potom uplne dole

/**
 *  @brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdgeSpline3D> {
public:
  typedef CBSplineEdge _TyEdge; /**< @brief the edge type to construct from the parsed type */
};


#endif // !__SE3_TYPES_INCLUDED
