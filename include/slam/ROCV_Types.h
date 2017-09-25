/*
								+----------------------------------+
								|                                  |
								|       ***  ROCV types  ***       |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|           ROCV_Types.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __ROCV_TYPES_INCLUDED
#define __ROCV_TYPES_INCLUDED

/**
 *	@file include/slam/ROCV_Types.h
 *	@brief range-only, constant velocity primitive types
 *	@author -tHE SWINe-
 *	@date 2014-07-08
 */

#include "slam/BaseTypes.h"
#include "slam/SE3_Types.h" // uses CVertexLandmark3D
#include "slam/Parser.h" // parsed types passed to constructors

/**
 *	@brief 3D position / velocity pose vertex type
 */
class CVertexPositionVelocity3D : public CBaseVertexImpl<CVertexPositionVelocity3D, 6> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPositionVelocity3D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPositionVelocity3D(const Eigen::Matrix<double, 6, 1> &r_v_state)
		:CBaseVertexImpl<CVertexPositionVelocity3D, 6>(r_v_state)
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_vertex is state vector initializer
	 */
	inline CVertexPositionVelocity3D(const CParserBase::TVertex3D &r_vertex)
		:CBaseVertexImpl<CVertexPositionVelocity3D, 6>(r_vertex.m_v_position)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<6>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<6>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
	}
};

/**
 *	@brief edge between 3D position / velocity pose and a landmark
 */
template <const bool b_use_squared_error = false>
class CEdgePosVel_Landmark3D : public CBaseEdgeImpl<CEdgePosVel_Landmark3D<b_use_squared_error>,
	MakeTypelist(CVertexPositionVelocity3D, CVertexLandmark3D), 1> {
public:
	typedef CBaseEdgeImpl<CEdgePosVel_Landmark3D<b_use_squared_error>,
		MakeTypelist(CVertexPositionVelocity3D, CVertexLandmark3D), 1> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePosVel_Landmark3D()
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
	CEdgePosVel_Landmark3D(const CParserBase::TROCV_RangeEdge &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		_TyBase::m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPositionVelocity3D>(
			r_t_edge.m_n_node_0, typename _TyBase::template CInitializeNullVertex<>());
		_TyBase::m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(
			r_t_edge.m_n_node_1, typename _TyBase::template CInitializeNullVertex<CVertexLandmark3D>()); // unable to initialize reasonably
		// get vertices

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; converts edge data to edge representation
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
	CEdgePosVel_Landmark3D(size_t n_node0, size_t n_node1, Eigen::Matrix<double, 6, 1> &r_v_delta,
		Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		_TyBase::m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPositionVelocity3D>(
			n_node0, typename _TyBase::template CInitializeNullVertex<>());
		_TyBase::m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(
			n_node1, typename _TyBase::template CInitializeNullVertex<CVertexLandmark3D>()); // unable to initialize reasonably
		// get vertices

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TROCV_RangeEdge &r_t_edge)
	{
		_TyBase::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 1, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 1, 3> &r_t_jacobian1, Eigen::Matrix<double, 1, 1> &r_v_expectation,
		Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		const Eigen::Matrix<double, 6, 1> &v_p = _TyBase::m_p_vertex0->r_v_State();
		const Eigen::Vector3d &v_l = _TyBase::m_p_vertex1->r_v_State();
		double x = v_p(0), y = v_p(1), z = v_p(2);
		double lx = v_l(0), ly = v_l(1), lz = v_l(2);

		if(!b_use_squared_error) {
			r_t_jacobian0(0) = (-lx + x) / sqrt(lx * lx - 2 * lx * x + x * x +
				ly * ly - 2 * ly * y + y * y + lz * lz - 2 * lz * z + z * z);
			r_t_jacobian0(1) = (-ly + y) / sqrt(lx * lx - 2 * lx * x + x * x +
				ly * ly - 2 * ly * y + y * y + lz * lz - 2 * lz * z + z * z);
			r_t_jacobian0(2) = (-lz + z) / sqrt(lx * lx - 2 * lx * x + x * x +
				ly * ly - 2 * ly * y + y * y + lz * lz - 2 * lz * z + z * z);
			//r_t_jacobian0(0) = -(lx-x)*(1/(sqrt(lx*lx-2.0*lx*x+x*x+ly*ly-2.0*ly*y+y*y+lz*lz-2.0*lz*z+z*z))); // simple
			//r_t_jacobian0(1) = -(ly-y)*(1/(sqrt(lx*lx-2.0*lx*x+x*x+ly*ly-2.0*ly*y+y*y+lz*lz-2.0*lz*z+z*z)));
			//r_t_jacobian0(2) = -(lz-z)*(1/(sqrt(lx*lx-2.0*lx*x+x*x+ly*ly-2.0*ly*y+y*y+lz*lz-2.0*lz*z+z*z)));
		} else {
			const double distmeas = _TyBase::m_v_measurement(0);
			r_t_jacobian0(0) = 2 * (lx - x) * (-distmeas + sqrt(lx * lx - 2 * lx * x + x * x + ly * ly - 2 *
				ly * y + y * y + lz * lz - 2 * lz * z + z * z)) / sqrt(lx * lx - 2 * lx * x + x * x + ly * ly -
				2 * ly * y + y * y + lz * lz - 2 * lz * z + z * z);
			r_t_jacobian0(1) = 2 * (ly - y) * (-distmeas + sqrt(lx * lx - 2 * lx * x + x * x + ly * ly - 2 *
				ly * y + y * y + lz * lz - 2 * lz * z + z * z)) / sqrt(lx * lx - 2 * lx * x + x * x + ly * ly -
				2 * ly * y + y * y + lz * lz - 2 * lz * z + z * z);
			r_t_jacobian0(2) = 2 * (lz - z) * (-distmeas + sqrt(lx * lx - 2 * lx * x + x * x + ly * ly - 2 *
				ly * y + y * y + lz * lz - 2 * lz * z + z * z)) / sqrt(lx * lx - 2 * lx * x + x * x + ly * ly -
				2 * ly * y + y * y + lz * lz - 2 * lz * z + z * z);
		}
		r_t_jacobian0.tail<3>().setZero();
		//r_t_jacobian0 = -r_t_jacobian0; // sign matters?

		r_t_jacobian1 = -r_t_jacobian0.head<3>();
		// no need to bother calculating the same stuff twice

		r_v_expectation(0) = (v_p.head<3>() - v_l).norm(); // just the distance
		r_v_error = _TyBase::m_v_measurement - r_v_expectation;
		//r_v_error = -r_v_error; // sign matters?
		if(b_use_squared_error)
			r_v_error(0) *= r_v_error(0); // error squared
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 1> v_expectation, v_error;
		v_expectation(0) = (_TyBase::m_p_vertex0->r_v_State().template head<3>() -
			_TyBase::m_p_vertex1->r_v_State()).norm(); // just the distance
		v_error = _TyBase::m_v_measurement - v_expectation;
		if(b_use_squared_error)
			v_error(0) *= v_error(0); // error squared
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * _TyBase::m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief 3D landmark position ptior edge
 */
class CEdgeLandmark3DPrior : public CBaseEdgeImpl<CEdgeLandmark3DPrior,
	MakeTypelist(CVertexLandmark3D), 3, 0> {
public:
	typedef CBaseEdgeImpl<CEdgeLandmark3DPrior, MakeTypelist(CVertexLandmark3D), 3, 0> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeLandmark3DPrior()
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
	CEdgeLandmark3DPrior(const CParserBase::TUnaryFactor3D &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, Eigen::Matrix<double, 0, 1>(), r_t_edge.m_t_factor)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(r_t_edge.m_n_node_0,
			CInitializeNullVertex<>());
		// get vertices

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; converts edge data to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgeLandmark3DPrior(int n_node0, Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, Eigen::Matrix<double, 0, 1>(), r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(n_node0,
			CInitializeNullVertex<>());
		// get vertices

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TUnaryFactor3D &r_t_edge)
	{
		_TyBase::Update(Eigen::Matrix<double, 0, 1>(), r_t_edge.m_t_factor);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 1> &r_v_expectation, Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		r_t_jacobian0.setIdentity();
		//r_t_jacobian0 = -r_t_jacobian0;
		r_v_expectation.setZero();
		r_v_error.setZero();
		//r_v_error = m_p_vertex0->r_v_State(); // this is not an ordinary UF, rather this is an anchor // and it does not work
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		return 0;
	}
};

/**
 *	@brief constant velocity constraint between two 3D position / velocity poses
 */
template <const bool b_1D_residual = false>
class CEdgeConstVelocity3D : public CBaseEdgeImpl<CEdgeConstVelocity3D<b_1D_residual>,
	MakeTypelist(CVertexPositionVelocity3D, CVertexPositionVelocity3D), (b_1D_residual)? 1 : 6, 1> {
public:
	typedef CBaseEdgeImpl<CEdgeConstVelocity3D<b_1D_residual>,
		MakeTypelist(CVertexPositionVelocity3D, CVertexPositionVelocity3D),
		(b_1D_residual)? 1 : 6, 1> _TyBase; /**< @brief base edge type */ // 6D residual but only 1D storage

	/**
	 *	@brief edge parameters, stored as enum
	 */
	enum {
		n_residual_dim = /*(b_1D_residual)? 1 : 6*/_TyBase::n_residual_dimension /**< @brief residual dimension shortcut */ // t_odo - use n_residual_dimension from _TyBase (see if we can)
	};

	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and delta time.
	 */
	class CConstVelocity_Initializer {
	protected:
		const Eigen::Matrix<double, 6, 1> &m_r_v_pose1; /**< @brief the first vertex */
		const Eigen::Matrix<double, 1, 1> &m_r_v_delta; /**< @brief delta time */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_delta is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CConstVelocity_Initializer(const Eigen::Matrix<double, 6, 1> &r_v_vertex1,
			const Eigen::Matrix<double, 1, 1> &r_v_delta)
			:m_r_v_pose1(r_v_vertex1), m_r_v_delta(r_v_delta)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexPositionVelocity3D() const
		{
			double f_delta_time = m_r_v_delta(0, 0); // measurement
			Eigen::Matrix<double, 6, 1> v_pose2 = m_r_v_pose1; // start at the previous pose
			v_pose2.head<3>() += v_pose2.tail<3>() * f_delta_time; // move by velocity times delta-time
			return CVertexPositionVelocity3D(v_pose2);
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeConstVelocity3D()
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
	CEdgeConstVelocity3D(const CParserBase::TROCV_DeltaTimeEdge &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, r_t_edge.m_v_delta,
		r_t_edge.m_t_inv_sigma.topLeftCorner<n_residual_dim, n_residual_dim>())
	{
		_TyBase::m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPositionVelocity3D>(
			r_t_edge.m_n_node_0, typename _TyBase::template CInitializeNullVertex<>());
		_TyBase::m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPositionVelocity3D>(r_t_edge.m_n_node_1,
			CConstVelocity_Initializer(_TyBase::m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get vertices

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; converts edge data to edge representation
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
	CEdgeConstVelocity3D(size_t n_node0, size_t n_node1, Eigen::Matrix<double, 1, 1> &r_v_delta,
		Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma.topLeftCorner<n_residual_dim, n_residual_dim>())
	{
		_TyBase::m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPositionVelocity3D>(
			n_node0, typename _TyBase::template CInitializeNullVertex<>());
		_TyBase::m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPositionVelocity3D>(n_node1,
			CConstVelocity_Initializer(_TyBase::m_p_vertex0->r_v_State(), r_v_delta));
		// get vertices

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TROCV_DeltaTimeEdge &r_t_edge)
	{
		_TyBase::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma.topLeftCorner<n_residual_dim, n_residual_dim>());
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, n_residual_dim, 6> &r_t_jacobian0,
		Eigen::Matrix<double, n_residual_dim, 6> &r_t_jacobian1, Eigen::Matrix<double, n_residual_dim, 1> &r_v_expectation,
		Eigen::Matrix<double, n_residual_dim, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		const Eigen::Matrix<double, 6, 1> &v_prev = _TyBase::m_p_vertex0->r_v_State();
		const Eigen::Matrix<double, 6, 1> &v_cur = _TyBase::m_p_vertex1->r_v_State();
		double f_delta_time = _TyBase::m_v_measurement(0);

		if(b_1D_residual) {
			{
				double dt = f_delta_time; // just rename
				double x = v_cur(0), y = v_cur(1), z = v_cur(2), vx = v_cur(3), vy = v_cur(4), vz = v_cur(5);
				double prevx = v_prev(0), prevy = v_prev(1), prevz = v_prev(2),
					prevvx = v_prev(3), prevvy = v_prev(4), prevvz = v_prev(5);
				double J0[1][6], J1[1][6];
				J0[0][0] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-
					2.0*z*dt*prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*
					prevvx+prevy*prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-
					2.0*vx*prevvx+prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*
					(2.0*dt*prevvx-2.0*x+2.0*prevx)/2.0;
				J0[0][1] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-
					2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx
					+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*
					prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*
					vz*prevvz+prevvx*prevvx))*(2.0*dt*prevvy+2.0*prevy-2.0*y)/2.0;
				J0[0][2] = 1/(sqrt(-2.0*y*dt*
					prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*prevvz+2.0*prevy*dt*prevvy-
					2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*prevy+dt*dt*prevvy*prevvy-2.0*
					y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+prevvz*prevvz+vy*vy-2.0*vy*prevvy+
					prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(2.0*dt*prevvz-2.0*z+2.0*prevz)/2.0;
				J0[0][3] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*prevvz+
					2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*prevy+dt*dt*
					prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+prevvz*prevvz+vy
					*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(2.0*dt*prevx-2.0*dt*x+2.0*dt*
					dt*prevvx-2.0*vx+2.0*prevvx)/2.0;
				J0[0][4] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*
					x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*
					x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+
					dt*dt*prevvz*prevvz-2.0*vx*prevvx+prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+
					prevvx*prevvx))*(-2.0*dt*y+2.0*dt*prevy+2.0*dt*dt*prevvy-2.0*vy+2.0*prevvy)/2.0;
				J0[0][5] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*
					prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*
					prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+
					prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(2.0*dt*prevz
					-2.0*dt*z+2.0*dt*dt*prevvz+2.0*prevvz-2.0*vz)/2.0;

				J1[0][0] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*
					prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*
					prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+
					prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(-2.0*dt*prevvx-
					2.0*prevx+2.0*x)/2.0;
				J1[0][1] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+
					2.0*prevz*dt*prevvz-2.0*z*dt*prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+
					dt*dt*prevvx*prevvx+prevy*prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*
					prevvz-2.0*vx*prevvx+prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*
					prevvx))*(-2.0*dt*prevvy+2.0*y-2.0*prevy)/2.0;
				J1[0][2] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*
					dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+
					prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*
					prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+
					vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(-2.0*dt*prevvz+2.0*z-2.0*prevz)/2.0;
				J1[0][3] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*
					prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*prevy+
					dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+prevvz*
					prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(2.0*vx-2.0*prevvx)/2.0;
				J1[0][4] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*
					prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*
					prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+
					prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(2.0*vy-2.0*
					prevvy)/2.0;
				J1[0][5] = 1/(sqrt(-2.0*y*dt*prevvy+2.0*prevx*dt*prevvx-2.0*x*dt*prevvx+2.0*prevz*dt*prevvz-2.0*z*dt*
					prevvz+2.0*prevy*dt*prevvy-2.0*x*prevx+y*y+prevx*prevx+x*x+z*z+vx*vx+dt*dt*prevvx*prevvx+prevy*
					prevy+dt*dt*prevvy*prevvy-2.0*y*prevy-2.0*z*prevz+prevz*prevz+dt*dt*prevvz*prevvz-2.0*vx*prevvx+
					prevvz*prevvz+vy*vy-2.0*vy*prevvy+prevvy*prevvy+vz*vz-2.0*vz*prevvz+prevvx*prevvx))*(2.0*vz-2.0*
					prevvz)/2.0;

				for(int c = 0; c < 6; ++ c)
					r_t_jacobian0(0, c) = J0[0][c];
				for(int c = 0; c < 6; ++ c)
					r_t_jacobian1(0, c) = J1[0][c];
			}
			// jacobians for squared residual

			Eigen::Matrix<double, 6, 1> v_expectation6;
			v_expectation6.head<3>() = v_prev.head<3>() + v_prev.tail<3>() * f_delta_time; // simple Newtonian motion
			v_expectation6.tail<3>() = v_prev.tail<3>(); // constant velocity model
			r_v_expectation(0) = f_delta_time; // should be time between the actual vertices, kind of unused anyways
			r_v_error(0) = (v_cur - v_expectation6).norm(); // 1D residual
			//printf("%g ", r_v_error.norm());
		} else {
#if 1
			{
				double dt = -f_delta_time; // just rename
				r_t_jacobian0 << -1,   0,   0,  dt,   0,   0,
								  0,  -1,   0,   0,  dt,   0,
								  0,   0,  -1,   0,   0,  dt,
								  0,   0,   0,  -1,   0,   0,
								  0,   0,   0,   0,  -1,   0,
								  0,   0,   0,   0,   0,  -1;
			}
			// this one is simple

			r_t_jacobian1.setIdentity();
			// this one is more simple

			//r_t_jacobian0 = -r_t_jacobian0; // sign matters?
			//r_t_jacobian1 = -r_t_jacobian1; // sign matters?
#else // 1
			{
				double dt = f_delta_time; // just rename
				double x = v_cur(0), y = v_cur(1), z = v_cur(2), vx = v_cur(3), vy = v_cur(4), vz = v_cur(5);
				double prevx = v_prev(0), prevy = v_prev(1), prevz = v_prev(2),
					prevvx = v_prev(3), prevvy = v_prev(4), prevvz = v_prev(5);
				double J0[6][6], J1[6][6];
				J0[0][0] = -2.0*x+2.0*prevx+2.0*dt*prevvx;
				J0[0][1] = 0.0;      J0[0][2] = 0.0;      J0[0][3] = -2.0*(x-prevx-dt*prevvx)*dt;
				J0[0][4] = 0.0;      J0[0][5] = 0.0;      J0[1][0] = 0.0;
				J0[1][1] = -2.0*y+2.0*prevy+2.0*dt*prevvy;      J0[1][2] = 0.0;      J0[1][3] = 0.0;
				J0[1][4] = -2.0*(y-prevy-dt*prevvy)*dt;      J0[1][5] = 0.0;
				J0[2][0] = 0.0;      J0[2][1] = 0.0;      J0[2][2] = -2.0*z+2.0*prevz+2.0*dt*prevvz;
				J0[2][3] = 0.0;      J0[2][4] = 0.0;      J0[2][5] = -2.0*(z-prevz-dt*prevvz)*dt;
				J0[3][0] = 0.0;      J0[3][1] = 0.0;      J0[3][2] = 0.0;      J0[3][3] = -2.0*vx+2.0*prevvx;
				J0[3][4] = 0.0;      J0[3][5] = 0.0;      J0[4][0] = 0.0;      J0[4][1] = 0.0;
				J0[4][2] = 0.0;      J0[4][3] = 0.0;      J0[4][4] = -2.0*vy+2.0*prevvy;
				J0[4][5] = 0.0;      J0[5][0] = 0.0;      J0[5][1] = 0.0;      J0[5][2] = 0.0;
				J0[5][3] = 0.0;      J0[5][4] = 0.0;      J0[5][5] = -2.0*vz+2.0*prevvz;

				J1[0][0] = 2.0*x-2.0*prevx-2.0*dt*prevvx;      J1[0][1] = 0.0;      J1[0][2] = 0.0;
				J1[0][3] = 0.0;      J1[0][4] = 0.0;      J1[0][5] = 0.0;      J1[1][0] = 0.0;
				J1[1][1] = 2.0*y-2.0*prevy-2.0*dt*prevvy;      J1[1][2] = 0.0;      J1[1][3] = 0.0;
				J1[1][4] = 0.0;      J1[1][5] = 0.0;      J1[2][0] = 0.0;      J1[2][1] = 0.0;
				J1[2][2] = 2.0*z-2.0*prevz-2.0*dt*prevvz;      J1[2][3] = 0.0;      J1[2][4] = 0.0;
				J1[2][5] = 0.0;      J1[3][0] = 0.0;      J1[3][1] = 0.0;      J1[3][2] = 0.0;
				J1[3][3] = 2.0*vx-2.0*prevvx;      J1[3][4] = 0.0;      J1[3][5] = 0.0;
				J1[4][0] = 0.0;      J1[4][1] = 0.0;      J1[4][2] = 0.0;      J1[4][3] = 0.0;
				J1[4][4] = 2.0*vy-2.0*prevvy;      J1[4][5] = 0.0;      J1[5][0] = 0.0;
				J1[5][1] = 0.0;      J1[5][2] = 0.0;      J1[5][3] = 0.0;      J1[5][4] = 0.0;
				J1[5][5] = 2.0*vz-2.0*prevvz;

				for(int r = 0; r < 6; ++ r) {
					for(int c = 0; c < 6; ++ c)
						r_t_jacobian0(r, c) = J0[r][c];
				}
				for(int r = 0; r < 6; ++ r) {
					for(int c = 0; c < 6; ++ c)
						r_t_jacobian1(r, c) = J1[r][c];
				}
			}
			// jacobians for squared residual (error = error.array() * error.array())
#endif // 1

			Eigen::Matrix<double, 6, 1> v_expectation, v_error;
			v_expectation.head<3>() = v_prev.head<3>() + v_prev.tail<3>() * f_delta_time; // simple Newtonian motion
			v_expectation.tail<3>() = v_prev.tail<3>(); // constant velocity
			v_error = -(v_cur - v_expectation); // meh (reversed sign makes nicer jacobians: specifying negative identity is hard)
			//v_error = -v_error; // sign matters?

			r_v_expectation.template head<n_residual_dim>() = v_expectation.template head<n_residual_dim>();
			r_v_error.template head<n_residual_dim>() = v_error.template head<n_residual_dim>();
			// hack - just to avoid writing specialized template
		}
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 1> v_expectation, v_error;
		const Eigen::Matrix<double, 6, 1> &v_prev = _TyBase::m_p_vertex0->r_v_State();
		const Eigen::Matrix<double, 6, 1> &v_cur = _TyBase::m_p_vertex1->r_v_State();
		double f_delta_time = _TyBase::m_v_measurement(0);
		v_expectation.head<3>() = v_prev.head<3>() + v_prev.tail<3>() * f_delta_time; // simple Newtonian motion
		v_expectation.tail<3>() = v_prev.tail<3>(); // constant velocity
		v_error = v_cur - v_expectation; // sign does not matter here
		if(b_1D_residual)
			v_error(0) = v_error.norm(); // hack - just to avoid writing specialized template
		// calculates the expectation, error and the jacobians

		return (v_error.head<n_residual_dim>().transpose() *
			_TyBase::m_t_sigma_inv).dot(v_error.head<n_residual_dim>()); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/** \addtogroup parser
 *	@{
 */

/**
 *	@brief vertex traits for BA solver
 */
template <class CParsedStructure>
class CROCVVertexTraits {
public:
	typedef CFailOnVertexType _TyVertex; /**< @brief it should fail on unknown vertex types */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "unknown vertex type occurred";
	}
};

/**
 *	@brief vertex traits for ROCV solver
 */
template <>
class CROCVVertexTraits<CParserBase::TVertex3D> {
public:
	typedef CVertexPositionVelocity3D _TyVertex; /**< @brief initialize pose vertex */
};

/**
 *	@brief vertex traits for ROCV solver
 */
template <>
class CROCVVertexTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexLandmark3D _TyVertex; /**< @brief initialize landmark vertex */
};

/**
 *	@brief vertex traits for ROCV solver
 */
template <>
class CROCVVertexTraits<CParserBase::TVertex3D_Reference> {
public:
	typedef CIgnoreVertexType _TyVertex; /**< @brief ignore pose vertex ground truth */
};

/**
 *	@brief edge traits for ROCV solver
 */
template <class CParsedStructure>
class CROCVEdgeTraits {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief it should fail on unknown edge types */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "unknown edge type occurred 2";
	}
};

/**
 *	@brief edge traits for ROCV solver
 */
template <>
class CROCVEdgeTraits<CParserBase::TROCV_DeltaTimeEdge> {
public:
	typedef CEdgeConstVelocity3D<> _TyEdge; /**< @brief initialize the constant velocity edge */
};

/**
 *	@brief edge traits for ROCV solver
 */
template <>
class CROCVEdgeTraits<CParserBase::TROCV_RangeEdge> {
public:
	typedef CEdgePosVel_Landmark3D<> _TyEdge; /**< @brief initialize the range measurement edge */
};

/**
 *	@brief edge traits for ROCV solver
 */
template <>
class CROCVEdgeTraits<CParserBase::TUnaryFactor3D> {
public:
	typedef CEdgeLandmark3DPrior _TyEdge; /**< @brief initialize the range measurement edge */
};

/** @} */ // end of group

#endif // __ROCV_TYPES_INCLUDED
