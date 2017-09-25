/*
								+-----------------------------------+
								|                                   |
								|       ***  SE(2) types  ***       |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|            SE2_Types.h            |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SE2_PRIMITIVE_TYPES_INCLUDED
#define __SE2_PRIMITIVE_TYPES_INCLUDED

/**
 *	@file include/slam/SE2_Types.h
 *	@brief SE(2) primitive types
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 *
 *	@date 2012-09-12
 *
 *	Fixed linux build issues (vertex constructors took reference to Eigen::Vector, instead
 *	of const reference, g++ was unable to pass reference to a temporary object instance
 *	created on stack).
 *
 *	Added support for aligned edge and vertex types for the use of SSE2.
 *
 *	t_odo Write base edge template, that would inherit from CBaseEdge, and would implement
 *		all the functionality, required by the linear solvers. The edge itself should only
 *		contain a single function which calculates jacobians and expectation.
 *	t_odo Do the same for CBaseVertex.
 *
 */

#include "slam/BaseTypes.h"
#include "slam/2DSolverBase.h"
#include "slam/Parser.h" // parsed types passed to constructors

/** \addtogroup se2
 *	@{
 */

/**
 *	@brief SE(2) pose vertex type
 */
class CVertexPose2D : public CBaseVertexImpl<CVertexPose2D, 3> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPose2D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPose2D(const Eigen::Vector3d &r_v_state)
		:CBaseVertexImpl<CVertexPose2D, 3>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<3>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		m_v_state(2) = C2DJacobians::f_ClampAngle_2Pi(m_v_state(2)); // clamp angle
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<3>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
		m_v_state(2) = C2DJacobians::f_ClampAngle_2Pi(m_v_state(2)); // clamp angle
	}
};

/**
 *	@brief SE(2) landmark vertex type
 */
class CVertexLandmark2D : public CBaseVertexImpl<CVertexLandmark2D, 2> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexLandmark2D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexLandmark2D(const Eigen::Vector2d &r_v_state)
		:CBaseVertexImpl<CVertexLandmark2D, 2>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<2>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		// todo - this is probably wrong, m_v_state(1) is angular, yet no clamp_angle_2pi() is here
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<2>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
		// todo - this is probably wrong, m_v_state(1) is angular, yet no clamp_angle_2pi() is here
	}
};

#if 0 // vertex traits unused now

/**
 *	@brief token for 2D pose vertex
 */
class _vertex_token_pose2d {} vertex_token_pose2d; /**< token for 2D pose vertex */

/**
 *	@brief token for 2D landmark vertex
 */
class _vertex_token_landmark2d {} vertex_token_landmark2d; /**< token for 2D landmark vertex */

/**
 *	@brief token for 3D pose vertex
 */
class _vertex_token_pose3d {} vertex_token_pose3d; /**< token for 3D pose vertex */

/**
 *	@brief token for 3D landmark vertex
 */
class _vertex_token_landmark3d {} vertex_token_landmark3d; /**< token for 3D landmark vertex */
// vertex type tokens

/**
 *	@brief vertex type traits
 */
template <class CTypeToken>
class CVertexTypeTraits {};

/**
 *	@brief vertex type traits (specialization for _vertex_token_pose2d)
 */
template <>
class CVertexTypeTraits<_vertex_token_pose2d> {
public:
	typedef CVertexPose2D _TyVertex; /**< @brief vertex type associated with the token */
};

/**
 *	@brief vertex type traits (specialization for _vertex_token_landmark2d)
 */
template <>
class CVertexTypeTraits<_vertex_token_landmark2d> {
public:
	typedef CVertexLandmark2D _TyVertex; /**< @brief vertex type associated with the token */
};

#endif // 0

/**
 *	@brief SE(2) pose-pose edge
 */
class CEdgePose2D : public CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D), 3> {
public:
	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and an XYT edge.
	 */
	class CRelative_to_Absolute_XYT_Initializer {
	protected:
		const Eigen::Vector3d &m_r_v_pose1; /**< @brief the first vertex */
		const Eigen::Vector3d &m_r_v_edge; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_edge is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_XYT_Initializer(const Eigen::Vector3d &r_v_vertex1,
			const Eigen::Vector3d &r_v_edge)
			:m_r_v_pose1(r_v_vertex1), m_r_v_edge(r_v_edge)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexPose2D() const
		{
			Eigen::Vector3d v_pose2;
			C2DJacobians::Relative_to_Absolute(m_r_v_pose1, m_r_v_edge, v_pose2);
			return CVertexPose2D(v_pose2);
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePose2D()
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
	CEdgePose2D(const CParserBase::TEdge2D &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D), 3>(r_t_edge.m_n_node_0,
		r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(r_t_edge.m_n_node_0,
			CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose2D>(r_t_edge.m_n_node_1,
			CRelative_to_Absolute_XYT_Initializer(m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get vertices (initialize if required)
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node
	 *	@param[in] v_delta is vector of delta x position, delta y-position and delta-theta
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePose2D(size_t n_node0, size_t n_node1, const Eigen::Vector3d &v_delta,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D), 3>(n_node0,
		n_node1, v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(n_node0,
			CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose2D>(n_node1,
			CRelative_to_Absolute_XYT_Initializer(m_p_vertex0->r_v_State(), v_delta));
		// get vertices (initialize if required)
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TEdge2D &r_t_edge)
	{
		CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D),
			3>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief updates the edge with new measurement
	 *
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 */
	inline void Update(const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv) // note that this is only required by MSVC which somehow does not see CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D), 3>::Update()
	{
		CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D),
			3>::Update(r_v_measurement, r_t_sigma_inv);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix3d &r_t_jacobian0,
		Eigen::Matrix3d &r_t_jacobian1, Eigen::Vector3d &r_v_expectation,
		Eigen::Vector3d &r_v_error) const
	{
		C2DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		r_v_error = m_v_measurement - r_v_expectation;
		r_v_error(2) = C2DJacobians::f_ClampAngularError_2Pi(r_v_error(2));
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix3d p_jacobi[2];
		Eigen::Vector3d v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi[0], p_jacobi[1], v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		//return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
		return v_error.dot(m_t_sigma_inv * v_error);
	}
};

/**
 *	@brief SE(2) pose-landmark edge
 */
class CEdgePoseLandmark2D : public CBaseEdgeImpl<CEdgePoseLandmark2D,
	MakeTypelist(CVertexPose2D, CVertexLandmark2D), 2> {
public:
	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and a range-bearing edge.
	 */
	class CRelative_to_Absolute_RangeBearing_Initializer {
	protected:
		const Eigen::Vector3d &m_r_v_pose1; /**< @brief the first vertex */
		const Eigen::Vector2d &m_r_v_edge; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_edge is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_RangeBearing_Initializer(const Eigen::Vector3d &r_v_vertex1,
			const Eigen::Vector2d &r_v_edge)
			:m_r_v_pose1(r_v_vertex1), m_r_v_edge(r_v_edge)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexLandmark2D() const
		{
			Eigen::Vector3d pose1(m_r_v_pose1(0), m_r_v_pose1(1), m_r_v_pose1(2));
			Eigen::Vector3d relPose(m_r_v_edge(0), 0, m_r_v_edge(1)); // x = range, y = 0, theta = bearing
			Eigen::Vector3d	v_result;
			C2DJacobians::Relative_to_Absolute(pose1, relPose, v_result);
			return CVertexLandmark2D(Eigen::Vector2d(v_result.segment<2>(0).norm(), v_result(2))); // range is length of displacement, bearing is theta
		}
	};

	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and a XY edge.
	 */
	class CRelative_to_Absolute_XY_Initializer {
	protected:
		const Eigen::Vector3d &m_r_v_pose1; /**< @brief the first vertex */
		const Eigen::Vector2d &m_r_v_edge; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_edge is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_XY_Initializer(const Eigen::Vector3d &r_v_vertex1,
			const Eigen::Vector2d &r_v_edge)
			:m_r_v_pose1(r_v_vertex1), m_r_v_edge(r_v_edge)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexLandmark2D() const
		{
			Eigen::Vector3d pose1(m_r_v_pose1(0), m_r_v_pose1(1), m_r_v_pose1(2));
			Eigen::Vector3d relPose(m_r_v_edge(0), m_r_v_edge(1), 0);
			Eigen::Vector3d	v_result;
			C2DJacobians::Relative_to_Absolute(pose1, relPose, v_result);
			return CVertexLandmark2D(Eigen::Vector2d(v_result.segment<2>(0)));
		}
	};

protected:
	//_TyMatrixAlign m_t_sigma_inv_xy; /**< @brief inverse sigma, in euclidean coordinate space */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePoseLandmark2D()
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
	CEdgePoseLandmark2D(const CParserBase::TLandmark2D_RB &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePoseLandmark2D, MakeTypelist(CVertexPose2D, CVertexLandmark2D),
		2>(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, r_t_edge.m_v_delta,
		r_t_edge.m_t_inv_sigma)
	{
		if(r_system.r_Vertex_Pool().n_Size() > size_t(r_t_edge.m_n_node_0) &&
		   r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 2)
			std::swap(m_p_vertex_id[0], m_p_vertex_id[1]);
		else if(r_system.r_Vertex_Pool().n_Size() > size_t(r_t_edge.m_n_node_1) &&
		   r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 3)
			std::swap(m_p_vertex_id[0], m_p_vertex_id[1]);
		// try to detect inverted edges (those where landmark is not second),
		// and invert them if needed

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(r_t_edge.m_n_node_0,
			CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark2D>(r_t_edge.m_n_node_1,
			CRelative_to_Absolute_RangeBearing_Initializer(m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get references to the vertices, initialize the vertices, if necessary
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 2);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

		//m_t_sigma_inv_xy = r_t_edge.m_t_inv_sigma;
		// it's in different coordinate space, save it
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
	CEdgePoseLandmark2D(const CParserBase::TLandmark2D_XY &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePoseLandmark2D, MakeTypelist(CVertexPose2D, CVertexLandmark2D),
		2>(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, v_ToPolar(r_t_edge.m_v_delta),
		t_ToPolar(r_t_edge.m_t_inv_sigma))
	{
		if(r_system.r_Vertex_Pool().n_Size() > size_t(r_t_edge.m_n_node_0) &&
		   r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 2)
			std::swap(m_p_vertex_id[0], m_p_vertex_id[1]);
		else if(r_system.r_Vertex_Pool().n_Size() > size_t(r_t_edge.m_n_node_1) &&
		   r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 3)
			std::swap(m_p_vertex_id[0], m_p_vertex_id[1]);
		// try to detect inverted edges (those where landmark is not second),
		// and invert them if needed

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(r_t_edge.m_n_node_0,
			CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark2D>(r_t_edge.m_n_node_1,
			CRelative_to_Absolute_XY_Initializer(m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get references to the vertices, initialize the vertices, if necessary
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 2);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

		//m_t_sigma_inv_xy = r_t_edge.m_t_inv_sigma;
		// it's in different coordinate space, save it
	}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node
	 *	@param[in] r_v_delta is vector of range and bearing
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgePoseLandmark2D(size_t n_node0, size_t n_node1, const Eigen::Vector2d &r_v_delta,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:CBaseEdgeImpl<CEdgePoseLandmark2D, MakeTypelist(CVertexPose2D, CVertexLandmark2D),
		2>(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		if(r_system.r_Vertex_Pool().n_Size() > size_t(n_node0) &&
		   r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 2)
			std::swap(m_p_vertex_id[0], m_p_vertex_id[1]);
		else if(r_system.r_Vertex_Pool().n_Size() > size_t(n_node1) &&
		   r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 3)
			std::swap(m_p_vertex_id[0], m_p_vertex_id[1]);
		// try to detect inverted edges (those where landmark is not second),
		// and invert them if needed

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(n_node0,
			CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark2D>(n_node1,
			CRelative_to_Absolute_RangeBearing_Initializer(m_p_vertex0->r_v_State(), r_v_delta));
		// get references to the vertices, initialize the vertices, if necessary
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 2);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

		//m_t_sigma_inv_xy = r_t_inv_sigma;
		// it's in different coordinate space, save it
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TLandmark2D_RB &r_t_edge)
	{
		CBaseEdgeImpl<CEdgePoseLandmark2D, MakeTypelist(CVertexPose2D, CVertexLandmark2D),
			2>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian0,
		Eigen::Matrix2d &r_t_jacobian1, Eigen::Vector2d &r_v_expectation,
		Eigen::Vector2d &r_v_error) const
	{
		C2DJacobians::Observation2D_RangeBearing(
			m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
			r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians (possibly re-calculates, if running A-SLAM)

		r_v_error = m_v_measurement - r_v_expectation;
		r_v_error(1) = C2DJacobians::f_ClampAngularError_2Pi(r_v_error(1));
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 3> t_jacobian0;
		Eigen::Matrix2d t_jacobian1;
		Eigen::Vector2d v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(t_jacobian0, t_jacobian1, v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return v_error.dot(m_t_sigma_inv * v_error);
		// have sigma inv in RB

		//Eigen::Vector2d v_error_xy(v_error(0) * cos(v_error(1)), v_error(0) * sin(v_error(1)));
		// how about converting error to the same coordinate space sigma inv is in

		//return (v_error_xy.transpose() * m_t_sigma_inv_xy).dot(v_error_xy); // ||z_i - h_i(O_i)||^2 lambda_i
		//return v_error_xy.dot(m_t_sigma_inv_xy * v_error_xy);
	}

	/**
	 *	@brief converts vector from Euclidean to polar coordinates
	 *	@param[in] r_v_vec is a vector in Euclidean coordinates
	 *	@return Returns the given vector, converted to polar coordinates.
	 */
	static inline Eigen::Vector2d v_ToPolar(const Eigen::Vector2d &r_v_vec)
	{
		return Eigen::Vector2d(r_v_vec.norm(),
			C2DJacobians::f_ClampAngle_2Pi(atan2(r_v_vec(1), r_v_vec(0))));
	}

	/**
	 *	@brief converts covariance matrix from Euclidean to polar coordinates
	 *	@param[in] r_t_mat is a covariance matrix in Euclidean coordinates
	 *	@return Returns the given matrix, converted to polar coordinates.
	 */
	static inline Eigen::Matrix2d t_ToPolar(const Eigen::Matrix2d &UNUSED(r_t_mat))
	{
		return Eigen::Matrix2d::Identity();
	}
};

/** @} */ // end of group

/** \addtogroup parser
 *	@{
 */

/**
 *	@brief edge traits for SE(2) solver
 */
template <class CParsedStructure>
class CSE2EdgeTraits {
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
 *	@brief edge traits for SE(2) solver (specialized for CParser::TEdge2D)
 */
template <>
class CSE2EdgeTraits<CParserBase::TEdge2D> {
public:
	typedef CEdgePose2D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(2) solver (specialized for CParser::TLandmark2D)
 */
template <>
class CSE2EdgeTraits<CParserBase::TLandmark2D_XY> {
public:
	typedef CEdgePoseLandmark2D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(2) solver (specialized for CParser::TLandmark2D)
 */
template <>
class CSE2EdgeTraits<CParserBase::TLandmark2D_RB> {
public:
	typedef CEdgePoseLandmark2D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(2) pose-only solver (specialized for CParser::TVertex2D)
 */
template <>
class CSE2EdgeTraits<CParserBase::TVertex2D> {
public:
	typedef CIgnoreEdgeType _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(2) pose-only solver
 */
template <class CParsedStructure>
class CSE2OnlyPoseEdgeTraits {
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
 *	@brief edge traits for SE(2) pose-only solver (specialized for CParser::TEdge2D)
 */
template <>
class CSE2OnlyPoseEdgeTraits<CParserBase::TEdge2D> {
public:
	typedef CEdgePose2D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(2) pose-only solver (specialized for CParser::TVertex2D)
 */
template <>
class CSE2OnlyPoseEdgeTraits<CParserBase::TVertex2D> {
public:
	typedef CIgnoreEdgeType _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(2) pose-only solver (specialized for CParser::TLandmark2D)
 */
template <>
class CSE2OnlyPoseEdgeTraits<CParserBase::TLandmark2D_XY> {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief the edge type to construct from the parsed type */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "landmark edges not permitted in pose-only solver";
	}
};

/**
 *	@brief edge traits for SE(2) pose-only solver (specialized for CParser::TLandmark2D)
 */
template <>
class CSE2OnlyPoseEdgeTraits<CParserBase::TLandmark2D_RB> {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief the edge type to construct from the parsed type */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "landmark edges not permitted in pose-only solver";
	}
};

/** @} */ // end of group

#endif // !__SE2_PRIMITIVE_TYPES_INCLUDED
