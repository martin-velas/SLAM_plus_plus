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
#include "slam/Sim3SolverBase.h"
#include "slam/Parser.h" // parsed types passed to constructors
#include "slam/RobustUtils.h"

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
 *	@brief SE(3) plane vertex type
 */
class CVertexPlane3D : public CBaseVertexImpl<CVertexPlane3D, 4> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPlane3D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPlane3D(const Eigen::Vector4d &r_v_state)
		:CBaseVertexImpl<CVertexPlane3D, 4>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		//m_v_state += r_v_delta.segment<4>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		//double d = r_v_delta.segment<4>(m_n_order)(3);
		Eigen::Vector3d vec = r_v_delta.segment<4>(m_n_order).head(3);

		m_v_state.head(3) = C3DJacobians::Operator_rot(vec) * m_v_state.head(3);
		m_v_state.tail(1) += r_v_delta.segment<4>(m_n_order).tail(1);
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		Operator_Plus(-r_v_delta.segment<4>(m_n_order));
		//m_v_state -= r_v_delta.segment<4>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
	}
};

/**
 *	@brief polynomial vertex type
 */
class CVertexPolynomial4 : public CBaseVertexImpl<CVertexPolynomial4, 4> {
public:
	int n_affected_dof;
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPolynomial4():
	n_affected_dof(0)
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPolynomial4(const Eigen::Vector4d &r_v_state, int n_dof = 0)
		:CBaseVertexImpl<CVertexPolynomial4, 4>(r_v_state),
		 n_affected_dof(n_dof)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<4>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		m_v_state.normalize();	// at 1, we want 1
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<4>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
		m_v_state.normalize();	// at 1, we want 1
	}
};

/**
 *	@brief polynomial vertex type
 */
class CVertexPolynomial4_6D : public CBaseVertexImpl<CVertexPolynomial4_6D, 24> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPolynomial4_6D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPolynomial4_6D(const Eigen::Matrix<double, 24, 1, Eigen::DontAlign> &r_v_state)
		:CBaseVertexImpl<CVertexPolynomial4_6D, 24>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<24>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		for(int a = 0; a < 6; ++a)
			m_v_state.segment<4>(a * 4) = m_v_state.segment<4>(a * 4) / m_v_state.segment<4>(a * 4).sum();
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<24>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
		for(int a = 0; a < 6; ++a)
			m_v_state.segment<4>(a * 4) = m_v_state.segment<4>(a * 4) / m_v_state.segment<4>(a * 4).sum();
	}
};

/**
 *	@brief polynomial vertex type
 */
class CVertexBezier_3D : public CBaseVertexImpl<CVertexBezier_3D, 6> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexBezier_3D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexBezier_3D(const Eigen::Matrix<double, 6, 1, Eigen::DontAlign> &r_v_state)
		:CBaseVertexImpl<CVertexBezier_3D, 6>(r_v_state)
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
 *	@brief polynomial vertex type
 */
class CVertexPolynomial3_6D : public CBaseVertexImpl<CVertexPolynomial3_6D, 18> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPolynomial3_6D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPolynomial3_6D(const Eigen::Matrix<double, 18, 1, Eigen::DontAlign> &r_v_state)
		:CBaseVertexImpl<CVertexPolynomial3_6D, 18>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<18>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		for(int a = 0; a < 6; ++a)
			m_v_state.segment<3>(a * 3) = m_v_state.segment<3>(a * 3) / m_v_state.segment<3>(a * 3).sum();
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<18>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
		for(int a = 0; a < 6; ++a)
			m_v_state.segment<3>(a * 3) = m_v_state.segment<3>(a * 3) / m_v_state.segment<3>(a * 3).sum();
	}
};

/**
 *	@brief polynomial vertex type
 */
class CVertexPolynomial5_6D : public CBaseVertexImpl<CVertexPolynomial5_6D, 30> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPolynomial5_6D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPolynomial5_6D(const Eigen::Matrix<double, 30, 1, Eigen::DontAlign> &r_v_state)
		:CBaseVertexImpl<CVertexPolynomial5_6D, 30>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<30>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		for(int a = 0; a < 6; ++a)
			m_v_state.segment<5>(a * 5) = m_v_state.segment<5>(a * 5) / m_v_state.segment<5>(a * 5).sum();
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		m_v_state -= r_v_delta.segment<30>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
		for(int a = 0; a < 6; ++a)
			m_v_state.segment<5>(a * 5) = m_v_state.segment<5>(a * 5) / m_v_state.segment<5>(a * 5).sum();
	}
};

/**
 *	@brief polynomial vertex type
 */
class CVertexPolynomial35_6D : public CBaseVertexImpl<CVertexPolynomial35_6D, 24> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexPolynomial35_6D()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexPolynomial35_6D(const Eigen::Matrix<double, 24, 1, Eigen::DontAlign> &r_v_state)
		:CBaseVertexImpl<CVertexPolynomial35_6D, 24>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<24>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
		for(int a = 0; a < 3; ++a)
		{
			m_v_state.segment<3>(a * 3) = m_v_state.segment<3>(a * 3) / m_v_state.segment<3>(a * 3).sum();
			m_v_state.segment<5>(9 + a * 5) = m_v_state.segment<5>(9 + a * 5) / m_v_state.segment<5>(9 + a * 5).sum();
		}
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		Operator_Plus(-r_v_delta);
	}
};

/**
 *	@brief SE(3) landmark vertex type
 */
class CVertexLandmark3DInverse : public CBaseVertexImpl<CVertexLandmark3DInverse, 3> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CVertexLandmark3DInverse()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexLandmark3DInverse(const Eigen::Vector3d &r_v_state)
		:CBaseVertexImpl<CVertexLandmark3DInverse, 3>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_v_state, r_v_delta.segment<3>(m_n_order), m_v_state); // delta is in XYZ
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
	inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_v_state, -r_v_delta.segment<3>(m_n_order), m_v_state); // delta is in XYZ
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
class CEdgePose3D : public CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 6, 6, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 100>, CHuberLossd>{
public:
	typedef CBaseEdgeImpl<CEdgePose3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 6, 6, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */
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
		:_TyBase(r_t_edge.m_n_node_0,
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
		:_TyBase(n_node0,
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
		_TyBase::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 6, 1> &r_v_delta, const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
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
		//std::cout << r_v_error.transpose() << std::endl;
		//std::cout << m_v_measurement.transpose() << " | " << r_v_expectation.transpose() << " | " << ((r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error)) << std::endl;
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
		//std::cout << "ln: " << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << ": " << (r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error) << std::endl;
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

/**
 *	@brief SE(3) pose-pose edge
 */
class CEdgePose3DSelf : public CBaseEdgeImpl<CEdgePose3DSelf, MakeTypelist(CVertexPose3D), 6, 6/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 100>, CHuberLossd>*/{
public:
	typedef CBaseEdgeImpl<CEdgePose3DSelf, MakeTypelist(CVertexPose3D), 6, 6/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base edge type */
public:
	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and an XYT edge.
	 */
	class C_Absolute_Initializer {
	protected:
		const Eigen::Matrix<double, 6, 1> &m_r_v_pose1; /**< @brief the first vertex */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_delta is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline C_Absolute_Initializer(const Eigen::Matrix<double, 6, 1> &r_v_vertex1) // just change the types, same as above
			:m_r_v_pose1(r_v_vertex1)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexPose3D() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
		{
			return CVertexPose3D(m_r_v_pose1);
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePose3DSelf()
	{}

	template <class CSystem>
	CEdgePose3DSelf(const CParserBase::TEdge3DSelf &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, C_Absolute_Initializer(r_t_edge.m_v_delta));
		// get vertices (initialize if required)
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
	CEdgePose3DSelf(size_t n_node0, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, r_v_delta, r_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, C_Absolute_Initializer(r_v_delta));
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 6, 1> &r_v_delta, const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 6, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 6, 1> &r_v_expectation, Eigen::Matrix<double, 6, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), m_v_measurement, r_v_expectation);
		r_v_error = r_v_expectation;
		// calculates the expectation

		//std::cout << "SELF: " << m_p_vertex0->r_v_State().transpose() << " | " << m_v_measurement.transpose() << " >> " <<
		//		r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, m_v_measurement, d1);
			r_t_jacobian0.col(j) = -(d1 - r_v_expectation) * scalar;
			// we expect zero
		}
		// compute jacobians

		//std::cout << r_t_jacobian0 << std::endl << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 6> p_jacobi[2];
		Eigen::Matrix<double, 6, 1> v_expectation, v_error;
		Calculate_Jacobian_Expectation_Error(p_jacobi[0], v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

class CEdgePoseTernary3D : public CBaseEdgeImpl<CEdgePoseTernary3D, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D), 6, 6/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 100>, CHuberLossd>*/{
public:
	typedef CBaseEdgeImpl<CEdgePoseTernary3D, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D), 6, 6/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePoseTernary3D()
	{}

	template <class CSystem>
	CEdgePoseTernary3D(const CParserBase::TEdge3DTernary &r_t_edge, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, r_t_edge.m_n_node_2),
				r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_2, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
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
	CEdgePoseTernary3D(size_t n_node0, size_t n_node1, size_t n_node2, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2), r_v_delta, r_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 6, 1> &r_v_delta, const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
			Eigen::Matrix<double, 6, 1> &r_v_expectation,
			Eigen::Matrix<double, 6, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		C3DJacobians::Absolute_to_Relative(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(), r_v_expectation);
		C3DJacobians::Absolute_to_Relative(r_v_expectation, m_vertex_ptr.Get<2>()->r_v_State(), r_v_error);
		// calculates the expectation and error

		//std::cout << "TERN: " << r_v_expectation.transpose() << " | " << m_vertex_ptr.Get<2>()->r_v_State().transpose() << " >> " <<
		//				r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> &J0 = r_t_jacobian_tuple.Get<0>();
		Eigen::Matrix<double, 6, 6> &J1 = r_t_jacobian_tuple.Get<1>();
		Eigen::Matrix<double, 6, 6> &J2 = r_t_jacobian_tuple.Get<2>();

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1, d2;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, m_vertex_ptr.Get<1>()->r_v_State(), d1);
			C3DJacobians::Absolute_to_Relative(d1, m_vertex_ptr.Get<2>()->r_v_State(), d2);
			J0.col(j) = -(d2 - r_v_error) * scalar;
			// we expect zero
		}
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1, d2;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(m_vertex_ptr.Get<0>()->r_v_State(), p_delta, d1);
			C3DJacobians::Absolute_to_Relative(d1, m_vertex_ptr.Get<2>()->r_v_State(), d2);
			J1.col(j) = -(d2 - r_v_error) * scalar;
			// we expect zero
		}
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1, d2;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(), d1);
			C3DJacobians::Absolute_to_Relative(d1, p_delta, d2);
			J2.col(j) = -(d2 - r_v_error) * scalar;
			// we expect zero
		}
		// compute jacobians

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 1> v_expectation, v_error;
		C3DJacobians::Absolute_to_Relative(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(), v_expectation);
		C3DJacobians::Absolute_to_Relative(v_expectation, m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
class CEdgeNormal3D : public CBaseEdgeImpl<CEdgeNormal3D, MakeTypelist(CVertexPose3D), 1, 3/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 100>, CHuberLossd>*/{
public:
	typedef CBaseEdgeImpl<CEdgeNormal3D, MakeTypelist(CVertexPose3D), 1, 3/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeNormal3D()
	{}

	template <class CSystem>
	CEdgeNormal3D(const CParserBase::TEdge3DNormal &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	/*template <class CSystem>
	CEdgeNormal3D(const CParserBase::TEdge3DSelf &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, C_Absolute_Initializer(r_t_edge.m_v_delta));
		// get vertices (initialize if required)
	}*/

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
	CEdgeNormal3D(size_t n_node0, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 3, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 1, 1> &r_v_expectation, Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		Eigen::Matrix3d R = C3DJacobians::Operator_rot(m_p_vertex0->r_v_State().tail(3));
		Eigen::Vector3d v = R * m_v_measurement.normalized(); // this should be close to [0, 1, 0]
		//r_v_error(0) = 1.0 - v.transpose() * Eigen::Vector3d(0, 1, 0);
		r_v_error(0) = acos( (v.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
		// transform measrement vector to world

		/*std::cout << R << std::endl;
		std::cout << m_v_measurement.normalized().transpose() << std::endl;
		std::cout << v.transpose() << " | " << ((v.transpose() * Eigen::Vector3d(0, 1, 0))(0)) << " | " <<  r_v_error(0) << std::endl;*/

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);

			R = C3DJacobians::Operator_rot(p_delta.tail(3));
			v = R * m_v_measurement.normalized(); // this should be close to [0, 1, 0]
			Eigen::Matrix<double, 1, 1> err;
			//err(0) = 1.0 - v.transpose() * Eigen::Vector3d(0, 1, 0);
			err(0) = acos( (v.transpose() * Eigen::Vector3d(0, 1, 0))(0) );

			r_t_jacobian0.col(j) = -(err - r_v_error) * scalar;
			// we expect zero
		}
		// compute jacobians

		//std::cout << r_t_jacobian0 << std::endl << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 6> p_jacobi[1];
		Eigen::Matrix<double, 1, 1> v_expectation, v_error;
		Calculate_Jacobian_Expectation_Error(p_jacobi[0], v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
class CEdgeNormal23D : public CBaseEdgeImpl<CEdgeNormal23D, MakeTypelist(CVertexPose3D, CVertexPose3D), 1, 3/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 100>, CHuberLossd>*/{
public:
	typedef CBaseEdgeImpl<CEdgeNormal23D, MakeTypelist(CVertexPose3D, CVertexPose3D), 1, 3/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeNormal23D()
	{}

	/*template <class CSystem>
	CEdgeNormal3D(const CParserBase::TEdge3DNormal &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}*/

	/*template <class CSystem>
	CEdgeNormal3D(const CParserBase::TEdge3DSelf &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, C_Absolute_Initializer(r_t_edge.m_v_delta));
		// get vertices (initialize if required)
	}*/

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
	CEdgeNormal23D(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 3, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
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
		Eigen::Matrix<double, 1, 6> &r_t_jacobian1, Eigen::Matrix<double, 1, 1> &r_v_expectation,
		Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		Eigen::Matrix4d Rt0 = Eigen::Matrix4d::Identity();
		Rt0.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(m_p_vertex0->r_v_State().tail(3));
		Rt0.block(0, 3, 3, 1) = m_p_vertex0->r_v_State().head(3);

		Eigen::Matrix4d Rt1 = Eigen::Matrix4d::Identity();
		Rt1.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(m_p_vertex1->r_v_State().tail(3));
		Rt1.block(0, 3, 3, 1) = m_p_vertex1->r_v_State().head(3);

		Eigen::Matrix4d R1w = Rt0.inverse() * Rt1;

		Eigen::Vector3d v = R1w.block(0, 0, 3, 3) * m_v_measurement.normalized(); // this should be close to [0, 1, 0]
		//r_v_error(0) = 1.0 - v.transpose() * Eigen::Vector3d(0, 1, 0);
		r_v_error(0) = acos( (v.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
		// transform measrement vector to world

		//std::cout << v.transpose() << " | " << ((v.transpose() * Eigen::Vector3d(0, 1, 0))(0)) << " | " <<  r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);

			Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
			Rt.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(p_delta.tail(3));
			Rt.block(0, 3, 3, 1) = p_delta.head(3);

			R1w = Rt.inverse() * Rt1;

			v = R1w.block(0, 0, 3, 3) * m_v_measurement.normalized(); // this should be close to [0, 1, 0]
			Eigen::Matrix<double, 1, 1> err;
			//err(0) = 1.0 - v.transpose() * Eigen::Vector3d(0, 1, 0);
			err(0) = acos( (v.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );

			r_t_jacobian0.col(j) = -(err - r_v_error) * scalar;
			// we expect zero
		}

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1;
			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);

			Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
			Rt.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(p_delta.tail(3));
			Rt.block(0, 3, 3, 1) = p_delta.head(3);

			R1w = Rt0.inverse() * Rt;

			v = R1w.block(0, 0, 3, 3) * m_v_measurement.normalized(); // this should be close to [0, 1, 0]
			Eigen::Matrix<double, 1, 1> err;
			//err(0) = 1.0 - v.transpose() * Eigen::Vector3d(0, 1, 0);
			err(0) = acos( (v.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );

			r_t_jacobian1.col(j) = -(err - r_v_error) * scalar;
			// we expect zero
		}
		// compute jacobians

		//std::cout << r_t_jacobian0 << std::endl << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 6> p_jacobi[2];
		Eigen::Matrix<double, 1, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi[0], p_jacobi[1], v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
class CEdgePosePoseNormal3D : public CBaseEdgeImpl<CEdgePosePoseNormal3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 8, 12/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 100>, CHuberLossd>*/{
public:
	typedef CBaseEdgeImpl<CEdgePosePoseNormal3D, MakeTypelist(CVertexPose3D, CVertexPose3D), 8, 12/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePosePoseNormal3D()
	{}

	/*template <class CSystem>
	CEdgeNormal3D(const CParserBase::TEdge3DNormal &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}*/

	/*template <class CSystem>
	CEdgeNormal3D(const CParserBase::TEdge3DSelf &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, C_Absolute_Initializer(r_t_edge.m_v_delta));
		// get vertices (initialize if required)
	}*/

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
	CEdgePosePoseNormal3D(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 12, 1> &r_v_delta,
		const Eigen::Matrix<double, 8, 8> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 6);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 12, 1> &r_v_delta, const Eigen::Matrix<double, 8, 8> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 8, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 8, 6> &r_t_jacobian1, Eigen::Matrix<double, 8, 1> &r_v_expectation,
		Eigen::Matrix<double, 8, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		Eigen::Matrix<double, 6, 1> tmp_expectation;
		Eigen::Matrix<double, 6, 1> tmp_error;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), tmp_expectation);
		C3DJacobians::Absolute_to_Relative(m_v_measurement.head(6), tmp_expectation, tmp_error);
		r_v_error.head(6) = tmp_error;

		Eigen::Matrix3d R0 = C3DJacobians::Operator_rot(m_p_vertex0->r_v_State().tail(3));
		Eigen::Vector3d v0 = R0 * m_v_measurement.tail(6).head(3).normalized(); // this should be close to [0, 1, 0]
		r_v_error(6) = -acos( (v0.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
		// compute normal error V0

		Eigen::Matrix3d R1 = C3DJacobians::Operator_rot(m_p_vertex1->r_v_State().tail(3));
		Eigen::Vector3d v1 = R1 * m_v_measurement.tail(3).normalized(); // this should be close to [0, 1, 0]
		r_v_error(7) = -acos( (v1.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
		// compute normal error V1

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1;
			Eigen::Matrix<double, 8, 1> error8;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);

			C3DJacobians::Absolute_to_Relative(p_delta, m_p_vertex1->r_v_State(), tmp_expectation);
			C3DJacobians::Absolute_to_Relative(m_v_measurement.head(6), tmp_expectation, tmp_error);
			error8.head(6) = tmp_error;
			// pos

			R0 = C3DJacobians::Operator_rot(p_delta.tail(3));
			Eigen::Vector3d v0 = R0 * m_v_measurement.tail(6).head(3).normalized(); // this should be close to [0, 1, 0]
			error8(6) = -acos( (v0.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
			// nor0

			R1 = C3DJacobians::Operator_rot(m_p_vertex1->r_v_State().tail(3));
			v1 = R1 * m_v_measurement.tail(3).normalized(); // this should be close to [0, 1, 0]
			error8(7) = -acos( (v1.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
			// nor1

			r_t_jacobian0.col(j) = -(error8 - r_v_error) * scalar;
		}
		// compute normal jacobian0

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta, d1;
			Eigen::Matrix<double, 8, 1> error8;
			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);

			C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), p_delta, tmp_expectation);
			C3DJacobians::Absolute_to_Relative(m_v_measurement.head(6), tmp_expectation, tmp_error);
			error8.head(6) = tmp_error;
			// pos

			R0 = C3DJacobians::Operator_rot(m_p_vertex0->r_v_State().tail(3));
			v0 = R0 * m_v_measurement.tail(6).head(3).normalized(); // this should be close to [0, 1, 0]
			error8(6) = -acos( (v0.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
			// nor0

			R1 = C3DJacobians::Operator_rot(p_delta.tail(3));
			v1 = R1 * m_v_measurement.tail(3).normalized(); // this should be close to [0, 1, 0]
			error8(7) = -acos( (v1.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
			// nor1

			r_t_jacobian1.col(j) = -(error8 - r_v_error) * scalar;
		}
		// compute normal jacobian0
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 1> tmp_expectation;
		Eigen::Matrix<double, 6, 1> tmp_error;
		Eigen::Matrix<double, 8, 1> v_error;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), tmp_expectation);
		C3DJacobians::Absolute_to_Relative(tmp_expectation, m_v_measurement.head(6), tmp_error);
		v_error.head(6) = tmp_error;

		Eigen::Matrix3d R0 = C3DJacobians::Operator_rot(m_p_vertex0->r_v_State().tail(3));
		Eigen::Vector3d v0 = R0 * m_v_measurement.tail(6).head(3).normalized(); // this should be close to [0, 1, 0]
		v_error(6) = acos( (v0.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
		// compute normal error V0

		Eigen::Matrix3d R1 = C3DJacobians::Operator_rot(m_p_vertex1->r_v_State().tail(3));
		Eigen::Vector3d v1 = R1 * m_v_measurement.tail(3).normalized(); // this should be close to [0, 1, 0]
		v_error(7) = acos( (v1.normalized().transpose() * Eigen::Vector3d(0, 1, 0))(0) );
		// compute normal error V1

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
//class CBSplineEdge : public CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 1, 1> {
//
//public:
//	typedef CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 1, 1> _TyBase; /**< @brief base class */
//
//public:
//	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this
//
//	/**
//	 *	@brief default constructor; has no effect
//	 */
//	inline CBSplineEdge()
//	{}
//
//	template <class CSystem>
//	CBSplineEdge(const CParserBase::TEdgeSpline3D &r_t_edge, CSystem &r_system)
//		:_TyBase(typename _TyBase::_TyVertexIndexTuple(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
//				r_t_edge.m_n_node_2, r_t_edge.m_n_node_3, r_t_edge.m_n_node_4),
//		r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
//	{
//		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_2, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_3, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_4, CInitializeNullVertex<CVertexPose3D>());
//		// get vertices (initialize if required)
//
//		//std::cout << r_t_edge.m_n_node_0 << " " << r_t_edge.m_n_node_1 << " " << r_t_edge.m_n_node_2 << " " <<
//		//		r_t_edge.m_n_node_3 << " " << r_t_edge.m_v_delta.transpose() << " " << r_t_edge.m_t_inv_sigma << std::endl;
//
//		/*std::cout << "? " << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << "? " << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << "? " << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << "? " << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/
//	}
//
//	/**
//	 *	@brief constructor; converts parsed edge to edge representation
//	 *
//	 *	@tparam CSystem is type of system where this edge is being stored
//	 *
//	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
//	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node
//	 *	@param[in] r_v_delta is measurement vector
//	 *	@param[in] r_t_inv_sigma is the information matrix
//	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
//	 */
//	template <class CSystem>
//	CBSplineEdge(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
//			const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system)
//		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4), r_v_delta, r_t_inv_sigma)
//	{
//		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node4, CInitializeNullVertex<CVertexPose3D>());
//		// get vertices (initialize if required)
//	}
//
//	/**
//	 *	@brief updates the edge with a new measurement
//	 *
//	 *	@param[in] r_v_delta is new measurement vector
//	 *	@param[in] r_t_inv_sigma is new information matrix
//	 */
//	inline void Update(const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
//	{
//		_TyBase::Update(r_v_delta, r_t_inv_sigma);
//	}
//
//	/**
//	 *	@brief calculates jacobians, expectation and error
//	 *
//	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
//	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
//	 *	@param[out] r_v_expectation is expecation vector
//	 *	@param[out] r_v_error is error vector
//	 */
//	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
//			Eigen::Matrix<double, 1, 1> &r_v_expectation, Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
//	{
//		// calculates the expectation and the jacobians
//		/*std::cout << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/
//
//		BSplineSE3 spline(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//				m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//		r_v_expectation = spline.bspline_error2(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State());
//		// expectation error
//
//		/*std::cout << "V0: " << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << "V1: " << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << "V2: " << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << "V3: " << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;
//		std::cout << "V4: " << m_vertex_ptr.Get<4>()->r_v_State().transpose() << std::endl;
//		std::cout << "033: " << spline.estimate(0.33).matrix() << std::endl;
//		std::cout << "066: " << spline.estimate(0.66).matrix() << std::endl;*/
//
//		//std::cout << "EXP: " << r_v_expectation << std::endl;
//
//		const double delta = 1e-3;	// smaller delta wont work ?? why? is the error function too ??
//		const double scalar = 1.0 / (delta);
//		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack
//		Eigen::Matrix<double, 6, 1> p_delta;
//
//		for(size_t j = 0; j < 6; ++j) // four 1x6 jacobians
//		{
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline0(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
//					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<0>().col(j) = (spline0.bspline_error2(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
//			// J0
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline1(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
//					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<1>().col(j) = (spline1.bspline_error2(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
//			// J1
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline2(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//					p_delta, m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<2>().col(j) = (spline2.bspline_error2(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
//			// J2
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline3(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//					m_vertex_ptr.Get<2>()->r_v_State(), p_delta);
//			r_t_jacobian_tuple.Get<3>().col(j) = (spline3.bspline_error2(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State()) - r_v_expectation) * scalar;
//			// J3
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<4>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline4(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<4>().col(j) = (spline4.bspline_error2(m_v_measurement(0), p_delta) - r_v_expectation) * scalar;
//			// J4
//		}
//		// error is given by spline function
//		/*std::cout << "J0: " << r_t_jacobian_tuple.Get<0>() << std::endl << std::endl;
//		std::cout << "J1: " << r_t_jacobian_tuple.Get<1>() << std::endl << std::endl;
//		std::cout << "J2: " << r_t_jacobian_tuple.Get<2>() << std::endl << std::endl;
//		std::cout << "J3: " << r_t_jacobian_tuple.Get<3>() << std::endl << std::endl;
//		std::cout << "J4: " << r_t_jacobian_tuple.Get<4>() << std::endl << std::endl;*/
//
//		r_v_error = -r_v_expectation;	// measurement should be zero
//		// calculates error (possibly re-calculates, if running A-SLAM)
//	}
//
//	/**
//	 *	@brief calculates \f$\chi^2\f$ error
//	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
//	 */
//	inline double f_Chi_Squared_Error() const
//	{
//		/*std::cout << "er " <<  m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << "er " <<  m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << "er " <<  m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << "er " <<  m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/
//
//		BSplineSE3 spline(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//			m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//		Eigen::Matrix<double, 1, 1> v_error = -spline.bspline_error2(m_v_measurement(0), m_vertex_ptr.Get<4>()->r_v_State());
//		// calculates the expectation, error and the jacobians
//
//		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
//	}
//};

/**
 *	@brief BSpline 5xpose edge
 */
class CBSplineEdge : public CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 6, 6> {

public:
	typedef CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 6, 6> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
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
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	/*inline void Update(const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
	{
		_TyBase::Update(r_v_delta, r_t_inv_sigma);
	}*/

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
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

		const double delta = 1e-3;	// smaller delta wont work ?? why? is the error function too ??
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

		r_v_error = - r_v_expectation;	// measurement should be zero
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
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

/**
 *	@brief SE(3) edge
 */
class CCollarLineEdge : public CBaseEdgeImpl<CCollarLineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D), 1/* we expect 0 */, 12> {
public:
	typedef CBaseEdgeImpl<CCollarLineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D), 1/* we expect 0 */, 12> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CCollarLineEdge()
	{}

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
	CCollarLineEdge(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 12, 1> &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline double CollarError(const Eigen::Vector6d &p0, const Eigen::Vector6d &p1) const
	{
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Rt0 = C3DJacobians::VecToPose(p0);
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Rt1 = C3DJacobians::VecToPose(p1);
		// poses of vertices

		Eigen::Vector4d tmp = Eigen::Vector4d::Ones();
		tmp.head(3) = m_v_measurement.segment(0, 3);
		Eigen::Vector3d X0 = (Rt0 * tmp).head(3);
		tmp.head(3) = m_v_measurement.segment(6, 3);
		Eigen::Vector3d X1 = (Rt1 * tmp).head(3);
		// positions to global

		Eigen::Vector3d O0 = Rt0.block(0, 0, 3, 3) * m_v_measurement.segment(3, 3);
		Eigen::Vector3d O1 = Rt1.block(0, 0, 3, 3) * m_v_measurement.segment(9, 3);
		// measurements to global

		// copmute nearest point on line segments
		Eigen::Vector3d w0 = X0 - X1;

		double a = (O0.transpose() * O0)(0);
		double b = (O0.transpose() * O1)(0);
		double c = (O1.transpose() * O1)(0);
		double d = (O0.transpose() * w0)(0);
		double e = (O1.transpose() * w0)(0);

		double denominator = a*c - b*b;
		double sc = (b*e - c*d) / denominator;
		double tc = (a*e - b*d) / denominator;

		Eigen::Vector3d vect = w0 + sc*O0 - tc*O1;

		// todo: penalize vectors outside of the line segment

		return vect.norm();
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
		Eigen::Matrix<double, 1, 6> &r_t_jacobian1, Eigen::Matrix<double, 1, 1> &r_v_expectation,
		Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation(0) = CollarError(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State());
		r_v_error(0) = r_v_expectation(0);
		// compute error

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> d1, p_delta;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			r_t_jacobian0(j) = (r_v_expectation(0) - CollarError(p_delta, m_p_vertex1->r_v_State())) * scalar;

			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);
			r_t_jacobian1(j) = (r_v_expectation(0) - CollarError(m_p_vertex0->r_v_State(), p_delta)) * scalar;
		}
		// compute jacobians

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 1> v_error;
		v_error(0) = CollarError(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State());
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneEdge_Local : public CBaseEdgeImpl<CPlaneEdge_Local, MakeTypelist(CVertexPlane3D), 2/* dist, angle */, 6, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneEdge_Local, MakeTypelist(CVertexPlane3D), 2, 6, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneEdge_Local()
	{}

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
	CPlaneEdge_Local(size_t n_node0, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &line) const
	{
		// plane is normal and distance
		// plane is local, c2w
		// line is Point + Vector
		// line is local

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(plane.head(3).transpose() * line.head(3) + plane(3)) / plane.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.tail(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((plane.head(3).transpose() * dir) / (plane.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		// todo: penalize vectors outside of the line segment

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 2, 4> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_p_vertex0->r_v_State(), m_v_measurement);
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "err " << r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			p_delta = m_p_vertex0->r_v_State() + Eps.col(j).head(4);
			r_t_jacobian0.col(j) = (r_v_expectation - PlaneError(p_delta, m_v_measurement)) * scalar;
		}
		// compute jacobians

		//std::cout << r_t_jacobian0 << std::endl;
		/*std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_p_vertex0->r_v_State(), m_v_measurement);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneEdge_Global : public CBaseEdgeImpl<CPlaneEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D), 2/* dist, angle */, 6, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D), 2, 6, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneEdge_Global()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane node
	 *	@param[in] n_node1 is (zero-based) index of the observing node
	 *	@param[in] n_node2 is (zero-based) index of the owner node
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneEdge_Global(size_t n_node0, size_t n_node1, size_t n_node2, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &line,
			const Eigen::Vector6d &observer, const Eigen::Vector6d &owner) const
	{
		// plane is normal and distance
		// plane is local in owner, c2w
		// line is Point + Vector
		// line is local in observer

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtObs = C3DJacobians::VecToPose(observer);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOwn = C3DJacobians::VecToPose(owner);		//c2w
		// poses of vertices

		Eigen::Vector4d pG = RtOwn.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtObs.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pL.head(3).transpose() * line.head(3) + pL(3)) / pL.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.tail(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pL.head(3).transpose() * dir) / (pL.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		// todo: penalize vectors outside of the line segment

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(),
				m_v_measurement, m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State());
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "err " << r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian_tuple.Get<1>();
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			p_delta = m_vertex_ptr.Get<0>()->r_v_State() + Eps.col(j).head(4);
			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_v_measurement, m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State())) * scalar;
		}
		// J0

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);
			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_v_measurement, p_delta,
					m_vertex_ptr.Get<2>()->r_v_State())) * scalar;
		}
		// J1

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_v_measurement,
					m_vertex_ptr.Get<1>()->r_v_State(),	p_delta)) * scalar;
		}
		// J2

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(),
				m_v_measurement, m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State());
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlanePolynomialEdge_Local : public CBaseEdgeImpl<CPlanePolynomialEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPolynomial4), 2/* dist, angle */, 10, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlanePolynomialEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPolynomial4), 2, 10, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlanePolynomialEdge_Local()
	{}

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
	CPlanePolynomialEdge_Local(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 10, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPolynomial4>(n_node1, CInitializeNullVertex<CVertexPolynomial4>());
		// get vertices (initialize if required)
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector4d &poly,
			const Eigen::Matrix<double, 10, 1> &line) const
	{
		// plane is normal and distance
		// plane is local, c2w
		// line is Point + Vector + phase
		// line is local
		// measurement depends on polynomial function

		double t = line(6);
		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t2*t2;
		double time = poly(0) * t + poly(1) * t2 + poly(2) * t3 + poly(3) * t4;
		double dt = time - t;
		double angle = (1 / (2 * M_PI)) * dt;
		Eigen::Matrix3d RG = C3DJacobians::Operator_rot(line.tail(3));
		// rotation to global space
		Eigen::Matrix3d R;
		R = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		  * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY())
		  * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
		// rotate measurement in local space
		// around Y
		Eigen::Vector3d p = RG * line.head(3);
		p = R * p;
		p = RG.inverse() * p;

		Eigen::Vector3d o = RG * line.segment(3, 3);
		o = R * o;
		o = RG.inverse() * o;

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(plane.head(3).transpose() * p/*line.head(3)*/ + plane(3)) / plane.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = o;/*line.tail(3) - line.head(3)*/;
		error(1) = /*asin(*/ ((plane.head(3).transpose() * dir) / (plane.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		// todo: penalize vectors outside of the line segment

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 4> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 4> &r_t_jacobian1,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_v_measurement);
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			p_delta = m_p_vertex0->r_v_State() + Eps.col(j).head(4);
			r_t_jacobian0.col(j) = (r_v_expectation - PlaneError(p_delta, m_p_vertex1->r_v_State(), m_v_measurement)) * scalar;

			p_delta = m_p_vertex1->r_v_State() + Eps.col(j).head(4);
			r_t_jacobian1.col(j) = (r_v_expectation - PlaneError(m_p_vertex0->r_v_State(), p_delta, m_v_measurement)) * scalar;
		}
		// compute jacobians

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_v_measurement);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlanePolynomialEdge_Global : public CBaseEdgeImpl<CPlanePolynomialEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPolynomial4, CVertexPose3D, CVertexPose3D), 2/* dist, angle */, 10, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlanePolynomialEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPolynomial4, CVertexPose3D, CVertexPose3D), 2, 10, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlanePolynomialEdge_Global()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane node
	 *	@param[in] n_node1 is (zero-based) index of the polynom
	 *	@param[in] n_node2 is (zero-based) index of the observing node
	 *	@param[in] n_node3 is (zero-based) index of the owner node
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlanePolynomialEdge_Global(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, const Eigen::Matrix<double, 10, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPolynomial4>(n_node1, CInitializeNullVertex<CVertexPolynomial4>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector4d &poly, const Eigen::Matrix<double, 10, 1> &line,
			const Eigen::Vector6d &observer, const Eigen::Vector6d &owner) const
	{
		// plane is normal and distance
		// plane is local in owner, c2w
		// line is Point + Vector
		// line is local in observer

		double t = line(6);
		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t2*t2;
		double time = poly(0) * t + poly(1) * t2 + poly(2) * t3 + poly(3) * t4;
		double dt = time - t;
		double angle = (1 / (2 * M_PI)) * dt;

		Eigen::Matrix3d RG = C3DJacobians::Operator_rot(line.tail(3));
		// rotation to global
		Eigen::Matrix3d R;
		R = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		  * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY())
		  * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
		// rotate measurement in local space
		// around Y
		Eigen::Vector3d p = RG * line.head(3);
		p = R * p;
		p = RG.inverse() * p;

		Eigen::Vector3d o = RG * line.segment(3, 3);
		o = R * o;
		o = RG.inverse() * o;

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtObs = C3DJacobians::VecToPose(observer);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOwn = C3DJacobians::VecToPose(owner);		//c2w
		// poses of vertices

		Eigen::Vector4d pG = RtOwn.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtObs.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pL.head(3).transpose() * p/*line.head(3)*/ + pL(3)) / pL.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = o/*line.tail(3) - line.head(3)*/;
		error(1) = /*asin(*/ ((pL.head(3).transpose() * dir) / (pL.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		// todo: penalize vectors outside of the line segment

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_v_measurement, m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();
		Eigen::Matrix<double, 2, 4> &J1 = r_t_jacobian_tuple.Get<1>();
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();
		Eigen::Matrix<double, 2, 6> &J3 = r_t_jacobian_tuple.Get<3>();

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			p_delta = m_vertex_ptr.Get<0>()->r_v_State() + Eps.col(j).head(4);
			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_vertex_ptr.Get<1>()->r_v_State(), m_v_measurement,
					m_vertex_ptr.Get<2>()->r_v_State(),	m_vertex_ptr.Get<3>()->r_v_State())) * scalar;
		}
		// J0

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			p_delta = m_vertex_ptr.Get<1>()->r_v_State() + Eps.col(j).head(4);
			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), p_delta, m_v_measurement,
					m_vertex_ptr.Get<2>()->r_v_State(),	m_vertex_ptr.Get<3>()->r_v_State())) * scalar;
		}
		// J1

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_v_measurement, p_delta, m_vertex_ptr.Get<3>()->r_v_State())) * scalar;
		}
		// J2

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j), p_delta);
			J3.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_v_measurement, m_vertex_ptr.Get<2>()->r_v_State(), p_delta)) * scalar;
		}
		// J3

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_v_measurement, m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetBezier3DEdge_Local : public CBaseEdgeImpl<CPlaneOffsetBezier3DEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexBezier_3D), 2/* dist, angle */, 7/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd>*/ {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetBezier3DEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexBezier_3D), 2, 7/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetBezier3DEdge_Local()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane
	 *	@param[in] n_node1 is (zero-based) index of the from
	 *	@param[in] n_node2 is (zero-based) index of the to
	 *	@param[in] n_node3 is (zero-based) index of the offset
	 *	@param[in] n_node4 is (zero-based) index of the polynome
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetBezier3DEdge_Local(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
			const Eigen::Matrix<double, 7, 1> &r_v_delta, const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexBezier_3D>(n_node4, CInitializeNullVertex<CVertexBezier_3D>());
		// get vertices (initialize if required)
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	inline void BezierPlus(const Eigen::VectorXd &poly, const Eigen::VectorXd &delta,
			Eigen::Matrix<double, 6, 1, Eigen::DontAlign> &out) const // "smart" plus
	{
		out = poly + delta; // pick part of the delta vector, belonging to this vertex, apply +
	}

	inline double lerp(const double a, const double b, const double t) const
	{
		return a + t * (b - a);
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &from, const Eigen::Vector6d &to,
			const Eigen::Vector6d &offset, const Eigen::VectorXd &poly,
			const Eigen::Matrix<double, 7, 1> &line) const
	{
		// plane is normal and distance
		// plane is local baseline, c2w
		// line is Point + Vector + phase
		// line is local
		// measurement depends on bezier function

		double t = line(6);
		double mt = 1 - t;
		double t2 = t * t;
		double mt2 = mt * mt;

		Eigen::Vector4d A;
		A << mt2 * mt, 3 * mt2 * t, 3 * mt * t2, t2 * t; // bernstein
		Eigen::Vector3d Bs;

		Eigen::Vector6d imm = Eigen::Vector6d::Zero();
		for(size_t i = 0; i < imm.rows(); ++i)
		{
			if(i < 3)	// pos only interpolate
			{
				imm(i) = lerp(from(i), to(i), t);
			}
			else		// rotation poly interpolate
			{
				Eigen::Vector4d P;
				P << 0, poly(2*(i - 3)), poly(2*(i - 3) + 1), 0;

				double Bt = A.transpose() * P;	// extra angle to linear run
				Bs(i - 3) = Bt;

				imm(i) = lerp(from(i), to(i), t) + Bt;
			}
		}
		// compute the immediate pose

		/*std::cout << "time: " << t << " | " << Bs.transpose() << std::endl;
		std::cout << "from: " << from.transpose() << std::endl;
		std::cout << "to: " << to.transpose() << std::endl;
		std::cout << "imm: " << imm.transpose() << std::endl;*/

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtFrom = C3DJacobians::VecToPose(from);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtImm = C3DJacobians::VecToPose(imm);		//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOff = C3DJacobians::VecToPose(offset);	//c2w
		// poses of measurement

		Eigen::Vector4d pG = RtFrom.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtImm.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector4d pOff = RtOff.transpose() * pL;
		// transform plane observer -> offset

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pOff.head(3).transpose() * line.head(3) + pOff(3)) / pOff.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.segment<3>(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pOff.head(3).transpose() * dir) / (pOff.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(), m_vertex_ptr.Get<4>()->r_v_State(),
				m_v_measurement);
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "exp: " << r_v_error.transpose() << std::endl;

		const int polyn = 6;

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();	// plane
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian_tuple.Get<1>();  // from
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();  // to
		Eigen::Matrix<double, 2, 6> &J3 = r_t_jacobian_tuple.Get<3>();  // offset
		Eigen::Matrix<double, 2, polyn> &J4 = r_t_jacobian_tuple.Get<4>();  // polynome

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, polyn, polyn> Eps;
		Eps = Eigen::Matrix<double, polyn, polyn>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j).head(4), p_delta);

			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// plane
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// from
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					p_delta, m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// to
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J3.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// offset

		for(int j = 0; j < polyn; ++ j) {
			Eigen::Matrix<double, polyn, 1, Eigen::DontAlign> p_delta;
			BezierPlus(m_vertex_ptr.Get<4>()->r_v_State(), Eps.col(j), p_delta);

			J4.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					p_delta, m_v_measurement)) * scalar;
		}
		// poly
		// compute jacobians

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;
		std::cout << J3 << std::endl << std::endl;
		std::cout << J4 << std::endl << std::endl;*/

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error =  PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
						m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
						m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetBezier3DEdge_Global : public CBaseEdgeImpl<CPlaneOffsetBezier3DEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexBezier_3D), 2/* dist, angle */, 7/*, CBaseEdge::Robust*/>/*,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd>*/ {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetBezier3DEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexBezier_3D), 2, 7/*, CBaseEdge::Robust*/> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetBezier3DEdge_Global()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane
	 *	@param[in] n_node1 is (zero-based) index of the from
	 *	@param[in] n_node2 is (zero-based) index of the to
	 *	@param[in] n_node3 is (zero-based) index of the offset
	 *	@param[in] n_node4 is (zero-based) index of the polynome
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetBezier3DEdge_Global(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
			size_t n_node5,	const Eigen::Matrix<double, 7, 1> &r_v_delta, const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma,
			CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4, n_node5), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node4, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<5>() = &r_system.template r_Get_Vertex<CVertexBezier_3D>(n_node5, CInitializeNullVertex<CVertexBezier_3D>());
		// get vertices (initialize if required)
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	inline void BezierPlus(const Eigen::VectorXd &poly, const Eigen::VectorXd &delta,
			Eigen::Matrix<double, 6, 1, Eigen::DontAlign> &out) const // "smart" plus
	{
		out = poly + delta; // pick part of the delta vector, belonging to this vertex, apply +
	}

	inline double lerp(const double a, const double b, const double t) const
	{
		return a + t * (b - a);
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &from, const Eigen::Vector6d &to,
			const Eigen::Vector6d &offset, const Eigen::Vector6d &owner, const Eigen::VectorXd &poly,
			const Eigen::Matrix<double, 7, 1> &line) const
	{
		// plane is normal and distance
		// plane is local, c2w
		// line is Point + Vector + phase
		// line is local
		// measurement depends on polynomial function

		double t = line(6);
		double mt = 1 - t;
		double t2 = t * t;
		double mt2 = mt * mt;

		Eigen::Vector4d A;
		A << mt2 * mt, 3 * mt2 * t, 3 * mt * t2, t2 * t; // bernstein

		Eigen::Vector6d imm = Eigen::Vector6d::Zero();
		for(size_t i = 0; i < imm.rows(); ++i)
		{
			if(i < 3)	// pos only interpolate
			{
				imm(i) = lerp(from(i), to(i), t);
			}
			else		// rotation poly interpolate
			{
				Eigen::Vector4d P;
				P << 0, poly(2*(i - 3)), poly(2*(i - 3) + 1), 0;

				double Bt = A.transpose() * P;	// extra angle to linear run

				imm(i) = lerp(from(i), to(i), t) + Bt;
			}
		}
		// compute the immediate pose

		//imm = from + (to - from) * line(6);

		/*std::cout << "time: " << t << std::endl;
		std::cout << "from: " << from.transpose() << std::endl;
		std::cout << "to: " << to.transpose() << std::endl;
		std::cout << "imm: " << imm.transpose() << std::endl;*/

		/*std::cout << "owner: " << owner.transpose() << std::endl;
		std::cout << "obser: " << from.transpose() << std::endl;
		std::cout << "offset: " << offset.transpose() << std::endl;*/

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOwner = C3DJacobians::VecToPose(owner);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtImm = C3DJacobians::VecToPose(imm);		//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOff = C3DJacobians::VecToPose(offset);	//c2w
		// poses of measurement

		Eigen::Vector4d pG = RtOwner.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtImm.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector4d pOff = RtOff.transpose() * pL;
		// transform plane observer -> offset

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pOff.head(3).transpose() * line.head(3) + pOff(3)) / pOff.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.segment<3>(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pOff.head(3).transpose() * dir) / (pOff.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		//std::cout << "err: " << error.transpose() << std::endl;

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(), m_vertex_ptr.Get<4>()->r_v_State(),
				m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement);
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "exp: " << r_v_error.transpose() << std::endl;

		const int polyn = 6;

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();	// plane
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian_tuple.Get<1>();  // from - observer
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();  // to
		Eigen::Matrix<double, 2, 6> &J3 = r_t_jacobian_tuple.Get<3>();  // offset
		Eigen::Matrix<double, 2, 6> &J4 = r_t_jacobian_tuple.Get<4>();  // owner
		Eigen::Matrix<double, 2, polyn> &J5 = r_t_jacobian_tuple.Get<5>();  // polynome of observer

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, polyn, polyn> Eps;
		Eps = Eigen::Matrix<double, polyn, polyn>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j).head(4), p_delta);

			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// plane
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// from
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					p_delta, m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// to
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J3.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// offset

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<4>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J4.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					p_delta, m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// owner

		for(int j = 0; j < polyn; ++ j) {
			Eigen::Matrix<double, polyn, 1, Eigen::DontAlign> p_delta;
			BezierPlus(m_vertex_ptr.Get<5>()->r_v_State(), Eps.col(j), p_delta);

			J5.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), p_delta, m_v_measurement)) * scalar;
		}
		// poly
		// compute jacobians

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;
		std::cout << J3 << std::endl << std::endl;
		std::cout << J4 << std::endl << std::endl;
		std::cout << J5 << std::endl << std::endl;*/

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error =  PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
						m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
						m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetPolynomial6DEdge_Local : public CBaseEdgeImpl<CPlaneOffsetPolynomial6DEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPolynomial4_6D), 2/* dist, angle */, 7, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetPolynomial6DEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPolynomial4_6D), 2, 7, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetPolynomial6DEdge_Local()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane
	 *	@param[in] n_node1 is (zero-based) index of the from
	 *	@param[in] n_node2 is (zero-based) index of the to
	 *	@param[in] n_node3 is (zero-based) index of the offset
	 *	@param[in] n_node4 is (zero-based) index of the polynome
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetPolynomial6DEdge_Local(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
			const Eigen::Matrix<double, 7, 1> &r_v_delta, const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPolynomial4_6D>(n_node4, CInitializeNullVertex<CVertexPolynomial4_6D>());
		// get vertices (initialize if required)
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	inline void PolyPlus(const Eigen::VectorXd &poly, const Eigen::VectorXd &delta,
			Eigen::Matrix<double, 24, 1, Eigen::DontAlign> &out) const // "smart" plus
	{
		const int seg = 4;
		out = poly + delta; // pick part of the delta vector, belonging to this vertex, apply +
		for(int a = 0; a < 6; ++a)
		{
			out.segment<seg>(a * seg) = out.segment<seg>(a * seg) / out.segment<seg>(a * seg).sum();
		}
		/*for(int a = 0; a < 3; ++a)
		{
			out.segment<3>(a * 3) = out.segment<3>(a * 3) / out.segment<3>(a * 3).sum();
			out.segment<5>(9 + a * 5) = out.segment<5>(9 + a * 5) / out.segment<5>(9 + a * 5).sum();
		}*/
	}

	inline double lerp(const double a, const double b, const double t) const
	{
		return a + t * (b - a);
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &from, const Eigen::Vector6d &to,
			const Eigen::Vector6d &offset, const Eigen::VectorXd &poly,
			const Eigen::Matrix<double, 7, 1> &line) const
	{
		// plane is normal and distance
		// plane is local, c2w
		// line is Point + Vector + phase
		// line is local
		// measurement depends on polynomial function

		const int seg = 4;

		Eigen::Vector5d time = Eigen::Vector5d::Zero();
		time(0) = line(6);				// x^1
		time(1) = time(0) * time(0);	// x^2
		time(2) = time(1) * time(0);	// x^3
		time(3) = time(1) * time(1);	// x^4
		time(4) = time(3) * time(0);	// x^5

		Eigen::Vector6d imm = Eigen::Vector6d::Zero();
		for(size_t i = 0; i < imm.rows(); ++i)
		{
			double t = (time.head(seg).transpose() * poly.segment<seg>(seg * i))(0);
			//double tr = (time.head(5).transpose() * poly.segment<5>(9 + 5 * i))(0);
			//imm(i) = lerp(from(i), to(i), line(6));

			if(i < 3)
				imm(i) = lerp(from(i), to(i), time(0));
				//imm(i) = lerp(from(i), to(i), line(6)) + (t - line(6)) * 0.3;
			else
				imm(i) = lerp(from(i), to(i), line(6)) + (t - line(6)) * M_PI * 2;
		}
		/*std::cout << "time: " << time.transpose() << std::endl;
		std::cout << "from: " << from.transpose() << std::endl;
		std::cout << "to: " << to.transpose() << std::endl;
		std::cout << "imm: " << imm.transpose() << std::endl;*/
		// compute the immediate pose

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtFrom = C3DJacobians::VecToPose(from);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtImm = C3DJacobians::VecToPose(imm);		//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOff = C3DJacobians::VecToPose(offset);	//c2w
		// poses of measurement

		Eigen::Vector4d pG = RtFrom.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtImm.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector4d pOff = RtOff.transpose() * pL;
		// transform plane observer -> offset

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pOff.head(3).transpose() * line.head(3) + pOff(3)) / pOff.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.segment<3>(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pOff.head(3).transpose() * dir) / (pOff.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(), m_vertex_ptr.Get<4>()->r_v_State(),
				m_v_measurement);
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "exp: " << r_v_error.transpose() << std::endl;

		const int polyn = 24;

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();	// plane
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian_tuple.Get<1>();  // from
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();  // to
		Eigen::Matrix<double, 2, 6> &J3 = r_t_jacobian_tuple.Get<3>();  // offset
		Eigen::Matrix<double, 2, polyn> &J4 = r_t_jacobian_tuple.Get<4>();  // polynome

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, polyn, polyn> Eps;
		Eps = Eigen::Matrix<double, polyn, polyn>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j).head(4), p_delta);

			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// plane
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// from
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					p_delta, m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// to
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J3.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// offset

		for(int j = 0; j < polyn; ++ j) {
			Eigen::Matrix<double, polyn, 1, Eigen::DontAlign> p_delta;
			PolyPlus(m_vertex_ptr.Get<4>()->r_v_State(), Eps.col(j), p_delta);

			J4.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					p_delta, m_v_measurement)) * scalar;
		}
		// poly
		// compute jacobians

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;
		std::cout << J3 << std::endl << std::endl;
		std::cout << J4 << std::endl << std::endl;*/

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error =  PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
						m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
						m_vertex_ptr.Get<4>()->r_v_State(), m_v_measurement);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetPolynomial6DEdge_Global : public CBaseEdgeImpl<CPlaneOffsetPolynomial6DEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPolynomial4_6D), 2/* dist, angle */, 7, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetPolynomial6DEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPolynomial4_6D), 2, 7, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetPolynomial6DEdge_Global()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane
	 *	@param[in] n_node1 is (zero-based) index of the from
	 *	@param[in] n_node2 is (zero-based) index of the to
	 *	@param[in] n_node3 is (zero-based) index of the offset
	 *	@param[in] n_node4 is (zero-based) index of the polynome
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetPolynomial6DEdge_Global(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
			size_t n_node5,	const Eigen::Matrix<double, 7, 1> &r_v_delta, const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma,
			CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4, n_node5), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node4, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<5>() = &r_system.template r_Get_Vertex<CVertexPolynomial4_6D>(n_node5, CInitializeNullVertex<CVertexPolynomial4_6D>());
		// get vertices (initialize if required)
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	inline void PolyPlus(const Eigen::VectorXd &poly, const Eigen::VectorXd &delta,
			Eigen::Matrix<double, 24, 1, Eigen::DontAlign> &out) const // "smart" plus
	{
		const int seg = 4;
		out = poly + delta; // pick part of the delta vector, belonging to this vertex, apply +
		for(int a = 0; a < 6; ++a)
		{
			out.segment<seg>(a * seg) = out.segment<seg>(a * seg) / out.segment<seg>(a * seg).sum();
		}
		/*for(int a = 0; a < 3; ++a)
		{
			out.segment<3>(a * 3) = out.segment<3>(a * 3) / out.segment<3>(a * 3).sum();
			out.segment<5>(9 + a * 5) = out.segment<5>(9 + a * 5) / out.segment<5>(9 + a * 5).sum();
		}*/
	}

	inline double lerp(const double a, const double b, const double t) const
	{
		return a + t * (b - a);
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &from, const Eigen::Vector6d &to,
			const Eigen::Vector6d &offset, const Eigen::Vector6d &owner, const Eigen::VectorXd &poly,
			const Eigen::Matrix<double, 7, 1> &line) const
	{
		// plane is normal and distance
		// plane is local, c2w
		// line is Point + Vector + phase
		// line is local
		// measurement depends on polynomial function

		Eigen::Vector5d time = Eigen::Vector5d::Zero();
		time(0) = line(6);				// x^1
		time(1) = time(0) * time(0);	// x^2
		time(2) = time(1) * time(0);	// x^3
		time(3) = time(1) * time(1);	// x^4
		time(4) = time(3) * time(0);	// x^5

		const int seg = 4;

		Eigen::Vector6d imm = Eigen::Vector6d::Zero();
		for(size_t i = 0; i < imm.rows(); ++i)
		{
			double t = (time.head(seg).transpose() * poly.segment<seg>(seg * i))(0);
			//double tr = (time.head(5).transpose() * poly.segment<5>(9 + 5 * i))(0);
			//imm(i) = lerp(from(i), to(i), t);

			if(i < 3)
				imm(i) = lerp(from(i), to(i), time(0));
				//imm(i) = lerp(from(i), to(i), line(6));
				//imm(i) = lerp(from(i), to(i), line(6)) + (t - line(6)) * 0.3;
			else
				imm(i) = lerp(from(i), to(i), line(6)) + (t - line(6)) * M_PI * 2;
		}
		// compute the immediate pose

		//imm = from + (to - from) * line(6);

		/*std::cout << "time: " << time.transpose() << std::endl;
		std::cout << "from: " << from.transpose() << std::endl;
		std::cout << "to: " << to.transpose() << std::endl;
		std::cout << "imm: " << imm.transpose() << std::endl;*/

		/*std::cout << "owner: " << owner.transpose() << std::endl;
		std::cout << "obser: " << from.transpose() << std::endl;
		std::cout << "offset: " << offset.transpose() << std::endl;*/

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOwner = C3DJacobians::VecToPose(owner);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtImm = C3DJacobians::VecToPose(imm);		//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOff = C3DJacobians::VecToPose(offset);	//c2w
		// poses of measurement

		Eigen::Vector4d pG = RtOwner.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtImm.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector4d pOff = RtOff.transpose() * pL;
		// transform plane observer -> offset

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pOff.head(3).transpose() * line.head(3) + pOff(3)) / pOff.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.segment<3>(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pOff.head(3).transpose() * dir) / (pOff.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		//std::cout << "err: " << error.transpose() << std::endl;

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(), m_vertex_ptr.Get<4>()->r_v_State(),
				m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement);
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "exp: " << r_v_error.transpose() << std::endl;

		const int polyn = 24;

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();	// plane
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian_tuple.Get<1>();  // from - observer
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();  // to
		Eigen::Matrix<double, 2, 6> &J3 = r_t_jacobian_tuple.Get<3>();  // offset
		Eigen::Matrix<double, 2, 6> &J4 = r_t_jacobian_tuple.Get<4>();  // owner
		Eigen::Matrix<double, 2, polyn> &J5 = r_t_jacobian_tuple.Get<5>();  // polynome of observer

		//std::cout << "err " << r_v_error(0) << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, polyn, polyn> Eps;
		Eps = Eigen::Matrix<double, polyn, polyn>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j).head(4), p_delta);

			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// plane
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// from
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					p_delta, m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// to
		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J3.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), p_delta,
					m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// offset

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<4>()->r_v_State(), Eps.col(j).head(6), p_delta);

			J4.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					p_delta, m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement)) * scalar;
		}
		// owner

		for(int j = 0; j < polyn; ++ j) {
			Eigen::Matrix<double, polyn, 1, Eigen::DontAlign> p_delta;
			PolyPlus(m_vertex_ptr.Get<5>()->r_v_State(), Eps.col(j), p_delta);

			J5.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
					m_vertex_ptr.Get<4>()->r_v_State(), p_delta, m_v_measurement)) * scalar;
		}
		// poly
		// compute jacobians

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;
		std::cout << J3 << std::endl << std::endl;
		std::cout << J4 << std::endl << std::endl;
		std::cout << J5 << std::endl << std::endl;*/

		/*std::cout << r_t_jacobian0 << std::endl;
		std::cout << r_t_jacobian1 << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error =  PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
						m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State(),
						m_vertex_ptr.Get<4>()->r_v_State(), m_vertex_ptr.Get<5>()->r_v_State(), m_v_measurement);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetEdge : public CBaseEdgeImpl<CPlaneOffsetEdge, MakeTypelist(CVertexPlane3D, CVertexPose3D), 2/* dist, angle */, 6, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetEdge, MakeTypelist(CVertexPlane3D, CVertexPose3D), 2, 6, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetEdge()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	/*template <class CSystem>
	CPlaneOffsetEdge(const CParserBase::TEdgePlane &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPlane3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPlane3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}*/

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane node
	 *	@param[in] n_node1 is (zero-based) index of the observing node
	 *	@param[in] n_node2 is (zero-based) index of the owner node
	 *	@param[in] n_node3 is (zero-based) index of the offset
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetEdge(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &line,
			const Eigen::Vector6d &observer) const
	{
		// plane is normal and distance
		// plane is global
		// line is Point + Vector
		// line is local in observer base base

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtObs = C3DJacobians::VecToPose(observer);	//c2w
		// poses of vertices

		Eigen::Vector4d pG = plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtObs.transpose() * pG;
		//std::cout << "loc: " << pL.transpose() << std::endl;
		// transform plane global -> observer

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pL.head(3).transpose() * line.head(3) + pL(3)) / pL.head(3).norm();
		//std::cout << "x: " << (pL.head(3).transpose() * line.head(3)) << " " << pL(3) << std::endl;
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.tail(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pL.head(3).transpose() * dir) / (pL.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		// todo: penalize vectors outside of the line segment

		return error;
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 4> &r_t_jacobian0,
			Eigen::Matrix<double, 2, 6> &r_t_jacobian1, Eigen::Matrix<double, 2, 1> &r_v_expectation,
			Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	/*inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required*/
	{
		//std:: cout << m_p_vertex0->r_v_State().transpose() << std::endl;
		//std:: cout << m_p_vertex1->r_v_State().transpose() << std::endl;

		r_v_expectation = PlaneError(m_p_vertex0->r_v_State(),
				m_v_measurement, m_p_vertex1->r_v_State());
		r_v_error = r_v_expectation;
		// compute error
		//std:: cout << r_v_error.transpose() << std::endl;

		//std::cout << "errG " << r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian0;
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian1;

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_p_vertex0->r_v_State(), Eps.col(j).head(4), p_delta);
			//p_delta = m_vertex_ptr.Get<0>()->r_v_State() + Eps.col(j).head(4);
			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_v_measurement, m_p_vertex1->r_v_State())) * scalar;
		}
		// J0

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);
			J1.col(j) = (r_v_expectation - PlaneError(m_p_vertex0->r_v_State(), m_v_measurement, p_delta)) * scalar;
		}
		// J1

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;*/
		//std::cout << J2 << std::endl << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_p_vertex0->r_v_State(),
				m_v_measurement, m_p_vertex1->r_v_State());
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetEdge_Local : public CBaseEdgeImpl<CPlaneOffsetEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPose3D), 2/* dist, angle */, 6, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetEdge_Local, MakeTypelist(CVertexPlane3D, CVertexPose3D), 2, 6, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetEdge_Local()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane
	 *	@param[in] n_node1 is (zero-based) index of the offset
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetEdge_Local(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &line,
				const Eigen::Vector6d &offset) const
	{
		// plane is normal and distance
		// plane is in baseframe, c2w
		// line is Point + Vector
		// line is in offset frame

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOff = C3DJacobians::VecToPose(offset);	//c2w
		// poses of measurement

		Eigen::Vector4d pOff = RtOff.transpose() * plane;
		// transform plane -> offset

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pOff.head(3).transpose() * line.head(3) + pOff(3)) / pOff.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.tail(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pOff.head(3).transpose() * dir) / (pOff.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		return error;
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 4> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 6> &r_t_jacobian1,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_p_vertex0->r_v_State(), m_v_measurement, m_p_vertex1->r_v_State());
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "errL " << r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_p_vertex0->r_v_State(), Eps.col(j).head(4), p_delta);
			//p_delta = m_p_vertex0->r_v_State() + Eps.col(j).head(4);
			r_t_jacobian0.col(j) =
					(r_v_expectation - PlaneError(p_delta, m_v_measurement, m_p_vertex1->r_v_State())) * scalar;
		}

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);
			r_t_jacobian1.col(j) =
					(r_v_expectation - PlaneError(m_p_vertex0->r_v_State(), m_v_measurement, p_delta)) * scalar;
		}
		// compute jacobians

		//std::cout << r_t_jacobian0 << std::endl;
		//std::cout << r_t_jacobian1 << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_p_vertex0->r_v_State(), m_v_measurement, m_p_vertex1->r_v_State());
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) edge
 */
class CPlaneOffsetEdge_Global : public CBaseEdgeImpl<CPlaneOffsetEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 2/* dist, angle */, 6, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CPlaneOffsetEdge_Global, MakeTypelist(CVertexPlane3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 2, 6, CBaseEdge::Robust> _TyBase; /**< @brief base class */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CPlaneOffsetEdge_Global()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the plane node
	 *	@param[in] n_node1 is (zero-based) index of the observing node
	 *	@param[in] n_node2 is (zero-based) index of the owner node
	 *	@param[in] n_node3 is (zero-based) index of the offset
	 *	@param[in] r_v_delta is measurement vector
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CPlaneOffsetEdge_Global(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, const Eigen::Matrix<double, 6, 1> &r_v_delta,
		const Eigen::Matrix<double, 2, 2> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPlane3D>(n_node0, CInitializeNullVertex<CVertexPlane3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Vector2d PlaneError(const Eigen::Vector4d &plane, const Eigen::Vector6d &line,
			const Eigen::Vector6d &observer, const Eigen::Vector6d &owner,const Eigen::Vector6d &offset) const
	{
		// plane is normal and distance
		// plane is local in owner framebase, c2w
		// line is Point + Vector
		// line is local in observer offset base

		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtObs = C3DJacobians::VecToPose(observer);	//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOwn = C3DJacobians::VecToPose(owner);		//c2w
		Eigen::Matrix<double, 4, 4, Eigen::DontAlign> RtOff = C3DJacobians::VecToPose(offset);		//c2w
		// poses of vertices

		Eigen::Vector4d pG = RtOwn.inverse().transpose() * plane;
		// transform plane owner -> global

		Eigen::Vector4d pL = RtObs.transpose() * pG;
		// transform plane global -> observer

		Eigen::Vector4d pOff = RtOff.transpose() * pL;
		// transform plane observer -> offset

		Eigen::Vector2d error = Eigen::Vector2d::Zero();
		error(0) = /*abs*/(pOff.head(3).transpose() * line.head(3) + pOff(3)) / pOff.head(3).norm();
		// distance error, abs should not matter

		Eigen::Vector3d dir = line.tail(3)/* - line.head(3)*/;
		error(1) = /*asin(*/ ((pOff.head(3).transpose() * dir) / (pOff.head(3).norm() * dir.norm()))(0) /*)*/;
		// angle error

		// todo: penalize vectors outside of the line segment

		return error;
	}

	inline void PlanePlus(const Eigen::Vector4d &plane, const Eigen::Vector4d &delta, Eigen::Vector4d &out) const // "smart" plus
	{
		Eigen::Vector3d vec = delta.head(3);

		out.head(3) = C3DJacobians::Operator_rot(vec) * plane.head(3);
		out.tail(1) = plane.tail(1) + delta.tail(1);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(),
				m_v_measurement, m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(),
				m_vertex_ptr.Get<3>()->r_v_State());
		r_v_error = r_v_expectation;
		// compute error

		//std::cout << "errG " << r_v_error.transpose() << std::endl;

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 2, 4> &J0 = r_t_jacobian_tuple.Get<0>();
		Eigen::Matrix<double, 2, 6> &J1 = r_t_jacobian_tuple.Get<1>();
		Eigen::Matrix<double, 2, 6> &J2 = r_t_jacobian_tuple.Get<2>();
		Eigen::Matrix<double, 2, 6> &J3 = r_t_jacobian_tuple.Get<3>();

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 4; ++ j) {
			Eigen::Matrix<double, 4, 1> p_delta;
			PlanePlus(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j).head(4), p_delta);
			//p_delta = m_vertex_ptr.Get<0>()->r_v_State() + Eps.col(j).head(4);
			J0.col(j) = (r_v_expectation - PlaneError(p_delta, m_v_measurement, m_vertex_ptr.Get<1>()->r_v_State(),
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State())) * scalar;
		}
		// J0

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);
			J1.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_v_measurement, p_delta,
					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State())) * scalar;
		}
		// J1

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
			J2.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_v_measurement,
					m_vertex_ptr.Get<1>()->r_v_State(),	p_delta, m_vertex_ptr.Get<3>()->r_v_State())) * scalar;
		}
		// J2

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> p_delta;
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j), p_delta);
			J3.col(j) = (r_v_expectation - PlaneError(m_vertex_ptr.Get<0>()->r_v_State(), m_v_measurement,
					m_vertex_ptr.Get<1>()->r_v_State(),	m_vertex_ptr.Get<2>()->r_v_State(), p_delta)) * scalar;
		}
		// J3

		/*std::cout << J0 << std::endl << std::endl;
		std::cout << J1 << std::endl << std::endl;
		std::cout << J2 << std::endl << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 2, 1> v_error;
		v_error = PlaneError(m_vertex_ptr.Get<0>()->r_v_State(),
				m_v_measurement, m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(),
				m_vertex_ptr.Get<3>()->r_v_State());
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-pose edge
 */
//class CBSplineEdge6D : public CBaseEdgeImpl<CBSplineEdge, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 12, 12> {
//
//public:
//	typedef CBaseEdgeImpl<CBSplineEdge6D, MakeTypelist(CVertexPose3D, CVertexPose3D, CVertexPose3D, CVertexPose3D), 12, 12> _TyBase; /**< @brief base class */
//
//public:
//	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this
//
//	/**
//	 *	@brief default constructor; has no effect
//	 */
//	inline CBSplineEdge6D()
//	{}
//
//	template <class CSystem>
//	CBSplineEdge6D(const CParserBase::TEdgeSpline3D &r_t_edge, CSystem &r_system)
//		:_TyBase(typename _TyBase::_TyVertexIndexTuple(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1, r_t_edge.m_n_node_2, r_t_edge.m_n_node_3),
//		r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
//	{
//		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_1, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_2, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(r_t_edge.m_n_node_3, CInitializeNullVertex<CVertexPose3D>());
//		// get vertices (initialize if required)
//
//		//std::cout << r_t_edge.m_n_node_0 << " " << r_t_edge.m_n_node_1 << " " << r_t_edge.m_n_node_2 << " " <<
//		//		r_t_edge.m_n_node_3 << " " << r_t_edge.m_v_delta.transpose() << " " << r_t_edge.m_t_inv_sigma << std::endl;
//
//		/*std::cout << "? " << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << "? " << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << "? " << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << "? " << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/
//	}
//
//	/**
//	 *	@brief constructor; converts parsed edge to edge representation
//	 *
//	 *	@tparam CSystem is type of system where this edge is being stored
//	 *
//	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
//	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node
//	 *	@param[in] r_v_delta is measurement vector
//	 *	@param[in] r_t_inv_sigma is the information matrix
//	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
//	 */
//	template <class CSystem>
//	CBSplineEdge6D(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, const Eigen::Matrix<double, 12, 1> &r_v_delta,
//		const Eigen::Matrix<double, 12, 12> &r_t_inv_sigma, CSystem &r_system)
//		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3), r_v_delta, r_t_inv_sigma)
//	{
//		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node1, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node2, CInitializeNullVertex<CVertexPose3D>());
//		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node3, CInitializeNullVertex<CVertexPose3D>());
//		// get vertices (initialize if required)
//	}
//
//	/**
//	 *	@brief updates the edge with a new measurement
//	 *
//	 *	@param[in] r_v_delta is new measurement vector
//	 *	@param[in] r_t_inv_sigma is new information matrix
//	 */
//	inline void Update(const Eigen::Matrix<double, 12, 1> &r_v_delta, const Eigen::Matrix<double, 12, 12> &r_t_inv_sigma) // for some reason this needs to be here, although the base already implements this
//	{
//		_TyBase::Update(r_v_delta, r_t_inv_sigma);
//	}
//
//	/**
//	 *	@brief calculates jacobians, expectation and error
//	 *
//	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
//	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
//	 *	@param[out] r_v_expectation is expecation vector
//	 *	@param[out] r_v_error is error vector
//	 */
//	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
//			Eigen::Matrix<double, 12, 1> &r_v_expectation, Eigen::Matrix<double, 12, 1> &r_v_error) const // change dimensionality of eigen types, if required
//	{
//		// calculates the expectation and the jacobians
//		/*std::cout << m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/
//
//		BSplineSE3 spline(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//				m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//		r_v_expectation = spline.bspline_error();
//		// expectation error
//
//		const double delta = 1e-9;
//		const double scalar = 1.0 / (delta);
//		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack
//		Eigen::Matrix<double, 6, 1> p_delta;
//
//		for(size_t j = 0; j < 6; ++j) // four 1x6 jacobians
//		{
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<0>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline0(p_delta, m_vertex_ptr.Get<1>()->r_v_State(),
//					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<0>().col(j) = (-spline0.bspline_error6D() + r_v_expectation) * scalar;
//			// J0
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline1(m_vertex_ptr.Get<0>()->r_v_State(), p_delta,
//					m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<1>().col(j) = (-spline1.bspline_error6D() + r_v_expectation) * scalar;
//			// J1
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline2(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//					p_delta, m_vertex_ptr.Get<3>()->r_v_State());
//			r_t_jacobian_tuple.Get<2>().col(j) = (-spline2.bspline_error6D() + r_v_expectation) * scalar;
//			// J2
//
//			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State(), Eps.col(j), p_delta);
//			BSplineSE3 spline3(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//					m_vertex_ptr.Get<2>()->r_v_State(), p_delta);
//			r_t_jacobian_tuple.Get<3>().col(j) = (-spline3.bspline_error6D() + r_v_expectation) * scalar;
//			// J3
//		}
//		// error is given by spline function
//
//		r_v_error = m_v_measurement - r_v_expectation;	// measurement should be zero
//		// calculates error (possibly re-calculates, if running A-SLAM)
//	}
//
//	/**
//	 *	@brief calculates \f$\chi^2\f$ error
//	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
//	 */
//	inline double f_Chi_Squared_Error() const
//	{
//		/*std::cout << "er " <<  m_vertex_ptr.Get<0>()->r_v_State().transpose() << std::endl;
//		std::cout << "er " <<  m_vertex_ptr.Get<1>()->r_v_State().transpose() << std::endl;
//		std::cout << "er " <<  m_vertex_ptr.Get<2>()->r_v_State().transpose() << std::endl;
//		std::cout << "er " <<  m_vertex_ptr.Get<3>()->r_v_State().transpose() << std::endl;*/
//
//		BSplineSE3 spline(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
//			m_vertex_ptr.Get<2>()->r_v_State(), m_vertex_ptr.Get<3>()->r_v_State());
//		Eigen::Matrix<double, 1, 1> v_error = m_v_measurement - spline.bspline_error();
//		// calculates the expectation, error and the jacobians
//
//		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
//	}
//};

/**
 *	@brief Sim(3) pose roll edge
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

/**
 *	@brief pose Z edge
 */
class CEdge3DY : public CBaseEdgeImpl<CEdge3DY, MakeTypelist(CVertexPose3D), 1> {
public:
	typedef CBaseEdgeImpl<CEdge3DY, MakeTypelist(CVertexPose3D), 1> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdge3DY()
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
	CEdge3DY(size_t n_vertex0, const Eigen::Matrix<double, 1, 1> &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_vertex0, CInitializeNullVertex<>());
		// initialize vertex 1 manually
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma)
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
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 6> &r_t_jacobian0,
			Eigen::Matrix<double, 1, 1> &r_v_expectation, Eigen::Matrix<double, 1, 1> &r_v_error) const
	{
		r_v_expectation(0) = m_p_vertex0->r_v_State()(1);

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		Eigen::Matrix<double, 1, 6> &H1 = r_t_jacobian0;
		// can actually work inplace

		Eigen::Matrix<double, 6, 1> p_delta;

		//for XYZ and RPY
		for(int j = 0; j < 6; ++ j) {
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);

			H1(j) = (p_delta(1) - r_v_expectation(0)) * scalar;
		}
		r_v_error(0) = m_v_measurement(0) - r_v_expectation(0);

		//std::cout << "exp: " << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() <<
		//		" : " << (r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error) << std::endl;
		//std::cout << H1 << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 1> v_error;

		v_error(0) = m_v_measurement(0) - m_p_vertex0->r_v_State()(1);
		//std::cout << "ch: " << m_v_measurement.transpose() << " <> " << m_p_vertex0->r_v_State().head<3>().transpose() << std::endl;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief pose Z edge
 */
class CEdge3DY2 : public CBaseEdgeImpl<CEdge3DY2, MakeTypelist(CVertexPose3D), 1> {
public:
	typedef CBaseEdgeImpl<CEdge3DY2, MakeTypelist(CVertexPose3D), 1> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this
	Eigen::Vector6d m_relative_vector;

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdge3DY2()
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
	CEdge3DY2(size_t n_vertex0, Eigen::Vector6d rel, const Eigen::Matrix<double, 1, 1> &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_vertex0, CInitializeNullVertex<>());
		m_relative_vector = rel;
		// initialize vertex 1 manually
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma)
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
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 6> &r_t_jacobian0,
			Eigen::Matrix<double, 1, 1> &r_v_expectation, Eigen::Matrix<double, 1, 1> &r_v_error) const
	{
		Eigen::Vector6d pose;
		C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), m_relative_vector, pose);
		r_v_expectation(0) = pose(1);

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		Eigen::Matrix<double, 1, 6> &H1 = r_t_jacobian0;
		// can actually work inplace

		Eigen::Matrix<double, 6, 1> p_delta, d;

		//for XYZ and RPY
		for(int j = 0; j < 6; ++ j) {
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Relative_to_Absolute(p_delta, m_relative_vector, d);
			H1(j) = (d(1) - r_v_expectation(0)) * scalar;
		}
		r_v_error(0) = m_v_measurement(0) - r_v_expectation(0);

		//std::cout << "exp: " << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() <<
		//		" : " << (r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error) << std::endl;
		//std::cout << H1 << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 1> v_error;

		v_error(0) = m_v_measurement(0) - m_p_vertex0->r_v_State()(1);
		//std::cout << "ch: " << m_v_measurement.transpose() << " <> " << m_p_vertex0->r_v_State().head<3>().transpose() << std::endl;

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

/**
 *	@brief SE(3) pose-landmark edge
 */
class CEdgeBearingLandmark3D : public CBaseEdgeImpl<CEdgeBearingLandmark3D, MakeTypelist(CVertexPose3D, CVertexLandmark3D), 3, 3, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeBearingLandmark3D, MakeTypelist(CVertexPose3D, CVertexLandmark3D), 3, 3, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeBearingLandmark3D()
	{}

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
	CEdgeBearingLandmark3D(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(n_node1, CInitializeNullVertex<CVertexLandmark3D>());
		// get vertices (initialize if required)
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
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 3> &r_t_jacobian1, Eigen::Matrix<double, 3, 1> &r_v_expectation,
		Eigen::Matrix<double, 3, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), r_v_expectation);
		// first go to local rep
		r_v_expectation = r_v_expectation.normalized();
		// now get the vector

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster

		Eigen::Matrix<double, 6, 1> p_delta;
		Eigen::Vector3d dest1, p_delta2;
		for(size_t j = 0; j < 6; ++j)
		{
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative_Landmark(p_delta, m_p_vertex1->r_v_State(), dest1);
			dest1 = dest1.normalized();
			r_t_jacobian0.col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - pose

		for(size_t j = 0; j < 3; ++j)
		{
			p_delta2 = m_p_vertex1->r_v_State() + Eps.col(j).template head<3>();
			C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(), p_delta2, dest1);
			dest1 = dest1.normalized();
			r_t_jacobian1.col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - xyz

		r_v_error = m_v_measurement.normalized() - r_v_expectation;

		//std::cout << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << " | " <<
		//				r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 1> v_error, v_expectation;
		C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), v_expectation);
		// first go to local rep
		v_expectation = v_expectation.normalized();
		// now get the vector

		v_error = m_v_measurement.normalized() - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-landmark edge
 */
class CEdgeBearingLandmark3DInverse : public CBaseEdgeImpl<CEdgeBearingLandmark3DInverse, MakeTypelist(CVertexPose3D, CVertexLandmark3DInverse), 3, 3, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeBearingLandmark3DInverse, MakeTypelist(CVertexPose3D, CVertexLandmark3DInverse), 3, 3, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeBearingLandmark3DInverse()
	{}

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
	CEdgeBearingLandmark3DInverse(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, n_node1, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose3D>(n_node0, CInitializeNullVertex<CVertexPose3D>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexLandmark3DInverse>(n_node1, CInitializeNullVertex<CVertexLandmark3DInverse>());
		// get vertices (initialize if required)
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
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 3> &r_t_jacobian1, Eigen::Matrix<double, 3, 1> &r_v_expectation,
		Eigen::Matrix<double, 3, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(),
				CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex1->r_v_State()), r_v_expectation);
		// first go to local rep
		r_v_expectation = r_v_expectation.normalized();
		// now get the vector

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster

		Eigen::Matrix<double, 6, 1> p_delta;
		Eigen::Vector3d dest1, p_delta2;
		for(size_t j = 0; j < 6; ++j)
		{
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative_Landmark(p_delta,
					CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex1->r_v_State()), dest1);
			dest1 = dest1.normalized();
			r_t_jacobian0.col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - pose

		for(size_t j = 0; j < 3; ++j)
		{
			CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_p_vertex1->r_v_State(),
					Eps.col(j).template head<3>(), p_delta2); // epsilon is in xyz
			C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(),
					CSim3Jacobians::v_InvDepth_to_XYZ(p_delta2), dest1);
			dest1 = dest1.normalized();
			r_t_jacobian1.col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - xyz

		r_v_error = m_v_measurement.normalized() - r_v_expectation;

		/*std::cout << CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex1->r_v_State()).transpose() << " ";
		std::cout << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << " | " <<
						r_v_error.transpose() << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 1> v_error, v_expectation;
		C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(),
				CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex1->r_v_State()), v_expectation);
		// first go to local rep
		v_expectation = v_expectation.normalized();
		// now get the vector

		v_error = m_v_measurement.normalized() - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-landmark edge
 */
class CEdgeBearingLandmark3DLocalInverse : public CBaseEdgeImpl<CEdgeBearingLandmark3DLocalInverse, MakeTypelist(CVertexLandmark3DInverse), 3, 3, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeBearingLandmark3DLocalInverse, MakeTypelist(CVertexLandmark3DInverse), 3, 3, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeBearingLandmark3DLocalInverse()
	{}

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
	CEdgeBearingLandmark3DLocalInverse(size_t n_node0, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexLandmark3DInverse>(n_node0, CInitializeNullVertex<CVertexLandmark3DInverse>());
		// get vertices (initialize if required)
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
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 1> &r_v_expectation, Eigen::Matrix<double, 3, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()).normalized();
		// now get the vector

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster

		Eigen::Vector3d dest1, p_delta2;
		for(size_t j = 0; j < 3; ++j)
		{
			CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_p_vertex0->r_v_State(),
					Eps.col(j).template head<3>(), p_delta2); // epsilon is in xyz
			dest1 =  CSim3Jacobians::v_InvDepth_to_XYZ(p_delta2).normalized();
			r_t_jacobian0.col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - xyz

		r_v_error = m_v_measurement.normalized() - r_v_expectation;

		//std::cout << "LS " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << " | " <<
		//				r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 1> v_error, v_expectation;
		v_expectation = CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()).normalized();
		// now get the vector

		v_error = m_v_measurement.normalized() - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

class CEdgeBearingLandmark3DOtherInverse : public CBaseEdgeImpl<CEdgeBearingLandmark3DOtherInverse, MakeTypelist_Safe((CVertexLandmark3DInverse, CVertexPose3D, CVertexPose3D)), 3, 3, CBaseEdge::Robust>,
	public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> { // note that this is a ternary edge
public:
	typedef CBaseEdgeImpl<CEdgeBearingLandmark3DOtherInverse, MakeTypelist_Safe((CVertexLandmark3DInverse, CVertexPose3D, CVertexPose3D)), 3, 3, CBaseEdge::Robust> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeBearingLandmark3DOtherInverse()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 */
	template <class CSystem>
	CEdgeBearingLandmark3DOtherInverse(size_t n_landmark_id, size_t n_observing_id, size_t n_owner_id, const Eigen::Vector3d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_id, n_owner_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexLandmark3DInverse>(n_landmark_id, CInitializeNullVertex<CVertexLandmark3DInverse>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_observing_id, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_owner_id, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Matrix<double, 3, 1> OwnerToObserver(const Eigen::Matrix<double, 3, 1> &landmark,
			const Eigen::Matrix<double, 6, 1> &observer, const Eigen::Matrix<double, 6, 1> &owner) const
	{
		/*Eigen::Matrix<double, 7, 1> obs = Eigen::Matrix<double, 7, 1>::Ones();
		obs.head(6) = observer;
		Eigen::Matrix<double, 7, 1> own = Eigen::Matrix<double, 7, 1>::Ones();
		own.head(6) = owner;
		const CSim3Jacobians::TSim3 t_observer(obs, CSim3Jacobians::TSim3::from_tRs_vector);
		const CSim3Jacobians::TSim3 t_owner(own, CSim3Jacobians::TSim3::from_tRs_vector);
		CSim3Jacobians::TSim3 t_diff_transform = t_observer;
		t_diff_transform.Inverse_Compose(t_owner); // calculate direct transform as inverse composition (t_observer^{-1} * t_owner)
		// transform landmark to observer

		return (t_diff_transform * CSim3Jacobians::v_InvDepth_to_XYZ(landmark));*/

		Eigen::Vector4d pt = Eigen::Vector4d::Ones();
		pt.head(3) = CSim3Jacobians::v_InvDepth_to_XYZ(landmark);
		// 3d point in owner cf

		Eigen::Matrix4d own_c2w = Eigen::Matrix4d::Identity();
		own_c2w.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(owner.tail(3));
		own_c2w.block(0, 3, 3, 1) = owner.head(3);
		// owner cf

		Eigen::Matrix4d obs_c2w = Eigen::Matrix4d::Identity();
		obs_c2w.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(observer.tail(3));
		obs_c2w.block(0, 3, 3, 1) = observer.head(3);
		// observer cf

		pt = own_c2w * pt;
		// pt -> world

		pt = obs_c2w.inverse() * pt;
		// pt -> observer

		return pt.head(3);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 3, 1> &r_v_expectation, Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		Eigen::Matrix<double, 3, 1> X = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State());
		r_v_expectation = X.normalized();
		// now get the vector

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster

		Eigen::Vector3d dest1, p_delta2;
		for(size_t j = 0; j < 3; ++j)
		{
			CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_vertex_ptr.Get<0>()->r_v_State(),
					Eps.col(j).template head<3>(), p_delta2); // epsilon is in xyz
			dest1 = OwnerToObserver(p_delta2, m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State()).normalized();

			r_t_jacobian_tuple.Get<0>().col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - xyz

		Eigen::Matrix<double, 6, 1> p_delta;
		for(size_t j = 0; j < 6; ++j)
		{
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);

			dest1 = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), p_delta, m_vertex_ptr.Get<2>()->r_v_State()).normalized();

			r_t_jacobian_tuple.Get<1>().col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - observer

		for(size_t j = 0; j < 6; ++j)
		{
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);

			dest1 = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(), p_delta).normalized();

			r_t_jacobian_tuple.Get<2>().col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - owner

		r_v_error = m_v_measurement.normalized() - r_v_expectation;

		//std::cout << "LO " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << " | " <<
		//						r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 1> v_error, v_expectation;
		v_expectation = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State()).normalized();
		// now get the vector

		v_error = m_v_measurement.normalized() - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief SE(3) pose-landmark edge
 */
class CEdgeBearingLandmark3DLocal : public CBaseEdgeImpl<CEdgeBearingLandmark3DLocal, MakeTypelist(CVertexLandmark3D), 3, 3, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeBearingLandmark3DLocal, MakeTypelist(CVertexLandmark3D), 3, 3, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeBearingLandmark3DLocal()
	{}

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
	CEdgeBearingLandmark3DLocal(size_t n_node0, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, r_v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexLandmark3D>(n_node0, CInitializeNullVertex<CVertexLandmark3D>());
		// get vertices (initialize if required)
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
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 1> &r_v_expectation, Eigen::Matrix<double, 3, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		r_v_expectation = m_p_vertex0->r_v_State().normalized();
		// now get the vector

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster

		Eigen::Vector3d dest1, p_delta2;
		for(size_t j = 0; j < 3; ++j)
		{
			p_delta2 = m_p_vertex0->r_v_State() + Eps.col(j).template head<3>();
			dest1 =  p_delta2.normalized();
			r_t_jacobian0.col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - xyz

		r_v_error = m_v_measurement.normalized() - r_v_expectation;

		//std::cout << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << " | " <<
		//				r_v_error.transpose() << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 1> v_error, v_expectation;
		v_expectation = m_p_vertex0->r_v_State().normalized();
		// now get the vector

		v_error = m_v_measurement.normalized() - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

class CEdgeBearingLandmark3DOther : public CBaseEdgeImpl<CEdgeBearingLandmark3DOther, MakeTypelist_Safe((CVertexLandmark3D, CVertexPose3D, CVertexPose3D)), 3, 3, CBaseEdge::Robust>,
	public CRobustify_ErrorNorm_Default<CCTFraction<148, 1000>, CHuberLossd> { // note that this is a ternary edge
public:
	typedef CBaseEdgeImpl<CEdgeBearingLandmark3DOther, MakeTypelist_Safe((CVertexLandmark3D, CVertexPose3D, CVertexPose3D)), 3, 3, CBaseEdge::Robust> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeBearingLandmark3DOther()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 */
	template <class CSystem>
	CEdgeBearingLandmark3DOther(size_t n_landmark_id, size_t n_observing_id, size_t n_owner_id, const Eigen::Vector3d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_id, n_owner_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexLandmark3D>(n_landmark_id, CInitializeNullVertex<CVertexLandmark3D>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_observing_id, CInitializeNullVertex<CVertexPose3D>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexPose3D>(n_owner_id, CInitializeNullVertex<CVertexPose3D>());
		// get vertices (initialize if required)
	}

	inline Eigen::Matrix<double,3, 1> OwnerToObserver(const Eigen::Matrix<double, 3, 1> &landmark,
			const Eigen::Matrix<double, 6, 1> &observer, const Eigen::Matrix<double, 6, 1> &owner) const
	{
		/*Eigen::Matrix<double, 7, 1> obs = Eigen::Matrix<double, 7, 1>::Ones();
		obs.head(6) = observer;
		Eigen::Matrix<double, 7, 1> own = Eigen::Matrix<double, 7, 1>::Ones();
		own.head(6) = owner;
		const CSim3Jacobians::TSim3 t_observer(obs, CSim3Jacobians::TSim3::from_tRs_vector);
		const CSim3Jacobians::TSim3 t_owner(own, CSim3Jacobians::TSim3::from_tRs_vector);
		CSim3Jacobians::TSim3 t_diff_transform = t_observer;
		t_diff_transform.Inverse_Compose(t_owner); // calculate direct transform as inverse composition (t_observer^{-1} * t_owner)
		// transform landmark to observer

		return (t_diff_transform * landmark);*/

		Eigen::Vector4d pt = Eigen::Vector4d::Ones();
		pt.head(3) = landmark;
		// 3d point in owner cf

		Eigen::Matrix4d own_c2w = Eigen::Matrix4d::Identity();
		own_c2w.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(owner.tail(3));
		own_c2w.block(0, 3, 3, 1) = owner.head(3);
		// owner cf

		Eigen::Matrix4d obs_c2w = Eigen::Matrix4d::Identity();
		obs_c2w.block(0, 0, 3, 3) = C3DJacobians::Operator_rot(observer.tail(3));
		obs_c2w.block(0, 3, 3, 1) = observer.head(3);
		// observer cf

		pt = own_c2w * pt;
		// pt -> world

		pt = obs_c2w.inverse() * pt;
		// pt -> observer

		return pt.head(3);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 3, 1> &r_v_expectation, Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		Eigen::Matrix<double, 3, 1> X = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State());
		r_v_expectation = X.normalized();
		// now get the vector

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster

		Eigen::Vector3d dest1, p_delta2;
		for(size_t j = 0; j < 3; ++j)
		{
			p_delta2 = m_vertex_ptr.Get<0>()->r_v_State() + Eps.col(j).template head<3>();
			dest1 = OwnerToObserver(p_delta2, m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State()).normalized();

			r_t_jacobian_tuple.Get<0>().col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - xyz

		Eigen::Matrix<double, 6, 1> p_delta;
		for(size_t j = 0; j < 6; ++j)
		{
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State(), Eps.col(j), p_delta);

			dest1 = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), p_delta, m_vertex_ptr.Get<2>()->r_v_State()).normalized();

			r_t_jacobian_tuple.Get<1>().col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - observer

		for(size_t j = 0; j < 6; ++j)
		{
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State(), Eps.col(j), p_delta);

			dest1 = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(), p_delta).normalized();

			r_t_jacobian_tuple.Get<2>().col(j) = (dest1 - r_v_expectation) * scalar;
		}
		// now jacobians - owner

		r_v_error = m_v_measurement.normalized() - r_v_expectation;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 1> v_error, v_expectation;
		v_expectation = OwnerToObserver(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State()).normalized();
		// now get the vector

		v_error = m_v_measurement.normalized() - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeZDist : public CBaseEdgeImpl<CEdgeZDist, MakeTypelist(CVertexPose3D, CVertexPose3D), 1> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeZDist()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node (camera)
	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node (vertex)
	 *	@param[in] v_delta is vector of delta x position, delta y-position
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgeZDist(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 1, 1> &v_delta, // won't align non-reference arg
		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:CBaseEdgeImpl<CEdgeZDist, MakeTypelist(CVertexPose3D, CVertexPose3D), 1>(n_node0,
		n_node1, v_delta, r_t_inv_sigma, CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexCam>(n_node0, CInitializeNullVertex<>());
		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node1, CInitializeNullVertex<CVertexXYZ>());
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	/*inline void Update(const CParserBase::TEdgeP2C3D &r_t_edge)
	{
		CBaseEdgeImpl<CEdgeP2C3D, MakeTypelist(CVertexCam, CVertexXYZ),
			2>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}*/
	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Matrix<double, 1, 1> &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma)
	{
		CBaseEdgeImpl<CEdgeZDist, MakeTypelist(CVertexPose3D, CVertexPose3D), 1>::Update(r_v_delta, r_t_inv_sigma);
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
		Eigen::Matrix<double, 1, 6> &r_t_jacobian1, Eigen::Matrix<double, 1, 1> &r_v_expectation,
		Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		Eigen::Matrix<double, 6, 1> pose0 = m_p_vertex0->r_v_State();
		Eigen::Matrix<double, 6, 1> pose1 = m_p_vertex1->r_v_State();

		Eigen::Matrix<double, 6, 1> rel;
		C3DJacobians::Absolute_to_Relative(pose0, pose1, rel);
		// calculates the expectation and the jacobians

		r_v_expectation(0) = rel(2); // Z
		r_v_error(0) = m_v_measurement(0) - r_v_expectation(0); /* */

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> d1, p_delta, p_delta_inv;
			C3DJacobians::Relative_to_Absolute(pose0, Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, pose1, d1);

			r_t_jacobian0(j) = (d1(2) - r_v_expectation(0)) * scalar;

			Eigen::Matrix<double, 6, 1> d2;
			C3DJacobians::Relative_to_Absolute(pose1, Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(pose0, pose1, d2);
			r_t_jacobian1(j) = (d2(2) - r_v_expectation(0)) * scalar;
		}
		// return jacobs

		// calculates error (possibly re-calculates, if running A-SLAM)
		//std::cout << "ln: " << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << ": " << (r_v_error.transpose() * m_t_sigma_inv).dot(r_v_error) << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 6> p_jacobi[2];
		Eigen::Matrix<double, 1, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi[0], p_jacobi[1], v_expectation, v_error);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge, in pixels.
	 */
	inline double f_Reprojection_Error() const
	{
		/*Eigen::Matrix<double, 2, 6> p_jacobi0;
		Eigen::Matrix<double, 2, 3> p_jacobi1;
		Eigen::Matrix<double, 2, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);*/
		// calculates the expectation, error and the jacobians
		//std::cerr << v_error[0] << " " << v_error[1] << " ----  " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;

		//Eigen::Vector2d v_error;
		//CBAJacobians::Project_P2C(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics(),
		//	m_p_vertex1->r_v_State() /* XYZ */, v_error);
		//v_error -= m_v_measurement; // this is actually negative error, but we only need the norm so sign does not matter
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return 0;// v_error.norm();
		/*return sqrt( (v_error[0] - v_expectation[0])*(v_error[0] - v_expectation[0]) +
				(v_error[1] - v_expectation[1])*(v_error[1] - v_expectation[1]) );*/
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
/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */

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

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdge3DSelf> {
public:
	typedef CEdgePose3DSelf _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdge3DTernary> {
public:
	typedef CEdgePoseTernary3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdge3DNormal> {
public:
	typedef CEdgeNormal3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for SE(3) solver (specialized for CParser::TEdge3D)
 */
template <>
class CSE3LandmarkPoseEdgeTraits<CParserBase::TEdgeSpline3D> {
public:
	typedef CBSplineEdge _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/** @} */ // end of group

#endif // !__SE3_TYPES_INCLUDED
