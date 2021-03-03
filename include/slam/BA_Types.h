/**
 *	@file include/slam/BA_Types.h
 *	@author Soso
 *	@date 2013
 *	@brief vertex and edge types for BA
 */

#pragma once
#ifndef __BA_TYPES_INCLUDED
#define __BA_TYPES_INCLUDED

#include "slam/BaseTypes.h"
//#include "slam/SE2_Types.h"
//#include "slam/SE3_Types.h" 
#include "slam/BASolverBase.h"
#include "slam/3DSolverBase.h" // that. want rot / arot
#include "slam/Parser.h" // parsed types passed to constructors
#include "slam/RobustUtils.h"

/** \addtogroup ba_group
 *	@{
 */

/*class CVertexIntrinsics : public CBaseVertexImpl<CVertexIntrinsics, 4> {
public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/ **
	 *	@brief default constructor; has no effect
	 * /
    inline CVertexPoseBA()
    {}

	/ **
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 * /
    inline CVertexIntrinsics(const Eigen::VectorXd &r_v_state)
        :CBaseVertexImpl<CVertexIntrinsics, 4>(r_v_state)
    {}

	/ **
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 * /
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
    	m_v_state.segment<4>() += r_v_delta.segment<4>(m_n_order);
    }
};*/

/**
 *	@brief bundle adjustment camera pose
 */
class CVertexCam : public CBaseVertexImpl<CVertexCam, 6> {
protected:
    Eigen::Matrix<double, 5, 1, Eigen::DontAlign> m_v_intrinsics; /**< @brief vertex cam should hold own camera params (these are not being optimized) */

public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexCam()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexCam(const Eigen::Matrix<double, 11, 1> &r_v_state)
        :CBaseVertexImpl<CVertexCam, 6>(r_v_state.head<6>()),
		m_v_intrinsics(r_v_state.tail<5>())
    {}

	/**
	 *	@brief constructor from optimized vector type (because of CBaseVertexImpl<CVertexCam, 6>)
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexCam(const Eigen::Matrix<double, 6, 1> &r_v_state)
        :CBaseVertexImpl<CVertexCam, 6>(r_v_state.head<6>())
    {
		m_v_intrinsics.setZero(); // we did not get this information
	}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is state of the vertex
	 */
    inline CVertexCam(const CParserBase::TVertexCam3D &r_t_vertex)
        :CBaseVertexImpl<CVertexCam, 6>(r_t_vertex.m_v_position.head<6>()),
		m_v_intrinsics(r_t_vertex.m_v_position.tail<5>())
    {}

	/**
	 *	@brief gets intrinsic parameters of the camera
	 *	@return Returns const reference to the parameters of the camera.
	 */
	inline const Eigen::Matrix<double, 5, 1, Eigen::DontAlign> &v_Intrinsics() const
	{
		return m_v_intrinsics;
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
		C3DJacobians::Relative_to_Absolute(m_v_state, r_v_delta.segment<6>(m_n_order), m_v_state);
		//CBAJacobians::Smart_Plus_Cam(m_v_state, r_v_delta.segment<6>(m_n_order), m_v_state);

        /*//focal lengths and principal points are added normally
    	//SOSO: ignore adding of fx and cx
    	//m_v_state.segment<4>(6) += r_v_delta.segment<4>(m_n_order + 6);

    	//pose is added as SE3
    	m_v_state.head<3>() += r_v_delta.segment<3>(m_n_order); // can select 3D segment starting at m_n_order; SSE accelerated

		//sum the rotations
		Eigen::Matrix3d pQ = C3DJacobians::t_AxisAngle_to_RotMatrix(m_v_state.tail<3>());
		Eigen::Matrix3d dQ = C3DJacobians::t_AxisAngle_to_RotMatrix(r_v_delta.segment<3>(m_n_order + 3));

		Eigen::Matrix3d QQ;// = pQ * dQ;
		QQ.noalias() = pQ * dQ; // noalias says that the destination is not the same as the multiplicands and that the multiplication can be carried out without allocating intermediate storage
		m_v_state.tail<3>() = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ); // also, no need for the intermediate object*/
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		//Operator_Plus(-r_v_delta); // ...
		//C3DJacobians::Relative_to_Absolute(m_v_state, -r_v_delta.segment<6>(m_n_order), m_v_state); // avoid calculating negative of the whole r_v_delta
		Eigen::Vector6d delta_inv;
		C3DJacobians::Absolute_to_Relative(r_v_delta.segment<6>(m_n_order), Eigen::Vector6d::Zero(), delta_inv);
		C3DJacobians::Relative_to_Absolute(m_v_state, delta_inv, m_v_state); // avoid calculating negative of the whole r_v_delta
	}
};

/**
 *	@brief bundle adjustment camera intrinsics
 */
class CVertexIntrinsics : public CBaseVertexImpl<CVertexIntrinsics, 5> {
public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexIntrinsics()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexIntrinsics(const Eigen::Matrix<double, 5, 1> &r_v_state)
        :CBaseVertexImpl<CVertexIntrinsics, 5>(r_v_state)
    {}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is state of the vertex
	 */
    inline CVertexIntrinsics(const CParserBase::TVertexIntrinsics &r_t_vertex)
        :CBaseVertexImpl<CVertexIntrinsics, 5>(r_t_vertex.m_v_position)
    {}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
    	// delta distortion coefficient must be normalized
    	double d = m_v_state(4) / (.5 * (m_v_state(0) * m_v_state(1)));
    	// transform to

    	d = d + r_v_delta(m_n_order + 4) / (.5 * (m_v_state(0) * m_v_state(1)));
    	m_v_state.head<4>() = m_v_state.head<4>() + r_v_delta.segment<4>(m_n_order);	// fx fy cx cy
    	// update

    	//m_v_state(4) = m_v_state(4) + r_v_delta(m_n_order + 4); //d * (.5 * (m_v_state(0) + m_v_state(1)));
    	m_v_state(4) = d * (.5 * (m_v_state(0) * m_v_state(1)));
    	// transform back

    	//m_v_state = m_v_state + r_v_delta.segment<5>(m_n_order);
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
    	// delta distortion coefficient must be normalized
		double d = m_v_state(4) / (.5 * (m_v_state(0) * m_v_state(1)));
		// transform to

		d = d - r_v_delta(m_n_order + 4) / (.5 * (m_v_state(0) * m_v_state(1)));
		m_v_state.head<4>() = m_v_state.head<4>() - r_v_delta.segment<4>(m_n_order);	// fx fy cx cy
		// update

		//m_v_state(4) = m_v_state(4) + r_v_delta(m_n_order + 4); //d * (.5 * (m_v_state(0) + m_v_state(1)));
		m_v_state(4) = d * (.5 * (m_v_state(0) * m_v_state(1)));
		// transform back

		//m_v_state = m_v_state + r_v_delta.segment<5>(m_n_order);
	}
};

/**
 *	@brief bundle adjustment camera intrinsics
 */
class CVertexIntrinsicsReduced : public CBaseVertexImpl<CVertexIntrinsicsReduced, 2> {
public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexIntrinsicsReduced()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexIntrinsicsReduced(const Eigen::Matrix<double, 2, 1, Eigen::DontAlign> &r_v_state)
        :CBaseVertexImpl<CVertexIntrinsicsReduced, 2>(r_v_state)
    {}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
    	// delta distortion coefficient must be normalized
    	double d = m_v_state(1) / (.5 * (m_v_state(0) * m_v_state(0)));
    	// transform to

    	d = d + r_v_delta(m_n_order + 1) / (.5 * (m_v_state(0) * m_v_state(0)));
    	m_v_state(0) = m_v_state(0) + r_v_delta(m_n_order);	// fx fy cx cy
    	// update

    	m_v_state(1) = d * (.5 * (m_v_state(0) * m_v_state(0)));
    	// transform back

    	//m_v_state = m_v_state + r_v_delta.segment<5>(m_n_order);
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
    	// delta distortion coefficient must be normalized
		double d = m_v_state(1) / (.5 * (m_v_state(0) * m_v_state(0)));
		// transform to

		d = d - r_v_delta(m_n_order + 1) / (.5 * (m_v_state(0) * m_v_state(0)));
		m_v_state(0) = m_v_state(0) - r_v_delta(m_n_order);	// fx fy cx cy
		// update

		m_v_state(1) = d * (.5 * (m_v_state(0) * m_v_state(0)));
		// transform back
	}
};

/**
 *	@brief bundle adjustment camera pose
 */
class CVertexSCam : public CBaseVertexImpl<CVertexSCam, 6> {
protected:
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> m_v_intrinsics; /**< @brief vertex cam should hold own camera params (these are not being optimized) */

public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexSCam()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexSCam(const Eigen::Matrix<double, 12, 1> &r_v_state)
        :CBaseVertexImpl<CVertexSCam, 6>(r_v_state.head<6>()),
		m_v_intrinsics(r_v_state.tail<6>())
    {}

	/**
	 *	@brief constructor from optimized vector type (because of CBaseVertexImpl<CVertexCam, 6>)
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexSCam(const Eigen::Matrix<double, 6, 1> &r_v_state)
        :CBaseVertexImpl<CVertexSCam, 6>(r_v_state.head<6>())
    {
		m_v_intrinsics.setZero(); // we did not get this information
	}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is state of the vertex
	 */
    inline CVertexSCam(const CParserBase::TVertexSCam3D &r_t_vertex)
        :CBaseVertexImpl<CVertexSCam, 6>(r_t_vertex.m_v_position.head<6>()),
		m_v_intrinsics(r_t_vertex.m_v_position.tail<6>())
    {}

	/**
	 *	@brief gets intrinsic parameters of the camera
	 *	@return Returns const reference to the parameters of the camera.
	 */
	inline const Eigen::Matrix<double, 6, 1, Eigen::DontAlign> &v_Intrinsics() const
	{
		return m_v_intrinsics;
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
		C3DJacobians::Relative_to_Absolute(m_v_state, r_v_delta.segment<6>(m_n_order), m_v_state);

        /*//focal lengths and principal points are added normally
    	//SOSO: ignore adding of fx and cx
    	//m_v_state.segment<4>(6) += r_v_delta.segment<4>(m_n_order + 6);

    	//pose is added as SE3
    	m_v_state.head<3>() += r_v_delta.segment<3>(m_n_order); // can select 3D segment starting at m_n_order; SSE accelerated

		//sum the rotations
		Eigen::Matrix3d pQ = C3DJacobians::t_AxisAngle_to_RotMatrix(m_v_state.tail<3>());
		Eigen::Matrix3d dQ = C3DJacobians::t_AxisAngle_to_RotMatrix(r_v_delta.segment<3>(m_n_order + 3));

		Eigen::Matrix3d QQ;// = pQ * dQ;
		QQ.noalias() = pQ * dQ; // noalias says that the destination is not the same as the multiplicands and that the multiplication can be carried out without allocating intermediate storage
		m_v_state.tail<3>() = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ); // also, no need for the intermediate object*/
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		//Operator_Plus(-r_v_delta); // ...
		//C3DJacobians::Relative_to_Absolute(m_v_state, -r_v_delta.segment<6>(m_n_order), m_v_state);
		Eigen::Vector6d delta_inv;
		C3DJacobians::Absolute_to_Relative(r_v_delta.segment<6>(m_n_order), Eigen::Vector6d::Zero(), delta_inv);
		C3DJacobians::Relative_to_Absolute(m_v_state, delta_inv, m_v_state); // avoid calculating negative of the whole r_v_delta
	}
};

/**
 *	@brief bundle adjustment camera pose
 */
class CVertexSpheron : public CBaseVertexImpl<CVertexSpheron, 6> {
public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexSpheron()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexSpheron(const Eigen::Matrix<double, 6, 1> &r_v_state)
        :CBaseVertexImpl<CVertexSpheron, 6>(r_v_state)
    {}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is state of the vertex
	 */
    inline CVertexSpheron(const CParserBase::TVertexSpheron &r_t_vertex)
        :CBaseVertexImpl<CVertexSpheron, 6>(r_t_vertex.m_v_position)
    {}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
		C3DJacobians::Relative_to_Absolute(m_v_state, r_v_delta.segment<6>(m_n_order), m_v_state);

       	/*//pose is added as SE3
    	m_v_state.head<3>() += r_v_delta.segment<3>(m_n_order); // can select 3D segment starting at m_n_order; SSE accelerated

		//sum the rotations
		Eigen::Matrix3d pQ = C3DJacobians::t_AxisAngle_to_RotMatrix(m_v_state.tail<3>());
		Eigen::Matrix3d dQ = C3DJacobians::t_AxisAngle_to_RotMatrix(r_v_delta.segment<3>(m_n_order + 3));

		Eigen::Matrix3d QQ;// = pQ * dQ;
		QQ.noalias() = pQ * dQ; // noalias says that the destination is not the same as the multiplicands and that the multiplication can be carried out without allocating intermediate storage
		m_v_state.tail<3>() = C3DJacobians::v_RotMatrix_to_AxisAngle(QQ); // also, no need for the intermediate object*/
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
	{
		//Operator_Plus(-r_v_delta); // ...
		//C3DJacobians::Relative_to_Absolute(m_v_state, -r_v_delta.segment<6>(m_n_order), m_v_state);
		Eigen::Vector6d delta_inv;
		C3DJacobians::Absolute_to_Relative(r_v_delta.segment<6>(m_n_order), Eigen::Vector6d::Zero(), delta_inv);
		C3DJacobians::Relative_to_Absolute(m_v_state, delta_inv, m_v_state); // avoid calculating negative of the whole r_v_delta
	}
};

/**
 *	@brief bundle adjustment (observed) keypoint vertex
 */
class CVertexXYZ : public CBaseVertexImpl<CVertexXYZ, 3> {
public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexXYZ()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexXYZ(const Eigen::Vector3d &r_v_state)
        :CBaseVertexImpl<CVertexXYZ, 3>(r_v_state)
    {}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is state of the vertex
	 */
    inline CVertexXYZ(const CParserBase::TVertexXYZ &r_t_vertex)
        :CBaseVertexImpl<CVertexXYZ, 3>(r_t_vertex.m_v_position)
    {}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
    {
    	//pose is added as SE3
    	m_v_state += r_v_delta.segment<3>(m_n_order); // can select 3D segment starting at m_n_order; SSE accelerated
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		//pose is added as SE3
		m_v_state -= r_v_delta.segment<3>(m_n_order); // can select 3D segment starting at m_n_order; SSE accelerated
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeCamCam : public CBaseEdgeImpl<CEdgeCamCam, MakeTypelist(CVertexCam, CVertexCam), 6> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeCamCam()
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
	CEdgeCamCam(const CParserBase::TEdgeP2C3D &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgeCamCam, MakeTypelist(CVertexCam, CVertexCam), 6>(r_t_edge.m_n_node_1,
		r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
		CBaseEdge::explicitly_initialized_vertices, r_system)
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
	CEdgeCamCam(size_t n_node0, size_t n_node1, const Eigen::Vector6d &v_delta, // won't align non-reference arg
		const Eigen::Matrix6d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:CBaseEdgeImpl<CEdgeCamCam, MakeTypelist(CVertexCam, CVertexCam), 6>(n_node0,
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
	inline void Update(const Eigen::Vector6d &r_v_delta, const Eigen::Matrix6d &r_t_inv_sigma)
	{
		CBaseEdgeImpl<CEdgeCamCam, MakeTypelist(CVertexCam, CVertexCam), 6>::Update(r_v_delta, r_t_inv_sigma);
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
		Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pose0;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), zero, pose0);
		Eigen::Matrix<double, 6, 1> pose1;
		C3DJacobians::Absolute_to_Relative(m_p_vertex1->r_v_State(), zero, pose1);

		C3DJacobians::Absolute_to_Relative(pose0, pose1, r_v_expectation);
		// calculates the expectation and the jacobians

		C3DJacobians::Absolute_to_Relative(r_v_expectation, m_v_measurement, r_v_error);
		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> d1, p_delta, p_delta_inv;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, zero, p_delta_inv);
			C3DJacobians::Absolute_to_Relative(p_delta_inv, pose1, d1);

			Eigen::Matrix<double, 6, 1> err;
			C3DJacobians::Absolute_to_Relative(r_v_expectation, d1, err);
			r_t_jacobian0.col(j) = err * scalar;

			Eigen::Matrix<double, 6, 1> d2;
			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, zero, p_delta_inv);
			C3DJacobians::Absolute_to_Relative(pose0, p_delta_inv, d2);

			C3DJacobians::Absolute_to_Relative(r_v_expectation, d2, err);
			r_t_jacobian1.col(j) = err * scalar;
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
		Eigen::Matrix<double, 6, 6> p_jacobi[2];
		Eigen::Matrix<double, 6, 1> v_expectation, v_error;
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

/**
 *	@brief BSpline 5xpose edge
 */
class CBCameraSplineEdge : public CBaseEdgeImpl<CBCameraSplineEdge, MakeTypelist(CVertexCam, CVertexCam, CVertexCam, CVertexCam, CVertexCam), 6, 6> {

public:
	typedef CBaseEdgeImpl<CBCameraSplineEdge, MakeTypelist(CVertexCam, CVertexCam, CVertexCam, CVertexCam, CVertexCam), 6, 6> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CBCameraSplineEdge()
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
	CBCameraSplineEdge(size_t n_node0, size_t n_node1, size_t n_node2, size_t n_node3, size_t n_node4,
			const Eigen::Matrix<double, 6, 1> &r_v_delta, const Eigen::Matrix<double, 6, 6> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node0, n_node1, n_node2, n_node3, n_node4), r_v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node0, CInitializeNullVertex<CVertexCam>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node1, CInitializeNullVertex<CVertexCam>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node2, CInitializeNullVertex<CVertexCam>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node3, CInitializeNullVertex<CVertexCam>());
		m_vertex_ptr.Get<4>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node4, CInitializeNullVertex<CVertexCam>());
		// get vertices (initialize if required)
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
			Eigen::Matrix<double, 6, 1> &r_v_expectation, Eigen::Matrix<double, 6, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		Eigen::Matrix<double, 6, 1> pose0, pose1, pose2, pose3, pose4;
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<0>()->r_v_State().head(6), pose0);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<1>()->r_v_State().head(6), pose1);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<2>()->r_v_State().head(6), pose2);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<3>()->r_v_State().head(6), pose3);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<4>()->r_v_State().head(6), pose4);

		// invert the poses
		BSplineSE3 spline(pose0, pose1, pose2, pose3);
		r_v_expectation = spline.bspline_error6D(m_v_measurement(0), pose4);
		// expectation error

		const double delta = 1e-3;	// smaller delta wont work ?? why? is the error function too ??
		const double scalar = 1.0 / (delta);
		Eigen::Matrix<double, 6, 6> Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack
		Eigen::Matrix<double, 6, 1> p_delta, inv_pose;

		for(size_t j = 0; j < 6; ++j) // four 1x6 jacobians
		{
			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<0>()->r_v_State().head(6), Eps.col(j), p_delta);
			C3DJacobians::Pose_Inverse(p_delta, inv_pose);
			BSplineSE3 spline0(inv_pose, pose1, pose2, pose3);
			r_t_jacobian_tuple.Get<0>().col(j) = (spline0.bspline_error6D(m_v_measurement(0), pose4) - r_v_expectation) * scalar;
			// J0

			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<1>()->r_v_State().head(6), Eps.col(j), p_delta);
			C3DJacobians::Pose_Inverse(p_delta, inv_pose);
			BSplineSE3 spline1(pose0, inv_pose, pose2, pose3);
			r_t_jacobian_tuple.Get<1>().col(j) = (spline1.bspline_error6D(m_v_measurement(0), pose4) - r_v_expectation) * scalar;
			// J1

			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<2>()->r_v_State().head(6), Eps.col(j), p_delta);
			C3DJacobians::Pose_Inverse(p_delta, inv_pose);
			BSplineSE3 spline2(pose0, pose1, inv_pose, pose3);
			r_t_jacobian_tuple.Get<2>().col(j) = (spline2.bspline_error6D(m_v_measurement(0), pose4) - r_v_expectation) * scalar;
			// J2

			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<3>()->r_v_State().head(6), Eps.col(j), p_delta);
			C3DJacobians::Pose_Inverse(p_delta, inv_pose);
			BSplineSE3 spline3(pose0, pose1, pose2, inv_pose);
			r_t_jacobian_tuple.Get<3>().col(j) = (spline3.bspline_error6D(m_v_measurement(0), pose4) - r_v_expectation) * scalar;
			// J3

			C3DJacobians::Relative_to_Absolute(m_vertex_ptr.Get<4>()->r_v_State().head(6), Eps.col(j), p_delta);
			C3DJacobians::Pose_Inverse(p_delta, inv_pose);
			r_t_jacobian_tuple.Get<4>().col(j) = (spline.bspline_error6D(m_v_measurement(0), inv_pose) - r_v_expectation) * scalar;
			// J4
		}
		// error is given by spline function

		r_v_error = - r_v_expectation;	// measurement should be zero
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 1> pose0, pose1, pose2, pose3, pose4;
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<0>()->r_v_State().head(6), pose0);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<1>()->r_v_State().head(6), pose1);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<2>()->r_v_State().head(6), pose2);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<3>()->r_v_State().head(6), pose3);
		C3DJacobians::Pose_Inverse(m_vertex_ptr.Get<4>()->r_v_State().head(6), pose4);
		// invert the poses
		BSplineSE3 spline(pose0, pose1, pose2, pose3);

		Eigen::Matrix<double, 6, 1> v_error = - spline.bspline_error6D(m_v_measurement(0), pose4);
		// calculates the expectation, error and the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeCamCamZDist : public CBaseEdgeImpl<CEdgeCamCamZDist, MakeTypelist(CVertexCam, CVertexCam), 1> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeCamCamZDist()
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
	CEdgeCamCamZDist(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 1, 1> &v_delta, // won't align non-reference arg
		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:CBaseEdgeImpl<CEdgeCamCamZDist, MakeTypelist(CVertexCam, CVertexCam), 1>(n_node0,
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
		CBaseEdgeImpl<CEdgeCamCamZDist, MakeTypelist(CVertexCam, CVertexCam), 1>::Update(r_v_delta, r_t_inv_sigma);
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
		Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pose0;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), zero, pose0);
		Eigen::Matrix<double, 6, 1> pose1;
		C3DJacobians::Absolute_to_Relative(m_p_vertex1->r_v_State(), zero, pose1);

		Eigen::Matrix<double, 6, 1> rel;
		C3DJacobians::Absolute_to_Relative(pose0, pose1, rel);
		// calculates the expectation and the jacobians

		r_v_expectation(0) = rel.head(3).norm(); // Z
		//if(rel(2) < 0)
		//	r_v_expectation(0) = -r_v_expectation(0);
		r_v_error(0) = m_v_measurement(0) - r_v_expectation(0); /* */

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		for(int j = 0; j < 6; ++ j) {
			Eigen::Matrix<double, 6, 1> d1, p_delta, p_delta_inv;
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, zero, p_delta_inv);
			C3DJacobians::Absolute_to_Relative(p_delta_inv, pose1, d1);

			//if(d1(2) >= 0)
			r_t_jacobian0(j) = (d1.head(3).norm() - r_v_expectation(0)) * scalar;
			//else
			//	r_t_jacobian0(j) = (-d1.head(3).norm() - r_v_expectation(0)) * scalar;

			Eigen::Matrix<double, 6, 1> d2;
			C3DJacobians::Relative_to_Absolute(m_p_vertex1->r_v_State(), Eps.col(j), p_delta);
			C3DJacobians::Absolute_to_Relative(p_delta, zero, p_delta_inv);
			C3DJacobians::Absolute_to_Relative(pose0, p_delta_inv, d2);

			//if(d2(2) >= 0)
			r_t_jacobian1(j) = (d2.head(3).norm() - r_v_expectation(0)) * scalar;
			//else
			//	r_t_jacobian1(j) = (-d2.head(3).norm() - r_v_expectation(0)) * scalar;
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

/**
 *	@brief Sim(3) pose roll edge
 */
class CEdgeXYZ : public CBaseEdgeImpl<CEdgeXYZ, MakeTypelist(CVertexCam), 3> {
public:
	typedef CBaseEdgeImpl<CEdgeXYZ, MakeTypelist(CVertexCam), 3> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeXYZ()
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
	CEdgeXYZ(size_t n_vertex0, const Eigen::Vector3d &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexCam>(n_vertex0, CInitializeNullVertex<>());
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
		Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pose0;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), zero, pose0);

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		Eigen::Matrix<double, 3, 6> &H1 = r_t_jacobian0;
		// can actually work inplace

		r_v_expectation = pose0.head<3>();

		Eigen::Matrix<double, 6, 1> p_delta;
		Eigen::Vector3d expectation2;

		//for XYZ and RPY
		for(int j = 0; j < 6; ++ j) {
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			Eigen::Matrix<double, 6, 1> posed;
			C3DJacobians::Absolute_to_Relative(p_delta, zero, posed);

			expectation2 = posed.head<3>();

			H1.col(j) = (expectation2 - r_v_expectation) * scalar;
		}
		//std::cout << "exp: " << m_v_measurement.transpose() << " <> " << pose0.head<3>().transpose() << std::endl;

		r_v_error = m_v_measurement - r_v_expectation;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pose0;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), zero, pose0);

		v_error = m_v_measurement - pose0.head<3>();
		//std::cout << "ch: " << m_v_measurement.transpose() << " <> " << pose0.head<3>().transpose() << std::endl;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeCamXYZ : public CBaseEdgeImpl<CEdgeCamXYZ, MakeTypelist(CVertexCam, CVertexXYZ), 3> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeCamXYZ()
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
	CEdgeCamXYZ(const CParserBase::TEdgeP2C3D &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgeCamXYZ, MakeTypelist(CVertexCam, CVertexXYZ), 6>(r_t_edge.m_n_node_0,
		r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
		CBaseEdge::explicitly_initialized_vertices, r_system)
	{}*/

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
	CEdgeCamXYZ(size_t n_node0, size_t n_node1, const Eigen::Vector3d &v_delta, // won't align non-reference arg
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:CBaseEdgeImpl<CEdgeCamXYZ, MakeTypelist(CVertexCam, CVertexXYZ), 3>(n_node0,
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
	inline void Update(const Eigen::Vector3d &r_v_delta, const Eigen::Matrix3d &r_t_inv_sigma)
	{
		CBaseEdgeImpl<CEdgeCamXYZ, MakeTypelist(CVertexCam, CVertexXYZ), 3>::Update(r_v_delta, r_t_inv_sigma);
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
		Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pose0;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), zero, pose0);

		const double delta = 1e-9;
		const double scalar = 1.0 / (delta);

		Eigen::Matrix<double, 6, 6> Eps;
		Eps = Eigen::Matrix<double, 6, 6>::Identity() * delta; // faster, all memory on stack

		Eigen::Matrix<double, 3, 6> &H1 = r_t_jacobian0;
		Eigen::Matrix<double, 3, 3> &H2 = r_t_jacobian1;
		// can actually work inplace

		r_v_expectation = pose0.head<3>() - m_p_vertex1->r_v_State();

		Eigen::Matrix<double, 6, 1> p_delta;
		Eigen::Vector3d p_delta2;
		Eigen::Vector3d expectation2;

		//for XYZ and RPY
		for(int j = 0; j < 6; ++ j) {
			C3DJacobians::Relative_to_Absolute(m_p_vertex0->r_v_State(), Eps.col(j), p_delta);
			Eigen::Matrix<double, 6, 1> posed;
			C3DJacobians::Absolute_to_Relative(p_delta, zero, posed);

			expectation2 = posed.head<3>() - m_p_vertex1->r_v_State();

			H1.col(j) = (expectation2 - r_v_expectation) * scalar;
		}
		for(int j = 0; j < 3; ++ j) {
			p_delta2 = m_p_vertex1->r_v_State() + Eps.col(j).head(3);

			expectation2 = pose0.head<3>() - p_delta2;

			H2.col(j) = (expectation2 - r_v_expectation) * scalar;
		}

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)

		std::cout << "exp " << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 1> pose0;
		C3DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(), zero, pose0);
		Eigen::Matrix<double, 3, 1> v_error = pose0.head<3>() - m_p_vertex1->r_v_State();
		// calculates the expectation, error and the jacobians
		std::cout << "ch " << pose0.head<3>().transpose() << " <> " << m_p_vertex1->r_v_State().transpose() << std::endl;

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

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2C3D : public CBaseEdgeImpl<CEdgeP2C3D, MakeTypelist(CVertexCam, CVertexXYZ), 2> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C3D()
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
	CEdgeP2C3D(const CParserBase::TEdgeP2C3D &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgeP2C3D, MakeTypelist(CVertexCam, CVertexXYZ), 2>(r_t_edge.m_n_node_1,
		r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
		CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexCam>(r_t_edge.m_n_node_1, CInitializeNullVertex<>());
		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexXYZ>());
		// ititialized by CBaseEdgeImpl constructor

		//std::swap(r_t_edge.m_n_node_1, r_t_edge.m_n_node_0);

		//fprintf(stderr, "verts: %d %d \n", r_t_edge.m_n_node_0, r_t_edge.m_n_node_1);
		//fprintf(stderr, ">>%f %f %f %f %f %f %f %f %f %f \n", m_p_vertex0->r_v_State()(0), m_p_vertex0->r_v_State()(1), m_p_vertex0->r_v_State()(2), m_p_vertex0->r_v_State()(3),
		//		m_p_vertex0->r_v_State()(4), m_p_vertex0->r_v_State()(5), m_p_vertex0->r_v_State()(6), m_p_vertex0->r_v_State()(7), m_p_vertex0->r_v_State()(8), m_p_vertex0->r_v_State()(9));

		//fprintf(stderr, "dims: %d %d \n", (m_p_vertex0)->n_Dimension(), (m_p_vertex1)->n_Dimension());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

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
	CEdgeP2C3D(size_t n_node1, size_t n_node0, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:CBaseEdgeImpl<CEdgeP2C3D, MakeTypelist(CVertexCam, CVertexXYZ), 2>(n_node0,
		n_node1, v_delta, r_t_inv_sigma, CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexCam>(n_node0, CInitializeNullVertex<>());
		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node1, CInitializeNullVertex<CVertexXYZ>());
		// get vertices (initialize if required)
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"
		// ititialized by CBaseEdgeImpl constructor

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
	inline void Update(const CParserBase::TEdgeP2C3D &r_t_edge)
	{
		CBaseEdgeImpl<CEdgeP2C3D, MakeTypelist(CVertexCam, CVertexXYZ),
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
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 6> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 3> &r_t_jacobian1, Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2C(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics() /* camera intrinsic params */,
			m_p_vertex1->r_v_State() /* XYZ */, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() << std::endl;
		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		/*Eigen::Matrix<double, 2, 6> p_jacobi0;
		Eigen::Matrix<double, 2, 3> p_jacobi1;
		Eigen::Matrix<double, 2, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
		// calculates the expectation, error and the jacobians
		//std::cerr << v_error[0] << " " << v_error[1] << " ----  " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;
		//std::cout << sqrt( v_error[0]*v_error[0] + v_error[1]*v_error[1] ) << std::endl;*/

		Eigen::Vector2d v_error;
		CBAJacobians::Project_P2C(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics(),
			m_p_vertex1->r_v_State() /* XYZ */, v_error);
		//std::cout << v_error.transpose() << " | " << m_v_measurement.transpose() << std::endl;

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
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

		Eigen::Vector2d v_error;
		CBAJacobians::Project_P2C(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics(),
			m_p_vertex1->r_v_State() /* XYZ */, v_error);
		v_error -= m_v_measurement; // this is actually negative error, but we only need the norm so sign does not matter
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
		/*return sqrt( (v_error[0] - v_expectation[0])*(v_error[0] - v_expectation[0]) +
				(v_error[1] - v_expectation[1])*(v_error[1] - v_expectation[1]) );*/
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI3D : public CBaseEdgeImpl<CEdgeP2CI3D, MakeTypelist(CVertexCam, CVertexXYZ, CVertexIntrinsics), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI3D, MakeTypelist(CVertexCam, CVertexXYZ, CVertexIntrinsics), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI3D()
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
	CEdgeP2CI3D(const CParserBase::TEdgeP2CI3D &r_t_edge, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(r_t_edge.m_n_node_1,
		r_t_edge.m_n_node_0, r_t_edge.m_n_node_2), r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
		CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexCam>(r_t_edge.m_n_node_1, CInitializeNullVertex<>());
		//m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexXYZ>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexXYZ>());
		//m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(r_t_edge.m_n_node_2, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)
		// initialized already in CBaseEdgeImpl constructor

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_2].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node (camera)
	 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node (xyz vertex)
	 *	@param[in] n_node_2 is (zero-based) index of the third (endpoint) node (intrinsics vertex)
	 *	@param[in] v_delta is vector of delta x position, delta y-position
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgeP2CI3D(size_t n_node_0, size_t n_node_1, size_t n_node_2, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node_0,
		n_node_1, n_node_2), v_delta, r_t_inv_sigma,
		CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node_0, CInitializeNullVertex<>());
		//m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node_1, CInitializeNullVertex<CVertexXYZ>());
		//m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_node_2, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)
		// initialized already in CBaseEdgeImpl constructor

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_1].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_0].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_2].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const CParserBase::TEdgeP2CI3D &r_t_edge)
	{
		_TyBase::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple structure holding all jacobians
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2CI(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<2>()->v_State(),
			m_vertex_ptr.Get<1>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		//std::cout << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector2d v_error;
		CBAJacobians::Project_P2C(m_vertex_ptr.Get<0>()->v_State() /* Cam */, m_vertex_ptr.Get<2>()->v_State(),
			m_vertex_ptr.Get<1>()->v_State() /* XYZ */, v_error);

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge, in pixels.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_expectation;
		CBAJacobians::Project_P2C(m_vertex_ptr.Get<0>()->v_State() /* Cam */, m_vertex_ptr.Get<2>()->v_State(),
			m_vertex_ptr.Get<1>()->v_State() /* XYZ */, v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2SC3D : public CBaseEdgeImpl<CEdgeP2SC3D, MakeTypelist(CVertexSCam, CVertexXYZ), 3> {
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2SC3D()
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
	CEdgeP2SC3D(const CParserBase::TEdgeP2SC3D &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgeP2SC3D, MakeTypelist(CVertexSCam, CVertexXYZ), 3>(r_t_edge.m_n_node_1,
		r_t_edge.m_n_node_0, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
		CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSCam>(r_t_edge.m_n_node_1, CInitializeNullVertex<>());
		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexXYZ>());
		// initialized already in CBaseEdgeImpl constructor

		//std::swap(r_t_edge.m_n_node_1, r_t_edge.m_n_node_0);

		//fprintf(stderr, "verts: %d %d \n", r_t_edge.m_n_node_0, r_t_edge.m_n_node_1);
		//fprintf(stderr, ">>%f %f %f %f %f %f %f %f %f %f \n", m_p_vertex0->r_v_State()(0), m_p_vertex0->r_v_State()(1), m_p_vertex0->r_v_State()(2), m_p_vertex0->r_v_State()(3),
		//		m_p_vertex0->r_v_State()(4), m_p_vertex0->r_v_State()(5), m_p_vertex0->r_v_State()(6), m_p_vertex0->r_v_State()(7), m_p_vertex0->r_v_State()(8), m_p_vertex0->r_v_State()(9));

		//fprintf(stderr, "dims: %d %d \n", (m_p_vertex0)->n_Dimension(), (m_p_vertex1)->n_Dimension());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

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
	CEdgeP2SC3D(size_t n_node1, size_t n_node0, const Eigen::Vector3d &v_delta, // won't align non-reference arg
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:CBaseEdgeImpl<CEdgeP2SC3D, MakeTypelist(CVertexSCam, CVertexXYZ), 3>(n_node0,
		n_node1, v_delta, r_t_inv_sigma, CBaseEdge::explicitly_initialized_vertices, r_system)
	{
		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSCam>(n_node0, CInitializeNullVertex<>());
		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node1, CInitializeNullVertex<CVertexXYZ>());
		// get vertices (initialize if required)
		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"
		// initialized already in CBaseEdgeImpl constructor

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
	inline void Update(const CParserBase::TEdgeP2SC3D &r_t_edge)
	{
		CBaseEdgeImpl<CEdgeP2SC3D, MakeTypelist(CVertexSCam, CVertexXYZ),
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
		CBAJacobians::Project_P2SC(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics() /* camera intrinsic params */,
			m_p_vertex1->r_v_State() /* XYZ */, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << "me: " << m_v_measurement[0] << " " << m_v_measurement[1] << " " << m_v_measurement[2] << " ----  " <<
		//		r_v_expectation[0] << " " << r_v_expectation[1] << " " << r_v_expectation[2] << std::endl;
		//std::cout << "J0:" << r_t_jacobian0 << std::endl;
		//std::cout << "J1:" << r_t_jacobian1 << std::endl;

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 3, 6> p_jacobi0;
		Eigen::Matrix<double, 3, 3> p_jacobi1;
		Eigen::Matrix<double, 3, 1> v_error;
		//Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
		CBAJacobians::Project_P2SC(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics(),
				m_p_vertex1->r_v_State() /* XYZ */, v_error);
		// calculates the expectation, error and the jacobians
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge, in pixels.
	 */
	inline double f_Reprojection_Error() const
	{
		/*Eigen::Matrix<double, 2, 6> p_jacobi0;
		Eigen::Matrix<double, 2, 3> p_jacobi1;
		Eigen::Matrix<double, 3, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
		// calculates the expectation, error and the jacobians
		//std::cerr << v_error[0] << " " << v_error[1] << " ----  " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;

		return sqrt( (v_error[0] - v_expectation[0])*(v_error[0] - v_expectation[0]) +
				(v_error[1] - v_expectation[1])*(v_error[1] - v_expectation[1]) );*/

		return 0;
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI3D_LS : public CBaseEdgeImpl<CEdgeP2CI3D_LS, MakeTypelist(CVertexXYZ, CVertexIntrinsics), 2> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI3D_LS, MakeTypelist(CVertexXYZ, CVertexIntrinsics), 2> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI3D_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node_0 is (zero-based) index of the first (endpoint) node (xyz vertex)
	 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node (intrinsics vertex)
	 *	@param[in] v_delta is vector of delta x position, delta y-position
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgeP2CI3D_LS(size_t n_node_0, size_t n_node_1, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(n_node_0, n_node_1, v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node_0, CInitializeNullVertex<CVertexXYZ>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_node_1, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)
		// initialized already in CBaseEdgeImpl constructor

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_0].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_1].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple structure holding all jacobians
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 2, 5> &r_t_jacobian1,	Eigen::Matrix<double, 2, 1> &r_v_expectation,
			Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2CI_Self(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			r_v_expectation, r_t_jacobian0,	r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " " << r_v_expectation.transpose() << std::endl;
		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector2d v_error;
		Eigen::Vector6d cam_zero = Eigen::Vector6d::Zero();	// cam in origin with no rotation
		CBAJacobians::Project_P2C(cam_zero, m_p_vertex1->r_v_State(), m_p_vertex0->r_v_State(), v_error);

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge, in pixels.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_expectation;
		Eigen::Vector6d cam_zero = Eigen::Vector6d::Zero();	// cam in origin with no rotation
		CBAJacobians::Project_P2C(cam_zero, m_p_vertex1->r_v_State(), m_p_vertex0->r_v_State(), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_expectation;
		Eigen::Vector6d cam_zero = Eigen::Vector6d::Zero();	// cam in origin with no rotation
		CBAJacobians::Project_P2C(cam_zero, m_p_vertex1->r_v_State(), m_p_vertex0->r_v_State(), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI3D_LO : public CBaseEdgeImpl<CEdgeP2CI3D_LO, MakeTypelist(CVertexXYZ, CVertexCam, CVertexIntrinsics, CVertexCam), 2> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI3D_LO, MakeTypelist(CVertexXYZ, CVertexCam, CVertexIntrinsics, CVertexCam), 2> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI3D_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node_0 is (zero-based) index of the first node (xyz)
	 *	@param[in] n_node_1 is (zero-based) index of the second node (observer camera)
	 *	@param[in] n_node_2 is (zero-based) index of the third node (intrinsics)
	 *	@param[in] n_node_3 is (zero-based) index of the fourth node (owner camera)
	 *	@param[in] v_delta is vector of delta x position, delta y-position
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgeP2CI3D_LO(size_t n_node_0, size_t n_node_1, size_t n_node_2, size_t n_node_3, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node_0,
		n_node_1, n_node_2, n_node_3), v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node_0, CInitializeNullVertex<CVertexXYZ>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node_1, CInitializeNullVertex<CVertexCam>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_node_2, CInitializeNullVertex<CVertexIntrinsics>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node_3, CInitializeNullVertex<CVertexCam>());
		// get vertices (initialize if required)
		// initialized already in CBaseEdgeImpl constructor

		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_1].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_0].n_Dimension() == 6);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_node_2].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple structure holding all jacobians
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2CI_Other(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
			m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
		// calculates the expectation and the jacobians

		//std::cout << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector2d v_error;
		CBAJacobians::Project_P2C_Other(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_error);

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge, in pixels.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_expectation;
		CBAJacobians::Project_P2C_Other(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_expectation;
		CBAJacobians::Project_P2C_Other(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

///**
// *	@brief Camera-to-Camera Epipolar constraint edge (C2CE)
// *
// *	This edge represents epipolar constraint
// *	between two 2D images \f$p0,p1\f$ of a 3D point:
// *	\f{equation}{ p1^T * F * p0 = 0 \f}
// *	where F is a fundamental matrix: \f$F = K2 * E * K1 \f$, \f$E = [t]_x * R \f$,
// *	\f$K\f$ is a calibration matrix, and \f$[R|t]\f$
// *	is a relative translation between cameras.
// */
//class CEdgeC2CE : public CBaseEdgeImpl<CEdgeC2CE, MakeTypelist(CVertexCam, CVertexCam), 1, 4> {
//public:
//	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this
//
//	/**
//	 *	@brief default constructor; has no effect
//	 */
//	inline CEdgeC2CE()
//	{}
//
//	/**
//	 *	@brief constructor; converts parsed edge to edge representation
//	 *
//	 *	@tparam CSystem is type of system where this edge is being stored
//	 *
//	 *	@param[in] r_t_edge is parsed edge
//	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
//	 */
//	template <class CSystem>
//	CEdgeC2CE(const CParserBase::TEdgeC2CE &r_t_edge, CSystem &r_system)
//		:CBaseEdgeImpl<CEdgeC2CE, MakeTypelist(CVertexCam, CVertexCam), 1, 4>(r_t_edge.m_n_node_0,
//		r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
//		CBaseEdge::explicitly_initialized_vertices, r_system)
//	{}
//
//	/**
//	 *	@brief constructor; initializes edge with data
//	 *
//	 *	@tparam CSystem is type of system where this edge is being stored
//	 *
//	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node (camera)
//	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node (camera)
//	 *	@param[in] v_delta contains corresponding 2D points coordinates - [x0, y0, x1, y1]
//	 *	@param[in] r_t_inv_sigma is the information matrix
//	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
//	 */
//	template <class CSystem>
//	CEdgeC2CE(size_t n_node0, size_t n_node1, const Eigen::Vector4d &v_delta, // won't align non-reference arg
//		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
//		:CBaseEdgeImpl<CEdgeC2CE, MakeTypelist(CVertexCam, CVertexCam), 1, 4>(n_node0,
//		n_node1, v_delta, r_t_inv_sigma, CBaseEdge::explicitly_initialized_vertices, r_system)
//	{}
//
//	/**
//	 *	@brief updates the edge with a new measurement
//	 *
//	 *	@param[in] r_t_edge is parsed edge
//	 */
//	inline void Update(const CParserBase::TEdgeC2CE &r_t_edge)
//	{
//		CBaseEdgeImpl<CEdgeC2CE, MakeTypelist(CVertexCam, CVertexCam),
//			1, 4>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
//	}
//
//	/**
//	 *	@brief calculates jacobians, expectation and error
//	 *
//	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
//	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second vertex
//	 *	@param[out] r_v_expectation is input vector, expectation is zero error
//	 *	@param[out] r_v_error is error vector
//	 */
//	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 1, 6> &r_t_jacobian0,
//		Eigen::Matrix<double, 1, 6> &r_t_jacobian1, Eigen::Matrix<double, 4, 1> &r_v_expectation,
//		Eigen::Matrix<double, 1, 1> &r_v_error) const // change dimensionality of eigen types, if required
//	{
//		Eigen::Vector4d input = m_v_measurement;
//
//		// expectation is zero, here r_v_expectation is input
//		CBAJacobians::Project_PC2CE(m_p_vertex0->r_v_State() /* Cam0 */, m_p_vertex1->r_v_State() /* Cam1 */,
//				m_p_vertex0->v_Intrinsics() /* camera intrinsic params 0 */, m_p_vertex1->v_Intrinsics() /* camera intrinsic params 1 */,
//				input, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
//		// calculates the expectation and the jacobians
//
//		// error is stored in expectation
//		r_v_error = r_v_expectation;
//		//std::cout << m_p_vertex0->r_v_State().transpose() << " | " << m_p_vertex1->r_v_State().transpose() << std::endl;
//		//std::cout << "err: " << r_v_error.transpose() << std::endl;
//		// calculates error (possibly re-calculates, if running A-SLAM)
//	}
//
//	/**
//	 *	@brief calculates \f$\chi^2\f$ error
//	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
//	 */
//	inline double f_Chi_Squared_Error() const
//	{
//		/*Eigen::Matrix<double, 2, 6> p_jacobi0;
//		Eigen::Matrix<double, 2, 3> p_jacobi1;
//		Eigen::Matrix<double, 2, 1> v_expectation, v_error;
//		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
//		// calculates the expectation, error and the jacobians
//		//std::cerr << v_error[0] << " " << v_error[1] << " ----  " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;
//		//std::cout << sqrt( v_error[0]*v_error[0] + v_error[1]*v_error[1] ) << std::endl;*/
//
//		Eigen::Vector4d input = m_v_measurement;
//		Eigen::Matrix<double, 1, 1> v_error;
//		CBAJacobians::Project_PC2CE(m_p_vertex0->r_v_State() /* Cam0 */, m_p_vertex1->r_v_State() /* Cam1 */,
//					m_p_vertex0->v_Intrinsics() /* camera intrinsic params 0 */, m_p_vertex1->v_Intrinsics() /* camera intrinsic params 1 */,
//					input, v_error);
//		//std::cout << v_error.transpose() << " | " << m_v_measurement.transpose() << std::endl;
//
//		//v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
//		// this should be faster, as it does not calculate the jacobians
//		// (in BA, this is actually timed as it is used in solver step validation)
//		//std::cout << m_p_vertex0->r_v_State().transpose() << " | " << m_p_vertex1->r_v_State().transpose() << std::endl;
//		//std::cout << "err: " << v_error.transpose() << " chi: " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;
//
//		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
//		//return 0;
//	}
//
//	/**
//	 *	@brief calculates reprojection error
//	 *	@return Returns reprojection error for this edge, in pixels.
//	 */
//	inline double f_Reprojection_Error() const
//	{
//		//Eigen::Vector2d v_error;
//		//CBAJacobians::Project_P2C(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics(),
//		//	m_p_vertex1->r_v_State() /* XYZ */, v_error);
//		//v_error -= m_v_measurement; // this is actually negative error, but we only need the norm so sign does not matter
//		// this should be faster, as it does not calculate the jacobians
//		// (in BA, this is actually timed as it is used in solver step validation)
//
//		//return v_error.norm();
//		return 0;
//	}
//};

/**
 *	@brief edge for 3D landmark observation by a spherical camera
 *	@note This is formally a SLAM edge, not BA.
 */
class CEdgeSpheronXYZ : public CBaseEdgeImpl<CEdgeSpheronXYZ, MakeTypelist(CVertexSpheron, CVertexXYZ), 3> {
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
		inline operator CVertexXYZ() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
		{
			Eigen::Matrix<double, 6, 1> v_pose2;
			Eigen::Matrix<double, 6, 1> relative_pose;
			relative_pose << m_r_v_delta(0), m_r_v_delta(1), m_r_v_delta(2), 0, 0, 0;	//dummy rotation
			/*std::cout << "p1: " << m_r_v_pose1(0) << " " << m_r_v_pose1(1) << " " << m_r_v_pose1(2) << " " << m_r_v_pose1(3) << " " <<
					m_r_v_pose1(4) << " " << m_r_v_pose1(5) << std::endl;
			std::cout << "p2: " << relative_pose(0) << " " << relative_pose(1) << " " << relative_pose(2) << " " << relative_pose(3) << " " <<
					relative_pose(4) << " " << relative_pose(5) << std::endl;*/
			C3DJacobians::Relative_to_Absolute(m_r_v_pose1, relative_pose, v_pose2); // implement your own equation here
			/*std::cout << "res: " << v_pose2(0) << " " << v_pose2(1) << " " << v_pose2(2) << " " << v_pose2(3) << " " <<
					v_pose2(4) << " " << v_pose2(5) << std::endl;*/
			return CVertexXYZ(v_pose2.segment<3>(0));
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeSpheronXYZ()
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
	CEdgeSpheronXYZ(const CParserBase::TEdgeSpheronXYZ &r_t_edge, CSystem &r_system)
		:CBaseEdgeImpl<CEdgeSpheronXYZ, MakeTypelist(CVertexSpheron, CVertexXYZ), 3>(r_t_edge.m_n_node_0,
		r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSpheron>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(r_t_edge.m_n_node_1,
			CRelative_to_Absolute_XYZ_Initializer(m_p_vertex0->r_v_State(), r_t_edge.m_v_delta));
		// get vertices (initialize if required)

		//print me edge
		//std::cout << r_t_edge.m_n_node_0 << " to " << r_t_edge.m_n_node_1 << std::endl;
		//std::cout << r_t_edge.m_v_delta << std::endl;

		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 6);
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
	CEdgeSpheronXYZ(size_t n_node0, size_t n_node1, const Eigen::Matrix<double, 3, 1> &r_v_delta,
		const Eigen::Matrix<double, 3, 3> &r_t_inv_sigma, CSystem &r_system)
		:CBaseEdgeImpl<CEdgeSpheronXYZ, MakeTypelist(CVertexSpheron, CVertexXYZ), 3>(n_node0,
		n_node1, r_v_delta, r_t_inv_sigma)
	{
		//fprintf(stderr, "%f %f %f\n", r_t_edge.m_v_delta(0), r_t_edge.m_v_delta(1), r_t_edge.m_v_delta(2));

		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSpheron>(n_node0, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node1,
			CRelative_to_Absolute_XYZ_Initializer(m_p_vertex0->r_v_State(), r_v_delta));
		// get vertices (initialize if required)

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
	inline void Update(const CParserBase::TEdgeSpheronXYZ &r_t_edge)
	{
		CBaseEdgeImpl<CEdgeSpheronXYZ, MakeTypelist(CVertexSpheron, CVertexXYZ),
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

		/*std::cout << "v1: " << m_p_vertex0->r_v_State()(0) << " " << m_p_vertex0->r_v_State()(1) << " " <<
				m_p_vertex0->r_v_State()(2) << " " << m_p_vertex0->r_v_State()(3) << " " << m_p_vertex0->r_v_State()(4) << " " <<
				m_p_vertex0->r_v_State()(5) << std::endl;
		std::cout << "v2: " << m_p_vertex1->r_v_State()(0) << " " << m_p_vertex1->r_v_State()(1) << " " <<
						m_p_vertex1->r_v_State()(2) << std::endl;*/
		//std::cout << "expect: " << r_v_expectation(0) << " " << r_v_expectation(1) << " " << r_v_expectation(2) << " -- ";
		///std::cout << m_v_measurement(0) << " " << m_v_measurement(1) << " " << m_v_measurement(2) << std::endl;

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
		/*Eigen::Matrix<double, 3, 6> p_jacobi0;
		Eigen::Matrix<double, 3, 3> p_jacobi1;
		Eigen::Matrix<double, 3, 1> v_expectation, v_error;
		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);*/
		// calculates the expectation, error and the jacobians

		//std::cout << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;

		Eigen::Matrix<double, 3, 1> v_error;
		C3DJacobians::Absolute_to_Relative_Landmark(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), v_error);

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}
};

/**
 *	@brief Edge connecting two cameras through Fundamental matrix and 2 points
 */
//class CEdgeF : public CBaseEdgeImpl<CEdgeF, MakeTypelist(CVertexCam, CVertexIntrinsics), 4> {
//public:
//	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this
//
//	/**
//	 *	@brief default constructor; has no effect
//	 */
//	inline CEdgeF()
//	{}
//
//	/**
//	 *	@brief constructor; converts parsed edge to edge representation
//	 *
//	 *	@tparam CSystem is type of system where this edge is being stored
//	 *
//	 *	@param[in] r_t_edge is parsed edge
//	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
//	 */
//	/*template <class CSystem>
//	CEdgeF(const CParserBase::TEdgeP2SC3D &r_t_edge, CSystem &r_system)
//		:CBaseEdgeImpl<CEdgeF, MakeTypelist(CVertexCam, CVertexIntrinsics), 4>(r_t_edge.m_n_cam_0,
//				r_t_edge.m_n_intrinsics_0, r_t_edge.m_n_cam_1, r_t_edge.m_n_intrinsics_1,
//				r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma,
//		CBaseEdge::explicitly_initialized_vertices, r_system)
//	{
//		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSCam>(r_t_edge.m_n_node_1, CInitializeNullVertex<>());
//		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(r_t_edge.m_n_node_0, CInitializeNullVertex<CVertexXYZ>());
//		// initialized already in CBaseEdgeImpl constructor
//
//		//std::swap(r_t_edge.m_n_node_1, r_t_edge.m_n_node_0);
//
//		//fprintf(stderr, "verts: %d %d \n", r_t_edge.m_n_node_0, r_t_edge.m_n_node_1);
//		//fprintf(stderr, ">>%f %f %f %f %f %f %f %f %f %f \n", m_p_vertex0->r_v_State()(0), m_p_vertex0->r_v_State()(1), m_p_vertex0->r_v_State()(2), m_p_vertex0->r_v_State()(3),
//		//		m_p_vertex0->r_v_State()(4), m_p_vertex0->r_v_State()(5), m_p_vertex0->r_v_State()(6), m_p_vertex0->r_v_State()(7), m_p_vertex0->r_v_State()(8), m_p_vertex0->r_v_State()(9));
//
//		//fprintf(stderr, "dims: %d %d \n", (m_p_vertex0)->n_Dimension(), (m_p_vertex1)->n_Dimension());
//		// get vertices (initialize if required)
//
//		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_1].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
//		//_ASSERTE(r_system.r_Vertex_Pool()[r_t_edge.m_n_node_0].n_Dimension() == 3);
//		// make sure the dimensionality is correct (might not be)
//		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
//	}*/
//
//	/**
//	 *	@brief constructor; initializes edge with data
//	 *
//	 *	@tparam CSystem is type of system where this edge is being stored
//	 *
//	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node (camera)
//	 *	@param[in] n_node1 is (zero-based) index of the second (endpoint) node (vertex)
//	 *	@param[in] v_delta is vector of delta x position, delta y-position
//	 *	@param[in] r_t_inv_sigma is the information matrix
//	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
//	 */
//	template <class CSystem>
//	CEdgeF(size_t n_cam0, size_t n_intr0, size_t n_cam1, size_t n_intr1, const Eigen::Vector4d &v_delta, // won't align non-reference arg
//		const Eigen::Matrix4d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
//		:CBaseEdgeImpl<CEdgeF, MakeTypelist(CVertexCam, CVertexIntrinsics), 4>(n_cam0, n_intr0,
//		n_cam1, n_intr1, v_delta, r_t_inv_sigma, CBaseEdge::explicitly_initialized_vertices, r_system)
//	{
//		//m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSCam>(n_node0, CInitializeNullVertex<>());
//		//m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_node1, CInitializeNullVertex<CVertexXYZ>());
//		// get vertices (initialize if required)
//		// "template" is required by g++, otherwise gives "expected primary-expression before '>' token"
//		// initialized already in CBaseEdgeImpl constructor
//
//		//_ASSERTE(r_system.r_Vertex_Pool()[n_node0].n_Dimension() == 6); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
//		//_ASSERTE(r_system.r_Vertex_Pool()[n_node1].n_Dimension() == 3);
//		// make sure the dimensionality is correct (might not be)
//		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
//	}
//
//	/**
//	 *	@brief updates the edge with a new measurement
//	 *
//	 *	@param[in] r_t_edge is parsed edge
//	 */
//	inline void Update(const CParserBase::TEdgeF &r_t_edge)
//	{
//		CBaseEdgeImpl<CEdgeF, MakeTypelist(CVertexCam, CVertexIntrinsics),
//				4>::Update(r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
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
//	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 6> &r_t_jacobian0,
//		Eigen::Matrix<double, 3, 3> &r_t_jacobian1, Eigen::Matrix<double, 3, 1> &r_v_expectation,
//		Eigen::Matrix<double, 3, 1> &r_v_error) const // change dimensionality of eigen types, if required
//	{
//		CBAJacobians::Project_P2SC(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics() /* camera intrinsic params */,
//			m_p_vertex1->r_v_State() /* XYZ */, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
//		// calculates the expectation and the jacobians
//
//		//std::cout << "me: " << m_v_measurement[0] << " " << m_v_measurement[1] << " " << m_v_measurement[2] << " ----  " <<
//		//		r_v_expectation[0] << " " << r_v_expectation[1] << " " << r_v_expectation[2] << std::endl;
//		//std::cout << "J0:" << r_t_jacobian0 << std::endl;
//		//std::cout << "J1:" << r_t_jacobian1 << std::endl;
//
//		r_v_error = m_v_measurement - r_v_expectation;
//		// calculates error (possibly re-calculates, if running A-SLAM)
//	}
//
//	/**
//	 *	@brief calculates \f$\chi^2\f$ error
//	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
//	 */
//	inline double f_Chi_Squared_Error() const
//	{
//		Eigen::Matrix<double, 3, 6> p_jacobi0;
//		Eigen::Matrix<double, 3, 3> p_jacobi1;
//		Eigen::Matrix<double, 3, 1> v_error;
//		//Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
//		CBAJacobians::Project_P2SC(m_p_vertex0->r_v_State() /* Cam */, m_p_vertex0->v_Intrinsics(),
//				m_p_vertex1->r_v_State() /* XYZ */, v_error);
//		// calculates the expectation, error and the jacobians
//		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
//		// this should be faster, as it does not calculate the jacobians
//
//		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
//	}
//
//	/**
//	 *	@brief calculates reprojection error
//	 *	@return Returns reprojection error for this edge, in pixels.
//	 */
//	inline double f_Reprojection_Error() const
//	{
//		/*Eigen::Matrix<double, 2, 6> p_jacobi0;
//		Eigen::Matrix<double, 2, 3> p_jacobi1;
//		Eigen::Matrix<double, 3, 1> v_expectation, v_error;
//		Calculate_Jacobians_Expectation_Error(p_jacobi0, p_jacobi1, v_expectation, v_error);
//		// calculates the expectation, error and the jacobians
//		//std::cerr << v_error[0] << " " << v_error[1] << " ----  " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << std::endl;
//
//		return sqrt( (v_error[0] - v_expectation[0])*(v_error[0] - v_expectation[0]) +
//				(v_error[1] - v_expectation[1])*(v_error[1] - v_expectation[1]) );*/
//
//		return 0;
//	}
//};

/** @} */ // end of group

/** \addtogroup parser
 *	@{
 */

/**
 *	@brief edge traits for BA solver
 */
template <class CParsedStructure>
class CBAEdgeTraits {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief it should fail on unknown edge types */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "unknown edge type occurred 1";
	}
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAEdgeTraits<CParserBase::TEdgeP2C3D> {
public:
	typedef CEdgeP2C3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

///**
// *	@brief edge traits for BA solver
// */
//template <>
//class CBAEdgeTraits<CParserBase::TEdgeC2CE> {
//public:
//	typedef CEdgeC2CE _TyEdge; /**< @brief the edge type to construct from the parsed type */
//};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAEdgeTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAEdgeTraits<CParserBase::TVertexCam3D> {
public:
	typedef CVertexCam _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <class CParsedStructure>
class CBAVertexTraits {
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
 *	@brief vertex traits for BA solver
 */
template <>
class CBAVertexTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <>
class CBAVertexTraits<CParserBase::TVertexCam3D> {
public:
	typedef CVertexCam _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <class CParsedStructure>
class CBASEdgeTraits {
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
 *	@brief edge traits for BA solver
 */
template <>
class CBASEdgeTraits<CParserBase::TEdgeP2SC3D> {
public:
	typedef CEdgeP2SC3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBASEdgeTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBASEdgeTraits<CParserBase::TVertexSCam3D> {
public:
	typedef CVertexSCam _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <class CParsedStructure>
class CBASVertexTraits {
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
 *	@brief vertex traits for BA solver
 */
template <>
class CBASVertexTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <>
class CBASVertexTraits<CParserBase::TVertexSCam3D> {
public:
	typedef CVertexSCam _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <class CParsedStructure>
class CSpheronEdgeTraits {
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
 *	@brief edge traits for Spheron solver
 */
template <>
class CSpheronEdgeTraits<CParserBase::TEdgeSpheronXYZ> {
public:
	typedef CEdgeSpheronXYZ _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for Spheron solver
 */
template <>
class CSpheronEdgeTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for Spheron solver
 */
template <>
class CSpheronEdgeTraits<CParserBase::TVertexSpheron> {
public:
	typedef CVertexSpheron _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for Spheron solver
 */
template <class CParsedStructure>
class CSpheronVertexTraits {
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
 *	@brief vertex traits for Spheron solver
 */
template <>
class CSpheronVertexTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for Spheron solver
 */
template <>
class CSpheronVertexTraits<CParserBase::TVertexSpheron> {
public:
	typedef CVertexSpheron _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <class CParsedStructure>
class CBAIntrinsicsEdgeTraits {
public:
	typedef CFailOnEdgeType _TyEdge; /**< @brief it should fail on unknown edge types */

	/**
	 *	@brief gets reason for error
	 *	@return Returns const null-terminated string, containing
	 *		description of the error (human readable).
	 */
	static const char *p_s_Reason()
	{
		return "unknown edge type occurred 1";
	}
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAIntrinsicsEdgeTraits<CParserBase::TEdgeP2CI3D> {
public:
	typedef CEdgeP2CI3D _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAIntrinsicsEdgeTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAIntrinsicsEdgeTraits<CParserBase::TVertexCam3D> {
public:
	typedef CVertexCam _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief edge traits for BA solver
 */
template <>
class CBAIntrinsicsEdgeTraits<CParserBase::TVertexIntrinsics> {
public:
	typedef CVertexIntrinsics _TyEdge; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <class CParsedStructure>
class CBAIntrinsicsVertexTraits {
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
 *	@brief vertex traits for BA solver
 */
template <>
class CBAIntrinsicsVertexTraits<CParserBase::TVertexXYZ> {
public:
	typedef CVertexXYZ _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <>
class CBAIntrinsicsVertexTraits<CParserBase::TVertexCam3D> {
public:
	typedef CVertexCam _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/**
 *	@brief vertex traits for BA solver
 */
template <>
class CBAIntrinsicsVertexTraits<CParserBase::TVertexIntrinsics> {
public:
	typedef CVertexIntrinsics _TyVertex; /**< @brief the edge type to construct from the parsed type */
};

/** @} */ // end of group

#endif // __BA_TYPES_INCLUDED
