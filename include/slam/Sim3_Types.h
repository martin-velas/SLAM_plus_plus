/*
								+----------------------------------+
								|                                  |
								|      ***  Sim(3) types  ***      |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|           Sim3_Types.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __SIM3_PRIMITIVE_TYPES_INCLUDED
#define __SIM3_PRIMITIVE_TYPES_INCLUDED

/**
 *	@file include/slam/Sim3_Types.h
 *	@brief Sim(3) primitive types
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include "slam/Sim3SolverBase.h"
//#include "slam/BaseTypes.h" // included from slam/BA_Types.h
#include "slam/BA_Types.h"

/** \addtogroup sim3
 *	@{
 */

/**
 *	@brief namespace with Sim3 implementation details
 */
namespace sim3_detail {

/**
 *	@brief configuration stored as enum
 */
enum {
#ifndef SIM3_USE_ROBUST_EDGES
	n_Sim3_edgeopts = CBaseEdge::Plain // unless stated otherwise, don't use the robust edges
#else // !SIM3_USE_ROBUST_EDGES
	n_Sim3_edgeopts = CBaseEdge::Robust
#endif // !SIM3_USE_ROBUST_EDGES
};

} // ~sim3_detail

/**
 *	@brief inverse depth 3D landmark
 */
class CVertexInvDepth : public CBaseVertexImpl<CVertexInvDepth, 3> {
public:
	typedef CBaseVertexImpl<CVertexInvDepth, 3> _TyBase; /**< @brief base class */

public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexInvDepth()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexInvDepth(const Eigen::Vector3d &r_v_state)
        :_TyBase(r_v_state)
    {}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is parsed vertex state in XYZ format (automatically converted to inverse depth)
	 */
    inline CVertexInvDepth(const CParserBase::TVertexXYZ &r_t_vertex)
		:_TyBase(CSim3Jacobians::v_XYZ_to_InvDepth(r_t_vertex.m_v_position))
    {}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta)
    {
    	CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_v_state, r_v_delta.segment<3>(m_n_order), m_v_state); // delta is in XYZ
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta)
	{
    	CSim3Jacobians::Relative_to_Absolute_InvDepth_Epsilon(m_v_state, -r_v_delta.segment<3>(m_n_order), m_v_state); // delta is in XYZ
	}
};

/**
 *	@brief inverse distance 3D landmark
 */
class CVertexInvDist : public CBaseVertexImpl<CVertexInvDist, 1> {
public:
	typedef CBaseVertexImpl<CVertexInvDist, 1> _TyBase; /**< @brief base class */

protected:
	Eigen::Vector3d m_v_uvw;

public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexInvDist()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex as a <tt>[u, v, w, q]</tt> vector
	 *	@note Only the <tt>q</tt> is a part of the state.
	 */
    inline CVertexInvDist(const Eigen::Vector4d &r_v_state)
        :_TyBase(r_v_state.tail<1>()), m_v_uvw(r_v_state.head<3>())
    {}

	/**
	 *	@brief constructor from parsed type
	 *	@param[in] r_t_vertex is parsed vertex state in XYZ format (automatically converted to inverse distance)
	 */
    inline CVertexInvDist(const CParserBase::TVertexXYZ &r_t_vertex)
		:_TyBase(CSim3Jacobians::v_XYZ_to_InvDist(r_t_vertex.m_v_position).tail<1>()), // q
		m_v_uvw(CSim3Jacobians::v_XYZ_to_InvDist(r_t_vertex.m_v_position).head<3>()) // uvw
    {}

	/**
	 *	@brief constructor from vector type
	 *
	 *	@param[in] r_v_inv_distance is state of the vertex (the inverse distance)
	 *	@param[in] r_v_uvw is normalized direction where the vertex was observed
	 */
	inline CVertexInvDist(const Eigen::Vector1d &r_v_inv_distance, const Eigen::Vector3d &r_v_uvw = Eigen::Vector3d::Zero())
        :_TyBase(r_v_inv_distance), m_v_uvw(r_v_uvw)
    {}

	/**
	 *	@brief get full <tt>[u, v, w, q]</tt> state of the vertex
	 *	@return Returns state of this vertex.
	 */
	inline Eigen::Vector4d v_State4() const
	{
		Eigen::Vector4d v_state;
		v_state.head<3>() = m_v_uvw;
		v_state.tail<1>() = m_v_state;
		return v_state;
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta)
    {
    	m_v_state(0) += r_v_delta(m_n_order); // delta added to q only
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta)
	{
    	m_v_state(0) -= r_v_delta(m_n_order); // delta added to q only
	}
};

/**
 *	@brief bundle adjustment camera pose
 */
class CVertexCamSim3 : public CBaseVertexImpl<CVertexCamSim3, 7> {
public:
	typedef CBaseVertexImpl<CVertexCamSim3, 7> _TyBase; /**< @brief base class */

protected:
	Eigen::Vector5d m_v_intrinsics; /**< @brief intrinsics vector (assumed to be constant, not optimized) */

public:
    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexCamSim3()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex (the first 7 elements are the pose
	 *		in \f$\mathfrak{sim}(3)\f$ the last 5 elements are intrinsics)
	 */
    inline CVertexCamSim3(const Eigen::Matrix<double, 12, 1> &r_v_state)
        :_TyBase(r_v_state.head<7>()), m_v_intrinsics(r_v_state.tail<5>())
    {}

	/**
	 *	@brief constructor from pose and intrinsics vectors
	 *
	 *	@param[in] r_v_pose is pose of the camera (in \f$\mathfrak{sim}(3)\f$)
	 *	@param[in] r_v_intrinsics is intrinsics vector
	 */
    inline CVertexCamSim3(const Eigen::Vector7d &r_v_pose,
		const Eigen::Vector5d &r_v_intrinsics = Eigen::Vector5d::Zero()) // default is required by CInitializeNullVertex (or we could write our own null initializer)
        :_TyBase(r_v_pose), m_v_intrinsics(r_v_intrinsics)
    {}

	/**
	 *	@brief gets intrinsic parameters of the camera
	 *	@return Returns const reference to the parameters of the camera.
	 */
	inline const Eigen::Vector5d &v_Intrinsics() const
	{
		return m_v_intrinsics;
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta)
    {
		CSim3Jacobians::Relative_to_Absolute(m_v_state, r_v_delta.segment<7>(m_n_order), m_v_state); // post-multiplicative update
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta)
	{
		CSim3Jacobians::Relative_to_Absolute(m_v_state, -r_v_delta.segment<7>(m_n_order), m_v_state); // post-multiplicative update
	}
};

/**
 *	@brief inverse depth vertex - Sim(3) camera edge, both are in global coordiantes
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDepth_Sim3_G : public CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_G, MakeTypelist(CVertexInvDepth, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDepth_Sim3_G> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_G, MakeTypelist(CVertexInvDepth, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDepth_Sim3_G()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDepth_Sim3_G(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_camera_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

#ifdef _DEBUG
		Eigen::Vector2d v_proj;
		bool b_is_behind = !CSim3Jacobians::Project_P2C_InvDepth(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
			m_p_vertex1->v_Intrinsics(), v_proj);
		if(b_is_behind)
			fprintf(stderr, "warning: landmark " PRIsize " is behind camera " PRIsize " (on init)\n", n_landmark_id, n_camera_id);
#endif // _DEBUG
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first (inverse depth) vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second (Sim3) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 7> &r_t_jacobian1, Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		bool b_is_behind = !CSim3Jacobians::Project_P2C_InvDepth(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
			m_p_vertex1->v_Intrinsics(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)

		/*if(b_is_behind) {
			double f_weight = std::max(r_t_jacobian0.array().abs().maxCoeff(),
				r_t_jacobian1.array().abs().maxCoeff()) / 1000;
			// let's say that 1000 is the maximum we want to have in the jacobians
			// todo - huber?

			if(f_weight > 1) {
				f_weight = 1 / f_weight;
				r_t_jacobian0 *= f_weight;
				r_t_jacobian1 *= f_weight;
				r_v_error *= f_weight;
			}
			// weight the contributions of the physically impossible edges
		}*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_InvDepth(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_InvDepth(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_InvDepth(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief inverse distance vertex - Sim(3) camera edge, both are in global coordiantes
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDist_Sim3_G : public CBaseEdgeImpl<CEdgeP2C_InvDist_Sim3_G, MakeTypelist(CVertexInvDist, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDist_Sim3_G> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDist_Sim3_G, MakeTypelist(CVertexInvDist, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDist_Sim3_G()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDist_Sim3_G(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_camera_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDist>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 1); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first (inverse distance) vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second (Sim3) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 1> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 7> &r_t_jacobian1, Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_InvDist(m_p_vertex0->v_State4(), m_p_vertex1->r_v_State(),
			m_p_vertex1->v_Intrinsics(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_InvDist(m_p_vertex0->v_State4(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_InvDist(m_p_vertex0->v_State4(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_InvDist(m_p_vertex0->v_State4(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Sim(3) camera edge, both are in global coordiantes
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_XYZ_Sim3_G : public CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_G, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_XYZ_Sim3_G> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_G, MakeTypelist(CVertexXYZ, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_XYZ_Sim3_G()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_XYZ_Sim3_G(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_camera_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first (XYZ) vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second (Sim3) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 2, 7> &r_t_jacobian1, Eigen::Matrix<double, 2, 1> &r_v_expectation,
		Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_XYZ(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
			m_p_vertex1->v_Intrinsics(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
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
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_XYZ(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_XYZ(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_XYZ(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in global coordinates
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_XYZ_Sim3_G : public CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_G, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3, CVertexIntrinsics)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_XYZ_Sim3_G> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_G, MakeTypelist(CVertexXYZ, CVertexCamSim3, CVertexIntrinsics), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_XYZ_Sim3_G()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_XYZ_Sim3_G(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_XYZ(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(),
			r_v_expectation, r_t_jacobian_tuple.Get<0>(),
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
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
		CSim3Jacobians::Project_P2C_XYZ(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			v_error);  // use squared normalization
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_XYZ(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_XYZ(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

typedef CEdgeP2C_InvDepth_Sim3_G CEdgeP2CSim3G; /**< @copydoc CEdgeP2C_InvDepth_Sim3_G */ // todo - remove me

/**
 *	@brief XYZ vertex - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of the same camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_XYZ_Sim3_LS : public CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_LS, MakeTypelist_Safe((CVertexXYZ)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a unary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_XYZ_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_LS, MakeTypelist(CVertexXYZ), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

protected:
	const CVertexCamSim3 *m_p_camera; /**< @brief pointer to the camera vertex @note This is needed for the intrinsics. */
	size_t m_n_camera_id; /**< @brief id of the camera vertex @note This is only needed to be able to write the edge in a graph file. */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_XYZ_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_XYZ_Sim3_LS(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, v_observation, r_t_inv_sigma), m_n_camera_id(n_camera_id)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_p_camera = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

#ifdef THREEDV_DUMP_REPROJECTION_ERROR // defined from the CMake commandline
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		v_error -= m_v_measurement;
		printf("new edge reprojection error: %.2f px, weight: %f\n", v_error.norm(), f_RobustWeight(v_error));
		if(v_error.norm() > 5000)
			m_t_sigma_inv *= 0.0; // "disable" this edge
		// debug - see magnitudes of the errors
#endif // THREEDV_DUMP_REPROJECTION_ERROR
	}

	/**
	 *	@brief gets the observing camera id
	 *	@return Returns zero-based id of the observing camera vertex.
	 */
	inline size_t n_ObservingCamera_Id() const
	{
		return m_n_camera_id;
	}

	/**
	 *	@brief calculates the jacobian, expectation and error
	 *
	 *	@param[out] r_t_jacobian is jacobian, associated with the landmark (XYZ) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(),
			m_p_camera->v_Intrinsics(), r_v_expectation, r_t_jacobian);
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Sim(3) - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_XYZ_Sim3_LO : public CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_LO, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3, CVertexCamSim3)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a ternary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_XYZ_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_LO, MakeTypelist(CVertexXYZ, CVertexCamSim3, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_XYZ_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the third vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_XYZ_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

#ifdef THREEDV_DUMP_REPROJECTION_ERROR // defined from the CMake commandline
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		v_error -= m_v_measurement;
		printf("new edge reprojection error: %.2f px, weight: %f\n", v_error.norm(), f_RobustWeight(v_error));
		if(v_error.norm() > 5000)
			m_t_sigma_inv *= 0.0; // "disable" this edge
		// debug - see magnitudes of the errors
#endif // THREEDV_DUMP_REPROJECTION_ERROR
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalXYZ_Other(
			m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Sim(3) + intrinsics, vertex is in local coordinates of camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_XYZ_Sim3_LS : public CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LS, MakeTypelist_Safe((CVertexXYZ, CVertexIntrinsics)), 2, -1, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a binary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_XYZ_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LS, MakeTypelist(CVertexXYZ, CVertexIntrinsics), 2, -1, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_XYZ_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_intrinsics_id is (zero-based) index of the second vertex (intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_XYZ_Sim3_LS(size_t n_landmark_id, size_t n_intrinsics_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_intrinsics_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 2, 5> &r_t_jacobian1,	Eigen::Matrix<double, 2, 1> &r_v_expectation,
			Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_LocalXYZ_Self(m_p_vertex0->r_v_State(), // landmark
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
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
			v_error); // use squared normalization
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
			v_error); // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalXYZ_Self(
				m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			v_error); // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_XYZ_Sim3_LO : public CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LO, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_XYZ_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LO, MakeTypelist(CVertexXYZ, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_XYZ_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the fourth vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_XYZ_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
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
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), v_error);  // use squared normalization
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), v_error);  // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalXYZ_Other(
			m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), v_error);  // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief invDepth vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in global coordinates
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_InvDepth_Sim3_G : public CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_G, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_InvDepth_Sim3_G> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_G, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_InvDepth_Sim3_G()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_InvDepth_Sim3_G(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_InvDepth(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(),
			r_v_expectation, r_t_jacobian_tuple.Get<0>(),
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
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
		CSim3Jacobians::Project_P2C_XYZ(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			v_error);  // use squared normalization
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_XYZ(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()),
				m_vertex_ptr.Get<1>()->r_v_State(),	m_vertex_ptr.Get<2>()->r_v_State(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_XYZ(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()),
				m_vertex_ptr.Get<1>()->r_v_State(),	m_vertex_ptr.Get<2>()->r_v_State(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief InvDepth vertex - Sim(3) + intrinsics, vertex is in local coordinates of camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_InvDepth_Sim3_LS : public CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LS, MakeTypelist_Safe((CVertexInvDepth, CVertexIntrinsics)), 2, -1, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a binary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_InvDepth_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LS, MakeTypelist(CVertexInvDepth, CVertexIntrinsics), 2, -1, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_InvDepth_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_intrinsics_id is (zero-based) index of the second vertex (intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_InvDepth_Sim3_LS(size_t n_landmark_id, size_t n_intrinsics_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_intrinsics_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 2, 5> &r_t_jacobian1,	Eigen::Matrix<double, 2, 1> &r_v_expectation,
			Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_LocalInvDepth_Self(m_p_vertex0->r_v_State(), // landmark
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
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
			v_error); // use squared normalization
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
			v_error); // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalXYZ_Self(
				CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			v_error); // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_InvDepth_Sim3_LO : public CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LO, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_InvDepth_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LO, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_InvDepth_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark, invDepth)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the fourth vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_InvDepth_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_LocalInvDepth_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
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
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), v_error);  // use squared normalization
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), v_error);  // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalXYZ_Other(
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), v_error);  // use squared normalization
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief inverse depth vertex - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of the same camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDepth_Sim3_LS : public CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_LS, MakeTypelist_Safe((CVertexInvDepth)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a unary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDepth_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_LS, MakeTypelist(CVertexInvDepth), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

protected:
	const CVertexCamSim3 *m_p_camera; /**< @brief pointer to the camera vertex @note This is needed for the intrinsics. */
	size_t m_n_camera_id; /**< @brief id of the camera vertex @note This is only needed to be able to write the edge in a graph file. */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDepth_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDepth_Sim3_LS(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, v_observation, r_t_inv_sigma), m_n_camera_id(n_camera_id)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_p_camera = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief gets the observing camera id
	 *	@return Returns zero-based id of the observing camera vertex.
	 */
	inline size_t n_ObservingCamera_Id() const
	{
		return m_n_camera_id;
	}

	/**
	 *	@brief calculates the jacobian, expectation and error
	 *
	 *	@param[out] r_t_jacobian is jacobian, associated with the landmark (inverse depth) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_LocalInvDepth_Self(m_p_vertex0->r_v_State(),
			m_p_camera->v_Intrinsics(), r_v_expectation, r_t_jacobian);
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_LocalInvDepth_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalInvDepth_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalInvDepth_Self(m_p_vertex0->r_v_State(), m_p_camera->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief inverse depth vertex - Sim(3) - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDepth_Sim3_LO : public CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_LO, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3, CVertexCamSim3)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a ternary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDepth_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_LO, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDepth_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the third vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDepth_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_LocalInvDepth_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_LocalInvDepth_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalInvDepth_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalInvDepth_Other(
			m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief inverse distance vertex - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of the same camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDist_Sim3_LS : public CBaseEdgeImpl<CEdgeP2C_InvDist_Sim3_LS, MakeTypelist_Safe((CVertexInvDist)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a unary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDist_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDist_Sim3_LS, MakeTypelist(CVertexInvDist), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

protected:
	const CVertexCamSim3 *m_p_camera; /**< @brief pointer to the camera vertex @note This is needed for the intrinsics. */
	size_t m_n_camera_id; /**< @brief id of the camera vertex @note This is only needed to be able to write the edge in a graph file. */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDist_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDist_Sim3_LS(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, v_observation, r_t_inv_sigma), m_n_camera_id(n_camera_id)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDist>(n_landmark_id, CInitializeNullVertex<>());
		m_p_camera = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 1); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief gets the observing camera id
	 *	@return Returns zero-based id of the observing camera vertex.
	 */
	inline size_t n_ObservingCamera_Id() const
	{
		return m_n_camera_id;
	}

	/**
	 *	@brief calculates the jacobian, expectation and error
	 *
	 *	@param[out] r_t_jacobian is jacobian, associated with the landmark (inverse distance) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 2, 1> &r_t_jacobian,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_LocalInvDist_Self(m_p_vertex0->v_State4(),
			m_p_camera->v_Intrinsics(), r_v_expectation, r_t_jacobian);
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_LocalInvDist_Self(m_p_vertex0->v_State4(), m_p_camera->v_Intrinsics(), v_error);
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalInvDist_Self(m_p_vertex0->v_State4(), m_p_camera->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalInvDist_Self(m_p_vertex0->v_State4(), m_p_camera->v_Intrinsics(), v_error);
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief inverse distance vertex - Sim(3) - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDist_Sim3_LO : public CBaseEdgeImpl<CEdgeP2C_InvDist_Sim3_LO, MakeTypelist_Safe((CVertexInvDist, CVertexCamSim3, CVertexCamSim3)), 2, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a ternary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDist_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDist_Sim3_LO, MakeTypelist(CVertexInvDist, CVertexCamSim3, CVertexCamSim3), 2, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDist_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the third vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDist_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDist>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 1); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_LocalInvDist_Other(m_vertex_ptr.Get<0>()->v_State4(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

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
		CSim3Jacobians::Project_P2C_LocalInvDist_Other(m_vertex_ptr.Get<0>()->v_State4(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalInvDist_Other(m_vertex_ptr.Get<0>()->v_State4(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector2d v_error;
		r_b_vertex_in_front_of_camera = CSim3Jacobians::Project_P2C_LocalInvDist_Other(
			m_vertex_ptr.Get<0>()->v_State4(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		return (v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

#if 1

/**
 *	@brief Sim(3) pose
 */
class CVertexSim3 : public CBaseVertexImpl<CVertexSim3, 7> {
public:
	typedef CBaseVertexImpl<CVertexSim3, 7> _TyBase; /**< @brief base class */

    __GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
    inline CVertexSim3()
    {}

	/**
	 *	@brief constructor from vector type
	 *	@param[in] r_v_state is state of the vertex
	 */
    inline CVertexSim3(const Eigen::Vector7d &r_v_state)
        :_TyBase(r_v_state)
    {}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta)
    {
		CSim3Jacobians::Relative_to_Absolute(m_v_state, r_v_delta.segment<7>(m_n_order), m_v_state); // post-multiplicative update
    }

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Minus()
	 */
    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta)
	{
		CSim3Jacobians::Relative_to_Absolute(m_v_state, -r_v_delta.segment<7>(m_n_order), m_v_state); // post-multiplicative update
	}
};

/**
 *	@brief Sim(3) camera-camera edge
 */
class CEdgeSim3 : public CBaseEdgeImpl<CEdgeSim3, MakeTypelist(CVertexSim3, CVertexSim3), 7> {
public:
	typedef CBaseEdgeImpl<CEdgeSim3, MakeTypelist(CVertexSim3, CVertexSim3), 7> _TyBase; /**< @brief base class */

	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and an XYT edge.
	 */
	class CRelative_to_Absolute_Initializer {
	protected:
		const CVertexSim3 &m_r_v_vertex1; /**< @brief the first vertex */
		const Eigen::Matrix<double, 7, 1> &m_r_v_delta; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_delta is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_Initializer(const CVertexSim3 &r_v_vertex1,
			const Eigen::Matrix<double, 7, 1> &r_v_delta)
			:m_r_v_vertex1(r_v_vertex1), m_r_v_delta(r_v_delta)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexSim3() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
		{
			Eigen::Matrix<double, 7, 1> v_pose2;
			CSim3Jacobians::Relative_to_Absolute(m_r_v_vertex1.r_v_State(), m_r_v_delta, v_pose2); // implement your own equation here
			return CVertexSim3(v_pose2);
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeSim3()
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
	CEdgeSim3(size_t n_vertex0, size_t n_vertex1, const Eigen::Vector7d &r_v_delta,
		const Eigen::Matrix7d &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, n_vertex1, r_v_delta, r_v_sigma, null_initialize_vertices,
		r_system, 0x02) // don't null-initialize vertex 1
	{
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexSim3>(n_vertex1,
			CRelative_to_Absolute_Initializer(*m_p_vertex0, r_v_delta));
		// initialize vertex 1 manually
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Vector7d &r_v_delta, const Eigen::Matrix7d &r_t_inv_sigma)
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
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix7d &r_t_jacobian0,
		Eigen::Matrix7d &r_t_jacobian1, Eigen::Vector7d &r_v_expectation,
		Eigen::Vector7d &r_v_error) const
	{
		CSim3Jacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		CSim3Jacobians::TSim3 t_expn(r_v_expectation, CSim3Jacobians::TSim3::from_sim3_vector);
		CSim3Jacobians::TSim3 t_meas(m_v_measurement, CSim3Jacobians::TSim3::from_sim3_vector);
		t_meas.Invert();
		t_expn *= t_meas;
		r_v_error = t_expn.v_Log();
		//std::cout << "0> " << r_v_expectation.transpose() << std::endl;
		//std::cout << "1> " << m_v_measurement.transpose() << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector7d v_expectation, v_error;
		CSim3Jacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), v_expectation);
		CSim3Jacobians::TSim3 t_expn(v_expectation, CSim3Jacobians::TSim3::from_sim3_vector);
		CSim3Jacobians::TSim3 t_meas(m_v_measurement, CSim3Jacobians::TSim3::from_sim3_vector);
		t_meas.Invert();
		t_expn *= t_meas;
		v_error = t_expn.v_Log();
		// calculates the expectation and error

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief Sim(3) pose roll edge
 */
class CEdgeSim3Roll : public CBaseEdgeImpl<CEdgeSim3Roll, MakeTypelist(CVertexSim3), 1> {
public:
	typedef CBaseEdgeImpl<CEdgeSim3Roll, MakeTypelist(CVertexSim3), 1> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeSim3Roll()
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
	CEdgeSim3Roll(size_t n_vertex0, const Eigen::Vector1d &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSim3>(n_vertex0, CInitializeNullVertex<>());
		// initialize vertex 1 manually
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Vector1d &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma)
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
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 7> &r_t_jacobian0,
			Eigen::Vector1d &r_v_expectation, Eigen::Vector1d &r_v_error) const
	{
		CSim3Jacobians::RollJacobians(m_p_vertex0->r_v_State(), r_v_expectation, r_t_jacobian0);
		// calculates the expectation and the jacobians

		r_v_error = m_v_measurement - r_v_expectation;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector1d v_expectation, v_error;

		CSim3Jacobians::TSim3 vert1;
		vert1.Exp(m_p_vertex0->r_v_State());
		// exp map

		Eigen::Vector7d pose = vert1.v_tRs();
		Eigen::Matrix3d rot = C3DJacobians::Operator_rot(pose.tail(4).head(3));
		Eigen::Vector3d up(0, 1, 0);
		Eigen::Vector3d up2 = rot/*.inverse()*/ * up;

		Eigen::Vector3d ea = rot.eulerAngles(1, 0, 2);
		v_expectation(0) = ea(2);

		if(ea(2) > M_PI / 2)
			v_expectation(0) = - M_PI + ea(2);
		if(ea(2) < -M_PI / 2)
			v_expectation(0) = M_PI + ea(2);

		//v_expectation(0) = 1-up2(1);//acos(up2(1));
		v_expectation(0) = /*(up2 - up).norm();*//*0.5 - 0.5 * cos(v_expectation(0));*/v_expectation(0) * v_expectation(0);
		//square to go to positive nums and stay differentiables

		v_error = m_v_measurement - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief Sim(3) pose pitch edge
 */
class CEdgeSim3Pitch : public CBaseEdgeImpl<CEdgeSim3Pitch, MakeTypelist(CVertexSim3), 1> {
public:
	typedef CBaseEdgeImpl<CEdgeSim3Pitch, MakeTypelist(CVertexSim3), 1> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeSim3Pitch()
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
	CEdgeSim3Pitch(size_t n_vertex0, const Eigen::Vector1d &r_v_delta,
		const Eigen::Matrix<double, 1, 1> &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, r_v_delta, r_v_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexSim3>(n_vertex0, CInitializeNullVertex<>());
		// initialize vertex 1 manually
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Vector1d &r_v_delta, const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma)
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
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 7> &r_t_jacobian0,
			Eigen::Vector1d &r_v_expectation, Eigen::Vector1d &r_v_error) const
	{
		CSim3Jacobians::PitchJacobians(m_p_vertex0->r_v_State(), r_v_expectation, r_t_jacobian0);
		// calculates the expectation and the jacobians

		r_v_error = m_v_measurement - r_v_expectation;
		//std::cout << r_v_expectation << " <> " << m_v_measurement << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector1d v_expectation, v_error;

		CSim3Jacobians::TSim3 vert1;
		vert1.Exp(m_p_vertex0->r_v_State());
		// exp map

		Eigen::Vector7d pose = vert1.v_tRs();
		Eigen::Matrix3d rot = C3DJacobians::Operator_rot(pose.tail(4).head(3));

		Eigen::Vector3d ea = rot.eulerAngles(1, 0, 2);
		v_expectation(0) = ea(1);

		if(ea(1) > M_PI / 2)
			v_expectation(0) = - M_PI + ea(1);
		if(ea(1) < -M_PI / 2)
			v_expectation(0) = M_PI + ea(1);

		v_expectation(0) = v_expectation(0) * v_expectation(0);//0.5 - 0.5 * cos(v_expectation(0));

		v_error = m_v_measurement - v_expectation;

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

#endif // 1

/**
 *	@brief XYZ vertex (Landmark) edge, vertex is in local coordinates of camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeLandmark_XYZ_Sim3_LS : public CBaseEdgeImpl<CEdgeLandmark_XYZ_Sim3_LS, MakeTypelist_Safe((CVertexXYZ)), 3, -1, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a binary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeLandmark_XYZ_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeLandmark_XYZ_Sim3_LS, MakeTypelist(CVertexXYZ), 3, -1, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeLandmark_XYZ_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] v_observation is 3D observation of the landmark by the camera (3d sensor)
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeLandmark_XYZ_Sim3_LS(size_t n_landmark_id, const Eigen::Vector3d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 3, 1> &r_v_expectation,
			Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_Landmark_LocalXYZ_Self(m_p_vertex0->r_v_State(),
				r_v_expectation,
				r_t_jacobian0);
		/*CSim3Jacobians::Project_P2CI_LocalXYZ_Self(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			r_v_expectation, r_t_jacobian0,	r_t_jacobian1);*/
		// calculates the expectation and the jacobians
		//std::cout << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << std::endl;

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error = m_p_vertex0->r_v_State();

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error = m_p_vertex0->r_v_State();
		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		r_b_vertex_in_front_of_camera = true;
		Eigen::Vector3d v_error = m_p_vertex0->r_v_State();
		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex (Landmark) edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeLandmark_XYZ_Sim3_LO : public CBaseEdgeImpl<CEdgeLandmark_XYZ_Sim3_LO, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3, CVertexCamSim3)), 3, -1, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a ternary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeLandmark_XYZ_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeLandmark_XYZ_Sim3_LO, MakeTypelist(CVertexXYZ, CVertexCamSim3, CVertexCamSim3), 3, -1, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeLandmark_XYZ_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the third vertex (landmark owner camera)
	 *	@param[in] v_observation is 3D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeLandmark_XYZ_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_owner_camera_id, const Eigen::Vector3d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

#ifdef THREEDV_DUMP_REPROJECTION_ERROR // defined from the CMake commandline
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		v_error -= m_v_measurement;
		printf("new edge reprojection error: %.2f px, weight: %f\n", v_error.norm(), f_RobustWeight(v_error));
		if(v_error.norm() > 5000)
			m_t_sigma_inv *= 0.0; // "disable" this edge
		// debug - see magnitudes of the errors
#endif // THREEDV_DUMP_REPROJECTION_ERROR
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
		CSim3Jacobians::Project_Landmark_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(),
				m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(),
				r_v_expectation,
				r_t_jacobian_tuple.Get<0>(),
				r_t_jacobian_tuple.Get<1>(),
				r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		//std::cout << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << std::endl;
		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;

		CSim3Jacobians::Project_Landmark_Local_to_Other(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;

		CSim3Jacobians::Project_Landmark_Local_to_Other(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
						m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		r_b_vertex_in_front_of_camera = true;
		Eigen::Vector3d v_error;

		CSim3Jacobians::Project_Landmark_Local_to_Other(m_vertex_ptr.Get<0>()->r_v_State(), m_vertex_ptr.Get<1>()->r_v_State(),
						m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex (Landmark) edge, vertex is in local coordinates of camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeLandmark_InvDepth_Sim3_LS : public CBaseEdgeImpl<CEdgeLandmark_InvDepth_Sim3_LS, MakeTypelist_Safe((CVertexInvDepth)), 3, -1, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a binary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeLandmark_InvDepth_Sim3_LS> {
public:
	typedef CBaseEdgeImpl<CEdgeLandmark_InvDepth_Sim3_LS, MakeTypelist(CVertexInvDepth), 3, -1, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeLandmark_InvDepth_Sim3_LS()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] v_observation is 3D observation of the landmark by the camera (3d sensor)
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeLandmark_InvDepth_Sim3_LS(size_t n_landmark_id, const Eigen::Vector3d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 3, 1> &r_v_expectation,
			Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_Landmark_LocalInvDepth_Self(m_p_vertex0->r_v_State(),
				r_v_expectation,
				r_t_jacobian0);
		/*CSim3Jacobians::Project_P2CI_LocalXYZ_Self(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			r_v_expectation, r_t_jacobian0,	r_t_jacobian1);*/
		// calculates the expectation and the jacobians
		//std::cout << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << std::endl;

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error = CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State());

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error = CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State());
		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		r_b_vertex_in_front_of_camera = true;
		Eigen::Vector3d v_error = CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State());
		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex (Landmark) edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeLandmark_InvDepth_Sim3_LO : public CBaseEdgeImpl<CEdgeLandmark_InvDepth_Sim3_LO, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3, CVertexCamSim3)), 3, -1, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a ternary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeLandmark_InvDepth_Sim3_LO> {
public:
	typedef CBaseEdgeImpl<CEdgeLandmark_InvDepth_Sim3_LO, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexCamSim3), 3, -1, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeLandmark_InvDepth_Sim3_LO()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the third vertex (landmark owner camera)
	 *	@param[in] v_observation is 3D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeLandmark_InvDepth_Sim3_LO(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_owner_camera_id, const Eigen::Vector3d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.

#ifdef THREEDV_DUMP_REPROJECTION_ERROR // defined from the CMake commandline
		Eigen::Vector2d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<1>()->v_Intrinsics(), // observing camera and its intrinsics
			m_vertex_ptr.Get<2>()->r_v_State(), v_error); // owner camera
		v_error -= m_v_measurement;
		printf("new edge reprojection error: %.2f px, weight: %f\n", v_error.norm(), f_RobustWeight(v_error));
		if(v_error.norm() > 5000)
			m_t_sigma_inv *= 0.0; // "disable" this edge
		// debug - see magnitudes of the errors
#endif // THREEDV_DUMP_REPROJECTION_ERROR
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
		CSim3Jacobians::Project_Landmark_LocalInvDepth_Other(m_vertex_ptr.Get<0>()->r_v_State(),
				m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(),
				r_v_expectation,
				r_t_jacobian_tuple.Get<0>(),
				r_t_jacobian_tuple.Get<1>(),
				r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		//std::cout << r_v_expectation.transpose() << " <> " << m_v_measurement.transpose() << std::endl;
		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;

		CSim3Jacobians::Project_Landmark_Local_to_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()),
				m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;

		CSim3Jacobians::Project_Landmark_Local_to_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()),
				m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		r_b_vertex_in_front_of_camera = true;
		Eigen::Vector3d v_error;

		CSim3Jacobians::Project_Landmark_Local_to_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()),
				m_vertex_ptr.Get<1>()->r_v_State(),
				m_vertex_ptr.Get<2>()->r_v_State(), v_error);

		return 0;//(v_error - m_v_measurement).norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Sim(3) + intrinsics, error is represented as angle between expectation and observation, vertex is in local coordinates of camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_XYZ_Sim3_LS_AngleErr : public CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LS_AngleErr, MakeTypelist_Safe((CVertexXYZ, CVertexIntrinsics)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a binary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_XYZ_Sim3_LS_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LS_AngleErr, MakeTypelist(CVertexXYZ, CVertexIntrinsics), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_XYZ_Sim3_LS_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_intrinsics_id is (zero-based) index of the second vertex (intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_XYZ_Sim3_LS_AngleErr(size_t n_landmark_id, size_t n_intrinsics_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_intrinsics_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 3, 5> &r_t_jacobian1,	Eigen::Matrix<double, 3, 1> &r_v_expectation,
			Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_LocalXYZ_Self_AngleErr(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
				m_v_measurement, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians
		r_v_error = r_v_expectation;

		//std::cout << "S:" << r_v_error.transpose() << std::endl;
		// error is already stored
		//std::cout << r_t_jacobian0 << std::endl;
		//std::cout << r_t_jacobian1 << std::endl << std::endl;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self_AngleErr(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
				m_v_measurement, v_error); // use squared normalization

#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self_AngleErr(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
				m_v_measurement, v_error); // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self_AngleErr(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
				m_v_measurement, v_error); // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_XYZ_Sim3_LO_AngleErr : public CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LO_AngleErr, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_XYZ_Sim3_LO_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_LO_AngleErr, MakeTypelist(CVertexXYZ, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_XYZ_Sim3_LO_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the fourth vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_XYZ_Sim3_LO_AngleErr(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
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
		CSim3Jacobians::Project_P2CI_LocalXYZ_Other_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " " << r_v_expectation.transpose() << std::endl;
		r_v_error = r_v_expectation;

		//std::cout << "O:" << r_v_error.transpose() << std::endl;
		// calculates error (possibly re-calculates, if running A-SLAM)
		/*std::cout << r_t_jacobian_tuple.Get<0>() << std::endl;
		std::cout << r_t_jacobian_tuple.Get<1>() << std::endl;
		std::cout << r_t_jacobian_tuple.Get<2>() << std::endl;
		std::cout << r_t_jacobian_tuple.Get<3>() << std::endl << std::endl;*/
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, v_error);  // use squared normalization
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other_AngleErr(
			m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Sim(3) + intrinsics, error is represented as angle between expectation and observation, vertex is in local coordinates of camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_InvDepth_Sim3_LS_AngleErr : public CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LS_AngleErr, MakeTypelist_Safe((CVertexInvDepth, CVertexIntrinsics)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a binary edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_InvDepth_Sim3_LS_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LS_AngleErr, MakeTypelist(CVertexInvDepth, CVertexIntrinsics), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_InvDepth_Sim3_LS_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_intrinsics_id is (zero-based) index of the second vertex (intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_InvDepth_Sim3_LS_AngleErr(size_t n_landmark_id, size_t n_intrinsics_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_intrinsics_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple is tuple of jacobians, associated with the corresponding vertices
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
			Eigen::Matrix<double, 3, 5> &r_t_jacobian1,	Eigen::Matrix<double, 3, 1> &r_v_expectation,
			Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2CI_LocalInvDepth_Self_AngleErr(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
				m_v_measurement, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians
		r_v_error = r_v_expectation;

		// error is already stored
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
				m_v_measurement, v_error); // use squared normalization

#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
				m_v_measurement, v_error); // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_LocalXYZ_Self_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), // landmark
				m_p_vertex1->r_v_State(), //intrinsics
				m_v_measurement, v_error); // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_InvDepth_Sim3_LO_AngleErr : public CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LO_AngleErr, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_InvDepth_Sim3_LO_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_LO_AngleErr, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics, CVertexCamSim3), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_InvDepth_Sim3_LO_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] n_owner_camera_id is (zero-based) index of the fourth vertex (landmark owner camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_InvDepth_Sim3_LO_AngleErr(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id, n_owner_camera_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_owner_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_owner_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
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
		CSim3Jacobians::Project_P2CI_LocalInvDepth_Other_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, r_v_expectation, r_t_jacobian_tuple.Get<0>(), // owner camera
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " " << r_v_expectation.transpose() << std::endl;
		r_v_error = r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, v_error);  // use squared normalization
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_LocalXYZ_Other_AngleErr(
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_vertex_ptr.Get<3>()->r_v_State(), m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Sim(3) camera edge, both are in global coordiantes
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_XYZ_Sim3_AngleErr : public CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_AngleErr, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_XYZ_Sim3_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_XYZ_Sim3_AngleErr, MakeTypelist(CVertexXYZ, CVertexCamSim3), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_XYZ_Sim3_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_XYZ_Sim3_AngleErr(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_camera_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first (XYZ) vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second (Sim3) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 7> &r_t_jacobian1, Eigen::Matrix<double, 3, 1> &r_v_expectation,
		Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
			m_p_vertex1->v_Intrinsics(), m_v_measurement, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() << std::endl;
		r_v_error = r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(),
				m_v_measurement, v_error);
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(),
						m_v_measurement, v_error);
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(),
						m_v_measurement, v_error);
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief XYZ vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_XYZ_Sim3_AngleErr : public CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_AngleErr, MakeTypelist_Safe((CVertexXYZ, CVertexCamSim3, CVertexIntrinsics)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_XYZ_Sim3_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_XYZ_Sim3_AngleErr, MakeTypelist(CVertexXYZ, CVertexCamSim3, CVertexIntrinsics), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_XYZ_Sim3_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_XYZ_Sim3_AngleErr(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexXYZ>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
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
		CSim3Jacobians::Project_P2CI_XYZ_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(),
			m_v_measurement, r_v_expectation, r_t_jacobian_tuple.Get<0>(),
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " " << r_v_expectation.transpose() << std::endl;
		r_v_error = r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_v_measurement, v_error);  // use squared normalization
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(
			m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief InvDepth vertex - Sim(3) camera edge, both are in global coordiantes
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2C_InvDepth_Sim3_AngleErr : public CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_AngleErr, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2C_InvDepth_Sim3_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2C_InvDepth_Sim3_AngleErr, MakeTypelist(CVertexInvDepth, CVertexCamSim3), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2C_InvDepth_Sim3_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_camera_id is (zero-based) index of the second vertex (camera)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2C_InvDepth_Sim3_AngleErr(size_t n_landmark_id, size_t n_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_landmark_id, n_camera_id, v_observation, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_camera_id].n_Dimension() == 7);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first (XYZ) vertex
	 *	@param[out] r_t_jacobian1 is jacobian, associated with the second (Sim3) vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix<double, 3, 3> &r_t_jacobian0,
		Eigen::Matrix<double, 3, 7> &r_t_jacobian1, Eigen::Matrix<double, 3, 1> &r_v_expectation,
		Eigen::Matrix<double, 3, 1> &r_v_error) const
	{
		CSim3Jacobians::Project_P2C_InvDepth_AngleErr(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
			m_p_vertex1->v_Intrinsics(), m_v_measurement, r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " <> " << r_v_expectation.transpose() << std::endl;
		r_v_error = r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()),
			m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), m_v_measurement, v_error);
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()),
			m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), m_v_measurement, v_error);
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()),
			m_p_vertex1->r_v_State(), m_p_vertex1->v_Intrinsics(), m_v_measurement, v_error);
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief InvDepth vertex - Intrinsics - Sim(3) camera edge, camera is in global coordinates and the vertex is in local coordinates of (other) owner camera
 *
 *	@note There are a few differences between this and the SE(3) BA edges. This has the landmark vertex as the
 *		first vertex and the camera as the second, same as in the graph file, and hence no argument order
 *		swapping takes place anywhere.
 */
class CEdgeP2CI_InvDepth_Sim3_AngleErr : public CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_AngleErr, MakeTypelist_Safe((CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics)), 3, 2, sim3_detail::n_Sim3_edgeopts>,
	public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> { // note that this is a 4ary (el quatronary?) edge
	//public CRobustify_Chi2_Default<CCTFraction<593, 25>, CHuberLossd, CEdgeP2CI_InvDepth_Sim3_AngleErr> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI_InvDepth_Sim3_AngleErr, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexIntrinsics), 3, 2, sim3_detail::n_Sim3_edgeopts> _TyBase; /**< @brief base class */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI_InvDepth_Sim3_AngleErr()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_landmark_id is (zero-based) index of the first vertex (landmark)
	 *	@param[in] n_observing_camera_id is (zero-based) index of the second vertex (observing camera)
	 *	@param[in] n_observing_camera_intrinsics_id is (zero-based) index of the third vertex (observing intrinsics)
	 *	@param[in] v_observation is 2D observation of the landmark by the camera
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (needed to query the vertices)
	 */
	template <class CSystem>
	CEdgeP2CI_InvDepth_Sim3_AngleErr(size_t n_landmark_id, size_t n_observing_camera_id, size_t n_observing_camera_intrinsics_id,
		size_t n_owner_camera_id, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix3d &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_landmark_id, n_observing_camera_id, n_observing_camera_intrinsics_id), v_observation, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_landmark_id, CInitializeNullVertex<>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_observing_camera_id, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsics>(n_observing_camera_intrinsics_id, CInitializeNullVertex<CVertexIntrinsics>());
		// get vertices (initialize if required)

		//_ASSERTE(r_system.r_Vertex_Pool()[n_landmark_id].n_Dimension() == 3); // get the vertices from the vertex pool to ensure a correct type is used, do not use m_p_vertex0 / m_p_vertex1 for this
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_id].n_Dimension() == 7);
		//_ASSERTE(r_system.r_Vertex_Pool()[n_observing_camera_intrinsics_id].n_Dimension() == 5);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
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
		CSim3Jacobians::Project_P2CI_InvDepth_AngleErr(m_vertex_ptr.Get<0>()->r_v_State(), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(),
			m_v_measurement, r_v_expectation, r_t_jacobian_tuple.Get<0>(),
			r_t_jacobian_tuple.Get<1>(), r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		//std::cout << m_v_measurement.transpose() << " " << r_v_expectation.transpose() << std::endl;
		r_v_error = r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_v_measurement, v_error);  // use squared normalization
#ifdef SIM3_USE_ROBUST_EDGES
		double w = f_RobustWeight(v_error);
#else // SIM3_USE_ROBUST_EDGES
		double w = 1;
#endif // SIM3_USE_ROBUST_EDGES
		return (v_error.transpose() * w * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector3d v_error;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}

	/**
	 *	@brief calculates reprojection error
	 *	@param[out] r_b_vertex_in_front_of_camera is vertex visibility flag
	 *	@return Returns reprojection error for this edge.
	 */
	inline double f_Reprojection_Error(bool &r_b_vertex_in_front_of_camera) const
	{
		Eigen::Vector3d v_error;
		r_b_vertex_in_front_of_camera = true;
		CSim3Jacobians::Project_P2C_XYZ_AngleErr(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->r_v_State()), // landmark
			m_vertex_ptr.Get<1>()->r_v_State(), m_vertex_ptr.Get<2>()->r_v_State(), // observing camera and its intrinsics
			m_v_measurement, v_error);  // use squared normalization
		return v_error.norm(); // norm of reprojection error (in pixels)
		// this is actually negative error, but it is squared below so the chi2 is the same
	}
};

/**
 *	@brief Sim(3) camera-camera edge
 */
class CEdgePoseCamSim3 : public CBaseEdgeImpl<CEdgePoseCamSim3, MakeTypelist(CVertexCamSim3, CVertexCamSim3), 7> {
public:
	typedef CBaseEdgeImpl<CEdgePoseCamSim3, MakeTypelist(CVertexCamSim3, CVertexCamSim3), 7> _TyBase; /**< @brief base class */

	/**
	 *	@brief vertex initialization functor
	 *	Calculates vertex position from the first vertex and an XYT edge.
	 */
	class CRelative_to_Absolute_Initializer {
	protected:
		const CVertexCamSim3 &m_r_v_vertex1; /**< @brief the first vertex */
		const Eigen::Matrix<double, 7, 1> &m_r_v_delta; /**< @brief the edge, shared by r_v_vertex1 and the vertex being initialized */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_v_vertex1 is the first vertex
		 *	@param[in] r_v_delta is the edge, shared by r_v_vertex1 and the vertex being initialized
		 */
		inline CRelative_to_Absolute_Initializer(const CVertexCamSim3 &r_v_vertex1,
			const Eigen::Matrix<double, 7, 1> &r_v_delta)
			:m_r_v_vertex1(r_v_vertex1), m_r_v_delta(r_v_delta)
		{}

		/**
		 *	@brief function operator
		 *	@return Returns the value of the vertex being initialized.
		 */
		inline operator CVertexCamSim3() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
		{
			Eigen::Matrix<double, 7, 1> v_pose2;
			CSim3Jacobians::Relative_to_Absolute(m_r_v_vertex1.r_v_State(), m_r_v_delta, v_pose2); // implement your own equation here
			return CVertexCamSim3(v_pose2, m_r_v_vertex1.v_Intrinsics());
		}
	};

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgePoseCamSim3()
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
	CEdgePoseCamSim3(size_t n_vertex0, size_t n_vertex1, const Eigen::Vector7d &r_v_delta,
		const Eigen::Matrix7d &r_v_sigma, CSystem &r_system)
		:_TyBase(n_vertex0, n_vertex1, r_v_delta, r_v_sigma, null_initialize_vertices,
		r_system, 0x02) // don't null-initialize vertex 1
	{
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_vertex1,
			CRelative_to_Absolute_Initializer(*m_p_vertex0, r_v_delta));
		// initialize vertex 1 manually
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_v_delta is new measurement vector
	 *	@param[in] r_t_inv_sigma is new information matrix
	 */
	inline void Update(const Eigen::Vector7d &r_v_delta, const Eigen::Matrix7d &r_t_inv_sigma)
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
	inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix7d &r_t_jacobian0,
		Eigen::Matrix7d &r_t_jacobian1, Eigen::Vector7d &r_v_expectation,
		Eigen::Vector7d &r_v_error) const
	{
		CSim3Jacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), r_v_expectation, r_t_jacobian0, r_t_jacobian1);
		// calculates the expectation and the jacobians

		CSim3Jacobians::TSim3 t_expn(r_v_expectation, CSim3Jacobians::TSim3::from_sim3_vector);
		CSim3Jacobians::TSim3 t_meas(m_v_measurement, CSim3Jacobians::TSim3::from_sim3_vector);
		t_meas.Invert();
		t_expn *= t_meas;
		r_v_error = t_expn.v_Log();
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector7d v_expectation, v_error;
		CSim3Jacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
			m_p_vertex1->r_v_State(), v_expectation);
		CSim3Jacobians::TSim3 t_expn(v_expectation, CSim3Jacobians::TSim3::from_sim3_vector);
		CSim3Jacobians::TSim3 t_meas(m_v_measurement, CSim3Jacobians::TSim3::from_sim3_vector);
		t_meas.Invert();
		t_expn *= t_meas;
		v_error = t_expn.v_Log();
		// calculates the expectation and error

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI3D_InvDepth : public CBaseEdgeImpl<CEdgeP2CI3D_InvDepth, MakeTypelist3(CVertexCam, CVertexInvDepth, CVertexIntrinsics), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI3D_InvDepth, MakeTypelist3(CVertexCam, CVertexInvDepth, CVertexIntrinsics), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI3D_InvDepth()
	{}

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
	CEdgeP2CI3D_InvDepth(size_t n_node_0, size_t n_node_1, size_t n_node_2, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node_1,
		n_node_0, n_node_2), v_delta, r_t_inv_sigma,
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
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple structure holding all jacobians
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2CI_InvDepth(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<2>()->v_State(),
			m_vertex_ptr.Get<1>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		/*std::cout << m_vertex_ptr.Get<0>()->v_State().transpose() << " | " <<
				CBAJacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()).transpose() <<
				" | " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;*/

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector2d v_error, e2;
		CBAJacobians::Project_P2C(m_vertex_ptr.Get<0>()->v_State() /* Cam */, m_vertex_ptr.Get<2>()->v_State(),
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()) /* XYZ */, v_error);

		//e2 = v_error;
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)
		/*std::cout << "chi: " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << " " << m_v_measurement.transpose() <<
				" " << e2.transpose() << std::endl;*/

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
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()) /* XYZ */, v_expectation);

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
		CBAJacobians::Project_P2C(m_vertex_ptr.Get<0>()->v_State() /* Cam */, m_vertex_ptr.Get<2>()->v_State(),
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()) /* XYZ */, v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI3D_InvDepth_LS : public CBaseEdgeImpl<CEdgeP2CI3D_InvDepth_LS, MakeTypelist(CVertexInvDepth, CVertexIntrinsics), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd>{
public:
	typedef CBaseEdgeImpl<CEdgeP2CI3D_InvDepth_LS, MakeTypelist(CVertexInvDepth, CVertexIntrinsics), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI3D_InvDepth_LS()
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
	CEdgeP2CI3D_InvDepth_LS(size_t n_node_0, size_t n_node_1, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(n_node_0, n_node_1, v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_node_0, CInitializeNullVertex<CVertexInvDepth>());
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
		CBAJacobians::Project_P2CI_Self_InvDepth(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			r_v_expectation, r_t_jacobian0,	r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << "LS " << m_v_measurement.transpose() << " " << r_v_expectation.transpose() << " | " <<
		//		m_p_vertex1->r_v_State().transpose() << std::endl;
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
		CBAJacobians::Project_P2C(cam_zero, m_p_vertex1->r_v_State(), CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), v_error);

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
		CBAJacobians::Project_P2C(cam_zero, m_p_vertex1->r_v_State(), CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), v_expectation);

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
		CBAJacobians::Project_P2C(cam_zero, m_p_vertex1->r_v_State(), CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI3D_InvDepth_LO : public CBaseEdgeImpl<CEdgeP2CI3D_InvDepth_LO, MakeTypelist(CVertexInvDepth, CVertexCam, CVertexIntrinsics, CVertexCam), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd>{
public:
	typedef CBaseEdgeImpl<CEdgeP2CI3D_InvDepth_LO, MakeTypelist(CVertexInvDepth, CVertexCam, CVertexIntrinsics, CVertexCam), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI3D_InvDepth_LO()
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
	CEdgeP2CI3D_InvDepth_LO(size_t n_node_0, size_t n_node_1, size_t n_node_2, size_t n_node_3, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node_0,
		n_node_1, n_node_2, n_node_3), v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_node_0, CInitializeNullVertex<CVertexInvDepth>());
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
		CBAJacobians::Project_P2CI_Other_InvDepth(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
			m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
		// calculates the expectation and the jacobians

		//std::cout << "LO: " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;

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
		CBAJacobians::Project_P2C_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
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
		CBAJacobians::Project_P2C_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
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
		CBAJacobians::Project_P2C_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/*
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI2_InvDepth : public CBaseEdgeImpl<CEdgeP2CI2_InvDepth, MakeTypelist3(CVertexCam, CVertexInvDepth, CVertexIntrinsicsReduced), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd> {
public:
	typedef CBaseEdgeImpl<CEdgeP2CI2_InvDepth, MakeTypelist3(CVertexCam, CVertexInvDepth, CVertexIntrinsicsReduced), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI2_InvDepth()
	{}

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
	CEdgeP2CI2_InvDepth(size_t n_node_0, size_t n_node_1, size_t n_node_2, const Eigen::Vector2d &v_delta, // won't align non-reference arg
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
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian_tuple structure holding all jacobians
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobians_Expectation_Error(_TyBase::_TyJacobianTuple &r_t_jacobian_tuple,
		Eigen::Matrix<double, 2, 1> &r_v_expectation, Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2CI2_InvDepth(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
			m_vertex_ptr.Get<2>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>());
		// calculates the expectation and the jacobians

		/*std::cout << m_vertex_ptr.Get<0>()->v_State().transpose() << " | " <<
				CBAJacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()).transpose() <<
				" | " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;*/

		r_v_error = m_v_measurement - r_v_expectation;
		// calculates error (possibly re-calculates, if running A-SLAM)
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Vector2d v_error, e2;
		bool tmp = true;
		CBAJacobians::Project_P2C2(m_vertex_ptr.Get<0>()->v_State() /* Cam */,
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()) /* XYZ */,
				m_vertex_ptr.Get<2>()->v_State(), v_error, tmp);

		//e2 = v_error;
		v_error -= m_v_measurement; // this is actually negative error, but it is squared below so the chi2 is the same
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)
		/*std::cout << "chi: " << (v_error.transpose() * m_t_sigma_inv).dot(v_error) << " " << m_v_measurement.transpose() <<
				" " << e2.transpose() << std::endl;*/

		return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i*/
	}

	/**
	 *	@brief calculates reprojection error
	 *	@return Returns reprojection error for this edge, in pixels.
	 */
	inline double f_Reprojection_Error() const
	{
		Eigen::Vector2d v_expectation;
		bool tmp = true;
		CBAJacobians::Project_P2C2(m_vertex_ptr.Get<0>()->v_State() /* Cam */,
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()) /* XYZ */,
				m_vertex_ptr.Get<2>()->v_State(), v_expectation, tmp);

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
		CBAJacobians::Project_P2C2(m_vertex_ptr.Get<0>()->v_State() /* Cam */,
				CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<1>()->v_State()) /* XYZ */,
				m_vertex_ptr.Get<2>()->v_State(), v_expectation,
				r_b_vertex_in_front_of_camera);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI2_InvDepth_LS : public CBaseEdgeImpl<CEdgeP2CI2_InvDepth_LS, MakeTypelist(CVertexInvDepth, CVertexIntrinsicsReduced), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd>{
public:
	typedef CBaseEdgeImpl<CEdgeP2CI2_InvDepth_LS, MakeTypelist(CVertexInvDepth, CVertexIntrinsicsReduced), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI2_InvDepth_LS()
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
	CEdgeP2CI2_InvDepth_LS(size_t n_node_0, size_t n_node_1, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(n_node_0, n_node_1, v_delta, r_t_inv_sigma)
	{
		m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_node_0, CInitializeNullVertex<CVertexInvDepth>());
		m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexIntrinsicsReduced>(n_node_1, CInitializeNullVertex<CVertexIntrinsicsReduced>());
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
			Eigen::Matrix<double, 2, 2> &r_t_jacobian1,	Eigen::Matrix<double, 2, 1> &r_v_expectation,
			Eigen::Matrix<double, 2, 1> &r_v_error) const // change dimensionality of eigen types, if required
	{
		CBAJacobians::Project_P2CI2_Self_InvDepth(m_p_vertex0->r_v_State(), // landmark
				m_p_vertex1->r_v_State(), // intrinsics
			r_v_expectation, r_t_jacobian0,	r_t_jacobian1);
		// calculates the expectation and the jacobians

		//std::cout << "LS " << r_v_expectation.transpose() << " " << m_v_measurement.transpose() << std::endl;
		//std::cout << r_t_jacobian0 << std::endl << std::endl;
		//std::cout << r_t_jacobian1 << std::endl << std::endl;
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
		bool tmp = true;
		CBAJacobians::Project_P2C2(cam_zero, CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()),
				m_p_vertex1->r_v_State(), v_error, tmp);

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
		bool tmp = true;
		CBAJacobians::Project_P2C2(cam_zero, CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()),
				m_p_vertex1->r_v_State(), v_expectation, tmp);

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
		CBAJacobians::Project_P2C2(cam_zero, CSim3Jacobians::v_InvDepth_to_XYZ(m_p_vertex0->r_v_State()),
				m_p_vertex1->r_v_State(), v_expectation,
				r_b_vertex_in_front_of_camera);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI2_InvDepth_LO : public CBaseEdgeImpl<CEdgeP2CI2_InvDepth_LO, MakeTypelist(CVertexInvDepth, CVertexCam, CVertexIntrinsicsReduced, CVertexCam), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd>{
public:
	typedef CBaseEdgeImpl<CEdgeP2CI2_InvDepth_LO, MakeTypelist(CVertexInvDepth, CVertexCam, CVertexIntrinsicsReduced, CVertexCam), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI2_InvDepth_LO()
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
	CEdgeP2CI2_InvDepth_LO(size_t n_node_0, size_t n_node_1, size_t n_node_2, size_t n_node_3, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node_0,
		n_node_1, n_node_2, n_node_3), v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_node_0, CInitializeNullVertex<CVertexInvDepth>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCam>(n_node_1, CInitializeNullVertex<CVertexCam>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsicsReduced>(n_node_2, CInitializeNullVertex<CVertexIntrinsicsReduced>());
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
		CBAJacobians::Project_P2CI2_Other_InvDepth(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
			m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
		// calculates the expectation and the jacobians

		//std::cout << "LO: " << m_vertex_ptr.Get<1>()->v_State().transpose() << " | " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;
		//std::cout << r_t_jacobian_tuple.Get<0>() << std::endl << std::endl;
		//std::cout << r_t_jacobian_tuple.Get<1>() << std::endl << std::endl;
		//std::cout << r_t_jacobian_tuple.Get<2>() << std::endl << std::endl;
		//std::cout << r_t_jacobian_tuple.Get<3>() << std::endl << std::endl;

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
		bool tmp = true;
		CBAJacobians::Project_P2C2_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_error, tmp);

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
		bool tmp = true;
		CBAJacobians::Project_P2C2_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_expectation, tmp);

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
		CBAJacobians::Project_P2C2_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_expectation, r_b_vertex_in_front_of_camera);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/**
 *	@brief BA vex-cam edge
 */
class CEdgeP2CI2_InvDepth_LO_Sim3 : public CBaseEdgeImpl<CEdgeP2CI2_InvDepth_LO_Sim3, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexIntrinsicsReduced, CVertexCamSim3), 2, 2, CBaseEdge::Robust>,
		public CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd>{
public:
	typedef CBaseEdgeImpl<CEdgeP2CI2_InvDepth_LO_Sim3, MakeTypelist(CVertexInvDepth, CVertexCamSim3, CVertexIntrinsicsReduced, CVertexCamSim3), 2, 2, CBaseEdge::Robust> _TyBase; /**< @brief base edge type */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CEdgeP2CI2_InvDepth_LO_Sim3()
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
	CEdgeP2CI2_InvDepth_LO_Sim3(size_t n_node_0, size_t n_node_1, size_t n_node_2, size_t n_node_3, const Eigen::Vector2d &v_delta, // won't align non-reference arg
		const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system) // respect const-ness!
		:_TyBase(typename _TyBase::_TyVertexIndexTuple(n_node_0,
		n_node_1, n_node_2, n_node_3), v_delta, r_t_inv_sigma)
	{
		m_vertex_ptr.Get<0>() = &r_system.template r_Get_Vertex<CVertexInvDepth>(n_node_0, CInitializeNullVertex<CVertexInvDepth>());
		m_vertex_ptr.Get<1>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_node_1, CInitializeNullVertex<CVertexCamSim3>());
		m_vertex_ptr.Get<2>() = &r_system.template r_Get_Vertex<CVertexIntrinsicsReduced>(n_node_2, CInitializeNullVertex<CVertexIntrinsicsReduced>());
		m_vertex_ptr.Get<3>() = &r_system.template r_Get_Vertex<CVertexCamSim3>(n_node_3, CInitializeNullVertex<CVertexCamSim3>());
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
		CSim3Jacobians::Project_P2CI2_LocalInvDepth_Other(m_vertex_ptr.Get<0>()->v_State(), m_vertex_ptr.Get<1>()->v_State(),
			m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), r_v_expectation,
			r_t_jacobian_tuple.Get<0>(), r_t_jacobian_tuple.Get<1>(),
			r_t_jacobian_tuple.Get<2>(), r_t_jacobian_tuple.Get<3>());
		// calculates the expectation and the jacobians

		//std::cout << "LO: " << m_vertex_ptr.Get<1>()->v_State().transpose() << " | " << r_v_expectation.transpose() << " | " << m_v_measurement.transpose() << std::endl;

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
		bool tmp =
		CSim3Jacobians::Project_P2CI2_LocalXYZ_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
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
		bool tmp =
		CSim3Jacobians::Project_P2CI2_LocalXYZ_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
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
		r_b_vertex_in_front_of_camera =
		CSim3Jacobians::Project_P2CI2_LocalXYZ_Other(CSim3Jacobians::v_InvDepth_to_XYZ(m_vertex_ptr.Get<0>()->v_State()), m_vertex_ptr.Get<1>()->v_State(),
					m_vertex_ptr.Get<2>()->v_State(), m_vertex_ptr.Get<3>()->v_State(), v_expectation);

		Eigen::Vector2d v_error = m_v_measurement - v_expectation;
		// this should be faster, as it does not calculate the jacobians
		// (in BA, this is actually timed as it is used in solver step validation)

		return v_error.norm();
	}
};

/** @} */ // end of group

#endif // !__SIM3_PRIMITIVE_TYPES_INCLUDED
