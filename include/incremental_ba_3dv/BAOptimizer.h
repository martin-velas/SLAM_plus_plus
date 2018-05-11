/*
								+----------------------------------+
								|                                  |
								| ***  BA optimizer interface  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|          BAOptimizer.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __BA_OPTIMIZER_INCLUDED
#define __BA_OPTIMIZER_INCLUDED

/**
 *	@file include/incremental_ba_3dv/BAOptimizer.h
 *	@brief experimental 3DV incremental bundle adjustment optimizer interface
 *	@date 2014-03-20
 *	@author -tHE SWINe-
 */

#ifdef __cplusplus // this is required if you want to use SLAM++ from pure "C"

/**
 *	@def BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS
 *	@brief if defined, it is possible to access optimizer vertex objects;
 *		otherwise only vertex state vectors can be accessed
 *	@note If not defined, the optimizer does not need to be included,
 *		resulting in much faster compilation times of files including this.
 */
//#define BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS

#ifdef BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS
#include "slam/LinearSolver_UberBlock.h"
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/Sim3_Types.h"
#else // BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS
#include "slam/Sim3SolverBase.h" // C3DJacobians, CBAJacobians, generally useful functions for BA and SE(3), does not need to be included
#endif // BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS
#include <map>

/**
 *	@brief bundle adjustment optimizer
 *	@note The purpose is not to have any interaction with CSystemType or CNonlinearSolverType
 *		in the header, in order to move the slow building of SLAM++ to BAOptimizer.cpp, which
 *		is rarely changed throughout the development process.
 */
class CBAOptimizer {
public:
#ifdef BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS
	typedef MakeTypelist_Safe((CVertexCamSim3, CVertexInvDepth)) TVertexTypelist;
	typedef MakeTypelist_Safe((CEdgeP2CSim3G)) TEdgeTypelist;
	typedef CFlatSystem<CSEBaseVertex, TVertexTypelist, CEdgeP2CSim3G, TEdgeTypelist> CSystemType;
#endif // BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS

	class CBAOptimizerCore; // forward declaration

protected:
	CBAOptimizerCore *m_p_optimizer; // PIMPL
	std::map<size_t, size_t> m_camera_ownerships; // using a side channel now. this should be a part of the CVertexInvDepth class but I actually want to experiment a bit with global / local convergence so I'm leaving it here for now
	size_t m_n_last_camera_vertex; // remember the last inserted camera vertex so that adding vertices in global space can still work even when optimizing in local space (will use the last camera as the anchor for the points added as global points - allows processing of incremental SE(3) datasets easily)

public:
	CBAOptimizer(bool b_verbose = false, bool b_use_schur = true,
		bool b_do_marginals = false, bool b_do_icra_style_marginals = false); // throw(srd::bad_alloc) // todo - add more optimization settings as required
	~CBAOptimizer();

	size_t n_Vertex_Num() const;

#ifdef SBA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS
	CSystemType::_TyConstVertexRef r_Vertex(size_t n_index) const; // returns const reference to a full vertex
	CSystemType::_TyVertexRef r_Vertex(size_t n_index); // returns reference to a full vertex
#endif // SBA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS

	Eigen::Map<const Eigen::VectorXd> r_Vertex_State(size_t n_index) const; // returns const map of vertex state only
	Eigen::Map<Eigen::VectorXd> r_Vertex_State(size_t n_index); // returns map of vertex state only

	// these can only be used with landmark ids, not camera ids
	Eigen::Vector3d v_XYZVertex_Global(size_t n_index) const;
	Eigen::Vector3d v_XYZVertex_Local(size_t n_index) const;
	Eigen::Vector3d v_InvDepthVertex_Global(size_t n_index) const;
	Eigen::Vector3d v_InvDepthVertex_Local(size_t n_index) const;
	Eigen::Vector4d v_InvDistVertex_Global(size_t n_index) const;
	Eigen::Vector4d v_InvDistVertex_Local(size_t n_index) const;

	void Delay_Optimization();
	void Enable_Optimization(); // throw(srd::bad_alloc, std::runtime_error)

	void Set_TrustRadius(double f_trust_radius);
	void Set_TrustRadius_Persistence(bool b_trust_radius_persistent);
	void Set_UpdateThreshold(double f_update_thresh);
	void Set_RelinThreshold(double f_relin_thresh);
	void Set_ForceIncSchur(bool b_force_inc_schur);
	void Set_AllBatch(bool b_all_batch);
	void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01, double f_min_dl_norm = .01); // throw(srd::bad_alloc, std::runtime_error)

	void Add_CamSim3Vertex(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state); // throw(srd::bad_alloc)

	// a vertex can be added either way, it gets automatically converted to the representation space
	void Add_XYZVertex_Global(size_t n_vertex_id, const Eigen::Vector3d &v_xyz_position); // throw(srd::bad_alloc)
	void Add_XYZVertex_Local(size_t n_vertex_id, const Eigen::Vector3d &v_xyz_position, size_t n_observing_camera_id); // throw(srd::bad_alloc)
	void Add_InvDepthVertex_Global(size_t n_vertex_id, const Eigen::Vector3d &v_inv_depth_position); // throw(srd::bad_alloc)
	void Add_InvDepthVertex_Local(size_t n_vertex_id, const Eigen::Vector3d &v_inv_depth_position, size_t n_observing_camera_id); // throw(srd::bad_alloc)
	void Add_InvDistVertex_Global(size_t n_vertex_id, const Eigen::Vector4d &v_inv_dist_position); // throw(srd::bad_alloc)
	void Add_InvDistVertex_Local(size_t n_vertex_id, const Eigen::Vector4d &v_inv_dist_position, size_t n_observing_camera_id); // throw(srd::bad_alloc)

	void Add_P2CSim3GEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
		const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information); // throw(srd::bad_alloc)

	void Show_Stats(bool b_calculate_eigenvalues = true) const;

	bool Dump_State(const char *p_s_filename) const; // note that this saves the state as Sim(3)
	bool Dump_Marginals(const char *p_s_filename) const; // note that this saves the marginals of (relative?) Sim(3)

	bool Dump_Poses_SE3(const char *p_s_filename) const; // only poses, not landmarks
	bool Dump_State_SE3(const char *p_s_filename) const; // note that this converts the state to SE(3) for viewing by our utility which does not support Sim(3) now
	bool Dump_Graph_SE3(const char *p_s_filename) const; // note that this converts the graph to SE(3) for viewing by our utility which does not support Sim(3) now

	/*void Add_SCamVertex(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state); // throw(srd::bad_alloc)
	void Add_SCamVertex(const CParserBase::TVertexSCam3D & vertex_scam); // throw(srd::bad_alloc)
	void Add_IntrinsicsVertex(size_t n_vertex_id, const Eigen::Matrix<double, 5, 1> &v_state); // throw(srd::bad_alloc)
	void Add_XYZVertex(const CParserBase::TVertexXYZ &r_t_vertex_xyz); // throw(srd::bad_alloc)
	void Add_P2SC3DEdge(size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
			const Eigen::Vector3d &v_observation, const Eigen::Matrix3d &t_information); // throw(srd::bad_alloc)
	void Add_P2SC3DEdge(const CParserBase::TEdgeP2SC3D & edge_p2sc3dT); // throw(srd::bad_alloc)
	void Add_P2CI3DEdge(size_t n_xyz_vertex_id, size_t n_cam_vertex_id, size_t n_intrinsics_vertex_id,
		const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information); // throw(srd::bad_alloc)*/
	// other types may be supported, as needed

	/**
	 *	@brief rasterizes a sparse matrix and saves as a .tga images
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] A is input matrix
	 *	@param[in] A_prev is previous state of the input matrix, for change tracking (can be 0)
	 *	@param[in] n_max_bitmap_size is the maximal size of the output, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Dump_SparseMatrix_SubsampleDx(const char *p_s_filename, const cs *A,
		const Eigen::VectorXd &r_v_dx, size_t n_max_bitmap_size = 640, bool b_symmetric = false);

	/**
	 *	@brief rasterizes a sparse matrix and saves as a .tga images
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] A is input matrix
	 *	@param[in] r_v_dx is reference to the dx vector for affected variable highlighting
	 *	@param[in] n_max_bitmap_size is the maximal size of the output, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Dump_SparseMatrix_SubsampleDx_AA(const char *p_s_filename, const cs *A,
		const Eigen::VectorXd &r_v_dx, size_t n_max_bitmap_size = 640, bool b_symmetric = false);

protected:
	Eigen::Vector3d v_LandmarkState_XYZ(size_t n_index) const;

private:
	CBAOptimizer(const CBAOptimizer &r_other); // no copy
	CBAOptimizer &operator =(const CBAOptimizer &r_other); // no copy
};

#endif // __cplusplus

#if defined(ENABLE_BA_OPTIMIZER_C_INTERFACE) || !defined(__cplusplus)

#ifdef __cplusplus
extern "C" { // we might want to use the same interface from C++ (ENABLE_BA_OPTIMIZER_C_INTERFACE must be defined in the file which includes this, not necessarily for all the files)
#endif // __cplusplus

// this is a "C" interface for the same optimizer - now you can call SLAM++ from pure "C"
// in case your compiler doesnt know size_t, typedef unsigned long size_t could work

typedef struct _optimizer_t *optimizer_t; /**< @brief optimizer handle type */

optimizer_t New_Optimizer(int b_verbose, int b_use_schur);
void Free_Optimizer(optimizer_t h_optimizer);
size_t n_Vertex_Num(optimizer_t h_optimizer);
double *p_Vertex_State(optimizer_t h_optimizer, size_t n_vertex);
int n_Vertex_Dimension(optimizer_t h_optimizer, size_t n_vertex);
void Delay_Optimization(optimizer_t h_optimizer);
void Enable_Optimization(optimizer_t h_optimizer);
void Optimize(optimizer_t h_optimizer, size_t n_max_iteration_num, double f_min_dx_norm);
void Add_CamSim3Vertex(optimizer_t h_optimizer, size_t n_vertex_id, const double p_cam_state[12]);
void Add_InvDepthVertex_Global(optimizer_t h_optimizer, size_t n_vertex_id, const double p_position[3]);
void Add_InvDepthVertex_Local(optimizer_t h_optimizer, size_t n_vertex_id, const double p_position[3], size_t n_observing_camera_id);
void Add_P2CSim3GEdge(optimizer_t h_optimizer, size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
	const double p_observation[2], const double p_information[4]);
int Dump_State(optimizer_t h_optimizer, const char *p_s_filename);
int Dump_State_SE3(optimizer_t h_optimizer, const char *p_s_filename);
int Dump_Graph_SE3(optimizer_t h_optimizer, const char *p_s_filename);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ENABLE_BA_OPTIMIZER_C_INTERFACE || !__cplusplus

#endif // !__BA_OPTIMIZER_INCLUDED
