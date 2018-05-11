/*
								+----------------------------------+
								|                                  |
								| ***  BA optimizer interface  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|         BAOptimizer.cpp          |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/incremental_ba_3dv/BAOptimizer.cpp
 *	@brief SLAM++ bundle adjustment optimizer interface
 *	@date 2014-03-20
 *	@author -tHE SWINe-
 */

#define LANDMARK_TYPE_XYZ
//#define LANDMARK_TYPE_InvDepth
//#define LANDMARK_TYPE_InvDist

#define LANDMARKS_LOCAL
//#define LANDMARKS_GLOBAL

#define SIM3_USE_ROBUST_EDGES

//#define USE_EXPLICIT_HARD_UF // use explicit UF via a const vertex?

#include <string.h>
#include <stdio.h>
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#include "slam/LinearSolver_UberBlock.h"
//#include "slam/LinearSolver_CholMod.h" // not used
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
//#include "slam/Timer.h" // not used
#include "slam/3DSolverBase.h" // want C3DJacobians::Quat_to_AxisAngle() and C3DJacobians::AxisAngle_to_Quat()
#include "slam/Sim3_Types.h"
//#include "slam/Marginals.h" // included from nonlinear solver, if supported
#include "slam/NonlinearSolver_Lambda_DL.h"
//#include "slam/BASolverBase.h" // included from BA_Types.h
#include "incremental_ba_3dv/BAOptimizer.h"
#include "slam/Eigenvalues.h"

/*
 *								=== CDelayedOptimizationCaller ===
 */

template <class CSolverType, const bool b_can_delay>
class CDelayedOptimizationCaller {
public:
	static void Delay(CSolverType &UNUSED(r_solver))
	{}

	static void Enable(CSolverType &UNUSED(r_solver))
	{}
};

template <class CSolverType>
class CDelayedOptimizationCaller<CSolverType, true> {
public:
	static void Delay(CSolverType &r_solver)
	{
		r_solver.Delay_Optimization();
	}

	static void Enable(CSolverType &r_solver)
	{
		r_solver.Enable_Optimization();
	}
};

/*
 *								=== ~CDelayedOptimizationCaller ===
 */

/*
 *								=== CBAOptimizer ===
 */

#if defined(LANDMARK_TYPE_XYZ)
typedef CVertexXYZ _TyLandmark;
#ifdef LANDMARKS_LOCAL
typedef CEdgeP2C_XYZ_Sim3_LS _TyObservationS;
typedef CEdgeP2C_XYZ_Sim3_LO _TyObservationO;
#else // LANDMARKS_LOCAL
typedef CEdgeP2C_XYZ_Sim3_G _TyObservation;
#endif // LANDMARKS_LOCAL
#elif defined(LANDMARK_TYPE_InvDepth)
typedef CVertexInvDepth _TyLandmark;
#ifdef LANDMARKS_LOCAL
typedef CEdgeP2C_InvDepth_Sim3_LS _TyObservationS;
typedef CEdgeP2C_InvDepth_Sim3_LO _TyObservationO;
#else // LANDMARKS_LOCAL
typedef CEdgeP2C_InvDepth_Sim3_G _TyObservation;
#endif // LANDMARKS_LOCAL
#elif defined(LANDMARK_TYPE_InvDist)
typedef CVertexInvDist _TyLandmark;
#ifdef LANDMARKS_LOCAL
typedef CEdgeP2C_InvDist_Sim3_LS _TyObservationS;
typedef CEdgeP2C_InvDist_Sim3_LO _TyObservationO;
#else // LANDMARKS_LOCAL
typedef CEdgeP2C_InvDist_Sim3_G _TyObservation;
#endif // LANDMARKS_LOCAL
#else
#error "fatal error: landmark type undefined"
#endif
// define the optimizer types

class CBAOptimizer::CBAOptimizerCore {
public:
	typedef MakeTypelist_Safe((CVertexCamSim3, _TyLandmark)) TVertexTypelist;
#ifdef USE_EXPLICIT_HARD_UF
	typedef MakeTypelist_Safe((CVertexCamSim3)) TConstVertexTypelist;
#endif // USE_EXPLICIT_HARD_UF
#ifdef LANDMARKS_LOCAL
#ifdef USE_EXPLICIT_HARD_UF
	typedef MakeTypelist_Safe((_TyObservationS, _TyObservationO, CEdgePoseCamSim3)) TEdgeTypelist;
	typedef CFlatSystem<CBaseVertex, TVertexTypelist, CBaseEdge, TEdgeTypelist, CNullUnaryFactorFactory,
		CVertexCamSim3, TConstVertexTypelist> CSystemType;
#else // USE_EXPLICIT_HARD_UF
	typedef MakeTypelist_Safe((_TyObservationS, _TyObservationO)) TEdgeTypelist;
	typedef CFlatSystem<CBaseVertex, TVertexTypelist, CBaseEdge, TEdgeTypelist> CSystemType;
#endif // USE_EXPLICIT_HARD_UF
#else // LANDMARKS_LOCAL
#ifdef USE_EXPLICIT_HARD_UF
	typedef MakeTypelist_Safe((_TyObservation, CEdgePoseCamSim3)) TEdgeTypelist;
	typedef CFlatSystem<CBaseVertex, TVertexTypelist, CBaseEdge, TEdgeTypelist, CNullUnaryFactorFactory,
		CVertexCamSim3, TConstVertexTypelist> CSystemType;
#else // USE_EXPLICIT_HARD_UF
	typedef MakeTypelist_Safe((_TyObservation)) TEdgeTypelist;
	typedef CFlatSystem<CBaseVertex, TVertexTypelist, _TyObservation, TEdgeTypelist> CSystemType;
#endif // USE_EXPLICIT_HARD_UF
#endif // LANDMARKS_LOCAL
	// note that if BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS is defined, any modifications here need to be carried also to BAOptimizer.h
	// also the "C" interface would need to be modified

	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
	typedef CNonlinearSolver_Lambda_DL<CSystemType, CLinearSolverType> CNonlinearSolverType;

protected:
	CSystemType m_system;
	CNonlinearSolverType m_solver;

public:
	inline CBAOptimizerCore(bool b_verbose = false, bool b_use_schur = true,
		bool b_do_marginals = false, bool b_do_icra_style_marginals = false)
		:m_solver(m_system, TIncrementalSolveSetting(solve::nonlinear, frequency::Every(-1)), // configure this as an incremental solver, solve every SIZE_MAX vertices added (that amounts to never, but the solver prints less verbose that way)
		TMarginalsComputationPolicy((b_do_marginals)? marginals::do_calculate : marginals::do_not_calculate,
		frequency::Never(), mpart_Diagonal, mpart_Diagonal), // batch marginals
		b_verbose, CLinearSolverType(), b_use_schur)
	{
		m_solver.Set_ICRA15_Style_IncrementalMargs(b_do_icra_style_marginals); // !!

#if defined(LANDMARK_TYPE_XYZ)
		fprintf(stderr, "debug: built with LANDMARK_TYPE_XYZ\n");
#elif defined(LANDMARK_TYPE_InvDepth)
		fprintf(stderr, "debug: built with LANDMARK_TYPE_InvDepth\n");
#elif defined(LANDMARK_TYPE_InvDist)
		fprintf(stderr, "debug: built with LANDMARK_TYPE_InvDist\n");
#endif // LANDMARK_TYPE_XYZ
#if defined(LANDMARKS_LOCAL)
		fprintf(stderr, "debug: built with LANDMARKS_LOCAL\n");
#elif defined(LANDMARKS_GLOBAL)
		fprintf(stderr, "debug: built with LANDMARKS_GLOBAL\n");
#endif // LANDMARKS_LOCAL
	}

	CSystemType::_TyConstVertexRef r_Vertex(size_t n_index) const
	{
		return m_system.r_Vertex_Pool()[n_index];
	}

	CSystemType::_TyVertexRef r_Vertex(size_t n_index)
	{
		return m_system.r_Vertex_Pool()[n_index];
	}

	inline CSystemType &r_System()
	{
		return m_system;
	}

	inline CNonlinearSolverType &r_Solver()
	{
		return m_solver;
	}

	inline const CSystemType &r_System() const
	{
		return m_system;
	}

	inline const CNonlinearSolverType &r_Solver() const
	{
		return m_solver;
	}

private:
	CBAOptimizerCore(const CBAOptimizerCore &r_other); // no copy
	CBAOptimizerCore &operator =(const CBAOptimizerCore &r_other); // no copy
};

CBAOptimizer::CBAOptimizer(bool b_verbose, bool b_use_schur,
	bool b_do_marginals, bool b_do_icra_style_marginals) // throw(srd::bad_alloc)
	:m_p_optimizer(new CBAOptimizerCore(b_verbose, b_use_schur,
	b_do_marginals, b_do_icra_style_marginals)), m_n_last_camera_vertex(size_t(-1))
{}

CBAOptimizer::~CBAOptimizer()
{
	delete m_p_optimizer;
}

size_t CBAOptimizer::n_Vertex_Num() const
{
	return m_p_optimizer->r_System().r_Vertex_Pool().n_Size();
}

#ifdef BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS

CBAOptimizer::CSystemType::_TyConstVertexRef CBAOptimizer::r_Vertex(size_t n_index) const
{
	return m_p_optimizer->r_System().r_Vertex_Pool()[n_index];
}

CBAOptimizer::CSystemType::_TyVertexRef CBAOptimizer::r_Vertex(size_t n_index)
{
	return m_p_optimizer->r_System().r_Vertex_Pool()[n_index];
}

#endif // BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS

Eigen::Map<const Eigen::VectorXd> CBAOptimizer::r_Vertex_State(size_t n_index) const
{
	return ((CBAOptimizerCore::CSystemType::_TyConstVertexRef)m_p_optimizer->r_Vertex(n_index)).v_StateC(); // explicit const required in windows for some reason // todo - this was working before; investigate
}

Eigen::Map<Eigen::VectorXd> CBAOptimizer::r_Vertex_State(size_t n_index)
{
	return m_p_optimizer->r_Vertex(n_index).v_State();
}

void CBAOptimizer::Delay_Optimization()
{
	CDelayedOptimizationCaller<CBAOptimizerCore::CNonlinearSolverType,
		CBAOptimizerCore::CNonlinearSolverType::solver_HasDelayedOptimization>::Delay(m_p_optimizer->r_Solver());
	// some solvers do not implement this
}

void CBAOptimizer::Enable_Optimization() // throw(srd::bad_alloc, std::runtime_error)
{
	CDelayedOptimizationCaller<CBAOptimizerCore::CNonlinearSolverType,
		CBAOptimizerCore::CNonlinearSolverType::solver_HasDelayedOptimization>::Enable(m_p_optimizer->r_Solver());
	// some solvers do not implement this
}

void CBAOptimizer::Set_TrustRadius(double f_trust_radius)
{
	m_p_optimizer->r_Solver().Set_StepSize(f_trust_radius);
}

void CBAOptimizer::Set_TrustRadius_Persistence(bool b_trust_radius_persistent)
{
	m_p_optimizer->r_Solver().Set_StepSize_Persistence(b_trust_radius_persistent);
}

void CBAOptimizer::Set_UpdateThreshold(double f_update_thresh)
{
	m_p_optimizer->r_Solver().Set_UpdateThreshold(f_update_thresh);
}

void CBAOptimizer::Set_RelinThreshold(double f_relin_thresh)
{
	m_p_optimizer->r_Solver().Set_RelinearizationThreshold(f_relin_thresh);
}

void CBAOptimizer::Set_AllBatch(bool b_all_batch)
{
	m_p_optimizer->r_Solver().Set_AllBatch(b_all_batch);
}

void CBAOptimizer::Set_ForceIncSchur(bool b_force_inc_schur)
{
	m_p_optimizer->r_Solver().Set_ForceIncSchur(b_force_inc_schur);
}

void CBAOptimizer::Optimize(size_t n_max_iteration_num /*= 5*/,
	double f_min_dx_norm /*= .01*/, double f_min_dl_norm /*= .01*/) // throw(srd::bad_alloc, std::runtime_error)
{
	m_p_optimizer->r_Solver().Optimize(n_max_iteration_num, f_min_dx_norm, f_min_dl_norm);

#if 0
	static size_t n_iteration = 0;
	char p_s_filename[256];
	sprintf(p_s_filename, "lambda_struct_dx_%05" _PRIsize ".tga", n_iteration);
	++ n_iteration;
	cs *p_bs = m_p_optimizer->r_Solver().r_Lambda().p_BlockStructure_to_Sparse();
	Dump_SparseMatrix_SubsampleDx(p_s_filename, p_bs, m_p_optimizer->r_Solver().r_v_LastDx(), 640, true);
	cs_spfree(p_bs);
#endif // 0
}

/*void CBAOptimizer::Add_CamVertex(size_t n_vertex_id, const Eigen::Matrix<double, 11, 1> &v_cam_state) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Get_Vertex<CVertexCam>(n_vertex_id, v_cam_state);
}

void CBAOptimizer::Add_XYZVertex(size_t n_vertex_id, const Eigen::Vector3d &v_position) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Get_Vertex<CVertexXYZ>(n_vertex_id, v_position);
}

void CBAOptimizer::Add_P2C3DEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
	const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Add_Edge(CEdgeP2C3D(n_landmark_vertex_id, n_cam_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
}*/

void CBAOptimizer::Add_CamSim3Vertex(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state) // throw(srd::bad_alloc)
{
	CVertexCamSim3 &r_cam0 = m_p_optimizer->r_System().r_Get_Vertex<CVertexCamSim3>(n_vertex_id, v_cam_state);
#ifdef USE_EXPLICIT_HARD_UF
	if(m_n_last_camera_vertex == size_t(-1)) {
		m_p_optimizer->r_System().r_Add_Edge(CEdgePoseCamSim3(n_vertex_id, size_t(-1),
			Eigen::Vector7d::Zero(), Eigen::Matrix7d::Identity() * 100, m_p_optimizer->r_System()));
		// add an explicit UF edge

		printf("debug: adding an explicit \'hard\' UF edge\n");
	}
#endif // USE_EXPLICIT_HARD_UF
	m_n_last_camera_vertex = n_vertex_id;
}

void CBAOptimizer::Add_XYZVertex_Global(size_t n_vertex_id, const Eigen::Vector3d &v_xyz_position) // throw(srd::bad_alloc)
{
	_ASSERTE(n_vertex_id <= INT_MAX);
#ifdef LANDMARKS_LOCAL
	if(m_n_last_camera_vertex == size_t(-1))
		throw std::runtime_error("no camera to become owner of a landmark");
	const CVertexCamSim3 &r_camera = m_p_optimizer->r_System().r_Vertex_Pool().r_At<CVertexCamSim3>(m_n_last_camera_vertex);
	// get the owner camera

	Eigen::Vector3d v_position = v_xyz_position;

	{
		v_position = CSim3Jacobians::TSim3(r_camera.r_v_State(), CSim3Jacobians::TSim3::from_sim3_vector).v_InvTransform(v_position);
		// rotate by the camera
	}
	// bring the point to the camera local space

	m_p_optimizer->r_System().r_Get_Vertex<_TyLandmark>(n_vertex_id,
		CParserBase::TVertexXYZ(int(n_vertex_id), v_position(0), v_position(1), v_position(2))); // initialize via parse primitive, let the vertex class convert to its representation
	m_camera_ownerships[n_vertex_id] = m_n_last_camera_vertex; // the vertex is owned by this camera
#else // LANDMARKS_LOCAL
	m_p_optimizer->r_System().r_Get_Vertex<_TyLandmark>(n_vertex_id,
		CParserBase::TVertexXYZ(int(n_vertex_id), v_xyz_position(0), v_xyz_position(1), v_xyz_position(2))); // initialize via parse primitive, let the vertex class convert to its representation
	m_camera_ownerships[n_vertex_id] = size_t(-1); // vertex defined in global space
#endif // LANDMARKS_LOCAL
}

void CBAOptimizer::Add_XYZVertex_Local(size_t n_vertex_id, const Eigen::Vector3d &v_xyz_position, size_t n_observing_camera_id) // throw(srd::bad_alloc)
{
	const CVertexCamSim3 &r_camera = m_p_optimizer->r_System().r_Vertex_Pool().r_At<CVertexCamSim3>(n_observing_camera_id);

	Eigen::Vector3d v_position = v_xyz_position;

#ifdef LANDMARKS_LOCAL
	// position already in local space, do nothing
#else // LANDMARKS_LOCAL
	{
		CSim3Jacobians::TSim3 t_cam;
		t_cam.Exp(r_camera.r_v_State());
		v_position = t_cam * v_position;
		// rotate by the camera
	}
	// this is needed for the optimization in global space (the vertices are not local to their cameras now)
#endif // LANDMARKS_LOCAL

	_ASSERTE(n_vertex_id <= INT_MAX);
	m_p_optimizer->r_System().r_Get_Vertex<_TyLandmark>(n_vertex_id,
		CParserBase::TVertexXYZ(int(n_vertex_id), v_position(0), v_position(1), v_position(2))); // initialize via parse primitive, let the vertex class convert to its representation
	m_camera_ownerships[n_vertex_id] = n_observing_camera_id; // observed by this camera
}

void CBAOptimizer::Add_InvDepthVertex_Global(size_t n_vertex_id, const Eigen::Vector3d &v_inv_depth_position) // throw(srd::bad_alloc)
{
	Add_XYZVertex_Global(n_vertex_id, CSim3Jacobians::v_InvDepth_to_XYZ(v_inv_depth_position));
}

void CBAOptimizer::Add_InvDepthVertex_Local(size_t n_vertex_id, const Eigen::Vector3d &v_inv_depth_position, size_t n_observing_camera_id) // throw(srd::bad_alloc)
{
	Add_XYZVertex_Local(n_vertex_id, CSim3Jacobians::v_InvDepth_to_XYZ(v_inv_depth_position), n_observing_camera_id);
}

void CBAOptimizer::Add_InvDistVertex_Global(size_t n_vertex_id, const Eigen::Vector4d &v_inv_dist_position) // throw(srd::bad_alloc)
{
	Add_XYZVertex_Global(n_vertex_id, CSim3Jacobians::v_InvDist_to_XYZ(v_inv_dist_position));
}

void CBAOptimizer::Add_InvDistVertex_Local(size_t n_vertex_id, const Eigen::Vector4d &v_inv_dist_position, size_t n_observing_camera_id) // throw(srd::bad_alloc)
{
	Add_XYZVertex_Local(n_vertex_id, CSim3Jacobians::v_InvDist_to_XYZ(v_inv_dist_position), n_observing_camera_id);
}

Eigen::Vector3d CBAOptimizer::v_LandmarkState_XYZ(size_t n_vertex_id) const
{
#if defined(LANDMARK_TYPE_XYZ)
	return m_p_optimizer->r_System().r_Vertex_Pool().r_At<_TyLandmark>(n_vertex_id).r_v_State();
#elif defined(LANDMARK_TYPE_InvDepth)
	return CSim3Jacobians::v_InvDepth_to_XYZ(m_p_optimizer->r_System().r_Vertex_Pool().r_At<_TyLandmark>(n_vertex_id).r_v_State());
#else // LANDMARK_TYPE_XYZ
	return CSim3Jacobians::v_InvDist_to_XYZ(m_p_optimizer->r_System().r_Vertex_Pool().r_At<_TyLandmark>(n_vertex_id).v_State4()); // !! the optimized state is otherwise only 1D
#endif // LANDMARK_TYPE_XYZ
}

Eigen::Vector3d CBAOptimizer::v_XYZVertex_Global(size_t n_vertex_id) const
{
#ifdef LANDMARKS_LOCAL
	_ASSERTE(m_camera_ownerships.count(n_vertex_id)); // make sure this is owned by a camera and the next line does not in fact add a new record
	size_t n_observing_camera_id = (*m_camera_ownerships.find(n_vertex_id)).second;
	if(n_observing_camera_id == size_t(-1))
		throw std::runtime_error("global vertices not permitted in local mode");
	/*if(n_observing_camera_id == size_t(-1))
		return v_LandmarkState_XYZ(n_vertex_id); // not much to do about it
	else*/ {
		Eigen::Vector3d v_position = v_LandmarkState_XYZ(n_vertex_id);
		// convert vertex position to xyz

		const CVertexCamSim3 &r_camera = m_p_optimizer->r_System().r_Vertex_Pool().r_At<CVertexCamSim3>(n_observing_camera_id);
		// get camera state

		return CSim3Jacobians::TSim3(r_camera.r_v_State(), CSim3Jacobians::TSim3::from_sim3_vector) * v_position;
		// transform from camera local to global coordinates
	}
#else // LANDMARKS_LOCAL
	return v_LandmarkState_XYZ(n_vertex_id); // now the vertices are global
#endif // LANDMARKS_LOCAL
}

Eigen::Vector3d CBAOptimizer::v_XYZVertex_Local(size_t n_vertex_id) const
{
#ifdef LANDMARKS_LOCAL
	return v_LandmarkState_XYZ(n_vertex_id); // now the vertices are local
#else // LANDMARKS_LOCAL
	_ASSERTE(m_camera_ownerships.count(n_vertex_id)); // make sure this is owned by a camera and the next line does not in fact add a new record
	size_t n_observing_camera_id = (*m_camera_ownerships.find(n_vertex_id)).second;
	if(n_observing_camera_id == size_t(-1))
		return v_XYZVertex_Global(n_vertex_id); // the point is not owned by any camera
	else {
		Eigen::Vector3d v_position = v_LandmarkState_XYZ(n_vertex_id);
		// convert vertex position to xyz

		const CVertexCamSim3 &r_camera = m_p_optimizer->r_System().r_Vertex_Pool().r_At<CVertexCamSim3>(n_observing_camera_id);
		CSim3Jacobians::TSim3 t_cam;
		t_cam.Exp(r_camera.r_v_State());
		// get camera state

		return t_cam.v_InvTransform(v_position);
		// transform to camera local coordinates
	}
#endif // LANDMARKS_LOCAL
}

Eigen::Vector3d CBAOptimizer::v_InvDepthVertex_Global(size_t n_vertex_id) const
{
	return CSim3Jacobians::v_XYZ_to_InvDepth(v_XYZVertex_Global(n_vertex_id));
}

Eigen::Vector3d CBAOptimizer::v_InvDepthVertex_Local(size_t n_vertex_id) const
{
	return CSim3Jacobians::v_XYZ_to_InvDepth(v_XYZVertex_Local(n_vertex_id));
}

Eigen::Vector4d CBAOptimizer::v_InvDistVertex_Global(size_t n_vertex_id) const
{
	return CSim3Jacobians::v_XYZ_to_InvDist(v_XYZVertex_Global(n_vertex_id));
}

Eigen::Vector4d CBAOptimizer::v_InvDistVertex_Local(size_t n_vertex_id) const
{
	return CSim3Jacobians::v_XYZ_to_InvDist(v_XYZVertex_Local(n_vertex_id));
}

void CBAOptimizer::Add_P2CSim3GEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
	const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) // throw(srd::bad_alloc)
{
#ifdef LANDMARKS_LOCAL
	size_t n_owner_camera_id = (*m_camera_ownerships.find(n_landmark_vertex_id)).second;
	if(n_owner_camera_id == size_t(-1))
		throw std::runtime_error("global vertices not permitted in local mode");
	if(n_cam_vertex_id == n_owner_camera_id) {
		m_p_optimizer->r_System().r_Add_Edge(_TyObservationS(n_landmark_vertex_id, n_cam_vertex_id, // landmark, observing = owner
			v_observation, t_information, m_p_optimizer->r_System()));
	} else {
		m_p_optimizer->r_System().r_Add_Edge(_TyObservationO(n_landmark_vertex_id, n_cam_vertex_id, n_owner_camera_id, // landmark, observing, owner
			v_observation, t_information, m_p_optimizer->r_System()));
	}
#else // LANDMARKS_LOCAL
	m_p_optimizer->r_System().r_Add_Edge(_TyObservation(n_landmark_vertex_id, n_cam_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
#endif // LANDMARKS_LOCAL
}

/*void CBAOptimizer::Add_SCamVertex(size_t n_vertex_id, const Eigen::Matrix<double, 12, 1> &v_cam_state) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Get_Vertex<CVertexSCam>(n_vertex_id, v_cam_state);
}

void CBAOptimizer::Add_SCamVertex(const CParserBase::TVertexSCam3D & vertex_scam) // throw(srd::bad_alloc)
{
	CBAOptimizer::Add_SCamVertex(vertex_scam.m_n_id, vertex_scam.m_v_position);
}

void CBAOptimizer::Add_XYZVertex(const CParserBase::TVertexXYZ &r_t_vertex_xyz) // throw(srd::bad_alloc)
{
	CBAOptimizer::Add_XYZVertex(vertex_xyz.m_n_id, vertex_xyz.m_v_position);
}

void CBAOptimizer::Add_IntrinsicsVertex(size_t n_vertex_id, const Eigen::Matrix<double, 5, 1> &v_state) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Get_Vertex<CVertexIntrinsics>(n_vertex_id, v_state);
}

void CBAOptimizer::Add_P2SC3DEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
	const Eigen::Vector3d &v_observation, const Eigen::Matrix3d &t_information) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Add_Edge(CEdgeP2SC3D(n_landmark_vertex_id, n_cam_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
}

void CBAOptimizer::Add_P2SC3DEdge(const CParserBase::TEdgeP2SC3D & edge_p2sc3dT) // throw(srd::bad_alloc)
{
	CBAOptimizer::Add_P2SC3DEdge(edge_p2sc3dT.m_n_node_0, edge_p2sc3dT.m_n_node_1, edge_p2sc3dT.m_v_delta,
		edge_p2sc3dT.m_t_inv_sigma);
}

void CBAOptimizer::Add_P2CI3DEdge(size_t n_landmark_vertex_id, size_t n_cam_vertex_id, size_t n_intrinsics_vertex_id,
	const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Add_Edge(CEdgeP2CI3D(n_cam_vertex_id, n_landmark_vertex_id, n_intrinsics_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
}*/

class CCalcRepErr { // todo - maybe change this so that it can collect all the stats in a single For_Each() call
protected:
	double &m_r_f_dest;
	double &m_r_f_robust_weight;
	bool &m_r_b_visible;

public:
	CCalcRepErr(double &r_f_dest, double &r_f_robust_weight, bool &r_b_visible)
		:m_r_f_dest(r_f_dest), m_r_f_robust_weight(r_f_robust_weight), m_r_b_visible(r_b_visible)
	{}

	template <class _TyEdge>
	void operator ()(const _TyEdge &r_t_edge)
	{
		m_r_f_dest = r_t_edge.f_Reprojection_Error(m_r_b_visible);
#ifdef SIM3_USE_ROBUST_EDGES
		Eigen::Vector2d r; r(0) = m_r_f_dest; r(1) = 0; // can't initialize directly, the constructor is ambiguous // must be 2D if we use chi2 error, to be able to multiply chi2!
		m_r_f_robust_weight = r_t_edge.f_RobustWeight(r);
#else // SIM3_USE_ROBUST_EDGES
		m_r_f_robust_weight = 1;
#endif // SIM3_USE_ROBUST_EDGES
		// this function is not exported via the facade, needs to be called like this
	}

	void operator ()(const CEdgePoseCamSim3 &UNUSED(r_t_edge)) // ignore pose-pose edges for calculation of reprojection error
	{
		m_r_f_dest = 0;
		m_r_f_robust_weight = 0; // !!
	}
};

#define BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
#define BITMAP_SUBSAMPLE_DUMP_PARALLELIZE

bool CBAOptimizer::Dump_SparseMatrix_SubsampleDx(const char *p_s_filename, const cs *A,
	const Eigen::VectorXd &r_v_dx, size_t n_max_bitmap_size /*= 640*/, bool b_symmetric /*= false*/)
{
	_ASSERTE(CS_CSC(A));
	// the matrices need to be in compressed column form

	const uint32_t n_background = 0xffffffffU,
		n_nonzero = 0xff8080ffU,
		n_nonzero_new = 0xffff0000U,
		n_zero_new = 0xff00ff00U;
	// colors of the individual areas

	size_t m = A->m;
	size_t n = A->n;

	size_t mb = m, nb = n;
	double f_scale = double(n_max_bitmap_size) / std::max(m, n);
	if(f_scale < 1) {
		mb = std::max(size_t(1), size_t(m * f_scale));
		nb = std::max(size_t(1), size_t(n * f_scale));
    }
	// scale the bitmap down if needed

	if(mb == SIZE_MAX || nb == SIZE_MAX || mb > INT_MAX ||
	   nb > INT_MAX || uint64_t(mb) * nb > INT_MAX)
		return false;

	TBmp *p_bitmap;

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(mb), int(nb)))) // transposed
		return false;
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(nb), int(mb))))
		return false;
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	p_bitmap->Clear(n_background);

	{
#ifdef BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		const int _n = int(n);
		#pragma omp parallel for schedule(dynamic, 1)
		for(int n_col = 0; n_col < _n; ++ n_col) {
			for(size_t n_p0 = A->p[n_col], n_p1 = A->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
				size_t n_row = A->i[n_p0];
				double f_value = (A->x)? A->x[n_p0] : 1;
				// read the value
#else // BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		CDebug::CSparseMatrixShapedIterator p_a_it(A, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				// make sure it iterates to the right position

				double f_value = *p_a_it;
				// read the value
#endif // BITMAP_SUBSAMPLE_DUMP_PARALLELIZE

				if(f_value != 0) {
					uint32_t n_color = (fabs(r_v_dx(n_row)) > 1e-5f || fabs(r_v_dx(n_col)) > 1e-5f)? n_nonzero_new : n_nonzero;
#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					size_t n_y = size_t(n_col * f_scale);
					size_t n_x = size_t(n_row * f_scale); // transposed
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					size_t n_x = size_t(n_col * f_scale);
					size_t n_y = size_t(n_row * f_scale);
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					p_bitmap->PutPixel(int(n_x), int(n_y), n_color);
					/*if(b_symmetric && n_x != n_y)
						p_bitmap->PutPixel(int(n_y), int(n_x), n_color);*/
				}
				// draw a square into the bitmap
			}
		}

		if(b_symmetric) {
			uint32_t *p_buff = p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					uint32_t *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					uint32_t *p_lo = p_buff + (y + x * w);
					*p_up = std::min(*p_up, *p_lo); // choose the darker color
					*p_lo = *p_up; // duplicate the result to the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices
	}
	// iterate the matrix, draw small tiles

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if((m != n || !b_symmetric) && !p_bitmap->Transpose()) {
		p_bitmap->Delete();
		return false;
	}
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

	bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_bitmap, false);

	p_bitmap->Delete();

	return b_result;
}

bool CBAOptimizer::Dump_SparseMatrix_SubsampleDx_AA(const char *p_s_filename, const cs *A,
	const Eigen::VectorXd &r_v_dx, size_t n_max_bitmap_size /*= 640*/, bool b_symmetric /*= false*/)
{
	_ASSERTE(CS_CSC(A));
	// the matrices need to be in compressed column form

	const uint32_t n_background = 0xffffffffU,
		n_nonzero = 0xff8080ffU,
		n_nonzero_new = 0xffff0000U,
		n_zero_new = 0xff00ff00U;
	// colors of the individual areas

	size_t m = A->m;
	size_t n = A->n;

	size_t mb = m, nb = n;
	double f_scale = double(n_max_bitmap_size) / std::max(m, n);
	if(f_scale < 1) {
		mb = std::max(size_t(1), size_t(m * f_scale));
		nb = std::max(size_t(1), size_t(n * f_scale));
    }
	// scale the bitmap down if needed

	if(mb == SIZE_MAX || nb == SIZE_MAX || mb > INT_MAX ||
	   nb > INT_MAX || uint64_t(mb) * nb > INT_MAX)
		return false;

	TBmp *p_bitmap;

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(mb), int(nb)))) // transposed
		return false;
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(nb), int(mb))))
		return false;
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	p_bitmap->Clear(n_background);

	{
#ifdef BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		size_t n_half_stripe_num = nb / 8 - 1; // want four-pixel stripes with at least four pixel clearance (the blend will overflow up to two pixels - one on the left, one on the right from each stripe), want the last one to be wider than the others
		_ASSERTE(n_half_stripe_num < INT_MAX);
		const int _n_half_stripe_num = int(n_half_stripe_num);
		for(int n_pass = 0; n_pass < 2; ++ n_pass) { // draw even stripes in one pass, odd stripes in the next, this guarantees that two threads never blend over each other's area
			#pragma omp parallel for schedule(dynamic, 1)
			for(int n_stripe = 0; n_stripe < _n_half_stripe_num; ++ n_stripe) { // each thread processes a single stripe
				size_t n_stripe_b = (n / n_half_stripe_num) * n_stripe;
				size_t n_stripe_e = (n_stripe + 1 == _n_half_stripe_num)? n : n_stripe_b + n / n_half_stripe_num;
				_ASSERTE(n_stripe_e >= n_stripe_b + 8);
				if(n_pass)
					n_stripe_b = (n_stripe_b + n_stripe_e) / 2;
				else
					n_stripe_e = (n_stripe_b + n_stripe_e) / 2;
				_ASSERTE(n_stripe_e >= n_stripe_b + 2);
				// begin and end of a stripe

				for(size_t n_col = n_stripe_b; n_col < n_stripe_e; ++ n_col) {
					for(size_t n_p0 = A->p[n_col], n_p1 = A->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
						size_t n_row = A->i[n_p0];
						double f_value = (A->x)? A->x[n_p0] : 1;
						// read the value
#else // BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		{
			{
				CDebug::CSparseMatrixShapedIterator p_a_it(A, m, n);
				for(size_t n_col = 0; n_col < n; ++ n_col) {
					for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it) {
						_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
						// make sure it iterates to the right position

						double f_value = *p_a_it;
						// read the value
#endif // BITMAP_SUBSAMPLE_DUMP_PARALLELIZE

						if(f_value != 0) {
							uint32_t n_color = (fabs(r_v_dx(n_row)) > 1e-5f || fabs(r_v_dx(n_col)) > 1e-5f)? n_nonzero_new : n_nonzero;
					
#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
							float f_x = float(n_row * f_scale);
							float f_y = float(n_col * f_scale); // transposed
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
							float f_x = float(n_col * f_scale);
							float f_y = float(n_row * f_scale);
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
							p_bitmap->PutPixel_AA(f_x, f_y, n_color);
							/*if(b_symmetric && f_x != f_y)
								p_bitmap->PutPixel_AA(f_y, f_x, n_color);*/
						}
						// draw a square into the bitmap
					}
				}
			}
		}

		if(b_symmetric) {
			uint32_t *p_buff = p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					uint32_t *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					uint32_t *p_lo = p_buff + (y + x * w);
					*p_up = std::min(*p_up, *p_lo); // choose the darker color
					*p_lo = *p_up; // duplicate the result to the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices
	}
	// iterate the matrix, draw small tiles

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if((m != n || !b_symmetric) && !p_bitmap->Transpose()) {
		p_bitmap->Delete();
		return false;
	}
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

	bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_bitmap, false);

	p_bitmap->Delete();

	return b_result;
}

void CBAOptimizer::Show_Stats(bool b_calculate_eigenvalues) const
{
	//m_p_optimizer->r_Solver().Dump_ReprojectionErrors(); // debug

	std::vector<int> visible_points(m_p_optimizer->r_System().r_Vertex_Pool().n_Size(), 0); // 0 = camera / visible, 1 = partially visible, 2 = invisible
	double f_AVG_reprojection_err = 0;
	double f_RMSE_reprojection_err = 0;
	double f_AVG_reprojection_err_visible = 0;
	double f_RMSE_reprojection_err_visible = 0;
	const CBAOptimizerCore::CSystemType::_TyEdgeMultiPool &edge_pool = m_p_optimizer->r_System().r_Edge_Pool();
	size_t n_visible_obs_num = 0;
#ifdef SIM3_USE_ROBUST_EDGES
	double f_AVG_reprojection_err_robust = 0;
	double f_RMSE_reprojection_err_robust = 0;
	double f_robust_weight_sum = 0;
#endif // SIM3_USE_ROBUST_EDGES
	if(!edge_pool.b_Empty()) {
		for(size_t i = 0, n = edge_pool.n_Size(); i < n; ++ i) {
			bool b_visible;
			double f_weight;
#if 1
			double f_reprojection_err;
			edge_pool.For_Each(i, i + 1, CCalcRepErr(f_reprojection_err, f_weight, b_visible)); // need to derive the edge type
			size_t n_point_id = edge_pool[i].n_Vertex_Id(0); // landmark id // use facade
#else
			double f_reprojection_err = edge_pool.r_At<_TyObservation>(i).f_Reprojection_Error(b_visible); // know the edge type
			Eigen::Vector1d r; r(0) = f_reprojection_err; // can't initialize directly, the constructor is ambiguous
			f_weight = edge_pool.r_At<_TyObservation>(i).f_RobustWeight(r);
			size_t n_point_id = edge_pool.r_At<_TyObservation>(i).n_Vertex_Id(0); // landmark id // know the edge type
#endif
			visible_points[n_point_id] |= (b_visible)? 1 : 2; // need to combine
			f_AVG_reprojection_err += f_reprojection_err;
			f_RMSE_reprojection_err += f_reprojection_err * f_reprojection_err;
#ifdef SIM3_USE_ROBUST_EDGES
			f_AVG_reprojection_err_robust += f_reprojection_err * f_weight;
			f_RMSE_reprojection_err_robust += f_reprojection_err * f_reprojection_err * f_weight;
			f_robust_weight_sum += f_weight;
#endif // SIM3_USE_ROBUST_EDGES
			if(b_visible) {
				++ n_visible_obs_num;
				f_AVG_reprojection_err_visible += f_reprojection_err;
				f_RMSE_reprojection_err_visible += f_reprojection_err * f_reprojection_err;
			}
		}
		f_AVG_reprojection_err /= edge_pool.n_Size();
		f_RMSE_reprojection_err /= edge_pool.n_Size();
		f_RMSE_reprojection_err = sqrt(f_RMSE_reprojection_err);
#ifdef SIM3_USE_ROBUST_EDGES
		f_AVG_reprojection_err_robust /= f_robust_weight_sum;
		f_RMSE_reprojection_err_robust /= f_robust_weight_sum;
		f_RMSE_reprojection_err_robust = sqrt(f_RMSE_reprojection_err_robust);
#endif // SIM3_USE_ROBUST_EDGES
		f_AVG_reprojection_err_visible /= n_visible_obs_num;
		f_RMSE_reprojection_err_visible /= n_visible_obs_num;
		f_RMSE_reprojection_err_visible = sqrt(f_RMSE_reprojection_err_visible);
	}

	size_t n_fully_visible = 0;
	size_t n_fully_invisible = 0;
	size_t n_partly_invisible = 0;
	FILE *p_fw = fopen("visibility_info.txt", "w");
	for(size_t i = 0, n = visible_points.size(); i < n; ++ i) {
		if(visible_points[i] == 0) {
			// a camera, do nothing
		} else if(visible_points[i] == 1) {
			visible_points[i] = 0; // fully visible
			++ n_fully_visible;
		} else if(visible_points[i] == 2)
			++ n_fully_invisible; // fully invisible
		else if(visible_points[i] == 3) {
			visible_points[i] = 1; // visible from some cameras, invisible from the others
			++ n_partly_invisible;
		}

		if(p_fw) {
			double f_value = visible_points[i] * 10;
			size_t n_dim = m_p_optimizer->r_System().r_Vertex_Pool()[i].n_Dimension();
			for(size_t j = 0; j < n_dim; ++ j)
				fprintf(p_fw, (j + 1 < n_dim)? "%f " : "%f\n", f_value);
		}
		// write "fake marginals" that can be displayed by the graph viewer as false colours
	}
	if(p_fw)
		fclose(p_fw);
	// transform the visibility flags

	printf("reprojection error avg: %g px, RMSE: %g px (" PRIsize " observations)\n",
		f_AVG_reprojection_err, f_RMSE_reprojection_err, edge_pool.n_Size());
#ifdef SIM3_USE_ROBUST_EDGES
	printf("reprojection error avg: %g px, RMSE: %g px (robustified, " PRIsize " observations, %.2f weight)\n",
		f_AVG_reprojection_err_robust, f_RMSE_reprojection_err_robust, edge_pool.n_Size(), f_robust_weight_sum);
#endif // SIM3_USE_ROBUST_EDGES
	printf("reprojection error avg: %g px, RMSE: %g px (visible points only, " PRIsize " observations)\n",
		f_AVG_reprojection_err_visible, f_RMSE_reprojection_err_visible, n_visible_obs_num);
	printf("there were " PRIsize " points fully visible, " PRIsize
		" points partially visible and " PRIsize " points fully invisible\n",
		n_fully_visible, n_partly_invisible, n_fully_invisible);
	// print also the visibility info

	//m_p_optimizer->r_Solver().Dump_ReprojectionErrors(); // debug

	m_p_optimizer->r_Solver().Dump();
	double f_error = m_p_optimizer->r_Solver().f_Chi_Squared_Error_Denorm();
	printf("denormalized chi2 error: %.2f\n", f_error);
	
	//m_p_optimizer->r_Solver().Dump_ReprojectionErrors(); // debug

	if(!m_p_optimizer->r_Solver().r_MarginalCovariance().r_SparseMatrix().b_Empty())
		m_p_optimizer->r_Solver().r_MarginalCovariance().Dump_Diagonal();
	// also save marginals if enabled

	/*if(m_p_optimizer->r_Solver().r_Lambda().Save_MatrixMarket("system.mtx", "system.bla"))
		printf("system matrix saved to lambda.mtx and lambda.bla\n");
	else
		fprintf(stderr, "error: failed to save the matrix\n");*/
	// save the matrix for inspecting the sparsity patterns

	/*try {
		Eigen::MatrixXd lambda;
		m_p_optimizer->r_Solver().r_Lambda().Convert_to_Dense(lambda);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(lambda, 0);
		double f_max_sing = svd.singularValues()(0);
		double f_min_sing = svd.singularValues()(lambda.cols() - 1);
		printf("condition number of the information matrix is %.10g (min SV %g, max SV %g)\n",
			f_max_sing / f_min_sing, f_min_sing, f_max_sing);
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: failed to calculate the condition number with SVD\n");
	}*/ // this is very slow even for small problems, runs out of memory for large
	{
		Eigen::VectorXd diag;
		m_p_optimizer->r_Solver().r_Lambda().Get_Diagonal(diag, true);
		double f_max_diag = diag.array().abs().maxCoeff();
		double f_min_diag = diag.array().abs().minCoeff();
		printf("approximate condition number of the information matrix is %.10g (min diag %g, max diag %g)\n",
			f_max_diag / f_min_diag, f_min_diag, f_max_diag);
	}
	cs *p_bs = m_p_optimizer->r_Solver().r_Lambda().p_BlockStructure_to_Sparse();
	/*m_p_optimizer->r_Solver().r_Lambda().Rasterize("lambda_5.tga", 5);
	m_p_optimizer->r_Solver().r_Lambda().Rasterize("lambda_3.tga", 3);
	m_p_optimizer->r_Solver().r_Lambda().Rasterize("lambda_2.tga", 2);*/ // not useful after all
	CDebug::Dump_SparseMatrix_Subsample("lambda_struct.tga", p_bs, 0, 640, true);
	CDebug::Dump_SparseMatrix_Subsample_AA("lambda_struct_aa.tga", p_bs, 0, 640, true);
	Dump_SparseMatrix_SubsampleDx("lambda_struct_dx.tga", p_bs, m_p_optimizer->r_Solver().r_v_LastDx(), 640, true);
	Dump_SparseMatrix_SubsampleDx_AA("lambda_struct_dx_aa.tga", p_bs, m_p_optimizer->r_Solver().r_v_LastDx(), 640, true);
	cs_spfree(p_bs);
	//m_p_optimizer->r_Solver().r_Lambda().Save_MatrixMarket("lambda.mtx", 0,
	//	"symmetric block matrix", "matrix coordinate real symmetric", 'U'); // save as symmetric, keep in mind that only the upper triangular is stored
	// save some space

	if(!b_calculate_eigenvalues)
		return; // no eigs
	std::vector<double> sm_eig, lg_eig;
	try {
		printf("debug: trying to calculate eigenvalues\n");

		cs *p_matrix_csc;
		{
			const CUberBlockMatrix &r_lambda = m_p_optimizer->r_Solver().r_Lambda();
			/*cs *p_matrix_upd = r_lambda.p_Convert_to_Sparse();
			cs *p_matrix_full = cs_spalloc(p_matrix_upd->m, p_matrix_upd->n, p_matrix_upd->p[p_matrix_upd->n] * 2, 1, 1); // alloc triplet
			for(size_t i = 0, n = p_matrix_upd->n; i < n; ++ i) {
				for(size_t p = p_matrix_upd->p[i], e = p_matrix_upd->p[i + 1]; p < e; ++ p) {
					size_t j = p_matrix_upd->i[p];

					// have an element at i, j
					if(i == j) // diag elem
						cs_entry(p_matrix_full, i, j, p_matrix_upd->x[p]);
					else if(i > j) { // upper triangular elem
						cs_entry(p_matrix_full, i, j, p_matrix_upd->x[p]); // above
						cs_entry(p_matrix_full, j, i, p_matrix_upd->x[p]); // and below
					}
				}
			}
			cs_spfree(p_matrix_upd);
			p_matrix_csc = cs_compress(p_matrix_full);
			cs_spfree(p_matrix_full;*/

			CUberBlockMatrix full_lambda;
			full_lambda.SelfAdjointViewOf(r_lambda, true);
			if(!(p_matrix_csc = full_lambda.p_Convert_to_Sparse()))
				throw std::bad_alloc();
		}
		// form a full matrix out of upper half only

		int n_eig_num = int(std::min(csi(50), p_matrix_csc->n - 1));
		lg_eig = SpSym_Eigenvalues(p_matrix_csc, n_eig_num, false); // try to recover the large ones first, it is an easier task (the smaller might throw)
		sm_eig = SpSym_Eigenvalues(p_matrix_csc, n_eig_num, true);
		if(sm_eig.empty()) {
			fprintf(stderr, "warning: inverse / shift mode eigensolver found no"
				" eigenvalues, trying to reduce precision\n");
			sm_eig = SpSym_Eigenvalues(p_matrix_csc, n_eig_num, true, 10000, 1e-5);
		}
		cs_spfree(p_matrix_csc);

		_ASSERTE(sm_eig.empty() || fabs(sm_eig.front()) >= fabs(sm_eig.back()));
		_ASSERTE(lg_eig.empty() || fabs(lg_eig.front()) >= fabs(lg_eig.back()));
		double f_min_eig = (sm_eig.empty())? 0 : sm_eig.back();
		double f_max_eig = (lg_eig.empty())? 0 : lg_eig.front();
		if(sm_eig.empty() || lg_eig.empty())
			fprintf(stderr, "error: failed to calculate all the eigenvalues\n");
		printf("condition number of the information matrix is %.10g (min eig %g, max eig %g)\n",
			f_max_eig / f_min_eig, f_min_eig, f_max_eig);
	} catch(std::exception &r_exc) {
		fprintf(stderr, "error: failed to calculate the condition number (%s; %s)\n",
			(sm_eig.empty() && lg_eig.empty())? "recovered no eigenvalues" :
			((sm_eig.empty() && !lg_eig.empty())? "recovered only large eigenvalues" :
			((!sm_eig.empty() && lg_eig.empty())? "recovered only small eigenvalues" :
			"recovered large and small eigenvalues")), r_exc.what());
	}
}

bool CBAOptimizer::Dump_State(const char *p_s_filename) const
{
	return m_p_optimizer->r_System().Dump(p_s_filename);
}

/**
 *	@brief saves SE(3) state file from the optimizer system
 */
class CSE3StatePrimitiveCallback {
protected:
	size_t m_n_id; /**< @brief zero-based id of the last written vertex @note This is needed because the vertices do not store their id, but they will be enumerated in the order they go. */
	FILE *m_p_fw; /**< @brief output file */
	const CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */
	bool m_b_ignore_landmarks; /**< @brief landmarks ignore flag */

public:
	inline CSE3StatePrimitiveCallback(FILE *p_fw, const CBAOptimizer &r_optimizer, bool b_ignore_landmarks = false)
		:m_p_fw(p_fw), m_n_id(size_t(-1)), m_r_optimizer(r_optimizer), m_b_ignore_landmarks(b_ignore_landmarks)
	{}

	void operator ()(const _TyLandmark &r_vertex_invd)
	{
		if(m_b_ignore_landmarks)
			return;
		Eigen::Vector3d v_state = m_r_optimizer.v_XYZVertex_Global(++ m_n_id);
		fprintf(m_p_fw, "%.15f %.15f %.15f\n", v_state(0), v_state(1), v_state(2));
	}

	void operator ()(const CVertexCamSim3 &r_vertex_cam)
	{
		++ m_n_id; // !!

		const Eigen::Matrix<double, 7, 1> &r_v_state = r_vertex_cam.r_v_State();
		const Eigen::Matrix<double, 5, 1> &r_v_intrinsics = r_vertex_cam.v_Intrinsics();
		// get state and intrinsics

		CSim3Jacobians::TSim3 t_pose(r_v_state, CSim3Jacobians::TSim3::from_sim3_vector);
		t_pose.Invert(); // the SE(3) internal representation uses inverse camera poses. these do not get saved as inverse to the graph file, but they do get dumped like that in the internal format
		/*Eigen::Quaterniond t_rotation = t_pose.t_rotation;
		Eigen::Vector3d v_rotation;
		C3DJacobians::Quat_to_AxisAngle(t_rotation, v_rotation);
		Eigen::Vector3d v_translation = t_pose.v_translation / t_pose.f_scale;*/ // fixme - is this correct? // division seems to be correct
		Eigen::Vector7d v_trs = t_pose.v_tRs();
		v_trs.head<3>() /= v_trs(6); // fixme - is this correct? // division seems to be correct
		// convert to SE(3), mind the scale this time

		fprintf(m_p_fw, "%.15f %.15f %.15f %.15g %.15g %.15g\n", // use %g for the rotation, as it will have less than pi numbers
		//	v_translation(0), v_translation(1), v_translation(2), v_rotation(0), v_rotation(1), v_rotation(2));
			v_trs(0), v_trs(1), v_trs(2), v_trs(3), v_trs(4), v_trs(5));
	}
};

bool CBAOptimizer::Dump_Marginals(const char *p_s_filename) const
{
	if(!m_p_optimizer->r_Solver().r_MarginalCovariance().r_SparseMatrix().b_Empty())
		return m_p_optimizer->r_Solver().r_MarginalCovariance().Dump_Diagonal(p_s_filename);
	return true;
}

bool CBAOptimizer::Dump_State_SE3(const char *p_s_filename) const
{
	const CBAOptimizerCore::CSystemType::_TyVertexMultiPool &vertex_pool = m_p_optimizer->r_System().r_Vertex_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

	CSE3StatePrimitiveCallback state_writer(p_fw, *this);
	vertex_pool.For_Each(state_writer);

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}

	return !fclose(p_fw);
}

bool CBAOptimizer::Dump_Poses_SE3(const char *p_s_filename) const
{
	const CBAOptimizerCore::CSystemType::_TyVertexMultiPool &vertex_pool = m_p_optimizer->r_System().r_Vertex_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

	CSE3StatePrimitiveCallback state_writer(p_fw, *this, true);
	vertex_pool.For_Each(state_writer);

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}

	return !fclose(p_fw);
}

/**
 *	@brief saves a graph file from the optimizer system
 */
class CGraphPrimitiveCallback {
protected:
	size_t m_n_id; /**< @brief zero-based id of the last written vertex @note This is needed because the vertices do not store their id, but they will be enumerated in the order they go. */
	FILE *m_p_fw; /**< @brief output file */
	const CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */

public:
	inline CGraphPrimitiveCallback(FILE *p_fw, const CBAOptimizer &r_optimizer)
		:m_p_fw(p_fw), m_n_id(size_t(-1)), m_r_optimizer(r_optimizer)
	{}

#if 0 // SE(3) types
	void operator ()(const CVertexXYZ &r_vertex_xyz)
	{
		const Eigen::Vector3d &r_v_state = r_vertex_xyz.r_v_State();
		fprintf(m_p_fw, "VERTEX_XYZ " PRIsize " %.15f %.15f %.15f\n", ++ m_n_id, r_v_state(0), r_v_state(1), r_v_state(2));
	}

	void operator ()(const CVertexCam &r_vertex_cam)
	{
		const Eigen::Matrix<double, 6, 1> &r_v_state = r_vertex_cam.r_v_State();
		const Eigen::Matrix<double, 5, 1> &r_v_intrinsics = r_vertex_cam.v_Intrinsics();
		// get state and intrinsics

		Eigen::Quaterniond t_rotation;
		C3DJacobians::AxisAngle_to_Quat(r_v_state.tail<3>(), t_rotation);
		t_rotation = t_rotation.conjugate(); // R^{-1}
		Eigen::Vector3d v_translation = -t_rotation._transformVector(r_v_state.head<3>()); // -R^{-1} * t
		// the convention in the graph file is inverse opposite to the internal state

		fprintf(m_p_fw, "VERTEX_CAM " PRIsize " %.15f %.15f %.15f %.15g %.15g %.15g %.15g %.15f %.15f %.15f %.15f %.15f\n", // use %g for the quat, as it will have less than zero numbers
			++ m_n_id, v_translation(0), v_translation(1), v_translation(2), t_rotation.x(), t_rotation.y(),
			t_rotation.z(), t_rotation.w(), r_v_intrinsics(0), r_v_intrinsics(1), r_v_intrinsics(2),
			r_v_intrinsics(3), r_v_intrinsics(4));
	}

	void operator ()(const CEdgeP2C3D &r_t_edge)
	{
		size_t n_point_id = r_t_edge.n_Vertex_Id(1);
		size_t n_camera_id = r_t_edge.n_Vertex_Id(0); // someone swapped this in the edge type

		const CEdgeP2C3D::_TyVectorAlign &r_v_measurement = r_t_edge.v_Measurement();
		const CEdgeP2C3D::_TyMatrixAlign &r_t_information = r_t_edge.t_Sigma_Inv();

		fprintf(m_p_fw, "EDGE_PROJECT_P2MC " PRIsize " " PRIsize " %f %f %g %g %g\n", // these do not need to be very precise (x, y detected by SIFT with barely any fractional precision, covariance typically large integer numbers)
			n_point_id, n_camera_id, r_v_measurement(0), r_v_measurement(1),
			r_t_information(0, 0), r_t_information(0, 1), r_t_information(1, 1)); // only the upper triangular part is stored
	}
#endif // 0

	void operator ()(const _TyLandmark &r_vertex_invd)
	{
		Eigen::Vector3d v_state = m_r_optimizer.v_XYZVertex_Global(++ m_n_id);
		fprintf(m_p_fw, "VERTEX_XYZ " PRIsize " %.15f %.15f %.15f\n", m_n_id, v_state(0), v_state(1), v_state(2));
	}

	void operator ()(const CVertexCamSim3 &r_vertex_cam)
	{
		const Eigen::Matrix<double, 7, 1> &r_v_state = r_vertex_cam.r_v_State();
		const Eigen::Matrix<double, 5, 1> &r_v_intrinsics = r_vertex_cam.v_Intrinsics();
		// get state and intrinsics

		CSim3Jacobians::TSim3 t_pose(r_v_state, CSim3Jacobians::TSim3::from_sim3_vector);
		Eigen::Quaterniond t_rotation = t_pose.t_rotation;
		Eigen::Vector3d v_translation = t_pose.v_translation / t_pose.f_scale; // fixme - is this correct?
		// convert to SE(3), mind the scale this time

		fprintf(m_p_fw, "VERTEX_CAM " PRIsize " %.15f %.15f %.15f %.15g %.15g %.15g %.15g %.15f %.15f %.15f %.15f %.15f\n", // use %g for the quat, as it will have less than zero numbers
			++ m_n_id, v_translation(0), v_translation(1), v_translation(2), t_rotation.x(), t_rotation.y(),
			t_rotation.z(), t_rotation.w(), r_v_intrinsics(0), r_v_intrinsics(1), r_v_intrinsics(2),
			r_v_intrinsics(3), r_v_intrinsics(4));
	}

#ifdef LANDMARKS_LOCAL
	void operator ()(const _TyObservationS &r_t_edge)
	{
		size_t n_point_id = r_t_edge.n_Vertex_Id(0); // landmark
		size_t n_observing_camera_id = r_t_edge.n_ObservingCamera_Id(); // this is not really a vertex of that edge, since it is unary

		const CEdgeP2CSim3G::_TyVectorAlign &r_v_measurement = r_t_edge.v_Measurement();
		const CEdgeP2CSim3G::_TyMatrixAlign &r_t_information = r_t_edge.t_Sigma_Inv();

		fprintf(m_p_fw, "EDGE_PROJECT_P2MC " PRIsize " " PRIsize " %f %f %g %g %g\n", // these do not need to be very precise (x, y detected by SIFT with barely any fractional precision, covariance typically large integer numbers)
			n_point_id, n_observing_camera_id, r_v_measurement(0), r_v_measurement(1),
			r_t_information(0, 0), r_t_information(0, 1), r_t_information(1, 1)); // only the upper triangular part is stored
	}

	void operator ()(const _TyObservationO &r_t_edge)
	{
		size_t n_point_id = r_t_edge.n_Vertex_Id(0); // landmark
		size_t n_observing_camera_id = r_t_edge.n_Vertex_Id(1); // observing camera
		//size_t n_owner_camera_id = r_t_edge.n_Vertex_Id(2); // owner camera (not needed)

		const CEdgeP2CSim3G::_TyVectorAlign &r_v_measurement = r_t_edge.v_Measurement();
		const CEdgeP2CSim3G::_TyMatrixAlign &r_t_information = r_t_edge.t_Sigma_Inv();

		fprintf(m_p_fw, "EDGE_PROJECT_P2MC " PRIsize " " PRIsize " %f %f %g %g %g\n", // these do not need to be very precise (x, y detected by SIFT with barely any fractional precision, covariance typically large integer numbers)
			n_point_id, n_observing_camera_id, r_v_measurement(0), r_v_measurement(1),
			r_t_information(0, 0), r_t_information(0, 1), r_t_information(1, 1)); // only the upper triangular part is stored
	}
#else // LANDMARKS_LOCAL
	void operator ()(const _TyObservation &r_t_edge)
	{
		size_t n_point_id = r_t_edge.n_Vertex_Id(0);
		size_t n_camera_id = r_t_edge.n_Vertex_Id(1); // not swapped

		const CEdgeP2CSim3G::_TyVectorAlign &r_v_measurement = r_t_edge.v_Measurement();
		const CEdgeP2CSim3G::_TyMatrixAlign &r_t_information = r_t_edge.t_Sigma_Inv();

		fprintf(m_p_fw, "EDGE_PROJECT_P2MC " PRIsize " " PRIsize " %f %f %g %g %g\n", // these do not need to be very precise (x, y detected by SIFT with barely any fractional precision, covariance typically large integer numbers)
			n_point_id, n_camera_id, r_v_measurement(0), r_v_measurement(1),
			r_t_information(0, 0), r_t_information(0, 1), r_t_information(1, 1)); // only the upper triangular part is stored
	}
#endif // LANDMARKS_LOCAL

	void operator ()(const CEdgePoseCamSim3 &r_t_edge)
	{
		size_t n_point0_id = r_t_edge.n_Vertex_Id(0);
		size_t n_point1_id = r_t_edge.n_Vertex_Id(1);

		fprintf(m_p_fw, "# hard UF edge " PRIsize " " PRIsize "\n", // these do not need to be very precise (x, y detected by SIFT with barely any fractional precision, covariance typically large integer numbers)
			n_point0_id, n_point1_id); // only the upper triangular part is stored
	}
};

bool CBAOptimizer::Dump_Graph_SE3(const char *p_s_filename) const
{
	const CBAOptimizerCore::CSystemType::_TyVertexMultiPool &vertex_pool = m_p_optimizer->r_System().r_Vertex_Pool();
	const CBAOptimizerCore::CSystemType::_TyEdgeMultiPool &edge_pool = m_p_optimizer->r_System().r_Edge_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

	CGraphPrimitiveCallback graph_writer(p_fw, *this);
	vertex_pool.For_Each(graph_writer);
	edge_pool.For_Each(graph_writer);

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}

	return !fclose(p_fw);
}

/*
 *								=== ~CBAOptimizer ===
 */

/*
 *								=== "C" wrappers ===
 */

extern "C" {

#ifndef ENABLE_BA_OPTIMIZER_C_INTERFACE // just make sure that optimizer_t is not declared twice if ENABLE_BA_OPTIMIZER_C_INTERFACE is defined globally for all the files
typedef struct _optimizer_t *optimizer_t;
#endif // !ENABLE_BA_OPTIMIZER_C_INTERFACE

optimizer_t New_Optimizer(int b_verbose, int b_use_schur)
{
	return (optimizer_t)(new(std::nothrow) CBAOptimizer(b_verbose != 0, b_use_schur != 0)); // don't throw, just return 0 if not enough memory
}

void Free_Optimizer(optimizer_t h_optimizer)
{
	if(h_optimizer)
		delete reinterpret_cast<CBAOptimizer*>(h_optimizer);
}

// note that the below function can throw std::bad_alloc or std::runtime_error which is not currently
// handled (for simplicity) and if called from "C", these will crash the application. one would have to
// surround the code in each of those functions with try-catch block and return some error code in case
// an error occured (or use setjmp()/longjpm() or whatever is used in "C" these days to handle errors).

size_t n_Vertex_Num(optimizer_t h_optimizer)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return r_opt.n_Vertex_Num();
}

double *p_Vertex_State(optimizer_t h_optimizer, size_t n_vertex)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return &r_opt.r_Vertex_State(n_vertex)(0);
}

int n_Vertex_Dimension(optimizer_t h_optimizer, size_t n_vertex)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return int(r_opt.r_Vertex_State(n_vertex).rows());
}

void Delay_Optimization(optimizer_t h_optimizer)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Delay_Optimization();
}

void Enable_Optimization(optimizer_t h_optimizer)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Enable_Optimization();
}

void Optimize(optimizer_t h_optimizer, size_t n_max_iteration_num, double f_min_dx_norm)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Optimize(n_max_iteration_num, f_min_dx_norm);
}

int Dump_State(optimizer_t h_optimizer, const char *p_s_filename)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return (r_opt.Dump_State(p_s_filename))? 1 : 0;
}

int Dump_State_SE3(optimizer_t h_optimizer, const char *p_s_filename)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return (r_opt.Dump_State_SE3(p_s_filename))? 1 : 0;
}

int Dump_Graph_SE3(optimizer_t h_optimizer, const char *p_s_filename)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return (r_opt.Dump_Graph_SE3(p_s_filename))? 1 : 0;
}

void Add_CamSim3Vertex(optimizer_t h_optimizer, size_t n_vertex_id, const double p_cam_state[12])
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_CamSim3Vertex(n_vertex_id, Eigen::Map<const Eigen::Matrix<double, 12, 1>, Eigen::DontAlign>(p_cam_state));
}

void Add_InvDepthVertex_Global(optimizer_t h_optimizer, size_t n_vertex_id, const double p_position[3])
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_InvDepthVertex_Global(n_vertex_id, Eigen::Map<const Eigen::Vector3d, Eigen::DontAlign>(p_position));
}

void Add_InvDepthVertex_Local(optimizer_t h_optimizer, size_t n_vertex_id, const double p_position[3], size_t n_observing_camera_id)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_InvDepthVertex_Local(n_vertex_id, Eigen::Map<const Eigen::Vector3d, Eigen::DontAlign>(p_position), n_observing_camera_id);
}

void Add_P2CSim3GEdge(optimizer_t h_optimizer, size_t n_landmark_vertex_id, size_t n_cam_vertex_id,
	const double p_observation[2], const double p_information[4])
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_P2CSim3GEdge(n_landmark_vertex_id, n_cam_vertex_id,
		Eigen::Map<const Eigen::Vector2d, Eigen::DontAlign>(p_observation),
		Eigen::Map<const Eigen::Matrix2d, Eigen::DontAlign>(p_information));
}

};

/*
 *								=== ~"C" wrappers ===
 */
