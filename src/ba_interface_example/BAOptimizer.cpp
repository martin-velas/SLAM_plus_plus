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
 *	@file src/ba_interface_example/BAOptimizer.cpp
 *	@brief SLAM++ bundle adjustment optimizer interface
 *	@date 2014-03-20
 *	@author -tHE SWINe-
 */

#include <string.h>
#include <stdio.h>
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#include "slam/LinearSolver_UberBlock.h"
//#include "slam/LinearSolver_CholMod.h" // not used
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
//#include "slam/Timer.h" // not used
#include "slam/BA_Types.h"
//#include "slam/Marginals.h" // included from nonlinear solver, if supported
#include "slam/NonlinearSolver_Lambda_LM.h"
//#include "slam/BASolverBase.h" // included from BA_Types.h
#include "ba_interface_example/BAOptimizer.h"

/*
 *								=== CDelayedOptimizationCaller ===
 */

/**
 *	@brief utility class for enabling or disabling optimization in nonlinear solvers
 *
 *	@tparam CSolverType is nonlinear solver type
 *	@tparam b_can_delay is delayed optimization support flag
 */
template <class CSolverType, const bool b_can_delay>
class CDelayedOptimizationCaller {
public:
	/**
	 *	@brief delays optimization upon Incremental_Step()
	 *	@param[in] r_solver is solver to delay the optimization at (unused)
	 */
	static void Delay(CSolverType &UNUSED(r_solver))
	{}

	/**
	 *	@brief enables optimization upon Incremental_Step()
	 *
	 *	This is default behavior. In case it was disabled (by Delay_Optimization()),
	 *	and optimization is required, this will also run the optimization.
	 *
	 *	@param[in] r_solver is solver to enable the optimization at (unused)
	 */
	static void Enable(CSolverType &UNUSED(r_solver))
	{}
};

/**
 *	@brief utility class for enabling or disabling optimization in nonlinear solvers
 *		(specialization for solvers which support enabling or disabling optimization)
 *
 *	@tparam CSolverType is nonlinear solver type
 */
template <class CSolverType>
class CDelayedOptimizationCaller<CSolverType, true> {
public:
	/**
	 *	@brief delays optimization upon Incremental_Step()
	 *	@param[in] r_solver is solver to delay the optimization at
	 */
	static void Delay(CSolverType &r_solver)
	{
		r_solver.Delay_Optimization();
	}

	/**
	 *	@brief enables optimization upon Incremental_Step()
	 *
	 *	This is default behavior. In case it was disabled (by Delay_Optimization()),
	 *	and optimization is required, this will also run the optimization.
	 *
	 *	@param[in] r_solver is solver to enable the optimization at
	 */
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

class CBAOptimizer::CBAOptimizerCore {
public:
	typedef MakeTypelist_Safe((CVertexCam, CVertexXYZ/*, CVertexSCam, CVertexIntrinsics*/)) TVertexTypelist;
	typedef MakeTypelist_Safe((CEdgeP2C3D/*, CEdgeP2SC3D, CEdgeP2CI3D*/)) TEdgeTypelist;
	typedef CFlatSystem<CBaseVertex, TVertexTypelist, CEdgeP2C3D/*CBaseEdge*/, TEdgeTypelist> CSystemType;
	// note that if BA_OPTIMIZER_WANT_DEEP_VERTEX_ACCESS is defined, any modifications here need to be carried also to BAOptimizer.h
	// also the "C" interface would need to be modified

	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
	typedef CNonlinearSolver_Lambda_LM<CSystemType, CLinearSolverType> CNonlinearSolverType;

protected:
	CSystemType m_system;
	CNonlinearSolverType m_solver;

public:
	inline CBAOptimizerCore(bool b_verbose = false, bool b_use_schur = true)
		:m_solver(m_system, TIncrementalSolveSetting(), TMarginalsComputationPolicy(), b_verbose, CLinearSolverType(), b_use_schur)
	{}

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

CBAOptimizer::CBAOptimizer(bool b_verbose, bool b_use_schur) // throw(srd::bad_alloc)
	:m_p_optimizer(new CBAOptimizerCore(b_verbose, b_use_schur))
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

void CBAOptimizer::Optimize(size_t n_max_iteration_num /*= 5*/, double f_min_dx_norm /*= .01*/) // throw(srd::bad_alloc, std::runtime_error)
{
	m_p_optimizer->r_Solver().Optimize(n_max_iteration_num, f_min_dx_norm);
}

void CBAOptimizer::Add_CamVertex(size_t n_vertex_id, const Eigen::Matrix<double, 11, 1> &v_cam_state) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Get_Vertex<CVertexCam>(n_vertex_id, v_cam_state);
}

void CBAOptimizer::Add_XYZVertex(size_t n_vertex_id, const Eigen::Vector3d &v_position) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Get_Vertex<CVertexXYZ>(n_vertex_id, v_position);
}

void CBAOptimizer::Add_P2C3DEdge(size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
	const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Add_Edge(CEdgeP2C3D(n_xyz_vertex_id, n_cam_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
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

void CBAOptimizer::Add_P2SC3DEdge(size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
	const Eigen::Vector3d &v_observation, const Eigen::Matrix3d &t_information) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Add_Edge(CEdgeP2SC3D(n_xyz_vertex_id, n_cam_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
}

void CBAOptimizer::Add_P2SC3DEdge(const CParserBase::TEdgeP2SC3D & edge_p2sc3dT) // throw(srd::bad_alloc)
{
	CBAOptimizer::Add_P2SC3DEdge(edge_p2sc3dT.m_n_node_0, edge_p2sc3dT.m_n_node_1, edge_p2sc3dT.m_v_delta,
		edge_p2sc3dT.m_t_inv_sigma);
}

void CBAOptimizer::Add_P2CI3DEdge(size_t n_xyz_vertex_id, size_t n_cam_vertex_id, size_t n_intrinsics_vertex_id,
	const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information) // throw(srd::bad_alloc)
{
	m_p_optimizer->r_System().r_Add_Edge(CEdgeP2CI3D(n_cam_vertex_id, n_xyz_vertex_id, n_intrinsics_vertex_id,
		v_observation, t_information, m_p_optimizer->r_System()));
}*/

void CBAOptimizer::Show_Stats() const
{
	m_p_optimizer->r_Solver().Dump();
}

bool CBAOptimizer::Dump_State(const char *p_s_filename) const
{
	return m_p_optimizer->r_System().Dump(p_s_filename);
}

/**
 *	@brief saves a graph file from the optimizer system
 */
class CGraphPrimitiveCallback {
protected:
	size_t m_n_id; /**< @brief zero-based id of the last written vertex @note This is needed because the vertices do not store their id, but they will be enumerated in the order they go. */
	FILE *m_p_fw; /**< @brief output file */

public:
	inline CGraphPrimitiveCallback(FILE *p_fw)
		:m_p_fw(p_fw), m_n_id(size_t(-1))
	{}

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
		t_rotation = t_rotation.conjugate(); // R^-1
		Eigen::Vector3d v_translation = -t_rotation._transformVector(r_v_state.head<3>()); // -R^-1 * t
		// the convention in the graph file is inverse opposite to the internal state

		fprintf(m_p_fw, "VERTEX_CAM " PRIsize " %.15f %.15f %.15f %.15g %.15g %.15g %.15g %.15f %.15f %.15f %.15f %.15f\n", // use %g for the quat, as it will have less than zero numbers
			++ m_n_id, v_translation(0), v_translation(1), v_translation(2), t_rotation.x(), t_rotation.y(),
			t_rotation.z(), t_rotation.w(), r_v_intrinsics(0), r_v_intrinsics(1), r_v_intrinsics(2),
			r_v_intrinsics(3), r_v_intrinsics(4));
	}

	void operator ()(const CEdgeP2C3D &r_t_edge)
	{
		size_t n_point_id = r_t_edge.n_Vertex_Id(1);
		size_t n_camera_id =  r_t_edge.n_Vertex_Id(0); // someone swapped this in the edge type

		const CEdgeP2C3D::_TyVectorAlign &r_v_measurement = r_t_edge.v_Measurement();
		const CEdgeP2C3D::_TyMatrixAlign &r_t_information = r_t_edge.t_Sigma_Inv();

		fprintf(m_p_fw, "EDGE_PROJECT_P2MC " PRIsize " " PRIsize " %f %f %g %g %g\n", // these do not need to be very precise (x, y detected by SIFT with barely any fractional precision, covariance typically large integer numbers)
			n_point_id, n_camera_id, r_v_measurement(0), r_v_measurement(1),
			r_t_information(0, 0), r_t_information(0, 1), r_t_information(1, 1)); // only the upper triangular part is stored
	}
};

bool CBAOptimizer::Dump_Graph(const char *p_s_filename) const
{
	const CBAOptimizerCore::CSystemType::_TyVertexMultiPool &vertex_pool =  m_p_optimizer->r_System().r_Vertex_Pool();
	const CBAOptimizerCore::CSystemType::_TyEdgeMultiPool &edge_pool =  m_p_optimizer->r_System().r_Edge_Pool();

	FILE *p_fw;
	if(!(p_fw = fopen(p_s_filename, "w")))
		return false;

	CGraphPrimitiveCallback graph_writer(p_fw);
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

int Dump_Graph(optimizer_t h_optimizer, const char *p_s_filename)
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	return (r_opt.Dump_Graph(p_s_filename))? 1 : 0;
}

void Add_CamVertex(optimizer_t h_optimizer, size_t n_vertex_id, const double p_cam_state[11])
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_CamVertex(n_vertex_id, Eigen::Map<const Eigen::Matrix<double, 11, 1>, Eigen::DontAlign>(p_cam_state));
}

void Add_XYZVertex(optimizer_t h_optimizer, size_t n_vertex_id, const double p_position[3])
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_XYZVertex(n_vertex_id, Eigen::Map<const Eigen::Vector3d, Eigen::DontAlign>(p_position));
}

void Add_P2C3DEdge(optimizer_t h_optimizer, size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
	const double p_observation[2], const double p_information[4])
{
	CBAOptimizer &r_opt = *reinterpret_cast<CBAOptimizer*>(h_optimizer);
	r_opt.Add_P2C3DEdge(n_xyz_vertex_id, n_cam_vertex_id,
		Eigen::Map<const Eigen::Vector2d, Eigen::DontAlign>(p_observation),
		Eigen::Map<const Eigen::Matrix2d, Eigen::DontAlign>(p_information));
}

};

/*
 *								=== ~"C" wrappers ===
 */
