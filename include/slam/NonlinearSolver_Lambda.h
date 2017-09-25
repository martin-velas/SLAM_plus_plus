/*
								+-----------------------------------+
								|                                   |
								| ***  Lambda nonlinear solver  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|     NonlinearSolver_Lambda.h      |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_LAMBDA_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_LAMBDA_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_Lambda.h
 *	@brief nonlinear blocky solver working above the lambda matrix
 *	@author -tHE SWINe-
 *	@date 2012-09-13
 */

#include "slam/LinearSolver_Schur.h"
#include "slam/OrderingMagic.h"
#include "slam/Marginals.h"
#include "slam/NonlinearSolver_Base.h"
#include "slam/NonlinearSolver_Lambda_Base.h"

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@brief nonlinear blocky solver working above the lambda matrix
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix
 *	@tparam CLambdaMatrixBlockSizes is list of block sizes in the information (Hessian) matrix
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes = typename CSystem::_TyJacobianMatrixBlockList,
	class CLambdaMatrixBlockSizes = typename CSystem::_TyHessianMatrixBlockList>
class CNonlinearSolver_Lambda : public nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, true> {
public:
	typedef CSystem _TySystem; /**< @brief system type */
	typedef CLinearSolver _TyLinearSolver; /**< @brief linear solver type */

	typedef typename CSystem::_TyBaseVertex _TyBaseVertex; /**< @brief the data type for storing vertices */
	typedef typename CSystem::_TyVertexTypelist _TyVertexTypelist; /**< @brief list of vertex types */
	typedef typename CSystem::_TyBaseEdge _TyBaseEdge; /**< @brief the data type for storing measurements */
	typedef typename CSystem::_TyEdgeTypelist _TyEdgeTypelist; /**< @brief list of edge types */

	typedef typename CSystem::_TyVertexMultiPool _TyVertexMultiPool; /**< @brief vertex multipool type */
	typedef typename CSystem::_TyEdgeMultiPool _TyEdgeMultiPool; /**< @brief edge multipool type */

	typedef typename CLinearSolver::_Tag _TySolverTag; /**< @brief linear solver tag */
	typedef CLinearSolverWrapper<_TyLinearSolver, _TySolverTag> _TyLinearSolverWrapper; /**< @brief wrapper for linear solvers (shields solver capability to solve blockwise) */

	typedef /*typename CUniqueTypelist<*/CAMatrixBlockSizes/*>::_TyResult*/ _TyAMatrixBlockSizes; /**< @brief possible block matrices, that can be found in A */
	typedef /*typename*/ CLambdaMatrixBlockSizes /*fbs_ut::CBlockMatrixTypesAfterPreMultiplyWithSelfTranspose<
		_TyAMatrixBlockSizes>::_TySizeList*/ _TyLambdaMatrixBlockSizes; /**< @brief possible block matrices, found in lambda and R */

	typedef typename CChooseType<lambda_utils::CLambdaOps<_TyLambdaMatrixBlockSizes>,
		lambda_utils::CLambdaOps2<_TyLambdaMatrixBlockSizes>, !base_iface::lambda_ReductionPlan_v2>::_TyResult _TyLambdaOps; /**< @brief implementation of operations for filling the lambda matrix */
	typedef typename _TyLambdaOps::_TyReductionPlan _TyReductionPlan; /**< @brief reduction plan implementation */

	/**
	 *	@brief solver interface properties, stored as enum (see also CSolverTraits)
	 */
	enum {
		solver_HasDump = true, /**< @brief timing statistics support flag */
		solver_HasChi2 = true, /**< @brief Chi2 error calculation support flag */
		solver_HasMarginals = true, /**< @brief marginal covariance support flag */
		solver_HasGaussNewton = true, /**< @brief Gauss-Newton support flag */
		solver_HasLevenberg = false, /**< @brief Levenberg-Marquardt support flag */
		solver_HasGradient = false, /**< @brief gradient-based linear solving support flag */
		solver_HasSchur = true, /**< @brief Schur complement support flag */
		solver_HasDelayedOptimization = false, /**< @brief delayed optimization support flag */
		solver_IsPreferredBatch = true, /**< @brief preferred batch solver flag */
		solver_IsPreferredIncremental = false, /**< @brief preferred incremental solver flag */
		solver_ExportsJacobian = false, /**< @brief interface for exporting jacobian system matrix flag */
		solver_ExportsHessian = true, /**< @brief interface for exporting hessian system matrix flag */
		solver_ExportsFactor = false /**< @brief interface for exporting factorized system matrix flag */
	};

protected:
	typedef nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, true> _TyBase; /**< @brief base solver utils type */

	CUberBlockMatrix m_lambda; /**< @brief the lambda matrix (built / updated incrementally) */
	_TyReductionPlan m_reduction_plan; /**< @brief lambda incremental reduction plan */
	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	size_t m_n_verts_in_lambda; /**< @brief number of vertices already in lambda */
	size_t m_n_edges_in_lambda; /**< @brief number of edges already in lambda */
	bool m_b_system_dirty; /**< @brief system updated without relinearization flag */
	// solver data

	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	double m_f_lambda_time; /**< @brief time spent updating lambda */
	double m_f_rhs_time; /**< @brief time spent gathering the right-hand-side vector */
	double m_f_chol_time; /**< @brief time spent in Choleski() section */
	double m_f_norm_time; /**< @brief time spent in norm calculation section */
	double m_f_vert_upd_time; /**< @brief time spent updating the system */
	double m_f_extra_chol_time; /**< @brief time spent in calculating extra Cholesky factorization for marginal covariances */
	double m_f_marginals_time; /**< @brief time spent in calculating marginal covariances (batch) */
	double m_f_incmarginals_time; /**< @brief time spent in calculating marginal covariances (update) */
	size_t m_n_incmarginals_num; /**< @brief number of times the marginals update ran instead of batch recalculation */
	// statistics

public:
	/**
	 *	@brief initializes the nonlinear solver
	 *
	 *	@param[in] r_system is the system to be optimized
	 *		(it is only referenced, not copied - must not be deleted)
	 *	@param[in] n_linear_solve_threshold is the step threshold
	 *		for linear solver to be called (0 = disable)
	 *	@param[in] n_nonlinear_solve_threshold is the step threshold
	 *		for nonlinear solver to be called (0 = disable)
	 *	@param[in] n_nonlinear_solve_max_iteration_num is maximal
	 *		number of iterations in nonlinear solver
	 *	@param[in] f_nonlinear_solve_error_threshold is error threshold
	 *		for the nonlinear solver
	 *	@param[in] b_verbose is verbosity flag
	 *	@param[in] linear_solver is linear solver instance
	 *	@param[in] b_use_schur is Schur complement trick flag
	 *
	 *	@deprecated This is deprecated version of the constructor, use constructor
	 *		with TIncrementalSolveSetting instead.
	 */
	CNonlinearSolver_Lambda(CSystem &r_system, size_t n_linear_solve_threshold,
		size_t n_nonlinear_solve_threshold, size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = true)
		:_TyBase(r_system, n_linear_solve_threshold,
		n_nonlinear_solve_threshold, n_nonlinear_solve_max_iteration_num,
		f_nonlinear_solve_error_threshold, b_verbose, linear_solver, b_use_schur),
		//this->m_r_system(r_system), this->m_linear_solver(linear_solver), this->m_schur_solver(linear_solver),
		m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
/*#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_linear_solve_threshold(n_linear_solve_threshold),
		m_n_nonlinear_solve_threshold(n_nonlinear_solve_threshold),
		m_n_nonlinear_solve_max_iteration_num(n_nonlinear_solve_max_iteration_num),
		m_f_nonlinear_solve_error_threshold(f_nonlinear_solve_error_threshold),
		this->m_b_verbose(b_verbose), this->m_b_use_schur(b_use_schur), m_n_real_step(0),*/
		m_b_system_dirty(false), m_n_iteration_num(0), m_f_lambda_time(0), m_f_rhs_time(0),
		m_f_chol_time(0), m_f_norm_time(0), m_f_vert_upd_time(0), //m_b_had_loop_closure(false),
		m_f_extra_chol_time(0), m_f_marginals_time(0), m_f_incmarginals_time(0), m_n_incmarginals_num(0)
	{
		//_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those
	}

	/**
	 *	@brief initializes the nonlinear solver
	 *
	 *	@param[in] r_system is the system to be optimized
	 *		(it is only referenced, not copied - must not be deleted)
	 *	@param[in] t_incremental_config is incremental solving configuration
	 *	@param[in] t_marginals_config is marginal covariance calculation configuration
	 *	@param[in] b_verbose is verbosity flag
	 *	@param[in] linear_solver is linear solver instance
	 *	@param[in] b_use_schur is Schur complement trick flag
	 */
	CNonlinearSolver_Lambda(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:_TyBase(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur),
		//this->m_r_system(r_system), this->m_linear_solver(linear_solver), this->m_schur_solver(linear_solver),
		m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
/*#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_linear_solve_threshold(t_incremental_config.t_linear_freq.n_period),
		m_n_nonlinear_solve_threshold(t_incremental_config.t_nonlinear_freq.n_period),
		m_n_nonlinear_solve_max_iteration_num(t_incremental_config.n_max_nonlinear_iteration_num),
		m_f_nonlinear_solve_error_threshold(t_incremental_config.f_nonlinear_error_thresh),
		this->m_b_verbose(b_verbose), this->m_b_use_schur(b_use_schur), m_n_real_step(0),*/
		m_b_system_dirty(false), m_n_iteration_num(0), m_f_lambda_time(0), m_f_rhs_time(0),
		m_f_chol_time(0), m_f_norm_time(0), m_f_vert_upd_time(0), /*m_b_had_loop_closure(false),
		m_t_marginals_config(t_marginals_config),*/ m_f_extra_chol_time(0), m_f_marginals_time(0),
		m_f_incmarginals_time(0), m_n_incmarginals_num(0)
	{
		//_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those
		//_ASSERTE(!t_marginals_config.b_calculate); // not supported at the moment // already partially supported

		if(t_marginals_config.b_calculate) {
			if(t_marginals_config.t_increment_freq.n_period != t_incremental_config.t_nonlinear_freq.n_period &&
			   t_marginals_config.t_increment_freq.n_period != t_incremental_config.t_linear_freq.n_period) {
				throw std::runtime_error("in CNonlinearSolver_Lambda, the marginals must"
					" be updated with the same frequency as the system");
			}
			// unfortunately, yes

			/*if(t_marginals_config.n_incremental_policy != (mpart_LastColumn | mpart_Diagonal)) {
				throw std::runtime_error("in CNonlinearSolver_Lambda, the marginals update"
					" policy must be mpart_LastColumn | mpart_Diagonal");
			}
			if(t_marginals_config.n_incremental_policy != t_marginals_config.n_relinearize_policy) {
				throw std::runtime_error("in CNonlinearSolver_Lambda, the marginals "
					" incremental and relinearize update policy must be the same");
			}*/ // these are now implemented
			if(t_marginals_config.n_cache_miss_policy != mpart_Nothing) {
				throw std::runtime_error("in CNonlinearSolver_Lambda, the marginals cache"
					" miss policy is not supported at the moment, sorry for inconvenience");
			}
			// nothing else is implemented so far
		}
	}

	/**
	 *	@brief gets the current system matrix (not necessarily up-to-date)
	 *	@return Returns const reference to the current system matrix.
	 */
	inline const CUberBlockMatrix &r_Lambda() const
	{
		return m_lambda;
	}

//	/**
//	 *	@brief gets marginal covariances
//	 *	@return Returns reference to the marginal covariances object.
//	 */
//	inline CMarginalCovariance &r_MarginalCovariance()
//	{
//		return this->m_marginals;
//	}
//
//	/**
//	 *	@brief gets marginal covariances
//	 *	@return Returns const reference to the marginal covariances object.
//	 */
//	inline const CMarginalCovariance &r_MarginalCovariance() const
//	{
//		return this->m_marginals;
//	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
		printf("solver spent %f seconds in parallelizable section (disparity %f seconds)\n",
			m_f_lambda_time + m_f_rhs_time, (f_total_time > 0)? f_total_time -
			(m_f_lambda_time + m_f_rhs_time + m_f_chol_time + m_f_norm_time +
			m_f_vert_upd_time + m_f_extra_chol_time + m_f_marginals_time + m_f_incmarginals_time) : 0);

		printf("out of which:\n");
		printf("\t   ,\\: %f\n", m_f_lambda_time);
		printf("\t  rhs: %f\n", m_f_rhs_time);
		if(this->m_t_marginals_config.b_calculate) {
			printf("solver spent %f seconds in marginals\n"
				"\t chol: %f\n"
				"\tmargs: %f\n"
				"\t incm: %f (ran " PRIsize " times)\n",
				m_f_extra_chol_time + m_f_marginals_time + m_f_incmarginals_time,
				m_f_extra_chol_time, m_f_marginals_time,
				m_f_incmarginals_time, m_n_incmarginals_num);
		}
		printf("solver spent %f seconds in serial section\n",
			m_f_chol_time + m_f_norm_time + m_f_vert_upd_time);
		printf("out of which:\n");
		printf("\t chol: %f\n", m_f_chol_time);
		printf("\t norm: %f\n", m_f_norm_time);
		printf("\tv-upd: %f\n", m_f_vert_upd_time);
	}

	/**
	 *	@brief writes system matrix for art purposes
	 *
	 *	@param[in] p_s_filename is output file name (.tga)
	 *	@param[in] n_scalar_size is size of one scalar, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Dump_SystemMatrix(const char *p_s_filename, int n_scalar_size = 5)
	{
		try {
			_TyLambdaOps::Extend_Lambda(this->m_r_system, m_reduction_plan, m_lambda,
				m_n_verts_in_lambda, m_n_edges_in_lambda); // recalculated all the jacobians inside Extend_Lambda()
			if(!m_b_system_dirty) {
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda,
					m_n_verts_in_lambda, m_n_edges_in_lambda); // calculate only for new edges // @todo - but how to mark affected vertices? // simple test if edge id is greater than m_n_edges_in_lambda, the vertex needs to be recalculated
			} else
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // calculate for entire system
			m_b_system_dirty = false;
			m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
			m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
			//const size_t n_variables_size = this->m_r_system.n_VertexElement_Num();
			_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
				m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
				m_lambda.n_Column_Num() == this->m_r_system.n_VertexElement_Num()); // lambda is square, blocks on either side = number of vertices
		} catch(std::bad_alloc&) {
			return false;
		}
		// need to have lambda

		return m_lambda.Rasterize_Symmetric(p_s_filename, n_scalar_size);
	}

	/**
	 *	@brief writes system matrix in matrix market for benchmarking purposes
	 *	@param[in] p_s_filename is output file name (.mtx)
	 *	@return Returns true on success, false on failure.
	 */
	bool Save_SystemMatrix_MM(const char *p_s_filename) const
	{
		char p_s_layout_file[256];
		strcpy(p_s_layout_file, p_s_filename);
		if(strrchr(p_s_layout_file, '.'))
			*(char*)strrchr(p_s_layout_file, '.') = 0;
		strcat(p_s_layout_file, ".bla");
		// only really required for landmark datasets

		return m_lambda.Save_MatrixMarket(p_s_filename, p_s_layout_file,
			"lambda matrix for SLAM problem", "matrix coordinate real symmetric", 'U');
	}

	/**
	 *	@brief incremental optimization function
	 *	@param[in] r_last_edge is the last edge that was added to the system
	 *	@note This function throws std::bad_alloc.
	 */
	void Incremental_Step(_TyBaseEdge &UNUSED(r_last_edge)) // throw(std::bad_alloc)
	{
		/*size_t n_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_had_loop_closure) {
			_ASSERTE(!this->m_r_system.r_Edge_Pool().b_Empty());
			typename _TyEdgeMultiPool::_TyConstBaseRef r_edge =
				this->m_r_system.r_Edge_Pool()[this->m_r_system.r_Edge_Pool().n_Size() - 1];
			// get a reference to the last edge interface

			if(r_edge.n_Vertex_Num() > 1) { // unary factors do not cause classical loop closures
				_ASSERTE(r_edge.n_Vertex_Id(0) != r_edge.n_Vertex_Id(1));
				size_t n_first_vertex = std::min(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1));
				m_b_had_loop_closure = (n_first_vertex < n_vertex_num - 2);
				_ASSERTE(m_b_had_loop_closure || std::max(r_edge.n_Vertex_Id(0),
					r_edge.n_Vertex_Id(1)) == n_vertex_num - 1);
			} else {
				size_t n_first_vertex = r_edge.n_Vertex_Id(0);
				m_b_had_loop_closure = (n_first_vertex < n_vertex_num - 1);
			}
			// todo - encapsulate this code, write code to support hyperedges as well, use it
		}
		// detect loop closures (otherwise the edges are initialized based on measurement and error would be zero)

		++ m_n_real_step;

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_vertex_num = n_vertex_num - m_n_last_optimized_vertex_num;
		// the optimization periods are counted in vertices, not in edges (should save work on 10k, 100k)
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		++ m_n_step;
		size_t n_new_edge_num = m_n_step;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES

		IncrementalPreOptDemo();

		bool b_new_vert = false, b_ran_opt = false;

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		if(m_n_nonlinear_solve_threshold && n_new_vertex_num >= m_n_nonlinear_solve_threshold) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		if(m_n_nonlinear_solve_threshold && m_n_step == m_n_nonlinear_solve_threshold) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure) {
				b_ran_opt = true;
				Optimize((m_b_had_loop_closure)?
					m_n_nonlinear_solve_max_iteration_num : 0,
					m_f_nonlinear_solve_error_threshold);
				m_b_had_loop_closure = false;
			}
			// nonlinear optimization

			b_new_vert = true;
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_n_linear_solve_threshold && n_new_vertex_num >= m_n_linear_solve_threshold) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_n_linear_solve_threshold && m_n_step % m_n_linear_solve_threshold == 0) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			_ASSERTE(!m_n_nonlinear_solve_threshold);
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			if(!m_n_nonlinear_solve_threshold)
				m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure) {
				b_ran_opt = true;
				m_b_had_loop_closure = false;
				Optimize(1, 0); // only if there was a loop (ignores possibly high residual after single step optimization)
			}
			// simple optimization

			b_new_vert = true;
		}*/

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size() -
			this->n_LastOptimized_Vertex_Num();
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_edge_num = this->n_New_Edge_Num() + 1;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES

		std::pair<bool, int> t_optimize = this->t_Incremental_Step(r_last_edge);
		bool b_new_vert = t_optimize.first, b_ran_opt = t_optimize.second != 0;

		IncrementalPreOptDemo();

		if(t_optimize.second == 2) {
			Optimize(this->m_t_incremental_config.n_max_nonlinear_iteration_num,
				this->m_t_incremental_config.f_nonlinear_error_thresh); // nonlinear optimization
		} else if(t_optimize.second == 1)
			Optimize(1, 0); // linear optimization
		// decide on incremental optimization

		if(b_new_vert && !b_ran_opt && this->m_t_marginals_config.b_calculate)
			Optimize(0, 0);
		// run optimization in order to calculate marginals after each vertex

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		IncrementalPostOptDemo(n_new_vertex_num);
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		IncrementalPostOptDemo(n_new_edge_num);
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	}

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
	CMatrixOrdering mord100;
	CUberBlockMatrix lambda_prev;
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

	/**
	 *	@brief norify the solver of linearization point update (e.g. change in robust
	 *		function parameters, external change to the current estimate, ...)
	 *
	 *	@param[in] n_first_changing_edge is zero-based index of the first edge being changed
	 *	@param[in] n_first_changing_vertex is zero-based index of the first vertex being changed
	 */
	void Notify_LinearizationChange(size_t UNUSED(n_first_changing_edge) = 0,
		size_t UNUSED(n_first_changing_vertex) = 0)
	{
		_ASSERTE(!n_first_changing_edge || n_first_changing_edge < this->m_r_system.r_Edge_Pool().n_Size());
		_ASSERTE(!n_first_changing_vertex || n_first_changing_vertex < this->m_r_system.r_Vertex_Pool().n_Size());
		// make sure those are valid indices

		m_b_system_dirty = true;
		// mark the system matrix as dirty, to force relinearization in the next step
	}

	/**
	 *	@brief final optimization function
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 */
	void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01) // throw(std::bad_alloc)
	{
		CTimerSampler timer(this->m_timer);

		const size_t n_variables_size = this->m_r_system.n_VertexElement_Num();
		const size_t n_measurements_size = this->m_r_system.n_EdgeElement_Num();
		if(n_variables_size > n_measurements_size) {
			if(n_measurements_size)
				fprintf(stderr, "warning: the system is underspecified\n");
			else
				fprintf(stderr, "warning: the system contains no edges at all: nothing to optimize\n");
			//return;
		}
		if(!n_measurements_size)
			return; // nothing to solve (but no results need to be generated so it's ok)
		// can't solve in such conditions

		_ASSERTE(this->m_r_system.b_AllVertices_Covered());
		// if not all vertices are covered then the system matrix will be rank deficient and this will fail
		// this triggers typically if solving BA problems with incremental solve each N steps (the "proper"
		// way is to use CONSISTENCY_MARKER and incremental solve period of SIZE_MAX).

		_TyLambdaOps::Extend_Lambda(this->m_r_system, m_reduction_plan,
			m_lambda, m_n_verts_in_lambda, m_n_edges_in_lambda); // recalculated all the jacobians inside Extend_Lambda()
		if(!m_b_system_dirty) {
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda,
				m_n_verts_in_lambda, m_n_edges_in_lambda); // calculate only for new edges // t_odo - but how to mark affected vertices?
		} else
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // calculate for entire system
		m_b_system_dirty = false;
		m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
		m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
		// need to have lambda

		if(m_lambda.n_BlockColumn_Num() < this->m_r_system.r_Vertex_Pool().n_Size()) {
			timer.Accum_DiffSample(m_f_lambda_time);
			fprintf(stderr, "warning: waiting for more edges\n");
			return;
		}
		// waiting for more edges

		_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
			m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
			m_lambda.n_Column_Num() == n_variables_size); // lambda is square, blocks on either side = number of vertices
		// invariants

		Dump_DemoData();

		m_v_dx.resize(n_variables_size, 1);

		if(this->m_b_use_schur)
			this->m_schur_solver.SymbolicDecomposition_Blocky(m_lambda/*, true*/); // the DL solver is forcing guided ordering, force it as well for comparisons
		// calculate the ordering once, it does not change

		if(this->m_b_verbose) {
			size_t n_sys_size = this->m_r_system.n_Allocation_Size();
			size_t n_rp_size = m_reduction_plan.n_Allocation_Size();
			size_t n_lam_size = m_lambda.n_Allocation_Size();
			printf("memory_use(sys: %.2f MB, redplan: %.2f MB, Lambda: %.2f MB)\n",
				n_sys_size / 1048576.0, n_rp_size / 1048576.0, n_lam_size / 1048576.0);
		}
		// print memory use statistics

		for(size_t n_iteration = 0; n_iteration < n_max_iteration_num; ++ n_iteration) {
			++ m_n_iteration_num;
			// debug

			if(this->m_b_verbose) {
				if(n_max_iteration_num == 1)
					printf("\n=== incremental optimization step ===\n\n");
				else
					printf("\n=== nonlinear optimization: iter #" PRIsize " ===\n\n", n_iteration);
			}
			// verbose

			if(n_iteration && m_b_system_dirty) {
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda);
				m_b_system_dirty = false;
			}
			// no need to rebuild lambda, just refresh the values that are being referenced

			timer.Accum_DiffSample(m_f_lambda_time);

			_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_dx);
			// collects the right-hand side vector

			timer.Accum_DiffSample(m_f_rhs_time);

#ifdef _DEBUG
			/*_ASSERTE(m_lambda.n_Row_Num() == n_variables_size); // should be the same
			// collect errors (in parallel)

			{
				CUberBlockMatrix A;
				{
					const Eigen::MatrixXd &r_t_uf = this->m_r_system.r_t_Unary_Factor();
					if(!A.Append_Block(r_t_uf, 0, 0))
						throw std::bad_alloc();
					// add unary factor

					this->m_r_system.r_Edge_Pool().For_Each(CAlloc_JacobianBlocks(A));
					// add all the hessian blocks

					this->m_r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Jacobians());
					// calculate the values as well
				}
				// calculate A

				Eigen::VectorXd v_error(n_measurements_size);
				Collect_R_Errors(v_error);
				Eigen::VectorXd v_eta(n_variables_size);
				v_eta.setZero(); // eta = 0
				Eigen::VectorXd &v_b = v_error; // b = Rz * p_errors
				_ASSERTE(v_eta.rows() == n_variables_size);
				_ASSERTE(v_b.rows() == n_measurements_size);
				A.PostMultiply_Add_FBS<_TyAMatrixBlockSizes>(&v_eta(0), n_variables_size, &v_b(0),
					n_measurements_size); // works (fast parallel post-multiply)
				// calculates eta

				_ASSERTE(v_eta.rows() == m_v_dx.rows());
				for(size_t i = 0, n = v_eta.rows(); i < n; ++ i) {
					if(fabs(m_v_dx(i) - v_eta(i)) > 1e-5)
						fprintf(stderr, "error: RHS vectors do not match\n");
				}
				// compare
			}*/
			// calculate eta = A^T * b
#endif // _DEBUG

			{
				bool b_cholesky_result;
				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					if(!this->m_b_use_schur) {
						if(n_max_iteration_num > 1) {
							do {
								if(!n_iteration &&
								   !_TyLinearSolverWrapper::FinalBlockStructure(this->m_linear_solver, m_lambda)) {
									b_cholesky_result = false;
									break;
								}
								// prepare symbolic factorization, structure of lambda won't change in the next steps
								b_cholesky_result = _TyLinearSolverWrapper::Solve(this->m_linear_solver, m_lambda, v_eta);
								// p_dx = eta = lambda / eta
							} while(0);
						} else
							b_cholesky_result = this->m_linear_solver.Solve_PosDef(m_lambda, v_eta); // p_dx = eta = lambda / eta
					} else { // use Schur complement
						b_cholesky_result = this->m_schur_solver.Solve_PosDef_Blocky(m_lambda, v_eta);
						// Schur
					}

					if(this->m_b_verbose) {
						printf("%s %s", (this->m_b_use_schur)? "Schur" : "Cholesky",
							(b_cholesky_result)? "succeeded\n" : "failed\n");
					}
				}
				// calculate cholesky, reuse block ordering if the linear solver supports it

				timer.Accum_DiffSample(m_f_chol_time);

				double f_residual_norm = 0;
				if(b_cholesky_result) {
					f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
					if(this->m_b_verbose)
						printf("residual norm: %.4f\n", f_residual_norm);
				}
				// calculate residual norm

				timer.Accum_DiffSample(m_f_norm_time);

				if(f_residual_norm <= f_min_dx_norm)
					break;
				// in case the error is low enough, quit (saves us recalculating the hessians)

				if(b_cholesky_result) {
					_TyLambdaOps::PushValuesInGraphSystem(this->m_r_system, m_v_dx);
					m_b_system_dirty = true;

					//printf("debug: just updated the linearization point\n");
					this->m_marginals.DisableUpdate(); // linearization point just changed, all the marginals will change - need full recalc

					timer.Accum_DiffSample(m_f_vert_upd_time);
				}
				// update the system (in parallel)

				if(!b_cholesky_result)
					break;
				// in case cholesky failed, quit
			}
		}
		// optimize the system

		if(this->m_t_marginals_config.b_calculate) {
			// todo - handle freq settings
			// todo - handle policies

			if(m_b_system_dirty) {
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda);
				m_b_system_dirty = false; // does this break something? seems not.

				timer.Accum_DiffSample(m_f_lambda_time);
			}
			_ASSERTE(m_n_verts_in_lambda == this->m_r_system.r_Vertex_Pool().n_Size());
			_ASSERTE(m_n_edges_in_lambda == this->m_r_system.r_Edge_Pool().n_Size());
			_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
				m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
				m_lambda.n_Column_Num() == n_variables_size);
			// need to update or will end up with forever bad marginals!

			CUberBlockMatrix R;
			//R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(m_lambda); // this makes ugly dense factor, dont do that

			//this->m_linear_solver.Factorize_PosDef_Blocky(R, m_lambda, std::vector<size_t>()); // dense as well, no ordering inside

			CMatrixOrdering mord;
			if((this->m_marginals.b_CanUpdate() && (this->m_t_marginals_config.n_incremental_policy &
			   mpart_LastColumn) == mpart_LastColumn) || // can tell for sure if incremental is going to be used
			   (this->m_t_marginals_config.n_relinearize_policy & mpart_LastColumn) == mpart_LastColumn) { // never know if we fallback to batch, though
				CLastElementOrderingConstraint leoc;
				mord.p_BlockOrdering(m_lambda, leoc.p_Get(m_lambda.n_BlockColumn_Num()),
					m_lambda.n_BlockColumn_Num(), true); // constrain the last column to be the last column (a quick fix) // todo - handle this properly, will be unable to constrain like this in fast R (well, actually ...) // todo - see what is this doing to the speed
			} else
				mord.p_BlockOrdering(m_lambda, true); // unconstrained; the last column may be anywhere (could reuse R from the linear solver here - relevant also in batch (e.g. on venice))
			const size_t *p_order = mord.p_Get_InverseOrdering();
			CUberBlockMatrix lambda_perm;
			m_lambda.Permute_UpperTriangular_To(lambda_perm, p_order, mord.n_Ordering_Size(), true);
			if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm))
				throw std::runtime_error("fatal error: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed");
			// note that now the marginals are calculated with ordering: need to count with that, otherwise those are useless!

			// todo - reuse what the linear solver calculated, if we have it (not if schur, )
			// todo - think of what happens if using schur ... have to accelerate the dense margs differently
			//		probably the whole mindset of having R is wrong, it would be much better to leave
			//		it up to the linear solver to solve for the columns

			/*printf("debug: matrix size: " PRIsize " / " PRIsize " (" PRIsize " nnz)\n",
				lambda_perm.n_BlockColumn_Num(), lambda_perm.n_Column_Num(), lambda_perm.n_NonZero_Num());
			float f_avg_block_size = float(lambda_perm.n_Column_Num()) / lambda_perm.n_BlockColumn_Num();
			printf("debug: diagonal nnz: " PRIsize "\n", size_t(lambda_perm.n_BlockColumn_Num() *
				(f_avg_block_size * f_avg_block_size)));
			printf("debug: factor size: " PRIsize " / " PRIsize " (" PRIsize " nnz)\n",
				R.n_BlockColumn_Num(), R.n_Column_Num(), R.n_NonZero_Num());*/
			// see how much we compute, compared to g2o

			timer.Accum_DiffSample(m_f_extra_chol_time);

			size_t n_add_edge_num = this->m_r_system.r_Edge_Pool().n_Size() - this->m_marginals.n_Edge_Num();
			bool b_incremental = this->m_marginals.b_CanUpdate() && CMarginals::b_PreferIncremental(this->m_r_system,
				this->m_marginals.r_SparseMatrix(), m_lambda, R, mord, this->m_marginals.n_Edge_Num(),
				this->m_t_marginals_config.n_incremental_policy);
//#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
			if(b_incremental) { // otherwise just update what we have
				CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
				if(!CMarginals::Update_BlockDiagonalMarginals_FBS<false>(this->m_r_system, r_m, m_lambda,
				   R, mord, this->m_marginals.n_Edge_Num(), this->m_t_marginals_config.n_incremental_policy)) {
#ifdef _DEBUG
					fprintf(stderr, "warning: Update_BlockDiagonalMarginals_FBS() had a numerical issue:"
						" restarting with Calculate_DenseMarginals_Recurrent_FBS() instead\n");
#endif // _DEBUG
					b_incremental = false;
					// failed, will enter the batch branch below, that will not have a numerical issue
				} else {
					this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed

					timer.Accum_DiffSample(m_f_incmarginals_time);
					++ m_n_incmarginals_num;
				}
			}
/*#else // __SE_TYPES_SUPPORT_L_SOLVERS
#pragma message("warning: the fast incremental marginals not available: __SE_TYPES_SUPPORT_L_SOLVERS not defined")
			b_incremental = false;
#endif // __SE_TYPES_SUPPORT_L_SOLVERS*/
			if(!b_incremental) { // if need batch marginals
				CUberBlockMatrix margs_ordered;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ordered, R,
					mord, this->m_t_marginals_config.n_relinearize_policy, false); // todo - use this in other solvers as well
				// calculate the thing

				CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
				margs_ordered.Permute_UpperTriangular_To(r_m, mord.p_Get_Ordering(),
					mord.n_Ordering_Size(), false); // no share! the original will be deleted
				this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
				// take care of having the correct permutation there

				this->m_marginals.EnableUpdate();
				// now the marginals are current and can be updated until the linearization point is changed again

				timer.Accum_DiffSample(m_f_marginals_time);
			}

			this->m_marginals.Set_Edge_Num(this->m_r_system.r_Edge_Pool().n_Size());
			// now all those edges are in the marginals

#if 0
			try {
				CUberBlockMatrix margs_ref, margs_untg;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ref,
					R, mord, mpart_Diagonal);
				margs_ref.Permute_UpperTriangular_To(margs_untg, mord.p_Get_Ordering(),
					mord.n_Ordering_Size(), true); // ref is ok, it will be a short lived matrix
				Eigen::MatrixXd margs_buffer;
				margs_untg.Convert_to_Dense(margs_buffer);
				Eigen::VectorXd diag_rec = margs_buffer.diagonal();
				// recursive does not hurt

				CUberBlockMatrix R_unord;
				R_unord.CholeskyOf(m_lambda); // sloow
				CMarginals::Calculate_DenseMarginals_Ref(margs_buffer, R_unord); // sloow
				//CMarginals::Calculate_DenseMarginals_Fast/*_Parallel_FBS<_TyLambdaMatrixBlockSizes>*/(margs_buffer, R,
				//	mord.p_Get_Ordering(), mord.n_Ordering_Size()); // why does this not work!?
				double f_full_norm = margs_buffer.norm();
				double f_full_max = margs_buffer.maxCoeff();
				double f_diag_norm = margs_buffer.diagonal().norm();
				double f_diag_max = margs_buffer.diagonal().maxCoeff();

				Eigen::VectorXd diag_ref = margs_buffer.diagonal();
				//if(!margs_untg.b_EqualLayout(this->m_marginals.r_SparseMatrix()))
				//	printf("error: the marginals have different block layout\n");
				double f_all_err = CMarginals::f_IncompleteDifference(margs_buffer, this->m_marginals.r_SparseMatrix());
				this->m_marginals.r_SparseMatrix().Convert_to_Dense(margs_buffer);
				double f_error = (diag_ref - margs_buffer.diagonal()).norm();
				printf("debug: vertex " PRIsize ": added " PRIsize
					" edges: marginals diagonal tracking: %g (weighted: %g, fulltrack: %g, mnorm: %g, mmax: %g, dnorm: %g, dmax: %g, recerr: %g)\n",
					m_lambda.n_BlockColumn_Num(), n_add_edge_num, f_error, f_error / f_diag_norm, f_all_err,
					f_full_norm, f_full_max, f_diag_norm, f_diag_max, (diag_rec - diag_ref).norm());
				if(f_error > 100 /*|| diag_ref.maxCoeff() < 250*/) { // the 250 thing is only good for debugging intel.txt
					printf("\tthis calls for special attention ... this is called as (%d, %g)\n",
						int(n_max_iteration_num), f_min_dx_norm);
					Eigen::VectorXd diag_ref2 = margs_buffer.diagonal(); // now there is this->m_marginals.r_SparseMatrix() in there
					CUberBlockMatrix R_unord;
					R_unord.CholeskyOf(m_lambda); // sloow
					CMarginals::Calculate_DenseMarginals_Ref(margs_buffer, R_unord); // sloow
					double f_error = (diag_ref2 - margs_buffer.diagonal()).norm();
					printf("debug again: vertex " PRIsize ": added " PRIsize
						" edges: marginals diagonal tracking: %g (max elem: %g, difference of refs: %g)\n",
						m_lambda.n_BlockColumn_Num(), n_add_edge_num, f_error,
						margs_buffer.diagonal().maxCoeff(), (margs_buffer.diagonal() - diag_ref).norm());

					this->m_marginals.DisableUpdate(); // see how long this lasts
				}
			} catch(std::bad_alloc&) {
				fprintf(stderr, "warning: unable to verify marginals (bad_alloc)\n");
			}
			// the new detailed comparison code

			// the old comparison code is below
#if 0
			try {
				CUberBlockMatrix margs_ref, margs_untg;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ref,
					R, mord, mpart_Diagonal);
				margs_ref.Permute_UpperTriangular_To(margs_untg, mord.p_Get_Ordering(),
					mord.n_Ordering_Size(), true); // ref is ok, it will be a short lived matrix
				Eigen::MatrixXd margs_buffer;
				margs_untg.Convert_to_Dense(margs_buffer);
				Eigen::VectorXd diag_ref = margs_buffer.diagonal();
				if(!margs_untg.b_EqualLayout(this->m_marginals.r_SparseMatrix()))
					printf("error: the marginals have different block layout\n");
				this->m_marginals.r_SparseMatrix().Convert_to_Dense(margs_buffer);
				double f_error = (diag_ref - margs_buffer.diagonal()).norm();
				printf("debug: vertex " PRIsize ": added " PRIsize
					" edges: marginals diagonal tracking: %g (%g weighted, max elem: %g)\n",
					m_lambda.n_BlockColumn_Num(), n_add_edge_num, f_error, f_error / diag_ref.norm(), diag_ref.maxCoeff());
				if(f_error > 100 /*|| diag_ref.maxCoeff() < 250*/) { // the 250 thing is only good for debugging intel.txt
					printf("\tthis calls for special attention ... this is called as (%d, %g)\n",
						int(n_max_iteration_num), f_min_dx_norm);
					Eigen::VectorXd diag_ref2 = margs_buffer.diagonal(); // now there is this->m_marginals.r_SparseMatrix() in there
					CUberBlockMatrix R_unord;
					R_unord.CholeskyOf(m_lambda); // sloow
					CMarginals::Calculate_DenseMarginals_Ref(margs_buffer, R_unord); // sloow
					double f_error = (diag_ref2 - margs_buffer.diagonal()).norm();
					printf("debug again: vertex " PRIsize ": added " PRIsize
						" edges: marginals diagonal tracking: %g (max elem: %g, difference of refs: %g)\n",
						m_lambda.n_BlockColumn_Num(), n_add_edge_num, f_error,
						margs_buffer.diagonal().maxCoeff(), (margs_buffer.diagonal() - diag_ref).norm());

					this->m_marginals.DisableUpdate(); // see how long this lasts
				}
			} catch(std::bad_alloc&) {
				fprintf(stderr, "warning: unable to verify marginals (bad_alloc)\n");
			}
			// calculate marginals again and subtract the diagonal to see how much off it gets
			// (i'm lazy, the whole diagonal blocks should be compared instead, that's why the dense bad_alloc business)
#endif // 0
#endif // 0

			/*FILE *p_fw;
			if((p_fw = fopen("marginals.txt", "w"))) {
				for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
					size_t n_order = m_lambda.n_BlockColumn_Base(i);
					size_t n_dimension = m_lambda.n_BlockColumn_Column_Num(i);
					// get col

					Eigen::MatrixXd block = this->m_marginals.r_Matrix().block(n_order, n_order, n_dimension, n_dimension);
					// get block

					fprintf(p_fw, "block_%d_%d = ", int(i), int(i));
					CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, block);
					// prints the matrix
				}
				fclose(p_fw);
			}*/
			this->m_marginals.Dump_Diagonal();
			// dump diagonal blocks of the marginals to a file
		}
		// now R is up to date, can get marginals
	}

protected:
	/**
	 *	@brief called from Incremental_Step() before optimizing, used to generate data for various demos
	 */
	void IncrementalPreOptDemo() const
	{
#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
		n_new_vertex_num = m_n_nonlinear_solve_threshold;
		m_b_had_loop_closure = true;
		// make it trigger
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

		/*{
			char p_s_name[256];
			sprintf(p_s_name, "edgeinfo_%03" _PRIsize ".m", this->m_r_system.r_Edge_Pool().n_Size());
			FILE *p_fw;
			if((p_fw = fopen(p_s_name, "w"))) {
				int n_edge_num = int(this->m_r_system.r_Edge_Pool().n_Size());
				fprintf(p_fw, "num_edges = %d;\n", n_edge_num);
				for(int i = 0; i < n_edge_num; ++ i) {
					const CEdgePose2D &e = *(const CEdgePose2D*)&this->m_r_system.r_Edge_Pool()[i];
					Eigen::Matrix3d t_jacobian0, t_jacobian1;
					Eigen::Vector3d v_expectation, v_error;
					e.Calculate_Jacobians_Expectation_Error(t_jacobian0, t_jacobian1, v_expectation, v_error);
					fprintf(p_fw, "edge_%d = [%d %d]; %% (zero-based) indices of vertices\n", i, e.n_Vertex_Id(0), e.n_Vertex_Id(1));
					fprintf(p_fw, "sigma_inv_%d = ", i);
					CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, e.t_Sigma_Inv());
					fprintf(p_fw, "jacobian_%d_0 = ", i);
					CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, t_jacobian0);
					fprintf(p_fw, "jacobian_%d_1 = ", i);
					CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, t_jacobian1);
				}

				fclose(p_fw);
			}
		}*/
		// save also image of the system as matlab
	}

	/**
	 *	@brief called from Optimize(), used to generate data for various demos
	 */
	void Dump_DemoData()
	{
		//Save_SystemMatrix_MM("lambda_system.mtx");
		//Dump_SystemMatrix("lambda_system.tga");

		//char p_s_filename[256];
		//sprintf(p_s_filename, "rss2013/%05d_0_lambda.tga", m_n_edges_in_lambda);
		//m_lambda.Rasterize_Symmetric(p_s_filename); // this lambda

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
		char p_s_filename[256];

		sprintf(p_s_filename, "rss2013/%05d_0_lambda.tga", m_n_edges_in_lambda);
		m_lambda.Rasterize_Symmetric(p_s_filename); // this lambda
		sprintf(p_s_filename, "rss2013/%05d_1_lambda2.tga", m_n_edges_in_lambda);
		m_lambda.Rasterize_Symmetric(lambda_prev, true, p_s_filename); // with changes marked, not symmetric

		CUberBlockMatrix Lprev;
		Lprev.CholeskyOf(lambda_prev);
		// do it now, will damage lambda_prev

		typename _TyEdgeMultiPool::_TyConstBaseRef r_last_edge =
			this->m_r_system.r_Edge_Pool()[this->m_r_system.r_Edge_Pool().n_Size() - 1];
		_ASSERTE(r_last_edge.n_Vertex_Num() == 2); // won't work for hyperedges
		size_t v0 = r_last_edge.n_Vertex_Id(0);
		size_t v1 = r_last_edge.n_Vertex_Id(1);
		if(lambda_prev.n_BlockColumn_Num()) {
			size_t v = std::min(v0, v1);
			_ASSERTE(v < this->m_r_system.r_Vertex_Pool().n_Size()); // make sure at least one vertex is not const
			lambda_prev.t_GetBlock_Log(v, v).setZero(); // make sure this is clear
		}

		sprintf(p_s_filename, "rss2013/%05d_2_lambda3.tga", m_n_edges_in_lambda);
		m_lambda.CopyTo(lambda_prev);
		lambda_prev.Scale(0); // zero out the whole thing
		m_lambda.Rasterize_Symmetric(lambda_prev, true, p_s_filename); // all changed, not symmetric

		CUberBlockMatrix L;
		CUberBlockMatrix Lem;
		L.CopyLayoutTo(Lem);
		L.CholeskyOf(m_lambda);
		sprintf(p_s_filename, "rss2013/%05d_3_Lnoord.tga", m_n_edges_in_lambda);
		L.Rasterize(p_s_filename); // no fill-in highlighting
		sprintf(p_s_filename, "rss2013/%05d_4_Lnoord_fill-in.tga", m_n_edges_in_lambda);
		L.Rasterize(m_lambda, false, p_s_filename); // highlight fill-in
		sprintf(p_s_filename, "rss2013/%05d_a_Lnoord_red.tga", m_n_edges_in_lambda);
		L.Rasterize(Lem, false, p_s_filename); // highlight fill-in
		sprintf(p_s_filename, "rss2013/%05d_b_Lnoord_inc.tga", m_n_edges_in_lambda);
		L.Rasterize(Lprev, true, p_s_filename); // highlight fill-in

		m_lambda.CopyTo(lambda_prev);
		// copy the lambda matrix

		std::vector<size_t> cumsum, ccumsum;
		for(size_t i = 0, n_csum = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			n_csum += m_lambda.n_BlockColumn_Column_Num(i);
			cumsum.push_back(n_csum);
		}
		size_t p_cumsum[1] = {1};
		ccumsum.insert(ccumsum.begin(), p_cumsum, p_cumsum + 1);
		CUberBlockMatrix rhs(cumsum.begin(), cumsum.end(), ccumsum.begin(), ccumsum.end());
		m_v_dx.resize(n_variables_size, 1);
		Collect_RightHandSide_Vector(m_v_dx);
		for(size_t i = 0, n_csum = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_w = m_lambda.n_BlockColumn_Column_Num(i);
			if(!rhs.Append_Block(m_v_dx.segment(n_csum, n_w), n_csum, 0))
				throw std::runtime_error("segment size fail");
			n_csum += n_w;
		}
		sprintf(p_s_filename, "rss2013/%05d_5_rhs.tga", m_n_edges_in_lambda);
		rhs.Rasterize(p_s_filename);

		CUberBlockMatrix rhs_nonzero, rhs_hl;
		rhs.CopyLayoutTo(rhs_nonzero);
		rhs.CopyLayoutTo(rhs_hl);
		m_v_dx.setConstant(1);
		for(size_t i = 0, n_csum = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_w = m_lambda.n_BlockColumn_Column_Num(i);
			if(!(i == v0 || i == v1)) {
				if(!rhs_hl.Append_Block(m_v_dx.segment(n_csum, n_w), n_csum, 0))
					throw std::runtime_error("segment size fail");
			}
			n_csum += n_w;
		}
		for(size_t i = 0, n_csum = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_w = m_lambda.n_BlockColumn_Column_Num(i);
			if(!rhs_nonzero.Append_Block(m_v_dx.segment(n_csum, n_w), n_csum, 0))
				throw std::runtime_error("segment size fail");
			n_csum += n_w;
		}
		sprintf(p_s_filename, "rss2013/%05d_9_rhs_add.tga", m_n_edges_in_lambda);
		rhs_nonzero.Rasterize(rhs_hl, true, p_s_filename);
		rhs.Scale(0);
		sprintf(p_s_filename, "rss2013/%05d_6_rhs_red.tga", m_n_edges_in_lambda);
		rhs_nonzero.Rasterize(rhs, true, p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_7_rhs_nnz.tga", m_n_edges_in_lambda);
		rhs_nonzero.Rasterize(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_8_rhs_z.tga", m_n_edges_in_lambda);
		rhs.Rasterize(p_s_filename);

#if 0
		/*sprintf(p_s_filename, "rss2013/%05d_0_lambda.tga", m_n_verts_in_lambda);
		m_lambda.Rasterize_Symmetric(p_s_filename);*/ // probably don't need lambdas

		CUberBlockMatrix lambda_perm;

		CMatrixOrdering mord;
		mord.p_BlockOrdering(m_lambda, true);
		const size_t *p_perm = mord.p_Get_InverseOrdering();

		if(mord100.n_Ordering_Size() < m_lambda.n_BlockColumn_Num()) {
			if(!(m_lambda.n_BlockColumn_Num() % 100))
				mord100.p_BlockOrdering(m_lambda, true); // do full every 100
			else {
				mord100.p_InvertOrdering(mord100.p_ExtendBlockOrdering_with_Identity(
					m_lambda.n_BlockColumn_Num()), m_lambda.n_BlockColumn_Num());
			}
		}
		// maintain an "every 100" ordering as well

		m_lambda.Permute_UpperTriangular_To(lambda_perm, p_perm, mord.n_Ordering_Size(), true);
		sprintf(p_s_filename, "rss2013/%05d_1_lambda-perm.tga", m_n_verts_in_lambda);
		//if(m_n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
		//	lambda_perm.Rasterize_Symmetric(p_s_filename, (m_n_verts_in_lambda < 750 * 6 / _TyAMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 5 : 3); // do not really need lambdas right now

		L.CholeskyOf(lambda_perm);
		size_t n_L_good_blocks = L.n_Block_Num();
		sprintf(p_s_filename, "rss2013/%05d_2_L.tga", m_n_verts_in_lambda);
		//if(m_n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			L.Rasterize(lambda_perm, false, p_s_filename, (m_n_verts_in_lambda < 750 * 6 / _TyAMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 3 : 2); // highlight fill-in

		L.CholeskyOf(m_lambda);
		sprintf(p_s_filename, "rss2013/%05d_9_Lnoord.tga", m_n_verts_in_lambda);
		//if(m_n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			L.Rasterize(lambda_perm, false, p_s_filename, (m_n_verts_in_lambda < 750 * 6 / _TyAMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 5 : 2); // highlight fill-in

		const size_t *p_perm100 = mord100.p_Get_InverseOrdering();
		m_lambda.Permute_UpperTriangular_To(lambda_perm, p_perm100, mord100.n_Ordering_Size(), true);
		sprintf(p_s_filename, "rss2013/%05d_4_lambda-perm-100.tga", m_n_verts_in_lambda);
		//if(m_n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
		//	lambda_perm.Rasterize_Symmetric(p_s_filename, (m_n_verts_in_lambda < 750 * 6 / _TyAMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 5 : 3); // do not really need lambdas right now

		L.CholeskyOf(lambda_perm);
		sprintf(p_s_filename, "rss2013/%05d_5_L100.tga", m_n_verts_in_lambda);
		//if(m_n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			L.Rasterize(lambda_perm, false, p_s_filename, (m_n_verts_in_lambda < 750 * 6 / _TyAMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 3 : 2); // highlight fill-in

		sprintf(p_s_filename, "rss2013/%05d_3_stats.txt", m_n_verts_in_lambda);
		FILE *p_fw;
		if((p_fw = fopen(p_s_filename, "w"))) {
			fprintf(p_fw, PRIsize "\n", lambda_perm.n_Block_Num()); // save density of lambda
			fprintf(p_fw, PRIsize "\n", n_L_good_blocks); // save density of L
			fprintf(p_fw, PRIsize "\n", L.n_Block_Num()); // save density of L at every100
			fclose(p_fw);
		}
#endif // 0
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

#ifdef __ICRA_GET_PRESENTATION_ANIMATION_DATA
		char p_s_filename[256];
		sprintf(p_s_filename, "lambda_%03d.tga", m_n_real_step);
		m_lambda.Rasterize(p_s_filename);
		// dump lambda

		sprintf(p_s_filename, "lambda_%03d.txt", m_n_real_step);
		cs *p_lam = m_lambda.p_BlockStructure_to_Sparse();
		FILE *p_fw = fopen(p_s_filename, "w");
		fprintf(p_fw, "%d x %d\n", p_lam->m, p_lam->n);

		{
			cs *m = p_lam;
			int cols = m->n;
			int rows = m->m;
			int size_i = m->p[cols];
			int col = 0;
			for(int i = 0; i < size_i; ++ i) { // 8x if, 8x add
				int row = m->i[i]; // 7x read
				while(m->p[col + 1] <= i) // 7x if / 10x if (in case of while), the same amount of reads
					col ++; // 4x add
				fprintf(p_fw, "%d, %d\n", row, col);
			}
			// 14 - 17x read, 12x add, 15 - 18x if
		}
		fclose(p_fw);
		sprintf(p_s_filename, "lambda_%03d_ref.txt", m_n_real_step);
		p_fw = fopen(p_s_filename, "w");
		fprintf(p_fw, "%d x %d\n", p_lam->m, p_lam->n);
		{
			cs *m = p_lam;
			int cols = m->n;
			int rows = m->m;
			for(int j = 0; j < cols; ++ j) { // 5x if, 5x add
				int col = j;
				int first_i = m->p[j]; // 4x read
				int last_i = m->p[j + 1]; // 4x read
				for(int i = first_i; i < last_i; ++ i) { // 7+4=11x if, 11x add
					int row = m->i[i]; // 7x read
					fprintf(p_fw, "%d, %d\n", row, col);
				}
			}
			// 15x read, 16x add, 16x if
		}
		fclose(p_fw);
		cs_spfree(p_lam);
#endif // __ICRA_GET_PRESENTATION_ANIMATION_DATA

		/*m_lambda.Rasterize_Symmetric("lambda.tga");
		{
			FILE *p_fw = fopen("system.dot", "w");
			fprintf(p_fw, "digraph system_in_A {\n");
			printf("\trankdir = LR;\n");
			fprintf(p_fw, "\tnode [shape = doublecircle];");
			for(size_t i = 0, n = this->m_r_system.r_Vertex_Pool().n_Size(); i < n; ++ i) {
				const _TyBaseVertex &r_vertex = this->m_r_system.r_Vertex_Pool()[i];
				if(r_vertex.n_Dimension() == 2) {
					char p_s_vname[64];
					sprintf(p_s_vname, "L" PRIsize, i);
					fprintf(p_fw, " %s", p_s_vname);
				}
			}
			printf("\n");
			fprintf(p_fw, "\tnode [shape = circle];\n");
			fprintf(p_fw, "\t%s -> %s;\n", "UF", "V0");
			for(size_t i = 0, n = this->m_r_system.r_Edge_Pool().n_Size(); i < n; ++ i) {
				const _TyBaseEdge &r_edge = this->m_r_system.r_Edge_Pool()[i];
				int n_v0 = r_edge.n_Vertex_Id(0);
				int n_v1 = r_edge.n_Vertex_Id(1);
				char p_s_vname0[64], p_s_vname1[64];
				sprintf(p_s_vname0, "%c" PRIsize, (this->m_r_system.r_Vertex_Pool()[n_v0].n_Dimension() == 2)? 'L' : 'V', n_v0);
				sprintf(p_s_vname1, "%c" PRIsize, (this->m_r_system.r_Vertex_Pool()[n_v1].n_Dimension() == 2)? 'L' : 'V', n_v1);
				fprintf(p_fw, "\t%s -> %s;\n", p_s_vname0, p_s_vname1);
			}
			fprintf(p_fw, "}\n");
			fclose(p_fw);
		}*/
		// save a matrix and make a .dot file of the graph
	}

	/**
	 *	@brief called from Incremental_Step() after optimizing, used to generate data for various demos
	 */
	void IncrementalPostOptDemo(size_t n_new_vertex_num) const
	{
#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
		if(n_new_vertex_num) {
			FILE *p_fw = fopen("chi2perVert.txt", (m_n_real_step > 0)? "a" : "w");
			fprintf(p_fw, "%f\n", f_Chi_Squared_Error_Denorm());
			fclose(p_fw);
			// dump chi2

            //printf("time is %lf\n", this->m_timer.f_Time()); // print incremental times (ICRA 2013)
#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_ICRA2013_ANIMATION_DATA
			char p_s_filename[256];
			sprintf(p_s_filename, "icra2013/sys%05" _PRIsize ".txt", this->m_r_system.r_Vertex_Pool().n_Size());
			this->m_r_system.Dump(p_s_filename);
			//sprintf(p_s_filename, "icra2013/sys%05" _PRIsize ".tga", this->m_r_system.r_Vertex_Pool().n_Size());
			//this->m_r_system.Plot2D(p_s_filename); // no plot, takes a ton of memory
			// dump vertices for the ICRA animation
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_ICRA2013_ANIMATION_DATA
		}
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2

		/*{
			char p_s_name[256], p_s_name1[256];
			sprintf(p_s_name, "lambda_%03" _PRIsize ".tga", this->m_r_system.r_Edge_Pool().n_Size());
			m_lambda.Rasterize(p_s_name);
			sprintf(p_s_name, "lambda_%03" _PRIsize ".mtx", this->m_r_system.r_Edge_Pool().n_Size());
			sprintf(p_s_name1, "lambda_%03" _PRIsize ".bla", this->m_r_system.r_Edge_Pool().n_Size());
			m_lambda.Save_MatrixMarket(p_s_name, p_s_name1);
		}*/
		// save matrix image and matrix market
	}

	CNonlinearSolver_Lambda(const CNonlinearSolver_Lambda &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_Lambda &operator =(const CNonlinearSolver_Lambda &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_LAMBDA_INCLUDED
