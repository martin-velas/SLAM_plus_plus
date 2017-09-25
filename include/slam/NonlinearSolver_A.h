/*
								+-----------------------------------+
								|                                   |
								|   ***  A non-linear solver  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|        NonlinearSolver_A.h        |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_A_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_A_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_A.h
 *	@brief nonlinear blocky solver working above the A matrix
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 */

#include "slam/LinearSolver_Schur.h"
#include "slam/NonlinearSolver_Base.h"

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@brief utilities for A solvers
 */
namespace A_utils {

/**
 *	@brief cless with function objects for Jacobian calculations
 */
// *	@tparam CBlockSizes is list of A matrix block sizes, as fbs_ut::CCTSize2D
//template <class CBlockSizes>
class CJacobianOps : public nonlinear_detail::CSolverOps_Base {
//public:
	//typedef CBlockSizes _TyAMatrixBlockSizes; /**< @brief possible block matrices, that can be found in A */

public:
	/**
	 *	@brief function object that calls hessian block allocation for all edges
	 */
	class CAlloc_JacobianBlocks {
	protected:
		CUberBlockMatrix &m_r_A; /**< @brief reference to the A matrix (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_A is reference to the A matrix
		 */
		inline CAlloc_JacobianBlocks(CUberBlockMatrix &r_A)
			:m_r_A(r_A)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to have hessian blocks allocated in A
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) // throw(std::bad_alloc)
		{
			r_t_edge.Alloc_JacobianBlocks(m_r_A);
		}
	};

	/**
	 *	@brief function object that copies ordering from the edges
	 */
	class CGetCumsums {
	protected:
		std::vector<size_t>::iterator m_p_cumsum_it; /**< @brief cumsum iterator (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_row_cumsum_list is cumsum iterator
		 */
		inline CGetCumsums(std::vector<size_t> &r_row_cumsum_list)
			:m_p_cumsum_it(r_row_cumsum_list.begin())
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to output its cumsum
		 */
		template <class _TyEdge>
		inline void operator ()(const _TyEdge &r_t_edge)
		{
			size_t n_order = r_t_edge.n_Order();
			//_ASSERTE(n_order > 0); // don't want 0, that is assigned to the unary factor // except if using explicit unary factor. can't check for it from here unfortunately
			*m_p_cumsum_it = n_order;
			++ m_p_cumsum_it;
		}

		/**
		 *	@brief conversion back to cumsum iterator
		 *	@return Returns the value of cumsum iterator.
		 */
		inline operator std::vector<size_t>::iterator() const
		{
			return m_p_cumsum_it;
		}
	};

	/**
	 *	@brief function object that calculates hessians in all the edges
	 */
	class CCalculate_Jacobians {
	public:
		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in] r_t_edge is edge to update it's hessians
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) const
		{
			r_t_edge.Calculate_Jacobians();
		}
	};

	/**
	 *	@brief function object that calls error vector calculation for all edges
	 */
	class CCollect_R_Errors {
	protected:
		Eigen::VectorXd &m_r_R_errors; /**< @brief reference to the R error vector (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_R_errors is reference to the R error vector
		 */
		inline CCollect_R_Errors(Eigen::VectorXd &r_R_errors)
			:m_r_R_errors(r_R_errors)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to output its part R error vector
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge)
		{
			r_t_edge.Get_R_Error(m_r_R_errors);
		}
	};

public:
	/**
	 *	@brief incrementally updates the A matrix structure (can be empty)
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in,out] r_system is reference to the system
	 *	@param[in,out] r_A is reference to the A matrix
	 *	@param[in] n_edges_already_in_A is number of edges, already in the system
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CSystem>
	static inline void Extend_A(CSystem &r_system, CUberBlockMatrix &r_A, size_t n_edges_already_in_A) // throw(std::bad_alloc)
	{
		if(!n_edges_already_in_A)
			AddEntriesInSparseSystem(r_system, r_A);
		else
			UpdateSparseSystem(r_system, r_A, n_edges_already_in_A);
		// create block matrix A
	}

	/**
	 *	@brief refreshes the A matrix by recalculating edge hessians
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in,out] r_system is reference to the system
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge to be refreshed
	 */
	template <class CSystem>
	static inline void Refresh_A(CSystem &r_system, size_t n_refresh_from_edge = 0)
	{
		if(n_refresh_from_edge) {
			r_system.r_Edge_Pool().For_Each_Parallel(n_refresh_from_edge,
				r_system.r_Edge_Pool().n_Size(), CCalculate_Jacobians());
		} else
			r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Jacobians());
		// can do this in parallel
	}

	/**
	 *	@brief calculates the error vector
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in] r_system is reference to the system
	 *	@param[out] r_v_R_error is reference to the R times error vector
	 */
	template <class CSystem>
	static inline void Collect_R_Errors(const CSystem &r_system, Eigen::VectorXd &r_v_R_error)
	{
		if(!CSystem::null_UnaryFactor) {
			const Eigen::VectorXd &r_v_err = r_system.r_v_Unary_Error();
			r_v_R_error.segment(0, r_v_err.rows()) = r_v_err;
		} else {
			if(!r_system.r_Vertex_Pool().b_Empty()) // otherwise r_v_R_error is empty
				r_v_R_error.segment(0, r_system.r_Vertex_Pool()[0].n_Dimension()).setZero();
		}
		// add the first error

		r_system.r_Edge_Pool().For_Each_Parallel(CCollect_R_Errors(r_v_R_error)); // can do this in parallel
		// collect errors
	}

protected:
	/**
	 *	@brief creates the A matrix from scratch
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in,out] r_system is reference to the system
	 *	@param[out] r_A is reference to the A matrix
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	template <class CSystem>
	static inline void AddEntriesInSparseSystem(CSystem &r_system, CUberBlockMatrix &r_A) // throw(std::bad_alloc, std::runtime_error)
	{
		if(r_system.r_Edge_Pool().n_Size() > 1000) { // wins 2.42237 - 2.48938 = .06701 seconds on 10k.graph, likely more on larger graphs
			//printf("building large matrix from scratch ...\n"); // debug
			std::vector<size_t> row_cumsum_list(r_system.r_Edge_Pool().n_Size());
			/*std::vector<size_t>::iterator p_end_it =*/
				r_system.r_Edge_Pool().For_Each(CGetCumsums(row_cumsum_list));
			//_ASSERTE(p_end_it == row_cumsum_list.end());
			// collect cumsums

			CUberBlockMatrix tmp(row_cumsum_list.begin(),
				row_cumsum_list.end(), r_system.r_Vertex_Pool().n_Size());
			r_A.Swap(tmp);
			// use this one instead

			// todo - see if there are some row_reindex on 100k, fix it by collecting
			// cumsums and building matrix with that (proven to be faster before)
		} else {
			//printf("building small matrix from scratch ...\n"); // debug
			r_A.Clear();
			// ...
		}

		if(!CSystem::null_UnaryFactor) {
			size_t n_UF_order = r_system.n_Unary_Factor_Order();
			const Eigen::MatrixXd &r_t_uf = r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_gauge_vertex = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Edge_Pool().b_Empty());
			size_t n_gauge_vertex = r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Vertex_Pool()[n_gauge_vertex].b_IsConstant()); // this one must not be
			size_t n_gauge_order = r_system.r_Vertex_Pool()[n_gauge_vertex].n_Order();
			if(!r_A.Append_Block(r_t_uf, n_UF_order, n_gauge_order)) // put the UF at the right spot
				throw std::bad_alloc();
			// add unary factor
		}

		r_system.r_Edge_Pool().For_Each(CAlloc_JacobianBlocks(r_A));
		// add all the hessian blocks

		//printf("building A from scratch finished\n"); // debug
	}

	/**
	 *	@brief incrementally updates the A matrix structure (must not be empty)
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in,out] r_system is reference to the system
	 *	@param[in,out] r_A is reference to the A matrix
	 *	@param[in] n_skip_edges is number of edges, already in the system
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CSystem>
	static inline void UpdateSparseSystem(CSystem &r_system, CUberBlockMatrix &r_A, size_t n_skip_edges) // throw(std::bad_alloc)
	{
		_ASSERTE(r_A.n_Row_Num() > 0 && r_A.n_Column_Num() > 0); // make sure A is not empty
		r_system.r_Edge_Pool().For_Each(n_skip_edges, r_system.r_Edge_Pool().n_Size(), CAlloc_JacobianBlocks(r_A));
		// add the hessian blocks of the new edges
	}
};

} // ~A_utils

/**
 *	@brief nonlinear blocky solver working above the A matrix
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes = typename CSystem::_TyJacobianMatrixBlockList>
class CNonlinearSolver_A : public nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, false> {
public:
	typedef CSystem _TySystem; /**< @brief sy	stem type */
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

	typedef A_utils::CJacobianOps/*<_TyAMatrixBlockSizes>*/ _TyJacOps; /**< @brief utility class for filling the A matrix */

	/**
	 *	@brief solver interface properties, stored as enum (see also CSolverTraits)
	 */
	enum {
		solver_HasDump = true, /**< @brief timing statistics support flag */
		solver_HasChi2 = true, /**< @brief Chi2 error calculation support flag */
		solver_HasMarginals = false, /**< @brief marginal covariance support flag */
		solver_HasGaussNewton = true, /**< @brief Gauss-Newton support flag */
		solver_HasLevenberg = false, /**< @brief Levenberg-Marquardt support flag */
		solver_HasGradient = false, /**< @brief gradient-based linear solving support flag */
		solver_HasSchur = true, /**< @brief Schur complement support flag */
		solver_HasDelayedOptimization = false, /**< @brief delayed optimization support flag */
		solver_IsPreferredBatch = true, /**< @brief preferred batch solver flag */
		solver_IsPreferredIncremental = false, /**< @brief preferred incremental solver flag */
		solver_ExportsJacobian = true, /**< @brief interface for exporting jacobian system matrix flag */
		solver_ExportsHessian = false, /**< @brief interface for exporting hessian system matrix flag */
		solver_ExportsFactor = false /**< @brief interface for exporting factorized system matrix flag */
	};

protected:
	typedef nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, false> _TyBase; /**< @brief base solver utils type */

//	CSystem &this->m_r_system; /**< @brief reference to the system */
//	CLinearSolver this->m_linear_solver; /**< @brief linear solver */
//	CLinearSolver_Schur<CLinearSolver, _TyAMatrixBlockSizes, CSystem> this->m_schur_solver; /**< @brief linear solver with Schur trick */

	CUberBlockMatrix m_A; /**< @brief the A matrix (built incrementally) */
	Eigen::VectorXd m_v_error; /**< @brief error vector */
	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	size_t m_n_edges_in_A; /**< @brief number of edges already in A */

//#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
//	size_t m_n_last_optimized_vertex_num;
//#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
//	size_t m_n_step; /**< @brief counter of incremental steps modulo m_n_nonlinear_solve_threshold */
//#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
//	size_t m_n_linear_solve_threshold; /**< @brief step threshold for linear solve */
//	size_t m_n_nonlinear_solve_threshold; /**< @brief step threshold for nonlinear solve */
//	size_t m_n_nonlinear_solve_max_iteration_num; /**< @brief maximal number of iterations in incremental nonlinear solve */
//	double m_f_nonlinear_solve_error_threshold; /**< @brief error threshold in incremental nonlinear solve */ // t_odo - document these in elementwise A and L
//	bool this->m_b_verbose; /**< @brief verbosity flag */
//	bool this->m_b_use_schur; /**< @brief use schur complement flag */
//	size_t m_n_real_step; /**< @brief counter of incremental steps (no modulo) */

	bool m_b_system_dirty; /**< @brief system updated without relinearization flag */

	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	double m_f_serial_time; /**< @brief time spent in serial section */
	double m_f_ata_time; /**< @brief time spent in A^T * A section */
	double m_f_premul_time; /**< @brief time spent in b * A section */
	double m_f_chol_time; /**< @brief time spent in Choleski() section */
	double m_f_norm_time; /**< @brief time spent in norm calculation section */

//	CTimer this->m_timer; /**< @brief timer object */
//
//	bool m_b_had_loop_closure; /**< @brief (probable) loop closure flag */

public:
	/**
	 *	@brief constructor; initializes the nonlinear solver
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
	CNonlinearSolver_A(CSystem &r_system, size_t n_linear_solve_threshold,
		size_t n_nonlinear_solve_threshold, size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = true)
		:_TyBase(r_system, n_linear_solve_threshold,
		n_nonlinear_solve_threshold, n_nonlinear_solve_max_iteration_num,
		f_nonlinear_solve_error_threshold, b_verbose, linear_solver, b_use_schur),
		/*this->m_r_system(r_system), this->m_linear_solver(linear_solver),
		this->m_schur_solver(linear_solver),*/ m_n_edges_in_A(0),
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
		m_b_system_dirty(false), m_n_iteration_num(0), m_f_serial_time(0), m_f_ata_time(0),
		m_f_premul_time(0), m_f_chol_time(0), m_f_norm_time(0)//, m_b_had_loop_closure(false)
	{
		//_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those
	}

	/**
	 *	@brief constructor; initializes the nonlinear solver
	 *
	 *	@param[in] r_system is the system to be optimized
	 *		(it is only referenced, not copied - must not be deleted)
	 *	@param[in] t_incremental_config is incremental solving configuration
	 *	@param[in] t_marginals_config is marginal covariance calculation configuration
	 *	@param[in] b_verbose is verbosity flag
	 *	@param[in] linear_solver is linear solver instance
	 *	@param[in] b_use_schur is Schur complement trick flag
	 */
	CNonlinearSolver_A(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = true)
		:_TyBase(r_system, t_incremental_config,
		TMarginalsComputationPolicy(), b_verbose, linear_solver, b_use_schur),
		/*this->m_r_system(r_system), this->m_linear_solver(linear_solver),
		this->m_schur_solver(linear_solver),*/ m_n_edges_in_A(0),
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
		m_b_system_dirty(false), m_n_iteration_num(0), m_f_serial_time(0), m_f_ata_time(0),
		m_f_premul_time(0), m_f_chol_time(0), m_f_norm_time(0)//, m_b_had_loop_closure(false)
	{
		//_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those
		if(t_marginals_config.b_calculate) // not supported at the moment
			throw std::runtime_error("CNonlinearSolver_A does not support marginals calculation");
	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
		if(f_total_time > 0)
			printf("solver spent %f seconds in parallelizable section (updating A)\n", f_total_time - m_f_serial_time);
		printf("solver spent %f seconds in serial section\n", m_f_serial_time);
		printf("out of which:\n");
		printf("\t  ata: %f\n", m_f_ata_time);
		printf("\tgaxpy: %f\n", m_f_premul_time);
		printf("\t chol: %f\n", m_f_chol_time);
		printf("\t norm: %f\n", m_f_norm_time);
		printf("\ttotal: %f\n", m_f_ata_time + m_f_premul_time + m_f_chol_time + m_f_norm_time);
	}

	/**
	 *	@brief gets the current system matrix (not necessarily up-to-date)
	 *	@return Returns const reference to the current system matrix.
	 */
	const CUberBlockMatrix &r_A() const
	{
		return m_A;
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
			_TyJacOps::Extend_A(this->m_r_system, m_A, m_n_edges_in_A); // recalculated all the jacobians inside Extend_A()
			if(!m_b_system_dirty)
				_TyJacOps::Refresh_A(this->m_r_system, m_n_edges_in_A); // calculate only for new edges
			else
				_TyJacOps::Refresh_A(this->m_r_system); // calculate for entire system
			m_b_system_dirty = false;
			m_n_edges_in_A = this->m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
			_ASSERTE(m_A.n_Row_Num() >= m_A.n_Column_Num()); // no, A is *not* square, but size of measurements >= vertices
		} catch(std::bad_alloc&) {
			return false;
		}
		// need to have A

		return m_A.Rasterize(p_s_filename, n_scalar_size);
	}

	/**
	 *	@brief writes system matrix in matrix market for benchmarking purposes
	 *
	 *	@param[in] p_s_filename is output file name (.mtx)
	 *
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

		return m_A.Save_MatrixMarket(p_s_filename, p_s_layout_file, "A matrix for SLAM problem");
	}

	/**
	 *	@brief incremental optimization function
	 *	@param[in] r_last_edge is the last edge that was added to the system
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
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES

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
				m_b_had_loop_closure = false;
				Optimize(m_n_nonlinear_solve_max_iteration_num, m_f_nonlinear_solve_error_threshold);
			}
			// nonlinear optimization
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
				m_b_had_loop_closure = false;
				Optimize(1, 0); // only if there was a loop (ignores possibly high residual after single step optimization)
			}
			// simple optimization
		}*/

		std::pair<bool, int> t_optimize = this->t_Incremental_Step(r_last_edge);
		if(t_optimize.second == 2) {
			Optimize(this->m_t_incremental_config.n_max_nonlinear_iteration_num,
				this->m_t_incremental_config.f_nonlinear_error_thresh); // nonlinear optimization
		} else if(t_optimize.second == 1)
			Optimize(1, 0); // linear optimization
		// decide on incremental optimization
	}

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
		const size_t n_variables_size = this->m_r_system.n_VertexElement_Num();
		const size_t n_measurements_size = this->m_r_system.n_EdgeElement_Num();
		if(n_variables_size > n_measurements_size) {
			if(n_measurements_size)
				fprintf(stderr, "warning: the system is underspecified\n");
			else
				fprintf(stderr, "warning: the system contains no edges at all: nothing to optimize\n");
			return;
		}
		if(!n_measurements_size)
			return; // nothing to solve (but no results need to be generated so it's ok)
		// can't solve in such conditions

		_ASSERTE(this->m_r_system.b_AllVertices_Covered());
		// if not all vertices are covered then the system matrix will be rank deficient and this will fail
		// this triggers typically if solving BA problems with incremental solve each N steps (the "proper"
		// way is to use CONSISTENCY_MARKER and incremental solve period of SIZE_MAX).

		_TyJacOps::Extend_A(this->m_r_system, m_A, m_n_edges_in_A); // recalculated all the jacobians inside Extend_A()
		if(!m_b_system_dirty)
			_TyJacOps::Refresh_A(this->m_r_system, m_n_edges_in_A); // calculate only for new edges
		else
			_TyJacOps::Refresh_A(this->m_r_system); // calculate for entire system
		m_b_system_dirty = false;
		m_n_edges_in_A = this->m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
		//_ASSERTE(m_A.n_Row_Num() >= m_A.n_Column_Num()); // no, A is *not* square, but size of measurements >= vertices // this is lifted with solving of underspecified systems
		// need to have A

		//Dump_SystemMatrix("A_system.tga");

		m_v_error.resize(n_measurements_size, 1);
		m_v_dx.resize(n_variables_size, 1);

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
				_TyJacOps::Refresh_A(this->m_r_system);
				m_b_system_dirty = false;
			}
			// no need to rebuild A, just refresh the values that are being referenced

			_ASSERTE(m_A.n_Row_Num() == n_measurements_size); // should be the same
			_TyJacOps::Collect_R_Errors(this->m_r_system, m_v_error);
			// collect errors (in parallel)

			double f_serial_start = this->m_timer.f_Time();

			{
				CUberBlockMatrix lambda;
				m_A.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyAMatrixBlockSizes>(lambda, true);
				// calculate lambda without calculating At (and only upper diagonal thereof)

				double f_ata_end = this->m_timer.f_Time();

				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					v_eta.setZero(); // eta = 0
					Eigen::VectorXd &v_b = m_v_error; // b = Rz * p_errors
					_ASSERTE(v_eta.rows() == n_variables_size);
					_ASSERTE(v_b.rows() == n_measurements_size);

					m_A.PostMultiply_Add_FBS_Parallel<_TyAMatrixBlockSizes>(&v_eta(0), n_variables_size, &v_b(0),
						n_measurements_size); // works (fast parallel post-multiply)
					// use the function with fixed-block sizes
				}
				// calculate eta = A^T * b

				double f_mul_end = this->m_timer.f_Time();

				bool b_cholesky_result;
				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					if(!this->m_b_use_schur) {
						if(n_max_iteration_num > 1) {
							do {
								if(!n_iteration &&
								   !_TyLinearSolverWrapper::FinalBlockStructure(this->m_linear_solver, lambda)) {
									b_cholesky_result = false;
									break;
								}
								// prepare symbolic factorization, structure of lambda won't change in the next steps

								b_cholesky_result = _TyLinearSolverWrapper::Solve(this->m_linear_solver, lambda, v_eta);
								// p_dx = eta = lambda / eta
							} while(0);
						} else
							b_cholesky_result = this->m_linear_solver.Solve_PosDef(lambda, v_eta); // p_dx = eta = lambda / eta
					} else { // use Schur complement
						if(!n_iteration)
							this->m_schur_solver.SymbolicDecomposition_Blocky(lambda);
						b_cholesky_result = this->m_schur_solver.Solve_PosDef_Blocky(lambda, v_eta);
						// Schur
					}

					if(this->m_b_verbose) {
						printf("%s %s", (this->m_b_use_schur)? "Schur" : "Cholesky",
							(b_cholesky_result)? "succeeded\n" : "failed\n");
					}
				}
				// calculate cholesky, reuse block ordering if the linear solver supports it

				double f_chol_end = this->m_timer.f_Time();

				double f_residual_norm = 0;
				if(b_cholesky_result) {
					f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
					if(this->m_b_verbose)
						printf("residual norm: %.4f\n", f_residual_norm);
				}
				// calculate residual norm

				double f_norm_end = this->m_timer.f_Time();
				double f_serial_time = f_norm_end - f_serial_start;
				m_f_ata_time += f_ata_end - f_serial_start;
				m_f_premul_time += f_mul_end - f_ata_end;
				m_f_chol_time += f_chol_end - f_mul_end;
				m_f_norm_time += f_norm_end - f_chol_end;
				m_f_serial_time += f_serial_time;
				// timing breakup

				if(f_residual_norm <= f_min_dx_norm)
					break;
				// in case the error is low enough, quit (saves us recalculating the hessians)

				if(b_cholesky_result) {
					_TyJacOps::PushValuesInGraphSystem(this->m_r_system, m_v_dx);
					m_b_system_dirty = true;

					// t_odo - create more efficient system; use FAP to keep vertices and edges (doesn't change addresses, can quickly iterate)
					// t_odo - keep in mind that the solver will have some matrix dimensions fixed, do that the way g2o guys do // did it better
					// t_odo - design the system so that vertices and edges are c++ objects? might present problems with allocators, but still ... might be nice to be able to template stuff away
					// t_odo - keep in mind that we might want to only put stuff in A (make refs to blocks), not to carry out actual calculation (makes management of m_b_system_dirty complex)
					// t_odo - *parallelize* PushValuesInGraphSystem() and p_Collect_R_Errors() and CalcJacobiansAndErrors(), if we can parallelize those, we're missing only about 10 seconds to be the best

					// t_odo - mark vertices affected by p_dx, recalculate error and jacobians in those only. // bad idea: all are affected (would have to threshold)
					// t_odo - find out an efficient way of mapping indices of different granularity to vertices // don't know what that means anymore
					// t_odo - find out how to calculate the set of edges that reference the vertices (could use per-vertex sets and merge those. don't want to use trees, think about sorted arrays) // kept in vertex, no need to sort it
					// t_odo - update lambda and at incrementaly (or use at = transpose(chol(lambda))?) // could also keep and update b // this is offtopic now
				}
				// update the system (in parallel)

				if(!b_cholesky_result)
					break;
				// in case cholesky failed, quit
			}
		}
	}

protected:
	CNonlinearSolver_A(const CNonlinearSolver_A &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_A &operator =(const CNonlinearSolver_A &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_A_INCLUDED
