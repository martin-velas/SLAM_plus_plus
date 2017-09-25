/*
								+-----------------------------------+
								|                                   |
								| *** L factor nonlinear solver *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|        NonlinearSolver_L.h        |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_L_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_L_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_L.h
 *	@brief nonlinear blocky solver working above the L factor matrix
 *	@author -tHE SWINe-
 *	@date 2012-09-13
 *
 *	@date 2012-11-29
 *
 *	Fixed a bug in two-level ordering; the ordering constraint vector must apparently
 *	use consecutive integers, starting with zero. That was not the case for very short
 *	constraints, causing problems (the ordering contained multiple occurences of the
 *	same number and missed another numbers altogether, resulting in undefined behavior
 *	of the nonlinear solver).
 *
 *	Fixed a bug with camd_internal.h defining NDEBUG even if _DEBUG was defined, potentially
 *	disabling debugging facilities.
 *
 */

#include "slam/FlatSystem.h"
#include "slam/IncrementalPolicy.h"

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@def __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
 *	@brief enables writes of diagnostic data (timing samples, ...)
 */
//#define __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS

/**
 *	@def __NONLINEAR_SOLVER_L_DUMP_CHI2
 *	@brief enables writes of chi2 errors at each step
 */
//#define __NONLINEAR_SOLVER_L_DUMP_CHI2

/**
 *	@def __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
 *	@brief if defined, chi2 is calculated at the last edge, just before introducing a new vertex,
 *		that gives a different chi2 than calculating chi2 just after introducing a new vertex (default)
 */
//#define __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE

/**
 *	@def __NONLINEAR_SOLVER_L_DUMP_DENSITY
 *	@brief enables writes of density of L given different ordering strategies
 */
//#define __NONLINEAR_SOLVER_L_DUMP_DENSITY

/**
 *	@def __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
 *	@brief if defined, enables writes of timing of different L update variants
 */
//#define __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES

/**
 *	@def __NONLINEAR_SOLVER_L_DETAILED_TIMING
 *	@brief replaces timer implementation with dummy timer to avoid costy runtime
 *		library calls to get the time (more costy on windows than on linux)
 */
#define __NONLINEAR_SOLVER_L_DETAILED_TIMING

/**
 *	@def __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
 *	@brief uses sparse backsubstitution; this requires keeping sparse copy of L,
 *		incurring further overhead (only use for reference results if something
 *		seems wrong)
 */
//#define __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST

/**
 *	@def __NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST
 *	@brief if defined, uses resumed backsubstitution for more efficient
 *		calculating d while doing incremental updates of L using L11
 */
#define __NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST

/**
 *	@def __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY
 *	@brief enables computation of dense cholesky for small loops
 *		(a speed optimization; this is currently limited to 2D problems
 *		as the decision tree is hardcoded - will not fail on other
 *		problems though, it will just not run faster)
 */
#define __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY

/**
 *	@def __NONLIVEAR_SOLVER_L_PARALLEL_MATMULT_THRESH
 *	@brief matrix size threshold for parallel multiplication
 *		in L update (in blocks, used to calculate L10L10T and L11L11T)
 */
#define __NONLIVEAR_SOLVER_L_PARALLEL_MATMULT_THRESH 200

//extern int n_dummy_param; // remove me

#error "fatal error: this solver is deprecated (was not updated to use the new reduction plan), use FastL instead"

/**
 *	@brief nonlinear blocky solver working above the L factor matrix
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix
 *	@tparam CLambdaMatrixBlockSizes is list of block sizes in the information (Hessian) matrix
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes = typename CSystem::_TyJacobianMatrixBlockList,
	class CLambdaMatrixBlockSizes = typename CSystem::_TyHessianMatrixBlockList>
class CNonlinearSolver_L {
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
	typedef /*typename*/ CLambdaMatrixBlockSizes /*fbs_ut::CBlockSizesAfterPreMultiplyWithSelfTranspose<
		_TyAMatrixBlockSizes>::_TyResult*/ _TyLambdaMatrixBlockSizes; /**< @brief possible block matrices, found in lambda and L */

	/**
	 *	@brief some run-time constants, stored as enum
	 */
	enum {
		b_Is_PoseOnly_SLAM = CTypelistLength<_TyAMatrixBlockSizes>::n_result == 1 /**< @brief determines if we're doing pose-only SLAM (10k) */
	};

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
		solver_HasSchur = false, /**< @brief Schur complement support flag */
		solver_HasDelayedOptimization = false, /**< @brief delayed optimization support flag */
		solver_IsPreferredBatch = false, /**< @brief preferred batch solver flag */
		solver_IsPreferredIncremental = true, /**< @brief preferred incremental solver flag */
		solver_ExportsJacobian = false, /**< @brief interface for exporting jacobian system matrix flag */
		solver_ExportsHessian = false, /**< @brief interface for exporting hessian system matrix flag */
		solver_ExportsFactor = false /**< @brief interface for exporting factorized system matrix flag */
	};

protected:
	CSystem &m_r_system; /**< @brief reference to the system */
	CLinearSolver m_linear_solver; /**< @brief linear solver */
	CLinearSolver m_linear_solver2; /**< @brief linear solver for calculating cholesky of L and cholesky of L increment */

#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
	cs *m_p_L; /**< @brief the copy of L matrix (required for backsubstitution) */ // todo - implement backsubstitution in CUberBlockMatrix, by blocks
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
	CUberBlockMatrix m_R; /**< @brief the L matrix (built / updated incrementally) */

	bool m_b_had_loop_closure; /**< @brief (probable) loop closure flag */
	bool m_b_first_iteration_use_L; /**< @brief flag for using the L matrix or rather lambda in the first iteration of nonlinear optimization */
	bool m_b_L_up_to_date; /**< @brief dirty flag for the L matrix (required to keep track after lambda updates and linearization point changes) */
	size_t m_n_last_full_L_update_size; /**< @brief the last number of block columns in L when it was fully updated */
	std::vector<size_t> m_L_row_lookup_table; /**< @brief row lookup table for L (used by b_Refresh_L() and Refresh_L11()) */
	size_t m_n_big_loop_threshold; /**< @brief threshold for what is considered a "big" loop (incrementing L is avoided) */

	CLastElementConstrainedMatrixOrdering m_lambda_ordering; /**< @brief lambda block ordering calculator (CAMD wrapper) */
	const size_t *m_p_lambda_block_ordering; /**< @brief lambda block ordering (only valid if m_b_L_up_to_date is set) */ // todo - convert all those to size_t
	size_t m_n_lambda_block_ordering_size; /**< @brief lambda block ordering size */
	const size_t *m_p_lambda_elem_ordering; /**< @brief lambda elementwise ordering (expanded from m_p_lambda_block_ordering) */
	CUberBlockMatrix m_lambda_perm; /**< @brief the reordered reference to the lambda matrix */
	CUberBlockMatrix m_lambda; /**< @brief the lambda matrix (built / updated incrementally) */

	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	Eigen::VectorXd m_v_d; /**< @brief d vector */
	Eigen::VectorXd m_v_perm_temp; /**< @brief temporary storage for the permutation vector, same dimension as d and dx */
	size_t m_n_verts_in_lambda; /**< @brief number of vertices already in lambda */
	size_t m_n_edges_in_lambda; /**< @brief number of edges already in lambda */
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_last_optimized_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_step; /**< @brief counter of incremental steps modulo m_n_nonlinear_solve_threshold */
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_linear_solve_threshold; /**< @brief step threshold for linear solve */
	size_t m_n_nonlinear_solve_threshold; /**< @brief step threshold for nonlinear solve */
	size_t m_n_nonlinear_solve_max_iteration_num; /**< @brief maximal number of iterations in incremental nonlinear solve */
	double m_f_nonlinear_solve_error_threshold; /**< @brief error threshold in incremental nonlinear solve */ // t_odo - document these in elementwise A and L
	bool m_b_verbose; /**< @brief verbosity flag */

	size_t m_n_real_step; /**< @brief counter of incremental steps (no modulo) */

	bool m_b_system_dirty; /**< @brief system updated without relinearization flag */
	bool m_b_linearization_dirty; /**< @brief system matrices updated but relinearization point was not set flag */

	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	double m_f_serial_time; /**< @brief time spent in serial section */
	double m_f_ata_time; /**< @brief time spent in A^T * A section */
	double m_f_premul_time; /**< @brief time spent in b * A section */
	double m_f_chol_time; /**< @brief time spent in Choleski() section */
	double m_f_norm_time; /**< @brief time spent in norm calculation section */

	size_t m_n_full_forwardsubst_num; /**< @brief number of d updates performed using full L forward substitution */
	size_t m_n_resumed_forwardsubst_num; /**< @brief number of d updates performed using resumed L forward substitution */
	size_t m_n_L_optim_num; /**< @brief number of system optimizations performed using L backsubstitution */
	size_t m_n_lambda_optim_num; /**< @brief number of system optimizations performed using cholsol(lambda) */
	size_t m_n_Lup_num; /**< @brief number of L increments */
	size_t m_n_omega_update_num; /**< @brief number of L increments calculated using omega */
	size_t m_n_lambda_update_num; /**< @brief number of L increments calculated using lambda */
	size_t m_n_full_L_num; /**< @brief number of L updates */
	double m_f_ordering_time; /**< @brief time spent calculating ordering of lambda */
	double m_f_fullL_time; /**< @brief time spent in updating L */
	double m_f_l11_omega_calc_time; /**< @brief time spent calculating omega (L increment) */
	double m_f_l11_omega_slice_time; /**< @brief time spent in slicing L11 (L increment) */
	double m_f_l11_omega_ata_time; /**< @brief time spent calculating L11TL11 (L increment) */
	double m_f_l11_omega_add_time; /**< @brief time spent adding L11TL11 + omega (L increment) */
	double m_f_l11_lambda_slice_time; /**< @brief time spent in slicing lambda11 and L01 (L increment) */
	double m_f_l11_lambda_ata_time; /**< @brief time spent calculating L01TL01 (L increment) */
	double m_f_l11_lambda_add_time; /**< @brief time spent adding L01TL01 + lambda11 (L increment) */
	double m_f_lupdate_time; /**< @brief time spent calculating cholesky of new L11 (L increment) */
	double m_f_toupper_time; /**< @brief time spent converting blocky L to elementwise sparse strictly upper-triangular matrix (L increment) */
	double m_f_d_time; /**< @brief time spent updating d (right hand side vector) */
	double m_f_backsubst_time; /**< @brief time spent in backsubstitution (solving for L / d) */
	double m_f_fullL_cholesky; /**< @brief time spent in calculating cholesky (L update) */
	double m_f_toupper2_time; /**< @brief time spent converting blocky L to elementwise sparse strictly upper-triangular matrix (L update) */

#ifndef __NONLINEAR_SOLVER_L_DETAILED_TIMING
	/**
	 *	@brief a fake timer class (frequent timing queries slows down, these go to multithreaded runtime library)
	 */
	class CFakeTimer {
	public:
		/**
		 *	@brief resets the timer (has no effect)
		 */
		inline void Reset() const
		{}

		/**
		 *	@brief gets current time in seconds (always returns zero)
		 *	@return Returns zero.
		 */
		inline double f_Time() const
		{
			return 0;
		}
	};

	CFakeTimer m_timer; /**< @brief timer object */
#else // !__NONLINEAR_SOLVER_L_DETAILED_TIMING
	CTimer m_timer; /**< @brief timer object */
#endif // !__NONLINEAR_SOLVER_L_DETAILED_TIMING

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
	size_t m_n_loop_size_cumsum; /**< @brief cumulative sum of loops processed so far */
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS

	/*size_t m_n_dense_cholesky_num;
	double m_f_dense_cholesky_time, m_f_sparse_cholesky_time;*/

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
	 *	@param[in] b_use_schur is Schur complement trick flag (not supported)
	 *
	 *	@deprecated This is deprecated version of the constructor, use constructor
	 *		with TIncrementalSolveSetting instead.
	 */
	CNonlinearSolver_L(CSystem &r_system, size_t n_linear_solve_threshold,
		size_t n_nonlinear_solve_threshold, size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool UNUSED(b_use_schur) = false)
		:m_r_system(r_system), m_linear_solver(linear_solver),
		m_linear_solver2(linear_solver2),
#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		m_p_L(0),
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		m_b_had_loop_closure(false), m_b_first_iteration_use_L(true), m_b_L_up_to_date(true),
		m_n_last_full_L_update_size(0), m_n_big_loop_threshold(/*(n_dummy_param > 0)? n_dummy_param :*/ 500),
		m_p_lambda_block_ordering(0), m_n_lambda_block_ordering_size(0), m_p_lambda_elem_ordering(0),
		m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_linear_solve_threshold(n_linear_solve_threshold),
		m_n_nonlinear_solve_threshold(n_nonlinear_solve_threshold),
		m_n_nonlinear_solve_max_iteration_num(n_nonlinear_solve_max_iteration_num),
		m_f_nonlinear_solve_error_threshold(f_nonlinear_solve_error_threshold),
		m_b_verbose(b_verbose), m_n_real_step(0), m_b_system_dirty(false),
		m_b_linearization_dirty(false), m_n_iteration_num(0), m_f_serial_time(0), m_f_ata_time(0),
		m_f_premul_time(0), m_f_chol_time(0), m_f_norm_time(0), m_n_full_forwardsubst_num(0),
		m_n_resumed_forwardsubst_num(0), m_n_L_optim_num(0), m_n_lambda_optim_num(0), m_n_Lup_num(0),
		m_n_omega_update_num(0), m_n_lambda_update_num(0), m_n_full_L_num(0), m_f_ordering_time(0),
		m_f_fullL_time(0), m_f_l11_omega_calc_time(0), m_f_l11_omega_slice_time(0),
		m_f_l11_omega_ata_time(0), m_f_l11_omega_add_time(0), m_f_l11_lambda_slice_time(0),
		m_f_l11_lambda_ata_time(0), m_f_l11_lambda_add_time(0), m_f_lupdate_time(0),
		m_f_toupper_time(0), m_f_d_time(0), m_f_backsubst_time(0), m_f_fullL_cholesky(0),
		m_f_toupper2_time(0)
	{
		_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those

		/*printf("hardcoded: ");
		fbs_ut::CDumpBlockMatrixTypelist<_TyAMatrixBlockSizes>::Print();
		printf("from system: ");
		fbs_ut::CDumpBlockMatrixTypelist<typename _TySystem::_TyJacobianMatrixBlockList>::Print();*/
		// debug (now these are the same)

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		m_n_loop_size_cumsum = 0;
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		/*m_n_dense_cholesky_num = 0;
		m_f_dense_cholesky_time = 0;
		m_f_sparse_cholesky_time = 0;*/
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
	 *	@param[in] b_use_schur is Schur complement trick flag (not supported)
	 */
	CNonlinearSolver_L(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool UNUSED(b_use_schur) = false)
		:m_r_system(r_system), m_linear_solver(linear_solver),
		m_linear_solver2(linear_solver2),
#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		m_p_L(0),
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		m_b_had_loop_closure(false), m_b_first_iteration_use_L(true), m_b_L_up_to_date(true),
		m_n_last_full_L_update_size(0), m_n_big_loop_threshold(/*(n_dummy_param > 0)? n_dummy_param :*/ 500),
		m_p_lambda_block_ordering(0), m_n_lambda_block_ordering_size(0), m_p_lambda_elem_ordering(0),
		m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_linear_solve_threshold(t_incremental_config.t_linear_freq.n_period),
		m_n_nonlinear_solve_threshold(t_incremental_config.t_nonlinear_freq.n_period),
		m_n_nonlinear_solve_max_iteration_num(t_incremental_config.n_max_nonlinear_iteration_num),
		m_f_nonlinear_solve_error_threshold(t_incremental_config.f_nonlinear_error_thresh),
		m_b_verbose(b_verbose), m_n_real_step(0), m_b_system_dirty(false),
		m_b_linearization_dirty(false), m_n_iteration_num(0), m_f_serial_time(0), m_f_ata_time(0),
		m_f_premul_time(0), m_f_chol_time(0), m_f_norm_time(0), m_n_full_forwardsubst_num(0),
		m_n_resumed_forwardsubst_num(0), m_n_L_optim_num(0), m_n_lambda_optim_num(0), m_n_Lup_num(0),
		m_n_omega_update_num(0), m_n_lambda_update_num(0), m_n_full_L_num(0), m_f_ordering_time(0),
		m_f_fullL_time(0), m_f_l11_omega_calc_time(0), m_f_l11_omega_slice_time(0),
		m_f_l11_omega_ata_time(0), m_f_l11_omega_add_time(0), m_f_l11_lambda_slice_time(0),
		m_f_l11_lambda_ata_time(0), m_f_l11_lambda_add_time(0), m_f_lupdate_time(0),
		m_f_toupper_time(0), m_f_d_time(0), m_f_backsubst_time(0), m_f_fullL_cholesky(0),
		m_f_toupper2_time(0)
	{
		_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those
		if(t_marginals_config.b_calculate) // not supported at the moment
			throw std::runtime_error("CNonlinearSolver_L does not support marginals calculation");

		/*printf("hardcoded: ");
		fbs_ut::CDumpBlockMatrixTypelist<_TyAMatrixBlockSizes>::Print();
		printf("from system: ");
		fbs_ut::CDumpBlockMatrixTypelist<typename _TySystem::_TyJacobianMatrixBlockList>::Print();*/
		// debug (now these are the same)

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		m_n_loop_size_cumsum = 0;
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		/*m_n_dense_cholesky_num = 0;
		m_f_dense_cholesky_time = 0;
		m_f_sparse_cholesky_time = 0;*/
	}

	/**
	 *	@brief destructor (only required if __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST is defined)
	 */
	inline ~CNonlinearSolver_L()
	{
#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		if(m_p_L)
			cs_spfree(m_p_L);
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
#ifdef __NONLINEAR_SOLVER_L_DETAILED_TIMING
		if(f_total_time > 0)
			printf("solver spent %f seconds in parallelizable section (updating L)\n",
			f_total_time - m_f_serial_time);
		printf("out of which:\n");
		printf("\torder: %f\n", m_f_ordering_time);
		printf("\tfullL: %f (ran " PRIsize " times)\n", m_f_fullL_time, m_n_full_L_num);
		printf("\tout of which:\n");
		printf("\t\t chol: %f\n", m_f_fullL_cholesky);
		printf("\t\tupper: %f\n", m_f_toupper2_time);
		double f_total_omega_up = m_f_l11_omega_calc_time + m_f_l11_omega_slice_time +
			m_f_l11_omega_ata_time + m_f_l11_omega_add_time;
		double f_total_lambda_up = m_f_l11_lambda_slice_time +
			m_f_l11_lambda_ata_time + m_f_l11_lambda_add_time;
		printf("\tL update: %f (ran " PRIsize " times)\n", f_total_omega_up +
			f_total_lambda_up + m_f_lupdate_time, m_n_Lup_num);
		printf("\t\tomega: %f (ran " PRIsize " times)\n", f_total_omega_up, m_n_omega_update_num);
		printf("\t\t\t calc: %f\n", m_f_l11_omega_calc_time);
		printf("\t\t\tslice: %f\n", m_f_l11_omega_slice_time);
		printf("\t\t\t Lata: %f\n", m_f_l11_omega_ata_time);
		printf("\t\t\tL11up: %f\n", m_f_l11_omega_add_time);
		printf("\t\t   ,\\: %f (ran " PRIsize " times)\n", f_total_lambda_up, m_n_lambda_update_num);
		printf("\t\t\tslice: %f\n", m_f_l11_lambda_slice_time);
		printf("\t\t\t Lata: %f\n", m_f_l11_lambda_ata_time);
		printf("\t\t\tL11up: %f\n", m_f_l11_lambda_add_time);
		printf("\t\t  Lup: %f // cholesky and fill\n", m_f_lupdate_time);
		printf("\t\tupper: %f // upper triangular to sparse\n", m_f_toupper_time);
		printf("\t    d: %f (resumed " PRIsize ", full " PRIsize ")\n", m_f_d_time,
			m_n_resumed_forwardsubst_num, m_n_full_forwardsubst_num);
		printf("\tbksub: %f (ran " PRIsize " times)\n", m_f_backsubst_time, m_n_L_optim_num);
		printf("solver spent %f seconds in serial section\n", m_f_serial_time);
		printf("out of which:\n");
		printf("\t  ata: %f\n", m_f_ata_time);
		printf("\tgaxpy: %f\n", m_f_premul_time);
		printf("\t chol: %f (ran " PRIsize " times)\n", m_f_chol_time, m_n_lambda_optim_num);
		printf("\t norm: %f\n", m_f_norm_time);
		printf("\ttotal: %f\n", m_f_ata_time + m_f_premul_time + m_f_chol_time + m_f_norm_time);
		/*printf("in unrelated news, small cholesky ran " PRIsize " times\n", m_n_dense_cholesky_num);
		printf("\t dense: %f\n", m_f_dense_cholesky_time);
		printf("\tsparse: %f\n", m_f_sparse_cholesky_time);*/ // dont want to do it runtime
#endif // __NONLINEAR_SOLVER_L_DETAILED_TIMING
	}

	/**
	 *	@brief calculates \f$\chi^2\f$d error
	 *	@return Returns \f$\chi^2\f$d error.
	 *	@note This only works with systems with edges of one degree of freedom
	 *		(won't work for e.g. systems with both poses and landmarks).
	 */
	inline double f_Chi_Squared_Error() const
	{
		if(m_r_system.r_Edge_Pool().b_Empty())
			return 0;
		if(m_b_linearization_dirty) {
			if(!CalculateOneTimeDx())
				return -1;
			PushValuesInGraphSystem(m_v_dx);
			double f_chi2 = m_r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError()) /
				(m_r_system.r_Edge_Pool().n_Size() - m_r_system.r_Edge_Pool()[0].n_Dimension());
			PushValuesInGraphSystem(-m_v_dx); // !!
			return f_chi2;
		} else {
			return m_r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError()) /
				(m_r_system.r_Edge_Pool().n_Size() - m_r_system.r_Edge_Pool()[0].n_Dimension());
		}
	}

	/**
	 *	@brief calculates denormalized \f$\chi^2\f$d error
	 *	@return Returns denormalized \f$\chi^2\f$d error.
	 *	@note This doesn't perform the final division by (number of edges - degree of freedoms).
	 */
	inline double f_Chi_Squared_Error_Denorm() const
	{
		if(m_r_system.r_Edge_Pool().b_Empty())
			return 0;
		if(m_b_linearization_dirty) {
			typedef CNonlinearSolver_L<CSystem, CLinearSolver, CAMatrixBlockSizes> TThisType;
			TThisType *p_this = const_cast<TThisType*>(this); // don't want to put 'mutable' around everything
			if(!p_this->CalculateOneTimeDx())
				return -1;
			p_this->PushValuesInGraphSystem(m_v_dx);
			double f_chi2 = m_r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError());
			p_this->PushValuesInGraphSystem(-m_v_dx); // !!
			return f_chi2;
		} else
			return m_r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError());
	}

	/**
	 *	@brief incremental optimization function
	 *	@param[in] r_last_edge is the last edge that was added to the system
	 *	@note This function throws std::bad_alloc and std::runtime_error (when L is not-pos-def).
	 */
	void Incremental_Step(_TyBaseEdge &UNUSED(r_last_edge)) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
		typename _TyEdgeMultiPool::_TyConstBaseRef r_edge =
			m_r_system.r_Edge_Pool()[m_r_system.r_Edge_Pool().n_Size() - 1];
		// get a reference to the last edge interface

		size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_had_loop_closure) {
			if(r_edge.n_Vertex_Num() > 1) { // unary factors do not cause classical loop closures
				_ASSERTE(r_edge.n_Vertex_Id(0) != r_edge.n_Vertex_Id(1));
				size_t n_first_vertex = std::min(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1));
				m_b_had_loop_closure = (n_first_vertex < n_vertex_num - 2);
				//_ASSERTE(m_b_had_loop_closure || std::max(r_edge.n_Vertex_Id(0),
				//	r_edge.n_Vertex_Id(1)) == n_vertex_num - 1); // this won't work with const vertices
			} else {
				size_t n_first_vertex = r_edge.n_Vertex_Id(0);
				m_b_had_loop_closure = (n_first_vertex < n_vertex_num - 1);
			}
			// todo - encapsulate this code, write code to support hyperedges as well, use it
			/*if(m_b_had_loop_closure) {
				printf("" PRIsize ", " PRIsize " (out of " PRIsize " and " PRIsize ")\n", n_vertex_num, n_first_vertex,
					r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1));
			}*/ // debug
		}
		// detect loop closures (otherwise the edges are initialized based on measurement and error would be zero)

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		{
			_ASSERTE(r_edge.n_Vertex_Num() == 2); // won't work with hyperedges, would have to modify
			_ASSERTE(std::max(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1)) <
				m_r_system.r_Vertex_Pool().n_Size()); // won't work with const vertices, would have to modify if needed
			FILE *p_fw = fopen("timeSteps_L.txt", (m_n_real_step > 0)? "a" : "w");
			fprintf(p_fw, "" PRIsize ";%f;" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize "\n", m_n_real_step, m_timer.f_Time(),
				m_n_Lup_num, m_n_full_L_num, m_n_L_optim_num, m_n_lambda_optim_num,
				std::max(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1)) -
				std::min(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1)),
				m_n_loop_size_cumsum);
			fclose(p_fw);
		}
		// dump time per step
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS

#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2
		bool b_new_vert = m_lambda.n_BlockColumn_Num() < m_r_system.r_Vertex_Pool().n_Size();

#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
		if(b_new_vert) {
			FILE *p_fw = fopen("chi2perVert.txt", (m_n_real_step > 0)? "a" : "w");
			double f_chi2 = 0;
			do {
				if(m_r_system.r_Edge_Pool().b_Empty())
					break;
				if(m_b_linearization_dirty) {
					typedef CNonlinearSolver_L<CSystem, CLinearSolver, CAMatrixBlockSizes> TThisType;
					TThisType *p_this = const_cast<TThisType*>(this); // don't want to put 'mutable' around everything
					if(!p_this->CalculateOneTimeDx(1)) // this is one smaller
						break;
					p_this->m_r_system.r_Vertex_Pool().For_Each_Parallel(0,
						m_r_system.r_Vertex_Pool().n_Size() - 1, CUpdateEstimates(m_v_dx)); // ignore the last vertex
					f_chi2 = m_r_system.r_Edge_Pool().For_Each(0,
						m_r_system.r_Edge_Pool().n_Size() - 1, CSum_ChiSquareError()); // ignore the last edge
					p_this->m_r_system.r_Vertex_Pool().For_Each_Parallel(0,
						m_r_system.r_Vertex_Pool().n_Size() - 1, CUpdateEstimates(-m_v_dx)); // !!
					break;
				} else {
					f_chi2 = m_r_system.r_Edge_Pool().For_Each(0,
						m_r_system.r_Edge_Pool().n_Size() - 1, CSum_ChiSquareError());
				}
			} while(0);
			// calculate chi2, excluding the last edge (the current one)

			fprintf(p_fw, "%f\n", f_chi2);
			fclose(p_fw);
		}
		// dump chi2
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2

		TryOptimize(m_n_nonlinear_solve_max_iteration_num, m_f_nonlinear_solve_error_threshold);
		// optimize

#ifndef __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2
		if(b_new_vert) {
			FILE *p_fw = fopen("chi2perVert.txt", (m_n_real_step > 0)? "a" : "w");
			fprintf(p_fw, "%f\n", f_Chi_Squared_Error_Denorm());
			fclose(p_fw);
		}
		// dump chi2
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE

#ifdef __NONLINEAR_SOLVER_L_DUMP_DENSITY
		if(b_new_vert) {
			size_t n_nnz_ideal, n_nnz_ideal_elem;
			{
				cs *A = m_lambda.p_Convert_to_Sparse();
				css *S;
				S = cs_schol(1, A); // use AMD
				csn *N = cs_chol(A, S);
				cs *L = N->L;

				n_nnz_ideal_elem = L->p[L->n]; // @todo - count this by bylocks

				CUberBlockMatrix blockL;
				m_lambda.CopyLayoutTo(blockL);
				std::vector<size_t> workspace;
				blockL.From_Sparse(0, 0, L, false, workspace);
				n_nnz_ideal = blockL.n_NonZero_Num();

				cs_sfree(S);
				cs_spfree(N->U);
				cs_free(N->pinv);
				cs_free(N->B);
				cs_spfree(L);
				cs_spfree(A);
				// calculate cholesky with elementwise AMD
			}
			// "ideal" ordering

			size_t n_nnz_blocky, n_nnz_blocky_elem;
			{
				CMatrixOrdering mord;
				const size_t *p_order = mord.p_InvertOrdering(mord.p_BlockOrdering(m_lambda,
					0, 0), m_lambda.n_BlockColumn_Num());
				// unconstrained blocky ordering on lambda

				CUberBlockMatrix lord;
				m_lambda.Permute_UpperTriangular_To(lord, p_order,
					m_lambda.n_BlockColumn_Num(), true);
				// order the matrix

				cs *A = lord.p_Convert_to_Sparse();
				css *S;
				S = cs_schol(0, A); // use AMD
				csn *N = cs_chol(A, S);
				cs *L = N->L;

				n_nnz_blocky_elem = L->p[L->n]; // @todo - count this by bylocks

				CUberBlockMatrix _blockL, blockL;
				m_lambda.CopyLayoutTo(_blockL);
				_blockL.Permute_UpperTriangular_To(blockL, p_order, m_lambda.n_BlockColumn_Num());
				std::vector<size_t> workspace;
				blockL.From_Sparse(0, 0, L, false, workspace);
				n_nnz_blocky = blockL.n_NonZero_Num();

				cs_sfree(S);
				cs_spfree(N->U);
				cs_free(N->pinv);
				cs_free(N->B);
				cs_spfree(L);
				cs_spfree(A);
				// calculate cholesky with natural ordering (= no ordering)
			}
			// blocky ordering

			size_t n_nnz_blocky_constr, n_nnz_blocky_constr_elem;
			{
				CLastElementOrderingConstraint constr;
				const size_t *p_constraint = constr.p_Get(m_lambda.n_BlockColumn_Num());
				CMatrixOrdering mord;
				const size_t *p_order = mord.p_InvertOrdering(mord.p_BlockOrdering(m_lambda,
					p_constraint, m_lambda.n_BlockColumn_Num()), m_lambda.n_BlockColumn_Num());
				// unconstrained blocky ordering on lambda

				CUberBlockMatrix lord;
				m_lambda.Permute_UpperTriangular_To(lord, p_order,
					m_lambda.n_BlockColumn_Num(), true);
				// order the matrix

				cs *A = lord.p_Convert_to_Sparse();
				css *S;
				S = cs_schol(0, A); // use AMD
				csn *N = cs_chol(A, S);
				cs *L = N->L;

				n_nnz_blocky_constr_elem = L->p[L->n]; // @todo - count this by bylocks

				CUberBlockMatrix _blockL, blockL;
				m_lambda.CopyLayoutTo(_blockL);
				_blockL.Permute_UpperTriangular_To(blockL, p_order, m_lambda.n_BlockColumn_Num());
				std::vector<size_t> workspace;
				blockL.From_Sparse(0, 0, L, false, workspace);
				n_nnz_blocky_constr = blockL.n_NonZero_Num();

				cs_sfree(S);
				cs_spfree(N->U);
				cs_free(N->pinv);
				cs_free(N->B);
				cs_spfree(L);
				cs_spfree(A);
				// calculate cholesky with natural ordering (= no ordering)
			}
			// constrained blocky ordering

			size_t n_nnz_actual = m_R.n_NonZero_Num();
			// actual NNZ

			FILE *p_fw = fopen("LDensityByOrdering.txt", (m_n_real_step > 0)? "a" : "w");
			if(!m_n_real_step) {
				fprintf(p_fw, "block-cols;amd;blocky-amd;blocky-constrained-amd;"
					"amd-blocks;blocky-amd-blocks;blocky-constrained-amd-blocks;actual-L-nnz-blocks\n");
			}
			fprintf(p_fw, "" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize "\n", m_lambda.n_BlockColumn_Num(), n_nnz_ideal_elem,
				n_nnz_blocky_elem, n_nnz_blocky_constr_elem, n_nnz_ideal, n_nnz_blocky,
				n_nnz_blocky_constr, n_nnz_actual);
			fclose(p_fw);
		}
		// dump nnz of L, given different ordering strategies
#endif // __NONLINEAR_SOLVER_L_DUMP_DENSITY

		++ m_n_real_step;
		// used only above
	}

protected:
	/**
	 *	@brief optimization function with optimization decision
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when L is not-pos-def).
	 */
	void TryOptimize(size_t n_max_iteration_num, double f_min_dx_norm) // throw(std::bad_alloc, std::runtime_error)
	{
		bool b_optimization_triggered = false;

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		size_t n_new_vertex_num = n_vertex_num - m_n_last_optimized_vertex_num;
		if(!n_new_vertex_num) // evidently, this backfires; need to have another m_n_last_optimized_vertex_num where it would remember when was the system last extended and check if there are new vertices since *then* // fixed now
			return; // no new vertices; don't go in ... (otherwise 2x slower on molson35, for obvious reasons)
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

			b_optimization_triggered = true;
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

			n_max_iteration_num = 1;
			f_min_dx_norm = 20;
			b_optimization_triggered = true;
			// simple optimization
		}

		if(!b_optimization_triggered) {
			if(m_lambda.n_BlockColumn_Num() == n_vertex_num)
				return;
			// there is enough vertices in lambda, none would be added

			Optimize(0, 0);
			// optimize but don't allow iterations - just updates lambda, d and L
			// in order to be able to generate approximate solutions on request
		} else
			Optimize(n_max_iteration_num, f_min_dx_norm);
	}

	/**
	 *	@brief refreshes system matrices lambda and L
	 *	@return Returns true if optimization should take place, otherwise returns false
	 */
	bool RefreshLambdaL()
	{
		const size_t n_variables_size = m_r_system.n_VertexElement_Num();
		const size_t n_measurements_size = m_r_system.n_EdgeElement_Num();
		if(n_variables_size > n_measurements_size) {
			if(n_measurements_size)
				fprintf(stderr, "warning: the system is underspecified\n");
			else
				fprintf(stderr, "warning: the system contains no edges at all: nothing to optimize\n");
			//return false;
		}
		if(!n_measurements_size)
			return false; // nothing to solve (but no results need to be generated so it's ok)
		// can't solve in such conditions

		_ASSERTE(this->m_r_system.b_AllVertices_Covered());
		// if not all vertices are covered then the system matrix will be rank deficient and this will fail
		// this triggers typically if solving BA problems with incremental solve each N steps (the "proper"
		// way is to use CONSISTENCY_MARKER and incremental solve period of SIZE_MAX).

		// note that n_order_min can be possibly used in Refresh_Lambda()
		// to minimize number of vertices that require update of hessian blocks
		// note that it needs to be the one with permutation? or does it? // todo - try that after it works

		Extend_LambdaL(m_n_verts_in_lambda, m_n_edges_in_lambda);
		// recalculated all the jacobians inside Extend_LambdaL(), also extend L structurally

		{
			//double f_refresh_start = m_timer.f_Time();

			if(!m_b_system_dirty)
				Refresh_Lambda(0/*m_n_verts_in_lambda*/, m_n_edges_in_lambda); // calculate only for new edges // @todo - but how to mark affected vertices?
			else
				Refresh_Lambda(); // calculate for entire system, rebuild L from scratch

			/*double f_refresh_end = m_timer.f_Time();
			printf("lambda block width: " PRIsize ", lambda block num: " PRIsize " "
				"(sparsity %.2f %%); update size (edges): " PRIsize " (took %.2f msec)\n",
				m_lambda.n_BlockColumn_Num(), m_lambda.n_Block_Num(),
				float(m_lambda.n_Block_Num() * 9) / (m_lambda.n_Row_Num() *
				m_lambda.n_Column_Num()) * 100, m_lambda.n_BlockColumn_Num() -
				m_n_verts_in_lambda, (f_refresh_end - f_refresh_start) * 1000);*/ // debug
		}
		// refresh lambda (does not fully refresh permutated lambda, even though it is a reference matrix)

		m_v_dx.resize(n_variables_size);
		m_v_perm_temp.resize(n_variables_size);
		if(m_b_L_up_to_date && !m_b_system_dirty)
			m_v_d.conservativeResize(n_variables_size); // b_Refresh_L() also refreshes d (rhs), needs dx as temp // !!
		else
			m_v_d.resize(n_variables_size); // in case we're about to rebuild L from scratch, don't care about contents of d
		// resize the helper vectors

		//static size_t n_last_update_vertices = 0;
		bool m_b_force_ordering_update = false;
		/*if(m_r_system.r_Vertex_Pool().n_Size() >= n_last_update_vertices + 2) {
			m_b_force_ordering_update = true;
			n_last_update_vertices = m_r_system.r_Vertex_Pool().n_Size();
		}*/ // don't
		// force periodical full updates of ordering

		{
			if(!m_b_force_ordering_update && m_b_L_up_to_date && // can increment L only if up to date
			   !m_b_system_dirty) // avoidance of big incremental updates of L is inside b_Refresh_L() - can only decide if ordering is known
				m_b_first_iteration_use_L = b_Refresh_L(0/*m_n_verts_in_lambda*/, m_n_edges_in_lambda); // calculate only for new edges // @todo - but how to mark affected vertices?
			else
				m_b_first_iteration_use_L = b_Refresh_L(); // calculate for entire system, rebuild L from scratch

			m_b_L_up_to_date = m_b_first_iteration_use_L;
			// in case L is not used, it will fall behind
		}
		// refresh L

		m_b_system_dirty = false;
		m_n_verts_in_lambda = m_r_system.r_Vertex_Pool().n_Size();
		m_n_edges_in_lambda = m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
		_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
			m_lambda.n_BlockColumn_Num() == m_r_system.r_Vertex_Pool().n_Size() &&
			m_lambda.n_Column_Num() == n_variables_size);
		_ASSERTE(!m_b_L_up_to_date || (m_R.n_Row_Num() == m_R.n_Column_Num() &&
			m_R.n_BlockColumn_Num() == m_r_system.r_Vertex_Pool().n_Size() &&
			m_R.n_Column_Num() == n_variables_size)); // lambda is square, blocks on either side = number of vertices
		// need to have lambda and perhaps also L

		return true;
	}

	/**
	 *	@brief updates the m_v_dx vector from the current L and d (or lambda eta)
	 *	@param[in] n_ignore_vertices is number of vertices at the end of the system to be ignored
	 *	@return Returns true on success, false on failure (numerical issues).
	 */
	bool CalculateOneTimeDx(size_t n_ignore_vertices = 0)
	{
		_ASSERTE(m_b_linearization_dirty); // this should only be called in case the linearization point was not updated

		if(m_b_L_up_to_date && m_b_first_iteration_use_L) { // Optimize() clears m_b_L_up_to_date but not m_b_first_iteration_use_L at the same time
#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
			_ASSERTE(m_p_L && m_b_L_up_to_date);
			_ASSERTE(m_p_L->m == m_p_L->n && m_p_L->m == n_variables_size);
#else // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
			_ASSERTE(m_b_L_up_to_date);
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
			// we have L and can use it efficiently

			{
				{}
				// no AtA :(

				{}
				// no mul :(

				bool b_cholesky_result;
#if 0
				{
					cs_usolve(m_p_L, &m_v_dx(0)); // dx = L'/d // note this never fails (except if L is null)
					b_cholesky_result = true; // always
				}
#elif  defined(__NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST)
				{
					_ASSERTE(m_p_lambda_elem_ordering);
					PermuteVector(m_p_lambda_elem_ordering, &m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows());
					Eigen::VectorXd vec_copy = m_v_perm_temp;
					_ASSERTE(vec_copy == m_v_perm_temp); // makes sure that the vectors are the same
					b_cholesky_result = m_R.UpperTriangular_Solve_FBS<_TyLambdaMatrixBlockSizes>(&vec_copy(0), vec_copy.rows());
					cs_usolve(m_p_L, &m_v_perm_temp(0)); // dx = L'/d // note this never fails (except if L is null)
					_ASSERTE((vec_copy - m_v_perm_temp).norm() < 1e-7); // makes sure that the vectors are the same
					InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows());
					b_cholesky_result = true; // always
				}
				// L solves with permutation (note that m_v_d is not modified!)
#else // 0
				{
					_ASSERTE(m_p_lambda_elem_ordering);
					PermuteVector(m_p_lambda_elem_ordering, &m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows());
					b_cholesky_result = m_R.UpperTriangular_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
					// dx = L'/d // note this never fails (except if L is null)
					InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows());
				}
				// L solves with permutation (note that m_v_d is not modified!)
#endif // 0
				// calculate cholesky, reuse block ordering if the linear solver supports it

				if(!b_cholesky_result)
					return false;

#ifdef _DEBUG
				for(size_t i = 0, n_variables_size = m_v_dx.rows(); i < n_variables_size; ++ i) {
					if(_isnan(m_v_dx(i))) {
						fprintf(stderr, "warning: p_dx[" PRIsize "] = NaN (file \'%s\', line "
							PRIsize ")\n", i, __FILE__, size_t(__LINE__));
					}
				}
				// detect the NaNs, if any (warn, but don't modify)
#endif // _DEBUG
			}
		} else {
			if(!n_ignore_vertices)
				Collect_RightHandSide_Vector(m_v_dx);
			else {
				m_r_system.r_Vertex_Pool().For_Each_Parallel(0,
					m_r_system.r_Vertex_Pool().n_Size() - n_ignore_vertices,
					CCollect_RightHandSide_Vector(m_v_dx));
			}
			// collects the right-hand side vector

			{
				{}
				// no AtA :(

				{}
				// no mul :(

				bool b_cholesky_result;
#if 1
				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					b_cholesky_result = m_linear_solver.Solve_PosDef(m_lambda, v_eta); // p_dx = eta = lambda / eta
					// dont reuse block ordering
				}
				// lambda is good without permutation (there is one inside and we save copying eta around)
#else // 1
				{
					_ASSERTE(m_p_lambda_elem_ordering);
					PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
					// permute the dx vector to eta (stored in m_v_perm_temp)

					b_cholesky_result = m_linear_solver.Solve_PosDef(m_lambda_perm, m_v_perm_temp); // p_dx = eta = lambda / eta
					// solve the permutated lambda (dont reuse block ordering)

					InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows());
					// permute the result in eta back to bx
				}
#endif // 1
				// calculate cholesky, reuse block ordering if the linear solver supports it

				if(!b_cholesky_result)
					return false;

#ifdef _DEBUG
				for(size_t i = 0, n_variables_size = m_v_dx.rows(); i < n_variables_size; ++ i) {
					if(_isnan(m_v_dx(i))) {
						fprintf(stderr, "warning: p_dx[" PRIsize "] = NaN (file \'%s\', line "
							PRIsize ")\n", i, __FILE__, size_t(__LINE__));
					}
				}
				// detect the NaNs, if any (warn, but don't modify)
#endif // _DEBUG
			}
		}
		// just solve and check NaNs in debug, nothing more
		// can't update timers as some part of pipeline is not run, don't update counters neither

		return true;
		// the result is in m_v_dx
	}

public:
	/**
	 *	@brief final optimization function
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when L is not-pos-def).
	 */
	void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01) // throw(std::bad_alloc, std::runtime_error)
	{
		if(!RefreshLambdaL())
			return;
		// decide whether to optimize or not

		if(!n_max_iteration_num) {
			m_b_linearization_dirty = true;
			return;
		}
		// in case we're not required to optimize, do nothing
		// (the user can still request solution, L is in good shape)

		if(m_b_had_loop_closure)
			m_b_had_loop_closure = false;
		else {
			m_b_linearization_dirty = true;
			return; // nothing to optimize, dx would be zero
		}
		// handle loop closures a bit differently

#if 0
		static bool b_had_lambda_up = false;
		if(m_b_first_iteration_use_L && b_had_lambda_up) {
			b_had_lambda_up = false;
			Check_LLambdaTracking(); // seems to be working now
		}
#endif // 0
		// make sure lambda and L contain the same system

		for(size_t n_iteration = 0; n_iteration < n_max_iteration_num; ++ n_iteration) {
			++ m_n_iteration_num;
			// debug

			if(m_b_verbose) {
				if(n_max_iteration_num == 1)
					printf("\n=== incremental optimization step ===\n\n");
				else
					printf("\n=== nonlinear optimization: iter #" PRIsize " ===\n\n", n_iteration);
			}
			// verbose

			if(m_b_first_iteration_use_L && !n_iteration) {
				++ m_n_L_optim_num;

#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
				_ASSERTE(m_p_L && m_b_L_up_to_date);
				_ASSERTE(m_p_L->m == m_p_L->n && m_p_L->m == n_variables_size);
#else // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
				_ASSERTE(m_b_L_up_to_date);
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
				// we have L and can use it efficiently

				double f_serial_start = m_timer.f_Time();

				{
					{}
					// no AtA :(

					double f_ata_end = f_serial_start;//m_timer.f_Time();

					{}
					// no mul :(

					double f_mul_end = f_ata_end;//m_timer.f_Time();

					bool b_cholesky_result;
#if 0
					{
						cs_usolve(m_p_L, &m_v_dx(0)); // dx = L'/d // note this never fails (except if L is null)
						b_cholesky_result = true; // always
						if(m_b_verbose)
							printf("backsubstitution succeeded\n");
					}
#elif  defined(__NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST)
					{
						_ASSERTE(m_p_lambda_elem_ordering);
						PermuteVector(m_p_lambda_elem_ordering, &m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows());
						Eigen::VectorXd vec_copy = m_v_perm_temp;
						_ASSERTE(vec_copy == m_v_perm_temp); // makes sure that the vectors are the same
						b_cholesky_result = m_R.UpperTriangular_Solve_FBS<_TyLambdaMatrixBlockSizes>(&vec_copy(0), vec_copy.rows());
						cs_usolve(m_p_L, &m_v_perm_temp(0)); // dx = L'/d // note this never fails (except if L is null)
						_ASSERTE((vec_copy - m_v_perm_temp).norm() < 1e-7); // makes sure that the vectors are the same
						InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows());
						b_cholesky_result = true; // always
						if(m_b_verbose)
							printf("backsubstitution succeeded\n");
					}
					// L solves with permutation (note that m_v_d is not modified!)
#else // 0
					{
						_ASSERTE(m_p_lambda_elem_ordering);
						PermuteVector(m_p_lambda_elem_ordering, &m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows());
						b_cholesky_result = m_R.UpperTriangular_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
						// dx = L'/d // note this never fails (except if L is null)

#ifdef _DEBUG
						//m_v_dx.segment(0, std::min(m_v_dx.rows(), 6)).setZero(); // nbpolok fixup

						/*size_t n_uncover_num = 0;
						for(size_t i = 0, n = m_v_dx.rows(); i < n; ++ i) {
							if(std::find(m_p_lambda_elem_ordering, m_p_lambda_elem_ordering + n, i) == m_p_lambda_elem_ordering + n) {
								fprintf(stderr, "error: element " PRIsize " of dx not covered\n", i);
								++ n_uncover_num;
							}
							// makes sure all the elements of the output vector are covered (seems they're not?)
						}
						if(n_uncover_num > 0) {
							fprintf(stderr, "error: " PRIsize " elements of dx (out of " PRIsize ") not covered\n",
								n_uncover_num, m_v_dx.rows());
						}*/ // checked right after building the ordering
#endif //_DEBUG

						InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows());
						if(m_b_verbose) {
							printf("%s", (b_cholesky_result)? "backsubstitution succeeded\n" :
								"backsubstitution failed\n");
						}
					}
					// L solves with permutation (note that m_v_d is not modified!)
#endif // 0
					// calculate cholesky, reuse block ordering if the linear solver supports it

					double f_chol_end = m_timer.f_Time();

#ifdef _DEBUG
					for(size_t i = 0, n_variables_size = m_r_system.n_VertexElement_Num();
					   i < n_variables_size; ++ i) {
						if(_isnan(m_v_dx(i))) {
							fprintf(stderr, "warning: p_dx[" PRIsize "] = NaN (file \'%s\', line "
								PRIsize ")\n", i, __FILE__, size_t(__LINE__));
						}
					}
					// detect the NaNs, if any (warn, but don't modify)
#endif // _DEBUG

					double f_residual_norm = 0;
					if(b_cholesky_result) {
						f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
						if(m_b_verbose)
							printf("residual norm: %.4f\n", f_residual_norm);
					}
					// calculate residual norm

					double f_norm_end = m_timer.f_Time();
					double f_serial_time = f_norm_end - f_serial_start;
					m_f_ata_time += f_ata_end - f_serial_start;
					m_f_premul_time += f_mul_end - f_ata_end;
					m_f_backsubst_time += f_chol_end - f_mul_end;
					m_f_norm_time += f_norm_end - f_chol_end;
					m_f_serial_time += f_serial_time;
					// timing breakup

					if(f_residual_norm <= f_min_dx_norm) {
						m_b_linearization_dirty = true;
						break;
					}
					// in case the error is low enough, quit (saves us recalculating the hessians)

					if(b_cholesky_result) {
						/*printf("just optimized using L\n");*/
						PushValuesInGraphSystem(m_v_dx); // note this kills L
						m_b_system_dirty = true;
						m_b_L_up_to_date = false; // !!

						m_b_linearization_dirty = false;
					}
					// update the system (in parallel)

					if(!b_cholesky_result)
						break;
					// in case cholesky failed, quit
				}
			} else {
				++ m_n_lambda_optim_num;
				// we fall back to lambda

				if(n_iteration && m_b_system_dirty) {
					Refresh_Lambda(); // want only lambda, leave L behind
					m_b_system_dirty = false;
					m_b_L_up_to_date = false; // lambda not dirty anymore, but L still is
				}
				// no need to rebuild lambda, just refresh the values that are being referenced

				Collect_RightHandSide_Vector(m_v_dx);
				// collects the right-hand side vector

				double f_serial_start = m_timer.f_Time();

				{
					{}
					// no AtA :(

					double f_ata_end = f_serial_start;//m_timer.f_Time();

					{}
					// no mul :(

					double f_mul_end = f_ata_end;//m_timer.f_Time();

					bool b_cholesky_result;
#if 1
					{
						Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
						if((/*m_b_first_iteration_use_L &&*/ n_max_iteration_num > 2) ||
						   (!m_b_first_iteration_use_L && n_max_iteration_num > 1)) {
							do {
								if(n_iteration == ((m_b_first_iteration_use_L)? 1 : 0) &&
								   !_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, m_lambda)) {
									b_cholesky_result = false;
									break;
								}
								// prepare symbolic factorization, structure of lambda won't change in the next steps

								b_cholesky_result = _TyLinearSolverWrapper::Solve(m_linear_solver, m_lambda, v_eta);
								// p_dx = eta = lambda / eta
							} while(0);
						} else
							b_cholesky_result = m_linear_solver.Solve_PosDef(m_lambda, v_eta); // p_dx = eta = lambda / eta

						if(m_b_verbose)
							printf("%s", (b_cholesky_result)? "Cholesky succeeded\n" : "Cholesky failed\n");
					}
					// lambda is good without permutation (there is one inside and we save copying eta around)
#else // 1
					{
						_ASSERTE(m_p_lambda_elem_ordering);
						PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
						// permute the dx vector to eta (stored in m_v_perm_temp)

						/*CDebug::Dump_SparseMatrix("lslam7-lambdacsperm.tga",
							cs_symperm(m_lambda.p_Convert_to_Sparse(), (csi*)m_p_lambda_elem_ordering, 1)); // see what cs_symperm() does
						m_lambda_perm.Rasterize("lslam8-lambdaperm.tga");
						m_lambda.Rasterize("lslam9-lambda.tga");*/

						if((/*m_b_first_iteration_use_L &&*/ n_max_iteration_num > 2) ||
						   (!m_b_first_iteration_use_L && n_max_iteration_num > 1)) {
							do {
								if(n_iteration == ((m_b_first_iteration_use_L)? 1 : 0) &&
								   !_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, m_lambda_perm)) {
									b_cholesky_result = false;
									break;
								}
								// prepare symbolic factorization, structure of lambda won't change in the next steps

								b_cholesky_result = _TyLinearSolverWrapper::Solve(m_linear_solver, m_lambda_perm, m_v_perm_temp);
								// p_dx = eta = lambda / eta
							} while(0);
						} else
							b_cholesky_result = m_linear_solver.Solve_PosDef(m_lambda_perm, m_v_perm_temp); // p_dx = eta = lambda / eta
						// solve the permutated lambda

						if(m_b_verbose)
							printf("%s", (b_cholesky_result)? "perm Cholesky succeeded\n" : "perm Cholesky failed\n");

						InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows());
						// permute the result in eta back to bx
					}
#endif // 1
					// calculate cholesky, reuse block ordering if the linear solver supports it

					double f_chol_end = m_timer.f_Time();

#ifdef _DEBUG
					for(size_t i = 0, n_variables_size = m_r_system.n_VertexElement_Num();
					   i < n_variables_size; ++ i) {
						if(_isnan(m_v_dx(i))) {
							fprintf(stderr, "warning: p_dx[" PRIsize "] = NaN (file \'%s\', line "
								PRIsize ")\n", i, __FILE__, size_t(__LINE__));
						}
					}
					// detect the NaNs, if any (warn, but don't modify)
#endif // _DEBUG

					double f_residual_norm = 0;
					if(b_cholesky_result) {
						f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
						if(m_b_verbose)
							printf("residual norm: %.4f\n", f_residual_norm);
					}
					// calculate residual norm

					double f_norm_end = m_timer.f_Time();
					double f_serial_time = f_norm_end - f_serial_start;
					m_f_ata_time += f_ata_end - f_serial_start;
					m_f_premul_time += f_mul_end - f_ata_end;
					m_f_chol_time += f_chol_end - f_mul_end;
					m_f_norm_time += f_norm_end - f_chol_end;
					m_f_serial_time += f_serial_time;
					// timing breakup

					if(f_residual_norm <= f_min_dx_norm) {
						m_b_linearization_dirty = true;
						break;
					}
					// in case the error is low enough, quit (saves us recalculating the hessians)

					if(b_cholesky_result) {
						/*printf("just optimized using lambda\n");*/
						PushValuesInGraphSystem(m_v_dx);
						m_b_system_dirty = true;
						m_b_L_up_to_date = false;

#if 0
						b_had_lambda_up = true; // debug
#endif // 0

						m_b_linearization_dirty = false;
					}
					// update the system (in parallel)

					if(!b_cholesky_result)
						break;
					// in case cholesky failed, quit
				}
			}
		}
	}

protected:
	/**
	 *	@brief function object that calls lambda hessian block allocation for all edges
	 */
	class CAlloc_LambdaLBlocks { // t_odo - L probably only allocates blocks on vertices; retain old version of this functor with lambda only for edges
	protected:
		CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix (out) */
		CUberBlockMatrix &m_r_L; /**< @brief reference to the L matrix (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_lambda is reference to the lambda matrix
		 *	@param[in] r_L is reference to the lambda matrix
		 */
		inline CAlloc_LambdaLBlocks(CUberBlockMatrix &r_lambda, CUberBlockMatrix &r_L)
			:m_r_lambda(r_lambda), m_r_L(r_L)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertexOrEdge is vertex or edge type
		 *	@param[in,out] r_vertex_or_edge is vertex or edge to have hessian blocks allocated in lambda
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertexOrEdge>
		inline void operator ()(_TyVertexOrEdge &r_vertex_or_edge) // throw(std::bad_alloc)
		{
			r_vertex_or_edge.Alloc_HessianBlocks(m_r_lambda);
			r_vertex_or_edge.Alloc_LBlocks(m_r_L); // t_odo - alloc L blocks as well
		}
	};

	/**
	 *	@brief function object that calls L factor block allocation for all vertices
	 */
	class CAlloc_LBlocks {
	protected:
		CUberBlockMatrix &m_r_L; /**< @brief reference to the L matrix (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_L is reference to the lambda matrix
		 */
		inline CAlloc_LBlocks(CUberBlockMatrix &r_L)
			:m_r_L(r_L)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertex is vertex type
		 *	@param[in,out] r_vertex is vertex to have hessian blocks allocated in L
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_vertex) // throw(std::bad_alloc)
		{
			r_vertex.Alloc_LBlocks(m_r_L); // t_odo - alloc L blocks as well
		}
	};

	/**
	 *	@brief function object that calls lambda hessian block allocation for all edges
	 */
	class CAlloc_LambdaBlocks {
	protected:
		CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_lambda is reference to the lambda matrix
		 */
		inline CAlloc_LambdaBlocks(CUberBlockMatrix &r_lambda)
			:m_r_lambda(r_lambda)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertexOrEdge is vertex or edge type
		 *	@param[in,out] r_vertex_or_edge is vertex or edge to have hessian blocks allocated in lambda
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertexOrEdge>
		inline void operator ()(_TyVertexOrEdge &r_vertex_or_edge) // throw(std::bad_alloc)
		{
			r_vertex_or_edge.Alloc_HessianBlocks(m_r_lambda);
		}
	};

	/**
	 *	@brief function object that calls omega hessian block allocation and evaluation for all edges
	 */
	class CCalculateOmega {
	protected:
		CUberBlockMatrix &m_r_omega; /**< @brief reference to the omega matrix (out) */
		size_t m_n_min_elem_order; /**< @brief minimal order of vertex to be filled in omega (in elements) */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_omega is reference to the omega matrix
		 *	@param[in] n_min_elem_order is minimal order of vertex to be filled in omega (in elements)
		 */
		inline CCalculateOmega(CUberBlockMatrix &r_omega, size_t n_min_elem_order)
			:m_r_omega(r_omega), m_n_min_elem_order(n_min_elem_order)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is an edge type
		 *	@param[in] r_edge is edge to have its hessian blocks added to omega
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertexOrEdge>
		inline void operator ()(const _TyVertexOrEdge &r_edge) // throw(std::bad_alloc)
		{
			r_edge.Calculate_Omega(m_r_omega, m_n_min_elem_order);
		}
	};

	/**
	 *	@brief function object that calls b vector calculation for all edges
	 */
	class CCollect_RightHandSide_Vector {
	protected:
		Eigen::VectorXd &m_r_b; /**< @brief reference to the right-hand side vector (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_b is reference to the right-hand side vector
		 */
		inline CCollect_RightHandSide_Vector(Eigen::VectorXd &r_b)
			:m_r_b(r_b)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertex is vertex type
		 *	@param[in,out] r_t_vertex is a vertex to output its part R error vector
		 */
		template <class _TyVertex>
		inline void operator ()(const _TyVertex &r_t_vertex) // throw(std::bad_alloc)
		{
			r_t_vertex.Get_RightHandSide_Vector(m_r_b);
		}
	};

	/**
	 *	@brief calculates the right-hand side vector
	 *	@param[out] r_v_b is the right-hand side vector (allocated by the caller)
	 */
	inline void Collect_RightHandSide_Vector(Eigen::VectorXd &r_v_b)
	{
		m_r_system.r_Vertex_Pool().For_Each_Parallel(CCollect_RightHandSide_Vector(r_v_b)); // can do this in parallel
		// collect b
	}

	/**
	 *	@brief creates the lambda matrix from scratch
	 */
	inline void AddEntriesInSparseSystem() // throw(std::bad_alloc, std::runtime_error)
	{
		// note: don't worry about very large matrices being built at once,
		// this will most likely only be any good for incremental

		m_lambda.Clear();
		{
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_first_vertex_id = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
			size_t n_first_vertex_id = m_r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!m_r_system.r_Vertex_Pool()[n_first_vertex_id].b_IsConstant()); // this one must not be constant
			size_t n_first_vertex_order = m_r_system.r_Vertex_Pool()[n_first_vertex_id].n_Order();
			// get id of the first vertex (usually zero)

			const Eigen::MatrixXd &r_t_uf = m_r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
			if(!m_lambda.Append_Block(Eigen::MatrixXd(r_t_uf.transpose() * r_t_uf),
			   n_first_vertex_order, n_first_vertex_order))
				throw std::bad_alloc();
		}
		// add unary factor (actually UF^T * UF, but it was square-rooted before)

		m_R.Clear();
		/*Eigen::MatrixXd t_uf_L = (r_t_uf.transpose() * r_t_uf).llt().matrixU();
		if(!m_R.Append_Block(t_uf_L, 0, 0))
			throw std::bad_alloc();*/ // UF is kept in lambda, it is propagated to L additively
		// add unary factor to L (actually cholesky(UF^T * UF), but it's the same matrix) // todo - is this right?

		//m_r_system.r_Edge_Pool().For_Each(CAlloc_LambdaLBlocks(m_lambda, m_R)); // no, edges do not add L blocks
		m_r_system.r_Edge_Pool().For_Each(CAlloc_LambdaBlocks(m_lambda));
		m_r_system.r_Vertex_Pool().For_Each(CAlloc_LambdaLBlocks(m_lambda, m_R)); // can stay, there is no ordering to be applied
		// add all the hessian blocks
	}

	/**
	 *	@brief incrementally updates the lambda matrix structure (must not be empty)
	 *
	 *	@param[in] n_skip_vertices is number of vertices before the first vertex that changes
	 *	@param[in] n_skip_edges is number of edges before the first edge that changes
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void UpdateSparseSystem(size_t n_skip_vertices, size_t n_skip_edges) // throw(std::bad_alloc)
	{
		//throw std::runtime_error("UpdateSparseSystem not implemented for L"); // t_odo

		_ASSERTE(m_lambda.n_Row_Num() > 0 && m_lambda.n_Column_Num() == m_lambda.n_Row_Num()); // make sure lambda is not empty
		//m_r_system.r_Edge_Pool().For_Each(n_skip_edges,
		//	m_r_system.r_Edge_Pool().n_Size(), CAlloc_LambdaLBlocks(m_lambda, m_R)); // no, edges do not add L blocks
		m_r_system.r_Edge_Pool().For_Each(n_skip_edges,
			m_r_system.r_Edge_Pool().n_Size(), CAlloc_LambdaBlocks(m_lambda));
		m_r_system.r_Vertex_Pool().For_Each(n_skip_vertices,
			m_r_system.r_Vertex_Pool().n_Size(), CAlloc_LambdaLBlocks(m_lambda, m_R)); // will not work if ordering is applied (but it mostly isn't, the increments follow identity ordering)
		// add the hessian blocks of the new edges
	}

	/**
	 *	@brief incrementally updates the lambda matrix structure (can be empty)
	 *
	 *	@param[in] n_vertices_already_in_lambda is number of vertices before the first vertex that changes
	 *	@param[in] n_edges_already_in_lambda is number of edges before the first edge that changes
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Extend_LambdaL(size_t n_vertices_already_in_lambda, size_t n_edges_already_in_lambda) // throw(std::bad_alloc)
	{
		if(!n_vertices_already_in_lambda && !n_edges_already_in_lambda)
			AddEntriesInSparseSystem(); // works for empty
		else
			UpdateSparseSystem(n_vertices_already_in_lambda, n_edges_already_in_lambda); // does not work for empty
		// create block matrix lambda
	}

#ifdef _DEBUG
	/**
	 *	@brief checks if L == chol(lambda), prints the norm of the difference to stdout
	 */
	void Check_LLambdaTracking() const
	{
		CUberBlockMatrix LtL_upper;
		m_R.PreMultiplyWithSelfTransposeTo(LtL_upper, true);
		//cs *p_L = m_R.p_Convert_to_Sparse();
		cs *p_lam = m_lambda.p_Convert_to_Sparse();
		//cs *p_Lt = cs_transpose(p_L, 1);
		cs *p_LtL = LtL_upper.p_Convert_to_Sparse();//cs_multiply(p_Lt, p_L);
		cs *p_diff = cs_add(p_LtL, p_lam, 1, -1);
		double f_norm = cs_norm(p_diff);
		//cs_spfree(p_L);
		cs_spfree(p_lam);
		//cs_spfree(p_Lt);
		cs_spfree(p_LtL);
		cs_spfree(p_diff);
		// calculate norm (L*L' - lambda)

		printf("L - lambda tracking: %f\n", f_norm);
	}
#endif // _DEBUG

	/**
	 *	@brief calculates the new L11 matrix
	 *
	 *	@param[in] n_order_min is the minimum vertex that changes in L (zero-based index in blocks)
	 *	@param[in] n_order_max is number of column blocks in L (in blocks)
	 *	@param[in] r_L11_new is matrix, containing the new L11, before calculating cholesky of it
	 *
	 *	@note This may modify / damage r_L11_new as it is no longer needed and it is *not* a reference
	 *		to a part of L, in case that enables speed optimizations.
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 *
	 *	@todo Refine code in this function, it's da spaghetti.
	 */
	void Refresh_L11(size_t n_order_min, size_t n_order_max, CUberBlockMatrix &r_L11_new) // throw(std::bad_alloc, std::runtime_error)
	{
#ifdef __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY
#ifndef __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY_USE_OLD_CODE
		_ASSERTE(r_L11_new.n_Row_Num() == r_L11_new.n_Column_Num());
		if(r_L11_new.n_Column_Num() < 150) {
			if(!r_L11_new.Cholesky_Dense_FBS<_TyLambdaMatrixBlockSizes, 15>())
				throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment L");
			// use statically sized matrices up to 30x30, then dynamically allocated matrices up to 150x150

			m_R.From_Matrix(n_order_min, n_order_min, r_L11_new);
			// put r_L11_new to m_R
		} else {
			CUberBlockMatrix L11_old;
			m_R.SliceTo(L11_old, n_order_min, n_order_max, n_order_min, n_order_max, true); // get L11 as well, need to clear the blocks first
			// todo - make a ClearBlocks() function to a) delete or b) memset(0) blocks in a rectangular area

			L11_old.Scale_FBS_Parallel<_TyLambdaMatrixBlockSizes>(0);
			// clears the data in the update area (it would be better to erase the blocks, but there is the fap) // todo - see indices of the blocks in L and see if these could be efficiently erased right now (while keeping the structure)
			// note that even if we thrash memory taken by (some) L11 blocks,
			// it will be recollected once a full update takes place

			if(!m_linear_solver2.Factorize_PosDef_Blocky(m_R, r_L11_new,
			   m_L_row_lookup_table, n_order_min, n_order_min))
				throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment L");
		}
		// the new code, using CUberBlockMatrix::Cholesky_Dense_FBS()
		// note that it does not contain debug checks; don't delete the attrocities below

		if(0) { // connect to the old code
#else // !__NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY_USE_OLD_CODE
#if 1
		if(r_L11_new.n_Row_Num() == 6 && r_L11_new.n_Column_Num() == 6) {
			/*{
				Eigen::Matrix<double, 6, 6> factor;
				r_L11_new.Convert_to_Dense(factor);
				// convert new L11 to a dense matrix

				Eigen::LLT<Eigen::Matrix<double, 6, 6>, Eigen::Upper> cholesky(factor);
				factor = cholesky.matrixU();
				// calculate upper cholesky by eigen

				size_t n_base_row = m_R.n_BlockRow_Base(n_order_min);
				size_t n_base_col = n_base_row;//m_R.n_BlockColumn_Base(n_order_min); // note this is unnecessary, L is symmetric
				m_R.Append_Block(factor.block<3, 3>(0, 0), n_base_row, n_base_col);
				m_R.Append_Block(factor.block<3, 3>(0, 3), n_base_row, n_base_col + 3);
				m_R.Append_Block(factor.block<3, 3>(3, 3), n_base_row + 3, n_base_col + 3);
				// put the factor back to L
			}*/
			// custom cholesky code (only 6x6 matrices with two 3x3 poses)

			{
#ifdef _DEBUG
				CUberBlockMatrix chol0;
				r_L11_new.CopyTo(chol0); // need structure
				chol0.Scale(0);
				std::vector<size_t> row_lookup_table;
				if(!m_linear_solver2.Factorize_PosDef_Blocky(chol0, r_L11_new, row_lookup_table, 0, 0))
					throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment L");
				cs *p_chol0 = chol0.p_Convert_to_Sparse();
				// calculate cholesky with linear solver

				cs *p_L11 = r_L11_new.p_Convert_to_Sparse();
				cs *p_chol1_L = p_SparseCholesky_Preordered(p_L11);
				cs *p_chol1 = cs_transpose(p_chol1_L, 1);
				cs_spfree(p_chol1_L);
				cs_spfree(p_L11);
				// calculate cholesky using csparse
#endif // _DEBUG

				r_L11_new.Cholesky_Dense<6>();
				// calculate dense cholesky inplace

#ifdef _DEBUG
				cs *p_chol2 = r_L11_new.p_Convert_to_Sparse();
				// another cholesky
#endif // _DEBUG

				m_R.From_Matrix(n_order_min, n_order_min, r_L11_new);
				// put r_L11_new to m_R

#ifdef _DEBUG
				CUberBlockMatrix chol2;
				m_R.SliceTo(chol2, n_order_min, m_R.n_BlockColumn_Num(),
					n_order_min, m_R.n_BlockColumn_Num());
				cs *p_chol3 = chol2.p_Convert_to_Sparse();
				// cholesky that is put in L

				{
					cs *p_diff = cs_add(p_chol0, p_chol1, -1, 1);
					_ASSERTE(cs_norm(p_diff) < 1e-7);
					cs_spfree(p_diff);
				}
				{
					cs *p_diff = cs_add(p_chol1, p_chol2, -1, 1);
					_ASSERTE(cs_norm(p_diff) < 1e-7);
					cs_spfree(p_diff);
				}
				{
					cs *p_diff = cs_add(p_chol1, p_chol3, -1, 1);
					_ASSERTE(cs_norm(p_diff) < 1e-7);
					cs_spfree(p_diff);
				}
				// compare all to CSparse
#endif // _DEBUG
			}
			// more general code (likely not much more efficient) // todo test it

			return; // complicate flow - better be safe and not do cholesky on L11 twice // todo
		} else if(r_L11_new.n_Row_Num() == 9 && r_L11_new.n_Column_Num() == 9) {
			r_L11_new.Cholesky_Dense<9>();
			// calculate dense cholesky inplace

			m_R.From_Matrix(n_order_min, n_order_min, r_L11_new);
			// put r_L11_new to m_R

			return; // complicate flow - better be safe and not do cholesky on L11 twice // todo
		} else if(r_L11_new.n_Row_Num() == 12 && r_L11_new.n_Column_Num() == 12) {
			r_L11_new.Cholesky_Dense<12>();
			// calculate dense cholesky inplace

			m_R.From_Matrix(n_order_min, n_order_min, r_L11_new);
			// put r_L11_new to m_R

			return; // complicate flow - better be safe and not do cholesky on L11 twice // todo
		} else if(r_L11_new.n_Row_Num() == 15 && r_L11_new.n_Column_Num() == 15) {
			r_L11_new.Cholesky_Dense<15>();
			// calculate dense cholesky inplace

			m_R.From_Matrix(n_order_min, n_order_min, r_L11_new);
			// put r_L11_new to m_R

			return; // complicate flow - better be safe and not do cholesky on L11 twice // todo
		} else {
#else // 0
		{
#endif // 0
#if defined(_WIN32) || defined(_WIN64)
			if(r_L11_new.n_Row_Num() < 150 && r_L11_new.n_Column_Num() < 150) {
				//double f_chol_start = m_timer.f_Time();

#ifdef _DEBUG
				CUberBlockMatrix chol0;
				r_L11_new.CopyTo(chol0);
				chol0.Scale(0);
				std::vector<size_t> row_lookup_table;
				if(!m_linear_solver2.Factorize_PosDef_Blocky(chol0, r_L11_new, row_lookup_table, 0, 0))
					throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment L");
				cs *p_chol0 = chol0.p_Convert_to_Sparse();
				// calculate cholesky with linear solver

				cs *p_L11 = r_L11_new.p_Convert_to_Sparse();
				cs *p_chol1_L = p_SparseCholesky_Preordered(p_L11);
				cs *p_chol1 = cs_transpose(p_chol1_L, 1);
				cs_spfree(p_chol1_L);
				cs_spfree(p_L11);
				// calculate cholesky using csparse
#endif // _DEBUG

				r_L11_new.Cholesky_Dense<Eigen::Dynamic>();
				// calculate dense cholesky inplace (in a dynamically allocated matrix)

#ifdef _DEBUG
				cs *p_chol2 = r_L11_new.p_Convert_to_Sparse();
				// another cholesky
#endif // _DEBUG

				m_R.From_Matrix(n_order_min, n_order_min, r_L11_new);
				// put r_L11_new to m_R

#ifdef _DEBUG
				CUberBlockMatrix chol3;
				m_R.SliceTo(chol3, n_order_min, m_R.n_BlockColumn_Num(),
					n_order_min, m_R.n_BlockColumn_Num());
				cs *p_chol3 = chol3.p_Convert_to_Sparse();
				// cholesky that is put in L

				{
					cs *p_diff = cs_add(p_chol0, p_chol1, -1, 1);
					double f_diff = cs_norm(p_diff);
					_ASSERTE(f_diff < 1e-7);
					cs_spfree(p_diff);
				}
				{
					cs *p_diff = cs_add(p_chol1, p_chol2, -1, 1);
					double f_diff = cs_norm(p_diff);
					_ASSERTE(f_diff < 1e-7);
					cs_spfree(p_diff);
				}
				{
					cs *p_diff = cs_add(p_chol1, p_chol3, -1, 1);
					double f_diff = cs_norm(p_diff);
					_ASSERTE(f_diff < 1e-7);
					cs_spfree(p_diff);
				}
				// compare all to CSparse
#endif // _DEBUG

				/*double f_dchol_end = m_timer.f_Time();

				{
					CUberBlockMatrix L11_old;
					m_R.SliceTo(L11_old, n_order_min, n_order_max, n_order_min, n_order_max, true); // get L11 as well, need to clear the blocks first
					// todo - make a ClearBlocks() function to a) delete or b) memset(0) blocks in a rectangular area

					L11_old.Scale_FBS_Parallel<_TyLambdaMatrixBlockSizes>(0);
					// clears the data in the update area (it would be better to erase the blocks, but there is the fap) // todo - see indices of the blocks in L and see if these could be efficiently erased right now (while keeping the structure)
					// note that even if we thrash memory taken by (some) L11 blocks,
					// it will be recollected once a full update takes place

					if(!m_linear_solver2.Factorize_PosDef_Blocky(m_R, r_L11_new, m_L_row_lookup_table, n_order_min, n_order_min))
						throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment L");
				}
				// and sparse

				double f_schol_end = m_timer.f_Time();

				++ m_n_dense_cholesky_num;
				m_f_dense_cholesky_time += f_dchol_end - f_chol_start;
				m_f_sparse_cholesky_time += f_schol_end - f_dchol_end;
#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
				{
					static bool b_first = true;
					FILE *p_fw = fopen("dense_cholesky.csv", (b_first)? "w" : "a");
					b_first = false;
					fprintf(p_fw, "" PRIsize ";%f;%f\n", m_n_dense_cholesky_num - 1,
						f_dchol_end - f_chol_start, f_schol_end - f_dchol_end);
					fclose(p_fw);
				}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
				// statistics, statistics ...

				return; // complicate flow - better be safe and not do cholesky on L11 twice // todo
			}
#endif // _WIN32 || _WIN64
#endif // !__NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY_USE_OLD_CODE
#else // __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY
		{
#endif // __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY
			CUberBlockMatrix L11_old;
			m_R.SliceTo(L11_old, n_order_min, n_order_max, n_order_min, n_order_max, true); // get L11 as well, need to clear the blocks first
			// todo - make a ClearBlocks() function to a) delete or b) memset(0) blocks in a rectangular area

			L11_old.Scale_FBS_Parallel<_TyLambdaMatrixBlockSizes>(0);
			// clears the data in the update area (it would be better to erase the blocks, but there is the fap) // todo - see indices of the blocks in L and see if these could be efficiently erased right now (while keeping the structure)
			// note that even if we thrash memory taken by (some) L11 blocks,
			// it will be recollected once a full update takes place

			if(!m_linear_solver2.Factorize_PosDef_Blocky(m_R, r_L11_new, m_L_row_lookup_table, n_order_min, n_order_min))
				throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment L");
		}
	}

	/**
	 *	@brief calculates the new L matrix incrementally using lambda or omega
	 *
	 *	@param[in] n_refresh_from_edge is the first edge that changes
	 *	@param[in] n_order_min is the minimum vertex that changes in L (zero-based index in blocks)
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	inline void Refresh_L_IncL11(size_t n_refresh_from_edge, size_t n_order_min) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(m_b_L_up_to_date);
		// make sure L is up to date with lambda and we can actually increment

		_ASSERTE(n_refresh_from_edge > 0);
		// make sure that lambda didn't rebuild the ordering completely

		++ m_n_Lup_num;

		const size_t n_order_max = m_lambda.n_BlockColumn_Num();
		// makes sure that max is fixed at the end

		bool b_identity_perm = true;
		for(size_t i = n_order_min; i < n_order_max; ++ i) {
			if(m_p_lambda_block_ordering[i] != i) {
				b_identity_perm = false;
				break;
			}
		}
		const bool b_is_identity_perm = b_identity_perm; // make a copy
		//if(!b_Is_PoseOnly_SLAM)
		//	b_identity_perm = false; // don't allow omega upd for VP // it *is* buggy
		// see if the ordering is identity ordering

#ifndef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		if(n_order_max - n_order_min >= 88) // disable this for timing bench
			b_identity_perm = false;
#endif // !__NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		// yet another threshold

		double f_omega_start = m_timer.f_Time();

		bool b_omega_available = b_identity_perm;
		//b_identity_perm = true; // i want both branches to run

		double f_omega_up_calc_end, f_omega_up_slice_end, f_omega_up_ata_end, f_omega_up_end; // times inside omega update
		CUberBlockMatrix L11TL11;
		if(b_identity_perm) {
			++ m_n_omega_update_num;
			// count them

			CUberBlockMatrix omega, L11;
			size_t n_elem_order_min = m_lambda_perm.n_BlockColumn_Base(n_order_min);
			m_r_system.r_Edge_Pool().For_Each(n_refresh_from_edge, m_r_system.r_Edge_Pool().n_Size(),
				CCalculateOmega(omega, n_elem_order_min));
			omega.CheckIntegrity();

			// a <- pointer to 2x2 block
			// b <- pointer to 3x3 block

			// write 2x2 block -> a
			// write 3x3 block -> b
			// as it should be

			// write 2x2 block -> b (ok, there is empty space left in b)
			// write 3x3 block -> a (should not be ok, should overrun)
			// when it swaps the pointers in the wrong way

			//omega.Rasterize("omega.tga"); // debug

			/*omega.Scale(0); // see if there is an error in doing initialization
			m_r_system.r_Edge_Pool().For_Each(n_refresh_from_edge, m_r_system.r_Edge_Pool().n_Size(),
				CCalculateOmega(omega, n_elem_order_min));

			omega.Rasterize("omega_reiter.tga"); // debug*/
			// nope, not error of such kind

			f_omega_up_calc_end = m_timer.f_Time();

			m_R.SliceTo(L11, n_order_min, n_order_max, n_order_min, n_order_max, true); // row(0 - min) x col(min - max)
			// calculate the omega matrix (ho, ho, ho) and slice L11

			f_omega_up_slice_end = m_timer.f_Time();

			// ordering is bad, edges between n_refresh_from_edge and the end might not be all edges that are needed (actually not, omega should only contain new edges the system is incremented with)
			// ordering is double bad, it needs to be directly inside CCalculateOmega() to make any sense (probably should be)
			// todo - calculate required size of lambda11
			// todo - detect cases with identity ordering and verify correctness

			if(n_order_max - n_order_min >= __NONLIVEAR_SOLVER_L_PARALLEL_MATMULT_THRESH) // big one // t_odo this never runs, the limit for using L is also 100
				L11.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyLambdaMatrixBlockSizes>(L11TL11, true); // calculate L11^T * L11 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
			else
				L11.PreMultiplyWithSelfTransposeTo_FBS<_TyLambdaMatrixBlockSizes>(L11TL11, true); // calculate L11^T * L11 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
			// calculate L11TL11

			f_omega_up_ata_end = m_timer.f_Time();

			bool UNUSED(b_result) = omega.AddTo_FBS<_TyLambdaMatrixBlockSizes>(L11TL11); // todo - maybe also parallel
			_ASSERTE(b_result); // if the block order in omega was wrong, this would fail
			// calculate L11TL11_new = L11TL11 + omega
			// note this uses faster addition algorithm

			f_omega_up_end = m_timer.f_Time();
		} else {
			f_omega_up_calc_end = f_omega_start;
			f_omega_up_slice_end = f_omega_start;
			f_omega_up_ata_end = f_omega_start;
			f_omega_up_end = f_omega_start;
			// omega update not performed - zero time
		}

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		b_identity_perm = false; // i want both branches to run
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES

		double f_lambda_up_slice_end, f_lambda_up_ata_end, f_lambda_up_end; // times inside lambda update
		CUberBlockMatrix L01TL01;
		if(!b_identity_perm) {
			CUberBlockMatrix lambda11, L01;
			++ m_n_lambda_update_num;
			// count them

			m_R.SliceTo(L01, 0, n_order_min, n_order_min, n_order_max, true); // row(0 - min) x col(min - max)
			m_lambda_perm.SliceTo(lambda11, n_order_min, n_order_max, n_order_min, n_order_max, true);

			/*printf("will SliceTo(" PRIsize ", " PRIsize ", " PRIsize ", " PRIsize ")\n", 0, n_order_min, n_order_min, n_order_max);
			printf("L is " PRIsize " x " PRIsize " (" PRIsize " x " PRIsize ")\n", m_R.n_BlockRow_Num(),
				m_R.n_BlockColumn_Num(), m_R.n_Row_Num(), m_R.n_Column_Num());*/ // debug
			// get slices of lambda_perm (equal to lambda11) and of L. will carry update on these
			// note that both L01 and lambda11 are references only, can't be destination for most of the ops!

#ifdef _DEBUG
/*#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
			bool b_write_out = false;//m_R.n_BlockRow_Num() == 3;
			if(b_write_out) {
				{
					FILE *p_fw = fopen("RbeforeUpd.txt", "w");
					CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, m_R.p_Convert_to_Sparse()); // leaks memory
					fclose(p_fw);
				}
				{
					FILE *p_fw = fopen("R01.txt", "w");
					CUberBlockMatrix L01copy;
					L01.SliceTo(L01copy, 0, L01.n_BlockRow_Num(), 0, L01.n_BlockColumn_Num());
					CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, L01copy.p_Convert_to_Sparse()); // leaks memory
					fclose(p_fw);
				}
				{
					FILE *p_fw = fopen("lambda-perm11.txt", "w");
					CUberBlockMatrix lambda11copy;
					lambda11.SliceTo(lambda11copy, 0, lambda11.n_BlockRow_Num(), 0, lambda11.n_BlockColumn_Num());
					CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, lambda11copy.p_Convert_to_Sparse()); // leaks memory
					fclose(p_fw);
				}
				{
					FILE *p_fw = fopen("lambda-perm.txt", "w");
					CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, m_lambda_perm.p_Convert_to_Sparse()); // leaks memory
					fclose(p_fw);
				}
			}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS

			L01.Rasterize("lslam0_L01.tga");
			lambda11.Rasterize("lslam1_lambda11.tga");*/
#endif // _DEBUG

			f_lambda_up_slice_end = m_timer.f_Time();

			/*if(L01.n_Storage_Size() == 0) {
				printf("R01 (" PRIsize " x " PRIsize " blocks) storage is empty (" PRIsize " blocks really inside)\n",
					L01.n_BlockRow_Num(), L01.n_BlockColumn_Num(), L01.n_Block_Num());
				// @todo - ommit the calculation
			} else {
				printf("R01 (" PRIsize " x " PRIsize " blocks) storage is not empty (" PRIsize " blocks really inside)\n",
					L01.n_BlockRow_Num(), L01.n_BlockColumn_Num(), L01.n_Block_Num());
			}*/

			if(n_order_max - n_order_min >= __NONLIVEAR_SOLVER_L_PARALLEL_MATMULT_THRESH) // big one // t_odo this never runs, the limit for using L is also 100
				L01.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyLambdaMatrixBlockSizes>(L01TL01, true); // calculate L01^T * L01 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
			else
				L01.PreMultiplyWithSelfTransposeTo_FBS<_TyLambdaMatrixBlockSizes>(L01TL01, true); // calculate L01^T * L01 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?

#ifdef _DEBUG
			/*L01TL01.Rasterize("lslam2_L01TL01.tga");

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
			if(b_write_out) {
				{
					FILE *p_fw = fopen("R01TR01.txt", "w");
					CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, L01TL01.p_Convert_to_Sparse()); // leaks memory
					fclose(p_fw);
				}
			}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
#endif // _DEBUG

			f_lambda_up_ata_end = m_timer.f_Time();

			lambda11.AddTo_FBS<_TyLambdaMatrixBlockSizes>(L01TL01, -1, 1); // t_odo - use FBS // todo - maybe also parallel
			// calculates L01TL01 = -L01TL01 + lambda11 (note the "-1, 1" is correct, the opposite way it crashes)
			// note that lambda11 is upper diagonal, as well as L01TL01
			// todo - the factors could be optimized away, if it's just +- 1 (more templates, but only slightly, like 6 lines of code per variant of AddTo())
			// todo - it might be faster to do a copy of lambda11 and do L01TL01.AddTo(lambda11, -1); (can't be done now, would kill lambda))

			f_lambda_up_end = m_timer.f_Time();
		} else {
			f_lambda_up_slice_end = f_omega_up_end;
			f_lambda_up_ata_end = f_omega_up_end;
			f_lambda_up_end = f_omega_up_end;
			// lambda update not taken
		}

#if 0 // def _DEBUG
		cs *p_lam_upd = L01TL01.p_Convert_to_Sparse();
		cs *p_omg_upd = L11TL11.p_Convert_to_Sparse();
		cs *p_diff = cs_add(p_lam_upd, p_omg_upd, -1, 1);
		double f_diff = cs_norm(p_diff);
		cs_spfree(p_diff);
		cs_spfree(p_omg_upd);
		cs_spfree(p_lam_upd);
		//if(f_diff > 1e-5) {
			fprintf(stderr, "lambda - omage update tracking: %f (loop size " PRIsize ")\n",
				f_diff, n_order_max - n_order_min); // see all
		//}
		// make sure that lambda and omega updates produce the same thing

/*#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		if(b_write_out) {
			{
				FILE *p_fw = fopen("lambda11 - R01TR01.txt", "w");
				CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, L01TL01.p_Convert_to_Sparse()); // leaks memory
				fclose(p_fw);
			}
		}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
#endif // _DEBUG

		// need to calculate L11 = cholesky(lambda11 - L10 * L10T)
		// the French code needed to calculate L11 = cholesky(L11T * L11 + lambda11) - found out not to be correct

		//double f_l11_omega_time = f_omega_up_end - f_omega_start;
		//double f_l11_lambda_time = f_lambda_up_end - f_omega_up_end; // unused
/*#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		{
			static bool b_first_iter = true;
			FILE *p_fw = fopen("lambda_omega_real.txt", (b_first_iter)? "w" : "a");
			b_first_iter = false;
			fprintf(p_fw, "%f;%f;" PRIsize ";%f;\n", // "incremental L update time(lambda: %f, omega: %f, loop size: " PRIsize ")\n"
				f_l11_lambda_time, (b_omega_available)? f_l11_omega_time :
				f_l11_lambda_time, n_order_max - n_order_min, f_l11_omega_time);
			fclose(p_fw);
		}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
		// get some more interesting results

#if 0
		if(b_omega_available) {
			_ASSERTE(L11TL11.b_EqualLayout(L01TL01));
			cs *p_l11tl11 = L11TL11.p_Convert_to_Sparse();
			cs *p_l01tl01 = L01TL01.p_Convert_to_Sparse();
			cs *p_diff = cs_add(p_l11tl11, p_l01tl01, -1, 1);
			double f_diff;
			if((f_diff = cs_norm(p_diff)) > 1e-5) {
				printf("warning: on lambda update " PRIsize "\n", m_n_lambda_update_num);
				printf("warning: norm(lambda - omega) = %f\n", f_diff);
				cs_droptol(p_diff, 1e-7);
				CDebug::Dump_SparseMatrix("incL11up_diff.tga", p_diff);
				L11TL11.Rasterize("incL11up_omega.tga");
				L01TL01.Rasterize("incL11up_lambda.tga");
				if(n_order_min > 0)
					printf("order[" PRIsize "] = " PRIsize "\n", n_order_min - 1, m_p_lambda_block_ordering[n_order_min - 1]);
				printf("order[" PRIsize ":" PRIsize "] = {" PRIsize "", n_order_min, n_order_max,
					m_p_lambda_block_ordering[n_order_min]);
				for(size_t i = n_order_min + 1; i < n_order_max; ++ i)
					printf(", " PRIsize "", m_p_lambda_block_ordering[i]);
				printf("}\n");
				// t_odo - spit ordering, rasterize matrices
			}
			cs_spfree(p_diff);
			cs_spfree(p_l11tl11);
			cs_spfree(p_l01tl01);
		}
#endif // 0
		// not buggy anymore

		Refresh_L11(n_order_min, n_order_max, (b_omega_available)? L11TL11 : L01TL01);

		double f_factor_update_end = m_timer.f_Time();

#ifdef _DEBUG
/*#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		if(b_write_out) {
			{
				FILE *p_fw = fopen("Rnew.txt", "w");
				CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, m_R.p_Convert_to_Sparse_UpperTriangular()); // leaks memory
				fclose(p_fw);
			}
			exit(0);
		}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS

		m_R.Rasterize("lslam4_L.tga");*/
#endif // _DEBUG

#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		m_p_L = m_R.p_Convert_to_Sparse_UpperTriangular(m_p_L);
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST

#ifdef _DEBUG
		//CDebug::Dump_SparseMatrix("lslam5_Lud.tga", m_p_L); // see to it that it is truly upper diagonal only

		/*static size_t n_step = 0;
		if(++ n_step == 2)
		exit(0);*/

		/*{
			cs *p_L_ref = m_R.p_Convert_to_Sparse_UpperDiagonal_Debug();
			_ASSERTE(m_p_L->m == m_p_L->n); // supposed to be square
			_ASSERTE(m_p_L->m == p_L_ref->m && m_p_L->n == p_L_ref->n); // supposed to have same dimensions
			bool b_columns_different = memcmp(p_L_ref->p, m_p_L->p, (m_p_L->n + 1) * sizeof(csi)) != 0;
			bool b_rows_different = memcmp(p_L_ref->i, m_p_L->i, m_p_L->p[m_p_L->n] * sizeof(csi)) != 0;
			bool b_data_different = memcmp(p_L_ref->x, m_p_L->x, m_p_L->p[m_p_L->n] * sizeof(double)) != 0;
			_ASSERTE(!b_columns_different && !b_rows_different && !b_data_different);
			cs_spfree(p_L_ref);
		}*/
#endif // _DEBUG

		double f_sparse_end = m_timer.f_Time();

		// note that L and Lambda should be equal structurally (same size, the same block layout, except that lambda is symmetric and L is upper/lower diagonal)

		// @todo - calculate LTL and compare to lambda (that's the way it's supposed to equal, right?)

		// omega is required to build L, HtR is required to build d. so far we're good with just omega
		// omega is build from an edge, exactly the same way lambda is built (omega is just a submatrix of lambda - could use the same code / data / storage)
		// watch out! lambda is actually *not* equal to omega, omega is only for one or more edges, the off-diagonal elements that correspond to vertices have sums from other edges than just those being updated - the update process needs to be carried out differently, or we need to figure out how to add only contributions for edges that are being added : a simple id/order range condition would take care of that

		// todo - if we want lambda, we need to recalc sums of blocks in vertices
		// (not much work, can use inverse condition and just add missing contributions, without clearing)
		// right now we have omega

		// evidently, to do the following, one needs to have proper lambda (not just omega, the vertices calculate with it)

		{
#if 0
			++ m_n_full_forwardsubst_num;

			Collect_RightHandSide_Vector(m_v_dx);
			// collects the right-hand side vector (eta)

			cs_utsolve(m_p_L, &m_v_dx(0)); // d = eta = eta/L
			m_v_d = m_v_dx; // !!
#elif defined(__NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST)
			++ m_n_full_forwardsubst_num;

			Collect_RightHandSide_Vector(m_v_dx);
			// collects the right-hand side vector (eta)

			_ASSERTE(m_p_lambda_elem_ordering);
			PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
			Eigen::VectorXd vec_copy = m_v_perm_temp;
			m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&vec_copy(0), vec_copy.rows());
			cs_utsolve(m_p_L, &m_v_perm_temp(0)); // d = eta = eta/L
			_ASSERTE((vec_copy - m_v_perm_temp).norm() < 1e-7); // makes sure that the vectors are the same
			InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows());
#elif !defined(__NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST)
			++ m_n_full_forwardsubst_num;

			Collect_RightHandSide_Vector(m_v_dx);
			// collects the right-hand side vector (eta)

			_ASSERTE(m_v_d.rows() == m_lambda.n_Column_Num());
			Eigen::VectorXd d_copy = m_v_d;

			//d_copy.conservativeResize(m_v_dx.rows()); // no need
			size_t n_elem_order_min = m_lambda.n_BlockColumn_Base(n_order_min);
			_ASSERTE(m_v_dx.rows() == m_lambda.n_Column_Num()); // should match
			//d_copy.segment(0, n_elem_order_min) = m_v_d.segment(0, n_elem_order_min); // ??
			d_copy.segment(n_elem_order_min, m_v_dx.rows() - n_elem_order_min) =
				m_v_dx.segment(n_elem_order_min, m_v_dx.rows() - n_elem_order_min);
			// copy a part of eta into D

			PermuteVector(m_p_lambda_elem_ordering, &d_copy(0), &m_v_perm_temp(0), m_v_dx.rows());
			m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(
				&m_v_perm_temp(0), m_v_perm_temp.rows(), n_order_min);
			// d = eta = eta/L
			InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &d_copy(0), m_v_dx.rows());
			// "resumed forward substitution"

			_ASSERTE(m_p_lambda_elem_ordering);
			PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
			m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
			// d = eta = eta/L
			InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows());

			if(b_is_identity_perm) {
				_ASSERTE((d_copy - m_v_d)/*.segment(0, n_elem_order_min)*/.norm() < 1e-7);
				//_ASSERTE(0);
			}
			// t_odo - do resumed forward substitution (decide where to resume, collect only a part of eta and utsolve only on that part)
#else // 0
			if(b_is_identity_perm) {
				++ m_n_resumed_forwardsubst_num;

				_ASSERTE(m_v_d.rows() == m_lambda.n_Column_Num());
				m_r_system.r_Vertex_Pool().For_Each_Parallel(n_order_min,
					m_r_system.r_Vertex_Pool().n_Size(), CCollect_RightHandSide_Vector(m_v_d));
				// collect part of b to the lower part of d (this is inside of Collect_RightHandSide_Vector())

				PermuteVector(m_p_lambda_elem_ordering, &m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows());
				m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(
					&m_v_perm_temp(0), m_v_perm_temp.rows(), n_order_min);
				// d = eta = eta/L
				InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows());
				// "resumed forward substitution"

				//m_v_dx.segment(0, 6).setZero(); // this is also enough
				//m_v_dx.setZero();
				// nbpolok fixup
			} else {
				++ m_n_full_forwardsubst_num;

				Collect_RightHandSide_Vector(m_v_dx);
				// collects the right-hand side vector (eta)

				_ASSERTE(m_p_lambda_elem_ordering);
				PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
				m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
				// d = eta = eta/L
				InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows());
				// standard forward substitution // todo - figure out how to resume this one
			}
#endif // 0
			// convert eta to d
		}
		// todo - update d incrementally as well

		double f_d_end = m_timer.f_Time();

		//m_f_ordering_time -= f_refresh_start - f_omega_start;

		m_f_l11_omega_calc_time += f_omega_up_calc_end - f_omega_start;
		m_f_l11_omega_slice_time += f_omega_up_slice_end - f_omega_up_calc_end;
		m_f_l11_omega_ata_time += f_omega_up_ata_end - f_omega_up_slice_end;
		m_f_l11_omega_add_time += f_omega_up_end - f_omega_up_ata_end;

		m_f_l11_lambda_slice_time += f_lambda_up_slice_end - f_omega_up_end;
		m_f_l11_lambda_ata_time += f_lambda_up_ata_end - f_lambda_up_slice_end;
		m_f_l11_lambda_add_time += f_lambda_up_end - f_lambda_up_ata_end;
		m_f_lupdate_time += f_factor_update_end - f_lambda_up_end;
		m_f_toupper_time += f_sparse_end - f_factor_update_end;
		m_f_d_time += f_d_end - f_sparse_end;

		//printf("-");
		/*if(n_order_max - n_order_min > m_n_big_loop_threshold)
			fprintf(stderr, "i don't believe what i'm seeing!\n"); // a jewish anecdote reference
		printf("L block width: " PRIsize ", L block num: " PRIsize " (sparsity %.2f %%); update size: " PRIsize " (took %.2f msec (without d))\n",
			m_R.n_BlockColumn_Num(), m_R.n_Block_Num(), float(m_R.n_Block_Num() * 9) /
			(m_R.n_Row_Num() * m_R.n_Column_Num()) * 100,
			n_order_max - n_order_min, (f_sparse_end/ *f_d_end* / - f_refresh_start) * 1000);*/ // debug
		//printf("building lambda from scratch finished\n"); // debug
	}

	/**
	 *	@brief calculates the new L matrix from scratch as Cholesky of lambda
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	inline void Refresh_L_FullL() // throw(std::bad_alloc, std::runtime_error)
	{
		m_n_last_full_L_update_size = m_lambda.n_BlockColumn_Num();
		// L will have the same size once updated ...

		++ m_n_full_L_num;
		// L is not up to date, need to rebuild from scratch

#ifdef _DEBUG
/*#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		bool b_write_out = m_lambda.n_BlockRow_Num() == 2;
		if(b_write_out) {
			{
				FILE *p_fw = fopen("0-lambda11.txt", "w"); // lambda11 is all of lambda
				CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, m_lambda.p_Convert_to_Sparse()); // leaks memory
				fclose(p_fw);
			}
		}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
#endif // _DEBUG

		double f_sparse_end = m_timer.f_Time();

		if(b_Is_PoseOnly_SLAM) { // this is known at compile-time, should optimize the unused branch away
			m_R.Clear();
			m_r_system.r_Vertex_Pool().For_Each(CAlloc_LBlocks(m_R)); // won't work with VP problems, need to set correct ordering to vertices
		} else {
			//m_R.Clear(); // already inside PermuteTo()
			CUberBlockMatrix t_new_L;
			m_r_system.r_Vertex_Pool().For_Each(CAlloc_LBlocks(t_new_L));
			t_new_L.PermuteTo(m_R, m_p_lambda_block_ordering, m_n_lambda_block_ordering_size);
		}
		// do the right thing and thrash L

		if(!m_linear_solver2.Factorize_PosDef_Blocky(m_R, m_lambda_perm, m_L_row_lookup_table, 0, 0))
			throw std::runtime_error("Factorize_PosDef_Blocky() failed to calculate full L");
		// factorize (uses cached cholesky, saves some time on allocation of workspace memory)

		double f_cholesky_end = m_timer.f_Time();

#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
		m_p_L = m_R.p_Convert_to_Sparse_UpperTriangular(m_p_L); // !!
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST

		double f_sparse2_end = m_timer.f_Time();

#ifdef _DEBUG
/*#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		if(b_write_out) {
			{
				FILE *p_fw = fopen("0-L.txt", "w");
				CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, m_p_L); // leaks memory
				fclose(p_fw);
			}
		}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
#endif // _DEBUG

		double f_fullL_end = m_timer.f_Time();

		// note that L and Lambda should be equal structurally (same size, the same block layout, except that lambda is symmetric and L is upper/lower diagonal)

		// @todo - calculate LTL and compare to lambda (that's the way it's supposed to equal, right?)

		// omega is required to build L, HtR is required to build d. so far we're good with just omega
		// omega is build from an edge, exactly the same way lambda is built (omega is just a submatrix of lambda - could use the same code / data / storage)
		// watch out! lambda is actually *not* equal to omega, omega is only for one or more edges, the off-diagonal elements that correspond to vertices have sums from other edges than just those being updated - the update process needs to be carried out differently, or we need to figure out how to add only contributions for edges that are being added : a simple id/order range condition would take care of that

		// todo - if we want lambda, we need to recalc sums of blocks in vertices
		// (not much work, can use inverse condition and just add missing contributions, without clearing)
		// right now we have omega

		// evidently, to do the following, one needs to have proper lambda (not just omega, the vertices calculate with it)

		{
			Collect_RightHandSide_Vector(m_v_dx);
			// collects the right-hand side vector (eta)

#if 0
			++ m_n_full_forwardsubst_num;

			cs_utsolve(m_p_L, &m_v_dx(0)); // d = eta = eta/L
			m_v_d = m_v_dx; // !!
#elif defined(__NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST)
			++ m_n_full_forwardsubst_num;

			_ASSERTE(m_p_lambda_elem_ordering);
			PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
			Eigen::VectorXd vec_copy = m_v_perm_temp;
			m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&vec_copy(0), vec_copy.rows());
			cs_utsolve(m_p_L, &m_v_perm_temp(0)); // d = eta = eta/L
			_ASSERTE((vec_copy - m_v_perm_temp).norm() < 1e-7); // makes sure that the vectors are the same
			InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows());
#else // 0
			++ m_n_full_forwardsubst_num;

			_ASSERTE(m_p_lambda_elem_ordering);
			PermuteVector(m_p_lambda_elem_ordering, &m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows());
			m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
			// d = eta = eta/L
			InversePermuteVector(m_p_lambda_elem_ordering, &m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows());
#endif // 0
			// convert eta to d
		}
		// todo - update d incrementally as well

		double f_d_end = m_timer.f_Time();

		m_f_fullL_time += f_fullL_end - f_sparse_end; // use range timer for complete L rebuilds // whatever that means ...
		m_f_fullL_cholesky += f_cholesky_end - f_sparse_end;
		m_f_toupper2_time += f_sparse2_end - f_cholesky_end;
		m_f_d_time += f_d_end - f_sparse2_end;

		/*printf("L block width: " PRIsize ", L block num: " PRIsize " (sparsity %.2f %%); taking full update (took %.5f sec)\n",
			m_R.n_BlockColumn_Num(), m_R.n_Block_Num(), float(m_R.n_Block_Num() * 9) /
			(m_R.n_Row_Num() * m_R.n_Column_Num()) * 100, f_fullL_end - f_omega_end);*/ // debug
	}

	/**
	 *	@brief refreshes the L matrix either from (pert of) lambda or from omega
	 *
	 *	@param[in] n_referesh_from_vertex is zero-based index of the first vertex that changes (unused)
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge that changes
	 *
	 *	@return Returns true if L was refreshed, false if it decided to take lambda fallback instead.
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	inline bool b_Refresh_L(size_t UNUSED(n_referesh_from_vertex) = 0, size_t n_refresh_from_edge = 0) // throw(std::bad_alloc, std::runtime_error)
	{
		double f_reorder_start = m_timer.f_Time();

		// note that lambda is now up to date

		bool b_allow_lambda_fallback = false; // disallow for benchmarks
		b_allow_lambda_fallback = b_allow_lambda_fallback &&
			(m_n_linear_solve_threshold == 1 || m_n_nonlinear_solve_threshold == 1); // only do lambda fallback if sloving each one
		// see if we can do lambda fallback

		bool b_force_reorder = !n_refresh_from_edge; // if rebuilding whole lambda, it would be shame not to reorder
		// flag for forcing reorder

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		if(false) { // no optimizations for L up variants timing
#else // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		if(true) { // if allow optimizations ...
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
			if(!b_force_reorder && !b_allow_lambda_fallback) { // in case we're doing every 1, we catch big loops with lambda fallback
				size_t n_last = m_r_system.r_Vertex_Pool().n_Size() - 1;
				size_t n_order_min_projection = (n_last < m_n_lambda_block_ordering_size)?
					m_p_lambda_block_ordering[n_last] : n_last;
				for(size_t i = n_refresh_from_edge, n = m_r_system.r_Edge_Pool().n_Size(); i < n; ++ i) {
					_ASSERTE(m_r_system.r_Edge_Pool()[i].n_Vertex_Num() == 2); // won't work for hyperedges // todo
					size_t n_order_v0 = m_r_system.r_Edge_Pool()[i].n_Vertex_Id(0);
					size_t n_order_v1 = m_r_system.r_Edge_Pool()[i].n_Vertex_Id(1); // note that these are ids, but these equal order at the moment
					_ASSERTE(n_order_v0 < m_r_system.r_Vertex_Pool().n_Size()); // make sure none of the vertices is constant
					_ASSERTE(n_order_v1 < m_r_system.r_Vertex_Pool().n_Size()); // make sure none of the vertices is constant
					n_order_v0 = (n_order_v0 < m_n_lambda_block_ordering_size)?
						m_p_lambda_block_ordering[n_order_v0] : n_order_v0;
					n_order_v1 = (n_order_v1 < m_n_lambda_block_ordering_size)?
						m_p_lambda_block_ordering[n_order_v1] : n_order_v1;
					n_order_min_projection = std::min(n_order_min_projection, std::min(n_order_v0, n_order_v1));
				}
				size_t n_order_max = m_lambda.n_BlockColumn_Num();
				// project where will n_order_min end up after ordering is updated

				size_t n_projected_update_size = n_order_max - n_order_min_projection;
				if(n_projected_update_size > m_n_big_loop_threshold)
					b_force_reorder = true;
			}
			// calculates how big would the next edge loop be, assuming that the permutation
			// is just extended with identity permutation and forces full L update if it was large

			if(!b_force_reorder && m_lambda.n_BlockColumn_Num() > m_n_last_full_L_update_size + 10) { // it's always dense at the beginning // hlamfb 10
				size_t n_nnz = m_R.n_Storage_Size();
				float f_area = float(m_R.n_Column_Num()) * m_R.n_Column_Num();
				float f_density = n_nnz / f_area;
				if(f_density > 0.02f) {
					b_force_reorder = true;
					//printf("L became too dense (%.2f %%), forcing reorder\n", f_density * 100); // verbose
				}
			}
			// permit 2% density in L, then rebuild
		}
		// these two should just set b_force_reorder

		if(b_force_reorder) {
			// only calculate a new ordering on full refresh or if forced

			//printf("build new ordering ...\n");

			m_p_lambda_block_ordering = m_lambda_ordering.p_InvertOrdering(
				//m_lambda_ordering.p_HybridBlockOrdering(m_lambda, m_r_system.r_Edge_Pool().n_Size()), m_lambda.n_BlockColumn_Num()); // uses hybrid aat calculation
				m_lambda_ordering.p_BlockOrdering(m_lambda), m_lambda.n_BlockColumn_Num()); // todo - make sure that the last vertex is a pose (otherwise we need to modify the constraint to select the pose, not the landmark)
			// get blockwise and elementwise ordering ...

			m_L_row_lookup_table.clear();
			// can't reuse lookup of L's rows since these change with ordering
		} else if(m_lambda.n_BlockColumn_Num() > m_lambda_perm.n_BlockColumn_Num()) {
			// simply appends ordering with a new value (identity ordering at the end)

			//printf("extend ordering by " PRIsize "\n", m_lambda.n_BlockColumn_Num() - m_lambda_perm.n_BlockColumn_Num());

			m_p_lambda_block_ordering = m_lambda_ordering.p_InvertOrdering(
				m_lambda_ordering.p_ExtendBlockOrdering_with_Identity(m_lambda.n_BlockColumn_Num()),
				m_lambda.n_BlockColumn_Num());
			// get blockwise and elementwise ordering ...
		}
		m_n_lambda_block_ordering_size = m_lambda.n_BlockColumn_Num();
		m_p_lambda_elem_ordering = 0; // needs to be updated
		// refresh/update the ordering (update means append with identity)

		m_p_lambda_elem_ordering = m_lambda_ordering.p_ExpandBlockOrdering(m_lambda);
		m_lambda.Permute_UpperTriangular_To(m_lambda_perm, m_p_lambda_block_ordering,
			m_lambda.n_BlockColumn_Num(), true);
		// make a reordered version of lambda (*always* changes if lambda changes)

#ifdef _DEBUG
		size_t n_uncover_num = 0;
		for(size_t i = 0, n = m_v_dx.rows(); i < n; ++ i) {
			const size_t *p_elem, *p_end;
			if((p_elem = std::find(m_p_lambda_elem_ordering,
			   p_end = m_p_lambda_elem_ordering + n, i)) == p_end) {
				fprintf(stderr, "error: element " PRIsize " of dx not covered\n", i);
				++ n_uncover_num;
			}
			// makes sure all the elements of the output vector are covered (seems they're not?)

			while(i + 1 < n && p_elem + 1 != p_end && p_elem[1] == i + 1) {
				++ i;
				++ p_elem;
			}
			// skip any sequences of consecutive elements (otherwise the search takes O(n^2))
		}
		if(n_uncover_num > 0) {
			fprintf(stderr, "error: " PRIsize " elements of dx (out of " PRIsize ") not covered\n",
				n_uncover_num, m_v_dx.rows());
		}
#endif // _DEBUG

		_ASSERTE(m_n_lambda_block_ordering_size == m_r_system.r_Vertex_Pool().n_Size());
		size_t n_order_min;
		if(b_force_reorder)
			n_order_min = 0; // a new ordering? from the ground up then ...
		else {
			n_order_min = m_p_lambda_block_ordering[m_n_lambda_block_ordering_size - 1];
			for(size_t i = n_refresh_from_edge, n = m_r_system.r_Edge_Pool().n_Size(); i < n; ++ i) {
				size_t n_order_v0 = m_p_lambda_block_ordering[m_r_system.r_Edge_Pool()[i].n_Vertex_Id(0)];
				size_t n_order_v1 = m_p_lambda_block_ordering[m_r_system.r_Edge_Pool()[i].n_Vertex_Id(1)]; // note that these are ids, but these equal order at the moment
				n_order_min = std::min(n_order_min, std::min(n_order_v0, n_order_v1));
			}
			_ASSERTE(n_order_min < m_r_system.r_Vertex_Pool().n_Size()); // make sure there are some non-const vertices
			//printf("loop size: " PRIsize "\n", m_n_lambda_block_ordering_size - 1 - n_order_min); // debug
		}
		// calculate min vertex order that needs to be updated (within the ordering!)

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		m_n_loop_size_cumsum += m_n_lambda_block_ordering_size - 1 - n_order_min;
		// calculate how much loops did it process
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS

#ifdef _DEBUG
		/*m_lambda_perm.Rasterize("lslam6_lambdaPermBeforeOpt.tga");
		m_R.Rasterize("lslam7_LBeforeOpt.tga");
#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
		{
			static bool b_first_iter = true;
			FILE *p_fw = fopen("lambda-block-ordering.txt", (b_first_iter)? "w" : "a");
			fprintf(p_fw, "ordering(" PRIsize ") = {" PRIsize "", m_lambda.n_BlockColumn_Num(), m_p_lambda_block_ordering[0]);
			for(size_t i = 1, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i)
				fprintf(p_fw, ", " PRIsize "", m_p_lambda_block_ordering[i]);
			size_t n_order_max = m_lambda.n_BlockColumn_Num();
			fprintf(p_fw, "}; // n_order_min = " PRIsize " (" PRIsize "), n_order_max - 1 = " PRIsize " (" PRIsize ")\n",
				n_order_min, m_p_lambda_block_ordering[n_order_min],
				n_order_max - 1, m_p_lambda_block_ordering[n_order_max - 1]);
			fclose(p_fw);
			b_first_iter = false;
		}
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS*/
#endif // _DEBUG

		double f_reorder_time = m_timer.f_Time() - f_reorder_start;
		m_f_ordering_time += f_reorder_time;
		// stats

		if(!b_force_reorder && b_allow_lambda_fallback &&
		   m_lambda.n_BlockColumn_Num() - n_order_min > m_n_big_loop_threshold)
			return false;
		// a big loop while solving each one: use lambda solver instead

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		static bool b_first_time_dump = true;
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES

		if(n_order_min > 0) {
			// if !n_order_min, L01 is empty and L merely equals chol(lambda)

			if(m_n_edges_in_lambda == m_r_system.r_Edge_Pool().n_Size()) {
				_ASSERTE(m_n_verts_in_lambda == m_lambda.n_BlockColumn_Num());
				_ASSERTE(m_n_verts_in_lambda == m_r_system.r_Vertex_Pool().n_Size());
				return true;
			}
			// this is final optimization, no need to refresh, there is no new edge / vertex

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
			double p_inc_upd_times_start[] = {
				m_f_lupdate_time,
				m_f_toupper_time,
				m_f_d_time
			};
			double p_omega_upd_times_start[] = {
				m_f_l11_omega_calc_time,
				m_f_l11_omega_slice_time,
				m_f_l11_omega_ata_time,
				m_f_l11_omega_add_time
			};
			double p_lambda_upd_times_start[] = {
				m_f_l11_lambda_slice_time,
				m_f_l11_lambda_ata_time,
				m_f_l11_lambda_add_time
			};
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES

			Refresh_L_IncL11(n_refresh_from_edge, n_order_min);
			// run the "fast" refresh of L

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
			double p_inc_upd_times[] = {
				m_f_lupdate_time,
				m_f_toupper_time,
				m_f_d_time
			};
			double f_inc_upd_sum = 0;
			for(int i = 0; i < 3; ++ i) {
				p_inc_upd_times[i] -= p_inc_upd_times_start[i];
				f_inc_upd_sum += p_inc_upd_times[i];
			}
			double p_omega_upd_times[] = {
				m_f_l11_omega_calc_time,
				m_f_l11_omega_slice_time,
				m_f_l11_omega_ata_time,
				m_f_l11_omega_add_time
			};
			double f_omega_upd_sum = f_inc_upd_sum;
			for(int i = 0; i < 4; ++ i) {
				p_omega_upd_times[i] -= p_omega_upd_times_start[i];
				f_omega_upd_sum += p_omega_upd_times[i];
			}
			double p_lambda_upd_times[] = {
				m_f_l11_lambda_slice_time,
				m_f_l11_lambda_ata_time,
				m_f_l11_lambda_add_time
			};
			double f_lambda_upd_sum = f_inc_upd_sum;
			for(int i = 0; i < 3; ++ i) {
				p_lambda_upd_times[i] -= p_lambda_upd_times_start[i];
				f_lambda_upd_sum += p_lambda_upd_times[i];
			}
			// calculate times

			size_t n_loop_size = m_n_lambda_block_ordering_size - 1 - n_order_min;
			bool b_had_omega_upd = f_omega_upd_sum > f_inc_upd_sum; // omega update only if it can
			_ASSERTE(f_lambda_upd_sum > 0); // lambda update always

			double f_full_L_start = m_timer.f_Time();
			Refresh_L_FullL();
			double f_full_L_time = m_timer.f_Time() - f_full_L_start;
			// measure full L as well

			FILE *p_fw = fopen("Lup_variants_time.txt", (b_first_time_dump)? "w" : "a");
			if(b_first_time_dump) {
				fprintf(p_fw, "verts-in-L;loop-size;full-L-time;lambda-up-time;lambda-slice-time;"
					"lambda-ata-time;lambda-add-time;omega-up-time;omega-calc-time;"
					"omega-slice-time;omega-ata-time;omega-add-time\n");
			}
			fprintf(p_fw, "" PRIsize ";" PRIsize ";%f;%f;%f;%f;%f", m_R.n_BlockColumn_Num(), n_loop_size, f_full_L_time,
				f_lambda_upd_sum, p_lambda_upd_times[0], p_lambda_upd_times[1], p_lambda_upd_times[2]);
			if(b_had_omega_upd) {
				fprintf(p_fw, ";%f;%f;%f;%f;%f\n", f_omega_upd_sum, p_omega_upd_times[0],
					p_omega_upd_times[1], p_omega_upd_times[2], p_omega_upd_times[3]);
			} else {
				fprintf(p_fw, ";;;;;\n");
			}
			fclose(p_fw);
			// print timing to a file
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		} else {
#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
			double f_full_L_start = m_timer.f_Time();
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES

			Refresh_L_FullL();
			// do the "full" L = chol(lambda)

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
			double f_full_L_time = m_timer.f_Time() - f_full_L_start;
			// measure full L

			size_t n_loop_size = m_n_lambda_block_ordering_size - 1 - n_order_min;

			FILE *p_fw = fopen("Lup_variants_time.txt", (b_first_time_dump)? "w" : "a");
			if(b_first_time_dump) {
				fprintf(p_fw, "verts-in-L;loop-size;full-L-time;lambda-up-time;lambda-slice-time;"
					"lambda-ata-time;lambda-add-time;omega-up-time;omega-calc-time;"
					"omega-slice-time;omega-ata-time;omega-add-time\n");
			}
			fprintf(p_fw, "" PRIsize ";" PRIsize ";%f;;;;", m_R.n_BlockColumn_Num(), n_loop_size, f_full_L_time); // no lambda upd
			fprintf(p_fw, ";;;;;\n"); // no omega upd
			fclose(p_fw);
			// print timing to a file
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		}

#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
		b_first_time_dump = false;
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES

#ifdef _DEBUG
		//m_R.Rasterize("lslam7_LAfterOpt.tga");
#endif // _DEBUG

		return true;
	}

	/**
	 *	@brief calculates inverse permutation of a dense vector
	 *
	 *	@param[in] p_permutation is the permutation
	 *	@param[in] p_vector is the original unpermuted vector
	 *	@param[in] p_dest_vector is the destination vector to be filled with
	 *		the permuted vector (allocated by the caller)
	 *	@param[in] n_vector_size is size of the vector in elements
	 *		(all of the previous parameters need to be allocated to this size)
	 *
	 *	@note Originally from CSparse, uses size_t for permutation to avoid long vectors in x64.
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 */
	static void InversePermuteVector(const size_t *p_permutation, const double *p_vector,
		double *p_dest_vector, size_t n_vector_size)
	{
		_ASSERTE(p_vector != p_dest_vector); // does not work inplace
		for(size_t i = 0; i < n_vector_size; ++ i)
			p_dest_vector[p_permutation[i]] = p_vector[i];
	}

	/**
	 *	@brief calculates forward permutation of a dense vector
	 *
	 *	@param[in] p_permutation is the permutation
	 *	@param[in] p_vector is the original unpermuted vector
	 *	@param[in] p_dest_vector is the destination vector to be filled with
	 *		the permuted vector (allocated by the caller)
	 *	@param[in] n_vector_size is size of the vector in elements
	 *		(all of the previous parameters need to be allocated to this size)
	 *
	 *	@note Originally from CSparse, uses size_t for permutation to avoid long vectors in x64.
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 */
	static void PermuteVector(const size_t *p_permutation, const double *p_vector,
		double *p_dest_vector, size_t n_vector_size)
	{
		_ASSERTE(p_vector != p_dest_vector); // does not work inplace
		for(size_t i = 0; i < n_vector_size; ++ i)
			p_dest_vector[i] = p_vector[p_permutation[i]];
	}

	/**
	 *	@brief refreshes the lambda matrix by recalculating edge hessians
	 *
	 *	@param[in] n_referesh_from_vertex is zero-based index of the first vertex that changes
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge that changes
	 */
	inline void Refresh_Lambda(size_t n_referesh_from_vertex = 0, size_t n_refresh_from_edge = 0) // throw(std::bad_alloc, std::runtime_error)
	{
		if(n_refresh_from_edge) {
			m_r_system.r_Edge_Pool().For_Each_Parallel(n_refresh_from_edge,
				m_r_system.r_Edge_Pool().n_Size(), CCalculate_Lambda()); // this is not ok, the update will contain some contributions that are not supposed to be there
			// in order to fix this, we need to modify the call to vertex update (the vertices need to specifically ignore contributions from some edges)
		} else
			m_r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Lambda()); // this is ok, the update is going to be calculated correctly
		if(n_referesh_from_vertex) {
			m_r_system.r_Vertex_Pool().For_Each_Parallel(n_referesh_from_vertex,
				m_r_system.r_Vertex_Pool().n_Size(), CCalculate_Lambda());
		} else
			m_r_system.r_Vertex_Pool().For_Each_Parallel(CCalculate_Lambda()); // this is currently always from vert 0
		// can do this in parallel

		if(!n_referesh_from_vertex) {
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_first_vertex_id = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
			size_t n_first_vertex_id = m_r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!m_r_system.r_Vertex_Pool()[n_first_vertex_id].b_IsConstant()); // this one must not be constant
			size_t n_first_vertex_order = m_r_system.r_Vertex_Pool()[n_first_vertex_id].n_Order();
			// get id of the first vertex (usually zero)

			const Eigen::MatrixXd &r_t_uf = m_r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
			m_lambda.t_FindBlock(n_first_vertex_order, n_first_vertex_order).noalias() += r_t_uf.transpose() * r_t_uf;
			// for lambda yes
		}
		// add unary factor (gets overwritten by the first vertex' block)
	}

#if 1
	/**
	 *	@brief calculates cholesky factorization of a sparse matrix
	 *	@param[in] p_A is a sparse matrix in compressed column form
	 *	@return Returns cholesky factorization of A, in compressed column form
	 *		(the caller is responsible for disposing of the matrix).
	 */
	static cs *p_SparseCholesky_Preordered(const cs *p_A) // t_odo - move this to linear solver, maybe create another linear solver for this task (not to disrupt the other linear solver workspace cache, ...)
	{
		css *p_sym_order;
		if(!(p_sym_order = cs_schol(0, p_A))) // do use symbolic something! (keeps L sparse)
			return 0;
		// ordering and symbolic analysis for a Cholesky factorization

		csn *p_factor;
		if(!(p_factor = cs_chol(p_A, p_sym_order))) {// @todo - use CLinearSolver_CSparse to do that, it caches workspace and stuff ...
			cs_sfree(p_sym_order);
			return 0;
		}

		cs *p_L = p_factor->L;//cs_symperm(p_factor->L, p_factor->pinv, 1);

		cs_sfree(p_sym_order);

		cs_spfree(p_factor->U); // t_odo - remove if possible
		cs_free(p_factor->pinv);
		cs_free(p_factor->B);
		//cs_free(p_factor->L);
		// note that the above pointers are most likely null (not used for Cholesky, only by QR or LU)

		return p_L;
	}
#endif // 0

	/**
	 *	@brief function object that calculates and accumulates chi^2 from all the edges
	 */
	class CSum_ChiSquareError {
	protected:
		double m_f_sum; /**< @brief a running sum of \f$\chi^2\f$ errors */

	public:
		/**
		 *	@brief default constructor
		 */
		inline CSum_ChiSquareError()
			:m_f_sum(0)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in] r_t_edge is edge to output its part R error vector
		 */
		template <class _TyEdge>
		inline void operator ()(const _TyEdge &r_t_edge)
		{
			m_f_sum += r_t_edge.f_Chi_Squared_Error();
		}

		/**
		 *	@brief gets the current value of the accumulator
		 *	@return Returns the current value of the accumulator.
		 */
		inline operator double() const
		{
			return m_f_sum;
		}
	};

	/**
	 *	@brief function object that updates states of all the vertices
	 */
	class CUpdateEstimates {
	protected:
		const Eigen::VectorXd &m_r_dx; /**< @brief vector of differences */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_dx is reference to the vector of differences
		 */
		inline CUpdateEstimates(const Eigen::VectorXd &r_dx)
			:m_r_dx(r_dx)
		{}

		/**
		 *	@brief updates vertex state
		 *	@tparam _TyVertex is type of vertex
		 *	@param[in,out] r_t_vertex is reference to vertex, being updated
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex) const
		{
			r_t_vertex.Operator_Plus(m_r_dx);
		}
	};

	/**
	 *	@brief updates states of all the vertices with the error vector
	 */
	inline void PushValuesInGraphSystem(const Eigen::VectorXd &r_v_dx)
	{
		m_r_system.r_Vertex_Pool().For_Each_Parallel(CUpdateEstimates(r_v_dx)); // can do this in parallel
	}

	/**
	 *	@brief function object that calculates hessians in all the edges
	 */
	class CCalculate_Lambda {
	public:
		/**
		 *	@brief function operator
		 *	@tparam _TyVertexOrEdge is edge type
		 *	@param[in] r_t_vertex_or_edge is vertex or edge to update it's hessians
		 */
		template <class _TyVertexOrEdge>
		inline void operator ()(_TyVertexOrEdge &r_t_vertex_or_edge) const
		{
			r_t_vertex_or_edge.Calculate_Hessians();
		}
	};

	CNonlinearSolver_L(const CNonlinearSolver_L &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_L &operator =(const CNonlinearSolver_L &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_L_INCLUDED
