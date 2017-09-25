/*
								+-----------------------------------+
								|                                   |
								| *** L factor nonlinear solver *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|      NonlinearSolver_FastL.h      |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_FAST_L_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_FAST_L_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_FastL.h
 *	@brief nonlinear blocky solver with progressive reordering, working above the L factor matrix
 *	@author -tHE SWINe-
 *	@date 2013-01-28
 */

#include "slam/FlatSystem.h"
#include "slam/OrderingMagic.h"
#include "slam/IncrementalPolicy.h"
#include "slam/Marginals.h"
#include "slam/NonlinearSolver_Base.h"
#include "slam/NonlinearSolver_Lambda_Base.h"
#include "slam/NonlinearSolver_FastL_Base.h"

/** \addtogroup nlsolve
 *	@{
 */

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
class CNonlinearSolver_FastL : public nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, false, true> {
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
	 *	@brief some run-time constants, stored as enum
	 */
	enum {
		b_Is_PoseOnly_SLAM = CTypelistLength<_TyAMatrixBlockSizes>::n_result == 1, /**< @brief determines if we're doing pose-only SLAM (10k) */
		b_Have_NativeSolver = /*fL_util::*/CIsNativeSolver<_TyLinearSolver>::b_result /**< @brief determines if the native linear solver is being used */
	};

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
		solver_HasSchur = false, /**< @brief Schur complement support flag */
		solver_HasDelayedOptimization = true, /**< @brief delayed optimization support flag */
		solver_IsPreferredBatch = false, /**< @brief preferred batch solver flag */
		solver_IsPreferredIncremental = true, /**< @brief preferred incremental solver flag */
		solver_ExportsJacobian = false, /**< @brief interface for exporting jacobian system matrix flag */
		solver_ExportsHessian = true, /**< @brief interface for exporting hessian system matrix flag */
		solver_ExportsFactor = true /**< @brief interface for exporting factorized system matrix flag */
	};

protected:
	typedef nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, false, true> _TyBase; /**< @brief base solver utils type */

//	CSystem &this->m_r_system; /**< @brief reference to the system */
//	CLinearSolver this->m_linear_solver; /**< @brief linear solver */
	CLinearSolver m_linear_solver2; /**< @brief linear solver for calculating cholesky of R and cholesky of R increment */

	std::vector<size_t> m_chol_etree; /**< @brief reusable e-tree storage */
	std::vector<size_t> m_chol_ereach_stack; /**< @brief reusable workspace for Cholesky */
	std::vector<size_t> m_chol_bitfield; /**< @brief reusable workspace for Cholesky */

	CUberBlockMatrix m_R; /**< @brief the R matrix (built / updated incrementally) */

	bool m_b_outstanding_loop_closures; /**< @brief (probable) loop closure flag */
	bool m_b_first_iteration_use_R; /**< @brief flag for using the R matrix or rather lambda in the first iteration of nonlinear optimization */
	bool m_b_R_up_to_date; /**< @brief dirty flag for the R matrix (required to keep track after lambda updates and linearization point changes) */
	bool m_b_R_updatable; /**< @brief dirty flag for the R matrix (if set, R is only missing some edges, if not set then the linearization changed and a full update is required) */
	size_t m_n_last_full_R_update_size; /**< @brief the last number of block columns in R when it was fully updated */
	std::vector<size_t> m_R_row_lookup_table; /**< @brief row lookup table for R (used by b_Refresh_R() and Refresh_R11()) */
	//size_t m_n_big_loop_threshold; /**< @brief threshold for what is considered a "big" loop (incrementing R is avoided) */

	CMatrixOrdering m_lambda_ordering; /**< @brief lambda block ordering calculator (CAMD wrapper) */
	const size_t *m_p_lambda_block_ordering; /**< @brief lambda block ordering (only valid if m_b_R_up_to_date is set) */ // todo - convert all those to size_t
	size_t m_n_lambda_block_ordering_size; /**< @brief lambda block ordering size */
	CUberBlockMatrix m_lambda_perm; /**< @brief the reordered reference to the lambda matrix */
	CUberBlockMatrix m_lambda; /**< @brief the lambda matrix (built / updated incrementally) */
	_TyReductionPlan m_reduction_plan; /**< @brief lambda incremental reduction plan */

	CMatrixOrdering m_lambda11_ordering; /**< @brief lambda11 block ordering calculator (CAMD wrapper) */
	const size_t *m_p_lambda11_block_ordering; /**< @brief lambda block ordering (only valid if m_b_R_up_to_date is set) */ // todo - convert all those to size_t
	size_t m_n_lambda_block11_ordering_size; /**< @brief lambda block ordering size */

	CFirstLastElementOrderingConstraint m_lambda11_constraint; /**< @brief incremental lambda ordering constraint */
	CLastElementOrderingConstraint m_lambda_constraint; /**< @brief global lambda ordering constraint */
	CMatrixOrdering m_lambda_alt_ordering; /**< @brief secondary lambda ordering, calculated from m_lambda_perm */
	CNFirst1LastElementOrderingConstraint m_lambda_alt_constraint; /**< @brief constraint for the secondary lambda ordering */

	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	Eigen::VectorXd m_v_d; /**< @brief d vector */
	Eigen::VectorXd m_v_perm_temp; /**< @brief temporary storage for the permutation vector, same dimension as d and dx */
	size_t m_n_verts_in_lambda; /**< @brief number of vertices already in lambda */
	size_t m_n_edges_in_lambda; /**< @brief number of edges already in lambda */
//#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
//	size_t m_n_last_optimized_vertex_num;
//#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
//	size_t m_n_step; /**< @brief counter of incremental steps modulo m_n_nonlinear_solve_threshold */
//#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
//	size_t m_n_linear_solve_threshold; /**< @brief step threshold for linear solve */
//	size_t m_n_nonlinear_solve_threshold; /**< @brief step threshold for nonlinear solve */
//	size_t m_n_nonlinear_solve_max_iteration_num; /**< @brief maximal number of iterations in incremental nonlinear solve */
//	double m_f_nonlinear_solve_error_threshold; /**< @brief error threshold in incremental nonlinear solve */ // t_odo - document these in elementwise A and R
//	bool this->m_b_verbose; /**< @brief verbosity flag */
//
//	size_t m_n_real_step; /**< @brief counter of incremental steps (no modulo) */

	bool m_b_system_dirty; /**< @brief system updated without relinearization flag */
	bool m_b_linearization_dirty; /**< @brief system matrices updated but relinearization point was not set flag */

#ifndef __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
	typedef CVoidTimerSampler _TyTimeSampler; /**< @brief timer sampler type */
#else // !__NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
	typedef CTimerSampler _TyTimeSampler; /**< @brief timer sampler type */
#endif // !__NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
	typedef _TyTimeSampler::_TySample _TyTime; /**< @brief time type */

	bool m_b_inhibit_optimization; /**< @brief optimization enable falg */
	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	_TyTime m_f_chol_time; /**< @brief time spent in Choleski() section */
	_TyTime m_f_norm_time; /**< @brief time spent in norm calculation section */
	_TyTime m_f_vert_upd_time; /**< @brief time spent in updating the vertices */

	size_t m_n_full_forwardsubst_num; /**< @brief number of d updates performed using full R forward substitution */
	size_t m_n_resumed_forwardsubst_num; /**< @brief number of d updates performed using resumed R forward substitution */
	size_t m_n_resumed_perm_forwardsubst_num; /**< @brief number of d updates performed using resumed R forward substitution, while being permutated in the updated area */
	size_t m_n_R_optim_num; /**< @brief number of system optimizations performed using R backsubstitution */
	size_t m_n_lambda_optim_num; /**< @brief number of system optimizations performed using cholsol(lambda) */
	size_t m_n_Rup_num; /**< @brief number of R increments */
	size_t m_n_omega_update_num; /**< @brief number of R increments calculated using omega */
	size_t m_n_lambda_update_num; /**< @brief number of R increments calculated using lambda */
	size_t m_n_full_R_num; /**< @brief number of R updates */
	_TyTime m_f_lambda_refresh_time; /**< @brief time spent in updating and allocating lambda */
	_TyTime m_f_rhs_time; /**< @brief time spent in updating right-hand side vector */
	_TyTime m_f_ordering_time; /**< @brief time spent calculating ordering of lambda */
	_TyTime m_f_fullR_d; /**< @brief time spent in updating d while doing full R */
	_TyTime m_f_r11_omega_calc_time; /**< @brief time spent calculating omega (R increment) */
	_TyTime m_f_r11_omega_slice_time; /**< @brief time spent in slicing \f$R_{11}\f$ (R increment) */
	_TyTime m_f_r11_omega_ata_time; /**< @brief time spent calculating \f$R_{11}^TR_{11}\f$ (R increment) */
	_TyTime m_f_r11_omega_add_time; /**< @brief time spent adding \f$R_{11}^TR_{11}\f$ + omega (R increment) */
	_TyTime m_f_r11_lambda_slice_time; /**< @brief time spent in slicing lambda11 and \f$R_{01}\f$ (R increment) */
	_TyTime m_f_r11_lambda_ata_time; /**< @brief time spent calculating \f$R_{01}^TR_{01}\f$ (R increment) */
	_TyTime m_f_r11_lambda_add_time; /**< @brief time spent adding \f$R_{01}^TR_{01} + \Lambda_{11}\f$ (R increment) */
	_TyTime m_f_Rupdate_time; /**< @brief time spent calculating cholesky of new \f$R_{11}\f$ (R increment) */
	_TyTime m_f_d_time; /**< @brief time spent updating d (right hand side vector) */
	_TyTime m_f_backsubst_time; /**< @brief time spent in backsubstitution (solving for R / d) */
	_TyTime m_f_fullR_cholesky; /**< @brief time spent in calculating cholesky (R update) */

	size_t m_n_resumed_chol_num; /**< @brief number of times the resumed Cholesky was used */
	size_t m_n_blocks_above_num; /**< @brief number of times there were blocks above lambda_11 */
	size_t m_n_limited_search_num; /**< @brief number of times there were blocks above lambda_11 but only a smaller submatrix was sufficient for permutation calculation */
	_TyTime m_f_ordering_fold_time; /**< @brief time spent folding two orderings */
	_TyTime m_f_repermute_time; /**< @brief time spent repermuting lambda matrix with incremented ordering */
	_TyTime m_f_Rslice_time; /**< @brief time spent slicing R for resumed Cholesky */
	_TyTime m_f_etree_time; /**< @brief time spent calculating the elimination tree */
	_TyTime m_f_resumed_chol_time; /**< @brief time spent in resumed Cholesky */
	_TyTime m_f_ordering11_time; /**< @brief time spent in calculating the incremental ordering */
	_TyTime m_f_ordering11_part_time; /**< @brief time spent in calculating the incremental ordering, only the small or inflated lambda_11 cases */
	_TyTime m_f_ordering11_full_time; /**< @brief time spent in calculating the incremental ordering, only the full lambda_perm cases */

	std::vector<size_t> lambda_perm_frontline; /**< @brief cached frontline of the lambda_perm matrix */

//	TMarginalsComputationPolicy this->m_t_marginals_config; /**< @brief marginal covariance computation configuration */
	_TyTime m_f_marginals_time; /**< @brief time spent in calculating marginal covariances (batch) */
	_TyTime m_f_incmarginals_time; /**< @brief time spent in calculating marginal covariances (update) */
	size_t m_n_incmarginals_num; /**< @brief number of times the marginals update ran instead of batch recalculation */
//	CMarginalCovariance this->m_marginals; /**< @brief marginals cache */

//	CTimer this->m_timer; /**< @brief timer object */

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
	size_t m_n_loop_size_cumsum; /**< @brief cumulative sum of loops processed so far */
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS

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
	CNonlinearSolver_FastL(CSystem &r_system, size_t n_linear_solve_threshold,
		size_t n_nonlinear_solve_threshold, size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool UNUSED(b_use_schur) = false)
		:_TyBase(r_system, n_linear_solve_threshold, n_nonlinear_solve_threshold,
		n_nonlinear_solve_max_iteration_num, f_nonlinear_solve_error_threshold,
		b_verbose, linear_solver, false), /*this->m_r_system(r_system), this->m_linear_solver(linear_solver),*/
		m_linear_solver2(linear_solver), m_b_outstanding_loop_closures(false),
		m_b_first_iteration_use_R(true), m_b_R_up_to_date(true), m_b_R_updatable(true), m_n_last_full_R_update_size(0),
		m_p_lambda_block_ordering(0), m_n_lambda_block_ordering_size(0),
		m_p_lambda11_block_ordering(0), m_n_lambda_block11_ordering_size(0),
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
		this->m_b_verbose(b_verbose), m_n_real_step(0),*/ m_b_system_dirty(false),
		m_b_linearization_dirty(false), m_b_inhibit_optimization(false),
		m_n_iteration_num(0), m_f_chol_time(0), m_f_norm_time(0), m_f_vert_upd_time(0),
		m_n_full_forwardsubst_num(0), m_n_resumed_forwardsubst_num(0),
		m_n_resumed_perm_forwardsubst_num(0), m_n_R_optim_num(0), m_n_lambda_optim_num(0),
		m_n_Rup_num(0), m_n_omega_update_num(0), m_n_lambda_update_num(0), m_n_full_R_num(0),
		m_f_lambda_refresh_time(0), m_f_rhs_time(0), m_f_ordering_time(0), m_f_fullR_d(0),
		m_f_r11_omega_calc_time(0), m_f_r11_omega_slice_time(0), m_f_r11_omega_ata_time(0),
		m_f_r11_omega_add_time(0), m_f_r11_lambda_slice_time(0), m_f_r11_lambda_ata_time(0),
		m_f_r11_lambda_add_time(0), m_f_Rupdate_time(0), m_f_d_time(0),
		m_f_backsubst_time(0), m_f_fullR_cholesky(0),
		m_n_resumed_chol_num(0), m_n_blocks_above_num(0), m_n_limited_search_num(0),
		m_f_ordering_fold_time(0), m_f_repermute_time(0), m_f_Rslice_time(0), m_f_etree_time(0),
		m_f_resumed_chol_time(0), m_f_ordering11_time(0), m_f_ordering11_part_time(0),
		m_f_ordering11_full_time(0), m_f_marginals_time(0), m_f_incmarginals_time(0),
		m_n_incmarginals_num(0)
	{
		//_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
		m_n_loop_size_cumsum = 0;
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
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
	CNonlinearSolver_FastL(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool UNUSED(b_use_schur) = false)
		:_TyBase(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, false),
		/*this->m_r_system(r_system), this->m_linear_solver(linear_solver),*/
		m_linear_solver2(linear_solver), m_b_outstanding_loop_closures(false),
		m_b_first_iteration_use_R(true), m_b_R_up_to_date(true), m_b_R_updatable(true), m_n_last_full_R_update_size(0),
		m_p_lambda_block_ordering(0), m_n_lambda_block_ordering_size(0),
		m_p_lambda11_block_ordering(0), m_n_lambda_block11_ordering_size(0),
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
		this->m_b_verbose(b_verbose), m_n_real_step(0),*/ m_b_system_dirty(false),
		m_b_linearization_dirty(false), m_b_inhibit_optimization(false),
		m_n_iteration_num(0), m_f_chol_time(0), m_f_norm_time(0), m_f_vert_upd_time(0),
		m_n_full_forwardsubst_num(0), m_n_resumed_forwardsubst_num(0),
		m_n_resumed_perm_forwardsubst_num(0), m_n_R_optim_num(0), m_n_lambda_optim_num(0),
		m_n_Rup_num(0), m_n_omega_update_num(0), m_n_lambda_update_num(0), m_n_full_R_num(0),
		m_f_lambda_refresh_time(0), m_f_rhs_time(0), m_f_ordering_time(0), m_f_fullR_d(0),
		m_f_r11_omega_calc_time(0), m_f_r11_omega_slice_time(0), m_f_r11_omega_ata_time(0),
		m_f_r11_omega_add_time(0), m_f_r11_lambda_slice_time(0), m_f_r11_lambda_ata_time(0),
		m_f_r11_lambda_add_time(0), m_f_Rupdate_time(0), m_f_d_time(0),
		m_f_backsubst_time(0), m_f_fullR_cholesky(0),
		m_n_resumed_chol_num(0), m_n_blocks_above_num(0), m_n_limited_search_num(0),
		m_f_ordering_fold_time(0), m_f_repermute_time(0), m_f_Rslice_time(0), m_f_etree_time(0),
		m_f_resumed_chol_time(0), m_f_ordering11_time(0), m_f_ordering11_part_time(0),
		m_f_ordering11_full_time(0), /*this->m_t_marginals_config(t_marginals_config),*/
		m_f_marginals_time(0), m_f_incmarginals_time(0), m_n_incmarginals_num(0)
	{
		//_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
		m_n_loop_size_cumsum = 0;
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS

		if(t_marginals_config.b_calculate) {
			if(t_marginals_config.t_increment_freq.n_period != t_incremental_config.t_nonlinear_freq.n_period &&
			   t_marginals_config.t_increment_freq.n_period != t_incremental_config.t_linear_freq.n_period) {
				throw std::runtime_error("in CNonlinearSolver_Lambda, the marginals must"
					" be updated with the same frequency as the system");
			}
			// unfortunately, yes, but in this particular solver that is easy to come around // todo

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
	 *	@brief gets the current system matrix
	 *	@return Returns const reference to the current system matrix.
	 *	@note This might not be up to date. To make sure, call Optimize(0) before calling this function.
	 */
	inline const CUberBlockMatrix &r_Lambda() const
	{
		return m_lambda;
	}

	/**
	 *	@brief gets the factor of the current system matrix
	 *	@return Returns const reference to the factor of the current system matrix.
	 *	@note This might not be up to date. To make sure, call Optimize(0) before calling this function.
	 */
	inline const CUberBlockMatrix &r_R() const
	{
		return m_R;
	}

	/**
	 *	@brief gets the ordering used in the current factorization of the system matrix
	 *	@return Returns pointer to the (inverse) block ordering of the factor of the current system matrix.
	 *
	 *	@note The ordering has length of n_R_Ordering_Size().
	 *	@note This might not be up to date. To make sure, call Optimize(0) before calling this function.
	 */
	inline const size_t *p_R_Ordering() const
	{
		return m_p_lambda_block_ordering;
	}

	/**
	 *	@brief gets the size of the ordering used in the current factorization of the system matrix
	 *	@return Returns size of the ordering used in the current factorization of the system matrix.
	 *
	 *	@note The ordering can be accessed by a call to p_R_Ordering().
	 *	@note This might not be up to date. To make sure, call Optimize(0) before calling this function.
	 */
	inline size_t n_R_Ordering_Size() const
	{
		return r_R().n_BlockColumn_Num();
	}

	/**
	 *	@brief gets the cumulative time of calculating the covariances
	 *	@return Returns the cumulative time of calculating the covariances, in seconds.
	 */
	inline double f_Marginals_CumTime() const
	{
		return m_f_marginals_time + m_f_incmarginals_time;
	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
#ifdef __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
		if(this->m_t_marginals_config.b_calculate) {
			printf("solver spent %f seconds in calculating the marginals\n",
				m_f_marginals_time + m_f_incmarginals_time);
			printf("\t batch: %f\n", m_f_marginals_time);
			printf("\t  incm: %f (ran " PRIsize " times)\n", m_f_incmarginals_time, m_n_incmarginals_num);
		}

		double f_serial_time = m_f_backsubst_time + m_f_chol_time + m_f_norm_time + m_f_vert_upd_time;
		if(f_total_time > 0) {
			printf("solver spent %f seconds in parallelizable section (updating R)\n",
				f_total_time - f_serial_time - (m_f_marginals_time + m_f_incmarginals_time));
		}
		double f_total_resumed_up = m_f_resumed_chol_time + m_f_Rslice_time +
			m_f_etree_time + m_f_ordering_fold_time + m_f_repermute_time;
		double f_total_omega_up = m_f_r11_omega_calc_time + m_f_r11_omega_slice_time +
			m_f_r11_omega_ata_time + m_f_r11_omega_add_time;
		double f_total_lambda_up = m_f_r11_lambda_slice_time +
			m_f_r11_lambda_ata_time + m_f_r11_lambda_add_time;
		double f_l_upd_time = f_total_resumed_up + f_total_omega_up +
			f_total_lambda_up + m_f_Rupdate_time + m_f_ordering11_time +
			m_f_ordering11_part_time + m_f_ordering11_full_time;
		double f_measured_parallel_time = m_f_lambda_refresh_time + m_f_rhs_time + m_f_ordering_time +
			m_f_fullR_d + m_f_fullR_cholesky + f_l_upd_time + m_f_d_time;
		printf("measured parallel time: %f, disparity: %f; out of which:\n", f_measured_parallel_time,
			(f_total_time > 0)? f_total_time - f_serial_time - f_measured_parallel_time -
			m_f_marginals_time - m_f_incmarginals_time : 0);
		printf("\t   ,\\: %f\n", m_f_lambda_refresh_time);
		printf("\t  rhs: %f\n", m_f_rhs_time);
		printf("\torder: %f\n", m_f_ordering_time);
		printf("\tfullR: %f (ran " PRIsize " times)\n", m_f_fullR_d + m_f_fullR_cholesky, m_n_full_R_num);
		printf("\tout of which:\n");
		printf("\t\t chol: %f\n", m_f_fullR_cholesky);
		printf("\t\t    d: %f\n", m_f_fullR_d);
		printf("\tR update: %f (ran " PRIsize " times)\n", f_l_upd_time, m_n_Rup_num);
		printf("\t\tordfu: %f (blocks above " PRIsize " times)\n", m_f_ordering11_full_time, m_n_blocks_above_num);
		printf("\t\tordli: %f (ran " PRIsize " times)\n", m_f_ordering11_part_time, m_n_limited_search_num);
		printf("\t\tordsm: %f (ran " PRIsize " times)\n", m_f_ordering11_time, m_n_Rup_num - m_n_blocks_above_num - m_n_limited_search_num);
		printf("\t\tresum: %f (ran " PRIsize " times)\n", f_total_resumed_up, m_n_resumed_chol_num);
		printf("\t\t\tofold: %f\n", m_f_ordering_fold_time);
		printf("\t\t\trperm: %f\n", m_f_repermute_time);
		printf("\t\t\tR cut: %f\n", m_f_Rslice_time);
		printf("\t\t\tetree: %f\n", m_f_etree_time);
		printf("\t\t\t chol: %f\n", m_f_resumed_chol_time);
		printf("\t\t  add: %f (ran " PRIsize " times)\n", f_total_omega_up + f_total_lambda_up +
			m_f_Rupdate_time, m_n_Rup_num - m_n_resumed_chol_num);
		printf("\t\t\tomega: %f (ran " PRIsize " times)\n", f_total_omega_up, m_n_omega_update_num);
		printf("\t\t\t\t calc: %f\n", m_f_r11_omega_calc_time);
		printf("\t\t\t\tslice: %f\n", m_f_r11_omega_slice_time);
		printf("\t\t\t\t Rata: %f\n", m_f_r11_omega_ata_time);
		printf("\t\t\t\tR11up: %f\n", m_f_r11_omega_add_time);
		printf("\t\t\t   ,\\: %f (ran " PRIsize " times)\n", f_total_lambda_up, m_n_lambda_update_num);
		printf("\t\t\t\tslice: %f\n", m_f_r11_lambda_slice_time);
		printf("\t\t\t\t Rata: %f\n", m_f_r11_lambda_ata_time);
		printf("\t\t\t\tR11up: %f\n", m_f_r11_lambda_add_time);
		printf("\t\t\t  Rup: %f // cholesky and fill\n", m_f_Rupdate_time);
		printf("\t    d: %f (resumed " PRIsize ", p-resumed " PRIsize ", full "
			PRIsize ")\n", m_f_d_time, m_n_resumed_forwardsubst_num,
			m_n_resumed_perm_forwardsubst_num, m_n_full_forwardsubst_num);
		printf("solver spent %f seconds in serial section\n", f_serial_time);
		printf("out of which:\n");
		printf("\t chol: %f (ran " PRIsize " times)\n", m_f_chol_time, m_n_lambda_optim_num);
		printf("\tbksub: %f (ran " PRIsize " times)\n", m_f_backsubst_time, m_n_R_optim_num);
		printf("\t norm: %f\n", m_f_norm_time);
		printf("\tv-upd: %f\n", m_f_vert_upd_time);
		/*printf("in unrelated news, small cholesky ran " PRIsize " times\n", m_n_dense_cholesky_num);
		printf("\t dense: %f\n", m_f_dense_cholesky_time);
		printf("\tsparse: %f\n", m_f_sparse_cholesky_time);*/ // dont want to do it runtime
#else // __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
		printf("it took: %f\n", f_total_time);
#endif // __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
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
				m_n_verts_in_lambda, m_n_edges_in_lambda);
			if(!m_b_system_dirty)
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda, 0, m_n_edges_in_lambda);
			else
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda);
			m_b_system_dirty = false;
			m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
			m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size();
			_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
				m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
				m_lambda.n_Column_Num() == this->m_r_system.n_VertexElement_Num()); // lambda is square, blocks on either side = number of vertices
			// need to have lambda
		} catch(std::bad_alloc&) {
			return false;
		}

		return m_lambda.Rasterize(p_s_filename, n_scalar_size);
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

		return m_lambda.Save_MatrixMarket(p_s_filename, p_s_layout_file, "lambda matrix for SLAM problem");
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns \f$\chi^2\f$ error.
	 *	@note This only works with systems with edges of one degree of freedom
	 *		(won't work for e.g. systems with both poses and landmarks).
	 */
	inline double f_Chi_Squared_Error() /*const*/
	{
		if(this->m_r_system.r_Edge_Pool().b_Empty())
			return 0;
#ifndef __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
		if(m_lambda.n_BlockColumn_Num() != this->m_r_system.r_Vertex_Pool().n_Size()) {
			Optimize(0, 0);
			// optimize but don't allow iterations - just updates lambda, d and R
			// in order to be able to generate approximate solutions on request
		}
#endif // !__NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
		if(m_b_linearization_dirty) {
			if(!CalculateOneTimeDx())
				return -1;
			nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, m_v_dx);
			double f_chi2 = _TyLambdaOps::f_Chi_Squared_Error(this->m_r_system);
			nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, -m_v_dx); // !!
			return f_chi2;
		} else {
			return _TyLambdaOps::f_Chi_Squared_Error(this->m_r_system);
		}
	}

	/**
	 *	@brief calculates denormalized \f$\chi^2\f$ error
	 *	@return Returns denormalized \f$\chi^2\f$ error.
	 *	@note This doesn't perform the final division by (number of edges - degree of freedoms).
	 */
	inline double f_Chi_Squared_Error_Denorm() /*const*/
	{
		if(this->m_r_system.r_Edge_Pool().b_Empty())
			return 0;
#ifndef __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
		if(m_lambda.n_BlockColumn_Num() != this->m_r_system.r_Vertex_Pool().n_Size()) {
			Optimize(0, 0);
			// optimize but don't allow iterations - just updates lambda, d and R
			// in order to be able to generate approximate solutions on request
		}
#endif // !__NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
		if(m_b_linearization_dirty) {
			typedef CNonlinearSolver_FastL<CSystem, CLinearSolver, CAMatrixBlockSizes> TThisType;
			if(!CalculateOneTimeDx())
				return -1;
			nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, m_v_dx);
			double f_chi2 = _TyLambdaOps::f_Chi_Squared_Error_Denorm(this->m_r_system);
			nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, -m_v_dx); // !!
			return f_chi2;
		} else
			return _TyLambdaOps::f_Chi_Squared_Error_Denorm(this->m_r_system);
	}

	/**
	 *	@brief delays optimization upon Incremental_Step()
	 */
	inline void Delay_Optimization()
	{
		m_b_inhibit_optimization = true;
	}

	/**
	 *	@brief enables optimization upon Incremental_Step()
	 *
	 *	This is default behavior. In case it was disabled (by Delay_Optimization()),
	 *	and optimization is required, this will also run the optimization.
	 */
	inline void Enable_Optimization()
	{
		if(m_b_inhibit_optimization) {
			m_b_inhibit_optimization = false;

			if(!this->m_r_system.r_Edge_Pool().b_Empty()) {
				bool b_optimized = TryOptimize(this->m_t_incremental_config.n_max_nonlinear_iteration_num,
					this->m_t_incremental_config.f_nonlinear_error_thresh,
					base_iface::r_GetBase(this->m_r_system.r_Edge_Pool()[this->m_r_system.r_Edge_Pool().n_Size() - 1]));
				// see if we need to optimize

				if(!b_optimized && m_b_outstanding_loop_closures) {
					Optimize(this->m_t_incremental_config.n_max_nonlinear_iteration_num,
						this->m_t_incremental_config.f_nonlinear_error_thresh);
				}
				// in case the solver avoided optimization, force it
			}
		}
	}

	/**
	 *	@brief measurement change on the last edge
	 *
	 *	This is used in incremental odometry scenario where the robot did not make enough movement in the last edge,
	 *	and we wish to include the next step in the same edge, rather than appending a new vertex and a new edge.
	 *	In simple applications, testing whether the edge has sufficient displacement before adding it into the system
	 *	in the first place might be enough, and this function is then not needed. However, when we want to calculate
	 *	information gain or some other probabilistic quantity, the edge may need to first be inserted to the system,
	 *	and only after that can we decide whether to keep it or discard it. At this point, SLAM++ does not allow removal
	 *	of edges or vertices, and that is where this function comes in.
	 *
	 *	@tparam _TyEdge is edge type
	 *	@tparam _TyMeasurement is measurement vector type (must be convertible to Eigen::Matrix of appropriate dimensions)
	 *	@tparam _TySigmaInv is information matrix type (must be convertible to Eigen::Matrix of appropriate dimensions)
	 *	@tparam _TyVertexState is vertex state type (a vector; must be convertible to Eigen::Matrix of appropriate dimensions)
	 *
	 *	@param[in] r_last_edge is the edge to be updated (must be currently the last edge in the system)
	 *	@param[in] r_v_new_delta is value of the new measurement (replaces the old measurement in the edge, is not additive)
	 *	@param[in] r_t_new_sigma is value of the new measurement information matrix (replaces the old one)
	 *	@param[in] r_t_new_vertex_state is value of the new vertex state (applied to vertex with index 1 in
	 *		the specified edge, or vertex 0 in case the edge is unary)
	 *	@param[in] b_do_omega_update is omega update flag (if set, the marginals are updated incrementally
	 *		when possible. if not set, marginals are always recalculated from scratch)
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when R is not-pos-def).
	 *	@note This is called in place of Incremental_Step(). Incremental_Step() does not need to be called afterwards.
	 *	@note This is an experimental feature and there are sometimes numberical issues when performing incremental updates.
	 */
	template <class _TyEdge, class _TyMeasurement, class _TySigmaInv, class _TyVertexState>
	void Change_LastEdge(_TyEdge &r_last_edge, const _TyMeasurement &r_v_new_delta,
		const _TySigmaInv &r_t_new_sigma, const _TyVertexState &r_t_new_vertex_state, bool b_do_omega_update = true) // throw(std::bad_alloc, std::runtime_error)
	{
		_TyTimeSampler timer(this->m_timer);

		_ASSERTE(!this->m_r_system.r_Edge_Pool().b_Empty()); // the system is not empty
		_ASSERTE(r_last_edge.n_Order() == this->m_r_system.r_Edge_Pool()[this->m_r_system.r_Edge_Pool().n_Size() - 1].n_Order()); // the last edge is the last
		_ASSERTE(m_n_lambda_block_ordering_size == this->m_r_system.r_Vertex_Pool().n_Size()); // make sure the ordering is up-to-date
		_ASSERTE(m_b_R_up_to_date && m_b_R_updatable); // make sure R is up to date with lambda and we can actually increment
		_ASSERTE(m_n_verts_in_lambda == this->m_r_system.r_Vertex_Pool().n_Size());
		_ASSERTE(m_n_edges_in_lambda == this->m_r_system.r_Edge_Pool().n_Size()); // make sure the system was not inconsistently incremented, after all, this is called by the end user, not from SLAM++ console application
		// prerequisites

		_ASSERTE(!m_b_inhibit_optimization);
		// currently one edge at a time, whixh makes sense with our use case. otherwise this
		// function would get slightly more complicated (especially the interface of it)

		const size_t n_refresh_from_edge = this->m_r_system.r_Edge_Pool().n_Size() - 1;
		// refreshing the last edge

		const fL_util::TGraphIncrementInfo gi(n_refresh_from_edge, this->m_r_system.r_Edge_Pool(),
			m_p_lambda_block_ordering, m_n_lambda_block_ordering_size);
		const size_t n_order_max = m_lambda.n_BlockColumn_Num();
		// calculate graph increment info

		timer.Accum_DiffSample(m_f_ordering_time);
		// stats

		CUberBlockMatrix delta_omega;
		/*if(gi.b_identity_perm)*/ { // need this for marginals update even if not identity
			fL_util::Calculate_Omega(delta_omega, gi.b_identity_perm, n_refresh_from_edge, gi.n_vertex_min,
				gi.n_order_min, this->m_r_system.r_Edge_Pool(), m_lambda, m_lambda_perm,
				m_p_lambda_block_ordering, m_n_lambda_block_ordering_size);
			//delta_omega.Rasterize("omega_old.tga");
			// calculate ordered omega

			timer.Accum_DiffSample(m_f_r11_omega_calc_time);
		}
		// calculates the omega matrix

		{
			r_last_edge.Update(r_v_new_delta, r_t_new_sigma);
			// update the edge

			size_t n_updated_vert = r_last_edge.n_Vertex_Id(std::min(size_t(1), r_last_edge.n_Vertex_Num() - 1)); // 0 for unary edges, 1 for all other edges
			if(this->m_r_system.r_Vertex_Pool()[n_updated_vert].b_IsConstant())
				throw std::runtime_error("constant vertices cannot be updated");
			this->m_r_system.r_Vertex_Pool()[n_updated_vert].v_State() = r_t_new_vertex_state;
			// update the new vertex // todo - this likely needs a better interface, or needs to be done before calling this function manually, as omega does not depend on it

			_ASSERTE(!m_b_system_dirty); // otherwise we might disturb something
			//if(!m_b_system_dirty)
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda, 0/*m_n_verts_in_lambda*/, n_refresh_from_edge); // calculate only for the affected edges
			/*else
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda);*/ // calculate for entire system, rebuild R from scratch
			// refresh lambda (does not fully refresh permutated lambda, even though it is a reference matrix)
			// this forces hessian recalculation in the edge

			timer.Accum_DiffSample(m_f_lambda_refresh_time);
			// stats

			_ASSERTE(m_lambda.n_BlockColumn_Num() == m_n_lambda_block_ordering_size);
			_ASSERTE(CMatrixOrdering::b_IsValidOrdering(m_p_lambda_block_ordering, m_n_lambda_block_ordering_size));
			// make sure that the ordering is good

			m_lambda.Permute_UpperTriangular_To(m_lambda_perm, m_p_lambda_block_ordering,
				m_lambda.n_BlockColumn_Num(), true);
			// make a reordered version of lambda (*always* changes if lambda changes)

			timer.Accum_DiffSample(m_f_ordering_time);
			// stats
		}

		/*if(gi.b_identity_perm)*/ { // need this for marginals update even if not identity
			CUberBlockMatrix omega_new;
			fL_util::Calculate_Omega(omega_new, gi.b_identity_perm, n_refresh_from_edge, gi.n_vertex_min,
				gi.n_order_min, this->m_r_system.r_Edge_Pool(), m_lambda, m_lambda_perm,
				m_p_lambda_block_ordering, m_n_lambda_block_ordering_size);
			//omega_new.Rasterize("omega_new.tga");
			// calculates the omega matrix after the update

			omega_new.AddTo(delta_omega, -1, +1); // this was wrong; could be FBS
			//delta_omega.Rasterize("omega_delta.tga");
			// delta_omega = delta_omega * -1 + omega_new * +1

			timer.Accum_DiffSample(m_f_r11_omega_calc_time);
		}
		// finalize delta omega

		if(n_refresh_from_edge && gi.n_order_min > 0) { // in case we loop back to vertex 0, we have to do a full update
			CUberBlockMatrix R11TR11;
			if(gi.b_identity_perm) {
				++ m_n_omega_update_num;
				// count them

				CUberBlockMatrix /*omega,*/ R11;
				/*size_t n_elem_order_min = m_lambda_perm.n_BlockColumn_Base(n_order_min);
				this->m_r_system.r_Edge_Pool().For_Each(n_refresh_from_edge, this->m_r_system.r_Edge_Pool().n_Size(),
					CCalculateOmega(omega, n_elem_order_min));
				omega.CheckIntegrity();*/
				const CUberBlockMatrix &omega = delta_omega; // already have omega .. of sorts

				timer.Accum_DiffSample(m_f_r11_omega_calc_time);

				m_R.SliceTo(R11, gi.n_order_min, n_order_max, gi.n_order_min, n_order_max, true); // row(0 - min) x col(min - max)
				// calculate the omega matrix (ho, ho, ho) and slice R11

				timer.Accum_DiffSample(m_f_r11_omega_slice_time);

				if(n_order_max - gi.n_order_min >= __NONLINEAR_SOLVER_FAST_L_PARALLEL_MATMULT_THRESH) // big one // t_odo this never runs, the limit for using R is also 100
					R11.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyLambdaMatrixBlockSizes>(R11TR11, true); // calculate R11^T * R11 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
				else
					R11.PreMultiplyWithSelfTransposeTo_FBS<_TyLambdaMatrixBlockSizes>(R11TR11, true); // calculate R11^T * R11 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
				// calculate R11TR11

				timer.Accum_DiffSample(m_f_r11_omega_ata_time);

				bool UNUSED(b_result) = omega.AddTo_FBS<_TyLambdaMatrixBlockSizes>(R11TR11); // todo - maybe also parallel
				_ASSERTE(b_result); // if the block order in omega was wrong, this would fail
				// calculate R11TR11_new = R11TR11 + omega
				// note this uses faster addition algorithm

				timer.Accum_DiffSample(m_f_r11_omega_add_time);
			}

			CUberBlockMatrix R01TR01;
			if(!gi.b_identity_perm) {
				CUberBlockMatrix lambda11, R01;
				++ m_n_lambda_update_num;
				// count them

				m_R.SliceTo(R01, 0, gi.n_order_min, gi.n_order_min, n_order_max, true); // row(0 - min) x col(min - max)
				m_lambda_perm.SliceTo(lambda11, gi.n_order_min, n_order_max, gi.n_order_min, n_order_max, true);

				timer.Accum_DiffSample(m_f_r11_lambda_slice_time);

				if(n_order_max - gi.n_order_min >= __NONLINEAR_SOLVER_FAST_L_PARALLEL_MATMULT_THRESH) // big one // t_odo this never runs, the limit for using R is also 100
					R01.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyLambdaMatrixBlockSizes>(R01TR01, true); // calculate R01^T * R01 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
				else
					R01.PreMultiplyWithSelfTransposeTo_FBS<_TyLambdaMatrixBlockSizes>(R01TR01, true); // calculate R01^T * R01 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?

				timer.Accum_DiffSample(m_f_r11_lambda_ata_time);

				lambda11.AddTo_FBS<_TyLambdaMatrixBlockSizes>(R01TR01, -1, 1); // t_odo - use FBS // todo - maybe also parallel
				// calculates R01TR01 = -R01TR01 + lambda11 (note the "-1, 1" is correct, the opposite way it crashes)
				// note that lambda11 is upper triangular, as well as R01TR01

				timer.Accum_DiffSample(m_f_r11_lambda_add_time);
			}

			Refresh_R11(gi.n_order_min, n_order_max, (gi.b_identity_perm)? R11TR11 : R01TR01);

			timer.Accum_DiffSample(m_f_Rupdate_time);

			Refresh_d_IncR11(n_refresh_from_edge, gi.n_order_min); // use the function, do not repeat code, it is ...
			// note that this contains its own timing inside
		} else {
			++ m_n_full_R_num;
			Refresh_R_FullR();
			// there is only a single edge, do not do incremental update, do batch
		}

		_ASSERTE(m_b_R_up_to_date && m_b_R_updatable); // should be, right?
		// and that should be it. now the marginals:

#if 1 // can't, have explicit omega
		if(this->m_t_marginals_config.b_calculate /*&& b_incremented*/) {
			// todo - handle freq settings (now they are tied to b_incremented - same as system) // i guess this does not really apply here
			// todo - handle policies

			if(!m_b_R_up_to_date)
				Optimize(0, 0); // R is not up to date // it is ok, linearization point cant be changed here
			_ASSERTE(m_b_R_up_to_date && m_b_R_updatable);
			// make sure that R is up to date and we can readily calculate the marginals

			double f_dummy;
			timer.Accum_DiffSample(f_dummy);
			// there were some samples taken in between

			//m_R.Rasterize("R_for_margs.tga"); // debug
			size_t n_add_edge_num = this->m_r_system.r_Edge_Pool().n_Size() - this->m_marginals.n_Edge_Num();
			bool b_incremental = b_do_omega_update && this->m_marginals.b_CanUpdate() && !n_add_edge_num; // omega always available, can update only if the marginals contain all the edges - otherwise the omega is not the correct omega
			//_ASSERTE(b_incremental); // should be always incremental, right? // not now, we don't know how to calculate it
			if(b_incremental) { // otherwise just update what we have
				CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
				if(!CMarginals::Update_BlockDiagonalMarginals_FBS_ExOmega<false>(this->m_r_system, r_m,
				   m_lambda, m_R, m_lambda_ordering, this->m_marginals.n_Edge_Num(),
				   this->m_t_marginals_config.n_incremental_policy, 0/*f_omega_build_time*/,
				   delta_omega, gi.incremented_lambda_perm_block_column_list)) { // new shiny better
				/*if(!CMarginals::Update_BlockDiagonalMarginals_FBS<false>(this->m_r_system, r_m, m_lambda,
				   m_R, m_lambda_ordering, this->m_marginals.n_Edge_Num(), this->m_t_marginals_config.n_incremental_policy)) {*/ // would not work, as there is no new edge and it wouldn't know by how much to increment / decrement
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
			if(!b_incremental) { // if need batch marginals
				CUberBlockMatrix margs_ordered;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ordered, m_R,
					m_lambda_ordering, this->m_t_marginals_config.n_relinearize_policy, false);
				// calculate the thing

				CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
				margs_ordered.Permute_UpperTriangular_To(r_m, m_lambda_ordering.p_Get_Ordering(),
					m_lambda_ordering.n_Ordering_Size(), false); // no share! the original will be deleted
				this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
				// take care of having the correct permutation there

				this->m_marginals.EnableUpdate();
				// now the marginals are current and can be updated until the linearization point is changed again

				timer.Accum_DiffSample(m_f_marginals_time);
			}
			// update the marginals, simple and easy

			this->m_marginals.Set_Edge_Num(this->m_r_system.r_Edge_Pool().n_Size());
			// now all those edges are in the marginals

#if 0
			try {
				CUberBlockMatrix R;
				CMatrixOrdering mord;
				mord.p_BlockOrdering(m_lambda, true);
				const size_t *p_order = mord.p_Get_InverseOrdering();
				CUberBlockMatrix lambda_perm;
				m_lambda.Permute_UpperTriangular_To(lambda_perm, p_order, mord.n_Ordering_Size(), true);
				if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm))
					throw std::runtime_error("fatal error: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed");
				// want to use from-the-scratch-reference R for comparison

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
					printf("\tthis calls for special attention ...\n");
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
			// check marginals against reference
		}
#else // 1
		TryMarginals(); // can't, have explicit omega
#endif // 1
		// handle also update of the marginals
	}

	/**
	 *	@brief incremental optimization function
	 *	@param[in] r_last_edge is the last edge that was added to the system
	 *	@note This function throws std::bad_alloc and std::runtime_error (when R is not-pos-def).
	 */
	void Incremental_Step(_TyBaseEdge &UNUSED(r_last_edge)) // throw(std::bad_alloc, std::runtime_error)
	{
		/*_ASSERTE(!this->m_r_system.r_Edge_Pool().b_Empty());
		typename _TyEdgeMultiPool::_TyConstBaseRef r_edge =
			this->m_r_system.r_Edge_Pool()[this->m_r_system.r_Edge_Pool().n_Size() - 1];
		// get a reference to the last edge interface

		size_t n_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_had_loop_closure) {
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
		}*/
		m_b_outstanding_loop_closures |= this->b_Detect_LoopClosures(r_last_edge);
		// detect loop closures (otherwise the edges are initialized based on measurement and error would be zero)

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
		{
			_ASSERTE(r_edge.n_Vertex_Num() == 2); // won't work with hyperedges, would have to modify
			_ASSERTE(std::max(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1)) <
				this->m_r_system.r_Vertex_Pool().n_Size()); // won't work with const vertices, would have to modify if needed
			FILE *p_fw = fopen("timeSteps_R.txt", (this->m_n_real_step > 0)? "a" : "w");
			fprintf(p_fw, "" PRIsize ";%f;" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";"
				PRIsize ";" PRIsize "\n", this->m_n_real_step, this->m_timer.f_Time(),
				m_n_Rup_num, m_n_full_R_num, m_n_R_optim_num, m_n_lambda_optim_num,
				std::max(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1)) -
				std::min(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1)),
				m_n_loop_size_cumsum);
			fclose(p_fw);
		}
		// dump time per step
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS

#if defined(__NONLINEAR_SOLVER_FAST_L_DUMP_WAFR2014_PRESENTATION_ANIMATION_DATA) || \
	defined(__NONLINEAR_SOLVER_FAST_L_DUMP_CHI2) || defined(__NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY)
		bool b_new_vert = m_lambda.n_BlockColumn_Num() < this->m_r_system.r_Vertex_Pool().n_Size();
#endif /* __NONLINEAR_SOLVER_FAST_L_DUMP_WAFR2014_PRESENTATION_ANIMATION_DATA ||
	__NONLINEAR_SOLVER_FAST_L_DUMP_CHI2 || __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY */

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
		if(b_new_vert) {
			FILE *p_fw = fopen("chi2perVert.txt", (this->m_n_real_step > 0)? "a" : "w");
			double f_chi2 = 0;
			do {
				if(this->m_r_system.r_Edge_Pool().b_Empty())
					break;
#ifndef __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
				if(m_lambda.n_BlockColumn_Num() != this->m_r_system.r_Vertex_Pool().n_Size()) {
#pragma error "this might not work, before lambda is too small and after it is too big"
					Optimize(0, 0);
					// optimize but don't allow iterations - just updates lambda, d and R
					// in order to be able to generate approximate solutions on request
				}
#endif // !__NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
				if(m_b_linearization_dirty) {
					typedef CNonlinearSolver_FastL<CSystem, CLinearSolver, CAMatrixBlockSizes> TThisType;
					TThisType *p_this = const_cast<TThisType*>(this); // don't want to put 'mutable' around everything
					if(!p_this->CalculateOneTimeDx(1)) // this is one smaller
						break;
					p_this->this->m_r_system.r_Vertex_Pool().For_Each_Parallel(0,
						this->m_r_system.r_Vertex_Pool().n_Size() - 1, nonlinear_detail::CSolverOps_Base::CUpdateEstimates(m_v_dx)); // ignore the last vertex
					f_chi2 = this->m_r_system.r_Edge_Pool().For_Each(0,
						this->m_r_system.r_Edge_Pool().n_Size() - 1, nonlinear_detail::CSolverOps_Base::CSum_ChiSquareError()); // ignore the last edge
					p_this->this->m_r_system.r_Vertex_Pool().For_Each_Parallel(0,
						this->m_r_system.r_Vertex_Pool().n_Size() - 1, nonlinear_detail::CSolverOps_Base::CUpdateEstimates(-m_v_dx)); // !!
					break;
				} else {
					f_chi2 = this->m_r_system.r_Edge_Pool().For_Each(0,
						this->m_r_system.r_Edge_Pool().n_Size() - 1, nonlinear_detail::CSolverOps_Base::CSum_ChiSquareError());
				}
			} while(0);
			// calculate chi2, excluding the last edge (the current one)

			fprintf(p_fw, "%f\n", f_chi2);
			fclose(p_fw);
		}
		// dump chi2
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2

		bool b_incremented = TryOptimize(this->m_t_incremental_config.n_max_nonlinear_iteration_num,
			this->m_t_incremental_config.f_nonlinear_error_thresh, r_last_edge);
		// optimize

		//++ m_n_real_step;
		// used only above
	}

	/**
	 *	@brief debugging functionality; makes sure that the R factor is correct
	 *	@param[in] b_allow_update is incremental update allowance flag (if set and the R factor is not
 	 *		up to date and can be incrementally updated, then it is updated; otherwise it is not updated)
	 *	@return Returns true only if the R factor is correct, returns false if there
	 *		is a difference or if the R factor could not be checked.
	 */
	bool Check_Factor(bool b_allow_update = false)
	{
		if(m_b_system_dirty) // can't check if lambda is not up to date
			return false; // could not be checked

		if(!m_b_R_up_to_date) {
			if(!m_b_R_updatable || !b_allow_update) // no reason to check fullR, that would be calculated the same way
				return false; // could not be checked

			//Optimize(0, 0); // R is not up to date // infinite recursion
			if(!RefreshLambdaR()) {
				if(this->m_marginals.r_SparseMatrix().n_BlockColumn_Num() !=
				   this->m_r_system.r_Vertex_Pool().n_Size()) {
					fprintf(stderr, "error: could not update R, marginals not updated\n");
					// should we throw? this is probably caused by having not enough edges, it should pass
				} else
					fprintf(stderr, "warning: could not update R, marginals not updated\n");
				return false; // cant calculate. better err here.
			}
		}

		// R was up to date, or was updated incrementally - can test it
		// (otherwise it was calculated from scratch, no sense in checking that)

		CUberBlockMatrix R;
		CUberBlockMatrix lambda_perm;
		m_lambda.Permute_UpperTriangular_To(lambda_perm, m_p_lambda_block_ordering,
			m_lambda.n_BlockColumn_Num(), true);
		if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm)) {
			fprintf(stderr, "warning: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed in Check_R()\n");
			return false;
		} else {
			{
				Eigen::VectorXd v_rhs(R.n_Column_Num());
				Eigen::VectorXd v_perm_temp(R.n_Column_Num());
				Eigen::VectorXd v_d(R.n_Column_Num());

				_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, v_rhs);
				// collects the right-hand side vector (eta)

				_ASSERTE(m_p_lambda_block_ordering);
				lambda_perm.InversePermute_LeftHandSide_Vector(&v_perm_temp(0), &v_rhs(0), v_rhs.rows(),
					m_p_lambda_block_ordering, lambda_perm.n_BlockRow_Num());
				bool b_uttsolve_result = R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&v_perm_temp(0), v_perm_temp.rows());
				lambda_perm.Permute_LeftHandSide_Vector(&v_d(0), &v_perm_temp(0), v_rhs.rows(),
					m_p_lambda_block_ordering, lambda_perm.n_BlockRow_Num());
				// d = eta = eta/R

				if(!b_uttsolve_result) {
					fprintf(stderr, "warning: solving for d failed\n");
					return false;
				}

				{
					double f_mag = v_d.norm();
					if(m_v_d.rows() != v_d.rows()) {
						fprintf(stderr, "warning: d addition failed, shape incorrect?\n");
						return false;
					}
					double f_err = (v_d - m_v_d).norm();
					if(f_err > 1e-5 && f_err / f_mag > 1e-5) {
						printf("warning: d is drifting away (%g abs error, %g rel error)\n", f_err, f_err / f_mag);
						return false;
					}
				}
			}
			// need to check d before we destroy R by subtracting the current R from it

			double f_mag = R.f_Norm();
			if(!m_R.AddTo(R, -1)) {
				fprintf(stderr, "warning: R addition failed, shape incorrect?\n");
				return false;
			}
			double f_err = R.f_Norm();
			if(f_err / f_mag > 1e-5) {
				printf("warning: R is drifting away (%g rel error)\n", f_err / f_mag);
				return false;
			}
		}
		// want to use from-the-scratch-reference R for comparison

		return true;
	}

	/**
	 *	@brief debugging functionality; makes sure that the marginals are up to date
	 *	@return Returns true if the matginals are up to date, returns false if there
	 *		is a difference or if the marginals could not be checked.
	 */
	bool Check_Marginals()
	{
		if(!this->m_t_marginals_config.b_calculate)
			return true;

#ifdef _DEBUG
		bool b_do_check_R = m_b_R_up_to_date || m_b_R_updatable;
#endif // _DEBUG

		if(!m_b_R_up_to_date) {
			//Optimize(0, 0); // R is not up to date // infinite recursion
			if(!RefreshLambdaR()) {
				if(this->m_marginals.r_SparseMatrix().n_BlockColumn_Num() !=
				   this->m_r_system.r_Vertex_Pool().n_Size()) {
					fprintf(stderr, "error: could not update R, marginals not updated\n");
					// should we throw? this is probably caused by having not enough edges, it should pass
				} else
					fprintf(stderr, "warning: could not update R, marginals not updated\n");
				return false; // cant calculate, possibly out of data marginals. better err here.
			}
		}

#ifdef _DEBUG
		if(b_do_check_R) {
			// R was up to date, or was updated incrementally - can test it
			// (otherwise it was calculated from scratch, no sense in checking that)

			CUberBlockMatrix R;
			CUberBlockMatrix lambda_perm;
			m_lambda.Permute_UpperTriangular_To(lambda_perm, m_p_lambda_block_ordering,
				m_lambda.n_BlockColumn_Num(), true);
			if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm))
				fprintf(stderr, "warning: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed in Check_R()\n");
			else {
				double f_mag = R.f_Norm();
				m_R.AddTo(R, -1);
				double f_err = R.f_Norm();
				if(f_err / f_mag > 1e-5)
					fprintf(stderr, "warning: R is drifting away (%g rel error)\n", f_err / f_mag);
			}
			// want to use from-the-scratch-reference R for comparison
		}
#endif // _DEBUG

		_TyTimeSampler timer(this->m_timer);

		CUberBlockMatrix margs_ordered;
		CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ordered, m_R,
			m_lambda_ordering, this->m_t_marginals_config.n_relinearize_policy, false);
		// calculate the thing

		CUberBlockMatrix margs_batch;
		margs_ordered.Permute_UpperTriangular_To(margs_batch, m_lambda_ordering.p_Get_Ordering(),
			m_lambda_ordering.n_Ordering_Size(), false); // no share! the original will be deleted
		// take care of having the correct permutation there

		timer.Accum_DiffSample(m_f_marginals_time);

		const CUberBlockMatrix &r_m = this->m_marginals.r_SparseMatrix();
		// get the marginals

		Eigen::VectorXd v_diag_ref, v_diag;
		margs_batch.Get_Diagonal(v_diag_ref);
		r_m.Get_Diagonal(v_diag);
		// todo - assert that the policy involves calculating the diagonal

		if(v_diag_ref.rows() != v_diag.rows())
			return false;
		// well ...

		double f_error = (v_diag - v_diag_ref).norm() / v_diag_ref.norm();
		// calculate error in the diagonal

		bool b_ok = f_error < 1e-5;
		// make sure the error is low

		if(!b_ok) {
				fprintf(stderr, "warning: marginals imprecise (%g rel error)\n", f_error);
			r_m.Rasterize("marginals_cur.tga");
			margs_batch.Rasterize("marginals_ref.tga");
			r_m.AddTo(margs_batch, -1);
			margs_batch.Rasterize("marginals_diff.tga");
		}
		// debug - save the matrices, see whats wrong

		return b_ok;
	}

protected:
	void TryMarginals()
	{
		if(this->m_t_marginals_config.b_calculate /*&& b_incremented*/) {
			// todo - handle freq settings (now they are tied to b_incremented - same as system)
			// todo - handle policies

			if(!m_b_R_up_to_date) {
				//Optimize(0, 0); // R is not up to date // infinite recursion
				if(!RefreshLambdaR()) {
					if(this->m_marginals.r_SparseMatrix().n_BlockColumn_Num() !=
					   this->m_r_system.r_Vertex_Pool().n_Size()) {
						fprintf(stderr, "error: could not update R, marginals not updated\n");
						// should we throw? this is probably caused by having not enough edges, it should pass
					} else
						fprintf(stderr, "warning: could not update R, marginals not updated\n");
					return;
				}
			}
			_ASSERTE(m_b_R_up_to_date && m_b_R_updatable);
			// make sure that R is up to date and we can readily calculate the marginals

			_TyTimeSampler timer(this->m_timer);
#if 0
			int firstvertexdim = this->m_r_system.r_Vertex_Pool()[0].n_Dimension();
			Eigen::MatrixXd marg_mat(m_R.n_Column_Num(), firstvertexdim);
			//CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
			//CMarginals::Calculate_DenseMarginals_Fast_ColumnBand_FBS<_TyLambdaMatrixBlockSizes>(marg_mat,
			//		m_R, 0, firstvertexdim, (m_lambda_ordering.p_Get_Ordering()), (m_lambda_ordering.n_Ordering_Size()), false);
			CMarginals::Calculate_DenseMarginals_Fast_ColumnBand_FBS<_TyLambdaMatrixBlockSizes>(marg_mat.block(0, 0, m_R.n_Column_Num(), firstvertexdim),
					m_R, 0, (m_lambda_ordering.p_Get_Ordering()), (m_lambda_ordering.n_Ordering_Size()), (m_lambda_ordering.n_Ordering_Size()));
			//this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed

			timer.Accum_DiffSample(m_f_incmarginals_time);
			++ m_n_incmarginals_num;
			// calculate the thing
#elif 0
			CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
			CMarginals::Calculate_DenseMarginals_Recurrent_FBS
				/*Calculate_DenseMarginals_Fast_Parallel_FBS*/<_TyLambdaMatrixBlockSizes>(
				r_m, m_R/*, p_order, mord.n_Ordering_Size()*/, mord, mpart_Diagonal);
			this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
			// calculate the thing
#endif // 0
			// don't want slow dense marginals
#if 1
			//m_R.Rasterize("R_for_margs.tga"); // debug
			size_t n_add_edge_num = this->m_r_system.r_Edge_Pool().n_Size() - this->m_marginals.n_Edge_Num();
			bool b_incremental = this->m_marginals.b_CanUpdate() && CMarginals::b_PreferIncremental(this->m_r_system,
				this->m_marginals.r_SparseMatrix(), m_lambda, m_R, m_lambda_ordering, this->m_marginals.n_Edge_Num(),
				this->m_t_marginals_config.n_incremental_policy);
			if(b_incremental) { // otherwise just update what we have
				CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
				if(!CMarginals::Update_BlockDiagonalMarginals_FBS<false>(this->m_r_system, r_m, m_lambda,
				   m_R, m_lambda_ordering, this->m_marginals.n_Edge_Num(), this->m_t_marginals_config.n_incremental_policy)) {
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
			if(!b_incremental) { // if need batch marginals
				CUberBlockMatrix margs_ordered;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ordered, m_R,
					m_lambda_ordering, this->m_t_marginals_config.n_relinearize_policy, false);
				// calculate the thing

				CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
				margs_ordered.Permute_UpperTriangular_To(r_m, m_lambda_ordering.p_Get_Ordering(),
					m_lambda_ordering.n_Ordering_Size(), false); // no share! the original will be deleted
				this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
				// take care of having the correct permutation there

				this->m_marginals.EnableUpdate();
				// now the marginals are current and can be updated until the linearization point is changed again

				timer.Accum_DiffSample(m_f_marginals_time);
			}
			// update the marginals, simple and easy

			this->m_marginals.Set_Edge_Num(this->m_r_system.r_Edge_Pool().n_Size());
			// now all those edges are in the marginals
#endif // 0
#if 0
			try {
				CUberBlockMatrix R;
				CMatrixOrdering mord;
				mord.p_BlockOrdering(m_lambda, true);
				const size_t *p_order = mord.p_Get_InverseOrdering();
				CUberBlockMatrix lambda_perm;
				m_lambda.Permute_UpperTriangular_To(lambda_perm, p_order, mord.n_Ordering_Size(), true);
				if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm))
					throw std::runtime_error("fatal error: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed");
				// want to use from-the-scratch-reference R for comparison

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
					printf("\tthis calls for special attention ...\n");
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
			// check marginals against reference

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
			// dump diagonal blocks of the marginals to a file
		}
		// now R is up to date, can get marginals

#ifndef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
		if(b_new_vert) {
			FILE *p_fw = fopen("chi2perVert.txt", (this->m_n_real_step > 1)? "a" : "w"); // m_n_real_step already incremented at this point
			fprintf(p_fw, "%f\n", f_Chi_Squared_Error_Denorm());
			fclose(p_fw);
		}
		// dump chi2
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY
		if(b_new_vert)
			Dump_RDensity();
		// dump nnz of R, given different ordering strategies
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_WAFR2014_PRESENTATION_ANIMATION_DATA
		if(b_new_vert) {
			_ASSERTE(m_R.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size());
			char p_s_filename[256];
			cs *p_R = m_R.p_BlockStructure_to_Sparse(); // !!
			_ASSERTE(p_R->m == p_R->n);
			_ASSERTE(p_R->m == this->m_r_system.r_Vertex_Pool().n_Size());
			if(m_R.n_BlockColumn_Num() == 2) { // the first time around
				system("mkdir R");
				system("mkdir sys");
			}
			sprintf(p_s_filename, "R/LDump" PRIsize ".pig", this->m_r_system.r_Vertex_Pool().n_Size());
			FILE *p_fw = fopen(p_s_filename, "wb");
			uint32_t n_iter_num = m_n_iteration_num, n = p_R->n, nnz = p_R->p[p_R->n]; // to detect LP changes
			fwrite(&n_iter_num, sizeof(uint32_t), 1, p_fw); // write number of solver iters at this point
			fwrite(&n, sizeof(uint32_t), 1, p_fw); // write matrix size
			fwrite(&nnz, sizeof(uint32_t), 1, p_fw); // write #nnz
			fwrite(p_R->p, sizeof(csi), p_R->n + 1, p_fw); // write row pointers
			fwrite(p_R->i, sizeof(csi), nnz, p_fw); // write indices
			fclose(p_fw);
			cs_spfree(p_R);
			sprintf(p_s_filename, "sys/sys%05" _PRIsize ".txt", this->m_r_system.r_Vertex_Pool().n_Size());
			this->m_r_system.Dump(p_s_filename); // this also
		}
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_WAFR2014_PRESENTATION_ANIMATION_DATA
	}

	/**
	 *	@brief optimization function with optimization decision
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 *	@param[in] r_last_edge is the edge to be updated (must be currently the last edge in the system)
	 *
	 *	@return Returns true if optimization triggered (number of new vertices
	 *		reached (non)linear solve threshold), otherwise returns false.
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when R is not-pos-def).
	 */
	bool TryOptimize(size_t n_max_iteration_num, double f_min_dx_norm, _TyBaseEdge &UNUSED(r_last_edge)) // throw(std::bad_alloc, std::runtime_error)
	{
		/*bool b_optimization_triggered = false;

		size_t n_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_inhibit_optimization) {
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			size_t n_new_vertex_num = n_vertex_num - m_n_last_optimized_vertex_num;
			if(!n_new_vertex_num) { // evidently, this backfires; need to have another m_n_last_optimized_vertex_num where it would remember when was the system last extended and check if there are new vertices since *then* // fixed now
				m_b_R_up_to_date = false; // now missing some edge
				//m_b_R_updatable = m_b_R_updatable; // this does not change. if it was updatable before, it still is
				return false; // no new vertices; don't go in ... (otherwise 2x slower on molson35, for obvious reasons)
			}
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
				f_min_dx_norm = this->m_t_incremental_config.f_nonlinear_error_thresh; // right?
				b_optimization_triggered = true;
				// simple optimization
			}
		}*/

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size() -
			m_n_verts_in_lambda/*this->n_LastOptimized_Vertex_Num()*/;
		// this was buggy, using this->n_LastOptimized_Vertex_Num() yields nonzero number of new
		// vertices when running e.g. each 10 (each n, n > 1) after the first new vertex appeared
		// and stays nonzero until 10 new vertices are added, effectively updating after each edge
		// then, making each 10 much slower than each 1
		if(!n_new_vertex_num) { // would use t_optimize.first, but then it would fail if the frequency would be larger than 1 (e.g. each 10 vertices)
			m_b_R_up_to_date = false; // now missing some edge
			//m_b_R_updatable = m_b_R_updatable; // this does not change. if it was updatable before, it still is
			return false; // no new vertices; don't go in ... (otherwise 2x slower on molson35, for obvious reasons)
		}
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		// in case there are no new vertices, wait with counting new vertices
		// do this before calling this->t_Incremental_Step()

		bool b_new_loops = this->b_Detect_LoopClosures(r_last_edge);
		_ASSERTE(!b_new_loops || m_b_outstanding_loop_closures); // no new ones should be detected here, we already tried before
		m_b_outstanding_loop_closures |= b_new_loops;
		std::pair<bool, int> t_optimize = this->t_Incremental_Step(r_last_edge, true); // trigger even without loop closures, will handle them differently
		//m_b_outstanding_loop_closures |= this->b_Had_LoopClosures(); // none can be added here, t_Incremental_Step() would have cleared the flag
		// run the loop closure detector

		bool b_optimization_triggered = false;
		if(!m_b_inhibit_optimization) {
			if(t_optimize.second == 1) {
				n_max_iteration_num = 1;
				f_min_dx_norm = this->m_t_incremental_config.f_nonlinear_error_thresh; // right?
			}
			b_optimization_triggered = t_optimize.second > 0;
		}
		// decide on incremental optimization

		if(!b_optimization_triggered) {
			size_t n_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size();
			if(this->m_t_marginals_config.b_calculate) {
				_ASSERTE(m_lambda.n_BlockColumn_Num() < n_vertex_num);
				// we already checked for this a little above, now we are sure
				// to be adding a new vertex, just that there is no loop closure

				Optimize(0, 0); // need to calculate marginals in any case
			} else {
#ifdef __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
				_ASSERTE(m_lambda.n_BlockColumn_Num() < n_vertex_num); // we already checked for this a little above, now we are sure to be adding a new vertex, just that there is no loop closure
				/*if(m_lambda.n_BlockColumn_Num() == n_vertex_num) {
					m_b_R_up_to_date = false; // now missing some edge
					//m_b_R_updatable = m_b_R_updatable; // this does not change. if it was updatable before, it still is
					return false;
				}*/
				// there is enough vertices in lambda, none would be added

				Optimize(0, 0); // big todo - remove this in order to be faster for each 100; move it to function that does approx solutions on request
				// optimize but don't allow iterations - just updates lambda, d and R
				// in order to be able to generate approximate solutions on request
#endif // __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
			}
		} else
			Optimize(n_max_iteration_num, f_min_dx_norm);

		return b_optimization_triggered;
	}

	/**
	 *	@brief refreshes system matrices lambda and R
	 *	@return Returns true if optimization should take place, otherwise returns false
	 */
	bool RefreshLambdaR()
	{
		_TyTimeSampler timer(this->m_timer);

		const size_t n_variables_size = this->m_r_system.n_VertexElement_Num();
		const size_t n_measurements_size = this->m_r_system.n_EdgeElement_Num();
		if(n_variables_size > n_measurements_size) {
			if(n_measurements_size)
				fprintf(stderr, "warning: the system is underspecified\n");
			else
				fprintf(stderr, "warning: the system contains no edges at all: nothing to optimize\n");
			return false;
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

		Extend_LambdaR(m_n_verts_in_lambda, m_n_edges_in_lambda);
		// recalculated all the jacobians inside Extend_LambdaR(), also extend R structurally

		if(!m_b_system_dirty)
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda, 0/*m_n_verts_in_lambda*/, m_n_edges_in_lambda); // calculate only for new edges // @todo - but how to mark affected vertices?
		else
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // calculate for entire system, rebuild R from scratch
		// refresh lambda (does not fully refresh permutated lambda, even though it is a reference matrix)

		if(m_lambda.n_BlockColumn_Num() < this->m_r_system.r_Vertex_Pool().n_Size()) {
			timer.Accum_DiffSample(m_f_lambda_refresh_time);
			fprintf(stderr, "warning: waiting for more edges\n");

			m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
			m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size();
			// probably breaks the solver otherwise

			m_b_R_up_to_date = false;
			m_b_R_updatable = false; // better be safe
			m_b_system_dirty = true;
			// force refresh next time atound

			return false; // breaks the L solver, R will not be correctly updated // t_odo
			// now it should work, but maybe it is not optimal (but anyway this only happens
			// at the beginning when the system is small, at least in our applications)
		}
		// waiting for more edges

		m_v_dx.resize(n_variables_size);
		m_v_perm_temp.resize(n_variables_size);
		if((m_b_R_up_to_date || m_b_R_updatable) && !m_b_system_dirty)
			m_v_d.conservativeResize(n_variables_size); // b_Refresh_R() also refreshes d (rhs), needs dx as temp // !!
		else
			m_v_d.resize(n_variables_size); // in case we're about to rebuild R from scratch, don't care about contents of d
		// resize the helper vectors

		timer.Accum_DiffSample(m_f_lambda_refresh_time);

		{
			try {
				if((m_b_R_up_to_date || m_b_R_updatable) && // can increment R only if up to date
				   !m_b_system_dirty) // avoidance of big incremental updates of R is inside b_Refresh_R() - can only decide if ordering is known
					m_b_first_iteration_use_R = b_Refresh_R(0/*m_n_verts_in_lambda*/, m_n_edges_in_lambda); // calculate only for new edges // @todo - but how to mark affected vertices?
				else
					m_b_first_iteration_use_R = b_Refresh_R(); // calculate for entire system, rebuild R from scratch
			} catch(std::runtime_error &r_exc) {
				if(!strcmp(r_exc.what(), "Factorize_PosDef_Blocky() failed to calculate full R")) { // could also use different exception types
					fprintf(stderr, "warning: %s\n", r_exc.what());
					m_b_R_up_to_date = false;
					m_b_R_updatable = false; // !!
					m_b_system_dirty = true;
					m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
					m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // probably needs to be here
					return false;
				} else
					throw r_exc; // rethrow
			}

			m_b_R_up_to_date = m_b_R_updatable = m_b_first_iteration_use_R;
			// in case R is not used, it will fall behind
		}
		// refresh R

		m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
		m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
		// not before R is refreshed

		m_b_system_dirty = false;
		_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
			m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
			m_lambda.n_Column_Num() == n_variables_size);
		_ASSERTE(!m_b_R_up_to_date || (m_R.n_Row_Num() == m_R.n_Column_Num() &&
			m_R.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
			m_R.n_Column_Num() == n_variables_size)); // lambda is square, blocks on either side = number of vertices
		// need to have lambda and perhaps also R

		return true;
	}

	/**
	 *	@brief updates the m_v_dx vector from the current R and d (or lambda eta)
	 *	@param[in] n_ignore_vertices is number of vertices at the end of the system to be ignored
	 *	@return Returns true on success, false on failure (numerical issues).
	 */
	bool CalculateOneTimeDx(size_t n_ignore_vertices = 0)
	{
		_ASSERTE(m_b_linearization_dirty); // this should only be called in case the linearization point was not updated

		if(m_b_R_up_to_date && m_b_first_iteration_use_R) { // Optimize() clears m_b_R_up_to_date but not m_b_first_iteration_use_R at the same time
			_ASSERTE(m_b_R_up_to_date);
			// we have R and can use it efficiently

			{
				bool b_cholesky_result;
				{
					_ASSERTE(m_p_lambda_block_ordering);
					m_lambda_perm.InversePermute_LeftHandSide_Vector(&m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows(),
						m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
					b_cholesky_result = m_R.UpperTriangular_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
					m_lambda_perm.Permute_LeftHandSide_Vector(&m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows(),
						m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
					// dx = R'/d // note this never fails (except if R is null)
				}
				// R solves with permutation (note that m_v_d is not modified!)
				// calculate cholesky, reuse block ordering if the linear solver supports it

				if(!b_cholesky_result)
					return false;

#ifdef _DEBUG
				nonlinear_detail::CSolverOps_Base::b_DetectNaNs(m_v_dx, true, "fL3 dx");
#endif // _DEBUG
			}
		} else {
			if(!n_ignore_vertices)
				_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_dx);
			else { // t_odo - see how this would work
				_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_dx,
					0, this->m_r_system.r_Vertex_Pool().n_Size() - n_ignore_vertices);
				/*this->m_r_system.r_Vertex_Pool().For_Each_Parallel(0,
					this->m_r_system.r_Vertex_Pool().n_Size() - n_ignore_vertices,
					_TyLambdaOps::CCollect_RightHandSide_Vector(m_v_dx));*/
			}
			// collects the right-hand side vector

			{
				bool b_cholesky_result;
				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					b_cholesky_result = this->m_linear_solver.Solve_PosDef(m_lambda, v_eta); // p_dx = eta = lambda / eta
					// dont reuse block ordering
				}
				// lambda is good without permutation (there is one inside and we save copying eta around)
				// calculate cholesky, reuse block ordering if the linear solver supports it

				if(!b_cholesky_result)
					return false;

#ifdef _DEBUG
				nonlinear_detail::CSolverOps_Base::b_DetectNaNs(m_v_dx, true, "fL0 dx");
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

		m_b_R_updatable = false; // !!
		m_b_system_dirty = true;
		// mark the system matrix as dirty, to force relinearization in the next step
	}

	/**
	 *	@brief final optimization function
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when R is not-pos-def).
	 */
	void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01) // throw(std::bad_alloc, std::runtime_error)
	{
		if(!RefreshLambdaR())
			return;
		// decide whether to optimize or not

#ifdef _DEBUG
		if(!Check_Factor())
			fprintf(stderr, "error: R, d incorrect at the beginning of Optimize()\n");
#endif // _DEBUG

		if(!n_max_iteration_num) {
			m_b_linearization_dirty = true;
			TryMarginals(); // !!
			return;
		}
		// in case we're not required to optimize, do nothing
		// (the user can still request solution, R is in good shape)

		if(m_b_outstanding_loop_closures/*m_b_had_loop_closure*/)
			m_b_outstanding_loop_closures/*m_b_had_loop_closure*/ = false;
		else {
			m_b_linearization_dirty = true;
			TryMarginals(); // !!
			return; // nothing to optimize, dx would be zero
		}
		// handle loop closures a bit differently

#if 0
		static bool b_had_lambda_up = false;
		if(m_b_first_iteration_use_R && b_had_lambda_up) {
			b_had_lambda_up = false;
			Check_RLambdaTracking(); // seems to be working now
		}
#endif // 0
		// make sure lambda and R contain the same system

		bool b_verbose = this->m_b_verbose;

		for(size_t n_iteration = 0; n_iteration < n_max_iteration_num; ++ n_iteration) {
			++ m_n_iteration_num;
			// debug

			if(b_verbose) {
				if(n_max_iteration_num == 1)
					printf("\n=== incremental optimization step ===\n\n");
				else
					printf("\n=== nonlinear optimization: iter #" PRIsize " ===\n\n", n_iteration);
			}
			b_verbose = this->m_b_verbose; // restore
			// verbose

			if(m_b_R_up_to_date/*m_b_first_iteration_use_R && !n_iteration*/) { // always when we have R
				++ m_n_R_optim_num;

				_ASSERTE(m_b_R_up_to_date);
				// we have R and can use it efficiently

				_TyTimeSampler timer(this->m_timer);

				{
					bool b_utsolve_result;
					{
						_ASSERTE(m_p_lambda_block_ordering);
						m_lambda_perm.InversePermute_LeftHandSide_Vector(&m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows(),
							m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
						b_utsolve_result = m_R.UpperTriangular_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
						m_lambda_perm.Permute_LeftHandSide_Vector(&m_v_dx(0), &m_v_perm_temp(0), m_v_dx.rows(),
							m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
						// dx = R'/d // note this never fails (except if R is null)

						if(this->m_b_verbose) {
							printf("%s", (b_utsolve_result)? "backsubstitution succeeded\n" :
								"backsubstitution failed\n");
						}
					}
					// R solves with permutation (note that m_v_d is not modified!)

					// calculate cholesky, reuse block ordering if the linear solver supports it

					timer.Accum_DiffSample(m_f_backsubst_time);

#ifdef _DEBUG
					nonlinear_detail::CSolverOps_Base::b_DetectNaNs(m_v_dx, true, "fL1 dx");
#endif // _DEBUG

					double f_residual_norm = 0;
					if(b_utsolve_result) {
						f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
						if(this->m_b_verbose)
							printf("residual norm: %.4f\n", f_residual_norm);
					}
					// calculate residual norm

					timer.Accum_DiffSample(m_f_norm_time);

					if(f_residual_norm <= f_min_dx_norm) {
						m_b_linearization_dirty = true;
						break;
					}
					// in case the error is low enough, quit (saves us recalculating the hessians)

					if(b_utsolve_result) {
						/*printf("just optimized using R\n");*/
						nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, m_v_dx); // note this kills R
						m_b_system_dirty = true;
						m_b_R_up_to_date = false; // !!
						m_b_R_updatable = false; // !! linpoint changed

						m_b_linearization_dirty = false;

						this->m_marginals.DisableUpdate(); // !!

						timer.Accum_DiffSample(m_f_vert_upd_time);
					}
					// update the system (in parallel)

					if(!b_utsolve_result)
						break;
					// in case cholesky failed, quit
				}
			} else {
				_TyTimeSampler timer(this->m_timer);

				if(n_iteration && m_b_system_dirty) {
					_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // want only lambda, leave R behind
					m_b_system_dirty = false;
					m_b_R_up_to_date = false; // lambda not dirty anymore, but R still is
					m_b_R_updatable = false; // this does not change, ir porbably wasnt updatable before (better be safe)

					timer.Accum_DiffSample(m_f_lambda_refresh_time);

#ifdef __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
					m_b_R_up_to_date = m_b_R_updatable = b_Refresh_R(0, 0, n_iteration > 1); // refresh R as well
					// suppress reordering in iterations 2 and above
					// (already reordered in iteration 1, won't get any better)
					// note that RHS vector is updated inside

					_TyTime f_dummy_sample = 0;
					timer.Accum_DiffSample(f_dummy_sample); // b_Refresh_R() contains timing inside

					if(m_b_R_up_to_date) {
						-- n_iteration;
						-- m_n_iteration_num;
						b_verbose = false; // suppress the banner
						continue;
					}
					// try again, this time with R
#endif // __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
				}
				// no need to rebuild lambda, just refresh the values that are being referenced

				++ m_n_lambda_optim_num;
				// we fall back to lambda

				_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_dx);
				// collects the right-hand side vector

				timer.Accum_DiffSample(m_f_rhs_time);

				{
					bool b_cholesky_result;
					{
						Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
						if((/*m_b_first_iteration_use_R &&*/ n_max_iteration_num > 2) ||
						   (!m_b_first_iteration_use_R && n_max_iteration_num > 1)) {
							do {
								if(n_iteration == ((m_b_first_iteration_use_R)? 1 : 0) &&
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

						if(this->m_b_verbose)
							printf("%s", (b_cholesky_result)? "Cholesky succeeded\n" : "Cholesky failed\n");
					}
					// lambda is good without permutation (there is one inside and we save copying eta around)
					// calculate cholesky, reuse block ordering if the linear solver supports it

					timer.Accum_DiffSample(m_f_chol_time);

#ifdef _DEBUG
					nonlinear_detail::CSolverOps_Base::b_DetectNaNs(m_v_dx, true, "fL2 dx");
#endif // _DEBUG

					double f_residual_norm = 0;
					if(b_cholesky_result) {
						f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
						if(this->m_b_verbose)
							printf("residual norm: %.4f\n", f_residual_norm);
					}
					// calculate residual norm

					timer.Accum_DiffSample(m_f_norm_time);
					// timing breakup

					if(f_residual_norm <= f_min_dx_norm) {
						m_b_linearization_dirty = true;
						break;
					}
					// in case the error is low enough, quit (saves us recalculating the hessians)

					if(b_cholesky_result) {
						/*printf("just optimized using lambda\n");*/

						nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, m_v_dx);

						timer.Accum_DiffSample(m_f_vert_upd_time);

						m_b_system_dirty = true;
						m_b_R_up_to_date = false;
						m_b_R_updatable = false; // linpoint just changed

						m_b_linearization_dirty = false;

						this->m_marginals.DisableUpdate(); // !!

						timer.Accum_DiffSample(m_f_vert_upd_time);
					}
					// update the system (in parallel)

					if(!b_cholesky_result)
						break;
					// in case cholesky failed, quit
				}
			}
		}

#ifdef __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
		_ASSERTE(m_b_system_dirty || m_b_R_up_to_date);
		// make sure that R is indeed kept up-to-date, unless the solver
		// was stopped by reaching the maximum number of iterations

		if(!m_b_R_up_to_date) {
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // want only lambda, leave R behind
			m_b_system_dirty = false;

			//b_Refresh_R(0, 0, b_did_reorder); // does this break something? // nope, all ok so far

			m_b_R_up_to_date = false; // lambda not dirty anymore, but R still is
			m_b_R_updatable = false; // this does not change. if it was updatable before, it still is (probably isnt)

			m_b_R_up_to_date = m_b_R_updatable = b_Refresh_R(0, 0, true); // refresh R as well, only reorder if there were no iterations taken

			m_b_R_updatable = false; // force update the next time around. this fixes the problem.
		}
		// workaround patch of the sphere / garage bug. not very sure how the error happens so this is not the final solution :(.
#endif // __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE

#ifdef _DEBUG
		if(!Check_Factor())
			fprintf(stderr, "error: R, d incorrect at the end of Optimize()\n");
#endif // _DEBUG

		TryMarginals(); // !!
	}

protected:
	/**
	 *	@brief incrementally updates the lambda matrix structure (can be empty)
	 *
	 *	@param[in] n_vertices_already_in_lambda is number of vertices before the first vertex that changes
	 *	@param[in] n_edges_already_in_lambda is number of edges before the first edge that changes
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Extend_LambdaR(size_t n_vertices_already_in_lambda, size_t n_edges_already_in_lambda) // throw(std::bad_alloc)
	{
		_TyLambdaOps::Extend_Lambda(this->m_r_system, m_reduction_plan, m_lambda,
			n_vertices_already_in_lambda, n_edges_already_in_lambda);
		// extend lambda using the lambda ops

		if(!n_vertices_already_in_lambda && !n_edges_already_in_lambda) {
			m_R.Clear();
			this->m_r_system.r_Vertex_Pool().For_Each(fL_util::CAlloc_RBlocks(m_R)); // can stay, there is no ordering to be applied

			//AddEntriesInSparseSystem(); // works for empty
		} else {
			_ASSERTE(m_lambda.n_Row_Num() > 0 && m_lambda.n_Column_Num() == m_lambda.n_Row_Num()); // make sure lambda is not empty
			this->m_r_system.r_Vertex_Pool().For_Each(n_vertices_already_in_lambda,
				this->m_r_system.r_Vertex_Pool().n_Size(), fL_util::CAlloc_RBlocks(m_R)); // will not work if ordering is applied (but it mostly isn't, the increments follow identity ordering)

			//UpdateSparseSystem(n_vertices_already_in_lambda, n_edges_already_in_lambda); // does not work for empty
		}
		// create block matrix lambda
		// todo - add this to L ops?
	}

#if 0
	/**
	 *	@brief checks if R == chol(lambda), prints the norm of the difference to stdout
	 */
	void Check_RLambdaTracking() const
	{
		CUberBlockMatrix RtR_upper;
		m_R.PreMultiplyWithSelfTransposeTo(RtR_upper, true);
		//cs *p_R = m_R.p_Convert_to_Sparse();
		cs *p_lam = m_lambda.p_Convert_to_Sparse();
		//cs *p_Rt = cs_transpose(p_R, 1);
		cs *p_RtR = RtR_upper.p_Convert_to_Sparse();//cs_multiply(p_Rt, p_R);
		cs *p_diff = cs_add(p_RtR, p_lam, 1, -1);
		double f_norm = cs_norm(p_diff);
		//cs_spfree(p_R);
		cs_spfree(p_lam);
		//cs_spfree(p_Rt);
		cs_spfree(p_RtR);
		cs_spfree(p_diff);
		// calculate norm (R*R' - lambda)

		printf("R - lambda tracking: %f\n", f_norm);
	}
#endif // 0

	/**
	 *	@brief calculates the new \f$R_{11}\f$ matrix
	 *
	 *	@param[in] n_order_min is the minimum vertex that changes in R (zero-based index in blocks)
	 *	@param[in] n_order_max is number of column blocks in R (in blocks)
	 *	@param[in] r_R11_new is matrix, containing the new \f$R_{11}\f$, before calculating Cholesky of it
	 *
	 *	@note This may modify / damage r_R11_new as it is no longer needed and it is *not* a reference
	 *		to a part of R, in case that enables speed optimizations.
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	void Refresh_R11(size_t n_order_min, size_t n_order_max, CUberBlockMatrix &r_R11_new) // throw(std::bad_alloc, std::runtime_error)
	{
#ifdef __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
		_ASSERTE(r_R11_new.n_Row_Num() == r_R11_new.n_Column_Num());
		if(r_R11_new.n_Column_Num() < 150) {
			if(!r_R11_new.Cholesky_Dense_FBS<_TyLambdaMatrixBlockSizes, 15>()) // 15, not 150 (would yield >1024 template depth)
				throw std::runtime_error("Cholesky_Dense_FBS() failed to increment R");
			// use statically sized matrices up to 30x30, then dynamically allocated matrices up to 150x150

			m_R.From_Matrix(n_order_min, n_order_min, r_R11_new);
			// put r_R11_new to m_R
		} else {
#else // __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
		{
#endif // __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
			if(!b_Have_NativeSolver) {
				CUberBlockMatrix R11_old;
				m_R.SliceTo(R11_old, n_order_min, n_order_max, n_order_min, n_order_max, true); // get R11 as well, need to clear the blocks first
				// todo - make a ClearBlocks() function to a) delete or b) memset(0) blocks in a rectangular area

				R11_old.Scale_FBS_Parallel<_TyLambdaMatrixBlockSizes>(0);
				// clears the data in the update area (it would be better to erase the blocks, but there is the fap) // todo - see indices of the blocks in R and see if these could be efficiently erased right now (while keeping the structure)
				// note that even if we thrash memory taken by (some) R11 blocks,
				// it will be recollected once a full update takes place
			}
			// only have to clear for the old solvers, the native solver does it automatically

			if(!m_linear_solver2.Factorize_PosDef_Blocky(m_R, r_R11_new,
			   m_R_row_lookup_table, n_order_min, n_order_min))
				throw std::runtime_error("Factorize_PosDef_Blocky() failed to increment R");
		}
	}

	/**
	 *	@brief calculates the new R matrix incrementally using lambda or omega
	 *
	 *	@param[in] n_refresh_from_edge is the first edge that changes
	 *	@param[in] n_order_min is the minimum vertex that changes in R (zero-based index in blocks)
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	inline void Refresh_R_IncR11(size_t n_refresh_from_edge, size_t n_order_min) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(m_b_R_up_to_date || m_b_R_updatable);
		// make sure R is up to date with lambda and we can actually increment

		_ASSERTE(n_refresh_from_edge > 0);
		// make sure that lambda didn't rebuild the ordering completely

		const size_t n_order_max = m_lambda.n_BlockColumn_Num();
		// makes sure that max is fixed at the end

		//size_t n_min_vertex = m_p_lambda_block_ordering[n_order_min]; // this is wrong, n_order_min is already ordered
		bool b_identity_perm = true;
		for(size_t i = n_order_min; i < n_order_max; ++ i) {
			if(m_p_lambda_block_ordering[i] != i) {
				b_identity_perm = false;
				break;
			}
		}
		if(!b_identity_perm) {
			//for(size_t i = n_order_min + 1; i < n_order_max; ++ i)
			//	n_min_vertex = std::min(n_min_vertex, m_p_lambda_block_ordering[i]);
		}
		// see if the ordering is identity ordering

#ifndef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		if(n_order_max - n_order_min >= 88) // disable this for timing bench
			b_identity_perm = false;
#endif // !__NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		// yet another threshold

		_TyTimeSampler timer(this->m_timer);

		bool b_omega_available = b_identity_perm;

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		b_identity_perm = true; // i want both branches to run
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

		CUberBlockMatrix R11TR11;
		if(b_identity_perm) {
			++ m_n_omega_update_num;
			// count them

			CUberBlockMatrix omega, R11;
			size_t n_elem_order_min = m_lambda_perm.n_BlockColumn_Base(n_order_min); // this should be m_p_lambda_block_ordering[n_order_min] but since it only runs if b_identity_perm is set, then its ok
			this->m_r_system.r_Edge_Pool().For_Each(n_refresh_from_edge, this->m_r_system.r_Edge_Pool().n_Size(),
				fL_util::CCalculateOmega(omega, n_elem_order_min));
			omega.CheckIntegrity();

			timer.Accum_DiffSample(m_f_r11_omega_calc_time);

			m_R.SliceTo(R11, n_order_min, n_order_max, n_order_min, n_order_max, true); // row(0 - min) x col(min - max)
			// calculate the omega matrix (ho, ho, ho) and slice R11

			timer.Accum_DiffSample(m_f_r11_omega_slice_time);

			if(n_order_max - n_order_min >= __NONLINEAR_SOLVER_FAST_L_PARALLEL_MATMULT_THRESH) // big one // t_odo this never runs, the limit for using R is also 100
				R11.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyLambdaMatrixBlockSizes>(R11TR11, true); // calculate R11^T * R11 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
			else
				R11.PreMultiplyWithSelfTransposeTo_FBS<_TyLambdaMatrixBlockSizes>(R11TR11, true); // calculate R11^T * R11 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
			// calculate R11TR11

			timer.Accum_DiffSample(m_f_r11_omega_ata_time);

			bool UNUSED(b_result) = omega.AddTo_FBS<_TyLambdaMatrixBlockSizes>(R11TR11); // todo - maybe also parallel
			_ASSERTE(b_result); // if the block order in omega was wrong, this would fail
			// calculate R11TR11_new = R11TR11 + omega
			// note this uses faster addition algorithm

			timer.Accum_DiffSample(m_f_r11_omega_add_time);
		}

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		b_identity_perm = false; // i want both branches to run
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

		CUberBlockMatrix R01TR01;
		if(!b_identity_perm) {
			CUberBlockMatrix lambda11, R01;
			++ m_n_lambda_update_num;
			// count them

			m_R.SliceTo(R01, 0, n_order_min, n_order_min, n_order_max, true); // row(0 - min) x col(min - max)
			m_lambda_perm.SliceTo(lambda11, n_order_min, n_order_max, n_order_min, n_order_max, true);

			timer.Accum_DiffSample(m_f_r11_lambda_slice_time);

			if(n_order_max - n_order_min >= __NONLINEAR_SOLVER_FAST_L_PARALLEL_MATMULT_THRESH) // big one // t_odo this never runs, the limit for using R is also 100
				R01.PreMultiplyWithSelfTransposeTo_FBS_Parallel<_TyLambdaMatrixBlockSizes>(R01TR01, true); // calculate R01^T * R01 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?
			else
				R01.PreMultiplyWithSelfTransposeTo_FBS<_TyLambdaMatrixBlockSizes>(R01TR01, true); // calculate R01^T * R01 // t_odo - use FBS and maybe also parallel // t_odo - only need lower diagonal, what to do?

			timer.Accum_DiffSample(m_f_r11_lambda_ata_time);

			lambda11.AddTo_FBS<_TyLambdaMatrixBlockSizes>(R01TR01, -1, 1); // t_odo - use FBS // todo - maybe also parallel
			// calculates R01TR01 = -R01TR01 + lambda11 (note the "-1, 1" is correct, the opposite way it crashes)
			// note that lambda11 is upper triangular, as well as R01TR01

			timer.Accum_DiffSample(m_f_r11_lambda_add_time);
		}

		Refresh_R11(n_order_min, n_order_max, (b_omega_available)? R11TR11 : R01TR01);

		timer.Accum_DiffSample(m_f_Rupdate_time);

		Refresh_d_IncR11(n_refresh_from_edge, n_order_min); // use the function, do not repeat code, it is ...
		// note that this contains its own timing inside
	}

	/**
	 *	@brief calculates the new right-hand-side vector, does it incrementally where possible
	 *
	 *	@param[in] n_refresh_from_edge is the first edge that changes
	 *	@param[in] n_order_min is the minimum vertex that changes in R (zero-based index in blocks)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Refresh_d_IncR11(size_t UNUSED(n_refresh_from_edge), size_t n_order_min) // throw(std::bad_alloc)
	{
		_ASSERTE(m_b_R_up_to_date || m_b_R_updatable);
		// make sure R is up to date with lambda and we can actually increment

		_ASSERTE(n_refresh_from_edge > 0);
		// make sure that lambda didn't rebuild the ordering completely

		_TyTimeSampler timer(this->m_timer);

		const size_t n_order_max = m_lambda.n_BlockColumn_Num();
		// makes sure that max is fixed at the end

		//size_t n_min_vertex = m_p_lambda_block_ordering[n_order_min]; // this is wrong, n_order_min is already ordered
		bool b_identity_perm = true;
		for(size_t i = n_order_min; i < n_order_max; ++ i) {
			if(m_p_lambda_block_ordering[i] != i) {
				b_identity_perm = false;
				break;
			}
		}
		/*if(!b_identity_perm) {
			for(size_t i = n_order_min + 1; i < n_order_max; ++ i)
				n_min_vertex = std::min(n_min_vertex, m_p_lambda_block_ordering[i]);
		}*/
		const bool b_is_identity_perm = b_identity_perm; // make a copy

		{
			if(b_is_identity_perm) {
				++ m_n_resumed_forwardsubst_num;

				_ASSERTE(m_v_d.rows() == m_lambda.n_Column_Num());
				_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_d,
					n_order_min, this->m_r_system.r_Vertex_Pool().n_Size());
				//this->m_r_system.r_Vertex_Pool().For_Each_Parallel(n_order_min,
				//	this->m_r_system.r_Vertex_Pool().n_Size(), CCollect_RightHandSide_Vector(m_v_d));
				// collect part of b to the lower part of d (this is inside of Collect_RightHandSide_Vector())

				timer.Accum_DiffSample(m_f_rhs_time);

				m_lambda_perm.InversePermute_LeftHandSide_Vector(&m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows(),
					m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
				m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(
					&m_v_perm_temp(0), m_v_perm_temp.rows(), n_order_min);
				m_lambda_perm.Permute_LeftHandSide_Vector(&m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows(),
					m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
				// "resumed forward substitution"
			} else {
				++ m_n_resumed_perm_forwardsubst_num; // a different category

				_ASSERTE(m_v_d.rows() == m_lambda.n_Column_Num());
				{
					if(!n_order_min/*n_min_vertex*/) {
						-- m_n_resumed_perm_forwardsubst_num;
						++ m_n_full_forwardsubst_num; // this is really a full one

						/*this->m_r_system.r_Vertex_Pool().For_Each_Parallel(n_min_vertex,
							this->m_r_system.r_Vertex_Pool().n_Size(), CCollect_RightHandSide_Vector(m_v_d));*/
						_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_d,
							n_order_min/*n_min_vertex*/, this->m_r_system.r_Vertex_Pool().n_Size());
					} else {
						const size_t *p_order_inv = m_lambda_ordering.p_Get_Ordering();
						//CCollect_RightHandSide_Vector collector(m_v_d);
#ifdef _OPENMP
						_ASSERTE(n_order_max <= INT_MAX);
						const int n = int(n_order_max);
						#pragma omp parallel for default(shared) if(n - int(n_order_min/*n_min_vertex*/) >= 50)
						for(int i = int(n_order_min/*n_min_vertex*/); i < n; ++ i)
#else // _OPENMP
						for(size_t i = n_order_min/*n_min_vertex*/; i < n_order_max; ++ i)
#endif // _OPENMP
							_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_d, p_order_inv[i]); // this requires ReduceSingle()
							//collector(this->m_r_system.r_Vertex_Pool()[p_order_inv[i]]);
						// can do this in parallel as well
					}
				}
				// do this instead

				timer.Accum_DiffSample(m_f_rhs_time);

				// so ... updating vertices n_order_min to n_order_max (a contiguous range?)
				// after permutation, they go to vector[m_p_lambda_block_ordering[n_order_min to n_order_max]]
				// so the whole part of the _permuted_ vector from m_p_lambda_block_ordering[n_order_min] to
				// the end must be updated, and *no more*

				m_lambda_perm.InversePermute_LeftHandSide_Vector(&m_v_perm_temp(0), &m_v_d(0), m_v_dx.rows(),
					m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num()); // dest[p ++] = *src ++
				m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(
					&m_v_perm_temp(0), m_v_perm_temp.rows(), n_order_min/*n_min_vertex*/);
				m_lambda_perm.Permute_LeftHandSide_Vector(&m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows(),
					m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num()); // *dest ++ = src[p ++]
				// "resumed forward substitution"
			}
			// convert eta to d (d = eta/R)
		}
		// update d incrementally as well

		timer.Accum_DiffSample(m_f_d_time);
	}

	/**
	 *	@brief calculates the new R matrix from scratch as Cholesky of lambda
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	inline void Refresh_R_FullR() // throw(std::bad_alloc, std::runtime_error)
	{
		_TyTimeSampler timer(this->m_timer);

		m_n_last_full_R_update_size = m_lambda.n_BlockColumn_Num();
		// R will have the same size once updated ...

		if(!b_Have_NativeSolver) {
			if(b_Is_PoseOnly_SLAM) { // this is known at compile-time, should optimize the unused branch away
				m_R.Clear();
				this->m_r_system.r_Vertex_Pool().For_Each(fL_util::CAlloc_RBlocks(m_R)); // won't work with VP problems, need to set correct ordering to vertices
			} else {
				//m_R.Clear(); // already inside PermuteTo()
				CUberBlockMatrix t_new_R;
				this->m_r_system.r_Vertex_Pool().For_Each(fL_util::CAlloc_RBlocks(t_new_R));
				t_new_R.PermuteTo(m_R, m_p_lambda_block_ordering, m_n_lambda_block_ordering_size);
			}
			// only need to do there things if not using the native solver
		}
		// do the right thing and thrash R

		if(!m_linear_solver2.Factorize_PosDef_Blocky(m_R, m_lambda_perm, m_R_row_lookup_table, 0, 0))
			throw std::runtime_error("Factorize_PosDef_Blocky() failed to calculate full R");
		// factorize (uses cached cholesky, saves some time on allocation of workspace memory)

		timer.Accum_DiffSample(m_f_fullR_cholesky);

		{
			_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_dx);
			// collects the right-hand side vector (eta)

			timer.Accum_DiffSample(m_f_rhs_time);

			//++ m_n_full_forwardsubst_num;
			// do not count it here, we know how many times we did full R, it is the same count

			_ASSERTE(m_p_lambda_block_ordering);
			m_lambda_perm.InversePermute_LeftHandSide_Vector(&m_v_perm_temp(0), &m_v_dx(0), m_v_dx.rows(),
				m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
			m_R.UpperTriangularTranspose_Solve_FBS<_TyLambdaMatrixBlockSizes>(&m_v_perm_temp(0), m_v_perm_temp.rows());
			m_lambda_perm.Permute_LeftHandSide_Vector(&m_v_d(0), &m_v_perm_temp(0), m_v_dx.rows(),
				m_p_lambda_block_ordering, m_lambda_perm.n_BlockRow_Num());
			// d = eta = eta/R
		}
		// convert eta to d

		timer.Accum_DiffSample(m_f_fullR_d);
	}

	/**
	 *	@brief refreshes the R matrix either from (pert of) lambda or from omega
	 *
	 *	@param[in] n_referesh_from_vertex is zero-based index of the first vertex that changes (unused)
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge that changes
	 *	@param[in] b_supress_reorder is ordering supression flag (if set, full reordering will not take place)
	 *
	 *	@return Returns true if R was refreshed, false if it decided to take lambda fallback instead.
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error (when not-pos-def).
	 */
	inline bool b_Refresh_R(size_t UNUSED(n_referesh_from_vertex) = 0,
		size_t n_refresh_from_edge = 0, bool b_supress_reorder = false) // throw(std::bad_alloc, std::runtime_error)
	{
		_TyTimeSampler timer(this->m_timer);

		// note that lambda is now up to date

		bool b_force_reorder = !n_refresh_from_edge && !b_supress_reorder; // if rebuilding whole lambda, it would be shame not to reorder
		// flag for forcing reorder

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		if(false) { // no optimizations for R up variants timing
#else // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		if(!b_supress_reorder) { // if allow optimizations ...
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
			if(!b_force_reorder && m_lambda.n_BlockColumn_Num() > m_n_last_full_R_update_size + 10) { // it's always dense at the beginning // hlamfb 10
				size_t n_nnz = m_R.n_Storage_Size();
				float f_area = float(m_R.n_Column_Num()) * m_R.n_Column_Num();
				float f_density = n_nnz / f_area;
				if(f_density > 0.02f) {
					b_force_reorder = true;
					//printf("R became too dense (%.2f %%), forcing reorder\n", f_density * 100); // verbose
				}
			}
			// permit 2% density in R, then rebuild
		}
		// these two should just set b_force_reorder

		if(b_force_reorder) {
			// only calculate a new ordering on full refresh or if forced

			//printf("build new ordering ...\n");

			m_lambda_ordering.p_BlockOrdering(m_lambda,
				m_lambda_constraint.p_Get(m_lambda.n_BlockColumn_Num()),
				m_lambda.n_BlockColumn_Num(), true); // constrained blocky, calculate inverse as well
			m_p_lambda_block_ordering = m_lambda_ordering.p_Get_InverseOrdering(); // todo - make sure that the last vertex is a pose (otherwise we need to modify the constraint to select the pose, not the landmark)
			// get blockwise and elementwise ordering ...

			if(!b_Have_NativeSolver)
				m_R_row_lookup_table.clear(); // unused in native solver
			// can't reuse lookup of R's rows since these change with ordering
		} else if(m_lambda.n_BlockColumn_Num() > m_lambda_perm.n_BlockColumn_Num()) {
			// simply appends ordering with a new value (identity ordering at the end)

			//printf("extend ordering by " PRIsize "\n", m_lambda.n_BlockColumn_Num() - m_lambda_perm.n_BlockColumn_Num());

			m_p_lambda_block_ordering = m_lambda_ordering.p_InvertOrdering(
				m_lambda_ordering.p_ExtendBlockOrdering_with_Identity(m_lambda.n_BlockColumn_Num()),
				m_lambda.n_BlockColumn_Num());
			// get blockwise and elementwise ordering ...
		}
		m_n_lambda_block_ordering_size = m_lambda.n_BlockColumn_Num();
		// refresh/update the ordering (update means append with identity)

		_ASSERTE(CMatrixOrdering::b_IsValidOrdering(m_p_lambda_block_ordering, m_lambda.n_BlockColumn_Num()));
		// make sure that the ordering is good

		m_lambda.Permute_UpperTriangular_To(m_lambda_perm, m_p_lambda_block_ordering,
			m_lambda.n_BlockColumn_Num(), true);
		// make a reordered version of lambda (*always* changes if lambda changes)

		_ASSERTE(m_n_lambda_block_ordering_size == this->m_r_system.r_Vertex_Pool().n_Size());
		size_t n_order_min;
		if(b_force_reorder)
			n_order_min = 0; // a new ordering? from the ground up then ...
		else if(b_supress_reorder)
			n_order_min = 0; // the whole system changed numerically? from the ground up then ...
		else {
			n_order_min = m_n_lambda_block_ordering_size - 1;//m_p_lambda_block_ordering[m_n_lambda_block_ordering_size - 1]; // bigtodo - is this that, or just m_n_lambda_block_ordering_size? should be better with m_n_lambda_block_ordering_size
			for(size_t i = n_refresh_from_edge, n = this->m_r_system.r_Edge_Pool().n_Size(); i < n; ++ i) {
				typename CSystem::_TyEdgeRef r_edge = this->m_r_system.r_Edge_Pool()[i];
				for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j) {
					size_t n_vertex = r_edge.n_Vertex_Id(j); // note that these are ids, but these equal order at the moment
					n_order_min = std::min(n_order_min, m_p_lambda_block_ordering[n_vertex]);
				}
				// hyperedge support
			}
			_ASSERTE(n_order_min < this->m_r_system.r_Vertex_Pool().n_Size()); // make sure this is of optimized vertex (not constant)
			//printf("loop size: " PRIsize "\n", m_n_lambda_block_ordering_size - 1 - n_order_min); // debug
		}
		// calculate min vertex order that needs to be updated (within the ordering!)

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
		m_n_loop_size_cumsum += m_n_lambda_block_ordering_size - 1 - n_order_min;
		// calculate how much loops did it process
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS

		timer.Accum_DiffSample(m_f_ordering_time);
		// stats

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		static bool b_first_time_dump = true;
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

		if(n_order_min > 0) {
			// if !n_order_min, L01 is empty and R merely equals chol(lambda)

			if(m_n_edges_in_lambda == this->m_r_system.r_Edge_Pool().n_Size()) {
				_ASSERTE(m_n_verts_in_lambda == m_lambda.n_BlockColumn_Num());
				_ASSERTE(m_n_verts_in_lambda == this->m_r_system.r_Vertex_Pool().n_Size());
				return true;
			}
			// this is final optimization, no need to refresh, there is no new edge / vertex

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
			double p_inc_upd_times_start[] = {
				m_f_Rupdate_time,
				m_f_d_time
			};
			double p_omega_upd_times_start[] = {
				m_f_r11_omega_calc_time,
				m_f_r11_omega_slice_time,
				m_f_r11_omega_ata_time,
				m_f_r11_omega_add_time
			};
			double p_lambda_upd_times_start[] = {
				m_f_r11_lambda_slice_time,
				m_f_r11_lambda_ata_time,
				m_f_r11_lambda_add_time
			};
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

			const size_t n_order_max = m_lambda.n_BlockColumn_Num();

			// now: a) extend the reorder area to contain all the blocks in frontline (so lambda10 is null)
			//      b) freezee the few blocks in the frontline using constrained ordering? will that help?
			//      c) ?

			bool b_limited_search_region = false;
			bool b_blocks_above = false;

			m_lambda_perm.Get_UpperTriangular_BlockFrontline(lambda_perm_frontline);
			// get the frontline

			_ASSERTE(lambda_perm_frontline.size() == n_order_max);
			for(size_t i = n_order_min + 1; i < n_order_max /*- 1*/; ++ i) { // not sure why dont i check the last one - todo
				if(lambda_perm_frontline[i] < n_order_min) {
					b_blocks_above = true;
					break;
				}
			}
			// see if there are blocks above (except in the first or last column which are fixed)

#ifdef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
			CUberBlockMatrix lambda11; // todo - make it a member?
#endif // __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
			if(!b_blocks_above) { // 4.2 seconds for full ordering always -> 2.3 seconds
				// this is the insufficient ordering only on lambda11
				// (causes a horrible fill-in if there are any blocks above)

#ifndef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
				CUberBlockMatrix lambda11; // local is enough
#endif // !__NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
				m_lambda_perm.SliceTo(lambda11, n_order_min,
					n_order_max, n_order_min, n_order_max, true);
				_ASSERTE(lambda11.b_Square() && lambda11.b_SymmetricLayout());

				m_p_lambda11_block_ordering = m_lambda11_ordering.p_BlockOrdering(lambda11,
					m_lambda11_constraint.p_Get(lambda11.n_BlockColumn_Num()), lambda11.n_BlockColumn_Num());
				m_n_lambda_block11_ordering_size = lambda11.n_BlockColumn_Num();
			} else {
				size_t n_frontline_min = lambda_perm_frontline[n_order_min];
				for(size_t i = n_order_min + 1; i < n_order_max; ++ i) {
					if(n_frontline_min > lambda_perm_frontline[i])
						n_frontline_min = lambda_perm_frontline[i];
				}
				// find a minimal frontline value over the updated submatrix

				size_t n_context_min = 0;
				if(n_frontline_min > 0) {
					for(size_t i = n_order_min; i > 0;) { // note that the loop terminates itself, the condition is not required (i think)
						-- i;
						if(n_frontline_min >= i) {
							n_context_min = i;
							// in case frontline is at the same level as a submatrix, we can order using a smaller matrix

							break;
						}
						//if(n_frontline_min > lambda_perm_frontline[i])
						//	n_frontline_min = lambda_perm_frontline[i]; // do not do this, will enlarge the search window too much, in order to include elements that can not be reordered anyway (seems like a good idea now)
					}
					// see if we can expand the ordering submatrix a little, to avoid using the whole matrix

#ifdef _DEBUG
					if(n_context_min > 0) {
						//for(size_t i = n_context_min + 1; i < n_order_max - 1; ++ i) // forbids first / last // do not do this
						for(size_t i = n_order_min + 1; i < n_order_max - 1; ++ i) // in case the expanding over previous area is ignored
							_ASSERTE(lambda_perm_frontline[i] >= n_context_min);
						/*printf("can order on " PRIsize " x " PRIsize " instead of " PRIsize " x " PRIsize
							" (update is " PRIsize " x " PRIsize ")\n", n_order_max - n_context_min,
							n_order_max - n_context_min, n_order_max, n_order_max, n_order_max - n_order_min,
							n_order_max - n_order_min);*/
					}
					// debug - see if we can use it and tell us
#endif // _DEBUG
				}
				_ASSERTE(n_context_min == n_frontline_min);
				// this is a no-op now, should remove it

				m_n_lambda_block11_ordering_size = n_order_max - n_order_min;
				_ASSERTE(m_lambda_perm.n_BlockColumn_Num() == m_lambda.n_BlockColumn_Num());

				if(n_context_min > n_order_max / 8 /*std::min(size_t(100), n_order_max / 16)*/) { // t_odo - need to dump n_context_min, probably citytrees have one at 0 or something, that can not be ordered away (make it go away by forcing it somewhere else?l)
					// this is prefix-constrained ordering on part of lambda perm (not full, not update)
					// this works rather well

					b_limited_search_region = true;
					// say we did it

					_ASSERTE(n_order_min > n_context_min);
					size_t n_skip_blocks = n_context_min; // how much to slice
					size_t n_skip_order = n_order_min - n_context_min; // how much to replace by diagonal
					size_t n_order_size = n_order_max - n_context_min; // size of the lambda11 part
					_ASSERTE(n_order_size > n_skip_order);

					const size_t *p_order = m_lambda_alt_ordering.p_BlockOrdering_MiniSkirt(m_lambda_perm,
						n_skip_blocks, n_order_min, m_lambda_alt_constraint.p_Get(n_order_size,
						n_skip_order), n_order_size);
					// get ordering on the submatrix of permuted lambda, without calculating the submatrix,
					// also with less items on the

#ifdef _DEBUG
					for(size_t i = 0; i < n_skip_order; ++ i)
						_ASSERTE(p_order[i] == i);
					// the prefix should be identity

					std::vector<bool> coverage;
					coverage.resize(m_n_lambda_block11_ordering_size, false);
					// make sure we produce a valid ordering
#endif // _DEBUG

					size_t *p_order11 = (size_t*)p_order + n_skip_order;
					for(size_t i = 0; i < m_n_lambda_block11_ordering_size; ++ i) {
						_ASSERTE(p_order11[i] >= n_skip_order);
						p_order11[i] -= n_skip_order;
						_ASSERTE(p_order11[i] < m_n_lambda_block11_ordering_size);

#ifdef _DEBUG
						_ASSERTE(!coverage[p_order11[i]]); // no repeated ones
						coverage[p_order11[i]] = true;
						// make sure we produce a valid ordering
#endif // _DEBUG
					}
					// just subtract to get ordering on lambda11

#ifdef _DEBUG
					_ASSERTE(std::find(coverage.begin(), coverage.end(), false) == coverage.end());
					// make sure all elements of lambda11 are covered
#endif // _DEBUG

					m_p_lambda11_block_ordering = p_order11;
				} else {
					// this is prefix-constrained ordering on the full lambda (perm)
					// this works rather well

					const size_t *p_order = m_lambda_alt_ordering.p_BlockOrdering_MiniSkirt(m_lambda_perm,
						0, n_order_min, m_lambda_alt_constraint.p_Get(m_lambda.n_BlockColumn_Num(),
						n_order_min), m_lambda.n_BlockColumn_Num());
					// just give diagonal matrix all the way from 0 to n_order_min, then the actual
					// sparsity pattern till n_order_max
					// get ordering on the permuted lambda
					// gets the full ordering on lambda, making sure that the prefix is the same and only the lambda11 suffix changes

#ifdef _DEBUG
					for(size_t i = 0; i < n_order_min; ++ i)
						_ASSERTE(p_order[i] == i);
					// the prefix should be identity

					std::vector<bool> coverage;
					coverage.resize(m_n_lambda_block11_ordering_size, false);
					// make sure we produce a valid ordering
#endif // _DEBUG

					size_t *p_order11 = (size_t*)p_order + n_order_min;
					for(size_t i = 0; i < m_n_lambda_block11_ordering_size; ++ i) {
						_ASSERTE(p_order11[i] >= n_order_min);
						p_order11[i] -= n_order_min;
						_ASSERTE(p_order11[i] < m_n_lambda_block11_ordering_size);

#ifdef _DEBUG
						_ASSERTE(!coverage[p_order11[i]]); // no repeated ones
						coverage[p_order11[i]] = true;
						// make sure we produce a valid ordering
#endif // _DEBUG
					}
					// just subtract to get ordering on lambda11

#ifdef _DEBUG
					_ASSERTE(std::find(coverage.begin(), coverage.end(), false) == coverage.end());
					// make sure all elements of lambda11 are covered
#endif // _DEBUG

					m_p_lambda11_block_ordering = p_order11;
					//m_p_lambda11_block_ordering = m_lambda11_ordering.p_InvertOrdering(p_order11,
					//	m_n_lambda_block11_ordering_size); // inverse
					// invert it for merging with the original ordering
				}

#ifdef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
				//CUberBlockMatrix lambda11; // above; scope / define hell
				m_lambda_perm.SliceTo(lambda11, n_order_min,
					n_order_max, n_order_min, n_order_max, true);
				_ASSERTE(lambda11.b_Square() && lambda11.b_SymmetricLayout());
				// will need this for verification purposes, otherwise not required by this method
#endif // __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
			}

			bool b_identity_ordering = true;
			for(size_t i = 0; i < m_n_lambda_block11_ordering_size; ++ i) {
				if(m_p_lambda11_block_ordering[i] != i) {
					b_identity_ordering = false;
					break;
				}
			}
			// calculates new block ordering for lambda11 (the updated area of lambda)

#ifdef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
			CUberBlockMatrix lambda00_p, lambda11_p;
			m_lambda_perm.SliceTo(lambda00_p, 0, n_order_min, 0, n_order_min, false); // make a deep copy
			lambda11.Permute_UpperTriangular_To(lambda11_p, m_p_lambda11_block_ordering,
				m_n_lambda_block11_ordering_size, false); // make a deep copy
			// copy lambda 00 and lambda 11 (don't care about lambda 01, it is hard to permute correctly at this point)
#endif // __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING

			_TyTime f_ordering11_time = 0;
			timer.Accum_DiffSample(f_ordering11_time);

			if(!b_identity_ordering) {
				if(!b_Have_NativeSolver)
					m_R_row_lookup_table.clear(); // unused in native solver
				// !! we defined a new ordering

				const size_t *p_order;
				m_p_lambda_block_ordering = m_lambda_ordering.p_InvertOrdering(p_order =
					m_lambda_ordering.p_ExtendBlockOrdering_with_SubOrdering(n_order_min,
					m_p_lambda11_block_ordering, m_n_lambda_block11_ordering_size), m_lambda.n_BlockColumn_Num());
				_ASSERTE(m_n_lambda_block_ordering_size == n_order_min + m_n_lambda_block11_ordering_size);
				// update the ordering (update means append with lambda11 sub-block ordering)
				// this is quick, no bottleneck in here

				_ASSERTE(CMatrixOrdering::b_IsValidOrdering(m_p_lambda_block_ordering,
					m_lambda.n_BlockColumn_Num()));
				// make sure that the ordering is good

				timer.Accum_DiffSample(m_f_ordering_fold_time);

				m_lambda.Permute_UpperTriangular_To(m_lambda_perm, m_p_lambda_block_ordering,
					m_lambda.n_BlockColumn_Num(), true, n_order_min, true); // last false is a strong approximate
				// note that this does leave allocated blocks, but in the next round, lambda will reperm and free those

#ifdef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
				CUberBlockMatrix lambda00_r, lambda11_r;
				m_lambda_perm.SliceTo(lambda00_r, 0, n_order_min, 0, n_order_min, false);
				m_lambda_perm.SliceTo(lambda11_r, n_order_min, n_order_max, n_order_min, n_order_max, false);
				// make copies of the new permutated lambda; should be identical to what was intended

				lambda00_p.AddTo(lambda00_r, -1);
				lambda11_p.AddTo(lambda11_r, -1);

				double f_diff0 = lambda00_r.f_Norm();
				double f_diff1 = lambda11_r.f_Norm();
				printf(" %g/%g", f_diff0, f_diff1);
#endif // __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING

				timer.Accum_DiffSample(m_f_repermute_time);

				m_R.SliceTo(m_R, n_order_min, n_order_min, true); // 3 seconds -> 0.3 seconds

				timer.Accum_DiffSample(m_f_Rslice_time);

				if(m_chol_bitfield.capacity() < n_order_max) {
					m_chol_bitfield.clear();
					m_chol_bitfield.reserve(std::max(n_order_max, 2 * m_chol_bitfield.capacity()));
				}
				m_chol_bitfield.resize(n_order_max, 0);

				m_lambda_perm.Build_EliminationTree(m_chol_etree, m_chol_ereach_stack); // use ereach stack as workspace
				_ASSERTE(m_chol_ereach_stack.size() == n_order_max);
				// build an elimination tree

				timer.Accum_DiffSample(m_f_etree_time);

				if(!m_R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(m_lambda_perm, m_chol_etree,
				   m_chol_ereach_stack, m_chol_bitfield, n_order_min)) { // todo - do incremental etree as well, might save considerable time
					//throw std::runtime_error("error: got not pos def in incR section anyways"); // does not really happen // it does, apparently, on 10k without fast math

					timer.Accum_DiffSample(m_f_resumed_chol_time);

					fprintf(stderr, "warning: got not pos def in permuted incR section ("
						PRIsize ", " PRIsize ")\n", m_R.n_BlockColumn_Num(), n_order_min);
					// we can still try to save the situation by redoing full R which may or may not help

					++ m_n_full_R_num;
					// R is not up to date, need to rebuild from scratch

					Refresh_R_FullR();
					// do the "full" R = chol(lambda)
				}
				// calculate updated R11 and R10 using resumed Cholesky

				++ m_n_resumed_chol_num;
				timer.Accum_DiffSample(m_f_resumed_chol_time);

				Refresh_d_IncR11(n_refresh_from_edge, n_order_min); // timing inside
				// all that is left to do is to refresh d
			} else {
				Refresh_R_IncR11(n_refresh_from_edge, n_order_min); // timing inside
				// run the "fast" refresh of R
			}
			// choose between progressive reordering and "fast" update to R

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
			char p_s_filename[256];

			CUberBlockMatrix lambda_perm;
			m_lambda.Permute_UpperTriangular_To(lambda_perm, m_p_lambda_block_ordering,
				m_lambda.n_BlockColumn_Num(), true);
			// need to reperm, may only have a part of lambda_perm, effectively selecting everything above n_order_min as a new nnz

			size_t n_verts_in_lambda = m_lambda.n_BlockColumn_Num();
			sprintf(p_s_filename, "rss2013/%05d_6_lambda-perm.tga", n_verts_in_lambda);
			//if(n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			//	lambda_perm.Rasterize_Symmetric(p_s_filename, (n_verts_in_lambda < 750 * 6 / _TyLambdaMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 5 : 3); // do not really need lambdas right now

			sprintf(p_s_filename, "rss2013/%05d_7_R.tga", n_verts_in_lambda);
			//if(n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			{
				int n_ss; // scalar size
				TBmp *p_img = m_R.p_Rasterize(lambda_perm, false, 0, n_ss = ((n_verts_in_lambda < 750 * 6 / _TyLambdaMatrixBlockSizes::_TyHead::n_column_num/*ColsAtCompileTime*/)? 3 : 2)); // highlight fill-in
				if(p_img) {
					CTgaCodec::Save_TGA(p_s_filename, *p_img, false, true);

					for(int y = 0, h = p_img->n_height; y < h; ++ y) {
						for(int x = 0, w = p_img->n_width; x < w; ++ x) {
							if(p_img->p_buffer[x + w * y] == 0xff808080U) // border (dots)?
								p_img->p_buffer[x + w * y] = 0xffffffffU; // white
						}
					}
					// remove the dotting from the image

					sprintf(p_s_filename, "rss2013/%05d_8_R_marked.tga", n_verts_in_lambda);
					//printf("drawing\n");
					int n_line0 = (n_ss - 1) * int(m_R.n_BlockColumn_Base(n_order_min));
					//int n_line1 = (n_ss - 1) * int(m_R.n_BlockColumn_Base(n_context_min));
					p_img->DrawLine(n_line0, n_line0, n_line0, p_img->n_height, 0xfff38630U, 8);
					p_img->DrawLine(n_line0, n_line0, p_img->n_width, n_line0, 0xfff38630U, 8); // draw line that separates order min
					for(int y = n_line0, h = p_img->n_height; y < h; ++ y) {
						for(int x = n_line0, w = p_img->n_width; x < w; ++ x) {
							if(p_img->p_buffer[x + w * y] == 0xffffffffU) // white?
								p_img->p_buffer[x + w * y] = 0xffffffbbU; // yellowish
						}
					}
					//p_img->DrawLine(n_line1, n_line1, n_line1, p_img->n_height, 0xff00ff00U, 3);
					//p_img->DrawLine(n_line1, n_line1, p_img->n_width, n_line1, 0xff00ff00U, 3); // draw line that separates context min
					//printf("saving\n");
					CTgaCodec::Save_TGA(p_s_filename, *p_img, false, true);
					//printf("deleting\n");
					p_img->Delete();
				} else
					fprintf(stderr, "error: not enough memory to rasterize the matrix\n");
			}

			sprintf(p_s_filename, "rss2013/%05d_9_stats.txt", n_verts_in_lambda);
			FILE *p_fw;
			if((p_fw = fopen(p_s_filename, "w"))) {
				fprintf(p_fw, PRIsize "\n", m_lambda_perm.n_Block_Num()); // save density of lambda
				fprintf(p_fw, PRIsize "\n", m_R.n_Block_Num()); // save density of R
				fprintf(p_fw, PRIsize "\n", n_order_min); // save density of R
				fclose(p_fw);
			}
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

			if(b_blocks_above) {
				if(b_limited_search_region) {
					++ m_n_limited_search_num;
					m_f_ordering11_part_time += f_ordering11_time;
				} else {
					++ m_n_blocks_above_num;
					m_f_ordering11_full_time += f_ordering11_time;
				}
			} else
				m_f_ordering11_time += f_ordering11_time;

			++ m_n_Rup_num;
			// count incremental R updates

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
			double p_inc_upd_times[] = {
				m_f_Rupdate_time,
				m_f_d_time
			};
			double f_inc_upd_sum = 0;
			for(int i = 0; i < 2; ++ i) {
				p_inc_upd_times[i] -= p_inc_upd_times_start[i];
				f_inc_upd_sum += p_inc_upd_times[i];
			}
			double p_omega_upd_times[] = {
				m_f_r11_omega_calc_time,
				m_f_r11_omega_slice_time,
				m_f_r11_omega_ata_time,
				m_f_r11_omega_add_time
			};
			double f_omega_upd_sum = f_inc_upd_sum;
			for(int i = 0; i < 4; ++ i) {
				p_omega_upd_times[i] -= p_omega_upd_times_start[i];
				f_omega_upd_sum += p_omega_upd_times[i];
			}
			double p_lambda_upd_times[] = {
				m_f_r11_lambda_slice_time,
				m_f_r11_lambda_ata_time,
				m_f_r11_lambda_add_time
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

			double f_full_R_start = this->m_timer.f_Time();
			Refresh_R_FullR();
			double f_full_R_time = this->m_timer.f_Time() - f_full_R_start;
			// measure full R as well

			FILE *p_fw = fopen("Rup_variants_time.txt", (b_first_time_dump)? "w" : "a");
			if(b_first_time_dump) {
				fprintf(p_fw, "verts-in-R;loop-size;full-R-time;lambda-up-time;lambda-slice-time;"
					"lambda-ata-time;lambda-add-time;omega-up-time;omega-calc-time;"
					"omega-slice-time;omega-ata-time;omega-add-time\n");
			}
			fprintf(p_fw, "" PRIsize ";" PRIsize ";%f;%f;%f;%f;%f", m_R.n_BlockColumn_Num(), n_loop_size, f_full_R_time,
				f_lambda_upd_sum, p_lambda_upd_times[0], p_lambda_upd_times[1], p_lambda_upd_times[2]);
			if(b_had_omega_upd) {
				fprintf(p_fw, ";%f;%f;%f;%f;%f\n", f_omega_upd_sum, p_omega_upd_times[0],
					p_omega_upd_times[1], p_omega_upd_times[2], p_omega_upd_times[3]);
			} else {
				fprintf(p_fw, ";;;;;\n");
			}
			fclose(p_fw);
			// print timing to a file
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		} else {
			//printf("doing full R\n"); // debug

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
			double f_full_R_start = this->m_timer.f_Time();
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

			++ m_n_full_R_num;
			// R is not up to date, need to rebuild from scratch

			Refresh_R_FullR();
			// do the "full" R = chol(lambda)

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
			double f_full_R_time = this->m_timer.f_Time() - f_full_R_start;
			// measure full R

			size_t n_loop_size = m_n_lambda_block_ordering_size - 1 - n_order_min;

			FILE *p_fw = fopen("Rup_variants_time.txt", (b_first_time_dump)? "w" : "a");
			if(b_first_time_dump) {
				fprintf(p_fw, "verts-in-R;loop-size;full-R-time;lambda-up-time;lambda-slice-time;"
					"lambda-ata-time;lambda-add-time;omega-up-time;omega-calc-time;"
					"omega-slice-time;omega-ata-time;omega-add-time\n");
			}
			fprintf(p_fw, "" PRIsize ";" PRIsize ";%f;;;;", m_R.n_BlockColumn_Num(), n_loop_size, f_full_R_time); // no lambda upd
			fprintf(p_fw, ";;;;;\n"); // no omega upd
			fclose(p_fw);
			// print timing to a file
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
			char p_s_filename[256];

			size_t n_verts_in_lambda = m_lambda.n_BlockColumn_Num();
			sprintf(p_s_filename, "rss2013/%05d_6_lambda-perm.tga", n_verts_in_lambda);
			//if(n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			//	lambda_perm.Rasterize_Symmetric(p_s_filename, (n_verts_in_lambda < 750 * 6 / _TyLambdaMatrixBlockSizes::_TyHead::ColsAtCompileTime)? 5 : 3); // do not really need lambdas right now

			sprintf(p_s_filename, "rss2013/%05d_7_R.tga", n_verts_in_lambda);
			//if(n_verts_in_lambda > size_t(n_dummy_param)) // continue from before
			{
				int n_ss; // scalar size
				TBmp *p_img = m_R.p_Rasterize(m_lambda_perm, false, 0, n_ss = ((n_verts_in_lambda < 750 * 6 / _TyLambdaMatrixBlockSizes::_TyHead::n_column_num/*ColsAtCompileTime*/)? 3 : 2)); // highlight fill-in
				if(p_img) {
					CTgaCodec::Save_TGA(p_s_filename, *p_img, false, true);

					for(int y = 0, h = p_img->n_height; y < h; ++ y) {
						for(int x = 0, w = p_img->n_width; x < w; ++ x) {
							if(p_img->p_buffer[x + w * y] == 0xff808080U) // border (dots)?
								p_img->p_buffer[x + w * y] = 0xffffffffU; // white
						}
					}
					// remove the dotting from the image

					sprintf(p_s_filename, "rss2013/%05d_8_R_marked.tga", n_verts_in_lambda);
					//printf("drawing\n");
					int n_line0 = 0;//(n_ss - 1) * m_R.n_BlockColumn_Base(n_order_min);
					//int n_line1 = (n_ss - 1) * m_R.n_BlockColumn_Base(n_context_min);
					p_img->DrawLine(n_line0, n_line0, n_line0, p_img->n_height, 0xfff38630U, 8);
					p_img->DrawLine(n_line0, n_line0, p_img->n_width, n_line0, 0xfff38630U, 8); // draw line that separates order min
					for(int y = n_line0, h = p_img->n_height; y < h; ++ y) {
						for(int x = n_line0, w = p_img->n_width; x < w; ++ x) {
							if(p_img->p_buffer[x + w * y] == 0xffffffffU) // white?
								p_img->p_buffer[x + w * y] = 0xffffffbbU;//0xffddddddU; // grayish // yellowish
						}
					}
					//p_img->DrawLine(n_line1, n_line1, n_line1, p_img->n_height, 0xff00ff00U, 3);
					//p_img->DrawLine(n_line1, n_line1, p_img->n_width, n_line1, 0xff00ff00U, 3); // draw line that separates context min
					//printf("saving\n");
					CTgaCodec::Save_TGA(p_s_filename, *p_img, false, true);
					//printf("deleting\n");
					p_img->Delete();
				} else
					fprintf(stderr, "error: not enough memory to rasterize the matrix\n");
			}

			sprintf(p_s_filename, "rss2013/%05d_9_stats.txt", n_verts_in_lambda);
			FILE *p_fw;
			if((p_fw = fopen(p_s_filename, "w"))) {
				fprintf(p_fw, PRIsize "\n", m_lambda_perm.n_Block_Num()); // save density of lambda
				fprintf(p_fw, PRIsize "\n", m_R.n_Block_Num()); // save density of R
				fclose(p_fw);
			}
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
		}

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
		b_first_time_dump = false;
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

		return true;
	}

#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY

	/**
	 *	@brief dumps density of R factor, given different ordering strategies
	 *	@note This is only available if __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY is defined.
	 */
	void Dump_RDensity()
	{
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
			mord.p_BlockOrdering(m_lambda, true); // unconstrained, calculate inverse right away
			const size_t *p_order = mord.p_Get_InverseOrdering();
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
			mord.p_BlockOrdering(m_lambda, p_constraint, m_lambda.n_BlockColumn_Num(), true);
			const size_t *p_order = mord.p_GetInverseOrdering();
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

		FILE *p_fw = fopen("RDensityByOrdering.txt", (this->m_n_real_step > 0)? "a" : "w");
		if(!this->m_n_real_step) {
			fprintf(p_fw, "block-cols;amd;blocky-amd;blocky-constrained-amd;"
				"amd-blocks;blocky-amd-blocks;blocky-constrained-amd-blocks;actual-R-nnz-blocks\n");
		}
		fprintf(p_fw, "" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize ";" PRIsize "\n", m_lambda.n_BlockColumn_Num(), n_nnz_ideal_elem,
			n_nnz_blocky_elem, n_nnz_blocky_constr_elem, n_nnz_ideal, n_nnz_blocky,
			n_nnz_blocky_constr, n_nnz_actual);
		fclose(p_fw);
	}

#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY

	CNonlinearSolver_FastL(const CNonlinearSolver_FastL &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_FastL &operator =(const CNonlinearSolver_FastL &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_FAST_L_INCLUDED
