/*
								+-----------------------------------+
								|                                   |
								| ***  Lambda nonlinear solver  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2016  |
								|                                   |
								|    NonlinearSolver_Lambda_DL.h    |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_Lambda_DL.h
 *	@brief nonlinear blocky solver working above the lambda matrix with fluid relinearization and Dogleg support
 *	@author -tHE SWINe-
 *	@date 2016-01-26
 */

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DL_INC_LINSOLVE_DEBUGGING
 *	@brief if defined, the linear solver precision is verified by multiplying back, also in release
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DL_INC_LINSOLVE_DEBUGGING

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DL_COLLECT_DX_STATS
 *	@brief if defined, the statistics about the norms of dx for the individual vertices are printed, also in release
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DL_COLLECT_DX_STATS

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_DEBUGGING
 *	@brief if defined, the deltas from the batch solution are printed even in release
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_DEBUGGING

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
 *	@brief if defined, numbers of FLOPs in the incremental Schur are recorded (slows down a lot)
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING

#ifdef __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
inline void DL_SC_Profile(size_t x) {} // force x to evaluate
#else
#define DL_SC_Profile(x) do {} while(0) // throw x away
#endif

#include "slam/LinearSolver_Schur.h"
#include "slam/OrderingMagic.h"
#include "slam/Marginals.h"
#include "slam/NonlinearSolver_Base.h"
#include "slam/NonlinearSolver_Lambda_Base.h"
#include "slam/NonlinearSolver_FastL_Base.h" // fL_util::CCalculateOmega

#include "sparse_flops/cts.hpp"
#include "sparse_flops/Instrument.h" // for evaluations

//#include "slam/MemUsage.h" // for memory evaluations

#include "slam/BAMarginals.h" // CSchurComplement_Marginals

// inject the 5D and 1D vector types into Eigen namespace
namespace Eigen {

#ifndef HAVE_EIGEN_5D_DOUBLE_VECTOR
#define HAVE_EIGEN_5D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 5, 1> Vector5d; /**< @brief 5D vector type */
#endif // !HAVE_EIGEN_5D_DOUBLE_VECTOR
#ifndef HAVE_EIGEN_1D_DOUBLE_VECTOR
#define HAVE_EIGEN_1D_DOUBLE_VECTOR
typedef Eigen::Matrix<double, 1, 1> Vector1d; /**< @brief 1D vector type (required for inverse distance vertices) */
#endif // !HAVE_EIGEN_1D_DOUBLE_VECTOR

} // ~Eigen

extern int n_dummy_param;
// used as damping

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@brief utilities for the \ref CNonlinearSolver_Lambda_DL solver
 */
namespace lambdaDL_util {

class CConcurrentBitArray {
protected:
	class CIsNonzero {
	public:
		bool operator ()(uint32_t n) const
		{
			return n != 0;
		}
	};

	class CConstConcurrentBitReference {
	protected:
		const uint32_t *m_p_data;
		const uint32_t m_n_mask;

	public:
		CConstConcurrentBitReference(const uint32_t *p_data, uint32_t n_mask)
			:m_p_data(p_data), m_n_mask(n_mask)
		{}

		CConstConcurrentBitReference(const CConstConcurrentBitReference &r_other)
			:m_p_data(r_other.m_p_data), m_n_mask(r_other.m_n_mask)
		{}

		operator bool() const
		{
			return (*m_p_data & m_n_mask) != 0;
		}

		bool operator !() const
		{
			return !(*m_p_data & m_n_mask);
		}
	};

	class CConcurrentBitReference : public CConstConcurrentBitReference {
	public:
		CConcurrentBitReference(uint32_t *p_data, uint32_t n_mask)
			:CConstConcurrentBitReference(p_data, n_mask)
		{}

		CConcurrentBitReference(const CConstConcurrentBitReference &r_other)
			:CConstConcurrentBitReference(r_other)
		{}

		CConcurrentBitReference &operator =(const CConstConcurrentBitReference &r_other)
		{
			return *this = bool(r_other); // rather than copying the reference, copy the value
		}

		CConcurrentBitReference &operator =(bool b_value)
		{
			if(b_value)
				*const_cast<uint32_t*>(m_p_data) |= m_n_mask;
			else
				*const_cast<uint32_t*>(m_p_data) &= ~m_n_mask;
			return *this;
		}

		void Flip()
		{
			*const_cast<uint32_t*>(m_p_data) ^= m_n_mask;
		}

		void Set()
		{
			*const_cast<uint32_t*>(m_p_data) |= m_n_mask;
		}

		void Clear()
		{
			*const_cast<uint32_t*>(m_p_data) &= ~m_n_mask;
		}

		void Atomic_Flip()
		{
			uint32_t *p_data = const_cast<uint32_t*>(m_p_data);
			#pragma omp atomic
			*p_data ^= m_n_mask;
		}

		void Atomic_Set()
		{
			uint32_t *p_data = const_cast<uint32_t*>(m_p_data);
			#pragma omp atomic
			*p_data |= m_n_mask;
		}

		void Atomic_Clear()
		{
			uint32_t *p_data = const_cast<uint32_t*>(m_p_data);
			#pragma omp atomic
			*p_data &= ~m_n_mask;
		}
	};

protected:
	std::vector<uint32_t> m_array;
	const size_t m_n_size;

public:
	CConcurrentBitArray(size_t n_size = 0, bool b_initializer = false)
		:m_array((n_size + 31) / 32, (b_initializer)? uint32_t(-1) : 0U), m_n_size(n_size)
	{}

	size_t n_Size() const
	{
		return m_n_size;
	}

	bool operator !() const // could use safe bool
	{
		return std::find_if(m_array.begin(), m_array.end(), CIsNonzero()) == m_array.end(); // handles empty ;)
	}

	bool operator =(bool b_value) // clear or raise the whole array
	{
		std::fill(m_array.begin(), m_array.end(), (b_value)? uint32_t(-1) : 0U);
		return b_value;
	}

	CConstConcurrentBitReference operator [](size_t n_index) const
	{
		_ASSERTE(n_index < m_n_size); // the vector will not catch out of bounds access till modulo 32
		return CConstConcurrentBitReference(&m_array[n_index / 32], uint32_t(1) << (n_index & 31));
	}

	CConcurrentBitReference operator [](size_t n_index)
	{
		_ASSERTE(n_index < m_n_size); // the vector will not catch out of bounds access till modulo 32
		return CConstConcurrentBitReference(&m_array[n_index / 32], uint32_t(1) << (n_index & 31));
	}

	// note that this is not resize-able to avoid bugs; dont need that functionality below anyways
};

} // ~lambdaDL_util

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
class CNonlinearSolver_Lambda_DL : public nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, true> {
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

	typedef typename CLinearSolver_Schur<CLinearSolver, CAMatrixBlockSizes, CSystem>::_TyGOH _TyGOH; /**< @brief guided Schur ordering helper */

	typedef typename _TyGOH::template CBlockSizes<_TyLambdaMatrixBlockSizes> _TySchurBlockSize_Helper; /**< @brief Schut complement block size and guided ordering calculation helper */
	typedef typename _TySchurBlockSize_Helper::_TyDBlockSizes _TyDBlockSizes; /**< @brief diagonal (landmark) matrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TySchurBlockSizes _TySchurBlockSizes; /**< @brief Schur complement (RCS) matrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TyUBlockSizes _TyUBlockSizes; /**< @brief upper off-diagonal submatrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TyVBlockSizes _TyVBlockSizes; /**< @brief lower off-diagonal submatrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TyOffDiagBlockSizes _TyOffDiagBlockSizes; /**< @brief off-diagonal submatrix block sizes */

	typedef CSchurComplement_Marginals<_TySchurBlockSizes, _TyUBlockSizes, _TyVBlockSizes, _TyDBlockSizes> CMargsHelper; /**< @brief specialization of marginal covariances helper for the given block sizes */

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
		solver_IsPreferredIncremental = true, /**< @brief preferred incremental solver flag */
		solver_ExportsJacobian = false, /**< @brief interface for exporting jacobian system matrix flag */
		solver_ExportsHessian = true, /**< @brief interface for exporting hessian system matrix flag */
		solver_ExportsFactor = false /**< @brief interface for exporting factorized system matrix flag */
	};

protected:
	typedef nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, true> _TyBase; /**< @brief base solver utils type */

	CUberBlockMatrix m_lambda; /**< @brief the lambda matrix (built / updated incrementally) */
	_TyReductionPlan m_reduction_plan; /**< @brief lambda incremental reduction plan */
	Eigen::VectorXd m_v_eta; /**< @brief eta vector (the steepest descent vector) */
	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	size_t m_n_verts_in_lambda; /**< @brief number of vertices already in lambda */
	size_t m_n_edges_in_lambda; /**< @brief number of edges already in lambda */
	//bool m_b_system_dirty; // not here; m_relin_vertex_list takes its role
	std::vector<size_t> m_relin_vertex_list; /**< @brief list of vertices to relinearize; if empty, then the linearization point is up to date */
	std::vector<size_t> m_last_relin_vertex_list; /**< @brief back-up of \ref m_relin_vertex_list to restore it after a bad step */
	std::vector<size_t> m_schur_ordering; /**< @brief Schur complement variable ordering (maintained incrementally) */
	size_t m_n_matrix_cut; /**< @brief rank of the reduced camera system (maintained incrementally) */
	size_t m_n_landmark_group_id; /**< @brief variable group used for landmarks (the eliminated variables in SC) */
	CUberBlockMatrix m_SchurCompl; /**< @brief Schur complement of lambda (updated incrementally) */
	CUberBlockMatrix m_minus_D_inv; /**< @brief negative inverse of the diagonal part of lambda (updated incrementally) */
	bool m_b_last_iteration_updated_schur; /**< @brief Schur complement update flag (if set, the next iteration may update SC incrementally, otherwise batch is needed) */
	// solver data

	double m_f_update_thresh; /**< @brief per-vertex update threshold (thresholds the norm of the vertex update) */
	double m_f_relin_thresh; /**< @brief omega relinearization threshold (thresholds the max of the lambda block update) */
	double m_f_delta; /**< @brief dogleg step size / trust radius */
	double m_f_initial_delta; /**< @brief dogleg step size / trust radius */
	double m_f_v; /**< @brief dogleg step size / trust radius scaling speed */
	bool m_b_keep_step_size; /**< @brief persistent doglig step size flag (if set, the step size is kept, if cleared it is ) */
	bool m_b_all_batch; /**< @brief all-batch flag (takes precedence over \ref m_b_force_inc_schur) */
	bool m_b_force_inc_schur; /**< @brief increment SC even despite the increment being more expensive */
	bool m_b_force_incremental_margs; /**< @brief use the formulas from ICRA 2015 paper for incremental marginals (only if SC is not used) */
	// solver settings

	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	double m_f_chi2_time; /**< @brief time spent in calculating chi2 */
	double m_f_dogleg_time; /**< @brief time spent in calculating dogleg step */
	double m_f_dampingupd_time; /**< @brief time spent in updating the damping */
	double m_f_lambda_time; /**< @brief time spent updating lambda */
	double m_f_omega_time; /**< @brief time spent updating omega */
	double m_f_rhs_time; /**< @brief time spent gathering the right-hand-side vector */
	double m_f_sgd_time; /**< @brief time spent computing the steepest gradient descent scale */
	double m_f_chol_time; /**< @brief time spent in Choleski() section */
	double m_f_norm_time; /**< @brief time spent in norm calculation section */
	double m_f_vert_upd_time; /**< @brief time spent updating the system */
	double m_f_extra_chol_time; /**< @brief time spent in calculating extra Cholesky factorization for marginal covariances */
	double m_f_UDinv_time; /**< @brief time spent in tecalculating the first matrix product \f$U \cdot D^{-1}\f$ for marginal covariance recovery (could be reused from the linear solver, currently not implemented that way) */
	double m_f_marginals_time; /**< @brief time spent in calculating marginal covariances (batch) */
	double m_f_marginals_test_time; /**< @brief time spent in calculating reference marginals and evaluating the precision */
	double m_f_incmarginals_time; /**< @brief time spent in calculating marginal covariances (update) */
	size_t m_n_incmarginals_num; /**< @brief number of times the marginals update ran instead of batch recalculation */
	double m_f_schur_update_time; /**< @brief time spent in incremental Schur complement updates */
	double m_f_ordering_update_time; /**< @brief time spent in incremental Schur complement symbolic ordering updates */
	double m_f_schursolve_alloc; /**< @brief time spent in matrix allocation in Schur complement linear solving */
	double m_f_schursolve_perm; /**< @brief time spent in permuting the matrix to block diagonal form in Schur complement linear solving */
	double m_f_schursolve_rcs_rhs; /**< @brief time spent in calculating the right-hand-side vector for the reduced camera system */
	double m_f_schursolve_rcs_solve; /**< @brief time spent in solcing the reduced camera system */
	double m_f_schursolve_lm_solve; /**< @brief time spent in solving for the landmarks (the eliminated variable update) */
	size_t m_n_schursolve_rcs_solve_times; /**< @brief number of times the Schur complement was used for linear solving */
	// statistics

	size_t m_n_hessian_update_num; /**< @brief number of times lambda/omega were calculated */ // does this match m_n_iteration_num? might not.
	size_t m_n_total_update_rank; /**< @brief rank in lambdas */
	size_t m_n_sparse_update_rank; /**< @brief rank in omegas */
	size_t m_n_vertex_update_num; /**< @brief number of times \ref PushValuesInGraphSystem() was called */ // probably neither matches m_n_iteration_num
	size_t m_n_total_updated_vertex_num; /**< @brief total number of vertices at all times of update */
	size_t m_n_thresh_updated_vertex_num; /**< @brief number of vertices that passed the minimum threshold */
	size_t m_n_full_rank_updare_num; /**< @brief number of times \ref PushValuesInGraphSystem() caused all the vertices to update */
	size_t m_n_inc_scupd_step_num; /**< @brief number of times the incremental Schur complement was used */
	size_t m_n_batch_scupd_step_num; /**< @brief number of times the batch Schur complement was used */
	size_t m_n_sgd_scale_eval_num; /**< @brief number of times the SGD scaling needed to be evaluated */
	// incremental schur specific statistics

	CMargsHelper m_margs; /**< @brief marginal covariances calculation and stats helper */

public:
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
	CNonlinearSolver_Lambda_DL(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:_TyBase(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur),
		m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
		/*m_b_system_dirty(false),*/ m_n_iteration_num(0), m_f_lambda_time(0),
		m_f_omega_time(0), m_f_rhs_time(0), m_f_sgd_time(0), m_f_chol_time(0),
		m_f_norm_time(0), m_f_vert_upd_time(0), m_f_extra_chol_time(0), m_f_UDinv_time(0),
		m_f_marginals_time(0), m_f_marginals_test_time(0), m_f_incmarginals_time(0),
		m_f_schur_update_time(0), m_f_ordering_update_time(0), m_n_incmarginals_num(0),
		m_n_hessian_update_num(0), m_n_total_update_rank(0), m_n_sparse_update_rank(0),
		m_n_vertex_update_num(0), m_n_total_updated_vertex_num(0),
		m_n_thresh_updated_vertex_num(0), m_n_full_rank_updare_num(0),
		m_n_inc_scupd_step_num(0), m_n_batch_scupd_step_num(0),
		m_n_sgd_scale_eval_num(0)
	{
		m_f_update_thresh = 1e-5; // per-vertex update threshold // this should be smaller than the dogleg step threshold, to keep the step direction relatively precise // todo - make this a param, add getter / setter
		m_f_relin_thresh = 1e-5; // per omega block update threshold
		m_n_matrix_cut = 0; // start at 0
		m_n_landmark_group_id = 1;
		// init new members

		m_f_delta = m_f_initial_delta = 2; // t_odo - this is a guess that could fit the BA problems; the user needs to supply her step size // this is now possible by specifying -dlss
		m_b_keep_step_size = false;
		// init the dogleg step size (needs to be given by the user)

		m_f_v = 2;
		// upon the first bad step, the step size is halved

		m_b_last_iteration_updated_schur = true;
		// can keep updating

		m_b_all_batch = false;
		m_b_force_inc_schur = false;
		// use the incremental Schur unless instructed otherwise

		m_b_force_incremental_margs = false;
		// don't use the ICRA'15 code by default

		m_f_chi2_time = 0;
		m_f_dogleg_time = 0;
		m_f_dampingupd_time = 0;

		m_f_schursolve_alloc = 0;
		m_f_schursolve_perm = 0;
		m_f_schursolve_rcs_rhs = 0;
		m_f_schursolve_rcs_solve = 0;
		m_n_schursolve_rcs_solve_times = 0;
		m_f_schursolve_lm_solve = 0;

		if(t_marginals_config.b_calculate) {
			if(t_marginals_config.t_increment_freq.n_period != t_incremental_config.t_nonlinear_freq.n_period &&
			   t_marginals_config.t_increment_freq.n_period != t_incremental_config.t_linear_freq.n_period) {
				throw std::runtime_error("in CNonlinearSolver_Lambda, the marginals must"
					" be updated with the same frequency as the system");
			}
			// unfortunately, yes

			if(t_marginals_config.n_incremental_policy != mpart_Diagonal) {
				throw std::runtime_error("in CNonlinearSolver_Lambda_DL, the marginals update"
					" policy must be mpart_Diagonal");
			}
			if(t_marginals_config.n_incremental_policy != t_marginals_config.n_relinearize_policy) {
				throw std::runtime_error("in CNonlinearSolver_Lambda_DL, the marginals "
					" incremental and relinearize update policy must be the same");
			} // these are now implemented
			if(t_marginals_config.n_cache_miss_policy != mpart_Nothing) {
				throw std::runtime_error("in CNonlinearSolver_Lambda_DL, the marginals cache"
					" miss policy is not supported at the moment, sorry for inconvenience");
			}
			// nothing else is implemented so far
		}

		if(!this->m_b_use_schur)
			fprintf(stderr, "warning: instancing CNonlinearSolver_Lambda_DL without Schur: will be slower\n");
		if(n_dummy_param)
			fprintf(stderr, "warning: using constant Levenberg-Marquardt damping of %d\n", n_dummy_param);
	}

	/**
	 *	@brief gets the current system matrix (not necessarily up-to-date)
	 *	@return Returns const reference to the current system matrix.
	 */
	inline const CUberBlockMatrix &r_Lambda() const
	{
		return m_lambda;
	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("dl-solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
		printf("lambda updated " PRIsize " times (avg full rank %.2f, avg thr rank %.2f)\n",
			m_n_hessian_update_num, double(m_n_total_update_rank) / m_n_hessian_update_num,
			double(m_n_sparse_update_rank) / m_n_hessian_update_num);
		printf("linpoint updated " PRIsize " times (avg full rank %.2f, avg thr rank"
			" %.2f, had " PRIsize " full-rank updates)\n", m_n_vertex_update_num,
			double(m_n_total_updated_vertex_num) / m_n_vertex_update_num,
			double(m_n_thresh_updated_vertex_num) / m_n_vertex_update_num, m_n_full_rank_updare_num);
		if(f_total_time < 0) {
			printf("dl-solver took %f seconds in total\n", (m_f_lambda_time +
				m_f_omega_time + m_f_rhs_time + m_f_ordering_update_time +
				m_f_schur_update_time + m_f_chol_time + m_f_chi2_time + m_f_dogleg_time + m_f_sgd_time +
				m_f_dampingupd_time + m_f_norm_time + m_f_vert_upd_time +
				m_f_extra_chol_time + m_f_UDinv_time + m_f_marginals_time + m_f_incmarginals_time));
		}
		printf("dl-solver spent %f seconds in parallelizable section (disparity %f seconds)\n",
			m_f_lambda_time + m_f_omega_time + m_f_rhs_time, (f_total_time > 0)? f_total_time -
			(m_f_lambda_time + m_f_omega_time + m_f_rhs_time + m_f_ordering_update_time +
			m_f_schur_update_time + m_f_chol_time + m_f_chi2_time + m_f_dogleg_time + m_f_sgd_time +
			m_f_dampingupd_time + m_f_norm_time + m_f_vert_upd_time +
			m_f_extra_chol_time + m_f_UDinv_time + m_f_marginals_time +
			m_f_marginals_test_time + m_f_incmarginals_time) : 0);

		printf("out of which:\n");
		printf("\t   ,\\: %f\n", m_f_lambda_time);
		printf("\tomega: %f\n", m_f_omega_time);
		printf("\t  rhs: %f\n", m_f_rhs_time);
		if(this->m_t_marginals_config.b_calculate) {
			printf("solver spent %f seconds in marginals\n"
				"\t chol: %f (discount this, could reuse the one from linear solver)\n"
				"\tUDinv: %f\n"
				"\tmargs: %f\n"
				"\t incm: %f (ran " PRIsize " times)\n",
				m_f_extra_chol_time + m_f_UDinv_time + m_f_marginals_time + m_f_incmarginals_time,
				m_f_extra_chol_time, m_f_UDinv_time, m_f_marginals_time,
				m_f_incmarginals_time, m_n_incmarginals_num);
			if(m_f_marginals_test_time)
				printf("\t test: %f\n", m_f_marginals_test_time);
			m_margs.Dump();
		}
		printf("solver spent %f seconds in serial section\n",
			m_f_ordering_update_time + m_f_schur_update_time + m_f_chol_time +
			m_f_chi2_time + m_f_dogleg_time + m_f_dampingupd_time + m_f_norm_time + m_f_sgd_time + m_f_vert_upd_time);
		printf("out of which:\n");
		printf("\t ordup: %f\n", m_f_ordering_update_time); // schur ordering update
		printf("\t schup: %f (incremented " PRIsize " times out of " PRIsize ")\n", m_f_schur_update_time,
			m_n_inc_scupd_step_num, m_n_inc_scupd_step_num + m_n_batch_scupd_step_num); // schur update
		printf("\t  chi2: %f\n", m_f_chi2_time); // (re)calculating chi2
		printf("\t solve: %f\n", m_f_chol_time); // linear solcing
		printf("\t\talloc: %f\n", m_f_schursolve_alloc);
		printf("\t\t perm: %f\n", m_f_schursolve_perm);
		printf("\t\tscrhs: %f\n", m_f_schursolve_rcs_rhs);
		printf("\t\tscsol: %f (ran " PRIsize " times)\n", m_f_schursolve_rcs_solve, m_n_schursolve_rcs_solve_times);
		printf("\t\tdgsol: %f\n", m_f_schursolve_lm_solve);
		printf("\t  norm: %f\n", m_f_norm_time); // calculating the GN solution norm
		printf("\t   sgd: %f (ran " PRIsize " times)\n", m_f_sgd_time, m_n_sgd_scale_eval_num); // calculating the sgd denominator
		printf("\tdogleg: %f\n", m_f_dogleg_time); // calculating the dogleg step
		printf("\t v-upd: %f\n", m_f_vert_upd_time); // updating the vertices
		printf("\t d-upd: %f\n", m_f_dampingupd_time); // updating the damping
	}

	/**
	 *	@brief gets the last update vector (for debugging purposes)
	 *	@return Returns const reference to the last update vector.
	 */
	const Eigen::VectorXd &r_v_LastDx() const
	{
		return m_v_dx;
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
			Optimize(0);
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
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size() -
			this->n_LastOptimized_Vertex_Num();
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_edge_num = this->n_New_Edge_Num() + 1;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES

		std::pair<bool, int> t_optimize = this->t_Incremental_Step(r_last_edge);
		bool b_new_vert = t_optimize.first, b_ran_opt = t_optimize.second != 0;

		if(t_optimize.second == 2) {
			Optimize(this->m_t_incremental_config.n_max_nonlinear_iteration_num,
				this->m_t_incremental_config.f_nonlinear_error_thresh); // nonlinear optimization
		} else if(t_optimize.second == 1)
			Optimize(1, 0); // linear optimization
		// decide on incremental optimization

		if(b_new_vert && !b_ran_opt && this->m_t_marginals_config.b_calculate)
			Optimize(0, 0);
		// run optimization in order to calculate marginals after each vertex
	}

	/**
	 *	@brief sets the marginal covariance calculation style
	 *	@param[in] b_enable_incremental is incremental marginal covariance calculation flag
	 *		(if set, the formulas from ICRA 2015 paper are used, if cleated then recursive
	 *		formula is used)
	 *	@note This only takes effect for non-Schur-complemented systems. For Schur-complemented
	 *		systems, the 3DV 2017 formulas are always used.
	 */
	void Set_ICRA15_Style_IncrementalMargs(bool b_enable_incremental)
	{
		m_b_force_incremental_margs = b_enable_incremental;
	}

	/**
	 *	@brief gets the marginal covariance calculation style
	 *	@return Returns true if the formulas from ICRA 2015 paper are used, false if the recursive
	 *		formula will be used.
	 *	@note This only takes effect for non-Schur-complemented systems. For Schur-complemented
	 *		systems, the 3DV 2017 formulas are always used.
	 */
	bool b_ICRA15_Style_IncrementalMargs() const
	{
		return m_b_force_incremental_margs;
	}

	/**
	 *	@brief sets all-batch processing
	 *	@param[in] b_all_batch is the new all-batch flag (applies only if Schur
	 *		complement is used, otherwise all-batch is default)
	 */
	void Set_AllBatch(bool b_all_batch)
	{
		m_b_all_batch = b_all_batch;
	}

	/**
	 *	@brief gets all-batch processing flag
	 *	@return Returns true if the linear solver recalculates Schur complement from
	 *		scratch at each step, or false if the incremental Schur complement is used.
	 *	@note The incremental Schur complement is used only if Schur complement is enabled.
	 */
	bool b_AllBatch() const
	{
		return m_b_all_batch;
	}

	/**
	 *	@brief sets the force incremental Schur complement flag
	 *	@param[in] b_force_inc_schur is force incremental Schur complement flag (if set,
	 *		incremental Schur complement is used at each step, if cleared a heuristic is used
	 *		to decide whether to do a batch or incremental step)
	 *	@note Incremental Schur complement is unly used if Schur complement is enabled and all batch is disabled.
	 */
	void Set_ForceIncSchur(bool b_force_inc_schur)
	{
		m_b_force_inc_schur = b_force_inc_schur;
	}

	/**
	 *	@brief gets the force incremental Schur complement flag
	 *	@return Returns true if the Schur complement will always be updated using the incremental
	 *		formula, false if heuristic is used to enable or disable incremental Schur complement update.
	 *	@note Incremental Schur complement is unly used if Schur complement is enabled and all batch is disabled.
	 */
	bool b_ForceIncSchur() const
	{
		return m_b_force_inc_schur;
	}

	/**
	 *	@brief sets update threshold
	 *	@param[in] f_update_thresh is the new update threshold (applied per vertex)
	 */
	void Set_UpdateThreshold(double f_update_thresh)
	{
		m_f_update_thresh = f_update_thresh;
	}

	/**
	 *	@brief gets update threshold
	 *	@return Returns the current update threshold.
	 */
	double f_UpdateThreshold() const
	{
		return m_f_update_thresh;
	}

	/**
	 *	@brief sets relinearization threshold
	 *	@param[in] f_relin_thresh is the new relinearization threshold
	 */
	void Set_RelinearizationThreshold(double f_relin_thresh)
	{
		m_f_relin_thresh = f_relin_thresh;
	}

	/**
	 *	@brief gets relinearization threshold
	 *	@return Returns the current relinearization threshold.
	 */
	double f_RelinearizationThreshold() const
	{
		return m_f_relin_thresh;
	}

	/**
	 *	@brief sets dogleg step size persistence
	 *	@param[in] b_persistent_step_size is persistence flag (set to make dogleg step size
	 *		persistent, clear for the step size to be reset at each call of \ref Optimize())
	 */
	void Set_StepSize_Persistence(bool b_persistent_step_size)
	{
		m_b_keep_step_size = b_persistent_step_size;
	}

	/**
	 *	@brief Determines whether dogleg step size resets to the default value at each call of \ref Optimize().
	 *	@return Returns true if dogleg step size is persistent, otherwise returns false.
	 */
	bool b_StepSize_Persistent() const
	{
		return m_b_keep_step_size;
	}

	/**
	 *	@brief sets dogleg step size
	 *	@param[in] f_new_delta is the new dogleg step size
	 */
	void Set_StepSize(double f_new_delta)
	{
		m_f_delta = m_f_initial_delta = f_new_delta;
	}

	/**
	 *	@brief gets dogleg step size
	 *	@return Returns the current dogleg step size.
	 */
	double f_StepSize() const
	{
		return m_f_initial_delta;
	}

	/**
	 *	@brief norify the solver of linearization point update (e.g. change in robust
	 *		function parameters, external change to the current estimate, ...)
	 *
	 *	@param[in] n_first_changing_edge is zero-based index of the first edge being changed
	 *	@param[in] n_first_changing_vertex is zero-based index of the first vertex being changed
	 *
	 *	@todo This is mostly untested; test this.
	 *
	 *	@note It is usually sufficient to provide either the first changing edge or the first
	 *		changing vertex in case those specify the same (or similar) variables.
	 *	@note This function throws std::bad_alloc.
	 */
	void Notify_LinearizationChange(size_t n_first_changing_edge = 0,
		size_t n_first_changing_vertex = 0) // throw(std::bad_alloc)
	{
		_ASSERTE(!n_first_changing_edge || n_first_changing_edge < this->m_r_system.r_Edge_Pool().n_Size());
		_ASSERTE(!n_first_changing_vertex || n_first_changing_vertex < this->m_r_system.r_Vertex_Pool().n_Size());
		// make sure those are valid indices

		const size_t n_edge_num = this->m_r_system.r_Edge_Pool().n_Size();
		const size_t n_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size();

		if(!n_first_changing_edge || !n_first_changing_vertex) {
			if(m_relin_vertex_list.capacity() < n_vertex_num)
				m_relin_vertex_list.clear();
			m_relin_vertex_list.resize(n_vertex_num);
			for(size_t n_vert = 0; n_vert < n_vertex_num; ++ n_vert)
				m_relin_vertex_list[n_vert] = n_vert;
			// all vertices change
		} else {
			for(size_t n_vert = n_first_changing_vertex; n_vert < n_vertex_num; ++ n_vert)
				m_relin_vertex_list.push_back(n_vert);
			// all vertices

			if(m_relin_vertex_list.size() < n_vertex_num) {
				this->m_r_system.r_Edge_Pool().For_Each(n_first_changing_edge, n_edge_num,
					CCollectVertexIds(m_relin_vertex_list, n_vertex_num));
				std::sort(m_relin_vertex_list.begin(), m_relin_vertex_list.end());
				m_relin_vertex_list.erase(std::unique(m_relin_vertex_list.begin(),
					m_relin_vertex_list.end()), m_relin_vertex_list.end());
			}
			// and also all the vertices affected by the changing edges
		}
		// mark the specified vertices as dirty, to force relinearization in the next step
		// (note that all the edges will be relinearized, see the call to _TyLambdaOps::Refresh_Lambda()
		// from UpdateLambda_CalculateOmega())
	}

	/**
	 *	@brief final optimization function
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is both the residual norm and the dogleg step threshold
	 */
	void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01) // throw(std::bad_alloc)
	{
		Optimize(n_max_iteration_num, f_min_dx_norm, f_min_dx_norm);
	}

	/**
	 *	@brief final optimization function
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 *	@param[in] f_min_dl_norm is the dogleg step norm threshold (thresholds
	 *		values in range [norm(dx), step size])
	 */
	void Optimize(size_t n_max_iteration_num,
		double f_min_dx_norm, double f_min_dl_norm) // throw(std::bad_alloc)
	{
		CTimerSampler timer(this->m_timer);

		if(!_TyGOH::b_can_use_guided_ordering)
			throw std::runtime_error("CNonlinearSolver_Lambda_DL does not support MIS ordering yet");
		// with guided ordering, it is easier to do the ordering incrementally. will support MIS later.

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

		// todo - identify which variables are new and figure out increments in the RCS and in D
		// todo - in case the ordering is bad (less than e.g. half of the matrix on the diagonal), see
		//		  if we could come up with a better ordering and redo the schur from schratch without omega.
		//		  this could be done well by calculating how many vertices need to be added in the best case
		//		  in order to improve the ordering sufficiently if all were added to the diagonal and only
		//		  retrying the new ordering after so many vertices were added (+maybe some minimum constant).
		// todo - add a strategy for combating numerical drift and redo the schur from schratch if that happens
		// todo - see what would happen if we wouldnt use norm(dx) at all but instead chose to relinearize the
		//		  vertices always; would likely improve convergence and could also reuse the min dx norm as the
		//		  threshold (although would need to modify the defaults for the DL solver)
		// todo - make a function for affected edge selection, consider making a reverse vertex lookup (dual
		//		  graph) and marking edges for update at the same time as the vertices are being updated

		// todo - handle situations where either all or none vertices are diagonal

		// t_odo - add update threshold / rank the updates and keep a vector of relinearized variables
		// t_odo - handle the relinearized variables, might need to remember the delta values or something
		// t_odo - see how can we use omega?
		// todo - see about (block)-RCM ordering for the factor; it changes every time, use dense cholesky

		double f_delta = m_f_delta; //m_f_delta = .2; // re-init the dogleg step size (needs to be given by the user)
		double f_v = m_f_v; //m_f_v = 2; // upon the first bad step, the step size is halved
		// reset the settings (todo - use a local variable instead)

		const bool b_prev_lambda_empty = !m_n_edges_in_lambda;
		// need this for error checking later on

		const size_t n_verts_in_prev_lambda = m_n_verts_in_lambda;
		// need this for ordering

		const size_t n_relin_vertex_num = m_relin_vertex_list.size(); // stats
		CUberBlockMatrix omega;
		UpdateLambda_CalculateOmega(omega, timer);
		// get omega, keep lambda up to date

		if(m_lambda.n_BlockColumn_Num() < this->m_r_system.r_Vertex_Pool().n_Size()) {
			fprintf(stderr, "warning: waiting for more edges\n");

			if(!b_prev_lambda_empty)
				throw std::runtime_error("CNonlinearSolver_Lambda_DL failed to track lambda updates");
			// in case there was already something in there, then we lost the information
			// about the previous linearization point; this however mostly happens at the
			// beginning of the dataset if at all so it should be ok (otherwise will need
			// to re-negate omega and accumulate it over several steps)

			m_n_verts_in_lambda = 0;
			m_n_edges_in_lambda = 0;
			m_lambda.Clear();
			// omega is not stored so it would be difficult to recalculate it again

			return;
		}
		// waiting for more edges (happens if the vertices are initialized explicitly)

		_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
			m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
			m_lambda.n_Column_Num() == n_variables_size); // lambda is square, blocks on either side = number of vertices
		// invariants

		m_v_eta.resize(n_variables_size, 1);
		m_v_dx.resize(n_variables_size, 1);

		bool b_reordering_needed = false;
		if(this->m_b_use_schur) {
			std::vector<size_t> new_rcs_vertices;

			size_t p_vertex_dimensions[_TyGOH::n_vertex_group_num];
			if(!_TyGOH::b_can_use_simple_ordering)
				_TyGOH::Get_VertexDimensions(p_vertex_dimensions, sizeof(p_vertex_dimensions) / sizeof(p_vertex_dimensions[0]));
			// gather dimensions of partite vertex types

			size_t p_vertex_dimension_sum[_TyGOH::n_vertex_group_num] = {0}; // todo - need to make this a part of the state, to be able to avoid summing up old vertex sizes
			// sum of dimensions of all the vertex types

			std::vector<size_t> p_vertex_type_indices[_TyGOH::n_vertex_group_num]; // the first list is for the non-partite vertices, if there are any
			// the same size as p_vertex_dimensions, except that they are ordered in the opposite direction

			for(size_t i = n_verts_in_prev_lambda, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
				size_t n_dim = m_lambda.n_BlockColumn_Column_Num(i);
				if(_TyGOH::b_can_use_simple_ordering) {
					enum {
						n_landmark_dim = _TyGOH::n_smallest_vertex_dim,
						n_pose_dim = _TyGOH::n_greatest_vertex_dim
					};
					if(n_dim == n_landmark_dim)
						m_schur_ordering.push_back(i); // this will be in the diagonal part
					else
						new_rcs_vertices.push_back(i); // this will be in the RCS part
					// simple case
				} else {
					size_t n_group = _TyGOH::n_Vertex_GroupIndex(n_dim);
					p_vertex_dimension_sum[n_group] += n_dim;
					if(m_n_landmark_group_id == n_group)
						m_schur_ordering.push_back(i); // this will be in the diagonal part
					else
						new_rcs_vertices.push_back(i); // this will be in the RCS part
					// case with groups
				}
			}
			// for all new vertices

			if(!_TyGOH::b_can_use_simple_ordering) {
				for(size_t i = 0; i < n_verts_in_prev_lambda; ++ i) {
					size_t n_dim = m_lambda.n_BlockColumn_Column_Num(i);
					size_t n_group = _TyGOH::n_Vertex_GroupIndex(n_dim);
					p_vertex_dimension_sum[n_group] += n_dim;
				}
				// for all old vertices (could cache this)

				size_t n_best_group_size = p_vertex_dimension_sum[m_n_landmark_group_id];
				size_t n_best_group_id = m_n_landmark_group_id;
				for(int i = _TyGOH::b_have_nonpartite_vertex_types; i < _TyGOH::n_vertex_group_num; ++ i) {
					if(n_best_group_size < p_vertex_dimension_sum[i]) {
						n_best_group_size = p_vertex_dimension_sum[i];
						n_best_group_id = i;
					}
				}
				if(n_best_group_id != m_n_landmark_group_id) {
					//throw std::runtime_error("not implemented");
					fprintf(stderr, "warning: using untested SC reordering branch\n"); // just so that we know that this takes place // todo - test this (will probably need a special graph to do that)

					b_reordering_needed = true;
					m_n_landmark_group_id = n_best_group_id;

					m_n_matrix_cut = 0;
					m_schur_ordering.clear();
					new_rcs_vertices.clear();
					for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
						size_t n_dim = m_lambda.n_BlockColumn_Column_Num(i);
						size_t n_group = _TyGOH::n_Vertex_GroupIndex(n_dim);
						if(m_n_landmark_group_id == n_group)
							m_schur_ordering.push_back(i); // this will be in the diagonal part
						else
							new_rcs_vertices.push_back(i); // this will be in the RCS part
						// case with groups
					}
					// reorder all vertices
				}
			}
			// see if reordering is needed

			m_schur_ordering.insert(m_schur_ordering.begin() + m_n_matrix_cut,
				new_rcs_vertices.begin(), new_rcs_vertices.end());
			m_n_matrix_cut += new_rcs_vertices.size();
			// increment the ordering
		}
		// incremental Schur ordering // todo - make this a function, support incremental MIS
		// todo - if b_prev_lambda_empty is set, use from the scratch ordering, this can become subobtimal and reordered in the next step

		const bool b_avoid_schur = (this->m_b_use_schur && !m_b_all_batch && (!m_n_matrix_cut ||
			m_n_matrix_cut == m_schur_ordering.size()));
		// in case the matrix is entirely diagonal or not diagonal at all, avoid using the incremental
		// Schur complement branch (the all batch already handles entirely diagonal matrices correctly)
		// this mostly handles the small matrices

		const bool b_use_schur = this->m_b_use_schur && !b_avoid_schur;
		// determine whether to use SC in this iteration

		CMatrixOrdering mord;
		const size_t *p_schur_order = (b_use_schur)? mord.p_InvertOrdering(&m_schur_ordering.front(),
			m_schur_ordering.size()) : 0;
		const size_t n_cut = m_n_matrix_cut, n_size = m_schur_ordering.size();
		// invert the ordering

		timer.Accum_DiffSample(m_f_ordering_update_time);

		double f_last_error = this->f_Chi_Squared_Error_Denorm();
		if(this->m_b_verbose)
			printf("chi2: %f\n", f_last_error);
		// calculate and dump chi2

		timer.Accum_DiffSample(m_f_chi2_time);

		CUberBlockMatrix lambda_perm_structure, newU;
		if(b_use_schur && !m_b_all_batch) {
			if(m_b_last_iteration_updated_schur && !b_reordering_needed)
				Update_Schur(p_schur_order, n_cut, n_size, omega, lambda_perm_structure, newU, n_relin_vertex_num);
			else
				Redo_Schur(p_schur_order, n_cut, n_size, lambda_perm_structure, newU);
			m_b_last_iteration_updated_schur = true; // it is now updated
		} else if(!b_use_schur)
			m_b_last_iteration_updated_schur = false; // the next iteration needs to redo from scratch
		// update the Schur complement member variables

		timer.Accum_DiffSample(m_f_schur_update_time);

		if(b_use_schur) {
			if(m_b_all_batch) // incSchur pipeline flag
				this->m_schur_solver.SymbolicDecomposition_Blocky(m_lambda, true); // force guided ordering here
		}
		// calculate the ordering once, it does not change

#if 0 // not all edges have reprojection error function; this effectively breaks all non-BA solver implementations
		if(this->m_b_verbose) {
			static bool b_ran = false;
			if(!b_ran) {
				b_ran = true;
				Eigen::Vector5d v_errors = v_ReprojectionErrors();
				printf("initial reprojection error: avg %f, RMSE %f, r-avg %f, r-RMSE %f, r-weight sum %f / "
					PRIsize " (for " PRIsize " cameras)\n", v_errors(0), v_errors(1), v_errors(2), v_errors(3), v_errors(4),
					this->m_r_system.r_Edge_Pool().n_Size(), n_cut);
				// calculate and dump reprojection error
			}
		}
		// debug
#endif // 0

		if(this->m_b_verbose && this->m_t_incremental_config.b_IsBatch()) {
			size_t n_sys_size = this->m_r_system.n_Allocation_Size();
			size_t n_rp_size = m_reduction_plan.n_Allocation_Size();
			size_t n_lam_size = m_lambda.n_Allocation_Size();
			printf("memory_use(sys: %.2f MB, redplan: %.2f MB, Lambda: %.2f MB)\n",
				n_sys_size / 1048576.0, n_rp_size / 1048576.0, n_lam_size / 1048576.0);
		}
		// print memory use statistics

		//size_t n_max_fail_num = 10; // allow some bad steps to recover // not needed in dogleg
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

			if(n_iteration && !m_relin_vertex_list.empty()) {
				size_t n_relin_vertex_num = m_relin_vertex_list.size();
				UpdateLambda_CalculateOmega(omega, timer);
				// update lambda and get a new omega

				if(b_use_schur && !m_b_all_batch)
					Update_Schur(p_schur_order, n_cut, n_size, omega, lambda_perm_structure, newU, n_relin_vertex_num);
				// update the Schur complement member variables

				timer.Accum_DiffSample(m_f_schur_update_time);
			}
			// no need to rebuild lambda, just refresh the values that are being referenced
			// timer sample taken inside

			_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_eta);
			// collects the right-hand side vector

			const Eigen::VectorXd v_orig_eta = m_v_eta; // back this up for later on

			timer.Accum_DiffSample(m_f_rhs_time);

			double f_gradient_descent_scale_denom = 0;
			bool b_have_gradient_descent_scale_denom = false;

			/*this->m_r_system.r_Edge_Pool().For_Each(
				CSumSteepestGradientDescentScaleDenom(f_gradient_descent_scale_denom, m_v_eta)); // t_odo - remove this, do it differently to take advantage of the calculated jacobians
			// sum up ||J J^t Sigma^-1 r||^2 for scaling the gradient descent steps in dogleg

			timer.Accum_DiffSample(m_f_sgd_time);
			// if the rhs time is large, could collect sparse rhs (only the parts
			// that correspond to (some of the) nnz omega columns actually change)*/ // this is evaluated lazily

			// note - can put a threshold on the gradient as well (if the gradient is zero,
			// we're done); this has robustness and scaling issues though, better not apply
			// this one

			const bool b_enable_cauchy_step = true;

			{
				bool b_cholesky_result;
				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					v_eta = m_v_eta; // need to copy; dogleg needs the original vector as well
					if(!b_use_schur) {
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
						if(!m_b_all_batch) { // incSchur pipeline flag
							b_cholesky_result = IncSchur_Solve(lambda_perm_structure, m_SchurCompl,
								m_minus_D_inv, newU, v_eta, p_schur_order, n_cut, n_size, n_max_iteration_num > 1);
							// incremental schur, implemented in here (more matrices are available)

							if(!b_enable_cauchy_step && !b_cholesky_result) { // bigtodo todo - look into this. it fails e.g. on batch venice (a large increment). diff the matrices, see what's the matter
								fprintf(stderr, "error: IncSchur_Solve() failed: trying again with batch solve\n");
								Redo_Schur(p_schur_order, n_cut, n_size, lambda_perm_structure, newU);
								b_cholesky_result = IncSchur_Solve(lambda_perm_structure, m_SchurCompl,
									m_minus_D_inv, newU, v_eta, p_schur_order, n_cut, n_size, true);
							}
							// if that fails, perhaps we drifted too far, redo SC using batch
						} else {
							b_cholesky_result = this->m_schur_solver.Solve_PosDef_Blocky(m_lambda, v_eta);
							// Schur
						}
					}

					if(this->m_b_verbose) {
						if(b_enable_cauchy_step && !b_cholesky_result)
							printf("making an indefinite step\n");
						else {
							printf("%s %s\n", (b_use_schur)? "Schur" : "Cholesky",
								(b_cholesky_result)? "succeeded" : "failed");
						}
					}

#if defined(_DEBUG) || defined(__NONLINEAR_SOLVER_LAMBDA_DL_INC_LINSOLVE_DEBUGGING)
					if(b_cholesky_result) {
						Eigen::VectorXd v_rhs = -m_v_eta;
						m_lambda.SymmetricMultiply_Add/*_ExpDiag*/(&v_rhs(0), v_rhs.rows(), &m_v_dx(0), m_v_dx.rows(), true);
						// multiply back, only use the upper half of lambda
						// note that the diagonal of m_lambda may not be entirely symmetric, due to numeric errors

						CUberBlockMatrix lambda_full;
						lambda_full.SelfAdjointViewOf/*SelfAdjointView_ExpDiag_Of*/(m_lambda, true);
						Eigen::VectorXd v_rhs_ref = -m_v_eta;
						lambda_full.PostMultiply_Add(&v_rhs_ref(0), v_rhs_ref.rows(), &m_v_dx(0), m_v_dx.rows());
						Eigen::VectorXd v_rhs_ref2 = -m_v_eta;
						lambda_full.PreMultiply_Add(&v_rhs_ref2(0), v_rhs_ref2.rows(), &m_v_dx(0), m_v_dx.rows());
						double f_conventional_norm = (v_rhs_ref.norm() + v_rhs_ref2.norm()) / 2;
						double f_conventional_diff = (v_rhs_ref - v_rhs_ref2).norm();
						double f_greater_diff = std::max((v_rhs - v_rhs_ref).norm(), (v_rhs - v_rhs_ref2).norm());
						double f_lesser_diff = std::min((v_rhs - v_rhs_ref).norm(), (v_rhs - v_rhs_ref2).norm());
						//double f_multiplication_error = (v_rhs - v_rhs_ref).norm();
						_ASSERTE(f_greater_diff <= 2 * f_conventional_diff + 1e-10); // the greater diff is mostly equal to the diff of pre / post multiplication of the symmetric matrix
						_ASSERTE(f_lesser_diff <= std::max(f_conventional_diff, 1e-10)); // the smaller diff is smaller or equal (typically zero)
						double f_lambda_norm = m_lambda.f_LInfNorm(); // this is also a factor
						_ASSERTE(f_conventional_diff / std::max(std::max(f_lambda_norm, f_conventional_norm), 1.0) < 1e-10); // this should work
						// debug debug - test SymmetricMultiply_Add()

						/*lambda_full.Save_MatrixMarket("lambda_full.mtx", "lambda_full.bla");
						FILE *p_fw = fopen("vectors.m", "w");
						if(p_fw) {
							CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &m_v_eta(0), m_v_eta.rows(), "eta = ");
							CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &m_v_dx(0), m_v_dx.rows(), "dx = ");
							CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_rhs(0), v_rhs.rows(), "result = ");
							CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_rhs_ref(0), v_rhs_ref.rows(), "refa = ");
							CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_rhs_ref2(0), v_rhs_ref2.rows(), "refb = ");
							fclose(p_fw);
						}
						// debug*/

						double f_rhs_norm = v_rhs.norm(), f_eta_norm = m_v_eta.norm(), f_dX_norm = m_v_dx.norm();
						printf("debug: linear solution precision: %g (eta: %g, dx: %g, relative: %g)\n",
							f_rhs_norm, f_eta_norm, f_dX_norm, f_rhs_norm / f_dX_norm);
					}
					// see how precise is the linear solver; could do a second iteration
#endif // _DEBUG || __NONLINEAR_SOLVER_LAMBDA_DL_INC_LINSOLVE_DEBUGGING
				}
				// calculate cholesky, reuse block ordering if the linear solver supports it

				timer.Accum_DiffSample(m_f_chol_time);

				if(!b_cholesky_result && !b_enable_cauchy_step)
					break;
				// in case Cholesky failed, quit

				double f_residual_norm = 0;
				if(b_cholesky_result) {
					f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP

					timer.Accum_DiffSample(m_f_norm_time);
				}
				// calculate residual norm

				if(b_cholesky_result && f_residual_norm <= f_min_dx_norm) {
					if(this->m_b_verbose)
						printf("residual norm: %.4f, early exit\n", f_residual_norm); // otherwise there'd be no message
					break;
				}
				// in case GN returns very small step, we are done

				const Eigen::VectorXd v_orig_dx = m_v_dx; // back this up for later on

				bool b_break_outer_loop = false;
				for(;;) {
					b_break_outer_loop = true; // if we break inside this loop, then break out of the outer loop as well

					{
						double f_dl_norm = 0;
						double f_direction_lerp; // beta in the paper
						const double f_steepest_descent_norm = m_v_eta.norm();
						/*const */double f_sgd_scale = f_steepest_descent_norm *
							f_steepest_descent_norm;// / f_gradient_descent_scale_denom; // alpha in the paper

						if(!b_cholesky_result || !(f_residual_norm <= f_delta)) { // if Cauchy or if not GN-only
							if(!b_have_gradient_descent_scale_denom) {
								++ m_n_sgd_scale_eval_num;

#if 0
								f_gradient_descent_scale_denom = 0;
#if 0
								this->m_r_system.r_Edge_Pool().For_Each(
									CSumSteepestGradientDescentScaleDenom(f_gradient_descent_scale_denom, m_v_eta)); // todo - remove this, do it differently to take advantage of the calculated jacobians
								// in series
#else // 0
								#pragma omp parallel if(this->m_r_system.r_Edge_Pool().n_Size() > 50)
								{
									double f_thread_accum = 0;
									this->m_r_system.r_Edge_Pool().For_Each_WorkShare(
										CSumSteepestGradientDescentScaleDenom(f_thread_accum, m_v_eta));

									#pragma omp atomic
									f_gradient_descent_scale_denom += f_thread_accum;
								}
								// in parallel, each thread accumulates in its own variable
#endif // 0
								// sum up ||J J^t Sigma^-1 r||^2 for scaling the gradient descent steps in dogleg
#else // 0
								Eigen::VectorXd v_rhs = Eigen::VectorXd::Zero(m_v_eta.rows());
								m_lambda.SymmetricMultiply_Add(&v_rhs(0), v_rhs.rows(), &m_v_eta(0), m_v_eta.rows(), true);
								double f_bLambT = m_v_eta.dot(v_rhs); // this can be negative if not SPD, right? not sure if useful for non-SPD detection, probably not very precise at all
								// multiply back, only use the upper half of lambda
#endif // 0
								/*printf("debug: gradient descent scale denom ref: %g, fast: %g, diff: %g (rel %g)\n",
									f_gradient_descent_scale_denom, f_bLambT,
									fabs(f_gradient_descent_scale_denom - f_bLambT),
									fabs(f_gradient_descent_scale_denom - f_bLambT) /
									std::max(f_gradient_descent_scale_denom, 1.0));*/
								// debug - figure out how to get this cheaper

								f_gradient_descent_scale_denom = f_bLambT;

								timer.Accum_DiffSample(m_f_sgd_time);
								// if the rhs time is large, could collect sparse rhs (only the parts
								// that correspond to (some of the) nnz omega columns actually change)

								b_have_gradient_descent_scale_denom = true;
							}
							f_sgd_scale /= f_gradient_descent_scale_denom;
						}
						// lazily evaluate f_gradient_descent_scale_denom and adjust f_sgd_scale

						if(b_cholesky_result) {
							//const double f_delta = m_f_delta; // dogleg step size, set by the caller and adjusted in the iteration

							Eigen::VectorXd &v_dl = m_v_dx;
							if(f_residual_norm <= f_delta) {
								f_direction_lerp = 1; // all Gauss-Newton
								//v_dl = m_v_dx; // use Gauss-Newton step
								// do nothing, this is already in place
							} else if(f_sgd_scale * f_steepest_descent_norm >= f_delta) {
								f_direction_lerp = 0; // all gradient descent
								v_dl = m_v_eta * /*-*/(f_delta / f_steepest_descent_norm); // eta is in the opposite direction from the steepest descent // should not be
							} else {
								Eigen::VectorXd &a = /*f_sgd_scale * */m_v_eta; // steepest descent step // work inplace
								a *= /*-*/f_sgd_scale; // eta is in the opposite direction from the steepest descent // should not be
								Eigen::VectorXd &b = m_v_dx; // Gauss-Newton step // work inplace
								b -= a; // only need (b - a) below, never b by itself
								double f_step_distance_sqr = (b/* - a*/).squaredNorm();
								double c = a.dot(b/* - a*/); // ã^T * (b - a)
								double f_denom = sqrt((c * c - f_step_distance_sqr * f_sgd_scale * f_sgd_scale) + // do this first, f_sgd_scale is typically pretty small, try to avoid some of the roundoff
									f_step_distance_sqr * f_delta * f_delta);
								double f_direction_lerp1;
								if(c <= 0) {
									f_direction_lerp = (-c + f_denom) / f_step_distance_sqr;
									f_direction_lerp1 = 0;
								} else {
									double f_term0 = f_delta * f_delta, f_term1 = -f_sgd_scale * f_sgd_scale;
									if(fabs(f_term0) < fabs(f_term1))
										std::swap(f_term0, f_term1); // want term0 to be the greater number
									double f_sum = f_term0 + f_term1;
									double f_roundoff = f_sum - f_term0;
									double f_sum1 = f_term1 - f_roundoff;
									f_direction_lerp = f_sum / (c + f_denom);
									f_direction_lerp1 = f_sum1 / (c + f_denom);

									a += f_direction_lerp1 * (b/* - a*/); // extended precision version; add the low part to a (which is presumably short)
								}
								_ASSERTE(f_direction_lerp >= 0 && f_direction_lerp <= 1); // somewhere between sd and GN
								// solve quadratic equation, pay attention to always divide by a large quantity

								v_dl = a + f_direction_lerp * (b/* - a*/); // no aliasing in vector math, can go ahead
								/*v_dl *= f_direction_lerp;
								v_dl += a;*/
								// perform the computation with b being in place of v_dl

								_ASSERTE(fabs(f_delta - v_dl.norm()) / std::max(1.0, f_delta) < 1e-1);
								// this is sometimes not extremely precise due to roundoff and large norm differences of the a and b
								// vectors (the extended arithmetics hack helps but there is still a lot of roundoff in b - a and c)
							}
							// compute the dogleg step

							f_dl_norm = (f_residual_norm <= f_delta)? f_residual_norm : f_delta/*v_dl.norm()*/;
							_ASSERTE(fabs(f_dl_norm - v_dl.norm()) / std::max(1.0, v_dl.norm()) < 1e-1); // this is sometimes not extremely precise due to roundoff and large norms of the vectors
							// compute the norm of the dogleg step

							v_dl *= f_dl_norm / v_dl.norm();
							_ASSERTE(fabs(f_dl_norm - v_dl.norm()) / std::max(1.0, v_dl.norm()) < 1e-10); // now it should be rather precise
							// make sure it has the correct length
						}
						// Powell's dogleg

						if(!b_cholesky_result) {
							double f_step_size;
							if(f_steepest_descent_norm < 1e-10)
								f_step_size = 0;
							else if(f_sgd_scale > 0 && f_sgd_scale < 1e300) // "cheap" isfinite
								f_step_size = std::min(f_sgd_scale, f_delta / f_steepest_descent_norm);
							else
								f_step_size = f_delta / f_steepest_descent_norm;
							// calculate step size

							Eigen::VectorXd &v_dl = m_v_dx;
							v_dl = m_v_eta * f_step_size;
							f_dl_norm = f_steepest_descent_norm * f_step_size;
							// and that should be that

							_ASSERTE(fabs(f_dl_norm - v_dl.norm()) / std::max(1.0, v_dl.norm()) < 1e-10); // now it should be rather precise
							// make sure it has the correct length

							// todo - count cauchy steps
						}
						// indefinite Cauchy step

						if(this->m_b_verbose) {
							if(b_cholesky_result) {
								double f_weight_GN = f_direction_lerp * f_residual_norm;
								double f_weight_sgd = (1 - f_direction_lerp) * (f_sgd_scale * f_steepest_descent_norm);
								_ASSERTE(f_weight_GN >= 0 && f_weight_sgd >= 0);
								double f_weight_denom = std::min(f_weight_GN, f_weight_sgd);
								if(!f_weight_denom) {
									if(f_weight_GN > f_weight_sgd) {
										f_weight_GN = 1;
										_ASSERTE(!f_weight_sgd);
									} else {
										f_weight_sgd = 1;
										_ASSERTE(!f_weight_GN);
									}
									f_weight_denom = 1;
								}
								printf("residual norm: %.4f, dogleg norm: %.4f, turning: %.2g (%.2f:%g GN:sgd)\n",
									f_residual_norm, f_dl_norm, f_direction_lerp,
									f_weight_GN / f_weight_denom, f_weight_sgd / f_weight_denom);
							} else {
								printf("residual norm: %.4f, cauchy norm: %.4f\n", f_residual_norm, f_dl_norm);
								// we did a simpler Cauchy step, no interpolation involved
							}
						}

						timer.Accum_DiffSample(m_f_dogleg_time);

						if(/*f_residual_norm <= f_min_dx_norm ||*/ f_dl_norm <= f_min_dl_norm) // the first part moved up
							break;
						// in case the error is low enough, quit (saves us recalculating the hessians)
					}
					// todo - move the dogleg logic into a function

					if(b_cholesky_result || b_enable_cauchy_step) {
						Eigen::VectorXd &v_saved_state = m_v_eta;
						// eta not needed anymore, can double for saving the state (also the dimension is correct)

						nonlinear_detail::CSolverOps_Base::Save_State(this->m_r_system, v_saved_state);
						m_last_relin_vertex_list = m_relin_vertex_list; // must copy
						const size_t n_total_updated_vertex_num_prev = m_n_total_updated_vertex_num;
						const size_t n_thresh_updated_vertex_num_prev = m_n_thresh_updated_vertex_num;
						// save optimizer state and stats

						PushValuesInGraphSystem(m_v_dx, m_relin_vertex_list, m_f_update_thresh);
						// push in graph system; may not actually relinearize anything if the update threshold is large

						timer.Accum_DiffSample(m_f_vert_upd_time);

						double f_error = this->f_Chi_Squared_Error_Denorm();
						if(this->m_b_verbose)
							printf("chi2: %f\n", f_error);
						// calculate and dump chi2

						timer.Accum_DiffSample(m_f_chi2_time);

						bool b_bad_step;
						if((b_bad_step = (f_error > f_last_error))) { // bad step
							fprintf(stderr, "warning: chi2 rising (delta = %g, decreasing to %g)\n",
								f_delta, f_delta / f_v);
							// verbose

							nonlinear_detail::CSolverOps_Base::Load_State(this->m_r_system, v_saved_state);
							m_relin_vertex_list.swap(m_last_relin_vertex_list); // swap ok
							// restore saved vertices

							-- m_n_vertex_update_num;
							m_n_total_updated_vertex_num = n_total_updated_vertex_num_prev;
							m_n_thresh_updated_vertex_num = n_thresh_updated_vertex_num_prev;
							// restore stats (!)

							/*if(n_max_fail_num > 0) {
								-- n_max_fail_num;
								n_max_iteration_num ++;
							}*/
							// increase the number of iterations, up to a certain limit // not needed in dogleg, no extra iteration will be taken (only a short iteration to solve for new step, based on already calculated eta and dx, this is very fast)

							if(f_delta / f_v > f_min_dl_norm) { // if can still make a step
								f_delta /= f_v;
								f_v *= 2;
								// we're fine even with cauchy, it can also reduce its step size
							} else {
								timer.Accum_DiffSample(m_f_dampingupd_time);
								if(this->m_b_verbose)
									printf("delta too low: %.4g, stopping iteration\n", f_delta / f_v);
								b_break_outer_loop = true; // break out of the outer loop as well
								break;
							}
							// update (decrease) the trust radius
						} else { // good step
#if defined(_DEBUG) || defined(__NONLINEAR_SOLVER_LAMBDA_DL_COLLECT_DX_STATS)
							size_t n_vertex_num = this->m_r_system.r_Vertex_Pool().n_Size();
							std::vector<double> L2_deltas(n_vertex_num), Linf_deltas(n_vertex_num);
							this->m_r_system.r_Vertex_Pool().For_Each(CCollectPerVertexDeltas<2>(m_v_dx, L2_deltas));
							this->m_r_system.r_Vertex_Pool().For_Each(CCollectPerVertexDeltas<Eigen::Infinity>(m_v_dx, Linf_deltas));
							std::sort(L2_deltas.begin(), L2_deltas.end());
							std::sort(Linf_deltas.begin(), Linf_deltas.end());
							if(!L2_deltas.empty()) {
								printf("debug: update L2 norm: min: %g, med: %g, 1k: %g, max: %g\n",
									L2_deltas.front(), L2_deltas[L2_deltas.size() / 2],
									L2_deltas[std::min(L2_deltas.size(), size_t(1000)) - 1],
									L2_deltas.back());
								_ASSERTE(!Linf_deltas.empty()); // should be the same size
								printf("debug: update Linf norm: min: %g, med: %g, 1k: %g, max: %g\n",
									Linf_deltas.front(), Linf_deltas[Linf_deltas.size() / 2],
									Linf_deltas[std::min(Linf_deltas.size(), size_t(1000)) - 1],
									Linf_deltas.back());
							}
							// incremental update profiling
#endif // _DEBUG || __NONLINEAR_SOLVER_LAMBDA_DL_COLLECT_DX_STATS

							/*double L0 = .5 * v_pure_residual.squaredNorm(); // r.h.s. (in this step, before updating) before being multiplied by J
							double Lh_dl = .5 * (v_pure_residual + J * v_dl).squaredNorm(); // J in this step, before updating
							double f_gain = (f_last_error - f_error) / (L0 - Lh_dl);*/
							/*dL = ||r||^2 - ||r + Jdx||^2 = r^Tr - (r + Jdx)^T(r + Jdx) =
							   = r^T(r - r - Jdx) - dx^TJ^T(r + Jdx) = -r^TJdx - dx^T(J^Tr + J^TJdx) =
							   = \eta^Tdx - dx^T(-\eta + \Lambda*dx) =	| can transpose the first term since the result is a scalar anyway
							   = dx^T\eta + dx^T(+\eta - \Lambda*dx) =
							   = dx^T(2\eta - \Lambda*dx)			| this is slightly inconvenient as we only have the upper half of lambda and it can be a relatively large matrix

							for pure GN, dx = \Lambda^-1\eta, so:

							dL = dx^T(2\eta-\Lambda*\Lambda^-1\eta) = dx^T\eta

							for pure sgd, dx = \eta/\beta, so:

							dL = dx^T(2\eta - \Lambda*\eta/\beta) =
							   = dx^T(2\eta + J^TJJ^Tr/\beta) =
							   = dx^T(2\eta) - J^Tr/\beta(J^TJJ^Tr/\beta)) =
							   = dx^T(2\eta) - J^TrJ^TJJ^Tr/(\beta^2) = ?*/
							// not easy to compute those

							//double f_gain = (f_last_error - f_error) / f_last_error; // returns gain < 1.0, decreases delta
							//double f_gain = (f_last_error - f_error) / std::max(1.0, f_error); // returns gain >= 1.0, does not decrease delta
							
							Eigen::VectorXd v_gain = -2.0 * v_orig_eta;
							m_lambda.SymmetricMultiply_Add(&v_gain(0), v_gain.rows(), &m_v_dx(0), m_v_dx.rows(), true);
							double f_gain = (f_last_error - f_error) / -m_v_dx.dot(v_gain);
							/*double f_gain1 = (f_last_error - f_error) / m_v_dx.dot(v_orig_eta); // this gets slightly off when sgd is being applied, otherwise it holds up to 14-15 decimals
							printf("debug: gain: %g, error: %g\n", f_gain, fabs(f_gain1 - f_gain));*/ // debug
							// some sort of optimization gain, based on chi2 (positive for decreased error)

							double f_prev_delta = f_delta;
							f_delta /= std::max(1.0 / 3, 1 - (2 * f_gain - 1) * (2 * f_gain - 1) * (2 * f_gain - 1));
							f_v = 2;
							// update the trust radius, make sure to not decrease

							if(!m_relin_vertex_list.empty()) { // todo - do some reasoning here, see when is it worth it; will need to remember a set of vertices that are not in marginals (that then translates to the edges and to matrix columns)
								//printf("debug: just updated the linearization point\n");
								//if(!m_b_force_incremental_margs) // this forces update at each step, but the values of the marginals would no longer be valid!
									this->m_marginals.DisableUpdate(); // linearization point just changed, all the marginals will change - need full recalc
							}

							f_last_error = f_error;
							// remember the error

							if(f_delta < f_min_dl_norm) {
								timer.Accum_DiffSample(m_f_dampingupd_time);
								if(this->m_b_verbose)
									printf("delta too low: %.4g, stopping iteration\n", f_delta);
								f_delta = (2 * f_delta >= f_min_dl_norm)? // might be more than 2x
									std::min(2 * f_delta, f_prev_delta) : f_prev_delta;
								break;
							}
							// in case the gain was too small, do not iterate any further
						}
						// trust region radius update

						timer.Accum_DiffSample(m_f_dampingupd_time);

						if(b_bad_step) {
							m_v_eta = v_orig_eta;
							m_v_dx = v_orig_dx;
							// restore the original vectors

							continue; // loop again, compute a dx with a smaller magnitude
						}
						if(m_relin_vertex_list.empty())
							break; // same as if f_residual_norm <= f_min_dx_norm failed
						// see if something updated at all
					}
					// update the system (in series; need to keep track of the relinearized vertex list)

					b_break_outer_loop = false; // continue in the outer loop normally
					break;
				}
				if(b_break_outer_loop)
					break;
				// a short loop for dogleg step adjustment (linear solving is outside, only the update is inside)
			}
		}
		// optimize the system

		if(m_b_keep_step_size)
			m_f_delta = std::max(m_f_initial_delta, f_delta);
		// remember step size

#if 0 // not all edges have reprojection error function; this effectively breaks all non-BA solver implementations
		if(this->m_b_verbose)
			Dump_ReprojectionErrors();
		// debug
#endif // 0

		if(this->m_t_marginals_config.b_calculate) {
			if(!m_relin_vertex_list.empty()) {
				size_t n_relin_vertex_num = m_relin_vertex_list.size();
				UpdateLambda_CalculateOmega(omega, timer);
				// update lambda and get a new omega

				if(b_use_schur && !m_b_all_batch)
					Update_Schur(p_schur_order, n_cut, n_size, omega, lambda_perm_structure, newU, n_relin_vertex_num);
				// update the Schur complement member variables

				timer.Accum_DiffSample(m_f_schur_update_time);
			}
			// need to update first

			if(b_use_schur) {
				if(m_b_all_batch)
					Redo_Schur(p_schur_order, n_cut, n_size, lambda_perm_structure, newU);
				// need schur; could also get it from the schur solver but this isnt implemented like that

				const CUberBlockMatrix &minus_Dinv = m_minus_D_inv, &SC = m_SchurCompl, &U = newU;

				CUberBlockMatrix S, SC_perm;
				CMatrixOrdering SC_mord;
				SC_mord.p_BlockOrdering(SC, true);
				SC.Permute_UpperTriangular_To(SC_perm, SC_mord.p_Get_InverseOrdering(),
					SC_mord.n_Ordering_Size(), true);
				bool b_Chol_succeded = S.CholeskyOf_FBS<_TySchurBlockSizes>(SC_perm);
				if(!b_Chol_succeded) {
					fprintf(stderr, "error: S.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(SC_perm) "
						"failed; trying modified Cholesky\n");
					// can fail once

					{
						CUberBlockMatrix SC_perm_copy = SC_perm;
						SC_perm.Swap(SC_perm_copy);
					}
					for(size_t i = 0, n = SC_perm.n_BlockColumn_Num(); i < n; ++ i) {
						size_t m = SC_perm.n_BlockColumn_Block_Num(i);
						_ASSERTE(SC_perm.n_Block_Row(i, m - 1) == i); // make sure the last block is always diagonal
						//CUberBlockMatrix::_TyMatrixXdRef t_block = SC_perm.t_Block_AtColumn(i, m - 1);
						SC_perm.t_Block_AtColumn(i, m - 1).diagonal().array() += 1.0;
					}
					// increase the diagonal a bit (a mock-up modified chol; the marginals will be off but we can do the timing)
				}
				if(!b_Chol_succeded && !S.CholeskyOf_FBS<_TySchurBlockSizes>(SC_perm))
					throw std::runtime_error("fatal error: S.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(SC_perm) failed");
				else {
					if(!b_Chol_succeded) {
						fprintf(stderr, "debug: modified Cholesky passed\n");
						fflush(stderr);
					}
					// debug

					timer.Accum_DiffSample(m_f_extra_chol_time);

					CUberBlockMatrix minus_U_Dinv;
					minus_U_Dinv.ProductOf_FBS<_TyUBlockSizes, _TyDBlockSizes>(U, minus_Dinv);
					// todo - see if we can recover this from the linear solver (in case we are relinearizing just above, will throw it away though)

					double f_UDinv_time = 0;
					timer.Accum_DiffSample(f_UDinv_time);
					m_f_UDinv_time += f_UDinv_time;

					CUberBlockMatrix margs_cams, margs_all;
					m_margs.Schur_Marginals(margs_cams, true, margs_all, S, SC_mord, minus_Dinv, minus_U_Dinv, true);
					if(m_margs.b_Verbose())
						printf("\tcalculating UDinv took %f sec\n", f_UDinv_time); // add to the verbose

					margs_cams.ExtendTo(m_lambda.n_Row_Num(), m_lambda.n_Column_Num());
					margs_all.ExtendTopLeftTo(m_lambda.n_Row_Num(), m_lambda.n_Column_Num());

					/*margs_cams.Rasterize("scmargs_00_cams.tga");
					margs_all.Rasterize("scmargs_01_pts.tga");*/

					margs_cams.AddTo(margs_all); // no need for FBS, there is no overlap, will only copy the blocks

					//margs_all.Rasterize("scmargs_02_all.tga");

					CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
					margs_all.Permute_UpperTriangular_To(r_m, &m_schur_ordering[0],
						m_schur_ordering.size(), false); // no share! the original will be deleted
					this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
					// take care of having the correct permutation there

					//r_m.Rasterize("scmargs_03_unperm.tga");

					this->m_marginals.EnableUpdate();
					// now the marginals are current and can be updated until the linearization point is changed again
				}
			} else {
				CUberBlockMatrix R;
				CMatrixOrdering lambda_AMD;
				if((this->m_marginals.b_CanUpdate() && (this->m_t_marginals_config.n_incremental_policy &
				   mpart_LastColumn) == mpart_LastColumn) || // can tell for sure if incremental is going to be used
				   (this->m_t_marginals_config.n_relinearize_policy & mpart_LastColumn) == mpart_LastColumn) { // never know if we fallback to batch, though
					CLastElementOrderingConstraint leoc;
					lambda_AMD.p_BlockOrdering(m_lambda, leoc.p_Get(m_lambda.n_BlockColumn_Num()),
						m_lambda.n_BlockColumn_Num(), true); // constrain the last column to be the last column (a quick fix) // todo - handle this properly, will be unable to constrain like this in fast R (well, actually ...) // todo - see what is this doing to the speed
				} else
					lambda_AMD.p_BlockOrdering(m_lambda, true); // unconstrained; the last column may be anywhere (could reuse R from the linear solver here - relevant also in batch (e.g. on venice))
				const size_t *p_order = lambda_AMD.p_Get_InverseOrdering();

				double f_lambda_AMD = 0;
				timer.Accum_DiffSample(f_lambda_AMD);

				CUberBlockMatrix lambda_perm;
				m_lambda.Permute_UpperTriangular_To(lambda_perm, p_order, lambda_AMD.n_Ordering_Size(), true);

				double f_lambda_perm = 0;
				timer.Accum_DiffSample(f_lambda_perm);

				if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm))
					throw std::runtime_error("fatal error: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed");
				// note that now the marginals are calculated with ordering: need to count with that, otherwise those are useless!

				double f_lambda_Chol = 0;
				timer.Accum_DiffSample(f_lambda_Chol);
				m_margs.Add_LambdaChol_Time(f_lambda_AMD, f_lambda_perm, f_lambda_Chol);
				m_f_marginals_time += f_lambda_AMD + f_lambda_perm + f_lambda_Chol; // keep these stats!
				// keep stats on the extra cholesky

				// note - this is flawed if the linearization points of some vertices are allowed to change;
				//        can't increment precisely, would have to keep all the info and collect an explicit
				//        omega (could do simple subtraction and then drop all zeros, this eats a ton of memory
				//        anyway)

				// note - running without -ptr typically gives a few chances to use the incremental update
				//        (the linpoint is left unchanged more often)

				size_t n_add_edge_num = this->m_r_system.r_Edge_Pool().n_Size() - this->m_marginals.n_Edge_Num();
				bool b_incremental = this->m_marginals.b_CanUpdate() && (m_b_force_incremental_margs ||
					CMarginals::b_PreferIncremental(this->m_r_system, this->m_marginals.r_SparseMatrix(),
					m_lambda, R, lambda_AMD, this->m_marginals.n_Edge_Num(),
					this->m_t_marginals_config.n_incremental_policy));

				if(b_incremental) { // otherwise just update what we have
					static bool b_warned = false;
					if(!b_warned) {
						b_warned = true;
						fprintf(stderr, "warning: using ICRA-15 style incremental marginal "
							"covariances: expect to run out of memory\n");
					}
					// debug - see if this branch is really being used

					CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
					if(!CMarginals::Update_BlockDiagonalMarginals_FBS<false>(this->m_r_system, r_m, m_lambda,
					   R, lambda_AMD, this->m_marginals.n_Edge_Num(), this->m_t_marginals_config.n_incremental_policy)) {
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
				if(!b_incremental) {
					CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
					m_margs.Recursive_Marginals(r_m, R, lambda_AMD);
					this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
					// get recursive margs (already unpermuted)
				}

				//rfmargs_all.Rasterize("scmargs_04_recformula.tga");

				this->m_marginals.EnableUpdate();
				// now the marginals are current and can be updated until the linearization point is changed again
			}

			this->m_marginals.Set_Edge_Num(this->m_r_system.r_Edge_Pool().n_Size());
			// now all those edges are in the marginals

			timer.Accum_DiffSample(m_f_marginals_time);

			if(0) { // do we want to compare to recursive formula?
				CUberBlockMatrix R;
				CMatrixOrdering lambda_AMD;
				if(m_margs.Get_CholeskyLambda(R, lambda_AMD, m_lambda)) {
					double f_update_cost = CMarginals::f_MarginalsUpdate_DenseBytes(this->m_r_system,
						this->m_marginals.r_SparseMatrix(), m_lambda, R, lambda_AMD, this->m_marginals.n_Edge_Num(),
						this->m_t_marginals_config.n_incremental_policy);
					printf("debug: updating marginals incrementally would cost %.2f MB\n",
						f_update_cost / 1048576.0);
					// verbose

					CUberBlockMatrix /*rfmargs_cams, rfmargs_landmarks,*/ rfmargs_all;
					m_margs.Recursive_Marginals(/*rfmargs_cams, rfmargs_landmarks,*/ rfmargs_all, R, lambda_AMD);
					// get recursive margs (already unpermuted)

					//rfmargs_all.Rasterize("scmargs_04_recformula.tga");

					if(b_use_schur) {
						CUberBlockMatrix rfmargs_all_U, rfmargs_all_D;
						rfmargs_all_U.TriangularViewOf(rfmargs_all, true, true); // only upper + diagonal
						rfmargs_all_D.TriangularViewOf(rfmargs_all_U, false, true); // only diagonal
						double f_margs_norm = rfmargs_all_D.f_Norm();
						CUberBlockMatrix scmargs_all_U, scmargs_all_D;
						scmargs_all_U.TriangularViewOf(this->m_marginals.r_SparseMatrix(), true, true); // only upper + diagonal
						scmargs_all_D.TriangularViewOf(scmargs_all_U, false, true); // only diagonal
						if(!scmargs_all_D.AddTo(rfmargs_all_D, -1))
							fprintf(stderr, "error: failed to take margs difference (bad ordering?)\n");
						double f_err_all = rfmargs_all_D.f_Norm();
						printf("debug: the marginals error: %g abs, %g rel\n",
							f_err_all, f_err_all / std::max(1.0, f_margs_norm));
						// compare

						//rfmargs_all_D.Rasterize("scmargs_05_diff.tga");
					}
				} else
					fprintf(stderr, "error: failed to recover recursive marginals (not pos def)\n");

				timer.Accum_DiffSample(m_f_marginals_test_time);
				// to make sure that the timing is correct
			}
			if(0) { // do we want to compare to backsubstitution?
				try {
					CUberBlockMatrix R;
					CMatrixOrdering lambda_AMD;
					if(m_margs.Get_CholeskyLambda(R, lambda_AMD, m_lambda)) {
						CUberBlockMatrix bsmargs_D;
						m_lambda.CopyLayoutTo(bsmargs_D);
						#pragma omp parallel
						{
							Eigen::MatrixXd dense_col; // reuse storage between iterations, storage is for each thread
							size_t n = m_lambda.n_BlockColumn_Num();
							_ASSERTE(n <= INT_MAX);
							int _n = int(n);
							#pragma omp for // work-share the loop
							for(int i = 0; i < _n; ++ i) {
								const size_t n_first_col = m_lambda.n_BlockColumn_Base(i); // needed by the parallel loop
								const size_t n_col_width = m_lambda.n_BlockColumn_Column_Num(i);
								//const size_t n_last_col = n_first_col + n_col_width;
								dense_col.resize(m_lambda.n_Column_Num(), n_col_width);
								// alloc space for a single dense column

								CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<_TyLambdaMatrixBlockSizes>(dense_col,
									R, n_first_col, /*n_last_col,*/ lambda_AMD.p_Get_InverseOrdering(),
									lambda_AMD.n_Ordering_Size(), lambda_AMD.n_Ordering_Size());
								// calculate one column of dense marginals
								// note - could actually do with only the part tall enough to hold
								// the upper triangle and the diagonal block (not easy with the perm)

								#pragma omp critical // only one thread at a time
								{
									bsmargs_D.t_GetBlock_Log(i, i, n_col_width, n_col_width,
										true, false) = dense_col.middleRows(n_first_col, n_col_width);
									// put it at the diagonal of the margs matrix
								}
								// we don't care much about the order of block allocations in the matrix
								// as we don't plan to do many operations on it

								//n_first_col = n_last_col; // !!
							}
						}
						// calculate a diagonal matrix using recursive formula (todo - make a comfy
						// function with matrix parts support for debugging)

						double f_margs_norm = bsmargs_D.f_Norm();
						CUberBlockMatrix scmargs_all_U, scmargs_all_D;
						scmargs_all_U.TriangularViewOf(this->m_marginals.r_SparseMatrix(), true, true); // only upper + diagonal
						scmargs_all_D.TriangularViewOf(scmargs_all_U, false, true); // only diagonal
						if(!scmargs_all_D.AddTo(bsmargs_D, -1))
							fprintf(stderr, "error: failed to take margs difference (bad ordering?)\n");
						double f_err_all = bsmargs_D.f_Norm();
						printf("debug: the marginals error: %g abs, %g rel (compared to back-substitution)\n",
							f_err_all, f_err_all / std::max(1.0, f_margs_norm));
						// compare

						fflush(stdout); // this takes a while, make sure the outputs are written
					} else
						fprintf(stderr, "error: failed to recover back-substitution marginals (not pos def)\n");
				} catch(std::bad_alloc&) {
					fprintf(stderr, "warning: unable to verify marginals (bad_alloc)\n");
				}
				// the new detailed comparison code

				timer.Accum_DiffSample(m_f_marginals_test_time);
				// to make sure that the timing is correct
			}

			if(m_f_incmarginals_time > 0) {
				printf("debug: the marginals time: %g xChol, %g UD^-1, %g + %g prod, %g test\n",
					m_f_extra_chol_time, m_f_UDinv_time, m_f_marginals_time, m_f_incmarginals_time,
					m_f_marginals_test_time);
			} else {
				printf("debug: the marginals time: %g xChol, %g UD^-1, %g prod, %g test\n",
					m_f_extra_chol_time, m_f_UDinv_time, m_f_marginals_time, m_f_marginals_test_time);
			}
			fflush(stdout);
			// debug - see the timing too

			//timer.Accum_DiffSample(m_f_marginals_test_time);
		}

		//printf("debug: process memory %.3f MB\n", CProcessMemInfo::n_MemoryUsage() / 1048576.0);
		// see (more or less) how much memory it needs
	}

protected:
	std::vector<Eigen::VectorXd> m_damping_stack;

	void Debug_PushDamping(double f_lambda)
	{
		Eigen::VectorXd v_original(m_lambda.n_Column_Num());
		size_t n_dest = 0;
		for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			CUberBlockMatrix::_TyMatrixXdRef t_diag_block =
				m_lambda.t_Block_AtColumn(i, m_lambda.n_BlockColumn_Block_Num(i) - 1);
			_ASSERTE(m_lambda.n_Block_Row(i, m_lambda.n_BlockColumn_Block_Num(i) - 1) == i); // make sure that's the diagonal block
			size_t n_cols = t_diag_block.cols();
			v_original.segment(n_dest, n_cols) = t_diag_block.diagonal(); // store the original diagonal value
			t_diag_block.diagonal().array() += f_lambda; // add damping
			n_dest += n_cols;
		}
		_ASSERTE(v_original.cols() == 1 && n_dest == v_original.rows());
		m_damping_stack.push_back(v_original); // !!
	}

	void Debug_PopDamping()
	{
		_ASSERTE(!m_damping_stack.empty());
		Eigen::VectorXd &r_v_original = m_damping_stack.back();
		_ASSERTE(r_v_original.cols() == 1 && m_lambda.n_Column_Num() == r_v_original.rows());
		size_t n_src = 0;
		for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			CUberBlockMatrix::_TyMatrixXdRef t_diag_block =
				m_lambda.t_Block_AtColumn(i, m_lambda.n_BlockColumn_Block_Num(i) - 1);
			_ASSERTE(m_lambda.n_Block_Row(i, m_lambda.n_BlockColumn_Block_Num(i) - 1) == i); // make sure that's the diagonal block
			size_t n_cols = t_diag_block.cols();
			t_diag_block.diagonal() = r_v_original.segment(n_src, n_cols);
			n_src += n_cols;
		}
		_ASSERTE(r_v_original.cols() == 1 && n_src == r_v_original.rows());
		m_damping_stack.erase(m_damping_stack.end() - 1);
	}

	/**
	 *	@brief function object that updates states of the sufficiently changing vertices
	 */
	template <int n_L_norm = 2> // or Eigen::Infinity
	class CCollectPerVertexDeltas {
	protected:
		const Eigen::VectorXd &m_r_dx; /**< @brief vector of differences */
		std::vector<double> &m_r_vertex_delta_list; /**< @brief a list of per vertex deltas */
		size_t m_n_vertex_id; /**< @brief vertex id counter (the vertices themselves do not know their id) */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_dx is reference to the vector of differences
		 *	@param[in] r_vertex_delta_list is reference to a vector to be filled
		 *		with the specified norms of the deltas of all the vertices in the system
		 */
		inline CCollectPerVertexDeltas(const Eigen::VectorXd &r_dx,
			std::vector<double> &r_vertex_delta_list)
			:m_r_dx(r_dx), m_r_vertex_delta_list(r_vertex_delta_list),
			m_n_vertex_id(0)
		{}

		/**
		 *	@brief updates vertex state
		 *	@tparam _TyVertex is type of vertex
		 *	@param[in,out] r_t_vertex is reference to vertex, being updated
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex) // throw(std::bad_alloc)
		{
			double f_change = m_r_dx.template segment<_TyVertex::n_dimension>(r_t_vertex.n_Order()).template lpNorm<n_L_norm>();
			m_r_vertex_delta_list[m_n_vertex_id] = f_change;
			++ m_n_vertex_id;
		}
	};

	/**
	 *	@brief function object that updates states of the sufficiently changing vertices
	 */
	class CUpdateEstimates_MarkChanges {
	protected:
		const Eigen::VectorXd &m_r_dx; /**< @brief vector of differences */
		const double m_f_update_threshold2; /**< @brief squared update norm threshold */
		std::vector<size_t> &m_r_relin_vertex_list; /**< @brief a list of vertices being relinearized */
		size_t m_n_vertex_id; /**< @brief vertex id counter (the vertices themselves do not know their id) */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_dx is reference to the vector of differences
		 *	@param[in] r_relin_vertex_list is reference to a vector to be filled
		 *		with the ids of vertices which are changing
		 *	@param[in] f_update_threshold is update norm threshold
		 */
		inline CUpdateEstimates_MarkChanges(const Eigen::VectorXd &r_dx,
			std::vector<size_t> &r_relin_vertex_list, double f_update_threshold)
			:m_r_dx(r_dx), m_r_relin_vertex_list(r_relin_vertex_list),
			m_f_update_threshold2(f_update_threshold * f_update_threshold),
			m_n_vertex_id(0)
		{}

		/**
		 *	@brief updates vertex state
		 *	@tparam _TyVertex is type of vertex
		 *	@param[in,out] r_t_vertex is reference to vertex, being updated
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex) // throw(std::bad_alloc)
		{
			double f_change = m_r_dx.segment<_TyVertex::n_dimension>(r_t_vertex.n_Order()).squaredNorm();
			if(f_change >= m_f_update_threshold2) { // todo - also implement rank-limited updates to get constant time
				r_t_vertex.Operator_Plus(m_r_dx); // todo - idea - update all the vertices and instead detect the changes in the *gradient*. more expensive to compute but should get a better quality solution while actually less of the factorization is changing
				m_r_relin_vertex_list.push_back(m_n_vertex_id);
			}

			++ m_n_vertex_id;
		}
	};

	/**
	 *	@brief function object that collects vertex indices for all edges
	 */
	class CCollectVertexIds {
	protected:
		std::vector<size_t> &m_r_relin_vertex_list; /**< @brief list of ids of vertices affected by linearization point change */
		const size_t m_n_optimized_vertex_num; /**< @brief number of the (optimized) vertices in the system */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_relin_vertex_list is reference to a vector with ids of vertices
		 *		where the linearization point has changed
		 *	@param[in] n_optimized_vertex_num is number of the (optimized) vertices in the system
		 */
		inline CCollectVertexIds(std::vector<size_t> &r_relin_vertex_list,
			size_t n_optimized_vertex_num)
			:m_r_relin_vertex_list(r_relin_vertex_list), m_n_optimized_vertex_num(n_optimized_vertex_num)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is an edge type
		 *	@param[in] r_edge is edge to have its vertices recorded
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_edge) // throw(std::bad_alloc)
		{
			bool b_is_affected = false;
			for(size_t i = 0, n = r_edge.n_Vertex_Num(); i < n; ++ i) {
				size_t n_vertex = r_edge.n_Vertex_Id(i); // t_odo - look the vertex up and see if it is constant or not (actually no need, if it is constant, it will simply not be in the list)
				if(n_vertex < m_n_optimized_vertex_num)
					m_r_relin_vertex_list.push_back(n_vertex);
			}
		}
	};

	/**
	 *	@brief function object that calls omega hessian block allocation and evaluation for all edges
	 */
	class CUpdateOmega {
	protected:
		CUberBlockMatrix &m_r_omega; /**< @brief reference to the omega matrix (out) */
		//size_t m_n_min_elem_order; /**< @brief minimal order of vertex to be filled in omega (in elements) */
		const std::vector<size_t> &m_r_relin_vertex_list; /**< @brief list of ids of vertices affected by linearization point change */
		std::vector<bool> m_relin_vertex_list;
		const bool m_b_update_hessians; /**< @brief hessian update flag */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_omega is reference to the omega matrix, new blocks are to be added
		 *		to it (with arithmetic additive semantics)
		 *	@param[in] r_relin_vertex_list is reference to a vector with ids of vertices
		 *		where the linearization point has changed
		 *	@param[in] b_update_hessians is hessian update flag
		 */
		// *	@param[in] n_min_elem_order is minimal order of vertex to be filled in omega (in elements)
		inline CUpdateOmega(CUberBlockMatrix &r_omega, const std::vector<size_t> &r_relin_vertex_list,
			bool b_update_hessians/*, size_t n_min_elem_order*/)
			:m_r_omega(r_omega)/*, m_n_min_elem_order(n_min_elem_order)*/,
			m_r_relin_vertex_list(r_relin_vertex_list), m_b_update_hessians(b_update_hessians)
		{
			if(!r_relin_vertex_list.empty()) {
				m_relin_vertex_list.resize(r_relin_vertex_list.back() + 1, false);
				for(size_t i = 0, n = r_relin_vertex_list.size(); i < n; ++ i)
					m_relin_vertex_list[r_relin_vertex_list[i]].flip(); // flip() is fast and the vector is unique so this is ok
			}
		}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is an edge type
		 *	@param[in] r_edge is edge to have its hessian blocks added to omega
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_edge) // throw(std::bad_alloc)
		{
			bool b_is_affected = false;
			for(size_t i = 0, n = r_edge.n_Vertex_Num(); i < n; ++ i) {
				size_t n_vertex = r_edge.n_Vertex_Id(i); // todo - look the vertex up and see if it is constant or not (actually no need, if it is constant, it will simply not be in the list)

				/*std::vector<size_t>::const_iterator p_vert_it;
				if((p_vert_it = std::lower_bound(m_r_relin_vertex_list.begin(),
				   m_r_relin_vertex_list.end(), n_vertex)) != m_r_relin_vertex_list.end() &&
				   *p_vert_it == n_vertex) {
					b_is_affected = true;
					break;
				}*/
				// use binary search

				if(n_vertex < m_relin_vertex_list.size() && m_relin_vertex_list[n_vertex]) {
					b_is_affected = true;
					break;
				}
				// use bit array
			}
			// see if the edge is affected (as long as PushValuesInGraphSystem() runs
			// in series, the vertices are ordered and it is ok to use std::lower_bound())

			// note that this might need to happen twice, once to calculate omega before
			// and once after linearization point change, in order to get the fluid
			// relinearization update. might be worthwhile to remember the set of affected
			// edges / calculate it outside.

			// note that if there are relatively many vertices in m_r_relin_vertex_list,
			// it might be worthwhile to put them in a bool vector and test against that
			// (100k vertices take 12.5 kB) // todo - figure out where is the ratio and implement the second branch that uses bool vector

			// lookup takes O(2E * log(V_upd)) if the vertex ids are ordered
			// (if not, it is efficient to sort them in O(V_upd * log(V_upd))
			// whenever O(2E * V_upd) > O((V_upd + 2E) * log(V_upd))).
			// bool vec takes O(k_1 * V + k_2 * (V_upd + 2E)).

			if(!b_is_affected)
				return;
			// only the affected edges

			if(m_b_update_hessians) {
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
				r_edge.Calculate_Hessians_v2();
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
				r_edge.Calculate_Hessians();
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
			}
			// update the hessians?

			r_edge.Calculate_Omega(m_r_omega, 0/*m_n_min_elem_order*/);
			// add the contribution to omega

			// note - do not collect indices of edge vertices; it would take O(n) to collect,
			// O(n log n) to sort and further O(n) to prune the duplicates (or O(n log n) to
			// build a set; if needed, it can be always picked up from omega in O(n)
		}
	};

	/**
	 *	@brief updates the vertices
	 *
	 *	@param[in] r_dx is reference to the vector of differences
	 *	@param[in] r_relin_vertex_list is reference to a vector to be filled
	 *		with the ids of vertices which are changing
	 *	@param[in] f_update_threshold is update norm threshold
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void PushValuesInGraphSystem(const Eigen::VectorXd &r_v_dx,
		std::vector<size_t> &r_relin_vertex_list, double f_update_threshold) // throw(std::bad_alloc)
	{
		++ m_n_vertex_update_num;
		m_n_total_updated_vertex_num += this->m_r_system.r_Vertex_Pool().n_Size();
		m_n_thresh_updated_vertex_num -= r_relin_vertex_list.size(); // underflow ok in here
		// stats

#ifdef _DEBUG
		_TyLambdaOps::b_DetectNaNs(r_v_dx, true, "dx");
#endif // _DEBUG
		this->m_r_system.r_Vertex_Pool().For_Each(CUpdateEstimates_MarkChanges(r_v_dx,
			r_relin_vertex_list, f_update_threshold));
		// todo - can do this in parallel with explicit reduction of the relin vectors at the end
		// could do that by spawning the threads manually and then statically dividing the vertex
		// pool among them, or by making some object that remembers thread ids and manages the contexts

		m_n_thresh_updated_vertex_num += r_relin_vertex_list.size(); // add delta
		if(r_relin_vertex_list.size() == this->m_r_system.r_Vertex_Pool().n_Size())
			++ m_n_full_rank_updare_num;
		// stats
	}

	/**
	 *	@brief updates the omega matrix
	 *
	 *	@param[in,out] r_omega is filled with the omega matrix (using addition)
	 *	@param[in] n_update_edge_num is the number of edges that are potentially afected by the update
	 *	@param[in] r_relin_vertex_list is reference to a vector with ids of vertices
	 *		where the linearization point has changed
	 *	@param[in] b_update_hessians is hessian update flag
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Update_Omega(CUberBlockMatrix &r_omega, size_t n_update_edge_num,
		const std::vector<size_t> &r_relin_vertex_list, bool b_update_hessians = false) // throw(std::bad_alloc)
	{
		this->m_r_system.r_Edge_Pool().For_Each(0, n_update_edge_num, // only the pre-existing edges
			CUpdateOmega(r_omega, r_relin_vertex_list, b_update_hessians));
		// this cannot run in parallel, except maybe with several omegas and their addition afterwards
	}

	/**
	 *	@brief increments the omega matrix with new edges
	 *
	 *	@param[in,out] r_omega is filled with the omega matrix (using addition)
	 *	@param[in] n_update_edge_num is the number of edges that are potentially afected by the update
	 *	@param[in] r_relin_vertex_list is reference to a vector with ids of vertices
	 *		where the linearization point has changed
	 *	@param[in] b_update_hessians is hessian update flag
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Increment_Omega(CUberBlockMatrix &r_omega,
		size_t n_first_increment_edge, bool b_update_hessians = false) // throw(std::bad_alloc)
	{
		_ASSERTE(!b_update_hessians);
		this->m_r_system.r_Edge_Pool().For_Each(n_first_increment_edge,
			this->m_r_system.r_Edge_Pool().n_Size(), fL_util::CCalculateOmega(r_omega, 0));
	}

	class CSumSteepestGradientDescentScaleDenom {
	protected:
		double &m_r_f_denom;
		const Eigen::VectorXd &m_r_v_g;

	public:
		inline CSumSteepestGradientDescentScaleDenom(double &r_f_denom, const Eigen::VectorXd &r_v_g)
			:m_r_f_denom(r_f_denom), m_r_v_g(r_v_g)
		{}

		template <class CEdge>
		inline void operator ()(/*const*/ CEdge &r_edge) // not const; this modifies the Hessians / rhs (even though it is likely already calculated)
		{
			m_r_f_denom += r_edge.f_Calculate_Steepest_Descent_Scale(m_r_v_g);//f_Calculate_Hessians_and_Steepest_Descent_Scale(); // this calculates the hessians and the r.h.s as well!
		}
	};

	class CSumSquaredReprojectionErrors {
	protected:
		Eigen::Vector5d &m_r_v_error_sum;

	public:
		inline CSumSquaredReprojectionErrors(Eigen::Vector5d &r_v_error_sum)
			:m_r_v_error_sum(r_v_error_sum)
		{}

		template <class CEdge>
		inline typename CEnableIf<CEdge::b_is_robust_edge>::T operator ()(const CEdge &r_edge)
		{
			double f_error = r_edge.f_Reprojection_Error();
			Eigen::Vector1d r; r(0) = f_error; // can't directly initialize, the constructor is ambiguous
			double f_weight = r_edge.f_RobustWeight(r);
			m_r_v_error_sum(0) += f_error;
			m_r_v_error_sum(1) += f_error * f_error;
			m_r_v_error_sum(2) += f_error * f_weight;
			m_r_v_error_sum(3) += f_error * f_error * f_weight;
			m_r_v_error_sum(4) += f_weight;
		}

		template <class CEdge>
		inline typename CEnableIf<!CEdge::b_is_robust_edge>::T operator ()(const CEdge &r_edge)
		{
			double f_error = r_edge.f_Reprojection_Error();
			//double f_weight = 1;
			m_r_v_error_sum(0) += f_error;
			m_r_v_error_sum(1) += f_error * f_error;
			m_r_v_error_sum(2) += f_error;
			m_r_v_error_sum(3) += f_error * f_error;
			m_r_v_error_sum(4) += 1.0;
		}
	};

#if 0 // not all edges have reprojection error function; this effectively breaks all non-BA solver implementations
	/**
	 *	@brief calculates reprojection errors for precision evaluation
	 *	@return Returns the vector of average reprojection error, RMSE, robustified
	 *		average, robustified RMSE and a sum of robust weights.
	 *	@note This requires the edges to implement <tt>f_Reprojection_Error()</tt>.
	 */
	Eigen::Vector5d v_ReprojectionErrors() const
	{
		Eigen::Vector5d v_sums = Eigen::Vector5d::Zero();
		this->m_r_system.r_Edge_Pool().For_Each(CSumSquaredReprojectionErrors(v_sums));
		v_sums.head<2>() /= this->m_r_system.r_Edge_Pool().n_Size(); // mean
		v_sums.segment(2, 2) /= v_sums(4); // weighted mean
		v_sums(1) = sqrt(v_sums(1)); // root
		v_sums(3) = sqrt(v_sums(3)); // root
		return v_sums;
	}

public:
	void Dump_ReprojectionErrors() const
	{
		Eigen::Vector5d v_errors = v_ReprojectionErrors();
		printf("reprojection error: avg %f, RMSE %f, r-avg %f, r-RMSE %f, r-weight sum %f / "
			PRIsize "\n", v_errors(0), v_errors(1), v_errors(2), v_errors(3), v_errors(4),
			this->m_r_system.r_Edge_Pool().n_Size());
		// calculate and dump reprojection error
	}
protected:
#endif // 0

	/**
	 *	@brief updates lambda and calculates omega
	 *
	 *	@param[out] r_omega is filled with the omega matrix
	 *	@param[in,out] timer is timer sampler for collecting statistics
	 *
	 *	@note This relies on the linearization point and the hessians not being tampered with.
	 *	@note This function throws std::bad_alloc.
	 *
	 *	@note It might not be actually needed to keep lambda
	 *		at all. Maybe we could do with omega only.
	 */
	void UpdateLambda_CalculateOmega(CUberBlockMatrix &r_omega, CTimerSampler &timer) // throw(std::bad_alloc)
	{
#ifdef _DEBUG
		CUberBlockMatrix lambda_diff = m_lambda;
		// debug - remember lambda before
#endif // _DEBUG

		_ASSERTE(!m_n_verts_in_lambda == !m_n_edges_in_lambda); // a bit awkward
		const bool b_prev_lambda_empty = !m_n_edges_in_lambda;

		if(!b_prev_lambda_empty && n_dummy_param)
			Debug_PopDamping();

		if(!m_b_all_batch) { // omega only needed by the incremental pipeline
			if(!b_prev_lambda_empty) {
				if(r_omega.n_BlockColumn_Num() == m_lambda.n_BlockColumn_Num()) {
					_ASSERTE(r_omega.n_BlockRow_Num() == r_omega.n_BlockColumn_Num()); // block square
					_ASSERTE(r_omega.n_Row_Num() == r_omega.n_Column_Num()); // square
					_ASSERTE(r_omega.n_Column_Num() == m_lambda.n_Column_Num()); // and the same size

					r_omega.SetZero();
					// has the right size, just remove any nnz blocks
				} else {
					{
						std::vector<size_t> row_col_cumsums(this->m_r_system.r_Vertex_Pool().n_Size());
						this->m_r_system.r_Vertex_Pool().For_Each(typename _TyLambdaOps::CCopyVariableDims(row_col_cumsums)); // serial, using type info, 0.000393 sec (on windows)
						// this initializes to the size of new lambda (including the new vertices)

						CUberBlockMatrix empty(row_col_cumsums.begin(), row_col_cumsums.end(),
							row_col_cumsums.begin(), row_col_cumsums.end());
						r_omega.Swap(empty);
					}
					// initialize the block rows and columns of the matrix, so that the order in which the edges are inserted
					// does not cause row reindexing (this is usually not a problem in SLAM, but it gets worse in BA)

					// todo - make this a function in _TyLambdaOps, make sure it is in v1 as well, check if it at least builds using v1
				}
				// build block layout for omega so that it can be permuted the same way that lambda can
				// note - alternatively, could reuse the permuting function from fL_util
			}
			// this only needs to happen if the previous lambda was not empty, otherwise
			// omega will be overwritten by lambda below and this would be wasted effort

			if(!b_prev_lambda_empty && !m_relin_vertex_list.empty()) {
				Update_Omega(r_omega, m_n_edges_in_lambda, m_relin_vertex_list); // for all edges 0 .. m_n_edges_in_lambda
				if(r_omega.n_Storage_Size() > 50 * 3 * 3)
					r_omega.Scale_FBS_Parallel<_TyLambdaMatrixBlockSizes>(-1); // omega = new - old; could fuse it in Update_Omega() maybe
				else
					r_omega.Scale_FBS<_TyLambdaMatrixBlockSizes>(-1); // probably too small for any parallelization benefit
			}
			// calculate "old" omega if not zero

			timer.Accum_DiffSample(m_f_omega_time);
		}

		_TyLambdaOps::Extend_Lambda(this->m_r_system, m_reduction_plan,
			m_lambda, m_n_verts_in_lambda, m_n_edges_in_lambda); // recalculated all the jacobians inside Extend_Lambda()
		// todo - since we know which edges to update, it is wasteful to recalculate all the hessians here; dont do it

		if(m_relin_vertex_list.empty()) { // no change in the linearization point
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda,
				m_n_verts_in_lambda, m_n_edges_in_lambda); // calculate only for new edges // t_odo - but how to mark affected vertices?
		} else
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // calculate for entire system

		if(n_dummy_param)
			Debug_PushDamping(n_dummy_param);
		// emulate levenberg

		timer.Accum_DiffSample(m_f_lambda_time);

		if(!m_b_all_batch) { // omega only needed by the incremental pipeline
			if(b_prev_lambda_empty) {
				r_omega = m_lambda;
				// in case this is the first iteration, omega is the same as
				// lambda (and we did not bother tracking the changes)
			} else {
				// omega is not the same as lambda

				if(!m_relin_vertex_list.empty())
					Update_Omega(r_omega, m_n_edges_in_lambda, m_relin_vertex_list); // for all edges 0 .. m_n_edges_in_lambda
				// add the new edge hessians to omega (adds to the negative blocks calculated before)

				// todo - might want to prealloc the block structure for the increment as well

				Increment_Omega(r_omega, m_n_edges_in_lambda); // for edges m_n_edges_in_lambda .. num_edges
				// add hessians of the edges that are new

				if(n_dummy_param) {
					for(size_t i = m_n_verts_in_lambda, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
						size_t n_col = m_lambda.n_BlockColumn_Base(i);
						size_t n_width = m_lambda.n_BlockColumn_Column_Num(i);
						r_omega.t_FindBlock(n_col, n_col, n_width, n_width, true, true).diagonal().array() += n_dummy_param;
					}
				}
				// emulate Levenberg Marquard damping on omega as well (Debug_PushDamping() not used on omega as it is not cached between the iterations)

				r_omega.ExtendTo(m_lambda.n_Row_Num(), m_lambda.n_Column_Num()); // todo - remove this once the structure is preallocated
				// in case there are no increments and the affected blocks are only somewhere on the left ...
			}
		}

		m_relin_vertex_list.clear();
		// now all the relinearizations are taken care of

		m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
		m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size();
		// now all the vertices and edges are in lambda

#ifdef _DEBUG
		if(!m_b_all_batch) { // omega only needed by the incremental pipeline
			//lambda_diff.Scale_FBS_Parallel<_TyLambdaMatrixBlockSizes>(-1); // no need, cannot be done anyway
			// use the same math; could use addition with src and dest factors but
			// that could add doubt about the results being the same (multiple ops
			// could be fused inside the FPU, with higher precision intermediate).

			lambda_diff.ExtendTo(m_lambda.n_Row_Num(), m_lambda.n_Column_Num()); // some new variables might have been added
			m_lambda.AddTo_FBS<_TyLambdaMatrixBlockSizes>(lambda_diff, -1, +1); // lambda_diff = -lambda_diff + m_lambda
			// debug - calculate difference of lambdas

			CUberBlockMatrix lambda_err = lambda_diff; // copy
			r_omega.AddTo_FBS<_TyLambdaMatrixBlockSizes>(lambda_err, -1);
			// subtract omega

			double f_norm = lambda_diff.f_Norm();
			double f_error = lambda_err.f_Norm() / std::max(1.0, f_norm);
			printf("debug: inc omega error %g (rel %g)\n", lambda_err.f_Norm(), lambda_err.f_Norm() / f_norm);
			if(f_error >= 1e-10) {
				if(!lambda_err.Rasterize("insc01_lambda_err.tga")) {
					cs *p_sparse = lambda_err.p_BlockStructure_to_Sparse();
					if(p_sparse) {
						CDebug::Dump_SparseMatrix_Subsample_AA("insc01_lambda_err.tga", p_sparse, 0, 1024, true);
						cs_spfree(p_sparse);
					}
				}
				if(!r_omega.Rasterize("insc02_omega.tga")) {
					cs *p_sparse = r_omega.p_BlockStructure_to_Sparse();
					if(p_sparse) {
						CDebug::Dump_SparseMatrix_Subsample_AA("insc02_omega.tga", p_sparse, 0, 1024, true);
						cs_spfree(p_sparse);
					}
				}
				if(!lambda_diff.Rasterize("insc03_omega_gt.tga")) {
					cs *p_sparse = lambda_diff.p_BlockStructure_to_Sparse();
					if(p_sparse) {
						CDebug::Dump_SparseMatrix_Subsample_AA("insc03_omega_gt.tga", p_sparse, 0, 1024, true);
						cs *p_sparse_om = r_omega.p_BlockStructure_to_Sparse();
						if(p_sparse_om) {
							CDebug::Dump_SparseMatrix_Subsample_AA("insc01_omega_gt_to_omega.tga", p_sparse, p_sparse_om, 1024, true);
							cs_spfree(p_sparse_om);
						}
						cs_spfree(p_sparse);
					}
				}
				r_omega.ToleranceViewOf(lambda_diff, 0, true, false); // deep copy!
				// fix it and lets go on // todo - see why it happens
			}
			_ASSERTE(f_error < 1e-8); // if this fails then something is wrong with the omega update
			// calculate error; it cannot be perfect because the order in which the reduction plan
			// sums the blocks up is undefined, some roundoff may occur (but should be quite precise)
		}
#endif // _DEBUG

		if(!m_b_all_batch && m_f_relin_thresh > 0) {
			size_t n_nzb_before = r_omega.n_Block_Num();

			CUberBlockMatrix omega_thresh;
			omega_thresh.ToleranceViewOf(r_omega, m_f_relin_thresh, true, false); // deep copy!
			r_omega.Swap(omega_thresh);
			// simply throw away small blocks of omega

			size_t n_nzb_after = r_omega.n_Block_Num();

			printf("debug: dropped " PRIsize " / " PRIsize " omega blocks\n",
				n_nzb_before - n_nzb_after, n_nzb_before);
		} else {
			static bool b_warned = false;
			if(!b_warned) {
				b_warned = true;
				fprintf(stderr, "warning: relinearization threshold disabled\n");
			}
		}

		// lambda is always up to date, using the current linearization point
		// omega is calculated from old lambda to the current lambda, so the error should not creep in

		/*r_omega.CopyLayoutTo(omega_thresh);
		for(size_t i = 0, n = r_omega.n_BlockColumn_Num(); i < n; ++ i) {
			for(size_t j = 0, m = r_omega.n_BlockColumn_Block_Num(i); j < m; ++ j) {
				size_t k = r_omega.n_Block_Row(i, j);
				CUberBlockMatrix::_TyMatrixXdRef t_block = t_Block_AtColumn(i, j);
				double f_residual = t_block.lpNorm<Eigen::Infinity>(); // todo - FBS it?
				std::map<std::pair<size_t, size_t>, double>::iterator p_it = m_accum_errors.find(std::make_pair(i, k));
				if(p_it != m_accum_errors.end())
					f_residual += *p_it;
				if(f_residual < f_thresh) { // 
					if(p_it != m_accum_errors.end())
						*p_it = f_residual;
					else
						m_accum_errors[std::make_pair(i, k)] = f_residual;
					// just store the residual to avoid drift errors
				} else {
					if(p_it != m_accum_errors.end())
						m_accum_errors.erase(p_it);
				}
			}
		}*/

		++ m_n_hessian_update_num;
		m_n_total_update_rank += m_lambda.n_BlockColumn_Num(); // rank in lambdas
		for(size_t i = 0, n = r_omega.n_BlockColumn_Num(); i < n; ++ i) {
			if(r_omega.n_BlockColumn_Block_Num(i))
				++ m_n_sparse_update_rank;
		}
		// stats

		if(!m_b_all_batch) // omega only needed by the incremental pipeline
			timer.Accum_DiffSample(m_f_omega_time);
		else
			timer.Accum_DiffSample(m_f_lambda_time);
	}

	std::map<std::pair<size_t, size_t>, double> m_accum_errors;

	void Redo_Schur(const size_t *p_schur_order, const size_t n_cut,
		const size_t n_size, CUberBlockMatrix &lambda_perm, CUberBlockMatrix &newU)
	{
		++ m_n_batch_scupd_step_num;

		//CUberBlockMatrix lambda_perm;
		m_lambda.Permute_UpperTriangular_To(lambda_perm, p_schur_order, m_schur_ordering.size(), true);
		// permute lambda

		CUberBlockMatrix /*newU,*/ newV, newD;
		CUberBlockMatrix &newA = m_SchurCompl; // will calculate that inplace
		lambda_perm.SliceTo(newA, 0, n_cut, 0, n_cut, false); // copy!! don't destroy lambda
		lambda_perm.SliceTo(newU, 0, n_cut, n_cut, n_size, false); // do not share data, will need to output this even after lambda perm perishes
		lambda_perm.SliceTo(newD, n_cut, n_size, n_cut, n_size, true);
		newV.TransposeOf(newU);
		// get the news (could calculate them by addition in case we choose to abandon lambda altogether)

		_ASSERTE(newD.b_BlockDiagonal()); // make sure it is block diagonal
		// the algorithms below wont work if D isnt a diagonal matrix

		CUberBlockMatrix &minus_newDinv = m_minus_D_inv;
		minus_newDinv.InverseOf_Symmteric_FBS<_TyDBlockSizes>(newD); // batch inverse, dont reuse incremental!
		minus_newDinv.Scale(-1);

		CUberBlockMatrix minus_newU_newDinv, minus_newU_newDinv_newV;
		minus_newU_newDinv.ProductOf_FBS<_TyUBlockSizes, _TyDBlockSizes>(newU, minus_newDinv);
		minus_newU_newDinv_newV.ProductOf_FBS<_TyUBlockSizes, _TyVBlockSizes>(minus_newU_newDinv, newV, true);

		CUberBlockMatrix &newSC_ref = newA;
		minus_newU_newDinv_newV.AddTo_FBS<_TySchurBlockSizes>(newSC_ref);
		// calculate batch SC
	}

	void Update_Schur(const size_t *p_schur_order, const size_t n_cut,
		const size_t n_size, const CUberBlockMatrix &omega,
		CUberBlockMatrix &omega_perm, CUberBlockMatrix &newU,
		const size_t UNUSED(n_relin_vertex_num)) // t_odo - output also newU in order to be able to solve (but no need to explicitly store it, unless we ditch lambda)
	{
		_ASSERTE(!m_b_all_batch); // this is intended for the incremental pipeline only
		_ASSERTE(m_b_last_iteration_updated_schur); // make sure we are updating from a valid schur complement

		if(!m_b_force_inc_schur) {
			size_t n_omega_nnz_block_col_num = 0;
			for(size_t i = 0, n = omega.n_BlockColumn_Num(); i < n; ++ i) {
				if(omega.n_BlockColumn_Block_Num(i))
					++ n_omega_nnz_block_col_num;
			}
			if(n_omega_nnz_block_col_num > omega.n_BlockColumn_Num() / 2) {
				Redo_Schur(p_schur_order, n_cut, n_size, omega_perm, newU);
				// omega_perm will contain lambda_perm, but only the structure is used (for permuting and invpermuting the solution vectors)
				return;
			}
		}
		++ m_n_inc_scupd_step_num;

		//CUberBlockMatrix omega_perm; // as output, will be needed for reordering
		omega.Permute_UpperTriangular_To(omega_perm, p_schur_order, m_schur_ordering.size(), true);
		// permute omega

		CUberBlockMatrix dA, dU, dV, dD;
		omega_perm.SliceTo(dA, 0, n_cut, 0, n_cut, false); // copy!! will update SC from that (todo - )
		omega_perm.SliceTo(dU, 0, n_cut, n_cut, n_size, true);
		omega_perm.SliceTo(dD, n_cut, n_size, n_cut, n_size, true);
		dV.TransposeOf(dU);
		// get the deltas

		CUberBlockMatrix lambda_perm;
		m_lambda.Permute_UpperTriangular_To(lambda_perm, p_schur_order, m_schur_ordering.size(), true);
		// permute lambda

		CUberBlockMatrix /*newU,*/ newV, newD;
		lambda_perm.SliceTo(newU, 0, n_cut, n_cut, n_size, false); // do not share data, will need to output this even after lambda perm perishes
		lambda_perm.SliceTo(newD, n_cut, n_size, n_cut, n_size, true);
		newV.TransposeOf(newU);
		// get the news (could calculate them by addition in case we choose to abandon lambda altogether)

		_ASSERTE(!dD.b_OffDiagonal_Blocks()); // make sure it is block diagonal, can be rank deficient
		_ASSERTE(newD.b_BlockDiagonal()); // make sure it is block diagonal
		// the algorithms below wont work if D isnt a diagonal matrix

		CUberBlockMatrix dDinv;
		if(m_minus_D_inv.b_Empty()) {
			dDinv.InverseOf_BlockDiag_FBS_Parallel<_TyDBlockSizes>(dD); // todo - block diag, parallel // todo - can filter block sizes to only the square ones that can occur on the diagonal of a symmetric matrix
			// Dnew = 0 + dD, dDinv = Dnew^-1 - D^-1 = dD^-1 - 0^-1 ... thats sort of strange
		} else {
			newD.CopyLayoutTo(dDinv); // the layout will be the same
			for(size_t i = 0, n = dD.n_BlockColumn_Num(); i < n; ++ i) {
				size_t bn;
				if((bn = dD.n_BlockColumn_Block_Num(i)) != 0) {
					_ASSERTE(bn == 1 && dD.n_Block_Row(i, 0) == i);
					// todo - this should really throw, the whole thing will be numerically off (rather than dense but correct as in batch schur)

					size_t n_base = dD.n_BlockColumn_Base(i), n_width = dD.n_BlockColumn_Column_Num(i);

					CUberBlockMatrix::_TyMatrixXdRef newDii = newD.t_FindBlock(n_base, n_base);
					_ASSERTE(newDii.rows() == newDii.cols() && newDii.cols() == n_width);

					CUberBlockMatrix::_TyMatrixXdRef minus_Dinvii = m_minus_D_inv.t_FindBlock(n_base, n_base);
					if(!minus_Dinvii.cols())
						dDinv.Append_Block(newDii.inverse(), n_base, n_base);
					else
						dDinv.Append_Block(newDii.inverse() + minus_Dinvii, n_base, n_base);
					// todo - FBS
				}
			}
			// assuming dD and m_minus_D_inv and newD are block diagonal, the delta is nonzero only in the nonzero parts of dD
		}
		// calculate delta of D inverse (delta of inverse rather than inverse of delta)

		m_SchurCompl.ExtendTo(dA.n_Row_Num(), dA.n_Column_Num()); // might have resized
		m_minus_D_inv.ExtendTo(dD.n_Row_Num(), dD.n_Column_Num()); // might have resized
		// extend the matrices so that they could be used below

#if defined(_DEBUG) || defined(__NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_DEBUGGING)
		{
			CUberBlockMatrix dDinv_ref;
			dDinv_ref.InverseOf_Symmteric(newD);
			m_minus_D_inv.AddTo(dDinv_ref);
			// calculate the reference dDinv (should equal what we calculated, except that there are plentiful blocks)

			double f_norm = dDinv_ref.f_Norm();
			dDinv.AddTo(dDinv_ref, -1);
			double f_error = dDinv_ref.f_Norm(); // there will likely be some error as the ad-hoc delta inverse does not have SSE yet
			printf("debug: inc dD^-1 error %g (rel %g)\n", f_error, f_error / f_norm);
			_ASSERTE(f_error < 1e-8);
		}
#endif // _DEBUG
		// check delta of D inverse

#ifdef __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
		CUberBlockMatrix Ela_SC = m_SchurCompl, Ela_dSC = dA; // save this before incrementing
		CUberBlockMatrix l_SC = m_SchurCompl, l_dSC = dA; // save this before incrementing
		size_t n_L_FLOP_num = 0, n_L_FLOP_delta;
#endif // __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING

		CUberBlockMatrix minus_dU_Dinv, minus_dU_Dinv_newV, minus_dU_Dinv_dV,
			newU_dDinv, newU_dDinv_newV, minus_dU_Dinv_newV_plus_dU_Dinv_dV,
			minus_dU_Dinv_newV_UT, minus_dU_Dinv_newV_plus_dU_Dinv_dV_UT;
		minus_dU_Dinv.ProductOf_FBS<_TyUBlockSizes, _TyDBlockSizes>(dU, m_minus_D_inv); // very sparse
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_GEMM_FLOP_Num(dU, m_minus_D_inv))); DL_SC_Profile(printf("L0: " PRIsize "\n", n_L_FLOP_delta));
		minus_dU_Dinv_newV.ProductOf_FBS<_TyUBlockSizes, _TyVBlockSizes>(minus_dU_Dinv, newV); // also sparse, unfortunately need both halves
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_GEMM_FLOP_Num(minus_dU_Dinv, newV))); DL_SC_Profile(printf("L1: " PRIsize "\n", n_L_FLOP_delta));
		minus_dU_Dinv_dV.ProductOf_FBS<_TyUBlockSizes, _TyVBlockSizes>(minus_dU_Dinv, dV, true); // even more sparse, only need the upper triangular
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_GEMM_FLOP_Num(minus_dU_Dinv, dV) / 2)); DL_SC_Profile(printf("L2: " PRIsize "\n", n_L_FLOP_delta));
		newU_dDinv.ProductOf_FBS<_TyUBlockSizes, _TyDBlockSizes>(newU, dDinv); // very sparse
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_GEMM_FLOP_Num(newU, dDinv))); DL_SC_Profile(printf("L3: " PRIsize "\n", n_L_FLOP_delta));
		newU_dDinv_newV.ProductOf_FBS<_TyUBlockSizes, _TyVBlockSizes>(newU_dDinv, newV, true); // also sparse, only need the upper triangular
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_GEMM_FLOP_Num(newU_dDinv, newV) / 2)); DL_SC_Profile(printf("L4: " PRIsize "\n", n_L_FLOP_delta));
		// calculate all the partials

		CUberBlockMatrix &dSC = dA; // rename, start with dA
		minus_dU_Dinv_newV_UT.TriangularViewOf(minus_dU_Dinv_newV, true, true);
		minus_dU_Dinv_newV_UT.AddTo_FBS<_TySchurBlockSizes>(dSC); // t_odo - need an upper tri view of this
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_ADD_FLOP_Num(minus_dU_Dinv_newV_UT, dSC, 1, 1))); DL_SC_Profile(printf("L5: " PRIsize "\n", n_L_FLOP_delta));
		newU_dDinv_newV.AddTo_FBS<_TySchurBlockSizes>(dSC, -1);
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_ADD_FLOP_Num(newU_dDinv_newV, dSC, 1, -1))); DL_SC_Profile(printf("L6: " PRIsize "\n", n_L_FLOP_delta));
		{
			minus_dU_Dinv_newV_plus_dU_Dinv_dV.TransposeOf(minus_dU_Dinv_newV);
			minus_dU_Dinv_dV.AddTo_FBS<_TySchurBlockSizes>(minus_dU_Dinv_newV_plus_dU_Dinv_dV, -1); // -dU_Dinv_newV - (-dU_Dinv_dV), invalidates minus_dU_Dinv_newV
			DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_ADD_FLOP_Num(minus_dU_Dinv_dV, minus_dU_Dinv_newV_plus_dU_Dinv_dV, 1, -1))); DL_SC_Profile(printf("L7: " PRIsize "\n", n_L_FLOP_delta));
		}
		minus_dU_Dinv_newV_plus_dU_Dinv_dV_UT.TriangularViewOf(minus_dU_Dinv_newV_plus_dU_Dinv_dV, true, true);
		minus_dU_Dinv_newV_plus_dU_Dinv_dV_UT.AddTo_FBS<_TySchurBlockSizes>(dSC); // t_odo - need an upper tri view of this
		DL_SC_Profile(n_L_FLOP_num += (n_L_FLOP_delta = n_ADD_FLOP_Num(minus_dU_Dinv_newV_plus_dU_Dinv_dV_UT, dSC, 1, 1))); DL_SC_Profile(printf("L8: " PRIsize "\n", n_L_FLOP_delta));
		// dSC = dA - (dU_Dinv_newV - dU_Dinv_dV)^T - dU_Dinv_newV - newU_dDinv_newV
		// dSC = dA - (dU_Dinv_newV^T - dU_Dinv_dV) - dU_Dinv_newV - newU_dDinv_newV // since dU_Dinv_dV is symmetric, can skip calculating its lower triangle

		dSC.AddTo_FBS<_TySchurBlockSizes>(m_SchurCompl);
		// calculate the new schur complement incrementally

#ifdef __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
		CUberBlockMatrix old_minus_D_inv = m_minus_D_inv;
#endif // __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING

		dDinv.AddTo_FBS<_TyDBlockSizes>(m_minus_D_inv, -1);
		// calculate the new Dinv incrementally (maybe a bad idea, could drift away,
		// maybe it would be better to just replace the blocks and ommit the add / sub)

#ifdef __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
		size_t n_E_FLOP_num = 0;

		{
			CUberBlockMatrix Ela_U_mod; // columns of \hat{U} which are nonzero in dU
			CUberBlockMatrix Ela_minus_Dinv_mod; // columns of \hat{D\inv} which are nonzero in dD
			{
				std::vector<size_t> mod_U_cols, mod_U_empty_cols;
				for(size_t i = 0, n = dU.n_BlockColumn_Num(); i < n; ++ i) {
					if(dU.n_BlockColumn_Block_Num(i))
						mod_U_cols.push_back(i);
					else
						mod_U_empty_cols.push_back(i);
				}
				size_t n_U_cut = mod_U_cols.size();
				mod_U_cols.insert(mod_U_cols.end(), mod_U_empty_cols.begin(), mod_U_empty_cols.end());
				// gather nonzero columns in dU, make a permutation which reorders them to the left

				CMatrixOrdering mord;
				const size_t *p_ord = &mod_U_cols.front(),
					*p_inv_ord = mord.p_InvertOrdering(p_ord, mod_U_cols.size());
				std::swap(p_ord, p_inv_ord);
				// PermuteTo scatters dest[perm[i]] = src[i]; mod_U_cols records source indices so we need to invert

				CUberBlockMatrix new_U_perm;
				newU.PermuteTo(new_U_perm, p_ord, mod_U_cols.size(), false, true, true);
				// reorder newU to have the changing columns on the left

				CUberBlockMatrix new_Dinv_perm;
				m_minus_D_inv.PermuteTo(new_Dinv_perm, p_ord, mod_U_cols.size(), true, true, true);
				// reorder newU to have the changing columns on the left

				new_U_perm.SliceTo(Ela_U_mod, new_U_perm.n_BlockRow_Num(), n_U_cut, true);
				new_Dinv_perm.SliceTo(Ela_minus_Dinv_mod, n_U_cut, n_U_cut, true);
				for(size_t i = 0, n = mod_U_empty_cols.size(), r = Ela_U_mod.n_Row_Num(),
				   c = Ela_U_mod.n_Column_Num(); i < n; ++ i) {
					c += dU.n_BlockColumn_Column_Num(mod_U_empty_cols[i]);
					Ela_U_mod.ExtendTo(r, c);
					Ela_minus_Dinv_mod.ExtendTo(c, c);
				}
				// slice only the changing columns to Ela_U_mod, backfill zero columns on the right

				Ela_U_mod.PermuteTo(new_U_perm, p_inv_ord, mod_U_cols.size(), false, true, false); // deep copy; make this writable
				Ela_U_mod.Swap(new_U_perm);
				Ela_minus_Dinv_mod.PermuteTo(new_Dinv_perm, p_inv_ord, mod_U_cols.size(), true, true, false); // deep copy; make this writable
				Ela_minus_Dinv_mod.Swap(new_Dinv_perm);
				// inverse permute

				/*CUberBlockMatrix check;
				check = m_minus_D_inv;
				Ela_minus_Dinv_mod.AddTo(check, -1);
				double f_check0 = check.f_Norm();
				check = newU;
				Ela_U_mod.AddTo(check, -1);
				double f_check1 = check.f_Norm();*/ // debug - makes sense only in the first step (supposed to be null)
			}
			// make Ela_U_mod and Ela_minus_Dinv_mod

			{
				CUberBlockMatrix EpA, Ela_U_mod_T, EpB;
				bool b_fail = !EpA.ProductOf(Ela_U_mod, Ela_minus_Dinv_mod);
				if(b_fail)
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
				n_E_FLOP_num += n_GEMM_FLOP_Num(Ela_U_mod, Ela_minus_Dinv_mod);
				Ela_U_mod_T.TransposeOf(Ela_U_mod);
				EpB.ProductOf(EpA, Ela_U_mod_T); // EpB = \hat{U} * -\hat{D^-1} * \hat{U}^T
				n_E_FLOP_num += n_GEMM_FLOP_Num(EpA, Ela_U_mod_T) / 2;
				b_fail |= !EpB.AddTo(Ela_dSC); // \Omega_c - \hat{U} * \hat{D}^-1 * \hat{U}^T
				if(b_fail)
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
				//printf("EpB %g ", EpB.f_Norm()); // debug

				n_E_FLOP_num += n_ADD_FLOP_Num(EpB, Ela_dSC, 1, -1);
				b_fail |= !dU.AddTo(Ela_U_mod, -1); // \hat{U} - \Omega_{CP} -> U (or perhaps -U, does not matter)
				if(b_fail)
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
				n_E_FLOP_num += n_ADD_FLOP_Num(dU, Ela_U_mod, -1, 1);
				b_fail |= !dDinv.AddTo(Ela_minus_Dinv_mod); // -\hat{D^}-1 - \dta {D^}-1 -> -{D^}-1
				if(b_fail)
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
				n_E_FLOP_num += n_ADD_FLOP_Num(dDinv, Ela_minus_Dinv_mod, 1, 1);
				b_fail |= !EpA.ProductOf(Ela_U_mod, Ela_minus_Dinv_mod);
				if(b_fail)
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
				n_E_FLOP_num += n_GEMM_FLOP_Num(Ela_U_mod, Ela_minus_Dinv_mod);
				Ela_U_mod_T.TransposeOf(Ela_U_mod);
				b_fail |= !EpB.ProductOf(EpA, Ela_U_mod_T); // EpB = U * -D^-1 * U^T
				if(b_fail)
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
				n_E_FLOP_num += n_GEMM_FLOP_Num(EpA, Ela_U_mod_T) / 2;
				//printf("%g\n", EpB.f_Norm()); // debug
				b_fail |= !EpB.AddTo(Ela_dSC, -1); // \Omega_c - \hat{U} * \hat{D}^-1 * \hat{U}^T + U * D^-1 * U^T or "subtract new, add old"
				n_E_FLOP_num += n_ADD_FLOP_Num(EpB, Ela_dSC, -1, 1);

				if(b_fail) {
					fprintf(stderr, "internal error: linalg fails in Sc update (line %d)\n", __LINE__);
					throw std::runtime_error("unexpected linalg fails");
				}
			}
			//Ela_SC = m_SchurCompl; // above, before it gets modified
			CUberBlockMatrix Ela_dSCU;
			Ela_dSCU.TriangularViewOf(Ela_dSC, true, true);
			Ela_dSCU.AddTo(Ela_SC);
		}
		// calculate the delta Schur Ela's way

		size_t n_l_FLOP_num = 0;
		{
			CUberBlockMatrix minus_F;
			minus_F.ProductOf(old_minus_D_inv, dV); // need full prod
				n_l_FLOP_num += n_GEMM_FLOP_Num(old_minus_D_inv, dV);
			CUberBlockMatrix oldU = newU;
			dU.AddTo(oldU, -1);
				n_l_FLOP_num += n_ADD_FLOP_Num(dU, oldU, -1, 1);
			CUberBlockMatrix lpA, lpB;
			lpA.ProductOf(oldU, minus_F); // only need upper part of this
				n_l_FLOP_num += n_GEMM_FLOP_Num(oldU, minus_F) / 2;
			CUberBlockMatrix lpAU;
			lpAU.TriangularViewOf(lpA, true, true);
			lpAU.AddTo(l_dSC);
				n_l_FLOP_num += n_ADD_FLOP_Num(lpAU, l_dSC, 1, 1);
			CUberBlockMatrix newUdDinv;
			newUdDinv.ProductOf(newU, dDinv); // need full prod
				n_l_FLOP_num += n_GEMM_FLOP_Num(newU, dDinv);
			CUberBlockMatrix minus_Ft;
			minus_Ft.TransposeOf(minus_F);
			newUdDinv.AddTo(minus_Ft, -1);
				n_l_FLOP_num += n_ADD_FLOP_Num(newUdDinv, minus_Ft, -1, 1);
			lpB.ProductOf(minus_Ft, newV); // only need upper part of this
				n_l_FLOP_num += n_GEMM_FLOP_Num(minus_Ft, newV) / 2;
			CUberBlockMatrix lpBU;
			lpBU.TriangularViewOf(lpB, true, true);
			lpBU.AddTo(l_dSC);
				n_l_FLOP_num += n_ADD_FLOP_Num(lpBU, l_dSC, 1, 1);

			//l_SC = m_SchurCompl; // above, before it gets modified
			CUberBlockMatrix l_dSCU;
			l_dSCU.TriangularViewOf(l_dSC, true, true);
			l_dSCU.AddTo(l_SC);
		}

		printf("L: " PRIsize " FLOP\n", n_L_FLOP_num);
		printf("l: " PRIsize " FLOP\n", n_l_FLOP_num);
		printf("E: " PRIsize " FLOP\n", n_E_FLOP_num);

		size_t n_B_FLOP_num = 0;
		{
			CUberBlockMatrix newA;
			lambda_perm.SliceTo(newA, 0, n_cut, 0, n_cut, false); // copy!! don't destroy lambda
			const CUberBlockMatrix &newDinv = newD; // just rename; the structure of the inverse will be the same

			CUberBlockMatrix newU_newDinv, newU_newDinv_newV;
			newU_newDinv.ProductOf(newU, newDinv);
			n_B_FLOP_num += n_GEMM_FLOP_Num(newU, newDinv);
			newU_newDinv_newV.ProductOf(newU_newDinv, newV, true);
			n_B_FLOP_num += n_GEMM_FLOP_Num(newU_newDinv, newV) / 2;
			n_B_FLOP_num += n_ADD_FLOP_Num(newU_newDinv_newV, newA, -1, 1);
		}
		// calculate FLOPs in batch SC (while trying to avoid actually calculating it)

		printf("B: " PRIsize " FLOP\n", n_B_FLOP_num);
		printf("rank of omega: " PRIsize " blocks (number of vertices: "
			PRIsize ", originating vertices: " PRIsize ")\n",
			n_omega_nnz_block_col_num, omega_perm.n_BlockColumn_Num(), n_relin_vertex_num);
#endif // __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING

#if defined(_DEBUG) || defined(__NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_DEBUGGING)
		{
			CUberBlockMatrix newA;
			lambda_perm.SliceTo(newA, 0, n_cut, 0, n_cut, false); // copy!! don't destroy lambda
			CUberBlockMatrix newDinv;
			newDinv.InverseOf_Symmteric(newD); // batch inverse, dont reuse incremental!

			CUberBlockMatrix newU_newDinv, newU_newDinv_newV;
			newU_newDinv.ProductOf(newU, newDinv);
			newU_newDinv_newV.ProductOf(newU_newDinv, newV, true);

			CUberBlockMatrix &newSC_ref = newA;
			newU_newDinv_newV.AddTo(newSC_ref, -1);
			// calculate batch SC

			CUberBlockMatrix SCerr = newSC_ref;
			m_SchurCompl.AddTo(SCerr, -1);
			CUberBlockMatrix Dinverr = newDinv;
			m_minus_D_inv.AddTo(Dinverr);
			double f_SC_error = SCerr.f_Norm(), f_SC_norm = newSC_ref.f_Norm();
			double f_Dinv_error = Dinverr.f_Norm(), f_Dinv_norm = newDinv.f_Norm();
			printf("debug: inc SC error %g (rel %g)\n", f_SC_error, f_SC_error / f_SC_norm);
#ifdef __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
			newSC_ref.AddTo(Ela_SC, -1); // Ela_SC not needed below, can calculate the difference inplace
			CUberBlockMatrix Ela_SCerrU;
			Ela_SCerrU.TriangularViewOf(Ela_SC, true, true);
			double f_Ela_SC_error = Ela_SCerrU.f_Norm();
			printf("debug: Ela's inc SC error %g (rel %g)\n", f_Ela_SC_error, f_Ela_SC_error / f_SC_norm);

			newSC_ref.AddTo(l_SC, -1); // Ela_SC not needed below, can calculate the difference inplace
			CUberBlockMatrix l_SCerrU;
			l_SCerrU.TriangularViewOf(l_SC, true, true);
			double f_l_SC_error = l_SCerrU.f_Norm();
			printf("debug: l's inc SC error %g (rel %g)\n", f_l_SC_error, f_l_SC_error / f_SC_norm);
#endif // __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
			printf("debug: inc D^-1 error %g (rel %g)\n", f_Dinv_error, f_Dinv_error / f_Dinv_norm);
			if(f_SC_error / std::max(1.0, f_SC_norm) >= 1e-10) {
				SCerr.Rasterize("insc04_sc_err.tga");
				m_SchurCompl.Rasterize("insc05_sc.tga");
				newSC_ref.Rasterize("insc06_sc_gt.tga");
			}
			_ASSERTE(f_SC_error / std::max(1.0, f_SC_norm) < 1e-8);
			_ASSERTE(f_Dinv_error / std::max(1.0, f_Dinv_norm) < 1e-8);

#ifdef __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
			if(f_Ela_SC_error / std::max(1.0, f_SC_norm) > 1e-10) {
				Ela_SCerrU.Rasterize("insc04_ela_sc_err.tga");
				Ela_SC.Rasterize("insc05_ela_sc.tga");
				newSC_ref.Rasterize("insc06_sc_gt.tga");
				//throw std::runtime_error("f_Ela_SC_error is large"); // this works well, it only happens in times of heavy roundoff
			}
			if(f_l_SC_error / std::max(1.0, f_SC_norm) > 1e-10) {
				l_SCerrU.Rasterize("insc04_l_sc_err.tga");
				l_SC.Rasterize("insc05_l_sc.tga");
				newSC_ref.Rasterize("insc06_sc_gt.tga");
				//throw std::runtime_error("f_Ela_SC_error is large"); // this works well, it only happens in times of heavy roundoff
			}
#endif // __NONLINEAR_SOLVER_LAMBDA_DL_INC_SCHUR_PROFILING
		}
		// check what is the error of the Schur complement
#endif // _DEBUG
	}

	std::vector<double> m_double_workspace; // for schur solver

	/**
	 *	@brief some run-time constants, stored as enum
	 */
	enum {
		b_Have_NativeSolver = CIsNativeSolver<_TyLinearSolver>::b_result /**< @brief determines if the native linear solver is being used */
	};

#if defined(__SCHUR_USE_DENSE_SOLVER) || defined(__SCHUR_DENSE_SOLVER_USE_GPU)
#ifdef __SCHUR_DENSE_SOLVER_USE_GPU
	typedef CLinearSolver_DenseGPU _TySchurSolver; /**< @brief name of the base linear solver */
#else // __SCHUR_DENSE_SOLVER_USE_GPU
	typedef CLinearSolver_DenseEigen _TySchurSolver; /**< @brief name of the base linear solver */
#endif // __SCHUR_DENSE_SOLVER_USE_GPU
#else // __SCHUR_USE_DENSE_SOLVER || __SCHUR_DENSE_SOLVER_USE_GPU
	typedef typename CChooseType<CLinearSolver_UberBlock<_TySchurBlockSizes>,
		_TyLinearSolver, b_Have_NativeSolver>::_TyResult _TySchurSolver; /**< @brief linear solver for Schur complement */
#endif // __SCHUR_USE_DENSE_SOLVER || __SCHUR_DENSE_SOLVER_USE_GPU
	// use GPU solver where needed

	typedef typename _TySchurSolver::_Tag _TySCSolverTag; /**< @brief RCS solver tag */
	typedef CLinearSolverWrapper<_TySchurSolver, _TySCSolverTag> _TySCSolverWrapper; /**< @brief wrapper for RCS solvers (shields solver capability to solve blockwise) */

	///*_TyLinearSolver*/CLinearSolver_UberBlock<_TySchurBlockSizes> m_rcs_solver; // for schur solver // t_odo - detect whether _TyLinearSolver is uberblock. if it is not, use _TyLinearSolver instead of uberblock!
	_TySchurSolver m_rcs_solver; // for schur solver

	bool IncSchur_Solve(const CUberBlockMatrix &r_lambda_perm_structure, const CUberBlockMatrix &r_SC,
		const CUberBlockMatrix &r_minus_D_inv, const CUberBlockMatrix &r_U, Eigen::VectorXd &r_v_eta,
		const size_t *p_schur_order, size_t n_cut, size_t n_size, bool b_keep_ordering) // throw(std::bad_alloc)
	{
		CTimerSampler timer(this->m_timer);

		size_t n_landmark_vector_size = r_minus_D_inv.n_Column_Num(); // 3 * (n - n_matrix_cut);
		size_t n_pose_vector_size = r_SC.n_Column_Num(); // 6 * n_matrix_cut;
		size_t n_rhs_vector_size = n_pose_vector_size + n_landmark_vector_size;
		// not block columns! element ones

		_ASSERTE(n_rhs_vector_size == r_v_eta.rows());
		_ASSERTE(r_lambda_perm_structure.n_Column_Num() == r_v_eta.rows());
		_ASSERTE(r_lambda_perm_structure.n_BlockColumn_Num() == n_size);
		_ASSERTE(r_SC.n_BlockColumn_Num() == n_cut);
		_ASSERTE(r_SC.n_BlockColumn_Num() + r_minus_D_inv.n_BlockColumn_Num() == n_size);
		// basic checks

		if(m_double_workspace.capacity() < n_rhs_vector_size) {
			m_double_workspace.clear(); // avoid data copying
			m_double_workspace.reserve(std::max(2 * m_double_workspace.capacity(), n_rhs_vector_size));
		}
		m_double_workspace.resize(n_rhs_vector_size);
		double *p_double_workspace = &m_double_workspace[0];
		// alloc workspace

		timer.Accum_DiffSample(m_f_schursolve_alloc);

		// t_odo - use optimal block sizes

		/*CUberBlockMatrix lambda_structure, lambda_perm_structure;
		m_lambda.CopyLayoutTo(lambda_structure);
		lambda_structure.PermuteTo(lambda_perm_structure, p_schur_order, m_schur_ordering.size());
		// need lambda_perm for block-reordering the vector, otherwise would not need it
		// could probably reuse omega perm for this to avoid doing it twice (should be fairly fast still)*/
		// t_odo - do this

		r_lambda_perm_structure.InversePermute_RightHandSide_Vector(p_double_workspace,
			&r_v_eta(0), n_rhs_vector_size, p_schur_order, n_size);
		// need to permute the vector !!

		Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
		Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace
		// get eta and cut it into pieces
		// \eta = | x |
		//        | l |

		timer.Accum_DiffSample(m_f_schursolve_perm);

		// we are now solving:
		// \lambda          \eta
		// | A U | | dx | = | x |
		// | V C | | dl |   | l |

#if 0
		CUberBlockMatrix minus_U_Cinv;
		minus_U_Cinv.ProductOf_FBS<_TyUBlockSizes, _TyDBlockSizes>(r_U, r_minus_D_inv);
		// todo - try doing an extra GAXPY instead, should be faster since we do not explicitly
		// need UCinv for anything (except when performing a full rank update, could optimize for that if it turns out it happens often)

		minus_U_Cinv.PreMultiply_Add_FBS<_TyUBlockSizes>(&v_x(0),
			n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
		// compute right-hand side x - U(C^-1)l via GEMM and GAXPY
#else // 0
		Eigen::VectorXd v_minus_Dinv_l = Eigen::VectorXd::Zero(n_landmark_vector_size);
		r_minus_D_inv.PreMultiply_Add_FBS<_TyDBlockSizes>(&v_minus_Dinv_l(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // -(C^-1)l
		r_U.PreMultiply_Add_FBS<_TyUBlockSizes>(&v_x(0),
			n_pose_vector_size, &v_minus_Dinv_l(0), n_landmark_vector_size); // x - U(C^-1)l
		// compute right-hand side x - U(C^-1)l via two GAXPY
#endif // 0

		timer.Accum_DiffSample(m_f_schursolve_rcs_rhs);

		if(!b_keep_ordering)
			_TySCSolverWrapper::FinalBlockStructure(m_rcs_solver, r_SC); // the ordering on r_SC will not change, can calculate it only in the first pass and then reuse
		bool b_result;// = _TyLinearSolverWrapper::Solve(m_linear_solver, r_SC, v_x);
		{
			//r_SC.Rasterize("insc_05_SC.tga");
			b_result = _TySCSolverWrapper::Solve(m_rcs_solver, r_SC, v_x);
		}
		Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
		// solve for dx = A - U(C^-1)V / x

		timer.Accum_DiffSample(m_f_schursolve_rcs_solve); ++ m_n_schursolve_rcs_solve_times;

		// note that r_SC only contains pose-sized blocks when guided ordering is used! could optimize for that
		// also note that r_SC is not completely dense if it is not many times smaller than C

		Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
		// obtained the first part of the solution

		Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // calculated inplace
		v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above

		r_U.PostMultiply_Add_FBS_Parallel<_TyUBlockSizes>(&v_l(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
		//printf("post-mad error %g (rel %g)\n", (v_l_copy - v_l).norm(), (v_l_copy - v_l).norm() / v_l.norm());

		// l = V * dx - l
		v_dl.setZero();
		r_minus_D_inv.PreMultiply_Add_FBS<_TyDBlockSizes>(&v_dl(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
		// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
		// the second part of the solution is calculated inplace in the dest vector

		timer.Accum_DiffSample(m_f_schursolve_lm_solve);

		r_lambda_perm_structure.Permute_RightHandSide_Vector(&r_v_eta(0), p_double_workspace,
			n_rhs_vector_size, p_schur_order, n_size);
		// permute back!

		timer.Accum_DiffSample(m_f_schursolve_perm); // perm / unperm

		return b_result;
	}

	CNonlinearSolver_Lambda_DL(const CNonlinearSolver_Lambda_DL &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_Lambda_DL &operator =(const CNonlinearSolver_Lambda_DL &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */

	static size_t n_GEMM_FLOP_Num(const CUberBlockMatrix &r_A, const CUberBlockMatrix &r_B)
	{
		cs *p_A = r_A.p_Convert_to_Sparse();
		cs *p_B = r_B.p_Convert_to_Sparse();
		size_t n_result = n_GEMM_FLOP_Num(p_A, p_B);
		cs_spfree(p_A);
		cs_spfree(p_B);
		return n_result;
	}

	static size_t n_ADD_FLOP_Num(const CUberBlockMatrix &r_A,
		const CUberBlockMatrix &r_B, double f_k1, double f_k2)
	{
		cs *p_A = r_A.p_Convert_to_Sparse();
		cs *p_B = r_B.p_Convert_to_Sparse();
		size_t n_result = n_ADD_FLOP_Num(p_A, p_B, f_k1, f_k2);
		cs_spfree(p_A);
		cs_spfree(p_B);
		return n_result;
	}

	typedef CTSparse<CFLOPCountingDouble> CFLOPCountingSparse;

	static size_t n_ADD_FLOP_Num(const cs *p_A, const cs *p_B, double f_k1, double f_k2)
	{
		size_t n_before = CFLOPCountingDouble::n_FLOP_Num();

		cs *p_result = CFLOPCountingSparse::p_ToSparse(CFLOPCountingSparse::add(
			CFLOPCountingSparse::p_FromSparse(p_A), CFLOPCountingSparse::p_FromSparse(p_B), f_k1, f_k2));
		cs_spfree(p_result);

		return CFLOPCountingDouble::n_FLOP_Num() - n_before;
	}

	static size_t n_GEMM_FLOP_Num(const cs *p_A, const cs *p_B)
	{
		size_t n_before = CFLOPCountingDouble::n_FLOP_Num();

		cs *p_result = CFLOPCountingSparse::p_ToSparse(CFLOPCountingSparse::multiply(
			CFLOPCountingSparse::p_FromSparse(p_A), CFLOPCountingSparse::p_FromSparse(p_B)));
		cs_spfree(p_result);

		return CFLOPCountingDouble::n_FLOP_Num() - n_before;
	}
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED
