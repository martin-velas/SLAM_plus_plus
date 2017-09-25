/*
								+-----------------------------------+
								|                                   |
								| ***  Lambda nonlinear solver  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|    NonlinearSolver_Lambda_LM.h    |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_LAMBDA_LM_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_LAMBDA_LM_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_Lambda_LM.h
 *	@brief nonlinear blocky solver working above the lambda matrix, with Levenberg Marquardt
 *	@author -tHE SWINe-
 *	@date 2012-09-13
 */

#include "slam/LinearSolver_Schur.h"
#include "slam/OrderingMagic.h"
#include "slam/Marginals.h"
//#include "slam/NonlinearSolver_Lambda.h" // the whole lambda not needed
#include "slam/NonlinearSolver_Base.h"
#include "slam/NonlinearSolver_Lambda_Base.h" // the utils will suffice

#include "slam/BAMarginals.h" // CSchurComplement_Marginals

extern int n_dummy_param;
// governs the thresholding of dx (a negative value is the power of 10 to threshold by)

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
 *	@brief enables writes of chi2 errors at each step
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
 *	@brief if defined, the number of iterations is quadrupled and after
 *		every full iteration there are three landmark settle iterations
 */
//#define __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
 *	@brief enables threshold on the dx vector. n_dummy_param used as
 *		a power of 10 to define the threshold (must be negative)
 */
//#define __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES
 *	@brief enables saving of the dx vector to a files, for diagnostic purposes
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
 *	@brief enables saving of the difference of the system matrix between
 *		different calls to Optimize() as images, for diagnostic purposes
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_LM_USE_DUMMY_LM_DAMPING
 *	@brief rather than using a heuristic for the Levenberg-Marquardt damping calculation,
 *		the value of \ref n_dummy_param is used instead (can be set from commandline by
 *		using the <tt>-dp</tt> switch).
 */
//#define __NONLINEAR_SOLVER_LAMBDA_LM_USE_DUMMY_LM_DAMPING

/**
 *	@brief trust region heuristic interface
 *	@tparam CLambdaLM_Solver is a specialization of CNonlinearSolver_Lambda_LM
 *	@note This just enforces that all the functions are declated and in correct form.
 */
template <class CLambdaLM_Solver>
class CTrustRegion_Interface {
public:
	/**
	 *	@brief calculates error metric for LM
	 *	@param[in] r_solver is reference to the solver (solver state should be all valid at this point)
	 *	@return Returns error metric for LM.
	 */
	virtual double f_Error(const CLambdaLM_Solver &r_solver) = 0;

	/**
	 *	@brief calculates initial damping for LM
	 *	@param[in] m_r_system is reference to the system (solver state is not completely up to date)
	 *	@return Returns the initial damping for LM.
	 */
	virtual double f_InitialDamping(const typename CLambdaLM_Solver::_TySystem& m_r_system) = 0;

	/**
	 *	@brief decides on stepping forward / backward after updating the system
	 *
	 *	@param[in,out] r_f_last_error is reference to error before the update
	 *	@param[in] f_error is error after the update
	 *	@param[in,out] r_f_alpha is the current damping factor
	 *	@param[in] r_lambda is reference to the system matrix before the update
	 *	@param[in] r_solver is reference to the solver
	 *	@param[in] r_v_dx is reference to the dx vector
	 *	@param[in] r_v_rhs is reference to the right-hand-side vector in this iteration
	 *
	 *	@return Returns true if the step was a success, false if the step needs to be rolled back.
	 */
	virtual bool Aftermath(double &r_f_last_error, double f_error, double &r_f_alpha, const CUberBlockMatrix &r_lambda,
		const CLambdaLM_Solver &r_solver, const Eigen::VectorXd &r_v_dx, const Eigen::VectorXd &r_v_rhs) = 0;

	/**
	 *	@brief adds the damping factor to the system matrix
	 *
	 *	@param[in,out] r_lambda is reference to the system matrix before the update
	 *	@param[in] f_alpha is the current damping factor
	 *	@param[in] n_first_vertex is zero-based index of the first column of lambda to refresh
	 *	@param[in] n_last_vertex is zero-based index of the one past the last column of lambda to refresh
	 */
	virtual void ApplyDamping(CUberBlockMatrix &r_lambda, double f_alpha, size_t n_first_vertex, size_t n_last_vertex) = 0;
};

/**
 *	@brief baseline LM algorithm
 *	@tparam CLambdaLM_Solver is a specialization of CNonlinearSolver_Lambda_LM
 */
template <class CLambdaLM_Solver>
class CLevenbergMarquardt_Baseline : public CTrustRegion_Interface<CLambdaLM_Solver> {
protected:
	double m_nu; /**< @brief damping step value */ // reused between nonlinear solver iterations

public:
	/**
	 *	@copydoc CTrustRegion_Interface::f_Error
	 */
	double f_Error(const CLambdaLM_Solver &r_solver)
	{
		return r_solver.f_Chi_Squared_Error_Denorm();
	}

	/**
	 *	@copydoc CTrustRegion_Interface::f_InitialDamping
	 */
	double f_InitialDamping(const typename CLambdaLM_Solver::_TySystem& m_r_system)
	{
		double f_alpha = 0.0;
		m_nu = 2.0;
		const double tau = 1e-3;
		/*for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_last = m_lambda.n_BlockColumn_Block_Num(i) - 1;
			CUberBlockMatrix::_TyMatrixXdRef block = m_lambda.t_BlockAt(n_last, i);
			for(size_t j = 0, m = block.rows(); j < m; ++ j)
				f_alpha = std::max(block(j, j), f_alpha);
		}*/
		for(size_t i = 0, n = m_r_system.n_Edge_Num(); i < n; ++ i) {
			/*typename CLambdaLM_Solver::_TyEdgeMultiPool::_TyConstBaseRef e = m_r_system.r_Edge_Pool()[i]; // mouthful
			f_alpha = std::max(e.f_Max_VertexHessianDiagValue(), f_alpha);*/
			f_alpha = std::max(m_r_system.r_Edge_Pool()[i].f_Max_VertexHessianDiagValue(), f_alpha); // simple
		}

//#define __FIND_BOTTLENECK_EDGE
#ifdef __FIND_BOTTLENECK_EDGE
		double f_average = 0;
		for(size_t i = 0, n = this->m_r_system.n_Edge_Num(); i < n; ++ i)
			f_average += this->m_r_system.r_Edge_Pool()[i].f_Max_VertexHessianDiagValue();
		f_average /= this->m_r_system.n_Edge_Num();
		double f_thresh = f_alpha / 1000000;
		for(size_t i = 0, n = this->m_r_system.n_Edge_Num(); i < n; ++ i) {
			double f_max_diag = this->m_r_system.r_Edge_Pool()[i].f_Max_VertexHessianDiagValue();
			if(f_max_diag > f_thresh) {
				double f_ratio = f_max_diag / f_average;
				fprintf(stderr, "warning: edge " PRIsize " exceeds the average hessian "
					"diagonal value by %d orders of magnitude\n",
					i, int(log(f_ratio) / log(10.0) + .5));
				fprintf(stderr, "warning: edge " PRIsize ": (", i);
				for(size_t j = 0, m = this->m_r_system.r_Edge_Pool()[i].n_Vertex_Num(); j < m; ++ j)
					printf((j)? ", " PRIsize : PRIsize, this->m_r_system.r_Edge_Pool()[i].n_Vertex_Id(j));
				printf(")\n");

				//const_cast<typename CLambdaLM_Solver::_TySystem&>(this->m_r_system).r_Edge_Pool()[i].Calculate_Hessians_v2(); // just for debuggingh purposes, to see how the big number comes into existence
				const_cast<typename CLambdaLM_Solver::_TySystem&>(this->m_r_system).r_Edge_Pool().For_Each(i, i + 1,
					typename CLambdaLM_Solver::_TyLambdaOps::CCalculate_Hessians_v2());
			}
		}
#endif // __FIND_BOTTLENECK_EDGE
		// debug - see where the extreme derivatives are produced

		f_alpha *= tau;
		//f_alpha = 1.44; // copy from g2o for 10khogman

		return f_alpha;
	}

	/**
	 *	@copydoc CTrustRegion_Interface::Aftermath
	 */
	bool Aftermath(double &r_f_last_error, double f_error, double &r_f_alpha, const CUberBlockMatrix &r_lambda,
		const CLambdaLM_Solver &r_solver, const Eigen::VectorXd &r_v_dx, const Eigen::VectorXd &r_v_rhs)
	{
		double rho = (r_f_last_error - f_error) / (r_v_dx.transpose()).dot(r_f_alpha * r_v_dx + r_v_rhs);
		if(rho > 0) {
			r_f_alpha *= std::max(1 / 3.0, 1.0 - pow((2 * rho - 1), 3));
			m_nu = 2;

			r_f_last_error = f_error;
			// step taken: update the last error to this error (not all strategies
			// may want to do this - hence it is a part of TR policy)

			return true; // good step
		} else {
			r_f_alpha *= m_nu; // g2o has ni
			m_nu *= 2;

			return false; // fail
		}
	}

	/**
	 *	@copydoc CTrustRegion_Interface::ApplyDamping
	 */
	void ApplyDamping(CUberBlockMatrix &r_lambda, double f_alpha, size_t n_first_vertex, size_t n_last_vertex)
	{
		_ASSERTE(n_first_vertex <= n_last_vertex);
		_ASSERTE(n_last_vertex <= r_lambda.n_BlockColumn_Num());
		for(size_t i = n_first_vertex; i < n_last_vertex; ++ i) {
			size_t n_last = r_lambda.n_BlockColumn_Block_Num(i) - 1;
			CUberBlockMatrix::_TyMatrixXdRef block = r_lambda.t_Block_AtColumn(i, n_last);
			//for(size_t j = 0, m = block.rows(); j < m; ++ j) // could use unroll (but wouldn't be able to use SSE anyway)
			//	block(j, j) += f_alpha;
			block.diagonal().array() += f_alpha;
		}
	}
};

/**
 *	@brief dummy constant damping LM algorithm (uses the n_dummy_param as damping)
 *	@tparam CLambdaLM_Solver is a specialization of CNonlinearSolver_Lambda_LM
 */
template <class CLambdaLM_Solver>
class CLevenbergMarquardt_ConstDamping : public CTrustRegion_Interface<CLambdaLM_Solver> {
public:
	/**
	 *	@copydoc CTrustRegion_Interface::f_Error
	 */
	double f_Error(const CLambdaLM_Solver &r_solver)
	{
		return r_solver.f_Chi_Squared_Error_Denorm();
	}

	/**
	 *	@copydoc CTrustRegion_Interface::f_InitialDamping
	 */
	double f_InitialDamping(const typename CLambdaLM_Solver::_TySystem& m_r_system)
	{
		return n_dummy_param;
	}

	/**
	 *	@copydoc CTrustRegion_Interface::Aftermath
	 */
	bool Aftermath(double &r_f_last_error, double f_error, double &r_f_alpha, const CUberBlockMatrix &r_lambda,
		const CLambdaLM_Solver &r_solver, const Eigen::VectorXd &r_v_dx, const Eigen::VectorXd &r_v_rhs)
	{
		r_f_last_error = f_error;
		// step taken: update the last error to this error (not all strategies
		// may want to do this - hence it is a part of TR policy)

		return true; // any step is a good step
	}

	/**
	 *	@copydoc CTrustRegion_Interface::ApplyDamping
	 */
	void ApplyDamping(CUberBlockMatrix &r_lambda, double f_alpha, size_t n_first_vertex, size_t n_last_vertex)
	{
		_ASSERTE(n_first_vertex <= n_last_vertex);
		_ASSERTE(n_last_vertex <= r_lambda.n_BlockColumn_Num());
		for(size_t i = n_first_vertex; i < n_last_vertex; ++ i) {
			size_t n_last = r_lambda.n_BlockColumn_Block_Num(i) - 1;
			CUberBlockMatrix::_TyMatrixXdRef block = r_lambda.t_Block_AtColumn(i, n_last);
			//for(size_t j = 0, m = block.rows(); j < m; ++ j) // could use unroll (but wouldn't be able to use SSE anyway)
			//	block(j, j) += f_alpha;
			block.diagonal().array() += f_alpha;
		}
	}
};

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_USE_DUMMY_LM_DAMPING

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
#error "__NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES and __USE_DUMMY_LM_DAMPING are mutually exclusive (they both need n_dummy_param)"
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES

#define CLevenbergMarquardt_DefaultPolicy CLevenbergMarquardt_ConstDamping
// use CLevenbergMarquardt_ConstDamping below instead

#else // __NONLINEAR_SOLVER_LAMBDA_LM_USE_DUMMY_LM_DAMPING
#define CLevenbergMarquardt_DefaultPolicy CLevenbergMarquardt_Baseline
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_USE_DUMMY_LM_DAMPING
// choose the default policy based on the settings // would use using keyword in c++11

/**
 *	@brief nonlinear blocky solver working above the lambda matrix
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix
 *	@tparam CLambdaMatrixBlockSizes is list of block sizes in the information (Hessian) matrix
 *	@tparam CTRAlgorithm is trust region algorithm
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes = typename CSystem::_TyJacobianMatrixBlockList,
	class CLambdaMatrixBlockSizes = typename CSystem::_TyHessianMatrixBlockList,
	template <class> class CTRAlgorithm = CLevenbergMarquardt_DefaultPolicy>
class CNonlinearSolver_Lambda_LM : public nonlinear_detail::CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, true, true> {
public:
	typedef CTRAlgorithm<CNonlinearSolver_Lambda_LM<CSystem, CLinearSolver,
		CAMatrixBlockSizes, CLambdaMatrixBlockSizes, CTRAlgorithm> > _TyTRAlgorithm; /**< @brief trust-region algorithm type */

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
	typedef /*typename*/ CLambdaMatrixBlockSizes/*fbs_ut::CBlockMatrixTypesAfterPreMultiplyWithSelfTranspose<
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
		solver_HasGaussNewton = false, /**< @brief Gauss-Newton support flag */
		solver_HasLevenberg = true, /**< @brief Levenberg-Marquardt support flag */
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

	_TyTRAlgorithm m_TR_algorithm; /**< @brief Levenberg-Marquardt algorithm */

	CUberBlockMatrix m_lambda; /**< @brief the lambda matrix (built / updated incrementally) */
	_TyReductionPlan m_reduction_plan; /**< @brief lambda incremental reduction plan */
	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	Eigen::VectorXd m_v_rhs; /**< @brief right hand side vector */
	Eigen::VectorXd m_v_saved_state; /**< @brief saved state of the vertices (for LM step back) */
	size_t m_n_verts_in_lambda; /**< @brief number of vertices already in lambda */
	size_t m_n_edges_in_lambda; /**< @brief number of edges already in lambda */
	bool m_b_system_dirty; /**< @brief system updated without relinearization flag */
	// solver data

	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	double m_f_chi2_time; /**< @brief time spent in calculating chi2 */
	double m_f_lambda_time; /**< @brief time spent updating lambda */
	double m_f_rhs_time; /**< @brief time spent in right-hand-side calculation */
	double m_f_linsolve_time; /**< @brief time spent in luinear solving (Cholesky / Schur) */
	double m_f_norm_time; /**< @brief time spent in norm calculation section */
	double m_f_damping_time; /**< @brief time spent in calculating initial damping */
	double m_f_dampingupd_time; /**< @brief time spent in updating the damping */
	double m_f_sysupdate_time; /**< @brief time spent in norm calculation section */
	double m_f_extra_chol_time; /**< @brief time spent in calculating extra Cholesky factorization for marginal covariances */
	double m_f_marginals_time; /**< @brief time spent in calculating marginal covariances (batch) */
	double m_f_incmarginals_time; /**< @brief time spent in calculating marginal covariances (update) */
	size_t m_n_incmarginals_num; /**< @brief number of times the marginals update ran instead of batch recalculation */
	// statistics

	typedef typename CLinearSolver_Schur<CLinearSolver, CAMatrixBlockSizes, CSystem>::_TyGOH _TyGOH; /**< @brief guided Schur ordering helper */

	typedef typename _TyGOH::template CBlockSizes<_TyLambdaMatrixBlockSizes> _TySchurBlockSize_Helper;
	typedef typename _TySchurBlockSize_Helper::_TyDBlockSizes _TyDBlockSizes; /**< @brief diagonal (landmark) matrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TySchurBlockSizes _TySchurBlockSizes; /**< @brief Schur complement (RCS) matrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TyUBlockSizes _TyUBlockSizes; /**< @brief upper off-diagonal submatrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TyVBlockSizes _TyVBlockSizes; /**< @brief lower off-diagonal submatrix block sizes */
	typedef typename _TySchurBlockSize_Helper::_TyOffDiagBlockSizes _TyOffDiagBlockSizes; /**< @brief off-diagonal submatrix block sizes */

	typedef CSchurComplement_Marginals<_TySchurBlockSizes, _TyUBlockSizes, _TyVBlockSizes, _TyDBlockSizes> CMargsHelper;

	CMargsHelper m_margs; /**< @brief Schur marginals helper object */

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
	CNonlinearSolver_Lambda_LM(CSystem &r_system, size_t n_linear_solve_threshold,
		size_t n_nonlinear_solve_threshold, size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = true)
		:_TyBase(r_system, n_linear_solve_threshold,
		n_nonlinear_solve_threshold, n_nonlinear_solve_max_iteration_num,
		f_nonlinear_solve_error_threshold, b_verbose, linear_solver, b_use_schur),
		/*m_schur_solver(linear_solver),*/ m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
		/*m_r_system(r_system), m_linear_solver(linear_solver),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_b_verbose(b_verbose),*/ /*m_b_use_schur(b_use_schur), m_n_real_step(0),*/ m_b_system_dirty(false),
		m_n_iteration_num(0), m_f_chi2_time(0), m_f_lambda_time(0), m_f_rhs_time(0), m_f_linsolve_time(0),
		m_f_norm_time(0), m_f_damping_time(0), m_f_dampingupd_time(0), m_f_sysupdate_time(0),
		/*m_b_had_loop_closure(false),*/ m_f_extra_chol_time(0), m_f_marginals_time(0),
		m_f_incmarginals_time(0), m_n_incmarginals_num(0)
	{
		/*m_t_incremental_config.t_linear_freq.n_period = n_linear_solve_threshold;
		m_t_incremental_config.t_nonlinear_freq.n_period = n_nonlinear_solve_threshold;
		m_t_incremental_config.n_max_nonlinear_iteration_num = n_nonlinear_solve_max_iteration_num;
		m_t_incremental_config.f_nonlinear_error_thresh = f_nonlinear_solve_error_threshold;
		// don't have simultaneously linear and nonlinear solve constructor

		_ASSERTE(!m_t_incremental_config.t_nonlinear_freq.n_period ||
			!m_t_incremental_config.t_linear_freq.n_period);*/ // only one of those

#ifdef  __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
		fprintf(stderr, "warning: running with __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES, results may be imprecise\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
#ifdef  __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES
		fprintf(stderr, "warning: running with __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES, may run slow and guff\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES
#ifdef  __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
		fprintf(stderr, "warning: running with __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES, will run slow & guff\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
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
	CNonlinearSolver_Lambda_LM(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = true)
		:_TyBase(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur), /*m_r_system(r_system), m_linear_solver(linear_solver),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_t_incremental_config(t_incremental_config),
		m_b_verbose(b_verbose),*/
		/*m_schur_solver(linear_solver),*/ m_n_verts_in_lambda(0), m_n_edges_in_lambda(0),
		/*m_b_use_schur(b_use_schur), m_n_real_step(0),*/ m_b_system_dirty(false),
		m_n_iteration_num(0), m_f_chi2_time(0), m_f_lambda_time(0), m_f_rhs_time(0),
		m_f_linsolve_time(0), m_f_norm_time(0), m_f_damping_time(0), m_f_dampingupd_time(0),
		m_f_sysupdate_time(0), /*m_b_had_loop_closure(false), m_t_marginals_config(t_marginals_config),*/
		m_f_extra_chol_time(0), m_f_marginals_time(0), m_f_incmarginals_time(0), m_n_incmarginals_num(0)
	{
		/*_ASSERTE(!m_t_incremental_config.t_nonlinear_freq.n_period ||
			!m_t_incremental_config.t_linear_freq.n_period); // only one of those*/

		/*if(t_marginals_config.b_calculate) // not supported at the moment
			throw std::runtime_error("CNonlinearSolver_Lambda_LM does not support marginals calculation");*/

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

#ifdef  __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
		fprintf(stderr, "warning: running with __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES, results may be imprecise\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
#ifdef  __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES
		fprintf(stderr, "warning: running with __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES, may run slow and guff\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES
#ifdef  __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
		fprintf(stderr, "warning: running with __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES, will run slow & guff\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
		double f_parallel_time = m_f_lambda_time + m_f_rhs_time;
		double f_all_time = f_parallel_time + m_f_chi2_time + m_f_linsolve_time -
			m_f_norm_time + m_f_damping_time + m_f_dampingupd_time + m_f_sysupdate_time +
			m_f_extra_chol_time + m_f_marginals_time + m_f_incmarginals_time;
		if(f_total_time < 0)
			printf("solver took %f seconds in total\n", f_all_time);
		printf("solver spent %f seconds in parallelizable section (updating lambda; disparity %g seconds)\n",
			f_parallel_time, (f_total_time > 0)? f_total_time - f_all_time : 0);
		printf("out of which:\n");
		printf("\tlambda: %f\n", m_f_lambda_time);
		printf("\t   rhs: %f\n", m_f_rhs_time);
		if(this->m_t_marginals_config.b_calculate) {
			printf("solver spent %f seconds in marginals\n"
				"\t chol: %f\n"
				"\tmargs: %f\n"
				"\t incm: %f (ran " PRIsize " times)\n",
				m_f_extra_chol_time + m_f_marginals_time + m_f_incmarginals_time,
				m_f_extra_chol_time, m_f_marginals_time,
				m_f_incmarginals_time, m_n_incmarginals_num);
			m_margs.Dump();
		}
		printf("solver spent %f seconds in serial section\n", f_all_time -
			f_parallel_time - (m_f_extra_chol_time + m_f_marginals_time + m_f_incmarginals_time));
		printf("out of which:\n");
		printf("\t  chi2: %f\n", m_f_chi2_time);
		printf("\t  damp: %f\n", m_f_damping_time);
		printf("\tlinsol: %f\n", m_f_linsolve_time);
		printf("\t  norm: %f\n", m_f_norm_time);
		printf("\tsysupd: %f\n", m_f_sysupdate_time);
		printf("\tdamupd: %f\n", m_f_dampingupd_time);
	}

	/**
	 *	@brief gets the current system matrix
	 *	@return Returns const reference to the current system matrix (without damping).
	 *	@note This function throws std::bad_alloc.
	 */
	inline const CUberBlockMatrix &r_Lambda() // throw(std::bad_alloc)
	{
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

		return m_lambda;
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
		/*size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_had_loop_closure) {
			_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
			typename _TyEdgeMultiPool::_TyConstBaseRef r_edge =
				m_r_system.r_Edge_Pool()[m_r_system.r_Edge_Pool().n_Size() - 1];
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

		bool b_new_vert = false, b_ran_opt = false;

		++ m_n_real_step;

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_vertex_num = n_vertex_num - m_n_last_optimized_vertex_num;
		// the optimization periods are counted in vertices, not in edges (should save work on 10k, 100k)
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		++ m_n_step;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		if(m_t_incremental_config.t_nonlinear_freq.n_period && n_new_vertex_num >= m_t_incremental_config.t_nonlinear_freq.n_period) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		if(m_t_incremental_config.t_nonlinear_freq.n_period && m_n_step == m_t_incremental_config.t_nonlinear_freq.n_period) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure) {
				b_ran_opt = true;
				m_b_had_loop_closure = false;
				Optimize(m_t_incremental_config.n_max_nonlinear_iteration_num, m_t_incremental_config.f_nonlinear_error_thresh);
			}
			// nonlinear optimization

			b_new_vert = true;
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_t_incremental_config.t_linear_freq.n_period && n_new_vertex_num >= m_t_incremental_config.t_linear_freq.n_period) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_t_incremental_config.t_linear_freq.n_period && m_n_step % m_t_incremental_config.t_linear_freq.n_period == 0) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			_ASSERTE(!m_t_incremental_config.t_nonlinear_freq.n_period);
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			if(!m_t_incremental_config.t_nonlinear_freq.n_period)
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

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
		if(b_new_vert) {
			FILE *p_fw = fopen("chi2perVert.txt", (m_n_real_step > 0)? "a" : "w");
			fprintf(p_fw, "%f\n", f_Chi_Squared_Error_Denorm());
			fclose(p_fw);
		}
		// dump chi2
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
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

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
		CUberBlockMatrix lambda_prev = m_lambda;
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES

		_TyLambdaOps::Extend_Lambda(this->m_r_system, m_reduction_plan, m_lambda,
			m_n_verts_in_lambda, m_n_edges_in_lambda); // recalculated all the jacobians inside Extend_Lambda()
		m_v_dx.resize(n_variables_size, 1);
		m_v_saved_state.resize(n_variables_size, 1);
		// allocate more memory

		if(!m_b_system_dirty)
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda, 0, m_n_edges_in_lambda); // calculate only for new edges // @todo - but how to mark affected vertices? // simple test if edge id is greater than m_n_edges_in_lambda, the vertex needs to be recalculated
		else
			_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda); // calculate for entire system
		//m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
		//m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // not yet, the damping was not applied
		//m_b_system_dirty = false; // cannot do that yet, the damping was not applied
		// lambda is (partially) updated, but (partially) without damping - don't give it to m_TR_algorithm as it would quite increase complexity

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES
		{
			char p_s_diff_filename[256];
			static size_t n_lambda_step = 0;
			sprintf(p_s_diff_filename, "lambda_diff_%03" _PRIsize ".tga", n_lambda_step);
			++ n_lambda_step;
			m_lambda.Rasterize(lambda_prev, false, p_s_diff_filename);
			CUberBlockMatrix empty;
			lambda_prev.Swap(empty); // free the memory
		}
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_DIFFERENCE_MATRICES

		timer.Accum_DiffSample(m_f_lambda_time);

		if(m_lambda.n_BlockColumn_Num() < this->m_r_system.r_Vertex_Pool().n_Size()) {
			fprintf(stderr, "warning: waiting for more edges\n");
			m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
			m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // probably necessary here
			m_b_system_dirty = true; // to correctly adjust damping to the whole matrix
			return;
		}
		// waiting for more edges

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
		printf("warning: landmark settle iterations enabled\n");
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS

		double f_alpha = m_TR_algorithm.f_InitialDamping(this->m_r_system); // lambda is not ready at this point yet, can only use the edges
		if(this->m_b_verbose) {
			printf("alpha: %f\n", f_alpha);

			/*std::vector<float> pix_err(this->m_r_system.r_Edge_Pool().n_Size());
			for(size_t i = 0, n = this->m_r_system.r_Edge_Pool().n_Size(); i < n; ++ i) {
				//const CEdgeP2C3D &r_edge = this->m_r_system.r_Edge_Pool().r_At<CEdgeP2C3D>(i);
				pix_err[i] = float(this->m_r_system.r_Edge_Pool().r_At<_TyBaseEdge>(i).f_Reprojection_Error());
			}
			size_t n_med = pix_err.size() / 2;
			std::nth_element(pix_err.begin(), pix_err.begin() + n_med, pix_err.end());
			printf("median reprojection error: %.2f px\n", pix_err[n_med]);*/
			// debug - print median reprojection error
		}

		//double f_errorx = m_TR_algorithm.f_Error(*this); // this is called for no good reason
		//printf("init chi: %f\n", f_errorx);

		timer.Accum_DiffSample(m_f_damping_time);

		if(!m_b_system_dirty)
			Apply_Damping(0, m_n_edges_in_lambda, f_alpha); // calculate only for new edges // t_odo - but how to mark affected vertices? // done inside
		else
			Apply_Damping(0, 0, f_alpha); // for the entire system

		m_b_system_dirty = false;
		m_n_verts_in_lambda = this->m_r_system.r_Vertex_Pool().n_Size();
		m_n_edges_in_lambda = this->m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
		_ASSERTE(m_lambda.n_Row_Num() == m_lambda.n_Column_Num() &&
			m_lambda.n_BlockColumn_Num() == this->m_r_system.r_Vertex_Pool().n_Size() &&
			m_lambda.n_Column_Num() == n_variables_size); // lambda is square, blocks on either side = number of vertices
		// need to have lambda

		timer.Accum_DiffSample(m_f_lambda_time);

		double f_last_error = m_TR_algorithm.f_Error(*this);
		// this may not be called until lambda is finished

		timer.Accum_DiffSample(m_f_chi2_time);

		if(this->m_b_verbose) {
			printf("initial chi2: %f\n", f_last_error);

			size_t n_sys_size = this->m_r_system.n_Allocation_Size();
			size_t n_rp_size = m_reduction_plan.n_Allocation_Size();
			size_t n_lam_size = m_lambda.n_Allocation_Size();
			printf("memory_use(sys: %.2f MB, redplan: %.2f MB, ,\\: %.2f MB)\n",
				n_sys_size / 1048576.0, n_rp_size / 1048576.0, n_lam_size / 1048576.0);
		}
		// print memory use statistics

		int fail = 10;
#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
		for(size_t n_iteration = 0; n_iteration < n_max_iteration_num * 4; ++ n_iteration) {
#else // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
		for(size_t n_iteration = 0; n_iteration < n_max_iteration_num; ++ n_iteration) {
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
			++ m_n_iteration_num;
			// debug

			if(this->m_b_verbose) {
				if(n_max_iteration_num == 1)
					printf("\n=== incremental optimization step ===\n\n");
				else {
#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
					if(!(n_iteration % 4))
						printf("\n=== nonlinear optimization: iter #" PRIsize " ===\n\n", n_iteration / 4);
					else {
						printf("\n=== nonlinear optimization: iter #" PRIsize
							", settle iter #" PRIsize " ===\n\n", n_iteration / 4, n_iteration % 4);
					}
#else // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
					printf("\n=== nonlinear optimization: iter #" PRIsize " ===\n\n", n_iteration);
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
				}
			}
			// verbose

			if(n_iteration) {
				if(m_b_system_dirty)
					_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda, 0, 0); // refresh all
				else
					m_reduction_plan.r_Lambda_ReductionPlan().ReduceAll(); // update all, to remove damping from the diagonal. hessians are not recalculated
				m_b_system_dirty = true; // for the sake of Apply_Damping() debug checks - we now did a full system refresh, that means it had to be dirty
				Apply_Damping(0, 0, f_alpha); // damp all, even if the system is not dirty
				m_b_system_dirty = false;

				timer.Accum_DiffSample(m_f_lambda_time);
			}
			// no need to rebuild lambda, just refresh the values that are being referenced

			_TyLambdaOps::Collect_RightHandSide_Vector(this->m_r_system, m_reduction_plan, m_v_dx);
			// collects the right-hand side vector

			m_v_rhs = m_v_dx; // copy intended
			// save the original rhs for step estimation

			timer.Accum_DiffSample(m_f_rhs_time);

#if 0 && defined(_DEBUG)
			AssertDamping(f_alpha);
#endif // _DEBUG

			bool b_cholesky_result = LinearSolve(n_iteration, n_max_iteration_num);
			// calculate cholesky, reuse block ordering if the linear solver supports it

			timer.Accum_DiffSample(m_f_linsolve_time);

			if(!b_cholesky_result)
				break;
			// in case cholesky failed, quit

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES
			if(n_max_iteration_num == 1) { // in case this is an incremental solve step, dump the dx vector
				static size_t n_iter_num = 0;
				++ n_iter_num;
				char p_s_filename[256];
				sprintf(p_s_filename, "dx_iter_%05" _PRIsize "_%02" _PRIsize ".txt", n_iter_num, n_iteration + 1);
				FILE *p_fw = fopen(p_s_filename, "w");
				for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
					size_t n_first = m_lambda.n_BlockColumn_Base(i);
					size_t n_DOF = m_lambda.n_BlockColumn_Column_Num(i);
					size_t n_last = n_first + n_DOF;
					// see where the variable is

					for(size_t j = n_first; j < n_last; ++ j)
						fprintf(p_fw, ((fabs(m_v_dx(j)) > 1)? " %f" : " %g") + (j == n_first), m_v_dx(j));
					fprintf(p_fw, "\n");
					// put each variable on a separate line
				}
				fprintf(p_fw, "\n");
				fclose(p_fw);
			}
			// debug - dump the dx vectors to see the ranks as the solution evolves
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_INCREMENTAL_UPDATES

			double f_residual_norm = 0;
			if(b_cholesky_result) {
				f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
				if(this->m_b_verbose)
					printf("residual norm: %.4f\n", f_residual_norm);

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
				if(n_dummy_param < 0) {
					double f_thresh = pow(10.0, double(n_dummy_param));
					static bool b_dp_warned = false;
					if(!b_dp_warned) {
						fprintf(stderr, "debug: thresholding dx elements with %g\n", f_thresh);
						b_dp_warned = true;
					}
					// calculate / announce the thresh

					size_t n_zeroed_num = 0;
					for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
						size_t n_first = m_lambda.n_BlockColumn_Base(i);
						size_t n_DOF = m_lambda.n_BlockColumn_Column_Num(i);
						size_t n_last = n_first + n_DOF;
						// see where the variable is

						bool b_zero = true;
						for(size_t j = n_first; j < n_last; ++ j) {
							if(fabs(m_v_dx(j)) > f_thresh) {
								b_zero = false;
								break;
							}
						}
						// see whether we should zero it

						if(b_zero) {
							++ n_zeroed_num;
							m_v_dx.segment(n_first, n_DOF).setZero();
						}
						// zero it
					}
					// threshold the vector

					if(this->m_b_verbose) {
						printf("threshold killed update on " PRIsize " / " PRIsize " variables\n", n_zeroed_num, m_lambda.n_BlockColumn_Num());
						printf("residual norm: %.4f\n", m_v_dx.norm());
					}
					// verbose
				}
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_THRESHOLD_UPDATES
			}

			// calculate residual norm

			timer.Accum_DiffSample(m_f_norm_time);
			// timing breakup

			if(f_residual_norm <= f_min_dx_norm/* && n_iteration % 4 == 0*/)
				break;
			// in case the error is low enough, quit (saves us recalculating the hessians)

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
			if(n_iteration % 4 == 0) // save state at the beginning!
				nonlinear_detail::CSolverOps_Base::Save_State(this->m_r_system, m_v_saved_state);
#else // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
			nonlinear_detail::CSolverOps_Base::Save_State(this->m_r_system, m_v_saved_state);
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
			// god save the vertices

			nonlinear_detail::CSolverOps_Base::PushValuesInGraphSystem(this->m_r_system, m_v_dx);
			// update the system (in parallel)

			timer.Accum_DiffSample(m_f_sysupdate_time);

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
			if(n_iteration % 4 < 3) { // evaluate chi2 at the end of the relaxation (this is maybe wrong, maybe a bad step is caught unnecessarily late)
				m_b_system_dirty = true;
				continue;
			}
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS

			double f_error = m_TR_algorithm.f_Error(*this);
			if(this->m_b_verbose) {
				printf("chi2: %f\n", f_error);

				/*std::vector<float> pix_err(this->m_r_system.r_Edge_Pool().n_Size());
				for(size_t i = 0, n = this->m_r_system.r_Edge_Pool().n_Size(); i < n; ++ i) {
					//const CEdgeP2C3D &r_edge = this->m_r_system.r_Edge_Pool().r_At<CEdgeP2C3D>(i);
					pix_err[i] = float(this->m_r_system.r_Edge_Pool().r_At<_TyBaseEdge>(i).f_Reprojection_Error());
				}
				size_t n_med = pix_err.size() / 2;
				std::nth_element(pix_err.begin(), pix_err.begin() + n_med, pix_err.end());
				printf("median reprojection error: %.2f px\n", pix_err[n_med]);
				// debug - print median reprojection error*/
			}

			timer.Accum_DiffSample(m_f_chi2_time);

			bool b_good_step = m_TR_algorithm.Aftermath(f_last_error, f_error, f_alpha, m_lambda, *this, m_v_dx, m_v_rhs);
			if(!b_good_step) {
				fprintf(stderr, "warning: chi2 rising\n");
				// verbose

				nonlinear_detail::CSolverOps_Base::Load_State(this->m_r_system, m_v_saved_state);
				// restore saved vertives

				if(fail > 0) {
					-- fail;
					++ n_max_iteration_num;
				}
				// increase the number of iterations, up to a certain limit
			} else {
				this->m_marginals.DisableUpdate(); // linearization point just changed, all the marginals will change - need full recalc
				m_b_system_dirty = true; // !!
			}

			timer.Accum_DiffSample(m_f_dampingupd_time);
			//m_b_system_dirty = true; // even though we saved state, we need to change alpha
		}
		// optimize the system

		if(this->m_t_marginals_config.b_calculate) {
			bool b_batch = !this->m_t_incremental_config.t_linear_freq.n_period &&
				!this->m_t_incremental_config.t_nonlinear_freq.n_period;
			// are we running batch?

			if(b_batch) {
				this->m_linear_solver.Free_Memory();
				this->m_schur_solver.Free_Memory();
			}
			// unable to reuse these, free memory

			if(this->m_b_verbose && b_batch)
				printf("\n=== calculating marginals ===\n\n");
			// todo - handle freq settings
			// todo - handle policies

			if(this->m_b_verbose && b_batch)
				printf("refreshing lambda with null damping\n");

			//m_b_system_dirty = true;
			if(f_alpha != 0 || m_b_system_dirty) {
				f_alpha = 0; // !! otherwise the marginals are something else
				_TyLambdaOps::Refresh_Lambda(this->m_r_system, m_reduction_plan, m_lambda, 0, 0);
				m_b_system_dirty = true; // for the sake of Apply_Damping() debug checks - we now did a full system refresh, that means it had to be dirty
				Apply_Damping(0, 0, f_alpha);
				m_b_system_dirty = false; // this is ok now, can save some time

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

#if 0 && defined(_DEBUG)
			AssertDamping(0);
#endif // _DEBUG

			if(this->m_b_use_schur &&
			   this->m_t_marginals_config.n_relinearize_policy == mpart_Diagonal) {
				if(this->m_b_verbose && b_batch)
					printf("calculating Schur ordering\n");

				size_t p_vertex_dimensions[_TyGOH::n_vertex_group_num];
				if(!_TyGOH::b_can_use_simple_ordering) {
					_TyGOH::Get_VertexDimensions(p_vertex_dimensions,
						sizeof(p_vertex_dimensions) / sizeof(p_vertex_dimensions[0]));
				}
				// gather dimensions of partite vertex types

				size_t p_vertex_dimension_sum[_TyGOH::n_vertex_group_num] = {0};
				// sum of dimensions of all the vertex types

				std::vector<size_t> p_vertex_type_indices[_TyGOH::n_vertex_group_num]; // the first list is for the non-partite vertices, if there are any
				// the same size as p_vertex_dimensions, except that they are ordered in the opposite direction

				std::vector<size_t> schur_ordering, new_rcs_vertices;
				size_t n_landmark_group_id = 0, n_matrix_cut = 0;
				for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
					size_t n_dim = m_lambda.n_BlockColumn_Column_Num(i);
					if(_TyGOH::b_can_use_simple_ordering) {
						enum {
							n_landmark_dim = _TyGOH::n_smallest_vertex_dim,
							n_pose_dim = _TyGOH::n_greatest_vertex_dim
						};
						if(n_dim == n_landmark_dim)
							schur_ordering.push_back(i); // this will be in the diagonal part
						else
							new_rcs_vertices.push_back(i); // this will be in the RCS part
						// simple case
					} else {
						size_t n_group = _TyGOH::n_Vertex_GroupIndex(n_dim);
						p_vertex_dimension_sum[n_group] += n_dim;
						if(n_landmark_group_id == n_group)
							schur_ordering.push_back(i); // this will be in the diagonal part
						else
							new_rcs_vertices.push_back(i); // this will be in the RCS part
						// case with groups
					}
				}
				// for all new vertices

				if(!_TyGOH::b_can_use_simple_ordering) {
					size_t n_best_group_size = p_vertex_dimension_sum[n_landmark_group_id];
					size_t n_best_group_id = n_landmark_group_id;
					for(int i = _TyGOH::b_have_nonpartite_vertex_types; i < _TyGOH::n_vertex_group_num; ++ i) {
						if(n_best_group_size < p_vertex_dimension_sum[i]) {
							n_best_group_size = p_vertex_dimension_sum[i];
							n_best_group_id = i;
						}
					}
					if(n_best_group_id != n_landmark_group_id) {
						//throw std::runtime_error("not implemented");
						fprintf(stderr, "warning: using untested SC reordering branch\n"); // just so that we know that this takes place // todo - test this (will probably need a special graph to do that)

						//b_reordering_needed = true;
						n_landmark_group_id = n_best_group_id;

						n_matrix_cut = 0;
						schur_ordering.clear();
						new_rcs_vertices.clear();
						for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
							size_t n_dim = m_lambda.n_BlockColumn_Column_Num(i);
							size_t n_group = _TyGOH::n_Vertex_GroupIndex(n_dim);
							if(n_landmark_group_id == n_group)
								schur_ordering.push_back(i); // this will be in the diagonal part
							else
								new_rcs_vertices.push_back(i); // this will be in the RCS part
							// case with groups
						}
						// reorder all vertices
					}
				}
				// see if reordering is needed

				schur_ordering.insert(schur_ordering.begin(),
					new_rcs_vertices.begin(), new_rcs_vertices.end());
				n_matrix_cut = new_rcs_vertices.size();
				// finalize the SC ordering

				CMatrixOrdering mord_sc;
				const size_t *p_schur_order = mord_sc.p_InvertOrdering(&schur_ordering.front(),
					schur_ordering.size());
				const size_t n_cut = n_matrix_cut, n_size = schur_ordering.size();
				// invert the ordering

				CUberBlockMatrix lambda_perm;
				m_lambda.Permute_UpperTriangular_To(lambda_perm, p_schur_order, n_size, true);
				// permute lambda

				CUberBlockMatrix SC, minus_Dinv, newU;
				{
					CUberBlockMatrix /*newU,*/ newV, newD;
					CUberBlockMatrix &newA = SC; // will calculate that inplace
					lambda_perm.SliceTo(newA, 0, n_cut, 0, n_cut, false); // copy!! don't destroy lambda
					lambda_perm.SliceTo(newU, 0, n_cut, n_cut, n_size, false); // do not share data, will need to output this even after lambda perm perishes
					lambda_perm.SliceTo(newD, n_cut, n_size, n_cut, n_size, true);
					newV.TransposeOf(newU);
					// get the news (could calculate them by addition in case we choose to abandon lambda altogether)

					_ASSERTE(newD.b_BlockDiagonal()); // make sure it is block diagonal
					// the algorithms below wont work if D isnt a diagonal matrix

					CUberBlockMatrix &minus_newDinv = minus_Dinv;
					minus_newDinv.InverseOf_Symmteric_FBS<_TyDBlockSizes>(newD); // batch inverse, dont reuse incremental!
					minus_newDinv.Scale(-1);

					CUberBlockMatrix minus_newU_newDinv, minus_newU_newDinv_newV;
					minus_newU_newDinv.ProductOf_FBS<_TyUBlockSizes, _TyDBlockSizes>(newU, minus_newDinv);
					minus_newU_newDinv_newV.ProductOf_FBS<_TyUBlockSizes, _TyVBlockSizes>(minus_newU_newDinv, newV, true);

					CUberBlockMatrix &newSC_ref = newA;
					minus_newU_newDinv_newV.AddTo_FBS<_TySchurBlockSizes>(newSC_ref);
					// calculate batch SC
				}

				const CUberBlockMatrix &U = newU;

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
					m_f_extra_chol_time/*m_f_UDinv_time*/ += f_UDinv_time; // no UDinv here, just account it to xchol

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
					margs_all.Permute_UpperTriangular_To(r_m, &schur_ordering[0],
						schur_ordering.size(), false); // no share! the original will be deleted
					this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
					// take care of having the correct permutation there

					//r_m.Rasterize("scmargs_03_unperm.tga");

					this->m_marginals.EnableUpdate();
					// now the marginals are current and can be updated until the linearization point is changed again
				}

				this->m_marginals.Set_Edge_Num(this->m_r_system.r_Edge_Pool().n_Size());
				// now all those edges are in the marginals

				timer.Accum_DiffSample(m_f_marginals_time);
			} else {
				if(this->m_b_use_schur) {
					_ASSERTE(this->m_t_marginals_config.n_relinearize_policy != mpart_Diagonal); // that's why we're in this branch
					static bool b_warned = false;
					if(!b_warned) {
						b_warned = true;
						fprintf(stderr, "warning: could use fast Schur marginals but at the"
							" moment those only implement recovery of mpart_Diagonal\n");
					}
				}
				if(this->m_b_verbose && b_batch)
					printf("calculating fill-reducing ordering\n");

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
				{
					CUberBlockMatrix lambda_perm; // not needed afterwards

					if(this->m_b_verbose && b_batch)
						printf("forming lambda perm\n");

					m_lambda.Permute_UpperTriangular_To(lambda_perm, p_order, mord.n_Ordering_Size(), true);

					if(this->m_b_verbose && b_batch)
						printf("calculating Cholesky\n");

					if(!R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm))
						throw std::runtime_error("fatal error: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed");
					// note that now the marginals are calculated with ordering: need to count with that, otherwise those are useless!

					if(this->m_b_verbose && b_batch)
						printf("memory_use(R: %.2f MB)\n", R.n_Allocation_Size() / 1048576.0);
					// verbose
				}
				if(this->m_b_verbose && b_batch)
					printf("calculating marginals (%s)\n", (this->m_marginals.b_CanUpdate())? "incrementally" : "batch");

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
						mord, this->m_t_marginals_config.n_relinearize_policy, false);
					// calculate the thing

					{
						CUberBlockMatrix empty;
						R.Swap(empty);
					}
					// delete R, don't need it and it eats a lot of memory

					if(this->m_b_verbose && b_batch)
						printf("reordering the marginals\n");

					CUberBlockMatrix &r_m = const_cast<CUberBlockMatrix&>(this->m_marginals.r_SparseMatrix()); // watch out, need to call Swap_SparseMatrix() afterwards
					margs_ordered.Permute_UpperTriangular_To(r_m, mord.p_Get_Ordering(),
						mord.n_Ordering_Size(), false); // no share! the original will be deleted
					this->m_marginals.Swap_SparseMatrix(r_m); // now the marginals know that the matrix changed
					// take care of having the correct permutation there

					this->m_marginals.EnableUpdate();
					// now the marginals are current and can be updated until the linearization point is changed again

					timer.Accum_DiffSample(m_f_marginals_time);
				}
			}

			this->m_marginals.Set_Edge_Num(this->m_r_system.r_Edge_Pool().n_Size());
			// now all those edges are in the marginals

			/*FILE *p_fw;
			if((p_fw = fopen("marginals.txt", "w"))) {
				for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
					size_t n_order = m_lambda.n_BlockColumn_Base(i);
					size_t n_dimension = m_lambda.n_BlockColumn_Column_Num(i);
					// get col

					CUberBlockMatrix::_TyConstMatrixXdRef block =
						this->m_marginals.r_SparseMatrix().t_FindBlock(n_order, n_order);
					// get block

					//fprintf(p_fw, "block_%d_%d = ", int(i), int(i));
					//CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, block);
					// prints the matrix

					_ASSERTE(block.rows() == block.cols() && block.cols() == n_dimension);
					for(size_t i = 0; i < n_dimension; ++ i)
						fprintf(p_fw, (i)? " %lf" : "%lf", block(i, i));
					fprintf(p_fw, "\n");
					// print just the diagonal, one line per every vertex
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
	 *	@brief solves m_lambda \ m_v_dx, result left in m_v_dx
	 *
	 *	@param[in] n_iteration is nonlinear solver iteration (zero-based index)
	 *	@param[in] n_max_iteration_num is maximum nonlinear iteration count
	 *
	 *	@return Returns true if the factorization succeeded, otherwise returns false.
	 */
	bool LinearSolve(size_t n_iteration, size_t n_max_iteration_num)
	{
		bool b_cholesky_result;
		{
			Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta

			if(!this->m_b_use_schur) { // use cholesky
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
				bool b_marginalize_out_poses = false;
				// set to true to optimize only landmarks

#ifdef __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS
				b_marginalize_out_poses = true;
				if(n_iteration % 4 == 0)
					b_marginalize_out_poses = false;
#endif // __NONLINEAR_SOLVER_LAMBDA_LM_LANDMARK_SETTLE_ITERATIONS

				if(!n_iteration) {
					bool b_force_guided_ordering = b_marginalize_out_poses; // or set this to true to make sure that it will be the poses and not landmarks or a mixture thereof that will be marginalized out in Schur_Solve_MarginalPoses()
					this->m_schur_solver.SymbolicDecomposition_Blocky(m_lambda, b_force_guided_ordering);
				}
				// calculate the ordering once, it does not change

				if(!b_marginalize_out_poses)
					b_cholesky_result = this->m_schur_solver.Solve_PosDef_Blocky(m_lambda, v_eta); // as usual
				else
					b_cholesky_result = this->m_schur_solver.Solve_PosDef_Blocky_MarginalPoses(m_lambda, v_eta); // then m_v_dx contains change only for the landmarks and the poses remain constant
				// Schur
			}

			if(this->m_b_verbose) {
				printf("%s %s", (this->m_b_use_schur)? "Schur" : "Cholesky",
					(b_cholesky_result)? "succeeded\n" : "failed\n");
			}

#ifdef _DEBUG
			nonlinear_detail::CSolverOps_Base::b_DetectNaNs(m_v_dx, true, "dx");
#endif // _DEBUG
		}
		// calculate cholesky, reuse block ordering if the linear solver supports it

		return b_cholesky_result;
	}

#ifdef _DEBUG
	/**
	 *	@brief makes sure that the damping on the diagonal of lambda is as expected
	 *	@param[in] f_expected_damping is the expected value of damping
	 *	@note This is available in the debug mode only.
	 */
	void AssertDamping(double f_expected_damping)
	{
		Eigen::VectorXd v_lambda_diagonal(m_lambda.n_Column_Num());
		/*for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			_ASSERTE(m_lambda.n_BlockColumn_Block_Num(i)); // not rank deficient
			size_t n_last = m_lambda.n_BlockColumn_Block_Num(i) - 1;
			CUberBlockMatrix::_TyMatrixXdRef block = m_lambda.t_Block_AtColumn(i, n_last);
			size_t n_org = m_lambda.n_BlockColumn_Base(i);
			v_lambda_diagonal.segment(n_org, block.rows()) = block.diagonal();
		}*/
		m_lambda.Get_Diagonal(v_lambda_diagonal);
		// go through lambda and get its diagonal // t_odo - this could be a function

		m_reduction_plan.r_Lambda_ReductionPlan().ReduceAll();
		// refresh everything (without recalculating jacobians)

		for(size_t i = 0, n = m_lambda.n_BlockColumn_Num(); i < n; ++ i) {
			_ASSERTE(m_lambda.n_BlockColumn_Block_Num(i)); // not rank deficient
			size_t n_last = m_lambda.n_BlockColumn_Block_Num(i) - 1;
			CUberBlockMatrix::_TyMatrixXdRef block = m_lambda.t_Block_AtColumn(i, n_last);
			size_t n_org = m_lambda.n_BlockColumn_Base(i);
			Eigen::VectorBlock<Eigen::VectorXd> v_diag = v_lambda_diagonal.segment(n_org, block.rows());
			for(size_t j = 0, m = block.rows(); j < m; ++ j) {
				double f_clean = block(j, j);
				double f_damped = v_diag(j);
				double f_damped_again = f_clean + f_expected_damping;

				_ASSERTE(fabs(f_damped - f_damped_again) < 1e-15); // should be zero since it is always calculated the same, roundoff in addition should be the same in both cases so it does not matter
				// make sure the damping is correct

				block(j, j) = f_damped;
			}
		}
		// go through lambda again, make sure that the damping was correct, restore the damped diagonal
	}
#endif // _DEBUG

	/**
	 *	@brief calls the TR algorithm to apply damping to the lambda matrix
	 *
	 *	@param[in] n_refresh_from_vertex is zero-based index of the first vertex to refresh (unused)
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge to refresh
	 *	@param[in] f_alpha is value of the damping factor
	 *
	 *	@note The damping is applied incrementally.
	 */
	void Apply_Damping(size_t n_refresh_from_vertex, size_t n_refresh_from_edge, double f_alpha)
	{
		_ASSERTE(!n_refresh_from_vertex); // always zero

#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
		if(n_refresh_from_edge) {
			_ASSERTE(!m_b_system_dirty);
			// if refreshing incrementally, it means that lambda was only partially refreshed
			// and the diagonal is (partially) still modified by the alpha from previous step(s)

			//fprintf(stderr, "debug: incremental damping update\n");

			m_reduction_plan.r_Lambda_ReductionPlan().ReduceAll(); // simple, parallel
			// recalculate the reduction plan. could recalculate only for the diagonal entries
			// if we kept the list or if the vertices kept their pointers (they do not in v2)
		} else {
			_ASSERTE(m_b_system_dirty || !m_n_iteration_num); // or we are simply running for the first time
			// this means that the whole reduction plan just ran and the diagonal is clean
		}
		// see if we need to remove the previous alpha from the diagonal

		if(!f_alpha)
			return;
		// nothing to add there; save some time in calculating the marginals

		/*if(n_refresh_from_edge) {
			std::vector<size_t> vertex_set;
			vertex_set.reserve(this->m_r_system.n_Edge_Num() * 2); // safe assumption in BA
			for(size_t e = n_refresh_from_edge, n = this->m_r_system.n_Edge_Num(); e < n; ++ e) { // t_odo - parallel
				typename CSystem::_TyConstEdgeRef r_edge = this->m_r_system.r_Edge_Pool()[e];
				for(int j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j) {
					if(r_edge.n_Vertex_Id(j) < this->m_r_system.r_Vertex_Pool().n_Size()) // only non-const vertices
						vertex_set.push_back(r_edge.n_Vertex_Id(j));
				}
			}
			std::sort(vertex_set.begin(), vertex_set.end());
			vertex_set.erase(std::unique(vertex_set.begin(), vertex_set.end()), vertex_set.end());
			// make a unique set of affected vertices

			_ASSERTE(vertex_set.size() <= INT_MAX);
			int _n = int(vertex_set.size());
			#pragma omp parallel for if(_n > 50)
			for(int v = 0; v < _n; ++ v) // t_odo - parallel
				m_TR_algorithm.ApplyDamping(m_lambda, f_alpha, v, v + 1); // only damp the new or updated vertices
			// refresh only vertices belonging to the new edges
		} else*/ {
			_ASSERTE(m_lambda.n_BlockColumn_Num() <= INT_MAX);
			int _n = int(m_lambda.n_BlockColumn_Num());
			#pragma omp parallel for if(_n > 50)
			for(int v = 0; v < _n; ++ v) // t_odo - parallel
				m_TR_algorithm.ApplyDamping(m_lambda, f_alpha, v, v + 1); // do it in parallel
			// refresh the full diagonal
		}
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
		m_TR_algorithm.ApplyDamping(m_lambda, f_alpha, n_refresh_from_vertex, m_lambda.n_BlockColumn_Num());
		// always all of the vertices (all of them updated)
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
	}

	CNonlinearSolver_Lambda_LM(const CNonlinearSolver_Lambda_LM &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_Lambda_LM &operator =(const CNonlinearSolver_Lambda_LM &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_LAMBDA_LM_INCLUDED
