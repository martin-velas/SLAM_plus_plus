/*
								+----------------------------------+
								|                                  |
								| ***  Non-linear solver base  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|      NonlinearSolver_Base.h      |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_SOLVER_BASE_INCLUDED
#define __NONLINEAR_SOLVER_BASE_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_Base.h
 *	@brief common implementation of nonlinear solvers
 *	@author -tHE SWINe-
 *	@date 2015-06-25
 */

#include "slam/FlatSystem.h"
#include "slam/IncrementalPolicy.h"

// note to self: when migrating solvers, replace:
/*

this->m_t_incremental_config
this->m_t_marginals_config
this->m_marginals
this->m_timer
this->m_r_system
this->m_b_verbose
this->m_b_use_schur
this->m_linear_solver
this->m_schur_solver

replace:
{m_t_incremental_config|m_t_marginals_config|m_marginals|m_timer|m_r_system|m_b_verbose|m_b_use_schur|m_linear_solver|m_schur_solver}
this->\1

replace:
this->m_t_incremental_config.n_max_nonlinear_iteration_num	n_nonlinear_solve_max_iteration_num
this->m_t_incremental_config.f_nonlinear_error_thresh		f_nonlinear_solve_error_threshold
this->m_t_incremental_config.t_linear_freq.n_period			n_linear_solve_threshold
this->m_t_incremental_config.t_nonlinear_freq.n_period		n_nonlinear_solve_threshold

*/

template <class CBaseSolver, class CAMatrixBlockSizes, class CSystem>
class CLinearSolver_Schur; // forward declaration
class CMarginalCovariance; // forward declaration

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@brief namespace with internal implementation used by the nonlinear solvers
 */
namespace nonlinear_detail {

/**
 *	@brief placeholder type
 */
struct TVoidType {
	/**
	 *	@brief default constructor; has no effect
	 */
	TVoidType()
	{}

	/**
	 *	@brief constructor; has no effect
	 *	@tparam CInitializerType is initializer type
	 *	@param[in] r_init is an initializer (unused)
	 */
	template <class CInitializerType>
	TVoidType(const CInitializerType &UNUSED(r_init))
	{}
};

/**
 *	@brief common nonlinear solver functions
 *	@note Those are not member functions of CNonlinearSolver_Base0 or CNonlinearSolver_Base
 *		because they do not depend on their template arguments and it would be wasteful to
 *		compile many instances of the same functions.
 */
class CSolverOps_Base {
public:
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
	 *	@brief function object that saves state of all the vertices
	 */
	class CSaveState {
	protected:
		Eigen::VectorXd &m_r_state; /**< @brief reference to the state vector (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_state is reference to the state vector
		 */
		inline CSaveState(Eigen::VectorXd &r_state)
			:m_r_state(r_state)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertex is vertex type
		 *	@param[in,out] r_t_vertex is vertex to output its part the state vector
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex)
		{
			r_t_vertex.SaveState(m_r_state);
		}
	};

	/**
	 *	@brief function object that saves state of all the vertices
	 */
	class CLoadState {
	protected:
		const Eigen::VectorXd &m_r_state; /**< @brief reference to the state vector (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_state is reference to the state vector
		 */
		inline CLoadState(const Eigen::VectorXd &r_state)
			:m_r_state(r_state)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertex is vertex type
		 *	@param[in,out] r_t_vertex is vertex to output its part the state vector
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex)
		{
			r_t_vertex.LoadState(m_r_state);
		}
	};

public:
	/**
	 *	@brief checks for NaN valus in the given vector (debugging functionality)
	 *
	 *	@param[in] r_v_dx is the delta vector
	 *	@param[in] b_stderr_output is stderr output flag
	 *	@param[in] p_s_vec_name is null-terminated string with the name of the vector for stderr outputs
	 *	@param[in] n_max_warning_num is maximum number of stderr warnings (use SIZE_MAX for unlimited)
	 *
	 *	@return Returns true if no NaN values occured,
	 *		otherwise detects false (indicating numerical failure).
	 */
	static bool b_DetectNaNs(const Eigen::VectorXd &r_v_vec, bool b_stderr_output = false,
		const char *p_s_vec_name = "vec", size_t n_max_warning_num = 10)
	{
		size_t n_NaN_num = false;
		for(size_t i = 0, n_variables_size = r_v_vec.rows(); i < n_variables_size; ++ i) {
			if(_isnan(r_v_vec(i))) {
				++ n_NaN_num;
				if(b_stderr_output && n_max_warning_num) {
					if(n_max_warning_num != SIZE_MAX)
						-- n_max_warning_num;
					fprintf(stderr, "warning: %s[" PRIsize "] = NaN\n", p_s_vec_name, i);
				}
			}
		}
		// detect the NaNs, if any (warn, but don't modify)

		if(b_stderr_output && n_NaN_num && !n_max_warning_num) {
			fprintf(stderr, "warning: " PRIsize " / " PRIsize " elements are NaN\n",
				n_NaN_num, size_t(r_v_vec.rows()));
		}
		// in case the output is truncated, print the number as well

		return !n_NaN_num;
	}

	/**
	 *	@brief updates states of all the vertices with the delta vector
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[out] r_system is reference to the system
	 *	@param[in] r_v_dx is the delta vector
	 *
	 *	@note This checks for NaN in debug.
	 */
	template <class CSystem>
	static inline void PushValuesInGraphSystem(CSystem &r_system, const Eigen::VectorXd &r_v_dx)
	{
#ifdef _DEBUG
		b_DetectNaNs(r_v_dx, true, "dx");
#endif // _DEBUG
		r_system.r_Vertex_Pool().For_Each_Parallel(CUpdateEstimates(r_v_dx)); // can do this in parallel
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@tparam CSystem is system type
	 *	@param[in] r_system is reference to the system
	 *	@return Returns \f$\chi^2\f$ error.
	 *	@note This only works with systems with edges of one degree of freedom
	 *		(won't work for e.g. systems with both poses and landmarks).
	 */
	template <class CSystem>
	static inline double f_Chi_Squared_Error(const CSystem &r_system)
	{
		if(r_system.r_Edge_Pool().b_Empty())
			return 0;
		return r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError()) /
			(r_system.r_Edge_Pool().n_Size() - r_system.r_Edge_Pool()[0].n_Dimension());
	}

	/**
	 *	@brief calculates denormalized \f$\chi^2\f$ error
	 *	@tparam CSystem is system type
	 *	@param[in] r_system is reference to the system
	 *	@return Returns denormalized \f$\chi^2\f$ error.
	 *	@note This doesn't perform the final division by (number of edges - degree of freedoms).
	 */
	template <class CSystem>
	static inline double f_Chi_Squared_Error_Denorm(const CSystem &r_system)
	{
		return r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError());
	}

	/**
	 *	@brief saves state of the optimized vertices to a vector
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in] r_system is reference to the system
	 *	@param[in,out] r_v_state is vector to save state (must be allocated to the dimension of the state)
	 */
	template <class CSystem>
	static inline void Save_State(const CSystem &r_system, Eigen::VectorXd &r_v_state)
	{
		r_system.r_Vertex_Pool().For_Each_Parallel(CSaveState(r_v_state));
	}

	/**
	 *	@brief loads state of the optimized vertices to a vector
	 *
	 *	@tparam CSystem is system type
	 *
	 *	@param[in,out] r_system is reference to the system
	 *	@param[in] r_v_state is vector to load the state from
	 */
	template <class CSystem>
	static inline void Load_State(CSystem &r_system, const Eigen::VectorXd &r_v_state)
	{
		r_system.r_Vertex_Pool().For_Each_Parallel(CLoadState(r_v_state));
	}
};

/**
 *	@brief common nonlinear solver functions implementation
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix (required for Schur)
 *	@tparam b_enable_schur is enable flag for Schur linear solver support
 *	@tparam b_enable_marginals is enable flag for marginal covariances support
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes =
	typename CSystem::_TyJacobianMatrixBlockList, const bool b_enable_schur = false,
	const bool b_enable_marginals = false>
class CNonlinearSolver_Base0 /*: public CInheritIf<CMarginalsGetter<CNonlinearSolver_Base<CSystem,
	CLinearSolver, CAMatrixBlockSizes, b_enable_schur, b_enable_marginals> >, b_enable_marginals>*/ {
protected:
	CSystem &m_r_system; /**< @brief reference to the system */
	CLinearSolver m_linear_solver; /**< @brief linear solver */
	typename CChooseType<CLinearSolver_Schur<CLinearSolver, CAMatrixBlockSizes,
		CSystem>, TVoidType, b_enable_schur>::_TyResult m_schur_solver; /**< @brief linear solver with Schur trick */

	TIncrementalSolveSetting m_t_incremental_config; /**< @brief incremental solving configuration */
	bool m_b_use_schur; /**< @brief use schur complement flag */

	bool m_b_verbose; /**< @brief verbosity flag */

	typename CChooseType<TMarginalsComputationPolicy, TVoidType,
		b_enable_marginals>::_TyResult m_t_marginals_config; /**< @brief marginal covariance computation configuration */
	typename CChooseType<CMarginalCovariance, TVoidType,
		b_enable_marginals>::_TyResult m_marginals; /**< @brief marginals cache */

	CTimer m_timer; /**< @brief timer object */

	size_t m_n_real_step; /**< @brief counter of incremental steps (no modulo) */

private:
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_last_optimized_vertex_num; /**< @brief counter of incremental steps, stores the number of vertices in the system upon the last optimization */
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_step; /**< @brief counter of incremental steps modulo m_t_incremental_config.t_nonlinear_freq.n_period */
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_prev_vertex_num; /**< @brief number of vertices at the previous loop closure detection */
	bool m_b_had_loop_closure; /**< @brief (probable) loop closure flag */
	// private state, not to be changed by the derived classes

protected:
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
	 *
	 *	@note This constructor is protected; this class needs to be ingerited from before it can be used.
	 */
	CNonlinearSolver_Base0(CSystem &r_system,
		size_t n_linear_solve_threshold, size_t n_nonlinear_solve_threshold,
		size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		const CLinearSolver &linear_solver = CLinearSolver(), bool b_use_schur = false)
		:m_r_system(r_system), m_linear_solver(linear_solver), m_schur_solver(linear_solver),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_b_use_schur(b_use_schur), m_b_verbose(b_verbose),
		m_b_had_loop_closure(false), m_n_real_step(0)
	{
		m_t_incremental_config.t_linear_freq.n_period = n_linear_solve_threshold;
		m_t_incremental_config.t_nonlinear_freq.n_period = n_nonlinear_solve_threshold;
		m_t_incremental_config.n_max_nonlinear_iteration_num = n_nonlinear_solve_max_iteration_num;
		m_t_incremental_config.f_nonlinear_error_thresh = f_nonlinear_solve_error_threshold;
		// don't have simultaneously linear and nonlinear solve constructor

		_ASSERTE(!m_t_incremental_config.t_nonlinear_freq.n_period ||
			!m_t_incremental_config.t_linear_freq.n_period); // only one of those

		_ASSERTE(b_enable_schur || !b_use_schur); // schur cannot be requested unless it is enabled
	}

	/**
	 *	@brief initializes the nonlinear solver with schur complement support
	 *
	 *	@param[in] r_system is the system to be optimized
	 *		(it is only referenced, not copied - must not be deleted)
	 *	@param[in] t_incremental_config is incremental solving configuration
	 *	@param[in] t_marginals_config is marginal covariance calculation configuration
	 *	@param[in] b_verbose is verbosity flag
	 *	@param[in] linear_solver is linear solver instance
	 *	@param[in] b_use_schur is Schur complement trick flag
	 *
	 *	@note This constructor is protected; this class needs to be ingerited from before it can be used.
	 */
	CNonlinearSolver_Base0(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false, const CLinearSolver &linear_solver = CLinearSolver(), bool b_use_schur = false)
		:m_r_system(r_system), m_linear_solver(linear_solver), m_schur_solver(linear_solver),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_t_incremental_config(t_incremental_config),
		m_t_marginals_config(t_marginals_config),
		m_b_use_schur(b_use_schur), m_b_verbose(b_verbose),
		m_b_had_loop_closure(false), m_n_real_step(0)
	{
		_ASSERTE(!m_t_incremental_config.t_nonlinear_freq.n_period ||
			!m_t_incremental_config.t_linear_freq.n_period); // only one of those

		_ASSERTE(b_enable_schur || !b_use_schur); // schur cannot be requested unless it is enabled
	}

public:
	/**
	 *	@brief gets the verbosity flag
	 *	@return Returns the value of the verbosity flag.
	 */
	bool b_Verbose() const
	{
		return m_b_verbose;
	}

	/**
	 *	@brief gets solver incremental configuration
	 *	@return Returns the incremental configuration.
	 */
	const TIncrementalSolveSetting &t_IncrementalConfig() const
	{
		return m_t_incremental_config;
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns \f$\chi^2\f$ error.
	 *	@note This only works with systems with edges of one degree of freedom
	 *		(won't work for e.g. systems with both poses and landmarks).
	 */
	double f_Chi_Squared_Error() const
	{
		return CSolverOps_Base::f_Chi_Squared_Error(m_r_system);
	}

	/**
	 *	@brief calculates denormalized \f$\chi^2\f$ error
	 *	@return Returns denormalized \f$\chi^2\f$ error.
	 *	@note This doesn't perform the final division by (number of edges - degree of freedoms).
	 */
	double f_Chi_Squared_Error_Denorm() const
	{
		return CSolverOps_Base::f_Chi_Squared_Error_Denorm(m_r_system);
	}

protected:
	/**
	 *	@brief loop closure detection
	 *	@param[in] r_last_edge is the last edge (already) added to the system (unused)
	 *	@return Returns true in case loop closures have been detected, otherwise returns false.
	 *	@note This is called at the beginning of t_Incremental_Step().
	 */
	bool b_Detect_LoopClosures(const typename CSystem::_TyBaseEdge &UNUSED(r_last_edge))
	{
		size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_had_loop_closure) {
			_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
			typename CSystem::_TyEdgeMultiPool::_TyConstBaseRef r_edge =
				m_r_system.r_Edge_Pool()[m_r_system.r_Edge_Pool().n_Size() - 1];
			// get a reference to the last edge interface

			const size_t n = r_edge.n_Vertex_Num();
			if(n > 1) { // unary factors do not cause classical loop closures
				_ASSERTE(r_edge.n_Vertex_Id(0) != r_edge.n_Vertex_Id(1));
				// covers most cases but can still be broken by hyperedges (not easy
				// to detect the number of unique vertices without making a list)

				size_t n_first_vertex = r_edge.n_Vertex_Id(0);
				for(size_t i = 1; i < n; ++ i)
					n_first_vertex = std::min(n_first_vertex, r_edge.n_Vertex_Id(i));
				_ASSERTE(n_first_vertex < m_r_system.r_Vertex_Pool().n_Size()); // make sure it is not a constant vertex
				// finds the vertex with the lowest id

				m_b_had_loop_closure = n_first_vertex + n < n_vertex_num; // this test was used in the pre-2.0 SLAM++
				//m_b_had_loop_closure = n_first_vertex + 1 < m_n_prev_vertex_num; // this test is more sensitive
				// determine whether the edge relates to an earlier vertex or whether
				// it adds all the vertices at the end of the state vector

				//_ASSERTE(m_b_had_loop_closure || std::max(r_edge.n_Vertex_Id(0),
				//	r_edge.n_Vertex_Id(1)) == n_vertex_num - 1); // this is broken with const vertices
				// not sure what this checks
			} else if(n == 1) { // no nullary factors expected in the system so far but lets be general
				size_t n_first_vertex = r_edge.n_Vertex_Id(0);
				_ASSERTE(n_first_vertex < m_r_system.r_Vertex_Pool().n_Size()); // make sure it is not a constant vertex (unary edge, shouldn't be)
				m_b_had_loop_closure = n_first_vertex + 1 < m_n_prev_vertex_num;
				// we need to optimize unless the edge did not introduce its vertex
			}
			// t_odo - encapsulate this code, write code to support hyperedges as well, use it
		}
		m_n_prev_vertex_num = n_vertex_num; // detects loop closures inside the new portion of the graph as well, those also need optimization (initial guess is unable to take care of those)
		// detect loop closures (otherwise the edges are initialized based on measurement and error would be zero)

		return m_b_had_loop_closure;
	}

	/**
	 *	@brief loop closure detection and optimization policy handling
	 *
	 *	@param[in] r_last_edge is the last edge (already) added to the system (unused)
	 *	@param[in] b_optimize_without_loop_closures is optimization without loop
	 *		closures flag (default not set; optimization is then triggered even if no loop
	 *		closures were detected; detection runs either way)
	 *
	 *	@return Returns a pair of incremental optimization counter trigger flag and the
	 *		optimization type to run (0 being none - no loop closure(s) occurred,
	 *		1 being linear and 2 nonlinear).
	 */
	std::pair<bool, int> t_Incremental_Step(const typename CSystem::_TyBaseEdge &UNUSED(r_last_edge),
		bool b_optimize_without_loop_closures = false)
	{
		size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		b_Detect_LoopClosures(r_last_edge);
		// detect loop closures (otherwise the edges are initialized based on measurement and error would be zero)

		bool b_new_vert = false;
		int n_opt_type = 0;

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
		if(m_t_incremental_config.t_nonlinear_freq.n_period && m_n_step >= m_t_incremental_config.t_nonlinear_freq.n_period) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure || b_optimize_without_loop_closures) {
				n_opt_type = 2;
				m_b_had_loop_closure = false;
				//Optimize(m_t_incremental_config.n_max_nonlinear_iteration_num, m_t_incremental_config.f_nonlinear_error_thresh);
			}
			// nonlinear optimization

			b_new_vert = true;
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_t_incremental_config.t_linear_freq.n_period && n_new_vertex_num >= m_t_incremental_config.t_linear_freq.n_period) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_t_incremental_config.t_linear_freq.n_period && ((!m_t_incremental_config.t_nonlinear_freq.n_period &&
		   m_n_step >= m_t_incremental_config.t_linear_freq.n_period) || m_n_step % m_t_incremental_config.t_linear_freq.n_period == 0)) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			_ASSERTE(!m_t_incremental_config.t_nonlinear_freq.n_period);
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			if(!m_t_incremental_config.t_nonlinear_freq.n_period)
				m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure || b_optimize_without_loop_closures) {
				n_opt_type = 1;
				m_b_had_loop_closure = false;
				//Optimize(1, 0); // only if there was a loop (ignores possibly high residual after single step optimization)
			}
			// simple optimization

			b_new_vert = true;
		}

		return std::make_pair(b_new_vert, n_opt_type);
	}

	/**
	 *	@brief gets the loop closure flag
	 *	@return Returns the loop closure flag.
	 */
	inline bool b_Had_LoopClosures() const
	{
		return m_b_had_loop_closure;
	}

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES

	/**
	 *	@brief gets the number of vertices at the time when the optimization last triggered
	 *	@return Returns the number of vertices at the time when the optimization last triggered.
	 */
	inline size_t n_LastOptimized_Vertex_Num() const
	{
		return m_n_last_optimized_vertex_num;
	}

#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES

	/**
	 *	@brief gets the number of edges at the time when the optimization
	 *		last triggered, modulo optimization frequency
	 *	@return Returns the number of edges at the time when the optimization
	 *		last triggered, modulo optimization frequency.
	 */
	inline size_t n_New_Edge_Num() const
	{
		return m_n_step;
	}

#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
};

/**
 *	@copydoc CNonlinearSolver_Base0
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes =
	typename CSystem::_TyJacobianMatrixBlockList, const bool b_enable_schur = false,
	const bool b_enable_marginals = false>
class CNonlinearSolver_Base : public CNonlinearSolver_Base0<CSystem, CLinearSolver,
	CAMatrixBlockSizes, b_enable_schur, b_enable_marginals> {
protected:
	/**
	 *	@copydoc CNonlinearSolver_Base0::CNonlinearSolver_Base0
	 */
	inline CNonlinearSolver_Base(CSystem &r_system,
		size_t n_linear_solve_threshold, size_t n_nonlinear_solve_threshold,
		size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		const CLinearSolver &linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Base0<CSystem, CLinearSolver, CAMatrixBlockSizes, b_enable_schur,
		b_enable_marginals>(r_system, n_linear_solve_threshold, n_nonlinear_solve_threshold,
		n_nonlinear_solve_max_iteration_num, f_nonlinear_solve_error_threshold, b_verbose,
		linear_solver, b_use_schur)
	{}

	/**
	 *	@copydoc CNonlinearSolver_Base0::CNonlinearSolver_Base0(CSystem&,TIncrementalSolveSetting,TMarginalsComputationPolicy,bool,CLinearSolver,bool)
	 */
	inline CNonlinearSolver_Base(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false, const CLinearSolver &linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Base0<CSystem, CLinearSolver, CAMatrixBlockSizes,
		b_enable_schur, b_enable_marginals>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

/**
 *	@brief common nonlinear solver functions implementation (specialization for marginals support)
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix (required for Schur)
 *	@tparam b_enable_schur is enable flag for Schur linear solver support
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes, const bool b_enable_schur>
class CNonlinearSolver_Base<CSystem, CLinearSolver, CAMatrixBlockSizes, b_enable_schur, true> :
	public CNonlinearSolver_Base0<CSystem, CLinearSolver, CAMatrixBlockSizes, b_enable_schur, true> {
protected:
	/**
	 *	@copydoc CNonlinearSolver_Base0::CNonlinearSolver_Base0
	 */
	inline CNonlinearSolver_Base(CSystem &r_system,
		size_t n_linear_solve_threshold, size_t n_nonlinear_solve_threshold,
		size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		const CLinearSolver &linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Base0<CSystem, CLinearSolver, CAMatrixBlockSizes,
		b_enable_schur, true>(r_system, n_linear_solve_threshold, n_nonlinear_solve_threshold,
		n_nonlinear_solve_max_iteration_num, f_nonlinear_solve_error_threshold, b_verbose,
		linear_solver, b_use_schur)
	{}

	/**
	 *	@copydoc CNonlinearSolver_Base0::CNonlinearSolver_Base0(CSystem&,TIncrementalSolveSetting,TMarginalsComputationPolicy,bool,CLinearSolver,bool)
	 */
	inline CNonlinearSolver_Base(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false, const CLinearSolver &linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Base0<CSystem, CLinearSolver, CAMatrixBlockSizes,
		b_enable_schur, true>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}

public:
	/**
	 *	@brief gets solver marginal covariance computation policy
	 *	@return Returns the marginal covariance computation policy.
	 *	@note This function is only enabled if this solver supports marginals.
	 */
	const TMarginalsComputationPolicy &t_MarginalsPolicy() const
	{
		return this->m_t_marginals_config;
	}

	/**
	 *	@brief gets marginal covariances
	 *	@return Returns reference to the marginal covariances object.
	 *	@note This function is only enabled if this solver supports marginals.
	 */
	inline CMarginalCovariance &r_MarginalCovariance()
	{
		return this->m_marginals;
	}

	/**
	 *	@brief gets marginal covariances
	 *	@return Returns const reference to the marginal covariances object.
	 *	@note This function is only enabled if this solver supports marginals.
	 */
	inline const CMarginalCovariance &r_MarginalCovariance() const
	{
		return this->m_marginals;
	}
};

} // ~nonlinear_detail

/** @} */ // end of group

#endif // !__NONLINEAR_SOLVER_BASE_INCLUDED
