/*
								+-----------------------------------+
								|                                   |
								|  ***  Incremental BA parser  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2016  |
								|                                   |
								|      IncBAParsePrimitives.h       |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __INCREMENTAL_BA_GRAPH_PARSER_PRIMITIVES_INCLUDED
#define __INCREMENTAL_BA_GRAPH_PARSER_PRIMITIVES_INCLUDED

/**
 *	@file include/slam_app/IncBAParsePrimitives.h
 *	@brief handler for the CONSISTENCY_MARKER token and the associated modified parse loop
 *	@author -tHE SWINe-
 *	@date 2016-05-06
 */

#include "slam/ParseLoop.h"
#include "slam/TypeList.h"

/**
 *	@brief consistency marker type (passed as an edge)
 */
class CConsistencyMarker {};

/**
 *	@brief consistency marker parse primitive handler
 */
class CConsistencyMarker_ParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throw(std::bad_alloc)
	{
		r_token_name_map["CONSISTENCY_MARKER"] = n_assigned_id;
		// add as uppercase!
	}

	/**
	 *	@brief parses this primitive and dispatches it to the parse loop
	 *
	 *	@param[in] n_line_no is zero-based line number (for error reporting)
	 *	@param[in] r_s_line is string, containing the current line (without the token)
	 *	@param[in] r_s_token is string, containing the token name (in uppercase)
	 *	@param[in,out] r_parse_loop is target for passing the parsed primitives to
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class _TyParseLoop>
	static bool Parse_and_Dispatch(size_t n_line_no, const std::string &r_s_line,
		const std::string &UNUSED(r_s_token), _TyParseLoop &r_parse_loop)
	{
		r_parse_loop.AppendSystem(CConsistencyMarker());
		// append fake measurement to invoke

		return true;
	}
};

/**
 *	@copydoc CParseLoop
 *	@note This is overloaded with CConsistencyMarker edge type for on-demand optimization.
 */
template <class CSystem, class CNonlinearSolver, template <class> class CEdgeTraits,
	template <class> class CVertexTraits = CIgnoreAllVertexTraits>
class CParseLoop_ConsistencyMarker : public CParseLoop<CSystem, CNonlinearSolver, CEdgeTraits, CVertexTraits> {
public:
	typedef CParseLoop<CSystem, CNonlinearSolver, CEdgeTraits, CVertexTraits> _TyBase; /**< @brief base class */

protected:
	size_t m_n_step; /**< @brief optimization step; increased every time a consistency marker is encountered */
	bool m_b_save_intermediates; /**< @brief save intermediate solution steps solutions (and marginals if enabled) */
	bool m_b_save_matrices; /**< @brief save intermediate system matrices */

public:
	/**
	 *	@copydoc CParseLoop::CParseLoop
	 */
	inline CParseLoop_ConsistencyMarker(CSystem &r_system, CNonlinearSolver &r_solver)
		:_TyBase(r_system, r_solver), m_n_step(0),
		m_b_save_intermediates(false), m_b_save_matrices(false)
	{}

	/**
	 *	@copydoc CParseLoop::AppendSystem
	 */
	template <class CParsedEdge>
	inline void AppendSystem(const CParsedEdge &r_edge) // throw(std::bad_alloc, std::runtime_error)
	{
		this->_TyBase::AppendSystem(r_edge);
	}

	/**
	 *	@brief saves marginal covariances to a text file
	 *	@tparam b_have_marginals is marginal covariance support flag from the nonlinear solver
	 *	@param[in] p_s_filename is a null-terminated string with output file name
	 */
	template <const bool b_have_marginals>
	typename CEnableIf<b_have_marginals>::T SaveMarginals(const char *p_s_filename)
	{
		if(!this->m_r_solver.t_MarginalsPolicy().b_calculate)
			return;
		// is it enabled at all?

		//if(!this->m_r_solver.r_MarginalCovariance().r_SparseMatrix().b_Empty()) 
			this->m_r_solver.r_MarginalCovariance().Dump_Diagonal(p_s_filename);
	}

	/**
	 *	@brief saves marginal covariances to a text file (specialization for solvers that do not support marginals)
	 *	@tparam b_have_marginals is marginal covariance support flag from the nonlinear solver
	 *	@param[in] p_s_filename is a null-terminated string with output file name (unused)
	 */
	template <const bool b_have_marginals>
	typename CEnableIf<!b_have_marginals>::T SaveMarginals(const char *UNUSED(p_s_filename))
	{
		// nothing to do, the marginals can't be recovered
	}

	/**
	 *	@brief saves system matrix to a matrix market file
	 *	@tparam b_have_sysmat is system matrix export flag from the nonlinear solver
	 *	@param[in] p_s_filename is a null-terminated string with output file name
	 */
	template <const bool b_have_sysmat>
	typename CEnableIf<b_have_sysmat>::T SaveSysMatrix(const char *p_s_filename)
	{
		this->m_r_solver.Save_SystemMatrix_MM(p_s_filename);
	}

	/**
	 *	@brief saves system matrix to a matrix market file (specialization for solvers that do not export system matrix)
	 *	@tparam b_have_sysmat is system matrix export flag from the nonlinear solver
	 *	@param[in] p_s_filename is a null-terminated string with output file name (unused)
	 */
	template <const bool b_have_sysmat>
	typename CEnableIf<!b_have_sysmat>::T SaveSysMatrix(const char *UNUSED(p_s_filename))
	{
		// nothing to do, the system matrix can't be saved
	}

	/**
	 *	@brief handles the consistency marker token
	 *	@param[in] r_marker is the consistency marker parse primitive (unused)
	 */
	void AppendSystem(const CConsistencyMarker &UNUSED(r_marker)) // throw(std::bad_alloc, std::runtime_error)
	{
		const TIncrementalSolveSetting &t_incremental_config = this->m_r_solver.t_IncrementalConfig();
		// user-specified incremental configuration for the optimization

		if(!t_incremental_config.t_nonlinear_freq.n_period &&
		   !t_incremental_config.t_linear_freq.n_period)
			return;
		// in case incremental solving is disabled, ignore the token and handle the dataset as batch

		this->m_r_solver.Optimize(t_incremental_config.n_max_nonlinear_iteration_num,
			t_incremental_config.f_nonlinear_error_thresh); // force optimization
		if(m_b_save_intermediates) { // enable this if you want to see system states at different incremental points in the graph // t_odo - add some commandline that handles this
			char p_s_filename[256];
			sprintf(p_s_filename, "solution_" PRIsize ".txt", m_n_step);
			this->m_r_system.Dump(p_s_filename);
			// save the current state to a file

			if(CSystem::have_ConstVertices /*&& !system.r_ConstVertex_Pool().b_Empty()*/) { // the second condition makes processing the results potentially cumbersome
				sprintf(p_s_filename, "solution_const_" PRIsize ".txt", m_n_step);
				this->m_r_system.Dump(p_s_filename, true);
			}
			// save the current state, inclusing const vertices to a file

			sprintf(p_s_filename, "marginals_" PRIsize ".txt", m_n_step);
			SaveMarginals<CNonlinearSolver::solver_HasMarginals>(p_s_filename);
			// save the marginals, in case the solver supports it (and in case -dm is specified)
		}
		if(m_b_save_matrices) {
			char p_s_filename[256];
			sprintf(p_s_filename, "system_" PRIsize ".mtx", m_n_step);
			SaveSysMatrix<CNonlinearSolver::solver_ExportsJacobian ||
				CNonlinearSolver::solver_ExportsHessian ||
				CNonlinearSolver::solver_ExportsFactor>(p_s_filename);
			// uncomment to save system matrix
		}

		++ m_n_step;
	}

	/**
	 *	@brief enables saving intermediate solutions (and marginals if enabled) at each optimization marker
	 *	@param[in] b_enable is a new value of the enable flag
	 */
	void Enable_Save_Solutions(bool b_enable)
	{
		m_b_save_intermediates = b_enable;
	}

	/**
	 *	@brief enables saving system matrices at each optimization marker
	 *	@param[in] b_enable is a new value of the enable flag
	 */
	void Enable_Save_SysMatrices(bool b_enable)
	{
		m_b_save_matrices = b_enable;
	}

	/**
	 *	@copydoc CParseLoop::InitializeVertex
	 */
	template <class CParsedVertex>
	inline void InitializeVertex(const CParsedVertex &r_vertex) // throw(std::bad_alloc, std::runtime_error)
	{
		this->_TyBase::InitializeVertex(r_vertex);
	}
};

#endif // __INCREMENTAL_BA_GRAPH_PARSER_PRIMITIVES_INCLUDED
