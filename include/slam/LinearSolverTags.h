/*
								+----------------------------------+
								|                                  |
								|   ***  Linear solver tags  ***   |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2012  |
								|                                  |
								|        LinearSolverTags.h        |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __LINEAR_SOLVER_TAGS_INCLUDED
#define __LINEAR_SOLVER_TAGS_INCLUDED

/**
 *	@file include/slam/LinearSolverTags.h
 *	@brief linear solver tags
 *	@author -tHE SWINe-
 *	@date 2012-09-04
 *
 *	Each solver class needs to have public type named _Tag,
 *	which is one of the types defined below. Based on that,
 *	the nonlinear solvers can then decide which functions
 *	the linear solver implements and what is the best
 *	approach to use.
 *
 */

/** \addtogroup linsolve
 *	@{
 */

/**
 *	@brief default tag for all the linear solvers
 */
class CBasicLinearSolverTag {};

/**
 *	@brief solvers supporting blockwise solutions
 *
 *	The usage of such solver is supposed to be:
 *
 *	@code
 *	UberBlockMatrix A;
 *	Eigen::VectorXd b;
 *	Solver s;
 *
 *	s.SymbolicDecomposition_Blocky(A);
 *	s.Solve_PosDef_Blocky(A, b);
 *	@endcode
 */
class CBlockwiseLinearSolverTag {};

#include "slam/BlockMatrix.h"

/**
 *	@brief wrapper for linear solvers (shields solver capability to solve blockwise)
 *
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CSolverTag is linear solver tag
 */
template <class CLinearSolver, class CSolverTag>
class CLinearSolverWrapper {
public:
	/**
	 *	@brief estabilishes final block matrix structure before solving iteratively
	 *
	 *	@param[in] r_solver is linear solver
	 *	@param[in] r_lambda is the block matrix
	 *
	 *	@return Always returns true.
	 */
	static inline bool FinalBlockStructure(CLinearSolver &r_solver,
		const CUberBlockMatrix &r_lambda)
	{
		return true;
	}

	/**
	 *	@brief calculates ordering, solves a system
	 *
	 *	@param[in] r_solver is linear solver
	 *	@param[in] r_lambda is the block matrix
	 *	@param[in] r_v_eta is the right side vector
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static inline bool Solve(CLinearSolver &r_solver,
		const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_v_eta)
	{
		return r_solver.Solve_PosDef(r_lambda, r_v_eta);
	}
};

/**
 *	@brief wrapper for linear solvers (specialization for CBlockwiseLinearSolverTag sovers)
 *	@tparam CLinearSolver is linear solver type
 */
template <class CLinearSolver>
class CLinearSolverWrapper<CLinearSolver, CBlockwiseLinearSolverTag> {
public:
	/**
	 *	@brief estabilishes final block matrix structure before solving iteratively
	 *
	 *	@param[in] r_solver is linear solver
	 *	@param[in] r_lambda is the block matrix
	 *
	 *	@return Always returns true.
	 */
	static inline bool FinalBlockStructure(CLinearSolver &r_solver,
		const CUberBlockMatrix &UNUSED(r_lambda))
	{
		r_solver.Clear_SymbolicDecomposition();
		// will trigger automatic recalculation and saves one needless converting lambda to cs*

		return true;
	}

	/**
	 *	@brief solves a system, reusing the previously calculated block ordering
	 *
	 *	@param[in] r_solver is linear solver
	 *	@param[in] r_lambda is the block matrix
	 *	@param[in] r_v_eta is the right side vector
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static inline bool Solve(CLinearSolver &r_solver,
		const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_v_eta)
	{
		return r_solver.Solve_PosDef_Blocky(r_lambda, r_v_eta);
	}
};

/** @} */ // end of group

#endif // !__LINEAR_SOLVER_TAGS_INCLUDED
