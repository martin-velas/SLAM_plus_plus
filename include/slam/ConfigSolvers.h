/*
								+-----------------------------------+
								|                                   |
								|  ***  Solvers configuration  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|          ConfigSolvers.h          |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SOLVERS_CONFIGURATION_INCLUDED
#define __SOLVERS_CONFIGURATION_INCLUDED

/**
 *	@file include/slam/ConfigSolvers.h
 *	@author -tHE SWINe-
 *	@brief a file, containing solvers configuration (intended for enabling / disabling solvers)
 *	@date 2013
 */

#if !defined(__SE_TYPES_SUPPORT_A_SOLVERS) && \
	!defined(__SE_TYPES_SUPPORT_LAMBDA_SOLVERS) && \
	!defined(__SE_TYPES_SUPPORT_L_SOLVERS)
#define __SE_TYPES_SUPPORT_A_SOLVERS
#define __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#define __SE_TYPES_SUPPORT_L_SOLVERS
#endif // !__SE_TYPES_SUPPORT_A_SOLVERS && !__SE_TYPES_SUPPORT_LAMBDA_SOLVERS && !__SE_TYPES_SUPPORT_L_SOLVERS
// if the config is not included, just enable everything (default)

/**
 *	@def __SLAM_COUNT_ITERATIONS_AS_VERTICES
 *	@brief if defined, the incremental updates are scheduled per N vertices,
 *		if not defined, they are scheduled per N edges (presumably faster in batch each 1,
 *		practically slower in batch each 10)
 */
#if !defined(__SLAM_COUNT_ITERATIONS_AS_EDGES) && !defined(__SLAM_COUNT_ITERATIONS_AS_VERTICES)
#define __SLAM_COUNT_ITERATIONS_AS_VERTICES
#endif // !__SLAM_COUNT_ITERATIONS_AS_EDGES && !__SLAM_COUNT_ITERATIONS_AS_VERTICES

#include "slam/NonlinearSolver_A.h"
#include "slam/NonlinearSolver_Lambda.h"
#include "slam/NonlinearSolver_Lambda_LM.h"
#include "slam/NonlinearSolver_Lambda_DL.h"
#include "slam/NonlinearSolver_FastL.h"
// hint - disable or enable solvers at will

#include "slam/SolverTraits.h"
// some helper classes

/**
 *	@def _TySolverA_Name
 *	@brief name of the nonlinear solver A (not using typedef as it is a template)
 */
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
#ifdef __NONLINEAR_BLOCKY_SOLVER_A_INCLUDED

/**
 *	@copydoc CNonlinearSolver_Dummmy_Specializer
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_A_Specializer : public CNonlinearSolver_A<CSystem, CLinearSolver> {
public:
	/**
	 *	@copydoc CNonlinearSolver_Dummmy_Specializer::CNonlinearSolver_Dummmy_Specializer
	 */
	inline CNonlinearSolver_A_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_A<CSystem, CLinearSolver>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

#define _TySolverA_Name CNonlinearSolver_A_Specializer
#else // __NONLINEAR_BLOCKY_SOLVER_A_INCLUDED
#define _TySolverA_Name CSolverNotIncluded
#endif // __NONLINEAR_BLOCKY_SOLVER_A_INCLUDED
#else // __SE_TYPES_SUPPORT_A_SOLVERS
#define _TySolverA_Name CSolverNotSupported
#endif // __SE_TYPES_SUPPORT_A_SOLVERS

/**
 *	@def _TySolverLambda_Name
 *	@brief name of the Lambda nonlinear solver (not using typedef as it is a template)
 */
#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#ifdef __NONLINEAR_BLOCKY_SOLVER_LAMBDA_INCLUDED

/**
 *	@copydoc CNonlinearSolver_Dummmy_Specializer
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_Lambda_Specializer : public CNonlinearSolver_Lambda<CSystem, CLinearSolver> {
public:
	/**
	 *	@copydoc CNonlinearSolver_Dummmy_Specializer::CNonlinearSolver_Dummmy_Specializer
	 */
	inline CNonlinearSolver_Lambda_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Lambda<CSystem, CLinearSolver>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

#define _TySolverLambda_Name CNonlinearSolver_Lambda_Specializer
#else // __NONLINEAR_BLOCKY_SOLVER_LAMBDA_INCLUDED
#define _TySolverLambda_Name CSolverNotIncluded
#endif // __NONLINEAR_BLOCKY_SOLVER_LAMBDA_INCLUDED
#else // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#define _TySolverLambda_Name CSolverNotSupported
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS

/**
 *	@def _TySolverLambdaLM_Name
 *	@brief name of the Lambda nonlinear solver with LM (not using typedef as it is a template)
 */
#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#ifdef __NONLINEAR_BLOCKY_SOLVER_LAMBDA_LM_INCLUDED

/**
 *	@copydoc CNonlinearSolver_Dummmy_Specializer
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_LambdaLM_Specializer : public CNonlinearSolver_Lambda_LM<CSystem, CLinearSolver> {
public:
	/**
	 *	@copydoc CNonlinearSolver_Dummmy_Specializer::CNonlinearSolver_Dummmy_Specializer
	 */
	inline CNonlinearSolver_LambdaLM_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Lambda_LM<CSystem, CLinearSolver>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

#define _TySolverLambdaLM_Name CNonlinearSolver_LambdaLM_Specializer
#else // __NONLINEAR_BLOCKY_SOLVER_LAMBDA_LM_INCLUDED
#define _TySolverLambdaLM_Name CSolverNotIncluded
#endif // __NONLINEAR_BLOCKY_SOLVER_LAMBDA_LM_INCLUDED
#else // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#define _TySolverLambdaLM_Name CSolverNotSupported
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS

/**
 *	@def _TySolverLambdaDL_Name
 *	@brief name of the Lambda nonlinear solver with DL (not using typedef as it is a template)
 */
#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#ifdef __NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED

/**
 *	@copydoc CNonlinearSolver_Dummmy_Specializer
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_LambdaDL_Specializer : public CNonlinearSolver_Lambda_DL<CSystem, CLinearSolver> {
public:
	/**
	 *	@copydoc CNonlinearSolver_Dummmy_Specializer::CNonlinearSolver_Dummmy_Specializer
	 */
	inline CNonlinearSolver_LambdaDL_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_Lambda_DL<CSystem, CLinearSolver>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

#define _TySolverLambdaDL_Name CNonlinearSolver_LambdaDL_Specializer
#else // __NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED
#define _TySolverLambdaDL_Name CSolverNotIncluded
#endif // __NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED
#else // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#define _TySolverLambdaDL_Name CSolverNotSupported
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS

/**
 *	@def _TySolverL_Name
 *	@brief name of the nonlinear solver L (not using typedef as it is a template)
 */
#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
#ifdef __NONLINEAR_BLOCKY_SOLVER_L_INCLUDED

/**
 *	@copydoc CNonlinearSolver_Dummmy_Specializer
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_L_Specializer : public CNonlinearSolver_L<CSystem, CLinearSolver> {
public:
	/**
	 *	@copydoc CNonlinearSolver_Dummmy_Specializer::CNonlinearSolver_Dummmy_Specializer
	 */
	inline CNonlinearSolver_L_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_L<CSystem, CLinearSolver>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

#define _TySolverL_Name CNonlinearSolver_L_Specializer
#else // __NONLINEAR_BLOCKY_SOLVER_L_INCLUDED
#define _TySolverL_Name CSolverNotIncluded
#endif // __NONLINEAR_BLOCKY_SOLVER_L_INCLUDED
#else // __SE_TYPES_SUPPORT_L_SOLVERS
#define _TySolverL_Name CSolverNotSupported
#endif // __SE_TYPES_SUPPORT_L_SOLVERS

/**
 *	@def _TySolverFastL_Name
 *	@brief name of the FastL nonlinear solver (not using typedef as it is a template)
 */
#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
#ifdef __NONLINEAR_BLOCKY_SOLVER_FAST_L_INCLUDED

/**
 *	@copydoc CNonlinearSolver_Dummmy_Specializer
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_fastL_Specializer : public CNonlinearSolver_FastL<CSystem, CLinearSolver> {
public:
	/**
	 *	@copydoc CNonlinearSolver_Dummmy_Specializer::CNonlinearSolver_Dummmy_Specializer
	 */
	inline CNonlinearSolver_fastL_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
		:CNonlinearSolver_FastL<CSystem, CLinearSolver>(r_system, t_incremental_config,
		t_marginals_config, b_verbose, linear_solver, b_use_schur)
	{}
};

#define _TySolverFastL_Name CNonlinearSolver_fastL_Specializer
#else // __NONLINEAR_BLOCKY_SOLVER_FAST_L_INCLUDED
#define _TySolverFastL_Name CSolverNotIncluded
#endif // __NONLINEAR_BLOCKY_SOLVER_FAST_L_INCLUDED
#else // __SE_TYPES_SUPPORT_L_SOLVERS
#define _TySolverFastL_Name CSolverNotSupported
#endif // __SE_TYPES_SUPPORT_L_SOLVERS

/**
 *	@brief nonlinear solver type
 */
enum ENonlinearSolverType {
	nlsolver_A, /**< @brief nonlinear solver A */
	nlsolver_Lambda, /**< @brief nonlinear solver lambda */
	nlsolver_LambdaLM, /**< @brief nonlinear solver lambda with Levenberg-Marquardt */
	nlsolver_L, /**< @brief nonlinear solver L */
	nlsolver_FastL, /**< @brief nonlinear progressively reordering solver L */
	nlsolver_LambdaDL /**< @brief nonlinear solver lambda with fluid relinearization and Dogleg */
};

/**
 *	@brief a list of all the available and unavailable solvers
 */
typedef MakeTypelist_Safe((
	CSolverTypeIdPair<nlsolver_A, _TySolverA_Name>,
	CSolverTypeIdPair<nlsolver_Lambda, _TySolverLambda_Name>,
	CSolverTypeIdPair<nlsolver_LambdaLM, _TySolverLambdaLM_Name>,
	CSolverTypeIdPair<nlsolver_L, _TySolverL_Name>,
	CSolverTypeIdPair<nlsolver_FastL, _TySolverFastL_Name>,
	CSolverTypeIdPair<nlsolver_LambdaDL, _TySolverLambdaDL_Name>
	)) CCompiledSolverList;
// hint - add new solvers here

#endif // !__SOLVERS_CONFIGURATION_INCLUDED
