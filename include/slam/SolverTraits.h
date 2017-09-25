/*
								+-----------------------------------+
								|                                   |
								|  ***  Solvers config traits  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|          SolverTraits.h           |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SOLVER_TRAITS_INCLUDED
#define __SOLVER_TRAITS_INCLUDED

/**
 *	@file include/slam/SolverTraits.h
 *	@author -tHE SWINe-
 *	@brief a file, containing helper traits for ConfigSolvers.h (to keep it clean and short)
 *	@date 2013
 */

#include "slam/IncrementalPolicy.h"

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@brief solver trait class
 *	@tparam CSolver is solver type
 */
template <class CSolver>
class CSolverTraits {
public:
	typedef CSolver _TySolver; /**< @brief solver type */

	/**
	 *	@brief solver interface properties, stored as enum
	 */
	enum {
		solver_HasDump = CSolver::solver_HasDump, /**< @brief timing statistics support flag; solvers that support it have <tt>void Dump(float f_total_time)</tt> */
		solver_HasChi2 = CSolver::solver_HasChi2, /**< @brief Chi2 error calculation support flag; solvers that support it have <tt>double f_Chi_Squared_Error() const</tt> and <tt>double f_Chi_Squared_Error_Denorm()</tt> */
		solver_HasMarginals = CSolver::solver_HasMarginals, /**< @brief marginal covariance support flag; solvers that support it have <tt>CMarginalCovariance &r_MarginalCovariance()</tt> */
		solver_HasGaussNewton = CSolver::solver_HasGaussNewton, /**< @brief Gauss-Newton support flag; no additional interface required, but use of Gauss-Newton in nonlinear solving is expected */
		solver_HasLevenberg = CSolver::solver_HasLevenberg, /**< @brief Levenberg-Marquardt support flag; no additional interface required, but use of Levenberg-Marquardt in nonlinear solving is expected */
		solver_HasGradient = CSolver::solver_HasGradient, /**< @brief gradient-based linear solving support flag; no additional interface required, but use of gradient methods in nonlinear solving is expected */
		solver_HasSchur = CSolver::solver_HasSchur, /**< @brief Schur complement support flag; no additional interface required, but solver must support accelerating linear solving using Schur complement */
		solver_HasDelayedOptimization = CSolver::solver_HasDelayedOptimization, /**< @brief delayed optimization support flag; solvers that support it have <tt>void Delay_Optimization()</tt> and <tt>void Enable_Optimization()</tt> */
		solver_IsPreferredBatch = CSolver::solver_IsPreferredBatch, /**< @brief preferred batch solver flag; no additional interface required, the solver is supposed to be good at batch solving */
		solver_IsPreferredIncremental = CSolver::solver_IsPreferredIncremental, /**< @brief preferred incremental solver flag; no additional interface required, the solver is supposed to be good at incremental solving */
		solver_ExportsJacobian = CSolver::solver_ExportsJacobian, /**< @brief interface for exporting jacobian system matrix flag; solvers that support it have <tt>const CUberBlockMatrix &r_A() const</tt> */
		solver_ExportsHessian = CSolver::solver_ExportsHessian, /**< @brief interface for exporting hessian system matrix flag; solvers that support it have <tt>const CUberBlockMatrix &r_Lambda() const</tt> */
		solver_ExportsFactor = CSolver::solver_ExportsFactor /**< @brief interface for exporting factorized system matrix flag; solvers that support it have <tt>const CUberBlockMatrix &r_R() const</tt>, <tt>const size_t *p_R_Ordering() const</tt> and <tt>size_t n_R_Ordering_Size() const</tt> */
	};
};

/** @} */ // end of group

/**
 *	@brief token, used as a placeholder for solver templates,
 *		that were not included in the build
 *
 *	@tparam CSystem is the system type (unused)
 *	@tparam CLinearSolver is a linear solver (unused)
 */
template <class CSystem, class CLinearSolver/*, class CBlockSizes*/> // *	@tparam CBlockSizes is a list of matrix block sizes (unused)
class CSolverNotIncluded {};

/**
 *	@brief token, used as a placeholder for solver templates,
 *		that are not supported by SE types
 *
 *	@tparam CSystem is the system type (unused)
 *	@tparam CLinearSolver is a linear solver (unused)

 */
template <class CSystem, class CLinearSolver/*, class CBlockSizes*/> // *	@tparam CBlockSizes is a list of matrix block sizes (unused)
class CSolverNotSupported {};

/**
 *	@brief pair of nonlinear solver id and the type, along with traits
 *
 *	@tparam n_solver_type_id is nonlinear solver id
 *	@tparam CNonlinearSolverType is nonlinear solver template name
 */
template <const int n_solver_type_id,
	template <class, class> class CNonlinearSolverType>
class CSolverTypeIdPair {
public:
	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		solver_type_Id = n_solver_type_id /**< @brief nonlinear solver id */
	};

	/**
	 *	@brief runs the application, using this solver
	 *
	 *	@tparam CSystemType is system type (derived from CFlatSystem)
	 *	@tparam CEdgeTraitsType is edge traits template name
	 *	@tparam CVertexTraitsType is vertex traits template name
	 *	@tparam CParseLoopType is parse loop template name
	 *	@tparam CParsedPrimitives is a list of parsed primitives (e.g. CStandardParsedPrimitives)
	 *	@tparam CRunEnvironment is type of environtment the solver is supposed to run in
	 *
	 *	@param[in] t_env is environtment the solver is supposed to run in
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CSystemType, template <class> class CEdgeTraitsType,
		template <class> class CVertexTraitsType,
		template <class, class, template <class> class vcneedsnamehere,
		template <class> class vcneedsnamehereaswell>
		class CParseLoopType, class CParsedPrimitives, class CRunEnvironment>
	static inline bool Run_MainApp(CRunEnvironment t_env) // throw(std::runtime_error, std::bad_alloc)
	{
		return t_env.template Run<CSystemType, CNonlinearSolverType,
			CEdgeTraitsType, CVertexTraitsType, CParseLoopType, CParsedPrimitives>();
		// run with parameters
	}
};

/**
 *	@brief pair of nonlinear solver id and the type,
 *		along with traits (specialization for solvers that were not included)
 *	@tparam n_solver_type_id is nonlinear solver id
 */
template <const int n_solver_type_id>
class CSolverTypeIdPair<n_solver_type_id, CSolverNotIncluded> {
public:
	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		solver_type_Id = n_solver_type_id /**< @brief nonlinear solver id */
	};

	/**
	 *	@brief runs the application, using this solver
	 *
	 *	@tparam CSystemType is system type (derived from CFlatSystem)
	 *	@tparam CEdgeTraitsType is edge traits template name
	 *	@tparam CVertexTraitsType is vertex traits template name
	 *	@tparam CParseLoopType is parse loop template name
	 *	@tparam CParsedPrimitives is a list of parsed primitives (e.g. CStandardParsedPrimitives)
	 *	@tparam CRunEnvironment is type of environtment the solver is supposed to run in
	 *
	 *	@param[in] t_env is environtment the solver is supposed to run in
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CSystemType, template <class> class CEdgeTraitsType,
		template <class> class CVertexTraitsType,
		template <class, class, template <class> class vcneedsnamehere,
		template <class> class vcneedsnamehereaswell>
		class CParseLoopType, class CParsedPrimitives, class CRunEnvironment>
	static inline bool Run_MainApp(CRunEnvironment UNUSED(t_env))
	{
		fprintf(stderr, "error: the selected solver was not included\n");
		return false;
	}
};

/**
 *	@brief pair of nonlinear solver id and the type,
 *		along with traits (specialization for unsupported solvers)
 *	@tparam n_solver_type_id is nonlinear solver id
 */
template <const int n_solver_type_id>
class CSolverTypeIdPair<n_solver_type_id, CSolverNotSupported> {
public:
	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		solver_type_Id = n_solver_type_id /**< @brief nonlinear solver id */
	};

	/**
	 *	@brief runs the application, using this solver
	 *
	 *	@tparam CSystemType is system type (derived from CFlatSystem)
	 *	@tparam CEdgeTraitsType is edge traits template name
	 *	@tparam CVertexTraitsType is vertex traits template name
	 *	@tparam CParseLoopType is parse loop template name
	 *	@tparam CParsedPrimitives is a list of parsed primitives (e.g. CStandardParsedPrimitives)
	 *	@tparam CRunEnvironment is type of environtment the solver is supposed to run in
	 *
	 *	@param[in] t_env is environtment the solver is supposed to run in
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CSystemType, template <class> class CEdgeTraitsType,
		template <class> class CVertexTraitsType,
		template <class, class, template <class> class vcneedsnamehere,
		template <class> class vcneedsnamehereaswell>
		class CParseLoopType, class CParsedPrimitives, class CRunEnvironment>
	static inline bool Run_MainApp(CRunEnvironment UNUSED(t_env))
	{
		fprintf(stderr, "error: the selected solver is not supported by the SE types\n");
		return false;
	}
};

/**
 *	@brief specialzies nonlinear solver template arguments using the defaults
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 */
template <class CSystem, class CLinearSolver>
class CNonlinearSolver_Dummmy_Specializer { // this is here only for the documentation
public:
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
	inline CNonlinearSolver_Dummmy_Specializer(CSystem &r_system,
		TIncrementalSolveSetting t_incremental_config = TIncrementalSolveSetting(),
		TMarginalsComputationPolicy t_marginals_config = TMarginalsComputationPolicy(),
		bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool b_use_schur = false)
	{}
};

#endif // !__SOLVER_TRAITS_INCLUDED
