/*
								+-----------------------------------+
								|                                   |
								|   ***  Spherical BA solver  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|       SolveSpheronImpl.cpp        |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam_app/SolveSpheronImpl.cpp
 *	@brief contains instantiation of spherical bundle adjustment solver templates
 *	@author isolony
 *	@date 2014-07-02
 */

/**
 *	@def __SPHERON_ENABLED
 *	@brief if defined, the solver specializations for BA are compiled
 */
#define __SPHERON_ENABLED

// Build Time 0:00
/*
#include "slam_app/Main.h"
	//#include "slam_app/Config.h"
		//#include "slam_app/ParsePrimitives.h"
			//#include "slam/Parser.h" // Build Time 0:04
				//#include "slam/TypeList.h" // Build Time 0:00
				//#include "eigen/Eigen/Core" // Build Time 0:03
			//#include "slam/2DSolverBase.h" // CBase2DSolver::Absolute_to_Relative() and such
			// Build Time 0:04
			//#include "slam/BlockMatrix.h"
			// Build Time 0:05
			//#include "slam/3DSolverBase.h" // CBase3DSolver::Absolute_to_Relative() and such
			// Build Time 0:06
		// Build Time 0:06
		//#include "slam/LinearSolver_UberBlock.h" // always include
		// Build Time 0:06
		//#include "slam/LinearSolver_Schur.h" // always include
			//#include "slam/LinearSolverTags.h"
			// Build Time 0:06
		// Build Time 0:07
	// Build Time 0:07
	//#include "slam/ConfigSolvers.h"
	// Build Time 0:07
// Build Time 0:07
#include "slam/ConfigSolvers.h"
// Build Time 0:07 (when all solvers disabled)
*/

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __SPHERON_ENABLED
#include "slam_app/Main.h" // Build Time 0:12 | Build Time 0:07 (when all solvers disabled), +1 for A, +5 for lambda, +1 for lambda LM, +5 for fastL
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h

// Build Time 0:12 | Build Time 0:07 (when all solvers disabled)

#include "slam/BA_Types.h"

// Build Time 0:36 | Build Time 0:28 (when all solvers disabled)

//#include "slam/SE3_Types.h"
#endif // __SPHERON_ENABLED

int n_Run_Spheron_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __SPHERON_ENABLED
	typedef MakeTypelist_Safe((CVertexSpheron, CVertexXYZ)) TVertexTypelist_BA;
	typedef MakeTypelist_Safe((CEdgeSpheronXYZ)) TEdgeTypelist_BA;
	// define types of vertices, edges

// Build Time 0:37 | Build Time 0:29 (when all solvers disabled)

	typedef CFlatSystem<CBaseVertex, TVertexTypelist_BA,
		CEdgeSpheronXYZ, TEdgeTypelist_BA, CNullUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CBaseVertex, TVertexTypelist_BA
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting BA vertex and edge types

// Build Time 0:38 | Build Time 0:29 (when all solvers disabled)

	typedef CSolverCaller<CSystemType, CSpheronEdgeTraits,
		CSpheronVertexTraits, CParseLoop> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

// Build Time 0:37 | Build Time 0:28 (when all solvers disabled)

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __SPHERON_ENABLED
	fprintf(stderr, "error: Spheron is disabled\n");
	return -1;
#endif // __SPHERON_ENABLED
}

// Build Time 3:26 | Build Time 0:29 (when all solvers disabled)
// Build Time 1:37 (A solver only)
// Build Time 3:10 (lambda solver only)
// Build Time 3:09 (lambda LM solver only)
// Build Time 2:47 (fastL solver only)
