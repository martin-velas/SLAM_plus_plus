/*
								+----------------------------------+
								|                                  |
								|     *** ROCV SLAM solver ***     |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|        SolveROCVImpl.cpp         |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/SolveROCVImpl.cpp
 *	@brief range-only, constant velocity SLAM solver
 *	@author -tHE SWINe-
 *	@date 2014-08-08
 */

/**
 *	@def __ROCV_ENABLED
 *	@brief if defined, the solver specializations for ROCV are compiled
 */
#define __ROCV_ENABLED

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __ROCV_ENABLED
#include "slam_app/Main.h"
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h
#include "slam/ROCV_Types.h"
#endif // __ROCV_ENABLED

int n_Run_ROCV_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __ROCV_ENABLED
	typedef MakeTypelist_Safe((CVertexPositionVelocity3D, CVertexLandmark3D)) TVertexTypelist_ROCV;
	typedef MakeTypelist_Safe((CEdgePosVel_Landmark3D<>,
		CEdgeConstVelocity3D<>, CEdgeLandmark3DPrior)) TEdgeTypelist_ROCV;
	// define types of vertices, edges

	typedef CFlatSystem<CBaseVertex, TVertexTypelist_ROCV,
		CBaseEdge, TEdgeTypelist_ROCV, CNullUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CBaseVertex, TVertexTypelist_ROCV
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting ROCV vertex and edge types

	typedef CSolverCaller<CSystemType, CROCVEdgeTraits,
		CROCVVertexTraits, CParseLoop> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __ROCV_ENABLED
	fprintf(stderr, "error: ROCV (range-only constant velocity) is disabled\n");
	return -1;
#endif // __ROCV_ENABLED
}
