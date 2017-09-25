/*
								+----------------------------------+
								|                                  |
								|  ***  2D solvers instances  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|         Solve2DImpl.cpp          |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/Solve2DPoseOnlyImpl.cpp
 *	@brief contains instantiation of SE(2) pose-only solver templates
 *	@author -tHE SWINe-
 *	@date 2013-06-14
 */

/**
 *	@def __SE2_POSE_ONLY_ENABLED
 *	@brief if defined, the solver specializations for pose-only SE(2) are compiled
 */
#define __SE2_POSE_ONLY_ENABLED

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __SE2_POSE_ONLY_ENABLED
#include "slam_app/Main.h"
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h
#include "slam/SE2_Types.h"
#endif // __SE2_POSE_ONLY_ENABLED

int n_Run_SE2PoseOnly_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __SE2_POSE_ONLY_ENABLED
	typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
	typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;
	// define types of vertices, edges

	typedef CFlatSystem<CVertexPose2D, TVertexTypelist,
		CEdgePose2D, TEdgeTypelist, CBasicUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CVertexPose2D, TVertexTypelist
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting SE(2) vertex and edge types

	typedef CSolverCaller<CSystemType, CSE2OnlyPoseEdgeTraits,
		CIgnoreAllVertexTraits, CParseLoop> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __SE2_POSE_ONLY_ENABLED
	fprintf(stderr, "error: pose-only SE(2) is disabled\n");
	return -1;
#endif // __SE2_POSE_ONLY_ENABLED
}
