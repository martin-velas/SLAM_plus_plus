/*
								+----------------------------------+
								|                                  |
								|  ***  3D solvers instances  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|     Solve3DPoseOnlyImpl.cpp      |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/Solve3DPoseOnlyImpl.cpp
 *	@brief contains instantiation of pose-only SE(3) solver templates
 *	@author -tHE SWINe-
 *	@date 2014-05-23
 */

/**
 *	@def __SE3_POSE_ONLY_ENABLED
 *	@brief if defined, the solver specializations for SE(3) are compiled
 */
#define __SE3_POSE_ONLY_ENABLED

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __SE3_POSE_ONLY_ENABLED
#include "slam_app/Main.h"
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h
#include "slam/SE3_Types.h"
#endif // __SE3_POSE_ONLY_ENABLED

int n_Run_SE3PoseOnly_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __SE3_POSE_ONLY_ENABLED
	_ASSERTE(t_args.b_pose_only);

	typedef MakeTypelist_Safe((CVertexPose3D)) TVertexTypelist_SE3;
	typedef MakeTypelist_Safe((CEdgePose3D)) TEdgeTypelist_SE3;
	// define types of vertices, edges

	typedef CFlatSystem<CVertexPose3D, TVertexTypelist_SE3,
		CEdgePose3D, TEdgeTypelist_SE3, CBasicUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CVertexPose3D, TVertexTypelist_SE3
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting SE(3) vertex and edge types

	typedef CSolverCaller<CSystemType, CSE3OnlyPoseEdgeTraits,
		CIgnoreAllVertexTraits, CParseLoop> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __SE3_POSE_ONLY_ENABLED
	fprintf(stderr, "error: pose-only SE(3) is disabled\n");
	return -1;
#endif // __SE3_POSE_ONLY_ENABLED
}
