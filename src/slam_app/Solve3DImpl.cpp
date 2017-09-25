/*
								+----------------------------------+
								|                                  |
								|  ***  3D solvers instances  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|         Solve3DImpl.cpp          |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/Solve3DImpl.cpp
 *	@brief contains instantiation of SE(3) solver templates
 *	@author -tHE SWINe-
 *	@date 2013-06-14
 */

/**
 *	@def __SE3_ENABLED
 *	@brief if defined, the solver specializations for SE(3) are compiled
 */
#define __SE3_ENABLED

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __SE3_ENABLED
#include "slam_app/Main.h"
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h
#include "slam/SE3_Types.h"
#endif // __SE3_ENABLED

int n_Run_SE3_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __SE3_ENABLED
	_ASSERTE(!t_args.b_pose_only);

	typedef MakeTypelist_Safe((CVertexPose3D, CVertexLandmark3D)) TVertexTypelist_SE3;
	typedef MakeTypelist_Safe((CEdgePose3D, CEdgePoseLandmark3D/*, CEdgePose3D_Ternary*/)) TEdgeTypelist_SE3; // testing hyperedges
	// define types of vertices, edges

	typedef CFlatSystem<CBaseVertex, TVertexTypelist_SE3,
		CBaseEdge, TEdgeTypelist_SE3, CBasicUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CBaseVertex, TVertexTypelist_SE3
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting SE(3) vertex and edge types

	typedef CSolverCaller<CSystemType, CSE3LandmarkPoseEdgeTraits,
		CIgnoreAllVertexTraits, CParseLoop> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __BA_ENABLED
	fprintf(stderr, "error: SE(3) is disabled\n");
	return -1;
#endif // __SE3_ENABLED
}
