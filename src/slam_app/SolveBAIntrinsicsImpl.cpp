/*
								+----------------------------------+
								|                                  |
								|  ***  BA solvers instances  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|      SolveBAIntrinsicsImpl.cpp   |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/SolveBAIntrinsicsImpl.cpp
 *	@brief contains instantiation of bundle adjustment solver templates (version with shared intrinsics)
 *	@author -tHE SWINe- + soso
 *	@date 2015-03-05
 */

/**
 *	@def __BA_INTRINSICS_ENABLED
 *	@brief if defined, the solver specializations for BA with explicit intrinsics vertices are compiled
 */
#define __BA_INTRINSICS_ENABLED

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __BA_INTRINSICS_ENABLED
#include "slam_app/Main.h"
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h
#include "slam/BA_Types.h"
//#include "slam/SE3_Types.h"
#endif // __BA_ENABLED

int n_Run_BA_Intrinsics_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __BA_INTRINSICS_ENABLED
	_ASSERTE(t_args.b_use_BAI);
	typedef MakeTypelist_Safe((CVertexCam, CVertexXYZ, CVertexIntrinsics)) TVertexTypelist_BAI;
	typedef MakeTypelist_Safe((CEdgeP2CI3D)) TEdgeTypelist_BAI;
	// define types of vertices, edges

	typedef CFlatSystem<CBaseVertex, TVertexTypelist_BAI,
			CEdgeP2CI3D, TEdgeTypelist_BAI, CBasicUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CBaseVertex, TVertexTypelist_BAI
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting BA vertex and edge types

	typedef CSolverCaller<CSystemType, CBAIntrinsicsEdgeTraits,
		CBAIntrinsicsVertexTraits, CParseLoop> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __BA_INTRINSICS_ENABLED
	fprintf(stderr, "error: BA with intrinsics is disabled\n");
	return -1;
#endif // __BA_INTRINSICS_ENABLED
}
