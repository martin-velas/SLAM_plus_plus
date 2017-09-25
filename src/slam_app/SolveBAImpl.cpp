/*
								+----------------------------------+
								|                                  |
								|  ***  BA solvers instances  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|         SolveBAImpl.cpp          |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/SolveBAImpl.cpp
 *	@brief contains instantiation of bundle adjustment solver templates
 *	@author -tHE SWINe-
 *	@date 2013-06-14
 */

/**
 *	@def __BA_ENABLED
 *	@brief if defined, the solver specializations for BA are compiled
 */
#define __BA_ENABLED

#include <stdio.h>
struct TCommandLineArgs; // forward declaration
#ifdef __BA_ENABLED
#include "slam_app/Main.h"
//#include "slam/ConfigSolvers.h" // included via slam_app/Main.h
#include "slam/BA_Types.h"
//#include "slam/SE3_Types.h"
#include "slam_app/IncBAParsePrimitives.h"
#endif // __BA_ENABLED

int n_Run_BA_Solver(const TCommandLineArgs &t_args) // throw(std::runtime_error, std::bad_alloc)
{
#ifdef __BA_ENABLED
	_ASSERTE(t_args.b_use_BA);
	typedef MakeTypelist_Safe((CVertexCam, CVertexXYZ)) TVertexTypelist_BA;
	typedef MakeTypelist_Safe((CEdgeP2C3D)) TEdgeTypelist_BA;
	// define types of vertices, edges

	typedef CTypelist<CConsistencyMarker_ParsePrimitive, CStandardParsedPrimitives> CBAParsedPrimitives;
	// use one more parsed primitive to signal points where incremental optimization should be performed

	typedef CFlatSystem<CBaseVertex, TVertexTypelist_BA,
			CEdgeP2C3D, TEdgeTypelist_BA, CBasicUnaryFactorFactory
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
		, CBaseVertex, TVertexTypelist_BA
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
		> CSystemType;
	// make a system permitting BA vertex and edge types

	typedef CSolverCaller<CSystemType, CBAEdgeTraits,
		CBAVertexTraits, CParseLoop_ConsistencyMarker, CBAParsedPrimitives> CSpecializedSolverCaller;
	// specify how the nonlinear solver should be called

	return CTypelistForEach<CCompiledSolverList,
		CSpecializedSolverCaller>::Run(CSpecializedSolverCaller(t_args)).n_Result();
#else // __BA_ENABLED
	fprintf(stderr, "error: BA is disabled\n");
	return -1;
#endif // __BA_ENABLED
}
