/*
								+-----------------------------------+
								|                                   |
								|      ***  Configuration  ***      |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|             Config.h              |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SLAMPP_CONFIGURATION_INCLUDED
#define __SLAMPP_CONFIGURATION_INCLUDED

/**
 *	@file include/slam_app/Config.h
 *	@author -tHE SWINe-
 *	@brief main application configuration file
 *	@note Also a precompiled header (makes the compilation faster in windows).
 *	@date 2012
 */

//#include <csparse/cs.hpp>
//#include "eigen/Eigen/Dense"
//#include "eigen/Eigen/Core"
#include <stdio.h>
#include <time.h>
#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h> // ShellExecute()
#endif // _WIN32 || _WIN64
//#include "slam/Debug.h"
//#include "slam/Unused.h"
#include "slam_app/ParsePrimitives.h"
//#include "slam/Timer.h"

/**
 *	@def __PIMP_MY_WINDOW
 *	@brief if defined, looks if there is a second monitor and moves the console window there
 *	@note This only works on windows platforms.
 */
#define __PIMP_MY_WINDOW

/**
 *	@def __COMPILE_LIBRARY_TESTS
 *	@brief if defined, csparse and eigen test functions are compiled (use to debug linking problems)
 */
//#define __COMPILE_LIBRARY_TESTS

/**
 *	@def __USE_NATIVE_CHOLESKY
 *	@brief if defined, the native linear solver is used instead of CSparse
 */
//#define __USE_NATIVE_CHOLESKY

/**
 *	@def __USE_CHOLMOD
 *	@brief if defined, CHOLMOD is used as linear solver instead of CSparse
 *		(only if __USE_NATIVE_CHOLESKY is not defined)
 */
//#define __USE_CHOLMOD

/**
 *	@def __CHOLMOD_SHORT
 *	@brief if defined, CHOLMOD is forced to use 32-bit indices (it is only effective when
 *		in x64 mode, if __USE_CHOLMOD is defined and __USE_NATIVE_CHOLESKY is not)
 */
//#define __CHOLMOD_SHORT

/**
 *	@def __USE_CXSPARSE
 *	@brief if defined, CXSparse is used as linear solver instead of CSparse
 *		(only if __USE_CHOLMOD or __USE_NATIVE_CHOLESKY is not defined)
 */
//#define __USE_CXSPARSE

/**
 *	@def __CXSPARSE_SHORT
 *	@brief if defined, CXSparse is forced to use 32-bit indices (it is only effective when in x64
 *		mode, if __USE_CXSPARSE is defined and __USE_CHOLMOD or __USE_NATIVE_CHOLESKY are not)
 */
//#define __CXSPARSE_SHORT

/**
 *	@def __LINEAR_SOLVER_OVERRIDE
 *	@brief enables linear solver selection from commandline (0 = CSparse, 1 = CXSparse, 2 = CHOLMOD, 3 = native)
 */
#ifdef __LINEAR_SOLVER_OVERRIDE
//#pragma message("__LINEAR_SOLVER_OVERRIDE = " __LINEAR_SOLVER_OVERRIDE)
#ifdef __USE_CHOLMOD
#undef __USE_CHOLMOD
#endif // __USE_CHOLMOD
#ifdef __USE_CXSPARSE
#undef __USE_CXSPARSE
#endif // __USE_CXSPARSE
#if __LINEAR_SOLVER_OVERRIDE == 0
// use CSparse
#elif __LINEAR_SOLVER_OVERRIDE == 1
#define __USE_CXSPARSE
#elif __LINEAR_SOLVER_OVERRIDE == 2
#define __USE_CHOLMOD
#else // __LINEAR_SOLVER_OVERRIDE == 0
#define __USE_NATIVE_CHOLESKY
#endif // __LINEAR_SOLVER_OVERRIDE == 0
#endif // __LINEAR_SOLVER_OVERRIDE
// add override option to easily compile from linux

/**
 *	@def __SE_TYPES_SUPPORT_A_SOLVERS
 *	@brief if defined, SE(2) types implement functions, required by CNonlinearSolver_A
 */
#define __SE_TYPES_SUPPORT_A_SOLVERS

/**
 *	@def __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
 *	@brief if defined, SE(2) types implement functions, required
 *		by CNonlinearSolver_L and CNonlinearSolver_Lambda
 */
#define __SE_TYPES_SUPPORT_LAMBDA_SOLVERS

/**
 *	@def __SE_TYPES_SUPPORT_L_SOLVERS
 *	@brief if defined, SE(2) types implement functions, required by CNonlinearSolver_L
 */
#define __SE_TYPES_SUPPORT_L_SOLVERS // disable L for the reduction plan debugging

//#include "slam/Segregated.h"
//#include "slam/BlockMatrix.h"
//#include "slam/Tetris.h" // disable to speed up builds
#ifdef __SLAM_APP_ENABLE_UNIT_TESTS
#include "slam_app/BlockUnit.h" // typically not needed by users
#endif // __SLAM_APP_ENABLE_UNIT_TESTS
#ifdef __SLAM_APP_ENABLE_PERF_TESTS
#include "slam_app/BlockBench.h" // typically not needed by users
#endif // __SLAM_APP_ENABLE_PERF_TESTS
//#include "slam/FlatSystem.h"
//#include "slam/ParseLoop.h"
#include "slam/LinearSolver_UberBlock.h" // always include
#if defined(__USE_NATIVE_CHOLESKY)
// ...
#elif defined(__USE_CHOLMOD)
#include "slam/LinearSolver_CholMod.h"
#elif defined __USE_CXSPARSE
#include "slam/LinearSolver_CXSparse.h"
#else // __USE_NATIVE_CHOLESKY
#include "slam/LinearSolver_CSparse.h"
#endif // __USE_NATIVE_CHOLESKY
#include "slam/LinearSolver_Schur.h" // always include
// rarely change (pch optimization)

#endif // !__SLAMPP_CONFIGURATION_INCLUDED
