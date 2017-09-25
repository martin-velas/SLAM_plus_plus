/*
								+----------------------------------+
								|                                  |
								|    ***  Block benchmarks  ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|       BlockBenchImpl1.cpp        |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/BlockBenchImpl1.cpp
 *	@brief contains instantiation of some of the block benchmark templates
 *	@author -tHE SWINe-
 *	@date 2013-06-14
 */

#include "slam_app/Config.h"
#ifdef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
#include "slam_app/Main.h"
#endif // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

/**
 *	@def BBPARAM
 *	@brief decorates arguments of n_Run_BlockBenchmark1() to avoid unused variable warnings
 *		if benchmarks are disabled
 */
#ifdef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
#define BBPARAM(x) x
#else // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
#define BBPARAM(x) UNUSED(x)
#endif // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

int n_Run_BlockBenchmark1(int BBPARAM(n_block_size),
	const char *BBPARAM(p_s_bench_name), const char *BBPARAM(p_s_bench_type))
{
#ifdef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
	bool b_result = false;
#ifdef _DEBUG
	const int n_divisor = 1000;
#else // _DEBUG
	const int n_divisor = 100;
#endif // _DEBUG
	switch(n_block_size) {
	case 8:
		b_result = CBlockBenchRunner<8>::Run(500 / n_divisor, p_s_bench_name, p_s_bench_type); // 1000 is too long
		break;
	case 10:
		b_result = CBlockBenchRunner<10>::Run(500 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 15:
		b_result = CBlockBenchRunner<15>::Run(100 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 16:
		b_result = CBlockBenchRunner<16>::Run(100 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 20:
		b_result = CBlockBenchRunner<20>::Run(50 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 25:
		b_result = CBlockBenchRunner<25>::Run(50 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 30:
		b_result = CBlockBenchRunner<30>::Run(50 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	default:
		fprintf(stderr, "warning: unexpected block size: can't run block becnhmarks\n");
		return -1;
	};
	if(!b_result)
		return -1;
	// use bench runner
#else // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
	fprintf(stderr, "error: BlockBench.h not included, can't run matrix benchmarks\n");
	return -1;
#endif // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

	return 0;
}
