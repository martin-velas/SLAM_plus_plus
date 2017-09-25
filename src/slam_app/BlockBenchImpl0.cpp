/*
								+----------------------------------+
								|                                  |
								|    ***  Block benchmarks  ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|       BlockBenchImpl0.cpp        |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/BlockBenchImpl0.cpp
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
 *	@brief decorates arguments of n_Run_BlockBenchmark() to avoid unused variable warnings
 *		if benchmarks are disabled
 */
#ifdef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
#define BBPARAM(x) x
#else // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
#define BBPARAM(x) UNUSED(x)
#endif // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

int n_Run_BlockBenchmark(int BBPARAM(n_block_size),
	const char *BBPARAM(p_s_bench_name), const char *BBPARAM(p_s_bench_type))
{
	/*CBlockMatrixBenchmark::MatrixMultiplication_UnitTest();
	CBlockMatrixBenchmark::MatrixAddition_UnitTest();
	return 0;
	// run unit tests

#ifdef _DEBUG
	CBlockMatrixBenchmark bmb(1, CBlockMatrixBenchmark::order_RectOutline);
#else // _DEBUG
	CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
#endif // _DEBUG
	if(!bmb.Run_AllocationBenchmark("G:\\bench_512")) // pcpolok
	//if(!bmb.Run_AllocationBenchmark("C:\\Users\\ipolok\\Documents\\G\\bench_short_good")) // nbpolok
		fprintf(stderr, "error: benchmark failed\n");
	bmb.ShowResults();
	if(!bmb.Save_ResultSheet("bench_512.csv"))
		fprintf(stderr, "error: i/o error while writing results\n");

	return 0;*/
	/*
	{
		CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
		if(!bmb.Run_AllocationBenchmark("G:\\bench_512")) // pcpolok
			fprintf(stderr, "error: benchmark failed\n");
		bmb.ShowResults();
		if(!bmb.Save_ResultSheet("bench_512.csv"))
			fprintf(stderr, "error: i/o error while writing results\n");
	}
	{
		CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
		if(!bmb.Run_AllocationBenchmark("G:\\bench_1024")) // pcpolok
			fprintf(stderr, "error: benchmark failed\n");
		bmb.ShowResults();
		if(!bmb.Save_ResultSheet("bench_1024.csv"))
			fprintf(stderr, "error: i/o error while writing results\n");
	}
	{
		CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
		if(!bmb.Run_AllocationBenchmark("G:\\bench_2048")) // pcpolok
			fprintf(stderr, "error: benchmark failed\n");
		bmb.ShowResults();
		if(!bmb.Save_ResultSheet("bench_2048.csv"))
			fprintf(stderr, "error: i/o error while writing results\n");
	}
	{
		CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
		if(!bmb.Run_AllocationBenchmark("G:\\bench_4096")) // pcpolok
			fprintf(stderr, "error: benchmark failed\n");
		bmb.ShowResults();
		if(!bmb.Save_ResultSheet("bench_4096.csv"))
			fprintf(stderr, "error: i/o error while writing results\n");
	}
	{
		CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
		if(!bmb.Run_AllocationBenchmark("G:\\bench_8192")) // pcpolok
			fprintf(stderr, "error: benchmark failed\n");
		bmb.ShowResults();
		if(!bmb.Save_ResultSheet("bench_8192.csv"))
			fprintf(stderr, "error: i/o error while writing results\n");
	}
	{
		CBlockMatrixBenchmark bmb(10, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM
		if(!bmb.Run_AllocationBenchmark("G:\\bench_16384")) // pcpolok
			fprintf(stderr, "error: benchmark failed\n");
		bmb.ShowResults();
		if(!bmb.Save_ResultSheet("bench_16384.csv"))
			fprintf(stderr, "error: i/o error while writing results\n");
	}

	return 0;
*/

#ifdef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
	bool b_result = false;
#ifdef _DEBUG
	const int n_divisor = 1000;
#else // _DEBUG
	const int n_divisor = 100;
#endif // _DEBUG
	switch(n_block_size) {
	case 1:
		b_result = CBlockBenchRunner<1>::Run(1000 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 2:
		b_result = CBlockBenchRunner<2>::Run(1000 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 3:
		b_result = CBlockBenchRunner<3>::Run(1000 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 4:
		b_result = CBlockBenchRunner<4>::Run(1000 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 5:
		b_result = CBlockBenchRunner<5>::Run(1000 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	case 6:
		b_result = CBlockBenchRunner<6>::Run(500 / n_divisor, p_s_bench_name, p_s_bench_type);
		break;
	default:
		return n_Run_BlockBenchmark1(n_block_size, p_s_bench_name, p_s_bench_type);
		// sizes 8 and above handled here
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
