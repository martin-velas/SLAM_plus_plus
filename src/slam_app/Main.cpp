/*
								+-----------------------------------+
								|                                   |
								|        ***  SLAM Main  ***        |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam_app/Main.cpp
 *	@brief contains the main() function of the SLAM program
 *	@author -tHE SWINe-
 *	@date 2012-03-30
 */

int n_dummy_param = 0;
/**< @brief a dummy parameter, used as a convenient commandline input, intended for debugging / testing */

#include "slam_app/Main.h"

/**
 *	@def p_s_Number_Suffix
 *	@brief gets a suffix for a number
 *	@param[in] n_x is a number for which the suffix is to be returned, must be positive
 *	@return Returns suffix for the given number.
 */
#define p_s_Number_Suffix(n_x) (((n_x + 9) % 10 > 2 || (n_x > 10 && n_x < 20))? "th" : \
	((n_x) % 10 == 1)? "st" : ((n_x) % 10 == 2)? "nd" : "rd")

/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int n_arg_num, const char **p_arg_list)
{
#if (defined(_WIN32) || defined(_WIN64)) && defined(__PIMP_MY_WINDOW)
	{
		char p_s_title[256];
		sprintf(p_s_title, "title slampp%04x", GetCurrentProcessId()); // use unique id
		system(p_s_title);
		HWND h_console = FindWindow(0, p_s_title + /*strlen("title ")*/6);
		system("title SLAM++\n");
		int n_all_mons_width = GetSystemMetrics(SM_CXVIRTUALSCREEN);
		int n_all_mons_height = GetSystemMetrics(SM_CYVIRTUALSCREEN);
		int n_pri_mon_width = GetSystemMetrics(SM_CXSCREEN);
		int n_pri_mon_height = GetSystemMetrics(SM_CYSCREEN);
		if(n_all_mons_width > n_pri_mon_width + n_pri_mon_width / 2) {
			RECT t_rect;
			GetWindowRect(h_console, &t_rect);
			int n_pri_mon_org = GetSystemMetrics(SM_XVIRTUALSCREEN); // handle situations where the coordinates do not start at (0, 0)
			if(n_pri_mon_org >= 0 && t_rect.left - n_pri_mon_org < n_pri_mon_width) // running a batch job? don't float away
				SetWindowPos(h_console, HWND_TOP, t_rect.left + n_pri_mon_width, t_rect.top, 0, 0, SWP_NOSIZE);
			else if(n_pri_mon_org < 0 && t_rect.left - n_pri_mon_width > n_pri_mon_org) // running a batch job? don't float away
				SetWindowPos(h_console, HWND_TOP, t_rect.left - n_pri_mon_width, t_rect.top, 0, 0, SWP_NOSIZE);
		} else if(n_all_mons_height > n_pri_mon_height + n_pri_mon_height / 2) {
			RECT t_rect;
			GetWindowRect(h_console, &t_rect);
			int n_pri_mon_org = GetSystemMetrics(SM_YVIRTUALSCREEN); // handle situations where the coordinates do not start at (0, 0)
			if(n_pri_mon_org >= 0 && t_rect.top - n_pri_mon_org < n_pri_mon_height) // running a batch job? don't float away
				SetWindowPos(h_console, HWND_TOP, t_rect.left, t_rect.top + n_pri_mon_height, 0, 0, SWP_NOSIZE);
			else if(n_pri_mon_org < 0 && t_rect.top - n_pri_mon_height > n_pri_mon_org) // running a batch job? don't float away
				SetWindowPos(h_console, HWND_TOP, t_rect.left, t_rect.top - n_pri_mon_height, 0, 0, SWP_NOSIZE);
		}
	}
	// windows hack - make the console window appear on the secondary monitor
#endif // (_WIN32 || _WIN64) && __PIMP_MY_WINDOW

	TCommandLineArgs t_cmd_args;
	t_cmd_args.Defaults(); // set defaults
	if(!t_cmd_args.Parse(n_arg_num, p_arg_list))
		return -1;
	// parse commandline

#ifdef _OPENMP
	if(t_cmd_args.n_omp_threads != size_t(-1))
		omp_set_num_threads(int(t_cmd_args.n_omp_threads)); // can use this to set no. of threads
	if(t_cmd_args.b_omp_dynamic)
		omp_set_dynamic(true); // dynamically allocate threads
#endif // _OPENMP

	if(t_cmd_args.b_run_matrix_unit_tests) {
#ifdef __UBER_BLOCK_MATRIX_UNIT_TESTS_INCLUDED
		CBlockMatrixUnitTests::RunAll();
		return 0;
#else // __UBER_BLOCK_MATRIX_UNIT_TESTS_INCLUDED
		fprintf(stderr, "warning: unit tests not included: skipping\n");
		return -1;
#endif // __UBER_BLOCK_MATRIX_UNIT_TESTS_INCLUDED
	}
	// run unit tests

	if(t_cmd_args.b_run_matrix_benchmarks)
		return n_Run_BlockBenchmark(n_dummy_param, t_cmd_args.p_s_bench_name, t_cmd_args.p_s_bench_type);
	// run benchmarks (allow without input file)

	if(!t_cmd_args.p_s_input_file) {
		fprintf(stderr, "error: no input file specified; find help below:\n");
		PrintHelp();
		return -1;
	}
	if(t_cmd_args.n_linear_solve_each_n_steps && t_cmd_args.n_nonlinear_solve_each_n_steps &&
	   t_cmd_args.n_nonlinear_solve_each_n_steps < t_cmd_args.n_linear_solve_each_n_steps) {
		fprintf(stderr, "warning: nonlinear solver is scheduled to run every %d%s step, "
			"linear solver runs every %d%s step. the nonlinear solver will never run.\n",
			int(t_cmd_args.n_linear_solve_each_n_steps),
			p_s_Number_Suffix(t_cmd_args.n_linear_solve_each_n_steps),
			int(t_cmd_args.n_nonlinear_solve_each_n_steps),
			p_s_Number_Suffix(t_cmd_args.n_nonlinear_solve_each_n_steps));
	}
	if(t_cmd_args.n_solver_choice != nlsolver_LambdaDL) {
		if(t_cmd_args.b_dogleg_all_batch)
			fprintf(stderr, "warning: dogleg solver not selected, --dogleg-all-batch ignored\n");
		if(t_cmd_args.f_dogleg_step_size != -1)
			fprintf(stderr, "warning: dogleg solver not selected, --dogleg-step-size ignored\n");
		if(t_cmd_args.f_dogleg_step_threshold != -1)
			fprintf(stderr, "warning: dogleg solver not selected, --dogleg-step-threshold ignored\n");
	}
	// check inputs

	if(t_cmd_args.b_show_commandline) {
		printf("> ./SLAM_plus_plus");
		for(int i = 1; i < n_arg_num; ++ i)
			printf(" %s", p_arg_list[i]);
		printf("\n");
	}
	// display commandline

	if(t_cmd_args.b_show_flags)
		DisplaySwitches();
	// display switches

	{
		FILE *p_fr;
		if((p_fr = fopen(t_cmd_args.p_s_input_file, "rb")))
			fclose(p_fr);
		else {
			fprintf(stderr, "error: can't open input file \'%s\'\n", t_cmd_args.p_s_input_file);
			return -1;
		}
	}
	// see if the input is there; otherwise will get some misleading errors about peek-parsing
	// and potentially about SE(2) (or other default solver) being disabled (if unlucky)

	TDatasetPeeker peek(t_cmd_args.p_s_input_file, t_cmd_args.n_max_lines_to_process);
	if(peek.b_has_rocv) {
		t_cmd_args.b_use_rocv = true;
		if(t_cmd_args.b_verbose)
			fprintf(stderr, "detected ROCV dataset\n");
	}
	if(!peek.b_has_rocv) { // ROCV reuses 3D landmarks for parsing
		if(t_cmd_args.b_pose_only && (peek.b_has_landmark || peek.b_has_ba)) { // BA implies landmarks
			fprintf(stderr, "error: --pose-only flag detected, but the system has landmarks (flag cleared)\n");
			t_cmd_args.b_pose_only = false;
		} else if(!t_cmd_args.b_pose_only && !peek.b_has_landmark && !peek.b_has_ba && !peek.b_has_ba_stereo && !peek.b_has_ba_intrinsics) { // BA implies landmarks
			fprintf(stderr, "warning: the system does not seem to have landmarks"
				" and --pose-only flag not present: could run faster\n");
		}
	}
	if(!peek.b_has_rocv && (peek.b_has_edge3d || peek.b_has_vertex3d)) { // ROCV reuses 3D landmarks for parsing
		t_cmd_args.b_use_SE3 = true;
		if(t_cmd_args.b_verbose)
			fprintf(stderr, "detected SE3\n");
		if(peek.b_has_landmark) {
			if(t_cmd_args.b_verbose)
				fprintf(stderr, "detected 3D landmarks\n");
		}
	}
	if(!peek.b_has_rocv && peek.b_has_ba) { // ROCV reuses 3D landmarks for parsing
		t_cmd_args.b_use_BA = true;
		if(t_cmd_args.b_verbose)
			fprintf(stderr, "detected BA\n");
	}
	if(peek.b_has_ba_stereo) {
		t_cmd_args.b_use_BAS = true;
		if(t_cmd_args.b_verbose)
			fprintf(stderr, "detected BA stereo\n");
	}
	if(peek.b_has_spheron) {
		t_cmd_args.b_use_spheron = true;
		if(t_cmd_args.b_verbose)
			fprintf(stderr, "detected Spheron dataset\n");
	}
	if(!peek.b_has_rocv && peek.b_has_ba_intrinsics) { // ROCV reuses 3D landmarks for parsing
		t_cmd_args.b_use_BAI = true;
		if(t_cmd_args.b_verbose)
			fprintf(stderr, "detected BA with optimized intrinsic camera parameters\n");
	}
	// detect landmarks, clear b_10k_opts where it would cause the code to crash

	if(t_cmd_args.b_use_old_system) {
		fprintf(stderr, "error: the legacy solver was not compiled\n");
		return -1;
	} else {
		try {
			if(t_cmd_args.b_use_BA || t_cmd_args.b_use_BAS ||
			   t_cmd_args.b_use_BAI || t_cmd_args.b_use_spheron) {
				if(t_cmd_args.n_solver_choice == nlsolver_Lambda)
					t_cmd_args.n_solver_choice = nlsolver_LambdaLM;
			}
			// use Levenberg-Marquardt for bundle adjustment, GN not good enough

			if(t_cmd_args.b_use_BA || t_cmd_args.b_use_BAS || t_cmd_args.b_use_BAI) { // BA mono / stereo
				if(t_cmd_args.b_use_BAI && !t_cmd_args.b_use_BAS) {
					if(n_Run_BA_Intrinsics_Solver(t_cmd_args)) // mono + intrinsics
						return -1;
				} else if(t_cmd_args.b_use_BA && !t_cmd_args.b_use_BAS) {
					if(n_Run_BA_Solver(t_cmd_args)) // mono
						return -1;
				} else {
					_ASSERTE(t_cmd_args.b_use_BAS);
					if(n_Run_BA_Stereo_Solver(t_cmd_args)) // stereo
						return -1;
				}
			} else if(t_cmd_args.b_use_spheron) { // Spheron
				if(n_Run_Spheron_Solver(t_cmd_args))
					return -1;
			} else if(t_cmd_args.b_use_rocv) { // ROCV
				if(n_Run_ROCV_Solver(t_cmd_args))
					return -1;
			} else if(t_cmd_args.b_use_SE3) { // SE(3)
				if(t_cmd_args.b_pose_only) {
					if(n_Run_SE3PoseOnly_Solver(t_cmd_args)) // pose-only
						return -1;
				} else {
					if(n_Run_SE3_Solver(t_cmd_args)) // pose-landmarks
						return -1;
				}
			} else { // SE(2)
				if(t_cmd_args.b_pose_only) {
					if(n_Run_SE2PoseOnly_Solver(t_cmd_args)) // pose-only
						return -1;
				} else {
					if(n_Run_SE2_Solver(t_cmd_args)) // pose-landmarks
						return -1;
				}
			}
			// a simple selection of solver and problem type, without all the ifdef mess
		} catch(std::runtime_error &r_exc) {
			fprintf(stderr, "error: uncaught runtime_error: \'%s\'\n", r_exc.what());
			return -1;
		} catch(std::bad_alloc &r_exc) {
			fprintf(stderr, "error: uncaught bad_alloc: \'%s\'\n", r_exc.what());
			return -1;
		} catch(std::exception &r_exc) {
			fprintf(stderr, "error: uncaught exception: \'%s\'\n", r_exc.what());
			return -1;
		}
	}

	if(!t_cmd_args.b_no_show) { // to get rid of the "set but not used" warning
#if defined(_WIN32) || defined(_WIN64)
		/*char p_s_class_name[256];
		GetClassName(FindWindow(0, "XnView - [solution.tga]"), p_s_class_name, 255);
		printf("classname is: \'%s\'\n", p_s_class_name);*/
		// this is how you get classname of your image browser window

		if(!FindWindow("XmainClass", 0))
			ShellExecute(0, "open", "solution.tga", 0, 0, SW_SHOW);
		else
			fprintf(stderr, "warning: XnView already running: new window not opened\n");
		// detect if image browser is running
#endif // _WIN32 || _WIN64
	}
	// maybe a bad idea ...

	return 0;
}

#include "slam/BaseTypes.h" // to display some of the flags
#include "slam/BlockMatrixBase.h"

/**
 *	@brief prints all the important compiler / optimization switches this app was built with
 */
void DisplaySwitches()
{
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	printf("SLAM++ version x64 (compiled at %s)\nbuilt with the following flags:\n", __DATE__);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	printf("SLAM++ version x86 (compiled at %s)\nbuilt with the following flags:\n", __DATE__);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

#ifdef _DEBUG
	printf("%s\n", "_DEBUG");
#endif // _DEBUG
#ifdef __FAST_MATH__
	printf("%s\n", "__FAST_MATH__");
#endif // __FAST_MATH__
#ifdef _OPENMP
#pragma omp parallel
#pragma omp master
	{
		printf("%s (%d threads)\n", "_OPENMP", omp_get_num_threads());
	}
#endif // _OPENMP

#ifdef __USE_NATIVE_CHOLESKY
	printf("%s\n", "__USE_NATIVE_CHOLESKY");
#elif defined(__USE_CHOLMOD)
	printf("%s\n", "__USE_CHOLMOD");
#ifdef __CHOLMOD_SHORT
	printf("%s\n", "__CHOLMOD_SHORT");
#endif // __CHOLMOD_SHORT
#else // __USE_CHOLMOD
#ifdef __USE_CXSPARSE
	printf("%s\n", "__USE_CXSPARSE");
#ifdef __CXSPARSE_SHORT
	printf("%s\n", "__CXSPARSE_SHORT");
#endif // __CXSPARSE_SHORT
#else // __USE_CXSPARSE
	printf("%s\n", "__USE_CSPARSE");
#endif // __USE_CXSPARSE
#endif // __USE_CHOLMOD

	const char *p_stacked_fmt[] = {"%s", ", %s", ",\n%s"},
		*p_stacked_fmtB[] = {"%s=%d", ", %s=%d", ",\n%s=%d"};
	const int n_stacking_cols = 3;
	int n_stacked_fmt = 0; 
#define STACKED_FMT (p_stacked_fmt[(++ n_stacked_fmt - 1)? ((n_stacked_fmt - 1) % n_stacking_cols)? 1 : 2 : 0])
#define STACKED_FMTB (p_stacked_fmtB[(++ n_stacked_fmt - 1)? ((n_stacked_fmt - 1) % n_stacking_cols)? 1 : 2 : 0])
	// utility for stacking tokens into simple paragraphs

#ifdef GPU_BLAS
	printf("%s\n", "GPU_BLAS");
#endif // GPU_BLAS

#define MKSTRING(a) EXPANDSTRING(a)
#define EXPANDSTRING(a) #a
	printf(STACKED_FMT, "EIGEN_" MKSTRING(EIGEN_WORLD_VERSION) "." MKSTRING(EIGEN_MAJOR_VERSION)
		"." MKSTRING(EIGEN_MINOR_VERSION));
	// always print Eigen version, now there are two to choose from

#ifdef EIGEN_VECTORIZE
	printf(STACKED_FMT, "EIGEN_VECTORIZE");
#endif // EIGEN_VECTORIZE
#ifdef EIGEN_UNALIGNED_VECTORIZE
	printf(STACKED_FMTB, "EIGEN_UNALIGNED_VECTORIZE", int(EIGEN_UNALIGNED_VECTORIZE));
#endif // EIGEN_UNALIGNED_VECTORIZE
#ifdef EIGEN_VECTORIZE_SSE
	printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE");
#endif // EIGEN_VECTORIZE_SSE
#ifdef EIGEN_VECTORIZE_SSE2
	printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE2");
#endif // EIGEN_VECTORIZE_SSE2
#ifdef EIGEN_VECTORIZE_SSE3
	printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE3");
#endif // EIGEN_VECTORIZE_SSE3
#ifdef EIGEN_VECTORIZE_SSSE3
	printf(STACKED_FMT, "EIGEN_VECTORIZE_SSSE3");
#endif // EIGEN_VECTORIZE_SSSE3
#ifdef EIGEN_VECTORIZE_SSE4_1
	printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE4_1");
#endif // EIGEN_VECTORIZE_SSE4_1
#ifdef EIGEN_VECTORIZE_SSE4_2
	printf(STACKED_FMT, "EIGEN_VECTORIZE_SSE4_2");
#endif // EIGEN_VECTORIZE_SSE4_2
#ifdef EIGEN_VECTORIZE_FMA
	printf(STACKED_FMT, "EIGEN_VECTORIZE_FMA");
#endif // EIGEN_VECTORIZE_FMA
#ifdef EIGEN_VECTORIZE_AVX
	printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX");
#endif // EIGEN_VECTORIZE_AVX
#ifdef EIGEN_VECTORIZE_AVX2
	printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX2");
#endif // EIGEN_VECTORIZE_AVX2
#ifdef EIGEN_VECTORIZE_AVX512
	printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX512");
#endif // EIGEN_VECTORIZE_AVX512
#ifdef EIGEN_VECTORIZE_AVX512DQ
	printf(STACKED_FMT, "EIGEN_VECTORIZE_AVX512DQ");
#endif // EIGEN_VECTORIZE_AVX512DQ
#ifdef EIGEN_VECTORIZE_NEON
	printf(STACKED_FMT, "EIGEN_VECTORIZE_NEON");
#endif // EIGEN_VECTORIZE_NEON
	if(n_stacked_fmt)
		printf("\n");
	// print Eigen flags stacked

#ifdef __DISABLE_GPU
	printf("%s\n", "__DISABLE_GPU");
#endif // __DISABLE_GPU
#ifdef __SCHUR_USE_DENSE_SOLVER
	printf("%s\n", "__SCHUR_USE_DENSE_SOLVER");
#endif // __SCHUR_USE_DENSE_SOLVER
#ifdef __SCHUR_DENSE_SOLVER_USE_GPU
	printf("%s\n", "__SCHUR_DENSE_SOLVER_USE_GPU");
#endif // __SCHUR_DENSE_SOLVER_USE_GPU

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
	printf("%s\n", "__BLOCK_BENCH_DUMP_MATRIX_IMAGES");
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
	printf("%s\n", "__BLOCK_BENCH_BLOCK_TYPE_A");
#else // __BLOCK_BENCH_BLOCK_TYPE_A
	printf("%s\n", "__BLOCK_BENCH_BLOCK_TYPE_B"); // nonexistent, but want to know
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
#ifdef __BLOCK_BENCH_CHOLESKY_USE_AMD
	printf("%s\n", "__BLOCK_BENCH_CHOLESKY_USE_AMD");
#endif // __BLOCK_BENCH_CHOLESKY_USE_AMD
#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
	printf("%s\n", "__BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT");
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
#ifdef __BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES
	printf("%s\n", "__BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES");
#endif // __BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES
#ifdef __BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES
	printf("%s\n", "__BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES");
#endif // __BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES

#if 0 // this is not so interesting at this point (tuning the solvers, the matrix is stable)
#ifdef __UBER_BLOCK_MATRIX_SUPRESS_FBS
	printf("%s\n", "__UBER_BLOCK_MATRIX_SUPRESS_FBS");
#endif // __UBER_BLOCK_MATRIX_SUPRESS_FBS
#ifdef __UBER_BLOCK_MATRIX_LAZY_PRODUCT
	printf("%s\n", "__UBER_BLOCK_MATRIX_LAZY_PRODUCT");
#endif // __UBER_BLOCK_MATRIX_LAZY_PRODUCT
#ifdef __UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT
	printf("%s\n", "__UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT");
#endif // __UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT
#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
	printf("%s\n", "__UBER_BLOCK_MATRIX_PERFCOUNTERS");
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_MEMORY_DEBUGGING
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_MEMORY_DEBUGGING");
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_MEMORY_DEBUGGING
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR");
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
#ifdef __UBER_BLOCK_MATRIX_HYBRID_AT_A
	printf("%s\n", "__UBER_BLOCK_MATRIX_HYBRID_AT_A");
#endif // __UBER_BLOCK_MATRIX_HYBRID_AT_A
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS");
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS
	printf("CUberBlockMatrix::map_Alignment = %d (%d, %d)\n", CUberBlockMatrix::map_Alignment,
		CUberBlockMatrix::pool_MemoryAlignment, CUberBlockMatrix::_TyDenseAllocator::n_memory_align);
#endif // 0
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
	printf("%s\n", "__UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON"); // this one is new (Q2 2017)
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
#ifdef __UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM
	printf("%s\n", "__UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM"); // this one is new (Q2 2017)
#endif // __UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM
#ifdef __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
	printf("%s\n", "__UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY");
#endif // __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
	printf("%s\n", "__SLAM_COUNT_ITERATIONS_AS_VERTICES");
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
	printf("%s\n", "__AUTO_UNARY_FACTOR_ON_VERTEX_ZERO");
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
#ifdef __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
	printf("%s\n", "__MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT");
#endif // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
#ifdef __MATRIX_ORDERING_CACHE_ALIGN
	printf("%s\n", "__MATRIX_ORDERING_CACHE_ALIGN");
#endif // __MATRIX_ORDERING_CACHE_ALIGN
#ifdef __MATRIX_ORDERING_USE_AMD1
	printf("%s\n", "__MATRIX_ORDERING_USE_AMD1");
#endif // __MATRIX_ORDERING_USE_AMD1
#ifdef __MATRIX_ORDERING_USE_AMD_AAT
	printf("%s\n", "__MATRIX_ORDERING_USE_AMD_AAT");
#endif // __MATRIX_ORDERING_USE_AMD_AAT
#ifdef __MATRIX_ORDERING_USE_MMD
	printf("%s\n", "__MATRIX_ORDERING_USE_MMD");
#endif // __MATRIX_ORDERING_USE_MMD

#ifdef __SEGREGATED_MAKE_CHECKED_ITERATORS
	printf("%s\n", "__SEGREGATED_MAKE_CHECKED_ITERATORS");
#endif // __SEGREGATED_MAKE_CHECKED_ITERATORS
#ifdef __SEGREGATED_COMPILE_FAP_TESTS
	printf("%s\n", "__SEGREGATED_COMPILE_FAP_TESTS");
#endif // __SEGREGATED_COMPILE_FAP_TESTS

#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
	printf("%s\n", "__FLAT_SYSTEM_USE_THUNK_TABLE");
#ifdef __FLAT_SYSTEM_STATIC_THUNK_TABLE
	printf("%s\n", "__FLAT_SYSTEM_STATIC_THUNK_TABLE");
#endif // __FLAT_SYSTEM_STATIC_THUNK_TABLE
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef __FLAT_SYSTEM_ALIGNED_MEMORY
	printf("%s\n", "__FLAT_SYSTEM_ALIGNED_MEMORY");
#endif // __FLAT_SYSTEM_ALIGNED_MEMORY

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
	printf("%s\n", "__SE_TYPES_SUPPORT_A_SOLVERS");
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
	printf("%s\n", "__SE_TYPES_SUPPORT_LAMBDA_SOLVERS");
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
	printf("%s\n", "__SE_TYPES_SUPPORT_L_SOLVERS");
#endif // __SE_TYPES_SUPPORT_L_SOLVERS

#ifdef __BASE_TYPES_ALLOW_CONST_VERTICES
	printf("%s\n", "__BASE_TYPES_ALLOW_CONST_VERTICES");
#endif // __BASE_TYPES_ALLOW_CONST_VERTICES
#ifdef __SLAM_APP_USE_CONSTANT_VERTICES
	printf("%s\n", "__SLAM_APP_USE_CONSTANT_VERTICES");
#endif // __SLAM_APP_USE_CONSTANT_VERTICES
#ifdef __GRAPH_TYPES_ALIGN_OPERATOR_NEW
#if defined(_MSC_VER) && !defined(__MWERKS__)
#define MAKESTRING2(x) #x // msvc fails with a missing argument error when using double stringification
#else // _MSC_VER && !__MWERKS__
#define MAKESTRING(x) #x
#define MAKESTRING2(x) MAKESTRING(x)
#endif // _MSC_VER && !__MWERKS__
	if(*MAKESTRING2(__GRAPH_TYPES_ALIGN_OPERATOR_NEW)) // if not an empty macro
		printf("%s\n", "__GRAPH_TYPES_ALIGN_OPERATOR_NEW");
#endif // __GRAPH_TYPES_ALIGN_OPERATOR_NEW
#ifdef __BASE_TYPES_USE_ALIGNED_MATRICES
	printf("%s\n", "__BASE_TYPES_USE_ALIGNED_MATRICES");
#endif // __BASE_TYPES_USE_ALIGNED_MATRICES

#ifdef __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
	printf("%s\n", "__NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2");
#endif // __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
	printf("%s\n", "__LAMBDA_USE_V2_REDUCTION_PLAN");
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN

#ifdef __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
	printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_TIMESTEPS");
#endif // __NONLINEAR_SOLVER_L_DUMP_TIMESTEPS
#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2
	printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_CHI2");
#ifdef __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
	printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE");
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2_AT_LAST_EDGE
#endif // __NONLINEAR_SOLVER_L_DUMP_CHI2
#ifdef __NONLINEAR_SOLVER_L_DUMP_DENSITY
	printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_DENSITY");
#endif // __NONLINEAR_SOLVER_L_DUMP_DENSITY
#ifdef __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
	printf("%s\n", "__NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES");
#endif // __NONLINEAR_SOLVER_L_DUMP_L_UPDATE_VARIANT_TIMES
#ifdef __NONLINEAR_SOLVER_L_DETAILED_TIMING
	printf("%s\n", "__NONLINEAR_SOLVER_L_DETAILED_TIMING");
#endif // __NONLINEAR_SOLVER_L_DETAILED_TIMING
#ifdef __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
	printf("%s\n", "__NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST");
#endif // __NONLINEAR_SOLVER_L_USE_SPARSE_BACKSUBST
#ifdef __NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST
	printf("%s\n", "__NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST");
#endif // __NONLINEAR_SOLVER_L_USE_RESUMED_BACKSUBST
#ifdef __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY
	printf("%s\n", "__NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY");
#endif // __NONLINEAR_SOLVER_L_ENABLE_DENSE_CHOLESKY

#ifdef __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1");
#endif // __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
#ifdef __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE");
#endif // __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
#ifdef __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING");
#endif // __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_CHI2");
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY
#ifdef __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES");
#endif // __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
#ifdef __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING");
#endif // __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
#ifdef __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
	printf("%s\n", "__NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY");
#endif // __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
	printf("\n");
}

void PrintHelp()
{
	printf("General use:\n"
		"    ./SLAM_plus_plus -i <filename> --no-detailed-timing\n"
		"\n"
		"To run the pose-only datasets more quickly:\n"
		"    ./SLAM_plus_plus -i <filename> --pose-only --no-detailed-timing\n"
		"\n"
		"To run incrementally:\n"
		"    ./SLAM_plus_plus -nsp <optimize-each-N-verts> -fL -i <filename> --no-detailed-timing\n"
		"\n"
		"This generates initial.txt and initial.tga, a description and image of the\n"
		"system before the final optimization, and solution.txt and solution.tga, a\n"
		"description and image of the final optimized system (unless --no-solution\n"
		"or --no-bitmaps are specified, respectively).\n"
		"\n"
		"--help|-h         displays this help screen\n"
		"--verbose|-v      displays verbose output while running (may slow down,\n"
		"                  especially in windows and if running incrementally)\n"
		"--silent|-s       suppresses displaying verbose output\n"
		"--no-show|-ns     does not show output image (windows only)\n"
		"--no-commandline|-nc    does not echo command line\n"
		"--no-flags|-nf    does not show compiler flags\n"
		"--no-detailed-timing|-ndt    does not show detailed timing breakup (use this, lest\n"
		"                  you will get confused)\n"
		"--no-bitmaps|-nb  does not write bitmaps initial.tga and solution.tga (does not\n"
		"                  affect the text files)\n"
		"--no-solution|-ns does not write text files initial.txt and solution.txt\n"
		"--xz-plots|-xz    turns bitmaps initial.tga and solution.tga into the X-Z plane\n"
		"--dump-system-matrix|-dsm    writes system matrix as system.mtx (matrix market)\n"
		"                  and system.bla (block layout) before optimization\n"
		"--pose-only|-po   enables optimisation for pose-only slam (will warn and ignore\n"
		"                  on datasets with landmarks (only the first 1000 lines checked,\n"
		"                  in case there are landmarks later, it would segfault))\n"
		"--use-old-code|-uogc    uses the old CSparse code (no block matrices in it)\n"
		"--a-solver|-A     uses A solver\n"
		"--lambda|-,\\      uses lambda solver (default, preferred batch solver)\n"
 		"--lambda-lm|-,\\lm uses lambda solver with Levenberg Marquardt (default for BA)\n"
 		"--lambda-dl|-,\\dl uses lambda solver with Dogleg and fluid relinearization\n"
		"--l-solver|-L     uses L solver\n"
		"--fast-l|-fL      uses the new fast L solver (preferred incremental solver)\n"
		"--use-schur|-us   uses Schur complement to accelerate linear solving\n"
		"--do-marginals|-dm enables marginal covariance calculation (experimental)\n"
		"--infile|-i <filename>    specifies input file <filename>; it can cope with\n"
		"                  many file types and conventions\n"
		"--parse-lines-limit|-pll <N>    sets limit of lines read from the input file\n"
		"                  (handy for testing), note this does not set limit of vertices\n"
		"                  nor edges!\n"
		"--linear-solve-period|-lsp <N>    sets period for incrementally running linear\n"
		"                  solver (default 0: disabled)\n"
		"--nonlinear-solve-period|-nsp <N>    sets period for incrementally running\n"
		"                  non-linear solver (default 0: disabled)\n"
		"--max-nonlinear-solve-iters|-mnsi <N>    sets maximal number of nonlinear\n"
		"                  solver iterations (default 10)\n"
		"--nonlinear-solve-error-thresh|-nset <f>    sets nonlinear solve error threshold\n"
		"                  (default 20)\n"
		"--max-final-nonlinear-solve-iters|-mfnsi <N>    sets maximal number of final\n"
		"                  optimization iterations (default 5)\n"
		"--final-nonlinear-solve-error-thresh|-fnset <f>    sets final nonlinear solve\n"
		"                  error threshold (default 0.01)\n"
		"--run-matrix-benchmarks|-rmb <benchmark-name> <benchmark-type>    runs block\n"
		"                  matrix benchmarks (benchmark-name is name of a folder with\n"
		"                  UFLSMC benchmark, benchmark-type is one of alloc, factor, all)\n"
		"--run-matrix-unit-tests|-rmut    runs block matrix unit tests\n"
		"--omp-set-num-threads <N>    sets number of threads to N (default is to use as\n"
		"                  many threads as there are CPU cores)\n"
		"--omp-set-dynamic <N>    enables dynamic adjustment of the number of threads is N\n"
		"                  is nonzero, disables if zero (disabled by default)\n"
		"--dogleg-step-size|-dlss <f>    sets the initial dogleg solver step size (default 2)\n"
		"--dogleg-all-batch|-dlabat    instructs the dogleg solver (disabled by default)\n"
		//"--dogleg-step-threshold|-dlst <f>    sets the dogleg solver step threshold\n" // unsupported at the moment
		"--iBA-save-intermediates|-iBAsi    enables saving of intermediate solutions at\n"
		"                  every step in incremental BA (disabled by default)\n"
		"--iBA-save-matrices|-iBAsm    enables saving of system matrices at every step in\n"
		"                  incremental BA (disabled by default)\n");
}

void TCommandLineArgs::Defaults()
{
	n_solver_choice = nlsolver_Lambda; /**< @brief nonlinear solver selector */
	// solver selection

	b_write_bitmaps = true;
	b_write_solution = true;
	b_xz_plots = false;
	b_write_system_matrix = false;
	b_no_show = false;
	b_show_commandline = true;
	b_show_flags = true;
	b_show_detailed_timing = true;
	b_verbose = true;
	// verbosity

	b_use_schur = false;

	b_run_matrix_benchmarks = false;
	b_run_matrix_unit_tests = false;
	b_use_old_system = false; // t_odo - make this commandline
	b_pose_only = false;
	b_use_SE3 = false; // note this is not overriden in commandline but detected in peek-parsing
	b_use_BA = false; // note this is not overriden in commandline but detected in peek-parsing
	b_use_BAS = false; // note this is not overriden in commandline but detected in peek-parsing
	b_use_BAI = false; // note this is not overriden in commandline but detected in peek-parsing
	b_use_spheron = false;
	b_use_rocv = false;

	p_s_input_file = 0; /** <@brief path to the data file */
	n_max_lines_to_process = 0; /** <@brief maximal number of lines to process */

	n_linear_solve_each_n_steps = 0; /**< @brief linear solve period, in steps (0 means disabled) */
	n_nonlinear_solve_each_n_steps = 0; /**< @brief nonlinear solve period, in steps (0 means disabled) */
	n_max_nonlinear_solve_iteration_num = 10; /**< @brief maximal number of iterations in nonlinear solve step */
	f_nonlinear_solve_error_threshold = 20; /**< @brief error threshold for nonlinear solve */
	n_max_final_optimization_iteration_num = 5; // as many other solvers
	f_final_optimization_threshold = 0.01;
	// optimization mode for slam

	p_s_bench_name = 0;
	p_s_bench_type = "all";

	n_omp_threads = size_t(-1);
	b_omp_dynamic = false;

	b_do_marginals = false;

	f_dogleg_step_size = -1;
	f_dogleg_step_threshold = -1;
	// "not set"

	b_dogleg_all_batch = false;

	b_inc_BA_save_intermediates = false;
	b_inc_BA_save_sysmats = false;
}

bool TCommandLineArgs::Parse(int n_arg_num, const char **p_arg_list)
{
	for(int i = 1; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			PrintHelp();
			//fprintf(stderr, "no help for you! mwuhahaha! (please read Main.cpp)\n"); // t_odo
			return false; // quit
		} else if(!strcmp(p_arg_list[i], "--verbose") || !strcmp(p_arg_list[i], "-v"))
			b_verbose = true;
		else if(!strcmp(p_arg_list[i], "--silent") || !strcmp(p_arg_list[i], "-s"))
			b_verbose = false;
		else if(!strcmp(p_arg_list[i], "--use-schur") || !strcmp(p_arg_list[i], "-us"))
			b_use_schur = true;
		else if(!strcmp(p_arg_list[i], "--no-show") || !strcmp(p_arg_list[i], "-ns"))
			b_no_show = true;
		else if(!strcmp(p_arg_list[i], "--no-commandline") || !strcmp(p_arg_list[i], "-nc"))
			b_show_commandline = false;
		else if(!strcmp(p_arg_list[i], "--do-marginals") || !strcmp(p_arg_list[i], "-dm"))
			b_do_marginals = true;
		else if(!strcmp(p_arg_list[i], "--lambda") || !strcmp(p_arg_list[i], "-,\\"))
			n_solver_choice = nlsolver_Lambda;
		else if(!strcmp(p_arg_list[i], "--lambda-lm") || !strcmp(p_arg_list[i], "-,\\lm"))
			n_solver_choice = nlsolver_LambdaLM;
		else if(!strcmp(p_arg_list[i], "--lambda-dl") || !strcmp(p_arg_list[i], "-,\\dl"))
			n_solver_choice = nlsolver_LambdaDL;
		else if(!strcmp(p_arg_list[i], "--no-flags") || !strcmp(p_arg_list[i], "-nf"))
			b_show_flags = false;
		else if(!strcmp(p_arg_list[i], "--run-matrix-unit-tests") || !strcmp(p_arg_list[i], "-rmut"))
			b_run_matrix_unit_tests = true;
		else if(!strcmp(p_arg_list[i], "--no-detailed-timing") || !strcmp(p_arg_list[i], "-ndt"))
			b_show_detailed_timing = false;
		else if(!strcmp(p_arg_list[i], "--use-old-code") || !strcmp(p_arg_list[i], "-uogc"))
			b_use_old_system = true;
		else if(!strcmp(p_arg_list[i], "--dump-system-matrix") || !strcmp(p_arg_list[i], "-dsm"))
			b_write_system_matrix = true;
		else if(!strcmp(p_arg_list[i], "--no-bitmaps") || !strcmp(p_arg_list[i], "-nb")) {
			b_write_bitmaps = false;
			b_no_show = true; // no bitmaps ... what can it show?
		} else if(!strcmp(p_arg_list[i], "--no-solution") || !strcmp(p_arg_list[i], "-ns"))
			b_write_solution = false;
		else if(!strcmp(p_arg_list[i], "--xz-plots") || !strcmp(p_arg_list[i], "-xz"))
			b_xz_plots = true;
		else if(!strcmp(p_arg_list[i], "--pose-only") || !strcmp(p_arg_list[i], "-po"))
			b_pose_only = true;
		else if(!strcmp(p_arg_list[i], "--a-solver") || !strcmp(p_arg_list[i], "-A"))
			n_solver_choice = nlsolver_A;
		else if(!strcmp(p_arg_list[i], "--l-solver") || !strcmp(p_arg_list[i], "-L"))
			n_solver_choice = nlsolver_L;
		else if(!strcmp(p_arg_list[i], "--fast-l") || !strcmp(p_arg_list[i], "-fL"))
			n_solver_choice = nlsolver_FastL;
		else if(!strcmp(p_arg_list[i], "--iBA-save-intermediates") || !strcmp(p_arg_list[i], "-iBAsi"))
			b_inc_BA_save_intermediates = true;
		else if(!strcmp(p_arg_list[i], "--iBA-save-matrices") || !strcmp(p_arg_list[i], "-iBAsm"))
			b_inc_BA_save_sysmats = true;
		else if(!strcmp(p_arg_list[i], "--dogleg-all-batch") || !strcmp(p_arg_list[i], "-dlabat"))
			b_dogleg_all_batch = true;
		else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return false;
		} else if(!strcmp(p_arg_list[i], "--infile") || !strcmp(p_arg_list[i], "-i"))
			p_s_input_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--parse-lines-limit") || !strcmp(p_arg_list[i], "-pll"))
			n_max_lines_to_process = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--linear-solve-period") || !strcmp(p_arg_list[i], "-lsp"))
			n_linear_solve_each_n_steps = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--nonlinear-solve-period") || !strcmp(p_arg_list[i], "-nsp"))
			n_nonlinear_solve_each_n_steps = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--max-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mnsi"))
			n_max_nonlinear_solve_iteration_num = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-nset"))
			f_nonlinear_solve_error_threshold = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--max-final-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mfnsi"))
			n_max_final_optimization_iteration_num = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--final-nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-fnset"))
			f_final_optimization_threshold = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--dogleg-step-size") || !strcmp(p_arg_list[i], "-dlss"))
			f_dogleg_step_size = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--dogleg-step-threshold") || !strcmp(p_arg_list[i], "-dlst"))
			f_dogleg_step_threshold = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--omp-set-num-threads"))
			n_omp_threads = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--omp-set-dynamic"))
			b_omp_dynamic = (atol(p_arg_list[++ i]) != 0);
		else if(!strcmp(p_arg_list[i], "--run-matrix-benchmarks") || !strcmp(p_arg_list[i], "-rmb")) {
			if(i + 2 >= n_arg_num) {
				fprintf(stderr, "error: argument \'%s\': missing the second value\n", p_arg_list[i]);
				return false;
			}
			b_run_matrix_benchmarks = true;
			p_s_bench_name = p_arg_list[++ i];
			p_s_bench_type = p_arg_list[++ i];
			if(strcmp(p_s_bench_type, "alloc") &&
			   strcmp(p_s_bench_type, "factor") &&
			   strcmp(p_s_bench_type, "all")) {
				fprintf(stderr, "error: argument \'%s\': unknown benchmark type\n", p_arg_list[i]);
				return false;
			}
		} else if(!strcmp(p_arg_list[i], "--dummy-param") || !strcmp(p_arg_list[i], "-dp"))
			n_dummy_param = atol(p_arg_list[++ i]);
		else {
			fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
			return false;
		}
	}
	// "parse" cmdline

	return true;
}

#ifdef __COMPILE_LIBRARY_TESTS

#include <iostream>

/**
 *	@brief makes sure that CSparse is linked correctly (and is working)
 */
static void CSparse_Test()
{
	cs *test_my_matrix = cs_spalloc(4, 4, 16, 1, 1);
	for(int i = 0; i < 4; ++ i)
		cs_entry(test_my_matrix, i, i, 1);
	cs *test_my_matrix2 = cs_compress(test_my_matrix);
	cs_print(test_my_matrix, 0);
	cs_print(test_my_matrix2, 0);
	cs_spfree(test_my_matrix);
	cs_spfree(test_my_matrix2);
	// just make sure that CSparse is working
}

/**
 *	@brief makes sure that eigen is linked correctly (and is working)
 */
static void Eigen3_Test()
{
	Eigen::Matrix<double, 3, 3> C;
	const double alpha = 30 / 180.0 * M_PI;

	double cosa = cos(alpha),
		   sina = sin(alpha);
	C << cosa, -sina,   0.0,
		 sina,  cosa,   0.0,
		 0.0,   0.0,   1.0;
	// Create the principal rotation matrix C_3(alpha)

	Eigen::Matrix<double, 3, 1> a(1, 2, 3);
	Eigen::Matrix<double, 3, 1> b = C * a;

	std::cout << "C = " << C << std::endl;
	std::cout << "a = " << a << std::endl;
	std::cout << "C * a = " << b << std::endl;
}

#if 0

/**
 *	@brief tests reference matrix concept (not a library test, actually)
 */
static void Test_RefMatrix() // t_odo - see assembly to see what's going on // don't have ReferencingMatrixXd anymore, using Eigen::Map<> now.
{
	double p_data[16] = {
		0, 1, 2, 3,
		4, 5, 6, 7,
		8, 9, 10, 11,
		12, 13, 14, 15
	};

	Eigen::ReferencingMatrixXd matrix(4, 4, p_data); // no copy of the data is made (at least i hope :))

	Eigen::Matrix4d b, c; // ordinary matrices

	b << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	c = matrix * b; // see what happens

	b = matrix * matrix;
	// seems legit - result of multiplying two reference matrices is matrixXd // t_odo - check it

	printf("b = {\n");
	for(int i = 0; i < 4; ++ i) {
		printf("    ");
		for(int j = 0; j < 4; ++ j)
			printf("%3g ", b(i, j));
		printf("\n");
	}
	printf("}\n");
	printf("c = {\n");
	for(int i = 0; i < 4; ++ i) {
		printf("    ");
		for(int j = 0; j < 4; ++ j)
			printf("%2g ", c(i, j));
		printf("\n");
	}
	printf("}\n");
}

#endif // 0

/**
 *	@brief tests forward allocated pool (not a library test, actually)
 */
static void Test_FAP()
{
	{
		forward_allocated_pool<int, 0> fap100(100);
		forward_allocated_pool<int, 0> fap200(200);
		fap100.swap(fap200);
	}
	{
		forward_allocated_pool<int, 0> fap100(100);
		forward_allocated_pool<int, 0> fap200(100);
		fap100.swap(fap200);
	}
	{
		forward_allocated_pool<int, 100> fap100;
		forward_allocated_pool<int, 0> fap200(100);
		fap100.swap(fap200);
	}
	{
		forward_allocated_pool<int, 100> fap100;
		forward_allocated_pool<int, 200> fap200;
		fap100.swap(fap200);
	}
	{
		forward_allocated_pool<int, 100> fap100;
		forward_allocated_pool<int, 100> fap200;
		fap100.swap(fap200);
	}

	{
		forward_allocated_pool<int, 100> fap100;
		_ASSERTE(fap100.page_size() == 100);
		forward_allocated_pool<int, 0> fap(100);
		_ASSERTE(fap.page_size() == 100);
		forward_allocated_pool<int> fap2(100);
		_ASSERTE(fap2.page_size() == 100);
	}

	if(!CFAP_Test<100, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<100, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");

	if(!CFAP_Test<10, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<10, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");

	if(!CFAP_Test<13, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<13, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");

	if(!CFAP_Test<128, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<128, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");

	if(!CFAP_Test<1, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<1, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");

	if(!CFAP_Test<2, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<2, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");

	if(!CFAP_Test<4, false>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
	if(!CFAP_Test<4, true>::FAP_Test(1000))
		fprintf(stderr, "error: tests failed\n");
}

/**
 *	@brief tests block matrix allocator using tetris (not a library test, actually)
 */
static void Test_Tetris(int n_arg_num, const char **p_arg_list)
{
	tetris_main(n_arg_num, p_arg_list);
}

#endif // __COMPILE_LIBRARY_TESTS

/*
 *	end-of-file
 */
