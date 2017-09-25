/*
								+----------------------------------+
								|                                  |
								|  ***  BA Marginals Example  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|             Main.cpp             |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/ba_marginals_example/Main.cpp
 *	@brief contains the main() function of the BA marginals example program
 *	@author -tHE SWINe-
 *	@date 2016-02-15
 */

#include <stdio.h> // printf
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#include "slam/LinearSolver_UberBlock.h"
#include "slam/OrderingMagic.h"
#include "slam/BAMarginals.h"

#include "sparse_flops/cts.hpp"
#include "sparse_flops/Instrument.h"

int n_dummy_param = 0; // required by the DL solver, otherwise causes a link error

int LU_Test(CUberBlockMatrix &lambda, bool b_clflush, bool b_no_FBS, bool b_count_FLOPs,
	bool b_intrablock_full_piv = true, bool b_interblock_part_piv = true,
	double f_min_piv_gain = 0, int n_amd_type = 2, bool b_symperm = true)
{
	printf("b_clflush = %s\n", (b_clflush)? "true" : "false");
	printf("b_no_FBS = %s\n", (b_no_FBS)? "true" : "false");
	printf("b_count_FLOPs = %s\n", (b_count_FLOPs)? "true" : "false");

	//printf("n_block_size = %d\n", n_block_size);
	//printf("b_block_bench_style_a = %s\n", (b_block_bench_style_a)?
	//	"true (elems inflated to blocks)" : "false (matrix partitioned to blocks)");
	printf("b_interblock_part_piv = %s\n", (b_interblock_part_piv)? "true" : "false");
	printf("f_min_piv_gain = %g%%\n", f_min_piv_gain * 100);
	//printf("n_piv_L_norm_type = %s\n", (n_norm_type == 1)? "1" : (n_norm_type == 2)? "2" :
	//	(n_norm_type == Eigen::Infinity)? "Eigen::Infinity" : "[other]");
	printf("b_intrablock_full_piv = %s\n", (b_intrablock_full_piv)? "true" : "false");
	//printf("b_Eigen_full_piv = %s\n", (b_Eigen_full_piv)? "true" : "false");
	printf("n_amd_type = %s\n", (n_amd_type == 0)? "0 (natural)" :
		(n_amd_type == 1)? "1 (Chol)" : (n_amd_type == 2)? "2 (LU)" : (n_amd_type == 3)? "3 (QR)" : "[other]");
	printf("b_symperm = %s\n\n", (b_symperm)? "true" : "false");

	printf("ordering ...\n"); fflush(stdout);

	cs *p_pat;
	if(!(p_pat = lambda.p_BlockStructure_to_Sparse()))
		throw std::bad_alloc();
	csi *p_block_perm;
	if(!(p_block_perm = cs_amd(n_amd_type, p_pat))) { // a different AMD than for Cholesky, apparently
		fprintf(stderr, "error: cs_amd() failed\n");
		return -1;
	}
	cs_spfree(p_pat);
	// get symbolic block ordering

	printf("permuting ...\n"); fflush(stdout);

	_ASSERTE(sizeof(size_t) == sizeof(csi));
	CMatrixOrdering mord;
	const size_t *p_inv_block_perm = mord.p_InvertOrdering((const size_t*)p_block_perm, lambda.n_BlockColumn_Num());
	CUberBlockMatrix lambda_amd;
	lambda.PermuteTo(lambda_amd, p_inv_block_perm,
		lambda.n_BlockColumn_Num(), b_symperm, true, false); // want to benchmark the decomposition, make a deep copy
	cs_free(p_block_perm);
	// repermute lambda

	//lambda_amd = lambda; // use no ordering

	lambda.Clear();
	// save memory, could run out on large matrices

	cs *p_A;
	if(!(p_A = lambda_amd.p_Convert_to_Sparse()))
		throw std::bad_alloc();

	const double f_norm = cs_norm(p_A);
	const double f_rel_err_scale = 1 / std::max(1.0, f_norm);

	if(b_count_FLOPs) {
		typedef CTSparse<CFLOPCountingDouble> CFLOPCountingSparse;

		size_t n_before_LU = CFLOPCountingDouble::n_FLOP_Num();

		CFLOPCountingSparse::css *p_S = CFLOPCountingSparse::sqr(0,
			CFLOPCountingSparse::p_FromSparse(p_A), 0); // symbolic analysis, use natural ordering

		size_t n_before_num = CFLOPCountingDouble::n_FLOP_Num();

		CFLOPCountingSparse::csn *p_N = CFLOPCountingSparse::lu(CFLOPCountingSparse::p_FromSparse(p_A),
			p_S, 1 + f_min_piv_gain); // numeric LU factorization

		size_t n_flops = CFLOPCountingDouble::n_FLOP_Num() - n_before_LU;
		size_t n_sym_flops = n_before_num - n_before_LU;

		if(p_S && p_N) {
			printf("number of FLOPs in sparse LU: " PRIsize " (" PRIsize
				" in symbolic decomposition)\n", n_flops, n_sym_flops);
		} else {
			printf("number of FLOPs in sparse LU: fail (fail"
				" in symbolic decomposition)\n");
		}
		fflush(stdout);

		CFLOPCountingSparse::sfree(p_S);
		CFLOPCountingSparse::nfree(p_N);
	}

	fbs_ut::CDummyAlgSpecializer::TBSMix t_block_sizes =
		fbs_ut::CDummyAlgSpecializer::t_BlockSize_Mixture(lambda_amd);
	std::sort(t_block_sizes.begin(), t_block_sizes.end());
	const bool b_use_FBS23 = !b_no_FBS && t_block_sizes.size() == 4 &&
		t_block_sizes[0] == std::make_pair(size_t(2), size_t(2)) &&
		t_block_sizes[1] == std::make_pair(size_t(2), size_t(3)) &&
		t_block_sizes[2] == std::make_pair(size_t(3), size_t(2)) &&
		t_block_sizes[3] == std::make_pair(size_t(3), size_t(3));
	const bool b_use_FBS3 = !b_no_FBS && t_block_sizes.size() == 1 &&
		t_block_sizes.front() == std::make_pair(size_t(3), size_t(3));
	const bool b_use_FBS4 = !b_no_FBS && t_block_sizes.size() == 1 &&
		t_block_sizes.front() == std::make_pair(size_t(4), size_t(4));
	const bool b_use_FBS5 = !b_no_FBS && t_block_sizes.size() == 1 &&
		t_block_sizes.front() == std::make_pair(size_t(5), size_t(5));
	const bool b_use_FBS6 = !b_no_FBS && t_block_sizes.size() == 1 &&
		t_block_sizes.front() == std::make_pair(size_t(6), size_t(6));
	const bool b_use_FBS36 = !b_no_FBS && t_block_sizes.size() == 4 &&
		t_block_sizes[0] == std::make_pair(size_t(3), size_t(3)) &&
		t_block_sizes[1] == std::make_pair(size_t(3), size_t(6)) &&
		t_block_sizes[2] == std::make_pair(size_t(6), size_t(3)) &&
		t_block_sizes[3] == std::make_pair(size_t(6), size_t(6));
	const bool b_use_FBS37 = !b_no_FBS && t_block_sizes.size() == 4 &&
		t_block_sizes[0] == std::make_pair(size_t(3), size_t(3)) &&
		t_block_sizes[1] == std::make_pair(size_t(3), size_t(7)) &&
		t_block_sizes[2] == std::make_pair(size_t(7), size_t(3)) &&
		t_block_sizes[3] == std::make_pair(size_t(7), size_t(7));

	printf((b_use_FBS3)? "decomposing (3x3 FBS) ...\n" :
		(b_use_FBS4)? "decomposing (4x4 FBS) ...\n" :
		(b_use_FBS5)? "decomposing (5x5 FBS) ...\n" :
		(b_use_FBS6)? "decomposing (6x6 FBS) ...\n" :
		(b_use_FBS23)? "decomposing (3x3 3x2 2x3 2x2 FBS) ...\n" :
		(b_use_FBS36)? "decomposing (6x6 6x3 3x6 3x3 FBS) ...\n" :
		(b_use_FBS37)? "decomposing (7x7 7x3 3x7 3x3 FBS) ...\n" :
		"decomposing ...\n"); fflush(stdout);

	CUberBlockMatrix L, U;
	std::vector<size_t> row_perm(lambda_amd.n_Row_Num()), col_perm(lambda_amd.n_Column_Num());

	if(b_clflush) {
		CDebug::Evict(row_perm);
		CDebug::Evict(col_perm);
		CDebug::Evict(L);
		CDebug::Evict(U);
		CDebug::Evict(lambda_amd);
		CDebug::Evict(p_A);
		CDebug::Swamp_Cache(); // just to make sure
	}

	CDeltaTimer dt;
	dt.Reset();

	bool b_did_piv;
	bool b_result;
	if(b_use_FBS3) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>)) TBS3;
		b_result = lambda_amd.LUTo_FBS<TBS3>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else if(b_use_FBS4) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<4, 4>)) TBS4;
		b_result = lambda_amd.LUTo_FBS<TBS4>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else if(b_use_FBS5) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<5, 5>)) TBS5;
		b_result = lambda_amd.LUTo_FBS<TBS5>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else if(b_use_FBS6) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<6, 6>)) TBS6;
		b_result = lambda_amd.LUTo_FBS<TBS6>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else if(b_use_FBS23) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>,
			fbs_ut::CCTSize2D<3, 2>, fbs_ut::CCTSize2D<2, 3>, fbs_ut::CCTSize2D<2, 2>)) TBS23;
		b_result = lambda_amd.LUTo_FBS<TBS23>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else if(b_use_FBS36) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<6, 6>,
			fbs_ut::CCTSize2D<6, 3>, fbs_ut::CCTSize2D<3, 6>, fbs_ut::CCTSize2D<3, 3>)) TBS36;
		b_result = lambda_amd.LUTo_FBS<TBS36>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else if(b_use_FBS37) {
		typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<7, 7>,
			fbs_ut::CCTSize2D<7, 3>, fbs_ut::CCTSize2D<3, 7>, fbs_ut::CCTSize2D<3, 3>)) TBS37;
		b_result = lambda_amd.LUTo_FBS<TBS37>(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
		// test simple FBS LU // todo - add a proper specializer for multiple sizes
	} else {
		b_result = lambda_amd.LUTo(L, U, &row_perm.front(), &col_perm.front(), &b_did_piv,
			b_interblock_part_piv, f_min_piv_gain, b_intrablock_full_piv);
	}
	// todo - time it

	double f_lub_time = dt.f_Time();

	if(b_result) {
		CUberBlockMatrix LU;
		LU.ProductOf(L, U);
		cs *p_LU = LU.p_Convert_to_Sparse();
		CMatrixOrdering pinv;
		cs *p_PAQ = cs_permute(p_A, (const csi*)/*pinv.p_InvertOrdering(*/&row_perm.front()/*, // this is apparently already the inverse?
			row_perm.size())*/, (const csi*)&col_perm.front(), 1);
		cs *p_err = cs_add(p_LU, p_PAQ, 1, -1);
		/*cs *p_PTLUQT = cs_permute(p_LU, (const csi*)&row_perm.front(),
			(const csi*)pinv.p_InvertOrdering(&col_perm.front(), col_perm.size()), 1);

		lambda_amd.Rasterize("LU_lambda_AMD.tga");
		L.Rasterize("LU_L.tga");
		U.Rasterize("LU_U.tga");
		LU.Rasterize("LU_LU.tga");
		CDebug::Dump_SparseMatrix_Subsample("LU_raw.tga", p_A);
		CDebug::Dump_SparseMatrix_Subsample("LU_PTLUQT.tga", p_PTLUQT);
		CDebug::Dump_SparseMatrix_Subsample("LU_correct.tga", p_PAQ);
		CDebug::Dump_SparseMatrix_Subsample("LU_ubm.tga", p_LU);
		CDebug::Dump_SparseMatrix_Subsample("LU_err.tga", p_err);

		cs_spfree(p_PTLUQT);*/
		cs_spfree(p_LU);
		cs_spfree(p_PAQ);
		double f_LUB_error = cs_norm(p_err);
		cs_spfree(p_err);

		printf("LUB %.3f msec, %g " PRIsize " " PRIsize " (%s piv)\n", f_lub_time * 1000,
			f_LUB_error * f_rel_err_scale,
			L.n_NonZero_Num(), U.n_NonZero_Num(), (b_did_piv)? "did" : "no"); fflush(stdout);
	} else
		printf("LUB -1 msec, fail fail fail\n"); fflush(stdout);
	// analyze block LU results

	L.Clear();
	U.Clear();
	// free some memory

	if(b_clflush) {
		CDebug::Evict(p_A);
		CDebug::Swamp_Cache(); // just to make sure
	}

	double f_stats_time = dt.f_Time();

	css *p_S = cs_sqr(0, p_A, 0); // symbolic analysis, use natural ordering
	csn *p_N = cs_lu(p_A, p_S, 1 + f_min_piv_gain); // numeric LU factorization
	// t_odo - time it

	double f_cs_time = dt.f_Time();

	if(p_S && p_N) {
		size_t n_cs_nnz_L = p_N->L->p[p_N->L->n];
		size_t n_cs_nnz_U = p_N->U->p[p_N->U->n];
		cs *p_cs_LU = cs_multiply(p_N->L, p_N->U);
		cs *p_PAQ = cs_permute(p_A, p_N->pinv, p_S->q, 1);
		cs_nfree(p_N);
		cs_sfree(p_S);
		cs *p_cs_err = cs_add(p_cs_LU, p_PAQ, 1, -1);
		cs_spfree(p_cs_LU);
		cs_spfree(p_PAQ);
		double f_cs_error = cs_norm(p_cs_err);
		cs_spfree(p_cs_err);

		printf("CSparse %.3f msec, %g " PRIsize " " PRIsize "\n", f_cs_time * 1000,
			f_cs_error * f_rel_err_scale, n_cs_nnz_L, n_cs_nnz_U); fflush(stdout);
	} else {
		printf("CSparse -1 msec, fail fail fail\n"); fflush(stdout);
		cs_sfree(p_S);
		cs_nfree(p_N);
	}
	// compute and analyze CSparse LU

	cs_spfree(p_A);

	return 0;
}

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
	std::string s_input = (n_arg_num >= 2)? p_arg_list[1] : "data/BA_sys/sososet";
	bool b_no_SchurD = false;
	bool b_no_SchurD_Chol = false;
	bool b_test_natural_Chol_only = false;
	bool b_test_LU_only = false;
	bool b_test_SC_LU_only = false;
	bool b_clflush = false;
	bool b_no_FBS = false;
	bool b_count_FLOPs = false;
	for(int i = 2; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "-nSD"))
			b_no_SchurD = true;
		else if(!strcmp(p_arg_list[i], "-tnCo"))
			b_test_natural_Chol_only = true;
		else if(!strcmp(p_arg_list[i], "-tLUo"))
			b_test_LU_only = true;
		else if(!strcmp(p_arg_list[i], "-tSCLUo"))
			b_test_SC_LU_only = true;
		else if(!strcmp(p_arg_list[i], "-nSDChol"))
			b_no_SchurD_Chol = true;
		else if(!strcmp(p_arg_list[i], "-clflush"))
			b_clflush = true;
		else if(!strcmp(p_arg_list[i], "-nFBS"))
			b_no_FBS = true;
		else if(!strcmp(p_arg_list[i], "-cFLOPs"))
			b_count_FLOPs = true;
		else {
			fprintf(stderr, "error: unknown switch: \'%s\'\n", p_arg_list[2]);
			return -1;
		}
	}
	if(b_clflush && !(b_test_LU_only || b_test_SC_LU_only))
		fprintf(stderr, "warning: -clflush specified without -tLUo / tSCLUo (ignored)\n");
	if(b_no_FBS && !(b_test_LU_only || b_test_SC_LU_only))
		fprintf(stderr, "warning: -nFBS specified without -tLUo / tSCLUo (ignored)\n");
	if(b_count_FLOPs && !(b_test_LU_only || b_test_SC_LU_only))
		fprintf(stderr, "warning: -cFLOPs specified without -tLUo / tSCLUo (ignored)\n");
	// "parse" commandline

#ifdef _OPENMP
#pragma omp parallel
#pragma omp master
	{
		printf("%s (%d threads)\n", "_OPENMP", omp_get_num_threads());
	}
#endif // _OPENMP

	printf("fear and loading \'%s\' ... ", (s_input + "/system.mtx").c_str()); fflush(stdout);
	CUberBlockMatrix lambda;
	if(!lambda.Load_MatrixMarket((s_input + "/system.mtx").c_str(),
	   (s_input + "/system.bla").c_str())) {
		printf("\n");
		fprintf(stderr, "error: failed to load the matrix\n");
		return -1;
	}
	printf("done (needs %.2f MB)\n", lambda.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);
	// load lambda (from BA)

	printf("debug: block sizes in lambda: ");
	fbs_ut::CDummyAlgSpecializer::TBSMix t_block_sizes =
		fbs_ut::CDummyAlgSpecializer::t_BlockSize_Mixture(lambda);
	fbs_ut::CDummyAlgSpecializer::Print_BlockSizes(t_block_sizes);
	printf("\n");
	// debug - see block sizes

	if(!b_test_LU_only)
		lambda.t_GetBlock_Log(0, 0).diagonal().array() += /*Eigen::MatrixXd::Identity(6, 6) **/ 10000; // just add to the diagonal, will work with any size
	// hack - raise the first block to avoid not pos def in SC
	// note that this is only needed to handle all 3rd party datasets. there is no such offset applied in SLAM++ solver
	// the alternative would be to use modified Cholesky or LDL^T instead, then positive-definiteness is not required

	{
		const char *p_s_filename_pat = "lambda_raw";
		const bool b_symmetric = !b_test_LU_only;

		char p_s_filename[256];
		sprintf(p_s_filename, "sc_0.0_%s.tga", p_s_filename_pat);
		if(!lambda.Rasterize(p_s_filename))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
		cs *p_mat = lambda.p_BlockStructure_to_Sparse();
		sprintf(p_s_filename, "sc_0.0_%s_SS.tga", p_s_filename_pat);
		if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
		   p_mat, 0, 640, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
		sprintf(p_s_filename, "sc_0.0_%s_SS_AA.tga", p_s_filename_pat);
		if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
		   p_mat, 0, 640/*8192*/, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
		cs_spfree(p_mat);
	}

	if(b_test_LU_only)
		return LU_Test(lambda, b_clflush, b_no_FBS, b_count_FLOPs);

	if(b_test_natural_Chol_only) {
		CUberBlockMatrix R;
		if(!R.CholeskyOf(lambda)) {
			fprintf(stderr, "error: got not pos def when factorizing lambda\n");
			return -1;
		}
		R.Rasterize("sc_0.8_R_raw.tga", 3);
		R.Rasterize(lambda, false, "sc_0.8_R_raw-fill-in.tga", 3);

		printf("R natural: " PRIsize " nzb\n", R.n_Block_Num());

		CUberBlockMatrix lambda_up;
		lambda_up.TriangularViewOf(lambda, true, true); // for AMD
		CMatrixOrdering lam_ord;
		lam_ord.p_BlockOrdering(lambda_up, true); // w.r.t. lambda_perm
		const size_t *p_lam_ord = lam_ord.p_Get_InverseOrdering();
		const size_t n_lam_ord_size = lam_ord.n_Ordering_Size();
		// ordering for lambda

		CUberBlockMatrix lambda_amd;
		lambda.Permute_UpperTriangular_To(lambda_amd, p_lam_ord, n_lam_ord_size, true);

		if(!R.CholeskyOf(lambda_amd)) {
			fprintf(stderr, "error: got not pos def when factorizing lambda\n");
			return -1;
		}
		R.Rasterize("sc_0.9_R_AMD.tga", 3);
		R.Rasterize(lambda_amd, false, "sc_0.9_R_AMD-fill-in.tga", 3);

		printf("R AMD: " PRIsize " nzb\n", R.n_Block_Num());

		return 0;
	}
	// get quick info on the full factorization using "-tnCo"

	CTimer t;
	CTimerSampler timer(t);

	printf("ordering ...\n"); fflush(stdout);
	std::vector<size_t> ordering;
	size_t n_matrix_cut, n_size = lambda.n_BlockColumn_Num();
	{
		std::vector<size_t> lm;
		for(size_t i = 0, n = lambda.n_BlockColumn_Num(); i < n; ++ i)
			((lambda.n_BlockColumn_Column_Num(i) != 3)? ordering : lm).push_back(i);
		n_matrix_cut = ordering.size();
		ordering.insert(ordering.end(), lm.begin(), lm.end());
	}
	// calculate guided schur order (want to compile fast so im reimplementing
	// silly stuff here rather than including files that compile slow)

	std::vector<size_t> inv_ordering(ordering.size());
	for(size_t i = 0, n = ordering.size(); i < n; ++ i)
		inv_ordering[ordering[i]] = i;
	// invert the ordering

	double f_ordering_time = 0;
	timer.Accum_DiffSample(f_ordering_time);

	printf("permuting and cutting ...\n"); fflush(stdout);
	CUberBlockMatrix lambda_perm;
	lambda.Permute_UpperTriangular_To(lambda_perm, &inv_ordering[0], inv_ordering.size(), true);
	// permute

	CUberBlockMatrix A, U, V, D;
	lambda_perm.SliceTo(A, 0, n_matrix_cut, 0, n_matrix_cut, false); // copy
	lambda_perm.SliceTo(U, 0, n_matrix_cut, n_matrix_cut, n_size, true);
	lambda_perm.SliceTo(D, n_matrix_cut, n_size, n_matrix_cut, n_size, true);
	V.TransposeOf(U);
	// get the submatrices

	double f_schur_perm_time = 0;
	timer.Accum_DiffSample(f_schur_perm_time);

	std::sort(t_block_sizes.begin(), t_block_sizes.end());
	const bool b_use_FBS36 = (t_block_sizes.size() == 4 &&
		t_block_sizes[0] == std::make_pair(size_t(3), size_t(3)) &&
		t_block_sizes[1] == std::make_pair(size_t(3), size_t(6)) &&
		t_block_sizes[2] == std::make_pair(size_t(6), size_t(3)) &&
		t_block_sizes[3] == std::make_pair(size_t(6), size_t(6))) ||
		(t_block_sizes.size() == 1 && t_block_sizes[0] == std::make_pair(size_t(6), size_t(6))) ||
		(t_block_sizes.size() == 1 && t_block_sizes[0] == std::make_pair(size_t(3), size_t(3)));
	const bool b_use_FBS37 = (t_block_sizes.size() == 4 &&
		t_block_sizes[0] == std::make_pair(size_t(3), size_t(3)) &&
		t_block_sizes[1] == std::make_pair(size_t(3), size_t(7)) &&
		t_block_sizes[2] == std::make_pair(size_t(7), size_t(3)) &&
		t_block_sizes[3] == std::make_pair(size_t(7), size_t(7))) ||
		(t_block_sizes.size() == 1 && t_block_sizes[0] == std::make_pair(size_t(7), size_t(7)));
	if(!b_use_FBS36 && !b_use_FBS37) {
		fprintf(stderr, "error: could not choose an appropriate block mixture, unable to continue\n");
		return -1;
	}
	// decide which specialization to use (this would have been better handled by wrapping
	// the rest of main() in fbs_ut::CDummyAlgSpecializer::CData::Run(). this was originally only
	// written for 6D poses and 3D landmarks so it was not an issue before. 3DV 2017 added Sim(3)
	// poses and support was needed here as well. too lazy to rewrite the whole thing properlys)

	typedef MakeTypelist_Safe((Eigen::Matrix<double, 6, 6>)) SC_BlockSizes36;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 6, 6>)) A_BlockSizes36;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 7, 7>)) SC_BlockSizes37;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 7, 7>)) A_BlockSizes37;
	//typedef MakeTypelist_Safe((Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 7, 7>, Eigen::Matrix<double, 6, 7>, Eigen::Matrix<double, 7, 6>)) SC_BlockSizes; // Cholesky needs all size combinations, new ones might come into existence via fill-in
	//typedef MakeTypelist_Safe((Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 7, 7>/*, Eigen::Matrix<double, 6, 7>, Eigen::Matrix<double, 7, 6>*/)) A_BlockSizes; // should be fine without
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 3, 3>)) D_BlockSizes;
	//typedef MakeTypelist_Safe((Eigen::Matrix<double, 6, 3>, Eigen::Matrix<double, 7, 3>)) U_BlockSizes;
	//typedef MakeTypelist_Safe((Eigen::Matrix<double, 3, 6>, Eigen::Matrix<double, 3, 7>)) V_BlockSizes;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 6, 3>)) U_BlockSizes36;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 3, 6>)) V_BlockSizes36;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 7, 3>)) U_BlockSizes37;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 3, 7>)) V_BlockSizes37;
	typedef MakeTypelist_Safe((Eigen::Matrix<double, 3, 3>)) SC2_BlockSizes;
	// block sizes for simple BA problems (landmarks are 3x3, poses are 6x6 or 7x7)

	printf("\ncalculating SC ...\n"); fflush(stdout);
	CUberBlockMatrix Dinv;
	Dinv.InverseOf_Symmteric_FBS<D_BlockSizes>(D);
	CUberBlockMatrix U_Dinv, U_Dinv_V,  SC = A;
	if(b_use_FBS36) {
		U_Dinv.ProductOf_FBS<U_BlockSizes36, D_BlockSizes>(U, Dinv);
		U_Dinv_V.ProductOf_FBS<U_BlockSizes36, V_BlockSizes36>(U_Dinv, V, true); // only the upper triangle is needed
		U_Dinv_V.AddTo_FBS<SC_BlockSizes36>(SC, -1);
	} else {
		U_Dinv.ProductOf_FBS<U_BlockSizes37, D_BlockSizes>(U, Dinv);
		U_Dinv_V.ProductOf_FBS<U_BlockSizes37, V_BlockSizes37>(U_Dinv, V, true); // only the upper triangle is needed
		U_Dinv_V.AddTo_FBS<SC_BlockSizes37>(SC, -1);
	}
	// ...

	double f_schur_time = 0;
	timer.Accum_DiffSample(f_schur_time);

	printf("stats:\n");
	printf("\tlmcut: %.2f %% of the rank (" PRIsize " landmarks, " PRIsize " cameras)\n",
		100 * double(D.n_BlockColumn_Num()) / lambda_perm.n_BlockColumn_Num(),
		D.n_BlockColumn_Num(), A.n_BlockColumn_Num());
	printf("\t    A: " PRIsize " elem. nnz, %.2f %% density\n",
		A.n_Block_Num() * 6 * 6, 100 * double(A.n_Block_Num() * 2 - A.n_BlockColumn_Num()) /
		(A.n_BlockColumn_Num() * A.n_BlockColumn_Num()));
	printf("\tSchur: " PRIsize " elem. nnz, %.2f %% density (needs %.2f MB)\n",
		SC.n_Block_Num() * 6 * 6, 100 * double(SC.n_Block_Num() * 2 - SC.n_BlockColumn_Num()) /
		(SC.n_BlockColumn_Num() * SC.n_BlockColumn_Num()), SC.n_Allocation_Size_NoLastPage() / 1048576.0);
	printf("\t    U: " PRIsize " elem. nnz, %.2f obs / pt\n",
		U.n_Block_Num() * 3 * 6, double(U.n_Block_Num()) / U.n_BlockColumn_Num());
	printf("\t    D: " PRIsize " elem. nnz, " PRIsize " blocks\n",
		D.n_Block_Num() * 3 * 3, D.n_BlockColumn_Num()); fflush(stdout);
	//printf("\t     : " PRIsize "\n", );

	if(b_test_SC_LU_only) {
		CUberBlockMatrix SC_full;
		SC_full.SelfAdjointViewOf(SC, true, false);
		// get a full symmetric SC

		A.Clear();
		U.Clear();
		D.Clear();
		V.Clear();
		lambda.Clear();
		Dinv.Clear();
		U_Dinv.Clear();
		U_Dinv_V.Clear();
		// free memory

		return LU_Test(SC_full, b_clflush, b_no_FBS, b_count_FLOPs);
	}
	// test LU of SC and exit

	double f_stats_time = 0;
	timer.Accum_DiffSample(f_stats_time);

	printf("square rooting SC\n"); fflush(stdout);
	CMatrixOrdering SC_ord;
	SC_ord.p_BlockOrdering(SC, true);
	const size_t *p_SC_ord = SC_ord.p_Get_InverseOrdering();
	const size_t n_SC_ord_size = SC_ord.n_Ordering_Size();
	// ordering for SC

	double f_SC_ord_time = 0;
	timer.Accum_DiffSample(f_SC_ord_time);

	CUberBlockMatrix SC_perm;
	SC.Permute_UpperTriangular_To(SC_perm, p_SC_ord, n_SC_ord_size, true);
	CUberBlockMatrix S;
	if((b_use_FBS36 && !S.CholeskyOf_FBS<SC_BlockSizes36>(SC_perm)) ||
	   (!b_use_FBS36 && !S.CholeskyOf_FBS<SC_BlockSizes37>(SC_perm))) {
		fprintf(stderr, "error: SC_perm is not pos def; unable to continue\n");
		return -1;
	}
	// permute and sqrt SC

	double f_SC_Chol_time = 0;
	timer.Accum_DiffSample(f_SC_Chol_time);

	printf("getting Schur complement took %.3f sec, out of which:\n", f_ordering_time +
		f_schur_perm_time + f_schur_time + f_SC_ord_time + f_SC_Chol_time);
	printf("\t  ord: %.3f\n", f_ordering_time);
	printf("\t perm: %.3f\n", f_schur_perm_time);
	printf("\tSchur: %.3f\n", f_schur_time);
	printf("\t  amd: %.3f\n", f_SC_ord_time);
	printf("\t Chol: %.3f, " PRIsize " elem. nnz (needs %.2f MB)\n",
		f_SC_Chol_time, S.n_NonZero_Num(), S.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);
	// print some stats

	timer.Accum_DiffSample(f_stats_time);

	// t_odo - evaluate recursive formula on Chol(lambda) // already sort of have that from the SLAM++ runs, except for the precision
	// todo - evaluate recursive formula on Chol(Schur(D))

	printf("\ncalculating sparse landmark marginals\n"); fflush(stdout);

	cs *p_S = S.p_BlockStructure_to_Sparse();

	double f_struct_time = 0;
	timer.Accum_DiffSample(f_struct_time);

	CUberBlockMatrix sp_lm_inv_serial = Dinv; // becomes full rank, no more allocations are needed.

	double f_diag_init_time = 0;
	timer.Accum_DiffSample(f_diag_init_time);

	std::vector<size_t> flops_histogram(Dinv.n_BlockColumn_Num());
	// number of FLOPs per each column

	double f_serial_sparsestr_time = 0;
	double f_serial_bases_time = 0;
	//double f_serial_depcols_time = 0;
	double f_serial_invonly_time = 0;
	double f_serial_invbases_time = 0;

	CUberBlockMatrix serial_S_bases; // want it for stats
	{
		CUberBlockMatrix U_Dinv_perm;
		CUberBlockMatrix &S_bases = serial_S_bases; // rename

		_ASSERTE(sizeof(size_t) == sizeof(csi));
		std::vector<size_t> workspace(n_SC_ord_size * 2);
		// alloc workspace for cs_reach()

		cs *p_St = cs_transpose(p_S, 0);
		// need p_St

		U_Dinv.PermuteTo(U_Dinv_perm, SC_ord.p_Get_InverseOrdering(), // this way and *not* the other way around!
			SC_ord.n_Ordering_Size(), true, false, true);
		// get a permuted view of U_Dinv

		cs *p_B = U_Dinv_perm.p_BlockStructure_to_Sparse();
		// grab its structure (will seldom need to realloc)

		timer.Accum_DiffSample(f_serial_sparsestr_time);

		S.CopyLayoutTo(S_bases);
		{
			cs *p_St = cs_transpose(p_S, 0);
			// need p_St

			/*csi p_col[2] = {0, 1};
			csi n_row = 0;
			cs B;
			B.m = p_St->m;
			B.n = 1;
			B.p = p_col;
			B.i = &n_row;
			B.x = 0;
			B.nzmax = 1;
			B.nz = -1;
			// prepare a single entry matrix*/

			Eigen::MatrixXd/*<double, Eigen::Dynamic, 6>*/ S_dense_basis(S_bases.n_Row_Num(), 6); // todo - implement matrix solving and try to make this row major
			// unit basis matrix

			for(size_t i = 0, n = S_bases.n_BlockColumn_Num(); i < n; ++ i) { // t_odo - do this in parallel (probably explicit matrix reduction rather than locking)
				if(b_use_FBS36) {
					sc_margs_detail::Calculate_UpperTriangularTransposeSolve_Bases_FBS<SC_BlockSizes36>(S_bases,
						S, p_St, i, S_dense_basis, workspace);
				} else {
					sc_margs_detail::Calculate_UpperTriangularTransposeSolve_Bases_FBS<SC_BlockSizes37>(S_bases,
						S, p_St, i, S_dense_basis, workspace);
				}
				// use a function

				/*size_t w = S_bases.n_BlockColumn_Column_Num(i); // t_odo - FBS it
				_ASSERTE(w == 6);
				n_row = i;

				size_t n_first_dep_col = cs_reach(p_St, &B, 0, (csi*)&workspace[0], 0); // modifies p_St but then puts it back
				size_t n_dep_col_num = n_SC_ord_size - n_first_dep_col;
				const size_t *p_dep_col = &workspace[n_first_dep_col];
				// get the list of columns of S that affect block U_Dinv_{i, *}

				S_dense_basis.setZero();
				S_dense_basis.middleRows<6>(S_bases.n_BlockColumn_Base(i)).setIdentity();
				// create a vector of zeros

				for(size_t c = 0; c < 6/*w* /; ++ c) {
					//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
					if(b_use_FBS36) {
						S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes36>(&S_dense_basis.col(c)(0),
							S_dense_basis.rows(), p_dep_col, n_dep_col_num);
					} else {
						S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes37>(&S_dense_basis.col(c)(0),
							S_dense_basis.rows(), p_dep_col, n_dep_col_num);
					}
				}
				// sparse sparse UTTSolve

				for(size_t j = 0; j < n_dep_col_num; ++ j) {
					size_t r = p_dep_col[j];
					size_t y = S_bases.n_BlockColumn_Base(r);
					size_t h = S_bases.n_BlockColumn_Column_Num(r); // those are rows but S_bases is symmetric
#ifdef _DEBUG
					if(j > 0) {
						const size_t r_prev = p_dep_col[j - 1];
						size_t y_prev = S_bases.n_BlockColumn_Base(r_prev);
						size_t h_prev = S_bases.n_BlockColumn_Column_Num(r_prev);
						size_t e_prev = y_prev + h_prev;
						_ASSERTE(S_dense_basis.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
					} else if(y > 0)
						_ASSERTE(S_dense_basis.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
					if(j + 1 == n_dep_col_num)
						_ASSERTE(S_dense_basis.bottomRows(S_dense_basis.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
					// make sure that there are only zeros in between the elements
#endif // _DEBUG
					_ASSERTE(h == 6);
					_ASSERTE(S_bases.n_BlockColumn_Column_Num(r) == 6); // t_odo - FBS it
					S_bases.t_GetBlock_Log(r, i, h, w) =
						S_dense_basis.middleRows<6>(S_bases.n_BlockColumn_Base(r)); // this is transposed (transpose the block as well?): each row is a single basis; this only works if the structure of S is symmetric
				}
				// sparse fill the bases matrix*/
			}

			//S_bases.Rasterize("S_bases.tga", 3); // ...
		}

		timer.Accum_DiffSample(f_serial_bases_time);

		/*std::vector<size_t> dep_col_list, dep_col_head;
		// list of dependent columns for each column, list of head pointers much as in CSC

		dep_col_head.reserve(U_Dinv_perm.n_BlockColumn_Num() + 1);
		dep_col_head.push_back(0);
		//
		for(size_t i = 0, n = U_Dinv_perm.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_first_dep_col = cs_reach(p_St, p_B, 0, (csi*)&workspace[0], 0); // modifies p_St but then puts it back
			size_t n_dep_col_num = n_SC_ord_size - n_first_dep_col;
			const size_t *p_dep_col = &workspace[n_first_dep_col];
			// get the list of columns of S that affect this

			dep_col_list.insert(dep_col_list.end(), p_dep_col, p_dep_col + n_dep_col_num);
			dep_col_head.push_back(dep_col_list.size());
		}
		// calculate reach (not needed for S_bases)

		Eigen::Matrix3d lm_i_cov;
		Eigen::Matrix<double, Eigen::Dynamic, 3> U_Dinv_i_permd(U_Dinv.n_Row_Num(), 3);
		// those can stay allocated, the size does not change throughout the algorithm

		timer.Accum_DiffSample(f_serial_depcols_time);*/

		cs_spfree(p_B);
		cs_spfree(p_St);
		// not needed anymore

		/*for(size_t i = 0, n = U_Dinv_perm.n_BlockColumn_Num(); i < n; ++ i) {
			const size_t b = sp_lm_inv_serial.n_BlockColumn_Base(i);
			const size_t w = sp_lm_inv_serial.n_BlockColumn_Column_Num(i);
			_ASSERTE(w == 3);
			const size_t e = b + w;

			//Eigen::MatrixXd SC_inv_col;
			//CMarginals::Calculate_DenseMarginals_Fast_ColumnBand(SC_inv_col, S, b, e, p_SC_ord, n_SC_ord_size);
			//// invert just a single block column of SC // t_odo - use the sparse sparse formula

			CUberBlockMatrix U_Dinv_i_perm;
			U_Dinv_perm.SliceTo(U_Dinv_i_perm, 0, U_Dinv_perm.n_BlockRow_Num(), i, i + 1, true);
			// grab a single column of U_Dinv_perm (via reference)

			U_Dinv_i_perm.Convert_to_Dense(U_Dinv_i_permd);
			// convert it to a dense matrix

			size_t n_dep_col_num = dep_col_head[i + 1] - dep_col_head[i];
			const size_t *p_dep_col = &dep_col_list[dep_col_head[i]];
			// get the list of columns of S that affect this

			for(size_t c = 0; c < 3/*w* /; ++ c) {
				//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				if(b_use_FBS36) {
					S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes36>(&U_Dinv_i_permd.col(c)(0),
						U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				} else {
					S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes37>(&U_Dinv_i_permd.col(c)(0),
						U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				}
			}
			// sparse sparse UTTSolve

#ifdef _DEBUG
			CUberBlockMatrix SinvT_U_Dinv_i_perm;
			if(b_use_FBS36)
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes36, U_BlockSizes36>(S_bases, U_Dinv_i_perm);
			else
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes37, U_BlockSizes37>(S_bases, U_Dinv_i_perm);
			// gets a sparse matrix, the size of U_Dinv_i_perm

			Eigen::MatrixXd SinvT_U_Dinv_i_permd;
			SinvT_U_Dinv_i_perm.Convert_to_Dense(SinvT_U_Dinv_i_permd);
			double f_error = (SinvT_U_Dinv_i_permd - U_Dinv_i_permd).norm();
			printf("error: %g\n", f_error);
#endif // _DEBUG
		}*/

		timer.Accum_DiffSample(f_serial_invonly_time);

		for(size_t i = 0, n = U_Dinv_perm.n_BlockColumn_Num(); i < n; ++ i) {
			CUberBlockMatrix U_Dinv_i_perm;
			U_Dinv_perm.SliceTo(U_Dinv_i_perm, 0, U_Dinv_perm.n_BlockRow_Num(), i, i + 1, true);
			// grab a single column of U_Dinv_perm (via reference)

			CUberBlockMatrix SinvT_U_Dinv_i_perm;
			if(b_use_FBS36)
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes36, U_BlockSizes36>(S_bases, U_Dinv_i_perm);
			else
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes37, U_BlockSizes37>(S_bases, U_Dinv_i_perm);
			// gets a sparse matrix, the size of U_Dinv_i_perm
		}

		timer.Accum_DiffSample(f_serial_invbases_time);

		f_serial_invonly_time = f_serial_invbases_time;

		/*for(size_t i = 0, n = U_Dinv_perm.n_BlockColumn_Num(); i < n; ++ i) {
			const size_t b = sp_lm_inv_serial.n_BlockColumn_Base(i);
			const size_t w = sp_lm_inv_serial.n_BlockColumn_Column_Num(i);
			_ASSERTE(w == 3);
			const size_t e = b + w;

			//Eigen::MatrixXd SC_inv_col;
			//CMarginals::Calculate_DenseMarginals_Fast_ColumnBand(SC_inv_col, S, b, e, p_SC_ord, n_SC_ord_size);
			//// invert just a single block column of SC // t_odo - use the sparse sparse formula

			CUberBlockMatrix U_Dinv_i_perm;
			U_Dinv_perm.SliceTo(U_Dinv_i_perm, 0, U_Dinv_perm.n_BlockRow_Num(), i, i + 1, true);
			// grab a single column of U_Dinv_perm (via reference)

			U_Dinv_i_perm.Convert_to_Dense(U_Dinv_i_permd);
			// convert it to a dense matrix

			size_t n_dep_col_num = dep_col_head[i + 1] - dep_col_head[i];
			const size_t *p_dep_col = &dep_col_list[dep_col_head[i]];
			// get the list of columns of S that affect this

			for(size_t c = 0; c < 3/*w* /; ++ c) {
				//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				if(b_use_FBS36) {
					S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes36>(&U_Dinv_i_permd.col(c)(0),
						U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				} else {
					S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes37>(&U_Dinv_i_permd.col(c)(0),
						U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				}
			}
			// sparse sparse UTTSolve

			if(n_dep_col_num < U_Dinv_i_perm.n_BlockRow_Num()) {
				lm_i_cov.setZero();
				for(size_t rr = 0; rr < n_dep_col_num; ++ rr) { // reduced-rank product (take advantage of the sparsity of the UTTSolve solution)
					const size_t r = p_dep_col[rr];
					size_t y = U_Dinv_i_perm.n_BlockRow_Base(r);
					size_t h = U_Dinv_i_perm.n_BlockRow_Row_Num(r);
#ifdef _DEBUG
					if(rr > 0) {
						const size_t r_prev = p_dep_col[rr - 1];
						size_t y_prev = U_Dinv_i_perm.n_BlockRow_Base(r_prev);
						size_t h_prev = U_Dinv_i_perm.n_BlockRow_Row_Num(r_prev);
						size_t e_prev = y_prev + h_prev;
						_ASSERTE(U_Dinv_i_permd.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
					} else if(y > 0)
						_ASSERTE(U_Dinv_i_permd.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
					if(rr + 1 == n_dep_col_num)
						_ASSERTE(U_Dinv_i_permd.bottomRows(U_Dinv_i_permd.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
					// make sure that there are only zeros in between the elements
#endif // _DEBUG
					_ASSERTE(h == 6);
					lm_i_cov.noalias() += U_Dinv_i_permd.middleRows<6>(y).transpose() *
						U_Dinv_i_permd.middleRows<6>(y);
				}
			} else
				lm_i_cov.noalias() = U_Dinv_i_permd.transpose() * U_Dinv_i_permd; // full rank product
			// sparse dot product

			sp_lm_inv_serial.t_GetBlock_Log(i, i) += lm_i_cov; // allocates nothing, can run in parallel
			// add it to the given diagonal block
		}*/

		for(size_t i = 0, n = U_Dinv_perm.n_BlockColumn_Num(); i < n; ++ i) {
			CUberBlockMatrix U_Dinv_i_perm;
			U_Dinv_perm.SliceTo(U_Dinv_i_perm, 0, U_Dinv_perm.n_BlockRow_Num(), i, i + 1, true);
			// grab a single column of U_Dinv_perm (via reference)

			CUberBlockMatrix SinvT_U_Dinv_i_perm;
			if(b_use_FBS36)
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes36, U_BlockSizes36>(S_bases, U_Dinv_i_perm);
			else
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes37, U_BlockSizes37>(S_bases, U_Dinv_i_perm);
			// gets a sparse matrix, the size of U_Dinv_i_perm

#if 0
			CUberBlockMatrix lm_i_cov_ii;
			if(b_use_FBS36)
				SinvT_U_Dinv_i_perm.PreMultiplyWithSelfTransposeTo_FBS<U_BlockSizes36>(lm_i_cov_ii);
			else
				SinvT_U_Dinv_i_perm.PreMultiplyWithSelfTransposeTo_FBS<U_BlockSizes37>(lm_i_cov_ii);
			// calculate stuff

			sp_lm_inv_serial.t_GetBlock_Log(i, i) += lm_i_cov_ii.t_GetBlock_Log(0, 0);
			// ATA: 1.237 sec on ff6, 31.101 sec on venice
#else // 0
			CUberBlockMatrix::_TyMatrixXdRef t_lminv_ii = sp_lm_inv_serial.t_GetBlock_Log(i, i);
			if(b_use_FBS36)
				sc_margs_detail::BlockVector_PreMultiplyWithSelfTranspose_Add_FBS<U_BlockSizes36>(t_lminv_ii, SinvT_U_Dinv_i_perm);
			else
				sc_margs_detail::BlockVector_PreMultiplyWithSelfTranspose_Add_FBS<U_BlockSizes37>(t_lminv_ii, SinvT_U_Dinv_i_perm);
			// about 6x cheaper FBS version of that
			// ATA: 0.194 sec on ff6, 5.413 sec on venice
#endif // 0
		}
	}

	double f_serial_inv_time = 0;
	timer.Accum_DiffSample(f_serial_inv_time);

	printf("serial version took %.3f sec, out of which:\n", f_struct_time + f_diag_init_time +
		f_serial_sparsestr_time + f_serial_bases_time + /*f_serial_depcols_time +*/ f_serial_inv_time);
	printf("\tstruc: %.3f\n", f_struct_time);
	printf("\tbases: %.3f (%.2f %% sparsity, S has %.2f %%, needed %.2f MB)\n",
		f_serial_bases_time, 100 * double(serial_S_bases.n_Block_Num() * 6 * 6) /
		(serial_S_bases.n_BlockColumn_Num() * serial_S_bases.n_BlockColumn_Num() * 6 * 6),
		100 * double(S.n_Block_Num() * 6 * 6) / (S.n_BlockColumn_Num() * S.n_BlockColumn_Num() * 6 * 6),
		serial_S_bases.n_Allocation_Size_NoLastPage() / 1048576.0);
	//printf("inverse using bases of S took: %.3f\n", f_serial_invbases_time); // f_serial_invonly_time matches f_serial_invbases_time
	printf("\tdinit: %.3f\n", f_diag_init_time);
	printf("\t  inv: %.3f\n", f_serial_sparsestr_time + /*f_serial_depcols_time +*/ f_serial_inv_time);
	printf("\t\t sstr: %.3f\n", f_serial_sparsestr_time);
	//printf("\t\treach: %.3f (unused now)\n", f_serial_depcols_time);
	printf("\t\t sinv: %.3f\n", f_serial_inv_time);
	printf("\t\t\tbksub: %.3f\n", f_serial_invonly_time);
	printf("\t\t\t  ATA: %.3f\n", f_serial_inv_time - f_serial_invonly_time); fflush(stdout);

	CUberBlockMatrix sp_lm_inv = Dinv;

	double f_lm_inv_reset_time = 0;
	timer.Accum_DiffSample(f_lm_inv_reset_time);

	/*

	{
		CUberBlockMatrix U_Dinv_perm;
		CUberBlockMatrix S_bases;

		_ASSERTE(sizeof(size_t) == sizeof(csi));
		std::vector<size_t> workspace(n_SC_ord_size * 2);
		// alloc workspace for cs_reach()

		cs *p_St = cs_transpose(p_S, 0);
		// need p_St

		U_Dinv.PermuteTo(U_Dinv_perm, p_SC_ord, n_SC_ord_size, true, false, true);
		// get a permuted view of U_Dinv

		cs *p_B = U_Dinv_perm.p_BlockStructure_to_Sparse();
		// grab its structure (will seldom need to realloc)

		timer.Accum_DiffSample(f_serial_sparsestr_time);

		S.CopyLayoutTo(S_bases);
		{
			cs *p_St = cs_transpose(p_S, 0);
			// need p_St

			csi p_col[2] = {0, 1};
			csi n_row = 0;
			cs B;
			B.m = p_St->m;
			B.n = 1;
			B.p = p_col;
			B.i = &n_row;
			B.x = 0;
			B.nzmax = 1;
			B.nz = -1;
			// prepare a single entry matrix

			Eigen::Matrix<double, Eigen::Dynamic, 6> S_dense_basis(S_bases.n_Row_Num(), 6); // t_odo - implement matrix solving and try to make this row major
			// unit basis matrix

			for(size_t i = 0, n = S_bases.n_BlockColumn_Num(); i < n; ++ i) { // t_odo - do this in parallel (probably explicit matrix reduction rather than locking)
				size_t w = S_bases.n_BlockColumn_Column_Num(i); // t_odo - FBS it
				_ASSERTE(w == 6);
				n_row = i;

				size_t n_first_dep_col = cs_reach(p_St, &B, 0, (csi*)&workspace[0], 0); // modifies p_St but then puts it back
				size_t n_dep_col_num = n_SC_ord_size - n_first_dep_col;
				const size_t *p_dep_col = &workspace[n_first_dep_col];
				// get the list of columns of S that affect block U_Dinv_{i, *}

				S_dense_basis.setZero();
				S_dense_basis.middleRows<6>(S_bases.n_BlockColumn_Base(i)).setIdentity();
				// create a vector of zeros

				for(size_t c = 0; c < 6/*w* /; ++ c) {
					//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
					if(b_use_FBS36) {
						S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes36>(&S_dense_basis.col(c)(0),
							S_dense_basis.rows(), p_dep_col, n_dep_col_num);
					} else {
						S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes37>(&S_dense_basis.col(c)(0),
							S_dense_basis.rows(), p_dep_col, n_dep_col_num);
					}
				}
				// sparse sparse UTTSolve

				for(size_t j = 0; j < n_dep_col_num; ++ j) {
					size_t r = p_dep_col[j];
					size_t y = S_bases.n_BlockColumn_Base(r);
					size_t h = S_bases.n_BlockColumn_Column_Num(r); // those are rows but S_bases is symmetric
#ifdef _DEBUG
					if(j > 0) {
						const size_t r_prev = p_dep_col[j - 1];
						size_t y_prev = S_bases.n_BlockColumn_Base(r_prev);
						size_t h_prev = S_bases.n_BlockColumn_Column_Num(r_prev);
						size_t e_prev = y_prev + h_prev;
						_ASSERTE(S_dense_basis.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
					} else if(y > 0)
						_ASSERTE(S_dense_basis.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
					if(j + 1 == n_dep_col_num)
						_ASSERTE(S_dense_basis.bottomRows(S_dense_basis.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
					// make sure that there are only zeros in between the elements
#endif // _DEBUG
					_ASSERTE(h == 6);
					_ASSERTE(S_bases.n_BlockColumn_Column_Num(r) == 6); // t_odo - FBS it
					S_bases.t_GetBlock_Log(r, i, h, w) =
						S_dense_basis.middleRows<6>(S_bases.n_BlockColumn_Base(r)); // this is transposed (transpose the block as well?): each row is a single basis; this only works if the structure of S is symmetric
				}
				// sparse fill the bases matrix
			}

			//S_bases.Rasterize("S_bases.tga", 3); // ...
		}
	*/

	CUberBlockMatrix U_Dinv_perm;
	U_Dinv.PermuteTo(U_Dinv_perm, p_SC_ord, n_SC_ord_size, true, false, true);
	// get a permuted view of U_Dinv (can be shared among the threads)
	
	cs *p_B = U_Dinv_perm.p_BlockStructure_to_Sparse();
	// grab its structure (can be shared among the threads)

	CUberBlockMatrix S_bases;

	std::vector<CUberBlockMatrix> S_bases_thr_list;

	double f_parallel_S_bases_time = 0;

	#pragma omp parallel
	{
#ifdef _OPENMP
		const size_t n_thread_num = omp_get_num_threads();
		const size_t n_thread_id = omp_get_thread_num();
#else // _OPENMP
		static const size_t n_thread_num = 1;
		static const size_t n_thread_id = 0;
#endif // _OPENMP

		#pragma omp master
		{
			S_bases_thr_list.resize(n_thread_num);
		}
		#pragma omp barrier
		// alloc partials in thread 0

		std::vector<size_t> workspace(n_SC_ord_size * 2);
		// alloc thread-private workspace for cs_reach()

		cs *p_St_thr = cs_transpose(p_S, 0);
		// need a private copy of p_St for each thread

	//	cs *p_B = 0;
		// thread-private workspace for the sparsity pattern

		{
			CUberBlockMatrix &S_bases_thr = S_bases_thr_list[n_thread_id];
			// rename

			S.CopyLayoutTo(S_bases_thr);

			/*csi p_col[2] = {0, 1};
			csi n_row = 0;
			cs B;
			B.m = p_St_thr->m;
			B.n = 1;
			B.p = p_col;
			B.i = &n_row;
			B.x = 0;
			B.nzmax = 1;
			B.nz = -1;*/
			// prepare a single entry matrix

			Eigen::MatrixXd/*<double, Eigen::Dynamic, 6>*/ S_dense_basis(S.n_Row_Num(), 6); // todo - implement matrix solving and try to make this row major
			// unit basis matrix

			const size_t n = S.n_BlockColumn_Num();
			_ASSERTE(n <= INT_MAX);
			const int _n = int(n);
			#pragma omp for schedule(dynamic, 1) // t_odo - dynamic schedule? each column will likely have a different cost (todo - build histograms)
			for(int i = 0; i < _n; ++ i) {
				if(b_use_FBS36) {
					sc_margs_detail::Calculate_UpperTriangularTransposeSolve_Bases_FBS<SC_BlockSizes36>(S_bases_thr,
						S, p_St_thr, i, S_dense_basis, workspace);
				} else {
					sc_margs_detail::Calculate_UpperTriangularTransposeSolve_Bases_FBS<SC_BlockSizes37>(S_bases_thr,
						S, p_St_thr, i, S_dense_basis, workspace);
				}
				// use the nice function instead

				/*size_t w = S.n_BlockColumn_Column_Num(i); // t_odo - FBS it
				_ASSERTE(w == 6);
				n_row = i;

				size_t n_first_dep_col = cs_reach(p_St_thr, &B, 0, &workspace[0], 0); // modifies p_St but then puts it back
				size_t n_dep_col_num = n_SC_ord_size - n_first_dep_col;
				_ASSERTE(sizeof(csi) == sizeof(size_t));
				const size_t *p_dep_col = (size_t*)&workspace[n_first_dep_col];
				// get the list of columns of S that affect block U_Dinv_{i, *}

				S_dense_basis.setZero();
				S_dense_basis.middleRows<6>(S.n_BlockColumn_Base(i)).setIdentity();
				// create a vector of zeros

				for(size_t c = 0; c < 6/*w* /; ++ c) {
					//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
					if(b_use_FBS36) {
						S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes36>(&S_dense_basis.col(c)(0),
							S_dense_basis.rows(), p_dep_col, n_dep_col_num);
					} else {
						S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes37>(&S_dense_basis.col(c)(0),
							S_dense_basis.rows(), p_dep_col, n_dep_col_num);
					}
				}
				// sparse sparse UTTSolve

				for(size_t j = 0; j < n_dep_col_num; ++ j) {
					size_t r = p_dep_col[j];
					size_t y = S.n_BlockColumn_Base(r);
					size_t h = S.n_BlockColumn_Column_Num(r); // those are rows but S is symmetric
#ifdef _DEBUG
					if(j > 0) {
						const size_t r_prev = p_dep_col[j - 1];
						size_t y_prev = S.n_BlockColumn_Base(r_prev);
						size_t h_prev = S.n_BlockColumn_Column_Num(r_prev);
						size_t e_prev = y_prev + h_prev;
						_ASSERTE(S_dense_basis.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
					} else if(y > 0)
						_ASSERTE(S_dense_basis.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
					if(j + 1 == n_dep_col_num)
						_ASSERTE(S_dense_basis.bottomRows(S_dense_basis.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
					// make sure that there are only zeros in between the elements
#endif // _DEBUG
					_ASSERTE(h == 6);
					_ASSERTE(S.n_BlockColumn_Column_Num(r) == 6); // t_odo - FBS it
					S_bases_thr.t_GetBlock_Log(r, i, h, w) =
						S_dense_basis.middleRows<6>(S.n_BlockColumn_Base(r)); // this is transposed (transpose the block as well?): each row is a single basis; this only works if the structure of S is symmetric
				}
				// sparse fill the bases matrix*/
			}
		}

		#pragma omp barrier
		// wait for all the threads to compute their bases

		#pragma omp master
		{
			S_bases.Swap(S_bases_thr_list.front()); // start with 0
			for(size_t i = 1, n = n_thread_num; i < n; ++ i)
				S_bases_thr_list[i].AddTo(S_bases); // no need for FBS, no two blocks will overlap
			// simple serial reduction in thread 0, could do a parallel one

			std::vector<CUberBlockMatrix> empty;
			S_bases_thr_list.swap(empty);

			timer.Accum_DiffSample(f_parallel_S_bases_time);
		}
		// reduce the bases

		#pragma omp barrier
		// synchronize the threads before continuing

		Eigen::Matrix3d lm_i_cov;
		Eigen::Matrix<double, Eigen::Dynamic, 3> U_Dinv_i_permd(U_Dinv.n_Row_Num(), 3);
		// those can stay allocated, the size does not change throughout the algorithm

		const size_t n = Dinv.n_BlockColumn_Num();
		_ASSERTE(n <= INT_MAX);
		const int _n = int(n);
		#pragma omp for schedule(dynamic, 1) // t_odo - dynamic schedule? each column will likely have a different cost (todo - build histograms)
		for(int i = 0; i < _n; ++ i) {
#if 0
			const size_t b = sp_lm_inv.n_BlockColumn_Base(i);
			const size_t w = sp_lm_inv.n_BlockColumn_Column_Num(i);
			_ASSERTE(w == 3);
			const size_t e = b + w;

			//Eigen::MatrixXd SC_inv_col;
			//CMarginals::Calculate_DenseMarginals_Fast_ColumnBand(SC_inv_col, S, b, e, p_SC_ord, n_SC_ord_size);
			//// invert just a single block column of SC // t_odo - use the sparse sparse formula

			CUberBlockMatrix U_Dinv_i;
			U_Dinv.SliceTo(U_Dinv_i, 0, U_Dinv.n_BlockRow_Num(), i, i + 1, true);
			// grab a single column of U_Dinv via reference

#if 0
			Eigen::MatrixXd U_Dinv_id;
			U_Dinv_i.Convert_to_Dense(U_Dinv_id);
			// convert it to a dense matrix

			lm_i_cov = U_Dinv_id.transpose() * SC_inv_dense * U_Dinv_id; // this works but needs all of the inverse of SC
			// get the i-th covariance diag. block
#elif 0
			Eigen::MatrixXd U_Dinv_id;
			U_Dinv_i.Convert_to_Dense(U_Dinv_id);
			// convert it to a dense matrix

			lm_i_cov.resize(U_Dinv_id.rows(), U_Dinv_id.cols());
			for(size_t c = 0; c < w; ++ c) {
				S.InversePermute_LeftHandSide_Vector(&lm_i_cov.col(c)(0), &U_Dinv_id.col(c)(0),
					U_Dinv_id.rows(), p_SC_ord, n_SC_ord_size);
				// lm_i_cov(perm[r], c) = U_Dinv_id(i, c) for each row r, using S.m_block_cols for block sizes

				S.UpperTriangularTranspose_Solve(&lm_i_cov.col(c)(0), lm_i_cov.rows());
				// UTTSolve
			}
			lm_i_cov = lm_i_cov.transpose() * lm_i_cov;
			// works but does not take advantage of the sparsity of U_Dinv
#else
			CUberBlockMatrix U_Dinv_i_perm;
			U_Dinv_i.PermuteTo(U_Dinv_i_perm, p_SC_ord, n_SC_ord_size, true, false, true);
			// get a permuted view of U_Dinv_i

			U_Dinv_i_perm.Convert_to_Dense(U_Dinv_i_permd);
			// convert it to a dense matrix

			p_B = U_Dinv_i_perm.p_BlockStructure_to_Sparse(p_B);
			// grab its structure (will seldom need to realloc)

			size_t n_first_dep_col = cs_reach(p_St_thr, p_B, 0, &workspace[0], 0); // modifies p_St but then puts it back
			size_t n_dep_col_num = n_SC_ord_size - n_first_dep_col;
			_ASSERTE(sizeof(size_t) == sizeof(csi));
			const size_t *p_dep_col = (const size_t*)&workspace[n_first_dep_col];
			// get the list of columns of S that affect this

			//lm_i_cov = U_Dinv_i_permd;//.resize(U_Dinv_id.rows(), U_Dinv_id.cols());
			for(size_t c = 0; c < 3/*w*/; ++ c) {
				//S.InversePermute_LeftHandSide_Vector(&lm_i_cov.col(c)(0), &U_Dinv_id.col(c)(0),
				//	U_Dinv_id.rows(), p_SC_ord, n_SC_ord_size);
				// lm_i_cov(perm[r], c) = U_Dinv_id(i, c) for each row r, using S.m_block_cols for block sizes
				// can do that before converting it to a dense matrix

				//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				if(b_use_FBS36) {
					S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes36>(&U_Dinv_i_permd.col(c)(0),
						U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				} else {
					S.UpperTriangularTranspose_Solve_FBS<SC_BlockSizes37>(&U_Dinv_i_permd.col(c)(0),
						U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				}
				// sparse sparse UTTSolve
			}
			if(n_dep_col_num < U_Dinv_i_perm.n_BlockRow_Num()) {
				lm_i_cov.setZero();
				for(size_t rr = 0; rr < n_dep_col_num; ++ rr) { // reduced-rank product (take advantage of the sparsity of the UTTSolve solution)
					const size_t r = p_dep_col[rr];
					size_t y = U_Dinv_i_perm.n_BlockRow_Base(r);
					size_t h = U_Dinv_i_perm.n_BlockRow_Row_Num(r);
#ifdef _DEBUG
					if(rr > 0) {
						const size_t r_prev = p_dep_col[rr - 1];
						size_t y_prev = U_Dinv_i_perm.n_BlockRow_Base(r_prev);
						size_t h_prev = U_Dinv_i_perm.n_BlockRow_Row_Num(r_prev);
						size_t e_prev = y_prev + h_prev;
						_ASSERTE(U_Dinv_i_permd.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
					} else if(y > 0)
						_ASSERTE(U_Dinv_i_permd.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
					if(rr + 1 == n_dep_col_num)
						_ASSERTE(U_Dinv_i_permd.bottomRows(U_Dinv_i_permd.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
					// make sure that there are only zeros in between the elements
#endif // _DEBUG
					_ASSERTE(h == 6);
					lm_i_cov.noalias() += U_Dinv_i_permd.middleRows<6>(y).transpose() *
						U_Dinv_i_permd.middleRows<6>(y);
				}
			} else
				lm_i_cov.noalias() = U_Dinv_i_permd.transpose() * U_Dinv_i_permd; // full rank product

#endif // 0

			sp_lm_inv.t_GetBlock_Log(i, i) += lm_i_cov; // allocates nothing, can run in parallel
			// add it to the given diagonal block
#else // 0
			CUberBlockMatrix U_Dinv_i_perm;
			U_Dinv_perm.SliceTo(U_Dinv_i_perm, 0, U_Dinv_perm.n_BlockRow_Num(), i, i + 1, true);
			// grab a single column of U_Dinv_perm (via reference)

			CUberBlockMatrix SinvT_U_Dinv_i_perm;
			if(b_use_FBS36)
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes36, U_BlockSizes36>(S_bases, U_Dinv_i_perm);
			else
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes37, U_BlockSizes37>(S_bases, U_Dinv_i_perm);
			// gets a sparse matrix, the size of U_Dinv_i_perm

			CUberBlockMatrix::_TyMatrixXdRef t_lminv_ii = sp_lm_inv.t_GetBlock_Log(i, i);
			if(b_use_FBS36)
				sc_margs_detail::BlockVector_PreMultiplyWithSelfTranspose_Add_FBS<U_BlockSizes36>(t_lminv_ii, SinvT_U_Dinv_i_perm);
			else
				sc_margs_detail::BlockVector_PreMultiplyWithSelfTranspose_Add_FBS<U_BlockSizes37>(t_lminv_ii, SinvT_U_Dinv_i_perm);
#endif // 0
		}

		cs_spfree(p_St_thr);
	//	cs_spfree(p_B);
	}
	// calculate block diagonal covariances of only the landmarks

	cs_spfree(p_S);
	cs_spfree(p_B);

	double f_inverse_time = 0, f_lminv_time = 0;
	timer.Accum_DiffSample(f_inverse_time);
	f_lminv_time = f_inverse_time + f_struct_time + f_diag_init_time + f_parallel_S_bases_time;

	printf("parallel version took %.3f sec, out of which:\n", f_lminv_time);
	printf("\tstruc: %.3f\n", f_struct_time);
	printf("\tbases: %.3f (%.2f %% sparsity, S has %.2f %%, needed %.2f MB)\n",
		f_parallel_S_bases_time, 100 * double(S_bases.n_Block_Num() * 6 * 6) /
		(S_bases.n_BlockColumn_Num() * S_bases.n_BlockColumn_Num() * 6 * 6),
		100 * double(S.n_Block_Num() * 6 * 6) / (S.n_BlockColumn_Num() *
		S.n_BlockColumn_Num() * 6 * 6), S_bases.n_Allocation_Size_NoLastPage() / 1048576.0);
	printf("\tdinit: %.3f\n", f_diag_init_time);
	printf("\t  inv: %.3f\n", f_inverse_time); fflush(stdout);

	timer.Accum_DiffSample(f_stats_time);

	CUberBlockMatrix rcs_cov;
	{
		CUberBlockMatrix margs_ordered;
		if(b_use_FBS36) {
			CMarginals::Calculate_DenseMarginals_Recurrent_FBS<SC_BlockSizes36>(margs_ordered, S,
				SC_ord, mpart_Diagonal, false);
		} else {
			CMarginals::Calculate_DenseMarginals_Recurrent_FBS<SC_BlockSizes37>(margs_ordered, S,
				SC_ord, mpart_Diagonal, false);
		}

		margs_ordered.Permute_UpperTriangular_To(rcs_cov, SC_ord.p_Get_Ordering(),
			SC_ord.n_Ordering_Size(), false); // no share! the original will be deleted
	}

	double f_rcs_inverse_time = 0;
	timer.Accum_DiffSample(f_rcs_inverse_time);

	printf("\ncalculating recursive inverse of cameras using SC\n");
	printf("it took %.3f sec (recovered " PRIsize " blocks)\n",
		f_rcs_inverse_time, rcs_cov.n_Block_Num()); fflush(stdout);

	printf("\ncalculating recursive inverse\n"); fflush(stdout);
	CMatrixOrdering lam_ord;
	lam_ord.p_BlockOrdering(lambda_perm, true); // w.r.t. lambda_perm
	const size_t *p_lam_ord = lam_ord.p_Get_InverseOrdering();
	const size_t n_lam_ord_size = lam_ord.n_Ordering_Size();
	// ordering for lambda

	double f_lambda_amd_time = 0;
	timer.Accum_DiffSample(f_lambda_amd_time);

	CUberBlockMatrix lambda_amd;
	lambda_perm.Permute_UpperTriangular_To(lambda_amd, p_lam_ord, n_lam_ord_size, true);

	double f_lambda_perm_time = 0;
	timer.Accum_DiffSample(f_lambda_perm_time);

	/*typedef CConcatTypelist<CConcatTypelist<SC_BlockSizes, U_BlockSizes>::_TyResult,
		CConcatTypelist<V_BlockSizes, D_BlockSizes>::_TyResult>::_TyResult Lambda_BlockSizes;*/
	typedef CConcatTypelist<CConcatTypelist<SC_BlockSizes36, U_BlockSizes36>::_TyResult,
		CConcatTypelist<V_BlockSizes36, D_BlockSizes>::_TyResult>::_TyResult Lambda_BlockSizes36;
	typedef CConcatTypelist<CConcatTypelist<SC_BlockSizes37, U_BlockSizes37>::_TyResult,
		CConcatTypelist<V_BlockSizes37, D_BlockSizes>::_TyResult>::_TyResult Lambda_BlockSizes37;

	CUberBlockMatrix R;
	if((b_use_FBS36 && !R.CholeskyOf_FBS<Lambda_BlockSizes36>(lambda_amd)) ||
	   (!b_use_FBS36 && !R.CholeskyOf_FBS<Lambda_BlockSizes37>(lambda_amd))) {
		fprintf(stderr, "error: got not pos def when factorizing lambda\n");
		return -1;
	}

	double f_lambda_chol_time = 0;
	timer.Accum_DiffSample(f_lambda_chol_time);

	{
		const char *p_s_filename_pat = "lambda_AMD";
		bool b_symmetric = true;

		char p_s_filename[256];
		cs *p_mat = lambda_amd.p_BlockStructure_to_Sparse();
		cs *p_mat2 = R.p_BlockStructure_to_Sparse();
		sprintf(p_s_filename, "sc_0.1_%s_SS.tga", p_s_filename_pat);
		if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
		   p_mat, 0, 640, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
		sprintf(p_s_filename, "sc_0.1_%s_SS_AA.tga", p_s_filename_pat);
		if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
		   p_mat, 0, 640/*1024*/, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing

		p_s_filename_pat = "Chol_lambda";
		b_symmetric = false;

		sprintf(p_s_filename, "sc_0.1_%s_SS.tga", p_s_filename_pat);
		if(!p_mat2 || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
		   p_mat2, 0, 640, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
		sprintf(p_s_filename, "sc_0.1_%s_SS_AA.tga", p_s_filename_pat);
		if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
		   p_mat2, 0, 640/*1024*/, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing

		p_s_filename_pat = "Chol_lambda_with-fill";

		sprintf(p_s_filename, "sc_0.1_%s_SS.tga", p_s_filename_pat);
		if(!p_mat2 || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
		   p_mat2, p_mat, 640, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
		sprintf(p_s_filename, "sc_0.1_%s_SS_AA.tga", p_s_filename_pat);
		if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
		   p_mat2, p_mat, 640/*1024*/, b_symmetric))
			remove(p_s_filename); // avoid keeping images from previous runs, would get confusing

		cs_spfree(p_mat);
		cs_spfree(p_mat2);
	}

	double f_lambda_chol_matdump_time = 0;
	timer.Accum_DiffSample(f_lambda_chol_matdump_time);

	double f_lambda_recformula_time = 0;
	CUberBlockMatrix margs_recursive;
	{
		CUberBlockMatrix margs_ordered;
		if(b_use_FBS36) {
			CMarginals::Calculate_DenseMarginals_Recurrent_FBS<Lambda_BlockSizes36>(margs_ordered, R,
				lam_ord, mpart_Diagonal, false);
		} else {
			CMarginals::Calculate_DenseMarginals_Recurrent_FBS<Lambda_BlockSizes37>(margs_ordered, R,
				lam_ord, mpart_Diagonal, false);
		}
		// calculate the thing

		timer.Accum_DiffSample(f_lambda_recformula_time);

		/*{
			CUberBlockMatrix empty;
			R.Swap(empty);
		}*/
		// delete R, don't need it and it eats a lot of memory

		margs_ordered.Permute_UpperTriangular_To(margs_recursive, lam_ord.p_Get_Ordering(),
			lam_ord.n_Ordering_Size(), false); // no share! the original will be deleted
	}

	double f_lambda_unperm_time = 0;
	timer.Accum_DiffSample(f_lambda_unperm_time);

	printf("it took %.3f sec, out of which:\n", f_lambda_unperm_time +
		f_lambda_recformula_time + f_lambda_chol_time + f_lambda_perm_time + f_lambda_amd_time);
	printf("\t  amd: %.3f\n", f_lambda_amd_time);
	printf("\t perm: %.3f\n", f_lambda_perm_time);
	printf("\t Chol: %.3f, " PRIsize " elem. nnz (needs %.2f MB)\n",
		f_lambda_chol_time, R.n_NonZero_Num(), R.n_Allocation_Size_NoLastPage() / 1048576.0);
	printf("\trform: %.3f\n", f_lambda_recformula_time);
	printf("\tunprm: %.3f\n", f_lambda_unperm_time); fflush(stdout);

	CUberBlockMatrix cam_cov_rec, lm_cov_rec;
	margs_recursive.SliceTo(cam_cov_rec, 0, n_matrix_cut, 0, n_matrix_cut, true);
	margs_recursive.SliceTo(lm_cov_rec, n_matrix_cut, n_size, n_matrix_cut, n_size, true);
	// get the submatrices

	{
		size_t n_common_caminv_block_num;
		double f_caminv_error = CMarginals::f_IncompleteDifference(n_common_caminv_block_num, cam_cov_rec, rcs_cov);
		printf("difference of recursive and SC camera marginals\n"
			"\t  abs: %g\n"
			"\t  rel: %g\n"
			"\tshblk: " PRIsize " blocks (recursive formula of ,\\ recovered " PRIsize ")\n",
			f_caminv_error, f_caminv_error / cam_cov_rec.f_Norm(),
			n_common_caminv_block_num, cam_cov_rec.n_Block_Num());
		size_t n_common_lminv_block_num;
		double f_lminv_error = CMarginals::f_IncompleteDifference(n_common_lminv_block_num, lm_cov_rec, sp_lm_inv);
		printf("difference of recursive and parallel SC landmark marginals\n"
			"\t  abs: %g\n"
			"\t  rel: %g\n"
			"\tshblk: " PRIsize " blocks (recursive formula of ,\\ recovered " PRIsize ")\n",
			f_lminv_error, f_lminv_error / lm_cov_rec.f_Norm(),
			n_common_lminv_block_num, lm_cov_rec.n_Block_Num()); fflush(stdout);
		f_lminv_error = CMarginals::f_IncompleteDifference(n_common_lminv_block_num, lm_cov_rec, sp_lm_inv_serial);
		printf("difference of recursive and serial SC landmark marginals\n"
			"\t  abs: %g\n"
			"\t  rel: %g\n"
			"\tshblk: " PRIsize " blocks (recursive formula of ,\\ recovered " PRIsize ")\n",
			f_lminv_error, f_lminv_error / lm_cov_rec.f_Norm(),
			n_common_lminv_block_num, lm_cov_rec.n_Block_Num()); fflush(stdout);
	}
	// compare the SC results to the ones of recursive formula

	timer.Accum_DiffSample(f_stats_time);

	CUberBlockMatrix lm_margs_recursive, SC2;
	try { // could potentially run out of memory in case Ainv becomes dense
		double f_ordering2_time = 0, f_schur2_perm_time = 0;

		printf("\ncalculating Schur complement of D ...\n"); fflush(stdout);

		/*size_t n_subgraph_num = 0;
		std::vector<size_t> vertex_subgraph(A.n_BlockColumn_Num(), size_t(-1));
		for(size_t i = vertex_subgraph.size(); i > 0;) { // A is upper triangular
			-- i; // here

			if(vertex_subgraph[i] != size_t(-1))
				continue;
			// in case a vertex is not marked yet

			std::vector<size_t> recurse_stack; // i dont like including <stack> just for the sake of it
			recurse_stack.push_back(i);
			while(!recurse_stack.empty()) {
				size_t n_vertex = recurse_stack.back();
				recurse_stack.erase(recurse_stack.end() - 1);
				_ASSERTE(vertex_subgraph[n_vertex] == size_t(-1) ||
					vertex_subgraph[n_vertex] == n_subgraph_num); // might have been already marked by a loop
				// get a vertex ...

				if(vertex_subgraph[n_vertex] == size_t(-1)) {
					vertex_subgraph[n_vertex] = n_subgraph_num;
					// assign a subgraph id

					for(size_t j = 0, m = A.n_BlockColumn_Block_Num(n_vertex); j < m; ++ j) {
						size_t n_neighbor = A.n_Block_Row(n_vertex, j);
						if(n_neighbor >= n_vertex)
							break; // only up to the diagonal element, avoid looping on self

						_ASSERTE(vertex_subgraph[n_neighbor] == size_t(-1) ||
							vertex_subgraph[n_neighbor] == n_subgraph_num);
						if(vertex_subgraph[n_neighbor] == size_t(-1))
							recurse_stack.push_back(n_neighbor);
					}
					// recurse to all the neighbors
				}
			}
			// mark all the connected vertices

			++ n_subgraph_num;
		}
		// t_odo - debug this on non-trivial graphs, make this a function

		for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i)
			vertex_subgraph[i] = n_subgraph_num - 1 - vertex_subgraph[i];
		// reverse subgraph to gain identity ordering on diagonal matrices
		// (the subgraph ids are assigned backwards so otherwise it would
		// yield a mirror ordering)

#ifdef _DEBUG
		for(size_t i = 0, n = A.n_BlockColumn_Num(); i < n; ++ i) {
			for(size_t j = 0, m = A.n_BlockColumn_Block_Num(i); j < m; ++ j) {
				size_t n_neighbor = A.n_Block_Row(i, j);
				if(n_neighbor >= i)
					break; // ignore the lower half of A, the algorithm above does that as well
				_ASSERTE(vertex_subgraph[i] == vertex_subgraph[n_neighbor]);
			}
		}
		// make sure that all the neighboring vertices indeed are in the same subgraph
#endif // _DEBUG*/

		std::vector<size_t> vertex_subgraph;
		size_t n_subgraph_num = CMatrixOrdering::n_Find_BlockStructure_Subgraphs(vertex_subgraph, A);
		// find subgraphs

		printf("debug: there are " PRIsize " subgraphs in A\n", n_subgraph_num); fflush(stdout);

		if(b_no_SchurD) {
			printf("debug: bailing out\n"); fflush(stdout);
			throw std::bad_alloc();
		}

		/*std::vector<size_t> vertex_order(vertex_subgraph.size()); // cannot work inplace vertex_subgraph
		{
			std::vector<size_t> subgraph_sizes(n_subgraph_num, 0);
			for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i) { // sum of vertex ranks, generate subgraph sizes
				size_t n_subgraph = vertex_subgraph[i];
				_ASSERTE(n_subgraph != size_t(-1)); // make sure all the vertices are assigned
				vertex_order[i] = subgraph_sizes[n_subgraph];
				++ subgraph_sizes[n_subgraph]; // could increment twice to save vertex_order of storage in exchange for subgraph_sizes storage
			}
			for(size_t i = 0, n = subgraph_sizes.size(), n_sum = 0; i < n; ++ i) { // exclusive sum
				size_t n_temp = n_sum;
				n_sum += subgraph_sizes[i];
				subgraph_sizes[i] = n_temp;
			}
			for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i) // add exclusive sums of subgraph sizes
				vertex_order[i] += subgraph_sizes[vertex_subgraph[i]];
		}
		// generate vertex ordering that orders subgraphs adjacently

		_ASSERTE(vertex_order.empty() || CMatrixOrdering::b_IsValidOrdering(&vertex_order.front(), vertex_order.size()));*/
		// make sure it is a valid ordering to begin with

		CMatrixOrdering A_ord;
		const size_t *p_inv_vertex_order =
			A_ord.p_InvertOrdering(A_ord.p_AdjacentLabel_Ordering(vertex_subgraph,
			n_subgraph_num), vertex_subgraph.size());
		// generate vertex ordering that orders subgraphs adjacently, invert the ordering

		timer.Accum_DiffSample(f_ordering2_time);

		CUberBlockMatrix Ainv;
		{
			CUberBlockMatrix A_perm;
			A.Permute_UpperTriangular_To(A_perm, A_ord.p_Get_Ordering(), A_ord.n_Ordering_Size(), true);

			timer.Accum_DiffSample(f_schur2_perm_time);

			CUberBlockMatrix Ainv_perm;
			if(b_use_FBS36)
				Ainv_perm.InverseOf_Symmteric_FBS<A_BlockSizes36>(A_perm, true); // this would fill in the matrix
			else
				Ainv_perm.InverseOf_Symmteric_FBS<A_BlockSizes37>(A_perm, true); // this would fill in the matrix
			Ainv_perm.PermuteTo(Ainv, p_inv_vertex_order, A_ord.n_Ordering_Size(), true, true, false); // don't share, will be deleted // need to use full ordering to also get the lower parts of the blocks (in case the matrix is block block diagonal)
			// ordering transcends inverse, can permute to block diagonal form, invert, and permute back for simplicity
		}
		CUberBlockMatrix V_Ainv, V_Ainv_U;
		if(b_use_FBS36) {
			V_Ainv.ProductOf_FBS<V_BlockSizes36, A_BlockSizes36>(V, Ainv);
			V_Ainv.MultiplyToWith_FBS<V_BlockSizes36, U_BlockSizes36>(V_Ainv_U, U, true); // only the upper triangle is needed
		} else {
			V_Ainv.ProductOf_FBS<V_BlockSizes37, A_BlockSizes37>(V, Ainv);
			V_Ainv.MultiplyToWith_FBS<V_BlockSizes37, U_BlockSizes37>(V_Ainv_U, U, true); // only the upper triangle is needed
		}
		/*CUberBlockMatrix*/ SC2 = D;
		V_Ainv_U.AddTo_FBS<SC2_BlockSizes>(SC2, -1);
		// ...

		double f_schur2_time = 0;
		timer.Accum_DiffSample(f_schur2_time);

		printf("stats:\n");
		printf("\tSchur: " PRIsize " elem. nnz, %.2f %% density (needs %.2f MB)\n",
			SC2.n_Block_Num() * 6 * 6, 100 * double(SC2.n_Block_Num() * 2 - SC2.n_BlockColumn_Num()) /
			(SC2.n_BlockColumn_Num() * SC2.n_BlockColumn_Num()), SC2.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);
		printf("\t  ord: %.3f\n", f_ordering2_time);
		printf("\t perm: %.3f\n", f_schur2_perm_time);
		printf("\tSchur: %.3f (needs %.2f MB)\n", f_schur2_time, SC2.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);

		timer.Accum_DiffSample(f_stats_time);

		printf("square rooting SC\n"); fflush(stdout);
		CMatrixOrdering SC2_ord;
		SC2_ord.p_BlockOrdering(SC2, true);
		const size_t *p_SC2_ord = SC2_ord.p_Get_InverseOrdering();
		const size_t n_SC2_ord_size = SC2_ord.n_Ordering_Size();
		// ordering for SC

		double f_SC2_ord_time = 0;
		timer.Accum_DiffSample(f_SC2_ord_time);

		if(b_no_SchurD_Chol) {
			printf("getting Schur complement took %.3f sec, out of which:\n", f_schur2_time +
				f_ordering2_time + f_schur2_perm_time + f_SC2_ord_time);
			printf("\t  ord: %.3f\n", f_ordering2_time);
			printf("\t perm: %.3f\n", f_schur2_perm_time);
			printf("\tSchur: %.3f (needs %.2f MB)\n", f_schur2_time, SC2.n_Allocation_Size_NoLastPage() / 1048576.0);
			printf("\t  amd: %.3f\n", f_SC2_ord_time); fflush(stdout);
			// print some stats
		}

		CUberBlockMatrix SC2_perm;
		SC2.Permute_UpperTriangular_To(SC2_perm, p_SC2_ord, n_SC2_ord_size, true);

		double f_SC2_Chol_time = 0;
		timer.Accum_DiffSample(f_SC2_Chol_time);

		{
			const char *p_s_filename_pat = "SC2_perm";
			const bool b_symmetric = true;

			char p_s_filename[256];
			cs *p_mat = SC2_perm.p_BlockStructure_to_Sparse();
			sprintf(p_s_filename, "sc_8.1_%s_SS.tga", p_s_filename_pat);
			if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
			   p_mat, 0, 640, b_symmetric))
				remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			sprintf(p_s_filename, "sc_8.1_%s_SS_AA.tga", p_s_filename_pat);
			if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
			   p_mat, 0, 640/*8192*/, b_symmetric))
				remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			cs_spfree(p_mat);
		}

		size_t n_worst_case_nnz = 0;
		bool b_overflow = false;
		for(size_t i = 0, n = SC2_perm.n_BlockColumn_Num(); i < n; ++ i) {
			_ASSERTE(SC2_perm.n_BlockColumn_Block_Num(i)); // should not be rank deficient
			size_t n_first_block_row = SC2_perm.n_BlockColumn_Base(SC2_perm.n_Block_Row(i, 0)); // should be row but it is symmetric
			size_t n_col_width = SC2_perm.n_BlockColumn_Column_Num(i);
			size_t n_last_diag_row = SC2_perm.n_BlockColumn_Base(i) + SC2_perm.n_BlockColumn_Column_Num(i); // should be rows but it is symmetric
			_ASSERTE(n_last_diag_row > n_first_block_row);
			size_t n_max_nnz = n_last_diag_row - n_first_block_row;
			if(n_max_nnz > SIZE_MAX / n_col_width) {
				b_overflow = true;
				break;
			}
			n_max_nnz *= n_col_width;
			if(n_worst_case_nnz > SIZE_MAX - n_max_nnz) {
				b_overflow = true;
				break;
			}
			n_worst_case_nnz += n_max_nnz;
		}
		printf("\tfactr: upper bound " PRIsize " elem. nnz (would need %.2f MB)\n",
			n_worst_case_nnz, n_worst_case_nnz * (8 / 1048576.0)); fflush(stdout);
		// see how much space would be required to represent the factor, based on its frontline

		double f_matwrite_time = 0; // dont want this in stats
		timer.Accum_DiffSample(f_matwrite_time);

		if(b_no_SchurD_Chol) {
			printf("debug: bailing out\n"); fflush(stdout);
			throw std::bad_alloc();
		}

		double f_matslice_time = 0;

		CUberBlockMatrix S2_sliced;
		{
			size_t n = SC2_perm.n_BlockColumn_Num();
			size_t n_slice_num = std::max(size_t(1), std::min(size_t(32), n / 16));
			CUberBlockMatrix SC2_perm_corner;
			size_t n_prev_block_num = 0;
			for(size_t i = 0; i < n_slice_num; ++ i) {
				size_t n_block_num = (n < SIZE_MAX / n_slice_num)? // would there be an overflow?
					n * (i + 1) / n_slice_num : // nice, more precise formula
					((n + 1 == n_slice_num)? n : (n / n_slice_num) * (i + 1)); // constant step formula
				_ASSERTE(n_block_num > n_prev_block_num);
				// calculate how many blocks to slice here

				SC2_perm.SliceTo(SC2_perm_corner, n_block_num, n_block_num, true);
				// get a slice

				timer.Accum_DiffSample(f_matslice_time); // will have the stats included in there

				if(!S2_sliced.CholeskyOf_FBS<SC2_BlockSizes>(SC2_perm_corner, n_prev_block_num)) {
					if(i)
						printf("\n");
					fprintf(stderr, "error: SC2_perm is not pos def; unable to continue\n");
					return -1;
				}
				// use resumed cholesky on that slice

				timer.Accum_DiffSample(f_SC2_Chol_time);

				printf("\tinfac: " PRIsize " / " PRIsize " columns added: %.3f, "
					PRIsize " elem. nnz (needs %.2f MB)\r", n_block_num, n, f_SC2_Chol_time,
					S2_sliced.n_NonZero_Num(), S2_sliced.n_Allocation_Size_NoLastPage() /
					1048576.0); fflush(stdout);
				// show some incremental info

				timer.Accum_DiffSample(f_stats_time);

				n_prev_block_num = n_block_num;
			}
			_ASSERTE(n_prev_block_num == n);
			printf("\n");
		}

		timer.Accum_DiffSample(f_stats_time);

		CUberBlockMatrix S2;
#ifdef _DEBUG
		if(!S2.CholeskyOf_FBS<SC2_BlockSizes>(SC2_perm)) {
			fprintf(stderr, "error: SC2_perm is not pos def; unable to continue\n");
			return -1;
		}
		S2.AddTo(S2_sliced, -1);
		double f_error = S2_sliced.f_Norm();
		_ASSERTE(!f_error); // should be exactly zero
#else // _DEBUG
		S2_sliced.Swap(S2); // use the sliced version
#endif // _DEBUG
		// permute and sqrt SC

		timer.Accum_DiffSample(f_SC2_Chol_time);

		printf("getting Schur complement took %.3f sec, out of which:\n", f_schur2_time +
			f_ordering2_time + f_schur2_perm_time + f_SC2_ord_time + f_SC2_Chol_time);
		printf("\t  ord: %.3f\n", f_ordering2_time);
		printf("\t perm: %.3f\n", f_schur2_perm_time);
		printf("\tSchur: %.3f (needs %.2f MB)\n",
			f_schur2_time, SC2.n_Allocation_Size_NoLastPage() / 1048576.0);
		printf("\t  amd: %.3f\n", f_SC2_ord_time);
		printf("\tslice: %.3f\n", f_matslice_time);
		printf("\t Chol: %.3f, " PRIsize " elem. nnz (needs %.2f MB)\n",
			f_SC2_Chol_time, S2.n_NonZero_Num(), S2.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);
		// print some stats

		timer.Accum_DiffSample(f_stats_time);

		{
			const char *p_s_filename_pat = "SC2_Chol";
			const bool b_symmetric = false;

			char p_s_filename[256];
			cs *p_mat = S2.p_BlockStructure_to_Sparse();
			sprintf(p_s_filename, "sc_8.2_%s_SS.tga", p_s_filename_pat);
			if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
			   p_mat, 0, 640, b_symmetric))
				remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			sprintf(p_s_filename, "sc_8.2_%s_SS_AA.tga", p_s_filename_pat);
			if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
			   p_mat, 0, 640/*8192*/, b_symmetric))
				remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			cs_spfree(p_mat);
		}
		// debug - save some matrices in case we get them

		timer.Accum_DiffSample(f_matwrite_time);

		double f_SC2_recformula_time = 0;
		{
			CUberBlockMatrix margs_ordered;
			if(b_use_FBS36) {
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<Lambda_BlockSizes36>(margs_ordered, S2,
					SC2_ord, mpart_Diagonal, false);
			} else {
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<Lambda_BlockSizes37>(margs_ordered, S2,
					SC2_ord, mpart_Diagonal, false);
			}
			// calculate the thing

			timer.Accum_DiffSample(f_SC2_recformula_time);

			/*{
				CUberBlockMatrix empty;
				R.Swap(empty);
			}*/
			// delete R, don't need it and it eats a lot of memory

			margs_ordered.Permute_UpperTriangular_To(lm_margs_recursive, SC2_ord.p_Get_Ordering(),
				SC2_ord.n_Ordering_Size(), false); // no share! the original will be deleted
		}

		double f_SC2_unperm_time = 0;
		timer.Accum_DiffSample(f_SC2_unperm_time);

		printf("recursive formula on Schur complement of D took %.3f, out of which:\n",
			f_SC2_recformula_time + f_SC2_unperm_time);
		printf("\trform: %.3f (recovered " PRIsize " blocks)\n",
			f_SC2_recformula_time, lm_margs_recursive.n_Block_Num());
		printf("\tunprm: %.3f\n", f_SC2_unperm_time);

		size_t n_common_lminv_block_num;
		double f_lminv_error = CMarginals::f_IncompleteDifference(n_common_lminv_block_num, lm_cov_rec, lm_margs_recursive);
		printf("difference of recursive and landmark SC recursive marginals\n"
			"\t  abs: %g\n"
			"\t  rel: %g\n"
			"\tshblk: " PRIsize " blocks (recursive formula of ,\\ recovered " PRIsize ")\n",
			f_lminv_error, f_lminv_error / lm_cov_rec.f_Norm(),
			n_common_lminv_block_num, lm_cov_rec.n_Block_Num()); fflush(stdout);

		timer.Accum_DiffSample(f_stats_time);
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: got bad_alloc while Schur-complementing D\n"); fflush(stdout);
	}
	// calculate SC of D to get marginals of the landmarks

	printf("\ndebug: writing matrix images "); fflush(stdout);
	{
		struct TMatrixInfo {
			const CUberBlockMatrix &r_matrix;
			const char *p_s_filename;
			bool b_symmetric;

			TMatrixInfo(const CUberBlockMatrix &_r_matrix, const char *_p_s_filename, bool _b_symmetric = false)
				:r_matrix(_r_matrix), p_s_filename(_p_s_filename), b_symmetric(_b_symmetric)
			{}
		} p_matrix_list[] = {
			TMatrixInfo(lambda_perm, "lambda", true),
			TMatrixInfo(A, "A", true),
			TMatrixInfo(U, "U"),
			TMatrixInfo(D, "D"),
			TMatrixInfo(SC, "SC", true),
			TMatrixInfo(S, "S"),
			TMatrixInfo(serial_S_bases, "S_bases"),
			TMatrixInfo(margs_recursive, "lam_inv", true),
			TMatrixInfo(SC2, "SC2", true),
			TMatrixInfo(lm_margs_recursive, "SC2_inv", true)
		};
		size_t n_matrix_num = sizeof(p_matrix_list) / sizeof(p_matrix_list[0]);

		for(size_t i = 0; i < n_matrix_num; ++ i) {
			char p_s_filename[256];
			sprintf(p_s_filename, "sc_" PRIsize "_%s.tga", i, p_matrix_list[i].p_s_filename);

			if(p_matrix_list[i].b_symmetric) {
				if(!p_matrix_list[i].r_matrix.Rasterize_Symmetric(p_s_filename, 3))
					remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			} else {
				if(!p_matrix_list[i].r_matrix.Rasterize(p_s_filename, 3))
					remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			}

			cs *p_mat = p_matrix_list[i].r_matrix.p_BlockStructure_to_Sparse();
			sprintf(p_s_filename, "sc_" PRIsize "_%s_SS.tga", i, p_matrix_list[i].p_s_filename);
			if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample(p_s_filename,
			   p_mat, 0, 640, p_matrix_list[i].b_symmetric))
				remove(p_s_filename); // avoid keeping images from previous runs, would get confusing
			sprintf(p_s_filename, "sc_" PRIsize "_%s_SS_AA.tga", i, p_matrix_list[i].p_s_filename);
			if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
			   p_mat, 0, 640/*8192*/, p_matrix_list[i].b_symmetric))
				remove(p_s_filename); // avoid keeping images from previous runs, would get confusing

			/*{
				sprintf(p_s_filename, "sc_%d_%s_SS_rAA.tga", i, p_matrix_list[i].p_s_filename);
				if(!p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
				   p_mat, 0, 640, p_matrix_list[i].b_symmetric))
					remove(p_s_filename); // avoid keeping images from previous runs, would get confusing

				cs *p_prev_mat = p_mat;//cs_spalloc(p_mat->m, p_mat->n, 0, 1, 0);
				//memset(p_prev_mat->p, 0, p_mat->n * sizeof(csi));

				sprintf(p_s_filename, "sc_%d_%s_SS_rpAA.tga", i, p_matrix_list[i].p_s_filename);
				if(!p_prev_mat || !p_mat || !CDebug::Dump_SparseMatrix_Subsample_AA(p_s_filename,
				   p_mat, p_prev_mat, 640, p_matrix_list[i].b_symmetric))
					remove(p_s_filename); // avoid keeping images from previous runs, would get confusing

				//if(p_prev_mat)
				//	cs_spfree(p_prev_mat);
			}*/
			// debug - test the improved AA image capabilities

			if(p_mat)
				cs_spfree(p_mat);
			printf("."); fflush(stdout);
		}
	}
	double f_bitmaps_time = 0;
	timer.Accum_DiffSample(f_bitmaps_time);

	printf(" done (it took %.3f)\n", f_bitmaps_time); fflush(stdout);

	printf("debug: stats time was %.3f\n", f_stats_time);

	// dense tests below ... those will likely die on large mats

	/*
	M = rand(10, 10);
	M = M' * M + 10 * eye(size(M));
	R = chol(M);
	S = inv(R);
	norm_M_minus_RTR = norm(M - R' * R) / norm(M)
	norm_invM_minus_SST = norm(inv(M) - S * S') / norm(inv(M))
	*/
	// note that inverse flips the transposes

	printf("\ncalculating reference inverse of SC only\n"); fflush(stdout);
	try {
		CUberBlockMatrix SST_perm, SST;
		S_bases.PreMultiplyWithSelfTransposeTo(SST_perm);
		SST_perm.PermuteTo(SST, SC_ord.p_Get_Ordering(), SC_ord.n_Ordering_Size(), true, true, true);
		// get SST_perm, SST (single storage)

		printf("note: AMD(SC) is %sidentity ordering\n",
			(CMatrixOrdering::b_IsIdentityOrdering(SC_ord.p_Get_Ordering(),
			SC_ord.n_Ordering_Size()))? "" : "not ");
		// see how this goes

		{
			Eigen::MatrixXd SST_dense;
			S.Convert_to_Dense(SST_dense);
			SST_dense = SST_dense.transpose() * SST_dense;
			// dense rather than sparse inverse

			Eigen::MatrixXd SC_ref;
			SC_perm.Convert_to_Dense(SC_ref);
			SC_ref.triangularView<Eigen::StrictlyLower>() =
				SC_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			// get SC_perm^-1

			double f_error = (SST_dense - SC_ref).norm(), f_norm = SC_ref.norm();
			printf("dense(S)^Tdense(S) - SC_perm = %g (abs; %g rel error)\n", f_error, f_error / f_norm);
		}
		/*{
			Eigen::MatrixXd SST_dense;
			S.Convert_to_Dense(SST_dense);
			SST_dense = SST_dense/* .triangularView<Eigen::Upper>()* /.inverse();
			SST_dense = SST_dense.transpose() * SST_dense;
			// dense rather than sparse inverse

			Eigen::MatrixXd SC_inv_ref;
			SC_perm.Convert_to_Dense(SC_inv_ref);
			SC_inv_ref.triangularView<Eigen::StrictlyLower>() =
				SC_inv_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			SC_inv_ref = SC_inv_ref.inverse();
			// get SC_perm^-1

			double f_error = (SST_dense - SC_inv_ref).norm(), f_norm = SC_inv_ref.norm();
			printf("dense(S)^-Tdense(S)^-1 - SC_perm^-1 = %g (abs; %g rel error)\n", f_error, f_error / f_norm);
		}*/ // S^-TS^-1 is indeed wrong
		{
			Eigen::MatrixXd SST_dense;
			S.Convert_to_Dense(SST_dense);
			SST_dense = SST_dense/*.triangularView<Eigen::Upper>()*/.inverse();
			SST_dense = SST_dense * SST_dense.transpose();
			// dense rather than sparse inverse

			Eigen::MatrixXd SC_inv_ref;
			SC_perm.Convert_to_Dense(SC_inv_ref);
			SC_inv_ref.triangularView<Eigen::StrictlyLower>() =
				SC_inv_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			SC_inv_ref = SC_inv_ref.inverse();
			// get SC_perm^-1

			double f_error = (SST_dense - SC_inv_ref).norm(), f_norm = SC_inv_ref.norm();
			printf("dense(S)^-1dense(S)^-T - SC_perm^-1 = %g (abs; %g rel error)\n", f_error, f_error / f_norm);
		}
		{
			Eigen::MatrixXd SST_dense;
			SST_perm.Convert_to_Dense(SST_dense);

			Eigen::MatrixXd SC_inv_ref;
			SC_perm.Convert_to_Dense(SC_inv_ref);
			SC_inv_ref.triangularView<Eigen::StrictlyLower>() =
				SC_inv_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			SC_inv_ref = SC_inv_ref.inverse();
			// get SC_perm^-1

			double f_error = (SST_dense - SC_inv_ref).norm(), f_norm = SC_inv_ref.norm();
			printf("S^-1S^-T - SC_perm^-1 = %g (abs; %g rel error)\n", f_error, f_error / f_norm);
		}
		{
			Eigen::MatrixXd SST_dense;
			SST.Convert_to_Dense(SST_dense);

			Eigen::MatrixXd SC_inv_ref;
			SC.Convert_to_Dense(SC_inv_ref);
			SC_inv_ref.triangularView<Eigen::StrictlyLower>() =
				SC_inv_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			SC_inv_ref = SC_inv_ref.inverse();
			// get SC^-1

			double f_error = (SST_dense - SC_inv_ref).norm(), f_norm = SC_inv_ref.norm();
			printf("P^-1S^-1S^-TP - SC^-1 = %g (abs; %g rel error)\n", f_error, f_error / f_norm);
		}
		SST_perm.PermuteTo(SST, SC_ord.p_Get_InverseOrdering(), SC_ord.n_Ordering_Size(), true, true, true);
		{
			Eigen::MatrixXd SST_dense;
			SST.Convert_to_Dense(SST_dense);

			Eigen::MatrixXd SC_inv_ref;
			SC.Convert_to_Dense(SC_inv_ref);
			SC_inv_ref.triangularView<Eigen::StrictlyLower>() =
				SC_inv_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			SC_inv_ref = SC_inv_ref.inverse();
			// get SC^-1

			double f_error = (SST_dense - SC_inv_ref).norm(), f_norm = SC_inv_ref.norm();
			printf("PS^-1S^-TP^-1 - SC^-1 = %g (abs; %g rel error; this is the wrong way)\n", f_error, f_error / f_norm);
		}
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: got bad_alloc: no reference will be available\n");
	}

	printf("\ncalculating reference inverse\n"); fflush(stdout);
	Eigen::MatrixXd Sigma_ref;
	try {
		lambda_perm.Convert_to_Dense(Sigma_ref);
		Sigma_ref.triangularView<Eigen::StrictlyLower>() =
			Sigma_ref.triangularView<Eigen::StrictlyUpper>().transpose();
		Sigma_ref = Sigma_ref.inverse();
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: got bad_alloc: no reference will be available\n");
		Sigma_ref.resize(0, 0); // to mark error
	}
	// calculate the reference inverse

	if(Sigma_ref.rows()) {
		try {
			Eigen::MatrixXd SC_inv_ref;
			SC.Convert_to_Dense(SC_inv_ref);
			SC_inv_ref.triangularView<Eigen::StrictlyLower>() =
				SC_inv_ref.triangularView<Eigen::StrictlyUpper>().transpose();
			SC_inv_ref = SC_inv_ref.inverse();
			// invert the schur

			Eigen::MatrixXd SC_inv_dense;
			CMarginals::Calculate_DenseMarginals_Fast_Parallel(SC_inv_dense, S, p_SC_ord, n_SC_ord_size);
			// invert the sqrt of SC

			Eigen::MatrixXd Ud, Dinvd;
			U.Convert_to_Dense(Ud);
			Dinv.Convert_to_Dense(Dinvd);
			Eigen::MatrixXd Ud_Dinvd = Ud * Dinvd;
			Eigen::MatrixXd lm_inv = Dinvd + (Dinvd.transpose() * Ud.transpose()) * SC_inv_ref * (Ud * Dinvd);
			// invert the diagonal part

			Eigen::MatrixXd cross_cov = -SC_inv_ref * Ud * Dinvd;
			_ASSERTE(cross_cov.rows() == SC_inv_ref.rows() && cross_cov.cols() == lm_inv.cols());
			// invert the cross-covariance block

			double f_rec_lm_inv_err = 0;
			double f_splm_inv_serial_err = 0;
			double f_splm_inv_err = 0;//(lm_inv - Sigma_ref.bottomRightCorner(lm_inv.rows(), lm_inv.cols())).norm();
			double f_splm_inv_err1 = 0;
			double f_splm_inv_norm = 0;//Sigma_ref.bottomRightCorner(lm_inv.rows(), lm_inv.cols()).norm();
			for(size_t i = 0, n = sp_lm_inv.n_BlockColumn_Num(); i < n; ++ i) {
				size_t o = sp_lm_inv.n_BlockColumn_Base(i), x = o + SC_inv_ref.cols(); // position of the diag. block in sigma
				size_t w = sp_lm_inv.n_BlockColumn_Column_Num(i); // size of the diag block
				f_splm_inv_serial_err += (sp_lm_inv_serial.t_GetBlock_Log(i, i) - Sigma_ref.block(x, x, w, w)).squaredNorm();
				f_splm_inv_err += (sp_lm_inv.t_GetBlock_Log(i, i) - Sigma_ref.block(x, x, w, w)).squaredNorm();
				f_splm_inv_err1 += (sp_lm_inv.t_GetBlock_Log(i, i) - lm_inv.block(o, o, w, w)).squaredNorm();
				f_splm_inv_norm += Sigma_ref.block(x, x, w, w).squaredNorm();
				f_rec_lm_inv_err += (margs_recursive.t_GetBlock_Log(i + n_SC_ord_size,
					i + n_SC_ord_size) - Sigma_ref.block(x, x, w, w)).squaredNorm();
			}
			f_splm_inv_serial_err = sqrt(f_splm_inv_serial_err);
			f_splm_inv_err = sqrt(f_splm_inv_err);
			f_splm_inv_err1 = sqrt(f_splm_inv_err1);
			f_splm_inv_norm = sqrt(f_splm_inv_norm);
			f_rec_lm_inv_err = sqrt(f_rec_lm_inv_err);
			printf("rec-lm-cov = inv(lambda).blockDiagonal()    with %g error (abs; %g rel error)\n",
				f_rec_lm_inv_err, f_rec_lm_inv_err / f_splm_inv_norm);
			printf("sp-lm-cov = inv(lambda).blockDiagonal()     with %g error (abs; %g rel error)\n",
				f_splm_inv_err, f_splm_inv_err / f_splm_inv_norm);
			printf("sp-lm-cov-ser = inv(lambda).blockDiagonal() with %g error (abs; %g rel error)\n",
				f_splm_inv_serial_err, f_splm_inv_serial_err / f_splm_inv_norm);
			printf("sp-lm-cov = lm-cov.blockDiagonal()          with %g error (abs; %g rel error)\n",
				f_splm_inv_err1, f_splm_inv_err1 / f_splm_inv_norm);

			double f_lm_inv_err = (lm_inv - Sigma_ref.bottomRightCorner(lm_inv.rows(), lm_inv.cols())).norm();
			double f_lm_inv_norm = Sigma_ref.bottomRightCorner(lm_inv.rows(), lm_inv.cols()).norm();
			printf("   lm-cov = inv(lambda).bottomRightCorner() with %g error (abs; %g rel error)\n",
				f_lm_inv_err, f_lm_inv_err / f_lm_inv_norm);

			double f_cc_inv_err = (cross_cov - Sigma_ref.topRightCorner(cross_cov.rows(), cross_cov.cols())).norm();
			double f_cc_inv_norm = Sigma_ref.topRightCorner(cross_cov.rows(), cross_cov.cols()).norm();
			printf("cross-cov = inv(lambda).topRightCorner()    with %g error (abs; %g rel error)\n",
				f_cc_inv_err, f_cc_inv_err / f_cc_inv_norm);

			double f_sc_inv_err = (SC_inv_ref - Sigma_ref.topLeftCorner(SC_inv_ref.rows(), SC_inv_ref.cols())).norm();
			double f_sc_inv_norm = Sigma_ref.topLeftCorner(SC_inv_ref.rows(), SC_inv_ref.cols()).norm();
			printf("  inv(SC) = inv(lambda).topLeftCorner()     with %g error (abs; %g rel error)\n",
				f_sc_inv_err, f_sc_inv_err / f_sc_inv_norm);
			double f_sc_inv_err1 = (SC_inv_ref - SC_inv_dense).norm();
			printf("  inv(SC) = inv(STS)                        with %g error (abs; %g rel error)\n",
				f_sc_inv_err1, f_sc_inv_err1 / f_sc_inv_norm);
		} catch(std::bad_alloc&) {
			fprintf(stderr, "error: got bad_alloc: no reference will be available\n");
		}
	}
	// test the formulas using dense tests

	return 0;
}

/**
 *	@page bamargsexample BA Covariance Recovery Example
 *
 *	This is an implementation for the EUSIPCO 2016 paper, yet to be described.
 *
 *	The example is used with a matrix of a BA system (use SLAM++ command line
 *	switch <tt>-dsm</tt> to dump the system matrix to get one).
 *
 *	Then run with a single argument, a path to the folder containing <tt>system.mtx</tt>.
 *
 *	The additional arguments <tt>-nSD</tt> and <tt>-nSDChol</tt> disable the computation of the Schur
 *	complement of the landmarks or its factorization, respectively (it can take vast amounts of memory).
 *
 *	The optional argument <tt>-tnCo</tt> tests factorization of the input matrix using several
 *	implementations and prints the timing.
 *
 */

/*
 *	end-of-file
 */
