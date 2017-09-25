/*
								+----------------------------------+
								|                                  |
								| *** Schur orderings for SLAM *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|             Main.cpp             |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_schur_orderings/Main.cpp
 *	@brief contains the main() function of the simple example SLAM program
 *	@author -tHE SWINe-
 *	@date 2016-03-25
 */

#include <stdio.h> // printf
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#define __MATRIX_TRANSPOSE_SUM_UNIT_TESTS // want AAT tests
#include "slam/OrderingMagic.h"
#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_CSparse.h"
#include "slam/LinearSolver_CholMod.h"
#include "slam/LinearSolver_Schur.h"
#include "slam/Timer.h"
#include "slam/Debug.h"

#include "sparse_flops/cts.hpp"
#include "sparse_flops/Instrument.h"
#include <map>
#include <numeric>

// todo write a class for A+AT and operations on it (mostly eliminations)

/**
 *	@brief a STL based storage for a sparse compressed column pattern matrix
 */
class CGraphStructure {
protected:
	size_t m_n_m; /**< @brief number of rows */
	size_t m_n_n; /**< @brief number of columns */
	std::vector<csi> m_column_pointers; /**< @brief n + 1 column pointers as cumulative sums */
	std::vector<csi> m_row_indices; /**< @brief row indices for each nonzero entry */

public:
	CGraphStructure(size_t m, size_t n)
		:m_n_m(m), m_n_n(n), m_column_pointers(n, csi(0))
	{}

	CGraphStructure(const cs *p_matrix)
		:m_n_m(p_matrix->m), m_n_n(p_matrix->n), m_column_pointers(p_matrix->p, (p_matrix->p + p_matrix->n + 1)),
		m_row_indices(p_matrix->i, p_matrix->i + p_matrix->p[p_matrix->n])
	{}

	const cs t_SparseView() const
	{
		cs t_sparse;
		t_sparse.m = (csi)m_n_m;
		t_sparse.n = (csi)m_n_n;
		t_sparse.p = (csi*)((m_column_pointers.empty())? (const csi*)0 : &m_column_pointers.front());
		t_sparse.i = (csi*)((m_row_indices.empty())? (const csi*)0 : &m_row_indices.front());
		t_sparse.x = 0;
		t_sparse.nz = -1;
		t_sparse.nzmax = m_row_indices.size(); // not capacity(), dont want writing to it beyond the size of the vector

		return t_sparse;
	}
};

/**
 *	@brief tests the default instantiation of the class by calling every function
 *	@note This is only available if __CSPARSE_TEMPLATE_UNIT_TESTS is defined.
 */
class CTemplateSparseTest_CFLOPCountingDouble {
public:
	typedef CTSparse<std::complex<double>, long, double> cts; /**< @brief the default instantiation type */

	/**
	 *	@brief a dummy callback function for cs_fkeep
	 *
	 *	@param[in] r is row of the element
	 *	@param[in] c is column of the element
	 *	@param[in] f is value of the element
	 *	@param[in] other is pointer to other user-specified data this callback might
	 *		need, equals the value of the third parameter of cs_fkeep()
	 *
	 *	@return Returns nonzero if the nonzero is to be kept, or zero for it to be erased.
	 */
	static cts::_TyInt dummy_fkeep(cts::_TyInt r, cts::_TyInt c, cts::_TyScalar f, void *other)
	{
		return 0;
	}

	/**
	 *	@brief default constructor; instantiates the test function
	 */
	CTemplateSparseTest_CFLOPCountingDouble()
	{
		void (*fun)() = &Test;
		printf("%x\n", (int)(ptrdiff_t)fun);
	}

protected:
	/**
	 *	@brief tries calling all the templated CSparse functions
	 *	@note This will crash if called, no memory is allocated.
	 */
	static void Test()
	{
		cts::cs A;
		cts::css S;
		cts::csn N;
		cts::csd D;
		cts::_TyScalar b, x, y;
		cts::_TyRealScalar beta;
		cts::_TyInt ok, p, c, xi, pinv, q, parent, post, head, next,
			stack, first, maxfirst, prevleaf, ancestor, jleaf, w;

		cts::add(&A, &A, 1.0, 1.0);
		cts::cholsol(0, &A, &b);
		cts::dupl(&A);
		cts::entry(&A, 0, 0, 3.14);
		cts::lusol(0, &A, &b, 1e-3);
		cts::gaxpy(&A, &x, &y);
		cts::multiply(&A, &A);
		cts::qrsol(0, &A, &b);
		cts::transpose(&A, 1);
		cts::compress(&A);
		cts::norm(&A);

		/* utilities */
		cts::calloc(1, 8);
		cts::free(0);
		cts::realloc(0, 1, 10, &ok);
		cts::spalloc(100, 100, 100, 1, 0);
		cts::spfree(&A);
		cts::sprealloc(&A, 1000);
		cts::malloc(1, 8);

		/* --- secondary CSparse routines and data structures ----------------------- */

		cts::amd(0, &A);
		cts::chol(&A, &S);
		cts::dmperm(&A, 12345);
		cts::droptol(&A, 1e-3);
		cts::dropzeros(&A);
		cts::happly(&A, 0, 1.0, &x);
		cts::ipvec(&p, &b, &x, 100);
		cts::lsolve(&A, &x);
		cts::ltsolve(&A, &x);
		cts::lu(&A, &S, 1e-3);
		cts::permute(&A, &pinv, &q, 1);
		cts::pinv(&p, 100);
		cts::pvec(&p, &b, &x, 100);
		cts::qr(&A, &S);
		cts::schol(0, &A);
		cts::sqr(0, &A, 1);
		cts::symperm(&A, &pinv, 1);
		cts::usolve(&A, &x);
		cts::utsolve(&A, &x);
		cts::updown(&A, +1, &A, &parent);

		/* utilities */
		cts::sfree(&S);
		cts::nfree(&N);
		cts::dfree(&D);

		/* --- tertiary CSparse routines -------------------------------------------- */

		cts::counts(&A, &parent, &post, 1);
		cts::cumsum(&p, &c, 100);
		cts::dfs(0, &A, 0, &xi, &p, &pinv);
		cts::etree(&A, 1);
		cts::fkeep(&A, &dummy_fkeep, 0);
		cts::house(&x, &beta, 100);
		cts::maxtrans(&A, 123456);
		cts::post(&parent, 100);
		cts::scc(&A);
		cts::scatter(&A, 0, beta, &w, &x, 0, &A, 1000);
		cts::tdfs(0, 0, &head, &next, &post, &stack);
		cts::leaf(0, 0, &first, &maxfirst, &prevleaf, &ancestor, &jleaf);
		cts::reach(&A, &A, 0, &xi, &pinv);
		cts::spsolve(&A, &A, 0, &xi, &x, &pinv, 0);
		cts::ereach(&A, 0, &parent, &xi, &w);
		cts::randperm(100, 123456);

		/* utilities */
		cts::dalloc(100, 100);
		cts::done(&A, &w, &x, 1);
		cts::idone(&p, &A, &w, 1);
		cts::ndone(&N, &A, &w, &x, 1);
		cts::ddone(&D, &A, &w, 1);
	}
};

typedef CTSparse<CFLOPCountingDouble> CFCSparse;

cs *p_AllocFull(csi m, csi n, double f_value = 1.0)
{
	if(n && m > LONG_MAX / n)
		return 0; // would overflow below
	cs *p_matrix = cs_spalloc(m, n, m * n, 1, 0);
	csi n_off = 0;
	for(csi i = 0; i < n; ++ i) {
		p_matrix->p[i] = n_off;
		for(csi j = 0; j < m; ++ j, ++ n_off) {
			p_matrix->i[n_off] = j;
			p_matrix->x[n_off] = f_value;
		}
	}
	p_matrix->p[n] = n_off;
	return p_matrix;
}

cs *p_AllocBlockArrow(csi n, csi l, csi bs = 3, double f_value = 1.0, double f_diag_value = 100.0)
{
	_ASSERTE(n >= l);
	csi p = n - l;

	csi n_whole_blocks = l / bs;
	csi n_partial_block_size = l - n_whole_blocks * bs;

	_ASSERTE(bs > 0);
	if(bs && (n_whole_blocks > LONG_MAX / bs || n_whole_blocks * bs > LONG_MAX / bs))
		return 0; // would overflow below
	csi n_diag_size = n_whole_blocks * bs * bs;
	if(n_partial_block_size) {
		if(n_partial_block_size > LONG_MAX / n_partial_block_size ||
		   n_diag_size > LONG_MAX - n_partial_block_size * n_partial_block_size)
			return 0; // would overflow below
		n_diag_size += n_partial_block_size * n_partial_block_size;
	}
	// calculate the size of the diagonal section, mind the overflows

	if(p && (l > (LONG_MAX / 2) / p || p > LONG_MAX / p || p * l * 2 > LONG_MAX - p * p))
		return 0; // would overflow below
	csi n_tail_size = p * l * 2 + p * p;
	// calculate the size of the tail, mind the overflows

	if(n_tail_size > LONG_MAX - n_diag_size)
		return 0; // would overflow below

	cs *p_matrix = cs_spalloc(n, n, n_tail_size + n_diag_size, 1, 0);
	csi n_off = 0;
	for(csi i = 0; i < n_whole_blocks; ++ i) {
		for(csi j = 0; j < bs; ++ j) {
			csi c = i * bs + j;
			// current column index

			p_matrix->p[c] = n_off;
			// store column data offset

			for(csi k = 0; k < bs; ++ k, ++ n_off) {
				p_matrix->i[n_off] = i * bs + k; // row
				p_matrix->x[n_off] = (k == j)? f_diag_value : f_value;
			}
			// one column of the diagonal block

			for(csi k = 0; k < p; ++ k, ++ n_off) {
				p_matrix->i[n_off] = l + k; // row
				p_matrix->x[n_off] = f_value;
			}
			// one column of the arrow tail (the part at the bottom of the matrix)
		}
	}
	// the block diagonal part

	if(n_partial_block_size) {
		for(csi j = 0; j < n_partial_block_size; ++ j) {
			csi c = n_whole_blocks * bs + j;
			// current column index

			p_matrix->p[c] = n_off;
			// store column data offset

			for(csi k = 0; k < n_partial_block_size; ++ k, ++ n_off) {
				p_matrix->i[n_off] = c + k; // row
				p_matrix->x[n_off] = (k == j)? f_diag_value : f_value;
			}
			// one column of the diagonal block

			for(csi k = 0; k < p; ++ k, ++ n_off) {
				p_matrix->i[n_off] = l + k; // row
				p_matrix->x[n_off] = f_value;
			}
			// one column of the arrow tail (the part at the bottom of the matrix)
		}
	}
	// the last partial block

	_ASSERTE(n_off == n_diag_size + l * p); // make sure we filled as many entries as expected

	for(csi i = 0; i < p; ++ i) {
		csi c = i + l;
		// current column index

		p_matrix->p[c] = n_off;
		// store column data offset

		for(csi j = 0; j < l; ++ j, ++ n_off) {
			p_matrix->i[n_off] = j; // row
			p_matrix->x[n_off] = f_value;
		}
		for(csi j = 0; j < p; ++ j, ++ n_off) {
			p_matrix->i[n_off] = l + j; // row
			p_matrix->x[n_off] = (i == j)? f_diag_value : f_value;
		}
	}
	// the last p dense columns

	_ASSERTE(n_off == n_tail_size + n_diag_size); // make sure we filled as many entries as expected
	p_matrix->p[n] = n_off;
	return p_matrix;
}

cs *p_AllocLower(csi m, csi n, double f_value = 1.0)
{
	if(n && m > LONG_MAX / n)
		return 0; // would overflow below
	cs *p_matrix = cs_spalloc(m, n, (std::max(m, n) * (std::min(m, n) - 1)) / 2 + std::min(m, n), 1, 0);
	csi n_off = 0;
	for(csi i = 0; i < n; ++ i) {
		p_matrix->p[i] = n_off;
		for(csi j = i; j < m; ++ j, ++ n_off) {
			p_matrix->i[n_off] = j;
			p_matrix->x[n_off] = 1.0;
		}
	}
	p_matrix->p[n] = n_off;
	return p_matrix;
}

size_t n_GEMM_FLOP_Num(const cs *A, const cs *B)
{
	size_t n_before = CFLOPCountingDouble::n_FLOP_Num();

	cs *p_result = CFCSparse::p_ToSparse(CFCSparse::multiply(
		CFCSparse::p_FromSparse(A), CFCSparse::p_FromSparse(B)));
	cs_spfree(p_result);

	return CFLOPCountingDouble::n_FLOP_Num() - n_before;
}

size_t n_GAXPY_FLOP_Num(const cs *A, const double *x, double *y)
{
	size_t n_before = CFLOPCountingDouble::n_FLOP_Num();

	CFCSparse::gaxpy(CFCSparse::p_FromSparse(A),
		(CFLOPCountingDouble*)x, (CFLOPCountingDouble*)y);

	return CFLOPCountingDouble::n_FLOP_Num() - n_before;
}

size_t n_TRSV_FLOP_Num(const cs *L, double *x)
{
	size_t n_before = CFLOPCountingDouble::n_FLOP_Num();

	CFCSparse::lsolve(CFCSparse::p_FromSparse(L), (CFLOPCountingDouble*)x);

	return CFLOPCountingDouble::n_FLOP_Num() - n_before;
}

size_t n_Chol_FLOP_Num(const cs *A, int order = CFCSparse::order_AMD_Chol, bool b_dump_chol = false)
{
	CFCSparse::css *S = CFCSparse::schol(order,
		CFCSparse::p_FromSparse(A)); // calls AMD

	size_t n_before_chol = CFLOPCountingDouble::n_FLOP_Num();

	CFCSparse::csn *N = CFCSparse::chol(CFCSparse::p_FromSparse(A), S);

	size_t n_flops = CFLOPCountingDouble::n_FLOP_Num() - n_before_chol;

	if(b_dump_chol) {
		if(N)
			CDebug::Dump_SparseMatrix_Subsample_AA("chol_Factor.tga", CFCSparse::p_ToSparse(N->L), 0, 1024);
		else
			fprintf(stderr, "error: sparse Cholesky seems to have failed\n");
	} else if(!N)
		fprintf(stderr, "warning: sparse Cholesky seems to have failed\n");

    CFCSparse::sfree(S);
    CFCSparse::nfree(N);

	return n_flops;
}

std::pair<size_t, size_t> t_CholSolve_FLOP_Num(const cs *A,
	const double *b, int order = CFCSparse::order_AMD_Chol, bool b_dump_chol = false)
{
	CFCSparse::css *S = CFCSparse::schol(order,
		CFCSparse::p_FromSparse(A)); // calls AMD

	size_t n_before_chol = CFLOPCountingDouble::n_FLOP_Num();

	CFCSparse::csn *N = CFCSparse::chol(CFCSparse::p_FromSparse(A), S);

	size_t n_after_flops = CFLOPCountingDouble::n_FLOP_Num();

	if(N && N->L) {
		if(b_dump_chol) {
			CDebug::Dump_SparseMatrix_Subsample_AA("chol_Factor.tga", CFCSparse::p_ToSparse(N->L), 0, 1024);
		}
		size_t n = N->L->n;
		std::vector<CFLOPCountingDouble> xv(n);
		CFLOPCountingDouble *x = (xv.empty())? 0 : &xv.front();
		CFCSparse::ipvec(S->pinv, (const CFLOPCountingDouble*)b, x, n);
		CFCSparse::lsolve(N->L, x);
		CFCSparse::ltsolve(N->L, x);
		CFCSparse::pvec(S->pinv, x, (CFLOPCountingDouble*)b, n);
	} else
		fprintf(stderr, "error: sparse Cholesky seems to have failed\n");

	size_t n_solve_flops = CFLOPCountingDouble::n_FLOP_Num();

    CFCSparse::sfree(S);
    CFCSparse::nfree(N);

	return std::make_pair(n_after_flops - n_before_chol, n_solve_flops - n_after_flops);
}

size_t n_LU_FLOP_Num(const cs *A, int order = CFCSparse::order_AMD_LU)
{
	CFCSparse::css *S = CFCSparse::sqr(order,
		CFCSparse::p_FromSparse(A), 0); // calls AMD

	size_t n_before_chol = CFLOPCountingDouble::n_FLOP_Num();

	CFCSparse::csn *N = CFCSparse::lu(CFCSparse::p_FromSparse(A), S, 1e-3);

	size_t n_flops = CFLOPCountingDouble::n_FLOP_Num() - n_before_chol;

    CFCSparse::sfree(S);
    CFCSparse::nfree(N);

	return n_flops;
}

void Test_SchurSparseCost(int n = 300, double f = .99, bool b_no_banner = false)
{
	int l = int(n * f);
	int p = n - l;

	/*cs *A = p_AllocFull(n, n);
	for(int i = 0; i < n; ++ i)
		A->x[i * n + i] += std::max(100.0, double(n) * n);
	// make the diagonal a bit larger in order for the matrix to be positive definite

	for(int i = 0; i < l; ++ i) {
		int ib = i - i % 3; // start of the current block
		for(int j = 0; j < ib; ++ j)
			A->x[i * n + j] = 0;
		for(int j = ib + 3; j < l; ++ j)
			A->x[i * n + j] = 0;
	}
	// make the arrow shape in A

	cs_dropzeros(A);
	// remove the zero entries*/
	cs *A = p_AllocBlockArrow(n, l, 3, 1, std::max(100.0, double(n) * n));
	// alloc A directly in arrow shape (avoids forming a dense matrix)

	if(!b_no_banner)
		CDebug::Dump_SparseMatrix_Subsample_AA("01_A_arrow.tga", A, 0, 1024);

	/*if(!b_no_banner) {
		size_t n_Chol_cost = n_Chol_FLOP_Num(A, CFCSparse::order_Natural, true);
		printf("FLOPs in Chol(A): " PRIsize " (natural order)\n", n_Chol_cost);
	}*/

	/*
	p l cols
	C U p rows
	V D l rows
	*/

	cs *C = p_AllocFull(p, p);
	for(int i = 0; i < p; ++ i)
		C->x[i * p + i] += std::max(100.0, double(n) * n); // make the diagonal a bit larger in order for the matrix to be positive definite
	cs *U = p_AllocFull(p, l);
	cs *V = cs_transpose(U, 1);
	/*cs *D = p_AllocFull(l, l);
	for(int i = 0; i < l; ++ i)
		D->x[i * l + i] += std::max(100.0, double(n) * n); // make the diagonal a bit larger in order for the matrix to be positive definite
	for(int i = 0; i < l; ++ i) {
		int ib = i - i % 3; // start of the current block
		for(int j = 0; j < ib; ++ j)
			D->x[i * l + j] = 0;
		for(int j = ib + 3; j < l; ++ j)
			D->x[i * l + j] = 0;
	}
	cs_dropzeros(D);*/
	cs *D = p_AllocBlockArrow(l, l, 3, 1, std::max(100.0, double(n) * n)); // alloc D directly in arrow shape (avoids forming a dense matrix)
	// just recreate the partial matrices, less work than slicing them

	if(!b_no_banner) {
		CDebug::Dump_SparseMatrix_Subsample_AA("02_C.tga", C, 0, 1024);
		CDebug::Dump_SparseMatrix_Subsample_AA("03_U.tga", U, 0, 1024);
		CDebug::Dump_SparseMatrix_Subsample_AA("04_V.tga", V, 0, 1024);
		CDebug::Dump_SparseMatrix_Subsample_AA("05_D.tga", D, 0, 1024);
	}

	cs *D_inv = cs_transpose(D, 1); // C is symmetric, use this to duplicate the matrix
	for(int i = 0; i < l; i += 3) {
		double *p_block = D_inv->x + D_inv->p[i];
		Eigen::Map<Eigen::Matrix3d, Eigen::DontAlign> block(p_block);
		Eigen::Matrix3d temp = block;
		block = -temp.inverse();
	}
	// calculate inverses of all the blocks

	if(!b_no_banner)
		printf("forming SC:\n");
	size_t n_Dinv_FLOPs = 58 * l / 3; // inverse by Gaus-elim and backsubst (Matlab), inverse by adjugate seems more expensive (about 300 FLOPs) but there could be some repeated ops
	if(!b_no_banner)
		printf("\t               Dinv FLOPs: " PRIsize "\n", n_Dinv_FLOPs);

	size_t n_Dinv_scale_FLOPs = D_inv->p[D_inv->n];
	if(!b_no_banner)
		printf("\t         Dinv scale FLOPs: " PRIsize "\n", n_Dinv_scale_FLOPs);

	CFLOPCountingDouble::Reset_Counters();
	typedef CFCSparse fcs; // shorten

	cs *p_minus_U_Dinv = fcs::p_ToSparse(fcs::multiply(fcs::p_FromSparse(U), fcs::p_FromSparse(D_inv)));
	// U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense

	size_t n_minus_U_Dinv_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	if(!b_no_banner)
		printf("\t            -U*Dinv FLOPs: " PRIsize "\n", n_minus_U_Dinv_FLOPs);
	CFLOPCountingDouble::Reset_Counters();

	cs *p_minus_U_Dinv_V = fcs::p_ToSparse(fcs::multiply(fcs::p_FromSparse(p_minus_U_Dinv), fcs::p_FromSparse(V)));
	// -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

	size_t n_minus_U_Dinv_V_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	if(!b_no_banner)
		printf("\t          -U*Dinv*V FLOPs: " PRIsize "\n", n_minus_U_Dinv_V_FLOPs);
	CFLOPCountingDouble::Reset_Counters();

	cs *p_schur_compl = fcs::p_ToSparse(fcs::add(fcs::p_FromSparse(p_minus_U_Dinv_V),
		fcs::p_FromSparse(C), 1.0, 1.0));
	// -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
	// compute left-hand side A - U(C^-1)V

	size_t n_SC_FLOPs = CFLOPCountingDouble::n_FLOP_Num() -
		CFLOPCountingDouble::n_Multiply_Num(); // don't count the scaling, the block matrices dont do it
	if(!b_no_banner)
		printf("\t      SC=A-U*Dinv*V FLOPs: " PRIsize "\n", n_SC_FLOPs);
	CFLOPCountingDouble::Reset_Counters();

	size_t n_Chol_FLOPs, n_backsubst_FLOPs;// = n_Chol_FLOP_Num(p_schur_compl);
	{
		std::vector<double> b(p_schur_compl->n);
		std::pair<size_t, size_t> cslv = t_CholSolve_FLOP_Num(p_schur_compl, (b.empty())? 0 : &b.front());
		n_Chol_FLOPs = cslv.first;
		n_backsubst_FLOPs = cslv.second;
	}
	if(!b_no_banner)
		printf("\t           Chol(SC) FLOPs: " PRIsize "\n", n_Chol_FLOPs);

	n_SC_FLOPs += n_minus_U_Dinv_V_FLOPs + n_minus_U_Dinv_FLOPs + n_Dinv_scale_FLOPs + n_Dinv_FLOPs + n_Chol_FLOPs;
	if(!b_no_banner)
		printf("\t           total SC FLOPs: " PRIsize "\n", n_SC_FLOPs);

	std::vector<CFLOPCountingDouble> rhs(n * 2, CFLOPCountingDouble(1.0));
	CFLOPCountingDouble::Reset_Counters();

	size_t n_SchurSolve_FLOPs = n_backsubst_FLOPs;

	// [permute rhs]

	size_t n_x_size = C->n, n_l_size = D->n;
	CFLOPCountingDouble *p_x = (rhs.empty())? 0 : &rhs.front(), *p_l = p_x + n_x_size,
		*p_dx = p_l + n_l_size, *p_dl = p_dx + n_x_size;
	// split RHS to vector of x and of l

	_ASSERTE(n_l_size == p_minus_U_Dinv->n); // b matches columns
	_ASSERTE(n_x_size == p_minus_U_Dinv->m); // c matches rows
	fcs::gaxpy(fcs::p_FromSparse(p_minus_U_Dinv), p_l, p_x); // gaxpy(A, b, c) ~ c += A * b
	// x += l * -U(D^-1)

	size_t n_GAXPY1_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	CFLOPCountingDouble::Reset_Counters();

	// now would solve  dx = A - U(C^-1)V \ x but we already counted the cholsolve flops above

	// [flip sign of l]

	_ASSERTE(n_x_size == V->n); // b matches columns
	_ASSERTE(n_l_size == V->m); // c matches rows
	fcs::gaxpy(fcs::p_FromSparse(V), p_dx, p_l); // gaxpy(A, b, c)
	// l -= V * dx

	size_t n_GAXPY2_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	CFLOPCountingDouble::Reset_Counters();

	// [set dl to null vector]

	_ASSERTE(n_l_size == D_inv->n); // b matches columns
	_ASSERTE(n_l_size == D_inv->m); // c matches rows
	fcs::gaxpy(fcs::p_FromSparse(D_inv), p_l, p_dl); // gaxpy(A, b, c) // never mind the aliasing
	// dl = (D^-1)(V * dx - l)

	size_t n_GAXPY3_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	CFLOPCountingDouble::Reset_Counters();

	// [unpermute solution]

	size_t n_SCsolve_FLOPs = n_GAXPY1_FLOPs + n_GAXPY2_FLOPs + n_GAXPY3_FLOPs + n_SchurSolve_FLOPs;
	if(!b_no_banner) {
		printf("solving with SC:\n");
		printf("\t    y + l * -U*Dinv FLOPs: " PRIsize "\n", n_GAXPY1_FLOPs);
		printf("\t         Chol-solve FLOPs: " PRIsize "\n", n_SchurSolve_FLOPs);
		printf("\t         l - V * dx FLOPs: " PRIsize "\n", n_GAXPY2_FLOPs);
		printf("\tDinv * (l - V * dx) FLOPs: " PRIsize "\n", n_GAXPY3_FLOPs);
		/*printf("\t       total SC-fac FLOPs: " PRIsize "\n", n_SC_FLOPs);
		printf("\t     total SC-solve FLOPs: " PRIsize "\n", n_SCsolve_FLOPs);*/
	}
	// report FLOPs spent in Schur-solve

	if(!b_no_banner)
		printf("solving with Chol:\n");
	std::fill(rhs.begin(), rhs.end(), 1.0);
	std::pair<size_t, size_t> cslv = t_CholSolve_FLOP_Num(A, (double*)((rhs.empty())? 0 : &rhs.front()));
	size_t n_FLOPs2 = cslv.first, n_FLOPs2_s = cslv.second; // chol and solve
	if(!b_no_banner) {
		printf("\t            Chol(A) FLOPs: " PRIsize "\n", n_FLOPs2);
		printf("\t          backsubst FLOPs: " PRIsize "\n", n_FLOPs2_s);
	}
	// also count FLOPs in Chol-solve

	//cs_spfree(p_D_inv); // resides in p_D as it is hard to compute
	cs_spfree(p_minus_U_Dinv);
	cs_spfree(p_minus_U_Dinv_V);
	cs_spfree(p_schur_compl);

	if(!b_no_banner)
		printf("\nn,l,p,diagonal percentage,SC FLOPs,SC-sol FLOPs,Chol FLOPs,Chol-sol FLOPs\n");
	printf("%d,%d,%d,%g," PRIsize "," PRIsize "," PRIsize "," PRIsize "\n",
		n, l, p, f * 100, n_SC_FLOPs, n_SCsolve_FLOPs, n_FLOPs2, n_FLOPs2_s);
	fflush(stdout);

	cs_spfree(A);
	cs_spfree(C);
	cs_spfree(U);
	cs_spfree(V);
	cs_spfree(D);
	cs_spfree(D_inv);
}

void Test_SparseOpsCost()
{
	cs *A = p_AllocFull(100, 100);

	printf("counting FLOPs in GEMM of two 100 x 100 matrices\n");
	size_t n_GEMM_cost = n_GEMM_FLOP_Num(A, A);
	size_t n_GEMM_cost_GT = 100 * 100 * (100 * 2 - 1); // the leading addition is saved
	printf("\tground truth FLOPs: " PRIsize "\n", n_GEMM_cost_GT);
	printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_GEMM_cost,
		(n_GEMM_cost == n_GEMM_cost_GT)? "pass" : "FAIL");

	printf("\ncounting FLOPs in GAXPY of a 100 x 100 matrix and a 100 x 1 vector\n");
	double x[100] = {0}, y[100] = {0};
	size_t n_GAXPY_cost = n_GAXPY_FLOP_Num(A, x, y);
	size_t n_GAXPY_cost_GT = 100 * 100 * 2;
	printf("\tground truth FLOPs: " PRIsize "\n", n_GAXPY_cost_GT);
	printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_GAXPY_cost,
		(n_GAXPY_cost == n_GAXPY_cost_GT)? "pass" : "FAIL");

	for(int i = 0; i < 100; ++ i)
		A->x[i * 100 + i] = 10.0;
	// make the diagonal a bit larger in order for the matrix to be positive definite

	printf("\ncounting FLOPs in Cholesky of a 100 x 100 matrix\n");
	size_t n_Chol_cost = n_Chol_FLOP_Num(A, CFCSparse::order_Natural);
	size_t n_Chol_cost_GT = 100 * 100 * 100 / 3 + (100 * (100 - 1)) / 2 + 100; // O(n^3/3 + nnz)
	printf("\tground truth FLOPs: " PRIsize "\n", n_Chol_cost_GT);
	printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_Chol_cost,
		(n_Chol_cost == n_Chol_cost_GT)? "pass" :
		(fabs(double(n_Chol_cost - n_Chol_cost_GT) / n_Chol_cost_GT) < 1e-3)?
		"pass within 0.1 %" : "FAIL"); // up to 0.1% discrepancy allowed

	cs *L = p_AllocLower(100, 100);
	// get a triangular matrix

	printf("\ncounting FLOPs in TRSV of a 100 x 100 lower-triangular matrix and a 100 x 1 vector\n");
	size_t n_TRSV_cost = n_TRSV_FLOP_Num(L, x);
	size_t n_TRSV_cost_GT = 100 * 100 / 2 * 2;
	printf("\tground truth FLOPs: " PRIsize "\n", n_TRSV_cost_GT);
	printf("\trecorded FLOPs: " PRIsize " (%s)\n", n_TRSV_cost,
		(n_TRSV_cost == n_TRSV_cost_GT)? "pass" : "FAIL");

	cs_spfree(A);
	cs_spfree(L);
}

void MIS_Guided(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &r_lambda)
{
	size_t n_pose_vertex_dimension, n_landmark_vertex_dimension;
	std::set<size_t> col_dist;
	for(size_t i = 0, n = r_lambda.n_BlockColumn_Num(); i < n && col_dist.size() < 2; ++ i)
		col_dist.insert(r_lambda.n_BlockColumn_Column_Num(i));
	if(col_dist.size() != 2)
		throw std::runtime_error("invalid matrix for guided ordering");
	n_landmark_vertex_dimension = *col_dist.begin();
	std::set<size_t>::const_iterator p_second_it = col_dist.begin();
	n_pose_vertex_dimension = *(++ p_second_it);
	size_t n_cut = CSchurOrdering::n_Calculate_GuidedOrdering(r_dest, n_pose_vertex_dimension,
		n_landmark_vertex_dimension, r_lambda);
	r_dest.erase(r_dest.begin(), r_dest.begin() + n_cut); // keep only the MIS part
}

void MIS_FF(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &UNUSED(r_lambda))
{
	r_dest = CSchurOrdering::t_MIS_FirstFit(p_lambda, false);
}

void MIS_Imp(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &UNUSED(r_lambda))
{
	r_dest = CSchurOrdering::t_MIS_FirstFit(p_lambda, true, 0);
}

void MIS_Imp2(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &UNUSED(r_lambda))
{
	r_dest = CSchurOrdering::t_MIS_FirstFit(p_lambda, true, size_t(-1));
}

void MIS_Imp2UW(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &r_lambda)
{
	std::vector<size_t> weights(p_lambda->n, size_t(1));
	for(size_t i = 0, n = weights.size(); i < n; ++ i)
		weights[i] = r_lambda.n_BlockColumn_Column_Num(i); // use weights
	r_dest = CSchurOrdering::t_MIS_FirstFit(p_lambda, weights, true, size_t(-1));
}

void MIS_Ex(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &UNUSED(r_lambda))
{
	r_dest = CSchurOrdering::t_MIS(p_lambda); // "perfect" MIS
}

void MIS_ExEx(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &UNUSED(r_lambda))
{
	r_dest = CSchurOrdering::t_MIS_ExStack(p_lambda); // "perfect" MIS
}

void MIS_ExPar(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &UNUSED(r_lambda))
{
	r_dest = CSchurOrdering::t_MIS_Parallel(p_lambda); // "perfect" MIS
}

#ifdef HAVE_IGRAPH

void MIS_igraph(std::vector<size_t> &r_dest, const cs *p_lambda, const CUberBlockMatrix &r_lambda)
{
	std::vector<size_t> weights(p_lambda->n, size_t(1));
	for(size_t i = 0, n = weights.size(); i < n; ++ i)
		weights[i] = r_lambda.n_BlockColumn_Column_Num(i); // use weights
	r_dest = CSchurOrdering::t_MIS_igraph(p_lambda, weights); // "perfect" MIS
}

#endif // HAVE_IGRAPH

/**
 *	@brief a helper object for benchmarking block Cholesky factorization
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CTestCholesky {
public:
	static inline void Do(std::pair<CUberBlockMatrix*, const CUberBlockMatrix*> out_in)
	{
		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		// block sizes for this specialization

		const CUberBlockMatrix &r_lambda_perm = *out_in.second;
		CUberBlockMatrix &r_R = *out_in.first;
		// input, output

		if(!r_R.CholeskyOf_FBS<CBlockSizes>(r_lambda_perm)) {
			fprintf(stderr, "error: not pos def\n");
			_ASSERTE(r_R.b_Empty());
		}
		// perform an FBS operation
	}
};

/**
 *	@brief a helper object for benchmarking block forward and back-substitution
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CTestCholSolve {
public:
	static inline void Do(std::pair<double*, const CUberBlockMatrix*> out_in)
	{
		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		// block sizes for this specialization

		const CUberBlockMatrix &r_R = *out_in.second;
		double *p_x = out_in.first;
		// input, output

		r_R.UpperTriangularTranspose_Solve_FBS<CBlockSizes>(p_x, r_R.n_Column_Num());
		r_R.UpperTriangular_Solve_FBS<CBlockSizes>(p_x, r_R.n_Column_Num());
		// perform an FBS operation
	}
};

typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>)) T2D_PoseSLAM;
typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>, fbs_ut::CCTSize2D<2, 3>,
	fbs_ut::CCTSize2D<3, 2>, fbs_ut::CCTSize2D<2, 2>)) T2D_SLAM;
typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<6, 6>)) T3D_PoseSLAM;
typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<6, 6>, fbs_ut::CCTSize2D<3, 6>,
	fbs_ut::CCTSize2D<6, 3>, fbs_ut::CCTSize2D<3, 3>)) T3D_SLAM_or_BA; // and also BA
typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 6>)) BA_SchurSec; // and also off-diag Schur
typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<7, 7>, fbs_ut::CCTSize2D<3, 7>,
	fbs_ut::CCTSize2D<7, 3>, fbs_ut::CCTSize2D<3, 3>)) TBA7; // sim(3) BA
typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 7>)) BA7_SchurSec; // and also off-diag Schur
typedef MakeTypelist(T2D_PoseSLAM, T3D_PoseSLAM, T2D_SLAM, T3D_SLAM_or_BA, TBA7) TBlockSizes; // indices 3 and 4 are BA (do not change order)
typedef MakeTypelist(BA7_SchurSec, BA_SchurSec, T2D_PoseSLAM, T3D_PoseSLAM,
	T2D_SLAM, T3D_SLAM_or_BA, TBA7) TBlockSizes2;

typedef fbs_ut::CDummyAlgSpecializer::CData<TBlockSizes> CFBSSpecializer;
typedef fbs_ut::CDummyAlgSpecializer::CData<TBlockSizes2> CFBSSpecializer2;
// fixed block size specializer for the common tasks

bool b_try_dense_CPU_solver = true;

/**
 *	@brief a helper object for benchmarking Schur complement formation and solving
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CTestSchur {
public:
	static inline void Do(std::pair<const CUberBlockMatrix*, std::pair<size_t,
		std::pair<const size_t*, std::pair<const char *, bool> > > > t_in)
	{
		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		// block sizes for this specialization

		const CUberBlockMatrix &r_lambda_perm = *t_in.first;
		const size_t n_cut = t_in.second.first, n_size = r_lambda_perm.n_BlockColumn_Num();
		const size_t *p_order_inv = t_in.second.second.first;
		const char *p_s_ordering_name = t_in.second.second.second.first;
		const bool b_skip_perf_test = t_in.second.second.second.second;
		// ugh ... should made a struct for that or use a tuple

		CDeltaTimer dt;
		dt.Reset();

		CUberBlockMatrix A, U, V, D;
		r_lambda_perm.SliceTo(A, 0, n_cut, 0, n_cut, true);
		r_lambda_perm.SliceTo(U, 0, n_cut, n_cut, n_size, true);
		r_lambda_perm.SliceTo(D, n_cut, n_size, n_cut, n_size, true);

		double f_slice_time = dt.f_Time();

		V.TransposeOf(U);

		double f_transpose_time = dt.f_Time();

		CUberBlockMatrix D_inv_storage; // only used if C is not diagonal
		bool b_is_diagonal = D.b_BlockDiagonal();
		CUberBlockMatrix &D_inv = /*(b_is_diagonal)? C :*/ D_inv_storage; // can do diagonal // do *not* reuse storage, will modify lambda!
		if(b_is_diagonal)
			D_inv.InverseOf_BlockDiag_FBS_Parallel<CBlockSizes>(D); // faster version, slightly less general
		else
			D_inv.InverseOf_Symmteric_FBS<CBlockSizes>(D, true); // C is block diagonal (should also be symmetric) // totally need a fast parallel version of this otherwise ordinary schurs win
		// inverse of D

		double f_inverse_time = dt.f_Time();

		{
			D_inv.Rasterize_Symmetric((std::string("D_inv_") + p_s_ordering_name + "5.tga").c_str());
			D_inv.Rasterize_Symmetric((std::string("D_inv_") + p_s_ordering_name + ".tga").c_str(), 3);
			cs *p_dinv_bs;
			if((p_dinv_bs = D_inv.p_BlockStructure_to_Sparse())) {
				CDebug::Dump_SparseMatrix_Subsample((std::string("D_inv_") + p_s_ordering_name +
					"_SS.tga").c_str(), p_dinv_bs, 0, 640, true);
				CDebug::Dump_SparseMatrix_Subsample_AA((std::string("D_inv_") + p_s_ordering_name +
					"_SS_AA.tga").c_str(), p_dinv_bs, 0, 1024, true);
				cs_spfree(p_dinv_bs);
			} else
				fprintf(stderr, "error: failed to get the block matrix structure\n");
			// save the matrix
		}

		double f_inverse_dump_time = dt.f_Time(); // do not time this

		D_inv.Scale_FBS<CBlockSizes>(-1.0);	// -U*(C^-1)

		double f_scale_time = dt.f_Time();

		CUberBlockMatrix minus_U_Dinv;
		minus_U_Dinv.ProductOf_FBS<CBlockSizes, CBlockSizes>(U, D_inv); // U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense

		double f_mul0_time = dt.f_Time();

		CUberBlockMatrix schur_compl; // not needed afterwards
		schur_compl.ProductOf_FBS<CBlockSizes, CBlockSizes>(minus_U_Dinv, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

		double f_mul1_time = dt.f_Time();

		A.AddTo_FBS<CBlockSizes>(schur_compl); // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
		// compute left-hand side A - U(C^-1)V
		// todo - need multiplication with transpose matrices (in this case schur * U^T)

		if(!b_skip_perf_test) {
			double f_add_time = dt.f_Time();

			size_t n_rhs_vector_size = r_lambda_perm.n_Column_Num();
			size_t n_pose_vector_size = A.n_Column_Num(); // 6 * m_n_matrix_cut;
			size_t n_landmark_vector_size = U.n_Column_Num(); // 3 * (n - m_n_matrix_cut);

			std::vector<double> x(n_rhs_vector_size, 1.0);

			std::vector<double> m_double_workspace(n_rhs_vector_size);
			double *p_double_workspace = &m_double_workspace.front();

			double f_alloc_time = dt.f_Time(); // don't count

			r_lambda_perm.InversePermute_RightHandSide_Vector(p_double_workspace,
				&x.front(), n_rhs_vector_size, p_order_inv, n_size);
			// need to permute the vector !!

			Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
			Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_double_workspace +
				n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace

			minus_U_Dinv.PreMultiply_Add_FBS<CBlockSizes>(&v_x(0),
				n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
			// compute right-hand side x - U(C^-1)l

			double f_RHS_time = dt.f_Time();

			Eigen::VectorXd v_x1 = v_x, v_x2 = v_x, v_x3 = v_x;

			double f_RHS_copy_time = dt.f_Time();

			CLinearSolver_UberBlock<CBlockSizes> solver;
			solver.Solve_PosDef(schur_compl, v_x); // todo - get R stats, save R, also time the solution separately! otherwise can't compare
			// solve for dx = A - U(C^-1)V / x

			double f_linsolve0_time = dt.f_Time();

			double f_linsolve0_dense_time = -1000;

			if(b_try_dense_CPU_solver) {
				try {
					CLinearSolver_DenseEigen solver;

					dt.Reset();

					solver.Solve_PosDef(schur_compl, v_x1);

					f_linsolve0_dense_time = dt.f_Time();
				} catch(std::bad_alloc&) {
					fprintf(stderr, "error: dense solver threw std::bad_alloc\n");
				}
			}

			double f_linsolve0_GPU_time = -1000;

#ifndef __DISABLE_GPU

			try {
				CLinearSolver_DenseGPU solver;
				for(int n_pass = 0; n_pass < 2; ++ n_pass) {
					dt.Reset();

					solver.Solve_PosDef(schur_compl, (n_pass)? v_x3 : v_x2);

					f_linsolve0_GPU_time = dt.f_Time();
				}
			} catch(std::bad_alloc&) {
				fprintf(stderr, "error: GPU solver threw std::bad_alloc\n");
			}

#endif // !__DISABLE_GPU

			Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
			Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
			// obtained the first part of the solution

			Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
				n_pose_vector_size, n_landmark_vector_size); // calculated inplace
			v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above

			/*Eigen::VectorXd v_l_copy = v_l;
			V.PreMultiply_Add_FBS<CBlockSizes>(&v_l_copy(0),
				n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx*/
			U.PostMultiply_Add_FBS_Parallel<CBlockSizes>(&v_l(0),
				n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
			//printf("post-mad error %g (rel %g)\n", (v_l_copy - v_l).norm(), (v_l_copy - v_l).norm() / v_l.norm());

			// l = V * dx - l
			v_dl.setZero();
			D_inv.PreMultiply_Add/*_FBS<CBlockSizes>*/(&v_dl(0),
				n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
			// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
			// the second part of the solution is calculated inplace in the dest vector

			r_lambda_perm.Permute_RightHandSide_Vector(&x.front(), p_double_workspace,
				n_rhs_vector_size, p_order_inv, n_size);
			// permute back!

			double f_linsolve1_time = dt.f_Time();

			double f_total_time = /*f_reperm_time +*/ f_slice_time + f_transpose_time +
				f_inverse_time + f_mul0_time + f_scale_time + f_mul1_time +
				f_add_time + f_RHS_time + f_linsolve0_time + f_linsolve1_time;

			printf("\tSchur took %.3f msec (%.3f msec dense or %.3f msec GPU), out of which:\n", f_total_time * 1000,
				(f_total_time - f_linsolve0_time + f_linsolve0_dense_time) * 1000,
				(f_total_time - f_linsolve0_time + f_linsolve0_GPU_time) * 1000);
			//printf("\t   reperm: %.3f\n", f_reperm_time * 1000);
			printf("\t    slice: %.3f\n", f_slice_time * 1000);
			printf("\ttranspose: %.3f\n", f_transpose_time * 1000);
			printf("\t  inverse: %.3f (this %s run in parallel)\n", f_inverse_time * 1000, (b_is_diagonal)? "did" : "can");
			printf("\t multiply: %.3f, out of which:\n", (f_mul0_time + f_scale_time + f_mul1_time) * 1000);
			printf("\t\tdiag gemm: %.3f (this can run in parallel)\n", f_mul0_time * 1000);
			printf("\t\t    scale: %.3f (this can run in parallel)\n", f_scale_time * 1000);
			printf("\t\t     gemm: %.3f (this can run in parallel)\n", f_mul1_time * 1000);
			printf("\t      add: %.3f\n", f_add_time * 1000);
			printf("\t RHS prep: %.3f\n", f_RHS_time * 1000);
			_ASSERTE(schur_compl.b_UpperBlockTriangular()); // only the upper part is stored
			printf("\t  cholsol: %.3f (SC was " PRIsize " x " PRIsize ", " PRIsize " nnz (%.2f %%))\n",
				f_linsolve0_time * 1000, schur_compl.n_Row_Num(), schur_compl.n_Column_Num(),
				schur_compl.n_Symmetric_NonZero_Num(), 100 * float(schur_compl.n_Symmetric_NonZero_Num()) /
				(schur_compl.n_Row_Num() * schur_compl.n_Column_Num()));
			printf("\t\t or dense: %.3f\n", f_linsolve0_dense_time * 1000);
			printf("\t\t   or GPU: %.3f\n", f_linsolve0_GPU_time * 1000);
			printf("\t dy solve: %.3f\n", f_linsolve1_time * 1000);
			// debug - do some profiling
		}

		std::string s_input = std::string("schur_") + p_s_ordering_name;
		int n_result = system((std::string("mkdir ") + s_input).c_str());
		if(!n_result) { // would fail anyway
			schur_compl.Save_MatrixMarket((s_input + "/system.mtx").c_str(),
				(s_input + "/system.bla").c_str());
		}
		// save a matrix to be able to recurse manually

		schur_compl.Rasterize_Symmetric((std::string("schur_compl_") + p_s_ordering_name + "5.tga").c_str());
		schur_compl.Rasterize_Symmetric((std::string("schur_compl_") + p_s_ordering_name + ".tga").c_str(), 3);
		cs *p_sc_bs;
		if((p_sc_bs = schur_compl.p_BlockStructure_to_Sparse())) {
			CDebug::Dump_SparseMatrix_Subsample((std::string("schur_compl_") + p_s_ordering_name +
				"_SS.tga").c_str(), p_sc_bs, 0, 640, true);
			CDebug::Dump_SparseMatrix_Subsample_AA((std::string("schur_compl_") + p_s_ordering_name +
				"_SS_AA.tga").c_str(), p_sc_bs, 0, 1024, true);
			cs_spfree(p_sc_bs);
		} else
			fprintf(stderr, "error: failed to get the block matrix structure\n");
		// save the matrix
	}
};

void Test_SchurComplement(const CUberBlockMatrix &r_lambda_perm,
	size_t n_cut, const size_t *p_order_inv, size_t n_specialization_id,
	const char *p_s_ordering_name, bool b_skip_perf_test)
{
	CFBSSpecializer::Run<CTestSchur>(n_specialization_id,
		std::make_pair(&r_lambda_perm, std::make_pair(n_cut,
		std::make_pair(p_order_inv, std::make_pair(p_s_ordering_name,
		b_skip_perf_test)))));
	// specialize

	if(b_skip_perf_test) {
		printf("skipping more performance tests\n");
		return;
	}

	CDeltaTimer dt;
	dt.Reset();

	CUberBlockMatrix A, U, D;
	const size_t n_size = r_lambda_perm.n_BlockColumn_Num();
	r_lambda_perm.SliceTo(A, 0, n_cut, 0, n_cut, true);
	r_lambda_perm.SliceTo(U, 0, n_cut, n_cut, n_size, true);
	r_lambda_perm.SliceTo(D, n_cut, n_size, n_cut, n_size, true);
	//
	cs *p_A = A.p_Convert_to_Sparse();
	cs *p_U = U.p_Convert_to_Sparse();
	CUberBlockMatrix D_inv;
	D_inv.InverseOf_Symmteric(D, true); // so that it is not upper triangular
	D_inv.Scale(-1.0);
	cs *p_D = D_inv.p_Convert_to_Sparse(); // hack - skip this calculation now
	cs *p_V = cs_transpose(p_U, 1);
	// get the mats

	typedef CFCSparse fcs; // shorten

	size_t n_Dinv_FLOPs_precise = 0;
	{
		std::vector<size_t> vertex_subgraph;
		size_t n_subgraph_num = CMatrixOrdering::n_Find_BlockStructure_Subgraphs(vertex_subgraph, D);
		// find subgraphs in D

		CMatrixOrdering mord;
		std::vector<size_t> first_vertex;
		const size_t *p_ord = mord.p_AdjacentLabel_Ordering(first_vertex, vertex_subgraph, n_subgraph_num);
		// find vertex ordering

		_ASSERTE(mord.b_IsIdentityOrdering(p_ord, mord.n_Ordering_Size()));
		// the blocks should already be ordered adjacently

		std::map<size_t, size_t> clique_size_hist;
		first_vertex.push_back(D.n_BlockColumn_Num()); // add one more to be able to subtract ranges easily
		for(size_t i = 0; i < n_subgraph_num; ++ i) {
			size_t n_clique_size_blocks = first_vertex[i + 1] - first_vertex[i];
			size_t n_clique_size = 0;
			for(size_t j = first_vertex[i], m = first_vertex[i + 1]; j < m; ++ j)
				n_clique_size += D.n_BlockColumn_Column_Num(j);
			n_Dinv_FLOPs_precise += n_clique_size * n_clique_size * n_clique_size; // invert in O(n^3)
			++ clique_size_hist[n_clique_size];
		}
		// subtract the indices of the first vertex of each
		// clique to get clique sizes, count flops to invert them

		{
			size_t n = clique_size_hist.size();
			if(n > 5)
				printf("\t...\n");
			for(std::map<size_t, size_t>::const_iterator p_size_it = clique_size_hist.begin(),
			   p_end_it = clique_size_hist.end(); p_size_it != p_end_it; ++ p_size_it, -- n) {
				if(n <= 5)
					printf("\tcliques of size " PRIsize " elements: " PRIsize "\n", (*p_size_it).first, (*p_size_it).second);
			}
		}
	}

	const cs *p_D_inv = p_D; // hack - skip this calculation now
	size_t n_Dinv_FLOPs = D.n_NonZero_Num(); // todo - this will be bigger
	printf("\tDinv FLOPs: " PRIsize " (imprecise; " PRIsize
		" based on clique sizes)\n", n_Dinv_FLOPs, n_Dinv_FLOPs_precise);

	size_t n_Dinv_scale_FLOPs = D.n_NonZero_Num();
	printf("\tDinv scale FLOPs: " PRIsize "\n", n_Dinv_scale_FLOPs);

	CFLOPCountingDouble::Reset_Counters();

	cs *p_minus_U_Dinv = fcs::p_ToSparse(fcs::multiply(fcs::p_FromSparse(p_U), fcs::p_FromSparse(p_D_inv)));
	// U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense

	size_t n_minus_U_Dinv_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	printf("\t-U*Dinv FLOPs: " PRIsize "\n", n_minus_U_Dinv_FLOPs);
	CFLOPCountingDouble::Reset_Counters();

	cs *p_minus_U_Dinv_V = fcs::p_ToSparse(fcs::multiply(fcs::p_FromSparse(p_minus_U_Dinv), fcs::p_FromSparse(p_V)));
	// -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

	size_t n_minus_U_Dinv_V_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	printf("\t-U*Dinv*V FLOPs: " PRIsize "\n", n_minus_U_Dinv_V_FLOPs);
	CFLOPCountingDouble::Reset_Counters();

	cs *p_schur_compl = fcs::p_ToSparse(fcs::add(fcs::p_FromSparse(p_minus_U_Dinv_V),
		fcs::p_FromSparse(p_A), 1.0, 1.0));
	// -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
	// compute left-hand side A - U(C^-1)V

	size_t n_SC_FLOPs = CFLOPCountingDouble::n_FLOP_Num() -
		CFLOPCountingDouble::n_Multiply_Num(); // don't count the scaling, the block matrices dont do it
	printf("\tSC=A-U*Dinv*V FLOPs: " PRIsize "\n", n_SC_FLOPs);
	CFLOPCountingDouble::Reset_Counters();

	size_t n_Chol_FLOPs, n_backsubst_FLOPs;// = n_Chol_FLOP_Num(p_schur_compl);
	{
		std::vector<double> b(p_schur_compl->n);
		std::pair<size_t, size_t> cslv = t_CholSolve_FLOP_Num(p_schur_compl, (b.empty())? 0 : &b.front());
		n_Chol_FLOPs = cslv.first;
		n_backsubst_FLOPs = cslv.second;
	}
	printf("\tChol(SC) FLOPs: " PRIsize "\n", n_Chol_FLOPs);

	n_SC_FLOPs += n_minus_U_Dinv_V_FLOPs + n_minus_U_Dinv_FLOPs + n_Dinv_scale_FLOPs + n_Dinv_FLOPs + n_Chol_FLOPs;
	printf("\ttotal SC FLOPs: " PRIsize "\n", n_SC_FLOPs);

	std::vector<CFLOPCountingDouble> rhs(r_lambda_perm.n_Column_Num() * 2, CFLOPCountingDouble(1.0));
	CFLOPCountingDouble::Reset_Counters();

	size_t n_SchurSolve_FLOPs = n_backsubst_FLOPs;

	// permute rhs

	size_t n_x_size = p_A->n, n_l_size = p_D->n;
	CFLOPCountingDouble *x = (rhs.empty())? 0 : &rhs.front(), *l = x + n_x_size,
		*dx = l + n_l_size, *dl = dx + n_x_size;
	// split RHS to vector of x and of l

	_ASSERTE(n_l_size == p_minus_U_Dinv->n); // b matches columns
	_ASSERTE(n_x_size == p_minus_U_Dinv->m); // c matches rows
	fcs::gaxpy(fcs::p_FromSparse(p_minus_U_Dinv), l, x); // gaxpy(A, b, c) ~ c += A * b
	// x += l * -U(D^-1)

	size_t n_GAXPY1_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	CFLOPCountingDouble::Reset_Counters();

	// now would solve  dx = A - U(C^-1)V \ x but we already counted the cholsolve flops above

	// [flip sign of l]

	_ASSERTE(n_x_size == p_V->n); // b matches columns
	_ASSERTE(n_l_size == p_V->m); // c matches rows
	fcs::gaxpy(fcs::p_FromSparse(p_V), dx, l); // gaxpy(A, b, c)
	// l -= V * dx

	size_t n_GAXPY2_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	CFLOPCountingDouble::Reset_Counters();

	// [set dl to null vector]

	_ASSERTE(n_l_size == p_D_inv->n); // b matches columns
	_ASSERTE(n_l_size == p_D_inv->m); // c matches rows
	fcs::gaxpy(fcs::p_FromSparse(p_D_inv), l, dl); // gaxpy(A, b, c) // never mind the aliasing
	// dl = (D^-1)(V * dx - l)

	size_t n_GAXPY3_FLOPs = CFLOPCountingDouble::n_FLOP_Num();
	CFLOPCountingDouble::Reset_Counters();

	// unpermute solution

	size_t n_SCsolve_FLOPs = n_GAXPY1_FLOPs + n_GAXPY2_FLOPs + n_GAXPY3_FLOPs + n_SchurSolve_FLOPs;
	printf("\tSchur y + l * -U*Dinv FLOPs: " PRIsize "\n", n_GAXPY1_FLOPs);
	printf("\tSchur Chol-solve FLOPs: " PRIsize "\n", n_SchurSolve_FLOPs);
	printf("\tSchur l - V * dx FLOPs: " PRIsize "\n", n_GAXPY2_FLOPs);
	printf("\tSchur Dinv * (l - V * dx) FLOPs: " PRIsize "\n", n_GAXPY3_FLOPs);
	printf("\ttotal SC-solve FLOPs: " PRIsize "\n", n_SCsolve_FLOPs);

	//cs_spfree(p_D_inv); // resides in p_D as it is hard to compute
	cs_spfree(p_minus_U_Dinv);
	cs_spfree(p_minus_U_Dinv_V);
	cs_spfree(p_schur_compl);
	cs_spfree(p_A);
	cs_spfree(p_U);
	cs_spfree(p_D);
	cs_spfree(p_V);
}

template <class T>
bool DrawLinesInMatImage(const char *p_s_out_filename, const char *p_s_in_filename,
	uint32_t n_color, float f_line_width, std::vector<T> &r_line_list, double f_rel_scale) // a design failure, this edits an existing images and writes it back
{
	if(!p_s_out_filename)
		p_s_out_filename = p_s_in_filename;
	if(!p_s_in_filename)
		p_s_in_filename = p_s_out_filename;
	TBmp *p_bmp;
	if(!(p_bmp = CTgaCodec::p_Load_TGA(p_s_in_filename)))
		return false;
	n_color = TBmp::n_RGB_to_BGR(n_color); // working in BGRA

	for(size_t i = 0, n = r_line_list.size(); i < n; ++ i) {
		float f = float(r_line_list[i] * f_rel_scale * p_bmp->n_width);
		p_bmp->DrawLine_AA(f, .0f, f, float(p_bmp->n_height), n_color, f_line_width);
		float g = float(r_line_list[i] * f_rel_scale * p_bmp->n_height);
		p_bmp->DrawLine_AA(.0f, g, float(p_bmp->n_width), g, n_color, f_line_width);
	}

	bool b_result = CTgaCodec::Save_TGA(p_s_out_filename, *p_bmp, true);

	p_bmp->Delete();

	return b_result;
}

/**
 *	@brief a helper object for benchmarking block Cholesky solving via \ref CLinearSolver_UberBlock
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CTestBlockSolve {
public:
	static void Do(std::pair<const CUberBlockMatrix*, Eigen::VectorXd*> t_in_out)
	{
		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		CLinearSolver_UberBlock<CBlockSizes>().Solve_PosDef(*t_in_out.first, *t_in_out.second);
	}
};

/**
 *	@brief a helper object for benchmarking block Cholesky factorization via \ref CLinearSolver_UberBlock
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CTestBlockFactorize {
public:
	static void Do(std::pair<CUberBlockMatrix*, const CUberBlockMatrix*> t_in_out)
	{
		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		std::vector<size_t> workspace; // g++ requires a temporary
		CLinearSolver_UberBlock<CBlockSizes>().Factorize_PosDef_Blocky(*t_in_out.first,
			*t_in_out.second, workspace);
	}
};

/**
 *	@brief Schur complement class
 */
struct TSchur {
	CUberBlockMatrix SC, minus_U_Dinv, U, minus_Dinv; // U is a reference to the lower level
	TSchur *p_next_in_cascade;

	void Composite_LambdaSC(CUberBlockMatrix &r_lambda_perm)
	{
		if(p_next_in_cascade)
			p_next_in_cascade->Composite_LambdaSC(r_lambda_perm);

		size_t n_cut = SC.n_BlockColumn_Num(), n_landmark_num = U.n_BlockColumn_Num();
		if(!p_next_in_cascade)
			SC.SliceTo(r_lambda_perm, 0, n_cut, 0, n_cut, true); // do ref

		r_lambda_perm.ExtendTo(SC.n_Column_Num() + U.n_Column_Num(),
			SC.n_Column_Num() + U.n_Column_Num());
		{
			CUberBlockMatrix U_ext;
			U.SliceTo(U_ext, 0, n_cut, 0, n_landmark_num, true); // do ref
			U_ext.ExtendTopLeftTo(U_ext.n_Row_Num(), r_lambda_perm.n_Column_Num());
			U_ext.ExtendTo(r_lambda_perm.n_Column_Num(), r_lambda_perm.n_Column_Num());
			U_ext.AddTo(r_lambda_perm);
		}
		{
			CUberBlockMatrix D_ext;
			minus_Dinv.SliceTo(D_ext, 0, n_landmark_num, 0, n_landmark_num, true); // do ref
			D_ext.ExtendTopLeftTo(r_lambda_perm.n_Column_Num(), r_lambda_perm.n_Column_Num());
			D_ext.AddTo(r_lambda_perm);
		}
	}

	void From_OrderedMatrix(const CUberBlockMatrix &r_lambda_perm, size_t n_cut,
		CUberBlockMatrix *p_store_D = 0)
	{
		_ASSERTE(r_lambda_perm.b_SymmetricLayout()); // and probably also upper triangular

		CUberBlockMatrix A, V, D;
		const size_t n_size = r_lambda_perm.n_BlockColumn_Num();
		r_lambda_perm.SliceTo(A, 0, n_cut, 0, n_cut, true);
		r_lambda_perm.SliceTo(U, 0, n_cut, n_cut, n_size, true);
		r_lambda_perm.SliceTo(D, n_cut, n_size, n_cut, n_size, true);
		V.TransposeOf(U);

		minus_Dinv.InverseOf_Symmteric(D, true);
		minus_Dinv.Scale(-1.0);	// -U*(C^-1)
		// inverse of D

		if(p_store_D)
			p_store_D->Swap(D);
		// get uninverted D

		minus_U_Dinv.ProductOf(U, minus_Dinv); // U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense

		SC.ProductOf(minus_U_Dinv, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

		A.AddTo(SC); // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
		// compute left-hand side A - U(C^-1)V
	}

	template <const int n_sizes_index, class CBlockSizesForest>
	class CFrom_OrderedMatrix_FBS {
	public:
		static inline void Do(std::pair<std::pair<const CUberBlockMatrix*,
			std::pair<size_t, CUberBlockMatrix*> >, TSchur*> t_in)
		{
			typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
			t_in.second->From_OrderedMatrix_FBS<CBlockSizes>(*t_in.first.first,
				t_in.first.second.first, t_in.first.second.second);
		}
	};

	template <class CBlockSizes>
	void From_OrderedMatrix_FBS(const CUberBlockMatrix &r_lambda_perm, size_t n_cut,
		CUberBlockMatrix *p_store_D = 0)
	{
		_ASSERTE(r_lambda_perm.b_SymmetricLayout()); // and probably also upper triangular

		CUberBlockMatrix A, V, D;
		const size_t n_size = r_lambda_perm.n_BlockColumn_Num();
		r_lambda_perm.SliceTo(A, 0, n_cut, 0, n_cut, true);
		r_lambda_perm.SliceTo(U, 0, n_cut, n_cut, n_size, true);
		r_lambda_perm.SliceTo(D, n_cut, n_size, n_cut, n_size, true);
		V.TransposeOf(U);

		minus_Dinv.InverseOf_Symmteric_FBS<CBlockSizes>(D, true);
		minus_Dinv.Scale_FBS<CBlockSizes>(-1.0);	// -U*(C^-1)
		// inverse of D

		if(p_store_D)
			p_store_D->Swap(D);
		// get uninverted D

		minus_U_Dinv.ProductOf_FBS<CBlockSizes, CBlockSizes>(U, minus_Dinv); // U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense

		SC.ProductOf_FBS<CBlockSizes, CBlockSizes>(minus_U_Dinv, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

		A.AddTo_FBS<CBlockSizes>(SC); // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
		// compute left-hand side A - U(C^-1)V
	}

	template <const int n_sizes_index, class CBlockSizesForest>
	class CSolve_FBS {
	public:
		static inline void Do(std::pair<std::pair<double*, std::pair<size_t, int> >, TSchur*> t_in)
		{
			typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
			t_in.second->Solve_FBS<CBlockSizes>(t_in.first.first, t_in.first.second.first, t_in.first.second.second);
		}
	};

	// n_solver_type is 0 for block, 1 for cholmod, 2 for dense, 3 for gpu dense
	template <class CBlockSizes>
	void Solve_FBS(double *p_x, size_t UNUSED(n_x_size), int n_solver_type) const // must invpermute x before and permute after
	{
		size_t n_pose_vector_size = SC.n_Column_Num(); // 6 * m_n_matrix_cut;
		size_t n_landmark_vector_size = minus_U_Dinv.n_Column_Num(); // 3 * (n - m_n_matrix_cut);
		size_t n_rhs_vector_size = n_pose_vector_size + n_landmark_vector_size;

		_ASSERTE(n_x_size == n_rhs_vector_size);

		Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_x, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
		Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_x +
			n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace

		minus_U_Dinv.PreMultiply_Add_FBS<CBlockSizes>(&v_x(0),
			n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
		// compute right-hand side x - U(C^-1)l

		//_ASSERTE(!p_next_in_cascade || n_solver_type == 4); // either there is no cascaded solver or the solver type is 4
		//_ASSERTE(p_next_in_cascade || n_solver_type != 4); // either there is a cascaded solver or the solver type is not 4
		_ASSERTE(n_solver_type >= 0 && n_solver_type < 4);
		if(p_next_in_cascade)
			p_next_in_cascade->Solve(&v_x(0), n_pose_vector_size, n_solver_type);
		else if(!n_solver_type) {
			CLinearSolver_UberBlock<CBlockSizes> solver;
			solver.Solve_PosDef(SC, v_x);
		} else if(n_solver_type == 1) {
			CLinearSolver_CholMod solver;
			solver.Solve_PosDef(SC, v_x);
		} else if(n_solver_type == 2) {
			CLinearSolver_DenseEigen solver;
			solver.Solve_PosDef(SC, v_x);
		} else {
#ifndef __DISABLE_GPU
			CLinearSolver_DenseGPU solver;
			solver.Solve_PosDef(SC, v_x);
#else // !__DISABLE_GPU
			fprintf(stderr, "error: GPU solver is disabled\n");
#endif // !__DISABLE_GPU
		}
		// solve for dx = A - U(C^-1)V / x

		Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
		Eigen::Map<Eigen::VectorXd>(p_x, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
		// obtained the first part of the solution

		Eigen::Map<Eigen::VectorXd> v_dl(p_x +
			n_pose_vector_size, n_landmark_vector_size); // calculated inplace
		v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above

		/*Eigen::VectorXd v_l_copy = v_l;
		V.PreMultiply_Add_FBS<CBlockSizes>(&v_l_copy(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx*/
		U.PostMultiply_Add_FBS/*_Parallel*/<CBlockSizes>(&v_l(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
		//printf("post-mad error %g (rel %g)\n", (v_l_copy - v_l).norm(), (v_l_copy - v_l).norm() / v_l.norm());

		// l = V * dx - l
		v_dl.setZero();
		minus_Dinv.PreMultiply_Add_FBS<CBlockSizes>(&v_dl(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
		// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
		// the second part of the solution is calculated inplace in the dest vector
	}

	// n_solver_type is 0 for block, 1 for cholmod, 2 for dense, 3 for gpu dense
	void Solve(double *p_x, size_t UNUSED(n_x_size), int n_solver_type) const // must invpermute x before and permute after
	{
		size_t n_pose_vector_size = SC.n_Column_Num(); // 6 * m_n_matrix_cut;
		size_t n_landmark_vector_size = minus_U_Dinv.n_Column_Num(); // 3 * (n - m_n_matrix_cut);
		size_t n_rhs_vector_size = n_pose_vector_size + n_landmark_vector_size;

		_ASSERTE(n_x_size == n_rhs_vector_size);

		Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_x, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
		Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_x +
			n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace

		minus_U_Dinv.PreMultiply_Add(&v_x(0),
			n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
		// compute right-hand side x - U(C^-1)l

		//_ASSERTE(!p_next_in_cascade || n_solver_type == 4); // either there is no cascaded solver or the solver type is 4
		//_ASSERTE(p_next_in_cascade || n_solver_type != 4); // either there is a cascaded solver or the solver type is not 4
		_ASSERTE(n_solver_type >= 0 && n_solver_type < 4);
		if(p_next_in_cascade)
			p_next_in_cascade->Solve(&v_x(0), n_pose_vector_size, n_solver_type);
		else if(!n_solver_type) {
			//CLinearSolver_UberBlock<CBlockSizes> solver;
			CLinearSolver_CSparse solver; // if block size not known, fall back to CSparse
			solver.Solve_PosDef(SC, v_x);
		} else if(n_solver_type == 1) {
			CLinearSolver_CholMod solver;
			solver.Solve_PosDef(SC, v_x);
		} else if(n_solver_type == 2) {
			CLinearSolver_DenseEigen solver;
			solver.Solve_PosDef(SC, v_x);
		} else {
#ifndef __DISABLE_GPU
			CLinearSolver_DenseGPU solver;
			solver.Solve_PosDef(SC, v_x);
#else // !__DISABLE_GPU
			fprintf(stderr, "error: GPU solver is disabled\n");
#endif // !__DISABLE_GPU
		}
		// solve for dx = A - U(C^-1)V / x

		Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
		Eigen::Map<Eigen::VectorXd>(p_x, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
		// obtained the first part of the solution

		Eigen::Map<Eigen::VectorXd> v_dl(p_x +
			n_pose_vector_size, n_landmark_vector_size); // calculated inplace
		v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above

		/*Eigen::VectorXd v_l_copy = v_l;
		V.PreMultiply_Add_FBS<CBlockSizes>(&v_l_copy(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx*/
		U.PostMultiply_Add/*_FBS_Parallel<CBlockSizes>*/(&v_l(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
		//printf("post-mad error %g (rel %g)\n", (v_l_copy - v_l).norm(), (v_l_copy - v_l).norm() / v_l.norm());

		// l = V * dx - l
		v_dl.setZero();
		minus_Dinv.PreMultiply_Add/*_FBS<CBlockSizes>*/(&v_dl(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
		// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
		// the second part of the solution is calculated inplace in the dest vector
	}
};

/**
 *	@brief a helper object for inverting sparse (thick) block diagonal matrix
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CInvertBlockDiagonals {
public:
	static inline void Do(std::vector<CUberBlockMatrix> *p_D_list)
	{
		std::vector<CUberBlockMatrix> &r_D_list = *p_D_list;
		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		for(size_t i = 0, n = r_D_list.size(); i < n; ++ i) {
			CUberBlockMatrix T;
			T.Swap(r_D_list[i]);
			r_D_list[i].InverseOf_Symmteric_FBS<CBlockSizes>(T, true);
		}
		// invert the matrices in the list to get more stats
	}
};

/**
 *	@brief a helper object for inverting sparse (thick) block diagonal matrix,
 *		in parallel and possibly with GPU and CPU cooperating on the individual blocks
 *
 *	@tparam n_sizes_index is zero-based index of block sizes
 *	@tparam CBlockSizesForest is list of block size list (n_sizes_index chooses the list to use)
 */
template <const int n_sizes_index, class CBlockSizesForest>
class CInvertBlockDiagonals_Parallel {
protected:
	class CBlockInverse {
	protected:
		CUberBlockMatrix::_TyMatrixXdRef &m_r_block;

	public:
		CBlockInverse(CUberBlockMatrix::_TyMatrixXdRef &r_block)
			:m_r_block(r_block)
		{}

		template <class CBlockSize>
		inline void operator ()()
		{
			enum {
				n_block_size = CBlockSize::n_size
			};

			_ASSERTE(m_r_block.rows() == m_r_block.cols() && m_r_block.cols() == n_block_size);
			Eigen::Map<Eigen::Matrix<double, n_block_size, n_block_size>,
				CUberBlockMatrix::map_Alignment> block(m_r_block.data());
			block = block.inverse();
		}
	};

public:
	static inline void Do(std::pair<std::vector<CUberBlockMatrix>*, bool> t_in_out)
	{
		std::vector<CUberBlockMatrix> &r_D_list = *t_in_out.first;
		const bool b_use_GPU = t_in_out.second;

		const bool b_immediate_large_inverse = b_use_GPU /*true*/; // use GPU from multiple threads?
		const bool b_use_GPU_CholInv = false; // use Chol or LU for inverse on GPU? // no, it is only semidefinite mostly

		const bool b_is_upper = true;
		const bool b_transpose_result_to_lower = true;
		const int n_GPU_thresh = 192;
		// empirically determined, CULA seems to be very slightly faster for
		// inverting a 192x192 double matrix (about 3.5 msec), on Tesla K20m

		/*CDeltaTimer dt;
		double f_block_find_time = 0;
		double f_inv_time = 0;
		double f_symmetrize_time = 0;*/ // profiling

		typedef typename CTypelistItemAt<CBlockSizesForest, n_sizes_index>::_TyResult CBlockSizes;
		for(size_t i = 0, n = r_D_list.size(); i < n; ++ i) {
			CUberBlockMatrix &T = r_D_list[i];

			/*std::vector<size_t> vertex_subgraph;
			size_t n_block_num = CMatrixOrdering::n_Find_BlockStructure_Subgraphs(vertex_subgraph, T);
			_ASSERTE(CDebug::b_IsWeaklySortedSet(vertex_subgraph.begin(), vertex_subgraph.end())); // make sure it is already ordered (block block diagonal)
			_ASSERTE(vertex_subgraph.empty() || !vertex_subgraph.front());*/ // the first subgraph is zero
			// t_odo - do a simpler analysis to find the blocks, do everything in thread 0 and just pass the begins/ends to the loop below

			_ASSERTE(T.b_SymmetricLayout()); // must be symmetric

			std::vector<std::pair<size_t, size_t> > block_list;
			{
				block_list.reserve(T.n_BlockColumn_Num() / 2); // or T.n_BlockColumn_Num(), to prevent reallocating too much
				size_t n_first = 0, n = T.n_BlockColumn_Num();
				for(size_t i = 0, n_prev_last_row = 0; i < n; ++ i) {
					size_t n_block_num = T.n_BlockColumn_Block_Num(i);
					if(!n_block_num) {
						if(i != n_first)
							block_list.push_back(std::make_pair(n_first, i));
						n_first = i + 1;
					} else if(T.n_Block_Row(i, 0) > n_prev_last_row) { // the first element of 
						if(i != n_first)
							block_list.push_back(std::make_pair(n_first, i));
						n_first = i;
						n_prev_last_row = T.n_Block_Row(i, n_block_num - 1);
					} else
						n_prev_last_row = T.n_Block_Row(i, n_block_num - 1);
				}
				if(n_first < n) {
					block_list.push_back(std::make_pair(n_first, n));
					// the last block, all the way to the end
				}
			}
			size_t n_block_num = block_list.size();
			// discover blocks in O(n), the integrity of the blocks will be checked below
			// note that the list can also be supplied to this function

#ifdef _DEBUG
			std::vector<size_t> vertex_subgraph;
			size_t n_subgraph_num = CMatrixOrdering::n_Find_BlockStructure_Subgraphs(vertex_subgraph, T);
			_ASSERTE(CDebug::b_IsWeaklySortedSet(vertex_subgraph.begin(), vertex_subgraph.end())); // make sure it is already ordered (block block diagonal)
			_ASSERTE(vertex_subgraph.empty() || !vertex_subgraph.front());
			_ASSERTE(n_block_num == n_subgraph_num);
			if(n_block_num != n_subgraph_num) {
				fprintf(stderr, "error: foo size mismatch %d %d\n", int(n_block_num), int(n_subgraph_num));
				block_list.resize(n_block_num = n_subgraph_num); // fix it
			}
			for(size_t n_block_id = 0, n_end_column = 0; n_block_id < n_subgraph_num; ++ n_block_id) {
				std::vector<size_t>::const_iterator p_start_it = std::lower_bound(vertex_subgraph.begin() +
					n_end_column, vertex_subgraph.end(), size_t(n_block_id));
				size_t n_start_column = p_start_it - vertex_subgraph.begin();
				_ASSERTE(*p_start_it == n_block_id);
				_ASSERTE(n_start_column >= n_block_id); // there might be more columns taking one block so start might get grater but never lower
				// find start column

				std::vector<size_t>::const_iterator p_end_it = std::lower_bound(p_start_it,
					const_cast<const std::vector<size_t>&>(vertex_subgraph).end(), size_t(n_block_id) + 1);
				n_end_column = p_end_it - vertex_subgraph.begin();
				_ASSERTE(p_end_it == vertex_subgraph.end() || *p_end_it == size_t(n_block_id) + 1);
				// find end column

				_ASSERTE(block_list[n_block_id].first == n_start_column);
				_ASSERTE(block_list[n_block_id].second == n_end_column);
				if(block_list[n_block_id].first != n_start_column ||
				   block_list[n_block_id].second != n_end_column) {
					fprintf(stderr, "error: foo %d %d %d %d\n", int(block_list[n_block_id].first),
						int(n_start_column), int(block_list[n_block_id].second), int(n_end_column));
					block_list[n_block_id].first = n_start_column;
					block_list[n_block_id].second = n_end_column; // fix it
				}
			}
#endif // _DEBUG

			//f_block_find_time += dt.f_Time(); // profiling

			_ASSERTE(n_block_num <= INT_MAX);
			const int _n_block_num = int(n_block_num);

			bool b_run_in_parallel = (b_use_GPU || n_block_num > 50); // only parallelize if it is worth it

#ifdef _OPENMP
			const size_t n_thread_num = (b_run_in_parallel)? omp_get_max_threads() : 1;
#else // _OPENMP
			const size_t n_thread_num = 1;
#endif // _OPENMP
#ifndef __DISABLE_GPU
			std::vector<CLinearSolver_DenseGPU> GPU_solvers((b_use_GPU)? n_thread_num : 0); // t_odo - put these inside a vector, can share the instances, can avoid CUDA init on a CPU
#endif // __DISABLE_GPU

			bool b_had_large_mats = false; // t_odo - make this into several std::vectors, one per thread (containing first/last col of large blocks, to be processed later)
			std::vector<std::vector<size_t> > large_block_lists((b_immediate_large_inverse)? 0 : n_thread_num);

			#pragma omp parallel if(b_run_in_parallel) // only parallelize if it is worth it
			{
#ifdef _OPENMP
				_ASSERTE(omp_get_num_threads() == n_thread_num); // must match
				const size_t n_tid = (b_use_GPU || !b_immediate_large_inverse)? omp_get_thread_num() : -1;
#else // _OPENMP
				const size_t n_tid = 0;
#endif // _OPENMP

#ifndef __DISABLE_GPU
				COMPCUDAContextGuard context_guard(b_use_GPU);

				CLinearSolver_DenseGPU *p_GPU_solver = (b_use_GPU)?
					&GPU_solvers[n_tid] : 0; // t_odo - put these inside a vector, can share the instances, can avoid CUDA init on a CPU
#endif // __DISABLE_GPU
				// make an instance of the GPU solver, one per thread

				Eigen::MatrixXd inv_storage; // thread-private storage

				//size_t n_end_column = 0;

				#pragma omp for nowait schedule(dynamic, 1)
				for(int n_block_id = 0; n_block_id < _n_block_num; ++ n_block_id) {
					/*std::vector<size_t>::const_iterator p_start_it = std::lower_bound(vertex_subgraph.begin() +
						n_end_column, vertex_subgraph.end(), size_t(n_block_id));
					size_t n_start_column = p_start_it - vertex_subgraph.begin();
					_ASSERTE(*p_start_it == n_block_id);
					_ASSERTE(n_start_column >= n_block_id); // there might be more columns taking one block so start might get grater but never lower
					// find start column

					std::vector<size_t>::const_iterator p_end_it = std::lower_bound(p_start_it,
						const_cast<const std::vector<size_t>&>(vertex_subgraph).end(), size_t(n_block_id) + 1);
					n_end_column = p_end_it - vertex_subgraph.begin();
					_ASSERTE(p_end_it == vertex_subgraph.end() || *p_end_it == size_t(n_block_id) + 1);*/
					// find end column

					const size_t n_start_column = block_list[n_block_id].first;
					const size_t n_end_column = block_list[n_block_id].second;
					// get start / end columns

					if(n_end_column == n_start_column + 1) {
						_ASSERTE(T.n_BlockColumn_Block_Num(n_start_column) == 1 &&
							T.n_Block_Row(n_start_column, 0) == n_start_column); // make sure there is a single diag. block
						CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(n_start_column, 0);
						// just a single block

						if(t_block.cols() < n_GPU_thresh)
							fbs_ut::CWrap3<>::template In_ColumnWidth_DecisionTree<CBlockSizes>(t_block.cols(), CBlockInverse(t_block));
						else {
							if(b_immediate_large_inverse) {
#ifndef __DISABLE_GPU
								if(b_use_GPU)
									p_GPU_solver->Dense_Inverse(t_block, t_block, false, b_use_GPU_CholInv);
								else
#endif // __DISABLE_GPU
									CLinearSolver_DenseEigen::Dense_Inverse(t_block, t_block, false); // fallback to CPU
							} else {
								large_block_lists[n_tid].push_back(n_block_id);
								b_had_large_mats = true;//CLinearSolver_DenseEigen::Dense_Inverse_Parallel(t_block, t_block, false);
							}
						}
						// invert it
					} else {
						size_t n_col_num = 0;
						for(size_t i = n_start_column; i < n_end_column; ++ i) {
							_ASSERTE(b_is_upper || T.n_BlockColumn_Block_Num(i) == n_end_column - n_start_column);
							_ASSERTE(b_is_upper || (T.n_Block_Row(i, 0) == n_start_column &&
								T.n_Block_Row(i, T.n_BlockColumn_Block_Num(i) - 1) == n_end_column));
							// make sure each column contains a completely filled dense block
							// (not so important at source but important at destination)

							_ASSERTE(!b_is_upper || T.n_BlockColumn_Block_Num(i) == i + 1 - n_start_column);
							_ASSERTE(!b_is_upper || (T.n_Block_Row(i, 0) == n_start_column && // starts at start column
								T.n_Block_Row(i, T.n_BlockColumn_Block_Num(i) - 1) == i)); // ends at the diagonal
							// for upper matrices which are missing the lower blocks

							n_col_num += T.n_BlockColumn_Column_Num(i);
						}
						// sum column widths

						if(n_col_num >= n_GPU_thresh && !b_immediate_large_inverse) {
							large_block_lists[n_tid].push_back(n_block_id);
							b_had_large_mats = true;
							continue;
						}
						// don't do anything now

						inv_storage.resize(n_col_num, n_col_num);
						// resize temp storage

						if(b_is_upper) {
							size_t n_col_base = 0;
							for(size_t i = n_start_column, m = 1; i < n_end_column; ++ i, ++ m) {
								size_t n_row_base = 0;
								for(size_t j = 0; j < m; ++ j) {
									CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
									inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols()) = t_block;
									n_row_base += t_block.rows();
								}
								n_col_base += T.n_BlockColumn_Column_Num(i);
							}
						} else {
							size_t n_col_base = 0;
							for(size_t i = n_start_column, m = n_end_column - n_start_column; i < n_end_column; ++ i) {
								size_t n_row_base = 0;
								for(size_t j = 0; j < m; ++ j) {
									CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
									inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols()) = t_block;
									n_row_base += t_block.rows();
								}
								n_col_base += T.n_BlockColumn_Column_Num(i);
							}
						}
						// copy blocks to the dense block

						if(n_col_num < n_GPU_thresh) {
							if(b_is_upper) {
								inv_storage.triangularView<Eigen::StrictlyLower>() =
									inv_storage.triangularView<Eigen::StrictlyUpper>().transpose(); // can't use selfadjointView for some reason
								inv_storage = inv_storage.inverse();
							} else
								inv_storage = inv_storage.inverse(); // can't predict block sizes (could predict combinations of two, three)
						} else {
#ifndef __DISABLE_GPU
							if(b_use_GPU)
								p_GPU_solver->Dense_Inverse(inv_storage, inv_storage, b_is_upper, b_use_GPU_CholInv);
							else
#endif // __DISABLE_GPU
								CLinearSolver_DenseEigen::Dense_Inverse(inv_storage, inv_storage, b_is_upper); // fallback to CPU
						}
						// invert it

						if(b_is_upper) {
							size_t n_col_base = 0;
							for(size_t i = n_start_column, m = 1; i < n_end_column; ++ i, ++ m) {
								size_t n_row_base = 0;
								for(size_t j = 0; j < m; ++ j) {
									CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
									t_block = inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols());
									n_row_base += t_block.rows();
								}
								n_col_base += T.n_BlockColumn_Column_Num(i);
							}
						} else {
							size_t n_col_base = 0;
							for(size_t i = n_start_column, m = n_end_column - n_start_column; i < n_end_column; ++ i) {
								size_t n_row_base = 0;
								for(size_t j = 0; j < m; ++ j) {
									CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
									t_block = inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols());
									n_row_base += t_block.rows();
								}
								n_col_base += T.n_BlockColumn_Column_Num(i);
							}
						}
						// copy the blocks back to the sparse matrix
					}
				}
				// for each diag block

#ifndef __DISABLE_GPU
				if(p_GPU_solver && n_tid)
					p_GPU_solver->Free_Memory();
				// delete solver memory whilw still in the current context
#endif // __DISABLE_GPU
			}

			if(b_had_large_mats) {
				_ASSERTE(!b_immediate_large_inverse); // should only enter here if mutlithreaded GPU tasking is disabled

				Eigen::MatrixXd inv_storage;

#ifndef __DISABLE_GPU
				CLinearSolver_DenseGPU *p_GPU_solver = (b_use_GPU)? &GPU_solvers.front() : 0;
#endif // __DISABLE_GPU
				// make an instance of the GPU solver

				//size_t n_end_column = 0;

#if 1
				_ASSERTE(!large_block_lists.empty());
				for(size_t n_orig_tid = 0, n_thread_num = large_block_lists.size(); n_orig_tid < n_thread_num; ++ n_orig_tid) {
					const std::vector<size_t> &r_large_blocks = large_block_lists[n_orig_tid];
					for(size_t n_lg_block = 0, n_lg_block_num = r_large_blocks.size(); n_lg_block < n_lg_block_num; ++ n_lg_block) {
						const size_t n_block_id = r_large_blocks[n_lg_block];
#else // 1
				{
					for(int n_block_id = 0; n_block_id < _n_block_num; ++ n_block_id) {
#endif // 1
						/*std::vector<size_t>::const_iterator p_start_it = std::lower_bound(vertex_subgraph.begin() +
							n_end_column, vertex_subgraph.end(), size_t(n_block_id));
						size_t n_start_column = p_start_it - vertex_subgraph.begin();
						_ASSERTE(*p_start_it == n_block_id);
						_ASSERTE(n_start_column >= n_block_id); // there might be more columns taking one block so start might get grater but never lower
						// find start column

						std::vector<size_t>::const_iterator p_end_it = std::lower_bound(p_start_it,
							const_cast<const std::vector<size_t>&>(vertex_subgraph).end(), size_t(n_block_id) + 1);
						n_end_column = p_end_it - vertex_subgraph.begin();
						_ASSERTE(p_end_it == vertex_subgraph.end() || *p_end_it == size_t(n_block_id) + 1);
						// find end column*/

						const size_t n_start_column = block_list[n_block_id].first;
						const size_t n_end_column = block_list[n_block_id].second;
						// get start / end columns

						if(n_end_column == n_start_column + 1) {
							_ASSERTE(T.n_BlockColumn_Block_Num(n_start_column) == 1 &&
								T.n_Block_Row(n_start_column, 0) == n_start_column); // make sure there is a single diag. block
							CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(n_start_column, 0);
							// just a single block

							if(t_block.cols() >= n_GPU_thresh) {
								//fbs_ut::CWrap<CBlockInverse>::In_ColumnWidth_DecisionTree<CBlockSizes>(int(t_block.cols()), t_block);
							//} else {
#ifndef __DISABLE_GPU
								if(b_use_GPU)
									p_GPU_solver->Dense_Inverse(t_block.data(), t_block.data(), t_block.cols(), false, b_use_GPU_CholInv);
								else
#endif // __DISABLE_GPU
									CLinearSolver_DenseEigen::Dense_Inverse_Parallel(t_block, t_block, false);
							}
							// invert it
						} else {
							size_t n_col_num = 0;
							for(size_t i = n_start_column; i < n_end_column; ++ i)
								n_col_num += T.n_BlockColumn_Column_Num(i);
							// sum column widths

							if(n_col_num < n_GPU_thresh)
								continue;
							// don't do anything with the small ones

							inv_storage.resize(n_col_num, n_col_num);
							// resize temp storage

							if(b_is_upper) {
								size_t n_col_base = 0;
								for(size_t i = n_start_column, m = 1; i < n_end_column; ++ i, ++ m) {
									size_t n_row_base = 0;
									for(size_t j = 0; j < m; ++ j) {
										CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
										inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols()) = t_block;
										n_row_base += t_block.rows();
									}
									n_col_base += T.n_BlockColumn_Column_Num(i);
								}
							} else {
								size_t n_col_base = 0;
								for(size_t i = n_start_column, m = n_end_column - n_start_column; i < n_end_column; ++ i) {
									size_t n_row_base = 0;
									for(size_t j = 0; j < m; ++ j) {
										CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
										inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols()) = t_block;
										n_row_base += t_block.rows();
									}
									n_col_base += T.n_BlockColumn_Column_Num(i);
								}
							}
							// copy blocks to the dense block

							//if(n_col_num < n_GPU_thresh) {
							//	inv_storage = inv_storage.inverse(); // can't predict block sizes (could predict combinations of two, three)
							//} else {
#ifndef __DISABLE_GPU
								if(b_use_GPU)
									p_GPU_solver->Dense_Inverse(inv_storage, inv_storage, b_is_upper, b_use_GPU_CholInv);
								else
#endif // __DISABLE_GPU
									CLinearSolver_DenseEigen::Dense_Inverse_Parallel(inv_storage, inv_storage, b_is_upper);
							//}
							// invert it

							if(b_is_upper) {
								size_t n_col_base = 0;
								for(size_t i = n_start_column, m = 1; i < n_end_column; ++ i, ++ m) {
									size_t n_row_base = 0;
									for(size_t j = 0; j < m; ++ j) {
										CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
										t_block = inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols());
										n_row_base += t_block.rows();
									}
									n_col_base += T.n_BlockColumn_Column_Num(i);
								}
							} else {
								size_t n_col_base = 0;
								for(size_t i = n_start_column, m = n_end_column - n_start_column; i < n_end_column; ++ i) {
									size_t n_row_base = 0;
									for(size_t j = 0; j < m; ++ j) {
										CUberBlockMatrix::_TyMatrixXdRef t_block = T.t_Block_AtColumn(i, j);
										t_block = inv_storage.block(n_row_base, n_col_base, t_block.rows(), t_block.cols());
										n_row_base += t_block.rows();
									}
									n_col_base += T.n_BlockColumn_Column_Num(i);
								}
							}
							// copy the blocks back to the sparse matrix
						}
					}
				}
			}
			// invert the large blocks using thread 0

			//f_inv_time += dt.f_Time(); // profiling

			if(b_is_upper && b_transpose_result_to_lower) {
				/*CUberBlockMatrix TU, TUT;
				TU.TriangularViewOf(T, true, true, -1); // strictly upper
				TUT.TransposeOf(TU);
				TUT.AddTo(T);*/ // t_odo - this unnecessarily sorts the columns, write a SelfAdjointViewOf() with the option to work inplace
				T.SelfAdjointViewOf(T, true);
			}
			// transpose the strictly upper part to lower

			//f_symmetrize_time += dt.f_Time(); // profiling
		}
		// invert the matrices in the list to get more stats

		//printf("minv time: %f find blocks, %f inv, %f symm\n", f_block_find_time, f_inv_time, f_symmetrize_time); // profiling
	}
};

class CFBS_SpDGEMM {
protected:
	CUberBlockMatrix &m_r_dest;
	const CUberBlockMatrix &m_r_a, m_r_b;
	bool m_b_upper_triag_only;

public:
	CFBS_SpDGEMM(CUberBlockMatrix &r_dest, const CUberBlockMatrix &r_a,
		const CUberBlockMatrix &r_b, bool b_upper_triag_only)
		:m_r_dest(r_dest), m_r_a(r_a), m_r_b(r_b), m_b_upper_triag_only(b_upper_triag_only)
	{}

	template <class CBlockSizes>
	inline void operator ()()
	{
		typedef typename CTransformTypelist<CBlockSizes,
			fbs_ut::CSize2DToTransposeSize2D>::_TyResult CBlockSizesTr;
		m_r_dest.ProductOf_FBS<CBlockSizesTr, CBlockSizes>(m_r_a, m_r_b, m_b_upper_triag_only);
	}
};

/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 *
 *	@note This function throws std::bad_alloc, std::runtime_error.
 */
int throwing_main(int n_arg_num, const char **p_arg_list) // throw(std::bad_alloc, std::runtime_error)
{
	//CMatrixTransposeSum_UnitTests::Test_AAT();
	//Test_SparseOpsCost();
	//return 0;
	// tests

	printf("> ./SLAM_schur_orderings");
	for(int i = 1; i < n_arg_num; ++ i) {
		const char *p_s_arg = p_arg_list[i], *p_s_arg_end = p_s_arg + strlen(p_s_arg);
		const char *p_s_white = " \t\b\r\n", *p_s_white_end = p_s_white + strlen(p_s_white);
		if(std::find_first_of(p_s_arg, p_s_arg_end, p_s_white, p_s_white_end) != p_s_arg_end)
			printf(" \"%s\"", p_arg_list[i]);
		else
			printf(" %s", p_arg_list[i]);
	}
	printf("\n\n");

	std::string s_input = (n_arg_num >= 2)? p_arg_list[1] : "data/SLAM_sys/manhattanOlson3500";

	int n_min_clique_size = 2;
	int n_max_clique_size = INT_MAX;
	bool b_prune_cliques = false;
	int n_default_resolution = 640; // subsampled
	int n_default_resolution2 = 1024; // subsampled, AA
	const char *p_s_combine_path = 0;
	size_t n_cut_matrix = 0;
	bool b_skip_expensive_orderings = false;
	bool b_skip_perf_test = false;
	bool b_try_exact_MICS = false;
	bool b_no_bf_cliques = false, b_no_ig_cliques = false;
#ifndef HAVE_IGRAPH
	b_no_ig_cliques = true; // if not compiled then default to true
#endif // !HAVE_IGRAPH
	bool b_try_MICS_AMICS_specified = false;
	bool b_try_MICS_AMICS = true;
	bool b_try_guided_specified = false;
	bool b_try_guided = false;
	bool b_no_additional_FLOPs_bench = false;
	bool b_no_casc_image = false;
	bool b_debug_verbose = false;
	bool b_do_GPU_perf_test_only = false; // todo - add an commandline parser for that
	bool b_do_arrow_SC_test_only = false;
	const char *p_s_arrow_SC_size_arg = 0;
	const char *p_s_arrow_SC_fraction_arg = 0;
	double f_damping = 0;
	for(int i = 2; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "-h") || !strcmp(p_arg_list[i], "--help")) {
			printf("use: %s <input> [options]\n\nwhere available [options] are:\n"
				"-nex         - no expensive ordering algorithms\n"
				"-naf         - no additional FLOPs benchmarks\n"
				"-emics       - try exact MICS\n"
				"-mics|-amics - try approximate MICS\n"
				"-nmics       - do not try approximate MICS\n"
				"-nic         - no igraph clique search\n"
				"-nbc         - no brute force clique search\n"
				"-ndcc        - no dense CPU Cholesky\n"
				"-gord        - enable guided ordering\n"
				"-cut <n>     - cuts the matrix to <n> by <n> blocks (no cutting by default)\n"
				"-nes <path>  - nested Schur analysis, starting at top level <path>\n"
				"               (this runs in a different mode and ignores most of other flags)\n"
				"-nni         - disables writing nested Schur images (no effect otherwise)\n"
				"-pc          - prune cliques\n"
				"-mcs <n>     - sets the maximum clique size to <n> variables (default unlimited)\n"
				"-ncs <n>     - sets the minimum clique size to <n> variables (default 2)\n"
				"-npt         - skip performance test\n"
				"-dv          - enables debugging verbose\n"
				"-Gpto        - GPU performance tests only\n"
				"-aSCo [size|all|small] [fract] - arrow SC test only\n", p_arg_list[0]);
			return 0;
		} else if(!strcmp(p_arg_list[i], "-nex"))
			b_skip_expensive_orderings = true;
		else if(!strcmp(p_arg_list[i], "-Gpto"))
			b_do_GPU_perf_test_only = true;
		else if(!strcmp(p_arg_list[i], "-aSCo")) {
			b_do_arrow_SC_test_only = true;
			if(i + 1 < n_arg_num)
				p_s_arrow_SC_size_arg = p_arg_list[++ i];
			if(i + 1 < n_arg_num)
				p_s_arrow_SC_fraction_arg = p_arg_list[++ i];
		} else if(!strcmp(p_arg_list[i], "-pc"))
			b_prune_cliques = true;
		else if(!strcmp(p_arg_list[i], "-dv"))
			b_debug_verbose = true;
		else if(!strcmp(p_arg_list[i], "-naf"))
			b_no_additional_FLOPs_bench = true;
		else if(!strcmp(p_arg_list[i], "-npt"))
			b_skip_perf_test = true;
		else if(!strcmp(p_arg_list[i], "-nni"))
			b_no_casc_image = true;
		else if(!strcmp(p_arg_list[i], "-emics")) {
#ifdef HAVE_IGRAPH
			b_try_exact_MICS = true;
			b_try_MICS_AMICS_specified = true;
#else // HAVE_IGRAPH
			fprintf(stderr, "error: compiled without igraph support: the -emics option ignored\n");
#endif // HAVE_IGRAPH
		} else if(!strcmp(p_arg_list[i], "-mics") || !strcmp(p_arg_list[i], "-amics")) {
			b_try_MICS_AMICS = true;
			b_try_MICS_AMICS_specified = true;
		} else if(!strcmp(p_arg_list[i], "-nmics")) {
			b_try_MICS_AMICS = false;
			b_try_MICS_AMICS_specified = true;
		} else if(!strcmp(p_arg_list[i], "-nbc")) {
#ifdef HAVE_IGRAPH
			b_no_bf_cliques = true;
#else // HAVE_IGRAPH
			fprintf(stderr, "error: compiled without igraph support: the -nbc option ignored\n");
#endif // HAVE_IGRAPH
		}else if(!strcmp(p_arg_list[i], "-nic"))
			b_no_ig_cliques = true;
		else if(!strcmp(p_arg_list[i], "-ndcc"))
			b_try_dense_CPU_solver = false;
		else if(!strcmp(p_arg_list[i], "-gord")) {
			b_try_guided = true;
			b_try_guided_specified = true;
		} else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: unknown switch or missing a value: \'%s\'\n", p_arg_list[i]);
			return -1;
		} else if(!strcmp(p_arg_list[i], "-nes"))
			p_s_combine_path = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "-mcs"))
			n_max_clique_size = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "-ncs")) {
			if((n_min_clique_size = atol(p_arg_list[++ i])) < 2) {
				fprintf(stderr, "warning: invalid minimum clique size: %d (resetting to 2)\n", n_min_clique_size);
				n_min_clique_size = 2;
			}
		} else if(!strcmp(p_arg_list[i], "-cut"))
			n_cut_matrix = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "-dm"))
			f_damping = atof(p_arg_list[++ i]);
		else {
			fprintf(stderr, "error: unknown switch: \'%s\'\n", p_arg_list[i]);
			return -1;
		}
	}
	// "parse" commandline

	if(b_no_bf_cliques && b_no_ig_cliques) {
		fprintf(stderr, "error: -nic and -nbc are mutually exclusive\n");
		return -1;
	}
	// check

#ifdef _OPENMP
#pragma omp parallel
#pragma omp master
	{
		printf("%s (%d threads)\n", "_OPENMP", omp_get_num_threads());
	}
#endif // _OPENMP
	
	if(b_do_arrow_SC_test_only) {
		if(p_s_arrow_SC_size_arg && !strcmp(p_s_arrow_SC_size_arg, "all")) {
			const float f = (p_s_arrow_SC_fraction_arg)? float(atof(p_s_arrow_SC_fraction_arg)) / 100 : .99f;
			for(int i = 0; i < 5; ++ i)
				Test_SchurSparseCost(300 << i, f, i > 0);
			for(int i = 0; i < 30; ++ i)
				Test_SchurSparseCost(6000 * (i + 1), f, true);
		} else if(p_s_arrow_SC_size_arg && !strcmp(p_s_arrow_SC_size_arg, "small")) {
			const float f = (p_s_arrow_SC_fraction_arg)? float(atof(p_s_arrow_SC_fraction_arg)) / 100 : .99f;
			for(int i = 0; i < 5; ++ i)
				Test_SchurSparseCost(300 << i, f, i > 0);
			for(int i = 0; i < 5; ++ i)
				Test_SchurSparseCost(6000 * (i + 1), f, true);
		} else {
			Test_SchurSparseCost((p_s_arrow_SC_size_arg)? atol(p_s_arrow_SC_size_arg) : 300,
				(p_s_arrow_SC_fraction_arg)? float(atof(p_s_arrow_SC_fraction_arg) / 100) : .99f);
		}
		return 0;
	}

	printf("fear and loading \'%s\' ... ", (s_input + "/system.mtx").c_str()); fflush(stdout);
	CUberBlockMatrix lambda;
	if(!lambda.Load_MatrixMarket((s_input + "/system.mtx").c_str(),
	   (s_input + "/system.bla").c_str())) {
		printf("\n");
		fprintf(stderr, "error: failed to load the matrix\n");
		return -1;
	}
	printf("done (needs %.3f MB)\n", lambda.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);
	// load lambda (from SLAM)

	if(f_damping) {
		for(size_t i = 0, n = lambda.n_BlockColumn_Num(); i < n; ++ i)
			lambda.t_GetBlock_Log(i, i).diagonal().array() += f_damping;
	}
	// apply damping for BA problems

	CFBSSpecializer specializer; // not a typedef!
	// prepare an algorithm specializer

	fbs_ut::CDummyAlgSpecializer::TBSMix t_block_sizes =
		fbs_ut::CDummyAlgSpecializer::t_BlockSize_Mixture(lambda);
	// get block sizes out of lambda

	const size_t n_specialization_id = specializer.n_Find_SuitableSpecialization(t_block_sizes);
	if(n_specialization_id == size_t(-1))
		throw std::runtime_error("failed to find FBS algorithm specialization");
	// select which specialization can handle those

	if(b_try_MICS_AMICS && !b_try_MICS_AMICS_specified && (n_specialization_id == 3 || n_specialization_id == 4)) {
		printf("solving a BA problem, disabling MICS type orderings (use -mics to enable)\n");
		b_try_MICS_AMICS = false;
	}
	if(!b_try_guided && !b_try_guided_specified && (n_specialization_id == 3 || n_specialization_id == 4)) {
		printf("solving a BA problem, enabling guided ordering\n");
		b_try_guided = true;
	}
	if(b_try_guided && n_specialization_id < 2) {
		printf("solving pose-only problem, disabling guided ordering\n");
		b_try_guided = false;
	}
	// shuffle the flags around

	if(n_cut_matrix && (n_cut_matrix < lambda.n_BlockColumn_Num() || n_cut_matrix < lambda.n_BlockRow_Num())) {
		CUberBlockMatrix lam_cut;
		lambda.SliceTo(lam_cut, std::min(n_cut_matrix, lambda.n_BlockRow_Num()),
			std::min(n_cut_matrix, lambda.n_BlockColumn_Num()));
		lambda.Swap(lam_cut);
		printf("cut to " PRIsize " x " PRIsize " blocks (needs %.3f MB)\n",
			lambda.n_BlockRow_Num(), lambda.n_BlockColumn_Num(),
			lambda.n_Allocation_Size_NoLastPage() / 1048576.0); fflush(stdout);
	}
	// cut lambda if a smaller size is specified

	CDeltaTimer dt;

	if(b_do_GPU_perf_test_only) {
		std::vector<size_t> ord, mis;
		if(1) {
			cs *p_lambda = lambda.p_BlockStructure_to_Sparse();
			
			if(!b_try_guided)
				MIS_FF(mis, p_lambda, lambda);
			else
				MIS_Guided(mis, p_lambda, lambda);

			cs_spfree(p_lambda);
			ord.clear();
			CSchurOrdering::Complement_VertexSet(ord, mis, lambda.n_BlockColumn_Num());
			const size_t n_cut = ord.size();
			ord.insert(ord.end(), mis.begin(), mis.end());
			// calculate full ordering, dependent vertices first

			printf("schur ordering: " PRIsize " / " PRIsize " variables\n", n_cut, lambda.n_BlockColumn_Num());
			// verbose

			CMatrixOrdering mord; // actually need an inverse ordering
			CUberBlockMatrix lambda_perm;
			const size_t *oi;
			lambda.Permute_UpperTriangular_To(lambda_perm,
				oi = mord.p_InvertOrdering(&ord.front(), ord.size()), ord.size(), true);

			CUberBlockMatrix A, U, D;
			const size_t n_size = lambda_perm.n_BlockColumn_Num();
			//lambda_perm.SliceTo(A, 0, n_cut, 0, n_cut, true);
			lambda_perm.SliceTo(U, 0, n_cut, n_cut, n_size, false); // deep copy
			//lambda_perm.SliceTo(D, n_cut, n_size, n_cut, n_size, true);
			U.TransposeTo(lambda);

			size_t n_alloc_size = lambda.n_Allocation_Size();
			printf("V: " PRIsize " x " PRIsize " (" PRIsize " x " PRIsize ", %.2f MB)\n",
				lambda.n_BlockRow_Num(), lambda.n_BlockColumn_Num(),
				lambda.n_Row_Num(), lambda.n_Column_Num(), n_alloc_size / 1048576.0);
			// verbose
		}
		// test on the schur product

		CUberBlockMatrix lambda_tr;
		lambda.TransposeTo(lambda_tr);

		CLinearSolver_Schur_GPUBase gpu;

		bool b_upper_prod = true;

		fbs_ut::CDummyAlgSpecializer::TBSMix t_block_sizes2 =
			fbs_ut::CDummyAlgSpecializer::t_BlockSize_Mixture(lambda);
		CFBSSpecializer2 speclializer2;
		const size_t n_specialization_id2 = speclializer2.n_Find_SuitableSpecialization(t_block_sizes2);
		if(n_specialization_id2 == size_t(-1))
			throw std::runtime_error("failed to find FBS algorithm specialization");

		printf("debug: block sizes in lambda: ");
		fbs_ut::CDummyAlgSpecializer::Print_BlockSizes(t_block_sizes2);
		printf("\n");
		printf("debug: using specialization " PRIsize ": ", n_specialization_id2);
		fbs_ut::CDummyAlgSpecializer::Print_BlockSizes(speclializer2.t_Specialization(n_specialization_id2));
		printf("\n");
		// verbose

		dt.Reset();

		CUberBlockMatrix gpu_prod;
		for(int i = -1; i < 10; ++ i) {
			gpu.SpDGEMM(gpu_prod, lambda_tr, lambda, b_upper_prod);
			if(i == -1)
				dt.Reset();
		}

		double f_gpu_time = dt.f_Time() / 10; printf("."); fflush(stdout); dt.Reset();

		CUberBlockMatrix fbs_prod;
		for(int i = 0; i < 10; ++ i) {
			CFBSSpecializer2::Run<fbs_ut::CCallFnOp>(n_specialization_id2,
				CFBS_SpDGEMM(fbs_prod, lambda_tr, lambda, b_upper_prod));
		}

		double f_fbs_time = dt.f_Time() / 10; printf("."); fflush(stdout); dt.Reset();

		CUberBlockMatrix ref_prod;
		for(int i = 0; i < 10; ++ i)
			ref_prod.ProductOf(lambda_tr, lambda, b_upper_prod);

		double f_ubm_time = dt.f_Time() / 10; printf("\r");

		double f_ref_norm = ref_prod.f_Norm();
		ref_prod.AddTo(fbs_prod, -1);
		ref_prod.AddTo(gpu_prod, -1);
		double f_gpu_err_norm = gpu_prod.f_Norm();
		double f_fbs_err_norm = fbs_prod.f_Norm();

		printf("SpDGEMM took %.3f msec UBM, %.3f msec FBS, %.3f msec GPU (FBS error %g abs, rel %g, GPU error %g abs, rel %g)\n",
			f_ubm_time * 1000, f_fbs_time * 1000, f_gpu_time * 1000, f_fbs_err_norm, f_fbs_err_norm / f_ref_norm,
			f_gpu_err_norm, f_gpu_err_norm / f_ref_norm);

		return 0;
	}

	if(p_s_combine_path) {
		printf("will combine results from \'%s\' ...\n", p_s_combine_path); fflush(stdout);

		std::string s_combine_path = p_s_combine_path;
		// make a copy so it can be edited

		while(!s_combine_path.empty() && strchr("/\\", *(s_combine_path.end() - 1)))
			s_combine_path.erase(s_combine_path.end() - 1);
		// remove trailing slash(es)

		const std::string s_output_path = s_combine_path;

		std::vector<CUberBlockMatrix> matrix_list;
		std::vector<std::pair<std::vector<size_t>, size_t> > ordering_list;

		{
			CUberBlockMatrix M;
			if(!M.Load_MatrixMarket((s_combine_path + "/system.mtx").c_str(),
			   (s_combine_path + "/system.bla").c_str())) {
				fprintf(stderr, "error: failed to load \'%s\'\n",
					(s_combine_path + "/system.mtx").c_str());
				return -1;
			}
			matrix_list.push_back(M);
		}

		std::vector<std::string> path_list;
		for(size_t n_pos; (n_pos = s_combine_path.find_last_of("/\\")) != std::string::npos;) {
			std::string s_last_part(s_combine_path.begin() + n_pos + 1, s_combine_path.end());
			if(s_last_part.find("schur_") != 0) {
				fprintf(stderr, "error: lost path: \'%s\'\n", s_combine_path.c_str());
				return -1;
			}
			s_last_part.erase(0, strlen("schur_"));
			s_combine_path.erase(n_pos); // leave only the path after

			printf("loading from \'%s\'\n", s_combine_path.c_str()); fflush(stdout); // verbose

			{
				FILE *p_fr;
				std::string s_ordering_file = s_combine_path + "/ordering_" + s_last_part + ".txt";
				if(!(p_fr = fopen(s_ordering_file.c_str(), "r"))) {
					fprintf(stderr, "error: cannot find \'%s\'\n", s_ordering_file.c_str());
					return -1;
				}
				size_t n, n_cut;
				if(fscanf(p_fr, PRIsize "\n" PRIsize "\n", &n, &n_cut) != 2) {
					fprintf(stderr, "error: failed parsing \'%s\'\n", s_ordering_file.c_str());
					fclose(p_fr);
				}
				std::vector<size_t> perm(n);
				for(size_t i = 0; i < n; ++ i) {
					if(fscanf(p_fr, PRIsize " ", &perm[i]) != 1) {
						fprintf(stderr, "error: failed parsing \'%s\'\n", s_ordering_file.c_str());
						fclose(p_fr);
					}
				}
				fclose(p_fr);
				if(!ordering_list.empty() && ordering_list.back().first.size() != n_cut) {
					fprintf(stderr, "error: ordering cut / ordering size mismatch at \'%s\' ("
						PRIsize " / " PRIsize ")\n", s_combine_path.c_str(),
						n_cut, ordering_list.back().first.size());
					return -1;
				}
				ordering_list.push_back(std::make_pair(perm, n_cut));
			}

			{
				CUberBlockMatrix M;
				if(ordering_list.back().first.size() == lambda.n_BlockColumn_Num()) {
					printf("found base path: \'%s\'\n", s_combine_path.c_str());
					M.Swap(lambda); // will make lambda empty
					matrix_list.push_back(M);
					break;
				}
				//if(!M.Load_MatrixMarket((s_combine_path + "/system.mtx").c_str(),
				//   (s_combine_path + "/system.bla").c_str())) {
				if(!M.Load_BlockLayout(//(s_combine_path + "/system.mtx").c_str(), // block layout is enough, just want to compare size
				   (s_combine_path + "/system.bla").c_str())) {
					/*if(ordering_list.back().first.size() == lambda.n_BlockColumn_Num()) { // not here, will get into trouble at some point
						printf("found base path: \'%s\'\n", s_combine_path.c_str());
						M.Swap(lambda); // will make lambda empty
						matrix_list.push_back(M);
						break;
					} else*/ {
						fprintf(stderr, "error: failed to load \'%s\'\n",
							(s_combine_path + "/system.mtx").c_str());
						return -1;
					}
				}
				matrix_list.push_back(M);
			}

			if(matrix_list.back().n_BlockColumn_Num() != ordering_list.back().first.size()) {
				fprintf(stderr, "error: matrix / ordering size mismatch at \'%s\' ("
					PRIsize " / " PRIsize ")\n", s_combine_path.c_str(),
					ordering_list.back().first.size(), matrix_list.back().n_BlockColumn_Num());
				return -1;
			}
			// make sure the matrix and the ordering do match
		}
		// load all levels

		std::reverse(matrix_list.begin(), matrix_list.end());
		std::reverse(ordering_list.begin(), ordering_list.end());
		_ASSERTE(ordering_list.size() + 1 == matrix_list.size()); // loaded one more matrix above the loop
		// ...

		if(ordering_list.size() < 1) {
			fprintf(stderr, "error: not a Schur complement\n");
			return -1;
		}
		if(ordering_list.size() < 2) // allow, mostly because of the GPU results
			fprintf(stderr, "warning: not a nested Schur complement\n");
		printf("have " PRIsize " level nested Schur complement\n", ordering_list.size());
		printf("\toriginal size " PRIsize "\n", matrix_list.front().n_BlockColumn_Num());
		for(size_t i = 0, n = ordering_list.size(); i < n; ++ i)
			printf("\tcut at " PRIsize "\n", ordering_list[i].second);
		fflush(stdout);

		// see what we got

		/*for(size_t i = 20; i <= 300; ++ i) {
			Eigen::MatrixXd mat(i, i);
			mat.setIdentity();

			const int n_pass_num = 10;

			dt.Reset();

			for(int j = 0; j < n_pass_num; ++ j)
				mat = mat.inverse();

			double f_CPU_time = dt.f_Time() / n_pass_num;

			for(int j = 0; j < n_pass_num; ++ j)
				CLinearSolver_DenseEigen::Dense_Inverse_Parallel(mat, mat, false); // does not run, compiled with EIGEN_DONT_PARALLELIZE

			double f_CPU2_time = dt.f_Time() / n_pass_num;

			CLinearSolver_DenseGPU GPU_solver;
			for(int j = 0; j < n_pass_num; ++ j)
				GPU_solver.Dense_Inverse(mat, mat, false);

			double f_GPU_time = dt.f_Time() / n_pass_num;

			printf("size " PRIsize " x " PRIsize ": %.3f msec CPU, %.3f msec CPU2, %.3f msec GPU\n",
				i, i, f_CPU_time * 1000, f_CPU2_time * 1000, f_GPU_time * 1000); fflush(stdout);
		}
		// benchmark different types of inversions*/

		CLinearSolver_DenseGPU keep_the_foot_in_the_door;
		// init CUDA, stay initialized for the remainder of this scope

		dt.Reset();

		std::vector<size_t> cut_list(1, ordering_list.front().second);
		std::vector<size_t> combined_perm = ordering_list.front().first;
		for(size_t i = 1, n = ordering_list.size(); i < n; ++ i) {
			const std::vector<size_t> &r_next_perm = ordering_list[i].first;
			const size_t m = r_next_perm.size();
			_ASSERTE(m == cut_list.back());
			std::vector<size_t> combined_front(combined_perm.begin(), combined_perm.begin() + m);
			for(size_t j = 0; j < m; ++ j)
				combined_perm[j] = combined_front[r_next_perm[j]];
			cut_list.push_back(ordering_list[i].second); // where to cut
		}
		// combine permutations into one, get a cut list

		CMatrixOrdering mord;
		matrix_list.front().Permute_UpperTriangular_To(lambda, mord.p_InvertOrdering((combined_perm.empty())?
			0 : &combined_perm.front(), combined_perm.size()), combined_perm.size(), true);
		// ...

		std::vector<TSchur> schur_list(cut_list.size());
		std::vector<CUberBlockMatrix> D_list0(1);
		std::vector<CUberBlockMatrix> D_list(cut_list.size() - 1);
		size_t n_nested_specialization_id = n_specialization_id;
		{
			const CUberBlockMatrix *p_mat = &lambda;
			for(size_t i = 0, n = cut_list.size(); i < n; ++ i) {
				schur_list[i].p_next_in_cascade = (i + 1 == n)? 0 : &schur_list[i + 1];
				specializer.Run<TSchur::CFrom_OrderedMatrix_FBS>((i)? n_nested_specialization_id :
					n_specialization_id, std::make_pair(std::make_pair(p_mat, std::make_pair(cut_list[i],
					(i)? &D_list[i - 1] : &D_list0.front())), &schur_list[i]));
				p_mat = &schur_list[i].SC;
				if(!i) {
					n_nested_specialization_id = specializer.n_Find_SuitableSpecialization(
						fbs_ut::CDummyAlgSpecializer::t_BlockSize_Mixture(*p_mat));
					// should be 6x6 only for BAs (if using guided at the bottom most level)
				}
			}
		}
		// build hierarchy of Schurs

		double f_cascaded_Schur_time = dt.f_Time();

		if(n_nested_specialization_id != n_specialization_id && n_nested_specialization_id < 2)
			printf("debug: nested Schur complements have a single block size\n");
		else if(n_nested_specialization_id != n_specialization_id) {
			printf("debug: nested Schur complements have a different block size mixture (%d, %d)\n",
				int(n_specialization_id), int(n_nested_specialization_id));
		}
		std::vector<CUberBlockMatrix> D_list20 = D_list0;
		std::vector<CUberBlockMatrix> D_list30 = D_list0;
		std::vector<CUberBlockMatrix> D_list40 = D_list0;
		std::vector<CUberBlockMatrix> D_list2 = D_list;
		std::vector<CUberBlockMatrix> D_list3 = D_list;
		std::vector<CUberBlockMatrix> D_list4 = D_list;

		dt.Reset();

		specializer.Run<CInvertBlockDiagonals>(n_specialization_id, &D_list0);
		specializer.Run<CInvertBlockDiagonals>(n_nested_specialization_id, &D_list);

		double f_block_inverse_time = dt.f_Time();

		specializer.Run<CInvertBlockDiagonals_Parallel>(n_nested_specialization_id, std::make_pair(&D_list2, false)); // CPU
		specializer.Run<CInvertBlockDiagonals_Parallel>(n_specialization_id, std::make_pair(&D_list20, false)); // CPU

		double f_block_inverse_parallel_time = dt.f_Time();

		double f_block_inverse_GPU_time = -1000;
		try {
			specializer.Run<CInvertBlockDiagonals_Parallel>(n_specialization_id, std::make_pair(&D_list30, true)); // GPU
			specializer.Run<CInvertBlockDiagonals_Parallel>(n_nested_specialization_id, std::make_pair(&D_list3, true)); // GPU

			double f_block_inverse_GPU_warmup_time = dt.f_Time();

			specializer.Run<CInvertBlockDiagonals_Parallel>(n_specialization_id, std::make_pair(&D_list40, true)); // GPU
			specializer.Run<CInvertBlockDiagonals_Parallel>(n_nested_specialization_id, std::make_pair(&D_list4, true)); // GPU

			f_block_inverse_GPU_time = dt.f_Time();
		} catch(std::bad_alloc&) {
			fprintf(stderr, "error: caught std::bad_alloc in GPU dense inverse\n");
		} catch(std::runtime_error &r_exc) {
			fprintf(stderr, "error: caught std::runtime_error (\'%s\') in GPU dense inverse\n", r_exc.what());
		} catch(std::exception &r_exc) {
			fprintf(stderr, "error: caught std::exception (\'%s\') in GPU dense inverse\n", r_exc.what());
		}

		{
			double f_err_sum = 0, f_max_err = 0;
			for(size_t i = 0, n = D_list.size(); i < n; ++ i) {
				D_list[i].AddTo(D_list2[i], -1);
				double f_err = D_list2[i].f_Squared_Norm();
				if(f_err > 1e-5) {
					char p_s_name[256];
					sprintf(p_s_name, "diff_%d.tga", int(i));
					D_list[i].Rasterize(p_s_name, 3);
				}
				f_err_sum += f_err;
				if(f_max_err < f_err)
					f_max_err = f_err;
			}
			for(size_t i = 0, n = D_list0.size(); i < n; ++ i) {
				D_list0[i].AddTo(D_list20[i], -1);
				double f_err = D_list20[i].f_Squared_Norm();
				if(f_err > 1e-5) {
					char p_s_name[256];
					sprintf(p_s_name, "diff0_%d.tga", int(i));
					D_list0[i].Rasterize(p_s_name, 3);
				}
				f_err_sum += f_err;
				if(f_max_err < f_err)
					f_max_err = f_err;
			}
			printf("debug: parallel block inverse error %g (abs, %g max block difference)\n",
				sqrt(f_err_sum), sqrt(f_max_err));
		}
		{
			double f_err_sum = 0, f_max_err = 0;
			for(size_t i = 0, n = D_list.size(); i < n; ++ i) {
				D_list[i].AddTo(D_list4[i], -1);
				double f_err = D_list4[i].f_Squared_Norm();
				f_err_sum += f_err;
				if(f_max_err < f_err)
					f_max_err = f_err;
			}
			for(size_t i = 0, n = D_list0.size(); i < n; ++ i) {
				D_list0[i].AddTo(D_list40[i], -1);
				double f_err = D_list40[i].f_Squared_Norm();
				f_err_sum += f_err;
				if(f_max_err < f_err)
					f_max_err = f_err;
			}
			printf("debug: GPU block inverse error %g (abs, %g max block difference)\n",
				sqrt(f_err_sum), sqrt(f_max_err));
		}

		size_t n_memory_needed = lambda.n_Allocation_Size_NoLastPage();
		size_t n_memory_last_SC = 0;
		double f_SC_nonzeros = 0;
		for(size_t i = 0, n = schur_list.size(); i < n; ++ i) {
			n_memory_needed += schur_list[i].minus_U_Dinv.n_Allocation_Size_NoLastPage();
			n_memory_needed += schur_list[i].U.n_Allocation_Size_NoLastPage();
			n_memory_needed += schur_list[i].minus_Dinv.n_Allocation_Size_NoLastPage();
			if(i + 1 < n) {
				n_memory_needed += schur_list[i].SC.n_Allocation_Size_NoLastPage();
				// sparse Schur complement (not the last level)
			} else {
				f_SC_nonzeros = double(schur_list[i].SC.n_Symmetric_NonZero_Num()) /
					(schur_list[i].SC.n_Row_Num() * schur_list[i].SC.n_Column_Num());
				n_memory_last_SC += schur_list[i].SC.n_Row_Num() *
					schur_list[i].SC.n_Column_Num() * sizeof(double);
				// dense Schur complement
			}
		}
		//
		printf("required memory: %.3f MB (" PRIsize " B), out of that:\n",
			(n_memory_needed + n_memory_last_SC) / 1048576.0, (n_memory_needed + n_memory_last_SC));
		printf("\tlambda: %.3f MB (" PRIsize " B)\n",
			matrix_list.front().n_Allocation_Size_NoLastPage() / 1048576.0,
			matrix_list.front().n_Allocation_Size_NoLastPage());
		printf("\tlast level dense SC: %.3f MB (" PRIsize " B; %.2f%% nonzeros)\n",
			n_memory_last_SC / 1048576.0, n_memory_last_SC, f_SC_nonzeros * 100);
		for(size_t n_level_num = schur_list.size() - 1; n_level_num > 0; -- n_level_num) {
			size_t n_memory_needed = lambda.n_Allocation_Size_NoLastPage();
			for(size_t i = 0; i < n_level_num; ++ i) {
				n_memory_needed += schur_list[i].minus_U_Dinv.n_Allocation_Size_NoLastPage();
				n_memory_needed += schur_list[i].U.n_Allocation_Size_NoLastPage();
				n_memory_needed += schur_list[i].minus_Dinv.n_Allocation_Size_NoLastPage();
				if(i + 1 < n_level_num) {
					n_memory_needed += schur_list[i].SC.n_Allocation_Size_NoLastPage();
					// sparse Schur complement (not the last level)
				} else {
					n_memory_needed += schur_list[i].SC.n_Row_Num() *
						schur_list[i].SC.n_Column_Num() * sizeof(double);
					// dense Schur complement
				}
			}
			printf(PRIsize "-level Schur would require %.3f MB (" PRIsize " B)\n",
				n_level_num, n_memory_needed / 1048576.0, n_memory_needed);
		}
		printf("full lambda would require %.3f MB (" PRIsize " B; %.2f%% nonzeros)\n",
			matrix_list.front().n_Row_Num() * matrix_list.front().n_Column_Num() * sizeof(double) / 1048576.0,
			matrix_list.front().n_Row_Num() * matrix_list.front().n_Column_Num() * sizeof(double),
			matrix_list.front().f_NonZero_Ratio() * 100); fflush(stdout);
		// memory stats

		Eigen::VectorXd ones = Eigen::VectorXd::Ones(lambda.n_Column_Num());
		Eigen::VectorXd rhs = Eigen::VectorXd::Zero(lambda.n_Column_Num());
		{
			CUberBlockMatrix lambda_full, lambda_su, lambda_sl;
			_ASSERTE(lambda.b_UpperBlockTriangular()); // should be
			lambda.SliceTo(lambda_full, 0, lambda.n_BlockColumn_Num(), 0, lambda.n_BlockColumn_Num(), true); // reference
			lambda_su.TriangularViewOf(lambda, true, true, -1); // strictly upper, reference
			lambda_sl.TransposeOf(lambda_su);
			lambda_sl.AddTo(lambda_full);
			lambda_full.PostMultiply_Add(&rhs(0), rhs.size(), &ones(0), ones.size()); // todo - product routines for symmetric matrices; check out the HPC'16 paper "let's agree on counting flops" for the algorithms
			// need a full matrix in order to compute
		}

		CUberBlockMatrix lambda_amd;
		{
			CMatrixOrdering mord;
			mord.p_BlockOrdering(lambda, true);
			lambda.Permute_UpperTriangular_To(lambda_amd,
				mord.p_Get_InverseOrdering(), mord.n_Ordering_Size(), true);
		}
		// form lambda with fill-reducing ordering

		dt.Reset(); if(b_debug_verbose) { printf("debug: specializer.Run<CTestBlockSolve>()\n"); fflush(stdout); }

		Eigen::VectorXd bcsol = rhs;
		specializer.Run<CTestBlockSolve>(n_specialization_id, std::make_pair(&lambda, &bcsol)); // this reorders the matrix anyways

		double f_bc_solve_time = dt.f_Time(); if(b_debug_verbose) { printf("debug: CLinearSolver_CSparse().Solve_PosDef()\n"); fflush(stdout); }

		Eigen::VectorXd cssol = rhs;
		CLinearSolver_CSparse().Solve_PosDef(lambda, cssol); // this reorders the matrix anyways

		double f_cs_solve_time = dt.f_Time(); if(b_debug_verbose) { printf("debug: CLinearSolver_CholMod(CHOLMOD_AUTO, "
			"CHOLMOD_AMD).Solve_PosDef()\n"); fflush(stdout); }

		Eigen::VectorXd chsol = rhs;
		CLinearSolver_CholMod(CHOLMOD_AUTO,
			CHOLMOD_AMD).Solve_PosDef(lambda, chsol); // this reorders the matrix anyways

		double f_ch_solve_time = dt.f_Time(); if(b_debug_verbose) { printf("debug: CLinearSolver_CholMod(CHOLMOD_SIMPLICIAL, "
			"CHOLMOD_AMD).Solve_PosDef()\n"); fflush(stdout); }

		Eigen::VectorXd chsolSimp = rhs;
		CLinearSolver_CholMod(CHOLMOD_SIMPLICIAL,
			CHOLMOD_AMD).Solve_PosDef(lambda, chsolSimp); // this reorders the matrix anyways

		double f_ch_simp_solve_time = dt.f_Time(); if(b_debug_verbose) { printf("debug: CLinearSolver_CholMod(CHOLMOD_SUPERNODAL, "
			"CHOLMOD_AMD).Solve_PosDef()\n"); fflush(stdout); }

		Eigen::VectorXd chsolSuper = rhs;
		CLinearSolver_CholMod(CHOLMOD_SUPERNODAL,
			CHOLMOD_AMD).Solve_PosDef(lambda, chsolSuper); // this reorders the matrix anyways

		double f_ch_sup_solve_time = dt.f_Time(); if(b_debug_verbose) { printf("debug: CLinearSolver_CholMod(CHOLMOD_AUTO, "
			"CHOLMOD_AMD).Factorize_PosDef_Blocky_Benchmark()\n"); fflush(stdout); }

		double f_ch_factorize_time;
		{
			CUberBlockMatrix R;
			lambda.CopyLayoutTo(R); // !!
			std::vector<size_t> workspace;
			CLinearSolver_CholMod(CHOLMOD_AUTO,
				CHOLMOD_AMD).Factorize_PosDef_Blocky_Benchmark(f_ch_factorize_time, R, lambda_amd, workspace);
		}

		double f_ch_factorize_overhead_time = dt.f_Time() - f_ch_factorize_time;

		double f_ch_simp_factorize_time; if(b_debug_verbose) { printf("debug: CLinearSolver_CholMod(CHOLMOD_SIMPLICIAL, "
			"CHOLMOD_AMD).Factorize_PosDef_Blocky_Benchmark()\n"); fflush(stdout); }
		{
			CUberBlockMatrix R;
			lambda.CopyLayoutTo(R); // !!
			std::vector<size_t> workspace;
			CLinearSolver_CholMod(CHOLMOD_SIMPLICIAL,
				CHOLMOD_AMD).Factorize_PosDef_Blocky_Benchmark(f_ch_simp_factorize_time, R, lambda_amd, workspace);
		}

		double f_ch_simp_factorize_overhead_time = dt.f_Time() - f_ch_simp_factorize_time;

		double f_ch_sup_factorize_time; if(b_debug_verbose) { printf("debug: CLinearSolver_CholMod(CHOLMOD_SUPERNODAL, "
			"CHOLMOD_AMD).Factorize_PosDef_Blocky_Benchmark()\n"); fflush(stdout); }
		{
			CUberBlockMatrix R;
			lambda.CopyLayoutTo(R); // !!
			std::vector<size_t> workspace;
			CLinearSolver_CholMod(CHOLMOD_SUPERNODAL,
				CHOLMOD_AMD).Factorize_PosDef_Blocky_Benchmark(f_ch_sup_factorize_time, R, lambda_amd, workspace);
		}

		double f_ch_sup_factorize_overhead_time = dt.f_Time() - f_ch_sup_factorize_time;

		double f_cs_factorize_time; if(b_debug_verbose) { printf("debug: CLinearSolver_CSparse().Factorize_PosDef_Blocky_Benchmark()\n"); fflush(stdout); }
		{
			CUberBlockMatrix R;
			lambda.CopyLayoutTo(R); // !!
			std::vector<size_t> workspace;
			CLinearSolver_CSparse().Factorize_PosDef_Blocky_Benchmark(f_cs_factorize_time, R, lambda_amd, workspace);
		}

		double f_cs_factorize_overhead_time = dt.f_Time() - f_cs_factorize_time; if(b_debug_verbose) { printf("debug: specializer.Run<CTestBlockFactorize>()\n"); fflush(stdout); }

		CUberBlockMatrix R;
		specializer.Run<CTestBlockFactorize>(n_specialization_id, std::make_pair(&R, &lambda_amd));

		double f_bc_factorize_time = dt.f_Time();

		double f_cs_error = (cssol - ones).norm();
		double f_cs_rel_error = f_cs_error / ones.norm();
		printf("CS solution error: %g (abs; %g rel)\n", f_cs_error, f_cs_rel_error);

		dt.Reset(); if(b_debug_verbose) { printf("debug: specializer.Run<TSchur::CSolve_FBS>() with block Cholesky\n"); fflush(stdout); }

		Eigen::VectorXd sol = rhs;
		//schur_list.front().Solve(&sol(0), sol.size(), 0);
		specializer.Run<TSchur::CSolve_FBS>(n_specialization_id,
			std::make_pair(std::make_pair(&sol(0), std::make_pair(sol.size(), 0)), &schur_list.front()));

		double f_sc_solve_time = dt.f_Time();

		double f_error = (sol - ones).norm();
		double f_rel_error = f_error / ones.norm();
		printf("SC solution error: %g (abs; %g rel)\n", f_error, f_rel_error); fflush(stdout);

		dt.Reset(); if(b_debug_verbose) { printf("debug: specializer.Run<TSchur::CSolve_FBS>() with Cholmod\n"); fflush(stdout); }

		Eigen::VectorXd sol1 = rhs;
		//schur_list.front().Solve(&sol(0), sol1.size(), 1);
		specializer.Run<TSchur::CSolve_FBS>(n_specialization_id,
			std::make_pair(std::make_pair(&sol1(0), std::make_pair(sol1.size(), 1)), &schur_list.front()));

		double f_sc_ch_solve_time = dt.f_Time(); if(b_debug_verbose) { printf("debug: specializer.Run<TSchur::CSolve_FBS>() with dense Eigen\n"); fflush(stdout); }

		double f_sc_ed_solve_time = -1000;
		try {
			sol1 = rhs;
			//schur_list.front().Solve(&sol(0), sol1.size(), 2);
			specializer.Run<TSchur::CSolve_FBS>(n_specialization_id,
				std::make_pair(std::make_pair(&sol1(0), std::make_pair(sol1.size(), 2)), &schur_list.front()));

			f_sc_ed_solve_time = dt.f_Time();
		} catch(std::bad_alloc&) {
			fprintf(stderr, "error: caught std::bad_alloc while dense solving\n");
		} catch(std::runtime_error &r_exc) {
			fprintf(stderr, "error: caught std::runtime_error (\'%s\') while dense solving\n", r_exc.what());
		} catch(std::exception &r_exc) {
			fprintf(stderr, "error: caught std::exception (\'%s\') while dense solving\n", r_exc.what());
		}

		double f_sc_gpu_solve_time = -1000; if(b_debug_verbose) { printf("debug: specializer.Run<TSchur::CSolve_FBS>() "
			"with GPU (will solve a " PRIsize " x " PRIsize " system)\n", schur_list.back().SC.n_Row_Num(),
			schur_list.back().SC.n_Column_Num()); fflush(stdout); }
#ifndef __DISABLE_GPU
		try {
			sol1 = rhs;
			//schur_list.front().Solve(&sol(0), sol1.size(), 3);
			specializer.Run<TSchur::CSolve_FBS>(n_specialization_id,
				std::make_pair(std::make_pair(&sol1(0), std::make_pair(sol1.size(), 3)), &schur_list.front()));

			double f_sc_gpu_solve_time_warmup = dt.f_Time();

			sol1 = rhs;
			//schur_list.front().Solve(&sol(0), sol1.size(), 3);
			specializer.Run<TSchur::CSolve_FBS>(n_specialization_id,
				std::make_pair(std::make_pair(&sol1(0), std::make_pair(sol1.size(), 3)), &schur_list.front()));

			f_sc_gpu_solve_time = dt.f_Time();
		} catch(std::bad_alloc&) {
			fprintf(stderr, "error: caught std::bad_alloc while GPU solving\n");
		} catch(std::runtime_error &r_exc) {
			fprintf(stderr, "error: caught std::runtime_error (\'%s\') while GPU solving\n", r_exc.what());
		} catch(std::exception &r_exc) {
			fprintf(stderr, "error: caught std::exception (\'%s\') while GPU solving\n", r_exc.what());
		}
#endif // __DISABLE_GPU

		printf("the solution took %.3f msec using CSparse (%.3f msec factorization, %.3f msec block conversion)\n",
			f_cs_solve_time * 1000, f_cs_factorize_time * 1000, f_cs_factorize_overhead_time * 1000);
		printf("the solution took %.3f msec using Cholmod (%.3f msec factorization, %.3f msec block conversion)\n",
			f_ch_solve_time * 1000, f_ch_factorize_time * 1000, f_ch_factorize_overhead_time * 1000);
		printf("the solution took %.3f msec using simplical Cholmod (%.3f msec factorization, %.3f msec block conversion)\n",
			f_ch_simp_solve_time * 1000, f_ch_simp_factorize_time * 1000, f_ch_simp_factorize_overhead_time * 1000);
		printf("the solution took %.3f msec using supernodal Cholmod (%.3f msec factorization, %.3f msec block conversion)\n",
			f_ch_sup_solve_time * 1000, f_ch_sup_factorize_time * 1000, f_ch_sup_factorize_overhead_time * 1000);
		printf("the solution took %.3f msec using block Chol (%.3f msec factorization, %.2f%% nnz (not density of Chol(SC)))\n",
			f_bc_solve_time * 1000, f_bc_factorize_time * 1000, double(R.n_Symmetric_NonZero_Num()) /
			(R.n_Row_Num() * R.n_Column_Num()) * 100);
		printf("the solution took %.3f + %.3f msec using SC + block Chol\n",
			f_sc_solve_time * 1000, f_cascaded_Schur_time * 1000);
		printf("the solution took %.3f + %.3f msec using SC + Cholmod\n",
			f_sc_ch_solve_time * 1000, f_cascaded_Schur_time * 1000);
		printf("the solution took %.3f + %.3f msec using SC + Eigen dense\n",
			f_sc_ed_solve_time * 1000, f_cascaded_Schur_time * 1000);
		printf("the solution took %.3f + %.3f msec using SC + GPU dense\n",
			f_sc_gpu_solve_time * 1000, f_cascaded_Schur_time * 1000);
		printf("block diagonal inverse took %.3f msec (in series)\n",
			f_block_inverse_time * 1000); fflush(stdout);
		printf("block diagonal inverse took %.3f msec (parallel)\n",
			f_block_inverse_parallel_time * 1000);
		printf("block diagonal inverse took %.3f msec (GPU)\n",
			f_block_inverse_GPU_time * 1000); fflush(stdout);

		if(!b_no_casc_image) {
			lambda.Rasterize_Symmetric((s_output_path + "/lambda_perm_cascaded.tga").c_str(), 3);
			cs *p_lam;
			if((p_lam = lambda.p_BlockStructure_to_Sparse())) {
				CDebug::Dump_SparseMatrix_Subsample((s_output_path +
					"/lambda_perm_cascaded_SS.tga").c_str(), p_lam, 0, n_default_resolution, true);
				CDebug::Dump_SparseMatrix_Subsample_AA((s_output_path +
					"/lambda_perm_cascaded_SS_AA.tga").c_str(), p_lam, 0, n_default_resolution2, true);
				cs_spfree(p_lam);
			}
			lambda.Rasterize_Symmetric((s_output_path + "/lambda_perm_cascaded5.tga").c_str(), 5);

			CUberBlockMatrix lambda_SC;
			schur_list.front().Composite_LambdaSC(lambda_SC);

			lambda_SC.Rasterize_Symmetric((s_output_path + "/lambda_SC_cascaded.tga").c_str(), 3);
			cs *p_lam1;
			if((p_lam1 = lambda_SC.p_BlockStructure_to_Sparse())) {
				CDebug::Dump_SparseMatrix_Subsample((s_output_path +
					"/lambda_SC_cascaded_SS.tga").c_str(), p_lam1, 0, n_default_resolution, true);
				CDebug::Dump_SparseMatrix_Subsample_AA((s_output_path +
					"/lambda_SC_cascaded_SS_AA.tga").c_str(), p_lam1, 0, n_default_resolution2, true);
				cs_spfree(p_lam1);
			}
			lambda_SC.Rasterize_Symmetric((s_output_path + "/lambda_SC_cascaded5.tga").c_str(), 5);

			CUberBlockMatrix lambda_SCu;
			lambda_SCu.TriangularViewOf(lambda_SC, true, true); // Rasterize_Symmetric() wants upper matrix, lambda_SC has some below-diagonal items where the inverses are (could fix this in the compositing function)
			lambda_SCu.Rasterize_Symmetric(lambda, false, (s_output_path + "/lambda_diff_cascaded.tga").c_str(), 3);
			//cs *p_lam, p_lam1;
			if((p_lam1 = lambda_SC.p_BlockStructure_to_Sparse())) {
				if((p_lam = lambda.p_BlockStructure_to_Sparse())) {
					CDebug::Dump_SparseMatrix_Subsample((s_output_path +
						"/lambda_diff_cascaded_SS.tga").c_str(), p_lam1, p_lam, n_default_resolution, true);
					CDebug::Dump_SparseMatrix_Subsample_AA((s_output_path +
						"/lambda_diff_cascaded_SS_AA.tga").c_str(), p_lam1, p_lam, n_default_resolution2, true);
					cs_spfree(p_lam);
				}
				cs_spfree(p_lam1);
			}
			lambda_SCu.Rasterize_Symmetric(lambda, false, (s_output_path + "/lambda_diff_cascaded5.tga").c_str(), 5);

			DrawLinesInMatImage((s_output_path + "/lambda_perm_cascaded.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/lambda_perm_cascaded5.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/lambda_perm_cascaded_SS.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/lambda_perm_cascaded_SS_AA.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());

			DrawLinesInMatImage((s_output_path + "/" "lambda_perm_cascaded.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_perm_cascaded5.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_perm_cascaded_SS.tga").c_str(), 0,
				0xffff0000, 1, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_perm_cascaded_SS_AA.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());

			DrawLinesInMatImage((s_output_path + "/" "lambda_SC_cascaded.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_SC_cascaded5.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_SC_cascaded_SS.tga").c_str(), 0,
				0xffff0000, 1, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_SC_cascaded_SS_AA.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());

			DrawLinesInMatImage((s_output_path + "/" "lambda_diff_cascaded.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_diff_cascaded5.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_diff_cascaded_SS.tga").c_str(), 0,
				0xffff0000, 1, cut_list, 1.0 / lambda.n_BlockColumn_Num());
			DrawLinesInMatImage((s_output_path + "/" "lambda_diff_cascaded_SS_AA.tga").c_str(), 0,
				0xffff0000, 3, cut_list, 1.0 / lambda.n_BlockColumn_Num());
		}

		return 0;
	}

	if(!b_skip_perf_test) {
		cs *p_lambda = lambda.p_Convert_to_Sparse(),
			*p_lambda_t = cs_transpose(p_lambda, true);

		cs *p_lambda_b = lambda.p_BlockStructure_to_Sparse();
		dt.Reset();
		csi *p_order_elem = cs_amd(1, p_lambda);
		csi *p_order_elem_inv = cs_pinv(p_order_elem, p_lambda->n);
		double f_amd_elem = dt.f_Time();
		csi *p_order_block = cs_amd(1, p_lambda_b);
		csi *p_order_block_inv = cs_pinv(p_order_block, p_lambda_b->n);
		double f_amd_block = dt.f_Time();
		cs_free(p_order_block);
		cs_free(p_order_block_inv);
		// time the ordering

		{
			dt.Reset();
			CMatrixOrdering mord;
			mord.p_Ordering(p_lambda_b);
			const size_t *oi = mord.p_InvertOrdering(mord.p_Get_Ordering(), mord.n_Ordering_Size());
			double f_amd_block2 = dt.f_Time();
			/*CUberBlockMatrix lambda_upper;
			lambda_upper.TriangularViewOf(lambda, true, true);*/
			dt.Reset();
			mord.p_BlockOrdering(lambda/*_upper*/, true, false); // needs an upper half (or has a third parameter which can be set so that it can take a full matrix ... duh)
			oi = mord.p_Get_InverseOrdering();
			double f_amd_block3 = dt.f_Time();
			CUberBlockMatrix lambda_perm;
			lambda.Permute_UpperTriangular_To(lambda_perm, oi, mord.n_Ordering_Size(), true);
			double f_block_perm = dt.f_Time();

			CUberBlockMatrix R; // output

			dt.Reset();

			/*fbs_ut::CWrap2<CTestCholesky,
				TBlockSizes>::In_ScalarSize_DecisionTree<typename TSpecializer::_TySelectorType>(
				n_specialization_id, lambda_perm);*/
			specializer.Run<CTestCholesky>(n_specialization_id,
				std::make_pair(&R, (const CUberBlockMatrix*)&lambda_perm));
			// wrap the code to be benchmarked

			double f_block_chol = dt.f_Time();

			if(R.b_Empty()) {
				fprintf(stderr, "error: Cholesky seems to have failed; trying to increase damping\n");

				CUberBlockMatrix::_TyMatrixXdRef t_block00 = lambda_perm.t_GetBlock_Log(0, 0);
				t_block00.diagonal().array() += 10000.0;
				dt.Reset();

				specializer.Run<CTestCholesky>(n_specialization_id,
					std::make_pair(&R, (const CUberBlockMatrix*)&lambda_perm));
				// wrap the code to be benchmarked

				f_block_chol = dt.f_Time();
			}

			std::vector<double> x(R.n_Column_Num(), 1.0);
			std::vector<double> y(R.n_Column_Num());

			dt.Reset();

			R.InversePermute_RightHandSide_Vector(&y.front(), &x.front(), x.size(), oi, mord.n_Ordering_Size());
			specializer.Run<CTestCholSolve>(n_specialization_id, std::make_pair(&x.front(), (const CUberBlockMatrix*)&R));
			R.Permute_RightHandSide_Vector(&x.front(), &y.front(), x.size(), oi, mord.n_Ordering_Size());

			double f_block_solve = dt.f_Time();

			printf("time for blockwise AMD: %.3f msec (from cs*, csparse)\n", f_amd_block * 1000);
			printf("time for blockwise AMD: %.3f msec (from cs*, AMD lib)\n", f_amd_block2 * 1000);
			printf("time for blockwise AMD: %.3f msec (from block mat)\n", f_amd_block3 * 1000);
			printf("time for blockwise permute: %.3f msec\n", f_block_perm * 1000);
			printf("time for blockwise Cholesky: %.3f msec (" PRIsize " nnz)\n",
				f_block_chol * 1000, R.n_NonZero_Num());
			printf("time for blockwise solve: %.3f msec\n", f_block_solve * 1000);
			fflush(stdout);
		}

		csi n = p_lambda->n;
		std::vector<double> b(n, 1.0); // rhs -> lhs
		std::vector<double> x(n); // helper

		dt.Reset();

		cs *p_lambda_perm = cs_symperm(p_lambda, p_order_elem_inv, 1);

		double f_elem_permute = dt.f_Time();

		css *S = cs_schol(0, p_lambda_perm); // use given ordering
		csn *N = cs_chol(p_lambda_perm, S);

		double f_elem_chol = dt.f_Time();

		if(N && N->L) {
			cs_ipvec(S->pinv, &b.front(), &x.front(), n);
			cs_lsolve(N->L, &x.front());
			cs_ltsolve(N->L, &x.front());
			cs_pvec(S->pinv, &x.front(), &b.front(), n);
		} else
			fprintf(stderr, "error: sparse Cholesky seems to have failed\n");

		double f_elem_solve = dt.f_Time();

		printf("time for elemwise AMD: %.3f msec\n", f_amd_elem * 1000);
		printf("time for elemwise permute: %.3f msec\n", f_elem_permute * 1000);
		printf("time for elemwise Cholesky: %.3f msec (" PRIsize " nnz)\n",
			f_elem_chol * 1000, (N && N->L)? N->L->p[N->L->n] : 0);
		printf("time for elemwise solve: %.3f msec\n", f_elem_solve * 1000);
		fflush(stdout);

		cs_free(p_order_elem);
		cs_free(p_order_elem_inv);
		cs_sfree(S);
		if(N)
			cs_nfree(N);
		cs_spfree(p_lambda_b);
		cs_spfree(p_lambda_perm);

		if(!b_no_additional_FLOPs_bench) {
			size_t n_FLOPs = n_GEMM_FLOP_Num(p_lambda_t, p_lambda);
			printf("\nnumber of FLOPs in lambdaT lambda: " PRIsize "\n", n_FLOPs);
		} else
			printf("\n");
		//size_t n_FLOPs2 = n_Chol_FLOP_Num(p_lambda);

		std::fill(b.begin(), b.end(), 1.0);
		std::pair<size_t, size_t> cslv = t_CholSolve_FLOP_Num(p_lambda, (b.empty())? 0 : &b.front());
		size_t n_FLOPs2 = cslv.first, n_FLOPs2_s = cslv.second; // chol and solve
		printf("number of FLOPs in Chol(lambda):   " PRIsize "\n", n_FLOPs2);
		printf("number of FLOPs in b = LL^T \\ x:   " PRIsize "\n", n_FLOPs2_s);
		/*if(!b_no_additional_FLOPs_bench) { // no LU benchmarks now, please
			size_t n_FLOPs3 = n_LU_FLOP_Num(p_lambda);
			printf("number of FLOPs in LU(lambda):     " PRIsize "\n\n", n_FLOPs3);
		} else*/
			printf("\n");
		fflush(stdout);
		cs_spfree(p_lambda);
		cs_spfree(p_lambda_t);
	} else
		printf("\nskipping performance tests\n\n");
	fflush(stdout);
	// count some FLOPs

	cs *p_lambda = 0;
	try {
		/*{
			cs *p_lambda_t, *p_lambda_ata;
			if(!(p_lambda = lambda.p_BlockStructure_to_Sparse()))
				throw std::bad_alloc(); // rethrow
			// get block structure as a sparse CSC

			if(!(p_lambda_t = cs_transpose(p_lambda, false))) {
				cs_spfree(p_lambda);
				throw std::bad_alloc(); // rethrow
			}
			if(!(p_lambda_ata = cs_add(p_lambda, p_lambda_t, 1, 1))) {
				cs_spfree(p_lambda);
				cs_spfree(p_lambda_t);
				throw std::bad_alloc(); // rethrow
			}
			cs_spfree(p_lambda);
			cs_spfree(p_lambda_t);
			p_lambda = p_lambda_ata;
		}*/
		// modify it to have symmetric structure // t_odo - implement this directly in CUberBlockMatrix

		{
			const size_t m = lambda.n_BlockRow_Num(), n = lambda.n_BlockColumn_Num();
			const size_t n_greater_dim = std::max(m, n);
			std::vector<size_t> col_lengths(n_greater_dim), workspace(n);

			size_t n_aat_nnz_num = lambda.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<false,
				true, false, true>(col_lengths.empty()? 0 : &col_lengths.front(), col_lengths.size(),
				(workspace.empty())? 0 : &workspace.front(), workspace.size());

			if(!(p_lambda = cs_spalloc(n_greater_dim, n_greater_dim, n_aat_nnz_num, 0, 0)))
				throw std::bad_alloc();

			enum {
				b_sorted = false // lambda is supposed to be symmetric, it will be sorted regardless
			};
			lambda.BlockStructure_SumWithSelfTranspose<false, true, false, true, b_sorted>(p_lambda->p,
				p_lambda->n + 1, p_lambda->i, n_aat_nnz_num, col_lengths.empty()? 0 : &col_lengths.front(),
				col_lengths.size(), (workspace.empty())? 0 : &workspace.front(), workspace.size());
		}
		// get block structure of A+A^T (with diagonal) as a sparse CSC

		//if(!(p_lambda = lambda.p_BlockStructure_SumWithSelfTransposeNoDiag_to_Sparse())) // structure of A+A^T but without the diagonal
		//	throw std::bad_alloc(); // rethrow
		// get block structure as a sparse CSC

		double f_block_structure_time = dt.f_Time();

		if(b_try_MICS_AMICS) {
			printf("debug: finding cliques ...\n"); fflush(stdout);
			std::vector<std::vector<size_t> > cliques_ig;
			double ig_time = -10000;
#ifdef HAVE_IGRAPH
			if(!b_no_ig_cliques) {
				dt.Reset();
				cliques_ig = CSchurOrdering::t_Find_Cliques_igraph(p_lambda, n_min_clique_size, n_max_clique_size/*305*/); // try to reduce max clique size, see if venice budges (303 is the largest clique in the data, found in VP)
				ig_time = dt.f_Time();
				printf("debug: igraph finished\n"); fflush(stdout);
			}
#endif // HAVE_IGRAPH
			std::vector<std::vector<size_t> > cliques;
			if(!b_no_bf_cliques) {
				dt.Reset();
				cliques = CSchurOrdering::t_Find_Cliques(p_lambda, false, n_min_clique_size, n_max_clique_size);
				double dummy_nf_time = dt.f_Time(), dummy_time;
				if(!b_prune_cliques) {
					cliques = CSchurOrdering::t_Find_Cliques(p_lambda, true, n_min_clique_size, n_max_clique_size);
					dummy_time = dt.f_Time();
				} else
					dummy_time = dummy_nf_time;
				std::sort(cliques.begin(), cliques.end());
				if(!b_no_ig_cliques) {
					std::sort(cliques_ig.begin(), cliques_ig.end());
					printf("ig takes %.3f msec to find cliques\n", ig_time * 1000);
				}
				printf("bf takes %.3f msec to find cliques\n", dummy_nf_time * 1000);
				printf("bf takes %.3f msec to find cliques and filter them\n", dummy_time * 1000);
				if(!b_prune_cliques && !b_no_ig_cliques) {
					if(cliques == cliques_ig)
						printf("debug: igraph found the same cliques\n");
					else
						printf("debug: igraph found different cliques\n");
					if(cliques.size() == cliques_ig.size())
						printf("debug: igraph found the same number of cliques\n");
					else if(cliques.size() > cliques_ig.size())
						printf("debug: igraph found less cliques\n");
					else
						printf("debug: igraph found more cliques\n");
				}
			} else {
				cliques.swap(cliques_ig); // use cliques_ig
				printf("ig takes %.3f msec to find cliques\n", ig_time * 1000);
			}
			fflush(stdout);
			// find some cliques quick

			if(b_prune_cliques) {
				printf("pruning the cliques somewhat\n");
				CSchurOrdering::PruneCliques_MaxVertexOrder(cliques, p_lambda->n);

				if(!b_no_ig_cliques && !b_no_bf_cliques) {
					CSchurOrdering::PruneCliques_MaxVertexOrder(cliques_ig, p_lambda->n);
					if(cliques == cliques_ig)
						printf("debug: igraph found the same pruned cliques\n");
					else
						printf("debug: igraph found different pruned cliques\n");
					if(cliques.size() == cliques_ig.size())
						printf("debug: igraph found the same number of pruned cliques\n");
					else if(cliques.size() > cliques_ig.size())
						printf("debug: igraph found less pruned cliques\n");
					else
						printf("debug: igraph found more pruned cliques\n");
				}
			}

			dt.Reset();

			const size_t n_old_graph_size = p_lambda->n,
				n_new_graph_size = p_lambda->n + cliques.size();

			std::vector<size_t> vertex_orders(n_old_graph_size, size_t(1));
			vertex_orders.reserve(n_new_graph_size);
			for(size_t i = 0, n = cliques.size(); i < n; ++ i)
				vertex_orders.push_back(cliques[i].size());
			// will attach the cliques to the graph, each clique will be treated as a vertex with a greater weight

			std::vector<size_t> vertex_weights(n_new_graph_size);
			for(size_t i = 0; i < n_old_graph_size; ++ i)
				vertex_weights[i] = lambda.n_BlockColumn_Column_Num(i);
			// initialize weights of the "scalar" vertices

			std::vector<std::vector<size_t> > vertex_clique_pseudovertices(n_old_graph_size);
			// edges from the individual vertices to all the cliques which include them (indexed by vertex id)

			for(size_t i = 0, n = cliques.size(); i < n; ++ i) {
				const std::vector<size_t> &r_clique = cliques[i];
				_ASSERTE(r_clique.size() > 1); // 2 or more vertices
				size_t n_clique_id = i + n_old_graph_size;
				// id of clique's pseudovertex

				size_t n_weight = 0;
				for(size_t j = 0, m = r_clique.size(); j < m; ++ j) {
					const size_t v = r_clique[j];
					n_weight += lambda.n_BlockColumn_Column_Num(v);
					vertex_clique_pseudovertices[v].push_back(n_clique_id);
				}
				vertex_weights[n_clique_id] = n_weight;
			}
			// for each vertex, collect the cliques that contain it
			// also calculates clique weights adjusted by block sizes while at it

			std::vector<std::vector<size_t> > new_edges(n_new_graph_size); // upper triangular incidence for each vertex
			for(size_t i = 0, n = cliques.size(); i < n; ++ i) {
				const std::vector<size_t> &r_clique = cliques[i];
				size_t n_clique_id = i + n_old_graph_size;
				// id of clique's pseudovertex

				std::vector<size_t> adjacent_vertices = r_clique;
				// vertices referenced are adjacent

				std::vector<size_t> adjacent_cliques;
				for(size_t j = 0, m = r_clique.size(); j < m; ++ j) {
					const size_t v = r_clique[j];
					adjacent_vertices.insert(adjacent_vertices.end(),
						&p_lambda->i[p_lambda->p[v]], &p_lambda->i[p_lambda->p[v + 1]]);
					// inherit also adjacencies of the vertices
				}
				std::sort(adjacent_vertices.begin(), adjacent_vertices.end());
				adjacent_vertices.erase(std::unique(adjacent_vertices.begin(),
					adjacent_vertices.end()), adjacent_vertices.end());
				// make an union

				for(size_t j = 0, m = adjacent_vertices/*r_clique*/.size(); j < m; ++ j) {
					const size_t v = adjacent_vertices/*r_clique*/[j];
					adjacent_cliques.insert(adjacent_cliques.end(),
						vertex_clique_pseudovertices[v].begin(),
						vertex_clique_pseudovertices[v].end());
					// refer also the adjacent cliques
				}

				std::sort(adjacent_cliques.begin(), adjacent_cliques.end());
				adjacent_cliques.erase(std::unique(adjacent_cliques.begin(),
					adjacent_cliques.end()), adjacent_cliques.end());
				_ASSERTE(adjacent_vertices.back() < adjacent_cliques.front()); // these lists do not overlap
				// make an union of those lists

				_ASSERTE(adjacent_vertices.back() < n_clique_id);
				//for(size_t j = 0, m = adjacent_vertices.size(); j < m; ++ j)
				//	new_edges[n_clique_id].push_back(adjacent_vertices[j]);
				new_edges[n_clique_id].insert(new_edges[n_clique_id].end(),
					adjacent_vertices.begin(), adjacent_vertices.end());
				// make edges to the original vertices

				for(size_t j = 0, m = adjacent_cliques.size(); j < m; ++ j) {
					if(adjacent_cliques[j] <= n_clique_id) // only half of them, will get the other half via transpose
						//new_edges[adjacent_cliques[j]].push_back(n_clique_id));
						new_edges[n_clique_id].push_back(adjacent_cliques[j]);
				}
				// make edges to the cliques
			}
			size_t n_new_nnz = 0;
			for(size_t i = 0, n = n_old_graph_size; i < n; ++ i) {
				new_edges[i].insert(new_edges[i].end(),
					&p_lambda->i[p_lambda->p[i]], /*&p_lambda->i[p_lambda->p[i + 1]]*/
					std::lower_bound(&p_lambda->i[p_lambda->p[i]],
					&p_lambda->i[p_lambda->p[i + 1]], csi(i + 1))); // t_odo - find the diagonal element and only add up to there
				_ASSERTE(new_edges[i].back() == i); // our structure matrices do have the diagonal
			}
			for(size_t i = 0, n = new_edges.size(); i < n; ++ i) {
				std::sort(new_edges[i].begin(), new_edges[i].end());
				n_new_nnz += new_edges[i].size();
			}
			// make a list of edges that need to be added to the graph

			cs *p_cliques_addition = cs_spalloc(n_new_graph_size,
				n_new_graph_size, n_new_nnz, 0, 0); // alloc a compressed binary matrix
			size_t n_off = 0;
			for(size_t i = 0, n = n_new_graph_size; i < n; ++ i) {
				p_cliques_addition->p[i] = n_off;
				/*std::copy(new_edges[i].begin(), new_edges[i].end(), p_cliques_addition->i + n_off);
				n_off += new_edges[i].size();*/
				for(size_t j = 0, m = new_edges[i].size(); j < m; ++ j, ++ n_off)
					p_cliques_addition->i[n_off] = new_edges[i][j]; // does the same thing as above but avoids the MSVC warning
			}
			p_cliques_addition->p[n_new_graph_size] = n_off;
			cs *p_cliques_addition_t = cs_transpose(p_cliques_addition, 0);
			cs *p_cliques_addition_ = cs_add(p_cliques_addition, p_cliques_addition_t, 1, 1);
			//cs *p_cliques_addition__ = cs_add(p_cliques_addition, p_lambda, 1, 1); // can't add like this
			cs_spfree(p_cliques_addition);
			cs_spfree(p_cliques_addition_t);
			//cs_spfree(p_cliques_addition_);

			double f_graph_extension = dt.f_Time();
			printf("debug: graph extension took %.3f msec\n", f_graph_extension * 1000); fflush(stdout);

			int n_extended_resolution = int((double(n_default_resolution) *
				n_new_graph_size) / n_old_graph_size + .5);
			int n_extended_resolution2 = int((double(n_default_resolution) *
				n_new_graph_size) / n_old_graph_size + .5);

			CDebug::Dump_SparseMatrix_Subsample("clique_extended_graph_sm_SS.tga",
				p_cliques_addition_, 0, n_default_resolution);
			CDebug::Dump_SparseMatrix_Subsample_AA("clique_extended_graph_sm_SS_AA.tga",
				p_cliques_addition_, 0, n_default_resolution2);
			CDebug::Dump_SparseMatrix_Subsample("clique_extended_graph_SS.tga",
				p_cliques_addition_, 0, n_extended_resolution);
			CDebug::Dump_SparseMatrix_Subsample_AA("clique_extended_graph_SS_AA.tga",
				p_cliques_addition_, 0, n_extended_resolution2);
			if(!CDebug::Dump_SparseMatrix_Subsample("clique_extended_graph_diff_SS.tga",
			   p_cliques_addition_, p_lambda, n_extended_resolution) &&
			   !CDebug::Dump_SparseMatrix_Subsample_AA("clique_extended_graph_diff_SS_AA.tga",
			   p_cliques_addition_, p_lambda, n_extended_resolution2)) {
				CDebug::Dump_SparseMatrix_Subsample_AA("clique_extended_graph_diff_sm_SS.tga",
					p_cliques_addition_, p_lambda, n_default_resolution);
				CDebug::Dump_SparseMatrix_Subsample_AA("clique_extended_graph_diff_sm_SS_AA.tga",
					p_cliques_addition_, p_lambda, n_default_resolution2);
			}
			// make a bunch of pictures for a paper

			printf("debug: saved all the images ...\n");
			fflush(stdout);

			// t_odo - will need exact MIS for weighted graphs, to solve toy problems and see how to choose the cliques
			for(int n_pass = 0; n_pass < ((b_try_exact_MICS)? 2 : 1); ++ n_pass) {
				if(n_pass) {
					printf("debug: calculating exact MICS ...\n");
					fflush(stdout);
				}
				dt.Reset();

				std::vector<size_t> MICS =
#ifdef HAVE_IGRAPH
					(n_pass)? CSchurOrdering::t_MIS_igraph(p_cliques_addition_, vertex_weights) :
#endif // HAVE_IGRAPH
					CSchurOrdering::t_MIS_FirstFit(p_cliques_addition_, vertex_weights, true, size_t(-1));

				double f_mics_time = dt.f_Time();

				std::map<size_t, size_t> clique_size_hist;
				size_t n_order_sum = 0, n_weight_sum = 0;
				for(size_t i = 0, n = MICS.size(); i < n; ++ i) {
					++ clique_size_hist[vertex_weights[MICS[i]]]; // debug
					n_weight_sum += vertex_weights[MICS[i]]; // number of elements in the matrix (adjusted for different block sizes)
					n_order_sum += vertex_orders[MICS[i]]; // number of blocks in the matrix (merely the number of variables)
				}
				printf("%sMICS ordering: " PRIsize " pseudovertices (" PRIsize
					" / " PRIsize " in the original graph (" PRIsize
					" / " PRIsize " elemwise), %.2f %%; it took %.3f msec)\n", (!n_pass)? "A" : "", MICS.size(),
					n_order_sum, lambda.n_BlockColumn_Num(), n_weight_sum, lambda.n_Column_Num(),
					double(n_order_sum) / lambda.n_BlockColumn_Num() * 100, f_mics_time * 1000); fflush(stdout);
				{
					size_t n = clique_size_hist.size();
					if(n > 5)
						printf("\t...\n");
					for(std::map<size_t, size_t>::const_iterator p_size_it = clique_size_hist.begin(),
					   p_end_it = clique_size_hist.end(); p_size_it != p_end_it; ++ p_size_it, -- n) {
						if(n <= 5)
							printf("\tcliques of size " PRIsize " elements: " PRIsize "\n", (*p_size_it).first, (*p_size_it).second);
					}
				}

				dt.Reset();

				std::vector<size_t> MICS_ew, ord;
				MICS_ew.reserve(n_order_sum);
				for(size_t i = 0, n = MICS.size(); i < n; ++ i) {
					size_t v = MICS[i];
					if(v < n_old_graph_size)
						MICS_ew.push_back(v); // it is a vertex
					else {
						v -= n_old_graph_size;
						MICS_ew.insert(MICS_ew.end(), cliques[v].begin(), cliques[v].end());
					}
				}

				ord.clear();
				{
					std::vector<size_t> MICS_ew_sorted;
					MICS_ew_sorted = MICS_ew; std::sort(MICS_ew_sorted.begin(),
						MICS_ew_sorted.end()); // need that sorted for the set complement
					CSchurOrdering::Complement_VertexSet(ord, MICS_ew_sorted, lambda.n_BlockColumn_Num());
				}
				const size_t n_cut = ord.size();
				ord.insert(ord.end(), MICS_ew.begin(), MICS_ew.end());
				// calculate full ordering, dependent vertices first

				double f_mics_finalize_time = dt.f_Time();
				printf("debug: ordering composition took %.3f msec\n", f_mics_finalize_time * 1000); fflush(stdout);

				const char *p_s_ordering_name = (n_pass)? "MICS" : "AMICS";

				CMatrixOrdering mord; // actually need an inverse ordering
				CUberBlockMatrix lambda_perm;
				const size_t *oi;
				lambda.Permute_UpperTriangular_To(lambda_perm,
					oi = mord.p_InvertOrdering(&ord.front(), ord.size()), ord.size(), true);

				lambda_perm.Rasterize_Symmetric((std::string("lambda_perm_") + p_s_ordering_name + "5.tga").c_str());
				lambda_perm.Rasterize_Symmetric((std::string("lambda_perm_") + p_s_ordering_name + ".tga").c_str(), 3);
				cs *p_perm_bs;
				if((p_perm_bs = lambda_perm.p_BlockStructure_to_Sparse())) {
					CDebug::Dump_SparseMatrix_Subsample((std::string("lambda_perm_") + p_s_ordering_name +
						"_SS.tga").c_str(), p_perm_bs, 0, 640, true);
					CDebug::Dump_SparseMatrix_Subsample_AA((std::string("lambda_perm_") + p_s_ordering_name +
						"_SS_AA.tga").c_str(), p_perm_bs, 0, 1024, true);
					cs_spfree(p_perm_bs);
				} else
					fprintf(stderr, "error: failed to get the block matrix structure\n");
				// save the matrix

				FILE *p_fw;
				if((p_fw = fopen((std::string("ordering_") + p_s_ordering_name + ".txt").c_str(), "w"))) {
					fprintf(p_fw, PRIsize "\n", ord.size());
					fprintf(p_fw, PRIsize "\n", n_cut);
					for(size_t i = 0, n = ord.size(); i < n; ++ i)
						fprintf(p_fw, (" " PRIsize) + !i, ord[i]);
					fprintf(p_fw, "\n");
					fclose(p_fw);
				} else
					fprintf(stderr, "error: failed to save the ordering\n");
				// save the ordering

				Test_SchurComplement(lambda_perm, n_cut, oi, n_specialization_id,
					p_s_ordering_name, b_skip_perf_test);
			}

			cs_spfree(p_cliques_addition_);

			printf("\n"); fflush(stdout);
		}

		double f_verbose_time = 0;

		std::vector<size_t> mis_ff, mis_imp, mis_imp2, mis_ex; // first-fit, improvement, improvement with quadratic alg, exact
		std::vector<size_t> ord_ff, ord_imp, ord_imp2, ord_ex; // first-fit, improvement, improvement with quadratic alg, exact
		std::vector<size_t> mis_gord, ord_gord;

		struct {
			const char *p_s_long_name, *p_s_name;
			std::vector<size_t> &r_mis, &r_ord;
			void (*p_function)(std::vector<size_t>&, const cs*, const CUberBlockMatrix&);
		} p_algorithm_list[] = {
			{"guided", "gord", mis_gord, ord_gord, &MIS_Guided},

			// guided ordering above

			{"first fit", "ff", mis_ff, ord_ff, &MIS_FF},
			{"improved", "imp", mis_imp, ord_imp, &MIS_Imp},
			{"improved quadratic", "impq", mis_imp2, ord_imp2, &MIS_Imp2},
			{"improved quadratic weighted", "impqw", mis_imp2, ord_imp2, &MIS_Imp2UW},

			// expensive algorithms below

			/*{"exact", "impq", mis_ex, ord_ex, &MIS_Ex},
			{"exact explicit", "impqe", mis_ex, ord_ex, &MIS_ExEx},*/
			{"exact parallel", "impqp", mis_ex, ord_ex, &MIS_ExPar}, // finishes in reasonable time on matrices up to about 100x100
#ifdef HAVE_IGRAPH
			{"exact igraph", "igrex", mis_ex, ord_ex, &MIS_igraph} // can be used for about 50x50 matrices, not much more
#endif // HAVE_IGRAPH
		};
		const size_t n_algorithm_num = sizeof(p_algorithm_list) / sizeof(p_algorithm_list[0]);
#ifdef HAVE_IGRAPH
		const size_t n_expensive_algorithm_num = 2;
#else // HAVE_IGRAPH
		const size_t n_expensive_algorithm_num = 1;
#endif // HAVE_IGRAPH
		_ASSERTE(n_algorithm_num >= n_expensive_algorithm_num);

		for(size_t i = ((b_try_guided)? 0 : 1); i < n_algorithm_num - ((b_skip_expensive_orderings)? n_expensive_algorithm_num : 0); ++ i) {
			std::vector<size_t> &mis = p_algorithm_list[i].r_mis;
			std::vector<size_t> &ord = p_algorithm_list[i].r_ord;
			const char *p_s_ordering_name = p_algorithm_list[i].p_s_name;
			// get destinations and name

			printf("calculating %s ordering\n", p_algorithm_list[i].p_s_long_name); fflush(stdout);
			f_verbose_time += dt.f_Time();

			(*p_algorithm_list[i].p_function)(mis, p_lambda, lambda);
			// calculate the ordering

			double f_mis_time = dt.f_Time();
			printf("\tit took %.3f msec\n", f_mis_time * 1000);
			printf("\tMIS size: " PRIsize " / " PRIsize " (%.2f %%)\n", mis.size(),
				lambda.n_BlockColumn_Num(), mis.size() / double(lambda.n_BlockColumn_Num()) * 100); fflush(stdout);
			{
				ord.clear();
				CSchurOrdering::Complement_VertexSet(ord, mis, lambda.n_BlockColumn_Num());
				const size_t n_cut = ord.size();
				ord.insert(ord.end(), mis.begin(), mis.end());
				// calculate full ordering, dependent vertices first

				CMatrixOrdering mord; // actually need an inverse ordering
				CUberBlockMatrix lambda_perm;
				const size_t *oi;
				lambda.Permute_UpperTriangular_To(lambda_perm,
					oi = mord.p_InvertOrdering(&ord.front(), ord.size()), ord.size(), true);

				lambda_perm.Rasterize_Symmetric((std::string("lambda_perm_") + p_s_ordering_name + "5.tga").c_str());
				lambda_perm.Rasterize_Symmetric((std::string("lambda_perm_") + p_s_ordering_name + ".tga").c_str(), 3);
				cs *p_perm_bs;
				if((p_perm_bs = lambda_perm.p_BlockStructure_to_Sparse())) {
					CDebug::Dump_SparseMatrix_Subsample((std::string("lambda_perm_") + p_s_ordering_name +
						"_SS.tga").c_str(), p_perm_bs, 0, 640, true);
					CDebug::Dump_SparseMatrix_Subsample_AA((std::string("lambda_perm_") + p_s_ordering_name +
						"_SS_AA.tga").c_str(), p_perm_bs, 0, 1024, true);
					cs_spfree(p_perm_bs);
				} else
					fprintf(stderr, "error: failed to get the block matrix structure\n");
				// save the matrix

				FILE *p_fw;
				if((p_fw = fopen((std::string("ordering_") + p_s_ordering_name + ".txt").c_str(), "w"))) {
					fprintf(p_fw, PRIsize "\n", ord.size());
					fprintf(p_fw, PRIsize "\n", n_cut);
					for(size_t i = 0, n = ord.size(); i < n; ++ i)
						fprintf(p_fw, (" " PRIsize) + !i, ord[i]);
					fprintf(p_fw, "\n");
					fclose(p_fw);
				} else
					fprintf(stderr, "error: failed to save the ordering\n");
				// save the ordering

				Test_SchurComplement(lambda_perm, n_cut, oi, n_specialization_id,
					p_s_ordering_name, b_skip_perf_test);
			}

			printf("\n"); fflush(stdout);
		}

		cs_spfree(p_lambda);
		p_lambda = 0;
	} catch(std::bad_alloc&) {
		if(p_lambda)
			cs_spfree(p_lambda);
		fprintf(stderr, "error: got bad_alloc\n");
		return -1;
	}

	// t_odo - calculate FLOPs for each ordering (for each step of forming the SC and solving it)
	// t_odo - write an algorithm for MICS / AMICS

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
	try {
		return throwing_main(n_arg_num, p_arg_list);
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: uncaught std::bad_alloc in main()\n");
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: uncaught std::runtime_error (\'%s\') in main()\n", r_exc.what());
	} catch(std::exception &r_exc) {
		fprintf(stderr, "error: uncaught std::exception (\'%s\') in main()\n", r_exc.what());
	}
}
