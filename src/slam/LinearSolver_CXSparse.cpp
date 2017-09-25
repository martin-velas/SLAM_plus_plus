/*
								+-----------------------------------+
								|                                   |
								|  ***  CSparse linear solver  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|     LinearSolver_CXSparse.cpp     |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/LinearSolver_CXSparse.cpp
 *	@brief linear solver model based on extended version of CSparse
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "csparse/cs.hpp" // need csparse before cxsparse
#include "slam/LinearSolver_CXSparse.h"

/*
 *								=== CLinearSolver_CXSparse ===
 */

CLinearSolver_CXSparse::CLinearSolver_CXSparse()
	:m_p_lambda(0)
#ifdef __CXSPARSE_BLOCKY_LINEAR_SOLVER
	, m_n_workspace_size(0), m_p_workspace_double(0), m_p_workspace_int(0), m_p_symbolic_decomposition(0),
	m_p_block_structure(0)
#endif // __CXSPARSE_BLOCKY_LINEAR_SOLVER
{}

CLinearSolver_CXSparse::CLinearSolver_CXSparse(const CLinearSolver_CXSparse &UNUSED(r_other))
	:m_p_lambda(0)
#ifdef __CXSPARSE_BLOCKY_LINEAR_SOLVER
	, m_n_workspace_size(0), m_p_workspace_double(0), m_p_workspace_int(0), m_p_symbolic_decomposition(0),
	m_p_block_structure(0)
#endif // __CXSPARSE_BLOCKY_LINEAR_SOLVER
{}

void CLinearSolver_CXSparse::Free_Memory()
{
#if defined (__CXSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE) || defined(__CXSPARSE_BLOCKY_LINEAR_SOLVER)
	if(m_p_workspace_double) {
		delete[] m_p_workspace_double;
		m_p_workspace_double = 0;
	}
	if(m_p_workspace_int) {
		delete[] m_p_workspace_int;
		m_p_workspace_int = 0;
	}
	m_n_workspace_size = 0;
#endif // __CXSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE || __CXSPARSE_BLOCKY_LINEAR_SOLVER
#ifdef __CXSPARSE_BLOCKY_LINEAR_SOLVER
	if(m_p_symbolic_decomposition) {
		cx::cxs_sfree(m_p_symbolic_decomposition);
		m_p_symbolic_decomposition = 0;
	}
	if(m_p_block_structure) {
		cs_spfree(m_p_block_structure);
		m_p_block_structure = 0;
	}
	{
		std::vector<_TyPerm> empty;
		m_v_permutation.swap(empty);
	}
#endif // __CXSPARSE_BLOCKY_LINEAR_SOLVER
	if(m_p_lambda) {
		cs_spfree(m_p_lambda);
		m_p_lambda = 0;
	}
}

CLinearSolver_CXSparse::~CLinearSolver_CXSparse()
{
	Free_Memory();
}

CLinearSolver_CXSparse &CLinearSolver_CXSparse::operator =(const CLinearSolver_CXSparse &UNUSED(r_other))
{
	return *this;
}

bool CLinearSolver_CXSparse::Solve_PosDef(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension
#ifdef __CXSPARSE_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CXSPARSE_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CXSPARSE_x64_BUT_SHORT
		throw std::bad_alloc();

	cx::cxs t_cx_lambda;
	t_cx_lambda.nzmax = _TyPerm(m_p_lambda->nzmax);
	t_cx_lambda.m = _TyPerm(m_p_lambda->m);
	t_cx_lambda.n = _TyPerm(m_p_lambda->n);
	t_cx_lambda.p = (_TyPerm*)m_p_lambda->p;
	t_cx_lambda.i = (_TyPerm*)m_p_lambda->i;
	t_cx_lambda.x = m_p_lambda->x;
	t_cx_lambda.nz = _TyPerm(m_p_lambda->nz);
	// convert to CXSparse matrix (note in case some pointers require conversions, CXSparse is not configured correctly)

#ifdef __CXSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
	cx::cxss *p_symbolic_decomposition = cx::cxs_schol(1, &t_cx_lambda);

	if(m_n_workspace_size < size_t(t_cx_lambda.n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(t_cx_lambda.n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new _TyPerm[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	bool b_result = cxs_cholsolsymb(&t_cx_lambda, &r_eta(0), p_symbolic_decomposition,
		m_p_workspace_double, m_p_workspace_int) != 0;

	cx::cxs_sfree(p_symbolic_decomposition);

	return b_result;
#else // __CXSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
	return int(cx::cxs_cholsol(1, &t_cx_lambda, &r_eta(0))) != 0;
	// simple solver, does allocate and free worspace memory every time
#endif // __CXSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
}

#ifdef __CXSPARSE_BLOCKY_LINEAR_SOLVER

bool CLinearSolver_CXSparse::Factorize_PosDef_Blocky(CUberBlockMatrix &r_factor,
	const CUberBlockMatrix &r_lambda, std::vector<size_t> &r_workspace,
	size_t n_dest_row_id /*= 0*/, size_t n_dest_column_id /*= 0*/,
	bool b_upper_factor /*= true*/) // throw(std::bad_alloc)
{
	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
#ifdef __CXSPARSE_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CXSPARSE_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CXSPARSE_x64_BUT_SHORT
		throw std::bad_alloc();

	cx::cxs t_cx_lambda;
	t_cx_lambda.nzmax = _TyPerm(m_p_lambda->nzmax);
	t_cx_lambda.m = _TyPerm(m_p_lambda->m);
	t_cx_lambda.n = _TyPerm(m_p_lambda->n);
	t_cx_lambda.p = (_TyPerm*)m_p_lambda->p;
	t_cx_lambda.i = (_TyPerm*)m_p_lambda->i;
	t_cx_lambda.x = m_p_lambda->x;
	t_cx_lambda.nz = _TyPerm(m_p_lambda->nz);
	// convert to CXSparse matrix (note in case some pointers require conversions, CXSparse is not configured correctly)

	cx::cxss *p_symbolic_decomposition = cx::cxs_schol(0, &t_cx_lambda);

	if(m_n_workspace_size < size_t(t_cx_lambda.n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(t_cx_lambda.n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new _TyPerm[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	cx::cxsn *p_cholesky;
	if(!(p_cholesky = cxs_chol_workspace(&t_cx_lambda, p_symbolic_decomposition,
	   m_p_workspace_int, m_p_workspace_double)))
		return false;
	// calculate numeric Cholesky factorization

	//const csi n_col_num = m_p_lambda->n; // unused
	bool b_result;
	cx::cxs *p_L = p_cholesky->L;
	if(b_upper_factor) {
		cx::cxs *p_transpose = fast_transpose(&t_cx_lambda, p_L, m_p_workspace_int); // todo - cache memory for transpose as well
		m_p_lambda->nzmax = t_cx_lambda.nzmax;
		m_p_lambda->i = (csi*)t_cx_lambda.i;
		m_p_lambda->x = t_cx_lambda.x;
		_ASSERTE(t_cx_lambda.m == _TyPerm(m_p_lambda->m));
		_ASSERTE(t_cx_lambda.n == _TyPerm(m_p_lambda->n)); // m, n and p are untouched, nz is unused in ccs
		// transpose() might realloc, remember how much space we have now

		cs t_factor;
		t_factor.nzmax = p_transpose->nzmax;
		t_factor.m = p_transpose->m;
		t_factor.n = p_transpose->n;
		t_factor.p = (csi*)p_transpose->p;
		t_factor.i = (csi*)p_transpose->i;
		t_factor.x = p_transpose->x;
		t_factor.nz = p_transpose->nz;
		// convert to cs struct

#if !(defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)) || !defined(__CXSPARSE_x64_BUT_SHORT)
		// both csparse and cxsparse have either 32bit or 64bit
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			&t_factor, false, r_workspace);
#else // x64 && !__CXSPARSE_x64_BUT_SHORT
		// csparse have 64 bit and cxsparse have 32 bit
		b_result = r_factor.From_Sparse32(n_dest_row_id, n_dest_column_id,
			&t_factor, false, r_workspace); // not implemented yet
#endif // x64 && !__CXSPARSE_x64_BUT_SHORT

		//cx::cxs_spfree(p_transpose); // do *not* free! it is cached
	} else {
		cs t_factor;
		t_factor.nzmax = p_L->nzmax;
		t_factor.m = p_L->m;
		t_factor.n = p_L->n;
		t_factor.p = (csi*)p_L->p;
		t_factor.i = (csi*)p_L->i;
		t_factor.x = p_L->x;
		t_factor.nz = p_L->nz;
		// convert to cs struct

#if !(defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)) || !defined(__CXSPARSE_x64_BUT_SHORT)
		// both csparse and cxsparse have either 32bit or 64bit
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			&t_factor, false, r_workspace);
#else // x64 && !__CXSPARSE_x64_BUT_SHORT
		// csparse have 64 bit and cxsparse have 32 bit
		b_result = r_factor.From_Sparse32(n_dest_row_id, n_dest_column_id,
			&t_factor, false, r_workspace); // not implemented yet
#endif // x64 && !__CXSPARSE_x64_BUT_SHORT
	}
	// fill L from the calculated factor

	cx::cxs_nfree(p_cholesky);
	cx::cxs_sfree(p_symbolic_decomposition);
	// cleanup

	return b_result;
}

bool CLinearSolver_CXSparse::SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
{
	Clear_SymbolicDecomposition();
	// forget symbolic factorization, if it had one

#ifdef __CXSPARSE_x64_BUT_SHORT
	m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse32(m_p_block_structure);
#else // __CXSPARSE_x64_BUT_SHORT
	m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse(m_p_block_structure);
#endif // __CXSPARSE_x64_BUT_SHORT
	// convert block structure to CSparse

	cx::cxs t_cx_block_structure;
	t_cx_block_structure.nzmax = _TyPerm(m_p_block_structure->nzmax);
	t_cx_block_structure.m = _TyPerm(m_p_block_structure->m);
	t_cx_block_structure.n = _TyPerm(m_p_block_structure->n);
	t_cx_block_structure.p = (_TyPerm*)m_p_block_structure->p;
	t_cx_block_structure.i = (_TyPerm*)m_p_block_structure->i;
	t_cx_block_structure.x = m_p_block_structure->x;
	t_cx_block_structure.nz = _TyPerm(m_p_block_structure->nz);
	// convert to CXSparse matrix (note in case some pointers require conversions, CXSparse is not configured correctly)

	_TyPerm *p_ordering;
	if(!(p_ordering = cx::cxs_amd(1, &t_cx_block_structure)))
		return false;
	// AMD ordering on the block structure

	const size_t n_col_num = r_lambda.n_Column_Num();
	{
		if(m_v_permutation.size() < n_col_num) {
			m_v_permutation.clear(); // avoid copying data on resize
			m_v_permutation.resize(std::max(n_col_num, 2 * m_v_permutation.capacity()));
		}
		size_t n_scalar_offset = 0;
		_TyPerm *p_order_ptr = p_ordering;
		for(int i = 0; i < t_cx_block_structure.n; ++ i, ++ p_order_ptr) {
			const _TyPerm n_order = *p_order_ptr;
			size_t n_block_base = r_lambda.n_BlockColumn_Base(n_order);
			size_t n_block_width = r_lambda.n_BlockColumn_Column_Num(n_order);
			for(size_t j = 0; j < n_block_width; ++ j, ++ n_scalar_offset, ++ n_block_base)
				m_v_permutation[n_scalar_offset] = _TyPerm(n_block_base);
		}
		assert(n_scalar_offset == n_col_num);
	}
	// extend the permutation to the full block matrix (it's permutated blockwise)

	cx::cxs_free(p_ordering);
	// don't need this anymore

#ifdef __CXSPARSE_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CXSPARSE_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CXSPARSE_x64_BUT_SHORT
		throw std::bad_alloc();
	// need lambda as well

	cx::cxs t_cx_lambda;
	t_cx_lambda.nzmax = _TyPerm(m_p_lambda->nzmax);
	t_cx_lambda.m = _TyPerm(m_p_lambda->m);
	t_cx_lambda.n = _TyPerm(m_p_lambda->n);
	t_cx_lambda.p = (_TyPerm*)m_p_lambda->p;
	t_cx_lambda.i = (_TyPerm*)m_p_lambda->i;
	t_cx_lambda.x = m_p_lambda->x;
	t_cx_lambda.nz = _TyPerm(m_p_lambda->nz);
	// convert to CXSparse matrix (note in case some pointers require conversions, CXSparse is not configured correctly)

	if(!(m_p_symbolic_decomposition = (cx::cxss*)cs_calloc(1, sizeof(cx::cxss))))
		throw std::bad_alloc(); // rethrow
	m_p_symbolic_decomposition->pinv = cx::cxs_pinv(&m_v_permutation[0], _TyPerm(n_col_num));
	cx::cxs *p_symbolic_permutation;
	if(!(p_symbolic_permutation = cx::cxs_symperm(&t_cx_lambda, m_p_symbolic_decomposition->pinv, 0)))
		throw std::bad_alloc(); // rethrow
	m_p_symbolic_decomposition->parent = cx::cxs_etree(p_symbolic_permutation, 0);
	_TyPerm *p_post_ordered, *p_counts;
	if(!(p_post_ordered = cx::cxs_post(m_p_symbolic_decomposition->parent, _TyPerm(n_col_num))) ||
	   !(p_counts = cx::cxs_counts(p_symbolic_permutation, m_p_symbolic_decomposition->parent, p_post_ordered, 0))) {
		cx::cxs_spfree(p_symbolic_permutation);
		if(p_post_ordered)
			cx::cxs_free(p_post_ordered);
		throw std::bad_alloc(); // rethrow
	}
	cx::cxs_spfree(p_symbolic_permutation);
	cx::cxs_free(p_post_ordered);
	if(!(m_p_symbolic_decomposition->cp = (_TyPerm*)cs_malloc(n_col_num + 1, sizeof(_TyPerm))))
		throw std::bad_alloc(); // rethrow
	m_p_symbolic_decomposition->unz = m_p_symbolic_decomposition->lnz =
		(double)fast_cumsum(m_p_symbolic_decomposition->cp, p_counts, n_col_num);
	cx::cxs_free(p_counts);
	if(m_p_symbolic_decomposition->lnz < 0) {
		cx::cxs_sfree(m_p_symbolic_decomposition);
		m_p_symbolic_decomposition = 0;
		return false;
	}
	// apply the scalar permutation to finish symbolic factorization

	return true;
}

bool CLinearSolver_CXSparse::Solve_PosDef_Blocky(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

	if(!m_p_symbolic_decomposition) {
		if(!SymbolicDecomposition_Blocky(r_lambda))
			return false;
	} else {
#ifdef __CXSPARSE_x64_BUT_SHORT
		if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CXSPARSE_x64_BUT_SHORT
		if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CXSPARSE_x64_BUT_SHORT
			throw std::bad_alloc();
		// lambda is otherwise calculated in SymbolicDecomposition_Blocky()
	}
	// (re)calculate symbolic factorization

	cx::cxs t_cx_lambda;
	t_cx_lambda.nzmax = _TyPerm(m_p_lambda->nzmax);
	t_cx_lambda.m = _TyPerm(m_p_lambda->m);
	t_cx_lambda.n = _TyPerm(m_p_lambda->n);
	t_cx_lambda.p = (_TyPerm*)m_p_lambda->p;
	t_cx_lambda.i = (_TyPerm*)m_p_lambda->i;
	t_cx_lambda.x = m_p_lambda->x;
	t_cx_lambda.nz = _TyPerm(m_p_lambda->nz);
	// convert to CXSparse matrix (note in case some pointers require conversions, CXSparse is not configured correctly)

	if(m_n_workspace_size < size_t(t_cx_lambda.n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(t_cx_lambda.n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new _TyPerm[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	cx::cxsn *p_cholesky;
	if(!(p_cholesky = cxs_chol_workspace(&t_cx_lambda, m_p_symbolic_decomposition,
	   m_p_workspace_int, m_p_workspace_double)))
		return false;
	// run cholesky

	double *p_b = &r_eta(0), *p_x = m_p_workspace_double;
	const csi n_col_num = m_p_lambda->n;
	cx::cxs_ipvec(m_p_symbolic_decomposition->pinv, p_b, p_x, _TyPerm(n_col_num)); // x = P*b
	cx::cxs_lsolve(p_cholesky->L, p_x); // x = L\x
	cx::cxs_ltsolve(p_cholesky->L, p_x); // x = L'\x
	cx::cxs_pvec(m_p_symbolic_decomposition->pinv, p_x, p_b, _TyPerm(n_col_num)); // b = P'*x
	// don't forget to solve, btw

	cx::cxs_nfree(p_cholesky);
	// cleanup

	return true;
}

#endif // __CXSPARSE_BLOCKY_LINEAR_SOLVER

size_t CLinearSolver_CXSparse::fast_cumsum(_TyPerm *p, const _TyPerm *c, size_t n)
{
	size_t n_sum = 0;
	for(size_t i = 0 ; i < n; ++ i) {
		p[i] = _TyPerm(n_sum);
		_ASSERTE(c[i] >= 0);
		_ASSERTE(n_sum <= SIZE_MAX - c[i]);
		n_sum += c[i];
	}
	p[n] = _TyPerm(n_sum);
	return n_sum;
}

void CLinearSolver_CXSparse::fast_cumsum2(_TyPerm *p, _TyPerm *c, size_t n)
{
	size_t nz = 0;
	for(_TyPerm *e = p + n; p != e; ++ p, ++ c) {
		_ASSERTE(nz <= CMaxIntValue<_TyPerm>::result());
		*p = _TyPerm(nz);
		nz += *c;
		*c = *p;
	}
	_ASSERTE(nz <= CMaxIntValue<_TyPerm>::result());
	*p = _TyPerm(nz);
}

cx::cxs *CLinearSolver_CXSparse::fast_transpose(cx::cxs *C, const cx::cxs *A, _TyPerm *p_workspace)
{
	_TyPerm *Cp, *Ci, n, m, *Ap, *Ai, *w;
	CS_ENTRY *Cx, *Ax;
	_ASSERTE(CS_CSC(A) && A->x && p_workspace);    /* check inputs */
	m = A->m; n = A->n; Ap = A->p; Ai = A->i; Ax = A->x;
	_ASSERTE(C->m == m && C->n == n && C->x);
	if(Ap[n] > C->nzmax && !cx::cxs_sprealloc(C, std::max(Ap[n], C->nzmax * 2)))
		return 0;       /* allocate result */
	w = p_workspace;
	memset(w, 0, m * sizeof(_TyPerm));                      /* get workspace */
	Cp = C->p; Ci = C->i; Cx = C->x;
	for(_TyPerm p = 0; p < Ap[n]; ++ p)
		++ w[Ai[p]];          /* row counts */
	fast_cumsum2(Cp, w, m); // cant use fast cumsum, need copy of the array       /* row pointers */
	for(_TyPerm j = 0; j < n; ++ j) {
		for(_TyPerm p = Ap[j]; p < Ap[j + 1]; ++ p) {
			_TyPerm q = w[Ai[p]];
			++ w[Ai[p]];
			Ci[q] = j; /* place A(i,j) as entry C(j,i) */
			Cx[q] = CS_CONJ(Ax[p]);
		}
	}
	return C;  /* success; do not free w, just return C */
}

CLinearSolver_CXSparse::_TyPerm CLinearSolver_CXSparse::cxs_cholsolsymb(
	const cx::cxs *A, double *b, const cx::cxss *S, double *x, _TyPerm *work)
{
	cx::cxsn *N;
	_TyPerm n, ok;
	if(!CS_CSC(A) || !b || ! S || !x) {
		fprintf(stderr, "%s: No valid input!\n", __FUNCTION__);
		assert(0); // get a backtrace in debug mode
		return (0) ;     /* check inputs */
	}
	n = A->n;
	N = cxs_chol_workspace(A, S, work, x);                    /* numeric Cholesky factorization */
	if(!N) {
		fprintf(stderr, "%s: cholesky failed!\n", __FUNCTION__);
		/*assert(0);*/
	}
	ok = (N != NULL);
	if(ok) {
		cx::cxs_ipvec(S->pinv, b, x, n);   /* x = P*b */
		cx::cxs_lsolve(N->L, x);           /* x = L\x */
		cx::cxs_ltsolve(N->L, x);          /* x = L'\x */
		cx::cxs_pvec(S->pinv, x, b, n);    /* b = P'*x */
	}
	cx::cxs_nfree(N);
	return (ok);
}

cx::cxsn *CLinearSolver_CXSparse::cxs_chol_workspace (const cx::cxs *A, const cx::cxss *S,
	_TyPerm* cin, double *xin)
{
	double d, lki, *Lx, *x, *Cx ;
	_TyPerm top, i, p, k, n, *Li, *Lp, *cp, *pinv, *s, *c, *parent, *Cp, *Ci ;
	cx::cxs *L, *C, *E ;
	cx::cxsn *N ;
	if (!CS_CSC (A) || !S || !S->cp || !S->parent) return (NULL) ;
	n = A->n ;
	N = (cx::cxsn*) cs_calloc (1, sizeof (cx::cxsn)) ;       /* allocate result */
	c = cin ;     /* get int workspace */
	x = xin ;    /* get double workspace */
	cp = S->cp ; pinv = S->pinv ; parent = S->parent ;
	C = pinv ? cx::cxs_symperm (A, pinv, 1) : ((cx::cxs *) A) ;
	E = pinv ? C : NULL ;           /* E is alias for A, or a copy E=A(p,p) */
	if (!N || !c || !x || !C) return (cx::cxs_ndone (N, E, NULL, NULL, 0)) ;
	s = c + n ;
	Cp = C->p ; Ci = C->i ; Cx = C->x ;
	N->L = L = cx::cxs_spalloc (n, n, cp [n], 1, 0) ;    /* allocate result */
	if (!L) return (cx::cxs_ndone (N, E, NULL, NULL, 0)) ;
	Lp = L->p ; Li = L->i ; Lx = L->x ;
	for (k = 0 ; k < n ; k++) Lp [k] = c [k] = cp [k] ;
	for (k = 0 ; k < n ; k++)       /* compute L(k,:) for L*L' = C */
	{
		/* --- Nonzero pattern of L(k,:) ------------------------------------ */
		top = cx::cxs_ereach (C, k, parent, s, c) ;      /* find pattern of L(k,:) */
		x [k] = 0 ;                                 /* x (0:k) is now zero */
		for (p = Cp [k] ; p < Cp [k+1] ; p++)       /* x = full(triu(C(:,k))) */
		{
			if (Ci [p] <= k) x [Ci [p]] = Cx [p] ;
		}
		d = x [k] ;                     /* d = C(k,k) */
		x [k] = 0 ;                     /* clear x for k+1st iteration */
		/* --- Triangular solve --------------------------------------------- */
		for ( ; top < n ; top++)    /* solve L(0:k-1,0:k-1) * x = C(:,k) */
		{
			i = s [top] ;               /* s [top..n-1] is pattern of L(k,:) */
			lki = x [i] / Lx [Lp [i]] ; /* L(k,i) = x (i) / L(i,i) */
			x [i] = 0 ;                 /* clear x for k+1st iteration */
			for (p = Lp [i] + 1 ; p < c [i] ; p++)
			{
				x [Li [p]] -= Lx [p] * lki ;
			}
			d -= lki * lki ;            /* d = d - L(k,i)*L(k,i) */
			p = c [i]++ ;
			Li [p] = k ;                /* store L(k,i) in column i */
			Lx [p] = lki ;
		}
		/* --- Compute L(k,k) ----------------------------------------------- */
		if (d <= 0) return (cx::cxs_ndone (N, E, NULL, NULL, 0)) ; /* not pos def */
		p = c [k]++ ;
		Li [p] = k ;                /* store L(k,k) = sqrt (d) in column k */
		Lx [p] = sqrt (d) ;
	}
	Lp [n] = cp [n] ;               /* finalize L */
	return (cx::cxs_ndone (N, E, NULL, NULL, 1)) ; /* success: free E,s,x; return N */
}

/*
 *								=== ~CLinearSolver_CXSparse ===
 */
