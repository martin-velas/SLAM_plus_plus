/*
								+-----------------------------------+
								|                                   |
								|  ***  CSparse linear solver  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|     LinearSolver_CSparse.cpp      |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/LinearSolver_CSparse.cpp
 *	@brief linear solver model based on CSparse
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 */

#include "slam/LinearSolver_CSparse.h"
#include "slam/Timer.h"

/*
 *								=== CLinearSolver_CSparse ===
 */

CLinearSolver_CSparse::CLinearSolver_CSparse()
	:m_p_lambda(0)
#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER
	, m_n_workspace_size(0), m_p_workspace_double(0), m_p_workspace_int(0), m_p_symbolic_decomposition(0),
	m_p_block_structure(0)
#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER
{
	//m_f_chol = m_f_tran = m_f_fill = 0;
}

CLinearSolver_CSparse::CLinearSolver_CSparse(const CLinearSolver_CSparse &UNUSED(r_other))
	:m_p_lambda(0)
#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER
	, m_n_workspace_size(0), m_p_workspace_double(0), m_p_workspace_int(0), m_p_symbolic_decomposition(0),
	m_p_block_structure(0)
#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER
{
	//m_f_chol = m_f_tran = m_f_fill = 0;
}

void CLinearSolver_CSparse::Free_Memory()
{
#if defined (__CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE) || defined(__CSPARSE_BLOCKY_LINEAR_SOLVER)
	if(m_p_workspace_double) {
		delete[] m_p_workspace_double;
		m_p_workspace_double = 0;
	}
	if(m_p_workspace_int) {
		delete[] m_p_workspace_int;
		m_p_workspace_int = 0;
	}
	m_n_workspace_size = 0;
#endif // __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE || __CSPARSE_BLOCKY_LINEAR_SOLVER
#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER
	if(m_p_symbolic_decomposition) {
		cs_sfree(m_p_symbolic_decomposition);
		m_p_symbolic_decomposition = 0;
	}
	if(m_p_block_structure) {
		cs_spfree(m_p_block_structure);
		m_p_block_structure = 0;
	}
	{
		std::vector<csi> empty;
		m_v_permutation.swap(empty);
	}
#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER
	if(m_p_lambda) {
		cs_spfree(m_p_lambda);
		m_p_lambda = 0;
	}
}

CLinearSolver_CSparse::~CLinearSolver_CSparse()
{
	Free_Memory();
	/*if(m_f_chol + m_f_tran + m_f_fill > 0) {
		printf("CLinearSolver_CSparse() time stats:\n");
		printf("chol: %lf\n", m_f_chol);
		printf("tran: %lf\n", m_f_tran);
		printf("fill: %lf\n", m_f_fill);
	}*/
}

CLinearSolver_CSparse &CLinearSolver_CSparse::operator =(const CLinearSolver_CSparse &UNUSED(r_other))
{
	return *this;
}

bool CLinearSolver_CSparse::Solve_PosDef(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
		throw std::bad_alloc();
#ifdef __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
	css *p_symbolic_decomposition = cs_schol(1, m_p_lambda);

	if(m_n_workspace_size < size_t(m_p_lambda->n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(m_p_lambda->n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new csi[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	bool b_result = cs_cholsolsymb(m_p_lambda, &r_eta(0), p_symbolic_decomposition,
		m_p_workspace_double, m_p_workspace_int) != 0;

	cs_sfree(p_symbolic_decomposition);

	return b_result;
#else // __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
	return int(cs_cholsol(1, m_p_lambda, &r_eta(0))) != 0;
	// simple solver, does allocate and free worspace memory every time
#endif // __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
}

#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER

bool CLinearSolver_CSparse::Factorize_PosDef_Blocky(CUberBlockMatrix &r_factor,
	const CUberBlockMatrix &r_lambda, std::vector<size_t> &r_workspace,
	size_t n_dest_row_id /*= 0*/, size_t n_dest_column_id /*= 0*/, bool b_upper_factor /*= true*/) // throw(std::bad_alloc)
{
	//double f_start_time = m_timer.f_Time();

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
		throw std::bad_alloc();

	css *p_symbolic_decomposition = cs_schol(0, m_p_lambda); // use natural ordering!

	if(m_n_workspace_size < size_t(m_p_lambda->n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(m_p_lambda->n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new csi[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	csn *p_cholesky;
	if(!(p_cholesky = cs_chol_workspace(m_p_lambda, p_symbolic_decomposition,
	   m_p_workspace_int, m_p_workspace_double)))
		return false;
	// calculate numeric Cholesky factorization

	/*double f_cholesky_end = m_timer.f_Time();
	double f_transpose_end;*/

	//const csi n_col_num = m_p_lambda->n; // unused
	bool b_result;
	cs *p_L = p_cholesky->L;
	if(b_upper_factor) {
		cs *p_transpose = fast_transpose(m_p_lambda, p_L, m_p_workspace_int); // t_odo - cache memory for transpose as well
		//f_transpose_end = m_timer.f_Time();
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			p_transpose, false, r_workspace);
		//cs_spfree(p_transpose); // do *not* free! it is cached
	} else {
		//f_transpose_end = f_cholesky_end;
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			p_L, false, r_workspace);
	}
	// fill L from the calculated factor

	cs_nfree(p_cholesky);
	cs_sfree(p_symbolic_decomposition);
	// cleanup

	/*double f_fill_end = m_timer.f_Time();
	m_f_chol += f_cholesky_end - f_start_time;
	m_f_tran += f_transpose_end - f_cholesky_end;
	m_f_fill += f_fill_end - f_transpose_end;*/

	return b_result;
}

bool CLinearSolver_CSparse::Factorize_PosDef_Blocky_Benchmark(double &r_f_time,
	CUberBlockMatrix &r_factor, const CUberBlockMatrix &r_lambda, std::vector<size_t> &r_workspace,
	size_t n_dest_row_id /*= 0*/, size_t n_dest_column_id /*= 0*/, bool b_upper_factor /*= true*/) // throw(std::bad_alloc)
{
	//double f_start_time = m_timer.f_Time();

	CDeltaTimer dt;

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
		throw std::bad_alloc();

	dt.Reset();
	// reset timer

	css *p_symbolic_decomposition = cs_schol(0, m_p_lambda); // use natural ordering!

	if(m_n_workspace_size < size_t(m_p_lambda->n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(m_p_lambda->n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new csi[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	csn *p_cholesky;
	if(!(p_cholesky = cs_chol_workspace(m_p_lambda, p_symbolic_decomposition,
	   m_p_workspace_int, m_p_workspace_double)))
		return false;
	// calculate numeric Cholesky factorization

	r_f_time = dt.f_Time();
	// sample the timer

	/*double f_cholesky_end = m_timer.f_Time();
	double f_transpose_end;*/

	//const csi n_col_num = m_p_lambda->n; // unused
	bool b_result;
	cs *p_L = p_cholesky->L;
	if(b_upper_factor) {
		cs *p_transpose = fast_transpose(m_p_lambda, p_L, m_p_workspace_int); // t_odo - cache memory for transpose as well
		//f_transpose_end = m_timer.f_Time();
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			p_transpose, false, r_workspace);
		//cs_spfree(p_transpose); // do *not* free! it is cached
	} else {
		//f_transpose_end = f_cholesky_end;
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			p_L, false, r_workspace);
	}
	// fill L from the calculated factor

	cs_nfree(p_cholesky);
	cs_sfree(p_symbolic_decomposition);
	// cleanup

	/*double f_fill_end = m_timer.f_Time();
	m_f_chol += f_cholesky_end - f_start_time;
	m_f_tran += f_transpose_end - f_cholesky_end;
	m_f_fill += f_fill_end - f_transpose_end;*/

	return b_result;
}

bool CLinearSolver_CSparse::SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
{
	Clear_SymbolicDecomposition();
	// forget symbolic factorization, if it had one

	m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse(m_p_block_structure);
	// convert block structure to CSparse

	csi *p_ordering;
	if(!(p_ordering = cs_amd(1, m_p_block_structure)))
		return false;
	// AMD ordering on the block structure

	const size_t n_col_num = r_lambda.n_Column_Num();
	{
		if(m_v_permutation.size() < n_col_num) {
			m_v_permutation.clear(); // avoid copying data on resize
			m_v_permutation.resize(std::max(n_col_num, 2 * m_v_permutation.capacity()));
		}
		size_t n_scalar_offset = 0;
		csi *p_order_ptr = p_ordering;
		for(int i = 0; i < m_p_block_structure->n; ++ i, ++ p_order_ptr) {
			const csi n_order = *p_order_ptr;
			size_t n_block_base = r_lambda.n_BlockColumn_Base(n_order);
			size_t n_block_width = r_lambda.n_BlockColumn_Column_Num(n_order);
			for(size_t j = 0; j < n_block_width; ++ j, ++ n_scalar_offset, ++ n_block_base)
				m_v_permutation[n_scalar_offset] = n_block_base;
		}
		assert(n_scalar_offset == n_col_num);
	}
	// extend the permutation to the full block matrix (it's permutated blockwise)

	cs_free(p_ordering);
	// don't need this anymore

	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
		throw std::bad_alloc();
	// need lambda as well

	if(!(m_p_symbolic_decomposition = (css*)cs_calloc(1, sizeof(css))))
		throw std::bad_alloc(); // rethrow
	m_p_symbolic_decomposition->pinv = cs_pinv(&m_v_permutation[0], n_col_num);
	cs *p_symbolic_permutation;
	if(!(p_symbolic_permutation = cs_symperm(m_p_lambda, m_p_symbolic_decomposition->pinv, 0)))
		throw std::bad_alloc(); // rethrow
	m_p_symbolic_decomposition->parent = cs_etree(p_symbolic_permutation, 0);
	csi *p_post_ordered, *p_counts;
	if(!(p_post_ordered = cs_post(m_p_symbolic_decomposition->parent, n_col_num)) ||
	   !(p_counts = cs_counts(p_symbolic_permutation, m_p_symbolic_decomposition->parent, p_post_ordered, 0))) {
		cs_spfree(p_symbolic_permutation);
		if(p_post_ordered)
			cs_free(p_post_ordered);
		throw std::bad_alloc(); // rethrow
	}
	cs_spfree(p_symbolic_permutation);
	cs_free(p_post_ordered);
	if(!(m_p_symbolic_decomposition->cp = (csi*)cs_malloc(n_col_num + 1, sizeof(csi))))
		throw std::bad_alloc(); // rethrow
	m_p_symbolic_decomposition->unz = m_p_symbolic_decomposition->lnz =
		(double)fast_cumsum(m_p_symbolic_decomposition->cp, p_counts, n_col_num);
	cs_free(p_counts);
	if(m_p_symbolic_decomposition->lnz < 0) {
		cs_sfree(m_p_symbolic_decomposition);
		m_p_symbolic_decomposition = 0;
		return false;
	}
	// apply the scalar permutation to finish symbolic factorization

	return true;
}

bool CLinearSolver_CSparse::Solve_PosDef_Blocky(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

	if(!m_p_symbolic_decomposition) {
		if(!SymbolicDecomposition_Blocky(r_lambda))
			return false;
	} else {
		if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
			throw std::bad_alloc();
		// lambda is otherwise calculated in SymbolicDecomposition_Blocky()
	}
	// (re)calculate symbolic factorization

	if(m_n_workspace_size < size_t(m_p_lambda->n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(m_p_lambda->n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
		if(m_p_workspace_int)
			delete[] m_p_workspace_int;
		m_p_workspace_int = new csi[2 * m_n_workspace_size];
	}
	// re-allocate the temporary workspace for cholesky

	csn *p_cholesky;
	if(!(p_cholesky = cs_chol_workspace(m_p_lambda, m_p_symbolic_decomposition,
	   m_p_workspace_int, m_p_workspace_double)))
		return false;
	// run cholesky

	double *p_b = &r_eta(0), *p_x = m_p_workspace_double;
	const csi n_col_num = m_p_lambda->n;
	cs_ipvec(m_p_symbolic_decomposition->pinv, p_b, p_x, n_col_num); // x = P*b
	cs_lsolve(p_cholesky->L, p_x); // x = L\x
	cs_ltsolve(p_cholesky->L, p_x); // x = L'\x
	cs_pvec(m_p_symbolic_decomposition->pinv, p_x, p_b, n_col_num); // b = P'*x
	// don't forget to solve, btw

	cs_nfree(p_cholesky);
	// cleanup

	return true;
}

#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER

size_t CLinearSolver_CSparse::fast_cumsum(csi *p, const csi *c, csi n)
{
	size_t n_sum = 0;
	for(csi i = 0 ; i < n; ++ i) {
		p[i] = csi(n_sum);
		_ASSERTE(c[i] >= 0);
		_ASSERTE(n_sum <= SIZE_MAX - c[i]);
		n_sum += c[i];
	}
	p[n] = csi(n_sum);
	return n_sum;
}

void CLinearSolver_CSparse::fast_cumsum2(csi *p, csi *c, size_t n)
{
	size_t nz = 0;
	for(csi *e = p + n; p != e; ++ p, ++ c) {
		*p = nz;
		nz += *c;
		*c = *p;
	}
	*p = nz;
}

cs *CLinearSolver_CSparse::fast_transpose(cs *C, const cs *A, csi *p_workspace)
{
	csi *Cp, *Ci, n, m, *Ap, *Ai, *w;
	double *Cx, *Ax;
	_ASSERTE(CS_CSC(A) && A->x && p_workspace);    /* check inputs */
	m = A->m; n = A->n; Ap = A->p; Ai = A->i; Ax = A->x;
	_ASSERTE(C->m == m && C->n == n && C->x);
	if(Ap[n] > C->nzmax && !cs_sprealloc(C, std::max(Ap[n], C->nzmax * 2)))
		return 0;       /* allocate result */
	w = p_workspace;
	memset(w, 0, m * sizeof(csi));                      /* get workspace */
	Cp = C->p; Ci = C->i; Cx = C->x;
	for(csi p = 0; p < Ap[n]; ++ p)
		++ w[Ai[p]];          /* row counts */
	fast_cumsum2(Cp, w, m); // cant use fast cumsum, need copy of the array       /* row pointers */
	for(csi j = 0; j < n; ++ j) {
		for(csi p = Ap[j]; p < Ap[j + 1]; ++ p) {
			csi q = w[Ai[p]];
			++ w[Ai[p]];
			Ci[q] = j; /* place A(i,j) as entry C(j,i) */
			Cx[q] = Ax[p];
		}
	}
	return C;  /* success; do not free w, just return C */
}

csi CLinearSolver_CSparse::cs_cholsolsymb(const cs *A, double *b, const css *S, double *x, csi *work)
{
	csn *N;
	csi n, ok;
	if(!CS_CSC(A) || !b || ! S || !x) {
		fprintf(stderr, "%s: No valid input!\n", __FUNCTION__);
		assert(0); // get a backtrace in debug mode
		return (0) ;     /* check inputs */
	}
	n = A->n;
	N = cs_chol_workspace(A, S, work, x);                    /* numeric Cholesky factorization */
	if(!N) {
		fprintf(stderr, "%s: cholesky failed!\n", __FUNCTION__);
		/*assert(0);*/
	}
	ok = (N != NULL);
	if(ok) {
		cs_ipvec(S->pinv, b, x, n);   /* x = P*b */
		cs_lsolve(N->L, x);           /* x = L\x */
		cs_ltsolve(N->L, x);          /* x = L'\x */
		cs_pvec(S->pinv, x, b, n);    /* b = P'*x */
	}
	cs_nfree(N);
	return (ok);
}

csn *CLinearSolver_CSparse::cs_chol_workspace (const cs *A, const css *S, csi* cin, double* xin)
{
	double d, lki, *Lx, *x, *Cx ;
	csi top, i, p, k, n, *Li, *Lp, *cp, *pinv, *s, *c, *parent, *Cp, *Ci ;
	cs *L, *C, *E ;
	csn *N ;
	if (!CS_CSC (A) || !S || !S->cp || !S->parent) return (NULL) ;
	n = A->n ;
	N = (csn*) cs_calloc (1, sizeof (csn)) ;       /* allocate result */
	c = cin ;     /* get int workspace */
	x = xin ;    /* get double workspace */
	cp = S->cp ; pinv = S->pinv ; parent = S->parent ;
	C = pinv ? cs_symperm (A, pinv, 1) : ((cs *) A) ;
	E = pinv ? C : NULL ;           /* E is alias for A, or a copy E=A(p,p) */
	if (!N || !c || !x || !C) return (cs_ndone (N, E, NULL, NULL, 0)) ;
	s = c + n ;
	Cp = C->p ; Ci = C->i ; Cx = C->x ;
	N->L = L = cs_spalloc (n, n, cp [n], 1, 0) ;    /* allocate result */
	if (!L) return (cs_ndone (N, E, NULL, NULL, 0)) ;
	Lp = L->p ; Li = L->i ; Lx = L->x ;
	for (k = 0 ; k < n ; k++) Lp [k] = c [k] = cp [k] ;
	for (k = 0 ; k < n ; k++)       /* compute L(k,:) for L*L' = C */
	{
		/* --- Nonzero pattern of L(k,:) ------------------------------------ */
		top = cs_ereach (C, k, parent, s, c) ;      /* find pattern of L(k,:) */
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
		if (d <= 0) return (cs_ndone (N, E, NULL, NULL, 0)) ; /* not pos def */
		p = c [k]++ ;
		Li [p] = k ;                /* store L(k,k) = sqrt (d) in column k */
		Lx [p] = sqrt (d) ;
	}
	Lp [n] = cp [n] ;               /* finalize L */
	return (cs_ndone (N, E, NULL, NULL, 1)) ; /* success: free E,s,x; return N */
}

/*
 *								=== ~CLinearSolver_CSparse ===
 */
