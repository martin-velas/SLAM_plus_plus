/*
								+-----------------------------------+
								|                                   |
								|  ***  CHOLMOD linear solver  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|     LinearSolver_CholMod.cpp      |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/LinearSolver_CholMod.cpp
 *	@brief linear solver model based on CHOLMOD
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 *
 *	@note This is faster than CSparse in x86, but slower in x64
 *		(possibly due to DLONG; would have to somehow use int).
 *	@note Using __CHOLMOD_SHORT, this is faster in x64 on windows,
 *		but actually slower on linux. Tried using raw arrays
 *		for workspace instead of std::vectors, but still nowhere
 *		near csparse.
 *
 */

#include "slam/LinearSolver_CholMod.h"
#include "slam/Timer.h"

/*
 *								=== CLinearSolver_CholMod ===
 */

CLinearSolver_CholMod::CLinearSolver_CholMod()
	:m_p_lambda(0), m_p_factor(0), m_p_block_structure(0)
#ifdef __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	, m_n_workspace_size(0), m_p_workspace_double(0)
#endif // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
{
#ifdef __CHOLMOD_x64
	cholmod_l_start(&m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_start(&m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// initialize cholmod!

	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = default_ordering_Method;
	m_t_cholmod_common.method[1].ordering = m_t_cholmod_common.method[0].ordering;
	m_t_cholmod_common.postorder = 1;
	//m_t_cholmod_common.postorder = 0;
	m_t_cholmod_common.supernodal = default_analysis_Type;
	// setup ordering strategy

	memset(&m_t_lambda, 0, sizeof(cholmod_sparse));
	m_t_lambda.stype = 1; // upper triangular block only (values in lower part are ignore, may ommit their calculation)
	m_t_lambda.itype = CHOLMOD_INT;
	m_t_lambda.xtype = CHOLMOD_REAL;
	m_t_lambda.dtype = CHOLMOD_DOUBLE;
	m_t_lambda.sorted = 1;
	m_t_lambda.packed = 1;
	// sets cholmod structure

#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER
	memset(&m_t_block_structure, 0, sizeof(cholmod_sparse));
	m_t_block_structure.nz = 0;
	m_t_block_structure.x = 0;
	m_t_block_structure.z = 0;
	m_t_block_structure.stype = 1;
	m_t_block_structure.xtype = CHOLMOD_PATTERN;
	m_t_block_structure.itype = CHOLMOD_INT;
	m_t_block_structure.dtype = CHOLMOD_DOUBLE;
	m_t_block_structure.sorted = 1;
	m_t_block_structure.packed = 1;
	// sets cholmod structure
#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

	/*m_f_tosparse_time = 0;
	m_f_analysis_time = 0;
	m_f_factor_time = 0;
	m_f_solve_time = 0;*/
}

CLinearSolver_CholMod::CLinearSolver_CholMod(int n_analysis_type /*= default_analysis_Type*/,
	int n_ordering_method /*= default_ordering_Method*/)
	:m_p_lambda(0), m_p_factor(0), m_p_block_structure(0)
#ifdef __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	, m_n_workspace_size(0), m_p_workspace_double(0)
#endif // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
{
#ifdef __CHOLMOD_x64
	cholmod_l_start(&m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_start(&m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// initialize cholmod!

	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = n_ordering_method;
	m_t_cholmod_common.method[1].ordering = m_t_cholmod_common.method[0].ordering;
	m_t_cholmod_common.postorder = 1;
	//m_t_cholmod_common.postorder = 0;
	m_t_cholmod_common.supernodal = n_analysis_type;
	// setup ordering strategy

	memset(&m_t_lambda, 0, sizeof(cholmod_sparse));
	m_t_lambda.stype = 1; // upper triangular block only (values in lower part are ignore, may ommit their calculation)
	m_t_lambda.itype = CHOLMOD_INT;
	m_t_lambda.xtype = CHOLMOD_REAL;
	m_t_lambda.dtype = CHOLMOD_DOUBLE;
	m_t_lambda.sorted = 1;
	m_t_lambda.packed = 1;
	// sets cholmod structure

#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER
	memset(&m_t_block_structure, 0, sizeof(cholmod_sparse));
	m_t_block_structure.nz = 0;
	m_t_block_structure.x = 0;
	m_t_block_structure.z = 0;
	m_t_block_structure.stype = 1;
	m_t_block_structure.xtype = CHOLMOD_PATTERN;
	m_t_block_structure.itype = CHOLMOD_INT;
	m_t_block_structure.dtype = CHOLMOD_DOUBLE;
	m_t_block_structure.sorted = 1;
	m_t_block_structure.packed = 1;
	// sets cholmod structure
#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

	/*m_f_tosparse_time = 0;
	m_f_analysis_time = 0;
	m_f_factor_time = 0;
	m_f_solve_time = 0;*/
}

CLinearSolver_CholMod::CLinearSolver_CholMod(const CLinearSolver_CholMod &r_other)
	:m_p_lambda(0), m_p_factor(0), m_p_block_structure(0)
#ifdef __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	, m_n_workspace_size(0), m_p_workspace_double(0)
#endif // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
{
#ifdef __CHOLMOD_x64
	cholmod_l_start(&m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_start(&m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// initialize cholmod!

	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = r_other.m_t_cholmod_common.method[0].ordering;
	m_t_cholmod_common.method[1].ordering = m_t_cholmod_common.method[0].ordering; // backup
	m_t_cholmod_common.postorder = 1;
	m_t_cholmod_common.supernodal = m_t_cholmod_common.supernodal;
	// setup ordering strategy (copy config from r_other)

	memset(&m_t_lambda, 0, sizeof(cholmod_sparse));
	m_t_lambda.stype = 1; // upper triangular block only (values in lower part are ignore, may ommit their calculation)
#ifdef __CHOLMOD_x64
	m_t_lambda.itype = CHOLMOD_LONG;
#else // __CHOLMOD_x64
	m_t_lambda.itype = CHOLMOD_INT;
#endif // __CHOLMOD_x64
	m_t_lambda.xtype = CHOLMOD_REAL;
	m_t_lambda.dtype = CHOLMOD_DOUBLE;
	m_t_lambda.sorted = 1;
	m_t_lambda.packed = 1;
	// sets cholmod structure

#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER
	memset(&m_t_block_structure, 0, sizeof(cholmod_sparse));
	m_t_block_structure.nz = 0;
	m_t_block_structure.x = 0;
	m_t_block_structure.z = 0;
	m_t_block_structure.stype = 1;
	m_t_block_structure.xtype = CHOLMOD_PATTERN;
#ifdef __CHOLMOD_x64
	m_t_block_structure.itype = CHOLMOD_LONG;
#else // __CHOLMOD_x64
	m_t_block_structure.itype = CHOLMOD_INT;
#endif // __CHOLMOD_x64
	m_t_block_structure.dtype = CHOLMOD_DOUBLE;
	m_t_block_structure.sorted = 1;
	m_t_block_structure.packed = 1;
	// sets cholmod structure
#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

	/*m_f_tosparse_time = 0;
	m_f_analysis_time = 0;
	m_f_factor_time = 0;
	m_f_solve_time = 0;*/
}

void CLinearSolver_CholMod::Free_Memory()
{
	{
		std::vector<_TyPerm> empty;
		m_p_scalar_permutation.swap(empty);
	}
	{
		std::vector<_TyPerm> empty;
		m_p_block_permutation.swap(empty);
	}
#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER
	if(m_p_block_structure) {
		cs_spfree(m_p_block_structure);
		m_p_block_structure = 0;
	}
	if(m_p_factor) {
#ifdef __CHOLMOD_x64
		cholmod_l_free_factor(&m_p_factor, &m_t_cholmod_common); // t_odo - dispose of m_p_factor
#else // __CHOLMOD_x64
		cholmod_free_factor(&m_p_factor, &m_t_cholmod_common); // t_odo - dispose of m_p_factor
#endif // __CHOLMOD_x64
		m_p_factor = 0;
	}
#ifdef __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	{
		std::vector<_TyPerm> empty;
		m_p_inverse_scalar_permutation.swap(empty);
	}
	if(m_p_workspace_double) {
		delete[] m_p_workspace_double;
		m_p_workspace_double = 0;
	}
	m_n_workspace_size = 0;
#endif // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER
	if(m_p_lambda) {
		cs_spfree(m_p_lambda);
		m_p_lambda = 0;
	}
}

CLinearSolver_CholMod::~CLinearSolver_CholMod()
{
	Free_Memory();
	// delete all aux matrices and buffers

#ifdef __CHOLMOD_x64
	cholmod_l_finish(&m_t_cholmod_common); // shutdown cholmod!
#else // __CHOLMOD_x64
	cholmod_finish(&m_t_cholmod_common); // shutdown cholmod!
#endif // __CHOLMOD_x64

	/*if(m_f_tosparse_time > 0) {
		printf("cholmod to-sparse time: %.2f sec\n", m_f_tosparse_time);
		printf("cholmod  analysis time: %.2f sec\n", m_f_analysis_time);
		printf("cholmod    factor time: %.2f sec\n", m_f_factor_time);
		printf("cholmod     solve time: %.2f sec\n", m_f_solve_time);
	}*/
	// dump timing stats
}

CLinearSolver_CholMod &CLinearSolver_CholMod::operator =(const CLinearSolver_CholMod &r_other)
{
	m_t_cholmod_common.method[0].ordering = r_other.m_t_cholmod_common.method[0].ordering;
	m_t_cholmod_common.method[1].ordering = m_t_cholmod_common.method[0].ordering; // backup
	m_t_cholmod_common.supernodal = m_t_cholmod_common.supernodal;
	// copy settings

	return *this;
}

bool CLinearSolver_CholMod::Solve_PosDef(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	//double f_to_sparse_start = m_timer.f_Time();

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension
#ifdef __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CHOLMOD_x64_BUT_SHORT
		throw std::bad_alloc();
	// convert to csparse matrix

	m_t_lambda.p = m_p_lambda->p;
	m_t_lambda.nzmax = m_p_lambda->nzmax;
	m_t_lambda.nrow = m_p_lambda->m;
	m_t_lambda.ncol = m_p_lambda->n;
	m_t_lambda.i = m_p_lambda->i;
	m_t_lambda.x = m_p_lambda->x;
	// fills cholsol matrix (just reref arrays)

	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = m_t_cholmod_common.method[1].ordering; // get from backup
	m_t_cholmod_common.postorder = 1;
	// set ordering up (blocky solver rewrites it)

	//double f_analyze_start = m_timer.f_Time();

	cholmod_factor *p_cholmod_factor;
#ifdef __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_l_analyze(&m_t_lambda, &m_t_cholmod_common)))
#else // __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_analyze(&m_t_lambda, &m_t_cholmod_common)))
#endif // __CHOLMOD_x64
		return false;
	// symbolic factorization

	//double f_factorize_start = m_timer.f_Time();

#ifdef __CHOLMOD_x64
	cholmod_l_factorize(&m_t_lambda, p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_factorize(&m_t_lambda, p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	if(m_t_cholmod_common.status == CHOLMOD_NOT_POSDEF) {
#ifdef __CHOLMOD_x64
		cholmod_l_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
		cholmod_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
		return false; // not positive definite
	}
	// factorize

	//double f_solve_start = m_timer.f_Time();

	cholmod_dense t_b_cholmod;
	t_b_cholmod.nrow = t_b_cholmod.d = m_t_lambda.nrow;
	t_b_cholmod.ncol = 1;
	t_b_cholmod.x = &r_eta(0);
	t_b_cholmod.xtype = CHOLMOD_REAL;
	t_b_cholmod.dtype = CHOLMOD_DOUBLE;
	// set up dense vector with eta for calling cholmod

#ifdef __CHOLMOD_x64
	cholmod_dense *p_x_cholmod = cholmod_l_solve(CHOLMOD_A, p_cholmod_factor, &t_b_cholmod, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_dense *p_x_cholmod = cholmod_solve(CHOLMOD_A, p_cholmod_factor, &t_b_cholmod, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	_ASSERTE(r_eta.rows() == t_b_cholmod.nrow); // it's symmetric, dimension of solution equals dimension of right side
	// call cholesky

	memcpy(&r_eta(0), p_x_cholmod->x, sizeof(double) * t_b_cholmod.nrow);
	// copy back to src array

#ifdef __CHOLMOD_x64
	cholmod_l_free_dense(&p_x_cholmod, &m_t_cholmod_common);
	cholmod_l_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_free_dense(&p_x_cholmod, &m_t_cholmod_common);
	cholmod_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// cleanup

	/*double f_solve_end = m_timer.f_Time();

	m_f_tosparse_time += f_analyze_start - f_to_sparse_start;
	m_f_analysis_time += f_factorize_start - f_analyze_start;
	m_f_factor_time += f_solve_start - f_factorize_start;
	m_f_solve_time += f_solve_end - f_solve_start;*/

	return true;
}

#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER

bool CLinearSolver_CholMod::Factorize_PosDef_Blocky(CUberBlockMatrix &r_factor,
	const CUberBlockMatrix &r_lambda, std::vector<size_t> &r_workspace, size_t n_dest_row_id /*= 0*/,
	size_t n_dest_column_id /*= 0*/, bool b_upper_factor /*= true*/) // throw(std::bad_alloc)
{
	/*return CLinearSolver_CSparse().Factorize_PosDef_Blocky(r_factor, r_lambda, r_workspace,
		n_dest_row_id, n_dest_column_id, b_upper_factor);*/
	// debug - reference solution

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
#ifdef __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CHOLMOD_x64_BUT_SHORT
		throw std::bad_alloc();
	// convert to csparse matrix

	m_t_lambda.p = m_p_lambda->p;
	m_t_lambda.nzmax = m_p_lambda->nzmax;
	m_t_lambda.nrow = m_p_lambda->m;
	m_t_lambda.ncol = m_p_lambda->n;
	m_t_lambda.i = m_p_lambda->i;
	m_t_lambda.x = m_p_lambda->x;
	// fills cholsol matrix (just reref arrays)

	m_t_cholmod_common.prefer_upper = b_upper_factor;
#if 1
	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = CHOLMOD_NATURAL; // no ordering
	m_t_cholmod_common.postorder = 0; // !!
	// set ordering up (blocky solver rewrites it)

	cholmod_factor *p_cholmod_factor;
#ifdef __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_l_analyze(&m_t_lambda, &m_t_cholmod_common)))
#else // __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_analyze(&m_t_lambda, &m_t_cholmod_common)))
#endif // __CHOLMOD_x64
		return false;
#else // 1
	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = CHOLMOD_GIVEN;//CHOLMOD_NATURAL; // no ordering
	m_t_cholmod_common.postorder = 0;
	// set ordering up (blocky solver rewrites it)

	{
		const size_t n = r_lambda.n_Column_Num();
		if(m_p_scalar_permutation.size() < n) {
			m_p_scalar_permutation.clear();
			m_p_scalar_permutation.resize(std::max(n, 2 * m_p_scalar_permutation.capacity()));
		}
		for(size_t i = 0; i < n; ++ i)
			m_p_scalar_permutation[i] = i;
	}
	// make identity permutation

	cholmod_factor *p_cholmod_factor;
#ifdef __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_l_analyze_p(&m_t_lambda,
	   &m_p_scalar_permutation[0], NULL, 0, &m_t_cholmod_common)))
#else // __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_analyze_p(&m_t_lambda,
	   &m_p_scalar_permutation[0], NULL, 0, &m_t_cholmod_common)))
#endif // __CHOLMOD_x64
		return false;
#endif // 1
	// symbolic factorization

#ifdef __CHOLMOD_x64
	cholmod_l_factorize(&m_t_lambda, p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_factorize(&m_t_lambda, p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	if(m_t_cholmod_common.status == CHOLMOD_NOT_POSDEF) {
#ifdef __CHOLMOD_x64
		cholmod_l_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
		cholmod_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
		return false; // not positive definite
	}
	// factorize

#ifdef __CHOLMOD_x64
	if(!cholmod_l_change_factor(CHOLMOD_REAL, 1, 0/*p_cholmod_factor->is_super*/, 1/*0*/, 1/*p_cholmod_factor->is_monotonic*/, p_cholmod_factor, &m_t_cholmod_common))
#else // __CHOLMOD_x64
	if(!cholmod_change_factor(CHOLMOD_REAL, 1, 0/*p_cholmod_factor->is_super*/, 1/*0*/, 1/*p_cholmod_factor->is_monotonic*/, p_cholmod_factor, &m_t_cholmod_common))
#endif // __CHOLMOD_x64
		return false;
	_ASSERTE(p_cholmod_factor->is_ll && !p_cholmod_factor->is_super && p_cholmod_factor->is_monotonic); // makes sure it comes in correct format
	// convert the factorization to LL, simplical, packed/*or not*/, monotonic

	cs L;
	L.nzmax = p_cholmod_factor->nzmax;
	L.nz = -1;
	L.p = (csi*)p_cholmod_factor->p;
	L.i = (csi*)p_cholmod_factor->i;
	L.x = (double*)p_cholmod_factor->x;
	L.m = p_cholmod_factor->n;
	L.n = p_cholmod_factor->n;
	// get L matrix from the factor

#if 0
	_ASSERTE(p_cholmod_factor->ordering == CHOLMOD_NATURAL ||
		p_cholmod_factor->ordering == CHOLMOD_GIVEN);
	cs *p_L;
	{
		const cs *A = m_p_lambda;
		css *S;
		if(!(S = cs_schol(0, A))) // do use symbolic something! (keeps L sparse)
			return false;
		// ordering and symbolic analysis for a Cholesky factorization

		csn *N;
		if(!(N = cs_chol(A, S))) {// @todo - use CLinearSolver_CSparse to do that, it caches workspace and stuff ...
			cs_sfree(S);
			return false;
		}

		p_L = N->L;//cs_symperm(N->L, N->pinv, 1);

		cs_sfree(S);

		cs_spfree(N->U); // t_odo - remove if possible
		cs_free(N->pinv);
		cs_free(N->B);
		//cs_free(N->L);
		// note that the above pointers are most likely null (not used for Cholesky, only by QR or LU)
	}

	cs *p_diff = cs_add(p_L, &L, -1, 1);
	double f_diff = cs_norm(p_diff);
	if(f_diff > 1e-5) {
		CDebug::Dump_SparseMatrix("L_csparse.tga", p_L);
		CDebug::Dump_SparseMatrix("L_cholmod.tga", &L);
		CDebug::Dump_SparseMatrix("L_diff.tga", p_diff);
	}
	cs_spfree(p_L);
	cs_spfree(p_diff);
#endif // 0
	// debug code to make sure no ordering is used

	_ASSERTE(sizeof(_TyCSIntType) == sizeof(_TyPerm));
	bool b_result;
	if(b_upper_factor) {
		_ASSERTE(L.n >= 0 && L.n <= SIZE_MAX);
		if(m_p_scalar_permutation.size() < size_t(L.m)) {
			m_p_scalar_permutation.clear();
			m_p_scalar_permutation.resize(std::max(size_t(L.m), 2 * m_p_scalar_permutation.capacity()));
		}
		// reuse storage

		cs *p_transpose = fast_transpose(&L, (_TyCSIntType*)&m_p_scalar_permutation[0]);
		// this calculates transpose with 32 or 64bit integers, based on target machine and __CHOLMOD_x64_BUT_SHORT

#ifdef __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse32(n_dest_row_id, n_dest_column_id,
			p_transpose, false, r_workspace);
#else // __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			p_transpose, false, r_workspace);
#endif // __CHOLMOD_x64_BUT_SHORT

		cs_spfree(p_transpose);
	} else {
#ifdef __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse32(n_dest_row_id, n_dest_column_id,
			&L, false, r_workspace);
#else // __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			&L, false, r_workspace);
#endif // __CHOLMOD_x64_BUT_SHORT
	}

#ifdef __CHOLMOD_x64
	cholmod_l_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// cleanup

	return b_result;
}

bool CLinearSolver_CholMod::Factorize_PosDef_Blocky_Benchmark(double &r_f_time, CUberBlockMatrix &r_factor,
	const CUberBlockMatrix &r_lambda, std::vector<size_t> &r_workspace, size_t n_dest_row_id /*= 0*/,
	size_t n_dest_column_id /*= 0*/, bool b_upper_factor /*= true*/) // throw(std::bad_alloc)
{
	/*return CLinearSolver_CSparse().Factorize_PosDef_Blocky(r_factor, r_lambda, r_workspace,
		n_dest_row_id, n_dest_column_id, b_upper_factor);*/
	// debug - reference solution

	CDeltaTimer dt;

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
#ifdef __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CHOLMOD_x64_BUT_SHORT
		throw std::bad_alloc();
	// convert to csparse matrix

	m_t_lambda.p = m_p_lambda->p;
	m_t_lambda.nzmax = m_p_lambda->nzmax;
	m_t_lambda.nrow = m_p_lambda->m;
	m_t_lambda.ncol = m_p_lambda->n;
	m_t_lambda.i = m_p_lambda->i;
	m_t_lambda.x = m_p_lambda->x;
	// fills cholsol matrix (just reref arrays)

	dt.Reset();
	// reset the timer

	m_t_cholmod_common.prefer_upper = b_upper_factor;
#if 1
	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = CHOLMOD_NATURAL; // no ordering
	m_t_cholmod_common.postorder = 0; // !!
	// set ordering up (blocky solver rewrites it)

	cholmod_factor *p_cholmod_factor;
#ifdef __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_l_analyze(&m_t_lambda, &m_t_cholmod_common)))
#else // __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_analyze(&m_t_lambda, &m_t_cholmod_common)))
#endif // __CHOLMOD_x64
		return false;
#else // 1
	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = CHOLMOD_GIVEN;//CHOLMOD_NATURAL; // no ordering
	m_t_cholmod_common.postorder = 0;
	// set ordering up (blocky solver rewrites it)

	{
		const size_t n = r_lambda.n_Column_Num();
		if(m_p_scalar_permutation.size() < n) {
			m_p_scalar_permutation.clear();
			m_p_scalar_permutation.resize(std::max(n, 2 * m_p_scalar_permutation.capacity()));
		}
		for(size_t i = 0; i < n; ++ i)
			m_p_scalar_permutation[i] = i;
	}
	// make identity permutation

	cholmod_factor *p_cholmod_factor;
#ifdef __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_l_analyze_p(&m_t_lambda,
	   &m_p_scalar_permutation[0], NULL, 0, &m_t_cholmod_common)))
#else // __CHOLMOD_x64
	if(!(p_cholmod_factor = cholmod_analyze_p(&m_t_lambda,
	   &m_p_scalar_permutation[0], NULL, 0, &m_t_cholmod_common)))
#endif // __CHOLMOD_x64
		return false;
#endif // 1
	// symbolic factorization

#ifdef __CHOLMOD_x64
	cholmod_l_factorize(&m_t_lambda, p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_factorize(&m_t_lambda, p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	if(m_t_cholmod_common.status == CHOLMOD_NOT_POSDEF) {
#ifdef __CHOLMOD_x64
		cholmod_l_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
		cholmod_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
		return false; // not positive definite
	}
	// factorize

#ifdef __CHOLMOD_x64
	if(!cholmod_l_change_factor(CHOLMOD_REAL, 1, 0/*p_cholmod_factor->is_super*/, 1/*0*/, 1/*p_cholmod_factor->is_monotonic*/, p_cholmod_factor, &m_t_cholmod_common))
#else // __CHOLMOD_x64
	if(!cholmod_change_factor(CHOLMOD_REAL, 1, 0/*p_cholmod_factor->is_super*/, 1/*0*/, 1/*p_cholmod_factor->is_monotonic*/, p_cholmod_factor, &m_t_cholmod_common))
#endif // __CHOLMOD_x64
		return false;
	_ASSERTE(p_cholmod_factor->is_ll && !p_cholmod_factor->is_super && p_cholmod_factor->is_monotonic); // makes sure it comes in correct format
	// convert the factorization to LL, simplical, packed/*or not*/, monotonic

	cs L;
	L.nzmax = p_cholmod_factor->nzmax;
	L.nz = -1;
	L.p = (csi*)p_cholmod_factor->p;
	L.i = (csi*)p_cholmod_factor->i;
	L.x = (double*)p_cholmod_factor->x;
	L.m = p_cholmod_factor->n;
	L.n = p_cholmod_factor->n;
	// get L matrix from the factor

	r_f_time = dt.f_Time();
	// sample the timer

#if 0
	_ASSERTE(p_cholmod_factor->ordering == CHOLMOD_NATURAL ||
		p_cholmod_factor->ordering == CHOLMOD_GIVEN);
	cs *p_L;
	{
		const cs *A = m_p_lambda;
		css *S;
		if(!(S = cs_schol(0, A))) // do use symbolic something! (keeps L sparse)
			return false;
		// ordering and symbolic analysis for a Cholesky factorization

		csn *N;
		if(!(N = cs_chol(A, S))) {// @todo - use CLinearSolver_CSparse to do that, it caches workspace and stuff ...
			cs_sfree(S);
			return false;
		}

		p_L = N->L;//cs_symperm(N->L, N->pinv, 1);

		cs_sfree(S);

		cs_spfree(N->U); // t_odo - remove if possible
		cs_free(N->pinv);
		cs_free(N->B);
		//cs_free(N->L);
		// note that the above pointers are most likely null (not used for Cholesky, only by QR or LU)
	}

	cs *p_diff = cs_add(p_L, &L, -1, 1);
	double f_diff = cs_norm(p_diff);
	if(f_diff > 1e-5) {
		CDebug::Dump_SparseMatrix("L_csparse.tga", p_L);
		CDebug::Dump_SparseMatrix("L_cholmod.tga", &L);
		CDebug::Dump_SparseMatrix("L_diff.tga", p_diff);
	}
	cs_spfree(p_L);
	cs_spfree(p_diff);
#endif // 0
	// debug code to make sure no ordering is used

	_ASSERTE(sizeof(_TyCSIntType) == sizeof(_TyPerm));
	bool b_result;
	if(b_upper_factor) {
		_ASSERTE(L.n >= 0 && L.n <= SIZE_MAX);
		if(m_p_scalar_permutation.size() < size_t(L.m)) {
			m_p_scalar_permutation.clear();
			m_p_scalar_permutation.resize(std::max(size_t(L.m), 2 * m_p_scalar_permutation.capacity()));
		}
		// reuse storage

		cs *p_transpose = fast_transpose(&L, (_TyCSIntType*)&m_p_scalar_permutation[0]);
		// this calculates transpose with 32 or 64bit integers, based on target machine and __CHOLMOD_x64_BUT_SHORT

#ifdef __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse32(n_dest_row_id, n_dest_column_id,
			p_transpose, false, r_workspace);
#else // __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			p_transpose, false, r_workspace);
#endif // __CHOLMOD_x64_BUT_SHORT

		cs_spfree(p_transpose);
	} else {
#ifdef __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse32(n_dest_row_id, n_dest_column_id,
			&L, false, r_workspace);
#else // __CHOLMOD_x64_BUT_SHORT
		b_result = r_factor.From_Sparse(n_dest_row_id, n_dest_column_id,
			&L, false, r_workspace);
#endif // __CHOLMOD_x64_BUT_SHORT
	}

#ifdef __CHOLMOD_x64
	cholmod_l_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_free_factor(&p_cholmod_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// cleanup

	return b_result;
}

bool CLinearSolver_CholMod::Solve_PosDef_Blocky(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	//double f_to_sparse_start = m_timer.f_Time(), f_to_sparse_end;

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
	_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension
	if(!m_p_factor) {
		if(!SymbolicDecomposition_Blocky(r_lambda))
			return false;
		_ASSERTE(m_p_factor);

		//f_to_sparse_end = f_to_sparse_start;
	} else {
#ifdef __CHOLMOD_x64_BUT_SHORT
		if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CHOLMOD_x64_BUT_SHORT
		if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CHOLMOD_x64_BUT_SHORT
			throw std::bad_alloc();
		// otherwise it's done in SymbolicDecomposition_Blocky(), can save the conversion here

		m_t_lambda.p = m_p_lambda->p;
		m_t_lambda.nzmax = m_p_lambda->nzmax;
		m_t_lambda.nrow = m_p_lambda->m;
		m_t_lambda.ncol = m_p_lambda->n;
		m_t_lambda.i = m_p_lambda->i;
		m_t_lambda.x = m_p_lambda->x;
		// fills cholsol matrix (just reref arrays)

		//f_to_sparse_end = m_timer.f_Time();
	}

#ifdef __CHOLMOD_x64
	cholmod_l_factorize(&m_t_lambda, m_p_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_factorize(&m_t_lambda, m_p_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	if(m_t_cholmod_common.status == CHOLMOD_NOT_POSDEF)
		return false;
	// calculate cholesky
	// t_odo this is only using GPU in special cases (supernodal). investigate.

#ifdef __CHOLMOD_x64
	if(!cholmod_l_change_factor(CHOLMOD_REAL, 1, 0, 1, 1, m_p_factor, &m_t_cholmod_common))
#else // __CHOLMOD_x64
	if(!cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1, m_p_factor, &m_t_cholmod_common))
#endif // __CHOLMOD_x64
		return false;
	_ASSERTE(m_p_factor->is_ll && !m_p_factor->is_super && m_p_factor->is_monotonic); // makes sure it comes in correct format
	// convert the factorization to LL, simplical, packed, monotonic
	// @todo see if this is ever called and if it is, see what exactly it does ...

	//double f_solve_start = m_timer.f_Time();

#ifdef __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	{
		const _TyPerm *p = (const _TyPerm*)m_p_factor->Perm;
		if(m_p_inverse_scalar_permutation.size() < m_t_lambda.ncol) {
			m_p_inverse_scalar_permutation.clear(); // avoids copying the data
			m_p_inverse_scalar_permutation.resize(m_t_lambda.ncol);
		}
		for(size_t i = 0, n = m_t_lambda.ncol; i < n; ++ i, ++ p)
			m_p_inverse_scalar_permutation[*p] = _TyPerm(i);
	}
	// invert the permutation

	cs L;
	L.nzmax = m_p_factor->nzmax;
	L.nz = -1;
	L.p = (csi*)m_p_factor->p;
	L.i = (csi*)m_p_factor->i;
	L.x = (double*)m_p_factor->x;
	L.m = m_p_factor->n;
	L.n = m_p_factor->n;
	// get L matrix from the factor

	if(m_n_workspace_size < size_t(m_p_lambda->n)) {
		m_n_workspace_size = std::max(2 * m_n_workspace_size, size_t(2 * m_p_lambda->n));
		if(m_p_workspace_double)
			delete[] m_p_workspace_double;
		m_p_workspace_double = new double[m_n_workspace_size];
	}
	// get some workspace

	double *p_b = &r_eta(0), *p_x = m_p_workspace_double;
	const csi n_col_num = m_p_lambda->n;
	cs_ipvec(&m_p_inverse_scalar_permutation[0], p_b, p_x, n_col_num); // x = P*b
	cs_lsolve(&L, p_x); // x = L\x
	cs_ltsolve(&L, p_x); // x = L'\x
	cs_pvec(&m_p_inverse_scalar_permutation[0], p_x, p_b, n_col_num); // b = P'*x
	// don't forget to solve, btw
#else // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	cholmod_dense t_b_cholmod;
	t_b_cholmod.nrow = t_b_cholmod.d = m_t_lambda.nrow;
	t_b_cholmod.ncol = 1;
	t_b_cholmod.x = &r_eta(0);
	t_b_cholmod.xtype = CHOLMOD_REAL;
	t_b_cholmod.dtype = CHOLMOD_DOUBLE;
	// set up dense vector with eta for calling cholmod

#ifdef __CHOLMOD_x64
	cholmod_dense *p_x_cholmod = cholmod_l_solve(CHOLMOD_A, m_p_factor, &t_b_cholmod, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_dense *p_x_cholmod = cholmod_solve(CHOLMOD_A, m_p_factor, &t_b_cholmod, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	_ASSERTE(r_eta.rows() == t_b_cholmod.nrow); // it's symmetric, dimension of solution equals dimension of right side
	// call cholesky

	memcpy(&r_eta(0), p_x_cholmod->x, sizeof(double) * t_b_cholmod.nrow);
	// copy back to src array

#ifdef __CHOLMOD_x64
	cholmod_l_free_dense(&p_x_cholmod, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	cholmod_free_dense(&p_x_cholmod, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	//cholmod_free_factor(&m_p_factor, &m_t_cholmod_common); // not the factor! will reuse it.
	// cleanup
#endif // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE

	/*double f_solve_end = m_timer.f_Time();

	m_f_tosparse_time += f_to_sparse_end - f_to_sparse_start;
	m_f_factor_time += f_solve_start - f_to_sparse_end;
	m_f_solve_time += f_solve_end - f_solve_start;*/

	return true;
}

bool CLinearSolver_CholMod::SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
{
	//double f_to_sparse_start = m_timer.f_Time();

	_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
#ifdef __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse32(m_p_lambda)))
#else // __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_lambda = r_lambda.p_Convert_to_Sparse(m_p_lambda)))
#endif // __CHOLMOD_x64_BUT_SHORT
		throw std::bad_alloc();
	// convert to csparse matrix

	m_t_lambda.p = m_p_lambda->p;
	m_t_lambda.nzmax = m_p_lambda->nzmax;
	m_t_lambda.nrow = m_p_lambda->m;
	m_t_lambda.ncol = m_p_lambda->n;
	m_t_lambda.i = m_p_lambda->i;
	m_t_lambda.x = m_p_lambda->x;
	// fills cholsol matrix (just reref arrays)

#ifdef __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse32(m_p_block_structure)))
#else // __CHOLMOD_x64_BUT_SHORT
	if(!(m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse(m_p_block_structure)))
#endif // __CHOLMOD_x64_BUT_SHORT
		throw std::bad_alloc();
	// convert block layout to csparse matrix

	const size_t n_column_block_num = r_lambda.n_BlockColumn_Num();
	if(m_p_block_permutation.size() < n_column_block_num) {
		m_p_block_permutation.clear(); // avoids copying data if resizing
		m_p_block_permutation.resize(std::max(n_column_block_num, 2 * m_p_block_permutation.capacity()));
	}
	// double space if resizing

	//double f_analyze_start = m_timer.f_Time();

	m_t_block_structure.nzmax = m_p_block_structure->nzmax;
	m_t_block_structure.nrow = m_t_block_structure.ncol = n_column_block_num;
	m_t_block_structure.p = m_p_block_structure->p;
	m_t_block_structure.i = m_p_block_structure->i;
#ifdef __CHOLMOD_x64
	if(!cholmod_l_amd(&m_t_block_structure, NULL, 0, &m_p_block_permutation[0], &m_t_cholmod_common))
#else // __CHOLMOD_x64
	if(!cholmod_amd(&m_t_block_structure, NULL, 0, &m_p_block_permutation[0], &m_t_cholmod_common))
#endif // __CHOLMOD_x64
		return false;
	// prepare AMD call via CHOLMOD

	const size_t n_column_num = r_lambda.n_Column_Num();
	if(m_p_scalar_permutation.size() < n_column_num) {
		m_p_scalar_permutation.clear();
		m_p_scalar_permutation.resize(std::max(n_column_num, 2 * m_p_scalar_permutation.capacity()));
	}
	size_t n_scalar_offset = 0;
	const _TyPerm *p_order_ptr = &m_p_block_permutation[0];
	for(size_t i = 0; i < n_column_block_num; ++ i, ++ p_order_ptr) {
		const size_t n_order = *p_order_ptr;
		size_t n_block_base = r_lambda.n_BlockColumn_Base(n_order);
		size_t n_block_width = r_lambda.n_BlockColumn_Column_Num(n_order);
		for(size_t j = 0; j < n_block_width; ++ j, ++ n_scalar_offset, ++ n_block_base)
			m_p_scalar_permutation[n_scalar_offset] = _TyPerm(n_block_base);
	}
	_ASSERTE(n_scalar_offset == n_column_num);
	// blow up the permutation to the scalar matrix

	m_t_cholmod_common.nmethods = 1;
	m_t_cholmod_common.method[0].ordering = CHOLMOD_GIVEN;
	m_t_cholmod_common.postorder = 1;
#ifdef __CHOLMOD_x64
	m_p_factor = cholmod_l_analyze_p(&m_t_lambda, &m_p_scalar_permutation[0], NULL, 0, &m_t_cholmod_common);
#else // __CHOLMOD_x64
	m_p_factor = cholmod_analyze_p(&m_t_lambda, &m_p_scalar_permutation[0], NULL, 0, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
	// apply the ordering

	/*double f_analyze_end = m_timer.f_Time();

	m_f_tosparse_time += f_analyze_start - f_to_sparse_start;
	m_f_analysis_time += f_analyze_end - f_analyze_start;*/

	return true;
}

#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

#ifdef __CHOLMOD_x64_BUT_SHORT

cs *CLinearSolver_CholMod::cs_spalloc32(csi m, csi n, csi nzmax, csi values, csi triplet)
{
	cs *A = (cs*)cs_calloc (1, sizeof (cs)) ;    /* allocate the cs struct */
	if (!A) return (NULL) ;                 /* out of memory */
	A->m = m ;                              /* define dimensions and nzmax */
	A->n = n ;
	A->nzmax = nzmax = CS_MAX (nzmax, 1) ;
	A->nz = triplet ? 0 : -1 ;              /* allocate triplet or comp.col */
	A->p = (csi*)cs_malloc (triplet ? nzmax : n+1, sizeof (int)) ;
	A->i = (csi*)cs_malloc (nzmax, sizeof (int)) ;
	A->x = (double*)(values ? cs_malloc (nzmax, sizeof (double)) : NULL) ;
	return ((!A->p || !A->i || (values && !A->x)) ? cs_spfree (A) : A) ;
}

#endif // __CHOLMOD_x64_BUT_SHORT

void CLinearSolver_CholMod::fast_cumsum2(_TyCSIntType *p, _TyCSIntType *c, size_t n)
{
	size_t nz = 0;
	for(_TyCSIntType *e = p + n; p != e; ++ p, ++ c) {
		_ASSERTE(nz <= CMaxIntValue<_TyCSIntType>::result());
		*p = _TyCSIntType(nz);
		nz += *c;
		*c = *p;
	}
	_ASSERTE(nz <= CMaxIntValue<_TyCSIntType>::result());
	*p = _TyCSIntType(nz);
}

cs *CLinearSolver_CholMod::fast_transpose(const cs *A, _TyCSIntType *p_workspace)
{
	csi m, n;
	_TyCSIntType *Cp, *Ci, *Ap, *Ai, *w;
	double *Cx, *Ax;
	_ASSERTE(CS_CSC(A) && A->x && p_workspace);    /* check inputs */
	m = A->m; n = A->n; Ap = (_TyCSIntType*)A->p; Ai = (_TyCSIntType*)A->i; Ax = A->x;
	cs *C;
#ifdef __CHOLMOD_x64_BUT_SHORT
	if(!(C = cs_spalloc32(n, m, Ap[n], 1, 0)))
#else // __CHOLMOD_x64_BUT_SHORT
	if(!(C = cs_spalloc(n, m, Ap[n], 1, 0)))
#endif // __CHOLMOD_x64_BUT_SHORT
		return 0;       /* allocate result */
	w = p_workspace;
	memset(w, 0, m * sizeof(_TyCSIntType));                      /* get workspace */
	Cp = (_TyCSIntType*)C->p; Ci = (_TyCSIntType*)C->i; Cx = C->x;
	for(_TyCSIntType p = 0; p < Ap[n]; ++ p)
		++ w[Ai[p]];          /* row counts */
	fast_cumsum2(Cp, w, m); // cant use fast cumsum, need copy of the array       /* row pointers */
	for(_TyCSIntType j = 0; j < n; ++ j) {
		for(_TyCSIntType p = Ap[j]; p < Ap[j + 1]; ++ p) {
			_TyCSIntType q = w[Ai[p]];
			++ w[Ai[p]];
			Ci[q] = j; /* place A(i,j) as entry C(j,i) */
			Cx[q] = Ax[p];
		}
	}
	return C;  /* success; do not free w, just return C */
}

/*
 *								=== ~CLinearSolver_CholMod ===
 */
