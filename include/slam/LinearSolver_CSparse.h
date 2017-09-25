/*
								+-----------------------------------+
								|                                   |
								|  ***  CSparse linear solver  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|      LinearSolver_CSparse.h       |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __LINEAR_SOLVER_CS_INCLUDED
#define __LINEAR_SOLVER_CS_INCLUDED

/**
 *	@file include/slam/LinearSolver_CSparse.h
 *	@brief linear solver model based on CSparse
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 */


/** \addtogroup linsolve
 *	@{
 */

/**
 *	@def __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE
 *	@brief if defined, CLinearSolver_CSparse::Solve_PosDef() will reuse
 *		workspace memory instead of allocating and deleting it every time
 */
#define __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE

/**
 *	@def __CSPARSE_BLOCKY_LINEAR_SOLVER
 *	@brief if defined, CLinearSolver_CSparse will be a blocky linear solver
 */
#define __CSPARSE_BLOCKY_LINEAR_SOLVER

#include "slam/LinearSolverTags.h"
#include "csparse/cs.hpp"
//#include "slam/BlockMatrix.h" // included from slam/LinearSolverTags.h

/**
 *	@brief linear solver model based on CSparse
 */
class CLinearSolver_CSparse {
public:
#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER
	typedef CBlockwiseLinearSolverTag _Tag; /**< @brief solver type tag */
#else // __CSPARSE_BLOCKY_LINEAR_SOLVER
	typedef CBasicLinearSolverTag _Tag; /**< @brief solver type tag */
#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER

protected:
	cs *m_p_lambda; /**< @brief memory for lambda matrix, in order to avoid reallocation */

#if defined (__CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE) || defined(__CSPARSE_BLOCKY_LINEAR_SOLVER)
	size_t m_n_workspace_size; /**< @brief size of workspace for Cholesky factorization */
	double *m_p_workspace_double; /**< @brief reusable workspace for Cholesky factorization */
	csi *m_p_workspace_int; /**< @brief reusable workspace for Cholesky factorization */
#endif // __CSPARSE_LINEAR_SOLVER_REUSE_WORKSPACE || __CSPARSE_BLOCKY_LINEAR_SOLVER

#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER
	css *m_p_symbolic_decomposition; /**< @brief symbolic factorization of lambda matrix */
	cs *m_p_block_structure; /**< @brief memory for block structure of the lambda matrix */
	std::vector<csi> m_v_permutation; /**< @brief memory for (scalar) permutation of the lambda matrix */
#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER

	/*CTimer m_timer;
	double m_f_chol, m_f_tran, m_f_fill;*/

public:
	/**
	 *	@brief default constructor
	 */
	CLinearSolver_CSparse();

	/**
	 *	@brief copy constructor (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 */
	CLinearSolver_CSparse(const CLinearSolver_CSparse &UNUSED(r_other));

	/**
	 *	@brief destructor (deletes memory for lambda, if allocated)
	 */
	~CLinearSolver_CSparse();

	/**
	 *	@brief deletes memory for all the auxiliary buffers and matrices, if allocated
	 */
	void Free_Memory();

	/**
	 *	@brief copy operator (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 *	@return Returns reference to this.
	 */
	CLinearSolver_CSparse &operator =(const CLinearSolver_CSparse &UNUSED(r_other));

	/**
	 *	@brief solves linear system given by positive-definite matrix
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	bool Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta); // throw(std::bad_alloc)

#ifdef __CSPARSE_BLOCKY_LINEAR_SOLVER

	/**
	 *	@brief factorizes a pre-ordered block matrix, puts result in another block matrix
	 *
	 *	@param[out] r_factor is destination for the factor (must contain structure but not any nonzero blocks)
	 *	@param[in] r_lambda is a pre-ordered block matrix to be factorized
	 *	@param[in] r_workspace is reverse row lookup of r_factor for CUberBlockMatrix::From_Sparse()
	 *	@param[in] n_dest_row_id is id of block row where the factor should be put in L
	 *	@param[in] n_dest_column_id is id of block column where the factor should be put in L
	 *	@param[in] b_upper_factor is the factor flag (if set, factor is U (R), if not se, factor is L)
	 *
	 *	@return Returns true on success, false on failure (not pos def or incorrect structure of r_factor).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	bool Factorize_PosDef_Blocky(CUberBlockMatrix &r_factor, const CUberBlockMatrix &r_lambda,
		std::vector<size_t> &r_workspace, size_t n_dest_row_id = 0,
		size_t n_dest_column_id = 0, bool b_upper_factor = true); // throw(std::bad_alloc)

	/**
	 *	@brief factorizes a pre-ordered block matrix, puts result in another block matrix
	 *
	 *	@param[out] r_f_time is time of the factorization only (ommits block conversion)
	 *	@param[out] r_factor is destination for the factor (must contain structure but not any nonzero blocks)
	 *	@param[in] r_lambda is a pre-ordered block matrix to be factorized
	 *	@param[in] r_workspace is reverse row lookup of r_factor for CUberBlockMatrix::From_Sparse()
	 *	@param[in] n_dest_row_id is id of block row where the factor should be put in L
	 *	@param[in] n_dest_column_id is id of block column where the factor should be put in L
	 *	@param[in] b_upper_factor is the factor flag (if set, factor is U (R), if not se, factor is L)
	 *
	 *	@return Returns true on success, false on failure (not pos def or incorrect structure of r_factor).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	bool Factorize_PosDef_Blocky_Benchmark(double &r_f_time, CUberBlockMatrix &r_factor,
		const CUberBlockMatrix &r_lambda, std::vector<size_t> &r_workspace, size_t n_dest_row_id = 0,
		size_t n_dest_column_id = 0, bool b_upper_factor = true); // throw(std::bad_alloc)

	/**
	 *	@brief deletes symbolic factorization, if calculated (forces a symbolic
	 *		factorization update in the next call to Solve_PosDef_Blocky())
	 */
	inline void Clear_SymbolicDecomposition()
	{
		if(m_p_symbolic_decomposition) {
			cs_sfree(m_p_symbolic_decomposition);
			m_p_symbolic_decomposition = 0;
		}
	}

	/**
	 *	@brief calculates symbolic factorization of a block
	 *		matrix for later (re)use in solving it
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@return Returns true on success, false on failure.
	 */
	bool SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda); // throw(std::bad_alloc)

	/**
	 *	@brief solves linear system given by positive-definite matrix
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This enables a reuse of previously calculated symbolic factorization,
	 *		it can be either calculated in SymbolicDecomposition_Blocky(), or it is
	 *		calculated automatically after the first call to this function,
	 *		or after Clear_SymbolicDecomposition() was called (preferred).
	 */
	bool Solve_PosDef_Blocky(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta); // throw(std::bad_alloc)

#endif // __CSPARSE_BLOCKY_LINEAR_SOLVER

protected:
	/**
	 *	@brief calculates cumulative sum
	 *
	 *	p [0..n] = cumulative sum of c [0..n-1]
	 *
	 *	@param[out] p is array to be filled with cumulative sums (must be allocated by caller, to n + 1 elements)
	 *	@param[in] c is array to be summed (contains n elements)
	 *	@param[in] n is number of elements to be summed
	 *
	 *	@return Reurtns the sum of all elements in c (also equal to p[n]).
	 *
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 *
	 *	@note This was slightly optimized to perform the sum in integer instead of double,
	 *		and to avoid the needless calculation of all the other stuff CSparse does.
	 */
	static size_t fast_cumsum(csi *p, const csi *c, csi n);

	/**
	 *	@brief calculates cumulative sum and copies n values back to input array
	 *
	 *	p [0..n] = cumulative sum of c [0..n-1]
	 *	c [0..n-1] = copy of p [0..n-1]
	 *
	 *	@param[out] p is array to be filled with cumulative sums (must be allocated by caller, to n + 1 elements)
	 *	@param[in,out] c is array to be summed (contains n elements)
	 *	@param[in] n is number of elements to be summed
	 *
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 *
	 *	@note This was slightly optimized to perform the sum in integer instead of double,
	 *		and to avoid the needless calculation of all the other stuff CSparse does.
	 */
	static void fast_cumsum2(csi *p, csi *c, size_t n);

	/**
	 *	@brief calculates matrix transpose
	 *
	 *	@param[out] C is the storage for the transpose matrix to be reused
	 *	@param[in] A is the matrix to be transposed, needs to be compressed column
	 *	@param[out] p_workspace is temporary workspace (of size A->n)
	 *
	 *	@return Returns transpose of A on success, 0 on failure.
	 *
	 *	@note Originally from CSparse, avoids memory re-allocations by giving workspace pointers.
	 *
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 */
	static cs *fast_transpose(cs *C, const cs *A, csi *p_workspace);

	/**
	 *	@brief solves a system using cholesky factorization
	 *
	 *	@param[in] A is the matrix
	 *	@param[in,out] b is the right-side vector (overwritten with solution)
	 *	@param[in] S is symbolic ordering
	 *	@param[out] x is remporary workspace (of size A->n)
	 *	@param[out] work is remporary workspace (of size 2 * A->n)
	 *
	 *	@return Returns 1 on success, 0 on failure.
	 *
	 *	@note Originally from CSparse, avoid memory re-allocations by giving workspace pointers.
	 *
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 */
	static csi cs_cholsolsymb(const cs *A, double *b, const css *S, double *x, csi *work);

	/**
	 *	@brief calculates cholesky factorization
	 *
	 *	@param[in] A is the matrix
	 *	@param[in] S is symbolic ordering
	 *	@param[out] cin is temporary workspace (of size 2 * A->n)
	 *	@param[out] xin is temporary workspace (of size A->n)
	 *
	 *	@return Returns numeric Cholesky factorization of A.
	 *
	 *	@note Originally from CSparse, avoid memory re-allocations by giving workspace pointers.
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 */
	static csn *cs_chol_workspace (const cs *A, const css *S, csi *cin, double *xin);
};

/** @} */ // end of group

#endif // !__LINEAR_SOLVER_CS_INCLUDED
