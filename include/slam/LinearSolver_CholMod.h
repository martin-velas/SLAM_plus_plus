/*
								+-----------------------------------+
								|                                   |
								|  ***  CHOLMOD linear solver  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|      LinearSolver_CholMod.h       |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __LINEAR_SOLVER_CHOLMOD_INCLUDED
#define __LINEAR_SOLVER_CHOLMOD_INCLUDED

/**
 *	@file include/slam/LinearSolver_CholMod.h
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

/** \addtogroup linsolve
 *	@{
 */

/**
 *	@def __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
 *	@brief uses CSparse for linear solve after the cholesky factor is calculated using CHOLMOD
 *	@note This is usually slower - don't use.
 */
//#define __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE

/**
 *	@def __CHOLMOD_BLOCKY_LINEAR_SOLVER
 *	@brief if defined, CLinearSolver_CholMod will be a blocky linear solver
 */
#define __CHOLMOD_BLOCKY_LINEAR_SOLVER

/**
 *	@def __CHOLMOD_PREFER_ELEMENTWISE_LINEAR_SOLVER
 *	@brief if defined, CLinearSolver_CholMod will be tagged
 *		as an elementwise solver, even despite having block interface
 *	@note This has only effect if __CHOLMOD_BLOCKY_LINEAR_SOLVER is defined.
 *	@todo This was written primarily to support CNonlinearSolver_FastL,
 *		but it crashes at 10K, have to debug it.
 */
#define __CHOLMOD_PREFER_ELEMENTWISE_LINEAR_SOLVER

#include "slam/LinearSolverTags.h"
#include "csparse/cs.hpp"
#include "cholmod/cholmod.h"
//#include "slam/BlockMatrix.h" // included from slam/LinearSolverTags.h

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#ifndef __CHOLMOD_SHORT
/**
 *	@def __CHOLMOD_x64
 *	@brief if defined, CHOLMOD is compuiled as x64, using 64-bit integers
 */
#define __CHOLMOD_x64
#else // __CHOLMOD_SHORT
/**
 *	@def __CHOLMOD_x64_BUT_SHORT
 *	@brief if defined, CHOLMOD is compuiled as x64, using 32-bit integers
 */
#define __CHOLMOD_x64_BUT_SHORT
#endif // __CHOLMOD_SHORT
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
// decide whether to have x64 CHOLMOD and whether to shorten it back to 32

/**
 *	@brief linear solver model based on CHOLMOD
 */
class CLinearSolver_CholMod {
public:
#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER
#ifdef __CHOLMOD_PREFER_ELEMENTWISE_LINEAR_SOLVER
	typedef CBasicLinearSolverTag _Tag; /**< @brief solver type tag */
#else // __CHOLMOD_PREFER_ELEMENTWISE_LINEAR_SOLVER
	typedef CBlockwiseLinearSolverTag _Tag; /**< @brief solver type tag */
#endif // __CHOLMOD_PREFER_ELEMENTWISE_LINEAR_SOLVER
#else // __CHOLMOD_BLOCKY_LINEAR_SOLVER
	typedef CBasicLinearSolverTag _Tag; /**< @brief solver type tag */
#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		default_ordering_Method = CHOLMOD_AMD, /**< @brief default symbolic ordering method (one of CHOLMOD_METIS, CHOLMOD_NESDIS, CHOLMOD_AMD or CHOLMOD_COLAMD) */
#ifdef GPU_BLAS
		default_analysis_Type = CHOLMOD_SUPERNODAL /**< @brief default symbolic analysis type (only CHOLMOD_SUPERNODAL is accelerated by the GPU Cholmod) */
#else // GPU_BLAS
		default_analysis_Type = CHOLMOD_AUTO /**< @brief default symbolic analysis type (one of CHOLMOD_AUTO, CHOLMOD_SIMPLICIAL or CHOLMOD_SUPERNODAL) */
#endif // GPU_BLAS
	};

protected:
	cs *m_p_lambda; /**< @brief memory for lambda matrix, in order to avoid reallocation */
	cholmod_common m_t_cholmod_common; /**< @brief cholmod library handle */
	cholmod_sparse m_t_lambda; /**< @brief lambda matrix in cholmod format (only points to m_p_lambda) */

#ifdef __CHOLMOD_x64_BUT_SHORT
	typedef int _TyCSIntType; /**< @brief integer type to be used by CSparse data structures */
#else // __CHOLMOD_x64_BUT_SHORT
	typedef csi _TyCSIntType; /**< @brief integer type to be used by CSparse data structures */
#endif // __CHOLMOD_x64_BUT_SHORT

	cholmod_factor *m_p_factor; /**< @brief cholmod factorization */
	cs *m_p_block_structure; /**< @brief memory for block structure of the lambda matrix */
#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER
	cholmod_sparse m_t_block_structure; /**< @brief block structure of the lambda matrix in cholmod format */ // t_odo - make this preinit member
#ifdef __CHOLMOD_x64
	typedef SuiteSparse_long _TyPerm; /**< @brief integer type, used by cholmod */ // possibly int64 (avoid signed/unsigned mismatches)
#else // __CHOLMOD_x64
	typedef int _TyPerm; /**< @brief integer type, used by cholmod */
#endif // __CHOLMOD_x64
	std::vector<_TyPerm> m_p_scalar_permutation; /**< @brief memory for scalar permutation of the lambda matrix */
	std::vector<_TyPerm> m_p_block_permutation; /**< @brief memory for block permutation of the lambda matrix */ // t_odo - resolve x64
#ifdef __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
	std::vector<_TyPerm> m_p_inverse_scalar_permutation; /**< @brief memory for inverse scalar permutation of the lambda matrix */
	double *m_p_workspace_double; /**< @brief reusable workspace for Cholesky factorization */
	size_t m_n_workspace_size; /**< @brief size of workspace for Cholesky factorization */
#endif // __LINEAR_SOLVER_CHOLMOD_CSPARSE_INPLACE_SOLVE
#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

	/*CTimer m_timer;

	double m_f_tosparse_time;
	double m_f_analysis_time;
	double m_f_factor_time;
	double m_f_solve_time;*/ // debug

public:
	/**
	 *	@brief default constructor
	 */
	CLinearSolver_CholMod();

	/**
	 *	@brief constructor; sets Cholmod analysis type and ordering
	 *
	 *	@param[in] n_analysis_type is Cholmod analysis type (one of \ref CHOLMOD_AUTO,
	 *		\ref CHOLMOD_SIMPLICIAL or \ref CHOLMOD_SUPERNODAL)
	 *	@param[in] n_ordering_method is Cholmod ordering method (one of \ref CHOLMOD_METIS,
	 *		\ref CHOLMOD_NESDIS, \ref CHOLMOD_AMD or \ref CHOLMOD_COLAMD)
	 */
	CLinearSolver_CholMod(int n_analysis_type /*= default_analysis_Type*/,
		int n_ordering_method /*= default_ordering_Method*/);

	/**
	 *	@brief copy constructor (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (Cholmod analysis and ordering settings are copied from it)
	 */
	CLinearSolver_CholMod(const CLinearSolver_CholMod &r_other);

	/**
	 *	@brief destructor (deletes memory for lambda, if allocated)
	 */
	~CLinearSolver_CholMod();

	/**
	 *	@brief deletes memory for all the auxiliary buffers and matrices, if allocated
	 */
	void Free_Memory();

	/**
	 *	@brief copy operator (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (Cholmod analysis and ordering settings are copied from it)
	 *	@return Returns reference to this.
	 */
	CLinearSolver_CholMod &operator =(const CLinearSolver_CholMod &r_other);

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

#ifdef __CHOLMOD_BLOCKY_LINEAR_SOLVER

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
		if(m_p_factor) {
#ifdef __CHOLMOD_x64
			cholmod_l_free_factor(&m_p_factor, &m_t_cholmod_common);
#else // __CHOLMOD_x64
			cholmod_free_factor(&m_p_factor, &m_t_cholmod_common);
#endif // __CHOLMOD_x64
			m_p_factor = 0;
		}
	}

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

	/**
	 *	@brief calculates symbolic factorization of a block
	 *		matrix for later (re)use in solving it
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@return Returns true on success, false on failure.
	 */
	bool SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda); // throw(std::bad_alloc)

#endif // __CHOLMOD_BLOCKY_LINEAR_SOLVER

protected:
#ifdef __CHOLMOD_x64_BUT_SHORT
	/* allocate a sparse matrix (triplet form or compressed-column form) */
	static cs *cs_spalloc32(csi m, csi n, csi nzmax, csi values, csi triplet);
#endif // __CHOLMOD_x64_BUT_SHORT

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
	static void fast_cumsum2(_TyCSIntType *p, _TyCSIntType *c, size_t n);

	/**
	 *	@brief calculates matrix transpose
	 *
	 *	@param[in] A is the matrix to be transposed, needs to be compressed column
	 *	@param[out] p_workspace is temporary workspace (of size A->n)
	 *
	 *	@return Returns transpose of A on success, 0 on failure.
	 *
	 *	@note Originally from CSparse, avoids memory re-allocations by giving workspace pointers.
	 *
	 *	@author CSparse: Copyright (c) 2006-2011, Timothy A. Davis.
	 */
	static cs *fast_transpose(const cs *A, _TyCSIntType *p_workspace);
};

/** @} */ // end of group

#endif // !__LINEAR_SOLVER_CHOLMOD_INCLUDED
