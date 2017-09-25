/*
								+----------------------------------+
								|                                  |
								|  ***  Native linear solver  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|     LinearSolver_UberBlock.h     |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __LINEAR_SOLVER_UBERBLOCK_INCLUDED
#define __LINEAR_SOLVER_UBERBLOCK_INCLUDED

/**
 *	@file include/slam/LinearSolver_UberBlock.h
 *	@brief the native linear solver model
 *	@author -tHE SWINe-
 *	@date 2013-01-28
 */

#include "slam/LinearSolverTags.h"
#include "csparse/cs.hpp"
//#include "slam/BlockMatrix.h" // included from slam/LinearSolverTags.h
#include "slam/OrderingMagic.h" // need AMD

/** \addtogroup linsolve
 *	@{
 */

/**
 *	@def __USE_CS_AMD
 *	@brief if defined, uses cs_amd() to calculate the ordering, otherwise uses AMD from OrderingMagic.cpp
 */
//#define __USE_CS_AMD

/**
 *	@brief linear solver model based on the native implementation (fixed block size)
 *	@tparam CBlockMatrixTypelist is is typelist, containing Eigen
 *		matrices with known compile-time sizes
 */
template <class CBlockMatrixTypelist>
class CLinearSolver_UberBlock {
public:
	typedef CBlockwiseLinearSolverTag _Tag; /**< @brief solver type tag */
	typedef CBlockMatrixTypelist _TyBlockSizes; /**< @brief list of block matrix sizes */

protected:
	cs *m_p_block_structure; /**< @brief block structure of the lambda matrix */
	CUberBlockMatrix m_perm; /**< @brief storage for the permuted lambda */
	CUberBlockMatrix m_R; /**< @brief the R factor */
	std::vector<size_t> m_etree; /**< @brief e-tree storage */
	std::vector<size_t> m_workspace; /**< @brief Cholesky workspace */
	std::vector<size_t> m_zeroes; /**< @brief Cholesky workspace */
	std::vector<double> m_workspace_double; /**< @brief space for permuted right-hand-side vector */
	const size_t *m_p_inv_order; /**< @brief pointer to the inverse ordering */

	CMatrixOrdering m_mord; /**< @brief storage for the inverse ordering */

public:
	/**
	 *	@brief default constructor (does nothing)
	 */
	inline CLinearSolver_UberBlock()
		:m_p_block_structure(0), m_p_inv_order(0)
	{}

	/**
	 *	@brief copy constructor (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 */
	CLinearSolver_UberBlock(const CLinearSolver_UberBlock &UNUSED(r_other))
		:m_p_block_structure(0), m_p_inv_order(0)
	{}

	/**
	 *	@brief destructor (deletes memory for lambda, if allocated)
	 */
	~CLinearSolver_UberBlock()
	{
		Free_Memory();
	}

	/**
	 *	@brief deletes memory for all the auxiliary buffers and matrices, if allocated
	 */
	void Free_Memory()
	{
		{
			CUberBlockMatrix empty;
			m_R.Swap(empty);
		}
		{
			CUberBlockMatrix empty;
			m_perm.Swap(empty);
		}
		{
			std::vector<size_t> empty;
			m_etree.swap(empty);
		}
		{
			std::vector<size_t> empty;
			m_workspace.swap(empty);
		}
		{
			std::vector<size_t> empty;
			m_zeroes.swap(empty);
		}
		{
			std::vector<double> empty;
			m_workspace_double.swap(empty);
		}
		if(m_p_block_structure) {
			cs_spfree(m_p_block_structure);
			m_p_block_structure = 0;
		}
		// don't free m_p_inv_order, it is referencing std::vector
	}

	/**
	 *	@brief copy operator (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 *	@return Returns reference to this.
	 */
	CLinearSolver_UberBlock &operator =(const CLinearSolver_UberBlock &UNUSED(r_other))
	{
		return *this;
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
	 */
	bool Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
	{
		_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
		_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

#if 0
		if(!(m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse(m_p_block_structure)))
			return false;
		// convert to sparse

		_ASSERTE(sizeof(csi) == sizeof(size_t));
		size_t *p_order = (size_t*)cs_amd(1, m_p_block_structure); // todo - make this native as well, or use AMD
		m_p_inv_order = m_mord.p_InvertOrdering(p_order, m_p_block_structure->n); // !!
		free(p_order); // allocated in "C"
		// calculate ordering

		bool b_result;
		try {
			const size_t n_elem_num = r_lambda.n_Column_Num();
			const size_t n_block_num = r_lambda.n_BlockColumn_Num();
			_ASSERTE(n_block_num == m_p_block_structure->n);
			r_lambda.Permute_UpperTriangular_To(m_perm, m_p_inv_order, n_block_num, true);
			_ASSERTE(n_block_num == m_perm.n_BlockColumn_Num());
			// fast, shares data (except for a few of transposes here and there)

			m_perm.Build_EliminationTree(m_etree, m_workspace);
			_ASSERTE(m_workspace.size() == n_block_num);
			// build e-tree

			m_workspace_double.resize(n_block_num);
			double *p_b = &r_eta(0), *p_x = &m_workspace_double[0];
			//cs_ipvec((const csi*)m_p_inv_order_elem, p_b, p_x, n_elem_num); // x = iperm(b)
			// permute there

			m_zeroes.resize(n_block_num, 0);
			b_result = m_R.template CholeskyOf_FBS<_TyBlockSizes>(m_perm, m_etree, m_workspace, m_zeroes);
			m_R.InversePermute_RightHandSide_Vector(p_x, p_b, n_elem_num, m_p_inv_order, n_block_num);
			m_R.template UpperTriangularTranspose_Solve_FBS<_TyBlockSizes>(p_x, n_elem_num);
			m_R.template UpperTriangular_Solve_FBS<_TyBlockSizes>(p_x, n_elem_num);
			// cholesky, utsolve, usolve

			m_R.Permute_RightHandSide_Vector(p_x, p_b, n_elem_num, m_p_inv_order, n_block_num);
			//cs_pvec((const csi*)m_p_inv_order_elem, p_x, p_b, n_elem_num); // b = perm(x)
			// permute back
		} catch(std::bad_alloc&) {
			b_result = false;
		}
#else // 0
		Clear_SymbolicDecomposition();

		bool b_result = Solve_PosDef_Blocky(r_lambda, r_eta);
		// just use the other function, don't repeat the code twice

		Clear_SymbolicDecomposition();
		// make sure that the symbolic factorization is not reused
#endif // 0

		return b_result;
	}

	/**
	 *	@brief factorizes a pre-ordered block matrix, puts result in another block matrix
	 *
	 *	@param[out] r_factor is destination for the factor (must contain structure but not any nonzero blocks)
	 *	@param[in] r_lambda is a pre-ordered block matrix to be factorized
	 *	@param[in] r_workspace is unused (served as reverse row lookup of r_factor for CUberBlockMatrix::From_Sparse())
	 *	@param[in] n_dest_row_id is id of block row where the factor should be put in L
	 *	@param[in] n_dest_column_id is id of block column where the factor should be put in L
	 *	@param[in] b_upper_factor is the factor flag (if set, factor is U (R), if not se, factor is L)
	 *
	 *	@return Returns true on success, false on failure (not pos def or incorrect structure of r_factor).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	bool Factorize_PosDef_Blocky(CUberBlockMatrix &r_factor, const CUberBlockMatrix &r_lambda,
		std::vector<size_t> &UNUSED(r_workspace), size_t n_dest_row_id = 0,
		size_t n_dest_column_id = 0, bool b_upper_factor = true) // throw(std::bad_alloc)
	{
		//_ASSERTE(b_upper_factor); // make sure we want upper factor

		bool b_result;
		try {
			const size_t n_block_num = r_lambda.n_BlockColumn_Num();
			r_lambda.Build_EliminationTree(m_etree, m_workspace);
			_ASSERTE(m_workspace.size() == n_block_num);
			// build e-tree

			m_zeroes.resize(n_block_num, 0);
			if(!n_dest_row_id && !n_dest_column_id) {
				b_result = r_factor.template CholeskyOf_FBS<_TyBlockSizes>(r_lambda, m_etree, m_workspace, m_zeroes);
				if(b_result && !b_upper_factor) {
					r_factor.TransposeTo(m_R);
					m_R.Swap(r_factor);
				}
			} else {
				b_result = m_R.template CholeskyOf_FBS<_TyBlockSizes>(r_lambda, m_etree, m_workspace, m_zeroes);
				if(b_result) {
					if(!b_upper_factor) {
						CUberBlockMatrix tmp;
						m_R.TransposeTo(tmp);
						m_R.Swap(tmp);
					}
					m_R.PasteTo(r_factor, n_dest_row_id, n_dest_column_id); // might actually work // t_odo - debug
				}
				//throw std::runtime_error("cholesky to part of factor not implemented in the native solver");
			}
			// cholesky
		} catch(std::bad_alloc&) {
			b_result = false;
		}

		return b_result;
	}

	/**
	 *	@brief deletes symbolic factorization, if calculated (forces a symbolic
	 *		factorization update in the next call to Solve_PosDef_Blocky())
	 */
	inline void Clear_SymbolicDecomposition()
	{
		if(m_p_inv_order)
			m_p_inv_order = 0; // don't delete
	}

	/**
	 *	@brief calculates symbolic factorization of a block
	 *		matrix for later (re)use in solving it
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@return Returns true on success, false on failure.
	 */
	bool SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
	{
		Clear_SymbolicDecomposition();
		// forget symbolic factorization, if it had one

#ifdef __USE_CS_AMD
		if(!(m_p_block_structure = r_lambda.p_BlockStructure_to_Sparse(m_p_block_structure)))
			return false;
#endif // __USE_CS_AMD
		// convert to sparse

		_ASSERTE(sizeof(csi) == sizeof(size_t));
#ifdef __USE_CS_AMD
		size_t *p_order = (size_t*)cs_amd(1, m_p_block_structure); // todo - make this native as well, or use AMD
		m_p_inv_order = m_mord.p_InvertOrdering(p_order, m_p_block_structure->n); // !!
		free(p_order); // allocated using "C" libs
#else // __USE_CS_AMD
		//const size_t *p_order = m_mord.p_BlockOrdering(r_lambda, 0, 0); // use CAMD with no constraints
		const size_t *p_order = m_mord.p_BlockOrdering(r_lambda, true); // use AMD (gives a better postordering than CAMD, requires less memory)
		m_p_inv_order = m_mord.p_Get_InverseOrdering(); // p_InvertOrdering(p_order, r_lambda.n_BlockColumn_Num()); // !!
#endif // __USE_CS_AMD
		// calculate ordering

		return true;
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
	bool Solve_PosDef_Blocky(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
	{
		_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
		_ASSERTE(r_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

		if(!m_p_inv_order) {
			if(!SymbolicDecomposition_Blocky(r_lambda))
				return false;
		}
		// (re)calculate matrix ordering

		bool b_result;
		try {
			const size_t n_elem_num = r_lambda.n_Column_Num();
			const size_t n_block_num = r_lambda.n_BlockColumn_Num();
			//_ASSERTE(n_block_num == m_p_block_structure->n); // m_p_block_structure unused if AMD employed for ordering
			r_lambda.Permute_UpperTriangular_To(m_perm, m_p_inv_order, n_block_num, true);
			_ASSERTE(n_block_num == m_perm.n_BlockColumn_Num());
			// fast, shares data (except for a few of transposes here and there)

			if(m_etree.capacity() < n_block_num) {
				m_etree.clear();
				m_etree.resize(std::max(n_block_num, 2 * m_etree.capacity()), 0);
			}
			if(m_workspace.capacity() < n_block_num) {
				m_workspace.clear();
				m_workspace.resize(std::max(n_block_num, 2 * m_workspace.capacity()), 0);
			}
			if(m_zeroes.capacity() < n_block_num) {
				m_zeroes.clear();
				m_zeroes.resize(std::max(n_block_num, 2 * m_zeroes.capacity()), 0);
			}
			if(m_workspace_double.capacity() < n_elem_num) {
				m_workspace_double.clear();
				m_workspace_double.resize(std::max(n_elem_num,
					2 * m_workspace_double.capacity()), 0);
			}
			double *p_x = &m_workspace_double[0];
			// allocate workspace

			m_perm.Build_EliminationTree(m_etree, m_workspace);
			_ASSERTE(m_workspace.size() == n_block_num);
			// build e-tree

#if 0
			m_perm.Rasterize("0_lambda.tga");
			{
				FILE *p_fw = fopen("1_etree.txt", "w");
				for(size_t i = 0; i < n_block_num; ++ i)
					fprintf(p_fw, PRIsize "\n", m_etree[i]);
				fclose(p_fw);
			}
			// save the matrix and the elimination tree

			size_t n_supernode_size = 0, n_supernode_num = 0;
			bool b_had_single = false;
			std::vector<size_t> ereach, ereach_pred, work;
			ereach.resize(n_block_num);
			work.resize(n_block_num, 0);
			size_t n_top_pred = 0;
			{
				FILE *p_fw = fopen("2_ereach.txt", "w");
				for(size_t j = 0; j < n_block_num; ++ j) {
					size_t n_top = m_perm.n_Build_EReach(j, m_etree, ereach, work);
					fprintf(p_fw, "ereach[" PRIsize "] = {", j);
					for(size_t i = n_top; i < n_block_num; ++ i)
						fprintf(p_fw, (i == n_top)? PRIsize : ", " PRIsize, ereach[i]);
					fprintf(p_fw, "}\n");
					if(n_top == n_top_pred - 1 && ereach[n_block_num - 1] == j - 1 && (n_top_pred == n_block_num ||
					   !memcmp(&ereach[n_top], &ereach_pred[n_top_pred],
					   (n_block_num - n_top_pred) * sizeof(size_t)))) {
						++ n_supernode_size;
					} else {
						if(n_supernode_size) { // not the first time
							printf("had supernode of size " PRIsize "\n", n_supernode_size);
							++ n_supernode_num;
						}
						n_supernode_size = 1;
					}
					ereach_pred = ereach;
					n_top_pred = n_top;
				}
				fclose(p_fw);
			}
			++ n_supernode_num;
			printf("had supernode of size " PRIsize "\n", n_supernode_size);
			printf("there is " PRIsize " nodes and " PRIsize "supernodes\n", n_block_num, n_supernode_num);
			// count supernodes and print characteristics
#endif // 0

			b_result = m_R.template CholeskyOf_FBS<_TyBlockSizes>(m_perm,
				m_etree, m_workspace, m_zeroes);

#if 0
			m_R.Rasterize("2_L.tga");
			// save the factor to see the difference
#endif // 0

			if(b_result) {
				m_R.InversePermute_LeftHandSide_Vector(p_x, &r_eta(0),
					n_elem_num, m_p_inv_order, n_block_num); // x = iperm(eta)
				m_R.template UpperTriangularTranspose_Solve_FBS<_TyBlockSizes>(p_x, n_elem_num);
				m_R.template UpperTriangular_Solve_FBS<_TyBlockSizes>(p_x, n_elem_num);
				m_R.Permute_LeftHandSide_Vector(&r_eta(0), p_x,
					n_elem_num, m_p_inv_order, n_block_num); // eta = perm(x)
				// cholesky, ipvec, utsolve, usolve, pvec
				// note that it is in fact left-hand side vector, but R is symmetric
			}
			// if cholesky fails, don't do ipvec(), block layout of R is invalid!
		} catch(std::bad_alloc&) {
			b_result = false;
		}

		return b_result;
	}
};

/**
 *	@brief a simple native solver predicate
 *	@tparam CLinearSolver is linear solver type
 */
template <class CLinearSolver>
class CIsNativeSolver {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = false /**< @brief result of comparison */
	};
};

/**
 *	@brief a simple native solver predicate (specialization for native solver)
 *	@tparam CBlockMatrixSizeList is list of block matrix sized
 */
template <class CBlockMatrixSizeList>
class CIsNativeSolver<CLinearSolver_UberBlock<CBlockMatrixSizeList> > {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = true /**< @brief result of comparison */
	};
};

/** @} */ // end of group

#endif // !__LINEAR_SOLVER_UBERBLOCK_INCLUDED
