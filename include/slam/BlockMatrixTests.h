/*
								+---------------------------------+
								|                                 |
								| *** Über Block Matrix tests *** |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2016 |
								|                                 |
								|       BlockMatrixTests.h        |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_TESTS_INCLUDED
#define __UBER_BLOCK_MATRIX_TESTS_INCLUDED

/**
 *	@file include/slam/BlockMatrixTests.h
 *	@brief block matrix testing functions and helpers
 *	@author -tHE SWINe-
 *	@date 2016-08-06
 */

#include <stdexcept>
#include "slam/Debug.h"
#include "slam/BlockMatrix.h"

/**
 *	@brief helper class for building special matrices for unit or performance tests
 */
class CTestMatrixFactory {
public:
	/**
	 *	@brief loads elementwise sparse matrix from a matrix market file
	 *	@return Returns pointer to the matrix (the caller needs to dispose of it) or 0 on failure.
	 *	@note This function throws std::bad_alloc.
	 */
	static cs *p_Load_PatternMatrix(const char *p_s_filename); // throw(std::bad_alloc)

	/**
	 *	@brief allocates upper-triangular elementwise sparse matrix
	 *	@return Returns pointer to the matrix (the caller needs to dispose of it).
	 *	@note This function throws std::bad_alloc.
	 */
	static cs *p_AllocUpper(csi m, csi n, double f_value = 1.0); // throw(std::bad_alloc)

	/**
	 *	@brief allocates lower-triangular elementwise sparse matrix
	 *	@return Returns pointer to the matrix (the caller needs to dispose of it).
	 *	@note This function throws std::bad_alloc.
	 */
	static cs *p_AllocLower(csi m, csi n, double f_value = 1.0); // throw(std::bad_alloc)

	static void Conform_SparseMatrix_to_BlockMatrix(CUberBlockMatrix &r_dest,
		const cs *p_matrix, size_t n_block_size, bool b_allow_underfilled_last = true,
		bool b_append_identity_diag_block = true); // throw(std::bad_alloc, std::runtime_error)

	static void Inflate_PatternMatrix_to_BlockMatrix(CUberBlockMatrix &r_dest,
		const cs *p_matrix, const std::vector<size_t> &r_block_sizes, bool b_symmetric); // throw(std::bad_alloc)

	/**
	 *	@brief calculates tha block matrix split by block sizes
	 *
	 *	This works in a few steps. First, the block row and block column sizes are recorded.
	 *	Then, a permutation of block rows and columns is constructed, so that blocks of each
	 *	particular size are in contiguous submatrices. Some empty space may need to be inserted
	 *	to align the top left block in each of those submatrices to the multiple of the block
	 *	size. E.g. if the matrix has a single 3x3 block and a single 4x4 block, there needs to
	 *	be a 1x1 block inserted after the 3x3 block so that the 4x4 block starts at row and
	 *	column which are multiple of four. Finally, the submatrices are split to several separate
	 *	block matrices in the output array.
	 *
	 *	Note that this process increases the bandwidth of the matrix, which hurts performance
	 *	on very long vectors where the data need to be fetched from RAM. This is not a flaw of
	 *	this implementation but rather a principal flaw of the splitting method.
	 *
	 *	@param[out] r_split_matrix is filled with a list of block matrices that sum up to
	 *		the original matrix, each containing only blocks of a single size (the blank
	 *		space is also divided to this size, to accomodate the BSR format)
	 *	@param[out] r_perm_layout is filled with block row and column layout of the split
	 *		matrix, which is used for permuting the lhs and rhs vectors (has no nonzeros)
	 *	@param[out] r_block_row_perm is filled with block row permutation required by splitting
	 *	@param[out] r_block_col_perm is filled with block column permutation required by splitting
	 *	@param[out] r_n_row_ext is the number of (scalar) rows that needed to be added to
	 *		the matrix in order to align the first block of each particulat size
	 *	@param[out] r_n_col_ext is the number of (scalar) columns that needed to be added to
	 *		the matrix in order to align the first block of each particulat size
	 *	@param[in] r_block_mat is the block matrix to split
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Split_BlockMatrix(std::vector<CUberBlockMatrix> &r_split_matrix,
		CUberBlockMatrix &r_perm_layout, std::vector<size_t> &r_block_row_perm,
		std::vector<size_t> &r_block_col_perm, size_t &r_n_row_ext, size_t &r_n_col_ext,
		const CUberBlockMatrix &r_block_mat); // throw(std::bad_alloc)
};

/**
 *	@brief unit tests for the CUberBlockMatrix class
 */
class CUberBlockMatrix_UnitTests {
public:
	/**
	 *	@brief tests \ref CUberBlockMatrix::b_LowerTriangular() and \ref CUberBlockMatrix::b_UpperTriangular().
	 *	@return Returns the number of failures (capped at INT_MAX) or 0 on success.
	 *	@note This function throws std::bad_alloc.
	 */
	static int Test_ElemwiseTriangularPredicates(); // throw(std::bad_alloc)
};

#endif // __UBER_BLOCK_MATRIX_TESTS_INCLUDED
