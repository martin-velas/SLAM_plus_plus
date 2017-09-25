/*
								+---------------------------------+
								|                                 |
								| *** Block matrix unit tests *** |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2012 |
								|                                 |
								|           BlockUnit.h           |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_UNIT_TESTS_INCLUDED
#define __UBER_BLOCK_MATRIX_UNIT_TESTS_INCLUDED

/**
 *	@file include/slam_app/BlockUnit.h
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief block matrix unit tests
 */

/**
 *	@def __BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES
 *	@brief if defined, some of the benchmarked matrices are written as images (for debugging purposes)
 */
//#define __BLOCK_ADD_UNIT_TEST_DUMP_MATRIX_IMAGES

/**
 *	@def __BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES
 *	@brief if defined, some of the benchmarked matrices are written as images (for debugging purposes)
 */
//#define __BLOCK_MUL_UNIT_TEST_DUMP_MATRIX_IMAGES

#include "slam/BlockMatrix.h"

/**
 *	@brief block matrix unit test implementations
 */
class CBlockMatrixUnitTests {
public:
	/**
	 *	@brief configuration enums
	 */
	enum {
		block_Size = 3, /**< @brief preset block size (const at compile-time) */
		block_Size2 = block_Size + 1 /**< @brief preset second block size (const at compile-time) */
	};

	typedef Eigen::Matrix<double, block_Size, block_Size> _TyBlockMatrix; /**< @brief dense matrix block */
	typedef Eigen::Matrix<double, block_Size2, block_Size2> _TyBlockMatrix2; /**< @brief dense matrix block of slightly different size than _TyBlockMatrix */

public:
	/**
	 *	@brief runs all unit tests (results in stdout and stderr)
	 *	@return Returns true if all tests passed, false in case there was an error.
	 */
	static bool RunAll();

	/**
	 *	@brief unit test for block matrix factorization functions (Cholesky)
	 *	@todo Add rectangular block tests.
	 */
	static void MatrixDecomposition_UnitTest();

	/**
	 *	@brief unit test for block matrix multiplication functions (A * B, A^T * A)
	 *	@note This also tests the _FBS A^T * A function.
	 *	@todo Add rectangular block tests.
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	static void MatrixMultiplication_UnitTest(); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief unit test for block matrix addition functions
	 *
	 *	This tests all of A + B, A + t_block_B, aA + t_block_B,
	 *	including the fixed block size variants. Tests on some
	 *	simple prepared cases, on random matrices with fixed
	 *	block size and on random matrices with free block size.
	 *	This also tests for addition failures (matrices of different
	 *	size or block layout).
	 *
	 *	Uses std::runtime_error as a means of reporting error.
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	static void MatrixAddition_UnitTest(); // throw(std::bad_alloc, std::runtime_error)

protected:
	/**
	 *	@brief tests addition of two matrices, expects success
	 *
	 *	@param[in,out] r_A is the first matrix
	 *	@param[in,out] r_B is the second matrix
	 *	@param[in] f_factor_A is the multiplication factor for the A matrix
	 *	@param[in] f_factor_B is the multiplication factor for the B matrix
	 *	@param[out] r_C is the ground truth (f_factor_A * A + f_factor_B * B)
	 *	@param[in] b_polarity is the polarity of operation to test (A += B or B += A)
	 *	@param[in] n_method is addition method (currently 0 - 6, depending on factors and FBS-ness)
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	static void Test_Add(CUberBlockMatrix &r_A, CUberBlockMatrix &r_B,
		double f_factor_A, double f_factor_B, const CUberBlockMatrix &UNUSED(r_C),
		bool b_polarity, int n_method); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief tests addition of two matrices, expects failure
	 *
	 *	@param[in,out] r_A is the first matrix
	 *	@param[in,out] r_B is the second matrix
	 *	@param[in] b_polarity is the polarity of operation to test (A += B or B += A)
	 *	@param[in] n_method is addition method (currently 0 - 6, depending on factors and FBS-ness)
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	static void Test_AddFailure(CUberBlockMatrix &r_A, CUberBlockMatrix &r_B,
		bool b_polarity, int n_method); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief generates two random matrices of the same size which can added, along with ground truth
	 *
	 *	@tparam _TyDenseBlock is type of dense block matrix
	 *
	 *	@param[out] r_A is the first matrix
	 *	@param[out] r_B is the second matrix
	 *	@param[in] f_factor_A is the multiplication factor for the A matrix
	 *	@param[in] f_factor_B is the multiplication factor for the B matrix
	 *	@param[out] r_C is the ground truth (f_factor_A * A + f_factor_B * B)
	 *	@param[in] r_t_block_A is dense seed matrix
	 *	@param[in] r_t_block_B is dense seed matrix
	 *	@param[in] r_t_block_C is dense seed matrix
	 *	@param[in] b_free_block_size is free block size flag (if set, blocks in the output
	 *		matrices will have any sizes from 1x1 up to the size of seed blocks; if not set,
	 *		the block size is fixed to block_Size and block_Size2)
	 *	@param[in] n_block_rows is number of block rows in the output matrices
	 *	@param[in] n_block_cols is number of block columns in the output matrices
	 *	@param[in] n_max_blocks is number of blocks to fill in the matrices
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	template <class _TyDenseBlock>
	static void Generate_Addable_Matrices(CUberBlockMatrix &r_A, CUberBlockMatrix &r_B,
		double f_factor_A, double f_factor_B, CUberBlockMatrix &r_C,
		const _TyDenseBlock &r_t_block_A, const _TyDenseBlock &r_t_block_B,
		const _TyDenseBlock &r_t_block_C, bool b_free_block_size, size_t n_block_rows = 10,
		size_t n_block_cols = 10, size_t n_max_blocks = 10) // throw(std::bad_alloc, std::runtime_error)
	{
		std::vector<size_t> row_sizes, col_sizes;
		if(b_free_block_size) {
			size_t n_max_rows = r_t_block_A.rows();
			size_t n_max_cols = r_t_block_A.cols();

			_ASSERTE(n_max_rows == r_t_block_B.rows() && n_max_rows == r_t_block_C.rows());
			_ASSERTE(n_max_cols == r_t_block_B.cols() && n_max_cols == r_t_block_C.cols());
			// the submatrices of these are used as seeds, make sure they are big enough

			for(size_t n_row = 0; n_row < n_block_rows; ++ n_row)
				row_sizes.push_back(rand() % n_max_rows + 1);
			for(size_t n_col = 0; n_col < n_block_cols; ++ n_col)
				col_sizes.push_back(rand() % n_max_cols + 1);
			// generate arbitrary size blocks
		} else {
			_ASSERTE(block_Size2 > block_Size);
			_ASSERTE(r_t_block_A.rows() == block_Size2 &&
				r_t_block_B.rows() == block_Size2 && r_t_block_C.rows() == block_Size2);
			_ASSERTE(r_t_block_A.cols() == block_Size2 &&
				r_t_block_B.cols() == block_Size2 && r_t_block_C.cols() == block_Size2);
			// the submatrices of these are used as seeds, make sure they are big enough

			for(size_t n_row = 0; n_row < n_block_rows; ++ n_row)
				row_sizes.push_back(((rand() & 1)? block_Size : block_Size2));
			for(size_t n_col = 0; n_col < n_block_cols; ++ n_col)
				col_sizes.push_back(((rand() & 1)? block_Size : block_Size2));
			// generate block of size or size2
		}
		// generate row and column sizes of the matrices

		std::vector<size_t> row_cumsums, col_cumsums;
		row_cumsums.push_back(0);
		col_cumsums.push_back(0);
		for(size_t n_row = 0; n_row < n_block_rows - 1; ++ n_row)
			row_cumsums.push_back(row_cumsums.back() + row_sizes[n_row]);
		for(size_t n_col = 0; n_col < n_block_cols - 1; ++ n_col)
			col_cumsums.push_back(col_cumsums.back() + col_sizes[n_col]);
		// generate row / column cumsums

		bool b_add = true;
		int n_extent_method = rand() % 3;
		if(!n_extent_method) {
			_ASSERTE(n_block_rows + n_block_cols - 1 >= 3); // make sure there is enough space for all the blocks to define size of the matrices

			size_t n_width_def_block_row_B = rand() % (n_block_rows - 1);
			size_t n_height_def_block_col_B = rand() % (n_block_cols - 1); // don't want the blocks to meet ...

			bool b_first_elem = !(rand() & 1);
			if(b_first_elem)
				b_add &= r_A.Append_Block(r_t_block_A.block(0, 0, row_sizes[0], col_sizes[0]), 0, 0);
			// this one isn't entirely required

			b_add &= r_A.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_block_cols - 1]),
				row_cumsums[n_block_rows - 1], col_cumsums[n_block_cols - 1]);
			// define size of A in lower / left corner

			b_add &= r_B.Append_Block(r_t_block_B.block(0, 0, row_sizes[n_width_def_block_row_B], col_sizes[n_block_cols - 1]),
				row_cumsums[n_width_def_block_row_B], col_cumsums[n_block_cols - 1]);
			b_add &= r_B.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_B]),
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_B]);
			// define size of B in arbitrary rows / columns in the last column / row, respectively

			if(b_first_elem)
				b_add &= r_C.Append_Block(r_t_block_A.block(0, 0, row_sizes[0], col_sizes[0]) * f_factor_A, 0, 0); // r_A
			b_add &= r_C.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_block_cols - 1]) * f_factor_A,
				row_cumsums[n_block_rows - 1], col_cumsums[n_block_cols - 1]); // r_A
			b_add &= r_C.Append_Block(r_t_block_B.block(0, 0, row_sizes[n_width_def_block_row_B], col_sizes[n_block_cols - 1]) * f_factor_B,
				row_cumsums[n_width_def_block_row_B], col_cumsums[n_block_cols - 1]); // r_B
			b_add &= r_C.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_B]) * f_factor_B,
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_B]); // r_B
		} else if(n_extent_method == 1) {
			_ASSERTE(n_block_rows + n_block_cols - 1 >= 3); // make sure there is enough space for all the blocks to define size of the matrices

			size_t n_width_def_block_row_A = rand() % (n_block_rows - 1);
			size_t n_height_def_block_col_A = rand() % (n_block_cols - 1); // don't want the blocks to meet ...

			bool b_first_elem = !(rand() & 1);
			if(b_first_elem)
				b_add &= r_B.Append_Block(r_t_block_A.block(0, 0, row_sizes[0], col_sizes[0]), 0, 0);
			// this one isn't entirely required

			b_add &= r_B.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_block_cols - 1]),
				row_cumsums[n_block_rows - 1], col_cumsums[n_block_cols - 1]);
			// define size of B in lower / left corner

			b_add &= r_A.Append_Block(r_t_block_B.block(0, 0, row_sizes[n_width_def_block_row_A], col_sizes[n_block_cols - 1]),
				row_cumsums[n_width_def_block_row_A], col_cumsums[n_block_cols - 1]);
			b_add &= r_A.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_A]),
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_A]);
			// define size of A in arbitrary rows / columns in the last column / row, respectively

			if(b_first_elem)
				b_add &= r_C.Append_Block(r_t_block_A.block(0, 0, row_sizes[0], col_sizes[0]) * f_factor_B, 0, 0); // r_B
			b_add &= r_C.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_block_cols - 1]) * f_factor_B,
				row_cumsums[n_block_rows - 1], col_cumsums[n_block_cols - 1]); // r_B
			b_add &= r_C.Append_Block(r_t_block_B.block(0, 0, row_sizes[n_width_def_block_row_A], col_sizes[n_block_cols - 1]) * f_factor_A,
				row_cumsums[n_width_def_block_row_A], col_cumsums[n_block_cols - 1]); // r_A
			b_add &= r_C.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_A]) * f_factor_A,
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_A]); // r_A
		} else {
			_ASSERTE(n_block_rows + n_block_cols - 1 >= 4); // make sure there is enough space for all the blocks to define size of the matrices

			size_t n_width_def_block_row_A = rand() % n_block_rows;
			size_t n_width_def_block_row_B = rand() % n_block_rows;
			while(n_width_def_block_row_A == n_width_def_block_row_B)
				n_width_def_block_row_B = rand() % n_block_rows; // don't want the blocks to meet ...
			size_t n_height_def_block_col_A = rand() % n_block_cols;
			while((n_width_def_block_row_A == n_block_rows - 1 ||
			   n_width_def_block_row_B == n_block_rows - 1) && n_height_def_block_col_A == n_block_cols - 1)
				n_height_def_block_col_A = rand() % n_block_cols; // don't want the blocks to meet ...
			size_t n_height_def_block_col_B = rand() % n_block_cols;
			while(((n_width_def_block_row_A == n_block_rows - 1 ||
			   n_width_def_block_row_B == n_block_rows - 1) && (n_height_def_block_col_B == n_block_cols - 1)) ||
			   n_height_def_block_col_B == n_height_def_block_col_A)
				n_height_def_block_col_B = rand() % n_block_cols; // don't want the blocks to meet ...
			// generates different positions for the blocks (requires at least 3 by 2 blocks matrix)

#ifdef _DEBUG
			std::set<std::pair<size_t, size_t> > coverage_map;
			coverage_map.insert(std::make_pair(row_cumsums[n_width_def_block_row_A], col_cumsums[n_block_cols - 1]));
			_ASSERTE(!coverage_map.count(std::make_pair(row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_A])));
			coverage_map.insert(std::make_pair(row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_A]));
			_ASSERTE(!coverage_map.count(std::make_pair(row_cumsums[n_width_def_block_row_B], col_cumsums[n_block_cols - 1])));
			coverage_map.insert(std::make_pair(row_cumsums[n_width_def_block_row_B], col_cumsums[n_block_cols - 1]));
			_ASSERTE(!coverage_map.count(std::make_pair(row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_B])));
			//coverage_map.insert(std::make_pair(row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_B)); // not required
#endif // _DEBUG
			// just make sure the above logic is correct and we do not cause block collision

			b_add &= r_A.Append_Block(r_t_block_A.block(0, 0, row_sizes[n_width_def_block_row_A], col_sizes[n_block_cols - 1]),
				row_cumsums[n_width_def_block_row_A], col_cumsums[n_block_cols - 1]);
			b_add &= r_A.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_A]),
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_A]);
			// define size of A in arbitrary rows / columns in the last column / row, respectively

			b_add &= r_B.Append_Block(r_t_block_B.block(0, 0, row_sizes[n_width_def_block_row_B], col_sizes[n_block_cols - 1]),
				row_cumsums[n_width_def_block_row_B], col_cumsums[n_block_cols - 1]);
			b_add &= r_B.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_B]),
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_B]);
			// define size of B in arbitrary rows / columns in the last column / row, respectively

			b_add &= r_C.Append_Block(r_t_block_A.block(0, 0, row_sizes[n_width_def_block_row_A], col_sizes[n_block_cols - 1]) * f_factor_A,
				row_cumsums[n_width_def_block_row_A], col_cumsums[n_block_cols - 1]);
			b_add &= r_C.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_A]) * f_factor_A,
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_A]);
			b_add &= r_C.Append_Block(r_t_block_B.block(0, 0, row_sizes[n_width_def_block_row_B], col_sizes[n_block_cols - 1]) * f_factor_B,
				row_cumsums[n_width_def_block_row_B], col_cumsums[n_block_cols - 1]);
			b_add &= r_C.Append_Block(r_t_block_C.block(0, 0, row_sizes[n_block_rows - 1], col_sizes[n_height_def_block_col_B]) * f_factor_B,
				row_cumsums[n_block_rows - 1], col_cumsums[n_height_def_block_col_B]);
			// add all the blocks to C as well (mind the factors)
		}
		// append blocks to make the matrices of equal size

		for(int i = 0, n = rand() % n_max_blocks; i < n; ++ i) {
			int r = rand() % n_block_rows;
			int c = rand() % n_block_cols;
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_A.t_FindBlock(row_cumsums[r], col_cumsums[c], row_sizes[r], col_sizes[c]) +=
				p_block->block(0, 0, row_sizes[r], col_sizes[c]);
			r_C.t_FindBlock(row_cumsums[r], col_cumsums[c], row_sizes[r], col_sizes[c]) +=
				p_block->block(0, 0, row_sizes[r], col_sizes[c]) * f_factor_A;
			// add block to the r_A matrix, and to the r_C matrix
		}
		for(int i = 0, n = rand() % n_max_blocks; i < n; ++ i) {
			int r = rand() % n_block_rows;
			int c = rand() % n_block_cols;
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_B.t_FindBlock(row_cumsums[r], col_cumsums[c], row_sizes[r], col_sizes[c]) +=
				p_block->block(0, 0, row_sizes[r], col_sizes[c]);
			r_C.t_FindBlock(row_cumsums[r], col_cumsums[c], row_sizes[r], col_sizes[c]) +=
				p_block->block(0, 0, row_sizes[r], col_sizes[c]) * f_factor_B;
			// add block to the r_B matrix, and to the r_C matrix
		}
		if(!b_add)
			throw std::runtime_error("Append_Block() failed");
		// add random blocks to r_A, r_C and to r_B, r_C (r_C now contains result, r_A and r_B contains parts)
	}

	/**
	 *	@brief generates two random matrices of the same size which can not be added (contain block conflicts)
	 *
	 *	@tparam _TyDenseBlock is type of dense block matrix
	 *
	 *	@param[out] r_A is the first matrix
	 *	@param[out] r_B is the second matrix
	 *	@param[in] r_t_block_A is dense seed matrix
	 *	@param[in] r_t_block_B is dense seed matrix
	 *	@param[in] r_t_block_C is dense seed matrix
	 *	@param[in] b_free_block_size is free block size flag (if set, blocks in the output
	 *		matrices will have any sizes from 1x1 up to the size of seed blocks; if not set,
	 *		the block size is fixed to block_Size and block_Size2)
	 *	@param[in] n_block_rows is number of block rows in the output matrices
	 *	@param[in] n_block_cols is number of block columns in the output matrices
	 *	@param[in] n_max_blocks is number of blocks to fill in the matrices
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	template <class _TyDenseBlock>
	static void Generate_UnAddable_Matrices(CUberBlockMatrix &r_A, CUberBlockMatrix &r_B,
		const _TyDenseBlock &r_t_block_A, const _TyDenseBlock &r_t_block_B,
		const _TyDenseBlock &r_t_block_C, bool b_free_block_size, size_t n_block_rows = 10,
		size_t n_block_cols = 10, size_t n_max_blocks = 10) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(n_block_cols > 1 || n_block_rows > 1);
		// this function must have some space in order to generate different layouts

		r_A.Clear();
		r_B.Clear();
		// make sure the matrices are empty

		std::vector<size_t> row_sizes_A, col_sizes_A;
		std::vector<size_t> row_sizes_B, col_sizes_B;
		std::vector<size_t> occupy_col_list_A, occupy_row_list_A;
		std::vector<size_t> occupy_col_list_B, occupy_row_list_B;
		for(;;) {
			row_sizes_A.clear();
			col_sizes_A.clear();
			row_sizes_B.clear();
			col_sizes_B.clear();
			occupy_col_list_A.clear();
			occupy_row_list_A.clear();
			occupy_col_list_B.clear();
			occupy_row_list_B.clear();
			// clear the lists

			if(b_free_block_size) {
				size_t n_max_rows = r_t_block_A.rows();
				size_t n_max_cols = r_t_block_A.cols();

				_ASSERTE(n_max_rows == r_t_block_B.rows() && n_max_rows == r_t_block_C.rows());
				_ASSERTE(n_max_cols == r_t_block_B.cols() && n_max_cols == r_t_block_C.cols());
				// the submatrices of these are used as seeds, make sure they are big enough

				for(size_t n_row = 0; n_row < n_block_rows; ++ n_row)
					row_sizes_A.push_back(rand() % n_max_rows + 1);
				for(size_t n_col = 0; n_col < n_block_cols; ++ n_col)
					col_sizes_A.push_back(rand() % n_max_cols + 1);
				// generate arbitrary size blocks
			} else {
				_ASSERTE(block_Size2 > block_Size);
				_ASSERTE(r_t_block_A.rows() == block_Size2 &&
					r_t_block_B.rows() == block_Size2 && r_t_block_C.rows() == block_Size2);
				_ASSERTE(r_t_block_A.cols() == block_Size2 &&
					r_t_block_B.cols() == block_Size2 && r_t_block_C.cols() == block_Size2);
				// the submatrices of these are used as seeds, make sure they are big enough

				for(size_t n_row = 0; n_row < n_block_rows; ++ n_row)
					row_sizes_A.push_back(((rand() & 1)? block_Size : block_Size2));
				for(size_t n_col = 0; n_col < n_block_cols; ++ n_col)
					col_sizes_A.push_back(((rand() & 1)? block_Size : block_Size2));
				// generate block of size or size2
			}
			// generate row and column sizes of A matrix

			row_sizes_B = row_sizes_A;
			col_sizes_B = col_sizes_A;
			// copy them to the B matrix

			bool b_change_col_layout = rand() & 1;
			if(b_change_col_layout) {
				size_t n_col = rand() % col_sizes_B.size();
				size_t n_col2 = rand() % col_sizes_B.size();
				size_t n_retry_num = 0;
				while(n_retry_num < 1000 && col_sizes_B[n_col] == col_sizes_B[n_col2]) {
					n_col2 = rand() % col_sizes_B.size();
					++ n_retry_num;
				}
				if(n_retry_num == 1000)
					b_change_col_layout = false; // maybe we generated all columns of the same width (freak accident)
				else {
					std::swap(col_sizes_B[n_col], col_sizes_B[n_col2]);
					// create difference in layouts

					if(rand() & 1)
						std::swap(n_col, n_col2);
					// choose where to put nonzero block (one of the columns is enough)

					bool b_ambiguous = false;
					if((n_col > 0 && col_sizes_B[n_col] + col_sizes_B[n_col - 1] <= col_sizes_B[n_col2]) ||
					   (n_col2 + 1 < col_sizes_B.size() && col_sizes_B[n_col2] + col_sizes_B[n_col2 + 1] <= col_sizes_B[n_col]))
						b_ambiguous = true;
					// sometimes placing blocks on either side of the column doesn't generate desired layout conflicts

					if(b_ambiguous || (rand() & 1) != 0) {
						occupy_col_list_A.push_back(n_col);
						occupy_col_list_B.push_back(n_col);
						// in the column itself
					} else {
						occupy_col_list_A.push_back(n_col2);
						occupy_col_list_B.push_back(n_col2);
						if(n_col > 0)
							occupy_col_list_A.push_back(n_col - 1);
						if(n_col + 1 < col_sizes_B.size())
							occupy_col_list_A.push_back(n_col + 1);
						if(n_col > 0)
							occupy_col_list_B.push_back(n_col - 1);
						if(n_col + 1 < col_sizes_B.size())
							occupy_col_list_B.push_back(n_col + 1);
						// on either side of the column
					}
					// decide where to put nonzero blocks in r_A an r_B (must be synchronized,
					// otherwise it sometimes generates matrices that can be added)
				}
			}
			// create a change in column layout and mark where blocks need to be inserted in order
			// that the change manifests itself (otherwise the column could be merged, if empty)

			bool b_change_row_layout = !b_change_col_layout || (rand() & 1) != 0;
			if(b_change_row_layout) {
				size_t n_row = rand() % row_sizes_B.size();
				size_t n_row2 = rand() % row_sizes_B.size();
				size_t n_retry_num = 0;
				while(n_retry_num < 1000 && row_sizes_B[n_row] == row_sizes_B[n_row2]) {
					n_row2 = rand() % row_sizes_B.size();
					++ n_retry_num;
				}
				if(n_retry_num == 1000)
					b_change_row_layout = false; // maybe we generated all rows of the same width (freak accident)
				else {
					std::swap(row_sizes_B[n_row], row_sizes_B[n_row2]);
					// create difference in layouts

					if(rand() & 1)
						std::swap(n_row, n_row2);
					// choose where to put nonzero block (one of the rowumns is enough)

					bool b_ambiguous = false;
					if((n_row > 0 && row_sizes_B[n_row] + row_sizes_B[n_row - 1] <= row_sizes_B[n_row2]) ||
					   (n_row2 + 1 < row_sizes_B.size() && row_sizes_B[n_row2] + row_sizes_B[n_row2 + 1] <= row_sizes_B[n_row]))
						b_ambiguous = true;
					// sometimes placing blocks on either side of the row doesn't generate desired layout conflicts

					if(b_ambiguous || (rand() & 1) != 0) {
						occupy_row_list_A.push_back(n_row);
						occupy_row_list_B.push_back(n_row);
						// in the row itself
					} else {
						occupy_row_list_A.push_back(n_row2);
						occupy_row_list_B.push_back(n_row2);
						if(n_row > 0)
							occupy_row_list_A.push_back(n_row - 1);
						if(n_row + 1 < row_sizes_B.size())
							occupy_row_list_A.push_back(n_row + 1);
						if(n_row > 0)
							occupy_row_list_B.push_back(n_row - 1);
						if(n_row + 1 < row_sizes_B.size())
							occupy_row_list_B.push_back(n_row + 1);
						// on either side of the row
					}
					// decide where to put nonzero blocks in r_A an r_B (must be synchronized,
					// otherwise it sometimes generates matrices that can be added)
				}
			}
			// create a change in column layout and mark where blocks need to be inserted in order
			// that the change manifests itself (otherwise the column could be merged, if empty)

			if(!b_change_row_layout && !b_change_col_layout)
				continue;
			// in case either layout was changed, try again

			break;
		}
		// in case the layout change failed, do not proceed - the addition would succeed

		std::vector<size_t> row_cumsums_A, col_cumsums_A;
		std::vector<size_t> row_cumsums_B, col_cumsums_B;
		row_cumsums_A.push_back(0);
		col_cumsums_A.push_back(0);
		row_cumsums_B.push_back(0);
		col_cumsums_B.push_back(0);
		for(size_t n_row = 0; n_row < n_block_rows - 1; ++ n_row) {
			row_cumsums_A.push_back(row_cumsums_A.back() + row_sizes_A[n_row]);
			row_cumsums_B.push_back(row_cumsums_B.back() + row_sizes_B[n_row]);
		}
		for(size_t n_col = 0; n_col < n_block_cols - 1; ++ n_col) {
			col_cumsums_A.push_back(col_cumsums_A.back() + col_sizes_A[n_col]);
			col_cumsums_B.push_back(col_cumsums_B.back() + col_sizes_B[n_col]);
		}
		// generate row / column cumsums

		bool b_add = true;
		if(rand() & 1) {
			if(rand() & 1)
				b_add &= r_A.Append_Block(r_t_block_A.block(0, 0, row_sizes_A[0], col_sizes_A[0]), 0, 0);
			b_add &= r_A.Append_Block(r_t_block_C.block(0, 0, row_sizes_A[n_block_rows - 1], col_sizes_A[n_block_cols - 1]),
				row_cumsums_A[n_block_rows - 1], col_cumsums_A[n_block_cols - 1]);
			b_add &= r_B.Append_Block(r_t_block_B.block(0, 0, row_sizes_B[0], col_sizes_B[n_block_cols - 1]),
				0, col_cumsums_B[n_block_cols - 1]);
			b_add &= r_B.Append_Block(r_t_block_C.block(0, 0, row_sizes_B[n_block_rows - 1], col_sizes_B[0]),
				row_cumsums_B[n_block_rows - 1], 0);
		} else {
			if(rand() & 1)
				b_add &= r_B.Append_Block(r_t_block_A.block(0, 0, row_sizes_B[0], col_sizes_B[0]), 0, 0);
			b_add &= r_B.Append_Block(r_t_block_C.block(0, 0, row_sizes_B[n_block_rows - 1], col_sizes_B[n_block_cols - 1]),
				row_cumsums_B[n_block_rows - 1], col_cumsums_B[n_block_cols - 1]);
			b_add &= r_A.Append_Block(r_t_block_B.block(0, 0, row_sizes_A[0], col_sizes_A[n_block_cols - 1]),
				0, col_cumsums_A[n_block_cols - 1]);
			b_add &= r_A.Append_Block(r_t_block_C.block(0, 0, row_sizes_A[n_block_rows - 1], col_sizes_A[0]),
				row_cumsums_A[n_block_rows - 1], 0);
		}
		// append blocks to make the matrices of equal size

		for(size_t i = 0, n = occupy_row_list_A.size(); i < n; ++ i) {
			size_t r = occupy_row_list_A[i];
			int c = rand() % n_block_cols;
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_A.t_FindBlock(row_cumsums_A[r], col_cumsums_A[c], row_sizes_A[r], col_sizes_A[c]) +=
				p_block->block(0, 0, row_sizes_A[r], col_sizes_A[c]);
			// add block to the r_A matrix
		}
		for(size_t i = 0, n = occupy_col_list_A.size(); i < n; ++ i) {
			int r = rand() % n_block_rows;
			size_t c = occupy_col_list_A[i];
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_A.t_FindBlock(row_cumsums_A[r], col_cumsums_A[c], row_sizes_A[r], col_sizes_A[c]) +=
				p_block->block(0, 0, row_sizes_A[r], col_sizes_A[c]);
			// add block to the r_A matrix
		}
		// make sure the columns and rows that make the layouts different in r_A are occupied

		for(size_t i = 0, n = occupy_row_list_B.size(); i < n; ++ i) {
			size_t r = occupy_row_list_B[i];
			int c = rand() % n_block_cols;
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_B.t_FindBlock(row_cumsums_B[r], col_cumsums_B[c], row_sizes_B[r], col_sizes_B[c]) +=
				p_block->block(0, 0, row_sizes_B[r], col_sizes_B[c]);
			// add block to the r_B matrix
		}
		for(size_t i = 0, n = occupy_col_list_B.size(); i < n; ++ i) {
			int r = rand() % n_block_rows;
			size_t c = occupy_col_list_B[i];
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_B.t_FindBlock(row_cumsums_B[r], col_cumsums_B[c], row_sizes_B[r], col_sizes_B[c]) +=
				p_block->block(0, 0, row_sizes_B[r], col_sizes_B[c]);
			// add block to the r_B matrix
		}
		// make sure the columns and rows that make the layouts different in r_B are occupied

		for(int i = 0, n = rand() % n_max_blocks; i < n; ++ i) {
			int r = rand() % n_block_rows;
			int c = rand() % n_block_cols;
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_A.t_FindBlock(row_cumsums_A[r], col_cumsums_A[c], row_sizes_A[r], col_sizes_A[c]) +=
				p_block->block(0, 0, row_sizes_A[r], col_sizes_A[c]);
			// add block to the r_A matrix
		}
		for(int i = 0, n = rand() % n_max_blocks; i < n; ++ i) {
			int r = rand() % n_block_rows;
			int c = rand() % n_block_cols;
			const _TyDenseBlock *p_block;
			{
				int b = rand() % 3;
				p_block = (b == 0)? &r_t_block_A : (b == 1)? &r_t_block_B : &r_t_block_C;
			}
			// pick row / col / block at random

			r_B.t_FindBlock(row_cumsums_B[r], col_cumsums_B[c], row_sizes_B[r], col_sizes_B[c]) +=
				p_block->block(0, 0, row_sizes_B[r], col_sizes_B[c]);
			// add block to the r_B matrix
		}
		if(!b_add)
			throw std::runtime_error("Append_Block() failed");
		// add random blocks to r_A and to r_B
	}

	/**
	 *	@brief performs a simple Cholesky test with a matrix from University of Florida Sparse Matrix Collection
	 *	@param[in] p_s_filename is name of a file with matrix-market Cholesky candidate matrix
	 */
	static void Simple_UFLSMC_CholTest(const char *p_s_filename);

	/**
	 *	@brief parses a line while skipping comments
	 *
	 *	@param[in] p_fr is the input file
	 *	@param[out] r_s_line is the string to read the line into
	 *	@param[in,out] r_n_cur_line is line counter (gets incremented
	 *		on every line read and also on commented lines skipped)
	 *
	 *	@return Returns true on success, false on failure or on end of file.
	 *
	 *	@note This calls CParserBase inside.
	 *	@note This function throws std::bad_alloc.
	 */
	static bool ReadLine(FILE *p_fr, std::string &r_s_line, int &r_n_cur_line); // throw(std::bad_alloc)

	/**
	 *	@brief loads a sparse matrix in matrix-market format
	 *	@param[in] p_s_filename is name of a file with matrix-market matrix
	 *	@return Returns a pointer ot a sparse matrix on success, or 0 on failure.
	 */
	static cs *p_LoadMM(const char *p_s_filename); // throw(std::bad_alloc)
};

#endif // !__UBER_BLOCK_MATRIX_UNIT_TESTS_INCLUDED
