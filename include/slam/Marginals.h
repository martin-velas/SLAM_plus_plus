/*
								+---------------------------------+
								|                                 |
								| ***  Marginals calculation  *** |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2013 |
								|                                 |
								|           Marginals.h           |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __MARGINALS_INCLUDED
#define __MARGINALS_INCLUDED

/**
 *	@file include/slam/Marginals.h
 *	@date 2013
 *	@author -tHE SWINe-
 *	@brief calculation of marginal covariances in sparse systems
 */

#include "slam/BlockMatrix.h"
//#include "slam/Timer.h" // included from slam/BlockMatrix.h
//#include "slam/Integer.h" // included from slam/BlockMatrix.h
#include "slam/IncrementalPolicy.h" // block matrix part names
#include "eigen/Eigen/Core"
#include "slam/OrderingMagic.h"

/** \addtogroup covs
 *	@{
 */

/**
 *	@def __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
 *	@brief if defined, dense lookup is used to get blocks of the marginals
 *	@note This quickly runs out of memory (100k would not fit), and it is
 *		slower for larger problems than without it (10k with 0.49 sec,
 *		without 0.24 sec).
 */
//#define __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP

/**
 *	@def __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP
 *	@brief if defined, blockwise diagonal loop is used instead of elementwise
 *
 *	with block diag:	0.19 sec 10k, 0.31 sphere, 4.9 sec 100k, 0.163 msec simple example
 *	without:			0.21 sec 10k, 0.33 sphere, 5.3 sec 100k, 0.204 msec simple example
 *
 *	@note This involves some extra calculation but saves memory traffic,
 *		the payoff will likely diminissh with growing block size.
 */
#define __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP

/**
 *	@def __MARGINALS_COMPACT_UPDATE
 *	@brief if defined, \ref CMarginals::Update_BlockDiagonalMarginals_FBS_ExOmega() will
 *		use less memory in exchange for potentially unaligned operations
 */
#define __MARGINALS_COMPACT_UPDATE

/**
 *	@brief prototype marginal covariance methods implementation
 *
 *	@todo (Re)implement non-FBS version of the fast recurrent.
 *	@todo Implement parallel calculation of more denser matrices or
 *		on-demand calculation of off-diagonal blocks using recurrent (rather simple).
 */
class CMarginals {
protected:
public:
	/**
	 *	@brief reference function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Ref(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);
		Eigen::MatrixXd &R_inv = r_marginals; // R_inv = S
		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			double *p_column = &R_inv.col(i)(0);
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			r_R.UpperTriangular_Solve(p_column, n, n_block_col); // backsub, only the nonzero part of the column (started at (block) column which contains column i, with no loss of generality)
		}
		r_marginals = R_inv * R_inv.transpose(); // C = SS^T, might need some memory
		// calculate the covariance (assume that this is correct)

		// 2015-08-17 - seems to work ok
	}

	/**
	 *	@brief slow function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Slow(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);
		r_marginals.setZero(); // !!

		Eigen::MatrixXd R_inv_column(n, 1); // R_inv = S
		double *p_column = &R_inv_column.col(0)(0);
		// get dense column data from the Eigen matrix (actually only need one)

		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			size_t UNUSED(n_block_column_size);
			size_t n_block_column = r_R.n_Find_BlockColumn(i, n_block_column_size);
			_ASSERTE(n_block_col == n_block_column); // should be the same
			// get which block column contains column i (optimize this away, probably need to use it when resuming)

			//_ASSERTE(n_block_column_size <= n_diag_band_width); // make this into a run-time check in the production code
			// make sure it is not longer than the diagonal (otherwise we will not have enough backlog to calculate all the off-diagonal elements)

			r_R.UpperTriangular_Solve(p_column, n, n_block_col); // backsub, only the nonzero part of the column (started at (block) column which contains column i, with no loss of generality)
			// this seems to be O(i) divisions + O(nnz) MADs in the given (block) column range
			// that sums up to O(n^2/2) divisions + O(nnz log(nnz))?? MADs ... some quadratute anyways

			std::vector<double> backsub_test(n, 0);
			backsub_test[i] = 1; // !!
			r_R.UpperTriangular_Solve(&backsub_test[0], n); // full backsub
			_ASSERTE(!memcmp(p_column, &backsub_test[0], n * sizeof(double)));
			// make sure that the result is correct

			_ASSERTE((Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(p_column + i + 1, n - i - 1).norm() == 0)); // double pars required because of the comma in Map params
			// everything below i is zero (true)

			for(size_t k = 0; k <= i; ++ k) {
				for(size_t j = 0; j <= i; ++ j)
					r_marginals(j, k) += p_column[j] * p_column[k]; // it is symmetric, indexing arbitrary
			}
			// accumulate the entries of the covariace matrix. this is O(n^3/2) MADs for the full matrix
			// note that to calculate even only the diagonal, we need full columns
		}
	}

	/**
	 *	@brief fast function that calculates a column band of the dense marginals matrix
	 *
	 *	@param[out] r_marginals is the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_start_column is zero-based index of the first column (in elements)
	 *	@param[in] n_end_column is zero-based index of one past the last column (in elements)
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *	@param[in] b_lower_diag_only is lower-diagonal flag
	 *	@param[in] b_band_only is band flag (only the specified band of the matrix is then stored)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Fast_ColumnBand(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_start_column, size_t n_end_column,
		const size_t *p_inv_order, size_t n_order_size, bool b_lower_diag_only = false,
		bool b_band_only = false) // throw(std::bad_alloc)
	{
		_ASSERTE(!b_lower_diag_only);
		// missing code to correctly resume, always calculating full

		_ASSERTE(p_inv_order);
		_ASSERTE(n_order_size == r_R.n_BlockColumn_Num());
		// make sure the ordering is there and has valid size

		const size_t n = r_R.n_Column_Num(); // in elements
		_ASSERTE(n_start_column <= n_end_column);
		_ASSERTE(n_end_column <= n); // not more than that
		r_marginals.resize(n, (b_band_only)? n_end_column - n_start_column : n); // should already have the size if called from Calculate_DenseMarginals_Fast(), then it is a no-op

		Eigen::VectorXd perm_column(n);

		size_t n_block_col = -1, n_col_remains = 1;
		const size_t nb = r_R.n_BlockColumn_Num();
		if(n_start_column) {
			/*n_block_col = r_R.n_Find_BlockColumn(n_start_column, n_col_remains);
			_ASSERTE(n_block_col != size_t(-1));
			// get a column

			size_t n_col_base = r_R.n_BlockColumn_Base(n_block_col);
			_ASSERTE(n_col_base <= n_start_column); // make sure that the right column was found
			_ASSERTE(n_col_remains > n_start_column - n_col_base); // make sure that the start column is inside the block column
			n_col_remains -= n_start_column - n_col_base; // get how many columns remain*/
			// this seems flawed, can't ignore the ordering if the blocks are not the same size

			//size_t n_col_base;
			for(size_t j = 0, n_one_pos = n_start_column; j < n_order_size; ++ j) {
				size_t n_col = p_inv_order[j];
				size_t n_block_base = r_R.n_BlockColumn_Base(n_col);
				size_t n_block_size = r_R.n_BlockColumn_Column_Num(n_col);
				if(n_one_pos < n_block_size) {
					//n_col_base = n_block_base; // unused
					n_col_remains = n_block_size /*- 1*/ - n_one_pos; // like this, without the "- 1"
					n_block_col = j; // before permutation!
					break;
				} else
					n_one_pos -= n_block_size;
			}
			_ASSERTE(n_block_col != size_t(-1));
			// need to find the correct column under the permutation

			++ n_col_remains; // will decrement at the beginning of the loop, compensate for that
		}
		for(size_t i = n_start_column, _n = std::min(n, n_end_column); i < _n; ++ i) {
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(p_inv_order[++ n_block_col]); // can't ignore the order here!
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			_ASSERTE(n_block_col < nb);
			//_ASSERTE(n_block_col + 1 == nb || i + n_col_remains == r_R.n_BlockColumn_Base(n_block_col + 1)); // will only work with a single block size
			// make sure that the numbers are correct

			double *p_column = &r_marginals.col((b_band_only)? i - n_start_column : i)(0);
#ifdef _DEBUG
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			r_R.InversePermute_LeftHandSide_Vector(&perm_column(0), p_column, n, p_inv_order, n_order_size);
			// this is mostly a waste of time, as the vector is mostly zeros
#endif // _DEBUG

			size_t p = p_inv_order[n_block_col]; // the block column is the one we want
			size_t n_perm_col = (p + 1 < nb)? r_R.n_BlockColumn_Base(p + 1) - n_col_remains : // quite simple
				r_R.n_BlockColumn_Base(p) + r_R.n_BlockColumn_Column_Num(p) - n_col_remains; // one more reference
			_ASSERTE(perm_column(n_perm_col) == 1); // all we need to do is write a single one here
			// calculate inverse permutation

#ifndef _DEBUG
			perm_column.setZero();
			perm_column(n_perm_col) = 1;
			// form the vector with a single 1 in it already permuted
#endif // !_DEBUG

			r_R.UpperTriangularTranspose_Solve(&perm_column(0), n, p/*n_block_col*/);
			/*if(b_lower_diag_only)
				r_R.UpperTriangular_Solve(&perm_column(0), n, n_block_col, r_R.n_BlockColumn_Num() - 1);
			else*/
				r_R.UpperTriangular_Solve(&perm_column(0), n);
			r_R.Permute_LeftHandSide_Vector(p_column, &perm_column(0), n, p_inv_order, n_order_size);
			// solve for the whole column thing, generates one column at a time
		}

		// 2015-08-17 - seems to work ok
	}

	/**
	 *	@brief fast FBS function that calculates dense marginals matrix
	 *
	 *	@tparam CBlockMatrixTypelist is list of Eigen::Matrix block sizes
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_start_column is zero-based index of the first column (in elements)
	 *	@param[in] n_end_column is zero-based index of one past the last column (in elements)
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *	@param[in] b_lower_diag_only is lower-diagonal flag
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Calculate_DenseMarginals_Fast_ColumnBand_FBS(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_start_column, size_t n_end_column,
		const size_t *p_inv_order, size_t n_order_size, bool b_lower_diag_only = false) // throw(std::bad_alloc)
	{
		_ASSERTE(!b_lower_diag_only);
		// missing code to correctly resume, always calculating full

		_ASSERTE(p_inv_order);
		_ASSERTE(n_order_size == r_R.n_BlockColumn_Num());
		// make sure the ordering is there and has valid size

		const size_t n = r_R.n_Column_Num(); // in elements
		_ASSERTE(n_start_column <= n_end_column);
		_ASSERTE(n_end_column <= n); // not more than that
		r_marginals.resize(n, n); // should already have the size if called from Calculate_DenseMarginals_Fast(), then it is a no-op

		Eigen::VectorXd perm_column(n);

		enum {
			b_single_block_size = CTypelistLength<CBlockMatrixTypelist>::n_result == 1,
			n_first_block_size = fbs_ut::CEigenToDimension<typename
				CBlockMatrixTypelist::_TyHead>::_TyResult::n_column_num
		};
		// optimize for just a single size in the typelist (compile-time constants)

#ifdef _DEBUG
		const bool _b_single_block_size = b_single_block_size != 0;
		const int _n_first_block_size = n_first_block_size;
#endif // _DEBUG
		// otherwise not visible to debugger

		size_t n_block_col = -1, n_col_remains = 1;
		const size_t nb = r_R.n_BlockColumn_Num();
		if(n_start_column) {
			size_t n_col_base;
			if(b_single_block_size) { // compile-time constant, should optimize away
				n_block_col = n_start_column / n_first_block_size;
				n_col_remains = n_first_block_size;
				size_t UNUSED_VAR(n_remains_check);
				_ASSERTE(n_block_col == r_R.n_Find_BlockColumn(n_start_column, n_remains_check));
				_ASSERTE(n_remains_check == n_col_remains);
				n_col_base = n_start_column - n_start_column % n_first_block_size;
				_ASSERTE(n_col_base == r_R.n_BlockColumn_Base(n_block_col));
				// code, optimized for a single block size

				_ASSERTE(n_block_col != size_t(-1));
				_ASSERTE(n_col_base <= n_start_column); // make sure that the right column was found
				_ASSERTE(n_col_remains > n_start_column - n_col_base); // make sure that the start column is inside the block column
				n_col_remains -= n_start_column - n_col_base; // get how many columns remain
			} else {
				/*n_block_col = r_R.n_Find_BlockColumn(n_start_column, n_col_remains);
				n_col_base = r_R.n_BlockColumn_Base(n_block_col);*/
				// this seems flawed, can't ignore the ordering if the blocks are not the same size

				for(size_t j = 0, n_one_pos = n_start_column; j < n_order_size; ++ j) {
					size_t n_col = p_inv_order[j];
					size_t n_block_base = r_R.n_BlockColumn_Base(n_col);
					size_t n_block_size = r_R.n_BlockColumn_Column_Num(n_col);
					if(n_one_pos < n_block_size) {
						n_col_base = n_block_base;
						n_col_remains = n_block_size /*- 1*/ - n_one_pos; // like this, without the "- 1"
						n_block_col = j; // before permutation!
						break;
					} else
						n_one_pos -= n_block_size;
				}
				_ASSERTE(n_block_col != size_t(-1));
				// need to find the correct column under the permutation
			}
			// get a column

			++ n_col_remains; // will decrement at the beginning of the loop, compensate for that
		}
		for(size_t i = n_start_column, _n = std::min(n, n_end_column); i < _n; ++ i) {
			if(!(-- n_col_remains)) { // triggers in the first iteration, loads up column width
				if(b_single_block_size) { // compile-time constant, should optimize away
					n_col_remains = n_first_block_size;
					++ n_block_col;
				} else
					n_col_remains = r_R.n_BlockColumn_Column_Num(p_inv_order[++ n_block_col]); // can't ignore the order here!
			}
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			_ASSERTE(n_block_col < nb);
			_ASSERTE(n_block_col + 1 == nb || !b_single_block_size ||
				i + n_col_remains == r_R.n_BlockColumn_Base(n_block_col + 1)); // will only work with a single block size
			// make sure that the numbers are correct

			double *p_column = &r_marginals.col(i)(0);
#ifdef _DEBUG
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			r_R.InversePermute_LeftHandSide_Vector(&perm_column(0), p_column, n, p_inv_order, n_order_size);
			// this is mostly a waste of time, as the vector is mostly zeros
#endif // _DEBUG

			size_t p = p_inv_order[n_block_col]; // the block column is the one we want
			size_t n_perm_col = (p + 1 < nb)? r_R.n_BlockColumn_Base(p + 1) - n_col_remains : // quite simple
				r_R.n_BlockColumn_Base(p) + r_R.n_BlockColumn_Column_Num(p) - n_col_remains; // one more reference
			_ASSERTE(perm_column(n_perm_col) == 1); // all we need to do is write a single one here
			// calculate inverse permutation

#ifndef _DEBUG
			perm_column.setZero();
			perm_column(n_perm_col) = 1;
			// form the vector with a single 1 in it already permuted
#endif // !_DEBUG

			r_R.UpperTriangularTranspose_Solve_FBS<CBlockMatrixTypelist>(&perm_column(0), n, p/*n_block_col*/);
			/*if(b_lower_diag_only) { // t_odo - write a prototype for this as well!
				r_R.UpperTriangular_Solve_FBS<CBlockMatrixTypelist>(&perm_column(0), n, n_block_col, r_R.n_BlockColumn_Num() - 1);
			} else*/
				r_R.UpperTriangular_Solve_FBS<CBlockMatrixTypelist>(&perm_column(0), n);
			r_R.Permute_LeftHandSide_Vector(p_column, &perm_column(0), n, p_inv_order, n_order_size);
			// solve for the whole column thing, generates one column at a time
		}
	}

	/**
	 *	@brief fast FBS function that calculates a block of a dense marginals matrix
	 *
	 *	This calculates a dense subblock of the marginals matrix. It can be smaller than
	 *	the full marginals matrix (which has the same size as R). The starting column of
	 *	the block is given by n_start_column, and the number of columns is arbitrary
	 *	(but not exceeding the size of the marginals). The starting row of the block is always
	 *	zero. The number of rows of the block must match the sum of sizes of block rows
	 *	selected by p_inv_order and n_smaller_order_size (must be block aligned).
	 *
	 *	@tparam CBlockMatrixTypelist is list of Eigen::Matrix block sizes
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[in,out] r_marginals is filled with the marginals matrix (must come preallocated)
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_start_column is zero-based index of the first column (in elements)
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *	@param[in] n_smaller_order_size is size of the smaller ordering for the corresponding
	 *		block (less or equal to n_order_size)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This used to be called Calculate_DenseMarginals_Fast_ColumnBand_FBS() which was
	 *		unfortunate because the function actually stores only a subblock of the marginals matrix
	 *		rather than the dense matrix and the semantics depend on the type of the matrix
	 *		(Eigen::MatrixXd for full matrix and anything else for matrix band). Seemed wrong
	 *		to call them the same.
	 */
	template <class CBlockMatrixTypelist, class Derived0>
	static void Calculate_SubblockMarginals_Fast_ColumnBand_FBS(Eigen::MatrixBase<Derived0> &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_start_column,
		const size_t *p_inv_order, size_t n_order_size, size_t n_smaller_order_size) // throw(std::bad_alloc)
	{
		_ASSERTE(p_inv_order);
		_ASSERTE(n_order_size == r_R.n_BlockColumn_Num());
		// make sure the ordering is there and has valid size

		const size_t n_end_column = n_start_column + r_marginals.cols();
		const size_t n = r_R.n_Column_Num(); // in elements
		_ASSERTE(n_start_column <= n_end_column);
		_ASSERTE(n_end_column <= n); // not more than that

		_ASSERTE(size_t(r_marginals.rows()) <= n); // number of rows must be the same or smaller (the tail is not stored)
		_ASSERTE(r_marginals.cols() == n_end_column - n_start_column); // number of columns is given
		const size_t n_result_rows = r_marginals.rows();
		_ASSERTE(n_smaller_order_size <= n_order_size); // not more
		//_ASSERTE(n_result_rows <= n); // not more // already checked above
		_ASSERTE(n_result_rows < n || n_smaller_order_size == n_order_size); // equal, unless less rows than n are requested
#ifdef _DEBUG
		size_t n_smaller_order_size_check = n_order_size;
		{
			size_t n_elems = n;
			while(n_elems > n_result_rows) {
				_ASSERTE(n_smaller_order_size_check > 0);
				-- n_smaller_order_size_check; // here

				size_t n_block_size = r_R.n_BlockColumn_Column_Num(p_inv_order[n_smaller_order_size_check]);
				_ASSERTE(n_elems >= n_block_size);
				n_elems -= n_block_size;
				_ASSERTE(n_elems >= n_result_rows); // must be greater or equal
			}
			_ASSERTE(n_elems == n_result_rows);
		}
		_ASSERTE(n_smaller_order_size_check == n_smaller_order_size);
		// just makes sure that n_smaller_order_size is calculated correctly
#endif // _DEBUG

		Eigen::VectorXd perm_column(n);

		enum {
			b_single_block_size = CTypelistLength<CBlockMatrixTypelist>::n_result == 1,
			n_first_block_size = fbs_ut::CEigenToDimension<typename
				CBlockMatrixTypelist::_TyHead>::_TyResult::n_column_num
		};
		// optimize for just a single size in the typelist (compile-time constants)

#ifdef _DEBUG
		const bool _b_single_block_size = b_single_block_size != 0;
		const int _n_first_block_size = n_first_block_size;
#endif // _DEBUG
		// otherwise not visible to debugger

		size_t n_block_col = size_t(-1), n_col_remains = 1;
		const size_t nb = r_R.n_BlockColumn_Num();
		if(n_start_column) {
			size_t n_col_base;
			if(b_single_block_size) { // compile-time constant, should optimize away
				n_block_col = n_start_column / n_first_block_size;
				n_col_remains = n_first_block_size;
				size_t UNUSED_VAR(n_remains_check);
				_ASSERTE(n_block_col == r_R.n_Find_BlockColumn(n_start_column, n_remains_check));
				_ASSERTE(n_remains_check == n_col_remains);
				n_col_base = n_start_column - n_start_column % n_first_block_size;
				_ASSERTE(n_col_base == r_R.n_BlockColumn_Base(n_block_col));
				// code, optimized for a single block size

				_ASSERTE(n_block_col != size_t(-1));
				_ASSERTE(n_col_base <= n_start_column); // make sure that the right column was found
				_ASSERTE(n_col_remains > n_start_column - n_col_base); // make sure that the start column is inside the block column
				n_col_remains -= n_start_column - n_col_base; // get how many columns remain
			} else {
				/*n_block_col = r_R.n_Find_BlockColumn(n_start_column, n_col_remains);
				n_col_base = r_R.n_BlockColumn_Base(n_block_col);*/
				// this seems flawed, can't ignore the ordering if the blocks are not the same size

				for(size_t j = 0, n_one_pos = n_start_column; j < n_order_size; ++ j) {
					size_t n_col = p_inv_order[j];
					size_t n_block_base = r_R.n_BlockColumn_Base(n_col);
					size_t n_block_size = r_R.n_BlockColumn_Column_Num(n_col);
					if(n_one_pos < n_block_size) {
						n_col_base = n_block_base;
						n_col_remains = n_block_size /*- 1*/ - n_one_pos; // like this, without the "- 1"
						n_block_col = j; // before permutation!
						break;
					} else
						n_one_pos -= n_block_size;
				}
				_ASSERTE(n_block_col != size_t(-1));
				// need to find the correct column under the permutation
			}
			// get a column

			++ n_col_remains; // will decrement at the beginning of the loop, compensate for that
		}
		for(size_t i = n_start_column, _n = std::min(n, n_end_column); i < _n; ++ i) {
			if(!(-- n_col_remains)) { // triggers in the first iteration, loads up column width
				if(b_single_block_size) { // compile-time constant, should optimize away
					n_col_remains = n_first_block_size;
					++ n_block_col;
				} else
					n_col_remains = r_R.n_BlockColumn_Column_Num(p_inv_order[++ n_block_col]); // can't ignore the order here!
			}
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			_ASSERTE(n_block_col < nb);
			_ASSERTE(n_block_col + 1 == nb || !b_single_block_size ||
				i + n_col_remains == r_R.n_BlockColumn_Base(n_block_col + 1)); // this only works if b_single_block_size
			// make sure that the numbers are correct

			double *p_column = &r_marginals.col(i - n_start_column)(0);
#ifdef _DEBUG
			Eigen::VectorXd dbg_column(n);
			dbg_column.setZero();
			dbg_column(i) = 1;
			// make a column vector with a single 1 in it

			r_R.InversePermute_LeftHandSide_Vector(&perm_column(0), &dbg_column(0), n, p_inv_order, n_order_size);
			// this is mostly a waste of time, as the vector is mostly zeros

			size_t n_correct_perm = std::find(&perm_column(0),
				&perm_column(0) + perm_column.rows(), 1.0) - &perm_column(0);
			// sometimes it is not obvious why it does not work
#endif // _DEBUG

			size_t p, n_perm_col;
			//if(b_single_block_size) { // compile-time constant, should optimize away
				p = p_inv_order[n_block_col]; // the block column is the one we want
				n_perm_col = (p + 1 < nb)? r_R.n_BlockColumn_Base(p + 1) - n_col_remains : // quite simple
					r_R.n_BlockColumn_Base(p) + r_R.n_BlockColumn_Column_Num(p) - n_col_remains; // one more reference
				// this is not the problem, and works even with multiple block sizes; calculation of n_block_col was the problem
			/*} else {
				p = size_t(-1);
				for(size_t j = 0, n_one_pos = i; j < n_order_size; ++ j) {
					size_t n_col = p_inv_order[j];
					size_t n_block_base = r_R.n_BlockColumn_Base(n_col);
					size_t n_block_size = r_R.n_BlockColumn_Column_Num(n_col);
					if(n_one_pos < n_block_size) {
						n_perm_col = n_block_base + n_one_pos;
						p = n_col;
						break;
					} else
						n_one_pos -= n_block_size;
				}
				_ASSERTE(p != size_t(-1));
				// find where the one will be after the permutation (should still be faster
				// than reading & writing the vector in InversePermute_LeftHandSide_Vector())
			}*/
			_ASSERTE(perm_column(n_perm_col) == 1); // all we need to do is write a single one here (checks if the above _DEBUG block calculated it, otherwise it is uninitialized)
			// calculate inverse permutation

			perm_column.setZero();
			perm_column(n_perm_col) = 1;
			// form the vector with a single 1 in it already permuted (use this in release)

			r_R.UpperTriangularTranspose_Solve_FBS<CBlockMatrixTypelist>(&perm_column(0), n, p/*n_block_col*/); // always resumed
#ifdef _DEBUG
			dbg_column.setZero();
			dbg_column(n_perm_col) = 1;
			r_R.UpperTriangularTranspose_Solve_FBS<CBlockMatrixTypelist>(&dbg_column(0), n); // make sure that the resume point is calculated correctly
			_ASSERTE(dbg_column == perm_column); // should be bit-by-bit identical
#endif // _DEBUG
			/*if(b_lower_diag_only) { // t_odo - write a prototype for this as well!
				r_R.UpperTriangular_Solve_FBS<CBlockMatrixTypelist>(&perm_column(0),
					n, n_block_col, r_R.n_BlockColumn_Num() - 1);
			} else*/
				r_R.UpperTriangular_Solve_FBS<CBlockMatrixTypelist>(&perm_column(0), n/*,
					0, / *n_smaller_order_size,* / r_R.n_BlockColumn_Num() - 1*/); // t_odo - use n_smaller_order_size here // can't, would have to see where the blocks are ordered, and only if at the beginning / end, could columns be skipped
			r_R.Permute_LeftHandSide_Vector(p_column, &perm_column(0),
				n_result_rows, p_inv_order, n_smaller_order_size);
			// solve for the whole column thing, generates one column at a time
		}
	}

	/**
	 *	@brief fast FBS function that calculates dense marginals matrix
	 *
	 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Calculate_DenseMarginals_Fast_FBS(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, const size_t *p_inv_order, size_t n_order_size) // throw(std::bad_alloc)
	{
		Calculate_DenseMarginals_Fast_ColumnBand_FBS<CBlockMatrixTypelist>(r_marginals, r_R,
			0, r_R.n_Column_Num(), p_inv_order, n_order_size, false);
		// no need for 2nd implementation, just call this

		/*r_marginals.triangularView<Eigen::StrictlyUpper>() =
			r_marginals.triangularView<Eigen::StrictlyLower>().transpose();*/
		// transpose elements below diagonal to elements above it
	}

	/**
	 *	@brief reference function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] p_inv_order is pointer to the inverse ordering which was used to calculate r_R
	 *	@param[in] n_order_size is size of the ordering, in blocks
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Fast(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, const size_t *p_inv_order, size_t n_order_size) // throw(std::bad_alloc)
	{
#if 1
		Calculate_DenseMarginals_Fast_ColumnBand(r_marginals, r_R,
			0, r_R.n_Column_Num(), p_inv_order, n_order_size, false);
		// no need for 2nd implementation, just call this

		/*r_marginals.triangularView<Eigen::StrictlyUpper>() =
			r_marginals.triangularView<Eigen::StrictlyLower>().transpose();*/
		// transpose elements below diagonal to elements above it
#else // 1
		_ASSERTE(p_inv_order);
		_ASSERTE(n_order_size == r_R.n_BlockColumn_Num());
		// make sure the ordering is there and has valid size

		const size_t n = r_R.n_Column_Num(); // in elements
		r_marginals.resize(n, n);

		Eigen::VectorXd perm_column(n);
		size_t n_block_col = -1, n_col_remains = 1;
		const size_t nb = r_R.n_BlockColumn_Num();
		for(size_t i = 0; i < n; ++ i) {
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			_ASSERTE(n_block_col < nb);
			//_ASSERTE(n_block_col + 1 == nb || i + n_col_remains == r_R.n_BlockColumn_Base(n_block_col + 1)); // only works with single block size
			// make sure that the numbers are correct

			double *p_column = &r_marginals.col(i)(0);
#ifdef _DEBUG
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			r_R.InversePermute_LeftHandSide_Vector(&perm_column(0), p_column, n, p_inv_order, n_order_size);
			// this is mostly a waste of time, as the vector is mostly zeros
#endif // _DEBUG

			size_t p = p_inv_order[n_block_col]; // the block column is the one we want
			size_t n_perm_col = (p + 1 < nb)? r_R.n_BlockColumn_Base(p + 1) - n_col_remains : // quite simple
				r_R.n_BlockColumn_Base(p) + r_R.n_BlockColumn_Column_Num(p) - n_col_remains; // one more reference
			_ASSERTE(perm_column(n_perm_col) == 1); // all we need to do is write a single one here
			// calculate inverse permutation

#ifndef _DEBUG
			perm_column.setZero();
			perm_column(n_perm_col) = 1;
			// form the vector with a single 1 in it already permuted
#endif // !_DEBUG

			r_R.UpperTriangularTranspose_Solve(&perm_column(0), n, p/*n_block_col*/);
			r_R.UpperTriangular_Solve(&perm_column(0), n/*, n_block_col, r_R.n_BlockColumn_Num() - 1*/); // only calculates valid items below the diagonal
			r_R.Permute_LeftHandSide_Vector(p_column, &perm_column(0), n, p_inv_order, n_order_size);
			// solve for the whole column thing, generates one column at a time
		}
#endif // 1
	}

	/**
	 *	@brief fast parallel function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Fast_Parallel(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, const size_t *p_inv_order, size_t n_order_size) // throw(std::bad_alloc)
	{
		_ASSERTE(p_inv_order);
		_ASSERTE(n_order_size == r_R.n_BlockColumn_Num());
		// make sure the ordering is there and has valid size

		const size_t n = r_R.n_Column_Num(); // in elements
		r_marginals.resize(n, n);

#ifdef _OPENMP
		#pragma omp parallel
		{
			int n_tid = omp_get_thread_num();
			int n_thread_num = omp_get_num_threads();
			size_t n_start = n_tid * (n / n_thread_num);
			size_t n_end = (n_tid + 1 < n_thread_num)? n_start + n / n_thread_num : n;
			// split to bands to be processed in parallel

			Calculate_DenseMarginals_Fast_ColumnBand(r_marginals, r_R, n_start, n_end, p_inv_order, n_order_size, false);
			// process in parallel
		}
		// calculate the lower-triangular marginals

		/*r_marginals.triangularView<Eigen::StrictlyUpper>() =
			r_marginals.triangularView<Eigen::StrictlyLower>().transpose();*/
		// transpose elements below diagonal to elements above it
#else // _OPENMP
		Calculate_DenseMarginals_Fast(r_marginals, r_R, p_inv_order, n_order_size);
		// otherwise use the serial section
#endif // _OPENMP
	}

	/**
	 *	@brief fast parallel FBS function that calculates dense marginals matrix
	 *
	 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Calculate_DenseMarginals_Fast_Parallel_FBS(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, const size_t *p_inv_order, size_t n_order_size) // throw(std::bad_alloc)
	{
		_ASSERTE(p_inv_order);
		_ASSERTE(n_order_size == r_R.n_BlockColumn_Num());
		// make sure the ordering is there and has valid size

		const size_t n = r_R.n_Column_Num(); // in elements
		r_marginals.resize(n, n);

#ifdef _OPENMP
		#pragma omp parallel
		{
			int n_tid = omp_get_thread_num();
			int n_thread_num = omp_get_num_threads();
			size_t n_start = n_tid * (n / n_thread_num);
			size_t n_end = (n_tid + 1 < n_thread_num)? n_start + n / n_thread_num : n;
			// split to bands to be processed in parallel

			Calculate_DenseMarginals_Fast_ColumnBand_FBS<CBlockMatrixTypelist>(r_marginals,
				r_R, n_start, n_end, p_inv_order, n_order_size, false);
			// process in parallel
		}
		// calculate the lower-triangular marginals

		/*r_marginals.triangularView<Eigen::StrictlyUpper>() =
			r_marginals.triangularView<Eigen::StrictlyLower>().transpose();*/
		// transpose elements below diagonal to elements above it
#else // _OPENMP
		Calculate_DenseMarginals_Fast_FBS<CBlockMatrixTypelist>(r_marginals, r_R, p_inv_order, n_order_size);
		// otherwise use the serial section
#endif // _OPENMP
	}

	/**
	 *	@brief fast right column band function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_column_num is number of columns to calculate (in elements)
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_LastNColumns_Fast(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_column_num,
		const size_t *p_inv_order, size_t n_order_size) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements
		return Calculate_DenseMarginals_Fast_ColumnBand(r_marginals, r_R,
			n - n_column_num, n, p_inv_order, n_order_size);

		/*r_marginals.resize(n, n);
		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			if(n > n_column_num - 1 && i < n - n_column_num - 1)
				continue;
			// skip the prefix columns (todo - just call r_R.n_Find_BlockColumn())

			double *p_column = &r_marginals.col(i)(0);
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			r_R.UpperTriangularTranspose_Solve(p_column, n, n_block_col);
			r_R.UpperTriangular_Solve(p_column, n/ *, n_block_col* /);
			// solve for the whole column thing, generates one column at a time
		}*/
	}

	/**
	 *	@brief fast FBS right column band function that calculates dense marginals matrix
	 *
	 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_column_num is number of columns to calculate (in elements)
	 *	@param[in] p_inv_order is inverse ordering on the R factor
	 *	@param[in] n_order_size is size of the ordering (must match number of block columns in R)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Calculate_DenseMarginals_LastNColumns_Fast_FBS(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_column_num,
		const size_t *p_inv_order, size_t n_order_size) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements
		return Calculate_DenseMarginals_Fast_ColumnBand_FBS<CBlockMatrixTypelist>(r_marginals, r_R,
			n - n_column_num, n, p_inv_order, n_order_size);
	}

	/**
	 *	@brief slow column band function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_column_num is number of columns to calculate (in elements)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_LastNColumns_Slow(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_column_num) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);
		r_marginals.setZero(); // !!

		Eigen::MatrixXd R_inv_column(n, 1); // R_inv = S
		double *p_column = &R_inv_column.col(0)(0);
		// get dense column data from the Eigen matrix (actually only need one)

		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			if(n > n_column_num - 1 && i < n - n_column_num - 1)
				continue;
			// skip the prefix columns (todo - just call

			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			size_t UNUSED_VAR(n_block_column_size);
			_ASSERTE(n_block_col == r_R.n_Find_BlockColumn(i, n_block_column_size)); // should be the same
			// get which block column contains column i (optimize this away, probably need to use it when resuming)

			//_ASSERTE(n_block_column_size <= n_diag_band_width); // make this into a run-time check in the production code
			// make sure it is not longer than the diagonal (otherwise we will not have enough backlog to calculate all the off-diagonal elements)

			r_R.UpperTriangular_Solve(p_column, n, n_block_col); // backsub, only the nonzero part of the column (started at (block) column which contains column i, with no loss of generality)
			// this seems to be O(i) divisions + O(nnz) MADs in the given (block) column range
			// that sums up to O(n^2/2) divisions + O(nnz log(nnz))?? MADs ... some quadratute anyways

#ifdef _DEBUG
			std::vector<double> backsub_test(n, 0);
			backsub_test[i] = 1; // !!
			r_R.UpperTriangular_Solve(&backsub_test[0], n); // full backsub
			_ASSERTE(!memcmp(p_column, &backsub_test[0], n * sizeof(double)));
			// make sure that the result is correct
#endif // _DEBUG

			_ASSERTE((Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(p_column + i + 1, n - i - 1).norm() == 0)); // double pars required because of the comma in Map params
			// everything below i is zero (true)

			for(size_t k = n - n_column_num; k <= i; ++ k) {
				for(size_t j = 0; j <= i; ++ j)
					r_marginals(j, k) += p_column[j] * p_column[k]; // it is symmetric, indexing arbitrary
			}
			// accumulate the entries of the covariace matrix. this is O(n^3/2) MADs for the full matrix
			// note that to calculate even only the diagonal, we need full columns
		}
	}

	/**
	 *	@brief FBS kernel for marginal covariance calculation
	 */ // todo - carry all the improvements to the non-FBS version (especially the sparsification and the inverse diagonal calculation)
	class CSparseBlockMarginals_Recurrent_FBSKernel { // todo - fill throws
	public:
		/**
		 *	@brief a simple wrapper for block lookup in marginals calculation
		 */
		struct TBlockMatrixLookup {
			CUberBlockMatrix &r_marginals; /**< @brief reference to the output matrix */

#ifdef __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
			std::vector<double*> block_list; /**< @brief block data lookup @note Potentially big; for 10k and x64, it is 762 MB. Troubling, but still, full dense matrix in 10k would be 6.7 GB so it is one order of magnitude better. */
#endif // __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP

#ifdef _DEBUG
			const Eigen::MatrixXd *p_ground_truth; /**< @brief ground truth (if available) */
#endif // _DEBUG

			/**
			 *	@brief default constructor; binds to an empty block matrix
			 *	@param[in] _r_marginals is reference to the output matrix
			 */
			TBlockMatrixLookup(CUberBlockMatrix &_r_marginals)
				:r_marginals(_r_marginals)
#ifdef __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
				, block_list(_r_marginals.n_BlockColumn_Num() * _r_marginals.n_BlockColumn_Num(), 0) // is symmetric
#endif // __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
#ifdef _DEBUG
				, p_ground_truth(0)
#endif // _DEBUG
			{
				_ASSERTE(r_marginals.n_BlockColumn_Num() == r_marginals.n_BlockRow_Num());
				_ASSERTE(r_marginals.b_SymmetricLayout()); // must be symmetric
				_ASSERTE(!r_marginals.n_Block_Num()); // must be empty
			}

			/**
			 *	@brief gets an existing block from the marginals matrix
			 *
			 *	@param[in] n_block_row is zero-based index of block row
			 *	@param[in] n_block_column is zero-based index of block column
			 *	@param[in] n_block_row_num is number of rows in the block
			 *	@param[in] n_block_col_num is number of columns in the block
			 *
			 *	@return Returns pointer to block data.
			 */
			const double *p_GetExistingBlock(size_t n_block_row,
				size_t n_block_column, int n_block_row_num, int n_block_col_num) const
			{
				_ASSERTE(n_block_row_num == r_marginals.n_BlockColumn_Column_Num(n_block_row));
				_ASSERTE(n_block_col_num == r_marginals.n_BlockColumn_Column_Num(n_block_column));
#ifndef __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
				return r_marginals.p_GetBlock_Log(n_block_row, n_block_column,
					n_block_row_num/*r_marginals.n_BlockColumn_Column_Num(n_block_row)*/,
					n_block_col_num/*r_marginals.n_BlockColumn_Column_Num(n_block_column)*/);
#else // __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
				_ASSERTE(n_block_row <= n_block_column); // upper triangular
				_ASSERTE(n_block_column <= r_marginals.n_BlockColumn_Num());
				_ASSERTE(n_block_row <= r_marginals.n_BlockColumn_Num());
				size_t n_index = n_block_column * r_marginals.n_BlockRow_Num() + n_block_row;
				_ASSERTE(block_list[n_index]); // should already be there
				return block_list[n_index];
#endif // __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
			}

			/**
			 *	@brief allocates a new block in the marginals matrix
			 *
			 *	@param[in] n_block_row is zero-based index of block row
			 *	@param[in] n_block_column is zero-based index of block column
			 *	@param[in] n_block_row_num is number of rows in the block
			 *	@param[in] n_block_col_num is number of columns in the block
			 *
			 *	@return Returns pointer to block data.
			 *	@note This function throws std::bad_alloc.
			 */
			double *p_GetNewBlock(size_t n_block_row,
				size_t n_block_column, int n_block_row_num, int n_block_col_num) // throw(std::bad_alloc)
			{
				_ASSERTE(n_block_row_num == r_marginals.n_BlockColumn_Column_Num(n_block_row));
				_ASSERTE(n_block_col_num == r_marginals.n_BlockColumn_Column_Num(n_block_column));
#ifndef __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
				return r_marginals.p_GetBlock_Log(n_block_row, n_block_column,
					n_block_row_num/*r_marginals.n_BlockColumn_Column_Num(n_block_row)*/,
					n_block_col_num/*r_marginals.n_BlockColumn_Column_Num(n_block_column)*/, true, false); // alloc, don't mind uninit block
#else // __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
				_ASSERTE(n_block_row <= n_block_column); // upper triangular
				_ASSERTE(n_block_column <= r_marginals.n_BlockColumn_Num());
				_ASSERTE(n_block_row <= r_marginals.n_BlockColumn_Num());
				size_t n_index = n_block_column * r_marginals.n_BlockRow_Num() + n_block_row;

				_ASSERTE(!block_list[n_index]); // should really be new, not existing (can safely ignore if this assertion fires, though)
				double *p_result = r_marginals.p_GetBlock_Log(n_block_row, n_block_column,
					n_block_row_num/*r_marginals.n_BlockColumn_Column_Num(n_block_row)*/,
					n_block_col_num/*r_marginals.n_BlockColumn_Column_Num(n_block_column)*/, true, false);
				block_list[n_index] = p_result; // save it
				// don't mind uninit block

				return p_result;
#endif // __MARGINALS_RECURRENT_KERNEL_USE_DENSE_BLOCK_LOOKUP
			}
		};

		/**
		 *	@brief call context of the outer loop
		 */
		struct TOuterContext {
			size_t n_column_i; /**< @brief zero-based index of the current column */
			const CUberBlockMatrix &L; /**< @brief reference to the L factor matrix */
			const CUberBlockMatrix &R; /**< @brief reference to the R factor matrix (only its block structure is relevant) */
			TBlockMatrixLookup &r_marginals; /**< @brief reference to the (output) marginals matrix */
			const Eigen::VectorXd &r_inv_diag_L; /**< @brief reference to the precalculated elementwise inverse of the L factor diagonal */
			const int n_column_part; /**< @brief one of 0 = diagonal only, 1 = copy structure of R, 2 = full column */

			/**
			 *	@brief default constructor; fills the context
			 *
			 *	@param[in] _n_column_i is zero-based index of the current column
			 *	@param[in] _L is reference to the L factor matrix
			 *	@param[in] _R is reference to the R factor matrix (only its block structure is relevant)
			 *	@param[in] _r_marginals is reference to the (output) marginals matrix
			 *	@param[in] _r_inv_diag_L is reference to the precalculated elementwise
			 *		inverse of the L factor diagonal
			 *	@param[in] _n_column_part is one of 0 = diagonal only, 1 = copy structure of R, 2 = full column
			 */
			inline TOuterContext(size_t _n_column_i, const CUberBlockMatrix &_L,
				const CUberBlockMatrix &_R, TBlockMatrixLookup &_r_marginals,
				const Eigen::VectorXd &_r_inv_diag_L, int _n_column_part)
				:n_column_i(_n_column_i), L(_L), R(_R), r_marginals(_r_marginals),
				r_inv_diag_L(_r_inv_diag_L), n_column_part(_n_column_part)
			{}
		};

		/**
		 *	@brief call context of the off-diagonal middle loop
		 */
		struct TMiddleOffDiagContext : public TOuterContext { // t_odo - remove the second "middle"
			size_t n_column_j; /**< @brief zero-based index of the current row (or column in R) */

			/**
			 *	@brief default constructor; fills the context
			 *
			 *	@param[in] t_ctx is the outer loop context (required in all the loops)
			 *	@param[in] _n_column_j is zero-based index of the current row (or column in R)
			 */
			inline TMiddleOffDiagContext(TOuterContext t_ctx, size_t _n_column_j)
				:TOuterContext(t_ctx), n_column_j(_n_column_j)
			{}
		};

		/**
		 *	@brief call context of the blockwise diagonal inner loop
		 */
		struct TInnerDiagContext_Blocky : public TOuterContext {
			size_t j; /**< @brief zero-based index of the current block in the current column */
			size_t n_row_j; /**< @brief zero-based row index of the current block */
			double *p_dest_block; /**< @brief pointer to the block accumulator (output of the loop) */

			/**
			 *	@brief default constructor; fills the context
			 *
			 *	@param[in] t_ctx is the outer loop context (required in all the loops)
			 *	@param[in] _j is zero-based index of the current block in the current column
			 *	@param[in] _n_row_j is zero-based row index of the current block
			 *	@param[in] _p_dest_block is pointer to the block accumulator (output of the loop)
			 */
			inline TInnerDiagContext_Blocky(TOuterContext t_ctx,
				size_t _j, size_t _n_row_j, double *_p_dest_block)
				:TOuterContext(t_ctx), j(_j), n_row_j(_n_row_j),
				p_dest_block(_p_dest_block)
			{}
		};

		/**
		 *	@brief call context of the elementwise diagonal inner loop
		 */
		struct TInnerDiagContext_Elem : public TOuterContext {
			size_t j; /**< @brief zero-based index of the current block in the current column */
			size_t n_row_j; /**< @brief zero-based row index of the current block */
			size_t n_elem; /**< @brief zero-based index of the destination column in the current block */
			size_t n_elem2; /**< @brief zero-based index of the destination row in the current block */
			double &r_f_diag_sum; /**< @brief reference to the accumulator (output of the loop) */

			/**
			 *	@brief default constructor; fills the context
			 *
			 *	@param[in] t_ctx is the outer loop context (required in all the loops)
			 *	@param[in] _j is zero-based index of the current block in the current column
			 *	@param[in] _n_row_j is zero-based row index of the current block
			 *	@param[in] _n_elem is zero-based index of the destination column in the current block
			 *	@param[in] _n_elem2 is zero-based index of the destination row in the current block
			 *	@param[in] _r_f_diag_sum is reference to the accumulator (output of the loop)
			 */
			inline TInnerDiagContext_Elem(TOuterContext t_ctx, size_t _j, size_t _n_row_j,
				size_t _n_elem, size_t _n_elem2, double &_r_f_diag_sum)
				:TOuterContext(t_ctx), j(_j), n_row_j(_n_row_j), n_elem(_n_elem),
				n_elem2(_n_elem2), r_f_diag_sum(_r_f_diag_sum)
			{}
		};

		/**
		 *	@brief call context of the off-diagonal inner loop
		 */
		struct TInnerOffDiagContext : public TMiddleOffDiagContext {
			size_t k; /**< @brief zero-based index of the current block in the current column */
			size_t n_row_k; /**< @brief zero-based row index of the current block */
			double *p_dest_block; /**< @brief pointer to the block accumulator (output of the loop) */

			/**
			 *	@brief default constructor; fills the context
			 *
			 *	@param[in] t_ctx is the middle loop context (required inside of this loops)
			 *	@param[in] _k is zero-based index of the current block in the current column
			 *	@param[in] _n_row_k is zero-based row index of the current block
			 *	@param[in] _p_dest_block is pointer to the block accumulator (output of the loop)
			 */
			inline TInnerOffDiagContext(TMiddleOffDiagContext t_ctx, /*size_t _n_column_j,*/
				size_t _k, size_t _n_row_k, double *_p_dest_block)
				:TMiddleOffDiagContext(t_ctx), n_row_k(_n_row_k),
				/*n_column_j(_n_column_j),*/ k(_k), p_dest_block(_p_dest_block)
			{}
		};

		/**
		 *	@brief diagonal inner loop body; calculates a dot product contribution
		 *		of a single depending block (to diagonal block of the marginals matrix)
		 *	@tparam n_row_j_size is size of the row (or column) j
		 */
		template <const int n_row_j_size, class CColumnISize>
		struct TDiagInnerLoop_Blocky {
			/**
			 *	@brief diagonal inner loop body implementation
			 *	@param[in] t_ctx is loop context
			 */
			static inline void Do(TInnerDiagContext_Blocky t_ctx)
			{
				enum {
					n_column_i_size = CColumnISize::n_size
				};
				const CUberBlockMatrix &L = t_ctx.L;
				TBlockMatrixLookup &r_marginals = t_ctx.r_marginals;
				const size_t n_column_i = t_ctx.n_column_i;
				const size_t j = t_ctx.j;
				const size_t n_row_j = t_ctx.n_row_j;
				typename CUberBlockMatrix::CMakeMatrixRef<n_column_i_size,
					n_column_i_size>::_Ty block_i_i_mar(t_ctx.p_dest_block);
				// unwrap the contexts

				typename CUberBlockMatrix::CMakeMatrixRef<n_row_j_size, n_column_i_size>::_TyConst
					block_j_i = L.t_Block_AtColumn<n_row_j_size, n_column_i_size>(n_column_i, j); // todo - rename to block_k_j, check for similar mistakes

				_ASSERTE(n_row_j > n_column_i); // make sure it is in lower tri (always should)

				typename CUberBlockMatrix::CMakeMatrixRef<n_column_i_size, n_row_j_size>::_TyConst
					block_i_j_mar(r_marginals.p_GetExistingBlock(n_column_i, n_row_j, n_column_i_size, n_row_j_size));
				// get the corresponding block in the marginals (already calculated at this moment)

				//block_i_i_mar.noalias() += (block_i_j_mar * block_j_i).transpose(); // 10.36 on 10k
				//block_i_i_mar.noalias() += block_j_i.transpose() * block_i_j_mar.transpose(); // 10.38 on 10k
				//block_i_i_mar += block_i_j_mar.lazyProduct(block_j_i).transpose(); // 10.43 on 10k
				block_i_i_mar += block_j_i.transpose().lazyProduct(block_i_j_mar.transpose()); // 10.35 on 10k
				// t_odo - try with lazyProduct(), try reverse order product and transpose the factors themselves
				// add dot of one column of the block with span of the current column of the marginals
			}
		};

		/**
		 *	@brief diagonal inner loop body; calculates a single dot product contribution
		 *		of a single column of a single off-diagonal block to a single diagonal block element
		 *
		 *	@tparam n_row_j_size is number of rows of the contributing block
		 *	@tparam CColumnISize is number of rows (columns) of the diagonal block
		 */
		template <const int n_row_j_size, class CColumnISize>
		struct TDiagInnerLoop_Elem {
			/**
			 *	@brief diagonal inner loop body implementation
			 *	@param[in] t_ctx is loop context
			 */
			static inline void Do(TInnerDiagContext_Elem t_ctx)
			{
				enum {
					n_column_i_size = CColumnISize::n_size // make this variable-like
				};
				const size_t n_column_i = t_ctx.n_column_i;
				const CUberBlockMatrix &L = t_ctx.L;
				TBlockMatrixLookup &r_marginals = t_ctx.r_marginals;
				const size_t j = t_ctx.j;
				const size_t n_row_j = t_ctx.n_row_j;
				const size_t n_elem = t_ctx.n_elem;
				const size_t n_elem2 = t_ctx.n_elem2;
				// unwrap the contexts

				typename CUberBlockMatrix::CMakeMatrixRef<n_row_j_size, n_column_i_size>::_TyConst
					block_i_j = L.t_Block_AtColumn<n_row_j_size, n_column_i_size>(n_column_i, j);
				_ASSERTE(n_row_j_size == block_i_j.rows());
				// look up the row in L

				typename CUberBlockMatrix::CMakeMatrixRef<n_column_i_size, n_row_j_size>::_TyConst
					block_i_j_mar(r_marginals.p_GetExistingBlock(n_column_i, n_row_j, n_column_i_size, n_row_j_size));
				// get the corresponding block in the marginals (already calculated at this moment)
				// note that (n_row_j, n_column_i) would be lower diagonal, here we access
				// the transpose block accross the diagonal

				_ASSERTE(n_row_j > n_column_i); // make sure the below dot product will sample uper diagonal of the marginals
				t_ctx.r_f_diag_sum += block_i_j_mar.row(n_elem).dot(block_i_j.col(n_elem2)); // t_odo - _FBS it
				// add dot of one column of the block with span of the current column of the marginals
			}
		};

		/**
		 *	@brief off-diagonal inner loop body; calculates a dot product contribution
		 *		of a single depending block (to off-diagonal block of the marginals matrix)
		 *	@tparam n_row_k_size is size of the row (or column) k
		 */
		template <const int n_row_k_size, class CContext2>
		struct TOffDiagInnerLoop {
			/**
			 *	@brief off-diagonal inner loop body implementation
			 *	@param[in] t_ctx is loop context
			 */
			static inline void Do(TInnerOffDiagContext t_ctx)
			{
				enum {
					n_column_j_size = CContext2::_TyHead::n_row_num,
					n_column_i_size = CContext2::_TyHead::n_column_num,
					b_upper_tri = CContext2::_TyTail::_TyHead::b_flag
				};
				const CUberBlockMatrix &L = t_ctx.L;
				TBlockMatrixLookup &r_marginals = t_ctx.r_marginals;
				const size_t n_column_i = t_ctx.n_column_i;
				const size_t n_column_j = t_ctx.n_column_j;
				const size_t k = t_ctx.k;
				const size_t n_row_k = t_ctx.n_row_k;//L.n_Block_Row(n_column_j, k); // todo - pass this through the context
				typename CUberBlockMatrix::CMakeMatrixRef<n_column_j_size,
					n_column_i_size>::_Ty block_j_i_mar(t_ctx.p_dest_block);
				// unwrap the contexts

				if(b_upper_tri) { // compile-time constant; should optimize away
					typename CUberBlockMatrix::CMakeMatrixRef<n_row_k_size, n_column_j_size>::_TyConst
						block_j_k = L.t_Block_AtColumn<n_row_k_size, n_column_j_size>(n_column_j, k);

					_ASSERTE(n_row_k <= n_column_i); // make sure it is in upper tri (don't mind the diagonal)

					typename CUberBlockMatrix::CMakeMatrixRef<n_row_k_size, n_column_i_size>::_TyConst
						block_i_k_mar(r_marginals.p_GetExistingBlock(n_row_k, n_column_i, n_row_k_size, n_column_i_size));
					// get the corresponding block in the marginals (already calculated at this moment)

					//block_j_i_mar.noalias() += block_j_k.transpose() * block_i_k_mar; // 10.35 on 10k
					block_j_i_mar += block_j_k.transpose().lazyProduct(block_i_k_mar); // 10.29 on 10k
					// t_odo - try with lazyProduct()
					// add dot of one column of the block with span of the current column of the marginals
				} else {
					typename CUberBlockMatrix::CMakeMatrixRef<n_row_k_size, n_column_j_size>::_TyConst
						block_j_k = L.t_Block_AtColumn<n_row_k_size, n_column_j_size>(n_column_j, k); // todo - rename to block_k_j, check for similar mistakes

					_ASSERTE(n_row_k >= n_column_i); // make sure it is in lower tri (don't mind the diagonal)

					typename CUberBlockMatrix::CMakeMatrixRef<n_column_i_size, n_row_k_size>::_TyConst
						block_i_k_mar(r_marginals.p_GetExistingBlock(n_column_i, n_row_k, n_column_i_size, n_row_k_size));
					// get the corresponding block in the marginals (already calculated at this moment)

					//block_j_i_mar.noalias() += (block_i_k_mar * block_j_k).transpose(); // 10.36 on 10k
					//block_j_i_mar.noalias() += block_j_k.transpose() * block_i_k_mar.transpose(); // 10.38 on 10k
					//block_j_i_mar += block_i_k_mar.lazyProduct(block_j_k).transpose(); // 10.43 on 10k
					block_j_i_mar += block_j_k.transpose().lazyProduct(block_i_k_mar.transpose()); // 10.35 on 10k
					// t_odo - try with lazyProduct(), try reverse order product and transpose the factors themselves
					// add dot of one column of the block with span of the current column of the marginals
				}
			}
		};

		/**
		 *	@brief off-diagonal middle loop body; calculates a single
		 *		off-diagonal block of the marginals matrix
		 *
		 *	@tparam n_column_j_size is size of the column (or row) j
		 *	@tparam CContext2 is a typelist, containing list of block sizes and size of column i
		 */
		template <const int n_column_j_size, class CContext2>
		struct TOffDiagMiddleLoop {
			typedef typename CContext2::_TyHead CMatrixBlockSizeList; /**< @brief block sizes */
			typedef typename CContext2::_TyTail::_TyHead CColumnISize; /**< @brief size of column i */

			/**
			 *	@brief off-diagonal middle loop implementation
			 *	@param[in] t_ctx is loop context
			 *	@note This function throws std::bad_alloc.
			 */
			static inline void Do(TMiddleOffDiagContext t_ctx) // throw(std::bad_alloc)
			{
				enum {
					n_column_i_size = CColumnISize::n_size // make this variable-like
				};
				const size_t n_column_i = t_ctx.n_column_i;
				const CUberBlockMatrix &L = t_ctx.L;
				TBlockMatrixLookup &r_marginals = t_ctx.r_marginals;
				const size_t n_column_j = t_ctx.n_column_j;
				const Eigen::VectorXd &r_inv_diag_L = t_ctx.r_inv_diag_L;
				// unwrap the contexts

				_ASSERTE(n_column_j_size == L.n_BlockColumn_Column_Num(n_column_j));
				size_t n_column_j_base = L.n_BlockColumn_Base(n_column_j);
				size_t n_column_j_block_num = L.n_BlockColumn_Block_Num(n_column_j); // t_odo - rename *block_j* to *column_j*
				// gets the corresponding block column

				typename CUberBlockMatrix::CMakeMatrixRef<n_column_j_size, n_column_j_size>::_TyConst
					cur_L_diag_block = L.t_Block_AtColumn<n_column_j_size, n_column_j_size>(n_column_j, 0); // t_odo - rename those to have "L" in them
				_ASSERTE(L.n_Block_Row(n_column_j, 0) == n_column_j); // make sure it is at the diagonal (L has symmetric layout)

				_ASSERTE(n_column_j < n_column_i); // strictly upper even
				typename CUberBlockMatrix::CMakeMatrixRef<n_column_j_size, n_column_i_size>::_Ty
					block_j_i_mar(r_marginals.p_GetNewBlock(n_column_j, n_column_i, n_column_j_size, n_column_i_size)); // t_odo rename this
				// work with blocks also in the matrix of the marginals

				const typename Eigen::VectorBlock<const Eigen::VectorXd, n_column_j_size>
					cur_L_inv_diag_span = r_inv_diag_L.segment<n_column_j_size>(n_column_j_base);

				//Eigen::Matrix<double, n_column_j_size, n_column_i_size> block_j_i_mar; // use local matrix
				block_j_i_mar.setZero();
				for(size_t k = 1; k < n_column_j_block_num; ++ k) { // all the blocks except the diagonal
					size_t n_row_k = L.n_Block_Row(n_column_j, k);
					size_t n_row_k_size = L.n_BlockRow_Row_Num(n_row_k);
					//size_t n_row_k_base = L.n_BlockRow_Base(n_row_k);

					if(n_row_k/*_base*/ < n_column_i/*_base*/) {
						typedef typename MakeTypelist_Safe((fbs_ut::CCTSize2D<n_column_j_size,
							n_column_i_size>, fbs_ut::CCTFlag<true>)) _TySecondaryContext;
						// secondary context for the loop

						fbs_ut::CWrap2<TOffDiagInnerLoop, _TySecondaryContext>::template
							In_RowHeight_DecisionTree_Given_ColumnWidth<CMatrixBlockSizeList,
							n_column_j_size>(int(n_row_k_size), TInnerOffDiagContext(t_ctx,
							/*n_column_j,*/ k, n_row_k, block_j_i_mar.data()));
					} else {
						typedef typename MakeTypelist_Safe((fbs_ut::CCTSize2D<n_column_j_size,
							n_column_i_size>, fbs_ut::CCTFlag<false>)) _TySecondaryContext;
						// secondary context for the loop

						for(;;) {
							fbs_ut::CWrap2<TOffDiagInnerLoop, _TySecondaryContext>::template
								In_RowHeight_DecisionTree_Given_ColumnWidth<CMatrixBlockSizeList,
								n_column_j_size>(int(n_row_k_size), TInnerOffDiagContext(t_ctx,
								/*n_column_j,*/ k, n_row_k, block_j_i_mar.data()));

							if(++ k == n_column_j_block_num)
								break;

							n_row_k = L.n_Block_Row(n_column_j, k);
							n_row_k_size = L.n_BlockRow_Row_Num(n_row_k);
							//n_row_k_base = L.n_BlockRow_Base(n_row_k);

							_ASSERTE(n_row_k/*_base*/ >= n_column_i/*_base*/); // once lower, always lower
						}
						// t_odo - this branch will be used for all subsequent blocks; make another
						// loop here without the test, then break out from the outer loop
						// does not save much, but doesn't complicate the code either

						break;
					}
					// call the inner loop
				}
				// calculate the first part of the sum in completely blockwise manner

				for(size_t n_elem = n_column_i_size; n_elem > 0;) { // go backwards
					-- n_elem; // here
#if 0 // different code for the last row of the result block
					{
						size_t n_elem2 = n_column_j_size - 1;
						// the last diagonal

						double f_L_jj_inv = cur_L_inv_diag_span(n_elem2);//1 / cur_L_diag_block(n_elem2, n_elem2);
						// get the diagonal block and element

						double f_diag_sum = block_j_i_mar(n_elem2, n_elem); // (calculated blockwise)
						// continue from here

						// no additional dot product, there is nothing below (would be size 0 dot product, complicated _FBS)

						block_j_i_mar(n_elem2, n_elem) = -f_diag_sum * f_L_jj_inv;
						// write only upper triangular, will mirror it at once (or not at all)
					}

					// the loop skips one elem
					for(size_t n_elem2 = n_column_j_size - 1; n_elem2 > 0;) { // go backwards
#else // 1
					for(size_t n_elem2 = n_column_j_size; n_elem2 > 0;) { // go backwards
#endif // 1
						-- n_elem2; // here

						double f_L_jj_inv = cur_L_inv_diag_span(n_elem2);//1 / cur_L_diag_block(n_elem2, n_elem2);
						//double f_L_jj_inv_gt = 1 / cur_L_diag_block(n_elem2, n_elem2);
						// get the diagonal block and element

						double f_diag_sum = block_j_i_mar(n_elem2, n_elem); // (calculated blockwise)
						// continue from here

						{
							size_t n_first_underdiag_elem = n_elem2 + 1;
							size_t n_underdiag_elem_num = n_column_j_size - n_first_underdiag_elem;
							f_diag_sum += block_j_i_mar.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(n_underdiag_elem_num)); // todo - _FBS it
							// complete the sum
						}
						// this is complicated in block approach (maybe not, maybe it is just the task for a triangular view)

						block_j_i_mar(n_elem2, n_elem) = -f_diag_sum * f_L_jj_inv;
						// write only upper triangular, will mirror it at once (or not at all)
					}
				}
				// finish the calculation in elementwise manner (can't be vectorized, the order is important)

#ifdef _DEBUG
				if(_isnan(block_j_i_mar.norm())) {
					fprintf(stderr, "error: marginal block (" PRIsize
						", " PRIsize ") contains NaNs\n", n_column_j, n_column_i);
				}
				if(r_marginals.p_ground_truth) {
					const Eigen::MatrixXd &r_gt = *r_marginals.p_ground_truth;
					size_t n_column_i_base = L.n_BlockColumn_Base(n_column_i);
					Eigen::Block<const Eigen::MatrixXd, n_column_j_size, n_column_i_size> block_j_i_gt =
						r_gt.block<n_column_j_size, n_column_i_size>(n_column_j_base, n_column_i_base);
					double f_error;
					Eigen::MatrixXd diff = (block_j_i_gt - block_j_i_mar); // required by g++, norm of Eigen::Block seems problematic
					if((f_error = /*(block_j_i_gt - block_j_i_mar)*/diff.norm()) > 1e-5) {
						fprintf(stderr, "error: marginal block (" PRIsize
							", " PRIsize ") is off by %g\n", n_column_j, n_column_i, f_error);
						CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, block_j_i_gt, "block_j_i_gt = ");
						CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, block_j_i_mar, "block_j_i_mar = ");
					}
				}
#endif // _DEBUG
				// check the block
			}
		};

		/**
		 *	@brief outer loop body; calculates one block column of the marginals
		 *
		 *	@tparam n_column_i_size is size of the current column, in elements
		 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
		 */
		template <const int n_column_i_size, class CMatrixBlockSizeList>
		struct TOuterLoop {
			/**
			 *	@brief outer loop implementation
			 *	@param[in] t_ctx is loop context
			 *	@note This function throws std::bad_alloc.
			 */
			static inline void Do(TOuterContext t_ctx) // throw(std::bad_alloc)
			{
				const size_t n_column_i = t_ctx.n_column_i;
				const CUberBlockMatrix &R = t_ctx.R;
				const CUberBlockMatrix &L = t_ctx.L;
				TBlockMatrixLookup/*Eigen::MatrixXd*/ &r_marginals = t_ctx.r_marginals;
				const Eigen::VectorXd &r_inv_diag_L = t_ctx.r_inv_diag_L;
				const int n_column_part = t_ctx.n_column_part;
				// unwrap the context

				_ASSERTE(size_t(n_column_i_size) == L.n_BlockColumn_Column_Num(n_column_i));
				size_t n_column_i_base = L.n_BlockColumn_Base(n_column_i);
				size_t n_column_i_block_num = L.n_BlockColumn_Block_Num(n_column_i); // t_odo - rename *block_i* to *column_i*
				// gets the corresponding block col

				const typename Eigen::VectorBlock<const Eigen::VectorXd, n_column_i_size>
					cur_L_inv_diag_span = r_inv_diag_L.segment<n_column_i_size>(n_column_i_base);
				// get diagonal span

				if(n_column_part != 2) { // if 2, then we are doing the full column, which also means that we are running the 2nd pass, and the diagonal block is already computed, no need to redo // not so sure about it at the moment // todo - put it back
					typename CUberBlockMatrix::CMakeMatrixRef<n_column_i_size, n_column_i_size>::_Ty
						block_i_i_mar(r_marginals.p_GetNewBlock(n_column_i, n_column_i, n_column_i_size, n_column_i_size));
					//Eigen::Matrix<double, n_column_i_size, n_column_i_size> block_i_i_mar; // uninit is ok
					//TDiagBlock block_i_i_mar =
					//	r_marginals.block<n_column_i_size, n_column_i_size>(n_column_i_base, n_column_i_base);
					// work with blocks also in the matrix of the marginals

					typename CUberBlockMatrix::CMakeMatrixRef<n_column_i_size, n_column_i_size>::_TyConst
						cur_L_diag_block = L.t_Block_AtColumn<n_column_i_size, n_column_i_size>(n_column_i, 0);
					_ASSERTE(L.n_Block_Row(n_column_i, 0) == n_column_i); // make sure it is at the diagonal (L has symmetric layout)
					// get diag block

					//cur_L_inv_diag_span = /*1 / */cur_L_diag_block.diagonal().array().inverse();
					// invert the diagonal (with SSE?)
					// this is wrong, the rest of the blocks will need higher diagonals, will ref uninit data
					// need to do this beforehand (the first column can in worst case reference all the diagonals)

#ifdef __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP
					block_i_i_mar.setZero();
					for(size_t j = 1; j < n_column_i_block_num; ++ j) { // all the blocks except the diagonal
						size_t n_row_j = L.n_Block_Row(n_column_i, j);
						size_t n_row_j_size = L.n_BlockColumn_Column_Num(n_row_j);

						/*_ASSERTE(n_row_j >= n_column_i); // all in lower tri
						fbs_ut::CWrap<TOffDiagInnerLoop>::template
							In_RowHeight_DecisionTree_Given_ColumnWidth<CMatrixBlockSizeList,
							n_column_i_size>(int(n_row_j_size), TInnerOffDiagContext<n_column_i_size,
							n_column_i_size, false>(t_ctx, n_column_i, j, n_row_j, block_i_i_mar));
						// t_odo - this is slightly more complicated than it needs to be, n_column_i is there twice now*/

						fbs_ut::CWrap2<TDiagInnerLoop_Blocky, fbs_ut::CCTSize<n_column_i_size> >::template
							In_RowHeight_DecisionTree_Given_ColumnWidth<CMatrixBlockSizeList,
							n_column_i_size>(int(n_row_j_size), TInnerDiagContext_Blocky(t_ctx, j,
							n_row_j, block_i_i_mar.data())); // more specialized version
						// call the inner loop
					}
					// calculate the first part of the sum in completely blockwise manner
					// note that only triangular part is required so some extra calculations are performed
					// but at the same time the blocks are only accessed once, possibly saving memory bandwidth
#endif // __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP

					for(size_t n_elem = n_column_i_size; n_elem > 0;) { // go backwards
						-- n_elem; // here
						for(size_t n_elem2 = n_elem + 1; n_elem2 > 0;) { // for all the elements above or on diagonal in this marginal block
							-- n_elem2; // here
							double f_L_jj_inv = cur_L_inv_diag_span(n_elem2);//1 / cur_L_diag_block(n_elem2, n_elem2);
							//double f_L_jj_inv_gt = 1 / cur_L_diag_block(n_elem2, n_elem2);

#ifdef __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP
							double f_diag_sum = block_i_i_mar(n_elem2, n_elem); // calculated blockwise
#else // __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP
							double f_diag_sum = 0;
							for(size_t j = 1; j < n_column_i_block_num; ++ j) { // all the blocks except the diagonal
								size_t n_row_j = L.n_Block_Row(n_column_i, j);
								size_t n_row_j_size = L.n_BlockColumn_Column_Num(n_row_j);

								fbs_ut::CWrap2<TDiagInnerLoop_Elem, fbs_ut::CCTSize<n_column_i_size> >::template
									In_RowHeight_DecisionTree_Given_ColumnWidth<CMatrixBlockSizeList,
									n_column_i_size>(int(n_row_j_size), TInnerDiagContext_Elem(t_ctx,
									j, n_row_j, n_elem, n_elem2, f_diag_sum));
								// call the inner loop
							}
							//double f_from_mat = block_i_i_mar(n_elem, n_elem2), f_fm2 = block_i_i_mar(n_elem2, n_elem);
							//_ASSERTE(fabs(f_diag_sum - block_i_i_mar(n_elem2, n_elem)) < 1e-5f); // todo - this can be done using blockwise code (instead of row-wise), same as off-diagonal elements
#endif // __MARGINALS_RECURRENT_KERNEL_USE_BLOCKY_DIAGONAL_LOOP
							{
								size_t n_first_underdiag_elem = n_elem2 + 1;
								size_t n_underdiag_elem_num = n_column_i_size - n_first_underdiag_elem;

								//if(n_column_i_size > n_elem2 + 1) // already in the .dot() loop, wouldn't bring any benefit
								f_diag_sum += block_i_i_mar.col(n_elem).segment(n_first_underdiag_elem,
									n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(n_underdiag_elem_num)); // todo - _FBS it
								// complete the sum
							}

							if(n_elem2 == n_elem) {
								block_i_i_mar(n_elem, n_elem) = f_L_jj_inv * (f_L_jj_inv - f_diag_sum);
								// diagonal elems have a different formula
							} else {
								block_i_i_mar(n_elem2, n_elem) = block_i_i_mar(n_elem, n_elem2) =
									f_L_jj_inv * -f_diag_sum; // use _jj! (different diagonal elem in the same block)
							}
						}
					}

#ifdef _DEBUG
					if(_isnan(block_i_i_mar.norm())) {
						fprintf(stderr, "error: marginal block (" PRIsize
							", " PRIsize ") contains NaNs\n", n_column_i, n_column_i);
					}
					if(r_marginals.p_ground_truth) {
						const Eigen::MatrixXd &r_gt = *r_marginals.p_ground_truth;
						Eigen::Matrix<double, n_column_i_size, n_column_i_size> block_i_i_gt =
							r_gt.block<n_column_i_size, n_column_i_size>(n_column_i_base, n_column_i_base);
						double f_error;
						if((f_error = (block_i_i_gt - block_i_i_mar).norm()) > 1e-5) {
							fprintf(stderr, "error: marginal block (" PRIsize
								", " PRIsize ") is off by %g\n", n_column_i, n_column_i, f_error);
							CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, block_i_i_gt, "block_i_i_gt = ");
							CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, block_i_i_mar, "block_i_i_mar = ");
						}
					}
#endif // _DEBUG
				}
				// do the block at the diagonal

				if(n_column_part == 1) { // copy sparsity of R
					size_t n_R_column_i_block_num = R.n_BlockColumn_Block_Num(n_column_i); // or use L and calculate the marginals in lower triangular part (probably a lot of changes)
					for(size_t j = n_R_column_i_block_num - 1; j > 0;) {
						-- j; // except the last block (the diagonal one)
						size_t n_column_j = R.n_Block_Row(n_column_i, j); // this needs R as well
						// try it like that (calculate marginals of only the spine blocks, that is much faster)

					//for(size_t j = n_column_i_block_num - 1; j > 0; -- j) { // except the block 0 (the diagonal one)
					//	size_t n_column_j = L.n_BlockColumn_Num() - 1 - L.n_Block_Row(n_column_i, j); // upside-down
					// this is incorrect, really need R

					//for(size_t n_column_j = n_column_i; n_column_j > 0;) { // note that this is only required for the blocks that are required on output or blocks that are coincident with nnz blocks in the L or R factor
					//	-- n_column_j; // here
					// this calculates full matrix

						size_t n_column_j_size = L.n_BlockColumn_Column_Num(n_column_j);

						typedef typename MakeTypelist(CMatrixBlockSizeList,
							fbs_ut::CCTSize<n_column_i_size>) _TySecondaryContext;
						fbs_ut::CWrap2<TOffDiagMiddleLoop, _TySecondaryContext>::template
							In_ColumnWidth_DecisionTree<CMatrixBlockSizeList>(int(n_column_j_size),
							TMiddleOffDiagContext(t_ctx, n_column_j));
						// call the middle loop
					}
				} else if(n_column_part == 2) { // full column (the part above the diagonal, anyway)
					_ASSERTE(n_column_i); // zero has no part above, i want the caller to handle this
					for(size_t n_column_j = n_column_i; n_column_j > 0;) {
						-- n_column_j; // except the last block (the diagonal one)
						size_t n_column_j_size = L.n_BlockColumn_Column_Num(n_column_j);
						typedef typename MakeTypelist(CMatrixBlockSizeList,
							fbs_ut::CCTSize<n_column_i_size>) _TySecondaryContext;
						fbs_ut::CWrap2<TOffDiagMiddleLoop, _TySecondaryContext>::template
							In_ColumnWidth_DecisionTree<CMatrixBlockSizeList>(int(n_column_j_size),
							TMiddleOffDiagContext(t_ctx, n_column_j));
						// call the middle loop
					}
					// dense (but under ordering, might run into trouble if ordered improperly)
				}

				// note - to calculate full matrix, everything is in place now, calculation of the remaining blocks
				// are independent of each other, and can be done in parallel!
			}
		};

	public:
		/**
		 *	@brief loop function (just calls the decision tree)
		 *
		 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
		 *
		 *	@param[in] n_column_i_size is size of the current column
		 *	@param[in] n_column_i is zero-based index of the current column
		 *	@param[in] L is reference to the L factor matrix
		 *	@param[in] R is reference to the R factor matrix (only its block structure is relevant)
		 *	@param[in] r_marginals is reference to the (output) marginals matrix
		 *	@param[in] r_inv_diag_L is reference to the precalculated elementwise inverse of the L factor diagonal
		 *	@param[in] n_column_part is one of 0 = diagonal only, 1 = copy structure of R, 2 = full column
		 *
		 *	@note This function throws std::bad_alloc.
		 */
		template <class CBlockMatrixTypelist>
		static inline void Run(size_t n_column_i_size, size_t n_column_i,
			const CUberBlockMatrix &L, const CUberBlockMatrix &R,
			TBlockMatrixLookup &r_marginals, const Eigen::VectorXd &r_inv_diag_L, int n_column_part) // throw(std::bad_alloc)
		{
			fbs_ut::CWrap2<TOuterLoop, CBlockMatrixTypelist>::template
				In_ColumnWidth_DecisionTree<CBlockMatrixTypelist>(
				int(n_column_i_size), TOuterContext(n_column_i, L, R, r_marginals, r_inv_diag_L, n_column_part));
			// wrap the outer loop
		}
	};

	/**
	 *	@brief reference function that calculates blockwise sparse marginals matrix
	 *
	 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R
	 *	@param[in] n_matrix_part is matrix part to be calculated (note that mpart_FullMatrix
	 *		really means the upper triangle, rather than a full dense matrix; default
	 *		mpart_Nothing)
	 *	@param[in] b_structure_of_R is additional matrix part to be calculated (if set, the
	 *		structure of R is calculated; if not set, only the part specified by n_matrix_part
	 *		is calculated)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Calculate_DenseMarginals_Recurrent_FBS(CUberBlockMatrix &r_marginals,
		const CUberBlockMatrix &r_R, const CMatrixOrdering &mord,
		EBlockMatrixPart n_matrix_part = mpart_Nothing, bool b_structure_of_R = true) // throw(std::bad_alloc)
	{
		_ASSERTE(!(n_matrix_part & mpart_Column)); // which column?

		//if(n_matrix_part == mpart_FullMatrix)
		//	throw std::runtime_error("mpart_FullMatrix in Calculate_DenseMarginals_Recurrent_FBS() not implemented");
		// t_odo - implement this not very useful feature (it just takes a *lot* of memory)

		if((n_matrix_part & mpart_Diagonal) == mpart_Diagonal)
			b_structure_of_R = true;
		// trade the diagonal for the structure of R, which includes diagonal

		if(b_structure_of_R)
			n_matrix_part = n_MPart_Subtract(n_matrix_part, EBlockMatrixPart(mpart_Diagonal | mpart_LastBlock));
		// the last block is implied in the structure of R, as is the diagonal

		if(!b_structure_of_R && n_matrix_part == mpart_Nothing) {
			r_marginals.SetZero();
			r_marginals.ExtendTo(r_R.n_Row_Num(), r_R.n_Column_Num()); // why bother?
			return;
		}
		// nothing to calculate

		/*{
			const size_t n = r_R.n_Column_Num(); // in elements
			r_marginals.resize(n, n);
		}*/

		const size_t n = r_R.n_BlockColumn_Num(); // in blocks!
		CUberBlockMatrix L;
		L.TransposeOf(r_R);
		// need transpose of R
		// todo - get rid of this, just make transpose ptr list, get all the layout info from R
		// note that the transpose takes less than 1% of time on linux for 300 x 300 sparse R (part of 10k)

		size_t n_first_block_col = 0; // all columns
		const size_t n_last_col = n - 1;
		if(!b_structure_of_R) {
			_ASSERTE(n_matrix_part == mpart_LastBlock);
			n_first_block_col = mord.p_Get_InverseOrdering()[n_last_col]; // only the last one (but it is not necessarily the last one in R)
		}
		// decide where to finish

		//printf("this is the correct blockwise Calculate_DenseMarginals_Recurrent_FBS()\n"); // debug

		_ASSERTE(r_R.b_SymmetricLayout());
		// must be symmetric (any result of cholesky would be)

		// note that the notation in the below code is slightly confusing, as any column in R
		// is also a row in L and vice versa. there are also some transpose accesses where
		// row and column are switched. it is a mess, really.

		// note that R is not used in the below code. although layout of R might reside
		// in CPU cache, the (dis)advantage should be marginal. i prefer to write code
		// that can be directly used with L factor, should there be efficient means to
		// calculate that instead of R (not the case in SLAM++ so far).

		// the sparse version however requires also the structure of the transpose block matrix
		// that might mix the things up a bit (dense recursive is slower than the parallelizable
		// dense fast)

		//r_marginals.Clear(); // not required, the next line does that
		L.CopyLayoutTo(r_marginals);
		// prepare the output matrix for the marginals

		CSparseBlockMarginals_Recurrent_FBSKernel::TBlockMatrixLookup margs_lookup(r_marginals);
/*#ifdef _DEBUG
		margs_lookup.p_ground_truth = &r_marginals_gt; // set ground truth
#endif // _DEBUG*/

		Eigen::VectorXd inv_diag_L(L.n_Column_Num());
		// vector for storing inverse diagonal of L

		for(size_t n_column_i = n_first_block_col; n_column_i < n; ++ n_column_i) {
			CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_column_i, 0);
			_ASSERTE(L.n_Block_Row(n_column_i, 0) == n_column_i); // make sure it is a diagonal block
			inv_diag_L.segment(L.n_BlockColumn_Base(n_column_i),
				cur_L_diag_block.cols()) = cur_L_diag_block.diagonal().array().inverse(); // don't re-read the data, invert it now
		}
		//inv_diag_L = inv_diag_L.array().inverse(); // long vector, but it re-reads the whole vector
		// gather and invert the diagonal of L (can be done in parallel or in fixed-size chunks)
		// todo - FBS inverse? (loop while block size is the same,
		// return to enter another decision tree only if it changes)

		// t_odo - add ordered index of the last column to the arguments of this function, handle it as L shape (the ordering breaks the column to an "L" where it goes below the diagonal)

		if(n_matrix_part == mpart_LastBlock) {
			_ASSERTE(!b_structure_of_R); // just the last block, right?
			size_t n_column_i = n - 1;
			size_t n_column_i_size = L.n_BlockColumn_Column_Num(n_column_i);
			CSparseBlockMarginals_Recurrent_FBSKernel::Run<CBlockMatrixTypelist>(
				n_column_i_size, n_column_i, L, r_R, margs_lookup/*r_marginals*/, inv_diag_L,
				0); // 0 = diagonal only, 1 = copy structure of R, 2 = full column
			// FBS column width
		} else {
			for(size_t n_column_i = n; n_column_i > n_first_block_col;) {
				-- n_column_i; // here

				size_t n_column_i_size = L.n_BlockColumn_Column_Num(n_column_i);

				CSparseBlockMarginals_Recurrent_FBSKernel::Run<CBlockMatrixTypelist>(
					n_column_i_size, n_column_i, L, r_R, margs_lookup/*r_marginals*/, inv_diag_L,
					1); // 0 = diagonal only, 1 = copy structure of R, 2 = full column
				// FBS column width
			}
			// calculate the structur of R first, up to n_first_block_col

			if(n_matrix_part == mpart_LastBlock) {
				// done, have the structure of R, up to n_first_block_col, which will get invpermuted to the last column
			} else if(n_matrix_part == mpart_FullMatrix) { // note that this could be a part of the below branch
				for(size_t n_column_i = n; n_column_i > n_first_block_col;) {
					-- n_column_i; // here

					size_t n_column_i_size = L.n_BlockColumn_Column_Num(n_column_i);

					CSparseBlockMarginals_Recurrent_FBSKernel::Run<CBlockMatrixTypelist>(
						n_column_i_size, n_column_i, L, r_R, margs_lookup/*r_marginals*/, inv_diag_L,
						2); // 0 = diagonal only, 1 = copy structure of R, 2 = full column
					// FBS column width
				}
			} else if((n_matrix_part & mpart_LastColumn) == mpart_LastColumn) {
				if(mord.p_Get_InverseOrdering()[n_last_col] == n_last_col) {
					size_t n_last_col_size = L.n_BlockColumn_Column_Num(n_last_col);
					CSparseBlockMarginals_Recurrent_FBSKernel::Run<CBlockMatrixTypelist>(
						n_last_col_size, n_last_col, L, r_R, margs_lookup/*r_marginals*/, inv_diag_L,
						2); // 0 = diagonal only, 1 = copy structure of R, 2 = full column
					// FBS column width
				} else {
					throw std::runtime_error("marginals: the last column not ordered last");
					// this is somehow broken, the marginals do not have the same values as they should, don't have time to solve it now

					/*char p_s_filename[256];
					sprintf(p_s_filename, "margs_ordered_%04" _PRIsize ".tga", n);
					r_marginals.Rasterize(p_s_filename);
					// debug

					typename CUberBlockMatrix::CMakeMatrixRef<3, 3>::_TyConst
						block_i_k_mar(margs_lookup.p_GetExistingBlock(2, 3, 3, 3));*/ // this one is null and it is needed in calculating the column (11th vertex of molson)
					// more debug - this is nonzero but gets zeroed

					size_t n_above_col = mord.p_Get_InverseOrdering()[n_last_col];
					if(n_above_col) {
						size_t n_above_col_size = L.n_BlockColumn_Column_Num(n_last_col);
						CSparseBlockMarginals_Recurrent_FBSKernel::Run<CBlockMatrixTypelist>(
							n_above_col_size, n_above_col, L, r_R, margs_lookup/*r_marginals*/, inv_diag_L,
							2); // 0 = diagonal only, 1 = copy structure of R, 2 = full column
					}
					// do the part of the permuted column that is above the diagonal

					/*r_marginals.Rasterize(p_s_filename);*/
					// debug

					const size_t n_row = n_above_col;
					for(size_t i = n; i > n_above_col;) {
						-- i;

						// todo - calculate a single block at (n_row, i)
					}
				}
			}
			// calculate the last column / full matrix using the structure of R (the last column does need the full structure of R, sadly)
		}

		//margs.Convert_to_Dense(r_marginals);
		// write back to a dense matrix

		/*CUberBlockMatrix margs;
		r_R.CopyLayoutTo(margs);
		{for(size_t i = 0, n = margs.n_BlockColumn_Num(); i < n; ++ i) {
			for(size_t j = 0; j < n; ++ j) {
				margs.t_GetBlock_Log(j, i, margs.n_BlockRow_Row_Num(j),
					margs.n_BlockColumn_Column_Num(i), true, false) =
					r_marginals.block(margs.n_BlockRow_Base(j),
					margs.n_BlockColumn_Base(i), margs.n_BlockRow_Row_Num(j),
					margs.n_BlockColumn_Column_Num(i));
			}
		}}
		margs.Rasterize("margs_covariance.tga");*/ // debug

		//r_marginals.triangularView<Eigen::StrictlyLower>() =
		//	r_marginals.triangularView<Eigen::StrictlyUpper>().transpose(); // mirror upper to lower
		// transpose elements below diagonal to elements above it
		// this is *crazy* slow, takes 8 seconds on 10k matrix
		// big todo - check all the code where triangularView() is used, see if it can be done better
	}

	/**
	 *	@brief reference function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Recurrent(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R) // throw(std::bad_alloc)
	{
		{
			const size_t n = r_R.n_Column_Num(); // in elements
			r_marginals.resize(n, n);
		}

		//r_marginals.setZero(); // debug
		//r_R.Rasterize("margs_R.tga"); // debug

		const size_t n = r_R.n_BlockColumn_Num(); // in blocks!
		CUberBlockMatrix L;
		L.TransposeOf(r_R);
		// need transpose of R
		// todo - get rid of this, just make transpose ptr list, get all the layout info from R
		// note that the transpose takes less than 1% of time on linux for 300 x 300 sparse R (part of 10k)

		//printf("this is the correct blockwise Calculate_DenseMarginals_Recurrent()\n"); // debug

		_ASSERTE(r_R.b_SymmetricLayout());
		// must be symmetric (any result of cholesky would be)

		// note that the notation in the below code is slightly confusing, as any column in R
		// is also a row in L and vice versa. there are also some transpose accesses where
		// row and column are switched. it is a mess, really.

		// note that R is not used in the below code. although layout of R might reside
		// in CPU cache, the (dis)advantage should be marginal. i prefer to write code
		// that can be directly used with L factor, should there be efficient means to
		// calculate that instead of R (not the case in SLAM++ so far).

		for(size_t n_column_i = n; n_column_i > 0;) {
			-- n_column_i; // here

			size_t n_column_i_size = L.n_BlockColumn_Column_Num(n_column_i);
			size_t n_column_i_base = L.n_BlockColumn_Base(n_column_i);
			size_t n_column_i_block_num = L.n_BlockColumn_Block_Num(n_column_i); // t_odo - rename *block_i* to *column_i*
			// gets the corresponding block col

			{
				Eigen::Block<Eigen::MatrixXd> block_i_i_mar = r_marginals.block(n_column_i_base,
					n_column_i_base, n_column_i_size, n_column_i_size);
				// work with blocks also in the matrix of the marginals

				CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_column_i, 0);
				_ASSERTE(L.n_Block_Row(n_column_i, 0) == n_column_i); // make sure it is at the diagonal (L has symmetric layout)
				for(size_t n_elem = n_column_i_size; n_elem > 0;) { // go backwards
					-- n_elem; // here
#if 0 // no extra code path for the diagonal element (it has slightly simpler addressing)
					double f_L_ii = cur_L_diag_block(n_elem, n_elem);
					double f_L_ii_inv = 1 / f_L_ii;
					// get the diagonal element

					{
						double f_diag_sum = 0;
						for(size_t j = 1; j < n_column_i_block_num; ++ j) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_Block_AtColumn(n_column_i, j);
							size_t n_row_j_size = block_i_j.rows();
							size_t n_row_j_base = L.n_BlockRow_Base(L.n_Block_Row(n_column_i, j));
							// look up the row in L

							_ASSERTE(L.n_BlockColumn_Column_Num(n_column_i) == n_column_i_size);
							_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_column_i, j)) == n_row_j_size); // symmetric layout, cols and rows interchangeable
							// make sure such block can exist in L (or R)

							Eigen::Block<Eigen::MatrixXd> block_i_j_mar =
								r_marginals.block(n_column_i_base, n_row_j_base, n_column_i_size, n_row_j_size);
							// get the corresponding block in the marginals (already calculated at this moment)
							// note that (n_row_j, n_column_i) would be lower diagonal, here we access
							// the transpose block accross the diagonal

							_ASSERTE(&block_i_j_mar.row(n_elem)(0) ==
								&r_marginals.row(n_column_i_base + n_elem).segment(n_row_j_base, n_row_j_size)(0));
							// make sure we got the addressing right

							_ASSERTE(n_row_j_base > n_column_i_base); // make sure the below dot product will sample uper diagonal of the marginals
							f_diag_sum += block_i_j_mar.row(n_elem).dot(block_i_j.col(n_elem)); // todo - _FBS it
							//f_diag_sum += r_marginals.row(n_column_i_base + n_elem).segment(n_row_j_base, // t_odo - try .block<1, Dynamic>(n_row_j_base, i, 0, n_row_j_size) or something like that?
							//	n_row_j_size).dot(block_i_j.col(n_elem));
							// add dot of one column of the block with span of the current column of the marginals
							// t_odo - blockify the access to r_marginals
						}
						{
							size_t n_row_j_size = n_column_i_size;//cur_L_diag_block.rows(); // also symmetric
							size_t n_row_j_base = n_column_i_base; // it is symmetric
							// look up the row in R (= column in L)

							size_t n_first_underdiag_elem = n_elem + 1;
							size_t n_underdiag_elem_num = n_column_i_size - n_first_underdiag_elem;

							_ASSERTE(n_elem + 1 == n_column_i_size || &block_i_i_mar.col(n_elem)(n_elem + 1) ==
								&r_marginals.col(n_column_i_base + n_elem).segment(n_first_underdiag_elem +
								n_row_j_base, n_underdiag_elem_num)(0));
							// make sure we got the addressing right

							f_diag_sum += block_i_i_mar.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem).tail(n_underdiag_elem_num));
							//f_diag_sum += r_marginals.col(n_column_i_base + n_elem).segment(n_first_underdiag_elem + n_row_j_base,
							//	n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem).tail(
							//	n_row_j_size - n_first_underdiag_elem)); // todo - _FBS it
							// complete the sum
							// t_odo - blockify the access to r_marginals (through cur_L_diag_block? the info should be there)
						}
						block_i_i_mar(n_elem, n_elem) = f_L_ii_inv * (f_L_ii_inv - f_diag_sum);
					}
					// calculate the diagonal element

					for(size_t n_elem2 = n_elem; n_elem2 > 0;) { // for the rest of the elements above diagonal in this marginal block
						-- n_elem2; // here
#else // 0
					for(size_t n_elem2 = n_elem + 1; n_elem2 > 0;) { // for all the elements above or on diagonal in this marginal block
						-- n_elem2; // here
#endif // 0
						double f_L_jj = cur_L_diag_block(n_elem2, n_elem2);
						double f_L_jj_inv = 1 / f_L_jj;

						double f_diag_sum = 0;
						for(size_t j = 1; j < n_column_i_block_num; ++ j) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_Block_AtColumn(n_column_i, j);
							size_t n_row_j_size = block_i_j.rows();
							size_t n_row_j_base = L.n_BlockRow_Base(L.n_Block_Row(n_column_i, j));
							// look up the row in R (= column in L)

							_ASSERTE(L.n_BlockColumn_Column_Num(n_column_i) == n_column_i_size);
							_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_column_i, j)) == n_row_j_size); // symmetric layout, cols and rows interchangeable
							// make sure such block can exist in L (or R)

							Eigen::Block<Eigen::MatrixXd> block_i_j_mar =
								r_marginals.block(n_column_i_base, n_row_j_base, n_column_i_size, n_row_j_size);
							// get the corresponding block in the marginals (already calculated at this moment)
							// note that (n_row_j, n_column_i) would be lower diagonal, here we access
							// the transpose block accross the diagonal

							_ASSERTE(&block_i_j_mar.row(n_elem)(0) ==
								&r_marginals.row(n_column_i_base + n_elem).segment(n_row_j_base, n_row_j_size)(0));
							// make sure we got the addressing right

							_ASSERTE(n_row_j_base > n_column_i_base); // make sure the below dot product will sample uper diagonal of the marginals
							f_diag_sum += block_i_j_mar.row(n_elem).dot(block_i_j.col(n_elem2)); // todo - _FBS it
							//f_diag_sum += r_marginals.row(n_column_i_base + n_elem).segment(n_row_j_base, // t_odo - try .block<1, Dynamic>(n_row_j_base, i, 0, n_row_j_size) or something like that?
							//	n_row_j_size).dot(block_i_j.col(n_elem2)); // t_odo - _FBS it
							// add dot of one column of the block with span of the current column of the marginals
							// t_odo - blockify the access to r_marginals
						}
						{
							size_t n_row_j_size = n_column_i_size;//cur_L_diag_block.rows(); // also symmetric
							size_t n_row_j_base = n_column_i_base; // it is symmetric
							// look up the row in R (= column in L)

							size_t n_first_underdiag_elem = n_elem2 + 1;
							size_t n_underdiag_elem_num = n_column_i_size - n_first_underdiag_elem;

							_ASSERTE(n_elem2 + 1 == n_column_i_size || &block_i_i_mar.col(n_elem)(n_elem2 + 1) ==
								&r_marginals.col(n_column_i_base + n_elem).segment(n_first_underdiag_elem +
								n_row_j_base, n_underdiag_elem_num)(0));
							// make sure we got the addressing right

							f_diag_sum += block_i_i_mar.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(n_underdiag_elem_num)); // todo - _FBS it
							//f_diag_sum += r_marginals.col(n_column_i_base + n_elem).segment(n_first_underdiag_elem + n_row_j_base,
							//	n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(
							//	n_row_j_size - n_first_underdiag_elem));
							// complete the sum
							// t_odo - blockify the access to r_marginals (through cur_L_diag_block? the info should be there)
						}
						// note that this is NOT the same code as for the diagonal entries, the final modification, the writes and the addressing with n_elem2 are different
						// note that n_elem2 could be also introduced in the diagonal branch above and held equal to n_elem

						if(n_elem2 == n_elem) {
							block_i_i_mar(n_elem, n_elem) = f_L_jj_inv * (f_L_jj_inv - f_diag_sum);
							// diagonal elems have a different formula
						} else {
							block_i_i_mar(n_elem2, n_elem) = block_i_i_mar(n_elem, n_elem2) =
								f_L_jj_inv * -f_diag_sum; // use _jj! (different diagonal elem in the same block)
						}
						// this is not required, if the extra code path for the diagonal element is enabled
						// (only the else part is required then, the branch is avoided)

						/*_ASSERTE(&block_i_i_mar(n_elem2, n_elem) == &r_marginals(n_column_i_base + n_elem2, n_column_i_base + n_elem));
						_ASSERTE(&block_i_i_mar(n_elem, n_elem2) == &r_marginals(n_column_i_base + n_elem, n_column_i_base + n_elem2));
						_ASSERTE(r_marginals_gt(n_column_i_base + n_elem, n_column_i_base + n_elem2) ==
							r_marginals_gt(n_column_i_base + n_elem2, n_column_i_base + n_elem)); // strong == ok here, should be symmetric
						_ASSERTE(fabs(r_marginals_gt(n_column_i_base + n_elem2, n_column_i_base + n_elem) -
							block_i_i_mar(n_elem2, n_elem)) < std::max(1e-5,
							1e-5 * r_marginals_gt(n_column_i_base + n_elem2, n_column_i_base + n_elem)));*/
						// compare to ground-truth
					}
					// calculate the off-diagonal elements inside this block
				}
			}
			// do the block at the diagonal

			for(size_t n_column_j = n_column_i; n_column_j > 0;) { // note that this is only required for the blocks that are required on output or blocks that are coincident with nnz blocks in the L or R factor
				-- n_column_j; // here

				size_t n_column_j_size = L.n_BlockColumn_Column_Num(n_column_j);
				size_t n_column_j_base = L.n_BlockColumn_Base(n_column_j);
				size_t n_column_j_block_num = L.n_BlockColumn_Block_Num(n_column_j); // t_odo - rename *block_j* to *column_j*
				// gets the corresponding block column

				_ASSERTE(n_column_j_base < n_column_i_base); // strictly upper even
				Eigen::Block<Eigen::MatrixXd> block_j_i_mar = r_marginals.block(n_column_j_base,
					n_column_i_base, n_column_j_size, n_column_i_size); // in upper triangular // t_odo rename this
				// work with blocks also in the matrix of the marginals

				CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_column_j, 0); // t_odo - rename those to have "L" in them
				_ASSERTE(L.n_Block_Row(n_column_j, 0) == n_column_j); // make sure it is at the diagonal (L has symmetric layout)

				block_j_i_mar.setZero();
				for(size_t k = 1; k < n_column_j_block_num; ++ k) { // all the blocks except the diagonal
					CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_Block_AtColumn(n_column_j, k);
					size_t n_row_k_size = block_j_k.rows();
					size_t n_row_k_base = L.n_BlockRow_Base(L.n_Block_Row(n_column_j, k));
					// look up the row in R (= column in L)

					_ASSERTE(L.n_BlockColumn_Column_Num(n_column_i) == n_column_i_size);
					_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_column_j, k)) == n_row_k_size); // symmetric layout, cols and rows interchangeable
					// make sure such block can exist in L (or R)

					Eigen::Block<Eigen::MatrixXd> block_i_k_mar = (n_row_k_base >= n_column_i_base)?
						r_marginals.block(n_column_i_base, n_row_k_base, n_column_i_size, n_row_k_size) :
						r_marginals.block(n_row_k_base, n_column_i_base, n_row_k_size, n_column_i_size);
					// get the corresponding block in the marginals (already calculated at this moment)

					if(n_row_k_base >= n_column_i_base) { // would be in lower diagonal
						// todo - this branch will be used for all subsequent blocks; make another loop here without the test, then break out from the outer loop

						_ASSERTE(&block_i_k_mar.row(/*n_elem*/0)(0) ==
							&r_marginals.row(n_column_i_base + /*n_elem*/0).segment(
							n_row_k_base, n_row_k_size)(0) &&
							(block_i_k_mar.rows() == 1 || &block_i_k_mar.row(/*n_elem*/1)(0) ==
							&r_marginals.row(n_column_i_base + /*n_elem*/1).segment(
							n_row_k_base, n_row_k_size)(0))); // t_odo - incomplete check, need to check at least one more address if size is greater than 1
						// make sure we got the addressing right (checks the first elem and if there is one, also the second elem)

						block_j_i_mar.noalias() += (block_i_k_mar * block_j_k).transpose(); // todo - try with lazyProduct(), try reverse order product and transpose the factors themselves
						//f_diag_sum += block_i_k_mar.row(n_elem).dot(block_j_k.col(n_elem2)); // t_odo - _FBS it
						//f_diag_sum += r_marginals.row(n_column_i_base + n_elem).segment(n_row_k_base,
						//	n_row_k_size).dot(block_j_k.col(n_elem2));
					} else {
						_ASSERTE(&block_i_k_mar.transpose().row(/*n_elem*/0)(0) ==
							&r_marginals.col(n_column_i_base + /*n_elem*/0).segment(
							n_row_k_base, n_row_k_size)(0) &&
							(block_i_k_mar.cols() == 1 || &block_i_k_mar.transpose().row(/*n_elem*/1)(0) ==
							&r_marginals.col(n_column_i_base + /*n_elem*/1).segment(
							n_row_k_base, n_row_k_size)(0))); // t_odo - incomplete check, need to check at least one more address if size is greater than 1
						// make sure we got the addressing right (checks the first elem and if there is one, also the second elem)

						block_j_i_mar.noalias() += block_j_k.transpose() * block_i_k_mar; // todo - try with lazyProduct()
						//f_diag_sum += block_i_k_mar.transpose().row(n_elem).dot(block_j_k.col(n_elem2)); // t_odo - _FBS it
						//f_diag_sum += r_marginals.col(n_column_i_base + n_elem).segment(n_row_k_base,
						//	n_row_k_size).dot(block_j_k.col(n_elem2)); // t_odo - _FBS it
					}
					// add dot of one column of the block with span of the current column of the marginals
				}
				// calculate the first part of the sum in completely blockwise manner

				for(size_t n_elem = n_column_i_size; n_elem > 0;) { // go backwards
					-- n_elem; // here
#if 1 // different code for the last row of the result block
					{
						size_t n_elem2 = n_column_j_size - 1;
						// the last diagonal

						double f_L_jj_inv = 1 / cur_L_diag_block(n_elem2, n_elem2);
						// get the diagonal block and element

						double f_diag_sum = block_j_i_mar(n_elem2, n_elem); // (calculated blockwise)
						// continue from here

						// no additional dot product, there is nothing below (would be size 0 dot product, complicated _FBS)

						block_j_i_mar(n_elem2, n_elem) = -f_diag_sum * f_L_jj_inv;
						// write only upper triangular, will mirror it at once (or not at all)

						/*_ASSERTE(&block_j_i_mar(n_elem2, n_elem) == &r_marginals(n_column_j_base + n_elem2,
							n_column_i_base + n_elem));
						_ASSERTE(fabs(block_j_i_mar(n_elem2, n_elem) - r_marginals_gt(n_column_j_base + n_elem2,
							n_column_i_base + n_elem)) < std::max(1e-5, 1e-5 * r_marginals_gt(n_column_j_base +
							n_elem2, n_column_i_base + n_elem)));*/
						// make sure it is correct
					}

					// the loop skips one elem
					for(size_t n_elem2 = n_column_j_size - 1; n_elem2 > 0;) { // go backwards
#else // 1
					for(size_t n_elem2 = n_column_j_size; n_elem2 > 0;) { // go backwards
#endif // 1
						-- n_elem2; // here

						double f_L_jj_inv = 1 / cur_L_diag_block(n_elem2, n_elem2);
						// get the diagonal block and element

						double f_diag_sum = block_j_i_mar(n_elem2, n_elem); // (calculated blockwise)
						// continue from here

						{
							size_t n_row_k_size = n_column_j_size;//cur_L_diag_block.rows(); // also symmetric
							size_t n_row_k_base = n_column_j_base; // it is symmetric
							// look up the row in R (= column in L)

							size_t n_first_underdiag_elem = n_elem2 + 1;
							size_t n_underdiag_elem_num = n_column_j_size - n_first_underdiag_elem;
							f_diag_sum += block_j_i_mar.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(n_underdiag_elem_num)); // todo - _FBS it
							//f_diag_sum += r_marginals.col(n_column_i_base + n_elem).segment(n_first_underdiag_elem +
							//	n_row_k_base, n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(
							//	n_row_k_size - n_first_underdiag_elem)); // t_odo - _FBS it
							// complete the sum
						}
						// this is complicated in block approach (maybe not, maybe it is just the task for a triangular view)

						block_j_i_mar(n_elem2, n_elem) = -f_diag_sum * f_L_jj_inv;
						// write only upper triangular, will mirror it at once (or not at all)

						/*_ASSERTE(&block_j_i_mar(n_elem2, n_elem) == &r_marginals(n_column_j_base + n_elem2,
							n_column_i_base + n_elem));
						_ASSERTE(fabs(block_j_i_mar(n_elem2, n_elem) - r_marginals_gt(n_column_j_base + n_elem2,
							n_column_i_base + n_elem)) < std::max(1e-5, 1e-5 * r_marginals_gt(n_column_j_base +
							n_elem2, n_column_i_base + n_elem)));*/
						// make sure it is correct
					}
				}
				// t_odo - get rid of this, do it in blockwise manner
				//block_j_i_mar = block_j_i_mar/*.triangularView<Eigen::StrictlyLower>()*/ *
				//	cur_L_diag_block.triangularView<Eigen::StrictlyLower>().transpose();
				// does not work, will not work, have to do it blockwise
				// t_odo - separate the two loops below, see how these are interdependent
				// the triangular part can't be done in blockwise manner as the order of elementwise
				// calculation is important
			}
		}

		/*CUberBlockMatrix margs;
		r_R.CopyLayoutTo(margs);
		{for(size_t i = 0, n = margs.n_BlockColumn_Num(); i < n; ++ i) {
			for(size_t j = 0; j < n; ++ j) {
				margs.t_GetBlock_Log(j, i, margs.n_BlockRow_Row_Num(j),
					margs.n_BlockColumn_Column_Num(i), true, false) =
					r_marginals.block(margs.n_BlockRow_Base(j),
					margs.n_BlockColumn_Base(i), margs.n_BlockRow_Row_Num(j),
					margs.n_BlockColumn_Column_Num(i));
			}
		}}
		margs.Rasterize("margs_covariance.tga");*/ // debug

		r_marginals.triangularView<Eigen::StrictlyLower>() =
			r_marginals.triangularView<Eigen::StrictlyUpper>().transpose(); // mirror upper to lower
		// transpose elements below diagonal to elements above it
	}

	/**
	 *	@brief development version of the recurrent function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_marginals_gt is the marginals matrix ground truth
	 *	@param[in] r_R is the Cholesky factor
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Recurrent_Devel(Eigen::MatrixXd &r_marginals,
		const Eigen::MatrixXd &r_marginals_gt, const CUberBlockMatrix &r_R) // throw(std::bad_alloc)
	{
		{
			const size_t n = r_R.n_Column_Num(); // in elements
			r_marginals.resize(n, n);
		}

		const size_t n = r_R.n_BlockColumn_Num(); // in blocks!
		CUberBlockMatrix L;
		L.TransposeOf(r_R); // todo - get rid of this, just make transpose ptr list, get all the layout info from R
		// need transpose of R

		//printf("this is the correct blockwise Calculate_DenseMarginals_Recurrent()\n"); // debug

		_ASSERTE(r_R.b_SymmetricLayout());
		// must be sym

		for(size_t n_block_i = n; n_block_i > 0;) {
			-- n_block_i; // here

			size_t n_block_i_size = L.n_BlockColumn_Column_Num(n_block_i);
			size_t n_block_i_base = L.n_BlockColumn_Base(n_block_i);
			size_t n_block_i_block_num = L.n_BlockColumn_Block_Num(n_block_i); // todo - rename *block_i* to *column_i*
			// gets the corresponding block col

			{
				Eigen::Block<Eigen::MatrixXd> dest_block = r_marginals.block(n_block_i_base,
					n_block_i_base, n_block_i_size, n_block_i_size);
				// work with blocks also in the matrix of the marginals

				CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_i, 0);
				_ASSERTE(L.n_Block_Row(n_block_i, 0) == n_block_i); // make sure it is at the diagonal (L has symmetric layout)
				for(size_t n_elem = n_block_i_size; n_elem > 0;) { // go backwards
					-- n_elem; // here

					double f_L_ii = cur_L_diag_block(n_elem, n_elem);
					double f_L_ii_inv = 1 / f_L_ii;
					// get the diagonal element

					{
						double f_diag_sum = 0;
						for(size_t j = 1; j < n_block_i_block_num; ++ j) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_Block_AtColumn(n_block_i, j);
							size_t n_block_row_size = block_i_j.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_i, j)); // todo - rename this to col
							// look up the row in R (= column in L)

							_ASSERTE(L.n_BlockColumn_Column_Num(n_block_i) == n_block_i_size);
							_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_block_i, j)) == n_block_row_size); // symmetric layout, cols and rows interchangeable
							// make sure such block can exist in L (or R)

							Eigen::Block<Eigen::MatrixXd> block_i_j_mar =
								r_marginals.block(n_block_i_base, n_block_row_base, n_block_i_size, n_block_row_size);
							// get the corresponding block in the marginals (already calculated at this moment)

							_ASSERTE(&block_i_j_mar.row(n_elem)(0) ==
								&r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base, n_block_row_size)(0));
							// make sure we got the addressing right

							_ASSERTE(n_block_row_base > n_block_i_base); // make sure the below dot product will sample uper diagonal of the marginals
							f_diag_sum += block_i_j_mar.row(n_elem).dot(block_i_j.col(n_elem)); // todo - _FBS it
							//f_diag_sum += r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base, // t_odo - try .block<1, Dynamic>(n_block_row_base, i, 0, n_block_row_size) or something like that?
							//	n_block_row_size).dot(block_i_j.col(n_elem));
							// add dot of one column of the block with span of the current column of the marginals
							// t_odo - blockify the access to r_marginals
						}
						{
							size_t n_block_row_size = n_block_i_size;//cur_L_diag_block.rows(); // also symmetric
							size_t n_block_row_base = n_block_i_base; // it is symmetric
							// look up the row in R (= column in L)

							size_t n_first_underdiag_elem = n_elem + 1;
							size_t n_underdiag_elem_num = n_block_i_size - n_first_underdiag_elem;

							_ASSERTE(n_elem + 1 == n_block_i_size || &dest_block.col(n_elem)(n_elem + 1) ==
								&r_marginals.col(n_block_i_base + n_elem).segment(n_first_underdiag_elem +
								n_block_row_base, n_underdiag_elem_num)(0));
							// make sure we got the addressing right

							f_diag_sum += dest_block.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem).tail(n_underdiag_elem_num));
							//f_diag_sum += r_marginals.col(n_block_i_base + n_elem).segment(n_first_underdiag_elem + n_block_row_base,
							//	n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem).tail(
							//	n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
							// complete the sum
							// t_odo - blockify the access to r_marginals (through cur_L_diag_block? the info should be there)
						}
						dest_block(n_elem, n_elem) = f_L_ii_inv * (f_L_ii_inv - f_diag_sum);
					}
					// calculate the diagonal element

					_ASSERTE(&dest_block(n_elem, n_elem) == &r_marginals(n_block_i_base + n_elem, n_block_i_base + n_elem));
					_ASSERTE(fabs(r_marginals_gt(n_block_i_base + n_elem, n_block_i_base + n_elem) -
						dest_block(n_elem, n_elem)) < std::max(1e-5,
						1e-5 * r_marginals_gt(n_block_i_base + n_elem, n_block_i_base + n_elem)));
					// compare to ground-truth

					for(size_t n_elem2 = n_elem; n_elem2 > 0;) { // for the rest of the elements above diagonal in this marginal block
						-- n_elem2; // here

						double f_L_jj = cur_L_diag_block(n_elem2, n_elem2);
						double f_L_jj_inv = 1 / f_L_jj;

						double f_diag_sum = 0;
						for(size_t j = 1; j < n_block_i_block_num; ++ j) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_Block_AtColumn(n_block_i, j);
							size_t n_block_row_size = block_i_j.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_i, j));
							// look up the row in R (= column in L)

							_ASSERTE(L.n_BlockColumn_Column_Num(n_block_i) == n_block_i_size);
							_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_block_i, j)) == n_block_row_size); // symmetric layout, cols and rows interchangeable
							// make sure such block can exist in L (or R)

							Eigen::Block<Eigen::MatrixXd> block_i_j_mar =
								r_marginals.block(n_block_i_base, n_block_row_base, n_block_i_size, n_block_row_size);
							// get the corresponding block in the marginals (already calculated at this moment)

							_ASSERTE(&block_i_j_mar.row(n_elem)(0) ==
								&r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base, n_block_row_size)(0));
							// make sure we got the addressing right

							_ASSERTE(n_block_row_base > n_block_i_base); // make sure the below dot product will sample uper diagonal of the marginals
							f_diag_sum += block_i_j_mar.row(n_elem).dot(block_i_j.col(n_elem2)); // todo - _FBS it
							//f_diag_sum += r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, i, 0, n_block_row_size) or something like that?
							//	n_block_row_size).dot(block_i_j.col(n_elem2)); // todo - _FBS it
							// add dot of one column of the block with span of the current column of the marginals
							// todo - blockify the access to r_marginals
						}
						{
							size_t n_block_row_size = n_block_i_size;//cur_L_diag_block.rows(); // also symmetric
							size_t n_block_row_base = n_block_i_base; // it is symmetric
							// look up the row in R (= column in L)

							size_t n_first_underdiag_elem = n_elem2 + 1;
							size_t n_underdiag_elem_num = n_block_i_size - n_first_underdiag_elem;

							_ASSERTE(n_elem2 + 1 == n_block_i_size || &dest_block.col(n_elem)(n_elem2 + 1) ==
								&r_marginals.col(n_block_i_base + n_elem).segment(n_first_underdiag_elem +
								n_block_row_base, n_underdiag_elem_num)(0));
							// make sure we got the addressing right

							f_diag_sum += dest_block.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(n_underdiag_elem_num)); // todo - _FBS it
							//f_diag_sum += r_marginals.col(n_block_i_base + n_elem).segment(n_first_underdiag_elem + n_block_row_base,
							//	n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(
							//	n_block_row_size - n_first_underdiag_elem));
							// complete the sum
							// t_odo - blockify the access to r_marginals (through cur_L_diag_block? the info should be there)
						}
						// note that this is NOT the same code as for the diagonal entries, the final modification, the writes and the addressing with n_elem2 are different
						// note that n_elem2 could be also introduced in the diagonal branch above and held equal to n_elem

						dest_block(n_elem2, n_elem) = dest_block(n_elem, n_elem2) = f_L_jj_inv * -f_diag_sum; // use _jj! (different diagonal elem in the same block)

						_ASSERTE(&dest_block(n_elem2, n_elem) == &r_marginals(n_block_i_base + n_elem2, n_block_i_base + n_elem));
						_ASSERTE(&dest_block(n_elem, n_elem2) == &r_marginals(n_block_i_base + n_elem, n_block_i_base + n_elem2));
						_ASSERTE(r_marginals_gt(n_block_i_base + n_elem, n_block_i_base + n_elem2) ==
							r_marginals_gt(n_block_i_base + n_elem2, n_block_i_base + n_elem)); // strong == ok here, should be symmetric
						_ASSERTE(fabs(r_marginals_gt(n_block_i_base + n_elem2, n_block_i_base + n_elem) -
							dest_block(n_elem2, n_elem)) < std::max(1e-5,
							1e-5 * r_marginals_gt(n_block_i_base + n_elem2, n_block_i_base + n_elem)));
						// compare to ground-truth
					}
					// calculate the off-diagonal elements inside this block
				}
			}
			// do the block at the diagonal

			for(size_t n_block_j = n_block_i; n_block_j > 0;) { // note that this is only required for the blocks that are required on output or blocks that are coincident with nnz blocks in the L or R factor
				-- n_block_j; // here

				size_t n_block_j_size = L.n_BlockColumn_Column_Num(n_block_j);
				size_t n_block_j_base = L.n_BlockColumn_Base(n_block_j);
				size_t n_block_j_block_num = L.n_BlockColumn_Block_Num(n_block_j); // todo - rename *block_j* to *row_j*
				// gets the corresponding block row

				_ASSERTE(n_block_j_base < n_block_i_base); // strictly upper even
				Eigen::Block<Eigen::MatrixXd> dest_block = r_marginals.block(n_block_j_base,
					n_block_i_base, n_block_j_size, n_block_i_size); // in upper triangular // todo rename this
				// work with blocks also in the matrix of the marginals

				CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_j, 0); // todo - rename those to have "L" in them
				_ASSERTE(L.n_Block_Row(n_block_j, 0) == n_block_j); // make sure it is at the diagonal (L has symmetric layout)

#if 0
				Eigen::MatrixXd result_block(n_block_j_size, n_block_i_size); // t_odo - get rid of this, can do it inplace in dest_block
#else // 0
				Eigen::Block<Eigen::MatrixXd> &result_block = dest_block; // now it is the same thing
#endif // 0
				result_block.setZero();
				// to contain data of blockwise calculation (for comparison)

				for(size_t k = 1; k < n_block_j_block_num; ++ k) { // all the blocks except the diagonal
					CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_Block_AtColumn(n_block_j, k);
					size_t n_block_row_size = block_j_k.rows();
					size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_j, k));
					// look up the row in R (= column in L)

					_ASSERTE(L.n_BlockColumn_Column_Num(n_block_i) == n_block_i_size);
					_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_block_j, k)) == n_block_row_size); // symmetric layout, cols and rows interchangeable
					// make sure such block can exist in L (or R)

					Eigen::Block<Eigen::MatrixXd> block_i_k_mar = (n_block_row_base >= n_block_i_base)?
						r_marginals.block(n_block_i_base, n_block_row_base, n_block_i_size, n_block_row_size) :
						r_marginals.block(n_block_row_base, n_block_i_base, n_block_row_size, n_block_i_size);
					// get the corresponding block in the marginals (already calculated at this moment)

					if(n_block_row_base >= n_block_i_base) { // would be in lower diagonal
						// todo - this branch will be used for all subsequent blocks; make another loop here without the test, then break out from the outer loop

						_ASSERTE(&block_i_k_mar.row(/*n_elem*/0)(0) ==
							&r_marginals.row(n_block_i_base + /*n_elem*/0).segment(
							n_block_row_base, n_block_row_size)(0) &&
							(block_i_k_mar.rows() == 1 || &block_i_k_mar.row(/*n_elem*/1)(0) ==
							&r_marginals.row(n_block_i_base + /*n_elem*/1).segment(
							n_block_row_base, n_block_row_size)(0))); // t_odo - incomplete check, need to check at least one more address if size is greater than 1
						// make sure we got the addressing right (checks the first elem and if there is one, also the second elem)

						result_block.noalias() += (block_i_k_mar * block_j_k).transpose(); // todo - try with lazyProduct(), try reverse order product and transpose the factors themselves
						//f_diag_sum += block_i_k_mar.row(n_elem).dot(block_j_k.col(n_elem2)); // todo - _FBS it
						//f_diag_sum += r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base,
						//	n_block_row_size).dot(block_j_k.col(n_elem2));
					} else {
						_ASSERTE(&block_i_k_mar.transpose().row(/*n_elem*/0)(0) ==
							&r_marginals.col(n_block_i_base + /*n_elem*/0).segment(
							n_block_row_base, n_block_row_size)(0) &&
							(block_i_k_mar.cols() == 1 || &block_i_k_mar.transpose().row(/*n_elem*/1)(0) ==
							&r_marginals.col(n_block_i_base + /*n_elem*/1).segment(
							n_block_row_base, n_block_row_size)(0))); // t_odo - incomplete check, need to check at least one more address if size is greater than 1
						// make sure we got the addressing right (checks the first elem and if there is one, also the second elem)

						result_block.noalias() += block_j_k.transpose() * block_i_k_mar; // todo - try with lazyProduct()
						//f_diag_sum += block_i_k_mar.transpose().row(n_elem).dot(block_j_k.col(n_elem2)); // todo - _FBS it
						//f_diag_sum += r_marginals.col(n_block_i_base + n_elem).segment(n_block_row_base,
						//	n_block_row_size).dot(block_j_k.col(n_elem2)); // todo - _FBS it
					}
					// add dot of one column of the block with span of the current column of the marginals
				}
				// calculate the first part of the sum in completely blockwise manner

				for(size_t n_elem = n_block_i_size; n_elem > 0;) { // go backwards
					-- n_elem; // here
#if 1 // different code for the last row of the result block
					{
						size_t n_elem2 = n_block_j_size - 1;
						// the last diagonal

						double f_L_jj_inv = 1 / cur_L_diag_block(n_elem2, n_elem2);
						// get the diagonal block and element

						double f_diag_sum = result_block(n_elem2, n_elem); // (calculated blockwise)
						// continue from here

						// no additional dot product, there is nothing below (would be size 0 dot product, complicated _FBS)

						dest_block(n_elem2, n_elem) = -f_diag_sum * f_L_jj_inv;
						// write only upper triangular, will mirror it at once (or not at all)

						_ASSERTE(&dest_block(n_elem2, n_elem) == &r_marginals(n_block_j_base + n_elem2,
							n_block_i_base + n_elem));
						_ASSERTE(fabs(dest_block(n_elem2, n_elem) - r_marginals_gt(n_block_j_base + n_elem2,
							n_block_i_base + n_elem)) < std::max(1e-5, 1e-5 * r_marginals_gt(n_block_j_base +
							n_elem2, n_block_i_base + n_elem)));
						// make sure it is correct
					}

					// the loop skips one elem
					for(size_t n_elem2 = n_block_j_size - 1; n_elem2 > 0;) { // go backwards
#else // 1
					for(size_t n_elem2 = n_block_j_size; n_elem2 > 0;) { // go backwards
#endif // 1
						-- n_elem2; // here

						double f_L_jj_inv = 1 / cur_L_diag_block(n_elem2, n_elem2);
						// get the diagonal block and element

#if 0 // calculates the result elementwise, compares to result_block (calculated blockwise)
						_ASSERTE(dest_block.data() != result_block.data()); // if this triggers, need to change the above ifdef, containing declaration of result_block accordingly
						double f_diag_sum = 0; // this is a matrix (!) now
						for(size_t k = 1; k < n_block_j_block_num; ++ k) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_Block_AtColumn(n_block_j, k);
							size_t n_block_row_size = block_j_k.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_j, k));
							// look up the row in R (= column in L)

							_ASSERTE(L.n_BlockColumn_Column_Num(n_block_i) == n_block_i_size);
							_ASSERTE(L.n_BlockColumn_Column_Num(L.n_Block_Row(n_block_j, k)) == n_block_row_size); // symmetric layout, cols and rows interchangeable
							// make sure such block can exist in L (or R)

							Eigen::Block<Eigen::MatrixXd> block_i_k_mar = (n_block_row_base >= n_block_i_base)?
								r_marginals.block(n_block_i_base, n_block_row_base, n_block_i_size, n_block_row_size) :
								r_marginals.block(n_block_row_base, n_block_i_base, n_block_row_size, n_block_i_size);
							// get the corresponding block in the marginals (already calculated at this moment)

							if(n_block_row_base >= n_block_i_base) { // would be in lower diagonal
								// todo - this branch will be used for all subsequent blocks; make another loop here without the test, then break out from the outer loop

								_ASSERTE(&block_i_k_mar.row(n_elem)(0) ==
									&r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base, n_block_row_size)(0));
								// make sure we got the addressing right

								f_diag_sum += block_i_k_mar.row(n_elem).dot(block_j_k.col(n_elem2)); // todo - _FBS it
								//f_diag_sum += r_marginals.row(n_block_i_base + n_elem).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, j, 0, n_block_row_size) or something like that?
								//	n_block_row_size).dot(block_j_k.col(n_elem2));
							} else {
								_ASSERTE(&block_i_k_mar.transpose().row(n_elem)(0) ==
									&r_marginals.col(n_block_i_base + n_elem).segment(n_block_row_base, n_block_row_size)(0));
								// make sure we got the addressing right

								f_diag_sum += block_i_k_mar.transpose().row(n_elem).dot(block_j_k.col(n_elem2)); // todo - _FBS it
								//f_diag_sum += r_marginals.col(n_block_i_base + n_elem).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, j, 0, n_block_row_size) or something like that?
								//	n_block_row_size).dot(block_j_k.col(n_elem2)); // todo - _FBS it
							}
							// add dot of one column of the block with span of the current column of the marginals
						}
						// t_odo - access marginals by blocks before moving on

						_ASSERTE(fabs(f_diag_sum - result_block(n_elem2, n_elem)) < std::max(1e-5, 1e-5 * f_diag_sum));
						// check blockwise calculation
#else // 0
						double f_diag_sum = result_block(n_elem2, n_elem); // (calculated blockwise)
						// continue from here
#endif // 0
						{
							size_t n_block_row_size = n_block_j_size;//cur_L_diag_block.rows(); // also symmetric
							size_t n_block_row_base = n_block_j_base; // it is symmetric
							// look up the row in R (= column in L)

							size_t n_first_underdiag_elem = n_elem2 + 1;
							size_t n_underdiag_elem_num = n_block_j_size - n_first_underdiag_elem;
							f_diag_sum += dest_block.col(n_elem).segment(n_first_underdiag_elem,
								n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(n_underdiag_elem_num)); // todo - _FBS it
							//f_diag_sum += r_marginals.col(n_block_i_base + n_elem).segment(n_first_underdiag_elem +
							//	n_block_row_base, n_underdiag_elem_num).dot(cur_L_diag_block.col(n_elem2).tail(
							//	n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
							// complete the sum
							// todo - this reuses the values of the diagonal block computed above
						}
						// this is complicated in block approach (maybe not, maybe it is just the task for a triangular view)

						dest_block(n_elem2, n_elem) = -f_diag_sum * f_L_jj_inv;
						// write only upper triangular, will mirror it at once (or not at all)

						_ASSERTE(&dest_block(n_elem2, n_elem) == &r_marginals(n_block_j_base + n_elem2,
							n_block_i_base + n_elem));
						_ASSERTE(fabs(dest_block(n_elem2, n_elem) - r_marginals_gt(n_block_j_base + n_elem2,
							n_block_i_base + n_elem)) < std::max(1e-5, 1e-5 * r_marginals_gt(n_block_j_base +
							n_elem2, n_block_i_base + n_elem)));
						// make sure it is correct
					}
				}
				// todo - get rid of this, do it in blockwise manner
				//result_block = dest_block/*.triangularView<Eigen::StrictlyLower>()*/ *
				//	cur_L_diag_block.triangularView<Eigen::StrictlyLower>().transpose();
				// does not work, will not work
				// todo - separate the two loops below, see how these are interdependent
				// the triangular part can't be done in blockwise manner as the order of elementwise
				// calculation is important
			}

			continue; // this is the end of a single block column process
#if 0
			{
				for(size_t j = i; j > 0;) { // note that this is only required for the elements that are required on output or elements that are above nnz in the L or R factor
					-- j; // j is i in the book
					// i is k in the book

					size_t n_block_j_size;
					size_t n_block_j = L.n_Find_BlockColumn(j, n_block_j_size);
					size_t n_block_j_base = L.n_BlockColumn_Base(n_block_j);
					size_t n_block_j_block_num = L.n_BlockColumn_Block_Num(n_block_j);

					CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_j, 0);
					double f_L_jj = cur_L_diag_block(j - n_block_j_base, j - n_block_j_base);
					double f_L_jj_inv = 1 / f_L_jj;
					// get the diagonal block and element

					double f_diag_sum = 0;
					for(size_t k = 1; k < n_block_j_block_num; ++ k) { // all the blocks except the diagonal
						CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_Block_AtColumn(n_block_j, k);
						size_t n_block_row_size = block_j_k.rows();
						size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_j, k));
						// look up the row in R (= column in L)

						f_diag_sum += r_marginals.row(i).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, j, 0, n_block_row_size) or something like that?
							n_block_row_size).dot(block_j_k.col(j - n_block_j_base)); // todo - _FBS it
						// add dot of one column of the block with span of the current column of the marginals
					}
					{
						size_t n_block_row_size = n_block_j_size;//cur_L_diag_block.rows(); // also symmetric
						size_t n_block_row_base = n_block_j_base; // it is symmetric
						// look up the row in R (= column in L)

						size_t n_subblock_col = j - n_block_j_base;
						size_t n_first_underdiag_elem = n_subblock_col + 1;
						size_t n_underdiag_elem_num = n_block_j_size - n_first_underdiag_elem;
						f_diag_sum += r_marginals.row(i).segment(n_first_underdiag_elem + n_block_row_base,
							n_underdiag_elem_num).dot(cur_L_diag_block.col(n_subblock_col).tail(
							n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
						// complete the sum
					}

#if 0 && defined(_DEBUG)
					double f_sum_Rjk_Cik = 0;//, f_sum_part0, f_sum_first_elem;
					for(size_t k = j + 1; k < n; ++ k) {
						size_t n_block_k_size;
						size_t n_block_k = L.n_Find_BlockColumn(k, n_block_k_size);
						size_t n_block_k_base = L.n_BlockColumn_Base(n_block_k);

						CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_GetBlock_Log(n_block_k, n_block_j);
						if(!block_j_k.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks

						f_sum_Rjk_Cik += r_marginals(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base);
						// product
					}
					_ASSERTE(fabs(f_sum_Rjk_Cik - f_diag_sum) < std::max(1e-5, fabs(f_sum_Rjk_Cik)));
					// use the "old" code to verify corectness
#endif // 0 && _DEBUG

					/*{ // debugging of more recurrent formula
						size_t m = i + 1; // wha?
						size_t n_block_m_size;
						size_t n_block_m = R.n_Find_BlockColumn(m, n_block_m_size);
						size_t n_block_m_base = R.n_BlockColumn_Base(n_block_m);
						CUberBlockMatrix::_TyMatrixXdRef diag_block_m =
							R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_m) - 1, n_block_m);
						double f_R_mm = cur_L_diag_block(m - n_block_m_base, m - n_block_m_base);
						double f_sum_part1 = -(C_dep(m, m) * f_R_mm - 1 / f_R_mm);
						double f_sum_parted = f_sum_part0 + f_sum_part1;
						double f_sum_err = fabs(f_sum_parted - f_sum_Rjk_Cik);
					}*/

					/*CUberBlockMatrix::_TyMatrixXdRef block_j_j =
						L.t_Block_AtColumn(n_block_j, L.n_BlockColumn_Block_Num(n_block_j) - 1);
					double L_j_j = block_j_j(j - n_block_j_base, j - n_block_j_base);
					// get another diagonal element of R*/ // unused

					r_marginals(i, j) = r_marginals(j, i) = -f_diag_sum * f_L_jj_inv; // write only upper triangular, will mirror it at once (or not at all)
					// todo - see to it that only upper triangular is accessed, add _ASSERTE() where reading it to make sure lower is not touched
				}
				// calculate C_dep_{n - 1 - i} by recurrent formula
			}
#endif // 0
		}

		//r_marginals.triangularView<Eigen::StrictlyUpper>() =
		//	r_marginals.triangularView<Eigen::StrictlyLower>().transpose(); // mirror lower to upper
		r_marginals.triangularView<Eigen::StrictlyLower>() =
			r_marginals.triangularView<Eigen::StrictlyUpper>().transpose(); // mirror upper to lower
		// transpose elements below diagonal to elements above it
	}

	/**
	 *	@brief recurrent elementwise function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_Recurrent_Elemwise(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);

		CUberBlockMatrix L;
		L.TransposeOf(r_R);
		// need transpose of R

		//printf("this is the correct Calculate_DenseMarginals_Recurrent()\n"); // debug

		for(size_t i = n; i > 0;) {
			-- i; // here

			/*if(i == n - 1) { // the last row / col is easy, as it has no refs to the next ones
				Eigen::MatrixXd::ColXpr last_col = r_marginals.col(n - 1);
				size_t lb = r_R.n_BlockColumn_Num() - 1;
				CUberBlockMatrix::_TyMatrixXdRef last_block =
					r_R.t_Block_AtColumn(lb, r_R.n_BlockColumn_Block_Num(lb) - 1);
				last_col.setZero(); // !!
				last_col(n - 1) = 1 / last_block(last_block.rows() - 1, last_block.cols() - 1);
				r_R.UpperTriangular_Solve(&last_col(0), n); // all columns needed
				// calculates the whole last column of C

				r_marginals.row(n - 1).head(n - 1) = r_marginals.col(n - 1).head(n - 1).transpose();
				// copy that also to the last row, to form the full matrix and not just upper-triangular
			} else*/
			// the code above is simple, but the usolve is quite expensive

			{ // columns with references to the subsequent columns
				size_t n_block_column_size;
				size_t n_block_column = L.n_Find_BlockColumn(i, n_block_column_size);
				size_t n_block_column_base = L.n_BlockColumn_Base(n_block_column);
				size_t n_block_column_block_num = L.n_BlockColumn_Block_Num(n_block_column);
				// gets the corresponding block col (can use decrementing strategy like in C_direct)

				{
					CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_column, 0);
					double f_L_ii = cur_L_diag_block(i - n_block_column_base, i - n_block_column_base);
					double f_L_ii_inv = 1 / f_L_ii;
					// get the diagonal element

					double f_diag_sum = 0;
					for(size_t j = 1; j < n_block_column_block_num; ++ j) { // all the blocks except the diagonal
						CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_Block_AtColumn(n_block_column, j);
						size_t n_block_row_size = block_i_j.rows();
						size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_column, j));
						// look up the row in R (= column in L)

						f_diag_sum += r_marginals.row(i).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, i, 0, n_block_row_size) or something like that?
							n_block_row_size).dot(block_i_j.col(i - n_block_column_base)); // todo - _FBS it
						// add dot of one column of the block with span of the current column of the marginals
					}
					{
						size_t n_block_row_size = n_block_column_size;//cur_L_diag_block.rows(); // also symmetric
						size_t n_block_row_base = n_block_column_base; // it is symmetric
						// look up the row in R (= column in L)

						size_t n_subblock_col = i - n_block_column_base;
						size_t n_first_underdiag_elem = n_subblock_col + 1;
						size_t n_underdiag_elem_num = n_block_column_size - n_first_underdiag_elem;
						f_diag_sum += r_marginals.row(i).segment(n_first_underdiag_elem + n_block_row_base,
							n_underdiag_elem_num).dot(cur_L_diag_block.col(n_subblock_col).tail(
							n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
						// complete the sum

						//for(size_t j = i - n_block_column_base + 1; j < n_block_column_size; ++ j)
						//	f_diag_sum += r_marginals(i, j + n_block_row_base) * cur_L_diag_block(j, i - n_block_column_base);
						// t_odo - make this a dot product on spans as well
					}
#if 0 && defined(_DEBUG)
					double f_ref_diag_sum = 0;
					for(size_t j = i + 1; j < n; ++ j) {
						size_t n_block_row_size;
						size_t n_block_row = r_R.n_Find_BlockColumn(j, n_block_row_size); // R has symmetric layout
						size_t n_block_row_base = r_R.n_BlockColumn_Base(n_block_row);
						// look up the row in R (= column in L)

						CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_GetBlock_Log(n_block_row, n_block_column);
						if(!block_i_j.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks instead of looking up all blocks in dense manner
						double R_i_j = block_i_j(j - n_block_row_base, i - n_block_column_base);
						f_ref_diag_sum += r_marginals(i, j) * R_i_j; // this is bad, accesses the matrix by rows (need transpose)
					}
					_ASSERTE(fabs(f_ref_diag_sum - f_diag_sum) < std::max(1e-5, fabs(f_ref_diag_sum)));
#endif // 0 && _DEBUG
					r_marginals(i, i) = f_L_ii_inv * (f_L_ii_inv - f_diag_sum);
					// calculate the diagonal element
				}

				for(size_t j = i; j > 0;) { // note that this is only required for the elements that are required on output or elements that are above nnz in the L or R factor
					-- j; // j is i in the book
					// i is k in the book

					size_t n_block_j_size;
					size_t n_block_j = L.n_Find_BlockColumn(j, n_block_j_size);
					size_t n_block_j_base = L.n_BlockColumn_Base(n_block_j);
					size_t n_block_j_block_num = L.n_BlockColumn_Block_Num(n_block_j);

					CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_j, 0);
					double f_L_jj = cur_L_diag_block(j - n_block_j_base, j - n_block_j_base);
					double f_L_jj_inv = 1 / f_L_jj;
					// get the diagonal block and element

					double f_diag_sum = 0;
					for(size_t k = 1; k < n_block_j_block_num; ++ k) { // all the blocks except the diagonal
						CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_Block_AtColumn(n_block_j, k);
						size_t n_block_row_size = block_j_k.rows();
						size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_j, k));
						// look up the row in R (= column in L)

						f_diag_sum += r_marginals.row(i).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, j, 0, n_block_row_size) or something like that?
							n_block_row_size).dot(block_j_k.col(j - n_block_j_base)); // todo - _FBS it
						// add dot of one column of the block with span of the current column of the marginals
					}
					{
						size_t n_block_row_size = n_block_j_size;//cur_L_diag_block.rows(); // also symmetric
						size_t n_block_row_base = n_block_j_base; // it is symmetric
						// look up the row in R (= column in L)

						size_t n_subblock_col = j - n_block_j_base;
						size_t n_first_underdiag_elem = n_subblock_col + 1;
						size_t n_underdiag_elem_num = n_block_j_size - n_first_underdiag_elem;
						f_diag_sum += r_marginals.row(i).segment(n_first_underdiag_elem + n_block_row_base,
							n_underdiag_elem_num).dot(cur_L_diag_block.col(n_subblock_col).tail(
							n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
						// complete the sum
					}

#if 0 && defined(_DEBUG)
					double f_sum_Rjk_Cik = 0;//, f_sum_part0, f_sum_first_elem;
					for(size_t k = j + 1; k < n; ++ k) {
						size_t n_block_k_size;
						size_t n_block_k = L.n_Find_BlockColumn(k, n_block_k_size);
						size_t n_block_k_base = L.n_BlockColumn_Base(n_block_k);

						CUberBlockMatrix::_TyConstMatrixXdRef block_j_k = L.t_GetBlock_Log(n_block_k, n_block_j);
						if(!block_j_k.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks

						f_sum_Rjk_Cik += r_marginals(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base);
						// product
					}
					_ASSERTE(fabs(f_sum_Rjk_Cik - f_diag_sum) < std::max(1e-5, fabs(f_sum_Rjk_Cik)));
					// use the "old" code to verify corectness
#endif // 0 && _DEBUG

					/*{ // debugging of more recurrent formula
						size_t m = i + 1; // wha?
						size_t n_block_m_size;
						size_t n_block_m = R.n_Find_BlockColumn(m, n_block_m_size);
						size_t n_block_m_base = R.n_BlockColumn_Base(n_block_m);
						CUberBlockMatrix::_TyConstMatrixXdRef diag_block_m =
							R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_m) - 1, n_block_m);
						double f_R_mm = cur_L_diag_block(m - n_block_m_base, m - n_block_m_base);
						double f_sum_part1 = -(C_dep(m, m) * f_R_mm - 1 / f_R_mm);
						double f_sum_parted = f_sum_part0 + f_sum_part1;
						double f_sum_err = fabs(f_sum_parted - f_sum_Rjk_Cik);
					}*/

					/*CUberBlockMatrix::_TyConstMatrixXdRef block_j_j =
						L.t_Block_AtColumn(n_block_j, L.n_BlockColumn_Block_Num(n_block_j) - 1);
					double L_j_j = block_j_j(j - n_block_j_base, j - n_block_j_base);
					// get another diagonal element of R*/ // unused

					r_marginals(i, j) = r_marginals(j, i) = -f_diag_sum * f_L_jj_inv; // write only upper triangular, will mirror it at once (or not at all)
					// todo - see to it that only upper triangular is accessed, add _ASSERTE() where reading it to make sure lower is not touched
				}
				// calculate C_dep_{n - 1 - i} by recurrent formula
			}
		}

		//r_marginals.triangularView<Eigen::StrictlyUpper>() =
		//	r_marginals.triangularView<Eigen::StrictlyLower>().transpose(); // mirror lower to upper
		//r_marginals.triangularView<Eigen::StrictlyLower>() =
		//	r_marginals.triangularView<Eigen::StrictlyUpper>().transpose(); // mirror upper to lower
		// transpose elements below diagonal to elements above it
	}

	/**
	 *	@brief recurrent elementwise FBS function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Calculate_DenseMarginals_Recurrent_Elemwise_FBS(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);

		CUberBlockMatrix L;
		L.TransposeOf(r_R);
		// need transpose of R

		enum {
			b_single_block_size = CTypelistLength<CBlockMatrixTypelist>::n_result == 1,
			n_first_block_size = fbs_ut::CEigenToDimension<typename
				CBlockMatrixTypelist::_TyHead>::_TyResult::n_column_num
		};
		// optimize for just a single size in the typelist (compile-time constants)

		//printf("this is the FBS version of the correct Calculate_DenseMarginals_Recurrent()\n"); // debug

		for(size_t i = n; i > 0;) {
			-- i; // here

			/*if(i == n - 1) { // the last row / col is easy, as it has no refs to the next ones
				Eigen::MatrixXd::ColXpr last_col = r_marginals.col(n - 1);
				size_t lb = r_R.n_BlockColumn_Num() - 1;
				CUberBlockMatrix::_TyMatrixXdRef last_block =
					r_R.t_Block_AtColumn(lb, r_R.n_BlockColumn_Block_Num(lb) - 1);
				last_col.setZero(); // !!
				last_col(n - 1) = 1 / last_block(last_block.rows() - 1, last_block.cols() - 1);
				r_R.UpperTriangular_Solve_FBS<CBlockMatrixTypelist>(&last_col(0), n); // all columns needed
				// calculates the whole last column of C

				r_marginals.row(n - 1).head(n - 1) = r_marginals.col(n - 1).head(n - 1).transpose();
				// copy that also to the last row, to form the full matrix and not just upper-triangular
			} else*/
			// the code above is simple, but the usolve is quite expensive

			{ // columns with references to the subsequent columns
				size_t n_block_column_size;
				size_t n_block_column = L.n_Find_BlockColumn(i, n_block_column_size);
				size_t n_block_column_base = L.n_BlockColumn_Base(n_block_column);
				size_t n_block_column_block_num = L.n_BlockColumn_Block_Num(n_block_column);
				// gets the corresponding block col (can use decrementing strategy like in C_direct)

				{
					CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_column, 0);
					double f_L_ii = cur_L_diag_block(i - n_block_column_base, i - n_block_column_base);
					double f_L_ii_inv = 1 / f_L_ii;
					// get the diagonal element

					double f_diag_sum = 0;
					if(b_single_block_size) { // compile-time constant; should get optimized away
						for(size_t j = 1; j < n_block_column_block_num; ++ j) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef _block_i_j = L.t_Block_AtColumn(n_block_column, j);
							typename CUberBlockMatrix::CMakeMatrixRef<n_first_block_size, n_first_block_size>::_Ty block_i_j(_block_i_j.data());
							size_t n_block_row_size = n_first_block_size;//block_i_j.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_column, j));
							// look up the row in R (= column in L)

							f_diag_sum += r_marginals.row(i).segment<n_first_block_size>(
								n_block_row_base).dot(block_i_j.col(i - n_block_column_base));
							// add dot of one column of the block with span of the current column of the marginals
						}
					} else {
						for(size_t j = 1; j < n_block_column_block_num; ++ j) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_Block_AtColumn(n_block_column, j);
							size_t n_block_row_size = block_i_j.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_column, j));
							// look up the row in R (= column in L)

							f_diag_sum += r_marginals.row(i).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, i, 0, n_block_row_size) or something like that?
								n_block_row_size).dot(block_i_j.col(i - n_block_column_base)); // t_odo - _FBS it
							// add dot of one column of the block with span of the current column of the marginals
						}
					}
					{
						size_t n_block_row_size = n_block_column_size;//cur_L_diag_block.rows(); // also symmetric
						size_t n_block_row_base = n_block_column_base; // it is symmetric
						// look up the row in R (= column in L)

						size_t n_subblock_col = i - n_block_column_base;
						size_t n_first_underdiag_elem = n_subblock_col + 1;
						size_t n_underdiag_elem_num = n_block_column_size - n_first_underdiag_elem;
						f_diag_sum += r_marginals.row(i).segment(n_first_underdiag_elem + n_block_row_base,
							n_underdiag_elem_num).dot(cur_L_diag_block.col(n_subblock_col).tail(
							n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
						// complete the sum

						//for(size_t j = i - n_block_column_base + 1; j < n_block_column_size; ++ j)
						//	f_diag_sum += r_marginals(i, j + n_block_row_base) * cur_L_diag_block(j, i - n_block_column_base);
						// t_odo - make this a dot product on spans as well
					}
#if 0 && defined(_DEBUG)
					double f_ref_diag_sum = 0;
					for(size_t j = i + 1; j < n; ++ j) {
						size_t n_block_row_size;
						size_t n_block_row = r_R.n_Find_BlockColumn(j, n_block_row_size); // R has symmetric layout
						size_t n_block_row_base = r_R.n_BlockColumn_Base(n_block_row);
						// look up the row in R (= column in L)

						CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_GetBlock_Log(n_block_row, n_block_column);
						if(!block_i_j.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks instead of looking up all blocks in dense manner
						double R_i_j = block_i_j(j - n_block_row_base, i - n_block_column_base);
						f_ref_diag_sum += r_marginals(i, j) * R_i_j; // this is bad, accesses the matrix by rows (need transpose)
					}
					_ASSERTE(fabs(f_ref_diag_sum - f_diag_sum) < std::max(1e-5, fabs(f_ref_diag_sum)));
#endif // 0 && _DEBUG
					r_marginals(i, i) = f_L_ii_inv * (f_L_ii_inv - f_diag_sum);
					// calculate the diagonal element
				}

				for(size_t j = i; j > 0;) { // note that this is only required for the elements that are required on output or elements that are above nnz in the L or R factor
					-- j; // j is i in the book
					// i is k in the book

					size_t n_block_j_size;
					size_t n_block_j = L.n_Find_BlockColumn(j, n_block_j_size);
					size_t n_block_j_base = L.n_BlockColumn_Base(n_block_j);
					size_t n_block_j_block_num = L.n_BlockColumn_Block_Num(n_block_j);

					CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_j, 0);
					double f_L_jj = cur_L_diag_block(j - n_block_j_base, j - n_block_j_base);
					double f_L_jj_inv = 1 / f_L_jj;
					// get the diagonal block and element

					double f_diag_sum = 0;
					if(b_single_block_size) { // compile-time constant; should get optimized away
						for(size_t k = 1; k < n_block_j_block_num; ++ k) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef _block_j_k = L.t_Block_AtColumn(n_block_j, k);
							typename CUberBlockMatrix::CMakeMatrixRef<n_first_block_size, n_first_block_size>::_Ty block_j_k(_block_j_k.data());
							size_t n_block_row_size = n_first_block_size;//block_j_k.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_j, k));
							// look up the row in R (= column in L)

							f_diag_sum += r_marginals.row(i).segment<n_first_block_size>(
								n_block_row_base).dot(block_j_k.col(j - n_block_j_base));
							// add dot of one column of the block with span of the current column of the marginals
						}
					} else {
						for(size_t k = 1; k < n_block_j_block_num; ++ k) { // all the blocks except the diagonal
							CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_Block_AtColumn(n_block_j, k);
							size_t n_block_row_size = block_j_k.rows();
							size_t n_block_row_base = L.n_BlockRow_Base(L.n_Block_Row(n_block_j, k));
							// look up the row in R (= column in L)

							f_diag_sum += r_marginals.row(i).segment(n_block_row_base, // todo - try .block<1, Dynamic>(n_block_row_base, j, 0, n_block_row_size) or something like that?
								n_block_row_size).dot(block_j_k.col(j - n_block_j_base)); // t_odo - _FBS it
							// add dot of one column of the block with span of the current column of the marginals
						}
					}
					{
						size_t n_block_row_size = n_block_j_size;//cur_L_diag_block.rows(); // also symmetric
						size_t n_block_row_base = n_block_j_base; // it is symmetric
						// look up the row in R (= column in L)

						size_t n_subblock_col = j - n_block_j_base;
						size_t n_first_underdiag_elem = n_subblock_col + 1;
						size_t n_underdiag_elem_num = n_block_j_size - n_first_underdiag_elem;
						f_diag_sum += r_marginals.row(i).segment(n_first_underdiag_elem + n_block_row_base,
							n_underdiag_elem_num).dot(cur_L_diag_block.col(n_subblock_col).tail(
							n_block_row_size - n_first_underdiag_elem)); // todo - _FBS it
						// complete the sum
					}

#if 0 && defined(_DEBUG)
					double f_sum_Rjk_Cik = 0;//, f_sum_part0, f_sum_first_elem;
					for(size_t k = j + 1; k < n; ++ k) {
						size_t n_block_k_size;
						size_t n_block_k = L.n_Find_BlockColumn(k, n_block_k_size);
						size_t n_block_k_base = L.n_BlockColumn_Base(n_block_k);

						CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_GetBlock_Log(n_block_k, n_block_j);
						if(!block_j_k.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks

						f_sum_Rjk_Cik += r_marginals(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base);
						// product
					}
					_ASSERTE(fabs(f_sum_Rjk_Cik - f_diag_sum) < std::max(1e-5, fabs(f_sum_Rjk_Cik)));
					// use the "old" code to verify corectness
#endif // 0 && _DEBUG

					/*{ // debugging of more recurrent formula
						size_t m = i + 1; // wha?
						size_t n_block_m_size;
						size_t n_block_m = R.n_Find_BlockColumn(m, n_block_m_size);
						size_t n_block_m_base = R.n_BlockColumn_Base(n_block_m);
						CUberBlockMatrix::_TyMatrixXdRef diag_block_m =
							R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_m) - 1, n_block_m);
						double f_R_mm = cur_L_diag_block(m - n_block_m_base, m - n_block_m_base);
						double f_sum_part1 = -(C_dep(m, m) * f_R_mm - 1 / f_R_mm);
						double f_sum_parted = f_sum_part0 + f_sum_part1;
						double f_sum_err = fabs(f_sum_parted - f_sum_Rjk_Cik);
					}*/

					/*CUberBlockMatrix::_TyMatrixXdRef block_j_j =
						L.t_Block_AtColumn(n_block_j, L.n_BlockColumn_Block_Num(n_block_j) - 1);
					double L_j_j = block_j_j(j - n_block_j_base, j - n_block_j_base);
					// get another diagonal element of R*/ // unused

					r_marginals(i, j) = r_marginals(j, i) = -f_diag_sum * f_L_jj_inv;
				}
				// calculate C_dep_{n - 1 - i} by recurrent formula
			}
		}
	}

	/**
	 *	@brief naive recurrent right column band function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_column_num is number of columns to calculate (in elements)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DenseMarginals_LastNCols_Recurrent(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_column_num) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);

		CUberBlockMatrix L;
		L.TransposeOf(r_R);
		// need transpose of R

		for(size_t i = n; i > 0;) {
			-- i;
			// here

			if(i == n - 1) { // the last row / col is easy, as it has no refs to the next ones
				Eigen::MatrixXd::ColXpr last_col = r_marginals.col(n - 1);
				size_t lb = r_R.n_BlockColumn_Num() - 1;
				CUberBlockMatrix::_TyConstMatrixXdRef last_block =
					r_R.t_Block_AtColumn(lb, r_R.n_BlockColumn_Block_Num(lb) - 1);
				last_col.setZero(); // !!
				last_col(n - 1) = 1 / last_block(last_block.rows() - 1, last_block.cols() - 1);
				r_R.UpperTriangular_Solve(&last_col(0), n); // all columns needed
				// calculates the whole last column of C

				r_marginals.row(n - 1).head(n - 1) = r_marginals.col(n - 1).head(n - 1).transpose();
				// copy that also to the last row, to form the full matrix and not just upper-triangular
			} else { // columns with references to the subsequent columns
				//i = n - 1 - i;
				// fill the matrix from the back

				size_t n_block_column_size;
				size_t n_block_column = L.n_Find_BlockColumn(i, n_block_column_size);
				size_t n_block_column_base = L.n_BlockColumn_Base(n_block_column);
				// gets the corresponding block col (can use decrementing strategy like in C_direct)

				CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block = L.t_Block_AtColumn(n_block_column, 0);
				double f_R_ii = cur_L_diag_block(i - n_block_column_base, i - n_block_column_base);
				double f_R_ii_inv = 1 / f_R_ii;
				// get the diagonal element

				double f_diag_sum = 0;
				for(size_t j = i + 1; j < n; ++ j) {
					size_t n_block_row_size;
					size_t n_block_row = L.n_Find_BlockColumn(j, n_block_row_size); // R has symmetric layout
					size_t n_block_row_base = L.n_BlockColumn_Base(n_block_row);
					// look up the row in R (= column in L)

					CUberBlockMatrix::_TyMatrixXdRef block_i_j = L.t_GetBlock_Log(n_block_row, n_block_column);
					if(!block_i_j.cols())
						continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks instead of looking up all blocks in dense manner
					double R_i_j = block_i_j(j - n_block_row_base, i - n_block_column_base);
					f_diag_sum += r_marginals(i, j) * R_i_j; // this is bad, accesses the matrix by rows (need transpose)
				}
				r_marginals(i, i) = f_R_ii_inv * (f_R_ii_inv - f_diag_sum);
				// calculate the diagonal element

				for(size_t j = i; j > 0;) {
					-- j; // j is i in the book
					// i is k in the book

					size_t n_block_j_size;
					size_t n_block_j = L.n_Find_BlockColumn(j, n_block_j_size);
					size_t n_block_j_base = L.n_BlockColumn_Base(n_block_j);

					double f_sum_Rjk_Cik = 0/*, f_sum_part0, f_sum_first_elem*/;
					for(size_t k = j + 1; k < n; ++ k) {
						//if(k == i + 1)
						//	f_sum_part0 = f_sum_Rjk_Cik; // note that the second half of the sum might be actually recurrent and easy to recover from the previous diagonal elements
						// less code this way

						size_t n_block_k_size;
						size_t n_block_k = L.n_Find_BlockColumn(k, n_block_k_size);
						size_t n_block_k_base = L.n_BlockColumn_Base(n_block_k);

						CUberBlockMatrix::_TyMatrixXdRef block_j_k = L.t_GetBlock_Log(n_block_k, n_block_j);
						if(!block_j_k.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks

						f_sum_Rjk_Cik += r_marginals(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base);
						// product

						//if(k == i)
						//	f_sum_first_elem = C_dep(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base); // save that as well
					}
					// note that this is a single loop, which skips iterating over element i

					/*{ // debugging of more recurrent formula
						size_t m = i + 1; // wha?
						size_t n_block_m_size;
						size_t n_block_m = R.n_Find_BlockColumn(m, n_block_m_size);
						size_t n_block_m_base = R.n_BlockColumn_Base(n_block_m);
						CUberBlockMatrix::_TyMatrixXdRef diag_block_m =
							R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_m) - 1, n_block_m);
						double f_R_mm = cur_L_diag_block(m - n_block_m_base, m - n_block_m_base);
						double f_sum_part1 = -(C_dep(m, m) * f_R_mm - 1 / f_R_mm);
						double f_sum_parted = f_sum_part0 + f_sum_part1;
						double f_sum_err = fabs(f_sum_parted - f_sum_Rjk_Cik);
					}*/

					CUberBlockMatrix::_TyMatrixXdRef block_j_j =
						L.t_Block_AtColumn(n_block_j, L.n_BlockColumn_Block_Num(n_block_j) - 1);
					double R_j_j = block_j_j(j - n_block_j_base, j - n_block_j_base);
					// get another diagonal element of R

					r_marginals(i, j) = r_marginals(j, i) = f_sum_Rjk_Cik / -R_j_j;
				}
				// calculate C_dep_{n - 1 - i} by recurrent formula

				//printf("%d, error: %g%6s\r", i, (C_dep.col(i) - C.col(i)).norm(), "");

				//i = n - 1 - i;
				// go back to i
			}
		}
	}

	/**
	 *	@brief band diagonal function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_diag_band_width is number of elements to calculate around the diagonal
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DiagonalMarginals(Eigen::MatrixXd &r_marginals, const CUberBlockMatrix &r_R,
		size_t n_diag_band_width = 3) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n);

		Eigen::MatrixXd prev_column_buffer(n, n_diag_band_width); // todo - cache align, will likely access the same elements in different vectors (needs to be implemented using a strided map)
		// previous columns

		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			Eigen::MatrixXd::ColXpr cur_row = prev_column_buffer.col(i % n_diag_band_width);
			cur_row.setZero();
			cur_row(i) = 1; // !!
			r_R.UpperTriangularTranspose_Solve(&cur_row(0), n, n_block_col); // forward substitution; can also skip to the current column
			// calculate a row of the R_inv? maybe? seems outright wrong, but actually gives a rather correct answer.

			_ASSERTE(cur_row.head(i).norm() == 0);
			// everything above i is zero (true)

			size_t n_block_org = r_R.n_BlockColumn_Base(n_block_col);
			r_marginals(i, i) = cur_row.tail(n - i).squaredNorm();
			//for(size_t j = std::max(n_diag_band_width, i) - n_diag_band_width; j < i; ++ j) { // could do two loops; one for i < n_diag_band_width and the one for the rest (low prio, almost doesn't matter)
			for(size_t j = i - n_block_org/*i % n_diag_band_width*/; j < i; ++ j) { // thin block diagonal, works even for mixed-size vertices
				Eigen::MatrixXd::ColXpr prev_row = prev_column_buffer.col(j % n_diag_band_width);
				r_marginals(j, i) = r_marginals(i, j) = prev_row.tail(n - j).dot(cur_row.tail(n - j));
			}
			// calculate banded diagonal; this works well, the banded diagonal is identical to the one in C or C_full
			// todo - store in a smaller matrix (will only contain block cross-covs of the vertices; that involves further ~50% reduction in computation)
		}
	}

	/**
	 *	@brief parallel band diagonal function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_diag_band_width is number of elements to calculate around the diagonal
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DiagonalMarginals_Parallel(Eigen::MatrixXd &r_marginals,
		const CUberBlockMatrix &r_R, size_t n_diag_band_width = 3) // throw(std::bad_alloc)
	{
		const size_t n = r_R.n_Column_Num(); // in elements
		r_marginals.resize(n, n);

#ifdef _OPENMP
		#pragma omp parallel
		{
			int n_tid = omp_get_thread_num();
			int n_thread_num = omp_get_num_threads();
			size_t n_start = n_tid * (n / n_thread_num);
			size_t n_end = (n_tid + 1 < n_thread_num)? n_start + n / n_thread_num : n;
			// split to bands to be processed in parallel

			Calculate_DiagonalMarginals(r_marginals, r_R, n_start, n_end, n_diag_band_width);
			// process in parallel
		}
#else // _OPENMP
		Calculate_DiagonalMarginals(r_marginals, r_R, n_diag_band_width);
#endif // _OPENMP
	}

	/**
	 *	@brief helper to the parallel band diagonal function that calculates dense marginals matrix
	 *
	 *	@param[out] r_marginals is filled with the marginals matrix
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_start_column is zero-based index of the first column (in elements)
	 *	@param[in] n_end_column is zero-based index of one past the last column (in elements)
	 *	@param[in] n_diag_band_width is number of elements to calculate around the diagonal
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_DiagonalMarginals(Eigen::MatrixXd &r_marginals, const CUberBlockMatrix &r_R,
		size_t n_start_column, size_t n_end_column, size_t n_diag_band_width = 3) // throw(std::bad_alloc)
	{
		_ASSERTE(n_start_column <= n_end_column); // should be a valid range

		const size_t n = r_R.n_Column_Num(); // in elements

		r_marginals.resize(n, n); // might work in a group, but then it is already allocated and it is a no-op

		Eigen::MatrixXd prev_column_buffer(n, n_diag_band_width); // todo - cache align, will likely access the same elements in different vectors (needs to be implemented using a strided map)
		// previous columns

		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < std::min(n, n_end_column); ++ i) {
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = r_R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			if(i + n_diag_band_width < n_start_column)
				continue;
			// todo - do a proper binary search for the column

			Eigen::MatrixXd::ColXpr cur_row = prev_column_buffer.col(i % n_diag_band_width);
			cur_row.setZero();
			cur_row(i) = 1; // !!
			r_R.UpperTriangularTranspose_Solve(&cur_row(0), n, n_block_col); // forward substitution; can also skip to the current column
			// calculate a row of the R_inv? maybe? seems outright wrong, but actually gives a rather correct answer.

			_ASSERTE(cur_row.head(i).norm() == 0);
			// everything above i is zero (true)

			size_t n_block_org = r_R.n_BlockColumn_Base(n_block_col);
			r_marginals(i, i) = cur_row.tail(n - i).squaredNorm();
			//for(size_t j = std::max(n_diag_band_width, i) - n_diag_band_width; j < i; ++ j) { // could do two loops; one for i < n_diag_band_width and the one for the rest (low prio, almost doesn't matter)
			for(size_t j = i - n_block_org/*i % n_diag_band_width*/; j < i; ++ j) { // thin block diagonal, works even for mixed-size vertices
				Eigen::MatrixXd::ColXpr prev_row = prev_column_buffer.col(j % n_diag_band_width);
				r_marginals(j, i) = r_marginals(i, j) = prev_row.tail(n - j).dot(cur_row.tail(n - j));
			}
			// calculate banded diagonal; this works well, the banded diagonal is identical to the one in C or C_full
			// todo - store in a smaller matrix (will only contain block cross-covs of the vertices; that involves further ~50% reduction in computation)
		}
	}

	/**
	 *	@brief benchmark of marginal calculation functions
	 *
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_diag_band_width is band width for diagonal and colmn band functions
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Marginals_Test(const CUberBlockMatrix &r_R, size_t n_diag_band_width = 3) // throw(std::bad_alloc)
	{
		if(r_R.n_Column_Num() < n_diag_band_width) {
			fprintf(stderr, "error: matrix too small for banded tests (%d x %d): skipping\n",
				int(r_R.n_Row_Num()), int(r_R.n_Column_Num()));
			return;
		}
		// might not be handled correctly

		Eigen::MatrixXd C_ref, C_slow, C_fast, C_rec, C_diag,
			C_diag_para, C_rband_slow, C_rband_fast, C_rband_rec;

		CTimer t;
		double f_time_ref = 0, f_time_slow = 0, f_time_rec = 0,
			f_time_diag = 0, f_time_diag_para = 0, f_time_rbslow = 0,
			f_time_rbfast = 0, f_time_rbrec = 0, f_time_fast = 0, f_time_transpose = 0;
		size_t n_ref_pass_num = 0, n_slow_pass_num = 0, n_rec_pass_num = 0,
			n_diag_pass_num = 0, n_diag_para_pass_num = 0, n_rbslow_pass_num = 0,
			n_rbfast_pass_num = 0, n_rbrec_pass_num = 0, n_fast_pass_num = 0, n_transpose_pass_num = 0;

		for(;;) {
			double f_start = t.f_Time();
			{
				CUberBlockMatrix L;
				L.TransposeOf(r_R);
			}
			++ n_transpose_pass_num;
			f_time_transpose += t.f_Time() - f_start;
			if((f_time_transpose >= 1 && n_transpose_pass_num >= 10) || f_time_transpose > 4)
				break;
		}
		f_time_transpose /= n_transpose_pass_num;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Ref(C_ref, r_R);
			++ n_ref_pass_num;
			f_time_ref += t.f_Time() - f_start;
			if((f_time_ref >= 1 && n_ref_pass_num >= 10) || f_time_ref > 4)
				break;
		}
		f_time_ref /= n_ref_pass_num;

		/*for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Slow(C_slow, r_R);
			++ n_slow_pass_num;
			f_time_slow += t.f_Time() - f_start;
			if((f_time_slow >= 1 && n_slow_pass_num >= 10) || f_time_slow > 4)
				break;
		}
		f_time_slow /= n_slow_pass_num;*/

		std::vector<size_t> fake_perm(r_R.n_BlockColumn_Num());
		for(size_t i = 0, n = fake_perm.size(); i < n; ++ i)
			fake_perm[i] = i;
		// build a fake permutation on R (we don't have the original one here)

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Fast(C_fast, r_R, &fake_perm[0], fake_perm.size());
			++ n_fast_pass_num;
			f_time_fast += t.f_Time() - f_start;
			if((f_time_fast >= 1 && n_fast_pass_num >= 10) || f_time_fast > 4)
				break;
		}
		f_time_fast /= n_fast_pass_num;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Recurrent(C_rec, r_R);
			//Calculate_DenseMarginals_Recurrent_Devel(C_rec, C_ref, r_R);
			++ n_rec_pass_num;
			f_time_rec += t.f_Time() - f_start;
			if((f_time_rec >= 1 && n_rec_pass_num >= 10) || f_time_rec > 4)
				break;
		}
		f_time_rec /= n_rec_pass_num;

		/*for(;;) {
			double f_start = t.f_Time();
			Calculate_DiagonalMarginals(C_diag, r_R, n_diag_band_width);
			++ n_diag_pass_num;
			f_time_diag += t.f_Time() - f_start;
			if((f_time_diag >= 1 && n_diag_pass_num >= 10) || f_time_diag > 4)
				break;
		}
		f_time_diag /= n_diag_pass_num;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DiagonalMarginals_Parallel(C_diag_para, r_R, n_diag_band_width);
			++ n_diag_para_pass_num;
			f_time_diag_para += t.f_Time() - f_start;
			if((f_time_diag_para >= 1 && n_diag_para_pass_num >= 10) || f_time_diag_para > 4)
				break;
		}
		f_time_diag_para /= n_diag_para_pass_num;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_LastNColumns_Slow(C_rband_slow, r_R, n_diag_band_width);
			++ n_rbslow_pass_num;
			f_time_rbslow += t.f_Time() - f_start;
			if((f_time_rbslow >= 1 && n_rbslow_pass_num >= 10) || f_time_rbslow > 4)
				break;
		}
		f_time_rbslow /= n_rbslow_pass_num;*/

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_LastNColumns_Fast(C_rband_fast, r_R, n_diag_band_width,
				&fake_perm[0], fake_perm.size());
			++ n_rbfast_pass_num;
			f_time_rbfast += t.f_Time() - f_start;
			if((f_time_rbfast >= 1 && n_rbfast_pass_num >= 10) || f_time_rbfast > 4)
				break;
		}
		f_time_rbfast /= n_rbfast_pass_num;

		/*for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_LastNCols_Recurrent(C_rband_rec, r_R, n_diag_band_width);
			++ n_rbrec_pass_num;
			f_time_rbrec += t.f_Time() - f_start;
			if((f_time_rbrec >= 1 && n_rbrec_pass_num >= 10) || f_time_rbrec > 4)
				break;
		}
		f_time_rbrec /= n_rbrec_pass_num;*/

		printf("%6s took %.3f msec\n", "ref", f_time_ref * 1000);
		//printf("%6s took %.3f msec\n", "slow", f_time_slow * 1000);
		printf("%6s took %.3f msec\n", "fast", f_time_fast * 1000);
		printf("%6s took %.3f msec\n", "rec", f_time_rec * 1000);
		printf("%6s took %.3f msec\n", "\ttranspose in rec", f_time_transpose * 1000);
		/*printf("%6s took %.3f msec\n", "diag", f_time_diag * 1000);
		printf("%6s took %.3f msec\n", "diag-p", f_time_diag_para * 1000);
		printf("%6s took %.3f msec\n", "rbslow", f_time_rbslow * 1000);
		printf("%6s took %.3f msec\n", "rbfast", f_time_rbfast * 1000);
		printf("%6s took %.3f msec\n", "rbrec", f_time_rbrec * 1000);*/
		// print times

		/*printf("norm of C_slow - C_ref is %g\n", (C_slow - C_ref).norm());*/
		printf("norm of C_fast - C_ref is %g\n", (C_fast - C_ref).norm());
		printf("norm of C_rec - C_ref is %g\n", (C_rec - C_ref).norm());
		/*printf("norm of the diagonal of C_diag - C_ref is %g\n", (C_diag.diagonal() - C_ref.diagonal()).norm());
		printf("norm of the diagonal of C_diag_para - C_ref is %g\n", (C_diag_para.diagonal() - C_ref.diagonal()).norm());
		printf("norm of the last N columns of C_rband_slow - C_ref is %g\n",
			(C_rband_slow.rightCols(n_diag_band_width) - C_ref.rightCols(n_diag_band_width)).norm());*/
		printf("norm of the last N columns of C_rband_fast - C_ref is %g\n",
			(C_rband_fast.rightCols(n_diag_band_width) - C_ref.rightCols(n_diag_band_width)).norm());
		/*printf("norm of the last N columns of C_rband_rec - C_ref is %g\n",
			(C_rband_rec.rightCols(n_diag_band_width) - C_ref.rightCols(n_diag_band_width)).norm());*/
		// print precision
	}

	/**
	 *	@brief benchmark of marginal FBS calculation functions
	 *
	 *	@tparam CMatrixBlockSizeList is a list of possible matrix block sizes
	 *
	 *	@param[in] r_R is the Cholesky factor
	 *	@param[in] n_diag_band_width is band width for diagonal and colmn band functions
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CBlockMatrixTypelist>
	static void Marginals_Test_FBS(const CUberBlockMatrix &r_R, size_t n_diag_band_width = 3) // throw(std::bad_alloc)
	{
		if(r_R.n_Column_Num() < n_diag_band_width) {
			fprintf(stderr, "error: matrix too small for banded tests (%d x %d): skipping\n",
				int(r_R.n_Row_Num()), int(r_R.n_Column_Num()));
			return;
		}
		// might not be handled correctly

		Eigen::MatrixXd C_ref, C_slow, C_fast, C_diag,
			C_diag_para, C_rband_slow, C_rband_fast, C_rband_rec;
		CUberBlockMatrix C_rec;

		CTimer t;
		double f_time_ref = 0, f_time_slow = 0, f_time_rec = 0,
			f_time_diag = 0, f_time_diag_para = 0, f_time_rbslow = 0,
			f_time_rbfast = 0, f_time_rbrec = 0, f_time_fast = 0;
		size_t n_ref_pass_num = 0, n_slow_pass_num = 0, n_rec_pass_num = 0,
			n_diag_pass_num = 0, n_diag_para_pass_num = 0, n_rbslow_pass_num = 0,
			n_rbfast_pass_num = 0, n_rbrec_pass_num = 0, n_fast_pass_num = 0;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Ref(C_ref, r_R);
			++ n_ref_pass_num;
			f_time_ref += t.f_Time() - f_start;
			if((f_time_ref >= 1 && n_ref_pass_num >= 10) || f_time_ref > 4)
				break;
		}
		f_time_ref /= n_ref_pass_num;

		/*for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Slow(C_slow, r_R);
			++ n_slow_pass_num;
			f_time_slow += t.f_Time() - f_start;
			if((f_time_slow >= 1 && n_slow_pass_num >= 10) || f_time_slow > 4)
				break;
		}
		f_time_slow /= n_slow_pass_num;*/

		std::vector<size_t> fake_perm(r_R.n_BlockColumn_Num());
		for(size_t i = 0, n = fake_perm.size(); i < n; ++ i)
			fake_perm[i] = i;
		// build a fake permutation on R (we don't have the original one here)

		CMatrixOrdering mord;
		mord.p_InvertOrdering(&fake_perm.front(), fake_perm.size());

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Fast_FBS<CBlockMatrixTypelist>(C_fast, r_R, &fake_perm[0], fake_perm.size());
			++ n_fast_pass_num;
			f_time_fast += t.f_Time() - f_start;
			if((f_time_fast >= 1 && n_fast_pass_num >= 10) || f_time_fast > 4)
				break;
		}
		f_time_fast /= n_fast_pass_num;

		for(;;) {
/*#ifdef _DEBUG
			C_rec = C_ref; // insert ground truth
#endif // _DEBUG*/
			double f_start = t.f_Time();
			Calculate_DenseMarginals_Recurrent_FBS<CBlockMatrixTypelist>(C_rec, r_R, mord, mpart_Diagonal);
			++ n_rec_pass_num;
			f_time_rec += t.f_Time() - f_start;
			if((f_time_rec >= 1 && n_rec_pass_num >= 10) || f_time_rec > 4)
				break;
		}
		f_time_rec /= n_rec_pass_num;

		/*for(;;) {
			double f_start = t.f_Time();
			Calculate_DiagonalMarginals(C_diag, r_R, n_diag_band_width);
			++ n_diag_pass_num;
			f_time_diag += t.f_Time() - f_start;
			if((f_time_diag >= 1 && n_diag_pass_num >= 10) || f_time_diag > 4)
				break;
		}
		f_time_diag /= n_diag_pass_num;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DiagonalMarginals_Parallel(C_diag_para, r_R, n_diag_band_width);
			++ n_diag_para_pass_num;
			f_time_diag_para += t.f_Time() - f_start;
			if((f_time_diag_para >= 1 && n_diag_para_pass_num >= 10) || f_time_diag_para > 4)
				break;
		}
		f_time_diag_para /= n_diag_para_pass_num;

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_LastNColumns_Slow(C_rband_slow, r_R, n_diag_band_width);
			++ n_rbslow_pass_num;
			f_time_rbslow += t.f_Time() - f_start;
			if((f_time_rbslow >= 1 && n_rbslow_pass_num >= 10) || f_time_rbslow > 4)
				break;
		}
		f_time_rbslow /= n_rbslow_pass_num;*/

		for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_LastNColumns_Fast_FBS<CBlockMatrixTypelist>(C_rband_fast, r_R, n_diag_band_width,
				&fake_perm[0], fake_perm.size());
			++ n_rbfast_pass_num;
			f_time_rbfast += t.f_Time() - f_start;
			if((f_time_rbfast >= 1 && n_rbfast_pass_num >= 10) || f_time_rbfast > 4)
				break;
		}
		f_time_rbfast /= n_rbfast_pass_num;

		/*for(;;) {
			double f_start = t.f_Time();
			Calculate_DenseMarginals_LastNCols_Recurrent(C_rband_rec, r_R, n_diag_band_width);
			++ n_rbrec_pass_num;
			f_time_rbrec += t.f_Time() - f_start;
			if((f_time_rbrec >= 1 && n_rbrec_pass_num >= 10) || f_time_rbrec > 4)
				break;
		}
		f_time_rbrec /= n_rbrec_pass_num;*/

		printf("%10s took %.3f msec\n", "ref", f_time_ref * 1000);
		printf("%10s took %.3f msec (calculated " PRIsize " nnz)\n", "rec_FBS",
			f_time_rec * 1000, C_rec.n_NonZero_Num());
		printf("%10s took %.3f msec\n", "fast_FBS", f_time_fast * 1000);
		printf("%10s took %.3f msec\n", "rbfast_FBS", f_time_rbfast * 1000);
		// print times

		/*printf("norm of C_slow - C_ref is %g\n", (C_slow - C_ref).norm());*/
		printf("norm of C_fast_FBS - C_ref is %g\n", (C_fast - C_ref).norm());
		printf("norm of C_rec_FBS - C_ref is %g\n", f_IncompleteDifference(C_ref, C_rec));
		//printf("norm of the diagonal of C_rec_FBS - C_ref is %g\n", (C_rec.diagonal() - C_ref.diagonal()).norm());
		/*printf("norm of the diagonal of C_diag - C_ref is %g\n", (C_diag.diagonal() - C_ref.diagonal()).norm());
		printf("norm of the diagonal of C_diag_para - C_ref is %g\n", (C_diag_para.diagonal() - C_ref.diagonal()).norm());
		printf("norm of the last N columns of C_rband_slow - C_ref is %g\n",
			(C_rband_slow.rightCols(n_diag_band_width) - C_ref.rightCols(n_diag_band_width)).norm());*/
		printf("norm of the last N columns of C_rband_fast_FBS - C_ref is %g\n",
			(C_rband_fast.rightCols(n_diag_band_width) - C_ref.rightCols(n_diag_band_width)).norm());
		/*printf("norm of the last N columns of C_rband_rec - C_ref is %g\n",
			(C_rband_rec.rightCols(n_diag_band_width) - C_ref.rightCols(n_diag_band_width)).norm());*/
		// print precision
	}

	/**
	 *	@brief calculates norm of difference between dense ground truth and sparse (incomplete) solution
	 *
	 *	@param[in] r_matrix is full matrix containing the marginals
	 *	@param[in] r_incomplete is partially calculated sparse block matrix with the same marginals
	 *
	 *	@return Returns norm of differences of the two matrices, ignoting the parts of the
	 *		dense matrix where the sparse block matrix doesn't have values.
	 */
	static double f_IncompleteDifference(const Eigen::MatrixXd &r_matrix, const CUberBlockMatrix &r_incomplete)
	{
		_ASSERTE(r_matrix.rows() == r_incomplete.n_Row_Num() &&
			r_matrix.cols() == r_incomplete.n_Column_Num());
		// should be same size

		double f_error = 0;
		for(size_t i = 0, n = r_incomplete.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_column_base = r_incomplete.n_BlockColumn_Base(i);
			for(size_t j = 0, m = r_incomplete.n_BlockColumn_Block_Num(i); j < m; ++ j) {
				size_t n_block_row = r_incomplete.n_Block_Row(i, j);
				size_t n_row_base = r_incomplete.n_BlockRow_Base(n_block_row);
				CUberBlockMatrix::_TyConstMatrixXdRef t_block = r_incomplete.t_Block_AtColumn(i, j);
				f_error += (r_matrix.block(n_row_base, n_column_base,
					t_block.rows(), t_block.cols()) - t_block).squaredNorm();
			}
		}
		// sum up error under blocks that were evaluated

		return sqrt(f_error);
	}

	/**
	 *	@brief calculates norm of difference between two sparse (incomplete) solutions
	 *
	 *	@param[in] r_incomplete_a is partially calculated sparse block matrix containing the marginals
	 *	@param[in] r_incomplete_b is partially calculated sparse block matrix with the same marginals
	 *
	 *	@return Returns norm of differences of the two matrices, ignoting the parts of the
	 *		dense matrix where the sparse block matrix doesn't have values.
	 */
	static double f_IncompleteDifference(size_t &r_n_common_block_num,
		const CUberBlockMatrix &r_incomplete_a, const CUberBlockMatrix &r_incomplete_b)
	{
		_ASSERTE(r_incomplete_a.n_Row_Num() == r_incomplete_b.n_Row_Num() &&
			r_incomplete_a.n_Column_Num() == r_incomplete_b.n_Column_Num());
		_ASSERTE(r_incomplete_a.b_EqualLayout(r_incomplete_b));
		// should be same size and the same layout

		r_n_common_block_num = 0;

		double f_error = 0;
		for(size_t i = 0, n = r_incomplete_b.n_BlockColumn_Num(); i < n; ++ i) {
			for(size_t j = 0, m = r_incomplete_a.n_BlockColumn_Block_Num(i),
			   k = 0, o = r_incomplete_b.n_BlockColumn_Block_Num(i); j < m && k < o;) {
				size_t n_block_row_j = r_incomplete_a.n_Block_Row(i, j),
					n_block_row_k = r_incomplete_b.n_Block_Row(i, k);

				if(n_block_row_j == n_block_row_k) {
					CUberBlockMatrix::_TyConstMatrixXdRef t_block_j = r_incomplete_a.t_Block_AtColumn(i, j),
						t_block_k = r_incomplete_b.t_Block_AtColumn(i, k);
					f_error += (t_block_j - t_block_k).squaredNorm();
					++ r_n_common_block_num;

					++ j;
					++ k;
				} else if(n_block_row_j < n_block_row_k)
					++ j;
				else
					++ k;
			}
		}
		// sum up error under blocks that were evaluated

		return sqrt(f_error);
	}

#if 0
	void Marginals_DevelCode()
	{
		const CUberBlockMatrix &lambda = solver.r_Lambda();
		// get system matrix

		cs *p_lam = lambda.p_Convert_to_Sparse();
		FILE *p_fw = fopen("lambda.txt", "w");
		CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_lam, "lambda = ", "' % this is supposed to be upper-tri\n");
		fclose(p_fw);
		cs_spfree(p_lam);
		// dump the optimized lambda

		CUberBlockMatrix R;
		R.CholeskyOf(lambda);
		// take Cholesky

		const size_t n = R.n_Column_Num(); // in elements

		Eigen::MatrixXd R_inv(n, n); // R_inv = S
		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			double *p_column = &R_inv.col(i)(0);
			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = R.n_BlockColumn_Column_Num(++ n_block_col);
			R.UpperTriangular_Solve(p_column, n, n_block_col); // backsub, only the nonzero part of the column (started at (block) column which contains column i, with no loss of generality)
		}
		Eigen::MatrixXd C = R_inv.lazyProduct(R_inv.transpose()); // C = SS^T
		// calculate the covariance (assume that this is correct)

		Eigen::MatrixXd C_full(n, n), C_direct(n, n), C_dep(n, n);
		C_full.setZero();
		C_direct.setZero();
		C_dep.setZero();
		// only the diagonal of C

		size_t n_diag_band_width = 3; // = max column width occuring in the matrix (max vertex dim)
		// size of the diagonal (also the size of the buffer for the past rows of inv(R))

		Eigen::MatrixXd prev_column_buffer(n, n_diag_band_width); // todo - cache align, will likely access the same elements in different vectors
		// previous columns

		CUberBlockMatrix R_tr;
		R.TransposeTo(R_tr);
		// need transpose of R

		//Eigen::MatrixXd R_inv(n, n); // R_inv = S
		for(size_t i = 0, n_block_col = -1, n_col_remains = 1; i < n; ++ i) {
			double *p_column = &R_inv.col(i)(0);
			// get dense column data from the Eigen matrix (should be packed)

			memset(p_column, 0, n * sizeof(double));
			p_column[i] = 1;
			// make a column vector with a single 1 in it

			if(!(-- n_col_remains)) // triggers in the first iteration, loads up column width
				n_col_remains = R.n_BlockColumn_Column_Num(++ n_block_col);
			// if done before, it avoids both referencing block col 0 in an empty matrix
			// and determining no. of columns in one past the last block column at the end

			size_t UNUSED(n_block_column_size);
			size_t n_block_column = R.n_Find_BlockColumn(i, n_block_column_size);
			_ASSERTE(n_block_col == n_block_column); // should be the same
			// get which block column contains column i (optimize this away, probably need to use it when resuming)

			_ASSERTE(n_block_column_size <= n_diag_band_width); // make this into a run-time check in the production code
			// make sure it is not longer than the diagonal (otherwise we will not have enough backlog to calculate all the off-diagonal elements)

			R.UpperTriangular_Solve(p_column, n, n_block_col); // backsub, only the nonzero part of the column (started at (block) column which contains column i, with no loss of generality)
			// this seems to be O(i) divisions + O(nnz) MADs in the given (block) column range
			// that sums up to O(n^2/2) divisions + O(nnz log(nnz))?? MADs ... some quadratute anyways

			std::vector<double> backsub_test(n, 0);
			backsub_test[i] = 1; // !!
			R.UpperTriangular_Solve(&backsub_test[0], n); // full backsub
			_ASSERTE(!memcmp(p_column, &backsub_test[0], n * sizeof(double)));
			// make sure that the result is correct

			_ASSERTE((Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(p_column + i + 1, n - i - 1).norm() == 0)); // double pars required because of the comma in Map params
			// everything below i is zero (true)

			for(size_t k = 0; k <= i; ++ k) {
				for(size_t j = 0; j <= i; ++ j)
					C_full(j, k) += p_column[j] * p_column[k]; // it is symmetric, indexing arbitrary
			}
			// accumulate the entries of the covariace matrix. this is O(n^3/2) MADs for the full matrix
			// note that to calculate even only the diagonal, we need full columns

			Eigen::MatrixXd::ColXpr cur_row = prev_column_buffer.col(i % n_diag_band_width);
			cur_row.setZero();
			cur_row(i) = 1; // !!
			R.UpperTriangularTranspose_Solve(&cur_row(0), n, n_block_col); // forward substitution; can also skip to the current column
			// calculate a row of the R_inv? maybe? seems outright wrong, but actually gives a rather correct answer.

			_ASSERTE(cur_row.head(i).norm() == 0);
			// everything above i is zero (true)

			size_t n_block_org = R.n_BlockColumn_Base(n_block_col);
			C_direct(i, i) = cur_row.tail(n - i).squaredNorm();
			//for(size_t j = std::max(n_diag_band_width, i) - n_diag_band_width; j < i; ++ j) { // could do two loops; one for i < n_diag_band_width and the one for the rest (low prio, almost doesn't matter)
			for(size_t j = i - n_block_org/*i % n_diag_band_width*/; j < i; ++ j) { // thin block diagonal, works even for mixed-size vertices
				Eigen::MatrixXd::ColXpr prev_row = prev_column_buffer.col(j % n_diag_band_width);
				C_direct(j, i) = C_direct(i, j) = prev_row.tail(n - j).dot(cur_row.tail(n - j));
			}
			// calculate banded diagonal; this works well, the banded diagonal is identical to the one in C or C_full
			// todo - store in a smaller matrix (will only contain block cross-covs of the vertices; that involves further ~50% reduction in computation)

			//C_direct(i, i) = 0;
			//for(size_t j = i; j < n; ++ j)
			//	C_direct(i, i) += p_column[j] * p_column[j]; // O(n^2/2) for the diagonal of the full matrix
			// the diagonal is just the stuff squared. to calculate off-diagonals, we need to cache previous columns as well
			// also the columns are independent and there is no communication involved; this is easily parallelised, even on GPU

			if(!i) { // the last row / col is easy, as it has no refs to the next ones
				Eigen::MatrixXd::ColXpr last_col = C_dep.col(n - 1);
				size_t lb = R.n_BlockColumn_Num() - 1;
				CUberBlockMatrix::_TyMatrixXdRef last_block =
					R.t_BlockAt(R.n_BlockColumn_Block_Num(lb) - 1, lb);
				last_col(n - 1) = 1 / last_block(last_block.rows() - 1, last_block.cols() - 1);
				R.UpperTriangular_Solve(&last_col(0), n); // all columns needed
				// calculates the whole last column of C

				C_dep.row(n - 1).head(n - 1) = C_dep.col(n - 1).head(n - 1).transpose();
				// copy that also to the last row, to form the full matrix and not just upper-triangular
			} else { // columns with references to the subsequent columns
				i = n - 1 - i;
				// fill the matrix from the back

				size_t n_block_column_size;
				size_t n_block_column = R.n_Find_BlockColumn(i, n_block_column_size);
				size_t n_block_column_base = R.n_BlockColumn_Base(n_block_column);
				// gets the corresponding block col (can use decrementing strategy like in C_direct)

				CUberBlockMatrix::_TyMatrixXdRef cur_L_diag_block =
					R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_column) - 1, n_block_column);
				double f_R_ii = cur_L_diag_block(i - n_block_column_base, i - n_block_column_base);
				double f_R_ii_inv = 1 / f_R_ii;
				// get the diagonal element

				double f_diag_sum = 0;
				for(size_t j = i + 1; j < n; ++ j) {
					size_t n_block_row_size;
					size_t n_block_row = R.n_Find_BlockColumn(j, n_block_row_size); // R has symmetric layout
					size_t n_block_row_base = R.n_BlockColumn_Base(n_block_row);
					// look up the row in R (= column in R_tr)

					CUberBlockMatrix::_TyMatrixXdRef block_i_j = R_tr.t_GetBlock_Log(n_block_row, n_block_column);
					if(!block_i_j.cols())
						continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks instead of looking up all blocks in dense manner
					double R_i_j = block_i_j(j - n_block_row_base, i - n_block_column_base);
					f_diag_sum += C_dep(i, j) * R_i_j; // this is bad, accesses the matrix by rows (need transpose)
				}
				C_dep(i, i) = f_R_ii_inv * (f_R_ii_inv - f_diag_sum);
				for(size_t j = i; j > 0;) {
					-- j; // j is i in the book
					// i is k in the book

					size_t n_block_j_size;
					size_t n_block_j = R.n_Find_BlockColumn(j, n_block_j_size);
					size_t n_block_j_base = R.n_BlockColumn_Base(n_block_j);

					double f_sum_Rjk_Cik = 0/*, f_sum_part0, f_sum_first_elem*/;
					for(size_t k = j + 1; k < n; ++ k) {
						//if(k == i + 1)
						//	f_sum_part0 = f_sum_Rjk_Cik; // note that the second half of the sum might be actually recurrent and easy to recover from the previous diagonal elements
						// less code this way

						size_t n_block_k_size;
						size_t n_block_k = R.n_Find_BlockColumn(k, n_block_k_size);
						size_t n_block_k_base = R.n_BlockColumn_Base(n_block_k);

						CUberBlockMatrix::_TyMatrixXdRef block_j_k = R_tr.t_GetBlock_Log(n_block_k, n_block_j);
						if(!block_j_k.cols())
							continue; // no such block (zero, sparse) // todo - rewrite to form of a loop over *existing* blocks

						f_sum_Rjk_Cik += C_dep(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base);
						// product

						//if(k == i)
						//	f_sum_first_elem = C_dep(i, k) * block_j_k(k - n_block_k_base, j - n_block_j_base); // save that as well
					}
					// note that this is a single loop, which skips iterating over element i

					/*{ // debugging of more recurrent formula
						size_t m = i + 1; // wha?
						size_t n_block_m_size;
						size_t n_block_m = R.n_Find_BlockColumn(m, n_block_m_size);
						size_t n_block_m_base = R.n_BlockColumn_Base(n_block_m);
						CUberBlockMatrix::_TyMatrixXdRef diag_block_m =
							R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_m) - 1, n_block_m);
						double f_R_mm = cur_L_diag_block(m - n_block_m_base, m - n_block_m_base);
						double f_sum_part1 = -(C_dep(m, m) * f_R_mm - 1 / f_R_mm);
						double f_sum_parted = f_sum_part0 + f_sum_part1;
						double f_sum_err = fabs(f_sum_parted - f_sum_Rjk_Cik);
					}*/

					CUberBlockMatrix::_TyMatrixXdRef block_j_j =
						R.t_BlockAt(R.n_BlockColumn_Block_Num(n_block_j) - 1, n_block_j);
					double R_j_j = block_j_j(j - n_block_j_base, j - n_block_j_base);
					// get another diagonal element of R

					C_dep(i, j) = C_dep(j, i) = f_sum_Rjk_Cik / -R_j_j;
				}
				// calculate C_dep_{n - 1 - i} by recurrent formula

				printf("%d, error: %g%6s\r", i, (C_dep.col(i) - C.col(i)).norm(), "");

				i = n - 1 - i;
				// go back to i
			}
		}
		// calculate inverse of R using backsubstitution

		printf("norm of C - C_full is %g\n", (C_full - C).norm());
		printf("norm of the diagonal of C - C_direct is %g\n", (C_direct.diagonal() - C.diagonal()).norm());
		printf("norm of C - C_dep is %g\n", (C_dep - C).norm());
		printf("norm of the last column of C - C_dep is %g\n", (C_dep.col(n - 1) - C.col(n - 1)).norm());
		printf("norm of the diagonal of C - C_dep is %g\n", (C_dep.diagonal() - C.diagonal()).norm());

		p_fw = fopen("R_inv.txt", "w");
		CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, R_inv, "R_inv = ", "\n");
		fclose(p_fw);
		p_fw = fopen("C.txt", "w");
		CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, C, "C = ", "\n");
		fclose(p_fw);
		p_fw = fopen("C_banded.txt", "w");
		CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, C_direct, "C_banded = ", "\n");
		fclose(p_fw);
		// save for comparison with matlab formulas
	}
#endif // 0

	/**
	 *	@brief the fixed-block-size kernel for incremental update of sparse block marginals
	 */
	class CMarginalsUpdate_FBSKernel {
	public:
		/**
		 *	@brief sums vertex dimensions in a binary edge (affects the size of the omega matrix)
		 *	@tparam CEdgeType is an edge type name
		 */
		template <class CEdgeType>
		class CEdgeTypeToSumOfVertexDims {
		public:
			/**
			 *	@brief result stored in an enum
			 */
			enum {
				n_edge_rank = CEdgeType::n_vertex_num, // handles edges of rank 1 and 2, higher ranks result in compile-time error
				n_result = CEdgeType::template CVertexTraits<0>::n_dimension + ((n_edge_rank == 1)? 0 : 1) *
					CEdgeType::template CVertexTraits<(n_edge_rank == 2)? 1 : (n_edge_rank == 1)? 0 : -1>::n_dimension /**< @brief sum of vertex dimensions */
				// rank 1 counts vertex 0 twice and shorts it by multiplying by zero
				// higher ranks cause compile-time error by accessing vertex -1
			};

			typedef fbs_ut::CCTSize<n_result> _TyResult; /**< @brief sum of vertex dimensions as CCTSize specialization */
		};

		/**
		 *	@brief strips a pointer or a reference from a type
		 *	@tparam CClass is input type
		 */
		template <class CClass>
		class CExtractTypename {
		public:
			typedef CClass _TyResult; /**< @brief resulting type with a single pointer / reference stripped */
		};

		/**
		 *	@brief strips a pointer or a reference from a type (specialization for pointer types)
		 *	@tparam CClass is input type
		 */
		template <class CClass>
		class CExtractTypename<CClass*> {
		public:
			typedef CClass _TyResult; /**< @brief resulting type with a single pointer / reference stripped */
		};

		/**
		 *	@brief strips a pointer or a reference from a type (specialization for reference types)
		 *	@tparam CClass is input type
		 */
		template <class CClass>
		class CExtractTypename<CClass&> {
		public:
			typedef CClass _TyResult; /**< @brief resulting type with a single pointer / reference stripped */
		};

		/**
		 *	@brief outer loop context
		 *	@tparam CTimerSamplerRefType is type of timer sampler (CVoidTimerSampler or CTimerSampler), or a reference to it
		 */
		template <class CTimerSamplerRefType> // timing or not timing
		struct TOuterContext {
			typedef typename CExtractTypename<CTimerSamplerRefType>::_TyResult _TyTimerSampler; /**< @brief type of timer sampler (CVoidTimerSampler or CTimerSampler) */

			bool &r_b_result; /**< @brief reference to calculation result flag; cleared upon numerical failure */
			CTimerSamplerRefType r_t_timer_sampler; /**< @brief timer sampler for profiling */ // not a reference, that type is either a reference or void

			const CUberBlockMatrix &r_omega_slim; /**< @brief omega matrix permuted to have no empty rows / columns */
			const std::vector<size_t> &r_required_column_list; /**< @brief the list of columns of Sigma required for the update */
			// intermediate

			CUberBlockMatrix &r_prev_marginals; /**< @brief previous marginals, will be updated inplace to current marginals */
			const CUberBlockMatrix &r_lambda_in_natural_order; /**< @brief reference to the lambda matrix (must be in natural order) */
			const CUberBlockMatrix &r_R; /**< @brief reference to ordered Cholesky factorization of r_lambda_in_natural_order */
			const CMatrixOrdering &r_mord; /**< @brief reference to the ordering, used in the Cholesky factorization of r_R */
			bool b_update_diag_only; /**< @brief diagonal update flag (if set, only the diagonal is updated) */
			// parameters

			/**
			 *	@brief default constructor
			 *
			 *	@param[in] _r_b_result is reference to calculation result flag; cleared upon numerical failure
			 *	@param[in] _r_t_timer_sampler is timer sampler for profiling
			 *	@param[in] _r_omega_slim is omega matrix permuted to have no empty rows / columns
			 *	@param[in] _r_required_column_list is the list of columns of Sigma required for the update
			 *	@param[in] _r_prev_marginals is previous marginals, will be updated inplace to current marginals
			 *	@param[in] _r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
			 *	@param[in] _r_R is reference to ordered Cholesky factorization of _r_lambda_in_natural_order
			 *	@param[in] _r_mord is reference to the ordering, used in the Cholesky factorization of _r_R
			 *	@param[in] _b_update_diag_only is diagonal update flag (if set, only the diagonal is updated)
			 */
			inline TOuterContext(bool &_r_b_result, _TyTimerSampler _r_t_timer_sampler,
				const CUberBlockMatrix &_r_omega_slim,
				const std::vector<size_t> &_r_required_column_list,
				CUberBlockMatrix &_r_prev_marginals,
				const CUberBlockMatrix &_r_lambda_in_natural_order, const CUberBlockMatrix &_r_R,
				const CMatrixOrdering &_r_mord, bool _b_update_diag_only)
				:r_b_result(_r_b_result), r_t_timer_sampler(_r_t_timer_sampler),
				r_omega_slim(_r_omega_slim), r_required_column_list(_r_required_column_list),
				r_prev_marginals(_r_prev_marginals),
				r_lambda_in_natural_order(_r_lambda_in_natural_order), r_R(_r_R),
				r_mord(_r_mord), b_update_diag_only(_b_update_diag_only)
			{}
		};

		/**
		 *	@brief middle loop context
		 */
		struct TMiddleContext {
			CUberBlockMatrix &r_prev_marginals; /**< @brief previous marginals, will be updated inplace to current marginals */
			size_t i; /**< @brief outer loop counter */
			size_t n_cur_state_size; /**< @brief current state size, in elements */
			size_t n_prev_state_size; /**< @brief previous marginals size, in elements */
			const double *p_Tu; /**< @brief pointer to the Tu matrix (aligned for SSE where applicable) */
			const double *p_Bu; /**< @brief pointer to the Bu matrix (aligned for SSE where applicable) */

			/**
			 *	@brief default constructor
			 *
			 *	@param[in] _r_prev_marginals is reference to the previous marginals, will be updated
			 *		inplace to current marginals
			 *	@param[in] _i is outer loop counter
			 *	@param[in] _n_cur_state_size is current state size, in elements
			 *	@param[in] _n_prev_state_size is previous marginals size, in elements
			 *	@param[in] _p_Tu is pointer to the Tu matrix (aligned for SSE where applicable)
			 *	@param[in] _p_Bu is pointer to the Bu matrix (aligned for SSE where applicable)
			 */
			inline TMiddleContext(CUberBlockMatrix &_r_prev_marginals,
				size_t _i, size_t _n_cur_state_size, size_t _n_prev_state_size,
				const double *_p_Tu, const double *_p_Bu)
				:r_prev_marginals(_r_prev_marginals), i(_i),
				n_cur_state_size(_n_cur_state_size), n_prev_state_size(_n_prev_state_size),
				p_Tu(_p_Tu), p_Bu(_p_Bu)
			{}
		};

		/**
		 *	@brief inner loop context
		 */
		struct TInnerContext : public TMiddleContext {
			size_t j; /**< @brief middle loop counter */
			size_t n_row_base; /**< @brief leading row of the block being updated (in the previous marginals matrix) */
			size_t n_col_base; /**< @brief leading column of the block being updated (in the previous marginals matrix) */

			/**
			 *	@brief default constructor
			 *
			 *	@param[in] t_ctx is middle loop context
			 *	@param[in] _j is middle loop counter
			 *	@param[in] _n_row_base is leading row of the block being updated (in the previous marginals matrix)
			 *	@param[in] _n_col_base is leading column of the block being updated (in the previous marginals matrix)
			 */
			inline TInnerContext(TMiddleContext t_ctx, size_t _j,
				size_t _n_row_base, size_t _n_col_base)
				:TMiddleContext(t_ctx), j(_j),
				n_row_base(_n_row_base), n_col_base(_n_col_base)
			{}
		};

		/**
		 *	@brief the inner loop; updates a single block of the marginals
		 *
		 *	@tparam n_rows is number of rows of the current block, in elements
		 *	@tparam COmegaElems_Cols is n_omega_elems and number of columns of the
		 *		current block (both in elements), packed as CCTSize2D; in case number
		 *		of columns is -1, it is set the same as the number of rows
		 */
		template <const int n_rows, class COmegaElems_Cols>
		struct TInnerLoop {
			/**
			 *	@brief loop body implementation
			 *	@param[in] t_ctx is loop context
			 */
			static inline void Do(TInnerContext t_ctx)
			{
				enum {
					n_omega_elems = COmegaElems_Cols::n_row_num,
					n_cols = (COmegaElems_Cols::n_column_num == -1)?
						n_rows : COmegaElems_Cols::n_column_num // using this for both diagonal : off-diagonal blocks
				};
				CUberBlockMatrix &r_prev_marginals = t_ctx.r_prev_marginals;
				const size_t i = t_ctx.i;
				const size_t j = t_ctx.j;
				const size_t n_row_base = t_ctx.n_row_base;
				const size_t n_col_base = t_ctx.n_col_base;
				//const size_t n_cur_state_size = t_ctx.n_cur_state_size;
				const size_t n_prev_state_size = t_ctx.n_prev_state_size;
				//const Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, n_omega_elems> >
				//	Tu_full(t_ctx.p_Tu, n_cur_state_size, n_omega_elems);
				enum {
					n_omega_stride = n_Align_Up_POT_Static(n_omega_elems, 8)
				};
				const Eigen::Map<const Eigen::Matrix<double, n_omega_elems, Eigen::Dynamic>,
					Eigen::Aligned, Eigen::OuterStride<n_omega_stride> >
					TuT(t_ctx.p_Tu, n_omega_elems, n_prev_state_size);
				const Eigen::Map<const Eigen::Matrix<double, n_omega_elems, Eigen::Dynamic>,
					Eigen::Aligned, Eigen::OuterStride<n_omega_stride> >
					Bu(t_ctx.p_Bu, n_omega_elems, n_prev_state_size);
				// unwrap the contexts

				//r_prev_marginals.t_Block_AtColumn<n_rows, n_cols>(i, j) -=
				//	Tu_full.template middleRows<n_rows>(n_row_base).lazyProduct(Bu.template middleCols<n_cols>(n_col_base));
				// or use .block(), not sure how this infers the rest of the dimensions (it could, though)

				/*r_prev_marginals.t_Block_AtColumn<n_rows, n_cols>(i, j) -=
					Tu_full.template block<n_rows, n_omega_elems>(n_row_base, 0).lazyProduct(
					Bu.template block<n_omega_elems, n_cols>(0, n_col_base));
				// more of the same with blocks (seems to work, though)*/

				r_prev_marginals.t_Block_AtColumn<n_rows, n_cols>(i, j) -=
					TuT.template block<n_omega_elems, n_rows>(0, n_row_base).transpose().lazyProduct(
					Bu.template block<n_omega_elems, n_cols>(0, n_col_base));
				// if we transpose Tu, the locality of reference increases a lot
			}
		};

		/**
		 *	@brief the middle loop; just visits every block in the given column
		 *		and calls the inner loop to calculate update
		 *
		 *	@tparam n_cols is size of the current block column, in elements
		 *	@tparam COmegaElems_HessianMatrixBlockList is a typelist, containing
		 *		value of n_omega_elems as CCTSize and list of hessian block sizes
		 */
		template <const int n_cols, class COmegaElems_HessianMatrixBlockList>
		struct TMiddleLoop {
			/**
			 *	@brief loop body implementation
			 *	@param[in] t_ctx is loop context
			 */
			static inline void Do(TMiddleContext t_ctx)
			{
				enum {
					n_omega_elems = COmegaElems_HessianMatrixBlockList::_TyHead::n_size
				};
				typedef typename COmegaElems_HessianMatrixBlockList::_TyTail::_TyHead
					_TyHessianMatrixBlockList;
				const CUberBlockMatrix &r_prev_marginals = t_ctx.r_prev_marginals;
				const size_t i = t_ctx.i;
				// unwrap the contexts

				size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);
				for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
					size_t n_row = r_prev_marginals.n_Block_Row(i, j);
					size_t n_row_base = r_prev_marginals.n_BlockColumn_Base(n_row);
					size_t n_rows = r_prev_marginals.n_BlockColumn_Column_Num(n_row); // is symmetric

					fbs_ut::CWrap2<TInnerLoop, fbs_ut::CCTSize2D<n_omega_elems, n_cols> >::template
						In_RowHeight_DecisionTree_Given_ColumnWidth<_TyHessianMatrixBlockList,
						n_cols>(int(n_rows), TInnerContext(t_ctx, j, n_row_base, n_col_base));

					/*r_prev_marginals.t_Block_AtColumn(i, j) -= Tu_full.middleRows(n_row_base,
						n_rows).lazyProduct(Bu.middleCols(n_col_base, n_cols));
					//r_prev_marginals.t_Block_AtColumn(i, j) -=
					//	Tu_full.block(n_row_base, 0, n_rows, n_omega_elems).lazyProduct(
					//	Bu.block(0, n_col_base, n_omega_elems, n_cols)); // maybe it would be faster using Tu_full? it makes no difference here.
					// could FBS these products*/
				}
			}
		};

		/**
		 *	@brief the outer loop of the update; calculates Bu and Tu and calls the inner loops
		 *
		 *	@tparam n_omega_elems is size of packed omega matrix
		 *	@tparam CSystemType is system type
		 */
		template <const int n_omega_elems, class CSystemType>
		struct TOuterLoop {
			typedef typename CSystemType::_TyHessianMatrixBlockList _TyHessianMatrixBlockList; /**< @brief list of block sizes in lambda / omega / marginals */

			/**
			 *	@brief loop body implementation
			 *	@tparam _TyOuterContext is context type (it is specialized by timer sampler type)
			 *	@param[in] t_ctx is loop context
			 *	@note This function throws std::bad_alloc.
			 */
			template <class _TyOuterContext>
			static inline void Do(_TyOuterContext t_ctx) // throw(std::bad_alloc)
			{
				typename _TyOuterContext::_TyTimerSampler timer = t_ctx.r_t_timer_sampler;
				bool &r_b_result = t_ctx.r_b_result;
				const CUberBlockMatrix &lambda = t_ctx.r_lambda_in_natural_order;
				CUberBlockMatrix &r_prev_marginals = t_ctx.r_prev_marginals;
				const CUberBlockMatrix &omega_slim = t_ctx.r_omega_slim;
				const std::vector<size_t> &required_column_list = t_ctx.r_required_column_list;
				const CUberBlockMatrix &r_R = t_ctx.r_R;
				const CMatrixOrdering &mord = t_ctx.r_mord;
				const bool b_update_diag_only = t_ctx.b_update_diag_only;
				// unwrap the context

				typename _TyOuterContext::_TyTimerSampler::_TySample f_omega_time = 0;
				typename _TyOuterContext::_TyTimerSampler::_TySample f_dense_margs_time = 0;
				typename _TyOuterContext::_TyTimerSampler::_TySample f_update_basis_time = 0;
				typename _TyOuterContext::_TyTimerSampler::_TySample f_update_time = 0;
				typename _TyOuterContext::_TyTimerSampler::_TySample f_extend_time = 0;
				// t_odo - is this even legal if _TyOuterContext::_TyTimerSampler is a reference to a type?
				// no, it is not legal, gives a "... is not a class or type name" error.
				// the reference needs to be stripped.

				Eigen::Matrix<double, n_omega_elems, n_omega_elems> omega_dense;
				omega_slim.Convert_to_Dense(omega_dense);
				// initialize the fixed-size matrix

				omega_dense.template triangularView<Eigen::StrictlyLower>() =
					omega_dense.template triangularView<Eigen::StrictlyUpper>().transpose();
				// need both halves! (block matrix omega doesn't contain its lower triangular part)

				timer.Accum_DiffSample(f_omega_time);

				const size_t n_cur_state_size = lambda.n_Column_Num();
				const size_t n_prev_state_size = r_prev_marginals.n_Column_Num();
				const size_t n_prev_state_block_num = r_prev_marginals.n_BlockColumn_Num();
				//const size_t n_omega_elems = omega_slim.n_Column_Num(); // compile-time size now
				const size_t n_packed_block_column_num = omega_slim.n_BlockColumn_Num(); // depends on how n-ary the edge is, currently this is always 2 in here

				Eigen::Matrix<double, Eigen::Dynamic, n_omega_elems> Tu_full(n_cur_state_size, n_omega_elems); // do not allocate it smaller, will need these to update the new covs!
				_ASSERTE(n_packed_block_column_num <= INT_MAX);
				int _n_packed_block_column_num = int(n_packed_block_column_num);
				#pragma omp parallel for if(n_prev_state_block_num > 1000)
				for(int i = 0; i < _n_packed_block_column_num; ++ i) { // t_odo - could run in parallel, but usually needs like two to six columns (threads)
					size_t n_block_base_margs = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
					size_t n_block_base_Tu = omega_slim.n_BlockColumn_Base(i);
					size_t n_block_cols = omega_slim.n_BlockColumn_Column_Num(i);
					// get dimensions of this block

					CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<
						typename CSystemType::_TyHessianMatrixBlockList>(
						Tu_full.block(0, n_block_base_Tu, n_cur_state_size, n_block_cols),
						r_R, n_block_base_margs, mord.p_Get_InverseOrdering(),
						mord.n_Ordering_Size(), mord.n_Ordering_Size()/*n_prev_state_block_num*/);
					// really calculate a block of dense marginals
				}
				Eigen::Block<Eigen::Matrix<double, Eigen::Dynamic, n_omega_elems>,
					Eigen::Dynamic, n_omega_elems> Tu = Tu_full.topRows(n_prev_state_size);
				// assemble Tu (wow, look at *that* type!)

				Eigen::Matrix<double, n_omega_elems, n_omega_elems> s;
				for(size_t i = 0; i < n_packed_block_column_num; ++ i) {
					size_t n_block_base_row_Tu = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
					size_t n_block_base_row_s = omega_slim.n_BlockColumn_Base(i);
					size_t n_block_rows = omega_slim.n_BlockColumn_Column_Num(i); // is symmetric
					// get dimensions of this block

					if(n_block_base_row_Tu < n_prev_state_size) {
						s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems) =
							Tu.block(n_block_base_row_Tu, 0, n_block_rows, n_omega_elems);
					} else
						s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems).setZero();
					// copy block from Tu to s
				}
				// cut out s (could be performed inside a sparse block matrix
				// multiplication, except now we don't have the data in a block matrix)

				timer.Accum_DiffSample(f_dense_margs_time);

#ifdef _DEBUG
				Eigen::LLT<Eigen::Matrix<double, n_omega_elems, n_omega_elems>, Eigen::Upper> llt(omega_dense);
				if(llt.info() == Eigen::Success)
					printf("debug: was able to use the first LL^T Cholesky\n"); // does this ever happen?
				// this happens a
#endif // _DEBUG

				Eigen::Matrix<double, n_omega_elems, n_omega_elems> V =
					Eigen::Matrix<double, n_omega_elems, n_omega_elems>::Identity() - s * omega_dense;
				// calculate V

				// t_odo - do more manual inversion (with LU?) of V,
				// in case it is not invertible, refrain to use batch marginals

				Eigen::FullPivLU<Eigen::Matrix<double, n_omega_elems, n_omega_elems> > luV(V);
				if(!luV.isInvertible()) {
					r_b_result = false;
					return; // that's a runtime err
				}

				typedef forward_allocated_pool<double, 0, 64> CAlloc;
				enum {
					n_omega_stride = n_Align_Up_POT_Static(n_omega_elems, 8)
				};
				double *p_bu_buffer = (double*)CAlloc::aligned_alloc(n_omega_stride * n_prev_state_size * sizeof(double));
				double *p_tut_buffer = (double*)CAlloc::aligned_alloc(n_omega_stride * n_prev_state_size * sizeof(double));
				// t_odo - try to increase n_omega_elems to the next multiple of 8

				Eigen::Map<Eigen::Matrix<double, n_omega_elems, Eigen::Dynamic>,
					Eigen::Aligned, Eigen::OuterStride<n_omega_stride> > Bu(p_bu_buffer, n_omega_elems, n_prev_state_size);
				Eigen::Map<Eigen::Matrix<double, n_omega_elems, Eigen::Dynamic>,
					Eigen::Aligned, Eigen::OuterStride<n_omega_stride> > TuT(p_tut_buffer, n_omega_elems, n_prev_state_size);
				TuT = Tu.transpose(); // keep this as well, for a better memory locality
				Bu = (omega_dense * luV.inverse()) * TuT;
				// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
				// of the product mutliply Tu (that way at least the computation is saved, if not storage)

				timer.Accum_DiffSample(f_update_basis_time);

				_ASSERTE(n_prev_state_block_num <= INT_MAX);
				int _n_prev_state_block_num = int(n_prev_state_block_num);
				#pragma omp parallel for if(n_prev_state_block_num > 1000)
				for(int i = 0; i < _n_prev_state_block_num; ++ i) { // t_odo - this needs to run in parallel
					size_t n_cols = r_prev_marginals.n_BlockColumn_Column_Num(i);
					if(b_update_diag_only) {
						size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);

						fbs_ut::CWrap2<TInnerLoop, fbs_ut::CCTSize2D<n_omega_elems, -1> >::template
							In_ColumnWidth_DecisionTree<_TyHessianMatrixBlockList>(int(n_cols),
							TInnerContext(TMiddleContext(r_prev_marginals, i, n_cur_state_size,
							n_prev_state_size, p_tut_buffer/*Tu_full.data()*/, p_bu_buffer/*Bu.data()*/), i, n_col_base, n_col_base));

						/*r_prev_marginals.t_GetBlock_Log(i, i) -= Tu_full.middleRows(n_col_base,
							n_cols).lazyProduct(Bu.middleCols(n_col_base, n_cols));
						// maybe it would be faster using Tu_full? it makes no difference here.
						// t_odo - could FBS these products*/
					} else {
						typedef typename MakeTypelist(fbs_ut::CCTSize<n_omega_elems>,
							_TyHessianMatrixBlockList) _TySecondaryContext;
						fbs_ut::CWrap2<TMiddleLoop, _TySecondaryContext>::template
							In_ColumnWidth_DecisionTree<_TyHessianMatrixBlockList>(int(n_cols),
							TMiddleContext(r_prev_marginals, i, n_cur_state_size,
							n_prev_state_size, p_tut_buffer/*Tu_full.data()*/, p_bu_buffer/*Bu.data()*/));

						/*size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);
						for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
							size_t n_row = r_prev_marginals.n_Block_Row(i, j);
							size_t n_row_base = r_prev_marginals.n_BlockColumn_Base(n_row);
							size_t n_rows = r_prev_marginals.n_BlockColumn_Column_Num(n_row); // is symmetric

							r_prev_marginals.t_Block_AtColumn(i, j) -= Tu_full.middleRows(n_row_base,
								n_rows).lazyProduct(Bu.middleCols(n_col_base, n_cols));
							//r_prev_marginals.t_Block_AtColumn(i, j) -=
							//	Tu_full.block(n_row_base, 0, n_rows, n_omega_elems).lazyProduct(
							//	Bu.block(0, n_col_base, n_omega_elems, n_cols)); // maybe it would be faster using Tu_full? it makes no difference here.
							// could FBS these products
						}*/
					}
				}
				// update (actually "minus downdate") the existing blocks; in order to check,
				// however, need to update all the blocks that are already there

				timer.Accum_DiffSample(f_update_time);

				size_t n_new_vertex_num = lambda.n_BlockColumn_Num() - n_prev_state_block_num;
				for(size_t i = 0; i < n_new_vertex_num; ++ i) { // can't run in parallel, changes the matrix layout
					size_t n_vertex = n_prev_state_block_num + i;
					size_t n_dim = lambda.n_BlockColumn_Column_Num(n_vertex);

					size_t n_block_col_in_Tu = std::find(required_column_list.begin(),
						required_column_list.end(), n_vertex) - required_column_list.begin();
					size_t n_col_in_Tu = omega_slim.n_BlockColumn_Base(n_block_col_in_Tu);
					size_t n_row_in_Tu = lambda.n_BlockColumn_Base(n_vertex);
					// see which vertex it is in Tu

					/*r_prev_marginals.ExtendTo(r_prev_marginals.n_Row_Num() + n_dim,
						r_prev_marginals.n_Column_Num() + n_dim);
					// debug - just enlarge, no new blocks*/

					r_prev_marginals.t_GetBlock_Log(n_vertex, n_vertex, n_dim, n_dim, true, false) =
						Tu_full.block(n_row_in_Tu, n_col_in_Tu, n_dim, n_dim); // amazingly correct
					// need Tu_full, the blocks are not present in just Tu
					// todo - FBS this (low prio, just a copy)
				}
				// put there the new blocks

				timer.Accum_DiffSample(f_extend_time);

				typename _TyOuterContext::_TyTimerSampler::_TySample f_total_time = 0;
				timer.Accum_CumTime_LastSample(f_total_time);

				if(!CEqualType<typename _TyOuterContext::_TyTimerSampler, CVoidTimerSampler>::b_result) {
					printf("marginals update took %.5f msec\n", f_total_time * 1000.0);
					printf("\tomega: %.5f msec\n", f_omega_time * 1000.0);
					printf("\tTu, s: %.5f msec\n", f_dense_margs_time * 1000.0);
					printf("\tbasis: %.5f msec\n", f_update_basis_time * 1000.0);
					printf("\t  upd: %.5f msec\n", f_update_time * 1000.0);
					printf("\t  ext: %.5f msec\n", f_extend_time * 1000.0);
				}

				CAlloc::aligned_free(p_bu_buffer);
				CAlloc::aligned_free(p_tut_buffer);

				r_b_result = true;
				// all ok
			}
		};

	public:
		/**
		 *	@brief incrementally updates sparse blocky marginals on diagonal
		 *		or everywhere (version with fixed block size)
		 *
		 *	@param[in,out] timer is timer sampler (for profiling)
		 *	@param[in] omega_slim is reordered and packed omega
		 *	@param[in] required_column_list is a list of vertices in omega
		 *	@param[in,out] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
		 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
		 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
		 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R
		 *	@param[in] b_update_diag_only is diagonal update flag (if set, only the diagonal is updated)
		 *
		 *	@return Returns true on success, false on numerical issue
		 *		(in that case, r_prev_marginals is not modified).
		 */
		template <class CSystemType, class _TyTimerSamplerRef>
		static bool Run(_TyTimerSamplerRef timer, const CUberBlockMatrix &omega_slim,
			const std::vector<size_t> &required_column_list, CUberBlockMatrix &r_prev_marginals,
			const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
			const CMatrixOrdering &mord, bool b_update_diag_only) // throw(std::bad_alloc)
		{
			const size_t n_omega_elems = omega_slim.n_Column_Num();
			typedef typename CTransformTypelist<typename CSystemType::_TyEdgeTypelist,
				CEdgeTypeToSumOfVertexDims>::_TyResult TEdgeVertsSizeList; // vertex sizes of those
			bool b_result;
			_ASSERTE(n_omega_elems <= INT_MAX);
			fbs_ut::CWrap2<TOuterLoop, CSystemType>::template
				In_ScalarSize_DecisionTree<TEdgeVertsSizeList>(int(n_omega_elems),
				TOuterContext<_TyTimerSamplerRef>(b_result, timer, omega_slim,
				required_column_list, r_prev_marginals, r_lambda_in_natural_order,
				r_R, mord, b_update_diag_only));
			// entere decision tree

			return b_result; // todo - move the decision tree code here
		}
	};

#if defined(__SE_TYPES_SUPPORT_L_SOLVERS) || 1

#if 0 // this was replaced by a more intelligent function
	/**
	 *	@brief incrementally updates sparse blocky marginals on diagonal
	 *		or everywhere (version with fixed block size)
	 *
	 *	@tparam b_enable_timer is timing enable flag (if disabled, incurs no runtime costs)
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in,out] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] b_update_diag_only is diagonal update flag (if set, only the diagonal is updated)
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	// *	@note This is only available if __SE_TYPES_SUPPORT_L_SOLVERS is defined (the Omega matrix is used).
	template <bool b_enable_timer, class CSystemType>
	static bool Update_BlockDiagonalMarginals_FBS(const CSystemType &system, CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &mord, size_t n_edges_in_prev_marginals, bool b_update_diag_only = false) // throw(std::bad_alloc)
	{
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSampler _TyTimerSampler;
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSamplerRef _TyTimerSamplerRef;
		typedef typename _TyTimerSampler::_TySample _TySample;
		// choose a timer based on whether it is enabled or not

		CTimer t;
		_TyTimerSampler timer(t);

		const CUberBlockMatrix &lambda = r_lambda_in_natural_order; // rename
		const size_t n_prev_edge_num = n_edges_in_prev_marginals;
		size_t n_edge_num = system.r_Edge_Pool().n_Size();

		if(n_prev_edge_num == n_edge_num)
			return true;
		// another job well done

		size_t n_order_min = r_prev_marginals.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = r_prev_marginals.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)

		CUberBlockMatrix omega; // todo - this might be up to the linear solver, in LM omega will be different
		std::vector<size_t> required_column_list;
		required_column_list.reserve(2 * (n_edge_num - n_prev_edge_num)); // a guess; might be edges with more or less than 2 verts
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) { // not parallel! (wouls have conflicts)
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			r_edge.Calculate_Omega(omega, n_elem_order_min);
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				required_column_list.push_back(r_edge.n_Vertex_Id(j));
		}
		// get omega and vertex id's

		std::sort(required_column_list.begin(), required_column_list.end());
		required_column_list.erase(std::unique(required_column_list.begin(),
			required_column_list.end()), required_column_list.end());
		// finalize the required column list (could use std::set, but that feels like overkill)

		std::vector<size_t> pack_order;
		pack_order.reserve(omega.n_BlockColumn_Num());
		for(size_t i = 0, n = required_column_list.size(); i < n; ++ i) {
			size_t n_col = required_column_list[i];
			size_t n_base = lambda.n_BlockColumn_Base(n_col) - n_elem_order_min;
			size_t n_col_size;
			size_t n_col_omega = omega.n_Find_BlockColumn(n_base, n_col_size);
			_ASSERTE(n_col_size == lambda.n_BlockColumn_Column_Num(n_col));

			pack_order.push_back(n_col_omega);
		}
		// get numbers of "interesting" columns of omega (no ordering here either)

		const size_t n_packed_block_column_num = pack_order.size();
		// remember this

		for(size_t i = 1; i < n_packed_block_column_num; ++ i) {
			size_t n_o0 = pack_order[i - 1];
			size_t n_o1 = pack_order[i];
			_ASSERTE(n_o1 > n_o0); // should be sorted

			for(size_t j = n_o0 + 1; j < n_o1; ++ j)
				pack_order.push_back(j);
			// append the rest of the cols at the end (only up to omega.n_BlockColumn_Num() / 2 of them)
		}
		_ASSERTE(pack_order.size() == omega.n_BlockColumn_Num());
		// finalize the order

		std::vector<size_t> inv_pack_order(pack_order.size());
		for(size_t i = 0, n = pack_order.size(); i < n; ++ i)
			inv_pack_order[pack_order[i]] = i;
		// inverse the order

		CUberBlockMatrix omega_slim;
		omega.Permute_UpperTriangular_To(omega_slim,
			&inv_pack_order[0], inv_pack_order.size(), true);
		// pack omega

		omega_slim.SliceTo(omega_slim, n_packed_block_column_num,
			n_packed_block_column_num, true);
		_ASSERTE(omega_slim.n_BlockColumn_Num() == n_packed_block_column_num);
		// slice off the empty columns

		if(n_prev_edge_num == n_edge_num + 1) {
			const size_t n_omega_elems = omega_slim.n_Column_Num();
#ifdef _DEBUG
			{
				size_t n_edge_dim = 0;
				typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[n_prev_edge_num];
				for(size_t i = 0, n = r_edge.n_Vertex_Num(); i < n; ++ i)
					n_edge_dim += system.r_Vertex_Pool()[r_edge.n_Vertex_Id(i)].n_Dimension();
				_ASSERTE(n_edge_dim == n_omega_elems);
			}
			// edge dimension should equal sum of dimensions of its vertices
#endif // _DEBUG

			return CMarginalsUpdate_FBSKernel::template Run<CSystemType, _TyTimerSamplerRef>(timer,
				omega_slim, required_column_list, r_prev_marginals, r_lambda_in_natural_order, r_R,
				mord, b_update_diag_only);
			/*typedef typename CTransformTypelist<typename CSystemType::_TyEdgeTypelist,
				CMarginalsUpdate_FBSKernel::CEdgeTypeToSumOfVertexDims>::_TyResult TEdgeVertsSizeList; // vertex sizes of those
			bool b_result;
			_ASSERTE(n_omega_elems <= INT_MAX);
			fbs_ut::CWrap2<CMarginalsUpdate_FBSKernel::TOuterLoop, CSystemType>::template
				In_ScalarSize_DecisionTree<TEdgeVertsSizeList>(int(n_omega_elems),
				CMarginalsUpdate_FBSKernel::TOuterContext<_TyTimerSamplerRef>(b_result, timer,
				omega_slim, required_column_list, r_prev_marginals, r_lambda_in_natural_order,
				r_R, mord, b_update_diag_only));
			// entere decision tree

			return b_result;*/
		} // not proficient; t_odo - make a fake block matrix to hold all the dense mats (Tu. Bu, S, ...) so that they are aligned!
		// handle FBS processing (one dimension of Tu and Bu, and both dimensions
		// of both matrices are known at compile-time)

		Eigen::MatrixXd omega_dense;
		omega_slim.Convert_to_Dense(omega_dense);
		// get dense omega

		//omega_dense.bottomRightCorner(omega_dense.rows() - omega_slim.n_BlockColumn_Column_Num(0),
		//	omega_dense.cols() - omega_slim.n_BlockColumn_Column_Num(0)).diagonal().array() += .1;
		// slightly lift the diagonal (except the first vertex), like it would happen in damped least squares

		_TySample f_omega_time = 0;
		_TySample f_dense_margs_time = 0;
		_TySample f_update_basis_time = 0;
		_TySample f_update_time = 0;
		_TySample f_extend_time = 0;

		timer.Accum_DiffSample(f_omega_time);

		const size_t n_cur_state_size = lambda.n_Column_Num();
		const size_t n_prev_state_size = r_prev_marginals.n_Column_Num();
		const size_t n_prev_state_block_num = r_prev_marginals.n_BlockColumn_Num();
		const size_t n_omega_elems = omega_slim.n_Column_Num();

		Eigen::MatrixXd Tu_full(n_cur_state_size, n_omega_elems); // do not allocate it smaller, will need these to update the new covs!
		_ASSERTE(n_packed_block_column_num <= INT_MAX);
		int _n_packed_block_column_num = int(n_packed_block_column_num);
		#pragma omp parallel for if(n_prev_state_block_num > 1000)
		for(int i = 0; i < _n_packed_block_column_num; ++ i) { // t_odo - could run in parallel, but usually needs like two to six columns (threads)
			size_t n_block_base_margs = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
			size_t n_block_base_Tu = omega_slim.n_BlockColumn_Base(i);
			size_t n_block_cols = omega_slim.n_BlockColumn_Column_Num(i);
			// get dimensions of this block

			CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<
				typename CSystemType::_TyHessianMatrixBlockList>(
				Tu_full.block(0, n_block_base_Tu, n_cur_state_size, n_block_cols),
				r_R, n_block_base_margs, mord.p_Get_InverseOrdering(),
				mord.n_Ordering_Size(), mord.n_Ordering_Size()/*n_prev_state_block_num*/);
			// really calculate a block of dense marginals
		}
		Eigen::Block<Eigen::MatrixXd> Tu = Tu_full.topLeftCorner(n_prev_state_size, n_omega_elems);
		// assemble Tu

		Eigen::MatrixXd s(n_omega_elems, n_omega_elems);
		for(size_t i = 0; i < n_packed_block_column_num; ++ i) {
			size_t n_block_base_row_Tu = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
			size_t n_block_base_row_s = omega_slim.n_BlockColumn_Base(i);
			size_t n_block_rows = omega_slim.n_BlockColumn_Column_Num(i); // is symmetric
			// get dimensions of this block

			if(n_block_base_row_Tu < n_prev_state_size) {
				s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems) =
					Tu.block(n_block_base_row_Tu, 0, n_block_rows, n_omega_elems);
			} else
				s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems).setZero();
			// copy block from Tu to s
		}
		// cut out s (could be performed inside a sparse block matrix
		// multiplication, except now we don't have the data in a block matrix)

		timer.Accum_DiffSample(f_dense_margs_time);

/*#ifdef _DEBUG
		Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> llt(omega_dense);
		if(llt.info() == Eigen::Success)
			printf("debug: was able to use the first LL^T Cholesky\n"); // does this ever happen? it does, when one is very certain, typically when the covariance associated with the edges is high
#endif // _DEBUG*/

		omega_dense.triangularView<Eigen::StrictlyLower>() =
			omega_dense.triangularView<Eigen::StrictlyUpper>().transpose();
		// need both halves! (block matrix omega doesn't contain its lower triangular part)

		Eigen::MatrixXd V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) - s * omega_dense;
		// calculate V

		// t_odo - do more manual inversion (with LU?) of V,
		// in case it is not invertible, refrain to use batch marginals

		Eigen::FullPivLU<Eigen::MatrixXd> luV(V);
		if(!luV.isInvertible())
			return false; // that's a runtime err

		//Eigen::MatrixXd Bu = ((omega_dense * luV.inverse()) * Tu.transpose());
		// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
		// of the product mutliply Tu (that way at least the computation is saved, if not storage)

		typedef forward_allocated_pool<double, 0, 64> CAlloc;
		const size_t n_omega_stride = n_Align_Up_POT(n_omega_elems, size_t(8));
		double *p_tut_buffer = (double*)CAlloc::aligned_alloc(n_omega_stride * n_prev_state_size * sizeof(double));
		double *p_bu_buffer = (double*)CAlloc::aligned_alloc(n_omega_stride * n_prev_state_size * sizeof(double));
		// t_odo - try to increase n_omega_elems to the next multiple of 8

		Eigen::Map<Eigen::MatrixXd, Eigen::Aligned, Eigen::OuterStride<> > TuT(p_tut_buffer, n_omega_elems,
			n_prev_state_size, Eigen::OuterStride<>(n_omega_stride));
		Eigen::Map<Eigen::MatrixXd, Eigen::Aligned, Eigen::OuterStride<> > Bu(p_bu_buffer, n_omega_elems,
			n_prev_state_size, Eigen::OuterStride<>(n_omega_stride));
		TuT = Tu.transpose(); // keep this as well, for a better memory locality
		Bu = (omega_dense * luV.inverse()) * TuT;
		// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
		// of the product mutliply Tu (that way at least the computation is saved, if not storage)

		timer.Accum_DiffSample(f_update_basis_time);

		_ASSERTE(n_prev_state_block_num <= INT_MAX);
		int _n_prev_state_block_num = int(n_prev_state_block_num);
		#pragma omp parallel for if(n_prev_state_block_num > 1000)
		for(int i = 0; i < _n_prev_state_block_num; ++ i) { // t_odo - this needs to run in parallel
			size_t n_cols = r_prev_marginals.n_BlockColumn_Column_Num(i);
			if(b_update_diag_only) {
				size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);
#if 0
				fbs_ut::CWrap2<CMarginalsUpdate_FBSKernel::TInnerLoop,
					fbs_ut::CCTSize2D<n_omega_elems, -1> >::template
					In_ColumnWidth_DecisionTree<_TyHessianMatrixBlockList>(int(n_cols),
					CMarginalsUpdate_FBSKernel::TInnerContext(
					CMarginalsUpdate_FBSKernel::TMiddleContext(r_prev_marginals, i, n_cur_state_size,
					n_prev_state_size, p_tut_buffer/*Tu_full.data()*/,
					p_bu_buffer/*Bu.data()*/), i, n_col_base, n_col_base));
#endif // 0
				// n_omega_elems is not constant, would have to rewrite this a bit (it wouldn't be completely FBS)

				for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
					if(r_prev_marginals.n_Block_Row(i, j) == i)
						continue; // not the diag block
					r_prev_marginals.t_Block_AtColumn(i, j).setZero();
				}
				// zero out other blocks so that i can see it

				r_prev_marginals.t_GetBlock_Log(i, i) -= TuT.middleCols(n_col_base,
					n_cols).transpose().lazyProduct(Bu.middleCols(n_col_base, n_cols));
				// maybe it would be faster using Tu_full? it makes no difference here.
				// t_odo - could FBS these products*/
			} else {
#if 0
				typedef typename MakeTypelist(fbs_ut::CCTSize<n_omega_elems>,
					_TyHessianMatrixBlockList) _TySecondaryContext;
				fbs_ut::CWrap2<CMarginalsUpdate_FBSKernel::TMiddleLoop, _TySecondaryContext>::template
					In_ColumnWidth_DecisionTree<_TyHessianMatrixBlockList>(int(n_cols),
					CMarginalsUpdate_FBSKernel::TMiddleContext(r_prev_marginals, i, n_cur_state_size,
					n_prev_state_size, p_tut_buffer/*Tu_full.data()*/, p_bu_buffer/*Bu.data()*/));
#endif // 0
				// n_omega_elems is not constant, would have to rewrite this a bit (it wouldn't be completely FBS)

				size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);
				for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
					size_t n_row = r_prev_marginals.n_Block_Row(i, j);
					size_t n_row_base = r_prev_marginals.n_BlockColumn_Base(n_row);
					size_t n_rows = r_prev_marginals.n_BlockColumn_Column_Num(n_row); // is symmetric

					r_prev_marginals.t_Block_AtColumn(i, j) -= TuT.middleCols(n_row_base,
						n_rows).transpose().lazyProduct(Bu.middleCols(n_col_base, n_cols));
					//r_prev_marginals.t_Block_AtColumn(i, j) -=
					//	Tu_full.block(n_row_base, 0, n_rows, n_omega_elems).lazyProduct(
					//	Bu.block(0, n_col_base, n_omega_elems, n_cols)); // maybe it would be faster using Tu_full? it makes no difference here.
					// could FBS these products
				}
			}
		}
		// update (actually "minus downdate") the existing blocks; in order to check,
		// however, need to update all the blocks that are already there

		timer.Accum_DiffSample(f_update_time);

		size_t n_new_vertex_num = lambda.n_BlockColumn_Num() - n_prev_state_block_num;
		for(size_t i = 0; i < n_new_vertex_num; ++ i) { // can't run in parallel, changes the matrix layout
			size_t n_vertex = n_prev_state_block_num + i;
			size_t n_dim = lambda.n_BlockColumn_Column_Num(n_vertex);

			size_t n_block_col_in_Tu = std::find(required_column_list.begin(),
				required_column_list.end(), n_vertex) - required_column_list.begin();
			size_t n_col_in_Tu = omega_slim.n_BlockColumn_Base(n_block_col_in_Tu);
			size_t n_row_in_Tu = lambda.n_BlockColumn_Base(n_vertex);
			// see which vertex it is in Tu

			/*r_prev_marginals.ExtendTo(r_prev_marginals.n_Row_Num() + n_dim,
				r_prev_marginals.n_Column_Num() + n_dim);
			// debug - just enlarge, no new blocks*/

			r_prev_marginals.t_GetBlock_Log(n_vertex, n_vertex, n_dim, n_dim, true, false) =
				Tu_full.block(n_row_in_Tu, n_col_in_Tu, n_dim, n_dim); // amazingly correct
			// need Tu_full, the blocks are not present in just Tu
		}
		// put there the new blocks

		CAlloc::aligned_free(p_bu_buffer);
		CAlloc::aligned_free(p_tut_buffer);
		// should free in catch(bad_alloc) as well, will leave memory allocated if something throws

		timer.Accum_DiffSample(f_extend_time);

		_TySample f_total_time = 0;
		timer.Accum_CumTime_LastSample(f_total_time);

		if(b_enable_timer) {
			printf("marginals update took %.5f msec\n", f_total_time * 1000.0);
			printf("\tomega: %.5f msec\n", f_omega_time * 1000.0);
			printf("\tTu, s: %.5f msec\n", f_dense_margs_time * 1000.0);
			printf("\tbasis: %.5f msec\n", f_update_basis_time * 1000.0);
			printf("\t  upd: %.5f msec\n", f_update_time * 1000.0);
			printf("\t  ext: %.5f msec\n", f_extend_time * 1000.0);
		}

		return true;
	}
#endif // 0

	/**
	 *	@brief calculates approximate number of FLOPs which will be spent in sparse
	 *		calculation of marginals using the recursive formula
	 */
	static double f_RecursiveFormula_FLOP_Num(const CUberBlockMatrix &r_R, EBlockMatrixPart n_matrix_part)
	{
		if(n_matrix_part == mpart_Nothing)
			return 0;

		size_t nnz = r_R.n_NonZero_Num(), N = r_R.n_Column_Num();
		if((n_matrix_part & mpart_Diagonal) != mpart_Diagonal) {
			return (nnz)? double(nnz) * r_R.n_BlockColumn_Column_Num(r_R.n_BlockColumn_Num() - 1) : 0;
			// very approximate cost for the last column / block using backsubstitution
		}
		// make sure that the diagonal is required; otherwise the cost is quite different

		return double(nnz) * nnz * N; // O(n_{nz}^2N) as in chapter IV of the paper
		// cost for calculating the skeleton marginals (the sparsity pattern of R),
		// which should be the dominating cost right now
	}

	/**
	 *	@brief calculates approximate number of FLOPs which will be spent in dense calculation in marginals update
	 *
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R (unused
	 *		at the moment, but could be useful to get a more precise approximate taking resumed
	 *		backsubstitution into account)
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] n_matrix_part is combination of EBlockMatrixPart
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CSystemType>
	static double f_MarginalsUpdate_DenseFLOP_Num(const CSystemType &system, const CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &UNUSED(mord), size_t n_edges_in_prev_marginals, EBlockMatrixPart n_matrix_part) // throw(std::bad_alloc)
	{
		const CUberBlockMatrix &lambda = r_lambda_in_natural_order; // rename
		const size_t n_prev_edge_num = n_edges_in_prev_marginals;
		size_t n_edge_num = system.r_Edge_Pool().n_Size();

		if(n_prev_edge_num == n_edge_num)
			return 0;
		if(n_matrix_part == mpart_Nothing)
			return 0;
		// free cases

		//size_t n_backsubst_cost = r_R.n_Storage_Size(); // a quick and dirty approximate of NNZs
		size_t n_backsubst_cost = r_R.n_NonZero_Num();
		if((n_matrix_part == mpart_LastBlock || n_matrix_part == mpart_LastColumn) &&
		   lambda.n_BlockColumn_Num() > r_prev_marginals.n_BlockColumn_Num()) {
			size_t n_new_column_num = lambda.n_BlockColumn_Num() - r_prev_marginals.n_BlockColumn_Num();
			return double(n_new_column_num) * n_backsubst_cost; // double so we don't have to care about overflows (as much)
		}
		// pure extension cannot be updated

#if 0
		size_t n_order_min = r_prev_marginals.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = r_prev_marginals.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)
#endif // 0
		// not needed, we dont form omega here

		std::vector<size_t> required_column_list;
		required_column_list.reserve(2 * (n_edge_num - n_prev_edge_num)); // a guess; might be edges with more or less than 2 verts
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) { // not parallel! (wouls have conflicts)
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				required_column_list.push_back(r_edge.n_Vertex_Id(j));
		}
		// get omega and vertex id's

		std::sort(required_column_list.begin(), required_column_list.end());
		required_column_list.erase(std::unique(required_column_list.begin(),
			required_column_list.end()), required_column_list.end());
		// finalize the required column list (could use std::set, but that feels like overkill)

		size_t n_Tu_full_column_num = 0;
		for(size_t i = 0, n = required_column_list.size(); i < n; ++ i)
			n_Tu_full_column_num += lambda.n_BlockColumn_Column_Num(required_column_list[i]);
		// calculate how many columns do we need to backsubstitute for

		return double(n_Tu_full_column_num) * n_backsubst_cost; // double so we don't have to care about overflows (as much)
	}

	/**
	 *	@brief calculates approximate memory size required for dense calculation in marginals update
	 *
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R (unused
	 *		at the moment, but could be useful to get a more precise approximate taking resumed
	 *		backsubstitution into account)
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] n_matrix_part is combination of EBlockMatrixPart
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CSystemType>
	static double f_MarginalsUpdate_DenseBytes(const CSystemType &system, const CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &UNUSED(mord), size_t n_edges_in_prev_marginals, EBlockMatrixPart n_matrix_part) // throw(std::bad_alloc)
	{
		const CUberBlockMatrix &lambda = r_lambda_in_natural_order; // rename
		const size_t n_prev_edge_num = n_edges_in_prev_marginals;
		size_t n_edge_num = system.r_Edge_Pool().n_Size();

		if(n_prev_edge_num == n_edge_num)
			return 0;
		if(n_matrix_part == mpart_Nothing)
			return 0;
		// free cases

		size_t n_row_size = r_R.n_Row_Num();
		if((n_matrix_part == mpart_LastBlock || n_matrix_part == mpart_LastColumn) &&
		   lambda.n_BlockColumn_Num() > r_prev_marginals.n_BlockColumn_Num()) {
			size_t n_new_column_num = lambda.n_BlockColumn_Num() - r_prev_marginals.n_BlockColumn_Num();
			return double(n_new_column_num) * sizeof(double) * n_row_size; // double so we don't have to care about overflows (as much)
		}
		// pure extension cannot be updated

#if 0
		size_t n_order_min = r_prev_marginals.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = r_prev_marginals.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)
#endif // 0
		// not needed, we dont form omega here

		std::vector<size_t> required_column_list;
		required_column_list.reserve(2 * (n_edge_num - n_prev_edge_num)); // a guess; might be edges with more or less than 2 verts
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) { // not parallel! (wouls have conflicts)
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				required_column_list.push_back(r_edge.n_Vertex_Id(j));
		}
		// get omega and vertex id's

		std::sort(required_column_list.begin(), required_column_list.end());
		required_column_list.erase(std::unique(required_column_list.begin(),
			required_column_list.end()), required_column_list.end());
		// finalize the required column list (could use std::set, but that feels like overkill)

		size_t n_Tu_full_column_num = 0;
		for(size_t i = 0, n = required_column_list.size(); i < n; ++ i)
			n_Tu_full_column_num += lambda.n_BlockColumn_Column_Num(required_column_list[i]);
		// calculate how many columns do we need to backsubstitute for

		return double(n_Tu_full_column_num) * sizeof(double) * n_row_size; // double so we don't have to care about overflows (as much)
	}

	/**
	 *	@brief a simple heuristic deciding whether it is better to update the marginals incrementally or batch
	 *
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R (unused
	 *		at the moment, but could be useful to get a more precise approximate taking resumed
	 *		backsubstitution into account)
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] n_matrix_part is combination of EBlockMatrixPart
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note The implementation of this function will almost certainly change.
	 */
	template <class CSystemType>
	static bool b_PreferIncremental(const CSystemType &system, const CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &mord, size_t n_edges_in_prev_marginals, EBlockMatrixPart n_matrix_part) // throw(std::bad_alloc)
	{
		double f_mem_cost = f_MarginalsUpdate_DenseBytes(system, r_prev_marginals,
			r_lambda_in_natural_order, r_R, mord, n_edges_in_prev_marginals, n_matrix_part);
		const double f_thresh = 1000000 * 64 * sizeof(double);
		return f_mem_cost < f_thresh;
		// for now just prefer based on memory use of the incremental update
		// allow update on 1M vertices with rank-64 update, or equivalent (512 MB worth of dense data)

		// the problem with f_MarginalsUpdate_DenseFLOP_Num() and f_RecursiveFormula_FLOP_Num() is that
		// while they somehow reflect the number of FLOPs, they are incomparable due to different constant
		// factors; will need to verify that these numbers indeed reflect the time spent there and estimate
		// the constant factors on the current hardware
	}

	/**
	 *	@brief incrementally updates sparse blocky marginals on diagonal
	 *		or everywhere (version with fixed block size)
	 *
	 *	@tparam b_enable_timer is timing enable flag (if disabled, incurs no runtime costs)
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in,out] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] n_matrix_part is combination of EBlockMatrixPart
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	// *	@note This is only available if __SE_TYPES_SUPPORT_L_SOLVERS is defined (the Omega matrix is used).
	template <bool b_enable_timer, class CSystemType>
	static bool Update_BlockDiagonalMarginals_FBS(const CSystemType &system, CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &mord, size_t n_edges_in_prev_marginals, EBlockMatrixPart n_matrix_part) // throw(std::bad_alloc)
	{
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSampler _TyTimerSampler;
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSamplerRef _TyTimerSamplerRef;
		typedef typename _TyTimerSampler::_TySample _TySample;
		// choose a timer based on whether it is enabled or not

		typedef typename CSystemType::_TyHessianMatrixBlockList _TyLambdaMatrixBlockSizes;
		// get the block sizes

		const CUberBlockMatrix &lambda = r_lambda_in_natural_order; // rename
		const size_t n_prev_edge_num = n_edges_in_prev_marginals;
		size_t n_edge_num = system.r_Edge_Pool().n_Size();

		if(n_prev_edge_num == n_edge_num) {
			_ASSERTE(r_prev_marginals.b_EqualLayout(lambda)); // should have, unless someone meddled with it
			return true;
		}
		// another job well done, there is no source for the update

		if(n_matrix_part & mpart_Column)
			n_matrix_part = EBlockMatrixPart(n_MPart_Subtract(n_matrix_part, mpart_Column) | mpart_LastColumn);
		_ASSERTE((n_matrix_part & mpart_Column) != mpart_Column);
		// if a column is wanted, it is the last column in this context

		if(n_matrix_part == mpart_Nothing) {
			r_prev_marginals.ExtendTo(lambda.n_Row_Num(), lambda.n_Column_Num()); // so it has correct size
			return true;
		}
		// nothing to do

		if(n_matrix_part == mpart_LastBlock || n_matrix_part == mpart_LastColumn) {
			if(lambda.n_BlockColumn_Num() > r_prev_marginals.n_BlockColumn_Num()) {
				CUberBlockMatrix margs_ordered;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<_TyLambdaMatrixBlockSizes>(margs_ordered, r_R, mord,
					/*(n_matrix_part == mpart_FullMatrix)? mpart_Nothing :*/ n_matrix_part, // mpart_FullMatrix intepreted as structure of R here (since this is an update)
					/*(n_matrix_part == mpart_FullMatrix)? true :*/ false); // meek point, as n_matrix_part == mpart_LastBlock || n_matrix_part == mpart_LastColumn
				// calculate the thing (always succeeds, except for std::bad_alloc)

				margs_ordered.Permute_UpperTriangular_To(r_prev_marginals, mord.p_Get_Ordering(),
					mord.n_Ordering_Size(), false); // no share! the original will be deleted
				// take care of having the correct permutation there

				return true;
			}
			// the last block or the last column can't be updated if the size of the matrix increased
		}

		CTimer t;
		_TyTimerSampler timer(t);

		size_t n_order_min = r_prev_marginals.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = r_prev_marginals.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)

		CUberBlockMatrix omega; // t_odo - this might be up to the linear solver, in LM omega will be different // in LM, a new un-damped lambda needs to be built to calculate R, so it makes a little difference
		std::vector<size_t> required_column_list;
		required_column_list.reserve(2 * (n_edge_num - n_prev_edge_num)); // a guess; might be edges with more or less than 2 verts
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) { // not parallel! (wouls have conflicts)
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			r_edge.Calculate_Omega(omega, n_elem_order_min);
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				required_column_list.push_back(r_edge.n_Vertex_Id(j));
		}
		// get omega and vertex id's

		std::sort(required_column_list.begin(), required_column_list.end());
		required_column_list.erase(std::unique(required_column_list.begin(),
			required_column_list.end()), required_column_list.end());
		// finalize the required column list (could use std::set, but that feels like overkill)

		_TySample f_omega_build_time = 0;
		timer.Accum_DiffSample(f_omega_build_time);

		return Update_BlockDiagonalMarginals_FBS_ExOmega<b_enable_timer>(system, r_prev_marginals,
			r_lambda_in_natural_order, r_R, mord, n_edges_in_prev_marginals, n_matrix_part,
			f_omega_build_time, omega, required_column_list); // massif: +35,893,584B (via a matrix product) +35,893,584B (via a matrix product) +8,388,608B (via block matrix p_Find_Block())
	}

	/**
	 *	@brief incrementally updates sparse blocky marginals on diagonal
	 *		or everywhere (version with fixed block size), using explicitly calculated omega
	 *
	 *	@tparam b_enable_timer is timing enable flag (if disabled, incurs no runtime costs)
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in,out] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] n_matrix_part is combination of EBlockMatrixPart
	 *	@param[in] f_omega_build_time is time required to calculate the omega matrix, in seconds (for profiling)
	 *	@param[in] omega is value of the omega matrix
	 *	@param[in] required_column_list is the list of columns of Sigma required for updating the marginals
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	// *	@note This is only available if __SE_TYPES_SUPPORT_L_SOLVERS is defined (the Omega matrix is used).
	template <bool b_enable_timer, class CSystemType>
	static bool Update_BlockDiagonalMarginals_FBS_ExOmega(const CSystemType &system, CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &mord, size_t n_edges_in_prev_marginals, EBlockMatrixPart n_matrix_part,
		double f_omega_build_time, const CUberBlockMatrix &omega, const std::vector<size_t> &required_column_list) // additional params to pass:
	{
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSampler _TyTimerSampler;
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSamplerRef _TyTimerSamplerRef;
		typedef typename _TyTimerSampler::_TySample _TySample;
		// choose a timer based on whether it is enabled or not

		typedef typename CSystemType::_TyHessianMatrixBlockList _TyLambdaMatrixBlockSizes;
		// get the block sizes

		const CUberBlockMatrix &lambda = r_lambda_in_natural_order; // rename
		const size_t n_prev_edge_num = n_edges_in_prev_marginals;
		const size_t n_edge_num = system.r_Edge_Pool().n_Size();

		_ASSERTE(!required_column_list.empty());
		const size_t n_order_min = required_column_list.front(); // basically contains ids of affected vertices, and is sorted
		const size_t n_elem_order_min = r_prev_marginals.n_BlockColumn_Base(n_order_min);

		// t_odo - we need to have omega; split this function ro two, with entry point where omega is already available

		CTimer t;
		_TyTimerSampler timer(t);

		std::vector<size_t> pack_order;
		pack_order.reserve(omega.n_BlockColumn_Num());
		for(size_t i = 0, n = required_column_list.size(); i < n; ++ i) {
			size_t n_col = required_column_list[i];
			size_t n_base = lambda.n_BlockColumn_Base(n_col) - n_elem_order_min;
			size_t n_col_size;
			size_t n_col_omega = omega.n_Find_BlockColumn(n_base, n_col_size);
			_ASSERTE(n_col_size == lambda.n_BlockColumn_Column_Num(n_col));

			pack_order.push_back(n_col_omega);
		}
		// get numbers of "interesting" columns of omega (no ordering here either)

		const size_t n_packed_block_column_num = pack_order.size();
		// remember this

		for(size_t i = 1; i < n_packed_block_column_num; ++ i) {
			size_t n_o0 = pack_order[i - 1];
			size_t n_o1 = pack_order[i];
			_ASSERTE(n_o1 > n_o0); // should be sorted

			for(size_t j = n_o0 + 1; j < n_o1; ++ j)
				pack_order.push_back(j);
			// append the rest of the cols at the end (only up to omega.n_BlockColumn_Num() / 2 of them)
		}
		_ASSERTE(pack_order.size() == omega.n_BlockColumn_Num());
		// finalize the order

		std::vector<size_t> inv_pack_order(pack_order.size());
		for(size_t i = 0, n = pack_order.size(); i < n; ++ i)
			inv_pack_order[pack_order[i]] = i;
		// inverse the order

		CUberBlockMatrix omega_slim;
		omega.Permute_UpperTriangular_To(omega_slim,
			&inv_pack_order[0], inv_pack_order.size(), true);
		// pack omega

		omega_slim.SliceTo(omega_slim, n_packed_block_column_num,
			n_packed_block_column_num, true);
		_ASSERTE(omega_slim.n_BlockColumn_Num() == n_packed_block_column_num);
		// slice off the empty columns

		if(n_prev_edge_num == n_edge_num + 1) {
			/*const size_t n_omega_elems = omega_slim.n_Column_Num();
#ifdef _DEBUG
			{
				size_t n_edge_dim = 0;
				typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[n_prev_edge_num];
				for(size_t i = 0, n = r_edge.n_Vertex_Num(); i < n; ++ i)
					n_edge_dim += system.r_Vertex_Pool()[r_edge.n_Vertex_Id(i)].n_Dimension();
				_ASSERTE(n_edge_dim == n_omega_elems);
			}
			// edge dimension should equal sum of dimensions of its vertices
#endif // _DEBUG

			return CMarginalsUpdate_FBSKernel::template Run<CSystemType, _TyTimerSamplerRef>(timer,
				omega_slim, required_column_list, r_prev_marginals, r_lambda_in_natural_order, r_R,
				mord, n_matrix_part);*/ // todo - implement this

			static bool b_warn = false;
			if(!b_warn) {
				b_warn = true;
				fprintf(stderr, "warning: CMarginalsUpdate_FBSKernel::Run() does not honor marginal policy, not used\n");
				// if this turns out to be performance bottleneck, must uncomment and rewrite the above code
			}

			/*typedef typename CTransformTypelist<typename CSystemType::_TyEdgeTypelist,
				CMarginalsUpdate_FBSKernel::CEdgeTypeToSumOfVertexDims>::_TyResult TEdgeVertsSizeList; // vertex sizes of those
			bool b_result;
			_ASSERTE(n_omega_elems <= INT_MAX);
			fbs_ut::CWrap2<CMarginalsUpdate_FBSKernel::TOuterLoop, CSystemType>::template
				In_ScalarSize_DecisionTree<TEdgeVertsSizeList>(int(n_omega_elems),
				CMarginalsUpdate_FBSKernel::TOuterContext<_TyTimerSamplerRef>(b_result, timer,
				omega_slim, required_column_list, r_prev_marginals, r_lambda_in_natural_order,
				r_R, mord, n_matrix_part));
			// entere decision tree

			return b_result;*/
		} // not proficient; t_odo - make a fake block matrix to hold all the dense mats (Tu. Bu, S, ...) so that they are aligned!
		// handle FBS processing (one dimension of Tu and Bu, and both dimensions
		// of both matrices are known at compile-time)

		Eigen::MatrixXd omega_dense;
		omega_slim.Convert_to_Dense(omega_dense);
		// get dense omega

#ifdef _DEBUG
		/*double f_omega_diag_min_abs_coeff = omega_dense.diagonal().array().abs().minCoeff();
		printf("debug: min diag coeff in omega: %g\n", f_omega_diag_min_abs_coeff);*/
#endif // _DEBUG

		//omega_dense.bottomRightCorner(omega_dense.rows() - omega_slim.n_BlockColumn_Column_Num(0),
		//	omega_dense.cols() - omega_slim.n_BlockColumn_Column_Num(0)).diagonal().array() += .1;
		// slightly lift the diagonal (except the first vertex), like it would happen in damped least squares

		_TySample f_omega_time = f_omega_build_time;
		_TySample f_dense_margs_time = 0;
		_TySample f_update_basis_time = 0;
		_TySample f_update_time = 0;
		_TySample f_extend_time = 0;

		timer.Accum_DiffSample(f_omega_time);

		const size_t n_cur_state_size = lambda.n_Column_Num();
		const size_t n_prev_state_size = r_prev_marginals.n_Column_Num();
		const size_t n_prev_state_block_num = r_prev_marginals.n_BlockColumn_Num();
		const size_t n_omega_elems = omega_slim.n_Column_Num();

		Eigen::MatrixXd Tu_full(n_cur_state_size, n_omega_elems); // massif: 35,897,688B // do not allocate it smaller, will need these to update the new covs!
		_ASSERTE(n_packed_block_column_num <= INT_MAX);
		int _n_packed_block_column_num = int(n_packed_block_column_num);
		#pragma omp parallel for if(n_prev_state_block_num > 1000)
		for(int i = 0; i < _n_packed_block_column_num; ++ i) { // t_odo - could run in parallel, but usually needs like two to six columns (threads)
			size_t n_block_base_margs = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
			size_t n_block_base_Tu = omega_slim.n_BlockColumn_Base(i);
			size_t n_block_cols = omega_slim.n_BlockColumn_Column_Num(i);
			// get dimensions of this block

			Eigen::Block<Eigen::MatrixXd> Tu_block =
				Tu_full.block(0, n_block_base_Tu, n_cur_state_size, n_block_cols); // g++ requires a temporary
			CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<
				typename CSystemType::_TyHessianMatrixBlockList>(Tu_block
				/*Tu_full.block(0, n_block_base_Tu, n_cur_state_size, n_block_cols)*/,
				r_R, n_block_base_margs, mord.p_Get_InverseOrdering(),
				mord.n_Ordering_Size(), mord.n_Ordering_Size()/*n_prev_state_block_num*/);
			// really calculate a block of dense marginals
		}
		Eigen::Block<Eigen::MatrixXd> Tu = Tu_full.topRows/*LeftCorner*/(n_prev_state_size/*, n_omega_elems*/);
		// assemble Tu

		Eigen::MatrixXd s(n_omega_elems, n_omega_elems);
		for(size_t i = 0; i < n_packed_block_column_num; ++ i) {
			size_t n_block_base_row_Tu = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
			size_t n_block_base_row_s = omega_slim.n_BlockColumn_Base(i);
			size_t n_block_rows = omega_slim.n_BlockColumn_Column_Num(i); // is symmetric
			// get dimensions of this block

			if(n_block_base_row_Tu < n_prev_state_size) {
				s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems) =
					Tu.block(n_block_base_row_Tu, 0, n_block_rows, n_omega_elems);
			} else
				s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems).setZero();
			// copy block from Tu to s
		}
		// cut out s (could be performed inside a sparse block matrix
		// multiplication, except now we don't have the data in a block matrix)

		timer.Accum_DiffSample(f_dense_margs_time);

/*#ifdef _DEBUG
		Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> llt(omega_dense);
		if(llt.info() == Eigen::Success)
			printf("debug: was able to use the first LL^T Cholesky\n"); // does this ever happen? it does, when one is very certain, typically when the covariance associated with the edges is high
#endif // _DEBUG*/

		omega_dense.triangularView<Eigen::StrictlyLower>() =
			omega_dense.triangularView<Eigen::StrictlyUpper>().transpose();
		// need both halves! (block matrix omega doesn't contain its lower triangular part)

		Eigen::MatrixXd V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) - s * omega_dense;
		// calculate V

#ifdef _DEBUG
		/*double f_V_diag_min_abs_coeff = V.diagonal().array().abs().minCoeff();
		printf("debug: min diag coeff in S: %g (will invert it)\n", f_V_diag_min_abs_coeff); // in the video this is called S*/
#endif // _DEBUG

		// t_odo - do more manual inversion (with LU?) of V,
		// in case it is not invertible, refrain to use batch marginals

		Eigen::FullPivLU<Eigen::MatrixXd> luV(V);
		if(!luV.isInvertible())
			return false; // that's a runtime err

		//Eigen::MatrixXd Bu = ((omega_dense * luV.inverse()) * Tu.transpose());
		// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
		// of the product mutliply Tu (that way at least the computation is saved, if not storage)

		typedef forward_allocated_pool<double, 0, 64> CAlloc;
		const size_t n_omega_stride = n_Align_Up_POT(n_omega_elems, size_t(8));
		double *p_bu_buffer = (double*)CAlloc::aligned_alloc(n_omega_stride * n_prev_state_size * sizeof(double)); // massif: +73,886,208B/2
		Eigen::Map<Eigen::MatrixXd, Eigen::Aligned, Eigen::OuterStride<> > Bu(p_bu_buffer, n_omega_elems,
			n_prev_state_size, Eigen::OuterStride<>(n_omega_stride)); // why did I do that as a map? to have stride?
		// t_odo - try to increase n_omega_elems to the next multiple of 8

#ifdef __MARGINALS_COMPACT_UPDATE
		// ommit TuT

		Bu.noalias() = (omega_dense * luV.inverse()) * Tu.transpose(); // a suspicious matrix product, this is probably where massif detects +35,893,584B +35,893,584B
#else // __MARGINALS_COMPACT_UPDATE
		double *p_tut_buffer = (double*)CAlloc::aligned_alloc(n_omega_stride * n_prev_state_size * sizeof(double)); // massif: +73,886,208B/2
		Eigen::Map<Eigen::MatrixXd, Eigen::Aligned, Eigen::OuterStride<> > TuT(p_tut_buffer, n_omega_elems,
			n_prev_state_size, Eigen::OuterStride<>(n_omega_stride));
		TuT = Tu.transpose(); // keep this as well, for a better memory locality

		Bu.noalias() = (omega_dense * luV.inverse()) * TuT; // a suspicious matrix product, this is probably where massif detects +35,893,584B +35,893,584B
#endif // __MARGINALS_COMPACT_UPDATE
		// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
		// of the product mutliply Tu (that way at least the computation is saved, if not storage)

		timer.Accum_DiffSample(f_update_basis_time);

		_ASSERTE(//n_matrix_part == mpart_Nothing || // handled above
				 n_matrix_part == mpart_FullMatrix || // does not combine with anything (is addition idempotent)
				 n_matrix_part == mpart_LastBlock || // does not combine with mpart_LastColumn or mpart_Diagonal
				 n_matrix_part == mpart_LastColumn ||
				 n_matrix_part == mpart_Diagonal ||
				 n_matrix_part == (mpart_LastColumn | mpart_Diagonal));
		// there are only a few choices of what one can compose out of mpart_*

		const bool b_diag_update =
			//n_matrix_part == mpart_FullMatrix || // no, need to update everything else
			n_matrix_part == mpart_LastBlock || // yes, but only in the last column
			//n_matrix_part == mpart_LastColumn || // no
			n_matrix_part == mpart_Diagonal || // yes
			n_matrix_part == (mpart_LastColumn | mpart_Diagonal); // yes
		const bool b_diag_only_last = n_matrix_part == mpart_LastBlock;
		_ASSERTE(!b_diag_only_last || b_diag_update); // only set if b_diag_update also set
		// diagonal-only update

		const bool b_full_update =
			n_matrix_part == mpart_FullMatrix || // yes
			//n_matrix_part == mpart_LastBlock || // no
			n_matrix_part == mpart_LastColumn || // yes, but only in the last column
			//n_matrix_part == mpart_Diagonal || // no
			n_matrix_part == (mpart_LastColumn | mpart_Diagonal); // yes, but only in the last column
		const bool b_full_only_last = (n_matrix_part & mpart_LastColumn) == mpart_LastColumn;
		_ASSERTE(!b_full_only_last || b_full_update); // only set if b_diag_update also set
		// full update

		const size_t n_new_vertex_num = lambda.n_BlockColumn_Num() - n_prev_state_block_num;
		const bool b_last_col_is_part_of_interior = !n_new_vertex_num;
		// the last column is a part of the inside of the old marginals

		_ASSERTE(b_full_update || b_diag_update); // at least one is set
		const bool b_update_interior = (b_diag_update && !b_diag_only_last) ||
			(b_full_update && !b_full_only_last) || b_last_col_is_part_of_interior;
		// do we need to update the interior of the matrix?

		_ASSERTE(!b_full_only_last || !b_diag_only_last); // at most one is set
		_ASSERTE(!(b_diag_only_last && b_full_update)); // never at the same time; if b_diag_only_last is set, full can't be set

		bool b_purged = false;
		size_t n_max_size = 0;

		if(b_update_interior) {
			const size_t n_news_stat_block_num = lambda.n_BlockColumn_Num();
			_ASSERTE(n_prev_state_block_num <= INT_MAX);
			int _n_prev_state_block_num = int(n_prev_state_block_num);
			int n_first = 0;
			if((b_full_update && b_full_only_last && !b_diag_update) ||
			   (b_diag_update && b_diag_only_last && !b_full_update)) // note thet both b_full_only_last and b_diag_only_last can't be set at the same time
				n_first = _n_prev_state_block_num - 1; // start at the last column
			// decide the interior update range

			CUberBlockMatrix new_margs_tmp;
			bool b_update_to_temporary = ((n_matrix_part & mpart_LastColumn) ==
				mpart_LastColumn && !b_last_col_is_part_of_interior) &&
				n_matrix_part != mpart_FullMatrix/*!b_full_update*/; // b_full_update means something else
			// decide whether to update to a temporary matrix

			if(b_update_to_temporary) {
				if(r_prev_marginals.n_Storage_Size() / 2 < (n_max_size = std::max(size_t(1048576), r_R.n_Storage_Size()))) // n_Storage_Size() is in elements, not bytes // could align up to whole blocks, like this it sometimes deletes right after allocation
					b_update_to_temporary = false; // don't want to use it, the relinearization will take care of throwing away the dense parts
				else {
					//printf("purge\n");
					b_purged = true;
				}
			} /*else
				printf("cannot purge\n");*/
			// only allow updating to a temporary if the marginals matrix takes much more space than the factor

			bool b_run_in_parallel = true;
			if(b_update_to_temporary) { // note that this is probably slightly more costy as it is doing M' = M + U istead of M += U
				//if(!n_first)
					r_prev_marginals.CopyLayoutTo(new_margs_tmp);
				/*else
					new_margs_tmp.ExtendTo(r_prev_marginals.n_Row_Num(), r_prev_marginals.n_Column_Num());*/
				// don't, the marginals are indexed by vertex id's (well not outside of this function or at least yet),
				// need to avoid having empty structure there
				// (but it's a bummer as it would be the perfect use case for the empty structure inside a matrix)
				// it could be handled here using a branch, but it would get impossible to handle updates
				// from that matrix, everyone would need to use row-col lookups
				// actually with some care it would not be difficult in the new vertex loop
				// and this loop could mostly use the existing structure of the columns (which will lead to
				// unbalanced OpenMP processing if only the last column is needed, as the rest of the columns
				// will be empty)
				// but, it would make the code (even more) messy and our use cases don't involve last
				// block / last column only (yet?)
				// todo

				if(1) {
					for(int i = n_first; i < _n_prev_state_block_num; ++ i) {
						const size_t n_cols = new_margs_tmp.n_BlockColumn_Column_Num(i);
						new_margs_tmp.t_GetBlock_Log(i, i, n_cols, n_cols, true, false); // massif: 8,388,608B
					}
					// preallocate structure
				} else
					b_run_in_parallel = false; // in case the structure is not allocated, would cause conflicts
			}
			// allocate structure in the temporary matrix (to save time when looking blocks up)

			#pragma omp parallel for if(b_run_in_parallel && n_prev_state_block_num - n_first > 1000)
			for(int i = n_first; i < _n_prev_state_block_num; ++ i) { // t_odo - this needs to run in parallel
				const size_t n_cols = r_prev_marginals.n_BlockColumn_Column_Num(i);
				const size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);
				const bool b_last_col = size_t(i) + 1 == n_news_stat_block_num; // new state, not the old one!
				if(!b_full_update || (b_full_only_last && !b_last_col)) { // if not full, then diagonal (must be doing something)
					_ASSERTE(!b_diag_only_last || b_last_col); // if we want to do only the last column, make sure we don't waste time on other columns
#if 0
					fbs_ut::CWrap2<CMarginalsUpdate_FBSKernel::TInnerLoop,
						fbs_ut::CCTSize2D<n_omega_elems, -1> >::template
						In_ColumnWidth_DecisionTree<_TyHessianMatrixBlockList>(int(n_cols),
						CMarginalsUpdate_FBSKernel::TInnerContext(
						CMarginalsUpdate_FBSKernel::TMiddleContext(r_prev_marginals, i, n_cur_state_size,
						n_prev_state_size, p_tut_buffer/*Tu_full.data()*/,
						p_bu_buffer/*Bu.data()*/), i, n_col_base, n_col_base));
#endif // 0
					// n_omega_elems is not constant, would have to rewrite this a bit (it wouldn't be completely FBS)

#ifdef _DEBUG
					if(!b_update_to_temporary) {
						for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
							if(r_prev_marginals.n_Block_Row(i, j) == i)
								continue; // not the diag block
							r_prev_marginals.t_Block_AtColumn(i, j).setZero();
						}
					}
#endif // _DEBUG
					// zero out other blocks so that it is obvious which parts are up to date (debug)

					if(b_update_to_temporary) {
#ifdef __MARGINALS_COMPACT_UPDATE
						new_margs_tmp.t_GetBlock_Log(i, i, n_cols, n_cols, true, false) =
							r_prev_marginals.t_GetBlock_Log(i, i) - Tu.middleRows(n_col_base,
							n_cols).lazyProduct(Bu.middleCols(n_col_base, n_cols));
#else // __MARGINALS_COMPACT_UPDATE
						new_margs_tmp.t_GetBlock_Log(i, i, n_cols, n_cols, true, false) =
							r_prev_marginals.t_GetBlock_Log(i, i) - TuT.middleCols(n_col_base,
							n_cols).transpose().lazyProduct(Bu.middleCols(n_col_base, n_cols));
#endif // __MARGINALS_COMPACT_UPDATE
					} else {
#ifdef __MARGINALS_COMPACT_UPDATE
						r_prev_marginals.t_GetBlock_Log(i, i) -= Tu.middleRows(n_col_base,
							n_cols).lazyProduct(Bu.middleCols(n_col_base, n_cols));
#else // __MARGINALS_COMPACT_UPDATE
						r_prev_marginals.t_GetBlock_Log(i, i) -= TuT.middleCols(n_col_base,
							n_cols).transpose().lazyProduct(Bu.middleCols(n_col_base, n_cols));
#endif // __MARGINALS_COMPACT_UPDATE
					}
					// maybe it would be faster using Tu_full? it makes no difference here.
					// t_odo - could FBS these products*/
				} else { // update all the existing blocks in the current column
					_ASSERTE(!b_full_only_last || b_last_col); // if we want to do only the last column, make sure we don't waste time on other columns
#if 0
					typedef typename MakeTypelist(fbs_ut::CCTSize<n_omega_elems>,
						_TyHessianMatrixBlockList) _TySecondaryContext;
					fbs_ut::CWrap2<CMarginalsUpdate_FBSKernel::TMiddleLoop, _TySecondaryContext>::template
						In_ColumnWidth_DecisionTree<_TyHessianMatrixBlockList>(int(n_cols),
						CMarginalsUpdate_FBSKernel::TMiddleContext(r_prev_marginals, i, n_cur_state_size,
						n_prev_state_size, p_tut_buffer/*Tu_full.data()*/, p_bu_buffer/*Bu.data()*/));
#endif // 0
					// n_omega_elems is not constant, would have to rewrite this a bit (it wouldn't be completely FBS)

					for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
						size_t n_row = r_prev_marginals.n_Block_Row(i, j);
						size_t n_row_base = r_prev_marginals.n_BlockColumn_Base(n_row);
						size_t n_rows = r_prev_marginals.n_BlockColumn_Column_Num(n_row); // is symmetric

						_ASSERTE(!b_update_to_temporary); // would copy the whole matrix anyway
						/*if(b_update_to_temporary) {
#ifdef __MARGINALS_COMPACT_UPDATE
							new_margs_tmp.t_GetBlock_Log(n_row, i, n_rows, n_cols, true, false) =
								r_prev_marginals.t_Block_AtColumn(i, j) - Tu.middleRows(n_row_base,
								n_rows).lazyProduct(Bu.middleCols(n_col_base, n_cols));
#else // __MARGINALS_COMPACT_UPDATE
							new_margs_tmp.t_GetBlock_Log(n_row, i, n_rows, n_cols, true, false) =
								r_prev_marginals.t_Block_AtColumn(i, j) - TuT.middleCols(n_row_base,
								n_rows).transpose().lazyProduct(Bu.middleCols(n_col_base, n_cols));
#endif // __MARGINALS_COMPACT_UPDATE
						} else*/ {
#ifdef __MARGINALS_COMPACT_UPDATE
							r_prev_marginals.t_Block_AtColumn(i, j) -= Tu.middleRows(n_row_base,
								n_rows).lazyProduct(Bu.middleCols(n_col_base, n_cols));
#else // __MARGINALS_COMPACT_UPDATE
							r_prev_marginals.t_Block_AtColumn(i, j) -= TuT.middleCols(n_row_base,
								n_rows).transpose().lazyProduct(Bu.middleCols(n_col_base, n_cols));
#endif // __MARGINALS_COMPACT_UPDATE
							//r_prev_marginals.t_Block_AtColumn(i, j) -=
							//	Tu_full.block(n_row_base, 0, n_rows, n_omega_elems).lazyProduct(
							//	Bu.block(0, n_col_base, n_omega_elems, n_cols)); // maybe it would be faster using Tu_full? it makes no difference here.
							// could FBS these products
						}
					}
				}
			}

			if(b_update_to_temporary)
				new_margs_tmp.Swap(r_prev_marginals);
			// t_odo: note that if mpart_LastColumn is selected, the matrix actually fills up to completely
			// dense if that is the case, we need to rewrite the loop to actually add to a different matrix
			// and discard the original one!

			// also note that leaving out-of-date blocks in the marginals will add on the complexity of deciding
			// whether or not to compute the on-demand updates (will have to see whether a block would have been
			// updated in the past, and if not, compute it - this will make the on-demand recursive marginals
			// calculation from R more complex as that one needs to know what blocks to use in the recursive
			// formula)
		} else {
			// the interior not updated, may as well cleanup
			r_prev_marginals.SetZero(); // not Clear(), that would delete the layout also
		}
		// update (actually "minus downdate") the existing blocks; in order to check,
		// however, need to update all the blocks that are already there

		_ASSERTE(r_prev_marginals.n_BlockColumn_Num() + n_new_vertex_num == lambda.n_BlockColumn_Num());
		// it is not extended yet

		timer.Accum_DiffSample(f_update_time);

		if(!b_last_col_is_part_of_interior) {
			size_t n_first = 0;
			if((b_full_update && b_full_only_last && !b_diag_update) ||
			   (b_diag_update && b_diag_only_last && !b_full_update)) // note thet both b_full_only_last and b_diag_only_last can't be set at the same time
				n_first = n_new_vertex_num - 1; // start at the last column
			// decide the interior update range

			for(size_t i = n_first; i < n_new_vertex_num; ++ i) { // can't run in parallel, changes the matrix layout (and is typically quite small)
				size_t n_vertex = n_prev_state_block_num + i;
				size_t n_dim = lambda.n_BlockColumn_Column_Num(n_vertex);

				size_t n_block_col_in_Tu = std::find(required_column_list.begin(),
					required_column_list.end(), n_vertex) - required_column_list.begin(); // lower bound?
				size_t n_col_in_Tu = omega_slim.n_BlockColumn_Base(n_block_col_in_Tu);
				size_t n_row_in_Tu = lambda.n_BlockColumn_Base(n_vertex);
				// see which vertex it is in Tu

				/*r_prev_marginals.ExtendTo(r_prev_marginals.n_Row_Num() + n_dim,
					r_prev_marginals.n_Column_Num() + n_dim);
				// debug - just enlarge, no new blocks*/

				const bool b_last_col = i + 1 == n_new_vertex_num;
				if(!b_full_update || (b_full_only_last && !b_last_col)) { // if not full, then diagonal (must be doing something)
					_ASSERTE(!b_diag_only_last || b_last_col); // if we want to do only the last column, make sure we don't waste time on other columns
					// fill in the diagonal block

					r_prev_marginals.t_GetBlock_Log(n_vertex, n_vertex, n_dim, n_dim, true, false) =
						Tu_full.block(n_row_in_Tu, n_col_in_Tu, n_dim, n_dim); // amazingly correct
					// need Tu_full, the blocks are not present in just Tu
				} else { // the last column (on the last column)
					_ASSERTE(!b_full_only_last || b_last_col); // if we want to do only the last column, make sure we don't waste time on other columns
					// fill in the whole column

					for(size_t j = 0, m = lambda.n_BlockColumn_Num(); j < m; ++ j) { // note that the other blocks presumably calculated from bottom to top
						size_t n_rows = lambda.n_BlockColumn_Column_Num(j); // should be row, but lambda is symmetric and this is in cache
						size_t n_row_in_Tu = lambda.n_BlockColumn_Base(j);
						r_prev_marginals.t_GetBlock_Log(j, n_vertex, n_rows, n_dim, true, false) = // massif: 33,554,432B
							Tu_full.block(n_row_in_Tu, n_col_in_Tu, n_rows, n_dim); // massif: 13,107,200B
					}
					// for each block in the column
				}
			}
		}
		// put there the new blocks
		// note that instead of updating what's between Tu_full, we could replace it for better
		// precision (or maybe instability, when mixed with the updated parts - that depends)

		CAlloc::aligned_free(p_bu_buffer);
#ifndef __MARGINALS_COMPACT_UPDATE
		CAlloc::aligned_free(p_tut_buffer);
#endif // !__MARGINALS_COMPACT_UPDATE
		// should free in catch(bad_alloc) as well, will leave memory allocated if something throws

		timer.Accum_DiffSample(f_extend_time);

		_TySample f_total_time = 0;
		timer.Accum_CumTime_LastSample(f_total_time);

		/*if(b_purged && r_prev_marginals.n_Storage_Size() / 2 > n_max_size)
			printf("failed purge, the matrix still takes more\n");*/

		if(b_enable_timer) {
			printf("marginals update took %.5f msec\n", f_total_time * 1000.0);
			printf("\tomega: %.5f msec\n", f_omega_time * 1000.0);
			printf("\tTu, s: %.5f msec\n", f_dense_margs_time * 1000.0);
			printf("\tbasis: %.5f msec\n", f_update_basis_time * 1000.0);
			printf("\t  upd: %.5f msec\n", f_update_time * 1000.0);
			printf("\t  ext: %.5f msec\n", f_extend_time * 1000.0);
		}

		return true;
	}

	/**
	 *	@brief incrementally updates sparse blocky marginals on diagonal or everywhere
	 *
	 *	@tparam b_enable_timer is timing enable flag (if disabled, incurs no runtime costs)
	 *	@tparam CSystemType is optimized system type (for optimized FBS calls)
	 *
	 *	@param[in] system is reference to the optimized system
	 *	@param[in,out] r_prev_marginals is reference to the marginals to be updated (must be in natural order)
	 *	@param[in] r_lambda_in_natural_order is reference to the lambda matrix (must be in natural order)
	 *	@param[in] r_R is reference to ordered Cholesky factorization of r_lambda_in_natural_order
	 *	@param[in] mord is reference to the ordering, used in the Cholesky factorization of r_R
	 *	@param[in] n_edges_in_prev_marginals is number of edges in r_prev_marginals
	 *	@param[in] b_update_diag_only is diagonal update flag (if set, only the diagonal is updated)
	 *
	 *	@return Returns true on success, false on numerical issue
	 *		(in that case, r_prev_marginals is not modified).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	// *	@note This is only available if __SE_TYPES_SUPPORT_L_SOLVERS is defined (the Omega matrix is used).
	template <bool b_enable_timer, class CSystemType>
	static bool Update_BlockDiagonalMarginals(const CSystemType &system, CUberBlockMatrix &r_prev_marginals,
		const CUberBlockMatrix &r_lambda_in_natural_order, const CUberBlockMatrix &r_R,
		const CMatrixOrdering &mord, size_t n_edges_in_prev_marginals, bool b_update_diag_only = false) // throw(std::bad_alloc)
	{
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSampler _TyTimerSampler;
		typedef typename CTimerSamplerTraits<b_enable_timer>::_TyTimerSamplerRef _TyTimerSamplerRef;
		typedef typename _TyTimerSampler::_TySample _TySample;
		// choose a timer based on whether it is enabled or not

		CTimer t;
		_TyTimerSampler timer(t);

		const CUberBlockMatrix &lambda = r_lambda_in_natural_order; // rename
		const size_t n_prev_edge_num = n_edges_in_prev_marginals;

		size_t n_edge_num = system.r_Edge_Pool().n_Size();
		size_t n_order_min = r_prev_marginals.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = r_prev_marginals.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)

		CUberBlockMatrix omega; // todo - this might be up to the linear solver, in LM omega will be different
		std::vector<size_t> required_column_list;
		required_column_list.reserve(2 * (n_edge_num - n_prev_edge_num)); // a guess; might be edges with more or less than 2 verts
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) { // not parallel! (wouls have conflicts)
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			r_edge.Calculate_Omega(omega, n_elem_order_min);
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				required_column_list.push_back(r_edge.n_Vertex_Id(j));
		}
		// get omega and vertex id's

		std::sort(required_column_list.begin(), required_column_list.end());
		required_column_list.erase(std::unique(required_column_list.begin(),
			required_column_list.end()), required_column_list.end());
		// finalize the required column list (could use std::set, but that feels like overkill)

		std::vector<size_t> pack_order;
		pack_order.reserve(omega.n_BlockColumn_Num());
		for(size_t i = 0, n = required_column_list.size(); i < n; ++ i) {
			size_t n_col = required_column_list[i];
			size_t n_base = lambda.n_BlockColumn_Base(n_col) - n_elem_order_min;
			size_t n_col_size;
			size_t n_col_omega = omega.n_Find_BlockColumn(n_base, n_col_size);
			_ASSERTE(n_col_size == lambda.n_BlockColumn_Column_Num(n_col));

			pack_order.push_back(n_col_omega);
		}
		// get numbers of "interesting" columns of omega (no ordering here either)

		const size_t n_packed_block_column_num = pack_order.size();
		// remember this

		for(size_t i = 1; i < n_packed_block_column_num; ++ i) {
			size_t n_o0 = pack_order[i - 1];
			size_t n_o1 = pack_order[i];
			_ASSERTE(n_o1 > n_o0); // should be sorted

			for(size_t j = n_o0 + 1; j < n_o1; ++ j)
				pack_order.push_back(j);
			// append the rest of the cols at the end (only up to omega.n_BlockColumn_Num() / 2 of them)
		}
		_ASSERTE(pack_order.size() == omega.n_BlockColumn_Num());
		// finalize the order

		std::vector<size_t> inv_pack_order(pack_order.size());
		for(size_t i = 0, n = pack_order.size(); i < n; ++ i)
			inv_pack_order[pack_order[i]] = i;
		// inverse the order

		CUberBlockMatrix omega_slim;
		omega.Permute_UpperTriangular_To(omega_slim,
			&inv_pack_order[0], inv_pack_order.size(), true);
		// pack omega

		omega_slim.SliceTo(omega_slim, n_packed_block_column_num,
			n_packed_block_column_num, true);
		_ASSERTE(omega_slim.n_BlockColumn_Num() == n_packed_block_column_num);
		// slice off the empty columns

		Eigen::MatrixXd omega_dense;
		omega_slim.Convert_to_Dense(omega_dense);
		// get dense omega

		_TySample f_omega_time = 0;
		_TySample f_dense_margs_time = 0;
		_TySample f_update_basis_time = 0;
		_TySample f_update_time = 0;
		_TySample f_extend_time = 0;

		timer.Accum_DiffSample(f_omega_time);

		const size_t n_cur_state_size = lambda.n_Column_Num();
		const size_t n_prev_state_size = r_prev_marginals.n_Column_Num();
		const size_t n_prev_state_block_num = r_prev_marginals.n_BlockColumn_Num();
		const size_t n_omega_elems = omega_slim.n_Column_Num();

		Eigen::MatrixXd Tu_full(n_cur_state_size, n_omega_elems); // do not allocate it smaller, will need these to update the new covs!
		_ASSERTE(n_packed_block_column_num <= INT_MAX);
		int _n_packed_block_column_num = int(n_packed_block_column_num);
		#pragma omp parallel for if(n_prev_state_block_num > 1000)
		for(int i = 0; i < _n_packed_block_column_num; ++ i) { // t_odo - could run in parallel, but usually needs like two to six columns (threads)
			size_t n_block_base_margs = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
			size_t n_block_base_Tu = omega_slim.n_BlockColumn_Base(i);
			size_t n_block_cols = omega_slim.n_BlockColumn_Column_Num(i);
			// get dimensions of this block

			CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<
				typename CSystemType::_TyHessianMatrixBlockList>(
				Tu_full.block(0, n_block_base_Tu, n_cur_state_size, n_block_cols),
				r_R, n_block_base_margs, mord.p_Get_InverseOrdering(),
				mord.n_Ordering_Size(), mord.n_Ordering_Size()/*n_prev_state_block_num*/);
			// really calculate a block of dense marginals
		}
		Eigen::Block<Eigen::MatrixXd> Tu = Tu_full.topLeftCorner(n_prev_state_size, n_omega_elems);
		// assemble Tu

		Eigen::MatrixXd s(n_omega_elems, n_omega_elems);
		for(size_t i = 0; i < n_packed_block_column_num; ++ i) {
			size_t n_block_base_row_Tu = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
			size_t n_block_base_row_s = omega_slim.n_BlockColumn_Base(i);
			size_t n_block_rows = omega_slim.n_BlockColumn_Column_Num(i); // is symmetric
			// get dimensions of this block

			if(n_block_base_row_Tu < n_prev_state_size) {
				s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems) =
					Tu.block(n_block_base_row_Tu, 0, n_block_rows, n_omega_elems);
			} else
				s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems).setZero();
			// copy block from Tu to s
		}
		// cut out s (could be performed inside a sparse block matrix
		// multiplication, except now we don't have the data in a block matrix)

		timer.Accum_DiffSample(f_dense_margs_time);

#ifdef _DEBUG
		Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> llt(omega_dense);
		if(llt.info() == Eigen::Success)
			printf("debug: was able to use the first LL^T Cholesky\n"); // does this ever happen?
#endif // _DEBUG

		omega_dense.triangularView<Eigen::StrictlyLower>() =
			omega_dense.triangularView<Eigen::StrictlyUpper>().transpose();
		// need both halves! (block matrix omega doesn't contain its lower triangular part)

		Eigen::MatrixXd V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) - s * omega_dense;
		// calculate V

		// t_odo - do more manual inversion (with LU?) of V,
		// in case it is not invertible, refrain to use batch marginals

		Eigen::FullPivLU<Eigen::MatrixXd> luV(V);
		if(!luV.isInvertible())
			return false; // that's a runtime err

		Eigen::MatrixXd Bu = ((omega_dense * luV.inverse()) * Tu.transpose());
		// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
		// of the product mutliply Tu (that way at least the computation is saved, if not storage)

		timer.Accum_DiffSample(f_update_basis_time);

		_ASSERTE(n_prev_state_block_num <= INT_MAX);
		int _n_prev_state_block_num = int(n_prev_state_block_num);
		#pragma omp parallel for if(n_prev_state_block_num > 1000)
		for(int i = 0; i < _n_prev_state_block_num; ++ i) { // t_odo - this needs to run in parallel
			size_t n_col_base = r_prev_marginals.n_BlockColumn_Base(i);
			size_t n_cols = r_prev_marginals.n_BlockColumn_Column_Num(i);
			if(b_update_diag_only) {
				r_prev_marginals.t_GetBlock_Log(i, i) -=
					Tu_full.block(n_col_base, 0, n_cols, n_omega_elems).lazyProduct(
					Bu.block(0, n_col_base, n_omega_elems, n_cols)); // maybe it would be faster using Tu_full? it makes no difference here.
				// could FBS these products
			} else {
				for(size_t j = 0, m = r_prev_marginals.n_BlockColumn_Block_Num(i); j < m; ++ j) {
					size_t n_row = r_prev_marginals.n_Block_Row(i, j);
					size_t n_row_base = r_prev_marginals.n_BlockColumn_Base(n_row);
					size_t n_rows = r_prev_marginals.n_BlockColumn_Column_Num(n_row); // is symmetric

					r_prev_marginals.t_Block_AtColumn(i, j) -=
						Tu_full.block(n_row_base, 0, n_rows, n_omega_elems).lazyProduct(
						Bu.block(0, n_col_base, n_omega_elems, n_cols)); // maybe it would be faster using Tu_full? it makes no difference here.
					// could FBS these products
				}
			}
		}
		// update (actually "minus downdate") the existing blocks; in order to check,
		// however, need to update all the blocks that are already there

		timer.Accum_DiffSample(f_update_time);

		size_t n_new_vertex_num = lambda.n_BlockColumn_Num() - n_prev_state_block_num;
		for(size_t i = 0; i < n_new_vertex_num; ++ i) { // can't run in parallel, changes the matrix layout
			size_t n_vertex = n_prev_state_block_num + i;
			size_t n_dim = lambda.n_BlockColumn_Column_Num(n_vertex);

			size_t n_block_col_in_Tu = std::find(required_column_list.begin(),
				required_column_list.end(), n_vertex) - required_column_list.begin();
			size_t n_col_in_Tu = omega_slim.n_BlockColumn_Base(n_block_col_in_Tu);
			size_t n_row_in_Tu = lambda.n_BlockColumn_Base(n_vertex);
			// see which vertex it is in Tu

			/*r_prev_marginals.ExtendTo(r_prev_marginals.n_Row_Num() + n_dim,
				r_prev_marginals.n_Column_Num() + n_dim);
			// debug - just enlarge, no new blocks*/

			r_prev_marginals.t_GetBlock_Log(n_vertex, n_vertex, n_dim, n_dim, true, false) =
				Tu_full.block(n_row_in_Tu, n_col_in_Tu, n_dim, n_dim); // amazingly correct
			// need Tu_full, the blocks are not present in just Tu
		}
		// put there the new blocks

		timer.Accum_DiffSample(f_extend_time);

		_TySample f_total_time = 0;
		timer.Accum_CumTime_LastSample(f_total_time);

		if(b_enable_timer) {
			printf("marginals update took %.5f msec\n", f_total_time * 1000.0);
			printf("\tomega: %.5f msec\n", f_omega_time * 1000.0);
			printf("\tTu, s: %.5f msec\n", f_dense_margs_time * 1000.0);
			printf("\tbasis: %.5f msec\n", f_update_basis_time * 1000.0);
			printf("\t  upd: %.5f msec\n", f_update_time * 1000.0);
			printf("\t  ext: %.5f msec\n", f_extend_time * 1000.0);
		}

		return true;
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS || 1

	/**
	 *	@brief performs some timing benchmark of updating marginals on a SE(2) system
	 *
	 *	@tparam CEdgeType is edge type to be added
	 *	@tparam CSystemType is optimized system type
	 *	@tparam CSolverType is solver type
	 *
	 *	@param[in,out] system is reference to the optimized system (gets extended by some edges / vertices)
	 *	@param[in,out] solver is reference to the nonlinear solver
	 *	@param[in] information is information matrix for the new edges
	 *
	 *	@note This reuses some of CMarginals functions to calculate / update the marginals.
	 */
	template <class CEdgeType, class CSystemType, class CSolverType>
	static void Test_ExperimentalIncrementalMarginals(CSystemType &system,
		CSolverType &solver, Eigen::Matrix3d information)
	{
		//system.r_Add_Edge(CEdgePose2D(83, 101, Eigen::Vector3d(-1.97704, -0.00703316, -3.16668), information, system));
		//solver.Optimize(); // want to have 101x101 system to begin with, to make S symmetric

		CUberBlockMatrix lambda_prev = solver.r_Lambda();
		// get a *copy* of lambda

		size_t n_prev_edge_num = system.r_Edge_Pool().n_Size();
		// remember how many edges there are

		system.r_Add_Edge(CEdgeType(83, 101, Eigen::Vector3d(-1.97704, -0.00703316, -3.16668), information, system));
		system.r_Add_Edge(CEdgeType(100, 101, Eigen::Vector3d(1.02465, 0.0317282, 0.024041), information, system));
		//system.r_Add_Edge(CEdgeType(96, 101/*102*/, Eigen::Vector3d(0.007666, 0.00487275, -3.16637), information, system)); // loop without changing dims (not present in the original dataset)
		system.r_Add_Edge(CEdgeType(96, 102, Eigen::Vector3d(0.007666, 0.00487275, -3.16637), information, system));
		system.r_Add_Edge(CEdgeType(96, 102, Eigen::Vector3d(0.0105401, -0.00932236, -3.11433), information, system));
		system.r_Add_Edge(CEdgeType(101, 102, Eigen::Vector3d(0.99396, 0.0202571, -0.00240724), information, system));
		// add some more edges

		solver.Optimize(0); // must call that to calculate hessians in the new lambda
		const CUberBlockMatrix &lambda = solver.r_Lambda();
		// get system matrix (without relinearizing!)

		/*{
			lambda_prev.Save_MatrixMarket("lambda_prev.mtx", "lambda_prev.bla");
			lambda.Save_MatrixMarket("lambda_cur.mtx", "lambda_cur.bla");
		}*/
		// for Ela

		size_t n_edge_num = system.r_Edge_Pool().n_Size();
		size_t n_order_min = lambda.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = lambda.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)

		CUberBlockMatrix cur_R_ord;
		CMatrixOrdering mord;
		mord.p_BlockOrdering(lambda, true);
		{
			const size_t *p_order = mord.p_Get_InverseOrdering();
			CUberBlockMatrix lambda_perm;
			lambda.Permute_UpperTriangular_To(lambda_perm, p_order, mord.n_Ordering_Size(), true);
			cur_R_ord.CholeskyOf_FBS<typename CSystemType::_TyHessianMatrixBlockList>(lambda_perm);
		}
		// take Cholesky (unordered so it is easier to compute the margs)

		CUberBlockMatrix prev_R_ord;
		CMatrixOrdering mord_prev;
		mord_prev.p_BlockOrdering(lambda_prev, true);
		{
			const size_t *p_order = mord_prev.p_Get_InverseOrdering();
			CUberBlockMatrix prev_lambda_perm;
			lambda_prev.Permute_UpperTriangular_To(prev_lambda_perm, p_order, mord_prev.n_Ordering_Size(), true);
			prev_R_ord.CholeskyOf_FBS<typename CSystemType::_TyHessianMatrixBlockList>(prev_lambda_perm);
		}
		// also of previous to have the sparse margs

		Eigen::MatrixXd margs_prev, margs;
#if 0
		{
			CUberBlockMatrix R, R_prev;
			R.CholeskyOf(lambda);
			R_prev.CholeskyOf(lambda_prev);
			CMarginals::Calculate_DenseMarginals_Ref(margs_prev, R_prev);
			CMarginals::Calculate_DenseMarginals_Ref(margs, R);
		}
#else // 0
		CMarginals::Calculate_DenseMarginals_Fast(margs_prev, prev_R_ord,
			mord_prev.p_Get_InverseOrdering(), mord_prev.n_Ordering_Size());
		CMarginals::Calculate_DenseMarginals_Fast(margs, cur_R_ord,
			mord.p_Get_InverseOrdering(), mord.n_Ordering_Size());
		// this more numerically stable (also faster, but that is not an issue here)
		// also, it is readilly verified that the result is the same as if using
		// unordered Cholesky in the branch above
#endif // 0
		// calculate marginals before and after

		CTimer t;
		CTimerSampler timer(t);

		double f_recurrent_time = 0;
		double f_reorder_time = 0;
		double f_compare_time = 0;
		double f_update_time = 0;

		CUberBlockMatrix sparse_margs_prev_ordered;
		CMarginals::Calculate_DenseMarginals_Recurrent_FBS<typename CSystemType::
			_TyHessianMatrixBlockList>(sparse_margs_prev_ordered, prev_R_ord, mord, mpart_Diagonal);

		timer.Accum_DiffSample(f_recurrent_time);

		CUberBlockMatrix sparse_margs_prev;
		sparse_margs_prev_ordered.Permute_UpperTriangular_To(sparse_margs_prev,
			mord_prev.p_Get_Ordering(), mord_prev.n_Ordering_Size(), false); // no share, but not much reason for it
		// calculate sparse marginals in the prev step and unorder them (to have the natural order)

		timer.Accum_DiffSample(f_reorder_time);

		double f_sparse_margs_error = f_IncompleteDifference(margs_prev, sparse_margs_prev);
		printf("debug: sparse marginals in prev step calculated with error: %g\n", f_sparse_margs_error);
		// see if the sparse marginals are indeed in the natural order

		CUberBlockMatrix sparse_margs_cur = sparse_margs_prev; // do not want to time the copy, it is for debug purposes only, otherwise no copy would be made

		timer.Accum_DiffSample(f_compare_time); // compare time is a dummy timer, really, it is not even displayed

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
		Update_BlockDiagonalMarginals<true>(system, sparse_margs_cur, lambda, cur_R_ord, mord, n_prev_edge_num);
		// update the marginals
#endif // __SE_TYPES_SUPPORT_L_SOLVERS

		timer.Accum_DiffSample(f_update_time);

		mord_prev.p_ExtendBlockOrdering_with_Identity(sparse_margs_cur.n_BlockColumn_Num());
		// make the ordering bigger, the new entries were added at the end

		//CUberBlockMatrix sparse_margs_cur;
		//sparse_margs_prev_ordered.Permute_UpperTriangular_To(sparse_margs_cur,
		//	mord_prev.p_Get_Ordering(), mord_prev.n_Ordering_Size(), false); // no share, but not much reason for it
		// note that mord_prev.p_Get_Ordering() now returns the extended ordering
		// the current marginals are now in "natural" ordering, no need to reperm

		// note that in case the user will be requesting a lot of updates, it would be better
		// to have the marginals in the same order as R (but that might be impossible after a couple
		// of steps, as R would be completely or partially reordered and calculating a new R with
		// extended identity ordering would most likely be quite expensive)

		// therefore, if not benchmarking, updating just the diagonal elements should be enough
		// (the last block column is calculated fully anyway, unless it was a loop closure, not
		// extending the size of the matrix)

		// but anyway, if calculating the marginals on demand, the diagonal can be always reused,
		// potentially saving a great portion of computation

		double f_sparse_margs_update_error = f_IncompleteDifference(margs, sparse_margs_cur);
		printf("debug: sparse marginals in cur step updated with error: %g\n", f_sparse_margs_update_error);
		// see if the sparse marginals are indeed in the natural order

		double f_time_total = -f_compare_time;
		timer.Accum_CumTime_LastSample(f_time_total);
		printf("marginals took: %.5f msec\n", f_time_total * 1000);
		printf("\trecurrent: %.5f msec (calculated " PRIsize " nnz)\n",
			f_recurrent_time * 1000, sparse_margs_prev.n_NonZero_Num());
		printf("\t  reorder: %.5f msec\n", f_reorder_time * 1000);
		printf("\t   update: %.5f msec (calculated " PRIsize " nnz)\n",
			f_update_time * 1000, sparse_margs_cur.n_NonZero_Num());
		// print stats
	}

	/**
	 *	@brief performs some timing and numerical analysis benchmark of updating marginals on a SE(2) system
	 *
	 *	@tparam CEdgeType is edge type to be added
	 *	@tparam CSystemType is optimized system type
	 *	@tparam CSolverType is solver type
	 *
	 *	@param[in,out] system is reference to the optimized system (gets extended by some edges / vertices)
	 *	@param[in,out] solver is reference to the nonlinear solver
	 *	@param[in] information is information matrix for the new edges
	 *
	 *	@note This reimplements most of the method to be able to access the internal state of the algorithm.
	 */
	template <class CEdgeType, class CSystemType, class CSolverType>
	static void Test_ExperimentalIncrementalMarginals2(CSystemType &system,
		CSolverType &solver, Eigen::Matrix3d information)
	{
		//system.r_Add_Edge(CEdgePose2D(83, 101, Eigen::Vector3d(-1.97704, -0.00703316, -3.16668), information, system));
		//solver.Optimize(); // want to have 101x101 system to begin with, to make S symmetric

		CUberBlockMatrix lambda_prev = solver.r_Lambda();
		// get a *copy* of lambda

		size_t n_prev_edge_num = system.r_Edge_Pool().n_Size();
		// remember how many edges there are

		system.r_Add_Edge(CEdgeType(83, 101, Eigen::Vector3d(-1.97704, -0.00703316, -3.16668), information, system));
		system.r_Add_Edge(CEdgeType(100, 101, Eigen::Vector3d(1.02465, 0.0317282, 0.024041), information, system));
		//system.r_Add_Edge(CEdgeType(96, 101/*102*/, Eigen::Vector3d(0.007666, 0.00487275, -3.16637), information, system)); // loop without changing dims (not present in the original dataset)
		system.r_Add_Edge(CEdgeType(96, 102, Eigen::Vector3d(0.007666, 0.00487275, -3.16637), information, system));
		system.r_Add_Edge(CEdgeType(96, 102, Eigen::Vector3d(0.0105401, -0.00932236, -3.11433), information, system));
		system.r_Add_Edge(CEdgeType(101, 102, Eigen::Vector3d(0.99396, 0.0202571, -0.00240724), information, system));
		// add some more edges

		solver.Optimize(0); // must call that to calculate hessians in the new lambda
		const CUberBlockMatrix &lambda = solver.r_Lambda();
		// get system matrix (without relinearizing!)

		/*{
			lambda_prev.Save_MatrixMarket("lambda_prev.mtx", "lambda_prev.bla");
			lambda.Save_MatrixMarket("lambda_cur.mtx", "lambda_cur.bla");
		}*/
		// for Ela

		size_t n_edge_num = system.r_Edge_Pool().n_Size();
		size_t n_order_min = lambda.n_BlockColumn_Num();
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) {
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				n_order_min = std::min(n_order_min, /*m_p_lambda_block_ordering[m_r_*/r_edge.n_Vertex_Id(j)/*]*/);
			// note that these are ids, but these equal order at the moment
		}
		size_t n_elem_order_min = lambda.n_BlockColumn_Base(n_order_min);
		// no ordering here, that is correct (omega is not ordered either)

		CUberBlockMatrix R, cur_R_ord;
		R.CholeskyOf(lambda);
		//
		CMatrixOrdering mord;
		mord.p_BlockOrdering(lambda, true);
		{
			const size_t *p_order = mord.p_Get_InverseOrdering();
			CUberBlockMatrix lambda_perm;
			lambda.Permute_UpperTriangular_To(lambda_perm, p_order, mord.n_Ordering_Size(), true);
			cur_R_ord.CholeskyOf_FBS<typename CSystemType::_TyHessianMatrixBlockList>(lambda_perm);
		}
		// take Cholesky (unordered so it is easier to compute the margs)

		CUberBlockMatrix R_prev;
		R_prev.CholeskyOf(lambda_prev);
		Eigen::MatrixXd margs_prev, margs;
		CMarginals::Calculate_DenseMarginals_Ref(margs_prev, R_prev);
		CMarginals::Calculate_DenseMarginals_Ref(margs, R);
		// calculate marginals before and after

		CDebug::Print_DenseMatrix_Dimensions(stdout, margs_prev, "margs_prev: ");
		CDebug::Print_DenseMatrix_Dimensions(stdout, margs, "margs: ");

		// --------- the badass inc method begins ---------

		CTimer t;
		CTimerSampler timer(t);

		double f_omega_time = 0;
		double f_dense_margs_time = 0;
		double f_update_basis_time = 0;
		double f_update_time = 0;

		CUberBlockMatrix omega;
		std::vector<size_t> required_column_list;
		required_column_list.reserve(2 * (n_edge_num - n_prev_edge_num)); // a guess; might be edges with more or less than 2 verts
		for(size_t i = n_prev_edge_num; i < n_edge_num; ++ i) { // not parallel! (wouls have conflicts)
			typename CSystemType::_TyConstEdgeRef r_edge = system.r_Edge_Pool()[i];
			r_edge.Calculate_Omega(omega, n_elem_order_min);
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j)
				required_column_list.push_back(r_edge.n_Vertex_Id(j));
		}
		// get omega and vertex id's

		/*{
			CUberBlockMatrix lam_diff = lambda_prev;
			lam_diff.ExtendTo(lambda.n_Row_Num(), lambda.n_Column_Num()); // otherwise add might fail because of different dims
			lambda.AddTo(lam_diff, -1);
			double f_err = omega.f_Norm() - lam_diff.f_Norm();
			printf("debug: norm of omega is %g\n", omega.f_Norm());
			printf("debug: norm of difference between lambdas is %g\n", lam_diff.f_Norm());
			printf("debug: difference between lambdas and calculated omega is %g\n", f_err);
		}*/ // debug

		std::sort(required_column_list.begin(), required_column_list.end());
		required_column_list.erase(std::unique(required_column_list.begin(),
			required_column_list.end()), required_column_list.end());
		// finalize the required column list (could use std::set, but that feels like overkill)

		{
			std::vector<size_t> pack_order;
			pack_order.reserve(omega.n_BlockColumn_Num());
			for(size_t i = 0, n = required_column_list.size(); i < n; ++ i) {
				size_t n_col = required_column_list[i];
				size_t n_base = lambda.n_BlockColumn_Base(n_col) - n_elem_order_min;
				size_t n_col_size;
				size_t n_col_omega = omega.n_Find_BlockColumn(n_base, n_col_size);
				_ASSERTE(n_col_size == lambda.n_BlockColumn_Column_Num(n_col));

				pack_order.push_back(n_col_omega);
			}
			// get numbers of "interesting" columns of omega (no ordering here either)

			const size_t n_packed_block_column_num = pack_order.size();
			// remember this

			for(size_t i = 1; i < n_packed_block_column_num; ++ i) {
				size_t n_o0 = pack_order[i - 1];
				size_t n_o1 = pack_order[i];
				_ASSERTE(n_o1 > n_o0); // should be sorted

				for(size_t j = n_o0 + 1; j < n_o1; ++ j)
					pack_order.push_back(j);
				// append the rest of the cols at the end (only up to omega.n_BlockColumn_Num() / 2 of them)
			}
			_ASSERTE(pack_order.size() == omega.n_BlockColumn_Num());
			// finalize the order

			std::vector<size_t> inv_pack_order(pack_order.size());
			for(size_t i = 0, n = pack_order.size(); i < n; ++ i)
				inv_pack_order[pack_order[i]] = i;
			// inverse the order

			CUberBlockMatrix omega_slim;
			omega.Permute_UpperTriangular_To(omega_slim,
				&inv_pack_order[0], inv_pack_order.size(), true);
			// pack omega

			omega_slim.SliceTo(omega_slim, n_packed_block_column_num,
				n_packed_block_column_num, true);
			_ASSERTE(omega_slim.n_BlockColumn_Num() == n_packed_block_column_num);
			// slice off the empty columns

			Eigen::MatrixXd omega_dense;
			omega_slim.Convert_to_Dense(omega_dense);
			// get dense omega

			timer.Accum_DiffSample(f_omega_time);

#ifdef _DEBUG
			//printf("debug: difference of norm(omega_dense) - norm(omega) = %g\n",
			//	fabs(omega_slim.f_Norm() - omega.f_Norm()));
			omega.Rasterize("incmargs2_01_omega.tga");
			omega_slim.Rasterize("incmargs2_02_omega-reorder.tga");
			omega_slim.Rasterize("incmargs2_03_omega-dense-slim.tga");
			CDebug::Print_DenseMatrix_Dimensions(stdout, omega_dense, "omega_dense: ");
			// debug
#endif // _DEBUG

			_ASSERTE(margs_prev.rows() <= margs.rows());
			const size_t n_prev_state_size = lambda_prev.n_Column_Num();
			const size_t n_prev_state_block_num = lambda_prev.n_BlockColumn_Num();
			const size_t n_omega_elems = omega_slim.n_Column_Num();
#if 0
			Eigen::MatrixXd Tu_ref(n_prev_state_size, n_omega_elems);
#endif // 0
			Eigen::MatrixXd Tu(n_prev_state_size/*lambda.n_Row_Num()*/, n_omega_elems); // allocate it smaller to begin with (saves expensive reshuffling in conservativeResize() below)
			//Tu.setConstant(123456); // in case we accidentally left some values uninitialized
			for(size_t i = 0/*, n_rows = Tu.rows()*/; i < n_packed_block_column_num; ++ i) { // could run in parallel, but usually needs like two to six columns (threads)
				size_t n_block_base_margs = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
				/*size_t n_column_orig = required_column_list[i];
				const size_t *p_order = mord.p_Get_InverseOrdering();
				size_t n_column_ordered = p_order[n_column_orig];
				size_t n_block_base_margs_R = cur_R_ord.n_BlockColumn_Base(n_column_ordered);
				_ASSERTE(n_block_base_margs == n_block_base_margs_R);*/ // does not work; cur_R_ord.n_BlockColumn_Base(x) == lambda.n_BlockColumn_Base(x), but order[x] != x, and this is only if there is a single block size: need the labda :(
				size_t n_block_base_Tu = omega_slim.n_BlockColumn_Base(i);
				size_t n_block_cols = omega_slim.n_BlockColumn_Column_Num(i);
				// get dimensions of this block

#if 0
				//if(n_block_base_margs < size_t(margs_prev.rows())) { // breaks it
					Tu_ref.block(0, n_block_base_Tu, /*n_rows*/n_prev_state_size, n_block_cols) =
						margs.block(0, n_block_base_margs, /*n_rows*/n_prev_state_size, n_block_cols);
				/*} else {
					Tu_ref.block(0, n_block_base_Tu, n_rows, n_block_cols).setZero(); // !!
					// t_odo - if this is really the case, can make it smaller; all the zeroes
					// would be concentrated on the right (will save ample computation later
					// on when calculating the aditive update as a product of Tu)
				}*/
				// calculate a block of dense marginals by whatever metod (in current time frame; here we just copy it from a full matrix)
#endif // 0

				CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<typename CSystemType::_TyHessianMatrixBlockList>(
					Tu.block(0, n_block_base_Tu, /*n_rows*/n_prev_state_size, n_block_cols),
					cur_R_ord, n_block_base_margs, mord.p_Get_InverseOrdering(), mord.n_Ordering_Size(),
					lambda_prev.n_BlockColumn_Num());
				// really calculate a block of dense marginals
			}
			// assemble Tu

#if 0
			printf("debug: error of inplace marginals: %g\n", (Tu_ref - Tu).norm());
#endif // 0

			//if(margs_prev.rows() < margs.rows())
			//	Tu.bottomLeftCorner(Tu.rows() - margs_prev.rows(), Tu.cols()).setZero();
			//if(margs_prev.rows() < margs.rows())
			//	Tu.conservativeResize(margs_prev.rows(), Tu.cols()); // saves some computation
			// zero out the bottom overhanging area in case the dimensions of the
			// covariance matrix grew. todo - what about the right overhanging area?

			Eigen::MatrixXd s(n_omega_elems, n_omega_elems);
			//s.setConstant(123456); // in case we accidentally left some values uninitialized
			for(size_t i = 0; i < n_packed_block_column_num; ++ i) {
				size_t n_block_base_row_Tu = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
				size_t n_block_base_row_s = omega_slim.n_BlockColumn_Base(i);
				size_t n_block_rows = omega_slim.n_BlockColumn_Column_Num(i); // is symmetric
				// get dimensions of this block

				if(n_block_base_row_Tu < n_prev_state_size) {
					s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems) =
						Tu.block(n_block_base_row_Tu, 0, n_block_rows, n_omega_elems);
				} else
					s.block(n_block_base_row_s, 0, n_block_rows, n_omega_elems).setZero();
				// copy block from Tu to s
			}
			// cut out s (could be performed inside a sparse block matrix
			// multiplication, except now we don't have the data in a block matrix)

			/*if(margs_prev.rows() < margs.rows()) {
				Tu.bottomLeftCorner(Tu.rows() - margs_prev.rows(), Tu.cols()).setZero();
				for(size_t i = 0, n_rows = Tu.rows(); i < n_packed_block_column_num; ++ i) {
					size_t n_block_base_margs = lambda.n_BlockColumn_Base(required_column_list[i]); // position in the (dense) marginals
					size_t n_block_base_Tu = omega_slim.n_BlockColumn_Base(i);
					size_t n_block_cols = omega_slim.n_BlockColumn_Column_Num(i);
					// get dimensions of this block

					if(n_block_base_margs >= size_t(margs_prev.rows())) {
						Tu.block(0, n_block_base_Tu, n_rows, n_block_cols).setZero(); // !!
						// t_odo - if this is really the case, can make it smaller; all the zeroes
						// would be concentrated on the right (will save ample computation later
						// on when calculating the aditive update as a product of Tu)
					}
					// calculate a block of dense marginals by whatever metod (in current time frame; here we just copy it from a full matrix)
				}
			}*/ // breaks it

			timer.Accum_DiffSample(f_dense_margs_time);

#ifdef _DEBUG
			CDebug::Print_DenseMatrix_Dimensions(stdout, Tu, "Tu: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, s, "s: ");
#endif // _DEBUG

//#define __INCREMENTAL_MARGINALS_USE_LDLT
//#define __INCREMENTAL_MARGINALS_LDLT_USE_DOUBLE_LDLT // will not work. ever. (well, to be fair, it will work if the size of the system did not change and if omega is positive semidefinite, probably not worth pursuing)
//#define __INCREMENTAL_MARGINALS_TRY_LLT // waste of time? yes.

#ifdef __INCREMENTAL_MARGINALS_USE_LDLT
			//omega_dense.triangularView<Eigen::StrictlyLower>() =
			//	omega_dense.triangularView<Eigen::StrictlyUpper>().transpose(); // LDLT need that? no, it is Eigen::LDLT<Eigen::MatrixXd, Eigen::Upper>, it does not touch lower
			Eigen::LDLT<Eigen::MatrixXd, Eigen::Upper> ldlt(omega_dense);
			_ASSERTE(ldlt.info() == Eigen::Success);
			// calculate Cholesky

			Eigen::MatrixXd H = Eigen::MatrixXd(ldlt.matrixU()) * ldlt.transpositionsP().transpose(); // need to convert, matrixU() returns triangular view
			Eigen::Diagonal<const Eigen::MatrixXd> d = ldlt.vectorD();
			// get H, d

			CDebug::Print_DenseMatrix_Dimensions(stdout, H, "H: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, d, "d: diagonal ");

#ifndef __INCREMENTAL_MARGINALS_LDLT_USE_DOUBLE_LDLT
			Eigen::MatrixXd V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) -
				H * s * H.transpose() * d.asDiagonal();
			// calculate V

			size_t n_zero_num = 0;
			{
				CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, d, "d = ", "\n", " %g");
				Eigen::VectorXd d_num = d;
				for(int i = d_num.rows(); i > 0;) {
					-- i; // here
					if(fabs(d_num(i)) == 0)
						++ n_zero_num;
					else
						break;
				}
				if(n_zero_num)
					printf("debug: the last %d entries in d are exactly zero\n", n_zero_num);
				//CDebug::Print_DenseVector_in_MatlabFormat(stdout, d.data(), d.rows(), "  = "); // does not work, has some strange layout
				// kind of curious; this potentionally cuts down dimensionality of Tu (trailing entries of d seem to be zeroes)
			}
			// debug

			Eigen::MatrixXd Bu = (H.transpose() * (d.asDiagonal() * V.inverse()) * H) * Tu.transpose();
			// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
			// of the product mutliply Tu (that way at least the computation is saved, if not storage)

			/*const int n_zero_num = 3; // bad idea, those are almost zero, but not quite*/
			//Tu.block(0, Tu.cols() - n_zero_num, Tu.rows(), n_zero_num).setZero();
			//Bu.block(Bu.rows() - n_zero_num, 0, n_zero_num, Bu.cols()).setZero();
			// see if we can shave Tu a bit (then we would store only a part of it and calculate only with a part of it)

			const size_t n_nonzero_num = n_omega_elems - n_zero_num;

			timer.Accum_DiffSample(f_update_basis_time);

			CDebug::Print_DenseMatrix_Dimensions(stdout, Tu.topLeftCorner(Tu.rows(), n_nonzero_num), "left _ cols of Tu: ");
			printf("debug: norm of left %d cols of Tu: %g\n", n_nonzero_num, Tu.topLeftCorner(Tu.rows(), n_nonzero_num).norm());
			CDebug::Print_DenseMatrix_Dimensions(stdout, Tu.topRightCorner(Tu.rows(), n_nonzero_num), "right _ cols of Tu: ");
			printf("debug: norm of right %d cols of Tu: %g\n", n_nonzero_num, Tu.topRightCorner(Tu.rows(), n_nonzero_num).norm());
			CDebug::Print_DenseMatrix_Dimensions(stdout, Bu.topLeftCorner(n_nonzero_num, Tu.rows()), "top _ rows of Bu: ");
			printf("debug: norm of top %d rows of Bu: %g\n", n_nonzero_num, Bu.topLeftCorner(n_nonzero_num, Tu.rows()).norm());
			CDebug::Print_DenseMatrix_Dimensions(stdout, Bu.bottomLeftCorner(n_nonzero_num, Tu.rows()), "bottom _ rows of Bu: ");
			printf("debug: norm of bottom %d rows of Bu: %g\n", n_nonzero_num, Bu.bottomLeftCorner(n_nonzero_num, Tu.rows()).norm());
			// all equally nonzero, no use for this formula

			Eigen::MatrixXd margs_diff2 = Tu * Bu;
			//Eigen::MatrixXd margs_diff2 = Tu.topLeftCorner(Tu.rows(), n_nonzero_num) *
			//	Bu.topLeftCorner(n_nonzero_num, Bu.cols()); // does not work for some reason
			// on odometry steps, there are half of the vector zeroes, can save computation
			// in the update step (a small motivation why do this)
#else // !__INCREMENTAL_MARGINALS_LDLT_USE_DOUBLE_LDLT
#error "this is conceptually flawed, can't do double LDLT as the second matrix for LDLT is not symmetric"
			Eigen::VectorXd d_abs_sqrt = d.array().abs().sqrt().matrix();
			Eigen::VectorXd d_sign(d.rows());
			for(int i = 0, n = d.rows(); i < n; ++ i)
				d_sign(i) = (d(i) < 0)? -1 : 1;
			// make a sign function and abs(sqrt(.)) function of d

			Eigen::MatrixXd H_sqrt_d = d_abs_sqrt.asDiagonal() * H; // mind the order (d is in the middle of H^T * d * H)
			{
				Eigen::MatrixXd omega_dense_sym = omega_dense;
				omega_dense_sym.triangularView<Eigen::StrictlyLower>() =
					omega_dense.triangularView<Eigen::StrictlyUpper>().transpose(); // need both halves for comparison
				printf("debug: norm of omega - H^T * d * H: %g\n",
					((H.transpose() * d.asDiagonal() * H) - omega_dense_sym).norm());
				printf("debug: norm of omega - H_sqrt(d)^T * sign(d) * H_sqrt(d): %g\n",
					((H_sqrt_d.transpose() * d_sign.asDiagonal() * H_sqrt_d) -
					omega_dense_sym).norm());
				printf("debug: norm of omega - chol.reconstructedMatrix(): %g\n",
					(ldlt.reconstructedMatrix() - omega_dense_sym).norm());
			}
			// make sure this is the order to multiply them

			Eigen::MatrixXd V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) -
				H_sqrt_d * s * H_sqrt_d.transpose() * d_sign.asDiagonal(); // d_sign breaks the symmetry
			// calculate a slightly different V

			size_t n_nonzero_num = n_omega_elems;// - n_zero_num;

			Eigen::FullPivLU<Eigen::MatrixXd> luV(V);
			_ASSERTE(luV.isInvertible());

			//Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> chol(d.asDiagonal() * V.inverse()); // fails: d * V is not symmetric and not pos def
			Eigen::LDLT<Eigen::MatrixXd, Eigen::Upper> chol(luV.inverse()); // works, error 4.113 - quite big
			//Eigen::LDLT<Eigen::MatrixXd, Eigen::Upper> chol(H.transpose() * (d.asDiagonal() * V.inverse()) * H);
			_ASSERTE(chol.info() == Eigen::Success);
			Eigen::MatrixXd Vy = Eigen::MatrixXd(chol.matrixU()) * chol.transpositionsP().transpose(); // need to convert, matrixU() returns triangular view
			Eigen::Diagonal<const Eigen::MatrixXd> d2 = chol.vectorD();
			//Eigen::MatrixXd d2 = //chol.transpositionsP().transpose() *
			//	Eigen::MatrixXd(chol.vectorD().asDiagonal()) /** chol.transpositionsP()*/; // has any effect?
			// can have a smaller dot product

			{
				CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, d_sign, "d_sign = ");
				double V_inv_symmetry = (luV.inverse() - luV.inverse().transpose()).norm();
				printf("debug: error of symmetry of inverse V is: %g\n", V_inv_symmetry);
				Eigen::MatrixXd _V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) -
					H * s * H.transpose() * d.asDiagonal();
				Eigen::MatrixXd ref = H.transpose() * d.asDiagonal() * _V.inverse() * H; // this is what we want
				Eigen::MatrixXd ref2 = H_sqrt_d.transpose() * d_sign.asDiagonal() * luV.inverse() * H_sqrt_d;
				Eigen::MatrixXd ref3 = H_sqrt_d.transpose() * d_sign.asDiagonal() * chol.reconstructedMatrix() * H_sqrt_d;
				Eigen::MatrixXd rec = H_sqrt_d.transpose() * Vy.transpose() * d_sign.asDiagonal() * d2.asDiagonal() * Vy * H_sqrt_d;
				Eigen::MatrixXd rec2 = H_sqrt_d.transpose() * d_sign.asDiagonal() * (Vy.transpose() * d2.asDiagonal() * Vy) * H_sqrt_d;
				printf("debug: norm of H^T * (d * V^-1) * H - Hsqd^T * d_sign * Vsqd^-1 * Hsqd: %g\n", (ref - ref2).norm());
				printf("debug: norm of H^T * (d * V^-1) * H - Hsqd^T * d_sign * chol(Vsqd^-1).reconst() * Hsqd: %g\n", (ref - ref3).norm());
				printf("debug: norm of H^T * (d * V^-1) * H - Hsqd^T * (Vy^T * d_sign * d2 * Vy) * Hsqd: %g\n", (ref - rec).norm());
				printf("debug: norm of H^T * (d * V^-1) * H - Hsqd^T * d_sign * (Vy^T * d2 * Vy) * Hsqd: %g\n", (ref - rec2).norm());
				/*printf("debug: norm of d * V^-1 - Vy^T * d2 * Vy: %g\n",
					((Vy.transpose() * d2 * Vy) - dVi).norm());
				printf("debug: norm of d * V^-1 - chol.reconstructedMatrix(): %g\n",
					(chol.reconstructedMatrix() - dVi).norm());*/
			}
			// can we cheat like this?

			/*Eigen::VectorXd d2_num = d2;
			d2_num.conservativeResize(n_nonzero_num); // consider just the nnz head

			std::vector<int> perm_vector;
			perm_vector.reserve(n_nonzero_num);
			size_t n_positive_head_size = 0, n_negative_tail_size = 0;
			for(size_t i = 0; i < n_nonzero_num; ++ i) {
				double f = d2_num(i);
				if(f > 0) {
					perm_vector.insert(perm_vector.begin() + n_positive_head_size, 1, i);
					++ n_positive_head_size;
				} else if(f < 0) {
					perm_vector.push_back(i);
					++ n_negative_tail_size;
				}
			}
			perm_vector.resize(n_negative_tail_size + n_positive_head_size);*/ // erase the rest
			// d2 is sorted by absolute magnitude, positive and negative entries
			// may be scattered, zeroes should be all at the end though (but somehow that does not always happen)

			/*Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, 1>, Eigen::Unaligned>
				perm_vec(&perm_vector[0], perm_vector.size()); // no copy is made
			Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> perm_mat(perm_vec); // symmetric permutation matrix

			Eigen::MatrixXd VH = (Vy.lazyProduct(H)).block(0, 0, n_nonzero_num, n_nonzero_num);
			Eigen::VectorXd d2_abs(perm_vector.size());
			Eigen::MatrixXd VH_perm(perm_vector.size(), perm_vector.size()),
				Tu_perm(Tu.rows(), perm_vector.size());
			for(size_t i = 0; i < n_positive_head_size; ++ i) {
				d2_abs(i) = d2(perm_vector[i]);
				Tu_perm.col(i) = Tu.col(perm_vector[i]);
				VH_perm.col(i) = VH.col(perm_vector[i]).head(perm_vector.size());
			}
			for(size_t i = n_positive_head_size, n = perm_vector.size(); i < n; ++ i) {
				d2_abs(i) = -d2(perm_vector[i]);
				Tu_perm.col(i) = Tu.col(perm_vector[i]);
				VH_perm.col(i) = VH.col(perm_vector[i]).head(perm_vector.size());
			}
			VH = VH_perm;
			for(size_t i = 0, n = perm_vector.size(); i < n; ++ i)
				VH_perm.row(i) = VH.row(perm_vector[i]);
			// make permuted version of d2 and Tu // todo - time these permutations, potentially a lot of data is moved

			//VH_perm = VH * perm_mat.transpose();// * perm_mat; // perm_mat could actually be smaller than VH. troubling.

			Eigen::MatrixXd Bu_tr = (d2_abs.array().sqrt().matrix().asDiagonal() * VH_perm).lazyProduct(Tu_perm.transpose());*/ // save something on this product already (the lazy stuff and expression templates should distribute the .block)
			// could save perms by doing d2_num.array().sqrt().matrix().asDiagonal() * VW * perm_matrix * Tu_perm.transpose() or also matrix - have to figure out which one is the fastest

			Eigen::MatrixXd Bu_tr_perm = (/*d2.array().abs().sqrt().matrix().asDiagonal() * */Vy * H).lazyProduct(Tu.transpose());
			/*Eigen::MatrixXd Bu_tr(perm_vector.size(), Bu_tr_perm.cols());
			for(size_t i = 0, n = perm_vector.size(); i < n; ++ i)
				Bu_tr.row(i) = Bu_tr_perm.row(perm_vector[i]);*/

			timer.Accum_DiffSample(f_update_basis_time);

			//CDebug::Print_DenseMatrix_Dimensions(stdout, Bu_tr, "Bu_tr: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, Bu_tr_perm, "Bu_tr_perm: ");

#ifdef _DEBUG
		/*	CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, d2_num, "d2 = ");
			printf("perm_vector = {");
			for(size_t i = 0, n = perm_vector.size(); i < n; ++ i)
				printf((i)? ", %d" : "%d", perm_vector[i]);
			printf("}\n");
			printf("positive head: %d, negative tail: %d\n", n_positive_head_size, n_negative_tail_size);*/
		//	CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, d2_abs, "d2_abs = ");
		//	CDebug::Print_DenseMatrix_Dimensions(stdout, Tu_perm, "Tu_perm: ");
#endif // _DEBUG

			Eigen::MatrixXd margs_diff2 = Bu_tr_perm.transpose() * d2.asDiagonal() * Bu_tr_perm;
			/*Eigen::MatrixXd margs_diff2 =
				Bu_tr.topLeftCorner(n_positive_head_size, Bu_tr.cols()).transpose() *
				Bu_tr.topLeftCorner(n_positive_head_size, Bu_tr.cols()) -
				Bu_tr.topRightCorner(n_negative_tail_size, Bu_tr.cols()).transpose() *
				Bu_tr.topRightCorner(n_negative_tail_size, Bu_tr.cols());*/
			// slightly more complicated, but same amount of FLOPs

			//Eigen::MatrixXd margs_diff2 = Bu_tr.transpose() /** d2_num.asDiagonal()*/ * Bu_tr; // d2 kind of spoils it
			// this is super imprecise, it only trades one small matrix multiplication for LDLT (probably more expensive due to pivoting)
#endif // !__INCREMENTAL_MARGINALS_LDLT_USE_DOUBLE_LDLT
			timer.Accum_DiffSample(f_update_time);

			CDebug::Print_DenseMatrix_Dimensions(stdout, V, "V: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, (H.transpose() * (d.asDiagonal() * V.inverse()) * H), "H^T * V^-1 * d * H: ");
#else // __INCREMENTAL_MARGINALS_USE_LDLT
			Eigen::MatrixXd V, margs_diff2;
#ifdef __INCREMENTAL_MARGINALS_TRY_LLT
			bool b_have_update;
			Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> llt(omega_dense);
			if((b_have_update = (llt.info() == Eigen::Success))) { // non-negative matrix factorization would probably be of more use here
				printf("debug: was able to use the first LL^T Cholesky\n"); // this happens

				Eigen::MatrixXd H = llt.matrixU();
				V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) - H.transpose() * s * H;
#if 0
				Eigen::LDLT<Eigen::MatrixXd, Eigen::Upper> ldlt(V.inverse());
				_ASSERTE(ldlt.info() == Eigen::Success); // sure ...
				Eigen::MatrixXd Vy = Eigen::MatrixXd(ldlt.matrixU()) * ldlt.transpositionsP().transpose(); // need to convert, matrixU() returns triangular view
				Eigen::Diagonal<const Eigen::MatrixXd> d2 = ldlt.vectorD();
				// get Vy, d2

				// now Bu_tr = sqrt(abs(d2)) * Vy * H * Tu.transpose()
				// and margs_diff2 = Bu_tr.transpose() * sign(d2) * Bu_tr
				// the signs can be reordered in such a way that they form
				// a difference of two blocked matrix multiplications

				// unfinished, the first LLT never seems to succeed
				// it should however work in damped least squares (LM) just fine
#else // 0
				Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> llt2(V.inverse());
				if(llt2.info() == Eigen::Success) { // if not, have to retreat to the other edge
					printf("debug: was able to use the second LL^T Cholesky\n"); // does this ever happen?
					Eigen::MatrixXd Bu_tr = (llt2.matrixU() * H) * Tu.transpose();

					timer.Accum_DiffSample(f_update_basis_time);

					margs_diff2 = Bu_tr.transpose() * Bu_tr;
					// have a symmetric update, using less memory (and therefore less bandwidth)

					timer.Accum_DiffSample(f_update_time);
				} else if((llt2 = Eigen::LLT<Eigen::MatrixXd, Eigen::Upper>(H.transpose() *
				   V.inverse() * H)).info() == Eigen::Success) { // or like this with H in it (need to change Bu_tr below accordingly)
					printf("debug: was able to use the third LL^T Cholesky\n"); // does this ever happen?
					Eigen::MatrixXd Bu_tr = (llt2.matrixU() /** H*/) * Tu.transpose(); // H already inside the factor (passed to chol() alongside V)

					timer.Accum_DiffSample(f_update_basis_time);

					margs_diff2 = Bu_tr.transpose() * Bu_tr;
					// have a symmetric update, using less memory (and therefore less bandwidth)

					timer.Accum_DiffSample(f_update_time);
				} else
					b_have_update = false;
#endif // 0
			}
			if(!b_have_update) {
				printf("debug: was NOT able to use LL^T Cholesky\n");
#else // __INCREMENTAL_MARGINALS_TRY_LLT
			{
#endif // __INCREMENTAL_MARGINALS_TRY_LLT

				// above:
				// omega = H^Td * H
				// V = I - H * s * H^Td
				// diff = Tu * H^Td * V^-1 * H * Tu^T

				// here:
				// omega = omega * I		| and I is square identity, idempotent to matrix product
				// V = I - I * s * omega
				// diff = Tu * omega * V^-1 * I * Tu^T

				// or:
				// omega = I * omega		| and I is square identity, idempotent to matrix product
				// V = I - omega * s * I
				// diff = Tu * I * V^-1 * omega * Tu^T

				omega_dense.triangularView<Eigen::StrictlyLower>() =
					omega_dense.triangularView<Eigen::StrictlyUpper>().transpose();
				// need both halves! (block matrix omega doesn't contain its lower triangular part)

				V = Eigen::MatrixXd::Identity(n_omega_elems, n_omega_elems) - s * omega_dense;
				// calculate V

				// todo - do more manual inversion (with LU?) of V,
				// in case it is not invertible, refrain to use batch marginals

				Eigen::MatrixXd Bu = ((omega_dense * V.inverse()) * Tu.transpose());
				// t_odo - produce the symmetrical product; or double the memory and have e.g. right side
				// of the product mutliply Tu (that way at least the computation is saved, if not storage)

				timer.Accum_DiffSample(f_update_basis_time);

				margs_diff2 = Tu * Bu; // the update is a product of a block row vector and a block column vector

				timer.Accum_DiffSample(f_update_time);
			}

#ifdef _DEBUG
			CDebug::Print_DenseMatrix_Dimensions(stdout, V, "V: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, (V.inverse() * omega_dense), "V^-1 * omega: ");
#endif // _DEBUG
#endif // __INCREMENTAL_MARGINALS_USE_LDLT
#ifdef _DEBUG
			CDebug::Print_DenseMatrix_Dimensions(stdout, margs_diff2, "margs_diff2: ");
#endif // _DEBUG

			Eigen::MatrixXd margs_downd = margs.topLeftCorner(margs_prev.rows(), margs_prev.cols()) + margs_diff2;

			/*{
				Eigen::MatrixXd omega_full = omega_dense;
				omega_full.triangularView<Eigen::StrictlyLower>() =
					omega_dense.triangularView<Eigen::StrictlyUpper>().transpose();
				CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, omega_full, "omega_dense = ");
			}
			CDebug::Print_DenseMatrix_in_MatlabFormat(stdout,
				Eigen::VectorXd(omega_dense.diagonal()), "omega_dense.diagonal() = ");*/
			fprintf(stderr, "norm of update: %g\n", margs_downd.norm());
			fprintf(stderr, "norm of 6 x 6 bottom-right corner of the update: %g\n", margs_downd.bottomRightCorner(6, 6).norm());
			/*if(margs_prev.rows() != margs.rows()) { // in case the system grew ...
				fprintf(stderr, "norm of inside update: %g\n",
					margs_downd.topLeftCorner(margs_prev.rows(), margs_prev.cols()).norm());
				fprintf(stderr, "norm of 6 x 6 bottom-right corner of the inside update: %g\n",
					margs_downd.topLeftCorner(margs_prev.rows(), margs_prev.cols()).bottomRightCorner(6, 6).norm());
			}*/
			//fprintf(stderr, "norm of 60 x 60 bottom-right corner of the update: %g\n", margs_downd.bottomRightCorner(60, 60).norm());
			double f_error = (margs_diff2/*.topLeftCorner(margs_prev.rows(), margs_prev.cols())*/ - (margs_prev - margs.topLeftCorner(margs_prev.rows(), margs_prev.cols()))).norm();
			double f_error2 = (margs_prev - margs_downd/*.topLeftCorner(margs_prev.rows(), margs_prev.cols())*/).norm(); // now it works even if dims change
			fprintf(stderr, "error of differences: %g\n", f_error);
			fprintf(stderr, "error of marginals: %g\n", f_error2);

			double f_margsupd_total_time = 0;
			timer.Accum_CumTime_LastSample(f_margsupd_total_time);
			printf("marginals update took %.5f msec\n", f_margsupd_total_time * 1000);
			printf("\tomega: %.5f msec\n", f_omega_time * 1000);
			printf("\tTu, s: %.5f msec\n", f_dense_margs_time * 1000);
			printf("\tbasis: %.5f msec\n", f_update_basis_time * 1000);
			printf("\t  upd: %.5f msec\n", f_update_time * 1000);
		}

#if 0
		if(lambda.b_EqualLayout(lambda_prev)) {
			lambda_prev.Rasterize("incmargs_00_lambda-prev.tga");
			lambda.Rasterize("incmargs_01_lambda.tga");

			CUberBlockMatrix omega_b;
			omega_b = lambda;
			lambda_prev.AddTo(omega_b, -1);
			// omega_b = lambda - lambda_prev

			omega_b.Rasterize("incmargs_02_lambda-diff.tga");

			//size_t n_omega_block_cols = omega.n_BlockColumn_Num(); // can't count that, does not have all the columns with good dims (very sparse)
			size_t n_lambda_block_cols = lambda.n_BlockColumn_Num();
			size_t n_first_block_col = n_order_min;//n_lambda_block_cols - n_omega_block_cols;
			CUberBlockMatrix omega_slice;
			omega_b.SliceTo(omega_slice, n_first_block_col, n_lambda_block_cols,
				n_first_block_col, n_lambda_block_cols, true);

			omega_slice.Rasterize("incmargs_03_lambda-diff-slice.tga");
			omega.AddTo(omega_slice, -1);
			omega.Rasterize("incmargs_04_omega.tga");
			omega_slice.Rasterize("incmargs_05_omega-diff.tga");

			double f_norm = omega_slice.f_Norm();
			printf("norm of omegas is %g\n", f_norm);

			CDebug::Print_DenseMatrix_Dimensions(stdout, margs_prev, "margs_prev: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, margs, "margs: ");

			Eigen::MatrixXd margs_diff = margs - margs_prev;
			// calculate the difference

			Eigen::MatrixXd Hu;
			Eigen::Matrix3d Hui, Hun;
			//omega.Convert_to_Dense(Hu);
			{
				_ASSERTE(n_edge_num - n_prev_edge_num == 1); // now we manually handle only a single edge
				const typename CSystemType::_TyBaseEdge &last_edge = system.r_Edge_Pool()[system.r_Edge_Pool().n_Size() - 1];
				Eigen::Vector3d exp, err;
				last_edge.Calculate_Jacobians_Expectation_Error(Hui, Hun, exp, err);
				// todo - need to derive the type appropriately, probably could be a function of the pool, call back with true type (it already is for_each()? do for_one()?)
				// interesting problem - have an indexable vfptr, generated by a template, per type
				// have N types, N vfptrs, stored in an array, each contains templated function (can't!)
				// would need to store type id (not so hard, just not nice)

				/*Hui = -Hui;
				Hun = -Hun;*/

				Hu.resize(3, 6);
				Hu.block(0, 0, 3, 3) = Hui;
				Hu.block(0, 3, 3, 3) = Hun;
				// Hu = [Hui Hun];
			}
			// get Hu

			CDebug::Print_DenseMatrix_Dimensions(stdout, Hui, "Hui: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, Hun, "Hun: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, Hu, "Hu: ");

			Eigen::MatrixXd Tu = margs/*_prev*/.block(0, n_elem_order_min,
				margs/*_prev*/.rows(), margs/*_prev*/.cols() - n_elem_order_min); // column block (all rows in it)
			Eigen::MatrixXd s = margs/*_prev*/.block(n_elem_order_min, n_elem_order_min,
				margs/*_prev*/.rows() - n_elem_order_min, margs/*_prev*/.cols() - n_elem_order_min); // smaller square block
			// get Tu, s

			CDebug::Print_DenseMatrix_Dimensions(stdout, Tu, "Tu: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, s, "s: ");

			_ASSERTE(Hu.rows() == 3 && Hu.cols() == 6); // just two verts, tightly packed
			// meh; need to compress Hu and Tu (and s) to avoid empty space. probably easy with a single edge, hard with e.g. each 10

			Eigen::MatrixXd Su = information.inverse() - Hu * s * Hu.transpose();
			// t_odo - need to get information for every edge? how to do multiple edges? one at a time?
			// t_odo - omega already has information in it!

			// on more edges, need to accumulate Bu (below), which has rows of lambda and cols of ?measurement? dimension.
			// adding measurements of different dims would require resizing Bu and adding them over each other, should
			// still be correct.

			Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> chol(Su.inverse());
			_ASSERTE(chol.info() == Eigen::Success);
			Eigen::MatrixXd Vy = chol.matrixU();
			//Eigen::MatrixXd Bu = Tu * (Hu.transpose() * Vy.transpose()); // Hu and Vy are small (and fixed-size), this is faster
			//Eigen::MatrixXd Bu_tr = (Tu * (Hu.transpose() * Vy.transpose())).transpose();
			//Eigen::MatrixXd Bu_tr = Eigen::MatrixXd((Hu.transpose() * Vy.transpose()).transpose()) * Tu.transpose(); // too many transposes for Eigen, seems to work with a temporary, but should equal the above / below formula
			Eigen::MatrixXd Bu_tr = (Vy * Hu) * Tu.transpose();
			//Eigen::MatrixXd margs_diff2 = -Bu * Bu.transpose();
			Eigen::MatrixXd margs_diff2 = Bu_tr.transpose() * Bu_tr; // each component of the diff is a short dot product / small matrix multiplication

			// or, just get Hu' * Vy' and get Tu outside, as there needs to be some "smart" mgmt of what
			// to calculate (need *dense* column ranges of the marginals)

			CDebug::Print_DenseMatrix_Dimensions(stdout, Su, "Su: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, Vy, "Vy: ");
			//CDebug::Print_DenseMatrix_Dimensions(stdout, Bu, "Bu: ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, Bu_tr, "Bu': ");
			CDebug::Print_DenseMatrix_Dimensions(stdout, margs_diff2, "margs_diff2: ");

			/*{
				FILE *p_fw = fopen("margs_diff.m", "w");
				CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, margs_diff2, "margsDiff = ");
				fclose(p_fw);
			}*/ // bluh

			Eigen::MatrixXd margs_upd = margs/*_prev*/ + margs_diff2;

			double f_error = (-margs_diff2 - margs_diff).norm();
			double f_error2 = (margs_prev - margs_upd).norm();
			fprintf(stderr, "norm of update: %g\n", margs_upd.norm());
			fprintf(stderr, "norm of 6 x 6 bottom-right corner of the update: %g\n", margs_upd.bottomRightCorner(6, 6).norm());
			//fprintf(stderr, "norm of 60 x 60 bottom-right corner of the update: %g\n", margs_upd.bottomRightCorner(60, 60).norm());
			fprintf(stderr, "error of differences: %g\n", f_error);
			fprintf(stderr, "error of marginals: %g\n", f_error2);
		}
		// just checks omega, in case the dimensions did not change (and calculates up / downdate to the marginals btw)
#endif // 0
	}
};

/**
 *	@brief marginal covariances cache object
 */
class CMarginalCovariance {
protected:
	mutable Eigen::MatrixXd m_matrix; /**< @brief marginal covariance matrix */
	CUberBlockMatrix m_sparse; /**< @brief sparse marginal covariance matrix */
	mutable bool m_b_dense_up_to_date; /**< @brief dense marginals matrix dirty flag */

	size_t m_n_edges_in_last_marginals; /**< @brief number of edges used for calculation of the last marginals (only stored here; set and read from outside) */
	bool m_b_can_update_marginals; /**< @brief incremental update availability flag (only stored here; set and read from outside) */

public:
	/**
	 *	@brief default constructor; initializes an empty marginals cache
	 */
	CMarginalCovariance()
		:m_b_dense_up_to_date(true), // initially both empty, that is ok
		m_n_edges_in_last_marginals(0),
		m_b_can_update_marginals(false) // initially can't update, that would amount to running full dense marginals
	{}

	/**
	 *	@brief gets the number of edges used for calculation of the marginals
	 *	@return Returns the number of edges used for calculation of the current marginals.
	 *	@note The consistency of this counter is maintained by the caller (which is the nonlinear solver).
	 */
	size_t n_Edge_Num() const
	{
		return m_n_edges_in_last_marginals;
	}

	/**
	 *	@brief updates the number of edges since the last marginals recalculation / update
	 *	@param[in] n_edge_num is number of edges used for calculation of the marginals
	 */
	void Set_Edge_Num(size_t n_edge_num)
	{
		m_n_edges_in_last_marginals = n_edge_num;
	}

	/**
	 *	@brief reads the update availability flag
	 *	@return Returns true if the marginals can be incrementally updated, false if they
	 *		need to be recalculated (that is typically after a linearization point change).
	 *	@note The consistency of this flag is maintained by the caller (which is the nonlinear solver).
	 */
	bool b_CanUpdate() const
	{
		return m_b_can_update_marginals;
	}

	/**
	 *	@brief raises the update availability flag
	 *	@note Call this after calculating marginals using batch.
	 */
	void EnableUpdate()
	{
		m_b_can_update_marginals = true;
	}

	/**
	 *	@brief clears the update availability flag
	 *	@note Call this on linearization point change.
	 */
	void DisableUpdate()
	{
		m_b_can_update_marginals = false;
	}

#if 0
	/**
	 *	@brief gets marginal covariance matrix
	 *
	 *	@return Returns reference to the marginal covariance matrix.
	 *
	 *	@note Depending on the policy used, this matrix might not contain all of the
	 *		marginal covariance values, or they might not be up to date.
	 *		See e.g. Request_Block() for more details.
	 *	@note The space complexity required to store the full matrix can be significant,
	 *		more efficient methods for passing only a part of the matrix will be provided.
	 */
	inline Eigen::MatrixXd &r_Matrix()
	{
		return m_matrix;
	}
#endif // 0

	/**
	 *	@brief sets marginal covariance matrix
	 *	@param[in] r_sparse is the new marginal covariance matrix
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Set_SparseMatrix(const CUberBlockMatrix &r_sparse) // throw(std::bad_alloc)
	{
		m_b_dense_up_to_date = false;
		m_sparse = r_sparse;
	}

	/**
	 *	@brief sets marginal covariance matrix
	 *	@param[in,out] r_sparse is the new marginal covariance matrix (destroyed in the process)
	 */
	inline void Swap_SparseMatrix(CUberBlockMatrix &r_sparse)
	{
		m_b_dense_up_to_date = false;
		m_sparse.Swap(r_sparse);
	}

	/**
	 *	@brief gets marginal covariance matrix
	 *
	 *	@return Returns reference to the marginal covariance matrix.
	 *
	 *	@note Depending on the policy used, this matrix might not contain all of the
	 *		marginal covariance values, or they might not be up to date.
	 *		See e.g. Request_Block() for more details.
	 */
	inline const CUberBlockMatrix &r_SparseMatrix() const
	{
		return m_sparse;
	}

	/**
	 *	@brief gets marginal covariance matrix
	 *
	 *	@return Returns const reference to the marginal covariance matrix.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note Depending on the policy used, this matrix might not contain all of the
	 *		marginal covariance values, or they might not be up to date.
	 *		See e.g. Request_Block() for more details.
	 *	@note The space complexity required to store the full matrix can be significant,
	 *		more efficient methods for passing only a part of the matrix will be provided.
	 */
	inline const Eigen::MatrixXd &r_Matrix() const // throw(std::bad_alloc)
	{
		if(!m_b_dense_up_to_date) {
			m_sparse.Convert_to_Dense(m_matrix);
			m_b_dense_up_to_date = true;
		}
		return m_matrix;
	}

	/**
	 *	@brief determines whether a specific block in the marginals matrix is up-to-date
	 *
	 *	@param[in] n_block_row is zero-based block row index (index of the first vertex)
	 *	@param[in] n_block_column is zero-based block column index (index of the second vertex)
	 *
	 *	@return Returns true if the queried area is up-to-date, otherwise returns false.
	 *
	 *	@note This function just determines if the queried area is up-to-date but does
	 *		not update it. Thus, it is very fast and can be used for e.g. covariance polling.
	 */
	inline bool b_Block_UpToDate(size_t UNUSED(n_block_row), size_t UNUSED(n_block_column)) const
	{
		return true;
	}

	/**
	 *	@brief determines whether a specific block column in the marginals matrix is up-to-date
	 *	@param[in] n_block_column is zero-based block row index (index of the first vertex)
	 *	@return Returns true if the queried area is up-to-date, otherwise returns false.
	 *	@note This function just determines if the queried area is up-to-date but does
	 *		not update it. Thus, it is very fast and can be used for e.g. covariance polling.
	 */
	inline bool b_Column_UpToDate(size_t UNUSED(n_block_column)) const
	{
		return true;
	}

	/**
	 *	@brief determines whether the block diagonal in the marginals matrix is up-to-date
	 *	@return Returns true if the queried area is up-to-date, otherwise returns false.
	 *	@note This function just determines if the queried area is up-to-date but does
	 *		not update it. Thus, it is very fast and can be used for e.g. covariance polling.
	 */
	inline bool b_Diagonal_UpToDate() const
	{
		return true;
	}

	/**
	 *	@brief determines whether all of the marginals matrix is up-to-date
	 *	@return Returns true if the queried area is up-to-date, otherwise returns false.
	 *	@note This function just determines if the queried area is up-to-date but does
	 *		not update it. Thus, it is very fast and can be used for e.g. covariance polling.
	 */
	inline bool b_FullMatrix_UpToDate() const
	{
		return true;
	}

	/**
	 *	@brief makes sure that a block, corresponding to covariance of two vertices
	 *		is calculated (will perform the calculation if it is not)
	 *
	 *	@param[in] n_block_row is zero-based block row index (index of the first vertex)
	 *	@param[in] n_block_column is zero-based block column index (index of the second vertex)
	 *
	 *	@note Depending on the cache miss policy, this will possibly also calculate
	 *		other parts of the matrix (e.g. the whole block column or the full matrix).
	 *	@note This function throws std::bad_alloc.
	 */
	void Request_Block(size_t UNUSED(n_block_row), size_t UNUSED(n_block_column)) // throw(std::bad_alloc)
	{
		// at the moment, all of the matrix is available, requests have no effect.
		// this will change with more efficient algorithms in the future ...
	}

	/**
	 *	@brief makes sure that a block column, corresponding to covariance of one vertex
	 *		with all the vertices (including itself) is calculated (will perform the
	 *		calculation if it is not)
	 *
	 *	@param[in] n_block_column is zero-based block column index
	 *
	 *	@note Depending on the cache miss policy, this will possibly also calculate
	 *		other parts of the matrix (e.g. the full matrix).
	 *	@note This function throws std::bad_alloc.
	 */
	void Request_BlockColumn(size_t UNUSED(n_block_column)) // throw(std::bad_alloc)
	{
		// at the moment, all of the matrix is available, requests have no effect.
		// this will change with more efficient algorithms in the future ...
	}

	/**
	 *	@brief makes sure that block diagonal portion of the covariance matrix is calculated
	 *		(will perform the calculation if it is not)
	 *
	 *	The block diagonal means blocks at the diagonal (corresponds to auto-covariances of each
	 *	vertex with itself).
	 *
	 *	@note Depending on the cache miss policy, this will possibly also calculate
	 *		other parts of the matrix (e.g. the full matrix).
	 *	@note This function throws std::bad_alloc.
	 */
	void Request_BlockDiagonal() // throw(std::bad_alloc)
	{
		// at the moment, all of the matrix is available, requests have no effect.
		// this will change with more efficient algorithms in the future ...
	}

	/**
	 *	@brief makes sure that everything in the marginal covariance matrix is up-to-date
	 *	@note This function throws std::bad_alloc.
	 */
	void Request_Full() // throw(std::bad_alloc)
	{
		// at the moment, all of the matrix is available, requests have no effect.
		// this will change with more efficient algorithms in the future ...
	}

	/**
	 *	@brief diagonals of the diagonal blocks of the marginals to a file
	 *	@param[in] p_s_filename is a null-termibnated string with output filename (default "marginals.txt")
	 *	@return Returns true on success, false on failure.
	 *	@note This does not do any checking whether the matrix is up to date.
	 */
	bool Dump_Diagonal(const char *p_s_filename = "marginals.txt") const
	{
		const CUberBlockMatrix &r_marginals = r_SparseMatrix();
		FILE *p_fw;
		if((p_fw = fopen(p_s_filename, "w"))) {
			for(size_t i = 0, n = r_marginals.n_BlockColumn_Num(); i < n; ++ i) {
				size_t n_order = r_marginals.n_BlockColumn_Base(i);
				size_t n_dimension = r_marginals.n_BlockColumn_Column_Num(i);
				// get col

				CUberBlockMatrix::_TyConstMatrixXdRef block =
					r_marginals.t_FindBlock(n_order, n_order);
				// get block

				//fprintf(p_fw, "block_%d_%d = ", int(i), int(i));
				//CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw, block);
				// prints the matrix

				_ASSERTE(block.rows() == block.cols() && block.cols() == n_dimension);
				for(size_t i = 0; i < n_dimension; ++ i) {
					double f = block(i, i);
					fprintf(p_fw, (fabs(f) > 1)? ((i)? " %.15f" : "%.15f") : ((i)? " %.15g" : "%.15g"), f);
				}
				fprintf(p_fw, "\n");
				// print just the diagonal, one line per every vertex
			}
			if(ferror(p_fw)) {
				fclose(p_fw);
				return false;
			}
			return !fclose(p_fw);
		}
		// dump diagonal blocks of the marginals to a file

		return false;
	}
};

/** @} */ // end of group

#endif // !__MARGINALS_INCLUDED
