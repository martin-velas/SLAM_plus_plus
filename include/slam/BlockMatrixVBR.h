/*
								+---------------------------------+
								|                                 |
								|  ***   Über Block Matrix   ***  |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2016 |
								|                                 |
								|        BlockMatrixVBR.h         |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_VBR_FORMAT_SUPPORT_INCLUDED
#define __UBER_BLOCK_MATRIX_VBR_FORMAT_SUPPORT_INCLUDED

/**
 *	@file include/slam/BlockMatrixVBR.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief conversion of \ref CUberBlockMatrix to standard BLAS block formats (<tt>BSR</tt> and <tt>VBR</tt>)
 */

#include "slam/BlockMatrix.h"

/*#include "spblas.h"
#include "dbscmml.h"*/

/**
 *	@brief BLAS-style sparse block matrix (<tt>BSR</tt> or <tt>VBR</tt>)
 */
struct TBLAS_SparseBlockMatrix {
	struct TFromTransposeMatrix_Token {};
	static const TFromTransposeMatrix_Token from_transpose_matrix;

	typedef int Int; // int type
	typedef double Val; // value type

	Int m, n;
	Int bm, bn;
	Int nnz, bnnz; // bananaz
	Int *rpntr; // bm + 1 block row origins
	Int *cpntr; // bn + 1 block column origins
	Int lb; // if zero then this is VBR, if nonzero then this is BSR (all blocks have the same size)
	Int lb_r, lb_c; // extension; if zero then this is VBR, if nonzero then this is rectangular BSR (all blocks have the same size but are not square)
	Val *val; // nnz data storage
	Int *indx; // bnnz + 1 indices of each starting block and total nnz at the end
	Int *bindx; // bnnz block column indices (the same as the cs::i array)
	union {
		Int *bpntr; // bm + 1 indices of the first block in each block row and the number of blocks
		Int *bpntrb; // bm indices of the first block in each block row (points to bpntr)
	};
	Int *bpntre; // bm indices of one past the last block in each block row (points to bpntr + 1)

	TBLAS_SparseBlockMatrix()
		:m(0), n(0), bm(0), bn(0), nnz(0), bnnz(0), rpntr(0), cpntr(0),
		lb(0), lb_r(0), lb_c(0), val(0), indx(0), bindx(0), bpntr(0)/*, bpntrb(0)*/, bpntre(0)
	{}

	TBLAS_SparseBlockMatrix(Int _m, Int _n, Int _bm, Int _bn, Int _nnz, Int _bnnz, bool b_values)
		:m(0), n(0), bm(0), bn(0), nnz(0), bnnz(0), rpntr(0), cpntr(0),
		lb(0), lb_r(0), lb_c(0), val(0), indx(0), bindx(0), bpntr(0)/*, bpntrb(0)*/, bpntre(0)
	{
		Alloc(_m, _n, _bm, _bn, _nnz, _bnnz, b_values);
	}

	TBLAS_SparseBlockMatrix(const TBLAS_SparseBlockMatrix &r_other)
		:m(0), n(0), bm(0), bn(0), nnz(0), bnnz(0), rpntr(0), cpntr(0),
		lb(0), lb_r(0), lb_c(0), val(0), indx(0), bindx(0), bpntr(0)/*, bpntrb(0)*/, bpntre(0)
	{
		lb = r_other.lb;
		lb_r = r_other.lb_r;
		lb_c = r_other.lb_c;
		// those are not copied by alloc

		Alloc(r_other.m, r_other.n, r_other.bm, r_other.bn, r_other.nnz, r_other.bnnz, r_other.val != 0);
		if(r_other.rpntr)
			memcpy(rpntr, r_other.rpntr, (bm + 1) * sizeof(Int));
		if(r_other.cpntr)
			memcpy(cpntr, r_other.cpntr, (bn + 1) * sizeof(Int));
		if(r_other.val)
			memcpy(val, r_other.val, nnz * sizeof(Val));
		if(r_other.indx)
			memcpy(indx, r_other.indx, (bnnz + 1) * sizeof(Int));
		if(r_other.bindx)
			memcpy(bindx, r_other.bindx, bnnz * sizeof(Int));
		if(r_other.bpntr)
			memcpy(bpntr, r_other.bpntr, (bm + 1) * sizeof(Int));
		//bpntre = bpntr + 1; // in Alloc()
	}

	/**
	 *	@brief conversion from Über block matrix
	 *
	 *	@param[in] r_ubm is block matrix to convert from
	 *	@param[in] n_index_base is index base (0 for "C", 1 for Fortran)
	 *	@param[in] token is token for the caller to be aware that this
	 *		transposes the matrix (from column-major CUberBlockMatrix to row major VBR)
	 *
	 *	@note This transposes the matrix.
	 */
	TBLAS_SparseBlockMatrix(const CUberBlockMatrix &r_ubm, size_t n_index_base, TFromTransposeMatrix_Token UNUSED(token))
		:m(0), n(0), bm(0), bn(0), nnz(0), bnnz(0), rpntr(0), cpntr(0),
		lb(0), lb_r(0), lb_c(0), val(0), indx(0), bindx(0), bpntr(0)/*, bpntrb(0)*/, bpntre(0)
	{
		size_t n_nnz = r_ubm.n_NonZero_Num();
		size_t n_bnnz = r_ubm.n_Block_Num();
		// need to calculate those

		_ASSERTE(r_ubm.n_Row_Num() <= CMaxIntValue<Int>::n_result);
		_ASSERTE(r_ubm.n_Column_Num() <= CMaxIntValue<Int>::n_result);
		_ASSERTE(r_ubm.n_BlockRow_Num() <= CMaxIntValue<Int>::n_result);
		_ASSERTE(r_ubm.n_BlockColumn_Num() <= CMaxIntValue<Int>::n_result);
		_ASSERTE(n_nnz <= CMaxIntValue<Int>::n_result);
		_ASSERTE(n_bnnz <= CMaxIntValue<Int>::n_result);

		Alloc(Int(r_ubm.n_Column_Num()), Int(r_ubm.n_Row_Num()), Int(r_ubm.n_BlockColumn_Num()),
			Int(r_ubm.n_BlockRow_Num()), Int(n_nnz), Int(n_bnnz), true);
		// allocate transpose matrix

		for(size_t i = 0, o = bm; i < o; ++ i)
			rpntr[i] = Int(r_ubm.n_BlockColumn_Base(i) + n_index_base);
		rpntr[bm] = Int(m + n_index_base);
		for(size_t i = 0, o = bn; i < o; ++ i)
			cpntr[i] = Int(r_ubm.n_BlockRow_Base(i) + n_index_base);
		cpntr[bn] = Int(n + n_index_base);
		// fill block row and block column origins (transpose!)

		size_t n_blocks_rows = (r_ubm.n_BlockColumn_Num())? r_ubm.n_BlockColumn_Column_Num(0) : 0; // transpose!
		size_t n_blocks_cols = (r_ubm.n_BlockRow_Num())? r_ubm.n_BlockRow_Row_Num(0) : 0; // transpose!
		//size_t n_blocks_size = (r_ubm.n_BlockColumn_Num())? r_ubm.n_BlockColumn_Column_Num(0) : 0;

		Val *p_dest = val;
		Int *p_first_block = bpntr, /**p_last_block = bpntre,*/ // not separate anymore
			*p_block_data_off = indx, *p_block_column = bindx;
		size_t n_block_num_sum = 0;
		for(size_t i = 0, o = bm; i < o; ++ i) {
			size_t n_block_num = r_ubm.n_BlockColumn_Block_Num(i);
			size_t n_width = r_ubm.n_BlockColumn_Column_Num(i);

			/*if(n_width != n_blocks_size)
				n_blocks_size = 0;*/
			if(n_width != n_blocks_rows)
				n_blocks_rows = 0; // transpose!

			for(size_t j = 0; j < n_block_num; ++ j) {
				size_t n_block_row = r_ubm.n_Block_Row(i, j);
				size_t n_height = r_ubm.n_BlockRow_Row_Num(n_block_row);
				CUberBlockMatrix::_TyConstMatrixXdRef t_block = r_ubm.t_Block_AtColumn(i, j);
				// get nnz block

				/*if(n_height != n_blocks_size)
					n_blocks_size = 0;*/
				if(n_height != n_blocks_cols)
					n_blocks_cols = 0; // transpose!

				Eigen::Map<Eigen::Matrix<Val, Eigen::Dynamic,
					Eigen::Dynamic> > dest_block(p_dest, n_width, n_height);
				dest_block.noalias() = t_block.transpose(); // tranpose the individual blocks as well!
				// copy matrix elements - fill val
				// note that those are stored as column major even though this is BSR/VBR

				*p_block_data_off = (Int)((p_dest - val) + n_index_base);
				++ p_block_data_off;
				*p_block_column = (Int)(n_block_row + n_index_base);
				++ p_block_column;
				// fill indx and bindx

				p_dest += n_width * n_height;
				// offset dest ptr afterwards!
			}

			*p_first_block = (Int)(n_block_num_sum + n_index_base);
			++ p_first_block;
			n_block_num_sum += n_block_num;
			// fill bpntr / bpntrb

			/**p_last_block = (Int)n_block_num_sum;
			++ p_last_block;
			// fill bpntre*/ // not separate anymore
		}

		*p_first_block = (Int)(n_block_num_sum + n_index_base);
		_ASSERTE(n_block_num_sum == bnnz);
		_ASSERTE(p_first_block == bpntrb + bm);
		//_ASSERTE(p_last_block == bpntre + bm); // not separate anymore
		*p_block_data_off = Int(nnz + n_index_base);
		_ASSERTE(p_block_data_off == indx + bnnz);
		_ASSERTE(p_block_column == bindx + bnnz);
		_ASSERTE(p_dest == val + nnz);
		// make sure all the arrays were filled as expected

		lb = Int((n_blocks_cols == n_blocks_rows)? n_blocks_cols : 0);
		lb_r = Int(n_blocks_rows);
		lb_c = Int(n_blocks_cols);
		// if all the blocks are of the same size, then this is set to that size
	}

	~TBLAS_SparseBlockMatrix()
	{
		Delete();
	}

	TBLAS_SparseBlockMatrix &operator =(const TBLAS_SparseBlockMatrix &r_other)
	{
		if(&r_other != this) {
			TBLAS_SparseBlockMatrix copy(r_other);
			copy.Swap(*this);
		}
		return *this;
	}

	void Swap(TBLAS_SparseBlockMatrix &r_other)
	{
		std::swap(m, r_other.m);
		std::swap(n, r_other.n);
		std::swap(bm, r_other.bm);
		std::swap(bn, r_other.bn);
		std::swap(nnz, r_other.nnz);
		std::swap(bnnz, r_other.bnnz);
		std::swap(rpntr, r_other.rpntr);
		std::swap(cpntr, r_other.cpntr);
		std::swap(lb, r_other.lb);
		std::swap(lb_r, r_other.lb_r);
		std::swap(lb_c, r_other.lb_c);
		std::swap(val, r_other.val);
		std::swap(indx, r_other.indx);
		std::swap(bindx, r_other.bindx);
		std::swap(bpntr, r_other.bpntr);
		//std::swap(bpntrb, r_other.bpntrb); // nope!
		std::swap(bpntre, r_other.bpntre); // yes
	}

	void Alloc(Int _m, Int _n, Int _bm, Int _bn, Int _nnz, Int _bnnz, bool b_values)
	{
		Delete();
		// delete any arrays there might have been

		try {
			rpntr = new Int[_bm + 1];
			cpntr = new Int[_bn + 1];
			if(b_values)
				val = new Val[_nnz];
			indx = new Int[_bnnz + 1];
			bindx = new Int[_bnnz];
			bpntr = bpntrb = new Int[_bm + 1];
			bpntre = bpntr + 1; // !!
		} catch(std::bad_alloc &r_exc) {
			Delete();
			throw r_exc;
		} catch(std::length_error &r_exc) {
			Delete();
			throw r_exc;
		}
		// allocate all the arrays

		m = _m;
		n = _n;
		bm = _bm;
		bn = _bn;
		nnz = _nnz;
		bnnz = _bnnz;
		// copy the dimensions
	}

	void Delete()
	{
		if(rpntr)
			delete[] rpntr;
		if(cpntr)
			delete[] cpntr;
		if(val)
			delete[] val;
		if(indx)
			delete[] indx;
		if(bindx)
			delete[] bindx;
		if(bpntr)
			delete[] bpntr;
		// do not delete bpntrb and bpntre
		m = 0;
		n = 0;
		bm = 0;
		bn = 0;
		nnz = 0;
		bnnz = 0;
		rpntr = 0;
		cpntr = 0;
		lb = 0;
		lb_r = 0;
		lb_c = 0;
		val = 0;
		indx = 0;
		bindx = 0;
		bpntr = 0;
		bpntrb = 0;
		bpntre = 0;
	}
};

#endif // !__UBER_BLOCK_MATRIX_VBR_FORMAT_SUPPORT_INCLUDED
