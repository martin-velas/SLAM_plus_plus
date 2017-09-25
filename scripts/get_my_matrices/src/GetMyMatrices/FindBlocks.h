#pragma once

#include <vector>
#include <algorithm>
#include "../UberLame_src/Unused.h"
#include "Sparse.h"

/**
 *	@brief routines ported from Fortran from the SPARSKIT packabe by Yousef Saad
 *
 *  -----------------------------------------------------------------------
 *                     S P A R S K I T   V E R S I O N  2.
 *  ----------------------------------------------------------------------- 
 *   
 *  Latest update : Tue Mar  8 11:01:12 CST 2005
 *   
 *  -----------------------------------------------------------------------
 *   
 *  Welcome  to SPARSKIT  VERSION  2.  SPARSKIT is  a  package of  FORTRAN
 *  subroutines  for working  with  sparse matrices.  It includes  general
 *  sparse  matrix  manipulation  routines  as  well as  a  few  iterative
 *  solvers, see detailed description of contents below.
 *   
 *   Copyright (C) 2005, the Regents of the University of Minnesota 
 *   
 *  SPARSKIT is  free software; you  can redistribute it and/or  modify it
 *  under the terms of the  GNU Lesser General Public License as published
 *  by the  Free Software Foundation [version  2.1 of the  License, or any
 *  later version.]
 *   
 *  A copy of  the licencing agreement is attached in  the file LGPL.  For
 *  additional information  contact the Free Software  Foundation Inc., 59
 *  Temple Place - Suite 330, Boston, MA 02111, USA or visit the web-site
 *   
 *   http://www.gnu.org/copyleft/lesser.html
 *
 */
namespace sparskit {

void csrkvstr(const size_t n, const size_t *ia/*[n + 1]*/,
	const size_t *ja, std::vector<size_t> &kvstr)
{
	// c-----------------------------------------------------------------------
	//  integer n, ia(n+1), ja(*), nr, kvstr(*)
	//c-----------------------------------------------------------------------
	//c     Finds block row partitioning of matrix in CSR format.
	//c-----------------------------------------------------------------------
	//c     On entry:
	//c--------------
	//c     n       = number of matrix scalar rows
	//c     ia,ja   = input matrix sparsity structure in CSR format
	//c
	//c     On return:
	//c---------------
	//c     nr      = number of block rows
	//c     kvstr   = first row number for each block row
	//c
	//c     Notes:
	//c-----------
	//c     Assumes that the matrix is sorted by columns.
	//c     This routine does not need any workspace.
	//c
	//c-----------------------------------------------------------------------
	//c     local variables
	//size_t i, j, jdiff;
	//c-----------------------------------------------------------------------
	kvstr.clear();
	kvstr.push_back(0);
	/*nr = 1;
	kvstr[1 - 1] = 1 - 1;*/
	//c---------------------------------
	for(size_t i = 2; i <= n; ++ i) {
		const size_t jdiff = ia[i] - ia[i - 1]; // number of nnz in row i-1
		if(jdiff == ia[i - 1] - ia[i - 2]) { // is it the same as in row i-2?
			for(size_t j = ia[i - 1]; j < ia[i]; ++ j) { // for all nonzeros in row i-1
				if(ja[j] != ja[j - jdiff]) { // are the nonzeros on the same columns?
					/*nr = nr + 1;
					kvstr[nr - 1] = i - 1;*/ // the block row ends here
					kvstr.push_back(i - 1);
					break;
				}
			}
		} else {
			/*nr = nr + 1;
            kvstr[nr - 1] = i - 1;*/ // the block row ends here
			kvstr.push_back(i - 1);
		}
	}
    //kvstr[nr+1 - 1] = n+1 - 1; // one more entry, where the matrix ends
	kvstr.push_back(n);
	// c---------------------------------
}

void csrkvstr_tol(const size_t n, const size_t *ia/*[n + 1]*/,
	const size_t *ja, std::vector<size_t> &kvstr, size_t n_tol_divider, size_t n_max_tol = SIZE_MAX)
{
	kvstr.clear();
	kvstr.push_back(0);

	size_t n_nnz_prev = ia[1] - ia[0];
	for(size_t i = 2; i <= n; ++ i) {
		const size_t n_nnz_i = ia[i] - ia[i - 1]; // number of nnz in row i-1
		size_t n_max_diff_num = min(n_max_tol, min(n_nnz_i, n_nnz_prev) / n_tol_divider); // how many nnz can differ in these two columns

		size_t n_diff = max(n_nnz_i, n_nnz_prev) - min(n_nnz_i, n_nnz_prev); // absolute value
		if(n_diff <= n_max_diff_num) { // the difference in the nnz in the two columns is not larger than the thresh
			size_t jc = ia[i - 1], ec = ia[i];
			size_t jp = ia[i - 2], ep = ia[i - 1];
			++ n_max_diff_num; // offset so that zero means failure and positive numbers mean there is some leaway left, even if the threshold is 0
			while(n_max_diff_num) {
				if(jc == ec) { // the current column exhausted
					if(jp == ep) // the prev as well, nothing more to compare
						break;
					++ jp;
					-- n_max_diff_num; // there is a nnz in prev column which does not match the current column
				} else if(jp == ep) {
					++ jc;
					-- n_max_diff_num; // there is a nnz in prev column which does not match the current column
				} else if(ja[jc] == ja[jp]) {
					++ jc;
					++ jp; // there is a nonzero in both current and prev columns
				} else if(ja[jc] < ja[jp]) {
					++ jc;
					-- n_max_diff_num; // there is a nonzero in current column and a zero in prev column
				} else {
					_ASSERTE(ja[jc] > ja[jp]);
					++ jp;
					-- n_max_diff_num; // there is a nonzero in previous column and a zero in current column
				}
			}
			// count differences between the two columns

			if(!n_max_diff_num) // exhausted the limit? the columns are too similar
				kvstr.push_back(i - 1);
		} else
			kvstr.push_back(i - 1);

		n_nnz_prev = n_nnz_i;
	}

	kvstr.push_back(n);
}

void csrkvstc(const size_t n, const size_t *ia/*[n + 1]*/,
	const size_t *ja, std::vector<size_t> &kvstc, size_t *iwk, const size_t UNUSED(n_iwk_size))
{
	//c-----------------------------------------------------------------------
	//      integer n, ia(n+1), ja(*), nc, kvstc(*), iwk(*)
	//c-----------------------------------------------------------------------
	//c     Finds block column partitioning of matrix in CSR format.
	//c-----------------------------------------------------------------------
	//c     On entry:
	//c--------------
	//c     n       = number of matrix scalar rows
	//c     ia,ja   = input matrix sparsity structure in CSR format
	//c
	//c     On return:
	//c---------------
	//c     nc      = number of block columns
	//c     kvstc   = first column number for each block column
	//c
	//c     Work space:
	//c----------------
	//c     iwk(*) of size equal to the number of scalar columns plus one.
	//c        Assumed initialized to 0, and left initialized on return.
	//c
	//c     Notes:
	//c-----------
	//c     Assumes that the matrix is sorted by columns.
	//c
	//c-----------------------------------------------------------------------
	//c     local variables
	//      integer i, j, k, ncol
	//c
	//c-----------------------------------------------------------------------
	//c-----use ncol to find maximum scalar column number
#ifdef _DEBUG
	size_t ncol_ref = *std::max_element(ja, ja + ia[n]) + 1;
	_ASSERTE(n_iwk_size > ncol_ref); // size equal to the number of scalar columns plus one
	_ASSERTE(*std::max_element(iwk, iwk + ncol_ref) == 0); //  (iwk is unsigned)
#endif // _DEBUG
	size_t ncol = 0; // this could have been an input (number of columns)
	//c-----mark the beginning position of the blocks in iwk
	size_t i, j, k;
	for(i = 1; i <= n; ++ i) {
		if(ia[i - 1] < ia[i]) { // are there nnz in row i - 1?
			j = ja[ia[i - 1]]; // the first entry in row i - 1
			_ASSERTE(n_iwk_size > j);
			iwk[j] = 1; // mark
			for(k = ia[i - 1] + 1; k < ia[i]; ++ k) {
				j = ja[k]; // the next entry in row i - 1
				_ASSERTE(ja[k - 1] + 1 <= j); // this is smaller or equal, unless the matrix is unsorted
				if(ja[k - 1] != j - 1) { // is there a gap?
					_ASSERTE(n_iwk_size > j);
					iwk[j] = 1; // mark the end of the zero run
					iwk[ja[k - 1] + 1] = 1; // and the start of it
				}
			}
			iwk[j + 1] = 1; // mark
			ncol = std::max(ncol, j); // get the highest nnz column index
		}
	}
	++ ncol; // we use 0-based indices, need to add one to get the count
	//c---------------------------------
	/*nc = 1;
	kvstc[1-1] = 1-1;*/
	kvstc.clear();
	kvstc.push_back(0);
	for(i = 2; i <= ncol; ++ i) {
		if(iwk[i - 1] != 0) { // is there a mark?
			/*nc = nc + 1;
			kvstc[nc - 1] = i;*/
			kvstc.push_back(i - 1);
			iwk[i - 1] = 0; // re-clear the workspace
		}
	}
	iwk[0] = 0; // !!
	_ASSERTE(kvstc.back() < ncol);
	kvstc.push_back(ncol);
	//_ASSERTE(kvstc.back() == ncol); // filled in the loop above
	//nc = nc - 1;
	//c---------------------------------
}

void kvstmerge(const size_t nr, const size_t *kvstr, const size_t nc, const size_t *kvstc, size_t &n, size_t *kvst)
{
	//c-----------------------------------------------------------------------
	//      integer nr, kvstr(nr+1), nc, kvstc(nc+1), n, kvst(*)
	//c-----------------------------------------------------------------------
	//c     Merges block partitionings, for conformal row/col pattern.
	//c-----------------------------------------------------------------------
	//c     On entry:
	//c--------------
	//c     nr,nc   = matrix block row and block column dimension
	//c     kvstr   = first row number for each block row
	//c     kvstc   = first column number for each block column
	//c
	//c     On return:
	//c---------------
	//c     n       = conformal row/col matrix block dimension
	//c     kvst    = conformal row/col block partitioning
	//c
	//c     Notes:
	//c-----------
	//c     If matrix is not square, this routine returns without warning.
	//c
	//c-----------------------------------------------------------------------
	//c-----local variables
	//      integer i,j
	//c---------------------------------
	if(kvstr[nr] != kvstc[nc])
		return; // not square
	size_t i = 1;
	size_t j = 1;
	n = 1;
	for(;;) {
		if(i > nr+1) { // all rows done
			kvst[n - 1] = kvstc[j - 1];
			j = j + 1;
		} else if(j > nc+1) { // all cols done
			kvst[n - 1] = kvstr[i - 1];
			i = i + 1;
		} else if(kvstc[j - 1] == kvstr[i - 1]) { // the same block start
			kvst[n - 1] = kvstc[j - 1];
			j = j + 1;
			i = i + 1;
		} else if(kvstc[j - 1] < kvstr[i - 1]) { // 
			kvst[n - 1] = kvstc[j - 1];
			j = j + 1;
		} else {
			kvst[n - 1] = kvstr[i - 1];
			i = i + 1;
		}
		n = n + 1;
		if(i <= nr+1 || j <= nc+1)
			continue;
		n = n - 2;
		break;
	}
}

} // ~sparskit

void Find_BlockLayout_SPARSKIT(std::vector<size_t> &r_block_rows,
	std::vector<size_t> &r_block_cols, const TSparse &r_mat) // throw(std::bad_alloc)
{
	sparskit::csrkvstr(r_mat.n, r_mat.p, r_mat.i, r_block_cols);
	std::vector<size_t> workspace(r_mat.m + 1, 0); // transpose! expects CSR!
	sparskit::csrkvstc(r_mat.n, r_mat.p, r_mat.i, r_block_rows, &workspace.front(), workspace.size());
	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 &&
		r_block_rows.back() <= r_mat.m); if(r_block_rows.back() != r_mat.m) r_block_rows.push_back(r_mat.m); // some matrices are have empty rows at the bottom, then this is lower
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 &&
		r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end())); // must be also strictly increasing
}

void Find_BlockLayout_SPARSKIT_Tol(std::vector<size_t> &r_block_rows,
	std::vector<size_t> &r_block_cols, const TSparse &r_mat, size_t n_tol = 5) // throw(std::bad_alloc)
{
	sparskit::csrkvstr_tol(r_mat.n, r_mat.p, r_mat.i, r_block_cols, n_tol); // allow every 5th nnz to differ for two columns to fall in the same block column
	{
		TSparse tr;
		r_mat.Transpose(tr); // need a transpose, difficult to do the merge in sparskit::csrkvstr_tol() for rows in a CSC matrix
		sparskit::csrkvstr_tol(tr.n, tr.p, tr.i, r_block_rows, 5); // allow every 5th nnz to differ for two rows to fall in the same block row
	}
	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == r_mat.m);
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end())); // must be also strictly increasing
}

class CCalculateBlockRowNNZ {
protected:
	std::vector<size_t>::const_iterator m_p_block_row_it;
	size_t m_n_last_block_row_base;
	size_t m_n_nnz;

public:
	CCalculateBlockRowNNZ(const std::vector<size_t> &r_block_rows)
		:m_p_block_row_it(r_block_rows.begin()), m_n_last_block_row_base(0), m_n_nnz(0)
	{}

	void operator ()(bool b_nonzero_row)
	{
		size_t n_next_block_row_base = *(++ m_p_block_row_it);
		if(b_nonzero_row)
			m_n_nnz += n_next_block_row_base - m_n_last_block_row_base;
		m_n_last_block_row_base = n_next_block_row_base;
	}

	operator size_t() const
	{
		return m_n_nnz;
	}
};

size_t n_SparseMatrix_BlockCol_FillIn_SlowDbg(const std::vector<size_t> &r_block_rows,
	size_t n_base, size_t n_end, const TSparse &r_mat, std::vector<bool> &r_workspace,
	size_t n_fill_in_thresh = 0) // very slow, only used in debug (it basically turns the problem into dense computation)
{
	std::vector<bool> &block_row_flags = r_workspace;

	size_t n_col_nnz_sum = 0;
	block_row_flags.assign(r_block_rows.size() - 1, false); // clear the temp array
	for(size_t c = n_base; c < n_end; ++ c) { // for each element column
		std::vector<size_t>::const_iterator p_br_it = r_block_rows.begin();
		for(size_t p = r_mat.p[c], e = r_mat.p[c + 1]; p < e; ++ p) { // for each element nonzero
			_ASSERTE(p == r_mat.p[c] || r_mat.i[p] >= r_mat.i[p - 1]); // make sure the elements are sorted by row
			size_t r = r_mat.i[p];
			p_br_it = std::upper_bound(p_br_it, r_block_rows.end(), r) - 1; // start where we last left off
			_ASSERTE(p_br_it >= r_block_rows.begin() && p_br_it < r_block_rows.end());
			_ASSERTE(r >= *p_br_it && r < *(p_br_it + 1));
			size_t br = p_br_it - r_block_rows.begin();
			block_row_flags[br] = true;
		}
		n_col_nnz_sum += r_mat.p[c + 1] - r_mat.p[c];
	}
	// count element-wise nonzeros in columns inside this block column
	// create a binary mask of the block row nonzero pattern

	size_t n_col_fill_in = std::for_each(block_row_flags.begin(),
		block_row_flags.end(), CCalculateBlockRowNNZ(r_block_rows));
	// take a sum over the nonzero pattern

	const size_t n_width = n_end - n_base;
	n_col_fill_in *= n_width;
	// multiply by block column width

	_ASSERTE(n_col_fill_in >= n_col_nnz_sum);
	return n_col_fill_in - n_col_nnz_sum;
	// subtract to get the fill-in figure
}

size_t n_SparseMatrix_BlockCol_FillIn_Lookup(const std::vector<size_t> &r_block_rows,
	const std::vector<size_t> &r_block_row_lookup, // wouldnt get much slower by losing this workspace
	size_t n_base, size_t n_end, const TSparse &r_mat, std::vector<bool> &r_workspace, // workspace expected to only contain zeros on entry/return
	size_t n_fill_in_thresh = 0)
{
	const size_t n_width = n_end - n_base;
	size_t n_col_nnz_sum = r_mat.p[n_end] - r_mat.p[n_base];

	std::vector<size_t> block_row_occurences;
	size_t n_col_fill_in = 0, n_col_nnz_sum_dbg = 0;
	if(n_col_nnz_sum / n_width < r_block_rows.size() / 100) { // handle very sparse matrices with many block rows
		for(size_t c = n_base; c < n_end; ++ c) { // for each element column
			for(size_t p = r_mat.p[c], e = r_mat.p[c + 1]; p < e; ++ p) { // for each element nonzero
				size_t r = r_mat.i[p];
				size_t br = r_block_row_lookup[r];
				_ASSERTE(r >= r_block_rows[br] && r < r_block_rows[br + 1]);
				block_row_occurences.push_back(br);
			}

			std::sort(block_row_occurences.begin(), block_row_occurences.end());
			block_row_occurences.erase(std::unique(block_row_occurences.begin(),
				block_row_occurences.end()), block_row_occurences.end());
			// compact after each column; keeps workspace to 2m at worst (m being the number of element rows)

#ifdef _DEBUG
			n_col_nnz_sum_dbg += r_mat.p[c + 1] - r_mat.p[c];
#endif // _DEBUG
		}
		// collect a vector of block row occurences

		for(size_t i = 0, n = block_row_occurences.size(); i < n; ++ i)
			n_col_fill_in += r_block_rows[block_row_occurences[i] + 1] - r_block_rows[block_row_occurences[i]];
		// count the fill-in
	} else if(n_width == 1) {
		const size_t c = n_base; // for the only element column
		for(size_t p = r_mat.p[c], e = r_mat.p[c + 1]; p < e;) { // for each element nonzero
			_ASSERTE(p == r_mat.p[c] || r_mat.i[p] >= r_mat.i[p - 1]); // make sure the elements are sorted by row
			size_t r = r_mat.i[p];
			size_t br = r_block_row_lookup[r];
			_ASSERTE(r >= r_block_rows[br] && r < r_block_rows[br + 1]);
			// get an element, lookup block row

			n_col_fill_in += r_block_rows[br + 1] - r_block_rows[br];
			// account for the fill-in

			size_t p_new = std::lower_bound(r_mat.i + p + 1, r_mat.i + e, r_block_rows[br + 1]) - r_mat.i;
			// resume with the element from the next block row

			_ASSERTE(p_new > p); // make sure we don't get stuck
#ifdef _DEBUG
			for(++ p; p < p_new; ++ p)
				_ASSERTE(r_block_row_lookup[r_mat.i[p]] == br);
			// make sure that all the skipped elements were in the same block row
#endif // _DEBUG
			_ASSERTE(p_new == e || r_block_row_lookup[r_mat.i[p_new]] > br);
			// make sure the next element is in (one of) the next block row(s)

			p = p_new;
		}
		// count block rows, skip over the next elements (works only for a single column,
		// otherwise there could be the same block rows counted several times)

#ifdef _DEBUG
		n_col_nnz_sum_dbg += r_mat.p[c + 1] - r_mat.p[c];
#endif // _DEBUG
	} else {
		r_workspace.resize(r_block_rows.size(), false);
		_ASSERTE(std::find(r_workspace.begin(), r_workspace.end(), true) == r_workspace.end());
		// prepare the workspace, make sure it is all zeros (this only allocates the workspace
		// in the first iteration and in the subsequent ones it doesn't do anything)

		for(size_t c = n_base; c < n_end; ++ c) { // for each element column
			for(size_t p = r_mat.p[c], e = r_mat.p[c + 1]; p < e;) { // for each element nonzero
				_ASSERTE(p == r_mat.p[c] || r_mat.i[p] >= r_mat.i[p - 1]); // make sure the elements are sorted by row
				size_t r = r_mat.i[p];
				size_t br = r_block_row_lookup[r];
				_ASSERTE(r >= r_block_rows[br] && r < r_block_rows[br + 1]);
				// get an element, lookup block row

				if(!r_workspace[br]) {
					r_workspace[br].flip();
					n_col_fill_in += r_block_rows[br + 1] - r_block_rows[br];
					block_row_occurences.push_back(br);
				}
				// account for the fill-in

				size_t p_new = std::lower_bound(r_mat.i + p + 1, r_mat.i + e, r_block_rows[br + 1]) - r_mat.i;
				// resume with the element from the next block row

				_ASSERTE(p_new > p); // make sure we don't get stuck
#ifdef _DEBUG
				for(++ p; p < p_new; ++ p)
					_ASSERTE(r_block_row_lookup[r_mat.i[p]] == br);
				// make sure that all the skipped elements were in the same block row
#endif // _DEBUG
				_ASSERTE(p_new == e || r_block_row_lookup[r_mat.i[p_new]] > br);
				// make sure the next element is in (one of) the next block row(s)

				p = p_new;
			}

#ifdef _DEBUG
			n_col_nnz_sum_dbg += r_mat.p[c + 1] - r_mat.p[c];
#endif // _DEBUG
		}

		for(size_t i = 0, n = block_row_occurences.size(); i < n; ++ i)
			r_workspace[block_row_occurences[i]].flip();
		_ASSERTE(std::find(r_workspace.begin(), r_workspace.end(), true) == r_workspace.end()); // make sure it is all zeros again
		// rezero the workspace array
	}
	_ASSERTE(n_col_nnz_sum_dbg == n_col_nnz_sum);

	n_col_fill_in *= n_width;
	// multiply by block column width

	_ASSERTE(n_col_fill_in >= n_col_nnz_sum);
	return n_col_fill_in - n_col_nnz_sum;
	// subtract to get the fill-in figure
}

size_t n_SparseMatrix_Block_FillIn(const std::vector<size_t> &r_block_rows,
	const std::vector<size_t> &r_block_cols, const TSparse &r_mat,
	size_t n_fill_in_thresh = 0, size_t n_first_block_col = 0, size_t n_last_block_col = size_t(-1)) // throw(std::bad_alloc)
{
	_ASSERTE(n_first_block_col < r_block_cols.size());
	_ASSERTE(n_last_block_col == size_t(-1) || n_last_block_col <= r_block_cols.size());
	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == r_mat.m);
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end())); // must be also strictly increasing

	size_t n_fill_in = 0;

	std::vector<size_t> block_row_lookup(r_mat.m);
	for(size_t i = 1, j = 0, n = r_block_rows.size(); i < n; ++ i) {
		for(const size_t m = r_block_rows[i]; j < m; ++ j)
			block_row_lookup[j] = i - 1;
	}
	// create a lookup for block rows

	std::vector<bool> block_row_flags;
	if(n_last_block_col == size_t(-1))
		n_last_block_col = r_block_cols.size();
	for(size_t bc = n_first_block_col + 1, nbc = n_last_block_col; bc < nbc; ++ bc) { // for each block column
		const size_t n_base = r_block_cols[bc - 1], n_end = r_block_cols[bc];

		size_t n_col_fill_in = n_SparseMatrix_BlockCol_FillIn_Lookup(r_block_rows, block_row_lookup, n_base, n_end,
			r_mat, block_row_flags, n_fill_in_thresh);

		n_fill_in += n_col_fill_in;
		// accumulate for the whole matrix

		if(n_fill_in_thresh && n_fill_in > n_fill_in_thresh)
			return n_fill_in;
		// in case we only allow some small fill in and are already over the budget, don't bother counting
	}

	return n_fill_in;
}

static struct TModifyBlockLayout_Stats {
	size_t n_call_num;
	size_t n_fillin_eval_num;
	size_t n_fillin_parteval_num;
	size_t n_fillin_bceval_num;
	size_t n_accept_num;
	size_t n_reject_fillin_num;
	size_t n_reject_geometry_num;
	size_t n_total_accepted_fillin;
	size_t n_total_accepted_nnz;

	TModifyBlockLayout_Stats()
		:n_call_num(0), n_fillin_eval_num(0), n_fillin_parteval_num(0),
		n_fillin_bceval_num(0), n_accept_num(0), n_reject_fillin_num(0),
		n_reject_geometry_num(0), n_total_accepted_fillin(0), n_total_accepted_nnz(0)
	{}

	~TModifyBlockLayout_Stats()
	{
		if(n_call_num) {
			printf("debug: Modify_BlockLayout_Greedy() stats:\n"
				"\tcalled " PRIsize " times\n"
				"\tfill-in evaluated " PRIsize " times\n"
				"\t\tout of that " PRIsize " times partially\n"
				"\t\tevaluated fill-in in " PRIsize " block columns\n"
				"\taccepted " PRIsize " matrices\n"
				"\trejected " PRIsize " matrices (" PRIsize " exceeded fill-in, " PRIsize " did not partition)\n"
				"\tgenerated %.2f%% fill-in (" PRIsize " elements)\n",
				n_call_num, n_fillin_eval_num + n_fillin_parteval_num, n_fillin_parteval_num,
				n_fillin_bceval_num, n_accept_num, n_reject_fillin_num + n_reject_geometry_num,
				n_reject_fillin_num, n_reject_geometry_num, 100.0f * n_total_accepted_fillin /
				n_total_accepted_nnz, n_total_accepted_fillin);
		}
	}
} mbl_stats;

typedef std::pair<std::map<std::pair<size_t, size_t>, size_t>, std::vector<size_t> > TBlockColFillinCache;

size_t n_SparseMatrix_Block_FillIn_Incremental(const std::vector<size_t> &r_block_rows,
	const std::vector<size_t> &r_block_cols, const TSparse &r_mat,
	size_t n_fill_in_thresh = 0, TBlockColFillinCache *p_cache = 0) // throw(std::bad_alloc)
{
	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == r_mat.m);
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end())); // must be also strictly increasing

	size_t n_fill_in = 0;

	std::vector<size_t> block_row_lookup_storage;
	std::vector<size_t> &r_block_row_lookup = (p_cache)? p_cache->second : block_row_lookup_storage;
	if(r_block_row_lookup.empty()) {
		r_block_row_lookup.resize(r_mat.m);
		for(size_t i = 1, j = 0, n = r_block_rows.size(); i < n; ++ i) {
			for(const size_t m = r_block_rows[i]; j < m; ++ j)
				r_block_row_lookup[j] = i - 1;
		}
	}
	// create a lookup for block rows

	std::vector<bool> block_row_flags, block_row_flags_dbg;
	for(size_t bc = 1, nbc = r_block_cols.size(); bc < nbc; ++ bc) { // for each block column
		const size_t n_base = r_block_cols[bc - 1], n_end = r_block_cols[bc];

		size_t n_col_fill_in;
		TBlockColFillinCache::first_type::const_iterator p_record_it;
		if(p_cache && (p_record_it = p_cache->first.find(std::make_pair(n_base, n_end))) != p_cache->first.end()) {
			n_col_fill_in = (*p_record_it).second;
			_ASSERTE(n_col_fill_in == n_SparseMatrix_BlockCol_FillIn_SlowDbg(r_block_rows, n_base, n_end,
				r_mat, block_row_flags_dbg, n_fill_in_thresh));
		} else {
			++ mbl_stats.n_fillin_bceval_num;
			n_col_fill_in = n_SparseMatrix_BlockCol_FillIn_Lookup(r_block_rows, r_block_row_lookup, n_base, n_end,
				r_mat, block_row_flags, n_fill_in_thresh);
			_ASSERTE(n_col_fill_in == n_SparseMatrix_BlockCol_FillIn_SlowDbg(r_block_rows, n_base, n_end,
				r_mat, block_row_flags_dbg, n_fill_in_thresh));
			if(p_cache) {
				try {
					(p_cache->first)[std::make_pair(n_base, n_end)] = n_col_fill_in;
				} catch(std::bad_alloc&) {
					static bool b_warned = false;
					if(!b_warned) {
						b_warned = true;
						fprintf(stderr, "warning: not enough memory for block column fill-in cache\n");
					}
					// ignore errors here, will just run slower
				}
			}
		}
		// get / update fill-in for the column

		n_fill_in += n_col_fill_in;
		// accumulate for the whole matrix

		if(n_fill_in_thresh && n_fill_in > n_fill_in_thresh)
			return n_fill_in;
		// in case we only allow some small fill in and are already over the budget, don't bother counting
	}

	return n_fill_in;
}

class CIsNonzero {
public:
	template <class T>
	inline bool operator ()(const T &r_x) const
	{
		return r_x != 0;
	}

	inline bool operator ()(bool b_x) const
	{
		return b_x;
	}
};

size_t n_SparseMatrix_Block_Num(const std::vector<size_t> &r_block_rows,
	const std::vector<size_t> &r_block_cols, const TSparse &r_mat) // throw(std::bad_alloc)
{
	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == r_mat.m);
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end())); // must be also strictly increasing

	size_t n_block_num = 0;

	std::vector<bool> block_row_flags;
	for(size_t bc = 1, nbc = r_block_cols.size(); bc < nbc; ++ bc) { // for each block column
		const size_t n_base = r_block_cols[bc - 1], n_end = r_block_cols[bc];

		block_row_flags.assign(r_block_rows.size() - 1, false); // clear the temp array
		for(size_t c = n_base; c < n_end; ++ c) { // for each element column
			for(size_t p = r_mat.p[c], e = r_mat.p[c + 1]; p < e; ++ p) { // for each element nonzero
				size_t r = r_mat.i[p];
				std::vector<size_t>::const_iterator p_br_it = std::upper_bound(r_block_rows.begin(),
					r_block_rows.end(), r) - 1;
				_ASSERTE(p_br_it >= r_block_rows.begin() && p_br_it < r_block_rows.end());
				_ASSERTE(r >= *p_br_it && r < *(p_br_it + 1));
				size_t br = p_br_it - r_block_rows.begin();
				block_row_flags[br] = true;
			}
		}
		// count element-wise nonzeros in columns inside this block column
		// create a binary mask of the block row nonzero pattern

		size_t n_col_nnz = std::count_if(block_row_flags.begin(),
			block_row_flags.end(), CIsNonzero());
		// take a sum over the nonzero pattern

		n_block_num += n_col_nnz;
	}

	return n_block_num;
}

/**
 *	@brief greatest common divisor
 */
size_t n_GCD(size_t u, size_t v)
{
	_ASSERTE(u <= v); // must be ordered

	if(!u)
		return v;
	_ASSERTE(v > 0);
	// handle the simple cases; GCD(0, v) = v; GCD(u, 0) = u, GCD(0, 0) = 0

	int shift;
	for(shift = 0; ((u | v) & 1) == 0; ++ shift) {
		u >>= 1;
		v >>= 1;
	}
	// shift both u and v right

	while(!(u & 1))
		u >>= 1;
	// remove factors of 2 in u

	do {
		while(!(v & 1))
			v >>= 1;
		// remove factors of 2 in v (v becomes odd)

		if(u > v)
			std::swap(u, v);
		v -= u;
		// take a difference (v becomes even)
	} while(v);

	return u << shift;
	// restore the common factors of 2
}

/**
 *	@brief tests whether a given number can be written as a sum of integer
 *		multiples of positive summands (constrained integer parition or the coin problem)
 *
 *	This uses a simple enumeration algorithm which runs in exponential time.
 *	The order of enumeration gives multipliers such that the largest summands
 *	are used first and complemented by the smaller ones if needed, although
 *	the best solution is not always returned (it would require enumerating much
 *	more solutions and would be much slower).
 *
 *	This makes sense for block matrices where larger blocks yield larger speedups.
 *	Otherwise, one would probably prefer using the least summands possible.
 *
 *	This function requires O(m) memory where m is the number of summands.
 *
 *	@param[in] n is the positive integer to be partitioned
 *	@param[in] r_summands is a set of parts to comprise the sum
 *	@param[out] p_multipliers is pointer to a vector to be filled with multipliers
 *		of the numbers at the corresponding positions in r_summands that equal n
 *
 *	@return Returns true if a parition was found, otherwise returns false.
 *
 *	@note This function throws std::bad_alloc.
 */
bool Partition(size_t n, const std::set<size_t> &r_summands,
	std::vector<size_t> *p_multipliers = 0) // throw(std::bad_alloc)
{
	if(p_multipliers)
		p_multipliers->clear();

	size_t n_remainder = n;
	for(std::set<size_t>::const_reverse_iterator p_summand_it = r_summands.rbegin(),
	   p_end_it = r_summands.rend(); p_summand_it != p_end_it; ++ p_summand_it) {
		if(*p_summand_it)
			n_remainder %= *p_summand_it;
	}
	if(!n_remainder) {
		if(p_multipliers) {
			p_multipliers->resize(r_summands.size(), size_t(0));
			size_t n_remainder = n, n_idx = r_summands.size() - 1;
			for(std::set<size_t>::const_reverse_iterator p_summand_it = r_summands.rbegin(),
			   p_end_it = r_summands.rend(); p_summand_it != p_end_it; ++ p_summand_it, -- n_idx) {
				if(*p_summand_it) {
					(*p_multipliers)[n_idx] = n_remainder / *p_summand_it;
					n_remainder %= *p_summand_it;
				}
			}
		}
		// do it again, record the fractions as well

		return true;
	}
	// try a greedy approach of dividing by the largest, then by the next largest and so on
	// ... this catches quite many cases before allocating anything and always returns well
	// ordered results (the below code does not guarantee the large-summand-first ordering)

	if(!p_multipliers) {
		//size_t n_idx = r_summands.size() - 1;
		for(std::set<size_t>::const_iterator/*const_reverse_iterator*/ p_summand_it = r_summands./*r*/begin(),
		   p_end_it = r_summands./*r*/end(); p_summand_it != p_end_it; ++ p_summand_it/*, -- n_idx*/) {
			if(*p_summand_it && !(n % *p_summand_it)) {
				_ASSERTE(!p_multipliers);
				/*if(p_multipliers) { // dead code
					p_multipliers->resize(r_summands.size(), size_t(0));
					(*p_multipliers)[n_idx] = n / *p_summand_it; // just a multiple of this number
				}*/
				return true; // easy: just a multiple of this number
			}
		}
		// in case n is directly divisible by a single number
	}
	// this returns suboptimal solutions; use only if we don't want to return the multipliers

#if 1 // see if this helps to speed up
	if(r_summands.size() == 2) {
		size_t n_hi = *r_summands.begin(), n_lo = *(-- r_summands.end());
		// get the two numbers

		if(!p_multipliers || // either checked in the branch above, or check on the following line
		   ((!n_lo || (n % n_lo) != 0) && (!n_hi || (n % n_hi) != 0))) { // in case n is divisible by one of the parts then this would only accept and since we want to recover the multipliers, fast accept is worthless

			if(!n_lo) {
				_ASSERTE(n % n_hi != 0); // checked by the loop above
				return false;
			}
			// otherwise would have to be divisible by n_hi

			size_t n_gcd = n_GCD(n_lo, n_hi);
			_ASSERTE(!(n_lo % n_gcd) && !(n_hi % n_gcd)); // make sure it really is a divisor
			if(n % n_gcd != 0)
				return false; // e.g. both numbers are odd, can't sum up to an even number

			if(!p_multipliers) {
				n_lo /= n_gcd;
				n_hi /= n_gcd;
				// modify the problem, n_lo and n_hi are now coprime

				// do not modify n! may need to execute the code below

				size_t n_min = n_lo * n_hi + 1 - n_lo - n_hi; // + 1 to avoid underflow when n_lo == 1
				if(n / n_gcd >= n_min)
					return true;
				// all the numbers above n_lo * n_hi - n_lo - n_hi can be represented
			}
			// easy accept, see where n_lo and n_hi start generating all the consecutive numbers
			// (according to Schur's theorem, since n_lo and n_hi are coprime, any sufficiently
			// large integer can be expressed as a linear combination of the two)
		}
	}
	// closed-form solution for two numbers
#endif // 1

	std::vector<size_t> small_summands, max_multiples;
	for(std::set<size_t>::const_iterator p_summand_it = r_summands.begin(),
	   p_end_it = r_summands.end(); p_summand_it != p_end_it; ++ p_summand_it) {
		const size_t s = *p_summand_it;
		if(s && n >= s) { // in case it fits at least once
			small_summands.push_back(s);
			max_multiples.push_back(n / s + 1); // exclusive maximum
		} else
			break; // the set is sorted, there will be no more smaller ones (could use std::upper_bound to get the end ptr)
	}
	// threshold the summands and calculate the maximum multipliers

	if(p_multipliers) {
		if(small_summands.empty()) {
			if(!n) {
				p_multipliers->resize(r_summands.size(), size_t(0));
				return true; // zero can be written as a zero sum, always
			}
			return false;
		}
	} else {
		if(small_summands.size() <= 1) {
			_ASSERTE(small_summands.empty() || (n % small_summands.front()) != 0);
			return !n; // zero can be written as a zero sum, always
		}
		// already checked the simple products above; either only a single
		// number which does not divide n or no all numbers larger than n
	}

	std::reverse(max_multiples.begin(), max_multiples.end());
	std::reverse(small_summands.begin(), small_summands.end());
	// reverse to use the large ones first

	// generate all combinations of multipliers, there is product_i^m max_multiples_i
	// of those, with m being the number of possible summands. this is a large number.
	// this is clearly not a way to solve but the smallest instances of thie problem.

	const size_t m = max_multiples.size() - 1;
	std::vector<size_t> multipliers(m, size_t(0));
	// multiplier combinations without the first constituent, will resolve that one using division

	for(size_t n_running_sum = 0;;) { // incrementally build the number
#ifdef _DEBUG
		size_t n_debug_sum = 0;
		for(size_t i = 0; i < m; ++ i)
			n_debug_sum += multipliers[i] * small_summands[i + 1];
		_ASSERTE(n_running_sum == n_debug_sum);
#endif // _DEBUG
		// take a product of the combinations by summands (could form it incrementally as well)

		if(n_running_sum <= n && !((n - n_running_sum) % small_summands.front())) {
			if(p_multipliers) {
				p_multipliers->resize(r_summands.size(), size_t(0));
				size_t n_largest_num = (n - n_running_sum) / small_summands.front();
				_ASSERTE(r_summands.count(small_summands.front()));
				(*p_multipliers)[std::distance(r_summands.begin(),
					r_summands.find(small_summands.front()))] = n_largest_num;
				for(size_t i = 0; i < m; ++ i) {
					_ASSERTE(r_summands.count(small_summands[i + 1]));
					(*p_multipliers)[std::distance(r_summands.begin(),
						r_summands.find(small_summands[i + 1]))] = multipliers[i];
				}
			}
			// return the multipliers if the caller wantes them

			return true;
		}
		// see if we can add an integer multiple of the largest constituent to get n

		if(!m) // otherwise gets stuck in a loop
			return false;
		else {
			for(size_t i = 0; /*i < m*/; ++ i) {
				if(++ multipliers[i] == max_multiples[i + 1]) {
					n_running_sum -= small_summands[i + 1] * (max_multiples[i + 1] - 1); // remove it from the running sum
					multipliers[i] = 0;
					if(i + 1 == m)
						return false; // exhausted all the combinations
				} else {
					n_running_sum += small_summands[i + 1]; // also add it to the running sum
					break;
				}
			}
		}
	}
	// loop through the combinations

	return false;

	/*if(r_summands.size() == 1) {
		return !(n % *r_summands.begin());
		// only a single number, n can be partitioned using that only if it is divisible
	} else if(!r_summands.empty() && r_summands.count(1))
		return true; // in case there is a 1 among the parts, any number can be partitioned // in fact if any of the parts divides n then it can
	else {
		// we need to write n as a sum of p_i * x_i where p are the parts and x_i are the unknown multipliers
		// the series of numbers that p generates is given by the combinations of the parts + min(x_i) * sum of the parts
		// so it boils down to calculating the remainder after division by the sum of the parts

		// 2, 3: 2, 3, 4, 5, ...
		// 2, 5: 2, 4, 5, 6, 7, 8, 9, 10, 11, 12 ...
		// 2, 4: 2, 4, 6, 8, 10, ...
		// 2, 6: 2, 4, 6, 8, 10, 12, ... will never cover odd numbers
		// 2, 9: 2, 4, 6, 8, 9, 10, 11, 12, ...

	}*/
}

/**
 *	@brief subdivides a block in a block layout
 *
 *	@param[in] r_blocks is block layout specified as a cumulative sum of block
 *		sizes (the first entry equals 0 and the last one equals the dimension)
 *	@param[in] i is one-based index of the block to be subdivided (e.g. 1 for the
 *		first block, 2 for the second)
 *	@param[in] r_block_size_set is set of available block sizes
 *	@param[in] r_block_counts is a corresponding list of counts of each block size
 *
 *	@return Returns the one-based index of the last block that
 *		was created by subdivision of the original block.
 */
size_t n_Subdivide_BlockLayout_Parition(std::vector<size_t> &r_blocks, size_t i,
	const std::set<size_t> &r_block_size_set, std::vector<size_t> &r_block_counts) // throw(std::bad_alloc)
{
	_ASSERTE(i); // not zero

	std::vector<size_t>::iterator p_last_nnz_it = std::find_if(r_block_counts.rbegin(),
		r_block_counts.rend(), CIsNonzero()).base(); // base() returns iterator which points one past the value
	if(p_last_nnz_it == r_block_counts.begin())
		return i; // no subdivisions take place
#ifdef _DEBUG
	size_t n_removed_last;
	{
		std::set<size_t>::const_iterator p_set_it = r_block_size_set.begin();
		std::advance(p_set_it, p_last_nnz_it - r_block_counts.begin() - 1); // base() returns iterator which is one off
		n_removed_last = *p_set_it;
	}
#endif // _DEBUG
	-- *(p_last_nnz_it - 1); // base() returns iterator which is one off
	// decrement the last block size, to not duplicate the last block boundary in the loop below

	const size_t n_end = r_blocks[i];
	size_t n_base = r_blocks[i - 1];
	std::set<size_t>::const_iterator p_bs_it = r_block_size_set.begin();
	for(std::vector<size_t>::const_iterator p_part_it = r_block_counts.begin()/*,
	   p_end_it = r_block_counts.end()*/; p_part_it != p_last_nnz_it/*p_end_it*/; ++ p_part_it, ++ p_bs_it) {
		size_t n_multipler = *p_part_it, n_size = *p_bs_it;
		for(size_t j = 0; j < n_multipler; ++ j) {
			n_base += n_size;
			r_blocks.insert(r_blocks.begin() + i, n_base);
			++ i;
		}
	}
	_ASSERTE(r_blocks[i] == n_end); // make sure the original end is still there
	_ASSERTE(n_base == n_end - n_removed_last); // make sure the last inserted block boundary is where we expect it
	// insert new block boundaries

	++ *(p_last_nnz_it - 1);
	// restore!

	return i;
}

//#define MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
// note that this potentially changes the results because the left/right fusions will be taken even
// if they exceed the absolute fill-in (the relative incremental fill-in is likely much smaller).
// this could now be modified into transparent incremental fill-in calculation (just remember the
// block structure and the corresponding fill-ins, recalculate only the changed ones).
// but as this is a greedy approach anyway, it is of low importance, it will just select a few matrices less.

#include <time.h>

class CWatchdog : public CRunable_Thread_ShallowCopy {
protected:
	bool &m_r_b_flag;
	double m_f_thresh;

public:
	CWatchdog(bool &r_b_flag, double f_threshold)
		:m_r_b_flag(r_b_flag), m_f_thresh(f_threshold)
	{
		m_r_b_flag = false;
	}

	/**
	 *	@brief simple stop function; stops the associated thread forcibly
	 *	@return Returns true on success, false on failure.
	 */
	inline bool Stop()
	{
		return m_thread.Stop(true);
	}

protected:
	virtual void Run()
	{
		//printf("_"); fflush(stdout); // debug
#if defined(_WIN32) || defined(_WIN64)
		Sleep(DWORD(m_f_thresh * 1000));
#else // _WIN32 || _WIN64
		struct timespec t_tv;
		t_tv.tv_sec = int(m_f_thresh);
		t_tv.tv_nsec = long((m_f_thresh - int(m_f_thresh)) * 1000000000);
		nanosleep(&t_tv, NULL);
#endif // _WIN32 || _WIN64
		m_r_b_flag = true;
		printf("!"); fflush(stdout); // debug
	}
};

bool Modify_BlockLayout_Greedy(std::vector<size_t> &r_block_rows,
	std::vector<size_t> &r_block_cols, const TSparse &r_mat,
	const std::vector<size_t> &r_target_block_sizes, size_t n_tol = 5) // throw(std::bad_alloc)
{
	++ mbl_stats.n_call_num;

	// simple greedy function
	// * in case a given block is of size among block sizes, do nothing
	// * in case a given block is larger than the largest, attempt to subdivide it (if can't, fail)
	// * in case a given block is smaller than the smallest, attempt to fuse left or right,
	//   without exceeding the allowed fill-in (if can't, fail)

	CTimer t;

	printf("."); fflush(stdout); // debug

	size_t n_max_fillin = r_mat.p[r_mat.n] / n_tol;
#ifdef _DEBUG
	{
		size_t n_initial_fillin = n_SparseMatrix_Block_FillIn(r_block_rows, r_block_cols, r_mat); // debug
#ifdef MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
		_ASSERTE(!n_initial_fillin);
		// the partial fill-in evaluation assumes that there is no fill-in before we start modifying
		// the matrix (it won't crash but the scores will be biased and the solution will be suboptimal)
#else // MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
		_ASSERTE(n_initial_fillin < n_max_fillin);
#endif // MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
	}
#endif // _DEBUG

	const std::set<size_t> block_size_set(r_target_block_sizes.begin(), r_target_block_sizes.end());
	const size_t n_smallest_block = *block_size_set.begin();
	const size_t n_largest_block = *(-- block_size_set.end());

	if(!Partition(r_mat.m, block_size_set) || (r_mat.m != r_mat.n && !Partition(r_mat.n, block_size_set))) {
		++ mbl_stats.n_reject_geometry_num;
		return false; // reject some matrices early on
	}
	// right now, we do not allow the matrix to get bigger

	printf("."); fflush(stdout); // debug

	size_t n_correlation = 0;
	for(size_t i = 1, n = r_block_rows.size(); i < n; ++ i)
		n_correlation += (Partition(r_block_rows[i] - r_block_rows[i - 1], block_size_set))? 1 : 0; // use partition rather than block_size_set.count(); subdividing large blocks makes no fillin and is totally ok
	for(size_t i = 1, n = r_block_cols.size(); i < n; ++ i)
		n_correlation += (Partition(r_block_cols[i] - r_block_cols[i - 1], block_size_set))? 1 : 0; // use partition rather than block_size_set.count(); subdividing large blocks makes no fillin and is totally ok
	double f_correlation = double(n_correlation) / (r_block_rows.size() + r_block_cols.size());
	if(std::max(r_mat.m, r_mat.n) > 500000 && f_correlation < .01) { // saw no accepts below 4% on UFLSMC, this should be relatively safe (although it is possible to modify a matrix with 0% correlation if one doesn't mind some fill-in)
		fprintf(stderr, "warning: low correlation reject (%.2f%%)\n", f_correlation * 100);
		return false;
	}
	// only get matrices which already have some blocks of the size required

	const size_t n_max_search = 50;//SIZE_MAX - 1; // limit the number of blocks that can be merged left/right at a time (use SIZE_MAX - 1 for no limit)
	_ASSERTE(n_max_search < SIZE_MAX); // will add 1
	// this should probably depend on the largest block but our blocks are usually small enough

	TSparse t_transpose_mat;
	bool b_have_transpose_matrix;
	try {
		r_mat.Transpose(t_transpose_mat);
		b_have_transpose_matrix = true;
	} catch(std::bad_alloc&) {
		b_have_transpose_matrix = false;
	}
	// see if we have enough memory to get a transpose matrix (if not, we can still run slower)

	printf("."); fflush(stdout); // debug

	double f_budget_left = 45 * 60 - t.f_Time(); // 45 minutes should be plenty; make sure there is some progress
	if(f_budget_left < 0) {
		double f_time = t.f_Time();
		fprintf(stderr, "warning: rejected because of time-out (" PRItime ")\n", PRItimeparams(f_time));
		return false;
	}

	bool b_timeout = false;
	CWatchdog watchdog(b_timeout, f_budget_left);
	if(!watchdog.Start()) {
		fprintf(stderr, "warning: the watchdog failed to start\n");
		// don't fail though
	}

	for(int n_pass = 0; n_pass < 2 && !b_timeout; ++ n_pass) { // vertical and horizontal pass
		std::vector<size_t> &r_blocks = (n_pass)? r_block_rows : r_block_cols;
		// modify this in this pass

		TBlockColFillinCache fillin_cache;
		// can cache fill-in results

		size_t n_last_i = 0, n_nochg_num = 0;

		for(size_t i = 1, n = r_blocks.size(); i < n && !b_timeout; ++ i) {
#ifdef _DEBUG
			if(i == n_last_i) {
				if(++ n_nochg_num > 10)
					fprintf(stderr, "warning: stall (%d, %d)\n", int(i), int(n));
			} else
				n_nochg_num = 0;
			n_last_i = i;
			// detect infinite loops

			printf("\b%c", "/-\\|"[i % 4]); // progress
#endif // _DEBUG
			size_t n_block_size = r_blocks[i] - r_blocks[i - 1];
			if(block_size_set.count(n_block_size)) {
				continue;
				// a given block is of size among block sizes, do nothing
			} else if(n_block_size > n_largest_block) {
				// in case a given block is larger than the largest, attempt to subdivide it

				std::vector<size_t> partition;
				if(Partition(n_block_size, block_size_set, &partition)) {
					i = n_Subdivide_BlockLayout_Parition(r_blocks, i, block_size_set, partition);
					n = r_blocks.size(); // !!
					// subdivide the layout

					/*if(n_SparseMatrix_Block_FillIn(r_block_rows, r_block_cols, r_mat, n_max_fillin) > n_max_fillin) {
						++ mbl_stats.n_reject_fillin_num;
						return false;
					}*/ // try not to check the fill-in very often, it gets expensive
					// see if the fill-in is still acceptable (could do that at the end though)

					continue;
				}
				// if can't partition this column, try to fuse left or right
			}

			// in case a given block is smaller than the smallest, attempt to fuse left or right
			// or if we fell from the branch above, the given block is larger but not a multiple
			// of the available block sizes

			_ASSERTE(n == r_blocks.size());

			bool b_can_go_left = false, b_can_go_right = false;
			size_t n_fillin_left_fusion = 0, n_fillin_right_fusion = 0,
				n_fillin_reject = 0, n_fuse_left = 0, n_fuse_right;
#ifdef _DEBUG
			std::vector<size_t> left_mod, right_mod;
			static size_t n_debug_step = 0;
			++ n_debug_step;
#endif // _DEBUG
			std::vector<size_t> partition_right, partition_left, erased_elem;
			for(size_t n_fuse = 1, n_search = min(n_max_search + 1, i); n_fuse < n_search; ++ n_fuse) { // how many blocks to fuse
				size_t n_prev_size = r_blocks[i - 1] - r_blocks[i - n_fuse - 1];
				if(Partition(n_prev_size + n_block_size, block_size_set, &partition_left)) { // only if a partition exists, otherwise no point in trying
#ifdef _DEBUG
					std::vector<size_t> dbg_copy = r_blocks;
#endif // _DEBUG
					erased_elem.assign(r_blocks.begin() + (i - n_fuse), r_blocks.begin() + i);
					//size_t n_erased = r_blocks[i - 1];
					r_blocks.erase(r_blocks.begin() + (i - n_fuse), r_blocks.begin() + i); // erase the left boundary to fuse left

					size_t n_next_i_left = n_Subdivide_BlockLayout_Parition(r_blocks,
						i - n_fuse, block_size_set, partition_left);

					_ASSERTE(i - n_fuse > 0);
#ifdef MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
					if(&r_block_cols == &r_blocks) {
						++ mbl_stats.n_fillin_parteval_num;
						n_fillin_left_fusion = n_SparseMatrix_Block_FillIn(r_block_rows,
							r_block_cols, r_mat, n_max_fillin, i - n_fuse - 1, n_next_i_left);
					} else if(b_have_transpose_matrix) {
							++ mbl_stats.n_fillin_parteval_num;
							n_fillin_left_fusion = n_SparseMatrix_Block_FillIn(r_block_cols,
								r_block_rows, t_transpose_mat, n_max_fillin, i - n_fuse - 1, n_next_i_left);
					} else
#endif // MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
					{
						++ mbl_stats.n_fillin_eval_num;
						if(&r_block_rows == &r_blocks && b_have_transpose_matrix) {
							n_fillin_left_fusion = n_SparseMatrix_Block_FillIn_Incremental(r_block_cols,
								r_block_rows, t_transpose_mat, n_max_fillin, &fillin_cache);
						} else {
							n_fillin_left_fusion = n_SparseMatrix_Block_FillIn_Incremental(r_block_rows,
								r_block_cols, r_mat, n_max_fillin, (&r_block_cols == &r_blocks)? &fillin_cache : 0); // can't, would need a transpose matrix for that
						}
					}
					if(n_fillin_left_fusion > n_max_fillin)
						++ n_fillin_reject;
					else {
						b_can_go_left = true;
#ifdef _DEBUG
						left_mod = r_blocks;
#endif // _DEBUG
					}

					r_blocks.erase(r_blocks.begin() + (i - n_fuse), r_blocks.begin() + n_next_i_left); // erase the blocks we just subdivided
					r_blocks.insert(r_blocks.begin() + (i - n_fuse), erased_elem.begin(), erased_elem.end());

					_ASSERTE(n == r_blocks.size());
					_ASSERTE(dbg_copy == r_blocks);

					n_fuse_left = n_fuse;
					break;
				}
			}
			for(size_t n_fuse = 1, n_search = min(n_max_search + 1, n - i); n_fuse < n_search; ++ n_fuse) { // how many blocks to fuse
				size_t n_next_size = r_blocks[i + n_fuse] - r_blocks[i];
				if(Partition(n_block_size + n_next_size, block_size_set, &partition_right)) { // only if a partition exists, otherwise no point in trying
#ifdef _DEBUG
					std::vector<size_t> dbg_copy = r_blocks;
#endif // _DEBUG
					//size_t n_erased = r_blocks[i];
					erased_elem.assign(r_blocks.begin() + i, r_blocks.begin() + (i + n_fuse));
					r_blocks.erase(r_blocks.begin() + i, r_blocks.begin() + (i + n_fuse)); // erase the right boundary to fuse right (not i+1!)

					size_t n_next_i_right = n_Subdivide_BlockLayout_Parition(r_blocks,
						i, block_size_set, partition_right);

					if(b_can_go_left) {
#ifdef MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
						if(&r_block_cols == &r_blocks) {
							++ mbl_stats.n_fillin_parteval_num;
							n_fillin_right_fusion = n_SparseMatrix_Block_FillIn(r_block_rows,
								r_block_cols, r_mat, n_max_fillin, i - (/*std::max(size_t(1),*/ n_fuse_left + 1/*)*/), n_next_i_right); // try not to calculate the fill-in very often
						} else if(b_have_transpose_matrix) {
								++ mbl_stats.n_fillin_parteval_num;
								n_fillin_left_fusion = n_SparseMatrix_Block_FillIn(r_block_cols,
									r_block_rows, t_transpose_mat, n_max_fillin, i - (/*std::max(size_t(1),*/ n_fuse_left + 1/*)*/), n_next_i_right);
						} else
						// evaluate the fill-in in columns starting with i - n_fuse_left - 1
						// so that the left fusion is not penalized by fill-in incoming from the left
						// (there is no fill-in on the right)
						// note that std::max(size_t(1), n_fuse_left + 1) = n_fuse_left + 1 since n_fuse_left >= 0
#endif // MBL_PARTIAL_FILLIN_EVAL_OPTIMIZATION
						{
							++ mbl_stats.n_fillin_eval_num;
							if(&r_block_rows == &r_blocks && b_have_transpose_matrix) {
								n_fillin_right_fusion = n_SparseMatrix_Block_FillIn_Incremental(r_block_cols,
									r_block_rows, t_transpose_mat, n_max_fillin, &fillin_cache);
							} else {
								n_fillin_right_fusion = n_SparseMatrix_Block_FillIn_Incremental(r_block_rows,
									r_block_cols, r_mat, n_max_fillin, (&r_block_cols == &r_blocks)? &fillin_cache : 0); // can't, would need a transpose matrix for that
							}
						}
					} else
						n_fillin_right_fusion = 0; // don't evaluate, nothing to compare with
					if(n_fillin_right_fusion > n_max_fillin)
						++ n_fillin_reject;
					else {
						b_can_go_right = true;
#ifdef _DEBUG
						right_mod = r_blocks;
#endif // _DEBUG
					}

					r_blocks.erase(r_blocks.begin() + i, r_blocks.begin() + n_next_i_right); // erase the blocks we just subdivided
					r_blocks.insert(r_blocks.begin() + i, erased_elem.begin(), erased_elem.end());

					_ASSERTE(n == r_blocks.size());
					_ASSERTE(dbg_copy == r_blocks);

					n_fuse_right = n_fuse;
					break;
				}
			}
			// try fusing left and right and see what happens

			if(!b_can_go_left && !b_can_go_right) {
				if(n_fillin_reject > 0)
					++ mbl_stats.n_reject_fillin_num;
				if(n_fillin_reject != 2)
					++ mbl_stats.n_reject_geometry_num;
				return false;
			}
			// can't fuse

			size_t n_blocks_left = std::accumulate(partition_left.begin(), partition_left.end(), size_t(0));
			size_t n_blocks_right = std::accumulate(partition_right.begin(), partition_right.end(), size_t(0));

			if(b_can_go_left && (!b_can_go_right || std::make_pair(n_fillin_left_fusion, n_blocks_left) < std::make_pair(n_fillin_right_fusion, n_blocks_right))) {
				//r_blocks.erase(r_blocks.begin() + (i - 1)); // erase the left boundary to fuse left
				r_blocks.erase(r_blocks.begin() + (i - n_fuse_left), r_blocks.begin() + i);
				size_t n_next_i_left = n_Subdivide_BlockLayout_Parition(r_blocks, i - n_fuse_left, block_size_set, partition_left);
				_ASSERTE(left_mod == r_blocks);
				i = n_next_i_left - 1; // -1 is just to counteract the ++ in the for loop
				n = r_blocks.size(); // !!
			} else {
				//r_blocks.erase(r_blocks.begin() + i); // erase the right boundary to fuse right
				r_blocks.erase(r_blocks.begin() + i, r_blocks.begin() + (i + n_fuse_right));
				size_t n_next_i_right = n_Subdivide_BlockLayout_Parition(r_blocks, i, block_size_set, partition_right);
				_ASSERTE(right_mod == r_blocks);
				i = n_next_i_right - 1; // -1 is just to counteract the ++ in the for loop
				n = r_blocks.size(); // !!
			}
			// fuse

			_ASSERTE(n == r_blocks.size());
			// make sure we got that right
		}

		for(size_t i = 1, n = r_blocks.size(); i < n; ++ i)
			_ASSERTE(b_timeout || block_size_set.count(r_blocks[i] - r_blocks[i - 1]));
		// make sure that all the blocks now have good size
	}

	watchdog.Stop(); // may fail if the watchdog is not running anymore

	if(b_timeout) {
		double f_time = t.f_Time();
		fprintf(stderr, "warning: rejected because of time-out (" PRItime ")\n", PRItimeparams(f_time));
		return false;
	}

	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == r_mat.m);
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end())); // must be also strictly increasing
	// make sure this doesn't damage the layouts

	++ mbl_stats.n_fillin_eval_num;
	size_t n_final_fillin = n_SparseMatrix_Block_FillIn(r_block_rows, r_block_cols, r_mat, n_max_fillin);
	if(n_final_fillin > n_max_fillin) {
		++ mbl_stats.n_reject_fillin_num;
		return false;
	}

	if(f_correlation < .05) // reject below 5%
		fprintf(stderr, "warning: notable low correlation accept (%.2f%%)\n", f_correlation * 100);
	// see if there are any

	printf("debug: final fill-in is " PRIsize " elements (%.2f%%)\n", n_final_fillin,
		(100.0f * n_final_fillin) / r_mat.p[r_mat.n]);
	//_ASSERTE(n_final_fillin <= n_max_fillin); // not anymore, now we prefer to check it at the end and not calculate it inside the loop at all, unless we need to choose between left/right fusion
	// if we got this far, then the fill-in must be ok

	mbl_stats.n_total_accepted_fillin += n_final_fillin;
	mbl_stats.n_total_accepted_nnz += r_mat.p[r_mat.n];
	++ mbl_stats.n_accept_num;

	return true;

	// * a final optional pass over the blocks, try to fuse pairs of blocks into larger ones without exceeding the fill-in
}

/*void Modify_BlockLayout(std::vector<size_t> &r_block_rows,
	std::vector<size_t> &r_block_cols, const TSparse &r_mat,
	const std::vector<size_t> &r_target_block_sizes, size_t n_tol = 5)
{
	// this modifies the block layout to only have blocks of the given size(s)

	// since the matrix is not empty, we can do mb-1 + nb-1 decisions whether to keep or kill the respective block boundary
	// each decision has an associated fill-in cost. the upper bound of the fill-in can be calculated without knowing
	// the target block sizes, the real value may depend on how the specific blocks are subdivided to target block sizes
	// (e.g. a block of 12 elements can be split to 6+6 or 3+3+3+3 or 8+4 or 4+8 and the fill in depends on that)

	// this looks like it will be quite slow to calculate the fill-ins

	// one could first try to only enumerate the splittings and then calculate the fill-in
	// after the fact, that seems quite easy to do and would not require too much memory
	// (only std::set for one block column, would go one block column at a time)

	// on the other hand all the fill-ins can be below the threshold and so it can be decided
	// that the specific subdivision it does not matter

	if(!r_mat.m || !r_mat.n || r_target_block_sizes.empty())
		return;
	// nothing to do on empty matrices

	_ASSERTE(r_block_rows.size() >= 2 && r_block_cols.size() >= 2);
	_ASSERTE(!r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == r_mat.m);
	_ASSERTE(!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == r_mat.n);
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_rows.begin(), r_block_rows.end()));
	_ASSERTE(stl_ut::b_IsStrictlySorted(r_block_cols.begin(), r_block_cols.end()));
	// sanity check

	size_t n_row_choice_num = r_block_rows.size() - 2;
	size_t ncol_choice_num = r_block_cols.size() - 2;
	size_t n_choice_num = n_row_choice_num + n_col_choice_num;
	if(!n_choice_num)
		return;
	// no choice

	// lets define the cost function domain as cxc, where each row corresponds to one choice
	// the cost of dropping one boundary i, c_i = absdiff(column(i-1)^column(i))
	// this however changes the the cost of dropping two adjacent boundaries is different (not a sum)
	// so keepers reset the cost function and the runs of kills form the cost somehow

	// in 1D, this forms an upper triangular matrix, where the first cost on row i corresponds
	// to keeping the boundary i and each next element j corresponds to killing the next j-1 boundaries

	// if the rows and columns are not symmetric then the cost is 2D, depending on how many cols and rows are killed at each step

	// nope, not even that. setting the first n boundaries only fixes the upper-left nxn blocks of the matrix,
	// however the fill-in in the next block boundaries still depends on the state of the first n boundaries
	// in the "inverted L" area of the matrix.

	// this does not seem to be solvable by DP in 2D
}*/

struct TBlockInfo {
	size_t n_block_elem_nnz;
	std::set<std::pair<size_t, size_t> > block_set; // set of (block_row, block_col) pairs
	std::map<std::pair<size_t, size_t>, size_t> block_size_freqs; // map of (block_height, block_width) -> count
	const TSparse &r_mat;
	const std::vector<size_t> &r_block_rows;
	const std::vector<size_t> &r_block_cols;

	bool b_CanUse_BSR() const
	{
		return block_size_freqs.size() == 1 &&
			(*block_size_freqs.begin()).first.first == (*block_size_freqs.begin()).first.second;
	}

	uint64_t n_COO_Storage(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
	{
		TSparse::TMMHeader t_header;
		t_header.b_binary = (n_scalar_size == 0);
		t_header.b_symmetric = false;
		t_header.n_rows = r_mat.m;
		t_header.n_cols = r_mat.n;
		t_header.n_full_nnz = r_mat.p[r_mat.n];
		t_header.n_nnz = r_mat.p[r_mat.n];

		return t_header.n_AllocationSize_COO(n_idx_size, (n_scalar_size)? n_scalar_size : sizeof(double));
	}

	uint64_t n_BSR_Storage(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
	{
		if(!b_CanUse_BSR())
			return -1;

		const size_t bnnz = block_set.size();

		TSparse::TMMHeader t_header;
		t_header.b_binary = (n_scalar_size == 0);
		t_header.b_symmetric = false;
		t_header.n_rows = r_block_rows.size();
		t_header.n_cols = r_block_cols.size();
		t_header.n_full_nnz = bnnz;
		t_header.n_nnz = bnnz;

		size_t n_block_size = (*block_size_freqs.begin()).first.first;

		return t_header.n_AllocationSize_CSC(n_idx_size,
			((n_scalar_size)? n_scalar_size : sizeof(double)) * n_block_size * n_block_size);
		// can easily reuse that; just set the scalar size to the size of the block
	}

	uint64_t n_VBR_Storage(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
	{
		const size_t bnnz = block_set.size();

		uint64_t n_size_cols = min((min(uint64_t(r_block_rows.size()),
			UINT64_MAX - 1) + 1), UINT64_MAX / n_idx_size) * n_idx_size; // cumsum of numbers of blocks in each block row in VBR
		uint64_t n_size_rows = min(uint64_t(bnnz), UINT64_MAX / n_idx_size) * n_idx_size; // column index in VBR
		uint64_t n_size_offs = min(n_size_rows, UINT64_MAX - n_idx_size) + n_idx_size; // block data offset in VBR
		uint64_t n_size_data = min(uint64_t(n_block_elem_nnz), UINT64_MAX / n_scalar_size) * n_scalar_size; // all the nnz
		uint64_t n_size_rpntr = n_size_cols; // cumsum of block origins in each block row in VBR
		uint64_t n_size_cpntr = min((min(uint64_t(r_block_cols.size()),
			UINT64_MAX - 1) + 1), UINT64_MAX / n_idx_size) * n_idx_size; // cumsum of block origins in each block column in VBR

		uint64_t n_size = n_size_cols;
		n_size = min(n_size, UINT64_MAX - n_size_rows) + n_size_rows;
		n_size = min(n_size, UINT64_MAX - n_size_offs) + n_size_offs;
		n_size = min(n_size, UINT64_MAX - n_size_data) + n_size_data;
		n_size = min(n_size, UINT64_MAX - n_size_rpntr) + n_size_rpntr;
		n_size = min(n_size, UINT64_MAX - n_size_cpntr) + n_size_cpntr;
		// sum it all up

		return n_size;
		// can easily reuse that; just set the scalar size to the size of the block
	}

	void Get_BlockSize_Histogram(std::map<size_t, size_t> &r_hist) const // throw(std::bad_alloc)
	{
		r_hist.clear();
		for(std::map<std::pair<size_t, size_t>, size_t>::const_iterator
		   p_bs_it = block_size_freqs.begin(), p_end_it = block_size_freqs.end();
		   p_bs_it != p_end_it; ++ p_bs_it) {
			std::pair<size_t, size_t> t_size = (*p_bs_it).first;
			size_t n_frequency = (*p_bs_it).second;
			r_hist[t_size.first] += n_frequency;
			r_hist[t_size.second] += n_frequency;
		}
	}

	size_t n_NNZ_BlockSize(size_t n_min_block_dim, size_t n_min_block_area) const
	{
		size_t n_nnz = 0;
		for(std::map<std::pair<size_t, size_t>, size_t>::const_iterator
		   p_bs_it = block_size_freqs.begin(), p_end_it = block_size_freqs.end();
		   p_bs_it != p_end_it; ++ p_bs_it) {
			std::pair<size_t, size_t> t_size = (*p_bs_it).first;
			size_t n_frequency = (*p_bs_it).second;
			if(t_size.first >= n_min_block_dim && t_size.second >= n_min_block_dim &&
			   t_size.first * t_size.second >= n_min_block_area)
				n_nnz += t_size.first * t_size.second * n_frequency;
		}
		return n_nnz;
	}

	TBlockInfo(const TSparse &_r_mat, const std::vector<size_t> &_r_block_rows,
		const std::vector<size_t> &_r_block_cols) // throw(std::bad_alloc)
		:n_block_elem_nnz(0), r_mat(_r_mat),
		r_block_rows(_r_block_rows), r_block_cols(_r_block_cols)
	{
		for(size_t i = 0, n = r_mat.n; i < n; ++ i) {
			size_t n_block_col = (r_block_cols.size() < 2)? 0 :
				(std::upper_bound(r_block_cols.begin(), r_block_cols.end(), i) - r_block_cols.begin()) - 1;
			_ASSERTE(r_block_cols.size() < 2 || (n_block_col < r_block_cols.size() && i >= r_block_cols[n_block_col] &&
				n_block_col + 1 < r_block_cols.size() &&  i < r_block_cols[n_block_col + 1]));
			// find block column of this nnz element

			size_t n_width = r_block_cols[n_block_col + 1] - r_block_cols[n_block_col];

			for(size_t p1 = r_mat.p[i], p2 = r_mat.p[i + 1]; p1 != p2; ++ p1) {
				size_t j = r_mat.i[p1];

				size_t n_block_row = (r_block_rows.size() < 2)? 0 :
					(std::upper_bound(r_block_rows.begin(), r_block_rows.end(), j) - r_block_rows.begin()) - 1;
				_ASSERTE(r_block_rows.size() < 2 || (n_block_row < r_block_rows.size() && j >= r_block_rows[n_block_row] &&
					n_block_row + 1 < r_block_rows.size() &&  j < r_block_rows[n_block_row + 1]));
				// find block row of this nnz element

				bool b_new_block = block_set.insert(std::make_pair(n_block_row, n_block_col)).second;
				// add it to the set

				size_t n_height = r_block_rows[n_block_row + 1] - r_block_rows[n_block_row];

				if(b_new_block) {
					++ block_size_freqs[std::make_pair(n_height, n_width)];
					// also get a set of sizes

					n_block_elem_nnz += n_height * n_width;
					// and sum up unique block area
				}
				// in case this is the first elem in this block
			}
		}
		// get a set of nonzero blocks
	}

	bool Write_BlockLayout(const char *p_s_output_filename) const
	{
		size_t bnnz = block_set.size();

		CUniqueFILE_Ptr p_fw;
		if(p_fw.Open(p_s_output_filename, "w")) {
			fprintf(p_fw, PRIsize " x " PRIsize " (" PRIsize ")\n",
				r_mat.m, r_mat.n, r_mat.p[r_mat.n]);
			fprintf(p_fw, PRIsize " x " PRIsize " (" PRIsize ")\n",
				r_block_rows.size() - 1, r_block_cols.size() - 1, bnnz);
			// write scalar size, block size and the number of nonzeros

			for(size_t i = 0, n = r_block_rows.size(); i < n; ++ i)
				fprintf(p_fw, " " PRIsize + ((i)? 0 : 1), r_block_rows[i]);
			fprintf(p_fw, "\n");
			for(size_t i = 0, n = r_block_cols.size(); i < n; ++ i)
				fprintf(p_fw, " " PRIsize + ((i)? 0 : 1), r_block_cols[i]);
			fprintf(p_fw, "\n");
			// write block row and column starting offsets

			fprintf(p_fw, "\n");

			const uint64_t n_vbr_size = n_VBR_Storage(), n_coo_size = n_COO_Storage();
			if(b_CanUse_BSR()) {
				const uint64_t n_bsr_size = n_BSR_Storage();
				fprintf(p_fw, "%% " PRIsizeB " as BSR (%.3f:1), " PRIsizeB " as VBR (%.3f:1)\n",
					PRIsizeBparams(n_bsr_size), n_coo_size / double(n_bsr_size),
					PRIsizeBparams(n_vbr_size), n_coo_size / double(n_vbr_size));
			} else {
				fprintf(p_fw, "%% " PRIsizeB " as VBR (%.3f:1)\n", PRIsizeBparams(n_vbr_size),
					n_coo_size / double(n_vbr_size));
			}
			fprintf(p_fw, "%% " PRIsize " block sizes\n", block_size_freqs.size());
			for(std::map<std::pair<size_t, size_t>, size_t>::const_iterator
			   p_bs_it = block_size_freqs.begin(), p_end_it = block_size_freqs.end();
			   p_bs_it != p_end_it; ++ p_bs_it) {
				std::pair<size_t, size_t> t_size = (*p_bs_it).first;
				size_t n_frequency = (*p_bs_it).second;
				fprintf(p_fw, "%% " PRIsize " x " PRIsize " (" PRIsize ")\n",
					t_size.first, t_size.second, n_frequency);
			}
			// write block size information in a comment (only for the benefit
			// of a human, can be obtained from the data at any time)

			_ASSERTE(r_mat.p[r_mat.n] <= n_block_elem_nnz); // if we used tolerance, there will be some losses
			fprintf(p_fw, "%% block fill-in %.2f%% (from %.2f%% to %.2f%%)\n",
				100 * double(n_block_elem_nnz - r_mat.p[r_mat.n]) / r_mat.p[r_mat.n],
				100 * (double(r_mat.p[r_mat.n]) / r_mat.m) / r_mat.n, // orig fill in
				100 * (double(n_block_elem_nnz) / r_mat.m) / r_mat.n); // new fill in
			// show also the block fill-in

			fprintf(p_fw, "%% %.2f%% of nnz in blocks with at least 3 nnz each\n",
				100 * double(n_NNZ_BlockSize(1, 3)) / n_block_elem_nnz);

			return true;
		}
		return false;
	}

	static bool Write_BlockLayout(const char *p_s_output_filename,
		const TSparse &r_mat, const std::vector<size_t> &r_block_rows,
		const std::vector<size_t> &r_block_cols, size_t n_block_nnz_num = 0)
	{
		size_t bnnz = n_block_nnz_num;
		if(!n_block_nnz_num)
			bnnz = n_SparseMatrix_Block_Num(r_block_rows, r_block_cols, r_mat);
		// ...

		CUniqueFILE_Ptr p_fw;
		if(p_fw.Open(p_s_output_filename, "w")) {
			fprintf(p_fw, PRIsize " x " PRIsize " (" PRIsize ")\n",
				r_mat.m, r_mat.n, r_mat.p[r_mat.n]);
			fprintf(p_fw, PRIsize " x " PRIsize " (" PRIsize ")\n",
				r_block_rows.size() - 1, r_block_cols.size() - 1, bnnz);
			// write scalar size, block size and the number of nonzeros

			for(size_t i = 0, n = r_block_rows.size(); i < n; ++ i)
				fprintf(p_fw, " " PRIsize + ((i)? 0 : 1), r_block_rows[i]);
			fprintf(p_fw, "\n");
			for(size_t i = 0, n = r_block_cols.size(); i < n; ++ i)
				fprintf(p_fw, " " PRIsize + ((i)? 0 : 1), r_block_cols[i]);
			fprintf(p_fw, "\n");
			// write block row and column starting offsets

			fclose(p_fw);
			return true;
		}
		return false;
	}

	static bool Load_BlockLayout(const char *p_s_filename,
		std::vector<size_t> &r_block_rows, std::vector<size_t> &r_block_cols,
		size_t *p_elem_nnz = 0, size_t *p_block_nnz = 0)
	{
		CUniqueFILE_Ptr p_fr;
		if(!p_fr.Open(p_s_filename, "r"))
			return false;

		size_t n_rows, n_cols, n_nnz, n_brows, n_bcols, n_bnnz;
		if(fscanf(p_fr, PRIsize " x " PRIsize " (" PRIsize ")\n"
		   PRIsize " x " PRIsize " (" PRIsize ")\n", &n_rows, &n_cols,
		   &n_nnz, &n_brows, &n_bcols, &n_bnnz) != 6)
			return false;
		// read matrix size

		if(!stl_ut::Resize_To_N(r_block_cols, n_bcols + 1) ||
		   !stl_ut::Resize_To_N(r_block_rows, n_brows + 1))
			return false;
		// alloc the outputs

		for(size_t i = 0; i <= n_brows; ++ i) {
			if(fscanf(p_fr, " " PRIsize, &r_block_rows[i]) != 1)
				return false;
		}
		for(size_t i = 0; i <= n_bcols; ++ i) {
			if(fscanf(p_fr, " " PRIsize, &r_block_cols[i]) != 1)
				return false;
		}

		if(p_elem_nnz)
			*p_elem_nnz = n_nnz;
		if(p_block_nnz)
			*p_block_nnz = n_bnnz;
		// copy to output

		return !r_block_rows.empty() && r_block_rows.front() == 0 && r_block_rows.back() == n_rows &&
			!r_block_cols.empty() && r_block_cols.front() == 0 && r_block_cols.back() == n_cols && n_bnnz <= n_nnz;
		// sanity check
	}
};
