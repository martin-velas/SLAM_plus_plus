/*
								+---------------------------------+
								|                                 |
								|  ***   Über Block Matrix   ***  |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2013 |
								|                                 |
								|         BlockMatrix.inl         |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_INLINES_INCLUDED
#define __UBER_BLOCK_MATRIX_INLINES_INCLUDED

/**
 *	@file include/slam/BlockMatrix.inl
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief the überblockmatrix inline and template function definitions
 */

#include "slam/BlockMatrix.h"

inline size_t CUberBlockMatrix::n_Find_BlockRow(size_t n_row, size_t &r_n_block_row_num) const
{
	size_t n_row_index;
	if(n_row >= m_n_row_num)
		return size_t(-1);
	else {
		_TyRowConstIter p_row_it = std::upper_bound(m_block_rows_list.begin(),
			m_block_rows_list.end(), n_row, CFindLowerRow()); // t_odo
		//_ASSERTE(p_row_it != m_block_rows_list.end());
		if(p_row_it == m_block_rows_list.begin())
			return -1;
		-- p_row_it;
		const TRow &r_t_row = *p_row_it;

		_ASSERTE(n_row >= r_t_row.n_cumulative_height_sum &&
			n_row < r_t_row.n_cumulative_height_sum + r_t_row.n_height);
		// since n_row is nonnegative, it is inside of the matrix (the condition above),
		// and block rows span the entire matrix area, it can't be not found

		r_n_block_row_num = r_t_row.n_height;
		n_row_index = p_row_it - m_block_rows_list.begin();
		// just find the index, the row in question does exist, and has the right size (ideal case)

		_ASSERTE(n_row_index < m_block_rows_list.size());
		_ASSERTE(m_block_rows_list[n_row_index].n_height == r_n_block_row_num);
		_ASSERTE(n_row >= m_block_rows_list[n_row_index].n_cumulative_height_sum &&
			n_row < m_block_rows_list[n_row_index].n_cumulative_height_sum +
			m_block_rows_list[n_row_index].n_height);
		// make sure that the column reference is really resolved

		return n_row_index;
	}
	// resolve row reference
}

inline size_t CUberBlockMatrix::n_Find_BlockColumn(size_t n_column, size_t &r_n_block_column_num) const
{
	size_t n_column_index;
	if(n_column >= m_n_col_num)
		return -1;
	else {
		_ASSERTE(!m_block_cols_list.empty());
		_TyColumnConstIter p_col_it = std::upper_bound(m_block_cols_list.begin(),
			m_block_cols_list.end(), n_column, CFindLowerColumn()); // t_odo
		//_ASSERTE(p_col_it != m_block_cols_list.end()); // t_odo - handle this as proper runtime error // done (the line below)
		if(p_col_it == m_block_cols_list.begin())
			return -1;
		-- p_col_it;
		const TColumn &r_t_col = *p_col_it;

		_ASSERTE(n_column >= r_t_col.n_cumulative_width_sum &&
			n_column < r_t_col.n_cumulative_width_sum + r_t_col.n_width);
		// since n_column is nonnegative, it is inside of the matrix (the condition above),
		// and block columns span the entire matrix area, it can't be not found

		r_n_block_column_num = r_t_col.n_width;
		n_column_index = p_col_it - m_block_cols_list.begin();
		// just find the index, the column in question does exist, and has the right size (ideal case)

		_ASSERTE(n_column_index < m_block_cols_list.size());
		_ASSERTE(m_block_cols_list[n_column_index].n_width == r_n_block_column_num);
		_ASSERTE(n_column >= m_block_cols_list[n_column_index].n_cumulative_width_sum &&
			n_column < m_block_cols_list[n_column_index].n_cumulative_width_sum +
			m_block_cols_list[n_column_index].n_width);
		// make sure that the column reference is really resolved

		return n_column_index;
	}
	// resolve column reference (essentially the same code as for the row reference)
}

inline CUberBlockMatrix::_TyConstMatrixXdRef CUberBlockMatrix::t_Block_AtColumn(size_t n_column_index, size_t n_block_index) const
{
	_ASSERTE(n_column_index < m_block_cols_list.size()); // make sure n_column_index points to a valid column
	_ASSERTE(n_block_index < m_block_cols_list[n_column_index].block_list.size()); // make sure n_block_index selects a block in this column
	const TColumn::TBlockEntry &r_block = m_block_cols_list[n_column_index].block_list[n_block_index];
	size_t n_block_row_num = m_block_rows_list[r_block.first].n_height,
		n_block_column_num = m_block_cols_list[n_column_index].n_width;
	return _TyConstMatrixXdRef(r_block.second, n_block_row_num, n_block_column_num);
}

inline CUberBlockMatrix::_TyMatrixXdRef CUberBlockMatrix::t_Block_AtColumn(size_t n_column_index, size_t n_block_index)
{
	_ASSERTE(n_column_index < m_block_cols_list.size()); // make sure n_column_index points to a valid column
	_ASSERTE(n_block_index < m_block_cols_list[n_column_index].block_list.size()); // make sure n_block_index selects a block in this column
	TColumn::TBlockEntry &r_block = m_block_cols_list[n_column_index].block_list[n_block_index];
	size_t n_block_row_num = m_block_rows_list[r_block.first].n_height,
		n_block_column_num = m_block_cols_list[n_column_index].n_width;
	return _TyMatrixXdRef(r_block.second, n_block_row_num, n_block_column_num);
}

template <const int n_block_row_num, const int n_block_column_num>
inline typename CUberBlockMatrix::CMakeMatrixRef<n_block_row_num, n_block_column_num>::_TyConst
	CUberBlockMatrix::t_Block_AtColumn(size_t n_column_index, size_t n_block_index) const
{
	_ASSERTE(n_column_index < m_block_cols_list.size()); // make sure n_column_index points to a valid column
	_ASSERTE(n_block_index < m_block_cols_list[n_column_index].block_list.size()); // make sure n_block_index selects a block in this column
	const TColumn::TBlockEntry &r_block = m_block_cols_list[n_column_index].block_list[n_block_index];
	_ASSERTE(n_block_row_num == m_block_rows_list[r_block.first].n_height &&
		n_block_column_num == m_block_cols_list[n_column_index].n_width);
	return typename CMakeMatrixRef<n_block_row_num, n_block_column_num>::_TyConst(r_block.second);
}

template <const int n_block_row_num, const int n_block_column_num>
inline typename CUberBlockMatrix::CMakeMatrixRef<n_block_row_num, n_block_column_num>::_Ty
	CUberBlockMatrix::t_Block_AtColumn(size_t n_column_index, size_t n_block_index)
{
	_ASSERTE(n_column_index < m_block_cols_list.size()); // make sure n_column_index points to a valid column
	_ASSERTE(n_block_index < m_block_cols_list[n_column_index].block_list.size()); // make sure n_block_index selects a block in this column
	const TColumn::TBlockEntry &r_block = m_block_cols_list[n_column_index].block_list[n_block_index];
	_ASSERTE(n_block_row_num == m_block_rows_list[r_block.first].n_height &&
		n_block_column_num == m_block_cols_list[n_column_index].n_width);
	return typename CMakeMatrixRef<n_block_row_num, n_block_column_num>::_Ty(r_block.second);
}

inline size_t CUberBlockMatrix::n_Row_Num() const
{
	_ASSERTE((!m_n_row_num && m_block_rows_list.empty()) ||
		(m_n_row_num && m_block_rows_list.back().n_height +
		m_block_rows_list.back().n_cumulative_height_sum == m_n_row_num));
	return m_n_row_num;
}

inline size_t CUberBlockMatrix::n_Column_Num() const
{
	_ASSERTE((!m_n_col_num && m_block_cols_list.empty()) ||
		(m_n_col_num && m_block_cols_list.back().n_width +
		m_block_cols_list.back().n_cumulative_width_sum == m_n_col_num));
	return m_n_col_num;
}

inline size_t CUberBlockMatrix::n_Block_Num() const
{
	size_t n_num = 0;
	for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i)
		n_num += m_block_cols_list[i].block_list.size();
	return n_num;
}

inline bool CUberBlockMatrix::b_Empty() const
{
	bool b_empty = !m_n_row_num && !m_n_col_num;
	_ASSERTE(!b_empty || m_block_rows_list.empty()); // if empty, the layouts should also be empty
	_ASSERTE(!b_empty || m_block_cols_list.empty()); // if empty, the layouts should also be empty
	_ASSERTE(!b_empty || m_data_pool.empty()); // if empty, the pool should also be empty
	_ASSERTE(!b_empty || !m_n_ref_elem_num); // if empty, m_n_ref_elem_num should also be null
	return b_empty;
}

inline CUberBlockMatrix::_TyMatrixXdRef CUberBlockMatrix::t_FindBlock(size_t n_row, size_t n_column,
	size_t n_block_row_num, size_t n_block_column_num, bool b_alloc_if_not_found /*= true*/,
	bool b_mind_uninitialized /*= true*/) // throw(std::bad_alloc)
{
	double *p_data = p_FindBlock(n_row, n_column, n_block_row_num,
		n_block_column_num, b_alloc_if_not_found, b_mind_uninitialized);
	if(p_data)
		return _TyMatrixXdRef(p_data, n_block_row_num, n_block_column_num);
	else
		return _TyMatrixXdRef(0, 0, 0); // fixme - does this throw an exception or what?
}

inline CUberBlockMatrix::_TyMatrixXdRef CUberBlockMatrix::t_FindBlock(size_t n_row, size_t n_column)
{
	size_t n_block_row_num, n_block_column_num;
	double *p_data = p_FindBlock_ResolveSize(n_row, n_column, n_block_row_num, n_block_column_num);
	if(p_data)
		return _TyMatrixXdRef(p_data, n_block_row_num, n_block_column_num);
	else
		return _TyMatrixXdRef(0, 0, 0); // fixme - does this throw an exception or what?
}

inline CUberBlockMatrix::_TyConstMatrixXdRef CUberBlockMatrix::t_FindBlock(size_t n_row, size_t n_column) const
{
	size_t n_block_row_num, n_block_column_num;
	const double *p_data = p_FindBlock_ResolveSize(n_row, n_column, n_block_row_num, n_block_column_num);
	if(p_data)
		return _TyConstMatrixXdRef(p_data, n_block_row_num, n_block_column_num);
	else
		return _TyConstMatrixXdRef(0, 0, 0); // fixme - does this throw an exception or what?
}

inline CUberBlockMatrix::_TyConstMatrixXdRef CUberBlockMatrix::t_GetBlock_Log(size_t n_row_index, size_t n_column_index) const
{
	_ASSERTE(n_row_index < m_block_rows_list.size());
	_ASSERTE(n_column_index < m_block_cols_list.size());
	size_t n_block_row_num = m_block_rows_list[n_row_index].n_height;
	size_t n_block_column_num = m_block_cols_list[n_column_index].n_width;
	const double *p_data = p_GetBlockData(n_row_index, n_column_index);
	if(p_data)
		return _TyConstMatrixXdRef(p_data, n_block_row_num, n_block_column_num);
	else
		return _TyConstMatrixXdRef(0, 0, 0); // fixme - does this throw an exception or what?
}

inline CUberBlockMatrix::_TyMatrixXdRef CUberBlockMatrix::t_GetBlock_Log(size_t n_row_index, size_t n_column_index)
{
	_ASSERTE(n_row_index < m_block_rows_list.size());
	_ASSERTE(n_column_index < m_block_cols_list.size());
	size_t n_block_row_num = m_block_rows_list[n_row_index].n_height;
	size_t n_block_column_num = m_block_cols_list[n_column_index].n_width;
	double *p_data = p_GetBlockData(n_row_index, n_column_index);
	if(p_data)
		return _TyMatrixXdRef(p_data, n_block_row_num, n_block_column_num);
	else
		return _TyMatrixXdRef(0, 0, 0); // fixme - does this throw an exception or what?
}

inline const double *CUberBlockMatrix::p_GetBlock_Log(size_t n_row_index, size_t n_column_index,
	size_t n_block_row_num, size_t n_block_column_num) const
{
	_ASSERTE(n_row_index < m_block_rows_list.size());
	_ASSERTE(n_column_index < m_block_cols_list.size());
	if(n_block_row_num != m_block_rows_list[n_row_index].n_height ||
	   n_block_column_num != m_block_cols_list[n_column_index].n_width)
		return 0; // size mismatch
	return p_GetBlockData(n_row_index, n_column_index);
}

#ifdef __UBER_BLOCK_MATRIX_IO

inline bool CUberBlockMatrix::Load_MatrixMarket(const char *p_s_filename,
	const char *p_s_layout_filename) // throw(std::bad_alloc)
{
	_ASSERTE(p_s_filename);
	_ASSERTE(p_s_layout_filename); // must always be specified in this version

	if(!Load_BlockLayout(p_s_layout_filename))
		return false;
	// load layout first

	return Load_MatrixMarket_Layout(p_s_filename);
}

#endif // __UBER_BLOCK_MATRIX_IO

inline CUberBlockMatrix::TColumn CUberBlockMatrix::t_ColumnCumsumCopy(const TColumn &r_t_src)
{
	TColumn t_dest;
	t_dest.n_width = r_t_src.n_width; // copy dimensions
	t_dest.n_cumulative_width_sum = r_t_src.n_cumulative_width_sum; // copy cumsums
	_ASSERTE(t_dest.block_list.empty()); // no blocks
	return t_dest;
}

inline CUberBlockMatrix::TRow CUberBlockMatrix::t_ColumnCumsumToRowCumsum(const TColumn &r_t_src)
{
	TRow t_dest;
	t_dest.n_height = r_t_src.n_width; // copy dimensions
	t_dest.n_cumulative_height_sum = r_t_src.n_cumulative_width_sum; // copy cumsums
	// no blocks in dest
	return t_dest;
}

inline CUberBlockMatrix::TColumn CUberBlockMatrix::t_RowCumsumToColumnCumsum(const TRow &r_t_src)
{
	TColumn t_dest;
	t_dest.n_width = r_t_src.n_height; // copy dimensions
	t_dest.n_cumulative_width_sum = r_t_src.n_cumulative_height_sum; // copy cumsums
	_ASSERTE(t_dest.block_list.empty()); // no blocks in src
	return t_dest;
}

template <class _TyDest, class _TyFirst, class _TySecond>
void CUberBlockMatrix::MergeLayout(std::vector<_TyDest> &r_merged_layout, const std::vector<_TyFirst> &r_layout_0,
	const std::vector<_TySecond> &r_layout_1, std::vector<size_t> &r_reindexing_table_0,
	std::vector<size_t> &r_reindexing_table_1) // throw(std::bad_alloc)
{
	r_reindexing_table_0.resize(r_layout_0.size());
	r_reindexing_table_1.resize(r_layout_1.size());
	r_merged_layout.clear(); // note this is always called from inside function where the layout is created, clearing is useless then
	r_merged_layout.reserve(std::max(r_layout_0.size(), r_layout_1.size()));
	// alloc in / out lists

	_ASSERTE(r_layout_0.empty() == r_layout_1.empty()); // both empty or both not
	_ASSERTE(r_layout_0.empty() || r_layout_1.empty() ||
		r_layout_0.back().n_GetCumulative() + r_layout_0.back().n_GetAbsolute() ==
		r_layout_1.back().n_GetCumulative() + r_layout_1.back().n_GetAbsolute()); // same last dimension
	// make sure matrix size is the same

	size_t n_last_cum = 0, i = 0, j = 0, m = r_layout_0.size(), n = r_layout_1.size();
	size_t n_last_first = 0;
	size_t n_last_second = 0;
	while(i < m && j < n) {
		size_t n_cum0 = r_layout_0[i].n_GetCumulative() + r_layout_0[i].n_GetAbsolute();
		size_t n_cum1 = r_layout_1[j].n_GetCumulative() + r_layout_1[j].n_GetAbsolute(); // want the end of the row/col
		size_t n_cum_next = std::min(n_cum0, n_cum1);
		// get cumsums, decide which to advance first

		size_t n_index = r_merged_layout.size();
		if(n_cum0 == n_cum_next) {
			r_reindexing_table_0[i] = (n_last_first == n_last_cum)? n_index : size_t(-1);
			n_last_first = n_cum_next;
			++ i;
		}
		if(n_cum1 == n_cum_next) {
			r_reindexing_table_1[j] = (n_last_second == n_last_cum)? n_index : size_t(-1);
			n_last_second = n_cum_next;
			++ j;
		}
		// build reindexing tables

		_TyDest t_row_or_column;
		t_row_or_column.SetAbsolute(n_cum_next - n_last_cum);
		t_row_or_column.SetCumulative(n_last_cum); // !!
		n_last_cum = n_cum_next;
		// make a new row (or a column, depending on _TyDest)

		r_merged_layout.push_back(t_row_or_column);
		// add it to the list
	}
	_ASSERTE(i == m && j == n); // in case the matrices have the same size, they both end at the same time
}

template <class _TyFirst, class _TySecond>
size_t CUberBlockMatrix::n_MergeLayout(const std::vector<_TyFirst> &r_layout_0,
	const std::vector<_TySecond> &r_layout_1, std::vector<size_t> &r_reindexing_table_0,
	std::vector<size_t> &r_reindexing_table_1) // throw(std::bad_alloc)
{
	r_reindexing_table_0.resize(r_layout_0.size());
	r_reindexing_table_1.resize(r_layout_1.size());
	// alloc out lists

	_ASSERTE(r_layout_0.empty() == r_layout_1.empty()); // both empty or both not
	_ASSERTE(r_layout_0.empty() || r_layout_1.empty() ||
		r_layout_0.back().n_GetCumulative() + r_layout_0.back().n_GetAbsolute() ==
		r_layout_1.back().n_GetCumulative() + r_layout_1.back().n_GetAbsolute()); // same last dimension
	// make sure matrix size is the same

	size_t n_layout_size = 0;
	size_t n_last_cum = 0, i = 0, j = 0, m = r_layout_0.size(), n = r_layout_1.size();
	size_t n_last_first = 0;
	size_t n_last_second = 0;
	while(i < m && j < n) {
		size_t n_cum0 = r_layout_0[i].n_GetCumulative() + r_layout_0[i].n_GetAbsolute();
		size_t n_cum1 = r_layout_1[j].n_GetCumulative() + r_layout_1[j].n_GetAbsolute(); // want the end of the row/col
		size_t n_cum_next = std::min(n_cum0, n_cum1);
		// get cumsums, decide which to advance first

		size_t n_index = n_layout_size;
		if(n_cum0 == n_cum_next) {
			r_reindexing_table_0[i] = (n_last_first == n_last_cum)? n_index : size_t(-1);
			n_last_first = n_cum_next;
			++ i;
		}
		if(n_cum1 == n_cum_next) {
			r_reindexing_table_1[j] = (n_last_second == n_last_cum)? n_index : size_t(-1);
			n_last_second = n_cum_next;
			++ j;
		}
		n_last_cum = n_cum_next;
		// build reindexing tables

		++ n_layout_size;
		// calculate the size of the resulting layout
	}
	_ASSERTE(i == m && j == n); // in case the matrices have the same size, they both end at the same time

	return n_layout_size;
}

inline size_t CUberBlockMatrix::n_RowGet(size_t n_row, size_t &r_n_block_row_num) const
{
	size_t n_row_index;
	if(n_row >= m_n_row_num)
		return -1;
	else {
		_TyRowConstIter p_row_it = std::upper_bound(m_block_rows_list.begin(),
			m_block_rows_list.end(), n_row, CFindLowerRow()); // t_odo
		//_ASSERTE(p_row_it != m_block_rows_list.end());
		if(p_row_it == m_block_rows_list.begin())
			return -1;
		-- p_row_it;
		const TRow &r_t_row = *p_row_it;

		if(r_t_row.n_cumulative_height_sum == n_row) {
			r_n_block_row_num = r_t_row.n_height;
			n_row_index = p_row_it - m_block_rows_list.begin();
			// just find the index, the row in question does exist, and has the right size (ideal case)

#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
			++ m_n_row_reref_num;
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS
		} else {
			return -1;
			// the current row would need to be fragmented
		}
	}
	// resolve row reference

	_ASSERTE(n_row_index < m_block_rows_list.size());
	_ASSERTE(m_block_rows_list[n_row_index].n_height == r_n_block_row_num);
	_ASSERTE(m_block_rows_list[n_row_index].n_cumulative_height_sum == n_row);
	// make sure that the row reference is really resolved

	//_ASSERTE(b_last_row == (n_row_index == m_block_rows_list.size() - 1));
	// makes sure the b_last_row flag is filled correctly

	return n_row_index;
}

inline size_t CUberBlockMatrix::n_ColumnGet(size_t n_column, size_t &r_n_block_column_num) const
{
	size_t n_column_index;
	if(n_column >= m_n_col_num)
		return -1;
	else {
		_TyColumnConstIter p_col_it;
		_ASSERTE(!m_block_cols_list.empty());
		/*if(m_block_cols_list.back().n_cumulative_width_sum == n_column) // saves almost 10% of the time if filling by columns
			p_col_it = m_block_cols_list.end() - 1;
		else*/ {
			p_col_it = std::upper_bound(m_block_cols_list.begin(),
				m_block_cols_list.end(), n_column, CFindLowerColumn()); // t_odo
			//_ASSERTE(p_col_it != m_block_cols_list.end()); // t_odo - handle this as proper runtime error // done (the line below)
			if(p_col_it == m_block_cols_list.begin())
				return -1;
			-- p_col_it;
		}
		const TColumn &r_t_col = *p_col_it;

		if(r_t_col.n_cumulative_width_sum == n_column) {
			r_n_block_column_num = r_t_col.n_width;
			n_column_index = p_col_it - m_block_cols_list.begin();
			// just find the index, the column in question does exist, and has the right size (ideal case)

#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
			++ m_n_col_reref_num;
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS
		} else {
			return -1;
			// handle the subdivision cases
		}
	}
	// resolve column reference (essentially the same code as for the row reference)

	_ASSERTE(n_column_index < m_block_cols_list.size());
	_ASSERTE(m_block_cols_list[n_column_index].n_width == r_n_block_column_num);
	_ASSERTE(m_block_cols_list[n_column_index].n_cumulative_width_sum == n_column);
	// make sure that the column reference is really resolved

	return n_column_index;
}

inline double *CUberBlockMatrix::p_AllocBlockData(size_t n_row_index, size_t n_column_index,
	size_t n_block_row_num, size_t n_block_column_num, bool &r_b_was_a_new_block) // throw(std::bad_alloc)
{
	_ASSERTE(n_column_index < m_block_cols_list.size());
	TColumn &r_t_col = m_block_cols_list[n_column_index];
	return p_AllocBlockData(n_row_index, r_t_col, n_block_row_num,
		n_block_column_num, r_b_was_a_new_block);
}

template <class CMatrixType>
bool CUberBlockMatrix::Append_Block(const CMatrixType &r_t_block, size_t n_row, size_t n_column) // throw(std::bad_alloc)
{
	//CheckIntegrity(); // not here
	// make sure the matrix is ok

	size_t n_row_index, n_column_index;
	if((n_row_index = n_RowAlloc(n_row, r_t_block.rows())) == size_t(-1) ||
	   (n_column_index = n_ColumnAlloc(n_column, r_t_block.cols())) == size_t(-1))
		return false;
	// find row / column

	double *p_data;
	bool b_uninitialized; // ignored
	if(!(p_data = p_AllocBlockData(n_row_index, n_column_index,
	   r_t_block.rows(), r_t_block.cols(), b_uninitialized)))
		return false;
	// allocate a new block / reuse existing one

	Eigen::Map<Eigen::MatrixXd, map_Alignment> dest(p_data, r_t_block.rows(), r_t_block.cols());
	dest = r_t_block; // can't always use memcpy (matrix expressions, matrices with stride, ...)
	// copy the dense data

	return true;
}

template <int n_compile_time_row_num, int n_compile_time_col_num, int n_options>
bool CUberBlockMatrix::Append_Block(const Eigen::Matrix<double, n_compile_time_row_num, n_compile_time_col_num,
	n_options, n_compile_time_row_num, n_compile_time_col_num> &r_t_block, size_t n_row, size_t n_column) // throw(std::bad_alloc)
{
	//CheckIntegrity(); // not here
	// make sure the matrix is ok

	size_t n_row_index, n_column_index;
	if((n_row_index = n_RowAlloc(n_row, r_t_block.rows())) == size_t(-1) ||
	   (n_column_index = n_ColumnAlloc(n_column, r_t_block.cols())) == size_t(-1))
		return false;
	// find row / column

	double *p_data;
	bool b_uninitialized; // ignored
	if(!(p_data = p_AllocBlockData(n_row_index, n_column_index,
	   r_t_block.rows(), r_t_block.cols(), b_uninitialized)))
		return false;
	// allocate a new block / reuse existing one

	memcpy(p_data, &r_t_block(0, 0), r_t_block.rows() * r_t_block.cols() * sizeof(double));
	// copy the dense data

	return true;
}

template <class CMatrixType>
bool CUberBlockMatrix::Append_Block_Log(const CMatrixType &r_t_block, size_t n_row_index, size_t n_column_index) // throw(std::bad_alloc)
{
	_ASSERTE(n_row_index <= m_block_rows_list.size());
	_ASSERTE(n_column_index <= m_block_cols_list.size());

	//CheckIntegrity(); // not here
	// make sure the matrix is ok

	if(n_row_index == m_block_rows_list.size() &&
	   (n_row_index = n_RowAlloc(m_n_row_num, r_t_block.rows())) == size_t(-1))
		return false;
	else if(m_block_rows_list[n_row_index].n_height != size_t(r_t_block.rows()))
		return false;
	if(n_column_index == m_block_cols_list.size() &&
	   (n_column_index = n_ColumnAlloc(m_n_col_num, r_t_block.cols())) == size_t(-1))
		return false;
	else if(m_block_cols_list[n_column_index].n_width != size_t(r_t_block.cols()))
		return false;
	// in case either index points one past the last row / column, creates a new one
	// note that n_ColumnAlloc() / n_RowAlloc() check dimensions,
	// if not used, the dimensions must be checked here

	double *p_data;
	bool b_uninitialized; // ignored
	if(!(p_data = p_AllocBlockData(n_row_index, n_column_index,
	   r_t_block.rows(), r_t_block.cols(), b_uninitialized)))
		return false;
	// allocate a new block / reuse existing one

	Eigen::Map<Eigen::MatrixXd, map_Alignment> dest(p_data, r_t_block.rows(), r_t_block.cols());
	dest = r_t_block; // can't always use memcpy (matrix expressions, matrices with stride, ...)
	// copy the dense data

	return false;
}

template <int n_compile_time_row_num, int n_compile_time_col_num, int n_options>
bool CUberBlockMatrix::Append_Block_Log(const Eigen::Matrix<double, n_compile_time_row_num, n_compile_time_col_num,
	n_options, n_compile_time_row_num, n_compile_time_col_num> &r_t_block, size_t n_row_index, size_t n_column_index) // throw(std::bad_alloc)
{
	_ASSERTE(n_row_index <= m_block_rows_list.size());
	_ASSERTE(n_column_index <= m_block_cols_list.size());

	//CheckIntegrity(); // not here
	// make sure the matrix is ok

	if(n_row_index == m_block_rows_list.size() &&
	   (n_row_index = n_RowAlloc(m_n_row_num, r_t_block.rows())) == size_t(-1))
		return false;
	else if(m_block_rows_list[n_row_index].n_height != r_t_block.rows())
		return false;
	if(n_column_index == m_block_cols_list.size() &&
	   (n_column_index = n_ColumnAlloc(m_n_col_num, r_t_block.cols())) == size_t(-1))
		return false;
	else if(m_block_cols_list[n_column_index].n_width != r_t_block.cols())
		return false;
	// in case either index points one past the last row / column, creates a new one
	// note that n_ColumnAlloc() / n_RowAlloc() check dimensions,
	// if not used, the dimensions must be checked here

	double *p_data;
	bool b_uninitialized; // ignored
	if(!(p_data = p_AllocBlockData(n_row_index, n_column_index,
	   r_t_block.rows(), r_t_block.cols(), b_uninitialized)))
		return false;
	// allocate a new block / reuse existing one

	memcpy(p_data, &r_t_block(0, 0), r_t_block.rows() * r_t_block.cols() * sizeof(double));
	// copy the dense data

	return true; // success
}

template <class CEigenMatrixType>
void CUberBlockMatrix::Get_Diagonal(CEigenMatrixType&r_dest,
	bool b_is_upper_triangular /*= false*/) const // throw(std::bad_alloc)
{
	_ASSERTE(b_SymmetricLayout());
	// otherwise the diagonal of the matrix does not match the diagonal of the
	// columns and this becomes really messy (but todo fot the future)

	size_t n_diag_length = std::min(m_n_row_num, m_n_col_num);
	r_dest.resize(n_diag_length);
	_ASSERTE(r_dest.rows() == n_diag_length && r_dest.cols() == 1); // make sure this is a correct size vector
	// resize the vector

	size_t n_prev_column_end = 0; // where we last wrote to the diagonal
	for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
		const TColumn &r_col = m_block_cols_list[i];
		size_t n_width = r_col.n_width, n_base = r_col.n_cumulative_width_sum;

		if(r_col.block_list.empty())
			continue;
		// skip empty columns, will zero fill before the next nonzero or at the end

		if(b_is_upper_triangular) {
			if(n_prev_column_end < n_base)
				r_dest.segment(n_prev_column_end, n_base - n_prev_column_end).setZero();
			n_prev_column_end = n_width + n_base;
			// take care of zero parts of the diagonal

			const TColumn::TBlockEntry &r_block = r_col.block_list.back();
			// get a block

			const TRow &r_row = m_block_rows_list[r_block.first];
			_ASSERTE(r_row.n_height == n_width && r_row.n_cumulative_height_sum == n_base);
			// make sure that this is really a diagonal block

			_TyConstMatrixXdRef t_block(r_block.second, n_width, n_width);
			r_dest.segment(n_base, n_width) = t_block.diagonal();
			// copy diagonal of the block to the diagonal vector
		} else {
			size_t n_height;
			size_t n_block_row = n_RowGet(n_base, n_height);
			if(n_block_row == size_t(-1))
				continue;
			// get a block row

			const TRow &r_row = m_block_rows_list[n_block_row];
			_ASSERTE(r_row.n_height == n_width && r_row.n_cumulative_height_sum == n_base);
			// make sure that this is really a diagonal block (if there is one at the given row)

			const double *p_block = p_GetBlockData(n_block_row, i);
			if(!p_block)
				continue; // zero diagonal
			// find the block

			if(n_prev_column_end < n_base)
				r_dest.segment(n_prev_column_end, n_base - n_prev_column_end).setZero();
			n_prev_column_end = n_width + n_base;
			// take care of zero parts of the diagonal

			_TyConstMatrixXdRef t_block(p_block, n_width, n_width);
			r_dest.segment(n_base, n_width) = t_block.diagonal();
			// copy diagonal of the block to the diagonal vector
		}
	}
	// go through the matrix, fill diagonal

	if(n_prev_column_end < n_diag_length)
		r_dest.segment(n_prev_column_end, n_diag_length - n_prev_column_end).setZero();
	// in case the diagonal is sparse, zero the tail of the vector

#ifdef _DEBUG
	if(m_n_row_num <= 4096 && m_n_col_num <= 4096) { // don't run out of memory here (4k * 4k = 128 MB matrix)
		Eigen::MatrixXd dense;
		Convert_to_Dense(dense);
		_ASSERTE(dense.diagonal() == r_dest); // make sure that the diagonal extracted is correct
	}
#endif // _DEBUG
	// potentially slow debug check
}

template <class Derived0>
void CUberBlockMatrix::Convert_to_Dense(Eigen::MatrixBase<Derived0> &r_dest) const // throw(std::bad_alloc)
{
	CheckIntegrity(true);

	//if(Derived0::RowsAtCompileTime == Eigen::Dynamic || Derived0::ColsAtCompileTime == Eigen::Dynamic)
	//	r_dest.resize(m_n_row_num, m_n_col_num); // in case only one of the dimensions is fixed
	_ASSERTE(r_dest.rows() == m_n_row_num && r_dest.cols() == m_n_col_num);
	r_dest.setZero();
	// alloc dest matrix

	for(size_t j = 0, n_col_num = m_block_cols_list.size(); j < n_col_num; ++ j) {
		const TColumn &t_col = m_block_cols_list[j];

		size_t n_x = t_col.n_cumulative_width_sum;
		size_t n_width = t_col.n_width;
		// dimensions of the block

		for(size_t i = 0, n_block_num = t_col.block_list.size(); i < n_block_num; ++ i) {
			size_t n_row = t_col.block_list[i].first;
			const double *p_data = t_col.block_list[i].second;

			size_t n_y = m_block_rows_list[n_row].n_cumulative_height_sum;
			size_t n_height = m_block_rows_list[n_row].n_height;
			// dimensions of the block

			for(size_t x = 0; x < n_width; ++ x) {
				for(size_t y = 0; y < n_height; ++ y) {
					double f_elem = p_data[x * n_height + y]; // t_odo - this is probably the other way around // nope, it's this way
					r_dest(n_y + y, n_x + x) = f_elem;
				}
			}
			// iterate through data, sparse fill ...
		}
	}
}

template <class COp>
void CUberBlockMatrix::ElementwiseUnaryOp_ZeroInvariant(COp op)
{
	CheckIntegrity(true);

	for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
		const TColumn &r_t_col = m_block_cols_list[i];
		const size_t n_block_width = r_t_col.n_width;
		for(size_t j = 0, m = r_t_col.block_list.size(); j < m; ++ j) {
			size_t n_row_idx = r_t_col.block_list[j].first;
			double *p_data = r_t_col.block_list[j].second;
			size_t n_block_height = m_block_rows_list[n_row_idx].n_height;
			// get block position, size and data

			for(const double *p_end = p_data + n_block_width * n_block_height; p_data != p_end; ++ p_data)
				*p_data = op(*p_data);
			// modify the block
		}
	}
}

template <class COp>
void CUberBlockMatrix::ElementwiseUnaryOp_ZeroInvariant_Parallel(COp op)
{
	CheckIntegrity(true);

#ifdef _OPENMP
	_ASSERTE(m_block_cols_list.size() <= INT_MAX);
	int n = int(m_block_cols_list.size());
	#pragma omp parallel for default(shared)
	for(int i = 0; i < n; ++ i) {
		const TColumn &r_t_col = m_block_cols_list[i];
#else // _OPENMP
	for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
		const TColumn &r_t_col = m_block_cols_list[i];
#endif // OPENMP
		const size_t n_block_width = r_t_col.n_width;
		for(size_t j = 0, m = r_t_col.block_list.size(); j < m; ++ j) {
			size_t n_row_idx = r_t_col.block_list[j].first;
			double *p_data = r_t_col.block_list[j].second;
			size_t n_block_height = m_block_rows_list[n_row_idx].n_height;
			// get block position, size and data

			for(const double *p_end = p_data + n_block_width * n_block_height; p_data != p_end; ++ p_data)
				*p_data = op(*p_data);
			// modify the block
		}
	}
}

template <class CBinaryOp>
bool CUberBlockMatrix::ElementwiseBinaryOp_ZeroInvariant(CUberBlockMatrix &r_dest, CBinaryOp op) const // throw(std::bad_alloc)
{
	CheckIntegrity(true);
	r_dest.CheckIntegrity(true);

	// t_odo - decide whether to optimize for addition of matrices with exactly the same block layout (no need to allocate anything, but will anyone use that?) // no
	// t_odo - optimize for matrices with no empty blocks / columns (such as in SLAM), that probably need to be another class // can't see what the optimization was anymore
	// note - this one is a bit hard, as the layout might be completely different in case there are dummy rows / columns,
	//		it is therefore hard to check the layout, and b_EqualStructure() doesn't help. maybe need something like b_CompatibleStructure()

	if(r_dest.m_n_row_num != m_n_row_num || r_dest.m_n_col_num != m_n_col_num)
		return false;
	// the dimensions must be the same

	const std::vector<TRow> &r_row_list_first = m_block_rows_list;
	const std::vector<TColumn> &r_column_list_first = m_block_cols_list;
	std::vector<TRow> &r_row_list_second = r_dest.m_block_rows_list;
	std::vector<TColumn> &r_column_list_second = r_dest.m_block_cols_list;

	std::vector<size_t> row_mapping;
	std::vector<size_t> column_mapping; // t_odo - pack this code to a function
	{
		if(!Build_AdditionLayouts(r_row_list_first, r_column_list_first,
		   r_row_list_second, r_column_list_second, row_mapping, column_mapping))
			return false;
		//r_dest.CheckIntegrity(); // make sure that this operation didn't damage matrix integrity
	}
	// reorganize the destination matrix so that the layout is compatible with this (if required)
	// the (amortized) complexity is linear, about O(row blocks + col blocks)

	for(size_t i = 0, n = r_column_list_first.size(); i < n; ++ i) {
		const TColumn &r_t_col = r_column_list_first[i];
		if(r_t_col.block_list.empty())
			continue;
		// skip empty columns ...

		size_t n_dest = column_mapping[i];
		if(n_dest == size_t(-1))
			return false; // this column was split, the blocks in it are now homeless
		TColumn &r_t_dest_col = r_column_list_second[n_dest];
		// find dest column

		_TyBlockIter p_first_it = r_t_dest_col.block_list.begin();
		// where to begin searching for blocks

		size_t j = 0, m = r_t_col.block_list.size();
		if(!r_t_dest_col.block_list.empty()) {
			for(; j < m; ++ j) {
				const TColumn::TBlockEntry &r_t_block = r_t_col.block_list[j];
				size_t n_old_row, n_new_row;
				if((n_new_row = row_mapping[n_old_row = r_t_block.first]) == size_t(-1))
					return false; // this row was split, the blocks in it are now homeless
				// get block and its new row

				double *p_value_src = r_t_block.second;
				size_t n_block_size = r_t_col.n_width * m_block_rows_list[n_old_row].n_height;
				// get block data

				_TyBlockIter p_block_it = std::lower_bound(p_first_it,
					r_t_dest_col.block_list.end(), n_new_row, CCompareBlockRow());
				// find where to put the block in the column

				bool b_insert_at_end;
				if((b_insert_at_end = (p_block_it == r_t_dest_col.block_list.end())) ||
				   (*p_block_it).first != n_new_row) {
					// a new block

					double *p_value = r_dest.p_Get_DenseStorage(n_block_size);
					memcpy(p_value, p_value_src, n_block_size * sizeof(double));
					// create a new block, initialize with values

					p_first_it = r_t_dest_col.block_list.insert(p_block_it, TColumn::TBlockEntry(n_new_row, p_value));
					// add it to the list, remember iterator

					++ p_first_it;
					// next time search from here (look out for iterator invalidation)

					if(b_insert_at_end) {
						_ASSERTE(p_first_it == r_t_dest_col.block_list.end()); // added to the end
						++ j; // don't forget to count this block as processed
						break; // blocks are sorted, will be always adding to the end from now on
					}
					// in case inserted at end, can continue using the optimized loop below
				} else {
					double *p_value_dest = (*p_block_it).second;
					// get existing block data

					for(double *p_value_end = p_value_dest + n_block_size;
					   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
						*p_value_dest = op(*p_value_dest, *p_value_src); // t_odo - replace by op()
					// add values to an existing block

					p_first_it = p_block_it + 1;
					// next time, search from here
				}
			}
		}
		for(; j < m; ++ j) {
			const TColumn::TBlockEntry &r_t_block = r_t_col.block_list[j];
			size_t n_old_row, n_new_row;
			if((n_new_row = row_mapping[n_old_row = r_t_block.first]) == size_t(-1))
				return false; // this row was split, the blocks in it are now homeless

			double *p_value_src = r_t_block.second;
			size_t n_block_size = r_t_col.n_width * m_block_rows_list[n_old_row].n_height;
			// get block data

			_ASSERTE(r_t_dest_col.block_list.empty() || r_t_dest_col.block_list.back().first < n_new_row);
			// make sure the new block comes at the end of the row

			double *p_value = r_dest.p_Get_DenseStorage(n_block_size);
			memcpy(p_value, p_value_src, n_block_size * sizeof(double));
			// create a new block, initialize with values

			r_t_dest_col.block_list.push_back(TColumn::TBlockEntry(n_new_row, p_value));
			// add it to the list
		}
		// merge blocks
	}

	//r_dest.CheckIntegrity(true);
	// make sure that this operation didn't damage matrix integrity

	return true;
}

template <class CBinaryOp>
bool CUberBlockMatrix::ElementwiseBinaryOp_RightSideZeroInvariant(CUberBlockMatrix &r_dest, CBinaryOp op) const // throw(std::bad_alloc)
{
	CheckIntegrity(true);
	r_dest.CheckIntegrity(true);

	if(r_dest.m_n_row_num != m_n_row_num || r_dest.m_n_col_num != m_n_col_num)
		return false;
	// the dimensions must be the same

	const std::vector<TRow> &r_row_list_first = m_block_rows_list;
	const std::vector<TColumn> &r_column_list_first = m_block_cols_list;
	std::vector<TRow> &r_row_list_second = r_dest.m_block_rows_list;
	std::vector<TColumn> &r_column_list_second = r_dest.m_block_cols_list;

	std::vector<size_t> row_mapping;
	std::vector<size_t> column_mapping; // t_odo - pack this code to a function
	{
		if(!Build_AdditionLayouts(r_row_list_first, r_column_list_first,
		   r_row_list_second, r_column_list_second, row_mapping, column_mapping))
			return false;
		//r_dest.CheckIntegrity(); // make sure that this operation didn't damage matrix integrity
	}
	// reorganize the destination matrix so that the layout is compatible with this (if required)
	// the (amortized) complexity is linear, about O(row blocks + col blocks)

	for(size_t i = 0, n = r_column_list_first.size(); i < n; ++ i) {
		const TColumn &r_t_col = r_column_list_first[i];
		if(r_t_col.block_list.empty())
			continue;
		// skip empty columns ...

		size_t n_dest = column_mapping[i];
		if(n_dest == size_t(-1))
			return false; // this column was split, the blocks in it are now homeless
		TColumn &r_t_dest_col = r_column_list_second[n_dest];
		// find dest column

		_TyBlockIter p_first_it = r_t_dest_col.block_list.begin();
		// where to begin searching for blocks

		size_t j = 0, m = r_t_col.block_list.size();
		if(!r_t_dest_col.block_list.empty()) {
			for(; j < m; ++ j) {
				const TColumn::TBlockEntry &r_t_block = r_t_col.block_list[j];
				size_t n_old_row, n_new_row;
				if((n_new_row = row_mapping[n_old_row = r_t_block.first]) == size_t(-1))
					return false; // this row was split, the blocks in it are now homeless
				// get block and its new row

				double *p_value_src = r_t_block.second;
				size_t n_block_size = r_t_col.n_width * m_block_rows_list[n_old_row].n_height;
				// get block data

				_TyBlockIter p_block_it = std::lower_bound(p_first_it,
					r_t_dest_col.block_list.end(), n_new_row, CCompareBlockRow());
				// find where to put the block in the column

				bool b_insert_at_end;
				if((b_insert_at_end = (p_block_it == r_t_dest_col.block_list.end())) ||
				   (*p_block_it).first != n_new_row) {
					// a new block

					double *p_value = r_dest.p_Get_DenseStorage(n_block_size);
					double *p_value_dest = p_value;
					for(double *p_value_end = p_value_dest + n_block_size;
					   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
						*p_value_dest = op(0, *p_value_src); // t_odo - replace by op() // note op needs to be zero-invariant (blocks of the second matrix that are missed by blocks of the first matrix are also untouched)
					// create a new block, initialize with values

					p_first_it = r_t_dest_col.block_list.insert(p_block_it, TColumn::TBlockEntry(n_new_row, p_value));
					// add it to the list, remember iterator

					++ p_first_it;
					// next time search from here (look out for iterator invalidation)

					if(b_insert_at_end) {
						_ASSERTE(p_first_it == r_t_dest_col.block_list.end()); // added to the end
						++ j; // don't forget to count this block as processed
						break; // blocks are sorted, will be always adding to the end from now on
					}
					// in case inserted at end, can continue using the optimized loop below
				} else {
					double *p_value_dest = (*p_block_it).second;
					// get existing block data

					for(double *p_value_end = p_value_dest + n_block_size;
					   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
						*p_value_dest = op(*p_value_dest, *p_value_src); // t_odo - replace by op()
					// add values to an existing block

					p_first_it = p_block_it + 1;
					// next time, search from here
				}
			}
		}
		for(; j < m; ++ j) {
			const TColumn::TBlockEntry &r_t_block = r_t_col.block_list[j];
			size_t n_old_row, n_new_row;
			if((n_new_row = row_mapping[n_old_row = r_t_block.first]) == size_t(-1))
				return false; // this row was split, the blocks in it are now homeless

			double *p_value_src = r_t_block.second;
			size_t n_block_size = r_t_col.n_width * m_block_rows_list[n_old_row].n_height;
			// get block data

			_ASSERTE(r_t_dest_col.block_list.empty() || r_t_dest_col.block_list.back().first < n_new_row);
			// make sure the new block comes at the end of the row

			double *p_value = r_dest.p_Get_DenseStorage(n_block_size);
			double *p_value_dest = p_value;
			for(double *p_value_end = p_value_dest + n_block_size;
			   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
				*p_value_dest = op(0, *p_value_src);  // t_odo - replace by op() // note that op needs to be zero-invariant (blocks of the second matrix that are missed by blocks of the first matrix are also untouched)
			// create a new block, initialize with values

			r_t_dest_col.block_list.push_back(TColumn::TBlockEntry(n_new_row, p_value));
			// add it to the list
		}
		// merge blocks
	}

	//r_dest.CheckIntegrity(true);
	// make sure that this operation didn't damage matrix integrity

	return true;
}

template <class CBinaryOp>
bool CUberBlockMatrix::ElementwiseBinaryOp(CUberBlockMatrix &r_dest, CBinaryOp op) const // throw(std::bad_alloc)
{
	CheckIntegrity(true);
	r_dest.CheckIntegrity(true);

	if(r_dest.m_n_row_num != m_n_row_num || r_dest.m_n_col_num != m_n_col_num)
		return false;
	// the dimensions must be the same

	const std::vector<TRow> &r_row_list_first = m_block_rows_list;
	const std::vector<TColumn> &r_column_list_first = m_block_cols_list;
	std::vector<TRow> &r_row_list_second = r_dest.m_block_rows_list;
	std::vector<TColumn> &r_column_list_second = r_dest.m_block_cols_list;

	std::vector<size_t> row_mapping;
	std::vector<size_t> column_mapping; // t_odo - pack this code to a function
	{
		if(!Build_AdditionLayouts(r_row_list_first, r_column_list_first,
		   r_row_list_second, r_column_list_second, row_mapping, column_mapping))
			return false;
		//r_dest.CheckIntegrity(); // make sure that this operation didn't damage matrix integrity
	}
	// reorganize the destination matrix so that the layout is compatible with this (if required)
	// the (amortized) complexity is linear, about O(row blocks + col blocks)

	size_t n_last_dest_col = 0;
	for(size_t i = 0, n = r_column_list_first.size(); i < n; ++ i) {
		const TColumn &r_t_col = r_column_list_first[i];
		if(r_t_col.block_list.empty())
			continue;
		// skip empty columns ...

		size_t n_dest = column_mapping[i];
		if(n_dest == size_t(-1))
			return false; // this column was split, the blocks in it are now homeless
		TColumn &r_t_dest_col = r_column_list_second[n_dest];
		// find dest column

		for(; n_last_dest_col != n_dest; ++ n_last_dest_col) {
			TColumn &r_t_dest_col = r_column_list_second[n_last_dest_col];
			for(size_t j = 0, m = r_t_dest_col.block_list.size(); j < m; ++ j) {
				const TColumn::TBlockEntry &r_t_block = r_t_dest_col.block_list[j];

				double *p_value_dest = r_t_block.second;
				size_t n_block_size_dest = r_t_dest_col.n_width * r_row_list_second[r_t_block.first].n_height;
				// get block data

				for(double *p_value_end = p_value_dest + n_block_size_dest;
				   p_value_dest != p_value_end; ++ p_value_dest)
					*p_value_dest = op(*p_value_dest, 0);
			}
		}
		// don't forget to modify all of the previous columns we skipped in dest

		_ASSERTE(n_last_dest_col == n_dest);
		++ n_last_dest_col;
		// and don't modify this column either

		_TyBlockIter p_first_it = r_t_dest_col.block_list.begin();
		// where to begin searching for blocks

		size_t j = 0, m = r_t_col.block_list.size();
		if(!r_t_dest_col.block_list.empty()) {
			for(; j < m; ++ j) {
				const TColumn::TBlockEntry &r_t_block = r_t_col.block_list[j];
				size_t n_old_row, n_new_row;
				if((n_new_row = row_mapping[n_old_row = r_t_block.first]) == size_t(-1))
					return false; // this row was split, the blocks in it are now homeless
				// get block and its new row

				double *p_value_src = r_t_block.second;
				size_t n_block_size = r_t_col.n_width * m_block_rows_list[n_old_row].n_height;
				// get block data

				_TyBlockIter p_block_it = std::lower_bound(p_first_it,
					r_t_dest_col.block_list.end(), n_new_row, CCompareBlockRow());
				// find where to put the block in the column

				for(; p_first_it != p_block_it; ++ p_first_it) {
					size_t n_dest_row = (*p_first_it).first;
					size_t n_block_size_dest = r_t_col.n_width * r_row_list_second[n_dest_row].n_height;
					double *p_value_dest = (*p_first_it).second;
					for(double *p_value_end = p_value_dest + n_block_size_dest;
					   p_value_dest != p_value_end; ++ p_value_dest)
						*p_value_dest = op(*p_value_dest, 0);
				}
				// don't forget to modify all the blocks in dest between last one and the one being added to

				bool b_insert_at_end;
				if((b_insert_at_end = (p_block_it == r_t_dest_col.block_list.end())) ||
				   (*p_block_it).first != n_new_row) {
					// a new block

					double *p_value = r_dest.p_Get_DenseStorage(n_block_size);
					double *p_value_dest = p_value;
					for(double *p_value_end = p_value_dest + n_block_size;
					   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
						*p_value_dest = op(0, *p_value_src); // t_odo - replace by op()
					// create a new block, initialize with values

					p_first_it = r_t_dest_col.block_list.insert(p_block_it, TColumn::TBlockEntry(n_new_row, p_value));
					// add it to the list, remember iterator

					++ p_first_it;
					// next time search from here (look out for iterator invalidation)

					if(b_insert_at_end) {
						_ASSERTE(p_first_it == r_t_dest_col.block_list.end()); // added to the end
						++ j; // don't forget to count this block as processed
						break; // blocks are sorted, will be always adding to the end from now on
					}
					// in case inserted at end, can continue using the optimized loop below
				} else {
					double *p_value_dest = (*p_block_it).second;
					// get existing block data

					for(double *p_value_end = p_value_dest + n_block_size;
					   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
						*p_value_dest = op(*p_value_dest, *p_value_src); // t_odo - replace by op()
					// add values to an existing block

					p_first_it = p_block_it + 1;
					// next time, search from here
				}
			}
		}
		{
			_TyBlockIter p_end_it = r_t_dest_col.block_list.end();
			for(; p_first_it != p_end_it; ++ p_first_it) {
				size_t n_dest_row = (*p_first_it).first;
				size_t n_block_size_dest = r_t_col.n_width * r_row_list_second[n_dest_row].n_height;
				double *p_value_dest = (*p_first_it).second;
				for(double *p_value_end = p_value_dest + n_block_size_dest;
				   p_value_dest != p_value_end; ++ p_value_dest)
					*p_value_dest = op(*p_value_dest, 0);
			}
			// don't forget to modify all the blocks in dest between last added and the end of the list
		}
		for(; j < m; ++ j) {
			const TColumn::TBlockEntry &r_t_block = r_t_col.block_list[j];
			size_t n_old_row, n_new_row;
			if((n_new_row = row_mapping[n_old_row = r_t_block.first]) == size_t(-1))
				return false; // this row was split, the blocks in it are now homeless

			double *p_value_src = r_t_block.second;
			size_t n_block_size = r_t_col.n_width * m_block_rows_list[n_old_row].n_height;
			// get block data

			_ASSERTE(r_t_dest_col.block_list.empty() || r_t_dest_col.block_list.back().first < n_new_row);
			// make sure the new block comes at the end of the row

			double *p_value = r_dest.p_Get_DenseStorage(n_block_size);
			double *p_value_dest = p_value;
			for(double *p_value_end = p_value_dest + n_block_size;
			   p_value_dest != p_value_end; ++ p_value_src, ++ p_value_dest)
				*p_value_dest = op(0, *p_value_src);  // t_odo - replace by op()
			// create a new block, initialize with values

			r_t_dest_col.block_list.push_back(TColumn::TBlockEntry(n_new_row, p_value));
			// add it to the list
		}
		// merge blocks
	}

	for(size_t n = r_column_list_second.size(); n_last_dest_col != n; ++ n_last_dest_col) {
		TColumn &r_t_dest_col = r_column_list_second[n_last_dest_col];
		for(size_t j = 0, m = r_t_dest_col.block_list.size(); j < m; ++ j) {
			const TColumn::TBlockEntry &r_t_block = r_t_dest_col.block_list[j];

			double *p_value_dest = r_t_block.second;
			size_t n_block_size_dest = r_t_dest_col.n_width * r_row_list_second[r_t_block.first].n_height;
			// get block data

			for(double *p_value_end = p_value_dest + n_block_size_dest;
			   p_value_dest != p_value_end; ++ p_value_dest)
				*p_value_dest = op(*p_value_dest, 0);
		}
	}
	// don't forget to modify all of the columns after the last modified one

	//r_dest.CheckIntegrity(true);
	// make sure that this operation didn't damage matrix integrity

	return true;
}

bool CUberBlockMatrix::ProductOf(const CUberBlockMatrix &r_A, const CUberBlockMatrix &r_B) // throw(std::bad_alloc)
{
	return r_A.MultiplyToWith(*this, r_B);
}

bool CUberBlockMatrix::ProductOf(const CUberBlockMatrix &r_A,
	const CUberBlockMatrix &r_B, bool b_upper_diag_only) // throw(std::bad_alloc)
{
	return r_A.MultiplyToWith(*this, r_B, b_upper_diag_only);
}

bool CUberBlockMatrix::MultiplyToWith(CUberBlockMatrix &r_dest, const CUberBlockMatrix &r_other) const // throw(std::bad_alloc)
{
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
	return MultiplyToWith_AccumLookup(r_dest, r_other, false);
#else // __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
#ifndef __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
	return MultiplyToWith_LogLookup(r_dest, r_other, false);
#else // !__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
	return MultiplyToWith_TransposeSort(r_dest, r_other, false);
#endif // !__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
}

bool CUberBlockMatrix::MultiplyToWith(CUberBlockMatrix &r_dest, const CUberBlockMatrix &r_other,
	bool b_upper_diag_only) const // throw(std::bad_alloc)
{
#ifdef __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
	return MultiplyToWith_AccumLookup(r_dest, r_other, b_upper_diag_only);
#else // __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
#ifndef __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
	return MultiplyToWith_LogLookup(r_dest, r_other, b_upper_diag_only);
#else // !__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
	return MultiplyToWith_TransposeSort(r_dest, r_other, b_upper_diag_only);
#endif // !__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
#endif // __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
}

template <const int MatrixRowsAtCompileTime>
bool CUberBlockMatrix::Cholesky_Dense() // throw(std::bad_alloc)
{
	CheckIntegrity(true);

	//Check_Block_Alignment();

	_ASSERTE(b_SymmetricLayout());
	// makes sure it has symmetric layout

	_ASSERTE(MatrixRowsAtCompileTime == Eigen::Dynamic || n_Row_Num() == MatrixRowsAtCompileTime);
	_ASSERTE(MatrixRowsAtCompileTime == Eigen::Dynamic || n_Column_Num() == MatrixRowsAtCompileTime);
	typedef Eigen::Matrix<double, MatrixRowsAtCompileTime, MatrixRowsAtCompileTime> TMatrixType;

	TMatrixType t_factor(n_Row_Num(), n_Column_Num()); // see what happens here with fixed-size matrices
	t_factor.setZero();
	// alloc dense matrix and clear it

	std::vector<size_t> frontline;
	frontline.resize(n_BlockColumn_Num());
	// alloc vector for frontline blocks (we can observe that cholesky leaves nonzero
	// values in columns above diagonal and under the top-most block of the original matrix)

	for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
		const TColumn &r_t_col = m_block_cols_list[i];
		if(r_t_col.block_list.empty()) {
			frontline[i] = i; // no block - frontline touches diagonal here
			continue;
		} else
			frontline[i] = r_t_col.block_list.front().first; // use the smallest row id as fronline
		// get frontline

		size_t n_x = r_t_col.n_cumulative_width_sum;
		size_t n_width = r_t_col.n_width;
		// dimensions of the block

		for(size_t i = 0, n_block_num = r_t_col.block_list.size(); i < n_block_num; ++ i) {
			size_t n_row = r_t_col.block_list[i].first;
			const double *p_data = r_t_col.block_list[i].second;

			size_t n_y = m_block_rows_list[n_row].n_cumulative_height_sum;
			size_t n_height = m_block_rows_list[n_row].n_height;
			// dimensions of the block

			for(size_t x = 0; x < n_width; ++ x) {
				for(size_t y = 0; y < n_height; ++ y) {
					double f_elem = p_data[x * n_height + y]; // t_odo - this is probably the other way around // nope, it's this way
					t_factor(n_y + y, n_x + x) = f_elem;
				}
			}
			// iterate through data, sparse fill ...
		}
	}
	// collect the frontline, convert the matrix to dense

	Eigen::LLT<TMatrixType, Eigen::Upper> cholesky(t_factor);
	if(cholesky.info() != Eigen::Success)
		return false; // probably Eigen::NumericalIssue - not pos def
	t_factor = cholesky.matrixU();
	// calculate upper cholesky by eigen

	m_data_pool.clear();
	// clears data pool, invalidates all the existing blocks (faster than
	// sorting the blocks - we already know the correct order)

	for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
		TColumn &r_t_col = m_block_cols_list[i];

		size_t n_x = r_t_col.n_cumulative_width_sum;
		size_t n_width = r_t_col.n_width;
		// dimensions of the block

		size_t n_min_row = frontline[i];
		size_t n_max_row = i + 1; // one past the last one
		// calculates min / max rows that will be nonzero

		if(r_t_col.block_list.capacity() < n_max_row - n_min_row)
			r_t_col.block_list.clear(); // avoid copying data in case it will be reallocated
		r_t_col.block_list.resize(n_max_row - n_min_row);
		// resize the list of blocks

		_TyBlockIter p_block_it = r_t_col.block_list.begin();
		for(size_t n_row = n_min_row; n_row < n_max_row; ++ n_row, ++ p_block_it) {
			TColumn::TBlockEntry &r_t_block = *p_block_it;
			// get block iterator

			size_t n_y = m_block_rows_list[n_row].n_cumulative_height_sum;
			size_t n_height = m_block_rows_list[n_row].n_height;
			// dimensions of the block

			r_t_block.first = n_row;
			double *p_data = r_t_block.second = p_Get_DenseStorage(n_width * n_height);
			// alloc the block

			for(size_t x = 0; x < n_width; ++ x) {
				for(size_t y = 0; y < n_height; ++ y)
					p_data[x * n_height + y] = t_factor(n_y + y, n_x + x); // note this could be faster through FBS and Eigen::Map
			}
			// get cholesky data back
		}
		// fill all the blocks with the data

#ifdef _DEBUG
		for(size_t n_row = 0; n_row < n_min_row; ++ n_row) {
			size_t n_y = m_block_rows_list[n_row].n_cumulative_height_sum;
			size_t n_height = m_block_rows_list[n_row].n_height;
			// dimensions of the block

			for(size_t x = 0; x < n_width; ++ x) {
				for(size_t y = 0; y < n_height; ++ y)
					_ASSERTE(fabs(t_factor(n_y + y, n_x + x)) < 1e-5f); // or directly == 0?
			}
		}
		for(size_t n_row = n_max_row, n_end = m_block_rows_list.size(); n_row < n_end; ++ n_row) {
			size_t n_y = m_block_rows_list[n_row].n_cumulative_height_sum;
			size_t n_height = m_block_rows_list[n_row].n_height;
			// dimensions of the block

			for(size_t x = 0; x < n_width; ++ x) {
				for(size_t y = 0; y < n_height; ++ y)
					_ASSERTE(fabs(t_factor(n_y + y, n_x + x)) < 1e-5f); // or directly == 0?
			}
		}
		// makes sure that all the other entries are really zero
#endif // _DEBUG
	}
	// gets data back from the dense matrix to this matrix

	//CheckIntegrity(true);

	return true;
}

inline bool CUberBlockMatrix::CholeskyOf(const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
{
	//r_lambda.CheckIntegrity(true); // inside Build_EliminationTree()
	//r_lambda.Check_Block_Alignment();

	const size_t n = r_lambda.m_block_cols_list.size();
	// get number of block columns

	std::vector<size_t> etree, ereach_stack, bitfield;
	bitfield.resize(n, 0);

	r_lambda.Build_EliminationTree(etree, ereach_stack); // use ereach stack as workspace
	_ASSERTE(ereach_stack.size() == n);
	// build an elimination tree

	return CholeskyOf(r_lambda, etree, ereach_stack, bitfield);
}

inline bool CUberBlockMatrix::CholeskyOf(const CUberBlockMatrix &r_lambda, size_t n_start_on_column) // throw(std::bad_alloc)
{
	//r_lambda.CheckIntegrity(true); // inside Build_EliminationTree()
	//r_lambda.Check_Block_Alignment();

	const size_t n = r_lambda.m_block_cols_list.size();
	// get number of block columns

	std::vector<size_t> etree, ereach_stack, bitfield;
	bitfield.resize(n, 0);

	r_lambda.Build_EliminationTree(etree, ereach_stack); // use ereach stack as workspace
	_ASSERTE(ereach_stack.size() == n);
	// build an elimination tree

	return CholeskyOf(r_lambda, etree, ereach_stack, bitfield, n_start_on_column);
}

template <bool b_upper_triangular, bool b_likely_upper_triangular,
	bool b_output_diagonal, bool b_output_full_diagonal, class CInt, class CInt1>
size_t CUberBlockMatrix::n_BlockStructure_SumWithSelfTranspose_ColumnLengths(CInt *__restrict p_column_lengths,
	size_t n_length_num, CInt1 *__restrict p_workspace, size_t n_workspace_size) const
{
	typedef typename blockmatrix_detail::CStaticAssert<!b_output_diagonal ||
		!b_output_full_diagonal>::MATRIX_MUTUALLY_EXCLUSIVE_OPTIONS_SELECTED CAssert0; // only one of those
	typedef typename blockmatrix_detail::CStaticAssert<!b_upper_triangular ||
		!b_likely_upper_triangular>::MATRIX_MUTUALLY_EXCLUSIVE_OPTIONS_SELECTED CAssert1; // only one of those
	// t_odo - compile time asserts

	_ASSERTE(!p_column_lengths || (void*)p_column_lengths != (void*)p_workspace);

	_ASSERTE(!b_upper_triangular || b_UpperBlockTriangular());
	// in case b_upper_triangular is set, then this matrix must be (otherwise set
	// b_likely_upper_triangular or nothing at all)

	CheckIntegrity();

	const size_t m = m_block_rows_list.size(), n = m_block_cols_list.size();

	_ASSERTE(n_length_num == std::max(m, n)); // must match
	_ASSERTE(b_upper_triangular || n_workspace_size >= size_t(n)); // must fit if not upper triangular (then it is not needed)

	const std::vector<TColumn> &__restrict r_block_cols_list = m_block_cols_list;

#ifdef _DEBUG
	std::fill(p_column_lengths, p_column_lengths + n_length_num, CInt(0xbaadf00d));
	std::fill(p_workspace, p_workspace + n_workspace_size, CInt1(0xbaadf00d));
	// fill the output and temp buffers with rubbish
#endif // _DEBUG

	CInt1 *__restrict p_transpose_col_off = (b_upper_triangular)? 0 : p_workspace; // unused if upper triangular / rename
	// those point to the individual column arrays

	size_t n_diag_nnz_num = 0, n_sym_nnz_num = 0;
	// the number of elements on the diagonal and (half) the number of elements
	// that have (structural) counterparts on the opposite side of the diagonal

	size_t nz = 0; // have to sum this up

	for(size_t i = 0; i < n; ++ i) {
		const std::vector<TColumn::TBlockEntry> &__restrict r_col = r_block_cols_list[i].block_list;
		nz += r_col.size();
		const _TyBlockConstIter p1 = r_col.begin(), p2 = r_col.end();
		_ASSERTE(CDebug::b_IsStrictlySortedSet(p1, p2, CCompareBlockRow())); // must be sorted

		_TyBlockConstIter p_diag; // todo - refactor variable names
		if(b_upper_triangular) { // compile-time-constant
			_ASSERTE(p1 <= p2);
			if(p1 != p2 && (*(p2 - 1)).first == i) {
				p_diag = p2 - 1;
				++ n_diag_nnz_num;
			} else
				p_diag = p2;
			_ASSERTE(p1 == p2 || p_diag == p2 || (*p_diag).first == i); // empty or not empty and points at the end or not empty and points before the end, at the diagonal element
			_ASSERTE(p1 == p2 || p_diag < p2 || (*(p_diag - 1)).first < i); // empty or not empty and points before the end or not empty and points at the end and the one before is above-diagonal
			// note that p_transpose_col_off[i] is intentionally not used, the workspace will not be needed
		} else {
			p_diag = (b_likely_upper_triangular && p1 != p2 && (*(p2 - 1)).first <= i)?
				(((*(p2 - 1)).first == i)? p2 - 1 : p2) : // it is one of the last two ones, depending whether the diagonal item is present or not
				std::lower_bound(p1, p2, i, CCompareBlockRow()); // find the diagonal // in case the matrix is strictly upper, then this is a slight waste of time
			_ASSERTE(!b_likely_upper_triangular || p_diag == std::lower_bound(p1, p2, i, CCompareBlockRow())); // make sure
			if(p_diag != p2 && (*p_diag).first == i) { // in case there is a diagonal element
				++ n_diag_nnz_num;
				p_transpose_col_off[i] = CInt1((p_diag - p1) + 1); // point to the first below-diagonal element
			} else {
				_ASSERTE(p_diag == p2 || (*p_diag).first > i); // otherwise it is the first below-diagonal element
				p_transpose_col_off[i] = CInt1(p_diag - p1);
			}
			_ASSERTE(p_transpose_col_off[i] >= 0 && p_transpose_col_off[i] <= SIZE_MAX); // make sure it is ok to cast to size_t
			_ASSERTE(size_t(p_transpose_col_off[i]) <= r_col.size()); // make sure it points to this column
			_ASSERTE(p_transpose_col_off[i] == r_col.size() ||
				r_col[p_transpose_col_off[i]].first > i); // make sure it points to a below-diagonal element
			_ASSERTE(r_col.empty() || size_t(p_transpose_col_off[i]) < r_col.size() ||
				r_col[p_transpose_col_off[i] - 1].first <= i); // make sure it is preceded by above-or-diagonal elements
		}
		// find the upper triangular portion of this column

		p_column_lengths[i] = CInt((b_output_full_diagonal)? (p_diag - p1) + 1 : // the number of above-diag elements + 1
			(b_output_diagonal)? // the number of elements in the upper part of the column, including the diagonal if present
				((b_upper_triangular)? (p_diag - p1) + ((p_diag != p2 && (*p_diag).first == i)? 1 : 0) : // simplified before, have to calculate it now
				p_transpose_col_off[i]) : // the above else branch already calculated it
			(p_diag - p1)); // the number of above-diag elements
		// number of (strictly) upper triangular items A(*, i)
		// in case the diagonal should be included in the column length, just use p_transpose_col_off[i]
		// the diagonal entry would be added here

		for(_TyBlockConstIter p_block_it = p1; p_block_it != p_diag; ++ p_block_it) {
			const size_t j = (*p_block_it).first;

			if(!b_upper_triangular) { // compile-time-constant
				_ASSERTE(j < n); // this array is only indexed by columns
				const std::vector<TColumn::TBlockEntry> &__restrict r_colj = r_block_cols_list[j].block_list;

				_TyBlockConstIter bj = r_colj.begin(),
					pj1 = bj + size_t(p_transpose_col_off[j]); // already initialized
				const _TyBlockConstIter pj2 = r_colj.end();
				_ASSERTE(pj1 <= pj2); // make sure it points to its column
				for(; pj1 != pj2; ++ pj1) { // in case the matrix is strictly upper, then this loop will not run
					const size_t ii = (*pj1).first;
					_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
					// even in case the diagonal should be included then ii can't equal j

					if(ii < i) { // A(ii, j) is only in the lower diagonal and not in the upper (assymetric element)
#ifdef _DEBUG
						_TyBlockConstIter p_assym_elem =
							std::lower_bound(r_block_cols_list[ii].block_list.begin(),
							r_block_cols_list[ii].block_list.end(), j, CCompareBlockRow());
						_ASSERTE(p_assym_elem == r_block_cols_list[ii].block_list.end() || (*p_assym_elem).first != j);
						// make sure A(j, ii) does not exist
#endif // _DEBUG
						++ p_column_lengths[ii]; // A(j, ii) in the upper triangle
						++ p_column_lengths[j]; // A(ii, j) in the lower triangle
					} else if(ii == i) { // A(ii, j) is a mirror of A(j, i), do not count
						_ASSERTE(std::lower_bound(p1, p_diag, j, CCompareBlockRow()) == p_block_it); // (A(j, ii))' ~ A(i, j) is this element actually
						++ n_sym_nnz_num; // count those entries
					} else
						break; // will do the rest of this transpose row later on
				}
				p_transpose_col_off[j] = CInt1(pj1 - bj); // remember where we stopped
				// ordered merge of transposed columns with the upper triangular entriex of this column
			}

			_ASSERTE(j < i);
			++ p_column_lengths[j]; // can increment, all the columns below i are already initialized
			// number of (strictly) lower triangular items A(i, *)
		}
		// loop over the upper elements and count them again (or rather count their transpose images)
		// making std::lower_bound() above saves checking for the value of *p_row
		// inside this loop - fewer comparisons are made
	}

	std::fill(p_column_lengths + n, p_column_lengths + n_length_num, CInt(0));
	// in case the matrix is square, need to zero part of the column length vector
	// the above loop does not write below n

	if(!b_upper_triangular) { // compile-time-constant
		for(size_t j = 0; j < n; ++ j) {
			_ASSERTE(j < n); // p_transpose_col_off array is only indexed by columns
			const std::vector<TColumn::TBlockEntry> &__restrict r_colj = r_block_cols_list[j].block_list;

			_ASSERTE(size_t(p_transpose_col_off[j]) <= r_colj.size()); // make sure it points to its column
			for(_TyBlockConstIter pj1 = r_colj.begin() +
			   size_t(p_transpose_col_off[j]), pj2 = r_colj.end(); pj1 != pj2; ++ pj1) {
				const size_t ii = (*pj1).first;
				_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
				// even in case the diagonal should be included then ii can't equal j

#ifdef _DEBUG
				if(ii < n) { // !!
					_TyBlockConstIter p_assym_elem =
						std::lower_bound(r_block_cols_list[ii].block_list.begin(),
						r_block_cols_list[ii].block_list.end(), j, CCompareBlockRow());
					_ASSERTE(p_assym_elem == r_block_cols_list[ii].block_list.end() || (*p_assym_elem).first != j);
					// make sure A(j, ii) does not exist
				}
#endif // _DEBUG

				++ p_column_lengths[ii]; // A(j, ii) in the upper triangle
				++ p_column_lengths[j]; // A(ii, j) in the lower triangle
			}
		}
		// handle entries in transpose rows which were previously not matched
	}

	if(b_output_full_diagonal && m != n) {
		for(size_t j = n; j < m; ++ j)
			++ p_column_lengths[j]; // the diagonal
	}
	// in case the matrix is rectangular and there are fewer columns than rows,
	// the full diagonals are not counted correctly

	_ASSERTE(nz >= n_diag_nnz_num && nz - n_diag_nnz_num >= n_sym_nnz_num); // make sure the below line does not overflow
	size_t n_nnz_num = (nz - n_diag_nnz_num - n_sym_nnz_num) * 2 +
		((b_output_diagonal)? n_diag_nnz_num : // in case the diagonal should be included, add n_diag_nnz_num back
		(b_output_full_diagonal)? std::max(m, n) : 0); // in case b_output_full_diagonal is set, add the length of the full diagonal back
	// calculate the number of nonzeros

	_ASSERTE(std::accumulate(p_column_lengths, p_column_lengths + n_length_num, size_t(0)) == n_nnz_num);
	// make sure that the sum of column lengths indeed gives nnz

	return n_nnz_num;
}

template <bool b_upper_triangular, bool b_likely_upper_triangular, bool b_output_diagonal,
	bool b_output_full_diagonal, bool b_need_sorted_items, class CInt, class CInt1, class CInt2>
void CUberBlockMatrix::BlockStructure_SumWithSelfTranspose(CInt *p_column_ptrs, size_t n_column_ptr_num,
	CInt *__restrict p_row_inds, size_t n_nnz_num, const CInt1 *__restrict p_column_lengths,
	size_t n_length_num, CInt2 *p_workspace, size_t n_workspace_size) const
{
	typedef typename blockmatrix_detail::CStaticAssert<!b_output_diagonal ||
		!b_output_full_diagonal>::MATRIX_MUTUALLY_EXCLUSIVE_OPTIONS_SELECTED CAssert0; // only one of those
	typedef typename blockmatrix_detail::CStaticAssert<!b_upper_triangular ||
		!b_likely_upper_triangular>::MATRIX_MUTUALLY_EXCLUSIVE_OPTIONS_SELECTED CAssert1; // only one of those
	// t_odo - compile time asserts

	_ASSERTE(!b_upper_triangular || b_UpperBlockTriangular());
	// in case b_upper_triangular is set, then this matrix must be (otherwise set
	// b_likely_upper_triangular or nothing at all)

	_ASSERTE(!p_column_lengths || (void*)p_column_lengths != (void*)p_workspace); // p_column_lengths and p_workspace may equal if they are both null (processing an empty matrix)
	_ASSERTE((void*)p_column_lengths != (void*)p_column_ptrs); // p_column_ptrs can't be null so they cant both be null
	_ASSERTE((void*)p_column_ptrs != (void*)p_workspace); // p_column_ptrs can't be null so they cant both be null

	CheckIntegrity();

	const size_t m = m_block_rows_list.size(), n = m_block_cols_list.size();

	_ASSERTE(n_length_num == std::max(m, n)); // must match
	_ASSERTE(n_column_ptr_num == std::max(m, n) + 1); // must match
	_ASSERTE(b_upper_triangular || n_workspace_size >= n); // must fit if not upper triangular (then it is not needed)

	enum {
		b_can_have_unsorted = !b_upper_triangular,
		b_handle_unsorted_items = b_can_have_unsorted && b_need_sorted_items
	};

	const std::vector<TColumn> &__restrict r_block_cols_list = m_block_cols_list;

#ifdef _DEBUG
	std::fill(p_row_inds, p_row_inds + n_nnz_num, CInt(0xbaadf00d));
	std::fill(p_column_ptrs, p_column_ptrs + n_column_ptr_num, CInt(0xbaadf00d));
	std::fill(p_workspace, p_workspace + n_workspace_size, CInt2(0xbaadf00d));
	// fill the output and temp buffers with rubbish
#endif // _DEBUG

	*p_column_ptrs = 0; // filled explicitly
	CInt *__restrict p_column_dest = p_column_ptrs + 1;
	size_t n_nnz_sum = 0;
	for(size_t j = 0; j < n_length_num; ++ j) { // not n!
		p_column_dest[j] = CInt(n_nnz_sum);
		n_nnz_sum += p_column_lengths[j];
	}
	_ASSERTE(n_nnz_sum == n_nnz_num); // must match
	// takes a cumsum of column lengths to generate the destination pointers

	CInt2 *__restrict p_transpose_col_off = (b_upper_triangular)? 0 : p_workspace; // unused if upper triangular / rename
	// those point to the Ai array

	size_t n_diag_nnz_num = 0, n_sym_nnz_num = 0;
	// the number of elements on the diagonal and (half) the number of elements
	// that have (structural) counterparts on the opposite side of the diagonal

	bool b_will_have_unsorted = false, b_filled_from_upper = false;

	for(size_t i = 0; i < n; ++ i) {
		const std::vector<TColumn::TBlockEntry> &__restrict r_col = r_block_cols_list[i].block_list;
		const _TyBlockConstIter p1 = r_col.begin(), p2 = r_col.end();
		_ASSERTE(CDebug::b_IsStrictlySortedSet(p1, p2, CCompareBlockRow())); // must be sorted

		_TyBlockConstIter p_diag; // todo - refactor variable names
		if(b_upper_triangular) { // compile-time-constant
			_ASSERTE(p1 <= p2);
			if(p1 != p2 && (*(p2 - 1)).first == i) {
				p_diag = p2 - 1;
				++ n_diag_nnz_num;
			} else
				p_diag = p2;
			_ASSERTE(p1 == p2 || p_diag == p2 || (*p_diag).first == i); // empty or not empty and points at the end or not empty and points before the end, at the diagonal element
			_ASSERTE(p1 == p2 || p_diag < p2 || (*(p_diag - 1)).first < i); // empty or not empty and points before the end or not empty and points at the end and the one before is above-diagonal
			// note that p_transpose_col_off[i] is intentionally not used, the workspace will not be needed
		} else {
			p_diag = (b_likely_upper_triangular && p1 != p2 && (*(p2 - 1)).first <= i)?
				(((*(p2 - 1)).first == i)? p2 - 1 : p2) : // it is one of the last two ones, depending whether the diagonal item is present or not
				std::lower_bound(p1, p2, i, CCompareBlockRow()); // find the diagonal // in case the matrix is strictly upper, then this is a slight waste of time
			_ASSERTE(!b_likely_upper_triangular || p_diag == std::lower_bound(p1, p2, i, CCompareBlockRow())); // make sure
			if(p_diag != p2 && (*p_diag).first == i) { // in case there is a diagonal element
				++ n_diag_nnz_num;
				p_transpose_col_off[i] = CInt2((p_diag - p1) + 1); // point to the first below-diagonal element
			} else {
				_ASSERTE(p_diag == p2 || (*p_diag).first > i); // otherwise it is the first below-diagonal element
				p_transpose_col_off[i] = CInt2(p_diag - p1);
			}
			_ASSERTE(p_transpose_col_off[i] >= 0 && p_transpose_col_off[i] <= SIZE_MAX); // make sure it is ok to cast to size_t
			_ASSERTE(size_t(p_transpose_col_off[i]) <= r_col.size()); // make sure it points to this column
			_ASSERTE(p_transpose_col_off[i] == r_col.size() ||
				r_col[p_transpose_col_off[i]].first > i); // make sure it points to a below-diagonal element
			_ASSERTE(r_col.empty() || size_t(p_transpose_col_off[i]) < r_col.size() ||
				r_col[p_transpose_col_off[i] - 1].first <= i); // make sure it is preceded by above-or-diagonal elements
		}
		// find the upper triangular portion of this column

		size_t n_add_nnz = CInt((b_output_full_diagonal)? (p_diag - p1) + 1 : // the number of above-diag elements + 1
			(b_output_diagonal)? // the number of elements in the upper part of the column, including the diagonal if present
				((b_upper_triangular)? (p_diag - p1) + ((p_diag != p2 && (*p_diag).first == i)? 1 : 0) : // simplified before, have to calculate it now
				p_transpose_col_off[i]) : // the above else branch already calculated it
			(p_diag - p1)); // the number of above-diag elements
		// number of (strictly) upper triangular items A(*, i)
		// in case the diagonal should be included in the column length, just use p_transpose_col_off[i]
		// the diagonal entry would be added here

		{
			CInt *__restrict p_next = p_row_inds + p_column_dest[i];
			for(_TyBlockConstIter p = p1, e = (b_output_diagonal)?
			   ((b_upper_triangular)? p1 + n_add_nnz : // either use n_add_nnz
			   p1 + p_transpose_col_off[i]) : // or reuse p_transpose_col_off if we have it
			   p_diag; p != e; ++ p, ++ p_next) // or use p_diag as the delimiter if we are not interested in the diagonal
				*p_next = CInt((*p).first);
			if(b_output_full_diagonal) {
				*p_next = CInt(i); // fill the diagonal entry
				_ASSERTE(p_next + 1 - (p_row_inds + p_column_dest[i]) == n_add_nnz); // make sure we filled what we promised
			} else
				_ASSERTE(p_next - (p_row_inds + p_column_dest[i]) == n_add_nnz); // make sure we filled what we promised
		}
		// the diagonal entry is added here

		p_column_dest[i] += CInt(n_add_nnz); // don't forget to shift the destination

		if(b_handle_unsorted_items) { // compile-time constant
			//if((!b_output_full_diagonal && !b_output_diagonal && n_add_nnz) || // certainly not diag
			//   (b_output_full_diagonal && n_add_nnz > 1) || // always one diag, if there is more than one then it is not diag
			//   (b_output_diagonal && (n_add_nnz > 1 || (n_add_nnz &&
			//   p_row_inds[p_column_dest[i] - 1] != i)))) // if not diag
			if(n_add_nnz) // even diagonal elems screw it up unfortunately
				b_filled_from_upper = true;
			// although this looks horrible, the diagonal flags are compile time constants
		}

		for(_TyBlockConstIter
		   p_block_it = p1; p_block_it != p_diag; ++ p_block_it) {
			const size_t j = (*p_block_it).first;
			_ASSERTE(j < i);

			// the below loop could also be simplified using std::lower_bound() but the numbers
			// of iterations are most likely very small (it just "catches up" with incrementing i)

			if(!b_upper_triangular) { // compile-time-constant
				_ASSERTE(j < n); // this array is only indexed by columns
				const std::vector<TColumn::TBlockEntry> &__restrict r_colj = r_block_cols_list[j].block_list;

				_TyBlockConstIter bj = r_colj.begin(),
					pj1 = bj + size_t(p_transpose_col_off[j]); // already initialized
				const _TyBlockConstIter pj2 = r_colj.end();
				_ASSERTE(pj1 <= pj2); // make sure it points to its column
				for(; pj1 != pj2; ++ pj1) { // in case the matrix is strictly upper, then this loop will not run
					const size_t ii = (*pj1).first;
					_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
					// even in case the diagonal should be included then ii can't equal j

					if(ii < i) { // A(ii, j) is only in the lower diagonal and not in the upper (assymetric element)
#ifdef _DEBUG
						_TyBlockConstIter p_assym_elem =
							std::lower_bound(r_block_cols_list[ii].block_list.begin(),
							r_block_cols_list[ii].block_list.end(), j, CCompareBlockRow());
						_ASSERTE(p_assym_elem == r_block_cols_list[ii].block_list.end() || (*p_assym_elem).first != j);
						// make sure A(j, ii) does not exist
#endif // _DEBUG
						if(b_handle_unsorted_items) // compile-time constant
							b_will_have_unsorted = true;

						//_ASSERTE(p_column_dest[j] == p_column_ptrs[j] || p_row_inds[p_column_dest[j] - 1] < ii);
						p_row_inds[p_column_dest[j]] = CInt(ii); // j < ii < i
						++ p_column_dest[j]; // A(ii, j) in the lower triangle
						//_ASSERTE(p_column_dest[ii] == p_column_ptrs[ii] || p_row_inds[p_column_dest[ii] - 1] < j);
						p_row_inds[p_column_dest[ii]] = CInt(j); // j < ii < i, j < i
						++ p_column_dest[ii]; // A(j, ii) in the upper triangle
					} else if(ii == i) { // A(ii, j) is a mirror of A(j, i), do not count
						_ASSERTE(std::lower_bound(p1, p_diag, j, CCompareBlockRow()) == p_block_it); // (A(j, ii))' ~ A(i, j) is this element actually
						++ n_sym_nnz_num; // count those entries
					} else
						break; // will do the rest of this transpose row later on
				}
				p_transpose_col_off[j] = CInt2(pj1 - bj); // remember where we stopped
				// ordered merge of transposed columns with the upper triangular entriex of this column
			}

			//_ASSERTE(p_column_dest[j] == p_column_ptrs[j] || p_row_inds[p_column_dest[j] - 1] < i);
			p_row_inds[p_column_dest[j]] = CInt(i); // scatter in the lower triangle, causes unordered accesses
			++ p_column_dest[j]; // can increment, all the columns below i are already initialized
			// number of (strictly) lower triangular items A(i, *)
			// need to add this *after* to maintain sorted order
			// t_odo - see if this makes any difference at all // yes, it makes difference e.g. on Lucifora/cell1
		}
		// loop over the upper elements and count them again (or rather count their transpose images)
		// making std::lower_bound() above saves checking for the value of *p_row
		// inside this loop - fewer comparisons are made
	}

	if(!b_upper_triangular) { // compile-time-constant
		for(size_t j = 0; j < n; ++ j) {
			_ASSERTE(j < n); // p_transpose_col_off array is only indexed by columns
			const std::vector<TColumn::TBlockEntry> &__restrict r_colj = r_block_cols_list[j].block_list;

			_ASSERTE(size_t(p_transpose_col_off[j]) <= r_colj.size()); // make sure it points to its column
			for(_TyBlockConstIter pj1 = r_colj.begin() +
			   size_t(p_transpose_col_off[j]), pj2 = r_colj.end(); pj1 != pj2; ++ pj1) {
				const size_t ii = (*pj1).first;
				_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
				// even in case the diagonal should be included then ii can't equal j

				if(b_handle_unsorted_items) // compile-time constant
					b_will_have_unsorted = true; // t_odo - see if the matrices that failed to predict on anselm are now predicted correctly or not

#ifdef _DEBUG
				if(ii < n) { // !!
					_TyBlockConstIter p_assym_elem =
						std::lower_bound(r_block_cols_list[ii].block_list.begin(),
						r_block_cols_list[ii].block_list.end(), j, CCompareBlockRow());
					_ASSERTE(p_assym_elem == r_block_cols_list[ii].block_list.end() || (*p_assym_elem).first != j);
					// make sure A(j, ii) does not exist
				}
#endif // _DEBUG

				//_ASSERTE(p_column_dest[j] == p_column_ptrs[j] || p_row_inds[p_column_dest[j] - 1] < ii);
				p_row_inds[p_column_dest[j]] = CInt(ii);
				++ p_column_dest[j]; // A(ii, j) in the lower triangle
				//_ASSERTE(p_column_dest[ii] == p_column_ptrs[ii] || p_row_inds[p_column_dest[ii] - 1] < j);
				p_row_inds[p_column_dest[ii]] = CInt(j);
				++ p_column_dest[ii]; // A(j, ii) in the upper triangle
			}
		}
		// handle entries in transpose rows which were previously not matched
	}

	if(b_handle_unsorted_items) { // compile-time constant
		if(!b_filled_from_upper)
			b_will_have_unsorted = false;
		// in case the matrix is strictly lower, then the entries are also filled nicely
	}

	if(b_output_full_diagonal && m > n) {
		for(size_t j = n; j < m; ++ j) {
			p_row_inds[p_column_dest[j]] = CInt(j);
			++ p_column_dest[j]; // the diagonal
		}
	}
	// in case the matrix is rectangular and there are fewer columns than rows,
	// the full diagonals are not counted correctly

#ifdef _DEBUG
	if(b_need_sorted_items) { // don't even check if sort is not needed
		bool b_had_unsorted = false;
		for(size_t j = 0; j < n_length_num; ++ j) {
			if(!CDebug::b_IsStrictlySortedSet(p_row_inds + p_column_ptrs[j],
			   p_row_inds + p_column_ptrs[j + 1])) {
				b_had_unsorted = true;
				break;
			}
		}
		_ASSERTE(!b_handle_unsorted_items || b_can_have_unsorted || !b_had_unsorted); // make sure that b_can_have_unsorted is determined correctly
		/*if(b_handle_unsorted_items && b_had_unsorted) {
			static int n_times = 0;
			++ n_times;
			fprintf(stderr, "warning: had unsorted entries (%d times already)\n", n_times); // at least see when it happens
		}*/
		_ASSERTE(!b_handle_unsorted_items || b_will_have_unsorted || !b_had_unsorted); // make sure that b_will_have_unsorted is determined correctly
		//if(b_handle_unsorted_items && !b_will_have_unsorted && b_had_unsorted)
		//	fprintf(stderr, "error: failed to predict unsorted entries\n");
		//if(b_will_have_unsorted && !b_had_unsorted)
		//	fprintf(stderr, "warning: UBM AAT predicted unsorted entries but had none\n");
		// it is not trivial to generate it sorted
		// AMD indeed does not require them sorted
	}
#endif // _DEBUG

	if(b_handle_unsorted_items) {
		if(b_will_have_unsorted) {
			for(size_t j = 0; j < n_length_num; ++ j)
				std::sort(p_row_inds + p_column_ptrs[j], p_row_inds + p_column_ptrs[j + 1]);
		}
	}
	// sort the columns as needed

#ifdef _DEBUG
	for(size_t j = 0, n_cumsum = 0; j < n_length_num; ++ j) {
		_ASSERTE(p_column_ptrs[j] == n_cumsum); // points at the beginning of the column
		n_cumsum += p_column_lengths[j];
		_ASSERTE(p_column_dest[j] == n_cumsum); // points at the end of the column

		_ASSERTE(!b_need_sorted_items || // unless we ask for unsorted
			CDebug::b_IsStrictlySortedSet(p_row_inds + p_column_ptrs[j],
			p_row_inds + p_column_ptrs[j + 1])); // make sure the result is not jumbled (should always be sorted)
	}
	_ASSERTE(p_column_ptrs[n_length_num] == n_nnz_num); // ...
	// make sure all the columns are filled, according to p_column_lengths
#endif // _DEBUG
}

#endif // !__UBER_BLOCK_MATRIX_INLINES_INCLUDED
