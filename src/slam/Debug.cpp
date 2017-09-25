/*
								+-----------------------------------+
								|                                   |
								| ***  Debugging functionality  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|             Debug.cpp             |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/Debug.cpp
 *	@brief basic debugging functionality
 *	@author -tHE SWINe-
 *	@date 2015-10-07
 */

#include "slam/Debug.h"
#include <csparse/cs.hpp>
#include "slam/Tga.h"
#include "slam/Parser.h"
#include "slam/BlockMatrix.h"
#include <math.h>
#include <cmath>

/*
 *								=== globals ===
 */

#if !defined(_WIN32) && !defined(_WIN64)

#include <stdio.h>
#include <ctype.h>
#include <string.h> // sprintf

bool _finite(double x)
{
#ifdef __FAST_MATH__
	// handle https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25975

	char p_s_str[256];
	size_t l = sprintf(p_s_str, "%f", x); // total number of characters written is returned, this count does not include the additional null
	//size_t l = strlen(p_s_str);
	std::transform(p_s_str, p_s_str + l, p_s_str, tolower);
	return !strstr(p_s_str, "nan") && !strstr(p_s_str, "ind") && !strstr(p_s_str, "inf"); // https://en.wikipedia.org/wiki/NaN#Display
#else // __FAST_MATH__
	//return isfinite(x); // math.h, did not work in some later versions of g++
	return std::isfinite(x); // cmath, should work
#endif // __FAST_MATH__
}

bool _isnan(double x)
{
#ifdef __FAST_MATH__
	// handle https://gcc.gnu.org/bugzilla/show_bug.cgi?id=25975

	char p_s_str[256];
	size_t l = sprintf(p_s_str, "%f", x); // total number of characters written is returned, this count does not include the additional null
	//size_t l = strlen(p_s_str);
	std::transform(p_s_str, p_s_str + l, p_s_str, tolower);
	return strstr(p_s_str, "nan") != 0 || strstr(p_s_str, "ind") != 0; // https://en.wikipedia.org/wiki/NaN#Display
#else // __FAST_MATH__
	//return isnan(x); // math.h, should work
	return std::isnan(x); // cmath, should work
#endif // __FAST_MATH__
}

#endif // !_WIN32 && !_WIN64

/*
 *								=== ~globals ===
 */

/*
 *								=== CDebug::CSparseMatrixShapedIterator ===
 */

CDebug::CSparseMatrixShapedIterator::CSparseMatrixShapedIterator(const cs *p_matrix)
	:m_p_matrix(p_matrix), m_n_rows(p_matrix->m), m_n_columns(p_matrix->n),
	m_n_column(0), m_n_row(0), m_n_row_pointer(p_matrix->p[0]),
	m_n_row_pointer_column_end(p_matrix->p[1]),
	m_n_row_pointer_end(p_matrix->p[p_matrix->n])
{
	if(m_n_row_pointer <= m_n_row_pointer_column_end) {
		size_t n_data_row = m_p_matrix->i[m_n_row_pointer];
		if(m_n_row_pointer == m_n_row_pointer_column_end || n_data_row != m_n_row) {
			size_t p = (m_n_column < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column] : m_n_row_pointer_end,
				e = (m_n_column + 1 < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : m_n_row_pointer_end;
			for(size_t i = p; i < e; ++ i) {
				if(size_t(m_p_matrix->i[i]) == m_n_row) {
					m_n_row_pointer = i;
					break;
				}
			}
			// e.g. cs_multiply() produces matrices where i is not sorted :(
		}
	}
}

CDebug::CSparseMatrixShapedIterator::CSparseMatrixShapedIterator(const cs *p_matrix, size_t n_rows, size_t n_cols)
	:m_p_matrix(p_matrix), m_n_rows(n_rows), m_n_columns(n_cols),
	m_n_column(0), m_n_row(0), m_n_row_pointer(p_matrix->p[0]),
	m_n_row_pointer_column_end(p_matrix->p[1]),
	m_n_row_pointer_end(p_matrix->p[p_matrix->n])
{
	if(m_n_row_pointer <= m_n_row_pointer_column_end) {
		size_t n_data_row = m_p_matrix->i[m_n_row_pointer];
		if(m_n_row_pointer == m_n_row_pointer_column_end || n_data_row != m_n_row) {
			size_t p = (m_n_column < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column] : m_n_row_pointer_end,
				e = (m_n_column + 1 < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : m_n_row_pointer_end;
			for(size_t i = p; i < e; ++ i) {
				if(size_t(m_p_matrix->i[i]) == m_n_row) {
					m_n_row_pointer = i;
					break;
				}
			}
			// e.g. cs_multiply() produces matrices where i is not sorted :(
		}
	}
}

double CDebug::CSparseMatrixShapedIterator::operator *() const
{
	_ASSERTE(m_n_column < m_n_columns && m_n_row < m_n_rows); // make sure the iterator is dereferencable
	if(m_n_row_pointer < m_n_row_pointer_column_end) {
		_ASSERTE(unsigned(m_p_matrix->i[m_n_row_pointer]) >= m_n_row);
		if(size_t(m_p_matrix->i[m_n_row_pointer]) == m_n_row)
			return (m_p_matrix->x)? m_p_matrix->x[m_n_row_pointer] : 1;
	}
	return 0; // no data (outside the matrix or being in "sparse area")
}

double CDebug::CSparseMatrixShapedIterator::f_Get(bool &r_b_value) const
{
	_ASSERTE(m_n_column < m_n_columns && m_n_row < m_n_rows); // make sure the iterator is dereferencable
	if(m_n_row_pointer < m_n_row_pointer_column_end) {
		_ASSERTE(unsigned(m_p_matrix->i[m_n_row_pointer]) >= m_n_row);
		if(size_t(m_p_matrix->i[m_n_row_pointer]) == m_n_row) {
			r_b_value = true;
			return (m_p_matrix->x)? m_p_matrix->x[m_n_row_pointer] : 1;
		}
	}
	r_b_value = false;
	return 0; // no data (outside the matrix or being in "sparse area")
}

void CDebug::CSparseMatrixShapedIterator::operator ++()
{
	_ASSERTE(m_n_column < m_n_columns);
	_ASSERTE(m_n_row < m_n_rows); // make sure we're not iterating too far

	if(m_n_column >= unsigned(m_p_matrix->n)) {
		if(++ m_n_row == m_n_rows) {
			m_n_row = 0;
			++ m_n_column;
		}
		// we are below the matrix data, just iterating zeroes
	} else if(m_n_row >= unsigned(m_p_matrix->m)) {
		if(++ m_n_row == m_n_rows) {
			m_n_row = 0;
			++ m_n_column;
			_ASSERTE(m_n_column <= unsigned(m_p_matrix->n));
			/*if(m_n_column <= m_p_matrix->n) */ { // always true // m_p_matrix->n is a valid index in p (sets m_n_row_pointer_offset equal to m_n_row_pointer_end)
				m_n_row_pointer = m_p_matrix->p[m_n_column];
				m_n_row_pointer_column_end = (m_n_column < unsigned(m_p_matrix->n))?
					m_p_matrix->p[m_n_column + 1] : m_n_row_pointer_end;
			}
		}
		// we are right of the matrix data, just iterating zeroes (the same code)
	} else {
		if(++ m_n_row == m_n_rows) {
			m_n_row = 0;
			++ m_n_column;
			_ASSERTE(m_n_column <= unsigned(m_p_matrix->n));
			/*if(m_n_column <= m_p_matrix->n) */ { // always true // m_p_matrix->n is a valid index in p (sets m_n_row_pointer_offset equal to m_n_row_pointer_end)
				m_n_row_pointer = m_p_matrix->p[m_n_column];
				m_n_row_pointer_column_end = (m_n_column < unsigned(m_p_matrix->n))?
					m_p_matrix->p[m_n_column + 1] : m_n_row_pointer_end;
			}
		}
		// shift to the next row / column

		size_t p = (m_n_column < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column] : m_n_row_pointer_end,
			e = (m_n_column + 1 < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : m_n_row_pointer_end;
		_ASSERTE(e >= p);
		_ASSERTE(m_n_row_pointer_column_end == e);
		_ASSERTE(m_n_row_pointer >= p && m_n_row_pointer <= e); // note that m_n_row_pointer == e is a valid state in case there is not enough data on the row
		// we are inside the matrix data; just need to find row in the CSC array

		if(m_n_row_pointer <= m_n_row_pointer_column_end) { // have to check if we hit the last nonzero row in the column
			size_t n_data_row = m_p_matrix->i[m_n_row_pointer];
			while(n_data_row < m_n_row && m_n_row_pointer < m_n_row_pointer_column_end)
				n_data_row = m_p_matrix->i[++ m_n_row_pointer];
			if(m_n_row_pointer == m_n_row_pointer_column_end || n_data_row != m_n_row) {
				for(size_t i = p; i < e; ++ i) {
					if(size_t(m_p_matrix->i[i]) == m_n_row) {
						m_n_row_pointer = i;
						break;
					}
				}
				// e.g. cs_multiply() produces matrices where i is not sorted :(
			}
		}
		// makes sure that m_n_row_pointer points to an element at the current or greater
	}

	_ASSERTE((m_n_column < m_n_columns && m_n_row < m_n_rows) ||
		(m_n_column == m_n_columns && !m_n_row)); // make sure we are still inside, or just outside
	if(m_n_column < unsigned(m_p_matrix->n)) {
#ifdef _DEBUG
		size_t p = (m_n_column < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column] : m_n_row_pointer_end,
			e = (m_n_column + 1 < unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : m_n_row_pointer_end;
#endif // _DEBUG
		_ASSERTE(m_n_row_pointer_column_end == e);
		if(m_n_row < unsigned(m_p_matrix->m)) {
			_ASSERTE(m_n_row_pointer == e ||
				m_n_row <= unsigned(m_p_matrix->i[m_n_row_pointer]));
			// either we are at the end of data, or m_n_row_pointer points at or after current row
		} else {
			_ASSERTE(m_n_row_pointer == e);
			// we are at the end of the row for sure
		}
	} else {
		_ASSERTE(m_n_row_pointer == m_n_row_pointer_end);
		_ASSERTE(m_n_row_pointer_column_end == m_n_row_pointer_end);
		// we are at the end
	}
	// iterator integrity check
}

/*
 *								=== ~CDebug::CSparseMatrixShapedIterator ===
 */

/*
 *								=== CDebug::CSparseMatrixIterator ===
 */

CDebug::CSparseMatrixIterator::CSparseMatrixIterator(const cs *p_matrix)
	:m_p_matrix(p_matrix), m_n_column(0), m_n_row(0), m_n_row_pointer(p_matrix->p[0]),
	m_n_row_pointer_column_end(p_matrix->p[1])
{
	if(m_n_row_pointer <= m_n_row_pointer_column_end) {
		size_t n_data_row = m_p_matrix->i[m_n_row_pointer];
		if(m_n_row_pointer == m_n_row_pointer_column_end || n_data_row != m_n_row) {
			size_t p = m_p_matrix->p[m_n_column], e = (m_n_column + 1 <= size_t(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : p;
			for(size_t i = p; i < e; ++ i) {
				if(size_t(m_p_matrix->i[i]) == m_n_row) {
					m_n_row_pointer = i;
					break;
				}
			}
			// e.g. cs_multiply() produces matrices where i is not sorted :(
		}
	}
}

double CDebug::CSparseMatrixIterator::operator *() const
{
	_ASSERTE(m_n_column < unsigned(m_p_matrix->n) && m_n_row < unsigned(m_p_matrix->m)); // make sure the iterator is dereferencable
	if(m_n_row_pointer < m_n_row_pointer_column_end) {
		_ASSERTE(unsigned(m_p_matrix->i[m_n_row_pointer]) >= m_n_row);
		if(size_t(m_p_matrix->i[m_n_row_pointer]) == m_n_row)
			return (m_p_matrix->x)? m_p_matrix->x[m_n_row_pointer] : 1;
	}
	return 0; // no data (being in "sparse area")
}

void CDebug::CSparseMatrixIterator::operator ++()
{
	_ASSERTE(m_n_column < unsigned(m_p_matrix->n));
	_ASSERTE(m_n_row < unsigned(m_p_matrix->m)); // make sure we're not iterating too far

	{
		if(++ m_n_row == size_t(m_p_matrix->m)) {
			m_n_row = 0;
			++ m_n_column;
			_ASSERTE(m_n_column <= unsigned(m_p_matrix->n));
			/*if(m_n_column <= m_p_matrix->n) */ { // always true // m_p_matrix->n is a valid index in p (sets m_n_row_pointer_offset equal to the ebd)
				m_n_row_pointer = m_p_matrix->p[m_n_column];
				m_n_row_pointer_column_end = (m_n_column < unsigned(m_p_matrix->n))?
					m_p_matrix->p[m_n_column + 1] : m_n_row_pointer; // m_n_row_pointer == p_matrix->p[p_matrix->n] by then
				_ASSERTE(m_n_column < unsigned(m_p_matrix->n) || m_n_row_pointer == m_p_matrix->p[m_p_matrix->n]); // just to make sure
			}
		}
		// shift to the next row / column

		_ASSERTE(m_n_column <= unsigned(m_p_matrix->n));
		size_t p = m_p_matrix->p[m_n_column], e = (m_n_column + 1 <= unsigned(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : p;
		_ASSERTE(e >= p);
		_ASSERTE(m_n_row_pointer_column_end == e);
		_ASSERTE(m_n_row_pointer >= p && m_n_row_pointer <= e); // note that m_n_row_pointer == e is a valid state in case there is not enough data on the row
		// we are inside the matrix data; just need to find row in the CSC array

		if(m_n_row_pointer <= m_n_row_pointer_column_end) { // have to check if we hit the last nonzero row in the column
			size_t n_data_row = m_p_matrix->i[m_n_row_pointer];
			while(n_data_row < m_n_row && m_n_row_pointer < m_n_row_pointer_column_end)
				n_data_row = m_p_matrix->i[++ m_n_row_pointer];
			if(m_n_row_pointer == m_n_row_pointer_column_end || n_data_row != m_n_row) {
				for(size_t i = p; i < e; ++ i) {
					if(size_t(m_p_matrix->i[i]) == m_n_row) {
						m_n_row_pointer = i;
						break;
					}
				}
				// e.g. cs_multiply() produces matrices where i is not sorted :(
			}
		}
		// makes sure that m_n_row_pointer points to an element at the current or greater
	}

	_ASSERTE((m_n_column < unsigned(m_p_matrix->n) && m_n_row < unsigned(m_p_matrix->m)) ||
		(m_n_column == m_p_matrix->n && !m_n_row)); // make sure we are still inside, or just outside
	if(m_n_column < unsigned(m_p_matrix->n)) {
		_ASSERTE(m_n_column <= unsigned(m_p_matrix->n));
#ifdef _DEBUG
		size_t p = m_p_matrix->p[m_n_column], e = (m_n_column + 1 <= size_t(m_p_matrix->n))? m_p_matrix->p[m_n_column + 1] : p;
#endif // _DEBUG
		_ASSERTE(m_n_row_pointer_column_end == e);
		if(m_n_row < unsigned(m_p_matrix->m)) {
			_ASSERTE(m_n_row_pointer == e ||
				m_n_row <= unsigned(m_p_matrix->i[m_n_row_pointer]));
			// either we are at the end of data, or m_n_row_pointer points at or after current row
		} else {
			_ASSERTE(m_n_row_pointer == e);
			// we are at the end of the row for sure
		}
	} else {
		_ASSERTE(m_n_row_pointer == m_n_row_pointer_column_end);
		_ASSERTE(m_n_row_pointer_column_end == m_p_matrix->p[m_p_matrix->n]);
		// we are at the end
	}
	// iterator integrity check
}

/*
 *								=== ~CDebug::CSparseMatrixIterator ===
 */

/*
 *								=== CDebug ===
 */

bool CDebug::Dump_SparseMatrix(const char *p_s_filename, const cs *A,
	const cs *A_prev /*= 0*/, int n_scalar_size /*= 5*/)
{
	_ASSERTE(CS_CSC(A));
	_ASSERTE(!A_prev || CS_CSC(A_prev));
	// the matrices need to be in compressed column form

	const uint32_t n_background = 0xffffffffU,
		n_nonzero = 0xff8080ffU,
		n_nonzero_new = 0xffff0000U,
		n_zero_new = 0xff00ff00U;
	// colors of the individual areas

	const uint32_t n_nonzero_border = 0x80000000U | ((n_nonzero >> 1) & 0x7f7f7f7fU),
		n_nonzero_new_border = 0x80000000U | ((n_nonzero_new >> 1) & 0x7f7f7f7fU),
		n_zero_new_border = 0x80000000U | ((n_zero_new >> 1) & 0x7f7f7f7fU);
	// colors of borders (just darker)

	size_t m = (A_prev)? std::max(A->m, A_prev->m) : A->m;
	size_t n = (A_prev)? std::max(A->n, A_prev->n) : A->n;

	if(m == SIZE_MAX || n == SIZE_MAX || m > INT_MAX || n > INT_MAX ||
	   m * (n_scalar_size - 1) + 1 > INT_MAX || n * (n_scalar_size - 1) + 1 > INT_MAX ||
	   uint64_t(m * (n_scalar_size - 1) + 1) * (m * (n_scalar_size - 1) + 1) > INT_MAX)
		return false;

	TBmp *p_bitmap;

	if(!(p_bitmap = TBmp::p_Alloc(int(n * (n_scalar_size - 1) + 1), int(m * (n_scalar_size - 1) + 1))))
		return false;
	p_bitmap->Clear(n_background);

	if(A_prev) {
		CSparseMatrixShapedIterator p_a_it(A, m, n);
		CSparseMatrixShapedIterator p_a_prev_it(A_prev, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it, ++ p_a_prev_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				_ASSERTE(n_row == p_a_prev_it.n_Row() && n_col == p_a_prev_it.n_Column());
				// make sure it iterates to the right position

				bool b_is;
				double f_value = p_a_it.f_Get(b_is);// = *p_a_it;
				double f_prev_value = *p_a_prev_it;
				// read the value

				double f_value_thr = (fabs(f_value) < 1e-10)? 0 : f_value;
				double f_prev_value_thr = (fabs(f_prev_value) < 1e-10)? 0 : f_prev_value;
				// threshold

				if(b_is || f_value != 0 || f_prev_value != 0) {
					uint32_t n_fill_color = (f_value_thr != 0 && f_prev_value_thr != 0)? n_nonzero :
										    (f_value_thr != 0 && f_prev_value_thr == 0)? n_nonzero_new :
										    (f_value != 0 && f_prev_value != 0)? n_nonzero : n_zero_new;
					uint32_t n_line_color = (f_value_thr != 0 && f_prev_value_thr != 0)? n_nonzero_border :
										    (f_value_thr != 0 && f_prev_value_thr == 0)? n_nonzero_new_border :
											(f_value != 0 && f_prev_value != 0)? n_nonzero_border : n_zero_new_border;

					size_t n_x = n_col * (n_scalar_size - 1);
					size_t n_y = n_row * (n_scalar_size - 1);
					for(int dy = 1; dy < n_scalar_size - 1; ++ dy)
						for(int dx = 1; dx < n_scalar_size - 1; ++ dx)
							p_bitmap->PutPixel(int(n_x + dx), int(n_y + dy), n_fill_color);
					p_bitmap->DrawLine_SP(float(n_x), float(n_y), float(n_x + n_scalar_size - 1), float(n_y), n_line_color);
					p_bitmap->DrawLine_SP(float(n_x), float(n_y + n_scalar_size - 1), float(n_x + n_scalar_size - 1),
						float(n_y + n_scalar_size - 1), n_line_color);
					p_bitmap->DrawLine_SP(float(n_x), float(n_y), float(n_x), float(n_y + n_scalar_size - 1), n_line_color);
					p_bitmap->DrawLine_SP(float(n_x + n_scalar_size - 1), float(n_y), float(n_x + n_scalar_size - 1),
						float(n_y + n_scalar_size - 1), n_line_color);
				}
				// draw a square into the bitmap
			}
		}
	} else {
		CSparseMatrixShapedIterator p_a_it(A, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				// make sure it iterates to the right position

				double f_value = *p_a_it;
				// read the value

				if(f_value != 0) {
					size_t n_x = n_col * (n_scalar_size - 1);
					size_t n_y = n_row * (n_scalar_size - 1);
					for(int dy = 1; dy < n_scalar_size - 1; ++ dy)
						for(int dx = 1; dx < n_scalar_size - 1; ++ dx)
							p_bitmap->PutPixel(int(n_x + dx), int(n_y + dy), n_nonzero);
					p_bitmap->DrawLine_SP(float(n_x), float(n_y), float(n_x + n_scalar_size - 1), float(n_y), n_nonzero_border);
					p_bitmap->DrawLine_SP(float(n_x), float(n_y + n_scalar_size - 1), float(n_x + n_scalar_size - 1),
						float(n_y + n_scalar_size - 1), n_nonzero_border);
					p_bitmap->DrawLine_SP(float(n_x), float(n_y), float(n_x), float(n_y + n_scalar_size - 1), n_nonzero_border);
					p_bitmap->DrawLine_SP(float(n_x + n_scalar_size - 1), float(n_y), float(n_x + n_scalar_size - 1),
						float(n_y + n_scalar_size - 1), n_nonzero_border);
				}
				// draw a square into the bitmap
			}
		}
	}
	// iterate the matrix, draw small tiles

	bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_bitmap, false);

	p_bitmap->Delete();

	/*for(int j = 0; j < n; ++ j) {
		//printf("    col %d\n", j);
		int y = 0;
		if(j < A->n) {
			for(int p = A->p[j]; p < A->p[j + 1]; ++ p, ++ y) {
				int n_cur_y = A->i[p];
				for(; y < n_cur_y; ++ y)
					;//printf(" %f, ", .0f); // zero entries
				double f_value = (A->x)? A->x[p] : 1; // nonzero entry
			}
			for(; y < m; ++ y)
				;//printf(" %f%s", .0f, (x + 1 == m)? "\n" : ", "); // zero entries
		}
	}*/
	// t_odo - make this an iterator anyway

	// t_odo - rasterize the matrix (A_prev is used to mark difference areas)
	// todo - implement libPNG image writing

	return b_result;
}

static inline void PutPixel_Max(TBmp *p_bitmap, int x, int y, uint32_t n_color)
{
	uint32_t *p_buffer = p_bitmap->p_buffer;
	const int n_width = p_bitmap->n_width;
	const int n_height = p_bitmap->n_height;
	if(x >= 0 && x < n_width && y >= 0 && y < n_height) {
		uint32_t &r_n_pixel = p_buffer[int(x) + n_width * int(y)];
		if((r_n_pixel & 0xffffff) < (n_color & 0xffffff))
			r_n_pixel = n_color;
	}
}

#define BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
// optimize bitmap write routines to access the bitmaps row-wise for better cache reuse

#define BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
// parallelize Dump_SparseMatrix_Subsample() and Dump_SparseMatrix_Subsample_AA() (only if BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS is defined)

bool CDebug::Dump_SparseMatrix_Subsample(const char *p_s_filename, const cs *A,
	const cs *A_prev /*= 0*/, size_t n_max_bitmap_size /*= 640*/, bool b_symmetric /*= false*/)
{
	_ASSERTE(CS_CSC(A));
	_ASSERTE(!A_prev || CS_CSC(A_prev));
	// the matrices need to be in compressed column form

	const uint32_t n_background = 0xffffffffU,
		n_nonzero = 0xff8080ffU,
		n_nonzero_new = 0xffff0000U,
		n_zero_new = 0xff00ff00U;
	// colors of the individual areas

	size_t m = (A_prev)? std::max(A->m, A_prev->m) : A->m;
	size_t n = (A_prev)? std::max(A->n, A_prev->n) : A->n;

	size_t mb = m, nb = n;
	double f_scale = double(n_max_bitmap_size) / std::max(m, n);
	if(f_scale < 1) {
		mb = std::max(size_t(1), size_t(m * f_scale));
		nb = std::max(size_t(1), size_t(n * f_scale));
    } else
		f_scale = 1;
	// scale the bitmap down if needed

	if(mb == SIZE_MAX || nb == SIZE_MAX || mb > INT_MAX ||
	   nb > INT_MAX || uint64_t(mb) * nb > INT_MAX)
		return false;

	TBmp *p_bitmap;

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(mb), int(nb)))) // transposed
		return false;
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(nb), int(mb))))
		return false;
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

	if(A_prev) {
		p_bitmap->Clear(0/*n_background*/);

		CSparseMatrixShapedIterator p_a_it(A, m, n);
		CSparseMatrixShapedIterator p_a_prev_it(A_prev, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it, ++ p_a_prev_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				_ASSERTE(n_row == p_a_prev_it.n_Row() && n_col == p_a_prev_it.n_Column());
				// make sure it iterates to the right position

				bool b_is;
				double f_value = p_a_it.f_Get(b_is);// = *p_a_it;
				double f_prev_value = *p_a_prev_it;
				// read the value

				double f_value_thr = (fabs(f_value) < 1e-10)? 0 : f_value;
				double f_prev_value_thr = (fabs(f_prev_value) < 1e-10)? 0 : f_prev_value;
				// threshold

				if(b_is || f_value != 0 || f_prev_value != 0) {
					uint32_t n_fill_color = (f_value_thr != 0 && f_prev_value_thr != 0)? 1/*n_nonzero*/ :
										    (f_value_thr != 0 && f_prev_value_thr == 0)? 3/*n_nonzero_new*/ :
										    (f_value != 0 && f_prev_value != 0)? 1/*n_nonzero*/ : 2/*n_zero_new*/;

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					size_t n_y = size_t(n_col * f_scale);
					size_t n_x = size_t(n_row * f_scale); // transposed
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					size_t n_x = size_t(n_col * f_scale);
					size_t n_y = size_t(n_row * f_scale);
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					PutPixel_Max(p_bitmap, /*->PutPixel(*/int(n_x), int(n_y), n_fill_color);
					//if(b_symmetric && n_x != n_y)
					//	PutPixel_Max(p_bitmap, /*->PutPixel(*/int(n_y), int(n_x), n_fill_color);
					// use PutPixel_Max() to get some deterministic precedences of the colors
				}
				// draw a dot into the bitmap
			}
		}

		if(b_symmetric) {
			uint32_t *p_buff = p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					uint32_t *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					uint32_t *p_lo = p_buff + (y + x * w);
					*p_up = std::max(*p_up, *p_lo); // choose the "color" with higher priority
					*p_lo = *p_up; // duplicate the result to the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices

		uint32_t p_color_table[] = {n_background, n_nonzero, n_zero_new, n_nonzero_new};
		for(uint32_t *p_buffer = p_bitmap->p_buffer, *p_end =
		   p_bitmap->p_buffer + nb * mb; p_buffer != p_end; ++ p_buffer) {
			_ASSERTE(*p_buffer < sizeof(p_color_table) / sizeof(p_color_table[0]));
			*p_buffer = p_color_table[*p_buffer];
		}
		// look the color table up
	} else {
		p_bitmap->Clear(n_background);

		/*CSparseMatrixShapedIterator p_a_it(A, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				// make sure it iterates to the right position*/
#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
#ifdef BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		if(n < INT_MAX && A->p[n] >= 10000) {
			const int _n = int(n);
			#pragma omp parallel for schedule(dynamic, 1)
			for(int n_col = 0; n_col < _n; ++ n_col) {
				for(size_t n_p0 = A->p[n_col], n_p1 = A->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
					size_t n_row = A->i[n_p0];
					double f_value = (A->x)? A->x[n_p0] : 1;
					// read the value

					if(f_value != 0)
						p_bitmap->PutPixel(int(n_row * f_scale), int(n_col * f_scale), n_nonzero);
					// draw a dot into the bitmap
				}
			}
		} else
#endif // BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		{
			for(size_t n_col = 0; n_col < n; ++ n_col) {
				for(size_t n_p0 = A->p[n_col], n_p1 = A->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
					size_t n_row = A->i[n_p0];
					const double f_one = 1, *p_a_it = (A->x)? A->x + n_p0 : &f_one;
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
			{
				CSparseMatrixIterator p_a_it(A); // can use the simple iterator here
				for(; p_a_it.n_Column() < n; ++ p_a_it) {
					size_t n_col = p_a_it.n_Column();
					size_t n_row = p_a_it.n_Row();
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

					double f_value = *p_a_it;
					// read the value

					if(f_value != 0) {
#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
						size_t n_y = size_t(n_col * f_scale);
						size_t n_x = size_t(n_row * f_scale); // transposed
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
						size_t n_x = size_t(n_col * f_scale);
						size_t n_y = size_t(n_row * f_scale);
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
						p_bitmap->PutPixel(int(n_x), int(n_y), n_nonzero);
						//if(b_symmetric && n_x != n_y)
						//	p_bitmap->PutPixel(int(n_y), int(n_x), n_nonzero);
					}
					// draw a square into the bitmap
				}
			}
		}

		if(b_symmetric) {
			uint32_t *p_buff = p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					uint32_t *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					uint32_t *p_lo = p_buff + (y + x * w);
					*p_up = std::min(*p_up, *p_lo); // choose the darker color
					*p_lo = *p_up; // duplicate the result to the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices
	}
	// iterate the matrix, draw small tiles

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if((m != n || !b_symmetric) && !p_bitmap->Transpose()) {
		p_bitmap->Delete();
		return false;
	}
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

	bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_bitmap, false);

	p_bitmap->Delete();

	return b_result;
}

static inline void AlphaBlend_RGBA_HQ(uint32_t &r_n_dest, uint32_t n_src, float f_alpha)
{
	int n_dest_a = (r_n_dest >> 24) & 0xff;
	int n_dest_r = (r_n_dest >> 16) & 0xff;
	int n_dest_g = (r_n_dest >> 8) & 0xff;
	int n_dest_b = (r_n_dest) & 0xff;
	int n_src_a = (n_src >> 24) & 0xff;
	int n_src_r = (n_src >> 16) & 0xff;
	int n_src_g = (n_src >> 8) & 0xff;
	int n_src_b = (n_src) & 0xff;
	n_dest_a = std::max(0, std::min(255, int(n_dest_a + f_alpha * (n_src_a - n_dest_a) + .5f)));
	n_dest_r = std::max(0, std::min(255, int(n_dest_r + f_alpha * (n_src_r - n_dest_r) + .5f)));
	n_dest_g = std::max(0, std::min(255, int(n_dest_g + f_alpha * (n_src_g - n_dest_g) + .5f)));
	n_dest_b = std::max(0, std::min(255, int(n_dest_b + f_alpha * (n_src_b - n_dest_b) + .5f)));
	r_n_dest = (n_dest_a << 24) | (n_dest_r << 16) | (n_dest_g << 8) | n_dest_b;
}

static inline void PutPixel_AA_RGBA_HQ(TBmp *p_bitmap, float f_x, float f_y, uint32_t n_color)
{
	uint32_t *p_buffer = p_bitmap->p_buffer;
	const int n_width = p_bitmap->n_width;
	const int n_height = p_bitmap->n_height;
	int x = int(floor(f_x)), y = int(floor(f_y));
	float f_frac_x = f_x - x;
	float f_frac_y = f_y - y;
	const float f_alpha = ((n_color >> 24) & 0xff) / 255.0f; // alpha of the incoming pixel
	float f_weight_00 = (f_alpha * (1 - f_frac_x) * (1 - f_frac_y));
	float f_weight_10 = (f_alpha * f_frac_x * (1 - f_frac_y));
	float f_weight_01 = (f_alpha * (1 - f_frac_x) * f_frac_y);
	float f_weight_11 = (f_alpha * f_frac_x * f_frac_y);

	if(x >= 0 && x < n_width && y >= 0 && y < n_height)
		AlphaBlend_RGBA_HQ(p_buffer[x + n_width * y], n_color, f_weight_00);
	if(x + 1 >= 0 && x + 1 < n_width && y >= 0 && y < n_height)
		AlphaBlend_RGBA_HQ(p_buffer[(x + 1) + n_width * y], n_color, f_weight_10);
	if(x >= 0 && x < n_width && y + 1 >= 0 && y + 1 < n_height)
		AlphaBlend_RGBA_HQ(p_buffer[x + n_width * (y + 1)], n_color, f_weight_01);
	if(x + 1 >= 0 && x + 1 < n_width && y + 1 >= 0 && y + 1 < n_height)
		AlphaBlend_RGBA_HQ(p_buffer[(x + 1) + n_width * (y + 1)], n_color, f_weight_11);
}

static inline void AlphaBlend_Float(float &r_f_dest, float f_src, float f_alpha)
{
	//r_f_dest = (r_f_dest * (1 - f_alpha)) + f_src * f_alpha;
	//r_f_dest = r_f_dest * 1 + f_src * f_alpha - r_f_dest * f_alpha;
	//r_f_dest = r_f_dest + (f_src - r_f_dest) * f_alpha;
	r_f_dest = r_f_dest + f_alpha * (f_src - r_f_dest);
}

static inline void PutPixel_AA_Float(TBmp *p_bitmap, float f_x, float f_y) // this is likely quite slow
{
	_ASSERTE(sizeof(float) == sizeof(uint32_t));
	float *p_buffer = (float*)p_bitmap->p_buffer;

	const int n_width = p_bitmap->n_width;
	const int n_height = p_bitmap->n_height;
	int x = int(floor(f_x)), y = int(floor(f_y));
	float f_frac_x = f_x - x;
	float f_frac_y = f_y - y;
	const float f_alpha = 1; // alpha of the incoming pixel
	float f_weight_00 = (f_alpha * (1 - f_frac_x) * (1 - f_frac_y));
	float f_weight_10 = (f_alpha * f_frac_x * (1 - f_frac_y));
	float f_weight_01 = (f_alpha * (1 - f_frac_x) * f_frac_y);
	float f_weight_11 = (f_alpha * f_frac_x * f_frac_y);
	if(x >= 0 && x < n_width && y >= 0 && y < n_height)
		AlphaBlend_Float(p_buffer[x + n_width * y], 1, f_weight_00);
	if(x + 1 >= 0 && x + 1 < n_width && y >= 0 && y < n_height)
		AlphaBlend_Float(p_buffer[(x + 1) + n_width * y], 1, f_weight_10);
	if(x >= 0 && x < n_width && y + 1 >= 0 && y + 1 < n_height)
		AlphaBlend_Float(p_buffer[x + n_width * (y + 1)], 1, f_weight_01);
	if(x + 1 >= 0 && x + 1 < n_width && y + 1 >= 0 && y + 1 < n_height)
		AlphaBlend_Float(p_buffer[(x + 1) + n_width * (y + 1)], 1, f_weight_11);
}

bool CDebug::Dump_SparseMatrix_Subsample_AA(const char *p_s_filename, const cs *A,
	const cs *A_prev /*= 0*/, size_t n_max_bitmap_size /*= 640*/, bool b_symmetric /*= false*/)
{
	_ASSERTE(CS_CSC(A));
	_ASSERTE(!A_prev || CS_CSC(A_prev));
	// the matrices need to be in compressed column form

	const uint32_t n_background = 0x00ffffffU,
		n_nonzero = 0xff8080ffU,
		n_nonzero_new = 0xffff0000U,
		n_zero_new = 0xff00ff00U;
	// colors of the individual areas

	size_t m = (A_prev)? std::max(A->m, A_prev->m) : A->m;
	size_t n = (A_prev)? std::max(A->n, A_prev->n) : A->n;

	size_t mb = m, nb = n;
	double f_scale = double(n_max_bitmap_size) / std::max(m, n);
	if(f_scale < 1) {
		mb = std::max(size_t(1), size_t(m * f_scale));
		nb = std::max(size_t(1), size_t(n * f_scale));
    } else
		f_scale = 1;
	// scale the bitmap down if needed

	if(mb == SIZE_MAX || nb == SIZE_MAX || mb > INT_MAX ||
	   nb > INT_MAX || uint64_t(mb) * nb > INT_MAX)
		return false;

	TBmp *p_bitmap;

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(mb), int(nb)))) // transposed
		return false;
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if(!(p_bitmap = TBmp::p_Alloc(int(nb), int(mb))))
		return false;
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

	// t_odo - dont use p_bitmap->PutPixel_AA(), the division of 0-255 by 256 diminishes
	//        the color, yielding black pixels in cases of extreme overdraw. use a float
	//        buffer or a fraction buffer instead

	if(A_prev) {
		p_bitmap->Clear(n_background);

		CSparseMatrixShapedIterator p_a_it(A, m, n);
		CSparseMatrixShapedIterator p_a_prev_it(A_prev, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it, ++ p_a_prev_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				_ASSERTE(n_row == p_a_prev_it.n_Row() && n_col == p_a_prev_it.n_Column());
				// make sure it iterates to the right position

				bool b_is;
				double f_value = p_a_it.f_Get(b_is);// = *p_a_it;
				double f_prev_value = *p_a_prev_it;
				// read the value

				double f_value_thr = (fabs(f_value) < 1e-10)? 0 : f_value;
				double f_prev_value_thr = (fabs(f_prev_value) < 1e-10)? 0 : f_prev_value;
				// threshold

				if(b_is || f_value != 0 || f_prev_value != 0) {
					uint32_t n_fill_color = (f_value_thr != 0 && f_prev_value_thr != 0)? n_nonzero :
										    (f_value_thr != 0 && f_prev_value_thr == 0)? n_nonzero_new :
										    (f_value != 0 && f_prev_value != 0)? n_nonzero : n_zero_new;

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					float f_y = float(n_col * f_scale);
					float f_x = float(n_row * f_scale); // transposed
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					float f_x = float(n_col * f_scale);
					float f_y = float(n_row * f_scale);
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
					PutPixel_AA_RGBA_HQ(p_bitmap, /*->PutPixel_AA(*/f_x, f_y, n_fill_color);
					//if(b_symmetric && f_x != f_y)
					//	PutPixel_AA_RGBA_HQ(p_bitmap, /*->PutPixel_AA(*/f_y, f_x, n_fill_color); // t_odo - float rgba? some more elaborate reweighting?
				}
				// draw a dot into the bitmap
			}
		}

		if(b_symmetric) {
			uint32_t *p_buff = p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					uint32_t *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					uint32_t *p_lo = p_buff + (y + x * w);
					AlphaBlend_RGBA_HQ(*p_up, *p_lo, ((*p_lo >> 24) & 0xff) / 255.0f); // blend the two colors
					*p_lo = *p_up; // duplicate the result in the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices
	} else {
		_ASSERTE(sizeof(float) == sizeof(uint32_t));
		std::fill((float*)p_bitmap->p_buffer, ((float*)p_bitmap->p_buffer) + nb * mb, .0f);
		// use as a float buffer

		/*CSparseMatrixShapedIterator p_a_it(A, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				// make sure it iterates to the right position*/
#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
#ifdef BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		if(n > 16 && n < INT_MAX && A->p[n] >= 10000) {
			size_t n_half_stripe_num = nb / 8 - 1; // want four-pixel stripes with at least four pixel clearance (the blend will overflow up to two pixels - one on the left, one on the right from each stripe), want the last one to be wider than the others
			_ASSERTE(n_half_stripe_num < INT_MAX);
			const int _n_half_stripe_num = int(n_half_stripe_num);
			for(int n_pass = 0; n_pass < 2; ++ n_pass) { // draw even stripes in one pass, odd stripes in the next, this guarantees that two threads never blend over each other's area
				#pragma omp parallel for schedule(dynamic, 1)
				for(int n_stripe = 0; n_stripe < _n_half_stripe_num; ++ n_stripe) { // each thread processes a single stripe
					size_t n_stripe_b = (n / n_half_stripe_num) * n_stripe;
					size_t n_stripe_e = (n_stripe + 1 == _n_half_stripe_num)? n : n_stripe_b + n / n_half_stripe_num;
					_ASSERTE(n_stripe_e >= n_stripe_b + 8);
					if(n_pass)
						n_stripe_b = (n_stripe_b + n_stripe_e) / 2;
					else
						n_stripe_e = (n_stripe_b + n_stripe_e) / 2;
					_ASSERTE(n_stripe_e >= n_stripe_b + 2);
					// begin and end of a stripe

					for(size_t n_col = n_stripe_b; n_col < n_stripe_e; ++ n_col) {
						for(size_t n_p0 = A->p[n_col], n_p1 = A->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
							size_t n_row = A->i[n_p0];
							double f_value = (A->x)? A->x[n_p0] : 1;
							// read the value

							if(f_value != 0)
								PutPixel_AA_Float(p_bitmap, float(n_row * f_scale), float(n_col * f_scale));
							// draw a dot into the bitmap
						}
					}
				}
				// implicit thread synchronization here
			}
		} else
#endif // BITMAP_SUBSAMPLE_DUMP_PARALLELIZE
		{
			for(size_t n_col = 0; n_col < n; ++ n_col) {
				for(size_t n_p0 = A->p[n_col], n_p1 = A->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
					size_t n_row = A->i[n_p0];
					const double f_one = 1, *p_a_it = (A->x)? A->x + n_p0 : &f_one;
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
			{
				CSparseMatrixIterator p_a_it(A); // can use the simple iterator here
				for(; p_a_it.n_Column() < n; ++ p_a_it) {
					size_t n_col = p_a_it.n_Column();
					size_t n_row = p_a_it.n_Row();
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

					double f_value = *p_a_it;
					// read the value

					if(f_value != 0) {
#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
						float f_y = float(n_col * f_scale);
						float f_x = float(n_row * f_scale); // transposed
#else // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
						float f_x = float(n_col * f_scale);
						float f_y = float(n_row * f_scale);
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
						PutPixel_AA_Float(p_bitmap, /*->PutPixel_AA(*/f_x, f_y/*, n_nonzero*/);
						//if(b_symmetric && f_x != f_y)
						//	PutPixel_AA_Float(p_bitmap, /*->PutPixel_AA(*/f_y, f_x/*, n_nonzero*/);
					}
					// draw a square into the bitmap
				}
			}
		}

		// alpha = 0
		// alpha = 0 + a1 * (1 - 0) = a1
		// alpha = a1 + a2 * (1 - a1) = a1 + a2 - a1a2
		//
		// alpha = 0
		// alpha = 0 + a2 * (1 - 0) = a2
		// alpha = a2 + a1 * (1 - a2) = a2 + a1 - a1a2
		// the order of blending does not make a difference

		if(b_symmetric) {
			float *p_buff = (float*)p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					float *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					float *p_lo = p_buff + (y + x * w);
					AlphaBlend_Float(*p_up, 1, *p_lo); // blend the two alphas
					*p_lo = *p_up; // duplicate the result in the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices

		for(float *p_buffer = (float*)p_bitmap->p_buffer, *p_end =
		   p_buffer + nb * mb; p_buffer != p_end; ++ p_buffer) {
			float f_alpha = *p_buffer;
			int n_alpha = std::min(255, std::max(0, int(255 * f_alpha)));
			*(uint32_t*)p_buffer = TBmp::n_Modulate(n_background, 255 - n_alpha) +
				TBmp::n_Modulate(n_nonzero, n_alpha); // integer blend should be enough now, only once per pixel
		}
		// convert float buffer to RGBA
	}
	// iterate the matrix, draw small tiles

#ifdef BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS
	if((m != n || !b_symmetric) && !p_bitmap->Transpose()) {
		p_bitmap->Delete();
		return false;
	}
#endif // BITMAP_SUBSAMPLE_DUMP_ROW_MAJOR_ACCESS

	bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_bitmap, false);

	p_bitmap->Delete();

	return b_result;
}

void CDebug::Print_SparseMatrix(const cs *A, const char *p_s_label /*= 0*/)
{
	if(p_s_label)
		printf("matrix \'%s\' = ", p_s_label);
	csi m, n, nzmax, /*nz,*/ *Ap, *Ai;
	double *Ax;
	if(!A) {
		printf("(null)\n");
		return;
	}
	cs *Ac = 0;
	if(CS_TRIPLET(A) && !(Ac = cs_compress(A))) {
		printf("[not enough memory]\n");
		return;
	}
	cs *At;
	if(!(At = cs_transpose((Ac)? Ac : A, 1))) {
		if(Ac)
			cs_spfree(Ac);
		printf("[not enough memory]\n");
		return;
	}
	m = At->m; n = At->n; Ap = At->p; Ai = At->i; Ax = At->x;
	nzmax = At->nzmax; //nz = At->nz; // unused

	_ASSERTE(CS_CSC(At));

	{
		_ASSERTE(n <= INT_MAX);
		_ASSERTE(m <= INT_MAX);
		_ASSERTE(nzmax <= INT_MAX);
		_ASSERTE(Ap[n] <= INT_MAX);
		printf("%d-by-%d, nzmax: %d nnz: %d, 1-norm: %g\n", int(n), int(m), int(nzmax),
				int(Ap[n]), cs_norm(A));
		for(csi j = 0; j < n; ++ j) {
			printf("    row %d\n", int(j));
			int x = 0;
			for(csi p = Ap[j]; p < Ap[j + 1]; ++ p, ++ x) {
				csi n_cur_x = Ai[p];
				for(; x < n_cur_x; ++ x)
					printf(" %f, ", .0f);
				printf("%s%f%s", (Ax && Ax[p] < 0)? "" : " ", (Ax)? Ax[p] : 1, (n_cur_x + 1 == m)? "\n" : ", ");
			}
			for(; x < m; ++ x)
				printf(" %f%s", .0f, (x + 1 == m)? "\n" : ", ");
		}
	}

	cs_spfree(At);
	if(Ac)
		cs_spfree(Ac);
}

void CDebug::Print_SparseMatrix_in_MatlabFormat(const cs *A, const char *p_s_label /*= 0*/)
{
	if(p_s_label)
		printf("matrix \'%s\' = ", p_s_label);
	csi m, n, /*nzmax, nz,*/ *Ap, *Ai;
	double *Ax;
	if(!A) {
		printf("(null)\n");
		return;
	}
	cs *Ac = 0;
	if(CS_TRIPLET(A) && !(Ac = cs_compress(A))) {
		printf("[not enough memory]\n");
		return;
	}
	cs *At;
	if(!(At = cs_transpose((Ac)? Ac : A, 1))) {
		if(Ac)
			cs_spfree(Ac);
		printf("[not enough memory]\n");
		return;
	}
	m = At->m; n = At->n; Ap = At->p; Ai = At->i; Ax = At->x;
	//nzmax = At->nzmax; nz = At->nz;

	_ASSERTE(CS_CSC(At));

	{
		printf("[");
		for(csi j = 0; j < n; ++ j) {
			csi x = 0;
			for(csi p = Ap[j]; p < Ap[j + 1]; ++ p, ++ x) {
				csi n_cur_x = Ai[p];
				for(; x < n_cur_x; ++ x)
					printf("%f ", .0f);
				printf("%f%s", (Ax)? Ax[p] : 1, (n_cur_x + 1 == m)? ((j + 1 == n)? "" : "; ") : " ");
			}
			for(; x < m; ++ x)
				printf("%f%s", .0f, (x + 1 == m)? ((j + 1 == n)? "" : "; ") : " ");
		}
		printf("]\n");
	}

	cs_spfree(At);
	if(Ac)
		cs_spfree(Ac);
}

bool CDebug::Print_SparseMatrix_in_MatlabFormat(FILE *p_fw,
	const cs *A, const char *p_s_prefix /*= 0*/, const char *p_s_suffix /*= ";\n"*/)
{
	if(p_s_prefix)
		fprintf(p_fw, "%s", p_s_prefix);

	{
		fprintf(p_fw, "[");
		CSparseMatrixShapedIterator it(A); // @todo - replace this by a simple iterator as soon as it's tested
		for(size_t i = 0, m = A->m; i < m; ++ i) { // iterate rows in the outer loop
			for(size_t j = 0, n = A->n; j < n; ++ j, ++ it) { // iterate columns in the inner loop
				double f_value = *it;
				fprintf(p_fw, " %f", f_value);
			}
			if(i + 1 != m)
				fprintf(p_fw, "; ");
		}
		fprintf(p_fw, "]");
	}

	// "to define a matrix, you can treat it like a column of row vectors"
	/*
	 *	>> A = [ 1 2 3; 3 4 5; 6 7 8]
	 *
	 *	A =
	 *
	 *		 1     2     3
	 *		 3     4     5
	 *		 6     7     8
	 */

	if(p_s_suffix)
		fprintf(p_fw, "%s", p_s_suffix);

	return true;
}

bool CDebug::Print_SparseMatrix_in_MatlabFormat2(FILE *p_fw, const cs *A,
	const char *p_s_label, const char *p_s_prefix /*= 0*/, const char *p_s_suffix /*= 0*/)
{
	if(p_s_prefix)
		fprintf(p_fw, "%s", p_s_prefix);

	fprintf(p_fw, "%s = sparse(" PRIdiff ", " PRIdiff ");\n", p_s_label, A->m, A->n);
	for(csi i = 0; i < A->n; ++ i) {
		for(csi p = A->p[i], e = A->p[i + 1]; p < e; ++ p) {
			csi j = A->i[p];
			double x = (A->x)? A->x[p] : 1;
			if(x != 0) { // hard comparison here, try to avoid zero lines introduced by conversion from block matrices
				fprintf(p_fw, (fabs(x) > 1)? "%s(" PRIdiff ", " PRIdiff ") = %f;\n" :
					"%s(" PRIdiff ", " PRIdiff ") = %g;\n", p_s_label, j + 1, i + 1, x);
			}
		}
	}

	// to make sparse unit matrix, one writes:
	/*
	 *	>> A = sparse(2, 2);
	 *	>> A(1, 1) = 1
	 *	>> A(2, 2) = 1
	 *
	 *	A =
	 *		 (1, 1)     1
	 *		 (2, 2)     1
	 */

	if(p_s_suffix)
		fprintf(p_fw, "%s", p_s_suffix);

	return true;
}

void CDebug::Print_DenseVector(const double *b, size_t n_vector_length, const char *p_s_label /*= 0*/)
{
	//_ASSERTE(n_vector_length > 0); // doesn't work for empties

	double f_min_abs = (n_vector_length)? fabs(b[0]) : 0;
	for(size_t i = 1; i < n_vector_length; ++ i)
		f_min_abs = (fabs(b[i]) > 0 && (!f_min_abs || f_min_abs > fabs(b[i])))? fabs(b[i]) : f_min_abs;
	int n_log10 = (f_min_abs > 0)? int(ceil(log(f_min_abs) / log(10.0))) : 0;
	// calculate log10 to display the smallest nonzero value as 0 to 1 number

	if(n_log10 < -1)
		n_log10 += 2;
	else if(n_log10 < 0)
		++ n_log10;
	// it's ok to have two first places 0 (in matlab it is, apparently)

	double f_scale = pow(10.0, -n_log10);
	// calculatze scale

	if(p_s_label)
		printf("%s = ", p_s_label);
	if(n_vector_length) {
		printf("vec(" PRIsize ") = 1e%+d * [%.4f", n_vector_length, n_log10, f_scale * b[0]);
		for(size_t i = 1; i < n_vector_length; ++ i)
			printf(", %.4f", f_scale * b[i]);
		printf("]\n");
	} else
		printf("[ ]\n");
	// print with scale
}

void CDebug::Print_DenseVector_in_MatlabFormat(FILE *p_fw, const double *b,
	size_t n_vector_length, const char *p_s_prefix /*= 0*/, const char *p_s_suffix /*= ";\n"*/)
{
	//_ASSERTE(n_vector_length > 0); // doesn't work for empties

	double f_min_abs = (n_vector_length)? fabs(b[0]) : 0;
	for(size_t i = 1; i < n_vector_length; ++ i)
		f_min_abs = (fabs(b[i]) > 0 && (!f_min_abs || f_min_abs > fabs(b[i])))? fabs(b[i]) : f_min_abs;
	int n_log10 = (f_min_abs > 0)? int(ceil(log(f_min_abs) / log(10.0))) : 0;
	// calculate log10 to display the smallest nonzero value as 0 to 1 number

	if(n_log10 < -1)
		n_log10 += 2;
	else if(n_log10 < 0)
		++ n_log10;
	// it's ok to have two first places 0 (in matlab it is, apparently)

	double f_scale = pow(10.0, -n_log10);
	// calculatze scale

	if(p_s_prefix)
		fprintf(p_fw, "%s", p_s_prefix);
	if(n_vector_length) {
		if(fabs(f_scale * b[0]) > 1)
			fprintf(p_fw, "1e%+d * [%.15f", n_log10, f_scale * b[0]);
		else
			fprintf(p_fw, "1e%+d * [%.15g", n_log10, f_scale * b[0]);
		for(size_t i = 1; i < n_vector_length; ++ i)
			fprintf(p_fw, (fabs(f_scale * b[i]) > 1)? " %.15f" : " %.15g", f_scale * b[i]);
		fprintf(p_fw, "]");
	} else
		fprintf(p_fw, "[ ]");
	if(p_s_suffix)
		fprintf(p_fw, "%s", p_s_suffix);
	// print with scale
}

#if (defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64) || \
    defined(_M_I86) || defined(_M_IX86) || defined(__X86__) || defined(_X86_) || defined(__IA32__) || defined(__i386)) && \
	1 // in case the below include is not found or _mm_clflush() or _mm_mfence() are undefined, switch to 0. this will only affect some debugging / performance profiling functionality

#include <emmintrin.h>

void CDebug::Evict_Buffer(const void *p_begin, size_t n_size)
{
	if(!p_begin)
		return;

	for(intptr_t n_addr = intptr_t(p_begin), n_end =
	   intptr_t(p_begin) + n_size; n_addr != n_end; ++ n_addr) {
		_mm_clflush((const void*)n_addr);
		// flush the corresponding cache line
	}

	_mm_mfence();
	// memory fence; force all operations to complete before going on
}

#else // (x86 || x64) && 1

//#pragma message "warning: emmintrin.h likely not present, evict semantics will not be available"

void CDebug::Evict_Buffer(const void *UNUSED(p_begin), size_t UNUSED(n_size))
{
	// we're not on x86, can't use _mm_*() intrinsics
	// this is needed if building e.g. for ARM
}

#endif // (x86 || x64) && 1

void CDebug::Evict(const cs *p_mat)
{
	if(CS_TRIPLET(p_mat))
		Evict_Buffer(p_mat->p, sizeof(csi) * p_mat->nzmax);
	else
		Evict_Buffer(p_mat->p, sizeof(csi) * (p_mat->n + 1));
	Evict_Buffer(p_mat->i, sizeof(csi) * p_mat->nzmax);
	Evict_Buffer(p_mat->x, sizeof(double) * p_mat->nzmax);
	// evict the internal buffers

	Evict_Buffer(p_mat, sizeof(cs));
	// evict the structure
}

void CDebug::Evict(const CUberBlockMatrix &r_mat)
{
	CUBM_EvictUtil<blockmatrix_detail::CUberBlockMatrix_Base>::EvictMembers(r_mat);
	// evict the internal vectors

	Evict_Buffer(&r_mat, sizeof(r_mat));
	// evict the structure
}

void CDebug::Swamp_Cache(size_t n_buffer_size /*= 100 * 1048576*/) // throw(std::bad_alloc)
{
	std::vector<uint8_t> buffer(n_buffer_size);
	for(size_t i = 0; i < n_buffer_size; ++ i)
		buffer[i] = uint8_t(n_SetBit_Num(i));
	// alloc a large buffer and fill it using nontrivial
	// ops, so that cache would not be bypassed by DMA as
	// e.g. for memset or memcpy
}

/*
 *								=== ~CDebug ===
 */

/*
 *								=== CSparseMatrixMemInfo ===
 */

uint64_t CSparseMatrixMemInfo::n_Allocation_Size(const cs *p_matrix)
{
	if(!p_matrix)
		return 0;
	if(p_matrix->nz <= 0) {
		return sizeof(cs) + (p_matrix->n + 1) * sizeof(csi) + p_matrix->p[p_matrix->n] *
			((p_matrix->x)? sizeof(double) + sizeof(csi) : sizeof(csi));
	} else {
		return sizeof(cs) + p_matrix->nzmax * ((p_matrix->x)? sizeof(double) +
			2 * sizeof(csi) : 2 * sizeof(csi));
	}
}

bool CSparseMatrixMemInfo::Peek_MatrixMarket_Header(const char *p_s_filename,
	CSparseMatrixMemInfo::TMMHeader &r_t_header)
{
	bool b_symmetric = false, b_binary = false;
	bool b_had_specifier = false;
	bool b_had_header = false;
	size_t n_rows, n_cols, n_nnz = -1, n_read_nnz = 0, n_full_nnz = 0;

	FILE *p_fr;
	if(!(p_fr = fopen(p_s_filename, "r")))
		return false;
	// open the matrix market file

	try {
		std::string s_line;
		while(!feof(p_fr)) {
			if(!CParserBase::ReadLine(s_line, p_fr)) {
				fclose(p_fr);
				return false;
			}
			// read a single line

			size_t n_pos;
			if(!b_had_specifier && (n_pos = s_line.find("%%MatrixMarket")) != std::string::npos) {
				s_line.erase(0, n_pos + strlen("%%MatrixMarket"));
				// get rid of header

				b_binary = s_line.find("pattern") != std::string::npos;
				if(s_line.find("matrix") == std::string::npos ||
				   s_line.find("coordinate") == std::string::npos ||
				   (s_line.find("real") == std::string::npos &&
				   s_line.find("integer") == std::string::npos &&
				   !b_binary)) // integer matrices are not real, but have values and are loadable
					return false;
				// must be matrix coordinate real

				if(s_line.find("general") != std::string::npos)
					b_symmetric = false;
				else if(s_line.find("symmetric") != std::string::npos)
					b_symmetric = true;
				else {
					b_symmetric = false;
					/*//fclose(p_fr);
					return false;*/ // or assume general
				}
				// either general or symmetric

				b_had_specifier = true;
				continue;
			}
			if((n_pos = s_line.find('%')) != std::string::npos)
				s_line.erase(n_pos);
			CParserBase::TrimSpace(s_line);
			if(s_line.empty())
				continue;
			// trim comments, skip empty lines

			if(!b_had_header) {
				if(!b_had_specifier) {
					fclose(p_fr);
					return false;
				}
				// specifier must come before header

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				if(sscanf_s(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
				   &n_rows, &n_cols, &n_nnz) != 3) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
				   &n_rows, &n_cols, &n_nnz) != 3) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					fclose(p_fr);
					return false;
				}
				// read header

				if(n_rows <= SIZE_MAX / std::max(size_t(1), n_cols) && n_nnz > n_rows * n_cols) {
					fclose(p_fr);
					return false;
				}
				// sanity check (may also fail on big matrices)

				b_had_header = true;
				break;
			}
		}
		if(!b_had_header) {
			fclose(p_fr);
			return false;
		}
		if(b_symmetric) {
			_ASSERTE(b_had_header && b_had_specifier);
			while(!feof(p_fr)) { // only elements follow
				if(!CParserBase::ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					return false;
				}
				// read a single line

				CParserBase::TrimSpace(s_line);
				if(s_line.empty() || s_line[0] == '%') // only handles comments at the beginning of the line; comments at the end will simply be ignored 
					continue;
				// trim comments, skip empty lines

				double f_value;
				size_t n_rowi, n_coli;
				do {
					const char *b = s_line.c_str();
					_ASSERTE(*b && !isspace(uint8_t(*b))); // not empty and not space
					for(n_rowi = 0; *b && isdigit(uint8_t(*b)); ++ b) // ignores overflows
						n_rowi = 10 * n_rowi + *b - '0';
					// parse the first number

					if(!*b || !isspace(uint8_t(*b))) {
						fclose(p_fr);
						return false;
					}
					++ b; // skip the first space
					while(*b && isspace(uint8_t(*b)))
						++ b;
					if(!*b || !isdigit(uint8_t(*b))) {
						fclose(p_fr);
						return false;
					}
					// skip space to the second number

					for(n_coli = 0; *b && isdigit(uint8_t(*b)); ++ b) // ignores overflows
						n_coli = 10 * n_coli + *b - '0';
					// parse the second number

					if(!*b) {
						f_value = 1; // a binary matrix?
						break;
					}
					if(!isspace(uint8_t(*b))) {
						fclose(p_fr);
						return false; // bad character
					}
					++ b; // skip the first space
					while(*b && isspace(uint8_t(*b)))
						++ b;
					_ASSERTE(*b); // there must be something since the string sure does not end with space (called TrimSpace() above)
					// skip space to the value

					f_value = 1;//atof(b); // don't care about the values here
				} while(0);
				// read a triplet

				-- n_rowi;
				-- n_coli;
				// indices are 1-based, i.e. a(1,1) is the first element

				if(n_rowi >= n_rows || n_coli >= n_cols || n_read_nnz >= n_nnz) {
					fclose(p_fr);
					return false;
				}
				// make sure it figures

				_ASSERTE(b_symmetric);
				n_full_nnz += (n_rowi != n_coli)? 2 : 1;
				++ n_read_nnz;
				// append the nonzero
			}
			// read the file line by line

			if(ferror(p_fr) || !b_had_header || n_read_nnz != n_nnz) {
				fclose(p_fr);
				return false;
			}
			// make sure no i/o errors occurred, and that the entire matrix was read
		}
		// in case the matrix is symmetric, need to read the indices to see which ones are at the
		// diagonal; does not actually store them, just goes through them quickly; the floats are
		// not parsed

		fclose(p_fr);
	} catch(std::bad_alloc&) {
		fclose(p_fr);
		return false;
	}

	r_t_header.b_binary = b_binary;
	r_t_header.b_symmetric = b_symmetric;
	r_t_header.n_nnz = n_nnz;
	r_t_header.n_rows = n_rows;
	r_t_header.n_cols = n_cols;

	if(!b_symmetric)
		r_t_header.n_full_nnz = n_nnz;
	else
		r_t_header.n_full_nnz = n_full_nnz;

	return true;
}

uint64_t CSparseMatrixMemInfo::n_Peek_MatrixMarket_SizeInMemory(const char *p_s_filename)
{
	TMMHeader t_header;
	if(!Peek_MatrixMarket_Header(p_s_filename, t_header))
		return uint64_t(-1);

	cs spmat = {0, 0, 0, 0, 0, 0, 0}; // put all to avoid -Wmissing-field-initializers
	return t_header.n_AllocationSize_CSC(sizeof(*spmat.p), sizeof(*spmat.x));
}

/*
 *								=== ~CSparseMatrixMemInfo ===
 */
