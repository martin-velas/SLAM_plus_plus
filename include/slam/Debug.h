/*
								+-----------------------------------+
								|                                   |
								| ***  Debugging functionality  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|              Debug.h              |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __DEBUGGING_FUNCTIONALITY_INCLUDED
#define __DEBUGGING_FUNCTIONALITY_INCLUDED

/**
 *	@file include/slam/Debug.h
 *	@brief basic debugging functionality
 *	@author -tHE SWINe-
 *	@date 2012-04-26
 *
 *	@date 2012-08-22
 *
 *	Fixed CSparseMatrixShapedIterator and CSparseMatrixIterator, there was a slight misconception
 *	of CSparse format, the i array is mostly in sorted order, but may not be. This caused some of
 *	matrix elements to be skipped from iteration.
 *
 */

#include <stdio.h>
#ifndef _ASSERTE
#include <assert.h>
/**
 *	@brief basic debug assertion macro
 *	@param[in] x is the condition that is supposed to hold
 */
#define _ASSERTE(x) assert(x)
#endif // !_ASSERTE

#ifndef _USE_MATH_DEFINES
/**
 *	@def _USE_MATH_DEFINES
 *	@brief enables math defines such as M_PI in MSVC
 */
#define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES

#include "slam/Integer.h" // PRIsize
#include <math.h>
#include <float.h>
#include <stdexcept> // runtime_error
#include <algorithm> // min, max
#include <vector> // required by the new clang

#if !defined(_WIN32) && !defined(_WIN64)

/**
 *	@brief determines if an input value is a finite number
 *	@param[in] x is the input value
 *	@return Returns true if the input value is a finite number (not inifinity or nan), otherwise returns false.
 *	@note This is only compiled on Linux where VS runtime library function _finite() is not available.
 */
bool _finite(double x);

/**
 *	@brief determines if an input value is not a number
 *	@param[in] x is the input value
 *	@return Returns true if the input value is not a number, otherwise returns false.
 *	@note This is only compiled on Linux where VS runtime library function _isnan() is not available.
 */
bool _isnan(double x);

#endif // !_WIN32 && !_WIN64
// we are using some advanced math functions to catch evil numbers

struct cs_sparse; // forward declaration
typedef cs_sparse cs; // forward declaration

class CUberBlockMatrix; // forward declaration

/*namespace std {

template <class _Ty, class _Alloc>
class vector; // forward declaration

}*/ // abolished by the new clang

namespace fap_detail {

template <class _Ty, const int n_page_size_elems, const int n_memory_alignment>
class fap_base; // forward declaration

}

namespace Eigen {

template <class Derived>
class MatrixBase; // forward declaration

}

/**
 *	@brief minimalist progress indicator which works in console
 */
class CMiniProgressIndicator {
protected:
	unsigned int m_n_state; /**< @brief indicator state (freely overflowing) */

public:
	/**
	 *	@brief default constructor; only needed to avoid uninitialized variable warnings
	 */
	CMiniProgressIndicator()
		:m_n_state(0)
	{}

	/**
	 *	@brief function operator; reports that progress is being made (prints a spinning bar to stdout)
	 */
	void operator ()()
	{
		printf("\b%c", "/-\\|"[(++ m_n_state) % 4]);
		// spin, you silly devil!
	}
};

/**
 *	@brief implements some basic debugging functionality (printing stuff to stdout, basically)
 */
class CDebug {
public:
	/**
	 *	@brief shaped sparse matrix iterator
	 */
	class CSparseMatrixShapedIterator {
	protected:
		const cs *m_p_matrix; /**< @brief the matrix in question */
		size_t m_n_rows; /**< @brief number of rows of shape to iterate through */
		size_t m_n_columns; /**< @brief number of columns of shape to iterate through */
		size_t m_n_column; /**< @brief current column */
		size_t m_n_row; /**< @brief current row */
		size_t m_n_row_pointer; /**< @brief current row pointer offset (index in m_p_matrix->p) */
		size_t m_n_row_pointer_column_end; /**< @brief maximum value m_n_row_pointer can take at the current column */
		size_t m_n_row_pointer_end; /**< @brief maximum value m_n_row_pointer can take */

	public:
		/**
		 *	@brief constructor
		 *	@param[in] p_matrix is a sparse matrix to iterate over
		 */
		CSparseMatrixShapedIterator(const cs *p_matrix);

		/**
		 *	@brief constructor
		 *	@param[in] p_matrix is a sparse matrix to iterate over
		 *	@param[in] n_rows is override for the number of matrix rows
		 *	@param[in] n_cols is override for the number of matrix columns
		 */
		CSparseMatrixShapedIterator(const cs *p_matrix, size_t n_rows, size_t n_cols);

		/**
		 *	@brief gets current row
		 *	@return Returns current row this iterator points to.
		 */
		inline size_t n_Row() const
		{
			return m_n_row;
		}

		/**
		 *	@brief gets current column
		 *	@return Returns current column this iterator points to.
		 */
		inline size_t n_Column() const
		{
			return m_n_column;
		}

		/**
		 *	@brief dereferences the iterator
		 *	@return Returns value under the iterator or 0 in case there
		 *		is no value associated with the current row / column (the matrix is sparse).
		 */
		double operator *() const;

		/**
		 *	@brief dereferences the iterator
		 *	@param[out] r_b_value is set in case there is a value associated
		 *		with the current row / column, otherwise it is cleared
		 *	@return Returns value under the iterator or 0 in case there
		 *		is no value associated with the current row / column (the matrix is sparse).
		 */
		double f_Get(bool &r_b_value) const;

		/**
		 *	@brief increments the iterator (to the next position, regardless of matrix sparsity)
		 */
		void operator ++();
	};

	/**
	 *	@brief simple sparse matrix iterator
	 */
	class CSparseMatrixIterator {
	protected:
		const cs *m_p_matrix; /**< @brief the matrix in question */
		size_t m_n_column; /**< @brief current column */
		size_t m_n_row; /**< @brief current row */
		size_t m_n_row_pointer; /**< @brief current row pointer offset (index in m_p_matrix->p) */
		size_t m_n_row_pointer_column_end; /**< @brief maximum value m_n_row_pointer can take at the current column */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_matrix is a sparse matrix to iterate over
		 */
		CSparseMatrixIterator(const cs *p_matrix);

		/**
		 *	@copydoc CSparseMatrixShapedIterator::n_Row()
		 */
		inline size_t n_Row() const
		{
			return m_n_row;
		}

		/**
		 *	@copydoc CSparseMatrixShapedIterator::n_Column()
		 */
		inline size_t n_Column() const
		{
			return m_n_column;
		}

		/**
		 *	@copydoc CSparseMatrixShapedIterator::operator *()
		 */
		double operator *() const;

		/**
		 *	@copydoc CSparseMatrixShapedIterator::operator ++()
		 */
		void operator ++();
	};

	/**
	 *	@brief debugging tool for pinpointing where changes happen in matrices
	 *	@tparam CMatrixType is block matrix type (forward declaration / cyclic inclusion workaround)
	 */
	template <class CMatrixType = CUberBlockMatrix>
	class CMatrixDeltaTracker_ {
	protected:
		CMatrixType m_matrix; /**< @brief reference matrix */
		const bool m_b_allow_dims_change; /**< @brief dimension change tolerance flag */

	public:
		/**
		 *	@brief default constructor; copies the matrix to track the changes in
		 *
		 *	@param[in] r_matrix is reference ot a matrix to be checked for changes (copied)
		 *	@param[in] b_allow_dims_change is dimension change tolerance flag (if set,
		 *		the matrix can change dimensions but must remain numerically unchanged;
		 *		if not set, the dimension change causes the exception to be thrown)
		 */
		CMatrixDeltaTracker_(const CMatrixType &r_matrix, bool b_allow_dims_change = false)
			:m_b_allow_dims_change(b_allow_dims_change)
		{
			m_matrix = r_matrix;
		}

		/**
		 *	@brief makes sure that the matrix did not change, throws an exception otherwise
		 *
		 *	@param[in] other_matrix is matrix to compare with the reference (copy intended)
		 *
		 *	@note This function throws std::runtime_error in case the matrix changed,
		 *		matrix size changed (unless allowed in the constructor) or matrix layout changed.
		 *	@note This function throws std::bad_alloc as it needs to allocate
		 *		a copy of the \ref other_matrix.
		 */
		void operator ()(CMatrixType other_matrix) const // throw(std::bad_alloc,std::runtime_error) // copy intended
		{
			char p_s_message[256];
			const char *p_s_throw = 0;

			if(!m_matrix.AddTo(other_matrix, -1)) {
				bool b_fail = true;
				double f_error = -1;
				const size_t n_other_rows = other_matrix.n_Row_Num(),
					n_other_cols = other_matrix.n_Column_Num();
				if(n_other_rows != m_matrix.n_Row_Num() || n_other_cols != m_matrix.n_Column_Num()) {
					if(n_other_rows <= m_matrix.n_Row_Num() && n_other_cols <= m_matrix.n_Column_Num()) {
						other_matrix.ExtendTo(m_matrix.n_Row_Num(), m_matrix.n_Column_Num());
						if((b_fail = m_matrix.AddTo(other_matrix, -1)))
							f_error = other_matrix.f_Norm();
					} else {
						size_t n_row_num = std::max(n_other_rows, m_matrix.n_Row_Num()),
							n_col_num = std::max(n_other_cols, m_matrix.n_Column_Num());
						other_matrix.ExtendTo(n_row_num, n_col_num);

						CMatrixType my_matrix;
						const_cast<CMatrixType&>(m_matrix).SliceTo(my_matrix,
							m_matrix.n_BlockRow_Num(), m_matrix.n_BlockColumn_Num(), true);
						// use slice to make a reference matrix, save most of the memory

						my_matrix.ExtendTo(n_row_num, n_col_num);
						if((b_fail = my_matrix.AddTo(other_matrix, -1)))
							f_error = other_matrix.f_Norm();
					}

					if(b_fail) {
						sprintf(p_s_message, "matrix dimensions changed (" PRIsize " x " PRIsize
							") to (" PRIsize " x " PRIsize "), layout changed", m_matrix.n_Row_Num(),
							m_matrix.n_Column_Num(), n_other_rows, n_other_cols);
						p_s_throw = p_s_message;
					} else if(!m_b_allow_dims_change || f_error > 0) {
						// either the dimensions are not allowed to change and we throw,
						// or they are allowed to change but there is a numberical error

						sprintf(p_s_message, "matrix dimensions changed (" PRIsize " x " PRIsize
							") to (" PRIsize " x " PRIsize "), numerical L2 difference %g",
							m_matrix.n_Row_Num(), m_matrix.n_Column_Num(), n_other_rows,
							n_other_cols, f_error);
						p_s_throw = p_s_message;
					}
				} else
					p_s_throw = "matrix layout changed";
			} else {
				double f_error = other_matrix.f_Norm();
				if(f_error > 0) {
					sprintf(p_s_message, "matrix is different: L2 error %g", f_error);
					p_s_throw = p_s_message;
				}
			}
			// track changes, handle different sizes and layouts

			if(p_s_throw)
				throw std::runtime_error(p_s_throw);
			// throw at one place so that it is easier to set a breakpoint
		}
	};

	typedef CMatrixDeltaTracker_<> CMatrixDeltaTracker; /**< @brief debugging tool for pinpointing where changes happen in matrices */

	/**
	 *	@brief rasterizes a sparse matrix and saves as a .tga images
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] A is input matrix
	 *	@param[in] A_prev is previous state of the input matrix, for change tracking (can be 0)
	 *	@param[in] n_scalar_size is size of scalar, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Dump_SparseMatrix(const char *p_s_filename, const cs *A,
		const cs *A_prev = 0, int n_scalar_size = 5);

	/**
	 *	@brief rasterizes a sparse matrix and saves as a .tga images
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] A is input matrix
	 *	@param[in] A_prev is previous state of the input matrix, for change tracking (can be 0)
	 *	@param[in] n_max_bitmap_size is the maximal size of the output, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Dump_SparseMatrix_Subsample(const char *p_s_filename, const cs *A,
		const cs *A_prev = 0, size_t n_max_bitmap_size = 640, bool b_symmetric = false);

	/**
	 *	@brief rasterizes a sparse matrix and saves as a .tga images
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] A is input matrix
	 *	@param[in] A_prev is previous state of the input matrix, for change tracking (can be 0)
	 *	@param[in] n_max_bitmap_size is the maximal size of the output, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Dump_SparseMatrix_Subsample_AA(const char *p_s_filename, const cs *A,
		const cs *A_prev = 0, size_t n_max_bitmap_size = 640, bool b_symmetric = false);

	/**
	 *	@brief prints sparse matrix as a dense matrix
	 *
	 *	@param[in] A is the matrix to be printed
	 *	@param[in] p_s_label is the name of the matrix (can be null)
	 */
	static void Print_SparseMatrix(const cs *A, const char *p_s_label = 0);

	/**
	 *	@brief prints sparse matrix as a dense matrix, in matlab format
	 *
	 *	@param[in] A is the matrix to be printed
	 *	@param[in] p_s_label is the name of the matrix (can be null)
	 */
	static void Print_SparseMatrix_in_MatlabFormat(const cs *A, const char *p_s_label = 0);

	/**
	 *	@brief prints sparse matrix as a dense matrix, in matlab format
	 *
	 *	@param[in] p_fw is the output stream
	 *	@param[in] A is the matrix to be printed
	 *	@param[in] p_s_prefix is prefix code before the matrix body (can be null)
	 *	@param[in] p_s_suffix is suffix code after the matrix body (can be null)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Print_SparseMatrix_in_MatlabFormat(FILE *p_fw,
		const cs *A, const char *p_s_prefix = 0, const char *p_s_suffix = ";\n");

	/**
	 *	@brief prints dense matrix dimensions
	 *
	 *	@tparam MatrixType is specialization of Eigen::Matrix
	 *
	 *	@param[in] p_fw is the output stream
	 *	@param[in] r_A is the matrix to be printed
	 *	@param[in] p_s_prefix is prefix code before the matrix body (can be null)
	 *	@param[in] p_s_suffix is suffix code after the matrix body (can be null)
	 */
	template <class MatrixType>
	static void Print_DenseMatrix_Dimensions(FILE *p_fw, const MatrixType &r_A,
		const char *p_s_prefix = 0, const char *p_s_suffix = "\n")
	{
		if(p_s_prefix)
			fprintf(p_fw, "%s", p_s_prefix);
		fprintf(p_fw, "dense, " PRIsize " x " PRIsize, r_A.rows(), r_A.cols());
		if(p_s_suffix)
			fprintf(p_fw, "%s", p_s_suffix);
	}

	/**
	 *	@brief prints dense matrix in matlab format
	 *
	 *	@tparam MatrixType is specialization of Eigen::Matrix
	 *
	 *	@param[in] p_fw is the output stream
	 *	@param[in] r_A is the matrix to be printed
	 *	@param[in] p_s_prefix is prefix code before the matrix body (can be null)
	 *	@param[in] p_s_suffix is suffix code after the matrix body (can be null)
	 *	@param[in] p_s_fmt is format string (" %f" by default; the space is important)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class MatrixType>
	static bool Print_DenseMatrix_in_MatlabFormat(FILE *p_fw,
		const MatrixType &r_A, const char *p_s_prefix = 0,
		const char *p_s_suffix = ";\n", const char *p_s_fmt = " %f")
	{
		if(p_s_prefix)
			fprintf(p_fw, "%s", p_s_prefix);

		{
			fprintf(p_fw, "[");
			for(size_t i = 0, m = r_A.rows(); i < m; ++ i) { // iterate rows in the outer loop
				for(size_t j = 0, n = r_A.cols(); j < n; ++ j) { // iterate columns in the inner loop
					double f_value = r_A(i, j);
					fprintf(p_fw, p_s_fmt, f_value);
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

	/**
	 *	@brief prints sparse matrix as a true sparse matrix,
	 *		in matlab format (requires suitesparse)
	 *
	 *	@param[in] p_fw is the output stream
	 *	@param[in] A is the matrix to be printed
	 *	@param[in] p_s_label is a name of the matrix variable
	 *	@param[in] p_s_prefix is prefix code before the matrix body (can be null)
	 *	@param[in] p_s_suffix is suffix code on the next
	 *		line after the matrix body (can be null)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Print_SparseMatrix_in_MatlabFormat2(FILE *p_fw, const cs *A,
		const char *p_s_label, const char *p_s_prefix = 0, const char *p_s_suffix = 0);

	/**
	 *	@brief prints a dense vector
	 *
	 *	@param[in] b is the vector to be printed
	 *	@param[in] n_vector_length is number of elements in b
	 *	@param[in] p_s_label is the name of the matrix (can be null)
	 */
	static void Print_DenseVector(const double *b, size_t n_vector_length, const char *p_s_label = 0);

	/**
	 *	@brief prints a dense vector in matlab format
	 *
	 *	@param[in] p_fw is a file where to save the vector
	 *	@param[in] b is the vector to be printed
	 *	@param[in] n_vector_length is number of elements in b
	 *	@param[in] p_s_prefix is prefix code before the vector body (can be null)
	 *	@param[in] p_s_suffix is suffix code after the vector body (can be null)
	 */
	static void Print_DenseVector_in_MatlabFormat(FILE *p_fw, const double *b,
		size_t n_vector_length, const char *p_s_prefix = 0, const char *p_s_suffix = ";\n");

	/**
	 *	@brief determines whether a set of elements is sorted, permits repeated elements
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
	 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
	 *
	 *	@return Returns true if the set is sorted in ascending
	 *		order, otherwise returns false.
	 */
	template <class _TyConstIter>
	static bool b_IsWeaklySortedSet(_TyConstIter p_begin_it, _TyConstIter p_end_it)
	{
		/*//typedef std::iterator_traits<_TyConstIter>::value_type T;
		// unused, dont want to copy the objects in case they are large

		_ASSERTE(p_end_it >= p_begin_it);
		if(p_begin_it == p_end_it)
			return true;
		// empty set is sorted

		_TyConstIter p_prev_it = p_begin_it;
		for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it) {
			if(*p_begin_it < *p_prev_it)
				return false; // not sorted, or contains repeating elements
			p_prev_it = p_begin_it;
		}
		return true;*/

		return b_IsWeaklySortedSet(p_begin_it, p_end_it,
			std::less<typename std::iterator_traits<_TyConstIter>::value_type>());
		// reuse the implementation
	}

	/**
	 *	@brief determines whether a set of elements is sorted and does not contain duplicate elements
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
	 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
	 *
	 *	@return Returns true if the set is sorted in ascending
	 *		order and does not contain duplicate elements, otherwise returns false.
	 */
	template <class _TyConstIter>
	static bool b_IsStrictlySortedSet(_TyConstIter p_begin_it, _TyConstIter p_end_it)
	{
		/*//typedef std::iterator_traits<_TyConstIter>::value_type T;
		// unused, dont want to copy the objects in case they are large

		_ASSERTE(p_end_it >= p_begin_it);
		if(p_begin_it == p_end_it)
			return true;
		// empty set is sorted

		_TyConstIter p_prev_it = p_begin_it;
		for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it) {
			if(!(*p_prev_it < *p_begin_it))
				return false; // not sorted, or contains repeating elements
			p_prev_it = p_begin_it;
		}
		return true;*/

		return b_IsStrictlySortedSet(p_begin_it, p_end_it,
			std::less<typename std::iterator_traits<_TyConstIter>::value_type>());
		// reuse the implementation
	}

	/**
	 *	@brief determines whether a set of elements is sorted, permits repeated elements
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
	 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
	 *
	 *	@return Returns true if the set is sorted in ascending
	 *		order, otherwise returns false.
	 */
	template <class _TyConstIter, class _TyComparator>
	static bool b_IsWeaklySortedSet(_TyConstIter p_begin_it, _TyConstIter p_end_it, _TyComparator comp)
	{
		//typedef std::iterator_traits<_TyConstIter>::value_type T;
		// unused, dont want to copy the objects in case they are large

		_ASSERTE(p_end_it >= p_begin_it);
		if(p_begin_it == p_end_it)
			return true;
		// empty set is sorted

		_TyConstIter p_prev_it = p_begin_it;
		for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it) {
			if(comp(*p_begin_it, *p_prev_it))
				return false; // not sorted, or contains repeating elements
			p_prev_it = p_begin_it;
		}
		return true;
	}

	/**
	 *	@brief determines whether a set of elements is sorted and does not contain duplicate elements
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *	@tparam _TyComparator is comparator object
	 *
	 *	@param[in] p_begin_it is iterator, pointing to the first vertex index
	 *	@param[in] p_end_it is iterator, pointing to one past the last vertex index
	 *
	 *	@return Returns true if the set is sorted in ascending
	 *		order and does not contain duplicate elements, otherwise returns false.
	 */
	template <class _TyConstIter, class _TyComparator>
	static bool b_IsStrictlySortedSet(_TyConstIter p_begin_it, _TyConstIter p_end_it, _TyComparator comp)
	{
		//typedef std::iterator_traits<_TyConstIter>::value_type T;
		// unused, dont want to copy the objects in case they are large

		_ASSERTE(p_end_it >= p_begin_it);
		if(p_begin_it == p_end_it)
			return true;
		// empty set is sorted

		_TyConstIter p_prev_it = p_begin_it;
		for(++ p_begin_it; p_begin_it != p_end_it; ++ p_begin_it) {
			if(!comp(*p_prev_it, *p_begin_it))
				return false; // not sorted, or contains repeating elements
			p_prev_it = p_begin_it;
		}
		return true;
	}

	static void Evict_Buffer(const void *p_begin, size_t n_size);

	static void Swamp_Cache(size_t n_buffer_size = 100 * 1048576); // throw(std::bad_alloc)

	template <class _Ty/*, class _Alloc*/>
	static void Evict(const std::vector<_Ty/*, _Alloc*/> &r_vec)
	{
		if(!r_vec.empty()) { // if empty, there might be some buffer but no way to get the front pointer
			//for(size_t i = 0, n = r_vec.capacity(); i < n; ++ i)
			//	Evict(&r_vec.front() + i); // evict each element to correctly handle vectors of vectors?
			// does not handle structs anyway, need to leave it up to the caller
			Evict_Buffer(&r_vec.front(), r_vec.capacity() * sizeof(_Ty));
		}
		Evict_Buffer(&r_vec, sizeof(r_vec));
	}

	/*template <class Derived>
	static void Evict(const Eigen::MatrixBase<Derived> &r_mat)
	{
		if(Derived::RowsAtCompileTime == Eigen::Dynamic ||
		   Derived::ColsAtCompileTime == Eigen::Dynamic ||
		   Derived::MaxRowsAtCompileTime == Eigen::Dynamic ||
		   Derived::MaxColsAtCompileTime == Eigen::Dynamic)
			Evict_Buffer(r_mat.data(), r_mat.rows() * r_mat.cols() * sizeof(typename Derived::Scalar));
		// evict the internal storage

		Evict_Buffer(&r_mat, sizeof(r_mat));
		// evict the structure
	}*/

	static void Evict(const cs *p_mat);

protected:

public:
	template <class _Ty, const int n_page_size_elems, const int n_memory_alignment>
	static void Evict(const fap_detail::fap_base<_Ty, n_page_size_elems, n_memory_alignment> &r_pool)
	{
		for(size_t i = 0, n = r_pool.page_num(), s = r_pool.page_size(); i < n; ++ i)
			Evict_Buffer(&r_pool[i * s], s * sizeof(_Ty));
		// evict the page data

		CFAP_EvictUtil<fap_detail::fap_base<_Ty, n_page_size_elems,
			n_memory_alignment> >::EvictMember(r_pool);
		// evict the page list

		Evict_Buffer(&r_pool, sizeof(r_pool));
		// evict the structure
	}

	static void Evict(const CUberBlockMatrix &r_mat);

protected:
	template <class CDerived>
	class CFAP_EvictUtil : public CDerived { // uncool hack to get to the members without having to backport this to the base
	public:
		static void EvictMember(const CDerived &r_derived)
		{
			const CFAP_EvictUtil &r_cast = (const CFAP_EvictUtil&)r_derived;

			::CDebug::Evict(r_cast.m_page_list);
		}
	};

	template <class CDerived>
	class CUBM_EvictUtil : public CDerived { // uncool hack to get to the members without having to backport this to the base
	public:
		static void EvictMembers(const CDerived &r_derived)
		{
			const CUBM_EvictUtil &r_cast = (const CUBM_EvictUtil&)r_derived;

			for(size_t i = 0, n = r_cast.m_block_cols_list.size(); i < n; ++ i)
				::CDebug::Evict(r_cast.m_block_cols_list[i].block_list);
			::CDebug::Evict(r_cast.m_block_cols_list);
			::CDebug::Evict(r_cast.m_block_rows_list);
			::CDebug::Evict(r_cast.m_data_pool);
		}
	};
};

/**
 *	@brief sparse matrix memory requirements information class
 */
class CSparseMatrixMemInfo {
public:
	/**
	 *	@brief calculates allocation size of a sparse matrix
	 *	@param[in] p_matrix is matrix to measure size of
	 *	@return Returns allocation size of the matrix, in bytes.
	 */
	static uint64_t n_Allocation_Size(const cs *p_matrix);

	/**
	 *	@brief matrix market sparse matrix header
	 */
	struct TMMHeader {
		bool b_symmetric; /**< @brief symmetry flag (if set, the matrix is symmetric and the nnz of only one triangle, upper or lower, is stored) */
		bool b_binary; /**< @brief pattern matrix flag (if set, the matrix is binary and there are no values for the elements) */
		size_t n_rows; /**< @brief number of rows */
		size_t n_cols; /**< @brief number of columns */
		size_t n_nnz;  /**< @brief number of stored nonzeros (about a half of the actual nonzeros if the matrix is symmetric) */ // n_nnz contains the number of *stored* nonzeros (about a half if the matrix is symmetric)
		size_t n_full_nnz; /**< @brief number of full matrix nonzeros (if not symmetric then this equals the stored nonzeros, otherwise this will be about a double of the stored nonzeros) */ // equal to n_nnz if not symmetric, or the about twice that amount (all that are in the full matrix)

		uint64_t n_AllocationSize_CSC(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
		{
			uint64_t n_size_cols = std::min((std::min(uint64_t(n_cols),
				UINT64_MAX - 1) + 1), UINT64_MAX / n_idx_size) * n_idx_size;
			uint64_t n_size_rows = std::min(uint64_t(n_full_nnz), UINT64_MAX / n_idx_size) * n_idx_size;
			uint64_t n_size_data = 0;
			if(!b_binary)
				n_size_data = std::min(uint64_t(n_full_nnz), UINT64_MAX / n_scalar_size) * n_scalar_size;
			uint64_t n_size = 0;
			n_size = std::min(n_size, UINT64_MAX - n_size_cols) + n_size_cols;
			n_size = std::min(n_size, UINT64_MAX - n_size_rows) + n_size_rows;
			n_size = std::min(n_size, UINT64_MAX - n_size_data) + n_size_data;
			// calculate size (with saturation arithmetics)

			return n_size;
		}

		uint64_t n_AllocationSize_COO(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
		{
			size_t n_triplet_size = (b_binary)? 2 * n_idx_size : 2 * n_idx_size + n_scalar_size;
			return std::min(uint64_t(n_full_nnz), UINT64_MAX / n_triplet_size) * n_triplet_size;
		}
	};

	/**
	 *	@brief peeks at the matrix header
	 *
	 *	@param[in] p_s_filename is null-terminated string containing input file name
	 *	@param[out] r_t_header is reference to a matrix market header
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Peek_MatrixMarket_Header(const char *p_s_filename, TMMHeader &r_t_header);

	/**
	 *	@brief peeks at the matrix header and returns the size of such matrix in memory
	 *	@param[in] p_s_filename is null-terminated string containing input file name
	 *	@return Returns the size of the CSC form of the matrix in memory, in bytes,
	 *		assuming double scalars and csi indices (32 or 64 bit, depending on the
	 *		target architecture).
	 */
	static uint64_t n_Peek_MatrixMarket_SizeInMemory(const char *p_s_filename);
};

#endif // !__DEBUGGING_FUNCTIONALITY_INCLUDED
