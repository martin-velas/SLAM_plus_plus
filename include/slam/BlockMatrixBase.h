/*
								+--------------------------------+
								|                                |
								| *** Über Block Matrix base *** |
								|                                |
								| Copyright (c) -tHE SWINe- 2013 |
								|                                |
								|       BlockMatrixBase.h        |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_BASE_INCLUDED
#define __UBER_BLOCK_MATRIX_BASE_INCLUDED

/**
 *	@file include/slam/BlockMatrixBase.h
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief the ÜberBlockMatrix base class
 *
 *	@date 2017-05-02
 *
 *	Changed the alignment rules to better accomodate new Eigen 3.3 features (increased
 *	alignment for AVX instruction sets, no alignment for unaligned vectorization).
 *
 */

/** \addtogroup ubm
 *	@{
 */

/**
 *	@def __UBER_BLOCK_MATRIX_IO
 *	@brief if defined, Load_MatrixMarket() and Save_MatrixMarket() functions are compiled
 */
#define __UBER_BLOCK_MATRIX_IO

/**
 *	@def __UBER_BLOCK_MATRIX_HAVE_CSPARSE
 *	@brief enables interop with CSparse
 */
#define __UBER_BLOCK_MATRIX_HAVE_CSPARSE

/**
 *	@def __UBER_BLOCK_MATRIX_PERFCOUNTERS
 *	@brief if defined, the block matrix contains some performance
 *		counters for diagnostic (not recommended for production code)
 *	@note These performance counters operate in block allocation code
 *		only, the arithmetic operations may bypass it.
 */
//#define __UBER_BLOCK_MATRIX_PERFCOUNTERS

/**
 *	@def __UBER_BLOCK_MATRIX_DISABLE_INTEGRITY_CHECK
 *	@brief disables matrix integrity checks (use when integrity checks
 *		are prohibitive during debugging, do not leave on forever)
 */
//#define __UBER_BLOCK_MATRIX_DISABLE_INTEGRITY_CHECK

/**
 *	@def __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK
 *	@brief enables debug checks for matrix block bounds overrun (_DEBUG only)
 *	@note This uses relatively large blocks, can double memory consumption
 *		if using matrices with very small blocks.
 */
#define __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

/**
 *	@def __UBER_BLOCK_MATRIX_TRIANGULAR_SOLVE_NEVER_FAILS
 *	@brief if defined, the triangular solving routines will not check for division by zero
 *		and will instead compute a vector with infinities and not-a-numbers if a zero occurs
 *	@note Division by zero is still checked in debug, via an assertion.
 */
//#define __UBER_BLOCK_MATRIX_TRIANGULAR_SOLVE_NEVER_FAILS

/**
 *	@def __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR
 *	@brief uses block matrix multiplication with constant time block
 *		seeking, making the bookkeeping time lower (there is some constant
 *		overhead, so probably not good for small matrices)
 */
#define __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR

/**
 *	@def __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON
 *	@brief uses another block matrix multiplication with constant time block
 *		seeking, making the bookkeeping time lower (there is some constant
 *		overhead, so probably not good for small matrices)
 *	@note This takes precedence over \ref __UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR.
 */
//#define __UBER_BLOCK_MATRIX_MULTIPLICATION_GUSTAVSON

/**
 *	@def __UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM
 *	@brief if defined, the legacy decision hand-coded tree code is used instead
 *		of the new-style fbs_ut::CWrap3<> one; if not defined, either the
 *		\ref MultiplyToWith_TransposeSort_FBS() or \ref MultiplyToWith_AccumLookup_FBS()
 *		variant is called from inside \ref MultiplyToWith_FBS()
 */
//#define __UBER_BLOCK_MATRIX_LEGACY_FBS_GEMM

/**
 *	@def __UBER_BLOCK_MATRIX_HYBRID_AT_A
 *	@brief uses hybrid block matrix multiplication for (pre)multiplying
 *		the matrix by its transpose
 *	@note Although the complexity isn't lower than in other versions
 *		of the routine, the memory coherency should be slightly better.
 */
#define __UBER_BLOCK_MATRIX_HYBRID_AT_A

/**
 *	@def __UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS
 *	@brief keeps track of numbers of blocks produced to avoid
 *		block list reallocations (only has effect if
 *		__UBER_BLOCK_MATRIX_MULTIPLICATION_LINEAR is defined)
 *	@note This doesn't gain any significant speedup and only
 *		uses slightly more memory, probably better to leave disabled.
 */
//#define __UBER_BLOCK_MATRIX_MULTIPLICATION_PREALLOCATES_BLOCK_LISTS

/**
 *	@def __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
 *	@brief aligns block memory to 16 or 32-byte boundaries in order to support SSE or AVX
 *	@note This also interacts with EIGEN_UNALIGNED_VECTORIZE.
 */
//#define __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY

/**
 *	@def __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_OPS
 *	@brief enables fast fixed block size functions (these with _FBS suffix)
 *	@note The FBS functions give numerically slightly different results
 *		(difference in the one or two least significant bits). Usually not
 *		enough to cause any trouble (e.g. such as two different the BLAS
 *		implementations likely give different results doesn't cause problems).
 */
#define __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_OPS

/**
 *	@def __UBER_BLOCK_MATRIX_SUPRESS_FBS_IN_DEBUG
 *	@brief disables fast fixed block size functions in debug builds
 *		(the functions are still available, but the non-FBS versions
 *		are called instead)
 *	@note The FBS functions give numerically slightly different results
 *		(difference in the one or two least significant bits), disabling
 *		them for debug may conceal some numerical problems which could
 *		later arise in release mode. Use with caution.
 */
//#define __UBER_BLOCK_MATRIX_SUPRESS_FBS_IN_DEBUG

/**
 *	@def __UBER_BLOCK_MATRIX_SUPRESS_FBS
 *	@brief disables fast fixed block size functions (the functions
 *		are still available, but the non-FBS versions are called instead)
 *	@note The FBS functions give numerically slightly different results
 *		(difference in the one or two least significant bits), disabling
 *		them for debug may conceal some numerical problems which could
 *		otherwise arise. Use with caution.
 */
//#define __UBER_BLOCK_MATRIX_SUPRESS_FBS

/**
 *	@def __UBER_BLOCK_MATRIX_LAZY_PRODUCT
 *	@brief enables the use of lazyProduct() in ordinary (non-FBS) functions
 *		instead of .noalias() l-value, where applicable
 */
#define __UBER_BLOCK_MATRIX_LAZY_PRODUCT

/**
 *	@def __UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT
 *	@brief enables the use of lazyProduct() in FBS functions instead
 *		of .noalias() l-value, where applicable
 */
#define __UBER_BLOCK_MATRIX_FBS_LAZY_PRODUCT

/**
 *	@def __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
 *	@brief enables debug functions for fixed block size code generating templates
 *	@note This has effect only if the __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_OPS
 *		macro is defined as well.
 */
//#define __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

/**
 *	@def __UBER_BLOCK_MATRIX_FROM_SPARSE_ITERATE_INTEGERS
 *	@brief changes From_Sparse() and its variants to iterate integer indices instead of pointers
 *	@note Seems to be faster with pointers.
 */
//#define __UBER_BLOCK_MATRIX_FROM_SPARSE_ITERATE_INTEGERS

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_DISABLE_INTEGRITY_CHECK)
#pragma message("warning: CUberBlockMatrix integrity checks disabled")
#endif // _DEBUG && __UBER_BLOCK_MATRIX_DISABLE_INTEGRITY_CHECK

#if !defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_PERFCOUNTERS)
#pragma message("warning: __UBER_BLOCK_MATRIX_PERFCOUNTERS defined in release mode")
#endif // !_DEBUG && __UBER_BLOCK_MATRIX_PERFCOUNTERS

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_SUPRESS_FBS_IN_DEBUG)
#ifndef __UBER_BLOCK_MATRIX_SUPRESS_FBS
#define __UBER_BLOCK_MATRIX_SUPRESS_FBS
#endif // !__UBER_BLOCK_MATRIX_SUPRESS_FBS
#endif // __DEBUG && __UBER_BLOCK_MATRIX_SUPRESS_FBS_IN_DEBUG

#ifdef __UBER_BLOCK_MATRIX_SUPRESS_FBS
#pragma message("warning: CUberBlockMatrix fixed block size operations supressed (results may vary)")
#endif // __UBER_BLOCK_MATRIX_SUPRESS_FBS

#include "eigen/Eigen/Core"
#include "eigen/Eigen/LU" // dense matrix inverse
#include "eigen/Eigen/Cholesky" // dense Cholesky

#if !defined(__UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY) && \
	defined(EIGEN_VECTORIZE) && !defined(EIGEN_DONT_VECTORIZE) && \
	(!defined(EIGEN_UNALIGNED_VECTORIZE) || EIGEN_UNALIGNED_VECTORIZE == 0)
//	(defined(EIGEN_VECTORIZE) || defined(_M_X64) || defined(_M_AMD64) || \
//	defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64))
#define __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
#endif // !__UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY && EIGEN_VECTORIZE && !EIGEN_DONT_VECTORIZE && !EIGEN_UNALIGNED_VECTORIZE
//&& (EIGEN_VECTORIZE || _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64)
// force __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY in x64,
// also if EIGEN_VECTORIZE (practically if SSE2 is enabled in MSVC compiler settings)
// this comes *after* eigen is included

#include "slam/Debug.h" // _ASSERTE, _isnan
#include <vector>
#include "slam/Segregated.h"
#include <utility>
#include <algorithm>
#include <set>
#include <stdexcept> // runtime_error
#ifdef __UBER_BLOCK_MATRIX_HAVE_CSPARSE
#include "csparse/cs.hpp" // CSparse
#endif // __UBER_BLOCK_MATRIX_HAVE_CSPARSE
#include "slam/Tga.h"
#include "slam/TypeList.h"
#include "slam/Timer.h" // profiling
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#ifdef __UBER_BLOCK_MATRIX_IO
#include <stdio.h>
#endif // __UBER_BLOCK_MATRIX_IO

/**
 *	@def __INLINE_RECURSION_ENABLE_PRAGMA
 *	@brief pragma that enables inline recursion of (seemingly) recursive functions
 *	@note Because it is impossible to put complete pragma inside a macro,
 *		this is just a name of the pragma. Where equivalent pragma not available,
 *		it is replaced by <tt>pack()</tt>, which is considered a no-op pragma for
 *		a function.
 */
#if defined(_MSC_VER) && !defined(__MWERKS__)
#define __INLINE_RECURSION_ENABLE_PRAGMA inline_recursion(on)
#else // _MSC_VER && !__MWERKS__
#define __INLINE_RECURSION_ENABLE_PRAGMA pack()
// this is supposedly no-op pragma
#endif // _MSC_VER && !__MWERKS__

/**
 *	@def __forceinline
 *	@brief replace __forceinline with inline on non-MSVC compilers
 */
#if !defined(_MSC_VER) || defined(__MWERKS__)
#define __forceinline inline
#endif // !_MSC_VER || __MWERKS__

class CGPU_BlockMatrix; // forward declaration

/**
 *	@brief contains static assertion template for alignment check
 */
namespace blockmatrix_detail {

/**
 *	@brief compile-time assertion helper
 *	@tparam b_expression is assertion value
 */
template <const bool b_expression>
class CStaticAssert {
public:
	typedef void MATRIX_POOL_MEMORY_ALIGNMENT_MUST_BE_INTEGER_MULTIPLE_OF_SCALAR_TYPE_SIZE; /**< @brief assertion tag */
	typedef void MATRIX_BLOCK_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME; /**< @brief assertion tag */
	typedef void MATRIX_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME; /**< @brief assertion tag */
	typedef void MATRIX_DIMENSIONS_MISMATCH; /**< @brief assertion tag */
	typedef void MATRIX_MUTUALLY_EXCLUSIVE_OPTIONS_SELECTED; /**< @brief assertion tag */
};

/**
 *	@brief compile-time assertion helper (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

/**
 *	@brief UberLame block matrix base class
 */
class CUberBlockMatrix_Base {
friend class ::CGPU_BlockMatrix;
public:
	/**
	 *	@brief parameters, stored as enums
	 */
	enum {
#ifdef __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
		pool_MemoryAlignment = EIGEN_IDEAL_MAX_ALIGN_BYTES, /**< @brief data storage memory alignment (in bytes) */
		map_Alignment = (pool_MemoryAlignment == 8)? Eigen::Aligned8 :
			(pool_MemoryAlignment == 32)? Eigen::Aligned32 :
			(pool_MemoryAlignment == 64)? Eigen::Aligned64 :
			(pool_MemoryAlignment == 128)? Eigen::Aligned128 :Eigen::Aligned16, /**< @brief alignment specification for aligned Eigen::Map */
#else // EIGEN_VERSION_AT_LEAST(3, 3, 0)
		pool_MemoryAlignment = 16, /**< @brief data storage memory alignment (in bytes) */
		map_Alignment = Eigen::Aligned, /**< @brief alignment specification for aligned Eigen::Map */
#endif // EIGEN_VERSION_AT_LEAST(3, 3, 0)
#else // __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
		pool_MemoryAlignment = 0, /**< @brief data storage memory alignment (in bytes) */
		map_Alignment = Eigen::Unaligned, /**< @brief alignment specification for aligned Eigen::Map */
#endif // __UBER_BLOCK_MATRIX_ALIGN_BLOCK_MEMORY
		pool_PageSize = 1048576 /**< @brief data storage page size */
	};

	typedef Eigen::Map<Eigen::Vector3d> _TyVector3dRef; /**< @brief 3D referencing vector type */
	typedef Eigen::Map<Eigen::RowVector3d> _TyRowVector3dRef; /**< @brief 3D referencing row vector type */
	typedef Eigen::Map<Eigen::Matrix3d, map_Alignment> _TyMatrix3dRef; /**< @brief 3x3 referencing matrix type */

	typedef Eigen::Map<Eigen::VectorXd> _TyVectorXdRef; /**< @brief dynamic-size referencing vector type */
	typedef Eigen::Map<Eigen::RowVectorXd> _TyRowVectorXdRef; /**< @brief dynamic-size referencing row vector type */
	typedef Eigen::Map<Eigen::MatrixXd, map_Alignment> _TyMatrixXdRef; /**< @brief dynamic-size referencing matrix type */

	typedef const Eigen::Map<const Eigen::VectorXd> _TyConstVectorXdRef; /**< @brief dynamic-size referencing const vector type */
	typedef const Eigen::Map<const Eigen::RowVectorXd> _TyConstRowVectorXdRef; /**< @brief dynamic-size referencing const row vector type */
	typedef const Eigen::Map<const Eigen::MatrixXd, map_Alignment> _TyConstMatrixXdRef; /**< @brief dynamic-size referencing const matrix type */

	/**
	 *	@brief reference matrix helper
	 *
	 *	@tparam n_row_num is number of matrix rows
	 *	@tparam n_column_num is number of matrix columns
	 */
	template <int n_row_num, int n_column_num>
	class CMakeMatrixRef {
	public:
		typedef Eigen::Matrix<double, n_row_num, n_column_num> _TyMatrix; /**< @brief plain matrix type */
		typedef Eigen::Map<_TyMatrix, map_Alignment> _Ty; /**< @brief referencing matrix type */
		typedef const Eigen::Map<const _TyMatrix, map_Alignment> _TyConst; /**< @brief referencing const matrix type */
	};

	/**
	 *	@brief reference (column) vector helper
	 *
	 *	@tparam n_row_num is number of vector rows
	 */
	template <int n_row_num>
	class CMakeVectorRef {
	public:
		typedef Eigen::Map<Eigen::Matrix<double, n_row_num, 1> > _Ty; /**< @brief referencing vector type */
		typedef const Eigen::Map<const Eigen::Matrix<double, n_row_num, 1> > _TyConst; /**< @brief referencing const vector type */
	};

	/**
	 *	@brief reference row vector helper
	 *
	 *	@tparam n_column_num is number of vector columns
	 */
	template <int n_column_num>
	class CMakeRowVectorRef {
	public:
		typedef Eigen::Map<Eigen::Matrix<double, 1, n_column_num> > _Ty; /**< @brief referencing row vector type */
		typedef const Eigen::Map<const Eigen::Matrix<double, 1, n_column_num> > _TyConst; /**< @brief referencing const row vector type */
	};

	typedef forward_allocated_pool<double, pool_PageSize, pool_MemoryAlignment> _TyPool; /**< @brief data storage type */

	/**
	 *	@brief row storage
	 *	@todo - try without storing height, only cummulative
	 */
	struct TRow {
		size_t n_height; /**< @brief height of the row */
		size_t n_cumulative_height_sum; /**< @brief position of (the first element of) the row in the matrix @todo - make this the position of one past the last element */

		/**
		 *	@brief block structure comparator
		 *	@param[in] r_t_other is the other block structure to compare to
		 *	@return Returns true if the rows are the same, otherwise returns false.
		 *	@note This compares both cumulative sum and height, it mihgt be wasteful
		 *		to use for all the rows, as equality in one implies equality in the other.
		 */
		inline bool operator ==(const TRow &r_t_other) const
		{
			return r_t_other.n_height == n_height &&
				r_t_other.n_cumulative_height_sum == n_cumulative_height_sum;
		}

		/**
		 *	@brief gets height of a row
		 *	@return Returns height of the supplied row.
		 */
		inline size_t n_GetAbsolute() const
		{
			return n_height;
		}

		/**
		 *	@brief gets cumulative height sum of a row
		 *	@return Returns cumulative height sum of the supplied row.
		 */
		inline size_t n_GetCumulative() const
		{
			return n_cumulative_height_sum;
		}

		/**
		 *	@brief sets height of a row
		 *	@param[in] n_value is initialization value
		 */
		inline void SetAbsolute(size_t n_value)
		{
			n_height = n_value;
		}

		/**
		 *	@brief sets cumulative height sum of a row
		 *	@param[in] n_value is initialization value
		 */
		inline void SetCumulative(size_t n_value)
		{
			n_cumulative_height_sum = n_value;
		}
	};

	/**
	 *	@brief column storage
	 *	@note This is essentially collumn-compressed format. The advantage is the ability of adding elements
	 *		without recalculating all the cumulative sums as in csparse. The disadvantage is the double vector,
	 *		causing many memory references before actually getting to the matrix data (but the matrix data are
	 *		laid out in potentionally contiguous memory, with near matrices being near in the memory, increasing
	 *		cache efficiency in the computational loops).
	 *	@note In A, we mostly have up to two blocks on every row / column. That would make block_list quite an overkill.
	 *	@todo try without storing width, only cummulative
	 *	@todo try storing row index, but also row height in each block (saves one ref per block on some ops)
	 */
	// *	t_odo See if Florida matrix collection has any block matrices in it, or think about how to blockify them.
	// *	t_odo We better avoid copying this structure. Make a matrix observer
	// *		to be able to select just a part of matrix for reading / writing.
	struct TColumn {
		typedef std::pair<size_t, double*> TBlockEntry; /**< @brief one block entry @todo - try put row height and position, might dramatically increase locality of reference */

		size_t n_width; /**< @brief width of the column */
		size_t n_cumulative_width_sum; /**< @brief position of (the first element of) the column in the matrix @todo - make this the position of one past the last element */
		std::vector<TBlockEntry> block_list; /**< @brief list of blocks (first is row index, second is storage) */

		/**
		 *	@brief gets width of a column
		 *	@return Returns width of the supplied column.
		 */
		inline size_t n_GetAbsolute() const
		{
			return n_width;
		}

		/**
		 *	@brief gets cumulative width sum of a column
		 *	@return Returns cumulative width sum of the supplied column.
		 */
		inline size_t n_GetCumulative() const
		{
			return n_cumulative_width_sum;
		}

		/**
		 *	@brief sets width of a column
		 *	@param[in] n_value is initialization value
		 */
		inline void SetAbsolute(size_t n_value)
		{
			n_width = n_value;
		}

		/**
		 *	@brief sets cumulative width sum of a column
		 *	@param[in] n_value is initialization value
		 */
		inline void SetCumulative(size_t n_value)
		{
			n_cumulative_width_sum = n_value;
		}

		/**
		 *	@brief swaps column contents with another column, which is not initialized
		 *	@param[in,out] r_t_dest is the other column for swapping
		 *	@note This fuction dammages this column (it is intended as "fast copy"
		 *		where the source operand is not needed afterwards).
		 */
		inline void UninitializedSwap(TColumn &r_t_dest)
		{
			r_t_dest.n_width = n_width;
			r_t_dest.n_cumulative_width_sum = n_cumulative_width_sum;
			r_t_dest.block_list.swap(block_list);
		}
	};

	typedef std::vector<TRow>::iterator _TyRowIter; /**< @brief row vector iterator */
	typedef std::vector<TRow>::const_iterator _TyRowConstIter; /**< @brief row vector const iterator */
	typedef std::vector<TColumn>::iterator _TyColumnIter; /**< @brief column vector iterator */
	typedef std::vector<TColumn>::const_iterator _TyColumnConstIter; /**< @brief column vector const iterator */
	typedef std::vector<TColumn::TBlockEntry>::iterator _TyBlockIter; /**< @brief block entry vector iterator */
	typedef std::vector<TColumn::TBlockEntry>::const_iterator _TyBlockConstIter; /**< @brief block entry vector const iterator */

	// @todo - try working with representation for a bit - storing just cumsusm? storing row height in block?
	// t_odo - implement Extend(rows, cols) operation that enables addition of matrices of different shape
	// @todo - come up with different designs, benchmark what is best for our use (template away? that'd be rather cool)
	// @todo - look into different sparse matrix representations (jagged diagonal, ...)

	/**
	 *	@brief equality comparison between row index in block entry and raw value
	 *	@note This is intended to be used as a predicate for std::find_if() that
	 *		can operate on unsorted arrays in intermediate matrices formed inside
	 *		various algorithms. Note that the block matrices returned to the caller
	 *		should always have the block entries sorted and std::lower_bound() is
	 *		preferred. This is intended only for internal debugging.
	 */
	class CFindBlockRow_Unordered {
	protected:
		const size_t m_n_row_id; /**< @brief compared block row id */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] n_row_id is the compared block row id
		 */
		CFindBlockRow_Unordered(size_t n_row_id)
			:m_n_row_id(n_row_id)
		{}

		/**
		 *	@brief block entry row comparison operator
		 *	@apram[in] r_t_block is const reference to a block entry
		 *	@return Returns true if the block lies on the same block row
		 *		as specified in the constructor, otherwise returns false.
		 */
		inline bool operator ()(const TColumn::TBlockEntry &r_t_block) const
		{
			return m_n_row_id == r_t_block.first;
		}
	};

	/**
	 *	@brief greater-than comparison between row index in block entry and raw value
	 *	@note This is intended to be used as a predicate for std::find_if() that
	 *		can operate on unsorted arrays in intermediate matrices formed inside
	 *		various algorithms. Note that the block matrices returned to the caller
	 *		should always have the block entries sorted and std::lower_bound() is
	 *		preferred. This is intended only for internal debugging.
	 */
	class CFindGreaterBlockRow_Unordered {
	protected:
		const size_t m_n_row_id; /**< @brief compared block row id */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] n_row_id is the compared block row id
		 */
		CFindGreaterBlockRow_Unordered(size_t n_row_id)
			:m_n_row_id(n_row_id)
		{}

		/**
		 *	@brief block entry row comparison operator
		 *	@apram[in] r_t_block is const reference to a block entry
		 *	@return Returns true if the block lies on a greater row than
		 *		specified in the constructor, otherwise returns false.
		 */
		inline bool operator ()(const TColumn::TBlockEntry &r_t_block) const
		{
			return m_n_row_id < r_t_block.first;
		}
	};

	/**
	 *	@brief comparison between row index in block entry and raw value
	 *
	 *	@note This is similar to CFindLowerRow, but operates on block entries
	 *		TColumn::TBlockEntry instead of row layout TRow.
	 *	@note This is needed for PermuteTo(), even in linux.
	 */
	class CCompareBlockRow {
	public:
		/**
		 *	@brief comparison between row index in block entry and raw value
		 *
		 *	@param[in] r_t_block is block entry (only row index is significant)
		 *	@param[in] n_row_id is (zero-based) row index
		 *
		 *	@return Returns true if row index of r_t_block is smaller than n_row_id,
		 *		otherwise returns false.
		 */
		inline bool operator ()(const TColumn::TBlockEntry &r_t_block, size_t n_row_id) const
		{
			return r_t_block.first < n_row_id;
		}

		/**
		 *	@brief comparison between row index in block entry and raw value
		 *
		 *	@param[in] r_t_block0 is block entry (only row index is significant)
		 *	@param[in] r_t_block1 is block entry (only row index is significant)
		 *
		 *	@return Returns true if row index of r_t_block0 is smaller than
		 *		of r_t_block1, otherwise returns false.
		 */
		inline bool operator ()(const TColumn::TBlockEntry &r_t_block0,
			const TColumn::TBlockEntry &r_t_block1) const
		{
			return r_t_block0.first < r_t_block1.first;
		}

		/**
		 *	@brief comparison between row index in block entry and raw value
		 *
		 *	@param[in] n_row_id is (zero-based) row index
		 *	@param[in] r_t_block is block entry (only row index is significant)
		 *
		 *	@return Returns true if n_row_id is smaller than row index of r_t_block,
		 *		otherwise returns false.
		 */
		inline bool operator ()(size_t n_row_id, const TColumn::TBlockEntry &r_t_block) const
		{
			return n_row_id < r_t_block.first;
		}
	};

	/**
	 *	@brief comparison between row height cumulative sum and raw value
	 *	@note This is similar to CCompareBlockRow, but operates on row layout TRow
	 *		instead of block entries TColumn::TBlockEntry.
	 */
	class CFindLowerRow {
	public:
		/**
		 *	@brief comparison between row height cumulative sum and raw value
		 *
		 *	@param[in] r_t_row is row (only cumulative height sum is significant)
		 *	@param[in] n_row is raw cumulative height sum value
		 *
		 *	@return Returns true if cumulative height sum of r_t_row is smaller than n_row,
		 *		otherwise returns false.
		 */
		inline bool operator ()(const TRow &r_t_row, size_t n_row) const
		{
			return r_t_row.n_cumulative_height_sum < n_row;
		}

		/**
		 *	@brief comparison between row height cumulative sum and raw value
		 *
		 *	@param[in] r_t_row0 is row (only cumulative height sum is significant)
		 *	@param[in] r_t_row1 is row (only cumulative height sum is significant)
		 *
		 *	@return Returns true if cumulative height sum of r_t_row0 is smaller than
		 *		of r_t_row1, otherwise returns false.
		 */
		inline bool operator ()(const TRow &r_t_row0, const TRow &r_t_row1) const
		{
			return r_t_row0.n_cumulative_height_sum < r_t_row1.n_cumulative_height_sum;
		}

		/**
		 *	@brief comparison between row height cumulative sum and raw value
		 *
		 *	@param[in] n_row is raw cumulative height sum value
		 *	@param[in] r_t_row is row (only cumulative height sum is significant)
		 *
		 *	@return Returns true if n_row is smaller than cumulative height sum of r_t_row,
		 *		otherwise returns false.
		 */
		inline bool operator ()(size_t n_row, const TRow &r_t_row) const
		{
			return n_row < r_t_row.n_cumulative_height_sum; // t_odo - why not >=? fix this.
		}
	};

	/**
	 *	@brief comparison between column width cumulative sum and raw value
	 */
	class CFindLowerColumn {
	public:
		/**
		 *	@brief comparison between column width cumulative sum and raw value
		 *
		 *	@param[in] r_t_column is column (only cumulative width sum is significant)
		 *	@param[in] n_column is raw cumulative width sum value
		 *
		 *	@return Returns true if cumulative width sum of r_t_column is smaller than n_column,
		 *		otherwise returns false.
		 */
		inline bool operator ()(const TColumn &r_t_column, size_t n_column) const
		{
			return r_t_column.n_cumulative_width_sum < n_column;
		}

		/**
		 *	@brief comparison between column width cumulative sum and raw value
		 *
		 *	@param[in] r_t_column0 is column (only cumulative width sum is significant)
		 *	@param[in] r_t_column1 is column (only cumulative width sum is significant)
		 *
		 *	@return Returns true if cumulative width sum of r_t_column0 is smaller than
		 *		of r_t_column1, otherwise returns false.
		 */
		inline bool operator ()(const TColumn &r_t_column0, const TColumn &r_t_column1) const
		{
			return r_t_column0.n_cumulative_width_sum < r_t_column1.n_cumulative_width_sum;
		}

		/**
		 *	@brief comparison between column width cumulative sum and raw value
		 *
		 *	@param[in] n_column is raw cumulative width sum value
		 *	@param[in] r_t_column is column (only cumulative width sum is significant)
		 *
		 *	@return Returns true if n_column is smaller than cumulative width sum of r_t_column,
		 *		otherwise returns false.
		 */
		inline bool operator ()(size_t n_column, const TColumn &r_t_column) const
		{
			return n_column < r_t_column.n_cumulative_width_sum; // t_odo - why not >=? fix this.
		}
	};

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
	/**
	 *	@brief implementation of block bounds checks using preinitialization with magic number
	 *	@tparam CPoolType is data pool specialization
	 */
	template <class CPoolType>
	class CBlockBoundsDebugInitializer {
	public:
		typedef typename CPoolType::_Ty _Ty; /**< @brief data type stored by the pool (double) */

		/**
		 *	@brief configuration parameters stored as enum
		 */
		enum {
			blockBoundMarker_Element_Num = 8, /**< @brief number of elements in the block bounds marker */
			memoryAlign_Elements = CPoolType::n_memory_alignment / sizeof(_Ty) /**< @brief pool memory alignment (_Ty elements) */
		};

		/**
		 *	@brief union for initialization of pool elements with magic numbers
		 *	@note This might not be the best idea, although these magic numbers are easy to spot in
		 *		integer, it is possibly not so easy to spot them in floating point representation.
		 */
		union TMagicInitializer {
			uint64_t n_magic_number; /**< @brief a magic number that denotes uninitialized data or block bound area */
			_Ty f_value; /**< @brief a magic number value, interpreted as floating-point type */ // will work for both float and double
		};

		/**
		 *	@brief initializes a new page with magic numbers
		 *
		 *	@param[in] r_pool is the pool with data
		 *	@param[in] n_block_element_num is number of block elements, including alignment
		 *
		 *	@note This function throws std::bad_alloc.
		 */
		static void Init_BeforeBlock(CPoolType &r_pool, size_t n_block_element_num) // throw(std::bad_alloc)
		{
			_ASSERTE(sizeof(uint64_t) >= sizeof(_Ty)); // we use union to initialize the storage

			TMagicInitializer t_uninitialized, t_block_bounds;
			t_uninitialized.n_magic_number = 0xdeadbeefU | (uint64_t(0xdeadbeefU) << 32); // -1.1885959257070704e+148
			t_block_bounds.n_magic_number = 0xbaadf00dU | (uint64_t(0xbaadf00dU) << 32); // -4.8366978272229995e-026
			// "magic" constants; note these must be small enough to modify numerically by addition, otherwise errors will go unnoticed!!

			size_t n_block_bound_size = blockBoundMarker_Element_Num;
			// in case memory alignment is used, fill the unused element(s) with block bound marker as well

			_ASSERTE(blockBoundMarker_Element_Num < r_pool.page_size());
			_ASSERTE(n_block_element_num + 2 * blockBoundMarker_Element_Num <= r_pool.page_size());
			// make sure the bound markers can fit in a page and still leave enough space for the element

			if(r_pool.empty() || r_pool.size() + n_block_element_num +
			   blockBoundMarker_Element_Num > r_pool.capacity()) {
				size_t n_new_page_org = r_pool.capacity();
				_ASSERTE(!n_new_page_org || !r_pool.empty()); // capacity of the empty pool should be 0
				r_pool.resize(n_new_page_org + blockBoundMarker_Element_Num);
				_Ty *p_page = &r_pool[n_new_page_org];
				for(size_t i = 0; i < blockBoundMarker_Element_Num; ++ i)
					p_page[i] = t_block_bounds.f_value; // set the first blockBoundMarker_Element_Num elements to block bounds
				for(size_t i = blockBoundMarker_Element_Num, n = r_pool.page_size(); i < n; ++ i)
					p_page[i] = t_uninitialized.f_value;
			}
			// the new block won't fit on the current page, need to alloc and initialize a new page
			// also put the block bounds before the first block on that page
		}

		/**
		 *	@brief writes block bound magic numbers after a block
		 *
		 *	@param[in] r_pool is the pool with data
		 *	@param[in] n_mem_align_element_num is number of block alignment elements
		 *		(these are a part of the already allocated block, and are not allocated here)
		 *
		 *	@note This function throws std::bad_alloc.
		 */
		static void Init_AfterBlock(CPoolType &r_pool, size_t n_mem_align_element_num) // throw(std::bad_alloc)
		{
			TMagicInitializer t_block_bounds;
			t_block_bounds.n_magic_number = 0xbaadf00dU | (uint64_t(0xbaadf00dU) << 32);
			// "magic" constants

			size_t n_bound_index = r_pool.size();
			_ASSERTE(n_bound_index + blockBoundMarker_Element_Num <= r_pool.capacity());
			// the first block should ensure that the block fits on the same page

			r_pool.resize(n_bound_index + blockBoundMarker_Element_Num);
			_ASSERTE(n_bound_index >= n_mem_align_element_num);
			_Ty *p_page = &r_pool[n_bound_index - n_mem_align_element_num];
			for(size_t i = 0; i < blockBoundMarker_Element_Num + n_mem_align_element_num; ++ i)
				p_page[i] = t_block_bounds.f_value; // set the next blockBoundMarker_Element_Num elements to block bounds
		}

		/**
		 *	@brief checks block bounds markers on a particular block
		 *
		 *	@param[in] r_pool is the pool with data
		 *	@param[in] p_block is the pointer to a data block (points to the first block matrix element)
		 *	@param[in] n_block_size is size of the block, in elements
		 *
		 *	@return Returns true if the block is a valid block and the markers are intact, otherwise returns false.
		 */
		static bool b_CheckBlockMarkers(const CPoolType &r_pool, const _Ty *p_block, size_t n_block_size)
		{
			TMagicInitializer t_block_bounds;
			t_block_bounds.n_magic_number = 0xbaadf00dU | (uint64_t(0xbaadf00dU) << 32);
			// "magic" constants

			size_t n_index = r_pool.index_of(p_block); // index of the first block element
			if(n_index < blockBoundMarker_Element_Num)
				return false; // there is not enough space for the bounds marker; not a valid block
			_ASSERTE(n_index != size_t(-1)); // make sure the block is found
			size_t n_page_org_index = n_index - n_index % r_pool.page_size(); // index of the first page
			if(n_page_org_index + blockBoundMarker_Element_Num > n_index)
				return false; // there is not enough space for the bounds marker; not a valid block
			// check if there is enough space for the bounds marker

			const _Ty *p_before_block = &r_pool[n_index - blockBoundMarker_Element_Num];
			for(size_t i = 0; i < blockBoundMarker_Element_Num; ++ i) {
				if(p_before_block[i] != t_block_bounds.f_value) // todo - is it safe to use floating point comparison? shouldn't we use the union instead?
					return false; // bounds marker overwritten with block contents
			}
			// check the "before" block bounds marker (note that it might be actually longer, if some alignment is applied)

			n_index += n_block_size;
			if(n_index + blockBoundMarker_Element_Num > n_page_org_index + r_pool.page_size() ||
			   n_index + blockBoundMarker_Element_Num > r_pool.size())
				return false; // there is not enough space for the bounds marker; not a valid block
			// make sure there is enough space for the after bounds marker

			const _Ty *p_after_block = &r_pool[n_index];
			for(size_t i = 0; i < blockBoundMarker_Element_Num; ++ i) {
				if(p_after_block[i] != t_block_bounds.f_value) // todo - is it safe to use floating point comparison? shouldn't we use the union instead?
					return false; // bounds marker overwritten with block contents
			}
			// check the "after" block bounds marker

			return true;
		}
	};
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

	/**
	 *	@brief pool allocator wrapper class
	 *	@tparam CPoolType is forward_allocated_pool specialization
	 */
	template <class CPoolType>
	class CDenseAllocator {
	public:
		typedef typename CPoolType::_Ty _Ty; /**< @brief data type stored by the pool (double) */

		/**
		 *	@brief parameters, stored as enums
		 */
		enum {
			n_memory_align = CPoolType::n_memory_alignment, /**< @brief pool memory alignment (bytes) */
			n_memory_align_elem = n_memory_align / sizeof(_Ty) /**< @brief pool memory alignment (_Ty elements) */
		};

	protected:
		CPoolType &m_r_pool; /**< @brief reference to the pool object */

		/**
		 *	@brief workarround for the '>' comparison of n_memory_align with zero inside a template arg
		 */
		enum {
			alignment_Assert_Value = (n_memory_align > 0 && n_memory_align % sizeof(_Ty) == 0) /**< @brief value of the assert condition */
		};

		typedef typename CStaticAssert<alignment_Assert_Value>::MATRIX_POOL_MEMORY_ALIGNMENT_MUST_BE_INTEGER_MULTIPLE_OF_SCALAR_TYPE_SIZE CAssert0; /**< @brief makes sure the alignment is correct */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_pool is reference to the pool object the data will be allocated in
		 */
		inline CDenseAllocator(CPoolType &r_pool)
			:m_r_pool(r_pool)
		{}

		/**
		 *	@brief gets aligned dense storage
		 *	@param[in] n_element_num is number of elements to allocate
		 *	@return Returns pointer (aligned to n_memory_align bytes) to array,
		 *		containing n_element_num elements. The pointer is owned by the
		 *		pool and must not be deleted.
		 *	@note This function throws std::bad_alloc.
		 */
		_Ty *p_Get_DenseStorage(size_t n_element_num) // throw(std::bad_alloc)
		{
			_ASSERTE(n_element_num <= m_r_pool.page_size());
			// makes sure the requested number of elements is possible
			// to be allocated in a contiguous block of memory

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
			size_t n_alignment_size = n_memory_align_elem - 1 - (n_element_num +
				n_memory_align_elem - 1) % n_memory_align_elem;
			size_t n_original_size = n_element_num;
			CBlockBoundsDebugInitializer<CPoolType>::Init_BeforeBlock(m_r_pool, n_element_num + n_alignment_size);
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

			n_element_num += n_memory_align_elem - 1;
			n_element_num -= n_element_num % n_memory_align_elem;
			// adjust (increase) element num so that the total size is integer
			// multiple of n_memory_align (to not disturb page memory alignment)

			_ASSERTE(n_element_num <= m_r_pool.page_size());
			// makes sure the requested number of elements is possible
			// to be allocated in a contiguous block of memory even after the adjustment

			size_t n_size;
			if((n_size = m_r_pool.size()) + n_element_num <= m_r_pool.capacity()) {
				m_r_pool.resize(n_size + n_element_num);
				// add the requested number of elements
			} else {
				m_r_pool.resize((n_size = m_r_pool.capacity()) + n_element_num);
				// in case there is not enough space in the last page, fill the page
				// end with dummy data, then add the requested number of elements
			}

			double *p_result = &m_r_pool[n_size];
			// then we can use ordinary unaligned version

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
			_ASSERTE(n_element_num == n_original_size + n_alignment_size);
			CBlockBoundsDebugInitializer<CPoolType>::Init_AfterBlock(m_r_pool, n_alignment_size);
			_ASSERTE((m_r_pool.size() % m_r_pool.page_size()) % n_memory_align_elem == 0); // make sure the next block will be aligned
			_ASSERTE(CBlockBoundsDebugInitializer<CPoolType>::b_CheckBlockMarkers(m_r_pool,
				p_result, n_original_size)); // make sure the block checks out
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

			_ASSERTE(uintptr_t(&m_r_pool[m_r_pool.capacity() -
				m_r_pool.page_size()]) % n_memory_align == 0); // make sure the last page is aligned
			_ASSERTE(uintptr_t(p_result) % n_memory_align == 0); // make sure the result is aligned
			// debug checks

			return p_result;
		}
	};

	/**
	 *	@brief (specialization for unaligned pools)
	 */
	template <class T, const int _n_page_size_elems>
	class CDenseAllocator<forward_allocated_pool<T, _n_page_size_elems, 0> > {
	public:
		typedef forward_allocated_pool<T, _n_page_size_elems, 0> CPoolType; /**< @brief pool data type */
		typedef typename CPoolType::_Ty _Ty; /**< @brief data type stored by the pool (double) */

		/**
		 *	@brief parameters, stored as enums
		 */
		enum {
			n_memory_align = CPoolType::n_memory_alignment, /**< @brief pool memory alignment (bytes) */
			n_memory_align_elem = n_memory_align / sizeof(_Ty) /**< @brief pool memory alignment (_Ty elements) */
		};

	protected:
		CPoolType &m_r_pool; /**< @brief reference to the pool object */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_pool is reference to the pool object the data will be allocated in
		 */
		inline CDenseAllocator(CPoolType &r_pool)
			:m_r_pool(r_pool)
		{}

		/**
		 *	@brief gets unaligned dense storage
		 *	@param[in] n_element_num is number of elements to allocate
		 *	@return Returns (unaligned) pointer to array, containing n_element_num elements.
		 *		The pointer is owned by the pool and must not be deleted.
		 *	@note This function throws std::bad_alloc.
		 */
		_Ty *p_Get_DenseStorage(size_t n_element_num) // throw(std::bad_alloc)
		{
			_ASSERTE(n_element_num <= m_r_pool.page_size());
			// makes sure the requested number of elements is possible
			// to be allocated in a contiguous block of memory

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
			CBlockBoundsDebugInitializer<CPoolType>::Init_BeforeBlock(m_r_pool, n_element_num);
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

			size_t n_size;
			if((n_size = m_r_pool.size()) + n_element_num <= m_r_pool.capacity()) {
				m_r_pool.resize(n_size + n_element_num);
				// add the requested number of elements
			} else {
				m_r_pool.resize((n_size = m_r_pool.capacity()) + n_element_num);
				// in case there is not enough space in the last page, fill the page
				// end with dummy data, then add the requested number of elements
			}

#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
			CBlockBoundsDebugInitializer<CPoolType>::Init_AfterBlock(m_r_pool, 0);
			_ASSERTE(CBlockBoundsDebugInitializer<CPoolType>::b_CheckBlockMarkers(m_r_pool,
				&m_r_pool[n_size], n_element_num)); // make sure the block checks out
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

			return &m_r_pool[n_size];
		}
	};

	typedef CDenseAllocator<_TyPool> _TyDenseAllocator; /**< memory allocator wrapper for the pool */
#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
	typedef CBlockBoundsDebugInitializer<_TyPool> _TyBoundsCheck; /**< block bounds checking object */
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

protected:
	/**
	 *	@brief a simple function object used for updating row references after row fragmentation
	 *	@tparam n_increment is number to increment row indices by (1 or 2)
	 */
	template <const int n_increment>
	class CIncrementRowIndex {
	protected:
		const size_t m_n_row; /**< @brief threshold row above (and including) which the indices are incremented */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] n_row is threshold row above (and including) which the indices should be incremented
		 */
		inline CIncrementRowIndex(size_t n_row)
			:m_n_row(n_row)
		{}

		/**
		 *	@brief performs the selective increment
		 *	@param[in] r_t_block is block where row references are to be updated
		 */
		inline void operator ()(TColumn::TBlockEntry &r_t_block) const
		{
			if(r_t_block.first >= m_n_row)
				r_t_block.first += n_increment;
			// in case the block references a column after
			// a newly inserted one(s), increment the column index
		}
	};

	/**
	 *	@brief a simple predicate comparing block row to reference value
	 */
	class CReferencesRow {
	protected:
		const size_t m_n_row; /**< @brief reference value */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] n_row is reference value
		 */
		inline CReferencesRow(size_t n_row)
			:m_n_row(n_row)
		{}

		/**
		 *	@brief compares block row to reference value
		 *	@param[in] r_t_block is block
		 *	@return Returns true if block references the selected row, otherwise returns false.
		 */
		inline bool operator ()(const TColumn::TBlockEntry &r_t_block) const
		{
			return r_t_block.first == m_n_row;
		}
	};

	/**
	 *	@brief a simple function object for converting raw cumulative sum layout to row layout
	 */
	class CTransformCumsumToRows {
	protected:
		size_t m_n_last_coord; /**< @brief last cumulative sum value */

	public:
		/**
		 *	@brief default constructor
		 */
		inline CTransformCumsumToRows()
			:m_n_last_coord(0)
		{}

		/**
		 *	@brief consumes one cumulative sum and produces a block-row
		 *	@param[in] n_cumsum is cumulative sum
		 *	@return Returns block-row, corresponding to the cumulative sum.
		 */
		inline TRow operator ()(size_t n_cumsum)
		{
			TRow t_row;
			t_row.n_height = n_cumsum - m_n_last_coord;
			t_row.n_cumulative_height_sum = m_n_last_coord;
			m_n_last_coord = n_cumsum;
			return t_row;
		}
	};

	/**
	 *	@brief a simple function object for converting raw cumulative sum layout to column layout
	 */
	class CTransformCumsumToColumns {
	protected:
		size_t m_n_last_coord; /**< @brief last cumulative sum value */

	public:
		/**
		 *	@brief default constructor
		 */
		inline CTransformCumsumToColumns()
			:m_n_last_coord(0)
		{}

		/**
		 *	@brief consumes one cumulative sum and produces a block-column
		 *	@param[in] n_cumsum is cumulative sum
		 *	@return Returns block-column, corresponding to the cumulative sum.
		 */
		inline TColumn operator ()(size_t n_cumsum)
		{
			TColumn t_col;
			t_col.n_width = n_cumsum - m_n_last_coord;
			t_col.n_cumulative_width_sum = m_n_last_coord;
			m_n_last_coord = n_cumsum;
			return t_col;
		}
	};

	/**
	 *	@brief a simple function object for elementwise multiplication
	 *		of matrix elements by a scalar factor
	 */
	class CScaleBy {
	protected:
		const double m_f_coeff; /**< @brief scalar factor */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] f_coeff is scalar factor the elements are to be multiplied by
		 */
		inline CScaleBy(double f_coeff)
			:m_f_coeff(f_coeff)
		{}

		/**
		 *	@brief performs elementwise multiplication of matrix data
		 *	@param[in] f_scalar is matrix element
		 *	@return Returns product of input value and the factor supplied to constructor.
		 */
		inline double operator ()(double f_scalar) const
		{
			return f_scalar * m_f_coeff;
		}
	};

	/**
	 *	@brief a simple function object for elementwise addition
	 *		of matrix elements with right-side scalar factor
	 */
	class CAddWeighted {
	protected:
		const double m_f_factor; /**< @brief right-side scalar factor */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] f_factor is scalar factor the right-side elements are to be multiplied by
		 */
		inline CAddWeighted(double f_factor)
			:m_f_factor(f_factor)
		{}

		/**
		 *	@brief performs elementwise addition of matrix data with right-side scaling
		 *	@param[in] f_a is left-side matrix element
		 *	@param[in] f_b is right-side matrix element
		 *	@return Returns sum of left-side input with product of right-side
		 *		input and the factor supplied to constructor.
		 */
		inline double operator ()(double f_a, double f_b) const
		{
			return f_a + f_b * m_f_factor;
		}
	};

	/**
	 *	@brief a simple function object for elementwise addition
	 *		of matrix elements with scalar factors for either side
	 */
	class CWeightedAddWeighted {
	protected:
		const double m_f_factor_dest; /**< @brief left-side scalar factor */
		const double m_f_factor_src; /**< @brief right-side scalar factor */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] f_factor_dest is scalar factor the left-side elements are to be multiplied by
		 *	@param[in] f_factor_src is scalar factor the right-side elements are to be multiplied by
		 */
		inline CWeightedAddWeighted(double f_factor_dest, double f_factor_src)
			:m_f_factor_dest(f_factor_dest), m_f_factor_src(f_factor_src)
		{}

		/**
		 *	@brief performs elementwise addition of matrix data with scaling
		 *	@param[in] f_a is left-side matrix element
		 *	@param[in] f_b is right-side matrix element
		 *	@return Returns sum of products of inputs with their respective
		 *		factors supplied to constructor.
		 */
		inline double operator ()(double f_a, double f_b) const
		{
			return f_a * m_f_factor_dest + f_b * m_f_factor_src;
		}
	};

	/**
	 *	@brief a simple function object for blockwise inversion
	 */
	class CInvertBlock {
	public:
		/**
		 *	@brief performs dense matrix block inversion
		 *	@param[out] r_dest is destination block
		 *	@param[in] r_src is source block (must not be the same as source)
		 */
		template <class _TyMatrix>
		static inline void Do(_TyMatrix &r_dest, const _TyMatrix &r_src)
		{
			_ASSERTE(&r_dest != &r_src); // noalias()
			r_dest = r_src.inverse();
		}
	};

protected:
	size_t m_n_row_num; /**< @brief number of matrix rows, in elements */
	size_t m_n_col_num; /**< @brief number of matrix columns, in elements */
	std::vector<TRow> m_block_rows_list; /**< @brief list of sizes of blocks rows (sum of all the sizes equals m_n_row_num) */
	std::vector<TColumn> m_block_cols_list; /**< @brief list of block columns (sum of all the sizes equals m_n_col_num) */

	_TyPool m_data_pool; /**< @brief data storage for matrix elements */
	size_t m_n_ref_elem_num; /**< @brief number of referenced elements (in shallow copies) */

	//CTimer m_timer; // profiling

#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
	size_t m_n_row_reindex_num; /**< @brief row reindex performance counter (incremented everytime a row is fragmented) */
	size_t m_n_rows_list_shift_num; /**< @brief row list shift performance counter (incremented everytime a row is inserted in the middle of the list) */
	size_t m_n_cols_list_shift_num; /**< @brief column list shift performance counter (incremented everytime a column is inserted in the middle of the list) */
	size_t m_n_rows_list_realloc_num; /**< @brief row list reallocation performance counter (incremented everytime row list size reaches its capacity) */
	size_t m_n_cols_list_realloc_num; /**< @brief column list reallocation performance counter (incremented everytime column list size reaches its capacity) */
	size_t m_n_dummy_row_num; /**< @brief dummy row performance counter (incremented everytime a dummy row is created) */
	size_t m_n_dummy_col_num; /**< @brief dummy column performance counter (incremented everytime a dummy column is created) */
	mutable size_t m_n_row_reref_num; /**< @brief row re-reference performance counter (incremented everytime an existing row is referenced) */
	mutable size_t m_n_col_reref_num; /**< @brief column re-reference performance counter (incremented everytime an existing column is referenced) */
	size_t m_n_row_append_num; /**< @brief row append performance counter (incremented everytime a row is inserted at the end of the list) */
	size_t m_n_col_append_num; /**< @brief column append performance counter (incremented everytime a column is inserted at the end of the list) */
	mutable size_t m_n_col_block_search_num; /**< @brief column block search performance counter (incremented everytime search is performed on the block list inside a column) */
	mutable size_t m_n_col_block_reref_num; /**< @brief column block re-reference performance counter (incremented everytime column block is re-referenced) */
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS

public:
	/**
	 *	@brief constructor with structure specification
	 *
	 *	This takes arrays of cumsums - as many entries as blocks, the first is the width (height),
	 *	of the first block, the last is the width (height) of the matrix.
	 *
	 *	@tparam CIterator is iterator data type
	 *	@param[in] p_rows_cumsum_begin is iterator pointing to the first row cumulative sum
	 *	@param[in] p_rows_cumsum_end is iterator pointing one past the last row cumulative sum
	 *	@param[in] p_columns_cumsum_begin is iterator pointing to the first column cumulative sum
	 *	@param[in] p_columns_cumsum_end is iterator pointing one past the last column cumulative sum
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note It is often beneficial to only supply the row structure to avoid reindexing
	 *		and leave column structure empty.
	 */
	template <class CIterator>
	inline CUberBlockMatrix_Base(CIterator p_rows_cumsum_begin, CIterator p_rows_cumsum_end,
		CIterator p_columns_cumsum_begin, CIterator p_columns_cumsum_end) // throw(std::bad_alloc)
		:m_n_row_num(0), m_n_col_num(0), m_n_ref_elem_num(0)
	{
		Reset_Perfcounters();

		m_block_rows_list.resize(p_rows_cumsum_end - p_rows_cumsum_begin);
		std::transform(p_rows_cumsum_begin, p_rows_cumsum_end,
			m_block_rows_list.begin(), CTransformCumsumToRows());
		m_block_cols_list.resize(p_columns_cumsum_end - p_columns_cumsum_begin);
		std::transform(p_columns_cumsum_begin, p_columns_cumsum_end,
			m_block_cols_list.begin(), CTransformCumsumToColumns());
		// copy row / col cumsums // t_odo - use std::transform to make this more elegant

		m_n_row_num = (p_rows_cumsum_begin == p_rows_cumsum_end)? 0 : *(-- p_rows_cumsum_end);
		m_n_col_num = (p_columns_cumsum_begin == p_columns_cumsum_end)?
			0 : *(-- p_columns_cumsum_end);
		// set dimensions

		CheckIntegrity();
		// make sure the matrix is ok
	}

	/**
	 *	@brief constructor with structure specification
	 *
	 *	This takes arrays of cumsums - as many entries as blocks, the first is the height,
	 *	of the first block, the last is the height of the matrix.
	 *
	 *	@tparam CIterator is iterator data type
	 *	@param[in] p_rows_cumsum_begin is iterator pointing to the first row cumulative sum
	 *	@param[in] p_rows_cumsum_end is iterator pointing one past the last row cumulative sum
	 *	@param[in] n_column_block_num is number of column blocks (to avoid reallocation, can be
	 *		an estimate - the number of columns of the matrix is set to 0 regardless of the value)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This seems to be the fastest way in UFSMC benchmarks, even on "good" matrices
	 *		(matrices with no dummy rows).
	 */
	template <class CIterator>
	inline CUberBlockMatrix_Base(CIterator p_rows_cumsum_begin,
		CIterator p_rows_cumsum_end, size_t n_column_block_num) // throw(std::bad_alloc)
		:m_n_row_num(0), m_n_col_num(0), m_n_ref_elem_num(0)
	{
		Reset_Perfcounters();

		m_block_rows_list.resize(p_rows_cumsum_end - p_rows_cumsum_begin);
		std::transform(p_rows_cumsum_begin, p_rows_cumsum_end,
			m_block_rows_list.begin(), CTransformCumsumToRows());
		// copy row cumsums

		m_block_cols_list.reserve(n_column_block_num);
		// prealloc columns

		m_n_row_num = (p_rows_cumsum_begin == p_rows_cumsum_end)? 0 : *(-- p_rows_cumsum_end);
		// set dimensions

		CheckIntegrity();
		// make sure the matrix is ok
	}

	/**
	 *	@brief default constructor
	 *
	 *	@param[in] n_target_row_block_num is number of row blocks (to avoid reallocation, can be
	 *		an estimate - the number of rows of the matrix is set to 0 regardless of the value)
	 *	@param[in] n_target_column_block_num is number of column blocks (to avoid reallocation, can be
	 *		an estimate - the number of columns of the matrix is set to 0 regardless of the value)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline CUberBlockMatrix_Base(size_t n_target_row_block_num = 0,
		size_t n_target_column_block_num = 0) // throw(std::bad_alloc)
		:m_n_row_num(0), m_n_col_num(0), m_n_ref_elem_num(0)
	{
		Reset_Perfcounters();

		if(n_target_row_block_num)
			m_block_rows_list.reserve(n_target_row_block_num);
		if(n_target_column_block_num)
			m_block_cols_list.reserve(n_target_column_block_num);
		// prealloc rows / columns

		// t_odo - still a sizable portion of time is likely spent in moving the list when adding new items - need to add some perfcounters
	}

	/**
	 *	@brief copy-constructor
	 *	@param[in] r_matrix is matrix to copy from
	 *	@note This function throws std::bad_alloc.
	 */
	inline CUberBlockMatrix_Base(const CUberBlockMatrix_Base &r_matrix)
		:m_n_row_num(0), m_n_col_num(0), m_n_ref_elem_num(0)
	{
		r_matrix.CopyTo(*this);
	}

	/**
	 *	@brief copy-operator
	 *	@param[in] r_matrix is matrix to copy from
	 *	@return Returns reference to this.
	 *	@note This function throws std::bad_alloc.
	 */
	inline CUberBlockMatrix_Base &operator =(const CUberBlockMatrix_Base &r_matrix)
	{
		r_matrix.CopyTo(*this);
		return *this;
	}

#ifdef _DEBUG
	/**
	 *	@brief destructor; checks the integrity of the matrix that is being destroyed
	 */
	inline ~CUberBlockMatrix_Base()
	{
		CheckIntegrity(true);
		// important - make sure than any malformed matrix goes unnoticed
	}
#endif // _DEBUG

	/**
	 *	@brief prints values of performance counters to stdout
	 *	@note This only has effect if the __UBER_BLOCK_MATRIX_PERFCOUNTERS macro is defined,
	 *		otherwise a line saying "performance counters not compiled" is displayed.
	 */
	void Dump_PerfCounters();

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const;

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 *	@note This is similar to \ref n_Allocation_Size(), except that this function discounts
	 *		the unused space in the last page of the elements storage.
	 */
	size_t n_Allocation_Size_NoLastPage() const;

	/**
	 *	@brief erases all the data of the matrix
	 */
	void Clear();

	/**
	 *	@brief swaps two matrices
	 *	@param[in,out] r_other is matrix to swap with
	 *	@note This doesn't involve any loops / data copies, (amortized) time complexity is O(1).
	 */
	void Swap(CUberBlockMatrix_Base &r_other);

	/**
	 *	@brief copies the matrix
	 *
	 *	@param[out] r_dest is destination matrix (will be overwritten)
	 *	@param[in] b_share_data is data sharing flag (if set, block data in the destination
	 *		matrix will point to this matrix pool; if new blocks are added to the copy
	 *		or the original(this), these are not mirrored in the other matrix)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	void CopyTo(CUberBlockMatrix_Base &r_dest, bool b_share_data = false) const; // throw(std::bad_alloc)

	/**
	 *	@brief resets all performance counters to zero
	 *	@note This only has effect if the __UBER_BLOCK_MATRIX_PERFCOUNTERS
	 *		macro is defined, otherwise it's an empty function.
	 */
	inline void Reset_Perfcounters()
	{
#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
		m_n_row_reindex_num = 0;
		m_n_rows_list_shift_num = 0;
		m_n_cols_list_shift_num = 0;
		m_n_rows_list_realloc_num = 0;
		m_n_cols_list_realloc_num = 0;
		m_n_dummy_row_num = 0;
		m_n_dummy_col_num = 0;
		m_n_row_reref_num = 0;
		m_n_col_reref_num = 0;
		m_n_row_append_num = 0;
		m_n_col_append_num = 0;
		m_n_col_block_search_num = 0;
		m_n_col_block_reref_num = 0;
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS
	}

	/**
	 *	@brief checks matrix integrity
	 *	@param[in] b_full_check is full check flag (if set, performs also checks that
	 *		require more than O(n) time, otherwise skips them)
	 *	@note This only takes effect in debug builds, otherwise it's an empty function.
	 */
	inline void CheckIntegrity(bool UNUSED(b_full_check) = false) const
	{
#if defined(_DEBUG) && !defined(__UBER_BLOCK_MATRIX_DISABLE_INTEGRITY_CHECK)
		_ASSERTE(m_block_cols_list.empty() || m_block_cols_list.front().n_cumulative_width_sum == 0);
		for(size_t i = 0, n = m_block_cols_list.size(), n_cum_sum = 0; i < n; ++ i) {
			const TColumn &r_t_col = m_block_cols_list[i];
			_ASSERTE(r_t_col.n_cumulative_width_sum == n_cum_sum);
			_ASSERTE(r_t_col.n_width > 0);
			n_cum_sum += r_t_col.n_width;
		}
		_ASSERTE((m_block_cols_list.empty() && !m_n_col_num) || (!m_block_cols_list.empty() &&
			m_n_col_num == m_block_cols_list.back().n_cumulative_width_sum + m_block_cols_list.back().n_width));
		// check column cumulative sum integrity

		for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
			const TColumn &r_t_col = m_block_cols_list[i];

			//std::set<size_t> row_ref_set;
			size_t n_prev_row;
			for(size_t j = 0, m = r_t_col.block_list.size(); j < m; ++ j) {
				_ASSERTE(r_t_col.block_list[j].second); // memory is allocated
				size_t n_row = r_t_col.block_list[j].first;
				_ASSERTE(n_row < m_block_rows_list.size()); // make sure row index is valid
				//_ASSERTE(row_ref_set.find(n_row) == row_ref_set.end()); // make sure a single row is only referenced up to once in a single column
				//row_ref_set.insert(n_row);
				_ASSERTE(!j || n_row > n_prev_row); // make sure the blocks come in increasing order // f_ixme - in case blocks are forced to come in strictly increasing order, do i still need the row reference set? // no!
				n_prev_row = n_row;
			}
		}
		// check column row references

		_ASSERTE(m_block_rows_list.empty() || m_block_rows_list.front().n_cumulative_height_sum == 0);
		for(size_t i = 0, n = m_block_rows_list.size(), n_cum_sum = 0; i < n; ++ i) {
			const TRow &r_t_row = m_block_rows_list[i];
			_ASSERTE(r_t_row.n_cumulative_height_sum == n_cum_sum);
			_ASSERTE(r_t_row.n_height > 0);
			n_cum_sum += r_t_row.n_height;
		}
		_ASSERTE((m_block_rows_list.empty() && !m_n_row_num) || (!m_block_rows_list.empty() &&
			m_n_row_num == m_block_rows_list.back().n_cumulative_height_sum + m_block_rows_list.back().n_height));
		// check row cumulative sum integrity

		if(b_full_check) {
#if defined(_DEBUG) && defined(__UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK)
			for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
				const TColumn &r_t_col = m_block_cols_list[i];
				size_t n_width = r_t_col.n_width;
				for(size_t j = 0, m = r_t_col.block_list.size(); j < m; ++ j) {
					const double *p_block = r_t_col.block_list[j].second;
					size_t n_row = r_t_col.block_list[j].first;
					size_t n_height = m_block_rows_list[n_row].n_height;
					if(m_n_ref_elem_num > 0 && m_data_pool.index_of(p_block) == size_t(-1))
						continue; // cannot check referenced blocks (would have to keep track of references); the original matrix can still be checked thouhg (and it will detect errors if any)
					_ASSERTE(_TyBoundsCheck::b_CheckBlockMarkers(m_data_pool, p_block, n_width * n_height)); // make sure noone overwritten block bound markers
				}
			}
			// check column row references
#endif // _DEBUG && __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK

			Check_Block_Alignment();
			// that too
		} else
			Check_ModuleSettings(); // this is pretty cheap
#endif // _DEBUG && !__UBER_BLOCK_MATRIX_DISABLE_INTEGRITY_CHECK
	}

	/**
	 *	@brief makes sure that the block matrix module was build with the same
	 *		settings as the calling code
	 *
	 *	This is aimed towards checking for differences that would cause binary
	 *	incompatibility, such as accessing data with different alignement and such.
	 *
	 *	If errors are detected, a message is also printed to stdout.
	 *
	 *	@param[in] n_pool_alignment is alignment of block data, in bytes
	 *	@param[in] n_map_alignment is Eigen matrix alignment enum (e.g. \ref Eigen::Align or \ref Eigen::Align16)
	 *	@param[in] b_debug_markers is debug markers flag
	 *
	 *	@return Returns true if no differences are detected, otherwise returns false.
	 */
	static bool Check_ModuleSettings(int n_pool_alignment, int n_map_alignment, bool b_debug_markers);

	/**
	 *	@brief makes sure that the block matrix module was build with the same
	 *		settings as the calling code
	 *
	 *	This is aimed towards checking for differences that would cause binary
	 *	incompatibility, such as accessing data with different alignement and such.
	 *
	 *	If errors are detected, a message is also printed to stderr.
	 *
	 *	@return Returns true if no differences are detected, otherwise returns false.
	 */
	static bool Check_ModuleSettings() // do not move the definition to a .cpp, it needs to stay in the header
	{
		return Check_ModuleSettings(pool_MemoryAlignment, map_Alignment,
#ifdef __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK
			true
#else // __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK
			false
#endif // __UBER_BLOCK_MATRIX_BUFFER_BOUNDS_CHECK
			);
		// perform the check with current settings
	}

	/**
	 *	@brief gets (aligned) dense storage
	 *
	 *	@param[in] n_element_num is number of elements to allocate
	 *
	 *	@return Returns (aligned) pointer to array, containing n_element_num elements.
	 *		The pointer is owned by m_data_pool, and must not be deleted.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note The memory alignment is given by pool_MemoryAlignment, and the returned
	 *		pointers are, in fact, unaligned in case it equals to zero.
	 */
	inline double *p_Get_DenseStorage(size_t n_element_num) // throw(std::bad_alloc)
	{
		_TyDenseAllocator alloc(m_data_pool);
		return alloc.p_Get_DenseStorage(n_element_num);
		// use the allocator class, but only instantiate it as needed so it is possible to swap matrices
	}

	/**
	 *	@brief checks matrix block alignment; available in release builds as well
	 *	@return Returns true in case there are no alignment issues, otherwise returns false.
	 *	@note This applies the alignment rules of the currently built module which might
	 *		be different from how the SLAM++ base library was build, in case a build
	 *		configuration mismatch occured.
	 */
	bool Check_Block_Alignment() const
	{
		bool b_result = Check_ModuleSettings();

		if(!pool_MemoryAlignment)
			return Check_Block_Alignment_ModuleCfg() && b_result; // unaligned addresses allowed

		enum {
			n_real_align = (pool_MemoryAlignment)? pool_MemoryAlignment : 1 // avoid "potential mod by 0" warning
		};

		int n_max_output = 0;
		for(size_t i = 0, n = m_block_cols_list.size(); i < n; ++ i) {
			for(size_t j = 0, m = m_block_cols_list[i].block_list.size(); j < m; ++ j) {
				double *p_block = m_block_cols_list[i].block_list[j].second;
				if(ptrdiff_t(p_block) % n_real_align != 0) {
					fprintf(stderr, "error: have a misaligned block at: 0x%08x\n",
						(unsigned int)(ptrdiff_t(p_block) & 0xffffffffU));
					b_result = false;
					if(++ n_max_output == 100) {
						fprintf(stderr, "error: the number of misaligned blocks "
							"exceeded %d; stopping the check\n", n_max_output);
						break;
					}
				}
			}
		}

		return b_result && Check_Block_Alignment_ModuleCfg();
	}

protected:
	/**
	 *	@brief checks matrix block alignment; available in release builds as well
	 *	@return Returns true in case there are no alignment issues, otherwise returns false.
	 *	@note This applies the alignment rules with which the SLAM++ base library was build
	 *		that might be different from that of the currently built module, in case a build
	 *		configuration mismatch occured.
	 */
	bool Check_Block_Alignment_ModuleCfg() const;
};

} // ~blockmatrix_detail

/**
 *	@brief checks dimension of a matrix against a reference
 *
 *	This asserts that the dimensions of the checked matrix matches that of the reference,
 *	and if possible does it at compile time. Nothing else than the dimensions are checked.
 *
 *	@tparam CReferenceMat is reference Eigen::Matrix specialization with constant dimensions
 *	@tparam CCheckedMat is checked Eigen::Matrix specialization
 *
 *	@param[in] r_matrix is the checked matrix
 */
template <class CReferenceMat, class CCheckedMat>
inline void DimensionCheck(const CCheckedMat &r_matrix)
{
	enum {
		n_correct_rows = CReferenceMat::RowsAtCompileTime,
		n_correct_cols = CReferenceMat::ColsAtCompileTime,

		b_ref_is_not_dynamic = n_correct_rows != Eigen::Dynamic && n_correct_cols != Eigen::Dynamic,

		n_checked_rows = CCheckedMat::RowsAtCompileTime,
		n_checked_cols = CCheckedMat::ColsAtCompileTime,

		b_checked_is_dynamic = int(n_checked_rows) == int(Eigen::Dynamic) || int(n_checked_cols) == int(Eigen::Dynamic),
		b_checked_matches = int(n_correct_rows) == int(n_checked_rows) && int(n_correct_cols) == int(n_checked_cols)
		// avoid g++ enum comparison warning
	};

	typedef typename blockmatrix_detail::CStaticAssert<b_ref_is_not_dynamic>::MATRIX_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME CAssert0; // reference type must not be dynamic
	typedef typename blockmatrix_detail::CStaticAssert<b_checked_is_dynamic || b_checked_matches>::MATRIX_DIMENSIONS_MISMATCH CAssert1; // checked type either matches at compile time, or is dynamic

	_ASSERTE(!b_checked_is_dynamic || (r_matrix.rows() == n_correct_rows && r_matrix.cols() == n_correct_cols)); // in case checked is not dynamic, this reduces to a compile time constant
	// in case the matrix is dynamic-sized, check at runtime
}

/**
 *	@brief derives a fixed-size double-precision matrix type from a derived type
 *
 *	@tparam Derived is Eigen derived matrix type (as in <tt>Eigen::MatrixBase<Derived></tt> function argument)
 *	@tparam b_allow_dynamic is dynamic matrix type flag (if set, dynamic size matrices are allowed, otherwise they trigger compile assertion; default false)
 */
template <class Derived, bool b_allow_dynamic = false>
class CDeriveMatrixType {
protected:
	enum {
		n_rows = Eigen::MatrixBase<Derived>::RowsAtCompileTime,
		n_cols = Eigen::MatrixBase<Derived>::ColsAtCompileTime,

		b_derived_is_not_dynamic = b_allow_dynamic || (n_rows != Eigen::Dynamic && n_cols != Eigen::Dynamic),
	};

	typedef typename blockmatrix_detail::CStaticAssert<b_derived_is_not_dynamic>::MATRIX_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME CAssert0; // the derived type must not be dynamic

public:
	typedef Eigen::Matrix<double, n_rows, n_cols> _TyResult; /**< @brief fixed-size double-precision matrix of the same size as Derived */
};

namespace blockmatrix_detail {

/**
 *	@brief utility class to convert a vector type to a reference to it
 *	@tparam CRefType is a map or reference type to convert to
 */
template <class CRefType>
class CMakeVectorRef {
public:
	/**
	 *	@brief makes a reference to a vector
	 *	@tparam CInputType is input vector type
	 *	@param[in] r_vector is reference to the input vector
	 *	@return Returns a reference to the input vector.
	 */
	template <class CInputType>
	static CRefType Ref(CInputType &r_vector)
	{
		return static_cast<CRefType>(r_vector); // this is for casting to a vector. do not allow making a map or a vector of a different type out of it
	}

	/**
	 *	@brief makes a const reference to a vector
	 *	@tparam CInputType is input vector type
	 *	@param[in] r_vector is reference to the input vector
	 *	@return Returns a const reference to the input vector.
	 */
	template <class CInputType>
	static CRefType ConstRef(const CInputType &r_vector)
	{
		return static_cast<CRefType>(r_vector); // this is for casting to a vector. do not allow making a map or a vector of a different type out of it
	}
};

/**
 *	@brief utility class to convert a vector type to \ref Eigen::Map
 *
 *	@tparam CVecType is the first template parameter of the resulting \ref Eigen::Map
 *	@tparam n_options is the second template parameter of the resulting \ref Eigen::Map
 */
template <class CVecType, int n_options>
class CMakeVectorRef<Eigen::Map<CVecType, n_options> > {
public:
	typedef Eigen::Map<CVecType, n_options> _TyRef; /**< @brief type of the resulting map */

public:
	/**
	 *	@brief makes a reference to a vector (specialization for dynamic size)
	 *	@tparam CInputType is input vector type
	 *	@param[in] r_vector is reference to the input vector
	 *	@return Returns a map wrapping the input vector.
	 */
	template <class CInputType>
	static typename CEnableIf<CInputType::RowsAtCompileTime == Eigen::Dynamic,
		_TyRef>::T Ref(CInputType &r_vector)
	{
		return _TyRef(r_vector.data(), r_vector.rows());
	}

	/**
	 *	@brief makes a reference to a vector (specialization for compile time constant size)
	 *	@tparam CInputType is input vector type
	 *	@param[in] r_vector is reference to the input vector
	 *	@return Returns a map wrapping the input vector.
	 */
	template <class CInputType>
	static typename CEnableIf<CInputType::RowsAtCompileTime != Eigen::Dynamic,
		_TyRef>::T Ref(CInputType &r_vector)
	{
		return _TyRef(r_vector.data(), CInputType::RowsAtCompileTime);
	}

	/**
	 *	@brief makes a const reference to a vector (specialization for dynamic size)
	 *	@tparam CInputType is input vector type
	 *	@param[in] r_vector is reference to the input vector
	 *	@return Returns a const map wrapping the input vector.
	 */
	template <class CInputType>
	static typename CEnableIf<CInputType::RowsAtCompileTime == Eigen::Dynamic,
		_TyRef>::T ConstRef(const CInputType &r_vector)
	{
		return _TyRef(r_vector.data(), r_vector.rows());
	}

	/**
	 *	@brief makes a const reference to a vector (specialization for compile time constant size)
	 *	@tparam CInputType is input vector type
	 *	@param[in] r_vector is reference to the input vector
	 *	@return Returns a const map wrapping the input vector.
	 */
	template <class CInputType>
	static typename CEnableIf<CInputType::RowsAtCompileTime != Eigen::Dynamic,
		_TyRef>::T ConstRef(const CInputType &r_vector)
	{
		return _TyRef(r_vector.data(), CInputType::RowsAtCompileTime);
	}
};

} // ~blockmatrix_detail

/**
 *	@brief converts a vector to a const map or a const reference to it
 *
 *	@tparam CRefType is a map or reference type to convert to
 *	@tparam CInputType is input vector type
 *
 *	@param[in] r_vector is reference to the input vector
 *
 *	@return Returns a const reference or a const map wrapping the input vector.
 *
 *	@note This facilitates simple conversion between maps of different types
 *		(but the same size) or "conversion" of vectors to maps.
 */
template <class CRefType, class CInputType>
inline CRefType t_MakeVectorRef(const CInputType &r_vector)
{
	return blockmatrix_detail::CMakeVectorRef<CRefType>::ConstRef(r_vector);
}

/**
 *	@brief converts a vector to a map or a reference to it
 *
 *	@tparam CRefType is a map or reference type to convert to
 *	@tparam CInputType is input vector type
 *
 *	@param[in] r_vector is reference to the input vector
 *
 *	@return Returns a reference or a map wrapping the input vector.
 *
 *	@note This facilitates simple conversion between maps of different types
 *		(but the same size) or "conversion" of vectors to maps.
 */
template <class CRefType, class CInputType>
inline CRefType t_MakeVectorRef(CInputType &r_vector)
{
	return blockmatrix_detail::CMakeVectorRef<CRefType>::Ref(r_vector);
}

/** @} */ // end of group

#endif // !__UBER_BLOCK_MATRIX_BASE_INCLUDED
