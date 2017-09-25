/*
								+---------------------------------+
								|                                 |
								|  ***   Über Block Matrix   ***  |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2012 |
								|                                 |
								|      BlockMatrixFBSUtil.h       |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_OPS_UTILITIES_INCLUDED
#define __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_OPS_UTILITIES_INCLUDED

/**
 *	@file include/slam/BlockMatrixFBSUtil.h
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief the überblockmatrix fixed block size utility classes
 *	@note This file is not to be included; it is automatically included from BlockMatrix.h
 */

#include "slam/BlockMatrixBase.h"

/** \addtogroup ubm
 *	@{
 */

/**
 *	@brief fixed block size utility classes and functions
 */
namespace fbs_ut {

/**
 *	@brief compile-time constant booelan
 *	@tparam _n_size is binary flag
 */
template <bool _b_flag>
class CCTFlag {
public:
	/**
	 *	@brief copy of template parameters as enums
	 */
	enum {
		b_flag = _b_flag /**< @brief the flag value */
	};
};

/**
 *	@brief compile-time constant scalar
 *	@tparam _n_size is scalar size
 */
template <int _n_size>
class CCTSize {
public:
	/**
	 *	@brief copy of template parameters as enums
	 */
	enum {
		n_size = _n_size /**< @brief size */
	};
};

/**
 *	@brief compile-time constant 2D vector
 *
 *	@tparam _n_row_num is number of rows (y component)
 *	@tparam _n_column_num is number of columns (x component)
 */
template <int _n_row_num, int _n_column_num>
class CCTSize2D {
public:
	/**
	 *	@brief copy of template parameters as enums
	 */
	enum {
		n_row_num = _n_row_num, /**< @brief number of rows */
		n_column_num = _n_column_num /**< @brief number of columns */
	};
};

/**
 *	@brief binary addition operating on scalars
 *
 *	@tparam _T1 is the first operand (specialization of CCTSize template)
 *	@tparam _T2 is the second operand (specialization of CCTSize template)
 */
template <class _T1, class _T2>
class CBinaryScalarAdd {
public:
	typedef CCTSize<_T1::n_size + _T2::n_size> _TyResult; /**< @brief result of the addition */
};

/**
 *	@brief binary maximum of two compile-time scalars
 *
 *	@tparam _T1 is the first operand (specialization of CCTSize template)
 *	@tparam _T2 is the second operand (specialization of CCTSize template)
 */
template <class _T1, class _T2>
class CBinaryScalarMax {
protected:
	/**
	 *	@brief intermediates stored as an enum
	 */
	enum {
		s0 = _T1::n_size, /**< @brief value of the first scalar */
		s1 = _T2::n_size, /**< @brief value of the second scalar */
		n_result = (size_t(s0) > size_t(s1))? s0 : s1 /**< @brief the maximum (in here to avoid template problems when using the greater-than operator) */
		// g++ requires cast to size_t here, to avoid the enum comparison warning
	};

public:
	typedef CCTSize<n_result> _TyResult; /**< @brief result of the addition */
};

/**
 *	@brief binary less than or equal comparison operating on scalars
 *
 *	@tparam _T1 is the first operand (specialization of CCTSize template)
 *	@tparam _T2 is the second operand (specialization of CCTSize template)
 */
template <class _T1, class _T2>
class CCompareScalar_LEqual {
public:
	/**
	 *	@brief result stored as an enum
	 */
	enum {
		b_result = size_t(_T1::n_size) <= size_t(_T2::n_size) /**< @brief result of the comparison */
	};
	// converting to size_t disables g++ warning: comparison between 'enum CCTSize<X>::<anonymous>' and 'enum CCTSize<Y>::<anonymous>'
};

/**
 *	@brief binary less than comparison operating on scalars
 *
 *	@tparam _T1 is the first operand (specialization of CCTSize template)
 *	@tparam _T2 is the second operand (specialization of CCTSize template)
 */
template <class _T1, class _T2>
class CCompareScalar_Less {
public:
	/**
	 *	@brief result stored as an enum
	 */
	enum {
		b_result = size_t(_T1::n_size) < size_t(_T2::n_size) /**< @brief result of the comparison */
	};
	// converting to size_t disables g++ warning: comparison between 'enum CCTSize<X>::<anonymous>' and 'enum CCTSize<Y>::<anonymous>'
};

/**
 *	@brief binary less than comparison operating on size 2D (number of columns
 *		being more significant, number of rows les significant)
 *
 *	@tparam _T1 is the first operand (specialization of CCTSize2D template)
 *	@tparam _T2 is the second operand (specialization of CCTSize2D template)
 */
template <class _T1, class _T2>
class CCompareSize2D_Less {
public:
	/**
	 *	@brief result stored as an enum
	 */
	enum {
		b_result = size_t(_T1::n_column_num) < size_t(_T2::n_column_num) ||
			(size_t(_T1::n_column_num) == size_t(_T2::n_column_num) &&
			size_t(_T1::n_row_num) < size_t(_T2::n_row_num)) /**< @brief result of the comparison */
	};
	// converting to size_t disables g++ warning: comparison between 'enum CCTSize2D<X>::<anonymous>' and 'enum CCTSize2D<Y>::<anonymous>'
};

/**
 *	@brief gets dimension for a given vertex
 *	@tparam CVertex is vertex type
 */
template <class CVertex>
class CGetVertexDimension {
public:
	typedef fbs_ut::CCTSize<CVertex::n_dimension> _TyResult; /**< @brief vertex dimension */
};

/**
 *	@brief gets state vector type of a given vertex
 *	@tparam CVertex is vertex type
 */
template <class CVertex>
class CGetVertexVectorType {
public:
	typedef typename CVertex::_TyVectorAlign _TyResult; /**< @brief aligned state vector type */
};

/**
 *	@brief extracts number of columns from compile-time constant 2D vector
 *	@tparam CDimensionType is a compile-time constant 2D vector
 */
template <class CDimensionType>
class CTransformDimensionColumnsToSize {
public:
	typedef CCTSize<CDimensionType::n_column_num> _TyResult; /**< @brief number of columns, extracted from CDimensionType */
};

/*template <int _n_row_num, int _n_column_num>
class CTransformDimensionColumnsToSize<CCTSize2D<_n_row_num, _n_column_num> > {
public:
	typedef CCTSize<_n_column_num> _TyResult;
};*/

/**
 *	@brief extracts number of rows from compile-time constant 2D vector
 *	@tparam CDimensionType is a compile-time constant 2D vector
 */
template <class CDimensionType>
class CTransformDimensionRowsToSize {
public:
	typedef CCTSize<CDimensionType::n_row_num> _TyResult; /**< @brief number of rows, extracted from CDimensionType */
};

/*template <int _n_row_num, int _n_column_num>
class CTransformDimensionRowsToSize<CCTSize2D<_n_row_num, _n_column_num> > {
public:
	typedef CCTSize<_n_row_num> _TyResult;
};*/

/**
 *	@brief calculates area of a rectangle from compile-time constant 2D vector
 *	@tparam CDimensionType is a compile-time constant 2D vector
 */
template <class CDimensionType>
class CTransformDimensionToAreaSize {
public:
	typedef CCTSize<CDimensionType::n_column_num * CDimensionType::n_row_num> _TyResult; /**< @brief number of columns times number of rows, extracted from CDimensionType */
};

/**
 *	@brief converts eigen matrix type to simple CCTSize2D
 *	@tparam CMatrixType is specialization of Eigen::Matrix
 */
template <class CMatrixType>
class CEigenToDimension {
public:
#if !defined(_MSC_VER) || defined(__MWERKS__) // the below version does not work with Intellisense, the one in else branch does
	typedef CCTSize2D<CMatrixType::RowsAtCompileTime,
		CMatrixType::ColsAtCompileTime> _TyResult; /**< @brief size of the matrix, represented as CCTSize2D */
#else // !_MSC_VER || __MWERKS__
	typedef Eigen::internal::traits<CMatrixType> _TyMatrixTraits; /**< @brief matrix traits */
	typedef CCTSize2D<_TyMatrixTraits::RowsAtCompileTime,
		_TyMatrixTraits::ColsAtCompileTime> _TyResult; /**< @brief size of the matrix, represented as CCTSize2D */
#endif // !_MSC_VER || __MWERKS__
};

/**
 *	@brief converts eigen matrix type to simple CCTSize2D
 *		(specialization for input already being CCTSize2D)
 *
 *	@tparam _n_row_num is number of rows (y component)
 *	@tparam _n_column_num is number of columns (x component)
 */
template <int _n_row_num, int _n_column_num>
class CEigenToDimension<CCTSize2D<_n_row_num, _n_column_num> > {
public:
	typedef CCTSize2D<_n_row_num, _n_column_num> _TyResult; /**< @brief size of the matrix, represented as CCTSize2D */
};

#if 0 // discarded as unnecessary complication, Eigen matrix types have RowsAtCompileTime and ColsAtCompileTime enum

/**
 *	@brief specialization for matrices of double (the only supported type)
 *
 *	@tparam n_compile_time_row_num number of rows
 *	@tparam n_compile_time_col_num number of columns
 *	@tparam n_options a combination of either Eigen::RowMajor or Eigen::ColMajor,
 *		and of either Eigen::AutoAlign or Eigen::DontAlign
 *	@tparam n_max_row_num maximum number of rows
 *	@tparam n_max_col_num maximum number of columns
 */
template <int n_compile_time_row_num, int n_compile_time_col_num,
	int n_options, int n_max_row_num, int n_max_col_num>
class CEigenToDimension<Eigen::Matrix<double, n_compile_time_row_num,
	n_compile_time_col_num, n_options, n_max_row_num, n_max_col_num> > {
public:
	typedef CCTSize2D<n_compile_time_row_num, n_compile_time_col_num> _TyResult; /**< @brief size of the matrix, represented as CCTSize2D */
};

#endif // 0

/**
 *	@brief transposes shape of Eigen::Matrix
 *	@tparam CMatrixType is specialization of Eigen::Matrix
 */
template <class CMatrixType>
class CEigenToTransposeEigen;

/**
 *	@brief specialization for matrices of double (the only supported type)
 *
 *	@tparam n_compile_time_row_num number of rows
 *	@tparam n_compile_time_col_num number of columns
 *	@tparam n_options a combination of either Eigen::RowMajor or Eigen::ColMajor,
 *		and of either Eigen::AutoAlign or Eigen::DontAlign
 *	@tparam n_max_row_num maximum number of rows
 *	@tparam n_max_col_num maximum number of columns
 */
template <int n_compile_time_row_num, int n_compile_time_col_num,
	int n_options, int n_max_row_num, int n_max_col_num>
class CEigenToTransposeEigen<Eigen::Matrix<double, n_compile_time_row_num,
	n_compile_time_col_num, n_options, n_max_row_num, n_max_col_num> > {
public:
	typedef Eigen::Matrix<double, n_compile_time_col_num,
		n_compile_time_row_num, n_options, n_max_col_num, n_max_row_num> _TyResult; /**< @brief transposed shape of the original matrix */
};

/**
 *	@brief conversion of a CCTSize2D to its transpose size
 *
 *	@tparam CSize2 is a specialization of CCTSize2D
 */
template <class CSize2>
class CSize2DToTransposeSize2D {
public:
	typedef CCTSize2D<CSize2::n_column_num, CSize2::n_row_num> _TyResult; /**< @brief the resulting 2D size type */
};

#if 0 // unused

/**
 *	@brief compile-time assertion (_FBS functions do not work
 *		with dynamic-size block matrices)
 *	@tparam b_expression is the expression being asserted
 */
template <const bool b_expression>
class CMATRIX_BLOCK_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME;

/**
 *	@brief compile-time assertion (specialization for assertion passing)
 *	@tparam b_expression is the expression being asserted
 */
template <>
class CMATRIX_BLOCK_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME<true> {};

#endif // 0

/**
 *	@brief predicate for filtering row height list by selected column width
 *
 *	@tparam CRowHeight is row height (value from the list that is being filtered)
 *	@tparam CColumnWidth is column width (reference value)
 */
template <class CDimsList_Uniq, class CRowHeight, class CColumnWidth>
class CHaveRowHeightForColumnWidth {
public:
	typedef CCTSize2D<CRowHeight::n_size, CColumnWidth::n_size> _TyNeedle; /**< @brief hypothetical matrix size */

	/**
	 *	@brief result stored as enum
	 */
	enum {
		b_result = CFindTypelistItem<CDimsList_Uniq, _TyNeedle>::b_result /**< @brief predicate result (looks for the matrix size in the list) */
	};
};

/**
 *	@brief predicate for filtering column width list by selected row height
 *
 *	@tparam CColumnWidth is column width (value from the list that is being filtered)
 *	@tparam CRowHeight is row height (reference value)
 */
template <class CDimsList_Uniq, class CColumnWidth, class CRowHeight>
class CHaveColumnWidthForRowHeight {
public:
	typedef CCTSize2D<CRowHeight::n_size, CColumnWidth::n_size> _TyNeedle; /**< @brief hypothetical matrix size */

	/**
	 *	@brief result stored as enum
	 */
	enum {
		b_result = CFindTypelistItem<CDimsList_Uniq, _TyNeedle>::b_result /**< @brief predicate result (looks for the matrix size in the list) */
	};
};

/**
 *	@brief conversion of a pair of CCTSize back to an Eigen matrix type with
 *		known compile-time sizes (the result can be found in the _TyResult type)
 *
 *	@tparam _T1 is a specialization of CCTSize, containing number of matrix rows
 *	@tparam _T2 is a specialization of CCTSize, containing number of matrix columns
 */
template <class _T1, class _T2>
class CMakeMatrixSizeType {
public:
	typedef Eigen::Matrix<double, _T1::n_size, _T2::n_size> _TyResult; /**< @brief the resulting Eigen matrix type */
};

/**
 *	@brief converts compile-time constant 2D vector to Eigen matrix type
 *	@tparam CDimensionType is a compile-time constant 2D vector
 */
template <class CDimensionType>
class CDimensionToEigen {
public:
	typedef Eigen::Matrix<double, CDimensionType::n_row_num, CDimensionType::n_column_num> _TyResult; /**< @brief Eigen matrix type, corresponding to the input block size */
};

/**
 *	@brief conversion of a pair of CCTSize to CCTSize2D type with
 *		known compile-time sizes (the result can be found in the _TyResult type)
 *
 *	@tparam CTRows is a specialization of CCTSize, containing number of matrix rows
 *	@tparam CTCols is a specialization of CCTSize, containing number of matrix columns
 */
template <class CTRows, class CTCols>
class CMakeCTSize2DType {
public:
	typedef CCTSize2D<CTRows::n_size, CTCols::n_size> _TyResult; /**< @brief the resulting 2D size type */
};

/**
 *	@brief conversion of a CCTSize to square CCTSize2D type with
 *		known compile-time size (the result can be found in the _TyResult type)
 *
 *	@tparam CSize is a specialization of CCTSize, containing number of matrix rows and columns
 */
template <class CSize>
class CMakeCTSize2DSquareType {
public:
	typedef CCTSize2D<CSize::n_size, CSize::n_size> _TyResult; /**< @brief the resulting 2D size type */
};

/**
 *	@brief calculates size of product size in form L^T * R
 *
 *	@tparam CSizeLeft is size of the left matrix
 *	@tparam CSizeRight is size of the right matrix
 */
template <class CSizeLeft, class CSizeRight>
class CMakeATAProductSize {
protected:
	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		n_left_col_num = CSizeLeft::n_row_num, /**< @brief number of columns of the tranposed left block */
		n_left_row_num = CSizeLeft::n_column_num, /**< @brief number of rows of the tranposed left block */
		// note that this is transposed (the product is A^T * A)

		n_right_row_num = CSizeRight::n_row_num, /**< @brief number of rows of the right block */
		n_right_col_num = CSizeRight::n_column_num, /**< @brief number of columns of the right block */

		b_can_multiply = (int(n_left_col_num) == int(n_right_row_num)) /**< @brief common dimension check */
		// int cast to avoid "warning: comparison between 'enum fbs_ut::CCTSize2D<X, Y>::<anonymous>' and 'enum fbs_ut::CCTSize2D<Z, W>::<anonymous>' [-Wenum-compare]"
	};

public:
	typedef CCTSize2D<(b_can_multiply)? n_left_row_num : -1,
		(b_can_multiply)? n_right_col_num : -1> _TyResult; /**< @brief the resulting block size, or CCTSize<-1, -1> if not multiplicable */
};

/**
 *	@brief nonnegative block size predicate
 *	@tparam CSize is size of the block
 */
template <class CSize>
class CIsNotNegsize {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = (CSize::n_row_num > 0 && CSize::n_column_num > 0) /**< @brief result; true if the size is a valid block size */
	};
};

/**
 *	@brief square block predicate
 *	@tparam CSize is size of the block
 */
template <class CSize>
class CIsSquare {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = (int(CSize::n_row_num) == int(CSize::n_column_num)) /**< @brief result; true if the size is square */
	};
};

/**
 *	@brief runtime 2D size comparator
 *	@tparam C2DSize is size of the block
 */
template <class C2DSize>
class CRuntimeCompareSize2D {
public:
	/**
	 *	@brief equality predicate
	 *	@param[in] t_size is pair with the number of rows and columns (in this order)
	 *	@return Returns true if the size specified by C2DSize is equal to t_size.
	 */
	static __forceinline bool b_Equal(std::pair<size_t, size_t> t_size)
	{
		return size_t(C2DSize::n_row_num) == t_size.first && size_t(C2DSize::n_column_num) == t_size.second;
	}

	/**
	 *	@brief less-than predicate
	 *	@param[in] t_size is pair with the number of rows and columns (in this order)
	 *	@return Returns true if the size specified by C2DSize is less than t_size.
	 *	@note The comparison is the same as in CCompareSize2D_Less.
	 */
	static __forceinline bool b_LessThan(std::pair<size_t, size_t> t_size)
	{
		return size_t(C2DSize::n_row_num) < t_size.first || (size_t(C2DSize::n_row_num) == t_size.first &&
			size_t(C2DSize::n_column_num) < t_size.second);
	}

	/**
	 *	@brief greater-than predicate
	 *	@param[in] t_size is pair with the number of rows and columns (in this order)
	 *	@return Returns true if the size specified by C2DSize is greater than t_size.
	 *	@note The comparison has the same ordering on first / second as in CCompareSize2D_Less.
	 */
	static __forceinline bool b_GreaterThan(std::pair<size_t, size_t> t_size)
	{
		return size_t(C2DSize::n_row_num) > t_size.first || (size_t(C2DSize::n_row_num) == t_size.first &&
			size_t(C2DSize::n_column_num) > t_size.second);
	}
};

/**
 *	@brief runtime size comparator
 *	@tparam CScalar is instance of CCTSize
 */
template <class CScalar>
class CRuntimeCompareScalar {
public:
	/**
	 *	@brief equality predicate
	 *	@param[in] n_size is value of the scalar
	 *	@return Returns true if the value specified by CScalar is equal to n_size.
	 */
	static __forceinline bool b_Equal(size_t n_size)
	{
		return size_t(CScalar::n_size) == n_size;
	}

	/**
	 *	@brief less-than predicate
	 *	@param[in] n_size is value of the scalar
	 *	@return Returns true if the value specified by CScalar is less than n_size.
	 *	@note The comparison is the same as in CCompareSize2D_Less.
	 */
	static __forceinline bool b_LessThan(size_t n_size)
	{
		return size_t(CScalar::n_size) < n_size;
	}

	/**
	 *	@brief greater-than predicate
	 *	@param[in] n_size is value of the scalar
	 *	@return Returns true if the value specified by CScalar is greater than n_size.
	 *	@note The comparison has the same ordering on first / second as in CCompareSize2D_Less.
	 */
	static __forceinline bool b_GreaterThan(size_t n_size)
	{
		return size_t(CScalar::n_size) > n_size;
	}
};

/**
 *	@brief calculates a typelist of block sizes after the PreMultiplyWithSelfTranspose()
 *		operation (the result can be found in the _TyResult type)
 *	@tparam CBlockSizeTypelist is typelist, containing
 *		list of block sizes as CCTSize2D
 */
template <class CBlockSizeTypelist>
class CBlockSizesAfterPreMultiplyWithSelfTranspose {
protected:
	typedef CBlockSizeTypelist CDimsList; /**< @brief list of block sizes as CCTSize2D */
	typedef typename CUniqueTypelist<CDimsList>::_TyResult CDimsList_Uniq; /**< @brief list of block sizes as CCTSize2D (duplicate records removed) */
	//typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_Uniq,
	//	CTransformDimensionRowsToSize>::_TyResult>::_TyResult CRowHeightsList; /**< @brief list of unique block row heights */
	typedef typename CUniqueTypelist<typename CCarthesianProductTypelist<CDimsList_Uniq,
		CDimsList_Uniq, CMakeATAProductSize>::_TyResult>::_TyResult CProdList; /**< @brief list of resulting block sizes and CCTSize<-1, -1> */

public:
	typedef typename CFilterTypelist1<CProdList, CIsNotNegsize>::_TyResult _TyResult; /**< @brief the resulting typelist */
	//typedef typename CUniqueTypelist<typename CCarthesianProductTypelist<CRowHeightsList,
	//	CRowHeightsList, CMakeCTSize2DType>::_TyResult>::_TyResult _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief calculates a typelist of block sizes after the PreMultiplyWithSelfTranspose()
 *		operation (the result can be found in the _TyResult type)
 *	@tparam CBlockMatrixTypelist is typelist, containing Eigen
 *		matrices with known compile-time sizes
 */
template <class CBlockMatrixTypelist>
class CBlockMatrixTypesAfterPreMultiplyWithSelfTranspose {
public:
	typedef typename CBlockSizesAfterPreMultiplyWithSelfTranspose<CBlockMatrixTypelist>::_TyResult _TySizeList; /**< @brief the resulting size list (as CCTSize2D) */
	typedef typename CTransformTypelist<_TySizeList, CDimensionToEigen>::_TyResult _TyResult; /**< @brief the resulting typelist */
};

/**
 *	@brief prints typelist containing Eigen matrices to stdout (debugging utility)
 *
 *	Use as follows:
 *	@code
 *	typedef MakeTypelist_Safe((Eigen::Matrix2d, Eigen::Matrix3d)) matrices;
 *	fbs_ut::CDumpBlockMatrixTypelist<matrices>::Print();
 *	@endcode
 *
 *	@tparam CBlockMatrixTypelist is typelist containing Eigen matrices
 */
template <class CBlockMatrixTypelist>
class CDumpBlockMatrixTypelist {
public:
	/**
	 *	@brief prints the given typelist to stdout
	 *	@note This assumes that scalar type is double, but in order to make
	 *		the implementation simple, it is only assumed, the scalar type is ignored.
	 */
	static void Print()
	{
		typedef typename CBlockMatrixTypelist::_TyHead THead;
		printf("Eigen::Matrix<double, %d, %d> ", THead::RowsAtCompileTime, THead::ColsAtCompileTime);
		CDumpBlockMatrixTypelist<typename CBlockMatrixTypelist::_TyTail>::Print();
	}
};

/**
 *	@brief prints typelist containing Eigen matrices to stdout
 *		(debugging utility; specialization for the end of the list)
 */
template <>
class CDumpBlockMatrixTypelist<CTypelistEnd> {
public:
	/**
	 *	@brief prints the given typelist to stdout
	 */
	static void Print()
	{
		printf("\n");
	}
};

/**
 *	@brief prints typelist containing CCTSize sizes to stdout (debugging utility)
 *
 *	Use as follows:
 *	@code
 *	typedef MakeTypelist_Safe((fbs_ut::CCTSize<1>, fbs_ut::CCTSize<2>)) sizes;
 *	fbs_ut::CDumpCTSizeTypelist<sizes>::Print();
 *	@endcode
 *
 *	@tparam CCTSizeTypelist is typelist containing CCTSize sizes
 */
template <class CCTSizeTypelist>
class CDumpCTSizeTypelist {
public:
	/**
	 *	@brief prints the given typelist to stdout
	 *	@note This assumes that scalar type is double, but in order to make
	 *		the implementation simple, it is only assumed, the scalar type is ignored.
	 */
	static void Print()
	{
		typedef typename CCTSizeTypelist::_TyHead THead;
		printf("CCTsize<%d> ", THead::n_size);
		CDumpCTSizeTypelist<typename CCTSizeTypelist::_TyTail>::Print();
	}
};

/**
 *	@brief prints typelist containing CCTSize sizes to stdout
 *		(debugging utility; specialization for the end of the list)
 */
template <>
class CDumpCTSizeTypelist<CTypelistEnd> {
public:
	/**
	 *	@brief prints the given typelist to stdout
	 */
	static void Print()
	{
		printf("\n");
	}
};

template <const int n_pivot, class CList>
class CCallFnOp {
public:
	typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyBlockSizes;
	// block sizes for this specialization

public:
	template <class _Ty>
	static __forceinline void Do(_Ty &r_fun)
	{
#if defined(_MSC_VER) && !defined(__MWERKS__)
		r_fun.operator ()<_TyBlockSizes>(); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
		r_fun.template operator ()<_TyBlockSizes>();
#endif // _MSC_VER && !__MWERKS__
		// call the function operator // note that if we used token-based scheduling here, this could be used with C++14 lambdas too
	}
};

template <class CList>
class CCallFnOp<-1, CList> {
public:
	template <class _Ty, class _Data>
	static inline void Do(_Ty &r_fun, _Data input_not_found)
	{
#if defined(_MSC_VER) && !defined(__MWERKS__)
		r_fun.operator ()(input_not_found); // MSVC cannot parse the .template syntax correctly with overloaded operators
#else // _MSC_VER && !__MWERKS__
		r_fun.template operator ()(input_not_found);
#endif // _MSC_VER && !__MWERKS__
		// call the function operator
	}
};

/**
 *	@brief makes a decission tree from a sorted typelist
 *
 *	@tparam CList is a sorted list of any type (defaults work with CCTSize)
 *	@tparam CFunctor is a functor template that specializes for a given size and
 *		for context which is equal to CContext2, having a global static function
 *		Do<index, CContext2>(reference) which is called in the leafs of the decision tree,
 *		index is either equal to the index found in the list and the reference argument is absent,
 *		or it is equal to -1 in case the item is not found and the reference matches the reference
 *		given to CMakeDecisionTree3::Do() (by default CCallFnOp)
 *	@tparam CComparator is compile-time list item type to run-time reference value
 *		comparator template (by default CRuntimeCompareScalar)
 *	@tparam CReference is reference value type (by default size_t)
 */
template <class CList, template <const int, class> class CFunctor = CCallFnOp,
	template <class> class CComparator = CRuntimeCompareScalar, class CReference = size_t>
class CMakeDecisionTree3 {
protected:
	typedef typename CMakeDecisionTreeSkeleton<CList, CComparator, CReference>::_TyRoot _TySkeleton; /**< @brief decision tree skeleton */

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode, class CContext>
	static __forceinline typename CEnableIf<CNode::b_leaf>::T RecurseEx(CReference n_size, CContext context)
	{
		if(CNode::b_Equals_Pivot(n_size))
			CFunctor<CNode::n_pivot, CList>::Do(context);
		else
			CFunctor<-1, CList>::Do(context, n_size);
		// this is it; call the functor
	}

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode, class CContext>
	static __forceinline typename CEnableIf<!CNode::b_leaf>::T RecurseEx(CReference n_size, CContext context)
	{
		if(CNode::b_Left_of_Pivot(n_size))
			RecurseEx<typename CNode::_TyLeft>(n_size, context);
		else
			RecurseEx<typename CNode::_TyRight>(n_size, context);
		// recurse left or right, based on the pivot value
	}

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode, class CContext>
	static __forceinline typename CEnableIf<CNode::b_leaf>::T Recurse(CReference n_size,
		CContext context, bool UNUSED(b_mind_inexistent))
	{
		_ASSERTE(!b_mind_inexistent || CNode::b_Equals_Pivot(n_size)); // either dont mind that the item isnt on the list, or it must match
		CFunctor<CNode::n_pivot, CList>::Do(context);
		// this is it; call the functor
	}

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode, class CContext>
	static __forceinline typename CEnableIf<!CNode::b_leaf>::T Recurse(CReference n_size,
		CContext context, bool b_mind_inexistent)
	{
		if(CNode::b_Left_of_Pivot(n_size))
			Recurse<typename CNode::_TyLeft>(n_size, context, b_mind_inexistent);
		else
			Recurse<typename CNode::_TyRight>(n_size, context, b_mind_inexistent);
		// recurse left or right, based on the pivot value
	}

public:
	/**
	 *	@brief the executive function; the size in question must exist
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 *	@param[in] b_mind_inexistent is inexistent tolerance flag (debug only)
	 */
	template <class CContext>
	static __forceinline void DoExisting(CReference n_size, CContext context, bool b_mind_inexistent = true)
	{
		Recurse<_TySkeleton>(n_size, context, b_mind_inexistent);
	}

	/**
	 *	@brief the extended executive function; the size in question may not exist
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 */
	template <class CContext>
	static __forceinline void Do(CReference n_size, CContext context)
	{
		RecurseEx<_TySkeleton>(n_size, context);
	}

#ifdef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

	/**
	 *	@brief prints (unidented) decission tree pseudocode to stdout
	 *	@note This is only available if
	 *		__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING is defined.
	 */
	static void Debug()
	{
		_TySkeleton::Debug();
	}

#endif // __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
};

template <template <const int> class CFunctor>
class CDTOp_Env {
public:
	template <const int n_pivot, class CList>
	class CDTOp_ExpandCTSize {
	public:
		typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyBlockSize;
		// block sizes for this specialization

	public:
		template <class CContext>
		static inline void Do(CContext c)
		{
			CFunctor<_TyBlockSize::n_size>::Do(c);
		}
	};
};

template <template <const int, class> class  CFunctor, class CContext2>
class CDTOp_Env2 {
public:
	template <const int n_pivot, class CList>
	class CDTOp_ExpandCTSize_Ctx {
	public:
		typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyBlockSize;
		// block sizes for this specialization

	public:
		template <class CContext>
		static inline void Do(CContext c)
		{
			CFunctor<_TyBlockSize::n_size, CContext2>::Do(c);
		}
	};
};

/**
 *	@brief makes a decission tree from a sorted typelist
 *
 *	@tparam CList is a sorted list of scalars (type CCTSize)
 *	@tparam CContext is a type of context, required by the operation
 *	@tparam CFunctor is a functor template that specializes for a given size,
 *		having a global static function Do(CContext) which is called in the
 *		leafs of the decision tree
 */
template <class CList, class CContext, template <const int> class CFunctor>
class CMakeDecisionTree {
protected:
#if 0
	typedef typename CMakeDecisionTreeSkeleton<CList, CRuntimeCompareScalar, size_t>::_TyRoot _TySkeleton; /**< @brief decision tree skeleton */

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode>
	static __forceinline typename CEnableIf<CNode::b_leaf>::T Recurse(const size_t UNUSED(n_size), CContext context)
	{
		_ASSERTE(CNode::b_Equals_Pivot(n_size));
		CFunctor<CNode::_TyPivot::n_size>::Do(context);
		// this is it; call the functor
	}

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode>
	static __forceinline typename CEnableIf<!CNode::b_leaf>::T Recurse(const size_t n_size, CContext context)
	{
		if(CNode::b_Left_of_Pivot(n_size))
			Recurse<typename CNode::_TyLeft>(n_size, context);
		else
			Recurse<typename CNode::_TyRight>(n_size, context);
		// recurse left or right, based on the pivot value
	}

public:
	/**
	 *	@brief the executive function
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 */
	static __forceinline void Do(const size_t n_size, CContext context)
	{
		Recurse<_TySkeleton>(n_size, context);
	}

#ifdef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

	/**
	 *	@brief prints (unidented) decission tree pseudocode to stdout
	 *	@note This is only available if
	 *		__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING is defined.
	 */
	static void Debug()
	{
		_TySkeleton::Debug();
	}

#endif // __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
#else // 0
protected:
	typedef CMakeDecisionTree3<CList, CDTOp_Env<CFunctor>::template CDTOp_ExpandCTSize> _TyImpl;

public:
	/**
	 *	@brief the executive function
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 */
	static __forceinline void Do(const size_t n_size, CContext context)
	{
		_TyImpl::DoExisting(n_size, context);
	}

#ifdef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

	/**
	 *	@brief prints (unidented) decission tree pseudocode to stdout
	 *	@note This is only available if
	 *		__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING is defined.
	 */
	static void Debug()
	{
		_TyImpl::Debug();
	}

#endif // __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
#endif // 0
};

/**
 *	@brief makes a decission tree from a sorted typelist
 *
 *	@tparam CList is a sorted list of scalars (type CCTSize)
 *	@tparam CContext is a type of context, required by the operation
 *	@tparam CContext2 is a type of context, passed as a second template parameter for the operation
 *	@tparam CFunctor is a functor template that specializes for a given size and
 *		for context which is equal to CContext2, having a global static function
 *		Do(CContext) which is called in the leafs of the decision tree
 */
template <class CList, class CContext, class CContext2,
	template <const int, class> class CFunctor>
class CMakeDecisionTree2 {
protected:
#if 0
	typedef typename CMakeDecisionTreeSkeleton<CList, CRuntimeCompareScalar, size_t>::_TyRoot _TySkeleton; /**< @brief decision tree skeleton */

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode>
	static __forceinline typename CEnableIf<CNode::b_leaf>::T Recurse(const size_t UNUSED(n_size), CContext context)
	{
		_ASSERTE(CNode::b_Equals_Pivot(n_size));
		CFunctor<CNode::_TyPivot::n_size, CContext2>::Do(context);
		// this is it; call the functor
	}

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma inline_recursion(on)
#endif // _MSC_VER && !__MWERKS__
	template <class CNode>
	static __forceinline typename CEnableIf<!CNode::b_leaf>::T Recurse(const size_t n_size, CContext context)
	{
		if(CNode::b_Left_of_Pivot(n_size))
			Recurse<typename CNode::_TyLeft>(n_size, context);
		else
			Recurse<typename CNode::_TyRight>(n_size, context);
		// recurse left or right, based on the pivot value
	}

public:
	/**
	 *	@brief the executive function
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 */
	static __forceinline void Do(const size_t n_size, CContext context)
	{
		Recurse<_TySkeleton>(n_size, context);
	}

#ifdef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

	/**
	 *	@brief prints (unidented) decission tree pseudocode to stdout
	 *	@note This is only available if
	 *		__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING is defined.
	 */
	static void Debug()
	{
		_TySkeleton::Debug();
	}

#endif // __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
#else // 0
protected:
	typedef CMakeDecisionTree3<CList, CDTOp_Env2<CFunctor, CContext2>::template CDTOp_ExpandCTSize_Ctx> _TyImpl;

public:
	/**
	 *	@brief the executive function
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 */
	static __forceinline void Do(const size_t n_size, CContext context)
	{
		_TyImpl::DoExisting(n_size, context);
	}

#ifdef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

	/**
	 *	@brief prints (unidented) decission tree pseudocode to stdout
	 *	@note This is only available if
	 *		__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING is defined.
	 */
	static void Debug()
	{
		_TyImpl::Debug();
	}

#endif // __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
#endif // 0
};

/**
 *	@brief a simple square size predicate
 *	@tparam _TySize is a specialization of CCTSize2D
 */
template <class _TySize>
class CIsSize2DSquare {
public:
	/**
	 *	@brief result stored as enum
	 */
	enum {
		b_result = (_TySize::n_row_num == _TySize::n_column_num)? 1 : 0 /**< @brief nonzero if _TySize is a square, otherwise zero */
	};
};

namespace deprecate {

/**
 *	@brief makes a decission tree for possible block matrix sizes,
 *		given a list of block sizes and a maximal size
 *
 *	@tparam CBlockMatrixTypelist is a list of possible block sizes as Eigen::Matrix
 *	@tparam n_max_matrix_size is maximum matrix size, in elements
 *
 *	@deprecated This is now deprecated in favor of CWrap3::In_SquareMatrixSize_DecisionTree().
 */
template <class CBlockMatrixTypelist, const int n_max_matrix_size>
class CMakeSquareMatrixSizeDecisionTree {
protected:
	typedef CBlockMatrixTypelist _TyBlockMatrixTypelist; /**< @brief list of block sizes, represented as Eigen::Matrix */
	typedef typename CTransformTypelist<_TyBlockMatrixTypelist,
		CEigenToDimension>::_TyResult CDimsList; /**< @brief list of block sizes as CCTSize2D */
	typedef typename CFilterTypelist1<CDimsList, CIsSize2DSquare>::_TyResult CSquareDimsList; /**< @brief list of square block sizes as CCTSize2D */
	typedef typename CTransformTypelist<CSquareDimsList, CTransformDimensionColumnsToSize>::_TyResult CWidthList; /**< @brief list of square block widths as CCTSize */
	typedef typename CBinaryCombinationTypelist<CWidthList, CBinaryScalarAdd,
		CCompareScalar_LEqual, CCTSize<n_max_matrix_size> >::_TyResult _TyPossibleWidthList; /**< @brief list of all possible matrix sizes */
	typedef typename CSortTypelist<_TyPossibleWidthList, CCompareScalar_Less>::_TyResult _TySortedList; /**< @brief sorted list of all possible matrix sizes */

	/**
	 *	@brief an empty functor, used for debugging
	 *	@tparam n_size is matrix size
	 */
	template <const int n_size>
	class CDoNothing {
	public:
		/**
		 *	@brief an empty function
		 *	@param[in] context is a context (unused)
		 */
		static void Do(int UNUSED(context))
		{}
	};

public:
	/**
	 *	@brief the executive function
	 *
	 *	@param[in] n_size is the parameter to be decided over
	 *	@param[in,out] context is the operation context (task specific)
	 */
	template <template <const int> class CFunctor, class CContext>
	static __forceinline void Do(const size_t n_size, CContext context)
	{
		CMakeDecisionTree<_TySortedList, CContext, CFunctor>::Do(n_size, context);
	}

#ifdef __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING

	/**
	 *	@brief prints (unidented) decission tree pseudocode to stdout
	 *	@note This is only available if
	 *		__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING is defined.
	 */
	static void Debug()
	{
		CMakeDecisionTree<_TySortedList, int, CDoNothing>::Debug();
	}

#endif // __UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_DEBUGGING
};

/**
 *	@brief fixed block size operations template class
 *
 *	@tparam CBlockMatrixTypelistA is typelist, containing Eigen
 *		matrices with known compile-time sizes for the matrix on the left
 *	@tparam CBlockMatrixTypelistB is typelist, containing Eigen
 *		matrices with known compile-time sizes for the matrix on the right
 *
 *	@deprecated This is now deprecated in favor of decision tree wrapping which
 *		does its own block size list filtering, as needed.
 */
template <class CBlockMatrixTypelistA, class CBlockMatrixTypelistB>
class CFixedBlockSize_BinaryBase {
public:
	typedef CBlockMatrixTypelistA _TyBlockMatrixTypelistA; /**< @brief list of block sizes, represented as Eigen::Matrix */
	typedef typename CTransformTypelist<_TyBlockMatrixTypelistA,
		CEigenToDimension>::_TyResult CDimsListA; /**< @brief list of block sizes as CCTSize2D */
	typedef typename CUniqueTypelist<CDimsListA>::_TyResult CDimsList_UniqA; /**< @brief list of block sizes as CCTSize2D (duplicate records removed) */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_UniqA,
		CTransformDimensionColumnsToSize>::_TyResult>::_TyResult CColumnWidthsListA; /**< @brief list of unique block column widths */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_UniqA,
		CTransformDimensionRowsToSize>::_TyResult>::_TyResult CRowHeightsListA; /**< @brief list of unique block row heights */

	typedef CBlockMatrixTypelistB _TyBlockMatrixTypelistB; /**< @brief list of block sizes, represented as Eigen::Matrix */
	typedef typename CTransformTypelist<_TyBlockMatrixTypelistB,
		CEigenToDimension>::_TyResult CDimsListB; /**< @brief list of block sizes as CCTSize2D */
	typedef typename CUniqueTypelist<CDimsListB>::_TyResult CDimsList_UniqB; /**< @brief list of block sizes as CCTSize2D (duplicate records removed) */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_UniqB,
		CTransformDimensionColumnsToSize>::_TyResult>::_TyResult CColumnWidthsListB; /**< @brief list of unique block column widths */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_UniqB,
		CTransformDimensionRowsToSize>::_TyResult>::_TyResult CRowHeightsListB; /**< @brief list of unique block row heights */
	// create typelists to generate the decision tree for block sizes

	typedef typename blockmatrix_detail::CStaticAssert<
		!CFindTypelistItem<CColumnWidthsListA, CCTSize<Eigen::Dynamic> >::b_result &&
		!CFindTypelistItem<CRowHeightsListA, CCTSize<Eigen::Dynamic> >::b_result &&
		!CFindTypelistItem<CColumnWidthsListB, CCTSize<Eigen::Dynamic> >::b_result &&
		!CFindTypelistItem<CRowHeightsListB, CCTSize<Eigen::Dynamic> >::b_result>::MATRIX_BLOCK_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME CAssert0; /**< @brief make sure that all the blocks have the sizes fixed and known at compile-time */
};

/**
 *	@brief fixed block size operations template class
 *	@tparam CBlockMatrixTypelist is typelist, containing Eigen
 *		matrices with known compile-time sizes
 *	@deprecated This is now deprecated in favor of decision tree wrapping which
 *		does its own block size list filtering, as needed.
 */
template <class CBlockMatrixTypelist>
class CFixedBlockSize_UnaryBase {
public:
	typedef CBlockMatrixTypelist _TyBlockMatrixTypelist; /**< @brief list of block sizes, represented as Eigen::Matrix */
	typedef typename CTransformTypelist<_TyBlockMatrixTypelist,
		CEigenToDimension>::_TyResult CDimsList; /**< @brief list of block sizes as CCTSize2D */
	typedef typename CUniqueTypelist<CDimsList>::_TyResult CDimsList_Uniq; /**< @brief list of block sizes as CCTSize2D (duplicate records removed) */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_Uniq,
		CTransformDimensionColumnsToSize>::_TyResult>::_TyResult CColumnWidthsList; /**< @brief list of unique block column widths */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_Uniq,
		CTransformDimensionRowsToSize>::_TyResult>::_TyResult CRowHeightsList; /**< @brief list of unique block row heights */
	typedef typename CUniqueTypelist<typename CTransformTypelist<CDimsList_Uniq,
		CTransformDimensionToAreaSize>::_TyResult>::_TyResult CBlockAreasList; /**< @brief list of unique block areas (for elementwise ops) */
	// create typelists to generate the decision tree for block sizes

	typedef typename blockmatrix_detail::CStaticAssert<
		!CFindTypelistItem<CColumnWidthsList, CCTSize<Eigen::Dynamic> >::b_result &&
		!CFindTypelistItem<CRowHeightsList, CCTSize<Eigen::Dynamic> >::b_result>::MATRIX_BLOCK_DIMENSIONS_MUST_BE_KNOWN_AT_COMPILE_TIME CAssert0; /**< @brief make sure that all the blocks have the sizes fixed and known at compile-time */
};

} // ~deprecate

/**
 *	@brief simple utility class for sorting block dimensions
 *	@tparam CBlockMatrixTypelist is typelist, containing Eigen
 *		matrices with known compile-time sizes
 */
template <class CBlockMatrixTypelist>
class CSortBlockDims {
public:
	typedef typename CSortTypelist<typename CUniqueTypelist<typename
		CTransformTypelist<CBlockMatrixTypelist, CEigenToDimension>::_TyResult>::_TyResult,
		CCompareSize2D_Less>::_TyResult _TyResult; /**< @brief unique sorted list of block matrix dimensions */
};

namespace deprecate {

/**
 *	@brief simple utility for writing clean FBS loops
 *
 *	This would be used as follows:
 *	@code
 *	class CMy_FBS_Loop {
 *	public:
 *		struct TContext { // call context
 *			int params; // parameters to the loop, return value, ...
 *
 *			inline TContext(int _params) // provide constructor
 *				:params(_params)
 *			{}
 *		};
 *
 *		template <const int n_fixed_size>
 *		struct TLoopBody { // loop body for a given fixed size
 *			static inline void Do(TContext t_ctx) // loop implementation
 *			{
 *				// implement the loop here
 *			}
 *		};
 *
 *	public:
 *		template <class CBlockMatrixTypelist> // list of possible block sizes (will fail on other sizes!)
 *		static inline void Loop(size_t n_size, int params) // loop interface; add parameters here
 *		{
 *			fbs_ut::CWrap<TLoopBody>::template
 *				In_ColumnWidth_DecisionTree<CBlockMatrixTypelist>(n_size, TContext(params));
 *			// wrap this loop in column width decision tree
 *		}
 *	};
 *	@endcode
 *
 *	@tparam CLoopType is a loop type; must be parametrizable by block size and
 *		must implement a <tt>Do(context)</tt> function
 *
 *	@note In case the loop body requires static context as well
 *		(such as block size list), use CWrap2 instead.
 */
template <template <const int n_size> class CLoopType>
class CWrap {
public:
	/**
	 *	@brief wraps a function in scalar size decision tree
	 *
	 *	@tparam CScalarSizeTypelist is list of all possible sizes (as CCTSize)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_scalar_size is size at invokation (must be one of sizes in CScalarSizeTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CScalarSizeTypelist, class _TyContext>
	static __forceinline void In_ScalarSize_DecisionTree(int n_scalar_size, _TyContext c)
	{
		typedef typename CSortTypelist<typename CUniqueTypelist<CScalarSizeTypelist>::_TyResult,
			fbs_ut::CCompareScalar_Less>::_TyResult _TySortedSizes;
		// sorted list of unique sizes

		CMakeDecisionTree<_TySortedSizes, _TyContext, CLoopType>::Do(n_scalar_size, c);
		// enter the decision tree
	}

	/**
	 *	@brief wraps a function in block width decision tree
	 *
	 *	@tparam CBlockMatrixTypelist is list of all possible matrix block sizes (as Eigen::Matrix with compile-time sizes)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_column_width is size at invokation (must be one of column widths in CBlockMatrixTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CBlockMatrixTypelist, class _TyContext>
	static __forceinline void In_ColumnWidth_DecisionTree(int n_column_width, _TyContext c)
	{
		typedef typename CUniqueTypelist<typename CTransformTypelist<typename
			CSortBlockDims<CBlockMatrixTypelist>::_TyResult,
			CTransformDimensionColumnsToSize>::_TyResult>::_TyResult _TySortedColumnWidths;
		// sorted list of unique block column widths

		CMakeDecisionTree<_TySortedColumnWidths, _TyContext, CLoopType>::Do(n_column_width, c);
		// enter the decision tree
	}

	/**
	 *	@brief wraps a function in block height decision tree, given column width
	 *
	 *	@tparam CBlockMatrixTypelist is list of all possible matrix block sizes (as Eigen::Matrix with compile-time sizes)
	 *	@tparam n_column_width is column width, in elements (must equal one of column widths in CBlockMatrixTypelist)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_row_height is size at invokation (must be one of row heights in CBlockMatrixTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CBlockMatrixTypelist, const int n_column_width, class _TyContext>
	static __forceinline void In_RowHeight_DecisionTree_Given_ColumnWidth(int n_row_height, _TyContext c)
	{
		typedef typename CSortBlockDims<CBlockMatrixTypelist>::_TyResult _TyDimsList;
		typedef typename CUniqueTypelist<typename CTransformTypelist<_TyDimsList,
			CTransformDimensionRowsToSize>::_TyResult>::_TyResult _TySortedRowHeights;
		// sorted list of unique block row heights

		typedef typename CFilterTypelist2<_TySortedRowHeights, fbs_ut::CCTSize<n_column_width>,
			fbs_ut::CHaveRowHeightForColumnWidth, _TyDimsList>::_TyResult _TySelectedRowHeights;
		// t_odo - apply column width filter here

		CMakeDecisionTree<_TySelectedRowHeights, _TyContext, CLoopType>::Do(n_row_height, c);
		// enter the decision tree
	}

	// todo - RowHeight
	// todo - conditional (ColumnWidth_Given_RowHeight and RowHeight_Given_ColumnWidth)
};

} // ~deprecate

/**
 *	@brief simple utility for writing clean FBS loops
 *
 *	This would be used as follows:
 *	@code
 *	class CMy_FBS_Loop {
 *	public:
 *		struct TContext { // call context
 *			int params; // parameters to the loop, return value, ...
 *
 *			inline TContext(int _params) // provide constructor
 *				:params(_params)
 *			{}
 *		};
 *
 *		template <const int n_fixed_size, class CBlockSizes> // CBlockSizes filled from the CContext2 argument
 *		struct TLoopBody { // loop body for a given fixed size
 *			static inline void Do(TContext t_ctx) // loop implementation
 *			{
 *				// implement the loop here
 *			}
 *		};
 *
 *	public:
 *		template <class CBlockMatrixTypelist> // list of possible block sizes (will fail on other sizes!)
 *		static inline void Loop(size_t n_size, int params) // loop interface; add parameters here
 *		{
 *			fbs_ut::CWrap2<TLoopBody, CBlockMatrixTypelist>::template
 *				In_ColumnWidth_DecisionTree<CBlockMatrixTypelist>(n_size, TContext(params));
 *			// wrap this loop in column width decision tree,
 *			// and also pass CBlockMatrixTypelist to TLoopBody
 *		}
 *	};
 *	@endcode
 *
 *	@tparam CLoopType is a loop type; must be parametrizable by block size and
 *		static context and must implement a <tt>Do(context)</tt> function
 *	@tparam CContext2 is a type of context, passed as a second template parameter for the operation
 *
 *	@note In case the loop body does not require the static context, use CWrap instead.
 */
template <template <const int, class> class CLoopType, class CContext2>
class CWrap2 {
public:
	/**
	 *	@brief wraps a function in scalar size decision tree
	 *
	 *	@tparam CScalarSizeTypelist is list of all possible sizes (as CCTSize)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_scalar_size is size at invokation (must be one of sizes in CScalarSizeTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CScalarSizeTypelist, class _TyContext>
	static __forceinline void In_ScalarSize_DecisionTree(int n_scalar_size, _TyContext c)
	{
		typedef typename CSortTypelist<typename CUniqueTypelist<CScalarSizeTypelist>::_TyResult,
			fbs_ut::CCompareScalar_Less>::_TyResult _TySortedSizes;
		// sorted list of unique sizes

		CMakeDecisionTree2<_TySortedSizes, _TyContext, CContext2, CLoopType>::Do(n_scalar_size, c);
		// enter the decision tree
	}

	/**
	 *	@brief wraps a function in block width decision tree
	 *
	 *	@tparam CBlockMatrixTypelist is list of all possible matrix block sizes (as Eigen::Matrix with compile-time sizes)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_column_width is size at invokation (must be one of column widths in CBlockMatrixTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CBlockMatrixTypelist, class _TyContext>
	static __forceinline void In_ColumnWidth_DecisionTree(int n_column_width, _TyContext c)
	{
		typedef typename CUniqueTypelist<typename CTransformTypelist<typename
			CSortBlockDims<CBlockMatrixTypelist>::_TyResult,
			CTransformDimensionColumnsToSize>::_TyResult>::_TyResult _TySortedColumnWidths;
		// sorted list of unique block column widths

		CMakeDecisionTree2<_TySortedColumnWidths, _TyContext, CContext2, CLoopType>::Do(n_column_width, c);
		// enter the decision tree
	}

	/**
	 *	@brief wraps a function in block height decision tree, given column width
	 *
	 *	@tparam CBlockMatrixTypelist is list of all possible matrix block sizes (as Eigen::Matrix with compile-time sizes)
	 *	@tparam n_column_width is column width, in elements (must equal one of column widths in CBlockMatrixTypelist)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_row_height is size at invokation (must be one of row heights in CBlockMatrixTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CBlockMatrixTypelist, const int n_column_width, class _TyContext>
	static __forceinline void In_RowHeight_DecisionTree_Given_ColumnWidth(int n_row_height, _TyContext c)
	{
		typedef typename CSortBlockDims<CBlockMatrixTypelist>::_TyResult _TyDimsList;
		typedef typename CUniqueTypelist<typename CTransformTypelist<_TyDimsList,
			CTransformDimensionRowsToSize>::_TyResult>::_TyResult _TySortedRowHeights;
		// sorted list of unique block row heights

		typedef typename CFilterTypelist2<_TySortedRowHeights, fbs_ut::CCTSize<n_column_width>,
			fbs_ut::CHaveRowHeightForColumnWidth, _TyDimsList>::_TyResult _TySelectedRowHeights;
		// t_odo - apply column width filter here

		CMakeDecisionTree2<_TySelectedRowHeights, _TyContext, CContext2, CLoopType>::Do(n_row_height, c);
		// enter the decision tree
	}

	// todo - RowHeight
	// todo - conditional (ColumnWidth_Given_RowHeight and RowHeight_Given_ColumnWidth)
};

/**
 *	@brief simple utility for writing clean FBS loops
 *
 *	This would be used as follows:
 *	@code
 *	class CMy_FBS_Loop {
 *		int params; // parameters to the loop, return value, ...
 *
 *	public:
 *		inline CMy_FBS_Loop(int _params) // provide constructor
 *			:params(_params)
 *		{}
 *
 *		template <class CBlockSize> // CBlockSize filled from the CScalarSizeTypelist argument
 *		static inline void operator ()() // loop implementation
 *		{
 *			// implement the loop here
 *		}
 *
 *	private:
 *		static inline void operator ()(size_t n_dynamic_size); // loop implementation if want to support dynamic sizes
 *	};
 *
 *	fbs_ut::CWrap3<>::template In_ColumnWidth_DecisionTree<CBlockMatrixTypelist>(n_size, CMy_FBS_Loop(params));
 *	@endcode
 *
 *	@tparam CLoopType is a loop type; must be parametrizable by block size and
 *		static context and must implement a <tt>Do<index, list>(context)</tt> function,
 *		by default \ref CCallFnOp
 */
template <template <const int, class> class CLoopType = CCallFnOp>
class CWrap3 {
public:
	/**
	 *	@brief wraps a function in scalar size decision tree
	 *
	 *	@tparam CScalarSizeTypelist is list of all possible sizes (as CCTSize)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_scalar_size is size at invokation (must be one of sizes in CScalarSizeTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CScalarSizeTypelist, class _TyContext>
	static __forceinline void In_ScalarSize_DecisionTree(size_t n_scalar_size,
		_TyContext c, bool b_mind_nonexistent = true)
	{
		typedef typename CSortTypelist<typename CUniqueTypelist<CScalarSizeTypelist>::_TyResult,
			fbs_ut::CCompareScalar_Less>::_TyResult _TySortedSizes;
		// sorted list of unique sizes

		CMakeDecisionTree3<_TySortedSizes, CLoopType>::DoExisting(n_scalar_size, c, b_mind_nonexistent);
		// enter the decision tree
	}

	/**
	 *	@brief wraps a function in block width decision tree
	 *
	 *	@tparam CBlockMatrixTypelist is list of all possible matrix block sizes (as Eigen::Matrix with compile-time sizes)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_column_width is size at invokation (must be one of column widths in CBlockMatrixTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CBlockMatrixTypelist, class _TyContext>
	static __forceinline void In_ColumnWidth_DecisionTree(size_t n_column_width,
		_TyContext c, bool b_mind_nonexistent = true)
	{
		typedef typename CUniqueTypelist<typename CTransformTypelist<typename
			CSortBlockDims<CBlockMatrixTypelist>::_TyResult,
			CTransformDimensionColumnsToSize>::_TyResult>::_TyResult _TySortedColumnWidths;
		// sorted list of unique block column widths

		CMakeDecisionTree3<_TySortedColumnWidths, CLoopType>::DoExisting(n_column_width, c, b_mind_nonexistent);
		// enter the decision tree
	}

	/**
	 *	@brief wraps a function in block height decision tree, given column width
	 *
	 *	@tparam CBlockMatrixTypelist is list of all possible matrix block sizes (as Eigen::Matrix with compile-time sizes)
	 *	@tparam n_column_width is column width, in elements (must equal one of column widths in CBlockMatrixTypelist)
	 *	@tparam _TyContext is type of context that the wrapped function requires
	 *
	 *	@param[in] n_row_height is size at invokation (must be one of row heights in CBlockMatrixTypelist)
	 *	@param[in] c is value of the context to be passed to the wrapped function
	 *
	 *	@note This executes operations at run time.
	 */
	template <class CBlockMatrixTypelist, const int n_column_width, class _TyContext>
	static __forceinline void In_RowHeight_DecisionTree_Given_ColumnWidth(size_t n_row_height,
		_TyContext c, bool b_mind_nonexistent = true)
	{
		typedef typename CSortBlockDims<CBlockMatrixTypelist>::_TyResult _TyDimsList;
		typedef typename CUniqueTypelist<typename CTransformTypelist<_TyDimsList,
			CTransformDimensionRowsToSize>::_TyResult>::_TyResult _TySortedRowHeights;
		// sorted list of unique block row heights

		typedef typename CFilterTypelist2<_TySortedRowHeights, fbs_ut::CCTSize<n_column_width>,
			fbs_ut::CHaveRowHeightForColumnWidth, _TyDimsList>::_TyResult _TySelectedRowHeights;
		// t_odo - apply column width filter here

		CMakeDecisionTree3<_TySelectedRowHeights, CLoopType>::DoExisting(n_row_height, c, b_mind_nonexistent);
		// enter the decision tree
	}

	template <class CBlockMatrixTypelist, const int n_max_matrix_size, class _TyContext>
	static __forceinline void In_SquareMatrixSize_DecisionTree(size_t n_matrix_size,
		_TyContext c, bool b_mind_nonexistent = true)
	{
		typedef typename CTransformTypelist<CBlockMatrixTypelist,
			CEigenToDimension>::_TyResult CDimsList; /**< @brief list of block sizes as CCTSize2D */
		typedef typename CUniqueTypelist<typename CFilterTypelist1<CDimsList, CIsSize2DSquare>::_TyResult>::_TyResult CSquareDimsList; /**< @brief list of square block sizes as CCTSize2D */
		typedef typename CTransformTypelist<CSquareDimsList, CTransformDimensionColumnsToSize>::_TyResult CWidthList; /**< @brief list of square block widths as CCTSize */
		typedef typename CBinaryCombinationTypelist<CWidthList, CBinaryScalarAdd,
			CCompareScalar_LEqual, CCTSize<n_max_matrix_size> >::_TyResult _TyPossibleWidthList; /**< @brief list of all possible matrix sizes */
		typedef typename CSortTypelist<_TyPossibleWidthList, CCompareScalar_Less>::_TyResult _TySortedList; /**< @brief sorted list of all possible matrix sizes */

		CMakeDecisionTree3<_TySortedList, CLoopType>::DoExisting(n_matrix_size, c, b_mind_nonexistent);
		// enter the decision tree
	}

	// todo - RowHeight
	// todo - conditional (ColumnWidth_Given_RowHeight and RowHeight_Given_ColumnWidth)
};

/**
 *	@brief helper object for copying typelist data to an array
 *	@tparam T is data type of the output array elements
 */
template <class T = int>
class CCopyCTSizesToArray {
protected:
	T *m_p_dest; /**< @brief pointer to the next output element */

public:
	/**
	 *	@brief default constructor; initializes the output pointer
	 *	@param[in] p_dest is pointer to the destination array (must be allocated to sufficient number of elements)
	 */
	CCopyCTSizesToArray(T *p_dest)
		:m_p_dest(p_dest)
	{}

	/**
	 *	@brief function operator; writes a single number in the output array
	 *	@tparam CDimension is a specialization of CCTSize (specifies the value to be written)
	 */
	template <class CDimension>
	void operator ()()
	{
		*m_p_dest = CDimension::n_size;
		++ m_p_dest;
	}

	/**
	 *	@brief cast to pointer
	 *	@return Returns pointer to the next output element.
	 */
	operator const T *() const
	{
		return m_p_dest;
	}
};

/**
 *	@brief helper object for copying typelist data to an array
 *	@tparam T is data type of the output array elements
 */
template <class T = std::pair<int, int> >
class CCopyCTSizes2DToArray {
protected:
	T *m_p_dest; /**< @brief pointer to the next output element */

public:
	/**
	 *	@brief default constructor; initializes the output pointer
	 *	@param[in] p_dest is pointer to the destination array (must be allocated to sufficient number of elements)
	 */
	CCopyCTSizes2DToArray(T *p_dest)
		:m_p_dest(p_dest)
	{}

	/**
	 *	@brief function operator; writes a single size in the output array
	 *	@tparam CDimension2D is a specialization of CCTSize2D (specifies the value to be written)
	 */
	template <class CDimension2D>
	void operator ()()
	{
		*m_p_dest = T(CDimension2D::n_row_num, CDimension2D::n_column_num);
		++ m_p_dest;
	}

	/**
	 *	@brief cast to pointer
	 *	@return Returns pointer to the next output element.
	 */
	operator const T *() const
	{
		return m_p_dest;
	}
};

/**
 *	@brief copies typelist of CCTSize to an array
 *
 *	@tparam CSizeList is typelist of CCTSize specializations
 *	@tparam T is data type of the output array elements
 *	@tparam n_size is size of the output array (must match the length of the list)
 *
 *	@param[out] p_dest is array of integers, filled with the data from the input typelist
 *
 *	@note This executes operations at run time.
 */
template <class CSizeList, class T, const size_t n_size>
void Copy_CTSizes_to_Array(T (&p_dest)[n_size])
{
	const T *p_end = CTypelistForEach<CSizeList, CCopyCTSizesToArray<T> >::Run(CCopyCTSizesToArray<T>(p_dest));
	_ASSERTE(p_end == p_dest + n_size); // make sure the size was correct
}

/**
 *	@brief copies typelist of CCTSize to an array
 *
 *	@tparam CSizeList is typelist of CCTSize specializations
 *	@tparam T is data type of the output array elements
 *
 *	@param[out] p_dest is array of integers, filled with the data from the input typelist
 *	@param[in] n_size is size of the output array (must match the length of the list; used only in debug)
 *
 *	@note This executes operations at run time.
 */
template <class CSizeList, class T>
void Copy_CTSizes_to_Array(T *p_dest, const size_t UNUSED(n_size))
{
	const T *p_end = CTypelistForEach<CSizeList, CCopyCTSizesToArray<T> >::Run(CCopyCTSizesToArray<T>(p_dest));
	_ASSERTE(p_end == p_dest + n_size); // make sure the size was correct
}

/**
 *	@brief copies typelist of CCTSize to an array
 *
 *	@tparam CSizeList is typelist of CCTSize specializations
 *	@tparam T is data type of the output array elements
 *	@tparam n_size is size of the output array (must match the length of the list)
 *
 *	@param[out] p_dest is array of integers, filled with the data from the input typelist
 *
 *	@note This executes operations at run time.
 */
template <class CSizeList, class T, const size_t n_size>
void Copy_CTSizes2D_to_Array(T (&p_dest)[n_size])
{
	const T *p_end = CTypelistForEach<CSizeList, CCopyCTSizes2DToArray<T> >::Run(CCopyCTSizes2DToArray<T>(p_dest));
	_ASSERTE(p_end == p_dest + n_size); // make sure the size was correct
}

/**
 *	@brief copies typelist of CCTSize to an array
 *
 *	@tparam CSizeList is typelist of CCTSize specializations
 *	@tparam T is data type of the output array elements
 *
 *	@param[out] p_dest is array of integers, filled with the data from the input typelist
 *	@param[in] n_size is size of the output array (must match the length of the list; used only in debug)
 *
 *	@note This executes operations at run time.
 */
template <class CSizeList, class T>
void Copy_CTSizes2D_to_Array(T *p_dest, const size_t UNUSED(n_size))
{
	const T *p_end = CTypelistForEach<CSizeList, CCopyCTSizes2DToArray<T> >::Run(CCopyCTSizes2DToArray<T>(p_dest));
	_ASSERTE(p_end == p_dest + n_size); // make sure the size was correct
}

template <class T = std::pair<int, int> >
class CCopyCTSizes2DForest {
protected:
	std::vector<std::vector<T> > &m_r_dest;

public:
	CCopyCTSizes2DForest(std::vector<std::vector<T> > &r_dest)
		:m_r_dest(r_dest)
	{
		r_dest.clear(); // !!
	}

	/**
	 *	@brief function operator; writes a single list in the output array
	 *	@tparam CSize2DList is a list of CCTSize2D specializations
	 */
	template <class CSize2DList>
	void operator ()() // throw(std::bad_alloc)
	{
		enum {
			n_len = CTypelistLength<CSize2DList>::n_result
		};
		m_r_dest.resize(m_r_dest.size() + 1);
		std::vector<T> &r_list = m_r_dest.back();
		r_list.resize(n_len);
		Copy_CTSizes2D_to_Array<CSize2DList, T>((n_len)? &r_list.front() : 0, r_list.size());
	}
};

template <class CSizeList, class T>
void Copy_CTSizes2DForest_to_Vector(std::vector<std::vector<T> > &r_dest) // throw(std::bad_alloc)
{
	CTypelistForEach<CSizeList, CCopyCTSizes2DForest<T> >::Run(CCopyCTSizes2DForest<T>(r_dest));
}

/**
 *	@brief constructor template
 *	@tparam n_size is value of the type being constructed
 *	@note This is useful e.g. with \ref ::CTypelistIOTA.
 */
template <const int n_size>
struct CCTSize_Constructor {
	typedef CCTSize<n_size> _TyResult; /**< @brief result type */
};

/**
 *	@brief sequence construction template
 *
 *	This constructs a sequence of integer stored as CCTSize with a given
 *	number of elements. The value of the first element can be adjusted
 *	(zero by default), the step by which the next elements increase can
 *	also be specified.
 *
 *	@tparam n_size is number of elements of the sequence (must be zero or greater)
 *	@tparam n_first is value associated with the first element
 *	@tparam n_step is value of the increment
 */
template <const int n_size, const int n_first = 0, const int n_step = 1>
class CTypelistIOTA {
/*protected:
	typedef CCTSize<n_first> _TyHead;
	typedef typename CTypelistIOTA<n_size - 1, n_first + n_step, n_step>::_TyResult _TyTail;*/

public:
	typedef /*CTypelist<_TyHead, _TyTail>*/ typename ::CTypelistIOTA<CCTSize_Constructor,
		n_size, n_first, n_step>::_TyResult _TyResult; /**< @brief resulting typelist */
	// reuse existing
};

/*template <const int n_first, const int n_step>
class CTypelistIOTA<0, n_first, n_step> {
public:
	typedef CTypelistEnd _TyResult;
};*/

/**
 *	@brief a very simple algorithm specializer which gets a list of lists
 *		of block sizes and allows choosing a suitable match based on matrix contents
 *
 *	Assume we have some block sizes that are expected in different scenarios.
 *	Those are represented as typelists:
 *
 *	@code
 *	typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>)) PoseSLAM_2D;
 *	typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<3, 3>, fbs_ut::CCTSize2D<2, 3>,
 *		fbs_ut::CCTSize2D<3, 2>, fbs_ut::CCTSize2D<2, 2>)) SLAM_2D;
 *	typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<6, 6>)) PoseSLAM_3D;
 *	typedef MakeTypelist_Safe((fbs_ut::CCTSize2D<6, 6>, fbs_ut::CCTSize2D<3, 6>,
 *		fbs_ut::CCTSize2D<6, 3>, fbs_ut::CCTSize2D<3, 3>)) SLAM_3D_or_BA;
 *	@endcode
 *
 *	It is possible to make another list of those scenarios and to use this class
 *	to make a object that can select the appropriate specialization, at runtime:
 *
 *	@code
 *	typedef MakeTypelist(PoseSLAM_2D, PoseSLAM_3D, SLAM_2D, SLAM_3D_or_BA) Specializations;
 *	// note that short lists go first because the list will be searched in this order
 *
 *	fbs_ut::CDummyAlgSpecializer::CData<Specializations> specializer; // not a typedef!
 *	@endcode
 *
 *	Now assume there is a block matrix (e.g. loaded from a file) and it is necessary
 *	to see if it is possible to select a specialization for it, from the list above:
 *
 *	@code
 *	CUberBlockMatrix lambda = ...;
 *
 *	fbs_ut::CDummyAlgSpecializer::TBSMix t_block_sizes =
 *		fbs_ut::CDummyAlgSpecializer::t_BlockSize_Mixture(lambda_perm, false);
 *	// get block sizes that can occur in lambda (can also filter that by the blocks
 *	// actually occurring in the matrix, by specifying true in the second argument)
 *
 *	size_t n_specialization_id = specializer.n_Find_SuitableSpecialization(t_block_sizes);
 *	if(n_specialization_id == size_t(-1))
 *		throw std::runtime_error("failed to find FBS algorithm specialization");
 *	// select which specialization can handle those (n_specialization is an index
 *	// in the Specializations list declared above)
 *	@endcode
 *
 *	Now that the specialization id is known, the algorithms can be selected based on its
 *	value. Assume a simple case of block Cholesky factorization:
 *
 *	@code
 *	CUberBlockMatrix R;
 *	R.CholeskyOf_FBS<BlockSizes>(lambda);
 *	@endcode
 *
 *	We need to determine what BlockSizes should be, based on the specialization id. For
 *	that, we need to wrap the above fragment:
 *
 *	@code
 *	template <const int n_specialization_id, class Specializations> // n_specialization_id is now constant
 *	class CCholeskyTest {
 *	public:
 *		static inline void Do(std::pair<CUberBlockMatrix*, const CUberBlockMatrix*> context)
 *		{
 *			typedef typename CTypelistItemAt<Specializations, n_specialization_id>::_TyResult BlockSizes;
 *			// block sizes for this specialization
 *
 *			const CUberBlockMatrix &lambda = *context.second; // input
 *			CUberBlockMatrix &R = *context.first; // output
 *
 *			R.CholeskyOf_FBS<BlockSizes>(lambda);
 *		}
 *	};
 *	@endcode
 *
 *	To call this fragment, one could use \ref CWrap2::In_ScalarSize_DecisionTree(). For convenience,
 *	this function is wrapped in \ref CDummyAlgSpecializer::CData::Run() or \ref CDummyAlgSpecializer::CData::Run2()
 *	in case additional types need to be passed. The use is simple:
 *
 *	@code
 *	CUberBlockMatrix R;
 *	specializer.Run<CCholeskyTest>(n_specialization_id, // what to run and which specialization
 *		std::make_pair(&R, (const CUberBlockMatrix*)&lambda_perm)); // the context (data out and in)
 *	@endcode
 *
 *	This selects the correct specialization and calls \ref CTestCholesky::Do() for the given data.
 *	The result will be left in R. Also note that both \ref CDummyAlgSpecializer::CData::Run() and
 *	\ref CDummyAlgSpecializer::CData::Run2() are static so they in fact do not need the value of
 *	<tt>specializer</tt>, they only need its type.
 *
 */
class CDummyAlgSpecializer {
public:
	typedef std::vector<std::pair<size_t, size_t> > TBSMix; /**< @brief block size mixture */

	/**
	 *	@brief runtime-generated decision data
	 *	@tparam CSize2DForest is typelist of typelists of CCTSize2D
	 */
	template <class CSize2DForest>
	class CData {
	public:
		typedef CSize2DForest _TySize2DForest; /**< @brief list of lists of supported block sizes */

		/**
		 *	@brief constants, stored as enum
		 */
		enum {
			n_specialization_num = CTypelistLength<_TySize2DForest>::n_result /**< @brief number of specializations */
		};

		typedef typename CTypelistIOTA<n_specialization_num>::_TyResult
			_TySelectorType; /**< @brief a dummy list of zero-based indices that can be used along with CWrap2::In_ScalarSize_DecisionTree() */

	protected:
		const std::vector<TBSMix> m_2D_sizes_forest; /**< @brief list of lists of supported block sizes */

	public:
		/**
		 *	@brief default constructor; unpacks the compile-time stored data to memory
		 *	@note This function throws std::bad_alloc.
		 */
		CData() // throw(std::bad_alloc)
			:m_2D_sizes_forest(t_Vec_Initializer())
		{
			//for(size_t i = 0, n = m_2D_sizes_forest.size(); i < n; ++ i)
			//	std::sort(m_2D_sizes_forest[i].begin(), m_2D_sizes_forest[i].end()); // make sure it is sorted
			// build the forest
		}

		/**
		 *	@brief finds a suitable specialization (such that it supports all the block sizes)
		 *	@param[in] r_matrix_blocks is a set of block sizes occurring in a matrix to be processed
		 *	@return Returns zero-based index of the corresponding entry in \ref _TySize2DForest
		 *		or <tt>size_t(-1)</tt> in case no specialization exists to handle this matrix.
		 *	@note The list of candidates is searched in order, shorter block lists should thus be at the beginning.
		 */
		size_t n_Find_SuitableSpecialization(const TBSMix &r_matrix_blocks) const
		{
			if(r_matrix_blocks.empty())
				return (m_2D_sizes_forest.empty())? size_t(-1) : 0;
			// no blocks in the matrix? does not matter then

			for(size_t i = 0, n = m_2D_sizes_forest.size(); i < n; ++ i) {
				const TBSMix &r_candidate = m_2D_sizes_forest[i];
				if(r_candidate.size() < r_matrix_blocks.size())
					continue;
				for(size_t j = 0, m = r_matrix_blocks.size();;) {
					//if(std::lower_bound(r_candidate.begin(), r_candidate.end(), // it is sorted, can use binary search
					if(std::find(r_candidate.begin(), r_candidate.end(), // it is usually small, this is likely faster
					   r_matrix_blocks[j]) == r_candidate.end())
						break;
					if(++ j == m)
						return i; // can use this one
				}
			}
			return size_t(-1);
		}

		template <template <const int, class> class CLoopType, class CContext>
		static void Run(size_t n_specialization_id, CContext t_context)
		{
			_ASSERTE(n_specialization_id < n_specialization_num);
			CWrap2<CLoopType, _TySize2DForest>::template
				In_ScalarSize_DecisionTree<_TySelectorType>(int(n_specialization_id), t_context);
		}

		template <template <const int, class> class CLoopType, class CStaticContext, class CContext>
		static void Run2(size_t n_specialization_id, CContext t_context)
		{
			_ASSERTE(n_specialization_id < n_specialization_num);
			typedef typename MakeTypelist(_TySize2DForest, CStaticContext) _TyStaticContext2;
			CWrap2<CLoopType, _TyStaticContext2>::template
				In_ScalarSize_DecisionTree<_TySelectorType>(int(n_specialization_id), t_context);
		}

		const TBSMix &t_Specialization(size_t n_specialization) const
		{
			return m_2D_sizes_forest[n_specialization];
		}

	protected:
		/**
		 *	@brief a dummy initializer so that \ref m_2D_sizes_forest can be const
		 *	@return Returns values of \ref _TySize2DForest unpacked to memory.
		 */
		static inline std::vector<TBSMix> t_Vec_Initializer() // throw(std::bad_alloc)
		{
			std::vector<TBSMix> v;
			Copy_CTSizes2DForest_to_Vector<_TySize2DForest>(v);
			return v;
		}
	};

	/**
	 *	@brief prints all the block sizes
	 *
	 */
	static void Print_BlockSizes(const TBSMix &r_sizes)
	{
		for(size_t i = 0, n = r_sizes.size(); i < n; ++ i)
			printf(", (" PRIsize " x " PRIsize ")" + ((i)? 0 : 2), r_sizes[i].first, r_sizes[i].second);
	}

	/**
	 *	@brief produces a lexicorgaphically sorted set of block sizes occurring in a given matrix
	 *
	 *	@param[in] r_matrix is sparse block matrix
	 *	@param[in] b_look_at_blocks is flag determining whether only the block sizes of blocks
	 *		actually in the matrix should be returned (default false; requires iterating over all blocks)
	 *	@param[in] n_many_sizes_threshold is threshold where enumerating all possible sizes from
	 *		block row and block column dimensions becomes more expensive than iterating over all the blocks
	 *		(only applies if b_look_at_blocks is set; default is 20)
	 *
	 *	@return Returns a lexicorgaphically sorted set of block sizes occurring in the given matrix.
	 */
	static TBSMix t_BlockSize_Mixture(const CUberBlockMatrix &r_matrix,
		bool b_look_at_blocks = false, size_t n_many_sizes_threshold = 20)
	{
		std::set<size_t> col_widths, row_heights;
		for(size_t i = 0, n = r_matrix.n_BlockRow_Num(); i < n; ++ i)
			row_heights.insert(r_matrix.n_BlockRow_Row_Num(i));
		if(b_look_at_blocks) {
			for(size_t i = 0, n = r_matrix.n_BlockColumn_Num(); i < n; ++ i) {
				if(r_matrix.n_BlockColumn_Block_Num(i))
					col_widths.insert(r_matrix.n_BlockColumn_Column_Num(i)); // only non-empty columns
			}
		} else {
			for(size_t i = 0, n = r_matrix.n_BlockColumn_Num(); i < n; ++ i)
				col_widths.insert(r_matrix.n_BlockColumn_Column_Num(i));
		}

		_ASSERTE(row_heights.empty() || col_widths.size() < SIZE_MAX / row_heights.size()); // make sure the line below does not overflow
		size_t n_size_num = col_widths.size() * row_heights.size();

		if(n_size_num > n_many_sizes_threshold && b_look_at_blocks) {
			std::set<std::pair<size_t, size_t> > blocks;
			for(size_t i = 0, n = r_matrix.n_BlockColumn_Num(); i < n; ++ i) {
				size_t n_width = r_matrix.n_BlockColumn_Column_Num(i);
				for(size_t j = 0, m = r_matrix.n_BlockColumn_Block_Num(i); j < m; ++ j) {
					size_t n_height = r_matrix.n_BlockRow_Row_Num(r_matrix.n_Block_Row(i, j));
					blocks.insert(std::make_pair(n_height, n_width));
				}
			}
			// make a set of block sizes that appear

			TBSMix blocks_linear(blocks.begin(), blocks.end());
			return blocks_linear;
			// convert to a linear vector and we're done
		}
		// seems like it is going to be faster to look at all the blocks, there are many possible mixtures, maybe not all are present

		TBSMix blocks_linear;
		blocks_linear.reserve(n_size_num);
		std::vector<size_t> col_widths_linear(col_widths.begin(), col_widths.end());
		for(std::set<size_t>::const_iterator p_it = row_heights.begin(),
		   p_end_it = row_heights.end(); p_it != p_end_it; ++ p_it) {
			size_t n_height = *p_it;
			for(size_t i = 0, n = col_widths_linear.size(); i < n; ++ i)
				blocks_linear.push_back(std::make_pair(n_height, col_widths_linear[i]));
		}
		_ASSERTE(blocks_linear.size() == n_size_num);
		_ASSERTE(CDebug::b_IsStrictlySortedSet(blocks_linear.begin(), blocks_linear.end()));
		// generate (lexicographically sorted) set product or row sizes x col sizes

		if(b_look_at_blocks) {
			std::set<std::pair<size_t, size_t> > blocks;
			for(size_t i = 0, n = r_matrix.n_BlockColumn_Num(); i < n; ++ i) {
				size_t n_width = r_matrix.n_BlockColumn_Column_Num(i);
				for(size_t j = 0, m = r_matrix.n_BlockColumn_Block_Num(i); j < m; ++ j) {
					size_t n_height = r_matrix.n_BlockRow_Row_Num(r_matrix.n_Block_Row(i, j));
					blocks.insert(std::make_pair(n_height, n_width));
					if(blocks.size() == n_size_num) {
#ifdef _DEBUG
						TBSMix blocks_linear2(blocks.begin(), blocks.end());
						_ASSERTE(blocks_linear == blocks_linear2); // those must be the same blocks
#endif // _DEBUG
						return blocks_linear;
					}
				}
			}
			// make a set of block sizes that appear

			TBSMix blocks_linear2(blocks.begin(), blocks.end());
			return blocks_linear2;
			// convert to a linear vector and we're done
		}
		// in case there is a suspicion that not all the sizes might actually be present ...

		return blocks_linear;
	}
};

} // ~fbs_ut

#include "slam/Self.h"
// it is recommended to use DECLARE_SELF and _TySelf in conjunction with fbs_ut::CWrap

/** @} */ // end of group

#endif // !__UBER_BLOCK_MATRIX_FIXED_BLOCK_SIZE_OPS_UTILITIES_INCLUDED
