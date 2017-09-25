// typelist debugging for fast compilation

#include <stdlib.h>
#include <stdio.h>
#include "slam/TypeList.h"
#include "slam/blockMatrix.h"

template <const int _n_value>
class CScalar {
public:
	enum {
		n_value = _n_value
	};

	static void Print()
	{
		printf("%d ", n_value);
	}
};

template <const int _n_value0, const int _n_value1>
class CSize2 {
public:
	enum {
		n_value0 = _n_value0,
		n_value1 = _n_value1
	};

	static void Print()
	{
		printf("(%d, %d) ", n_value0, n_value1);
	}
};

template <class _TL>
class CPrintTypelist {
public:
	static void Print()
	{
		typedef typename _TL::_TyHead THead;
		THead::Print();
		//printf("%d ", typename _TL::_TyHead::n_value);
		CPrintTypelist<typename _TL::_TyTail>::Print();
	}
};

template <>
class CPrintTypelist<CTypelistEnd> {
public:
	static void Print()
	{
		printf("\n");
	}
};

template <class _T1, class _T2>
class CMakePair {
public:
	typedef CSize2<_T1::n_value, _T2::n_value> _TyResult;
};

/**
 *	@brief binary addition operating on scalars
 *
 *	@tparam _T1 is the first operand (specialization of CScalar template)
 *	@tparam _T2 is the second operand (specialization of CScalar template)
 */
template <class _T1, class _T2>
class CBinaryScalarAdd {
public:
	typedef CScalar<_T1::n_value + _T2::n_value> _TyResult; /**< @brief result of the addition */
};

/**
 *	@brief binary less than or equal comparison operating on scalars
 *
 *	@tparam _T1 is the first operand (specialization of CScalar template)
 *	@tparam _T2 is the second operand (specialization of CScalar template)
 */
template <class _T1, class _T2>
class CCompareScalar_LEqual {
public:
	/**
	 *	@brief result stored as an enum
	 */
	enum {
		b_result = size_t(_T1::n_value) <= size_t(_T2::n_value) /**< @brief result of the comparison */
		// converting to size_t disables g++ warning: comparison between 'enum CScalar<X>::<anonymous>' and 'enum CScalar<Y>::<anonymous>'
	};
};

/**
 *	@brief binary less than comparison operating on scalars
 *
 *	@tparam _T1 is the first operand (specialization of CScalar template)
 *	@tparam _T2 is the second operand (specialization of CScalar template)
 */
template <class _T1, class _T2>
class CCompareScalar_Less {
public:
	/**
	 *	@brief result stored as an enum
	 */
	enum {
		b_result = size_t(_T1::n_value) < size_t(_T2::n_value) /**< @brief result of the comparison */
		// converting to size_t disables g++ warning: comparison between 'enum CScalar<X>::<anonymous>' and 'enum CScalar<Y>::<anonymous>'
	};
};

#if 0
template <class CConcatList, class CSizeList, const size_t n_max_size>
class CCombinatList {
protected:
	typedef typename CCarthesianProductTypelist<CConcatList,
		CSizeList, CBinaryScalarAdd>::_TyResult _TyAddList; /**< @brief typelist of the original list with new numbers added */
	typedef typename CUniqueTypelist<typename CConcatTypelist<CConcatList,
		_TyAddList>::_TyResult>::_TyResult _TyConcatList;
	typedef typename CFilterTypelist<_TyConcatList, CScalar<n_max_size>,
		CBinaryScalarLEqual>::_TyResult _TyFilteredList;

	enum {
		b_list_was_extended = !CEqualType<CConcatList, _TyFilteredList>::b_result
	};

public:
	typedef typename CCombinatList<_TyFilteredList, CSizeList,
		(b_list_was_extended)? n_max_size : 0>::_TyResult _TyResult;
};

template <class CConcatList, class CSizeList>
class CCombinatList<CConcatList, CSizeList, 0> {
public:
	typedef CConcatList _TyResult;
};

template <class CSizeList, const size_t n_max_size>
class CBinaryCombinationTypelist {
public:
	typedef typename CCombinatList<CSizeList, CSizeList, n_max_size>::_TyResult _TyResult;
};

template <class CConcatList, class CSizeList, const size_t n_max_size>
class CCombinatList {
protected:
	typedef typename CCarthesianProductTypelist<CConcatList,
		CSizeList, CBinaryScalarAdd>::_TyResult _TyAddList; /**< @brief typelist of the original list with new numbers added */
	typedef typename CFilterTypelist<_TyAddList, CScalar<n_max_size>,
		CBinaryScalarLEqual>::_TyResult _TyFilteredList; /**< @brief original list with new numbers added, only smaller than n_max_size */

	enum {
		b_list_was_extended = !CEqualType<CConcatList, _TyFilteredList>::b_result /**< @brief flag whether to continue expansion or whether any addition will end above the limit and be cut off */
	};

	typedef typename CCombinatList<_TyFilteredList, CSizeList,
		(b_list_was_extended)? n_max_size : 0>::_TyResult _TyNextList; /**< @brief another round of addition (the original list need not be included) */

public:
	typedef typename CConcatTypelist<CConcatList, _TyNextList>::_TyResult _TyResult; /**< @brief the resulting list */
};

template <class CConcatList, class CSizeList>
class CCombinatList<CConcatList, CSizeList, 0> {
public:
	typedef CConcatList _TyResult; /**< @brief the resulting list */
};

template <class CSizeList, const size_t n_max_size>
class CBinaryCombinationTypelist {
public:
	typedef typename CUniqueTypelist<typename CCombinatList<CSizeList,
		CSizeList, n_max_size>::_TyResult>::_TyResult _TyResult;
};

template <class CSizeList, const size_t n_max_size>
class CBinaryCombinationTypelist {
protected:
	typedef typename CCarthesianProductTypelist<CSizeList,
		CSizeList, CBinaryScalarAdd>::_TyResult _TyAddList; /**< @brief typelist of the original list with new numbers added */
	typedef typename CUniqueTypelist<typename CConcatTypelist<CSizeList,
		_TyAddList>::_TyResult>::_TyResult _TyConcatList;
	typedef typename CFilterTypelist<_TyConcatList, CScalar<n_max_size>,
		CBinaryScalarLEqual>::_TyResult _TyFilteredList;

	enum {
		b_list_was_extended = !CEqualType<CSizeList, _TyFilteredList>::b_result
	};

public:
	typedef typename CBinaryCombinationTypelist<_TyFilteredList,
		(b_list_was_extended)? n_max_size : 0>::_TyResult _TyResult;
};

template <class CSizeList>
class CBinaryCombinationTypelist<CSizeList, 0> {
public:
	typedef CSizeList _TyResult;
};

#endif // 0
// failed variants (working, but much slower)

template <class CList>
class CMakeDecissionTree {
protected:
	template <class CList, const int n_begin, const int n_length>
	class CDecissionTree {
	public:
		enum {
			n_half = (n_length + 1) / 2,
			n_pivot = n_begin + n_half,
			n_rest = n_length - n_half
		};

		typedef typename CTypelistItemAt<CList, n_pivot>::_TyResult _TyPivot;

		static void Debug()
		{
			//printf("if(x < i[%d]) {\n", n_pivot);
			printf("if(x < %d) {\n", _TyPivot::n_value);
			CDecissionTree<CList, n_begin, n_half>::Debug();
			printf("} else {\n");
			CDecissionTree<CList, n_pivot, n_rest>::Debug();
			printf("}\n");
		}
	};

	template <class CList, const int n_begin>
	class CDecissionTree<CList, n_begin, 1> {
	public:
		typedef typename CTypelistItemAt<CList, n_begin>::_TyResult _TyPivot;

		static void Debug()
		{
			//printf("_ASSERTE(x == i[%d]);\n", n_begin);
			printf("_ASSERTE(x == %d);\n", _TyPivot::n_value);
		}
	};

public:
	static void Debug()
	{
		CDecissionTree<CList, 0, CTypelistLength<CList>::n_result>::Debug();
	}
};

/*template <class CWidthList, const int n_max_matrix_size>
class CMakeSquareMatrixSizeDecisionTree {
protected:
	typedef CBinaryCombinationTypelist<primes, CBinaryScalarAdd,
		CCompareScalar_LEqual, CScalar<n_max_matrix_size> >::_TyResult matrixSizeList2;
	typedef CSortTypelist<matrixSizeList2, CCompareScalar_Less>::_TyResult sortedList;
};*/

void dtlmain()
{
	typedef CTypelistEnd epsilon;
	typedef MakeTypelist_Safe((CScalar<1>, CScalar<2>, CScalar<3>)) one;
	typedef MakeTypelist_Safe((CScalar<4>, CScalar<5>, CScalar<6>)) two;

	CPrintTypelist<one>::Print();
	CPrintTypelist<two>::Print();

	/*typedef CConcatTypelist<one, two>::_TyResult mr_onetwo;
	typedef CConcatTypelist<two, one>::_TyResult mr_twoone;
	typedef CConcatTypelist<two, two>::_TyResult mr_twotwo;
	CPrintTypelist<mr_onetwo>::Print();
	CPrintTypelist<mr_twoone>::Print();
	CPrintTypelist<mr_twotwo>::Print();

	typedef CConcatTypelist<one, epsilon>::_TyResult mr_onee;
	typedef CConcatTypelist<two, epsilon>::_TyResult mr_twoe;
	typedef CConcatTypelist<epsilon, one>::_TyResult mr_eone;
	typedef CConcatTypelist<epsilon, two>::_TyResult mr_etwo;
	typedef CConcatTypelist<epsilon, epsilon>::_TyResult mr_ee;
	CPrintTypelist<mr_onee>::Print();
	CPrintTypelist<mr_twoe>::Print();
	CPrintTypelist<mr_eone>::Print();
	CPrintTypelist<mr_etwo>::Print();
	CPrintTypelist<mr_ee>::Print();

	typedef CLeftProductTypelist<CScalar<7>, one, CMakePair>::_TyResult _7_x_one;
	CPrintTypelist<_7_x_one>::Print();
	typedef CCarthesianProductTypelist<one, MakeTypelist_Safe((CScalar<7>)), CMakePair>::_TyResult one_x_7;
	CPrintTypelist<one_x_7>::Print();
	typedef CCarthesianProductTypelist<MakeTypelist_Safe((CScalar<8>)), one, CMakePair>::_TyResult _8_x_one;
	CPrintTypelist<_8_x_one>::Print();
	typedef CCarthesianProductTypelist<one, two, CMakePair>::_TyResult one_x_two;
	CPrintTypelist<one_x_two>::Print();
	typedef CCarthesianProductTypelist<one, one, CMakePair>::_TyResult one_x_one;
	CPrintTypelist<one_x_one>::Print();
	typedef CCarthesianProductTypelist<one, epsilon, CMakePair>::_TyResult one_x_epsilon;
	CPrintTypelist<one_x_epsilon>::Print();
	typedef CCarthesianProductTypelist<epsilon, two, CMakePair>::_TyResult epsilon_x_two;
	CPrintTypelist<epsilon_x_two>::Print();
	typedef CCarthesianProductTypelist<epsilon, epsilon, CMakePair>::_TyResult epsilon_2;
	CPrintTypelist<epsilon_2>::Print();

	typedef MakeTypelist_Safe((Eigen::Matrix3d, Eigen::Matrix2d)) orig;
	typedef fbs_ut::CBlockSizesAfterPreMultiplyWithSelfTranspose<orig>::_TyResult ata;

	fbs_ut::CDumpBlockMatrixTypelist<orig>::Print();
	fbs_ut::CDumpBlockMatrixTypelist<ata>::Print();*/

	/*typedef CBinaryCombinationTypelist<one, 50>::_TyResult matrixSizeList;
	CPrintTypelist<matrixSizeList>::Print();*/

	/*typedef MakeTypelist_Safe((CScalar<3>, CScalar<2>)) primes;

	typedef CBinaryCombinationTypelist<primes, CBinaryScalarAdd,
		CCompareScalar_LEqual, CScalar<20> >::_TyResult matrixSizeList2;
	typedef CSortTypelist<matrixSizeList2, CCompareScalar_Less>::_TyResult sortedList;
	CPrintTypelist<matrixSizeList2>::Print();
	CPrintTypelist<sortedList>::Print();

	typedef MakeTypelist_Safe((CScalar<1>, CScalar<2>, CScalar<3>, CScalar<4>,
		CScalar<5>, CScalar<6>)) three;
	CMakeDecissionTree<three>::Debug();*/

	typedef MakeTypelist_Safe((Eigen::Matrix3d, Eigen::Matrix2d)) orig;
	typedef fbs_ut::CBlockSizesAfterPreMultiplyWithSelfTranspose<orig>::_TyResult ata;
	fbs_ut::CMakeSquareMatrixSizeDecisionTree<ata, 10>::Debug();
}

class CTypelistDebugger {
public:
	CTypelistDebugger()
	{
		dtlmain();
		exit(0);
	}
} run_it;
