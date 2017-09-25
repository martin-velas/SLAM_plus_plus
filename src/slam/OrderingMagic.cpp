/*
								+-----------------------------------+
								|                                   |
								| *** Matrix ordering utilities *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|         OrderingMagic.cpp         |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/OrderingMagic.cpp
 *	@brief matrix ordering utilities
 *	@author -tHE SWINe-
 *	@date 2013-02-07
 */

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma warning(disable: 4996)
// get rid of VS warning "C4996: 'std::copy': Function call with parameters that may be
// unsafe - this call relies on the caller to check that the passed values are correct"
#endif // _MSC_VER && !__MWERKS__
// VS 2015 requires this to be here, before the headers are included

#include <math.h>
#include <stdio.h>
#include "csparse/cs.hpp"
#include "cholmod/AMD/amd.h"
#include "cholmod/CAMD/camd.h" // a bit unfortunate, now there are two camd.h, one in the global namespace and the other in __camd_internal
#include "slam/Timer.h"

#include "slam/OrderingMagic.h"

#define __MATRIX_ORDERING_USE_UBM_AAT 1

#ifdef __MATRIX_ORDERING_USE_AMD1

/**
 *	@brief hides functions from camd_internal.h which are not normally user-callable
 */
namespace __camd_internal {

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#ifndef DLONG
#define DLONG
#endif // DLONG
#ifdef NLONG
#undef NLONG
#endif // NLONG
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
// makes sure that DLONG is defined in x64 (for CAMD)

extern "C" {
#include "cholmod/CAMD/camd_internal.h" // required by some other blah

#if defined(NDEBUG) && defined(_DEBUG)
#undef NDEBUG
#endif // NDEBUG && _DEBUG
// camd_internal.h defines NDEBUG if it is not defined alteady; might confuse programs

}
// t_odo - put this inside another file, this is inconvenient

}

/**
 *	@brief hides functions from amd_internal.h which are not normally user-callable
 */
namespace __amd_internal {

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#ifndef DLONG
#define DLONG
#endif // DLONG
#ifdef NLONG
#undef NLONG
#endif // NLONG
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
// makes sure that DLONG is defined in x64 (for AMD)

extern "C" {

#ifdef PRINTF
#undef PRINTF
#endif // !PRINTF
// avoid macro redefinition in amd_internal.h

#include "cholmod/AMD/amd_internal.h" // required by some other blah

#if defined(NDEBUG) && defined(_DEBUG)
#undef NDEBUG
#endif // NDEBUG && _DEBUG
// amd_internal.h defines NDEBUG if it is not defined alteady; might confuse programs

}

}

#endif // __MATRIX_ORDERING_USE_AMD1

/*
 *								=== global ===
 */

/**
 *	@brief debugging predicate
 *	@tparam _Ty is an integral type
 *	@param[in] n_x is a number
 *	@return Returns true if the given number is nonzero, otherwise returns false.
 */
template <class _Ty>
static inline bool b_IsNonzero(_Ty n_x) // just for debugging
{
	return n_x != 0;
}

/*
 *								=== ~global ===
 */

/*
 *								=== CLastElementOrderingConstraint ===
 */

const size_t *CLastElementOrderingConstraint::p_Get(size_t n_size) // throw(std::bad_alloc)
{
#if defined(__MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT)
	const size_t n_groups_size = 14; // todo move this to the template arg or something
#endif // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT

	if(!m_constraints.empty()) {
		if(m_constraints.capacity() < n_size)
			m_constraints.clear(); // memset() cheaper than memcpy()
		else {
#if defined(__MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT)
			_ASSERTE(m_constraints.back() == ((m_constraints.size() == 1)? 0 :
				((m_constraints.size() > n_groups_size)? 2 : 1))); // t_odo copy to L as well
			size_t n = m_constraints.size();
			for(size_t i = n - std::min(n, n_groups_size); i < n; ++ i)
				m_constraints[i] = 0;
#else // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
			_ASSERTE(m_constraints.back() == 1);
			m_constraints.back() = 0;
#endif // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
			// there is a 'one' at the end,
		}
	}
	m_constraints.resize(n_size, 0); // resize to contain all zeroes
	_ASSERTE(std::find_if(m_constraints.begin(), m_constraints.end(),
		b_IsNonzero<size_t>) == m_constraints.end()); // makes sure it's all zeroes
	// maintain vector of zeroes (and do it efficiently)

#if defined(__MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT)
	if(n_size <= 2)
		m_constraints[n_size - 1] = (n_size > 1)? 1 : 0; // either "0" or "0, 1"
	else if(n_size > n_groups_size) {
		for(size_t i = n_size - n_groups_size; i < n_size; ++ i)
			m_constraints[i] = 1;
		m_constraints[n_size - 1] = 2; // "0, 0, 0, ... , 0, 1, 1, 1, ... , 1, 2"
	} else {
		for(size_t i = 0; i < n_size; ++ i)
			m_constraints[i] = 0;
		m_constraints[n_size - 1] = 1; // "0, 0, 0, ... , 0, 1"
	}
	// make two groups, one forcing the last pose to be the last and the other forcing several other recent poses to still be at the end
#else // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
	m_constraints[n_size - 1] = 1; // impose no constraint on ordering // t_odo remove me
	// put a one at the end
#endif // __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT

	return &m_constraints[0];
	// return pointer for use by camd / ccolamd
}

/*
 *								=== ~CLastElementOrderingConstraint ===
 */

/*
 *								=== CFirstLastElementOrderingConstraint ===
 */

const size_t *CFirstLastElementOrderingConstraint::p_Get(size_t n_size) // throw(std::bad_alloc)
{
	const size_t n_groups_size = 14; // todo move this to the template arg or something

	if(m_constraints.capacity() < n_size) {
		m_constraints.clear();
		m_constraints.reserve(std::max(n_size, 2 * m_constraints.capacity()));
	}
	m_constraints.resize(n_size);

	if(n_size == 1)
		m_constraints[0] = 0;
	else if(n_size == 2) {
		m_constraints[0] = 0;
		m_constraints[1] = 1;
	} else if(n_size <= 2 + n_groups_size) {
		m_constraints[0] = 0;
		_ASSERTE(n_size - 1 > 1); // fill at least one
		for(size_t i = 1; i < n_size - 1; ++ i)
			m_constraints[i] = 1;
		m_constraints[n_size - 1] = 2;
	} else {
		m_constraints[0] = 0;
		_ASSERTE(n_size - (n_groups_size + 1) > 1); // fill at least one
		for(size_t i = 1; i < n_size - (n_groups_size + 1); ++ i)
			m_constraints[i] = 1;
		_ASSERTE(n_size - (n_groups_size + 1) < n_size - 1); // fill at least one
		for(size_t i = n_size - (n_groups_size + 1); i < n_size - 1; ++ i)
			m_constraints[i] = 2;
		m_constraints[n_size - 1] = 3;
	}
	// fixes the first elem as well; very complicated (don't even attempting to do lazy updates on that)

	return &m_constraints[0];
	// return pointer for use by camd / ccolamd
}

/*
 *								=== ~CFirstLastElementOrderingConstraint ===
 */

/*
 *								=== CNFirst1LastElementOrderingConstraint ===
 */

const size_t *CNFirst1LastElementOrderingConstraint::p_Get(size_t n_size, size_t n_first_constraint_num) // throw(std::bad_alloc)
{
	const size_t n_groups_size = 14; // 14 was the best on VP, molson35 and kill

	if(m_constraints.capacity() < n_size) {
		m_constraints.clear();
		m_constraints.reserve(std::max(n_size, 2 * m_constraints.capacity()));
	}
	m_constraints.resize(n_size);

	_ASSERTE(n_size > 0 && n_first_constraint_num > 0);
	if(n_size == 1)
		m_constraints[0] = 0;
	else if(n_size == 2) {
		m_constraints[0] = 0;
		m_constraints[1] = 1;
	} else if(n_size <= n_first_constraint_num + 1) { // onle the constraints and the last one
		_ASSERTE(n_size - 1 > 0); // fill at least one
		for(size_t i = 0; i < n_size - 1; ++ i)
			m_constraints[i] = i; // constrain all
		m_constraints[n_size - 1] = n_size - 1;
	} else if(n_size <= n_first_constraint_num + 1 + n_groups_size) { // only the constraints, the last group and the last one
		_ASSERTE(n_first_constraint_num > 0); // fill at least one
		for(size_t i = 0; i < n_first_constraint_num; ++ i)
			m_constraints[i] = i; // constrain all
		_ASSERTE(n_first_constraint_num < n_size - 1); // fill at least one
		_ASSERTE(n_first_constraint_num + n_groups_size >= n_size - 1); // fill at most n_groups_size
		for(size_t i = n_first_constraint_num; i < n_size - 1; ++ i)
			m_constraints[i] = n_first_constraint_num; // the first last group
		m_constraints[n_size - 1] = n_first_constraint_num + 1; // the second last group
	} else { // the constraints, free elements, last group, last one
		_ASSERTE(n_first_constraint_num > 0); // fill at least one
		for(size_t i = 0; i < n_first_constraint_num; ++ i)
			m_constraints[i] = i; // constrain all
		_ASSERTE(n_first_constraint_num < n_size - (n_groups_size + 1)); // fill at least one
		for(size_t i = n_first_constraint_num; i < n_size - (n_groups_size + 1); ++ i)
			m_constraints[i] = n_first_constraint_num; // free elements
		_ASSERTE(n_size - (n_groups_size + 1) < n_size - 1); // fill at least one
		for(size_t i = n_size - (n_groups_size + 1); i < n_size - 1; ++ i)
			m_constraints[i] = n_first_constraint_num + 1; // last group
		m_constraints[n_size - 1] = n_first_constraint_num + 2; // last one
	}
	// fixes the first elem as well; very complicated (don't even attempting to do lazy updates on that)

	return &m_constraints[0];
	// return pointer for use by camd / ccolamd
}

/*
 *								=== ~CNFirst1LastElementOrderingConstraint ===
 */

/*
 *								=== CMatrixOrdering ===
 */

CMatrixOrdering::~CMatrixOrdering()
{
	if(m_p_block)
		cs_spfree(m_p_block);
}

const size_t *CMatrixOrdering::p_InvertOrdering(const size_t *p_ordering, size_t n_ordering_size) // throw(std::bad_alloc)
{
	_ASSERTE(m_ordering_invert.empty() || p_ordering != &m_ordering_invert[0]); // can't invert twice
	if(m_ordering_invert.capacity() < n_ordering_size) {
		m_ordering_invert.clear();
		m_ordering_invert.reserve(std::max(n_ordering_size, 2 * m_ordering_invert.capacity()));
	}
	m_ordering_invert.resize(n_ordering_size);
	for(size_t i = 0; i < n_ordering_size; ++ i)
		m_ordering_invert[p_ordering[i]] = i;
	return &m_ordering_invert[0];
}

const size_t *CMatrixOrdering::p_ExtendBlockOrdering_with_SubOrdering(size_t n_order_min,
	const size_t *p_sub_block_ordering, size_t n_sub_block_size) // throw(std::bad_alloc)
{
	_ASSERTE(n_order_min + n_sub_block_size >= m_ordering.size());
	_ASSERTE(n_order_min <= m_ordering.size());
	if(n_order_min < m_ordering.size()) { // probably all cases in SLAM (otherwise would have a disconnected graph)
#if 1
		// this is working well and it works on direct ordering

		if(m_ordering_invert.capacity() < n_sub_block_size) {
			m_ordering_invert.clear();
			m_ordering_invert.reserve(std::max(n_sub_block_size, 2 * m_ordering_invert.capacity()));
		}
		m_ordering_invert.resize(n_sub_block_size);
		size_t *p_temp = &m_ordering_invert[0];
		// will use the inverse ordering as temporary ordering (can't do that inplace)

		size_t n_old_size = m_ordering.size();
		for(size_t i = 0; i < n_sub_block_size; ++ i) {
			size_t n_order = p_sub_block_ordering[i] + n_order_min;
			_ASSERTE(n_order >= n_order_min);
			p_temp[i] = (n_order < n_old_size)? m_ordering[n_order] : n_order;
		}
		// permute the end of the old ordering using the new ordering, extend with identity where needed

		m_ordering.erase(m_ordering.begin() + n_order_min, m_ordering.end()); // !!
		m_ordering.insert(m_ordering.end(), p_temp, p_temp + n_sub_block_size);
		// append with the new ordering
#elif 0
		// buggy

		size_t *p_inv_orig = &m_ordering_invert[n_sub_block_size];
		// will use the inverse ordering as temporary ordering (can't do that inplace)

		size_t n_old_size = m_ordering.size();
		for(size_t i = 0; i < n_order_min + n_sub_block_size; ++ i) {
			size_t n_order = (i < n_old_size)? m_ordering[i] : i;
			// extend the new ordering with identity

			if(n_order >= n_order_min) {
				_ASSERTE(n_order < n_order_min + n_sub_block_size);
				p_inv_orig[n_order - n_order_min] = i;
			}
		}
		// invert the original ordering to a temp array

		for(size_t i = 0; i < n_sub_block_size; ++ i) {
			size_t n_order = p_sub_block_ordering[i] + n_order_min;
			_ASSERTE(n_order >= n_order_min);
			p_temp[i] = p_inv_orig[n_order - n_order_min];//(n_order < n_old_size)? m_ordering[n_order] : n_order; // permute in inverse
		}
		// permute the end of the old ordering using the new ordering, extend with identity where needed

		m_ordering.erase(m_ordering.begin() + n_order_min, m_ordering.end()); // !!
		m_ordering.insert(m_ordering.end(), p_temp, p_temp + n_sub_block_size);
#elif 0
		// buggy

		_ASSERTE(m_ordering_invert.size() >= m_ordering.size()); // make sure the inverse is up-to-date
		// this works with inverse since we are ordering on an already ordered matrix

		std::vector<size_t> p_temp(n_sub_block_size);
		size_t n_old_size = m_ordering.size();
		for(size_t i = 0; i < n_sub_block_size; ++ i) {
			size_t n_order = p_sub_block_ordering[i] + n_order_min;
			_ASSERTE(n_order >= n_order_min);
			p_temp[i] = (n_order < n_old_size)? m_ordering_invert[n_order] : n_order;
		}
		// permute the end of inverse of the old ordering using the new ordering, extend with identity where needed

		m_ordering_invert.erase(m_ordering_invert.begin() + n_order_min, m_ordering_invert.end()); // !!
		m_ordering_invert.insert(m_ordering_invert.end(), p_temp.begin(), p_temp.begin() + n_sub_block_size);
		// append with the new ordering

		size_t n_new_size = m_ordering_invert.size();
		if(m_ordering.capacity() < n_new_size) {
			m_ordering.clear();
			m_ordering.resize(std::max(n_new_size, 2 * m_ordering.capacity()));
		}
		for(size_t i = 0; i < n_new_size; ++ i)
			m_ordering[m_ordering_invert[i]] = i;
		// inverse back
#else
		// this is working well and it works on inverse ordering

		size_t n_old_size = m_ordering.size();
		size_t n_new_size = n_order_min + n_sub_block_size;
		m_ordering.resize(n_new_size);
		for(size_t i = n_old_size; i < n_new_size; ++ i)
			m_ordering[i] = i;
		// extend with identity

		p_InvertOrdering(&m_ordering[0], m_ordering.size());

		for(size_t i = 0; i < n_new_size; ++ i) {
			size_t n_order = m_ordering_invert[i];
			m_ordering[(n_order < n_order_min)? n_order :
				p_sub_block_ordering[n_order - n_order_min] + n_order_min] = i;
		}
		// reorder the order
#endif
	} else {
		_ASSERTE(n_order_min == m_ordering.size());
		m_ordering.resize(n_order_min + n_sub_block_size);
		for(size_t i = 0; i < n_sub_block_size; ++ i)
			m_ordering[n_order_min + i] = p_sub_block_ordering[i] + n_order_min;
		// just resize and append the ordering with the new numbers
	}

	return &m_ordering[0];
}

const size_t *CMatrixOrdering::p_BlockOrdering(const CUberBlockMatrix &r_A, const size_t *p_constraints,
	size_t UNUSED(n_constraints_size), bool b_need_inverse /*= false*/, bool b_A_is_upper_triangular /*= true*/) // throw(std::bad_alloc)
{
	_ASSERTE(r_A.b_Square_BlockSquare());
	_ASSERTE(!p_constraints || n_constraints_size == r_A.n_BlockColumn_Num());

#if !__MATRIX_ORDERING_USE_UBM_AAT
	if(!(m_p_block = r_A.p_BlockStructure_to_Sparse(m_p_block)))
		throw std::bad_alloc(); // rethrow
	// get block structure of A

	return p_DestructiveOrdering(m_p_block, p_constraints,
		n_constraints_size, b_need_inverse);
	// calculate ordering by block, can destroy the pattern matrix in the process
#else // !__MATRIX_ORDERING_USE_UBM_AAT
	const size_t m = r_A.n_BlockRow_Num(), n = r_A.n_BlockColumn_Num();
	_ASSERTE(m == n); // only square matrices (could support other matrices though)

	_ASSERTE(!p_constraints || n_constraints_size == n);

	_ASSERTE(n >= 0 && n <= SIZE_MAX);
	if(m_ordering.capacity() < n) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(n, 2 * m_ordering.capacity()));
	}
	m_ordering.resize(n);
	// maintain an ordering array

	if(b_need_inverse) {
		if(m_ordering_invert.capacity() < n) {
			m_ordering_invert.clear(); // avoid copying data in resizing
			m_ordering_invert.reserve(std::max(n, 2 * m_ordering_invert.capacity()));
		}
		m_ordering_invert.resize(n);
	}
	// allocate an extra inverse ordering array, if needed

	if(n >= SIZE_MAX / sizeof(size_t) /*||
	   nz >= SIZE_MAX / sizeof(size_t)*/)
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY

#ifdef __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_alignment = __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES / sizeof(size_t);
	const size_t n_al = n_Align_Up_POT(n, n_alignment);
	const size_t n_al1 = n_Align_Up_POT(n + 1, n_alignment);
#else // __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_alignment = 1;
	const size_t n_al = n;
	const size_t n_al1 = n + 1;
#endif // __MATRIX_ORDERING_CACHE_ALIGN
	// to align, or not to align ...

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
	_ASSERTE(sizeof(size_t) > 1); // note 2 * n does not overflow if sizeof(size_t) > 1
	if(m_camd_workspace.capacity() < ((b_need_inverse)? n_al : n + n_al)) {
		m_camd_workspace.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace.reserve(std::max((b_need_inverse)? n_al : n + n_al, m_camd_workspace.capacity() * 2));
	}
	m_camd_workspace.resize((b_need_inverse)? n_al : n + n_al);
	size_t *Len = &m_camd_workspace[0];
	size_t *Pinv = (b_need_inverse)? &m_ordering_invert[0] : &m_camd_workspace[n_al]; // this is actually inverse ordering, we need it most of the time, it is a waste to throw it away
	// alloc workspace

	enum {
		b_upper = true, // lambda in SLAM++ is always upper triangular
		b_sorted = false // lambda is supposed to be symmetric, it will be sorted regardless
	};
	size_t nzaat;
	if(b_A_is_upper_triangular) {
		nzaat = r_A.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<b_upper,
			!b_upper, false, false>(Len, n, Pinv, n); // use Pinv as workspace
	} else {
		nzaat = r_A.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<false,
			true, false, false>(Len, n, Pinv, n); // use Pinv as workspace
	}

	size_t slen = nzaat;
	if(slen > SIZE_MAX - (n_alignment - 1)) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += n_alignment - 1;
	if(slen > SIZE_MAX - nzaat / 5) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += nzaat / 5;
	if(n_al1 > SIZE_MAX / 9 || slen > SIZE_MAX - 9 * n_al1) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += 9 * n_al1;
	if(m_camd_workspace1.size() < slen) {
		m_camd_workspace1.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace1.resize(std::max(slen, m_camd_workspace1.size() * 2));
	}
	_ASSERTE(m_camd_workspace1.size() >= slen);
	slen = m_camd_workspace1.size();
	// allocate workspace for matrix, elbow room (size n), and 7 (n + alignment) vectors

	size_t iwlen = slen - 5 * n_al - 2 * n_al1;
	size_t *Pe, *Nv, *Head, *Elen, *Degree, *W, *Iw, *BucketSet;
	{
		size_t *S = &m_camd_workspace1[0];

#ifdef __MATRIX_ORDERING_CACHE_ALIGN
		size_t n_align = n_Align_Up_POT(ptrdiff_t(S), ptrdiff_t(n_alignment)) - ptrdiff_t(S);
		S += n_align;
		iwlen -= n_align; // !!
		// make sure the first pointer is aligned (allocated n_alignment - 1 more space)
#endif // __MATRIX_ORDERING_CACHE_ALIGN

		Pe = S;		S += n_al;
		Nv = S;		S += n_al;
		Head = S;	S += n_al1; // note: 1 more than AMD
		Elen = S;	S += n_al;
		Degree = S;	S += n_al;
		W = S;		S += n_al1; // note: 1 more than AMD
		BucketSet = S;
		Iw = S + n_al;
	}
	// one array actually stores many smaller ones

	_ASSERTE(iwlen == &m_camd_workspace1.back() + 1 - Iw); // make sure we got this right (the + 1 is so that we get the one-past-the-last pointer)
	// one array actually stores many smaller ones // t_odo play with cache here? could align n to 64B offset so that each array fills different cache entry (if 6bit cache off)

	const size_t pfree = nzaat; // this is needed in amd_l2(), equals nzaat
	_ASSERTE(iwlen >= pfree + n); // as dictated by amd_l2()

	if(b_A_is_upper_triangular) {
		r_A.BlockStructure_SumWithSelfTranspose<b_upper, !b_upper, false, false, b_sorted>(Pe,
			n + 1, Iw, nzaat, Len, n, Head, n_al); // can't use Nv as workspace because Nv[0] will contain value of nzaat if there is no alignment; could use Pinv or Head then (Head is allocated to cache line, maybe better)
	} else {
		r_A.BlockStructure_SumWithSelfTranspose<false, true, false, false, b_sorted>(Pe,
			n + 1, Iw, nzaat, Len, n, Head, n_al);
	}
	// calculate structure of A+AT in Pe, Iw

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		camd_l2(n, (SuiteSparse_long*)Pe, (SuiteSparse_long*)Iw,
			(SuiteSparse_long*)Len, iwlen, pfree, (SuiteSparse_long*)Nv,
			(SuiteSparse_long*)Pinv, (SuiteSparse_long*)&m_ordering[0],
			(SuiteSparse_long*)Head, (SuiteSparse_long*)Elen,
			(SuiteSparse_long*)Degree, (SuiteSparse_long*)W, 0, 0,
			(SuiteSparse_long*)p_constraints, (SuiteSparse_long*)BucketSet);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		camd_2(n, (int*)Pe, (int*)Iw, (int*)Len, iwlen, pfree, (int*)Nv, (int*)Pinv,
			(int*)&m_ordering[0], (int*)Head, (int*)Elen, (int*)Degree, (int*)W, 0, 0,
			(int*)p_constraints, (int*)BucketSet);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

	if(b_need_inverse) {
		size_t *p_ordering = &m_ordering[0];
		for(size_t i = 0; i < n; ++ i)
			Pinv[p_ordering[i]] = i;
	}
	// CAMD does not output the inverse ordering on output, at least not in this version

	return &m_ordering[0];
	// return ordering
#endif // !__MATRIX_ORDERING_USE_UBM_AAT
}

const size_t *CMatrixOrdering::p_BlockOrdering_MiniSkirt(const CUberBlockMatrix &r_A,
	size_t n_matrix_cut, size_t n_matrix_diag, const size_t *p_constraints,
	size_t UNUSED(n_constraints_size), bool b_need_inverse) // throw(std::bad_alloc)
{
	_ASSERTE(r_A.b_Square_BlockSquare());
	_ASSERTE(!p_constraints || n_constraints_size == r_A.n_BlockColumn_Num() - n_matrix_cut);

	if(!(m_p_block = r_A.p_BlockStructure_to_Sparse_Apron(n_matrix_cut, n_matrix_diag, m_p_block)))
		throw std::bad_alloc(); // rethrow
	// get block structure of A

	return p_DestructiveOrdering(m_p_block, p_constraints,
		n_constraints_size, b_need_inverse);
	// calculate ordering by block, can destroy the pattern matrix in the process
}

/**
 *	@brief CRC calculation template
 *
 *	@param TScalar is CRC scalar type (uint16_t for crc16, uint32_t for crc32)
 *	@param n_polynom is CRC polynom
 *	@param n_start is CRC starting value
 *	@param n_final_xor is final reflection value (usually all 0's or all 1's)
 *
 *	@note It is possible to use predefined specializations CCrc_16 and CCrc_32,
 *		implementing standard CRC16 and CRC32 algorithms, respectively.
 */
template<class TScalar, const TScalar n_polynom, const TScalar n_start, const TScalar n_final_xor>
class CCrc {
public:
	/**
	 *	@brief gets CRC starting value
	 *	@return Returns starting value of CRC.
	 */
	static inline TScalar n_Start()
	{
		return n_start;
	}

	/**
	 *	@brief calculates final xor
	 *
	 *	@param[in] n_prev_crc is either n_Start() (CRC of empty buffer),
	 *		or result of previous call to n_Crc() (when calculating CRC of one or more
	 *		concatenated buffers)
	 *
	 *	@return Returns final value of CRC.
	 */
	static inline TScalar n_Finalize(TScalar n_prev_crc)
	{
		return n_prev_crc ^ n_final_xor;
	}

	/**
	 *	@brief calculates CRC on a stream of data
	 *
	 *	Calculates CRC of n_size bytes from buffer p_data.
	 *
	 *	@param[in] n_size is size of input data, in bytes
	 *	@param[in] p_data is input data buffer, contains n_size bytes
	 *	@param[in] n_prev_crc is either n_Start() (starting new CRC calculation),
	 *		or result of previous call to n_Crc() (when calculating CRC of more
	 *		concatenated buffers)
	 *
	 *	@return Returns CRC value, no data reflection, no final xor.
	 */
	static TScalar n_Crc(size_t n_size, const void *p_data, TScalar n_prev_crc)
	{
		const uint8_t *_p_data = (const uint8_t*)p_data;

		static TScalar p_crc_table[256];
		static bool b_crc_table_ready = false;
		if(!b_crc_table_ready) {
			b_crc_table_ready = true;
			for(int i = 0; i < 256; ++ i) {
				TScalar n_root = TScalar(i);
				for(int j = 0; j < 8; ++ j) {
					if(n_root & 1)
						n_root = (n_root >> 1) ^ n_polynom;
					else
						n_root >>= 1;
				}
				p_crc_table[i] = n_root;
			}
		}
		// prepare the table

		TScalar n_crc = n_prev_crc;
		for(const uint8_t *p_end = _p_data + n_size; _p_data != p_end; ++ _p_data)
			n_crc = p_crc_table[(n_crc ^ *_p_data) & 0xff] ^ (n_crc >> 8);
		// calculate CRC

		return n_crc;
	}

	/**
	 *	@brief calculates CRC
	 *
	 *	Calculates CRC of n_size bytes from buffer p_data.
	 *
	 *	@param[in] n_size is size of input data, in bytes
	 *	@param[in] p_data is input data buffer, contains n_size bytes
	 *
	 *	@return Returns the final CRC value.
	 */
	static inline TScalar n_Crc(size_t n_size, const void *p_data)
	{
		return n_Finalize(n_Crc(n_size, p_data, n_Start()));
	}
};

typedef CCrc<uint16_t, 0xA001, 0xffff, 0xffff> CCrc_16; /**< CRC-16, as defined by ANSI (X3.28) */
typedef CCrc<uint32_t, 0xedb88320U, 0xffffffffU, 0xffffffffU> CCrc_32; /**< CRC-32, as defined by POSIX */

#ifdef __MATRIX_ORDERING_USE_MMD

extern "C" {

#include "cholmod/METIS/metis.h"
extern void libmetis__genmmd(idx_t neqns, idx_t *xadj, idx_t *adjncy, idx_t *invp, idx_t *perm,
     idx_t delta, idx_t *head, idx_t *qsize, idx_t *list, idx_t *marker,
     idx_t maxint, idx_t *ncsub);

};

#endif // __MATRIX_ORDERING_USE_MMD

const size_t *CMatrixOrdering::p_BlockOrdering(const CUberBlockMatrix &r_A,
	bool b_need_inverse /*= false*/, bool b_A_is_upper_triangular /*= true*/) // throw(std::bad_alloc)
{
	_ASSERTE(!r_A.b_Empty());
	_ASSERTE(r_A.b_Square_BlockSquare());

#ifdef __MATRIX_ORDERING_USE_MMD
	// todo - this currently seems to be giving worse ordering (more fill-in)
	// than AMD. read the paper and see what kind of graph does it expect ...

	if(!(m_p_block = r_A.p_BlockStructure_to_Sparse(m_p_block)))
		throw std::bad_alloc(); // rethrow
	// get block structure of A

	_ASSERTE(r_A.b_SymmetricLayout()); // well, it should be
	// it should also be upper-triangular

	if(!m_p_block->n) {
		m_ordering.clear();
		m_ordering_invert.clear();
		return 0;
	}
	// empty ordering

	if(sizeof(idx_t) != sizeof(size_t))
		throw std::runtime_error("size of METIS idx_t not configured correctly");
	// must be the same

	size_t nzaat = 2 * (m_p_block->p[m_p_block->n] - m_p_block->n); // we know this
	size_t naat = m_p_block->n;
	size_t aat_size = naat + 1 + nzaat;
	if(m_camd_workspace.capacity() < aat_size + naat) {
		m_camd_workspace.clear();
		m_camd_workspace.resize(std::max(aat_size + naat, m_camd_workspace.capacity() * 2));
	}
	size_t *Len = &m_camd_workspace[0];
	size_t *AATp = Len + naat;
	size_t *AATi = AATp + naat + 1;
	{
		const size_t n = m_p_block->n, nz = m_p_block->p[m_p_block->n];
		const ptrdiff_t *Ap = m_p_block->p, *Ai = m_p_block->i; // antialiass
		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			size_t p2 = Ap[i + 1];
			_ASSERTE(p1 < p2); // no empty columns!
			_ASSERTE(Ai[p2 - 1] == i); // diagonal is here (skip)
			_ASSERTE(p2 > p1); // makes sure the next line will not underflow
			Len[i] = p2 - p1 - 2; // could actually underflow by 1, the diag entry will fix it
			p1 = p2;
			_ASSERTE(p1 == Ap[i + 1]); // this will soo fail on jumbled matrices (_DEBUG checks for them)
		}
		for(size_t i = 0; i < nz; ++ i)
			++ Len[Ai[i]];
		// this could be vectorized, essentially len = Ap+1 + Ap + Ai - 2, at least the first two could do with integer sse
		// alternately, memset could be ommited by splitting to one loop over Ap (assigning) and one over Ai (incrementing)
	}
	{
		const size_t n = m_p_block->n;
		const ptrdiff_t *Ap = m_p_block->p, *Ai = m_p_block->i; // antialiass

		size_t pfree = 0; // this is needed in amd_2()
		{
			size_t *Pe = AATp, *Iw = AATi;

			*Pe = 0; // not really calculated, needs to be written here
			size_t *p_column_dest = Pe + 1; // calculate Pe inplace
			for(size_t j = 0; j < n; ++ j) {
				p_column_dest[j] = pfree; // pointer to the next entry to write in column j
				pfree += Len[j];
			} // t_odo could only accum p_column_dest, and employ that p_column_dest[i] == Pe[i + 1] at the end. just need to off by 1 and add the starting 0
			_ASSERTE(pfree == nzaat);
			// initialize column pointers

			for(size_t k = 0; k < n; ++ k) {
				size_t p1 = Ap[k];
				size_t p2 = Ap[k + 1];

				_ASSERTE(p1 < p2); // no empty columns!
				_ASSERTE(Ai[p2 - 1] == k); // diagonal is here (skip)
				-- p2; // skip the diagonal
				_ASSERTE(p2 >= p1); // makes sure the next line will not overflow

				for(size_t p = p1; p < p2; ++ p) {
					size_t j = Ai[p];
					_ASSERTE(j >= 0 && j < n);
					_ASSERTE(j < k); // only above-diagonal elements
					// scan the upper triangular part of A

					Iw[p_column_dest[j] ++] = k;
					Iw[p_column_dest[k] ++] = j;
					// entry A(j, k) in the strictly upper triangular part
				}
				// construct one column of A+A^T
			}
			// builds A+A^T from a symmetric matrix (todo - make this a part of CUberBlockMatrix, avoid forming A, build A+A^T directly)
		}
	}

	idx_t nvtxs = m_p_block->n,
		*xadj = (idx_t*)AATp,//m_p_block->p,
		*adjncy = (idx_t*)AATi;//m_p_block->i;

	for(idx_t i = 0, k = xadj[nvtxs]; i < k; ++ i)
		++ adjncy[i];
	for(idx_t i = 0, n = nvtxs + 1; i < n; ++ i)
		++ xadj[i];
	// genmmd() uses 1-based indexing

	const size_t n = nvtxs + 5;
#ifdef __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_al = n_Align_Up_POT(n, __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES / sizeof(size_t));
#else // __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_al = n;
#endif // __MATRIX_ORDERING_CACHE_ALIGN
	// to align, or not to align ...

	size_t slen = n_al;
	if(slen > SIZE_MAX / 4) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen *= 4;
	if(m_camd_workspace1.size() < slen) {
		m_camd_workspace1.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace1.resize(std::max(slen, m_camd_workspace1.size() * 2));
	}
	_ASSERTE(m_camd_workspace1.size() >= slen);
	idx_t *head = (idx_t*)&m_camd_workspace1[0];
	idx_t *qsize = head + n_al;
	idx_t *list = qsize + n_al;
	idx_t *marker = list + n_al;
	// alloc temp buffers

	if(m_ordering.capacity() < n_al) {
		m_ordering.clear(); // avoid copying data if it needs to realloc
		m_ordering.resize(std::max(n_al, m_ordering.size() * 2));
	}
	if(m_ordering_invert.capacity() < n_al) {
		m_ordering_invert.clear(); // avoid copying data if it needs to realloc
		m_ordering_invert.resize(std::max(n_al, m_ordering_invert.size() * 2));
	}
	m_ordering.resize(nvtxs);
	m_ordering_invert.resize(nvtxs);
	idx_t *perm = (idx_t*)&m_ordering[0];
	idx_t *iperm = (idx_t*)&m_ordering_invert[0];
	// alloc ordering

	//std::swap(perm, iperm); // bad idea, bad ordering
	// or the other way around?

	idx_t nofsub; // unused output
	libmetis__genmmd(nvtxs, xadj, adjncy, iperm, perm, __MATRIX_ORDERING_MMD_DELTA,
		head, qsize, list, marker, IDX_MAX, &nofsub);
	// calculate the ordering

	for(idx_t i = 0; i < nvtxs; ++ i) {
		-- perm[i];
		-- iperm[i];
	}
	// genmmd() uses 1-based indexing

	/*for(idx_t i = 0; i < nvtxs; ++ i) { // bad idea, bad ordering
		perm[i] = nvtxs - 1 - perm[i];
		iperm[perm[i]] = i; // update inverse as well (is there an easier way?)
	}*/
	// reverse the vertex labels permutatuion

	_ASSERTE(b_IsValidOrdering(&m_ordering[0], m_ordering.size()));

	return (size_t*)perm;
	// calculate ordering by block, m_p_block damaged in the process
#else // __MATRIX_ORDERING_USE_MMD
	/*CTimer timer;
	double f_start = timer.f_Time();*/

#if !__MATRIX_ORDERING_USE_UBM_AAT
	if(!(m_p_block = r_A.p_BlockStructure_to_Sparse(m_p_block)))
		throw std::bad_alloc(); // rethrow
	// get block structure of A

	_ASSERTE(r_A.b_SymmetricLayout()); // well, it should be
	// it should also be upper-triangular

	//const size_t *p_result = p_Ordering(m_p_block); // AMD, not CAMD
	// can do with any kind of matrix

	const size_t *p_result = p_DestructiveOrdering(m_p_block, b_need_inverse); // AMD, not CAMD
	// only upper-diagonal, only well sorted (guaranteed by p_BlockStructure_to_Sparse())

	/*double f_time = timer.f_Time() - f_start;

#if 1
	uint32_t n_ordering_crc32 = CCrc_32::n_Finalize(CCrc_32::n_Crc(r_A.n_BlockColumn_Num() *
		sizeof(size_t), p_result));
	printf("calculated ordering of size " PRIsize ", crc32: 0x%08x\n",
		r_A.n_BlockColumn_Num(), n_ordering_crc32);
	printf("it took %f sec\n", f_time);
#endif // 1*/
	// debug the orderings generated

	return p_result;
	// calculate ordering by block, can destroy the pattern matrix in the process
#else // !__MATRIX_ORDERING_USE_UBM_AAT
	const size_t m = r_A.n_BlockRow_Num(), n = r_A.n_BlockColumn_Num();
	_ASSERTE(m == n); // only square matrices (could support other matrices though)

	_ASSERTE(n >= 0 && n <= SIZE_MAX);
	if(m_ordering.capacity() < n) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(n, 2 * m_ordering.capacity()));
	}
	m_ordering.resize(n);
	// maintain an ordering array

	if(b_need_inverse) {
		if(m_ordering_invert.capacity() < n) {
			m_ordering_invert.clear(); // avoid copying data in resizing
			m_ordering_invert.reserve(std::max(n, 2 * m_ordering_invert.capacity()));
		}
		m_ordering_invert.resize(n);
	}
	// allocate an extra inverse ordering array, if needed

	//const size_t n = p_A->n, nz = p_A->p[p_A->n];
	if(n >= SIZE_MAX / sizeof(size_t) /*||
	   nz >= SIZE_MAX / sizeof(size_t)*/)
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY

#ifdef __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_alignment = __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES / sizeof(size_t);
	const size_t n_al = n_Align_Up_POT(n, n_alignment);
#else // __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_alignment = 1;
	const size_t n_al = n;
#endif // __MATRIX_ORDERING_CACHE_ALIGN
	// to align, or not to align ...

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
	_ASSERTE(sizeof(size_t) > 1); // note 2 * n does not overflow if sizeof(size_t) > 1
	if(m_camd_workspace.capacity() < ((b_need_inverse)? n_al : n + n_al)) {
		m_camd_workspace.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace.reserve(std::max((b_need_inverse)? n_al : n + n_al, m_camd_workspace.capacity() * 2));
	}
	m_camd_workspace.resize((b_need_inverse)? n_al : n + n_al);
	size_t *Len = &m_camd_workspace[0];
	size_t *Pinv = (b_need_inverse)? &m_ordering_invert[0] : &m_camd_workspace[n_al]; // this is actually inverse ordering, we need it most of the time, it is a waste to throw it away
	// alloc workspace

	enum {
		b_upper = true, // lambda in SLAM++ is always upper triangular
		b_sorted = false // lambda is supposed to be symmetric, it will be sorted regardless
	};
	size_t nzaat;
	if(b_A_is_upper_triangular) {
		nzaat = r_A.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<b_upper,
			!b_upper, false, false>(Len, n, Pinv, n); // use Pinv as workspace
	} else {
		nzaat = r_A.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<false,
			true, false, false>(Len, n, Pinv, n); // use Pinv as workspace
	}

	size_t slen = nzaat;
	if(slen > SIZE_MAX - (n_alignment - 1)) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += n_alignment - 1;
	if(slen > SIZE_MAX - nzaat / 5) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += nzaat / 5;
	if(n_al > SIZE_MAX / 8 || slen > SIZE_MAX - 8 * n_al) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += 8 * n_al;
	if(m_camd_workspace1.size() < slen) {
		m_camd_workspace1.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace1.resize(std::max(slen, m_camd_workspace1.size() * 2));
	}
	_ASSERTE(m_camd_workspace1.size() >= slen);
	slen = m_camd_workspace1.size();
	// allocate workspace for matrix, elbow room (size n), and 7 (n + alignment) vectors

	size_t iwlen = slen - 6 * n_al;
	size_t *Pe, *Nv, *Head, *Elen, *Degree, *W, *Iw;
	{
		size_t *S = &m_camd_workspace1[0];

#ifdef __MATRIX_ORDERING_CACHE_ALIGN
		size_t n_align = n_Align_Up_POT(ptrdiff_t(S), ptrdiff_t(n_alignment)) - ptrdiff_t(S);
		S += n_align;
		iwlen -= n_align; // !!
		// make sure the first pointer is aligned (allocated n_alignment - 1 more space)
#endif // __MATRIX_ORDERING_CACHE_ALIGN

		Pe = S;		S += n_al;
		Nv = S;		S += n_al;
		Head = S;	S += n_al;
		Elen = S;	S += n_al;
		Degree = S;	S += n_al;
		W = S;
		Iw = S + n_al;
	}
	_ASSERTE(iwlen == &m_camd_workspace1.back() + 1 - Iw); // make sure we got this right (the + 1 is so that we get the one-past-the-last pointer)
	// one array actually stores many smaller ones // t_odo play with cache here? could align n to 64B offset so that each array fills different cache entry (if 6bit cache off)

	const size_t pfree = nzaat; // this is needed in amd_l2(), equals nzaat
	_ASSERTE(iwlen >= pfree + n); // as dictated by amd_l2()

	if(b_A_is_upper_triangular) {
		r_A.BlockStructure_SumWithSelfTranspose<b_upper, !b_upper, false, false, b_sorted>(Pe,
			n + 1, Iw, nzaat, Len, n, Head, n_al); // can't use Nv as workspace because Nv[0] will contain value of nzaat if there is no alignment; could use Pinv or Head then (Head is allocated to cache line, maybe better)
	} else {
		r_A.BlockStructure_SumWithSelfTranspose<false, true, false, false, b_sorted>(Pe,
			n + 1, Iw, nzaat, Len, n, Head, n_al);
	}
	// calculate structure of A+AT in Pe, Iw

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	amd_l2(n, (SuiteSparse_long*)Pe, (SuiteSparse_long*)Iw,
		(SuiteSparse_long*)Len, iwlen, pfree, (SuiteSparse_long*)Nv,
		(SuiteSparse_long*)Pinv, (SuiteSparse_long*)&m_ordering[0],
		(SuiteSparse_long*)Head, (SuiteSparse_long*)Elen,
		(SuiteSparse_long*)Degree, (SuiteSparse_long*)W, 0, 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	amd_2(n, (int*)Pe, (int*)Iw, (int*)Len, iwlen, pfree, (int*)Nv, (int*)Pinv,
		(int*)&m_ordering[0], (int*)Head, (int*)Elen, (int*)Degree, (int*)W, 0, 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

	if(b_need_inverse) {
		size_t *p_ordering = &m_ordering[0];
		for(size_t i = 0; i < n; ++ i)
			Pinv[p_ordering[i]] = i;
	}
	// AMD does not output the inverse ordering on output, at least not in this version // todo - or does it?

	return &m_ordering[0];
	// return ordering
#endif // !__MATRIX_ORDERING_USE_UBM_AAT
#endif // __MATRIX_ORDERING_USE_MMD
}

#if 0
const size_t *CMatrixOrdering::p_HybridBlockOrdering(const CUberBlockMatrix &r_A, size_t n_off_diagonal_num,
	const size_t *p_constraints, size_t UNUSED(n_constraints_size)) // throw(std::bad_alloc)
{
	_ASSERTE(r_A.b_Square_BlockSquare());
	_ASSERTE(!p_constraints || n_constraints_size == r_A.n_BlockColumn_Num());

	if(!(m_p_block = r_A.p_BlockStructure_to_Sparse(m_p_block)))
		throw std::bad_alloc(); // rethrow
	// get block structure of A

	return p_HybridDestructiveOrdering(m_p_block, n_off_diagonal_num, p_constraints, n_constraints_size);
	// calculate ordering by block, can destroy the pattern matrix in the process
}
#endif // 0

const size_t *CMatrixOrdering::p_ExpandBlockOrdering(const CUberBlockMatrix &r_A, bool b_inverse) // throw(std::bad_alloc)
{
	_ASSERTE(r_A.b_Square_BlockSquare());
	const size_t n_column_num = r_A.n_Column_Num();
	const size_t n_column_block_num = r_A.n_BlockColumn_Num();

	_ASSERTE(m_ordering.size() == n_column_block_num);
	// make sure the ordering can be possibly applied to the matrix

	_ASSERTE(!b_inverse);
	// can't expand inverse ordering, the r_A.n_BlockColumn_Base(n_order)
	// corresponds to a different block in the inverse ordering, than it did in the original ordering
	// r_A would have to be the permutation of r_A, but that makes this function dangerous
	// just assume we don't need the inverse ordering at the moment

	if(m_ordering_expand.size() < n_column_num) {
		m_ordering_expand.clear();
		m_ordering_expand.resize(std::max(n_column_num, 2 * m_ordering_expand.capacity()));
	}
	size_t n_scalar_offset = 0;
	const size_t *p_order_ptr = (b_inverse)? &m_ordering_invert[0] : &m_ordering[0];
	for(size_t i = 0; i < n_column_block_num; ++ i, ++ p_order_ptr) {
		const size_t n_order = *p_order_ptr;
		size_t n_block_base = r_A.n_BlockColumn_Base(n_order); // wonder what would happen if it was m_ordering[n_order] or something like that ... would that get a correct column in the inverse ordering?
		size_t n_block_width = r_A.n_BlockColumn_Column_Num(n_order);
		for(size_t j = 0; j < n_block_width; ++ j, ++ n_scalar_offset, ++ n_block_base)
			m_ordering_expand[n_scalar_offset] = n_block_base;
	}
	_ASSERTE(n_scalar_offset == n_column_num);
	// blow up the permutation from block level to scalar level

	return &m_ordering_expand[0];
}

const size_t *CMatrixOrdering::p_Ordering(const cs *p_A, const size_t *p_constraints,
	size_t UNUSED(n_constraints_size)) // throw(std::bad_alloc)
{
	_ASSERTE(!p_constraints || n_constraints_size == p_A->n);

	if(m_ordering.capacity() < size_t(p_A->n)) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(size_t(p_A->n), 2 * m_ordering.capacity()));
	}
	m_ordering.resize(p_A->n);
	// maintain an ordering array

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	switch(camd_l_order(p_A->n, p_A->p, p_A->i, (SuiteSparse_long*)&m_ordering[0],
	   0, 0, (SuiteSparse_long*)p_constraints)) {
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	switch(camd_order(p_A->n, p_A->p, p_A->i, (int*)&m_ordering[0], 0, 0, (int*)p_constraints)) {
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	case CAMD_OK:
	case CAMD_OK_BUT_JUMBLED:
		break;
	case CAMD_OUT_OF_MEMORY:
		throw std::bad_alloc();
	case CAMD_INVALID:
		return 0;
	}
	// call camd

	return &m_ordering[0];
	// return ordering
}

const size_t *CMatrixOrdering::p_Ordering(const cs *p_A) // throw(std::bad_alloc)
{
	if(m_ordering.capacity() < size_t(p_A->n)) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(size_t(p_A->n), 2 * m_ordering.capacity()));
	}
	m_ordering.resize(p_A->n);
	// maintain an ordering array

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	switch(amd_l_order(p_A->n, p_A->p, p_A->i, (SuiteSparse_long*)&m_ordering[0], 0, 0)) {
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	switch(amd_order(p_A->n, p_A->p, p_A->i, (int*)&m_ordering[0], 0, 0)) {
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	case AMD_OK:
	case AMD_OK_BUT_JUMBLED:
		break;
	case AMD_OUT_OF_MEMORY:
		throw std::bad_alloc();
	case AMD_INVALID:
		return 0;
	}
	// call amd

	return &m_ordering[0];
	// return ordering
}

const size_t *CMatrixOrdering::p_DestructiveOrdering(cs *p_A, bool b_need_inverse) // throw(std::bad_alloc)
{
	_ASSERTE(p_A->n >= 0 && p_A->n <= SIZE_MAX);
	if(m_ordering.capacity() < size_t(p_A->n)) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(size_t(p_A->n), 2 * m_ordering.capacity()));
	}
	m_ordering.resize(p_A->n);
	// maintain an ordering array

	if(b_need_inverse) {
		if(m_ordering_invert.capacity() < size_t(p_A->n)) {
			m_ordering_invert.clear(); // avoid copying data in resizing
			m_ordering_invert.reserve(std::max(size_t(p_A->n), 2 * m_ordering_invert.capacity()));
		}
		m_ordering_invert.resize(p_A->n);
	}
	// allocate an extra inverse ordering array, if needed

	const size_t n = p_A->n, nz = p_A->p[p_A->n];
	if(n >= SIZE_MAX / sizeof(size_t) ||
	   nz >= SIZE_MAX / sizeof(size_t))
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY

#ifdef __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_al = n_Align_Up_POT(n, __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES / sizeof(size_t));
#else // __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_al = n;
#endif // __MATRIX_ORDERING_CACHE_ALIGN
	// to align, or not to align ...

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	_ASSERTE(amd_l_valid(n, n, p_A->p, p_A->i) == AMD_OK); // note that this fails on "jumbled" matrices (different behavior than p_Ordering())
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	_ASSERTE(amd_valid(n, n, p_A->p, p_A->i) == AMD_OK); // note that this fails on "jumbled" matrices (different behavior than p_Ordering())
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	// check if it's valid (debug only)

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
	_ASSERTE(sizeof(size_t) > 1); // note 2 * n does not overflow if sizeof(size_t) > 1
	if(m_camd_workspace.capacity() < ((b_need_inverse)? n : n + n_al)) {
		m_camd_workspace.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace.reserve(std::max((b_need_inverse)? n : n + n_al, m_camd_workspace.capacity() * 2));
	}
	m_camd_workspace.resize((b_need_inverse)? n : n + n_al);
	size_t *Len = &m_camd_workspace[0];
	size_t *Pinv = (b_need_inverse)? &m_ordering_invert[0] : &m_camd_workspace[n_al]; // this is actually inverse ordering, we need it most of the time, it is a waste to throw it away
	// alloc workspace

#ifdef __MATRIX_ORDERING_USE_AMD_AAT
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	size_t nzaat = __amd_internal::amd_l_aat(n, p_A->p, p_A->i,
		(SuiteSparse_long*)Len, (SuiteSparse_long*)&m_ordering[0], 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	size_t nzaat = __amd_internal::amd_aat(n, p_A->p, p_A->i,
		(int*)Len, (int*)&m_ordering[0], 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	_ASSERTE(nzaat == 2 * (nz - n)); // A is upper-triangular, nz of A+A^T (excluding diagonal) should equal 2 * (nz - n)
#else // __MATRIX_ORDERING_USE_AMD_AAT
	{
		const ptrdiff_t *Ap = p_A->p, *Ai = p_A->i; // antialiass
		/*memset(Len, 0, p_A->n * sizeof(size_t)); // memset Len to 0
		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			size_t p2 = Ap[i + 1];
			_ASSERTE(p1 < p2); // no empty columns!
			_ASSERTE(Ai[p2 - 1] == i); // diagonal is here (skip)
			-- p2; // skip the diagonal
			_ASSERTE(p2 >= p1); // makes sure the next line will not overflow
			Len[i] += p2 - p1;
			for(; p1 < p2; ++ p1) {
				size_t j = Ai[p1];
				_ASSERTE(j < i); // upper-triangular and we skipped the diagonal before the loop
				++ Len[j]; // off-diagonal
			}
			++ p1;
			_ASSERTE(p1 == Ap[i + 1]); // this will soo fail on jumbled matrices (_DEBUG checks for them)
		}*/
		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			size_t p2 = Ap[i + 1];
			_ASSERTE(p1 < p2); // no empty columns!
			_ASSERTE(Ai[p2 - 1] == i); // diagonal is here (skip)
			_ASSERTE(p2 > p1); // makes sure the next line will not underflow
			Len[i] = p2 - p1 - 2; // could actually underflow by 1, the diag entry will fix it
			p1 = p2;
			_ASSERTE(p1 == Ap[i + 1]); // this will soo fail on jumbled matrices (_DEBUG checks for them)
		}
		for(size_t i = 0; i < nz; ++ i)
			++ Len[Ai[i]];
		// this could be vectorized, essentially len = Ap+1 + Ap + Ai - 2, at least the first two could do with integer sse
		// alternately, memset could be ommited by splitting to one loop over Ap (assigning) and one over Ai (incrementing)
	}
	size_t nzaat = 2 * (nz - n); // we know this
#endif // __MATRIX_ORDERING_USE_AMD_AAT
	_ASSERTE((std::max(nz, n) - n <= nzaat) && (nzaat <= 2 * (size_t)nz));
	//printf("nzaat = " PRIsize "\n", nzaat); // 805348 for 100k, my custom code, amd_aat() gives the same
	// determine the symmetry and count off-diagonal nonzeros in A+A'

	size_t slen = nzaat;
	if(slen > SIZE_MAX - nzaat / 5) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += nzaat / 5;
	if(n_al > SIZE_MAX / 8 || slen > SIZE_MAX - 8 * n_al) // check overflow
		throw std::bad_alloc(); // would get AMD_OUT_OF_MEMORY
	slen += 8 * n_al;
	if(m_camd_workspace1.size() < slen) {
		m_camd_workspace1.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace1.resize(std::max(slen, m_camd_workspace1.size() * 2));
	}
	_ASSERTE(m_camd_workspace1.size() >= slen);
	slen = m_camd_workspace1.size();
	// allocate workspace for matrix, elbow room (size n), and 7 (n + alignment) vectors

	// todo finish with aligning, compare the actual orderings

#ifdef __MATRIX_ORDERING_USE_AMD1
	size_t *S = &m_camd_workspace1[0];
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	__amd_internal::amd_l1(n, p_A->p, p_A->i, (SuiteSparse_long*)&m_ordering[0],
		(SuiteSparse_long*)Pinv, (SuiteSparse_long*)Len, slen,
		(SuiteSparse_long*)S, 0, 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	__amd_internal::amd_1(n, p_A->p, p_A->i, (int*)&m_ordering[0], (int*)Pinv,
		(int*)Len, slen, (int*)S, 0, 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	// order the matrix
#else // __MATRIX_ORDERING_USE_AMD1
	{
		const ptrdiff_t *Ap = p_A->p, *Ai = p_A->i; // antialiass

		size_t iwlen = slen - 6 * n_al;
		size_t *Pe, *Nv, *Head, *Elen, *Degree, *W, *Iw;
		{
			size_t *S = &m_camd_workspace1[0];
			Pe = S;		S += n_al;
			Nv = S;		S += n_al;
			Head = S;	S += n_al;
			Elen = S;	S += n_al;
			Degree = S;	S += n_al;
			W = S;
			Iw = S + n_al;
		}
		// one array actually stores many smaller ones // t_odo play with cache here? could align n to 64B offset so that each array fills different cache entry (if 6bit cache off)

		size_t pfree = 0; // this is needed in amd_2()
		{
			*Pe = 0; // not really calculated, needs to be written here
			size_t *p_column_dest = Pe + 1; // calculate Pe inplace
#ifdef _DEBUG
			size_t *p_column_end = Head; // use Head as workspace (do not use Nv, the first item gets overwritten)
#endif // _DEBUG
			for(size_t j = 0; j < n; ++ j) {
				p_column_dest[j] = pfree; // pointer to the next entry to write in column j
				pfree += Len[j];
#ifdef _DEBUG
				p_column_end[j] = pfree; // (j+1)-th column pointer of A+A^T
#endif // _DEBUG
			} // t_odo could only accum p_column_dest, and employ that p_column_dest[i] == Pe[i + 1] at the end. just need to off by 1 and add the starting 0
			_ASSERTE(iwlen >= pfree + n);
			// initialize column pointers

			for(size_t k = 0; k < n; ++ k) {
				size_t p1 = Ap[k];
				size_t p2 = Ap[k + 1];

				_ASSERTE(p1 < p2); // no empty columns!
				_ASSERTE(Ai[p2 - 1] == k); // diagonal is here (skip)
				-- p2; // skip the diagonal
				_ASSERTE(p2 >= p1); // makes sure the next line will not overflow

				for(size_t p = p1; p < p2; ++ p) {
					size_t j = Ai[p];
					_ASSERTE(j >= 0 && j < n);
					_ASSERTE(j < k); // only above-diagonal elements
					// scan the upper triangular part of A

					_ASSERTE(p_column_dest[j] < p_column_end[j]);
					_ASSERTE(p_column_dest[k] < p_column_end[k]);
					Iw[p_column_dest[j] ++] = k;
					Iw[p_column_dest[k] ++] = j;
					// entry A(j, k) in the strictly upper triangular part
				}
				// construct one column of A+A^T
			}
			// builds A+A^T from a symmetric matrix (todo - make this a part of CUberBlockMatrix, avoid forming A, build A+A^T directly)

#ifdef _DEBUG
			for(size_t j = 0, n_cumsum = 0; j < n; ++ j) {
				_ASSERTE(Pe[j] == n_cumsum); // points at the beginning of the column
				n_cumsum += Len[j];
				_ASSERTE(p_column_dest[j] == n_cumsum); // points at the end of the column
			}
			// make sure all the columns are filled, according to Len
#endif // _DEBUG
		}

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		amd_l2(n, (SuiteSparse_long*)Pe, (SuiteSparse_long*)Iw,
			(SuiteSparse_long*)Len, iwlen, pfree, (SuiteSparse_long*)Nv,
			(SuiteSparse_long*)Pinv, (SuiteSparse_long*)&m_ordering[0],
			(SuiteSparse_long*)Head, (SuiteSparse_long*)Elen,
			(SuiteSparse_long*)Degree, (SuiteSparse_long*)W, 0, 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		amd_2(n, (int*)Pe, (int*)Iw, (int*)Len, iwlen, pfree, (int*)Nv, (int*)Pinv,
			(int*)&m_ordering[0], (int*)Head, (int*)Elen, (int*)Degree, (int*)W, 0, 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	}
	// original code taken from AMD by Tim Davis
#endif // __MATRIX_ORDERING_USE_AMD1

	if(b_need_inverse) {
		size_t *p_ordering = &m_ordering[0];
		for(size_t i = 0; i < n; ++ i)
			Pinv[p_ordering[i]] = i;
	}
	// AMD does not output the inverse ordering on output, at least not in this version // todo - or does it?

	// note that amd_1 constructs A^T | A (A is a binary matrix) first,
	// it could be optimized / calculated inplace / implemented directly in block matrix // t_odo

	return &m_ordering[0];
	// return ordering
}

const size_t *CMatrixOrdering::p_DestructiveOrdering(cs *p_A, const size_t *p_constraints,
	size_t UNUSED(n_constraints_size), bool b_need_inverse) // throw(std::bad_alloc)
{
	_ASSERTE(!p_constraints || n_constraints_size == p_A->n);

	_ASSERTE(p_A->n >= 0 && p_A->n <= SIZE_MAX);
	if(m_ordering.capacity() < size_t(p_A->n)) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(size_t(p_A->n), 2 * m_ordering.capacity()));
	}
	m_ordering.resize(p_A->n);
	// maintain an ordering array

	if(b_need_inverse) {
		if(m_ordering_invert.capacity() < size_t(p_A->n)) {
			m_ordering_invert.clear(); // avoid copying data in resizing
			m_ordering_invert.reserve(std::max(size_t(p_A->n), 2 * m_ordering_invert.capacity()));
		}
		m_ordering_invert.resize(p_A->n);
	}
	// allocate an extra inverse ordering array, if needed

	const size_t n = p_A->n, nz = p_A->p[p_A->n];
	if(n >= SIZE_MAX / sizeof(size_t) ||
	   nz >= SIZE_MAX / sizeof(size_t))
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY

#ifdef __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_al = n_Align_Up_POT(n, __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES / sizeof(size_t));
#else // __MATRIX_ORDERING_CACHE_ALIGN
	const size_t n_al = n;
#endif // __MATRIX_ORDERING_CACHE_ALIGN
	// to align, or not to align ...

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	_ASSERTE(camd_l_valid(n, n, p_A->p, p_A->i) == CAMD_OK); // note that this fails on "jumbled" matrices (different behavior than p_Ordering())
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	_ASSERTE(camd_valid(n, n, p_A->p, p_A->i) == CAMD_OK); // note that this fails on "jumbled" matrices (different behavior than p_Ordering())
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	// check if it's valid (debug only)

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
	_ASSERTE(sizeof(size_t) > 1); // note 2 * n does not overflow if sizeof(size_t) > 1
	if(m_camd_workspace.capacity() < ((b_need_inverse)? n : n + n_al)) {
		m_camd_workspace.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace.reserve(std::max((b_need_inverse)? n : n + n_al, m_camd_workspace.capacity() * 2));
	}
	m_camd_workspace.resize((b_need_inverse)? n : n + n_al);
	size_t *Len = &m_camd_workspace[0];
	size_t *Pinv = (b_need_inverse)? &m_ordering_invert[0] : &m_camd_workspace[n_al]; // this is actually inverse ordering, we need it most of the time, it is a waste to throw it away
	// alloc workspace

#ifdef __MATRIX_ORDERING_USE_AMD_AAT
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	size_t nzaat = __camd_internal::camd_l_aat(n, p_A->p, p_A->i,
		(SuiteSparse_long*)Len, (SuiteSparse_long*)&m_ordering[0], 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	size_t nzaat = __camd_internal::camd_aat(n, p_A->p, p_A->i,
		(int*)Len, (int*)&m_ordering[0], 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	_ASSERTE(nzaat == 2 * (nz - n)); // A is upper-triangular, nz of A+A^T (excluding diagonal) should equal 2 * (nz - n)
#else // __MATRIX_ORDERING_USE_AMD_AAT
	{
		const ptrdiff_t *Ap = p_A->p, *Ai = p_A->i; // antialiass
		/*memset(Len, 0, p_A->n * sizeof(size_t)); // memset Len to 0
		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			size_t p2 = Ap[i + 1];
			_ASSERTE(p1 < p2); // no empty columns!
			_ASSERTE(Ai[p2 - 1] == i); // diagonal is here (skip)
			-- p2; // skip the diagonal
			_ASSERTE(p2 >= p1); // makes sure the next line will not overflow
			Len[i] += p2 - p1;
			for(; p1 < p2; ++ p1) {
				size_t j = Ai[p1];
				_ASSERTE(j < i); // upper-triangular and we skipped the diagonal before the loop
				++ Len[j]; // off-diagonal
			}
			++ p1;
			_ASSERTE(p1 == Ap[i + 1]); // this will soo fail on jumbled matrices (_DEBUG checks for them)
		}*/
		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			size_t p2 = Ap[i + 1];
			_ASSERTE(p1 < p2); // no empty columns!
			_ASSERTE(Ai[p2 - 1] == i); // diagonal is here (skip)
			_ASSERTE(p2 > p1); // makes sure the next line will not underflow
			Len[i] = p2 - p1 - 2; // could actually underflow by 1, the diag entry will fix it
			p1 = p2;
			_ASSERTE(p1 == Ap[i + 1]); // this will soo fail on jumbled matrices (_DEBUG checks for them)
		}
		for(size_t i = 0; i < nz; ++ i)
			++ Len[Ai[i]];
	}
	size_t nzaat = 2 * (nz - n); // we know this
#endif // __MATRIX_ORDERING_USE_AMD_AAT
	_ASSERTE((std::max(nz, n) - n <= nzaat) && (nzaat <= 2 * (size_t)nz));
	// determine the symmetry and count off-diagonal nonzeros in A+A'

	size_t slen = nzaat;
	if(slen > SIZE_MAX - nzaat / 5) // check overflow
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY
	slen += nzaat / 5;
	size_t n1 = n_al + 1; // for sure does not overflow
	if(n1 > SIZE_MAX / 9 || (n1 == SIZE_MAX && SIZE_MAX % 9) || slen > SIZE_MAX - 9 * n1) // check overflow
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY
	slen += 9 * n1; // note it is 1 bigger than it has to be // t_odo - check if we can save memory (probably not, CAMD requires 1 more than AMD, that will be it) // not a good idea, more memory runs faster
	if(m_camd_workspace1.size() < slen) {
		m_camd_workspace1.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace1.resize(std::max(slen, m_camd_workspace1.size() * 2));
	}
	_ASSERTE(m_camd_workspace1.size() >= slen);
	slen = m_camd_workspace1.size(); // give it all we've got
	// allocate workspace for matrix, elbow room (size n), and 7 (n + alignment) vectors

#ifdef __MATRIX_ORDERING_USE_AMD1
	size_t *S = &m_camd_workspace1[0];
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	__camd_internal::camd_l1(n, p_A->p, p_A->i, (SuiteSparse_long*)&m_ordering[0],
		(SuiteSparse_long*)Pinv, (SuiteSparse_long*)Len, slen,
		(SuiteSparse_long*)S, 0, 0, (SuiteSparse_long*)p_constraints);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	__camd_internal::camd_1(n, p_A->p, p_A->i, (int*)&m_ordering[0], (int*)Pinv,
		(int*)Len, slen, (int*)S, 0, 0, (int*)p_constraints);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	// order the matrix
#else // __MATRIX_ORDERING_USE_AMD1
	{
		const ptrdiff_t *Ap = p_A->p, *Ai = p_A->i; // antialiass

		size_t iwlen = slen - 7 * n_al - 2;
		size_t *Pe, *Nv, *Head, *Elen, *Degree, *W, *Iw, *BucketSet;
		{
			size_t *S = &m_camd_workspace1[0];
			Pe = S;		S += n_al;
			Nv = S;		S += n_al;
			Head = S;	S += n_al + 1; // note: 1 more than AMD
			Elen = S;	S += n_al;
			Degree = S;	S += n_al;
			W = S;		S += n_al + 1; // note: 1 more than AMD
			BucketSet = S;
			Iw = S + n_al;
		}
		// one array actually stores many smaller ones

		size_t pfree = 0; // this is needed in amd_2()
		{
			*Pe = 0; // not really calculated, needs to be written here
			size_t *p_column_dest = Pe + 1; // calculate Pe inplace
#ifdef _DEBUG
			size_t *p_column_end = Head; // use Head as workspace (do not use Nv, the first item gets overwritten)
#endif // _DEBUG
			for(size_t j = 0; j < n; ++ j) {
				p_column_dest[j] = pfree; // pointer to the next entry to write in column j
				pfree += Len[j];
#ifdef _DEBUG
				p_column_end[j] = pfree; // (j+1)-th column pointer of A+A^T
#endif // _DEBUG
			} // t_odo could only accum p_column_dest, and employ that p_column_dest[i] == Pe[i + 1] at the end. just need to off by 1 and add the starting 0
			_ASSERTE(iwlen >= pfree + n);
			// initialize column pointers

			for(size_t k = 0; k < n; ++ k) {
				size_t p1 = Ap[k];
				size_t p2 = Ap[k + 1];

				_ASSERTE(p1 < p2); // no empty columns!
				_ASSERTE(Ai[p2 - 1] == k); // diagonal is here (skip)
				-- p2; // skip the diagonal
				_ASSERTE(p2 >= p1); // makes sure the next line will not overflow

				for(size_t p = p1; p < p2; ++ p) {
					size_t j = Ai[p];
					_ASSERTE(j >= 0 && j < n);
					_ASSERTE(j < k); // only above-diagonal elements
					// scan the upper triangular part of A

					_ASSERTE(p_column_dest[j] < p_column_end[j]);
					_ASSERTE(p_column_dest[k] < p_column_end[k]);
					Iw[p_column_dest[j] ++] = k;
					Iw[p_column_dest[k] ++] = j;
					// entry A(j, k) in the strictly upper triangular part
				}
				// construct one column of A+A^T
			}
			// builds A+A^T from a symmetric matrix (todo - make this a part of CUberBlockMatrix, avoid forming A, build A+A^T directly)

#ifdef _DEBUG
			for(size_t j = 0, n_cumsum = 0; j < n; ++ j) {
				_ASSERTE(Pe[j] == n_cumsum); // points at the beginning of the column
				n_cumsum += Len[j];
				_ASSERTE(p_column_dest[j] == n_cumsum); // points at the end of the column
			}
			// make sure all the columns are filled, according to Len
#endif // _DEBUG
		}

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		camd_l2(n, (SuiteSparse_long*)Pe, (SuiteSparse_long*)Iw,
			(SuiteSparse_long*)Len, iwlen, pfree, (SuiteSparse_long*)Nv,
			(SuiteSparse_long*)Pinv, (SuiteSparse_long*)&m_ordering[0],
			(SuiteSparse_long*)Head, (SuiteSparse_long*)Elen,
			(SuiteSparse_long*)Degree, (SuiteSparse_long*)W, 0, 0,
			(SuiteSparse_long*)p_constraints, (SuiteSparse_long*)BucketSet);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		camd_2(n, (int*)Pe, (int*)Iw, (int*)Len, iwlen, pfree, (int*)Nv, (int*)Pinv,
			(int*)&m_ordering[0], (int*)Head, (int*)Elen, (int*)Degree, (int*)W, 0, 0,
			(int*)p_constraints, (int*)BucketSet);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	}
	// original code taken from AMD by Tim Davis
#endif // __MATRIX_ORDERING_USE_AMD1

	if(b_need_inverse) {
		size_t *p_ordering = &m_ordering[0];
		for(size_t i = 0; i < n; ++ i)
			Pinv[p_ordering[i]] = i;
	}
	// CAMD does not output the inverse ordering on output, at least not in this version

	// note that camd_1 constructs A^T | A (A is a binary matrix) first,
	// it could be optimized / calculated inplace / implemented directly in block matrix // t_odo

	return &m_ordering[0];
	// return ordering
}

#if 0
const size_t *CMatrixOrdering::p_HybridDestructiveOrdering(cs *p_A, size_t n_off_diagonal_num,
	const size_t *p_constraints, size_t UNUSED(n_constraints_size)) // throw(std::bad_alloc)
{
	_ASSERTE(!p_constraints || n_constraints_size == p_A->n);

	_ASSERTE(p_A->n >= 0 && p_A->n <= SIZE_MAX);
	if(m_ordering.capacity() < size_t(p_A->n))
		m_ordering.clear(); // avoid copying data in resizing
	m_ordering.resize(p_A->n);
	// maintain an ordering array

	const size_t n = p_A->n, nz = p_A->p[p_A->n];
	if(n >= SIZE_MAX / sizeof(size_t) ||
	   nz >= SIZE_MAX / sizeof(size_t))
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	_ASSERTE(camd_l_valid(n, n, p_A->p, p_A->i) == CAMD_OK); // note that this fails on "jumbled" matrices (different behavior than p_Ordering())
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	_ASSERTE(camd_valid(n, n, p_A->p, p_A->i) == CAMD_OK); // note that this fails on "jumbled" matrices (different behavior than p_Ordering())
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	// check if it's valid (debug only)

	_ASSERTE(sizeof(SuiteSparse_long) == sizeof(size_t));
	_ASSERTE(sizeof(size_t) > 1); // note 2 * n does not overflow if sizeof(size_t) > 1
	if(m_camd_workspace.capacity() < 2 * n) {
		m_camd_workspace.clear(); // avoid copying data if it needs to realloc
		m_camd_workspace.reserve(std::max(2 * n, m_camd_workspace.capacity() * 2));
	}
	m_camd_workspace.resize(2 * n);
	size_t *Len = &m_camd_workspace[0];
	size_t *Pinv = &m_camd_workspace[n];
	// alloc workspace

#if 1
	size_t nzaat = /*p_A->n +*/ 2 * n_off_diagonal_num; // calculate A+A^T nnz elements (A is upper triangular) // excluding diagonal!
	memset(Len, 0, p_A->n * sizeof(size_t)); // memset Len to 0
/*#if 0 //def _OPENMP
	if(p_A->p[p_A->n] > 1000) {
		const ptrdiff_t *p_i = p_A->i;
		int nnz = int(p_A->p[p_A->n]);
		#pragma omp parallel for default(none) shared(Len, p_i, inz) schedule(static)
		for(int i = 0; i < inz; ++ i)
			++ Len[p_i[i]]; // access violation, need to use atomics, slow
	} else
#endif // _OPENMP
	{
		const ptrdiff_t *p_i = p_A->i;
		size_t nnz = p_A->p[n];
		for(size_t i = 0; i < nnz; ++ i, ++ p_i)
			++ Len[*p_i]; // off-diagonal or discounted diagonal
	}
	for(size_t i = 0, n = p_A->n; i < n; ++ i) {
		size_t p1 = p_A->p[i], p2 = p_A->p[i + 1];
		_ASSERTE(p1 < p2); // no empty columns!
		_ASSERTE(p_A->i[p2 - 1] == i); // diagonal is here (skip)
		//-- p2; // skip the diagonal
		_ASSERTE(Len[i] > 0); // makes sure the next line will not underflow
		//-- Len[i]; // counted the diagonal as well
		_ASSERTE(p2 > p1); // makes sure the next line will not overflow
		Len[i] += p2 - p1 - 2; // accounts for diagonal, twice
	}*/
	const ptrdiff_t *Ap = p_A->p, *Ai = p_A->i; // antialiass
	for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
		size_t p2 = Ap[i + 1];
		_ASSERTE(p1 < p2); // no empty columns!
		_ASSERTE(Ai[p2 - 1] == i); // diagonal is here (skip)
		-- p2; // skip the diagonal
		_ASSERTE(p2 >= p1); // makes sure the next line will not overflow
		Len[i] += p2 - p1;
		for(; p1 < p2; ++ p1) {
			size_t j = Ai[p1];
			_ASSERTE(j < i); // upper-triangular and we skipped the diagonal before the loop
			++ Len[j]; // off-diagonal
		}
		++ p1;
		_ASSERTE(p1 == Ap[i + 1]); // this will soo fail on jumbled matrices (_DEBUG checks for them)
	}
	/*//size_t nzaat2 = 0;
	for(size_t i = 0, n = p_A->n; i < n; ++ i) {
		size_t p1 = p_A->p[i], p2 = p_A->p[i + 1];
		_ASSERTE(p1 < p2); // no empty columns!
		_ASSERTE(p_A->i[p2 - 1] == i); // diagonal is here (skip)
		-- p2; // skip the diagonal
		_ASSERTE(p2 >= p1); // makes sure the next line will not overflow
		Len[i] += p2 - p1;
		for(; p1 < p2; ++ p1) {
			size_t j = p_A->i[p1];
			_ASSERTE(j <= i); // upper-triangular
			_ASSERTE(i != j); // we skipped the diagonal before the loop
			//if(i != j) {
			//++ Len[i]; // excluding diagonal! // moved above
			//++ nzaat2;
				++ Len[j]; // off-diagonal
				//++ nzaat2;
			//}
		}
	}
	//_ASSERTE(nzaat == nzaat2);*/
	// simpler implementation of camd_aat() for upper-triangular matrices
#else // 1
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	size_t nzaat = __camd_internal::camd_l_aat(n, p_A->p, p_A->i,
		(SuiteSparse_long*)Len, (SuiteSparse_long*)&m_ordering[0], 0);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	size_t nzaat = __camd_internal::camd_aat(n, p_A->p, p_A->i,
		(int*)Len, (int*)&m_ordering[0], 0);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#endif // 1
	_ASSERTE((std::max(nz, n) - n <= nzaat) && (nzaat <= 2 * (size_t)nz));
	// determine the symmetry and count off-diagonal nonzeros in A+A'

	size_t slen = nzaat;
	if(slen > SIZE_MAX - nzaat / 5) // check overflow
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY
	slen += nzaat / 5;
	size_t n1 = n + 1; // for sure does not overflow
	if(n1 > SIZE_MAX / 8 || (n1 == SIZE_MAX && SIZE_MAX % 8) || slen > SIZE_MAX - 8 * n1) // check overflow
		throw std::bad_alloc(); // would get CAMD_OUT_OF_MEMORY
	slen += 8 * n1; // note it is 1 bigger than it has to be // todo - check if we can save memory (probably not, CAMD requires 1 more than AMD, that will be it)
	if(m_camd_workspace1.capacity() < slen)
		m_camd_workspace1.clear(); // avoid copying data if it needs to realloc
	m_camd_workspace1.resize(slen);
	size_t *S = &m_camd_workspace1[0];
	// allocate workspace for matrix, elbow room (size n), and 7 (n + alignment) vectors

#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
	__camd_internal::camd_l1(n, p_A->p, p_A->i, (SuiteSparse_long*)&m_ordering[0],
		(SuiteSparse_long*)Pinv, (SuiteSparse_long*)Len, slen,
		(SuiteSparse_long*)S, 0, 0, (SuiteSparse_long*)p_constraints);
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	__camd_internal::camd_1(n, p_A->p, p_A->i, (int*)&m_ordering[0], (int*)Pinv,
		(int*)Len, slen, (int*)S, 0, 0, (int*)p_constraints);
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
	// order the matrix

	// note that camd_1 constructs A^T | A (A is a binary matrix) first,
	// it could be optimized / calculated inplace / implemented directly in block matrix // todo

	return &m_ordering[0];
	// return ordering
}
#endif // 0

const size_t *CMatrixOrdering::p_ExtendBlockOrdering_with_Identity(size_t n_new_size) // throw(std::bad_alloc)
{
	size_t n_old_size = m_ordering.size();
	_ASSERTE(n_new_size >= n_old_size);
	m_ordering.resize(n_new_size);
	for(size_t i = n_old_size; i < n_new_size; ++ i)
		m_ordering[i] = i;
	// extend ordering with identity ordering

	return &m_ordering[0];
	// return ordering
}

bool CMatrixOrdering::b_IsValidOrdering(const size_t *p_order, size_t n_size) // throw(std::bad_alloc)
{
	for(size_t i = 0; i < n_size; ++ i) {
		if(p_order[i] >= n_size)
			return false;
	}
	// primitive check - see if there are other values than they should be

	std::vector<bool> coverage(n_size, false);
	// make a coverage mask

	for(size_t i = 0; i < n_size; ++ i) {
		if(coverage[p_order[i]])
			return false;
		// there is a repeated element

		coverage[p_order[i]] = true;
	}
	// check for repeated elements

#ifdef _DEBUG
	for(size_t i = 0; i < n_size; ++ i)
		_ASSERTE(coverage[p_order[i]]);
	// all items are covered (already implied by the two checks above)
#endif // _DEBUG

	return true;
}

bool CMatrixOrdering::b_IsIdentityOrdering(const size_t *p_order, size_t n_size)
{
	for(size_t i = 0; i < n_size; ++ i) {
		if(p_order[i] != i)
			return false;
	}
	return true;
}

const size_t *CMatrixOrdering::p_AdjacentLabel_Ordering(const std::vector<size_t> &r_vertex_label,
	size_t n_label_num) // throw(std::bad_alloc)
{
	_ASSERTE(!n_label_num || *std::min_element(r_vertex_label.begin(), r_vertex_label.end()) == 0); // the lowest label must be 0
	_ASSERTE(!n_label_num || n_label_num - 1 ==
		*std::max_element(r_vertex_label.begin(), r_vertex_label.end())); // the greatest label must be n_label_num - 1
	// make sure n_label_num matches maximal value of values in r_vertex_label

	if(m_ordering.capacity() < r_vertex_label.size()) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(r_vertex_label.size(), 2 * m_ordering.capacity()));
	}
	m_ordering.resize(r_vertex_label.size());
	// maintain an ordering array

	std::vector<size_t> &vertex_order = m_ordering;
	const std::vector<size_t> &vertex_subgraph = r_vertex_label;
	// rename

	{
		std::vector<size_t> subgraph_sizes(n_label_num, 0);
		for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i) {
			size_t n_subgraph = vertex_subgraph[i];
			_ASSERTE(n_subgraph != size_t(-1)); // make sure all the vertices are assigned
			vertex_order[i] = subgraph_sizes[n_subgraph];
			++ subgraph_sizes[n_subgraph]; // could increment twice to save vertex_order of storage in exchange for subgraph_sizes storage
		}
		// sum of vertex ranks, generate subgraph sizes

		_ASSERTE(subgraph_sizes.empty() || *std::min_element(subgraph_sizes.begin(),
			subgraph_sizes.end()) > 0);
		// make sure each subgraph id is used at least once (input validation) 

		for(size_t i = 0, n = subgraph_sizes.size(), n_sum = 0; i < n; ++ i) {
			size_t n_temp = n_sum;
			n_sum += subgraph_sizes[i];
			subgraph_sizes[i] = n_temp;
		}
		// exclusive scan

		for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i)
			vertex_order[i] += subgraph_sizes[vertex_subgraph[i]];
		// add exclusive sums of subgraph sizes
	}
	// generate vertex ordering that orders subgraphs adjacently

	_ASSERTE(vertex_order.empty() || b_IsValidOrdering(&vertex_order.front(), vertex_order.size()));
	// make sure it is a valid ordering to begin with

#ifdef _DEBUG
	std::vector<size_t> inv_vertex_order(vertex_order.size()); // well thats unfortunate
	for(size_t i = 0, n = vertex_order.size(); i < n; ++ i) // O(n)
		inv_vertex_order[vertex_order[i]] = i;
	_ASSERTE(vertex_order.empty() || vertex_subgraph[inv_vertex_order[0]] == 0);
	for(size_t i = 1, n = vertex_order.size(); i < n; ++ i) { // also O(n)
		size_t n_vertex_i = inv_vertex_order[i];
		size_t n_vertex_prev = inv_vertex_order[i - 1];
		_ASSERTE(vertex_subgraph[n_vertex_i] == vertex_subgraph[n_vertex_prev] ||
			vertex_subgraph[n_vertex_i] == vertex_subgraph[n_vertex_prev] + 1);
	}
	// make sure that different subgraphs are ordered in contiguous manner
#endif // _DEBUG

	return (m_ordering.empty())? 0 : &m_ordering.front();
	// return ordering
}

#if 0 // p_AdjacentLabel_Ordering() and n_Find_BlockStructure_Subgraphs() tests
#include <map>

static class CTestAdjacentOrderings {
public:
	CTestAdjacentOrderings()
	{
		Test(1, 1, false);
		Test(1, 1, true);
		Test(0, 0, false);
		Test(0, 0, true);
		Test(20, 1, false);
		Test(20, 1, true);
		Test(20, 4, false);
		Test(20, 4, true);
		Test(20, 20, false);
		Test(20, 20, true);
		Test(20, 13, false);
		Test(20, 13, true);
		Test(17, 1, false);
		Test(17, 1, true);
		Test(17, 4, false);
		Test(17, 4, true);
		Test(17, 17, false);
		Test(17, 17, true);
		Test(17, 13, false);
		Test(17, 13, true);
		for(int i = 0; i < 10000; ++ i) {
			int n = 1 + rand() % 32;
			int l = 1 + rand() % n;
			Test(n, l, false);
			Test(n, l, true);
		}
		// run some tests
	}

	void Test(size_t n_vertex_num, size_t n_label_num, bool b_use_full_matrices = true)
	{
		_ASSERTE(!n_vertex_num || n_label_num > 0); // the first half is only to allow Test(0, 0)
		_ASSERTE(n_vertex_num >= n_label_num);
		std::vector<size_t> labels(n_vertex_num);
		for(size_t i = 0; i < n_label_num; ++ i)
			labels[i] = i;
		for(size_t i = n_label_num; i < n_vertex_num; ++ i)
			labels[i] = rand() % n_label_num;
		// generate random labels, make sure each label is present at least once

		std::vector<size_t> perm(n_vertex_num), inv_perm(n_vertex_num);
		for(size_t i = 0; i < n_vertex_num; ++ i)
			perm[i] = i;
		for(size_t i = 1; i < n_vertex_num; ++ i)
			std::swap(perm[i - 1], perm[i + rand() % (n_vertex_num - i)]);
		_ASSERTE(CMatrixOrdering::b_IsValidOrdering((perm.empty())? 0 : &perm.front(), perm.size()));
		for(size_t i = 1; i < n_vertex_num; ++ i)
			inv_perm[perm[i]] = i;
		// make a random ordering

		std::vector<size_t> cumsums(n_vertex_num);
		for(size_t i = 0; i < n_vertex_num; ++ i)
			cumsums[i] = i + 1;
		CUberBlockMatrix A(cumsums.begin(), cumsums.end(), cumsums.begin(), cumsums.end());
		for(size_t i = 0; i < n_vertex_num; ++ i) {
			size_t l = labels[i];
			// for each vertex

			A.Append_Block(Eigen::Matrix<double, 1, 1>::Identity(), i, i);
			for(size_t j = i + 1; j < n_vertex_num; ++ j) {
				if(labels[j] == l) {
					A.Append_Block(Eigen::Matrix<double, 1, 1>::Identity(), i, j);
					A.Append_Block(Eigen::Matrix<double, 1, 1>::Identity(), j, i);
				}
			}
			// add all vertices that are adjacent to it (including itself)
		}
		// build a block matrix of the vertices

		//A.Rasterize("00_adjacency_test_A.tga");
		// debug

		CUberBlockMatrix A_upper;
		if(b_use_full_matrices) {
			if(n_vertex_num)
				A.SliceTo(A_upper, 0, n_vertex_num, 0, n_vertex_num, true);
			else
				A_upper.Clear(); // SliceTo() requires non-empty output
		} else
			A_upper.TriangularViewOf(A, true, true);
		// get an upper/full view of A

		//A_upper.Rasterize("01_adjacency_test_A-upper.tga");
		// debug

		std::vector<size_t> recovererd_labels;
		size_t n_recovered_label_num =
			CMatrixOrdering::n_Find_BlockStructure_Subgraphs(recovererd_labels,
			A_upper, b_use_full_matrices);

		_ASSERTE(b_Equal_LabelSets(labels, recovererd_labels));

		std::vector<size_t> recovererd_labels_cann = recovererd_labels;
		Canonicalize_LabelSet(recovererd_labels_cann);
		_ASSERTE(!b_use_full_matrices || recovererd_labels_cann == recovererd_labels);

		CUberBlockMatrix A_upper_perm;
		if(b_use_full_matrices) {
			A_upper.PermuteTo(A_upper_perm,
				(perm.empty())? 0 : &perm.front(), perm.size(), true, true, true);
		} else {
			A_upper.Permute_UpperTriangular_To(A_upper_perm,
				(perm.empty())? 0 : &perm.front(), perm.size(), true);
		}
		// permute A, also upper

		//A_upper_perm.Rasterize("02_adjacency_test_A-upper-perm.tga");
		// debug

		std::vector<size_t> recovererd_perm_labels;
		size_t n_recovered_perm_label_num =
			CMatrixOrdering::n_Find_BlockStructure_Subgraphs(recovererd_perm_labels,
			A_upper_perm, b_use_full_matrices);

		std::vector<size_t> perm_labels(n_vertex_num);
		for(size_t i = 0; i < n_vertex_num; ++ i)
			perm_labels[perm[i]] = labels[i];
		// permute the labels in the same way the matrix is permuted

		_ASSERTE(b_Equal_LabelSets(perm_labels, recovererd_perm_labels));

		std::vector<size_t> recovererd_perm_labels_cann = recovererd_perm_labels;
		Canonicalize_LabelSet(recovererd_perm_labels_cann);
		_ASSERTE(!b_use_full_matrices || recovererd_perm_labels_cann == recovererd_perm_labels);

		std::vector<size_t> first_vertex;
		CMatrixOrdering mord;
		const size_t *p_ord = mord.p_AdjacentLabel_Ordering(first_vertex, perm_labels, n_label_num);
		// get adjacent label ordering

		//_ASSERTE((n_label_num != 1 && n_label_num != n_vertex_num) ||
		//	CMatrixOrdering::b_IsIdentityOrdering(p_ord, mord.n_Ordering_Size())); // not here, perm_labels might be reordered and not canonical
		// in case all vertices are independent or in case all are a part of a single
		// clique, make sure the returned ordering is identity one

		CMatrixOrdering mord2;
		const size_t *p_ord2 = mord2.p_AdjacentLabel_Ordering(perm_labels, n_label_num);
		_ASSERTE(mord.n_Ordering_Size() == mord2.n_Ordering_Size());
		_ASSERTE(!memcmp(p_ord, p_ord2, mord2.n_Ordering_Size() * sizeof(size_t)));
		// make sure this returns the same thing as the function without the first_vertex argument

		CUberBlockMatrix A_upper_unperm;
		if(b_use_full_matrices)
			A_upper_perm.PermuteTo(A_upper_unperm, p_ord, mord.n_Ordering_Size(), true, true, true);
		else
			A_upper_perm.Permute_UpperTriangular_To(A_upper_unperm, p_ord, mord.n_Ordering_Size(), true);
		// permute A, also upper

		//A_upper_unperm.Rasterize("03_adjacency_test_A-upper-unperm.tga");
		// debug

		for(int n_pass = 0;; ++ n_pass) { // break in the middle below
			//std::vector<size_t> cannon_labels = perm_labels;
			//Canonicalize_LabelSet(cannon_labels);

			_ASSERTE(first_vertex.size() == n_label_num); // should be the same number
			for(size_t i = 0; i < n_label_num; ++ i) {
				//_ASSERTE(A_upper_unperm.n_BlockColumn_Block_Num(first_vertex[i]) == 1); // a single nnz block
				//_ASSERTE(A_upper_unperm.n_Block_Row(first_vertex[i], 0) == first_vertex[i]); // and a diagonal block to be exact
				// make sure this is a column with a single diagonal item in an
				// upper-triangular view of the symmetric matrix

				size_t n_first = first_vertex[i], n_last = ((i + 1) < n_label_num)?
					first_vertex[i + 1] : A_upper_unperm.n_BlockColumn_Num();
				for(size_t j = n_first; j < n_last; ++ j) {
					size_t n_block_num = A_upper_unperm.n_BlockColumn_Block_Num(first_vertex[i]);
					_ASSERTE(n_block_num > 0); // make sure it is not empry
					_ASSERTE(A_upper_unperm.n_Block_Row(j, 0) >= n_first);
					_ASSERTE(A_upper_unperm.n_Block_Row(j, n_block_num - 1) < n_last);
				}
				// make sure the non-zeros in columns [n_first, n_last) are boxed
				// in a square with its diagonal coincident with the matrix diagonal

				/*size_t n_first_label_position = std::find(perm_labels.begin(), perm_labels.end(), i) - perm_labels.begin();
				size_t n_first_olabel_position = std::find(labels.begin(), labels.end(), i) - labels.begin();
				size_t n_first_clabel_position = std::find(cannon_labels.begin(), cannon_labels.end(), i) - cannon_labels.begin();
				size_t n_first_label_vertex = perm[n_first_label_position];
				size_t n_first_olabel_vertex = perm[n_first_olabel_position];
				size_t n_first_clabel_vertex = perm[n_first_clabel_position];
				size_t n_ifirst_label_vertex = inv_perm[n_first_label_position];
				size_t n_ifirst_olabel_vertex = inv_perm[n_first_olabel_position];
				size_t n_ifirst_clabel_vertex = inv_perm[n_first_clabel_position];*/
				// no meaning whatsoever
			}
			// make sure that the first_vertex array does point to the first vertices in a block in the permuted matrix

			if(n_pass)
				break;

			p_ord = mord.p_AdjacentLabel_Ordering(first_vertex,
				recovererd_perm_labels, n_recovered_perm_label_num/*perm_labels, n_label_num*/);
			// get adjacent label ordering

			p_ord2 = mord2.p_AdjacentLabel_Ordering(recovererd_perm_labels,
				n_recovered_perm_label_num/*perm_labels, n_label_num*/);
			_ASSERTE(mord.n_Ordering_Size() == mord2.n_Ordering_Size());
			_ASSERTE(!memcmp(p_ord, p_ord2, mord2.n_Ordering_Size() * sizeof(size_t)));
			// make sure this returns the same thing as the function without the first_vertex argument

			if(b_use_full_matrices)
				A_upper_perm.PermuteTo(A_upper_unperm, p_ord, mord.n_Ordering_Size(), true, true, true);
			else
				A_upper_perm.Permute_UpperTriangular_To(A_upper_unperm, p_ord, mord.n_Ordering_Size(), true);
			// do it again, use the recovered label sets (which can have different label id assignments)

			//A_upper_unperm.Rasterize("04_adjacency_test_A-upper-unperm-own.tga");
			// debug

			_ASSERTE((n_label_num != 1 && n_label_num != n_vertex_num) ||
				CMatrixOrdering::b_IsIdentityOrdering(p_ord, mord.n_Ordering_Size())); // here it should hold
			// in case all vertices are independent or in case all are a part of a single
			// clique, make sure the returned ordering is identity one
		}
	}

	/**
	 *	@brief compare two label sets by first making them canonical and then comparing them
	 *
	 *	@param[in] A is a label set to be compared
	 *	@param[in] B is a label set to be compared
	 *
	 *	@return Returns true if the label sets contain the same labelling (up to the assignment
	 *		of label ids), otherwise returns false.
	 */
	bool b_Equal_LabelSets(std::vector<size_t> A, std::vector<size_t> B) // throw(std::bad_alloc) // copy intended
	{
		Canonicalize_LabelSet(A);
		Canonicalize_LabelSet(B);
		return A == B;
	}

	/**
	 *	@brief convert label set to a canonical one by mapping all the indices to different
	 *		indices so that taking each first occurence of an index gives a contiguous
	 *		sequence starting with zero
	 *
	 *	@param[in,out] r_labels is a label set to convert to a canonical one
	 */
	static void Canonicalize_LabelSet(std::vector<size_t> &r_labels)
	{
		size_t n = r_labels.size(), l = 0;
		std::map<size_t, size_t> p;
		for(size_t i = 0; i < n; ++ i) {
			if(!p.count(r_labels[i])) {
				p[r_labels[i]] = l;
				++ l;
			}
			r_labels[i] = p[r_labels[i]];
		}
	}
} run_test;
#endif // 0

const size_t *CMatrixOrdering::p_AdjacentLabel_Ordering(std::vector<size_t> &r_first_vertex,
	const std::vector<size_t> &r_vertex_label, size_t n_label_num) // throw(std::bad_alloc)
{
	_ASSERTE(!n_label_num || *std::min_element(r_vertex_label.begin(), r_vertex_label.end()) == 0); // the lowest label must be 0
	_ASSERTE(!n_label_num || n_label_num - 1 ==
		*std::max_element(r_vertex_label.begin(), r_vertex_label.end())); // the greatest label must be n_label_num - 1
	// make sure n_label_num matches maximal value of values in r_vertex_label

	r_first_vertex.clear();
	r_first_vertex.resize(n_label_num);
	// allocate an array of first-vertices

	if(m_ordering.capacity() < r_vertex_label.size()) {
		m_ordering.clear(); // avoid copying data in resizing
		m_ordering.reserve(std::max(r_vertex_label.size(), 2 * m_ordering.capacity()));
	}
	m_ordering.resize(r_vertex_label.size());
	// maintain an ordering array

	std::vector<size_t> &vertex_order = m_ordering;
	const std::vector<size_t> &vertex_subgraph = r_vertex_label;
	// rename

	{
		std::vector<size_t> subgraph_sizes(n_label_num, 0);

		r_first_vertex.swap(subgraph_sizes); // will use subgraph_sizes as a temp for r_first_vertex

		for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i) {
			size_t n_subgraph = vertex_subgraph[i];
			_ASSERTE(n_subgraph != size_t(-1)); // make sure all the vertices are assigned
			vertex_order[i] = subgraph_sizes[n_subgraph];
			if(++ subgraph_sizes[n_subgraph] == 1)
				r_first_vertex[n_subgraph] = i;
		}
		// sum of vertex ranks, generate subgraph sizes

		_ASSERTE(subgraph_sizes.empty() || *std::min_element(subgraph_sizes.begin(),
			subgraph_sizes.end()) > 0);
		// make sure each subgraph id is used at least once (input validation) 

		for(size_t i = 0, n = subgraph_sizes.size(), n_sum = 0; i < n; ++ i) {
			size_t n_temp = n_sum;
			n_sum += subgraph_sizes[i];
			subgraph_sizes[i] = n_temp;
		}
		// exclusive scan

		for(size_t i = 0, n = vertex_subgraph.size(); i < n; ++ i)
			vertex_order[i] += subgraph_sizes[vertex_subgraph[i]];
		// add exclusive sums of subgraph sizes

		r_first_vertex.swap(subgraph_sizes); // swap it back before the caller notices

		for(size_t i = 0; i < n_label_num; ++ i)
			r_first_vertex[i] = m_ordering[subgraph_sizes[i]];
		// permute the first vertices
	}
	// generate vertex ordering that orders subgraphs adjacently

	_ASSERTE(vertex_order.empty() || b_IsValidOrdering(&vertex_order.front(), vertex_order.size()));
	// make sure it is a valid ordering to begin with

#ifdef _DEBUG
	std::vector<size_t> inv_vertex_order(vertex_order.size()); // well thats unfortunate
	for(size_t i = 0, n = vertex_order.size(); i < n; ++ i) // O(n)
		inv_vertex_order[vertex_order[i]] = i;
	_ASSERTE(vertex_order.empty() || vertex_subgraph[inv_vertex_order[0]] == 0);
	_ASSERTE(vertex_order.empty() || r_first_vertex[0] == 0); // make sure r_first_vertex[0] is zero // this might be all wrong // todo: test this with randomly permuted block block matrix
	for(size_t i = 1, n = vertex_order.size(); i < n; ++ i) { // also O(n)
		size_t n_vertex_i = inv_vertex_order[i];
		size_t n_vertex_prev = inv_vertex_order[i - 1];
		_ASSERTE(vertex_subgraph[n_vertex_i] == vertex_subgraph[n_vertex_prev] ||
			vertex_subgraph[n_vertex_i] == vertex_subgraph[n_vertex_prev] + 1);
	}
	for(size_t i = 1; i < n_label_num; ++ i) { // todo: test this with randomly permuted block block matrix
		size_t v = inv_vertex_order[r_first_vertex[i]];
		size_t v_prev = inv_vertex_order[r_first_vertex[i - 1]];
		_ASSERTE(r_first_vertex[i] > r_first_vertex[i - 1]); // strictly ordered
		_ASSERTE(vertex_subgraph[v] == vertex_subgraph[v_prev] + 1); // make sure this is the first vertex of the next subgraph
		_ASSERTE(vertex_subgraph[v] == i);
	}
	// make sure that different subgraphs are ordered in contiguous manner
#endif // _DEBUG

	return (m_ordering.empty())? 0 : &m_ordering.front();
	// return ordering
}

size_t CMatrixOrdering::n_Find_BlockStructure_Subgraphs(std::vector<size_t> &r_vertex_subgraph,
	const CUberBlockMatrix &r_A, bool b_is_full_matrix /*= false*/) // throw(std::bad_alloc)
{
	_ASSERTE(b_is_full_matrix || (r_A.b_SymmetricLayout() /*&& r_A.b_UpperBlockTriangular()*/)); // simply ignore the lower triangle
	// if b_is_full_matrix = false, A must be square and upper

	_ASSERTE(!b_is_full_matrix || r_A.b_SymmetricBlockStructure());
	// if b_is_full_matrix = true, A must be symmetric

	r_vertex_subgraph.clear();
	r_vertex_subgraph.resize(r_A.n_BlockColumn_Num(), size_t(-1));
	size_t n_subgraph_num = 0;
	if(b_is_full_matrix) { // the two branches differ in 1) looping order and 2) inner loop skip condition
		for(size_t i = 0, n = r_vertex_subgraph.size(); i < n; ++ i) { // A is full
			if(r_vertex_subgraph[i] != size_t(-1))
				continue;
			// in case a vertex is not marked yet

			std::vector<size_t> recurse_stack; // i dont like including <stack> just for the sake of it
			recurse_stack.push_back(i);
			while(!recurse_stack.empty()) {
				size_t n_vertex = recurse_stack.back();
				recurse_stack.erase(recurse_stack.end() - 1);
				_ASSERTE(r_vertex_subgraph[n_vertex] == size_t(-1) ||
					r_vertex_subgraph[n_vertex] == n_subgraph_num); // might have been already marked by a loop
				// get a vertex ...

				if(r_vertex_subgraph[n_vertex] == size_t(-1)) {
					r_vertex_subgraph[n_vertex] = n_subgraph_num;
					// assign a subgraph id

					for(size_t j = 0, m = r_A.n_BlockColumn_Block_Num(n_vertex); j < m; ++ j) {
						size_t n_neighbor = r_A.n_Block_Row(n_vertex, j);
						if(n_neighbor == n_vertex)
							continue; // only up to the diagonal element, avoid looping on self

						_ASSERTE(r_vertex_subgraph[n_neighbor] == size_t(-1) ||
							r_vertex_subgraph[n_neighbor] == n_subgraph_num);
						if(r_vertex_subgraph[n_neighbor] == size_t(-1))
							recurse_stack.push_back(n_neighbor);
					}
					// recurse to all the neighbors
				}
			}
			// mark all the connected vertices

			++ n_subgraph_num;
		}
	} else {
		for(size_t i = r_vertex_subgraph.size(); i > 0;) { // A is upper triangular
			-- i; // here

			if(r_vertex_subgraph[i] != size_t(-1))
				continue;
			// in case a vertex is not marked yet

			std::vector<size_t> recurse_stack; // i dont like including <stack> just for the sake of it
			recurse_stack.push_back(i);
			while(!recurse_stack.empty()) {
				size_t n_vertex = recurse_stack.back();
				recurse_stack.erase(recurse_stack.end() - 1);
				_ASSERTE(r_vertex_subgraph[n_vertex] == size_t(-1) ||
					r_vertex_subgraph[n_vertex] == n_subgraph_num); // might have been already marked by a loop
				// get a vertex ...

				if(r_vertex_subgraph[n_vertex] == size_t(-1)) {
					r_vertex_subgraph[n_vertex] = n_subgraph_num;
					// assign a subgraph id

					for(size_t j = 0, m = r_A.n_BlockColumn_Block_Num(n_vertex); j < m; ++ j) {
						size_t n_neighbor = r_A.n_Block_Row(n_vertex, j);
						if(n_neighbor >= n_vertex)
							break; // only up to the diagonal element, avoid looping on self

						_ASSERTE(r_vertex_subgraph[n_neighbor] == size_t(-1) ||
							r_vertex_subgraph[n_neighbor] == n_subgraph_num);
						if(r_vertex_subgraph[n_neighbor] == size_t(-1))
							recurse_stack.push_back(n_neighbor);
					}
					// recurse to all the neighbors
				}
			}
			// mark all the connected vertices

			++ n_subgraph_num;
		}

		for(size_t i = 0, n = r_vertex_subgraph.size(); i < n; ++ i)
			r_vertex_subgraph[i] = n_subgraph_num - 1 - r_vertex_subgraph[i];
		// reverse subgraph to gain identity ordering on diagonal matrices
		// (the subgraph ids are assigned backwards so otherwise it would
		// yield a mirror ordering)
	}
	// t_odo - debug this on non-trivial graphs, make this a function

#ifdef _DEBUG
	for(size_t i = 0, n = r_A.n_BlockColumn_Num(); i < n; ++ i) {
		for(size_t j = 0, m = r_A.n_BlockColumn_Block_Num(i); j < m; ++ j) {
			size_t n_neighbor = r_A.n_Block_Row(i, j);
			if(n_neighbor >= i)
				break; // ignore the lower half of A, the algorithm above does that as well
			_ASSERTE(r_vertex_subgraph[i] == r_vertex_subgraph[n_neighbor]);
		}
	}
	// make sure that all the neighboring vertices indeed are in the same subgraph
#endif // _DEBUG

	return n_subgraph_num;
}

/*
 *								=== ~CMatrixOrdering ===
 */

/*
 *								=== CMatrixTransposeSum ===
 */

size_t CMatrixTransposeSum::n_ColumnLengths_AAT_Ref(size_t *p_column_lengths,
	size_t n_length_num, const cs *p_matrix, bool b_AAT_with_diagonal /*= false*/) // throw(std::bad_alloc)
{
	cs t_matrix;
	std::vector<csi> col_ptrs_rect;
	if(p_matrix->n != p_matrix->m) {
		t_matrix = *p_matrix; // copy the matrix
		t_matrix.m = t_matrix.n = std::max(p_matrix->n, p_matrix->m); // make it square
		if(p_matrix->n < t_matrix.n) { // in case the number of columns changes ...
			col_ptrs_rect.insert(col_ptrs_rect.end(),
				p_matrix->p, p_matrix->p + (p_matrix->n + 1)); // copy the column pointers
			col_ptrs_rect.resize(t_matrix.n + 1, col_ptrs_rect.back()); // duplicate the last pointer enough times
			t_matrix.p = &col_ptrs_rect.front(); // replace pointer
		}
		p_matrix = &t_matrix; // now use the modified matrix
	}
	// handle rectangular matrices

	_ASSERTE(p_matrix->n == p_matrix->m); // square matrices only
	_ASSERTE(n_length_num == p_matrix->n); // must match

	cs *p_transpose = cs_transpose(p_matrix, 0);
	if(!p_transpose)
		throw std::bad_alloc();
	cs *p_sum = cs_add(p_matrix, p_transpose, 1, 1);
	cs_spfree(p_transpose);
	if(!p_sum)
		throw std::bad_alloc();

	size_t n_nnz = 0;
	for(size_t i = 0, n = p_sum->n; i < n; ++ i) {
		p_column_lengths[i] = p_sum->p[i + 1] - p_sum->p[i];
		// nnz in column i

		if(!b_AAT_with_diagonal && std::find(&p_sum->i[p_sum->p[i]],
		   &p_sum->i[p_sum->p[i + 1]], csi(i)) != &p_sum->i[p_sum->p[i + 1]])
			-- p_column_lengths[i];
		// discount the diagonal if unwanted and present

		n_nnz += p_column_lengths[i];
	}

	cs_spfree(p_sum);

	return n_nnz;
}

size_t CMatrixTransposeSum::n_ColumnLengths_AATNoDiag_UpperFullDiagonal(size_t *p_column_lengths,
	size_t n_length_num, const cs *p_matrix)
{
	_ASSERTE(p_matrix->n == p_matrix->m); // square matrices only
	_ASSERTE(n_length_num == p_matrix->n); // must match

	const csi *Ap = p_matrix->p, *Ai = p_matrix->i;
	const size_t n = n_length_num, nz = Ap[n];

	for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
		const size_t p2 = Ap[i + 1]; // don't modify, will become the next p1
		_ASSERTE(p1 < p2); // no empty columns!
		_ASSERTE(Ai[p2 - 1] == i); // the matrix is upper-trinagular and the diagonal is present
		_ASSERTE(p2 > p1); // makes sure the next line will not underflow

		// the diagonal is counted twice since it is symmetric, have to subtract it twice to get rid of it
		// (if it was supposed to be there, subtract once, if it isnt in the matrix, subtract nothing)

		p_column_lengths[i] = p2 - p1 - 2; // could actually underflow by 1, the diag entry added below will fix it
		p1 = p2;
	}
	for(size_t i = 0; i < nz; ++ i)
		++ p_column_lengths[Ai[i]];

	size_t n_nnz_aat = 2 * (nz - n);
	return n_nnz_aat;
}

size_t CMatrixTransposeSum::n_ColumnLengths_AATNoDiag(size_t *p_column_lengths, size_t n_length_num,
	const cs *p_matrix, size_t *p_workspace, size_t n_workspace_size)
{
	_ASSERTE(n_length_num == std::max(p_matrix->m, p_matrix->n)); // must match
	_ASSERTE(n_workspace_size >= size_t(p_matrix->n)); // must fit

	const csi *Ap = p_matrix->p, *Ai = p_matrix->i;
	const size_t n = p_matrix->n, nz = Ap[n];

	std::fill(p_column_lengths, p_column_lengths + n_length_num, size_t(0)); // zero the lengths
	// this is at least partially required

	//std::fill(p_workspace, p_workspace + n_workspace_size, size_t(0)); // zero the workspace
	// no need to zero workspace

	size_t *p_transpose_col_off = p_workspace; // rename
	// those point to the Ai array

	size_t n_diag_nnz_num = 0, n_sym_nnz_num = 0;
	// the number of elements on the diagonal and (half) the number of elements
	// that have (structural) counterparts on the opposite side of the diagonal

	for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
		const size_t p2 = Ap[i + 1]; // don't modify, will become the next p1
		_ASSERTE(CDebug::b_IsStrictlySortedSet(Ai + p1, Ai + p2)); // must be sorted
		const csi *p_diag = std::lower_bound(Ai + p1, Ai + p2, csi(i)); // find the diagonal // in case the matrix is strictly upper, then this is a slight waste of time
		if(p_diag != Ai + p2 && size_t(*p_diag) == i) { // in case there is a diagonal element
			++ n_diag_nnz_num;
			p_transpose_col_off[i] = p_diag - (Ai /*+ p1*/) + 1; // point to the first below-diagonal element
		} else {
			_ASSERTE(p_diag == Ai + p2 || size_t(*p_diag) > i); // otherwise it is the first below-diagonal element
			p_transpose_col_off[i] = p_diag - (Ai /*+ p1*/);
		}
		_ASSERTE(p_transpose_col_off[i] >= p1 && p_transpose_col_off[i] <= p2); // make sure it points to this column
		_ASSERTE(p_transpose_col_off[i] == p2 || size_t(Ai[p_transpose_col_off[i]]) > i); // make sure it points to a below-diagonal element
		// find the upper triangular portion of this column

		p_column_lengths[i] = p_diag - (Ai + p1);
		// number of (strictly) upper triangular items A(*, i)
		// in case the diagonal should be included in the column length, just use p_transpose_col_off[i] - p1
		// the diagonal entry would be added here

		for(const csi *p_row = Ai + p1; p_row != p_diag; ++ p_row) {
			const size_t j = *p_row;
			_ASSERTE(j < i);
			++ p_column_lengths[j]; // can increment, all the columns below i are already initialized
			// number of (strictly) lower triangular items A(i, *)

			_ASSERTE(j < n); // this array is only indexed by columns
			size_t pj1 = p_transpose_col_off[j]; // already initialized
			const size_t pj2 = Ap[j + 1];
			_ASSERTE(pj1 >= size_t(Ap[j]) && pj1 <= pj2); // make sure it points to its column
			for(; pj1 < pj2; ++ pj1) { // in case the matrix is strictly upper, then this loop will not run
				const size_t ii = Ai[pj1];
				_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
				// even in case the diagonal should be included then ii can't equal j

				if(ii < i) { // A(ii, j) is only in the lower diagonal and not in the upper (assymetric element)
#ifdef _DEBUG
					const csi *p_assym_elem = std::lower_bound(Ai + Ap[ii], Ai + Ap[ii + 1], csi(j));
					_ASSERTE(p_assym_elem == Ai + Ap[ii + 1] || *p_assym_elem != j);
					// make sure A(j, ii) does not exist
#endif // _DEBUG
					++ p_column_lengths[ii]; // A(j, ii) in the upper triangle
					++ p_column_lengths[j]; // A(ii, j) in the lower triangle
				} else if(ii == i) { // A(ii, j) is a mirror of A(j, i), do not count
					_ASSERTE(std::lower_bound(Ai + p1, p_diag, csi(j)) == p_row); // (A(j, ii))' ~ A(i, j) is this element actually
					++ n_sym_nnz_num; // count those entries
				} else
					break; // will do the rest of this transpose row later on
			}
			p_transpose_col_off[j] = pj1; // remember where we stopped
			// ordered merge of transposed columns with the upper triangular entriex of this column
		}
		// loop over the upper elements and count them again (or rather count their transpose images)
		// making std::lower_bound() above saves checking for the value of *p_row
		// inside this loop - fewer comparisons are made

		p1 = p2;
	}

	for(size_t j = 0; j < n; ++ j) {
		_ASSERTE(j < n); // p_transpose_col_off array is only indexed by columns
		_ASSERTE(p_transpose_col_off[j] >= size_t(Ap[j]) && p_transpose_col_off[j] <= size_t(Ap[j + 1])); // make sure it points to its column
		for(size_t pj1 = p_transpose_col_off[j], pj2 = Ap[j + 1]; pj1 < pj2; ++ pj1) {
			const size_t ii = Ai[pj1];
			_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
			// even in case the diagonal should be included then ii can't equal j

#ifdef _DEBUG
			if(ii < n) { // !!
				const csi *p_assym_elem = std::lower_bound(Ai + Ap[ii], Ai + Ap[ii + 1], csi(j));
				_ASSERTE(p_assym_elem == Ai + Ap[ii + 1] || *p_assym_elem != j);
				// make sure A(j, ii) does not exist
			}
#endif // _DEBUG

			++ p_column_lengths[ii]; // A(j, ii) in the upper triangle
			++ p_column_lengths[j]; // A(ii, j) in the lower triangle
		}
	}
	// handle entries in transpose rows which were previously not matched

	_ASSERTE(nz >= n_diag_nnz_num && nz - n_diag_nnz_num >= n_sym_nnz_num); // make sure the below line does not overflow
	return (nz - n_diag_nnz_num - n_sym_nnz_num) * 2;
	// in case the diagonal should be included, add n_diag_nnz_num back
}

void CMatrixTransposeSum::AAT_Ref(size_t *p_column_ptrs, size_t n_column_ptr_num,
	size_t *p_row_inds, size_t n_nnz_num, const size_t *p_column_lengths, size_t n_length_num,
	const cs *p_matrix, bool b_AAT_with_diagonal /*= false*/) // throw(std::bad_alloc)
{
	cs t_matrix;
	std::vector<csi> col_ptrs_rect;
	if(p_matrix->n != p_matrix->m) {
		t_matrix = *p_matrix; // copy the matrix
		t_matrix.m = t_matrix.n = std::max(p_matrix->n, p_matrix->m); // make it square
		if(p_matrix->n < t_matrix.n) { // in case the number of columns changes ...
			col_ptrs_rect.insert(col_ptrs_rect.end(),
				p_matrix->p, p_matrix->p + (p_matrix->n + 1)); // copy the column pointers
			col_ptrs_rect.resize(t_matrix.n + 1, col_ptrs_rect.back()); // duplicate the last pointer enough times
			t_matrix.p = &col_ptrs_rect.front(); // replace pointer
		}
		p_matrix = &t_matrix; // now use the modified matrix
	}
	// handle rectangular matrices

	_ASSERTE(p_matrix->n == p_matrix->m); // square matrices only
	_ASSERTE(n_column_ptr_num == p_matrix->n + 1); // must match
	_ASSERTE(n_length_num == p_matrix->n); // must match

	cs *p_transpose = cs_transpose(p_matrix, 0);
	if(!p_transpose)
		throw std::bad_alloc();
	cs *p_sum = cs_add(p_matrix, p_transpose, 1, 1);
	cs_spfree(p_transpose);
	if(!p_sum)
		throw std::bad_alloc();

	size_t n_nnz = 0;
	for(size_t i = 0, n = p_sum->n; i < n; ++ i) {
		size_t p_column_length_I = p_sum->p[i + 1] - p_sum->p[i];
		// nnz in column i

		p_column_ptrs[i] = n_nnz;

		const csi *p_diag_elem;
		if(!b_AAT_with_diagonal && (p_diag_elem = std::find(&p_sum->i[p_sum->p[i]],
		   &p_sum->i[p_sum->p[i + 1]], csi(i))) != &p_sum->i[p_sum->p[i + 1]]) {
			-- p_column_length_I;
			size_t *p_next = std::copy((const csi*)&p_sum->i[p_sum->p[i]], p_diag_elem,
				p_row_inds + p_column_ptrs[i]);
			std::copy(p_diag_elem + 1, (const csi*)&p_sum->i[p_sum->p[i + 1]], p_next);
		} else {
			std::copy(&p_sum->i[p_sum->p[i]], &p_sum->i[p_sum->p[i + 1]],
				p_row_inds + p_column_ptrs[i]);
		}
		// discount the diagonal if unwanted and present

		std::sort(p_row_inds + p_column_ptrs[i], p_row_inds + (p_column_ptrs[i] + p_column_length_I));
		// ugh ... cs_add() jumbles matrices? // seems like

		n_nnz += p_column_length_I;
	}
	p_column_ptrs[p_sum->n] = n_nnz; // !!

#ifdef _DEBUG
	for(size_t j = 0, n_cumsum = 0, n = p_sum->n; j < n; ++ j) {
		_ASSERTE(p_column_ptrs[j] == n_cumsum); // points at the beginning of the column
		n_cumsum += p_column_lengths[j];
		_ASSERTE(CDebug::b_IsStrictlySortedSet(p_row_inds + p_column_ptrs[j],
			p_row_inds + p_column_ptrs[j + 1])); // make sure the result is not jumbled (should always be sorted)
	}
	_ASSERTE(p_column_ptrs[p_sum->n] == n_nnz_num); // ...
	// make sure all the columns are filled, according to p_column_lengths
#endif // _DEBUG

	cs_spfree(p_sum);
}

void CMatrixTransposeSum::AATNoDiag_UpperFullDiagonal(size_t *p_column_ptrs,
	size_t n_column_ptr_num, size_t *p_row_inds, size_t n_nnz_num,
	const size_t *p_column_lengths, size_t n_length_num, const cs *p_matrix)
{
	_ASSERTE(p_matrix->n == p_matrix->m); // square matrices only
	_ASSERTE(p_column_ptrs != p_column_lengths &&
		p_column_ptrs + 1 != p_column_lengths); // does not work inplace, although it could
	_ASSERTE(n_length_num == p_matrix->n); // must match
	_ASSERTE(n_column_ptr_num == p_matrix->n + 1); // must match
	//_ASSERTE(n_nnz_num == p_matrix->p[p_matrix->n]); // not this!

	const csi *Ap = p_matrix->p, *Ai = p_matrix->i;
	const size_t n = n_length_num, nz = Ap[n];

#ifdef _DEBUG
	std::vector<size_t> column_end_list(n);
#endif // _DEBUG

	*p_column_ptrs = 0; // filled explicitly
	size_t *p_column_dest = p_column_ptrs + 1;
	size_t n_nnz_sum = 0;
	for(size_t j = 0; j < n; ++ j) {
		p_column_dest[j] = n_nnz_sum;
		n_nnz_sum += p_column_lengths[j];
#ifdef _DEBUG
		column_end_list[j] = n_nnz_sum;
#endif // _DEBUG
	}
	_ASSERTE(n_nnz_sum == n_nnz_num); // must match
	// takes a cumsum of column lengths to generate the destination pointers

	for(size_t k = 0; k < n; ++ k) {
		size_t p1 = Ap[k], p2 = Ap[k + 1];

		_ASSERTE(p1 < p2); // no empty columns!
		_ASSERTE(Ai[p2 - 1] == k); // diagonal is here (skip)
		-- p2; // skip the diagonal
		_ASSERTE(p2 >= p1); // makes sure the next line will not overflow

		for(size_t p = p1; p < p2; ++ p) {
			size_t j = Ai[p];
			_ASSERTE(j >= 0 && j < n);
			_ASSERTE(j < k); // only above-diagonal elements
			// scan the upper triangular part of A

#ifdef _DEBUG
			_ASSERTE(size_t(p_column_dest[j]) < column_end_list[j]);
			_ASSERTE(size_t(p_column_dest[k]) < column_end_list[k]);
#endif // _DEBUG
			p_row_inds[p_column_dest[j] ++] = k;
			p_row_inds[p_column_dest[k] ++] = j;
			// entry A(j, k) in the strictly upper triangular part
		}
		// construct one column of A+A^T
	}
	// builds A+A^T from a symmetric matrix (t_odo - make this a part of CUberBlockMatrix, avoid forming A, build A+A^T directly)

#ifdef _DEBUG
	for(size_t j = 0, n_cumsum = 0; j < n; ++ j) {
		_ASSERTE(p_column_ptrs[j] == n_cumsum); // points at the beginning of the column
		n_cumsum += p_column_lengths[j];
		_ASSERTE(p_column_dest[j] == n_cumsum); // points at the end of the column
	}
	// make sure all the columns are filled, according to p_column_lengths
#endif // _DEBUG
}

void CMatrixTransposeSum::AATNoDiag(size_t *p_column_ptrs, size_t n_column_ptr_num,
	size_t *p_row_inds, size_t n_nnz_num, const size_t *p_column_lengths, size_t n_length_num,
	const cs *p_matrix, size_t *p_workspace, size_t n_workspace_size)
{
	_ASSERTE(n_length_num == std::max(p_matrix->m, p_matrix->n)); // must match
	_ASSERTE(n_column_ptr_num == std::max(p_matrix->m, p_matrix->n) + 1); // must match
	_ASSERTE(n_workspace_size >= size_t(p_matrix->n)); // must fit

	const csi *Ap = p_matrix->p, *Ai = p_matrix->i;
	const size_t n = p_matrix->n, nz = Ap[n];

	*p_column_ptrs = 0; // filled explicitly
	size_t *p_column_dest = p_column_ptrs + 1;
	size_t n_nnz_sum = 0;
	for(size_t j = 0; j < n_length_num; ++ j) { // not n!
		p_column_dest[j] = n_nnz_sum;
		n_nnz_sum += p_column_lengths[j];
	}
	_ASSERTE(n_nnz_sum == n_nnz_num); // must match
	// takes a cumsum of column lengths to generate the destination pointers

	//std::fill(p_column_lengths, p_column_lengths + n_length_num, size_t(0)); // zero the lengths
	//std::fill(p_workspace, p_workspace + n_workspace_size, size_t(0)); // zero the workspace
	// no need to zero anything

	size_t *p_transpose_col_off = p_workspace; // rename
	// those point to the Ai array

	size_t n_diag_nnz_num = 0, n_sym_nnz_num = 0;
	// the number of elements on the diagonal and (half) the number of elements
	// that have (structural) counterparts on the opposite side of the diagonal

	bool b_will_have_unsorted = false, b_filled_from_upper = false;
	// detect if sorting the row indices will be needed

	for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
		const size_t p2 = Ap[i + 1]; // don't modify, will become the next p1
		_ASSERTE(CDebug::b_IsStrictlySortedSet(Ai + p1, Ai + p2)); // must be sorted
		const csi *p_diag = std::lower_bound(Ai + p1, Ai + p2, csi(i)); // find the diagonal // in case the matrix is strictly upper, then this is a slight waste of time
		if(p_diag != Ai + p2 && size_t(*p_diag) == i) { // in case there is a diagonal element
			++ n_diag_nnz_num;
			p_transpose_col_off[i] = p_diag - (Ai /*+ p1*/) + 1; // point to the first below-diagonal element
		} else {
			_ASSERTE(p_diag == Ai + p2 || size_t(*p_diag) > i); // otherwise it is the first below-diagonal element
			p_transpose_col_off[i] = p_diag - (Ai /*+ p1*/);
		}
		_ASSERTE(p_transpose_col_off[i] >= p1 && p_transpose_col_off[i] <= p2); // make sure it points to this column
		_ASSERTE(p_transpose_col_off[i] == p2 || size_t(Ai[p_transpose_col_off[i]]) > i); // make sure it points to a below-diagonal element
		// find the upper triangular portion of this column

		size_t n_add_nnz = p_diag - (Ai + p1);
		// number of (strictly) upper triangular items A(*, i)
		// in case the diagonal should be included in the column length, just use p_transpose_col_off[i] - p1

		std::copy(Ai + p1, p_diag, p_row_inds + p_column_dest[i]); // add the first n_add_nnz entries of column i
		// the diagonal entry would be added here

		p_column_dest[i] += n_add_nnz; // don't forget to shift the destination

		if(n_add_nnz)
			b_filled_from_upper = true;

		for(const csi *p_row = Ai + p1; p_row != p_diag; ++ p_row) {
			const size_t j = *p_row;
			_ASSERTE(j < i);

			// the below loop could also be simplified using std::lower_bound() but the numbers
			// of iterations are most likely very small (it just "catches up" with incrementing i)

			_ASSERTE(j < n); // this array is only indexed by columns
			size_t pj1 = p_transpose_col_off[j]; // already initialized
			const size_t pj2 = Ap[j + 1];
			_ASSERTE(pj1 >= size_t(Ap[j]) && pj1 <= pj2); // make sure it points to its column
			for(; pj1 < pj2; ++ pj1) { // in case the matrix is strictly upper, then this loop will not run
				const size_t ii = Ai[pj1];
				_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
				// even in case the diagonal should be included then ii can't equal j

				if(ii < i) { // A(ii, j) is only in the lower diagonal and not in the upper (assymetric element)
#ifdef _DEBUG
					const csi *p_assym_elem = std::lower_bound(Ai + Ap[ii], Ai + Ap[ii + 1], csi(j));
					_ASSERTE(p_assym_elem == Ai + Ap[ii + 1] || *p_assym_elem != j);
					// make sure A(j, ii) does not exist
#endif // _DEBUG
					b_will_have_unsorted = true;

					//_ASSERTE(p_column_dest[j] == p_column_ptrs[j] || p_row_inds[p_column_dest[j] - 1] < ii);
					p_row_inds[p_column_dest[j]] = ii; // j < ii < i
					++ p_column_dest[j]; // A(ii, j) in the lower triangle
					//_ASSERTE(p_column_dest[ii] == p_column_ptrs[ii] || p_row_inds[p_column_dest[ii] - 1] < j);
					p_row_inds[p_column_dest[ii]] = j; // j < ii < i, j < i
					++ p_column_dest[ii]; // A(j, ii) in the upper triangle
				} else if(ii == i) { // A(ii, j) is a mirror of A(j, i), do not count
					_ASSERTE(std::lower_bound(Ai + p1, p_diag, csi(j)) == p_row); // (A(j, ii))' ~ A(i, j) is this element actually
					++ n_sym_nnz_num; // count those entries
				} else
					break; // will do the rest of this transpose row later on
			}
			p_transpose_col_off[j] = pj1; // remember where we stopped
			// ordered merge of transposed columns with the upper triangular entriex of this column

			//_ASSERTE(p_column_dest[j] == p_column_ptrs[j] || p_row_inds[p_column_dest[j] - 1] < i);
			p_row_inds[p_column_dest[j]] = i; // scatter in the lower triangle, causes unordered accesses
			++ p_column_dest[j]; // can increment, all the columns below i are already initialized
			// number of (strictly) lower triangular items A(i, *)
			// need to add this *after* to maintain sorted order
			// t_odo - see if this makes any difference at all // yes, it makes difference e.g. on Lucifora/cell1
		}
		// loop over the upper elements and count them again (or rather count their transpose images)
		// making std::lower_bound() above saves checking for the value of *p_row
		// inside this loop - fewer comparisons are made

		p1 = p2;
	}

	for(size_t j = 0; j < n; ++ j) {
		_ASSERTE(j < n); // p_transpose_col_off array is only indexed by columns
		_ASSERTE(p_transpose_col_off[j] >= size_t(Ap[j]) && p_transpose_col_off[j] <= size_t(Ap[j + 1])); // make sure it points to its column
		for(size_t pj1 = p_transpose_col_off[j], pj2 = Ap[j + 1]; pj1 < pj2; ++ pj1) {
			const size_t ii = Ai[pj1];
			_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
			// even in case the diagonal should be included then ii can't equal j

			b_will_have_unsorted = true; // t_odo - see if the matrices that failed to predict on anselm are now predicted correctly or not

#ifdef _DEBUG
			if(ii < n) { // !!
				const csi *p_assym_elem = std::lower_bound(Ai + Ap[ii], Ai + Ap[ii + 1], csi(j));
				_ASSERTE(p_assym_elem == Ai + Ap[ii + 1] || *p_assym_elem != j);
				// make sure A(j, ii) does not exist
			}
#endif // _DEBUG

			//_ASSERTE(p_column_dest[j] == p_column_ptrs[j] || p_row_inds[p_column_dest[j] - 1] < ii);
			p_row_inds[p_column_dest[j]] = ii;
			++ p_column_dest[j]; // A(ii, j) in the lower triangle
			//_ASSERTE(p_column_dest[ii] == p_column_ptrs[ii] || p_row_inds[p_column_dest[ii] - 1] < j);
			p_row_inds[p_column_dest[ii]] = j;
			++ p_column_dest[ii]; // A(j, ii) in the upper triangle
		}
	}
	// handle entries in transpose rows which were previously not matched

	if(!b_filled_from_upper)
		b_will_have_unsorted = false;
	// in case the matrix is strictly lower, then the entries are also filled nicely

	bool b_had_unsorted = false;
	for(size_t j = 0; j < n_length_num; ++ j) {
		b_had_unsorted = b_had_unsorted ||
			!CDebug::b_IsStrictlySortedSet(p_row_inds + p_column_ptrs[j],
			p_row_inds + p_column_ptrs[j + 1]);
		std::sort(p_row_inds + p_column_ptrs[j], p_row_inds + p_column_ptrs[j + 1]);
	}
	/*if(b_had_unsorted) {
		static int n_times = 0;
		++ n_times;
		fprintf(stderr, "warning: had unsorted entries (%d times already)\n", n_times); // at least see when it happens
	}*/
	//if(b_will_have_unsorted != b_had_unsorted)
	if(!b_will_have_unsorted && b_had_unsorted)
		fprintf(stderr, "error: failed to predict unsorted entries\n");
	//if(b_will_have_unsorted && !b_had_unsorted)
	//	fprintf(stderr, "warning: predicted unsorted entries but had none\n");
	// it is not trivial to generate it sorted
	// AMD indeed does not require them sorted

#ifdef _DEBUG
	for(size_t j = 0, n_cumsum = 0; j < n_length_num; ++ j) {
		_ASSERTE(p_column_ptrs[j] == n_cumsum); // points at the beginning of the column
		n_cumsum += p_column_lengths[j];
		_ASSERTE(p_column_dest[j] == n_cumsum); // points at the end of the column

		_ASSERTE(CDebug::b_IsStrictlySortedSet(p_row_inds + p_column_ptrs[j],
			p_row_inds + p_column_ptrs[j + 1])); // make sure the result is not jumbled (should always be sorted)
	}
	_ASSERTE(p_column_ptrs[n_length_num] == n_nnz_num); // ...
	// make sure all the columns are filled, according to p_column_lengths
#endif // _DEBUG
}

/*
 *								=== ~CMatrixTransposeSum ===
 */
