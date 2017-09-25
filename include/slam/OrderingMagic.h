/*
								+-----------------------------------+
								|                                   |
								| *** Matrix ordering utilities *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|          OrderingMagic.h          |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __MATRIX_ORDERING_UTILS_INCLUDED
#define __MATRIX_ORDERING_UTILS_INCLUDED

/**
 *	@file include/slam/OrderingMagic.h
 *	@brief matrix ordering utilities
 *	@author -tHE SWINe-
 *	@date 2013-02-07
 *
 *	@date 2013-05-24
 *
 *	Removed the explicitly hybrid orderings, as actually the NNZ of AAT is easy
 *	to calculate from just upper triangular of A (which is actually lambda).
 *
 *	@date 2017-04-04
 *
 *	Fixed a warning in VS 2015 using checked iterators. Thanks to a flaw in checked iterators
 *	not allowing zero-length null array the code is rather messy. Re-ran AAT unit tests on both
 *	VS 2015 and the old branch of the code (successfully).
 *
 */

/** \addtogroup ubm
 *	@{
 */

/**
 *	@def __MATRIX_ORDERING_USE_MMD
 *	@brief if defined, METIS multiple minimum degree ordering is used instead
 *		of AMD approximate minimum degree in CMatrixOrdering::p_BlockOrdering()
 */
//#define __MATRIX_ORDERING_USE_MMD

#ifdef __MATRIX_ORDERING_USE_MMD

/**
 *	@def __MATRIX_ORDERING_MMD_DELTA
 *
 *	If the value of DELTA is greater than or equal to zero, multiple elimination will
 *	be used in producing the ordering, and DELTA provides the tolerance factor as
 *	described in Section 4.2. If DELTA is -1, the subroutine will produce the conventional
 *	minimum-degree ordering (using external degree), that is, one degree update after each
 *	mass elimination. In the results tabulated in this section, we have labeled this
 *	the minimum-external-degree algorithm.
 *
 *	@brief value of the MMD DELTA parameter
 *
 *	@note This was seen using deltas of -1, 0, 1 and 5.
 *
 *	On the first 1000 edges of 10k, this yields the following
 *	sizes of R (in allocated doubles reported by the data pool):
 *		44735 = -1
 *		36779 = 1
 *		36286 = 0
 *		36813 = 5
 *		34008 = AMD
 *
 *	On venice871.g2o, this yields the following runtimes:
 *		271.281028 = -1
 *		196.979716 = 1
 *		198.957934 = 0
 *		190.020217 = 5
 *		153.331193 = AMD
 */
#define __MATRIX_ORDERING_MMD_DELTA 5

#endif // __MATRIX_ORDERING_USE_MMD

/**
 *	@def __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT
 *	@brief if enabled, two-level ordering constraint is applied to lambda
 *		in CLastElementOrderingConstraint
 */
//#define __MATRIX_ORDERING_TWO_LEVEL_CONSTRAINT

/**
 *	@def __MATRIX_ORDERING_USE_AMD1
 *	@brief if defined, CMatrixOrdering::p_DestructiveOrdering() will use
 *		amd_1() or camd_1() function, otherwise it will use amd_2() or camd_2().
 *	@note Using amd_2() or camd_2() should be slightly faster.
 */
//#define __MATRIX_ORDERING_USE_AMD1

/**
 *	@def __MATRIX_ORDERING_USE_AMD_AAT
 *	@brief if defined, CMatrixOrdering::p_DestructiveOrdering() will use
 *		amd_aat() or camd_aat() function to calculate the sparsity pattern,
 *		otherwise a custom code is used.
 *	@note Using amd_aat() or camd_aat() is slower.
 */
//#define __MATRIX_ORDERING_USE_AMD_AAT

/**
 *	@def __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES
 *	@brief Cache alignment, in bytes. Must be divisible by 8, must be power of two.
 *	@note This is only in effect if __MATRIX_ORDERING_CACHE_ALIGN is defined.
 */
#define __MATRIX_ORDERING_CACHE_ALIGNMENT_BYTES 64

#ifndef __MATRIX_ORDERING_USE_AMD1

/**
 *	@def __MATRIX_ORDERING_CACHE_ALIGN
 *	@brief If enabled, CMatrixOrdering::p_DestructiveOrdering() will align
 *		working arrays for AMD so that accessing the same element in different
 *		arrays uses a different cache line.
 *	@note This is only effective if __MATRIX_ORDERING_USE_AMD1 is not defined
 *		(then, the alignment is up to amd_1() or camd_1()).
 */
#define __MATRIX_ORDERING_CACHE_ALIGN

#endif // !__MATRIX_ORDERING_USE_AMD1

#include <vector>
#include <algorithm>
#include <numeric>
#include "slam/BlockMatrix.h"

/**
 *	@brief ordering constraint for C(COL)AMD libraries
 *
 *	This maintans a constraint vector which forces the last element
 *	to be the last even after symbolic ordering.
 */
class CLastElementOrderingConstraint {
protected:
	std::vector<size_t> m_constraints; /**< @brief storage for the constraint vector */

public:
	/**
	 *	@brief gets the constraint vector of a specified size
	 *	@param[in] n_size is the size of the constraint vector (in elements)
	 *	@return Returns const pointer to the constraint vector
	 *		of the specified size (not to be deleted).
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_Get(size_t n_size); // throw(std::bad_alloc)
};

/**
 *	@brief ordering constraint for C(COL)AMD libraries
 *
 *	This maintans a constraint vector which forces the first and the last elements
 *	to be the first and the last, respecitvely, even after symbolic ordering.
 */
class CFirstLastElementOrderingConstraint {
protected:
	std::vector<size_t> m_constraints; /**< @brief storage for the constraint vector */

public:
	/**
	 *	@brief gets the constraint vector of a specified size
	 *	@param[in] n_size is the size of the constraint vector (in elements)
	 *	@return Returns const pointer to the constraint vector
	 *		of the specified size (not to be deleted).
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_Get(size_t n_size); // throw(std::bad_alloc)
};

/**
 *	@brief ordering constraint for C(COL)AMD libraries
 *
 *	This maintans a constraint vector which forces the first and the last elements
 *	to be the first and the last, respecitvely, even after symbolic ordering.
 */
class CNFirst1LastElementOrderingConstraint {
protected:
	std::vector<size_t> m_constraints; /**< @brief storage for the constraint vector */

public:
	/**
	 *	@brief gets the constraint vector of a specified size
	 *
	 *	@param[in] n_size is the size of the constraint vector (in elements)
	 *	@param[in] n_first_constraint_num is number of the constraints of the first columns
	 *
	 *	@return Returns const pointer to the constraint vector
	 *		of the specified size (not to be deleted).
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_Get(size_t n_size, size_t n_first_constraint_num); // throw(std::bad_alloc)
};

/**
 *	@brief matrix ordering calculator (CAMD wrapper)
 */
class CMatrixOrdering { // t_odo - fill throws, and in related classes
protected:
	std::vector<size_t> m_ordering; /**< @brief storage for the blockwise ordering vector */
	std::vector<size_t> m_ordering_expand; /**< @brief storage for the elementwise ordering vector */
	std::vector<size_t> m_ordering_invert; /**< @brief storage for the inverse elementwise ordering vector */
	cs *m_p_block; /**< @brief matrix block structure (reuses memory storage) */

	std::vector<size_t> m_camd_workspace; /**< @brief workspace for camd_aat() */
	std::vector<size_t> m_camd_workspace1; /**< @brief workspace for camd_1() */

public:
	/**
	 *	@brief default constructor; has no effect
	 */
	inline CMatrixOrdering()
		:m_p_block(0)
	{}

	/**
	 *	@brief destructor; deletes allocated memory
	 */
	~CMatrixOrdering();

	/**
	 *	@brief gets the size of matrix ordering
	 *	@return Returns the size of the current matrix ordering.
	 */
	inline size_t n_Ordering_Size() const
	{
		return m_ordering.size();
	}

	/**
	 *	@brief gets the ordering
	 *	@return Returns the current matrix ordering, or 0 in case there is no ordering.
	 */
	inline const size_t *p_Get_Ordering() const
	{
		return (!m_ordering.empty())? &m_ordering[0] : 0;
	}

	/**
	 *	@brief gets the inverse ordering
	 *	@return Returns the current matrix inverse ordering,
	 *		or 0 in case there is no inverse ordering.
	 *	@note Even though non-null value is returned, the inverse ordering
	 *		can be out-of-date, as it is calculated upon request.
	 */
	inline const size_t *p_Get_InverseOrdering() const
	{
		_ASSERTE(m_ordering_invert.empty() ||
			m_ordering_invert.size() == n_Ordering_Size()); // make sure contents will match in size
		return (!m_ordering_invert.empty())? &m_ordering_invert[0] : 0;
	}

	/**
	 *	@brief calculates inverse ordering while maintaining and reusing storage
	 *
	 *	@param[in] p_ordering is an ordering to be inverted
	 *	@param[in] n_ordering_size is size of the given ordering
	 *
	 *	@return Returns const pointer to the inverse of the given ordering (not to be deleted).
	 *
	 *	@note The buffer for inverse ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple inverse orderings need to exist at the same time).
	 *	@note Due to the buffer reuse, this can not invert ordering, inverted
	 *		by the same object (calling the same function on the result of the previous
	 *		call, on the same object). This doesn't apply to the results of the other functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_InvertOrdering(const size_t *p_ordering, size_t n_ordering_size); // throw(std::bad_alloc)

	/**
	 *	@brief extremely intricate function for progressive reordering
	 *
	 *	@param[in] n_order_min is zero-based index of the first element of the ordering to be modified
	 *	@param[in] p_sub_block_ordering is pointer to incremental ordering
	 *	@param[in] n_sub_block_size is size of the incremental ordering; in case it
	 *		is longer than the current ordering, it is extended with identity
	 *
	 *	@return Returns const pointer to the extended ordering (not to be deleted).
	 *
	 *	@note This function invalidates the inverse ordering; needs to be recalculated.
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 */
	const size_t *p_ExtendBlockOrdering_with_SubOrdering(size_t n_order_min,
		const size_t *p_sub_block_ordering, size_t n_sub_block_size); // throw(std::bad_alloc)

	/**
	 *	@brief calculates blockwise ordering in a matrix, using the CAMD library
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *	@param[in] p_constraints is the ordering constraint vector (can be 0)
	 *	@param[in] n_constraints_size is size og the ordering constraint vector
	 *		(must match the number of r_A block columns, except it is ignored
	 *		if p_constraints is 0)
	 *	@param[in] b_need_inverse is inverse ordering flag (if set, inverse ordering is
	 *		calculated as well, possibly at lower cost; otherwise it is left unchanged)
	 *	@param[in] b_A_is_upper_triangular is upper-triangular flag (if set, A is expected
	 *		to be stored only in the upper triangle, otherwise both triangles may be used)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_BlockOrdering(const CUberBlockMatrix &r_A, const size_t *p_constraints,
		size_t UNUSED(n_constraints_size), bool b_need_inverse = false,
		bool b_A_is_upper_triangular = true); // throw(std::bad_alloc)

	/**
	 *	@brief calculates blockwise ordering in a matrix, using the CAMD library
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *	@param[in] n_matrix_cut is the number of block rows and columns to cut
	 *		off of the r_A matrix (from left and up)
	 *	@param[in] n_matrix_diag is the number of block rows and columns to make
	 *		diagonal in the r_A matrix (from left and up, before cutting)
	 *	@param[in] p_constraints is the ordering constraint vector (can be 0)
	 *	@param[in] n_constraints_size is size og the ordering constraint vector
	 *		(must match the number of r_A block columns, except it is ignored
	 *		if p_constraints is 0)
	 *	@param[in] b_need_inverse is inverse ordering flag (if set, inverse ordering is
	 *		calculated as well, possibly at lower cost; otherwise it is left unchanged)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_BlockOrdering_MiniSkirt(const CUberBlockMatrix &r_A, size_t n_matrix_cut,
		size_t n_matrix_diag, const size_t *p_constraints, size_t UNUSED(n_constraints_size),
		bool b_need_inverse = false); // throw(std::bad_alloc)

	/**
	 *	@brief calculates blockwise ordering in a matrix, using the AMD library
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *	@param[in] b_need_inverse is inverse ordering flag (if set, inverse ordering is
	 *		calculated as well, possibly at lower cost; otherwise it is left unchanged)
	 *	@param[in] b_A_is_upper_triangular is upper-triangular flag (if set, A is expected
	 *		to be stored only in the upper triangle, otherwise both triangles may be used)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_BlockOrdering(const CUberBlockMatrix &r_A, bool b_need_inverse = false,
		bool b_A_is_upper_triangular = true); // throw(std::bad_alloc)

#if 0
	/**
	 *	@brief calculates blockwise ordering in a matrix, using the CAMD library
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *	@param[in] n_off_diagonal_num is the number of off-diagonal elements
	 *		(equals number of edges in binary graphs or the sum of edge ranks
	 *		minus the number of vertices in hypergraphs)
	 *	@param[in] p_constraints is the ordering constraint vector (can be 0)
	 *	@param[in] n_constraints_size is size og the ordering constraint vector
	 *		(must match the number of r_A block columns, except it is ignored
	 *		if p_constraints is 0)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_HybridBlockOrdering(const CUberBlockMatrix &r_A, size_t n_off_diagonal_num,
		const size_t *p_constraints, size_t UNUSED(n_constraints_size)); // throw(std::bad_alloc)
#endif // 0

	/**
	 *	@brief calculates elementwise ordering by expanding blockwise ordering on a block matrix
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *	@param[in] b_inverse is inverse ordering flag which chooses the source
	 *		ordering to expand; if set, it is the last ordering produced by a call
	 *		to p_InvertOrdering(), if not set it is the last ordering produced
	 *		by a call to p_BlockOrdering().
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The b_inverse flag must not be set (there is a bug in expanding inverse ordering directly).
	 *	@note The buffer for elementwise ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple elementwise orderings need to exist at the same time).
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_ExpandBlockOrdering(const CUberBlockMatrix &r_A, bool b_inverse); // throw(std::bad_alloc)

	/**
	 *	@brief calculates ordering for an elementwise sparse matrix using CAMD
	 *
	 *	@param[in] p_A is the sparse matrix (must be symmetric)
	 *	@param[in] p_constraints is the ordering constraint vector (can be 0)
	 *	@param[in] n_constraints_size is size og the ordering constraint vector
	 *		(must match the number of p_A columns, except it is ignored
	 *		if p_constraints is 0)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note In case the matrix is no longer needed, it is better to call
	 *		p_DestructiveOrdering() as this function needs to make a copy of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_Ordering(const cs *p_A, const size_t *p_constraints,
		size_t UNUSED(n_constraints_size)); // throw(std::bad_alloc)

	/**
	 *	@brief calculates ordering for an elementwise sparse matrix using AMD
	 *
	 *	@param[in] p_A is the sparse matrix (must be symmetric)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note In case the matrix is no longer needed, it is better to call
	 *		p_DestructiveOrdering() as this function needs to make a copy of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_Ordering(const cs *p_A); // throw(std::bad_alloc)

	/**
	 *	@brief calculates ordering for an elementwise sparse matrix using CAMD
	 *
	 *	@param[in] p_A is the sparse matrix (must be symmetric, the contents will
	 *		be dammaged by this call)
	 *	@param[in] p_constraints is the ordering constraint vector (can be 0)
	 *	@param[in] n_constraints_size is size og the ordering constraint vector
	 *		(must match the number of p_A columns, except it is ignored
	 *		if p_constraints is 0)
	 *	@param[in] b_need_inverse is inverse ordering flag (if set, inverse ordering is
	 *		calculated as well, possibly at lower cost; otherwise it is left unchanged)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note In case the matrix is needed to remain untouched, it is needed to call
	 *		p_Ordering() as this function dammages the contents of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_DestructiveOrdering(cs *p_A, const size_t *p_constraints,
		size_t UNUSED(n_constraints_size), bool b_need_inverse = false); // throw(std::bad_alloc)

	/**
	 *	@brief calculates ordering for an elementwise sparse matrix using AMD
	 *
	 *	@param[in] p_A is the sparse matrix (must be symmetric, the contents will
	 *		be dammaged by this call)
	 *	@param[in] b_need_inverse is inverse ordering flag (if set, inverse ordering is
	 *		calculated as well, possibly at lower cost; otherwise it is left unchanged)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note In case the matrix is needed to remain untouched, it is needed to call
	 *		p_Ordering() as this function dammages the contents of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_DestructiveOrdering(cs *p_A, bool b_need_inverse = false); // throw(std::bad_alloc)

#if 0
	/**
	 *	@brief calculates ordering for an elementwise sparse matrix using CAMD
	 *
	 *	@param[in] p_A is the sparse matrix (must be symmetric, the contents will
	 *		be dammaged by this call)
	 *	@param[in] n_off_diagonal_num is the number of off-diagonal elements
	 *		(equals number of edges in binary graphs or the sum of edge ranks
	 *		minus the number of vertices in hypergraphs)
	 *	@param[in] p_constraints is the ordering constraint vector (can be 0)
	 *	@param[in] n_constraints_size is size og the ordering constraint vector
	 *		(must match the number of p_A columns, except it is ignored
	 *		if p_constraints is 0)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note In case the matrix is needed to remain untouched, it is needed to call
	 *		p_Ordering() as this function dammages the contents of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_HybridDestructiveOrdering(cs *p_A, size_t n_off_diagonal_num,
		const size_t *p_constraints, size_t UNUSED(n_constraints_size)); // throw(std::bad_alloc)
#endif // 0

	/**
	 *	@brief extends an existing block ordering with identity ordering
	 *
	 *	@param[in] n_new_size is required size of the new ordering (must be greater than old size)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_ExtendBlockOrdering_with_Identity(size_t n_new_size); // throw(std::bad_alloc)

	/**
	 *	@brief checks if a given permutation is a valid ordering
	 *
	 *	A valid ordering is a permutation of a sequence of integers, starting
	 *	with zero and going to size - 1. Every index therefore occurs exactly
	 *	once, and there are no out of range indices (and therefore no index is
	 *	missing).
	 *
	 *	@param[in] p_order is the permutation array
	 *	@param[in] n_size is size of the permutation (in elements)
	 *
	 *	@return Returns true if the ordering is valid, otherwise returns false.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static bool b_IsValidOrdering(const size_t *p_order, size_t n_size); // throw(std::bad_alloc)

	/**
	 *	@brief checks if a given permutation is an identity permutation
	 *
	 *	A valid ordering is a permutation of a sequence of integers, starting
	 *	with zero and going to size - 1. Every index therefore occurs exactly
	 *	once, and there are no out of range indices (and therefore no index is
	 *	missing).
	 *
	 *	@param[in] p_order is the permutation array
	 *	@param[in] n_size is size of the permutation (in elements)
	 *
	 *	@return Returns true if the ordering is valid and indetity, otherwise returns false.
	 */
	static bool b_IsIdentityOrdering(const size_t *p_order, size_t n_size);

	/**
	 *	@brief calculates adjacent label ordering
	 *
	 *	Given a set of vertex labels, this permutation orders vertices with the
	 *	same label to be adjacent while maintaining the relative ordering among
	 *	the vertices with the same label.
	 *
	 *	@param[in] r_vertex_label is a list of labels, one for each vertex (zero-based, contiguous ids)
	 *	@param[in] n_label_num is number of labels in r_vertex_label (number of different values rather
	 *		than the size of the vector)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_AdjacentLabel_Ordering(const std::vector<size_t> &r_vertex_label,
		size_t n_label_num); // throw(std::bad_alloc)

	/**
	 *	@brief calculates adjacent label ordering and recovers index of the first vertex in each label set
	 *
	 *	Given a set of vertex labels, this permutation orders vertices with the
	 *	same label to be adjacent while maintaining the relative ordering among
	 *	the vertices with the same label.
	 *
	 *	Consider the following:
	 *	@code
	 *	CUberBlockMatrix A = ...;
	 *
	 *	std::vector<size_t> subgraph_labels;
	 *	size_t n_subgraph_num =
	 *		CMatrixOrdering::n_Find_BlockStructure_Subgraphs(subgraph_labels, A);
	 *
	 *	std::vector<size_t> first_vertex;
	 *	CMatrixOrdering mord;
	 *	const size_t *p_ord = mord.p_AdjacentLabel_Ordering(first_vertex, subgraph_labels, n_subgraph_num);
	 *	// get adjacent label ordering
	 *
	 *	// note thet p_ord is applied to the matrix without being inverted
	 *
	 *	CUberBlockMatrix A_perm;
	 *	A.Permute_UpperTriangular_To(A_perm, p_ord, mord.n_Ordering_Size(), true); // shallow copy if possible
	 *	// A_perm will have the subgraphs separated. block columns first_vertex[i] through
	 *
	 *	first_vertex.push_Back(A.n_BlockColumn_Num());
	 *	// (first_vertex[i + 1] - 1) (inclusive) of A_perm will have all the vertices that
	 *	// belong to subgrapg subgraph_labels[i]. in case this subgraph is full (a clique),
	 *	// these blocks will form a square centered on the diagonal for 0 <= i < n_subgraph_num
	 *	@endcode
	 *
	 *	@param[out] r_first_vertex is filled with indices of the first vertex of each label,
	 *		in the reordered sequence (will contain n_label_num numbers)
	 *	@param[in] r_vertex_label is a list of labels, one for each vertex (zero-based, contiguous ids)
	 *	@param[in] n_label_num is number of labels in r_vertex_label (number of different values rather
	 *		than the size of the vector)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@todo This is mostly untested; test this.
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of CMatrixOrdering
	 *		if multiple block orderings need to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering(),
	 *		p_ExtendBlockOrdering_with_Identity() and p_AdjacentLabel_Ordering().
	 *		A call to any of those functions invalidates previous results of all
	 *		of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	const size_t *p_AdjacentLabel_Ordering(std::vector<size_t> &r_first_vertex,
		const std::vector<size_t> &r_vertex_label, size_t n_label_num); // throw(std::bad_alloc)

	/**
	 *	@brief finds subgraphs (connected components) in a sparse matrix
	 *
	 *	The list of subgraph ids is arbitrary, in a way that the ids can be assigned to the
	 *	subgraphs in many different ways. The list of ids is canonical in case when read
	 *	sequentially, each new id equals the number of ids spotted so far. So for example:
	 *
	 *	@code
	 *	{0, 1, 2, 3, 4, 5} // canonical
	 *	{0, 0, 0, 0, 0, 0} // canonical
	 *	{0, 1, 1, 0, 1, 2} // canonical
	 *	{0, 2, 2, 0, 2, 1} // not canonical (2 appears before 1; same labelling as the above)
	 *	@endcode
	 *
	 *	In case full matrices are processed, the output subgraph ids are always canonical.
	 *	In case upper-triangular matrices are processed (default), the subgraph ids are only
	 *	canonical in case the number of subgraphs equals the number of vertices (each vertex
	 *	forms its own subgraph) or in case the graph is fully connected (all vertices belong
	 *	to subgraph 0).
	 *
	 *	@param[out] r_vertex_subgraph is a list of (zero-based) vertex subgraphs affiliations
	 *	@param[in] r_A is block matrix which gives the graph by its block structure,
	 *		must be square, symmetric and upper-triangular (the lower triangle is ignored,
	 *		unless b_is_full_matrix is set)
	 *	@param[in] b_is_full_matrix is full matrix flag (if set, r_A must be square, symmetric
	 *		and both upper and lower triangles are stored; default cleared)
	 *
	 *	@return Returns the number of subgraphs in r_A.
	 *
	 *	@note The list of subgraphs is not an ordering! Use \ref p_AdjacentLabel_Ordering()
	 *		and \ref p_InvertOrdering() to convert this to an ordering.
	 *	@note This function throws std::bad_alloc.
	 */
	static size_t n_Find_BlockStructure_Subgraphs(std::vector<size_t> &r_vertex_subgraph,
		const CUberBlockMatrix &r_A, bool b_is_full_matrix = false); // throw(std::bad_alloc)
};

/**
 *	@brief constrained matrix ordering calculator (CAMD wrapper)
 *
 *	This is a convenience wrapper for CLastElementOrderingConstraint and CMatrixOrdering,
 *	calculating the constraint automatically for the caller. The constraint forces the
 *	last matrix block to still be the last, even after the ordering.
 */
class CLastElementConstrainedMatrixOrdering {
protected:
	CLastElementOrderingConstraint m_c; /**< @brief ordering constriant object */
	CMatrixOrdering m_o; /**< @brief matrix ordering (CAMD wrapper) */

public:
	/**
	 *	@brief calculates blockwise ordering in a matrix, using the CAMD library
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of
	 *		CLastElementConstrainedMatrixOrdering if multiple block orderings need
	 *		to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering()
	 *		and p_ExtendBlockOrdering_with_Identity(). A call to any of those
	 *		functions invalidates previous results of all of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	inline const size_t *p_BlockOrdering(const CUberBlockMatrix &r_A) // throw(std::bad_alloc)
	{
		size_t n_block_num = r_A.n_BlockColumn_Num();
		return m_o.p_BlockOrdering(r_A, m_c.p_Get(n_block_num), n_block_num);
	}

#if 0
	/**
	 *	@brief calculates blockwise ordering in a matrix, using the CAMD library
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *	@param[in] n_off_diagonal_num is the number of off-diagonal elements
	 *		(equals number of edges in binary graphs or the sum of edge ranks
	 *		minus the number of vertices in hypergraphs)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of
	 *		CLastElementConstrainedMatrixOrdering if multiple block orderings need
	 *		to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering()
	 *		and p_ExtendBlockOrdering_with_Identity(). A call to any of those
	 *		functions invalidates previous results of all of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	inline const size_t *p_HybridBlockOrdering(const CUberBlockMatrix &r_A, size_t n_off_diagonal_num) // throw(std::bad_alloc)
	{
		size_t n_block_num = r_A.n_BlockColumn_Num();
		return m_o.p_HybridBlockOrdering(r_A, n_off_diagonal_num, m_c.p_Get(n_block_num), n_block_num);
	}
#endif // 0

	/**
	 *	@brief extends an existing block ordering with identity ordering
	 *
	 *	@param[in] n_new_size is required size of the new ordering (must be greater than old size)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for block ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of
	 *		CLastElementConstrainedMatrixOrdering if multiple block orderings need
	 *		to exist at the same time).
	 *	@note The same buffer is used by p_BlockOrdering(), p_HybridBlockOrdering,
	 *		p_Ordering(), p_DestructiveOrdering() p_HybridDestructiveOrdering()
	 *		and p_ExtendBlockOrdering_with_Identity(). A call to any of those
	 *		functions invalidates previous results of all of those functions.
	 *	@note This function throws std::bad_alloc.
	 */
	inline const size_t *p_ExtendBlockOrdering_with_Identity(size_t n_new_size) // throw(std::bad_alloc)
	{
		return m_o.p_ExtendBlockOrdering_with_Identity(n_new_size);
	}

	/**
	 *	@brief calculates elementwise ordering by expanding blockwise ordering on a block matrix
	 *
	 *	This takes blockwise ordering calculated by the last call to p_BlockOrdering()
	 *	or to p_ExtendBlockOrdering_with_Identity() as input. Calling this function before
	 *	calling one of those is not permitted.
	 *
	 *	@param[in] r_A is the block matrix (must be symmetric)
	 *
	 *	@return Returns const pointer to the ordering vector (not to be deleted).
	 *
	 *	@note The buffer for elementwise ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of
	 *		CLastElementConstrainedMatrixOrdering if multiple block orderings need
	 *		to exist at the same time).
	 *	@note This function throws std::bad_alloc.
	 */
	inline const size_t *p_ExpandBlockOrdering(const CUberBlockMatrix &r_A) // throw(std::bad_alloc)
	{
		return m_o.p_ExpandBlockOrdering(r_A, false);
	}

	/**
	 *	@brief calculates inverse ordering while maintaining and reusing storage
	 *
	 *	@param[in] p_ordering is an ordering to be inverted
	 *	@param[in] n_ordering_size is size of the given ordering
	 *
	 *	@return Returns const pointer to the inverse of the given ordering (not to be deleted).
	 *
	 *	@note The buffer for inverse ordering is reused in the next function call,
	 *		invalidating the previous result (create more instances of
	 *		CLastElementConstrainedMatrixOrdering if multiple block orderings need
	 *		to exist at the same time).
	 *	@note Due to the buffer reuse, this can not invert ordering, inverted
	 *		by the same object (calling the same function on the result of the previous
	 *		call, on the same object). This doesn't apply to the results of the other functions.
	 *	@note This function throws std::bad_alloc.
	 */
	inline const size_t *p_InvertOrdering(const size_t *p_ordering, size_t n_ordering_size) // throw(std::bad_alloc)
	{
		return m_o.p_InvertOrdering(p_ordering, n_ordering_size);
	}
};

/**
 *	@brief algorithms for calculating matrix transpose and its sum (\f$A+A^T\f$)
 *
 *	This is useful in matrix ordering algorithms where an undirected graph is required.
 *	When represented using the compressed sparse column structure, both upper and lower
 *	halves are needed for efficient lookup.
 */
class CMatrixTransposeSum {
public:
	/**
	 *	@brief calculates numbers of entries in each column in \f$A+A^T\f$
	 *		(reference version, slow but works always, with any matrices)
	 *
	 *	@param[out] p_column_lengths is pointer to the array to store the numbers of nonzeros
	 *		in each column of \f$A+A^T\f$ (must be allocated to the greater dimension of p_matrix)
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the greater
	 *		dimension of p_matrix)
	 *	@param[in] p_matrix is input matrix, need not be square
	 *	@param[in] b_AAT_with_diagonal is diagonal output flag (if set, the diagonal of p_matrix
	 *		will be present in the output; cleared by default)
	 *
	 *	@return Returns the number of nonzero elements in \f$A+A^T\f$, possibly without
	 *		the diagonal based on the last argument.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static size_t n_ColumnLengths_AAT_Ref(size_t *p_column_lengths,
		size_t n_length_num, const cs *p_matrix, bool b_AAT_with_diagonal = false); // throw(std::bad_alloc)

	/**
	 *	@brief calculates numbers of entries in each column in \f$A+A^T\f$ without the diagonal
	 *
	 *	@param[out] p_column_lengths is pointer to the array to store the numbers of nonzeros
	 *		in each column of \f$A+A^T\f$ (must be allocated to the number of columns of p_matrix)
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the number
	 *		of columns of p_matrix)
	 *	@param[in] p_matrix is input matrix, must be square upper-triangular with full
	 *		diagonal, the column entries must to be sorted by row
	 *
	 *	@return Returns the number of nonzero elements in \f$A+A^T\f$, without the diagonal.
	 */
	static size_t n_ColumnLengths_AATNoDiag_UpperFullDiagonal(size_t *p_column_lengths,
		size_t n_length_num, const cs *p_matrix);

	/**
	 *	@brief calculates numbers of entries in each column in \f$A+A^T\f$, without the diagonal
	 *
	 *	@param[out] p_column_lengths is pointer to the array to store the numbers of nonzeros
	 *		in each column of \f$A+A^T\f$ (must be allocated to the greater dimension of p_matrix)
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the greater
	 *		dimension of p_matrix)
	 *	@param[in] p_matrix is the input matrix, the column entries must to be sorted by row
	 *	@param[out] p_workspace is pointer to a temporary array (must be allocated at least to the
	 *		number of columns of p_matrix; may not reuse the same array as p_column_lengths)
	 *	@param[in] n_workspace_size is size of the p_workspace array (must be at least the number
	 *		of columns of p_matrix)
	 *
	 *	@return Returns the number of nonzero elements in \f$A+A^T\f$, without the diagonal.
	 */
	static size_t n_ColumnLengths_AATNoDiag(size_t *p_column_lengths, size_t n_length_num,
		const cs *p_matrix, size_t *p_workspace, size_t n_workspace_size);

	/**
	 *	@brief calculates numbers of entries in each column in \f$A+A^T\f$, without the diagonal
	 *
	 *	@tparam b_upper_triangular is upper-triangular input flag (the presence of the diagonal
	 *		does not matter either way; the function will fail if the input is not upper triangular)
	 *	@tparam b_likely_upper_triangular is likely upper-triangular input flag (mutually exclusive
	 *		with b_upper_triangular; the function will be slower but will not fail on non-upper input)
	 *	@tparam b_output_diagonal is diagonal output flag (if set, the block diagonal structure of
	 *		this matrix will be present in the output; otherwise only the off-diagonal blocks are processed)
	 *	@tparam b_output_full_diagonal is diagonal output flag (mutually exclusive with
	 *		b_output_diagonal; if set, the output will always contain a full diagonal spanning the
	 *		entire \f$A+A^T\f$)
	 *	@tparam CInt is integer data type for the column lengths array (can be signed or unsigned)
	 *	@tparam CInt1 is integer data type for the workspace array (can be signed or unsigned)
	 *
	 *	@param[out] p_column_lengths is pointer to the array to store the numbers of nonzeros
	 *		in each column of \f$A+A^T\f$ (must be allocated to the greater dimension of p_matrix)
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the greater
	 *		dimension of p_matrix)
	 *	@param[in] p_matrix is the input matrix, the column entries must to be sorted by row
	 *	@param[out] p_workspace is pointer to a temporary array (must be allocated at least to the
	 *		number of columns of p_matrix; may not reuse the same array as p_column_lengths)
	 *	@param[in] n_workspace_size is size of the p_workspace array (must be at least the number
	 *		of columns of p_matrix)
	 *
	 *	@return Returns the number of nonzero elements in \f$A+A^T\f$, possibly with the diagonal
	 *		omitted (or extended) based on the template arguments.
	 */
	template <bool b_upper_triangular, bool b_likely_upper_triangular,
		bool b_output_diagonal, bool b_output_full_diagonal, class CInt, class CInt1>
	static size_t n_ColumnLengths_AAT(CInt *__restrict p_column_lengths, size_t n_length_num,
		const cs *__restrict p_matrix, CInt1 *__restrict p_workspace, size_t n_workspace_size) // calculates numbers of entries in each column in A+A^T without the diagonal, taking an upper-triangular matrix as input. requires the column entries to be sorted by row.
	{
		_ASSERTE(!b_output_diagonal || !b_output_full_diagonal); // only one of those
		_ASSERTE(!b_upper_triangular || !b_likely_upper_triangular); // only one of those
		// todo - compile time asserts

		_ASSERTE(n_length_num == std::max(p_matrix->m, p_matrix->n)); // must match
		_ASSERTE(b_upper_triangular || n_workspace_size >= size_t(p_matrix->n)); // must fit if not upper triangular (then it is not needed)

		const csi *__restrict Ap = p_matrix->p, *__restrict Ai = p_matrix->i;
		const size_t n = p_matrix->n, nz = Ap[n];

		//std::fill(p_column_lengths, p_column_lengths + n_length_num, size_t(0)); // zero the lengths
		//std::fill(p_workspace, p_workspace + n_workspace_size, size_t(0)); // zero the workspace
		// no need to zero anything

#ifdef _DEBUG
		std::fill(p_column_lengths, p_column_lengths + n_length_num, CInt(0xbaadf00d));
		std::fill(p_workspace, p_workspace + n_workspace_size, CInt1(0xbaadf00d));
		// fill the output and temp buffers with rubbish
#endif // _DEBUG

		CInt1 *__restrict p_transpose_col_off = (b_upper_triangular)? 0 : p_workspace; // unused if upper triangular / rename
		// those point to the Ai array

		size_t n_diag_nnz_num = 0, n_sym_nnz_num = 0;
		// the number of elements on the diagonal and (half) the number of elements
		// that have (structural) counterparts on the opposite side of the diagonal

		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			const size_t p2 = Ap[i + 1]; // don't modify, will become the next p1
			_ASSERTE(CDebug::b_IsStrictlySortedSet(Ai + p1, Ai + p2)); // must be sorted
			const csi *__restrict p_diag;
			if(b_upper_triangular) { // compile-time-constant
				_ASSERTE(p1 <= p2);
				if(p1 != p2 && size_t(Ai[p2 - 1]) == i) {
					p_diag = Ai + (p2 - 1);
					++ n_diag_nnz_num;
				} else
					p_diag = Ai + p2;
				_ASSERTE(p1 == p2 || p_diag == Ai + p2 || *p_diag == i); // empty or not empty and points at the end or not empty and points before the end, at the diagonal element
				_ASSERTE(p1 == p2 || p_diag < Ai + p2 || *(p_diag - 1) < csi(i)); // empty or not empty and points before the end or not empty and points at the end and the one before is above-diagonal
				// note that p_transpose_col_off[i] is intentionally not used, the workspace will not be needed
			} else {
				p_diag = (b_likely_upper_triangular && p1 != p2 && Ai[p2 - 1] <= csi(i))?
					((Ai[p2 - 1] == csi(i))? Ai + (p2 - 1) : Ai + p2) : // it is one of the last two ones, depending whether the diagonal item is present or not
					std::lower_bound(Ai + p1, Ai + p2, csi(i)); // find the diagonal // in case the matrix is strictly upper, then this is a slight waste of time
				_ASSERTE(!b_likely_upper_triangular || p_diag == std::lower_bound(Ai + p1, Ai + p2, csi(i))); // make sure 
				if(p_diag != Ai + p2 && size_t(*p_diag) == i) { // in case there is a diagonal element
					++ n_diag_nnz_num;
					p_transpose_col_off[i] = CInt1(p_diag - (Ai /*+ p1*/) + 1); // point to the first below-diagonal element
				} else {
					_ASSERTE(p_diag == Ai + p2 || size_t(*p_diag) > i); // otherwise it is the first below-diagonal element
					p_transpose_col_off[i] = CInt1(p_diag - (Ai /*+ p1*/));
				}
				_ASSERTE(p_transpose_col_off[i] >= 0 && p_transpose_col_off[i] <= SIZE_MAX); // make sure it is ok to cast to size_t
				_ASSERTE(size_t(p_transpose_col_off[i]) >= p1 && size_t(p_transpose_col_off[i]) <= p2); // make sure it points to this column
				_ASSERTE(p_transpose_col_off[i] == p2 || size_t(Ai[p_transpose_col_off[i]]) > i); // make sure it points to a below-diagonal element
				_ASSERTE(p1 == p2 || p_transpose_col_off[i] != p2 || size_t(Ai[p_transpose_col_off[i] - 1]) <= i); // make sure it is preceded by above-or-diagonal elements
			}
			// find the upper triangular portion of this column

			p_column_lengths[i] = CInt((b_output_full_diagonal)? (p_diag - (Ai + p1)) + 1 : // the number of above-diag elements + 1
				(b_output_diagonal)? // the number of elements in the upper part of the column, including the diagonal if present
					((b_upper_triangular)? (p_diag - (Ai + p1)) + ((p_diag != Ai + p2 && size_t(*p_diag) == i)? 1 : 0) : // simplified before, have to calculate it now
					p_transpose_col_off[i] - p1) : // the above else branch already calculated it
				(p_diag - (Ai + p1))); // the number of above-diag elements
			// number of (strictly) upper triangular items A(*, i)
			// in case the diagonal should be included in the column length, just use p_transpose_col_off[i] - p1
			// the diagonal entry would be added here

			for(const csi *__restrict p_row = Ai + p1; p_row != p_diag; ++ p_row) {
				const size_t j = *p_row;

				if(!b_upper_triangular) { // compile-time-constant
					_ASSERTE(j < n); // this array is only indexed by columns
					size_t pj1 = size_t(p_transpose_col_off[j]); // already initialized
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
					p_transpose_col_off[j] = CInt1(pj1); // remember where we stopped
					// ordered merge of transposed columns with the upper triangular entriex of this column
				}

				_ASSERTE(j < i);
				++ p_column_lengths[j]; // can increment, all the columns below i are already initialized
				// number of (strictly) lower triangular items A(i, *)
			}
			// loop over the upper elements and count them again (or rather count their transpose images)
			// making std::lower_bound() above saves checking for the value of *p_row
			// inside this loop - fewer comparisons are made

			p1 = p2;
		}

		std::fill(p_column_lengths + n, p_column_lengths + n_length_num, CInt(0));
		// in case the matrix is square, need to zero part of the column length vector
		// the above loop does not write below n

		if(!b_upper_triangular) { // compile-time-constant
			for(size_t j = 0; j < n; ++ j) {
				_ASSERTE(j < n); // p_transpose_col_off array is only indexed by columns
				_ASSERTE(p_transpose_col_off[j] >= 0 && p_transpose_col_off[j] <= SIZE_MAX); // make sure it is ok to cast to size_t
				_ASSERTE(size_t(p_transpose_col_off[j]) >= size_t(Ap[j]) &&
					size_t(p_transpose_col_off[j]) <= size_t(Ap[j + 1])); // make sure it points to its column
				for(size_t pj1 = size_t(p_transpose_col_off[j]), pj2 = Ap[j + 1]; pj1 < pj2; ++ pj1) {
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
		}

		if(b_output_full_diagonal && p_matrix->m > p_matrix->n) {
			for(size_t j = n, m = size_t(p_matrix->m); j < m; ++ j)
				++ p_column_lengths[j]; // the diagonal
		}
		// in case the matrix is rectangular and there are fewer columns than rows,
		// the full diagonals are not counted correctly

		_ASSERTE(nz >= n_diag_nnz_num && nz - n_diag_nnz_num >= n_sym_nnz_num); // make sure the below line does not overflow
		size_t n_nnz_num = (nz - n_diag_nnz_num - n_sym_nnz_num) * 2 +
			((b_output_diagonal)? n_diag_nnz_num : // in case the diagonal should be included, add n_diag_nnz_num back
			(b_output_full_diagonal)? std::max(p_matrix->m, p_matrix->n) : 0); // in case b_output_full_diagonal is set, add the length of the full diagonal back
		// calculate the number of nonzeros

		_ASSERTE(std::accumulate(p_column_lengths, p_column_lengths + n_length_num, size_t(0)) == n_nnz_num);
		// make sure that the sum of column lengths indeed gives nnz

		return n_nnz_num;
	}

	/**
	 *	@brief calculates binary pattern of \f$A+A^T\f$
	 *
	 *	@param[out] p_column_ptrs is pointer to the array to be filled with compressed column pointers
	 *		(must be allocated to one plus the greater dimension of p_matrix)
	 *	@param[in] n_column_ptr_num is size of the p_column_ptrs array (must equal one plus the greater
	 *		dimension of p_matrix)
	 *	@param[out] p_row_inds is pointer to the array to be filled with row indices of nonzero entries
	 *		in \f$A+A^T\f$ (must be allocated to the total number of nonzeros, as computed e.g. by using
	 *		\ref n_ColumnLengths_AAT_Ref())
	 *	@param[in] n_nnz_num is size of the p_row_inds array (must match the number of nonzeros of \f$A+A^T\f$,
	 *		as returned by \ref n_ColumnLengths_AAT_Ref(), which also equals the sum of all p_column_lengths elements)
	 *	@param[in] p_column_lengths is pointer to the array with the numbers of nonzeros in each
	 *		column of \f$A+A^T\f$ (as computed e.g. using \ref n_ColumnLengths_AAT_Ref())
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the number
	 *		of columns of p_matrix)
	 *	@param[in] p_matrix is input matrix, need not be square
	 *	@param[in] b_AAT_with_diagonal is diagonal output flag (if set, the diagonal of p_matrix
	 *		will be present in the output; cleared by default)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void AAT_Ref(size_t *p_column_ptrs, size_t n_column_ptr_num,
		size_t *p_row_inds, size_t n_nnz_num, const size_t *p_column_lengths, size_t n_length_num,
		const cs *p_matrix, bool b_AAT_with_diagonal = false); // throw(std::bad_alloc)

	/**
	 *	@brief calculates binary pattern of \f$A+A^T\f$, omitting the diagonal
	 *
	 *	@param[out] p_column_ptrs is pointer to the array to be filled with compressed column pointers
	 *		(must be allocated to one plus the number of columns of p_matrix)
	 *	@param[in] n_column_ptr_num is size of the p_column_ptrs array (must equal one plus the number
	 *		of columns of p_matrix)
	 *	@param[out] p_row_inds is pointer to the array to be filled with row indices of nonzero entries
	 *		in \f$A+A^T\f$ (must be allocated to the total number of nonzeros, as computed e.g. by using
	 *		\ref n_ColumnLengths_AATNoDiag_UpperFullDiagonal())
	 *	@param[in] n_nnz_num is size of the p_row_inds array (must match the number of nonzeros of \f$A+A^T\f$,
	 *		as returned by \ref n_ColumnLengths_AATNoDiag_UpperFullDiagonal(), which also equals the sum of
	 *		all p_column_lengths elements)
	 *	@param[in] p_column_lengths is pointer to the array with the numbers of nonzeros in each
	 *		column of \f$A+A^T\f$ (as computed e.g. using \ref n_ColumnLengths_AATNoDiag_UpperFullDiagonal())
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the number
	 *		of columns of p_matrix)
	 *	@param[in] p_matrix is input matrix, must be square upper-triangular with full
	 *		diagonal, the column entries must to be sorted by row
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void AATNoDiag_UpperFullDiagonal(size_t *p_column_ptrs, size_t n_column_ptr_num,
		size_t *p_row_inds, size_t n_nnz_num, const size_t *p_column_lengths,
		size_t n_length_num, const cs *p_matrix);

	/**
	 *	@brief calculates binary pattern of \f$A+A^T\f$, omittinmg the diagonal
	 *
	 *	@param[out] p_column_ptrs is pointer to the array to be filled with compressed column pointers
	 *		(must be allocated to one plus the greater dimension of p_matrix)
	 *	@param[in] n_column_ptr_num is size of the p_column_ptrs array (must equal one plus the greater
	 *		dimension of p_matrix)
	 *	@param[out] p_row_inds is pointer to the array to be filled with row indices of nonzero entries
	 *		in \f$A+A^T\f$ (must be allocated to the total number of nonzeros, as computed e.g. by using
	 *		\ref n_ColumnLengths_AATNoDiag())
	 *	@param[in] n_nnz_num is size of the p_row_inds array (must match the number of nonzeros of \f$A+A^T\f$,
	 *		as returned by \ref n_ColumnLengths_AATNoDiag(), which also equals the sum of all p_column_lengths elements)
	 *	@param[in] p_column_lengths is pointer to the array with the numbers of nonzeros in each
	 *		column of \f$A+A^T\f$ (as computed e.g. using \ref n_ColumnLengths_AATNoDiag())
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the greater dimension
	 *		of p_matrix)
	 *	@param[in] p_matrix is the input matrix, the column entries must to be sorted by row
	 *	@param[out] p_workspace is pointer to a temporary array (must be allocated at least to the
	 *		number of columns of p_matrix; may not reuse the same array as p_column_lengths)
	 *	@param[in] n_workspace_size is size of the p_workspace array (must be at least the number
	 *		of columns of p_matrix)
	 */
	static void AATNoDiag(size_t *p_column_ptrs, size_t n_column_ptr_num,
		size_t *p_row_inds, size_t n_nnz_num, const size_t *p_column_lengths, size_t n_length_num,
		const cs *p_matrix, size_t *p_workspace, size_t n_workspace_size);

	/**
	 *	@brief calculates binary pattern of \f$A+A^T\f$, possibly omittinmg (or extending) the diagonal
	 *		based on the template arguments
	 *
	 *	@tparam b_upper_triangular is upper-triangular input flag (the presence of the diagonal
	 *		does not matter either way; the function will fail if the input is not upper triangular)
	 *	@tparam b_likely_upper_triangular is likely upper-triangular input flag (mutually exclusive
	 *		with b_upper_triangular; the function will be slower but will not fail on non-upper input)
	 *	@tparam b_output_diagonal is diagonal output flag (if set, the block diagonal structure of
	 *		this matrix will be present in the output; otherwise only the off-diagonal blocks are processed)
	 *	@tparam b_output_full_diagonal is diagonal output flag (mutually exclusive with
	 *		b_output_diagonal; if set, the output will always contain a full diagonal spanning the
	 *		entire \f$A+A^T\f$)
	 *	@tparam b_need_sorted_items is sorted output flag (if set, the row indices will be strictly
	 *		ordered in each column; if not set, they may come in arbitrary order unless the input
	 *		matrix is either symmetric or triangular; note that e.g. AMD does not require sorted rows)
	 *	@tparam CInt is integer data type for the compressed column pointers array (can be signed or unsigned)
	 *	@tparam CInt1 is integer data type for the column lengths array (can be signed or unsigned)
	 *	@tparam CInt2 is integer data type for the workspace array (can be signed or unsigned)
	 *
	 *	@param[out] p_column_ptrs is pointer to the array to be filled with compressed column pointers
	 *		(must be allocated to one plus the greater dimension of p_matrix)
	 *	@param[in] n_column_ptr_num is size of the p_column_ptrs array (must equal one plus the greater
	 *		dimension of p_matrix)
	 *	@param[out] p_row_inds is pointer to the array to be filled with row indices of nonzero entries
	 *		in \f$A+A^T\f$ (must be allocated to the total number of nonzeros, as computed e.g. by using
	 *		\ref n_ColumnLengths_AATNoDiag())
	 *	@param[in] n_nnz_num is size of the p_row_inds array (must match the number of nonzeros of \f$A+A^T\f$,
	 *		as returned by \ref n_ColumnLengths_AATNoDiag(), which also equals the sum of all p_column_lengths elements)
	 *	@param[in] p_column_lengths is pointer to the array with the numbers of nonzeros in each
	 *		column of \f$A+A^T\f$ (as computed e.g. using \ref n_ColumnLengths_AATNoDiag())
	 *	@param[in] n_length_num is size of the p_column_lengths array (must match the greater dimension
	 *		of p_matrix)
	 *	@param[in] p_matrix is the input matrix, the column entries must to be sorted by row
	 *	@param[out] p_workspace is pointer to a temporary array (must be allocated at least to the
	 *		number of columns of p_matrix; may not reuse the same array as p_column_lengths)
	 *	@param[in] n_workspace_size is size of the p_workspace array (must be at least the number
	 *		of columns of p_matrix)
	 */
	template <bool b_upper_triangular, bool b_likely_upper_triangular, bool b_output_diagonal,
		bool b_output_full_diagonal, bool b_need_sorted_items, class CInt, class CInt1, class CInt2>
	static void AAT(CInt *p_column_ptrs, size_t n_column_ptr_num,
		CInt *__restrict p_row_inds, size_t n_nnz_num, const CInt1 *__restrict p_column_lengths, size_t n_length_num,
		const cs *__restrict p_matrix, CInt2 *p_workspace, size_t n_workspace_size)
	{
		_ASSERTE(!b_output_diagonal || !b_output_full_diagonal); // only one of those
		_ASSERTE(!b_upper_triangular || !b_likely_upper_triangular); // only one of those
		// todo - compile time asserts

		_ASSERTE(n_length_num == std::max(p_matrix->m, p_matrix->n)); // must match
		_ASSERTE(n_column_ptr_num == std::max(p_matrix->m, p_matrix->n) + 1); // must match
		_ASSERTE(b_upper_triangular || n_workspace_size >= size_t(p_matrix->n)); // must fit if not upper triangular (then it is not needed)

		enum {
			b_can_have_unsorted = !b_upper_triangular,
			b_handle_unsorted_items = b_can_have_unsorted && b_need_sorted_items
		};

		const csi *__restrict Ap = p_matrix->p, *__restrict Ai = p_matrix->i;
		const size_t n = p_matrix->n, nz = Ap[n];

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

		for(size_t i = 0, p1 = Ap[0]; i < n; ++ i) {
			const size_t p2 = Ap[i + 1]; // don't modify, will become the next p1
			_ASSERTE(CDebug::b_IsStrictlySortedSet(Ai + p1, Ai + p2)); // must be sorted
			const csi *__restrict p_diag;
			if(b_upper_triangular) { // compile-time-constant
				_ASSERTE(p1 <= p2);
				if(p1 != p2 && size_t(Ai[p2 - 1]) == i) {
					p_diag = Ai + (p2 - 1);
					++ n_diag_nnz_num;
				} else
					p_diag = Ai + p2;
				_ASSERTE(p1 == p2 || p_diag == Ai + p2 || *p_diag == i); // empty or not empty and points at the end or not empty and points before the end, at the diagonal element
				_ASSERTE(p1 == p2 || p_diag < Ai + p2 || *(p_diag - 1) < csi(i)); // empty or not empty and points before the end or not empty and points at the end and the one before is above-diagonal
				// note that p_transpose_col_off[i] is intentionally not used, the workspace will not be needed
			} else {
				p_diag = (b_likely_upper_triangular && p1 != p2 && Ai[p2 - 1] <= csi(i))?
					((Ai[p2 - 1] == csi(i))? Ai + (p2 - 1) : Ai + p2) : // it is one of the last two ones, depending whether the diagonal item is present or not
					std::lower_bound(Ai + p1, Ai + p2, csi(i)); // find the diagonal // in case the matrix is strictly upper, then this is a slight waste of time
				_ASSERTE(!b_likely_upper_triangular || p_diag == std::lower_bound(Ai + p1, Ai + p2, csi(i))); // make sure 
				if(p_diag != Ai + p2 && size_t(*p_diag) == i) { // in case there is a diagonal element
					++ n_diag_nnz_num;
					p_transpose_col_off[i] = CInt2(p_diag - (Ai /*+ p1*/) + 1); // point to the first below-diagonal element
				} else {
					_ASSERTE(p_diag == Ai + p2 || size_t(*p_diag) > i); // otherwise it is the first below-diagonal element
					p_transpose_col_off[i] = CInt2(p_diag - (Ai /*+ p1*/));
				}
				_ASSERTE(p_transpose_col_off[i] >= 0 && p_transpose_col_off[i] <= SIZE_MAX); // make sure it is ok to cast to size_t
				_ASSERTE(size_t(p_transpose_col_off[i]) >= p1 && size_t(p_transpose_col_off[i]) <= p2); // make sure it points to this column
				_ASSERTE(p_transpose_col_off[i] == p2 || size_t(Ai[p_transpose_col_off[i]]) > i); // make sure it points to a below-diagonal element
				_ASSERTE(p1 == p2 || p_transpose_col_off[i] != p2 || size_t(Ai[p_transpose_col_off[i] - 1]) <= i); // make sure it is preceded by above-or-diagonal elements
			}
			// find the upper triangular portion of this column

			size_t n_add_nnz = (b_output_full_diagonal)? (p_diag - (Ai + p1)) + 1 : // the number of above-diag elements + 1
				(b_output_diagonal)? // the number of elements in the upper part of the column, including the diagonal if present
				((b_upper_triangular)? (p_diag - (Ai + p1)) + ((p_diag != Ai + p2 && size_t(*p_diag) == i)? 1 : 0) : // simplified before, have to calculate it now
				p_transpose_col_off[i] - p1) : // the above else branch already calculated it
				(p_diag - (Ai + p1)); // the number of above-diag elements
			// number of (strictly) upper triangular items A(*, i)
			// in case the diagonal should be included in the column length, just use p_transpose_col_off[i] - p1
			// the diagonal entry would be added here

			CInt *__restrict p_next;
			{
				const csi *__restrict p_copy_end = ((b_output_diagonal)? ((b_upper_triangular)?
					Ai + (p1 + n_add_nnz) : Ai + p_transpose_col_off[i]) : p_diag); // either reuse p_transpose_col_off if we have it or use n_add_nnz if we don't
#if defined(_DEBUG) && defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1600
				// visual studio 2015 is giving a hard time with this warning; personally I dont like having
				// two versions of the code for debug / release but there seems to be no other (portable) way
				stdext::checked_array_iterator<const csi*> Ai_checked(Ai + p1, p2 - p1); // make sure we only access inside this col
				CInt n_dummy = 0xbaadf00d;
				_ASSERTE(p_row_inds != 0 || (p_column_dest[i] == 0 && ((i + 1 == n)? n_nnz_sum : p_column_dest[i + 1]) - p_column_dest[i] == 0));
				stdext::checked_array_iterator<CInt*> row_inds_checked((p_row_inds)? p_row_inds + p_column_dest[i] : &n_dummy, // checked iterator becomes utterly unusable if it is null (but we want it valid if it is also a zero length array)
					((i + 1 == n)? n_nnz_sum : p_column_dest[i + 1]) - p_column_dest[i]); // make sure we only access inside this col
				p_next = p_row_inds + ((std::copy(Ai_checked, Ai_checked + ((p_copy_end - Ai) - p1),
					row_inds_checked) - row_inds_checked) + p_column_dest[i]); // checked copy
				_ASSERTE(n_dummy == 0xbaadf00d); // make sure this wasnt overwritten
#else // _DEBUG && _MSC_VER && !__MWERKS__ && _MSC_VER >= 1700
#if defined(_MSC_VER) && !defined(__MWERKS__)
				#pragma warning(suppress: 4996) // suppress the MSVC warning about std::copy with unpreotected iterators below
#endif // _MSC_VER && !__MWERKS__
				p_next = std::copy(Ai + p1, p_copy_end, p_row_inds + p_column_dest[i]); // add the first n_add_nnz entries of column i
#endif // _DEBUG && _MSC_VER && !__MWERKS__ && _MSC_VER >= 1700
			}
			if(b_output_full_diagonal) {
				*p_next = i; // fill the diagonal entry
				_ASSERTE(++ p_next - (p_row_inds + p_column_dest[i]) == n_add_nnz); // make sure we filled what we promised
			} else
				_ASSERTE(p_next - (p_row_inds + p_column_dest[i]) == n_add_nnz); // make sure we filled what we promised
			// the diagonal entry is added here

			p_column_dest[i] += n_add_nnz; // don't forget to shift the destination

			if(b_handle_unsorted_items) { // compile-time constant
				//if((!b_output_full_diagonal && !b_output_diagonal && n_add_nnz) || // certainly not diag
				//   (b_output_full_diagonal && n_add_nnz > 1) || // always one diag, if there is more than one then it is not diag
				//   (b_output_diagonal && (n_add_nnz > 1 ||
				//   (n_add_nnz != 0 && p_row_inds[p_column_dest[i] - 1] != i)))) // if not diag
				if(n_add_nnz) // unfortunately any diagonals throw it off
					b_filled_from_upper = true;
				// although this looks horrible, the diagonal flags are compile time constants
			}

			for(const csi *p_row = Ai + p1; p_row != p_diag; ++ p_row) {
				const size_t j = *p_row;
				_ASSERTE(j < i);

				// the below loop could also be simplified using std::lower_bound() but the numbers
				// of iterations are most likely very small (it just "catches up" with incrementing i)

				if(!b_upper_triangular) {
					_ASSERTE(j < n); // this array is only indexed by columns
					size_t pj1 = size_t(p_transpose_col_off[j]); // already initialized
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
							if(b_handle_unsorted_items) // compile-time constant
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
					p_transpose_col_off[j] = CInt2(pj1); // remember where we stopped
					// ordered merge of transposed columns with the upper triangular entriex of this column
				}

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

		if(!b_upper_triangular) {
			for(size_t j = 0; j < n; ++ j) {
				_ASSERTE(j < n); // p_transpose_col_off array is only indexed by columns
				_ASSERTE(p_transpose_col_off[j] >= 0 && p_transpose_col_off[j] <= SIZE_MAX); // make sure it is ok to cast to size_t
				_ASSERTE(size_t(p_transpose_col_off[j]) >= size_t(Ap[j]) && size_t(p_transpose_col_off[j]) <= size_t(Ap[j + 1])); // make sure it points to its column
				for(size_t pj1 = size_t(p_transpose_col_off[j]), pj2 = Ap[j + 1]; pj1 < pj2; ++ pj1) {
					const size_t ii = Ai[pj1];
					_ASSERTE(ii > j); // this is in the lower triangle (in the untranposed matrix)
					// even in case the diagonal should be included then ii can't equal j

					if(b_handle_unsorted_items) // compile-time constant
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
		}

		if(b_handle_unsorted_items) { // compile-time constant
			if(!b_filled_from_upper)
				b_will_have_unsorted = false;
			// in case the matrix is strictly lower, then the entries are also filled nicely
		}

		if(b_output_full_diagonal && p_matrix->m > p_matrix->n) {
			for(size_t j = n, m = size_t(p_matrix->m); j < m; ++ j) {
				p_row_inds[p_column_dest[j]] = j;
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
			//	fprintf(stderr, "warning: predicted unsorted entries but had none\n");
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
};

#ifdef __MATRIX_TRANSPOSE_SUM_UNIT_TESTS

/**
 *	@brief unit tests for algorithms implemented in CMatrixTransposeSum
 *	@bote This is only available if __MATRIX_TRANSPOSE_SUM_UNIT_TESTS is defined.
 */
class CMatrixTransposeSum_UnitTests {
public:
	static void Test_AAT(const char *p_s_data_path = "data/AAT_testing") // throw(std::bad_alloc)
	{
		printf("testing an empry 0x0 matrix\n");
		{
			CUberBlockMatrix empty;
			Test_AAT(empty);
		}

		printf("testing a 1x1 matrix\n");
		{
			CUberBlockMatrix onesquared;
			onesquared.Append_Block(Eigen::Matrix<double, 1, 1>::Identity(), 0, 0);
			Test_AAT(onesquared);
		}

		printf("testing a 100x100 diagonal matrix\n");
		{
			std::vector<size_t> cumsums(100);
			for(size_t i = 0, n = cumsums.size(); i < n; ++ i)
				cumsums[i] = i + 1;
			CUberBlockMatrix diagonal(cumsums.begin(), cumsums.end(), cumsums.begin(), cumsums.end());
			diagonal.SetIdentity();
			Test_AAT(diagonal);
		}

		printf("testing a 100x100 zero matrix\n");
		{
			std::vector<size_t> cumsums(100);
			for(size_t i = 0, n = cumsums.size(); i < n; ++ i)
				cumsums[i] = i + 1;
			CUberBlockMatrix nonempty_zero(cumsums.begin(), cumsums.end(), cumsums.begin(), cumsums.end());
			Test_AAT(nonempty_zero);
		}

		printf("testing a 100x100 full matrix\n");
		{
			std::vector<size_t> cumsums(100);
			for(size_t i = 0, n = cumsums.size(); i < n; ++ i)
				cumsums[i] = i + 1;
			CUberBlockMatrix full(cumsums.begin(), cumsums.end(), cumsums.begin(), cumsums.end());
			for(size_t i = 0, n = full.n_BlockColumn_Num(); i < n; ++ i) {
				for(size_t j = 0; j < n; ++ j)
					full.Append_Block(Eigen::Matrix<double, 1, 1>::Identity(), j, i);
			}
			Test_AAT(full);
		}

		printf("testing all %d 4x4 pattern matrices\n", 1 << (4 * 4));
		{
			const size_t n = 4;
			std::vector<size_t> cumsums(n);
			for(size_t i = 0; i < n; ++ i)
				cumsums[i] = i + 1;
			CUberBlockMatrix matrix(cumsums.begin(), cumsums.end(), cumsums.begin(), cumsums.end());
			for(size_t i = 0, n_combo_num = size_t(1) << (n * n); i < n_combo_num; ++ i) {
				matrix.SetZero();
				for(size_t c = 0, n_fl = i; c < n; ++ c) {
					for(size_t r = 0; r < n; ++ r, n_fl >>= 1) {
						if(n_fl & 1)
							matrix.Append_Block(Eigen::Matrix<double, 1, 1>::Identity(), r, c);
					}
				}
				Test_AAT(matrix, i + 1 == n_combo_num);
			}
		}

		if(p_s_data_path) {
			std::string dp(p_s_data_path);
			Test_AAT_MatrixMarket((dp + "/assym/rajat01.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/assym/rbsa480.mtx").c_str()); // failure
			Test_AAT_MatrixMarket((dp + "/rect/Franz1.mtx").c_str());

			Test_AAT_MatrixMarket((dp + "/sym/msc01050.mtx").c_str()); // small ones first
			Test_AAT_MatrixMarket((dp + "/sym/msc01440.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/bcsstk37.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/bcsstm22.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/bcsstm35.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/debr.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/pct20stif.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/geom.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/Erdos02.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/sym/cell1.mtx").c_str());

			Test_AAT_MatrixMarket((dp + "/assym/rbsb480.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/assym/tols340.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/assym/internet.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/assym/soc-Epinions1.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/assym/email-EuAll.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/assym/NotreDame_www.mtx").c_str());

			Test_AAT_MatrixMarket((dp + "/rect/bibd_9_3.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/rect/bibd_9_5.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/rect/Franz10.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/rect/GL7d12.mtx").c_str());
			Test_AAT_MatrixMarket((dp + "/rect/IG5-8.mtx").c_str());
		}
	}

	static bool Test_AAT_MatrixMarket(const char *p_s_filename)
	{
		CUberBlockMatrix A;
		printf("debug: loading \'%s\' ... ", p_s_filename);
		if(!A.Load_MatrixMarket(p_s_filename, 1, false)) {
			printf("\n");
			fprintf(stderr, "error: failed to load \'%s\'\n", p_s_filename);
			return false;
		}
		printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
			(A.b_Square())? 1 : 0, (A.b_SymmetricBlockStructure())? 1 : 0,
			(A.b_UpperBlockTriangular())? 1 : 0, (A.b_LowerBlockTriangular())? 1 : 0,
			(A.b_All_Diagonal_Blocks())? "full" : (A.b_No_Diagonal_Blocks())? "none" : "some");
		Test_AAT(A);

		printf("debug: getting upper \'%s\' ... ", p_s_filename);
		CUberBlockMatrix U;
		U.TriangularViewOf(A, true); // get an upper-triangular view of the potentially rectangular matrix
		_ASSERTE(U.b_UpperBlockTriangular());
		printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
			(U.b_Square())? 1 : 0, (U.b_SymmetricBlockStructure())? 1 : 0,
			(U.b_UpperBlockTriangular())? 1 : 0, (U.b_LowerBlockTriangular())? 1 : 0,
			(U.b_All_Diagonal_Blocks())? "full" : (U.b_No_Diagonal_Blocks())? "none" : "some");
		Test_AAT(U);

		printf("debug: transposing \'%s\' ... ", p_s_filename);
		CUberBlockMatrix AT;
		AT.TransposeOf(A); // ...
		printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
			(AT.b_Square())? 1 : 0, (AT.b_SymmetricBlockStructure())? 1 : 0,
			(AT.b_UpperBlockTriangular())? 1 : 0, (AT.b_LowerBlockTriangular())? 1 : 0,
			(AT.b_All_Diagonal_Blocks())? "full" : (AT.b_No_Diagonal_Blocks())? "none" : "some");
		Test_AAT(AT);

		printf("debug: transposing upper \'%s\' ... ", p_s_filename);
		CUberBlockMatrix UT;
		UT.TransposeOf(U); // ...
		printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
			(UT.b_Square())? 1 : 0, (UT.b_SymmetricBlockStructure())? 1 : 0,
			(UT.b_UpperBlockTriangular())? 1 : 0, (UT.b_LowerBlockTriangular())? 1 : 0,
			(UT.b_All_Diagonal_Blocks())? "full" : (UT.b_No_Diagonal_Blocks())? "none" : "some");
		Test_AAT(UT);

		if(!A.b_SymmetricLayout()) {
			const size_t n_smaller_dim = std::min(A.n_BlockRow_Num(), A.n_BlockColumn_Num());

			{
				CUberBlockMatrix square;
				printf("debug: getting top-left square of \'%s\' ... ", p_s_filename);
				A.SliceTo(square, 0, n_smaller_dim, 0, n_smaller_dim, true); // make sure it is a *square* matrix
				printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
					(square.b_Square())? 1 : 0, (square.b_SymmetricBlockStructure())? 1 : 0,
					(square.b_UpperBlockTriangular())? 1 : 0, (square.b_LowerBlockTriangular())? 1 : 0,
					(square.b_All_Diagonal_Blocks())? "full" : (square.b_No_Diagonal_Blocks())? "none" : "some");
				Test_AAT(square);
			}
			{
				CUberBlockMatrix square;
				printf("debug: getting top-left square transpose of \'%s\' ... ", p_s_filename);
				AT.SliceTo(square, 0, n_smaller_dim, 0, n_smaller_dim, true); // make sure it is a *square* matrix
				printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
					(square.b_Square())? 1 : 0, (square.b_SymmetricBlockStructure())? 1 : 0,
					(square.b_UpperBlockTriangular())? 1 : 0, (square.b_LowerBlockTriangular())? 1 : 0,
					(square.b_All_Diagonal_Blocks())? "full" : (square.b_No_Diagonal_Blocks())? "none" : "some");
				Test_AAT(square);
			}
			{
				CUberBlockMatrix square;
				printf("debug: getting upper triangule of top-left square of \'%s\' ... ", p_s_filename);
				U.SliceTo(square, 0, n_smaller_dim, 0, n_smaller_dim, true); // make sure it is a *square* matrix
				printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
					(square.b_Square())? 1 : 0, (square.b_SymmetricBlockStructure())? 1 : 0,
					(square.b_UpperBlockTriangular())? 1 : 0, (square.b_LowerBlockTriangular())? 1 : 0,
					(square.b_All_Diagonal_Blocks())? "full" : (square.b_No_Diagonal_Blocks())? "none" : "some");
				Test_AAT(square);
			}
			{
				CUberBlockMatrix square;
				printf("debug: getting lower triangule of top-left square transpose of \'%s\' ... ", p_s_filename);
				UT.SliceTo(square, 0, n_smaller_dim, 0, n_smaller_dim, true); // make sure it is a *square* matrix
				printf("done (square: %d, sym: %d, upper: %d, lower: %d, diag: %s)\n",
					(square.b_Square())? 1 : 0, (square.b_SymmetricBlockStructure())? 1 : 0,
					(square.b_UpperBlockTriangular())? 1 : 0, (square.b_LowerBlockTriangular())? 1 : 0,
					(square.b_All_Diagonal_Blocks())? "full" : (square.b_No_Diagonal_Blocks())? "none" : "some");
				Test_AAT(square);
			}
		}

		return true;
	}

	template <bool b_upper_input, bool b_maybe_upper, bool b_want_diagonal, bool b_want_full_diag>
	static void Test_AAT_TemplateInstances(size_t &n_fail_num, const CUberBlockMatrix &r_A,
		const cs *p_A, const size_t n, const size_t n_bigger_dim, const size_t n_aat_nnz_ref,
		const std::vector<size_t> &col_counts_ref, const std::vector<size_t> &aat_col_ptrs_ref,
		const std::vector<size_t> &aat_row_inds_ref)
	{
		std::vector<size_t> workspace(n, size_t(0xbaadf00d));
		std::vector<size_t> col_counts_tpt(n_bigger_dim, size_t(0xbaadf00d));
		size_t n_aat_nnz_tpt = CMatrixTransposeSum::n_ColumnLengths_AAT<b_upper_input, b_maybe_upper,
			b_want_diagonal, b_want_full_diag>((col_counts_tpt.empty())? 0 : &col_counts_tpt.front(),
			col_counts_tpt.size(), p_A, (workspace.empty())? 0 : &workspace.front(), workspace.size());

		n_fail_num += (n_aat_nnz_ref == n_aat_nnz_tpt)? 0 : 1;
		n_fail_num += (col_counts_ref == col_counts_tpt)? 0 : 1;
		_ASSERTE(n_aat_nnz_ref == n_aat_nnz_tpt);
		_ASSERTE(col_counts_ref == col_counts_tpt);

		std::fill(workspace.begin(), workspace.end(), size_t(0xbaadf00d));
		std::fill(col_counts_tpt.begin(), col_counts_tpt.end(), size_t(0xbaadf00d));

		size_t n_aat_nnz_ubm = r_A.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<b_upper_input,
			b_maybe_upper, b_want_diagonal, b_want_full_diag>((col_counts_tpt.empty())? 0 :
			&col_counts_tpt.front(), col_counts_tpt.size(), (workspace.empty())? 0 : &workspace.front(),
			workspace.size());
		// test the implementation directly in UBM

		n_fail_num += (n_aat_nnz_ref == n_aat_nnz_ubm)? 0 : 1;
		n_fail_num += (col_counts_ref == col_counts_tpt)? 0 : 1;
		_ASSERTE(n_aat_nnz_ref == n_aat_nnz_ubm);
		_ASSERTE(col_counts_ref == col_counts_tpt);

		std::fill(workspace.begin(), workspace.end(), size_t(0xbaadf00d));

		std::vector<size_t> aat_col_ptrs_tpt(n_bigger_dim + 1, size_t(0xbaadf00d)),
			aat_row_inds_tpt(n_aat_nnz_tpt, size_t(0xbaadf00d));
		CMatrixTransposeSum::AAT<b_upper_input, b_maybe_upper, b_want_diagonal, b_want_full_diag,
			true>(&aat_col_ptrs_tpt.front(), aat_col_ptrs_tpt.size(),
			(aat_row_inds_tpt.empty())? 0 : &aat_row_inds_tpt.front(), aat_row_inds_tpt.size(),
			(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(), p_A,
			(workspace.empty())? 0 : &workspace.front(), workspace.size());

		n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_tpt)? 0 : 1;
		n_fail_num += (aat_row_inds_ref == aat_row_inds_tpt)? 0 : 1;
		_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_tpt);
		_ASSERTE(aat_row_inds_ref == aat_row_inds_tpt);

		std::fill(workspace.begin(), workspace.end(), size_t(0xbaadf00d));
		std::fill(aat_col_ptrs_tpt.begin(), aat_col_ptrs_tpt.end(), csi(0xbaadf00d));
		std::fill(aat_row_inds_tpt.begin(), aat_row_inds_tpt.end(), csi(0xbaadf00d));
		// make sure it doesn't deftly reuse previous results

		r_A.BlockStructure_SumWithSelfTranspose<b_upper_input, b_maybe_upper, b_want_diagonal,
			b_want_full_diag, true>(&aat_col_ptrs_tpt.front(), aat_col_ptrs_tpt.size(),
			(aat_row_inds_tpt.empty())? 0 : &aat_row_inds_tpt.front(), aat_row_inds_tpt.size(),
			(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
			(workspace.empty())? 0 : &workspace.front(), workspace.size());
		// test the implementation directly in UBM

		n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_tpt)? 0 : 1;
		n_fail_num += (aat_row_inds_ref == aat_row_inds_tpt)? 0 : 1;
		_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_tpt);
		_ASSERTE(aat_row_inds_ref == aat_row_inds_tpt);

		{
			std::fill(workspace.begin(), workspace.end(), size_t(0xbaadf00d));

			std::vector<csi> aat2_col_ptrs_tpt(n_bigger_dim + 1, csi(0xbaadf00d)),
				aat2_row_inds_tpt(n_aat_nnz_tpt, csi(0xbaadf00d));

			CMatrixTransposeSum::AAT<b_upper_input, b_maybe_upper, b_want_diagonal, b_want_full_diag,
				true>(&aat2_col_ptrs_tpt.front(), aat2_col_ptrs_tpt.size(),
				(aat2_row_inds_tpt.empty())? 0 : &aat2_row_inds_tpt.front(), aat2_row_inds_tpt.size(),
				(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
				p_A, (workspace.empty())? 0 : &workspace.front(), workspace.size());
			// csi column counts and row indices, size_t everything else

			{
				bool b_col_ptrs_ok = aat2_col_ptrs_tpt.size() == aat_col_ptrs_tpt.size() &&
					std::equal(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(),
					aat_col_ptrs_tpt.begin());
				bool b_row_inds_ok = aat2_row_inds_tpt.size() == aat_row_inds_tpt.size() &&
					std::equal(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(),
					aat_row_inds_tpt.begin());

				n_fail_num += (b_col_ptrs_ok)? 0 : 1;
				n_fail_num += (b_row_inds_ok)? 0 : 1;
				_ASSERTE(b_col_ptrs_ok);
				_ASSERTE(b_row_inds_ok);
			}

			std::fill(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(), csi(0xbaadf00d));
			std::fill(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(), csi(0xbaadf00d));
			// make sure it doesn't deftly reuse previous results

			r_A.BlockStructure_SumWithSelfTranspose<b_upper_input, b_maybe_upper, b_want_diagonal,
				b_want_full_diag, true>(&aat2_col_ptrs_tpt.front(), aat2_col_ptrs_tpt.size(),
				(aat2_row_inds_tpt.empty())? 0 : &aat2_row_inds_tpt.front(), aat2_row_inds_tpt.size(),
				(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
				(workspace.empty())? 0 : &workspace.front(), workspace.size());
			// test the implementation directly in UBM

			{
				bool b_col_ptrs_ok = aat2_col_ptrs_tpt.size() == aat_col_ptrs_tpt.size() &&
					std::equal(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(),
					aat_col_ptrs_tpt.begin());
				bool b_row_inds_ok = aat2_row_inds_tpt.size() == aat_row_inds_tpt.size() &&
					std::equal(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(),
					aat_row_inds_tpt.begin());

				n_fail_num += (b_col_ptrs_ok)? 0 : 1;
				n_fail_num += (b_row_inds_ok)? 0 : 1;
				_ASSERTE(b_col_ptrs_ok);
				_ASSERTE(b_row_inds_ok);
			}

			std::fill(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(), csi(0xbaadf00d));
			std::fill(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(), csi(0xbaadf00d));
			// make sure it doesn't deftly reuse previous results

			std::vector<int> workspacei(n, 0xbaadf00d); // just to try with a different type

			CMatrixTransposeSum::AAT<b_upper_input, b_maybe_upper, b_want_diagonal, b_want_full_diag,
				true>(&aat2_col_ptrs_tpt.front(), aat2_col_ptrs_tpt.size(),
				(aat2_row_inds_tpt.empty())? 0 : &aat2_row_inds_tpt.front(), aat2_row_inds_tpt.size(),
				(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
				p_A, (workspacei.empty())? 0 : &workspacei.front(), workspacei.size());
			// csi column counts and row indices, int workspace, size_t column counts

			{
				bool b_col_ptrs_ok = aat2_col_ptrs_tpt.size() == aat_col_ptrs_tpt.size() &&
					std::equal(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(),
					aat_col_ptrs_tpt.begin());
				bool b_row_inds_ok = aat2_row_inds_tpt.size() == aat_row_inds_tpt.size() &&
					std::equal(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(),
					aat_row_inds_tpt.begin());

				n_fail_num += (b_col_ptrs_ok)? 0 : 1;
				n_fail_num += (b_row_inds_ok)? 0 : 1;
				_ASSERTE(b_col_ptrs_ok);
				_ASSERTE(b_row_inds_ok);
			}

			std::fill(workspacei.begin(), workspacei.end(), 0xbaadf00d);
			std::fill(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(), csi(0xbaadf00d));
			std::fill(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(), csi(0xbaadf00d));
			// make sure it doesn't deftly reuse previous results

			r_A.BlockStructure_SumWithSelfTranspose<b_upper_input, b_maybe_upper, b_want_diagonal,
				b_want_full_diag, true>(&aat2_col_ptrs_tpt.front(), aat2_col_ptrs_tpt.size(),
				(aat2_row_inds_tpt.empty())? 0 : &aat2_row_inds_tpt.front(), aat2_row_inds_tpt.size(),
				(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
				(workspacei.empty())? 0 : &workspacei.front(), workspacei.size());
			// test the implementation directly in UBM

			{
				bool b_col_ptrs_ok = aat2_col_ptrs_tpt.size() == aat_col_ptrs_tpt.size() &&
					std::equal(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(),
					aat_col_ptrs_tpt.begin());
				bool b_row_inds_ok = aat2_row_inds_tpt.size() == aat_row_inds_tpt.size() &&
					std::equal(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(),
					aat_row_inds_tpt.begin());

				n_fail_num += (b_col_ptrs_ok)? 0 : 1;
				n_fail_num += (b_row_inds_ok)? 0 : 1;
				_ASSERTE(b_col_ptrs_ok);
				_ASSERTE(b_row_inds_ok);
			}

			std::fill(workspacei.begin(), workspacei.end(), 0xbaadf00d);
			std::fill(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(), csi(0xbaadf00d));
			std::fill(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(), csi(0xbaadf00d));
			// make sure it doesn't deftly reuse previous results

			std::vector<long> col_countsl_tpt(n_bigger_dim, long(0xbaadf00d));
			size_t n_aat_nnz_tpt = CMatrixTransposeSum::n_ColumnLengths_AAT<b_upper_input, b_maybe_upper,
				b_want_diagonal, b_want_full_diag>((col_countsl_tpt.empty())? 0 : &col_countsl_tpt.front(),
				col_countsl_tpt.size(), p_A, (workspacei.empty())? 0 : &workspacei.front(), workspacei.size());

			{
				bool b_col_lens_ok = col_countsl_tpt.size() == col_counts_ref.size() &&
					std::equal(col_countsl_tpt.begin(), col_countsl_tpt.end(),
					col_counts_ref.begin());

				n_fail_num += (n_aat_nnz_ref == n_aat_nnz_tpt)? 0 : 1;
				n_fail_num += (b_col_lens_ok)? 0 : 1;
				_ASSERTE(n_aat_nnz_ref == n_aat_nnz_tpt);
				_ASSERTE(b_col_lens_ok);
			}

			CMatrixTransposeSum::AAT<b_upper_input, b_maybe_upper, b_want_diagonal, b_want_full_diag,
				true>(&aat2_col_ptrs_tpt.front(), aat2_col_ptrs_tpt.size(),
				(aat2_row_inds_tpt.empty())? 0 : &aat2_row_inds_tpt.front(), aat2_row_inds_tpt.size(),
				(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
				p_A, (workspacei.empty())? 0 : &workspacei.front(), workspacei.size());
			// csi column counts and row indices, int workspace, long column counts

			{
				bool b_col_ptrs_ok = aat2_col_ptrs_tpt.size() == aat_col_ptrs_tpt.size() &&
					std::equal(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(),
					aat_col_ptrs_tpt.begin());
				bool b_row_inds_ok = aat2_row_inds_tpt.size() == aat_row_inds_tpt.size() &&
					std::equal(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(),
					aat_row_inds_tpt.begin());

				n_fail_num += (b_col_ptrs_ok)? 0 : 1;
				n_fail_num += (b_row_inds_ok)? 0 : 1;
				_ASSERTE(b_col_ptrs_ok);
				_ASSERTE(b_row_inds_ok);
			}

			std::fill(workspacei.begin(), workspacei.end(), 0xbaadf00d);
			std::fill(col_countsl_tpt.begin(), col_countsl_tpt.end(), long(0xbaadf00d));
			// make sure it doesn't deftly reuse previous results

			size_t n_aat_nnz_ubm = r_A.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<b_upper_input,
				b_maybe_upper, b_want_diagonal, b_want_full_diag>((col_countsl_tpt.empty())? 0 :
				&col_countsl_tpt.front(), col_countsl_tpt.size(), (workspacei.empty())? 0 :
				&workspacei.front(), workspacei.size());

			{
				bool b_col_lens_ok = col_countsl_tpt.size() == col_counts_ref.size() &&
					std::equal(col_countsl_tpt.begin(), col_countsl_tpt.end(),
					col_counts_ref.begin());

				n_fail_num += (n_aat_nnz_ref == n_aat_nnz_ubm)? 0 : 1;
				n_fail_num += (b_col_lens_ok)? 0 : 1;
				_ASSERTE(n_aat_nnz_ref == n_aat_nnz_ubm);
				_ASSERTE(b_col_lens_ok);
			}

			std::fill(workspacei.begin(), workspacei.end(), 0xbaadf00d);
			std::fill(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(), csi(0xbaadf00d));
			std::fill(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(), csi(0xbaadf00d));
			// make sure it doesn't deftly reuse previous results

			r_A.BlockStructure_SumWithSelfTranspose<b_upper_input, b_maybe_upper, b_want_diagonal,
				b_want_full_diag, true>(&aat2_col_ptrs_tpt.front(), aat2_col_ptrs_tpt.size(),
				(aat2_row_inds_tpt.empty())? 0 : &aat2_row_inds_tpt.front(), aat2_row_inds_tpt.size(),
				(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
				(workspacei.empty())? 0 : &workspacei.front(), workspacei.size());
			// test the implementation directly in UBM

			{
				bool b_col_ptrs_ok = aat2_col_ptrs_tpt.size() == aat_col_ptrs_tpt.size() &&
					std::equal(aat2_col_ptrs_tpt.begin(), aat2_col_ptrs_tpt.end(),
					aat_col_ptrs_tpt.begin());
				bool b_row_inds_ok = aat2_row_inds_tpt.size() == aat_row_inds_tpt.size() &&
					std::equal(aat2_row_inds_tpt.begin(), aat2_row_inds_tpt.end(),
					aat_row_inds_tpt.begin());

				n_fail_num += (b_col_ptrs_ok)? 0 : 1;
				n_fail_num += (b_row_inds_ok)? 0 : 1;
				_ASSERTE(b_col_ptrs_ok);
				_ASSERTE(b_row_inds_ok);
			}
		}
		// just try once to build it with a different type

		std::fill(workspace.begin(), workspace.end(), size_t(0xbaadf00d));
		std::fill(aat_col_ptrs_tpt.begin(), aat_col_ptrs_tpt.end(), size_t(0xbaadf00d));
		std::fill(aat_row_inds_tpt.begin(), aat_row_inds_tpt.end(), size_t(0xbaadf00d));
		// make sure it doesn't deftly reuse previous results
		//
		CMatrixTransposeSum::AAT<b_upper_input, b_maybe_upper, b_want_diagonal, b_want_full_diag,
			false>(&aat_col_ptrs_tpt.front(), aat_col_ptrs_tpt.size(),
			(aat_row_inds_tpt.empty())? 0 : &aat_row_inds_tpt.front(), aat_row_inds_tpt.size(),
			(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(), p_A,
			(workspace.empty())? 0 : &workspace.front(), workspace.size());
		for(size_t i = 1; i < aat_col_ptrs_tpt.size(); ++ i) {
			std::sort(aat_row_inds_tpt.begin() + aat_col_ptrs_tpt[i - 1],
				aat_row_inds_tpt.begin() + aat_col_ptrs_tpt[i]);
		}
		// generate unsorted, sort manually after-the-fact

		n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_tpt)? 0 : 1;
		n_fail_num += (aat_row_inds_ref == aat_row_inds_tpt)? 0 : 1;
		_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_tpt);
		_ASSERTE(aat_row_inds_ref == aat_row_inds_tpt);

		std::fill(workspace.begin(), workspace.end(), size_t(0xbaadf00d));
		std::fill(aat_col_ptrs_tpt.begin(), aat_col_ptrs_tpt.end(), csi(0xbaadf00d));
		std::fill(aat_row_inds_tpt.begin(), aat_row_inds_tpt.end(), csi(0xbaadf00d));
		// make sure it doesn't deftly reuse previous results

		r_A.BlockStructure_SumWithSelfTranspose<b_upper_input, b_maybe_upper, b_want_diagonal,
			b_want_full_diag, false>(&aat_col_ptrs_tpt.front(), aat_col_ptrs_tpt.size(),
			(aat_row_inds_tpt.empty())? 0 : &aat_row_inds_tpt.front(), aat_row_inds_tpt.size(),
			(col_counts_tpt.empty())? 0 : &col_counts_tpt.front(), col_counts_tpt.size(),
			(workspace.empty())? 0 : &workspace.front(), workspace.size());
		// test the implementation directly in UBM

		for(size_t i = 1; i < aat_col_ptrs_tpt.size(); ++ i) {
			std::sort(aat_row_inds_tpt.begin() + aat_col_ptrs_tpt[i - 1],
				aat_row_inds_tpt.begin() + aat_col_ptrs_tpt[i]);
		}
		// generate unsorted, sort manually after-the-fact

		n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_tpt)? 0 : 1;
		n_fail_num += (aat_row_inds_ref == aat_row_inds_tpt)? 0 : 1;
		_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_tpt);
		_ASSERTE(aat_row_inds_ref == aat_row_inds_tpt);
	}

	static void Test_AAT(const CUberBlockMatrix &r_mat, bool b_verbose = true)
	{
		{
			const CUberBlockMatrix &M = r_mat;
			
			bool b_shallow_copy = true; // or false
			
			CUberBlockMatrix upper; upper.TriangularViewOf(M, true, b_shallow_copy);
			_ASSERTE(upper.b_UpperBlockTriangular());
			
			CUberBlockMatrix strictly_upper; strictly_upper.TriangularViewOf(M, true, b_shallow_copy, -1);
			_ASSERTE(strictly_upper.b_StrictlyUpperBlockTriangular());
			
			CUberBlockMatrix lower; lower.TriangularViewOf(M, false, b_shallow_copy);
			_ASSERTE(lower.b_LowerBlockTriangular());
			
			CUberBlockMatrix strictly_lower; strictly_lower.TriangularViewOf(M, false, b_shallow_copy, +1);
			_ASSERTE(strictly_lower.b_StrictlyLowerBlockTriangular());
		}
		// test triangular views

		cs *p_A = r_mat.p_BlockStructure_to_Sparse();
		const size_t n_bigger_dim = std::max(p_A->m, p_A->n), n = p_A->n;

		std::vector<size_t> col_counts_ref(n_bigger_dim, size_t(0xbaadf00d)),
			col_counts_gen(n_bigger_dim, size_t(0xbaadf00d));
		size_t n_aat_nnz_ref = CMatrixTransposeSum::n_ColumnLengths_AAT_Ref(
			(col_counts_ref.empty())? 0 : &col_counts_ref.front(), col_counts_ref.size(), p_A);
		//
		std::vector<size_t> col_counts_diag_ref(n_bigger_dim, size_t(0xbaadf00d));
		size_t n_aat_nnz_diag_ref = CMatrixTransposeSum::n_ColumnLengths_AAT_Ref(
			(col_counts_diag_ref.empty())? 0 : &col_counts_diag_ref.front(),
			col_counts_diag_ref.size(), p_A, true);
		//
		std::vector<size_t> aat_col_ptrs_ref(n_bigger_dim + 1, size_t(0xbaadf00d)),
			aat_row_inds_ref(n_aat_nnz_ref, size_t(0xbaadf00d));
		CMatrixTransposeSum::AAT_Ref(&aat_col_ptrs_ref.front(), aat_col_ptrs_ref.size(),
			(aat_row_inds_ref.empty())? 0 : &aat_row_inds_ref.front(), aat_row_inds_ref.size(),
			(col_counts_ref.empty())? 0 : &col_counts_ref.front(), col_counts_ref.size(), p_A);
		//
		std::vector<size_t> aat_col_ptrs_diag_ref(n_bigger_dim + 1, size_t(0xbaadf00d)),
			aat_row_inds_diag_ref(n_aat_nnz_diag_ref, size_t(0xbaadf00d));
		CMatrixTransposeSum::AAT_Ref(&aat_col_ptrs_diag_ref.front(), aat_col_ptrs_diag_ref.size(),
			(aat_row_inds_diag_ref.empty())? 0 : &aat_row_inds_diag_ref.front(),
			aat_row_inds_diag_ref.size(), (col_counts_diag_ref.empty())? 0 :
			&col_counts_diag_ref.front(), col_counts_diag_ref.size(), p_A, true);
		// build reference A+AT without and with the original diagonal

		std::vector<size_t> workspace(n, size_t(0xbaadf00d));
		size_t n_aat_nnz_gen = CMatrixTransposeSum::n_ColumnLengths_AATNoDiag(
			(col_counts_gen.empty())? 0 : &col_counts_gen.front(), col_counts_gen.size(),
			p_A, (workspace.empty())? 0 : &workspace.front(), workspace.size());
		// the basic test

		size_t n_fail_num = 0;
		n_fail_num += (n_aat_nnz_ref == n_aat_nnz_gen)? 0 : 1;
		n_fail_num += (col_counts_ref == col_counts_gen)? 0 : 1;
		_ASSERTE(n_aat_nnz_ref == n_aat_nnz_gen);
		_ASSERTE(col_counts_ref == col_counts_gen);

		size_t n_aat_nnz_fulldiag_ref;
		std::vector<size_t> col_counts_fulldiag_ref(n_bigger_dim, size_t(0xbaadf00d));
		std::vector<size_t> aat_col_ptrs_fulldiag_ref(n_bigger_dim + 1, size_t(0xbaadf00d)),
			aat_row_inds_fulldiag_ref;
		{
			CUberBlockMatrix square_mat;
			if(!r_mat.b_Square()) {
				r_mat.SliceTo(square_mat, 0, r_mat.n_BlockRow_Num(), 0, r_mat.n_BlockColumn_Num(), true); // shallow copy
				for(size_t c = square_mat.n_Column_Num(), r = square_mat.n_Row_Num(); c < r;)
					square_mat.ExtendTo(r, ++ c);
				for(size_t c = square_mat.n_Column_Num(), r = square_mat.n_Row_Num(); r < c;)
					square_mat.ExtendTo(++ r, c);
				_ASSERTE(square_mat.b_Square()); // now it should be square
			}
			const CUberBlockMatrix &A = (r_mat.b_Square())? r_mat : square_mat;
			// extend the matrix to square, if not already (when adding the diagonal,
			// we need it over the whole square, not just the part of the diagonal
			// that fits the rectangle)

			CUberBlockMatrix diag;
			A.CopyLayoutTo(diag);
			diag.SetIdentity();
			A.AddTo(diag);
			cs *p_full_diag_mat = diag.p_BlockStructure_to_Sparse();

			n_aat_nnz_fulldiag_ref = CMatrixTransposeSum::n_ColumnLengths_AAT_Ref(
				(col_counts_fulldiag_ref.empty())? 0 : &col_counts_fulldiag_ref.front(),
				col_counts_fulldiag_ref.size(), p_full_diag_mat, true);

			aat_row_inds_fulldiag_ref.resize(n_aat_nnz_fulldiag_ref);
			CMatrixTransposeSum::AAT_Ref(&aat_col_ptrs_fulldiag_ref.front(), aat_col_ptrs_fulldiag_ref.size(),
				(aat_row_inds_fulldiag_ref.empty())? 0 : &aat_row_inds_fulldiag_ref.front(),
				aat_row_inds_fulldiag_ref.size(), (col_counts_fulldiag_ref.empty())? 0 :
				&col_counts_fulldiag_ref.front(), col_counts_fulldiag_ref.size(), p_full_diag_mat, true);

			cs_spfree(p_full_diag_mat);

			_ASSERTE(!r_mat.b_Square() || !r_mat.b_All_Diagonal_Blocks() ||
				n_aat_nnz_fulldiag_ref == n_aat_nnz_diag_ref);
			_ASSERTE(!r_mat.b_Square() || !r_mat.b_All_Diagonal_Blocks() ||
				col_counts_fulldiag_ref == col_counts_diag_ref);
			// in case the matrix is square and has full diagonal, the resutls
			// here should be the same as in the branch with the diagonal
		}
		// build reference A+AT with full diagonal

		const bool b_is_upper = r_mat.b_UpperBlockTriangular();

		{
			Test_AAT_TemplateInstances<false, false, false, false>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_ref, col_counts_ref, aat_col_ptrs_ref, aat_row_inds_ref);
		}
		if(b_is_upper) {
			Test_AAT_TemplateInstances<true, false, false, false>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_ref, col_counts_ref, aat_col_ptrs_ref, aat_row_inds_ref);
		}
		{
			Test_AAT_TemplateInstances<false, true, false, false>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_ref, col_counts_ref, aat_col_ptrs_ref, aat_row_inds_ref);
		}
		// tests of A+AT without diagonal

		{
			Test_AAT_TemplateInstances<false, false, true, false>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_diag_ref, col_counts_diag_ref, aat_col_ptrs_diag_ref, aat_row_inds_diag_ref);
		}
		if(b_is_upper) {
			Test_AAT_TemplateInstances<true, false, true, false>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_diag_ref, col_counts_diag_ref, aat_col_ptrs_diag_ref, aat_row_inds_diag_ref);
		}
		{
			Test_AAT_TemplateInstances<false, true, true, false>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_diag_ref, col_counts_diag_ref, aat_col_ptrs_diag_ref, aat_row_inds_diag_ref);
		}
		// tests of A+AT with the original diagonal

		{
			Test_AAT_TemplateInstances<false, false, false, true>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_fulldiag_ref, col_counts_fulldiag_ref, aat_col_ptrs_fulldiag_ref, aat_row_inds_fulldiag_ref);
		}
		if(b_is_upper) {
			Test_AAT_TemplateInstances<true, false, false, true>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_fulldiag_ref, col_counts_fulldiag_ref, aat_col_ptrs_fulldiag_ref, aat_row_inds_fulldiag_ref);
		}
		{
			Test_AAT_TemplateInstances<false, true, false, true>(n_fail_num, r_mat, p_A, n, n_bigger_dim,
				n_aat_nnz_fulldiag_ref, col_counts_fulldiag_ref, aat_col_ptrs_fulldiag_ref, aat_row_inds_fulldiag_ref);
		}
		// tests of A+AT with full diagonal

		std::vector<size_t> aat_col_ptrs_gen(n_bigger_dim + 1, size_t(0xbaadf00d)),
			aat_row_inds_gen(n_aat_nnz_gen, size_t(0xbaadf00d));
		CMatrixTransposeSum::AATNoDiag(&aat_col_ptrs_gen.front(), aat_col_ptrs_gen.size(),
			(aat_row_inds_gen.empty())? 0 : &aat_row_inds_gen.front(), aat_row_inds_gen.size(),
			(col_counts_gen.empty())? 0 : &col_counts_gen.front(), col_counts_gen.size(), p_A,
			(workspace.empty())? 0 : &workspace.front(), workspace.size());

		n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_gen)? 0 : 1;
		n_fail_num += (aat_row_inds_ref == aat_row_inds_gen)? 0 : 1;
		_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_gen);
		_ASSERTE(aat_row_inds_ref == aat_row_inds_gen);

		/*if(!r_mat.b_SymmetricLayout() || !b_is_upper) { // works but now is unnecessary
			CUberBlockMatrix square;
			const size_t n_smaller_dim = std::min(r_mat.n_BlockRow_Num(), r_mat.n_BlockColumn_Num()), n = n_smaller_dim;
			r_mat.SliceTo(square, 0, n_smaller_dim, 0, n_smaller_dim, true); // make sure it is a *square* matrix
			CUberBlockMatrix utvs;
			utvs.TriangularViewOf(square, true); // get a triangular view of the square
			p_A = utvs.p_BlockStructure_to_Sparse(p_A);

			if(r_mat.b_SymmetricBlockStructure()) {
				n_aat_nnz_gen = n_aat_nnz_ref;
				col_counts_gen = col_counts_ref;
				aat_col_ptrs_gen = aat_col_ptrs_ref;
				aat_row_inds_gen = aat_row_inds_ref;
				// backup the reference result
			}
			// if the matrix has symmetric structure

			col_counts_ref.resize(n_smaller_dim);
			n_aat_nnz_ref = CMatrixTransposeSum::n_ColumnLengths_AAT_Ref((col_counts_ref.empty())? 0 :
				&col_counts_ref.front(), col_counts_ref.size(), p_A);

			aat_col_ptrs_ref.resize(n_smaller_dim + 1);
			aat_row_inds_ref.resize(n_aat_nnz_ref);
			CMatrixTransposeSum::AAT_Ref(&aat_col_ptrs_ref.front(), aat_col_ptrs_ref.size(),
				(aat_row_inds_ref.empty())? 0 : &aat_row_inds_ref.front(), aat_row_inds_ref.size(),
				(col_counts_ref.empty())? 0 : &col_counts_ref.front(), col_counts_ref.size(), p_A);

			if(r_mat.b_SymmetricBlockStructure()) {
				_ASSERTE(n_aat_nnz_ref == n_aat_nnz_gen);
				_ASSERTE(col_counts_ref == col_counts_gen);
				_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_gen);
				_ASSERTE(aat_row_inds_ref == aat_row_inds_gen);
				n_fail_num += (n_aat_nnz_ref == n_aat_nnz_gen)? 0 : 1;
				n_fail_num += (col_counts_ref == col_counts_gen)? 0 : 1;
				n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_gen)? 0 : 1;
				n_fail_num += (aat_row_inds_ref == aat_row_inds_gen)? 0 : 1;
				// make sure the result of the reference algorithm did not change
			}
			// if the matrix has symmetric structure

			col_counts_gen.resize(n_smaller_dim);
			workspace.resize(n);
			n_aat_nnz_gen = CMatrixTransposeSum::n_ColumnLengths_AATNoDiag((col_counts_gen.empty())? 0 :
				&col_counts_gen.front(), col_counts_gen.size(), p_A, (workspace.empty())? 0 :
				&workspace.front(), workspace.size());

			n_fail_num += (n_aat_nnz_ref == n_aat_nnz_gen)? 0 : 1;
			n_fail_num += (col_counts_ref == col_counts_gen)? 0 : 1;
			_ASSERTE(n_aat_nnz_ref == n_aat_nnz_gen);
			_ASSERTE(col_counts_ref == col_counts_gen);

			aat_col_ptrs_gen.resize(n_smaller_dim + 1);
			aat_row_inds_gen.resize(n_aat_nnz_gen);
			CMatrixTransposeSum::AATNoDiag(&aat_col_ptrs_gen.front(), aat_col_ptrs_gen.size(),
				(aat_row_inds_gen.empty())? 0 : &aat_row_inds_gen.front(), aat_row_inds_gen.size(),
				(col_counts_gen.empty())? 0 : &col_counts_gen.front(), col_counts_gen.size(),
				p_A, (workspace.empty())? 0 : &workspace.front(), workspace.size());

			n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_gen)? 0 : 1;
			n_fail_num += (aat_row_inds_ref == aat_row_inds_gen)? 0 : 1;
			_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_gen);
			_ASSERTE(aat_row_inds_ref == aat_row_inds_gen);

			if(r_mat.b_All_Diagonal_Blocks()) { // otherwise n_ColumnLengths_AATNoDiag_UpperFullDiagonal() does not work
				std::vector<size_t> col_counts_ufd(n_smaller_dim, size_t(0xbaadf00d));
				size_t n_aat_nnz_ufd = CMatrixTransposeSum::n_ColumnLengths_AATNoDiag_UpperFullDiagonal(
					(col_counts_ufd.empty())? 0 : &col_counts_ufd.front(),
					col_counts_ufd.size(), p_A);

				n_fail_num += (n_aat_nnz_ref == n_aat_nnz_ufd)? 0 : 1;
				n_fail_num += (col_counts_ref == col_counts_ufd)? 0 : 1;
				_ASSERTE(n_aat_nnz_ref == n_aat_nnz_ufd);
				_ASSERTE(col_counts_ref == col_counts_ufd);

				std::vector<size_t> aat_col_ptrs_ufd(n_smaller_dim + 1, size_t(0xbaadf00d)),
					aat_row_inds_ufd(n_aat_nnz_ufd, size_t(0xbaadf00d));
				CMatrixTransposeSum::AATNoDiag_UpperFullDiagonal(&aat_col_ptrs_ufd.front(),
				aat_col_ptrs_ufd.size(), (aat_row_inds_ufd.empty())? 0 : &aat_row_inds_ufd.front(),
					aat_row_inds_ufd.size(), (col_counts_ufd.empty())? 0 : &col_counts_ufd.front(),
					col_counts_ufd.size(), p_A);

				n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_ufd)? 0 : 1;
				n_fail_num += (aat_row_inds_ref == aat_row_inds_ufd)? 0 : 1;
				_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_ufd);
				_ASSERTE(aat_row_inds_ref == aat_row_inds_ufd);
			}
		} else {
			if(r_mat.b_All_Diagonal_Blocks()) { // otherwise n_ColumnLengths_AATNoDiag_UpperFullDiagonal() does not work
				std::vector<size_t> col_counts_ufd(n_bigger_dim, size_t(0xbaadf00d));
				size_t n_aat_nnz_ufd = CMatrixTransposeSum::n_ColumnLengths_AATNoDiag_UpperFullDiagonal(
					(col_counts_ufd.empty())? 0 : &col_counts_ufd.front(),
					col_counts_ufd.size(), p_A);

				_ASSERTE(n_aat_nnz_ref == n_aat_nnz_ufd);
				_ASSERTE(col_counts_ref == col_counts_ufd);
				n_fail_num += (n_aat_nnz_ref == n_aat_nnz_ufd)? 0 : 1;
				n_fail_num += (col_counts_ref == col_counts_ufd)? 0 : 1;

				std::vector<size_t> aat_col_ptrs_ufd(n_bigger_dim + 1, size_t(0xbaadf00d)),
					aat_row_inds_ufd(n_aat_nnz_ufd, size_t(0xbaadf00d));
				CMatrixTransposeSum::AATNoDiag_UpperFullDiagonal(&aat_col_ptrs_ufd.front(),
				aat_col_ptrs_ufd.size(), (aat_row_inds_ufd.empty())? 0 : &aat_row_inds_ufd.front(),
					aat_row_inds_ufd.size(), (col_counts_ufd.empty())? 0 : &col_counts_ufd.front(),
					col_counts_ufd.size(), p_A);

				n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_ufd)? 0 : 1;
				n_fail_num += (aat_row_inds_ref == aat_row_inds_ufd)? 0 : 1;
				_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_ufd);
				_ASSERTE(aat_row_inds_ref == aat_row_inds_ufd);
			}
		}*/

		if(r_mat.b_SymmetricLayout() && b_is_upper && r_mat.b_All_Diagonal_Blocks()) { // otherwise n_ColumnLengths_AATNoDiag_UpperFullDiagonal() does not work
			std::vector<size_t> col_counts_ufd(n_bigger_dim, size_t(0xbaadf00d));
			size_t n_aat_nnz_ufd = CMatrixTransposeSum::n_ColumnLengths_AATNoDiag_UpperFullDiagonal(
				(col_counts_ufd.empty())? 0 : &col_counts_ufd.front(),
				col_counts_ufd.size(), p_A);

			_ASSERTE(n_aat_nnz_ref == n_aat_nnz_ufd);
			_ASSERTE(col_counts_ref == col_counts_ufd);
			n_fail_num += (n_aat_nnz_ref == n_aat_nnz_ufd)? 0 : 1;
			n_fail_num += (col_counts_ref == col_counts_ufd)? 0 : 1;

			std::vector<size_t> aat_col_ptrs_ufd(n_bigger_dim + 1, size_t(0xbaadf00d)),
				aat_row_inds_ufd(n_aat_nnz_ufd, size_t(0xbaadf00d));
			CMatrixTransposeSum::AATNoDiag_UpperFullDiagonal(&aat_col_ptrs_ufd.front(),
				aat_col_ptrs_ufd.size(), (aat_row_inds_ufd.empty())? 0 : &aat_row_inds_ufd.front(),
				aat_row_inds_ufd.size(), (col_counts_ufd.empty())? 0 : &col_counts_ufd.front(),
				col_counts_ufd.size(), p_A);

			n_fail_num += (aat_col_ptrs_ref == aat_col_ptrs_ufd)? 0 : 1;
			n_fail_num += (aat_row_inds_ref == aat_row_inds_ufd)? 0 : 1;
			_ASSERTE(aat_col_ptrs_ref == aat_col_ptrs_ufd);
			_ASSERTE(aat_row_inds_ref == aat_row_inds_ufd);
		}

		cs_spfree(p_A);

		static size_t n_global_fail_num = 0;
		n_global_fail_num += n_fail_num;

		if(n_fail_num)
			fprintf(stderr, "error: there were %d fails\n", int(n_fail_num));
		else if(n_global_fail_num) {
			if(b_verbose)
				printf("\tall tests passed but some failed in the past (%d)\n", int(n_global_fail_num));
		} else {
			if(b_verbose)
				printf("\tall tests passed\n");
		}
	}
};

#endif // __MATRIX_TRANSPOSE_SUM_UNIT_TESTS

/** @} */ // end of group

#endif // !__MATRIX_ORDERING_UTILS_INCLUDED
