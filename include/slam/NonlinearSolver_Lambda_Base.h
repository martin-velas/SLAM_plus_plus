/*
								+-----------------------------------+
								|                                   |
								| ***  Lambda nonlinear solver  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|   NonlinearSolver_Lambda_Base.h   |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_SOLVER_LAMBDA_UTILS
#define __NONLINEAR_SOLVER_LAMBDA_UTILS

/**
 *	@file include/slam/NonlinearSolver_Lambda_Base.h
 *	@brief utilitites for nonlinear solvers working above the lambda matrix
 *	@author -tHE SWINe-
 *	@date 2015-06-25
 */

#include "slam/BlockMatrix.h"
//#include <numeric>

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2
 *	@brief enables writes of chi2 errors at each step
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DUMP_CHI2

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DUMP_ICRA2013_ANIMATION_DATA
 *	@brief dump ICRA 2013 slam race data
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DUMP_ICRA2013_ANIMATION_DATA

/**
 *	@def __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
 *	@brief dump RSS 2013 matrix animation data
 */
//#define __NONLINEAR_SOLVER_LAMBDA_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

/**
 *	@brief utilities for lambda solvers
 */
namespace lambda_utils {

/**
 *	@brief static assertion helper
 *	@brief b_expression is expression being asserted
 */
template <bool b_expression>
class CReductionPlanAssert {
public:
	typedef void BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST; /**< @brief static assertion tag */
};

/**
 *	@brief static assertion helper (specialization for assertion failed)
 */
template <>
class CReductionPlanAssert<false> {};

/**
 *	@brief calculates std::set memory allocation size estimate
 *	@tparam P is set payload type
 *	@param[in] r_set is the set to estimate memory size of
 *	@return Returns the approximate size of the given set, in bytes.
 */
template <class P>
static size_t n_Set_Allocation_Size(const std::set<P> &r_set)
{
	return (sizeof(P) + sizeof(int) + 2 * sizeof(void*)) * // size of a node (payload, red/black, children)
		((r_set.size() * 2) / 3) + sizeof(std::set<P>); // number of leafs + other nodes + size of the struct
}

/**
 *	@brief calculates std::map memory allocation size estimate
 *
 *	@tparam K is map key type
 *	@tparam P is map payload type
 *
 *	@param[in] r_map is the map to estimate memory size of
 *
 *	@return Returns the approximate size of the given map, in bytes.
 */
template <class K, class P>
static size_t n_Map_Allocation_Size(const std::map<K, P> &r_map)
{
	return (sizeof(K) + sizeof(P) + sizeof(int) + 2 * sizeof(void*)) * // size of a node (key, payload, red/black, children)
		((r_map.size() * 2) / 3) + sizeof(std::map<K, P>); // number of leafs + other nodes + size of the struct
}

// todo - consider not storing the reductions in a pool, instead store them in the map and keep a vector of pointers for parallel processing?

/**
 *	@brief right hand side vector reduction plan
 *
 *	This takes care of summing up right hand side (residual) vector contributions from different edges.
 *	In the v1 reduction plan this was done by the vertex class, each vertex had to keep a list of referencing
 *	edges and each edge contained a vector for the r.h.s. contribution. This does not improve efficiency but
 *	seems like a slightly cleaner solution.
 *
 *	@tparam CDimsList is list of block sizes in lambda matrix
 */
template <class CDimsList> // todo - need clear!
class CVectorReductionPlan { // t_odo - could we possibly move this to nl solver lambda.h? // seems like we did
public:
	typedef typename CTransformTypelist<CDimsList, fbs_ut::CEigenToDimension>::_TyResult _TyBlockSizeList; /**< @brief list of block sizes in lambda */
	typedef typename CUniqueTypelist<typename CFilterTypelist1<_TyBlockSizeList,
		fbs_ut::CIsSquare>::_TyResult>::_TyResult _TyVertDimsList; /**< @brief list of square block sizes (corresponding to vertices) */
	typedef typename CTransformTypelist<_TyVertDimsList,
		fbs_ut::CTransformDimensionColumnsToSize>::_TyResult _TyDimensionList; /**< @brief list of vertex dimensions (as fbs_ut::CCTSize, not 2D) */

	/**
	 *	@brief reduction plan parameters, stored as enum
	 */
	enum {
		n_reductor_num = CTypelistLength<_TyDimensionList>::n_result, /**< @brief number of different reduction sizes */
		n_pool_page_size = 4096, /**< @brief reduction pool page size */
		n_pool_memory_align = 0 /**< @brief reduction pool memory alignment */ // see no benefit in alignment right now, this stores the TReduction elements, those do not need to be aligned
	};

protected:
	/**
	 *	@brief reduction description
	 *	@note This does not store reduction dimension; that is given by the index of the pool that this is found in.
	 */
	struct TReduction {
		size_t n_offset; /**< @brief offset in the right hand side vector */
		std::vector<const double*> src_list; /**< @brief list of sources */
	};

	typedef forward_allocated_pool<TReduction,
		n_pool_page_size, n_pool_memory_align> _TyPool; /**< @brief pool for storing reduction info */ // provides random access and const pointers
	typedef std::map<size_t, TReduction*> _TyReductionMap; /**< @brief map of reductions, ordered by dest block address */ // note that the dest block address is duplicated here, maybe could use std::set with an appropriate comparison

	_TyPool m_p_reduction_pool[n_reductor_num]; /**< @brief list of reduction pools, one per each vertex dimension */
	CUberBlockMatrix m_p_matrix[n_reductor_num]; /**< @brief list of data stores for the per-edge contributions, one per each vertex dimension */ // storage only (could we do the same with just a pool and an allocator from CUberBlockMatrix, that would be better, p_Get_DenseStorage() could be protected again)
	_TyReductionMap m_p_reduction[n_reductor_num]; /**< @brief list of reductions, one per each vertex dimension */ // need to have them sorted by size for loop unrolling

	/**
	 *	@brief reduction function object
	 *	@note This is used to calculate reduction of all the vertices' r.h.s. vectors at once.
	 */
	class CReduce {
	protected:
		const CVectorReductionPlan<CDimsList> *m_p_this; /**< @brief pointer to the parent reduction plan */
		Eigen::VectorXd &m_r_dest; /**< @brief destination r.h.s. vector */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] p_this is pointer to the parent reduction plan
		 *	@param[out] r_dest is destination r.h.s. vector (filled once the function operator is invoked)
		 */
		inline CReduce(const CVectorReductionPlan<CDimsList> *p_this, Eigen::VectorXd &r_dest)
			:m_p_this(p_this), m_r_dest(r_dest)
		{}

		/**
		 *	@brief function operator; performs all the reductions of one vertex dimension
		 *	@tparam C1DSize is vertex size (a specialization of fbs_ut::CCTSize)
		 */
		template <class C1DSize>
		inline void operator ()()
		{
			typedef CFindTypelistItem<_TyDimensionList, C1DSize> CSearch; // look for the size
			typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
			// the search happens at compile-time, each call to t_GetTempBlock is already linked to use a specific index

			typedef typename Eigen::VectorBlock<Eigen::VectorXd, C1DSize::n_size> CDestMap;
			typedef typename CUberBlockMatrix::CMakeMatrixRef<C1DSize::n_size, 1>::_TyConst CSrcMap;

			const _TyPool &reduction_pool = m_p_this->m_p_reduction_pool[CSearch::n_index];
			size_t _n = reduction_pool.size();
			_ASSERTE(_n <= INT_MAX);
			int n = int(_n);
			#pragma omp parallel for if(n > 50) // todo - dynamic schedule and below as well
			for(int i = 0; i < n; ++ i) {
				const TReduction &r_red = reduction_pool[i];
				CDestMap dest_map = m_r_dest.segment<C1DSize::n_size>(r_red.n_offset);
				CSrcMap src0_map(r_red.src_list.front());
				dest_map = src0_map; // can this be used? can the first block still use the original block inside the matrix, without having to use a temporary? probably not. todo
				for(size_t j = 1, m = r_red.src_list.size(); j < m; ++ j)
					dest_map += CSrcMap(r_red.src_list[j]);
				// reduce
			}
		}
	};

	/**
	 *	@brief reduction function object
	 *	@note This is used to calculate reduction of a subset of vertices.
	 */
	class CReduceRange {
	protected:
		const CVectorReductionPlan<CDimsList> *m_p_this; /**< @brief pointer to the parent reduction plan */
		Eigen::VectorXd &m_r_dest; /**< @brief destination r.h.s. vector */
		size_t m_n_begin; /**< @brief zero-based index of the first element of the r.h.s. vector to calculate (not vertex id) */
		size_t m_n_end; /**< @brief zero-based index of one past the last element of the r.h.s. vector to calculate (not vertex id) */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] p_this is pointer to the parent reduction plan
		 *	@param[out] r_dest is destination r.h.s. vector (filled once the function operator is invoked)
		 *	@param[in] n_begin is zero-based index of the first element of the r.h.s. vector to calculate (not vertex id)
		 *	@param[in] n_end is zero-based index of one past the last element of the r.h.s. vector to calculate (not vertex id)
		 */
		inline CReduceRange(const CVectorReductionPlan<CDimsList> *p_this,
			Eigen::VectorXd &r_dest, size_t n_begin, size_t n_end)
			:m_p_this(p_this), m_r_dest(r_dest), m_n_begin(n_begin), m_n_end(n_end)
		{}

		/**
		 *	@brief function operator; performs the selected reductions of one vertex dimension
		 *	@tparam C1DSize is vertex size (a specialization of fbs_ut::CCTSize)
		 */
		template <class C1DSize>
		inline void operator ()()
		{
			typedef CFindTypelistItem<_TyDimensionList, C1DSize> CSearch; // look for the size
			typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
			// the search happens at compile-time, each call to t_GetTempBlock is already linked to use a specific index

			typedef typename Eigen::VectorBlock<Eigen::VectorXd, C1DSize::n_size> CDestMap;
			typedef typename CUberBlockMatrix::CMakeMatrixRef<C1DSize::n_size, 1>::_TyConst CSrcMap;

			const _TyReductionMap &reduction_map = m_p_this->m_p_reduction[CSearch::n_index];
			typename _TyReductionMap::const_iterator p_begin_it = reduction_map.lower_bound(m_n_begin);
			typename _TyReductionMap::const_iterator p_end_it = reduction_map.upper_bound(m_n_end);
			// find range of reductions of the selected size to perform

			for(; p_begin_it != p_end_it; ++ p_begin_it) {
				const TReduction &r_red = *(*p_begin_it).second;
				CDestMap dest_map = m_r_dest.segment<C1DSize::n_size>(r_red.n_offset);
				CSrcMap src0_map(r_red.src_list.front());
				dest_map = src0_map; // can this be used? can the first block still use the original block inside the matrix, without having to use a temporary? probably not. // not sure what was wrong with it, this works.
				for(size_t j = 1, m = r_red.src_list.size(); j < m; ++ j)
					dest_map += CSrcMap(r_red.src_list[j]);
				// reduce
			}
			// can't do this easily in parallel
		}
	};

public:
	/**
	 *	@brief destructor; performs consistency checks in debug
	 */
	~CVectorReductionPlan()
	{
#ifdef _DEBUG
		int p_dims_list[n_reductor_num];
		fbs_ut::Copy_CTSizes_to_Array<_TyDimensionList>(p_dims_list);
		// convert the typelist to an array so that we can index it at runtime

		std::vector<std::pair<size_t, int> > allocated_segments;
		for(int i = 0; i < n_reductor_num; ++ i) {
			const int n_dim = p_dims_list[i];
			//_ASSERTE(!i || n_dim > p_dims_list[i - 1]); // it is not sorted, only unique, would have to check differently
			const _TyReductionMap &reduction_map = m_p_reduction[i];
			for(typename _TyReductionMap::const_iterator p_begin_it = reduction_map.begin(),
			   p_end_it = reduction_map.end(); p_begin_it != p_end_it; ++ p_begin_it) {
				const TReduction &r_red = *(*p_begin_it).second;
				std::pair<size_t, int> segment(r_red.n_offset, n_dim);
				allocated_segments.push_back(segment);
			}
		}
		// collect allocated segments as (offset, dimension) pairs

		std::sort(allocated_segments.begin(), allocated_segments.end());
		// sort the segments (they are sorted for each dimension, merging them explicitly
		// would be more efficient but also error prone; this is debug, correctness is paramount)

		_ASSERTE(std::unique(allocated_segments.begin(), allocated_segments.end()) == allocated_segments.end());
		// make sure there are no duplicates

		_ASSERTE(allocated_segments.empty() || !allocated_segments.front().first);
		// make sure that the first segment starts at zero

		for(size_t i = 1, n = allocated_segments.size(); i < n; ++ i)
			_ASSERTE(allocated_segments[i].first == allocated_segments[i - 1].first + allocated_segments[i - 1].second); // if this triggers, most likely there are vertices in the system which are not observed; please, if you have unobserved vertices, do not put them in the system just yet; add them in the system once there are corresponding observations // todo - make an earlier assert which makes sure that a more verbose message is shown to the user
		// make sure that the next segment starts where the previous one ends
#endif // _DEBUG
	}

	/**
	 *	@brief gets the maximum reduced dimension
	 *	@return Returns the maximum reduced dimension, in elements.
	 *	@note This is also the expected size of the r.h.s. vector.
	 */
	size_t n_Max_Dimension() const
	{
		int p_dims_list[n_reductor_num];
		fbs_ut::Copy_CTSizes_to_Array<_TyDimensionList>(p_dims_list);
		// convert the typelist to an array so that we can index it at runtime

		size_t n_max = 0;
		for(int i = 0; i < n_reductor_num; ++ i) {
			if(!m_p_reduction[i].empty()) {
				typename _TyReductionMap::const_iterator p_back_it = -- m_p_reduction[i].end(); // no .back() on this thing
				const TReduction &r_red = *(*p_back_it).second;
				size_t n_last = r_red.n_offset + p_dims_list[i];
				if(n_max < n_last)
					n_max = n_last;
			}
		}
		// find the one past the last element that will be written

		return n_max;
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		size_t n_size = sizeof(CVectorReductionPlan<CDimsList>);
		size_t n_data_size = 0, n_maps_size = 0, n_pools_size = 0,
			n_vectors_size = 0, n_vectors_slack = 0;
		for(int i = 0; i < n_reductor_num; ++ i) {
			n_data_size += m_p_matrix[i].n_Allocation_Size() - sizeof(CUberBlockMatrix);
			n_maps_size += n_Map_Allocation_Size(m_p_reduction[i]) - sizeof(m_p_reduction[i]);
			n_pools_size += m_p_reduction_pool[i].capacity() * sizeof(TReduction) +
				m_p_reduction_pool[i].page_num() * sizeof(TReduction*);
			for(size_t j = 0, m = m_p_reduction_pool[i].size(); j < m; ++ j) {
				const std::vector<const double*> src_list = m_p_reduction_pool[i][j].src_list;
				n_vectors_size += src_list.capacity() * sizeof(const double*);
				n_vectors_slack += (src_list.capacity() - src_list.size()) * sizeof(const double*);
			}
		}
		return n_size + n_data_size + n_maps_size + n_pools_size + n_vectors_size;
	}

	/**
	 *	@brief gets a temporary vector assigned for reduction
	 *
	 *	@tparam n_dimension is size of the requested vector (must match one of vertex dimensions)
	 *
	 *	@param[in] n_vector_offset is offset in the r.h.s. vector, in elements
	 *
	 *	@return Returns pointer to the assigned memory (guaranteed not to change,
	 *		deleted at the end of the lifetime of this object).
	 *
	 *	@note Note that the reduction conflicts are unchecked here (could have multiple block
	 *		sizes reducing in the same destination or could have overlapping blocks).
	 *	@note This function throws std::bad_alloc.
	 */
	template <const int n_dimension>
	double *p_Get_ReductionBlock(size_t n_vector_offset) // throw(std::bad_alloc)
	{
		typedef fbs_ut::CCTSize<n_dimension> C1DBlockSize;
		typedef CFindTypelistItem<_TyDimensionList, C1DBlockSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to p_GetTempBlock is already linked to use a specific index

		_TyReductionMap &r_reduction_list = m_p_reduction[CSearch::n_index];
		_TyPool &r_pool = m_p_reduction_pool[CSearch::n_index];
		typename _TyReductionMap::iterator p_red_it = r_reduction_list.find(n_vector_offset);
		if(p_red_it == r_reduction_list.end()) {
			r_pool.resize(r_pool.size() + 1);
			TReduction *p_red = &(*(r_pool.end() - 1));
			p_red->n_offset = n_vector_offset; // remember
			_ASSERTE(p_red->src_list.empty()); // should be initialized and empty
			p_red_it = r_reduction_list.insert(std::pair<const size_t,
				TReduction*>(n_vector_offset, p_red)).first;
		}
		TReduction &r_red = *(*p_red_it).second;
		//r_red.src_list.reserve(r_red.src_list.size() + 1); // this is a major bottleneck!!
		CUberBlockMatrix &r_matrix = m_p_matrix[CSearch::n_index];
		double *p_block = r_matrix.p_Get_DenseStorage(n_dimension);
		r_red.src_list.push_back(p_block); // might throw, then the block is orphaned, but we quit anyway

		return p_block; // store result for reduction here
	}

	/**
	 *	@brief reduce all the blocks (runs in parallel, where available)
	 *	@param[out] r_dest is the destination vector (must be allocated by the caller)
	 *	@note This does not check the integrity of the reduction; if initialized
	 *		incorrectly, some parts of the vector can be left uninitialized.
	 */
	void ReduceAll(Eigen::VectorXd &r_dest) const
	{
		_ASSERTE(r_dest.rows() == n_Max_Dimension()); // check the size of the vector
		CTypelistForEach<_TyDimensionList, CReduce>::Run(CReduce(this, r_dest));
	}

	/**
	 *	@brief reduce all the blocks (runs in parallel, where available)
	 *
	 *	@param[out] r_dest is the destination vector (must be allocated by the caller)
	 *	@param[in] n_begin is zero-based index of the first element of the r.h.s. vector to calculate (not vertex id)
	 *	@param[in] n_end is zero-based index of one past the last element of the r.h.s. vector to calculate (not vertex id)
	 *
	 *	@note This does not check if the begin / end boundaries match vertex boundaries. Upper bound function
	 *		is used to find the nearest conservative boundaries (slightly more elements are updated if the
	 *		begin / end is misaligned).
	 *	@note This does not check the integrity of the reduction; if initialized
	 *		incorrectly, some parts of the vector can be left uninitialized.
	 */
	void ReduceRange(Eigen::VectorXd &r_dest, size_t n_begin, size_t n_end) const // ReduceSingle() would be easier to implement and could run in parallel
	{
		_ASSERTE(r_dest.rows() == n_Max_Dimension()); // check the size of the vector
		CTypelistForEach<_TyDimensionList, CReduceRange>::Run(CReduceRange(this, r_dest, n_begin, n_end));
	}

	/**
	 *	@brief reduce a single block
	 *
	 *	@tparam n_dimension is size of the requested vector (must match one of vertex dimensions)
	 *
	 *	@param[out] r_dest is the destination vector (must be allocated by the caller)
	 *	@param[in] n_order is order of the vertex to reduce (offset in the r.h.s. vector, in elements)
	 *
	 *	@note This assumes that a vertex with the given order is in the reduction plan and that the
	 *		dimension is correct (only checked in debug).
	 */
	template <const int n_dimension>
	void Reduce_Single(Eigen::VectorXd &r_dest, size_t n_order) const
	{
		_ASSERTE(r_dest.rows() == n_Max_Dimension()); // check the size of the vector

		typedef fbs_ut::CCTSize<n_dimension> C1DBlockSize;
		typedef CFindTypelistItem<_TyDimensionList, C1DBlockSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to p_GetTempBlock is already linked to use a specific index

		const _TyReductionMap &r_reduction_list = m_p_reduction[CSearch::n_index];
		typename _TyReductionMap::const_iterator p_it = r_reduction_list.find(n_order);
		_ASSERTE(p_it != r_reduction_list.end()); // it should not

		typedef typename Eigen::VectorBlock<Eigen::VectorXd, n_dimension> CDestMap;
		typedef typename CUberBlockMatrix::CMakeMatrixRef<n_dimension, 1>::_TyConst CSrcMap;

		const TReduction &r_red = *(*p_it).second;
		CDestMap dest_map = r_dest.segment<n_dimension>(r_red.n_offset);
		CSrcMap src0_map(r_red.src_list.front());
		dest_map = src0_map; // can this be used? can the first block still use the original block inside the matrix, without having to use a temporary? probably not. todo
		for(size_t j = 1, m = r_red.src_list.size(); j < m; ++ j)
			dest_map += CSrcMap(r_red.src_list[j]);
		// reduce
	}
};

/**
 *	@brief matrix reduction key type selection and traits
 *
 *	For reduced block identification, one can use either pointer to the original
 *	block in lambda or its coordinates. Using coordinates has minor advantages for
 *	the edge system, as most of the edges do not have the pointer to the original
 *	block, instead they store a pointer to a reduced block and getting the original
 *	then requiers <tt>O(log n)</tt> lookup.
 *
 *	@tparam b_use_block_coord_as_reduction_key is key type selector
 */
template <bool b_use_block_coord_as_reduction_key>
class CMatrixReductionKey_Traits {
public:
	typedef const double *TKey; /**< @brief key type */

	/**
	 *	@brief utility function; distills key from block description
	 *
	 *	@param[in] n_row is zero-based block row (unused)
	 *	@param[in] n_col is zero-based block column (unused)
	 *	@param[in] p_address is block address
	 *
	 *	@return Returns key of the given block.
	 */
	static TKey t_MakeKey(size_t UNUSED(n_row), size_t UNUSED(n_col), const double *p_address)
	{
		return p_address;
	}
};

/**
 *	@brief matrix reduction key type selection and traits (specialization for coordinate-based keys)
 */
template <>
class CMatrixReductionKey_Traits<true> {
public:
	typedef std::pair<size_t, size_t> TKey; /**< @brief key type */

	/**
	 *	@brief utility function; distills key from block description
	 *
	 *	@param[in] n_row is zero-based block row
	 *	@param[in] n_col is zero-based block column
	 *	@param[in] p_address is block address (unused)
	 *
	 *	@return Returns key of the given block.
	 */
	static TKey t_MakeKey(size_t n_row, size_t n_col, const double *UNUSED(p_address))
	{
		return std::make_pair(n_row, n_col);
	}
};

/**
 *	@brief parallel reduction plan for efficiently calculating and updating the hessian matrix
 *
 *	@tparam CDimsList is list of hessian matrix block dimensions, as fbs_ut::CCTSize2D
 *
 *	@note This uses block dimensions to differentiate between blocks, assumes that there will
 *		be no conflicts between blocks of different dimensions (does not check for that).
 *	@todo Redesign the pointers to be objects that wrap the pointer and remove the illusion of
 *		being constant (the shared pointers may change upon block conflict without the knowledge
 *		of the object owning it).
 */
template <class CDimsList>
class CMatrixReductionPlan { // todo - need clear!
public:
	typedef typename CTransformTypelist<CDimsList, fbs_ut::CEigenToDimension>::_TyResult _TyDimensionList; /**< @brief list of hessian matrix block dimensions, as fbs_ut::CCTSize2D */

	/**
	 *	@brief parameters, stored as enum
	 */
	enum {
		n_reductor_num = CTypelistLength<_TyDimensionList>::n_result, /**< @brief number of different reduction sizes */
		n_pool_page_size = 4096, /**< @brief reduction pool page size */
		n_pool_memory_align = 0, /**< @brief reduction pool memory alignment */ // see no benefit in alignment right now, this stores the TReduction elements, those do not need to be aligned
		b_use_block_coord_keys = 1 /**< @brief if set, use <tt>(row, col)</tt> coordinates instead of pointers to identify the blocks */
	};

	typedef typename CMatrixReductionKey_Traits<b_use_block_coord_keys != 0>::TKey TKey; /**< @brief block key type */

	/**
	 *	@brief reduction description
	 *	@note This does not store block dimensions; that is given by the index of the pool that this is found in.
	 */
	struct TReduction {
		double *p_dest; /**< @brief destination block in the lambda matrix */
		std::vector<const double*> src_list; /**< @brief list of reduced blocks */
	};

	typedef forward_allocated_pool<TReduction,
		n_pool_page_size, n_pool_memory_align> _TyPool; /**< @brief pool for storing reduction info */ // provides random access and const pointers
	typedef std::map<TKey, TReduction*> _TyReductionMap; /**< @brief map of reductions, ordered by dest block address / coords */ // sorted by dest block address / coords
	typedef std::map<const double*, double**> _TyOwnerLookup; /**< @brief reverse block lookup */ // sorted by dest block address

protected:
	_TyOwnerLookup m_p_owner_lookup[n_reductor_num]; /**< @brief list of reverse block lookups, one per each vertex dimension @note This only contains records for the un-conflicted owners; it eliminates copying the Jacobian contribution from per-edge memory to the matrix (size 1 reduction). @note Only the off-diagonal block owners are stored here. The diagonal ones where conflicts are anticipated do not have size 1 reduction elimination. @note In SLAM, there can be some conflicts in the off-diagonal blocks (multiple edges between the same vertices, e.g. from different sensors, re-observation, etc.). In BA, there typically aren't. */
	_TyPool m_p_reduction_pool[n_reductor_num]; /**< @brief list of reduction pools, one per each vertex dimension */ // actually need that for parallel processing
	_TyReductionMap m_p_reduction_list[n_reductor_num]; /**< @brief list of reductions, one per each vertex dimension */
	CUberBlockMatrix m_p_matrix[n_reductor_num]; /**< @brief list of data stores for the per-edge contributions, one per each vertex dimension */ // storage only (could we do the same with just a pool and an allocator from CUberBlockMatrix, that would be better, p_Get_DenseStorage() could be protected again)

	/**
	 *	@brief reduction function object
	 */
	class CReduce {
	protected:
		const CMatrixReductionPlan<CDimsList> *m_p_this; /**< @brief pointer to the parent reduction plan */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_this is pointer to the parent reduction plan
		 */
		inline CReduce(const CMatrixReductionPlan<CDimsList> *p_this)
			:m_p_this(p_this)
		{}

		/**
		 *	@brief function operator; performs all the reductions of a single Jacobian dimension
		 *	@tparam C2DSize is Jacobian size (a specialization of fbs_ut::CCTSize2D)
		 */
		template <class C2DSize>
		inline void operator ()()
		{
			typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
			typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
			// the search happens at compile-time, each call to t_GetTempBlock is already linked to use a specific index

			typedef typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num,
				C2DSize::n_column_num>::_Ty CDestMap;
			typedef typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num,
				C2DSize::n_column_num>::_TyConst CSrcMap;

			const _TyPool &reduction_pool = m_p_this->m_p_reduction_pool[CSearch::n_index];
			size_t _n = reduction_pool.size();
			_ASSERTE(_n <= INT_MAX);
			int n = int(_n);
			#pragma omp parallel for if(n > 50) // todo - export the parallel thresh as an arg, maybe consider dynamic schedule
			for(int i = 0; i < n; ++ i) {
				const TReduction &r_red = reduction_pool[i];
				CDestMap dest_map((double*)r_red.p_dest);
				CSrcMap src0_map(r_red.src_list.front());
				dest_map = src0_map; // can this be used? can the first block still use the original block inside the matrix, without having to use a temporary? probably not. todo
				for(size_t j = 1, m = r_red.src_list.size(); j < m; ++ j)
					dest_map += CSrcMap(r_red.src_list[j]);
				// reduce
			}
		}
	};

public:
	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		size_t n_size = sizeof(CMatrixReductionPlan<CDimsList>);
		size_t n_data_size = 0, n_maps_size = 0, n_pools_size = 0,
			n_vectors_size = 0, n_vectors_slack = 0;
		for(int i = 0; i < n_reductor_num; ++ i) {
			n_data_size += m_p_matrix[i].n_Allocation_Size() - sizeof(CUberBlockMatrix);
			n_maps_size += n_Map_Allocation_Size(m_p_owner_lookup[i]) - sizeof(m_p_owner_lookup[i]);
			n_maps_size += n_Map_Allocation_Size(m_p_reduction_list[i]) - sizeof(m_p_reduction_list[i]);
			n_pools_size += m_p_reduction_pool[i].capacity() * sizeof(TReduction) +
				m_p_reduction_pool[i].page_num() * sizeof(TReduction*);
			for(size_t j = 0, m = m_p_reduction_pool[i].size(); j < m; ++ j) {
				const std::vector<const double*> src_list = m_p_reduction_pool[i][j].src_list;
				n_vectors_size += src_list.capacity() * sizeof(const double*);;
				n_vectors_slack += (src_list.capacity() - src_list.size()) * sizeof(const double*);
			}
		}
		return n_size + n_data_size + n_maps_size + n_pools_size + n_vectors_size;
	}

	/**
	 *	@brief gets a single reduction block in case we know it is most likely going to be conflicted (like the blocks at the diagonal)
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *
	 *	@param[in] n_row is zero-based block row
	 *	@param[in] n_col is zero-based block column
	 *	@param[in] p_reduction_dest is pointer to the first element of the block inside the matrix
	 *
	 *	@return Returns pointer to a temporary block from which the values will be reduced to the specified destination.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class C2DSize>
	inline double *p_Diagonal_GetTempBlock(size_t n_row, size_t n_col, double *p_reduction_dest) // throw(std::bad_alloc)
	{
		//_ASSERTE(n_row == n_col); // it does not have to be diagonal, it is just a type of block where collision is anticipated in most of the blocks

		TKey t_key = t_MakeKey(n_row, n_col, p_reduction_dest);
		typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to p_GetTempBlock is already linked to use a specific index

		_TyReductionMap &r_reduction_list = m_p_reduction_list[CSearch::n_index];
		CUberBlockMatrix &r_storage = m_p_matrix[CSearch::n_index];
		_TyPool &r_pool = m_p_reduction_pool[CSearch::n_index];
		return p_GetSingle(r_reduction_list, r_storage, r_pool, C2DSize::n_row_num *
			C2DSize::n_column_num, t_key, p_reduction_dest); // just push those three on the stack and go
	}

	/**
	 *	@brief gets a single reduction block in case we know it is most likely going to be unique (like the off-diagonal blocks)
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *
	 *	@param[in] p_reduction_dest is pointer to the first element of the block inside the matrix
	 *	@param[in] p_owner_storage is pointer to the pointer to address of the block inside the
	 *		owner object (is liable to be changed upon conflict)
	 *
	 *	@return Returns pointer to the original block (no conflict occurred yet).
	 *
	 *	@note This should be used in case the block did not exist in the matrix (<tt>b_uninitialized</tt> is set).
	 *	@note This assumes that there is only a single pointer to the block stored, which can be replaced
	 *		by the pointer to a pointer, passed as the second argument. In case this does not apply, this
	 *		reductor cannot be used.
	 *	@note This function throws std::bad_alloc.
	 */
	template <class C2DSize>
	inline double *p_OffDiagonal_GetTempBlock(double *p_reduction_dest, double **p_owner_storage) // throw(std::bad_alloc)
	{
#ifdef _DEBUG
		typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to p_GetTempBlock is already linked to use a specific index

		_TyReductionMap &r_reduction_list = m_p_reduction_list[CSearch::n_index];
		_ASSERTE(b_use_block_coord_keys || r_reduction_list.find(t_MakeKey(0,
			0, p_reduction_dest)) == r_reduction_list.end()); // if b_use_block_coord_keys is set, we can't verify anything
		// make sure the block is not there
#endif // _DEBUG

		Set_BlockOwner<C2DSize>(p_reduction_dest, p_owner_storage);
		return p_reduction_dest;
	}

	/**
	 *	@brief gets a single reduction block in case we know it is most likely going to be unique (like the off-diagonal blocks)
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *
	 *	@param[in] n_row is zero-based block row
	 *	@param[in] n_col is zero-based block column
	 *	@param[in] p_reduction_dest is pointer to the first element of the block inside the matrix
	 *
	 *	@return Returns pointer to a temporary block from which the values will be reduced to the specified destination.
	 *
	 *	@note This should be used in case the block did already exist in the matrix (<tt>b_uninitialized</tt> is not set).
	 *	@note This function throws std::bad_alloc.
	 */
	template <class C2DSize>
	inline double *p_OffDiagonal_GetTempBlock(size_t n_row, size_t n_col, double *p_reduction_dest)
	{
		TKey t_key = t_MakeKey(n_row, n_col, p_reduction_dest);
		std::pair<double*, double*> t_storage = t_GetTempBlock<C2DSize>(t_key, p_reduction_dest);
		if(t_storage.second) {
			double **p_owner_variable = p_Get_BlockOwner<C2DSize>(p_reduction_dest, true);
			_ASSERTE(p_owner_variable != 0);
			// we are replacing the owner, there should be one registered

			*p_owner_variable = t_storage.first;
			// the original owner should use the first temp block

			memcpy(t_storage.first, p_reduction_dest, C2DSize::n_row_num *
				C2DSize::n_column_num * sizeof(double));
			// this block might already have significant value, need to store it in the temp block

			return t_storage.second;
			// the second reducer should use the second temp block
		} else {
			_ASSERTE(!p_Get_BlockOwner<C2DSize>(p_reduction_dest, false)); // there is no owner anymore, we already replaced it
			return t_storage.first;
			// nothing special, just another block
		}
	}

	/**
	 *	@brief reduce all the blocks (runs in parallel, where available)
	 */
	void ReduceAll() const
	{
		/*eigen::MatrixXd::identity();
		for(size_t i = 0; i < n_reductor_num; ++ i) { // todo - do typelist_foreach first, to have fixed block size ;) (will probably lose for loop scheduling though)
			_TyReductionMap &r_reduction_list = m_p_reduction_list[i];
			_TyReductionMap::iterator p_it = r_reduction_list.begin();
			_TyReductionMap::iterator p_end_it = r_reduction_list.end();
			for(; p_it != p_end_it; ++ p_it) { // duh; how to parallelize that? can't.
			}
		}*/
		// can do everything in parallel, need to see which strategy is the fastest

		CTypelistForEach<_TyDimensionList, CReduce>::Run(CReduce(this));
	}

	/**
	 *	@brief reduce a single block
	 *
	 *	Reduce a single block address, this is callable by an edge, probably does
	 *	good enough job in incremental updates where a single edge / a couple edges
	 *	is added. It will recalculate a couple of vertex' sums, but it should still
	 *	be less than all of them, as it is now.
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *	@param[in] t_key is block key (address / row col coordinate)
	 *	@note Note that this can not run in parallel over the edges. to do it in parallel,
	 *		one would need to collect the *unique* block addresses and then run in parallel.
	 */
	template <class C2DSize>
	void ReduceSingle(TKey t_key) const
	{
		typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to p_GetTempBlock is already linked to use a specific index

		const _TyReductionMap &r_reduction_list = m_p_reduction_list[CSearch::n_index];
		// get the list

		typename _TyReductionMap::const_iterator p_red_it = r_reduction_list.find(t_key);
		if(p_red_it == r_reduction_list.end())
			return; // the block is not there (may happen with off-diagonal blocks, which do not have a conflict yet and do not need to be reduced)

		const TReduction &r_red = *(*p_red_it).second;
		_ASSERTE(!r_red.src_list.empty()); // should never be (but there can be 1, in case of a lonely vertex where collision was expected but did not occur (yet))
		// get the particular reduction

		typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num,
			C2DSize::n_column_num>::_Ty dest_map((double*)r_red.p_dest);
		typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num,
			C2DSize::n_column_num>::_TyConst src0_map(r_red.src_list.front());
		dest_map = src0_map;
		for(size_t i = 1, n = r_red.src_list.size(); i < n; ++ i) {
			typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num,
				C2DSize::n_column_num>::_TyConst src_map(r_red.src_list[i]);
			dest_map += src_map;
		}
		// reduce
	}

	/**
	 *	@brief utility function for making key
	 *
	 *	@param[in] n_row is zero-based block row
	 *	@param[in] n_col is zero-based block column
	 *	@param[in] p_address is block address
	 *
	 *	@return Returns key of the given block.
	 *
	 *	@note Depending on which type of key is used, some of the arguments are ignored.
	 */
	static inline TKey t_MakeKey(size_t n_row, size_t n_col, const double *p_address)
	{
		return CMatrixReductionKey_Traits<b_use_block_coord_keys != 0>::t_MakeKey(n_row, n_col, p_address);
	}

protected:
	/**
	 *	@brief gets one or two blocks, based on whether there would be a conflict
	 *
	 *	Get a temp block, and in case p_reduction_dest was not present in the list yet,
	 *	alloc also the second block for the original owner of p_reduction_dest (which
	 *	wrote directly to the matrix, bypassing the reduction).
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *
	 *	@param[in] t_key is block key (address / row col coordinate)
	 *	@param[in] p_reduction_dest is pointer to the first element of the block inside the matrix
	 *
	 *	@return Returns a pair of reduced block storages, first is always a valid pointer and
	 *		second may be set to null if the block was already shared before.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class C2DSize>
	inline std::pair<double*, double*> t_GetTempBlock(TKey t_key, double *p_reduction_dest) // throw(std::bad_alloc)
	{
		typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to t_GetTempBlock is already linked to use a specific index

		_TyReductionMap &r_reduction_list = m_p_reduction_list[CSearch::n_index];
		CUberBlockMatrix &r_storage = m_p_matrix[CSearch::n_index];
		_TyPool &r_pool = m_p_reduction_pool[CSearch::n_index];
		return t_GetPair(r_reduction_list, r_storage, r_pool, C2DSize::n_row_num *
			C2DSize::n_column_num, t_key, p_reduction_dest); // just push those three on the stack and go
		//_ASSERTE(t_pair.second && (m_p_owner_lookup[CSearch::n_index].find(p_reduction_dest) !=
		//	m_p_owner_lookup[CSearch::n_index].end())); // if there is second, there was a reduction conflict and the block should be owned (but not vice versa)
		//return t_pair;
	}

	/**
	 *	@brief sets pointer to the block owner pointer storage
	 *
	 *	This is used to bypass size 1 reductions in the blocks where the conflicts are unlikely.
	 *	This adds a record of the original pointer in the matrix, and a pointer to the pointer
	 *	to this block in the edge class, so that when a conflict occurs, the pointer in the original
	 *	owner edge can be modified. This mandates that pointers to the blocks be stored only in
	 *	a single instance (as multiple pointer instances can't be modified like this).
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *
	 *	@param[in] p_block_addr is pointer to the first element of the block inside the matrix
	 *	@param[in] p_owner is pointer to the pointer to address of the block inside the
	 *		owner object (is liable to be changed upon conflict)
	 *
	 *	@note This should be used in case the block did not exist in the matrix (<tt>b_uninitialized</tt> is set).
	 *	@note This assumes that there is only a single pointer to the block stored, which can be replaced
	 *		by the pointer to a pointer, passed as the second argument. In case this does not apply, this
	 *		reductor cannot be used.
	 *	@note This function throws std::bad_alloc.
	 */
	template <class C2DSize>
	void Set_BlockOwner(const double *p_block_addr, double **p_owner) // throw(std::bad_alloc)
	{
		typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to t_GetTempBlock is already linked to use a specific index

		_TyOwnerLookup &r_lookup_map = m_p_owner_lookup[CSearch::n_index];
		_ASSERTE(r_lookup_map.find(p_block_addr) == r_lookup_map.end()); // make sure that there is no owner so far
		r_lookup_map[p_block_addr] = p_owner;
		// stores the original block woner
	}

	/**
	 *	@brief gets a pointer to block owner pointer
	 *
	 *	@tparam C2DSize is block size as fbs_ut::CCTSize2D
	 *
	 *	@param[in] p_block_addr is pointer to the first element of the block inside the matrix
	 *	@param[in] b_clear_owner is clear owner flag (if set, and the order exists, it is )
	 *
	 *	@return Returns pointer to the pointer inside the owner of the specified block, or null if the
	 *		block does not have an owner (or has multiple owners and the first owner was cleared).
	 */
	template <class C2DSize>
	double **p_Get_BlockOwner(const double *p_block_addr, bool b_clear_owner = true)
	{
		typedef CFindTypelistItem<_TyDimensionList, C2DSize> CSearch; // look for the size
		typedef typename CReductionPlanAssert<CSearch::b_result>::BLOCK_SIZE_NOT_PRESENT_IN_THE_LIST _TyAssert0; // make sure it is there
		// the search happens at compile-time, each call to t_GetTempBlock is already linked to use a specific index

		_TyOwnerLookup &r_lookup_map = m_p_owner_lookup[CSearch::n_index];
		_TyOwnerLookup::iterator p_it = r_lookup_map.find(p_block_addr);
		if(p_it == r_lookup_map.end())
			return 0; // no such owner
		double **p_owner = (*p_it).second;
		if(b_clear_owner)
			r_lookup_map.erase(p_it); // remove, now all edges will use the reductor anyway
		return p_owner;
		// find the owner
	}

	/**
	 *	@brief gets a single reduced block
	 *
	 *	@param[in,out] r_reduction_list is reduction map
	 *	@param[in,out] r_storage is matrix where the reduced blocks are allocated
	 *	@param[in,out] r_pool is a pool for reduction description objects
	 *	@param[in] n_size is size of the requested block, in elements
	 *	@param[in] t_key is block key (address / row col coordinate)
	 *	@param[in] p_reduction_dest is pointer to the first element of the block inside the matrix
	 *
	 *	@return Returns a pointer to reduced block storage.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	double *p_GetSingle(_TyReductionMap &r_reduction_list, CUberBlockMatrix &r_storage,
		_TyPool &r_pool, int n_size, TKey t_key, double *p_reduction_dest) // throw(std::bad_alloc)
	{
		typename _TyReductionMap::iterator p_red_it = r_reduction_list.find(t_key);
		if(p_red_it == r_reduction_list.end()) {
			r_pool.resize(r_pool.size() + 1);
			TReduction *p_red = &(*(r_pool.end() - 1));
			p_red->p_dest = p_reduction_dest; // remember
			_ASSERTE(p_red->src_list.empty()); // should be initialized and empty
			p_red_it = r_reduction_list.insert(std::pair<const TKey,
				TReduction*>(t_key, p_red)).first;
		}
		std::vector<const double*> &r_list = (*p_red_it).second->src_list;
		double *p_ptr = r_storage.p_Get_DenseStorage(n_size);
		r_list.push_back(p_ptr);
		return p_ptr;
	}

	/**
	 *	@brief gets one or two blocks, based on whether there would be a conflict
	 *
	 *	@param[in,out] r_reduction_list is reduction map
	 *	@param[in,out] r_storage is matrix where the reduced blocks are allocated
	 *	@param[in,out] r_pool is a pool for reduction description objects
	 *	@param[in] n_size is size of the requested block, in elements
	 *	@param[in] t_key is block key (address / row col coordinate)
	 *	@param[in] p_reduction_dest is pointer to the first element of the block inside the matrix
	 *
	 *	@return Returns a pair of reduced block storages, first is always a valid pointer and
	 *		second may be set to null if the block was already shared before.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	std::pair<double*, double*> t_GetPair(_TyReductionMap &r_reduction_list,
		CUberBlockMatrix &r_storage, _TyPool &r_pool, int n_size, TKey t_key, double *p_reduction_dest) // throw(std::bad_alloc)
	{
		typename _TyReductionMap::iterator p_red_it = r_reduction_list.find(t_key);
		if(p_red_it == r_reduction_list.end()) {
			r_pool.resize(r_pool.size() + 1);
			TReduction *p_red = &(*(r_pool.end() - 1));
			p_red->p_dest = p_reduction_dest; // remember
			_ASSERTE(p_red->src_list.empty()); // should be initialized and empty
			p_red_it = r_reduction_list.insert(std::pair<const TKey,
				TReduction*>(t_key, p_red)).first;
		}
		std::vector<const double*> &r_list = (*p_red_it).second->src_list;
		if(!r_list.empty()) {
			double *p_ptr = r_storage.p_Get_DenseStorage(n_size);
			r_list.push_back(p_ptr);
			return std::make_pair(p_ptr, (double*)0);
			// just another block
		} else {
			double *p_ptr0 = r_storage.p_Get_DenseStorage(n_size);
			double *p_ptr1 = r_storage.p_Get_DenseStorage(n_size); // must call twice, want each of them aligned
			r_list.push_back(p_ptr0);
			r_list.push_back(p_ptr1);
			return std::make_pair(p_ptr0, p_ptr1);
			// the first time around: alloc two blocks
		}
	}
};

/**
 *	@brief wrapper reduction plans for lambda and the right-hand-side vector
 *
 *	@tparam CDimsList is list of hessian matrix block dimensions, as fbs_ut::CCTSize2D
 */
template <class CDimsList>
class CLambdaReductionPlan {
public:
	typedef CVectorReductionPlan<CDimsList> CRHSReductor; /**< @brief right hand side vector reduction plan type */
	typedef CMatrixReductionPlan<CDimsList> CLambdaReductor; /**< @brief lambda reduction plan type */

protected:
	CVectorReductionPlan<CDimsList> m_vec_plan; /**< @brief right hand side vector reduction plan */
	CMatrixReductionPlan<CDimsList> m_mat_plan; /**< @brief lambda reduction plan */

public:
	/**
	 *	@brief gets right hand side vector reduction plan
	 *	@return Returns a reference to the right hand side vector reduction plan.
	 */
	inline CRHSReductor &r_RHS_ReductionPlan()
	{
		return m_vec_plan;
	}

	/**
	 *	@brief gets lambda reduction plan
	 *	@return Returns a reference to the lambda reduction plan.
	 */
	inline CLambdaReductor &r_Lambda_ReductionPlan()
	{
		return m_mat_plan;
	}

	/**
	 *	@brief gets right hand side vector reduction plan
	 *	@return Returns a const reference to the right hand side vector reduction plan.
	 */
	inline const CRHSReductor &r_RHS_ReductionPlan() const
	{
		return m_vec_plan;
	}

	/**
	 *	@brief gets lambda reduction plan
	 *	@return Returns a const reference to the lambda reduction plan.
	 */
	inline const CLambdaReductor &r_Lambda_ReductionPlan() const
	{
		return m_mat_plan;
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		return m_vec_plan.n_Allocation_Size() + m_mat_plan.n_Allocation_Size();
	}
};

/**
 *	@brief v1 lambda solver utility function
 *	@tparam CDimsList is list of lambda matrix block sizes, as fbs_ut::CCTSize2D
 */
template <class CDimsList>
class CLambdaOps : public nonlinear_detail::CSolverOps_Base {
public:
	typedef CDimsList _TyLambdaMatrixBlockSizes; /**< @brief list of lambda matrix block sizes, as fbs_ut::CCTSize2D */
	struct _TyReductionPlan {}; /**< @brief reduction plan type (an empty class) */ // the v1 lambda did not need reduction plan, it was stored in the edges / vertices

	/**
	 *	@brief function object that calls lambda hessian block allocation for all edges
	 */
	class CAlloc_HessianBlocks {
	protected:
		CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_lambda is reference to the lambda matrix
		 */
		inline CAlloc_HessianBlocks(CUberBlockMatrix &r_lambda)
			:m_r_lambda(r_lambda)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertexOrEdge is vertex or edge type
		 *	@param[in,out] r_vertex_or_edge is vertex or edge to have hessian blocks allocated in lambda
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyVertexOrEdge>
		inline void operator ()(_TyVertexOrEdge &r_vertex_or_edge) // throw(std::bad_alloc)
		{
			r_vertex_or_edge.Alloc_HessianBlocks(m_r_lambda);
		}
	};

	/**
	 *	@brief function object that calculates hessians in all the edges
	 */
	class CCalculate_Hessians {
	public:
		/**
		 *	@brief function operator
		 *	@tparam _TyVertexOrEdge is vertex or edge type
		 *	@param[in] r_t_vertex_or_edge is vertex or edge to update it's hessians
		 */
		template <class _TyVertexOrEdge>
		inline void operator ()(_TyVertexOrEdge &r_t_vertex_or_edge) const
		{
			r_t_vertex_or_edge.Calculate_Hessians();
		}
	};

	/**
	 *	@brief function object that calls b vector calculation for all edges
	 */
	class CCollect_RightHandSide_Vector {
	protected:
		Eigen::VectorXd &m_r_b; /**< @brief reference to the right-hand side vector (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_b is reference to the right-hand side vector
		 */
		inline CCollect_RightHandSide_Vector(Eigen::VectorXd &r_b)
			:m_r_b(r_b)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyVertex is vertex type
		 *	@param[in,out] r_t_vertex is a vertex to output its part R error vector
		 */
		template <class _TyVertex>
		inline void operator ()(const _TyVertex &r_t_vertex) // throw(std::bad_alloc)
		{
			r_t_vertex.Get_RightHandSide_Vector(m_r_b);
		}
	};

public:
	/**
	 *	@brief incrementally updates the lambda matrix structure (can be empty)
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in,out] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan (unused in v1)
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *	@param[in] n_vertices_already_in_lambda is number of vertices which are already in the matrix
	 *	@param[in] n_edges_already_in_lambda is number of edges which are already in the matrix
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CSystem>
	static inline void Extend_Lambda(CSystem &r_system, _TyReductionPlan &UNUSED(r_reduction_plan),
		CUberBlockMatrix &r_lambda, size_t n_vertices_already_in_lambda, size_t n_edges_already_in_lambda) // throw(std::bad_alloc)
	{
		if(!n_vertices_already_in_lambda && !n_edges_already_in_lambda)
			AddEntriesInSparseSystem(r_system, r_lambda); // works for empty
		else
			UpdateSparseSystem(r_system, r_lambda, n_vertices_already_in_lambda, n_edges_already_in_lambda); // does not work for empty
		// create block matrix lambda

#ifdef _DEBUG
		/*{
			CUberBlockMatrix A;
			const Eigen::MatrixXd &r_t_uf = r_system.r_t_Unary_Factor();
			if(!A.Append_Block(r_t_uf, 0, 0))
				throw std::bad_alloc();
			// add unary factor

			r_system.r_Edge_Pool().For_Each(CAlloc_JacobianBlocks(A));
			// add all the hessian blocks

			CUberBlockMatrix lambda_ref;
			A.PreMultiplyWithSelfTransposeTo(lambda_ref, true); // only upper diag!
			// calculate lambda = AtA

			if(!r_lambda.b_EqualStructure(lambda_ref)) {
				lambda_ref.Rasterize("lambda1_reference_structure.tga");
				r_lambda.Rasterize("lambda0_structure.tga");
			}

			_ASSERTE(r_lambda.b_EqualStructure(lambda_ref));
			// make sure the matrix has the same structure
		}*/
#endif // _DEBUG
	}

	/**
	 *	@brief refreshes the lambda matrix by recalculating edge hessians
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in,out] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan (unused in v1)
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *	@param[in] n_referesh_from_vertex is zero-based index of the first vertex to refresh
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge to refresh
	 *
	 *	@note This throws std::bad_alloc and std::runtime_error;
	 */
	template <class CSystem>
	static inline void Refresh_Lambda(CSystem &r_system, _TyReductionPlan &UNUSED(r_reduction_plan),
		CUberBlockMatrix &r_lambda, size_t n_referesh_from_vertex = 0, size_t n_refresh_from_edge = 0) // throw(std::bad_alloc, std::runtime_error)
	{
		if(n_refresh_from_edge) {
			r_system.r_Edge_Pool().For_Each_Parallel(n_refresh_from_edge,
				r_system.r_Edge_Pool().n_Size(), CCalculate_Hessians());
		} else {
			r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Hessians());
		}
		if(n_referesh_from_vertex) {
			r_system.r_Vertex_Pool().For_Each_Parallel(n_referesh_from_vertex,
				r_system.r_Vertex_Pool().n_Size(), CCalculate_Hessians());
		} else {
			r_system.r_Vertex_Pool().For_Each_Parallel(CCalculate_Hessians());
		}
		// can do this in parallel

		if(!CSystem::null_UnaryFactor && !n_referesh_from_vertex) {
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_first_vertex_id = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Edge_Pool().b_Empty());
			size_t n_first_vertex_id = r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Vertex_Pool()[n_first_vertex_id].b_IsConstant()); // this one must not be constant
			size_t n_first_vertex_order = r_system.r_Vertex_Pool()[n_first_vertex_id].n_Order();
			// get id of the first vertex (usually zero)

			const Eigen::MatrixXd &r_t_uf = r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
			r_lambda.t_FindBlock(n_first_vertex_order, n_first_vertex_order).noalias() += r_t_uf.transpose() * r_t_uf;
		}
		// add unary factor (gets overwritten by the first vertex' block)
#ifdef _DEBUG
		/*{
			CUberBlockMatrix A;
			const Eigen::MatrixXd &r_t_uf = r_system.r_t_Unary_Factor();
			if(!A.Append_Block(r_t_uf, 0, 0))
				throw std::bad_alloc();
			// add unary factor

			r_system.r_Edge_Pool().For_Each(CAlloc_JacobianBlocks(A));
			// add all the hessian blocks

			r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Jacobians());
			// calculate the values as well

			CUberBlockMatrix lambda_ref;
			A.PreMultiplyWithSelfTransposeTo(lambda_ref, true); // only upper diag!
			// calculate lambda = AtA

			if(!r_lambda.b_Equal(lambda_ref, 1e-3)) {
				r_lambda.Rasterize("lambda2_values.tga");
				lambda_ref.Rasterize("lambda3_reference_values.tga");
				CUberBlockMatrix &diff = lambda_ref;
				r_lambda.AddTo(diff, -1);
				diff.Rasterize("lambda4_diff_values.tga");
				fprintf(stderr, "error: lambda and it's reference have different value\n");
				exit(-1);
			}

			_ASSERTE(r_lambda.b_EqualStructure(lambda_ref));
			// make sure the matrix has the same structure
		}*/
#endif // _DEBUG
	}

	/**
	 *	@brief calculates the right-hand side vector
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan (unused in v1)
	 *	@param[in,out] r_v_b is reference to the r.h.s. vector (needs to be
	 *		allocated by the caller to the appropriate dimension)
	 */
	template <class CSystem>
	static inline void Collect_RightHandSide_Vector(const CSystem &r_system,
		const _TyReductionPlan &UNUSED(r_reduction_plan), Eigen::VectorXd &r_v_b)
	{
		r_system.r_Vertex_Pool().For_Each_Parallel(CCollect_RightHandSide_Vector(r_v_b)); // can do this in parallel
		// collect b
	}

	/**
	 *	@brief calculates a segment of the right-hand side vector, corresponding to a range of vertices
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan (unused in v1)
	 *	@param[in,out] r_v_b is reference to the r.h.s. vector (needs to be
	 *		allocated by the caller to the appropriate dimension)
	 *	@param[in] n_begin is zero-based index of the first vertex to calculate the r.h.s. for
	 *	@param[in] n_end is zero-based index of one past the last vertex to calculate the r.h.s. for
	 */
	template <class CSystem>
	static inline void Collect_RightHandSide_Vector(const CSystem &r_system,
		const _TyReductionPlan &UNUSED(r_reduction_plan), Eigen::VectorXd &r_v_b, size_t n_begin, size_t n_end)
	{
		r_system.r_Vertex_Pool().For_Each_Parallel(n_begin, n_end,
			CCollect_RightHandSide_Vector(r_v_b)); // can do this in parallel
		// collect b
	}

	/**
	 *	@brief calculates a segment of the right-hand side vector, corresponding to a single vertex
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan (unused in v1)
	 *	@param[in,out] r_v_b is reference to the r.h.s. vector (needs to be
	 *		allocated by the caller to the appropriate dimension)
	 *	@param[in] n_vertex is zero-based index of the vertex to calculate the r.h.s. for
	 */
	template <class CSystem>
	static inline void Collect_RightHandSide_Vector(const CSystem &r_system,
		const _TyReductionPlan &r_reduction_plan, Eigen::VectorXd &r_v_b, size_t n_vertex)
	{
		r_system.r_Vertex_Pool().For_Each(n_vertex, n_vertex + 1,
			CCollect_RightHandSide_Vector(r_v_b)); // this may not be the most efficient way
		// collect b
	}

protected:
	/**
	 *	@brief creates the lambda matrix from scratch
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in,out] r_system is optimized system
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	template <class CSystem>
	static inline void AddEntriesInSparseSystem(CSystem &r_system, CUberBlockMatrix &r_lambda) // throw(std::bad_alloc, std::runtime_error)
	{
#if 0
		if(r_system.r_Edge_Pool().n_Size() > 1000) { // wins 2.42237 - 2.48938 = .06701 seconds on 10k.graph, likely more on larger graphs
			//printf("building large matrix from scratch ...\n"); // debug
			std::vector<size_t> row_cumsum_list(r_system.r_Edge_Pool().n_Size());
			/*std::vector<size_t>::iterator p_end_it =*/
				r_system.r_Edge_Pool().For_Each(CGetCumsums(row_cumsum_list));
			//_ASSERTE(p_end_it == row_cumsum_list.end());
			// collect cumsums

			CUberBlockMatrix tmp(row_cumsum_list.begin(),
				row_cumsum_list.end(), r_system.r_Vertex_Pool().n_Size());
			r_lambda.Swap(tmp);
			// use this one instead

			// todo - see if there are some row_reindex on 100k, fix it by collecting
			// cumsums and building matrix with that (proven to be faster before)
		} else
#endif // 0 // todo - need to write function that gets cumsums from vertices (it's not difficult)
		{
			//printf("building small matrix from scratch ...\n"); // debug
			r_lambda.Clear();
			// ...
		}

		if(!CSystem::null_UnaryFactor) {
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_first_vertex_id = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Edge_Pool().b_Empty());
			size_t n_first_vertex_id = r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Vertex_Pool()[n_first_vertex_id].b_IsConstant()); // this one must not be constant
			size_t n_first_vertex_order = r_system.r_Vertex_Pool()[n_first_vertex_id].n_Order();
			// get id of the first vertex (usually zero)

			const Eigen::MatrixXd &r_t_uf = r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
			if(!r_lambda.Append_Block(Eigen::MatrixXd(r_t_uf.transpose() * r_t_uf),
			   n_first_vertex_order, n_first_vertex_order))
				throw std::bad_alloc();
		}
		// add unary factor (actually UF^T * UF, it was square-rooted before)

		// note that the unary error cannot be easily added without introducing a dummy
		// edge that would add itself as a reference to vertex 0
		if(r_system.r_v_Unary_Error().squaredNorm() > 0)
			throw std::runtime_error("unary error is not supported by the v1 reduction plan");
		// this is slightly obsolete so we will not support it for now

		r_system.r_Edge_Pool().For_Each(CAlloc_HessianBlocks(r_lambda));
		r_system.r_Vertex_Pool().For_Each(CAlloc_HessianBlocks(r_lambda));
		// add all the hessian blocks

		//printf("building lambda from scratch finished\n"); // debug
	}

	/**
	 *	@brief incrementally updates the lambda matrix structure (must not be empty)
	 */
	template <class CSystem>
	static inline void UpdateSparseSystem(CSystem &r_system,
		CUberBlockMatrix &r_lambda, size_t n_skip_vertices, size_t n_skip_edges) // throw(std::bad_alloc)
	{
		_ASSERTE(r_lambda.n_Row_Num() > 0 && r_lambda.n_Column_Num() == r_lambda.n_Row_Num()); // make sure lambda is not empty
		r_system.r_Edge_Pool().For_Each(n_skip_edges,
			r_system.r_Edge_Pool().n_Size(), CAlloc_HessianBlocks(r_lambda));
		r_system.r_Vertex_Pool().For_Each(n_skip_vertices,
			r_system.r_Vertex_Pool().n_Size(), CAlloc_HessianBlocks(r_lambda));
		// add the hessian blocks of the new edges
	}
};

/**
 *	@brief v2 lambda solver utility function
 *	@tparam CDimsList is list of lambda matrix block sizes, as fbs_ut::CCTSize2D
 */
template <class CDimsList>
class CLambdaOps2 : public nonlinear_detail::CSolverOps_Base {
public:
	typedef CDimsList _TyLambdaMatrixBlockSizes; /**< @brief list of lambda matrix block sizes, as fbs_ut::CCTSize2D */
	typedef CLambdaReductionPlan<CDimsList> _TyReductionPlan; /**< @brief reduction plan type */

	/**
	 *	@brief function object that calls lambda hessian block allocation for all edges
	 */
	class CAlloc_HessianBlocks_v2 {
	protected:
		CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix (out) */
		_TyReductionPlan &m_r_redplan; /**< @brief reference to the reduction plan */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in,out] r_lambda is reference to the lambda matrix (modified once the function operator is invoked)
		 *	@param[in,out] r_redplan is reference to the reduction plan (modified once the function operator is invoked)
		 */
		inline CAlloc_HessianBlocks_v2(CUberBlockMatrix &r_lambda, _TyReductionPlan &r_redplan)
			:m_r_lambda(r_lambda), m_r_redplan(r_redplan)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to have hessian blocks allocated in lambda
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) // throw(std::bad_alloc)
		{
			r_t_edge.Alloc_HessianBlocks_v2(m_r_lambda, m_r_redplan);
		}
	};

	/**
	 *	@brief function object that calculates hessians in all the edges
	 */
	class CCalculate_Hessians_v2 {
	public:
		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in] r_t_edge is edge to update it's hessians
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) const
		{
			r_t_edge.Calculate_Hessians_v2();
		}
	};

	/**
	 *	@brief function object that calculates hessians in the selected edges
	 */
	class CUpdate_Hessians_v2 {
	protected:
		const CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix */
		_TyReductionPlan &m_r_redplan; /**< @brief reference to the reduction plan */
		bool m_b_recalc; /**< @brief Jacobian recalculation flag */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_lambda is reference to the lambda matrix
		 *	@param[in,out] r_redplan is reference to the reduction plan (executed once the function operator is invoked)
		 *	@param[in] b_recalc is Jacobian recalculation flag (if set, the edge Jacobians are recalculated prior to the reduction)
		 */
		inline CUpdate_Hessians_v2(const CUberBlockMatrix &r_lambda, _TyReductionPlan &r_redplan, bool b_recalc)
			:m_r_lambda(r_lambda), m_r_redplan(r_redplan), m_b_recalc(b_recalc)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in] r_t_edge is edge to update it's hessians
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) const
		{
			if(m_b_recalc) // may choose to do that in parallel earlier
				r_t_edge.Calculate_Hessians_v2(); // calculate
			r_t_edge.Reduce_Hessians_v2(m_r_lambda, m_r_redplan); // reduce (may overwrite an earlier reduce
			// of a shared jacobian, but that one was incomplete because r_t_edge.Calculate_Hessians_v2()
			// was not called yet on this edge)
		}
	};

	/**
	 *	@brief unary factor helper
	 *
	 *	The size of UF depends on the dimension of the first vertex (unknown),
	 *	it needs to be found at runtime in the block size typelist. This is a
	 *	callback for CTypelistItemBFind.
	 */
	class CAddUnaryFactor {
	protected:
		_TyReductionPlan &m_r_rp; /**< @brief reference to lambda and RHS reductor */
		const Eigen::MatrixXd &m_r_t_uf; /**< @brief const reference to the unary factor */
		const Eigen::VectorXd &m_r_t_uerr; /**< @brief const reference to the unary error */
		double *m_p_uf_block; /**< @brief pointer to the hessian block of the first vertex in lambda */
		size_t m_n_vertes_id; /**< @brief id of the anchor vertex the unary factor will be applied to */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] r_rp is reference to lambda / RHS reductors
		 *	@param[in] r_t_uf is const reference to the unary factor
		 *	@param[in] r_t_uerr is const reference to the unary error
		 *	@param[in] p_uf_block is pointer to the hessian block of the first vertex in lambda
		 *	@param[in] n_vertes_id is id of the anchor vertex the unary factor will be applied to
		 */
		inline CAddUnaryFactor(_TyReductionPlan &r_rp, const Eigen::MatrixXd &r_t_uf,
			const Eigen::VectorXd &r_t_uerr, double *p_uf_block, size_t n_vertes_id)
			:m_r_rp(r_rp), m_r_t_uf(r_t_uf), m_r_t_uerr(r_t_uerr), m_p_uf_block(p_uf_block),
			m_n_vertes_id(n_vertes_id)
		{
			_ASSERTE(p_uf_block);
		}

		/**
		 *	@brief callback operator; adds the unary factor ro the reduction plan
		 *	@tparam C2DSize is size of the unary factor
		 */
		template <class C2DSize>
		void operator ()()
		{
			double *p_temp = m_r_rp.r_Lambda_ReductionPlan().template
				p_Diagonal_GetTempBlock<C2DSize>(m_n_vertes_id, m_n_vertes_id, m_p_uf_block);
			// get a temp reduction block for the unary factor

			typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num,
				C2DSize::n_column_num>::_Ty dest(p_temp);
			dest.noalias() = m_r_t_uf.transpose() * m_r_t_uf;
			// copy UF

			if(m_r_t_uerr.squaredNorm() > 0) {
				double *p_temp_vec = m_r_rp.r_RHS_ReductionPlan().template
					p_Get_ReductionBlock<C2DSize::n_row_num>(m_n_vertes_id);
				// get a temp reduction block for the unary error

				typename CUberBlockMatrix::CMakeMatrixRef<C2DSize::n_row_num, 1>::_Ty vec(p_temp_vec);
				vec = m_r_t_uerr;
				// copy the error
			}
			// if norm of the unary error is nonzero (only special applications, it usualy is zero)
		}

		/**
		 *	@brief adds unary factor to the reduction plan
		 *
		 *	@param[in] r_rp is reference to lambda reductor
		 *	@param[in] r_t_uf is const reference to the unary factor
		 *	@param[in] r_t_uerr is const reference to the unary error
		 *	@param[in] r_lambda is reference to lambda
		 *	@param[in] n_vertes_id is id of the anchor vertex the unary factor will be applied to
		 *
		 *	@note This can only be used when the structure of lambda is fully allocated.
		 */
		static void Add_UnaryFactor(_TyReductionPlan &r_rp, const Eigen::MatrixXd &r_t_uf,
			const Eigen::VectorXd &r_t_uerr, CUberBlockMatrix &r_lambda, size_t n_vertes_id)
		{
			double *p_UF_block = r_lambda.p_GetBlock_Log(n_vertes_id, n_vertes_id, r_t_uf.rows(), r_t_uf.cols(), true, false);
			CAddUnaryFactor add_uf(r_rp, r_t_uf, r_t_uerr, p_UF_block, n_vertes_id);
			CTypelistItemBFind<typename CSortTypelist<_TyLambdaMatrixBlockSizes,
				fbs_ut::CCompareSize2D_Less>::_TyResult, fbs_ut::CRuntimeCompareSize2D,
				std::pair<size_t, size_t>, CAddUnaryFactor>::FindExisting(std::make_pair(r_t_uf.rows(),
				r_t_uf.cols()), add_uf);
		}

		/**
		 *	@brief adds unary factor to the reduction plan
		 *
		 *	@param[in] r_rp is reference to lambda reductor
		 *	@param[in] r_t_uf is const reference to the unary factor
		 *	@param[in] r_t_uerr is const reference to the unary error
		 *	@param[in] r_lambda is reference to lambda
		 *	@param[in] n_vertes_id is id of the anchor vertex the unary factor will be applied to
		 *	@param[in] n_vertes_order is order of the vertex with id n_vertes_id
		 */
		static void Add_UnaryFactor(_TyReductionPlan &r_rp, const Eigen::MatrixXd &r_t_uf,
			const Eigen::VectorXd &r_t_uerr, CUberBlockMatrix &r_lambda, size_t n_vertes_id, size_t n_vertes_order)
		{
			double *p_UF_block = r_lambda.p_FindBlock(n_vertes_order, n_vertes_order,
				r_t_uf.rows(), r_t_uf.cols(), true, false);
			CAddUnaryFactor add_uf(r_rp, r_t_uf, r_t_uerr, p_UF_block, n_vertes_id);
			CTypelistItemBFind<typename CSortTypelist<_TyLambdaMatrixBlockSizes,
				fbs_ut::CCompareSize2D_Less>::_TyResult, fbs_ut::CRuntimeCompareSize2D,
				std::pair<size_t, size_t>, CAddUnaryFactor>::FindExisting(std::make_pair(r_t_uf.rows(),
				r_t_uf.cols()), add_uf);
		}
	};

public:
	/**
	 *	@brief incrementally updates the lambda matrix structure (can be empty)
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in,out] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *	@param[in] n_vertices_already_in_lambda is number of vertices which are already in the matrix
	 *	@param[in] n_edges_already_in_lambda is number of edges which are already in the matrix
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CSystem>
	static inline void Extend_Lambda(CSystem &r_system,
		_TyReductionPlan &r_reduction_plan, CUberBlockMatrix &r_lambda,
		size_t n_vertices_already_in_lambda, size_t n_edges_already_in_lambda) // throw(std::bad_alloc)
	{
		if(!n_vertices_already_in_lambda && !n_edges_already_in_lambda)
			AddEntriesInSparseSystem(r_system, r_reduction_plan, r_lambda); // works for empty
		else {
			UpdateSparseSystem(r_system, r_reduction_plan, r_lambda,
				n_vertices_already_in_lambda, n_edges_already_in_lambda); // does not work for empty
		}
		// create block matrix lambda
	}

	/**
	 *	@brief refreshes the lambda matrix by recalculating edge hessians
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in,out] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *	@param[in] n_refresh_from_vertex is zero-based index of the first vertex to refresh (unused)
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge to refresh
	 */
	template <class CSystem>
	static inline void Refresh_Lambda(CSystem &r_system, _TyReductionPlan &r_reduction_plan,
		CUberBlockMatrix &r_lambda, size_t UNUSED(n_refresh_from_vertex) = 0, size_t n_refresh_from_edge = 0)
	{
		size_t n_edge_num = r_system.r_Edge_Pool().n_Size();
		size_t n_new_edge_num = n_edge_num - n_refresh_from_edge;
		const size_t n_parallel_thresh = 50;
		if(n_refresh_from_edge) {
			if(n_new_edge_num > n_parallel_thresh)
				r_system.r_Edge_Pool().For_Each_Parallel(n_refresh_from_edge, n_edge_num, CCalculate_Hessians_v2(), 0); // always run in parallel
		} else
			r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Hessians_v2());
		// can do this in parallel

		if(n_refresh_from_edge) {
			if(n_new_edge_num > n_parallel_thresh) {
				r_system.r_Edge_Pool().For_Each/*_Parallel*/(n_refresh_from_edge,
					r_system.r_Edge_Pool().n_Size(), CUpdate_Hessians_v2(r_lambda, r_reduction_plan, false)/*, 0*/); // always run in parallel, the hessians are already calculated
				// reduce only, can't do this in parallel since the reductions do not use temporaries
				// and although they would have the same results, they will not have the same results
				// if reducing in parallel into the same variable. would have to use a temporary. but
				// we already have the hessians precalculated so at least that is saved.
			} else {
				r_system.r_Edge_Pool().For_Each(n_refresh_from_edge,
					r_system.r_Edge_Pool().n_Size(), CUpdate_Hessians_v2(r_lambda, r_reduction_plan, true)); // not in parallel, recalculate the hessians as well
				// calculate and reduce, *not* in parallel
			}
		} else
			r_reduction_plan.r_Lambda_ReductionPlan().ReduceAll(); // simple, parallel
		// run the reduction plan
	}

	/**
	 *	@brief calculates the right-hand side vector
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in] r_system is optimized system (unused)
	 *	@param[in] r_reduction_plan is reduction plan
	 *	@param[in,out] r_v_b is reference to the r.h.s. vector (needs to be
	 *		allocated by the caller to the appropriate dimension)
	 */
	template <class CSystem>
	static inline void Collect_RightHandSide_Vector(const CSystem &UNUSED(r_system),
		const _TyReductionPlan &r_reduction_plan, Eigen::VectorXd &r_v_b)
	{
		r_reduction_plan.r_RHS_ReductionPlan().ReduceAll(r_v_b);
		// collect b
	}

	/**
	 *	@brief calculates a segment of the right-hand side vector, corresponding to a range of vertices
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan
	 *	@param[in,out] r_v_b is reference to the r.h.s. vector (needs to be
	 *		allocated by the caller to the appropriate dimension)
	 *	@param[in] n_begin is zero-based index of the first vertex to calculate the r.h.s. for
	 *	@param[in] n_end is zero-based index of one past the last vertex to calculate the r.h.s. for
	 */
	template <class CSystem>
	static inline void Collect_RightHandSide_Vector(const CSystem &r_system,
		const _TyReductionPlan &r_reduction_plan, Eigen::VectorXd &r_v_b, size_t n_begin, size_t n_end)
	{
		/*if(n_end - n_begin > 50) {
			// t_odo - parallel implementation using ReduceSingle
			// can't, would have to use a reduce to temporary and then overwrite the result
		}*/

		n_begin = r_system.r_Vertex_Pool()[n_begin].n_Order();
		typename CSystem::_TyConstVertexRef last = r_system.r_Vertex_Pool()[n_end - 1];
		n_end = last.n_Order() + last.n_Dimension();
		// need to convert from vertex indices to element indices

		r_reduction_plan.r_RHS_ReductionPlan().ReduceRange(r_v_b, n_begin, n_end);
		// collect b
	}

protected:
	/**
	 *	@brief signle vertex r.h.s. reduction functor
	 */
	class CSingleRHSReducer {
	protected:
		const _TyReductionPlan &r_reduction_plan; /**< @brief reference to the reduction plan */
		Eigen::VectorXd &r_v_b; /**< @brief reference to the r.h.s. vector */
		size_t n_order; /**< @brief order of the reduced vertex */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] _r_reduction_plan is reference to the reduction plan
		 *	@param[in] _r_v_b is reference to the r.h.s. vector
		 *	@param[in] _n_order is order of the reduced vertex
		 */
		inline CSingleRHSReducer(const _TyReductionPlan &_r_reduction_plan, Eigen::VectorXd &_r_v_b, size_t _n_order)
			:r_reduction_plan(_r_reduction_plan), r_v_b(_r_v_b), n_order(_n_order)
		{}

		/**
		 *	@brief performs a single vertex r.h.s. reduction
		 */
		template <class CBlockSize>
		inline void operator ()()
		{
			r_reduction_plan.r_RHS_ReductionPlan().template
				Reduce_Single<CBlockSize::n_size>(r_v_b, n_order);
		}
	};

public:
	/**
	 *	@brief calculates a segment of the right-hand side vector, corresponding to a single vertex
	 *
	 *	@tparam CSystem is optimized system type
	 *
	 *	@param[in] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan
	 *	@param[in,out] r_v_b is reference to the r.h.s. vector (needs to be
	 *		allocated by the caller to the appropriate dimension)
	 *	@param[in] n_vertex is zero-based index of the vertex to calculate the r.h.s. for
	 */
	template <class CSystem>
	static inline void Collect_RightHandSide_Vector(const CSystem &r_system,
		const _TyReductionPlan &r_reduction_plan, Eigen::VectorXd &r_v_b, size_t n_vertex)
	{
		typename CSystem::_TyConstVertexRef vertex = r_system.r_Vertex_Pool()[n_vertex];

		fbs_ut::CWrap3<>::template In_ScalarSize_DecisionTree<typename
			_TyReductionPlan::CRHSReductor::_TyDimensionList>(vertex.n_Dimension(),
			CSingleRHSReducer(r_reduction_plan, r_v_b, vertex.n_Order()));
	}

protected:
	/*class CSumEdgeDims { // profiling
	protected:
		size_t &m_r_n_sum;

	public:
		CSumEdgeDims(size_t &r_n_dim_sum)
			:m_r_n_sum(r_n_dim_sum)
		{}

		template <class _TyEdge>
		inline void operator ()(const _TyEdge &UNUSED(r_edge))
		{
			m_r_n_sum += _TyEdge::n_residual_dimension;
		}
	};*/

public:
	/**
	 *	@brief functor which copies a cumulative sum of variable dimensions to an array
	 */
	class CCopyVariableDims {
	protected:
		std::vector<size_t>::iterator m_p_dest_it; /**< @brief iterator to write the next vertex dimension */
		size_t m_n_prev_value; /**< @brief iterator to write the next vertex dimension */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_dest is a reference to a vector to fill with vertex dimensions (must be allocated)
		 */
		CCopyVariableDims(std::vector<size_t> &r_dest)
			:m_p_dest_it(r_dest.begin()), m_n_prev_value(0)
		{}

		/**
		 *	@brief function operator; stores the variable dimension
		 *	@tparam _TyVertex is vertex type
		 *	@param[in] r_vertex is reference to the vertex (unused)
		 */
		template <class _TyVertex>
		inline void operator ()(const _TyVertex &UNUSED(r_vertex))
		{
			*m_p_dest_it = m_n_prev_value += _TyVertex::n_dimension; // hopefully the compiler can optimize away the decision tree and just act on the types
			++ m_p_dest_it;
		}
	};

protected:
	/**
	 *	@brief creates the lambda matrix from scratch
	 *
	 *	@param[in,out] r_system is optimized system
	 *	@param[in] r_reduction_plan is reduction plan
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	template <class CSystem>
	static inline void AddEntriesInSparseSystem(CSystem &r_system,
		_TyReductionPlan &r_reduction_plan, CUberBlockMatrix &r_lambda) // throw(std::bad_alloc, std::runtime_error)
	{
		/*CDeltaTimer t;
		printf("one: %f\n", t.f_Time());*/ // profiling
#if 0
		r_lambda.Clear();
		// ...
#else // 0
		{
			std::vector<size_t> row_col_cumsums(r_system.r_Vertex_Pool().n_Size());
#if 1
			r_system.r_Vertex_Pool().For_Each(CCopyVariableDims(row_col_cumsums)); // serial, using type info, 0.000393 sec (on windows)
			// faster
#else
			_ASSERTE(r_system.r_Vertex_Pool().n_Size() <= INT_MAX);
			#pragma omp parallel if(r_system.r_Vertex_Pool().n_Size() > 50)
			for(int i = 0, n = int(r_system.r_Vertex_Pool().n_Size()); i < n; ++ i)
				row_col_cumsums[i] = r_system.r_Vertex_Pool()[i].n_Dimension(); // parallel, using facades // 0.001340 sec (on windows)
			std::partial_sum(row_col_cumsums.begin(), row_col_cumsums.end(), row_col_cumsums.begin()); // inclusive
#endif
			// get inclusive sum of the dimensions of the vertices

			//printf("two: %f\n", t.f_Time()); // profiling

			CUberBlockMatrix empty(row_col_cumsums.begin(), row_col_cumsums.end(),
				row_col_cumsums.begin(), row_col_cumsums.end());
			r_lambda.Swap(empty);
		}
		// initialize the block rows and columns of the matrix, so that the order in which the edges are inserted
		// does not cause row reindexing (this is usually not a problem in SLAM, but it gets worse in BA)
#endif // 0

		/*printf("three: %f\n", t.f_Time());
		{
			std::vector<size_t> edge_dims(r_system.r_Edge_Pool().n_Size());
			size_t n_dim_sum = 0;
			r_system.r_Edge_Pool().For_Each(CSumEdgeDims(n_dim_sum));

			printf("three.5: %f (" PRIsize ")\n", t.f_Time(), n_dim_sum);
		}
		n_dummy_param = 0;
		r_lambda.Reset_Perfcounters();*/ // profiling

		r_system.r_Edge_Pool().For_Each(CAlloc_HessianBlocks_v2(r_lambda, r_reduction_plan));
		// add all the hessian blocks

		/*printf("four: %f (" PRIsize" / " PRIsize ")\n", t.f_Time(), n_dummy_param, r_lambda.n_Block_Num());
		r_lambda.Dump_PerfCounters();*/ // profiling

		if(!CSystem::null_UnaryFactor) {
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_first_vertex_id = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Edge_Pool().b_Empty());
			size_t n_first_vertex_id = r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!r_system.r_Vertex_Pool()[n_first_vertex_id].b_IsConstant()); // this one must not be constant
			size_t n_first_vertex_order = r_system.r_Vertex_Pool()[n_first_vertex_id].n_Order();
			// get id of the first vertex (usually zero)

			const Eigen::MatrixXd &r_t_uf = r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
			/*if(!r_lambda.Append_Block_Log(Eigen::MatrixXd(r_t_uf.transpose() * r_t_uf),
			   n_first_vertex_id, n_first_vertex_id))
				throw std::bad_alloc();*/ // no need to do this anymore

			CAddUnaryFactor::Add_UnaryFactor(r_reduction_plan, r_t_uf,
				r_system.r_v_Unary_Error(), r_lambda, n_first_vertex_id, n_first_vertex_order);
			// add unary factor (actually UF^T * UF, it was square-rooted before)
		}
		// add unary factor to the reductor so that we don't have to care about ir anymore
		// do this after the edges, as we are using Append_Block_Log() and hence we need
		// the row / column with n_first_vertex_id to exist (and if n_first_vertex_id > 0,
		// then it might not)

		//printf("five: %f\n", t.f_Time()); // profiling
	}

	/**
	 *	@brief incrementally updates the lambda matrix structure (must not be empty)
	 */
	template <class CSystem>
	static inline void UpdateSparseSystem(CSystem &r_system,
		_TyReductionPlan &r_reduction_plan, CUberBlockMatrix &r_lambda,
		size_t n_skip_vertices, size_t n_skip_edges) // throw(std::bad_alloc)
	{
		_ASSERTE(r_lambda.n_Row_Num() > 0 && r_lambda.n_Column_Num() == r_lambda.n_Row_Num()); // make sure lambda is not empty
		r_system.r_Edge_Pool().For_Each(n_skip_edges,
			r_system.r_Edge_Pool().n_Size(), CAlloc_HessianBlocks_v2(r_lambda, r_reduction_plan));
		// add the hessian blocks of the new edges
	}
};

} // ~lambda_utils

/** @} */ // end of group

#endif // !__NONLINEAR_SOLVER_LAMBDA_UTILS
