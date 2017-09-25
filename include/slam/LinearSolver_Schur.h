/*
								+-----------------------------------+
								|                                   |
								|   ***  Schur linear solver  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|       LinearSolver_Schur.h        |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __LINEAR_SOLVER_SCHUR_INCLUDED
#define __LINEAR_SOLVER_SCHUR_INCLUDED

/**
 *	@file include/slam/LinearSolver_Schur.h
 *	@brief blocky linear solver wrapper that enables Schur complement optimization
 *	@author -tHE SWINe-
 *	@date 2013-01-28
 */

#include "slam/LinearSolverTags.h"
//#include "slam/LinearSolver_CSparse.h" // needed for debugging/profiling only
//#include "slam/LinearSolver_CholMod.h" // needed for debugging/profiling only
#include "csparse/cs.hpp"
//#include "slam/BlockMatrix.h" // included from slam/LinearSolverTags.h
#include "slam/LinearSolver_UberBlock.h" // fallback for small/diagonal matrices

/** \addtogroup linsolve
 *	@{
 */

/**
 *	@def __SCHUR_PROFILING
 *	@brief enables printing Schur complement timing greakdown
 */
//#define __SCHUR_PROFILING

/**
 *	@def __DISABLE_GPU
 *	@brief overrides other GPU switches, GPU won't be used
 *	@note Interfaces of the GPU objects will remain declared, but will throw.
 */
//#define __DISABLE_GPU

/**
 *	@def __SCHUR_USE_DENSE_SOLVER
 *	@brief if defined, a dense linear solver is used on the Schur complement
 *	@note This is usually faster if the Schur complement is a small portion
 *		of the system and is dense enough (e.g. the upper triangular is more
 *		than 10% of full square matrix NNZs).
 */
#define __SCHUR_USE_DENSE_SOLVER

#ifndef __DISABLE_GPU

/**
 *	@def __GPU_SCHUR_NO_PRODS
 *	@brief if defined, GPU is not used to accelerate \f$U(C^{-1})\f$ and \f$U(C^{-1})V\f$ products
 *	@note This might yield faster code if running on a slower mobile GPU. cuSparse
 *		is barely faster than SSE ÜberBlockMatrix on Tesla GPUs.
 */
#define __GPU_SCHUR_NO_PRODS

/**
 *	@def __SCHUR_DENSE_SOLVER_USE_GPU
 *	@brief if defined (and __DISABLE_GPU not defined), a CULA dense linear solver is used on the Schur complement
 */
#define __SCHUR_DENSE_SOLVER_USE_GPU

#endif // !__DISABLE_GPU

/**
 *	@brief implementation of matrix orderings for Schur complement
 */
class CSchurOrdering {
public:
	/**
	 *	@brief a simple IOTA functor
	 */
	class CIOTA {
	protected:
		size_t m_n_counter; /**< @brief counter value */

	public:
		/**
		 *	@brief default constructor; initializes the counter
		 */
		inline CIOTA()
			:m_n_counter(-1)
		{}

		/**
		 *	@brief counter invokation
		 *	@return Returns the next number in sequence, starting with zero.
		 */
		inline size_t operator ()()
		{
			return ++ m_n_counter;
		}
	};

	/**
	 *	@brief a simple function object for vertex degree comparison
	 */
	class CCompareVertexDegree {
	protected:
		const cs *m_p_graph; /**< @brief pointer to the graph */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_graph is const pointer to the graph
		 */
		inline CCompareVertexDegree(const cs *p_graph)
			:m_p_graph(p_graph)
		{}

		/**
		 *	@brief vertex degree comparison operator
		 *
		 *	@param[in] n_vertex_a is zero-based index of the first vertex, in the given graph
		 *	@param[in] n_vertex_b is zero-based index of the second vertex, in the given graph
		 *
		 *	@return Returns true if the first vertex has lower degree than the second vertex.
		 */
		inline bool operator ()(size_t n_vertex_a, size_t n_vertex_b) const
		{
			_ASSERTE(n_vertex_a < size_t(m_p_graph->n) && n_vertex_b < size_t(m_p_graph->n));
			return m_p_graph->p[n_vertex_a + 1] - m_p_graph->p[n_vertex_a] <
				m_p_graph->p[n_vertex_b + 1] - m_p_graph->p[n_vertex_b];
		}
	};

	/**
	 *	@brief a simple function object for weighted vertex degree comparison
	 */
	class CCompareVertexDegree_Weights {
	protected:
		const cs *m_p_graph; /**< @brief pointer to the graph */
		const std::vector<size_t> &m_r_weights; /**< @brief vector of weights for each vertex of the graph */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] p_graph is const pointer to the graph
		 *	@param[in] r_weights is const reference to a vector of weights for each vertex of the graph
		 */
		inline CCompareVertexDegree_Weights(const cs *p_graph, const std::vector<size_t> &r_weights)
			:m_p_graph(p_graph), m_r_weights(r_weights)
		{
			_ASSERTE(m_r_weights.size() == m_p_graph->n);
		}

		/**
		 *	@brief vertex degree comparison operator
		 *
		 *	@param[in] n_vertex_a is zero-based index of the first vertex, in the given graph
		 *	@param[in] n_vertex_b is zero-based index of the second vertex, in the given graph
		 *
		 *	@return Returns true if the first vertex has lower degree than the second vertex.
		 */
		inline bool operator ()(size_t n_vertex_a, size_t n_vertex_b) const
		{
			_ASSERTE(n_vertex_a < size_t(m_p_graph->n) && n_vertex_b < size_t(m_p_graph->n));
			return n_Weighted_IncidenceSum(n_vertex_a) < n_Weighted_IncidenceSum(n_vertex_b);
			// could optimize for one of them having zero incident vertices but that does not happen in SLAM / BA
		}

		/**
		 *	@brief calculates weighted sum of incident vertices
		 *	@param[in] n_vertex_a is zero-based index of a vertex in the given graph
		 *	@return Returns the weighted sum of vertices incident to the given vertex.
		 */
		inline size_t n_Weighted_IncidenceSum(size_t n_vertex_a) const
		{
			_ASSERTE(n_vertex_a < size_t(m_p_graph->n));
			size_t n_incidences_a = 0;
			for(size_t i = m_p_graph->p[n_vertex_a], n = m_p_graph->p[n_vertex_a + 1]; i < n; ++ i) {
				if(size_t(m_p_graph->i[i]) != n_vertex_a)
					n_incidences_a += m_r_weights[m_p_graph->i[i]];
			}
			return n_incidences_a;// + m_r_weights[n_vertex_a]; // make sure the diagonal term is always coutned
		}
	};

	/**
	 *	@brief a simple function object for weighted vertex degree comparison
	 */
	class CCompareVertexDegree_WeightsContribution {
	protected:
		const cs *m_p_graph; /**< @brief pointer to the graph */
		const std::vector<size_t> &m_r_weights; /**< @brief vector of weights for each vertex of the graph */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] p_graph is const pointer to the graph
		 *	@param[in] r_weights is const reference to a vector of weights for each vertex of the graph
		 */
		inline CCompareVertexDegree_WeightsContribution(const cs *p_graph, const std::vector<size_t> &r_weights)
			:m_p_graph(p_graph), m_r_weights(r_weights)
		{
			_ASSERTE(m_r_weights.size() == m_p_graph->n);
		}

		/**
		 *	@brief vertex degree comparison operator
		 *
		 *	@param[in] n_vertex_a is zero-based index of the first vertex, in the given graph
		 *	@param[in] n_vertex_b is zero-based index of the second vertex, in the given graph
		 *
		 *	@return Returns true if the first vertex has lower degree than the second vertex.
		 */
		inline bool operator ()(size_t n_vertex_a, size_t n_vertex_b) const
		{
			_ASSERTE(n_vertex_a < size_t(m_p_graph->n) && n_vertex_b < size_t(m_p_graph->n));
			return n_Weighted_IncidenceContribution(n_vertex_a) < n_Weighted_IncidenceContribution(n_vertex_b);
			// could optimize for one of them having zero incident vertices but that does not happen in SLAM / BA
		}

		/**
		 *	@brief calculates weight of a vertex minus the weighted sum of incident vertices
		 *	@param[in] n_vertex_a is zero-based index of a vertex in the given graph
		 *	@return Returns the weighted of the given vertex minus zjr weighted sum of its incident vertices.
		 */
		inline ptrdiff_t n_Weighted_IncidenceContribution(size_t n_vertex_a) const
		{
			_ASSERTE(n_vertex_a < size_t(m_p_graph->n));
			size_t n_incidences_a = 0;
			for(size_t i = m_p_graph->p[n_vertex_a], n = m_p_graph->p[n_vertex_a + 1]; i < n; ++ i) {
				if(size_t(m_p_graph->i[i]) != n_vertex_a)
					n_incidences_a += m_r_weights[m_p_graph->i[i]];
			}
			return ptrdiff_t(m_r_weights[n_vertex_a]) - ptrdiff_t(n_incidences_a); // what n_vertex_a brings and what it precludes
		}
	};

	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		max_Recurse_Size = 128, /**< @brief maximum MIS recursion */
		thread_Num = 1 << 8, /**< @brief the number of threads */ // make a bit more to allow for better scheduling (at each other step, the graph is only cut a little, need to have enough steps to make all cores busy with the big graphs)
		thread_Num_Log2 = n_Log2_Static(thread_Num) /**< @brief base 2 logarithm of the number of threads */
	};

	/**
	 *	@brief explicit stack frame, for t_MIS_ExStack() and t_MIS_Parallel()
	 */
	struct TMIS_StackFrame {
		std::vector<size_t> complement_set; /**< @brief storage for the vertex complement set */
		size_t n_intermediate; /**< @brief intermediate value (used to store the pivot) */
		std::vector<size_t> int_result; /**< @brief intermediate result */
		const cs *p_graph; /**< @brief pointer to the original graph (not to be deleted) */
		cs *p_subgraph; /**< @brief pointer to a subgraph (to be deleted when this stack frame returns) */
		int n_phase; /**< @brief index of the phase of the algorithm, in which the stack frame was saved */
	};

public:
	/**
	 *	@brief calculates ordering for Schur
	 *
	 *	@param[out] r_ordering is filled with the ordering (does not need to be allocated)
	 *	@param[in] r_lambda is the matrix to be ordered on
	 *
	 *	@return Returns number of connected vertices (the size of the Schur factor).
	 *
	 *	@note There is some workspace, which could be reused.
	 */
	static size_t n_Calculate_Ordering(std::vector<size_t> &r_ordering,
		const CUberBlockMatrix &r_lambda); // throw(std::bad_alloc)

	/**
	 *	@brief calculates guided ordering for Schur
	 *
	 *	@param[out] r_ordering is filled with the ordering (does not need to be allocated)
	 *	@param[in] n_pose_vertex_dimension is dimension of pose (e.g. 3 for 2D SLAM)
	 *	@param[in] n_landmark_vertex_dimension is dimension of landmark (e.g. 2 for 2D SLAM)
	 *	@param[in] r_lambda is the matrix to be ordered on
	 *
	 *	@return Returns number of poses (the size of the Schur complement).
	 *
	 *	@note In case this is used, simpler linear solver can be used
	 *		as the block sizes are known beforehand, and are constant
	 *		for each section of the factorized matrix.
	 */
	static size_t n_Calculate_GuidedOrdering(std::vector<size_t> &r_ordering,
		size_t n_pose_vertex_dimension, size_t n_landmark_vertex_dimension,
		const CUberBlockMatrix &r_lambda); // throw(std::bad_alloc)

#ifdef HAVE_IGRAPH

	static std::vector<std::vector<size_t> > t_Find_Cliques_igraph(const cs *p_graph,
		int n_min_clique_size = 2, int n_max_clique_size = INT_MAX); // throw(std::bad_alloc,std::runtime_error)

	static std::vector<size_t> t_MIS_igraph(const cs *p_graph,
		const std::vector<size_t> &r_weights); // throw(std::bad_alloc,std::runtime_error)

	static std::vector<size_t> t_MIS_igraph(const cs *p_graph); // throw(std::bad_alloc,std::runtime_error)

#endif // HAVE_IGRAPH

	static void Suppress_NonmaximalCliques(std::vector<std::vector<size_t> > &linear_clique_list);

	static void PruneCliques_MaxVertexOrder(std::vector<std::vector<size_t> > &r_clique_list,
		size_t n_vertex_num); // throw(std::bad_alloc)

	static std::vector<std::vector<size_t> > t_Find_Cliques(const cs *p_graph,
		bool b_final_nonmaximal_clique_removal = true, size_t n_min_clique_size = 2,
		size_t n_max_clique_size = SIZE_MAX/*16*/); // throw(std::bad_alloc)

	/**
	 *	@brief calculates MIS using sorted first-fit, followed by vertex swap improvement
	 *
	 *	@param[in] p_graph is A^T+A graph (the diagonal must be present)
	 *	@param[in] b_allow_improvement is iterative improvement flag (swap out vertices; default enabled)
	 *	@param[in] n_chain_improvement_size_limit is maximum graph size to run vertex
	 *		chain improvement on (runs in quadratic time, default 64)
	 *
	 *	@return Returns (sorted) maximum independent set of vertices.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function is only approximate, and therefore, it is fast. Usually,
	 *		it comes within 10% of the perfect solution (at least on graph SLAM problems).
	 *		To calculate perfect MIS, use t_MIS_Parallel() or t_MIS_ExStack().
	 */
	static std::vector<size_t> t_MIS_FirstFit(const cs *p_graph,
		bool b_allow_improvement = true, size_t n_chain_improvement_size_limit = 64); // throw(std::bad_alloc)

	/**
	 *	@brief calculates MIS using sorted first-fit, followed by vertex swap improvement
	 *
	 *	@param[in] p_graph is A^T+A graph (the diagonal must be present)
	 *	@param[in] r_weights is reference to a list of weights for each vertex of the graph
	 *	@param[in] b_allow_improvement is iterative improvement flag (swap out vertices; default enabled)
	 *	@param[in] n_chain_improvement_size_limit is maximum graph size to run vertex
	 *		chain improvement on (runs in quadratic time, default 64)
	 *
	 *	@return Returns (sorted) maximum independent set of vertices.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function is only approximate, and therefore, it is fast. Usually,
	 *		it comes within 10% of the perfect solution (at least on graph SLAM problems).
	 *		To calculate perfect MIS, use t_MIS_Parallel() or t_MIS_ExStack().
	 */
	static std::vector<size_t> t_MIS_FirstFit(const cs *p_graph, const std::vector<size_t> &r_weights,
		bool b_allow_improvement = true, size_t n_chain_improvement_size_limit = 64); // throw(std::bad_alloc)

	/**
	 *	@brief calculates maximum independent set
	 *
	 *	@param[in] p_graph is A^T+A graph (the diagonal may or may not be present)
	 *
	 *	@return Returns the size of the maximum independent set.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function has exponential time complexity,
	 *		and is unfeasible to call it on sizable graphs.
	 */
	static size_t n_MIS(const cs *p_graph); // throw(std::bad_alloc)

	/**
	 *	@brief calculates maximum independent set
	 *
	 *	@param[in] p_graph is A^T+A graph (the diagonal may or may not be present)
	 *
	 *	@return Returns list of (zero-based) indices of vertices in the maximum independent set.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function has exponential time complexity,
	 *		and is unfeasible to call it on sizable graphs.
	 */
	static std::vector<size_t> t_MIS(const cs *p_graph); // throw(std::bad_alloc)

	/**
	 *	@brief calculates maximum independent set, using explicit stack
	 *
	 *	@param[in] p_graph is A^T+A graph (the diagonal may or may not be present)
	 *
	 *	@return Returns list of (zero-based) indices of vertices in the maximum independent set.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function has exponential time complexity,
	 *		and is unfeasible to call it on sizable graphs.
	 *	@note This function uses explicit stack, so it can work on graphs of
	 *		arbitrary size (unline t_MIS()), but it is slightly slower.
	 */
	static std::vector<size_t> t_MIS_ExStack(const cs *p_graph); // throw(std::bad_alloc)

	/**
	 *	@brief calculates maximum independent set, in parallel
	 *
	 *	@param[in] p_graph is A^T+A graph (the diagonal may or may not be present)
	 *
	 *	@return Returns list of (zero-based) indices of vertices in the maximum independent set.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function has exponential time complexity,
	 *		and is unfeasible to call it on sizable graphs.
	 *	@note This function uses explicit stack, so it can work on graphs of
	 *		arbitrary size (unline t_MIS()), but it is slightly slower.
	 *	@note This function uses OpenMP to run the calculation in parallel;
	 *		the scheduling is, however, only static - and thus imperfect.
	 */
	static std::vector<size_t> t_MIS_Parallel(const cs *p_graph); // throw(std::bad_alloc)

	/**
	 *	@brief determines whether a graph is connected
	 *	@param[in] p_graph is pointer to a graph symmetric matrix (the diagonal is not accessed)
	 *	@return Returns true if the given graph is connected, otherwise returns false.
	 *	@note This function throws std::bad_alloc.
	 */
	static bool b_IsConnected(const cs *p_graph); // throw(std::bad_alloc)

	//void Get_MaxConnectedSet(const cs *p_graph, std::vector<size_t> &r_max_conn); // not required

	/**
	 *	@brief gets minimum connected set
	 *
	 *	@param[in] p_graph is pointer to a graph symmetric matrix (the diagonal is not accessed)
	 *	@param[out] r_min_conn is filled with minimum connected set
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Get_MinConnectedSet(const cs *p_graph, std::vector<size_t> &r_min_conn); // throw(std::bad_alloc)

	/**
	 *	@brief determines whether a set of vertex indices is sorted
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in] p_vertex_begin is iterator, pointing to the first vertex index
	 *	@param[in] p_vertex_end is iterator, pointing to one past the last vertex index
	 *
	 *	@return Returns true if the set is sorted in ascending
	 *		order, and does not contain duplicate elements.
	 */
	template <class _TyConstIter>
	static bool b_IsSortedSet(_TyConstIter p_vertex_begin, _TyConstIter p_vertex_end)
	{
		_ASSERTE(p_vertex_end >= p_vertex_begin);
		if(p_vertex_begin == p_vertex_end)
			return true;
		// empty set is sorted

		size_t n_prev = *p_vertex_begin;
		for(++ p_vertex_begin; p_vertex_begin != p_vertex_end; ++ p_vertex_begin) {
			size_t n_cur = *p_vertex_begin;
			if(n_prev >= n_cur)
				return false; // not sorted, or contains repeating elements
			n_prev = n_cur;
		}
		return true;
	}

	/**
	 *	@brief determines whether a set of vertex indices is sorted
	 *	@param[in] r_set is vector, containing a set of vertex indices
	 *	@return Returns true if the set is sorted in ascending
	 *		order, and does not contain duplicate elements.
	 */
	static inline bool b_IsSortedSet(const std::vector<size_t> &r_set)
	{
		return b_IsSortedSet(r_set.begin(), r_set.end());
	}

	/**
	 *	@brief builds a subgraph of a graph
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in] p_graph is pointer to a graph symmetric matrix (the diagonal is not accessed)
	 *	@param[in] p_vertex_begin is iterator, pointing to the first vertex index
	 *	@param[in] p_vertex_end is iterator, pointing to one past the last vertex index
	 *
	 *	@return Returns pointer to the specified subgraph
	 *		(the caller is responsible for freeing it).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class _TyConstIter>
	static cs *p_Subgraph(const cs *p_graph,
		_TyConstIter p_vertex_begin, _TyConstIter p_vertex_end) // throw(std::bad_alloc)
	{
		_ASSERTE(b_IsSortedSet(p_vertex_begin, p_vertex_end));

		const size_t n = p_graph->n;
		_ASSERTE(p_vertex_end >= p_vertex_begin);
		size_t n_remove_verts = p_vertex_end - p_vertex_begin;
		_ASSERTE(n >= n_remove_verts);

		cs *p_new_graph;
		if(!(p_new_graph = cs_spalloc(n - n_remove_verts,
		   n - n_remove_verts, p_graph->p[n], 0, 1)))
			throw std::bad_alloc(); // rehtrow
		// alloc workspace for a graph with up to last graph nnz entries (conservative estimate)

		size_t n_dest = 0;
		for(size_t v = 0; v < n; ++ v) {
			_TyConstIter p_lb_it = std::lower_bound(p_vertex_begin, p_vertex_end, v);
			if(p_lb_it != p_vertex_end && *p_lb_it == v)
				continue;
			// find out if the vertex is removed

			size_t n_col = v - (p_lb_it - p_vertex_begin);
			// calculate destination column (column minus number of vertices skipped)

			for(size_t p0 = p_graph->p[v], p1 = p_graph->p[v + 1]; p0 < p1; ++ p0) {
				size_t v1 = p_graph->i[p0];
				_TyConstIter p_lb2_it = std::lower_bound(p_vertex_begin, p_vertex_end, v1);
				if(p_lb2_it != p_vertex_end && *p_lb2_it == v1)
					continue;
				// find out if the (referenced) vertex is removed

				size_t n_row = v1 - (p_lb2_it - p_vertex_begin);
				// calculate destination row (row minus number of vertices skipped)

				p_new_graph->p[n_dest] = n_col;
				p_new_graph->i[n_dest] = n_row;
				++ n_dest;
			}
		}
		// assemble the graph in triplet form

		p_new_graph->nz = n_dest;
		cs *p_new_graph_c;
		if(!(p_new_graph_c = cs_compress(p_new_graph)))
			throw std::bad_alloc(); // rethrow
		cs_spfree(p_new_graph);
		// compress and free

		return p_new_graph_c;
	}

	/**
	 *	@brief calculates a complement of a set of vertex indices
	 *
	 *	@param[out] r_complement is vector, containing a complement of a set of vertex indices
	 *	@param[in] r_set is vector, containing a set of vertex indices
	 *	@param[in] n_graph_size is number of vertices in the whole graph
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Complement_VertexSet(std::vector<size_t> &r_complement,
		const std::vector<size_t> &r_set, size_t n_graph_size); // throw(std::bad_alloc)

	/**
	 *	@brief transforms vertex indices of a subgraph to vertex indices of the original graph,
	 *		using complement to a set of vertices of the subgraph
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in,out] r_indices is a list of indices to be transformed (works inplace)
	 *	@param[in] r_generating_set_complement is complement vertex set
	 */
	static inline void Transform_Indices_Complement(std::vector<size_t> &r_indices,
		std::vector<size_t> &r_generating_set_complement)
	{
		Transform_Indices_Complement(r_indices, r_generating_set_complement.begin(),
			r_generating_set_complement.end());
	}

	/**
	 *	@brief transforms vertex indices of a subgraph to vertex indices of the original graph,
	 *		using complement to a set of vertices of the subgraph
	 *
	 *	@tparam _TyConstIter is vertex index const iterator type
	 *
	 *	@param[in,out] r_indices is a list of indices to be transformed (works inplace)
	 *	@param[in] p_gsc_begin is iterator, pointing to the first complement vertex index
	 *	@param[in] p_gsc_end is iterator, pointing to one past the last complement vertex index
	 */
	template <class _TyConstIter>
	static void Transform_Indices_Complement(std::vector<size_t> &r_indices,
		_TyConstIter p_gsc_begin, _TyConstIter p_gsc_end)
	{
		_ASSERTE(p_gsc_begin <= p_gsc_end);
		_ASSERTE(b_IsSortedSet(p_gsc_begin, p_gsc_end));
		size_t n_off = 0;
		for(size_t i = 0, m = r_indices.size(); i < m; ++ i) {
			size_t v = r_indices[i];
			while(p_gsc_begin != p_gsc_end && size_t(*p_gsc_begin) <= v + n_off) {
				++ p_gsc_begin;
				++ n_off;
			}
			r_indices[i] = v + n_off;
		}
		_ASSERTE(b_IsSortedSet(r_indices));
		// transform indices back, based on the generating set complement
	}
};

/**
 *	@brief template helpers for the Schur linear solver
 */
namespace schur_detail {

/**
 *	@brief partiteness violating edge predicate
 *
 *	An edge violates partiteness for a given vertex if it can connect two (or more) such vertices together.
 *
 *	@tparam CEdgeType is edge type
 *	@tparam CVertexType is vertex type
 */
template <class CEdgeType, class CVertexType>
class CIsPartitenessViolatingEdge {
protected:
	typedef typename CEdgeType::_TyVertices _TyEdgeVertices; /**< @brief types of vertices the current edge connects */
	typedef typename CFilterTypelist<_TyEdgeVertices, CVertexType, CEqualType>::_TyResult _TyVertexTypeOccurences; /**< @brief only vertices that are the same as the vertex in question, and that the current edge connects */

public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		n_vertex_occurence_num = CTypelistLength<_TyVertexTypeOccurences>::n_result, /**< @brief the number of occurences of the current vertex in the current edge vertices; it can be zero or more (the current edge may be unrelated to the current vertex type) */
		b_result = n_vertex_occurence_num > 1 /**< @brief the result; true if the edge connects more than one vertex of the current vertex type, otherwise false */
	};
};

/**
 *	@brief partiteness violating edge predicate (uses detection by vertex dimension rather than type name)
 *
 *	An edge violates partiteness for a given vertex if it can connect two (or more) such vertices together.
 *
 *	@tparam CEdgeType is edge type
 *	@tparam CVertexType is vertex type
 */
template <class CEdgeType, class CVertexType>
class CIsPartitenessViolatingEdge_ByDim {
protected:
	typedef typename CEdgeType::_TyVertices _TyEdgeVertices; /**< @brief types of vertices the current edge connects */
	typedef typename CTransformTypelist<_TyEdgeVertices, fbs_ut::CGetVertexDimension>::_TyResult CEdgeVertexDims; /**< @brief list of all vertex dimensions */
	typedef typename CFilterTypelist<CEdgeVertexDims, typename
		fbs_ut::CGetVertexDimension<CVertexType>::_TyResult,
		CEqualType>::_TyResult _TyVertexTypeOccurences; /**< @brief only vertices that have the same dimension as the vertex in question, and that the current edge connects */

public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		n_vertex_occurence_num = CTypelistLength<_TyVertexTypeOccurences>::n_result, /**< @brief the number of occurences of the current vertex in the current edge vertices; it can be zero or more (the current edge may be unrelated to the current vertex type) */
		b_result = n_vertex_occurence_num > 1 /**< @brief the result; true if the edge connects more than one vertex of the current vertex type, otherwise false */
	};
};

/**
 *	@brief partite vertex predicate
 *
 *	A vertex type is partite, if there is no edge that would connect two different vertices of this type together.
 *	Note that edges which connect them transitively through different vertices do not matter (vertex type A is
 *	partite for edges of type A-B, B-C and C-A, as no two A vertices can directly be connected, in contrary
 *	if there is edge A-A or possibly a hyperedge *-A-*-A-* (such as A-A-B), the vertex is not partite).
 *
 *	@tparam CVertexType is vertex type
 *	@tparam CEdgeTypeList is type list of edge types
 */
template <class CVertexType, class CEdgeTypeList>
class CIsPartiteVertex {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = !CFindTypelistItem_If<CEdgeTypeList, CVertexType,
			schur_detail::CIsPartitenessViolatingEdge>::b_result /**< @brief result (true if the vertex is partite, false otherwise) */
	};
};

/**
 *	@brief partite vertex predicate (uses detection by vertex dimension rather than type name)
 *
 *	A vertex type is partite, if there is no edge that would connect two different vertices of this type together.
 *	Note that edges which connect them transitively through different vertices do not matter (vertex type A is
 *	partite for edges of type A-B, B-C and C-A, as no two A vertices can directly be connected, in contrary
 *	if there is edge A-A or possibly a hyperedge *-A-*-A-* (such as A-A-B), the vertex is not partite).
 *
 *	@tparam CVertexType is vertex type
 *	@tparam CEdgeTypeList is type list of edge types
 */
template <class CVertexType, class CEdgeTypeList>
class CIsPartiteVertex_ByDim {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = !CFindTypelistItem_If<CEdgeTypeList, CVertexType,
			schur_detail::CIsPartitenessViolatingEdge_ByDim>::b_result /**< @brief result (true if the vertex is partite, false otherwise) */
	};
};

/**
 *	@brief helper function; makes sure a list of fbs_ut::CCTSize is not empty
 */
template <class CList>
struct CNonEmptySizeList {
public:
	typedef CList _TyResult; /**< @brief the resulting non-empty list */
};

/**
 *	@brief helper function; makes sure a list of fbs_ut::CCTSize is not empty (specialization for an empty list; appends fbs_ut::CCTSize<-1>)
 */
template <>
struct CNonEmptySizeList<CTypelistEnd> {
public:
	typedef CTypelist<fbs_ut::CCTSize<-1>, CTypelistEnd> _TyResult; /**< @brief the resulting non-empty list */
};

/**
 *	@brief guided Schur ordering reasoning helper
 *
 *	@tparam CVertexList is list of vertex types
 *	@tparam CEdgeList is list of edge types
 */
template <class CVertexList, class CEdgeList>
class CGuidedOrdering_Helper {
public:
	typedef CVertexList _TyVertexTypelist; /**< @brief list of vertex types */
	typedef CEdgeList _TyEdgeTypelist; /**< @brief list of edge types */

	typedef typename CFilterTypelist<_TyVertexTypelist, _TyEdgeTypelist, CIsPartiteVertex_ByDim>::_TyResult CPartiteVertexList; /**< @brief list of partite vertices */
	typedef typename CTypelistDifference<_TyVertexTypelist, CPartiteVertexList>::_TyResult CNonPartiteVertexList; /**< @brief list of non-partite vertices */

	typedef typename CTransformTypelist<_TyVertexTypelist, fbs_ut::CGetVertexDimension>::_TyResult CAllVertexDimsList; /**< @brief list of all vertex dimensions */
	typedef typename CUniqueTypelist<CAllVertexDimsList>::_TyResult CAllUniqueVertexDims; /**< @brief list of unique vertex dimensions */
	typedef typename CSortTypelist<CAllUniqueVertexDims, fbs_ut::CCompareScalar_Less>::_TyResult CAllUniqueSortedVertexDims; /**< @brief list of sorted unique vertex dimensions */

	typedef typename CTransformTypelist<CPartiteVertexList, fbs_ut::CGetVertexDimension>::_TyResult CPartiteVertexDimsList; /**< @brief list of partite vertex dimensions (with possible repetitions) */
	typedef typename CTransformTypelist<CNonPartiteVertexList, fbs_ut::CGetVertexDimension>::_TyResult CNonPartiteVertexDimsList; /**< @brief list of non-partite vertex dimensions (with possible repetitions) */

	typedef typename CUniqueTypelist<CPartiteVertexDimsList>::_TyResult CPartiteUniqueVertexDims; /**< @brief list of partite unique vertex dimensions */
	typedef typename CSortTypelist<CPartiteUniqueVertexDims, fbs_ut::CCompareScalar_Less>::_TyResult CPartiteUniqueSortedVertexDims; /**< @brief list of sorted unique partite vertex dimensions */
	typedef typename CUniqueTypelist<CNonPartiteVertexDimsList>::_TyResult CNonPartiteUniqueVertexDims; /**< @brief list of unique non-partite vertex dimensions */

	typedef typename CTypelistDifference<CPartiteUniqueSortedVertexDims, CNonPartiteUniqueVertexDims>::_TyResult CRecognizablePartiteVertexDims; /**< @brief list of partite vertex dimensions that can be recognized from non-partite */ // only used to count them

	// todo - figure out if the choice of grouping compatible partite vertex types is an easy one and if it is,
	//		  see about modifying the code to be able to order multiple different vertex groups in the diagonal

	/**
	 *	@brief bitwise mask creation
	 *
	 *	@tparam _T1 is the first operand (specialization of CCTSize template), containing position of a bit to set
	 *	@tparam _T2 is the second operand (specialization of CCTSize template), containing running value of the mask
	 */
	template <class _T1, class _T2>
	class CAccumulateBitwiseMask {
		/**
		 *	@brief intermediates stored as an enum
		 */
		enum {
			n_mask_value = _T2::n_size | (1 << _T1::n_size) /**< @brief the result (in here to avoid template problems when using the shift left operator) */
			// note that we could save one bit by assuming that there will be no vertices of dimension zero
			// but this is not a pressing problem right now
		};

	public:
		typedef fbs_ut::CCTSize<n_mask_value> _TyResult; /**< @brief result of the addition */
	};

	/**
	 *	@brief intermediates stored as an enum
	 */
	enum {
		n_nonpartite_vertex_type_num = CTypelistLength<CNonPartiteVertexList>::n_result, /**< @brief number of non-partite vertices */
		b_have_nonpartite_vertex_types = (n_nonpartite_vertex_type_num != 0)? 1 : 0, /**< @brief non-partite vertices presence flag */
		n_partite_vertex_type_num = CTypelistLength<CPartiteVertexList>::n_result, /**< @brief number of partite vertices */
		n_partite_vertex_dimension_num = CTypelistLength<CPartiteUniqueVertexDims>::n_result, /**< @brief number of partite vertices recoginzable by their dimension */
		n_partite_unambiguous_vertex_dimension_num = CTypelistLength<CRecognizablePartiteVertexDims>::n_result, /**< @brief number of partite vertices recoginzable by their dimension, which can't be confused for non-partite */

		n_vertex_group_num = n_partite_unambiguous_vertex_dimension_num + b_have_nonpartite_vertex_types, /**< @brief the number of vertex groups (one for non-partite vertices if there are any and then one for each recognizable dimension of partite vertices) */

		b_can_use_guided_ordering = n_partite_unambiguous_vertex_dimension_num > 0, // cannot differentiate between partite and non-pertite vertices, based on their dimension (or maybe there are no partite vertices whatsoever)
		b_can_use_simple_ordering = !b_have_nonpartite_vertex_types && n_partite_vertex_dimension_num == 2, // in case there are only two types of vertices and they are both partite (such as in basic BA)
		b_efficient_guided_ordering = size_t(n_partite_unambiguous_vertex_dimension_num) == size_t(n_partite_vertex_dimension_num), // can differentiate all of the partite vertices (if not, we can still try, but we can get a poor ordering in the sense that the size of the diagonal part will be low, compared to the rest of the system)
		b_perfect_guided_ordering = b_efficient_guided_ordering && size_t(n_partite_vertex_type_num) == size_t(n_partite_vertex_dimension_num), // have perfect ordering (if not, we can potentially run into trouble by ordering "eiffel tower" vertices in the diagonal part, making the Schur complement completely dense)
		// different levels of usability of the guided ordering
		// g++ requires cast to size_t here, to avoid the enum comparison warning

		n_unique_vertex_dim_num = CTypelistLength<CAllUniqueVertexDims>::n_result, /**< @brief number of unique vertex dimensions */

		n_smallest_vertex_dim = CTypelistItemAt<CAllUniqueSortedVertexDims, 0>::_TyResult::n_size,
		n_greatest_vertex_dim = CTypelistItemAt<CAllUniqueSortedVertexDims, n_unique_vertex_dim_num - 1>::_TyResult::n_size,

		n_max_partite_vertex_dimension = CTypelistReduce2<CPartiteUniqueVertexDims,
			fbs_ut::CBinaryScalarMax, fbs_ut::CCTSize<0> >::_TyResult::n_size, /**< @brief maximum dimension of a partite vertex */
		b_can_use_binary_mask_to_detect_partitedness = n_max_partite_vertex_dimension < 32, /**< @brief if set, the partite vertex dimensions can be stored in a bitmask */
		n_partite_vertex_mask = CTypelistReduce2<typename CChooseType<CPartiteUniqueVertexDims,
			CTypelistEnd, b_can_use_binary_mask_to_detect_partitedness>::_TyResult, // use empty list in case the maximum dimension is too large to avoid a bunch of integer overflow warnings
			CAccumulateBitwiseMask, fbs_ut::CCTSize<0> >::_TyResult::n_size /**< @brief bit mask, encoding ones at shifts corresponding to partite vertex dimensions @note This is only valid if b_can_use_binary_mask_to_detect_partitedness is set. */ // value of the partite vertex mask
		// partite vertex detection using a bit mask
	};

	/**
	 *	@brief a simple utility class that calculates block sizes
	 *		in Schur complement ordered using simple guided ordering
	 *	@tparam CLambdaMatrixBlockSizes is list of matrix block sizes in the information matrix (\f$\Lambda\f$)
	 *	@note Use this if \ref b_can_use_simple_ordering is true.
	 */
	template <class CLambdaMatrixBlockSizes>
	class CSimpleOrderingBlockSizes {
	public:
		typedef typename MakeTypelist(fbs_ut::CCTSize2D<n_smallest_vertex_dim,
			n_smallest_vertex_dim>) _TyDBlockSizes; /**< @brief diagonal (landmark) matrix block sizes */
		typedef typename MakeTypelist(fbs_ut::CCTSize2D<n_greatest_vertex_dim,
			n_greatest_vertex_dim>) _TySchurBlockSizes; /**< @brief Schur complement (RCS) matrix block sizes */
		typedef typename MakeTypelist(fbs_ut::CCTSize2D<n_greatest_vertex_dim,
			n_smallest_vertex_dim>) _TyUBlockSizes; /**< @brief upper off-diagonal submatrix block sizes */
		typedef typename MakeTypelist(fbs_ut::CCTSize2D<n_smallest_vertex_dim,
			n_greatest_vertex_dim>) _TyVBlockSizes; /**< @brief lower off-diagonal submatrix block sizes */
		typedef typename CConcatTypelist<_TyUBlockSizes,
			_TyVBlockSizes>::_TyResult _TyOffDiagBlockSizes; /**< @brief off-diagonal submatrix block sizes */
	};

	/**
	 *	@brief utility class that calculates block sizes in Schur
	 *		complement ordered using group-based guided ordering
	 *
	 *	@tparam CLambdaMatrixBlockSizes is list of matrix block sizes in the information matrix (\f$\Lambda\f$)
	 *	@tparam n_D_group_id is index of vertex group that is used for the diagonal (landmark) part,
	 *		must be in <tt>[(\ref b_have_nonpartite_vertex_types)? 1 : 0, \ref n_vertex_group_num)</tt> interval
	 *
	 *	@note Use this if \ref b_can_use_guided_ordering is true but \ref b_can_use_simple_ordering is false.
	 */
	template <class CLambdaMatrixBlockSizes, const int n_D_group_id>
	class CGuidedOrderingBlockSizes {
	public:
		/**
		 *	@brief intermediates, stored as enum
		 */
		enum {
			n_D_group_dim = CTypelistItemAt<CRecognizablePartiteVertexDims,
				n_D_group_id - b_have_nonpartite_vertex_types>::_TyResult::n_size /**< @brief dimension of vertices in the selected group */
		};

		typedef typename MakeTypelist(fbs_ut::CCTSize2D<n_D_group_dim, n_D_group_dim>) _TyDBlockSizes; /**< @brief diagonal (landmark) matrix block sizes */

	protected:
		typedef typename MakeTypelist(fbs_ut::CCTSize<n_D_group_dim>) _TyDBlockSizes_1D; /**< @brief diagonal (landmark) matrix block sizes as 1D CCTSize */
		typedef typename CTypelistDifference<CAllUniqueVertexDims,
			_TyDBlockSizes_1D>::_TyResult _TyOtherDiagBlockSizes; /**< @brief diagonal block sizes in the RCS part */
		typedef typename CCarthesianProductTypelist<_TyOtherDiagBlockSizes,
			_TyDBlockSizes_1D, fbs_ut::CMakeCTSize2DType>::_TyResult _TyAllPossibleUBlockSizes; /**< @brief possible upper off-diagonal block sizes, possibly with some sizes that there are no edges for */
		typedef typename CUniqueTypelist<typename CCarthesianProductTypelist<_TyOtherDiagBlockSizes,
			_TyOtherDiagBlockSizes, fbs_ut::CMakeCTSize2DType>::_TyResult>::_TyResult
			_TyAllPossibleSchurBlockSizes; /**< @brief possible Schur block sizes, possibly with some sizes that there are no edges for */

	public:
		typedef typename CTypelistIntersection<CLambdaMatrixBlockSizes,
			_TyAllPossibleSchurBlockSizes>::_TyResult _TySchurBlockSizes; /**< @brief Schur complement (RCS) matrix block sizes */
		typedef typename CTypelistIntersection<CLambdaMatrixBlockSizes,
			_TyAllPossibleUBlockSizes>::_TyResult _TyUBlockSizes; /**< @brief upper off-diagonal submatrix block sizes */
		typedef typename CTransformTypelist<_TyUBlockSizes,
			fbs_ut::CSize2DToTransposeSize2D>::_TyResult _TyVBlockSizes; /**< @brief lower off-diagonal submatrix block sizes */
		typedef typename CUniqueTypelist<typename CConcatTypelist<_TyUBlockSizes,
			_TyVBlockSizes>::_TyResult>::_TyResult _TyOffDiagBlockSizes; /**< @brief off-diagonal submatrix block sizes */
	};

	/**
	 *	@brief utility class that calculates block sizes in Schur
	 *		complement ordered using group-based guided ordering,
	 *		regardless of which (combination of) group was chosen
	 *	@tparam CLambdaMatrixBlockSizes is list of matrix block sizes in the information matrix (\f$\Lambda\f$)
	 *	@note Use this if \ref b_can_use_guided_ordering is true but \ref b_can_use_simple_ordering is false.
	 */
	template <class CLambdaMatrixBlockSizes>
	class CGuidedOrderingAnyBlockSizes {
	public:
		typedef typename CTransformTypelist<CRecognizablePartiteVertexDims,
			fbs_ut::CMakeCTSize2DSquareType>::_TyResult _TyDBlockSizes; /**< @brief diagonal (landmark) matrix block sizes */

	protected:
		typedef typename CCarthesianProductTypelist<CAllUniqueVertexDims,
			CRecognizablePartiteVertexDims, fbs_ut::CMakeCTSize2DType>::_TyResult _TyAllPossibleUBlockSizes; /**< @brief possible upper off-diagonal block sizes, possibly with some sizes that there are no edges for */

	public:
		typedef CLambdaMatrixBlockSizes _TySchurBlockSizes; /**< @brief Schur complement (RCS) matrix block sizes */
		typedef typename CTypelistIntersection<CLambdaMatrixBlockSizes,
			_TyAllPossibleUBlockSizes>::_TyResult _TyUBlockSizes; /**< @brief upper off-diagonal submatrix block sizes */
		typedef typename CTransformTypelist<_TyUBlockSizes,
			fbs_ut::CSize2DToTransposeSize2D>::_TyResult _TyVBlockSizes; /**< @brief lower off-diagonal submatrix block sizes */
		typedef typename CUniqueTypelist<typename CConcatTypelist<_TyUBlockSizes,
			_TyVBlockSizes>::_TyResult>::_TyResult _TyOffDiagBlockSizes; /**< @brief off-diagonal submatrix block sizes */
	};

	/**
	 *	@brief utility class that calculates block sizes in Schur
	 *		complement ordered using group-based guided ordering,
	 *		regardless of which (combination of) group was chosen
	 *	@tparam CLambdaMatrixBlockSizes is list of matrix block sizes in the information matrix (\f$\Lambda\f$)
	 *	@note Use this if \ref b_can_use_guided_ordering is false.
	 */
	template <class CLambdaMatrixBlockSizes>
	class CGenericOrderingBlockSizes {
	public:
		typedef typename CTransformTypelist<CAllUniqueVertexDims,
			fbs_ut::CMakeCTSize2DSquareType>::_TyResult _TyDBlockSizes; /**< @brief diagonal (landmark) matrix block sizes */
		typedef CLambdaMatrixBlockSizes _TySchurBlockSizes; /**< @brief Schur complement (RCS) matrix block sizes */
		typedef CLambdaMatrixBlockSizes _TyUBlockSizes; /**< @brief upper off-diagonal submatrix block sizes */
		typedef CLambdaMatrixBlockSizes _TyVBlockSizes; /**< @brief lower off-diagonal submatrix block sizes */
		typedef CLambdaMatrixBlockSizes _TyOffDiagBlockSizes; /**< @brief off-diagonal submatrix block sizes */
	};

	template <class CLambdaMatrixBlockSizes, const bool b_can_use_simple_ordering, const bool b_can_use_guided_ordering>
	class CChooseBlockSizeAlgorithm;

	template <class CLambdaMatrixBlockSizes>
	class CChooseBlockSizeAlgorithm<CLambdaMatrixBlockSizes, true, true> :
		public CSimpleOrderingBlockSizes<CLambdaMatrixBlockSizes> {};

	template <class CLambdaMatrixBlockSizes>
	class CChooseBlockSizeAlgorithm<CLambdaMatrixBlockSizes, false, true> :
		public CGuidedOrderingAnyBlockSizes<CLambdaMatrixBlockSizes> {};

	template <class CLambdaMatrixBlockSizes>
	class CChooseBlockSizeAlgorithm<CLambdaMatrixBlockSizes, false, false> :
		public CGenericOrderingBlockSizes<CLambdaMatrixBlockSizes> {};

	template <class CLambdaMatrixBlockSizes>
	class CBlockSizes : public CChooseBlockSizeAlgorithm<CLambdaMatrixBlockSizes,
		b_can_use_simple_ordering, b_can_use_guided_ordering> {};

	/*class CFillDimensions { // moved to fbs_ut
	protected:
		size_t *m_p_dest;

	public:
		CFillDimensions(size_t *p_dest)
			:m_p_dest(p_dest)
		{}

		template <class CDimension>
		void operator ()()
		{
			*m_p_dest = CDimension::n_size;
			++ m_p_dest;
		}

		operator const size_t *() const
		{
			return m_p_dest;
		}
	};*/

public:
	static inline void Get_VertexDimensions(size_t *p_vertex_dimensions, const size_t UNUSED(n_vertex_dimension_num))
	{
		_ASSERTE(n_vertex_dimension_num == n_vertex_group_num);
		// make sure the array is allocated correctly

		if(b_have_nonpartite_vertex_types)
			*p_vertex_dimensions = 0;
		// somehow fill the non-partite group

		//const int *p_end = CTypelistForEach<CPartiteUniqueSortedVertexDims/*CPartiteUniqueVertexDims*/, CFillDimensions>::Run( // todo - is CPartiteUniqueSortedVertexDims correct here?
		//	CFillDimensions(&p_vertex_dimensions[b_have_nonpartite_vertex_types]));
		//_ASSERTE(p_end == &p_vertex_dimensions[0] + sizeof(p_vertex_dimensions) / sizeof(p_vertex_dimensions[0])); // make sure we filled all
		fbs_ut::Copy_CTSizes_to_Array<CRecognizablePartiteVertexDims
			/*CPartiteUniqueSortedVertexDims*/>(p_vertex_dimensions + b_have_nonpartite_vertex_types,
			n_partite_unambiguous_vertex_dimension_num/*n_partite_vertex_dimension_num*/); // use a fancy function
		// gather dimensions of unambiguous partite vertex types
	}

	/**
	 *	@brief determines whether a vertex is partite, based on its dimension
	 *	@param[in] n_vertex_dimension is dimension of the vertex
	 *	@return Returns true if the vertex with the given dimension is partite,
	 *		returns false otherwise.
	 */
	static inline bool b_IsPartite_Vertex(size_t n_vertex_dimension)
	{
		bool b_is_partite_vertex;
		if(!b_have_nonpartite_vertex_types)
			b_is_partite_vertex = true; // there are no nonpartite vertices in this system
		else if(b_can_use_binary_mask_to_detect_partitedness) {
			b_is_partite_vertex = ((1 << n_vertex_dimension) & n_partite_vertex_mask) != 0;
			// find out whether the vertex is partite using a mask - very fast, should be
			// sufficient in most cases (vertex dimensions less than 32 (32 not permitted
			// for compatibility reasons, as the mask is stored as an enum))
		} else {
			b_is_partite_vertex = !CTypelistItemBFind<typename schur_detail::CNonEmptySizeList<CNonPartiteUniqueVertexDims>::_TyResult,
				fbs_ut::CRuntimeCompareScalar, size_t, CBinarySearchResult>::Find(n_vertex_dimension, CBinarySearchResult());
			// find the partite vertex at runtime
		}
		// determine whether the vertex is partite, based on its dimension

		return b_is_partite_vertex;
	}

	/**
	 *	@brief determines which group the vertex should be sorted in, based on its dimension
	 *	@param[in] n_vertex_dimension is dimension of the vertex
	 *	@return Returns vertex group index; the vertices in group 0 are non-partite
	 *		(if there are any such vertices), otherwise the groups of partite vertices
	 *		are ordered by descending dimensionality. There are \ref n_vertex_group_num
	 *		groups.
	 */
	static inline size_t n_Vertex_GroupIndex(size_t n_vertex_dimension)
	{
		bool b_is_partite_vertex = b_IsPartite_Vertex(n_vertex_dimension);
		// determine whether the vertex is partite, based on its dimension

		if(!b_is_partite_vertex)
			return 0; // non-partite vertices all go to the first group
		else {
			// the vertex is partite, need to find out which type of a vertex it is

			typedef CBinarySearchResultWithIndex<CRecognizablePartiteVertexDims/*CPartiteUniqueSortedVertexDims*/> TFindResult;
			size_t n_vertex_type_index = CTypelistItemBFind<typename
				CNonEmptySizeList<CRecognizablePartiteVertexDims/*CPartiteUniqueSortedVertexDims*/>::_TyResult,
				fbs_ut::CRuntimeCompareScalar, size_t,
				TFindResult>::FindExisting(n_vertex_dimension, TFindResult()).n_Index();
			_ASSERTE(n_vertex_type_index < n_partite_unambiguous_vertex_dimension_num/*n_partite_vertex_dimension_num*/);
			// finds the current dimentsion in the list of partite vertex dimension

			//n_vertex_type_index = n_partite_unambiguous_vertex_dimension_num
			//	/*n_partite_vertex_dimension_num*/ - n_vertex_type_index - 1;
			_ASSERTE(n_vertex_type_index < n_partite_unambiguous_vertex_dimension_num
				/*n_partite_vertex_dimension_num*/); // make sure it did not somehow over/underflow
			// do NOT reverse the index so that vertices with higher dimensions are stored in lower index lists
			// the vertices with higher dimension will now be at the end of the list

			if(b_have_nonpartite_vertex_types)
				++ n_vertex_type_index;
			// if there are nonpartite vertices, they are occupying index 0

			return n_vertex_type_index;
		}
	}
};

} // ~schur_detail

/**
 *	@brief dense linear solver model based on Eigen
 */
class CLinearSolver_DenseEigen {
public:
	typedef CBasicLinearSolverTag _Tag; /**< @brief solver type tag */

	typedef Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> _TyDecomposition; /**< @brief matrix factorization type (Cholesky) */

	// on Venice:
	// sparse			20 sec / iteration
	// LLT				9 sec / iteration
	// LDLT				70 sec / iteration
	// PartialPivLU		19 sec / iteration // without parallelization
	// PartialPivLU		10 sec / iteration // with parallelization (8 cores), still slower than serial LLT
	// FullPivLU		486 sec / iteration // very slightly lower chi2
	// HouseholderQR	64 sec / iteration // pivotless

protected:
	Eigen::MatrixXd m_t_lambda; /**< @brief memory for lambda matrix, in order to avoid reallocation */
	_TyDecomposition m_t_factorization; /**< @brief matrix factorization object */

public:
	/**
	 *	@brief default constructor
	 */
	inline CLinearSolver_DenseEigen()
	{};

	/**
	 *	@brief copy constructor (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 */
	inline CLinearSolver_DenseEigen(const CLinearSolver_DenseEigen &UNUSED(r_other))
	{}

	/**
	 *	@brief copy operator (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 *	@return Returns reference to this.
	 */
	inline CLinearSolver_DenseEigen &operator =(const CLinearSolver_DenseEigen &UNUSED(r_other))
	{
		return *this;
	}

	/**
	 *	@brief deletes memory for all the auxiliary buffers and matrices, if allocated
	 */
	void Free_Memory();

	/**
	 *	@brief solves linear system given by positive-definite matrix
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	bool Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta); // throw(std::bad_alloc)

	/**
	 *	@brief calculates inverse of a dense positive-definite matrix using partially pivoted LU
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[out] r_dest is place to store the inverse (can be the same as the source matrix)
	 *	@param[in] r_src is the matrix to be inverted
	 *	@param[in] b_upper_storage is upper-triangular storage flag (if set, the lower triangle of the matrix is ignored)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc or std::runtime_error.
	 */
	template <class CDerived0, class CDerived1>
	static bool Dense_Inverse(Eigen::MatrixBase<CDerived0> &r_dest,
		const Eigen::MatrixBase<CDerived1> &r_src, bool b_upper_storage) // throw(std::bad_alloc, std::runtime_error)
	{
		typedef Eigen::PartialPivLU<Eigen::MatrixXd> LUDecomposition;

		LUDecomposition factorization;
		if(b_upper_storage)
			factorization.compute(r_src.template selfadjointView<Eigen::Upper>());
		else
			factorization.compute(r_src);
		//if(factorization.info() != Eigen::Success) // it always does
		//	return false;
		// LU

		r_dest = factorization.inverse();
		// invert

		return true;
	}

	/**
	 *	@brief calculates inverse of a dense positive-definite
	 *		matrix using partially pivoted LU, in parallel
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[out] r_dest is place to store the inverse (can be the same as the source matrix)
	 *	@param[in] r_src is the matrix to be inverted
	 *	@param[in] b_upper_storage is upper-triangular storage flag (if set, the lower triangle of the matrix is ignored)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc or std::runtime_error.
	 */
	template <class CDerived0, class CDerived1>
	static bool Dense_Inverse_Parallel(Eigen::MatrixBase<CDerived0> &r_dest,
		const Eigen::MatrixBase<CDerived1> &r_src, bool b_upper_storage) // throw(std::bad_alloc, std::runtime_error)
	{
		Start_ParallelEigenSection(); // ...

		bool b_result;
		try {
			b_result = Dense_Inverse(r_dest, r_src, b_upper_storage);
		} catch(std::exception &r_exc) {
			End_ParallelEigenSection();
			throw(r_exc);
		}

		End_ParallelEigenSection();

		return b_result;
	}

protected:
	static void Start_ParallelEigenSection(); // could have used RAII
	static void End_ParallelEigenSection();
};

/**
 *	@brief OpenMP CUDA context guard; pushes CUDA context on construction
 *		and pops it when leaving the scope automatically
 *	@note This needs to be used by user code calling GPU functionality from
 *		multiple threads. Memory allocated in one context cannot be used
 *		by a different thread so some care needs to be taken.
 */
class COMPCUDAContextGuard {
protected:
	bool m_b_pushed; /**< @brief active flag (if set, the destructor will pop the current CUDA context) */

public:
	/**
	 *	@brief default constructor; associates a CUDA context with the current OpenMP thread
	 *	@note This function throws std::runtime_error.
	 */
	COMPCUDAContextGuard(); // throw(std::runtime_error)

	/**
	 *	@brief constructor; associates a CUDA context with the current OpenMP thread
	 *	@param[in] b_activate is context activate flag (if not set, this has no effect)
	 *	@note This function throws std::runtime_error.
	 */
	COMPCUDAContextGuard(bool b_activate); // throw(std::runtime_error)

	/**
	 *	@brief destructor; pops the current CUDA context
	 *	@note This function throws std::runtime_error.
	 */
	~COMPCUDAContextGuard(); // throw(std::runtime_error)

protected:
	COMPCUDAContextGuard(const COMPCUDAContextGuard &r_other); // no-copy
	COMPCUDAContextGuard &operator =(const COMPCUDAContextGuard &r_other); // no-copy
};

/**
 *	@brief dense linear solver model, running on a GPU
 */
class CLinearSolver_DenseGPU {
public:
	typedef CBasicLinearSolverTag _Tag; /**< @brief solver type tag */

protected:
	Eigen::MatrixXd m_t_lambda; /**< @brief memory for lambda matrix, in order to avoid reallocation */
	static bool b_cula_initialized; /**< @brief determines whether CULA is initialized */
	static size_t n_instance_num; /**< @brief number of instances of CLinearSolver_DenseGPU */

	/**
	 *	@brief holds GPU data
	 */
	struct TGPUData {
		double *p_lambda_storage; /**< @brief array for storing the dense lambda matrix */
		double *p_rhs_storage; /**< @brief array for storing the right hand side vector */
		size_t n_lambda_size; /**< @brief number of rows / columns in p_lambda_storage and number of rows in p_rhs_storage */
		//int *p_LU_perm; /**< @brief LU permutation */
	};

	TGPUData m_gpu; /**< @brief GPU data */

public:
	/**
	 *	@brief default constructor; initializes CULA
	 *	@note This function throws std::runtime_error.
	 */
	CLinearSolver_DenseGPU(); // throw(std::runtime_error)

	/**
	 *	@brief copy constructor (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 */
	CLinearSolver_DenseGPU(const CLinearSolver_DenseGPU &UNUSED(r_other));

	/**
	 *	@brief destructor; uninitializes CULA
	 */
	~CLinearSolver_DenseGPU();

	/**
	 *	@brief copy operator (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from (unused)
	 *	@return Returns reference to this.
	 */
	inline CLinearSolver_DenseGPU &operator =(const CLinearSolver_DenseGPU &UNUSED(r_other))
	{
		return *this;
	}

	/**
	 *	@brief deletes memory for all the auxiliary buffers and matrices, if allocated
	 */
	void Free_Memory();

	/**
	 *	@brief solves linear system given by positive-definite matrix
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc or std::runtime_error.
	 */
	bool Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_eta); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief calculates inverse of a dense positive-definite matrix
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[out] r_dest is place to store the inverse (can be the same as the source matrix)
	 *	@param[in] r_src is the matrix to be inverted
	 *	@param[in] b_upper_storage is upper-triangular storage flag (if set, the lower triangle of the matrix is ignored)
	 *	@param[in] b_try_Cholesky is Cholesky inverse flag (if set, the matrix will be inverted using Cholesky,
	 *		otherwise using LU)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc or std::runtime_error.
	 */
	template <class CDerived0, class CDerived1>
	bool Dense_Inverse(/*Eigen::MatrixBase<*/CDerived0/*>*/ &r_dest,
		const /*Eigen::MatrixBase<*/CDerived1/*>*/ &r_src,
		bool b_upper_storage, bool b_try_Cholesky = false) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(r_src.rows() == r_src.cols()); // make sure it is square
		if(&r_dest != &r_src)
			r_dest.resize(r_src.rows(), r_src.cols());
		// make sure src and dest are the same size

		return Dense_Inverse(r_dest.data(), r_src.data(),//&r_dest(0, 0), &r_src(0, 0),
			r_src.cols(), b_upper_storage, b_try_Cholesky);
		// call the version with pointers
	}

	/**
	 *	@brief calculates inverse of a dense positive-definite matrix
	 *
	 *	@param[out] p_dest is pointer to the output matrix elements (can be the same as the source matrix)
	 *	@param[in] p_src is pointer to the input matrix elements
	 *	@param[in] n is size of the source matrix (must be square)
	 *	@param[in] b_upper_storage is upper-triangular storage flag (if set, the lower triangle of the matrix is ignored)
	 *	@param[in] b_try_Cholesky is Cholesky inverse flag (if set, the matrix will be inverted using Cholesky,
	 *		otherwise using LU)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc or std::runtime_error.
	 */
	bool Dense_Inverse(double *p_dest, const double *p_src, const size_t n,
		bool b_upper_storage, bool b_try_Cholesky = false); // throw(std::bad_alloc, std::runtime_error)
};

/**
 *	@brief linear solver model based on Schur complement, running on a GPU
 */
class CLinearSolver_Schur_GPUBase {
public:
	typedef CLinearSolver_DenseGPU _TyBaseSolver; /**< @brief name of the base linear solver */
	typedef _TyBaseSolver::_Tag _TyBaseSolverTag; /**< @brief linear solver tag */
	typedef CLinearSolverWrapper<_TyBaseSolver, _TyBaseSolverTag> _TyLinearSolverWrapper; /**< @brief wrapper for the base linear solvers (shields solver capability to solve blockwise) */

protected:
	static bool b_cuda_initialized; /**< @brief determines whether CUDA is initialized */
	static size_t n_instance_num; /**< @brief number of instances of CLinearSolver_Schur_GPUBase */

	/**
	 *	@brief holds GPU data
	 */
	struct TGPUData {
		void *t_cusparse; /**< @brief cuSparse handle */ // cusparseHandle_t
		void *t_cublas; /**< @brief CUBLAS handle */ // cublasHandle_t
		void *t_matrix_descr; /**< @brief cuSparse general matrix descriptor */ // cusparseMatDescr_t
		void *t_sym_matrix_descr; /**< @brief cuSparse symmetric matrix descriptor */ // cusparseMatDescr_t
		cs *p_A; /**< @brief sparse storage for U / V parts of Schur complement @note This always uses 32-bit indices, even in x64. */
		cs *p_B; /**< @brief sparse storage for D part of Schur complement @note This always uses 32-bit indices, even in x64. */
		int *csrRowPtrD; /**< @brief array for compressed row indices of the product */
		int csrRowPtrD_size; /**< @brief size of csrRowPtrD, in elements */
		int nnzD; /**< @brief number of nonzeros of the product */

		/**
		 *	@brief default constructor; clears all the members to zero
		 */
		TGPUData()
			:t_cusparse(0), t_cublas(0), t_matrix_descr(0), t_sym_matrix_descr(0),
			p_A(0), p_B(0), csrRowPtrD(0), csrRowPtrD_size(0), nnzD(0)
		{}
	};

	TGPUData m_gpu; /**< @brief GPU data */

public:
	/**
	 *	@brief default constructor; increments instance counter
	 */
	CLinearSolver_Schur_GPUBase();

	/**
	 *	@brief destructor; decrements instance counter, deletes CUDA objects
	 */
	~CLinearSolver_Schur_GPUBase();

	/**
	 *	@brief solve a linear system using Schur complement on a GPU
	 *
	 *	@param[in] r_lambda is the system matrix
	 *	@param[in,out] r_v_eta is the right hand side vector on input and solution on output
	 *	@param[in] b_keep_ordering is ordering change flag (if set, the ordering is the same as in the last iteration)
	 *	@param[in] n_landmark_dimension is dimension of the landmark vertices (or -1 in case there are multiple
	 *		different dimensions; this is used for the diagonal inverse optimization in case the landmarks are 3D)
	 *	@param[in] m_double_workspace is workspace for permuting the right hand side (allocated inside this function)
	 *	@param[in] m_order is ordering vector
	 *	@param[in] m_n_matrix_cut is size of the reduced camera system, in vertices / blocks
	 *	@param[in,out] m_linear_solver is instance of the linear solver for solving the reduced camera system
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	bool GPUSolve(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_v_eta,
		bool b_keep_ordering, size_t n_landmark_dimension, std::vector<double> &m_double_workspace,
		const std::vector<size_t> &m_order, const size_t m_n_matrix_cut, _TyBaseSolver &m_linear_solver); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief fast block matrix product on GPU
	 *
	 *	This is not as efficient as it could be, the GPU data is allocated and then deleted.
	 *	when doing multiple products, allocations and transfers can possibly be saved.
	 */
	bool SpDGEMM(CUberBlockMatrix &r_dest, const CUberBlockMatrix &r_a,
		const CUberBlockMatrix &r_b, bool b_upper_diag_only = false);
};

/**
 *	@brief linear solver model based on Schur complement (a wrapper for another linear solver)
 *
 *	@tparam CBaseSolver is a type of basic linear solver,
 *		to be used to solve the Schur complement subproblem
 *	@tparam CAMatrixBlockSizes is typelist, containing Eigen
 *		matrices with known compile-time sizes (unused, kept only for backward compatibility)
 *	@tparam CSystem is optimized system type
 */
template <class CBaseSolver, class CAMatrixBlockSizes, class CSystem>
class CLinearSolver_Schur {
public:
	typedef CBlockwiseLinearSolverTag _Tag; /**< @brief solver type tag */
#if defined(__SCHUR_USE_DENSE_SOLVER) || defined(__SCHUR_DENSE_SOLVER_USE_GPU)
#ifdef __SCHUR_DENSE_SOLVER_USE_GPU
	typedef CLinearSolver_DenseGPU _TyBaseSolver; /**< @brief name of the base linear solver */
#else // __SCHUR_DENSE_SOLVER_USE_GPU
	typedef CLinearSolver_DenseEigen _TyBaseSolver; /**< @brief name of the base linear solver */
#endif // __SCHUR_DENSE_SOLVER_USE_GPU
#else // __SCHUR_USE_DENSE_SOLVER || __SCHUR_DENSE_SOLVER_USE_GPU
	typedef CBaseSolver _TyBaseSolver; /**< @brief name of the base linear solver */
#endif // __SCHUR_USE_DENSE_SOLVER || __SCHUR_DENSE_SOLVER_USE_GPU

	//typedef typename CUniqueTypelist<CAMatrixBlockSizes>::_TyResult _TyAMatrixBlockSizes; /**< @brief list of block matrix sizes */
	//typedef typename fbs_ut::CBlockMatrixTypesAfterPreMultiplyWithSelfTranspose<
	//	_TyAMatrixBlockSizes>::_TyResult _TyLambdaMatrixBlockSizes; /**< @brief possible block matrices, found in lambda and L */
	//typedef typename _TyBaseSolver::_TySystem::_TyHessianMatrixBlockList _TyLambdaMatrixBlockSizes; // could use that instead, but there might be other prior knowledge of block sizes

	typedef typename CSystem::_TyVertexTypelist _TyVertexTypelist; /**< @brief list of vertex types */
	typedef typename CSystem::_TyEdgeTypelist _TyEdgeTypelist; /**< @brief list of edge types */

	typedef typename CSystem::_TyHessianMatrixBlockList _TyLambdaMatrixBlockSizes; /**< @brief possible block matrices, found in lambda and L */

	typedef typename _TyBaseSolver::_Tag _TyBaseSolverTag; /**< @brief linear solver tag */

	typedef schur_detail::CGuidedOrdering_Helper<_TyVertexTypelist, _TyEdgeTypelist> _TyGOH; /**< @brief guided ordering helper */

protected:
	_TyBaseSolver m_linear_solver; /**< @brief linear solver */

	typedef CLinearSolverWrapper<_TyBaseSolver, _TyBaseSolverTag> _TyLinearSolverWrapper; /**< @brief wrapper for the base linear solvers (shields solver capability to solve blockwise) */

	std::vector<double> m_double_workspace; /**< @brief temporary workspace, used to reorder the vectors (not used outside Solve_PosDef_Blocky()) */
	std::vector<size_t> m_order_workspace; /**< @brief temporary workspace, used to invert the ordering (not used outside CalculateOrdering()) */
	std::vector<size_t> m_order; /**< @brief ordering that separates matrix into diagonal part and dense parts */
	size_t m_n_matrix_cut; /**< @brief separation between the diagonal part and the dense parts (in blocks) */
	bool m_b_base_solver_reorder; /**< @brief base solver reordering flag */
	bool m_b_single_landmark_size; /**< @brief single landmark size flag (if set, the diagonal part of the matrix contains only square blocks of m_n_landmark_size) */
	size_t m_n_landmark_size; /**< @brief dimension of the landmark vertex (only valid if m_b_single_landmark_size is set) */

#if /*(defined(_WIN32) || defined(_WIN64)) &&*/ !defined(__DISABLE_GPU) && !defined(__GPU_SCHUR_NO_PRODS)
	CLinearSolver_Schur_GPUBase m_gpu_schur; /**< @brief GPU Schur solver state */
#endif // /*(_WIN32 || _WIN64) &&*/ !__DISABLE_GPU && !defined(__GPU_SCHUR_NO_PRODS)

public:
	/**
	 *	@brief default constructor (does nothing)
	 */
	inline CLinearSolver_Schur(const /*_TyBaseSolver*/CBaseSolver &UNUSED(r_solver))
		:/*m_linear_solver(r_solver),*/ m_b_base_solver_reorder(true), m_b_single_landmark_size(false)
	{}

	/**
	 *	@brief copy constructor (has no effect)
	 *	@param[in] r_other is the solver to copy from
	 */
	inline CLinearSolver_Schur(const CLinearSolver_Schur &r_other)
		:m_linear_solver(r_other.m_linear_solver), m_b_base_solver_reorder(true), m_b_single_landmark_size(false)
	{}

	/**
	 *	@brief deletes memory for all the auxiliary buffers and matrices, if allocated
	 */
	void Free_Memory()
	{
		m_linear_solver.Free_Memory();
		{
			std::vector<double> empty;
			m_double_workspace.swap(empty);
		}
		{
			std::vector<size_t> empty;
			m_order_workspace.swap(empty);
		}
		{
			std::vector<size_t> empty;
			m_order.swap(empty);
		}
	}

	/**
	 *	@brief copy operator (has no effect; memory for lambda not copied)
	 *	@param[in] r_other is the solver to copy from
	 *	@return Returns reference to this.
	 */
	CLinearSolver_Schur &operator =(const CLinearSolver_Schur &r_other)
	{
		m_linear_solver = r_other.m_linear_solver;
		return *this;
	}

	/**
	 *	@brief solves linear system given by positive-definite matrix using Schur complement
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_v_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	bool Solve_PosDef(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_v_eta) // throw(std::bad_alloc)
	{
		SymbolicDecomposition_Blocky(r_lambda); // calculate ordering
		// calculate the ordering

		return Solve_PosDef_Blocky(r_lambda, r_v_eta); // solve
	}

	/**
	 *	@brief deletes symbolic factorization, if calculated (forces a symbolic
	 *		factorization update in the next call to Solve_PosDef_Blocky())
	 */
	inline void Clear_SymbolicDecomposition()
	{
		m_order.clear();
		m_b_base_solver_reorder = true;
		m_b_single_landmark_size = false;
		m_n_matrix_cut = size_t(-1);
	}

	/**
	 *	@brief calculates symbolic factorization of a block
	 *		matrix for later (re)use in solving it
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@return Returns true on success, false on failure.
	 */
	inline bool SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
	{
		return SymbolicDecomposition_Blocky(r_lambda, false);
	}

	/**
	 *	@brief calculates symbolic factorization of a block
	 *		matrix for later (re)use in solving it
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in] b_force_guided_ordering is guided ordering flag
	 *		(if set, the landmarks are determined based on their dimension)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool SymbolicDecomposition_Blocky(const CUberBlockMatrix &r_lambda, bool b_force_guided_ordering) // throw(std::bad_alloc)
	{
		m_b_base_solver_reorder = true;
		// will need to reorder later on

		if(m_order.capacity() < r_lambda.n_BlockColumn_Num()) {
			m_order.clear(); // prevent copying
			m_order.reserve(std::max(2 * m_order.capacity(), r_lambda.n_BlockColumn_Num()));
		}
		m_order.resize(r_lambda.n_BlockColumn_Num());
		// allocate the ordering array

		if(_TyGOH::b_can_use_guided_ordering) { // count of 2 also implies that the sizes are different
			m_n_matrix_cut = n_Calculate_GuidedOrdering(m_order_workspace, r_lambda); // it is a member function now, requires different lists we'd made
			/*m_n_matrix_cut = CSchurOrdering::n_Calculate_GuidedOrdering(m_order_workspace,
				vertex_Size_Big, vertex_Size_Small, r_lambda);*/
			// this is the "guided ordering", need to have two types of vertices with a different dimensionality
			// note this may give inferior results, i.e. on landmark datasets like Victoria park
		} else if(b_force_guided_ordering)
			throw std::runtime_error("guided ordering forced but unable to distinguish landmarks"); // todo - handle this better
		if(!b_force_guided_ordering && (!_TyGOH::b_can_use_guided_ordering || m_n_matrix_cut >= r_lambda.n_BlockColumn_Num() / 2)) { // pose-only without -po also tries to use guided ordering (but there are no landmarks - wasted time)
			m_n_matrix_cut = CSchurOrdering::n_Calculate_Ordering(m_order_workspace, r_lambda);
			// this is full ordering, using the graph structure, always gives the best results

			m_b_single_landmark_size = (_TyGOH::n_unique_vertex_dim_num == 1);
			m_n_landmark_size = _TyGOH::n_smallest_vertex_dim;//CTypelistItemAt<typename _TyGOH::CAllUniqueVertexDims, 0>::_TyResult::n_size;
			// can still have a single landmark size if all the vertices have the same dimension
			// otherwise can't guarantee it: the vertices can be ordered arbitrarily
		}

		/*printf("debug: Schur inverse size: " PRIsize " vertices (out of " PRIsize ")\n",
			r_lambda.n_BlockColumn_Num() - m_n_matrix_cut, r_lambda.n_BlockColumn_Num());*/
		// see how successful the ordering was

		for(size_t i = 0, n = m_order.size(); i < n; ++ i)
			m_order[m_order_workspace[i]] = i;
		//	m_order[i] = m_order_workspace[i]; // bad ordering - to test inverse of not purely diagonal matrix
		// have to use inverse ordering for matrix reordering

		return true;
	}

public:
	/**
	 *	@brief solves linear system given by positive-definite matrix
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_v_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This enables a reuse of previously calculated symbolic factorization,
	 *		it can be either calculated in SymbolicDecomposition_Blocky(), or it is
	 *		calculated automatically after the first call to this function,
	 *		or after Clear_SymbolicDecomposition() was called (preferred).
	 */
	bool Solve_PosDef_Blocky(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_v_eta) // throw(std::bad_alloc)
	{
		//typename CDebug::CMatrixDeltaTracker track(r_lambda);

		if(r_lambda.n_BlockColumn_Num() != m_order.size())
			SymbolicDecomposition_Blocky(r_lambda);
		// in case the ordering is nonconforming, calculate a new one

		if(m_order.size() == 1)
			return m_linear_solver.Solve_PosDef(r_lambda, r_v_eta);
		// in case there is only a single variable, pass it to the simple linear solver

		if(m_n_matrix_cut == 0 || m_n_matrix_cut == m_order.size()) {
			CLinearSolver_UberBlock<_TyLambdaMatrixBlockSizes> solver;
			return solver.Solve_PosDef_Blocky(r_lambda, r_v_eta);
		}
		// in case the matrix is diagonal (and thus the partitioning is ambiguous),
		// solve it efficiently with a sparse solver

		bool b_keep_ordering = !m_b_base_solver_reorder;
		m_b_base_solver_reorder = false; // only once, or until the ordering changes
		// in case we need to reorder the base linear solver as well

		/*Eigen::VectorXd sol_copy = r_v_eta;
		_TyBaseSolver solver;
		solver.Solve_PosDef(r_lambda, sol_copy);*/
		// calculate reference "good" solution

		//bool b_result = Schur_Solve(r_lambda, r_v_eta, r_v_eta, &m_order[0],
		//	m_order.size(), m_n_matrix_cut, b_keep_ordering); // solve

		/*sol_copy -= r_v_eta;
		printf("error in solution is %g (head %g, tail %g)\n",
			sol_copy.norm(), sol_copy.head(m_n_matrix_cut * 3).norm(),
			sol_copy.tail((r_lambda.n_BlockColumn_Num() - m_n_matrix_cut) * 2).norm());*/
		// calculate difference in the solution

		//return b_result;

#if /*(defined(_WIN32) || defined(_WIN64)) &&*/ !defined(__DISABLE_GPU) && !defined(__GPU_SCHUR_NO_PRODS)

		if(m_b_single_landmark_size) {
			return m_gpu_schur.GPUSolve(r_lambda, r_v_eta, b_keep_ordering,
				m_n_landmark_size, m_double_workspace, m_order, m_n_matrix_cut, m_linear_solver);
		}
		// if the landmarks are of a single size, solve it on a GPU

#endif // /*(_WIN32 || _WIN64) &&*/ !__DISABLE_GPU && !defined(__GPU_SCHUR_NO_PRODS)

		_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
		_ASSERTE(r_v_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

		_ASSERTE(r_lambda.b_SymmetricLayout());
		_ASSERTE(r_lambda.n_BlockColumn_Num() == m_order.size());
		_ASSERTE((m_order.empty() && !m_n_matrix_cut) || (!m_order.empty() &&
			m_n_matrix_cut > 0 /*&& m_n_matrix_cut < SIZE_MAX*/ && m_n_matrix_cut /*+ 1*/ < m_order.size()));
		_ASSERTE(r_v_eta.rows() == r_lambda.n_Column_Num());

#ifdef __SCHUR_PROFILING
		CDeltaTimer dt;
#endif // __SCHUR_PROFILING

		//track(r_lambda);

		CUberBlockMatrix lambda_perm;
		r_lambda.Permute_UpperTriangular_To(lambda_perm, &m_order[0], m_order.size(), true);
		// reorder the matrix

		//track(r_lambda);

#ifdef __SCHUR_PROFILING
		double f_reperm_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		const size_t n = lambda_perm.n_BlockColumn_Num();

		CUberBlockMatrix A, U, C, V;
		const size_t n_matrix_cut = m_n_matrix_cut; // antialiass
		lambda_perm.SliceTo(A, 0, n_matrix_cut, 0, n_matrix_cut, true);
		lambda_perm.SliceTo(U, 0, n_matrix_cut, n_matrix_cut, n, true);
		lambda_perm.SliceTo(C, n_matrix_cut, n, n_matrix_cut, n, true);

#ifdef __SCHUR_PROFILING
		double f_slice_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		U.TransposeTo(V); // because lower-triangular of lambda is not calculated
		// cut Lambda matrix into pieces
		// \lambda = | A U |
		//           | V C |

#ifdef __SCHUR_PROFILING
		double f_transpose_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		//track(r_lambda);

		CUberBlockMatrix C_inv_storage; // only used if C is not diagonal
		bool b_is_diagonal = C.b_BlockDiagonal();
		CUberBlockMatrix &C_inv = /*(b_is_diagonal)? C :*/ C_inv_storage; // can do diagonal // do *not* reuse storage, will modify lambda!
		if(b_is_diagonal)
			C_inv.InverseOf_BlockDiag_FBS_Parallel<_TyLambdaMatrixBlockSizes>(C); // faster version, slightly less general
		else
			C_inv.InverseOf_Symmteric_FBS<_TyLambdaMatrixBlockSizes>(C, true); // C is block diagonal (should also be symmetric)
		// inverse of C

		//track(r_lambda);

#ifdef __SCHUR_PROFILING
		double f_inverse_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		C_inv.Scale(-1.0);	// -U*(C^-1)

#ifdef __SCHUR_PROFILING
		double f_scale_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		//track(r_lambda);

		CUberBlockMatrix minus_U_Cinv;
		U.MultiplyToWith_FBS<_TyLambdaMatrixBlockSizes,
			_TyLambdaMatrixBlockSizes>(minus_U_Cinv, C_inv);	// U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense

#ifdef __SCHUR_PROFILING
		double f_mul0_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		/*minus_U_Cinv.Scale(-1.0);	// -U*(C^-1)

		double f_scale_time = dt.f_Time();*/

		//track(r_lambda);

		CUberBlockMatrix schur_compl; // not needed afterwards
		minus_U_Cinv.MultiplyToWith_FBS<_TyLambdaMatrixBlockSizes,
			_TyLambdaMatrixBlockSizes>(schur_compl, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part

#ifdef __SCHUR_PROFILING
		double f_mul1_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		//track(r_lambda);

		A.AddTo_FBS<_TyLambdaMatrixBlockSizes>(schur_compl); // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
		// compute left-hand side A - U(C^-1)V
		// todo - need multiplication with transpose matrices (in this case schur * U^T)

#ifdef __SCHUR_PROFILING
		double f_add_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		// note that the sum and difference of two symmetric matrices is again symmetric,
		// but this is not always true for the product

		/*lambda_perm.Save_MatrixMarket("lambda_perm.mtx", "lambda_perm.bla");
		A.Save_MatrixMarket("lambda_perm00.mtx", "lambda_perm00.bla");
		U.Save_MatrixMarket("lambda_perm01.mtx", "lambda_perm01.bla");
		V.Save_MatrixMarket("lambda_perm10.mtx", "lambda_perm10.bla");
		C.Save_MatrixMarket("lambda_perm11.mtx", "lambda_perm11.bla");
		C_inv.Save_MatrixMarket("lambda_perm11_inv.mtx", "lambda_perm11_inv.bla");
		schur_compl.Save_MatrixMarket("schur.mtx", "schur.bla");*/
		/*lambda_perm.Rasterize("schur0_lambda_perm.tga", 3);
		A.Rasterize("schur1_lambda_perm00.tga", 3);
		U.Rasterize("schur2_lambda_perm01.tga", 3);
		V.Rasterize("schur3_lambda_perm10.tga", 3);
		C.Rasterize("schur4_lambda_perm11.tga", 3);
		schur_compl.Rasterize("schur5_A-(UC-1V).tga", 3);
		C_inv.Rasterize("schur6_lambda_perm11_inv.tga", 3);*/
		// debug // note that C gets overwritten by Cinv, need to save that one before!!

		size_t n_rhs_vector_size = r_lambda.n_Column_Num();
		size_t n_pose_vector_size = A.n_Column_Num(); // 6 * m_n_matrix_cut;
		size_t n_landmark_vector_size = U.n_Column_Num(); // 3 * (n - m_n_matrix_cut);
		// not block columns! element ones

		//track(r_lambda);

		if(m_double_workspace.capacity() < n_rhs_vector_size) {
			m_double_workspace.clear(); // avoid data copying
			m_double_workspace.reserve(std::max(2 * m_double_workspace.capacity(), n_rhs_vector_size));
		}
		m_double_workspace.resize(n_rhs_vector_size);
		double *p_double_workspace = &m_double_workspace[0];
		// alloc workspace

		//track(r_lambda);

		lambda_perm.InversePermute_RightHandSide_Vector(p_double_workspace,
			&r_v_eta(0), n_rhs_vector_size, &m_order[0], m_order.size());
		// need to permute the vector !!

		//track(r_lambda);

		Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
		Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace
		// get eta and cut it into pieces
		// \eta = | x |
		//        | l |

		// we are now solving:
		// \lambda          \eta
		// | A U | | dx | = | x |
		// | V C | | dl |   | l |

		minus_U_Cinv.PreMultiply_Add_FBS<_TyLambdaMatrixBlockSizes>(&v_x(0),
			n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
		// compute right-hand side x - U(C^-1)l

#ifdef __SCHUR_PROFILING
		double f_RHS_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		//track(r_lambda);

		if(!b_keep_ordering)
			_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, schur_compl); // the ordering on schur_compl will not change, can calculate it only in the first pass and then reuse
		bool b_result;
		try {
			b_result = _TyLinearSolverWrapper::Solve(m_linear_solver, schur_compl, v_x);
		} catch(std::bad_alloc &r_exc) {
#if defined(__SCHUR_USE_DENSE_SOLVER) || defined(__SCHUR_DENSE_SOLVER_USE_GPU)
			fprintf(stderr, "error: dense linear solver threw "
				"std::bad_alloc (%s): fallback to sparse\n", r_exc.what());
			CLinearSolver_UberBlock<_TyLambdaMatrixBlockSizes> solver;
			b_result = solver.Solve_PosDef_Blocky(schur_compl, v_x);
#else // __SCHUR_USE_DENSE_SOLVER || __SCHUR_DENSE_SOLVER_USE_GPU
			throw r_exc; // in case it was thrown by a sparse solver, just rethrow
#endif // __SCHUR_USE_DENSE_SOLVER || __SCHUR_DENSE_SOLVER_USE_GPU
		}
		Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
		// solve for dx = A - U(C^-1)V / x

#ifdef __SCHUR_PROFILING
		double f_linsolve0_time = dt.f_Time();
#endif // __SCHUR_PROFILING

		// note that schur_compl only contains pose-sized blocks when guided ordering is used! could optimize for that
		// also note that schur_compl is not completely dense if it is not many times smaller than C

		Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
		// obtained the first part of the solution

		Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // calculated inplace
		v_l = -v_l; // trades setZero() for sign negation and a smaller sign negation above

		/*Eigen::VectorXd v_l_copy = v_l;
		V.PreMultiply_Add_FBS<_TyLambdaMatrixBlockSizes>(&v_l_copy(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx*/
		U.PostMultiply_Add_FBS_Parallel<_TyLambdaMatrixBlockSizes>(&v_l(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // dx^T * U^T = (V * dx)^T and the vector transposes are ignored
		//printf("post-mad error %g (rel %g)\n", (v_l_copy - v_l).norm(), (v_l_copy - v_l).norm() / v_l.norm());

		// l = V * dx - l
		v_dl.setZero();
		C_inv.PreMultiply_Add/*_FBS<_TyLambdaMatrixBlockSizes>*/(&v_dl(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
		// solve for dl = (C^-1)(l - V * dx) = -(C^-1)(V * dx - l)
		// the second part of the solution is calculated inplace in the dest vector

		lambda_perm.Permute_RightHandSide_Vector(&r_v_eta(0), p_double_workspace,
			n_rhs_vector_size, &m_order[0], m_order.size());
		// permute back!

#ifdef __SCHUR_PROFILING
		double f_linsolve1_time = dt.f_Time();
		double f_totel_time = f_reperm_time + f_slice_time + f_transpose_time +
			f_inverse_time + f_mul0_time + f_scale_time + f_mul1_time +
			f_add_time + f_RHS_time + f_linsolve0_time + f_linsolve1_time;

		printf("Schur took %f sec, out of which:\n", f_totel_time);
		printf("   reperm: %f\n", f_reperm_time);
		printf("    slice: %f\n", f_slice_time);
		printf("transpose: %f\n", f_transpose_time);
		printf("  inverse: %f\n", f_inverse_time);
		printf(" multiply: %f, out of which:\n", f_mul0_time + f_scale_time + f_mul1_time);
		printf("\tdiag gemm: %f\n", f_mul0_time);
		printf("\t    scale: %f\n", f_scale_time);
		printf("\t     gemm: %f\n", f_mul1_time);
		printf("      add: %f\n", f_add_time);
		printf(" RHS prep: %f\n", f_RHS_time);
		printf("  cholsol: %f (" PRIsize " x " PRIsize ", " PRIsize " nnz (%.2f %%))\n",
			f_linsolve0_time, schur_compl.n_Row_Num(), schur_compl.n_Column_Num(),
			schur_compl.n_NonZero_Num(), 100 * float(schur_compl.n_NonZero_Num()) /
			(schur_compl.n_Row_Num() * schur_compl.n_Column_Num()));
		printf(" dy solve: %f\n", f_linsolve1_time);
		// debug - do some profiling
#endif // __SCHUR_PROFILING

		/*static size_t n_iter = 0;
		if(!n_iter) {
			std::string s_name;
			{
				char p_s_it_nr[256];
				sprintf(p_s_it_nr, "schur/lambda_perm_" PRIsize "_", n_iter);
				s_name = p_s_it_nr;
				++ n_iter;
			}
			A.Save_MatrixMarket((s_name + "00.mtx").c_str(), (s_name + "00.bla").c_str());
			U.Save_MatrixMarket((s_name + "01.mtx").c_str(), (s_name + "01.bla").c_str());
			V.Save_MatrixMarket((s_name + "10.mtx").c_str(), (s_name + "10.bla").c_str());
			C.Save_MatrixMarket((s_name + "11.mtx").c_str(), (s_name + "11.bla").c_str());
			C_inv.Save_MatrixMarket((s_name + "11_inv.mtx").c_str(), (s_name + "11_inv.bla").c_str());
			schur_compl.Save_MatrixMarket((s_name + "schur.mtx").c_str(), (s_name + "schur.bla").c_str());
		}*/
		// debug - dump the matrices

		//track(r_lambda);

		return b_result;
	}

	/**
	 *	@brief solves linear system given by positive-definite matrix, marginalizes out the poses
	 *
	 *	@param[in] r_lambda is positive-definite matrix
	 *	@param[in,out] r_v_eta is the right-side vector, and is overwritten with the solution
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This enables a reuse of previously calculated symbolic factorization,
	 *		it can be either calculated in SymbolicDecomposition_Blocky(), or it is
	 *		calculated automatically after the first call to this function,
	 *		or after Clear_SymbolicDecomposition() was called (preferred).
	 *	@note This only solves for the landmarks (the vertices with lower dimension),
	 *		the solution for poses is zeroed out. This is faster than Solve_PosDef_Blocky().
	 *	@note This requires ordering to be guided by the dimensionality, use
	 *		SymbolicDecomposition_Blocky(const CUberBlockMatrix&, bool) with
	 *		<tt>b_force_guided_ordering</tt> set to <tt>true</tt>.
	 */
	bool Solve_PosDef_Blocky_MarginalPoses(const CUberBlockMatrix &r_lambda, Eigen::VectorXd &r_v_eta) // throw(std::bad_alloc)
	{
		if(r_lambda.n_BlockColumn_Num() != m_order.size())
			SymbolicDecomposition_Blocky(r_lambda, true); // force guided!
		// in case the ordering is nonconforming, calculate a new one

		//bool b_keep_ordering = !m_b_base_solver_reorder;
		//m_b_base_solver_reorder = false; // only once, or until the ordering changes
		//// in case we need to reorder the base linear solver as well
		// do not touch this, we do no cholesky now

		//bool b_result = Schur_Solve_MarginalPoses(r_lambda, r_v_eta, r_v_eta, &m_order[0],
		//	m_order.size(), m_n_matrix_cut/*, b_keep_ordering*/); // fast solve

#if 1 // solve vertex-only system
//		_ASSERTE(vertex_Size_Num == 2); // otherwise the landmarks could not have been distinguisjed (they could, but it would not be clear what are we marginalizing out)
		// todo - will have to think about this a bit harder, possibly pass more parameters

		_ASSERTE(r_lambda.b_SymmetricLayout()); // pos-def is supposed to be symmetric
		_ASSERTE(r_v_eta.rows() == r_lambda.n_Row_Num()); // make sure the vector has correct dimension

		_ASSERTE(r_lambda.b_SymmetricLayout());
		_ASSERTE(r_lambda.n_BlockColumn_Num() == m_order.size());
		_ASSERTE((m_order.empty() && !m_n_matrix_cut) || (!m_order.empty() &&
			m_n_matrix_cut > 0 && m_n_matrix_cut < SIZE_MAX && m_n_matrix_cut + 1 < m_order.size()));
		_ASSERTE(r_v_eta.rows() == r_lambda.n_Column_Num());

		CUberBlockMatrix lambda_perm;
		r_lambda.Permute_UpperTriangular_To(lambda_perm,
			&m_order[0], m_order.size(), true);
		// reorder the matrix

		const size_t n = lambda_perm.n_BlockColumn_Num();

		CUberBlockMatrix /*A, U,*/ C/*, V*/;
		/*lambda_perm.SliceTo(A, 0, n_matrix_cut, 0, n_matrix_cut, true);
		lambda_perm.SliceTo(U, 0, n_matrix_cut, n_matrix_cut, n, true);*/
		lambda_perm.SliceTo(C, m_n_matrix_cut, n, m_n_matrix_cut, n, true);
		/*U.TransposeTo(V);*/ // because lower-triangular of lambda is not calculated
		/*CUberBlockMatrix &A = lambda_perm;
		A.SliceTo(A, n_matrix_cut, n_matrix_cut, true);*/ // can't, would free data that U, C and V references
		// cut Lambda matrix into pieces
		// \lambda = | A U |
		//           | V C |

#if 1
		CUberBlockMatrix C_inv;
		C_inv.InverseOf_Symmteric_FBS<_TyLambdaMatrixBlockSizes>(C, true); // C is block diagonal (should also be symmetric)
#else // 1
		CUberBlockMatrix &C_inv = C;
		C_inv.InverseOf_BlockDiag_FBS_Parallel<_TyLambdaMatrixBlockSizes>(C); // faster version, slightly less general
#endif // 1
		// inverse of C

		/*CUberBlockMatrix unity, u_ref;
		unity.ProductOf(C, C_inv);
		unity.CopyLayoutTo(u_ref);
		u_ref.SetIdentity();
		unity.AddTo(u_ref, -1);
		double f_error = u_ref.f_Norm();
		fprintf(stderr, "error of matrix inverse is: %g\n", f_error);*/
		// check inverse

		/*CUberBlockMatrix minus_U_Cinv;
		U.MultiplyToWith_FBS<_TyLambdaMatrixBlockSizes,
			_TyLambdaMatrixBlockSizes>(minus_U_Cinv, C_inv);	// U*(C^-1) // U is not symmetric, it is rather dense, tmp is again dense
		minus_U_Cinv.Scale(-1.0);	// -U*(C^-1)
		CUberBlockMatrix schur_compl; // not needed afterwards
		minus_U_Cinv.MultiplyToWith_FBS<_TyLambdaMatrixBlockSizes,
			_TyLambdaMatrixBlockSizes>(schur_compl, V, true); // -U*(C^-1)V // UV is symmetric, the whole product should be symmetric, calculate only the upper triangular part
		A.AddTo_FBS<_TyLambdaMatrixBlockSizes>(schur_compl);*/ // -U*(C^-1)V + A // A is symmetric, if schur_compl is symmetric, the difference also is
		// compute left-hand side A - U(C^-1)V
		// todo - need multiplication with transpose matrices (in this case schur * U^T)

		// note that the sum and difference of two symmetric matrices is again symmetric,
		// but this is not always true for the product

		/*lambda_perm.Save_MatrixMarket("lambda_perm.mtx", "lambda_perm.bla");
		A.Save_MatrixMarket("lambda_perm00.mtx", "lambda_perm00.bla");
		U.Save_MatrixMarket("lambda_perm01.mtx", "lambda_perm01.bla");
		V.Save_MatrixMarket("lambda_perm10.mtx", "lambda_perm10.bla");
		C.Save_MatrixMarket("lambda_perm11.mtx", "lambda_perm11.bla");
		C_inv.Save_MatrixMarket("lambda_perm11_inv.mtx", "lambda_perm11_inv.bla");
		schur_compl.Save_MatrixMarket("schur.mtx", "schur.bla");*/
		/*lambda_perm.Rasterize("schur0_lambda_perm.tga", 3);
		A.Rasterize("schur1_lambda_perm00.tga", 3);
		U.Rasterize("schur2_lambda_perm01.tga", 3);
		V.Rasterize("schur3_lambda_perm10.tga", 3);
		C.Rasterize("schur4_lambda_perm11.tga", 3);
		schur_compl.Rasterize("schur5_A-(UC-1V).tga", 3);
		C_inv.Rasterize("schur6_lambda_perm11_inv.tga", 3);*/
		// debug

		size_t n_rhs_vector_size = r_lambda.n_Column_Num();
		size_t n_landmark_vector_size = C.n_Column_Num(); // 3 * (n - n_matrix_cut);
		size_t n_pose_vector_size = n_rhs_vector_size - n_landmark_vector_size; // 6 * n_matrix_cut;
		// not block columns! element ones

		if(m_double_workspace.capacity() < n_rhs_vector_size) {
			m_double_workspace.clear(); // avoid data copying
			m_double_workspace.reserve(std::max(2 * m_double_workspace.capacity(), n_rhs_vector_size));
		}
		m_double_workspace.resize(n_rhs_vector_size);
		double *p_double_workspace = &m_double_workspace[0];
		// alloc workspace

		lambda_perm.InversePermute_RightHandSide_Vector(p_double_workspace,
			&r_v_eta(0), n_rhs_vector_size, &m_order[0], m_order.size());
		// need to permute the vector !!

		//Eigen::VectorXd v_x = Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size); // don't really need a copy, but need Eigen::VectorXd for _TyLinearSolverWrapper::Solve()
		Eigen::VectorXd v_l = Eigen::Map<Eigen::VectorXd>(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // need a copy, need one vector of workspace
		// get eta and cut it into pieces
		// \eta = | x |
		//        | l |

		// we are now solving:
		// \lambda          \eta
		// | A U | | dx | = | x |
		// | V C | | dl |   | l |

		/*minus_U_Cinv.PreMultiply_Add_FBS<_TyLambdaMatrixBlockSizes>(&v_x(0),
			n_pose_vector_size, &v_l(0), n_landmark_vector_size); // x - U(C^-1)l
		// compute right-hand side x - U(C^-1)l

		if(!b_reuse_block_structure)
			_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, schur_compl); // the ordering on schur_compl will not change, can calculate it only in the first pass and then reuse
		bool b_result = _TyLinearSolverWrapper::Solve(m_linear_solver, schur_compl, v_x);
		Eigen::VectorXd &v_dx = v_x; // rename, solves inplace
		// solve for dx = A - U(C^-1)V / x

		// note that schur_compl only contains pose-sized blocks when guided ordering is used! could optimize for that
		// also note that schur_compl is not completely dense if it is not many times smaller than C

		Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size) = v_dx; // an unnecessary copy, maybe could work around
		// obtained the first part of the solution

		Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // calculated inplace
		v_dl.setZero();
		V.PreMultiply_Add_FBS<_TyLambdaMatrixBlockSizes>(&v_dl(0),
			n_landmark_vector_size, &v_dx(0), n_pose_vector_size); // V * dx
		v_l -= v_dl; // (l - dl)
		v_dl.setZero();
		C_inv.PreMultiply_Add_FBS<_TyLambdaMatrixBlockSizes>(&v_dl(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // (C^-1)(l - V * dx)
		// solve for dl = (C^-1)(l - V * dx)
		// the second part of the solution is calculated inplace in the dest vector*/

		Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size).setZero();
		// the dx for poses is zero

		Eigen::Map<Eigen::VectorXd> v_dl(p_double_workspace +
			n_pose_vector_size, n_landmark_vector_size); // calculated inplace
		v_dl.setZero();
		C_inv.PreMultiply_Add_FBS<_TyLambdaMatrixBlockSizes>(&v_dl(0),
			n_landmark_vector_size, &v_l(0), n_landmark_vector_size); // C^-1 * eta
		// the dl is just C^-1 times eta, simple

		lambda_perm.Permute_RightHandSide_Vector(&r_v_eta(0), p_double_workspace,
			n_rhs_vector_size, &m_order[0], m_order.size());
		// permute back!

		// todo - do some profiling

		return true; // no cholesky, no not pos def will ever happen
#else // just zero out the camera component
		bool b_result = Solve_PosDef_Blocky(r_lambda, r_v_eta);
		// reuse code

		_ASSERTE(m_double_workspace.size() == r_lambda.n_Column_Num());
		double *p_double_workspace = &m_double_workspace[0];

		lambda_perm.InversePermute_RightHandSide_Vector(p_double_workspace,
			&r_v_eta(0), n_rhs_vector_size, &m_order[0], m_order.size());
		// need to permute the vector

		Eigen::Map<Eigen::VectorXd>(p_double_workspace, n_pose_vector_size).setZero();
		// the poses are forcibly zeroed-out

		lambda_perm.Permute_RightHandSide_Vector(&r_v_eta(0), p_double_workspace,
			n_rhs_vector_size, &m_order[0], m_order.size());
		// permute back

		return b_result;
#endif
	}

protected:
	/**
	 *	@brief calculates ordering guided by vertex dimensions (corresponds to lambda block sizes)
	 *
	 *	@param[out] r_ordering is ordering vector (allocated and filled in this function)
	 *	@param[in] r_lambda is the lambda matrix to be ordered
	 *
	 *	@return Returns the number of vertices in the reduced camera system (in vertices / blocks, not in elements).
	 */
	size_t n_Calculate_GuidedOrdering(std::vector<size_t> &r_ordering,
		const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
	{
		const size_t n = r_lambda.n_BlockColumn_Num();
		if(r_ordering.capacity() < n) {
			r_ordering.clear();
			r_ordering.reserve(std::max(2 * r_ordering.capacity(), n));
		}
		r_ordering.resize(n);
		// allocate space

		_ASSERTE(_TyGOH::b_can_use_guided_ordering);
		static bool b_warned_about_ordering = false;
		if(!b_warned_about_ordering) {
			b_warned_about_ordering = true;
			if(!_TyGOH::b_efficient_guided_ordering) {
				fprintf(stderr, "warning: guided Schur ordering with ambiguous vertex dimensions:"
					" may result in low speedups\n");
				// if this happens, make sure that the size of the schur complement is what it should be.
				// if it is not, consider disabling the guided ordering and using MIS always to save time.
			} /*else if(!_TyGOH::b_perfect_guided_ordering) {
				fprintf(stderr, "warning: guided Schur ordering with ambiguous vertex dimensions:"
					" the Schur complement may be dense\n");
				// if this happens, make sure that the sparsity of the schur complement is reasonable.
				// if it is completely dense, try using dense Cholesky factorization instead of sparse.
			}*/ // does not happen anymore since CIsPartiteVertex_ByDim is used
			_ASSERTE(_TyGOH::b_perfect_guided_ordering ==
				_TyGOH::b_efficient_guided_ordering); // make sure that it indeed does not happen
		}
		// warn about less-than-perfect orderings in order to help solving performance issues

		size_t p_vertex_dimensions[_TyGOH::n_vertex_group_num];
		_TyGOH::Get_VertexDimensions(p_vertex_dimensions, sizeof(p_vertex_dimensions) / sizeof(p_vertex_dimensions[0]));
		// gather dimensions of partite vertex types

		size_t n_nonpartite_dimension_sum = 0;
		// sum of dimensions of all the nonpartite vertices

		std::vector<size_t> p_vertex_type_indices[_TyGOH::n_vertex_group_num]; // the first list is for the non-partite vertices, if there are any
		// the same size as p_vertex_dimensions, except that they are ordered in the opposite direction

		if(_TyGOH::b_can_use_simple_ordering) {
			enum {
				n_landmark_dim = _TyGOH::n_smallest_vertex_dim,
				n_pose_dim = _TyGOH::n_greatest_vertex_dim
			};
			_ASSERTE(n_landmark_dim == p_vertex_dimensions[0] && n_pose_dim == p_vertex_dimensions[1]);

			m_b_single_landmark_size = true;
			m_n_landmark_size = n_landmark_dim/*p_vertex_dimensions[0]*/;

			return CSchurOrdering::n_Calculate_GuidedOrdering(r_ordering,
				n_pose_dim/*p_vertex_dimensions[1]*/, n_landmark_dim/*p_vertex_dimensions[0]*/, r_lambda);
		}
		// t_odo - write a specialized implementation for ideal simple bundle adjustment
		// (all cameras at the beginning, all points at the end)
		// we actually already had that implemented ;)

		for(size_t i = 0; i < n; ++ i) {
			size_t n_vertex_i_dimension = r_lambda.n_BlockColumn_Column_Num(i);
			// get vertex dimension

			size_t n_vertex_i_group = _TyGOH::n_Vertex_GroupIndex(n_vertex_i_dimension);
			// calculate which group the vertex belongs to

			if(!n_vertex_i_group && _TyGOH::b_have_nonpartite_vertex_types) { // in case it is a non-partite vertex
				_ASSERTE(!_TyGOH::b_IsPartite_Vertex(n_vertex_i_dimension));
				n_nonpartite_dimension_sum += n_vertex_i_dimension; // not sure if this is useful for anything
				p_vertex_type_indices[0].push_back(i); // add it to the list of non-partite vertices
			} else {
				_ASSERTE(_TyGOH::b_IsPartite_Vertex(n_vertex_i_dimension));
				p_vertex_type_indices[n_vertex_i_group].push_back(i);
				// add it to the list of partite vertices of the given type
			}
		}
		// sort all the vertex types in the matrix to a list

		if(_TyGOH::b_have_nonpartite_vertex_types)
			r_ordering.swap(p_vertex_type_indices[0]);
		else
			r_ordering.clear();
		// the ordering starts with the non-partite vertex types

#if 0
		size_t n_pose_num = 0;
		size_t n_partite_vertex_num = n - r_ordering.size();
		for(int n_pass = 0; n_pass < 2; ++ n_pass) {
			for(int i = 0; i < (int)n_partite_vertex_dimension_num; ++ i) {
				std::vector<size_t> &r_vertex_type_i_indices =
					p_vertex_type_indices[i + b_have_nonpartite_vertex_types];
				// indices of vertex of this type

				size_t n_vertex_num = r_vertex_type_i_indices.size();
				if((!n_pass && n_partite_vertex_num / n_vertex_num > 4) ||
				   (n_pass && n_partite_vertex_num / n_vertex_num <= 4)) { // todo - make this threshold a parameter of the solver
					if(r_ordering.empty())
						r_ordering.swap(r_vertex_type_i_indices);
					else {
						r_ordering.insert(r_ordering.end(), r_vertex_type_i_indices.begin(),
							r_vertex_type_i_indices.end());
					}
				}
				// in the first pass:
				// in case the vertices are partite, but there is only a few of them, then they are
				// likely seen by most of the other vertices, and would create a very dense Schur
				// complement if ordered in the diagonal part: keep them in the Schur complement
				// in the second pass:
				// add the rest of the vertices that were not chosen in the first pass. these will
				// form the diagonal matrix
			}

			if(!n_pass)
				n_pose_num = r_ordering.size();
			// at the end of the first pass, the ordering contains all the vertices that
			// will form the Schur complement; the rest of the vertices will form the
			// diagonal part
		}
		// this is flawed: this will not form a diagonal part, unless all of the partite vertices
		// are independent of each other. if there are possible edges between these vertices,
		// then the inverse of D would be completely dense!
		// need to have a test of which partite vertex types can be combined in the diagonal
		// part together (todo - do this when there is such a system available)

		// if we had a function which finds which pairs of vertices are connected by an edge,
		// this function would form a graph. we would need to enumerate all independent sets
		// of this graph and choose such a one, which has the greatest dimension of vertices,
		// corresponding to the vertex types in that set (a recursion of the MIS problem that
		// is being solved by this function).

		m_b_single_landmark_size = false;
		// not sure how this branch works. better be safe.
#else // 0
		std::vector<size_t> *p_VT_indices = &p_vertex_type_indices[0] + _TyGOH::b_have_nonpartite_vertex_types;
		const size_t *p_VT_dimensions = &p_vertex_dimensions[0] + _TyGOH::b_have_nonpartite_vertex_types;
		int n_largest_partite_vertices = 0;
		size_t n_largest_partite_vertices_size = p_VT_indices[0].size() * p_VT_dimensions[0/*_TyGOH::n_partite_unambiguous_vertex_dimension_num/ *n_partite_vertex_dimension_num* / - 0 - 1*/]; // the dimensions are NOT reversed in the loops above anyore!
		for(int i = 1; i < (int)_TyGOH::n_partite_unambiguous_vertex_dimension_num/*n_partite_vertex_dimension_num*/; ++ i) {
			size_t n_ith_partite_vertices_size = p_VT_indices[i].size() * p_VT_dimensions[i/*_TyGOH::n_partite_unambiguous_vertex_dimension_num/ *n_partite_vertex_dimension_num* / - i - 1*/]; // the dimensions are NOT reversed in the loops above anyore!
			if(n_ith_partite_vertices_size > n_largest_partite_vertices_size) {
				n_largest_partite_vertices = i;
				n_largest_partite_vertices_size = n_ith_partite_vertices_size;
			}
		}
		// find the index of the largest partite vertices

		for(int i = 0; i < (int)_TyGOH::n_partite_unambiguous_vertex_dimension_num/*n_partite_vertex_dimension_num*/; ++ i) {
			if(i == n_largest_partite_vertices)
				continue;
			// skip the largest ones

			std::vector<size_t> &r_vertex_type_i_indices = p_VT_indices[i];
			// indices of vertex of this type

			if(!_TyGOH::b_have_nonpartite_vertex_types && r_ordering.empty()) // if we have nonpartite verts, then the ordering wont be empty anymore
				r_ordering.swap(r_vertex_type_i_indices);
			else {
				r_ordering.insert(r_ordering.end(), r_vertex_type_i_indices.begin(),
					r_vertex_type_i_indices.end());
			}
		}
		// gather all the types of vertices that are less numerous, those will all go to the
		// Schur complement, as they could potentially make the diagonal part not diagonal
		// (we do not have a test for which vertices would do that implemented, as of yet,
		// but in BA, the less plentiful cameras and camera parameters all connect to the
		// (plentiful) landmarks and therefore need to be excluded from the diagonal and
		// this code therefore serves us extremely well)

		size_t n_pose_num = r_ordering.size();
		// now the ordering contains all the vertices that will form the Schur complement;
		// the rest of the vertices will form the diagonal part

		r_ordering.insert(r_ordering.end(),
			p_VT_indices[n_largest_partite_vertices].begin(),
			p_VT_indices[n_largest_partite_vertices].end());
		// add the largest partite at the end to form the diagonal

		m_b_single_landmark_size = true;
		m_n_landmark_size = p_VT_dimensions[/*_TyGOH::n_partite_unambiguous_vertex_dimension_num*//*n_partite_vertex_dimension_num*//* -*/
			n_largest_partite_vertices /*- 1*/]; // the dimensions are NOT reversed in the loops above anyore!
		// have a single landmark size
#endif // 0

		/*size_t n_pose_num = 0, n_landmark_num = 0;
		{
			size_t i = 0;

			for(; i < n; ++ i) {
				if(r_lambda.n_BlockColumn_Column_Num(i) == n_pose_vertex_dimension) {
					r_ordering[n_pose_num] = i;
					++ n_pose_num;
				} else
					break;
			}
			// look for all the poses

			n_landmark_num = n_pose_num; // offset the destination index
			for(; i < n; ++ i) {
				if(r_lambda.n_BlockColumn_Column_Num(i) == n_landmark_vertex_dimension) {
					r_ordering[n_landmark_num] = i;
					++ n_landmark_num;
				} else
					break;
			}
			n_landmark_num -= n_pose_num; // offset back
			// look for all the landmarks (assume landmarks are smaller than poses)

			if(i < n) {
				std::vector<size_t> &r_poses = r_ordering; // keep only poses in the destination ordering
				std::vector<size_t> landmarks(n - n_pose_num); // allocate space for the remaining landmarks
				// get memory

				std::copy(r_ordering.begin() + n_pose_num, r_ordering.begin() +
					(n_pose_num + n_landmark_num), landmarks.begin());
				// copy the landmarks away to a second array

				for(; i < n; ++ i) {
					if(r_lambda.n_BlockColumn_Column_Num(i) == n_pose_vertex_dimension) {
						r_poses[n_pose_num] = i;
						++ n_pose_num;
					} else {
						_ASSERTE(r_lambda.n_BlockColumn_Column_Num(i) == n_landmark_vertex_dimension);
						landmarks[n_landmark_num] = i;
						++ n_landmark_num;
					}
				}
				// loop through the rest of the vertices

				std::copy(landmarks.begin(), landmarks.begin() + n_landmark_num,
					r_ordering.begin() + n_pose_num);
				// copy the landmarks back
			}
		}
		_ASSERTE(n_pose_num + n_landmark_num == r_lambda.n_BlockColumn_Num());*/
		// calculate the simple guided ordering (assumes that landmarks have smaller
		// dimension than poses, and that landmarks are not connected among each other)

		return n_pose_num;
	}
};

/** @} */ // end of group

#endif // !__LINEAR_SOLVER_SCHUR_INCLUDED
