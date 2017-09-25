/*
								+-----------------------------------+
								|                                   |
								|   ***  Schur linear solver  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|      LinearSolver_Schur.cpp       |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/LinearSolver_Schur.cpp
 *	@brief blocky linear solver wrapper that enables Schur complement optimization
 *	@author -tHE SWINe-
 *	@date 2015-10-07
 */

#include "slam/LinearSolver_Schur.h"
#include <map>
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

/*
 *								=== CSchurOrdering ===
 */

#ifdef HAVE_IGRAPH

#include "igraph/igraph.h"

std::vector<std::vector<size_t> > CSchurOrdering::t_Find_Cliques_igraph(const cs *p_graph,
	int n_min_clique_size /*= 2*/, int n_max_clique_size /*= INT_MAX*/) // throw(std::runtime_error)
{
	const size_t n = p_graph->n;
	if(n > uint64_t(1) << 53) // igraph_real_t wouldn't hold integers this big
		throw std::runtime_error("graph too latge for igraph");

	std::vector<igraph_real_t> edges;
	edges.reserve(p_graph->p[n] + 2 * n); // a guess, there is only a half of elements required to represent edges but the matrix is symmetric so it cancels out. allocate more in case the diagonal is not present.
	for(size_t i = 0; i < n; ++ i) {
		_ASSERTE(CSchurOrdering::b_IsSortedSet(&p_graph->i[p_graph->p[i]], &p_graph->i[p_graph->p[i + 1]]));
		for(size_t p = p_graph->p[i], pe = p_graph->p[i + 1]; p < pe && size_t(p_graph->i[p]) < i; ++ p) {
			edges.push_back((igraph_real_t)i);
			_ASSERTE(size_t(edges.back()) == i); // make sure no data is lost
			edges.push_back((igraph_real_t)p_graph->i[p]);
			_ASSERTE(size_t(edges.back()) == p_graph->i[p]); // make sure no data is lost
		}
	}
	// convert the cs matrix to a list of edges

	igraph_vector_t e;
	igraph_real_t f_dummy; // so that e doesn't have null pointers
	igraph_vector_view(&e, (edges.empty())? &f_dummy : &edges[0], long(edges.size())); // from edge list
	// wrap it with igraph structures

	igraph_t g;
	igraph_create(&g, &e, 0, IGRAPH_UNDIRECTED);
	// build a graph (probably calculates other stuff)

	igraph_vector_ptr_t clique_set = IGRAPH_VECTOR_PTR_NULL;
	igraph_vector_ptr_init(&clique_set, 0);
	igraph_maximal_cliques(&g, &clique_set, n_min_clique_size, n_max_clique_size);

	std::vector<std::vector<size_t> > linear_clique_set;

	try {
		const size_t n_clique_num = igraph_vector_ptr_size(&clique_set);
		linear_clique_set.resize(n_clique_num);
		for(size_t i = 0; i < n_clique_num; ++ i) {
			igraph_vector_t *clique_i = (igraph_vector_t*)igraph_vector_ptr_e(&clique_set, long(i));

			const size_t clique_i_size = igraph_vector_size(clique_i);
			linear_clique_set[i].resize(clique_i_size);
			for(size_t j = 0; j < clique_i_size; ++ j)
				linear_clique_set[i][j] = size_t(igraph_vector_e(clique_i, long(j)));
			std::sort(linear_clique_set[i].begin(), linear_clique_set[i].end()); // want those sorted
		}
		// copy to STL
	} catch(std::bad_alloc &r_exc) {
		igraph_destroy(&g);
		igraph_vector_ptr_destroy(&clique_set);

		throw r_exc; // rethrow
	}

	igraph_destroy(&g);
	igraph_vector_ptr_destroy(&clique_set);

#ifdef _DEBUG
	std::map<size_t, size_t> clique_size_hist;
	for(size_t j = 0, m = linear_clique_set.size(); j < m; ++ j)
		++ clique_size_hist[linear_clique_set[j].size()]; // debug
	for(std::map<size_t, size_t>::const_iterator p_size_it = clique_size_hist.begin(),
	   p_end_it = clique_size_hist.end(); p_size_it != p_end_it; ++ p_size_it)
		printf("igraph cliques of size " PRIsize ": " PRIsize "\n", (*p_size_it).first, (*p_size_it).second);
	// debug - make a histogram
#endif // _DEBUG

	return linear_clique_set;
}

std::vector<size_t> CSchurOrdering::t_MIS_igraph(const cs *p_graph, const std::vector<size_t> &r_weights) // throw(std::runtime_error)
{
	const size_t n = p_graph->n;
	if(n > uint64_t(1) << 53) // igraph_real_t wouldn't hold integers this big
		throw std::runtime_error("graph too latge for igraph");

	std::vector<igraph_real_t> edges;
	edges.reserve(p_graph->p[n] + 2 * n); // a guess, there is only a half of elements required to represent edges but the matrix is symmetric so it cancels out. allocate more in case the diagonal is not present.
	for(size_t i = 0; i < n; ++ i) {
		_ASSERTE(CSchurOrdering::b_IsSortedSet(&p_graph->i[p_graph->p[i]], &p_graph->i[p_graph->p[i + 1]]));
		for(size_t p = p_graph->p[i], pe = p_graph->p[i + 1]; p < pe && size_t(p_graph->i[p]) < i; ++ p) {
			edges.push_back((igraph_real_t)i);
			_ASSERTE(size_t(edges.back()) == i); // make sure no data is lost
			edges.push_back((igraph_real_t)p_graph->i[p]);
			_ASSERTE(size_t(edges.back()) == p_graph->i[p]); // make sure no data is lost
		}
	}
	// convert the cs matrix to a list of edges

	igraph_vector_t e;
	igraph_real_t f_dummy; // so that e doesn't have null pointers
	igraph_vector_view(&e, (edges.empty())? &f_dummy : &edges[0], long(edges.size())); // from edge list
	// wrap it with igraph structures

	igraph_t g;
	igraph_create(&g, &e, 0, IGRAPH_UNDIRECTED);
	// build a graph (probably calculates other stuff)

	igraph_vector_ptr_t mis = IGRAPH_VECTOR_PTR_NULL;
	igraph_vector_ptr_init(&mis, 0);
	//igraph_largest_independent_vertex_sets(&g, &mis); // only the largest set(s)
	igraph_maximal_independent_vertex_sets(&g, &mis); // all the sets that cannot be extended by adding another vertex
	// calculate MIS

	size_t n_best_mis = 0, n_best_size = 0;
	const size_t n_set_num = igraph_vector_ptr_size(&mis);
	for(size_t i = 0; i < n_set_num; ++ i) {
		igraph_vector_t *mis_i = (igraph_vector_t*)igraph_vector_ptr_e(&mis, long(i));

		const size_t mis_i_size = igraph_vector_size(mis_i);
		size_t mis_i_size_w = 0;
		for(size_t j = 0; j < mis_i_size; ++ j)
			mis_i_size_w += r_weights[size_t(igraph_vector_e(mis_i, long(j)))];
		if(n_best_size < mis_i_size_w) {
			n_best_size = mis_i_size_w;
			n_best_mis = i;
		}
	}
	// select the best weighted MIS

	igraph_vector_t *best_mis = (igraph_vector_t*)igraph_vector_ptr_e(&mis, long(n_best_mis));
	const size_t best_mis_size = igraph_vector_size(best_mis);
	std::vector<size_t> MIS;
	try {
		MIS.resize(best_mis_size);
		for(size_t j = 0; j < best_mis_size; ++ j)
			MIS[j] += size_t(igraph_vector_e(best_mis, long(j)));
		// convert to STL vector
	} catch(std::bad_alloc &r_exc) {
		igraph_destroy(&g);
		igraph_vector_ptr_destroy(&mis);

		throw r_exc; // rethrow
	}

	igraph_destroy(&g);
	igraph_vector_ptr_destroy(&mis);

	return MIS;
}

std::vector<size_t> CSchurOrdering::t_MIS_igraph(const cs *p_graph) // throw(std::runtime_error) // version without weights
{
	const size_t n = p_graph->n;
	if(n > uint64_t(1) << 53) // igraph_real_t wouldn't hold integers this big
		throw std::runtime_error("graph too latge for igraph");

	std::vector<igraph_real_t> edges;
	edges.reserve(p_graph->p[n] + 2 * n); // a guess, there is only a half of elements required to represent edges but the matrix is symmetric so it cancels out. allocate more in case the diagonal is not present.
	for(size_t i = 0; i < n; ++ i) {
		_ASSERTE(CSchurOrdering::b_IsSortedSet(&p_graph->i[p_graph->p[i]], &p_graph->i[p_graph->p[i + 1]]));
		for(size_t p = p_graph->p[i], pe = p_graph->p[i + 1]; p < pe && size_t(p_graph->i[p]) < i; ++ p) {
			edges.push_back((igraph_real_t)i);
			_ASSERTE(size_t(edges.back()) == i); // make sure no data is lost
			edges.push_back((igraph_real_t)p_graph->i[p]);
			_ASSERTE(size_t(edges.back()) == p_graph->i[p]); // make sure no data is lost
		}
	}
	// convert the cs matrix to a list of edges

	igraph_vector_t e;
	igraph_real_t f_dummy; // so that e doesn't have null pointers
	igraph_vector_view(&e, (edges.empty())? &f_dummy : &edges[0], long(edges.size())); // from edge list
	// wrap it with igraph structures

	igraph_t g;
	igraph_create(&g, &e, 0, IGRAPH_UNDIRECTED);
	// build a graph (probably calculates other stuff)

	igraph_vector_ptr_t mis = IGRAPH_VECTOR_PTR_NULL;
	igraph_vector_ptr_init(&mis, 0);
	igraph_largest_independent_vertex_sets(&g, &mis); // only the largest set(s)
	//igraph_maximal_independent_vertex_sets(&g, &mis); // all the sets that cannot be extended by adding another vertex
	// calculate MIS

	size_t n_best_mis = 0, n_best_size = 0;
	const size_t n_set_num = igraph_vector_ptr_size(&mis);
	for(size_t i = 0; i < n_set_num; ++ i) {
		igraph_vector_t *mis_i = (igraph_vector_t*)igraph_vector_ptr_e(&mis, long(i));

		const size_t mis_i_size = igraph_vector_size(mis_i);
		size_t mis_i_size_w = mis_i_size; // unit weights
		if(n_best_size < mis_i_size_w) {
			n_best_size = mis_i_size_w;
			n_best_mis = i;
		}
	}
	// select the best weighted MIS

	igraph_vector_t *best_mis = (igraph_vector_t*)igraph_vector_ptr_e(&mis, long(n_best_mis));
	const size_t best_mis_size = igraph_vector_size(best_mis);
	std::vector<size_t> MIS;
	try {
		MIS.resize(best_mis_size);
		for(size_t j = 0; j < best_mis_size; ++ j)
			MIS[j] += size_t(igraph_vector_e(best_mis, long(j)));
		// convert to STL vector
	} catch(std::bad_alloc &r_exc) {
		igraph_destroy(&g);
		igraph_vector_ptr_destroy(&mis);

		throw r_exc; // rethrow
	}

	igraph_destroy(&g);
	igraph_vector_ptr_destroy(&mis);

	return MIS;
}

#endif // HAVE_IGRAPH

static inline bool b_Incident(size_t v0, size_t v1, const cs *p_graph)
{
	_ASSERTE(v0 < v1); // could be equal but that would be a wasted optimization opportunity
	if(p_graph->p[v0 + 1] - p_graph->p[v0] > p_graph->p[v1 + 1] - p_graph->p[v1])
		std::swap(v0, v1);
	const csi *p_where = std::lower_bound(&p_graph->i[p_graph->p[v0]], &p_graph->i[p_graph->p[v0 + 1]], csi(v1));
	return p_where != &p_graph->i[p_graph->p[v0 + 1]] && *p_where == csi(v1);
}

void CSchurOrdering::Suppress_NonmaximalCliques(std::vector<std::vector<size_t> > &linear_clique_list)
{
	bool b_global_list_sorted = true;
	_ASSERTE(linear_clique_list.empty() || !linear_clique_list.front().empty());
	for(size_t j = 1, m = linear_clique_list.size(); j < m; ++ j) {
		_ASSERTE(!linear_clique_list[j].empty());
		if(linear_clique_list[j - 1].front() > linear_clique_list[j].front()) {
			b_global_list_sorted = false;
			break;
		}
	}
	// see if the list is sorted by the first element (cliques generated by the BF algorithm are)

	for(size_t j = 0, m = linear_clique_list.size(); j < m; ++ j) {
		const std::vector<size_t> &A = linear_clique_list[j];
		_ASSERTE(CSchurOrdering::b_IsSortedSet(A.begin(), A.end()));
		const size_t n_size_a = A.size();

		for(size_t k = j + 1; k < m; ++ k) {
			const std::vector<size_t> &B = linear_clique_list[k];
			_ASSERTE(CSchurOrdering::b_IsSortedSet(B.begin(), B.end()));
			const size_t n_size_b = B.size();

			if(n_size_a == n_size_b)
				continue;
			// they are unique; two sets of the same size cant be subsets of one another

			if(A.back() < B.front()) { // the elements of both cliques are sorted and the cliques are also ordered by the leading index in the clique list
				if(b_global_list_sorted) {
#ifdef _DEBUG
					for(++ k; k < m; ++ k) {
						const std::vector<size_t> &B = linear_clique_list[k];
						_ASSERTE(A.back() < B.front()); // make sure all the following ones would be skipped
					}
#endif // _DEBUG
					break; // all the following cliques will also be completely different
				} else
					continue;
			}
			// these are completely different elements, can't possibly be a subset

			_ASSERTE(A != B); // there are no duplicates
			bool b_A_has_unique = false, b_B_has_unique = false;
			size_t a = 0, b = 0;
			/*while(a < n_size_a && b < n_size_b) { // ordered merge, O(n) in the shorter clique size
				if(A[a] < B[b]) {
					++ a;
					b_A_has_unique = true;
				} else if(A[a] > B[b]) {
					++ b;
					b_B_has_unique = true;
				} else {
					++ a;
					++ b;
				}
			}*/ // simple merge
			while(a < n_size_a && b < n_size_b) {
				if(A[a] < B[b]) {
					++ a;
					b_A_has_unique = true;
					while(a < n_size_a && b < n_size_b) { // ordered merge, O(n) in the shorter clique size
						if(A[a] < B[b]) {
							++ a;
							//b_A_has_unique = true; // already set above
						} else if(A[a] > B[b]) {
							++ b;
							b_B_has_unique = true;
							break; // now both have unique elements, early exit
						} else {
							++ a;
							++ b;
						}
					}
					break;
				} else if(A[a] > B[b]) {
					++ b;
					b_B_has_unique = true;
					while(a < n_size_a && b < n_size_b) { // ordered merge, O(n) in the shorter clique size
						if(A[a] < B[b]) {
							++ a;
							b_A_has_unique = true;
							break; // now both have unique elements, early exit
						} else if(A[a] > B[b]) {
							++ b;
							//b_B_has_unique = true; // already set above
						} else {
							++ a;
							++ b;
						}
					}
					break;
				} else {
					++ a;
					++ b;
				}
			} // use specialized code to eliminate repeated assignments and to enable early exit
			b_A_has_unique = b_A_has_unique || a < n_size_a;
			b_B_has_unique = b_B_has_unique || b < n_size_b;
			_ASSERTE(b_A_has_unique || b_B_has_unique); // there must be at least some unique elements otherwise the sets would be equal

			if(!b_B_has_unique) { // this one is probably faster to remove
				linear_clique_list.erase(linear_clique_list.begin() + k);
				-- k;
				-- m;
				_ASSERTE(&A == &linear_clique_list[j]); // make sure it did not change the addresses
				continue;
			} else if(!b_A_has_unique) {
				linear_clique_list.erase(linear_clique_list.begin() + j);
				-- j;
				-- m;
				break;
			}
		}
	}
	// remove cliques that are subsets of other cliques O(n^2) in the number of cliques found in this round
}

/**
 *	@copydoc CSchurOrdering::Suppress_NonmaximalCliques
 *	@note This is the same function as CSchurOrdering::Suppress_NonmaximalCliques() but without some
 *		optimizations, it is intended for per-vertex clique lists which are always sorted and overlapping.
 */
static void Suppress_NonmaximalCliques_Dumb(std::vector<std::vector<size_t> > &linear_clique_list)
{
	/*bool b_global_list_sorted = true;
	_ASSERTE(linear_clique_list.empty() || !linear_clique_list.front().empty());
	for(size_t j = 1, m = linear_clique_list.size(); j < m; ++ j) {
		_ASSERTE(!linear_clique_list[j].empty());
		if(linear_clique_list[j - 1].front() > linear_clique_list[j].front()) {
			b_global_list_sorted = false;
			break;
		}
	}
	// see if the list is sorted by the first element (cliques generated by the BF algorithm are)*/

	for(size_t j = 0, m = linear_clique_list.size(); j < m; ++ j) {
		const std::vector<size_t> &A = linear_clique_list[j];
		_ASSERTE(CSchurOrdering::b_IsSortedSet(A.begin(), A.end()));
		const size_t n_size_a = A.size();

		for(size_t k = j + 1; k < m; ++ k) {
			const std::vector<size_t> &B = linear_clique_list[k];
			_ASSERTE(CSchurOrdering::b_IsSortedSet(B.begin(), B.end()));
			const size_t n_size_b = B.size();

			if(n_size_a == n_size_b)
				continue;
			// they are unique; two sets of the same size cant be subsets of one another

			/*if(A.back() < B.front()) { // the elements of both cliques are sorted and the cliques are also ordered by the leading index in the clique list
				if(b_global_list_sorted) {
#ifdef _DEBUG
					for(++ k; k < m; ++ k) {
						const std::vector<size_t> &B = linear_clique_list[k];
						_ASSERTE(A.back() < B.front()); // make sure all the following ones would be skipped
					}
#endif // _DEBUG
					break; // all the following cliques will also be completely different
				} else
					continue;
			}
			// these are completely different elements, can't possibly be a subset*/

			_ASSERTE(A != B); // there are no duplicates
			bool b_A_has_unique = false, b_B_has_unique = false;
			size_t a = 0, b = 0;
			/*while(a < n_size_a && b < n_size_b) { // ordered merge, O(n) in the shorter clique size
				if(A[a] < B[b]) {
					++ a;
					b_A_has_unique = true;
				} else if(A[a] > B[b]) {
					++ b;
					b_B_has_unique = true;
				} else {
					++ a;
					++ b;
				}
			}*/ // simple merge
			while(a < n_size_a && b < n_size_b) {
				if(A[a] < B[b]) {
					++ a;
					b_A_has_unique = true;
					while(a < n_size_a && b < n_size_b) { // ordered merge, O(n) in the shorter clique size
						if(A[a] < B[b]) {
							++ a;
							//b_A_has_unique = true; // already set above
						} else if(A[a] > B[b]) {
							++ b;
							b_B_has_unique = true;
							break; // now both have unique elements, early exit
						} else {
							++ a;
							++ b;
						}
					}
					break;
				} else if(A[a] > B[b]) {
					++ b;
					b_B_has_unique = true;
					while(a < n_size_a && b < n_size_b) { // ordered merge, O(n) in the shorter clique size
						if(A[a] < B[b]) {
							++ a;
							b_A_has_unique = true;
							break; // now both have unique elements, early exit
						} else if(A[a] > B[b]) {
							++ b;
							//b_B_has_unique = true; // already set above
						} else {
							++ a;
							++ b;
						}
					}
					break;
				} else {
					++ a;
					++ b;
				}
			} // use specialized code to eliminate repeated assignments and to enable early exit
			b_A_has_unique = b_A_has_unique || a < n_size_a;
			b_B_has_unique = b_B_has_unique || b < n_size_b;
			_ASSERTE(b_A_has_unique || b_B_has_unique); // there must be at least some unique elements otherwise the sets would be equal

			if(!b_B_has_unique) { // this one is probably faster to remove
				linear_clique_list.erase(linear_clique_list.begin() + k);
				-- k;
				-- m;
				_ASSERTE(&A == &linear_clique_list[j]); // make sure it did not change the addresses
				continue;
			} else if(!b_A_has_unique) {
				linear_clique_list.erase(linear_clique_list.begin() + j);
				-- j;
				-- m;
				break;
			}
		}
	}
	// remove cliques that are subsets of other cliques O(n^2) in the number of cliques found in this round
}

void CSchurOrdering::PruneCliques_MaxVertexOrder(std::vector<std::vector<size_t> > &r_clique_list, size_t n_vertex_num)
{
	std::vector<std::pair<size_t, size_t> > largest_vertex_clique(n_vertex_num,
		std::pair<size_t, size_t>(size_t(-1), 0));
	for(size_t i = 0, n = r_clique_list.size(); i < n; ++ i) {
		const std::vector<size_t> &r_clique = r_clique_list[i];
		for(size_t j = 0, m = r_clique.size(); j < m; ++ j) {
			const size_t v = r_clique[j];
			if(largest_vertex_clique[v].second < m) {
				largest_vertex_clique[v].first = i;
				largest_vertex_clique[v].second = m;
			}
		}
	}
	// record the largest clique per each vertex

	std::vector<size_t> keep_clique_set;
	for(size_t i = 0; i < n_vertex_num; ++ i) {
		if(largest_vertex_clique[i].first != size_t(-1))
			keep_clique_set.push_back(largest_vertex_clique[i].first);
	}
	std::sort(keep_clique_set.begin(), keep_clique_set.end());
	keep_clique_set.erase(std::unique(keep_clique_set.begin(),
		keep_clique_set.end()), keep_clique_set.end());
	// make a set of cliques to keep

	std::vector<std::vector<size_t> > new_clique_list(keep_clique_set.size());
	for(size_t i = 0, n = keep_clique_set.size(); i < n; ++ i)
		new_clique_list[i].swap(r_clique_list[keep_clique_set[i]]);
	r_clique_list.swap(new_clique_list);
	// swap out the cliques to be kept, strictly less than n_vertex_num of them
}

std::vector<std::vector<size_t> > CSchurOrdering::t_Find_Cliques(const cs *p_graph,
	bool b_final_nonmaximal_clique_removal /*= true*/, size_t n_min_clique_size /*= 2*/,
	size_t n_max_clique_size /*= SIZE_MAX*/) // a brute force algorithm that finds cliques
{
	std::vector<std::vector<size_t> > all_clique_list;
	size_t n_generated_clique_num = 0;

	std::vector<size_t> subgraph, perm, clique; // reuse memory
	for(size_t i = 0, n = p_graph->n; i < n; ++ i) {
		// generate all cliques that go through vertex v

		{
			subgraph.clear();
			subgraph.insert(subgraph.begin(), &p_graph->i[p_graph->p[i]], &p_graph->i[p_graph->p[i + 1]]);
			_ASSERTE(CSchurOrdering::b_IsSortedSet(subgraph.begin(), subgraph.end()));
			std::vector<size_t>::iterator p_where_it = std::lower_bound(subgraph.begin(), subgraph.end(), i);

			if(p_where_it == subgraph.end() || *p_where_it != i)
				p_where_it = subgraph.insert(p_where_it, i);

			subgraph.erase(subgraph.begin(), p_where_it); // erases except p_where_it
			_ASSERTE(subgraph.front() == i && CSchurOrdering::b_IsSortedSet(subgraph.begin(), subgraph.end()));
			// enumerate only such subgraphs where i is the first vertex, other subgraphs will be enumerated by other vertices
		}
		// the cliques will be formed around the adjacent vertices

		_ASSERTE(!subgraph.empty());
		if(subgraph.size() < n_min_clique_size)
			continue;
		// in case the vertex is disconnected, there is no clique

		// enumerate all the permutations of the subgraph and verify that each permutation is a clique itself
		// since the vertices are sorted, at the first vertex that is a non-clique one the permutation can be
		// incremented

		if(subgraph.size() > n_max_clique_size)
			subgraph.resize(n_max_clique_size);
		// subgraph enumeration is the bottleneck here, don't enumerate very large subgraphs

		std::set<std::vector<size_t> > new_clique_set;

		{
			perm.clear();
			perm.resize(subgraph.size());
			std::generate(perm.begin(), perm.end(), CSchurOrdering::CIOTA());
			// generate an identity permutation

			size_t n_max_vertex = subgraph.size();
			size_t n_perm_size = perm.size();
			size_t n_min_changed_vertex = 2;
			//printf("permutations of %d\n", int(n_perm_size)); // debug
			do {
				/*for(size_t j = 0; j < n_perm_size; ++ j)
					printf("%d ", int(perm[j]));
				printf("\n");*/
				// debug

				size_t n_clique_size = n_min_changed_vertex/*2*/; // all the vertices are incident
				for(; n_clique_size < n_perm_size; ++ n_clique_size) {
					bool b_disconnected = false;
					for(size_t j = 1; j < n_clique_size; ++ j) { // all vertices incident to vertex 0, no need to check
						if(!b_Incident(subgraph[perm[j]], subgraph[perm[n_clique_size]], p_graph)) {
							b_disconnected = true;
							break;
						}
					}
					if(b_disconnected) {
						//-- n_clique_size; // disable this and the compensation below
						break;
					}

					/*printf("clique of %d: ", int(n_clique_size + 1));
					for(size_t j = 0; j <= n_clique_size; ++ j)
						printf("%d ", int(subgraph[perm[j]]));
					printf("\n");*/
					// debug
				}
				//if(n_clique_size < n_perm_size)
				//	++ n_clique_size; // the loop finished by break rather than by exhausting the iterations
				// detect cliques as prefixes of this permutation
				// t_odo - not sure if this actually gets all the cliques, there could be some suffix ones
				// (should get some example graphs and write a true brute-force algorithm using Combinat.h
				// and see if all are enumerated) // it does get the same cliques as igraph

				if(n_clique_size >= n_min_clique_size) {
					clique.clear();
					clique.resize(n_clique_size);
					for(size_t j = 0; j < n_clique_size; ++ j)
						clique[j] = subgraph[perm[j]];
					new_clique_set.insert(clique);
				}
				// accumulate cliques generated here, there will be some duplicates

				for(size_t j = std::min(n_clique_size + 1, n_perm_size); /*j > 0*/;) { // can skip some work as changing the suffix does no good in case the prefix wont pass the clique test in the next iteration. now we feel smart.
					_ASSERTE(j);
					-- j; // here
					if(++ perm[j] == n_max_vertex) {
						_ASSERTE(j); // since we are not going to count with perm[0], it should never overflow
						continue; // carry
					} else {
						n_min_changed_vertex = std::max(j, size_t(2));
						// see where to restart the clique check in the next round

						_ASSERTE(perm[j] < n_max_vertex);
						n_perm_size = j + n_max_vertex - perm[j]; // the permutation gets shorter
						for(++ j; j < n_perm_size; ++ j)
							perm[j] = perm[j - 1] + 1; // no carry, regenerate the rest of the sorted sequence
						_ASSERTE(perm[n_perm_size - 1] < n_max_vertex); // make sure this is a valid permutation
						break;
					}
				}
				// increment the permutation
			} while(perm.front() == 0); // only permutations where the first vertex is not
		}

		std::vector<std::vector<size_t> > linear_clique_list;
		linear_clique_list.insert(linear_clique_list.begin(),
			new_clique_set.begin(), new_clique_set.end());
		Suppress_NonmaximalCliques_Dumb(linear_clique_list);
		// remove cliques that are subsets of other cliques O(n^2) in the number of cliques found in this round

		all_clique_list.insert(all_clique_list.end(),
			linear_clique_list.begin(), linear_clique_list.end());
		n_generated_clique_num += new_clique_set.size();

		//for(std::set<std::vector<size_t> >::const_iterator p_clique_it = new_clique_set.begin(),
		//   p_end_it = new_clique_set.end(); p_clique_it != p_end_it; ++ p_clique_it) {
		/*for(std::vector<std::vector<size_t> >::const_iterator p_clique_it = linear_clique_list.begin(),
		   p_end_it = linear_clique_list.end(); p_clique_it != p_end_it; ++ p_clique_it) {
			const std::vector<size_t> &r_clique = *p_clique_it;

			if(r_clique.size() > 2) {
				printf("clique of %d: ", int(r_clique.size()));
				for(size_t j = 0; j < r_clique.size(); ++ j)
					printf("%d ", int(r_clique[j]));
				printf("\n");
			}
			// debug
		}*/
	}

	//PruneCliques_MaxVertexOrder(all_clique_list, p_graph->n);
	// removes most of the cliques, keeps only the largest clique
	// of each vertex so we end up with less than p_graph->n cliques

	if(b_final_nonmaximal_clique_removal)
		Suppress_NonmaximalCliques(all_clique_list); // does not remove anything that PruneCliques_MaxVertexOrder() didn't remove
	// should do this as well but it runs in O(n^2) and brings only small savings; will likely prune the cliques differently

#ifdef _DEBUG
	std::map<size_t, size_t> clique_size_hist;
	for(size_t j = 0, m = all_clique_list.size(); j < m; ++ j)
		++ clique_size_hist[all_clique_list[j].size()]; // debug
	for(std::map<size_t, size_t>::const_iterator p_size_it = clique_size_hist.begin(),
	   p_end_it = clique_size_hist.end(); p_size_it != p_end_it; ++ p_size_it)
		printf("cliques of size " PRIsize ": " PRIsize "\n", (*p_size_it).first, (*p_size_it).second);
	// debug - make a histogram
#endif // _DEBUG

	return all_clique_list;
}

size_t CSchurOrdering::n_Calculate_Ordering(std::vector<size_t> &r_ordering,
	const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
{
	const size_t n = r_lambda.n_BlockColumn_Num();
	if(r_ordering.capacity() < n) {
		r_ordering.clear();
		r_ordering.reserve(std::max(2 * r_ordering.capacity(), n));
	}
	r_ordering.resize(n);
	// allocate space

	/*CDeltaTimer dt;
	dt.Reset();

	cs *p_lambda, *p_lambda_t, *p_lambda_ata;
	if(!(p_lambda = r_lambda.p_BlockStructure_to_Sparse()))
		throw std::bad_alloc(); // rethrow
	// get block structure as a sparse CSC

	if(!(p_lambda_t = cs_transpose(p_lambda, false))) {
		cs_spfree(p_lambda);
		throw std::bad_alloc(); // rethrow
	}
	if(!(p_lambda_ata = cs_add(p_lambda, p_lambda_t, 1, 1))) {
		cs_spfree(p_lambda);
		cs_spfree(p_lambda_t);
		throw std::bad_alloc(); // rethrow
	}
	cs_spfree(p_lambda);
	cs_spfree(p_lambda_t);
	p_lambda = p_lambda_ata;
	// modify it to have symmetric structure // todo - implement this directly in CUberBlockMatrix

	double f_cs_time = dt.f_Time();*/

	cs *p_lambda;
	{
		const size_t m = r_lambda.n_BlockRow_Num(), n = r_lambda.n_BlockColumn_Num();
		const size_t n_greater_dim = std::max(m, n);
		std::vector<size_t> col_lengths(n_greater_dim), workspace(n);

		size_t n_aat_nnz_num = r_lambda.n_BlockStructure_SumWithSelfTranspose_ColumnLengths<false,
			true, false, true>(col_lengths.empty()? 0 : &col_lengths.front(), col_lengths.size(),
			(workspace.empty())? 0 : &workspace.front(), workspace.size());

		if(!(p_lambda = cs_spalloc(n_greater_dim, n_greater_dim, n_aat_nnz_num, 0, 0)))
			throw std::bad_alloc();

		enum {
			b_sorted = false // lambda is supposed to be symmetric, it will be sorted regardless
		};
		r_lambda.BlockStructure_SumWithSelfTranspose<false, true, false, true, b_sorted>(p_lambda->p,
			p_lambda->n + 1, p_lambda->i, n_aat_nnz_num, col_lengths.empty()? 0 : &col_lengths.front(),
			col_lengths.size(), (workspace.empty())? 0 : &workspace.front(), workspace.size());
	}
	// get block structure of A+A^T (with diagonal) as a sparse CSC

	/*double f_ubm_time = dt.f_Time();
	printf("time A+A^T: csparse %.3f msec, ubm %.3f msec\n", f_cs_time * 1000, f_ubm_time * 1000);*/

	try {
		std::vector<size_t> mis = CSchurOrdering::t_MIS_FirstFit(p_lambda);
		// calculate MIS

		CSchurOrdering::Complement_VertexSet(r_ordering, mis, n);
		// collect the rest of the vertices not in MIS

		r_ordering.insert(r_ordering.end(), mis.begin(), mis.end());
		// append with MIS to gain the final ordering

		cs_spfree(p_lambda);
		// cleanup

		return n - mis.size();
		// return number of connected vertices
	} catch(std::bad_alloc &r_exc) {
		cs_spfree(p_lambda);
		throw r_exc; // rethrow
	}
}

size_t CSchurOrdering::n_Calculate_GuidedOrdering(std::vector<size_t> &r_ordering,
	size_t n_pose_vertex_dimension, size_t n_landmark_vertex_dimension,
	const CUberBlockMatrix &r_lambda) // throw(std::bad_alloc)
{
	const size_t n = r_lambda.n_BlockColumn_Num();
	if(r_ordering.capacity() < n) {
		r_ordering.clear();
		r_ordering.reserve(std::max(2 * r_ordering.capacity(), n));
	}
	r_ordering.resize(n);
	// allocate space

	size_t n_pose_num = 0, n_landmark_num = 0;
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
	_ASSERTE(n_pose_num + n_landmark_num == r_lambda.n_BlockColumn_Num());
	// calculate the simple guided ordering (assumes that landmarks have smaller
	// dimension than poses, and that landmarks are not connected among each other)

	return n_pose_num;
}

std::vector<size_t> CSchurOrdering::t_MIS_FirstFit(const cs *p_graph, const std::vector<size_t> &r_weights,
	bool b_allow_improvement /*= true*/, size_t n_chain_improvement_size_limit /*= 64*/)
{
	const size_t n = p_graph->n;

	std::vector<size_t> perm(n);
	std::generate(perm.begin(), perm.end(), CIOTA());
	// generate an identity permutation

#if 1
	std::stable_sort(perm.begin(), perm.end(), CCompareVertexDegree_Weights(p_graph, r_weights));
	std::reverse(perm.begin(), perm.end());
#else // 1
	std::sort(perm.begin(), perm.end(),
		CCompareVertexDegree_WeightsContribution(p_graph, r_weights)); // t_odo - try sorting by incident weights minus self weight rather than all together
	//std::reverse(perm.begin(), perm.end()); // nope
#endif // 1
	// make ordering, starting with smallest degree (turns out to be better
	// than starting with the greatest degree)
	// note that ordering with vertex folding could yield even better results

	size_t n_mis_weight = 0, n_best_weight = 0;
	std::vector<size_t> mis, best_mis; // best_mis used only in case we are iterating
	mis.reserve(n / 2);
	// for most of the pose graphs, MIS of under half of the graph is resonable

	const int n_max_restart_num = 50;

	std::vector<bool> vertex_coverage; // need small auxiliary space
	for(int n_pass = 0; n_pass < 2; ++ n_pass) {
		if(n_pass) {
			std::sort(perm.begin(), perm.end(),
				CCompareVertexDegree_WeightsContribution(p_graph, r_weights));
		}
		// try to sort differently in the second pass (only takes 2x the time, still very little compared to bf exact)

		size_t m = n; // initially this is the same
		size_t n_shake_vertex_num = n; // number of vertices to shake between the iterations (alternately trying to find a few independent vertices with large degrees or many independent vertices with small degrees)
		for(int n_pass = 0;;) {
			vertex_coverage.resize(n, false);
			// (re)alloc coverage flags

			_ASSERTE(m == perm.size());
			for(size_t j = 0; j < m; ++ j) {
				size_t i = perm[j];
				// pick a vertex from permutation

				if(vertex_coverage[i])
					continue;
				// skip neighbours of vertices that are already in MIS

				n_mis_weight += r_weights[i];
				mis.push_back(i);
				// add uncovered vertex to MIS

				_ASSERTE(*std::find(p_graph->i + p_graph->p[i], p_graph->i + p_graph->p[i + 1], i) == i); // make sure the diagonal entry is present
				for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
					vertex_coverage[p_graph->i[p0]] = true;
				// set all neighbours (including self) to covered
			}
			// a simple greedy approach to MIS

			if(++ n_pass == n_max_restart_num)
				break;

			if(n_best_weight < n_mis_weight/*best_mis.size() < mis.size()*/) {
				n_best_weight = n_mis_weight;
				best_mis = mis;
			}
			// record the best mis so far

			mis.clear(); // reuse mis as a temp array (will be deleted anyway)
			for(size_t j = m; j > 0;) {
				-- j; // here
				size_t v = perm[j];
				if(vertex_coverage[v]) {
					mis.push_back(v);
					perm.erase(perm.begin() + j);
				}
			}
			/*_ASSERTE(n_shake_vertex_num >= mis.size());
			n_shake_vertex_num -= mis.size();*/
			n_shake_vertex_num = perm.size(); // or just do this
			std::reverse(perm.begin(), perm.begin() + n_shake_vertex_num); // reverse the range of vertices that haven't been used before
			perm.insert(perm.end(), mis.begin(), mis.end()); // do not throw them away completely!
			// erase covered vertices, will start with the best uncovered vertex the next time around

			//if(perm.empty()) // does not happen now, perm conserves size now
			//	break;
			// all vertices covered already?

			m = perm.size(); // !!
			vertex_coverage.clear();
			n_mis_weight = 0;
			mis.clear();
			// !! for the next round
		}
		// the choice of the first vertex matters a lot, so try to restart a bunch of times
	}

	if(n_best_weight > n_mis_weight/*best_mis.size() > mis.size()*/) // not always true, e.g. if n_max_restart_num == 1
		best_mis.swap(mis);
	{
		std::vector<size_t> empty;
		empty.swap(best_mis);
	}
	// make sure mis has the best mis

	if(!b_allow_improvement) {
/*#ifdef _DEBUG
		for(size_t j = 1, m = mis.size(); j < m; ++ j)
			_ASSERTE(mis[j - 1] < mis[j]);
#endif // _DEBUG*/
		std::sort(mis.begin(), mis.end());
		// note that mis is implicitly sorted, if there is no permutation // not true? or maybe meant "only for some BA datasets"

		return mis;
	}
	// in case improvement is not allowed

	std::vector<bool> &mis_membership = vertex_coverage;
	// note that vertex_coverage is not needed below, could reuse as mis_membership

	std::fill(mis_membership.begin(), mis_membership.end(), false);
	for(size_t j = 0, m = mis.size(); j < m; ++ j)
		mis_membership[mis[j]] = true;
	// need a fast lookup of what is in MIS

	bool b_swap;
	do {
		b_swap = false;

		for(size_t j = 0, m = mis.size(); j < m; ++ j) {
			size_t i = mis[j];
			// get a vertex that belongs to MIS

			mis_membership[i] = false;
			// assume we remove it

			std::vector<size_t> mis_add;
			size_t n_remove_incidence = r_weights[i];
			size_t n_add_incidence = 0;
			for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0) {
				size_t v = p_graph->i[p0];
				if(v == i)
					continue;
				// get a vertex, that is a neighbor of i ...

				_ASSERTE(!mis_membership[v]);
				// for sure is not a member of MIS (as it is a neighbor of i), and it should be covered

				bool b_covered = false;
				for(size_t r0 = p_graph->p[v], r1 = p_graph->p[v + 1]; r0 < r1; ++ r0) {
					if(mis_membership[p_graph->i[r0]]) {
						b_covered = true;
						break;
					}
				}
				// see if this vertex is covered

				if(!b_covered) {
					mis_membership[v] = true; // assume temporary membership
					n_add_incidence += r_weights[v];
					mis_add.push_back(v);
				}
				// this vertex could be traded for i
			}
			// go through the neighbors of i

			if(n_add_incidence > n_remove_incidence/*mis_add.size() > 1*/) {
				size_t n_v0 = mis_add.front();
				mis[j] = n_v0;
				mis_membership[n_v0] = true;
				// add the first vertex in place of the old vertex

				mis.reserve(m = mis.size() + mis_add.size() - 1); // also update m
				for(size_t k = 1, o = mis_add.size(); k < o; ++ k) {
					size_t n_vk = mis_add[k];
					mis.push_back(n_vk);
					mis_membership[n_vk] = true;
				}
				// add the next vertices at the end

				-- j; // can try again on the first one (after all, the adding is greedy)
				b_swap = true;
				// we just traded some vertices
			} else if(n_add_incidence == n_remove_incidence/*!mis_add.empty()*/ && CCompareVertexDegree(p_graph)(i, mis_add.front())) {
				_ASSERTE(!mis_add.empty()/*mis_add.size() == 1*/); // not empty /*and less than 2*/
				size_t n_v0 = mis_add.front();
				mis[j] = n_v0;
				mis_membership[n_v0] = true;
				// put the vertex in place of the old vertex

				-- j; // can try again on this one (will not swap back, degree won't allow it)
				b_swap = true;
				// we just traded i for a vertex with smaller degree
				// (MIS can theoretically contain more vertices of smaller degree)
			} else {
				for(size_t k = 0, o = mis_add.size(); k < o; ++ k)
					mis_membership[mis_add[k]] = false; // remove the temporary memberships
				mis_membership[i] = true; // in the end, we do not remove it
			}
			// trade the vertices

#if 1
			_ASSERTE(m == mis.size());
			for(size_t k = 0; k < m; ++ k) {
				size_t i = mis[k];
				_ASSERTE(mis_membership[i]);
				for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
					_ASSERTE(p_graph->i[p0] == i || !mis_membership[p_graph->i[p0]]);
			}
			// debug checking - makes it really slow
#endif // 0
		}
	} while(b_swap);
	// improve MIS by trading membership of single vertices for more than one vertices

	if(n < n_chain_improvement_size_limit) {
		// below algorithm is O(n^2), does not improve the solution much (maybe disable completely)
		// but sometimes it helps to find the perfect solution in small graphs

		bool b_swap_outer;
		do {
			b_swap_outer = false;

			for(size_t l = 0, m = mis.size(); l < m; ++ l) {
				const size_t v0 = mis[l];
				// get a vertex that belongs to MIS

				mis_membership[v0] = false;
				// assume we remove it

				size_t n_remove_incidence = r_weights[v0];
				size_t n_add0_incidence = 0;
				std::vector<size_t> mis_add;
				for(size_t p0 = p_graph->p[v0], p1 = p_graph->p[v0 + 1]; p0 < p1; ++ p0) {
					size_t v = p_graph->i[p0];
					if(v == v0)
						continue;
					// get a vertex, that is a neighbor of v0 ...

					_ASSERTE(!mis_membership[v]);
					// for sure is not a member of MIS (as it is a neighbor of i), and it should be covered

					bool b_covered = false;
					for(size_t r0 = p_graph->p[v], r1 = p_graph->p[v + 1]; r0 < r1; ++ r0) {
						if(mis_membership[p_graph->i[r0]]) {
							b_covered = true;
							break;
						}
					}
					// see if this vertex is covered

					if(!b_covered) {
						mis_membership[v] = true; // assume temporary membership
						n_add0_incidence += r_weights[v];
						mis_add.push_back(v);
					}
					// this vertex could be traded for v0
				}
				// go through the neighbors of v0

				if(mis_add.empty()) {
					mis_membership[v0] = true; // !!
					continue;
				}
				//_ASSERTE(mis_add.size() == 1); // t_odo - not true anymore, due to the weights

				_ASSERTE(m == mis.size());
				//const size_t v1 = mis_add.front();
				if(n_add0_incidence < n_remove_incidence) { // are we going to make it worse?
					//mis_membership[v1] = false; // remove the temporary membership
					for(size_t k = 0, o = mis_add.size(); k < o; ++ k)
						mis_membership[mis_add[k]] = false; // remove the temporary memberships
					mis_membership[v0] = true; // !!
					continue;
				}
				size_t n_before_add = n_add0_incidence - n_remove_incidence/*m*/;
				// see the score

				//_ASSERTE(mis_membership[v1] == true); // already set
				//mis[l] = v1;
				mis.erase(mis.begin() + l); -- m; // remove the vertex from MIS so that it cannot be removed again in the inner loop
				// assume the swap helps something

				do {
					b_swap = false;

					for(size_t j = 0; j < m; ++ j) {
						size_t i = mis[j];
						// get a vertex that belongs to MIS

						mis_membership[i] = false;
						// assume we remove it

						size_t n_add_incidence = 0;
						std::vector<size_t> mis_add;
						for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0) {
							size_t v = p_graph->i[p0];
							if(v == i)
								continue;
							// get a vertex, that is a neighbor of i ...

							_ASSERTE(!mis_membership[v]);
							// for sure is not a member of MIS (as it is a neighbor of i), and it should be covered

							bool b_covered = false;
							for(size_t r0 = p_graph->p[v], r1 = p_graph->p[v + 1]; r0 < r1; ++ r0) {
								if(mis_membership[p_graph->i[r0]]) {
									b_covered = true;
									break;
								}
							}
							// see if this vertex is covered

							if(!b_covered) {
								mis_membership[v] = true; // assume temporary membership
								n_add_incidence += r_weights[v];
								mis_add.push_back(v);
							}
							// this vertex could be traded for i
						}
						// go through the neighbors of i

						size_t n_remove_incidence = r_weights[i];
						if(n_add_incidence > n_remove_incidence/*mis_add.size() > 1*/) {
							size_t n_v0 = mis_add.front();
							mis[j] = n_v0;
							mis_membership[n_v0] = true;
							// add the first vertex in place of the old vertex

							mis.reserve(m = mis.size() + mis_add.size() - 1); // also update m
							for(size_t k = 1, o = mis_add.size(); k < o; ++ k) {
								size_t n_vk = mis_add[k];
								mis.push_back(n_vk);
								mis_membership[n_vk] = true;
							}
							// add the next vertices at the end

							n_before_add += n_add_incidence;
							_ASSERTE(n_before_add > n_remove_incidence); // certainly not smaller
							n_before_add -= n_remove_incidence;

							-- j; // can try again on the first one (after all, the adding is greedy)
							b_swap = true;
							// we just traded some vertices
						} else {
							for(size_t k = 0, o = mis_add.size(); k < o; ++ k)
								mis_membership[mis_add[k]] = false; // remove the temporary memberships
							mis_membership[i] = true; // in the end, we do not remove it
						}
						// trade the vertices
					}
				} while(b_swap);

				_ASSERTE(/*m >= n_before_add*/n_before_add >= 0); // certainly not smaller
				if(/*m == n_before_add*/n_before_add == 0) { // no change
					mis.insert(mis.begin() + l, v0); ++ m; // add it back
					//mis[l] = v0;
					mis_membership[v0] = true;
					//mis_membership[v1] = false;
					for(size_t k = 0, o = mis_add.size(); k < o; ++ k)
						mis_membership[mis_add[k]] = false; // remove the temporary memberships
					// the swap does not help anything, put it back
				} else {
					mis.insert(mis.begin() + l, mis_add.begin(), mis_add.end()); m += mis_add.size(); // weren't added before
					b_swap_outer = true; // we did it, we managed a chain swap
				}
				// see if the swap was proficient

#if 1
				_ASSERTE(m == mis.size());
				for(size_t k = 0; k < m; ++ k) {
					size_t i = mis[k];
					_ASSERTE(mis_membership[i]);
					for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
						_ASSERTE(p_graph->i[p0] == i || !mis_membership[p_graph->i[p0]]);
				}
				// debug checking - makes it really slow
#endif // 0
			}
		} while(b_swap_outer);
	}
	// try to swap chains of two vertices

	std::sort(mis.begin(), mis.end()); // "postorder" .. ?
	// note that mis is implicitly sorted, if there is no permutation

	// t_odo - approaches with sorting of the vertices based on the degree,
	// and swapping of vertices

	return mis;
}

std::vector<size_t> CSchurOrdering::t_MIS_FirstFit(const cs *p_graph,
	bool b_allow_improvement, size_t n_chain_improvement_size_limit) // throw(std::bad_alloc)
{
	const size_t n = p_graph->n;

	std::vector<size_t> perm(n);
	std::generate(perm.begin(), perm.end(), CIOTA());
	// generate an identity permutation

	std::stable_sort(perm.begin(), perm.end(), CCompareVertexDegree(p_graph));
	// make ordering, starting with smallest degree (turns out to be better
	// than starting with the greatest degree)
	// note that ordering with vertex folding could yield even better results

#if 1
	std::vector<size_t> mis, best_mis; // best_mis used only in case we are iterating
	mis.reserve(n / 2);
	// for most of the pose graphs, MIS of under half of the graph is resonable

	const int n_max_restart_num = 50;

	std::vector<bool> vertex_coverage; // need small auxiliary space
	size_t m = n; // initially this is the same
	for(int n_pass = 0;;) {
		vertex_coverage.resize(n, false);
		// (re)alloc coverage flags

		_ASSERTE(m == perm.size());
		for(size_t j = 0; j < m; ++ j) {
			size_t i = perm[j];
			// pick a vertex from permutation

			if(vertex_coverage[i])
				continue;
			// skip neighbours of vertices that are already in MIS

			mis.push_back(i);
			// add uncovered vertex to MIS

			_ASSERTE(*std::find(p_graph->i + p_graph->p[i], p_graph->i + p_graph->p[i + 1], i) == i); // make sure the diagonal entry is present
			for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
				vertex_coverage[p_graph->i[p0]] = true;
			// set all neighbours (including self) to covered
		}
		// a simple greedy approach to MIS

		if(++ n_pass == n_max_restart_num)
			break;

		if(best_mis.size() < mis.size())
			best_mis = mis;
		// record the best mis so far

		mis.clear(); // reuse mis as a temp array (will be deleted anyway)
		for(size_t j = m; j > 0;) {
			-- j; // here
			size_t v = perm[j];
			if(vertex_coverage[v]) {
				mis.push_back(v);
				perm.erase(perm.begin() + j);
			}
		}
		perm.insert(perm.end(), mis.begin(), mis.end()); // do not throw them away completely!
		// erase covered vertices, will start with the best uncovered vertex the next time around

		//if(perm.empty()) // does not happen now, perm conserves size now
		//	break;
		// all vertices covered already?

		m = perm.size(); // !!
		vertex_coverage.clear();
		mis.clear();
		// !! for the next round
	}
	// the choice of the first vertex matters a lot, so try to restart a bunch of times

	if(best_mis.size() > mis.size()) // not always true, e.g. if n_max_restart_num == 1
		best_mis.swap(mis);
	{
		std::vector<size_t> empty;
		empty.swap(best_mis);
	}
	// make sure mis has the best mis
#else // 1
	std::vector<size_t> mis;
	mis.reserve(n / 2);
	// for most of the pose graphs, MIS of under half of the graph is resonable

	std::vector<bool> vertex_coverage(n, false); // need small auxiliary space
	for(size_t j = 0; j < n; ++ j) {
		size_t i = perm[j];
		// pick a vertex from permutation

		if(vertex_coverage[i])
			continue;
		// skip neighbours of vertices that are already in MIS

		mis.push_back(i);
		// add uncovered vertex to MIS

		for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
			vertex_coverage[p_graph->i[p0]] = true;
		// set all neighbours (including self) to covered
	}
	// a simple greedy approach to MIS
#endif // 1

	if(!b_allow_improvement) {
/*#ifdef _DEBUG
		for(size_t j = 1, m = mis.size(); j < m; ++ j)
			_ASSERTE(mis[j - 1] < mis[j]);
#endif // _DEBUG*/
		std::sort(mis.begin(), mis.end());
		// note that mis is implicitly sorted, if there is no permutation // not true? or maybe meant "only for some BA datasets"

		return mis;
	}
	// in case improvement is not allowed

	std::vector<bool> &mis_membership = vertex_coverage;
	// note that vertex_coverage is not needed below, could reuse as mis_membership

	std::fill(mis_membership.begin(), mis_membership.end(), false);
	for(size_t j = 0, m = mis.size(); j < m; ++ j)
		mis_membership[mis[j]] = true;
	// need a fast lookup of what is in MIS

	bool b_swap;
	do {
		b_swap = false;

		for(size_t j = 0, m = mis.size(); j < m; ++ j) {
			size_t i = mis[j];
			// get a vertex that belongs to MIS

			mis_membership[i] = false;
			// assume we remove it

			std::vector<size_t> mis_add;
			for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0) {
				size_t v = p_graph->i[p0];
				if(v == i)
					continue;
				// get a vertex, that is a neighbor of i ...

				_ASSERTE(!mis_membership[v]);
				// for sure is not a member of MIS (as it is a neighbor of i), and it should be covered

				bool b_covered = false;
				for(size_t r0 = p_graph->p[v], r1 = p_graph->p[v + 1]; r0 < r1; ++ r0) {
					if(mis_membership[p_graph->i[r0]]) {
						b_covered = true;
						break;
					}
				}
				// see if this vertex is covered

				if(!b_covered) {
					mis_membership[v] = true; // assume temporary membership
					mis_add.push_back(v);
				}
				// this vertex could be traded for i
			}
			// go through neighbors of i

			if(mis_add.size() > 1) {
				size_t n_v0 = mis_add.front();
				mis[j] = n_v0;
				mis_membership[n_v0] = true;
				// add the first vertex in place of the old vertex

				mis.reserve(m = mis.size() + mis_add.size() - 1); // also update m
				for(size_t k = 1, o = mis_add.size(); k < o; ++ k) {
					size_t n_vk = mis_add[k];
					mis.push_back(n_vk);
					mis_membership[n_vk] = true;
				}
				// add the next vertices at the end

				-- j; // can try again on the first one (after all, the adding is greedy)
				b_swap = true;
				// we just traded some vertices
			} else if(!mis_add.empty() && CCompareVertexDegree(p_graph)(i, mis_add.front())) {
				_ASSERTE(mis_add.size() == 1); // not empty and less than 2
				size_t n_v0 = mis_add.front();
				mis[j] = n_v0;
				mis_membership[n_v0] = true;
				// put the vertex in place of the old vertex

				-- j; // can try again on this one (will not swap back, degree won't allow it)
				b_swap = true;
				// we just traded i for a vertex with smaller degree
				// (MIS can theoretically contain more vertices of smaller degree)
			} else {
				for(size_t k = 0, o = mis_add.size(); k < o; ++ k)
					mis_membership[mis_add[k]] = false; // remove the temporary memberships
				mis_membership[i] = true; // in the end, we do not remove it
			}
			// trade the vertices

#if 0
			_ASSERTE(m == mis.size());
			for(size_t k = 0; k < m; ++ k) {
				size_t i = mis[k];
				_ASSERTE(mis_membership[i]);
				for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
					_ASSERTE(p_graph->i[p0] == i || !mis_membership[p_graph->i[p0]]);
			}
			// debug checking - makes it really slow
#endif // 0
		}
	} while(b_swap);
	// improve MIS by trading membership of single vertices for more than one vertices

	if(n < n_chain_improvement_size_limit) {
		// below algorithm is O(n^2), does not improve the solution much (maybe disable completely)
		// but sometimes it helps to find the perfect solution in small graphs

		bool b_swap_outer;
		do {
			b_swap_outer = false;

			for(size_t l = 0, m = mis.size(); l < m; ++ l) {
				const size_t v0 = mis[l];
				// get a vertex that belongs to MIS

				mis_membership[v0] = false;
				// assume we remove it

				std::vector<size_t> mis_add;
				for(size_t p0 = p_graph->p[v0], p1 = p_graph->p[v0 + 1]; p0 < p1; ++ p0) {
					size_t v = p_graph->i[p0];
					if(v == v0)
						continue;
					// get a vertex, that is a neighbor of v0 ...

					_ASSERTE(!mis_membership[v]);
					// for sure is not a member of MIS (as it is a neighbor of i), and it should be covered

					bool b_covered = false;
					for(size_t r0 = p_graph->p[v], r1 = p_graph->p[v + 1]; r0 < r1; ++ r0) {
						if(mis_membership[p_graph->i[r0]]) {
							b_covered = true;
							break;
						}
					}
					// see if this vertex is covered

					if(!b_covered) {
						mis_membership[v] = true; // assume temporary membership
						mis_add.push_back(v);
					}
					// this vertex could be traded for v0
				}
				// go through the neighbors of v0

				if(mis_add.empty()) {
					mis_membership[v0] = true; // !!
					continue;
				}
				_ASSERTE(mis_add.size() == 1); // the others already eliminated above or in the inner loop

				const size_t v1 = mis_add.front();
				_ASSERTE(mis_membership[v1] == true); // already set
				mis[l] = v1;
				// assume the swap helps something

				_ASSERTE(m == mis.size());
				size_t n_before_add = m;

				do {
					b_swap = false;

					for(size_t j = 0; j < m; ++ j) {
						size_t i = mis[j];
						// get a vertex that belongs to MIS

						mis_membership[i] = false;
						// assume we remove it

						std::vector<size_t> mis_add;
						for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0) {
							size_t v = p_graph->i[p0];
							if(v == i)
								continue;
							// get a vertex, that is a neighbor of i ...

							_ASSERTE(!mis_membership[v]);
							// for sure is not a member of MIS (as it is a neighbor of i), and it should be covered

							bool b_covered = false;
							for(size_t r0 = p_graph->p[v], r1 = p_graph->p[v + 1]; r0 < r1; ++ r0) {
								if(mis_membership[p_graph->i[r0]]) {
									b_covered = true;
									break;
								}
							}
							// see if this vertex is covered

							if(!b_covered) {
								mis_membership[v] = true; // assume temporary membership
								mis_add.push_back(v);
							}
							// this vertex could be traded for i
						}
						// go through the neighbors of i

						if(mis_add.size() > 1) {
							size_t n_v0 = mis_add.front();
							mis[j] = n_v0;
							mis_membership[n_v0] = true;
							// add the first vertex in place of the old vertex

							mis.reserve(m = mis.size() + mis_add.size() - 1); // also update m
							for(size_t k = 1, o = mis_add.size(); k < o; ++ k) {
								size_t n_vk = mis_add[k];
								mis.push_back(n_vk);
								mis_membership[n_vk] = true;
							}
							// add the next vertices at the end

							-- j; // can try again on the first one (after all, the adding is greedy)
							b_swap = true;
							// we just traded some vertices
						} else {
							for(size_t k = 0, o = mis_add.size(); k < o; ++ k)
								mis_membership[mis_add[k]] = false; // remove the temporary memberships
							mis_membership[i] = true; // in the end, we do not remove it
						}
						// trade the vertices
					}
				} while(b_swap);

				_ASSERTE(m >= n_before_add); // certainly not smaller
				if(m == n_before_add) { // no change
					mis[l] = v0;
					mis_membership[v0] = true;
					mis_membership[v1] = false;
					// the swap does not help anything, put it back
				} else
					b_swap_outer = true; // we did it, we managed a chain swap
				// see if the swap was proficient

#if 0
				_ASSERTE(m == mis.size());
				for(size_t k = 0; k < m; ++ k) {
					size_t i = mis[k];
					_ASSERTE(mis_membership[i]);
					for(size_t p0 = p_graph->p[i], p1 = p_graph->p[i + 1]; p0 < p1; ++ p0)
						_ASSERTE(p_graph->i[p0] == i || !mis_membership[p_graph->i[p0]]);
				}
				// debug checking - makes it really slow
#endif // 0
			}
		} while(b_swap_outer);
	}
	// try to swap chains of two vertices

	std::sort(mis.begin(), mis.end()); // "postorder" .. ?
	// note that mis is implicitly sorted, if there is no permutation

	// t_odo - approaches with sorting of the vertices based on the degree,
	// and swapping of vertices

	return mis;
}

size_t CSchurOrdering::n_MIS(const cs *p_graph) // throw(std::bad_alloc)
{
	if(!b_IsConnected(p_graph)) {
		std::vector<size_t> C;
		Get_MinConnectedSet(p_graph, C);
		// get minimal connected set

		cs *p_GC = p_Subgraph(p_graph, C.begin(), C.end());
		size_t n_result = n_MIS(p_GC);
		// recurse with the rest of the graph

		cs_spfree(p_GC);
		if(C.size() <= 2)
			n_result += 1; // one of vertices in C used (no matter which one)
		else {
			std::vector<size_t> inv_C;
			Complement_VertexSet(inv_C, C, p_graph->n);
			cs *p_C = p_Subgraph(p_graph, inv_C.begin(), inv_C.end());
			n_result += n_MIS(p_C);
			cs_spfree(p_C);
			// biggest subset; need to recurse
		}

		return n_result;
	}
	// handle disconnected graphs

	if(p_graph->n <= 1)
		return p_graph->n;
	// handle trivial graphs

	size_t n_greatest = 0;
	size_t n_degree = p_graph->p[1] - p_graph->p[0];
	const size_t n = p_graph->n;
	for(size_t i = 1; i < n; ++ i) {
		size_t n_deg = p_graph->p[i + 1] - p_graph->p[i];
		if(n_deg > n_degree) {
			n_degree = n_deg;
			n_greatest = i;
		}
	}
	// find a vertex with maximum degree (note that this is approximate as there is only the upper triangular, some vertices may have bigger degrees; would need n workspace to calculate degrees in O(nnz))

	cs *p_GB = p_Subgraph(p_graph, &n_greatest, &n_greatest + 1);
	size_t n_not_used = n_MIS(p_GB);
	cs_spfree(p_GB);
	// case when the vertex (B) is not used

	_ASSERTE(sizeof(csi) == sizeof(size_t));
	cs *p_GNB = p_Subgraph(p_graph, (size_t*)p_graph->i + p_graph->p[n_greatest],
		(size_t*)p_graph->i + p_graph->p[n_greatest + 1]);
	size_t n_used = n_MIS(p_GNB);
	cs_spfree(p_GNB);
	// case when the vertex (B) is used

	return std::max(n_used, n_not_used);
}

std::vector<size_t> CSchurOrdering::t_MIS(const cs *p_graph) // throw(std::bad_alloc)
{
	if(p_graph->n <= 1) {
		std::vector<size_t> result;
		if(p_graph->n)
			result.push_back(0);
		return result;
	}
	// handle trivial graphs

	if(!b_IsConnected(p_graph)) {
		std::vector<size_t> C;
		Get_MinConnectedSet(p_graph, C);
		// get minimal connected set

		cs *p_GC = p_Subgraph(p_graph, C.begin(), C.end());
		std::vector<size_t> result = t_MIS(p_GC); // t_odo - transform

		// recursion unroll save point 0

		Transform_Indices_Complement(result, C);
		// recurse with the rest of the graph

		cs_spfree(p_GC);
		if(C.size() <= 2) {
			size_t n_vertex = C.front(); // one of vertices in C used (no matter which one)
			result.insert(std::lower_bound(result.begin(), result.end(), n_vertex), n_vertex);
		} else {
			std::vector<size_t> inv_C;
			Complement_VertexSet(inv_C, C, p_graph->n);
			cs *p_C = p_Subgraph(p_graph, inv_C.begin(), inv_C.end());
			std::vector<size_t> result_C = t_MIS(p_C); // t_odo - transform

			// recursion unroll save point 1

			Transform_Indices_Complement(result_C, inv_C);
			result.insert(result.end(), result_C.begin(), result_C.end());
			cs_spfree(p_C);
			// biggest subset; need to recurse

			std::sort(result.begin(), result.end());
			// !!
		}

		return result;
	}
	// handle disconnected graphs

	size_t n_greatest = 0;
	size_t n_degree = p_graph->p[1] - p_graph->p[0];
	const size_t n = p_graph->n;
	for(size_t i = 1; i < n; ++ i) {
		size_t n_deg = p_graph->p[i + 1] - p_graph->p[i];
		if(n_deg > n_degree) {
			n_degree = n_deg;
			n_greatest = i;
		}
	}
	// find a vertex with maximum degree

	cs *p_GB = p_Subgraph(p_graph, &n_greatest, &n_greatest + 1);
	std::vector<size_t> not_used = t_MIS(p_GB); // t_odo - transform
	cs_spfree(p_GB);
	// case when the vertex (B) is not used

	// recursion unroll save point 2

	_ASSERTE(sizeof(csi) == sizeof(size_t));
	cs *p_GNB = p_Subgraph(p_graph, (size_t*)p_graph->i + p_graph->p[n_greatest],
		(size_t*)p_graph->i + p_graph->p[n_greatest + 1]);
	std::vector<size_t> used = t_MIS(p_GNB); // t_odo - transform
	cs_spfree(p_GNB);
	// case when the vertex (B) is used

	// recursion unroll save point 3

	if(not_used.size() > used.size() + 1) {
		Transform_Indices_Complement(not_used, &n_greatest, &n_greatest + 1);
		return not_used;
	} else {
		Transform_Indices_Complement(used, p_graph->i + p_graph->p[n_greatest],
			p_graph->i + p_graph->p[n_greatest + 1]);
		used.insert(std::lower_bound(used.begin(), used.end(), n_greatest), n_greatest);
		return used;
	}
	// transform indices and return
}

std::vector<size_t> CSchurOrdering::t_MIS_ExStack(const cs *p_graph) // throw(std::bad_alloc)
{
	std::vector<size_t> recursion_result;

	TMIS_StackFrame t_base;
	t_base.p_graph = p_graph;
	t_base.p_subgraph = 0;
	t_base.n_phase = 0;
	std::vector<TMIS_StackFrame> stack;
	stack.push_back(t_base);
	while(!stack.empty()) {
		TMIS_StackFrame &t_fr = stack.back();

		p_graph = t_fr.p_graph;
		// assume current stack frame graph

		if(t_fr.n_phase == 0) {
			recursion_result.clear();
			// always begin with empty result

			if(p_graph->n <= 1) {
				//recursion_result.clear(); // no need, done just above
				if(p_graph->n)
					recursion_result.push_back(0);
				// leave result in recursion_result

				stack.erase(stack.end() - 1); // note that the reference to t_fr is invalidated
				continue;
				// pop stack frame, return to the one above
			}
			// handle trivial graphs

			if(!b_IsConnected(p_graph)) {
				std::vector<size_t> &C = t_fr.complement_set; // store in this frame
				Get_MinConnectedSet(p_graph, C);
				// get minimal connected set

				cs *p_GC = p_Subgraph(p_graph, C.begin(), C.end());
				t_fr.p_subgraph = p_GC; // save to delete later
				++ t_fr.n_phase; // jump to the next phase

				if(p_GC->n < max_Recurse_Size)
					recursion_result = t_MIS(p_GC); // the graph is small, can use implicit stack (faster)
				else {
					TMIS_StackFrame t_recurse;
					t_recurse.n_phase = 0;
					t_recurse.p_graph = p_GC;
					t_recurse.p_subgraph = 0;
					stack.push_back(t_recurse); // note that the reference to t_fr is invalidated
					continue;
					// push stack frame, recurse
				}
			} else
				t_fr.n_phase = 4; // jump to phase 4 (skip phase 1 - 3)
		}
		// handle the first phase of the algorithm

		if(t_fr.n_phase == 1) {
			_ASSERTE(p_graph->n > 1 && !b_IsConnected(p_graph));
			// we were inside the disconnected graph branch

			{
				std::vector<size_t> &C = t_fr.complement_set;
				cs *p_GC = t_fr.p_subgraph; // stored in this frame
				t_fr.p_subgraph = 0; // ...
				std::vector<size_t> &result = recursion_result; // got result from the recursion

				Transform_Indices_Complement(result, C);
				cs_spfree(p_GC);
				if(C.size() <= 2) {
					result.push_back(C.front()); // one of vertices in C used (no matter which one)
					t_fr.int_result.swap(result); // save result
					t_fr.n_phase = 3; // go to phase 3
				} else {
					{
						std::vector<size_t> inv_C;
						Complement_VertexSet(inv_C, C, p_graph->n);
						t_fr.complement_set.swap(inv_C);
					}
					std::vector<size_t> &inv_C = t_fr.complement_set;
					cs *p_C = p_Subgraph(p_graph, inv_C.begin(), inv_C.end());

					t_fr.p_subgraph = p_C; // save to delete later
					t_fr.int_result.swap(result); // save result
					++ t_fr.n_phase; // jump to the next phase

					if(p_C->n < max_Recurse_Size)
						recursion_result = t_MIS(p_C); // the graph is small, can use implicit stack (faster)
					else {
						TMIS_StackFrame t_recurse;
						t_recurse.n_phase = 0;
						t_recurse.p_graph = p_C;
						t_recurse.p_subgraph = 0;
						stack.push_back(t_recurse); // note that the reference to t_fr is invalidated
						continue;
						// push stack frame, recurse
					}
				}
			}
		}

		if(t_fr.n_phase == 2) {
			_ASSERTE(p_graph->n > 1 && !b_IsConnected(p_graph));
			// we were inside the disconnected graph branch, C was bigger than 2

			std::vector<size_t> &inv_C = t_fr.complement_set; // size smaller than or equal to n - 2
			cs *p_C = t_fr.p_subgraph;
			t_fr.p_subgraph = 0; // ...
			std::vector<size_t> &result = t_fr.int_result, &result_C = recursion_result;
			// resture locals

			{
				Transform_Indices_Complement(result_C, inv_C);
				result.insert(result.end(), result_C.begin(), result_C.end());
				cs_spfree(p_C);
				// biggest subset; need to recurse

				++ t_fr.n_phase; // move to the next phase
			}
		}

		if(t_fr.n_phase == 3) {
			_ASSERTE(p_graph->n > 1 && !b_IsConnected(p_graph));
			// we were inside the disconnected graph branch, C was bigger than 2

			std::vector<size_t> &result = t_fr.int_result;
			// resture locals

			std::sort(result.begin(), result.end()); // note that in the upper branch, sorting could be replaced by insertion at lower_bound
			// !!

			recursion_result.swap(result); // leave result in recursion_result
			stack.erase(stack.end() - 1); // note that the reference to t_fr is invalidated
			continue;
			// pop stack frame, return to the one above
		}

		if(t_fr.n_phase == 4) {
			_ASSERTE(p_graph->n > 1 && b_IsConnected(p_graph));
			// we are below the disconnected graph branch

			size_t n_greatest = 0;
			size_t n_degree = p_graph->p[1] - p_graph->p[0];
			const size_t n = p_graph->n;
			for(size_t i = 1; i < n; ++ i) {
				size_t n_deg = p_graph->p[i + 1] - p_graph->p[i];
				if(n_deg > n_degree) {
					n_degree = n_deg;
					n_greatest = i;
				}
			}
			// find a vertex with maximum degree

			cs *p_GB = p_Subgraph(p_graph, &n_greatest, &n_greatest + 1);

			t_fr.n_intermediate = n_greatest;
			t_fr.p_subgraph = p_GB;
			++ t_fr.n_phase;

			if(p_GB->n < max_Recurse_Size)
				recursion_result = t_MIS(p_GB); // the graph is small, can use implicit stack (faster)
			else {
				TMIS_StackFrame t_recurse;
				t_recurse.n_phase = 0;
				t_recurse.p_graph = p_GB;
				t_recurse.p_subgraph = 0;
				stack.push_back(t_recurse); // note that the reference to t_fr is invalidated
				continue;
				// push stack frame, recurse
			}
		}

		if(t_fr.n_phase == 5) {
			_ASSERTE(p_graph->n > 1 && b_IsConnected(p_graph));
			// we are below the disconnected graph branch

			cs *p_GB = t_fr.p_subgraph;
			t_fr.p_subgraph = 0; // ...
			size_t n_greatest = t_fr.n_intermediate;
			// restore locals

			cs_spfree(p_GB);
			// case when the vertex (B) is not used

			_ASSERTE(sizeof(csi) == sizeof(size_t));
			cs *p_GNB = p_Subgraph(p_graph, (size_t*)p_graph->i + p_graph->p[n_greatest],
				(size_t*)p_graph->i + p_graph->p[n_greatest + 1]);

			t_fr.int_result.swap(recursion_result); // save this
			t_fr.p_subgraph = p_GNB;
			++ t_fr.n_phase;

			if(p_GNB->n < max_Recurse_Size)
				recursion_result = t_MIS(p_GNB); // the graph is small, can use implicit stack (faster)
			else {
				TMIS_StackFrame t_recurse;
				t_recurse.n_phase = 0;
				t_recurse.p_graph = p_GNB;
				t_recurse.p_subgraph = 0;
				stack.push_back(t_recurse); // note that the reference to t_fr is invalidated
				continue;
				// push stack frame, recurse
			}
		}

		if(t_fr.n_phase == 6) {
			_ASSERTE(p_graph->n > 1 && b_IsConnected(p_graph));
			// we are below the disconnected graph branch

			cs *p_GNB = t_fr.p_subgraph;
			t_fr.p_subgraph = 0; // ...
			size_t n_greatest = t_fr.n_intermediate;
			std::vector<size_t> &not_used = t_fr.int_result;
			std::vector<size_t> &used = recursion_result;
			// restore locals

			cs_spfree(p_GNB);
			// case when the vertex (B) is used

			if(not_used.size() > used.size() + 1) {
				Transform_Indices_Complement(not_used, &n_greatest, &n_greatest + 1);

				recursion_result.swap(not_used); // leave result in recursion_result
				stack.erase(stack.end() - 1); // note that the reference to t_fr is invalidated
				continue;
				// pop stack frame, return to the one above
			} else {
				Transform_Indices_Complement(used, p_graph->i + p_graph->p[n_greatest],
					p_graph->i + p_graph->p[n_greatest + 1]);
				used.insert(std::lower_bound(used.begin(), used.end(), n_greatest), n_greatest);

				//recursion_result.swap(used); // already there
				stack.erase(stack.end() - 1); // note that the reference to t_fr is invalidated
				continue;
				// pop stack frame, return to the one above
			}
			// transform indices and return
		}
	}

	return recursion_result;
}

std::vector<size_t> CSchurOrdering::t_MIS_Parallel(const cs *p_graph) // throw(std::bad_alloc)
{
	if(p_graph->n <= thread_Num_Log2)
		return t_MIS_ExStack(p_graph);
	// don't parallelize vary small graphs

	const int n_max_level = std::min(int(thread_Num_Log2), int(n_Log2(p_graph->n) - 1));
	const int n_thread_num = 1 << n_max_level;
	//printf("max_level: %d\n", n_max_level); // verbose
	// decide maxmium generator recursion

#ifdef _OPENMP
	int n_CPU_num;
	#pragma omp parallel
	{
		#pragma omp master
		{
			n_CPU_num = omp_get_num_threads();
		}
	}
	omp_set_num_threads(std::max(1, n_CPU_num - 2));
#endif // _OPENMP
	// don't use all the threads

	TMIS_StackFrame search_tree[thread_Num_Log2 + 1][thread_Num];
	search_tree[0][0].p_graph = p_graph;
	for(int i = 0; i < n_max_level; ++ i) {
		int n_stage_size = 1 << i;

		#pragma omp parallel for schedule(static, 1)
		for(int j = 0; j < n_stage_size; ++ j) {
			const cs *p_subgraph = search_tree[i][j].p_graph;
			// get the graph

			size_t n_greatest = 0;
			size_t n_degree = -1;
			const size_t n = p_subgraph->n;
			for(size_t k = 0; k < n; ++ k) {
				size_t n_deg = p_subgraph->p[k + 1] - p_subgraph->p[k];
				if(n_deg > n_degree) {
					n_degree = n_deg;
					n_greatest = k;
				}
			}
			// find a vertex with maximum degree

			search_tree[i][j].n_intermediate = n_greatest;

			_ASSERTE(sizeof(csi) == sizeof(size_t));
			cs *p_GB = p_Subgraph(p_subgraph, &n_greatest, &n_greatest + 1);
			cs *p_GNB = p_Subgraph(p_subgraph, (size_t*)p_subgraph->i + p_subgraph->p[n_greatest],
				(size_t*)p_subgraph->i + p_subgraph->p[n_greatest + 1]);
			// generate subgraphs

			search_tree[i + 1][j * 2 + 0].p_graph = p_GB;
			search_tree[i + 1][j * 2 + 1].p_graph = p_GNB;
			// generate two branches of the recursion tree
		}

		//printf(".");
	}
	//printf("\n");
	// generate a set of subgraphs to be solved in parallel

	#pragma omp parallel for schedule(dynamic, 1)
	for(int i = 0; i < n_thread_num; ++ i) {
		search_tree[n_max_level][i].int_result =
			t_MIS_ExStack(search_tree[n_max_level][i].p_graph);
		//printf("%d\n", int(search_tree[n_max_level][i].int_result.size()));
		//printf("*");
	}
	//printf("\n");
	// calculate the intermediate result for each subgraph (can be done in parallel)

	for(int i = n_max_level; i > 0;) {
		-- i;
		int n_stage_size = 1 << i;

		#pragma omp parallel for schedule(static, 1)
		for(int j = 0; j < n_stage_size; ++ j) {
			p_graph = search_tree[i][j].p_graph;
			// get the graph

			std::vector<size_t> &not_used = search_tree[i + 1][j * 2 + 0].int_result;
			std::vector<size_t> &used = search_tree[i + 1][j * 2 + 1].int_result;
			// get the results of the two subgraphs

			size_t n_greatest = search_tree[i][j].n_intermediate;

			if(not_used.size() > used.size() + 1) {
				Transform_Indices_Complement(not_used, &n_greatest, &n_greatest + 1);
				search_tree[i][j].int_result.swap(not_used);
			} else {
				Transform_Indices_Complement(used, p_graph->i + p_graph->p[n_greatest],
					p_graph->i + p_graph->p[n_greatest + 1]);
				used.insert(std::lower_bound(used.begin(), used.end(), n_greatest), n_greatest);
				search_tree[i][j].int_result.swap(used);
			}
			// choose the bigger MIS

			cs_spfree((cs*)search_tree[i + 1][j * 2 + 0].p_graph);
			cs_spfree((cs*)search_tree[i + 1][j * 2 + 1].p_graph);
			// delete the graphs
		}

		//printf(".");
	}
	//printf("\n");
	// reduce the search tree results

	return search_tree[0][0].int_result;
	// the result is in the root
}

bool CSchurOrdering::b_IsConnected(const cs *p_graph) // throw(std::bad_alloc)
{
	const size_t n = p_graph->n;
	if(!n)
		return true;

	size_t n_connected_num = 0;
	// calculates the number of connected vertices with vertex 0

	std::vector<bool> close(n, false);
	std::vector<size_t> open;
	open.push_back(0);
	while(!open.empty()) {
		size_t v = open.back();
		open.erase(open.end() - 1);
		if(close[v])
			continue; // already closed
		close[v] = true;
		++ n_connected_num;
		// get a vertex

		for(size_t p0 = p_graph->p[v], p1 = p_graph->p[v + 1]; p0 < p1; ++ p0) {
			size_t v1 = p_graph->i[p0];
			if(!close[v1] && std::find(open.begin(), open.end(), v1) == open.end())
				open.push_back(v1);
		}
		// add all the neighbours to open
	}
	// recurse the graph in DFS fashion; maybe unnecessarily complicated

	return n_connected_num == n;
}

/*void CSchurOrdering::Get_MaxConnectedSet(const cs *p_graph, std::vector<size_t> &r_max_conn) // not required
{
	const size_t n = p_graph->n;

	r_max_conn.clear();

	size_t n_connected_so_far = 0;
	size_t n_max_connected_size = 0;

	std::vector<size_t> open;
	std::vector<bool> close(n, false);
	std::vector<size_t> connected_set;
	for(;;) {
		size_t n_not_closed = 0;
		for(; n_not_closed < n; ++ n_not_closed) {
			if(!close[n_not_closed])
				break;
		}
		if(n_not_closed == n)
			break; // all vertices closed
		// find a vertex that is not closed

		connected_set.clear();
		open.push_back(n_not_closed);
		while(!open.empty()) {
			size_t v = open.back();
			open.erase(open.end() - 1);
			if(close[v])
				continue; // already closed
			close[v] = true;
			connected_set.push_back(v);
			// get a vertex

			for(size_t p0 = p_graph->p[v], p1 = p_graph->p[v + 1]; p0 < p1; ++ p0) {
				size_t v1 = p_graph->i[p0];
				if(!close[v1] && std::find(open.begin(), open.end(), v1) == open.end())
					open.push_back(v1);
			}
			// add all the neighbours to open
		}
		// get connected set with n_not_closed

		n_connected_so_far += connected_set.size();
		if(connected_set.size() > n_max_connected_size) {
			n_max_connected_size = connected_set.size();
			r_max_conn.swap(connected_set);
		}
		// look for the greatest connected set

		if(n - n_connected_so_far < n_max_connected_size)
			break; // early termination
		// even if all the remaining vertices were connected, it wouldn't be enough
	}

	_ASSERTE(!n || !r_max_conn.empty());
	// there should be at least one vertex in the maximum
	// connected set, in case the graph is not empty
}*/

void CSchurOrdering::Get_MinConnectedSet(const cs *p_graph, std::vector<size_t> &r_min_conn) // throw(std::bad_alloc)
{
	const size_t n = p_graph->n;

	r_min_conn.clear();
	if(!n)
		return;

	size_t n_min_connected_size = n + 1;

	std::vector<size_t> open;
	std::vector<bool> close(n, false);
	std::vector<size_t> connected_set;
	for(;;) {
		size_t n_not_closed = 0;
		for(; n_not_closed < n; ++ n_not_closed) {
			if(!close[n_not_closed])
				break;
		}
		if(n_not_closed == n)
			break; // all vertices closed
		// find a vertex that is not closed

		connected_set.clear();
		open.push_back(n_not_closed);
		while(!open.empty()) {
			size_t v = open.back();
			open.erase(open.end() - 1);
			if(close[v])
				continue; // already closed
			close[v] = true;
			connected_set.push_back(v);
			// get a vertex

			for(size_t p0 = p_graph->p[v], p1 = p_graph->p[v + 1]; p0 < p1; ++ p0) {
				size_t v1 = p_graph->i[p0];
				if(!close[v1] && std::find(open.begin(), open.end(), v1) == open.end())
					open.push_back(v1);
			}
			// add all the neighbours to open
		}
		// get connected set with n_not_closed

		if(connected_set.size() < n_min_connected_size) {
			n_min_connected_size = connected_set.size();
			r_min_conn.swap(connected_set);

			if(n_min_connected_size == 1)
				break;
			// won't get any smaller now
		}
		// look for the smallest connected set
	}

	_ASSERTE(!n || !r_min_conn.empty());
	// there should be at least one vertex in the maximum
	// connected set, in case the graph is not empty

	std::sort(r_min_conn.begin(), r_min_conn.end());
	// p_Subgraph() needs a sorted set
}

void CSchurOrdering::Complement_VertexSet(std::vector<size_t> &r_complement,
	const std::vector<size_t> &r_set, size_t n_graph_size) // throw(std::bad_alloc)
{
	_ASSERTE(b_IsSortedSet(r_set));

	const size_t n = n_graph_size; // rename
	r_complement.clear();
	if(!r_set.empty()) {
		if(r_set.size() == n)
			return;
		// complement of everything is nothing

		for(size_t i = 0, m = r_set.size(); i < m; ++ i) {
			if(!i) {
				for(size_t j = 0, e = r_set[i]; j < e; ++ j)
					r_complement.push_back(j);
				// include all the vertices before the first one in the set
			} else {
				for(size_t j = r_set[i - 1] + 1, e = r_set[i]; j < e; ++ j)
					r_complement.push_back(j);
				// include all the vertices between two set elements
			}
		}
		for(size_t j = r_set.back() + 1; j < n; ++ j)
			r_complement.push_back(j);
		// include all the vertices after the last one in the set
	} else {
		for(size_t i = 0; i < n; ++ i)
			r_complement.push_back(i);
		// complement of nothing is everything
	}

	_ASSERTE(b_IsSortedSet(r_complement));
}

/*
 *								=== ~CSchurOrdering ===
 */

/*
 *								=== CLinearSolver_DenseEigen ===
 */

void CLinearSolver_DenseEigen::Free_Memory()
{
	{
		Eigen::MatrixXd empty;
		std::swap(empty, m_t_lambda);
	}
	{
		_TyDecomposition empty;
		std::swap(empty, m_t_factorization);
	}
}

bool CLinearSolver_DenseEigen::Solve_PosDef(const CUberBlockMatrix &r_lambda,
	Eigen::VectorXd &r_eta) // throw(std::bad_alloc)
{
	r_lambda.Convert_to_Dense(m_t_lambda);

	//m_t_factorization.compute(m_t_lambda.selfadjointView<Eigen::Upper>());
	// LU

	m_t_factorization.compute(m_t_lambda);
	if(m_t_factorization.info() != Eigen::Success)
		return false;
	// Cholesky

	r_eta = m_t_factorization.solve(r_eta);
	// solve

	return true;
}

/*bool CLinearSolver_DenseEigen::Dense_Inverse(Eigen::Ref<Eigen::MatrixXd> &r_dest,
	const Eigen::Ref<Eigen::MatrixXd> &r_src, bool b_upper_storage) // throw(std::bad_alloc, std::runtime_error)
{
	typedef Eigen::PartialPivLU<Eigen::MatrixXd> LUDecomposition;

	LUDecomposition factorization;
	if(b_upper_storage)
		factorization.compute(r_src.selfadjointView<Eigen::Upper>());
	else
		factorization.compute(r_src);
	//if(factorization.info() != Eigen::Success) // it always does
	//	return false;
	// LU

	r_dest = factorization.inverse();
	// invert

	return true;
}

bool CLinearSolver_DenseEigen::Dense_Inverse_Parallel(Eigen::Ref<Eigen::MatrixXd> &r_dest,
	const Eigen::Ref<Eigen::MatrixXd> &r_src, bool b_upper_storage) // throw(std::bad_alloc, std::runtime_error)
{
	Eigen::setNbThreads(omp_get_max_threads()); // ...

	bool b_result;
	try {
		b_result = Dense_Inverse(r_dest, r_src, b_upper_storage);
	} catch(std::exception &r_exc) {
		Eigen::setNbThreads(1);
		throw(r_exc);
	}

	Eigen::setNbThreads(1);

	return b_result;
}*/

void CLinearSolver_DenseEigen::Start_ParallelEigenSection()
{
#ifdef _OPENMP
	Eigen::setNbThreads(omp_get_max_threads()); // ...
#endif // _OPENMP
}

void CLinearSolver_DenseEigen::End_ParallelEigenSection()
{
#ifdef _OPENMP
	Eigen::setNbThreads(1);
#endif // _OPENMP
}

/*
 *								=== ~CLinearSolver_DenseEigen ===
 */
