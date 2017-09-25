/*
								+-----------------------------------+
								|                                   |
								| *** L factor nonlinear solver *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|   NonlinearSolver_FastL_Base.h    |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_SOLVER_FAST_L_UTILS
#define __NONLINEAR_SOLVER_FAST_L_UTILS

/**
 *	@file include/slam/NonlinearSolver_FastL_Base.h
 *	@brief utilitites for nonlinear solvers, working above the L factor matrix
 *	@author -tHE SWINe-
 *	@date 2015-06-25
 */

#include "slam/BlockMatrix.h"
#include "slam/LinearSolver_UberBlock.h"

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1
 *	@brief if defined, solution is calcualted at each step, otherwise it is calculated
 *		only at the specified intervals (but is ready to be calculated at each step quickly)
 */
#define __NONLINEAR_SOLVER_FAST_L_BACKSUBSTITUTE_EACH_1

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE
 *	@brief if defined, R is always used to calculate updates, even in the subsequent iterations,
 *		so that when the solver finishes, R is up to date and does not need to be refreshed again
 */
#define __NONLINEAR_SOLVER_FAST_L_ALWAYS_L_UPDATE

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING
 *	@brief if defined, permutation folding is verified by comparing parts
 *		of lambda (lambda00 and lambda11)
 *	@note This slows down quite a bit, should not be enabled for production builds.
 */
//#define __NONLINEAR_SOLVER_FAST_L_VERIFY_PERM_FOLDING

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS
 *	@brief enables writes of diagnostic data (timing samples, ...)
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_TIMESTEPS

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2
 *	@brief enables writes of chi2 errors at each step
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE
 *	@brief if defined, chi2 is calculated at the last edge, just before introducing a new vertex,
 *		that gives a different chi2 than calculating chi2 just after introducing a new vertex (default)
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_CHI2_AT_LAST_EDGE

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY
 *	@brief enables writes of density of R given different ordering strategies
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_DENSITY

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES
 *	@brief if defined, enables writes of timing of different R update variants
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_L_UPDATE_VARIANT_TIMES

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING
 *	@brief replaces timer implementation with dummy timer to avoid costy runtime
 *		library calls to get the time (more costy on windows than on linux)
 */
#define __NONLINEAR_SOLVER_FAST_L_DETAILED_TIMING

#ifndef _DEBUG

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY
 *	@brief enables computation of dense cholesky for small loops (a speed optimization)
 */
#define __NONLINEAR_SOLVER_FAST_L_ENABLE_DENSE_CHOLESKY

#endif // !_DEBUG

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_PARALLEL_MATMULT_THRESH
 *	@brief matrix size threshold for parallel multiplication
 *		in R update (in blocks, used to calculate \f$R_{10}R_{10}^T\f$ and \f$R_{11}R_{11}^T\f$)
 */
#define __NONLINEAR_SOLVER_FAST_L_PARALLEL_MATMULT_THRESH 200

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
 *	@brief dump RSS 2013 matrix animation data
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

/**
 *	@def __NONLINEAR_SOLVER_FAST_L_DUMP_WAFR2014_PRESENTATION_ANIMATION_DATA
 *	@brief dump WAFR 2014 matrix animation data
 */
//#define __NONLINEAR_SOLVER_FAST_L_DUMP_WAFR2014_PRESENTATION_ANIMATION_DATA

/**
 *	@brief namespace with fast L solver utilities
 */
namespace fL_util {

/**
 *	@brief configuration stored as enum
 */
enum {
#ifdef __BASE_TYPES_ALLOW_CONST_VERTICES
	no_ConstVertices = false /**< @brief no const vertices flag @note This is used to optimize away some const-ness checks which the compiler cannot optimize by itself. */
#else // __BASE_TYPES_ALLOW_CONST_VERTICES
	no_ConstVertices = true /**< @brief no const vertices flag @note This is used to optimize away some const-ness checks which the compiler cannot optimize by itself. */
#endif // __BASE_TYPES_ALLOW_CONST_VERTICES
};

/**
 *	@brief a simple native solver predicate
 *	@tparam CLinearSolver is linear solver type
 *	@deprecated \ref ::CIsNativeSolver was moved to \ref include/slam/LinearSolver_UberBlock.h.
 */
template <class CLinearSolver>
class CIsNativeSolver {
public:
	/**
	 *	@brief result, stored as enum
	 */
	enum {
		b_result = ::CIsNativeSolver<CLinearSolver>::b_result /**< @brief result of comparison */
	};
};

/**
 *	@brief function object that calls omega hessian block allocation and evaluation for all edges
 */
class CCalculateOmega {
protected:
	CUberBlockMatrix &m_r_omega; /**< @brief reference to the omega matrix (out) */
	size_t m_n_min_elem_order; /**< @brief minimal order of vertex to be filled in omega (in elements) */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] r_omega is reference to the omega matrix
	 *	@param[in] n_min_elem_order is minimal order of vertex to be filled in omega (in elements)
	 */
	inline CCalculateOmega(CUberBlockMatrix &r_omega, size_t n_min_elem_order)
		:m_r_omega(r_omega), m_n_min_elem_order(n_min_elem_order)
	{}

	/**
	 *	@brief function operator
	 *	@tparam _TyEdge is an edge type
	 *	@param[in] r_edge is edge to have its hessian blocks added to omega
	 *	@note This function throws std::bad_alloc.
	 */
	template <class _TyEdge>
	inline void operator ()(const _TyEdge &r_edge) // throw(std::bad_alloc)
	{
		r_edge.Calculate_Omega(m_r_omega, m_n_min_elem_order);
	}
};

/**
 *	@brief function object that calls R factor block allocation for all vertices
 */
class CAlloc_RBlocks {
protected:
	CUberBlockMatrix &m_r_R; /**< @brief reference to the R matrix (out) */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_R is reference to the lambda matrix
	 */
	inline CAlloc_RBlocks(CUberBlockMatrix &r_R)
		:m_r_R(r_R)
	{}

	/**
	 *	@brief function operator
	 *	@tparam _TyVertex is vertex type
	 *	@param[in,out] r_vertex is vertex to have hessian blocks allocated in R
	 *	@note This function throws std::bad_alloc.
	 */
	template <class _TyVertex>
	inline void operator ()(_TyVertex &r_vertex) // throw(std::bad_alloc)
	{
		r_vertex.Alloc_LBlocks(m_r_R); // t_odo - alloc R blocks as well
	}
};

/**
 *	@brief calculates (ordered) omega matrix
 *
 *	@tparam CEdgePool is edge pool type
 *
 *	@param[out] r_omega is filled with the omega matrix
 *	@param[in] b_identity_perm is identity permutation flag (if set, the omega is unordered)
 *	@param[in] n_first_edge is the first edge to be added to omega
 *	@param[in] n_vertex_min is the minumum id of the ids of vertices referenced by the omega edges
 *	@param[in] n_order_min is the minumum permuted id of the ids of vertices referenced by the omega edges
 *		(it might not correspond to n_vertex_min if the ordering is not monotonically increasing)
 *	@param[in] r_edge_pool is reference to the optimized system edge pool
 *	@param[in] r_lambda is reference to the unordered lambda matrix
 *		(need it for vertex order lookup (both direct and inverse), the values are not accessed)
 *	@param[in] r_lambda_perm is reference to the ordered lambda matrix
 *		(need it for permuted vertex order lookup (direct), the values are not accessed)
 *	@param[in] p_lambda_block_ordering is pointer to block ordering of the lambda matrix
 *	@param[in] n_lambda_block_ordering_size is the number of elements of lambda block ordering
 *
 *	@note This function throws std::bad_alloc.
 */
template <class CEdgePool>
void Calculate_Omega(CUberBlockMatrix &r_omega, const bool b_identity_perm,
	const size_t n_first_edge, const size_t n_vertex_min, const size_t n_order_min,
	const CEdgePool &r_edge_pool, const CUberBlockMatrix &r_lambda, const CUberBlockMatrix &r_lambda_perm,
	const size_t *p_lambda_block_ordering, const size_t UNUSED(n_lambda_block_ordering_size)) // throw(std::bad_alloc) // todo - move this to L ops
{
	_ASSERTE(n_lambda_block_ordering_size == r_lambda.n_BlockColumn_Num());
	_ASSERTE(r_lambda_perm.n_BlockColumn_Num() == r_lambda.n_BlockColumn_Num());
	_ASSERTE(n_vertex_min <= r_lambda.n_BlockColumn_Num());
	_ASSERTE(n_order_min <= r_lambda_perm.n_BlockColumn_Num());
	//_ASSERTE(n_order_min <= p_lambda_block_ordering[n_vertex_min]); // this is not true, it can be a completely different vertex!

	r_omega.Clear();
	// !!

	CUberBlockMatrix unord_omega;
	const size_t n_elem_min_unord = r_lambda.n_BlockColumn_Base(n_vertex_min); // not ordered
	/*r_system.r_Edge_Pool()*/r_edge_pool.For_Each(n_first_edge, /*r_system.r_Edge_Pool()*/r_edge_pool.n_Size(),
		CCalculateOmega((b_identity_perm)? r_omega : unord_omega, n_elem_min_unord));
	// collect unordered omega, either directly to the output variable or to temp in case ordering is needed

	if(!b_identity_perm) {
		const size_t n_elem_order_min = r_lambda_perm.n_BlockColumn_Base(n_order_min);
		_ASSERTE(r_omega.b_Empty());
		_ASSERTE(!unord_omega.b_Empty() && unord_omega.b_SymmetricLayout());
		for(size_t i = 0, n = unord_omega.n_BlockColumn_Num(); i < n; ++ i) {
			size_t n_cols;
			size_t n_col = r_lambda.n_Find_BlockColumn(unord_omega.n_BlockColumn_Base(i) +
				n_elem_min_unord, n_cols); // note that this is ever increasing, could resume search where it last stopped
			_ASSERTE(n_col != size_t(-1));
			_ASSERTE(unord_omega.n_BlockColumn_Column_Num(i) == n_cols);
			// find the corresponding vertex id

			size_t n_col_ord = p_lambda_block_ordering[n_col];
			size_t n_col_base = r_lambda_perm.n_BlockColumn_Base(n_col_ord);
			_ASSERTE(n_col_base >= n_elem_order_min);
			// reorder

			for(size_t j = 0, m = unord_omega.n_BlockColumn_Block_Num(i); j < m; ++ j) {
				size_t n_row = unord_omega.n_Block_Row(i, j);
				CUberBlockMatrix::_TyMatrixXdRef t_block = unord_omega.t_Block_AtColumn(i, j);
				// get a block of the unordered matrix

				size_t n_rows;
				n_row = r_lambda.n_Find_BlockColumn(unord_omega.n_BlockColumn_Base(n_row) +
					n_elem_min_unord, n_rows); // both lambda and omega are symmetric, can use col layout for rows
				_ASSERTE(n_row != size_t(-1));
				_ASSERTE(unord_omega.n_BlockColumn_Column_Num(i) == n_cols);
				// find the corresponding vertex id

				size_t n_row_ord = p_lambda_block_ordering[n_row];
				size_t n_row_base = r_lambda_perm.n_BlockColumn_Base(n_row_ord);
				_ASSERTE(n_row_base >= n_elem_order_min);
				// reorder

				if(n_col_base >= n_row_base)
					r_omega.Append_Block(t_block, n_row_base - n_elem_order_min, n_col_base - n_elem_order_min);
				else
					r_omega.Append_Block(t_block.transpose(), n_col_base - n_elem_order_min, n_row_base - n_elem_order_min);
				// put the block in the reordered matrix, keep it upper-triangular
			}
		}
		// pick the nonzero blocks and scatter them in the destination matrix
		// (probably ok, since there will only be a few blocks)
		// or could also fragment omega and use the "built-in" reordering function
	}
	// reorder omega in case this is not identity permutation
	// note that this could be done only once, for the difference of omega,
	// had we not needed it for updating the factorization
}

/**
 *	@brief graph increment information
 */
struct TGraphIncrementInfo {
	size_t n_vertex_min; /**< @brief the minumum id of the ids of vertices referenced by the new edges */
	size_t n_order_min; /**< @brief the minumum permuted id of the ids of vertices referenced by the new edges @note This might not correspond to n_vertex_min if the ordering is not monotonically increasing. */
	std::vector<size_t> incremented_lambda_perm_block_column_list; /**< @brief list of unique sorted permuted vertex ids of vertices referenced by the new edges */
	bool b_identity_perm; /**< @brief identity permutation flag (if set, the part of the permutation corresponding to the vertices referenced by the new edges is an identity permutation. its prefix may not be identity) */

	/**
	 *	@brief constructor; calculates graph increment informaiton
	 *
	 *	@tparam CEdgePool is edge pool type
	 *
	 *	@param[in] n_first_edge is the first edge to be added to omega
	 *	@param[in] r_edge_pool is reference to the optimized system edge pool
	 *	@param[in] p_lambda_block_ordering is pointer to block ordering of the lambda matrix
	 *	@param[in] n_lambda_block_ordering_size is the number of elements of lambda block ordering
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CEdgePool>
	TGraphIncrementInfo(const size_t n_first_edge, const CEdgePool &r_edge_pool,
		const size_t *p_lambda_block_ordering, const size_t n_lambda_block_ordering_size) // throw(std::bad_alloc)
	{
		_ASSERTE(n_first_edge < r_edge_pool.n_Size());
		// make sure that there are some new edges

		n_vertex_min = SIZE_MAX;
		for(size_t i = n_first_edge, n = r_edge_pool.n_Size(); i < n; ++ i) {
			typename CEdgePool::_TyConstBaseRef r_edge = r_edge_pool[i];
			for(size_t j = 0, m = r_edge.n_vertex_num; j < m; ++ j) {
				size_t n_vertex = r_edge.n_Vertex_Id(j);
				if(fL_util::no_ConstVertices || n_vertex < n_lambda_block_ordering_size) {
					n_vertex_min = std::min(n_vertex_min, n_vertex);
					incremented_lambda_perm_block_column_list.push_back(p_lambda_block_ordering[n_vertex]);
				}
				// only non-const vertices (note that n_lambda_block_ordering_size equals vertex pool size)
			}
		}
		std::sort(incremented_lambda_perm_block_column_list.begin(), incremented_lambda_perm_block_column_list.end());
		incremented_lambda_perm_block_column_list.erase(std::unique(incremented_lambda_perm_block_column_list.begin(),
			incremented_lambda_perm_block_column_list.end()), incremented_lambda_perm_block_column_list.end());
		_ASSERTE(!incremented_lambda_perm_block_column_list.empty());
		// calculate min order and the required column list (for marginals update)

		n_order_min = incremented_lambda_perm_block_column_list.front();
		const size_t n_order_max = n_lambda_block_ordering_size; // rename
		// makes sure that max is fixed at the end

		b_identity_perm = true;
		for(size_t i = n_order_min; i < n_order_max; ++ i) {
			if(p_lambda_block_ordering[i] != i) {
				b_identity_perm = false;
				break;
			}
		}
		// see if omega has identity ordering (probably iff the edge is (v0, v1)
		// where v1 = v0 + 1 = m_r_system.r_Vertex_Pool().n_Size() - 1)
	}

	/**
	 *	@brief copy-constructor
	 *	@param[in] r_other is the other graph increment info to copy from
	 *	@note This function throws std::bad_alloc.
	 */
	inline TGraphIncrementInfo(const TGraphIncrementInfo &r_other) // throw(std::bad_alloc)
		:n_vertex_min(r_other.n_vertex_min), n_order_min(r_other.n_order_min),
		incremented_lambda_perm_block_column_list(r_other.incremented_lambda_perm_block_column_list),
		b_identity_perm(r_other.b_identity_perm)
	{}
};

#if 0 // unused function objects

/**
 *	@brief function object that calls lambda hessian block allocation for all edges
 */
class CAlloc_LambdaRBlocks { // t_odo - R probably only allocates blocks on vertices; retain old version of this functor with lambda only for edges
protected:
	CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix (out) */
	CUberBlockMatrix &m_r_R; /**< @brief reference to the R matrix (out) */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_lambda is reference to the lambda matrix
	 *	@param[in] r_R is reference to the lambda matrix
	 */
	inline CAlloc_LambdaRBlocks(CUberBlockMatrix &r_lambda, CUberBlockMatrix &r_R)
		:m_r_lambda(r_lambda), m_r_R(r_R)
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
		r_vertex_or_edge.Alloc_LBlocks(m_r_R); // t_odo - alloc R blocks as well
	}
};

/**
 *	@brief function object that calls lambda hessian block allocation for all edges
 */
class CAlloc_LambdaBlocks {
protected:
	CUberBlockMatrix &m_r_lambda; /**< @brief reference to the lambda matrix (out) */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_lambda is reference to the lambda matrix
	 */
	inline CAlloc_LambdaBlocks(CUberBlockMatrix &r_lambda)
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

/**
 *	@brief function object that calculates and accumulates chi^2 from all the edges
 */
class CSum_ChiSquareError {
protected:
	double m_f_sum; /**< @brief a running sum of \f$\chi^2\f$ errors */

public:
	/**
	 *	@brief default constructor
	 */
	inline CSum_ChiSquareError()
		:m_f_sum(0)
	{}

	/**
	 *	@brief function operator
	 *	@tparam _TyEdge is edge type
	 *	@param[in] r_t_edge is edge to output its part R error vector
	 */
	template <class _TyEdge>
	inline void operator ()(const _TyEdge &r_t_edge)
	{
		m_f_sum += r_t_edge.f_Chi_Squared_Error();
	}

	/**
	 *	@brief gets the current value of the accumulator
	 *	@return Returns the current value of the accumulator.
	 */
	inline operator double() const
	{
		return m_f_sum;
	}
};

/**
 *	@brief function object that updates states of all the vertices
 */
class CUpdateEstimates {
protected:
	const Eigen::VectorXd &m_r_dx; /**< @brief vector of differences */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_dx is reference to the vector of differences
	 */
	inline CUpdateEstimates(const Eigen::VectorXd &r_dx)
		:m_r_dx(r_dx)
	{}

	/**
	 *	@brief updates vertex state
	 *	@tparam _TyVertex is type of vertex
	 *	@param[in,out] r_t_vertex is reference to vertex, being updated
	 */
	template <class _TyVertex>
	inline void operator ()(_TyVertex &r_t_vertex) const
	{
		r_t_vertex.Operator_Plus(m_r_dx);
	}
};

/**
 *	@brief function object that calculates hessians in all the edges
 */
class CCalculate_Lambda {
public:
	/**
	 *	@brief function operator
	 *	@tparam _TyVertexOrEdge is edge type
	 *	@param[in] r_t_vertex_or_edge is vertex or edge to update it's hessians
	 */
	template <class _TyVertexOrEdge>
	inline void operator ()(_TyVertexOrEdge &r_t_vertex_or_edge) const
	{
		r_t_vertex_or_edge.Calculate_Hessians();
	}
};

#endif // 0

} // ~fL_util

/** @} */ // end of group

#endif // !__NONLINEAR_SOLVER_FAST_L_UTILS
