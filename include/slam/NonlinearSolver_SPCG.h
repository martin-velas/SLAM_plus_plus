/*
								+-----------------------------------+
								|                                   |
								|  ***  SPCG nonlinear solver  ***  |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|      NonlinearSolver_SPCG.h       |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __NONLINEAR_BLOCKY_SOLVER_SPCG_INCLUDED
#define __NONLINEAR_BLOCKY_SOLVER_SPCG_INCLUDED

/**
 *	@file include/slam/NonlinearSolver_SPCG.h
 *	@brief nonlinear blocky solver working above the A matrix, optimized using subgraph preconditioned conjugate gradient
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 */

#include "slam/FlatSystem.h"

/** \addtogroup nlsolve
 *	@{
 */

/**
 *	@def __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL
 *	@brief if defined, residual norm of the CG solution is calculated and saved in
 *		residualRound*.m file (one for every nonlinear solver iteration)
 */
//#define __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL

/**
 *	@def __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION
 *	@brief if defined, the CG solution is compared to the exact solution calculated using the A solver
 */
//#define __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION

/**
 *	@def __NONLINEAR_SOLVER_SPCG_DUMP_VARS
 *	@brief dumps system variables as an .m file in debug
 */
//#define __NONLINEAR_SOLVER_SPCG_DUMP_VARS

#error "fatal error: this solver is deprecated (was never finished)"

/**
 *	@brief nonlinear blocky solver working above the A matrix, optimized using subgraph preconditioned conjugate gradient
 *
 *	@tparam CSystem is optimization system type
 *	@tparam CLinearSolver is linear solver type
 *	@tparam CAMatrixBlockSizes is list of block sizes in the Jacobian matrix
 *	@tparam CLambdaMatrixBlockSizes is list of block sizes in the information (Hessian) matrix
 */
template <class CSystem, class CLinearSolver, class CAMatrixBlockSizes = typename CSystem::_TyJacobianMatrixBlockList,
	class CLambdaMatrixBlockSizes = typename CSystem::_TyHessianMatrixBlockList>
class CNonlinearSolver_SPCG {
public:
	typedef CSystem _TySystem; /**< @brief system type */
	typedef CLinearSolver _TyLinearSolver; /**< @brief linear solver type */

	typedef typename CSystem::_TyBaseVertex _TyBaseVertex; /**< @brief the data type for storing vertices */
	typedef typename CSystem::_TyVertexTypelist _TyVertexTypelist; /**< @brief list of vertex types */
	typedef typename CSystem::_TyBaseEdge _TyBaseEdge; /**< @brief the data type for storing measurements */
	typedef typename CSystem::_TyEdgeTypelist _TyEdgeTypelist; /**< @brief list of edge types */

	typedef typename CSystem::_TyVertexMultiPool _TyVertexMultiPool; /**< @brief vertex multipool type */
	typedef typename CSystem::_TyEdgeMultiPool _TyEdgeMultiPool; /**< @brief edge multipool type */

	typedef typename CLinearSolver::_Tag _TySolverTag; /**< @brief linear solver tag */
	typedef CLinearSolverWrapper<_TyLinearSolver, _TySolverTag> _TyLinearSolverWrapper; /**< @brief wrapper for linear solvers (shields solver capability to solve blockwise) */

	typedef /*typename CUniqueTypelist<*/CAMatrixBlockSizes/*>::_TyResult*/ _TyAMatrixBlockSizes; /**< @brief possible block matrices, that can be found in A */
	typedef /*typename*/ CLambdaMatrixBlockSizes /*CTransformTypelist<_TyAMatrixBlockSizes,
		fbs_ut::CEigenToTransposeEigen>::_TyResult*/ _TyATransposeMatrixBlockSizes; /**< @brief possible block matrices, that can be found in transpose A */

	/**
	 *	@brief solver interface properties, stored as enum (see also CSolverTraits)
	 */
	enum {
		solver_HasDump = true, /**< @brief timing statistics support flag */
		solver_HasChi2 = true, /**< @brief Chi2 error calculation support flag */
		solver_HasMarginals = false, /**< @brief marginal covariance support flag */
		solver_HasGaussNewton = false, /**< @brief Gauss-Newton support flag */
		solver_HasLevenberg = false, /**< @brief Levenberg-Marquardt support flag */
		solver_HasGradient = true, /**< @brief gradient-based linear solving support flag */
		solver_HasSchur = false, /**< @brief Schur complement support flag */
		solver_HasDelayedOptimization = false, /**< @brief delayed optimization support flag */
		solver_IsPreferredBatch = false, /**< @brief preferred batch solver flag */
		solver_IsPreferredIncremental = false, /**< @brief preferred incremental solver flag */
		solver_ExportsJacobian = false, /**< @brief interface for exporting jacobian system matrix flag */
		solver_ExportsHessian = false, /**< @brief interface for exporting hessian system matrix flag */
		solver_ExportsFactor = false /**< @brief interface for exporting factorized system matrix flag */
	};

protected:
	CSystem &m_r_system; /**< @brief reference to the system */
	CLinearSolver m_linear_solver; /**< @brief linear solver */

	CUberBlockMatrix m_A; /**< @brief the A matrix (built incrementally) */
	Eigen::VectorXd m_v_error; /**< @brief error vector */
	Eigen::VectorXd m_v_dx; /**< @brief dx vector */
	size_t m_n_edges_in_A; /**< @brief number of edges already in A */
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_last_optimized_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_step; /**< @brief counter of incremental steps modulo m_n_nonlinear_solve_threshold */
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
	size_t m_n_linear_solve_threshold; /**< @brief step threshold for linear solve */
	size_t m_n_nonlinear_solve_threshold; /**< @brief step threshold for nonlinear solve */
	size_t m_n_nonlinear_solve_max_iteration_num; /**< @brief maximal number of iterations in incremental nonlinear solve */
	double m_f_nonlinear_solve_error_threshold; /**< @brief error threshold in incremental nonlinear solve */ // t_odo - document these in elementwise A and L
	bool m_b_verbose; /**< @brief verbosity flag */

	size_t m_n_real_step; /**< @brief counter of incremental steps (no modulo) */

	bool m_b_system_dirty; /**< @brief system updated without relinearization flag */

	size_t m_n_iteration_num; /**< @brief number of linear solver iterations */
	double m_f_serial_time; /**< @brief time spent in serial section */
	double m_f_ata_time; /**< @brief time spent in A^T * A section */
	double m_f_premul_time; /**< @brief time spent in b * A section */
	double m_f_chol_time; /**< @brief time spent in Choleski() section */
	double m_f_norm_time; /**< @brief time spent in norm calculation section */

	double m_f_A_refresh_time; /**< @brief time spent in refreshing A */
	double m_f_spanning_tree_time; /**< @brief time spent in calculating the spanning tree */
	double m_f_A_perm_time; /**< @brief time spent in calculating the permutation of A */

	double m_f_rhs_time; /**< @brief time spent in refreshing the right hand side */
	double m_f_perm_b_time; /**< @brief time spent in permuting the right hand side */
	double m_f_qr_time; /**< @brief time spent in calculating QR of A */
	double m_f_solve_time; /**< @brief time spent in solving QR of A */

	CTimer m_timer; /**< @brief timer object */

	bool m_b_had_loop_closure; /**< @brief (probable) loop closure flag */

	/**
	 *	@brief matrix ordering calculator (CAMD wrapper)
	 */
	class CMatrixOrdering { // t_odo - fill throws, and in related classes
	protected:
		std::vector<size_t> m_ordering_expand; /**< @brief storage for the elementwise ordering vector */
		//std::vector<size_t> m_ordering_invert; /**< @brief storage for the inverse elementwise ordering vector */

	public:
		/**
		 *	@brief calculates elementwise ordering by expanding blockwise ordering on a block matrix
		 *
		 *	@param[in] m_ordering is the ordering to invert
		 *	@param[in] r_A is the block matrix (must be symmetric)
		 *
		 *	@return Returns const pointer to the ordering vector (not to be deleted).
		 *
		 *	@note The buffer for elementwise ordering is reused in the next function call,
		 *		invalidating the previous result (create more instances of CMatrixOrdering
		 *		if multiple elementwise orderings need to exist at the same time).
		 *	@note This function throws std::bad_alloc.
		 */
		const size_t *p_ExpandBlockOrdering(const std::vector<size_t> &m_ordering, const CUberBlockMatrix &r_A) // throw(std::bad_alloc)
		{
			const size_t n_column_num = r_A.n_Row_Num();
			const size_t n_column_block_num = r_A.n_BlockRow_Num();

			_ASSERTE(m_ordering.size() == n_column_block_num);
			// make sure the ordering can be possibly applied to the matrix

			if(m_ordering_expand.size() < n_column_num) {
				m_ordering_expand.clear();
				m_ordering_expand.resize(std::max(n_column_num, 2 * m_ordering_expand.capacity()));
			}
			size_t n_scalar_offset = 0;
			const size_t *p_order_ptr = &m_ordering[0];
			for(size_t i = 0; i < n_column_block_num; ++ i, ++ p_order_ptr) {
				const size_t n_order = *p_order_ptr;
				size_t n_block_base = r_A.n_BlockRow_Base(n_order); // wonder what would happen if it was m_ordering[n_order] or something like that ... would that get a correct column in the inverse ordering?
				size_t n_block_width = r_A.n_BlockRow_Row_Num(n_order);
				for(size_t j = 0; j < n_block_width; ++ j, ++ n_scalar_offset, ++ n_block_base)
					m_ordering_expand[n_scalar_offset] = n_block_base;
			}
			_ASSERTE(n_scalar_offset == n_column_num);
			// blow up the permutation from block level to scalar level

			return &m_ordering_expand[0];
		}
	};

public:
	/**
	 *	@brief initializes the nonlinear solver
	 *
	 *	@param[in] r_system is the system to be optimized
	 *		(it is only referenced, not copied - must not be deleted)
	 *	@param[in] n_linear_solve_threshold is the step threshold
	 *		for linear solver to be called (0 = disable)
	 *	@param[in] n_nonlinear_solve_threshold is the step threshold
	 *		for nonlinear solver to be called (0 = disable)
	 *	@param[in] n_nonlinear_solve_max_iteration_num is maximal
	 *		number of iterations in nonlinear solver
	 *	@param[in] f_nonlinear_solve_error_threshold is error threshold
	 *		for the nonlinear solver
	 *	@param[in] b_verbose is verbosity flag
	 *	@param[in] linear_solver is linear solver instance
	 *	@param[in] b_use_schur is Schur complement trick flag (not supported)
	 */
	CNonlinearSolver_SPCG(CSystem &r_system, size_t n_linear_solve_threshold = 0,
		size_t n_nonlinear_solve_threshold = 0, size_t n_nonlinear_solve_max_iteration_num = 5,
		double f_nonlinear_solve_error_threshold = .01, bool b_verbose = false,
		CLinearSolver linear_solver = CLinearSolver(), bool UNUSED(b_use_schur) = false)
		:m_r_system(r_system), m_linear_solver(linear_solver), m_n_edges_in_A(0),
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_last_optimized_vertex_num(0),
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_step(0),
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		m_n_linear_solve_threshold(n_linear_solve_threshold),
		m_n_nonlinear_solve_threshold(n_nonlinear_solve_threshold),
		m_n_nonlinear_solve_max_iteration_num(n_nonlinear_solve_max_iteration_num),
		m_f_nonlinear_solve_error_threshold(f_nonlinear_solve_error_threshold),
		m_b_verbose(b_verbose), m_n_real_step(0), m_b_system_dirty(false),
		m_n_iteration_num(0), m_f_serial_time(0), m_f_ata_time(0), m_f_premul_time(0),
		m_f_chol_time(0), m_f_norm_time(0), m_b_had_loop_closure(false)
	{
		_ASSERTE(!m_n_nonlinear_solve_threshold || !m_n_linear_solve_threshold); // only one of those

		m_f_A_refresh_time = 0;
		m_f_spanning_tree_time = 0;
		m_f_A_perm_time = 0;

		m_f_rhs_time = 0;
		m_f_perm_b_time = 0;
		m_f_qr_time = 0;
		m_f_solve_time = 0;
	}

	/**
	 *	@brief calculates spanning tree of an oriented graph
	 *
	 *	@param[out] p_dest is destination array for edges of the spanning tree
	 *	@param[in] p_graph is a graph, represented by a sparse matrix (columns are vertices, rows are edges)
	 *	@param[in] r_graph_edges is a list of graph edges to speed up adjacency
	 *		lookup (otherwise have to transpose p_graph)
	 *	@param[in] n_starting_vertex is a vertex to root the spanning tree at
	 *
	 *	@return Returns number of edges in the spanning tree.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static size_t n_OrientedSpanningTree(size_t *p_dest, const cs *p_graph,
		const _TyEdgeMultiPool &r_graph_edges, size_t n_starting_vertex = 0) // throw(std::bad_alloc)
	{
		*p_dest = n_starting_vertex; // UF
		size_t n_perm_dest = 1;
		// 0 already contains the UF (assume we want to keep it in A1)

		size_t n_vertex_num = p_graph->n;
		if(n_vertex_num > 0) {
			const cs *p_A_struct = p_graph;
			const csi *bAp = p_A_struct->p;
			const csi *bAi = p_A_struct->i;
			// get structure of the matrix

			std::vector<bool> is_T; // todo - reuse bool vector
			is_T.resize(n_vertex_num, false);
			// make a set of T (to contain vertices); initially empty

			is_T[0] = true;
			// the 0th vertex is in T (we begin with that)

			std::vector<size_t> backlog; // vector used as a queue (suboptimal, but will rarely iterate)
			backlog.push_back(0);
			while(!backlog.empty()) {
				size_t n_vertex = backlog.front();
				_ASSERTE(n_vertex < size_t(p_A_struct->n));
				backlog.erase(backlog.begin());
				// get a vertex

				_ASSERTE(is_T[n_vertex]); // vertices that are to be examined should already be in the set
				size_t UNUSED(n_orig_vertex) = n_vertex;

				//printf("starting from " PRIsize " ...\n", n_orig_vertex); // debug

				for(;;) {
					size_t n_begin = bAp[n_vertex];
					size_t n_end = bAp[n_vertex + 1];
					// get range of edges leading from that vertex

					size_t n_next_vertex = size_t(-1);
					for(; n_begin < n_end; ++ n_begin) {
						size_t n_edge = bAi[n_begin];
						// get an edge that touches this vertex

						if(!n_edge)
							continue;
						-- n_edge;
						// skip UF

						const _TyBaseEdge &r_edge = r_graph_edges[n_edge];
						_ASSERTE(r_edge.n_Vertex_Num() == 2); // won't work for hyperedges
						size_t n_other_vertex = r_edge.n_Vertex_Id(1);
						_ASSERTE(n_other_vertex < n_vertex_num); // make sure that this is not const vertex (would just most likely skip it but i'm not going to debug this right now)
						//if(n_other_vertex == n_vertex)
						//	n_other_vertex = r_edge.n_Vertex_Id(0); // graph is oriented
						if(n_other_vertex == n_vertex || is_T[n_other_vertex])
							continue;
						// if the edge leads to the same vertex (UF?)
						// or to a vertex that is already in T, do not take it

						p_dest[n_perm_dest] = n_edge + 1; // skip UF again
						++ n_perm_dest;
						// add edge to the permutation (the spanning tree)

						n_next_vertex = n_other_vertex;
						_ASSERTE(n_next_vertex != n_orig_vertex); // returning to the origin vertex would break it; it should never happen
						// otherwise traverse to other vertex

						//printf("taking edge " PRIsize ": (" PRIsize " -> " PRIsize ") ...\n", n_edge, n_vertex, n_next_vertex); // debug

						is_T[n_next_vertex] = true;
						// also, the next vertex is now part of T

						for(; n_begin < n_end; ++ n_begin) {
							size_t n_edge = bAi[n_begin];
							// get an edge that touches this vertex

							if(!n_edge)
								continue;
							-- n_edge;
							// skip UF (it can't actually occur here, though)

							const _TyBaseEdge &r_edge = r_graph_edges[n_edge];
							_ASSERTE(r_edge.n_Vertex_Num() == 2); // won't work for hyperedges
							size_t n_other_vertex = r_edge.n_Vertex_Id(1);
							_ASSERTE(n_other_vertex < n_vertex_num); // make sure that this is not const vertex (would just most likely skip it but i'm not going to debug this right now)
							//if(n_other_vertex == n_vertex)
							//	n_other_vertex = r_edge.n_Vertex_Id(0); // graph is oriented
							if(n_other_vertex == n_vertex || is_T[n_other_vertex])
								continue;
							// if the edge leads to the same vertex (UF?)
							// or to a vertex that is already in T, do not take it

							//printf("there are multiple ways from " PRIsize ", saving for later\n", n_vertex); // debug
							backlog.push_back(n_vertex);
							// there is a neighbor vertex that is not visited yet, save this vertex
							// as there is an edge that can be taken to that vertex from here

							break;
						}
						// see if there are other vertices to be visited
						// and save this node for later if there are ...

						break;
					}

					if(n_next_vertex == size_t(-1))
						break;
					n_vertex = n_next_vertex;
					// if there is no safe edge, go back
				}

				//if(n_orig_vertex == n_vertex)
				//	backlog.push_back(n_vertex); // the tree is growing from the other side, to neighbours *not* in the tree
				// this might come to a vertex that does not have any neighbours in the spanning tree
				// then we process the rest of vertices first and come back to this one later
			}
			// note this will loop infinitely if the graph is disconnected (does not happen with SLAM)

			//std::sort(p_dest, p_dest + n_perm_dest);
			// sort it? gives more diagonal matrix on landmark datasets (otherwise the landmarks tend to skip vertices and generate disorder)

			for(size_t i = 0; i < n_vertex_num; ++ i) {
				_ASSERTE(is_T[i]); // in debug, throw user breakpoint before the exception
				if(!is_T[i]) {
					char p_s_error[256];
					sprintf(p_s_error, "vertex " PRIsize " is not in T", i);
					throw std::runtime_error(p_s_error);
				}
			}

			// to calculate spanning tree (Jarnik), do:
			//		* visit a vertex, get a safe edge (the first edge, we have no weights) and add it to T
			//			* note that vertices are columns
			//		* repeat untill all vertices are covered

			//cs_spfree(p_A_struct); // not here
			// cleanup

			printf("backlog capacity: " PRIsize "\n", backlog.capacity()); // debug
		}
		// slightly questionable algorithm to calculate minimum spanning tree (Jarnik)

		return n_perm_dest;
	}

	/**
	 *	@brief configuration stored as enum
	 */
	enum {
		fast_vec_Step = 8192 /**< @brief chonk size for the fast vector functions */
	};

	/**
	 *	@brief calculates vector norm; optimized for very long vectors
	 *	@param[in] r_vec is a dense vector
	 *	@return Returns squared L2 norm of the given vector.
	 */
	static double f_FastSquaredVectorNorm(const Eigen::VectorXd &r_vec)
	{
		double f_norm = 0;
		for(size_t i = 0, n = r_vec.rows();; i += fast_vec_Step) {
			if(i + fast_vec_Step < n) {
				f_norm += r_vec.segment<fast_vec_Step>(i).squaredNorm();
				// potentially uses SSE, even though the vector might be unaligned (not in x64)
			} else {
				f_norm += r_vec.segment(i, n - i).squaredNorm();
				// the misaligned part comes last; keeps the other addresses aligned
				break;
			}
		}

		_ASSERTE(fabs(f_norm - r_vec.squaredNorm()) < 1e-7);
		// makes sure we got it right

		return f_norm;
	}

	/**
	 *	@brief calculates multiplication and addition of two vectors; optimized for very long vectors
	 *
	 *	Calculates <tt>r_dest += r_src * f_scale</tt>.
	 *
	 *	@param[in,out] r_dest is the destination dense vector
	 *	@param[in] r_src is the source dense vector
	 *	@param[in] f_scale is scaling factor for the source vector
	 */
	static void FastVectorMAD(Eigen::VectorXd &r_dest, const Eigen::VectorXd &r_src, double f_scale)
	{
		_ASSERTE(r_dest.rows() == r_src.rows());

		for(size_t i = 0, n = r_dest.rows();; i += fast_vec_Step) {
			if(i + fast_vec_Step < n) {
				r_dest.segment<fast_vec_Step>(i) += r_src.segment<fast_vec_Step>(i) * f_scale;
				// potentially uses SSE, even though the vector might be unaligned (not in x64)
			} else {
				r_dest.segment(i, n - i) += r_src.segment(i, n - i) * f_scale;
				// the misaligned part comes last; keeps the other addresses aligned
				break;
			}
		}
	}

	/**
	 *	@brief calculates multiplication and addition of two vectors; optimized for very long vectors
	 *
	 *	Calculates <tt>r_dest_neg -= r_src * f_scale</tt> and <tt>r_dest_pos += r_src * f_scale</tt>.
	 *
	 *	@param[in,out] r_dest_neg is the destination dense vector with negative scale factor
	 *	@param[in,out] r_dest_pos is the destination dense vector with positive scale factor
	 *	@param[in] r_src is the source dense vector
	 *	@param[in] f_scale is scaling factor for the source vector
	 */
	static void FastVectorMAD_NegPos(Eigen::VectorXd &r_dest_neg,
		Eigen::VectorXd &r_dest_pos, const Eigen::VectorXd &r_src, double f_scale)
	{
		_ASSERTE(r_dest_neg.rows() == r_src.rows());
		_ASSERTE(r_dest_pos.rows() == r_dest_neg.rows());

		for(size_t i = 0, n = r_dest_neg.rows();; i += fast_vec_Step) {
			if(i + fast_vec_Step < n) {
				r_dest_pos.segment<fast_vec_Step>(i) += r_src.segment<fast_vec_Step>(i) * f_scale;
				r_dest_neg.segment<fast_vec_Step>(i) -= r_src.segment<fast_vec_Step>(i) * f_scale;
				// potentially uses SSE, even though the vector might be unaligned (not in x64)
			} else {
				r_dest_pos.segment(i, n - i) += r_src.segment(i, n - i) * f_scale;
				r_dest_neg.segment(i, n - i) -= r_src.segment(i, n - i) * f_scale;
				// the misaligned part comes last; keeps the other addresses aligned
				break;
			}
		}
	}

	/**
	 *	@brief calculates multiplication and addition of two vectors; optimized for very long vectors
	 *
	 *	Calculates <tt>r_dest = r_dest * f_scale + r_src</tt>.
	 *
	 *	@param[in,out] r_dest is the destination dense vector
	 *	@param[in] r_src is the source dense vector
	 *	@param[in] f_scale is scaling factor for the source vector
	 */
	static void FastVectorAMD(Eigen::VectorXd &r_dest, double f_scale, const Eigen::VectorXd &r_src)
	{
		_ASSERTE(r_dest.rows() == r_src.rows());

		for(size_t i = 0, n = r_dest.rows();; i += fast_vec_Step) {
			if(i + fast_vec_Step < n) {
				r_dest.segment<fast_vec_Step>(i) *= f_scale;
				r_dest.segment<fast_vec_Step>(i) += r_src.segment<fast_vec_Step>(i);
				// potentially uses SSE, even though the vector might be unaligned (not in x64)
			} else {
				r_dest.segment(i, n - i) *= f_scale;
				r_dest.segment(i, n - i) += r_src.segment(i, n - i);
				// the misaligned part comes last; keeps the other addresses aligned
				break;
			}
		}
	}

	/**
	 *	@brief final optimization function
	 *
	 *	@param[in] n_max_iteration_num is the maximal number of iterations
	 *	@param[in] f_min_dx_norm is the residual norm threshold
	 */
	void Optimize(size_t n_max_iteration_num = 5, double f_min_dx_norm = .01) // throw(std::bad_alloc)
	{
		const size_t n_variables_size = m_r_system.n_VertexElement_Num();
		const size_t n_measurements_size = m_r_system.n_EdgeElement_Num();
		if(n_variables_size > n_measurements_size) {
			if(n_measurements_size)
				fprintf(stderr, "warning: the system is underspecified\n");
			else
				fprintf(stderr, "warning: the system contains no edges at all: nothing to optimize\n");
			//return;
		}
		if(!n_measurements_size)
			return; // nothing to solve (but no results need to be generated so it's ok)
		// can't solve in such conditions

		_ASSERTE(this->m_r_system.b_AllVertices_Covered());
		// if not all vertices are covered then the system matrix will be rank deficient and this will fail
		// this triggers typically if solving BA problems with incremental solve each N steps (the "proper"
		// way is to use CONSISTENCY_MARKER and incremental solve period of SIZE_MAX).

		double f_time_start = m_timer.f_Time();

		Extend_A(m_n_edges_in_A); // recalculated all the jacobians inside Extend_A()
		if(!m_b_system_dirty)
			Refresh_A(m_n_edges_in_A); // calculate only for new edges
		else
			Refresh_A(); // calculate for entire system
		m_b_system_dirty = false;
		m_n_edges_in_A = m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
		_ASSERTE(m_A.n_Row_Num() >= m_A.n_Column_Num()); // no, A is *not* square, but size of measurements >= vertices
		// need to have A

		m_v_error.resize(n_measurements_size, 1);
		m_v_dx.resize(n_variables_size, 1);

		double f_A_refresh_end = m_timer.f_Time();

		const size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		const size_t n_edge_num = m_r_system.r_Edge_Pool().n_Size();
		const size_t n_row_num = n_edge_num + 1; // and the unary factor
		_ASSERTE(n_vertex_num == m_A.n_BlockColumn_Num()); // A should have the same number of columns as there are vertices
		_ASSERTE(n_row_num == m_A.n_BlockRow_Num()); // A should have the same number of rows as there are edges

		std::vector<size_t> perm_blockwise;
		perm_blockwise.resize(n_row_num);
		//for(size_t i = 0; i < n_row_num; ++ i)
		//	perm_blockwise[i] = i;
		// create a permutation (todo - put the tree thing in here)

		cs *p_A_struct = m_A.p_BlockStructure_to_Sparse();
		size_t n_perm_dest = n_OrientedSpanningTree(&perm_blockwise[0],
			p_A_struct, m_r_system.r_Edge_Pool(), 0); // t_odo - make this a function
		cs_spfree(p_A_struct);
		// calculate a spanning tree (uses structure of the graph in a sparse
		// matrix and graph edges to avoid having to transpose the matrix)

		_ASSERTE(n_perm_dest == n_vertex_num); // this only holds if the subgraph is a spanning tree!
		// make sure that number of edges used in the spanning tree equals the number of vertices

		size_t n_A1_block_row_num = n_perm_dest;//m_A.n_BlockColumn_Num();
		_ASSERTE(n_A1_block_row_num > 0);
		// A1 is square (but only if the subgraph is a spanning tree!)

		printf("perm dest " PRIsize ", block col num " PRIsize "\n", n_perm_dest, m_A.n_BlockColumn_Num());

		{
			std::vector<bool> support_edge_set; // todo - reuse bool vector
			support_edge_set.resize(n_row_num, false);
			for(size_t i = 0; i < n_perm_dest; ++ i)
				support_edge_set[perm_blockwise[i]] = true;
			// tick the edges that are in support

			for(size_t i = 0; i < n_row_num; ++ i) {
				if(!support_edge_set[i]) {
					_ASSERTE(n_perm_dest < n_row_num);
					perm_blockwise[n_perm_dest] = i;
					++ n_perm_dest;
				}
			}
			_ASSERTE(n_perm_dest == n_row_num);
			// gather the remaining edges,
		}
		// finish the edge permutation vector

		{
			std::vector<size_t> perm_blockwise_inv; // todo - reuse vector
			perm_blockwise_inv.resize(perm_blockwise.size());
			for(size_t i = 0, n = perm_blockwise.size(); i < n; ++ i)
				perm_blockwise_inv[perm_blockwise[i]] = i;
			perm_blockwise_inv.swap(perm_blockwise);
		}
		// invert the permutation

		CMatrixOrdering mord;
		const csi *p_elem_order = (const csi*)mord.p_ExpandBlockOrdering(perm_blockwise, m_A);

		double f_spanning_tree_end = m_timer.f_Time();

		CUberBlockMatrix perm_A;
		m_A.PermuteTo(perm_A, &perm_blockwise[0], perm_blockwise.size(), true, false, true);
		// make a shallow copy of A and permutate it

		size_t n_A2_block_row_num = n_row_num - n_A1_block_row_num;
		_ASSERTE(n_A2_block_row_num >= 0); // if the odometry is straight line with no loops, A2 will be empty
		// A2 is rectangular, under A1

		CUberBlockMatrix A1, A2;
		for(;;) {
			perm_A.SliceTo(A1, 0, n_A1_block_row_num, 0, n_vertex_num, true);
			if(n_A2_block_row_num > 0)
				perm_A.SliceTo(A2, n_A1_block_row_num, n_row_num, 0, n_vertex_num, true);

			if(A1.n_Column_Num() <= A1.n_Row_Num())
				break;
			// in case A1 is tall, we're done

			printf("A1 is short (" PRIsize " x " PRIsize ")\n", A1.n_Row_Num(), A1.n_Column_Num()); // debug

			break; // it would seem this is not a cool thing to do

			if(!n_A2_block_row_num)
				throw std::runtime_error("the system is undetermined");
			++ n_A1_block_row_num;
			-- n_A2_block_row_num;
			A2.Clear(); // perm_A.SliceTo(A2, ...) might not get called if n_A2_block_row_num is now zero
			// hack - our solver can't handle wide A1 (undetermined system) at the moment
			// just shuffle one or more block rows from A2 in order to add constrains
			// note this will likely cause considerable fill-in later but at least we solve the system
		}
		// slice the permutated matrix to two matrices A1 and A2
		// note that these are shallow copies and any numerical changes to A will show in A1 or A2,
		// and also any numerical modifications to A1 or A2 will show in A; this does not apply to
		// structural changes, adding new blocks to A will not show in A1 or A2 (and vice versa)

		if(A1.n_Column_Num() > A1.n_Row_Num())
			throw std::runtime_error("A1 is undetermined"); // in that case, QR is done on transpose matrices and all this stuff probably won't work
		//if(A1.n_Column_Num() > A1.n_Row_Num())
		//	throw std::runtime_error("A1 is rect"); // this can happen, especially with landmarks (even if number of block rows and columns is equal as implied by the spanning tree, number of rows and columns may not be)
		_ASSERTE(A1.n_Column_Num() == A2.n_Column_Num());
		_ASSERTE(A1.n_Column_Num() == m_A.n_Column_Num());
		_ASSERTE(A1.n_Column_Num() == n_variables_size);
		_ASSERTE(A1.n_Row_Num() + A2.n_Row_Num() == m_A.n_Row_Num());
		_ASSERTE(n_measurements_size == m_A.n_Row_Num());
		// some assumptions about the matrices

		double f_A_perm_end = m_timer.f_Time();

		m_f_A_refresh_time += f_A_refresh_end - f_time_start;
		m_f_spanning_tree_time += f_spanning_tree_end - f_A_refresh_end;
		m_f_A_perm_time += f_A_perm_end - f_spanning_tree_end;

		cs *csA1 = 0;
		for(size_t n_iteration = 0; n_iteration < n_max_iteration_num; ++ n_iteration) {
			++ m_n_iteration_num;
			// debug

			if(m_b_verbose) {
				if(n_max_iteration_num == 1)
					printf("\n=== incremental optimization step ===\n\n");
				else
					printf("\n=== nonlinear optimization: iter #" PRIsize " ===\n\n", n_iteration);
			}
			// verbose

			if(n_iteration && m_b_system_dirty) {
				Refresh_A();
				m_b_system_dirty = false;
			}
			// no need to rebuild A, just refresh the values that are being referenced

			double f_loop_start_time = m_timer.f_Time();

			_ASSERTE(m_A.n_Row_Num() == n_measurements_size); // should be the same
			Collect_R_Errors(m_v_error);
			// collect errors (in parallel)

			double f_rhs_end = m_timer.f_Time();

			{
				// Soso programs somewhere around here
				// in here, A is up to date, A1 and A2 contain the parts of A they are supposed to,
				// and m_v_error contains a vector of R * errors for every vertex

				// below is the classical A pipeline:
				//		* calculate lambda
				//		* calculate eta = A * (R * errors)
				//		* solve for dx = cholsolv(lambda, eta)

				// i would recommend not deleting this code, calculating dx the old way
				// and the new way and compare at the end (norm of difference should be below 1e-7)
				Eigen::VectorXd &v_b = m_v_error; // b = Rz * p_errors
				Eigen::VectorXd v_bp(v_b.rows()); // alloc b perm
				cs_ipvec(p_elem_order, &v_b(0), &v_bp(0), v_b.rows()); // permutate b
				Eigen::VectorBlock<Eigen::VectorXd> v_b1 = v_bp.segment(0, A1.n_Row_Num()); // slice b1
				Eigen::VectorBlock<Eigen::VectorXd> v_b2 = v_bp.segment(A1.n_Row_Num(), v_bp.rows() - A1.n_Row_Num()); // slice b2
				_ASSERTE(v_b1.rows() == A1.n_Row_Num());

				double f_perm_b_end = m_timer.f_Time();

#if defined(__NONLINEAR_SOLVER_SPCG_DUMP_VARS) && defined(_DEBUG) && (defined(_WIN32) || defined(_WIN64))
				{
					FILE *p_fw;
					p_fw = fopen("sysDump.m", "w"); // all in one
					//p_fw = fopen("fb.m", "w");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_b(0), v_b.rows(), "b = ");
					//fclose(p_fw);
					//p_fw = fopen("fb1.m", "w");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_b1(0), v_b1.rows(), "b1 = ");
					//fclose(p_fw);
					//p_fw = fopen("fb2.m", "w");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_b2(0), v_b2.rows(), "b2 = ");
					//fclose(p_fw);
					//p_fw = fopen("fA.m", "w");
					CDebug::Print_SparseMatrix_in_MatlabFormat2(p_fw, m_A.p_Convert_to_Sparse(), "A");
					//fclose(p_fw);
					//p_fw = fopen("fA1.m", "w");
					CDebug::Print_SparseMatrix_in_MatlabFormat2(p_fw, A1.p_Convert_to_Sparse(), "A1");
					//fclose(p_fw);
					//p_fw = fopen("fA2.m", "w");
					CDebug::Print_SparseMatrix_in_MatlabFormat2(p_fw, A2.p_Convert_to_Sparse(), "A2");
					//fclose(p_fw);
					//p_fw = fopen("fperm.m", "w");
					{
						fprintf(p_fw, "permutation = [" PRIsize "", p_elem_order[0]);
						for(size_t i = 1, n = m_A.n_Row_Num(); i < n; ++ i)
							fprintf(p_fw, " " PRIsize "", p_elem_order[i]);
						fprintf(p_fw, "];\n");
					}
					fclose(p_fw);

					//exit(0);
				}
				// matlab printout
#endif // __NONLINEAR_SOLVER_SPCG_DUMP_VARS && _DEBUG && (_WIN32 || _WIN64)

				csA1 = A1.p_Convert_to_Sparse(csA1); // to sparse, reuse

#if 0
				A1.Rasterize("spcg_0_A1.tga");
				A2.Rasterize("spcg_1_A2.tga");
				m_A.Rasterize("spcg_2_A.tga");
				perm_A.Rasterize("spcg_3_permA.tga");

				exit(0);
#endif // 0

#if 0
				_ASSERTE(A1.n_Column_Num() <= A1.n_Row_Num());
				Eigen::VectorXd v_xBar_ref(A1.n_Row_Num()); // m in, n out; n is less than m
				v_xBar_ref.setZero();
				v_xBar_ref.segment(0, v_b1.rows()) = v_b1;
				double f_xbref0_n = v_xBar_ref.norm();
				bool b_qrsol_success = cs_qrsol(0, csA1, &v_xBar_ref(0)) != 0; // vector of m in (that's b), vector of n out (that's x)
				double f_xbref_n = v_xBar_ref.norm();
				{
					Eigen::VectorXd errorCheck(v_b1.rows());
					errorCheck.setZero();
					A1.PreMultiply_Add_FBS<_TyAMatrixBlockSizes>(&errorCheck(0),
						errorCheck.rows(), &v_xBar_ref(0), n_variables_size);
					double f_diff = (errorCheck - v_b1).norm();
					_ASSERTE(f_diff < 1e-7);
				}
				// calculate reference xBar using csparse
#endif // 0

				css *qrsym = cs_sqr(0, csA1, 1); // symbolic stuff for qr (pivoting, ...)
				if(!qrsym)
					throw std::runtime_error("qr sym failed");
				csn *qrfac = cs_qr(csA1, qrsym);
				if(!qrfac)
					throw std::runtime_error("qr failed");
				const cs *R1 = qrfac->U; // R for qr(A1)

				double f_qr_end = m_timer.f_Time();

				_ASSERTE(size_t(v_b1.rows()) >= n_variables_size);
				_ASSERTE(qrsym->m2 >= v_b1.rows());
				Eigen::VectorXd v_xBarWork(qrsym->m2); // actually need larger workspace
				Eigen::VectorBlock<Eigen::VectorXd> v_xBar = v_xBarWork.segment(0, n_variables_size);
				{
					//if(qrsym->m2 > v_b1.rows())
					//	v_xBarWork.segment(v_b1.rows(), qrsym->m2 - v_b1.rows()).setZero();
					v_xBarWork.setZero(); // csparse uses calloc(), maybe the permutation spans all of x
					cs_ipvec(qrsym->pinv, &v_b1(0), &v_xBar(0), v_b1.rows());   /* x(0:m-1) = b(p(0:m-1) */// v_xBar = v_b1; // copy b1 (cs_qrsol() would do this using cs_ipvec(), we have no ordering)

					//double f_xw_n = v_xBarWork.norm();
					//double f_xb_n = v_xBar.norm();
					//double f_xw1_n = (qrsym->m2 > /*A1.n_Column_Num()*/v_b1.rows())? v_xBarWork.segment(/*A1.n_Column_Num()*/v_b1.rows(), qrsym->m2 - /*A1.n_Column_Num()*/v_b1.rows()).norm() : 0; // rectfix
					//double f_e_n = m_v_error.norm();
					//double f_bp_n = v_bp.norm();
					//double f_b1_n = v_b1.norm();
					//double f_b2_n = v_b2.norm(); // debug

					double *xBar = &v_xBar(0);
					for(size_t k = 0, n = n_variables_size; k < n; ++ k)
						cs_happly(qrfac->L, k, qrfac->B[k], xBar); // apply Householder refl. to xBar
					// xBar is indexed by qrfac->L->i, by the rows of L

					//double f_xw1_n2 = (qrsym->m2 > /*A1.n_Column_Num()*/v_b1.rows())? v_xBarWork.segment(/*A1.n_Column_Num()*/v_b1.rows(), qrsym->m2 - /*A1.n_Column_Num()*/v_b1.rows()).norm() : 0; // rectfix
					//double f_xb_n2 = v_xBar.norm(); // debug

					cs_usolve(R1, xBar); // xBar = R \ xBar
					_ASSERTE(!qrsym->q); // no permutation on this end, otherwise we need to execute the next line
					//cs_ipvec(qrsym->q, x, b, n_variables_size);
					// solve for xBar, taking care not to overwrite v_b1 as a side effect
				}
				// solve for xBar = A1 / b1 using QR

				double f_solve_end = m_timer.f_Time();

				//double f_xw1_n3 = (qrsym->m2 > /*A1.n_Column_Num()*/v_b1.rows())? v_xBarWork.segment(/*A1.n_Column_Num()*/v_b1.rows(), qrsym->m2 - /*A1.n_Column_Num()*/v_b1.rows()).norm() : 0; // rectfix
				//double f_xb_n3 = v_xBar.norm(); // debug

#ifdef _DEBUG
				{
					Eigen::VectorXd errorCheck(v_b1.rows());
					errorCheck.setZero();
					A1.PreMultiply_Add_FBS<_TyAMatrixBlockSizes>(&errorCheck(0),
						errorCheck.rows(), &v_xBar(0), v_xBar.rows());
					double f_diff = (errorCheck - v_b1).norm();
					_ASSERTE(f_diff < 1e-7); // make sure we get the same thing
				} // broken if A1 is square // should work now

				{
					Eigen::VectorXd original(v_xBarWork.rows());
					original.setZero();

					PushValuesInGraphSystem2(original); // extract the original solution

					Eigen::VectorXd new_sys = original + v_xBarWork;
					PushValuesInGraphSystem2(new_sys); // add original + xBar (and new_sys is null afterwards)

					//Eigen::VectorXd initial_copy = v_xBarWork; // not a good idea

					char p_s_fielname[256];
					sprintf(p_s_fielname, "partial" PRIsize ".tga", n_iteration);
					m_r_system.Plot2D(p_s_fielname);

					PushValuesInGraphSystem2(original); // put the system back in the original state

					//v_xBarWork = initial_copy; // use initial prior instead of xbar // not a good idea
				}
				// plot the partial QR solutions (ones without loop closures)
#endif // _DEBUG

				/*double *c1 = &v_b1(0); // inplace
				cs_usolve(R1, c1); // c1 = R \ b1*/
				// solve for c1, overwriting v_b1 as a side effect (note it might overwrite b2 as well as R is qrsym->m2 tall)

				CUberBlockMatrix A2_t;
				A2.TransposeTo(A2_t);
				// calculate transpose of A2 to speed up the processing

				Eigen::VectorXd v_bBar(v_b.rows()); // maybe not required in its entirety, maybe only v_bBar2 is required (norm of v_bBar is required, but not v_bBar)
				Eigen::VectorBlock<Eigen::VectorXd> v_bBar2 = v_bBar.segment(v_b1.rows(), v_b2.rows());
				{
					v_bBar./*segment(0, v_b1.rows()).*/setZero(); // we're using gaxpy, zero it all
					_ASSERTE(v_b1.rows() + v_b2.rows() == v_b.rows());
					A2_t.PostMultiply_Add_FBS_Parallel<_TyATransposeMatrixBlockSizes>(&v_bBar2(0),
						v_bBar2.rows(), &v_xBar(0), v_xBar.rows()); // v_bBar2 = v_xBar * A2
					v_bBar2 = v_b2 - v_bBar2;
				}
				// calculate bBar (strange thing, the leading of it seems to always be null - maybe it doesn't need to be stored at all) // todo

				//double f_epsilon = 1e-3 * v_b.norm() / v_bBar.norm(); // not correct maybe
				// calculate epsilon

				Eigen::VectorXd v_y(n_variables_size);
				v_y.setZero();
				// an initial prior of y

				Eigen::VectorXd v_s(n_variables_size);
				Eigen::VectorXd v_r1(v_s.rows());
				v_r1.setZero();
				Eigen::VectorXd v_r2 = v_bBar2;
				{
					v_s.setZero();
					A2.PostMultiply_Add_FBS_Parallel<_TyAMatrixBlockSizes>(&v_s(0), v_s.rows(), &v_r2(0), v_r2.rows());

#if defined(__NONLINEAR_SOLVER_SPCG_DUMP_VARS) && defined(_DEBUG) && (defined(_WIN32) || defined(_WIN64))
					{
						FILE *p_fw;
						p_fw = fopen("sysDump.m", "a"); // all in one - append
						CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_s(0), v_s.rows(), "A2tr2 = ");
						fclose(p_fw);
					}
					// matlab printout
#endif // __NONLINEAR_SOLVER_SPCG_DUMP_VARS && _DEBUG && (_WIN32 || _WIN64)

					//cs *R1t = cs_transpose(R1, 1);
					cs_utsolve(R1, &v_s(0)); // R1' / (r2' * A2)'
					//cs_ltsolve(R1t, &v_s(0)); // R1' / (r2' * A2)' // no better
					//cs_spfree(R1t);
					_ASSERTE(v_r1.norm() == 0); // otherwise the next line is necessary
					//v_s += v_r1; // it is null
				}
				Eigen::VectorXd v_p = v_s; // p = s
				// calculate step (not sure whether usolve or utsolve)

#if defined(__NONLINEAR_SOLVER_SPCG_DUMP_VARS) && defined(_DEBUG) && (defined(_WIN32) || defined(_WIN64))
				static const double p_herP[] = {-7.1561, 4.0106, 2.3120, -9.6991, -4.8006, 5.3846, -10.7609, 3.3830,
					-5.3577, -12.4109, 1.9755, -5.4319, -14.4078, 0.2935, -3.6227, -16.5037, -1.1430, 14.1516, 17.0886, 2.0536, 20.7937, 16.1898, -2.6815, -22.7078, -15.3945, 2.8650, -25.0825, -15.4366, 2.5715, -25.9794, -15.9864, 2.0982, 17.5037, 18.0504, -1.5170, -10.6065, 20.1594, 0.8962, -4.2228, 22.1841, 0.6695, 0.4678, 23.6597, 1.2373, 3.4878,
					24.4239, 2.2929, -25.1994, 25.0728, -4.3541, 45.6033, -25.3557, 6.7029, 65.3281, -25.0222, -9.2543, 82.6045, 23.3625, 11.9944, 95.6465, -20.0700, -15.4061, -74.5042, -15.9992, 16.6832, 86.0340, -10.6638, 16.9455, 98.3472, -5.8631, -16.2055, 105.2390, -1.5180, -13.0823, 106.3519, 2.7711, -9.1964, -98.2190, 5.4964, 6.1443, 96.3705, -6.4237,
					-3.9799, 95.3531, -5.7126, 2.5386, 92.8049, 3.7470, -1.5495, 87.8771, -1.6360, 0.6865, -78.2481, 0.4100, -1.3371, 70.6592, 2.8083, -2.4781, 62.8527, 5.5585, 3.8821, 53.4480, 8.3672, 4.9381, 48.3464, 10.9952, 5.2347, -54.5198, 13.4988, -5.3040, 64.5552, -16.0867, 5.4725, 75.7345, -18.4998, -6.1197, 90.8732, 19.8355, 6.6235, 104.8890,
					-21.5707, -8.0816, -103.3619, -20.5725, 9.3101, 90.2953, -18.0892, 9.8012, 99.7339, -15.7334, -9.5730, 102.3063, -12.9558, -8.6800, 100.1070, -9.8132, -8.1846, -83.0768, -6.7805, 7.5514, 72.3042, 3.8634, -7.3644, 64.8026, 0.5158, 6.1533, 60.4424, 2.9906, -5.3605, 58.5989, -6.9925, 4.3819, -50.2649, -10.5060, -3.4214, 42.8776, -14.4756,
					-2.5865, 36.5877, -18.4462, 2.1250, 27.3499, -21.7236, 1.5470, 21.1813, -24.5611, 1.2602, 1.9704, -27.6707, -1.0057, -30.8617, 31.0783, 0.2123, -59.5818, 34.1068, 0.8743, -86.9449, -37.4375, -3.1284, -120.6758, 41.1056, 5.6925, 117.8823, 41.9880, 7.7680, 108.8804, 40.9241, -9.0123, -93.1526, -40.0857, -10.2595, -75.7941, -39.5350, -11.6028,
					-63.6024, -39.2826, -12.9170, 14.9180, 39.1817, -13.2907, 25.5523, 39.0448, -12.8648, 61.3517, -38.7217, -11.9633, 93.8880, 38.0674, 10.6707, 118.4669, -37.0601, -9.1430, -112.1060, -36.1702, 7.6911, 96.7339, -35.5075, 6.4609, 87.7226, -34.9630, -5.3724, 78.7210, -34.5201, -4.4387, 69.3171, -34.1694, -3.6546, -31.3607, -34.0459, 3.3466, -7.1062,
					33.9514, -3.3997, -39.9050, 33.7194, 3.6893, -70.7924, -33.2217, -4.1446, -96.6540, 32.4009, 4.6921, 88.9114, 31.7083, -5.0985, -87.0631, 31.0571, -5.4116, -83.0511, 30.4702, 5.6473, -77.7611, 29.8302, 5.7010, -76.7561, 29.2510, 5.2840, 102.1485, 28.2997, 5.2587, 122.3359, -27.1634, 5.3413, -143.2177, 25.6310, 5.3672, -156.9912, -23.8043,
					-5.3334, -166.1744, 21.7681, 5.2359, 153.1593, 20.0477, -5.0768, -141.2569, 18.5945, -4.8600, -134.5860, 17.2847, 4.5906, -128.3987, 16.1017, 4.2763, -121.7218, 15.0454, 3.9398, -98.9613, 14.3475, 3.7156, -80.8835, 13.8819, -3.5588, -63.6584, 13.5934, 3.4539, -40.7218, -15.6250, -0.7791, -26.0920, 689.5702, -425.8646, 5.1885};
				size_t n_PhiPher = sizeof(p_herP) / sizeof(p_herP[0]);
				_ASSERTE(n_PhiPher == v_p.rows());
				double f_abs_diff_r = 0;
				double f_diff_r = 0;
				double f_ndiff_r = 0;
				for(size_t i = 0; i < n_PhiPher; ++ i) {
					double f_abs_diff = fabs(p_herP[i]) - fabs(v_p(i));
					f_abs_diff_r += f_abs_diff * f_abs_diff;
					double f_diff = p_herP[i] - v_p(i);
					f_diff_r += f_diff * f_diff;
					double f_ndiff = p_herP[i] + v_p(i);
					f_ndiff_r += f_ndiff * f_ndiff;
				}
				f_abs_diff_r = sqrt(f_abs_diff_r);
				f_diff_r = sqrt(f_diff_r);
				f_ndiff_r = sqrt(f_ndiff_r);
				// compare p with reference value
#endif // __NONLINEAR_SOLVER_SPCG_DUMP_VARS && _DEBUG && (_WIN32 || _WIN64)

				double f_gamma = v_s.squaredNorm();
				// calculate gamma = s' * s

				const double f_epsilon_in = 1e-3;
				double f_thresh = f_epsilon_in * f_epsilon_in * f_gamma * v_b.squaredNorm() / v_bBar.squaredNorm();
				// calculate threshold

#ifdef __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL
				std::vector<double> residual_vector;
#pragma message("warning: __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL is defined, computation will be slow")
#endif // __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL

#if defined(__NONLINEAR_SOLVER_SPCG_DUMP_VARS) && defined(_DEBUG) && (defined(_WIN32) || defined(_WIN64))
				{
					FILE *p_fw;
					p_fw = fopen("sysDump.m", "a"); // all in one - append
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_xBar(0), v_xBar.rows(), "xBarL = ");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_bBar2(0), v_bBar2.rows(), "b2BarL = ");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_p(0), v_p.rows(), "pL = ");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_r1(0), v_r1.rows(), "r1L = ");
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &v_r2(0), v_r2.rows(), "r2L = ");
					CDebug::Print_SparseMatrix_in_MatlabFormat2(p_fw, R1, "R1L");
					fclose(p_fw);

					exit(0);
				}
				// matlab printout
#endif // __NONLINEAR_SOLVER_SPCG_DUMP_VARS && _DEBUG && (_WIN32 || _WIN64)

				size_t n_max_iterations = m_A.n_Column_Num(); // this is theoretical maximum (no of variables)
				{Eigen::VectorXd v_q(A2.n_Row_Num()); // reuse the memory
				Eigen::VectorXd v_tmp(v_p.rows()); // reuse the memory
				for(size_t n_iteration_num = 0;; ++ n_iteration_num) {
					{
						v_q.setZero();
						v_tmp = v_p; // t_odo - reuse storage
						cs_usolve(R1, &v_tmp(0)); // tmp = R1 / p
						A2_t.PostMultiply_Add_FBS_Parallel<_TyATransposeMatrixBlockSizes>(&v_q(0),
							v_q.rows(), &v_tmp(0), v_tmp.rows());
					}
					// q = A2 * (R1 / p) = A2 * (R1 / s)

					//double f_alpha = f_gamma / (v_p.squaredNorm() + v_q.squaredNorm());
					double f_alpha = f_gamma / (f_FastSquaredVectorNorm(v_p) + f_FastSquaredVectorNorm(v_q));

					FastVectorMAD_NegPos(v_r1, v_y, v_p, f_alpha); // v_y += v_p * f_alpha,  v_r1 -= v_p * f_alpha;
					FastVectorMAD(v_r2, v_q, -f_alpha); // v_r2 -= v_q * f_alpha;

					{
						v_s.setZero();
						A2.PostMultiply_Add_FBS_Parallel<_TyAMatrixBlockSizes>(&v_s(0),
							v_s.rows(), &v_r2(0), v_r2.rows()); // s = r2' * A2
						cs_utsolve(R1, &v_s(0)); // s = R1' / (r2' * A2)'
						v_s += v_r1; // s = r1 + R1' / (r2' * A2)'
					}
					// calculate step (not sure whether usolve or utsolve)

					double f_new_gamma = f_FastSquaredVectorNorm(v_s); // v_s.squaredNorm();
					double f_beta = f_new_gamma / f_gamma;
					f_gamma = f_new_gamma;

					FastVectorAMD(v_p, f_beta, v_s); // v_p *= f_beta; v_p += v_s;
					// p = s + beta * p

#ifdef __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL
					{
						Eigen::VectorXd v_y_copy = v_y;
						cs_usolve(R1, &v_y_copy(0)); // y = y / R1
						m_v_dx = v_y_copy + v_xBar;
						// calculate the final solution

						Eigen::VectorXd v_diff = -v_bp;
						perm_A.PreMultiply_Add_FBS<_TyAMatrixBlockSizes>(&v_diff(0), v_diff.rows(), &m_v_dx(0), m_v_dx.rows());
						// v_diff = -(b - A * x)
						double f_residual = v_diff.norm() / v_bp.norm();
						// calculate the residual

						residual_vector.push_back(f_residual);
					}
#endif // __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL

					if(n_iteration_num > n_max_iterations || f_gamma < f_thresh) { // f_gamma = f_new_gamma by now
						cs_usolve(R1, &v_y(0)); // y = y / R1
						m_v_dx = v_y + v_xBar; // dx = y / R1 + xbar
						// calculate the final solution

						printf("CG linear solver took " PRIsize " iterations\n", n_iteration_num + 1);

						break;
					}
					// stopping condition
				}}
				// spcg, brotha!

				/*{
					cs *csA2 = A2.p_Convert_to_Sparse();
					Eigen::VectorXd xTilde(std::max(size_t(v_bBar2.rows()), n_variables_size));
					xTilde.setZero(); // not required
					xTilde.segment(0, v_bBar2.rows()) = v_bBar2;
					cs_qrsol(0, csA2, &xTilde(0));
					Eigen::VectorXd yCheck(n_variables_size);
					cs_spfree(csA2);
					yCheck.setZero();
					cs *R1t = cs_transpose(R1, 1);
					cs_gaxpy(R1t, &xTilde(0), &yCheck(0));
					cs_spfree(R1t);
					double f_diff0 = (v_y - yCheck).norm();
					yCheck.setZero();
					cs_gaxpy(R1, &xTilde(0), &yCheck(0));
					double f_diff1 = (v_y - yCheck).norm();
				}*/ // not working
				// see if A2R1^-1y - b2Bar = 0 holds

#ifdef __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL
				{
					char p_s_filename[256];
					sprintf(p_s_filename, "residualRound" PRIsize ".m", n_iteration);
					FILE *p_fw;
					p_fw = fopen(p_s_filename, "w"); // all in one
					CDebug::Print_DenseVector_in_MatlabFormat(p_fw, &residual_vector[0], residual_vector.size(), "res_v_slampp = ");
					fclose(p_fw);
				}
#endif // __NONLINEAR_SOLVER_SPCG_PLOT_RESIDUAL

				cs_sfree(qrsym);
				cs_nfree(qrfac);

				m_f_rhs_time += f_rhs_end - f_loop_start_time;
				m_f_perm_b_time += f_perm_b_end - f_rhs_end;
				m_f_qr_time += f_qr_end - f_perm_b_end;
				m_f_solve_time += f_solve_end - f_qr_end;

#ifdef __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION
				Eigen::VectorXd v_spcg_dx = m_v_dx;

				CUberBlockMatrix lambda;
				m_A.PreMultiplyWithSelfTransposeTo_FBS<_TyAMatrixBlockSizes>(lambda, true);
				// calculate lambda without calculating At (and only upper diagonal thereof)

				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					v_eta.setZero(); // eta = 0
					Eigen::VectorXd &v_b = m_v_error; // b = Rz * p_errors
					_ASSERTE(v_eta.rows() == n_variables_size);
					_ASSERTE(v_b.rows() == n_measurements_size);

					m_A.PostMultiply_Add_FBS_Parallel<_TyAMatrixBlockSizes>(&v_eta(0), n_variables_size, &v_b(0),
						n_measurements_size);
					// use the function with fixed-block sizes
				}
				// calculate eta = A^T * b

				bool b_cholesky_result;
				{
					Eigen::VectorXd &v_eta = m_v_dx; // dx is calculated inplace from eta
					if(n_max_iteration_num > 1) {
						do {
							if(!n_iteration &&
							   !_TyLinearSolverWrapper::FinalBlockStructure(m_linear_solver, lambda)) {
								b_cholesky_result = false;
								break;
							}
							// prepare symbolic factorization, structure of lambda won't change in the next steps

							b_cholesky_result = _TyLinearSolverWrapper::Solve(m_linear_solver, lambda, v_eta);
							// p_dx = eta = lambda / eta
						} while(0);
					} else
						b_cholesky_result = m_linear_solver.Solve_PosDef(lambda, v_eta); // p_dx = eta = lambda / eta

					if(m_b_verbose)
						printf("%s", (b_cholesky_result)? "Cholesky succeeded\n" : "Cholesky failed\n");
				}
				// calculate cholesky, reuse block ordering if the linear solver supports it
#else // __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION
				bool b_cholesky_result = true;
#endif // __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION

#ifdef _DEBUG
				for(size_t i = 0; i < n_variables_size; ++ i) {
					if(_isnan(m_v_dx(i))) {
						fprintf(stderr, "warning: p_dx[" PRIsize "] = NaN (file \'%s\', line "
							PRIsize ")\n", i, __FILE__, size_t(__LINE__));
					}
				}
				// detect the NaNs, if any (warn, but don't modify)
#endif // _DEBUG

				double f_residual_norm = 0;
				if(b_cholesky_result) {
					f_residual_norm = m_v_dx.norm(); // Eigen likely uses SSE and OpenMP
					if(m_b_verbose)
						printf("residual norm: %.4f\n", f_residual_norm);
				}
				// calculate residual norm

				//f_residual_norm = v_spcg_dx.norm(); // use SPCG solution

				if(f_residual_norm <= f_min_dx_norm)
					break;
				// in case the error is low enough, quit (saves us recalculating the hessians)

				if(b_cholesky_result) {
#ifdef __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION
					double f_difference_spcg_exact = (m_v_dx - v_spcg_dx).norm();
					printf("difference between SPCG and the exact solution: %f\n", f_difference_spcg_exact);

					PushValuesInGraphSystem(v_spcg_dx/*m_v_dx*/); // use SPCG solution (backed up)
#else // __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION
					PushValuesInGraphSystem(m_v_dx); // use SPCG solution
#endif // __NONLINEAR_SOLVER_SPCG_DEBUG_SOLUTION
					m_b_system_dirty = true;
				}
				// update the system (in parallel)

				if(!b_cholesky_result)
					break;
				// in case cholesky failed, quit
			}
		}

		if(csA1)
			cs_spfree(csA1);

		{
			//ok soso lets do this ... or you can just curl under the table and sob uncontrollably... no, man up!
			//you can do this... we need more brain juice.. where is the beer? ...

			//step line 20-21
			//_ASSERTE(A1.n_row_num >= A2.n_column_num);
		}
		// by an unknown poet
	}

	/**
	 *	@brief displays performance info on stdout
	 *	@param[in] f_total_time is total time taken by everything (can be -1 to ommit)
	 */
	void Dump(double f_total_time = -1) const
	{
		printf("the SPCG solver took " PRIsize " iterations\n", m_n_iteration_num); // debug, to be able to say we didn't botch it numerically
		if(f_total_time > 0)
			printf("solver spent %f seconds in parallelizable section (updating A)\n", f_total_time - m_f_serial_time);
		printf("out of which:\n");
		printf("\t Aref: %f\n", m_f_A_refresh_time);
		printf("\tspant: %f\n", m_f_spanning_tree_time);
		printf("\tAperm: %f\n", m_f_A_perm_time);

		printf("solver spent %f seconds in serial section\n", m_f_serial_time);
		printf("out of which:\n");
		printf("\t  rhs: %f\n", m_f_rhs_time);
		printf("\tbperm: %f\n", m_f_perm_b_time);
		printf("\t   qr: %f\n", m_f_qr_time);
		printf("\tsolve: %f\n", m_f_solve_time);
		printf("\t  ata: %f\n", m_f_ata_time);
		printf("\tgaxpy: %f\n", m_f_premul_time);
		printf("\t chol: %f\n", m_f_chol_time);
		printf("\t norm: %f\n", m_f_norm_time);
		printf("\ttotal: %f\n", m_f_ata_time + m_f_premul_time + m_f_chol_time + m_f_norm_time);
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns \f$\chi^2\f$ error.
	 *	@note This only works with systems with edges of one degree of freedom
	 *		(won't work for e.g. systems with both poses and landmarks).
	 */
	inline double f_Chi_Squared_Error() const
	{
		if(m_r_system.r_Edge_Pool().b_Empty())
			return 0;
		return m_r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError()) /
			(m_r_system.r_Edge_Pool().n_Size() - m_r_system.r_Edge_Pool()[0].n_Dimension());
	}

	/**
	 *	@brief calculates denormalized \f$\chi^2\f$ error
	 *	@return Returns denormalized \f$\chi^2\f$ error.
	 *	@note This doesn't perform the final division by (number of edges - degree of freedoms).
	 */
	inline double f_Chi_Squared_Error_Denorm() const
	{
		if(m_r_system.r_Edge_Pool().b_Empty())
			return 0;
		return m_r_system.r_Edge_Pool().For_Each(CSum_ChiSquareError());
	}

	/**
	 *	@brief writes system matrix for art purposes
	 *
	 *	@param[in] p_s_filename is output file name (.tga)
	 *	@param[in] n_scalar_size is size of one scalar, in pixels
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Dump_SystemMatrix(const char *p_s_filename, int n_scalar_size = 5)
	{
		try {
			Extend_A(m_n_edges_in_A); // recalculated all the jacobians inside Extend_A()
			if(!m_b_system_dirty)
				Refresh_A(m_n_edges_in_A); // calculate only for new edges
			else
				Refresh_A(); // calculate for entire system
			m_b_system_dirty = false;
			m_n_edges_in_A = m_r_system.r_Edge_Pool().n_Size(); // right? // yes.
			_ASSERTE(m_A.n_Row_Num() >= m_A.n_Column_Num()); // no, A is *not* square, but size of measurements >= vertices
			// need to have A
		} catch(std::bad_alloc&) {
			return false;
		}

		return m_A.Rasterize(p_s_filename, n_scalar_size);
	}

	/**
	 *	@brief writes system matrix in matrix market for benchmarking purposes
	 *
	 *	@param[in] p_s_filename is output file name (.mtx)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Save_SystemMatrix_MM(const char *p_s_filename) const
	{
		size_t n_rows = m_A.n_Row_Num();
		size_t n_columns = m_A.n_Column_Num();
		size_t n_nonzeros = m_A.n_NonZero_Num();
		FILE *p_fw;
		if(!(p_fw = fopen(p_s_filename, "w")))
			return false;
		fprintf(p_fw, "%%%%MatrixMarket matrix coordinate real general\n");
		fprintf(p_fw, "%%-------------------------------------------------------------------------------\n");
		fprintf(p_fw, "%% SLAM_plus_plus matrix dump\n");
		fprintf(p_fw, "%% kind: A matrix for SLAM problem\n");
		fprintf(p_fw, "%%-------------------------------------------------------------------------------\n");
		fprintf(p_fw, "" PRIsize " " PRIsize " " PRIsize "\n", n_rows, n_columns, n_nonzeros);
		for(size_t n_col = 0, n_column_blocks = m_A.n_BlockColumn_Num();
		   n_col < n_column_blocks; ++ n_col) {
			size_t n_col_base = m_A.n_BlockColumn_Base(n_col);
			size_t n_col_width = m_A.n_BlockColumn_Column_Num(n_col);
			size_t n_block_num = m_A.n_BlockColumn_Block_Num(n_col);
			for(size_t j = 0; j < n_block_num; ++ j) {
				size_t n_row = m_A.n_Block_Row(n_col, j);
				size_t n_row_base = m_A.n_BlockRow_Base(n_row);
				size_t n_row_height = m_A.n_BlockRow_Row_Num(n_row);

				CUberBlockMatrix::_TyMatrixXdRef t_block = m_A.t_BlockAt(j, n_col);
				_ASSERTE(t_block.rows() == n_row_height && t_block.cols() == n_col_width);
				// get a block

				for(size_t k = 0; k < n_row_height; ++ k) {
					for(size_t l = 0; l < n_col_width; ++ l) {
						fprintf(p_fw, "" PRIsize " " PRIsize " %f\n", n_row_base + 1 + k,
							n_col_base + 1 + l, t_block(k, l));
					}
				}
				// print a block (includes the nulls, but so does n_NonZero_Num())
			}
			// for all blocks in a column
		}
		// for all columns

		if(ferror(p_fw)) {
			fclose(p_fw);
			return false;
		}
		fclose(p_fw);

		return true;
	}

	/**
	 *	@brief incremental optimization function
	 *	@param[in] r_last_edge is the last edge that was added to the system
	 *	@note This function throws std::bad_alloc.
	 */
	void Incremental_Step(_TyBaseEdge &UNUSED(r_last_edge)) // throw(std::bad_alloc)
	{
		size_t n_vertex_num = m_r_system.r_Vertex_Pool().n_Size();
		if(!m_b_had_loop_closure) {
			_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
			typename _TyEdgeMultiPool::_TyConstBaseRef r_edge =
				m_r_system.r_Edge_Pool()[m_r_system.r_Edge_Pool().n_Size() - 1];
			// get a reference to the last edge interface

			if(r_edge.n_Vertex_Num() > 1) { // unary factors do not cause classical loop closures
				_ASSERTE(r_edge.n_Vertex_Id(0) != r_edge.n_Vertex_Id(1));
				size_t n_first_vertex = std::min(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1));
				m_b_had_loop_closure = (n_first_vertex < n_vertex_num - 2);
				//_ASSERTE(m_b_had_loop_closure || std::max(r_edge.n_Vertex_Id(0),
				//	r_edge.n_Vertex_Id(1)) == n_vertex_num - 1); // won't work with const vertices
			} else {
				size_t n_first_vertex = r_edge.n_Vertex_Id(0);
				m_b_had_loop_closure = (n_first_vertex < n_vertex_num - 1);
			}
			// todo - encapsulate this code, write code to support hyperedges as well, use it
		}
		// detect loop closures (otherwise the edges are initialized based on measurement and error would be zero)

		++ m_n_real_step;

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		size_t n_new_vertex_num = n_vertex_num - m_n_last_optimized_vertex_num;
		// the optimization periods are counted in vertices, not in edges (should save work on 10k, 100k)
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		++ m_n_step;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES

#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		if(m_n_nonlinear_solve_threshold && n_new_vertex_num >= m_n_nonlinear_solve_threshold) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		if(m_n_nonlinear_solve_threshold && m_n_step == m_n_nonlinear_solve_threshold) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure) {
				m_b_had_loop_closure = false;
				Optimize(m_n_nonlinear_solve_max_iteration_num, m_f_nonlinear_solve_error_threshold);
			}
			// nonlinear optimization
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_n_linear_solve_threshold && n_new_vertex_num >= m_n_linear_solve_threshold) {
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
		} else if(m_n_linear_solve_threshold && m_n_step % m_n_linear_solve_threshold == 0) {
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
#ifdef __SLAM_COUNT_ITERATIONS_AS_VERTICES
			_ASSERTE(!m_n_nonlinear_solve_threshold);
			m_n_last_optimized_vertex_num = n_vertex_num;
#else // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			if(!m_n_nonlinear_solve_threshold)
				m_n_step = 0;
#endif // __SLAM_COUNT_ITERATIONS_AS_VERTICES
			// do this first, in case Optimize() threw

			if(m_b_had_loop_closure) {
				m_b_had_loop_closure = false;
				Optimize(1, 0); // only if there was a loop (ignores possibly high residual after single step optimization)
			}
			// simple optimization
		}
	}

protected:
	/**
	 *	@brief function object that calls hessian block allocation for all edges
	 */
	class CAlloc_JacobianBlocks {
	protected:
		CUberBlockMatrix &m_r_A; /**< @brief reference to the A matrix (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_A is reference to the A matrix
		 */
		inline CAlloc_JacobianBlocks(CUberBlockMatrix &r_A)
			:m_r_A(r_A)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to have hessian blocks allocated in A
		 *	@note This function throws std::bad_alloc.
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) // throw(std::bad_alloc)
		{
			r_t_edge.Alloc_JacobianBlocks(m_r_A);
		}
	};

	/**
	 *	@brief function object that calls error vector calculation for all edges
	 */
	class CCollect_R_Errors {
	protected:
		Eigen::VectorXd &m_r_R_errors; /**< @brief reference to the R error vector (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_R_errors is reference to the R error vector
		 */
		inline CCollect_R_Errors(Eigen::VectorXd &r_R_errors)
			:m_r_R_errors(r_R_errors)
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to output its part R error vector
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge)
		{
			r_t_edge.Get_R_Error(m_r_R_errors);
		}
	};

	/**
	 *	@brief calculates the error vector
	 */
	inline void Collect_R_Errors(Eigen::VectorXd &r_v_R_error)
	{
		const Eigen::VectorXd &r_v_err = m_r_system.r_v_Unary_Error();
		r_v_R_error.segment(0, r_v_err.rows()) = r_v_err;
		// add the first error

		m_r_system.r_Edge_Pool().For_Each_Parallel(CCollect_R_Errors(r_v_R_error)); // can do this in parallel
		// collect errors
	}

	/**
	 *	@brief function object that copies ordering from the edges
	 */
	class CGetCumsums {
	protected:
		std::vector<size_t>::iterator m_p_cumsum_it; /**< @brief cumsum iterator (out) */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_row_cumsum_list is cumsum iterator
		 */
		inline CGetCumsums(std::vector<size_t> &r_row_cumsum_list)
			:m_p_cumsum_it(r_row_cumsum_list.begin())
		{}

		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in,out] r_t_edge is edge to output its cumsum
		 */
		template <class _TyEdge>
		inline void operator ()(const _TyEdge &r_t_edge)
		{
			size_t n_order = r_t_edge.n_Order();
			_ASSERTE(n_order > 0); // don't want 0, but that is assigned to the unary factor
			*m_p_cumsum_it = n_order;
			++ m_p_cumsum_it;
		}

		/**
		 *	@brief conversion back to cumsum iterator
		 *	@return Returns the value of cumsum iterator.
		 */
		inline operator std::vector<size_t>::iterator() const
		{
			return m_p_cumsum_it;
		}
	};

	/**
	 *	@brief creates the A matrix from scratch
	 */
	inline void AddEntriesInSparseSystem() // throw(std::bad_alloc, std::runtime_error)
	{
		if(m_r_system.r_Edge_Pool().n_Size() > 1000) { // wins 2.42237 - 2.48938 = .06701 seconds on 10k.graph, likely more on larger graphs
			//printf("building large matrix from scratch ...\n"); // debug
			std::vector<size_t> row_cumsum_list(m_r_system.r_Edge_Pool().n_Size());
			/*std::vector<size_t>::iterator p_end_it =*/
				m_r_system.r_Edge_Pool().For_Each(CGetCumsums(row_cumsum_list));
			//_ASSERTE(p_end_it == row_cumsum_list.end());
			// collect cumsums

			CUberBlockMatrix tmp(row_cumsum_list.begin(),
				row_cumsum_list.end(), m_r_system.r_Vertex_Pool().n_Size());
			m_A.Swap(tmp);
			// use this one instead

			// todo - see if there are some row_reindex on 100k, fix it by collecting
			// cumsums and building matrix with that (proven to be faster before)
		} else {
			//printf("building small matrix from scratch ...\n"); // debug
			m_A.Clear();
			// ...
		}

		{
			size_t n_UF_order = m_r_system.n_Unary_Factor_Order();
			const Eigen::MatrixXd &r_t_uf = m_r_system.r_t_Unary_Factor();
			if(!r_t_uf.cols())
				throw std::runtime_error("system matrix assembled but unary factor not initialized yet"); // if this triggers, consider sorting your dataset or using an explicit CUnaryFactor
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			size_t n_gauge_vertex = 0; // simple
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!m_r_system.r_Edge_Pool().b_Empty());
			size_t n_gauge_vertex = m_r_system.r_Edge_Pool()[0].n_Vertex_Id(0);
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
			_ASSERTE(!m_r_system.r_Vertex_Pool()[n_gauge_vertex].b_IsConstant()); // this one must not be
			size_t n_gauge_order = m_r_system.r_Vertex_Pool()[n_gauge_vertex].n_Order();
			if(!m_A.Append_Block(r_t_uf, n_UF_order, n_gauge_order)) // put the UF at the right spot
				throw std::bad_alloc();
		}
		// add unary factor

		m_r_system.r_Edge_Pool().For_Each(CAlloc_JacobianBlocks(m_A));
		// add all the hessian blocks

		//printf("building A from scratch finished\n"); // debug
	}

	/**
	 *	@brief incrementally updates the A matrix structure (must not be empty)
	 *	@param[in] n_skip_edges is number of edges, already in the system
	 *	@note This function throws std::bad_alloc.
	 */
	inline void UpdateSparseSystem(size_t n_skip_edges) // throw(std::bad_alloc)
	{
		_ASSERTE(m_A.n_Row_Num() > 0 && m_A.n_Column_Num() > 0); // make sure A is not empty
		m_r_system.r_Edge_Pool().For_Each(n_skip_edges, m_r_system.r_Edge_Pool().n_Size(), CAlloc_JacobianBlocks(m_A));
		// add the hessian blocks of the new edges
	}

	/**
	 *	@brief incrementally updates the A matrix structure (can be empty)
	 *	@param[in] n_edges_already_in_A is number of edges, already in the system
	 *	@note This function throws std::bad_alloc.
	 */
	inline void Extend_A(size_t n_edges_already_in_A) // throw(std::bad_alloc)
	{
		if(!n_edges_already_in_A)
			AddEntriesInSparseSystem();
		else
			UpdateSparseSystem(n_edges_already_in_A);
		// create block matrix A
	}

	/**
	 *	@brief refreshes the A matrix by recalculating edge hessians
	 *	@param[in] n_refresh_from_edge is zero-based index of the first edge to be refreshed
	 */
	inline void Refresh_A(size_t n_refresh_from_edge = 0)
	{
		if(n_refresh_from_edge) {
			m_r_system.r_Edge_Pool().For_Each_Parallel(n_refresh_from_edge,
				m_r_system.r_Edge_Pool().n_Size(), CCalculate_Jacobians());
		} else
			m_r_system.r_Edge_Pool().For_Each_Parallel(CCalculate_Jacobians());
		// can do this in parallel
	}

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
		const Eigen::VectorXd &m_r_v_dx; /**< @brief vector of differences */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_v_dx is reference to the vector of differences
		 */
		inline CUpdateEstimates(const Eigen::VectorXd &r_v_dx)
			:m_r_v_dx(r_v_dx)
		{}

		/**
		 *	@brief updates vertex state
		 *	@tparam _TyVertex is type of vertex
		 *	@param[in,out] r_t_vertex is reference to vertex, being updated
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex) const
		{
			r_t_vertex.Operator_Plus(m_r_v_dx);
		}
	};

	/**
	 *	@brief function object that swaps states of all the vertices
	 */
	class CSwapEstimates {
	protected:
		Eigen::VectorXd &m_r_v_x; /**< @brief the solution vector */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in,out] r_v_x is reference to the solution vector
		 */
		inline CSwapEstimates(Eigen::VectorXd &r_v_x)
			:m_r_v_x(r_v_x)
		{}

		/**
		 *	@brief updates vertex state
		 *	@tparam _TyVertex is type of vertex
		 *	@param[in,out] r_t_vertex is reference to vertex, being updated
		 */
		template <class _TyVertex>
		inline void operator ()(_TyVertex &r_t_vertex)
		{
			r_t_vertex.SwapState(m_r_v_x);
		}
	};

	/**
	 *	@brief updates states of all the vertices with the error vector
	 *	@param[in] r_v_dx is reference to the vector of differences
	 */
	inline void PushValuesInGraphSystem(const Eigen::VectorXd &r_v_dx)
	{
		m_r_system.r_Vertex_Pool().For_Each_Parallel(CUpdateEstimates(r_v_dx)); // can do this in parallel
	}

	/**
	 *	@brief swaps states of all the vertices with the x vector
	 *	@param[in,out] r_v_x is reference to the solution vector
	 */
	inline void PushValuesInGraphSystem2(Eigen::VectorXd &r_v_x)
	{
		m_r_system.r_Vertex_Pool().For_Each_Parallel(CSwapEstimates(r_v_x)); // can do this in parallel
	}

	/**
	 *	@brief function object that calculates hessians in all the edges
	 */
	class CCalculate_Jacobians {
	public:
		/**
		 *	@brief function operator
		 *	@tparam _TyEdge is edge type
		 *	@param[in] r_t_edge is edge to update it's hessians
		 */
		template <class _TyEdge>
		inline void operator ()(_TyEdge &r_t_edge) const
		{
			r_t_edge.Calculate_Jacobians();
		}
	};

	CNonlinearSolver_SPCG(const CNonlinearSolver_SPCG &UNUSED(r_solver)); /**< @brief the object is not copyable */
	CNonlinearSolver_SPCG &operator =(const CNonlinearSolver_SPCG &UNUSED(r_solver)) { return *this; } /**< @brief the object is not copyable */
};

/** @} */ // end of group

#endif // !__NONLINEAR_BLOCKY_SOLVER_SPCG_INCLUDED
