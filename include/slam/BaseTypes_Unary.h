/*
								+----------------------------------+
								|                                  |
								|    ***  Base graph types  ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|        BaseTypes_Unary.h         |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __BASE_GRAPH_PRIMITIVE_TYPES_UNARY_EDGE_IMPL_SPECIALIZATION_INCLUDED
#define __BASE_GRAPH_PRIMITIVE_TYPES_UNARY_EDGE_IMPL_SPECIALIZATION_INCLUDED

/**
 *	@file include/slam/BaseTypes_Unary.h
 *	@brief base graph primitive types, specialization of unary edges
 *	@author -tHE SWINe-
 *	@date 2014-11-05
 *	@note This file is not supposed to be included by itself, it is included from include/slam/BaseTypes.h.
 */

#ifdef DOXYGEN
#include "slam/BaseTypes.h" // for doxygen
#endif // DOXYGEN

/** \addtogroup graph
 *	@{
 */

/**
 *	@brief implementation of solver required functions for a generic edge type (specialization for unary edges)
 *
 *	@tparam CDerivedEdge is the name of derived edge class
 *	@tparam CVertexTypeList is list of types of the vertices
 *	@tparam _n_residual_dimension is residual vector dimension
 *	@tparam _n_storage_dimension is state vector storage dimension (or -1 if the same as _n_residual_dimension)
 */
template <class CDerivedEdge, class CVertexType, int _n_residual_dimension,
	int _n_storage_dimension /*= -1*/, unsigned int _n_options /*= 0*/>
class CBaseEdgeImpl<CDerivedEdge, CTypelist<CVertexType, CTypelistEnd>,
	_n_residual_dimension, _n_storage_dimension, _n_options> : public CBaseEdge {
public:
	typedef CTypelist<CVertexType, CTypelistEnd> _TyVertices; /**< @brief list of vertex types */
	typedef CVertexType /*typename CTypelistItemAt<_TyVertices, 0>::_TyResult*/ _TyVertex; /**< @brief name of the vertex class */

	/**
	 *	@brief copy of template parameters
	 */
	enum {
		n_vertex_num = 1, /**< @brief number of vertices */
		n_measurement_dimension = _n_residual_dimension, /**< @brief measurement vector dimension (edge dimension) @deprecated This is slightly unclear, use n_residual_dimension and n_storage_dimension instead. */
		n_residual_dimension = _n_residual_dimension, /**< @brief residual vector dimension */
		n_storage_dimension = (_n_storage_dimension == -1)? _n_residual_dimension : _n_storage_dimension, /**< @brief edge state storage dimension */
#ifdef __BASE_TYPES_USE_ALIGNED_MATRICES
		n_matrix_alignment = Eigen::AutoAlign | Eigen::ColMajor,
#else // __BASE_TYPES_USE_ALIGNED_MATRICES
		n_matrix_alignment = Eigen::DontAlign | Eigen::ColMajor,
#endif // __BASE_TYPES_USE_ALIGNED_MATRICES
		n_options = _n_options /**< @brief edge options bitfield */
	};

	/**
	 *	@brief local vertex traits
	 *	@tparam n_index is zero-based index of a vertex in CVertexTypeList
	 */
	template <const int n_index>
	class CVertexTraits {
	public:
		typedef typename CTypelistItemAt<_TyVertices, n_index>::_TyResult _TyVertex; /**< @brief type of vertex */

		/**
		 *	@brief copy of vertex parameters
		 */
		enum {
			n_dimension = _TyVertex::n_dimension, /**< @brief vertex dimension */
		};

		typedef fbs_ut::CCTSize2D<n_dimension, n_dimension> _TyMatrixSize; /**< @brief size of the matrix as fbs_ut::CCTSize2D */
		typedef typename _TyVertex::_TyVector _TyVector; /**< @brief state vector type */
		typedef typename _TyVertex::_TyMatrix _TyMatrix; /**< @brief a compatible matrix type */
		typedef typename _TyVertex::_TyVectorAlign _TyVectorAlign; /**< @brief aligned state vector type */
		typedef typename _TyVertex::_TyMatrixAlign _TyMatrixAlign; /**< @brief a compatible aligned matrix type */
		typedef Eigen::Matrix<double, n_residual_dimension, n_dimension> _TyJacobianMatrix; /**< @brief mixed edge / vertex dimension matrix type */
	};

	/**
	 *	@copydoc base_edge_detail::CInitializeNullVertex
	 */
	template <class CVertex = _TyVertex>
	class CInitializeNullVertex : public base_edge_detail::CInitializeNullVertex<CVertex> {};

	/**
	 *	@copydoc base_edge_detail::CInitializeVertex_Disallow
	 */
	template <class CVertex = _TyVertex>
	class CInitializeVertex_Disallow : public base_edge_detail::CInitializeVertex_Disallow<CVertex> {};

	typedef Eigen::Matrix<double, n_residual_dimension, 1> _TyVector; /**< @brief edge dimension vector type */
	typedef Eigen::Matrix<double, n_storage_dimension, 1> _TyStorageVector; /**< @brief storage dimension vector type */
	typedef Eigen::Matrix<double, n_residual_dimension, n_residual_dimension> _TyMatrix; /**< @brief edge dimension matrix type */

	typedef Eigen::Matrix<double, n_storage_dimension, 1, n_matrix_alignment> _TyStorageVectorAlign; /**< @brief edge dimension storage vector type with member alignment */
	typedef Eigen::Matrix<double, n_residual_dimension, 1, n_matrix_alignment> _TyVectorAlign; /**< @brief edge dimension vector type with member alignment */
	typedef Eigen::Matrix<double, n_residual_dimension, n_residual_dimension, n_matrix_alignment> _TyMatrixAlign; /**< @brief edge dimension matrix type with member alignment */

	/**
	 *	@brief edge parameters, stored as enum
	 */
	enum {
		b_is_robust_edge = (n_options & CBaseEdge::Robust) != 0 /**< @brief robust edge detection */
		//b_is_robust_edge = base_edge_detail::CIsRobustEdge<CBaseEdgeImpl<CDerivedEdge, _TyVertices, _n_residual_dimension, _n_storage_dimension>, _TyVector>::b_result
	};

protected:
	size_t m_p_vertex_id[n_vertex_num]; /**< @brief ids of referenced vertices */
	_TyVertex *m_p_vertex0; /**< @brief pointers to the referenced vertices */
	_TyStorageVectorAlign m_v_measurement; /**< @brief the measurement */
	_TyMatrixAlign m_t_sigma_inv; /**< @brief information matrix */

private:
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific members ---

	_TyMatrixAlign m_t_square_root_sigma_inv_upper; /**< @brief the R matrix (upper diagonal) = transpose(chol(t_sigma_inv)) */
	_TyVectorAlign m_v_error; /**< @brief error vector (needs to be filled explicitly by calling p_error_function) */
	double *m_p_RH[n_vertex_num]; /**< @brief blocks of memory where the R * H matrices are stored */

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific members ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific members ---
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN

	double *m_p_HtSiH_vert[n_vertex_num];
	double *m_p_RHS_vert[n_vertex_num];
	// this is used only by the new lambda reduction

#else // __LAMBDA_USE_V2_REDUCTION_PLAN

	typename CVertexTraits<0>::_TyMatrixAlign m_t_HtSiH_vertex0; /**< @brief block of memory where Ht * inv(Sigma) * H matrix is stored (the vertex contribution) */
	typename CVertexTraits<0>::_TyVectorAlign m_t_right_hand_vertex0; /**< @brief block of memory where right-hand side vector is stored (the vertex contribution) */
	// t_odo - this also needs to be passed through the reductor, did not think of this (it is a slightly different
	// philosophy, it is supposed to reduce to a dense vector, but it will be reduced to a sparse blah. maybe do
	// the reduction using an offset table only)
	// this is only used by the oldschool lambda reduction

#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific members ---

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CBaseEdgeImpl()
	{}

	/**
	 *	@brief unary edge constructor
	 *
	 *	@param[in] n_vertex_id is id of the vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *
	 *	@note This fills the structure, except for the vertex pointer.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 */
	inline CBaseEdgeImpl(size_t n_vertex_id, const _TyStorageVector &r_v_measurement,
		const _TyMatrix &r_t_sigma_inv)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		m_p_vertex_id[0] = n_vertex_id;
		// get references to the vertices, initialize the vertices, if necessary

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief unary edge constructor with implicit vertex initialization (to null)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_vertex0_id is id of the first vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in,out] r_system is reference to system (used to query or insert edge vertices)
	 *	@param[in] b_disallow_implicit_init is flag determining whether the initialization is
	 *		allowed - an exception is thrown if the vertex is not explicitly inizialized prior
	 *		to calling this function
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::bad_alloc if there is not enough memory for the vertices,
	 *		and throws std::runtime_error if b_disallow_implicit_init is set and the vertex
	 *		n_vertex0_id was not explicitly initialized before calling this function.
	 */
	template <class CSystem>
	inline CBaseEdgeImpl(size_t n_vertex0_id,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		null_initialize_vertices_tag UNUSED(t_tag), CSystem &r_system,
		bool b_disallow_implicit_init = false) // throw(std::bad_alloc, std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		m_p_vertex_id[0] = n_vertex0_id;
		// get references to the vertices, initialize the vertices, if necessary

		if(!b_disallow_implicit_init)
			m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex>(n_vertex0_id, CInitializeNullVertex<>());
		else
			m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex>(n_vertex0_id, CInitializeVertex_Disallow<>());
		// initialize the vertices

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief unary edge constructor with exmplicit vertex initialization
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_vertex0_id is id of the first vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in] r_system is reference to system (used to query edge vertices)
	 *
	 *	@note This fills the structure, including the vertex pointer.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::runtime_error if the vertex n_vertex0_id was not
	 *		explicitly initialized before calling this function.
	 */
	template <class CSystem>
	inline CBaseEdgeImpl(size_t n_vertex0_id,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		explicitly_initialized_vertices_tag UNUSED(t_tag), CSystem &r_system) // throw(std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		m_p_vertex_id[0] = n_vertex0_id;
		// get references to the vertices, initialize the vertices, if necessary

		m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex>(n_vertex0_id, CInitializeVertex_Disallow<>());
		// initialize the vertex pointer, throw an exception if not initialized already

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief updates the edge with new measurement
	 *
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 */
	inline void Update(const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv)
	{
		m_v_measurement = r_v_measurement;
		m_t_sigma_inv = r_t_sigma_inv;
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		m_t_square_root_sigma_inv_upper = r_t_sigma_inv.llt().matrixU(); // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	}

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::n_Dimension()
	 */
	inline size_t n_Dimension() const
	{
		return n_residual_dimension;
	}

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::n_Vertex_Num
	 */
	inline size_t n_Vertex_Num() const
	{
		return n_vertex_num;
	}

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::n_Vertex_Id
	 */
	inline size_t n_Vertex_Id(size_t n_vertex) const
	{
		_ASSERTE(n_vertex < n_vertex_num);
		return m_p_vertex_id[n_vertex];
	}

	/**
	 *	@brief gets the measurement
	 *	@return Returns const reference to the measurement.
	 */
	inline const _TyStorageVectorAlign &v_Measurement() const
	{
		return m_v_measurement;
	}

	/**
	 *	@brief gets inverse sigma of the measurement
	 *	@return Returns const reference to the inverse sigma matrix.
	 */
	inline const _TyMatrixAlign &t_Sigma_Inv() const
	{
		return m_t_sigma_inv;
	}

	/**
	 *	@brief calculates Jacobian, based on a state of an arbitrary vertex
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, corresponding to the vertex
	 *	@param[out] r_v_expectation is expectation
	 *	@param[out] r_v_error is error
	 *	@param[in] r_vertex0 is reference to the vertex
	 *	@param[in] r_v_measurement is measurement
	 *
	 *	@note In case the derived edge has extended state, it should implement
	 *		a custom version of this function.
	 */
	static inline void Calculate_Jacobians(
		typename CVertexTraits<0>::_TyJacobianMatrix &r_t_jacobian0,
		_TyVector &r_v_expectation, _TyVector &r_v_error,
		const typename CVertexTraits<0>::_TyVertex &r_vertex0,
		const _TyStorageVector &r_v_measurement)
	{
		_ASSERTE(n_vertex_num == 1); // this is always true, this is a specialization for binary edges
		// t_odo - support n-ary edges (need a different interface for that)

		CDerivedEdge dummy; // relies on having default constructor (most edges should)
		dummy.m_p_vertex0 = &const_cast<typename CVertexTraits<0>::_TyVertex&>(r_vertex0);
		dummy.m_v_measurement = r_v_measurement;
		((const CDerivedEdge&)dummy).Calculate_Jacobian_Expectation_Error(r_t_jacobian0, r_v_expectation, r_v_error);
		// Calculate_Jacobians_Expectation_Error in CDerivedEdge should have been
		// static and it would be simple, but now we can'T force the users to rewrite
		// all their code. meh.

		// this will not be modified for robust edges, we're just
		// after the jacobians here and not the sigma or weight
	}

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

	/**
	 *	@copydoc base_iface::CEdgeFacade::Alloc_JacobianBlocks
	 */
	inline void Alloc_JacobianBlocks(CUberBlockMatrix &r_A)
	{
		_ASSERTE(!m_p_vertex0->b_IsConstant()); // make sure the vertex is not const (if it is, do not put this edge in the system in the first place!)
		if(m_p_vertex0->b_IsConstant())
			throw std::runtime_error("unary edge created with a constant vertex"); // make this a release check as well, it depends on the data

		// easily implemented using a loop
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
		m_p_RH[0] = r_A.p_GetBlock_Log(m_n_id + 1, m_p_vertex_id[0],
			n_residual_dimension, CVertexTraits<0>::n_dimension, true, false);
#else // __BASE_TYPES_USE_ID_ADDRESSING
		m_p_RH[0] = r_A.p_FindBlock(m_n_order, m_p_vertex0->n_Order(),
			n_residual_dimension, CVertexTraits<0>::n_dimension, true, false);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
		// just place blocks at (edge order, vertex order)

		_ASSERTE(m_p_RH[0]);
		// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
		// and the edges come in some random order
	}

protected: // todo - this is overly hackish, just write the code twice for robust / not robust edges
	template <class V>
	inline typename CEnableIf<b_is_robust_edge && sizeof(V), double>::T f_Get_RobustWeight(const V &r_v_error)
	{
		return ((CDerivedEdge*)this)->f_RobustWeight(r_v_error);
	}

	template <class V>
	inline typename CEnableIf<!b_is_robust_edge && sizeof(V), double>::T f_Get_RobustWeight(const V &UNUSED(r_v_error))
	{
		return 1;
	}

public:
	/**
	 *	@copydoc base_iface::CEdgeFacade::Calculate_Jacobians
	 */
	inline void Calculate_Jacobians()
	{
		// easily implemented using a loop
		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobian_Expectation_Error(t_jacobian0, v_expectation, v_error);

		if(b_is_robust_edge) { // compile-time constant
			double f_robust_weight = sqrt(f_Get_RobustWeight(v_error));

			m_v_error = v_error * f_robust_weight;
			// calculates the expectation, error and the jacobians (implemented by the edge)

			Eigen::Map<typename CVertexTraits<0>::_TyJacobianMatrix, CUberBlockMatrix::map_Alignment> t_RH0(m_p_RH[0]);
			// map Jacobian matrices

			t_RH0 = m_t_square_root_sigma_inv_upper * t_jacobian0 * f_robust_weight;
			// recalculate RH (transpose cholesky of sigma times the jacobian)
			// note this references the A block matrix
		} else {
			m_v_error = v_error;
			// calculates the expectation, error and the jacobians (implemented by the edge)

			Eigen::Map<typename CVertexTraits<0>::_TyJacobianMatrix, CUberBlockMatrix::map_Alignment> t_RH0(m_p_RH[0]);
			// map Jacobian matrices

			t_RH0 = m_t_square_root_sigma_inv_upper * t_jacobian0;
			// recalculate RH (transpose cholesky of sigma times the jacobian)
			// note this references the A block matrix
		}
	}

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::Get_R_Error
	 */
	inline void Get_R_Error(Eigen::VectorXd &r_v_dest) const
	{
		r_v_dest.segment<n_residual_dimension>(m_n_order) = m_t_square_root_sigma_inv_upper * m_v_error;
	}

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::Get_Error
	 */
	inline void Get_Error(Eigen::VectorXd &r_v_dest) const
	{
		r_v_dest.segment<n_residual_dimension>(m_n_order) = m_v_error;
	}

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- ~A-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- lambda-SLAM specific functions ---
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@brief allocates Hessian blocks
	 *
	 *	@tparam CReductionPlan is lambda reduction plan ?nstantiation
	 *
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@param[in,out] r_rp is lambda reduction plan
	 *
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	template <class CReductionPlan>
	inline void Alloc_HessianBlocks_v2(CUberBlockMatrix &r_lambda, CReductionPlan &r_rp)
	{
		_ASSERTE(!m_p_vertex0->b_IsConstant()); // make sure the vertex is not const (if it is, do not put this edge in the system in the first place!)
		if(m_p_vertex0->b_IsConstant())
			throw std::runtime_error("unary edge created with a constant vertex"); // make this a release check as well, it depends on the data

#ifdef __BASE_TYPES_USE_ID_ADDRESSING
		size_t n_id_0 = m_p_vertex_id[0];
		m_p_HtSiH_vert[0] = r_lambda.p_GetBlock_Log(n_id_0, n_id_0,
			CVertexTraits<0>::n_dimension, CVertexTraits<0>::n_dimension, true, false);
#else // __BASE_TYPES_USE_ID_ADDRESSING
		const size_t n_order_0 = m_p_vertex0->n_Order();
		m_p_HtSiH_vert[0] = r_lambda.p_FindBlock(n_order_0, n_order_0,
			CVertexTraits<0>::n_dimension, CVertexTraits<0>::n_dimension, true, false);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
		// find a block for vertices' Hessian on the diagonal (do not use the potentially swapped id / order)
		// note that if this is added after the off-diagonal Hessian, it will be likely
		// added at the end of the block list in the matrix

		{
			typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();

			size_t n_id_0 = m_p_vertex_id[0];
			m_p_HtSiH_vert[0] = rp.template p_Diagonal_GetTempBlock<typename
				CVertexTraits<0>::_TyMatrixSize>(n_id_0, n_id_0, m_p_HtSiH_vert[0]);
		}
		// make space for the vertex in the reduction scheme

		{
			typename CReductionPlan::CRHSReductor &rp = r_rp.r_RHS_ReductionPlan();

			m_p_RHS_vert[0] = rp.template p_Get_ReductionBlock<CVertexTraits<0>::n_dimension>(m_p_vertex0->n_Order());
		}
		// alloc the RHS reduction temporary

		// in virtual call scenario, either system or solver would need to specialize a virtual interface
		// on reduction plan, which would in turn need to export non-template functions with runtime seatch
		// of appropriate bin for a given block size
	}

	/**
	 *	@brief sums up Hessian block contributions to get values of the blocks in lambda
	 *
	 *	@tparam CReductionPlan is lambda reduction plan ?nstantiation
	 *
	 *	@param[in] r_lambda is the target matrix where the blocks are stored
	 *	@param[in,out] r_rp is lambda reduction plan
	 *
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 *	@note This only reduces the Hessians, Calculate_Hessians_v2() must be called
	 *		before to have stuff to calculate.
	 */
	template <class CReductionPlan>
	inline void Reduce_Hessians_v2(const CUberBlockMatrix &r_lambda, CReductionPlan &r_rp) // todo - doc
	{
		typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();
		const size_t n_id_0 = m_p_vertex_id[0];//, n_id_1 = m_p_vertex_id[1];

		typename CReductionPlan::CLambdaReductor::TKey p_key_vert[n_vertex_num];
		if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) // compile-time const
			p_key_vert[0] = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_0, 0); // pointer ignored
		else {
			const double *p_HtSiH_vert[n_vertex_num];
			p_HtSiH_vert[0] = r_lambda.p_GetBlock_Log(n_id_0, n_id_0,
				CVertexTraits<0>::n_dimension, CVertexTraits<0>::n_dimension);
			_ASSERTE(p_HtSiH_vert[0]);
			// need the address of the unique *original* block, not of one of the many reduction temporaries
			// t_odo - perhaps it would be better to use (row, col) as unique index to the reduction, this lookup would then be avoided
			p_key_vert[0] = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_0, p_HtSiH_vert[0]);
		}
		rp.template ReduceSingle<typename CVertexTraits<0>::_TyMatrixSize>(p_key_vert[0]);
		// reduce off-diagonal Hessian and both diagonal Hessians
	}

	/**
	 *	@brief calculates Hessian contributions
	 *
	 *	@note This only calculates the Hessians, either Reduce_Hessians_v2() or the
	 *		reduction plan needs to be used to fill lambda with values.
	 */
	inline void Calculate_Hessians_v2()
	{
		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobian_Expectation_Error(t_jacobian0, v_expectation, v_error);
		// calculates the expectation and the jacobian

		if(b_is_robust_edge) { // compile-time constant
			double f_robust_weight = f_Get_RobustWeight(v_error);

			Eigen::Map<typename CVertexTraits<0>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment>
				m_t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			if(hyperedge_detail::explicit_SymmetricProduct) // compile-time const
				m_t_HtSiH_vertex0/*.noalias()*/ = (t_jacobian0.transpose() * m_t_sigma_inv * t_jacobian0 * f_robust_weight).template selfadjointView<Eigen::Upper>();
			else
				m_t_HtSiH_vertex0.noalias() = t_jacobian0.transpose() * m_t_sigma_inv * t_jacobian0 * f_robust_weight;
			// calculate diagonal Hessian contribution

			//_ASSERTE(m_t_HtSiH_vertex0 == m_t_HtSiH_vertex0.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<0>::_TyVectorAlign, CUberBlockMatrix::map_Alignment>
				m_t_right_hand_vertex0(m_p_RHS_vert[0]);
			m_t_right_hand_vertex0.noalias() = t_jacobian0.transpose() * (m_t_sigma_inv * v_error * f_robust_weight);
			// calculate right hand side vector contribution
		} else {
			Eigen::Map<typename CVertexTraits<0>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment>
				m_t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			if(hyperedge_detail::explicit_SymmetricProduct) // compile-time const
				m_t_HtSiH_vertex0/*.noalias()*/ = (t_jacobian0.transpose() * m_t_sigma_inv * t_jacobian0).template selfadjointView<Eigen::Upper>();
			else
				m_t_HtSiH_vertex0.noalias() = t_jacobian0.transpose() * m_t_sigma_inv * t_jacobian0;
			// calculate diagonal Hessian contribution

			//_ASSERTE(m_t_HtSiH_vertex0 == m_t_HtSiH_vertex0.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<0>::_TyVectorAlign, CUberBlockMatrix::map_Alignment>
				m_t_right_hand_vertex0(m_p_RHS_vert[0]);
			m_t_right_hand_vertex0.noalias() = t_jacobian0.transpose() * (m_t_sigma_inv * v_error);
			// calculate right hand side vector contribution
		}
	}

	/**
	 *	@brief calculates the gradient descent scaling factor
	 *	@param[in] r_v_g is the g vector from the dogleg paper (\f$-J^T \Sigma^{-1} r\f$)
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 */
	inline double f_Calculate_Steepest_Descent_Scale(const Eigen::VectorXd &r_v_g)
	{
		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobian_Expectation_Error(t_jacobian0, v_expectation, v_error);
		// calculates the expectation and the jacobian // this is inefficient as it needs to re-evaluate the jacobians

		_TyVector v_Jg = t_jacobian0 * r_v_g.segment<CVertexTraits<0>::n_dimension>(m_p_vertex0->n_Order());
		// calculate the right hand side vector contribution
		// no need to change this for robust edges; it works

		return v_Jg.squaredNorm(); // need to sum up in vector first, then take the norm
	}

	/**
	 *	@brief calculates Hessian contributions and the gradient descent scaling factor
	 *
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 *
	 *	@note This only calculates the Hessians, either Reduce_Hessians_v2() or the
	 *		reduction plan needs to be used to fill lambda with values.
	 */
	inline double f_Calculate_Hessians_and_Steepest_Descent_Scale()
	{
		_ASSERTE(!b_is_robust_edge); // robust edges not supported here (todo if it ends up being used, otherwise remove it)

		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobian_Expectation_Error(t_jacobian0, v_expectation, v_error);
		// calculates the expectation and the jacobian

		Eigen::Map<typename CVertexTraits<0>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment>
			m_t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
		m_t_HtSiH_vertex0.noalias() = t_jacobian0.transpose() * m_t_sigma_inv * t_jacobian0;
		// calculate diagonal Hessian contribution

		Eigen::Map<typename CVertexTraits<0>::_TyVectorAlign, CUberBlockMatrix::map_Alignment>
			m_t_right_hand_vertex0(m_p_RHS_vert[0]);
		typename CVertexTraits<0>::_TyVectorAlign g = t_jacobian0.transpose() * (m_t_sigma_inv * v_error);
		m_t_right_hand_vertex0.noalias() = g;
		// calculate right hand side vector contribution

		return (t_jacobian0 * g).squaredNorm();
	}

#else // __LAMBDA_USE_V2_REDUCTION_PLAN

	inline bool Notify_HessianBlock_Conflict(double *p_block, CUberBlockMatrix &r_lambda)
	{
		// can't happen with unary edges
		return false; // not me
	}

	inline void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda)
	{
		m_p_vertex0->Add_ReferencingEdge(this);
		// the vertices need to know this edge in order to calculate sum of Hessians on the diagonal of lambda // t_odo - should the lambda solver call this instead? might avoid duplicate records in the future // lambda solver is calling this only once in it's lifetime, no duplicates will occur

		// alloc nothing, the vertex will allocate the diagonal block
	}

	inline void Calculate_Hessians()
	{
		_ASSERTE(m_p_vertex0->b_IsReferencingEdge(this));
		// makes sure this is amongst the referencing edges

		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobian_Expectation_Error(t_jacobian0, v_expectation, v_error);
		// calculates the expectation and the jacobian

		m_t_HtSiH_vertex0.noalias() = t_jacobian0.transpose() * m_t_sigma_inv * t_jacobian0;
		// calculate diagonal Hessian contribution

		m_t_right_hand_vertex0.noalias() = t_jacobian0.transpose() * (m_t_sigma_inv * v_error);
		// calculate right hand side vector contribution
	}

	inline std::pair<const double*, const double*> t_Get_LambdaEta_Contribution(const CBaseVertex *p_which)
	{
		// no redfuction here, it all takes place in the vertex

		_ASSERTE(p_which == m_p_vertex0);
		return std::make_pair(m_t_HtSiH_vertex0.data(), m_t_right_hand_vertex0.data());
		// return pointer to the matrix data with Hessian contribution
	}

#endif // __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::f_Max_VertexHessianDiagValue
	 */
	double f_Max_VertexHessianDiagValue() const // todo surround with levenberg support ifdef
	{
		double f_max = 0;

#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
		Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
			CUberBlockMatrix::map_Alignment> m_t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
		// need to map them first (note that storing a map instead of a pointer would save some hurdles)
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN

		for(size_t i = 0; i < CVertexTraits<0>::n_dimension; ++ i)
			f_max = std::max(f_max, m_t_HtSiH_vertex0(i, i));
		return f_max;
	}

#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::Alloc_LBlocks
	 */
	inline void Alloc_LBlocks(CUberBlockMatrix &UNUSED(r_L)) const
	{
		// no offdiag blocks
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---

	/**
	 *	@brief calculates just a part of the lambda matrix, called omega
	 *
	 *	This is used in CNonlinearSolver_L for incremental updates to the factor.
	 *	It calculates omega from scratch, uses serial execution, as opposed
	 *	to Calculate_Hessians(). While it may seem slower, it can't likely be parallelized
	 *	due to very small size of omega (can be just a single edge).
	 *
	 *	@param[out] r_omega is the omega matrix to be filled (must be initially empty)
	 *	@param[in] n_min_vertex_order is order offset, in elements, used to position
	 *		the Hessian blocks in the upper-left corner of omega
	 */
	inline void Calculate_Omega(CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const
	{
		_ASSERTE(!m_p_vertex0->b_IsConstant()); // make sure the vertex is not const (if it is, do not put this edge in the system in the first place!)

		const size_t n_dimension0 = CVertexTraits<0>::n_dimension;
		const size_t n_order_0 = m_p_vertex0->n_Order();
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_order_0 >= n_min_vertex_order);
		double *p_v0 = r_omega.p_FindBlock(n_order_0 - n_min_vertex_order,
			n_order_0 - n_min_vertex_order, n_dimension0, n_dimension0, true, true);
		// alloc and initialize / find existing blocks for all the Hessians, above the diagonal only

		bool b_recalculate = false;
		// recalculate from scratch, or use values already calculated by the lambda solver?

		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		if(b_recalculate) {
			_TyVector v_expectation, v_error;
			((CDerivedEdge*)this)->Calculate_Jacobian_Expectation_Error(t_jacobian0,
				v_expectation, v_error);
		}
		// calculates the expectation and the jacobians

		Eigen::Matrix<double, CVertexTraits<0>::n_dimension, n_residual_dimension> t_H0_sigma_inv;
		if(b_recalculate)
			t_H0_sigma_inv = t_jacobian0.transpose() * m_t_sigma_inv;

		Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
			CUberBlockMatrix::map_Alignment> t_HtSiH_v0(p_v0);
		if(b_recalculate) {
			t_HtSiH_v0.noalias() += t_H0_sigma_inv * t_jacobian0;
		} else {
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
			Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			t_HtSiH_v0 += t_HtSiH_vertex0;
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
			t_HtSiH_v0 += m_t_HtSiH_vertex0;
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
		}
		// calculate diagonal Hessian contribution (note the increment)
	}
};

/**
 *	@brief unary factor template
 *
 *	This lets one set a unary factor on an arbitrary vertex, in addition
 *	to the first vertex in the system which receives one automatically (unless
 *	the system is specialized with \ref CNullUnaryFactorFactory).
 *	This version assumes that the unary error is zero but it would be quite
 *	easy to modify. Effectively, this reduces the movement of the vertex
 *	throughout the optimization but does not anchor it to a specific position
 *	(it is as if it was always chained to its current position), potentially
 *	accelerating convergence.
 *
 *	To get nonzero errors, it is also trivial to use a binary edge
 *	in conjunction with a constant vertex, where the constant vertex
 *	would amount to the anchor (perhaps represented by a null vector
 *	to anchor at the origin) and the binary edge would provide derivatives
 *	and error calculation. You can see \ref constvertices for more details
 *	on the use of constant vertices.
 *
 *	The rules for automatic unary factor placement depend on
 *	\ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO being defined. If defined,
 *	vertex with id 0 is chosen, otherwise the first vertex of the first
 *	edge is chosen. By default, the unary factor is an identity matrix
 *	of the same dimension as the vertex (see \ref CBasicUnaryFactorFactory).
 *
 *	@tparam CVertexType is vertex type to build the factor around
 */
template <class CVertexType>
class CUnaryFactor : public CBaseEdgeImpl<CUnaryFactor<CVertexType>,
	typename MakeTypelist(CVertexType), CVertexType::n_dimension, 0> {
public:
	typedef CBaseEdgeImpl<CUnaryFactor<CVertexType>, typename
		MakeTypelist(CVertexType), CVertexType::n_dimension, 0> _TyBase; /**< @brief base edge type */

	typedef CVertexType _TyVertex; /**< @brief vertex type */
	typedef typename _TyBase::_TyMatrix _TyFactor; /**< @brief unary factor type */
	typedef typename _TyBase::_TyVector _TyError; /**< @brief unary error type */
	typedef Eigen::Matrix<double, 0, 1> NullVector; /**< @brief null vector type */
	typedef std::pair<size_t, _TyFactor> TParsed; /**< @brief parsed representation of this edge */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CUnaryFactor()
	{}

	/**
	 *	@brief constructor; converts parsed edge to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] r_t_edge is parsed edge
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CUnaryFactor(const TParsed &r_t_edge, CSystem &r_system)
		:_TyBase(r_t_edge.first, NullVector(), r_t_edge.second)
	{
		this->m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex>(r_t_edge.first,
			typename _TyBase::template CInitializeNullVertex<>());
		// get vertices

		_ASSERTE(!hyperedge_detail::no_ConstVertices ||
			r_system.r_Vertex_Pool()[r_t_edge.first].n_Dimension() == CVertexType::n_dimension);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief constructor; converts edge data to edge representation
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_node0 is (zero-based) index of the first (origin) node
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CUnaryFactor(int n_node0, const _TyFactor &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_node0, NullVector(), r_t_inv_sigma)
	{
		this->m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex>(n_node0,
			typename _TyBase::template CInitializeNullVertex<>());
		// get vertices

		_ASSERTE(!hyperedge_detail::no_ConstVertices ||
			r_system.r_Vertex_Pool()[n_node0].n_Dimension() == CVertexType::n_dimension);
		// make sure the dimensionality is correct (might not be)
		// this fails with const vertices, for obvious reasons. with the thunk tables this can be safely removed.
	}

	/**
	 *	@brief updates the edge with a new measurement
	 *
	 *	@param[in] r_t_edge is parsed edge
	 */
	inline void Update(const TParsed &r_t_edge)
	{
		_TyBase::Update(NullVector(), r_t_edge.second);
	}

	/**
	 *	@brief calculates Jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, associated with the first vertex
	 *	@param[out] r_v_expectation is expecation vector
	 *	@param[out] r_v_error is error vector
	 */
	inline void Calculate_Jacobian_Expectation_Error(_TyFactor &r_t_jacobian0,
		_TyError &r_v_expectation, _TyError &r_v_error) const
	{
		r_t_jacobian0 = this->m_t_sigma_inv; // !!
		r_v_expectation = this->m_p_vertex0->r_v_State(); // !!
		r_v_error.setZero();
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		return 0;
	}
};

/**
 *	@brief implementation of solver required functions for a generic edge type (specialization for unary edges)
 *
 *	@tparam CDerivedEdge is the name of derived edge class
 *	@tparam CVertexTypeList is list of types of the vertices
 *	@tparam _n_residual_dimension is residual vector dimension
 *	@tparam _n_storage_dimension is state vector storage dimension (or -1 if the same as _n_residual_dimension)
 *
 *	@deprecated This name is deprecated; use CBaseEdgeImpl instead.
 */
template <class CDerivedEdge, class CVertexType, int _n_residual_dimension, int _n_storage_dimension /*= -1*/>
class CSEBaseEdgeImpl<CDerivedEdge, CTypelist<CVertexType, CTypelistEnd>,
	_n_residual_dimension, _n_storage_dimension> : public CBaseEdgeImpl<CDerivedEdge,
	CTypelist<CVertexType, CTypelistEnd>, _n_residual_dimension, _n_storage_dimension> {
private:
	typedef CBaseEdgeImpl<CDerivedEdge, CTypelist<CVertexType, CTypelistEnd>,
		_n_residual_dimension, _n_storage_dimension> _TyNotDeprecated; /**< @brief name of the parent type that is not deprecated */
	// this is not intended to be used by the derived classes because CBaseEdgeImpl does not have such type

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CSEBaseEdgeImpl()
		:_TyNotDeprecated()
	{}

	/**
	 *	@brief unary edge constructor
	 *
	 *	@param[in] n_vertex_id is id of the vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *
	 *	@note This fills the structure, except for the vertex pointer.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 */
	inline CSEBaseEdgeImpl(size_t n_vertex_id,
		const typename _TyNotDeprecated::_TyStorageVector &r_v_measurement,
		const typename _TyNotDeprecated::_TyMatrix &r_t_sigma_inv)
		:_TyNotDeprecated(n_vertex_id, r_v_measurement, r_t_sigma_inv)
	{}
};

/** @} */ // end of group

/**
 *	@page unaryfactors Using Unary Factors
 *
 *	Unary factor is a simple factor, much like other measurements, which anchors the
 *	graph in order for the problem to be well defined (to reduce so called gauge freedom,
 *	and as a side effect also for the system matrix to be positive definite).
 *
 *	In some optimizers, the user is required to add unary factor explicitly. In SLAM++
 *	this happens automatically and the unary factor is placed on vertex 0. Usually, one
 *	does not need to worry about it too much, but when needed, this can be changed.
 *
 *	The default implicit unary factor is given by the fifth template argument
 *	to the \ref CFlatSystem and by default this is \ref CBasicUnaryFactorFactory
 *	(it sets unit covariance and zero error, effectively reducing the norm of the
 *	updates taken on this vertex by the nonlinear solver; see extended discussion
 *	in \ref CUnaryFactor documentation). It is possible to also initialize it via
 *	the \ref CFlatSystem constructor, to e.g. specify different than unit covariance.
 *	Setting the unary factor this way has the advantage that although unary factor
 *	is formally an edge, it does not need to be in the list of edges, and for systems
 *	with only a single edge type in them, this brings some simplifications and possible
 *	compiler optimizations. If needed, it is quite simple to implement a custom unary
 *	factor factory.
 *
 *	If the unary factor is supposed to be on a different vertex than vertex 0,
 *	one can disable the implicit unary factor factory and to use an explicit unary
 *	factor. If we for example had:
 *
 *	@code
 *	typedef MakeTypelist(CVertexPose2D, CVertexLandmark2D) TVertexTypelist;
 *	typedef MakeTypelist(CEdgePose2D, CEdgePoseLandmark2D) TEdgeTypelist;
 *	@endcode
 *
 *	and:
 *
 *	@code
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist> CSystemType;
 *	@endcode
 *
 *	which is equivalent to:
 *
 *	@code
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist, CBasicUnaryFactorFactory> CSystemType;
 *	@endcode
 *
 *	We can easily disable the implicit unary factor by changing this to:
 *
 *	@code
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist, CNullUnaryFactorFactory> CSystemType;
 *	@endcode
 *
 *	As the name suggests, \ref CNullUnaryFactorFactory sets the implicit unary
 *	factor to a null matrix (although most solvers specialize for that and do not
 *	actually involve it in the calculations in any way). Now we have a system with
 *	no unary factor, which cannot be optimized. To add an explicit unary factor,
 *	we use the \ref CUnaryFactor template:
 *
 *	@code
 *	typedef MakeTypelist(CEdgePose2D, CEdgePoseLandmark2D,
 *		CUnaryFactor<CVertexPose2D>) TEdgeTypelist; // add it here
 *
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist, CNullUnaryFactorFactory> CSystemType;
 *	@endcode
 *
 *	and then explicitly specify this factor:
 *
 *	@code
 *	CSystemType system; // instance of the optimized system
 *
 *	system.r_Add_Edge(CUnaryFactor<CVertexPose2D>(0, Eigen::Matrix3d::Identity() * 100, system));
 *	// adds unary factor of eye(3) * 100 on vertex 0, which is a pose
 *	@endcode
 */

#endif // !__BASE_GRAPH_PRIMITIVE_TYPES_UNARY_EDGE_IMPL_SPECIALIZATION_INCLUDED
