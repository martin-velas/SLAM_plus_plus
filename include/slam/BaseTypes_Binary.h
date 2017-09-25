/*
								+----------------------------------+
								|                                  |
								|    ***  Base graph types  ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|        BaseTypes_Binary.h        |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __BASE_GRAPH_PRIMITIVE_TYPES_BINARY_EDGE_IMPL_SPECIALIZATION_INCLUDED
#define __BASE_GRAPH_PRIMITIVE_TYPES_BINARY_EDGE_IMPL_SPECIALIZATION_INCLUDED

/**
 *	@file include/slam/BaseTypes_Binary.h
 *	@brief base graph primitive types, specialization of binary edges
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
 *	@brief implementation of solver required functions for a generic edge type (specialization for binary edges)
 *
 *	@tparam CDerivedEdge is the name of derived edge class
 *	@tparam CVertexType0 is type of the first vertiex
 *	@tparam CVertexType1 is type of the second vertiex
 *	@tparam _n_residual_dimension is residual vector dimension
 *	@tparam _n_storage_dimension is state vector storage dimension (or -1 if the same as _n_residual_dimension)
 */
template <class CDerivedEdge, class CVertexType0, class CVertexType1,
	int _n_residual_dimension, int _n_storage_dimension /*= -1*/, unsigned int _n_options /*= 0*/>
class CBaseEdgeImpl<CDerivedEdge, MakeTypelist2(CVertexType0, CVertexType1),
	_n_residual_dimension, _n_storage_dimension, _n_options> : public CBaseEdge {
public:
	typedef typename MakeTypelist(CVertexType0, CVertexType1) _TyVertices; /**< @brief list of vertex types */
	typedef CVertexType0 /*typename CTypelistItemAt<_TyVertices, 0>::_TyResult*/ _TyVertex0; /**< @brief name of the first vertex class */
	typedef CVertexType1 /*typename CTypelistItemAt<_TyVertices, 1>::_TyResult*/ _TyVertex1; /**< @brief name of the second vertex class */

	/**
	 *	@brief copy of template parameters
	 */
	enum {
		n_vertex_num = 2/*CTypelistLength<_TyVertices>::n_result*/, /**< @brief number of vertices */
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
	template <class CVertex = _TyVertex0>
	class CInitializeNullVertex: public base_edge_detail::CInitializeNullVertex<CVertex> {};

	/**
	 *	@copydoc base_edge_detail::CInitializeVertex_Disallow
	 */
	template <class CVertex = _TyVertex0>
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
		//b_is_robust_edge = base_edge_detail::CIsRobustEdge</*CBaseEdgeImpl<*/CDerivedEdge/*, _TyVertices, _n_residual_dimension, _n_storage_dimension>*/, _TyVector>::b_result
	};

	//typedef int YOU_MADE_A_ROBUST_EDGE[(b_is_robust_edge)? -1 : 1]; // debug; used when writing detection of robust edges from the presence of the f_RobustWeight() function (doesn't work, the derived type is incomplete at the point of instantiating this class)

protected:
	size_t m_p_vertex_id[n_vertex_num]; /**< @brief ids of referenced vertices */
	_TyVertex0 *m_p_vertex0; /**< @brief pointers to the referenced vertices */
	_TyVertex1 *m_p_vertex1; /**< @brief pointers to the referenced vertices */
	_TyStorageVectorAlign m_v_measurement; /**< @brief the measurement */
	_TyMatrixAlign m_t_sigma_inv; /**< @brief information matrix */

private:
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific members ---

	_TyMatrixAlign m_t_square_root_sigma_inv_upper; /**< @brief the R matrix (upper diagonal) = transpose(chol(t_sigma_inv)) */
	_TyVectorAlign m_v_error; /**< @brief error vector (needs to be filled explicitly by calling p_error_function) */
	double *m_p_RH[n_vertex_num]; /**< @brief blocks of memory where the R * H matrices are stored */

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific members ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific members ---

	double *m_p_HtSiH; /**< @brief block of memory where Ht * inv(Sigma) * H matrix is stored (the one above diagonal) */

#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN

	double *m_p_HtSiH_vert[n_vertex_num];
	double *m_p_RHS_vert[n_vertex_num];
	// this is used only by the new lambda reduction

#else // __LAMBDA_USE_V2_REDUCTION_PLAN

	double *m_p_HtSiH_reduce; /**< @brief block of memory where Ht * inv(Sigma) * H matrix is stored in the matrix (should there be conflicts between duplicate edges, this is the target of reduction) */
	bool m_b_first_reductor; /**< @brief reduction flag */
	typename CVertexTraits<0>::_TyMatrixAlign m_t_HtSiH_vertex0; /**< @brief block of memory where Ht * inv(Sigma) * H matrix is stored (the diagonal block) */
	typename CVertexTraits<1>::_TyMatrixAlign m_t_HtSiH_vertex1; /**< @brief block of memory where Ht * inv(Sigma) * H matrix is stored (the diagonal block) */
	typename CVertexTraits<0>::_TyVectorAlign m_t_right_hand_vertex0; /**< @brief block of memory where right-hand side vector is stored (vertex 0 error) */
	typename CVertexTraits<1>::_TyVectorAlign m_t_right_hand_vertex1; /**< @brief block of memory where right-hand side vector is stored (vertex 1 error) */
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
	 *	@brief binary edge constructor
	 *
	 *	@param[in] n_vertex0_id is id of the first vertex
	 *	@param[in] n_vertex1_id is id of the second vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *
	 *	@note This fills the structure, except for the vertex pointers.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 */
	inline CBaseEdgeImpl(size_t n_vertex0_id, size_t n_vertex1_id,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		m_p_vertex_id[0] = n_vertex0_id;
		m_p_vertex_id[1] = n_vertex1_id;
		// get references to the vertices, initialize the vertices, if necessary

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief binary edge constructor with implicit vertex initialization (to null)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_vertex0_id is id of the first vertex
	 *	@param[in] n_vertex1_id is id of the second vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in,out] r_system is reference to system (used to query or insert edge vertices)
	 *	@param[in] n_list_skip is bitfield of vertices for which the initialization should
	 *		be disabled so that the derived edge can perform it (LSB corresponds to vertex 0,
	 *		the second least significant bit to vertex 1)
	 *	@param[in] n_list_disallow is bitfield of vertices for which the initialization is not
	 *		allowed - an exception is thrown if the vertex is not explicitly inizialized prior
	 *		to calling this function (value unused)
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::bad_alloc if there is not enough memory for the vertices,
	 *		and throws std::runtime_error if a vertex in n_list_disallow and not in n_list_skip
	 *		was not explicitly initialized before calling this function.
	 *	@note n_list_skip takes precedence over n_list_disallow.
	 */
	template <class CSystem>
	inline CBaseEdgeImpl(size_t n_vertex0_id, size_t n_vertex1_id,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		null_initialize_vertices_tag UNUSED(t_tag), CSystem &r_system,
		int n_list_skip = 0, int n_list_disallow = 0) // throw(std::bad_alloc, std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		m_p_vertex_id[0] = n_vertex0_id;
		m_p_vertex_id[1] = n_vertex1_id;
		// get references to the vertices, initialize the vertices, if necessary

		if(!(n_list_skip & 1)) {
			if(!(n_list_disallow & 1))
				m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex0>(n_vertex0_id, CInitializeNullVertex<_TyVertex0>());
			else
				m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex0>(n_vertex0_id, CInitializeVertex_Disallow<_TyVertex0>());
		}
		if(!(n_list_skip & 2)) {
			if(!(n_list_disallow & 2))
				m_p_vertex1 = &r_system.template r_Get_Vertex<_TyVertex1>(n_vertex1_id, CInitializeNullVertex<_TyVertex1>());
			else
				m_p_vertex1 = &r_system.template r_Get_Vertex<_TyVertex1>(n_vertex1_id, CInitializeVertex_Disallow<_TyVertex1>());
		}
		// initialize the vertices

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief binary edge constructor with exmplicit vertex initialization
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_vertex0_id is id of the first vertex
	 *	@param[in] n_vertex1_id is id of the second vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in] r_system is reference to system (used to query edge vertices)
	 *	@param[in] n_list_skip is bitfield of vertices for which the initialization should
	 *		be disabled so that the derived edge can perform it (LSB corresponds to vertex 0,
	 *		the second least significant bit to vertex 1)
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::runtime_error if a vertex not in n_list_skip was not
	 *		explicitly initialized before calling this function.
	 */
	template <class CSystem>
	inline CBaseEdgeImpl(size_t n_vertex0_id, size_t n_vertex1_id,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		explicitly_initialized_vertices_tag UNUSED(t_tag), CSystem &r_system,
		int n_list_skip = 0) // throw(std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		m_p_vertex_id[0] = n_vertex0_id;
		m_p_vertex_id[1] = n_vertex1_id;
		// get references to the vertices, initialize the vertices, if necessary

		if(!(n_list_skip & 1))
			m_p_vertex0 = &r_system.template r_Get_Vertex<_TyVertex0>(n_vertex0_id, CInitializeVertex_Disallow<_TyVertex0>());
		if(!(n_list_skip & 2))
			m_p_vertex1 = &r_system.template r_Get_Vertex<_TyVertex1>(n_vertex1_id, CInitializeVertex_Disallow<_TyVertex1>());
		// initialize the vertex pointers, throw an exception if not initialized already

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
	 *	@brief calculates Jacobians, based on states of two arbitrary vertices
	 *
	 *	@param[out] r_t_jacobian0 is jacobian, corresponding to the first vertex
	 *	@param[out] r_t_jacobian1 is jacobian, corresponding to the second vertex
	 *	@param[out] r_v_expectation is expectation
	 *	@param[out] r_v_error is error
	 *	@param[in] r_vertex0 is reference to the first vertex
	 *	@param[in] r_vertex1 is reference to the second vertex
	 *	@param[in] r_v_measurement is measurement
	 *
	 *	@note In case the derived edge has extended state, it should implement
	 *		a custom version of this function.
	 */
	static inline void Calculate_Jacobians(
		typename CVertexTraits<0>::_TyJacobianMatrix &r_t_jacobian0,
		typename CVertexTraits<1>::_TyJacobianMatrix &r_t_jacobian1,
		_TyVector &r_v_expectation, _TyVector &r_v_error,
		const typename CVertexTraits<0>::_TyVertex &r_vertex0,
		const typename CVertexTraits<1>::_TyVertex &r_vertex1,
		const _TyStorageVector &r_v_measurement)
	{
		_ASSERTE(n_vertex_num == 2); // this is always true, this is a specialization for binary edges
		// todo - support n-ary edges (need a different interface for that)

		CDerivedEdge dummy; // relies on having default constructor (most edges should)
		dummy.m_p_vertex0 = &const_cast<typename CVertexTraits<0>::_TyVertex&>(r_vertex0);
		dummy.m_p_vertex1 = &const_cast<typename CVertexTraits<1>::_TyVertex&>(r_vertex1);
		dummy.m_v_measurement = r_v_measurement;
		((const CDerivedEdge&)dummy).Calculate_Jacobians_Expectation_Error(r_t_jacobian0,
			r_t_jacobian1, r_v_expectation, r_v_error);
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
		_ASSERTE(!m_p_vertex0->b_IsConstant() || !m_p_vertex1->b_IsConstant());
		// if this triggers then all the vertices are constant. do not add this edge to the system in the first place

#ifdef _DEBUG
		for(size_t i = 0; i < n_vertex_num; ++ i)
			m_p_RH[i] = 0;
#endif // _DEBUG

#ifdef __BASE_TYPES_USE_ID_ADDRESSING
		if(!m_p_vertex0->b_IsConstant()) {
			m_p_RH[0] = r_A.p_GetBlock_Log(m_n_id + 1, m_p_vertex_id[0],
				n_residual_dimension, CVertexTraits<0>::n_dimension, true, false);
		}
		if(!m_p_vertex1->b_IsConstant()) {
			m_p_RH[1] = r_A.p_GetBlock_Log(m_n_id + 1, m_p_vertex_id[1],
				n_residual_dimension, CVertexTraits<1>::n_dimension, true, false);
		}
#else // __BASE_TYPES_USE_ID_ADDRESSING
		if(!m_p_vertex0->b_IsConstant()) {
			m_p_RH[0] = r_A.p_FindBlock(m_n_order, m_p_vertex0->n_Order(),
				n_residual_dimension, CVertexTraits<0>::n_dimension, true, false);
		}
		if(!m_p_vertex1->b_IsConstant()) {
			m_p_RH[1] = r_A.p_FindBlock(m_n_order, m_p_vertex1->n_Order(),
				n_residual_dimension, CVertexTraits<1>::n_dimension, true, false);
		}
#endif // __BASE_TYPES_USE_ID_ADDRESSING
		// just place blocks at (edge order, vertex order)

		_ASSERTE(m_p_RH[0] || m_p_vertex0->b_IsConstant());
		_ASSERTE(m_p_RH[1] || m_p_vertex1->b_IsConstant());
		// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
		// and the edges come in some random order
	}

	/**
	 *	@copydoc base_iface::CEdgeFacade::Calculate_Jacobians
	 */
	inline void Calculate_Jacobians()
	{
		// easily implemented using a loop
		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
			t_jacobian1, v_expectation, v_error);

		if(b_is_robust_edge) { // compile-time constant
			double f_robust_weight = sqrt(f_Get_RobustWeight(v_error));

			m_v_error = v_error * f_robust_weight;
			// calculates the expectation, error and the jacobians (implemented by the edge)

			if(!m_p_vertex0->b_IsConstant()) {
				Eigen::Map<typename CVertexTraits<0>::_TyJacobianMatrix, CUberBlockMatrix::map_Alignment> t_RH0(m_p_RH[0]);
				t_RH0 = f_robust_weight * m_t_square_root_sigma_inv_upper * t_jacobian0;
			}
			if(!m_p_vertex1->b_IsConstant()) {
				Eigen::Map<typename CVertexTraits<1>::_TyJacobianMatrix, CUberBlockMatrix::map_Alignment> t_RH1(m_p_RH[1]); // t_odo - support aligned if the uberblockmatrix is aligned!
				t_RH1 = f_robust_weight * m_t_square_root_sigma_inv_upper * t_jacobian1;
			}
			// recalculate RH (transpose cholesky of sigma times the jacobian)
			// note this references the A block matrix
		} else {
			m_v_error = v_error;
			// calculates the expectation, error and the jacobians (implemented by the edge)

			if(!m_p_vertex0->b_IsConstant()) {
				Eigen::Map<typename CVertexTraits<0>::_TyJacobianMatrix, CUberBlockMatrix::map_Alignment> t_RH0(m_p_RH[0]);
				t_RH0 = m_t_square_root_sigma_inv_upper * t_jacobian0;
			}
			if(!m_p_vertex1->b_IsConstant()) {
				Eigen::Map<typename CVertexTraits<1>::_TyJacobianMatrix, CUberBlockMatrix::map_Alignment> t_RH1(m_p_RH[1]); // t_odo - support aligned if the uberblockmatrix is aligned!
				t_RH1 = m_t_square_root_sigma_inv_upper * t_jacobian1;
			}
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
		_ASSERTE(!m_p_vertex0->b_IsConstant() || !m_p_vertex1->b_IsConstant());
		// if this triggers then all the vertices are constant. do not add this edge to the system in the first place

		_ASSERTE((m_p_vertex_id[0] > m_p_vertex_id[1]) ==
			(m_p_vertex0->n_Order() > m_p_vertex1->n_Order()) ||
			m_p_vertex0->b_IsConstant() || m_p_vertex1->b_IsConstant());
		// if this triggers, then the edge has the vertices assigned in a different
		// order than the ids (vertex[0] is id[1] and vice versa) consequently, the Hessians
		// will have correct shape in the matrix, but the data will be transposed
		// and you will get either not pos def / rubbish solutions

		const bool b_optimize0 = !m_p_vertex0->b_IsConstant();
		const bool b_optimize1 = !m_p_vertex1->b_IsConstant();

		bool b_transpose_block, b_uninit_block;
		if(b_optimize0 && b_optimize1) { // only need the off-diagonal block if both are optimized
			size_t n_dimension0 = CVertexTraits<0>::n_dimension;
			size_t n_dimension1 = CVertexTraits<1>::n_dimension;
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			size_t n_id_0 = m_p_vertex_id[0];
			size_t n_id_1 = m_p_vertex_id[1];
			if((b_transpose_block = (n_id_0 > n_id_1))) {
				std::swap(n_id_0, n_id_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_id_0 < n_id_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			m_p_HtSiH = r_lambda.p_GetBlock_Log_Alloc(n_id_0, n_id_1, n_dimension0, n_dimension1, b_uninit_block);
#else // __BASE_TYPES_USE_ID_ADDRESSING
			size_t n_order_0 = m_p_vertex0->n_Order();
			size_t n_order_1 = m_p_vertex1->n_Order();
			if((b_transpose_block = (n_order_0 > n_order_1))) {
				std::swap(n_order_0, n_order_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_order_0 < n_order_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			m_p_HtSiH = r_lambda.p_FindBlock_Alloc(n_order_0, n_order_1, n_dimension0, n_dimension1, b_uninit_block);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			// find a block for Hessian above the diagonal, and with the right shape

			_ASSERTE(m_p_HtSiH);
			// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
			// and the edges come in some random order
		} else
			m_p_HtSiH = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING
		if(b_optimize0) {
			m_p_HtSiH_vert[0] = r_lambda.p_GetBlock_Log(m_p_vertex_id[0], m_p_vertex_id[0],
				CVertexTraits<0>::n_dimension, CVertexTraits<0>::n_dimension, true, false);
		}
		if(b_optimize1) {
			m_p_HtSiH_vert[1] = r_lambda.p_GetBlock_Log(m_p_vertex_id[1], m_p_vertex_id[1],
				CVertexTraits<1>::n_dimension, CVertexTraits<1>::n_dimension, true, false);
		}
#else // __BASE_TYPES_USE_ID_ADDRESSING
		if(b_optimize0) {
			m_p_HtSiH_vert[0] = r_lambda.p_FindBlock(m_p_vertex0->n_Order(), m_p_vertex0->n_Order(),
				CVertexTraits<0>::n_dimension, CVertexTraits<0>::n_dimension, true, false);
		}
		if(b_optimize1) {
			m_p_HtSiH_vert[1] = r_lambda.p_FindBlock(m_p_vertex1->n_Order(), m_p_vertex1->n_Order(),
				CVertexTraits<1>::n_dimension, CVertexTraits<1>::n_dimension, true, false);
		}
#endif // __BASE_TYPES_USE_ID_ADDRESSING
		// find a block for vertices' Hessian on the diagonal (do not use the potentially swapped id / order)
		// note that if this is added after the off-diagonal Hessian, it will be likely
		// added at the end of the block list in the matrix

		{
			typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();

			if(b_optimize0) {
				m_p_HtSiH_vert[0] = rp.template p_Diagonal_GetTempBlock<typename
					CVertexTraits<0>::_TyMatrixSize>(m_p_vertex_id[0], m_p_vertex_id[0], m_p_HtSiH_vert[0]);
			} else
				m_p_HtSiH_vert[0] = 0;
			if(b_optimize1) {
				m_p_HtSiH_vert[1] = rp.template p_Diagonal_GetTempBlock<typename
					CVertexTraits<1>::_TyMatrixSize>(m_p_vertex_id[1], m_p_vertex_id[1], m_p_HtSiH_vert[1]);
			} else
				m_p_HtSiH_vert[1] = 0;
			// make space for the diagonal blocks in the reduction scheme

			if(b_optimize0 && b_optimize1) {
				if(b_transpose_block) {
					typedef fbs_ut::CCTSize2D<CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension> BlockSize;
					if(b_uninit_block)
						rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_HtSiH, &m_p_HtSiH); // return value ignored (equals m_p_HtSiH)
					else {
						m_p_HtSiH = rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_vertex_id[1],
							m_p_vertex_id[0], m_p_HtSiH); // potentially perform relocation to a temp block for reduction
					}
				} else {
					typedef fbs_ut::CCTSize2D<CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension> BlockSize;
					if(b_uninit_block)
						rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_HtSiH, &m_p_HtSiH); // return value ignored (equals m_p_HtSiH)
					else {
						m_p_HtSiH = rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_vertex_id[0],
							m_p_vertex_id[1], m_p_HtSiH); // potentially perform relocation to a temp block for reduction
					}
				}
				// make space for the off-diagonal block in the reduction scheme
			}
		}

		{
			typename CReductionPlan::CRHSReductor &rp = r_rp.r_RHS_ReductionPlan();

			if(b_optimize0)
				m_p_RHS_vert[0] = rp.template p_Get_ReductionBlock<CVertexTraits<0>::n_dimension>(m_p_vertex0->n_Order());
			if(b_optimize1)
				m_p_RHS_vert[1] = rp.template p_Get_ReductionBlock<CVertexTraits<1>::n_dimension>(m_p_vertex1->n_Order());
		}
		// alloc the RHS reduction temporaries

		// t_odo - continue with this // there's nothing more to do here

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
		const size_t n_id_0 = m_p_vertex_id[0], n_id_1 = m_p_vertex_id[1];
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH) {
			_ASSERTE((n_id_0 > n_id_1) == (m_p_vertex0->n_Order() > m_p_vertex1->n_Order())); // if this triggers, then the edge has the vertices assigned in different order than the ids (vertex[0] is id[1] and vice versa)
			if(n_id_0 > n_id_1) {
				typename CReductionPlan::CLambdaReductor::TKey t_key;
				if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) // compile-time const
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_1, n_id_0, 0); // pointer ignored
				else {
					const double *p_HtSiH = r_lambda.p_GetBlock_Log(n_id_1, n_id_0, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension);
					_ASSERTE(p_HtSiH);
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_1, n_id_0, p_HtSiH);
					// need the address of the unique *original* block, not of one of the many reduction temporaries
				}

				typedef fbs_ut::CCTSize2D<CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension> BlockSize;
				rp.template ReduceSingle<BlockSize>(t_key);
			} else {
				typename CReductionPlan::CLambdaReductor::TKey t_key;
				if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) // compile-time const
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_1, 0); // pointer ignored
				else {
					const double *p_HtSiH = r_lambda.p_GetBlock_Log(n_id_0, n_id_1, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension);
					_ASSERTE(p_HtSiH);
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_1, p_HtSiH);
					// need the address of the unique *original* block, not of one of the many reduction temporaries
				}

				typedef fbs_ut::CCTSize2D<CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension> BlockSize;
				rp.template ReduceSingle<BlockSize>(t_key);
			}
		}

		typename CReductionPlan::CLambdaReductor::TKey p_key_vert[n_vertex_num];
		if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) { // compile-time const
			p_key_vert[0] = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_0, 0);
			p_key_vert[1] = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_1, n_id_1, 0); // pointer ignored
		} else {
			const double *p_HtSiH_vert[2];
			if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0]) {
				p_HtSiH_vert[0] = r_lambda.p_GetBlock_Log(n_id_0, n_id_0, CVertexTraits<0>::n_dimension, CVertexTraits<0>::n_dimension);
				_ASSERTE(p_HtSiH_vert[0]);
			}
			if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1]) {
				p_HtSiH_vert[1] = r_lambda.p_GetBlock_Log(n_id_1, n_id_1, CVertexTraits<1>::n_dimension, CVertexTraits<1>::n_dimension);
				_ASSERTE(p_HtSiH_vert[1]);
			}
			// need the address of the unique *original* block, not of one of the many reduction temporaries
			// t_odo - perhaps it would be better to use (row, col) as unique index to the reduction, this lookup would then be avoided

			if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0])
				p_key_vert[0] = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_0, p_HtSiH_vert[0]);
			if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1])
				p_key_vert[1] = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_1, n_id_1, p_HtSiH_vert[1]);
		}
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0])
			rp.template ReduceSingle<typename CVertexTraits<0>::_TyMatrixSize>(p_key_vert[0]);
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1])
			rp.template ReduceSingle<typename CVertexTraits<1>::_TyMatrixSize>(p_key_vert[1]);
		// reduce off-diagonal Hessian and both diagonal Hessians
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
	 *	@brief calculates Hessian contributions
	 *
	 *	@note This only calculates the Hessians, either Reduce_Hessians_v2() or the
	 *		reduction plan needs to be used to fill lambda with values.
	 */
	inline void Calculate_Hessians_v2()
	{
		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
			t_jacobian1, v_expectation, v_error);
		// calculates the expectation and the jacobians

		Eigen::Matrix<double, CVertexTraits<0>::n_dimension, n_residual_dimension> t_H0_sigma_inv;
		double f_robust_weight;
		if(b_is_robust_edge) { // compile-time constant
			f_robust_weight = f_Get_RobustWeight(v_error);
			t_H0_sigma_inv = t_jacobian0.transpose() * m_t_sigma_inv * f_robust_weight;
		} else
			t_H0_sigma_inv = t_jacobian0.transpose() * m_t_sigma_inv;

		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH) {
			bool b_transpose_hessian;
			size_t n_dimension0 = CVertexTraits<0>::n_dimension;
			size_t n_dimension1 = CVertexTraits<1>::n_dimension;
			size_t n_id_0 = m_p_vertex_id[0];
			size_t n_id_1 = m_p_vertex_id[1]; // this is closer in cache
			_ASSERTE((n_id_0 > n_id_1) == (m_p_vertex0->n_Order() > m_p_vertex1->n_Order())); // if this triggers, then the edge has the vertices assigned in different order than the ids (vertex[0] is id[1] and vice versa)
			if((b_transpose_hessian = (n_id_0 > n_id_1))) {
				std::swap(n_id_0, n_id_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_id_0 < n_id_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			if(b_transpose_hessian) {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				t_HtSiH.noalias() = t_jacobian1.transpose() * t_H0_sigma_inv.transpose();
			} else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				t_HtSiH.noalias() = t_H0_sigma_inv * t_jacobian1;
			}
			// calculate the off-diagonal block
		}

		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0]) {
			Eigen::Map<typename CVertexTraits<0>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment> m_t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			if(hyperedge_detail::explicit_SymmetricProduct) // compile-time const
				m_t_HtSiH_vertex0/*.noalias()*/ = (t_H0_sigma_inv * t_jacobian0).template selfadjointView<Eigen::Upper>();
			else
				m_t_HtSiH_vertex0.noalias() = t_H0_sigma_inv * t_jacobian0;

			//_ASSERTE(m_t_HtSiH_vertex0 == m_t_HtSiH_vertex0.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<0>::_TyVectorAlign, CUberBlockMatrix::map_Alignment> m_t_right_hand_vertex0(m_p_RHS_vert[0]);
			if(b_is_robust_edge) // compile-time constant
				m_t_right_hand_vertex0.noalias() = t_H0_sigma_inv * v_error * f_robust_weight;
			else
				m_t_right_hand_vertex0.noalias() = t_H0_sigma_inv * v_error;
		}
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1]) {
			Eigen::Map<typename CVertexTraits<1>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment> m_t_HtSiH_vertex1(m_p_HtSiH_vert[1]);
			if(hyperedge_detail::explicit_SymmetricProduct) { // compile-time const
				if(b_is_robust_edge) // compile-time constant
					m_t_HtSiH_vertex1/*.noalias()*/ = (t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1 * f_robust_weight).template selfadjointView<Eigen::Upper>();
				else
					m_t_HtSiH_vertex1/*.noalias()*/ = (t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1).template selfadjointView<Eigen::Upper>();
			} else {
				if(b_is_robust_edge) // compile-time constant
					m_t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1 * f_robust_weight;
				else
					m_t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;
			}

			//_ASSERTE(m_t_HtSiH_vertex1 == m_t_HtSiH_vertex1.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<1>::_TyVectorAlign, CUberBlockMatrix::map_Alignment> m_t_right_hand_vertex1(m_p_RHS_vert[1]);
			if(b_is_robust_edge) // compile-time constant
				m_t_right_hand_vertex1.noalias() = t_jacobian1.transpose() * (m_t_sigma_inv * v_error) * f_robust_weight;
			else
				m_t_right_hand_vertex1.noalias() = t_jacobian1.transpose() * (m_t_sigma_inv * v_error);
		}
		// calculate the diagonal blocks and the right hand side vector contributions
	}

	/**
	 *	@brief calculates the gradient descent scaling factor
	 *	@param[in] r_v_g is the g vector from the dogleg paper (\f$-J^T \Sigma^{-1} r\f$)
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 */
	inline double f_Calculate_Steepest_Descent_Scale(const Eigen::VectorXd &r_v_g)
	{
		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
			t_jacobian1, v_expectation, v_error);
		// calculates the expectation and the jacobians // this is inefficient as it needs to re-evaluate the jacobians

		_TyVector v_Jg;
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0])
			v_Jg = t_jacobian0 * r_v_g.segment<CVertexTraits<0>::n_dimension>(m_p_vertex0->n_Order());
		else
			v_Jg.setZero();
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1])
			v_Jg += t_jacobian1 * r_v_g.segment<CVertexTraits<1>::n_dimension>(m_p_vertex1->n_Order());
		// calculate the right hand side vector contributions
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
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
			t_jacobian1, v_expectation, v_error);
		// calculates the expectation and the jacobians

		Eigen::Matrix<double, CVertexTraits<0>::n_dimension, n_residual_dimension> t_H0_sigma_inv =
			t_jacobian0.transpose() * m_t_sigma_inv;

		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH) {
			bool b_transpose_hessian;
			size_t n_dimension0 = CVertexTraits<0>::n_dimension;
			size_t n_dimension1 = CVertexTraits<1>::n_dimension;
			size_t n_id_0 = m_p_vertex_id[0];
			size_t n_id_1 = m_p_vertex_id[1]; // this is closer in cache
			_ASSERTE((n_id_0 > n_id_1) == (m_p_vertex0->n_Order() > m_p_vertex1->n_Order())); // if this triggers, then the edge has the vertices assigned in different order than the ids (vertex[0] is id[1] and vice versa)
			if((b_transpose_hessian = (n_id_0 > n_id_1))) {
				std::swap(n_id_0, n_id_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_id_0 < n_id_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			if(b_transpose_hessian) {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				t_HtSiH.noalias() = t_jacobian1.transpose() * t_H0_sigma_inv.transpose();
			} else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				t_HtSiH.noalias() = t_H0_sigma_inv * t_jacobian1;
			}
			// calculate the off-diagonal block
		}

		_TyVector v_Jg;
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0]) {
			Eigen::Map<typename CVertexTraits<0>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment> m_t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			m_t_HtSiH_vertex0.noalias() = t_H0_sigma_inv * t_jacobian0;
			Eigen::Map<typename CVertexTraits<0>::_TyVectorAlign, CUberBlockMatrix::map_Alignment> m_t_right_hand_vertex0(m_p_RHS_vert[0]);
			typename CVertexTraits<0>::_TyVectorAlign g = t_jacobian0.transpose() * (m_t_sigma_inv * v_error);
			m_t_right_hand_vertex0.noalias() = g;
			v_Jg = t_jacobian0 * g;
		} else
			v_Jg.setZero();
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1]) {
			Eigen::Map<typename CVertexTraits<1>::_TyMatrixAlign, CUberBlockMatrix::map_Alignment> m_t_HtSiH_vertex1(m_p_HtSiH_vert[1]);
			m_t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;
			Eigen::Map<typename CVertexTraits<1>::_TyVectorAlign, CUberBlockMatrix::map_Alignment> m_t_right_hand_vertex1(m_p_RHS_vert[1]);
			typename CVertexTraits<1>::_TyVectorAlign g = t_jacobian1.transpose() * (m_t_sigma_inv * v_error);
			m_t_right_hand_vertex1.noalias() = g;
			v_Jg += t_jacobian1 * g;
		}
		// calculate the diagonal blocks and the right hand side vector contributions

		return v_Jg.squaredNorm(); // need to sum up first, then take the norm
	}

#else // __LAMBDA_USE_V2_REDUCTION_PLAN

	inline bool Notify_HessianBlock_Conflict(double *p_block, CUberBlockMatrix &r_lambda)
	{
		if(m_p_HtSiH == p_block) {
			if(!m_p_HtSiH_reduce) {
				m_p_HtSiH = r_lambda.p_Get_DenseStorage(CVertexTraits<0>::n_dimension * CVertexTraits<1>::n_dimension);
				// get a temporary block

				memcpy(m_p_HtSiH, p_block, CVertexTraits<0>::n_dimension * CVertexTraits<1>::n_dimension * sizeof(double));
				// this off-diagonal Hessian might already have significant value, need to store it
			}
			// first time around this is called, alloc a different block to store the off-diagonal Hessian

			m_p_HtSiH_reduce = p_block;
			m_b_first_reductor = true; // i was the first

			return true; // acknowledge notify
		}
		return false; // not me
	}

	inline void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda)
	{
		if(m_p_vertex0->b_IsConstant() || m_p_vertex1->b_IsConstant())
			throw std::runtime_error("v1 reduction plan does not support const vertices"); // low priority todo

		m_p_vertex0->Add_ReferencingEdge(this);
		m_p_vertex1->Add_ReferencingEdge(this);
		// the vertices need to know this edge in order to calculate sum of Hessians on the diagonal of lambda // t_odo - should the lambda solver call this instead? might avoid duplicate records in the future // lambda solver is calling this only once in it's lifetime, no duplicates will occur

		_ASSERTE((m_p_vertex_id[0] > m_p_vertex_id[1]) ==
			(m_p_vertex0->n_Order() > m_p_vertex1->n_Order()) ||
			m_p_vertex0->b_IsConstant() || m_p_vertex1->b_IsConstant());
		// if this triggers, then the edge has the vertices assigned in a different
		// order than the ids (vertex[0] is id[1] and vice versa) consequently, the Hessians
		// will have correct shape in the matrix, but the data will be transposed
		// and you will get either not pos def / rubbish solutions

		bool b_uninit_block;
		size_t n_dimension0 = CVertexTraits<0>::n_dimension;
		size_t n_dimension1 = CVertexTraits<1>::n_dimension;
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
		size_t n_id_0 = m_p_vertex_id[0];
		size_t n_id_1 = m_p_vertex_id[1];
		if(n_id_0 > n_id_1) {
			std::swap(n_id_0, n_id_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_id_0 < n_id_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		m_p_HtSiH = r_lambda.p_GetBlock_Log_Alloc(n_id_0, n_id_1, n_dimension0, n_dimension1, b_uninit_block);
#else // __BASE_TYPES_USE_ID_ADDRESSING
		size_t n_order_0 = m_p_vertex0->n_Order();
		size_t n_order_1 = m_p_vertex1->n_Order();
		if(n_order_0 > n_order_1) {
			std::swap(n_order_0, n_order_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_order_0 < n_order_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		m_p_HtSiH = r_lambda.p_FindBlock_Alloc(n_order_0, n_order_1, n_dimension0, n_dimension1, b_uninit_block);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
		// find a block for Hessian above the diagonal, and with the right shape

		// t_odo - need a new function that finds a block, and says if it was there before, or not.
		// that could break, if the lambda was already allocated. does that ever happen?
		// note that m_p_vertex[]->Add_ReferencingEdge() is called here as well, and it is guaranteed
		// to only be called once. that brings some guarantees ...

		_ASSERTE(m_p_HtSiH);
		// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
		// and the edges come in some random order

		bool b_duplicate_block = !b_uninit_block; // was it there already?
		if(b_duplicate_block) {
			const std::vector<typename _TyVertex1::_TyBaseEdge*> &r_list =
				m_p_vertex1->r_ReferencingEdge_List(); // the second vertex has presumably less references, making the search faster; could select the smaller one manually
			for(size_t i = 0, n = r_list.size(); i < n; ++ i) {
				if(r_list[i]->Notify_HessianBlock_Conflict(m_p_HtSiH, r_lambda))
					break;
				_ASSERTE(i + 1 != n); // make sure we found it
			}
			// find the first reductor!

			m_b_first_reductor = false; // this is duplicate, there already was another
			m_p_HtSiH_reduce = m_p_HtSiH; // where to reduce
			m_p_HtSiH = r_lambda.p_Get_DenseStorage(n_dimension0 * n_dimension1); // a temporary
			// this edge is a duplicate edge; reduction of diagonal blocks is taken care of in the vertices.
			// reduction of off-diagonal blocks needs to be taken care of explicitly

			// need a new storage for the block data, so that Calculate_Hessians() has where to put it
			// could put it in the lambda matrix pool, without putting it in any column (a bit dirty, but ok)

			// finally, the reduction needs to be carried out somehow

			// each duplicate edge between two vertices can be processed by either of the vertices,
			// in CBaseVertexImpl::Calculate_Hessians(). there are some problems with load balancing, but
			// it should generally work. there is a problem with matrix size, as the vertices do not have
			// to be the same size, and each vertex does not know about the other vertex' dimension

			// it needs to be done in the edge, but potentially invoked by the vertex
			// to avoid conflicts, the reduction should be done by vertex with e.g. the larger id
			// this will require one more pointer in edge (for the orig block) and one more list in the vertex
			// otherwise, could have one more field in vertex, which would say where to reduce. that saves space
			// as those lists would be normally empty

			// what will happen in N-ary edges?
		} else {
			//m_b_first_reductor = false; // leave uninitialized, don't care
			m_p_HtSiH_reduce = 0; // do not reduce
		}
	}

	inline void Calculate_Hessians()
	{
		_ASSERTE(!b_is_robust_edge); // robust edges not supported here

		_ASSERTE(m_p_vertex0->b_IsReferencingEdge(this));
		_ASSERTE(m_p_vertex1->b_IsReferencingEdge(this));
		// makes sure this is amongst the referencing edges

		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
			t_jacobian1, v_expectation, v_error);
		// calculates the expectation and the jacobians

		bool b_transpose_hessian;
		size_t n_dimension0 = CVertexTraits<0>::n_dimension;
		size_t n_dimension1 = CVertexTraits<1>::n_dimension;
#if 1
		size_t n_id_0 = m_p_vertex_id[0];
		size_t n_id_1 = m_p_vertex_id[1]; // this is closer in cache
		_ASSERTE((n_id_0 > n_id_1) == (m_p_vertex0->n_Order() > m_p_vertex1->n_Order())); // if this triggers, then the edge has the vertices assigned in different order than the ids (vertex[0] is id[1] and vice versa)
		if((b_transpose_hessian = (n_id_0 > n_id_1))) {
			std::swap(n_id_0, n_id_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_id_0 < n_id_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)
#else // 1
		size_t n_order_0 = m_p_vertex0->n_Order();
		size_t n_order_1 = m_p_vertex1->n_Order();
		if((b_transpose_hessian = (n_order_0 > n_order_1))) {
			std::swap(n_order_0, n_order_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_order_0 < n_order_1);
#endif // 1
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		Eigen::Matrix<double, CVertexTraits<0>::n_dimension, n_residual_dimension> t_H0_sigma_inv =
			t_jacobian0.transpose() * m_t_sigma_inv;

		if(b_transpose_hessian) {
			Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
				CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
			// map the matrix above diagonal

			t_HtSiH.noalias() = t_jacobian1.transpose() * t_H0_sigma_inv.transpose();
		} else {
			Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
				CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
			// map the matrix above diagonal

			t_HtSiH.noalias() = t_H0_sigma_inv * t_jacobian1;
		}
		// calculate the matrix above diagonal

		m_t_HtSiH_vertex0.noalias() = t_H0_sigma_inv * t_jacobian0;
		m_t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;
		// calculate diagonal Hessian contributions

		m_t_right_hand_vertex0.noalias() = t_jacobian0.transpose() * (m_t_sigma_inv * v_error);
		m_t_right_hand_vertex1.noalias() = t_jacobian1.transpose() * (m_t_sigma_inv * v_error);
		// calculate right hand side vector contributions
	}

	inline std::pair<const double*, const double*> t_Get_LambdaEta_Contribution(const CBaseVertex *p_which)
	{
		if(m_p_HtSiH_reduce) {
			const CBaseVertex *p_min_vert = (m_p_vertex_id[0] < m_p_vertex_id[1])?
				(const CBaseVertex*)m_p_vertex0 : (const CBaseVertex*)m_p_vertex1; // this cast is ok, as the vertex is never accessed
			if(p_min_vert == p_which) {
				if(m_b_first_reductor) {
					Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
						CUberBlockMatrix::map_Alignment> t_HtSiH_reduce(m_p_HtSiH_reduce);
					t_HtSiH_reduce.setZero();
				}
				// don't zero it out at random !! zero it out at min vertex.

				/*if(n_id_0 > n_id_1) { // note that it does not matter, it is just elementwise add
					Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
						CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH), t_HtSiH_reduce(m_p_HtSiH_reduce);
					t_HtSiH_reduce.noalias() += t_HtSiH;
				} else*/ {
					Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
						CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH), t_HtSiH_reduce(m_p_HtSiH_reduce);
					t_HtSiH_reduce.noalias() += t_HtSiH;
				}
				// this is otherwise nicely ordered by the loop in CBaseVertexImpl::Calculate_Hessians()
			}
		}
		// take care of Hessian reduction, kind of silently, but it is guaranteed
		// that this is called for all the updated vertices

		// this is annoying. could we come up with some more general "reduction scheme" for calculating
		// lambda? it would be a simple block matrix where the edges / verts allocate duplicate blocks,
		// along with a list of reductions. there is a problem of FBS-ness, but could probably have
		// a separate list for each block size (which could be even better in the end)
		// t_odo - implement this sometimes. in the meantime, this works quite well.

		_ASSERTE(p_which == m_p_vertex0 || p_which == m_p_vertex1);
		return (p_which == m_p_vertex0)?
			std::make_pair(m_t_HtSiH_vertex0.data(), m_t_right_hand_vertex0.data()) :
			std::make_pair(m_t_HtSiH_vertex1.data(), m_t_right_hand_vertex1.data());
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
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[0]) {
			Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			for(size_t i = 0; i < CVertexTraits<0>::n_dimension; ++ i)
				f_max = std::max(f_max, t_HtSiH_vertex0(i, i));
		}
		if(hyperedge_detail::no_ConstVertices || m_p_HtSiH_vert[1]) {
			Eigen::Map<typename CVertexTraits<1>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[1]);
			for(size_t i = 0; i < CVertexTraits<1>::n_dimension; ++ i)
				f_max = std::max(f_max, t_HtSiH_vertex1(i, i));
		}
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
		for(size_t i = 0; i < CVertexTraits<0>::n_dimension; ++ i)
			f_max = std::max(f_max, m_t_HtSiH_vertex0(i, i));
		for(size_t i = 0; i < CVertexTraits<1>::n_dimension; ++ i)
			f_max = std::max(f_max, m_t_HtSiH_vertex1(i, i));
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
		return f_max;
	}

#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::Alloc_LBlocks
	 */
	inline void Alloc_LBlocks(CUberBlockMatrix &UNUSED(r_L)) const
	{
		// no vertex refs, that is already done in Alloc_HessianBlocks()
#if 0
		size_t n_dimension0 = CVertexTraits<0>::n_dimension;
		size_t n_dimension1 = CVertexTraits<1>::n_dimension;
		size_t n_order_0 = m_p_vertex0->n_Order();
		size_t n_order_1 = m_p_vertex1->n_Order();
		if(n_order_0 > n_order_1) {
			std::swap(n_order_0, n_order_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_order_0 < n_order_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		r_L.p_FindBlock(n_order_0, n_order_1, n_dimension0, n_dimension1, true, true); // pointer not remembered, will handle it differently
		// find a block for Hessian above the diagonal, and with the right shape // also set it to zero, L is updated additively
#endif // 0
		// edges should not allocate off-diagonal blocks, it will just screw the reordering and increase fill-in
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---

#if 0
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
	inline void Calculate_Omega_4122(CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const
	{
		//_ASSERTE(r_omega.b_Empty()); // should be empty // can't assert that here, other edges might have added something already

		if(m_p_vertex0->b_IsConstant() || m_p_vertex1->b_IsConstant())
			throw std::runtime_error("fL solver can't cope with const vertices yet (is buggy)");

		bool b_transpose_hessian;
		size_t n_dimension0 = CVertexTraits<0>::n_dimension;
		size_t n_dimension1 = CVertexTraits<1>::n_dimension;

#if 0 // this does not work, omega tends to be very sparse, might need to create some empty columns / rows and p_GetBlock_Log() does not support that
		size_t n_id_0 = m_p_vertex_id[0];
		size_t n_id_1 = m_p_vertex_id[1];
		if((b_transpose_hessian = (n_id_0 > n_id_1))) {
			std::swap(n_id_0, n_id_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_id_0 < n_id_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		if(n_id_0 < n_min_vertex_id) // note this might be superficial // could make this an assert
			return;
		// this edge doesn't have any Hessians inside omega

		double *p_v0 = r_omega.p_GetBlock_Log(n_id_0 - n_min_vertex_id,
			n_id_0 - n_min_vertex_id, n_dimension0, n_dimension0, true, true);
		double *p_v1 = r_omega.p_GetBlock_Log(n_id_1 - n_min_vertex_id,
			n_id_1 - n_min_vertex_id, n_dimension1, n_dimension1, true, true);

		double *p_edge = r_omega.p_GetBlock_Log(n_id_0 - n_min_vertex_id,
			n_id_1 - n_min_vertex_id, n_dimension0, n_dimension1, true, false); // doesn't need to be initialized, there's only one
		// must come last, n_id_1 might not exist in r_omega before the vertices are added
		// this does not work anyway, omega tends to be very sparse, might need to create some empty columns / rows and p_GetBlock_Log() does not support that
#else // 0
		size_t n_order_0 = m_p_vertex0->n_Order();
		size_t n_order_1 = m_p_vertex1->n_Order();
		if((b_transpose_hessian = (n_order_0 > n_order_1))) {
			std::swap(n_order_0, n_order_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_order_0 < n_order_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		_ASSERTE(n_order_0 >= n_min_vertex_order); // note this might be superficial // t_odo - make this an assert
		//	return;
		// this edge doesn't have any Hessians inside omega

		double *p_edge = r_omega.p_FindBlock(n_order_0 - n_min_vertex_order,
			n_order_1 - n_min_vertex_order, n_dimension0, n_dimension1, true, true); // needs to be initialized, in case there are duplicate edges; totally not true -> // doesn't need to be initialized, there's only one
		double *p_v0 = r_omega.p_FindBlock(n_order_0 - n_min_vertex_order,
			n_order_0 - n_min_vertex_order, n_dimension0, n_dimension0, true, true);
		double *p_v1 = r_omega.p_FindBlock(n_order_1 - n_min_vertex_order,
			n_order_1 - n_min_vertex_order, n_dimension1, n_dimension1, true, true);
#endif // 0
		// alloc and initialize / find existing blocks for all the Hessians, above the diagonal only

		bool b_recalculate = false;
		// recalculate from scratch, or use values already calculated by the lambda solver?

		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		if(b_recalculate) {
			_TyVector v_expectation, v_error;
			((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
				t_jacobian1, v_expectation, v_error);
		}
		// calculates the expectation and the jacobians

		Eigen::Matrix<double, CVertexTraits<0>::n_dimension, n_residual_dimension> t_H0_sigma_inv;
		if(b_recalculate)
			t_H0_sigma_inv = t_jacobian0.transpose() * m_t_sigma_inv;

		if(b_transpose_hessian) {
			Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
				CUberBlockMatrix::map_Alignment> t_HtSiH(p_edge); // t_odo - support aligned if the uberblockmatrix is aligned!
			// map the matrix above diagonal

			if(b_recalculate)
				t_HtSiH.noalias() += t_jacobian1.transpose() * t_H0_sigma_inv.transpose();
			else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH_lambda(m_p_HtSiH);
				t_HtSiH += t_HtSiH_lambda;
			}
		} else {
			Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
				CUberBlockMatrix::map_Alignment> t_HtSiH(p_edge); // t_odo - support aligned if the uberblockmatrix is aligned!
			// map the matrix above diagonal

			if(b_recalculate)
				t_HtSiH.noalias() += t_H0_sigma_inv * t_jacobian1;
			else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH_lambda(m_p_HtSiH);
				t_HtSiH += t_HtSiH_lambda;
			}
		}
		// calculate the matrix above diagonal

		//if(CVertexTraits<0>::n_dimension != CVertexTraits<1>::n_dimension)
		//	b_transpose_hessian = !b_transpose_hessian;
		// cause a bug deliberately to test the block bounds checks in block matrix

		Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
			CUberBlockMatrix::map_Alignment> t_HtSiH_v0((b_transpose_hessian)? p_v1 : p_v0);
		Eigen::Map<typename CVertexTraits<1>::_TyMatrix,
			CUberBlockMatrix::map_Alignment> t_HtSiH_v1((b_transpose_hessian)? p_v0 : p_v1);
		if(b_recalculate) {
			t_HtSiH_v0.noalias() += t_H0_sigma_inv * t_jacobian0;
			t_HtSiH_v1.noalias() += t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;
		} else {
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
			Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
			Eigen::Map<typename CVertexTraits<1>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[1]);
			t_HtSiH_v0 += t_HtSiH_vertex0;
			t_HtSiH_v1 += t_HtSiH_vertex1;
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
			t_HtSiH_v0 += m_t_HtSiH_vertex0;
			t_HtSiH_v1 += m_t_HtSiH_vertex1;
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
		}
		// calculate vertex Hessian contributions (note the increments)
	}
#endif // 0

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
	inline void Calculate_Omega/*_HeadRevision*/(CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const
	{
		//_ASSERTE(r_omega.b_Empty()); // should be empty // can't assert that here, other edges might have added something already

		bool b_transpose_hessian;
		size_t n_dimension0 = CVertexTraits<0>::n_dimension;
		size_t n_dimension1 = CVertexTraits<1>::n_dimension;
		const bool b_optimize0 = !m_p_vertex0->b_IsConstant();
		const bool b_optimize1 = !m_p_vertex1->b_IsConstant();

#if 0 // this does not work, omega tends to be very sparse, might need to create some empty columns / rows and p_GetBlock_Log() does not support that
		double *p_v0 = r_omega.p_GetBlock_Log(n_id_0 - n_min_vertex_id,
			n_id_0 - n_min_vertex_id, n_dimension0, n_dimension0, true, true);
		double *p_v1 = r_omega.p_GetBlock_Log(n_id_1 - n_min_vertex_id,
			n_id_1 - n_min_vertex_id, n_dimension1, n_dimension1, true, true);

		size_t n_id_0 = m_p_vertex_id[0];
		size_t n_id_1 = m_p_vertex_id[1];
		if((b_transpose_hessian = (n_id_0 > n_id_1))) {
			std::swap(n_id_0, n_id_1);
			std::swap(n_dimension0, n_dimension1);
		}
		// make sure the order is sorted (if swapping, will have to transpose the result,
		// but we will deal with that laters)

		_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
		_ASSERTE(n_id_0 < n_id_1);
		// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

		_ASSERTE(n_id_0 >= n_min_vertex_id); // if this triggers then this edge doesn't have any Hessians inside omega

		double *p_edge = r_omega.p_GetBlock_Log(n_id_0 - n_min_vertex_id,
			n_id_1 - n_min_vertex_id, n_dimension0, n_dimension1, true, false); // doesn't need to be initialized, there's only one
		// must come last, n_id_1 might not exist in r_omega before the vertices are added
		// this does not work anyway, omega tends to be very sparse, might need to create some empty columns / rows and p_GetBlock_Log() does not support that
#else // 0
		size_t n_order_0 = m_p_vertex0->n_Order();
		size_t n_order_1 = m_p_vertex1->n_Order();
		const size_t n_order_0_orig = n_order_0;
		const size_t n_order_1_orig = n_order_1;
		double *p_edge;
		if(b_optimize0 && b_optimize1) { // only need the off-diagonal block if both vertices are optimized
			if((b_transpose_hessian = (n_order_0 > n_order_1))) {
				std::swap(n_order_0, n_order_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_order_0 < n_order_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			_ASSERTE(n_order_0 >= n_min_vertex_order); // note this might be superficial // t_odo - make this an assert
			//	return;
			// this edge doesn't have any Hessians inside omega

			p_edge = r_omega.p_FindBlock(n_order_0 - n_min_vertex_order,
				n_order_1 - n_min_vertex_order, n_dimension0, n_dimension1, true, true); // needs to be initialized, in case there are duplicate edges; totally not true -> // doesn't need to be initialized, there's only one
		} else
			p_edge = 0;
		// allocate the off-diagonal block

		double *p_v0 = (b_optimize0)? r_omega.p_FindBlock(n_order_0_orig - n_min_vertex_order,
			n_order_0_orig - n_min_vertex_order, CVertexTraits<0>::n_dimension,
			CVertexTraits<0>::n_dimension, true, true) : 0;
		double *p_v1 = (b_optimize1)? r_omega.p_FindBlock(n_order_1_orig - n_min_vertex_order,
			n_order_1_orig - n_min_vertex_order, CVertexTraits<1>::n_dimension,
			CVertexTraits<1>::n_dimension, true, true) : 0;
		// allocate the diagonal blocks (doint that after the off-diagonal reduces the movements in the block matrix column vectors)
#endif // 0
		// alloc and initialize / find existing blocks for all the Hessians, above the diagonal only

		bool b_recalculate = false;
		// recalculate from scratch, or use values already calculated by the lambda solver?

		typename CVertexTraits<0>::_TyJacobianMatrix t_jacobian0;
		typename CVertexTraits<1>::_TyJacobianMatrix t_jacobian1;
		if(b_recalculate) {
			_TyVector v_expectation, v_error;
			((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian0,
				t_jacobian1, v_expectation, v_error);
		}
		// calculates the expectation and the jacobians

		Eigen::Matrix<double, CVertexTraits<0>::n_dimension, n_residual_dimension> t_H0_sigma_inv;
		if(b_recalculate)
			t_H0_sigma_inv = t_jacobian0.transpose() * m_t_sigma_inv;

		if(b_optimize0 && b_optimize1) { // only need the off-diagonal block if both vertices are optimized
			if(b_transpose_hessian) {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(p_edge); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				if(b_recalculate)
					t_HtSiH.noalias() += t_jacobian1.transpose() * t_H0_sigma_inv.transpose();
				else {
					Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension, CVertexTraits<0>::n_dimension>,
						CUberBlockMatrix::map_Alignment> t_HtSiH_lambda(m_p_HtSiH);
					t_HtSiH += t_HtSiH_lambda;
				}
			} else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(p_edge); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				if(b_recalculate)
					t_HtSiH.noalias() += t_H0_sigma_inv * t_jacobian1;
				else {
					Eigen::Map<Eigen::Matrix<double, CVertexTraits<0>::n_dimension, CVertexTraits<1>::n_dimension>,
						CUberBlockMatrix::map_Alignment> t_HtSiH_lambda(m_p_HtSiH);
					t_HtSiH += t_HtSiH_lambda;
				}
			}
		}
		// calculate the matrix above diagonal

		//if(CVertexTraits<0>::n_dimension != CVertexTraits<1>::n_dimension)
		//	b_transpose_hessian = !b_transpose_hessian;
		// cause a bug deliberately to test the block bounds checks in block matrix

		if(b_recalculate) {
			if(b_optimize0) {
				Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
					CUberBlockMatrix::map_Alignment> t_HtSiH_v0(p_v0);
				t_HtSiH_v0.noalias() += t_H0_sigma_inv * t_jacobian0;
			}
			if(b_optimize1) {
				Eigen::Map<typename CVertexTraits<1>::_TyMatrix,
					CUberBlockMatrix::map_Alignment> t_HtSiH_v1(p_v1);
				t_HtSiH_v1.noalias() += t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;
			}
		} else {
			if(b_optimize0) {
				Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
					CUberBlockMatrix::map_Alignment> t_HtSiH_v0(p_v0);
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
				Eigen::Map<typename CVertexTraits<0>::_TyMatrix,
					CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[0]);
				t_HtSiH_v0 += t_HtSiH_vertex0;
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
				t_HtSiH_v0 += m_t_HtSiH_vertex0;
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
			}
			if(b_optimize1) {
				Eigen::Map<typename CVertexTraits<1>::_TyMatrix,
					CUberBlockMatrix::map_Alignment> t_HtSiH_v1(p_v1);
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
				Eigen::Map<typename CVertexTraits<1>::_TyMatrix,
					CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[1]);
				t_HtSiH_v1 += t_HtSiH_vertex1;
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
				t_HtSiH_v1 += m_t_HtSiH_vertex1;
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
			}
		}
		// calculate diagonal Hessian contributions (note the increments)
	}
};

/**
 *	@brief implementation of solver required functions for a generic edge type (specialization for binary edges)
 *
 *	@tparam CDerivedEdge is the name of derived edge class
 *	@tparam CVertexType0 is type of the first vertiex
 *	@tparam CVertexType1 is type of the second vertiex
 *	@tparam _n_residual_dimension is residual vector dimension
 *	@tparam _n_storage_dimension is state vector storage dimension (or -1 if the same as _n_residual_dimension)
 *
 *	@deprecated This name is deprecated; use CBaseEdgeImpl instead.
 */
template <class CDerivedEdge, class CVertexType0, class CVertexType1,
	int _n_residual_dimension, int _n_storage_dimension /*= -1*/>
class CSEBaseEdgeImpl<CDerivedEdge, MakeTypelist2(CVertexType0, CVertexType1),
	_n_residual_dimension, _n_storage_dimension> : public CBaseEdgeImpl<CDerivedEdge,
	MakeTypelist2(CVertexType0, CVertexType1), _n_residual_dimension, _n_storage_dimension> {
private:
	typedef CBaseEdgeImpl<CDerivedEdge, MakeTypelist2(CVertexType0, CVertexType1),
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
	 *	@brief binary edge constructor
	 *
	 *	@param[in] n_vertex0_id is id of the first vertex
	 *	@param[in] n_vertex1_id is id of the second vertex
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *
	 *	@note This fills the structure, except for the vertex pointers.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 */
	inline CSEBaseEdgeImpl(size_t n_vertex0_id, size_t n_vertex1_id,
		const typename _TyNotDeprecated::_TyStorageVector &r_v_measurement,
		const typename _TyNotDeprecated::_TyMatrix &r_t_sigma_inv)
		:_TyNotDeprecated(n_vertex0_id, n_vertex1_id, r_v_measurement, r_t_sigma_inv)
	{}
};

/** @} */ // end of group

#endif // __BASE_GRAPH_PRIMITIVE_TYPES_BINARY_EDGE_IMPL_SPECIALIZATION_INCLUDED
