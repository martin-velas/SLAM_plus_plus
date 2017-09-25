/*
								+----------------------------------+
								|                                  |
								|    ***  Base graph types  ***    |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2012  |
								|                                  |
								|           BaseTypes.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __BASE_GRAPH_PRIMITIVE_TYPES_INCLUDED
#define __BASE_GRAPH_PRIMITIVE_TYPES_INCLUDED

/**
 *	@file include/slam/BaseTypes.h
 *	@brief base graph primitive types
 *	@author -tHE SWINe-
 *	@date 2012-09-21
 */

#include "slam/FlatSystem.h"
#include "slam/ParseLoop.h" // todo this should not be here
#include "slam/Tuple.h"
#include "slam/Self.h"

/** \addtogroup graph
 *	@{
 */

#if !defined(__SE_TYPES_SUPPORT_A_SOLVERS) && \
	!defined(__SE_TYPES_SUPPORT_LAMBDA_SOLVERS) && \
	!defined(__SE_TYPES_SUPPORT_L_SOLVERS)
#error "error: slam/BaseTypes.h is included and no solvers are supported"
// if this triggers, you are probably including BaseTypes.h before ConfigSolvers.h or own config
// this means that the types will not support any solver type (and if trying to use a solver, likely
// there will be a lot of errors like "CBaseEdge has no member named Calculate_Hessians")
#endif // !__SE_TYPES_SUPPORT_A_SOLVERS && !__SE_TYPES_SUPPORT_LAMBDA_SOLVERS && !__SE_TYPES_SUPPORT_L_SOLVERS

// looking for #define __BASE_TYPES_USE_ID_ADDRESSING? it is in slam/FlatSystem.h ...

#ifdef __FLAT_SYSTEM_ALIGNED_MEMORY // defined in slam/FlatSystem.h
/**
 *	@def __BASE_TYPES_USE_ALIGNED_MATRICES
 *	@brief if defined, vectors in vertices and edges will be aligned
 */
#define __BASE_TYPES_USE_ALIGNED_MATRICES

/**
 *	@def __GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *	@brief evaluates to <tt>EIGEN_MAKE_ALIGNED_OPERATOR_NEW</tt>
 */
#define __GRAPH_TYPES_ALIGN_OPERATOR_NEW EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#else // __FLAT_SYSTEM_ALIGNED_MEMORY
/**
 *	@def __GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *	@brief evaluates to an empty expression 
 */
#define __GRAPH_TYPES_ALIGN_OPERATOR_NEW
#endif // __FLAT_SYSTEM_ALIGNED_MEMORY
// decide whether to align types

#if defined(__BASE_TYPES_USE_ALIGNED_MATRICES)
/**
 *	@def __SE2_TYPES_USE_ALIGNED_VECTORS
 *	@brief if defined, the base types will use aligned vectors and matrices
 *	@deprecated This is deprecated in favor of \ref __BASE_TYPES_USE_ALIGNED_MATRICES
 *		and is provided for backward compatibility only.
 */
#define __SE2_TYPES_USE_ALIGNED_VECTORS
#elif defined(__SE2_TYPES_USE_ALIGNED_VECTORS)
#undef __SE2_TYPES_USE_ALIGNED_VECTORS
#endif // __BASE_TYPES_USE_ALIGNED_MATRICES

/**
 *	@def __SE2_TYPES_ALIGN_OPERATOR_NEW
 *	@brief if defined, the base types will have aligned operator new
 *	@deprecated This is deprecated in favor of \ref __GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *		and is provided for backward compatibility only.
 */
#define __SE2_TYPES_ALIGN_OPERATOR_NEW __GRAPH_TYPES_ALIGN_OPERATOR_NEW

/**
 *	@def __SE3_TYPES_ALIGN_OPERATOR_NEW
 *	@brief if defined, the base types will have aligned operator new
 *	@deprecated This is deprecated in favor of \ref __GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *		and is provided for backward compatibility only.
 */
#define __SE3_TYPES_ALIGN_OPERATOR_NEW __GRAPH_TYPES_ALIGN_OPERATOR_NEW

/**
 *	@def __BA_TYPES_ALIGN_OPERATOR_NEW
 *	@brief if defined, the base types will have aligned operator new
 *	@deprecated This is deprecated in favor of \ref __GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *		and is provided for backward compatibility only.
 */
#define __BA_TYPES_ALIGN_OPERATOR_NEW __GRAPH_TYPES_ALIGN_OPERATOR_NEW

/**
 *	@def __SIM3_TYPES_ALIGN_OPERATOR_NEW
 *	@brief if defined, the base types will have aligned operator new
 *	@deprecated This is deprecated in favor of \ref __GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *		and is provided for backward compatibility only.
 */
#define __SIM3_TYPES_ALIGN_OPERATOR_NEW __GRAPH_TYPES_ALIGN_OPERATOR_NEW

/**
 *	@brief namespace with helper objects for edges
 */
namespace base_edge_detail {

/**
 *	@brief vertex initialization functor
 *	Just clears the vertex to null.
 */
template <class CVertex>
class CInitializeNullVertex {
public:
	/**
	 *	@brief function operator
	 *	@return Returns the value of the vertex being initialized.
	 */
	inline operator CVertex() const
	{
		typename CVertex::_TyVector v_null;
		v_null.setZero();
		return CVertex(v_null);
	}
};

/**
 *	@brief vertex initialization functor
 *	Throws exception because the edge cannot meaningfully initialize the vertex.
 */
template <class CVertex>
class CInitializeVertex_Disallow {
public:
	/**
	 *	@brief function operator
	 *	@return Returns the value of the vertex being initialized.
	 *	@note This function always throws std::runtime_error.
	 */
	inline operator CVertex() const // throw(std::runtime_error)
	{
		throw std::runtime_error("implicit vertex initialization not allowed");
		return CInitializeNullVertex<CVertex>(); // just to return something
	}
};

// this does not work as the type is incomplete at the time of making the stuff
/*template<class CEdgeType, class CArgType>
class CIsRobustEdge {
protected:
    template <class TType>
	static TType &Instantiate(); // undefined

	static const CArgType &MakeArg(); // undefined

	struct TNo { int p_pad[1]; };
	struct TYes { int p_pad[2]; }; // todo - two no make a yes

	template <size_t n_dummy_size>
	class CValueToType;

	template <class CSignature, CSignature p_pointer>
	class CSignatureCheck;

    template <class T>
	static TYes DeclCheck(CValueToType<sizeof(&T::f_RobustWeight)> *p_dummy_arg);
	//static TYes DeclCheck(CSignatureCheck<double(T::*)(const CArgType&), &T::f_RobustWeight> *p_dummy_arg);

    template <class T>
	static TNo DeclCheck(...);

	template <class T>
	static TYes SigCheck(CValueToType<sizeof(Instantiate<T>().f_RobustWeight(MakeArg()))> *p_dummy_arg); // this fails on g++; we'll get that error anyway upon it being called though

    template <class T>
	static TNo SigCheck(...);

public:
	enum {
		b_declared = sizeof(DeclCheck<CEdgeType>(0)) == sizeof(TYes),
		b_good_signature = true,//b_declared && sizeof(SigCheck<CEdgeType>(0)) == sizeof(TYes),
		b_incorrect_signature = b_declared && !b_good_signature,
		b_result = b_declared && b_good_signature, 
	};
};*/

} // ~base_edge_detail

/**
 *	@brief namespace with helper objects for hyperedges
 */
namespace hyperedge_detail {

/**
 *	@brief configuration stored as enum
 */
enum {
#ifdef __BASE_TYPES_ALLOW_CONST_VERTICES
	no_ConstVertices = false, /**< @brief no const vertices flag @note This is used to optimize away some const-ness checks which the compiler cannot optimize by itself. */
#else // __BASE_TYPES_ALLOW_CONST_VERTICES
	no_ConstVertices = true, /**< @brief no const vertices flag @note This is used to optimize away some const-ness checks which the compiler cannot optimize by itself. */
#endif // __BASE_TYPES_ALLOW_CONST_VERTICES
	explicit_SymmetricProduct = true /**< @brief use explicitly symmetric product when calculating diagonal blocks of the Hessian */
};

/**
 *	@brief static assertion helper
 *	@brief b_expression is expression being asserted
 */
template <const bool b_expression>
class CStaticAssert {
public:
	typedef void NOT_STRICTLY_UPPER_TRIANGULAR; /**< @brief static assertion tag; internal error */
};

/**
 *	@brief static assertion helper (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

/**
 *	@brief calculates offset in densely packed buffer storing
 *		a strictly upper triangular matrix (i.e. excluding the diagonal)
 *
 *	@param[in] r is zero-based index of row, in elements
 *	@param[in] c is zero-based index of column, in elements
 *	@param[in] n is number of columns of the matrix, in elements
 */
template <const int r, const int c, const int n>
struct CStrictlyTriangularOffset {
	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		b_is_strictly_upper = c > r, /**< @brief strictly upper triangular flag (if set, the given coordinates point to strictly upper triangular portion of the matrix) */
		n_result = (((r * (n + n - 3 - r)) >> 1) + c - 1) /**< @brief the resulting index */
	};

	typedef typename CStaticAssert<b_is_strictly_upper>::NOT_STRICTLY_UPPER_TRIANGULAR CAssert0; /**< @brief static assertion */ // t_odo - do a proper static assert here
};

/**
 *	@brief type transformation functor; makes a pointer to a given type
 *	@tparam T is input type
 */
template <class T>
struct CMakePointer {
	typedef T *_TyResult; /**< @brief result type */
};

/**
 *	@brief type transformation functor; makes a const pointer to a given type
 *	@tparam T is input type
 */
template <class T>
struct CMakeConstPointer {
	typedef const T *_TyResult; /**< @brief result type */
};

/**
 *	@brief type transformation functor; always returns size_t
 *	@tparam T is input type
 */
template <class T>
struct CMakeSize {
	typedef size_t _TyResult; /**< @brief result type */ // note that T is unused
};

/**
 *	@brief context for Jacobian matrices
 *	@tparam n_residual_dimension is residual dimension of an edge (shared dimension of its Jacobians)
 */
template <const int n_residual_dimension>
struct CJacobianContext {
	/**
	 *	@brief vertex type to jacobian matrix type conversion
	 *	@tparam CVertex is a vertex type
	 */
	template <class CVertex>
	struct CMakeJacobian {
		typedef Eigen::Matrix<double, n_residual_dimension, CVertex::n_dimension> _TyResult; /**< @brief Jacobian matrix type */
	};
};

/**
 *	@brief vertex type to Hessian matrix type conversion
 *	@tparam CVertex is a vertex type
 */
template <class CVertex>
struct CMakeHessian {
	typedef Eigen::Matrix<double, CVertex::n_dimension, CVertex::n_dimension> _TyResult; /**< @brief Hessian matrix type */
};

/**
 *	@brief vertex type to right hand side vector type conversion
 *	@tparam CVertex is a vertex type
 */
template <class CVertex>
struct CMakeRHS {
	typedef Eigen::Matrix<double, CVertex::n_dimension, 1> _TyResult; /**< @brief RHS vector type */
};

} // ~hyperedge_detail

/**
 *	@brief base edge class
 */
class CBaseEdge : public CEdgeInterface {
public:
	/**
	 *	@brief edge options, stored as enum
	 */
	enum {
		Plain = 0, /**< @brief plain edge type */
		Robust = 1 /**< @brief edge with robust function */
		// note that those need to be powers of two
	};

	struct null_initialize_vertices_tag {}; /**< @brief tag-scheduling type for automatic vertex initialization */
	struct explicitly_initialized_vertices_tag {}; /**< @brief tag-scheduling type for explicit vertex initialization */

	static const null_initialize_vertices_tag null_initialize_vertices; /**< @brief tag-scheduling value for automatic vertex initialization */
	static const explicitly_initialized_vertices_tag explicitly_initialized_vertices; /**< @brief tag-scheduling value for explicit vertex initialization */

protected:
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
	size_t m_n_id; /**< @brief edge id (block row in a matrix where the edge block is inserted) */
#endif // __BASE_TYPES_USE_ID_ADDRESSING
	size_t m_n_order; /**< @brief edge order (row in a matrix where the edge block is inserted) */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::n_Order
	 */
	inline size_t n_Order() const
	{
		return m_n_order;
	}

	/**
	 *	@copydoc base_iface::CEdgeFacade::Set_Order
	 */
	inline void Set_Order(size_t n_first_element_index)
	{
		m_n_order = n_first_element_index;
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::n_Id
	 */
	inline size_t n_Id() const
	{
		return m_n_id;
	}

	/**
	 *	@copydoc base_iface::CEdgeFacade::Set_Id
	 */
	inline void Set_Id(size_t n_id)
	{
		m_n_id = n_id;
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING
};

/**
 *	@brief base vertex class
 */
class CBaseVertex : public CVertexInterface {
protected:
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
	size_t m_n_id; /**< @brief vertex id (block column in a matrix where the vertex block is inseted) */
#endif // __BASE_TYPES_USE_ID_ADDRESSING
	size_t m_n_order; /**< @brief vertex order (column in a matrix where the vertex block is inseted) */

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief gets vertex order
	 *	@return Returns vertex order (column in a matrix where the vertex block is inseted).
	 */
	inline size_t n_Order() const
	{
		return m_n_order;
	}

	/**
	 *	@brief determines whether this vertex is a constant vertex
	 *	@return Returns true if this vertex is a constant vertex, otherwise returns false.
	 */
	inline bool b_IsConstant() const
	{
#ifdef __BASE_TYPES_ALLOW_CONST_VERTICES
		return m_n_order == size_t(-1);
#else // __BASE_TYPES_ALLOW_CONST_VERTICES
		_ASSERTE(m_n_order != size_t(-1)); // make sure we are not lying here
		return false;
#endif // __BASE_TYPES_ALLOW_CONST_VERTICES
	}

	/**
	 *	@brief sets vertex order
	 *	@param[in] n_first_element_index is vertex order
	 *		(column in a matrix where the vertex block is inseted).
	 */
	inline void Set_Order(size_t n_first_element_index)
	{
		m_n_order = n_first_element_index;
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets vertex id
	 *	@return Returns vertex order (block column in a matrix where the vertex block is inseted).
	 */
	inline size_t n_Id() const
	{
		return m_n_id;
	}

	/**
	 *	@brief sets vertex id
	 *	@param[in] n_id is vertex id
	 *		(block column in a matrix where the vertex block is inseted).
	 */
	inline void Set_Id(size_t n_id)
	{
		m_n_id = n_id;
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING
};

typedef CBaseEdge CSEBaseEdge; /**< @brief base edge for SE types @deprecated Provided only for backward compatibility, use CBaseEdge instead. */
typedef CBaseVertex CSEBaseVertex; /**< @brief base vertex for SE types @deprecated Provided only for backward compatibility, use CBaseVertex instead. */
// for backward compatibility

/**
 *	@brief implementation of solver required functions for a generic vertex type
 *
 *	@tparam CDerivedVertex is name of the derived vertex class
 *	@tparam _n_dimension is vertex dimension (number of DOFs)
 */
template <class CDerivedVertex, int _n_dimension>
class CBaseVertexImpl : public CBaseVertex {
public:
	/**
	 *	@brief copy of template parameters
	 */
	enum {
		n_dimension = _n_dimension, /**< @brief vertex dimension (number of DOFs) */
#ifdef __BASE_TYPES_USE_ALIGNED_MATRICES
		n_matrix_alignment = Eigen::AutoAlign | Eigen::ColMajor
#else // __BASE_TYPES_USE_ALIGNED_MATRICES
		n_matrix_alignment = Eigen::DontAlign | Eigen::ColMajor
#endif // __BASE_TYPES_USE_ALIGNED_MATRICES
	};

	typedef Eigen::Matrix<double, n_dimension, 1> _TyVector; /**< @brief state vector type */
	typedef Eigen::Matrix<double, n_dimension, n_dimension> _TyMatrix; /**< @brief a compatible matrix type */
	typedef Eigen::Matrix<double, n_dimension, 1, n_matrix_alignment> _TyVectorAlign; /**< @brief state vector storage type (have to use unaligned version due to the use in pools) */
	typedef Eigen::Matrix<double, n_dimension, n_dimension, n_matrix_alignment> _TyMatrixAlign; /**< @brief a compatible matrix type */

protected:
	_TyVectorAlign m_v_state; /**< @brief state vector */

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific members ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	double *m_p_HtSiH; /**< @brief pointer to diagonal block in the Hessian matrix */
	_TyVectorAlign m_v_right_hand_side; /**< @brief right-hand side vector */

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific members ---

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; has no effect
	 */
	inline CBaseVertexImpl()
	{}

	/**
	 *	@brief constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CBaseVertexImpl(const _TyVector &r_v_state)
		:m_v_state(r_v_state)
	{}

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns reference to vertex state vector.
	 */
	inline _TyVectorAlign &r_v_State()
	{
		return m_v_state;
	}

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns const reference to vertex state vector.
	 */
	inline const _TyVectorAlign &r_v_State() const
	{
		return m_v_state;
	}

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns a map of vertex state vector.
	 */
	inline Eigen::Map<const Eigen::VectorXd> v_State() const
	{
		_ASSERTE(m_v_state.rows() == n_dimension);
		return Eigen::Map<const Eigen::VectorXd>(m_v_state.data(), n_dimension);
	}

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns a map of vertex state vector.
	 */
	inline Eigen::Map<Eigen::VectorXd> v_State()
	{
		_ASSERTE(m_v_state.rows() == n_dimension);
		return Eigen::Map<Eigen::VectorXd>(m_v_state.data(), n_dimension);
	}

	/**
	 *	@copydoc base_iface::CConstVertexFacade::n_Dimension()
	 */
	inline size_t n_Dimension() const
	{
		return n_dimension;
	}

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	inline void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda)
	{
		if(!b_IsConstant()) {
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			m_p_HtSiH = r_lambda.p_GetBlock_Log(m_n_id, m_n_id, n_dimension, n_dimension, true, false);
#else // __BASE_TYPES_USE_ID_ADDRESSING
			m_p_HtSiH = r_lambda.p_FindBlock(m_n_order, m_n_order, n_dimension, n_dimension, true, false);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			// find a block for Hessian on the diagonal (edges can't do it, would have conflicts)

			_ASSERTE(m_p_HtSiH);
			// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
			// and the edges come in some random order
		} else
			m_p_HtSiH = 0; // this is a const vertex
	}

	inline void Calculate_Hessians()
	{
		if(!b_IsConstant()) {
			Eigen::Map<_TyMatrix, CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH); // t_odo - support aligned if the uberblockmatrix is aligned!
			t_HtSiH.setZero(); // !!
			m_v_right_hand_side.setZero(); // !!
			_ASSERTE(!m_edge_list.empty());
			for(size_t i = 0, n = m_edge_list.size(); i < n; ++ i) {
				std::pair<const double*, const double*> t_contrib = m_edge_list[i]->t_Get_LambdaEta_Contribution(this);
				Eigen::Map<_TyMatrix, n_matrix_alignment> t_lambda_contrib((double*)t_contrib.first); // t_odo - support aligned if the member matrices are aligned!
				t_HtSiH += t_lambda_contrib;
				Eigen::Map<_TyVector, n_matrix_alignment> v_eta_contrib((double*)t_contrib.second);  // t_odo - support aligned if the member matrices are aligned!
				m_v_right_hand_side += v_eta_contrib;
			}
			// calculate running sum (can't parallelize, in 10k or 100k there is
			// only about 6 edges / vertex, and that is the top density you get)
		}
	}

	inline void Get_RightHandSide_Vector(Eigen::VectorXd &r_v_eta) const
	{
		if(!b_IsConstant())
			r_v_eta.segment<n_dimension>(m_n_order) = m_v_right_hand_side;
	}

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc base_iface::CConstVertexFacade::Alloc_LBlocks
	 */
	inline void Alloc_LBlocks(CUberBlockMatrix &r_L) const
	{
		if(!b_IsConstant()) {
			double *UNUSED(p_block_addr);
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			p_block_addr = r_L.p_GetBlock_Log(m_n_id, m_n_id, n_dimension, n_dimension, true, true); // don't care about the pointer, it is handled differently
#else // __BASE_TYPES_USE_ID_ADDRESSING
			p_block_addr = r_L.p_FindBlock(m_n_order, m_n_order, n_dimension, n_dimension, true, true); // don't care about the pointer, it is handled differently
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			// find a block for Hessian on the diagonal (edges can't do it, would have conflicts) // also clear it to zero! L is updated additively

			_ASSERTE(p_block_addr);
			// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
			// and the edges come in some random order
		}
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---

	/**
	 *	@copydoc base_iface::CVertexFacade::SwapState()
	 */
	inline void SwapState(Eigen::VectorXd &r_v_x)
	{
		if(!b_IsConstant())
			m_v_state.swap(r_v_x.segment<n_dimension>(m_n_order)); // swap the numbers
	}

	/**
	 *	@copydoc base_iface::CConstVertexFacade::SaveState()
	 */
	inline void SaveState(Eigen::VectorXd &r_v_x) const
	{
		if(!b_IsConstant())
			r_v_x.segment<n_dimension>(m_n_order) = m_v_state; // save the state vector
	}

	/**
	 *	@copydoc base_iface::CVertexFacade::LoadState()
	 */
	inline void LoadState(const Eigen::VectorXd &r_v_x)
	{
		if(!b_IsConstant())
			m_v_state = r_v_x.segment<n_dimension>(m_n_order); // restore the state vector
	}
};

/**
 *	@brief implementation of solver required functions for a generic vertex type
 *
 *	@tparam CDerivedVertex is name of the derived vertex class
 *	@tparam _n_dimension is vertex dimension (number of DOFs)
 *
 *	@deprecated This name is deprecated; use CBaseVertexImpl instead.
 */
template <class CDerivedVertex, int _n_dimension>
class CSEBaseVertexImpl : public CBaseVertexImpl<CDerivedVertex, _n_dimension> {
private:
	typedef CBaseVertexImpl<CDerivedVertex, _n_dimension> _TyNotDeprecated; /**< @brief name of the parent type that is not deprecated */
	// this is not intended to be used by the derived classes because CBaseVertexImpl does not have such type

public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@copydoc CBaseVertexImpl::CBaseVertexImpl()
	 */
	inline CSEBaseVertexImpl()
		:_TyNotDeprecated()
	{}

	/**
	 *	@copydoc CBaseVertexImpl::CBaseVertexImpl(const _TyVector&)
	 */
	inline CSEBaseVertexImpl(const typename _TyNotDeprecated::_TyVector &r_v_state)
		:_TyNotDeprecated(r_v_state)
	{}
};

/**
 *	@brief implementation of solver required functions for a generic edge type
 *
 *	@tparam CDerivedEdge is the name of derived edge class
 *	@tparam CVertexTypeList is list of types of the vertices
 *	@tparam _n_residual_dimension is residual vector dimension
 *	@tparam _n_storage_dimension is state vector storage dimension (or -1 if the same as _n_residual_dimension)
 *	@tparam _n_options is a bitfield of options (use e.g. CBaseEdge::Robust)
 */
template <class CDerivedEdge, class CVertexTypeList, int _n_residual_dimension,
	int _n_storage_dimension = -1, unsigned int _n_options = CBaseEdge::Plain>
class CBaseEdgeImpl : public CBaseEdge {
public:
	typedef CVertexTypeList _TyVertices; /**< @brief list of vertex types */
	typedef typename CTransformTypelist<CVertexTypeList, hyperedge_detail::CMakePointer>::_TyResult _TyVertexPointers; /**< @brief list of pointers to vertex types */
	typedef typename CTransformTypelist<CVertexTypeList, hyperedge_detail::CMakeConstPointer>::_TyResult _TyVertexConstPointers; /**< @brief list of const pointers to vertex types */
	typedef typename CTransformTypelist<CVertexTypeList, hyperedge_detail::CMakeSize>::_TyResult _TyVertexIndices; /**< @brief list of size_t of the same length as vertex type list */
	typedef CTuple<_TyVertexIndices> _TyVertexIndexTuple; /**< @brief tuple of vertex indices */
	typedef CTuple<_TyVertexPointers> _TyVertexPointerTuple; /**< @brief tuple of pointers to the vertices */
	typedef CTuple<_TyVertexConstPointers> _TyVertexConstPointerTuple; /**< @brief tuple of const pointers to the vertices */
	typedef typename CTypelistItemAt<_TyVertices, 0>::_TyResult _TyVertex0; /**< @brief name of the first vertex class */
	typedef typename CTypelistItemAt<_TyVertices, 1>::_TyResult _TyVertex1; /**< @brief name of the second vertex class */
	typedef typename CTypelistItemAt<_TyVertices, 2>::_TyResult _TyVertex2; /**< @brief name of the third vertex class */
	// note that there may be more than three vertices, but we are unable to predict how many;
	// just use "typename CTypelistItemAt<_TyVertices, N>::_TyResult" where N is vertex number

	/**
	 *	@brief copy of template parameters
	 */
	enum {
		n_vertex_num = CTypelistLength<CVertexTypeList>::n_result, /**< @brief number of vertices */
		n_measurement_dimension = _n_residual_dimension, /**< @brief measurement vector dimension (edge dimension) @deprecated This is slightly unclear, use n_residual_dimension and n_storage_dimension instead. */
		n_residual_dimension = _n_residual_dimension, /**< @brief residual vector dimension */
		n_storage_dimension = (_n_storage_dimension == -1)? _n_residual_dimension : _n_storage_dimension, /**< @brief edge state storage dimension */
#ifdef __BASE_TYPES_USE_ALIGNED_MATRICES
		n_matrix_alignment = Eigen::AutoAlign | Eigen::ColMajor, /**< @brief Eigen matrix alignment flags */
#else // __BASE_TYPES_USE_ALIGNED_MATRICES
		n_matrix_alignment = Eigen::DontAlign | Eigen::ColMajor, /**< @brief Eigen matrix alignment flags */
#endif // __BASE_TYPES_USE_ALIGNED_MATRICES
		n_offdiag_block_num = (n_vertex_num * n_vertex_num - n_vertex_num) / 2, /**< @brief the number of off-diag blocks */
		// the number of off-diag blocks rises with O(n^2), it is every-to-every, but only upper diag
		// (1: 0, 2: 1, 3: 3, 4: 6)
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
		typedef Eigen::Matrix<double, n_residual_dimension, n_dimension, Eigen::DontAlign> _TyJacobianMatrixU; /**< @brief unaligned Jacobian matrix type */
	};

	/**
	 *	@brief vertex type to jacobian matrix type conversion
	 *	@tparam CVertex is a vertex type
	 */
	template <class CVertex>
	struct CMakeJacobian {
		typedef Eigen::Matrix<double, n_residual_dimension, CVertex::n_dimension> _TyResult; /**< @brief Jacobian matrix type */
	};
	// needs to be here, g++ has problems parsing hyperedge_detail::CJacobianContext

	typedef typename CTransformTypelist<CVertexTypeList,
		/*hyperedge_detail::CJacobianContext<n_residual_dimension>::*/CMakeJacobian>::_TyResult _TyVertexJacobians; /**< @brief list of types of vertex Jacobians */
	typedef CTuple<_TyVertexJacobians> _TyJacobianTuple; /**< @brief tuple of vertex Jacobians */

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
		//base_edge_detail::CIsRobustEdge<CBaseEdgeImpl<CDerivedEdge, CVertexTypeList, _n_residual_dimension, _n_storage_dimension>, _TyVector>::b_result
	};

protected:
	size_t m_p_vertex_id[n_vertex_num]; /**< @brief ids of referenced vertices */
	_TyVertexPointerTuple m_vertex_ptr; /**< @brief pointers to the referenced vertices */
	_TyStorageVectorAlign m_v_measurement; /**< @brief the measurement */
	_TyMatrixAlign m_t_sigma_inv; /**< @brief information matrix */

private:
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific members ---

	_TyMatrixAlign m_t_square_root_sigma_inv_upper; /**< @brief the R matrix (upper diagonal) = transpose(chol(t_sigma_inv)) */
	_TyVectorAlign m_v_error; /**< @brief error vector (needs to be filled explicitly by calling p_error_function) */
	double *m_p_RH[n_vertex_num]; /**< @brief blocks of memory where the R * H matrices are stored */

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific members ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific members ---

	double *m_p_HtSiH[n_offdiag_block_num]; /**< @brief blocks of memory where Ht * inv(Sigma) * H matrices are stored (the ones above the diagonal) */
	// t_odo - there will need to be more of them

#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN

	double *m_p_HtSiH_vert[n_vertex_num];
	double *m_p_RHS_vert[n_vertex_num];
	// this is used only by the new lambda reduction

#else // __LAMBDA_USE_V2_REDUCTION_PLAN

	double *m_p_HtSiH_reduce[n_offdiag_block_num]; /**< @brief blocks of memory where Ht * inv(Sigma) * H matrices are stored in the matrix (should there be conflicts between duplicate edges, this is the target of reduction) */
	bool m_p_first_reductor[n_offdiag_block_num]; /**< @brief reduction flags */

	typedef typename CTransformTypelist<CVertexTypeList, hyperedge_detail::CMakeHessian>::_TyResult _TyVertexHessians; /**< @brief list of types of vertex Hessians */
	typedef CTuple<_TyVertexHessians> _TyVertexHessianTuple; /**< @brief tuple of vertex Hessians */

	typedef typename CTransformTypelist<CVertexTypeList, hyperedge_detail::CMakeRHS>::_TyResult _TyVertexResiduals; /**< @brief list of types of vertex residuals */
	typedef CTuple<_TyVertexResiduals> _TyVertexResidualTuple; /**< @brief tuple of vertex residuals */

	_TyVertexHessianTuple m_t_HtSiH_tuple; /**< @brief blocks of memory where the Ht * inv(Sigma) * H matrices are stored (the diagonal blocks) */
	_TyVertexResidualTuple m_t_right_hand_side_tuple; /**< @brief blocks of memory where right-hand side vectors are stored (the per-vertex errors) */
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

protected:
	/**
	 *	@brief copies vertex ids from a tuple to an array
	 *
	 *	This copies the vertex ids from a tuple to m_p_vertex_id array.
	 *
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in] r_vertex_id_tuple is a tuple containing the vertex ids
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex>
	void _CopyVertexIds(const _TyVertexIndexTuple &r_vertex_id_tuple,
		fbs_ut::CCTSize<n_vertex> UNUSED(tag))
	{
		m_p_vertex_id[n_vertex] = r_vertex_id_tuple.template Get<n_vertex>();
		_CopyVertexIds(r_vertex_id_tuple, fbs_ut::CCTSize<n_vertex + 1>());
	}

	/**
	 *	@brief copies vertex ids from a tuple to an array (specialization for the end of the tuple)
	 *
	 *	@param[in] r_vertex_id_tuple is a tuple containing the vertex ids (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	void _CopyVertexIds(const _TyVertexIndexTuple &UNUSED(r_vertex_id_tuple),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

	/**
	 *	@brief copies vertex pointers from a tuple to an array
	 *
	 *	This copies the vertex pointers from a tuple to m_vertex_ptr array.
	 *
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in] r_vertex_ptr_tuple is a tuple containing the vertex pointers
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex>
	void _CopyVertexPtrs(const _TyVertexConstPointerTuple &r_vertex_ptr_tuple,
		fbs_ut::CCTSize<n_vertex> UNUSED(tag))
	{
		typedef typename _TyVertexPointerTuple::template TElem<n_vertex>::Type _TyPtr; // non-const pointer type
		m_vertex_ptr.template Get<n_vertex> = const_cast<_TyPtr>(r_vertex_ptr_tuple.template Get<n_vertex>()); // cast & copy
		_CopyVertexPtrs(r_vertex_ptr_tuple, fbs_ut::CCTSize<n_vertex + 1>());
	}

	/**
	 *	@brief copies vertex pointers from a tuple to an array (specialization for the end of the tuple)
	 *
	 *	@param[in] r_vertex_ptr_tuple is a tuple containing the vertex pointers (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	void _CopyVertexPtrs(const _TyVertexConstPointerTuple &UNUSED(r_vertex_ptr_tuple),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

	/**
	 *	@brief initializes vertex pointers, adds vertices which do not exist to the system
	 *		or throws exceptions if implicit initialization is not allowed for that vertex
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *	@tparam CSkipInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not to be initialized so that the derived edge can initialize them
	 *		(indices in the edge, for e.g. binary edges those are 0 and 1; use CTypelistEnd for none)
	 *	@tparam CDisallowInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not allowed to be initialized implicitly; if they are not in the system yet,
	 *		a std::runtime_error exception is thrown (indices in the edge, for e.g. binary edges
	 *		those are 0 and 1; use CTypelistEnd for none)
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 *	@param[in] t_list_skip is type list of vertex indices (in the edge) for which
	 *		the initialization should be disabled so that the derived edge can perform it (value unused)
	 *	@param[in] t_list_disallow is type list of vertex indices (in the edge) for which
	 *		the initialization is not allowed - if the vertex is not explicitly inizialized
	 *		prior to calling this function (value unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@note This function throws std::bad_alloc if there is not enough memory for the vertices,
	 *		and throws std::runtime_error if a vertex in the list of vertices not allowed to
	 *		initialize implicitly was not explicitly initialized before calling this function.
	 */
	template <class CSystem, class CSkipInitIndexList,
		class CDisallowInitIndexList, const int n_vertex>
	void _InitializeVertexPtrs(CSystem &r_system,
		CSkipInitIndexList UNUSED(t_list_skip),
		CDisallowInitIndexList UNUSED(t_list_disallow),
		fbs_ut::CCTSize<n_vertex> UNUSED(t_vertex_id)) // throw(std::bad_alloc, std::runtime_error)
	{
		typedef typename CVertexTraits<n_vertex>::_TyVertex _TyVertex; // vertex type
		enum {
			b_do_init = !CFindTypelistItem<CSkipInitIndexList, fbs_ut::CCTSize<n_vertex> >::b_result, // initialize this vertex?
			b_disallow_init = CFindTypelistItem<CDisallowInitIndexList, fbs_ut::CCTSize<n_vertex> >::b_result // implicit initialization not allowed?
		};
		typedef typename CChooseType<CInitializeVertex_Disallow<_TyVertex>,
			CInitializeNullVertex<_TyVertex>, b_disallow_init>::_TyResult _TyInitializer; // choose vertex initializer

		if(b_do_init) { // only if not skipping, so that we can skip over disallowed vertices as well
			m_vertex_ptr.template Get<n_vertex>() = &r_system.template
				r_Get_Vertex<_TyVertex>(m_p_vertex_id[n_vertex], _TyInitializer());
		}

		_InitializeVertexPtrs(r_system, t_list_skip, t_list_disallow, fbs_ut::CCTSize<n_vertex + 1>());
		// recurse
	}

	/**
	 *	@brief initializes vertex pointers (specialization for the one past the last vertex)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *	@tparam CSkipInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not to be initialized so that the derived edge can initialize them
	 *		(indices in the edge, for e.g. binary edges those are 0 and 1; use CTypelistEnd for none)
	 *	@tparam CDisallowInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not allowed to be initialized implicitly; if they are not in the system yet,
	 *		a std::runtime_error exception is thrown (indices in the edge, for e.g. binary edges
	 *		those are 0 and 1; use CTypelistEnd for none)
	 *
	 *	@param[in] r_system is reference to system (unused)
	 *	@param[in] t_list_skip is type list of vertex indices (unused)
	 *	@param[in] t_list_disallow is type list of vertex indices (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused)
	 */
	template <class CSystem, class CSkipInitIndexList,
		class CDisallowInitIndexList>
	void _InitializeVertexPtrs(CSystem &UNUSED(r_system),
		CSkipInitIndexList UNUSED(t_list_skip),
		CDisallowInitIndexList UNUSED(t_list_disallow),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(t_vertex_id))
	{}

public:
	/**
	 *	@brief constructor
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
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
	inline CBaseEdgeImpl(_TyVertexIndexTuple vertex_id_tuple,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		_CopyVertexIds(vertex_id_tuple, fbs_ut::CCTSize<0>()); // todo - make a loop, or a tuple to array binding; will need to support assignment of different types of tuples (like assignment of a tuple of ints to a tuple of references to ints)
		// get references to the vertices, initialize the vertices, if necessary

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief constructor with implicit vertex initialization (to null)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *	@tparam CSkipInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not to be initialized so that the derived edge can initialize them
	 *		(indices in the edge, for e.g. binary edges those are 0 and 1; use CTypelistEnd for none)
	 *	@tparam CDisallowInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not allowed to be initialized implicitly; if they are not in the system yet,
	 *		a std::runtime_error exception is thrown (indices in the edge, for e.g. binary edges
	 *		those are 0 and 1; use CTypelistEnd for none)
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in,out] r_system is reference to system (used to query or insert edge vertices)
	 *	@param[in] t_list_skip is type list of vertex indices (in the edge) for which
	 *		the initialization should be disabled so that the derived edge can perform it (value unused)
	 *	@param[in] t_list_disallow is type list of vertex indices (in the edge) for which
	 *		the initialization is not allowed - an exception is thrown if the vertex is not
	 *		explicitly inizialized prior to calling this function (value unused)
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::bad_alloc if there is not enough memory for the vertices,
	 *		and throws std::runtime_error if a vertex in CDisallowInitIndexList and not in
	 *		CSkipInitIndexList was not explicitly initialized before calling this function.
	 *	@note CSkipInitIndexList takes precedence over CDisallowInitIndexList.
	 */
	template <class CSystem, class CSkipInitIndexList, class CDisallowInitIndexList>
	inline CBaseEdgeImpl(_TyVertexIndexTuple vertex_id_tuple,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		null_initialize_vertices_tag UNUSED(t_tag), CSystem &r_system,
		CSkipInitIndexList UNUSED(t_list_skip),
		CDisallowInitIndexList UNUSED(t_list_disallow)) // throw(std::bad_alloc, std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		_CopyVertexIds(vertex_id_tuple, fbs_ut::CCTSize<0>()); // todo - make a loop, or a tuple to array binding; will need to support assignment of different types of tuples (like assignment of a tuple of ints to a tuple of references to ints)
		// get references to the vertices, initialize the vertices, if necessary

		_InitializeVertexPtrs(r_system, t_list_skip, t_list_disallow, fbs_ut::CCTSize<0>());
		// initialize vertex pointers

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief constructor with implicit vertex initialization (to null; overload without the disallow list)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *	@tparam CSkipInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not to be initialized so that the derived edge can initialize them
	 *		(indices in the edge, for e.g. binary edges those are 0 and 1; use CTypelistEnd for none)
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in,out] r_system is reference to system (used to query or insert edge vertices)
	 *	@param[in] t_list_skip is type list of vertex indices (in the edge) for which
	 *		the initialization should be disabled so that the derived edge can perform it (value unused)
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::bad_alloc if there is not enough memory for the vertices.
	 */
	template <class CSystem, class CSkipInitIndexList>
	inline CBaseEdgeImpl(_TyVertexIndexTuple vertex_id_tuple,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		null_initialize_vertices_tag UNUSED(t_tag), CSystem &r_system,
		CSkipInitIndexList UNUSED(t_list_skip)) // throw(std::bad_alloc)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		_CopyVertexIds(vertex_id_tuple, fbs_ut::CCTSize<0>()); // todo - make a loop, or a tuple to array binding; will need to support assignment of different types of tuples (like assignment of a tuple of ints to a tuple of references to ints)
		// get references to the vertices, initialize the vertices, if necessary

		_InitializeVertexPtrs(r_system, t_list_skip, CTypelistEnd(), fbs_ut::CCTSize<0>());
		// initialize vertex pointers

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief constructor with implicit vertex initialization (to null; overload without the disallow and skip list)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in,out] r_system is reference to system (used to query or insert edge vertices)
	 *
	 *	@note This fills the structure, including the vertex pointers.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::bad_alloc if there is not enough memory for the vertices.
	 */
	template <class CSystem>
	inline CBaseEdgeImpl(_TyVertexIndexTuple vertex_id_tuple,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		null_initialize_vertices_tag UNUSED(t_tag), CSystem &r_system) // throw(std::bad_alloc)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		_CopyVertexIds(vertex_id_tuple, fbs_ut::CCTSize<0>()); // todo - make a loop, or a tuple to array binding; will need to support assignment of different types of tuples (like assignment of a tuple of ints to a tuple of references to ints)
		// get references to the vertices, initialize the vertices, if necessary

		_InitializeVertexPtrs(r_system, CTypelistEnd(), CTypelistEnd(), fbs_ut::CCTSize<0>());
		// initialize vertex pointers

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief constructor for explicit vertex initialization
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *	@tparam CSkipInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not to be initialized so that the derived edge can initialize them
	 *		(indices in the edge, for e.g. binary edges those are 0 and 1; use CTypelistEnd for none)
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in] r_system is reference to system (used to query edge vertices)
	 *	@param[in] t_list_skip is type list of vertex indices (in the edge) for which
	 *		the initialization should be disabled so that the derived edge can perform it (value unused)
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::runtime_error if a vertex not in CSkipInitIndexList
	 *		was not explicitly initialized before calling this function.
	 */
	template <class CSystem, class CSkipInitIndexList>
	inline CBaseEdgeImpl(_TyVertexIndexTuple vertex_id_tuple,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		explicitly_initialized_vertices_tag UNUSED(t_tag), CSystem &r_system,
		CSkipInitIndexList UNUSED(t_list_skip)) // throw(std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		_CopyVertexIds(vertex_id_tuple, fbs_ut::CCTSize<0>()); // todo - make a loop, or a tuple to array binding; will need to support assignment of different types of tuples (like assignment of a tuple of ints to a tuple of references to ints)
		// get references to the vertices, initialize the vertices, if necessary

		typedef typename fbs_ut::CTypelistIOTA<n_vertex_num>::_TyResult _TyAllVertsList;
		// do not allow initialization of any vertex, except for those specified in t_list_skip

		_InitializeVertexPtrs(r_system, t_list_skip, _TyAllVertsList(), fbs_ut::CCTSize<0>());
		// initialize vertex pointers

		// note that errors, expectations and jacobian matrices are not cleared
	}

	/**
	 *	@brief constructor for explicit vertex initialization (overload without the skip list)
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *	@tparam CSkipInitIndexList is type list of indices of vertices (as fbs_ut::CCTSize) which
	 *		are not to be initialized so that the derived edge can initialize them
	 *		(indices in the edge, for e.g. binary edges those are 0 and 1; use CTypelistEnd for none)
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
	 *	@param[in] r_v_measurement is the measurement vector
	 *	@param[in] r_t_sigma_inv is inverse sigma matrix
	 *	@param[in] t_tag is used for tag scheduling, specifies this constructor (value unused)
	 *	@param[in] r_system is reference to system (used to query edge vertices)
	 *	@param[in] t_list_skip is type list of vertex indices (in the edge) for which
	 *		the initialization should be disabled so that the derived edge can perform it (value unused)
	 *
	 *	@note This fills the structure, including the vertex pointers, except those indicated
	 *		in CSkipInitIndexList.
	 *	@note With thunk tables, the derived classes don't need to check for vertex dimensions
	 *		being really what they expect (type checking performed by the vertex pool).
	 *	@note The r_t_sigma_inv is inverse sigma matrix, which is *not* square-rooted.
	 *		This hasn't changed since the previous releases, but the documentation
	 *		in parser was misleading back then.
	 *	@note This function throws std::runtime_error if any of the vertices
	 *		were not explicitly initialized before calling this function.
	 */
	template <class CSystem>
	inline CBaseEdgeImpl(_TyVertexIndexTuple vertex_id_tuple,
		const _TyStorageVector &r_v_measurement, const _TyMatrix &r_t_sigma_inv,
		explicitly_initialized_vertices_tag UNUSED(t_tag), CSystem &r_system) // throw(std::runtime_error)
		:m_v_measurement(r_v_measurement), m_t_sigma_inv(r_t_sigma_inv)
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
		, m_t_square_root_sigma_inv_upper(r_t_sigma_inv.llt().matrixU()) // calculate R
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
	{
		_CopyVertexIds(vertex_id_tuple, fbs_ut::CCTSize<0>()); // todo - make a loop, or a tuple to array binding; will need to support assignment of different types of tuples (like assignment of a tuple of ints to a tuple of references to ints)
		// get references to the vertices, initialize the vertices, if necessary

		typedef typename fbs_ut::CTypelistIOTA<n_vertex_num>::_TyResult _TyAllVertsList;
		// do not allow initialization of any vertex, except for those specified in t_list_skip

		_InitializeVertexPtrs(r_system, CTypelistEnd(), _TyAllVertsList(), fbs_ut::CCTSize<0>());
		// initialize vertex pointers

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
	 *	@param[out] r_t_jacobian_tuple is jacobian tuple, one for each vertex
	 *	@param[out] r_v_expectation is expectation
	 *	@param[out] r_v_error is error
	 *	@param[in] t_vertex_ptr_tuple is tuple of const pointers to each vertex
	 *	@param[in] r_v_measurement is measurement
	 *
	 *	@note In case the derived edge has extended state, it should implement
	 *		a custom version of this function.
	 */
	static inline void Calculate_Jacobians( // t_odo - support n-ary edges (need a different interface for that - one using tuples)
		_TyJacobianTuple &r_t_jacobian_tuple,
		_TyVector &r_v_expectation, _TyVector &r_v_error,
		_TyVertexConstPointerTuple t_vertex_ptr_tuple,
		const _TyStorageVector &r_v_measurement)
	{
		CDerivedEdge dummy; // relies on having default constructor (most edges should)
		dummy._CopyVertexPtrs(t_vertex_ptr_tuple, fbs_ut::CCTSize<0>()); // copy the pointers to the vertices
		dummy.m_v_measurement = r_v_measurement;
		((const CDerivedEdge&)dummy).Calculate_Jacobians_Expectation_Error(r_t_jacobian_tuple, r_v_expectation, r_v_error);
		// Calculate_Jacobians_Expectation_Error in CDerivedEdge should have been
		// static and it would be simple, but now we can't force the users to rewrite
		// all their code. meh.

		// this will not be modified for robust edges, we're just
		// after the jacobians here and not the sigma or weight
	}

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

protected:
	/**
	 *	@brief allocates Jacobian blocks
	 *
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in,out] r_A is reference to the A matrix
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex>
	inline void _Alloc_JacobianBlocks(CUberBlockMatrix &r_A, fbs_ut::CCTSize<n_vertex> UNUSED(tag))
	{
		if(!m_vertex_ptr.template Get<n_vertex>()->b_IsConstant()) { // if the vertex is not constant
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			m_p_RH[n_vertex] = r_A.p_GetBlock_Log(m_n_id + 1, m_p_vertex_id[n_vertex],
				n_residual_dimension, CVertexTraits<n_vertex>::n_dimension, true, false);
#else // __BASE_TYPES_USE_ID_ADDRESSING
			m_p_RH[n_vertex] = r_A.p_FindBlock(m_n_order, m_vertex_ptr.template Get<n_vertex>()->n_Order(),
				n_residual_dimension, CVertexTraits<n_vertex>::n_dimension, true, false);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			// just place blocks at (edge order, vertex order)
		}

		_Alloc_JacobianBlocks(r_A, fbs_ut::CCTSize<n_vertex + 1>());
		// loop by recursion
	}

	/**
	 *	@brief allocates Jacobian blocks (specialization for the end of the tuple)
	 *
	 *	@param[in] r_A is reference to the A matrix (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	inline void _Alloc_JacobianBlocks(CUberBlockMatrix &r_A, fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

public:
	/**
	 *	@copydoc base_iface::CEdgeFacade::Alloc_JacobianBlocks
	 */
	inline void Alloc_JacobianBlocks(CUberBlockMatrix &r_A)
	{
		_Alloc_JacobianBlocks(r_A, fbs_ut::CCTSize<0>());
#ifdef _DEBUG
#ifndef __BASE_TYPES_ALLOW_CONST_VERTICES
		for(size_t i = 0; i < n_vertex_num; ++ i)
			_ASSERTE(m_p_RH[i]);
		// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
		// and the edges come in some random order
#else // !__BASE_TYPES_ALLOW_CONST_VERTICES
		for(size_t i = 0; i < n_vertex_num; ++ i) {
			if(m_p_RH[i])
				break;
			_ASSERTE(i + 1 != n_vertex_num);
			// if this fails then all vertices in this edge are constant. do not add this edge to the system in the first place
		}
#endif // !__BASE_TYPES_ALLOW_CONST_VERTICES
#endif // _DEBUG
	}

protected:
	/**
	 *	@brief multiplies Jacobians by square root of sigma inverse to get values of the Jacobian blocks
	 *
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in] r_t_jacobian_tuple is tuple containging the vertex Jacobians
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex>
	inline void _Calculate_Jacobians(const _TyJacobianTuple &r_t_jacobian_tuple,
		fbs_ut::CCTSize<n_vertex> UNUSED(tag))
	{
		if(m_p_RH[n_vertex]) { // may be null if const vertices are allowed
			Eigen::Map<typename CVertexTraits<n_vertex>::_TyJacobianMatrix,
				CUberBlockMatrix::map_Alignment> t_RH(m_p_RH[n_vertex]);
			// map Jacobian matrix

			t_RH = m_t_square_root_sigma_inv_upper * r_t_jacobian_tuple.template Get<n_vertex>();
			// recalculate RH (transpose cholesky of sigma times the jacobian)
			// note that this references the A block matrix
		}

		_Calculate_Jacobians(r_t_jacobian_tuple, fbs_ut::CCTSize<n_vertex + 1>());
		// loop
	}

	/**
	 *	@brief multiplies Jacobians by square root of sigma inverse to get values of the
	 *		Jacobian blocks (recursion termination specialization)
	 *
	 *	@param[in] r_t_jacobian_tuple is tuple containging the vertex Jacobians (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	inline void _Calculate_Jacobians(const _TyJacobianTuple &UNUSED(r_t_jacobian_tuple),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

	/**
	 *	@brief multiplies Jacobians by square root of sigma inverse to get values of the Jacobian blocks
	 *
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in] r_t_jacobian_tuple is tuple containging the vertex Jacobians
	 *	@param[in] f_weight_sqrt is square-rooted edge weight
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex>
	inline void _Calculate_Jacobians(const _TyJacobianTuple &r_t_jacobian_tuple,
		double f_weight_sqrt, fbs_ut::CCTSize<n_vertex> UNUSED(tag))
	{
		if(m_p_RH[n_vertex]) { // may be null if const vertices are allowed
			Eigen::Map<typename CVertexTraits<n_vertex>::_TyJacobianMatrix,
				CUberBlockMatrix::map_Alignment> t_RH(m_p_RH[n_vertex]);
			// map Jacobian matrix

			t_RH = f_weight_sqrt * m_t_square_root_sigma_inv_upper * r_t_jacobian_tuple.template Get<n_vertex>();
			// recalculate RH (transpose cholesky of sigma times the jacobian)
			// note that this references the A block matrix
		}

		_Calculate_Jacobians(r_t_jacobian_tuple, f_weight_sqrt, fbs_ut::CCTSize<n_vertex + 1>());
		// loop
	}

	/**
	 *	@brief multiplies Jacobians by square root of sigma inverse to get values of the
	 *		Jacobian blocks (recursion termination specialization)
	 *
	 *	@param[in] r_t_jacobian_tuple is tuple containging the vertex Jacobians (unused)
	 *	@param[in] f_weight_sqrt is square-rooted edge weight (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	inline void _Calculate_Jacobians(const _TyJacobianTuple &UNUSED(r_t_jacobian_tuple),
		double UNUSED(f_weight_sqrt), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

	inline void _Calculate_Jacobians_ChooseRobust(const _TyJacobianTuple &t_jacobians,
		const _TyVector &v_error, fbs_ut::CCTSize<0> UNUSED(t_robust_tag))
	{
		m_v_error = v_error;
		_Calculate_Jacobians(t_jacobians, fbs_ut::CCTSize<0>());
	}

	inline void _Calculate_Jacobians_ChooseRobust(const _TyJacobianTuple &t_jacobians,
		const _TyVector &v_error, fbs_ut::CCTSize<1> UNUSED(t_robust_tag))
	{
		double f_robust_weight = sqrt(((CDerivedEdge*)this)->f_RobustWeight(v_error));
		m_v_error = v_error * f_robust_weight;
		_Calculate_Jacobians(t_jacobians, f_robust_weight, fbs_ut::CCTSize<0>());
	}

	// if there was also a need for explicit weight for edge switching, there could be another version

public:
	/**
	 *	@copydoc base_iface::CEdgeFacade::Calculate_Jacobians
	 */
	inline void Calculate_Jacobians()
	{
		// easily implemented using a loop
		_TyJacobianTuple t_jacobian_tuple;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobian_tuple,
			v_expectation, v_error);
		// calculates the expectation, error and the jacobians (implemented by the edge)

		_Calculate_Jacobians_ChooseRobust(t_jacobian_tuple, v_error, fbs_ut::CCTSize<(b_is_robust_edge)? 1 : 0>());
		// per-vertex processing
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

protected:
	/**
	 *	@brief allocates off-diagonal Hessian blocks (inner loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *	@param[in,out] r_rp is reference to the reduction plan
	 *	@param[in] tag0 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *	@param[in] tag1 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex1, const int n_vertex0, class CReductionPlan>
	inline void _Alloc_HessianBlocks_v2_Inner(CUberBlockMatrix &r_lambda, CReductionPlan &r_rp,
		fbs_ut::CCTSize<n_vertex1> UNUSED(tag0), fbs_ut::CCTSize<n_vertex0> tag1)
	{
		// we need to allocate a block between n_vertex1 and n_vertex0

		_ASSERTE(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()); // make sure that n_vertex0 is not constant

		enum {
			n_block = hyperedge_detail::CStrictlyTriangularOffset<n_vertex0, n_vertex1, n_vertex_num>::n_result
		};
		// calculate index of the block (among all upper triangular blocks of this edge)

#ifdef _DEBUG // in the unlikely event that _DEBUG is not defined but _ASSERTE is still checking, this would trigger as the variables are zeroed only if _DEBUG is defined
		_ASSERTE(!m_p_HtSiH[n_block]);
#endif // _DEBUG
		// make sure that they are not set twice

		if(!m_vertex_ptr.template Get<n_vertex1>()->b_IsConstant()) { // if n_vertex1 is not constant
			_ASSERTE((m_p_vertex_id[n_vertex0] > m_p_vertex_id[n_vertex1]) ==
				(m_vertex_ptr.template Get<n_vertex0>()->n_Order() > m_vertex_ptr.template Get<n_vertex1>()->n_Order()));
			// if this triggers, then the edge has the vertices assigned in a different
			// order than the ids (vertex[0] is id[1] and vice versa) consequently, the Hessians
			// will have correct shape in the matrix, but the data will be transposed
			// and you will get either not pos def / rubbish solutions

			bool b_transpose_block, b_uninit_block;
			size_t n_dimension0 = CVertexTraits<n_vertex0>::n_dimension;
			size_t n_dimension1 = CVertexTraits<n_vertex1>::n_dimension;
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			size_t n_id_0 = m_p_vertex_id[n_vertex0];
			size_t n_id_1 = m_p_vertex_id[n_vertex1];
			if((b_transpose_block = (n_id_0 > n_id_1))) {
				std::swap(n_id_0, n_id_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_id_0 < n_id_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			m_p_HtSiH[n_block] = r_lambda.p_GetBlock_Log_Alloc(n_id_0, n_id_1, n_dimension0, n_dimension1, b_uninit_block);
			// this cannot fragment large empty rows / columns, which becomes a problem with hyperedges
			// (it is harder to enforce strict vertex order as with binary edges);
			// t_odo - what if binary and unary edges are mixed with hyperedges? then likely we cannot use
			// this function either. - actually, unary and binary edges will always use p_FindBlock_Alloc()
			// unless __BASE_TYPES_USE_ID_ADDRESSING is defined (which is supposed to be used with caution)
			// so there is no problem
#else // __BASE_TYPES_USE_ID_ADDRESSING
			size_t n_order_0 = m_vertex_ptr.template Get<n_vertex0>()->n_Order();
			size_t n_order_1 = m_vertex_ptr.template Get<n_vertex1>()->n_Order();
			if((b_transpose_block = (n_order_0 > n_order_1))) {
				std::swap(n_order_0, n_order_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_order_0 < n_order_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			m_p_HtSiH[n_block] = r_lambda.p_FindBlock_Alloc(n_order_0, n_order_1, n_dimension0, n_dimension1, b_uninit_block);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			// find a block for Hessian above the diagonal, and with the right shape

			_ASSERTE(m_p_HtSiH[n_block]);
			// if this triggers, most likely __BASE_TYPES_USE_ID_ADDRESSING is enabled (see FlatSystem.h)
			// and the edges come in some random order

			{
				typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();

				if(b_transpose_block) {
					typedef fbs_ut::CCTSize2D<CVertexTraits<n_vertex1>::n_dimension, CVertexTraits<n_vertex0>::n_dimension> BlockSize;
					if(b_uninit_block)
						rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_HtSiH[n_block], &m_p_HtSiH[n_block]); // return value ignored (equals m_p_HtSiH[n_block])
					else {
						m_p_HtSiH[n_block] = rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_vertex_id[n_vertex1],
							m_p_vertex_id[n_vertex0], m_p_HtSiH[n_block]); // potentially perform relocation to a temp block for reduction
					}
				} else {
					typedef fbs_ut::CCTSize2D<CVertexTraits<n_vertex0>::n_dimension, CVertexTraits<n_vertex1>::n_dimension> BlockSize;
					if(b_uninit_block)
						rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_HtSiH[n_block], &m_p_HtSiH[n_block]); // return value ignored (equals m_p_HtSiH[n_block])
					else {
						m_p_HtSiH[n_block] = rp.template p_OffDiagonal_GetTempBlock<BlockSize>(m_p_vertex_id[n_vertex0],
							m_p_vertex_id[n_vertex1], m_p_HtSiH[n_block]); // potentially perform relocation to a temp block for reduction
					}
				}
				// make space for the off-diagonal block in the reduction scheme
			}

			// t_odo - continue with this // there's nothing more to do here
		} else
			m_p_HtSiH[n_block] = 0; // the vertex is constant

		_Alloc_HessianBlocks_v2_Inner(r_lambda, r_rp, fbs_ut::CCTSize<n_vertex1 + 1>(), tag1);
		// loop
	}

	/**
	 *	@brief allocates off-diagonal Hessian blocks (inner loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in,out] r_lambda is reference to the lambda matrix (unused)
	 *	@param[in,out] r_rp is reference to the reduction plan (unused)
	 *	@param[in] tag0 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *	@param[in] tag1 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0, class CReductionPlan>
	inline void _Alloc_HessianBlocks_v2_Inner(CUberBlockMatrix &UNUSED(r_lambda), CReductionPlan &UNUSED(r_rp),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag0), fbs_ut::CCTSize<n_vertex0> UNUSED(tag1))
	{}

	/**
	 *	@brief allocates diagonal Hessian blocks (outer loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in,out] r_lambda is reference to the lambda matrix
	 *	@param[in,out] r_rp is reference to the reduction plan
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0, class CReductionPlan>
	inline void _Alloc_HessianBlocks_v2(CUberBlockMatrix &r_lambda, CReductionPlan &r_rp,
		fbs_ut::CCTSize<n_vertex0> tag)
	{
#ifdef _DEBUG // in the unlikely event that _DEBUG is not defined but _ASSERTE is still checking, this would trigger as the variables are zeroed only if _DEBUG is defined
		_ASSERTE(!m_p_HtSiH_vert[n_vertex0]);
		_ASSERTE(!m_p_RHS_vert[n_vertex0]);
#endif // _DEBUG
		// make sure that they are not set twice

		if(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()) { // if the vertex is not constant
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			m_p_HtSiH_vert[n_vertex0] = r_lambda.p_GetBlock_Log(m_p_vertex_id[n_vertex0], m_p_vertex_id[n_vertex0],
				CVertexTraits<n_vertex0>::n_dimension, CVertexTraits<n_vertex0>::n_dimension, true, false);
#else // __BASE_TYPES_USE_ID_ADDRESSING
			m_p_HtSiH_vert[n_vertex0] = r_lambda.p_FindBlock(m_vertex_ptr.template Get<n_vertex0>()->n_Order(),
				m_vertex_ptr.template Get<n_vertex0>()->n_Order(), CVertexTraits<n_vertex0>::n_dimension,
				CVertexTraits<n_vertex0>::n_dimension, true, false);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			// find a block for vertices' Hessian on the diagonal (do not use the potentially swapped id / order)
			// note that if this is added after the off-diagonal Hessian, it will be likely
			// added at the end of the block list in the matrix

			{
				typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();

				m_p_HtSiH_vert[n_vertex0] = rp.template p_Diagonal_GetTempBlock<typename
					CVertexTraits<n_vertex0>::_TyMatrixSize>(m_p_vertex_id[n_vertex0],
					m_p_vertex_id[n_vertex0], m_p_HtSiH_vert[n_vertex0]);
				// make space for the vertices in the reduction scheme
			}

			{
				typename CReductionPlan::CRHSReductor &rp = r_rp.r_RHS_ReductionPlan();

				m_p_RHS_vert[n_vertex0] =
					rp.template p_Get_ReductionBlock<CVertexTraits<n_vertex0>::n_dimension>(
					m_vertex_ptr.template Get<n_vertex0>()->n_Order());
			}
			// alloc the RHS reduction temporaries

			_Alloc_HessianBlocks_v2_Inner(r_lambda, r_rp, fbs_ut::CCTSize<n_vertex0 + 1>(), tag);
			// alloc off-diagonal blocks
		} else {
			m_p_HtSiH_vert[n_vertex0] = 0;
			m_p_RHS_vert[n_vertex0] = 0;
		}

		_Alloc_HessianBlocks_v2(r_lambda, r_rp, fbs_ut::CCTSize<n_vertex0 + 1>());
		// loop
	}

	/**
	 *	@brief allocates diagonal Hessian blocks (outer loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in,out] r_lambda is reference to the lambda matrix (unused)
	 *	@param[in,out] r_rp is reference to the reduction plan (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <class CReductionPlan>
	inline void _Alloc_HessianBlocks_v2(CUberBlockMatrix &UNUSED(r_lambda),
		CReductionPlan &UNUSED(r_rp), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

public:
	/**
	 *	@brief allocates Hessian blocks
	 *
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@param[in,out] r_rp is lambda reduction plan
	 *
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	template <class CReductionPlan>
	inline void Alloc_HessianBlocks_v2(CUberBlockMatrix &r_lambda, CReductionPlan &r_rp)
	{
		// somewhat complicated with loops, need to make it clear how the matrix is supposed to look for hypergraphs
		// the number of off-diag blocks rises with O(n^2), it is every-to-every, but only upper diag
		// it is num_blocks = (n * n - n) / 2
		// 1: 0
		// 2: 1
		// 3: 3
		// 4: 6
		// all the off-diagonal blocks need to be managed for duplicates (that *should* be automatic, by the reduction plan)
		// all the blocks need to be sorted and transposed accordingly (but that is probably easily done on pairwise basis - yes)
		// also need to start thinking about supporting constant vertices properly - those will not be in the matrix! (actually very simple to do)

#ifdef _DEBUG
		memset(m_p_HtSiH, 0, sizeof(double*) * n_offdiag_block_num);
		memset(m_p_RHS_vert, 0, sizeof(double*) * n_vertex_num);
		memset(m_p_HtSiH_vert, 0, sizeof(double*) * n_vertex_num);
		// set to null to make sure we're not loading anything twice
#endif // _DEBUG

		_Alloc_HessianBlocks_v2(r_lambda, r_rp, fbs_ut::CCTSize<0>());
		// loops for many vertices

#ifdef _DEBUG
#ifndef __BASE_TYPES_ALLOW_CONST_VERTICES
		for(int i = 0; i < int(n_offdiag_block_num); ++ i)
			_ASSERTE(m_p_HtSiH[i]);
		for(int i = 0; i < int(n_vertex_num); ++ i) {
			_ASSERTE(m_p_RHS_vert[i]);
			_ASSERTE(m_p_HtSiH_vert[i]);
		}
		// make sure all were initialized
#else // !__BASE_TYPES_ALLOW_CONST_VERTICES
		bool b_had_nonconst = false;
		for(int i = 0; i < int(n_vertex_num); ++ i) {
			_ASSERTE(!m_p_RHS_vert[i] == !m_p_HtSiH_vert[i]); // both zero or not zero at the same time
			if(m_p_HtSiH_vert[i])
				b_had_nonconst = true;
		}
		_ASSERTE(b_had_nonconst); // if this triggers then all the vertices are constant. do not add this edge to the system in the first place
		// more basic checks
#endif // !__BASE_TYPES_ALLOW_CONST_VERTICES
#endif // _DEBUG

		// in virtual call scenario, either system or solver would need to specialize a virtual interface
		// on reduction plan, which would in turn need to export non-template functions with runtime seatch
		// of appropriate bin for a given block size
	}

protected:
	/**
	 *	@brief reduces off-diagonal Hessian blocks (inner loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in] r_lambda is reference to the lambda matrix
	 *	@param[in,out] r_rp is reference to the reduction plan
	 *	@param[in] tag0 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *	@param[in] tag1 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex1, const int n_vertex0, class CReductionPlan>
	inline void _Reduce_Hessians_v2_Inner(const CUberBlockMatrix &r_lambda, CReductionPlan &r_rp,
		fbs_ut::CCTSize<n_vertex1> UNUSED(tag0), fbs_ut::CCTSize<n_vertex0> tag1)
	{
		// we need to reduce a block between n_vertex1 and n_vertex0

		_ASSERTE(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()); // make sure that n_vertex0 is not constant

		enum {
			n_block = hyperedge_detail::CStrictlyTriangularOffset<n_vertex0, n_vertex1, n_vertex_num>::n_result
		};
		// calculate index of the block (among all upper triangular blocks of this edge)

		if(!m_vertex_ptr.template Get<n_vertex1>()->b_IsConstant()) { // if n_vertex1 is not constant
			typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();
			const size_t n_id_0 = m_p_vertex_id[n_vertex0], n_id_1 = m_p_vertex_id[n_vertex1];
#if defined(__BASE_TYPES_USE_ID_ADDRESSING) || !defined(__BASE_TYPES_HYPEREDGES_AVOID_ID_ORDERING)
			// this would be usually a better choice, but the vertex ordering is more problematic with hyperedges,
			// it might be hard to make an edge where the vertex order corresponds to vertex ids, therefore we will
			// not use ids for ordering here, although it would have been faster

			_ASSERTE((n_id_0 > n_id_1) == (m_vertex_ptr.template Get<n_vertex0>()->n_Order() >
				m_vertex_ptr.template Get<n_vertex1>()->n_Order())); // if this triggers, then the edge has the vertices assigned in different order than the ids (vertex[0] is id[1] and vice versa)
			if(n_id_0 > n_id_1) {
#else // __BASE_TYPES_USE_ID_ADDRESSING || !__BASE_TYPES_HYPEREDGES_AVOID_ID_ORDERING
			const size_t n_order_0 = m_vertex_ptr.template Get<n_vertex0>()->n_Order();
			const size_t n_order_1 = m_vertex_ptr.template Get<n_vertex1>()->n_Order();
			if(n_order_0 > n_order_1) {
#endif // __BASE_TYPES_USE_ID_ADDRESSING || !__BASE_TYPES_HYPEREDGES_AVOID_ID_ORDERING
				typename CReductionPlan::CLambdaReductor::TKey t_key;
				if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) // compile-time const
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_1, n_id_0, 0); // pointer ignored
				else {
					const double *p_HtSiH = r_lambda.p_GetBlock_Log(n_id_1, n_id_0,
						CVertexTraits<n_vertex1>::n_dimension, CVertexTraits<n_vertex0>::n_dimension);
					_ASSERTE(p_HtSiH);
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_1, n_id_0, p_HtSiH);
					// need the address of the unique *original* block, not of one of the many reduction temporaries
				}

				typedef fbs_ut::CCTSize2D<CVertexTraits<n_vertex1>::n_dimension,
					CVertexTraits<n_vertex0>::n_dimension> BlockSize;
				rp.template ReduceSingle<BlockSize>(t_key);
			} else {
				typename CReductionPlan::CLambdaReductor::TKey t_key;
				if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) // compile-time const
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_1, 0); // pointer ignored
				else {
					const double *p_HtSiH = r_lambda.p_GetBlock_Log(n_id_0, n_id_1,
						CVertexTraits<n_vertex0>::n_dimension, CVertexTraits<n_vertex1>::n_dimension);
					_ASSERTE(p_HtSiH);
					t_key = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_1, p_HtSiH);
					// need the address of the unique *original* block, not of one of the many reduction temporaries
				}

				typedef fbs_ut::CCTSize2D<CVertexTraits<n_vertex0>::n_dimension,
					CVertexTraits<n_vertex1>::n_dimension> BlockSize;
				rp.template ReduceSingle<BlockSize>(t_key);
			}
		}

		_Reduce_Hessians_v2_Inner(r_lambda, r_rp, fbs_ut::CCTSize<n_vertex1 + 1>(), tag1);
		// loop
	}

	/**
	 *	@brief reduces off-diagonal Hessian blocks (inner loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in] r_lambda is reference to the lambda matrix (unused)
	 *	@param[in,out] r_rp is reference to the reduction plan (unused)
	 *	@param[in] tag0 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *	@param[in] tag1 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0, class CReductionPlan>
	inline void _Reduce_Hessians_v2_Inner(const CUberBlockMatrix &UNUSED(r_lambda), CReductionPlan &UNUSED(r_rp),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag0), fbs_ut::CCTSize<n_vertex0> UNUSED(tag1))
	{}

	/**
	 *	@brief reduces diagonal Hessian blocks (outer loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in] r_lambda is reference to the lambda matrix
	 *	@param[in,out] r_rp is reference to the reduction plan
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0, class CReductionPlan>
	inline void _Reduce_Hessians_v2(const CUberBlockMatrix &r_lambda, CReductionPlan &r_rp,
		fbs_ut::CCTSize<n_vertex0> tag)
	{
		if(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()) { // if the vertex is not constant
			typename CReductionPlan::CLambdaReductor &rp = r_rp.r_Lambda_ReductionPlan();
			const size_t n_id_0 = m_p_vertex_id[n_vertex0];

			typename CReductionPlan::CLambdaReductor::TKey p_key_vert; // one at a time
			if(CReductionPlan::CLambdaReductor::b_use_block_coord_keys) // compile-time const
				p_key_vert = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_0, 0); // pointer ignored
			else { // potentially untested code below
				const double *p_HtSiH_vert; // one at a time
				p_HtSiH_vert = r_lambda.p_GetBlock_Log(n_id_0, n_id_0,
					CVertexTraits<n_vertex0>::n_dimension, CVertexTraits<n_vertex0>::n_dimension);
				_ASSERTE(p_HtSiH_vert);
				// need the address of the unique *original* block, not of one of the many reduction temporaries
				// t_odo - perhaps it would be better to use (row, col) as unique index to the reduction, this lookup would then be avoided

				p_key_vert = CReductionPlan::CLambdaReductor::t_MakeKey(n_id_0, n_id_0, p_HtSiH_vert);
			}
			rp.template ReduceSingle<typename CVertexTraits<n_vertex0>::_TyMatrixSize>(p_key_vert);
			// reduce the current vertex

			_Reduce_Hessians_v2_Inner(r_lambda, r_rp, fbs_ut::CCTSize<n_vertex0 + 1>(), tag);
			// reduce off-diagonal blocks
		}

		_Reduce_Hessians_v2(r_lambda, r_rp, fbs_ut::CCTSize<n_vertex0 + 1>());
		// loop
	}

	/**
	 *	@brief reduces diagonal Hessian blocks (outer loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in] r_lambda is reference to the lambda matrix (unused)
	 *	@param[in,out] r_rp is reference to the reduction plan (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <class CReductionPlan>
	inline void _Reduce_Hessians_v2(const CUberBlockMatrix &UNUSED(r_lambda),
		CReductionPlan &UNUSED(r_rp), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

public:
	/**
	 *	@brief sums up Hessian block contributions to get values of the blocks in lambda
	 *
	 *	@tparam CReductionPlan is lambda reduction plan instantiation
	 *
	 *	@param[in] r_lambda is the target matrix where the blocks are stored
	 *	@param[in,out] r_rp is lambda reduction plan
	 *
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 *	@note This only reduces the Hessians, Calculate_Hessians_v2() must be called
	 *		before to have stuff to calculate.
	 */
	template <class CReductionPlan>
	inline void Reduce_Hessians_v2(const CUberBlockMatrix &r_lambda, CReductionPlan &r_rp)
	{
		_Reduce_Hessians_v2(r_lambda, r_rp, fbs_ut::CCTSize<0>());
	}

protected:
	/**
	 *	@brief calculates off-diagonal Hessian blocks (inner loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex
	 *	@param[in] t_H0_sigma_inv is a product of Jacobian of n_vertex0 with sigma inverse
	 *	@param[in] tag0 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *	@param[in] tag1 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex1, const int n_vertex0, class CH0_Sigma_inv_Matrix>
	inline void _Calculate_Hessians_v2_Inner(const _TyJacobianTuple &r_t_jacobians,
		const CH0_Sigma_inv_Matrix &t_H0_sigma_inv, fbs_ut::CCTSize<n_vertex1> UNUSED(tag0),
		fbs_ut::CCTSize<n_vertex0> tag1)
	{
		// no need to change this for robust edges, it works

		// we need to reduce a block between n_vertex1 and n_vertex0

		_ASSERTE(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()); // make sure that n_vertex0 is not constant

		enum {
			n_block = hyperedge_detail::CStrictlyTriangularOffset<n_vertex0, n_vertex1, n_vertex_num>::n_result
		};
		// calculate index of the block (among all upper triangular blocks of this edge)

		if(!m_vertex_ptr.template Get<n_vertex1>()->b_IsConstant()) { // if n_vertex1 is not constant
			const typename CVertexTraits<n_vertex1>::_TyJacobianMatrix &t_jacobian1 =
				r_t_jacobians.template Get<n_vertex1>();

			bool b_transpose_hessian;
			size_t n_dimension0 = CVertexTraits<n_vertex0>::n_dimension;
			size_t n_dimension1 = CVertexTraits<n_vertex1>::n_dimension;
#if defined(__BASE_TYPES_USE_ID_ADDRESSING) || !defined(__BASE_TYPES_HYPEREDGES_AVOID_ID_ORDERING)
			// this would be usually a better choice, but the vertex ordering is more problematic with hyperedges,
			// it might be hard to make an edge where the vertex order corresponds to vertex ids, therefore we will
			// not use ids for ordering here, although it would have been faster

			size_t n_id_0 = m_p_vertex_id[n_vertex0];
			size_t n_id_1 = m_p_vertex_id[n_vertex1]; // this is closer in cache
			_ASSERTE((n_id_0 > n_id_1) == (m_vertex_ptr.template Get<n_vertex0>()->n_Order() >
				m_vertex_ptr.template Get<n_vertex1>()->n_Order())); // if this triggers, then the edge has the vertices assigned in different order than the ids (vertex[0] is id[1] and vice versa)
			if((b_transpose_hessian = (n_id_0 > n_id_1))) {
				std::swap(n_id_0, n_id_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_id_0 != n_id_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_id_0 < n_id_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)
#else // __BASE_TYPES_USE_ID_ADDRESSING || !__BASE_TYPES_HYPEREDGES_AVOID_ID_ORDERING
			size_t n_order_0 = m_vertex_ptr.template Get<n_vertex0>()->n_Order();
			size_t n_order_1 = m_vertex_ptr.template Get<n_vertex1>()->n_Order();
			if((b_transpose_hessian = (n_order_0 > n_order_1))) {
				std::swap(n_order_0, n_order_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_order_0 < n_order_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)
#endif // __BASE_TYPES_USE_ID_ADDRESSING || !__BASE_TYPES_HYPEREDGES_AVOID_ID_ORDERING

			if(b_transpose_hessian) {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<n_vertex1>::n_dimension, CVertexTraits<n_vertex0>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH[n_block]); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				t_HtSiH.noalias() = t_jacobian1.transpose() * t_H0_sigma_inv.transpose();
			} else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension, CVertexTraits<n_vertex1>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH(m_p_HtSiH[n_block]); // t_odo - support aligned if the uberblockmatrix is aligned!
				// map the matrix above diagonal

				t_HtSiH.noalias() = t_H0_sigma_inv * t_jacobian1;
			}
			// calculate the matrix above diagonal
		}

		_Calculate_Hessians_v2_Inner(r_t_jacobians,
			t_H0_sigma_inv, fbs_ut::CCTSize<n_vertex1 + 1>(), tag1);
		// loop
	}

	/**
	 *	@brief calculates off-diagonal Hessian blocks (inner loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex (unused)
	 *	@param[in] t_H0_sigma_inv is a product of Jacobian of n_vertex0 with sigma inverse (unused)
	 *	@param[in] tag0 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *	@param[in] tag1 is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0, class CH0_Sigma_inv_Matrix>
	inline void _Calculate_Hessians_v2_Inner(const _TyJacobianTuple &UNUSED(r_t_jacobians),
		const CH0_Sigma_inv_Matrix &UNUSED(t_H0_sigma_inv), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag0),
		fbs_ut::CCTSize<n_vertex0> UNUSED(tag1))
	{}

	/**
	 *	@brief calculates diagonal Hessian blocks and the corresponding parts
	 *		of the right hand side vector (outer loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex
	 *	@param[in] r_v_error is error vector (measurement - expectation)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0>
	inline void _Calculate_Hessians_v2(const _TyJacobianTuple &r_t_jacobians,
		const _TyVector &r_v_error, fbs_ut::CCTSize<n_vertex0> tag)
	{
		if(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()) { // if the vertex is not constant
			const typename CVertexTraits<n_vertex0>::_TyJacobianMatrix &t_jacobian0 = r_t_jacobians.template Get<n_vertex0>();
			//const typename CVertexTraits<n_vertex1>::_TyJacobianMatrix &t_jacobian1 = r_t_jacobians.template Get<n_vertex1>();
			// can tradeoff a copy for unaligned processing here

			Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension, n_residual_dimension> t_H0_sigma_inv =
				t_jacobian0.transpose() * m_t_sigma_inv; // will need to pass this down

			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyMatrixAlign,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[n_vertex0]);
			if(hyperedge_detail::explicit_SymmetricProduct) // compile-time const
				t_HtSiH_vertex0/*.noalias()*/ = (t_H0_sigma_inv * t_jacobian0).template selfadjointView<Eigen::Upper>();
			else
				t_HtSiH_vertex0.noalias() = t_H0_sigma_inv * t_jacobian0;
			/*Eigen::Map<typename CVertexTraits<n_vertex1>::_TyMatrixAlign,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[n_vertex1]);
			t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;*/
			// calculate diagonal Hessian contributions

			//_ASSERTE(t_HtSiH_vertex0 == t_HtSiH_vertex0.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyVectorAlign,
				CUberBlockMatrix::map_Alignment> t_right_hand_vertex0(m_p_RHS_vert[n_vertex0]);
			t_right_hand_vertex0.noalias() = t_H0_sigma_inv * r_v_error;//t_jacobian0.transpose() * (m_t_sigma_inv * r_v_error);
			/*Eigen::Map<typename CVertexTraits<n_vertex1>::_TyVectorAlign,
				CUberBlockMatrix::map_Alignment> t_right_hand_vertex1(m_p_RHS_vert[n_vertex1]);
			t_right_hand_vertex1.noalias() = t_jacobian1.transpose() * (m_t_sigma_inv * r_v_error);*/
			// calculate right hand side vector contributions

			_Calculate_Hessians_v2_Inner(r_t_jacobians, t_H0_sigma_inv,
				fbs_ut::CCTSize<n_vertex0 + 1>(), tag);
			// calculate off-diagonal blocks
		}

		_Calculate_Hessians_v2(r_t_jacobians, r_v_error, fbs_ut::CCTSize<n_vertex0 + 1>());
		// loop
	}

	/**
	 *	@brief calculates diagonal Hessian blocks and the corresponding parts
	 *		of the right hand side vector (outer loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex (unused)
	 *	@param[in] r_v_error is error vector (measurement - expectation; unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	inline void _Calculate_Hessians_v2(const _TyJacobianTuple &UNUSED(r_t_jacobians),
		const _TyVector &UNUSED(r_v_error), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

	/**
	 *	@brief calculates diagonal Hessian blocks and the corresponding parts
	 *		of the right hand side vector (outer loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex
	 *	@param[in] r_v_error is error vector (measurement - expectation)
	 *	@param[in] f_weight is edge weight
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	template <const int n_vertex0>
	inline void _Calculate_Hessians_v2(const _TyJacobianTuple &r_t_jacobians,
		const _TyVector &r_v_error, double f_weight, fbs_ut::CCTSize<n_vertex0> tag)
	{
		if(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()) { // if the vertex is not constant
			const typename CVertexTraits<n_vertex0>::_TyJacobianMatrix &t_jacobian0 = r_t_jacobians.template Get<n_vertex0>();
			//const typename CVertexTraits<n_vertex1>::_TyJacobianMatrix &t_jacobian1 = r_t_jacobians.template Get<n_vertex1>();
			// can tradeoff a copy for unaligned processing here

			Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension, n_residual_dimension> t_H0_sigma_inv =
				t_jacobian0.transpose() * m_t_sigma_inv * f_weight; // will need to pass this down
			// this already contains the weight, so the inner loop does
			// not need to be reimplemented; it also changes the r.h.s.

			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyMatrixAlign,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[n_vertex0]);
			if(hyperedge_detail::explicit_SymmetricProduct) // compile-time const
				t_HtSiH_vertex0/*.noalias()*/ = (t_H0_sigma_inv * t_jacobian0).template selfadjointView<Eigen::Upper>();
			else
				t_HtSiH_vertex0.noalias() = t_H0_sigma_inv * t_jacobian0;
			/*Eigen::Map<typename CVertexTraits<n_vertex1>::_TyMatrixAlign,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[n_vertex1]);
			t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;*/
			// calculate diagonal Hessian contributions

			//_ASSERTE(t_HtSiH_vertex0 == t_HtSiH_vertex0.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyVectorAlign,
				CUberBlockMatrix::map_Alignment> t_right_hand_vertex0(m_p_RHS_vert[n_vertex0]);
			t_right_hand_vertex0.noalias() = t_H0_sigma_inv * r_v_error;//t_jacobian0.transpose() * (m_t_sigma_inv * r_v_error);
			/*Eigen::Map<typename CVertexTraits<n_vertex1>::_TyVectorAlign,
				CUberBlockMatrix::map_Alignment> t_right_hand_vertex1(m_p_RHS_vert[n_vertex1]);
			t_right_hand_vertex1.noalias() = t_jacobian1.transpose() * (m_t_sigma_inv * r_v_error);*/
			// calculate right hand side vector contributions

			_Calculate_Hessians_v2_Inner(r_t_jacobians, t_H0_sigma_inv,
				fbs_ut::CCTSize<n_vertex0 + 1>(), tag);
			// calculate off-diagonal blocks
		}

		_Calculate_Hessians_v2(r_t_jacobians, r_v_error, f_weight, fbs_ut::CCTSize<n_vertex0 + 1>());
		// loop
	}

	/**
	 *	@brief calculates diagonal Hessian blocks and the corresponding parts
	 *		of the right hand side vector (outer loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex (unused)
	 *	@param[in] r_v_error is error vector (measurement - expectation; unused)
	 *	@param[in] f_weight is edge weight (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 */
	inline void _Calculate_Hessians_v2(const _TyJacobianTuple &UNUSED(r_t_jacobians),
		const _TyVector &UNUSED(r_v_error), double UNUSED(f_weight),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{}

	/**
	 *	@brief calculates diagonal Hessian blocks and the corresponding parts
	 *		of the right hand side vector (outer loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex
	 *	@param[in] r_v_error is error vector (measurement - expectation)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 */
	template <const int n_vertex0>
	inline _TyVector _v_Calculate_Hessians_v2(const _TyJacobianTuple &r_t_jacobians,
		const _TyVector &r_v_error, fbs_ut::CCTSize<n_vertex0> tag)
	{
		if(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()) { // if the vertex is not constant
			const typename CVertexTraits<n_vertex0>::_TyJacobianMatrix &t_jacobian0 = r_t_jacobians.template Get<n_vertex0>();
			//const typename CVertexTraits<n_vertex1>::_TyJacobianMatrix &t_jacobian1 = r_t_jacobians.template Get<n_vertex1>();
			// can tradeoff a copy for unaligned processing here

			Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension, n_residual_dimension> t_H0_sigma_inv =
				t_jacobian0.transpose() * m_t_sigma_inv; // will need to pass this down

			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyMatrixAlign,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[n_vertex0]);
			if(hyperedge_detail::explicit_SymmetricProduct) // compile-time const
				t_HtSiH_vertex0/*.noalias()*/ = (t_H0_sigma_inv * t_jacobian0).template selfadjointView<Eigen::Upper>();
			else
				t_HtSiH_vertex0.noalias() = t_H0_sigma_inv * t_jacobian0;
			/*Eigen::Map<typename CVertexTraits<n_vertex1>::_TyMatrixAlign,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[n_vertex1]);
			t_HtSiH_vertex1.noalias() = t_jacobian1.transpose() * m_t_sigma_inv * t_jacobian1;*/
			// calculate diagonal Hessian contributions

			//_ASSERTE(t_HtSiH_vertex0 == t_HtSiH_vertex0.transpose()); // is this symmetric?

			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyVectorAlign,
				CUberBlockMatrix::map_Alignment> t_right_hand_vertex0(m_p_RHS_vert[n_vertex0]);
			typename CVertexTraits<n_vertex0>::_TyVectorAlign g = t_jacobian0.transpose() * (m_t_sigma_inv * r_v_error);
			t_right_hand_vertex0.noalias() = g;
			/*Eigen::Map<typename CVertexTraits<n_vertex1>::_TyVectorAlign,
				CUberBlockMatrix::map_Alignment> t_right_hand_vertex1(m_p_RHS_vert[n_vertex1]);
			t_right_hand_vertex1.noalias() = t_jacobian1.transpose() * (m_t_sigma_inv * r_v_error);*/
			// calculate right hand side vector contributions

			_TyVector v_Jg = t_jacobian0 * g;
			// calculate squared norm of ||J g||^2 = ||J J^T Sigma^(-1) r||^2
			// note that Jg is a vector with the same dimension as the error vector

			_Calculate_Hessians_v2_Inner(r_t_jacobians, t_H0_sigma_inv,
				fbs_ut::CCTSize<n_vertex0 + 1>(), tag);
			// calculate off-diagonal blocks

			return v_Jg + _v_Calculate_Hessians_v2(r_t_jacobians,
				r_v_error, fbs_ut::CCTSize<n_vertex0 + 1>());
			// loop (breaks tail recursion)
		} else {
			return _v_Calculate_Hessians_v2(r_t_jacobians,
				r_v_error, fbs_ut::CCTSize<n_vertex0 + 1>());
			// loop
		}
	}

	/**
	 *	@brief calculates diagonal Hessian blocks and the corresponding parts
	 *		of the right hand side vector (outer loop; specialisation for recursion termination)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (in this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (in this edge)
	 *	@tparam CH0_Sigma_inv_Matrix is specialization of Eigen::Matrix which holds the result
	 *		of the product of Jacobian of n_vertex0 with sigma inverse
	 *
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex (unused)
	 *	@param[in] r_v_error is error vector (measurement - expectation; unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 */
	inline _TyVector _v_Calculate_Hessians_v2(const _TyJacobianTuple &UNUSED(r_t_jacobians),
		const _TyVector &UNUSED(r_v_error), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{
		return _TyVector::Zero();
	}

	/**
	 *	@brief calculates the denominator of the gradient descent scaling factor
	 *
	 *	@tparam n_vertex is zero-based index of the vertex (in this edge)
	 *
	 *	@param[in] r_v_g is the g vector from the dogleg paper (\f$-J^T \Sigma^{-1} r\f$)
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 */
	template <const int n_vertex>
	inline _TyVector _v_Calculate_SDS(const Eigen::VectorXd &r_v_g,
		const _TyJacobianTuple &r_t_jacobians, fbs_ut::CCTSize<n_vertex> tag)
	{
		if(!m_vertex_ptr.template Get<n_vertex>()->b_IsConstant()) { // if the vertex is not constant
			const typename CVertexTraits<n_vertex>::_TyJacobianMatrix &t_jacobian0 = r_t_jacobians.template Get<n_vertex>();
			//const typename CVertexTraits<n_vertex1>::_TyJacobianMatrix &t_jacobian1 = r_t_jacobians.template Get<n_vertex1>();
			// can tradeoff a copy for unaligned processing here

			_TyVector v_Jg = t_jacobian0 * r_v_g.segment<CVertexTraits<n_vertex>::n_dimension>(
				m_vertex_ptr.template Get<n_vertex>()->n_Order());
			// calculate squared norm of ||J g||^2 = ||J J^T Sigma^(-1) r||^2
			// note that Jg is a vector with the same dimension as the error vector
			// no need to change this for robust edges; it works

			return v_Jg + _v_Calculate_SDS(r_v_g, r_t_jacobians, fbs_ut::CCTSize<n_vertex + 1>());
			// loop (breaks tail recursion)
		} else {
			return _v_Calculate_SDS(r_v_g, r_t_jacobians, fbs_ut::CCTSize<n_vertex + 1>());
			// loop
		}
	}

	/**
	 *	@brief calculates the denominator of the gradient descent scaling factor
	 *		(specialization for recursion termination)
	 *
	 *	@param[in] r_v_g is the g vector from the dogleg paper (unused)
	 *	@param[in] r_t_jacobians is tuple of Jacobians, one for each vertex (unused)
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@return Returns a null vector.
	 */
	inline _TyVector _v_Calculate_SDS(const Eigen::VectorXd &UNUSED(r_v_g),
		const _TyJacobianTuple &UNUSED(r_t_jacobians), fbs_ut::CCTSize<n_vertex_num> UNUSED(tag))
	{
		return _TyVector::Zero();
	}

	inline void _Calculate_Hessians_v2_ChooseRobust(const _TyJacobianTuple &t_jacobians,
		const _TyVector &v_error, fbs_ut::CCTSize<0> UNUSED(t_robust_tag))
	{
		_Calculate_Hessians_v2(t_jacobians, v_error, fbs_ut::CCTSize<0>());
	}

	inline void _Calculate_Hessians_v2_ChooseRobust(const _TyJacobianTuple &t_jacobians,
		const _TyVector &v_error, fbs_ut::CCTSize<1> UNUSED(t_robust_tag))
	{
		double f_robust_weight = ((CDerivedEdge*)this)->f_RobustWeight(v_error);
		_Calculate_Hessians_v2(t_jacobians, v_error, f_robust_weight, fbs_ut::CCTSize<0>());
	}

	// if there was also a need for explicit weight for edge switching, there could be another version

public:
	/**
	 *	@brief calculates Hessian contributions
	 *
	 *	@note This only calculates the Hessians, either Reduce_Hessians_v2() or the
	 *		reduction plan needs to be used to fill lambda with values.
	 */
	inline void Calculate_Hessians_v2()
	{
		_TyJacobianTuple t_jacobians;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobians, v_expectation, v_error);
		// calculates the expectation and the jacobians

		_Calculate_Hessians_v2_ChooseRobust(t_jacobians, v_error, fbs_ut::CCTSize<(b_is_robust_edge)? 1 : 0>());
	}

	/**
	 *	@brief calculates the gradient descent scaling factor
	 *	@param[in] r_v_g is the g vector from the dogleg paper (\f$-J^T \Sigma^{-1} r\f$)
	 *	@return Returns the denominator of the gradient descent scaling factor (needed by dogleg).
	 */
	inline double f_Calculate_Steepest_Descent_Scale(const Eigen::VectorXd &r_v_g)
	{
		_TyJacobianTuple t_jacobians;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobians, v_expectation, v_error);
		// calculates the expectation and the jacobians

		return _v_Calculate_SDS(r_v_g, t_jacobians, fbs_ut::CCTSize<0>()).squaredNorm(); // need to sum up in vector first, then take the norm
		// no need to change this for robust edges; it works
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
		_TyJacobianTuple t_jacobians;
		_TyVector v_expectation, v_error;
		((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(t_jacobians, v_expectation, v_error);
		// calculates the expectation and the jacobians

		_ASSERTE(!b_is_robust_edge); // robust edges not supported here (todo if it ends up being used, otherwise remove it)
		return _v_Calculate_Hessians_v2(t_jacobians, v_error, fbs_ut::CCTSize<0>()).squaredNorm();
	}

#else // __LAMBDA_USE_V2_REDUCTION_PLAN

#error this code is not updated to work with hyperedges (low priority TODO)
#error this code is not updated to work with const vertices (low priority TODO)

	inline bool Notify_HessianBlock_Conflict(double *p_block, CUberBlockMatrix &r_lambda)
	{
		if(m_p_HtSiH == p_block) {
			if(!m_p_HtSiH_reduce) {
				m_p_HtSiH = r_lambda.p_Get_DenseStorage(CVertexTraits<0>::n_dimension * CVertexTraits<1>::n_dimension);
				// get a temporary block

				memcpy(m_p_HtSiH, p_block, CVertexTraits<0>::n_dimension * CVertexTraits<1>::n_dimension * sizeof(double));
				// this off-diagonal Hessian might already have significant value, need to store it
			}
			// first time around this is called, alloc a different block to store this off-diagonal Hessian

			m_p_HtSiH_reduce = p_block;
			m_b_first_reductor = true; // i was the first

			return true; // acknowledge notify
		}
		return false; // not me
	}

	inline void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda)
	{
		m_p_vertex0->Add_ReferencingEdge(this);
		m_p_vertex1->Add_ReferencingEdge(this);
		// the vertices need to know this edge in order to calculate sum of Hessians on the diagonal of lambda // t_odo - should the lambda solver call this instead? might avoid duplicate records in the future // lambda solver is calling this only once in it's lifetime, no duplicates will occur

		_ASSERTE((m_p_vertex_id[0] > m_p_vertex_id[1]) ==
			(m_p_vertex0->n_Order() > m_p_vertex1->n_Order()));
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
			const std::vector<typename _TyVertex0::_TyBaseEdge*> &r_list =
				m_p_vertex0->r_ReferencingEdge_List();
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

protected:
	/**
	 *	@brief calculates maximum of value of diagonals of the diagonal Hessian blocks
	 *
	 *	@tparam n_vertex is zero-based vertex index (in this edge)
	 *
	 *	@param[in] f_max is the maximum value so far
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@return Returns maximum of f_max and the values at the diagonal of n_vertex Hessian block.
	 */
	template <const int n_vertex>
	double _f_Max_VertexHessianDiagValue(double f_max, fbs_ut::CCTSize<n_vertex> UNUSED(tag)) const // todo surround with levenberg support ifdef
	{
		if(!m_vertex_ptr.template Get<n_vertex>()->b_IsConstant()) { // if the vertex is not constant
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
			Eigen::Map<typename CVertexTraits<n_vertex>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex(m_p_HtSiH_vert[n_vertex]); // uses pointer array
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
			const Eigen::Map<typename CVertexTraits<n_vertex>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> &t_HtSiH_vertex = m_t_HtSiH_tuple.Get<n_vertex>(); // uses tuple
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN

			for(size_t i = 0; i < CVertexTraits<n_vertex>::n_dimension; ++ i)
				f_max = std::max(f_max, t_HtSiH_vertex(i, i));
		}

		return _f_Max_VertexHessianDiagValue(f_max, fbs_ut::CCTSize<n_vertex + 1>());
	}

	/**
	 *	@brief calculates maximum of value of diagonals of the diagonal Hessian blocks
	 *		(specialisation for recursion termination)
	 *
	 *	@param[in] f_max is the maximum value so far
	 *	@param[in] tag is specialization of fbs_ut::CCTSize for tag dispatch (unused at runtime)
	 *
	 *	@return Returns f_max.
	 */
	double _f_Max_VertexHessianDiagValue(double f_max, fbs_ut::CCTSize<n_vertex_num> UNUSED(tag)) const
	{
		return f_max;
	}

public:
	/**
	 *	@copydoc base_iface::CConstEdgeFacade::f_Max_VertexHessianDiagValue
	 */
	double f_Max_VertexHessianDiagValue() const // todo surround with levenberg support ifdef
	{
		return _f_Max_VertexHessianDiagValue(0, fbs_ut::CCTSize<0>());
	}

#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc base_iface::CConstEdgeFacade::Alloc_LBlocks
	 */
	inline void Alloc_LBlocks(CUberBlockMatrix &UNUSED(r_L)) const
	{
		// no vertex refs, that is already done in Alloc_HessianBlocks()
		// edges should not allocate off-diagonal blocks, it will just screw the reordering and increase fill-in
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---

protected:
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@brief inner loop for omega block allocation
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (inside this edge)
	 *	@tparam n_vertex1 is zero-based index of the second vertex (inside this edge)
	 *
	 *	@param[in,out] r_omega is the omega matrix
	 *	@param[in] n_min_vertex_order is minimum vertex order of vertices in omega
	 *	@param[in] tag0 is tag for tag-based dispatch (value unuesd at runtime)
	 *	@param[in] tag1 is tag for tag-based dispatch (value unuesd at runtime)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <const int n_vertex1, const int n_vertex0>
	inline void Get_OmegaBlocks_v2_Inner(CUberBlockMatrix &r_omega, size_t n_min_vertex_order,
		fbs_ut::CCTSize<n_vertex1> UNUSED(tag0), fbs_ut::CCTSize<n_vertex0> tag1) const // throw(std::bad_alloc)
	{
		// we need to allocate a block between n_vertex1 and n_vertex0

		_ASSERTE(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()); // make sure that n_vertex0 is not constant

		if(!m_vertex_ptr.template Get<n_vertex1>()->b_IsConstant()) { // if n_vertex1 is not constant
			_ASSERTE((m_p_vertex_id[n_vertex0] > m_p_vertex_id[n_vertex1]) ==
				(m_vertex_ptr.template Get<n_vertex0>()->n_Order() > m_vertex_ptr.template Get<n_vertex1>()->n_Order()));
			// if this triggers, then the edge has the vertices assigned in a different
			// order than the ids (vertex[0] is id[1] and vice versa) consequently, the Hessians
			// will have correct shape in the matrix, but the data will be transposed
			// and you will get either not pos def / rubbish solutions

			enum {
				n_block = hyperedge_detail::CStrictlyTriangularOffset<n_vertex0, n_vertex1, n_vertex_num>::n_result,
				n_block_dim0 = CVertexTraits<n_vertex0>::n_dimension,
				n_block_dim1 = CVertexTraits<n_vertex1>::n_dimension
			};
			// calculate index of the block (among all upper triangular blocks of this edge)

			size_t n_dimension0 = n_block_dim0, n_dimension1 = n_block_dim1;
			size_t n_order_0 = m_vertex_ptr.template Get<n_vertex0>()->n_Order();
			size_t n_order_1 = m_vertex_ptr.template Get<n_vertex1>()->n_Order();
			if(n_order_0 > n_order_1) {
				std::swap(n_order_0, n_order_1);
				std::swap(n_dimension0, n_dimension1);
			}
			// make sure the order is sorted (if swapping, will have to transpose the result,
			// but we will deal with that laters)

			_ASSERTE(n_order_0 != n_order_1); // will otherwise overwrite blocks with vertex blocks (malformed system)
			_ASSERTE(n_order_0 < n_order_1);
			// they dont care about the man that ends up under the stage, they only care about the one above (column > row)

			_ASSERTE(n_order_0 >= n_min_vertex_order && n_order_1 >= n_min_vertex_order);
			double *p_block = r_omega.p_FindBlock(n_order_0 - n_min_vertex_order,
				n_order_1 - n_min_vertex_order, n_dimension0, n_dimension1, true, true);
			// find a block for Hessian above the diagonal, and with the right shape

			_ASSERTE(p_block && m_p_HtSiH[n_block]);
			// if this triggers, most likely this is called before Alloc_HessianBlocks() was called

			Eigen::Map<Eigen::Matrix<double, n_block_dim0, n_block_dim1> > dest_map(p_block); // note that we ignore the transpose-ness here, it actually might be n_block_dim1 * n_block_dim0, but it does not matter as the elements are tightly packed
			Eigen::Map<const Eigen::Matrix<double, n_block_dim0, n_block_dim1> > src_map(m_p_HtSiH[n_block]); // note that we ignore the transpose-ness here, it actually might be n_block_dim1 * n_block_dim0, but it does not matter as the elements are tightly packed
			dest_map += src_map;
			// use Eigen for addition (assumes that the Hessians are up to date)
		}

		Get_OmegaBlocks_v2_Inner(r_omega, n_min_vertex_order, fbs_ut::CCTSize<n_vertex1 + 1>(), tag1);
		// loop
	}

	/**
	 *	@brief inner loop for omega block allocation (specialization for the end of the loop)
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (inside this edge)
	 *
	 *	@param[in,out] r_omega is the omega matrix (unused)
	 *	@param[in] n_min_vertex_order is minimum vertex order of vertices in omega (unused)
	 *	@param[in] tag0 is tag for tag-based dispatch (value unuesd at runtime)
	 *	@param[in] tag1 is tag for tag-based dispatch (value unuesd at runtime)
	 */
	template <const int n_vertex0>
	inline void Get_OmegaBlocks_v2_Inner(CUberBlockMatrix &UNUSED(r_omega), size_t UNUSED(n_min_vertex_order),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag0), fbs_ut::CCTSize<n_vertex0> UNUSED(tag1)) const
	{}

	/**
	 *	@brief outer loop for omega block allocation
	 *
	 *	@tparam n_vertex0 is zero-based index of the first vertex (inside this edge)
	 *
	 *	@param[in,out] r_omega is the omega matrix
	 *	@param[in] n_min_vertex_order is minimum vertex order of vertices in omega
	 *	@param[in] tag is tag for tag-based dispatch (value unuesd at runtime)
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <const int n_vertex0>
	inline void Get_OmegaBlocks_v2(CUberBlockMatrix &r_omega, size_t n_min_vertex_order,
		fbs_ut::CCTSize<n_vertex0> tag) const // throw(std::bad_alloc)
	{
		if(!m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant()) { // if the vertex is not constant
			size_t n_order_0 = m_vertex_ptr.template Get<n_vertex0>()->n_Order();
			enum {
				n_dimension0 = CVertexTraits<n_vertex0>::n_dimension
			};

			_ASSERTE(n_order_0 >= n_min_vertex_order);
			double *p_block = r_omega.p_FindBlock(n_order_0 - n_min_vertex_order,
				n_order_0 - n_min_vertex_order, n_dimension0, n_dimension0, true, true);
			// find a block for vertices' Hessian on the diagonal (do not use the potentially swapped id / order)
			// note that if this is added after the off-diagonal Hessian, it will be likely
			// added at the end of the block list in the matrix

			_ASSERTE(p_block && m_p_HtSiH_vert[n_vertex0]);
			// if this triggers, most likely this is called before Alloc_HessianBlocks() was called

			Eigen::Map<Eigen::Matrix<double, n_dimension0, n_dimension0> > dest_map(p_block);
			Eigen::Map<const Eigen::Matrix<double, n_dimension0, n_dimension0> > src_map(m_p_HtSiH_vert[n_vertex0]);
			dest_map += src_map;
			// use Eigen for addition (assumes that the Hessians are up to date)

			Get_OmegaBlocks_v2_Inner(r_omega, n_min_vertex_order, fbs_ut::CCTSize<n_vertex0 + 1>(), tag);
			// alloc off-diagonal blocks
		}

		Get_OmegaBlocks_v2(r_omega, n_min_vertex_order, fbs_ut::CCTSize<n_vertex0 + 1>());
		// loop
	}

	/**
	 *	@brief outer loop for omega block allocation (specialization for the end of the loop)
	 *
	 *	@param[in,out] r_omega is the omega matrix (unused)
	 *	@param[in] n_min_vertex_order is minimum vertex order of vertices in omega (unused)
	 *	@param[in] tag is tag for tag-based dispatch (value unuesd at runtime)
	 */
	inline void Get_OmegaBlocks_v2(CUberBlockMatrix &UNUSED(r_omega), size_t UNUSED(n_min_vertex_order),
		fbs_ut::CCTSize<n_vertex_num> UNUSED(tag)) const
	{}

#endif // __LAMBDA_USE_V2_REDUCTION_PLAN

public:
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
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN

		bool b_recalculate = false;
		// recalculate from scratch, or use values already calculated by the lambda solver?

		_ASSERTE(!b_recalculate);
		// this only copies the already calculated Hessians into omega, doesn't actually
		// calculate anything (would be trivial to add should it ever be needed; but for
		// the binary edge it was never needed so far)

		Get_OmegaBlocks_v2(r_omega, n_min_vertex_order, fbs_ut::CCTSize<0>());
		// loops for many vertices

#else // __LAMBDA_USE_V2_REDUCTION_PLAN
		_ASSERTE(!b_is_robust_edge); // robust edges not supported here

		//_ASSERTE(r_omega.b_Empty()); // should be empty // can't assert that here, other edges might have added something already

		_ASSERTE(n_vertex_num == 2);
		if(n_vertex_num != 2)
			throw std::runtime_error("the fastL solver cannot be used with hyperedges just yet");
		// todo - will have to separate writing diagonal Hessians and off-diagonal ones, then split to loops

		enum {
			n_vertex0 = 0, n_vertex1 = 1, // dummy
		};

		if(m_vertex_ptr.template Get<n_vertex0>()->b_IsConstant() ||
		   m_vertex_ptr.template Get<n_vertex1>()->b_IsConstant()) // if either vertex is constant
			throw std::runtime_error("v1 reduction plan does not support const vertices in hyperedges");

		enum {
			n_block = hyperedge_detail::CStrictlyTriangularOffset<n_vertex0, n_vertex1, n_vertex_num>::n_result
		};
		// calculate index of the block (among all upper triangular blocks of this edge)

		bool b_transpose_hessian;
		size_t n_dimension0 = CVertexTraits<n_vertex0>::n_dimension;
		size_t n_dimension1 = CVertexTraits<n_vertex1>::n_dimension;

		size_t n_order_0 = m_vertex_ptr.template Get<n_vertex0>()->n_Order();
		size_t n_order_1 = m_vertex_ptr.template Get<n_vertex1>()->n_Order();
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
		// alloc and initialize / find existing blocks for all the Hessians, above the diagonal only

		bool b_recalculate = false;
		// recalculate from scratch, or use values already calculated by the lambda solver?

		_TyJacobianTuple jacobian_tuple;
		Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension, n_residual_dimension> t_H0_sigma_inv;
		if(b_recalculate) {
			_TyVector v_expectation, v_error;
			((CDerivedEdge*)this)->Calculate_Jacobians_Expectation_Error(jacobian_tuple, v_expectation, v_error);
			t_H0_sigma_inv = jacobian_tuple.template Get<n_vertex0>().transpose() * m_t_sigma_inv;
		}
		// calculates the Jacobians and precalculate H0 * sigma^-1

		if(b_transpose_hessian) {
			Eigen::Map<Eigen::Matrix<double, CVertexTraits<n_vertex1>::n_dimension,
				CVertexTraits<n_vertex0>::n_dimension>,
				CUberBlockMatrix::map_Alignment> t_HtSiH(p_edge); // t_odo - support aligned if the uberblockmatrix is aligned!
			// map the matrix above diagonal

			if(b_recalculate)
				t_HtSiH.noalias() += jacobian_tuple.template Get<n_vertex1>().transpose() * t_H0_sigma_inv.transpose();
			else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<1>::n_dimension,
					CVertexTraits<n_vertex0>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH_lambda(m_p_HtSiH[n_block]);
				t_HtSiH += t_HtSiH_lambda;
			}
		} else {
			Eigen::Map<Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension,
				CVertexTraits<n_vertex1>::n_dimension>,
				CUberBlockMatrix::map_Alignment> t_HtSiH(p_edge); // t_odo - support aligned if the uberblockmatrix is aligned!
			// map the matrix above diagonal

			if(b_recalculate)
				t_HtSiH.noalias() += t_H0_sigma_inv * jacobian_tuple.template Get<n_vertex1>();
			else {
				Eigen::Map<Eigen::Matrix<double, CVertexTraits<n_vertex0>::n_dimension,
					CVertexTraits<n_vertex1>::n_dimension>,
					CUberBlockMatrix::map_Alignment> t_HtSiH_lambda(m_p_HtSiH[n_block]);
				t_HtSiH += t_HtSiH_lambda;
			}
		}
		// calculate the block above the diagonal

		Eigen::Map<typename CVertexTraits<n_vertex0>::_TyMatrix,
			CUberBlockMatrix::map_Alignment> t_HtSiH_v0((b_transpose_hessian)? p_v1 : p_v0);
		Eigen::Map<typename CVertexTraits<n_vertex1>::_TyMatrix,
			CUberBlockMatrix::map_Alignment> t_HtSiH_v1((b_transpose_hessian)? p_v0 : p_v1);
		if(b_recalculate) {
			t_HtSiH_v0.noalias() += t_H0_sigma_inv * jacobian_tuple.template Get<n_vertex0>();
			t_HtSiH_v1.noalias() += jacobian_tuple.template Get<n_vertex1>().transpose() *
				m_t_sigma_inv * jacobian_tuple.template Get<n_vertex1>();
		} else {
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
			Eigen::Map<typename CVertexTraits<n_vertex0>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex0(m_p_HtSiH_vert[n_vertex0]);
			Eigen::Map<typename CVertexTraits<n_vertex1>::_TyMatrix,
				CUberBlockMatrix::map_Alignment> t_HtSiH_vertex1(m_p_HtSiH_vert[n_vertex1]);
			t_HtSiH_v0 += t_HtSiH_vertex0;
			t_HtSiH_v1 += t_HtSiH_vertex1;
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
			t_HtSiH_v0 += m_t_HtSiH_vertex0;
			t_HtSiH_v1 += m_t_HtSiH_vertex1;
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
		}
		// calculate diagonal Hessian contributions (note the increments)
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
	}
};

/**
 *	@brief implementation of solver required functions for a generic edge type
 *
 *	@tparam CDerivedEdge is the name of derived edge class
 *	@tparam CVertexTypeList is list of types of the vertices
 *	@tparam _n_residual_dimension is residual vector dimension
 *	@tparam _n_storage_dimension is state vector storage dimension (or -1 if the same as _n_residual_dimension)
 *
 *	@deprecated This name is deprecated; use CBaseEdgeImpl instead.
 */
template <class CDerivedEdge, class CVertexTypeList, int _n_residual_dimension, int _n_storage_dimension = -1>
class CSEBaseEdgeImpl : public CBaseEdgeImpl<CDerivedEdge, CVertexTypeList, _n_residual_dimension, _n_storage_dimension> {
private:
	typedef CBaseEdgeImpl<CDerivedEdge, CVertexTypeList, _n_residual_dimension,
		_n_storage_dimension> _TyNotDeprecated; /**< @brief name of the parent type that is not deprecated */
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
	 *	@brief constructor
	 *
	 *	@param[in] vertex_id_tuple is tuple of vertex ids
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
	inline CSEBaseEdgeImpl(typename _TyNotDeprecated::_TyVertexIndexTuple vertex_id_tuple,
		const typename _TyNotDeprecated::_TyStorageVector &r_v_measurement,
		const typename _TyNotDeprecated::_TyMatrix &r_t_sigma_inv)
		:_TyNotDeprecated(vertex_id_tuple, r_v_measurement, r_t_sigma_inv)
	{}
};

#include "slam/BaseTypes_Unary.h"
#include "slam/BaseTypes_Binary.h"
// include the specializations

/** @} */ // end of group

#endif // !__BASE_GRAPH_PRIMITIVE_TYPES_INCLUDED
