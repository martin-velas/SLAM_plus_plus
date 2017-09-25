/*
								+----------------------------------+
								|                                  |
								|  ***  Base types interface  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|          BaseInterface.h         |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __BASE_SE_TYPES_INTERFACE_INCLUDED
#define __BASE_SE_TYPES_INTERFACE_INCLUDED

/**
 *	@file include/slam/BaseInterface.h
 *	@brief base primitive types interface
 *	@author -tHE SWINe-
 *	@date 2014-05-20
 */

/**
 *	@page facades Graph Element Interfaces
 *
 *	Since version 1.5, SLAM++ uses some advanced trickery for interfacing with graph elements (edges
 *	and vertices). In the older versions, the vertices and edges contained virtual functions, so that
 *	one could take any pointer to a graph element and call a function, and the correct implementation
 *	would be called. While very convenient, this has the disadvantage of cache thrashing: each vertex
 *	and edge needs to carry pointer to the virtual function table (which may be the same for all the
 *	vertices / edges) and depending on the compiler also offset for the <tt>this</tt> pointer. When
 *	iterating e.g. through vertex states, these pointers are not needed but need to be read as well,
 *	reducing the useful memory bandwidth. This is where element interfaces come in.
 *
 *	Currently, the vertex and edge types have no virtual functions whatsoever. In case the graph contains
 *	only a single kind of vertex or edge, the same type must be used as the base type, and there is nothing
 *	to take care about - the specific type is known at compile time and all is good. Trouble comes when
 *	there are multiple different types. One option would be to use the nonstandard RTTI extension (run time
 *	type info) but that would be enabled globally for all types, and that is not needed. Therefore SLAM++
 *	vertex and edge pools store ad-hoc type info and use facades for interfacing with vertex and edge types.
 *	With conjunction with some other optimizations this saves about 20% of time on the Venice dataset.
 *
 *	A facade is an object which wraps the graph element and has the same interface, but it is not the element
 *	nor it is a descendant of any vertex / edge class. While this may seem complex, it is actually quite
 *	simple to use. Let's assume a similar code as @ref simpleexample uses:
 *
 *	@code
 *	#include <stdio.h> // printf
 *	#include "slam/LinearSolver_UberBlock.h" // linear solver
 *	#include "slam/ConfigSolvers.h" // nonlinear graph solvers
 *	#include "slam/SE2_Types.h" // SE(2) types
 *
 *	int main(int n_arg_num, const char **p_arg_list)
 *	{
 *		typedef MakeTypelist(CVertexPose2D, CVertexLandmark2D) TVertexTypelist;
 *		typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;
 *
 *		typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		// there are multiple vertex types, CBaseVertex must be used as a common base
 *			CEdgePose2D, TEdgeTypelist>
 *		// there is only a single edge (CEdgePose2D) and it must be used as its base (otherwise compilation error ensues)
 *			CSystemType;
 *
 *		typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
 *		typedef CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> CNonlinearSolverType;
 *
 *		CSystemType system;
 *		CNonlinearSolverType solver(system);
 *
 *		// [add edges / vertices]
 *
 *		solver.Optimize();
 *	}
 *	@endcode
 *
 *	This is all nice and simple, we declared a simple graph with some SE(2) vertices and a single edge.
 *	Note that there is no edge to connect pose and landmark vertex and this is intentional - this is just
 *	a toy example.
 *
 *	Now, if one wants to access edges, matters are simple: there is only a single edge type, so that:
 *
 *	@code
 *	CSystemType::_TyEdgeMultiPool &edge_pool = system.r_Edge_Pool();
 *	// pool with all the edges, indexable by zero-based edge id
 *
 *	const CEdgePose2D &edge = edge_pool[123];
 *	// simple; operator [] returns reference to CEdgePose2D, can do anything we want with it
 *	@endcode
 *
 *	This is very simple, we get only a single edge type and there is nothing to worry about. The situation
 *	is slightly more complicated with vertices:
 *
 *	@code
 *	CSystemType::_TyVertexMultiPool &vertex_pool = system.r_Vertex_Pool();
 *	// pool with all the vertices, indexable by zero-based vertex id
 *
 *	CSystemType::_TyConstVertexRef vertex = vertex_pool[123];
 *	// what type is vertex 123? it depends on the input graph
 *	@endcode
 *
 *	This is a tad more complicated. When accessing a vertex with some id, one may not be sure what type
 *	that vertex is: it could be CVertexPose2D or CVertexLandmark2D. Since there are no virtual functions,
 *	the base class CBaseVertex is essentially empty, and there is not much that one can do with it.
 *	Note the use of the CFlatSystem::_TyConstVertexRef typedef. This is defined for any pool (for example
 *	<tt>CSystemType::_TyConstEdgeRef</tt> would evaluate simply to <tt>const CEdgePose2D&</tt>).
 *	In the case of vertices, however, this evaluates to a facade object. It has all the functions of
 *	the objects, but it is in fact a wrapper class. One can do:
 *
 *	@code
 *	Eigen::VectorXd state = vertex.v_State();
 *	// get state vector
 *	@endcode
 *
 *	An important point to make is that the facade object does not have the same address as the original
 *	vertex object. So in cases when you know what type a specific vertex is, this is how to get it:
 *
 *	@code
 *	const CVertexPose2D &as_2D_pose_flawed_a = (const CVertexPose2D&)vertex;
 *	// this will not compile, vertex is a facade object, which is not related to the vertex types
 *
 *	const CVertexPose2D &as_2D_pose_flawed_b = *(const CVertexPose2D*)&vertex;
 *	// will not work, vertex is a facade object, which does not have the same address as the original vertex
 *
 *	const CVertexPose2D &as_2D_pose = (const CVertexPose2D&)vertex.r_Get();
 *	// this will get the correct vertex; base_iface::CVertexFacade::r_Get returns reference to base vertex
 *	@endcode
 *
 *	While the latter approach works correctly, there is no type checking. If the vertex 123 is in fact
 *	a CVertexLandmark2D, noone will know until the program crashes (in the better case). Therefore,
 *	a better way of doing it would be:
 *
 *	@code
 *	const CVertexPose2D &as_2D_pose_better = vertex_pool.r_At<CVertexPose2D>(123);
 *	// this will get the vertex as CVertexPose2D and assert that it is indeed of that type
 *	@endcode
 *
 *	This will (in debug mode) contain assertions on type id, and will prompt if the given vertex is not
 *	of the expected type.
 *
 *	One last possibility of accessing the graph elements is when the type of a particular element is unknown
 *	but is needed. This can also be achieved, using a function object and the for-each functionality:
 *
 *	@code
 *	struct MyFunctionObject {
 *		template <class CVertexType>
 *		void operator ()(const CVertexType &vertex) const
 *		{
 *			// vertex has the final type here (either CVertexPose2D or CVertexLandmark2D in our example)
 *		}
 *	};
 *
 *	vertex_pool.For_Each(123, 123 + 1, MyFunctionObject());
 *	// runs the function on vertex 123. MyFunctionObject::operator () is called, with that vertex as an argument.
 *	@endcode
 *
 *	The multipool::CMultiPool::For_Each() makes use of the internal type id information and calls the function
 *	operator with the appropriate type. If different code paths are required for different types of vertices,
 *	it is possible to simply overload this operator:
 *
 *	@code
 *	struct MyFunctionObject {
 *		void operator ()(const CVertexPose2D &vertex) const
 *		{
 *			printf("vertex %d is a pose\n", int(vertex.n_Id()));
 *		}
 *
 *		void operator ()(const CVertexLandmark2D &vertex) const
 *		{
 *			printf("vertex %d is a landmark\n", int(vertex.n_Id()));
 *		}
 *
 *		template <class CVertexType>
 *		void operator ()(const CVertexType &vertex) const
 *		{
 *			printf("vertex %d is a different type\n", int(vertex.n_Id()));
 *		}
 *		// In case you dont want to specify all the possible types, this will catch any
 *		// of the types not explicitly specified (similar to using default in a switch).
 *		// Note that this is not needed if you specify all the types - then it will, in
 *		// fact, never be called.
 *	};
 *
 *	vertex_pool.For_Each(123, 123 + 1, MyFunctionObject());
 *	// runs the function on vertex 123. MyFunctionObject::operator () is called, with that vertex as an argument.
 *	@endcode
 *
 *	This prints the type of vertex 123. Of course, it is possible to use the for each approach
 *	with a set of vertices (or even all the vertices), not with just a single vertex.
 *
 *	\section conc Conclusions
 *
 *	A few important points to take away.
 *
 *	The functionality for vertices and edges is the same, there are
 *	no differences (the differences in the above example were given by the configuration of the system,
 *	or more precisely by the number of vertices and edges in the system).
 *
 *	* If you know the type, use multipool::CMultiPool::r_At().
 *	* If you dont know the type and dont need it, use multipool::CMultiPool::operator[]() which returns an iterface.
 *		* Note that the interface does not have the same address as the object. If you need it, use <tt>r_Get()</tt>
 *		  (e.g. \ref base_iface::CConstVertexFacade::r_Get(), \ref base_iface::CVertexFacade::r_Get(),
 *		  \ref base_iface::CConstEdgeFacade::r_Get() or \ref base_iface::CEdgeFacade::r_Get()).
 *	* If you dont know the type but need it, use multipool::CMultiPool::For_Each() along with a function object.
 *
 *	All those, with the exception of <tt>r_Get()</tt> will also work in case there is only a single vertex / edge
 *	type and no interface wrapping is needed (this makes writing generic code a bit easier). In case you need
 *	to use <tt>r_Get()</tt> in generic code where types without interfaces may be used, you can use
 *	<tt>base_iface::r_GetBase()</tt> to convert any vertex or edge object to the base type.
 *
 */

#include <map>

/** \addtogroup graph
 *	@{
 */

/**
 *	@def __FLAT_SYSTEM_USE_THUNK_TABLE
 *	@brief if defined, thunk tables are used for virtual call evasion in vertex and edge types
 */
#define __FLAT_SYSTEM_USE_THUNK_TABLE

/**
 *	@def __FLAT_SYSTEM_STATIC_THUNK_TABLE
 *	@brief if defined, the thunk table is statically linked (and the functions are potentially inlined)
 *	@note This only takes effect it __FLAT_SYSTEM_USE_THUNK_TABLE is defined.
 */
#define __FLAT_SYSTEM_STATIC_THUNK_TABLE

#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE

/**
 *	@def __LAMBDA_USE_V2_REDUCTION_PLAN
 *	@brief if defined, CNonlinearSolver_Lambda and derived will use the new v2 reduction plan
 *	@note This is only available if __FLAT_SYSTEM_USE_THUNK_TABLE is defined.
 */
#define __LAMBDA_USE_V2_REDUCTION_PLAN

#endif // __FLAT_SYSTEM_USE_THUNK_TABLE

// todo - revise the include chain

class CBaseVertex; // forward declaration
class CBaseEdge; // forward declaration
// base types of all edges / vertices, implementing pretty much nothing

// todo - fix broken doxygen links

/**
 *	@brief namespace, containing internal classes of detached facades and thunk table helpers
 */
namespace base_iface {

/**
 *	@brief vertex polymorphic interface base
 */
class CVertexInterface_Base {
public:
#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns vertex state vector.
	 */
	virtual Eigen::Map<const Eigen::VectorXd> v_StateC() const = 0;

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns vertex state vector.
	 */
	virtual Eigen::Map<Eigen::VectorXd> v_State() = 0;

	/**
	 *	@brief gets vertex dimension
	 *	@return Returns vertex dimension.
	 */
	virtual size_t n_Dimension() const = 0;

	/**
	 *	@brief gets vertex order
	 *	@return Returns vertex order (column in a matrix where the vertex block is inseted).
	 */
	virtual size_t n_Order() const = 0;

	/**
	 *	@brief determines whether this vertex is a constant vertex
	 *	@return Returns true if this vertex is a constant vertex, otherwise returns false.
	 */
	virtual bool b_IsConstant() const = 0;

	/**
	 *	@brief sets vertex order
	 *	@param[in] n_first_element_index is vertex order
	 *		(column in a matrix where the vertex block is inserted).
	 */
	virtual void Set_Order(size_t n_first_element_index) = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets vertex id
	 *	@return Returns vertex order (block column in a matrix where the vertex block is inseted).
	 */
	virtual size_t n_Id() const = 0;

	/**
	 *	@brief sets vertex id
	 *	@param[in] n_id is vertex id
	 *		(block column in a matrix where the vertex block is inseted).
	 */
	virtual void Set_Id(size_t n_id) = 0;

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief swaps solution vector with vertex state and minds the manifold space
	 *	@param[in,out] r_v_x is solution vector
	 *	@note This is required by the experimental SPCG solver, no need to implement
	 *		it everywhere (now it is just in SE(2)).
	 */
	virtual inline void SwapState(Eigen::VectorXd &r_v_x) = 0;

	/**
	 *	@brief saves the vertex state in a vector
	 *	@param[out] r_v_x is the state vector to copy the state to
	 */
	virtual inline void SaveState(Eigen::VectorXd &r_v_x) const = 0;

	/**
	 *	@brief restores the vertex state from a vector
	 *	@param[out] r_v_x is the state vector to copy the state from
	 */
	virtual inline void LoadState(const Eigen::VectorXd &r_v_x) = 0;

	/**
	 *	@brief adds delta vector to vertex state and minds the manifold space ("smart" plus)
	 *	@param[in] r_v_delta is delta vector
	 */
	virtual inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) = 0;

	/**
	 *	@brief subtracts delta vector to vertex state and minds the manifold space ("smart" minus)
	 *	@param[in] r_v_delta is delta vector
	 */
	virtual inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) = 0; // "smart" minus

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief vertex polymorphic interface for lambda solver using the v1 reduction plan
 */
class CVertexPrerequisities_Lambda_v1_ReductionPlan {
public:
	typedef CBaseEdge _TyBaseEdge; /**< @brief base edge type, assumed for this vertex */

protected:
	std::vector<_TyBaseEdge*> m_edge_list; /**< @brief a list of edges, referencing this vertex (only used by Lambda solver) */

public:
	/**
	 *	@brief adds an edge that references this vertex
	 *
	 *	@param[in] p_edge is an edge that references this vertex (must be called
	 *		only once with each edge to avoid duplicates)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	inline void Add_ReferencingEdge(_TyBaseEdge *p_edge) // throw(std::bad_alloc)
	{
		_ASSERTE(!b_IsReferencingEdge(p_edge));
		m_edge_list.push_back(p_edge);
	}

	/**
	 *	@brief checks if an edge references this vertex
	 *	@param[in] p_edge is pointer to an edge
	 *	@return Returns true if the given edge is in the list
	 *		of referencing edges, otherwise returns false.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	inline bool b_IsReferencingEdge(const _TyBaseEdge *p_edge) const
	{
		return std::find(m_edge_list.begin(), m_edge_list.end(), p_edge) != m_edge_list.end();
	}

	/**
	 *	@brief gets list of referencing edges
	 *	@return Returns const reference to the list of referencing edges.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	inline const std::vector<_TyBaseEdge*> &r_ReferencingEdge_List() const
	{
		return m_edge_list;
	}

#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	/**
	 *	@brief allocates Hessian blocks
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual inline void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda) = 0;

	/**
	 *	@brief calculates diagonal lambda Hessian blocks
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual inline void Calculate_Hessians() = 0;

	/**
	 *	@brief gets a portion of right-hand side vector, associated with this vertex
	 *	@param[out] r_v_eta is the right-hand side vector
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual inline void Get_RightHandSide_Vector(Eigen::VectorXd &r_v_eta) const = 0;

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief vertex polymorphic interface for lambda solvers
 */
class CVertexInterface_Lambda { // nothing
public:
};

/**
 *	@brief vertex polymorphic interface for L solvers
 */
class CVertexInterface_L {
public:
#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	/**
	 *	@brief allocates L factor blocks
	 *	@param[out] r_L is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_L.
	 */
	virtual inline void Alloc_LBlocks(CUberBlockMatrix &r_L) const = 0;

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief edge polymorphic interface base
 */
class CEdgeInterface_Base {
public:
#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	/**
	 *	@brief gets edge dimension
	 *	@return Returns edge dimension.
	 */
	virtual size_t n_Dimension() const = 0;

	/**
	 *	@brief gets number of edge vertices
	 *	@return Returns number of edge vertices (2 in binary edges).
	 */
	virtual size_t n_Vertex_Num() const = 0;

	/**
	 *	@brief gets id of edge vertex
	 *	@param[in] n_vertex is zero-based vertex index
	 *	@return Returns id of the selected edge vertex.
	 */
	virtual size_t n_Vertex_Id(size_t n_vertex) const = 0;

	/**
	 *	@brief gets edge order
	 *	@return Returns edge order (row in a matrix where the edge block is inseted).
	 */
	virtual size_t n_Order() const = 0;

	/**
	 *	@brief sets edge order
	 *	@param[in] n_first_element_index is edge order
	 *		(row in a matrix where the edge block is inseted).
	 */
	virtual void Set_Order(size_t n_first_element_index) = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets edge id
	 *	@return Returns edge id (block row in a matrix where the edge block is inseted).
	 */
	virtual size_t n_Id() const = 0;

	/**
	 *	@brief sets edge order
	 *	@param[in] n_id is edge order
	 *		(block row in a matrix where the edge block is inseted).
	 */
	virtual void Set_Id(size_t n_id) = 0;

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error.
	 *	@note This is supposed to be divided by (m - n), where m is number of edges
	 *		and n is number of degrees of freedom (equals to n_Dimension()).
	 */
	virtual double f_Chi_Squared_Error() const = 0;

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
	virtual void Calculate_Omega(CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const = 0;

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief edge polymorphic interface for A solvers
 */
class CEdgeInterface_A {
public:
#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	/**
	 *	@brief allocates Jacobian blocks
	 *	@param[out] r_A is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Alloc_JacobianBlocks(CUberBlockMatrix &r_A) = 0;

	/**
	 *	@brief calculates Jacobians
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Calculate_Jacobians() = 0;

	/**
	 *	@brief calculates error, multiplied by the R matrix
	 *		(upper diagonal Choleski of inverse sigma matrix)
	 *	@param[out] r_v_dest is the error vector
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Get_R_Error(Eigen::VectorXd &r_v_dest) const = 0;

	/**
	 *	@brief calculates bare error
	 *	@param[out] r_v_dest is the error vector
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Get_Error(Eigen::VectorXd &r_v_dest) const = 0;

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief edge polymorphic interface for lambda solvers using the v1 reduction plan
 */
class CEdgePrerequisities_Lambda_v1_ReductionPlan { // this is always there, with or without the thunk table
public:
	/**
	 *	@brief allocates Hessian blocks
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda) = 0;

	/**
	 *	@brief calculates off-diagonal lambda Hessian blocks
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Calculate_Hessians() = 0;

	/**
	 *	@brief notifies an edge of a conflict (duplicate edge)
	 *
	 *	@param[in] p_block is pointer to block data (directly in the matrix, as returned by CUberBlockMatrix::p_GetBlock() or the like)
	 *	@param[in] r_lambda is reference to the Hessian matrix
	 *
	 *	@return Returns true if this is the original owner of the block.
	 */
	virtual bool Notify_HessianBlock_Conflict(double *p_block, CUberBlockMatrix &r_lambda) = 0;

	/**
	 *	@brief gets edge contributions for lambda and eta blocks
	 *	@param[in] p_which is vertex, associated with queried contributions
	 *	@return Returns a pair of pointers to memory blocks where lambda contribution
	 *		and eta contribution are stored.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual std::pair<const double*, const double*>
		t_Get_LambdaEta_Contribution(const CBaseVertex *p_which) = 0;
};

/**
 *	@brief edge polymorphic interface for lambda solvers
 */
class CEdgeInterface_Lambda {
public:
#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	virtual double f_Max_VertexHessianDiagValue() const = 0; // todo surround with levenberg support ifdef

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief edge polymorphic interface for L solvers
 */
class CEdgeInterface_L {
public:
#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE

	/**
	 *	@brief allocates L factor blocks
	 *	@param[out] r_L is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_L.
	 */
	virtual void Alloc_LBlocks(CUberBlockMatrix &r_L) const = 0;

#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE
};

/**
 *	@brief solver suport flags converted to enums
 */
enum {
#ifdef __SE_TYPES_SUPPORT_A_SOLVERS
	support_A = true, /**< @brief A solver support flag */
#else // __SE_TYPES_SUPPORT_A_SOLVERS
	support_A = false, /**< @brief A solver support flag */
#endif // __SE_TYPES_SUPPORT_A_SOLVERS
#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
	support_Lambda = true, /**< @brief lambda solver support flag */
#ifdef __LAMBDA_USE_V2_REDUCTION_PLAN
	lambda_ReductionPlan_v2 = true, /**< @brief lambda v2 reduction plan flag */
#else // __LAMBDA_USE_V2_REDUCTION_PLAN
	lambda_ReductionPlan_v2 = false, /**< @brief lambda v2 reduction plan flag */
#endif // __LAMBDA_USE_V2_REDUCTION_PLAN
#else // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
	support_Lambda = false, /**< @brief lambda solver support flag */
	lambda_ReductionPlan_v2 = false, /**< @brief lambda v2 reduction plan flag */ // does not really matter as lambda is not supported
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS
#ifdef __SE_TYPES_SUPPORT_L_SOLVERS
	support_L = true /**< @brief L solver support flag */
#else // __SE_TYPES_SUPPORT_L_SOLVERS
	support_L = false /**< @brief L solver support flag */
#endif // __SE_TYPES_SUPPORT_L_SOLVERS
};
// convert flags to enums

/**
 *	@brief configured vertex interface
 *
 *	@tparam b_support_A is A solver support flag
 *	@tparam b_support_Lambda is lambda solver support flag
 *	@tparam b_lambda_reduction_v2 is lambda v2 reduction plan flag
 *	@tparam b_support_L is L solver support flag
 */
template <bool b_support_A = support_A, bool b_support_Lambda = support_Lambda,
	bool b_lambda_reduction_v2 = lambda_ReductionPlan_v2, bool b_support_L = support_L>
class CVertexInterface_Config : public CVertexInterface_Base,
	public CInheritIf<CVertexInterface_Lambda, b_support_Lambda>,
	public CInheritIf<CVertexPrerequisities_Lambda_v1_ReductionPlan, b_support_Lambda && !b_lambda_reduction_v2>,
	public CInheritIf<CVertexInterface_L, b_support_L> {
};
// conditionally inherit different parts of the interface

/**
 *	@brief configured edge interface
 *
 *	@tparam b_support_A is A solver support flag
 *	@tparam b_support_Lambda is lambda solver support flag
 *	@tparam b_lambda_reduction_v2 is lambda v2 reduction plan flag
 *	@tparam b_support_L is L solver support flag
 */
template <bool b_support_A = support_A, bool b_support_Lambda = support_Lambda,
	bool b_lambda_reduction_v2 = lambda_ReductionPlan_v2, bool b_support_L = support_L>
class CEdgeInterface_Config : public CEdgeInterface_Base,
	public CInheritIf<CEdgeInterface_A, b_support_A>,
	public CInheritIf<CEdgeInterface_Lambda, b_support_Lambda>,
	public CInheritIf<CEdgePrerequisities_Lambda_v1_ReductionPlan, b_support_Lambda && !b_lambda_reduction_v2>,
	public CInheritIf<CEdgeInterface_L, b_support_L> {
};
// conditionally inherit different parts of the interface

} // ~base_iface

typedef base_iface::CVertexInterface_Config<> CVertexInterface; /**< @brief configured vertex interface */
typedef base_iface::CEdgeInterface_Config<> CEdgeInterface; /**< @brief configured edge interface */
// those are used to make some vertex / edge functions virtual, in case thunk tables are not used
// if thunk tables are used, those are empty

#if 0 // do we need to keep this around?
class CVertexInterface { // this is likely going to be unused, remove it later, copy doc somewhere else
public:
	typedef CBaseEdge _TyBaseEdge; // todo - will get rid of that

public:
	/**
	 *	@brief gets vertex state vector
	 *	@return Returns vertex state vector.
	 */
	virtual Eigen::Map<const Eigen::VectorXd> v_StateC() const = 0;

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns vertex state vector.
	 */
	virtual Eigen::Map<Eigen::VectorXd> v_State() = 0;

	/**
	 *	@brief gets vertex dimension
	 *	@return Returns vertex dimension.
	 */
	virtual size_t n_Dimension() const = 0;

	/**
	 *	@brief gets vertex order
	 *	@return Returns vertex order (column in a matrix where the vertex block is inseted).
	 */
	virtual size_t n_Order() const = 0;

	/**
	 *	@brief determines whether this vertex is a constant vertex
	 *	@return Returns true if this vertex is a constant vertex, otherwise returns false.
	 */
	virtual bool b_IsConstant() const = 0;

	/**
	 *	@brief sets vertex order
	 *	@param[in] n_first_element_index is vertex order
	 *		(column in a matrix where the vertex block is inserted).
	 */
	virtual void Set_Order(size_t n_first_element_index) = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets vertex id
	 *	@return Returns vertex order (block column in a matrix where the vertex block is inseted).
	 */
	virtual size_t n_Id() const = 0;

	/**
	 *	@brief sets vertex id
	 *	@param[in] n_id is vertex id
	 *		(block column in a matrix where the vertex block is inseted).
	 */
	virtual void Set_Id(size_t n_id) = 0;

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief swaps solution vector with vertex state and minds the manifold space
	 *	@param[in,out] r_v_x is solution vector
	 *	@note This is required by the experimental SPCG solver, no need to implement
	 *		it everywhere (now it is just in SE(2)).
	 */
	virtual void SwapState(Eigen::VectorXd &r_v_x) = 0;

	/**
	 *	@brief saves the vertex state in a vector
	 *	@param[out] r_v_x is the state vector to copy the state to
	 */
	virtual void SaveState(Eigen::VectorXd &r_v_x) const = 0;

	/**
	 *	@brief restores the vertex state from a vector
	 *	@param[out] r_v_x is the state vector to copy the state from
	 */
	virtual void LoadState(const Eigen::VectorXd &r_v_x) = 0;

	/**
	 *	@brief adds delta vector to vertex state and minds the manifold space ("smart" plus)
	 *	@param[in] r_v_delta is delta vector
	 */
	virtual void Operator_Plus(const Eigen::VectorXd &r_v_delta) = 0;

	/**
	 *	@brief subtracts delta vector to vertex state and minds the manifold space ("smart" minus)
	 *	@param[in] r_v_delta is delta vector
	 */
	virtual void Operator_Minus(const Eigen::VectorXd &r_v_delta) = 0; // "smart" minus

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@brief adds an edge that references this vertex
	 *
	 *	@param[in] p_edge is an edge that references this vertex (must be called
	 *		only once with each edge to avoid duplicates)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Add_ReferencingEdge(_TyBaseEdge *p_edge) = 0; // throw(std::bad_alloc)

	/**
	 *	@brief checks if an edge references this vertex
	 *	@param[in] p_edge is pointer to an edge
	 *	@return Returns true if the given edge is in the list
	 *		of referencing edges, otherwise returns false.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual bool b_IsReferencingEdge(const _TyBaseEdge *p_edge) const = 0;

	/**
	 *	@brief gets list of referencing edges
	 *	@return Returns const reference to the list of referencing edges.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual const std::vector<_TyBaseEdge*> &r_ReferencingEdge_List() const = 0;


	/**
	 *	@brief allocates Hessian blocks
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda) = 0;

	/**
	 *	@brief calculates diagonal lambda Hessian blocks
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Calculate_Hessians() = 0;

	/**
	 *	@brief gets a portion of right-hand side vector, associated with this vertex
	 *	@param[out] r_v_eta is the right-hand side vector
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Get_RightHandSide_Vector(Eigen::VectorXd &r_v_eta) const = 0;

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@brief allocates L factor blocks
	 *	@param[out] r_L is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_L.
	 */
	virtual void Alloc_LBlocks(CUberBlockMatrix &r_L) const = 0;

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---
};

class CEdgeInterface { // this is likely going to be unused, remove it later, copy doc somewhere else
public:
	/**
	 *	@brief gets edge dimension
	 *	@return Returns edge dimension.
	 */
	virtual size_t n_Dimension() const = 0;

	/**
	 *	@brief gets number of edge vertices
	 *	@return Returns number of edge vertices (2 in binary edges).
	 */
	virtual size_t n_Vertex_Num() const = 0;

	/**
	 *	@brief gets id of edge vertex
	 *	@param[in] n_vertex is zero-based vertex index
	 *	@return Returns id of the selected edge vertex.
	 */
	virtual size_t n_Vertex_Id(size_t n_vertex) const = 0;

	/**
	 *	@brief gets edge order
	 *	@return Returns edge order (row in a matrix where the edge block is inseted).
	 */
	virtual size_t n_Order() const = 0;

	/**
	 *	@brief sets edge order
	 *	@param[in] n_first_element_index is edge order
	 *		(row in a matrix where the edge block is inseted).
	 */
	virtual void Set_Order(size_t n_first_element_index) = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets edge id
	 *	@return Returns edge id (block row in a matrix where the edge block is inseted).
	 */
	virtual size_t n_Id() const = 0;

	/**
	 *	@brief sets edge order
	 *	@param[in] n_id is edge order
	 *		(block row in a matrix where the edge block is inseted).
	 */
	virtual void Set_Id(size_t n_id) = 0;

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error.
	 *	@note This is supposed to be divided by (m - n), where m is number of edges
	 *		and n is number of degrees of freedom (equals to n_Dimension()).
	 */
	virtual double f_Chi_Squared_Error() const = 0;

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

	/**
	 *	@brief allocates Jacobian blocks
	 *	@param[out] r_A is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Alloc_JacobianBlocks(CUberBlockMatrix &r_A) = 0;

	/**
	 *	@brief calculates Jacobians
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Calculate_Jacobians() = 0;

	/**
	 *	@brief calculates error, multiplied by the R matrix
	 *		(upper diagonal Choleski of inverse sigma matrix)
	 *	@param[out] r_v_dest is the error vector
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Get_R_Error(Eigen::VectorXd &r_v_dest) const = 0;

	/**
	 *	@brief calculates bare error
	 *	@param[out] r_v_dest is the error vector
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	virtual void Get_Error(Eigen::VectorXd &r_v_dest) const = 0;

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- ~A-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

	virtual double f_Max_VertexHessianDiagValue() const = 0; // todo surround with levenberg support ifdef

#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@brief notifies an edge of a conflict (duplicate edge)
	 *	@return Returns true if this is the original owner of the block.
	 */
	virtual bool Notify_HessianBlock_Conflict(double *p_block, CUberBlockMatrix &r_lambda) = 0;

	/**
	 *	@brief allocates Hessian blocks
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda) = 0;

	/**
	 *	@brief calculates off-diagonal lambda Hessian blocks
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual void Calculate_Hessians() = 0;

	/**
	 *	@brief gets edge contributions for lambda and eta blocks
	 *	@param[in] p_which is vertex, associated with queried contributions
	 *	@return Returns a pair of pointers to memory blocks where lambda contribution
	 *		and eta contribution are stored.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	virtual std::pair<const double*, const double*>
		t_Get_LambdaEta_Contribution(const CBaseVertex *p_which) = 0;

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@brief allocates L factor blocks
	 *	@param[out] r_L is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_L.
	 */
	virtual void Alloc_LBlocks(CUberBlockMatrix &r_L) const = 0;

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
	virtual void Calculate_Omega(CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const = 0;
};
#endif // 0

template <class CBaseType>
class CFacadeTraits; // forward declaration

namespace base_iface {

/**
 *	@brief detached vertex facade interface
 *	@tparam CBaseVertexType is edge base type (default CBaseVertex)
 */
template <class CBaseVertexType = CBaseVertex> // does this need to be template? probably yes, might want to have more bases in the future
class CDetachedVertexFacade_Base {
public:
	/**
	 *	@copydoc CConstVertexFacade::v_State
	 *	@param[in] p_this pointer to the base object
	 */
	virtual Eigen::Map<const Eigen::VectorXd> v_StateC(const CBaseVertexType *p_this) const = 0; // todo - use a map to avoid copy? maybe an overkill, but still saves a new()

	/**
	 *	@copydoc CVertexFacade::v_State
	 *	@param[in] p_this pointer to the base object
	 */
	virtual Eigen::Map<Eigen::VectorXd> v_State(CBaseVertexType *p_this) const = 0; // todo - use a map to avoid copy? maybe an overkill, but still saves a new()

	/**
	 *	@copydoc CConstVertexFacade::n_Dimension
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Dimension(const CBaseVertexType *p_this) const = 0;

	/**
	 *	@copydoc CConstVertexFacade::n_Order
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Order(const CBaseVertexType *p_this) const = 0;

	/**
	 *	@copydoc CConstVertexFacade::b_IsConstant
	 *	@param[in] p_this pointer to the base object
	 */
	virtual bool b_IsConstant(const CBaseVertexType *p_this) const = 0;

	/**
	 *	@copydoc CVertexFacade::Set_Order
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Set_Order(CBaseVertexType *p_this, size_t n_first_element_index) const = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CConstVertexFacade::n_Id
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Id(const CBaseVertexType *p_this) const = 0;

	/**
	 *	@copydoc CVertexFacade::Set_Id
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Set_Id(CBaseVertexType *p_this, size_t n_id) const = 0;

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CVertexFacade::SwapState
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void SwapState(CBaseVertexType *p_this, Eigen::VectorXd &r_v_x) const = 0;

	/**
	 *	@copydoc CConstVertexFacade::SaveState
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void SaveState(const CBaseVertexType *p_this, Eigen::VectorXd &r_v_x) const = 0;

	/**
	 *	@copydoc CVertexFacade::LoadState
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void LoadState(CBaseVertexType *p_this, const Eigen::VectorXd &r_v_x) const = 0;

	/**
	 *	@copydoc CVertexFacade::Operator_Plus
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Operator_Plus(CBaseVertexType *p_this, const Eigen::VectorXd &r_v_delta) const = 0;

	/**
	 *	@copydoc CVertexFacade::Operator_Minus
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Operator_Minus(CBaseVertexType *p_this, const Eigen::VectorXd &r_v_delta) const = 0; // "smart" minus

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	typedef typename CVertexInterface::_TyBaseEdge _TyBaseEdge; // todo - get rid of this

	/**
	 *	@copydoc CVertexFacade::Add_ReferencingEdge
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Add_ReferencingEdge(CBaseVertexType *p_this, _TyBaseEdge *p_edge) const = 0; // throw(std::bad_alloc)

	/**
	 *	@copydoc CConstVertexFacade::b_IsReferencingEdge
	 *	@param[in] p_this pointer to the base object
	 */
	virtual bool b_IsReferencingEdge(const CBaseVertexType *p_this, const _TyBaseEdge *p_edge) const = 0;

	/**
	 *	@copydoc CConstVertexFacade::r_ReferencingEdge_List
	 *	@param[in] p_this pointer to the base object
	 */
	virtual const std::vector<_TyBaseEdge*> &r_ReferencingEdge_List(const CBaseVertexType *p_this) const = 0;

	/**
	 *	@copydoc CVertexFacade::Alloc_HessianBlocks
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Alloc_HessianBlocks(CBaseVertexType *p_this, CUberBlockMatrix &r_lambda) const = 0;

	/**
	 *	@copydoc CVertexFacade::Calculate_Hessians
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Calculate_Hessians(CBaseVertexType *p_this) const = 0;

	/**
	 *	@copydoc CConstVertexFacade::Get_RightHandSide_Vector
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Get_RightHandSide_Vector(const CBaseVertexType *p_this, Eigen::VectorXd &r_v_eta) const = 0;

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc CConstVertexFacade::Alloc_LBlocks
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Alloc_LBlocks(const CBaseVertexType *p_this, CUberBlockMatrix &r_L) const = 0;

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---
};

/**
 *	@brief detached vertex facade implementation
 *
 *	@tparam CVertexType is vertex type for which this implementation is specialized
 *	@tparam CBaseVertexType is vertex base type (default CBaseVertex)
 */
template <class CVertexType, class CBaseVertexType = CBaseVertex>
class CDetachedVertexFacade : public CDetachedVertexFacade_Base<CBaseVertexType> {
public:
	/**
	 *	@copydoc CDetachedVertexFacade_Base::v_State
	 */
	virtual Eigen::Map<const Eigen::VectorXd> v_StateC(const CBaseVertexType *p_this) const // todo - use a map to avoid copy? maybe an overkill, but still saves a new()
	{
		return static_cast<const CVertexType*>(p_this)->v_State();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::v_State
	 */
	virtual Eigen::Map<Eigen::VectorXd> v_State(CBaseVertexType *p_this) const // todo - use a map to avoid copy? maybe an overkill, but still saves a new()
	{
		return static_cast<CVertexType*>(p_this)->v_State();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::n_Dimension
	 */
	virtual size_t n_Dimension(const CBaseVertexType *p_this) const
	{
		return static_cast<const CVertexType*>(p_this)->n_Dimension();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::n_Order
	 */
	virtual size_t n_Order(const CBaseVertexType *p_this) const
	{
		return static_cast<const CVertexType*>(p_this)->n_Order();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::b_IsConstant
	 */
	virtual bool b_IsConstant(const CBaseVertexType *p_this) const
	{
		return static_cast<const CVertexType*>(p_this)->b_IsConstant();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Set_Order
	 */
	virtual void Set_Order(CBaseVertexType *p_this, size_t n_first_element_index) const
	{
		static_cast<CVertexType*>(p_this)->Set_Order(n_first_element_index);
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CDetachedVertexFacade_Base::n_Id
	 */
	virtual size_t n_Id(const CBaseVertexType *p_this) const
	{
		return static_cast<const CVertexType*>(p_this)->n_Id();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Set_Id
	 */
	virtual void Set_Id(CBaseVertexType *p_this, size_t n_id) const
	{
		static_cast<const CVertexType*>(p_this)->Set_Id(n_id);
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CDetachedVertexFacade_Base::SwapState
	 */
	virtual void SwapState(CBaseVertexType *p_this, Eigen::VectorXd &r_v_x) const
	{
		static_cast<CVertexType*>(p_this)->SwapState(r_v_x);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::SaveState
	 */
	virtual void SaveState(const CBaseVertexType *p_this, Eigen::VectorXd &r_v_x) const
	{
		static_cast<const CVertexType*>(p_this)->SaveState(r_v_x);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::LoadState
	 */
	virtual void LoadState(CBaseVertexType *p_this, const Eigen::VectorXd &r_v_x) const
	{
		static_cast<CVertexType*>(p_this)->LoadState(r_v_x);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Operator_Plus
	 */
	virtual void Operator_Plus(CBaseVertexType *p_this, const Eigen::VectorXd &r_v_delta) const
	{
		static_cast<CVertexType*>(p_this)->Operator_Plus(r_v_delta);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Operator_Minus
	 */
	virtual void Operator_Minus(CBaseVertexType *p_this, const Eigen::VectorXd &r_v_delta) const
	{
		static_cast<CVertexType*>(p_this)->Operator_Minus(r_v_delta);
	}

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	typedef typename CDetachedVertexFacade_Base<CBaseVertexType>::_TyBaseEdge _TyBaseEdge; // todo - get rid of this

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Add_ReferencingEdge
	 */
	virtual void Add_ReferencingEdge(CBaseVertexType *p_this, _TyBaseEdge *p_edge) const // throw(std::bad_alloc)
	{
		static_cast<CVertexType*>(p_this)->Add_ReferencingEdge(p_edge);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::b_IsReferencingEdge
	 */
	virtual bool b_IsReferencingEdge(const CBaseVertexType *p_this, const _TyBaseEdge *p_edge) const
	{
		return static_cast<const CVertexType*>(p_this)->b_IsReferencingEdge(p_edge);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::r_ReferencingEdge_List
	 */
	virtual const std::vector<_TyBaseEdge*> &r_ReferencingEdge_List(const CBaseVertexType *p_this) const
	{
		return static_cast<const CVertexType*>(p_this)->r_ReferencingEdge_List();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Alloc_HessianBlocks
	 */
	virtual void Alloc_HessianBlocks(CBaseVertexType *p_this, CUberBlockMatrix &r_lambda) const
	{
		static_cast<CVertexType*>(p_this)->Alloc_HessianBlocks(r_lambda);
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Calculate_Hessians
	 */
	virtual void Calculate_Hessians(CBaseVertexType *p_this) const
	{
		static_cast<CVertexType*>(p_this)->Calculate_Hessians();
	}

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Get_RightHandSide_Vector
	 */
	virtual void Get_RightHandSide_Vector(const CBaseVertexType *p_this, Eigen::VectorXd &r_v_eta) const
	{
		static_cast<const CVertexType*>(p_this)->Get_RightHandSide_Vector(r_v_eta);
	}

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc CDetachedVertexFacade_Base::Alloc_LBlocks
	 */
	virtual void Alloc_LBlocks(const CBaseVertexType *p_this, CUberBlockMatrix &r_L) const
	{
		static_cast<const CVertexType*>(p_this)->Alloc_LBlocks(r_L);
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---
};

/**
 *	@brief const vertex facade implementation
 *	@tparam CBaseVertexType is vertex base type (default CBaseVertex)
 */
template <class CBaseVertexType = CBaseVertex>
class CConstVertexFacade { // this could have virtual functions and a base class, but at this point there is only a single base vertex type (the CBaseVertex), so no base seems to be needed and we save one layer of virtual functions
public:
	typedef CBaseVertexType _TyVertex; /**< @brief base vertex type (default CBaseVertex) */
	typedef CDetachedVertexFacade_Base<CBaseVertexType> _TyStaticFacade; /**< @brief facade base type */

protected:
	_TyVertex *m_p_vertex; /**< @brief pointer to the vertex instance */
	const _TyStaticFacade *m_p_facade; /**< @brief pointer to the facade implementation @note This is statically allocated so that CVertexFacade can reside on stack and can be passed by value without causing problems. */

public:
	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_vertex is reference to the vertex instance
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CConstVertexFacade(_TyVertex &r_vertex, const _TyStaticFacade &r_facade)
		:m_p_vertex(&r_vertex), m_p_facade(&r_facade)
	{}

	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_other is reference to another vertex facade
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CConstVertexFacade(const CConstVertexFacade<CBaseVertexType> &r_other, const _TyStaticFacade &r_facade)
		:m_p_vertex(const_cast<_TyVertex*>(r_other.m_p_vertex)), m_p_facade(&r_facade)
	{}

	/**
	 *	@brief copy operator
	 *	@param[in] r_other is reference to another vertex facade
	 *	@return Returns reference to this.
	 */
	CConstVertexFacade &operator =(const CConstVertexFacade<CBaseVertexType> &r_other)
	{
		m_p_vertex = const_cast<_TyVertex*>(r_other.m_p_vertex);
		m_p_facade = r_other.m_p_facade;
		return *this;
	}

	/**
	 *	@brief gets reference to the wrapped object
	 *	@return Returns const reference to the wrapped object.
	 *	@note This only returns reference to a base object. Casting it to some type is not,
	 *		generally, safe. Instead, use CMultiPool::r_At(), which is type-checked and therefore safer.
	 *		Still, this can be used if the base object has enough functionality for a given purpose,
	 *		or for checking whether an object is the one in this facade (by comparing their addresses).
	 */
	const _TyVertex &r_Get() const
	{
		return *m_p_vertex;
	}

	/**
	 *	@brief gets vertex dimension
	 *	@return Returns vertex dimension.
	 */
	inline Eigen::Map<const Eigen::VectorXd> v_State() const
	{
		return m_p_facade->v_StateC(m_p_vertex);
	}

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns vertex state vector.
	 */
	inline Eigen::Map<const Eigen::VectorXd> v_StateC() const // explicit const
	{
		return m_p_facade->v_StateC(m_p_vertex);
	}

	/**
	 *	@brief gets vertex dimension
	 *	@return Returns vertex dimension.
	 */
	size_t n_Dimension() const
	{
		return m_p_facade->n_Dimension(m_p_vertex);
	}

	/**
	 *	@brief gets vertex order
	 *	@return Returns vertex order (column in a matrix where the vertex block is inseted).
	 */
	size_t n_Order() const
	{
		return m_p_facade->n_Order(m_p_vertex);
	}

	/**
	 *	@brief determines whether this vertex is a constant vertex
	 *	@return Returns true if this vertex is a constant vertex, otherwise returns false.
	 */
	bool b_IsConstant() const
	{
		return m_p_facade->b_IsConstant(m_p_vertex);
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets vertex id
	 *	@return Returns vertex order (block column in a matrix where the vertex block is inseted).
	 */
	size_t n_Id() const
	{
		return m_p_facade->n_Id(m_p_vertex);
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief saves the vertex state in a vector
	 *	@param[out] r_v_x is the state vector to copy the state to
	 */
	void SaveState(Eigen::VectorXd &r_v_x) const
	{
		m_p_facade->SaveState(m_p_vertex, r_v_x);
	}

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	typedef CVertexInterface::_TyBaseEdge _TyBaseEdge; // todo - get rid of this

	/**
	 *	@brief checks if an edge references this vertex
	 *	@param[in] p_edge is pointer to an edge
	 *	@return Returns true if the given edge is in the list
	 *		of referencing edges, otherwise returns false.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	bool b_IsReferencingEdge(const _TyBaseEdge *p_edge) const
	{
		return m_p_facade->b_IsReferencingEdge(m_p_vertex, p_edge);
	}

	/**
	 *	@brief gets list of referencing edges
	 *	@return Returns const reference to the list of referencing edges.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	const std::vector<_TyBaseEdge*> &r_ReferencingEdge_List() const
	{
		return m_p_facade->r_ReferencingEdge_List(m_p_vertex);
	}

	/**
	 *	@brief gets a portion of right-hand side vector, associated with this vertex
	 *	@param[out] r_v_eta is the right-hand side vector
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	void Get_RightHandSide_Vector(Eigen::VectorXd &r_v_eta) const
	{
		m_p_facade->Get_RightHandSide_Vector(m_p_vertex, r_v_eta);
	}

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@brief allocates L factor blocks
	 *	@param[out] r_L is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_L.
	 */
	void Alloc_LBlocks(CUberBlockMatrix &r_L) const
	{
		m_p_facade->Alloc_LBlocks(m_p_vertex, r_L);
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---
};

/**
 *	@brief vertex facade implementation
 *	@tparam CBaseVertexType is vertex base type (default CBaseVertex)
 */
template <class CBaseVertexType = CBaseVertex>
class CVertexFacade : public CConstVertexFacade<CBaseVertexType> {
public:
	typedef CBaseVertexType _TyVertex; /**< @brief base vertex type (default CBaseVertex) */
	typedef CDetachedVertexFacade_Base<CBaseVertexType> _TyStaticFacade; /**< @brief facade base type */

public:
	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_vertex is reference to the vertex instance
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CVertexFacade(_TyVertex &r_vertex, const _TyStaticFacade &r_facade)
		:CConstVertexFacade<CBaseVertexType>(r_vertex, r_facade)
	{}

	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_other is reference to another vertex facade
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CVertexFacade(const CVertexFacade<CBaseVertexType> &r_other, const _TyStaticFacade &r_facade) // can't convert const to non-const
		:CConstVertexFacade<CBaseVertexType>(r_other, r_facade)
	{}

	/**
	 *	@brief copy operator
	 *	@param[in] r_other is reference to another vertex facade
	 *	@return Returns reference to this.
	 */
	CVertexFacade &operator =(const CVertexFacade<CBaseVertexType> &r_other) // can't convert const to non-const
	{
		this->m_p_vertex = const_cast<_TyVertex*>(r_other.m_p_vertex);
		this->m_p_facade = r_other.m_p_facade;
		return *this;
	}

	/**
	 *	@brief gets reference to the wrapped object
	 *	@return Returns reference to the wrapped object.
	 *	@note This only returns reference to a base object. Casting it to some type is not,
	 *		generally, safe. Instead, use CMultiPool::r_At(), which is type-checked and therefore safer.
	 *		Still, this can be used if the base object has enough functionality for a given purpose,
	 *		or for checking whether an object is the one in this facade (by comparing their addresses).
	 */
	_TyVertex &r_Get()
	{
		return *this->m_p_vertex;
	}

	/**
	 *	@brief gets vertex state vector
	 *	@return Returns vertex state vector.
	 */
	inline Eigen::Map<Eigen::VectorXd> v_State()
	{
		return this->m_p_facade->v_State(this->m_p_vertex);
	}

	/**
	 *	@brief sets vertex order
	 *	@param[in] n_first_element_index is vertex order
	 *		(column in a matrix where the vertex block is inserted).
	 */
	void Set_Order(size_t n_first_element_index)
	{
		this->m_p_facade->Set_Order(this->m_p_vertex, n_first_element_index);
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief sets vertex id
	 *	@param[in] n_id is vertex id
	 *		(block column in a matrix where the vertex block is inseted).
	 */
	void Set_Id(size_t n_id)
	{
		this->m_p_facade->Set_Id(this->m_p_vertex, n_id);
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief swaps solution vector with vertex state and minds the manifold space
	 *	@param[in,out] r_v_x is solution vector
	 *	@note This is required by the experimental SPCG solver, no need to implement
	 *		it everywhere (now it is just in SE(2)).
	 */
	void SwapState(Eigen::VectorXd &r_v_x)
	{
		this->m_p_facade->SwapState(this->m_p_vertex, r_v_x);
	}

	/**
	 *	@brief restores the vertex state from a vector
	 *	@param[out] r_v_x is the state vector to copy the state from
	 */
	void LoadState(const Eigen::VectorXd &r_v_x)
	{
		this->m_p_facade->LoadState(this->m_p_vertex, r_v_x);
	}

	/**
	 *	@brief adds delta vector to vertex state and minds the manifold space ("smart" plus)
	 *	@param[in] r_v_delta is delta vector
	 */
	void Operator_Plus(const Eigen::VectorXd &r_v_delta)
	{
		this->m_p_facade->Operator_Plus(this->m_p_vertex, r_v_delta);
	}

	/**
	 *	@brief subtracts delta vector to vertex state and minds the manifold space ("smart" minus)
	 *	@param[in] r_v_delta is delta vector
	 */
	void Operator_Minus(const Eigen::VectorXd &r_v_delta)
	{
		this->m_p_facade->Operator_Minus(this->m_p_vertex, r_v_delta);
	}

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---
#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@brief adds an edge that references this vertex
	 *
	 *	@param[in] p_edge is an edge that references this vertex (must be called
	 *		only once with each edge to avoid duplicates)
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	void Add_ReferencingEdge(CConstVertexFacade<CBaseVertexType>::_TyBaseEdge *p_edge) // throw(std::bad_alloc)
	{
		this->m_p_facade->Add_ReferencingEdge(this->m_p_vertex, p_edge);
	}

	/**
	 *	@brief allocates Hessian blocks
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda)
	{
		this->m_p_facade->Alloc_HessianBlocks(this->m_p_vertex, r_lambda);
	}

	/**
	 *	@brief calculates diagonal lambda Hessian blocks
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	void Calculate_Hessians()
	{
		this->m_p_facade->Calculate_Hessians(this->m_p_vertex);
	}

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---
};

/**
 *	@brief detached edge facade interface
 *	@tparam CBaseEdgeType is edge base type (default CBaseEdge)
 */
template <class CBaseEdgeType = CBaseEdge> // does this need to be template? probably yes, might want to have more bases in the future
class CDetachedEdgeFacade_Base {
public:
	/**
	 *	@copydoc CConstEdgeFacade::n_Dimension
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Dimension(const CBaseEdgeType *p_this) const = 0;

	/**
	 *	@copydoc CConstEdgeFacade::n_Vertex_Num
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Vertex_Num(const CBaseEdgeType *p_this) const = 0;

	/**
	 *	@copydoc CConstEdgeFacade::n_Vertex_Id
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Vertex_Id(const CBaseEdgeType *p_this, size_t n_vertex) const = 0;

	/**
	 *	@copydoc CConstEdgeFacade::n_Order
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Order(const CBaseEdgeType *p_this) const = 0;

	/**
	 *	@copydoc CEdgeFacade::Set_Order
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Set_Order(CBaseEdgeType *p_this, size_t n_first_element_index) const = 0;

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CConstEdgeFacade::n_Id
	 *	@param[in] p_this pointer to the base object
	 */
	virtual size_t n_Id(const CBaseEdgeType *p_this) const = 0;

	/**
	 *	@copydoc CEdgeFacade::Set_Id
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Set_Id(CBaseEdgeType *p_this, size_t n_id) const = 0;

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CConstEdgeFacade::f_Chi_Squared_Error
	 *	@param[in] p_this pointer to the base object
	 */
	virtual double f_Chi_Squared_Error(const CBaseEdgeType *p_this) const = 0;

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

	/**
	 *	@copydoc CEdgeFacade::Alloc_JacobianBlocks
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Alloc_JacobianBlocks(CBaseEdgeType *p_this, CUberBlockMatrix &r_A) const = 0;

	/**
	 *	@copydoc CEdgeFacade::Calculate_Jacobians
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Calculate_Jacobians(CBaseEdgeType *p_this) const = 0;

	/**
	 *	@copydoc CConstEdgeFacade::Get_R_Error
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Get_R_Error(const CBaseEdgeType *p_this, Eigen::VectorXd &r_v_dest) const = 0;

	/**
	 *	@copydoc CConstEdgeFacade::Get_Error
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Get_Error(const CBaseEdgeType *p_this, Eigen::VectorXd &r_v_dest) const = 0;

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- ~A-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

	/**
	 *	@copydoc CConstEdgeFacade::f_Max_VertexHessianDiagValue
	 *	@param[in] p_this pointer to the base object
	 */
	virtual double f_Max_VertexHessianDiagValue(const CBaseEdgeType *p_this) const = 0; // todo surround with levenberg support ifdef

#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@copydoc CEdgeFacade::Alloc_HessianBlocks
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Alloc_HessianBlocks(CBaseEdgeType *p_this, CUberBlockMatrix &r_lambda) const = 0;

	/**
	 *	@copydoc CEdgeFacade::Calculate_Hessians
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Calculate_Hessians(CBaseEdgeType *p_this) const = 0;

	/**
	 *	@copydoc CEdgeFacade::Notify_HessianBlock_Conflict
	 *	@param[in] p_this pointer to the base object
	 */
	virtual bool Notify_HessianBlock_Conflict(CBaseEdgeType *p_this,
		double *p_block, CUberBlockMatrix &r_lambda) const = 0;

	/**
	 *	@copydoc CEdgeFacade::t_Get_LambdaEta_Contribution
	 *	@param[in] p_this pointer to the base object
	 */
	virtual std::pair<const double*, const double*>
		t_Get_LambdaEta_Contribution(CBaseEdgeType *p_this, const CBaseVertex *p_which) const = 0;

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc CConstEdgeFacade::Alloc_LBlocks
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Alloc_LBlocks(const CBaseEdgeType *p_this, CUberBlockMatrix &r_L) const = 0;

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---

	/**
	 *	@copydoc CConstEdgeFacade::Calculate_Omega
	 *	@param[in] p_this pointer to the base object
	 */
	virtual void Calculate_Omega(const CBaseEdgeType *p_this, CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const = 0;
};

/**
 *	@brief detached edge facade implementation
 *
 *	@tparam CEdgeType is edge type for which this implementation is specialized
 *	@tparam CBaseEdgeType is edge base type (default CBaseEdge)
 */
template <class CEdgeType, class CBaseEdgeType = CBaseEdge>
class CDetachedEdgeFacade : public CDetachedEdgeFacade_Base<CBaseEdgeType> {
public:
	/**
	 *	@copydoc CDetachedEdgeFacade_Base::n_Dimension
	 */
	virtual size_t n_Dimension(const CBaseEdgeType *p_this) const
	{
		return static_cast<const CEdgeType*>(p_this)->n_Dimension();
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::n_Vertex_Num
	 */
	virtual size_t n_Vertex_Num(const CBaseEdgeType *p_this) const
	{
		return static_cast<const CEdgeType*>(p_this)->n_Vertex_Num();
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::n_Vertex_Id
	 */
	virtual size_t n_Vertex_Id(const CBaseEdgeType *p_this, size_t n_vertex) const
	{
		return static_cast<const CEdgeType*>(p_this)->n_Vertex_Id(n_vertex);
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::n_Order
	 */
	virtual size_t n_Order(const CBaseEdgeType *p_this) const
	{
		return static_cast<const CEdgeType*>(p_this)->n_Order();
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Set_Order
	 */
	virtual void Set_Order(CBaseEdgeType *p_this, size_t n_first_element_index) const
	{
		static_cast<CEdgeType*>(p_this)->Set_Order(n_first_element_index);
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::n_Id
	 */
	virtual size_t n_Id(const CBaseEdgeType *p_this) const
	{
		return static_cast<const CEdgeType*>(p_this)->n_Id();
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Set_Id
	 */
	virtual void Set_Id(CBaseEdgeType *p_this, size_t n_id) const
	{
		static_cast<CEdgeType*>(p_this)->Set_Id(n_id);
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::f_Chi_Squared_Error
	 */
	virtual double f_Chi_Squared_Error(const CBaseEdgeType *p_this) const
	{
		return static_cast<const CEdgeType*>(p_this)->f_Chi_Squared_Error();
	}

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Alloc_JacobianBlocks
	 */
	virtual void Alloc_JacobianBlocks(CBaseEdgeType *p_this, CUberBlockMatrix &r_A) const
	{
		static_cast<CEdgeType*>(p_this)->Alloc_JacobianBlocks(r_A);
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Calculate_Jacobians
	 */
	virtual void Calculate_Jacobians(CBaseEdgeType *p_this) const
	{
		static_cast<CEdgeType*>(p_this)->Calculate_Jacobians();
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Get_R_Error
	 */
	virtual void Get_R_Error(const CBaseEdgeType *p_this, Eigen::VectorXd &r_v_dest) const
	{
		static_cast<const CEdgeType*>(p_this)->Get_R_Error(r_v_dest);
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Get_Error
	 */
	virtual void Get_Error(const CBaseEdgeType *p_this, Eigen::VectorXd &r_v_dest) const
	{
		static_cast<const CEdgeType*>(p_this)->Get_Error(r_v_dest);
	}

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- ~A-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::f_Max_VertexHessianDiagValue
	 */
	virtual double f_Max_VertexHessianDiagValue(const CBaseEdgeType *p_this) const // todo surround with levenberg support ifdef
	{
		return static_cast<const CEdgeType*>(p_this)->f_Max_VertexHessianDiagValue();
	}

#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Alloc_HessianBlocks
	 */
	virtual void Alloc_HessianBlocks(CBaseEdgeType *p_this, CUberBlockMatrix &r_lambda) const
	{
		static_cast<CEdgeType*>(p_this)->Alloc_HessianBlocks(r_lambda);
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Calculate_Hessians
	 */
	virtual void Calculate_Hessians(CBaseEdgeType *p_this) const
	{
		static_cast<CEdgeType*>(p_this)->Calculate_Hessians();
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Notify_HessianBlock_Conflict
	 */
	virtual bool Notify_HessianBlock_Conflict(CBaseEdgeType *p_this, double *p_block, CUberBlockMatrix &r_lambda) const
	{
		return static_cast<CEdgeType*>(p_this)->Notify_HessianBlock_Conflict(p_block, r_lambda);
	}

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::t_Get_LambdaEta_Contribution
	 */
	virtual std::pair<const double*, const double*>
		t_Get_LambdaEta_Contribution(CBaseEdgeType *p_this, const CBaseVertex *p_which) const
	{
		return static_cast<CEdgeType*>(p_this)->t_Get_LambdaEta_Contribution(p_which);
	}

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Alloc_LBlocks
	 */
	virtual void Alloc_LBlocks(const CBaseEdgeType *p_this, CUberBlockMatrix &r_L) const
	{
		static_cast<const CEdgeType*>(p_this)->Alloc_LBlocks(r_L);
	}

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---

	/**
	 *	@copydoc CDetachedEdgeFacade_Base::Calculate_Omega
	 */
	virtual void Calculate_Omega(const CBaseEdgeType *p_this, CUberBlockMatrix &r_omega, size_t n_min_vertex_order) const
	{
		static_cast<const CEdgeType*>(p_this)->Calculate_Omega(r_omega, n_min_vertex_order);
	}
};

/**
 *	@brief const edge facade implementation
 *	@tparam CBaseEdgeType is edge base type (default CBaseEdge)
 */
template <class CBaseEdgeType = CBaseEdge>
class CConstEdgeFacade { // this could have virtual functions and a base class, but at this point there is only a single base vertex type (the CBaseVertex), so no base seems to be needed and we save one layer of virtual functions
public:
	typedef CBaseEdgeType _TyEdge; /**< @brief base edge type (default CBaseEdge) */
	typedef CDetachedEdgeFacade_Base<CBaseEdgeType> _TyStaticFacade; /**< @brief facade base type */

protected:
	_TyEdge *m_p_edge; /**< @brief pointer to the edge instance */
	const _TyStaticFacade *m_p_facade; /**< @brief pointer to the facade implementation @note This is statically allocated so that CEdgeFacade can reside on stack and can be passed by value without causing problems. */

public:
	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_edge is reference to the edge instance
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CConstEdgeFacade(_TyEdge &r_edge, const _TyStaticFacade &r_facade)
		:m_p_edge(&r_edge), m_p_facade(&r_facade)
	{}

	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_other is reference to another edge facade
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CConstEdgeFacade(const CConstEdgeFacade<CBaseEdgeType> &r_other, const _TyStaticFacade &r_facade)
		:m_p_edge(const_cast<_TyEdge*>(r_other.m_p_edge)), m_p_facade(&r_facade)
	{}

	/**
	 *	@brief copy operator
	 *	@param[in] r_other is reference to another edge facade
	 *	@return Returns reference to this.
	 */
	CConstEdgeFacade &operator =(const CConstEdgeFacade<CBaseEdgeType> &r_other)
	{
		m_p_edge = const_cast<_TyEdge*>(r_other.m_p_edge);
		m_p_facade = r_other.m_p_facade;
		return *this;
	}

	/**
	 *	@brief gets reference to the wrapped object
	 *	@return Returns reference to the wrapped object.
	 *	@note This only returns reference to a base object. Casting it to some type is not,
	 *		generally, safe. Instead, use CMultiPool::r_At(), which is type-checked and therefore safer.
	 *		Still, this can be used if the base object has enough functionality for a given purpose,
	 *		or for checking whether an object is the one in this facade (by comparing their addresses).
	 */
	const _TyEdge &r_Get() const // returns const reference to the wrapped object
	{
		return *m_p_edge;
	}

	/**
	 *	@brief gets edge dimension
	 *	@return Returns edge dimension.
	 */
	inline size_t n_Dimension() const
	{
		return m_p_facade->n_Dimension(m_p_edge);
	}

	/**
	 *	@brief gets number of edge vertices
	 *	@return Returns number of edge vertices (2 in binary edges).
	 */
	inline size_t n_Vertex_Num() const
	{
		return m_p_facade->n_Vertex_Num(m_p_edge);
	}

	/**
	 *	@brief gets id of edge vertex
	 *	@param[in] n_vertex is zero-based vertex index
	 *	@return Returns id of the selected edge vertex.
	 */
	inline size_t n_Vertex_Id(size_t n_vertex) const
	{
		return m_p_facade->n_Vertex_Id(m_p_edge, n_vertex);
	}

	/**
	 *	@brief gets edge order
	 *	@return Returns edge order (row in a matrix where the edge block is inseted).
	 */
	inline size_t n_Order() const
	{
		return m_p_facade->n_Order(m_p_edge);
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief gets edge id
	 *	@return Returns edge id (block row in a matrix where the edge block is inseted).
	 */
	inline size_t n_Id() const
	{
		return m_p_facade->n_Id(m_p_edge);
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error.
	 *	@note This is supposed to be divided by (m - n), where m is number of edges
	 *		and n is number of degrees of freedom (equals to n_Dimension()).
	 */
	inline double f_Chi_Squared_Error() const
	{
		return m_p_facade->f_Chi_Squared_Error(m_p_edge);
	}

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

	/**
	 *	@brief calculates error, multiplied by the R matrix
	 *		(upper diagonal Choleski of inverse sigma matrix)
	 *	@param[out] r_v_dest is the error vector
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	inline void Get_R_Error(Eigen::VectorXd &r_v_dest) const
	{
		m_p_facade->Get_R_Error(m_p_edge, r_v_dest);
	}

	/**
	 *	@brief calculates bare error
	 *	@param[out] r_v_dest is the error vector
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	inline void Get_Error(Eigen::VectorXd &r_v_dest) const
	{
		m_p_facade->Get_Error(m_p_edge, r_v_dest);
	}

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- ~A-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

	/**
	 *	@brief gets the maximum Hessian diagonal value (a damping heuristic for LM)
	 *	@return Returns the maximum Hessian diagonal value.
	 */
	inline double f_Max_VertexHessianDiagValue() const
	{
		return m_p_facade->f_Max_VertexHessianDiagValue(m_p_edge);
	}

#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

	/**
	 *	@brief allocates L factor blocks
	 *	@param[out] r_L is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_L.
	 */
	inline void Alloc_LBlocks(CUberBlockMatrix &r_L) const
	{
		m_p_facade->Alloc_LBlocks(m_p_edge, r_L);
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
		m_p_facade->Calculate_Omega(m_p_edge, r_omega, n_min_vertex_order);
	}
};


/**
 *	@brief edge facade implementation
 *	@tparam CBaseEdgeType is edge base type (default CBaseEdge)
 */
template <class CBaseEdgeType = CBaseEdge>
class CEdgeFacade : public CConstEdgeFacade<CBaseEdgeType> { // this could have virtual functions and a base class, but at this point there is only a single base vertex type (the CBaseVertex), so no base seems to be needed and we save one layer of virtual functions
public:
	typedef CBaseEdgeType _TyEdge; /**< @brief base edge type (default CBaseEdge) */
	typedef CDetachedEdgeFacade_Base<CBaseEdgeType> _TyStaticFacade; /**< @brief facade base type */

public:
	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_edge is reference to the edge instance
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CEdgeFacade(_TyEdge &r_edge, const _TyStaticFacade &r_facade)
		:CConstEdgeFacade<CBaseEdgeType>(r_edge, r_facade)
	{}

	/**
	 *	@brief constructor
	 *
	 *	@param[in] r_other is reference to another edge facade
	 *	@param[in] r_facade is reference to the facade instance
	 */
	CEdgeFacade(const CEdgeFacade<CBaseEdgeType> &r_other, const _TyStaticFacade &r_facade)
		:CConstEdgeFacade<CBaseEdgeType>(r_other, r_facade)
	{}

	/**
	 *	@brief copy operator
	 *	@param[in] r_other is reference to another edge facade
	 *	@return Returns reference to this.
	 */
	CEdgeFacade &operator =(const CEdgeFacade<CBaseEdgeType> &r_other)
	{
		this->m_p_edge = const_cast<_TyEdge*>(r_other.m_p_edge);
		this->m_p_facade = r_other.m_p_facade;
		return *this;
	}

	/**
	 *	@brief gets reference to the wrapped object
	 *	@return Returns reference to the wrapped object.
	 *	@note This only returns reference to a base object. Casting it to some type is not,
	 *		generally, safe. Instead, use CMultiPool::r_At(), which is type-checked and therefore safer.
	 *		Still, this can be used if the base object has enough functionality for a given purpose,
	 *		or for checking whether an object is the one in this facade (by comparing their addresses).
	 */
	_TyEdge &r_Get() // returns reference to the wrapped object
	{
		return *this->m_p_edge;
	}

	/**
	 *	@brief sets edge order
	 *	@param[in] n_first_element_index is edge order
	 *		(row in a matrix where the edge block is inseted).
	 */
	inline void Set_Order(size_t n_first_element_index)
	{
		this->m_p_facade->Set_Order(this->m_p_edge, n_first_element_index);
	}

#ifdef __BASE_TYPES_USE_ID_ADDRESSING

	/**
	 *	@brief sets edge order
	 *	@param[in] n_id is edge order
	 *		(block row in a matrix where the edge block is inseted).
	 */
	inline void Set_Id(size_t n_id)
	{
		this->m_p_facade->Set_Id(this->m_p_edge, n_id);
	}

#endif // __BASE_TYPES_USE_ID_ADDRESSING

#ifdef __SE_TYPES_SUPPORT_A_SOLVERS // --- A-SLAM specific functions ---

	/**
	 *	@brief allocates Jacobian blocks
	 *	@param[out] r_A is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	inline void Alloc_JacobianBlocks(CUberBlockMatrix &r_A)
	{
		this->m_p_facade->Alloc_JacobianBlocks(this->m_p_edge, r_A);
	}

	/**
	 *	@brief calculates Jacobians
	 *	@note This function is required for CNonlinearSolver_A.
	 */
	inline void Calculate_Jacobians()
	{
		this->m_p_facade->Calculate_Jacobians(this->m_p_edge);
	}

#endif // __SE_TYPES_SUPPORT_A_SOLVERS // --- ~A-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifndef __LAMBDA_USE_V2_REDUCTION_PLAN

	/**
	 *	@brief allocates Hessian blocks
	 *	@param[out] r_lambda is the target matrix where the blocks are stored
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	inline void Alloc_HessianBlocks(CUberBlockMatrix &r_lambda)
	{
		this->m_p_facade->Alloc_HessianBlocks(this->m_p_edge, r_lambda);
	}

	/**
	 *	@brief calculates off-diagonal lambda Hessian blocks
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	inline void Calculate_Hessians()
	{
		this->m_p_facade->Calculate_Hessians(this->m_p_edge);
	}

	/**
	 *	@brief notifies an edge of a conflict (duplicate edge)
	 *	@return Returns true if this is the original owner of the block.
	 */
	inline bool Notify_HessianBlock_Conflict(double *p_block, CUberBlockMatrix &r_lambda)
	{
		return this->m_p_facade->Notify_HessianBlock_Conflict(this->m_p_edge, p_block, r_lambda);
	}

	/**
	 *	@brief gets edge contributions for lambda and eta blocks
	 *	@param[in] p_which is vertex, associated with queried contributions
	 *	@return Returns a pair of pointers to memory blocks where lambda contribution
	 *		and eta contribution are stored.
	 *	@note This function is required for CNonlinearSolver_Lambda.
	 */
	inline std::pair<const double*, const double*>
		t_Get_LambdaEta_Contribution(const CBaseVertex *p_which)
	{
		return this->m_p_facade->t_Get_LambdaEta_Contribution(this->m_p_edge, p_which);
	}

#endif // !__LAMBDA_USE_V2_REDUCTION_PLAN
#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS // --- ~lambda-SLAM specific functions ---

#ifdef __SE_TYPES_SUPPORT_L_SOLVERS // --- L-SLAM specific functions ---

#endif // __SE_TYPES_SUPPORT_L_SOLVERS // --- ~L-SLAM specific functions ---S
};

/**
 *	@brief makes sure that the facades are allocated statically
 *
 *	(maybe a bad idea, maybe the facades should be inside the pool object, which would also be easy to make)
 *
 *	@tparam CFacadeType is facade type
 */
template <class CFacadeType>
class CFacadeAgglomerator {
public:
	typedef CFacadeType _TyFacade; /**< @brief facade type */

public:
	/**
	 *	@brief gets facade instance
	 *	@return Returns facade instance.
	 */
	static inline _TyFacade &r_Get()
	{
		static _TyFacade f;
		return f;
	}
};

/**
 *	@brief facade table generator
 *
 *	@tparam CCommonBase is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam CDetachedFacadeBase is detached facade base type (e.g.
 *		CDetachedVertexFacade_Base or CDetachedEdgeFacade_Base, specialized for CCommonBase)
 *	@tparam CDetachedFacade is detached facade implementation template (e.g.
 *		CDetachedVertexFacade or CDetachedEdgeFacade)
 */
template <class CCommonBase, class CTypeList, class CDetachedFacadeBase,
	template <class, class> class CDetachedFacade>
class CFacadeTable_Gen {
public:
	typedef typename CTypeList::_TyHead _Ty; /**< @brief facade specialization type (a specific vertex or edge type) */
	typedef CDetachedFacade<_Ty, CCommonBase> _TyFacadeImpl; /**< @brief facade implementation */

public:
	/**
	 *	@brief builds the facade table
	 *	@param[out] p_facade_list is facade table, each item points to a facade implementation
	 *		corresponding to the CTypeList item at the same position; must be allocated by the
	 *		caller to hold at least CTypelistLength<CTypeList>::n_result pointers
	 */
    static inline void BuildTable(CDetachedFacadeBase **p_facade_list)
    {
		*p_facade_list = &CFacadeAgglomerator<_TyFacadeImpl>::r_Get();
        CFacadeTable_Gen<CCommonBase, typename CTypeList::_TyTail,
			CDetachedFacadeBase, CDetachedFacade>::BuildTable(p_facade_list + 1);
    }
};

/**
 *	@brief facade table generator (specialization for an empty list or the end of the list)
 *
 *	@tparam CCommonBase is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CDetachedFacadeBase is detached facade base type (e.g.
 *		CDetachedVertexFacade_Base or CDetachedEdgeFacade_Base, specialized for CCommonBase)
 *	@tparam CDetachedFacade is detached facade implementation template (e.g.
 *		CDetachedVertexFacade or CDetachedEdgeFacade)
 */
template <class CCommonBase, class CDetachedFacadeBase, template <class, class> class CDetachedFacade>
class CFacadeTable_Gen<CCommonBase, CTypelistEnd, CDetachedFacadeBase, CDetachedFacade> {
public:
	/**
	 *	@brief builds the facade table
	 *	@param[out] p_facade_list is facade table (unused)
	 */
    static inline void BuildTable(CDetachedFacadeBase **UNUSED(p_facade_list))
	{}
};

/**
 *	@brief facade table
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam CDetachedFacadeBase is detached facade base type (e.g.
 *		CDetachedVertexFacade_Base or CDetachedEdgeFacade_Base, specialized for CCommonBase)
 *	@tparam CDetachedFacade is detached facade implementation template (e.g.
 *		CDetachedVertexFacade or CDetachedEdgeFacade)
 */
template <class CBaseType, class CTypeList, class CDetachedFacadeBase,
	template <class, class> class CDetachedFacade>
class CFacadeTable {
protected:
	/**
	 *	@brief parameters, stored as enum
	 */
	enum {
		type_Count = CTypelistLength<CTypeList>::n_result /**< @brief number of types this facade is specialized for */
	};

	CDetachedFacadeBase *m_p_facade_list[type_Count]; /**< @brief list of facade instance pointers (allocated on stack, not to be deleted) */

public:
	/**
	 *	@brief default constructor; fills the table with pointers
	 *	@note This does not allocate anything, no std::bad_alloc here.
	 */
	inline CFacadeTable()
	{
		CFacadeTable_Gen<CBaseType, CTypeList, CDetachedFacadeBase,
			CDetachedFacade>::BuildTable(m_p_facade_list);
	}

	/**
	 *	@brief facade accessor; gets facade for a given type
	 *	@param[in] n_type_id is zero-based type index (in CTypeList)
	 *	@return Returns const reference to the facade for the given type.
	 */
	inline const CDetachedFacadeBase &operator [](size_t n_type_id) const
	{
		_ASSERTE(n_type_id < type_Count);
		return *m_p_facade_list[n_type_id];
	}
};

/**
 *	@brief facade instantiation helper
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam CDetachedFacadeBase is detached facade base type (e.g.
 *		CDetachedVertexFacade_Base or CDetachedEdgeFacade_Base, specialized for CCommonBase)
 *	@tparam CDetachedFacade is detached facade implementation template (e.g.
 *		CDetachedVertexFacade or CDetachedEdgeFacade)
 *
 *	@note This could have been a part of CFacadeTable, but it selects the storage
 *		for the facade table (CFacadeAgglomerator in this case). Could have another
 *		implementation which stores it somewhere else.
 */
template <class CBaseType, class CTypeList, class CDetachedFacadeBase,
	template <class, class> class CDetachedFacade>
class CFacadeInstantiationHelper {
public:
	typedef typename CFacadeTraits<CBaseType>::_TyReference _TyReference; /**< @copydoc CFacadeTraits::_TyReference */
	typedef typename CFacadeTraits<CBaseType>::_TyConstReference _TyConstReference; /**< @copydoc CFacadeTraits::_TyConstReference */
	typedef CFacadeTable<CBaseType, CTypeList, CDetachedFacadeBase, CDetachedFacade> _TyFacadeTableImpl; /**< @brief facade table specialization */

public:
	/**
	 *	@brief makes a facade around a specified object
	 *
	 *	@param[in] r_object is reference to the object to be wrapped (passed as reference to the base type)
	 *	@param[in] n_type_id is type id of the object (zero-based index in CTypeList)
	 *
	 *	@return Returns instance of a facade around the specified object.
	 */
	static inline _TyReference MakeRef(CBaseType &r_object, size_t n_type_id)
	{
		const CDetachedFacadeBase &r_facade = CFacadeAgglomerator<_TyFacadeTableImpl>::r_Get()[n_type_id];
		return _TyReference(r_object, r_facade);
	}

	/**
	 *	@brief makes a const facade around a specified object
	 *
	 *	@param[in] r_object is const reference to the object to be wrapped (passed as const reference to the base type)
	 *	@param[in] n_type_id is type id of the object (zero-based index in CTypeList)
	 *
	 *	@return Returns instance of a const facade around the specified object.
	 */
	static inline _TyConstReference MakeRef(const CBaseType &r_object, size_t n_type_id)
	{
		const CDetachedFacadeBase &r_facade = CFacadeAgglomerator<_TyFacadeTableImpl>::r_Get()[n_type_id];
		return _TyReference(r_object, r_facade);
	}
};

/**
 *	@brief statically linked thunk table
 *
 *	This thunk table is built around decision trees which perform
 *	a binary search to jump into the specialized thunk for a given type.
 *	While this might be slightly slower, it might be easier to predict
 *	branching and ultimately it leads to fewer pipeline stalls than
 *	a function call via function pointer (or a virtual function call).
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam COperation is function object type
 */
template <class CBaseType, class CTypeList, class COperation>
class CStaticallyLinkedThunkTable {
public:
	/**
	 *	@brief thunk table lookup implementation
	 */
	class CTableLookup {
	protected:
		/**
		 *	@brief callback object; remembers the instance of the vertex / edge object and the function onject
		 */
		class CCallback {
		protected:
			CBaseType *m_p_item; /**< @brief pointer to the vertex / edge object */
			COperation m_op; /**< @brief function onject */

		public:
			/**
			 *	@brief default constructor
			 *
			 *	@param[in] p_item is pointer to the vertex / edge object
			 *	@param[in] op is function onject
			 */
			inline CCallback(CBaseType *p_item, COperation op)
				:m_p_item(p_item), m_op(op)
			{}

			/**
			 *	@brief function operator; calls the function object on the resolved type
			 *	@tparam CSelectedType is type of the vertex / edge type, selected based on type id
			 */
			template <class CSelectedType>
			inline void operator ()()
			{
				m_op(*static_cast<CSelectedType*>(m_p_item));
				// call the function operator on the selected type
			}
		};

	protected:
		size_t m_n_type_id; /**< @brief type id for this call */

	public:
		/**
		 *	@brief default constructor; only remembers the type id, the lookup is deferred
		 *	@param[in] n_type_id is type id (zero-based index in CTypeList)
		 */
		inline CTableLookup(size_t n_type_id)
			:m_n_type_id(n_type_id)
		{}

		/**
		 *	@brief function operator; performs the type lookup and the call
		 *
		 *	@param[in] p_item is pointer to the vertex / edge object
		 *	@param[in] op is function onject
		 */
		inline void operator ()(CBaseType *p_item, COperation op) const
		{
			CTypelistItemSelect<CTypeList, CCallback>::Select(m_n_type_id, CCallback(p_item, op));
		}
	};

public:
	/**
	 *	@brief thunk table lookup
	 *	@param[in] n_type_id is type id of the object (zero-based index in CTypeList)
	 *	@return Returns thunk function object.
	 *	@note This performs type lookup via decision tree on each invokation of the function object.
	 */
	inline CTableLookup operator [](size_t n_type_id) const
	{
		return CTableLookup(n_type_id);
	}
};

/**
 *	@brief thunk table generator
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam COperation is function object type
 *
 *	@note This is only needed for CDynamicallyLinkedThunkTable.
 */
template <class CBaseType, class CTypeList, class COperation>
class CThunkTable_Gen {
public:
	/**
	 *	@brief thunk function for a given type and a function object type
	 *
	 *	Calls function object on the given object.
	 *
	 *	@param[in] p is pointer to the object (of type CBaseType)
	 *	@param[in] op is function object instance
	 */
    static void Thunk(CBaseType *p, COperation op)
    {
		typedef typename CTypeList::_TyHead _Ty;
        op(*static_cast<_Ty*>(p));
    }

	/**
	 *	@brief builds the table
	 *	@param[out] p_function_list is list of thunk function pointers (must be allocated
	 *		by the caller to hold at least CTypelistLength<CTypeList>::n_result pointers)
	 */
    static inline void BuildTable(void (**p_function_list)(CBaseType*, COperation))
    {
        *p_function_list = &Thunk;
        CThunkTable_Gen<CBaseType, typename CTypeList::_TyTail,
			COperation>::BuildTable(p_function_list + 1);
    }
};

/**
 *	@brief thunk table generator (specialization for an empty list or for the end of the list)
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam COperation is function object type
 *
 *	@note This is only needed for CDynamicallyLinkedThunkTable.
 */
template <class CBaseType, class COperation>
class CThunkTable_Gen<CBaseType, CTypelistEnd, COperation> {
public:
	typedef void (**_TyFunctionList)(CBaseType*, COperation); /**< @brief thunk function pointer list type */
	// otherwise g++ has problems with decorating it as UNUSED

	/**
	 *	@brief builds the table
	 *	@param[out] p_function_list is list of thunk function pointers (unused)
	 */
    static inline void BuildTable(_TyFunctionList UNUSED(p_function_list))
    {}
};

/**
 *	@brief dynamically linked thunk table
 *
 *	This thunk table is built around a simple function pointer table.
 *	The advantage is simplicity, the disadvantage is the lookup and
 *	a function call dependent on value of a pointer in memory, which
 *	may disrupt pipelining.
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam COperation is function object type
 */
template <class CBaseType, class CTypeList, class COperation>
class CDynamicallyLinkedThunkTable {
public:
	/**
	 *	@brief parameters, stored as enum
	 */
	enum {
		type_Count = CTypelistLength<CTypeList>::n_result /**< @brief number of types this facade is specialized for */
	};

	typedef void (*_TyThunkPointer)(CBaseType*, COperation); /**< @brief thunk function pointer */

protected:
	_TyThunkPointer m_p_function_list[type_Count]; /**< @brief thunk pointer table */

public:
	/**
	 *	@brief default constructor; fills the thunk table with function pointers
	 */
	inline CDynamicallyLinkedThunkTable()
	{
		CThunkTable_Gen<CBaseType, CTypeList, COperation>::BuildTable(m_p_function_list);
		// fill the list of functions using the typelist recursion
		// (small toll for a for-each loop, a few types per a lot of elements)
	}

	/**
	 *	@brief thunk table lookup
	 *	@param[in] n_type_id is type id of the object (zero-based index in CTypeList)
	 *	@return Returns thunk function pointer.
	 */
	inline _TyThunkPointer operator [](size_t n_type_id) const
	{
		_ASSERTE(n_type_id < type_Count);
		return m_p_function_list[n_type_id];
	}
};

/**
 *	@brief default thunk table
 *
 *	@tparam CBaseType is common base class (e.g. CBaseVertex or CBaseEdge)
 *	@tparam CTypeList is list of types the facades are to be specialized for
 *	@tparam COperation is function object type
 *
 *	@note The specific implementation of the thunk table depends on the
 *		__FLAT_SYSTEM_STATIC_THUNK_TABLE macro, and is either
 *		CDynamicallyLinkedThunkTable or CStaticallyLinkedThunkTable.
 */
#ifndef __FLAT_SYSTEM_STATIC_THUNK_TABLE // t_odo - make a macro to switch that on/off, add macros to the flag printout list
template <class CBaseType, class CTypeList, class COperation>
class CThunkTable : public CDynamicallyLinkedThunkTable<CBaseType, CTypeList, COperation> {};
#else // !__FLAT_SYSTEM_STATIC_THUNK_TABLE
template <class CBaseType, class CTypeList, class COperation>
class CThunkTable : public CStaticallyLinkedThunkTable<CBaseType, CTypeList, COperation> {};
#endif // !__FLAT_SYSTEM_STATIC_THUNK_TABLE

// note that CSolverOps_Base moved from here to NonlinearSolver_Base.h to nonlinear_detail namespace

/**
 *	@brief converts an object wrapped with a facade to reference to the base type
 *	@tparam CVertexType is base vertex type
 *	@param[in] facade is vertex facade instance
 *	@return Returns reference to the base pointed to by the given facade.
 */
template <class CVertexType>
CVertexType &r_GetBase(CVertexFacade<CVertexType> facade)
{
	return facade.r_Get();
}

/**
 *	@brief converts an object wrapped with a facade to reference to the base type
 *	@tparam CVertexType is base vertex type
 *	@param[in] facade is vertex facade instance
 *	@return Returns const reference to the base pointed to by the given facade.
 */
template <class CVertexType>
const CVertexType &r_GetBase(CConstVertexFacade<CVertexType> facade)
{
	return facade.r_Get();
}

/**
 *	@brief converts an object wrapped with a facade to reference to the base type
 *	@tparam CEdgeType is base edge type
 *	@param[in] facade is edge facade instance
 *	@return Returns reference to the base pointed to by the given facade.
 */
template <class CEdgeType>
CEdgeType &r_GetBase(CEdgeFacade<CEdgeType> facade)
{
	return facade.r_Get();
}

/**
 *	@brief converts an object wrapped with a facade to reference to the base type
 *	@tparam CEdgeType is base edge type
 *	@param[in] facade is edge facade instance
 *	@return Returns const reference to the base pointed to by the given facade.
 */
template <class CEdgeType>
const CEdgeType &r_GetBase(CConstEdgeFacade<CEdgeType> facade)
{
	return facade.r_Get();
}

/**
 *	@brief converts an object wrapped with a facade to reference to the base type
 *	@tparam CVertexOrEdgeType is base vertex or edge type
 *	@param[in] r_base is reference to the base object
 *	@return Returns reference to the base object.
 */
template <class CVertexOrEdgeType>
CVertexOrEdgeType &r_GetBase(CVertexOrEdgeType &r_base)
{
	return r_base;
}

/**
 *	@brief converts an object wrapped with a facade to reference to the base type
 *	@tparam CVertexOrEdgeType is base vertex or edge type
 *	@param[in] r_base is const reference to the base object
 *	@return Returns const reference to the base object.
 */
template <class CVertexOrEdgeType>
const CVertexOrEdgeType &r_GetBase(const CVertexOrEdgeType &r_base)
{
	return r_base;
}

} // ~base_iface

namespace multipool {

template <const bool b_expression>
class CStaticAssert; // forward declaration

} // ~multipool

/**
 *	@brief facade traits template ("specialization" for types that do not require facades for interaction)
 *	@tparam CBaseType is base object type
 *	@note This does not use type list for deciding the specialization, as for each base type
 *		there needs to be facade interface, detached facade and facade implementation so adding
 *		a new base type is not trivial (unless it has exactly the same interface as already
 *		supported base type).
 */
template <class CBaseType>
class CFacadeTraits { // this assumes that the CBaseType is either a fully specialized type or a type with virtual heterogenecy that does not want virtual evasion
public:
	typedef CBaseType &_TyReference; /**< @brief reference type */
	typedef const CBaseType &_TyConstReference; /**< @brief const reference type */

	/**
	 *	@brief facade instantiation helper
	 *	@tparam CTypeList is list of types the facades are to be specialized for (assumed to contain just CBaseType)
	 */
	template <class CTypeList>
	struct H {
		typedef typename multipool::CStaticAssert<CEqualType<CTypeList,
			MakeTypelist1(CBaseType)>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; /**< @brief static assert */
		// there must be just the CBaseType in CTypeList (if this triggers,
		// perhaps a CFacadeTraits specialization is missing for a new base type)

		/**
		 *	@brief gets a reference to an object
		 *
		 *	@param[in] r_object is reference to the object to be wrapped (passed as reference
		 *		to the base type, which is also the final type in this case)
		 *	@param[in] n_type_id is type id of the object (zero-based index in CTypeList, must be zero, unused)
		 *
		 *	@return Returns reference to the specified object.
		 */
		static inline _TyReference MakeRef(CBaseType &r_object, size_t UNUSED(n_type_id))
		{
			_ASSERTE(n_type_id == 0); // this should be used only if there is a single type (same as CBaseType) in the pool
			return r_object;
		}

		/**
		 *	@brief gets a const reference to an object
		 *
		 *	@param[in] r_object is const reference to the object to be wrapped (passed as reference
		 *		to the base type, which is also the final type in this case)
		 *	@param[in] n_type_id is type id of the object (zero-based index in CTypeList, must be zero, unused)
		 *
		 *	@return Returns const reference to the specified object.
		 */
		static inline _TyConstReference MakeRef(const CBaseType &r_object, size_t UNUSED(n_type_id))
		{
			_ASSERTE(n_type_id == 0); // this should be used only if there is a single type (same as CBaseType) in the pool
			return r_object;
		}
	};
};

/*template <const int n_vertex_num = 2>
class CBaseEdge_; // forward declaration
// not a real base - this is already implementing something

template <const int n_vertex_num>
class CFacadeTraits<CBaseEdge_<n_vertex_num> > {
public:
	typedef base_iface::CEdgeFacade<CBaseEdge_<n_vertex_num> > _TyReference;
	typedef const base_iface::CEdgeFacade<CBaseEdge_<n_vertex_num> > _TyConstReference; // easily type casted to simple facade, though, but at least prevents "const_pool[index].nonconstMethod()"

	template <class CTypeList>
	struct H : public base_iface::CFacadeInstantiationHelper<CBaseEdge_<n_vertex_num>, CTypeList,
		base_iface::CDetachedEdgeFacade_Base<CBaseEdge_<n_vertex_num> >, base_iface::CDetachedEdgeFacade> {};
};*/

/**
 *	@brief facade traits template (specialization for CBaseEdge)
 */
template <>
class CFacadeTraits<CBaseEdge> {
public:
	typedef base_iface::CEdgeFacade<CBaseEdge> _TyReference; /**< @brief facade type */
	typedef base_iface::CConstEdgeFacade<CBaseEdge> _TyConstReference; /**< @brief const facade type */

	/**
	 *	@brief facade instantiation helper
	 *	@tparam CTypeList is list of types the facades are to be specialized for (assumed to contain just CBaseType)
	 */
	template <class CTypeList>
	struct H : public base_iface::CFacadeInstantiationHelper<CBaseEdge, CTypeList,
		base_iface::CDetachedEdgeFacade_Base<CBaseEdge>, base_iface::CDetachedEdgeFacade> {};
};

/**
 *	@brief facade traits template (specialization for CBaseVertex)
 */
template <>
class CFacadeTraits<CBaseVertex> {
public:
	typedef base_iface::CVertexFacade<CBaseVertex> _TyReference; /**< @brief facade type */
	typedef base_iface::CConstVertexFacade<CBaseVertex> _TyConstReference; /**< @brief const facade type */

	/**
	 *	@brief facade instantiation helper
	 *	@tparam CTypeList is list of types the facades are to be specialized for (assumed to contain just CBaseType)
	 */
	template <class CTypeList>
	struct H : public base_iface::CFacadeInstantiationHelper<CBaseVertex, CTypeList,
		base_iface::CDetachedVertexFacade_Base<CBaseVertex>, base_iface::CDetachedVertexFacade> {};
};

/** @} */ // end of group

#endif // !__BASE_SE_TYPES_INTERFACE_INCLUDED
