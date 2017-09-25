/*
								+----------------------------------+
								|                                  |
								| ***  Pool-based flat system  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2012  |
								|                                  |
								|           FlatSystem.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __FLAT_SYSTEM_TEMPLATE_INCLUDED
#define __FLAT_SYSTEM_TEMPLATE_INCLUDED

/**
 *	@file include/slam/FlatSystem.h
 *	@brief pool-based flat system template
 *	@author -tHE SWINe-
 *	@date 2012-09-03
 *
 *	This is the new take on implementation of reusable system, this time with templates.
 *	This is very simple to use, one merely needs to list all the types handled by the
 *	system in advance, specialize the system template, and then almost all of the calls
 *	to edge and vertex member functions can be statically linked (not virtual). It is
 *	also very simple to modify SE(2) solver to SE(3) or anything else, really.
 *
 *	t_odo - see if we can implement template token list and have:
 *		* vertex_pose_token, vertex_landmark_token,
 *		* edge_pose_pose_token, edge_landmark_landmark_token
 *	and to create lists of pools from that
 *	and to create vertex / edge addition functions featuring those tokens
 *	then there is just a vector of edge pointers and that shields everything out of that
 *		* yes, this was indeed implemented
 *
 *	t_odo - figure out what else the edge needs to implement and add it, implement ASLAM solver
 *		and test it with the new structure
 *	t_odo - describe the "virtual call evasion" technique used here
 *	t_odo - figure out what operations can be possibly offloaded to solver to simplify the edge
 *			(and vertex) classes // created base edge and vertex implementation templates
 *	t_odo - handle the unary factor bussiness nicely
 *	t_odo - abstract the linear solvers, test cholmod performance
 *
 *	@date 2012-09-12
 *
 *	Fixed linux build issues, namely missing template argument in CMultiPool specialization
 *	for a single type, added namespace for types contained in CFlatSystem, and some other minor bugs.
 *
 *	@date 2013-05-23
 *
 *	Added minimal parallel threshold in For_Each_Parallel(), maybe that was not originally intended
 *	(the caller was supposed to decide whether they want parallel), but it ended up used in a way
 *	that requires this.
 *
 *	@date 2013-11-01
 *
 *	Added support for fixed vertices (they must have b_IsFixed() function, and the vertices must
 *	be determined on initialization, changing to fixed / not fixed later on is not supported at the
 *	moment).
 *
 *	@date 2013-12-05
 *
 *	Added __FLAT_SYSTEM_USE_SIZE2D_FOR_BLOCK_SIZE which speeds compilation up, and improves
 *	Intellisense support by making types easier to resolve (Intellisense in VC 2008 cannot
 *	resolve Eigen matrices).
 *
 *	@date 2015-10-06
 *
 *	Added __BASE_TYPES_ALLOW_CONST_VERTICES and a proper constant vertex support. The one from
 *	2013-11-01 was not very practical (required modification to the vertex types) and was never
 *	really finished / supported. That goes a long way saying how needed this feature is.
 *
 */

/**
 *	@page constvertices Using Constant Vertices
 *
 *	Usually, all the vertices are <em>optimized</em>. That means they are somehow represented
 *	in the system matrix and their value potentially changes with optimization. Sometimes, it
 *	is required to have <em>constant</em> vertices, which will not be optimized. A constant
 *	vertex is not represented in the system matrix and its value never changes.
 *
 *	Before this was implemented, it was possible to have ad-hoc constant vertices by reducing
 *	degree of any edge that was to reference such vertex and keeping a pointer to this vertex,
 *	which would be stored in user - managed storage instead of in the optimized system. Then
 *	the jacobian and right hand side for this vertex would not be returned by the edge, and would
 *	not make it to the system. While functional, it is kind of a lot of work.
 *
 *	@section constvertices_sec0 Constant Vertex Support in SLAM++ 2.0
 *
 *	Since version 2.0 RC 2, constant vertices can be stored in the optimized system, reducing the
 *	need for user - managed storage. This also means that we need to specify which vertex types
 *	will be constant:
 *
 *	@code
 *	typedef MakeTypelist_Safe((CEdgePose2D, CEdgePoseLandmark2D)) TEdgeTypelist; // a list of edges, as before
 *	typedef MakeTypelist_Safe((CVertexPose2D, CVertexLandmark2D)) TVertexTypelist; // a list of optimized vertices, as before
 *	typedef MakeTypelist_Safe((CVertexPose2D, CVertexLandmark2D)) TConstVertexTypelist; // a new list of constant vertices
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist, CBasicUnaryFactorFactory,
 *		CBaseVertex, TConstVertexTypelist> CSystemType;
 *	// make a system permitting SE(2) vertex and edge types
 *	@endcode
 *
 *	Note that this actually increases optimization of the solver, as it is possible to have a set of
 *	vertex types to be optimized and a completely different set of vertex types to be constant. The
 *	optimizer then does not need to support matrices with block sizes corresponding to the constant
 *	vertices, which potentially makes it faster.
 *
 *	You can add constant vertices by \ref CFlatSystem::r_Get_ConstVertex()
 *	and access them by \ref CFlatSystem::r_ConstVertex_Pool(). There is just one catch.
 *
 *	@section constvertices_sec1 Constant Vertex Indexing
 *
 *	To be able to seamlessly support constant vertices without having to modify the vertex types
 *	or the graph file parser, constant vertices are identified using their indices. The optimized vertices
 *	have indices starting with zero and increasing by one (plain zero-based indices). The constant vertices
 *	have indices starting with \ref SIZE_MAX (the maximum value of the index) and <em>decreasing</em> by one.
 *	This also means that it is possible to specify the constant vertex indices in the graph file as
 *	negative numbers, starting with -1 (-1 converts to all ones, -2 converts to all ones minus one, etc.).
 *
 *	That way, it is possible to add the constant vertices using \ref CFlatSystem::r_Get_Vertex() which
 *	is called by all the edge types and they are added that way when parsing a graph file. The
 *	\ref CFlatSystem::r_Get_ConstVertex() function is using <em>the same</em> indices (large unsigned numbers)
 *	and it cannot be used to add optimized vertices (small unsigned numbers).
 *
 *	<em>However</em>, the vertex pool \ref CFlatSystem::r_ConstVertex_Pool() uses simple zero-based
 *	indices <tt>i</tt> which map to <tt>SIZE_MAX - i</tt> in the global index space (the one used in the graph
 *	file and by \ref CFlatSystem::r_Get_Vertex() or \ref CFlatSystem::r_Get_ConstVertex()). The pool does not
 *	know that its contents are special and thus uses standard zero-based indices.
 *
 *	To determine whether a vertex is constant, do not use its id. The vertex class has
 *	a \ref CBaseVertex::b_IsConstant() function. In case you don't want to use constant vertices
 *	and thus \ref __BASE_TYPES_ALLOW_CONST_VERTICES is not defined, this function will be optimized
 *	away and your code will run faster.
 *
 *	Note that there is no fixed limit for the number of constant or optimized vertices. Since the vertex
 *	ids have to be contiguous, the optimized vertices are getting ids starting with zero and the constant
 *	vertices are getting the decreasing ids from the opposite end of index value range. If you do not use
 *	constant vertices, you can still have <tt>SIZE_MAX - 1</tt> optimized vertices. If you want only constant
 *	vertices, you can also have <tt>SIZE_MAX - 1</tt> of them. Whether the vertex is constant is not
 *	determined by the most significant bit of the id but simply by id adjacency. That limits the total
 *	number of vertices to <tt>num_const + num_optimized <= SIZE_MAX - 1</tt> (one id needs to remain free
 *	since it would be ambiguous to which pool the vertex belongs). Determining the const-ness of a vertex
 *	by the most significant bit of the id would limit the number of const or optimized vertices to
 *	<tt>SIZE_MAX / 2</tt>.
 *
 *	@section constvertices_sec2 A Note on Cost and Possible Optimizations
 *
 *	The run-time overhead of supporting constant vertices when they are not being used corresponds mostly
 *	to the edges having to check whether their vertices are constant or not. If you are not using constant
 *	vertices, you can get very slightly more efficient code by undefining \ref __BASE_TYPES_ALLOW_CONST_VERTICES.
 *
 *	Another overhead of supporting constant vertices while actually using them comes from the edge
 *	interface, where the base edge implementation calls the <tt>Calculate_Jacobians_Expectation_Error()</tt>
 *	function of the derived edge. The derived edge needs to return all the Jacobians by definition,
 *	while the ones corresponding to the constant vertices will be unused. If the calculation of the
 *	Jacobians is costly, it is possible for the derived edge to check which vertices are constant and
 *	omitting calculation of those Jacobians (they need to be returned, but they can be left uninitialized
 *	and they are guaranteed not to be used as long as the corresponding vertex is constant). This could look like:
 *
 *	@code
 *	void SomeDerivedEdge::Calculate_Jacobians_Expectation_Error(Eigen::Matrix6d &r_t_jacobian0,
 *		Eigen::Matrix6d &r_t_jacobian1, Eigen::Vector6d &r_v_expectation, Eigen::Vector6d &r_v_error) const
 *	{
 *		CalculateExpectation(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(), r_v_expectation);
 *		InverseCompose(r_v_expectation, m_v_measurement, r_v_error);
 *		// calculates the expectation and error
 *
 *		if(!m_p_vertex0->b_IsConstant())
 *			r_t_jacobian0 = CalculateFirstJacobian(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State());
 *		if(!m_p_vertex1->b_IsConstant())
 *			r_t_jacobian1 = CalculateSecondJacobian(m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State());
 *		// calculate the jacobians as needed
 *	}
 *	@endcode
 *
 *	In case it is known that a specific edge will always have a constant vertex, it is possible to create
 *	a modified edge type with one less vertex (so e.g. if the original edge was binary, this will make it
 *	unary). It is still possible to use \ref CFlatSystem::r_Get_ConstVertex() for storage, and it is again
 *	possible to save calculation of the Jacobian corresponding to the constant vertex. As an example,
 *	we can take \ref CEdgePoseLandmark2D and modify it, assuming that the landmark vertex will be always
 *	constant:
 *
 *	@code
 *	class CEdgePoseConstLandmark2D : public CBaseEdgeImpl<CEdgePoseConstLandmark2D,
 *		MakeTypelist(CVertexPose2D), 2> { // unary edge, 2D measurement
 *	protected:
 *		typedef CBaseEdgeImpl<CEdgePoseConstLandmark2D, MakeTypelist(CVertexPose2D), 2> _TyBase;
 *
 *	protected:
 *		const CVertexLandmark2D *m_p_vertex1; // pointer to the constant vertex, have to define it ourselves
 *
 *	public:
 *		template <class CSystem>
 *		CEdgePoseLandmark2D(size_t n_node0, size_t n_node1, const Eigen::Vector2d &r_v_delta,
 *			const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
 *			:_TyBase(n_node0, r_v_delta, r_t_inv_sigma) // although this ctor is binary, the edge is unary
 *		{
 *			m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(n_node0,
 *				CInitializeNullVertex<>());
 *			// get the optimized vertex
 *
 *			m_p_vertex1 = &r_system.template r_Get_ConstVertex<CVertexLandmark2D>(n_node1,
 *				CEdgePoseLandmark2D::CRelative_to_Absolute_RangeBearing_Initializer(m_p_vertex0->r_v_State(), r_v_delta));
 *			// get the const vertex (will fail if n_node1 is not a valid id of a const vertex)
 *		}
 *
 *		void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian,
 *			Eigen::Vector2d &r_v_expectation, Eigen::Vector2d &r_v_error) const
 *		{
 *			Eigen::Matrix2d dummy_jacobian;
 *			C2DJacobians::Observation2D_RangeBearing(
 *				m_p_vertex0->r_v_State(), m_p_vertex1->r_v_State(),
 *				r_v_expectation, r_t_jacobian0, dummy_jacobian);
 *			// calculates the expectation and the jacobians (the second jacobian is not needed, we could optimize for that)
 *
 *			r_v_error = m_v_measurement - r_v_expectation;
 *			r_v_error(1) = C2DJacobians::f_ClampAngularError_2Pi(r_v_error(1));
 *		}
 *	};
 *	@endcode
 *
 *	Sometimes, if there is 1:1 mapping of constant vertices to edges, it is again possible to reduce the
 *	degree of the edge, but this time rather than using constant vertices, make the vertex state a part
 *	of the edge (be it by sharing the measurement vector for both the measurement and the state or by adding
 *	a new member variable). A simple use case of this could be optimizing trajectory of a robot with a GPS,
 *	where the GPS positions are not optimized and they are linked to robot poses by binary edges. These
 *	edges will become unary and the GPS positions will be a part of the edge, rather than a new vertex.
 *	Modifying the example above, this would look like:
 *
 *	@code
 *	class CEdgePoseConstLandmark2D : public CBaseEdgeImpl<CEdgePoseConstLandmark2D,
 *		MakeTypelist(CVertexPose2D), 2> { // unary edge, 2D measurement
 *	protected:
 *		typedef CBaseEdgeImpl<CEdgePoseConstLandmark2D, MakeTypelist(CVertexPose2D), 2> _TyBase;
 *
 *	protected:
 *		const Eigen::Vector2d m_v_vertex1; // state of the constant vertex, stored as a separate variable
 *		// alternately we could define the base as:
 *		// CBaseEdgeImpl<CEdgePoseConstLandmark2D, MakeTypelist(CVertexPose2D), 2, 4>
 *		// which allocates 4D vector for measurement (we use the first two elements for the
 *		// measurement and the last two for the vertex 1 state)
 *
 *	public:
 *		template <class CSystem>
 *		CEdgePoseLandmark2D(size_t n_node0, const Eigen::Vector2d &r_v_node1,
 *			const Eigen::Vector2d &r_v_delta, const Eigen::Matrix2d &r_t_inv_sigma, CSystem &r_system)
 *			:_TyBase(n_node0, r_v_delta, r_t_inv_sigma), m_v_vertex1(r_v_node1)
 *		{
 *			m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(n_node0,
 *				CInitializeNullVertex<>());
 *			// get the optimized vertex
 *		}
 *
 *		void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 2, 3> &r_t_jacobian,
 *			Eigen::Vector2d &r_v_expectation, Eigen::Vector2d &r_v_error) const
 *		{
 *			Eigen::Matrix2d dummy_jacobian;
 *			C2DJacobians::Observation2D_RangeBearing(
 *				m_p_vertex0->r_v_State(), m_v_vertex1, // use the remembered state here
 *				r_v_expectation, r_t_jacobian0, dummy_jacobian);
 *			// calculates the expectation and the jacobians (the second jacobian is not needed, we could optimize for that)
 *
 *			r_v_error = m_v_measurement - r_v_expectation;
 *			r_v_error(1) = C2DJacobians::f_ClampAngularError_2Pi(r_v_error(1));
 *		}
 *	};
 *	@endcode
 *
 *	@section constvertices_sec3 Limitations
 *
 *	Each edge must reference at least a single optimized vertex, otherwise such an edge is a waste
 *	of computation (although it still contributes to \f$\chi^2\f$ error, it cannot have direct effect
 *	on the optimization). As a consequence, unary edges cannot use constant vertices at all.
 *
 *	One limitation of this system is that vertices cannot be easily changed from optimized
 *	to constant or vice versa once they were introduced to the system.
 *
 *	If \ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO is defined (default), some edge must contain vertex with id 0
 *	before the system can be optimized (ideally the first edge but that may not always be the case). This is
 *	usually not a problem since a) in each edge there must be at least one optimized vertex and at the
 *	same time, b) vertices must be added in incremental manner starting with zero. But e.g. in bundle
 *	adjustment, the vertices are initialized explicitly rather than being added by the edges which
 *	reference them. Then the edge with vertex 0 no longer has to be the first. In the Venice dataset, the
 *	edge with vertex 0 appears quite far in the dataset and as a consequence it is not possible to process
 *	this dataset incrementally because when the first edge is added and the optimizer would run, the unary
 *	factor is not yet initialized (also most vertices are disconnected). But it is still fine for batch.
 *
 *	If \ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO is not defined, then the first vertex in the first edge
 *	(which might not be vertex with id 0) must not be a constant vertex. If you need that and want to avoid
 *	\ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO for some reason, you need to use \ref CNullUnaryFactorFactory
 *	as the fifth template argument in your \ref CFlatSystem specialization and explicitly add a
 *	\ref CUnaryFactor.
 *
 */

#include "eigen/Eigen/Core"
#include "slam/TypeList.h"
#include "slam/Segregated.h"
#include "slam/BlockMatrix.h"
#include "slam/BaseInterface.h"

/**
 *	@def __BASE_TYPES_USE_ID_ADDRESSING
 *	@brief if defined, the faster matrix addressing using vertex and edge id's is used (blockwise)
 *		otherwise addressing based on vertex and edge orders (elementwise) is used
 *	@note This must be defined here as CFlatSystem depends on it.
 */
//#define __BASE_TYPES_USE_ID_ADDRESSING

/**
 *	@def __BASE_TYPES_ALLOW_CONST_VERTICES
 *	@brief if defined, the base vertex and edge implementations will check for constant vertices,
 *		otherwise all the vertices are assumed to be optimized
 *	@note This must be defined here as CFlatSystem depends on it.
 */
#define __BASE_TYPES_ALLOW_CONST_VERTICES

/**
 *	@def __FLAT_SYSTEM_USE_SIZE2D_FOR_BLOCK_SIZE
 *	@brief if defined, CFlatSystem::_TyJacobianMatrixBlockList and
 *		CFlatSystem::_TyHessianMatrixBlockList will contain lists of fbs_ut::CCTSize2D
 *		instead of Eigen::Matrix specializazions
 *	@note This speeds up compilation as fbs_ut::CCTSize2D is easier to resolve.
 */
#define __FLAT_SYSTEM_USE_SIZE2D_FOR_BLOCK_SIZE

/**
 *	@def __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
 *	@brief if defined, the automatically inserted unary factor is on vertex with id 0
 *		rather than the first vertex of the first edge which may not be the same vertex.
 *	@note This potentially changes the behavior of SLAM++, although it does not change
 *		the behavior of the built-in types on standard datasets.
 */
#define __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO

/**
 *	@def __FLAT_SYSTEM_ALIGNED_MEMORY
 *	@brief if defined, vertex and edge pools will be aligned so that SSE can be used
 *
 *	This option subsequently enables \ref __GRAPH_TYPES_ALIGN_OPERATOR_NEW and
 *	\ref __BASE_TYPES_USE_ALIGNED_MATRICES in \ref include/slam/BaseTypes.h and
 *	makes all the graph primitives aligned. This means that some of the operations
 *	on the vertices / measurements will potentially use SSE (the matrix operations
 *	will use SSE nevertheless, unless disabled). It also means that the system will
 *	be allocated in aligned memory and the graph types will be slightly larger
 *	due to alignment padding where needed, making SLAM++ use slightly more memory.
 *
 *	On modern architectures, the use of unaligned memory for graph primitives
 *	does not cost as much as could be expected. Using aligned memory costs a bit
 *	because the aligned malloc is slightly slower and also the alignment causes
 *	slight cache thrashing. All in all, using aligned memory saves about 1 second
 *	on the Venice dataset on a modern Xeon CPU (about 2% improvement of time).
 *
 *	This is disabled by default and can be enabled in CMake configuration.
 */
//#define __FLAT_SYSTEM_ALIGNED_MEMORY

/**
 *	@brief basic implementation of null unary factor initializer
 *	@note An explicit UF needsto be provided to make the system matrix positive definite.
 */
class CNullUnaryFactorFactory {
public:
	/**
	 *	@brief initializes unary factor, based on the first edge introduced to the system
	 *
	 *	@tparam CEdge is edge type
	 *
	 *	@param[out] r_t_unary_factor is the unary factor matrix
	 *	@param[out] r_v_unary_error is the error vector associated with the first vertex
	 *	@param[in] r_edge is the first edge in the system
	 *
	 *	@note This function throws std::runtime_error if \ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
	 *		is defined and the edge does not contain vertex with id 0.
	 */
	template <class CEdge>
	inline void operator ()(Eigen::MatrixXd &r_t_unary_factor,
		Eigen::VectorXd &r_v_unary_error, const CEdge &r_edge) const // throw(std::runtime_error)
	{
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		size_t n_vertex_dimension = n_FirstVertex_Dimension(r_edge);
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		typedef typename CEdge::template CVertexTraits<0> VT; // g++
		size_t n_vertex_dimension = VT::n_dimension;
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		r_t_unary_factor.resize(n_vertex_dimension, n_vertex_dimension); // unary factor is a unit matrix
		r_t_unary_factor.setZero();
		r_v_unary_error = Eigen::VectorXd(n_vertex_dimension);
		r_v_unary_error.setZero(); // no error on the first vertex
	}

	/**
	 *	@brief determines dimension of the first vertex (assumed to be in the edge)
	 *
	 *	@tparam CEdge is edge type
	 *
	 *	@param[in] r_edge is the first edge in the system
	 *
	 *	@return Returns the dimension of the first vertex in the system.
	 *
	 *	@note This is only used if \ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO is defined.
	 *	@note This function throws std::runtime_error if the edge does not contain vertex with id 0.
	 */
	template <class CEdge>
	static size_t n_FirstVertex_Dimension(const CEdge &r_edge) // throw(std::runtime_error)
	{
		typedef typename CTransformTypelist<typename CEdge::_TyVertices,
			fbs_ut::CGetVertexDimension>::_TyResult CVertexSizeList;
		int p_vertex_dimension[CEdge::n_vertex_num];
		fbs_ut::Copy_CTSizes_to_Array<CVertexSizeList>(p_vertex_dimension);
		// copy to a runtime-addressable array

		for(int i = 0; i < int(CEdge::n_vertex_num); ++ i) {
			if(!r_edge.n_Vertex_Id(i))
				return p_vertex_dimension[i];
		}
		// look for vertex with id 0

		_ASSERTE(0); // if this triggers, this edge does not have vertex with id 0, and is therefore not the first edge in the system
		throw std::runtime_error("edge 0 does not reference vertex 0: the graph is disconnected");

		return size_t(-1); // just to satisfy the compiler
	}
};

/**
 *	@brief very basic implementation of unary factor initialization that uses unit UF
 */
class CBasicUnaryFactorFactory {
protected:
	double m_f_information; /**< @brief information value; the information matrix will be identity matrix scaled by this value */

public:
	/**
	 *	@brief default constructor; sets information value
	 *	@param[in] f_information is information matrix diagonal value (default 1.0)
	 */
	inline CBasicUnaryFactorFactory(double f_information = 1)
		:m_f_information(f_information)
	{}

	/**
	 *	@brief initializes unary factor, based on the first edge introduced to the system
	 *
	 *	@param[out] r_t_unary_factor is the unary factor matrix
	 *	@param[out] r_v_unary_error is the error vector associated with the first vertex
	 *	@param[in] r_edge is the first edge in the system (value not used)
	 *
	 *	@note This function throws std::runtime_error if \ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
	 *		is defined and the edge does not contain vertex with id 0.
	 */
	template <class CEdge>
	inline void operator ()(Eigen::MatrixXd &r_t_unary_factor,
		Eigen::VectorXd &r_v_unary_error, const CEdge &r_edge) const // throw(std::runtime_error)
	{
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		size_t n_vertex_dimension = CNullUnaryFactorFactory::n_FirstVertex_Dimension(r_edge);
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		typedef typename CEdge::template CVertexTraits<0> VT; // g++
		size_t n_vertex_dimension = VT::n_dimension;
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		//r_t_unary_factor.resize(n_vertex_dimension, n_vertex_dimension); // unary factor is a unit matrix
		//r_t_unary_factor.setIdentity();
		r_t_unary_factor = Eigen::MatrixXd::Identity(n_vertex_dimension, n_vertex_dimension) * m_f_information;
		r_v_unary_error = Eigen::VectorXd(n_vertex_dimension);
		r_v_unary_error.setZero(); // no error on the first vertex
	}
};

/**
 *	@brief implementation of unary factor initialization that takes cholesky
 *		of the information matrix of the first edge as unary factor
 *	@note For this to work, the measurement dimension must equal the first vertex dimension.
 */
class CProportionalUnaryFactorFactory {
public:
	/**
	 *	@brief initializes unary factor, based on the first edge introduced to the system
	 *
	 *	@param[out] r_t_unary_factor is the unary factor matrix
	 *	@param[out] r_v_unary_error is the error vector associated with the first vertex
	 *	@param[in] r_edge is the first edge in the system
	 *
	 *	@note This function throws std::runtime_error if \ref __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
	 *		is defined and the edge does not contain vertex  0 or if there is dimension mismatch.
	 */
	template <class CEdge>
	inline void operator ()(Eigen::MatrixXd &r_t_unary_factor,
		Eigen::VectorXd &r_v_unary_error, const CEdge &r_edge) const // throw(std::runtime_error)
	{
#ifdef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		size_t n_vertex_dimension = CNullUnaryFactorFactory::n_FirstVertex_Dimension(r_edge);
#else // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		typedef typename CEdge::template CVertexTraits<0> VT; // g++
		size_t n_vertex_dimension = VT::n_dimension;
#endif // __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
		if(n_vertex_dimension != r_edge.n_Dimension()) {
			throw std::runtime_error("dimension of the first measurement does not "
				"match the first vertex dimension: unable to derive unary factor");
		}
		r_t_unary_factor = r_edge.t_Sigma_Inv().llt().matrixU(); // unary factor is cholesky of the information matrix
		r_v_unary_error = Eigen::VectorXd(n_vertex_dimension);
		r_v_unary_error.setZero(); // no error on the first vertex
	}
};

/**
 *	@brief multiple data type (heterogenous), yet statically typed pools
 *	@todo - put this to multipool.h or something
 */
namespace multipool {

/**
 *	@brief static assertion helper
 *	@brief b_expression is expression being asserted
 */
template <const bool b_expression>
class CStaticAssert {
public:
	typedef void BASE_TYPE_MUST_MATCH_THE_ONLY_TYPE_IN_THE_LIST; /**< @brief static assertion tag; when defining multipools with just a single type, the base type is required to be the same as that type */
	typedef void REQUESTED_TYPE_MISMATCH; /**< @brief static assertion tag; when calling CMultiPool::r_At(), the requested type must match one of the types in the list the CMultiPool was specialized with */
	typedef void MULTIPOOL_TYPE_ID_INTERNAL_ERROR; /**< @brief static assertion tag; internal error in CMultiPool type id calculation */
};

/**
 *	@brief static assertion helper (specialization for assertion failed)
 */
template <>
class CStaticAssert<false> {};

/**
 *	@brief list of pools per every data type in the list
 *	@note While this works as a simple multipool, it isn't entirely practical without accessors.
 */
template <class CListType, const int n_pool_page_size, const int n_pool_memory_align>
class CPoolList {
public:
	typedef typename CListType::_TyHead TPayloadType; /**< @brief payload data type */
	typedef forward_allocated_pool<TPayloadType, n_pool_page_size, n_pool_memory_align> _TyFirstPool; /**< @brief pool of the first data type being stored */

protected:
	_TyFirstPool m_pool; /**< @brief contains vertex pool of a given type */
	CPoolList<typename CListType::_TyTail, n_pool_page_size, n_pool_memory_align> m_recurse; /**< @brief contains vertex pools of all the types following in the list */

public:
	/**
	 *	@brief adds a new element to the pool
	 *
	 *	@param[out] r_p_storage is pointer to the place where the element is stored
	 *	@param[in] r_t_payload is initialization value for the new element
	 *
	 *	@return Returns index of the pool the element was added to.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CGenericPalyoadType>
	inline int n_AddElement(CGenericPalyoadType *&r_p_storage, const CGenericPalyoadType &r_t_payload)
	{
		return 1 + m_recurse.n_AddElement(r_p_storage, r_t_payload); // not this type of payload, perhaps recurse can contain it
		// note that if the compiler generates errors here, the type geing inserted is most likely not on the list
		// t_odo - make static assertion here that would print a human-readable message as well
	}

	/**
	 *	@brief adds a new element to the pool (version specialized for the payload type of this pool)
	 *
	 *	@param[out] r_p_storage is pointer to the place where the element is stored
	 *	@param[in] r_t_payload is initialization value for the new element
	 *
	 *	@return Returns index of the pool the element was added to.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline int n_AddElement(TPayloadType *&r_p_storage, const TPayloadType &r_t_payload)
	{
		m_pool.push_back(r_t_payload); // our type of payload
		r_p_storage = &*(m_pool.end() - 1); // return pointer to the payload
		return 0;
	}

	/**
	 *	@brief gets access to the data pool, containing the first type in the list
	 *	@return Returns reference to the first data pool.
	 */
	inline _TyFirstPool &r_Pool()
	{
		return m_pool;
	}

	/**
	 *	@brief gets access to the data pool, containing the first type in the list
	 *	@return Returns const reference to the first data pool.
	 */
	inline const _TyFirstPool &r_Pool() const
	{
		return m_pool;
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		return sizeof(CPoolList<CListType, n_pool_page_size, n_pool_memory_align>) -
			sizeof(m_recurse) + m_pool.capacity() * sizeof(TPayloadType) +
			m_pool.page_num() * sizeof(TPayloadType*);
	}
};

/**
 *	@brief list of pools per every data type in the list (the list terminator specialization)
 */
template <const int n_pool_page_size, const int n_pool_memory_align>
class CPoolList<CTypelistEnd, n_pool_page_size, n_pool_memory_align> {
public:
	/**
	 *	@copydoc CPoolList::n_Allocation_Size()
	 */
	inline size_t n_Allocation_Size() const
	{
		return 0;
	}
};

/**
 *	@brief heterogenous (but static) pool container
 *
 *	@tparam CBaseType is base type of items being stored in the multipool
 *	@tparam CTypelist is list of types being stored in the multipool
 *	@tparam n_pool_page_size is pool page size (per each type being stored)
 */
template <class CBaseType, class CTypelist, const size_t n_pool_page_size = 1024>
class CMultiPool {
public:
	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		pool_PageSize = n_pool_page_size, /**< @brief size of pool page, in elements */
#ifdef __FLAT_SYSTEM_ALIGNED_MEMORY
		pool_MemoryAlign = 16 /**< @brief memory alignment of data stored in the pool */
#else // __FLAT_SYSTEM_ALIGNED_MEMORY
		pool_MemoryAlign = 0 /**< @brief memory alignment of data stored in the pool */ // or 16 for aligned types and SSE // t_odo
#endif // __FLAT_SYSTEM_ALIGNED_MEMORY
	};

	typedef CBaseType _TyBaseType; /**< @brief base type of items being stored in the multipool */
	typedef CTypelist _TyTypelist; /**< @brief list of types being stored in the multipool */

#ifndef __FLAT_SYSTEM_USE_THUNK_TABLE
	typedef _TyBaseType &_TyBaseRef; /**< @brief base type reference */
	typedef const _TyBaseType &_TyConstBaseRef; /**< @brief base type const */
#else // !__FLAT_SYSTEM_USE_THUNK_TABLE
	typedef CFacadeTraits<CBaseType> _TyFacade; /**< @brief facade traits */
	typedef typename _TyFacade::_TyReference _TyBaseRef; /**< @brief base type reference (wrapped in a facade interface) */
	typedef typename _TyFacade::_TyConstReference _TyConstBaseRef; /**< @brief base type const (wrapped in a facade interface) */
#endif // !__FLAT_SYSTEM_USE_THUNK_TABLE

protected:
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		type_Count = CTypelistLength<CTypelist>::n_result, /**< @brief number of data types in the pool */
		type_Id_Bit_Num = n_Bit_Width_Static(type_Count - 1), /**< @brief number of bits required to store a type id */
		type_Id_Byte_Num = (type_Id_Bit_Num + 7) / 8, /**< @brief number of bytes required to store a type id */
		type_Id_Index = (type_Count == 2)? 0 : type_Id_Byte_Num, /**< @brief index of type id in the lookup table below */
		debug_Checks = type_Count >= 2 && type_Id_Bit_Num > 0 && type_Id_Byte_Num > 0 &&
			type_Id_Bit_Num <= type_Id_Byte_Num * 8 && !(~n_Mask_Static(type_Id_Bit_Num) & (type_Count - 1)) &&
			(type_Count != 2 || !type_Id_Index) /**< @brief assorted debug checks */
	};

	typedef uint8_t TOneBitInt; /**< @brief data type to store a single bit integer (could be bool or a byte, depending on what is faster) @note Using bool requires less memory but seems to be very slightly slower. */
	typedef MakeTypelist(TOneBitInt, uint8_t, uint16_t, int32_t, uint32_t,
		int64_t, int64_t, int64_t, uint64_t) CIntList; /**< @brief helper list of integer types that can handle a given number of types (zero-based index) */
	typedef typename CTypelistItemAt<CIntList, type_Id_Index>::_TyResult CTypeIdIntType; /**< @brief integral type, able to store a type id */
	// note that if std::vector<bool> turns out to be faster, we could roll our own bit array to support various size int types (e.g. pack 3 types in 2 bits, ...)

	/**
	 *	@brief intermediates, stored as enum
	 */
	enum {
		debug_Checks2 = sizeof(CTypeIdIntType) == n_Make_POT_Static(type_Id_Byte_Num) || !type_Id_Index /**< @brief make sure that the number of bytes of type if matches the expectations, with the exception of type id 0 which is reserved for single bit ids @note Although <tt>sizeof(bool)</tt> is 1 on most platforms, according to §5.3.3/1 it is implementation defined. */
	};

	typedef typename CStaticAssert<debug_Checks && debug_Checks2>::MULTIPOOL_TYPE_ID_INTERNAL_ERROR CAssert0; /**< @brief static assertion making sure that the type id calculations are correct (if this triggers, please let us know) */

	std::vector<CTypeIdIntType> m_type_id_list; /**< @brief list of type ids (zero-based index in _TyTypelist) */
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	std::vector<_TyBaseType*> m_uniform_list; /**< @brief list of pointers with uniform data types (use virtual functions) */
	CPoolList<_TyTypelist, pool_PageSize, pool_MemoryAlign> m_pool_list;  /**< @brief list of all the pools for all the data types */

	/**
	 *	@brief function object that transparently inserts pointer dereferencing
	 *		between std::for_each and client function object
	 *	@tparam COp is client function object
	 */
	template <class COp>
	class CDereference {
	protected:
		COp m_op; /**< @brief client function object */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] op is instance of client function object
		 */
		inline CDereference(COp op)
			:m_op(op)
		{}

		/**
		 *	@brief function operator; dereferences the element
		 *		and passes it to the client function object
		 *	@param[in] p_elem is pointer to the element of the multipool
		 */
		inline void operator ()(_TyBaseType *p_elem)
		{
			m_op(*p_elem);
		}

		/**
		 *	@brief conversion to the client function object
		 *	@return Returns instance of client function object,
		 *		passed to the constructor.
		 */
		inline operator COp() const
		{
			return m_op;
		}
	};

public:
	/**
	 *	@brief adds an element to the multipool (at the end)
	 *	@tparam CGenericPalyoadType is type of the element being inserted
	 *		(must be on the list, otherwise the call will result in compile errors)
	 *	@param[in] r_t_payload is the element being inserted
	 *	@return Returns reference to the inserted element
	 *		(the address is guaranteed not to change).
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CGenericPalyoadType>
	CGenericPalyoadType &r_Add_Element(const CGenericPalyoadType &r_t_payload) // throw(std::bad_alloc)
	{
		CGenericPalyoadType *p_storage;
		int n_type_id = m_pool_list.n_AddElement(p_storage, r_t_payload);
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		_ASSERTE(n_type_id >= 0 && n_type_id < type_Count);
		_ASSERTE(CTypeIdIntType(n_type_id) == n_type_id);
		m_type_id_list.push_back(CTypeIdIntType(n_type_id));
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
		m_uniform_list.push_back(static_cast<CBaseType*>(p_storage));
		return *p_storage;
	}

	/**
	 *	@brief determines whether the multipool is empty
	 *	@return Returns true in case the multipool is empty, otherwise returns false.
	 */
	inline bool b_Empty() const
	{
		return m_uniform_list.empty();
	}

	/**
	 *	@brief gets number of elements
	 *	@return Returns number of elements stored in the multipool.
	 */
	inline size_t n_Size() const
	{
		return m_uniform_list.size();
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		return m_uniform_list.capacity() * sizeof(_TyBaseType*) +
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
			m_type_id_list.capacity() * sizeof(CTypeIdIntType) +
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
			sizeof(CMultiPool<CBaseType, CTypelist, n_pool_page_size>) -
			sizeof(m_pool_list) + m_pool_list.n_Allocation_Size();
	}

	/**
	 *	@brief gets an element with a specified type
	 *	@tparam CRequestedType is type of the element (must be one of types in
	 *		_TyTypelist and must match the type the specified element was created as)
	 *	@param[in] n_index is zero-based element index
	 *	@return Returns a reference to the selected element.
	 */
	template <class CRequestedType>
	inline CRequestedType &r_At(size_t n_index)
	{
		typedef typename CStaticAssert<CFindTypelistItem<_TyTypelist,
			CRequestedType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be one of the types in the list

#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		enum {
			compiletime_TypeId = CFindTypelistItem<_TyTypelist, CRequestedType>::n_index
		};
		_ASSERTE(compiletime_TypeId == m_type_id_list[n_index]);
		// make sure that the requested type matches the type id at runtime
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE

		return *static_cast<CRequestedType*>(m_uniform_list[n_index]);
	}

	/**
	 *	@brief gets an element with a specified type
	 *	@tparam CRequestedType is type of the element (must be one of types in
	 *		_TyTypelist and must match the type the specified element was created as)
	 *	@param[in] n_index is zero-based element index
	 *	@return Returns a const reference to the selected element.
	 */
	template <class CRequestedType>
	inline const CRequestedType &r_At(size_t n_index) const
	{
		typedef typename CStaticAssert<CFindTypelistItem<_TyTypelist,
			CRequestedType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be one of the types in the list

#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		enum {
			compiletime_TypeId = CFindTypelistItem<_TyTypelist, CRequestedType>::n_index
		};
		_ASSERTE(compiletime_TypeId == m_type_id_list[n_index]);
		// make sure that the requested type matches the type id at runtime
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE

		return *static_cast<const CRequestedType*>(m_uniform_list[n_index]);
	}

	/**
	 *	@brief gets an element
	 *	@param[in] n_index is zero-based element index
	 *	@return Returns a polymorphic wrapper of the selected element.
	 */
	inline _TyBaseRef operator [](size_t n_index)
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		return _TyFacade::template H<_TyTypelist>::MakeRef(*m_uniform_list[n_index], m_type_id_list[n_index]);
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
		return *m_uniform_list[n_index];
		// this is a problem, as even vertex state accessor is virtual, need to make a thunkinterface
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief gets an element
	 *	@param[in] n_index is zero-based element index
	 *	@return Returns a polymorphic wrapper of the selected element.
	 */
	inline _TyConstBaseRef operator [](size_t n_index) const
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		return _TyFacade::template H<_TyTypelist>::MakeRef(*m_uniform_list[n_index], m_type_id_list[n_index]);
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
		return *m_uniform_list[n_index];
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over all the elements in this multipool and performs operation on each
	 *	@tparam COp is client function object
	 *	@param[in] op is instance of the client function object
	 *	@return Returns instance of the client function object,
	 *		after performing all the operations (can e.g. perform reduction).
	 */
	template <class COp>
	COp For_Each(COp op)
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> >::r_Get();
		for(size_t i = 0, n = m_uniform_list.size(); i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
		return op;
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
		return std::for_each(m_uniform_list.begin(),
			m_uniform_list.end(), CDereference<COp>(op));
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over all the elements in this multipool and performs operation on each
	 *	@tparam COp is client function object
	 *	@param[in] op is instance of the client function object
	 *	@return Returns instance of the client function object,
	 *		after performing all the operations (can e.g. perform reduction).
	 */
	template <class COp>
	COp For_Each(COp op) const
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> >::r_Get();
		for(size_t i = 0, n = m_uniform_list.size(); i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
		return op;
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
		return std::for_each(m_uniform_list.begin(),
			m_uniform_list.end(), CDereference<COp>(op));
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over all the elements in this multipool and performs operation on each
	 *
	 *	@tparam COp is client function object
	 *
	 *	@param[in] op is instance of the client function object
	 *	@param[in] n_parallel_thresh is threshold for parallelized processing
	 *
	 *	@note This performs the iteration in parallel, the function object must be reentrant.
	 *	@note This does not return the function object (avoids synchronization
	 *		and explicit reduction).
	 */
	template <class COp>
	void For_Each_Parallel(COp op, const int n_parallel_thresh = 50)
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> >::r_Get();
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp parallel for default(shared) if(n >= n_parallel_thresh)
		for(int i = 0; i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef _OPENMP
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp parallel for default(shared) if(n >= n_parallel_thresh)
		for(int i = 0; i < n; ++ i)
			op(*m_uniform_list[i]);
#else // _OPENMP
		std::for_each(m_uniform_list.begin(), m_uniform_list.end(), CDereference<COp>(op));
#endif // _OPENMP
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over all the elements in this multipool and performs operation on each
	 *
	 *	@tparam COp is client function object
	 *
	 *	@param[in] op is instance of the client function object
	 *	@param[in] n_parallel_thresh is threshold for parallelized processing
	 *
	 *	@note This performs the iteration in parallel, the function object must be reentrant.
	 *	@note This does not return the function object (avoids synchronization
	 *		and explicit reduction).
	 */
	template <class COp>
	void For_Each_Parallel(COp op, const int n_parallel_thresh = 50) const
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> >::r_Get();
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp parallel for default(shared) if(n >= n_parallel_thresh)
		for(int i = 0; i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef _OPENMP
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp parallel for default(shared) if(n >= n_parallel_thresh)
		for(int i = 0; i < n; ++ i)
			op(*m_uniform_list[i]);
#else // _OPENMP
		std::for_each(m_uniform_list.begin(), m_uniform_list.end(), CDereference<COp>(op));
#endif // _OPENMP
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over all the elements in this multipool and performs
	 *		operation on each, allows cooperation of several threads
	 *	@tparam COp is client function object
	 *	@param[in] op is instance of the client function object
	 *	@return Returns instance of the client function object,
	 *		after performing all the operations (can e.g. perform reduction).
	 *	@note This does not create a new parallel region, rather it is expected to be called
	 *		from one. Therefore, there are no options for minimal parallel threshold (set that
	 *		up at the point where the parallel region is created).
	 */
	template <class COp>
	inline COp For_Each_WorkShare(COp op) const // just as parallel but does not start a parallel region, is supposed to be in one
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> >::r_Get();
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp for
		for(int i = 0; i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
		return op;
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef _OPENMP
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp for
		for(int i = 0; i < n; ++ i)
			op(*m_uniform_list[i]);
		return op;
#else // _OPENMP
		return std::for_each(m_uniform_list.begin(), m_uniform_list.end(), CDereference<COp>(op));
#endif // _OPENMP
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over all the elements in this multipool and performs
	 *		operation on each, allows cooperation of several threads
	 *	@tparam COp is client function object
	 *	@param[in] op is instance of the client function object
	 *	@return Returns instance of the client function object,
	 *		after performing all the operations (can e.g. perform reduction).
	 *	@note This does not create a new parallel region, rather it is expected to be called
	 *		from one. Therefore, there are no options for minimal parallel threshold (set that
	 *		up at the point where the parallel region is created).
	 */
	template <class COp>
	inline COp For_Each_WorkShare(COp op) // just as parallel but does not start a parallel region, is supposed to be in one
	{
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> >::r_Get();
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp for
		for(int i = 0; i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
		return op;
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef _OPENMP
		_ASSERTE(m_uniform_list.size() <= INT_MAX);
		const int n = int(m_uniform_list.size());
		#pragma omp for
		for(int i = 0; i < n; ++ i)
			op(*m_uniform_list[i]);
		return op;
#else // _OPENMP
		return std::for_each(m_uniform_list.begin(), m_uniform_list.end(), CDereference<COp>(op));
#endif // _OPENMP
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over a selected range of elements
	 *		in this multipool and performs operation on each
	 *
	 *	@tparam COp is client function object
	 *
	 *	@param[in] n_first is zero-based index of the first element to be processed
	 *	@param[in] n_last is zero-based index of one after the last element to be processed
	 *	@param[in] op is instance of the client function object
	 *
	 *	@return Returns instance of the client function object,
	 *		after performing all the operations (can e.g. perform reduction).
	 */
	template <class COp>
	COp For_Each(size_t n_first, size_t n_last, COp op)
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_uniform_list.size());
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> >::r_Get();
		for(size_t i = n_first; i < n_last; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
		return op;
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
		return std::for_each(m_uniform_list.begin() + n_first,
			m_uniform_list.begin() + n_last, CDereference<COp>(op));
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over a selected range of elements
	 *		in this multipool and performs operation on each
	 *
	 *	@tparam COp is client function object
	 *
	 *	@param[in] n_first is zero-based index of the first element to be processed
	 *	@param[in] n_last is zero-based index of one after the last element to be processed
	 *	@param[in] op is instance of the client function object
	 *
	 *	@return Returns instance of the client function object,
	 *		after performing all the operations (can e.g. perform reduction).
	 */
	template <class COp>
	COp For_Each(size_t n_first, size_t n_last, COp op) const
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_uniform_list.size());
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp&> >::r_Get();
		for(size_t i = n_first; i < n_last; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
		return op;
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
		return std::for_each(m_uniform_list.begin() + n_first,
			m_uniform_list.begin() + n_last, CDereference<COp>(op));
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over a selected range of elements
	 *		in this multipool and performs operation on each
	 *
	 *	@tparam COp is client function object
	 *	@param[in] n_parallel_thresh is threshold for parallelized processing
	 *
	 *	@param[in] n_first is zero-based index of the first element to be processed
	 *	@param[in] n_last is zero-based index of one after the last element to be processed
	 *	@param[in] op is instance of the client function object
	 *
	 *	@note This performs the iteration in parallel, the function object must be reentrant.
	 *	@note This does not return the function object (avoids synchronization
	 *		and explicit reduction).
	 */
	template <class COp>
	void For_Each_Parallel(size_t n_first, size_t n_last, COp op, const int n_parallel_thresh = 50)
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_uniform_list.size());
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> >::r_Get();
		_ASSERTE(n_last <= INT_MAX);
		const int n = int(n_last);
		#pragma omp parallel for default(shared) if(n - int(n_first) >= n_parallel_thresh)
		for(int i = int(n_first); i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef _OPENMP
		_ASSERTE(n_last <= INT_MAX);
		const int n = int(n_last);
		#pragma omp parallel for default(shared) if(n - int(n_first) >= n_parallel_thresh)
		for(int i = int(n_first); i < n; ++ i)
			op(*m_uniform_list[i]);
#else // _OPENMP
		std::for_each(m_uniform_list.begin() + n_first, m_uniform_list.begin() + n_last, CDereference<COp>(op));
#endif // _OPENMP
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}

	/**
	 *	@brief iterates over a selected range of elements
	 *		in this multipool and performs operation on each
	 *
	 *	@tparam COp is client function object
	 *	@param[in] n_parallel_thresh is threshold for parallelized processing
	 *
	 *	@param[in] n_first is zero-based index of the first element to be processed
	 *	@param[in] n_last is zero-based index of one after the last element to be processed
	 *	@param[in] op is instance of the client function object
	 *
	 *	@note This performs the iteration in parallel, the function object must be reentrant.
	 *	@note This does not return the function object (avoids synchronization
	 *		and explicit reduction).
	 */
	template <class COp>
	void For_Each_Parallel(size_t n_first, size_t n_last, COp op, const int n_parallel_thresh = 50) const
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_uniform_list.size());
#ifdef __FLAT_SYSTEM_USE_THUNK_TABLE
		base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> &thunk_table =
			base_iface::CFacadeAgglomerator<base_iface::CThunkTable<_TyBaseType, _TyTypelist, COp> >::r_Get();
		_ASSERTE(n_last <= INT_MAX);
		const int n = int(n_last);
		#pragma omp parallel for default(shared) if(n - int(n_first) >= n_parallel_thresh)
		for(int i = int(n_first); i < n; ++ i)
			thunk_table[m_type_id_list[i]](m_uniform_list[i], op);
#else // __FLAT_SYSTEM_USE_THUNK_TABLE
#ifdef _OPENMP
		_ASSERTE(n_last <= INT_MAX);
		const int n = int(n_last);
		#pragma omp parallel for default(shared) if(n - int(n_first) >= n_parallel_thresh)
		for(int i = int(n_first); i < n; ++ i)
			op(*m_uniform_list[i]);
#else // _OPENMP
		std::for_each(m_uniform_list.begin() + n_first, m_uniform_list.begin() + n_last, CDereference<COp>(op));
#endif // _OPENMP
#endif // __FLAT_SYSTEM_USE_THUNK_TABLE
	}
};

/**
 *	@brief heterogenous (but static) pool container (specialization
 *		for only a single type being stored)
 *
 *	@tparam CBaseType is base type of items being stored in the multipool
 *	@tparam CDerivedType is the only type of items being stored in the multipool
 *		(derived from CBaseType, might be equal to CBaseType)
 *	@tparam n_pool_page_size is pool page size (per each type being stored)
 *
 *	@todo Try storing pointers in std::vector for faster parallel for_each(),
 *		and try to write fap::for_each() and see what is faster.
 */
template <class CBaseType, class CDerivedType, const size_t n_pool_page_size>
class CMultiPool<CBaseType, CTypelist<CDerivedType, CTypelistEnd>, n_pool_page_size> {
public:
	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		pool_PageSize = n_pool_page_size, /**< @brief size of pool page, in elements */
#ifdef __FLAT_SYSTEM_ALIGNED_MEMORY
		pool_MemoryAlign = 16 /**< @brief memory alignment of data stored in the pool */
#else // __FLAT_SYSTEM_ALIGNED_MEMORY
		pool_MemoryAlign = 0 /**< @brief memory alignment of data stored in the pool */ // or 16 for aligned types and SSE // t_odo
#endif // __FLAT_SYSTEM_ALIGNED_MEMORY
	};

	typedef CBaseType _TyBaseType; /**< @brief base type of items being stored in the multipool */
	typedef CDerivedType _TyDerivedType; /**< @brief the only type of items being stored in the multipool */

	typedef _TyBaseType &_TyBaseRef; /**< @brief base type reference */
	typedef const _TyBaseType &_TyConstBaseRef; /**< @brief base type const */

protected:
	typedef typename CStaticAssert<CEqualType<_TyBaseType,
		_TyDerivedType>::b_result>::BASE_TYPE_MUST_MATCH_THE_ONLY_TYPE_IN_THE_LIST CAssert0; /**< @brief static assertion */
	// this is required so that this class can be made much simpler (and faster)

	forward_allocated_pool<_TyDerivedType, pool_PageSize, pool_MemoryAlign> m_pool; /**< @brief pool for the single type being handled */
	// (static function binding, no space wasted for base type pointers)

public:
	/**
	 *	@brief adds an element to the multipool (at the end)
	 *	@param[in] r_t_payload is the element being inserted
	 *	@return Returns reference to the inserted element
	 *		(the address is guaranteed not to change).
	 *	@note This function throws std::bad_alloc.
	 */
	_TyDerivedType &r_Add_Element(const _TyDerivedType &r_t_payload) // throw(std::bad_alloc)
	{
		m_pool.push_back(r_t_payload);
		return *(m_pool.end() - 1);
	}

	/**
	 *	@copydoc CMultiPool::b_Empty()
	 */
	inline bool b_Empty() const
	{
		return m_pool.empty();
	}

	/**
	 *	@copydoc CMultiPool::n_Size()
	 */
	inline size_t n_Size() const
	{
		return m_pool.size();
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		return m_pool.capacity() * sizeof(_TyDerivedType) +
			m_pool.page_num() * sizeof(_TyDerivedType*) +
			sizeof(CMultiPool<CBaseType, CTypelist<CDerivedType, CTypelistEnd>, n_pool_page_size>);
	}

	/**
	 *	@copydoc CMultiPool::r_At(size_t)
	 */
	template <class CRequestedType>
	inline _TyDerivedType &r_At(size_t n_index)
	{
		typedef typename CStaticAssert<CEqualType<CRequestedType,
			_TyDerivedType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be the derived type, there is no other type

		return m_pool[n_index];
	}

	/**
	 *	@copydoc CMultiPool::r_At(size_t)
	 */
	template <class CRequestedType>
	inline const _TyDerivedType &r_At(size_t n_index) const
	{
		typedef typename CStaticAssert<CEqualType<CRequestedType,
			_TyDerivedType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be the derived type, there is no other type

		return m_pool[n_index];
	}

	/**
	 *	@copydoc CMultiPool::operator [](size_t)
	 */
	inline _TyBaseRef operator [](size_t n_index)
	{
		return m_pool[n_index];
	}

	/**
	 *	@copydoc CMultiPool::operator [](size_t) const
	 */
	inline _TyConstBaseRef operator [](size_t n_index) const
	{
		return m_pool[n_index];
	}

	/**
	 *	@copydoc CMultiPool::For_Each(COp)
	 */
	template <class COp>
	COp For_Each(COp op)
	{
		return std::for_each(m_pool.begin(), m_pool.end(), op);
	}

	/**
	 *	@copydoc CMultiPool::For_Each(COp)
	 */
	template <class COp>
	COp For_Each(COp op) const
	{
		return std::for_each(m_pool.begin(), m_pool.end(), op);
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(COp op, const int n_parallel_thresh = 50)
	{
#ifdef _OPENMP
		_ASSERTE(m_pool.size() <= INT_MAX);
		const int n = int(m_pool.size());
		#pragma omp parallel for default(shared) if(n >= n_parallel_thresh)
		for(int i = 0; i < n; ++ i)
			op(m_pool[i]); // todo - write pool::parallel_for_each that would simplify it's pointer arithmetic
#else // _OPENMP
		std::for_each(m_pool.begin(), m_pool.end(), op);
#endif // _OPENMP
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(COp op, const int n_parallel_thresh = 50) const
	{
#ifdef _OPENMP
		_ASSERTE(m_pool.size() <= INT_MAX);
		const int n = int(m_pool.size());
		#pragma omp parallel for default(shared) if(n >= n_parallel_thresh)
		for(int i = 0; i < n; ++ i)
			op(m_pool[i]); // todo - write pool::parallel_for_each that would simplify it's pointer arithmetic
#else // _OPENMP
		std::for_each(m_pool.begin(), m_pool.end(), op);
#endif // _OPENMP
	}

	/**
	 *	@copydoc CMultiPool::For_Each_WorkShare
	 */
	template <class COp>
	inline COp For_Each_WorkShare(COp op) const // just as parallel but does not start a parallel region, is supposed to be in one
	{
#ifdef _OPENMP
		_ASSERTE(m_pool.size() <= INT_MAX);
		const int n = int(m_pool.size());
		#pragma omp for
		for(int i = 0; i < n; ++ i)
			op(m_pool[i]); // todo - write pool::parallel_for_each that would simplify it's pointer arithmetic
		return op;
#else // _OPENMP
		return std::for_each(m_pool.begin(), m_pool.end(), op);
#endif // _OPENMP
	}

	/**
	 *	@copydoc CMultiPool::For_Each_WorkShare
	 */
	template <class COp>
	inline COp For_Each_WorkShare(COp op) // just as parallel but does not start a parallel region, is supposed to be in one
	{
#ifdef _OPENMP
		_ASSERTE(m_pool.size() <= INT_MAX);
		const int n = int(m_pool.size());
		#pragma omp for
		for(int i = 0; i < n; ++ i)
			op(m_pool[i]); // todo - write pool::parallel_for_each that would simplify it's pointer arithmetic
		return op;
#else // _OPENMP
		return std::for_each(m_pool.begin(), m_pool.end(), op);
#endif // _OPENMP
	}

	/**
	 *	@copydoc CMultiPool::For_Each(size_t,size_t,COp)
	 */
	template <class COp>
	COp For_Each(size_t n_first, size_t n_last, COp op)
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_pool.size());
		return std::for_each(m_pool.begin() + n_first, m_pool.begin() + n_last, op);
	}

	/**
	 *	@copydoc CMultiPool::For_Each(size_t,size_t,COp)
	 */
	template <class COp>
	COp For_Each(size_t n_first, size_t n_last, COp op) const
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_pool.size());
		return std::for_each(m_pool.begin() + n_first, m_pool.begin() + n_last, op);
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(size_t,size_t,COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(size_t n_first, size_t n_last, COp op, const int n_parallel_thresh = 50)
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_pool.size());
#ifdef _OPENMP
		_ASSERTE(n_last <= INT_MAX);
		const int n = int(n_last);
		#pragma omp parallel for default(shared) if(n - int(n_first) >= n_parallel_thresh)
		for(int i = int(n_first); i < n; ++ i)
			op(m_pool[i]); // todo - write pool::parallel_for_each that would simplify it's pointer arithmetic
#else // _OPENMP
		std::for_each(m_pool.begin() + n_first, m_pool.begin() + n_last, op);
#endif // _OPENMP
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(size_t,size_t,COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(size_t n_first, size_t n_last, COp op, const int n_parallel_thresh = 50) const
	{
		_ASSERTE(n_first <= n_last);
		_ASSERTE(n_last <= m_pool.size());
#ifdef _OPENMP
		_ASSERTE(n_last <= INT_MAX);
		const int n = int(n_last);
		#pragma omp parallel for default(shared) if(n - int(n_first) >= n_parallel_thresh)
		for(int i = int(n_first); i < n; ++ i)
			op(m_pool[i]); // todo - write pool::parallel_for_each that would simplify it's pointer arithmetic
#else // _OPENMP
		std::for_each(m_pool.begin() + n_first, m_pool.begin() + n_last, op);
#endif // _OPENMP
	}
};

/**
 *	@brief heterogenous (but static) pool container (specialization
 *		for no type being stored)
 *
 *	@tparam CBaseType is base type of items being stored in the multipool
 *	@tparam CDerivedType is the only type of items being stored in the multipool
 *		(derived from CBaseType, might be equal to CBaseType)
 *	@tparam n_pool_page_size is pool page size (per each type being stored)
 *
 *	@todo Try storing pointers in std::vector for faster parallel for_each(),
 *		and try to write fap::for_each() and see what is faster.
 */
template <class CBaseType, const size_t n_pool_page_size>
class CMultiPool<CBaseType, CTypelistEnd, n_pool_page_size> {
public:
	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		pool_PageSize = n_pool_page_size, /**< @brief size of pool page, in elements */
#ifdef __FLAT_SYSTEM_ALIGNED_MEMORY
		pool_MemoryAlign = 16 /**< @brief memory alignment of data stored in the pool */
#else // __FLAT_SYSTEM_ALIGNED_MEMORY
		pool_MemoryAlign = 0 /**< @brief memory alignment of data stored in the pool */ // or 16 for aligned types and SSE // t_odo
#endif // __FLAT_SYSTEM_ALIGNED_MEMORY
	};

	typedef CBaseType _TyBaseType; /**< @brief base type of items being stored in the multipool */
	typedef CTypelistEnd _TyTypelist; /**< @brief list of types being stored in the multipool (an empty list) */

	typedef _TyBaseType &_TyBaseRef; /**< @brief base type reference */
	typedef const _TyBaseType &_TyConstBaseRef; /**< @brief base type const */

public:
	/**
	 *	@brief adds an element to the multipool (at the end)
	 *	@tparam CGenericPalyoadType is type of the element being inserted
	 *		(must be on the list, otherwise the call will result in compile errors)
	 *	@param[in] t_payload is the element being inserted (unused)
	 *	@return Returns reference to the inserted element
	 *		(the address is guaranteed not to change).
	 *	@note Calling this function always results in compile-time failure.
	 */
	template <class CGenericPalyoadType>
	CGenericPalyoadType &r_Add_Element(CGenericPalyoadType UNUSED(t_payload))
	{
		typedef typename CStaticAssert<CFindTypelistItem<_TyTypelist,
			CGenericPalyoadType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be one of the types in the list (will fail always, _TyTypelist is empty)

		return *((CGenericPalyoadType*)0); // return a null reference
	}

	/**
	 *	@copydoc CMultiPool::b_Empty()
	 */
	inline bool b_Empty() const
	{
		return true;
	}

	/**
	 *	@copydoc CMultiPool::n_Size()
	 */
	inline size_t n_Size() const
	{
		return 0;
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		return sizeof(CMultiPool<CBaseType, CTypelistEnd, n_pool_page_size>);
	}

	/**
	 *	@copydoc CMultiPool::r_At(size_t)
	 *	@note Calling this function always results in compile-time failure.
	 */
	template <class CRequestedType>
	inline CRequestedType &r_At(size_t n_index)
	{
		typedef typename CStaticAssert<CFindTypelistItem<_TyTypelist,
			CRequestedType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be one of the types in the list (will fail always, _TyTypelist is empty)

		return *((CRequestedType*)0); // return a null reference
	}

	/**
	 *	@copydoc CMultiPool::r_At(size_t)
	 *	@note Calling this function always results in compile-time failure.
	 */
	template <class CRequestedType>
	inline const CRequestedType &r_At(size_t n_index) const
	{
		typedef typename CStaticAssert<CFindTypelistItem<_TyTypelist,
			CRequestedType>::b_result>::REQUESTED_TYPE_MISMATCH CAssert0; // if this triggers, either the type you request is not in the system, or you are confusing vertex pool with edge pool
		// the type requested must be one of the types in the list (will fail always, _TyTypelist is empty)

		return *((CRequestedType*)0); // return a null reference
	}

	/**
	 *	@copydoc CMultiPool::operator [](size_t)
	 *	@note Calling this function always results in run-time assertion failure.
	 */
	inline _TyBaseRef operator [](size_t n_index)
	{
		_ASSERTE(0); // this should never be called
		return *((_TyBaseType*)0); // return a null reference
	}

	/**
	 *	@copydoc CMultiPool::operator [](size_t) const
	 *	@note Calling this function always results in run-time assertion failure.
	 */
	inline _TyConstBaseRef operator [](size_t n_index) const
	{
		_ASSERTE(0); // this should never be called
		return *((_TyBaseType*)0); // return a null reference
	}

	/**
	 *	@copydoc CMultiPool::For_Each(COp)
	 */
	template <class COp>
	COp For_Each(COp op)
	{
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each(COp)
	 */
	template <class COp>
	COp For_Each(COp op) const
	{
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(COp op, const int UNUSED(n_parallel_thresh) = 50)
	{
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(COp op, const int UNUSED(n_parallel_thresh) = 50) const
	{
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each(size_t,size_t,COp)
	 */
	template <class COp>
	COp For_Each(size_t UNUSED(n_first), size_t UNUSED(n_last), COp op)
	{
		_ASSERTE(n_first == n_last); // otherwise dereference and OOB access would ensue
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each(size_t,size_t,COp)
	 */
	template <class COp>
	COp For_Each(size_t UNUSED(n_first), size_t UNUSED(n_last), COp op) const
	{
		_ASSERTE(n_first == n_last); // otherwise dereference and OOB access would ensue
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(size_t,size_t,COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(size_t UNUSED(n_first), size_t UNUSED(n_last),
		COp op, const int UNUSED(n_parallel_thresh) = 50)
	{
		_ASSERTE(n_first == n_last); // otherwise dereference and OOB access would ensue
		return op;
	}

	/**
	 *	@copydoc CMultiPool::For_Each_Parallel(size_t,size_t,COp,const int)
	 */
	template <class COp>
	void For_Each_Parallel(size_t UNUSED(n_first), size_t UNUSED(n_last),
		COp op, const int UNUSED(n_parallel_thresh) = 50) const
	{
		_ASSERTE(n_first == n_last); // otherwise dereference and OOB access would ensue
		return op;
	}
};

} // ~multipool

/**
 *	@brief plot quality profile names
 */
namespace plot_quality {

/**
 *	@brief plot quality profile names
 */
enum EQualityProfile {
	plot_Draft, /**< @brief fast plots for debugging */
	plot_Printing, /**< @brief nice plots for papers or web */
	plot_Printing_LandmarkTicksOnly, /**< @brief same as plot_Printing but only landmarks have ticks (not poses) */
	plot_Printing_NoTicks /**< @brief same as plot_Printing but there are no ticks for poses or landmarks */
};

} // ~plot_quality

/**
 *	@brief namespace with plotting functions
 */
namespace plot {

typedef ::plot_quality::EQualityProfile EQualityProfile; /**< @copydoc ::plot_quality::EQualityProfile */

/**
 *	@brief tick types
 */
enum {
	tick_Cross_Subpixel = 0, /**< @brief cross ticks without antialiasing */
	tick_Cross, /**< @brief antialiased cross ticks */
	tick_TiltedCross, /**< @brief tilted cross ticks */
	tick_Circle, /**< @brief circle ticks */
	tick_Disc, /**< @brief disc ticks */
	tick_Square, /**< @brief stroke square ticks */
	tick_FilledSquare /**< @brief filled square ticks */
};

/**
 *	@brief plotting functions
 */
class CPlotUtils {
public:
	/**
	 *	@brief calculates resolution of a bitmap, given the vertex bounding box
	 *
	 *	@param[out] r_n_width is filled with image width, in pixels
	 *	@param[out] r_n_height is filled with image height, in pixels
	 *	@param[out] r_f_scale is filled with scale ratio from vertex space to raster coordinates
	 *	@param[in] r_v_min minimum coordinates of the vertices to be plotted
	 *	@param[in] r_v_max maximum coordinates of the vertices to be plotted
	 *	@param[in] n_smaller_side_resoluion is the resolution of the shorter side of the image. in pixels
	 *	@param[in] n_max_resolution is the maximal resolution limit for either dimension, in pixels
	 *		(in case the aspect ratio is too high, the longer side is set to n_max_resolution
	 *		and the shorter side is scaled appropriately)
	 */
	static void Calc_Resolution(int &r_n_width, int &r_n_height, double &r_f_scale,
		const Eigen::Vector2d &r_v_min, const Eigen::Vector2d &r_v_max,
		int n_smaller_side_resoluion = 1024, int n_max_resolution = 8192)
	{
		Eigen::Vector2d v_size = r_v_max - r_v_min;
		double f_short_side = std::min(v_size(0), v_size(1));
		double f_long_side = std::max(v_size(0), v_size(1));
		r_f_scale = n_smaller_side_resoluion / f_short_side;
		if(f_long_side * r_f_scale > n_max_resolution)
			r_f_scale = n_max_resolution / f_long_side;

		r_n_width = std::max(1, int(v_size(0) * r_f_scale));
		r_n_height = std::max(1, int(v_size(1) * r_f_scale));
		// calculate image size
	}

	/**
	 *	@brief calculates a direct transformation from vertex space to raster coordinates
	 *
	 *	This function takes a transformation matrix for projecting the vertices into
	 *	the plane and modifies it so that it is projecting the vertices directly into
	 *	the raster coordinates. Assuming that we have:
	 *
	 *	@code
	 *	Eigen::Vector2d v_min, v_max;
	 *	Eigen::Matrix<double, 2, 3> t_transform = ...;
	 *	int n_width, n_height;
	 *	double f_scale;
	 *	system.Vertex_BoundingBox(v_min, v_max, t_transform);
	 *	Draw_CalcResolution(n_width, n_height, f_scale, v_min, v_max, 1024);
	 *	const int n_padding = 32; // px
	 *	@endcode
	 *
	 *	It is not necessary for each vertex to:
	 *
	 *	@code
	 *	for(size_t i = 0; i < n_vertex_num; ++ i) {
	 *		Eigen::Vector2d v_planar = t_transform * system.v_VertexState(i);
	 *		Eigen::Vector2d v_raster = (v_planar - v_min) * f_scale;
	 *		v_raster(0) += n_padding;
	 *		v_raster(1) = n_height + n_padding - v_raster(1); // raster coords have origin at top left corner
	 *
	 *		// [draw]
	 *	}
	 *	@endcode
	 *
	 *	But rather it is possible to modify the transformation as follows:
	 *
	 *	@code
	 *	Eigen::Vector2d v_offset;
	 *	Draw_CalcRasterTransform(t_transform, v_offset, n_width, n_height,
	 *		n_padding, f_scale, v_min, t_transform);
	 *	for(size_t i = 0; i < n_vertex_num; ++ i) {
	 *		Eigen::Vector2d v_raster = t_transform * system.v_VertexState(i) + v_offset;
	 *
	 *		// [draw]
	 *	}
	 *	@endcode
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[out] r_t_raster_transform is filled with transformation from vertex
	 *		to raster coordinates
	 *	@param[out] r_v_offset is filled with additive offset, in raster coordinates
	 *	@param[in] n_width is raster width, in pixels
	 *	@param[in] n_height is raster height, in pixels
	 *	@param[in] n_padding is raster padding, in pixels (not part of width or height)
	 *	@param[in] f_scale is scale factor from vertex space to raster space
	 *	@param[in] r_v_min is minimum corner of the vertex bounding box
	 *	@param[in] r_t_transform is the same vertex transformation matrix as in
	 *		the call to \ref Vertex_BoundingBox() (can point to the same object
	 *		as r_t_raster_transform)
	 */
	template <class Derived0, class Derived1>
	static void Calc_RasterTransform(Eigen::MatrixBase<Derived0> &r_t_raster_transform,
		Eigen::Vector2d &r_v_offset, int n_width, int n_height, int n_padding, double f_scale,
		const Eigen::Vector2d &r_v_min, const Eigen::MatrixBase<Derived1> &r_t_transform)
	{
		_ASSERTE(r_t_transform.rows() == 2 && r_t_transform.rows() == r_t_raster_transform.rows() &&
			r_t_transform.cols() == r_t_raster_transform.cols());
		r_v_offset = Eigen::Vector2d(n_padding, n_height + n_padding) -
			(r_v_min.array() * Eigen::Array2d(1.0, -1.0)).matrix() * f_scale;
		r_t_raster_transform.row(0) = r_t_transform.row(0) * f_scale;
		r_t_raster_transform.row(1) = r_t_transform.row(1) * -f_scale; // the origin is at the top left corner, flip y
		// modify the transform to yield raster coordinates
	}

	/**
	 *	@brief draws a vertex tick
	 *
	 *	@param[in,out] p_bitmap is pointer to the bitmap image to plot into
	 *	@param[in] f_x is horizontal vertex position, in pixels
	 *	@param[in] f_y is vertical vertex position, in pixels
	 *	@param[in] n_color is tick color
	 *	@param[in] n_tick_type is tick type (0 for simple cross, 1 for antialiased cross,
	 *		2 for tilted cross, 3 for circle, 4 for disc, 5 for square, 6 for filled square)
	 *	@param[in] f_tick_size is tick size in pixels
	 *	@param[in] f_line_width is tick line width
	 */
	static void Draw_Tick(TBmp *p_bitmap, float f_x, float f_y, uint32_t n_color,
		int n_tick_type = 0, float f_tick_size = 2, float f_line_width = 1)
	{
		switch(n_tick_type) {
		case tick_Cross_Subpixel:
		default:
			p_bitmap->DrawLine_SP(f_x - f_tick_size, f_y,
				f_x + f_tick_size, f_y, n_color, int(f_line_width) | 1);
			p_bitmap->DrawLine_SP(f_x, f_y - f_tick_size,
				f_x, f_y + f_tick_size, n_color, int(f_line_width) | 1);
			// a simple cross ticks, no antialiasing
			break;
		case tick_Cross:
			p_bitmap->DrawLine_AA(f_x - f_tick_size, f_y,
				f_x + f_tick_size, f_y, n_color, f_line_width);
			p_bitmap->DrawLine_AA(f_x, f_y - f_tick_size,
				f_x, f_y + f_tick_size, n_color, f_line_width);
			// antialiased cross ticks
			break;
		case tick_TiltedCross:
			f_tick_size *= .707f; // do not make it larger, avoid clipping at the borders
			p_bitmap->DrawLine_AA(f_x - f_tick_size, f_y - f_tick_size,
				f_x + f_tick_size, f_y + f_tick_size, n_color, f_line_width, 1);
			p_bitmap->DrawLine_AA(f_x + f_tick_size, f_y - f_tick_size,
				f_x - f_tick_size, f_y + f_tick_size, n_color, f_line_width, 1);
			// pi/4 rotated antialiased cross ticks
			break;
		case tick_Circle:
			f_tick_size -= f_line_width * .5f;
			p_bitmap->DrawCircle_AA(f_x + .5f, f_y + .5f, f_tick_size, n_color, f_line_width);
			// antialiased circles
			break;
		case tick_Disc:
			p_bitmap->FillCircle_AA(f_x + .5f, f_y + .5f, f_tick_size, n_color);
			// antialiased discs
			break;
		case tick_Square:
			f_tick_size -= f_line_width * .5f;
			p_bitmap->DrawRect(int(f_x - f_tick_size + .5f), int(f_y - f_tick_size + .5f),
				int(f_x + f_tick_size + .5f), int(f_y + f_tick_size + .5f), n_color,
				int(f_line_width + .5f));
			// simple stroke rectangles
			break;
		case tick_FilledSquare:
			p_bitmap->FillRect(int(f_x - f_tick_size + .5f), int(f_y - f_tick_size + .5f),
				int(f_x + f_tick_size + .5f), int(f_y + f_tick_size + .5f), n_color);
			// simple filled rectangles
			break;
		}
	}
};

};

/**
 *	@brief optimization system, customized for work with different primitives
 *
 *	@tparam CBaseVertexType is base (optimized) vertex type (all vertex types must be derived from it)
 *	@tparam CVertexTypelist is list of (optimized) vertex types permitted in the system
 *	@tparam CBaseEdgeType is base edge type (all edge types must be derived from it)
 *	@tparam CEdgeTypelist is list of edge types permitted in the system
 *	@tparam CUnaryFactorFactory is class, responsible for the initialization of unary factor
 *	@tparam CBaseConstVertexType is base type for the const vertices (default void)
 *	@tparam CConstVertexTypelist is list of constant vertex types permitted in the system
 *	@tparam n_pool_page_size is edge or vertex pool page size, in elements
 */
template <class CBaseVertexType, class CVertexTypelist, /*template <class> class CVertexTypeTraits,*/ // unused
	class CBaseEdgeType, class CEdgeTypelist, class CUnaryFactorFactory = CBasicUnaryFactorFactory,
	class CBaseConstVertexType = int, class CConstVertexTypelist = CTypelistEnd,
	const int n_pool_page_size = 1024>
class CFlatSystem {
protected:
	/**
	 *	@brief extracts sizes of the jacobian matrices from a generic n-ary edge
	 *	@tparam CEdgeType is an edge type name
	 */
	template <class CEdgeType>
	class CEdgeTypeToJacobianSizes {
	protected:
		typedef typename CTypelistIntersection<typename CEdgeType::_TyVertices,
			CVertexTypelist>::_TyResult VertexList; /**< @brief list of edge vertices which are optimized */

		/**
		 *	@brief gets jacobian dimension for a given vertex
		 *	@tparam CVertex is vertex type
		 */
		template <class CVertex>
		class CGetDimension {
		public:
			typedef fbs_ut::CCTSize2D<CEdgeType::n_measurement_dimension,
				CVertex::n_dimension> _TyResult; /**< @brief jacobian block size */
		};

	public:
		typedef typename CTransformTypelist<VertexList, CGetDimension>::_TyResult _TyResult; /**< @brief list of sizes of vertex blocks */
	};

	/**
	 *	@brief extracts sizes of the unary factor matrices from a generic n-ary edge
	 *	@tparam CEdgeType is an edge type name
	 */
	template <class CEdgeType>
	class CEdgeTypeToUnaryFactorSizes {
	protected:
		typedef typename CTypelistIntersection<typename CEdgeType::_TyVertices,
			CVertexTypelist>::_TyResult VertexList; /**< @brief list of edge vertices which are optimized */

		/**
		 *	@brief gets unary factor dimension for a given vertex
		 *	@tparam CVertex is vertex type
		 */
		template <class CVertex>
		class CGetDimension {
		public:
			typedef fbs_ut::CCTSize2D<CVertex::n_dimension, CVertex::n_dimension> _TyResult; /**< @brief UF block size */
		};

	public:
		typedef typename CTransformTypelist<VertexList, CGetDimension>::_TyResult _TyResult; /**< @brief list of sizes of vertex blocks */
	};

	typedef typename CTransformTypelist<CEdgeTypelist,
		CEdgeTypeToJacobianSizes>::_TyResult TJacobianLists; /**< @brief list of lists of jacobian matrix block types in all edges (some of the blocks in A) */
	typedef typename CTransformTypelist<CEdgeTypelist,
		CEdgeTypeToUnaryFactorSizes>::_TyResult TUnaryFactorLists; /**< @brief list of lists of unary factor matrix block types in all edges (some of the blocks in A) */
	// note that UFs don't contribute new dimensions to lambda.

	typedef typename CUniqueTypelist<typename
		CConcatListOfTypelists<typename CConcatTypelist<TJacobianLists,
		TUnaryFactorLists>::_TyResult>::_TyResult>::_TyResult TJacobianMatrixBlockList; /**< @brief list of jacobian and UF matrix block sizes as fbs_ut::CCTSize2D (blocks in A) */

#if 0
	typedef typename fbs_ut::CBlockSizesAfterPreMultiplyWithSelfTranspose<
		TJacobianMatrixBlockList>::_TyResult THessianMatrixBlockList; /**< @brief list of hessian matrix block sizes as fbs_ut::CCTSize2D (blocks in Lambda and L) */
	// t_odo - this is not good in BA! there are 6x2x3 blocks in A, but only 6x6, 6x3, 3x6 or 3x3 in lambda. this needs to be fixed. now it is fixed (for BA, only 6x6, 6x3, 3x6 and 3x3 are used).
	// t_odo - this is still not good enough, the hessian matrix block sizes cannot be inferred from
	//		jacobian matrix sizes but only from the edges (there might be 2D measurements linking 6x6 and
	//		3x3, leading to 6x6, 6x3, 3x6, 3x3 and different 2D measurements linking 2x2 with 3x3, then
	//		that adds only 2x2, 3x2 and 2x3 but not 6x2 or 2x6 as the above code would)
	//		will need to transform each edge type to a list of jacobian sizes
#else // 0
	typedef typename CTransformTypelist<typename CConcatTypelist<TJacobianLists,
		TUnaryFactorLists>::_TyResult,
		fbs_ut::CBlockSizesAfterPreMultiplyWithSelfTranspose>::_TyResult THessianLists; /**< @brief list of lists of hessian matrix block types in all edges (blocks in Lambda and L) */
	// do the ATA on the jacobian lists of each edge separately, this avoids introducing impossible block sizes

	typedef typename CUniqueTypelist<typename
		CConcatListOfTypelists<THessianLists>::_TyResult>::_TyResult THessianMatrixBlockList; /**< @brief list of hessian matrix block sizes as fbs_ut::CCTSize2D (blocks in Lambda and L) */
	// then concatenate the list of lists to a flat list
#endif // 0

public:
	typedef size_t _TyId; /**< @brief data type used as the index in the flat structures */

	/**
	 *	@brief configuration, stored as enum
	 */
	enum {
		pool_PageSize = n_pool_page_size, /**< @brief data storage page size */
		have_ConstVertices = !CTypelistEmpty<CConstVertexTypelist>::b_result, /**< @brief allows / disallows const vertices */
		null_UnaryFactor = CEqualType<CNullUnaryFactorFactory, CUnaryFactorFactory>::b_result /**< @brief explicit unary factor flag */
	};

	typedef CBaseVertexType _TyBaseVertex; /**< @brief the base type of optimized vertices */
	typedef CVertexTypelist _TyVertexTypelist; /**< @brief list of optimized vertex types */
	typedef CBaseEdgeType _TyBaseEdge; /**< @brief the base type of edges */
	typedef CEdgeTypelist _TyEdgeTypelist; /**< @brief list of edge types */
	typedef CBaseConstVertexType _TyBaseConstVertex; /**< @brief the base type of const vertices */
	typedef CConstVertexTypelist _TyConstVertexTypelist; /**< @brief list of const vertex types */

#ifdef __FLAT_SYSTEM_USE_SIZE2D_FOR_BLOCK_SIZE
	typedef TJacobianMatrixBlockList _TyJacobianMatrixBlockList; /**< @brief list of jacobian and UF matrix block sizes (blocks in A) */
	typedef THessianMatrixBlockList _TyHessianMatrixBlockList; /**< @brief list of hessian block sizes (blocks in Lambda and L) */
	// note that these are not Eigen matrix types, but rather fbs_ut::CCTsize2D, which are easier to resolve for the compiler
#else // __FLAT_SYSTEM_USE_SIZE2D_FOR_BLOCK_SIZE
	typedef typename CTransformTypelist<TJacobianMatrixBlockList,
		fbs_ut::CDimensionToEigen>::_TyResult _TyJacobianMatrixBlockList; /**< @brief list of jacobian and UF matrix block types (blocks in A) */
	typedef typename CTransformTypelist<THessianMatrixBlockList,
		fbs_ut::CDimensionToEigen>::_TyResult _TyHessianMatrixBlockList; /**< @brief list of hessian block types (blocks in Lambda and L) */
#endif // __FLAT_SYSTEM_USE_SIZE2D_FOR_BLOCK_SIZE

	typedef multipool::CMultiPool<_TyBaseVertex, _TyVertexTypelist, pool_PageSize> _TyVertexMultiPool; /**< @brief vertex multipool type */
	//typedef typename CChooseType<multipool::CMultiPool<_TyBaseConstVertex, _TyConstVertexTypelist,
	//	pool_PageSize>, TVoidPool<_TyBaseVertex>, have_ConstVertices>::_TyResult _TyConstVertexMultiPool; // unnecessarily complicated
	typedef multipool::CMultiPool<_TyBaseConstVertex,
		_TyConstVertexTypelist, pool_PageSize> _TyConstVertexMultiPool; /**< @brief const vertex multipool type (same as vertex multipool if enabled, otherwise a void type) */
	typedef multipool::CMultiPool<_TyBaseEdge, _TyEdgeTypelist, pool_PageSize> _TyEdgeMultiPool; /**< @brief edge multipool type */

	typedef typename _TyVertexMultiPool::_TyBaseRef _TyVertexRef; /**< @brief reference to base vertex type */
	typedef typename _TyVertexMultiPool::_TyConstBaseRef _TyConstVertexRef; /**< @brief const reference to base vertex type */
	typedef typename _TyEdgeMultiPool::_TyBaseRef _TyEdgeRef; /**< @brief reference to base edge type */
	typedef typename _TyEdgeMultiPool::_TyConstBaseRef _TyConstEdgeRef; /**< @brief const reference to base edge type */

	typedef typename CUniqueTypelist<typename CConcatTypelist<_TyVertexTypelist,
		_TyConstVertexTypelist>::_TyResult>::_TyResult _TyAllVertexTypelist; /**< @brief types of both constant and optimized vertices */

	typedef typename CUniqueTypelist<typename CTransformTypelist<_TyVertexTypelist,
		fbs_ut::CGetVertexVectorType>::_TyResult>::_TyResult _TyVertexStateVectorTypes; /**< @brief types of (unique, optimized) vertex state vectors */
	typedef typename CUniqueTypelist<typename CTransformTypelist<_TyConstVertexTypelist,
		fbs_ut::CGetVertexVectorType>::_TyResult>::_TyResult _TyConstVertexStateVectorTypes; /**< @brief types of (unique) constant vertex state vectors */
	typedef typename CUniqueTypelist<typename CConcatTypelist<_TyVertexStateVectorTypes,
		_TyConstVertexStateVectorTypes>::_TyResult>::_TyResult _TyAllVertexStateVectorTypes; /**< @brief types of (unique) vertex state vectors of both optimized and constant vertices */

	struct NO_VERTICES_OF_THIS_TYPE;
	typedef CTypelist<NO_VERTICES_OF_THIS_TYPE, CTypelistEnd> _TyDummyList;

	typedef std::pair<void, int> NamelessReferenceToVoid; // g++ does not seem to complain // todo - remove me

	typedef typename CChooseType<Eigen::VectorXd,
		typename CConcatTypelist<_TyVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead,
		CTypelistLength<_TyVertexStateVectorTypes>::n_result !=
		1>::_TyResult _TyVertexState; /**< @brief vector type that can hold (optimized) vertex state @note In case all the optimized vertices in the system have the same dimension, then this is a fixed size vector. */
	typedef typename CChooseType<Eigen::VectorXd,
		typename CConcatTypelist<_TyConstVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead,
		CTypelistLength<_TyConstVertexStateVectorTypes>::n_result !=
		1>::_TyResult _TyConstVertexState; /**< @brief vector type that can hold constant vertex state @note In case all the constant vertices in the system have the same dimension, then this is a fixed size vector. */
	typedef typename CChooseType<Eigen::VectorXd,
		typename CConcatTypelist<_TyAllVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead,
		CTypelistLength<_TyAllVertexStateVectorTypes>::n_result !=
		1>::_TyResult _TyAnyVertexState; /**< @brief vector type that can hold optimized or constant vertex state @note In case all the vertices in the system have the same dimension, then this is a fixed size vector. */

	typedef /*typename CChooseType<*/Eigen::Map<_TyVertexState>/*,
		typename CConcatTypelist<_TyVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead&,
		CTypelistLength<_TyVertexStateVectorTypes>::n_result !=
		1>::_TyResult*/ _TyVertexStateRef; /**< @brief type of reference to (optimized) vertex state */
	typedef /*typename CChooseType<*/Eigen::Map<_TyConstVertexState>/*,
		typename CConcatTypelist<_TyConstVertexStateVectorTypes,
		_TyDummyList>::_TyResult::_TyHead&,
		CTypelistLength<_TyConstVertexStateVectorTypes>::n_result !=
		1>::_TyResult*/ _TyConstVertexStateRef; /**< @brief type of reference to constant vertex state */
	typedef /*typename CChooseType<*/Eigen::Map<_TyAnyVertexState>/*,
		typename CConcatTypelist<_TyAllVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead&,
		CTypelistLength<_TyAllVertexStateVectorTypes>::n_result !=
		1>::_TyResult*/ _TyAnyVertexStateRef; /**< @brief type of reference to optimized or constant vertex state */

	typedef /*typename CChooseType<*/Eigen::Map<const _TyVertexState>/*,
		const typename CConcatTypelist<_TyVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead&,
		CTypelistLength<_TyVertexStateVectorTypes>::n_result !=
		1>::_TyResult*/ _TyVertexStateConstRef; /**< @brief type of constant reference to (optimized) vertex state */
	typedef /*typename CChooseType<*/Eigen::Map<const _TyConstVertexState>/*,
		const typename CConcatTypelist<_TyConstVertexStateVectorTypes,
		_TyDummyList>::_TyResult::_TyHead&,
		CTypelistLength<_TyConstVertexStateVectorTypes>::n_result !=
		1>::_TyResult*/ _TyConstVertexStateConstRef; /**< @brief type of constant reference to constant vertex state */
	typedef /*typename CChooseType<*/Eigen::Map<const _TyAnyVertexState>/*,
		const typename CConcatTypelist<_TyAllVertexStateVectorTypes,
		_TyDummyList >::_TyResult::_TyHead&,
		CTypelistLength<_TyAllVertexStateVectorTypes>::n_result !=
		1>::_TyResult*/ _TyAnyVertexStateConstRef; /**< @brief type of constant reference to optimized or constant vertex state */
	// unfortunately all the refs must be maps, even if there is only a single vertex
	// because v_State() returns a map by default. we could specialize the hell out of
	// v_VertexState() but there should be no performance loss in using Eigen::Map rather
	// than a reference to a vector. and if the user-store vertices get implemented in
	// the future, there maz not be any vectors in the first place. this is ok.

protected:
	//_TyVertexLookup m_vertex_lookup; /**< @brief lookup table for vertices (by id; only used if have_ConstVertices is set) */
	_TyVertexMultiPool m_vertex_pool; /**< @brief vertex multipool */
	_TyEdgeMultiPool m_edge_pool; /**< @brief edge multipool */
	size_t m_n_vertex_element_num; /**< @brief sum of the numbers of all vertex elements, (also the size of permutation vector) */
	size_t m_n_edge_element_num; /**< @brief sum of all permutation vector components + unary factor rank (also size of the A matrix) */

	_TyConstVertexMultiPool m_const_vertex_pool; /**< @brief const vertex multipool (only used if have_ConstVertices is set) */

	CUnaryFactorFactory m_unary_factor_factory; /**< @brief unary factor initializer */
	size_t m_n_unary_factor_order; /**< @brief the row of the Jacobian matrix where the unary factor should be placed */
	Eigen::MatrixXd m_t_unary_factor; /**< @brief the unary factor matrix */
	Eigen::VectorXd m_v_unary_error; /**< @brief the error vector associated with the first vertex */

public:
	/**
	 *	@brief default constructor; has no effect
	 *	@param[in] unary_factor_factory is unary factor initializer
	 */
	inline CFlatSystem(CUnaryFactorFactory unary_factor_factory = CUnaryFactorFactory())
		:m_n_vertex_element_num(0), m_n_edge_element_num(0),
		m_unary_factor_factory(unary_factor_factory), m_n_unary_factor_order(-1)
	{
#ifndef __BASE_TYPES_ALLOW_CONST_VERTICES
		if(have_ConstVertices)
			fprintf(stderr, "warning: __BASE_TYPES_ALLOW_CONST_VERTICES not defined\n");
#endif // !__BASE_TYPES_ALLOW_CONST_VERTICES
		// be tolerant here, generate assertions only once a const vertex is inserted
	}

	/**
	 *	@brief calculates the size of this object in memory
	 *	@return Returns the size of this object (and of all associated
	 *		arrays or buffers) in memory, in bytes.
	 */
	size_t n_Allocation_Size() const
	{
		return sizeof(CFlatSystem<CBaseVertexType, CVertexTypelist, CBaseEdgeType,
			CEdgeTypelist, CUnaryFactorFactory, CBaseConstVertexType,
			CConstVertexTypelist, n_pool_page_size>) +
			m_vertex_pool.n_Allocation_Size() - sizeof(m_vertex_pool) +
			m_edge_pool.n_Allocation_Size() - sizeof(m_edge_pool) +
			m_const_vertex_pool.n_Allocation_Size() - sizeof(m_const_vertex_pool) +
			//m_vertex_lookup.capacity() * sizeof(_TyBaseVertex*) +
			(m_v_unary_error.size() + m_t_unary_factor.size()) * sizeof(double);
	}

	/**
	 *	@brief gets order of the unary factor
	 *	@return Returns the order of the unary factor (the position in the jacobian
	 *		matrix where the unary factor should be inserted).
	 */
	size_t n_Unary_Factor_Order() const
	{
		return m_n_unary_factor_order;
	}

	/**
	 *	@brief gets the unary factor matrix
	 *	@return Returns reference to the unary factor matrix.
	 */
	Eigen::MatrixXd &r_t_Unary_Factor()
	{
		return m_t_unary_factor;
	}

	/**
	 *	@brief gets the unary factor matrix
	 *	@return Returns const reference to the unary factor matrix.
	 */
	const Eigen::MatrixXd &r_t_Unary_Factor() const
	{
		return m_t_unary_factor;
	}

	/**
	 *	@brief gets the error vector, associated with unary factor
	 *	@return Returns reference to the error vector, associated with unary factor.
	 */
	Eigen::VectorXd &r_v_Unary_Error()
	{
		return m_v_unary_error;
	}

	/**
	 *	@brief gets the error vector, associated with unary factor
	 *	@return Returns const reference to the error vector, associated with unary factor.
	 */
	const Eigen::VectorXd &r_v_Unary_Error() const
	{
		return m_v_unary_error;
	}

	/**
	 *	@brief gets number of edges in the system
	 *	@return Returns number of edges in the system.
	 */
	inline size_t n_Edge_Num() const
	{
		return m_edge_pool.n_Size();
	}

	/**
	 *	@brief gets number of vertices in the system
	 *	@return Returns number of vertices in the system.
	 *	@note This is the number of the optimized vertices, const vertices are not included.
	 */
	inline size_t n_Vertex_Num() const
	{
		return m_vertex_pool.n_Size();
	}

	/**
	 *	@brief gets number of const vertices in the system
	 *	@return Returns number of const vertices in the system.
	 */
	inline size_t n_ConstVertex_Num() const
	{
		return m_const_vertex_pool.n_Size();
	}

	/**
	 *	@brief gets the number of all permutation vector components + unary factor rank
	 *	@return Returns the sum of all permutation vector components + unary factor rank (also the size of the A matrix).
	 */
	inline size_t n_EdgeElement_Num() const
	{
		return m_n_edge_element_num;
	}

	/**
	 *	@brief gets the size of the state vector
	 *	@return Returns the sum of the numbers of all vertex elements, (also the size of the state vector).
	 *	@note This is sum of numbers of elements of the optimized vertices,
	 *		const vertices are not included.
	 */
	inline size_t n_VertexElement_Num() const
	{
		return m_n_vertex_element_num;
	}

	/**
	 *	@brief gets reference to edge multipool
	 *	@return Returns reference to edge multipool.
	 */
	inline _TyEdgeMultiPool &r_Edge_Pool()
	{
		return m_edge_pool;
	}

	/**
	 *	@brief gets reference to edge multipool
	 *	@return Returns const reference to edge multipool.
	 */
	inline const _TyEdgeMultiPool &r_Edge_Pool() const
	{
		return m_edge_pool;
	}

	/**
	 *	@brief determines whether all the vertices are covered
	 *	@return Returns true if there is a measurement associated
	 *		with each optimized vertex, otherwise returns false.
	 */
	bool b_AllVertices_Covered() const // throw(std::bad_alloc)
	{
		const size_t n_optimized_vertex_num = m_vertex_pool.n_Size();
		std::vector<bool> cover(n_optimized_vertex_num, false); // throws

		for(size_t i = 0, n = m_edge_pool.n_Size(); i < n; ++ i) {
			typename _TyEdgeMultiPool::_TyConstBaseRef r_edge = m_edge_pool[i];
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j) {
				size_t n_vertex = r_edge.n_Vertex_Id(j);
				_ASSERTE(have_ConstVertices || n_vertex < n_optimized_vertex_num);
				if(!have_ConstVertices  || n_vertex < n_optimized_vertex_num)
					cover[n_vertex] = true;
			}
		}
		// accumulate vertex cover

		return std::find(cover.begin(), cover.end(), false) == cover.end();
		// see if all the vertices are covered
	}

	/**
	 *	@brief counts the number of connected components of the graph
	 *	@param[in] b_use_sets is explicit vertex set flag; if set, vertex sets will
	 *		be maintained in addition to vertex labels; this makes relabelling joined
	 *		sets faster (may speed up on large graphs with unordered edges)
	 *	@return Returns the number of disjunct groups of vertices, connected
	 *		by edges. Disconnected vertices are not counted.
	 */
	size_t n_ConnectedComponent_Num(bool b_use_sets = true) const // throw(std::bad_alloc)
	{
		const size_t n_optimized_vertex_num = m_vertex_pool.n_Size();
		std::vector<size_t> comp(n_optimized_vertex_num, 0); // throws
		size_t n_last_component_label = 0;

		std::map<size_t, std::vector<size_t> > comp_vertices;
		// will have up to n sets in total n vertices so in theory the memory is O(n) but the STL
		// preallocation might increase it quite a bit if there are many disconnected components

		for(size_t i = 0, n = m_edge_pool.n_Size(); i < n; ++ i) {
			typename _TyEdgeMultiPool::_TyConstBaseRef r_edge = m_edge_pool[i];
			size_t n_vertex_comp = 0, n_unlabeled_num = 0;
			for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j) {
				size_t n_vertex = r_edge.n_Vertex_Id(j);
				_ASSERTE(have_ConstVertices || n_vertex < n_optimized_vertex_num);
				if(!have_ConstVertices  || n_vertex < n_optimized_vertex_num) {
					size_t n_cur_comp = comp[n_vertex];
					if(n_cur_comp) { // a vertex is already colored
						if(n_vertex_comp && n_cur_comp != n_vertex_comp) { // two or more groups of vertices labeled as different components are connected
							if(b_use_sets) {
								std::vector<size_t> &r_vertex_set = comp_vertices[n_vertex_comp],
									&r_cur_set = comp_vertices[n_cur_comp];
								// get the two sets
								std::vector<size_t> &r_src_set = (r_vertex_set.size() < r_cur_set.size())?
									r_vertex_set : r_cur_set, &r_tar_set = (r_vertex_set.size() < r_cur_set.size())?
									r_cur_set : r_vertex_set;

								if(r_vertex_set.size() < r_cur_set.size())
									std::swap(n_vertex_comp, n_cur_comp); // join the smaller to the larger set, it is cheaper that way (less relabeling to be done)

								for(size_t k = 0, o = r_src_set.size(); k < o; ++ k)
									comp[r_src_set[k]] = n_vertex_comp; // !!
								_ASSERTE(comp[n_vertex] == n_vertex_comp); // was replaced above; make sure this vertex was part of the set
								r_tar_set.insert(r_tar_set.end(), r_src_set.begin(), r_src_set.end());
								comp_vertices.erase(n_cur_comp); // invalidates r_cur_set, memory is potentially freed
								// use the component vertex list to quickly re-label the vertices
							} else {
								std::replace(comp.begin(), comp.end(), n_cur_comp, n_vertex_comp);
								comp[n_vertex] = n_vertex_comp; // !!
							}
							// replace all occurences of n_cur_comp with n_vertex_comp
							// it would be better to take advantage of connectivity and follow
							// the edges maybe? or maybe make a list of lists and just merge the
							// lists (an orthogonal data structure to the labels)
						} else
							n_vertex_comp = n_cur_comp;
					} else
						++ n_unlabeled_num;
				}
			}
			// go through the edge vertices, record the component of one of the vertices and record any unlabeled vertices

			if(n_unlabeled_num) {
				if(!n_vertex_comp)
					n_vertex_comp = ++ n_last_component_label;
				// decide on what the label should be

				std::vector<size_t> *p_vertex_set; // avoid repeated std::map lookup in the loop below
				if(b_use_sets)
					p_vertex_set = &comp_vertices[n_vertex_comp];
				// make sure the set is allocated

				for(size_t j = 0, m = r_edge.n_Vertex_Num(); j < m; ++ j) {
					size_t n_vertex = r_edge.n_Vertex_Id(j);
					_ASSERTE(have_ConstVertices || n_vertex < n_optimized_vertex_num);
					if(!have_ConstVertices  || n_vertex < n_optimized_vertex_num) {
						_ASSERTE(!comp[n_vertex] || comp[n_vertex] == n_vertex_comp);
						// either no label or the component we're labelling

						if(b_use_sets && comp[n_vertex] != n_vertex_comp)
							p_vertex_set->push_back(n_vertex);
						comp[n_vertex] = n_vertex_comp;
						// assign a (potentially) new label
					}
				}
				// go through the vertices again and label them (cant do that in the first pass, the label may change)
			}
		}
		// go through the edges and label the so far disconnected
		// vertices, merge groups of connected vertices
		// note that the merge is slow

		// t_odo - optimize the merge, make this more reusable (return the components, then make a function that merely counts them)
		// note that there is probably a component-adjacent ordering function somewhere that has code to non-destructively reassign the labels

		if(b_use_sets) {
			return comp_vertices.size();
			// much simpler and faster
		} else {
			std::sort(comp.begin(), comp.end());
			comp.erase(std::unique(comp.begin(), comp.end()), comp.end());
			// only keep unique component ids
			// note that this is slow but needs no extra storage (well, merge sort ...)

			if(!comp.empty() && comp.front() == 0)
				return comp.size() - 1; // the "component" of disconnected vertices does not count
			return comp.size(); // all vertices are connected, all components count
		}
	}

	/**
	 *	@brief gets reference to vertex multipool
	 *	@return Returns reference to vertex multipool.
	 *	@note This pool contains only the optimized vertices, not const vertices.
	 */
	inline _TyVertexMultiPool &r_Vertex_Pool()
	{
		return m_vertex_pool;
	}

	/**
	 *	@brief gets reference to vertex multipool
	 *	@return Returns const reference to vertex multipool.
	 *	@note This pool contains only the optimized vertices, not const vertices.
	 */
	inline const _TyVertexMultiPool &r_Vertex_Pool() const
	{
		return m_vertex_pool;
	}

	/**
	 *	@brief gets reference to const vertex multipool
	 *	@return Returns reference to const vertex multipool.
	 *	@note This is only available if have_ConstVertices is set.
	 */
	inline _TyConstVertexMultiPool &r_ConstVertex_Pool()
	{
		return m_const_vertex_pool;
	}

	/**
	 *	@brief gets reference to const vertex multipool
	 *	@return Returns const reference to const vertex multipool.
	 *	@note This is only available if have_ConstVertices is set.
	 */
	inline const _TyConstVertexMultiPool &r_ConstVertex_Pool() const
	{
		return m_const_vertex_pool;
	}

	/**
	 *	@brief finds vertex by id or creates a new one in case there is no vertex with such an id
	 *
	 *	@tparam CVertexType is the vertex type name
	 *	@tparam CInitializer is the lazy initializer type name
	 *
	 *	@param[in] n_id is the id of the vertex required (must be id of an existing optimized vertex,
	 *		one larger than id of the last optimized vertex or a valid id of a const vertex)
	 *	@param[in] init is the initializer functor; it is only called (converted to vertex) in case new vertex is created
	 *
	 *	@return Returns reference to the vertex, associated with id n_id.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function throws std::runtime_error (in case vertex ids aren't contiguous).
	 *	@note 2016-08-09 - changed the initializer to be a const reference to be able to initialize
	 *		vertices directly with Eigen types (on x86, Visual Studio fails to align function arguments properly).
	 */
	template <class CVertexType, class CInitializer>
	inline CVertexType &r_Get_Vertex(_TyId n_id, const CInitializer &init) // throw(std::bad_alloc, std::runtime_error)
	{
		if(n_id == m_vertex_pool.n_Size()) {
			if(have_ConstVertices && SIZE_MAX - n_id == m_const_vertex_pool.n_Size()) // in case the next id could also belong to the const vertex pool
				throw std::runtime_error("vertex ids depleted: constness ambiguous");

			CVertexType &r_t_new_vert = m_vertex_pool.r_Add_Element((CVertexType)(init));
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			r_t_new_vert.Set_Id(n_id);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			r_t_new_vert.Set_Order(m_n_vertex_element_num);
			m_n_vertex_element_num += r_t_new_vert.n_Dimension();
			return r_t_new_vert;
		} else {
			if(n_id < m_vertex_pool.n_Size())
				return m_vertex_pool.template r_At<CVertexType>(n_id);
			else if(have_ConstVertices && n_id >= SIZE_MAX - m_const_vertex_pool.n_Size()) {

				return this->template r_Get_ConstVertex<CVertexType>(n_id, init); // the id points in the constant vertex pool
			}

			throw std::runtime_error("vertices must be accessed in incremental manner");
		}

		//return _TyGetVertexImpl::template r_Get_Vertex<CVertexType>(n_id, init,
		//	m_vertex_pool, m_const_vertex_pool, m_vertex_lookup, m_n_vertex_element_num);
		// use specialized implementation, based on whether const vertices are enabled or not
		// complicated

		/*if(n_id == m_vertex_pool.n_Size()) {
			CVertexType &r_t_new_vert = m_vertex_pool.r_Add_Element((CVertexType)(init));
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			r_t_new_vert.Set_Id(n_id);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			r_t_new_vert.Set_Order(m_n_vertex_element_num);
			m_n_vertex_element_num += r_t_new_vert.n_Dimension();
			return r_t_new_vert;
		} else {
			if(n_id < m_vertex_pool.n_Size())
				return *(CVertexType*)&m_vertex_pool[n_id]; // a bit of dirty casting
			throw std::runtime_error("vertices must be accessed in incremental manner");
		}*/
		// old implementation that does not allow const vertices
	}

	/**
	 *	@brief finds a constant vertex by id or creates a new one in case there is no vertex with such an id
	 *
	 *	@tparam CVertexType is the vertex type name
	 *	@tparam CInitializer is the lazy initializer type name
	 *
	 *	@param[in] n_id is the id of the vertex required (must be id of an existing const vertex,
	 *		one smaller than id of the last const vertex or SIZE_MAX if no const vertices exist yet)
	 *	@param[in] init is the initializer functor; it is only called (converted to vertex) in case new vertex is created
	 *
	 *	@return Returns reference to the vertex, associated with id n_id.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function throws std::runtime_error (in case vertex ids aren't contiguous).
	 *	@note 2016-08-09 - changed the initializer to be a const reference to be able to initialize
	 *		vertices directly with Eigen types (on x86, Visual Studio fails to align function arguments properly).
	 */
	template <class CVertexType, class CInitializer>
	inline typename CEnableIf<have_ConstVertices, CVertexType&>::T
		r_Get_ConstVertex(_TyId n_id, const CInitializer &init) // throw(std::bad_alloc, std::runtime_error)
	{
#ifndef __BASE_TYPES_ALLOW_CONST_VERTICES
		throw std::runtime_error("__BASE_TYPES_ALLOW_CONST_VERTICES not defined but const vertex added");
#endif // !__BASE_TYPES_ALLOW_CONST_VERTICES

		return this->template r_Get_ConstVertex_ConstCheck<CVertexType>(n_id, init);
	}

protected:
	// todo - doc
	template <class CVertexType, class CInitializer>
	inline typename CEnableIf<have_ConstVertices && CFindTypelistItem<_TyConstVertexTypelist,
		CVertexType>::b_result, CVertexType&>::T
		r_Get_ConstVertex_ConstCheck(_TyId n_id, const CInitializer &init) // throw(std::bad_alloc, std::runtime_error)
	{
		_ASSERTE(n_id >= SIZE_MAX - m_const_vertex_pool.n_Size());
		// make sure noone is trying to call this with zero-based indices (if this triggers, please read the documentation section on using contstant vertices)

		n_id = SIZE_MAX - n_id;
		// convert to zero-based ids

		if(n_id == m_const_vertex_pool.n_Size()) {
			if(SIZE_MAX - n_id == m_vertex_pool.n_Size()) // in case the next id could also belong to the optimized vertex pool
				throw std::runtime_error("vertex ids depleted: constness ambiguous");

			CVertexType &r_t_new_vert = m_const_vertex_pool.r_Add_Element((CVertexType)(init));
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
			r_t_new_vert.Set_Id(-1); // const vertices do not have id assigned
#endif // __BASE_TYPES_USE_ID_ADDRESSING
			r_t_new_vert.Set_Order(-1); // const vertices do not have order assigned
			return r_t_new_vert;
		} else {
			_ASSERTE(n_id < m_const_vertex_pool.n_Size()); // we checked this before entering this branch

			return m_const_vertex_pool.template r_At<CVertexType>(n_id);
		}
	}

	template <class CVertexType, class CInitializer>
	inline typename CEnableIf<!have_ConstVertices || !CFindTypelistItem<_TyConstVertexTypelist,
		CVertexType>::b_result, CVertexType&>::T
		r_Get_ConstVertex_ConstCheck(_TyId n_id, const CInitializer &init) // throw(std::bad_alloc, std::runtime_error)
	{
		throw std::runtime_error("attempted to create constant vertex of type not in constant vertex list");

		static CVertexType dummy_value = (CVertexType)init;
		return dummy_value; // avoid unwanted build errors
	}

public:
	/**
	 *	@brief finds a constant vertex by id or creates a new one in case there is no vertex with such an id
	 *		(specialization for const vertices disabled)
	 *
	 *	This function always throws std::runtime_error, as the const vertices are disabled.
	 *
	 *	@tparam CVertexType is the vertex type name
	 *	@tparam CInitializer is the lazy initializer type name
	 *
	 *	@param[in] n_id is the id of the vertex required (unused)
	 *	@param[in] init is the initializer functor; it is only called (converted to vertex) in case new vertex is created
	 *
	 *	@return Returns reference to the vertex, associated with id n_id.
	 *
	 *	@note This function throws std::runtime_error (const vertices not enabled).
	 *	@note 2016-08-09 - changed the initializer to be a const reference to be able to initialize
	 *		vertices directly with Eigen types (on x86, Visual Studio fails to align function arguments properly).
	 */
	template <class CVertexType, class CInitializer>
	inline typename CEnableIf<!have_ConstVertices, CVertexType&>::T
		r_Get_ConstVertex(_TyId UNUSED(n_id), const CInitializer &init) // throw(std::bad_alloc, std::runtime_error)
	{
		// do not use static assert here, this is actually called from r_Get_Vertex(), although the code never gets there

		throw std::runtime_error("const vertices not enabled"); // not enabled, can't insert any vertices
		static CVertexType dummy_value = (CVertexType)init; // now it does not need to be default-constructible
		return dummy_value;
	}

	/**
	 *	@brief reads vertex state
	 *
	 *	@param[in] n_id is vertex id, may include const vertices much like \ref r_Get_Vertex()
	 *	@param[in] n_optimized_vertex_num is the number of optimized vertices (the value
	 *		returned by \ref r_Vertex_Pool().n_Size())
	 *
	 *	@return Returns a read-only reference to the vertex state.
	 *
	 *	@note This function cannot add new vertices and while protected by an assertion in
	 *		debug mode, accessing vertices with unallocated id will result in undefined behavior.
	 */
	inline _TyAnyVertexStateConstRef v_VertexState(_TyId n_id, size_t n_optimized_vertex_num) const
	{
		return v_VertexState_Impl<have_ConstVertices>(n_id, n_optimized_vertex_num);
		// this needs to be wrapped inside a template function, otherwise
		// accessing the const vertex pool will yield compile time errors
	}

	/**
	 *	@brief reads vertex state
	 *
	 *	@param[in] n_id is vertex id, may include const vertices much like \ref r_Get_Vertex()
	 *
	 *	@return Returns a read-only reference to the vertex state.
	 *
	 *	@note This function cannot add new vertices and while protected by an assertion in
	 *		debug mode, accessing vertices with unallocated id will result in undefined behavior.
	 *	@note This function calls \ref r_Vertex_Pool().n_Size() so if calling it repeatedly,
	 *		it might be more efficient to use the overload with two parameters.
	 */
	inline _TyAnyVertexStateConstRef v_VertexState(_TyId n_id) const
	{
		return v_VertexState_Impl<have_ConstVertices>(n_id, r_Vertex_Pool().n_Size());
		// this needs to be wrapped inside a template function, otherwise
		// accessing the const vertex pool will yield compile time errors
	}

	/**
	 *	@brief adds a new edge to the system
	 *
	 *	@param[in] t_edge is the initial edge value
	 *
	 *	@return Returns reference to the new edge.
	 *
	 *	@note This function throws std::bad_alloc.
	 *	@note This function throws std::runtime_error if problems with unary factor arise.
	 *	@note This sets edge order, do not overwrite it!
	 */
	template <class _TyEdge>
	_TyEdge &r_Add_Edge(_TyEdge t_edge) // throw(std::bad_alloc, std::runtime_error)
	{
		if(!null_UnaryFactor && !m_t_unary_factor.cols()) { // !m_t_unary_factor.cols() probably as expensive (or less) as m_edge_pool.b_Empty()
			do {
#ifndef __AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
				if(have_ConstVertices && t_edge.n_Vertex_Id(0) > m_vertex_pool.n_Size()) {
					throw std::runtime_error("the first vertex of the first edge may not be "
						"constant unless CNullUnaryFactorFactory is used");
				}
				// either do not have const vertices, or the vertex 0 of the first edge is not a const vertex.
				// otherwise we would have to choose a different vertex for the unary factor which is something
				// that the current implementations of UF factories do not do
#else // !__AUTO_UNARY_FACTOR_ON_VERTEX_ZERO
				bool b_have_vertex_0 = false;
				for(int i = 0; i < int(_TyEdge::n_vertex_num); ++ i) {
					if(!t_edge.n_Vertex_Id(i)) {
						b_have_vertex_0 = true;
						break;
					}
				}
				if(!b_have_vertex_0)
					break;
				// if the edge does not reference vertex 0, do not add the UF now and wait it out

				// should not add if r_edge does not contain vertex with id 0 (can occur if the vertex
				// is initialized explicitly, such as in BA problems, Venice is a good example).
				// actually should construct unary factors from vertices, that would solve the problem,
				// only the vertices do not come with information matrices. on the other hand cannot
				// wait, as the edge could trigger incremental optimization. have to wait and handle
				// the possibility of the UF missing.

				// while this is troubling and may slow down a little, unless your data comes in totally
				// unsorted, this will only be called for a single edge (and then the UF will initialize
				// and we'll never enter this branch again)
#endif // !__AUTO_UNARY_FACTOR_ON_VERTEX_ZERO

				m_unary_factor_factory(m_t_unary_factor, m_v_unary_error, t_edge);
				_ASSERTE(m_t_unary_factor.cols() > 0); // make sure it initialized it
				m_n_unary_factor_order = m_n_edge_element_num;
				// auto-initialize the unary factor

				m_n_edge_element_num += m_t_unary_factor.rows();
			} while(0);
		}
		// in case it is the first edge, gets the unary factor and the error associated with it

		_TyEdge &r_t_new_edge = m_edge_pool.r_Add_Element(t_edge);
#ifdef __BASE_TYPES_USE_ID_ADDRESSING
		r_t_new_edge.Set_Id(m_edge_pool.n_Size() - 1);
#endif // __BASE_TYPES_USE_ID_ADDRESSING
		r_t_new_edge.Set_Order(m_n_edge_element_num);

		m_n_edge_element_num += r_t_new_edge.n_Dimension();
		// the first edge is preceded by the unary factor

		return r_t_new_edge;
	}

	/**
	 *	@brief plots the system as a .tga (targa) image
	 *
	 *	@param[in] p_s_filename is the output file name
	 *	@param[in] n_quality_profile is plot profile (one of plot_*)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Plot2D(const char *p_s_filename, plot_quality::EQualityProfile n_quality_profile) const
	{
		switch(n_quality_profile) {
		case plot_quality::plot_Draft:
			return Plot2D(p_s_filename, 1024, 8192, 2, 1, 1, 1, false, true, 32, false);
		case plot_quality::plot_Printing:
			return Plot2D(p_s_filename, 2048, 2048, 10, 3, 7, 1, true, false, 10);
		case plot_quality::plot_Printing_LandmarkTicksOnly:
			return Plot2D(p_s_filename, 2048, 2048, 10, 3, 7, 1, true, false, 10, true);
		case plot_quality::plot_Printing_NoTicks:
			return Plot2D(p_s_filename, 2048, 2048, 0, 0, 7, 1, true, false, 4);
		};
		return false;
	}

	/**
	 *	@brief plots the system as a .tga (targa) image
	 *
	 *	This version assumes a 3D system, it plots it as its projection to the XZ plane.
	 *
	 *	@param[in] p_s_filename is the output file name
	 *	@param[in] n_quality_profile is plot profile (one of plot_*)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Plot3D(const char *p_s_filename, plot_quality::EQualityProfile n_quality_profile) const
	{
		switch(n_quality_profile) {
		case plot_quality::plot_Draft:
			return Plot3D(p_s_filename, 1024, 8192, 2, 1, 1, 1, false, true, 32, false);
		case plot_quality::plot_Printing:
			return Plot3D(p_s_filename, 2048, 2048, 10, 3, 7, 1, true, false, 10);
		case plot_quality::plot_Printing_LandmarkTicksOnly:
			return Plot3D(p_s_filename, 2048, 2048, 10, 3, 7, 1, true, false, 10, true);
		case plot_quality::plot_Printing_NoTicks:
			return Plot3D(p_s_filename, 2048, 2048, 0, 0, 7, 1, true, false, 4);
		};
		return false;
	}

	/**
	 *	@brief plots the system as a .tga (targa) image
	 *
	 *	@param[in] p_s_filename is the output file name
	 *	@param[in] n_smaller_side_resoluion is the resolution of the shorter side of the image. in pixels
	 *	@param[in] n_max_resolution is the maximal resolution limit for either dimension, in pixels
	 *		(in case the aspect ratio is too high, the longer side is set to n_max_resolution
	 *		and the shorter side is scaled appropriately)
	 *	@param[in] f_tick_size is vertex tick size, in pixels (advanced settings)
	 *	@param[in] f_tick_line_width is vertex tick line width, in pixels (advanced settings)
	 *	@param[in] f_edge_line_width is edge line width, in pixels (advanced settings)
	 *	@param[in] f_landmark_edge_line_width is pose-landmark edge line width, in pixels (advanced settings)
	 *	@param[in] b_dark_landmark_edges is pose-landmark color selector; if set,
	 *		it is black, otherwise it is light gray (advanced settings)
	 *	@param[in] b_draw_frame is flag for drawing a frame around the image
	 *		(the frame is fit tightly around the image)
	 *	@param[in] n_padding is padding around the image (around the frame, if rendered)
	 *	@param[in] b_landmark_ticks_only is vertex rendering flag (if set, only landmarks are rendered,
	 *		otherwise all vertices are rendered)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Plot2D(const char *p_s_filename, int n_smaller_side_resoluion = 1024,
		int n_max_resolution = 8192, float f_tick_size = 2, float f_tick_line_width = 1,
		float f_edge_line_width = 1, float f_landmark_edge_line_width = 1,
		bool b_dark_landmark_edges = false, bool b_draw_frame = true, int n_padding = 32,
		bool b_landmark_ticks_only = false) const
	{
		Eigen::Vector2d v_min(-1, -1), v_max(1, 1);
		Vertex_BoundingBox(v_min, v_max, Eigen::Matrix2d::Identity());
		// find minima / maxima

		int n_width, n_height;
		double f_scale;
		plot::CPlotUtils::Calc_Resolution(n_width, n_height, f_scale, v_min, v_max,
			n_smaller_side_resoluion, n_max_resolution);
		// calculate image size

		Eigen::Matrix2d t_transform;
		Eigen::Vector2d v_offset;
		plot::CPlotUtils::Calc_RasterTransform(t_transform, v_offset, n_width, n_height,
			n_padding, f_scale, v_min, Eigen::Matrix2d::Identity());
		// update spatial transform to raster transform

		TBmp *p_image;
		if(!(p_image = TBmp::p_Alloc(n_width + 2 * n_padding, n_height + 2 * n_padding)))
			return false;

		p_image->Clear(0xffffffU);
		// white background

		if(b_draw_frame) {
			int n_tick_offset = int(ceil(fabs(f_tick_size)));
			n_padding -= n_tick_offset;
			n_width += 2 * n_tick_offset;
			n_height += 2 * n_tick_offset;
			p_image->DrawRect(n_padding, n_padding, n_width + n_padding, n_height + n_padding, 0xff000000U);
			n_padding += n_tick_offset;
			n_width -= 2 * n_tick_offset;
			n_height -= 2 * n_tick_offset;
		}
		// black borders

		enum {
			n_landmark_edge_dim = 2,
			n_landmark_vertex_dim = 2,
			n_min_vertex_dim = 2
		};
		_ASSERTE(t_transform.cols() == n_min_vertex_dim);

		Draw_Edges(p_image, t_transform, v_offset, (b_dark_landmark_edges)?
			0xffaaaaaaU : 0xffeeeeeeU, f_landmark_edge_line_width,
			n_landmark_edge_dim, n_landmark_edge_dim);
		// draw edges to landmarks

		const int n_tick_type = (f_tick_size > 2.5f)? plot::tick_Cross :
			plot::tick_Cross_Subpixel, n_ctick_type = plot::tick_TiltedCross;
		Draw_Vertices(p_image, t_transform, v_offset, 0xffff0000U, n_tick_type, f_tick_size,
			f_tick_line_width, (b_landmark_ticks_only)? int(n_landmark_vertex_dim) : 0,
			(b_landmark_ticks_only)? int(n_landmark_vertex_dim) : SIZE_MAX);
		Draw_ConstVertices(p_image, t_transform, v_offset, 0xff8800ffU, n_ctick_type, f_tick_size,
			f_tick_line_width, (b_landmark_ticks_only)? int(n_landmark_vertex_dim) : 0,
			(b_landmark_ticks_only)? int(n_landmark_vertex_dim) : SIZE_MAX);
		// draw all the vertices

		Draw_Edges(p_image, t_transform, v_offset, 0xff0000ffU,
			f_edge_line_width, n_landmark_edge_dim, n_landmark_edge_dim, true);
		// draw all other edges

		bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_image, false);
		p_image->Delete();

		return b_result;
	}

	/**
	 *	@brief plots the system as a .tga (targa) image
	 *
	 *	This version assumes a 3D system, it plots it as its projection to the XZ plane.
	 *
	 *	@param[in] p_s_filename is the output file name
	 *	@param[in] n_smaller_side_resoluion is the resolution of the shorter side of the image. in pixels
	 *	@param[in] n_max_resolution is the maximal resolution limit for either dimension, in pixels
	 *		(in case the aspect ratio is too high, the longer side is set to n_max_resolution
	 *		and the shorter side is scaled appropriately)
	 *	@param[in] f_tick_size is vertex tick size, in pixels (advanced settings)
	 *	@param[in] f_tick_line_width is vertex tick line width, in pixels (advanced settings)
	 *	@param[in] f_edge_line_width is edge line width, in pixels (advanced settings)
	 *	@param[in] f_landmark_edge_line_width is pose-landmark edge line width, in pixels (advanced settings)
	 *	@param[in] b_dark_landmark_edges is pose-landmark color selector; if set,
	 *		it is black, otherwise it is light gray (advanced settings)
	 *	@param[in] b_draw_frame is flag for drawing a frame around the image
	 *		(the frame is fit tightly around the image)
	 *	@param[in] n_padding is padding around the image (around the frame, if rendered)
	 *	@param[in] b_landmark_ticks_only is vertex rendering flag (if set, only landmarks are rendered,
	 *		otherwise all vertices are rendered)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Plot3D(const char *p_s_filename, int n_smaller_side_resoluion = 1024,
		int n_max_resolution = 8192, float f_tick_size = 2, float f_tick_line_width = 1,
		float f_edge_line_width = 1, float f_landmark_edge_line_width = 1,
		bool b_dark_landmark_edges = false, bool b_draw_frame = true, int n_padding = 32,
		bool b_landmark_ticks_only = false) const
	{
		Eigen::Vector2d v_min(-1, -1), v_max(1, 1);
		Eigen::Matrix<double, 2, 3> t_transform;
		t_transform << 1, 0, 0, 0, 0, 1; // extract x and z coordinates
		Vertex_BoundingBox(v_min, v_max, t_transform);
		// find minima / maxima

		int n_width, n_height;
		double f_scale;
		plot::CPlotUtils::Calc_Resolution(n_width, n_height, f_scale, v_min, v_max,
			n_smaller_side_resoluion, n_max_resolution);
		// calculate image size

		Eigen::Vector2d v_offset;
		plot::CPlotUtils::Calc_RasterTransform(t_transform, v_offset, n_width, n_height,
			n_padding, f_scale, v_min, t_transform);
		// update spatial transform to raster transform

		TBmp *p_image;
		if(!(p_image = TBmp::p_Alloc(n_width + 2 * n_padding, n_height + 2 * n_padding)))
			return false;

		p_image->Clear(0xffffffU);
		// white background

		if(b_draw_frame) {
			int n_tick_offset = int(ceil(fabs(f_tick_size)));
			n_padding -= n_tick_offset;
			n_width += 2 * n_tick_offset;
			n_height += 2 * n_tick_offset;
			p_image->DrawRect(n_padding, n_padding, n_width + n_padding, n_height + n_padding, 0xff000000U);
			n_padding += n_tick_offset;
			n_width -= 2 * n_tick_offset;
			n_height -= 2 * n_tick_offset;
		}
		// black borders

		enum {
			n_landmark_edge_dim = 3,
			n_landmark_vertex_dim = 3,
			n_min_vertex_dim = 3
		};
		_ASSERTE(t_transform.cols() == n_min_vertex_dim);

		Draw_Edges(p_image, t_transform, v_offset, (b_dark_landmark_edges)?
			0xffaaaaaaU : 0xffeeeeeeU, f_landmark_edge_line_width,
			n_landmark_edge_dim - 1, n_landmark_edge_dim); // BA landmark edges are 2D, SE(3) landmark edges are 3D
		// draw edges to landmarks

		const int n_tick_type = (f_tick_size > 2.5f)? plot::tick_Cross :
			plot::tick_Cross_Subpixel, n_ctick_type = plot::tick_TiltedCross;
		Draw_Vertices(p_image, t_transform, v_offset, 0xffff0000U, n_tick_type, f_tick_size,
			f_tick_line_width, (b_landmark_ticks_only)? int(n_landmark_vertex_dim) - 1 : 0,
			(b_landmark_ticks_only)? int(n_landmark_vertex_dim) : SIZE_MAX);
		Draw_ConstVertices(p_image, t_transform, v_offset, 0xff8800ffU, n_ctick_type, f_tick_size,
			f_tick_line_width, (b_landmark_ticks_only)? int(n_landmark_vertex_dim) - 1 : 0,
			(b_landmark_ticks_only)? int(n_landmark_vertex_dim) : SIZE_MAX);
		// draw all the vertices

		Draw_Edges(p_image, t_transform, v_offset, 0xff0000ffU,
			f_edge_line_width, n_landmark_edge_dim - 1, n_landmark_edge_dim, true); // BA landmark edges are 2D, SE(3) landmark edges are 3D
		// draw all other edges

		bool b_result = CTgaCodec::Save_TGA(p_s_filename, *p_image, false);
		p_image->Delete();

		return b_result;
	}

	/**
	 *	@brief saves vertex positions into a text file
	 *
	 *	@param[in] p_s_filename is the output file name
	 *	@param[in] b_dump_also_const_vertices is const vertex output flag (if set, all the constant
	 *		vertices go after all the optimized vertices and are ordered by their increasing unsigned
	 *		id--that is reverse order than in which they were inserted; if cleared, no constant
	 *		vertices are present in the output (default))
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Dump(const char *p_s_filename, bool b_dump_also_const_vertices = false) const
	{
		FILE *p_fw;
		if(!(p_fw = fopen(p_s_filename, "w"))) {
			//fprintf(stderr, "error: failed to open \'%\' for writing\n", p_s_filename);
			return false;
		}

		if(!have_ConstVertices || !b_dump_also_const_vertices) { // only the optimized vertices are stored
			for(size_t i = 0, n = m_vertex_pool.n_Size(); i < n; ++ i) {
				Eigen::Map<const Eigen::VectorXd> v_state = m_vertex_pool[i].v_State();
				for(size_t j = 0, m = v_state.rows(); j < m; ++ j) {
					double f = v_state(j);
					if(fabs(f) > 1)
						fprintf(p_fw, (j)? ((j + 1 == m)? " %f\n" : " %f") : ((j + 1 == m)? "%f\n" : "%f"), f);
					else
						fprintf(p_fw, (j)? ((j + 1 == m)? " %g\n" : " %g") : ((j + 1 == m)? "%g\n" : "%g"), f);
				}
			}
		} else {
			for(size_t i = 0, n = m_vertex_pool.n_Size(); i < n; ++ i) {
				Eigen::Map<const Eigen::VectorXd> v_state = m_vertex_pool[i].v_State();
				for(size_t j = 0, m = v_state.rows(); j < m; ++ j) {
					double f = v_state(j);
					if(fabs(f) > 1)
						fprintf(p_fw, (j)? ((j + 1 == m)? " %f\n" : " %f") : ((j + 1 == m)? "%f\n" : "%f"), f);
					else
						fprintf(p_fw, (j)? ((j + 1 == m)? " %g\n" : " %g") : ((j + 1 == m)? "%g\n" : "%g"), f);
				}
			}
			// first the optimized vertices in natural order

			for(size_t i = m_const_vertex_pool.n_Size(); i > 0;) {
				-- i; // here
				_TyConstVertexStateConstRef v_state = v_ConstVertexState_Impl<have_ConstVertices>(i);
				for(size_t j = 0, m = v_state.rows(); j < m; ++ j) {
					double f = v_state(j);
					if(fabs(f) > 1)
						fprintf(p_fw, (j)? ((j + 1 == m)? " %f\n" : " %f") : ((j + 1 == m)? "%f\n" : "%f"), f);
					else
						fprintf(p_fw, (j)? ((j + 1 == m)? " %g\n" : " %g") : ((j + 1 == m)? "%g\n" : "%g"), f);
				}
			}
			// const vertices afterwards, in reverse order (in the increasing unsigned id order)
		}

		if(ferror(p_fw)) {
			//fprintf(stderr, "error: error writing \'%\' (disk full?)\n", p_s_filename);
			fclose(p_fw);
			return false;
		}

		return !fclose(p_fw);
	}

	/**
	 *	@brief calculates bounding box of the vertices in the system
	 *
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[out] r_v_min is filled with the minimum coordinates of the transformed vertices
	 *	@param[out] r_v_max is filled with the maximum coordinates of the transformed vertices
	 *	@param[in] r_t_transform is vertex transformation matrix (only vertices of dimension
	 *		greater or equal to the number of columns of this matrix are considered)
	 *	@param[in] b_include_const_vertices is const vertex include flag
	 *		(has no effect if there are no const vertices)
	 *
	 *	@return Returns true if there were any vertices to initialize the bounding box,
	 *		otherwise returns false.
	 */
	template <class Derived0, class Derived1, class Derived2>
	bool Vertex_BoundingBox(Eigen::MatrixBase<Derived1> &r_v_min,
		Eigen::MatrixBase<Derived2> &r_v_max,
		const Eigen::MatrixBase<Derived0> &r_t_transform,
		bool b_include_const_vertices = true) const
	{
		const size_t n_input_dim = r_t_transform.cols();
		const size_t n_output_dim = r_t_transform.rows();
		// this is probably already done in Eigen::Matrix::rows() and Eigen::Matrix::cols()

		_ASSERTE(r_v_min.rows() == n_output_dim);
		_ASSERTE(r_v_max.rows() == n_output_dim);
		// make sure that the dimensions are correct

		r_v_min.setZero();
		r_v_max.setZero();
		// handle the case of no vertices

		bool b_had_vertex = false;
		Eigen::Matrix<double, Derived0::RowsAtCompileTime, 1> v_projected;
		for(size_t i = 0, n = m_vertex_pool.n_Size(); i < n; ++ i) {
			_TyVertexStateConstRef v_state = t_MakeVectorRef<_TyVertexStateConstRef>(m_vertex_pool[i].v_State());
			if(size_t(v_state.rows()) < n_input_dim)
				continue; // or could append zeros in a similar fashion OpenGL attribs work
			// get vertex state of a sufficient dimension (this might skip landmarks or some other parameters)

			//v_projected = r_t_transform * v_state.head(n_input_dim); // ambiguous with Eigen::EigenBase
			//v_projected = r_t_transform.lazyProduct(v_state.head(n_input_dim)); // lazyProduct only in Eigen::MatrixBase
			//v_projected = (v_state.head(n_input_dim).transpose().lazyProduct(r_t_transform.transpose())).transpose(); // transpose not a member of Eigen::EigenBase
			//v_projected = (v_state.head(n_input_dim).transpose().lazyProduct(
			//	Eigen::Transpose<Eigen::EigenBase<Derived0> >(r_t_transform))).transpose(); // plain broken
			v_projected = r_t_transform * v_state.head(n_input_dim); // need to use Eigen::MatrixBase, unable to optimize for special matrices (e.g. identity)
			if(b_had_vertex) {
				r_v_min = r_v_min.array().min(v_projected.array()).matrix();
				r_v_max = r_v_max.array().max(v_projected.array()).matrix();
			} else {
				r_v_min = r_v_max = v_projected;
				b_had_vertex = true;
			}
		}
		if(b_include_const_vertices) {
			for(size_t i = 0, n = m_const_vertex_pool.n_Size(); i < n; ++ i) {
				_TyConstVertexStateConstRef v_state = v_ConstVertexState_Impl<have_ConstVertices>(i);
				if(size_t(v_state.rows()) < n_input_dim)
					continue; // or could append zeros in a similar fashion OpenGL attribs work
				// get vertex state of a sufficient dimension (this might skip landmarks or some other parameters)

				v_projected = r_t_transform * v_state.head(n_input_dim);
				if(b_had_vertex) {
					r_v_min = r_v_min.array().min(v_projected.array()).matrix();
					r_v_max = r_v_max.array().max(v_projected.array()).matrix();
				} else {
					r_v_min = r_v_max = v_projected;
					b_had_vertex = true;
				}
			}
		}

		return b_had_vertex;
	}

	/**
	 *	@brief plots the selected edges as lines
	 *
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[in,out] p_bitmap is pointer to the bitmap image to plot into
	 *	@param[in] r_t_transform is transformation matrix from vertex to raster coordinates
	 *	@param[in] r_v_offset is raster coordinates offset
	 *	@param[in] n_color is edge color
	 *	@param[in] f_line_width is line width
	 *	@param[in] n_min_dimension is minimum measurement dimension (inclusive)
	 *	@param[in] n_max_dimension is maximum measurement dimension (inclusive)
	 *	@param[in] b_invert_dim_test is dimension test inversion flag (if set,
	 *		only edges with dimension outside of the [n_min_dimension, n_max_dimension]
	 *		interval are plotted; if cleared, only edges inside the interval are plotted)
	 *	@param[in] n_first_edge is zero-based index of the first edge to draw
	 *	@param[in] n_last_edge is zero-based index of one past the last edge to draw
	 *		or -1 for the number of edges in the system
	 *
	 *	@note This function does not attempt to draw unary edges, and for hyperedges
	 *		only the first two vertices are connected by a line.
	 */
	template <class Derived>
	void Draw_Edges(TBmp *p_bitmap, const Eigen::MatrixBase<Derived> &r_t_transform,
		const Eigen::Vector2d &r_v_offset, uint32_t n_color, float f_line_width = 1,
		size_t n_min_dimension = 0, size_t n_max_dimension = SIZE_MAX,
		bool b_invert_dim_test = false, size_t n_first_edge = 0, size_t n_last_edge = size_t(-1)) const
	{
		const size_t n_min_vertex_dim = r_t_transform.cols();
		const size_t n_optimized_vertex_num = m_vertex_pool.n_Size();

		// could use for-each and traits for deciding which edge types to draw and which to skip
		for(size_t i = n_first_edge, n = (n_last_edge == size_t(-1))?
		   m_edge_pool.n_Size() : n_last_edge; i < n; ++ i) {
			typename _TyEdgeMultiPool::_TyConstBaseRef r_edge = m_edge_pool[i];

			size_t n_dim = r_edge.n_Dimension();
			if((!b_invert_dim_test && (n_dim < n_min_dimension || n_dim > n_max_dimension)) ||
			   (b_invert_dim_test && !(n_dim < n_min_dimension || n_dim > n_max_dimension)) ||
			   r_edge.n_Vertex_Num() < 2)
				continue;
			// only binary edges of selected measurement dimensions

			_TyAnyVertexStateConstRef r_t_vert0 = v_VertexState(r_edge.n_Vertex_Id(0), n_optimized_vertex_num);
			_TyAnyVertexStateConstRef r_t_vert1 = v_VertexState(r_edge.n_Vertex_Id(1), n_optimized_vertex_num);
			if(size_t(r_t_vert0.rows()) < n_min_vertex_dim || size_t(r_t_vert1.rows()) < n_min_vertex_dim)
				continue;
			Eigen::Vector2d v_vert0 = r_t_transform * r_t_vert0.head(n_min_vertex_dim) + r_v_offset;
			Eigen::Vector2d v_vert1 = r_t_transform * r_t_vert1.head(n_min_vertex_dim) + r_v_offset;
			// transform to raster coords

			float f_x0 = float(v_vert0(0)), f_y0 = float(v_vert0(1));
			float f_x1 = float(v_vert1(0)), f_y1 = float(v_vert1(1));
			// convert to float

			p_bitmap->DrawLine_AA(f_x0, f_y0, f_x1, f_y1, n_color, f_line_width);
		}
		// draw edges
	}

	/**
	 *	@brief plots the selected (optimized) vertices
	 *
	 *	@param[in,out] p_bitmap is pointer to the bitmap image to plot into
	 *	@param[in] r_t_transform is transformation matrix from vertex to raster coordinates
	 *	@param[in] r_v_offset is raster coordinates offset
	 *	@param[in] n_color is edge color
	 *	@param[in] n_tick_type is tick type (0 for simple cross, 1 for antialiased cross,
	 *		2 for tilted cross, 3 for circle, 4 for disc, 5 for square, 6 for filled square)
	 *	@param[in] f_tick_size is tick size in pixels
	 *	@param[in] f_line_width is tick line width
	 *	@param[in] n_min_dimension is minimum vertex dimension (inclusive)
	 *	@param[in] n_max_dimension is maximum vertex dimension (inclusive)
	 *	@param[in] b_invert_dim_test is dimension test inversion flag (if set,
	 *		only vertices with dimension outside of the [n_min_dimension, n_max_dimension]
	 *		interval are plotted; if cleared, only vertices inside the interval are plotted)
	 *	@param[in] n_first_vertex is zero-based index of the first vertex to draw
	 *	@param[in] n_last_vertex is zero-based index of one past the last vertex to draw
	 *		or -1 for the number of vertices in the system
	 */
	template <class Derived>
	void Draw_Vertices(TBmp *p_bitmap, const Eigen::MatrixBase<Derived> &r_t_transform,
		const Eigen::Vector2d &r_v_offset, uint32_t n_color, int n_tick_type = 0, float f_tick_size = 2,
		float f_line_width = 1, size_t n_min_dimension = 0, size_t n_max_dimension = SIZE_MAX,
		bool b_invert_dim_test = false, size_t n_first_vertex = 0, size_t n_last_vertex = size_t(-1)) const
	{
		if(!f_tick_size && !f_line_width)
			return;

		const size_t n_min_vertex_dim = r_t_transform.cols();

		for(size_t i = n_first_vertex, n = (n_last_vertex == size_t(-1))?
		   m_vertex_pool.n_Size() : n_last_vertex; i < n; ++ i) {
			Eigen::Map<const Eigen::VectorXd> v_state = m_vertex_pool[i].v_State();

			size_t n_dim = v_state.rows();
			if((!b_invert_dim_test && (n_dim < n_min_dimension || n_dim > n_max_dimension)) ||
			   (b_invert_dim_test && !(n_dim < n_min_dimension || n_dim > n_max_dimension)) ||
			   n_dim < n_min_vertex_dim)
				continue;

			Eigen::Vector2d v_vert = r_t_transform * v_state.head(n_min_vertex_dim) + r_v_offset;
			float f_x = float(v_vert(0)), f_y = float(v_vert(1));
			// transform to raster coords, convert to float

			plot::CPlotUtils::Draw_Tick(p_bitmap, f_x, f_y, n_color, n_tick_type, f_tick_size, f_line_width);
			// draw a tick
		}
	}

	/**
	 *	@brief plots the selected const vertices
	 *
	 *	@param[in,out] p_bitmap is pointer to the bitmap image to plot into
	 *	@param[in] r_t_transform is transformation matrix from vertex to raster coordinates
	 *	@param[in] r_v_offset is raster coordinates offset
	 *	@param[in] n_color is edge color
	 *	@param[in] n_tick_type is tick type (0 for simple cross, 1 for antialiased cross,
	 *		2 for tilted cross, 3 for circle, 4 for disc, 5 for square, 6 for filled square)
	 *	@param[in] f_tick_size is tick size in pixels
	 *	@param[in] f_line_width is tick line width
	 *	@param[in] n_min_dimension is minimum vertex dimension (inclusive)
	 *	@param[in] n_max_dimension is maximum vertex dimension (inclusive)
	 *	@param[in] b_invert_dim_test is dimension test inversion flag (if set,
	 *		only vertices with dimension outside of the [n_min_dimension, n_max_dimension]
	 *		interval are plotted; if cleared, only vertices inside the interval are plotted)
	 *	@param[in] n_first_vertex is zero-based index of the first vertex to draw
	 *	@param[in] n_last_vertex is zero-based index of one past the last vertex to draw
	 *		or -1 for the number of const vertices in the system
	 */
	template <class Derived>
	void Draw_ConstVertices(TBmp *p_bitmap, const Eigen::MatrixBase<Derived> &r_t_transform,
		const Eigen::Vector2d &r_v_offset, uint32_t n_color, int n_tick_type = 0, float f_tick_size = 2,
		float f_line_width = 1, size_t n_min_dimension = 0, size_t n_max_dimension = SIZE_MAX,
		bool b_invert_dim_test = false, size_t n_first_vertex = 0, size_t n_last_vertex = size_t(-1)) const
	{
		if(!have_ConstVertices || (!f_tick_size && !f_line_width))
			return;

		const size_t n_min_vertex_dim = r_t_transform.cols();

		for(size_t i = n_first_vertex, n = (n_last_vertex == size_t(-1))?
		   m_const_vertex_pool.n_Size() : n_last_vertex; i < n; ++ i) {
			_TyConstVertexStateConstRef v_state = v_ConstVertexState_Impl<have_ConstVertices>(i);

			size_t n_dim = v_state.rows();
			if((!b_invert_dim_test && (n_dim < n_min_dimension || n_dim > n_max_dimension)) ||
			   (b_invert_dim_test && !(n_dim < n_min_dimension || n_dim > n_max_dimension)) ||
			   n_dim < n_min_vertex_dim)
				continue;

			Eigen::Vector2d v_vert = r_t_transform * v_state.head(n_min_vertex_dim) + r_v_offset;
			float f_x = float(v_vert(0)), f_y = float(v_vert(1));
			// transform to raster coords, convert to float

			plot::CPlotUtils::Draw_Tick(p_bitmap, f_x, f_y, n_color, n_tick_type, f_tick_size, f_line_width);
			// draw a tick
		}
	}

protected:
	/**
	 *	@brief vertex state access implementation (specialization with const vertices)
	 *
	 *	@tparam b_with_const_vertices is const vertex flag (must equal have_ConstVertices)
	 *
	 *	@param[in] n_id is vertex id, may include const vertices much like \ref r_Get_Vertex()
	 *	@param[in] n_optimized_vertex_num is the number of optimized vertices (the value
	 *		returned by \ref r_Vertex_Pool().n_Size())
	 *
	 *	@return Returns a read-only reference to the vertex state.
	 *
	 *	@note This function cannot add new vertices and while protected by an assertion in
	 *		debug mode, accessing vertices with unallocated id will result in undefined behavior.
	 */
	template <const bool b_with_const_vertices>
	inline typename CEnableIf<b_with_const_vertices, _TyAnyVertexStateConstRef>::T
		v_VertexState_Impl(_TyId n_id, size_t n_optimized_vertex_num) const
	{
		return (n_id < n_optimized_vertex_num)?
			t_MakeVectorRef<_TyAnyVertexStateConstRef>(m_vertex_pool[n_id].v_State()) :
			t_MakeVectorRef<_TyAnyVertexStateConstRef>(m_const_vertex_pool[SIZE_MAX - n_id].v_State());
		// note that we need to cast the result of v_State() to _TyAnyVertexStateConstRef
		// as either branch of the ternary operator may be returning a different type
	}

	/**
	 *	@brief vertex state access implementation (specialization for no const vertices)
	 *
	 *	@tparam b_with_const_vertices is const vertex flag (must equal have_ConstVertices)
	 *
	 *	@param[in] n_id is vertex id, may include const vertices much like \ref r_Get_Vertex()
	 *	@param[in] n_optimized_vertex_num is the number of optimized vertices (unused)
	 *
	 *	@return Returns a read-only reference to the vertex state.
	 *
	 *	@note This function cannot add new vertices and while protected by an assertion in
	 *		debug mode, accessing vertices with unallocated id will result in undefined behavior.
	 */
	template <const bool b_with_const_vertices>
	inline typename CEnableIf<!b_with_const_vertices, _TyAnyVertexStateConstRef>::T
		v_VertexState_Impl(_TyId n_id, size_t UNUSED(n_optimized_vertex_num)) const
	{
		return t_MakeVectorRef<_TyAnyVertexStateConstRef>(m_vertex_pool[n_id].v_State());
	}

	/**
	 *	@brief const vertex state access implementation (specialization with const vertices)
	 *	@tparam b_with_const_vertices is const vertex flag (must equal have_ConstVertices)
	 *	@param[in] n_index is zero-based const vertex index
	 *	@return Returns a read-only reference to the selected const vertex state.
	 *	@note This function cannot add new vertices and while protected by an assertion in
	 *		debug mode, accessing vertices with unallocated id will result in undefined behavior.
	 */
	template <const bool b_with_const_vertices>
	inline typename CEnableIf<b_with_const_vertices, _TyConstVertexStateConstRef>::T
		v_ConstVertexState_Impl(_TyId n_index) const
	{
		return t_MakeVectorRef<_TyConstVertexStateConstRef>(m_const_vertex_pool[n_index].v_State());
	}

	/**
	 *	@brief const vertex state access implementation (specialization for no const vertices)
	 *	@tparam b_with_const_vertices is const vertex flag (must equal have_ConstVertices)
	 *	@param[in] n_index is zero-based const vertex index
	 *	@return Returns a read-only reference to the selected const vertex state.
	 *	@note This function cannot add new vertices and while protected by an assertion in
	 *		debug mode, accessing vertices with unallocated id will result in undefined behavior.
	 */
	template <const bool b_with_const_vertices>
	inline typename CEnableIf<!b_with_const_vertices, _TyConstVertexStateConstRef>::T
		v_ConstVertexState_Impl(_TyId n_id) const
	{
		_ASSERTE(0); // not supposed to access this
		return *(_TyConstVertexStateConstRef*)0; // die
	}
};

#endif // !__FLAT_SYSTEM_TEMPLATE_INCLUDED
