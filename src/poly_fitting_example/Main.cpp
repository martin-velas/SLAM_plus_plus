/*
								+------------------------------------+
								|                                    |
								| *** Polynomial Fitting Example *** |
								|                                    |
								|   Copyright (c) -tHE SWINe- 2016   |
								|                                    |
								|              Main.cpp              |
								|                                    |
								+------------------------------------+
*/

/**
 *	@file src/poly_fitting_example/Main.cpp
 *	@brief contains the main() function of the polynomial fitting example
 *	@author -tHE SWINe-
 *	@date 2016-09-02
 */

#include <stdio.h> // printf
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_Schur.h"
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/BaseTypes.h" // CBaseVertexImpl, CBaseEdgeImpl
#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h> // to show the result image, otherwise not required
#endif // _WIN32 || _WIN64

/**
 *	@brief cubic polynomial vertex type (this is what we want to estimate)
 */
class CVertexCubicPolynomial : public CBaseVertexImpl<CVertexCubicPolynomial, 4> { // this is a 4D vertex
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	/**
	 *	@brief default constructor; initializes state vector
	 *	@param[in] r_v_state is state vector initializer
	 */
	inline CVertexCubicPolynomial(const Eigen::Vector4d &r_v_state = Eigen::Vector4d::Zero())
		:CBaseVertexImpl<CVertexCubicPolynomial, 4>(r_v_state)
	{}

	/**
	 *	@copydoc base_iface::CVertexFacade::Operator_Plus()
	 */
	inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
	{
		m_v_state += r_v_delta.segment<4>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
	}
};

/**
 *	@brief function value observation edge
 */
class CEdgeFunctionSample : public CBaseEdgeImpl<CEdgeFunctionSample,
	MakeTypelist(CVertexCubicPolynomial), // a list of vertices this edge relates (just the polynomial)
	1, // dimension of the residual: the difference in observed and evaluated y
	2> { // dimension of the observation vector: (x, y) sample of the function value (this argument is optional and if not entered, it is assumed to be the same as the residual dimension)
public:
	__GRAPH_TYPES_ALIGN_OPERATOR_NEW

	typedef CBaseEdgeImpl<CEdgeFunctionSample, MakeTypelist(CVertexCubicPolynomial), 1, 2> _TyBase; // a shorthand for the base type

	/**
	 *	@brief default constructor; required by the storage
	 */
	CEdgeFunctionSample()
	{}

	/**
	 *	@brief constructor; initializes edge with data
	 *
	 *	@tparam CSystem is type of system where this edge is being stored
	 *
	 *	@param[in] n_vertex is (zero-based) index of the polynomial vertex
	 *	@param[in] v_observation is vector of (x, y) observed function value
	 *	@param[in] r_t_inv_sigma is the information matrix
	 *	@param[in,out] r_system is reference to system (used to query edge vertices)
	 */
	template <class CSystem>
	CEdgeFunctionSample(size_t n_vertex, const Eigen::Vector2d &v_observation,
		const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system)
		:_TyBase(n_vertex, v_observation, r_t_inv_sigma, null_initialize_vertices, r_system)
	{}

	/**
	 *	@brief calculates jacobians, expectation and error
	 *
	 *	@param[out] r_t_jacobian is jacobian of the residual with respect to the polynomial coefficients
	 *	@param[out] r_v_expectation is expecation value (1D vector)
	 *	@param[out] r_v_error is error value (1D vector)
	 */
	inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 4> &r_t_jacobian,
		Eigen::Matrix<double, 1, 1> &r_v_expectation, Eigen::Matrix<double, 1, 1> &r_v_error) const
	{
		const double x = m_v_measurement(0), y = m_v_measurement(1);
		// this is our observation of the function value

		const Eigen::Vector4d v_poly = m_p_vertex0->v_State();
		r_v_expectation(0) = v_poly(0) + v_poly(1) * x + v_poly(2) * x * x + v_poly(3) * x * x * x; // a simple cubic function
		// calculates the expectation

		r_v_error(0) = y - r_v_expectation(0);
		// calculates error as a difference of the observation and the current expectation

		r_t_jacobian << 1, x, x * x, x * x * x;
		// a very simple jacobian of (negative) delta error / delta polynomial coefficients
		// use e.g. Matlab:
		// >> syms x y r a b c d real
		// >> r = y - (a + b * x + c * x * x + d * x * x * x)
		// >> jacobian(-r, [a b c d])
		//
		// ans =
		//		[   1,   x, x^2, x^3]
	}

	/**
	 *	@brief calculates \f$\chi^2\f$ error
	 *	@return Returns (unweighted) \f$\chi^2\f$ error for this edge.
	 */
	inline double f_Chi_Squared_Error() const
	{
		Eigen::Matrix<double, 1, 4> jacobian;
		Eigen::Matrix<double, 1, 1> v_expectation, v_error;
		Calculate_Jacobian_Expectation_Error(jacobian, v_expectation, v_error);
		// calculates the error (and also expectation and jacobian that are not needed)

		return v_error.dot(m_t_sigma_inv * v_error); // calculate chi2; equals squared error if the covariance is identity
	}
};

/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int UNUSED(n_arg_num), const char **UNUSED(p_arg_list))
{
	typedef MakeTypelist(CVertexCubicPolynomial) TVertexTypelist;
	typedef MakeTypelist(CEdgeFunctionSample) TEdgeTypelist;
	typedef CFlatSystem<CVertexCubicPolynomial, TVertexTypelist, CEdgeFunctionSample, TEdgeTypelist> CSystemType;
	// set up the optimized system

	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
	// set up the linear solver

	CSystemType system;
	CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> solver(system);
	// make a new system and a new nonlinear solver

	Eigen::Matrix<double, 1, 1> information = Eigen::Matrix<double, 1, 1>::Identity() * 100;
	// prepare the information matrix (all edges have the same)

	const Eigen::Vector2d p_observations[] = {
		Eigen::Vector2d(-0.894531, -0.789062), Eigen::Vector2d(-0.589844, -0.792969),
		Eigen::Vector2d(-0.460937, -0.71875), Eigen::Vector2d(-0.089844, -0.152344),
		Eigen::Vector2d(-0.125001, -0.324219), Eigen::Vector2d(0.0117185, -0.0429685),
		Eigen::Vector2d(0.554687, 0.277344), Eigen::Vector2d(0.527344, 0.453125),
		Eigen::Vector2d(0.777344, -0.167969), Eigen::Vector2d(0.839844, -0.347656),
		Eigen::Vector2d(0.925781, -0.582031), Eigen::Vector2d(0.6875, 0.0468748),
		Eigen::Vector2d(0.468751, 0.527344), Eigen::Vector2d(0.335938, 0.480469),
		Eigen::Vector2d(0.167969, 0.332032), Eigen::Vector2d(0.226563, 0.421875),
		Eigen::Vector2d(-0.359375, -0.597656), Eigen::Vector2d(-0.820313, -0.8125),
		Eigen::Vector2d(-0.953125, -0.785156), Eigen::Vector2d(-0.21875, -0.265625),
		Eigen::Vector2d(-0.308594, -0.535156), Eigen::Vector2d(-0.71875, -0.804687),
		Eigen::Vector2d(0.902344, -0.761719), Eigen::Vector2d(-0.195313, -0.472657),
		Eigen::Vector2d(0.0390625, 0.171875), Eigen::Vector2d(-0.03125, 0.0664063),
		Eigen::Vector2d(0.65625, 0.210938), Eigen::Vector2d(0.863281, -0.554688)
	};
	size_t n_observation_num = sizeof(p_observations) / sizeof(p_observations[0]);
	// some prepared 2D points

	for(size_t i = 0; i < n_observation_num; ++ i)
		system.r_Add_Edge(CEdgeFunctionSample(0, p_observations[i], information, system));
	// put edges in the system (all observe the same polynomial 0)

	// note that vertex 0 is zero-initialized automatically; this corresponds
	// to the initial guess of a straight horizontal line

	solver.Optimize();
	// optimize the system, use up to 5 iterations

	solver.Dump();
	// show some stats

	Eigen::Vector4d v_poly = system.r_Vertex_Pool()[0].v_State();
	// get the resulting polynomial

	{
		TBmp *p_bitmap = TBmp::p_Alloc(512, 512);
		p_bitmap->Clear(0xffffffffU);
		// create a white bitmap

		double f_prev_x = -2, f_prev_y = 0;
		for(double x = -1; x <= 1; x += .02) {
			double y = v_poly(0) + v_poly(1) * x + v_poly(2) * x * x + v_poly(3) * x * x * x;
			p_bitmap->DrawLine_AA(float(f_prev_x * 256 + 256), float(-f_prev_y * 256 + 256),
				float(x * 256 + 256), float(-y * 256 + 256), 0xff0000ffU, 2);
			f_prev_x = x;
			f_prev_y = y;
		}
		// draw the estimated line (flip y because the y axis of the bitmap points down, we want it up)

		for(size_t i = 0; i < n_observation_num; ++ i) {
			p_bitmap->FillCircle_AA(float(p_observations[i](0) * 256 + 256),
				float(-p_observations[i](1) * 256 + 256), 5, 0Xffff0000U);
		}
		// draw the observed values as red discs

		CTgaCodec::Save_TGA("result.tga", *p_bitmap, false);
		// save to a file

		p_bitmap->Delete();
		// cleanup
	}
	// draw the output as an image

#if defined(_WIN32) || defined(_WIN64)
	ShellExecute(0, "open", "result.tga", 0, 0, SW_SHOW);
#endif // _WIN32 || _WIN64
	// on windows, we can open the file with the results in a window

	return 0;
}

/**
 *	@page polyfitexample Polynomial Fitting Example
 *
 *	This is an example of polynomial fitting. While SLAM++ is geared towards graph problems,
 *	it is possible to also solve such general problems. This example is self-contained; it
 *	declares its vertex and edge types, to make it easier to understand how this is done.
 *
 *	We will solve a classic least squares problem of finding a polynomial fit to a function,
 *	given a set of function samples in the form of <tt>(x, y)</tt> argument-value pairs.
 *
 *	Our graph will consist of a single vertex (the polynomial to be estimated) and several
 *	edges (the observations of the function value).
 *
 *	The vertex type is very simple and looks like this:
 *
 *	@code
 *	class CVertexCubicPolynomial : public CBaseVertexImpl<CVertexCubicPolynomial, 4> {
 *	public:
 *		__GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *
 *		inline CVertexCubicPolynomial(const Eigen::Vector4d &r_v_state = Eigen::Vector4d::Zero())
 *			:CBaseVertexImpl<CVertexCubicPolynomial, 4>(r_v_state)
 *		{}
 *
 *		inline void Operator_Plus(const Eigen::VectorXd &r_v_delta)
 *		{
 *			m_v_state += r_v_delta.segment<4>(m_n_order);
 *		}
 *	};
 *	@endcode
 *
 *	It uses the CRTP idiom where it tells its base class \ref CBaseVertexImpl the final class name.
 *	The second parameter of \ref CBaseVertexImpl is the dimension of this vertex. Since our polynomial
 *	will be cubic, this is four. The constructor just copies the state vector. The <tt>Operator_Plus</tt>
 *	takes a delta vector that has the dimension equal to the sum of all the optimized variables.
 *	The function thus takes a segment out of it which corresponds to the current vertex and adds it to
 *	the state vector. For variables involving rotations in space, this addition becomes composition.
 *
 *	Next, it is needed to declare an edge which serves as the observation of the function value:
 *
 *	@code
 *	class CEdgeFunctionSample : public CBaseEdgeImpl<CEdgeFunctionSample, MakeTypelist(CVertexCubicPolynomial), 1, 2> {
 *	public:
 *		__GRAPH_TYPES_ALIGN_OPERATOR_NEW
 *
 *		typedef CBaseEdgeImpl<CEdgeFunctionSample, MakeTypelist(CVertexCubicPolynomial), 1, 2> _TyBase;
 *
 *		CEdgeFunctionSample()
 *		{}
 *
 *		template <class CSystem>
 *		CEdgeFunctionSample(size_t n_vertex, const Eigen::Vector2d &v_observation,
 *			const Eigen::Matrix<double, 1, 1> &r_t_inv_sigma, CSystem &r_system)
 *			:_TyBase(n_vertex, v_observation, r_t_inv_sigma, null_initialize_vertices, r_system)
 *		{}
 *
 *		inline void Calculate_Jacobian_Expectation_Error(Eigen::Matrix<double, 1, 4> &r_t_jacobian,
 *			Eigen::Matrix<double, 1, 1> &r_v_expectation, Eigen::Matrix<double, 1, 1> &r_v_error) const
 *		{
 *			const double x = m_v_measurement(0), y = m_v_measurement(1);
 *			const Eigen::Vector4d v_poly = m_p_vertex0->v_State();
 *			r_v_expectation(0) = v_poly(0) + v_poly(1) * x + v_poly(2) * x * x + v_poly(3) * x * x * x;
 *			// calculates the expectation
 *
 *			r_v_error(0) = y - r_v_expectation(0);
 *			// calculates error as a difference of the observation and the current expectation
 *
 *			r_t_jacobian << 1, x, x * x, x * x * x;
 *			// a very simple jacobian of (negative) delta error / delta polynomial coefficients
 *		}
 *
 *		inline double f_Chi_Squared_Error() const
 *		{
 *			Eigen::Matrix<double, 1, 4> jacobian;
 *			Eigen::Matrix<double, 1, 1> v_expectation, v_error;
 *			Calculate_Jacobian_Expectation_Error(jacobian, v_expectation, v_error);
 *			return v_error.dot(m_t_sigma_inv * v_error); // calculate chi2
 *		}
 *	};
 *	@endcode
 *
 *	This is basically only a default constructor, a constructor and a function that calculates
 *	the error and the Jacobian. The Jacobian can be found e.g. using Matlab, by writing:
 *
 *	@code
 *	>> syms x y r a b c d real
 *	>> r = y - (a + b * x + c * x * x + d * x * x * x)
 *	>> jacobian(-r, [a b c d])
 *
 *	ans =
 *		[   1,   x, x^2, x^3]
 *	@endcode
 *
 *	The <tt>f_Chi_Squared_Error</tt> is required like this for performance purposes; one can save
 *	calculating the Jacobian when only the \f$\chi^2\f$ error is needed. If the performance is not
 *	an issue, then this code fragment can be copied around.
 *
 *	Now that the graph types are defined, we can define and declare the optimizable system:
 *
 *	@code
 *	typedef MakeTypelist(CVertexCubicPolynomial) TVertexTypelist;
 *	typedef MakeTypelist(CEdgeFunctionSample) TEdgeTypelist;
 *	typedef CFlatSystem<CVertexCubicPolynomial, TVertexTypelist, CEdgeFunctionSample, TEdgeTypelist> CSystemType;
 *
 *	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
 *
 *	CSystemType system;
 *	CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> solver(system);
 *	@endcode
 *
 *	The first three lines define a graph; a list of admissible vertices and edges and
 *	the final system class, where the base types are specified as well. Since there is only
 *	a single vertex type and a single edge type, those are base types themselves.
 *
 *	The next line declares the type of linear solver to use. This makes it possible to use our
 *	fast block Cholesky, specialized for block sizes corresponding to the variables in the system.
 *	In this case, all the blocks are \f$4 \times 4\f$.
 *
 *	The last two lines declare an optimized system and the nonlinear solver that attaches to it.
 *	At this point, it is possible to add edges to the system, using \ref CFlatSystem::r_Add_Edge().
 *	This also initializes the vertices, as a side effect. The way the vertices are initialized is
 *	set in the edge constructor - in this case, the vertex is initialized to zero. This corresponds
 *	to an initial guess of a constant zero function.
 *
 *	The vertices can be also initialized manually, before adding edges that reference them, by using
 *	\ref CFlatSystem::r_Get_Vertex().
 *
 *	The system is optimized by calling:
 *
 *	@code
 *	solver.Optimize();
 *	@endcode
 *
 *	This takes up to five iterations by default (it can be overriden by the first argument).
 *	The example ends by drawing an image with the observations and the estimated polynomial.
 *
 *	@section pfe_next_sec Further Reading
 *
 *	Some basic topics are covered here:
 *	* \ref simpleexample shows how to use SLAM++ for batch solving using the predefined types
 *	* \ref onlineexample shows how to implement a simple online solver
 *	* \ref rot3d discusses different representations of rotations in 3D
 *	* \ref baifaceexample shows how to easily wrap SLAM++ with a simple interface
 *
 *	@section pfe_adv_sec Advanced
 *
 *	Some advanced topics are covered in the following pages:
 *	* \ref facades shows how to access edges and vertices stored in the optimized graph
 *	* \ref ownsolvers shows how to implement solver for a custom problem
 *	* \ref constvertices shows how to use constant vertices
 *	* \ref unaryfactors shows how to use unary factors
 */

/*
 *	end-of-file
 */
