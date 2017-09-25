/*
								+-----------------------------------+
								|                                   |
								|   ***  Online SLAM Example  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam_online_example/Main.cpp
 *	@brief contains the main() function of the online example SLAM program
 *	@author -tHE SWINe-
 *	@date 2013-11-27
 */

#include <stdio.h> // printf
#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_CholMod.h" // linear solvers (only one is required)
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/SE2_Types.h" // SE(2) types

#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h> // to show the result image, otherwise not required
#endif // _WIN32 || _WIN64

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
	typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
	typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;

	typedef CFlatSystem<CVertexPose2D, TVertexTypelist, CEdgePose2D, TEdgeTypelist> CSystemType;

	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
	
	typedef CNonlinearSolver_FastL<CSystemType, CLinearSolverType> CNonlinearSolverType;

	CSystemType system;
	CNonlinearSolverType solver(system, solve::Nonlinear(frequency::Every(1))); // solve each 1

	system.r_Get_Vertex<CVertexPose2D>(0, Eigen::Vector3d(0, 0, M_PI / 4));
	// initialize the first vertex (the robot starts rotated 45 debrees)

	Eigen::Matrix3d information;
	information <<
		45,  0,  0,
		 0, 45,  0,
		 0,  0, 45;
	// prepare the information matrix (all edges have the same)

	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(0, 1,
		Eigen::Vector3d(1.02765, -0.00745597, 0.00483283), information, system))); // replace (1.02765, -0.00745597, 0.00483283) with readings from the ROS stack
	// get the solution every step

	system.r_Vertex_Pool(); // returns container with all the vertices which are optimized
	system.Dump("vertices.txt");  // puts the same vertices in a text file
	system.Plot2D("afterFirstStep.tga", plot_quality::plot_Printing); // plot in print quality
	// get a snapshot of the system

	solver.Delay_Optimization();
	// when adding a bunch of measurements, maybe we don't care about incremental solutions in between

	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(1, 2,
		Eigen::Vector3d(-0.0120155, 1.00436, 1.56679), information, system)));
	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(2, 3,
		Eigen::Vector3d(0.0440195, 0.988477, -1.56353), information, system)));
	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(3, 0,
		Eigen::Vector3d(0.0139844, -1.02344, -0.00780158), information, system)));
	// put edges in the system, the vertices are created and initialized automatically

	solver.Enable_Optimization();
	// re-enable optimization, and optimize right away if it needs to

	system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality

#if defined(_WIN32) || defined(_WIN64)
	ShellExecute(0, "open", "result.tga", 0, 0, SW_SHOW);
#endif // _WIN32 || _WIN64
	// on windows, we can open the file with the results in a window

	return 0;
}

/**
 *	@page onlineexample Simple Online Example
 *
 *	This example shows how to implement a simple incremental SLAM for online scenarios.
 *	It is in \ref slam_online_example/Main.cpp.
 *
 *	We begin by including several files:
 *	@code
 *	#include <stdio.h> // printf
 *	#include "slam/LinearSolver_UberBlock.h" // linear solver
 *	#include "slam/ConfigSolvers.h" // nonlinear graph solvers
 *	#include "slam/SE2_Types.h" // SE(2) types
 *	@endcode
 *
 *	In case your problem is in different space than SE(2), you can choose from several different types
 *	that are implemented in SLAM++ core, or easily \ref ownsolvers "implement your own types". Different
 *	types can be freely combined in a single problem.
 *
 *	\section systemsolverspec Specializing System and Solver Types
 *
 *	SLAM++ is based on compile-time specialization. Most of the informations is passed using typelists.
 *	First thing to specify is set of edges and vertices that are permitted in the system. This is important,
 *	as the dimensions of those will govern block sizes in the system matrix, and will be consequently used
 *	to specialize our BLAS (basic linear algebra) library in order to make processing faster. While it is
 *	permitted to put all possible types in the list, but the performance will not be the best.
 *
 *	To specify vertex types in our small example, we simply do the following:
 *
 *	@code
 *	typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
 *	typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;
 *	@endcode
 *
 *	This creates lists for vertices and edges: only 2D poses are allowed. To allow also 2D landmarks,
 *	one would simply do:
 *
 *	@code
 *	typedef MakeTypelist(CVertexPose2D, CVertexLandmark2D) TVertexTypelist;
 *	typedef MakeTypelist(CEdgePose2D, CEdgePoseLandmark2D) TEdgeTypelist;
 *	@endcode
 *
 *	Once we have the lists, we can specialize our system (will contain graph of vertices and measurements):
 *
 *	@code
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist> CSystemType;
 *	@endcode
 *
 *	Here, CBaseVertex is base class of all the vertices, CBaseEdge is base clas of all the edges.
 *	In case only a single type of vertex or edge is used, it can be its own base type (it is ever so slightly
 *	faster as the compiler can inline some of the function calls which would otherwise need to be virtual).
 *	Now an instance of the CSystemType can hold our system.
 *
 *	Based on the system, it is possible to specialize linear solver type:
 *
 *	@code
 *	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;
 *	@endcode
 *
 *	Here, CSystemType::_TyHessianMatrixBlockList contains a list of block sizes in the Hessian
 *	matrix (the normal form). CLinearSolver_UberBlock is solver, based on our block matrix scheme.
 *	It is also possible to use CLinearSolver_CSparse, CLinearSolver_CXSparse or CLinearSolver_CholMod.
 *
 *	Similarly, we prepare a nonlinear solver type:
 *
 *	@code
 *	typedef CNonlinearSolver_FastL<CSystemType, CLinearSolverType> CNonlinearSolverType;
 *	@endcode
 *
 *	Here, CNonlinearSolver_FastL determines the specific implementation of the solver, fast L is
 *	the best at incremental solving.
 *
 *	\section firstpervert Creating the System and Adding the First Vertex
 *
 *	Finally, all the types that are required are specified, and we can make them do the work:
 *
 *	@code
 *	CSystemType system;
 *	CNonlinearSolverType solver(system, solve::Nonlinear(frequency::Every(1)));
 *	@endcode
 *
 *	The system is empty, the solver is instructed to optimize that system after every single
 *	step (methods to set different modes are described in the \ref incrementalsolvepolicy
 *	"incremental policies page"). Adding a vertex to the system (for example to set the
 *	initial pose) is very simple:
 *
 *	@code
 *	system.r_Get_Vertex<CVertexPose2D>(0, Eigen::Vector3d(0, 0, M_PI / 4));
 *	@endcode
 *
 *	This will create a new 2D pose with index 0, position (0, 0) and rotation \f$\pi/ 4\f$.
 *	You can experiment with this initial pose and see how that effects the solution.
 *	If no initial pose is supplied, the default pose is constructed by the edge (in SE edge
 *	implementations, \ref CBaseEdgeImpl::CInitializeNullVertex "null" vector is the default pose).
 *
 *	\section addedges Adding Edges
 *
 *	The solver is normally driven by inserting edges, initializing vertices other than
 *	the inital pose or perhaps camera parameters in bundle adjustment is unusual. To add a new
 *	edge to the system, one just does:
 *
 *	@code
 *	Eigen::Matrix3d information;
 *	information <<
 *		45,  0,  0,
 *		 0, 45,  0,
 *		 0,  0, 45;
 *	// prepare the information matrix (all edges have the same)
 *
 *	CEdgePose2D new_edge = CEdgePose2D(0, 1,
 *		Eigen::Vector3d(1.02765, -0.00745597, 0.00483283),
 *		information,
 *		system);
 *	// create a new edge
 *
 *	CEdgePose2D &r_new_edge = system.r_Add_Edge(new_edge);
 *	// add edge to the system
 *
 *	solver.Incremental_Step(r_new_edge);
 *	// get the solution every step
 *	@endcode
 *
 *	This might seem quite complicated, but it is quite simple (and in the example it is combined
 *	in just three lines of code). The first bit creates a \f$3 \times 3\f$ matrix, containing the
 *	inverse covariance: the information matrix.
 *
 *	The next bit constructs an edge between two 2D poses (that is given by the use of CEdgePose2D type).
 *	The index of the first pose is 0 (SLAM++ is written in C++ and uses zero-based indices!), index
 *	of the second pose is 1. The poses must be numbered from 0, and there must be no "empty" space
 *	between the indices (having only vertices 0, 2 is illegal). After that comes the actual measurement,
 *	a \f$R^3\f$ vector with \f$\Delta\f$x, \f$\Delta\f$y and \f$\Delta\f$rotation. In an actual online
 *	scenario, this would be replaced by reading values from a sensor. Then there is the information
 *	matrix, and finally a system. Reference to a system is required, as the edge needs to get its
 *	vertices from there (and in case it happens to create a new vertex, it also puts it there).
 *
 *	After that, the edge itself is added to the system. The CFlatSystem::r_Add_Edge() function returns
 *	reference (address) to the edge in the system, and this address is guaranteed not to change in
 *	the progress of solving. It is a different address than the one at which the new_edge variable is stored.
 *
 *	Finally, the solver is notified that we made an incremental step. Based on the incremental solve
 *	settings, it may decide to optimize the system, or just duly note the edge and return quickly.
 *
 *	The last three bits are usually contracted to a single compact command:
 *
 *	@code
 *	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(0, 1,
 *		Eigen::Vector3d(1.02765, -0.00745597, 0.00483283), information, system)));
 *	@endcode
 *
 *	A part of the optimization decision process is also a threshold on the norm of the residual. If the
 *	residual (the step by which all the vertices in the system would be displaced) is very small, the
 *	solver does not update anything. The size of this threshold is set along with the \ref incrementalsolvepolicy
 *	"incremental solving policy", and can be set explicitly in the solver constructor, like this:
 *
 *	@code
 *	CNonlinearSolverType solver(system, solve::Nonlinear(frequency::Every(1), 5, 0.01));
 *	@endcode
 *
 *	Where <tt>5</tt> is the numbef of iterations the nonlinear solver is allowed to take and <tt>0.01</tt>
 *	is the threshold on the residual norm. You can use \ref CNonlinearSolver_FastL::Dump() to print the
 *	number of iterations the solver took (among other statistics). If this number is zero, then a high
 *	threshold might have prevented the solver from actually optimizing anything (the norm of the residual
 *	depends on the scale and parameterization of the problem).
 *
 *	\section stateaccess Accessing the State
 *
 *	Now that a new measurement was added, the system state now entails two vertices. One we initialized
 *	manually and the other one is given by the measurement. To access the vertices at any time, one can
 *	use:
 *
 *	@code
 *	system.r_Vertex_Pool();
 *	@endcode
 *
 *	That returns reference to a container with all the vertices which are optimized. Alternately, one can
 *	write out all the vertices to a text file, one vertex per line:
 *
 *	@code
 *	system.Dump("vertices.txt");
 *	@endcode
 *
 *	There is also a function to draw a bitmap with vertices and edges in it, which we use to generate
 *	pictures in our papers. It is as simple as this: 
 *
 *	@code
 *	system.Plot2D("afterFirstStep.tga", plot_quality::plot_Printing); // plot in print quality
 *	@endcode
 *
 *	\section delayed Making It All Faster
 *
 *	As described in section \ref addedges, it is possible to add new measurements and the system will
 *	always contains the state of all the objects. When instructing the solver to update each 1 (as in
 *	the example), the system will always be optimized and close to the solution. Sometimes, it might
 *	not be necessary to optimize all the time, for example if a whole bunch of measurements is added
 *	from different sensors, we might not be interested how state changes in the process of integrating
 *	those in the system, rather we want just the state after the whole bunch is added, possibly saving
 *	some time in calculating needless updates. It is as simple as:
 *
 *	@code
 *	solver.Delay_Optimization();
 *
 *	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(1, 2,
 *		Eigen::Vector3d(-0.0120155, 1.00436, 1.56679), information, system)));
 *	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(2, 3,
 *		Eigen::Vector3d(0.0440195, 0.988477, -1.56353), information, system)));
 *	solver.Incremental_Step(system.r_Add_Edge(CEdgePose2D(3, 0,
 *		Eigen::Vector3d(0.0139844, -1.02344, -0.00780158), information, system)));
 *	// put edges in the system, the vertices are created and initialized automatically
 *
 *	solver.Enable_Optimization();
 *	@endcode
 *
 *	The first line instructs the solver to delay optimization until further notice. The last line
 *	tells it to optimize again, and may in fact optimize right away if it needs to.
 *
 *	\section concl Conclusion
 *
 *	This concludes the small online example. To see the final solution, one does:
 *
 *	@code
 *	system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
 *	@endcode
 *
 *	Which should draw a rough rectangle (the "robot" closed its first simple loop arroung the block
 *	in a Manhattan-like world).
 *
 */

/*
 *	end-of-file
 */
