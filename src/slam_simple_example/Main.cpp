/*
								+-----------------------------------+
								|                                   |
								|   ***  Simple SLAM Example  ***   |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam_simple_example/Main.cpp
 *	@brief contains the main() function of the simple example SLAM program
 *	@author -tHE SWINe-
 *	@date 2013-11-13
 */

#include <stdio.h> // printf
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#include "slam/LinearSolver_UberBlock.h"
#include "slam/LinearSolver_Schur.h"
//#include "slam/LinearSolver_CholMod.h" // linear solvers (only one is required)
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/SE2_Types.h" // SE(2) types

#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h> // to show the result image, otherwise not required
#endif // _WIN32 || _WIN64

/**
 *	@brief a helper function that adds a bunch of edges
 *
 *	@tparam CSystemType is type of the system, which is defined inside main()
 *
 *	@param[out] system is system to add the edges to
 *	@param[in] information is the information matrix for the edges
 *
 *	@note The data are taken from Olson's Manhattan dataset.
 */
template <class CSystemType>
void Add_More_ManhattanEdges(CSystemType &system, Eigen::Matrix3d information);

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
	//typedef CLinearSolver_CholMod CLinearSolverType; // or cholmod

	CSystemType system;
	CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> solver(system);

	Eigen::Matrix3d information;
	information <<
		45,  0,  0,
		 0, 45,  0,
		 0,  0, 45;
	// prepare the information matrix (all edges have the same)

	system.r_Add_Edge(CEdgePose2D(0, 1, Eigen::Vector3d(1.03039, 0.0113498, -0.0129577), information, system));
	system.r_Add_Edge(CEdgePose2D(1, 2, Eigen::Vector3d(1.0139, -0.0586393, -0.013225), information, system));
	system.r_Add_Edge(CEdgePose2D(2, 3, Eigen::Vector3d(1.02765, -0.00745597, 0.00483283), information, system));
	system.r_Add_Edge(CEdgePose2D(3, 4, Eigen::Vector3d(-0.0120155, 1.00436, 1.56679), information, system));
	Add_More_ManhattanEdges(system, information); // to avoid making this too long
	// put edges in the system, the vertices are created and initialized

	system.Plot2D("resultUnoptim.tga", plot_quality::plot_Printing); // plot in print quality

	solver.Optimize();
	// optimize the system

	system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
	solver.Dump(); // show some stats

#if defined(_WIN32) || defined(_WIN64)
	ShellExecute(0, "open", "result.tga", 0, 0, SW_SHOW);
#endif // _WIN32 || _WIN64
	// on windows, we can open the file with the results in a window

	return 0;
}

template <class CSystemType>
void Add_More_ManhattanEdges(CSystemType &system, Eigen::Matrix3d information)
{
	system.r_Add_Edge(CEdgePose2D(4, 5, Eigen::Vector3d(1.01603, 0.0145648, -0.0163041), information, system));
	system.r_Add_Edge(CEdgePose2D(5, 6, Eigen::Vector3d(1.02389, 0.00680757, 0.0109813), information, system));
	system.r_Add_Edge(CEdgePose2D(6, 7, Eigen::Vector3d(0.957734, 0.00315932, 0.0109005), information, system));
	system.r_Add_Edge(CEdgePose2D(7, 8, Eigen::Vector3d(-1.02382, -0.0136683, -3.09324), information, system));
	system.r_Add_Edge(CEdgePose2D(5, 9, Eigen::Vector3d(0.0339432, 0.0324387, -3.1274), information, system));
	system.r_Add_Edge(CEdgePose2D(8, 9, Eigen::Vector3d(1.02344, 0.0139844, -0.00780158), information, system));
	system.r_Add_Edge(CEdgePose2D(3, 10, Eigen::Vector3d(0.0440195, 0.988477, -1.56353), information, system));
	system.r_Add_Edge(CEdgePose2D(9, 10, Eigen::Vector3d(1.00335, 0.0222496, 0.0234909), information, system));
	system.r_Add_Edge(CEdgePose2D(10, 11, Eigen::Vector3d(0.977245, 0.0190423, -0.0286232), information, system));
	system.r_Add_Edge(CEdgePose2D(11, 12, Eigen::Vector3d(-0.99688, -0.0255117, 3.15627), information, system));
	system.r_Add_Edge(CEdgePose2D(12, 13, Eigen::Vector3d(0.990646, 0.0183964, -0.0165195), information, system));
	system.r_Add_Edge(CEdgePose2D(8, 14, Eigen::Vector3d(0.0158085, 0.0210588, 3.12831), information, system));
	system.r_Add_Edge(CEdgePose2D(13, 14, Eigen::Vector3d(0.945873, 0.00889308, -0.00260169), information, system));
	system.r_Add_Edge(CEdgePose2D(7, 15, Eigen::Vector3d(-0.0147277, -0.00159495, -0.0195786), information, system));
	system.r_Add_Edge(CEdgePose2D(14, 15, Eigen::Vector3d(1.00001, 0.00642824, 0.0282342), information, system));
	system.r_Add_Edge(CEdgePose2D(15, 16, Eigen::Vector3d(0.0378719, -1.02609, -1.5353), information, system));
	system.r_Add_Edge(CEdgePose2D(16, 17, Eigen::Vector3d(0.98379, 0.019891, 0.0240848), information, system));
	system.r_Add_Edge(CEdgePose2D(17, 18, Eigen::Vector3d(0.957199, 0.0295867, -0.0115004), information, system));
	system.r_Add_Edge(CEdgePose2D(18, 19, Eigen::Vector3d(0.99214, 0.0192015, -0.00729783), information, system));
	system.r_Add_Edge(CEdgePose2D(19, 20, Eigen::Vector3d(-0.0459215, -1.01632, -1.53912), information, system));
	system.r_Add_Edge(CEdgePose2D(20, 21, Eigen::Vector3d(0.99845, -0.00523202, -0.0340973), information, system));
	system.r_Add_Edge(CEdgePose2D(21, 22, Eigen::Vector3d(0.988728, 0.00903381, -0.0129141), information, system));
	system.r_Add_Edge(CEdgePose2D(22, 23, Eigen::Vector3d(0.989422, 0.00698231, -0.0242835), information, system));
	system.r_Add_Edge(CEdgePose2D(23, 24, Eigen::Vector3d(-1.00201, -0.00626341, 3.13974), information, system));
	system.r_Add_Edge(CEdgePose2D(24, 25, Eigen::Vector3d(1.01535, 0.00491314, 3.02279e-05), information, system));
	system.r_Add_Edge(CEdgePose2D(21, 26, Eigen::Vector3d(-0.95214, -0.0418463, 3.13475), information, system));
	system.r_Add_Edge(CEdgePose2D(25, 26, Eigen::Vector3d(1.03299, -0.00172652, 0.0224073), information, system));
	system.r_Add_Edge(CEdgePose2D(19, 27, Eigen::Vector3d(-0.0176158, -0.0052181, 1.56791), information, system));
	system.r_Add_Edge(CEdgePose2D(26, 27, Eigen::Vector3d(0.989137, -0.00857052, -0.0209045), information, system));
	system.r_Add_Edge(CEdgePose2D(27, 28, Eigen::Vector3d(-0.0483998, 0.981715, 1.56408), information, system));
	system.r_Add_Edge(CEdgePose2D(28, 29, Eigen::Vector3d(1.03082, -0.021271, -0.06069), information, system));
	system.r_Add_Edge(CEdgePose2D(29, 30, Eigen::Vector3d(1.01192, 0.0164477, -0.0352014), information, system));
	system.r_Add_Edge(CEdgePose2D(30, 31, Eigen::Vector3d(0.991338, 0.00781231, 0.0305919), information, system));
	system.r_Add_Edge(CEdgePose2D(31, 32, Eigen::Vector3d(0.00861057, -0.974025, -1.56961), information, system));
	system.r_Add_Edge(CEdgePose2D(32, 33, Eigen::Vector3d(1.04256, 0.0106692, 0.0220136), information, system));
	system.r_Add_Edge(CEdgePose2D(33, 34, Eigen::Vector3d(0.990826, 0.0166949, -0.0427845), information, system));
	system.r_Add_Edge(CEdgePose2D(34, 35, Eigen::Vector3d(0.995988, 0.029526, -0.0144112), information, system));
	system.r_Add_Edge(CEdgePose2D(35, 36, Eigen::Vector3d(-0.0107743, 0.996051, 1.59416), information, system));
	system.r_Add_Edge(CEdgePose2D(36, 37, Eigen::Vector3d(1.00499, 0.0110863, -0.00316511), information, system));
	system.r_Add_Edge(CEdgePose2D(37, 38, Eigen::Vector3d(1.03843, 0.0146778, -0.0323211), information, system));
	system.r_Add_Edge(CEdgePose2D(38, 39, Eigen::Vector3d(1.00625, 0.00674436, -0.0280641), information, system));
	system.r_Add_Edge(CEdgePose2D(39, 40, Eigen::Vector3d(0.0561635, 0.984988, -4.70377), information, system));
	system.r_Add_Edge(CEdgePose2D(40, 41, Eigen::Vector3d(0.984656, -0.0319246, 0.0110837), information, system));
	system.r_Add_Edge(CEdgePose2D(41, 42, Eigen::Vector3d(1.00266, 0.030635, 0.0300476), information, system));
	system.r_Add_Edge(CEdgePose2D(42, 43, Eigen::Vector3d(0.986417, -0.0130982, -0.0241183), information, system));
	system.r_Add_Edge(CEdgePose2D(43, 44, Eigen::Vector3d(0.97872, 0.0120778, -0.0117429), information, system));
	system.r_Add_Edge(CEdgePose2D(44, 45, Eigen::Vector3d(0.996113, -0.0407306, -0.0152182), information, system));
	system.r_Add_Edge(CEdgePose2D(45, 46, Eigen::Vector3d(1.00255, -0.00216301, -0.0105021), information, system));
	system.r_Add_Edge(CEdgePose2D(46, 47, Eigen::Vector3d(0.999641, -0.0336501, 0.0188071), information, system));
	system.r_Add_Edge(CEdgePose2D(47, 48, Eigen::Vector3d(-0.949748, 0.0117583, 3.11376), information, system));
	system.r_Add_Edge(CEdgePose2D(48, 49, Eigen::Vector3d(1.01739, 0.0123797, -0.00893411), information, system));
	system.r_Add_Edge(CEdgePose2D(49, 50, Eigen::Vector3d(1.01548, 0.0274024, -0.019191), information, system));
	system.r_Add_Edge(CEdgePose2D(40, 51, Eigen::Vector3d(2.97743, 0.0326539, 3.1211), information, system));
	system.r_Add_Edge(CEdgePose2D(50, 51, Eigen::Vector3d(1.05227, 0.0147383, -0.00136236), information, system));
	system.r_Add_Edge(CEdgePose2D(51, 52, Eigen::Vector3d(-0.0108141, -0.98436, -1.56099), information, system));
	system.r_Add_Edge(CEdgePose2D(52, 53, Eigen::Vector3d(1.03071, 0.00895876, -0.00840075), information, system));
	system.r_Add_Edge(CEdgePose2D(53, 54, Eigen::Vector3d(0.98342, 0.00979391, -0.0306844), information, system));
	system.r_Add_Edge(CEdgePose2D(7, 55, Eigen::Vector3d(-0.0335046, -0.00680906, -1.58407), information, system));
	system.r_Add_Edge(CEdgePose2D(54, 55, Eigen::Vector3d(1.01204, -0.015331, 0.00584842), information, system));
	system.r_Add_Edge(CEdgePose2D(14, 56, Eigen::Vector3d(0.00385417, 0.000186059, -3.14717), information, system));
	system.r_Add_Edge(CEdgePose2D(8, 56, Eigen::Vector3d(-0.0196555, 0.00673762, 0.0127182), information, system));
	system.r_Add_Edge(CEdgePose2D(55, 56, Eigen::Vector3d(-0.00365754, -0.984986, -1.57285), information, system));
	system.r_Add_Edge(CEdgePose2D(5, 57, Eigen::Vector3d(-0.048046, -0.00753482, -3.16285), information, system));
	system.r_Add_Edge(CEdgePose2D(56, 57, Eigen::Vector3d(1.031, -0.0163252, -0.0169613), information, system));
	system.r_Add_Edge(CEdgePose2D(57, 58, Eigen::Vector3d(0.983393, -0.0113447, -0.0148402), information, system));
	system.r_Add_Edge(CEdgePose2D(58, 59, Eigen::Vector3d(1.01024, 0.011576, 0.00432891), information, system));
	system.r_Add_Edge(CEdgePose2D(59, 60, Eigen::Vector3d(0.0201084, -1.00859, 4.73677), information, system));
	system.r_Add_Edge(CEdgePose2D(60, 61, Eigen::Vector3d(0.992544, -0.00406336, 0.00906878), information, system));
	system.r_Add_Edge(CEdgePose2D(61, 62, Eigen::Vector3d(0.980911, -0.0126781, 0.0247609), information, system));
	system.r_Add_Edge(CEdgePose2D(62, 63, Eigen::Vector3d(1.00765, -0.0370944, -0.00745089), information, system));
	system.r_Add_Edge(CEdgePose2D(47, 64, Eigen::Vector3d(-0.992098, -0.0164591, 3.12281), information, system));
	system.r_Add_Edge(CEdgePose2D(63, 64, Eigen::Vector3d(-0.0145417, -0.998609, -1.54739), information, system));
	system.r_Add_Edge(CEdgePose2D(64, 65, Eigen::Vector3d(1.03794, -0.0168313, -0.0130817), information, system));
	system.r_Add_Edge(CEdgePose2D(65, 66, Eigen::Vector3d(0.9912, 0.0115711, -0.0249519), information, system));
	system.r_Add_Edge(CEdgePose2D(66, 67, Eigen::Vector3d(0.949443, -0.0154924, -0.0091255), information, system));
	system.r_Add_Edge(CEdgePose2D(43, 68, Eigen::Vector3d(0.993623, 0.0391936, -0.00106149), information, system));
	system.r_Add_Edge(CEdgePose2D(67, 68, Eigen::Vector3d(-0.978361, -0.00927414, -3.13791), information, system));
	system.r_Add_Edge(CEdgePose2D(45, 69, Eigen::Vector3d(-0.006758, -0.00679624, -0.00213649), information, system));
	system.r_Add_Edge(CEdgePose2D(68, 69, Eigen::Vector3d(1.00367, -0.0352973, 0.0340684), information, system));
	system.r_Add_Edge(CEdgePose2D(69, 70, Eigen::Vector3d(1.02981, 0.00255454, 0.0150012), information, system));
	system.r_Add_Edge(CEdgePose2D(70, 71, Eigen::Vector3d(1.03652, 0.0118072, -0.00163612), information, system));
	system.r_Add_Edge(CEdgePose2D(71, 72, Eigen::Vector3d(0.00398168, -0.993979, 4.69836), information, system));
	system.r_Add_Edge(CEdgePose2D(72, 73, Eigen::Vector3d(0.969371, -0.0306017, -0.0326511), information, system));
	system.r_Add_Edge(CEdgePose2D(73, 74, Eigen::Vector3d(0.985691, 0.0111442, -0.00166414), information, system));
	system.r_Add_Edge(CEdgePose2D(74, 75, Eigen::Vector3d(0.981205, -0.00596464, 0.0226695), information, system));
	system.r_Add_Edge(CEdgePose2D(75, 76, Eigen::Vector3d(-0.00825988, 0.981841, -4.71863), information, system));
	system.r_Add_Edge(CEdgePose2D(76, 77, Eigen::Vector3d(1.01399, 0.0332094, -0.0649213), information, system));
	system.r_Add_Edge(CEdgePose2D(77, 78, Eigen::Vector3d(1.02795, 0.00984078, 0.0340066), information, system));
	system.r_Add_Edge(CEdgePose2D(78, 79, Eigen::Vector3d(1.00265, -0.00774271, 0.00950595), information, system));
	system.r_Add_Edge(CEdgePose2D(79, 80, Eigen::Vector3d(-0.0102099, -0.978673, 4.74423), information, system));
	system.r_Add_Edge(CEdgePose2D(80, 81, Eigen::Vector3d(1.01265, 0.0192011, -0.00199527), information, system));
	system.r_Add_Edge(CEdgePose2D(81, 82, Eigen::Vector3d(0.994241, -0.0319085, -0.0197558), information, system));
	system.r_Add_Edge(CEdgePose2D(82, 83, Eigen::Vector3d(1.00925, 0.00590969, -0.0214812), information, system));
	system.r_Add_Edge(CEdgePose2D(83, 84, Eigen::Vector3d(-0.0184826, 1.03307, -4.72819), information, system));
	system.r_Add_Edge(CEdgePose2D(84, 85, Eigen::Vector3d(0.984696, 0.0196236, 0.00877518), information, system));
	system.r_Add_Edge(CEdgePose2D(85, 86, Eigen::Vector3d(0.993027, 0.010799, 0.0107298), information, system));
	system.r_Add_Edge(CEdgePose2D(86, 87, Eigen::Vector3d(0.992905, 0.0213607, 0.0110665), information, system));
	system.r_Add_Edge(CEdgePose2D(87, 88, Eigen::Vector3d(0.00121839, 1.04031, 1.53711), information, system));
	system.r_Add_Edge(CEdgePose2D(88, 89, Eigen::Vector3d(1.00767, -0.0150986, 0.0147958), information, system));
	system.r_Add_Edge(CEdgePose2D(89, 90, Eigen::Vector3d(1.01226, -0.00539061, 0.030011), information, system));
	system.r_Add_Edge(CEdgePose2D(90, 91, Eigen::Vector3d(1.03457, 0.00297329, -0.00901519), information, system));
	system.r_Add_Edge(CEdgePose2D(91, 92, Eigen::Vector3d(-0.0159521, 0.972423, 1.55259), information, system));
	system.r_Add_Edge(CEdgePose2D(92, 93, Eigen::Vector3d(0.990753, 0.0620248, -0.0146912), information, system));
	system.r_Add_Edge(CEdgePose2D(93, 94, Eigen::Vector3d(0.971423, 0.0142496, 0.000217408), information, system));
	system.r_Add_Edge(CEdgePose2D(94, 95, Eigen::Vector3d(1.02272, -0.0278824, 0.000365479), information, system));
	system.r_Add_Edge(CEdgePose2D(95, 96, Eigen::Vector3d(-0.0193242, 1.04934, 1.57253), information, system));
	system.r_Add_Edge(CEdgePose2D(96, 97, Eigen::Vector3d(1.03931, -0.0130887, 0.0113687), information, system));
	system.r_Add_Edge(CEdgePose2D(97, 98, Eigen::Vector3d(0.993004, 0.0393656, -0.0105709), information, system));
	system.r_Add_Edge(CEdgePose2D(98, 99, Eigen::Vector3d(1.03897, -0.0238964, -0.0173253), information, system));
	system.r_Add_Edge(CEdgePose2D(99, 100, Eigen::Vector3d(-0.985853, -0.00979836, -3.15198), information, system));
}

/**
 *	@page simpleexample Simple SLAM++ Example
 *
 *	This example shows how to implement a simple batch SLAM for offline scenarios.
 *	It is in \ref slam_simple_example/Main.cpp.
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
 *	typedef CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> CNonlinearSolverType;
 *	@endcode
 *
 *	Here, CNonlinearSolver_Lambda determines the specific implementation of the solver, lambda is
 *	the best at batch solving.
 *
 *	\section makesys Creating the System
 *
 *	Finally, all the types that are required are specified, and we can make them do the work:
 *
 *	@code
 *	CSystemType system;
 *	CNonlinearSolverType solver(system);
 *	@endcode
 *
 *	The system is empty, the solver is in its default state where it is instructed to optimize
 *	on-demand.
 *
 *	\section addedges Adding Edges
 *
 *	The solver is driven by inserting edges. To add a new edge to the system, one just does:
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
 *	system.r_Add_Edge(new_edge);
 *	// add edge to the system
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
 *	Finally, the edge itself is added to the system using the CFlatSystem::r_Add_Edge() function.
 *
 *	The last two bits are usually contracted to a single compact command:
 *
 *	@code
 *	system.r_Add_Edge(CEdgePose2D(0, 1,
 *		Eigen::Vector3d(1.02765, -0.00745597, 0.00483283), information, system));
 *	@endcode
 *
 *	\section optim Optimizing the System
 *
 *	By adding all the edges that are supposed to be in the system, a graph is constructed, and the
 *	vertices are initialized automatically (based on the edges' measurements). Since the measurements
 *	may be noisy, the vertices do not have correct values and not all the measurements are satisfied.
 *	We can see that by calling:
 *
 *	@code
 *	system.Plot2D("resultUnoptim.tga", plot_quality::plot_Printing); // plot in print quality
 *	@endcode
 *
 *	That draws a bitmap with vertices and edges in it, which we use to generate
 *	pictures in our papers. It should be apparent that the system is in a state of disarray,
 *	the manhattan blocks are not quite orthogonal.
 *
 *	To optimize the system, one just calls:
 *
 *	@code
 *	solver.Optimize();
 *	@endcode
 *
 *	And that executes by default up to five iterations of nonlinear solving (it can be changed
 *	by passing arguments to the \ref CNonlinearSolver_Lambda::Optimize() "Optimize()" function).
 *	To verify that the result is indeed optimized, one can do:
 *
 *	@code
 *	system.Plot2D("afterFirstStep.tga", plot_quality::plot_Printing); // plot in print quality
 *	@endcode
 *
 *	And now we see that everything is allright in blockworld.
 *
 *	\section more Advanced Topics
 *
 *	Some of the more advanced topics such as vertex initialization, incremental optimization
 *	or accessing the solution are covered in the \ref onlineexample "online example". To implement
 *	solver for a different problem than the ones already covered by SLAM++, see the \ref ownsolvers
 *	"own solver implementation" tutorial.
 *
 */

/*
 *	end-of-file
 */
