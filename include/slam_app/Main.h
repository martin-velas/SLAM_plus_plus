/*
								+-----------------------------------+
								|                                   |
								|        ***  SLAM Main  ***        |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|              Main.h               |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SLAMPP_MAIN_INCLUDED
#define __SLAMPP_MAIN_INCLUDED

/**
 *	@file include/slam_app/Main.h
 *	@brief contains some of the common classes required in main() and the documentation pages
 *	@author -tHE SWINe-
 *	@date 2013-06-14
 */

#include "slam_app/Config.h"
#include "slam/ConfigSolvers.h" // need solver types for commandline parser

/**
 *	@mainpage SLAM++
 *
 *  @section intro_sec Introduction
 *
 *	SLAM++ is a fast nonlinear optimization package for solving sparse graph problems.
 *	It is very fast in both batch and incremental modes and offers highly efficient marginal
 *	covariance recovery.
 *
 *	@section install_sec Building
 *
 *	There is a CMakeFile. To be able to change the code and commit back
 *	to the svn, do an out-of-source build, like this:
 *
 *	@code{.f}
 *	$ mkdir slam
 *	$ cd slam
 *	$ svn checkout svn://svn.code.sf.net/p/slam-plus-plus/code/trunk .
 *	$ cd build
 *	$ cmake ..
 *	@endcode
 *
 *	This will configure the project using the default configuration. To change configuration, run <tt>cmake -i ..</tt>
 *	instead of the last line above (or at a later time, should a change in the configuration be needed).
 *	One interesting option is to specify the default linear solver. Supernodal CHOLMOD or block Cholesky are
 *	the fastest, CSparse is slightly slower and simplical CHOLMOD is the slowest.
 *	Another option is to enable GPU acceleration support, which currently applies to the Schur complement
 *	solver only (specify <tt>-us</tt> when running SLAM++; requires CUDA and CULA toolkits).
 *
 *	On Mac, you might want to configure your C and C++ compilers to be the GNU ones (e.g. from the
 *	MacPorts project) rather than Clang which does not support OpenMP nowadays and your code will be
 *	somewhat slower. You can do that by using:
 *
 *	@code{.sh}
 *	$ cmake -D CMAKE_C_COMPILER=/opt/local/bin/gcc-mp-6.3 -D CMAKE_CXX_COMPILER=/opt/local/bin/g++-mp-6.3 ..
 *	@endcode
 *
 *	Where you might want to change the version to the latest one that you have installed. But it will
 *	build and run correctly even without that.
 *
 *	@code{.sh} $ make @endcode
 *
 *	Will finish the building process. You can also use fast parallel build, like this:
 *
 *	@code{.sh}
 *	$ make -j
 *	@endcode
 *
 *	This seemed to work on Ubuntu (x64 or x86 alike), and on Mac.
 *
 *	In case you are working on Windows, there is a pre-configured workspace inside
 *	the <tt>build15</tt> folder in the source code distribution. In the previous CMake
 *	versions, there were varius issues with Windows. Since CMake 3.x, you can easily
 *	generate Visual Studio workspace using cmake-gui, that will work just as well.
 *	It only does not have project grouping, which is a very minor cosmetic issue.
 *	We tested this with Visual Studio 2008, 2010, 2012, 2013, 2015 and 2017.
 *
 *	There is an internal compiler error in Visual Studio 2012 if Eigen complex BLAS
 *	is enabled (added an extra CMake option to enable it, as it is usually not needed).
 *
 *	If you have trouble building or during your development, you can visit
 *	<a href="https://sourceforge.net/p/slam-plus-plus/wiki/Bug%20Atlas" onclick="window.open(this.href); return false;">bug atlas</a>
 *	to see descriptions of the commonly observed errors and ways to fix them.
 *
 *	You should now be able to run by typing:
 *
 *	@code{.sh} $ ../bin/slam_plus_plus --help @endcode
 *
 *	Which should reveal something like this (also depending on the actual configuration):
 *
 *	\htmlonly
 *	<div class="fragment">
 *	<div class="line">General use:																				</div>
 *	<div class="line">    ./slam_plus_plus -i &lt;filename&gt; --no-detailed-timing								</div>
 *	<div class="line">																							</div>
 *	<div class="line">To run the pose-only datasets more quickly:												</div>
 *	<div class="line">    ./slam_plus_plus -i &lt;filename&gt; --pose-only --no-detailed-timing					</div>
 *	<div class="line">																							</div>
 *	<div class="line">To run incrementally:																		</div>
 *	<div class="line">    ./slam_plus_plus -nsp &lt;optimize-each-N-verts&gt; -fL -i &lt;filename&gt; --no-detailed-timing	</div>
 *	<div class="line">																							</div>
 *	<div class="line">This generates initial.txt and initial.tga, a description and image of the				</div>
 *	<div class="line">system before the final optimization, and solution.txt and solution.tga, a				</div>
 *	<div class="line">description and image of the final optimized system (unless --no-solution					</div>
 *	<div class="line">or --no-bitmaps are specified, respectively).												</div>
 *	<div class="line">																							</div>
 *	<div class="line">--help|-h         displays this help screen												</div>
 *	<div class="line">--verbose|-v      displays verbose output while running (may slow down,					</div>
 *	<div class="line">                  especially in windows and if running incrementally)						</div>
 *	<div class="line">--silent|-s       suppresses displaying verbose output									</div>
 *	<div class="line">--no-show|-ns     does not show output image (windows only)								</div>
 *	<div class="line">--no-commandline|-nc    does not echo command line										</div>
 *	<div class="line">--no-flags|-nf    does not show compiler flags											</div>
 *	<div class="line">--no-detailed-timing|-ndt    does not show detailed timing breakup (use this, lest		</div>
 *	<div class="line">                  you will get confused)													</div>
 *	<div class="line">--no-bitmaps|-nb  does not write bitmaps initial.tga and solution.tga (does not			</div>
 *	<div class="line">                  affect the text files)													</div>
 *	<div class="line">--no-solution|-ns does not write text files initial.txt and solution.txt					</div>
 *	<div class="line">--xz-plots|-xz    turns bitmaps initial.tga and solution.tga into the X-Z plane			</div>
 *	<div class="line">--dump-system-matrix|-dsm    writes system matrix as system.mtx (matrix market)			</div>
 *	<div class="line">                  and system.bla (block layout) before optimization						</div>
 *	<div class="line">--pose-only|-po   enables optimisation for pose-only slam (will warn and ignore			</div>
 *	<div class="line">                  on datasets with landmarks (only the first 1000 lines checked,			</div>
 *	<div class="line">                  in case there are landmarks later, it would segfault))					</div>
 *	<div class="line">--use-old-code|-uogc    uses the old CSparse code (no block matrices in it)				</div>
 *	<div class="line">--a-solver|-A     uses A solver															</div>
 *	<div class="line">--lambda|-,\      uses lambda solver (default, preferred batch solver)					</div>
 *	<div class="line">--lambda-lm|-,\lm uses lambda solver with Levenberg Marquardt (default for BA)			</div>
 *	<div class="line">--lambda-dl|-,\dl uses lambda solver with Dogleg and fluid relinearization				</div>
 *	<div class="line">--l-solver|-L     uses L solver															</div>
 *	<div class="line">--fast-l|-fL      uses the new fast L solver (preferred incremental solver)				</div>
 *	<div class="line">--use-schur|-us   uses Schur complement to accelerate linear solving						</div>
 *	<div class="line">--do-marginals|-dm enables marginal covariance calculation (experimental)					</div>
 *	<div class="line">--infile|-i &lt;filename&gt;    specifies input file &lt;filename&gt;; it can cope with	</div>
 *	<div class="line">                  many file types and conventions											</div>
 *	<div class="line">--parse-lines-limit|-pll &lt;N&gt;    sets limit of lines read from the input file		</div>
 *	<div class="line">                  (handy for testing), note this does not set limit of vertices			</div>
 *	<div class="line">                  nor edges!																</div>
 *	<div class="line">--linear-solve-period|-lsp &lt;N&gt;    sets period for incrementally running linear		</div>
 *	<div class="line">                  solver (default 0: disabled)											</div>
 *	<div class="line">--nonlinear-solve-period|-nsp &lt;N&gt;    sets period for incrementally running			</div>
 *	<div class="line">                  non-linear solver (default 0: disabled)									</div>
 *	<div class="line">--max-nonlinear-solve-iters|-mnsi &lt;N&gt;    sets maximal number of nonlinear			</div>
 *	<div class="line">                  solver iterations (default 10)											</div>
 *	<div class="line">--nonlinear-solve-error-thresh|-nset &lt;f&gt;    sets nonlinear solve error threshold	</div>
 *	<div class="line">                  (default 20)															</div>
 *	<div class="line">--max-final-nonlinear-solve-iters|-mfnsi &lt;N&gt;    sets maximal number of final		</div>
 *	<div class="line">                  optimization iterations (default 5)										</div>
 *	<div class="line">--final-nonlinear-solve-error-thresh|-fnset &lt;f&gt;    sets final nonlinear solve		</div>
 *	<div class="line">                  error threshold (default 0.01)											</div>
 *	<div class="line">--run-matrix-benchmarks|-rmb &lt;benchmark-name&gt; &lt;benchmark-type&gt;    runs block	</div>
 *	<div class="line">                  matrix benchmarks (benchmark-name is name of a folder with				</div>
 *	<div class="line">                  UFLSMC benchmark, benchmark-type is one of alloc, factor, all)			</div>
 *	<div class="line">--run-matrix-unit-tests|-rmut    runs block matrix unit tests								</div>
 *	<div class="line">--omp-set-num-threads &lt;N&gt;    sets number of threads to N (default is to use as many	</div>
 *	<div class="line">                  threads as there are CPU cores)											</div>
 *	<div class="line">--omp-set-dynamic &lt;N&gt;    enables dynamic adjustment of the number of threads is N is	</div>
 *	<div class="line">                  nonzero, disables if zero (disabled by default)							</div>
 *	<div class="line">--dogleg-step-size|-dlss <f>    sets the initial dogleg solver step size (default 2)		</div>
 *	<div class="line">--dogleg-all-batch|-dlabat    instructs the dogleg solver (disabled by default)			</div>
 *	<div class="line">--iBA-save-intermediates|-iBAsi	enables saving of intermediate solutions at				</div>
 *	<div class="line">                  every step in incremental BA (disabled by default)						</div>
 *	<div class="line">--iBA-save-matrices|-iBAsm	enables saving of system matrices at every step in			</div>
 *	<div class="line">                  incremental BA (disabled by default)									</div>
 *	</div>
 *	\endhtmlonly
 *
 *	@section use_sec Usage
 *
 *	The simplest use case is:
 *
 *	@code{.sh} $ ../bin/slam_plus_plus -i ../data/manhattanOlson3500.txt --pose-only @endcode
 *
 *	In case you want to do incremental processing, use:
 *
 *	@code{.sh} $ ../bin/slam_plus_plus -i ../data/manhattanOlson3500.txt -nsp 1 -fL --pose-only @endcode
 *
 *	Where <tt>nsp</tt> is nonlinear solve period (solve at each new vertex) and <tt>fL</tt> selects the
 *	fast incremental solver. To do Bundle Adjustment, one can:
 *
 *	@code{.sh} $ ../bin/slam_plus_plus -i ../data/venice791.g2o -us @endcode
 *
 *	Where <tt>us</tt> enables the use of the Schur complement.
 *
 *	@section next_sec Next Steps
 *
 *	Some basic topics are covered here:
 *	* \ref polyfitexample shows very simple polynomial fitting with SLAM++
 *	* \ref simpleexample shows how to use SLAM++ for batch solving
 *	* \ref onlineexample shows how to implement a simple online solver
 *	* \ref rot3d discusses different representations of rotations in 3D
 *	* \ref baifaceexample shows how to easily wrap SLAM++ with a simple interface
 *
 *	@section adv_sec Advanced
 *
 *	Some advanced topics are covered in the following pages:
 *	* \ref facades shows how to access edges and vertices stored in the optimized graph
 *	* \ref ownsolvers shows how to implement solver for a custom problem
 *	* \ref constvertices shows how to use constant vertices
 *	* \ref unaryfactors shows how to use unary factors
 */

/**
 *	@page ownsolvers Implementing Own Solvers
 *
 *	In order to implement solver for your problem of desired dimensionality and
 *	with appropriate jacobians, one needs to execute the following steps:
 *
 *	* implement a new parser code if required (the parser now only processes 2D and 3D types)
 *		* note that this might not be required if you are going to use your own means of passing data to the solver
 *	* create a new header file for the new code
 *	* implement new vertex types (contain "smart" plus code)
 *	* implement new edge types (contain jacobians code)
 *	* implement edge type traits to connect to parse loop if using the builtin parser, or implement your own parse loop
 *		* implement vertex type traits (only if using the builtin parser and vertex initialization is required)
 *	* write specialization of a system that would use your new types
 *	* add a new branch in Main.cpp to call your new code, or call the optimizer yourself
 *
 *	@section sec0 Implement a new parser code
 *
 *	In case the edge types can not be parsed using the existing code, you need to implement parser for
 *	them. Note that this might not be required if you are going to use your own means of passing data
 *	to the solver (such as e.g. from a video stream if implementing visual SLAM).
 *
 *	Nevertheless if you are going to use the built-in parser, you need to implement storage structure
 *	for your new edge, let's start by copying the CParseEntity_XYT_Edge_2D type that can be found in Parser.h
 *	in the class CParserBase (copy it just below it):
 *
 *	@code{.cpp}
 *	struct CParseEntity_XYT_Edge_2D : public CParseEntity { // change name here (and also below)
 *	    size_t m_n_node_0;
 *	    size_t m_n_node_1;
 *	    Eigen::Vector3d m_v_delta;
 *	    Eigen::Matrix3d m_t_inv_sigma; // don't change names of variables unless you must, just change the types from 3d to whatever you need
 *
 *	    inline CParseEntity_XYT_Edge_2D(size_t n_node_0, size_t n_node_1,
 *	        double f_delta_x, double f_delta_y, double f_delta_theta,
 *	        const double *p_upper_matrix_3x3) // you can change the parameters as you wish
 *	        :m_n_node_0(n_node_0), m_n_node_1(n_node_1), // fill the indices of edge vertices (these should be zero-based)
 *	        m_v_delta(f_delta_x, f_delta_y, f_delta_theta) // fill the measurement vector
 *	    {
 *	        m_t_inv_sigma <<
 *	            p_upper_matrix_3x3[0], p_upper_matrix_3x3[1], p_upper_matrix_3x3[2],
 *	            p_upper_matrix_3x3[1], p_upper_matrix_3x3[3], p_upper_matrix_3x3[4],
 *	            p_upper_matrix_3x3[2], p_upper_matrix_3x3[4], p_upper_matrix_3x3[5]; // fill the inverse sigma matrix
 *	    }
 *
 *	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // this is imposed by the use of eigen, copy it
 *	};
 *	@endcode
 *
 *	Note that parsed vertex types are implemented in similar fashion, just start
 *	by copying e.g. CParserBase::TVertexXYZ and modifying it to suit your needs.
 *
 *	Once you have parsed type, you need to implement code to parse it. Start by making a copy
 *	of CIgnoreParsePrimitive in ParsePrimitives.h:
 *
 *	@code{.cpp}
 *	class MyNewEdgeParsePrimitive {
 *	public:
 *		static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
 *			int n_assigned_id) // throw(std::bad_alloc)
 *		{
 *			r_token_name_map["MY-NEW-TOKEN-NAME-IN-UPPERCASE"] = n_assigned_id;
 *			r_token_name_map["MY-OTHER-NEW-TOKEN-NAME-IN-UPPERCASE"] = n_assigned_id;
 *			// associate all the tokens you want to handle in this parse primitive
 *		}
 *
 *		template <class _TyParseLoop>
 *		static bool Parse_and_Dispatch(size_t n_line_no, const std::string &r_s_line,
 *			const std::string &UNUSED(r_s_token), _TyParseLoop &r_parse_loop)
 *		{
 *			// here, a primitive of type r_s_token should be parsed from r_s_line
 *			// and if successful, passed to r_parse_loop by calling InitializeVertex()
 *			// for vertex types or AppendSystem() for edge types
 *
 *			return true;
 *		}
 *	};
 *	@endcode
 *
 *	You will need to associate the token names inside EnumerateTokens(). Note that the names should
 *	all point to the same primitive (e.g. it is ok to put "EDGE:SE2" and "EDGE2", but not so ok to
 *	put "EDGE" and "LANDMARK" together as these carry different data). If handling of more types
 *	is required, more parse primitives need to be written.
 *
 *	In the second function, you need to implement the actual parsing and passing of the parsed
 *	structure to the parse loop for processing. Add your custom code for the parsing itself:
 *
 *	@code{.cpp}
 *	template <class _TyParseLoop>
 *	static bool Parse_and_Dispatch(size_t n_line_no, const std::string &r_s_line,
 *		const std::string &UNUSED(r_s_token), _TyParseLoop &r_parse_loop)
 *	{
 *	    int p_pose_idx[2];
 *	    double p_measurement[3];
 *	    double p_matrix[6];
 *	    // data that are stored on the line (change the measurement and matrix dimensions as required)
 *
 *	    if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf", // modify the numbers of %lf to reflect number of numbers you need to read from the line
 *	       p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1, p_measurement + 2,
 *	       p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, // put all the matrix and measurement elements, in the order they come
 *	       p_matrix + 5) != 2 + 3 + 6) { // modify the condition
 *	        _ASSERTE(n_line_no < SIZE_MAX);
 *	        fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
 *	        return false;
 *	    }
 *	    // read the individual numbers
 *
 *	    CParseEntity_XYT_Edge_2D edge(p_pose_idx[0], p_pose_idx[1],
 *	        p_measurement[0], p_measurement[1], p_measurement[2], p_matrix); // this creates an object of type that you introduced to Parser.h (rename it)
 *	    // process the measurement
 *
 *	    r_parse_loop.AppendSystem(edge);
 *	    // append the measurement to the system, or something
 *
 *		return true;
 *	}
 *	@endcode
 *
 *	This adds handling of the new edge type. If you want to extend the code and make your new type
 *	one of the default handled types, you need to add your new type to the list of standard types
 *	at the end of ParsePrimitives.h:
 *
 *	@code{.cpp}
 *	typedef MakeTypelist_Safe((CEdge2DParsePrimitive, CLandmark2DParsePrimitive,
 *		CEdge3DParsePrimitive, CVertex2DParsePrimitive, CVertex3DParsePrimitive,
 *		CIgnoreParsePrimitive, MyNewEdgeParsePrimitive)) CStandardParsedPrimitives;
 *	@endcode
 *
 *	If you did that, you also need to change the CParserBase::CParserAdaptor class,
 *	back in Parser.h to include your new type(s). Create a variant of the
 *	CParserBase::CParserAdaptor::AppendSystem() function for every new edge type added
 *	(just copy-paste and rename), or a variant of CParserBase::CParserAdaptor::InitializeVertex()
 *	in case the persed type is a vertex. This also needs to be done in TDatasetPeeker in Main.cpp
 *	since it inherits from parser adaptor.
 *
 *	This is, however, not always necessary. If you just want to solve your specific problem,
 *	you can leave CStandardParsedPrimitives and CParserBase::CParserAdaptor as is.
 *
 *	@section sec1 Create a new header file for the new code
 *
 *	To accomplish the second step, create a new file, it should be in the include/slam folder
 *	and it should be named Something_Types.h, where "Something" is replaced by something that makes
 *	more sense (e.g. SE2 types are stored in SE2_Types.h). The file should be blank and it should contain
 *	the following include directives:
 *
 *	@code{.cpp}
 *	//   @file include/slam/Something_Types.h // change this to your name
 *	//   @author <your name here>
 *	//   @date 2012
 *	//   @brief <brief description of what types are being defined here and what problem it can solve>
 *
 *	#ifndef __SOMETHING_TYPES_INCLUDED
 *	#define __SOMETHING_TYPES_INCLUDED // change this to your name
 *
 *	#include "slam/BaseTypes.h" // this needs to be included for base type implementations, required by the solvers
 *	#include "slam/SE2_Types.h" // include this as well to have some common classes (type traits)
 *
 *	// you can start implementing stuff here
 *
 *	#endif // __SOMETHING_TYPES_INCLUDED
 *	@endcode
 *
 *	Note that modifications to CMakeFiles.txt are not required since it is only a header file.
 *
 *	Also note that for SE2 and SE3 types there was the convention of making "xDSolverBase.h" file (such as 2DSolverBase.h and 3DSolverBase.h)
 *	with some common functions. This is not completely necessary as it was only required by the legacy code
 *	and this file is not really needed for anything anymore.
 *
 *	@section sec2 Implement new vertex types
 *
 *	In order to start implementing your types, it is best to begin by exploring of what
 *	is in slam/SE2_Types.h. Note the CVertexPose2D class:
 *
 *	@code{.cpp}
 *	class CVertexPose2D : public CBaseVertexImpl<CVertexPose2D, 3> { // this says to use base vertex implementation for class with name CVertexPose2D, while the vertex has 3 dimensions; this will generate member variable m_v_state ("member vector" state), which will be Eigen dense column vector with the given number of dimensions
 *	public:
 *	    __SE2_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, copy this
 *
 *	    inline CVertexPose2D() // copy this
 *	    {}
 *
 *	    inline CVertexPose2D(const Eigen::Vector3d &r_v_state) // copy this, change the dimension of the vector to appropriate
 *	        :CBaseVertexImpl<CVertexPose2D, 3>(r_v_state) // change the dimension here as well
 *	    {}
 *
 *	    inline CVertexPose2D(const CParserBase::TVertex2D &r_v_vertex) // copy this, change the dimension of the vector to appropriate
 *	        :CBaseVertexImpl<CVertexPose2D, 3>(r_v_vertex.m_v_position) // change the dimension here as well
 *	    {}
 *
 *	    inline void Operator_Plus(const Eigen::VectorXd &r_v_delta) // "smart" plus
 *	    {
 *	        m_v_state += r_v_delta.segment<3>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply +
 *	        m_v_state(2) = C2DJacobians::f_ClampAngle_2Pi(m_v_state(2)); // clamp angle
 *	    }
 *
 *	    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
 *	    {
 *	        m_v_state -= r_v_delta.segment<3>(m_n_order); // pick part of the delta vector, belonging to this vertex, apply -
 *	        m_v_state(2) = C2DJacobians::f_ClampAngle_2Pi(m_v_state(2)); // clamp angle
 *	    }
 *	};
 *	@endcode
 *
 *	Basically what you need to do is to change the name of the class (don't delete this one,
 *	copy it to your Something_Types.h and edit it there), set the number of dimensions and
 *	implement Operator_Plus(). You can leave operator minus empty, it is not really used,
 *	or at least do this:
 *
 *	@code{.cpp}
 *	    inline void Operator_Minus(const Eigen::VectorXd &r_v_delta) // "smart" minus
 *	    {
 *	        Operator_Plus(-r_v_delta); // call plus with negative delta, that should do the trick
 *	    }
 *	@endcode
 *
 *	That gives you your new shiny vertex type. Once you have that, you need to implement
 *	the edge type(s).
 *
 *	@section sec3 Implement new edge types
 *
 *	Now since the parser is ready, you can implement your edge type; let's begin by copying
 *	CEdgePose2D from SE2_Types.h to your Something_Types.h:
 *
 *	@code{.cpp}
 *	class CEdgePose2D : public CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D), 3> { // again, this tells that base implementation for base edge for type that will be called
 *	CEdgePose2D, and it will be an edge between two vertices of type CVertexPose2D, and the measurement will have 3 dimensions (in order to have hyperedges, you need to provide your own
 *	version of CBaseEdgeImpl, that is an advanced topic)
 *	public:
 *		typedef CBaseEdgeImpl<CEdgePose2D, MakeTypelist(CVertexPose2D, CVertexPose2D), 3> _TyBase; // base type
 *
 *	    class CRelative_to_Absolute_XYT_Initializer { // this is an object which is used to lazy initialize vertices (copy it)
 *	    protected:
 *	        const Eigen::Vector3d &m_r_v_pose1; // this is a reference to the first vertex state (change the dimensions if required)
 *	        const CParserBase::CParseEntity_XYT_Edge_2D &m_r_edge; // this is a reference to parser entity, containing the current edge (change the type to your type)
 *
 *	    public:
 *	        inline CRelative_to_Absolute_XYT_Initializer(const Eigen::Vector3d &r_v_vertex1,
 *	            const CParserBase::CParseEntity_XYT_Edge_2D &r_edge) // just change the types, same as above
 *	            :m_r_v_pose1(r_v_vertex1), m_r_edge(r_edge)
 *	        {}
 *
 *	        inline operator CVertexPose2D() const // this function calculates initial prior from the state of the first vertex m_r_v_pose1 and from the edge measurement m_r_edge
 *	        {
 *	            Eigen::Vector3d v_pose2;
 *	            C2DJacobians::Relative_to_Absolute(m_r_v_pose1, m_r_edge.m_v_delta, v_pose2); // implement your own equation here
 *	            return CVertexPose2D(v_pose2);
 *	        }
 *	    };
 *
 *	public:
 *	    __SE2_TYPES_ALIGN_OPERATOR_NEW // imposed by the use of eigen, just copy this
 *
 *	    inline CEdgePose2D() // copy this
 *	    {}
 *
 *	    template <class CSystem>
 *	    CEdgePose2D(const CParserBase::CParseEntity_XYT_Edge_2D &r_t_edge, CSystem &r_system) // this is edge constructor how it is called in the parse loop; you need to change type of the edge
 *	        :_TyBase(r_t_edge.m_n_node_0,  r_t_edge.m_n_node_1, r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma) // this just calls the base edge implementation, you need to change types and maintain the parameters if
 *	        required (these are: index of first and of second vertex, the measurement vector and the inverse sigma matrix)
 *	    {
 *	        m_p_vertex0 = &r_system.template r_Get_Vertex<CVertexPose2D>(r_t_edge.m_n_node_0, CInitializeNullVertex<>());
 *	        m_p_vertex1 = &r_system.template r_Get_Vertex<CVertexPose2D>(r_t_edge.m_n_node_1, CRelative_to_Absolute_XYT_Initializer(m_p_vertex0->r_v_State(), r_t_edge)); // rename your initializer if required
 *	        // get vertices (initialize if required)
 *	        // the strange syntax with "template" is required by g++, otherwise gives "expected primary-expression before '>' token"
 *	    }
 *
 *	    inline void Calculate_Jacobians_Expectation_Error(Eigen::Matrix3d &r_t_jacobian0,
 *	        Eigen::Matrix3d &r_t_jacobian1, Eigen::Vector3d &r_v_expectation,
 *	        Eigen::Vector3d &r_v_error) const // change dimensionality of eigen types, if required
 *	    {
 *	        C2DJacobians::Absolute_to_Relative(m_p_vertex0->r_v_State(),
 *	            m_p_vertex1->r_v_State(), r_v_expectation, r_t_jacobian0, r_t_jacobian1); // write your jacobian calculation code here (vertex state vectors are inputs, the rest are the outputs)
 *	            that you need to fill)
 *	        // calculates the expectation and the jacobians
 *
 *	        r_v_error = m_v_measurement - r_v_expectation;
 *	        r_v_error(2) = C2DJacobians::f_ClampAngularError_2Pi(r_v_error(2)); // write your error calculation code here
 *	        // calculates error (possibly re-calculates, if running A-SLAM)
 *	    }
 *
 *	    inline double f_Chi_Squared_Error() const // this function should mostly work as is, you just need to change dimensions of the vectors and matrices
 *	    {
 *	        Eigen::Matrix3d p_jacobi[2];
 *	        Eigen::Vector3d v_expectation, v_error;
 *	        Calculate_Jacobians_Expectation_Error(p_jacobi[0], p_jacobi[1], v_expectation, v_error);
 *	        // calculates the expectation, error and the jacobians
 *
 *	        return (v_error.transpose() * m_t_sigma_inv).dot(v_error); // ||z_i - h_i(O_i)||^2 lambda_i
 *	    }
 *	};
 *	@endcode
 *
 *	Note that there is \ref CBaseEdgeImpl::CInitializeNullVertex which initializes a vertex state to a null
 *	vector, and also \ref CBaseEdgeImpl::CInitializeVertex_Disallow which does not allow a vertex being
 *	initialized by and edge and will throw a <tt>std::runtime_error</tt> exception if the vertex in question
 *	does not yet exist in the system. Also, if you want to initialize all the vertices to null, you don't have
 *	to write the code for that, but just call the <tt>CBaseEdgeImpl</tt> constructor with the
 *	\ref CBaseEdge::null_initialize_vertices tag. Alternately if the vertices cannot be meaningfully initialized
 *	by the edge (such as e.g. in bundle adjustment) and you want exceptions thrown automatically, use the
 *	\ref CBaseEdge::explicitly_initialized_vertices tag. The constructors in \ref CBaseEdgeImpl also support
 *	specifying vertices not to be auto-initialized or not to be checked for existence (for unary edges using
 *	a bool flag, for binary edges using a bit field and for hyperedges using a typelist of \ref fbs_ut::CCTSize
 *	indices).
 *
 *	@section sec4 Implement edge type traits to connect to parse loop
 *
 *	This chapter only applies if using the built-in parser (otherwise see the \ref sec5 "next chapter").
 *
 *	And that is it, you have an edge! At this point, it is still a little bit to do. In case
 *	you will be using the built-in parser, you need to hook up the edges with the parse loop.
 *	For every edge type you've created, you need to write trait class specialization. These
 *	traits help the parse loop decide which edge type to create from which parsed structure
 *	(or whether to ignore / fail):
 *
 *	@code{.cpp}
 *	template <class CParsedStructure>
 *	class CMyNewEdgeTraits { // replace "MyNew" with something (e.g. SE2 types have CSE2EdgeTraits)
 *	    typedef CFailOnEdgeType _TyEdge; // it should fail on unknown edge types
 *	    static const char *p_s_Reason()
 *	    { return "unknown edge type occured"; }
 *	};
 *
 *	template <>
 *	class CMyNewEdgeTraits<CParserBase::TOdometry2D> { // this is a trait for *parsed* type CParserBase::TOdometry2D
 *	public:
 *	    typedef CEdgePose2D _TyEdge; // here we say to create CEdgePose2D type
 *	};
 *
 *	template <>
 *	class CSE2OnlyPoseEdgeTraits<CParserBase::TLandmark2D> { // an example, don't write "CSE2OnlyPoseEdgeTraits", that is already defined
 *	public:
 *	    typedef CFailOnEdgeType _TyEdge; // it should fail on this edge type
 *
 *	    static const char *p_s_Reason()
 *	    {
 *	        return "landmark edges not permitted in pose-only solver"; // tell us why
 *	    }
 *	};
 *	@endcode
 *
 *	You need to fill traits with all the types you want to permit in the system. You can add
 *	also the edges which you don't want to handle to provide more meaningful error messages
 *	than "unknown edge type occured", but it is not mandatory.
 *
 *	If you require vertex initialization, you need to implement vertex traits in pretty
 *	much the same fashion. Otherwise, CIgnoreAllVertexTraits can be used.
 *
 *	With all that, the parser knows how to parse your new data types, the parse loop knows
 *	how to redirect those to the system and how to call the optimizer.
 *
 *	@section sec5 Implement your own parse loop
 *
 *	If you don't want to use
 *	the built-in parser, look inside ParseLoop.h and see how the edges are passed to the
 *	system for optimization. You need to write your own code that does the same. Basically
 *	you need to:
 *
 *	@code{.cpp}
 *	while(have more edges) {
 *	    TParsedType edge_in;
 *	    // fill this with the new measurement somehow
 *
 *	    CYourEdgeType &r_internal_edge_rep = system.r_Add_Edge(CYourEdgeType(edge_in, system));
 *	    // add the edge to the system ("convert" parsed edge to internal representation)
 *
 *	    solver.Incremental_Step(r_internal_edge_rep);
 *	    // call solver with the new edge
 *	}
 *	@endcode
 *
 *	Note that at this point you don't have <tt>system</tt> and you don't have <tt>solver</tt>. The next chapters
 *	contain information on where to get those.
 *
 *	@section sec6 Write specialization of a system
 *
 *	Once you have that, finally we need to make a solver to solve using your jacobians and
 *	system to contain your edges and vertices. You write:
 *
 *	@code{.cpp}
 *	typedef MakeTypelist_Safe((CVertexPose2D, CVertexLandmark2D)) TVertexTypelist; // just put your vertex types in the list (note the double parentheses are required)
 *	typedef MakeTypelist_Safe((CEdgePose2D, CEdgePoseLandmark2D)) TEdgeTypelist; // just put your edge types in the list (note the double parentheses are required)
 *	typedef CFlatSystem<CBaseVertex, TVertexTypelist,
 *		CBaseEdge, TEdgeTypelist> CSystemType; // the Base types can be your types in case you are not using CBaseSEVertexImpl / CBaseSEEdgeImpl
 *	// make a system permitting SE(2) vertex and edge types
 *	@endcode
 *
 *	@section sec7 Calling the new system
 *
 *	@code{.cpp}
 *	CSystemType system;
 *	// declare the system
 *
 *	typedef CLinearSolver_CSparse CLinearSolverType; // use CSparse as linear solver (or any other)
 *	CLinearSolverType linear_solver;
 *	// you need a linear solver
 *
 *	typedef CNonlinearSolver_Lambda CNonlinearSolverType; // or use A, these two are rather stable and we're confident they will give correct results
 *	typedef CNonlinearSolverType<CSystemType, CLinearSolverType> CSpecializedNonlinearSolverType;
 *	CSpecializedNonlinearSolverType nonlinear_solver(system, n_linear_solve_each_n_steps,
 *		n_nonlinear_solve_each_n_steps, n_max_nonlinear_solve_iteration_num,
 *		f_nonlinear_solve_error_threshold, b_verbose, linear_solver); // initialize nonlinear solver
 *	// prepare nonlinear solver
 *
 *	typedef CMyNewEdgeTraits CEdgeTraitsType; // use any edge traits appropriate
 *	typedef CIgnoreAllVertexTraits CVertexTraitsType; // use any vertex traits appropriate
 *	typedef CParseLoop<CSystemType, CSpecializedNonlinearSolverType,
 *	    CEdgeTraitsType, CVertexTraitsType> CSpecializedParseLoopType;
 *	CSpecializedParseLoopType parse_loop(system, nonlinear_solver);
 *	// prepare parse loop (using the default vertex traits)
 *
 *	typedef MakeTypelist_Safe((MyNewEdgeParsePrimitive,
 *		MyNewVertexParsePrimitives)) CMyParsedPrimitives; // list of the new parsed primitives
 *	typedef typename CConcatTypelist<CMyParsedPrimitives,
 *		CStandardParsedPrimitives>::_TyResult CParsePrimitivesList; // list of all parsed primitives
 *	// this is only required if you did not modify CStandardParsedPrimitives already
 *	// also, for handling your custom problems, CMyParsedPrimitives might be just enough (no concatenation required)
 *
 *	CParserTemplate<CSpecializedParseLoopType, CParsePrimitivesList> p;
 *	if(!p.Parse(p_s_input_file, &parse_loop, n_max_lines_to_process)) {
 *	    fprintf(stderr, "error: failed to parse input file\n");
 *	    exit(-1);
 *	}
 *	// run the parser
 *
 *	system.Plot2D("initial.tga");
 *	system.Dump("initial.txt");
 *	// save the solution
 *
 *	nonlinear_solver.Optimize(n_max_final_optimization_iteration_num, f_final_optimization_threshold);
 *	// perform the final optimization
 *
 *	system.Plot2D("solution.tga");
 *	system.Dump("solution.txt");
 *	// save the solution
 *	@endcode
 *
 *	And as they say, you have optimized.
 *
 *	@section sec8 Modifying Main.cpp to use your new code via commandline
 *
 *	In case you want to update the slam_plus_plus program with your new types then you don't have
 *	to write all this, you just find the variable b_10k_opts and follow it around, mirroring
 *	everything that is done for it - that should also include declaring new system type with your
 *	list of vertices and edges per every solver type you want to support (you might e.g. want to
 *	write b_bundle_adjustment).
 *
 *	@section sec9 Final Thoughts
 *
 *	There is kind of a lot of duplicate editing in vertex and edge types, some of it might be handled
 *	using some \#defines but i prefer to keep the code clean. It is not so much extra work, just use
 *	"find and replace all", making sure to tick "Case Sensitive" and "Whole Words Only".
 *
 *	If you run in any problems while implementing your SLAM, let me know (to help you and to improve
 *	this tutorial / the code).
 *
 */

/**
 *	@defgroup ubm Block Matrix
 *	@brief Entities related to sparse block matrices and sparse block BLAS.
 */

/**
 *	@defgroup linsolve Linear Solvers
 *	@brief Solvers of linear systems of equations.
 */

/**
 *	@defgroup nlsolve Nonlinear Solvers
 *	@brief Nonlinear Least Squares solvers.
 */

/**
 *	@defgroup parser Parser
 *	@brief Parsing related objects and the parser template.
 */

/**
 *	@defgroup covs Covariances
 *	@brief Support for calculating covariances of the estimate.
 */

/**
 *	@defgroup graph Graph
 *	@brief Core functionality related to optimizable graphs.
 */

/**
 *	@defgroup se2 SE(2)
 *	@brief Edges and vertices in the SE(2) group.
 *
 *	The Jacobians and pose composition functions are in \ref C2DJacobians.
 *	The default pose vertex is \ref CVertexPose2D, the default landmark is \ref CVertexLandmark2D
 *	and the default odometry function is \ref CEdgePose2D and the landmark observation
 *	functions is \ref CEdgePoseLandmark2D.
 *
 */

/**
 *	@defgroup se3 SE(3)
 *	@brief Edges and vertices in the SE(3) group.
 *
 *	The Jacobians and pose composition functions are in \ref C3DJacobians.
 *	There is also a reusable class for representing a pose in 3D, \ref C3DJacobians::TSE3.
 *	The default pose vertex is \ref CVertexPose3D, the default landmark is \ref CVertexLandmark3D
 *	and the default odometry function is \ref CEdgePose3D and the landmark observation
 *	functions is \ref CEdgePoseLandmark3D.
 *
 *	There is also a description of \ref rot3d.
 *
 *	The original 3D code in SLAM++ was written without the use of Lie algebra and
 *	exponential / logarithm functions. The optimizer simply converts the poses from
 *	the internal representation to a vectorial form and optimizes that. Since the vectorial
 *	form is 3D position and axis-angle rotation (which is close to what se(3) actually looks
 *	like), it works. It is retained for backward compatibility. However, if implementing
 *	new 3D solver, derivatives based on SE(3) and se(3) should be used (this feature will
 *	be aviailable in SLAM++ 3.0).
 *
 *	To represent a coordinate frame in SE(3), one can use \ref C3DJacobians::TSE3.
 */

/**
 *	@defgroup sim3 Sim(3)
 *	@brief Edges and vertices in the Sim(3) group.
 *
 *	The Jacobians and pose composition functions are in \ref CSim3Jacobians.
 *	There is also a reusable class for representing a pose in 3D, \ref CSim3Jacobians::TSim3.
 *	The default camera pose vertex is \ref CVertexCamSim3, the default landmark vertex is
 *	\ref CVertexXYZ. The matching observation function is \ref CEdgeP2C_XYZ_Sim3_G. In our
 *	2015 ACRA paper "The Effect of Different Parameterisations in Incremental Structure
 *	from Motion", different parameterizations are described and those are all availavle in
 *	\ref include/slam/Sim3_Types.h.
 *
 *	There is also a description of \ref rot3d.
 *
 *	To represent a coordinate frame in Sim(3), one can use \ref CSim3Jacobians::TSim3.
 *	Note that it is similar to the representation used in the TooN library by Tom Drummond
 *	and is slightly different from Hauke Strasdat's Sophus library. The following simple test
 *	can be used for disambiguating the representations:
 *
 *	@code
 *	Eigen::Matrix4d T;
 *	T <<  0.6928,       0,  0.4000,  2.0000,
 *			   0,  0.8000,       0,  2.0000,
 *		 -0.4000,       0,  0.6928,  2.0000,
 *			   0,       0,       0,  1.0000;
 *	// a pose as a matrix; note that the rotation part has scale equal to 0.8
 *
 *	Eigen::Vector7d v_tRs;
 *	v_tRs.head<3>() = T.topRightCorner<3, 1>(); // translation
 *	v_tRs(6) = T.topLeftCorner<3, 3>().norm() / sqrt(3.0); // scale
 *	Eigen::Vector3d v_rot;
 *	C3DJacobians::Quat_to_AxisAngle(Eigen::Quaterniond(T.topLeftCorner<3, 3>() / v_tRs(6)).normalized(),
 *		v_rot); // rotation
 *	v_tRs.segment<3>(3) = v_rot;
 *	// convert the pose to a translation-rotation-scale vector
 *
 *	std::cout << "T =" << std::endl << T << std::endl << std::endl;
 *	std::cout << "v_tRs =" << std::endl << v_tRs << std::endl << std::endl;
 *
 *	CSim3Jacobians::TSim3 t_sim3(v_tRs, CSim3Jacobians::TSim3::from_tRs_vector);
 *	Eigen::Vector7d v_log = t_sim3.v_Log();
 *	// calculate the logarithmic map
 *
 *	std::cout << "v_log =" << std::endl << v_log << std::endl;
 *	@endcode
 *
 *	In SLAM++, it gives the following result:
 *
 *	@code
 *	T =
 *	    0.6928      0       0.4      2
 *			 0    0.8         0      2
 *		  -0.4      0    0.6928      2
 *			 0      0         0      1
 *
 *	v_tRs =
 *			   2
 *			   2
 *			   2
 *			   0
 *		0.523608
 *			   0
 *		0.799988
 *
 *	v_log =
 *		  1.62293
 *		  2.23145
 *		  2.74863
 *				0
 *		 0.523608
 *				0
 *		-0.223158
 *	@endcode
 *
 *	Where <tt>T</tt> is a transformation matrix in Sim(3), <tt>v_tRs</tt> is a 7D vector
 *	containing translation (3D), rotation as axis-angle (3D) and scle (1D). Finally,
 *	<tt>v_log</tt> is the corresponding element of \f$\mathfrak{sim}(3)\f$ obtained using
 *	the logarithmic map.
 *
 */

/**
 *	@defgroup ba_group Bundle Adjustment
 *	@brief Edges and vertices for Bundle Adjustment.
 *
 *	The Jacobians and pose composition functions are in \ref CBAJacobians. The default
 *	camera pose vertex is \ref CVertexCam, the default landmark vertex is \ref CVertexXYZ.
 *	The matching observation function is \ref CEdgeP2C3D.
 *
 *	There is a description of \ref rot3d.
 */

/**
 *	@defgroup geom Geometry
 *	@brief Utilities for geometric calculations in 3D.
 */

/**
 *	@brief peeks at the dataset to detect what's inside
 */
struct TDatasetPeeker : public CParserBase::CParserAdaptor {
	bool b_has_odometry; /**< @brief set, if 2D odometry token was found in the file */
	bool b_has_landmark; /**< @brief set, if 2D landmark token was found in the file */
	bool b_has_edge2d; /**< @brief set, if 2D edge (odometry) token was found in the file */
	bool b_has_vertex; /**< @brief set, if 2D vertex token was found in the file */
	bool b_has_edge3d; /**< @brief set, if 3D edge (odometry) token was found in the file */
	bool b_has_vertex3d; /**< @brief set, if 3D vertex token was found in the file */
	bool b_has_ba; /**< @brief set, if BA token was found in the file */
	bool b_has_ba_stereo; /**< @brief set, if BA token was found in the file */
	bool b_has_ba_intrinsics; /**< @brief set, if BA token was found in the file and also intrinsic camera params */
	bool b_has_spheron; /**< @brief set, if Spheron BA token was found in the file */
	bool b_has_rocv; /**< @brief set, if ROCV token was found in the file */

	/**
	 *	@brief peeks at the dataset
	 */
	TDatasetPeeker(const char *p_s_input_file, size_t n_max_lines_to_process = 0)
		:b_has_odometry(false), b_has_landmark(false), b_has_edge2d(false),
		b_has_vertex(false), b_has_edge3d(false), b_has_vertex3d(false), b_has_ba(false),
		b_has_ba_stereo(false), b_has_ba_intrinsics(false), b_has_spheron(false), b_has_rocv(false)
	{
		CStandardParser p;
		if(!p.Parse(p_s_input_file, *this, (n_max_lines_to_process > 0)?
		   n_max_lines_to_process : 1000)) {
			fprintf(stderr, "warning: failed to peek-parse input file\n");
			/*b_has_odometry = true;
			b_has_landmark = true;
			b_has_edge2d = true;
			b_has_vertex = true;*/
			// don't do that, it will fail anyway and this only makes up for the --pose-only error
			// which will confuse everyone in this case
		}
	}

	/**
	 *	@brief appends the system with an odometry measurement
	 *	@param[in] r_t_odo is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TEdge2D &UNUSED(r_t_odo))
	{
		b_has_odometry = true;
	}

	/**
	 *	@brief appends the system with a landmark measurement
	 *	@param[in] r_t_landmark is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TLandmark2D_RB &UNUSED(r_t_landmark))
	{
		b_has_landmark = true;
	}

	/**
	 *	@brief appends the system with a landmark measurement
	 *	@param[in] r_t_landmark is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TLandmark2D_XY &UNUSED(r_t_landmark))
	{
		b_has_landmark = true;
	}

	/**
	 *	@brief appends the system with vertex position
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertex2D &UNUSED(r_t_vertex))
	{
		b_has_vertex = true;
	}

	/**
	 *	@brief appends the system with an odometry measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TEdge3D &UNUSED(r_t_edge))
	{
		b_has_edge3d = true;
	}

	/**
	 *	@brief appends the system with an landmark 3d xyz measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TLandmark3D_XYZ &UNUSED(r_t_edge))
	{
		b_has_landmark = true;
	}

	/**
	 *	@brief appends the system with a projection measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TEdgeSpheronXYZ &UNUSED(r_t_edge))
	{
		b_has_spheron = true;
	}

	/**
	 *	@brief appends the system with vertex position
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertex3D &UNUSED(r_t_vertex))
	{
		b_has_vertex3d = true;
	}

	/**
	 *	@brief appends the system with vertex position
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertexXYZ &UNUSED(r_t_vertex))
	{
		b_has_ba = true;
	}

	/**
	 *	@brief appends the system with camera position and parameters
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertexCam3D &UNUSED(r_t_vertex))
	{
		b_has_ba = true;
	}

	/**
	 *	@brief appends the system with intrinsics
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertexIntrinsics &UNUSED(r_t_vertex))
	{
		b_has_ba_intrinsics = true;
	}

	/**
	 *	@brief appends the system with camera position and parameters
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertexSCam3D &UNUSED(r_t_vertex))
	{
		b_has_ba_stereo = true;
	}

	/**
	 *	@brief appends the system with camera position and parameters
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertexSpheron &UNUSED(r_t_vertex))
	{
		b_has_spheron = true;
	}

	/**
	 *	@brief appends the system with a projection measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TEdgeP2C3D &UNUSED(r_t_edge))
	{
		b_has_ba = true;
	}

	/**
	 *	@brief appends the system with a projection measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TEdgeP2CI3D &UNUSED(r_t_edge))
	{
		b_has_ba_intrinsics = true;
	}

	/**
	 *	@brief appends the system with a camera measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TEdgeP2SC3D &UNUSED(r_t_edge))
	{
		b_has_ba_stereo = true;
	}

	/**
	 *	@brief appends the system with a reference 3D pose
	 *	@param[in] r_t_vertex is the vertex to be appended
	 *	@note The vertices can be ignored in most of the solvers.
	 */
	virtual void InitializeVertex(const CParserBase::TVertex3D_Reference &UNUSED(r_t_vertex))
	{
		b_has_rocv = true;
	}

	/**
	 *	@brief appends the system with a delta-time edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TROCV_DeltaTimeEdge &UNUSED(r_t_edge))
	{
		b_has_rocv = true;
	}

	/**
	 *	@brief appends the system with a range measurement edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TROCV_RangeEdge &UNUSED(r_t_edge))
	{
		b_has_rocv = true;
	}

	/**
	 *	@brief appends the system with a range measurement edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	virtual void AppendSystem(const CParserBase::TUnaryFactor3D &UNUSED(r_t_edge))
	{
		// no vote, could be used in many situations
	}
};

//#include "slam/Marginals.h"
//#include "slam/IncrementalPolicy.h"

#include "slam_app/IncBAParsePrimitives.h" // a very small header, includes nothing

template <class CParseLoopType>
class CParseLoopTraits {
public:
	enum {
		b_saves_incremental_solutions = false
	};
};

template <class CSystem, class CNonlinearSolver, template <class> class CEdgeTraits,
	template <class> class CVertexTraits>
class CParseLoopTraits<CParseLoop_ConsistencyMarker<CSystem,
	CNonlinearSolver, CEdgeTraits, CVertexTraits> > {
public:
	enum {
		b_saves_incremental_solutions = true
	};
};

template <class CNonlinearSolverType>
class CNonlinearSolverDogLegTraits {
public:
	enum {
		b_is_dogleg_solver = false
	};
};

#if defined(__SE_TYPES_SUPPORT_LAMBDA_SOLVERS) && defined(__NONLINEAR_BLOCKY_SOLVER_LAMBDA_DOGLEG_INCLUDED)

template <class CSystem, class CLinearSolver,
	class CAMatrixBlockSizes, class CLambdaMatrixBlockSizes>
class CNonlinearSolverDogLegTraits<CNonlinearSolver_Lambda_DL<CSystem,
	CLinearSolver, CAMatrixBlockSizes, CLambdaMatrixBlockSizes> > {
public:
	enum {
		b_is_dogleg_solver = true
	};
};

#endif // __SE_TYPES_SUPPORT_LAMBDA_SOLVERS && __SE_TYPES_SUPPORT_LAMBDA_SOLVERS

/**
 *	@brief a helper template for running tests with given type configuration (to avoid repeating the same code)
 *
 *	@tparam CSystemType is system type (derived from CFlatSystem)
 *	@tparam CNonlinearSolverType is nonlinear solver template name
 *	@tparam CEdgeTraitsType is edge traits template name
 *	@tparam CVertexTraitsType is vertex traits template name
 *	@tparam CParseLoopType is parse loop template name
 *	@tparam CParsedPrimitives is a list of parsed primitives (default CStandardParsedPrimitives)
 */
template <class CSystemType,
	template <class, class/*, class*/> class CNonlinearSolverType,
	template <class> class CEdgeTraitsType,
	template <class> class CVertexTraitsType,
	template <class, class, template <class> class, template <class> class> class CParseLoopType,
	class CParsedPrimitives = CStandardParsedPrimitives>
class CTester {
public:
#ifdef __USE_NATIVE_CHOLESKY
	typedef typename CSystemType::_TyJacobianMatrixBlockList _TyAMatrixBlockSizes; /** @brief list of jacobian block sizes */
	typedef typename CSystemType::_TyHessianMatrixBlockList _TyLambdaMatrixBlockSizes; /**< @brief possible block matrices, found in lambda and L */

	typedef CLinearSolver_UberBlock<_TyLambdaMatrixBlockSizes> CLinearSolverType; /**< @brief linear solver type (native) */
#elif defined(__USE_CHOLMOD)
	typedef CLinearSolver_CholMod CLinearSolverType; /**< @brief linear solver type (CHOLMOD) */
#else // __USE_NATIVE_CHOLESKY
#ifdef __USE_CXSPARSE
	typedef CLinearSolver_CXSparse CLinearSolverType; /**< @brief linear solver type (CXSparse) */
#else // __USE_CXSPARSE
	typedef CLinearSolver_CSparse CLinearSolverType; /**< @brief linear solver type (CSparse) */
#endif // __USE_CXSPARSE
#endif // __USE_NATIVE_CHOLESKY

	/**
	 *	@brief utility for saving the system matrix sparsity pattern (handlers solvers which cannot export)
	 *
	 *	@tparam CSolverType is specialized nonlinear solver type
	 *	@tparam b_can_get_A is solver dump capability flag (extracted from solver traits)
	 *	@tparam b_can_get_Lambda is solver dump capability flag (extracted from solver traits)
	 *	@tparam b_can_get_R is solver dump capability flag (extracted from solver traits)
	 */
	template <class CSolverType, const bool b_can_get_A, const bool b_can_get_Lambda, const bool b_can_get_R>
	class CDumpSystemMatrix_SparsityPattern {
	public:
		/**
		 *	@brief saves the system matrix
		 *
		 *	@param[in] r_solver is solver to save the matrix
		 *	@param[in] p_s_name is output file name (will be saved in matrix market format)
		 *
		 *	@return Returns false.
		 */
		static bool Dump(const CSolverType &r_solver, const char *p_s_name = "system.mtx")
		{
			fprintf(stderr, "warning: the selected solver cannot save the system matrix\n");
			return false;
		}
	};

	/**
	 *	@brief utility for saving the system matrix sparsity pattern (specialization for lambda solvers)
	 *
	 *	@tparam CSolverType is specialized nonlinear solver type
	 *	@tparam b_can_get_A is solver dump capability flag (extracted from solver traits)
	 *	@tparam b_can_get_R is solver dump capability flag (extracted from solver traits)
	 */
	template <class CSolverType, const bool b_can_get_A, const bool b_can_get_R>
	class CDumpSystemMatrix_SparsityPattern<CSolverType, b_can_get_A, true, b_can_get_R> {
	public:
		/**
		 *	@brief saves the system matrix
		 *
		 *	@param[in] r_solver is solver to save the matrix
		 *	@param[in] p_s_name is output file name (will be saved in matrix market format)
		 *
		 *	@return Returns true on success, false on failure.
		 */
		static bool Dump(const CSolverType &r_solver, const char *p_s_name = "system_Lambda.tga")
		{
			const CUberBlockMatrix &r_sysmat = const_cast<CSolverType&>(r_solver).r_Lambda(); // LM modifies the solver, in order to remove the damping ... no good way around that
			cs *p_bs;
			if(!(p_bs = r_sysmat.p_BlockStructure_to_Sparse()))
				return false;
			bool b_result = CDebug::Dump_SparseMatrix_Subsample(p_s_name, p_bs, 0, 640, true);
			cs_spfree(p_bs);
			return b_result;
		}
	};

	/**
	 *	@brief utility for saving the system matrix sparsity pattern (specialization for R solvers)
	 *
	 *	@tparam CSolverType is specialized nonlinear solver type
	 *	@tparam b_can_get_A is solver dump capability flag (extracted from solver traits)
	 *	@tparam b_can_get_Lambda is solver dump capability flag (extracted from solver traits)
	 */
	template <class CSolverType, const bool b_can_get_A, const bool b_can_get_Lambda>
	class CDumpSystemMatrix_SparsityPattern<CSolverType, b_can_get_A, b_can_get_Lambda, true> {
	public:
		/**
		 *	@brief saves the system matrix
		 *
		 *	@param[in] r_solver is solver to save the matrix
		 *	@param[in] p_s_name is output file name (will be saved in matrix market format)
		 *
		 *	@return Returns true on success, false on failure.
		 */
		static bool Dump(const CSolverType &r_solver, const char *p_s_name = "system_R.tga")
		{
			const CUberBlockMatrix &r_sysmat = r_solver.r_R();
			cs *p_bs;
			if(!(p_bs = r_sysmat.p_BlockStructure_to_Sparse()))
				return false;
			bool b_result = CDebug::Dump_SparseMatrix_Subsample(p_s_name, p_bs, 0, 640);
			cs_spfree(p_bs);
			return b_result;
		}
	};

	/**
	 *	@brief utility for saving the system matrix sparsity pattern (specialization for A solvers)
	 *
	 *	@tparam CSolverType is specialized nonlinear solver type
	 *	@tparam b_can_get_Lambda is solver dump capability flag (extracted from solver traits)
	 *	@tparam b_can_get_R is solver dump capability flag (extracted from solver traits)
	 */
	template <class CSolverType, const bool b_can_get_Lambda, const bool b_can_get_R>
	class CDumpSystemMatrix_SparsityPattern<CSolverType, true, b_can_get_Lambda, b_can_get_R> {
	public:
		/**
		 *	@brief saves the system matrix
		 *
		 *	@param[in] r_solver is solver to save the matrix
		 *	@param[in] p_s_name is output file name (will be saved in matrix market format)
		 *
		 *	@return Returns true on success, false on failure.
		 */
		static bool Dump(const CSolverType &r_solver, const char *p_s_name = "system_A.tga")
		{
			const CUberBlockMatrix &r_sysmat = r_solver.r_A();
			cs *p_bs;
			if(!(p_bs = r_sysmat.p_BlockStructure_to_Sparse()))
				return false;
			bool b_result = CDebug::Dump_SparseMatrix_Subsample(p_s_name, p_bs, 0, 640);
			cs_spfree(p_bs);
			return b_result;
		}
	};

	/**
	 *	@brief saves the system matrix
	 *
	 *	@tparam b_can_dump is solver dump capability flag (extracted from solver traits)
	 *	@tparam CSolverType is specialized nonlinear solver type
	 *
	 *	@param[in] r_solver is solver to save the matrix
	 *	@param[in] p_s_name is output file name (will be saved in matrix market format)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <const bool b_can_dump, class CSolverType>
	static typename CEnableIf<b_can_dump, bool>::T DumpSystemMatrix(const CSolverType &r_solver,
		const char *p_s_name = "system.mtx")
	{
		return r_solver.Save_SystemMatrix_MM(p_s_name);
	}

	/**
	 *	@brief saves the system matrix (this only writes a warning that it cant save)
	 *
	 *	@tparam b_can_dump is solver dump capability flag (extracted from solver traits)
	 *	@tparam CSolverType is specialized nonlinear solver type
	 *
	 *	@param[in] r_solver is solver to save the matrix
	 *	@param[in] p_s_name is output file name (would be saved in matrix market format)
	 *
	 *	@return Returns false.
	 */
	template <const bool b_can_dump, class CSolverType>
	static typename CEnableIf<!b_can_dump, bool>::T DumpSystemMatrix(const CSolverType &r_solver,
		const char *p_s_name = "system.mtx")
	{
		fprintf(stderr, "warning: the selected solver cannot save the system matrix\n");
		return false;
	}

	template <const bool b_is_dogleg, class CSolverType, class TRunConfig>
	static typename CEnableIf<b_is_dogleg>::T
		Additional_SolverConfig(CSolverType &r_solver, const TRunConfig &r_t_args)
	{
		r_solver.Set_AllBatch(r_t_args.b_dogleg_all_batch);
		if(r_t_args.f_dogleg_step_size != -1)
			r_solver.Set_StepSize(r_t_args.f_dogleg_step_size);
		if(r_t_args.f_dogleg_step_threshold != -1)
			fprintf(stderr, "warning: value of dogleg step threshold (-dlst) ignored\n"); // the "normal" -fnset threshold is currently used instead
	}

	template <const bool b_is_dogleg, class CSolverType, class TRunConfig>
	static typename CEnableIf<!b_is_dogleg>::T
		Additional_SolverConfig(CSolverType &UNUSED(r_solver), const TRunConfig &r_t_args)
	{
		if(r_t_args.b_dogleg_all_batch)
			fprintf(stderr, "warning: value of dogleg all-batch flag (-dlabat) ignored\n");
		if(r_t_args.f_dogleg_step_size != -1)
			fprintf(stderr, "warning: value of dogleg step size (-dlss) ignored\n");
		if(r_t_args.f_dogleg_step_threshold != -1)
			fprintf(stderr, "warning: value of dogleg step threshold (-dlst) ignored\n"); // the "normal" -fnset threshold is currently used instead
		// this is used with a solver other than dogleg
	}

	template <const bool b_can_save_intermediates, class CParseLoopType_, class TRunConfig>
	static typename CEnableIf<b_can_save_intermediates>::T
		Additional_ParseLoopConfig(CParseLoopType_ &r_ploop, const TRunConfig &r_t_args)
	{
		r_ploop.Enable_Save_Solutions(r_t_args.b_inc_BA_save_intermediates);
		r_ploop.Enable_Save_SysMatrices(r_t_args.b_inc_BA_save_sysmats);
	}

	template <const bool b_can_save_intermediates, class CParseLoopType_, class TRunConfig>
	static typename CEnableIf<!b_can_save_intermediates>::T
		Additional_ParseLoopConfig(CParseLoopType_ &UNUSED(r_ploop), const TRunConfig &r_t_args)
	{
		if(r_t_args.b_inc_BA_save_intermediates)
			fprintf(stderr, "warning: saving intermediates (-iBAsi) not supported in the current setup\n");
		if(r_t_args.b_inc_BA_save_sysmats)
			fprintf(stderr, "warning: saving matrices (-iBAsm) not supported in the current setup\n");
		// this is used with a different parse loop type, will not save anything
	}

public:
	/**
	 *	@brief runs optimization of system stored in a file, measures timing
	 *	@tparam TRunConfig is command-line arguments type (avoids forward declaration)
	 *	@param[in] r_t_args is a structure containing command-line arguments
	 *	@return Returns true on success, false on failure.
	 */
	template <class TRunConfig>
	static inline bool Run_and_Shout(const TRunConfig &r_t_args)
	{
		CSystemType system;

		CLinearSolverType linear_solver;
		// prepare a linear solver

		//typedef typename CSystemType::_TyJacobianMatrixBlockList CMatrixTypelist; // use the default from the system
		typedef CNonlinearSolverType<CSystemType, CLinearSolverType/*, CMatrixTypelist*/> // have to supply CMatrixTypelist because in CTester template there is no default param for CNonlinearSolverType :(
			CSpecializedNonlinearSolverType;

		enum {
			b_solver_can_save = CSpecializedNonlinearSolverType::solver_ExportsJacobian ||
				CSpecializedNonlinearSolverType::solver_ExportsHessian ||
				CSpecializedNonlinearSolverType::solver_ExportsFactor,
			b_can_get_A = CSpecializedNonlinearSolverType::solver_ExportsJacobian &&
				!CSpecializedNonlinearSolverType::solver_ExportsHessian &&
				!CSpecializedNonlinearSolverType::solver_ExportsFactor, // only if we can't get any other matrix
			b_can_get_Lambda = CSpecializedNonlinearSolverType::solver_ExportsHessian, // any time
			b_can_get_R = CSpecializedNonlinearSolverType::solver_ExportsFactor &&
				!CSpecializedNonlinearSolverType::solver_ExportsHessian // only if we can't get lambda
		};
		_ASSERTE(((b_can_get_A)? 1 : 0) + ((b_can_get_Lambda)? 1 : 0) + ((b_can_get_R)? 1 : 0) <= 1);
		// see whether the solver can dump a matrix

		TIncrementalSolveSetting t_incremental_cfg(solve::linear,
			frequency::Every(r_t_args.n_linear_solve_each_n_steps),
			solve::nonlinear, frequency::Every(r_t_args.n_nonlinear_solve_each_n_steps),
			r_t_args.n_max_nonlinear_solve_iteration_num,
			r_t_args.f_nonlinear_solve_error_threshold);
		// t_odo - assemble this earlier, simplify Run_and_Shout() arglist

		TMarginalsComputationPolicy t_marginals_cfg((r_t_args.b_do_marginals)?
			marginals::do_calculate : marginals::do_not_calculate,
			frequency::Every((r_t_args.n_nonlinear_solve_each_n_steps)?
			r_t_args.n_nonlinear_solve_each_n_steps : r_t_args.n_linear_solve_each_n_steps),
			EBlockMatrixPart(mpart_Diagonal | mpart_LastColumn),
			EBlockMatrixPart(mpart_Diagonal | mpart_LastColumn), mpart_Nothing);
		// enable marginals, set the same freq as solver and set mpart_Diagonal | mpart_LastColumn
		// for both increment / relin, and nothing for miss

		CSpecializedNonlinearSolverType nonlinear_solver(system, t_incremental_cfg,
			t_marginals_cfg, r_t_args.b_verbose, linear_solver, r_t_args.b_use_schur);
		// prepare nonlinear solver

		enum {
			b_is_dogleg_solver = CNonlinearSolverDogLegTraits<CSpecializedNonlinearSolverType>::b_is_dogleg_solver
		};
		Additional_SolverConfig<b_is_dogleg_solver>(nonlinear_solver, r_t_args);
		// call Set_AllBatch(), Set_UpdateThreshold(), Set_StepSize(), according to the parameters

		typedef CParseLoopType<CSystemType, CSpecializedNonlinearSolverType,
			CEdgeTraitsType, CVertexTraitsType> CSpecializedParseLoop;
		CSpecializedParseLoop parse_loop(system, nonlinear_solver);
		// prepare parse loop

		enum {
			b_parse_loop_can_save = CParseLoopTraits<CSpecializedParseLoop>::b_saves_incremental_solutions
		};
		Additional_ParseLoopConfig<b_parse_loop_can_save>(parse_loop, r_t_args);
		// call Enable_Save_Solutions(), Enable_Save_SysMatrices() if set and if requested from commandline

		if(r_t_args.b_verbose && r_t_args.b_use_schur)
			printf("using Schur complement\n");
		if(r_t_args.b_verbose && t_marginals_cfg.b_calculate) {
			printf("marginals will be calculated (inc: 0x%x, relin: 0x%x, miss: 0x%x)\n",
				t_marginals_cfg.n_incremental_policy, t_marginals_cfg.n_relinearize_policy,
				t_marginals_cfg.n_cache_miss_policy);
			// print also which parts were selected for update (0xc = diagonal + last column, 0x8 = diagonal, 0x0 = nothing)
		}
		// verbose

		CTimer t;
		t.ResetTimer();
		// start meassuring time

		//printf("n_max_lines_to_process = %d\n", int(n_max_lines_to_process)); // debug
		//CParser p;
		CParserTemplate<CSpecializedParseLoop, CParsedPrimitives> p;
		if(!p.Parse(r_t_args.p_s_input_file, parse_loop, r_t_args.n_max_lines_to_process)) {
			fprintf(stderr, "error: failed to parse input file\n");
			return false;
		}
		// run the parser, solver incremental function is called

		if(!t_incremental_cfg.t_linear_freq.n_period && !t_incremental_cfg.t_nonlinear_freq.n_period) {
			if(r_t_args.b_show_detailed_timing) {
				double f_error = nonlinear_solver.f_Chi_Squared_Error_Denorm();
				printf("initial denormalized chi2 error: %.2f\n", f_error);
			}
			if(r_t_args.b_verbose)
				fprintf(stderr, "warning: running in batch mode. ignoring time spent in parser\n");
			t.ResetTimer();
		}
		// in case we're running in batch mode, don't count the time
		// spent in parser (because no computation was done)

		double f_time_initial_save_start = t.f_Time();
		if(r_t_args.b_write_bitmaps) {
			if(r_t_args.b_xz_plots)
				system.Plot3D("initial.tga");
			else
				system.Plot2D("initial.tga");

			/*double f_error = nonlinear_solver.f_Chi_Squared_Error_Denorm();
			printf("denormalized initial chi2 error: %.2lf\n", f_error);*/
			// need to calculate jacobians & errors first, don't do it ...
		}
		if(r_t_args.b_write_solution) {
			system.Dump("initial.txt");
			if(CSystemType::have_ConstVertices && !system.r_ConstVertex_Pool().b_Empty())
				system.Dump("initial_const.txt", true);
			// save the initial configuration to a file
		}
		if(r_t_args.b_write_system_matrix) {
			nonlinear_solver.Optimize(0);
			// force the solver to build the matrices (will do the marginals as well
			// though but this is only for debugging anyways)

			DumpSystemMatrix<b_solver_can_save>(nonlinear_solver);
			// calls solver.Save_SystemMatrix_MM("system.mtx"); but only if the solver supports it

			CDumpSystemMatrix_SparsityPattern<CSpecializedNonlinearSolverType,
				b_can_get_A, b_can_get_Lambda, b_can_get_R>::Dump(nonlinear_solver);
			// also save the sparsity pattern
		}
		double f_time_initial_save_end = t.f_Time();

		if(!t_incremental_cfg.t_linear_freq.n_period && !t_incremental_cfg.t_nonlinear_freq.n_period) {
			nonlinear_solver.Optimize(r_t_args.n_max_final_optimization_iteration_num,
				r_t_args.f_final_optimization_threshold);
		}
		// perform the final optimization (only in batch)

		double f_time = t.f_Time() - (f_time_initial_save_end - f_time_initial_save_start); // don't count saving of the initial system state (rasterizing the image takes some time)
		printf("\ndone. it took " PRItimeprecise " (%f sec)\n", PRItimeparams(f_time), f_time);
		//printf("time / 1.6 = " PRItimeprecise "\n", PRItimeparams(f_time / 1.6));
		// display time it took

		if(r_t_args.b_show_detailed_timing) {
			nonlinear_solver.Dump(f_time);
			double f_error = nonlinear_solver.f_Chi_Squared_Error_Denorm();
			printf("denormalized chi2 error: %.2f\n", f_error);
		}

		/*const CUberBlockMatrix &lambda = nonlinear_solver.r_Lambda();
		// get system matrix

		CUberBlockMatrix R;
		R.CholeskyOf(lambda);
		// take Cholesky

		CMarginals::Marginals_Test(R, 3);
		// test the marginals*/

		if(r_t_args.b_write_bitmaps) {
			/*if(!nonlinear_solver.Dump_SystemMatrix("system_matrix.tga", 5) &&
			   !nonlinear_solver.Dump_SystemMatrix("system_matrix.tga", 4) &&
			   !nonlinear_solver.Dump_SystemMatrix("system_matrix.tga", 3) &&
			   !nonlinear_solver.Dump_SystemMatrix("system_matrix.tga", 2))
				fprintf(stderr, "error: failed to dump system matrix image\n");*/
			if(r_t_args.b_xz_plots) {
				system.Plot3D("solution.tga");
				system.Plot3D("solution_print.tga", 2048, 2048, 10, 3, 7, 1, true, false, 10); // print size images
				system.Plot3D("solution_print_landmarks.tga", 2048, 2048, 10, 3, 7, 1, true, false, 10, true); // print size images
				system.Plot3D("solution_print_noticks.tga", 2048, 2048, 0, 0, 7, 1, true, false, 4); // print size images
			} else {
				system.Plot2D("solution.tga");
				system.Plot2D("solution_print.tga", 2048, 2048, 10, 3, 7, 1, true, false, 10); // print size images
				system.Plot2D("solution_print_landmarks.tga", 2048, 2048, 10, 3, 7, 1, true, false, 10, true); // print size images
				system.Plot2D("solution_print_noticks.tga", 2048, 2048, 0, 0, 7, 1, true, false, 4); // print size images
			}
			//nonlinear_solver.Save_SystemMatrix_MM("system_optimized.mtx");
		}
		if(r_t_args.b_write_solution) {
			system.Dump("solution.txt");
			if(CSystemType::have_ConstVertices && !system.r_ConstVertex_Pool().b_Empty())
				system.Dump("solution_const.txt", true);
			// save the solution to a file
		}

		return true;
	}
};

extern int n_dummy_param;

#ifdef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

/**
 *	@brief matrix benchmark runner
 *	@tparam n_block_size is matrix block size for the given benchmark
 */
template <const int n_block_size>
class CBlockBenchRunner {
public:
	/**
	 *	@brief runs a benchmark
	 *
	 *	@param[in] n_iter_num is number of iterations to take
	 *	@param[in] p_s_bench_name is name of a folder with the benchmark
	 *	@param[in] p_s_bench_type is type of benchmark to be run ("alloc", "chol" ro "all")
	 *
	 *	@return Returns true on success, false on failure (benchmark
	 *		failed or benchmars were not compiled).
	 */
	static bool Run(int n_iter_num, const char *p_s_bench_name, const char *p_s_bench_type)
	{
		if(!n_iter_num)
			n_iter_num = 1;

		CBlockMatrixBenchmark bmb(n_iter_num, CBlockMatrixBenchmark::order_RectOutline); // more realistic type of fill as incremental SLAM

		/*char p_s_infile[256];
#if defined(_WIN32) || defined(_WIN64)
		strcpy(p_s_infile, "G:\\uflsmc\\");
#else // _WIN32 || _WIN64
		strcpy(p_s_infile, "../data/"); // ubuntu
#endif // _WIN32 || _WIN64
		strcat(p_s_infile, p_s_bench_name);*/
		const char *p_s_infile = p_s_bench_name;
		if(!strcmp(p_s_bench_type, "alloc") || !strcmp(p_s_bench_type, "all")) {
			if(!bmb.template Run_AllocationBenchmark<n_block_size>(p_s_infile)) {
				fprintf(stderr, "error: benchmark failed\n");
				bmb.ShowResults(); // show at least partial results
				return false;
			}
		}
		if(!strcmp(p_s_bench_type, "factor") || !strcmp(p_s_bench_type, "all")) {
			if(!bmb.template Run_FactorizationBenchmark<n_block_size>(p_s_infile)) {
				fprintf(stderr, "error: benchmark failed\n");
				bmb.ShowResults(); // show at least partial results
				return false;
			}
		}
		bmb.ShowResults();
		char p_s_outfile[256];
#ifdef __BLOCK_BENCH_CHOLESKY_USE_AMD
		sprintf(p_s_outfile, "%s_%d_AMD", p_s_bench_name, n_block_size);
#else // __BLOCK_BENCH_CHOLESKY_USE_AMD
		sprintf(p_s_outfile, "%s_%d", p_s_bench_name, n_block_size);
#endif // __BLOCK_BENCH_CHOLESKY_USE_AMD

		for(char *p_char = p_s_outfile; *p_char; ++ p_char) {
			if(strchr("/\\?:@.^;\"\'", *p_char))
				*p_char = '_';
		}
		// replace special characters that should not appear in paths

		strcat(p_s_outfile, ".csv");
		// after replacement, to keep the dot

		if(!bmb.Save_ResultSheet(p_s_outfile))
			fprintf(stderr, "error: i/o error while writing results\n");
#ifdef __BLOCK_BENCH_CHOLESKY_USE_AMD
		sprintf(p_s_outfile, "%s_%d_avg_AMD.csv", p_s_bench_name, n_block_size);
#else // __BLOCK_BENCH_CHOLESKY_USE_AMD
		sprintf(p_s_outfile, "%s_%d_avg.csv", p_s_bench_name, n_block_size);
#endif // __BLOCK_BENCH_CHOLESKY_USE_AMD
		if(!bmb.Save_TestBased_ResultSheet(p_s_outfile))
			fprintf(stderr, "error: i/o error while writing results\n");
		// benchmarks on saved matrices (see if they save properly)

		return true;
	}
};

#endif // __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

/**
 *	@brief runs a block matrix benchmark
 *
 *	@param[in] n_block_size is matrix block size for the given benchmark
 *		(one of 1, 2, 3, 4, 5, 6, 8, 10, 15, 16, 20, 25 or 30)
 *	@param[in] p_s_bench_name is benchmark name (not path)
 *	@param[in] p_s_bench_type is benchmark type, one of "factor", "alloc" or "all"
 *
 *	@note This handles block sizes 1 - 30 with certain step, the implementation
 *		is split to BlockBenchImpl0.cpp and BlockBenchImpl1.cpp in order to
 *		make parallel building faster (each file instantiates test templates
 *		with different block sizes).
 *	@note The benchmarks mostly use four block sizes, the specified n x n
 *		and (n + 1) x n, n x (n + 1) and (n + 1) x (n + 1), as happens
 *		in landmark datasets.
 *
 *	@return Returns benchmark result, 0 on success, nonzero on failure.
 */
int n_Run_BlockBenchmark(int n_block_size,
	const char *p_s_bench_name, const char *p_s_bench_type);

/**
 *	@copydoc n_Run_BlockBenchmark()
 *	@note This handles block sizes 8 - 30.
 */
int n_Run_BlockBenchmark1(int n_block_size,
	const char *p_s_bench_name, const char *p_s_bench_type);

/**
 *	@brief prints commandline help
 */
void PrintHelp();

/**
 *	@brief prints all the important compiler / optimization switches this app was built with
 */
void DisplaySwitches();

/**
 *	@brief structure, containing values of all the commandline arguments
 */
struct TCommandLineArgs {
	ENonlinearSolverType n_solver_choice; /**< @brief nonlinear solver selector */
	bool b_write_bitmaps; /**< @brief bitmaps write flag */
	bool b_write_solution; /**< @brief initial / solution write flag */
	bool b_xz_plots; /**< @brief x-z bitmaps orientation flag */
	bool b_write_system_matrix; /**< @brief matrix write flag */
	bool b_no_show; /**< @brief bitmaps show flag (only on windows) */
	bool b_show_commandline; /**< @brief commandline repeat flag */
	bool b_show_flags; /**< @brief show build flags flag */
	bool b_show_detailed_timing; /**< @brief show detailed timing flag */
	bool b_verbose; /**< @brief verbosity flag; true means verbose, false means silent */
	bool b_use_schur; /**< @brief use Schur component flag */
	bool b_run_matrix_benchmarks; /**< @brief run block matrix benchmarks flag */
	bool b_run_matrix_unit_tests; /**< @brief run block matrix unit tests flag */
	bool b_use_old_system; /**< @brief old system flag (deprecated) */
	bool b_pose_only; /**< @brief optimize pose-only problems */
	bool b_use_spheron; /**< @brief process Spheron SLAM system (stereo spherical camera alignment) @note This is not overriden in commandline but detected in peek-parsing. */
	bool b_use_rocv; /**< @brief process range-only constant velocity system (triangulated navigation prototype) @note This is not overriden in commandline but detected in peek-parsing. */
	bool b_use_SE3; /**< @brief process SE3 system @note This is not overriden in commandline but detected in peek-parsing. */
	bool b_use_BA; /**< @brief process bundle adjustment system @note This is not overriden in commandline but detected in peek-parsing. */
	bool b_use_BAS; /**< @brief process stereo bundle adjustment system @note This is not overriden in commandline but detected in peek-parsing. */
	bool b_use_BAI; /**< @brief process bundle adjustment system with intrinsic cam parameters @note This is not overriden in commandline but detected in peek-parsing. */
	const char *p_s_input_file; /**< @brief path to the data file */
	int n_max_lines_to_process; /**< @brief maximal number of lines to process */
	size_t n_linear_solve_each_n_steps; /**< @brief linear solve period, in steps (0 means disabled) */
	size_t n_nonlinear_solve_each_n_steps; /**< @brief nonlinear solve period, in steps (0 means disabled) */
	size_t n_max_nonlinear_solve_iteration_num; /**< @brief maximal number of iterations in nonlinear solve step */
	double f_nonlinear_solve_error_threshold; /**< @brief error threshold for nonlinear solve */
	size_t n_max_final_optimization_iteration_num; /**< @brief number of nonlinear solver iterations */ // as many other solvers
	double f_final_optimization_threshold; /**< @brief final optimization threshold */
	const char *p_s_bench_name; /**< @brief benchmark file name (only if b_run_matrix_benchmarks is set) */
	const char *p_s_bench_type; /**< @brief benchmark type (only if b_run_matrix_benchmarks is set) */
	size_t n_omp_threads; /**< @brief OpenMP number of threads override */
	bool b_omp_dynamic; /**< @brief OpenMP dynamic scheduling override enable flag */
	bool b_do_marginals; /**< @brief marginal covariance calculation enable flag */
	double f_dogleg_step_size; /**< @brief dogleg solver step size */
	double f_dogleg_step_threshold; /**< @brief dogleg solver step threshold */
	bool b_dogleg_all_batch; /**< dogleg solver all batch flag */
	bool b_inc_BA_save_intermediates; /**< @brief in incremental BA, save intermediate solutions (and marginals if enabled) at every step */
	bool b_inc_BA_save_sysmats; /**< @brief in incremental BA, save system matrices at every step */

	/**
	 *	@brief selects default values for commandline args
	 */
	void Defaults();

	/**
	 *	@brief parse commandline arguments
	 *
	 *	@param[in] n_arg_num is number of commandline arguments
	 *	@param[in] p_arg_list is the list of commandline arguments
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Parse(int n_arg_num, const char **p_arg_list);

	/**
	 *	@brief runs the application, using this solver
	 *
	 *	@tparam CSystemType is system type (derived from CFlatSystem)
	 *	@tparam CEdgeTraitsType is edge traits template name
	 *	@tparam CVertexTraitsType is vertex traits template name
	 *	@tparam CParseLoopType is parse loop template name
	 *	@tparam CParsedPrimitives is a list of parsed primitives (e.g. CStandardParsedPrimitives)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CSystemType, template <class, class/*, class*/> class CNonlinearSolverType,
		template <class> class CEdgeTraitsType, template <class> class CVertexTraitsType,
		template <class, class, template <class> class vcneedsnamehere,
		template <class> class vcneedsnamehereaswell> class CParseLoopType,
		class CParsedPrimitives>
	inline bool Run() // throw(std::runtime_error, std::bad_alloc)
	{
		return CTester<CSystemType, CNonlinearSolverType, CEdgeTraitsType,
			CVertexTraitsType, CParseLoopType, CParsedPrimitives>::Run_and_Shout(*this);
		// run with parameters
	}
};

/**
 *	@brief functor for CTypelistForEach; selects requested solver and runs the application
 *
 *	@tparam CSystemType is system type (derived from CFlatSystem)
 *	@tparam CEdgeTraitsType is edge traits template name
 *	@tparam CVertexTraitsType is vertex traits template name
 *	@tparam CParseLoopType is parse loop template name
 *	@tparam CParsedPrimitives is a list of parsed primitives (default CStandardParsedPrimitives)
 */
template <class CSystemType, template <class> class CEdgeTraitsType,
	template <class> class CVertexTraitsType, template <class, class,
	template <class> class, template <class> class> class CParseLoopType,
	class CParsedPrimitives = CStandardParsedPrimitives>
class CSolverCaller {
protected:
	TCommandLineArgs m_t_args; /**< @brief copy of parsed commandline args */
	int m_n_result; /**< @brief final result of the application */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] t_args is a copy of parsed commandline args
	 */
	inline CSolverCaller(TCommandLineArgs t_args)
		:m_t_args(t_args), m_n_result(-1)
	{}

	/**
	 *	@brief function operator; checks for suitable solver and runs
	 *	@tparam CSolverType is a pair of solver id and type
	 */
	template <class CSolverType>
	inline void operator ()() // throw(std::runtime_error, std::bad_alloc)
	{
		if(m_t_args.n_solver_choice == int(CSolverType::solver_type_Id)) {
			m_n_result = (CSolverType::template Run_MainApp<CSystemType, CEdgeTraitsType,
				CVertexTraitsType, CParseLoopType, CParsedPrimitives>(m_t_args))? 0 : -1;
			// call the traits
		}
		// find the solver in the list
	}

	/**
	 *	@brief gets result value of the solver call (as in main() result values)
	 *	@return Returns value of the result, 0 means success, other values are failure.
	 */
	inline int n_Result() const
	{
		return m_n_result;
	}
};

/**
 *	@brief runs bundle adjustment with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_BA_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs bundle adjustment with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_BA_Intrinsics_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs bundle adjustment with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_Spheron_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs stereo bundle adjustment with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_BA_Stereo_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs range-only constant velocity with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_ROCV_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs pose-landmark 3D SLAM with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_SE3_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs (pose-only) 3D SLAM with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_SE3PoseOnly_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs 2D SLAM with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_SE2_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

/**
 *	@brief runs pose-only 2D SLAM with a specified solver
 *	@param[in] t_args is a copy of parsed commandline arguments
 *	@return Returns 0 on success, -1 on failure.
 */
int n_Run_SE2PoseOnly_Solver(const TCommandLineArgs &t_args); // throw(std::runtime_error, std::bad_alloc)

#endif // !__SLAMPP_MAIN_INCLUDED
