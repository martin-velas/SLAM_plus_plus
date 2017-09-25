/*
								+-----------------------------------+
								|                                   |
								| ***  Simple BA iface example  *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/ba_interface_example/Main.cpp
 *	@brief contains the main() function of the simple example BA program
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include <stdio.h> // printf
#include "ba_interface_example/BAOptimizer.h" // BA types
// note that nothing else from SLAM++ needs to be included

#include "slam/Parser.h" // want to be able to parse .graph files
#include "slam_app/ParsePrimitives.h" // reuse SLAM++ standard parse primitives
// we need this to parse .graph files

int n_dummy_param = 0; // required by the DL solver, otherwise causes a link error

/**
 *	@brief a simple parse loop for BA systems
 */
class CMyParseLoop {
protected:
	CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_optimizer is reference to the optimizer to be filled with edges and vertices
	 */
	CMyParseLoop(CBAOptimizer &r_optimizer)
		:m_r_optimizer(r_optimizer)
	{}

	/**
	 *	@brief appends the system with a general edge of unknown type
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_P2C3DEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
			r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief initializes a camera vertex
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_CamVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief initializes a structure point vertex
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const CParserBase::TVertexXYZ &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_XYZVertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
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
int main(int n_arg_num, const char **p_arg_list)
{
	bool b_verbose = true;
	const char *p_s_input_file = 0;
	for(int i = 1; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			printf("use: ba_iface_example [-i|--input <input-graph-file>] [-q|--quiet]\n");
			return 0;
		} else if(!strcmp(p_arg_list[i], "--quiet") || !strcmp(p_arg_list[i], "-q"))
			b_verbose = false;
		else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return -1;
		} else if(!strcmp(p_arg_list[i], "--input") || !strcmp(p_arg_list[i], "-i"))
			p_s_input_file = p_arg_list[++ i];
		else {
			fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
			return -1;
		}
	}
	if(!p_s_input_file) {
		fprintf(stderr, "error: no input file specified. run with -h or --help to get help\n");
		return -1;
	}
	// "parse" cmdline

	CBAOptimizer optimizer(b_verbose);
	// create the optimizer object

	{
		typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
			CEdgeP2C3DParsePrimitive) CMyParsedPrimitives;
		typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
		CMyParseLoop parse_loop(optimizer);
		if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
			fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
			return -1;
		}
	}
	// initialize the vertices, add edges

	optimizer.Optimize();
	// optimize the system

	optimizer.Show_Stats();
	optimizer.Dump_State("solution.txt");
	// show the timing, save the results

	return 0;
}

/**
 *	@page baifaceexample Simple SLAM++ Interface Example
 *
 *	This example shows how to interface SLAM++ with other code, may that be C++ or even "C".
 *	It also demonstrates a very simple bundle adjustment (BA) optimizer, which takes graph files
 *	such as Venice (file venice871.g2o). The BA problem is posed using SE(3) global poses for
 *	cameras and global positions for points (rather than relative camera poses and points relative
 *	to camera).
 *
 *	The code is divided to two files, @ref Main.cpp and @ref BAOptimizer.cpp. All of SLAM++ heavy
 *	duty code is included only in BAOptimizer.cpp, while Main.cpp only includes the optimizer
 *	interface. This makes development of applications using SLAM++ easier, as Main.cpp now
 *	compiles in a few seconds (three seconds on a Core i7 laptop), while BAOptimizer.cpp takes
 *	a while to compile but needs to be rarely changed during the development.
 *
 *	We begin by creating an optimizer objecet:
 *
 *	@code
 *	CBAOptimizer optimizer
 *	@endcode
 *
 *	This object contains SLAM++ solver as well as the optimized system state. To fill it, one
 *	can call one of these functions:
 *
 * 	@code
 *	Eigen::Matrix<double, 11, 1> first_cam_pose, second_cam_pose;
 *	first_cam_pose << 0 0 0 0 0 0 500 500 0 0 0;
 *	optimizer.Add_CamVertex(0, first_cam_pose);
 *	second_cam_pose << 0 0 -10 0 0 0 500 500 0 0 0;
 *	optimizer.Add_CamVertex(1, second_cam_pose);
 *	// add a camera at the origin and another 10 units in front of it along z+
 *	// the optical centre is at (500, 500) pixels and the camera has no skew / distortion
 *	// (note that the poses are inverse - they transform points from world space to camera space)
 *
 *	optimizer.Add_XYZVertex(2, Eigen::Vector3d(10, 20, 30));
 *	// add a 3D point
 *
 *	optimizer.Add_P2C3DEdge(2, 0, Eigen::Vector2d(50, 50), Eigen::Matrix2d::Identity() * 10);
 *	optimizer.Add_P2C3DEdge(2, 1, Eigen::Vector2d(75, 75), Eigen::Matrix2d::Identity() * 10);
 *	// add an observation of the point by the two cameras, with covariance [10 0; 0 10]
 *	@endcode
 *
 *	We can access the state using:
 *
 * 	@code
 *	Eigen::Map<Eigen::VectorXd> camera_0_state = optimizer.r_Vertex_State(0);
 *	@endcode
 *
 *	Where the address of the vector data does not change over time (e.g. when more vertices are
 *	added, as would happen with std::vector elements).
 *	Now that we have the system ready, we can optimize using:
 *
 * 	@code
 *	optimizer.Optimize();
 *	@endcode
 *
 *	This also makes the value of camera_0_state change, so that the optimized camera pose
 *	is available. Alternately, we can save the results using:
 *
 * 	@code
 *	Dump_State("solution.txt");
 *	@endcode
 *
 *	It is also possible to save the optimized graph, like this:
 *
 * 	@code
 *	optimizer.Dump_Graph("solution.graph");
 *	@endcode
 *
 *	This is especially handy if doing processing from sensorial data - other researchers can
 *	then use this graph file without having to process the sensor data again. Note that this
 *	graph does not preserve the order in which the edges and vertices were added to the optimizer.
 *	It outputs all the vertices first (ordered by their indices), then outputs the edges (in the
 *	order they were introduced to the optimizer).
 *
 *	It is now possible to create a statically linked library which you can link to your existing
 *	code easily, and just use the simple optimizer interface where no templates are involved.
 *	@ref BAOptimizer.h also features a simple "C" interface which can be easily extended with
 *	additional functionality.
 *
 */

/*
 *	end-of-file
 */
