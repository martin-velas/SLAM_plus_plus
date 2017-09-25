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
 *	@file src/incremental_ba_3dv/Main.cpp
 *	@brief experiments for the 2017 3DV paper "Fast Incremental Bundle Adjustment with Covariance
 *		Recovery" with Viorela Ila, Marek Solony and Klemen Istenic.
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include <stdio.h> // printf
#include "incremental_ba_3dv/BAOptimizer.h" // BA types
// note that nothing else from SLAM++ needs to be included

#include "slam/Parser.h" // want to be able to parse .graph files
#include "slam_app/ParsePrimitives.h" // reuse SLAM++ standard parse primitives
// we need this to parse .graph files

#include "slam/ConfigSolvers.h"
#include "slam/Sim3SolverBase.h"
#include "ba_parameter_acra/Sim3_ParsePrimitives.h"
#include "slam/ErrorEval.h"

/**
 *	@brief consistency marker type (passed as an edge)
 */
class CConsistencyMarker : public CParserBase::CParseEntity {};

/**
 *	@brief consistency marker parse primitive handler
 */
class CConsistencyMarker_ParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throw(std::bad_alloc)
	{
		r_token_name_map["CONSISTENCY_MARKER"] = n_assigned_id;
		// add as uppercase!
	}

	/**
	 *	@brief parses this primitive and dispatches it to the parse loop
	 *
	 *	@param[in] n_line_no is zero-based line number (for error reporting)
	 *	@param[in] r_s_line is string, containing the current line (without the token)
	 *	@param[in] r_s_token is string, containing the token name (in uppercase)
	 *	@param[in,out] r_parse_loop is target for passing the parsed primitives to
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class _TyParseLoop>
	static bool Parse_and_Dispatch(size_t n_line_no, const std::string &r_s_line,
		const std::string &UNUSED(r_s_token), _TyParseLoop &r_parse_loop)
	{
		r_parse_loop.AppendSystem(CConsistencyMarker());
		// append fake measurement to invoke

		return true;
	}
};

/**
 *	@brief a simple parse loop for BA systems
 */
class CMyParseLoop {
protected:
	CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */

	int m_n_inc_nls_iter_num; // incremental optimization setting
	double m_f_inc_nlsolve_thresh; // incremental optimization setting
	bool m_b_save_inc_outputs; // incremental optimization setting

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] r_optimizer is reference to the optimizer to be filled with edges and vertices
	 *	@param[in] n_inc_nls_iter_num
	 *	@param[in] f_inc_nlsolve_thresh
	 *	@param[in] b_save_inc_outputs
	 */
	CMyParseLoop(CBAOptimizer &r_optimizer, int n_inc_nls_iter_num = 1,
		double f_inc_nlsolve_thresh = .005, bool b_save_inc_outputs = false)
		:m_r_optimizer(r_optimizer), m_n_inc_nls_iter_num(n_inc_nls_iter_num),
		m_f_inc_nlsolve_thresh(f_inc_nlsolve_thresh), m_b_save_inc_outputs(b_save_inc_outputs)
	{}

	/**
	 *	@brief initializes a SE(3) camera vertex (embeds it in Sim(3))
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
	{
		Eigen::Vector7d v_pos;
		v_pos.head<6>() = r_t_vertex.m_v_position.head<6>();
		v_pos(6) = 1; // scale = 1

		Eigen::Vector5d v_intrinsics = r_t_vertex.m_v_position.tail<5>();

		CSim3Jacobians::TSim3 t_pose = CSim3Jacobians::TSim3(v_pos, CSim3Jacobians::TSim3::from_tRs_vector); // convert from SE(3)
		t_pose.Invert(); // invert (the SE(3) cameras are represented by inverse transform)
		v_pos = t_pose.v_Log();

		Eigen::Matrix<double, 12, 1> v_state;
		v_state.head<7>() = v_pos;
		v_state.tail<5>() = v_intrinsics;

		m_r_optimizer.Add_CamSim3Vertex(r_t_vertex.m_n_id, v_state);
	}

	/**
	 *	@brief initializes a Sim(3) camera vertex
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const TVertexCamSim3 &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_CamSim3Vertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief initializes a structure point vertex from XYZ
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const CParserBase::TVertexXYZ &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_XYZVertex_Global(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief initializes a structure point vertex from inverse depth and a camera id
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const TVertexInvDepth &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_InvDepthVertex_Local(r_t_vertex.n_vertex_id, r_t_vertex.v_position, r_t_vertex.n_camera_id); // add vertex as local to a specific camera
		_ASSERTE((m_r_optimizer.v_InvDepthVertex_Local(r_t_vertex.n_vertex_id) - r_t_vertex.v_position).norm() < 1e-10); // make sure that the local get works as well
	}

	/**
	 *	@brief appends the system with a projection edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_P2CSim3GEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
			r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief appends the system with a projection edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const TEdgeP2CSim3G &r_t_edge) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_P2CSim3GEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
			r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief handles the consistency marker token
	 *	@param[in] r_marker is the consistency marker parse primitive (unused)
	 */
	void AppendSystem(const CConsistencyMarker &UNUSED(r_marker)) // throw(std::bad_alloc)
	{
		m_r_optimizer.Optimize(m_n_inc_nls_iter_num,
			m_f_inc_nlsolve_thresh, m_f_inc_nlsolve_thresh); // handle incremental optimization

#if 0
		if(m_b_save_inc_outputs) { // in case this is an incremental solve step, dump the dx vector
			static size_t n_iter_num = 0;
			++ n_iter_num;
			char p_s_filename[256];
			sprintf(p_s_filename, "solution_SE3_iter_%05" _PRIsize ".txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_State_SE3(p_s_filename);
			sprintf(p_s_filename, "marginals_iter_%05" _PRIsize ".txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_Marginals(p_s_filename);
		}
#endif // 0
	}
};

/**
 *	@brief a simple parse loop for BA systems; this one first records the parsed
 *		data and then passes it to the optimizer from memory (avoids timing file I/O)
 */
class CMyParseLoop_Recordable {
protected:
	CBAOptimizer &m_r_optimizer; /**< @brief reference to the optimizer */

	int m_n_inc_nls_iter_num; // incremental optimization setting
	double m_f_inc_nlsolve_thresh; // incremental optimization setting
	bool m_b_save_inc_outputs; // incremental optimization setting

	typedef MakeTypelist5(TVertexCamSim3, CParserBase::TVertexXYZ,
		TVertexInvDepth, TEdgeP2CSim3G, CConsistencyMarker) ParsedItems; // types of parsed items // note that there is currently no mechanism of how to read this from the parsed primitives, is there?
	typedef multipool::CMultiPool<CParserBase::CParseEntity, ParsedItems> CParsedPool; // type of pool for storing the parsed items

	CParsedPool m_parsed_data; // all data parsed from the file

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] r_optimizer is reference to the optimizer to be filled with edges and vertices
	 *	@param[in] n_inc_nls_iter_num
	 *	@param[in] f_inc_nlsolve_thresh
	 *	@param[in] b_save_inc_outputs
	 */
	CMyParseLoop_Recordable(CBAOptimizer &r_optimizer, int n_inc_nls_iter_num = 1,
		double f_inc_nlsolve_thresh = .005, bool b_save_inc_outputs = false)
		:m_r_optimizer(r_optimizer), m_n_inc_nls_iter_num(n_inc_nls_iter_num),
		m_f_inc_nlsolve_thresh(f_inc_nlsolve_thresh), m_b_save_inc_outputs(b_save_inc_outputs)
	{}

	/**
	 *	@brief records a SE(3) camera vertex (embeds it in Sim(3))
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const CParserBase::TVertexCam3D &r_t_vertex) // throw(std::bad_alloc)
	{
		Eigen::Vector7d v_pos;
		v_pos.head<6>() = r_t_vertex.m_v_position.head<6>();
		v_pos(6) = 1; // scale = 1

		Eigen::Vector5d v_intrinsics = r_t_vertex.m_v_position.tail<5>();

		CSim3Jacobians::TSim3 t_pose = CSim3Jacobians::TSim3(v_pos, CSim3Jacobians::TSim3::from_tRs_vector); // convert from SE(3)
		t_pose.Invert(); // invert (the SE(3) cameras are represented by inverse transform)
		v_pos = t_pose.v_Log();

		Eigen::Matrix<double, 12, 1> v_state;
		v_state.head<7>() = v_pos;
		v_state.tail<5>() = v_intrinsics;

		TVertexCamSim3 t_cam(r_t_vertex.m_n_id, v_state(0), v_state(1), v_state(2),
			v_state(3), v_state(4), v_state(5), v_state(6), v_state(7), v_state(8),
			v_state(9), v_state(10), v_state(11));
		t_cam.m_v_position(11) = v_state(11); // this was already transformed

		m_parsed_data.r_Add_Element(t_cam);
		//m_r_optimizer.Add_CamSim3Vertex(r_t_vertex.m_n_id, v_state);
	}

	/**
	 *	@brief records a Sim(3) camera vertex
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const TVertexCamSim3 &r_t_vertex) // throw(std::bad_alloc)
	{
		m_parsed_data.r_Add_Element(r_t_vertex);
		//m_r_optimizer.Add_CamSim3Vertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief records a structure point vertex given as XYZ
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const CParserBase::TVertexXYZ &r_t_vertex) // throw(std::bad_alloc)
	{
		m_parsed_data.r_Add_Element(r_t_vertex);
		//m_r_optimizer.Add_XYZVertex_Global(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief records a structure point vertex given as inverse depth and a camera id
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void InitializeVertex(const TVertexInvDepth &r_t_vertex) // throw(std::bad_alloc)
	{
		m_parsed_data.r_Add_Element(r_t_vertex);
		//m_r_optimizer.Add_InvDepthVertex_Local(r_t_vertex.n_vertex_id, r_t_vertex.v_position, r_t_vertex.n_camera_id); // add vertex as local to a specific camera
		//_ASSERTE((m_r_optimizer.v_InvDepthVertex_Local(r_t_vertex.n_vertex_id) - r_t_vertex.v_position).norm() < 1e-10); // make sure that the local get works as well
	}

	/**
	 *	@brief records a projection edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const CParserBase::TEdgeP2C3D &r_t_edge) // throw(std::bad_alloc)
	{
		m_parsed_data.r_Add_Element((const TEdgeP2CSim3G&)r_t_edge);
		//m_r_optimizer.Add_P2CSim3GEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
		//	r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief records a projection edge
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const TEdgeP2CSim3G &r_t_edge) // throw(std::bad_alloc)
	{
		m_parsed_data.r_Add_Element(r_t_edge);
		//m_r_optimizer.Add_P2CSim3GEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
		//	r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief records the consistency marker token
	 *	@param[in] r_marker is the consistency marker parse primitive (unused)
	 */
	void AppendSystem(const CConsistencyMarker &r_marker) // throw(std::bad_alloc)
	{
		m_parsed_data.r_Add_Element(r_marker);
		/*m_r_optimizer.Optimize(m_n_inc_nls_iter_num,
			m_f_inc_nlsolve_thresh, m_f_inc_nlsolve_thresh); // handle incremental optimization

#if 0
		if(m_b_save_inc_outputs) { // in case this is an incremental solve step, dump the dx vector
			static size_t n_iter_num = 0;
			++ n_iter_num;
			char p_s_filename[256];
			sprintf(p_s_filename, "solution_SE3_iter_%05" _PRIsize ".txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_State_SE3(p_s_filename);
			sprintf(p_s_filename, "marginals_iter_%05" _PRIsize ".txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_Marginals(p_s_filename);
		}
#endif // 0*/
	}

	// note - the operators () could read vertex / edge traits and call a regular parse loop instead

	/**
	 *	@brief initializes a Sim(3) camera vertex during the \ref Playback() phase
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void operator ()(const TVertexCamSim3 &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_CamSim3Vertex(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief initializes a structure point vertex from XYZ during the \ref Playback() phase
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void operator ()(const CParserBase::TVertexXYZ &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_XYZVertex_Global(r_t_vertex.m_n_id, r_t_vertex.m_v_position);
	}

	/**
	 *	@brief initializes a structure point vertex from inverse depth and a camera id during the \ref Playback() phase
	 *	@param[in] r_t_vertex is the vertex to be initialized
	 */
	void operator ()(const TVertexInvDepth &r_t_vertex) // throw(std::bad_alloc)
	{
		m_r_optimizer.Add_InvDepthVertex_Local(r_t_vertex.n_vertex_id, r_t_vertex.v_position, r_t_vertex.n_camera_id); // add vertex as local to a specific camera
		_ASSERTE((m_r_optimizer.v_InvDepthVertex_Local(r_t_vertex.n_vertex_id) - r_t_vertex.v_position).norm() < 1e-10); // make sure that the local get works as well
	}

	/**
	 *	@brief appends the system with a projection edge during the \ref Playback() phase
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void operator ()(const TEdgeP2CSim3G &r_t_edge)
	{
		m_r_optimizer.Add_P2CSim3GEdge(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
			r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma);
	}

	/**
	 *	@brief handles the consistency marker token during the \ref Playback() phase
	 *	@param[in] r_marker is the consistency marker parse primitive (unused)
	 */
	void operator ()(const CConsistencyMarker &r_marker)
	{
		m_r_optimizer.Optimize(m_n_inc_nls_iter_num,
			m_f_inc_nlsolve_thresh, m_f_inc_nlsolve_thresh); // handle incremental optimization

#if 0
		if(m_b_save_inc_outputs) { // in case this is an incremental solve step, dump the dx vector
			static size_t n_iter_num = 0;
			++ n_iter_num;
			char p_s_filename[256];
			sprintf(p_s_filename, "solution_SE3_iter_%05" _PRIsize ".txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_State_SE3(p_s_filename);
			sprintf(p_s_filename, "marginals_iter_%05" _PRIsize ".txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_Marginals(p_s_filename);
		}
#endif // 0
	}

	/**
	 *	@brief processes all the recorded data
	 */
	void Playback() // throw(std::bad_alloc)
	{
		m_parsed_data.For_Each(*this);
		// push the parsed items into the optimizer
	}
};

int n_dummy_param = 0;

class CEvaluation : public CErrorEvaluation<false> {
public:
	static bool Load_SE3_PoseSet(const char *p_s_filename, std::vector<_TyVectorUnalign> &r_dest, bool b_inverse_poses = true)
	{
		return CErrorEvaluation<false>::Load_PoseSet(p_s_filename, r_dest, b_inverse_poses);
	}

	static bool ComputeError(const char *p_s_estimated_file, const char *p_s_ground_truth_file)
	{
		std::vector<_TyVectorUnalign> poses_a, poses_b;
		return Load_SE3_PoseSet(p_s_estimated_file, poses_a) && // estimated stored as inverse for viewing
			Load_SE3_PoseSet(p_s_ground_truth_file, poses_b) && ComputeError(poses_a, poses_b); // ground truth stored by vincent, apparently also inverse :)
	}

	/**
	 *	@brief computes error comparing to the ground truth, prints to stdout
	 *
	 *	@param[in] est_vertices is a vector of estimated camera poses
	 *	@param[in] gt_vertices is vector of ground truth camera poses
	 *
	 *	The poses are represented as 3D position + axis-angle rotation.
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool ComputeError(const std::vector<_TyVectorUnalign> &est_vertices,
		const std::vector<_TyVectorUnalign> &gt_vertices) // throw(std::bad_alloc)
	{
		Eigen::Vector6d v_RMSE;
		return CErrorEvaluation<false>::Compute_AllErrors(est_vertices, gt_vertices, false, v_RMSE, true);
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
int main(int n_arg_num, const char **p_arg_list) // standalone app
{
	bool b_verbose = true, b_incremental = false, b_use_schur = false,
		b_all_batch = false, b_force_inc_schur = false, b_do_marginals = false,
		b_do_icra_style_marginals = false, b_calculate_eigenvalues = false,
		b_save_inc_outputs = false;
	int n_max_inc_iters = 1, n_max_final_iters = 5;
	double f_inc_nlsolve_thresh = .005, f_final_nlsolve_thresh = .005;
	int n_update_thresh = -4;
	int n_relin_thresh = -4;
	double f_trust_radius = .2;
	bool b_trust_radius_persistent = false; // reset at each iteration
	const char *p_s_input_file = 0, *p_s_ground_truth_file = 0;
	for(int i = 1; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			printf("use: incremental_ba_3dv [-i|--input <input-graph-file>] [-q|--quiet]\n"
				   "     [-inc|--incremental] [-dxt|--dx-thresh <negative-per-vertex-exponent>]\n"
				   "     [-rlt|--relin-threshold <negative-per-vertex-exponent>] [-fis|--force-inc-schur]\n"
				   "     [-gt|--ground-truth <ground-truth-file>] [-dp|--dummy-param <LM-damping-integer>]\n"
				   "     [-tr|--trust-radius <initial-dogleg-trust-radius>]\n"
				   "     [-ptr|--persistent-trust-radius]\n"
				   "     [-mfnsi|--max-final-nonlinear-solve-iters <num-iterations>]\n"
				   "     [-fnset|--final-nonlinear-solve-error-thresh <threshold>]\n"
				   "     [-dm|--do-marginals] [-de|--do-eigs] [-sio|--save-incremental-outputs]\n"
				   "\n"
				   "The default nonlinear solve threshold is 0.005, the default final number of iterations\n"
				   "is 5, the default update threshold is 10^-4 and the default trust radius is 0.2.\n");
			return 0;
		} else if(!strcmp(p_arg_list[i], "--quiet") || !strcmp(p_arg_list[i], "-q"))
			b_verbose = false;
		else if(!strcmp(p_arg_list[i], "--incremental") || !strcmp(p_arg_list[i], "-inc"))
			b_incremental = true;
		else if(!strcmp(p_arg_list[i], "--use-schur") || !strcmp(p_arg_list[i], "-us"))
			b_use_schur = true;
		else if(!strcmp(p_arg_list[i], "--all-batch") || !strcmp(p_arg_list[i], "-ab"))
			b_all_batch = true;
		else if(!strcmp(p_arg_list[i], "--force-inc-schur") || !strcmp(p_arg_list[i], "-fis"))
			b_force_inc_schur = true;
		else if(!strcmp(p_arg_list[i], "--do-marginals") || !strcmp(p_arg_list[i], "-dm"))
			b_do_marginals = true;
		else if(!strcmp(p_arg_list[i], "--do-icra15-marginals") || !strcmp(p_arg_list[i], "-icram"))
			b_do_icra_style_marginals = true;
		else if(!strcmp(p_arg_list[i], "--do-eigs") || !strcmp(p_arg_list[i], "-de"))
			b_calculate_eigenvalues = true;
		else if(!strcmp(p_arg_list[i], "--save-incremental-outputs") || !strcmp(p_arg_list[i], "-sio"))
			b_save_inc_outputs = true;
		else if(!strcmp(p_arg_list[i], "--persistent-trust-radius") || !strcmp(p_arg_list[i], "-ptr"))
			b_trust_radius_persistent = true;
		else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return -1;
		} else if(!strcmp(p_arg_list[i], "--input") || !strcmp(p_arg_list[i], "-i"))
			p_s_input_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--ground-truth") || !strcmp(p_arg_list[i], "-gt"))
			p_s_ground_truth_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--dummy-param") || !strcmp(p_arg_list[i], "-dp"))
			n_dummy_param = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--max-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mnsi"))
			n_max_inc_iters = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-nset"))
			f_inc_nlsolve_thresh = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--max-final-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mfnsi"))
			n_max_final_iters = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--final-nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-fnset"))
			f_final_nlsolve_thresh = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--dx-threshold") || !strcmp(p_arg_list[i], "-dxt")) { // use 0 for none
			n_update_thresh = atol(p_arg_list[++ i]);
			if(n_update_thresh > 0)
				fprintf(stderr, "warning: using unusually high update threshold\n");
			else if(!n_update_thresh)
				fprintf(stderr, "warning: using update threshold 0, disabling update threshold\n");
		} else if(!strcmp(p_arg_list[i], "--relin-threshold") || !strcmp(p_arg_list[i], "-rlt")) { // use 0 for none
			n_relin_thresh = atol(p_arg_list[++ i]);
			if(n_relin_thresh > 0)
				fprintf(stderr, "warning: using unusually high relinearization threshold\n");
			else if(!n_relin_thresh)
				fprintf(stderr, "warning: using relinearization threshold 0, disabling relinearization threshold\n");
		} else if(!strcmp(p_arg_list[i], "--trust-radius") || !strcmp(p_arg_list[i], "-tr"))
			f_trust_radius = atof(p_arg_list[++ i]);
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

	printf("> ./incremental_ba_3dv");
	for(int i = 1; i < n_arg_num; ++ i)
		printf(" %s", p_arg_list[i]);
	printf("\n");
	// display commandline

	if(n_dummy_param)
		printf("running with dummy LM damping %d\n", n_dummy_param);
	//if(f_trust_radius != .2)
		printf("running with trust radius of %g\n", f_trust_radius);
	if(n_update_thresh)
		printf("running with dx thresh of 10^%d\n", n_update_thresh);
	else
		printf("running with dx thresh of 0\n");
	double f_update_thresh = (!n_update_thresh)? 0 : pow(10.0, double(n_update_thresh));
	if(n_relin_thresh)
		printf("running with relin thresh of 10^%d\n", n_relin_thresh);
	else
		printf("running with relin thresh of 0\n");
	double f_relin_thresh = (!n_relin_thresh)? 0 : pow(10.0, double(n_relin_thresh));
	// verbose

	if(b_do_icra_style_marginals && (b_use_schur || !b_do_marginals)) {
		if(b_use_schur && !b_do_marginals)
			fprintf(stderr, "error: to do ICRA15-style incremental marginals, you need to disable Schur and enable marginals\n");
		else if(!b_do_marginals)
			fprintf(stderr, "error: to do ICRA15-style incremental marginals, you need to enable marginals\n");
		else //if(b_use_schur)
			fprintf(stderr, "error: to do ICRA15-style incremental marginals, you need to disable Schur\n");
		return -1;
	}
	// sanity check

	try {
		CBAOptimizer optimizer(b_verbose, b_use_schur, b_do_marginals, b_do_icra_style_marginals);
		// create the optimizer object

		optimizer.Set_AllBatch(b_all_batch);
		optimizer.Set_ForceIncSchur(b_force_inc_schur);
		optimizer.Set_UpdateThreshold(f_update_thresh);
		optimizer.Set_RelinThreshold(f_relin_thresh);
		optimizer.Set_TrustRadius(f_trust_radius);
		optimizer.Set_TrustRadius_Persistence(b_trust_radius_persistent);
		// set params

#if 1
		if(!b_incremental) {
			typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive, // SE(3) allowed, will be embedded in Sim(3)
				CEdgeP2C3DParsePrimitive, CVertexCamSim3ParsePrimitive, CVertexInvDepthParsePrimitive,
				CEdgeProjSelfParsePrimitive, CEdgeProjOtherParsePrimitive) CMyParsedPrimitives;
			typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
			CMyParseLoop parse_loop(optimizer, n_max_inc_iters, f_inc_nlsolve_thresh, b_save_inc_outputs);
			if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
				fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
				return -1;
			}
		} else {
			typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive, // SE(3) allowed, will be embedded in Sim(3)
				CEdgeP2C3DParsePrimitive, CVertexCamSim3ParsePrimitive, CVertexInvDepthParsePrimitive,
				CEdgeProjSelfParsePrimitive, CEdgeProjOtherParsePrimitive, CConsistencyMarker_ParsePrimitive) CMyParsedPrimitives; // add the incremental parse primitive (or the parse loop could handle it)
			typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
			CMyParseLoop parse_loop(optimizer, n_max_inc_iters, f_inc_nlsolve_thresh, b_save_inc_outputs);
			if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
				fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
				return -1;
			}
		}
		// parsing is timed with the processing
#else
		{
			CTimer t;
			CMyParseLoop_Recordable parse_loop(optimizer, n_max_inc_iters, f_inc_nlsolve_thresh, b_save_inc_outputs);
			if(!b_incremental) {
				typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive, // SE(3) allowed, will be embedded in Sim(3)
					CEdgeP2C3DParsePrimitive, CVertexCamSim3ParsePrimitive, CVertexInvDepthParsePrimitive,
					CEdgeProjSelfParsePrimitive, CEdgeProjOtherParsePrimitive) CMyParsedPrimitives;
				typedef CParserTemplate<CMyParseLoop_Recordable, CMyParsedPrimitives> CMyParser;
				if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
					fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
					return -1;
				}
			} else {
				typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive, // SE(3) allowed, will be embedded in Sim(3)
					CEdgeP2C3DParsePrimitive, CVertexCamSim3ParsePrimitive, CVertexInvDepthParsePrimitive,
					CEdgeProjSelfParsePrimitive, CEdgeProjOtherParsePrimitive, CConsistencyMarker_ParsePrimitive) CMyParsedPrimitives; // add the incremental parse primitive (or the parse loop could handle it)
				typedef CParserTemplate<CMyParseLoop_Recordable, CMyParsedPrimitives> CMyParser;
				if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
					fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
					return -1;
				}
			}
			double f_parsing_time = t.f_Time();
			printf("debug: parsed the data; it took %f sec\n", f_parsing_time);
			parse_loop.Playback();
			double f_processing_time = t.f_Time() - f_parsing_time;
			printf("debug: fed the data to the optimizer; it took %f sec\n", f_processing_time);
		}
		// parsing is done at the beginning, then the parsed primitives are processed
#endif
		// initialize the vertices, add edges

		//optimizer.Dump_State("initial.txt");
		if(!b_incremental) {
			optimizer.Dump_State("initial.txt"); // this is Sim(3)
			optimizer.Dump_State_SE3("initial_SE3.txt"); // this is SE(3) for the viewer
			optimizer.Dump_Poses_SE3("initial_SE3_only-poses.txt"); // this is SE(3) for the viewer
		}
		// debug

		if(!b_incremental)
			optimizer.Optimize(n_max_final_iters, f_final_nlsolve_thresh, f_final_nlsolve_thresh);
		// optimize the system (batch only)

		optimizer.Show_Stats(b_calculate_eigenvalues);
		optimizer.Dump_Graph_SE3("solution.graph"); // this is SE(3) for the viewer
		optimizer.Dump_State("solution.txt"); // this is Sim(3)
		optimizer.Dump_State_SE3("solution_SE3.txt"); // this is SE(3) for the viewer
		optimizer.Dump_Poses_SE3("solution_SE3_only-poses.txt"); // this is SE(3) for the viewer
		// show the timing, save the results

		if(p_s_ground_truth_file)
			CEvaluation::ComputeError("solution_SE3_only-poses.txt", p_s_ground_truth_file);
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: uncaught runtime_error: \"%s\"\n", r_exc.what());
		return -1;
	} catch(std::bad_alloc&) {
		fprintf(stderr, "error: uncaught bad_alloc\n");
		return -1;
	} catch(std::exception &r_exc) {
		fprintf(stderr, "error: uncaught exception: \"%s\"\n", r_exc.what());
		return -1;
	}

	return 0;
}

/**
 *	@page threedeeveeexample SLAM++ 3DV Incremental Bundle Adjustment Example
 *
 *	This example shows how can the incremental dog leg (DL) solver be used.
 *	Its implementation relies on relative coordinate representation of the landmarks
 *	and on the fact, that adding new measurements into the system rarely affects all
 *	of the variables. Even if all variables, are affected, the solution is still correct,
 *	it just automatically switches to doing batch steps. So this works by automatically
 *	extracting the numerical sparsity from the updates (which is problem-specific).
 *
 *	Another important feature, often missed in the reviews, was the possibility to
 *	efficiently recover marginal covariances of the structure points. The proposed
 *	algorithm is exact (not approximative), although there is some minor loss of precision
 *	occuring due to passing the covariances for all of the structure through the Schur
 *	complement, in conjunction with the limited precision of floating point arithmetics.
 *	Compared to recursive formula, the algorithm is fast (order of magnitude speedup).
 *	The ICRA 2015 incremental covariance recovery algorithm is unfortunately not practical
 *	for BA-type problems due to very high rank of the updates at each step.
 *
 *	This solver comes with quite many thresholds. An important one is the trust radius
 *	setting (for more details about dog leg, please refer to Madsen's 2004 paper
 *	"Methods for Nonlinear Least Squares"). Trust radius is usually set to a large number
 *	in the literature and left on the solver to reduce automatically (since the failed
 *	steps are much cheaper than in Levenberg-Marquardt type solvers). However, due to
 *	the highly nonlinear nature of the SLAM problems, we found that doing so, the solver
 *	often takes a step of ridiculous size, to end up in a local minimum. Therefore, we
 *	recommend choosing the initial step size that best suits the scale of your problem.
 *	We used value of 0.2 for all datasets for the 3DV paper, with the exception of Boreas,
 *	where the value of 2.0 gave better results.
 *
 *	Furthermore, for incremental processing, persistent step size (maintained between
 *	the temporal steps) can lead to better results.
 *
 *	Finally, the Cholesky solver requires the matrix to be positive-definite. To cope
 *	with potential numerical issues, you can use <tt>-dp</tt> to set the damping parameter.
 *	Note that this is only intended to simulate the modified Cholesky algorithm and does
 *	not serve as a Levenberg-Marquardt-style damping (for the problems tackled, the LM value
 *	would be on the order of \f$10^6\f$ to \f$10^8\f$; here, value of \f$1.0\f$ suffices).
 *	This is something that could be solved by implementation of better factorization
 *	algorithm, for which we did not have enough time. This is alternatively also solved
 *	by taking indefinite steps, a strategy we have not found terribly useful in practice
 *	(refer to Rosen's 2014 paper "RISE: An incremental trust-region method for robust online
 *	sparse least-squares estimation").
 *
 */

/*
 *	end-of-file
 */
