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
 *	@file src/ba_parameter_acra/MainL.cpp
 *	@brief experiments for the 2015 ACRA paper "The Effect of Different Parameterisations in Incremental
 *		Structure from Motion" with Vincent Lui, Viorela Ila, Tom Drummond and Robert Mahony.
 *	@author -tHE SWINe-
 *	@date 2015-08-05
 */

#include <stdio.h> // printf
#include "ba_parameter_acra/BAOptimizer.h" // BA types
// note that nothing else from SLAM++ needs to be included

#include "slam/Parser.h" // want to be able to parse .graph files
#include "slam_app/ParsePrimitives.h" // reuse SLAM++ standard parse primitives
// we need this to parse .graph files

#include "slam/ConfigSolvers.h"
#include "slam/Sim3SolverBase.h"
#include "ba_parameter_acra/Sim3_ParsePrimitives.h"
#include "slam/ErrorEval.h"
#include "slam/Eigenvalues.h"

/**
 *	@brief consistency marker type (passed as an edge)
 */
class CConsistencyMarker {};

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

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_optimizer is reference to the optimizer to be filled with edges and vertices
	 *	@param[in] n_inc_nls_iter_num
	 *	@param[in] f_inc_nlsolve_thresh
	 */
	CMyParseLoop(CBAOptimizer &r_optimizer, int n_inc_nls_iter_num = 1,
		double f_inc_nlsolve_thresh = .005)
		:m_r_optimizer(r_optimizer), m_n_inc_nls_iter_num(n_inc_nls_iter_num),
		m_f_inc_nlsolve_thresh(f_inc_nlsolve_thresh)
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
			m_f_inc_nlsolve_thresh); // handle incremental optimization

		/*{ // in case this is an incremental solve step, dump the dx vector
			static size_t n_iter_num = 0;
			++ n_iter_num;
			char p_s_filename[256];
			sprintf(p_s_filename, "dx_iter_%05" _PRIsize "_SE3_solution.txt", n_iter_num - 1); // zero based!
			m_r_optimizer.Dump_State_SE3(p_s_filename);
		}*/
	}
};

int n_dummy_param = 0;

std::vector<double> Get_Eigenvalues(const cs *p_matrix, size_t n_eig_num, bool b_smallest)
{
	return SpSym_Eigenvalues(p_matrix, n_eig_num, b_smallest);
}

class CEvaluation : public CErrorEvaluation<false> {
public:
	static Eigen::Vector6d v_Invert_SE3_Pose(const Eigen::Vector6d &r_v)
	{
		Eigen::Vector6d v = r_v; // copy intended but can't do it in an argument because x86 msvc won't align arguments
		Eigen::Quaterniond q;
		C3DJacobians::AxisAngle_to_Quat(v.tail<3>(), q);
		q = q.conjugate();
		v.head<3>() = -q._transformVector(v.head<3>());
		Eigen::VectorBlock<Eigen::Vector6d, 3> v_rot_part = v.tail<3>(); // g++ requires a temporary
		C3DJacobians::Quat_to_AxisAngle(q, v_rot_part);
		return v;
	}

	static bool Load_SE3_PoseSet(const char *p_s_filename, std::vector<_TyVectorUnalign> &r_dest, bool b_inverse_poses = true)
	{
		r_dest.clear();
		// !!

		FILE *p_fr;
		if(!(p_fr = fopen(p_s_filename, "r")))
			return false;
		while(!feof(p_fr)) {
			_TyVectorUnalign v;
			switch(fscanf(p_fr, "%lf %lf %lf %lf %lf %lf\n", &v(0), &v(1), &v(2), &v(3), &v(4), &v(5))) {
			case 6:
				if(b_inverse_poses)
					v = v_Invert_SE3_Pose(v);
				r_dest.push_back(v);
				break;
			case 0:
				break; // eof?
			default:
				fclose(p_fr);
				return false;
			}
		}
		if(ferror(p_fr)) {
			fclose(p_fr);
			return false;
		}
		fclose(p_fr);

		return true;
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
		if(est_vertices.size() != gt_vertices.size()) {
			fprintf(stderr, "error: the number of poses in the estimated trajectory ("
				PRIsize ") does not correspond to the ground truth (" PRIsize ")\n", est_vertices.size(), gt_vertices.size());
			return false;
		}
		// the sizes must match

		std::pair<_TyVector, double> t_transform = v_Align_PoseSets(est_vertices, gt_vertices); // transform * est approaches gt
		// compute a rigid transformation that puts ground truth vertices close to the solution vertices

		std::vector<_TyVectorUnalign> vertices(est_vertices.size()); // copy intended
		for(size_t i = 0, n = est_vertices.size(); i < n; ++ i) {
			_TyVector v_aligned_pose = est_vertices[i];
			v_aligned_pose.head<3>() *= t_transform.second; // apply scale
			_TyPoseOpsImpl::Relative_to_Absolute(t_transform.first, v_aligned_pose, v_aligned_pose); // arg order! relative pose is on the left!
			vertices[i] = v_aligned_pose;
		}
		// apply transform on the estimated poses to align them with the ground truth

		FILE *p_fw;
		if((p_fw = fopen("solution_SE3_gt-camera-poses_est-aligned.txt", "w"))) {
			for(size_t i = 0, n = vertices.size(); i < n; ++ i) {
				_TyVector v_aligned_pose = vertices[i];
				v_aligned_pose = v_Invert_SE3_Pose(v_aligned_pose); // invert for the viewer
				fprintf(p_fw, "%.10f %.10f %.10f %g %g %g\n", v_aligned_pose(0), v_aligned_pose(1),
					v_aligned_pose(2), v_aligned_pose(3), v_aligned_pose(4), v_aligned_pose(5));
			}
			fclose(p_fw);
		}
		if((p_fw = fopen("solution_SE3_gt-inverse.txt", "w"))) {
			for(size_t i = 0, n = vertices.size(); i < n; ++ i) {
				_TyVector v_aligned_pose = gt_vertices[i];
				v_aligned_pose = v_Invert_SE3_Pose(v_aligned_pose); // invert for the viewer
				fprintf(p_fw, "%.10f %.10f %.10f %g %g %g\n", v_aligned_pose(0), v_aligned_pose(1),
					v_aligned_pose(2), v_aligned_pose(3), v_aligned_pose(4), v_aligned_pose(5));
			}
			fclose(p_fw);
		}
		// debug - write the output for the graph viewer

		t_transform.first = Eigen::Vector6d::Zero();
		t_transform.second = 1.0;
		// no transform, we already put the vertices into a common frame

		double f_pos_ate, f_rot_ate, f_pos2_ate, f_rot2_ate;
		double f_pos_rpe, f_rot_rpe, f_pos2_rpe, f_rot2_rpe;
		double f_pos_rpe_aa, f_rot_rpe_aa, f_pos2_rpe_aa, f_rot2_rpe_aa;
		// results of different error metrics, direct and squared

		Compute_AbsoluteTrajectoryError(vertices, gt_vertices,
			f_pos_ate, f_rot_ate, f_pos2_ate, f_rot2_ate);
		// calculate the ATEs

		Compute_RelativePoseError(vertices, gt_vertices, 1,
			f_pos_rpe, f_rot_rpe, f_pos2_rpe, f_rot2_rpe);
		// calculate the RPEs

		Compute_RelativePoseError_AllToAll(vertices, gt_vertices,
			f_pos_rpe_aa, f_rot_rpe_aa, f_pos2_rpe_aa, f_rot2_rpe_aa);
		// calculate the RPE-AAs

		printf("cumulative ATE of the aligned camera poses is %f, %f\n", f_pos_ate, f_rot_ate);

		f_pos_ate /= gt_vertices.size();
		f_rot_ate /= gt_vertices.size();
		f_pos2_ate /= gt_vertices.size();
		f_rot2_ate /= gt_vertices.size();

		printf("per vert. ATE of the aligned camera poses is %f, %f\n", f_pos_ate, f_rot_ate);
		printf("RMSE ATE of the aligned camera poses is %f, %f\n", sqrt(f_pos2_ate), sqrt(f_rot2_ate));

		printf("\ncumulative RPE of the aligned camera poses is %f, %f\n", f_pos_rpe, f_rot_rpe);

		f_pos_rpe /= gt_vertices.size();
		f_rot_rpe /= gt_vertices.size();
		f_pos2_rpe /= gt_vertices.size();
		f_rot2_rpe /= gt_vertices.size();

		printf("per vert. RPE of the aligned camera poses is %f, %f\n", f_pos_rpe, f_rot_rpe);
		printf("RMSE RPE of the aligned camera poses is %f, %f\n", sqrt(f_pos2_rpe), sqrt(f_rot2_rpe));

		printf("\ncumulative RPE all-to-all of the aligned camera poses is %f, %f\n", f_pos_rpe_aa, f_rot_rpe_aa);

		f_pos_rpe_aa /= gt_vertices.size();
		f_rot_rpe_aa /= gt_vertices.size();
		f_pos2_rpe_aa /= gt_vertices.size();
		f_rot2_rpe_aa /= gt_vertices.size();

		printf("per vert. RPE all-to-all of the aligned camera poses is %f, %f\n", f_pos_rpe_aa, f_rot_rpe_aa);
		printf("RMSE RPE all-to-all of the aligned camera poses is %f, %f\n", sqrt(f_pos2_rpe_aa), sqrt(f_rot2_rpe_aa));

		printf("note that all the rotation errors are in degrees\n");

		return true;
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
#ifdef RUN_STANDALONE
int main(int n_arg_num, const char **p_arg_list) // standalone app
#else // RUN_STANDALONE
int mainL(int n_arg_num, const char **p_arg_list) // started from Vincent's code
#endif // RUN_STANDALONE
{
	bool b_verbose = true, b_incremental = false;
	int n_update_thresh = 0;
	const char *p_s_input_file = 0, *p_s_ground_truth_file = 0;
	int n_max_inc_iters = 1, n_max_final_iters = 5;
	double f_inc_nlsolve_thresh = .005, f_final_nlsolve_thresh = .005;
	for(int i = 1; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			printf("use: %s [-i|--input <input-graph-file>] [-q|--quiet] [-inc|--incremental]\n"
				   "     [-dxt|--dx-thresh <negative-pervert-exponent>]\n", p_arg_list[0]);
			return 0;
		} else if(!strcmp(p_arg_list[i], "--quiet") || !strcmp(p_arg_list[i], "-q"))
			b_verbose = false;
		else if(!strcmp(p_arg_list[i], "--incremental") || !strcmp(p_arg_list[i], "-inc"))
			b_incremental = true;
		else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return -1;
		} else if(!strcmp(p_arg_list[i], "--input") || !strcmp(p_arg_list[i], "-i"))
			p_s_input_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--ground-truth") || !strcmp(p_arg_list[i], "-gt"))
			p_s_ground_truth_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--dx-threshold") || !strcmp(p_arg_list[i], "-dxt"))
			n_dummy_param = n_update_thresh = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--max-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mnsi"))
			n_max_inc_iters = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-nset"))
			f_inc_nlsolve_thresh = atof(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--max-final-nonlinear-solve-iters") || !strcmp(p_arg_list[i], "-mfnsi"))
			n_max_final_iters = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--final-nonlinear-solve-error-thresh") || !strcmp(p_arg_list[i], "-fnset"))
			f_final_nlsolve_thresh = atof(p_arg_list[++ i]);
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

	if(n_update_thresh)
		printf("running with dx thresh of 10^%d\n", n_update_thresh);
	// verbose

	CBAOptimizer optimizer(b_verbose);
	// create the optimizer object

	if(!b_incremental) {
		typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive, // SE(3) not allowed now, all is in Sim(3)
			CEdgeP2C3DParsePrimitive, CVertexCamSim3ParsePrimitive, CVertexInvDepthParsePrimitive,
			CEdgeProjSelfParsePrimitive, CEdgeProjOtherParsePrimitive) CMyParsedPrimitives;
		typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
		CMyParseLoop parse_loop(optimizer);
		if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
			fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
			return -1;
		}
	} else {
		typedef MakeTypelist(CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive, // SE(3) not allowed now, all is in Sim(3)
			CEdgeP2C3DParsePrimitive, CVertexCamSim3ParsePrimitive, CVertexInvDepthParsePrimitive,
			CEdgeProjSelfParsePrimitive, CEdgeProjOtherParsePrimitive, CConsistencyMarker_ParsePrimitive) CMyParsedPrimitives; // add the incremental parse primitive (or the parse loop could handle it)
		typedef CParserTemplate<CMyParseLoop, CMyParsedPrimitives> CMyParser;
		CMyParseLoop parse_loop(optimizer, n_max_inc_iters, f_inc_nlsolve_thresh);
		if(!CMyParser().Parse(p_s_input_file, parse_loop)) {
			fprintf(stderr, "error: failed to parse \'%s\'\n", p_s_input_file);
			return -1;
		}
	}
	// initialize the vertices, add edges

	//optimizer.Dump_State("initial.txt");
	//optimizer.Dump_Graph("initial.graph");
	// debug

	if(!b_incremental)
		optimizer.Optimize(n_max_final_iters, f_final_nlsolve_thresh);
	// optimize the system (batch only)

	optimizer.Show_Stats();
	optimizer.Dump_State("solution.txt"); // this is Sim(3)
	optimizer.Dump_State_SE3("solution_SE3.txt"); // this is SE(3) for the viewer
	optimizer.Dump_Poses_SE3("solution_SE3_only-poses.txt"); // this is SE(3) for the viewer
	// show the timing, save the results

	if(p_s_ground_truth_file)
		CEvaluation::ComputeError("solution_SE3_only-poses.txt", p_s_ground_truth_file);

#if defined(_WIN32) || defined(_WIN64)
	system("graphview_"); // launch graph viewer in lukas' computer
#endif // _WIN32 || _WIN64

	return 0;
}

/*
 *	end-of-file
 */
