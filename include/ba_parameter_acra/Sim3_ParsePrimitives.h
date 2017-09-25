/*
								+-----------------------------------+
								|                                   |
								|      ***  Sim(3) Parser  ***      |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|      Sim3_ParsePrimitives.h       |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __SIM3_GRAPH_PARSER_PRIMITIVES_INCLUDED
#define __SIM3_GRAPH_PARSER_PRIMITIVES_INCLUDED

/**
 *	@file include/ba_parameter_acra/Sim3_ParsePrimitives.h
 *	@brief plugins for a simple .graph file parser
 *	@author -tHE SWINe-
 *	@date 2015-08-06
 */

#include "slam/Parser.h"
#include "slam/Sim3SolverBase.h" 

/**
 *	@brief Sim3 pose parse primitive handler
 */
class CVertexSim3ParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throws(std::bad_alloc)
	{
		r_token_name_map["VERTEX:SIM3"] = n_assigned_id;
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
		int n_pose_id;
		Eigen::Matrix<double, 7, 1> vertex;
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf",
		   &n_pose_id, &vertex(0), &vertex(1), &vertex(2), &vertex(3), &vertex(4),
		   &vertex(5), &vertex(6)) != 1 + 7) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CSim3Jacobians::TSim3 pose;// = CSim3Jacobians::t_Exp(vertex); // not Sim3 yet
		pose.v_translation = vertex.head<3>();
		C3DJacobians::AxisAngle_to_Quat(vertex.tail<4>().head<3>(), pose.t_rotation);
		// get a pose

		Eigen::Quaterniond quat = pose.t_rotation.inverse();
		Eigen::Vector3d c = quat * -pose.v_translation;
		Eigen::Vector3d axis;
		C3DJacobians::Quat_to_AxisAngle(quat, axis);
		CParserBase::TVertexCam3D vert(n_pose_id, c[0],
			c[1], c[2], axis(0), axis(1), axis(2), .5, .5, 0, 0, 0);
		// convert to SE(3) and invert it

		r_parse_loop.InitializeVertex(vert);

		return true;
	}
};

/**
 *	@brief Sim(3) camera vertex initializer ("VERTEX_CAM:SIM3" in the datafile)
 */
struct TVertexCamSim3 : public CParserBase::CParseEntity {
	int m_n_id; /**< @brief vertex id */
	Eigen::Matrix<double, 12, 1> m_v_position; /**< @brief vertex position (the state vector) */

	/**
	 *	@brief default constructor
	 */
	inline TVertexCamSim3()
	{}

	/**
	 *	@brief constructor
	 *
	 *	@param[in] n_node_id is (zero-based) index of the node
	 *	@param[in] p_x is x position
	 *	@param[in] p_y is y position
	 *	@param[in] p_z is z position
	 *	@param[in] ax is x axis angle
	 *	@param[in] ay is y axis angle
	 *	@param[in] az is z axis angle
	 *	@param[in] s is scale
	 *	@param[in] fx is focal length
	 *	@param[in] fy is focal length
	 *	@param[in] cx is principal point in x axis
	 *	@param[in] cy is principal point in y axis
	 *	@param[in] d is first distortion coefficient of the radial distortion
	 *
	 *	@note The vertices are only used as ground truth. Those are mostly not processed.
	 */
	inline TVertexCamSim3(int n_node_id, double p_x, double p_y, double p_z,
		double ax, double ay, double az, double s, double fx, double fy,
		double cx, double cy, double d)
		:m_n_id(n_node_id)
	{
		d *= .5 * (fx + fy)/*sqrt(fx * fy)*/; // since post-ICRA2015 release the distortion is scaled by focal length in the internal representation
		m_v_position << p_x, p_y, p_z, ax, ay, az, s, fx, fy, cx, cy, d;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 *	@brief Sim3 camera pose and intrinsics parse primitive handler
 */
class CVertexCamSim3ParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throws(std::bad_alloc)
	{
		r_token_name_map["VERTEX_CAM:SIM3"] = n_assigned_id;
	}

	static inline Eigen::Matrix3d t_Cross(const Eigen::Vector3d &r)
	{
		Eigen::Matrix3d t_cross;
		t_cross <<  0, -r(2),  r(1),
			     r(2),     0, -r(0),
			    -r(1),  r(0),     0;
		return t_cross;
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
		int n_pose_id;
		Eigen::Matrix<double, 7, 1> vertex;
		Eigen::Matrix<double, 5, 1> intrinsics;
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   &n_pose_id, &vertex(0), &vertex(1), &vertex(2), &vertex(3), &vertex(4),
		   &vertex(5), &vertex(6), &intrinsics(0), &intrinsics(1), &intrinsics(2),
		   &intrinsics(3), &intrinsics(4)) != 1 + 7 + 5) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		/*{
			const CSim3Jacobians::TSim3 t_pose = CSim3Jacobians::t_Exp_SE3(vertex); // note that this is in fact not on the manifold but it is good enough for test data
			Eigen::Vector7d v_result = CSim3Jacobians::v_Log_SE3(t_pose);
			double f_error = (v_result - vertex).norm();
			printf("log map error: %g\n", f_error);
			// test SE(3) exp/log
		}*/
		/*{
			const double f_scale = vertex(6);
			const Eigen::Vector3d t = vertex.head<3>(), w = vertex.tail<4>().head<3>();

			CSim3Jacobians::TSim3 t_pose;
			const double f_angle = C3DJacobians::f_AxisAngle_to_Quat(w, t_pose.t_rotation);
			_ASSERTE(f_angle >= 0); // positive angle, possibly not bounded (axis angle can potentially have norm of more than 2pi)
			const double f_exp_scale = t_pose.f_scale = exp(f_scale);

			const bool b_zero_angle = f_angle < 1e-10, b_zero_scale = fabs(f_scale) < 1e-10;
			if(b_zero_angle) {
				if(b_zero_scale) {
					t_pose.v_translation = t;
					// f_a = 1, f_b = 0, f_c = 0

					// f_a = 1 http://www.wolframalpha.com/input/?i=lim+%28exp%28s%29-1%29%2Fs%2C+s%3D0
					// f_b = 1/2 http://www.wolframalpha.com/input/?i=lim+%281-cos%28a%29%29%2F%28a*a%29%2C+a%3D0 and http://www.wolframalpha.com/input/?i=lim+%281%2Bexp%28s%29*%28s-1%29%29%2F%28s*s%29%2C+s%3D0
					// f_c = 1/6 http://www.wolframalpha.com/input/?i=lim+%28exp%28s%29*%282-2*s%2Bs*s%29-2%29%2F%282*s*s*s%29%2C+s%3D0 and http://www.wolframalpha.com/input/?i=lim+%28a-sin%28a%29%29%2F%28a*a*a%29%2C+a%3D0
				} else {
					t_pose.v_translation = t * ((f_exp_scale - 1) / f_scale);
					// f_a = (f_exp_scale - 1) / f_scale, f_b = 0, f_c = 0

					// f_a = (f_exp_scale - 1) / f_scale // unaffected by zero angle in any way
					// f_b = (1 + f_exp_scale * (f_scale - 1)) / (f_scale * f_scale) // but f_b is nonzero! http://www.wolframalpha.com/input/?i=lim+%28a*%281-exp%28s%29*cos%28a%29%29%2Bexp%28s%29*s*sin%28a%29%29%2F%28a*%28a*a%2Bs*s%29%29%2C+a%3D0
					// f_c = (f_exp_scale * (2 - 2 * f_scale + f_scale * f_scale) - 2) / (2 * f_scale * f_scale * f_scale); // but f_c is nonzero! http://www.wolframalpha.com/input/?i=lim+%28exp%28s%29+-+1%29%2F%28s*a*a%29-exp%28s%29*sin%28a%29%2F%28a*%28s*s%2Ba*a%29%29-s*%28exp%28s%29cos%28a%29-1%29%2F%28a*a*%28s*s%2Ba*a%29%29%2C+a%3D0
				}
			} else {
				double f_a, f_b, f_c;
				if(b_zero_scale) {
					f_a = 1; // this is ok by http://www.wolframalpha.com/input/?i=lim+%28exp%28s%29-1%29%2Fs%2C+s%3D0
					const double f_angle_sq = f_angle * f_angle;
					f_b = (1 - cos(f_angle)) / f_angle_sq; // this is ok by http://www.wolframalpha.com/input/?i=lim+%28a*%281-exp%28s%29*cos%28a%29%29%2Bexp%28s%29*s*sin%28a%29%29%2F%28a*%28a*a%2Bs*s%29%29%2C+s%3D0
					f_c = (f_angle - sin(f_angle)) / (f_angle_sq * f_angle); // less division is probably better // this is ok by http://www.wolframalpha.com/input/?i=lim+%28exp%28s%29+-+1%29%2F%28s*a*a%29-exp%28s%29*sin%28a%29%2F%28a*%28s*s%2Ba*a%29%29-s*%28exp%28s%29cos%28a%29-1%29%2F%28a*a*%28s*s%2Ba*a%29%29%2C+s%3D0
				} else {
					f_a = (f_exp_scale - 1) / f_scale;
					const double f_angle_sq = f_angle * f_angle, f_1_e_cos = 1 - f_exp_scale * cos(f_angle),
						f_inv_denom = 1 / (f_angle * (f_angle_sq + f_scale * f_scale)), f_e_sin = f_exp_scale * sin(f_angle);
					f_b = (f_angle * f_1_e_cos + f_scale * f_e_sin) * f_inv_denom;
					f_c = f_a / f_angle_sq - (f_e_sin - f_scale * f_1_e_cos / f_angle) * f_inv_denom;
					// as due to C. Allen–Blanchette, S. Leonardos, and J. Gallier, "Motion Interpolation in SIM(3)", 2014
				}
				Eigen::Vector3d v_cross = w.cross(t);
				t_pose.v_translation = t * f_a + v_cross * f_b + w.cross(v_cross) * f_c;
			}

			double f_error = (t_pose.v_Log() - vertex).norm();
			printf("exp map error: %-12g\n", f_error);
		}*/
		//CSim3Jacobians::Test_LnExp(vertex);
		//CSim3Jacobians::Test_Sim3(vertex);
		// all seems to be in order

#if 0 // convert to SE(3)
		CSim3Jacobians::TSim3 pose;// = CSim3Jacobians::t_Exp(vertex); // not log of Sim3!
		pose.v_translation = vertex.head<3>();
		C3DJacobians::AxisAngle_to_Quat(vertex.tail<4>().head<3>(), pose.t_rotation);
		// get a pose

		Eigen::Quaterniond quat = pose.t_rotation.inverse();
		Eigen::Vector3d c = quat * -pose.v_translation;
		Eigen::Vector3d axis;
		C3DJacobians::Quat_to_AxisAngle(quat, axis);
		CParserBase::TVertexCam3D vert(n_pose_id, c[0],
			c[1], c[2], axis(0), axis(1), axis(2), intrinsics(0),
			intrinsics(1), intrinsics(2), intrinsics(3), intrinsics(4));
		// convert to SE(3) and invert it
#else // 0 // run in Sim(3)
		CSim3Jacobians::TSim3 pose(vertex, CSim3Jacobians::TSim3::from_tRs_vector);// = CSim3Jacobians::t_Exp(vertex); // not log of Sim3!
		/*pose.v_translation = vertex.head<3>();
		C3DJacobians::AxisAngle_to_Quat(vertex.tail<4>().head<3>(), pose.t_rotation);
		pose.f_scale = vertex(6); // was not needed before*/
		vertex = pose.v_Log(); // this is not inverse, but it still does not use the same internal representation
		// get a pose, logarithm it

		TVertexCamSim3 vert(n_pose_id, vertex(0), vertex(1), vertex(2),
			vertex(3), vertex(4), vertex(5), vertex(6), intrinsics(0),
			intrinsics(1), intrinsics(2), intrinsics(3), intrinsics(4));
		// store as Sim(3), do not invert
#endif // 0

		r_parse_loop.InitializeVertex(vert);

		return true;
	}
};

struct TVertexInvDepth : public CParserBase::CParseEntity {
	size_t n_vertex_id;
	size_t n_camera_id;
	Eigen::Vector3d v_position;

	TVertexInvDepth(size_t _n_vertex_id = -1, size_t _n_camera_id = -1,
		const Eigen::Vector3d &r_v_position = Eigen::Vector3d())
		:n_vertex_id(_n_vertex_id), n_camera_id(_n_camera_id), v_position(r_v_position)
	{}
};

/**
 *	@brief 3D inverse depth vertex parse primitive handler
 */
class CVertexInvDepthParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throws(std::bad_alloc)
	{
		r_token_name_map["VERTEX:INVD"] = n_assigned_id;
	}

	class CAutoFILE { // todo - put it to debug, its pretty neat
	protected:
		FILE *m_p_f;

	public:
		CAutoFILE(const char *p_s_filename, const char *p_s_mode)
			:m_p_f(fopen(p_s_filename, p_s_mode))
		{}

		~CAutoFILE()
		{
			if(m_p_f)
				fclose(m_p_f);
		}

		operator FILE *()
		{
			return m_p_f;
		}
	};

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
		int n_vertex_id, n_camera_id;
		Eigen::Vector3d vertex;
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf",
		   &n_vertex_id, &n_camera_id, &vertex(0), &vertex(1), &vertex(2)) != 2 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		//static CAutoFILE raw_verts("raw_verts.txt", "w");
		//fprintf(raw_verts, "%f %f %f\n", vertex(0) / vertex(2), vertex(1) / vertex(2), 1 / vertex(2));

		TVertexInvDepth vert(n_vertex_id, n_camera_id, vertex);

		r_parse_loop.InitializeVertex(vert);

		return true;
	}
};

struct TEdgeP2CSim3G : public CParserBase::TEdgeP2C3D {
	inline TEdgeP2CSim3G()
	{}

	inline TEdgeP2CSim3G(size_t n_node_0, size_t n_node_1,
		double f_delta_x, double f_delta_y, const double *p_upper_matrix_2x2)
		:CParserBase::TEdgeP2C3D(n_node_0, n_node_1, f_delta_x, f_delta_y, p_upper_matrix_2x2)
	{}
};

/**
 *	@brief projection of a point onto the owner camera parse primitive handler
 */
class CEdgeProjSelfParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throws(std::bad_alloc)
	{
		r_token_name_map["EDGE_PROJ_SELF"] = n_assigned_id;
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
		int n_landmark_id, n_camera_id;
		Eigen::Matrix<double, 5, 1> v_edge;
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf",
		   &n_landmark_id, &n_camera_id, &v_edge(0), &v_edge(1), &v_edge(2), &v_edge(3), &v_edge(4)) != 2 + 5) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		TEdgeP2CSim3G edge(n_landmark_id, n_camera_id, v_edge(0), v_edge(1), &v_edge(2));

		r_parse_loop.AppendSystem(edge);

		return true;
	}
};

/**
 *	@brief projection of a point onto a non-owner camera parse primitive handler
 */
class CEdgeProjOtherParsePrimitive {
public:
	/**
	 *	@brief enumerates all tokens that identify this parsed primitive
	 *
	 *	@param[in,out] r_token_name_map is map of token names
	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
	 */
	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
		int n_assigned_id) // throws(std::bad_alloc)
	{
		r_token_name_map["EDGE_PROJ_OTHER"] = n_assigned_id;
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
		//std::string s_field; // not needed after all
		std::vector<size_t> integers;
		std::vector<double> doubles; // could be Eigen::Matrix<double, 5, 1> and size_t doubles_size to avoid dynamic allocation
		size_t n_int_num = 2; // upper bound of integers.size(), still needed
		for(size_t n_field = 0, b = 0, e = r_s_line.length(); b < e; ++ n_field) {
			while(b < e && isspace(uint8_t(r_s_line[b])))
				++ b;
			// skip space

			size_t n_field_begin = b;
			while(b < e && !isspace(uint8_t(r_s_line[b])))
				++ b;
			size_t n_field_end = b;
			if(n_field_end == n_field_begin) {
				_ASSERTE(b == e);
				break;
			}
			// skip field contents

			//s_field.clear();
			//s_filed.insert(s_field.begin(), r_s_line.begin() + n_field_begin, r_s_line.begin() + n_field_end);
			const char *p_s_field = r_s_line.c_str() + n_field_begin; // delimited by a terminating null or by whitespace, sscanf() will skip neither
			// grab the field

			if(n_field < n_int_num) {
				size_t n_int;
				if(sscanf(p_s_field, PRIsize, &n_int) != 1)
					return false;
				integers.push_back(n_int);
				if(n_field == 1)
					n_int_num += integers.back(); // the second number is the count of cameras
			} else {
				double f_double;
				if(sscanf(p_s_field, "%lf", &f_double) != 1)
					return false;
				doubles.push_back(f_double);
			}
			// decide to which list the field belongs (could split this to two loops, the second one without branches)
		}
		// parse the edge (a bit complicated on account of it being a variable length edge)

		if(integers.size() < 4 || integers.size() != 2 + integers[1] || doubles.size() != 5)
			return false;
		// there must be two integers + the size of the camera chain (at least 2 more)
		// there must be five doubles (x y cov_00 cov_01 cov_11)

		size_t n_landmark_id = integers.front(); // the first number is the landmark id
		size_t n_owner_camera_id = integers.back(); // the owner camera is the last in the chain
		size_t n_observing_camera_id = integers[2]; // the observing camera is the first in the chain (the third int, after landmark id and chain length)

		TEdgeP2CSim3G edge(n_landmark_id, n_observing_camera_id, doubles[0], doubles[1], &doubles[2]);

		r_parse_loop.AppendSystem(edge);

		return true;
	}
};

#endif // !__SIM3_GRAPH_PARSER_PRIMITIVES_INCLUDED
