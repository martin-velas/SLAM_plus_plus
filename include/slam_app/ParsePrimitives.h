/*
								+-----------------------------------+
								|                                   |
								|      ***  .graph Parser  ***      |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2013  |
								|                                   |
								|         ParsePrimitives.h         |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __GRAPH_PARSER_PRIMITIVES_INCLUDED
#define __GRAPH_PARSER_PRIMITIVES_INCLUDED

/**
 *	@file include/slam_app/ParsePrimitives.h
 *	@brief plugins for a simple .graph file parser
 *	@author -tHE SWINe-
 *	@date 2012-02-06
 */

#include "slam/Parser.h"
#include "slam/2DSolverBase.h" // CBase2DSolver::Absolute_to_Relative() and such
#include "slam/3DSolverBase.h" // CBase3DSolver::Absolute_to_Relative() and such

/** \addtogroup parser
 *	@{
 */

/**
 *	@brief an example parse primitive handler
 */
class CIgnoreParsePrimitive {
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
		r_token_name_map["EQUIV"] = n_assigned_id;
		// list of standard tokens that are ignored
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
	static bool Parse_and_Dispatch(size_t UNUSED(n_line_no), const std::string &UNUSED(r_s_line),
		const std::string &UNUSED(r_s_token), _TyParseLoop &UNUSED(r_parse_loop))
	{
		// here, a primitive of type r_s_token should be parsed from r_s_line
		// and if successful, passed to r_parse_loop by calling InitializeVertex()
		// for vertex types or AppendSystem() for edge types

		return true;
	}
};

/**
 *	@brief 2D pose edge parse primitive handler
 */
class CEdge2DParsePrimitive {
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
		r_token_name_map["EDGE2"] = n_assigned_id;
		r_token_name_map["EDGE_SE2"] = n_assigned_id;
		r_token_name_map["EDGE"] = n_assigned_id;
		r_token_name_map["ODOMETRY"] = n_assigned_id;
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
		// here, a primitive of type r_s_token should be parsed from r_s_line
		// and if successful, passed to r_parse_loop

		int p_pose_idx[2];
		double p_measurement[3];
		double p_matrix[6];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1, p_measurement + 2,
		   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4,
		   p_matrix + 5) != 2 + 3 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		/*const double ro = 0.02, mean = 0;
		for(int i = 0; i < 3; ++ i) {
			double x0 = (rand() + 1.0) / (RAND_MAX + 1.0); // avoid 0.0 as that will result in -inf
			double x1 = rand() / double(RAND_MAX);
			p_measurement[i] += mean + (sqrt(-2 * log(x0) / log(exp(1.0))) * cos(2 * M_PI * x1)) * ro; // Box-Muller
		}*/
		// add some stronger gaussian noise

		if(p_pose_idx[0] < p_pose_idx[1]) {
			//fprintf(stderr, "warning: doing inversion on possibly straight edge. contact your dataset master\n");
			// not any more, you don't

			if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[3]) < 1e-5 || fabs(p_matrix[5]) < 1e-5) {
				if(fabs(p_matrix[0]) > 1e-5 && fabs(p_matrix[2]) > 1e-5 && fabs(p_matrix[3]) > 1e-5) {
					bool b_warned_about_reverse_order = false;
					// this has the drawback of only warning once per application execution.
					// instead it should warn once per parser instance.
					// it actually happens twice because this function template has two different
					// instances (for parse loop and for dataset peeker). but still twice per app.

					if(!b_warned_about_reverse_order) {
						fprintf(stderr, "warning: the inverse sigma matrix is in the french order."
							" contact your dataset master\n");
						b_warned_about_reverse_order = true;
					}
				} else {
					fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
						" contact your dataset master\n");
				}
			}
			// make sure inverse sigma is diagonal and ordered as expected

			//if((*p_tok_it).second == token_Edge2D) {
				CParserBase::TEdge2D edge(p_pose_idx[0], p_pose_idx[1],
					p_measurement[0], p_measurement[1], p_measurement[2], p_matrix);
				// process the measurement

				r_parse_loop.AppendSystem(edge);
				// append the measurement to the system, or something
			/*} else {
				TOdometry2D odo(p_pose_idx[0], p_pose_idx[1], p_measurement[0],
					p_measurement[1], p_measurement[2], p_matrix);
				// process the measurement

				r_parse_loop.AppendSystem(odo);
				// t_odo - append the measurement to the system, or something
			}*/
		} else {
			// the manhattan datasets have poses in descending order (e.g. EDGE 1 0), the edges need to be inverted here
			// in case the order is ascending, it is possible that the inversion is inadvertent

			bool b_is_g2o_order = false;
			if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[2]) < 1e-5 || fabs(p_matrix[3]) < 1e-5) {
				if(fabs(p_matrix[0]) > 1e-5 && fabs(p_matrix[3]) > 1e-5 && fabs(p_matrix[5]) > 1e-5) {
					b_is_g2o_order = true;
					// use the sigma as is then

					static bool b_warned_about_reverse_order = false;
					// this has the drawback of only warning once per application execution.
					// instead it should warn once per parser instance.
					// it actually happens twice because this function template has two different
					// instances (for parse loop and for dataset peeker). but still twice per app.

					if(!b_warned_about_reverse_order) {
						fprintf(stderr, "warning: the inverse sigma matrix is in the g2o order."
							" however, this dataset requires edge inversion and french order was expected."
							" contact your dataset master\n");
						b_warned_about_reverse_order = true;
					}
				} else {
					fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
						" contact your dataset master\n");
				}
			}
			// make sure inverse sigma is diagonal and ordered as expected

			const double p_intel_matrix[6] = {
				p_matrix[0], p_matrix[1], p_matrix[5],
				p_matrix[2], p_matrix[4], p_matrix[3]
			};
			/*
			 *	it should be this:
			 *	@code
			 *	|0 1 2|
			 *	|  3 4|
			 *	|    5|
			 *	@endcode
			 *
			 *	but it is like this:
			 *	@code
			 *	|0 1 5|
			 *	|  2 4|
			 *	|    3|
			 *	@endcode
			 *	(not sure about 1, 4 and 5 but they are null anyway)
			 */

			const int p_ela_pose_idx[2] = {
				p_pose_idx[1], p_pose_idx[0]
			};
			// Ela's datasets have first and second vertex swapped

			Eigen::Vector3d v_new_edge;
			C2DJacobians::Absolute_to_Relative(
				Eigen::Vector3d(p_measurement[0], p_measurement[1], p_measurement[2]),
				Eigen::Vector3d(0, 0, 0), v_new_edge); // t_odo - move this to the parser (it is a part of edge inversion)
			for(int i = 0; i < 3; ++ i)
				p_measurement[i] = v_new_edge(i);
			// inverts the edge
			// t_odo - put the edge inversion code here

			//if((*p_tok_it).second == token_Edge2D) {
				CParserBase::TEdge2D edge(p_ela_pose_idx/*p_pose_idx*/[0], p_ela_pose_idx/*p_pose_idx*/[1],
					p_measurement[0], p_measurement[1], p_measurement[2],
					(b_is_g2o_order)? p_matrix : p_intel_matrix/*p_matrix*/);
				// process the measurement

				r_parse_loop.AppendSystem(edge);
				// append the measurement to the system, or something
			/*} else {
				TOdometry2D odo(p_ela_pose_idx[0], p_ela_pose_idx[1], p_measurement[0],
					p_measurement[1], p_measurement[2], p_intel_matrix);
				// process the measurement

				r_parse_loop.AppendSystem(odo);
				// t_odo - append the measurement to the system, or something
			}*/
		}

		return true;
	}
};

/**
 *	@brief 2D landmark edge parse primitive handler
 */
class CLandmark2D_XY_ParsePrimitive {
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
		r_token_name_map["LANDMARK2:XY"] = n_assigned_id;
		r_token_name_map["EDGE_SE2_XY"] = n_assigned_id;
		r_token_name_map["EDGE_BEARING_SE2_XY"] = n_assigned_id;
		r_token_name_map["LANDMARK"] = n_assigned_id;
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
		const std::string &r_s_token, _TyParseLoop &r_parse_loop)
	{
		int p_pose_idx[2];
		double p_measurement[2];
		double p_matrix[3];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1,
		   p_matrix, p_matrix + 1, p_matrix + 2) != 2 + 2 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		static bool b_warned_landmark = false;
		if(!b_warned_landmark && r_s_token == "LANDMARK") {
			b_warned_landmark = true;
			// this has the drawback of only warning once per application execution.
			// instead it should warn once per parser instance.
			// it actually happens twice because this function template has two different
			// instances (for parse loop and for dataset peeker). but still twice per app.

			fprintf(stderr, "warning: the \'LANDMARK\' token is ambiguous: "
				"interpreted as XY landmark (rather than RB)\n");
			// yes, this is bad, we have no idea what is inside ...
		}
		// warn about the use of the "LANDMARK" token

		static bool b_warned = false;
		if(!b_warned) {
			b_warned = true;
			// this has the drawback of only warning once per application execution.
			// instead it should warn once per parser instance.
			// it actually happens twice because this function template has two different
			// instances (for parse loop and for dataset peeker). but still twice per app.

			fprintf(stderr, "warning: XY landmark edge occured: covariance "
				"conversion to RB not performed (chi2 will be incorrect)\n");

			// the covariance conversion is not easy (or at least it seems to).
			//
			// there is a naive method for conversion outlined in Covado.cpp,
			// but it is a manual process (because of missing commandline interface
			// and possibly also rounding) and involves large random sample
			// generation - not suitable for runtime execution, only for dataset
			// conversion.
		}
		// warn about covariances

		CParserBase::TLandmark2D_XY landmark(p_pose_idx[0], p_pose_idx[1],
			p_measurement[0], p_measurement[1], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 2D landmark edge parse primitive handler
 */
class CLandmark2D_RangeBearing_ParsePrimitive {
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
		r_token_name_map["LANDMARK2:RB"] = n_assigned_id;
		r_token_name_map["EDGE_SE2_RB"] = n_assigned_id;
		r_token_name_map["EDGE_BEARING_SE2_RB"] = n_assigned_id;
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
		int p_pose_idx[2];
		double p_measurement[2];
		double p_matrix[3];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1,
		   p_matrix, p_matrix + 1, p_matrix + 2) != 2 + 2 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TLandmark2D_RB landmark(p_pose_idx[0], p_pose_idx[1],
			p_measurement[0], p_measurement[1], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 2D vertex parse primitive handler
 */
class CVertex2DParsePrimitive {
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
		r_token_name_map["VERTEX2"] = n_assigned_id;
		r_token_name_map["VERTEX_SE2"] = n_assigned_id;
		r_token_name_map["VERTEX"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[3];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf",
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2) != 1 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TVertex2D vert(n_pose_id, p_vertex[0], p_vertex[1], p_vertex[2]);
		// process the measurement

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D pose edge parse primitive handler
 */
class CEdge3DParsePrimitive {
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
		r_token_name_map["EDGE3"] = n_assigned_id;
		r_token_name_map["EDGE_SE3"] = n_assigned_id;
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
		int p_pose_idx[2];
		double p_measurement[6];
		double p_matrix[21];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf " // pose indices and 6D measurement
		   "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf" // covariance follows
		   " %lf %lf %lf %lf %lf %lf %lf", p_pose_idx, p_pose_idx + 1, p_measurement,
		   p_measurement + 1, p_measurement + 2, p_measurement + 3, p_measurement + 4,
		   p_measurement + 5, p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3,
		   p_matrix + 4, p_matrix + 5, p_matrix + 6, p_matrix + 7, p_matrix + 8,
		   p_matrix + 9, p_matrix + 10, p_matrix + 11, p_matrix + 12, p_matrix + 13,
		   p_matrix + 14, p_matrix + 15, p_matrix + 16, p_matrix + 17, p_matrix + 18,
		   p_matrix + 19, p_matrix + 20) != 2 + 6 + 21) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		if(p_pose_idx[0] < p_pose_idx[1]) {
			if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[6]) < 1e-5 || fabs(p_matrix[11]) < 1e-5 ||
			   fabs(p_matrix[15]) < 1e-5 || fabs(p_matrix[18]) < 1e-5 || fabs(p_matrix[20]) < 1e-5) {
				fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
					" contact your dataset master\n");
			}
			// make sure inverse sigma is diagonal and ordered as expected

			//convert RPY to Axis-angle rep
			double cos_x = cos(p_measurement[5]);
			double sin_x = sin(p_measurement[5]);
			double cos_y = cos(p_measurement[4]);
			double sin_y = sin(p_measurement[4]); // gui
			double cos_z = cos(p_measurement[3]);
			double sin_z = sin(p_measurement[3]);
			Eigen::Matrix3d Q;
			Q << cos_y * cos_x, -cos_z * sin_x + sin_z * sin_y * cos_x, sin_z * sin_x + cos_z * sin_y * cos_x,
				 cos_y * sin_x, cos_z * cos_x + sin_z * sin_y * sin_x, -sin_z * cos_x + cos_z * sin_y * sin_x,
				 -sin_y, sin_z * cos_y, cos_z * cos_y;

			Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);

			CParserBase::TEdge3D edge(p_pose_idx[0], p_pose_idx[1],
				p_measurement[0], p_measurement[1], p_measurement[2],
				axis(0), axis(1), axis(2), p_matrix);
			// process the measurement

			r_parse_loop.AppendSystem(edge);
			// append the measurement to the system, or something
		} else {
			fprintf(stderr, "error: edges are in switched order"
				" contact your dataset master\n");
			// ignores this kind of error
		}

		return true;
	}
};

/**
 *	@brief 3D pose edge parse primitive handler
 */
class CEdge3DParsePrimitiveAxisAngle {
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
		r_token_name_map["EDGE3:AXISANGLE"] = n_assigned_id;
		r_token_name_map["EDGE_SE3:AXISANGLE"] = n_assigned_id;
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
		int p_pose_idx[2];
		double p_measurement[6];
		double p_matrix[21];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf " // pose indices and 6D measurement
		   "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf" // covariance follows
		   " %lf %lf %lf %lf %lf %lf %lf", p_pose_idx, p_pose_idx + 1, p_measurement,
		   p_measurement + 1, p_measurement + 2, p_measurement + 3, p_measurement + 4,
		   p_measurement + 5, p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3,
		   p_matrix + 4, p_matrix + 5, p_matrix + 6, p_matrix + 7, p_matrix + 8,
		   p_matrix + 9, p_matrix + 10, p_matrix + 11, p_matrix + 12, p_matrix + 13,
		   p_matrix + 14, p_matrix + 15, p_matrix + 16, p_matrix + 17, p_matrix + 18,
		   p_matrix + 19, p_matrix + 20) != 2 + 6 + 21) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		//if(p_pose_idx[0] < p_pose_idx[1]) { // this was used as a disambiguation trait for g2o / isam format datasets. since none of them use EDGE_SE3:AXISANGLE, it is not needed here.
			if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[6]) < 1e-5 || fabs(p_matrix[11]) < 1e-5 ||
			   fabs(p_matrix[15]) < 1e-5 || fabs(p_matrix[18]) < 1e-5 || fabs(p_matrix[20]) < 1e-5) {
				fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
					" contact your dataset master\n");
			}
			// make sure inverse sigma is diagonal and ordered as expected

			CParserBase::TEdge3D edge(p_pose_idx[0], p_pose_idx[1],
				p_measurement[0], p_measurement[1], p_measurement[2],
				p_measurement[3], p_measurement[4], p_measurement[5], p_matrix);
			// process the measurement

			r_parse_loop.AppendSystem(edge);
			// append the measurement to the system, or something
		/*} else {
			fprintf(stderr, "error: edges are in switched order"
				" contact your dataset master\n");
			// ignores this kind of error
		}*/

		return true;
	}
};

/**
 *	@brief 2D landmark edge parse primitive handler
 */
class CLandmark3D_XYZ_ParsePrimitive {
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
		r_token_name_map["LANDMARK3:XYZ"] = n_assigned_id;
		r_token_name_map["EDGE_SE3_XYZ"] = n_assigned_id;
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
		const std::string &r_s_token, _TyParseLoop &r_parse_loop)
	{
		int p_pose_idx[2];
		double p_measurement[3];
		double p_matrix[6];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx +1, p_measurement, p_measurement + 1, p_measurement + 2,
		   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, p_matrix + 5) != 2 + 3 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TLandmark3D_XYZ landmark(p_pose_idx[0], p_pose_idx[1],
			p_measurement[0], p_measurement[1], p_measurement[2], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 2D landmark edge parse primitive handler
 */
class CEdgeSpheron_XYZ_ParsePrimitive {
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
		r_token_name_map["EDGE_SPHERON_XYZ"] = n_assigned_id;
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
		const std::string &r_s_token, _TyParseLoop &r_parse_loop)
	{
		int p_pose_idx[2];
		double p_measurement[3];
		double p_matrix[6];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx +1, p_measurement, p_measurement + 1, p_measurement + 2,
		   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, p_matrix + 5) != 2 + 3 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TEdgeSpheronXYZ landmark(p_pose_idx[0], p_pose_idx[1],
			p_measurement[0], p_measurement[1], p_measurement[2], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D vertex parse primitive handler
 */
class CVertex3DParsePrimitive {
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
		r_token_name_map["VERTEX3"] = n_assigned_id;
		r_token_name_map["VERTEX_SE3"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[6];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf",
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2,
		   p_vertex + 3, p_vertex + 4, p_vertex + 5) != 1 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		//convert RPY to Axis-angle rep
		double cos_x = cos(p_vertex[5]);
		double sin_x = sin(p_vertex[5]);
		double cos_y = cos(p_vertex[4]);
		double sin_y = sin(p_vertex[4]);
		double cos_z = cos(p_vertex[3]);
		double sin_z = sin(p_vertex[3]);
		Eigen::Matrix3d Q;
		Q << cos_y*cos_x, -cos_z*sin_x + sin_z*sin_y*cos_x, sin_z*sin_x + cos_z*sin_y*cos_x,
		     cos_y*sin_x, cos_z*cos_x + sin_z*sin_y*sin_x, -sin_z*cos_x + cos_z*sin_y*sin_x,
		     -sin_y, sin_z*cos_y, cos_z*cos_y;

		Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);

		CParserBase::TVertex3D vert(n_pose_id, p_vertex[0],
			p_vertex[1], p_vertex[2], axis(0), axis(1), axis(2));
		// process the measurement

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D vertex parse primitive handler
 */
class CVertexXYZParsePrimitive {
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
		r_token_name_map["VERTEX_XYZ"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[3];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf",
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2) != 1 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TVertexXYZ vert(n_pose_id, p_vertex[0], p_vertex[1], p_vertex[2]);
		// process the measurement

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D vertex parse primitive handler
 */
class CVertexCam3DParsePrimitive {
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
		r_token_name_map["VERTEX_CAM"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[12];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"/*" %lf %lf"*/,
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2, p_vertex + 3, p_vertex + 4,
		   p_vertex + 5, p_vertex + 6, p_vertex + 7, p_vertex + 8, p_vertex + 9, p_vertex + 10,
		   p_vertex + 11) != 1 + 7 + 5) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		Eigen::Quaternion<double> quat(p_vertex[6], p_vertex[3], p_vertex[4], p_vertex[5]);
		quat.normalize();
		quat = quat.inverse();
		//Eigen::Matrix3d Q = quat.toRotationMatrix();
		//Q = Q.inverse().eval();
		//Q = Q.householderQr().householderQ();

		Eigen::Vector3d t_vec(p_vertex[0], p_vertex[1], p_vertex[2]);
		//rotate
		Eigen::Vector3d c = quat * (-t_vec);
		//Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
		Eigen::Vector3d axis;
		C3DJacobians::Quat_to_AxisAngle(quat, axis);

		CParserBase::TVertexCam3D vert(n_pose_id, c[0],
			c[1], c[2], axis(0), axis(1), axis(2), p_vertex[7], p_vertex[8], p_vertex[9], p_vertex[10], p_vertex[11]);
		// process the measurement

		//camera intrinsic parameters

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D vertex parse primitive handler
 */
class CVertexIntrinsicsParsePrimitive {
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
		r_token_name_map["VERTEX_INTRINSICS"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[12];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf",
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2, p_vertex + 3, p_vertex + 4) != 1 + 5) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}

		// read the individual numbers
		CParserBase::TVertexIntrinsics vert(n_pose_id, p_vertex[0], p_vertex[1], p_vertex[2],
			p_vertex[3], p_vertex[4]);

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D vertex parse primitive handler
 */
class CVertexSCam3DParsePrimitive {
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
		r_token_name_map["VERTEX_SCAM"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[13];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"/*" %lf %lf"*/,
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2, p_vertex + 3, p_vertex + 4,
		   p_vertex + 5, p_vertex + 6, p_vertex + 7, p_vertex + 8, p_vertex + 9, p_vertex + 10,
		   p_vertex + 11, p_vertex + 12) != 1 + 7 + 5 + 1) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		Eigen::Quaternion<double> quat(p_vertex[6], p_vertex[3], p_vertex[4], p_vertex[5]);
		quat = quat.inverse();
		//Eigen::Matrix3d Q = quat.toRotationMatrix();
		//Q = Q.inverse().eval();
		//Q = Q.householderQr().householderQ();

		Eigen::Vector3d t_vec(p_vertex[0], p_vertex[1], p_vertex[2]);
		//rotate
		Eigen::Vector3d c = quat * (-t_vec);
		//Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
		Eigen::Vector3d axis;
		C3DJacobians::Quat_to_AxisAngle(quat, axis);

		CParserBase::TVertexSCam3D vert(n_pose_id, c[0],
			c[1], c[2], axis(0), axis(1), axis(2), p_vertex[7], p_vertex[8], p_vertex[9], p_vertex[10], p_vertex[11], p_vertex[12]);
		// process the measurement

		//camera intrinsic parameters

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D vertex parse primitive handler
 */
class CVertexSpheronParsePrimitive {
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
		r_token_name_map["VERTEX_SPHERON:QUAT"] = n_assigned_id;
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
		int n_pose_id;
		double p_vertex[7];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf"/*" %lf %lf"*/,
		   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2, p_vertex + 3, p_vertex + 4,
		   p_vertex + 5, p_vertex + 6) != 1 + 7) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		Eigen::Quaternion<double> quat(p_vertex[6], p_vertex[3], p_vertex[4], p_vertex[5]);
		//quat = quat.inverse();
		//Eigen::Matrix3d Q = quat.toRotationMatrix();
		//Q = Q.inverse().eval();
		//Q = Q.householderQr().householderQ();

		Eigen::Vector3d t_vec(p_vertex[0], p_vertex[1], p_vertex[2]);
		//rotate
		Eigen::Vector3d c = t_vec;/*quat * (-t_vec);*/
		//Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
		Eigen::Vector3d axis;
		C3DJacobians::Quat_to_AxisAngle(quat, axis);

		CParserBase::TVertexSpheron vert(n_pose_id, c[0],
			c[1], c[2], axis(0), axis(1), axis(2));
		// process the measurement

		//camera intrinsic parameters

		r_parse_loop.InitializeVertex(vert);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief point - camera projection
 */
class CEdgeP2C3DParsePrimitive {
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
		r_token_name_map["EDGE_PROJECT_P2MC"] = n_assigned_id;
		r_token_name_map["EDGE_P2MC"] = n_assigned_id;
		r_token_name_map["EDGE_P2C"] = n_assigned_id;
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
		int p_pose_idx[2];
		double p_measurement[2];
		double p_matrix[3];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1,
		   p_matrix, p_matrix + 1, p_matrix + 2) != 2 + 2 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		//if(p_pose_idx[0] > p_pose_idx[1])
		//	fprintf(stderr, "warning: don't know how to do landmark inversion\n");
		// rubbish! can't detect this, landmarks and poses must be ordered up front.
		// both orders are correct in a single datafile.

		CParserBase::TEdgeP2C3D landmark(p_pose_idx[0], p_pose_idx[1],
			p_measurement[0], p_measurement[1], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

///**
// *	@brief point - camera epipolar edge
// */
//class CEdgeC2CEParsePrimitive {
//public:
//	/**
//	 *	@brief enumerates all tokens that identify this parsed primitive
//	 *
//	 *	@param[in,out] r_token_name_map is map of token names
//	 *	@param[in] n_assigned_id is id assigned by the parser to this primitive
//	 */
//	static void EnumerateTokens(std::map<std::string, int> &r_token_name_map,
//		int n_assigned_id) // throws(std::bad_alloc)
//	{
//		r_token_name_map["EDGE_PROJECT_C2CE"] = n_assigned_id;
//		r_token_name_map["EDGE_C2CE"] = n_assigned_id;
//		// add as uppercase!
//	}
//
//	/**
//	 *	@brief parses this primitive and dispatches it to the parse loop
//	 *
//	 *	@param[in] n_line_no is zero-based line number (for error reporting)
//	 *	@param[in] r_s_line is string, containing the current line (without the token)
//	 *	@param[in] r_s_token is string, containing the token name (in uppercase)
//	 *	@param[in,out] r_parse_loop is target for passing the parsed primitives to
//	 *
//	 *	@return Returns true on success, false on failure.
//	 */
//	template <class _TyParseLoop>
//	static bool Parse_and_Dispatch(size_t n_line_no, const std::string &r_s_line,
//		const std::string &UNUSED(r_s_token), _TyParseLoop &r_parse_loop)
//	{
//		int p_pose_idx[2];
//		double p_measurement[4];
//		double p_matrix[10];
//		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
//		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1, p_measurement + 2, p_measurement + 3,
//		   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, p_matrix + 5, p_matrix + 6, p_matrix + 7,
//		   p_matrix + 8, p_matrix + 9) != 2 + 4 + 10) {
//		   	_ASSERTE(n_line_no < SIZE_MAX);
//			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
//			return false;
//		}
//		// read the individual numbers
//		CParserBase::TEdgeC2CE landmark(p_pose_idx[0], p_pose_idx[1],
//			p_measurement[0], p_measurement[1], p_measurement[2], p_measurement[3], p_matrix);
//		// process the measurement
//
//		r_parse_loop.AppendSystem(landmark);
//		// t_odo - append the measurement to the system, or something
//
//		return true;
//	}
//};

/**
 *	@brief point - camera projection
 */
class CEdgeP2CI3DParsePrimitive {
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
		r_token_name_map["EDGE_PROJECT_P2MCI"] = n_assigned_id;
		r_token_name_map["EDGE_P2MCI"] = n_assigned_id;
		r_token_name_map["EDGE_P2CI"] = n_assigned_id;
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
		int p_pose_idx[3];
		double p_measurement[2];
		double p_matrix[3];
		if(sscanf(r_s_line.c_str(), "%d %d %d %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_pose_idx + 2, p_measurement, p_measurement + 1,
		   p_matrix, p_matrix + 1, p_matrix + 2) != 3 + 2 + 3) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		//if(p_pose_idx[0] > p_pose_idx[1])
		//	fprintf(stderr, "warning: don't know how to do landmark inversion\n");
		// rubbish! can't detect this, landmarks and poses must be ordered up front.
		// both orders are correct in a single datafile.

		CParserBase::TEdgeP2CI3D landmark(p_pose_idx[0], p_pose_idx[1], p_pose_idx[2],
			p_measurement[0], p_measurement[1], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief point - camera projection
 */
class CEdgeP2SC3DParsePrimitive {
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
		r_token_name_map["EDGE_PROJECT_P2SC"] = n_assigned_id;
		r_token_name_map["EDGE_P2SC"] = n_assigned_id;
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
		int p_pose_idx[2];
		double p_measurement[3];
		double p_matrix[6];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1, p_measurement + 2,
		   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, p_matrix + 5) != 2 + 3 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		//if(p_pose_idx[0] > p_pose_idx[1])
		//	fprintf(stderr, "warning: don't know how to do landmark inversion\n");
		// rubbish! can't detect this, landmarks and poses must be ordered up front.
		// both orders are correct in a single datafile.

		CParserBase::TEdgeP2SC3D landmark(p_pose_idx[0], p_pose_idx[1],
			p_measurement[0], p_measurement[1], p_measurement[2], p_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(landmark);
		// t_odo - append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 2D pose edge parse primitive handler
 */
class CGPSPhaseParsePrimitive {
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
		r_token_name_map["PHASE"] = n_assigned_id;
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
		// #PHASE PoseNum AmbNum PhaseMeas East North Up
		int p_pose_idx[2];
		double p_measurement[4];
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf %lf %lf",
		   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1,
		   p_measurement + 2, p_measurement + 3) != 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		// todo - add the GPS edge to the system

		return true;
	}
};

/**
 *	@brief 3D landmark vertex parse primitive handler
 */
class CROCV_Landmark_ParsePrimitive : public CVertexXYZParsePrimitive {
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
		r_token_name_map["ROCV:TRANSMITTER"] = n_assigned_id; // add as uppercase!
	}
};

/**
 *	@brief 3D position unary factor parse primitive handler
 */
class CROCV_Landmark_UF_ParsePrimitive {
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
		r_token_name_map["ROCV:TRANSMITTER_UF"] = n_assigned_id; // add as uppercase!
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
		const std::string &r_s_token, _TyParseLoop &r_parse_loop)
	{
		int n_landmark_idx;
		double p_covariance[6];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf",
		   &n_landmark_idx, p_covariance, p_covariance + 1, p_covariance + 2,
		   p_covariance + 3, p_covariance + 4, p_covariance + 5) != 1 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TUnaryFactor3D factor(n_landmark_idx, p_covariance);
		// process the factor

		r_parse_loop.AppendSystem(factor);
		// append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D position-velocity pose vertex parse primitive handler
 */
class CROCV_Pose_ParsePrimitive {
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
		r_token_name_map["ROCV:RECEIVER"] = n_assigned_id; // add as uppercase!
		//r_token_name_map["ROCV:RECEIVER_GT"] = n_assigned_id; // add as uppercase! // GT is cheating
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
		const std::string &r_s_token, _TyParseLoop &r_parse_loop)
	{
		int n_pose_idx;
		double p_measurement[6];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf",
		   &n_pose_idx, p_measurement, p_measurement + 1, p_measurement + 2,
		   p_measurement + 3, p_measurement + 4, p_measurement + 5) != 1 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TVertex3D vertex(n_pose_idx,
			p_measurement[0], p_measurement[1], p_measurement[2], // pos
			p_measurement[3], p_measurement[4], p_measurement[5]); // vel
		// process the measurement

		r_parse_loop.InitializeVertex(vertex);
		// append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief 3D position-velocity pose ground truth vertex parse primitive handler
 */
class CROCV_PoseGroundTruth_ParsePrimitive : public CROCV_Pose_ParsePrimitive {
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
		r_token_name_map["ROCV:RECEIVER_GTfake"] = n_assigned_id; // add as uppercase!
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
		const std::string &r_s_token, _TyParseLoop &r_parse_loop)
	{
		int n_pose_idx;
		double p_measurement[6];
		if(sscanf(r_s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf",
		   &n_pose_idx, p_measurement, p_measurement + 1, p_measurement + 2,
		   p_measurement + 3, p_measurement + 4, p_measurement + 5) != 1 + 6) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TVertex3D_Reference vertex(n_pose_idx,
			p_measurement[0], p_measurement[1], p_measurement[2], // pos
			p_measurement[3], p_measurement[4], p_measurement[5]); // vel
		// process the measurement

		r_parse_loop.InitializeVertex(vertex);
		// append the measurement to the system, or something

		return true;
	}
};

/**
 *	@brief delta-time edge parse primitive handler
 */
class CROCV_DeltaTimeEdge_ParsePrimitive {
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
		r_token_name_map["ROCV:DELTA_TIME"] = n_assigned_id;
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
		int p_pose_idx[2];
		double f_measurement;
		double p_matrix[21];
		if(sscanf(r_s_line.c_str(), "%d %d %lf " // pose indices and 1D measurement
		   "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf" // covariance follows
		   " %lf %lf %lf %lf %lf %lf %lf", p_pose_idx, p_pose_idx + 1, &f_measurement,
		   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, p_matrix + 5,
		   p_matrix + 6, p_matrix + 7, p_matrix + 8, p_matrix + 9, p_matrix + 10, p_matrix + 11,
		   p_matrix + 12, p_matrix + 13, p_matrix + 14, p_matrix + 15, p_matrix + 16,
		   p_matrix + 17, p_matrix + 18, p_matrix + 19, p_matrix + 20) != 2 + 1 + 21) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		if(p_pose_idx[0] < p_pose_idx[1]) { // expect (previous, current) order on the vertices
			if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[6]) < 1e-5 || fabs(p_matrix[11]) < 1e-5 ||
			   fabs(p_matrix[15]) < 1e-5 || fabs(p_matrix[18]) < 1e-5 || fabs(p_matrix[20]) < 1e-5) {
				fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
					" contact your dataset master\n");
			}
			// make sure inverse sigma is diagonal and ordered as expected

			CParserBase::TROCV_DeltaTimeEdge edge(p_pose_idx[0], p_pose_idx[1], f_measurement, p_matrix);
			// process the measurement

			r_parse_loop.AppendSystem(edge);
			// append the measurement to the system, or something
		} else {
			fprintf(stderr, "error: edges are in switched order (Soso was too lazy "
				".. nooo.. hmm... was in hurry so he didnt implement switching)."
				" contact your dataset master\n");
			// ignores this kind of error
		}

		return true;
	}
};

/**
 *	@brief delta-time edge parse primitive handler
 */
class CROCV_RangeEdge_ParsePrimitive {
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
		r_token_name_map["ROCV:RANGE"] = n_assigned_id;
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
		int p_pose_idx[2];
		double f_measurement;
		double f_matrix;
		if(sscanf(r_s_line.c_str(), "%d %d %lf %lf", // pose indices and 1D measurement, 1D covariance follows
		   p_pose_idx, p_pose_idx + 1, &f_measurement, &f_matrix) != 2 + 1 + 1) {
		   	_ASSERTE(n_line_no < SIZE_MAX);
			fprintf(stderr, "error: line " PRIsize ": line is truncated\n", n_line_no + 1);
			return false;
		}
		// read the individual numbers

		CParserBase::TROCV_RangeEdge edge(p_pose_idx[0], p_pose_idx[1], f_measurement, f_matrix);
		// process the measurement

		r_parse_loop.AppendSystem(edge);
		// append the measurement to the system, or something

		return true;
	}
};

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma warning(disable: 4503)
#endif // _MSC_VER && !__MWERKS__
// avoid msvc warning "decorated name length exceeded, name was truncated"

typedef CConcatTypelist<MakeTypelist_Safe((CEdge2DParsePrimitive,
	CLandmark2D_RangeBearing_ParsePrimitive, CLandmark2D_XY_ParsePrimitive,
	CEdge3DParsePrimitive, CEdge3DParsePrimitiveAxisAngle, CVertex2DParsePrimitive,
	CVertex3DParsePrimitive, CLandmark3D_XYZ_ParsePrimitive,
	CIgnoreParsePrimitive, CVertexXYZParsePrimitive, CVertexCam3DParsePrimitive,
	CEdgeP2C3DParsePrimitive, CVertexSCam3DParsePrimitive, CVertexSpheronParsePrimitive, CEdgeSpheron_XYZ_ParsePrimitive,
	CEdgeP2SC3DParsePrimitive)), MakeTypelist_Safe((CROCV_Landmark_UF_ParsePrimitive,
	CROCV_Landmark_ParsePrimitive, CROCV_Pose_ParsePrimitive, CROCV_PoseGroundTruth_ParsePrimitive,
	CROCV_DeltaTimeEdge_ParsePrimitive, CROCV_RangeEdge_ParsePrimitive,
	CVertexIntrinsicsParsePrimitive, CEdgeP2CI3DParsePrimitive))>::_TyResult CStandardParsedPrimitives; /**< @brief a list of standard parsed primitives @note If you are going to modify this, you will have to modify CParserBase::CParserAdaptor and CDatasetPeeker which implements it. */

typedef CParserTemplate<CParserBase::CParserAdaptor, CStandardParsedPrimitives> CStandardParser; /**< @brief standard parser */

/** @} */ // end of group

#endif // !__GRAPH_PARSER_PRIMITIVES_INCLUDED
