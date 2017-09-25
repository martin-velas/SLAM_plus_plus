/*
								+-----------------------------------+
								|                                   |
								|      ***  .graph Parser  ***      |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|            Parser.cpp             |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam/Parser.cpp
 *	@brief a simple .graph file parser
 *	@author -tHE SWINe-
 *	@date 2012-03-30
 */

//#include "slam/Debug.h"
#include "slam/Parser.h"
/*#include <map>
#include <set>
#include <algorithm>
#include "slam/2DSolverBase.h"
#include "slam/3DSolverBase.h"*/

// the following constructors are declared as "inline"
#if 0

/*
 *								=== CParser::COdometry ===
 */

CParserBase::COdometry::COdometry(int n_pose_0, int n_pose_1, double f_x, double f_y,
	double f_theta, const double *p_lower_matrix_3x3)
{
	// t_odo - do something about the measurement :)
}

/*
 *								=== ~CParser::COdometry ===
 */

/*
 *								=== CParser::CLandmark ===
 */

CParserBase::CLandmark::CLandmark(int n_pose_0, int n_pose_1, double f_x, double f_y,
	const double *p_lower_matrix_2x2)
{
	// t_odo - do something about the measurement :)
}

/*
 *								=== ~CParser::CLandmark ===
 */

/*
 *								=== CParser::CEdge ===
 */

CParserBase::CEdge::CEdge(int n_pose_0, int n_pose_1, double f_x, double f_y,
	double f_theta, const double *p_lower_matrix_3x3)
	:COdometry(n_pose_0, n_pose_1, f_x, f_y, f_theta, p_lower_matrix_3x3)
{
	// t_odo - do something about the measurement :)
}

/*
 *								=== ~CParser::CEdge ===
 */

/*
 *								=== CParser::CVertex ===
 */

CParserBase::CVertex::CVertex(int n_pose_id, double f_x, double f_y, double f_theta)
{
	// t_odo - do something about the measurement :)
}

/*
 *								=== ~CParser::CVertex ===
 */

#endif // 0

/*
 *								=== CParser ===
 */

std::map<std::string, std::pair<std::set<std::string>,
	std::pair<bool, bool> > > CParserBase::m_per_file_warned_token_set;

#if 0
bool CParser::Parse(const char *p_s_filename, CParserAdaptor *p_callback,
	size_t n_line_limit /*= 0*/, bool b_ignore_case /*= false*/)
{
	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open the input file

	//size_t n_rd = 0; // debug
	try {
		std::set<std::pair<int, int> > edge_set;
		std::map<std::string, int> token_name_map;
		{
			token_name_map["EDGE2"] = token_Edge2D;
			token_name_map["EDGE_SE2"] = token_Edge2D; // fixme - is this the same? (it apparently is, but ...)
			token_name_map["EDGE"] = token_Edge2D; // f_ixme - is this the same? (it apparently is, but ...)
			token_name_map["ODOMETRY"] = token_Odometry2D; // note this is the same as the edge

			token_name_map["VERTEX2"] = token_Vertex2D;
			token_name_map["VERTEX_SE2"] = token_Vertex2D;
			token_name_map["VERTEX"] = token_Vertex2D; // f_ixme - is this the same? (it apparently is, but ...)

			token_name_map["LANDMARK2:XY"] = token_Landmark2D;
			token_name_map["LANDMARK2:RB"] = token_Landmark2D_RangeBearing; // this is a different kind of landmark, needs to be processed separately
			// the preferred names of 2D primitives

			token_name_map["EDGE_SE2_XY"] = token_Landmark2D;
			token_name_map["EDGE_BEARING_SE2_XY"] = token_Landmark2D;
			token_name_map["LANDMARK"] = token_Landmark2D;
			// other names of 2D primitives

			token_name_map["EQUIV"] = token_NoParse; // this is found in 10k.graph (don't know what that means)
			// ignore names

			// t_odo - names of 3D primitives
			token_name_map["VERTEX3"] = token_Vertex3D;
			token_name_map["VERTEX_SE3"] = token_Vertex3D;
			token_name_map["EDGE3"] = token_Edge3D;
			token_name_map["EDGE_SE3"] = token_Edge3D;

			// names for BA
			token_name_map["EDGE_PROJECT_P2MC"] = token_Projection_Point3D2Camera;
		}
		// build a map to speed up looking for token types

		bool b_new_file = m_per_file_warned_token_set.find(std::string(p_s_filename)) ==
			m_per_file_warned_token_set.end();
		bool &r_b_warned_about_duplicates =
			m_per_file_warned_token_set[std::string(p_s_filename)].second.first;
		bool &r_b_warned_about_reverse_order =
			m_per_file_warned_token_set[std::string(p_s_filename)].second.second;
		if(b_new_file) {
			r_b_warned_about_duplicates = false;
			r_b_warned_about_reverse_order = false;
		}
		std::set<std::string> &r_bad_token_name_set =
			m_per_file_warned_token_set[std::string(p_s_filename)].first;
		// set of unknown token names (remember them for application runtime)

		std::string s_line, s_token;
		for(size_t n_line_no = 0; !feof(p_fr) && (!n_line_limit || n_line_no < n_line_limit); ++ n_line_no) {
			//++ n_rd; // debug
			if(!ReadLine(s_line, p_fr)) {
				fclose(p_fr);
				return false;
			}
			TrimSpace(s_line);
			if(s_line.empty())
				continue;
			// read the file line by line, skip the empty lines

			s_token.erase();
			size_t n_pos;
			if((n_pos = s_line.find_first_of(" \t")) != std::string::npos) {
				s_token.insert(s_token.begin(), s_line.begin(), s_line.begin() + n_pos);
				s_line.erase(0, n_pos + 1);
				TrimSpace(s_line); // ...
			} else {
				s_token.insert(s_token.begin(), s_line.begin(), s_line.end());
				s_line.erase();
			}
			// read the token (separated by space; otherwise the entire line is a token)
			// and remove the token from the line

			if(b_ignore_case)
				std::for_each(s_token.begin(), s_token.end(), toupper);
			// map is case sensitive, if there is dataset with lowercase tokens, it needs to be fixed

			std::map<std::string, int>::const_iterator p_tok_it;
			if((p_tok_it = token_name_map.find(s_token)) == token_name_map.end()) {
				if(r_bad_token_name_set.find(s_token) == r_bad_token_name_set.end()) {
					fprintf(stderr, "warning: unknown token: \'%s\' (ignored)\n", s_token.c_str());
					r_bad_token_name_set.insert(s_token);
				}
				// t_odo - add set of unknown token names so each warning is only printed once

				continue;
			}
			// identify the token, skip unknown tokens

			switch((*p_tok_it).second) {
			case token_Odometry2D: // note i joined the code for odo and edge at 2012-06-04. if any bugs arise, it might be this.
			case token_Edge2D:
				{
					int p_pose_idx[2];
					double p_measurement[3];
					double p_matrix[6];
					if(sscanf(s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
					   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1, p_measurement + 2,
					   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4,
					   p_matrix + 5) != 2 + 3 + 6) {
					   	_ASSERTE(n_line_no < INT_MAX);
						fprintf(stderr, "error: line %d: line is truncated\n", int(n_line_no) + 1);
						fclose(p_fr);
						return false;
					}
					// read the individual numbers

					std::pair<int, int> t_edge_id(std::min(p_pose_idx[0], p_pose_idx[1]),
						std::max(p_pose_idx[0], p_pose_idx[1]));
					if(edge_set.find(t_edge_id) != edge_set.end()) {
						if(!r_b_warned_about_duplicates) {
							fprintf(stderr, "warning: duplicate edges detected\n");
							r_b_warned_about_duplicates = true;
						}
						break;
					}
					edge_set.insert(t_edge_id);
					// ignore duplicate edges

					if(p_pose_idx[0] < p_pose_idx[1]) {
						//fprintf(stderr, "warning: doing inversion on possibly straight edge. contact your dataset master\n");
						// not any more, you don't

						if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[3]) < 1e-5 || fabs(p_matrix[5]) < 1e-5) {
							if(fabs(p_matrix[0]) > 1e-5 && fabs(p_matrix[2]) > 1e-5 && fabs(p_matrix[3]) > 1e-5) {
								if(!r_b_warned_about_reverse_order) {
									fprintf(stderr, "warning: the inverse sigma matrix is in the french order."
										" contact your dataset master\n");
									r_b_warned_about_reverse_order = true;
								}
							} else {
								fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
									" contact your dataset master\n");
							}
						}
						// make sure inverse sigma is diagonal and ordered as expected

						if((*p_tok_it).second == token_Edge2D) {
							TEdge2D edge(p_pose_idx[0], p_pose_idx[1],
								p_measurement[0], p_measurement[1], p_measurement[2], p_matrix);
							// process the measurement

							p_callback->AppendSystem(edge);
							// append the measurement to the system, or something
						} else {
							TOdometry2D odo(p_pose_idx[0], p_pose_idx[1], p_measurement[0],
								p_measurement[1], p_measurement[2], p_matrix);
							// process the measurement

							p_callback->AppendSystem(odo);
							// t_odo - append the measurement to the system, or something
						}
					} else {
						// the manhattan datasets have poses in descending order (e.g. EDGE 1 0), the edges need to be inverted here
						// in case the order is ascending, it is possible that the inversion is inadvertent

						if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[2]) < 1e-5 || fabs(p_matrix[3]) < 1e-5) {
							if(fabs(p_matrix[0]) > 1e-5 && fabs(p_matrix[3]) > 1e-5 && fabs(p_matrix[5]) > 1e-5) {
								if(!r_b_warned_about_reverse_order) {
									fprintf(stderr, "warning: the inverse sigma matrix is in the g2o order."
										" however, this dataset requires edge inversion and french order was expected."
										" contact your dataset master\n");
									r_b_warned_about_reverse_order = true;
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

						if((*p_tok_it).second == token_Edge2D) {
							TEdge2D edge(p_ela_pose_idx/*p_pose_idx*/[0], p_ela_pose_idx/*p_pose_idx*/[1],
								p_measurement[0], p_measurement[1], p_measurement[2], p_intel_matrix/*p_matrix*/);
							// process the measurement

							p_callback->AppendSystem(edge);
							// append the measurement to the system, or something
						} else {
							TOdometry2D odo(p_ela_pose_idx[0], p_ela_pose_idx[1], p_measurement[0],
								p_measurement[1], p_measurement[2], p_intel_matrix);
							// process the measurement

							p_callback->AppendSystem(odo);
							// t_odo - append the measurement to the system, or something
						}
					}
				}
				break;
			case token_Landmark2D:
				{
					int p_pose_idx[2];
					double p_measurement[2];
					double p_matrix[3];
					if(sscanf(s_line.c_str(), "%d %d %lf %lf %lf %lf %lf",
					   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1,
					   p_matrix, p_matrix + 1, p_matrix + 2) != 2 + 2 + 3) {
					   	_ASSERTE(n_line_no < INT_MAX);
						fprintf(stderr, "error: line %d: line is truncated\n", int(n_line_no) + 1);
						fclose(p_fr);
						return false;
					}
					// read the individual numbers

					std::pair<int, int> t_edge_id(std::min(p_pose_idx[0], p_pose_idx[1]),
						std::max(p_pose_idx[0], p_pose_idx[1]));
					if(edge_set.find(t_edge_id) != edge_set.end()) {
						if(!r_b_warned_about_duplicates) {
							fprintf(stderr, "warning: duplicate edges detected\n");
							r_b_warned_about_duplicates = true;
						}
						break;
					}
					edge_set.insert(t_edge_id);
					// ignore duplicate edges

					//if(p_pose_idx[0] > p_pose_idx[1])
					//	fprintf(stderr, "warning: don't know how to do landmark inversion\n");
					// rubbish! can't detect this, landmarks and poses must be ordered up front.
					// both orders are correct in a single datafile.

					TLandmark2D landmark(p_pose_idx[0], p_pose_idx[1],
						p_measurement[0], p_measurement[1], p_matrix);
					// process the measurement

					p_callback->AppendSystem(landmark);
					// t_odo - append the measurement to the system, or something
				}
				break;
			case token_Vertex2D:
				{
					int n_pose_id;
					double p_vertex[3];
					if(sscanf(s_line.c_str(), "%d %lf %lf %lf",
					   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2) != 1 + 3) {
					   	_ASSERTE(n_line_no < INT_MAX);
						fprintf(stderr, "error: line %d: line is truncated\n", int(n_line_no) + 1);
						fclose(p_fr);
						return false;
					}
					// read the individual numbers

					TVertex2D vert(n_pose_id, p_vertex[0], p_vertex[1], p_vertex[2]);
					// process the measurement

					p_callback->AppendSystem(vert);
					// t_odo - append the measurement to the system, or something
				}
				break;
			case token_Vertex3D:
				{
					int n_pose_id;
					double p_vertex[6];
					if(sscanf(s_line.c_str(), "%d %lf %lf %lf %lf %lf %lf",
					   &n_pose_id, p_vertex, p_vertex + 1, p_vertex + 2, p_vertex + 3, p_vertex + 4, p_vertex + 5) != 1 + 6) {
					   	_ASSERTE(n_line_no < INT_MAX);
						fprintf(stderr, "error: line %d: line is truncated\n", int(n_line_no) + 1);
						fclose(p_fr);
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

					TVertex3D vert(n_pose_id, p_vertex[0], p_vertex[1], p_vertex[2], axis(0), axis(1), axis(2));
					// process the measurement

					p_callback->AppendSystem(vert);
					// t_odo - append the measurement to the system, or something
				}
				break;
			case token_Edge3D:
				{
					int p_pose_idx[2];
					double p_measurement[6];
					double p_matrix[21];
					if(sscanf(s_line.c_str(), "%d %d    %lf %lf %lf %lf %lf %lf    %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
					   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1, p_measurement + 2, p_measurement + 3, p_measurement + 4, p_measurement + 5,
					   p_matrix, p_matrix + 1, p_matrix + 2, p_matrix + 3, p_matrix + 4, p_matrix + 5, p_matrix + 6, p_matrix + 7, p_matrix + 8, p_matrix + 9,
					   p_matrix + 10, p_matrix + 11, p_matrix + 12, p_matrix + 13, p_matrix + 14, p_matrix + 15, p_matrix + 16, p_matrix + 17, p_matrix + 18, p_matrix + 19,
					   p_matrix + 20) != 2 + 6 + 21) {
					   	_ASSERTE(n_line_no < INT_MAX);
						fprintf(stderr, "error: line %d: line is truncated\n", int(n_line_no) + 1);
						fclose(p_fr);
						return false;
					}
					// read the individual numbers

					std::pair<int, int> t_edge_id(std::min(p_pose_idx[0], p_pose_idx[1]),
						std::max(p_pose_idx[0], p_pose_idx[1]));
					if(edge_set.find(t_edge_id) != edge_set.end()) {
						if(!r_b_warned_about_duplicates) {
							fprintf(stderr, "warning: duplicate edges detected\n");
							r_b_warned_about_duplicates = true;
						}
						break;
					}
					edge_set.insert(t_edge_id);
					// ignore duplicate edges

					if(p_pose_idx[0] < p_pose_idx[1]) {
						if(fabs(p_matrix[0]) < 1e-5 || fabs(p_matrix[6]) < 1e-5 || fabs(p_matrix[11]) < 1e-5 ||
								fabs(p_matrix[15]) < 1e-5  || fabs(p_matrix[18]) < 1e-5 || fabs(p_matrix[20]) < 1e-5) {
							fprintf(stderr, "error: the inverse sigma matrix is in unknown order."
									" contact your dataset master\n");
						}
						// make sure inverse sigma is diagonal and ordered as expected

						//convert RPY to Axis-angle rep
						double cos_x = cos(p_measurement[5]);
						double sin_x = sin(p_measurement[5]);
						double cos_y = cos(p_measurement[4]);
						double sin_y = sin(p_measurement[4]);//gui
						double cos_z = cos(p_measurement[3]);
						double sin_z = sin(p_measurement[3]);
						Eigen::Matrix3d Q;
						Q << cos_y*cos_x, -cos_z*sin_x + sin_z*sin_y*cos_x, sin_z*sin_x + cos_z*sin_y*cos_x,
							 cos_y*sin_x, cos_z*cos_x + sin_z*sin_y*sin_x, -sin_z*cos_x + cos_z*sin_y*sin_x,
							 -sin_y, sin_z*cos_y, cos_z*cos_y;

						Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);

						TEdge3D edge(p_pose_idx[0], p_pose_idx[1],
							p_measurement[0], p_measurement[1], p_measurement[2], axis(0), axis(1), axis(2), p_matrix);
						// process the measurement

						p_callback->AppendSystem(edge);
						// append the measurement to the system, or something
					} else {
						fprintf(stderr, "error: edges are in switched order (Soso was too lazy .. nooo.. hmm... was in hurry so he didnt implement switching)."
						" contact your dataset master\n");
					}
				}
				break;
			/*case token_Projection_Point3D2Camera: // this is handling your new edge type, after the parser encountered any of the tokens you put in the map
				int p_pose_idx[2];
				double p_measurement[2];
				double p_matrix[3];
				// data that are stored on the line (change the measurement and matrix dimensions as required)

				if(sscanf(s_line.c_str(), "%d %d %lf %lf %lf %lf %lf %lf", // modify the numbers of %lf to reflect number of numbers you need to read from the line
				   p_pose_idx, p_pose_idx + 1, p_measurement, p_measurement + 1,
				   p_matrix, p_matrix + 1, p_matrix + 2) != 2 + 2 + 3) { // modify the condition
					_ASSERTE(n_line_no < INT_MAX);
					fprintf(stderr, "error: line %d: line is truncated\n", int(n_line_no) + 1);
					fclose(p_fr);
					return false;
				}
				// read the individual numbers

				CParseEntity_Projection_Point3D2Camera edge(p_pose_idx[0], p_pose_idx[1],
					p_measurement[0], p_measurement[1], p_matrix); // this creates an object of type that you introduced to Parser.h (rename it)
				// process the measurement

				p_callback->AppendSystem(edge);
				// append the measurement to the system, or something
				break;*/

			case token_NoParse:
				// this kind of token is not parsed, just skip to the next line
				break;

			case token_Landmark2D_RangeBearing:
				// this case intentionally falls through (not supported)
			default:
				fprintf(stderr, "error: internal program error: unknown token occurred\n");
				fclose(p_fr);
				return false;
			}
			// parse the given token
		}
	} catch(std::bad_alloc&) {
		fclose(p_fr);
		return false;
	}
	// loop thorough the file

	//printf("read %d lines\n", n_rd); // debug

	if(ferror(p_fr)) {
		fclose(p_fr);
		return false;
	}
	fclose(p_fr);
	// check for errors, close the file

	return true;
}

#endif // 0

void CParserBase::TrimSpace(std::string &r_s_string)
{
	size_t b = 0, e = r_s_string.length();
	while(e > 0 && isspace((unsigned char)(r_s_string[e - 1])))
		-- e;
	while(b < e && isspace((unsigned char)(r_s_string[b])))
		++ b;
	r_s_string.erase(e);
	r_s_string.erase(0, b);
}

bool CParserBase::ReadLine(std::string &r_s_line, FILE *p_fr)
{
	r_s_line.erase();
	try {
		for(int c = fgetc(p_fr); c != '\n' && c != EOF; c = fgetc(p_fr)) {
			/*if(ferror(p_fr)) // if some other reading error happens, fgetc() also returns EOF
				return false;*/
			r_s_line += c; // some string implementations don't have push_back()
		}
	} catch(std::bad_alloc&) {
		return false;
	}
	// read line

	return !ferror(p_fr);
}

bool CParserBase::ReadField(std::string &r_s_line, FILE *p_fr)
{
	r_s_line.erase();

	int c = fgetc(p_fr);
	while(isspace(uint8_t(c)) && c != EOF)
		c = fgetc(p_fr);
	// skip space

	try {
		for(; !isspace(uint8_t(c)) && c != EOF; c = fgetc(p_fr)) {
			/*if(ferror(p_fr)) // if some other reading error happens, fgetc() also returns EOF
				return false;*/
			r_s_line += c; // some string implementations don't have push_back()
		}
	} catch(std::bad_alloc&) {
		return false;
	}
	// read non-whitespace

	return !ferror(p_fr);
}

bool CParserBase::Split(std::vector<std::string> &r_s_dest, const std::string &r_s_string,
	const char *p_s_separator, int n_thresh)
{
	r_s_dest.clear();
	const size_t n_separator_skip = strlen(p_s_separator);
	try {
		size_t n_pos = 0;
		size_t n_next_pos;
		while((n_next_pos = r_s_string.find(p_s_separator, n_pos)) != std::string::npos) {
			if(n_thresh < 0 || n_next_pos - n_pos > unsigned(n_thresh)) {
				r_s_dest.resize(r_s_dest.size() + 1);
				std::string &r_s_new = r_s_dest.back();
				r_s_new.insert(r_s_new.begin(), r_s_string.begin() + n_pos,
					r_s_string.begin() + n_next_pos);
			}
			n_pos = n_next_pos + n_separator_skip; // skip over the separator
		}
		if(n_thresh < 0 || r_s_string.length() - n_pos > unsigned(n_thresh)) {
			r_s_dest.resize(r_s_dest.size() + 1);
			std::string &r_s_new = r_s_dest.back();
			r_s_new.insert(r_s_new.begin(), r_s_string.begin() + n_pos,
				r_s_string.end());
		}
	} catch(std::bad_alloc&) {
		return false;
	}

	return true;
}

/*
 *								=== ~CParser ===
 */
