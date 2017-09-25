/**
 *	@file nvm2graph.cpp
 *	@author Marek Solony (isolony(at)fit.vutbr.cz)
 *	@date 2014
 *	@brief Convertor from VisualSFM output (.nvm) into graph file for SLAM++
 *		details about .nvm format can be found at: http://ccwu.me/vsfm/doc.html
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <Eigen/Dense>

namespace Eigen { // inject some new types in Eigen namespace

typedef Eigen::Matrix<int, 2, 1, Eigen::DontAlign> Vector2i_u;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2d_u;
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3d_u;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 10, 1, Eigen::DontAlign> Vector10d_u;

};

/**
 *	@brief read a token (sequence of non-whitespace characters) from a file
 *
 *	@param[out] r_dest is filled with the token upon return (contents overwritten)
 *	@param[in] p_fr is pointer to a file opened for reading (reads any preceding
 *		space, the token and one more space character which is discarded)
 */
void ReadToken(std::string &r_dest, FILE *p_fr)
{
	r_dest.erase();
	if(!feof(p_fr)) {
		int c;
		while(isspace((unsigned char)(c = fgetc(p_fr))) && c != EOF)
			;
		if(c == EOF)
			return;
		// skip any preceding space

		assert(!isspace((unsigned char)c));
		r_dest += c;
		while(!isspace((unsigned char)(c = fgetc(p_fr))) && c != EOF)
			r_dest += c;
		// read all non-space characters and one EOF or space (will be discarded)
	}
}

/**
 *	@brief a simple text-based progress indicator
 */
class CTextProgressIndicator {
protected:
	const char *m_p_s_task_name; /**< @brief progress indicator label */
	const long m_n_update_interval; /**< @brief update interval (clocks) */
	long m_n_next_opdate; /**< @brief next update (clocks) */
	const int m_n_indent; /**< @brief indentation */
	std::string m_s_progress; /**< @brief last progress printed */
	FILE *m_p_stream; /**< @brief output stream (default stdout) */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] p_s_banner is progress indicator label
	 *	@param[in] f_update_interval is update interval, in seconds (default 0.25)
	 *	@param[in] n_indent is progress indicator indentation, in characters (default 16)
	 *
	 *	@note This does not display the progress indicator; Show() needs to be called.
	 */
	CTextProgressIndicator(const char *p_s_banner, double f_update_interval = .25, int n_indent = 16)
		:m_p_s_task_name(p_s_banner), m_n_update_interval(long(f_update_interval * CLOCKS_PER_SEC)),
		m_n_next_opdate(0), m_n_indent(n_indent), m_p_stream(stdout)
	{}

	/**
	 *	@brief sets the output stream (default stream is stdout)
	 *	@param[in] p_stream is pointer to the stream to be used
	 */
	void Set_Stream(FILE *p_stream)
	{
		m_p_stream = p_stream;
	}

	/**
	 *	@brief displays progress indicator with fraction
	 *
	 *	@param[in] n_progress is current progress value
	 *	@param[in] n_goal is goal progress value
	 */
	void Show(unsigned int n_progress, unsigned int n_goal)
	{
		_ASSERTE(n_goal > 0); // otherwise causes division by zero

		long n_time = clock();
		if(n_time < m_n_next_opdate)
			return;
		m_n_next_opdate = n_time + m_n_update_interval;
		// do not update too often

		n_progress = std::min(n_goal, n_progress); // do not exceed goal, otherwise will do out of bounds access
		char p_s_counts[64], p_s_progress[21];
		sprintf(p_s_counts, "%u / %u", n_progress, n_goal);
		sprintf(p_s_progress, "%-20s", "====================" +
			std::max(0, std::min(20, int((20 - (n_progress * 20) / n_goal)))));

		_ASSERTE(strlen(p_s_progress) >= strlen(p_s_counts));
		size_t n_left = (strlen(p_s_progress) > strlen(p_s_counts))?
			(strlen(p_s_progress) - strlen(p_s_counts)) / 2 : 0;
		size_t n_right = n_left + strlen(p_s_counts);
		_ASSERTE(strlen(p_s_progress) > 0); // otherwise p_s_progress.length() - 1 underflows
		for(size_t j = n_left, m = strlen(p_s_progress) - 1; j < n_right; ++ j) {
			if(p_s_counts[j - n_left] != ' ' && (j < m ||
			   p_s_progress[j] != '=' || p_s_progress[j + 1] != ' '))
				p_s_progress[j] = p_s_counts[j - n_left];
			// keep the leading '=' always visible
		}

		if(!m_s_progress.compare(p_s_progress))
			return;
		// don't print the same thing again

		fprintf(m_p_stream, "\r%-*s [%s]", m_n_indent, m_p_s_task_name, p_s_progress);
		m_s_progress = p_s_progress;
	}

	/**
	 *	@brief displays bare progress indicator
	 *	@param[in] f_progress is current progress value (in range 0 - 1)
	 */
	void Show(double f_progress)
	{
		long n_time = clock();
		if(n_time < m_n_next_opdate)
			return;
		m_n_next_opdate = n_time + m_n_update_interval;
		// do not update too often

		int n_progress = std::max(0, std::min(20, int(20 - (f_progress * 20))));
		char p_s_progress[21];
		sprintf(p_s_progress, "%-20s", "====================" + n_progress);

		if(!m_s_progress.compare(p_s_progress))
			return;
		// don't print the same thing again

		fprintf(m_p_stream, "\r%-*s [%s]", m_n_indent, m_p_s_task_name, p_s_progress);
		m_s_progress = p_s_progress;
	}

	/**
	 *	@brief prints a full progress indicator
	 *	@param[in] p_s_additional_info is additional information (printed after
	 *		a colon after the indicator, should not contain newlines, can be null)
	 */
	void Done(const char *p_s_additional_info = 0)
	{
		if(p_s_additional_info) {
			fprintf(m_p_stream, "\r%-*s [%s] : %s\n", m_n_indent, m_p_s_task_name,
				"====================", p_s_additional_info);
		} else
			fprintf(m_p_stream, "\r%-*s [%s]\n", m_n_indent, m_p_s_task_name, "====================");
		m_s_progress.erase();
	}
};

int main(int argc, char *argv[])
{	
	std::vector<std::pair<Eigen::Vector3d_u, int> > pts3d; // position, number of refs
	std::vector<Eigen::Vector10d_u> cam;
	std::vector<std::pair<Eigen::Vector2d_u, Eigen::Vector2i_u> > observation; // 2D observation, camera id, point id

	if(argc != 3) {
		std::cout << "Convertor from VisualSFM output (.nvm) into graph file for SLAM++" << std::endl <<
				"details about .nvm format can be found at: http://ccwu.me/vsfm/doc.html" << std::endl << std::endl;
		std::cout << "Usage: nvm2graph <input .nvm file> <output .graph filename>" << std::endl;
		return -1;
	}
	// check commandline

	FILE *pFile;
	if(!(pFile = fopen(argv[1], "r+"))) {
		fprintf(stderr, "error: failed to open input file \'%s\'\n", argv[1]);
		return -1;
	}

	std::cout << "nvm -> graph" << std::endl;

	std::string s_token;
	ReadToken(s_token, pFile);
	if(s_token != "NVM_V3") {
		fprintf(stderr, "error: bad format \'%s\': expected \'NVM_V3\'\n", s_token.c_str());
		fclose(pFile);
		return -1;
	}
	// make sure there is NVM_V3 at the beginning

	int cams;
	ReadToken(s_token, pFile);
	if(s_token == "FixedK") {
		double K[5];
		fscanf(pFile, "%lf %lf %lf %lf %lf", &K[0], &K[1], &K[2], &K[3], &K[4]);
		std::cout << "have FixedK (fx cx fy cy r): " << K[0] << " " << K[1] << " " << K[2] << " " << K[3] << " " << K[4] << std::endl;
		fscanf(pFile, "%d", &cams);
	} else if(s_token.empty() || !isdigit(s_token[0])) {
		fprintf(stderr, "error: bad format \'%s\': expected <number of cameras>\n", s_token.c_str());
		fclose(pFile);
		return -1;
	} else
		cams = atol(s_token.c_str());
	// either there is fixedK or there is the number of cameras

	std::cout << "cams: " << cams << std::endl;
	cam.reserve(cams);
	for(int a = 0; a < cams; ++ a) {
		std::string s_image_file; // todo - not sure what happens if there are spaces? does it put quotes?
		ReadToken(s_image_file, pFile);
		Eigen::Vector10d v1;
		for(int b = 0; b < 10; ++ b)
			fscanf (pFile, "%lf", &v1(b)); // use doubles!

		cam.push_back(v1);
	}
	// load cams

	int points;
	fscanf(pFile, "%d", &points);
	std::cout << "pnts: " << points << std::endl;

	CTextProgressIndicator progress("loading nvm");
	pts3d.reserve(points);
	for(int a = 0; a < points; ++ a) {
		progress.Show(a, points);

		Eigen::Vector3d_u v1;
		fscanf(pFile, "%lf %lf %lf", &v1(0), &v1(1), &v1(2)); // use doubles!
		int rgb[3];
		fscanf(pFile, "%d %d %d", &rgb[0], &rgb[1], &rgb[2]);
		int refs;
		fscanf(pFile, "%d", &refs);
		// <XYZ> <RGB> <number of measurements>

		pts3d.push_back(std::make_pair(v1, refs));

		for(int b = 0; b < refs; ++ b) {
			Eigen::Vector2i_u i2;
			Eigen::Vector2d_u v2;
			fscanf(pFile, "%d %d %lf %lf", &i2(0), &i2(1), &v2(0), &v2(1)); // use doubles!
			i2(1) = a;	//which point?
			// <Image index> <Feature Index> <xy>

			observation.push_back(std::make_pair(v2, i2));
		}
	}
	fclose(pFile);
	progress.Done();
	// load points and observations (this may take a while)

	std::ofstream wFile;
	wFile.open(argv[2]);
	if(!wFile.is_open()) {
		fprintf(stderr, "error: failed to open output file \'%s\'\n", argv[2]);
		return -1;
	}

	for(size_t a = 0, n = cam.size(); a < n; ++ a) {
		Eigen::Quaternion<double> quat((cam[a])(1), (cam[a])(2), (cam[a])(3), (cam[a])(4));
		quat = quat.inverse();
		// get quat

		Eigen::Vector3d t_vec((cam[a])(5), (cam[a])(6), (cam[a])(7));
		Eigen::Vector3d c = t_vec;//-(quat * (t_vec)); // rotate?
		// yes its like this, they have R and C

		wFile << "VERTEX_CAM " << a << " " << c(0) << " " << c(1) << " " << c(2) << " " <<
			quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " " <<
			(cam[a])(0) << " " << (cam[a])(0) << " " << 0 << " " << 0 << " " << 0/*(cam[a])(8)*/ << std::endl;
		// <focal length> <quaternion WXYZ> <camera center> <radial distortion> <null>
		// 0              1 2 3 4           5 6 7           8                   9
		// store to file
	}
	for(size_t a = 0, n = pts3d.size(), obs_count = 0; a < n; ++ a) {
		wFile << "VERTEX_XYZ " << (a + cams) << " " << (pts3d[a].first)(0) << " " <<
			(pts3d[a].first)(1) << " " << (pts3d[a].first)(2) << std::endl;

		for(int b = 0, m = pts3d[a].second; b < m; ++ b, ++ obs_count) {
			wFile << "EDGE_PROJECT_P2MC " << (a + cams) << " " <<
				(observation[obs_count].second)(0) << " " <<
				(observation[obs_count].first)(0) << " " <<
				(observation[obs_count].first)(1) << " 1 0 1" << std::endl;
		}
	}
	// dump the dataset to graph format

	wFile.close();
	std::cout << "done: " << cams << std::endl;

	return 0;
}
