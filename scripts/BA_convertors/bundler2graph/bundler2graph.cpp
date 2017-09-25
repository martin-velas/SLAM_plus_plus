/**
 *	@author Marek Solony (isolony(at)fit.vutbr.cz)
 *	@date 2014
 *	@brief Convertor from Bundler output (.out) into graph file for SLAM++
 *		details about .out format can be found at: http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html#S6
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Dense>

#define PI 3.14159265359

using namespace std;

int counter = 0;
std::map<int, int> table;

static void Quat_to_AxisAngle(const Eigen::Quaterniond &r_quat, Eigen::Vector3d &r_axis_angle)
{
	double f_half_angle = /*(r_quat.w() <= 0)? asin(r_quat.vec().norm()) :*/ acos(r_quat.w()); // 0 .. pi

	if(f_half_angle < 1e-12)
		r_axis_angle = Eigen::Vector3d(0, 0, 0); // lim(sin(x) / x) for x->0 equals 1, we're therefore multiplying a null vector by 1
	else {
		double f_angle = 2 * ((r_quat.w() <= 0)? f_half_angle - M_PI : f_half_angle);
		r_axis_angle = r_quat.vec() * (f_angle / sin(f_half_angle));
	}
}
static void AxisAngle_to_Quat(const Eigen::Vector3d &r_axis_angle, Eigen::Quaterniond &r_quat)
{
	double f_angle = r_axis_angle.norm();
	if(f_angle < 1e-12)
		r_quat = Eigen::Quaterniond(1, 0, 0, 0); // cos(0) = 1
	else {
		//_ASSERTE(f_angle <= M_PI); // sometimes broken
		f_angle = fmod(f_angle, M_PI * 2);
		double q = (sin(f_angle * .5) / f_angle);
		r_quat = Eigen::Quaterniond(cos(f_angle * .5), r_axis_angle(0) * q,
			r_axis_angle(1) * q, r_axis_angle(2) * q);
		r_quat.normalize();
	}
}


int main(int argc, char *argv[]) {
	char edge[64];
	int v[2];
	float pos[3];
	int col[3];
	float rot[4];
	float conv[21];

	std::vector<Eigen::VectorXd> pts3d;
	std::vector<Eigen::VectorXd> cam;
	std::vector<Eigen::VectorXd> observation;

	if(argc == 3)
	{
		FILE * pFile = fopen (argv[1],"r+");

		std::ofstream wFile;
		wFile.open(argv[2]);

		std::cout << "bundler -> graph" << std::endl;
		//scan n cams and n points
		int cams, points;
		fscanf (pFile, "%d %d", &cams, &points);

		std::cout << "cams: " << cams << std::endl;
		std::cout << "points: " << points << std::endl;
		//load cams
		std::vector<int> camdxs;
		for(int a = 0; a < cams; a++) {
			Eigen::VectorXd v1(15);

			camdxs.push_back(-1);

			for(int b = 0; b < 15; b++) {
				fscanf (pFile, "%f", &conv[0] );
				v1(b) = conv[0];
			}
			cam.push_back(v1);
		}
		//load points and observations
		for(int a = 0; a < points; a++) {
			if(a % 100000 == 0)
				std::cout << a << std::endl;
	
			Eigen::VectorXd v1(4);
			fscanf (pFile, "%f %f %f", &pos[0], &pos[1], &pos[2] );
			v1(0) = pos[0];
			v1(1) = pos[1];
			v1(2) = pos[2];
			fscanf (pFile, "%d %d %d", &col[0], &col[1], &col[2] );

			int refs;
			fscanf (pFile, "%d", &refs );

			v1(3) = refs;
			pts3d.push_back(v1);

			for(int b = 0; b < refs; b++) {
				Eigen::VectorXd v2(4);
				v2(1) = a;	//which point?
				fscanf (pFile, "%d %d %f %f", &v[1], &v[0], &pos[0], &pos[1]);

				camdxs.at(v[1]) = v[1];

				v2(0) = v[1];
				v2(2) = pos[0];
				v2(3) = pos[1];

				observation.push_back(v2);
			}
		}

		std::cout << "cams " << cam.size() << std::endl;
		std::cout << "pts " << pts3d.size() << std::endl;
		//DUMP THE DATASET TO GRAPH FORMAT
		for (int a = 0; a < cam.size(); a++) {
			//convert rotation matrix to inverse quaternion
			Eigen::Matrix3d rot;
			rot << (cam[a])(3), (cam[a])(4), (cam[a])(5), 
				(cam[a])(6), (cam[a])(7), (cam[a])(8),
				(cam[a])(9), (cam[a])(10), (cam[a])(11);
			//invert
			rot = rot.inverse().eval();

			//to quat
			Eigen::Quaternion<double> quat(rot);

			Eigen::Vector3d t_vec((cam[a])(12), (cam[a])(13), (cam[a])(14));
			//rotate
			Eigen::Vector3d c = -(quat * (t_vec));

			//store to file
			wFile << "VERTEX_CAM " << a << " " << c(0) << " " << c(1) << " " << c(2) << " " <<
				quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " " <<
				(cam[a])(0) << " " << (cam[a])(0) << " " << 0 << " " << 0 << " " << 0 << std::endl;
		}

		int obs_count = 0;
		for (int a = 0; a < pts3d.size(); a++) {
			wFile << "VERTEX_XYZ " << a + cam.size() << " " << 
				(pts3d[a])(0) << " " << (pts3d[a])(1) << " " << (pts3d[a])(2) << std::endl;
			
			for(int b = 0; b < (pts3d[a])(3); b++) {
				wFile << "EDGE_PROJECT_P2MC " << a + cam.size() << " " << (observation[obs_count])(0) << " " <<
				-(observation[obs_count])(2) << " " << -(observation[obs_count])(3) << " 1 0 1" << std::endl;

				camdxs.at((observation[b])(0)) = (observation[b])(0);
				
				obs_count ++;
			}
		}

	//pFile.close();
	wFile.close();
	std::cout << "done " << cams << std::endl;
	} else {
		std::cout << "Convertor from Bundler output (.out) into graph file for SLAM++" << std::endl <<
				"details about .out format can be found at: http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html#S6" << std::endl << std::endl;
		std::cout << "Usage: bundler2graph <input .out file> <output .graph filename>" << std::endl;
	}

	return 0;
}
