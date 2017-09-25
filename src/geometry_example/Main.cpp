/*
								+-----------------------------------+
								|                                   |
								| ***  Geometric Tools Example  *** |
								|                                   |
								|  Copyright  (c) Viorela Ila 2016  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/geometry_example/Main.cpp
 *	@brief contains the main() function of the geometric tools example program
 *	@author Viorela Ila
 *	@date 2016-10-12
 */

#include <stdio.h> // printf
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#include "slam/3DSolverBase.h" // SE(3) functions
#include "slam/Timer.h" // timing

typedef C3DJacobians::TSE3 TSE3; // SE(3) pose

#include "geometry/P3P.h" // todo - implement own P3P in here

#include "geometry/Polynomial.h"
#include "geometry/PolySolve.h"
#include "geometry/Kabsch.h"
#include "geometry/DistortionModel.h"
#include "geometry/StructAverage.h"
#include "geometry/Triangulate.h"
#include "geometry/TwoView.h"
#include "geometry/Homography.h"
// include them just to test compiling them

void P3P_Test();
void Kabsch_Test();
void TwoView_GB_Test();
void Homography_Test();

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
	P3P_Test();
	Kabsch_Test();
	TwoView_GB_Test();
	Homography_Test();

	return 0;
}

void Kabsch_Test()
{
	for(int i = 0; i < 100; ++ i) {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
		Eigen::Quaterniond v_quat = Eigen::Quaterniond::UnitRandom();
#else // EIGEN_VERSION_AT_LEAST(3, 3, 0)
		Eigen::Vector3d u = (Eigen::Vector3d::Random().array() * .5 + .5).matrix();
		Eigen::Quaterniond v_quat(sqrt(u(0)) * cos(M_PI * 2 * u(2)), // w
			sqrt(1.0 - u(0)) * sin(M_PI * 2 * u(1)), // x
			sqrt(1.0 - u(0)) * cos(M_PI * 2 * u(1)), // y
			sqrt(u(0)) * sin(M_PI * 2 * u(2))); // z
		_ASSERTE(fabs(v_quat.norm() - 1) < 1e-5);
#endif // EIGEN_VERSION_AT_LEAST(3, 3, 0)
		// get a random rotation

		bool b_use_scale = (i & 1) != 0;
		double f_scale = (b_use_scale)? .1 + 2 * double(rand()) / RAND_MAX : 1;
		Eigen::Matrix4d T;
		T.setIdentity();
		T.topLeftCorner<3, 3>() = v_quat.toRotationMatrix() * f_scale;
		T.rightCols<1>().head<3>() = Eigen::Vector3d::Random() * 10;
		// get a random translation, assemble a Rt matrix T

		Eigen::Matrix<double, 3, 50> pointsA;
		pointsA.setRandom();
		// get points (each column is a point)

		Eigen::Matrix<double, 3, 50> pointsB;
		for(int j = 0; j < pointsB.cols(); ++ j)
			pointsB.col(j) = (T.topLeftCorner<3, 3>() * pointsA.col(j) + T.rightCols<1>().head<3>());
		// get transformed points (each column is a point); B = T * A

		{
			CAttitudeEstimator_Kabsch kabsch(
				CAttitudeEstimator_Kabsch::v_CentroidStacked(pointsB), // tar
				CAttitudeEstimator_Kabsch::v_CentroidStacked(pointsA)); // src
			kabsch.ObserveStacked(pointsB, pointsA); // tar, src
			Eigen::Matrix4d T_est = kabsch.t_Estimate_Transform(b_use_scale);

			double f_error = (T - T_est).norm();
			_ASSERTE(f_error < 1e-10); // should be zero

			Eigen::Vector8d v_est = kabsch.v_Estimate_Transform_Quat(b_use_scale);
			Eigen::Matrix4d T_est1 = Eigen::Matrix4d::Identity();
			T_est1.topLeftCorner<3, 3>() = Eigen::Quaterniond(v_est.head<7>().tail<4>()).toRotationMatrix() * v_est(7);
			T_est1.rightCols<1>().head<3>() = v_est.head<3>();
			// also test the quat function

			double f_error1 = (T - T_est1).norm();
			_ASSERTE(f_error1 < 1e-10); // should be zero
		}
		// stacked

		{
			CAttitudeEstimator_Kabsch kabsch(
				CAttitudeEstimator_Kabsch::v_CentroidStacked(pointsB), // tar
				CAttitudeEstimator_Kabsch::v_CentroidStacked(pointsA)); // src
			for(int j = 0; j < pointsB.cols(); ++ j)
				kabsch.ObservePair(pointsB.col(j), pointsA.col(j)); // tar, src
			Eigen::Matrix4d T_est = kabsch.t_Estimate_Transform(b_use_scale);

			double f_error = (T - T_est).norm();
			_ASSERTE(f_error < 1e-10); // should be zero

			Eigen::Vector8d v_est = kabsch.v_Estimate_Transform_Quat(b_use_scale);
			Eigen::Matrix4d T_est1 = Eigen::Matrix4d::Identity();
			T_est1.topLeftCorner<3, 3>() = Eigen::Quaterniond(v_est.head<7>().tail<4>()).toRotationMatrix() * v_est(7);
			T_est1.rightCols<1>().head<3>() = v_est.head<3>();
			// also test the quat function

			double f_error1 = (T - T_est1).norm();
			_ASSERTE(f_error1 < 1e-10); // should be zero
		}
		// pairs

		{
			std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign> >
				points_A(pointsA.cols()), points_B(pointsB.cols());
			for(int j = 0; j < pointsB.cols(); ++ j) {
				points_A[j] = pointsA.col(j);
				points_B[j] = pointsB.col(j);
			}
			// copy to vectors

			CAttitudeEstimator_Kabsch kabsch(
				CAttitudeEstimator_Kabsch::v_Centroid(points_B.begin(), points_B.end()), // tar
				CAttitudeEstimator_Kabsch::v_Centroid(points_A.begin(), points_A.end())); // src
			for(int j = 0; j < pointsB.cols(); ++ j)
				kabsch.ObservePair(points_B[j], points_A[j]); // tar, src
			Eigen::Matrix4d T_est = kabsch.t_Estimate_Transform(b_use_scale);

			double f_error = (T - T_est).norm();
			_ASSERTE(f_error < 1e-10); // should be zero

			Eigen::Vector8d v_est = kabsch.v_Estimate_Transform_Quat(b_use_scale);
			Eigen::Matrix4d T_est1 = Eigen::Matrix4d::Identity();
			T_est1.topLeftCorner<3, 3>() = Eigen::Quaterniond(v_est.head<7>().tail<4>()).toRotationMatrix() * v_est(7);
			T_est1.rightCols<1>().head<3>() = v_est.head<3>();
			// also test the quat function

			double f_error1 = (T - T_est1).norm();
			_ASSERTE(f_error1 < 1e-10); // should be zero
		}
		// vectors
	}
}

void Polynomial_CoverageTest()
{
	_ASSERTE(0); // coverage test only; don't call (there are no data in the samples / weights, would fail at runtime)

	std::vector<Eigen::Matrix<double, 2, 1, Eigen::DontAlign> > samples;
	std::vector<double> weights;

	CPolynomial<4, 0, 1> poly;
	poly.LeastSquares_Fit(samples);
	poly.LeastSquares_Fit(samples, weights);
	poly.f_LeastSquares_Error(samples);
	poly.f_LeastSquares_Error(samples, weights);
	poly.Robust_Fit_Update(samples, CHuberLossd());
	poly.Robust_Fit_Update(samples, weights, CHuberLossd());
	poly.f_LeastSquares_RobustError(samples, CHuberLossd());
	poly.f_LeastSquares_RobustError(samples, weights, CHuberLossd());
}

void TwoView_GB_Test()
{
	for(int i = 0; i < 100; ++ i) {
		Eigen::Matrix<double, 3, 5> A = Eigen::Matrix<double, 3, 5>::Random(),
			B = Eigen::Matrix<double, 3, 5>::Random();

		std::vector<Eigen::Matrix<double, 3, 3, Eigen::DontAlign> > mats;
		CFivePoint_EssentialSolver_Grobner::Solve(A, B, mats);
		for(size_t j = 0, m = mats.size(); j < m; ++ j) {
			Eigen::Matrix3d E = mats[j];
			double f_det = E.determinant();
			Eigen::Matrix3d t_trace_constraint = 2 * E * E.transpose() * E -
				(E * E.transpose()).trace() * E;
			Eigen::Matrix<double, 5, 1> v_reproj = (A.transpose() * E * B).diagonal();
			_ASSERTE(fabs(f_det) < 1e-10); // should be zero
			_ASSERTE(t_trace_constraint.lpNorm<Eigen::Infinity>() < 1e-9); // should be zero
			_ASSERTE(v_reproj.lpNorm<Eigen::Infinity>() < 1e-10); // should be zero
		}
	}
	// test the two-view solver
}

void P3P_Test()
{
	int n_test_num = 1000;
	int n_point_num = 50;
	double f_min_point_dist = .1;
	double f_max_point_dist = 10;
	double f_min_problem_scale = .1, f_max_problem_scale = 10; // min/max scale of the whole thing
	double f_max_problem_translation = 100; // random translation of the whole thing (after the random scaling)
	double f_max_point_noise = 0; // no noise to evaluate numerics
	double f_max_observation_noise = 0;
	// test parameters

	CDeltaTimer t;
	double f_gen_time = 0;
	double f_p3p_time = 0;
	double f_p3p_rev_time = 0;
	double f_pnp_time = 0;
	double f_fpnp_time = 0;

	std::vector<double> p3p_Rt_devs;
	std::vector<double> p3p_rev_Rt_devs;
	std::vector<double> pnp_Rt_devs;
	std::vector<double> fpnp_Rt_devs;
	std::vector<double> p3p_reproj_RMSE;
	std::vector<double> p3p_rev_reproj_RMSE;
	std::vector<double> pnp_reproj_RMSE;
	std::vector<double> fpnp_reproj_RMSE;

	for(int n_test = 0; n_test < n_test_num; ++ n_test) {
		std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign> > points(n_point_num);
		for(int i = 0; i < n_point_num; ++ i)
			points[i] = Eigen::Vector3d::Random();
		// generate some random points in a [-1, 1] cube

		Eigen::Vector3d v_aang = Eigen::Vector3d::Random();
		v_aang.array() /= v_aang.array().cos();
		v_aang *= fmod(Eigen::Matrix<double, 1, 1>::Random()(0) * 2 * M_PI, 2 * M_PI) / v_aang.norm();
		// generate random rotation (with uniformly distributed rotation axes and angles) // todo - make this a function

		Eigen::Matrix3d R;
		{
			Eigen::Quaterniond quat;
			C3DJacobians::AxisAngle_to_Quat(v_aang, quat);
			R = quat.toRotationMatrix();
		}
		// get a rotation matrix

		Eigen::Vector3d v_position;
		do {
			v_position = Eigen::Vector3d::Random() * f_max_point_dist;
		} while(!(v_position.array().abs() > 1.0 + f_min_point_dist).all());
		// generate a random position that is outside of the unit cube that contains the points

		bool b_points_behind = false;
		std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign> > directions(n_point_num);
		for(int i = 0; i < n_point_num; ++ i) {
			directions[i] = (R * (points[i] - v_position)).normalized();
			if(directions[i](2) <= 1e-3) {
				b_points_behind = true; // silly way
				break;
			}
		}
		if(b_points_behind) { // will trip PNP which uses this information to discard solutions
			-- n_test;
			continue;
		}
		// generate rotated directions from the camera position to the points

		Eigen::Vector3d v_world_translation = Eigen::Vector3d::Random() * f_max_problem_translation;
		double f_scale = f_min_problem_scale + double(rand()) / RAND_MAX *
			(f_max_problem_scale - f_min_problem_scale);

		Eigen::Matrix<double, 3, Eigen::Dynamic> t_positions(3, n_point_num), t_directions(3, n_point_num);
		for(int i = 0; i < n_point_num; ++ i) {
			t_positions.col(i) = points[i] * f_scale + Eigen::Vector3d::Random() * f_max_point_noise +
				v_world_translation;
			t_directions.col(i) = (directions[i] + Eigen::Vector3d::Random() *
				f_max_observation_noise).normalized();
		}
		// assemble the matrices for PNP / P3P

		Eigen::Matrix3d t_expected_R = R/*.transpose()*/;
		Eigen::Vector3d v_expected_t = -R * (v_position * f_scale/*-(t_expected_R * (v_position * f_scale))*/ + v_world_translation);
		// expectation of the results (both solvers give transformation from world to camera) // since 2017-07-04, before it was camera to world

		f_gen_time += t.f_Time();

		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > p3p_rev_solutions;
		CP3PSolver::Solve_Rev(t_directions.leftCols<3>(), t_positions.leftCols<3>(), p3p_rev_solutions);
		// run the P3P solver (get the reverse transform using the original code)

		f_p3p_rev_time += t.f_Time();

		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > p3p_solutions;
		CP3PSolver::Solve(t_directions.leftCols<3>(), t_positions.leftCols<3>(), p3p_solutions);
		// run the P3P solver

		f_p3p_time += t.f_Time();

		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > pnp_solutions;
		CPNPSolver_DLS::Solve(t_directions, t_positions, pnp_solutions);
		// run the PNP solver

		f_pnp_time += t.f_Time();

		std::vector<Eigen::Matrix<double, 3, 4, Eigen::DontAlign> > fpnp_solutions;
		CPNPSolver_DLS::FastSolve(t_directions, t_positions, fpnp_solutions);
		// run the PNP solver

		f_fpnp_time += t.f_Time();

		Eigen::Matrix<double, 3, 4> Rt_ref;
		Rt_ref.leftCols<3>() = t_expected_R;
		Rt_ref.rightCols<1>() = v_expected_t;
		if(!p3p_solutions.empty()) {
			double f_P3P_best = 1e100, f_P3P_best_RMSE = 1e100;
			for(size_t i = 0, n = p3p_solutions.size(); i < n; ++ i) {
				f_P3P_best = std::min(f_P3P_best, (Rt_ref - p3p_solutions[i]).norm());

				Eigen::Matrix4d t_world_to_cam = Eigen::Matrix4d::Identity();
				t_world_to_cam.topRows<3>() = p3p_solutions[i];
				// the solution is a world to camera transformation

				double f_RMSE = 0;
				for(int j = 0; j < n_point_num; ++ j) {
					Eigen::Vector4d v_expectation4 = t_world_to_cam.leftCols<3>() *
						t_positions.col(j) + t_world_to_cam.rightCols<1>();
					Eigen::Vector3d v_expectation = v_expectation4.head<3>() / v_expectation4(3);
					Eigen::Vector3d v_observation = t_directions.col(j);
					double f_error = (v_observation.normalized() - v_expectation.normalized()).squaredNorm();
					f_RMSE += f_error;
				}
				f_RMSE = sqrt(f_RMSE / n_point_num);
				f_P3P_best_RMSE = std::min(f_P3P_best_RMSE, f_RMSE);
			}
			p3p_Rt_devs.push_back(f_P3P_best);
			p3p_reproj_RMSE.push_back(f_P3P_best_RMSE);
		}
		if(!p3p_rev_solutions.empty()) {
			double f_P3P_best = 1e100, f_P3P_best_RMSE = 1e100;
			for(size_t i = 0, n = p3p_rev_solutions.size(); i < n; ++ i) {

				Eigen::Matrix4d t_cam_to_world = Eigen::Matrix4d::Identity();
				t_cam_to_world.topRows<3>() = p3p_rev_solutions[i];
				// the solution is a camera to world transformation
				
				Eigen::Matrix4d t_world_to_cam = t_cam_to_world.inverse();
				// we need the other one to evaluate

				f_P3P_best = std::min(f_P3P_best, (Rt_ref - t_world_to_cam.topRows<3>()).norm());

				double f_RMSE = 0;
				for(int j = 0; j < n_point_num; ++ j) {
					Eigen::Vector4d v_expectation4 = t_world_to_cam.leftCols<3>() *
						t_positions.col(j) + t_world_to_cam.rightCols<1>();
					Eigen::Vector3d v_expectation = v_expectation4.head<3>() / v_expectation4(3);
					Eigen::Vector3d v_observation = t_directions.col(j);
					double f_error = (v_observation.normalized() - v_expectation.normalized()).squaredNorm();
					f_RMSE += f_error;
				}
				f_RMSE = sqrt(f_RMSE / n_point_num);
				f_P3P_best_RMSE = std::min(f_P3P_best_RMSE, f_RMSE);
			}
			p3p_rev_Rt_devs.push_back(f_P3P_best);
			p3p_rev_reproj_RMSE.push_back(f_P3P_best_RMSE);
		}
		if(!pnp_solutions.empty()) {
			double f_PNP_best = 1e100, f_PNP_best_RMSE = 1e100;
			for(size_t i = 0, n = pnp_solutions.size(); i < n; ++ i) {
				f_PNP_best = std::min(f_PNP_best, (Rt_ref - pnp_solutions[i]).norm());

				Eigen::Matrix4d t_world_to_cam = Eigen::Matrix4d::Identity();
				t_world_to_cam.topRows<3>() = pnp_solutions[i];
				// the solution is a world to camera transformation

				double f_RMSE = 0;
				for(int j = 0; j < n_point_num; ++ j) {
					Eigen::Vector4d v_expectation4 = t_world_to_cam.leftCols<3>() *
						t_positions.col(j) + t_world_to_cam.rightCols<1>();
					Eigen::Vector3d v_expectation = v_expectation4.head<3>() / v_expectation4(3);
					Eigen::Vector3d v_observation = t_directions.col(j);
					double f_error = (v_observation.normalized() - v_expectation.normalized()).squaredNorm();
					f_RMSE += f_error;
				}
				f_RMSE = sqrt(f_RMSE / n_point_num);
				f_PNP_best_RMSE = std::min(f_PNP_best_RMSE, f_RMSE);
			}
			pnp_Rt_devs.push_back(f_PNP_best);
			pnp_reproj_RMSE.push_back(f_PNP_best_RMSE);
		}
		if(!fpnp_solutions.empty()) {
			double f_FPNP_best = 1e100, f_FPNP_best_RMSE = 1e100;
			for(size_t i = 0, n = fpnp_solutions.size(); i < n; ++ i) {
				f_FPNP_best = std::min(f_FPNP_best, (Rt_ref - fpnp_solutions[i]).norm());

				Eigen::Matrix4d t_world_to_cam = Eigen::Matrix4d::Identity();
				t_world_to_cam.topRows<3>() = fpnp_solutions[i];
				// the solution is a world to camera transformation

				double f_RMSE = 0;
				for(int j = 0; j < n_point_num; ++ j) {
					Eigen::Vector4d v_expectation4 = t_world_to_cam.leftCols<3>() *
						t_positions.col(j) + t_world_to_cam.rightCols<1>();
					Eigen::Vector3d v_expectation = v_expectation4.head<3>() / v_expectation4(3);
					Eigen::Vector3d v_observation = t_directions.col(j);
					double f_error = (v_observation.normalized() - v_expectation.normalized()).squaredNorm();
					f_RMSE += f_error;
				}
				f_RMSE = sqrt(f_RMSE / n_point_num);
				f_FPNP_best_RMSE = std::min(f_FPNP_best_RMSE, f_RMSE);
			}
			fpnp_Rt_devs.push_back(f_FPNP_best);
			fpnp_reproj_RMSE.push_back(f_FPNP_best_RMSE);
		}
		// silly error eval

		// t_odo - read or generate test data
		// t_odo - calculate P3P on the test data
		// t_odo - evaluate angular / translational errors? evaluate reprojection errors?
		//      - we have no K but could give errors in radians
		// todo - accumulate error histograms, and mean
	}

	printf("P3P time (Kneip et al.): %.5f msec\n", f_p3p_time / n_test_num * 1000);
	printf("P3P rev time (Kneip et al.): %.5f msec\n", f_p3p_rev_time / n_test_num * 1000);
	printf("PNP time (Hesch et al.): %.5f msec\n", f_pnp_time / n_test_num * 1000);
	printf("PNP time (Hesch et al.): %.5f msec (fast variant with Cayley singularity)\n", f_fpnp_time / n_test_num * 1000);

	std::nth_element(p3p_Rt_devs.begin(), p3p_Rt_devs.begin() + (p3p_Rt_devs.size() / 2), p3p_Rt_devs.end());
	std::nth_element(p3p_rev_Rt_devs.begin(), p3p_rev_Rt_devs.begin() +
		(p3p_rev_Rt_devs.size() / 2), p3p_rev_Rt_devs.end());
	std::nth_element(pnp_Rt_devs.begin(), pnp_Rt_devs.begin() + (pnp_Rt_devs.size() / 2), pnp_Rt_devs.end());
	std::nth_element(fpnp_Rt_devs.begin(), fpnp_Rt_devs.begin() + (fpnp_Rt_devs.size() / 2), fpnp_Rt_devs.end());
	std::nth_element(p3p_reproj_RMSE.begin(), p3p_reproj_RMSE.begin() +
		(p3p_reproj_RMSE.size() / 2), p3p_reproj_RMSE.end());
	std::nth_element(p3p_rev_reproj_RMSE.begin(), p3p_rev_reproj_RMSE.begin() +
		(p3p_rev_reproj_RMSE.size() / 2), p3p_rev_reproj_RMSE.end());
	std::nth_element(pnp_reproj_RMSE.begin(), pnp_reproj_RMSE.begin() +
		(pnp_reproj_RMSE.size() / 2), pnp_reproj_RMSE.end());
	std::nth_element(fpnp_reproj_RMSE.begin(), fpnp_reproj_RMSE.begin() +
		(fpnp_reproj_RMSE.size() / 2), fpnp_reproj_RMSE.end());
	double f_p3p_median = p3p_Rt_devs[p3p_Rt_devs.size() / 2];
	double f_p3p_rev_median = p3p_rev_Rt_devs[p3p_rev_Rt_devs.size() / 2];
	double f_pnp_median = pnp_Rt_devs[pnp_Rt_devs.size() / 2];
	double f_fpnp_median = fpnp_Rt_devs[fpnp_Rt_devs.size() / 2];
	double f_p3p_mean = std::accumulate(p3p_Rt_devs.begin(), p3p_Rt_devs.end(), .0) / p3p_Rt_devs.size();
	double f_p3p_rev_mean = std::accumulate(p3p_rev_Rt_devs.begin(), p3p_rev_Rt_devs.end(), .0) / p3p_rev_Rt_devs.size();
	double f_pnp_mean = std::accumulate(pnp_Rt_devs.begin(), pnp_Rt_devs.end(), .0) / pnp_Rt_devs.size();
	double f_fpnp_mean = std::accumulate(fpnp_Rt_devs.begin(), fpnp_Rt_devs.end(), .0) / fpnp_Rt_devs.size();
	double f_p3p_reproj_median = p3p_reproj_RMSE[p3p_reproj_RMSE.size() / 2];
	double f_p3p_rev_reproj_median = p3p_rev_reproj_RMSE[p3p_rev_reproj_RMSE.size() / 2];
	double f_pnp_reproj_median = pnp_reproj_RMSE[pnp_reproj_RMSE.size() / 2];
	double f_fpnp_reproj_median = fpnp_reproj_RMSE[fpnp_reproj_RMSE.size() / 2];
	double f_p3p_reproj_mean = std::accumulate(p3p_reproj_RMSE.begin(),
		p3p_reproj_RMSE.end(), .0) / p3p_reproj_RMSE.size();
	double f_p3p_rev_reproj_mean = std::accumulate(p3p_rev_reproj_RMSE.begin(),
		p3p_rev_reproj_RMSE.end(), .0) / p3p_rev_reproj_RMSE.size();
	double f_pnp_reproj_mean = std::accumulate(pnp_reproj_RMSE.begin(),
		pnp_reproj_RMSE.end(), .0) / pnp_reproj_RMSE.size();
	double f_fpnp_reproj_mean = std::accumulate(fpnp_reproj_RMSE.begin(),
		fpnp_reproj_RMSE.end(), .0) / fpnp_reproj_RMSE.size();

	printf("P3P raw [Rt] error (Kneip et al.): %.3g mean (" PRIsize
		" samples), %.3g med\n", f_p3p_mean, p3p_Rt_devs.size(), f_p3p_median);
	printf("P3Pr raw [Rt] error (Kneip et al.): %.3g mean (" PRIsize
		" samples), %.3g med\n", f_p3p_rev_mean, p3p_rev_Rt_devs.size(), f_p3p_rev_median);
	printf("PNP raw [Rt] error (Hesch et al.): %.3g mean (" PRIsize
		" samples), %.3g med\n", f_pnp_mean, pnp_Rt_devs.size(), f_pnp_median);
	printf("PNP raw [Rt] error (Hesch et al.): %.3g mean (" PRIsize " samples), %.3g med "
		"(fast variant with Cayley singularity)\n", f_fpnp_mean, fpnp_Rt_devs.size(), f_fpnp_median);
	printf("P3P reprojection error (Kneip et al.): %.3g mean (" PRIsize
		" samples), %.3g med\n", f_p3p_reproj_mean, p3p_Rt_devs.size(), f_p3p_reproj_median);
	printf("P3Pr reprojection error (Kneip et al.): %.3g mean (" PRIsize
		" samples), %.3g med\n", f_p3p_rev_reproj_mean, p3p_rev_Rt_devs.size(), f_p3p_rev_reproj_median);
	printf("PNP reprojection error (Hesch et al.): %.3g mean (" PRIsize
		" samples), %.3g med\n", f_pnp_reproj_mean, pnp_Rt_devs.size(), f_pnp_reproj_median);
	printf("PNP reprojection error (Hesch et al.): %.3g mean (" PRIsize " samples), %.3g med "
		"(fast variant with Cayley singularity)\n", f_fpnp_reproj_mean, fpnp_Rt_devs.size(), f_fpnp_reproj_median);
}

void Homography_Test()
{
	printf("running homography test ...\n");

	const int p_point_num_list[] = {4, 10, 20, 50, 100, 200, 500, 1000};
	for(size_t n_opass = 0; n_opass < sizeof(p_point_num_list) / sizeof(p_point_num_list[0]); ++ n_opass) {
		const size_t n_point_num = p_point_num_list[n_opass];

		std::vector<double> hom_time_transfer, hom_time_symtransfer, hom_time_sampson, hom_time_reproj;
		std::vector<double> hom_RMSE_transfer, hom_RMSE_symtransfer, hom_RMSE_sampson, hom_RMSE_reproj;
		const int n_NLS_iters = 1; // assume all the methods do at least 1 without failing
		const bool b_use_eigenvectors = true;

		std::vector<Eigen::Matrix<double, 2, 1, Eigen::DontAlign> > corners_img, corners_world; // reuse between iterations
		for(int n_pass = 0; n_pass < ((n_opass < 4)? 1000 : 100); ++ n_pass) {
			Eigen::Vector3d v_position = Eigen::Vector3d::Random();
			v_position = v_position.normalized() * 1.5; // normalize the position so that the observed points would be inside the viewing frustum
			if(fabs(fabs(v_position(1)) - 1.5) < 1e-2) { // in case we would look straight up or down, generate another target
				-- n_pass;
				continue;
			}
			const Eigen::Vector3d v_target(0, 0, 0), v_up_canonical(0, 1, 0);
			Eigen::Vector3d v_dir = (v_target - v_position).normalized();
			Eigen::Vector3d v_up = (v_up_canonical - v_dir * v_dir.dot(v_up_canonical)).normalized();
			Eigen::Vector3d v_rt = v_up.cross(v_dir);
			Eigen::Matrix4d V = Eigen::Matrix4d::Identity(), T = Eigen::Matrix4d::Identity();
			V.topRows<1>().head<3>() = v_rt; // right
			V.bottomRows<3>().topRows<1>().head<3>() = v_up; // up
			V.bottomRows<2>().topRows<1>().head<3>() = v_dir; // fwd
			V.rightCols<1>().head<3>() = V.topLeftCorner<3, 3>() * -v_position; // translate by negative amount
			// assemble a random camera matrix

			double f = 1500, cx = 640, cy = 512; // 1280 x 1024 image
			Eigen::Matrix4d P;
			P << f, 0, cx, 0,
				 0, f, cy, 0,
				 0, 0,  3, -4, // transform z in [1, 2] to [-1, 1]
				 0, 0, -1, 0;
			// assemble a projection matrix

			Eigen::Matrix4d VPpx = P * V;
			// view-projection matrix, in pixels

			corners_img.clear();
			corners_world.clear(); // !!
			for(size_t n_point = 0; n_point < n_point_num; ++ n_point) {
				Eigen::Vector2d v_corner2 = Eigen::Vector2d::Random().normalized() * .5; // points on a disc with r = .5 (to maintain distance from the camera in [1, 2])
				Eigen::Vector4d v_corner4(v_corner2(0), 0, v_corner2(1), 1); // points are at plane z = 0

				Eigen::Vector4d v_image_VP4 = VPpx * v_corner4;
				Eigen::Vector3d v_image_VP = v_image_VP4.head<3>() / v_image_VP4(3);

				double f_z_lin = V.topLeftCorner<3, 3>().bottomRows<1>().dot(v_corner4.head<3>()) + V.rightCols<1>()(2);
				_ASSERTE(f_z_lin >= 1 && f_z_lin <= 2); // linear z is in [near, far] = [1, 2] by default
				double f_z = v_image_VP(2);
				_ASSERTE(f_z >= -1 && f_z <= 1); // nonlinear normalized z is in [-1, 1], -1 corresponding to the close clipping plane and +1 to the far clipping plane
				// normalized depth; depths in [1, 2] correspond to z [-1, 1]

				corners_world.push_back(v_corner2);
				corners_img.push_back(v_image_VP.head<2>());
			}

			Eigen::Matrix3d H_est;
#ifdef _DEBUG
			const int n_timing_passes = 1;
			const bool b_homog_est_verbose = false;
#else // _DEBUG
			const int n_timing_passes = 10;
			const bool b_homog_est_verbose = false;
#endif // _DEBUG
			{
				typedef CHomography::CReprojectionErrorObjective CObjective;
				CDeltaTimer dt;
				for(int j = 0; j < n_timing_passes; ++ j) {
					H_est = CHomography::t_Homography<CObjective>(corners_img,
						corners_world, true, b_use_eigenvectors, n_NLS_iters, b_homog_est_verbose); // turn verbose on
				}
				hom_time_reproj.push_back(dt.f_Time() / n_timing_passes);
				double f_mean = CHomography::f_Error<CObjective>(H_est, corners_img, corners_world, corners_world) / corners_world.size();
				double f_RMSE = CHomography::f_RMSError<CObjective>(H_est, corners_img, corners_world, corners_world);
				double f_Chi2 = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world, corners_world);
				double f_Chi2W = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world, corners_world,
					CObjective::_TyCovariance::Identity() * 10);
				hom_RMSE_reproj.push_back(f_RMSE);
			}
			{
				typedef CHomography::CTransferErrorObjective CObjective;
				Eigen::Matrix3d H_est;
				CDeltaTimer dt;
				for(int j = 0; j < n_timing_passes; ++ j) {
					H_est = CHomography::t_Homography<CObjective>(corners_img,
						corners_world, true, b_use_eigenvectors, n_NLS_iters, b_homog_est_verbose); // turn verbose on
				}
				hom_time_transfer.push_back(dt.f_Time() / n_timing_passes);
				double f_mean = CHomography::f_Error<CObjective>(H_est, corners_img, corners_world) / corners_world.size();
				double f_RMSE = CHomography::f_RMSError<CObjective>(H_est, corners_img, corners_world);
				double f_Chi2 = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world);
				double f_Chi2W = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world,
					CObjective::_TyCovariance::Identity() * 10);
				hom_RMSE_transfer.push_back(f_RMSE);
			}
			{
				typedef CHomography::CSymTransferErrorObjective CObjective;
				Eigen::Matrix3d H_est;
				CDeltaTimer dt;
				for(int j = 0; j < n_timing_passes; ++ j) {
					H_est = CHomography::t_Homography<CObjective>(corners_img,
						corners_world, true, b_use_eigenvectors, n_NLS_iters, b_homog_est_verbose); // turn verbose on
				}
				hom_time_symtransfer.push_back(dt.f_Time() / n_timing_passes);
				double f_mean = CHomography::f_Error<CObjective>(H_est, corners_img, corners_world) / corners_world.size();
				double f_RMSE = CHomography::f_RMSError<CObjective>(H_est, corners_img, corners_world);
				double f_Chi2 = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world);
				double f_Chi2W = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world,
					CObjective::_TyCovariance::Identity() * 10);
				hom_RMSE_symtransfer.push_back(f_RMSE);
			}
			{
				typedef CHomography::CSampsonErrorObjective CObjective;
				Eigen::Matrix3d H_est;
				CDeltaTimer dt;
				for(int j = 0; j < n_timing_passes; ++ j) {
					H_est = CHomography::t_Homography<CObjective>(corners_img,
						corners_world, true, b_use_eigenvectors, n_NLS_iters, b_homog_est_verbose); // turn verbose on
				}
				hom_time_sampson.push_back(dt.f_Time() / n_timing_passes);
				double f_mean = CHomography::f_Error<CObjective>(H_est, corners_img, corners_world) / corners_world.size();
				double f_RMSE = CHomography::f_RMSError<CObjective>(H_est, corners_img, corners_world);
				double f_Chi2 = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world);
				double f_Chi2W = CHomography::f_Chi2Error<CObjective>(H_est, corners_img, corners_world,
					CObjective::_TyCovariance::Identity() * 10);
				hom_RMSE_sampson.push_back(f_RMSE);
			}

			printf(".");
		}

		printf("\nfor " PRIsize " points, DLT using %s and %d NLS iters:\n", n_point_num,
			(b_use_eigenvectors)? "eigen-vectors" : "singular vectors", n_NLS_iters);
		std::sort(hom_time_transfer.begin(), hom_time_transfer.end());
		std::sort(hom_RMSE_transfer.begin(), hom_RMSE_transfer.end());
		std::sort(hom_time_symtransfer.begin(), hom_time_symtransfer.end());
		std::sort(hom_RMSE_symtransfer.begin(), hom_RMSE_symtransfer.end());
		std::sort(hom_time_sampson.begin(), hom_time_sampson.end());
		std::sort(hom_RMSE_sampson.begin(), hom_RMSE_sampson.end());
		std::sort(hom_time_reproj.begin(), hom_time_reproj.end());
		std::sort(hom_RMSE_reproj.begin(), hom_RMSE_reproj.end());

		printf("transfer error homography %f msec avg, %f msec med, %g RMSE avg %g RMSE med %g RMSE max\n",
			1000 * std::accumulate(hom_time_transfer.begin(), hom_time_transfer.end(), .0) / hom_time_transfer.size(),
			1000 * hom_time_transfer[hom_time_transfer.size() / 2],
			std::accumulate(hom_RMSE_transfer.begin(), hom_RMSE_transfer.end(), .0) / hom_RMSE_transfer.size(),
			hom_RMSE_transfer[hom_RMSE_transfer.size() / 2], hom_RMSE_transfer.back());
		printf("symtransfer error homography %f msec avg, %f msec med, %g RMSE avg %g RMSE med %g RMSE max\n",
			1000 * std::accumulate(hom_time_symtransfer.begin(), hom_time_symtransfer.end(), .0) / hom_time_symtransfer.size(),
			1000 * hom_time_symtransfer[hom_time_symtransfer.size() / 2],
			std::accumulate(hom_RMSE_symtransfer.begin(), hom_RMSE_symtransfer.end(), .0) / hom_RMSE_symtransfer.size(),
			hom_RMSE_symtransfer[hom_RMSE_symtransfer.size() / 2], hom_RMSE_symtransfer.back());
		printf("sampson error homography %f msec avg, %f msec med, %g RMSE avg %g RMSE med %g RMSE max\n",
			1000 * std::accumulate(hom_time_sampson.begin(), hom_time_sampson.end(), .0) / hom_time_sampson.size(),
			1000 * hom_time_sampson[hom_time_sampson.size() / 2],
			std::accumulate(hom_RMSE_sampson.begin(), hom_RMSE_sampson.end(), .0) / hom_RMSE_sampson.size(),
			hom_RMSE_sampson[hom_RMSE_sampson.size() / 2], hom_RMSE_sampson.back());
		printf("reproj error homography %f msec avg, %f msec med, %g RMSE avg %g RMSE med %g RMSE max\n",
			1000 * std::accumulate(hom_time_reproj.begin(), hom_time_reproj.end(), .0) / hom_time_reproj.size(),
			1000 * hom_time_reproj[hom_time_reproj.size() / 2],
			std::accumulate(hom_RMSE_reproj.begin(), hom_RMSE_reproj.end(), .0) / hom_RMSE_reproj.size(),
			hom_RMSE_reproj[hom_RMSE_reproj.size() / 2], hom_RMSE_reproj.back());
	}
}

/**
 *	@page geomtoolsexample Geometric Tools Example
 *
 *	This example showcases the use of geometric tools available in SLAM++.
 *	It is in \ref geometry_example/Main.cpp.
 *
 */

/*
 *	end-of-file
 */
