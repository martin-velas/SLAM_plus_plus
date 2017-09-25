/*
								+----------------------------------+
								|                                  |
								|         ***  COVADO  ***         |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|            Covado.cpp            |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam_app/Covado.cpp
 *	@brief covariance conversion utility
 *	@author -tHE SWINe-
 *	@date 2013-06-14
 *
 *	This is a simple "utility" that we used to convert different covariance
 *	representation in datasets, e.g. to convert covariance of quaternion measurements
 *	to covariance of axis-angle measurements. Note that this conversion is not
 *	fully automated and requires editing this source code (it is not hard, though).
 *	Also, the resulting covariances are no longer diagonal matrices.
 */

/**
 *	@def __RUN_COVADO
 *	@brief define if in need of covariance conversion (hijacks the application)
 */
//#define __RUN_COVADO

/**
 *	@def __RUN_COVARIANCE_CONVERSION
 *	@brief define if in need of covariance conversion, otherwise runs cholesky
 */
//#define __RUN_COVARIANCE_CONVERSION

/**
 *	@def __COVADO_CONVERT_QUATCOV_TO_AXISANGLECOV
 *	@brief chooses quaternion covariance to axis-angle covariance (otherwise quaternion to RPY)
 */
//#define __COVADO_CONVERT_QUATCOV_TO_AXISANGLECOV

#ifdef __RUN_COVADO

#include "slam_app/Config.h"
#include "slam_app/SE2_Types.h"
#include "slam_app/SE3_Types.h"
#include "slam_app/BA_Types.h"

#include <iostream> // want to print Eigen matrices in easy way

/**
 *	@brief covariance conversion for dataset operation
 */
class CCovado {
public:
	/**
	 *	@brief default constructor; runs COVADO
	 */
	CCovado()
	{
#ifndef __RUN_COVARIANCE_CONVERSION
		printf("running cholesky\n");

/*

         100  2.15114e-05 -3.46442e-05    0.0101222    -0.128098    -0.342183
 2.15114e-05          100 -2.66435e-05  -0.00611334    -0.160674    -0.101645
-3.46442e-05 -2.66435e-05          100   -0.0230755     0.354741     0.117736
   0.0101222  -0.00611334   -0.0230755      156.068  -0.00913407    0.0357455
   -0.128098    -0.160674     0.354741  -0.00913407      2500.13     0.011439
   -0.342183    -0.101645     0.117736    0.0357455     0.011439      2499.07

          10            0            0            0            0            0
           0           10            0            0            0            0
           0            0           10            0            0            0
           0            0            0      12.4927            0            0
           0            0            0 -0.000731152      50.0013            0
           0            0            0   0.00286131  0.000228816      49.9907

10 0 0 0 0 0 10 0 0 0 0 10 0 0 0 12.4927 -0.000731152 0.00286131 50.0013 0.000228816 49.9907

*/ // YPR

		//const double p_u[] = {100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 2499.07, 0.011439, 0.0357455, 2500.13, -0.00913407, 156.068};
		//const double p_u[] = {100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 2498.43, 0.000198889, 0.0397551, 2498.43, -0.0116887, 156.089};
		const double p_u[] = {100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 156.068, -0.00913407, 0.0357455, 2500.13, 0.011439, 2499.07};

		Eigen::Matrix<double, 6, 6> information;
		information <<
			p_u[0],  p_u[1],  p_u[2],  p_u[3],  p_u[4],  p_u[5],
			p_u[1],  p_u[6],  p_u[7],  p_u[8],  p_u[9], p_u[10],
			p_u[2],  p_u[7], p_u[11], p_u[12], p_u[13], p_u[14],
			p_u[3],  p_u[8], p_u[12], p_u[15], p_u[16], p_u[17],
			p_u[4],  p_u[9], p_u[13], p_u[16], p_u[18], p_u[19],
			p_u[5], p_u[10], p_u[14], p_u[17], p_u[19], p_u[20];

		Eigen::LLT<Eigen::Matrix<double, 6, 6>, Eigen::Lower> chol_information(information);

		Eigen::Matrix<double, 6, 6> information_sqrt = chol_information.matrixL();

		std::cout << "information" << std::endl << information << std::endl;
		std::cout << "sqrt information" << std::endl << information_sqrt << std::endl;

		exit(-1);
#else // __RUN_COVARIANCE_CONVERSION
		printf("running covado\n");

		const double p_u[] = {100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 10000, 0, 0, 10000, 0, 625};
		//		      100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 10000, 0, 0, 10000, 0, 625

		Eigen::Matrix<double, 6, 6> information_g2o;
		information_g2o <<
			p_u[0],  p_u[1],  p_u[2],  p_u[3],  p_u[4],  p_u[5],
			p_u[1],  p_u[6],  p_u[7],  p_u[8],  p_u[9], p_u[10],
			p_u[2],  p_u[7], p_u[11], p_u[12], p_u[13], p_u[14],
			p_u[3],  p_u[8], p_u[12], p_u[15], p_u[16], p_u[17],
			p_u[4],  p_u[9], p_u[13], p_u[16], p_u[18], p_u[19],
			p_u[5], p_u[10], p_u[14], p_u[17], p_u[19], p_u[20];


		Eigen::Matrix<double, 6, 6> cov_g2o = information_g2o.inverse();

		std::cout << "g2o inf" << std::endl << information_g2o << std::endl;
		std::cout << "g2o cov" << std::endl << cov_g2o << std::endl;

		std::vector<Eigen::Matrix<double, 6, 1> > samples;
		for(int i = 0; i < 100000000; ++ i) {
			Eigen::Matrix<double, 6, 1> v;
			v << f_Rand(), f_Rand(), f_Rand(), f_Rand(), f_Rand(), f_Rand();
			// make uniform distribution 6D vector

			samples.push_back(v);
		}

		Eigen::Matrix<double, 6, 6> covariance;
		{
			Eigen::Matrix<double, 6, 1> v_average;
			v_average.setZero();
			for(size_t i = 0, n = samples.size(); i < n; ++ i)
				v_average += samples[i];
			v_average /= samples.size();
			// calculate mean

			std::cout << "mean" << std::endl << v_average << std::endl;

			covariance.setZero();
			for(size_t i = 0, n = samples.size(); i < n; ++ i)
				covariance += (samples[i] - v_average) * (samples[i] - v_average).transpose();
			covariance /= (samples.size() - 1);
		}
		// calculate covariance of the samples

		std::cout << "cov" << std::endl << covariance << std::endl;

		Eigen::LLT<Eigen::Matrix<double, 6, 6>, Eigen::Lower> cholCov(covariance.inverse());
		Eigen::LLT<Eigen::Matrix<double, 6, 6>, Eigen::Lower> cholg2oCov(cov_g2o);
		Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>(cholCov.matrixL()) * Eigen::Matrix<double, 6, 6>(cholg2oCov.matrixL());
		// calculate transform

		for(size_t i = 0, n = samples.size(); i < n; ++ i)
			samples[i] = transform * samples[i];

		{
			Eigen::Matrix<double, 6, 1> v_average;
			v_average.setZero();
			for(size_t i = 0, n = samples.size(); i < n; ++ i)
				v_average += samples[i];
			v_average /= samples.size();
			// calculate mean

			std::cout << "mean" << std::endl << v_average << std::endl;

			Eigen::Matrix<double, 6, 6> covariance2;
			covariance2.setZero();
			for(size_t i = 0, n = samples.size(); i < n; ++ i)
				covariance2 += (samples[i] - v_average) * (samples[i] - v_average).transpose();
			covariance2 /= (samples.size() - 1);
			// calculate covariance of the samples

			std::cout << "cov after transform" << std::endl << covariance2 << std::endl;
		}

#ifdef __COVADO_CONVERT_QUATCOV_TO_AXISANGLECOV
		for(size_t i = 0, n = samples.size(); i < n; ++ i) {
			Eigen::Vector3d v_pos = samples[i].segment<3>(0);
			Eigen::Vector3d v_rot = samples[i].segment<3>(3);

			Eigen::Quaterniond v_quat;
			v_quat.w() = sqrt(1 - v_rot.norm() * v_rot.norm()); // sin2 + cos2 = 1
			v_quat.vec() = v_rot;
			// transform the rotation to a (normalized) quaternion

			Eigen::Vector3d v_axis_angle;
			C3DJacobians::Quat_to_AxisAngle(v_quat, v_axis_angle);

			samples[i].segment<3>(3) = v_axis_angle;
			// the rotation part changes
		}
#else // __COVADO_CONVERT_QUATCOV_TO_AXISANGLECOV
		for(size_t i = 0, n = samples.size(); i < n; ++ i) {
			Eigen::Vector3d v_pos = samples[i].segment<3>(0);
			Eigen::Vector3d v_rot = samples[i].segment<3>(3);

			Eigen::Quaterniond v_quat;
			v_quat.w() = sqrt(1 - v_rot.norm() * v_rot.norm()); // sin2 + cos2 = 1
			v_quat.vec() = v_rot;
			// transform the rotation to a (normalized) quaternion

			Eigen::Vector3d v_RPY;
			double q0 = v_quat.w(), q1 = v_quat.x(), q2 = v_quat.y(), q3 = v_quat.z(); // wikipedia notation
#if 0
			v_RPY.x() = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
			v_RPY.y() = asin(2 * (q0 * q2 - q3 * q1));
			v_RPY.z() = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
			// RPY
#else
			v_RPY.z() = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
			v_RPY.y() = asin(2 * (q0 * q2 - q3 * q1));
			v_RPY.x() = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
			// YPR
#endif
			// transform that quaternion to roll pitch yaw (blame [http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles])

			samples[i].segment<3>(3) = v_RPY;
			// the rotation part changes
		}
#endif // __COVADO_CONVERT_QUATCOV_TO_AXISANGLECOV

		{
			Eigen::Matrix<double, 6, 1> v_average;
			v_average.setZero();
			for(size_t i = 0, n = samples.size(); i < n; ++ i)
				v_average += samples[i];
			v_average /= samples.size();
			// calculate mean

			std::cout << "mean" << std::endl << v_average << std::endl;

			Eigen::Matrix<double, 6, 6> covariance2;
			covariance2.setZero();
			for(size_t i = 0, n = samples.size(); i < n; ++ i)
				covariance2 += (samples[i] - v_average) * (samples[i] - v_average).transpose();
			covariance2 /= (samples.size() - 1);
			// calculate covariance of the samples

			std::cout << "cov after space conversion" << std::endl << covariance2 << std::endl;

			std::cout << "information after space conversion" << std::endl << covariance2.inverse() << std::endl;
		}

		exit(-1);
#endif // __RUN_COVARIANCE_CONVERSION
	}

	static double f_Rand() // uniform distribution in [-1, 1]
	{
		return double(rand()) / RAND_MAX * 2 - 1;
	}
} covado;

/*

running covado
g2o inf
  100     0     0     0     0     0
    0   100     0     0     0     0
    0     0   100     0     0     0
    0     0     0 10000     0     0
    0     0     0     0 10000     0
    0     0     0     0     0   625
g2o cov
  0.01      0      0      0      0      0
     0   0.01      0      0      0      0
     0      0   0.01      0      0      0
     0      0      0 0.0001      0      0
     0      0      0      0 0.0001      0
     0      0      0      0      0 0.0016
mean
6.12858e-05
2.58306e-05
-4.77033e-05
0.000116611
3.85139e-05
-1.02417e-05
cov
     0.33334   4.8468e-05  5.63561e-06 -2.55678e-05 -9.45738e-06  1.80132e-05
  4.8468e-05     0.333349 -1.02368e-05 -7.46834e-06  -1.1752e-05 -1.08595e-05
 5.63561e-06 -1.02368e-05     0.333318  8.86376e-06  2.63536e-05 -4.10111e-05
-2.55678e-05 -7.46834e-06  8.86376e-06     0.333302 -1.49279e-06 -2.83143e-05
-9.45738e-06  -1.1752e-05  2.63536e-05 -1.49279e-06     0.333311  8.28715e-06
 1.80132e-05 -1.08595e-05 -4.10111e-05 -2.83143e-05  8.28715e-06     0.333277
mean
1.06149e-05
4.47234e-06
-8.26269e-06
2.02099e-06
6.68224e-07
-7.10914e-07
cov after transform
        0.01  2.41864e-11 -1.14956e-10  6.90354e-07  2.55384e-07  -3.2426e-07
 2.41864e-11         0.01  -3.9222e-12  2.01706e-07  3.17323e-07  1.95496e-07
-1.14956e-10  -3.9222e-12         0.01 -2.39243e-07 -7.11627e-07  7.38345e-07
 6.90354e-07  2.01706e-07 -2.39243e-07       0.0001  4.03325e-11 -2.55194e-08
 2.55384e-07  3.17323e-07 -7.11627e-07  4.03325e-11       0.0001  7.40846e-09
 -3.2426e-07  1.95496e-07  7.38345e-07 -2.55194e-08  7.40846e-09       0.0016
mean
1.06149e-05
4.47234e-06
-8.26269e-06
4.04273e-06
1.33738e-06
-1.42425e-06
cov after space conversion
        0.01  2.41864e-11 -1.14956e-10  1.38107e-06  5.10993e-07 -6.48934e-07
 2.41864e-11         0.01  -3.9222e-12  4.03601e-07  6.34849e-07  3.91598e-07
-1.14956e-10  -3.9222e-12         0.01 -4.78611e-07 -1.42374e-06  1.47798e-06
 1.38107e-06  4.03601e-07 -4.78611e-07  0.000400251  1.31997e-10 -1.02086e-07
 5.10993e-07  6.34849e-07 -1.42374e-06  1.31997e-10  0.000400251  2.97538e-08
-6.48934e-07  3.91598e-07  1.47798e-06 -1.02086e-07  2.97538e-08   0.00640658
information after space conversion
         100  2.13929e-05  -3.5038e-05    -0.345049    -0.127669    0.0101243
 2.13929e-05          100 -2.64658e-05    -0.100839    -0.158612  -0.00611329
 -3.5038e-05 -2.64658e-05          100     0.119572     0.355714   -0.0230695
   -0.345049    -0.100839     0.119572      2498.43  0.000198889    0.0397551
   -0.127669    -0.158612     0.355714  0.000198889      2498.43   -0.0116887
   0.0101243  -0.00611329   -0.0230695    0.0397551   -0.0116887      156.089

so the corresponding information matrix for axis-angle representations should be:
100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 2498.43 0.000198889 0.0397551 2498.43 -0.0116887 156.089

running cholesky
information
        100           0           0           0           0           0
          0         100           0           0           0           0
          0           0         100           0           0           0
          0           0           0     2498.43 0.000198889   0.0397551
          0           0           0 0.000198889     2498.43  -0.0116887
          0           0           0   0.0397551  -0.0116887     156.089
sqrt information
          10            0            0            0            0            0
           0           10            0            0            0            0
           0            0           10            0            0            0
           0            0            0      49.9843            0            0
           0            0            0  3.97903e-06      49.9843            0
           0            0            0  0.000795352 -0.000233848      12.4936

so the corresponding information matrix for axis-angle square-rooted representations should be:
10 0 0 0 0 0 10 0 0 0 0 10 0 0 0 49.9843 3.97903e-06 0.000795352 49.9843 -0.000233848 12.4936

*/

/*

running covado
g2o inf
  100     0     0     0     0     0
    0   100     0     0     0     0
    0     0   100     0     0     0
    0     0     0 10000     0     0
    0     0     0     0 10000     0
    0     0     0     0     0   625
g2o cov
  0.01      0      0      0      0      0
     0   0.01      0      0      0      0
     0      0   0.01      0      0      0
     0      0      0 0.0001      0      0
     0      0      0      0 0.0001      0
     0      0      0      0      0 0.0016
mean
6.12858e-05
2.58306e-05
-4.77033e-05
0.000116611
3.85139e-05
-1.02417e-05
cov
     0.33334   4.8468e-05  5.63561e-06 -2.55678e-05 -9.45738e-06  1.80132e-05
  4.8468e-05     0.333349 -1.02368e-05 -7.46834e-06  -1.1752e-05 -1.08595e-05
 5.63561e-06 -1.02368e-05     0.333318  8.86376e-06  2.63536e-05 -4.10111e-05
-2.55678e-05 -7.46834e-06  8.86376e-06     0.333302 -1.49279e-06 -2.83143e-05
-9.45738e-06  -1.1752e-05  2.63536e-05 -1.49279e-06     0.333311  8.28715e-06
 1.80132e-05 -1.08595e-05 -4.10111e-05 -2.83143e-05  8.28715e-06     0.333277
mean
1.06149e-05
4.47234e-06
-8.26269e-06
2.02099e-06
6.68224e-07
-7.10914e-07
cov after transform
        0.01  2.41864e-11 -1.14956e-10  6.90354e-07  2.55384e-07  -3.2426e-07
 2.41864e-11         0.01  -3.9222e-12  2.01706e-07  3.17323e-07  1.95496e-07
-1.14956e-10  -3.9222e-12         0.01 -2.39243e-07 -7.11627e-07  7.38345e-07
 6.90354e-07  2.01706e-07 -2.39243e-07       0.0001  4.03325e-11 -2.55194e-08
 2.55384e-07  3.17323e-07 -7.11627e-07  4.03325e-11       0.0001  7.40846e-09
 -3.2426e-07  1.95496e-07  7.38345e-07 -2.55194e-08  7.40846e-09       0.0016
mean
1.06149e-05
4.47234e-06
-8.26269e-06
4.05567e-06
1.385e-06
-1.4247e-06
cov after space conversion
        0.01  2.41864e-11 -1.14956e-10  1.36925e-06  5.12359e-07 -6.48861e-07
 2.41864e-11         0.01  -3.9222e-12  4.06722e-07  6.42664e-07  3.91656e-07
-1.14956e-10  -3.9222e-12         0.01 -4.71132e-07 -1.41889e-06  1.47858e-06
 1.36925e-06  4.06722e-07 -4.71132e-07  0.000400148 -1.66802e-09 -9.17922e-08
 5.12359e-07  6.42664e-07 -1.41889e-06 -1.66802e-09   0.00039998  2.31919e-08
-6.48861e-07  3.91656e-07  1.47858e-06 -9.17922e-08  2.31919e-08   0.00640748
information after space conversion
         100  2.15114e-05 -3.46442e-05    -0.342183    -0.128098    0.0101222
 2.15114e-05          100 -2.66435e-05    -0.101645    -0.160674  -0.00611334
-3.46442e-05 -2.66435e-05          100     0.117736     0.354741   -0.0230755
   -0.342183    -0.101645     0.117736      2499.07     0.011439    0.0357455
   -0.128098    -0.160674     0.354741     0.011439      2500.13  -0.00913407
   0.0101222  -0.00611334   -0.0230755    0.0357455  -0.00913407      156.068

so the corresponding information matrix for RPY representations should be:
100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 2499.07 0.011439 0.0357455 2500.13 -0.00913407 156.068

running cholesky
information
        100           0           0           0           0           0
          0         100           0           0           0           0
          0           0         100           0           0           0
          0           0           0     2499.07    0.011439   0.0357455
          0           0           0    0.011439     2500.13 -0.00913407
          0           0           0   0.0357455 -0.00913407     156.068
sqrt information
         10           0           0           0           0           0
          0          10           0           0           0           0
          0           0          10           0           0           0
          0           0           0     49.9907           0           0
          0           0           0 0.000228823     50.0013           0
          0           0           0 0.000715043 -0.00018268     12.4927


so the corresponding information matrix for RPY square-rooted representations should be:
10 0 0 0 0 0 10 0 0 0 0 10 0 0 0 49.9907 0.000228823 0.000715043 50.0013 -0.00018268 12.4927

*/

#endif // __RUN_COVADO

//#define __DUMP_RSS2013_PRESENTATION_ANIMATION_DATA
#ifdef __DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

#include "slam/Config.h"
class CRSSImagesCompositor {
protected:
	TBmp *p_lambda_add;
	TBmp *p_lambda_hot;
	TBmp *p_lambda_cold;
	TBmp *p_chol_hot;
	TBmp *p_chol_cold;
	TBmp *p_chol_add;
	TBmp *p_rhs_add;
	TBmp *p_rhs_hot;
	TBmp *p_rhs_cold;
	TBmp *p_lhs_hot;
	TBmp *p_lhs_cold;

public:
	CRSSImagesCompositor()
		:p_lambda_add(0), p_lambda_hot(0), p_lambda_cold(0), p_chol_add(0),
		p_chol_hot(0), p_chol_cold(0), p_rhs_hot(0), p_lhs_hot(0),
		p_rhs_cold(0), p_lhs_cold(0), p_rhs_add(0)
	{
		int n_image_num = 17;
		int n_padding = 10; // px

		LoadImages(n_image_num);

		int n_mat_w = p_lambda_add->n_width;
		int n_mat_h = p_lambda_add->n_height;
		int n_vec_w = p_rhs_hot->n_width;
		int n_frame_width = 2 * n_mat_w + n_vec_w + 1 * n_padding + n_mat_w / 2;
		int n_frame_height = n_mat_w;
		TBmp *p_frame = TBmp::p_Alloc(n_frame_width, n_frame_height);

		TBmp *p_prev_chol = TBmp::p_Alloc(1, 1);
		p_prev_chol->Clear(-1);
		TBmp *p_prev_dx = TBmp::p_Alloc(1, 1);
		p_prev_dx->Clear(-1);

#if 0
		for(int i = 1; i <= n_image_num; ++ i) {
			LoadImages(i);
			// load all the images

			for(int n_pass = 0; n_pass < 4; ++ n_pass) {
				{
					p_frame->Clear(0xffffffffU);
					// clear the frame

					Blit(p_frame, (n_pass == 0)? p_lambda_add : (n_pass == 2)? p_lambda_hot : p_lambda_cold, 0, 0);
					// the first is lambda

					Blit(p_frame, (n_pass == 0)? p_rhs_add : (n_pass == 2)? p_rhs_hot : p_rhs_cold,
						/*2 **/ (n_mat_w + n_padding), 0);
					// rhs

					//if(n_pass == 1 /*|| n_pass == 2*/ || (i == n_image_num && n_pass > 1)) {
						Blit(p_frame, (n_pass == 0)? p_prev_chol : (n_pass == 1)? p_chol_hot : p_chol_cold,
							n_mat_w + n_vec_w + n_padding + n_mat_w / 2, 0);
						//Blit(p_frame, (n_pass == 0)? p_prev_chol : /*(n_pass == 1)? p_chol_hot :*/
						//	p_chol_cold, n_mat_w + n_vec_w + n_padding + n_mat_w / 2, 0);
						// then cholesky

						//Blit(p_frame, (n_pass == 0)? p_prev_dx : (n_pass == 1)? p_lhs_hot : p_lhs_cold,
						//	n_mat_w + n_padding, n_mat_w + n_padding); // no dX
						// lhs
					//}
				}

				char p_s_filename[256];
				sprintf(p_s_filename, "rss2013/anim_%05d_%d.tga", i, n_pass);
				CTgaCodec::Save_TGA(p_s_filename, *p_frame, true, true);
				// save as an image
			}

			p_prev_chol->Delete();
			p_prev_chol = p_chol_cold->p_Clone();
			p_prev_dx->Delete();
			p_prev_dx = p_lhs_cold->p_Clone();
		}
		// make RSS-style very flashing animation
#else // 0
		for(int i = 1; i <= n_image_num; ++ i) {
			LoadImages(i);
			// load all the images

			for(int n_pass = 0; n_pass < 2; ++ n_pass) {
				p_frame->Clear(0xffffffffU);
				// clear the frame

				Blit(p_frame, p_lambda_add, 0, 0);
				// the first is lambda

				Blit(p_frame, p_rhs_add,
					/*2 **/ (n_mat_w + n_padding), 0);
				// rhs

				//Blit(p_frame, p_chol_add, n_mat_w + n_vec_w + n_padding + n_mat_w / 2, 0);
				if(n_pass)
					Blit(p_frame, p_chol_cold, n_mat_w + n_vec_w + n_padding + n_mat_w / 2, 0);

				char p_s_filename[256];
				sprintf(p_s_filename, "rss2013/anim_%05d_%d.tga", i, n_pass);
				CTgaCodec::Save_TGA(p_s_filename, *p_frame, true, true);
				// save as an image
			}

			p_prev_chol->Delete();
			p_prev_chol = p_chol_cold->p_Clone();
			p_prev_dx->Delete();
			p_prev_dx = p_lhs_cold->p_Clone();
		}
		// make IAV-style incremental animation
#endif // 0

		p_frame->Delete();
		p_prev_chol->Delete();
		p_prev_dx->Delete();

		int n_anim_frame_num = 4 * n_image_num; // number of frames in the sequence
		float n_anim_FPS = 25;
		float f_anim_wait_start = 5;//10;
		float f_anim_wait_end = 10; // a bit over-time but at least it won't finish in the slide
		float p_anim_wait_1stloop_stages[] = {10, 6, 7, 5};
		float f_anim_wait_1stloop_stages_sum =
			p_anim_wait_1stloop_stages[0] + p_anim_wait_1stloop_stages[1] +
			p_anim_wait_1stloop_stages[2] + p_anim_wait_1stloop_stages[3];
		float f_anim_length = 55;//60; // seconds

		int n_anim_wait_start_frame_num = int(ceil(n_anim_FPS * f_anim_wait_start));
		int n_anim_wait_end_frame_num = int(ceil(n_anim_FPS * f_anim_wait_end));
		int p_anim_wait_1stloop_stages_frame_num[4];
		for(int i = 0; i < 4; ++ i)
			p_anim_wait_1stloop_stages_frame_num[i] = int(ceil(n_anim_FPS * p_anim_wait_1stloop_stages[i]));
		int n_anim_wait_next_stages_frame_num = int(ceil(n_anim_FPS * (f_anim_length -
			f_anim_wait_start - f_anim_wait_end - f_anim_wait_1stloop_stages_sum) / (n_anim_frame_num - 5))); // except the first ones and the last one

		int n_wait_slide_id = 10;

		FILE *p_fw;
		if((p_fw = fopen("rss2013\\list.txt", "w"))) {
			{
				int i = 0/*n_image_num*/, n_pass = 3;
				char p_s_filename[256];
				sprintf(p_s_filename, "anim_%05d_%d.png", i, n_pass); // start with the /*last*/ zero-th, manually edited frame

				for(int j = 0; j < n_anim_wait_start_frame_num; ++ j)
					fprintf(p_fw, "%s\n", p_s_filename);
			}
			// wait at the beginning

			for(int i = 1; i <= n_image_num; ++ i) {
				for(int n_pass = 0; n_pass < ((i == n_image_num)? 3 : 4); ++ n_pass) { // the last one displayed explicitly
					char p_s_filename[256];
					sprintf(p_s_filename, "anim_%05d_%d.png", i, n_pass);
					int n_frame_num = (i == n_wait_slide_id)? p_anim_wait_1stloop_stages_frame_num[n_pass] :
						n_anim_wait_next_stages_frame_num;
					for(int j = 0; j < n_frame_num; ++ j)
						fprintf(p_fw, "%s\n", p_s_filename);
				}
			}
			// put all the other animation frames

			{
				int i = n_image_num, n_pass = 3;
				char p_s_filename[256];
				sprintf(p_s_filename, "anim_%05d_%d.png", i, n_pass); // end with the last frame

				for(int j = 0; j < n_anim_wait_end_frame_num; ++ j)
					fprintf(p_fw, "%s\n", p_s_filename);
			}
			// wait at the end

			fclose(p_fw);
		}

		if((p_fw = fopen("rss2013\\list2.txt", "w"))) {
			{
				for(int j = 0; j < n_anim_wait_start_frame_num; ++ j)
					fprintf(p_fw, "%s\n", "arrow_blank.png"); // no arrow
			}
			// wait at the beginning

			for(int i = 1; i <= n_image_num; ++ i) {
				for(int n_pass = 0; n_pass < ((i == n_image_num)? 3 : 4); ++ n_pass) { // the last one displayed explicitly
					char p_s_filename[256];
					sprintf(p_s_filename, "arrow_step%d.png", n_pass);
					int n_frame_num = (i == n_wait_slide_id)? p_anim_wait_1stloop_stages_frame_num[n_pass] :
						n_anim_wait_next_stages_frame_num;
					for(int j = 0; j < n_frame_num; ++ j)
						fprintf(p_fw, "%s\n", p_s_filename);
				}
			}
			// put all the other animation frames

			{
				for(int j = 0; j < n_anim_wait_start_frame_num; ++ j)
					fprintf(p_fw, "%s\n", "arrow_step3.png"); // no arrow
			}
			// wait at the end

			fclose(p_fw);
		}

		if((p_fw = fopen("rss2013\\list3.txt", "w"))) {
			{
				for(int j = 0; j < n_anim_wait_start_frame_num; ++ j)
					fprintf(p_fw, "%s\n", "arrow_blank.png"); // no arrow
			}
			// wait at the beginning

			for(int i = 1; i <= n_image_num; ++ i) {
				for(int n_pass = 0; n_pass < ((i == n_image_num)? 3 : 4); ++ n_pass) { // the last one displayed explicitly
					char p_s_filename[256];
					sprintf(p_s_filename, "arrow_step%d.png", n_pass);
					int n_frame_num = (i == n_wait_slide_id)? p_anim_wait_1stloop_stages_frame_num[n_pass] :
						n_anim_wait_next_stages_frame_num;
					for(int j = 0; j < n_frame_num; ++ j)
						fprintf(p_fw, "%s\n", (i != n_wait_slide_id)? "arrow_blank.png" : p_s_filename);
				}
			}
			// put all the other animation frames

			{
				for(int j = 0; j < n_anim_wait_start_frame_num; ++ j)
					fprintf(p_fw, "%s\n", "arrow_step3.png"); // no arrow
			}
			// wait at the end

			fclose(p_fw);
		}

		exit(-3);
	}

	void Blit(TBmp *p_dest, const TBmp *p_src, int dx, int dy)
	{
		const int dw = p_dest->n_width, dh = p_dest->n_height;
		for(int y = 0, w = p_src->n_width, h = p_src->n_height; y < h; ++ y) {
			for(int x = 0; x < w; ++ x) {
				if(x + dx >= 0 && x + dx < dw && y + dy >= 0 && y + dy < dh)
					p_dest->p_buffer[x + dx + (y + dy) * dw] = p_src->p_buffer[x + y * w];
			}
		}
	}

	~CRSSImagesCompositor()
	{
		DeleteImages(); // !!
	}

	void LoadImages(int n)
	{
		DeleteImages(); // !!

		char p_s_filename[256];
		sprintf(p_s_filename, "rss2013/%05d_0_lambda.tga", n);
		p_lambda_cold = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_1_lambda2.tga", n);
		p_lambda_add = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_2_lambda3.tga", n);
		p_lambda_hot = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_a_Lnoord_red.tga", n);
		p_chol_hot = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_b_Lnoord_inc.tga", n);
		p_chol_add = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_3_Lnoord.tga", n);
		p_chol_cold = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_6_rhs_red.tga", n);
		p_rhs_hot = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_7_rhs_nnz.tga", n);
		p_rhs_cold = CTgaCodec::p_Load_TGA(p_s_filename);
		sprintf(p_s_filename, "rss2013/%05d_9_rhs_add.tga", n);
		p_rhs_add = CTgaCodec::p_Load_TGA(p_s_filename);

		p_lhs_cold = p_rhs_cold->p_Clone(true);
		p_lhs_hot = p_rhs_hot->p_Clone(true);
		std::swap(p_lhs_cold->n_width, p_lhs_cold->n_height);
		std::swap(p_lhs_hot->n_width, p_lhs_hot->n_height);
		for(int y = 0, w = p_lhs_cold->n_width, h = p_lhs_cold->n_height; y < h; ++ y) {
			for(int x = 0; x < w; ++ x) {
				p_lhs_hot->p_buffer[x + w * y] = p_rhs_hot->p_buffer[y + h * x];
				p_lhs_cold->p_buffer[x + w * y] = p_rhs_cold->p_buffer[y + h * x];
			}
		}
		// transpose here
	}

	void DeleteImages()
	{
		if(p_lambda_add) {
			p_lambda_add->Delete();
			p_lambda_add = 0;
		}
		if(p_lambda_cold) {
			p_lambda_cold->Delete();
			p_lambda_cold = 0;
		}
		if(p_lambda_hot) {
			p_lambda_hot->Delete();
			p_lambda_hot = 0;
		}
		if(p_chol_add) {
			p_chol_add->Delete();
			p_chol_add = 0;
		}
		if(p_chol_hot) {
			p_chol_hot->Delete();
			p_chol_hot = 0;
		}
		if(p_rhs_hot) {
			p_rhs_hot->Delete();
			p_rhs_hot = 0;
		}
		if(p_rhs_cold) {
			p_rhs_cold->Delete();
			p_rhs_cold = 0;
		}
		if(p_rhs_add) {
			p_rhs_add->Delete();
			p_rhs_add = 0;
		}
		if(p_lhs_hot) {
			p_lhs_hot->Delete();
			p_lhs_hot = 0;
		}
		if(p_lhs_cold) {
			p_lhs_cold->Delete();
			p_lhs_cold = 0;
		}
	}
} compo;
// composite rss-generated images

#endif // __DUMP_RSS2013_PRESENTATION_ANIMATION_DATA

#ifdef __TEST_SCHUR_ORDERINGS

#include "slam_app/Config.h"

/**
 *	@brief test of matrix orderings (for fastL debugging)
 */
class COrderingTest {
public:
	/**
	 *	@brief default constructor; hijacks the application on startup (if enabled)
	 */
	inline COrderingTest()
	{
		/*main();

		exit(-5);*/
	}

	/**
	 *	@brief main function; runs the test
	 */
	void main()
	{
		int n_frame = 943;

		char p_s_info[256], p_s_matrix[256], p_s_layout[256];
		sprintf(p_s_info, "fastLdump/%05d_0_stats.txt", n_frame);
		sprintf(p_s_matrix, "fastLdump/%05d_1_lambda.mtx", n_frame);
		sprintf(p_s_layout, "fastLdump/%05d_1_lambda.bla", n_frame);

		CUberBlockMatrix lambda;
		if(!lambda.Load_MatrixMarket(p_s_matrix, p_s_layout)) {
			fprintf(stderr, "error: failed to load block matrix (\'%s\', \'%s\')\n",
				p_s_matrix, p_s_layout);
			return;
		}

		printf("have lambda " PRIsize " x " PRIsize ", " PRIsize " nnz blocks\n",
			lambda.n_BlockRow_Num(), lambda.n_BlockColumn_Num(), lambda.n_Block_Num());

		size_t n_lambda_nnzb, n_L_nnzb, n_order_min;
		std::vector<size_t> order_fastL;

		{
			order_fastL.resize(lambda.n_BlockColumn_Num());
			FILE *p_fr = fopen(p_s_info, "r");
			if(fscanf(p_fr, PRIsize " %% blocks in lambda\n", &n_lambda_nnzb) != 1 ||
			   fscanf(p_fr, PRIsize " %% blocks in L\n", &n_L_nnzb) != 1 ||
			   fscanf(p_fr, PRIsize " %% order_min\n", &n_order_min) != 1) {
				fclose(p_fr);
				return;
			}
			for(size_t i = 0, n = order_fastL.size(); i < n; ++ i) {
				if(fscanf(p_fr, (i)? ", " PRIsize : "order = {" PRIsize, &order_fastL[i]) != 1) {
					fclose(p_fr);
					return;
				}
			}	
			fclose(p_fr);
		}
		// read the stats file

		if(CMatrixOrdering::b_IsValidOrdering(&order_fastL[0], order_fastL.size()))
			printf("have a valid ordering\n");
		else
			fprintf(stderr, "error: the given ordering is invalid\n");
		// make sure the rodering is ok

		CUberBlockMatrix lambda_perm;
		lambda.Permute_UpperTriangular_To(lambda_perm, &order_fastL[0], order_fastL.size(), true);
		// apply the permutation

		CUberBlockMatrix R;
		R.CholeskyOf(lambda_perm);

		printf("the calculated R has " PRIsize " nnz blocks\n", R.n_Block_Num());
	}
} ord_test; /**< @brief test of matrix orderings (for fastL debugging) */

/**
 *	@brief test of maximum independent set calculation
 */
class CMISTest {
public:
	/**
	 *	@brief default constructor; hijacks the application on startup (if enabled)
	 */
	inline CMISTest()
	{
		/*main();

		exit(-5);*/
	}

	/**
	 *	@brief main function; runs the test
	 */
	void main()
	{
		int n_frame = 64;

		char p_s_info[256], p_s_matrix[256], p_s_layout[256];
		sprintf(p_s_info, "fastLdump/%05d_0_stats.txt", n_frame);
		sprintf(p_s_matrix, "fastLdump/%05d_1_lambda.mtx", n_frame);
		sprintf(p_s_layout, "fastLdump/%05d_1_lambda.bla", n_frame);

		CUberBlockMatrix lambda;
		if(!lambda.Load_MatrixMarket(p_s_matrix, p_s_layout)) {
			fprintf(stderr, "error: failed to load block matrix (\'%s\', \'%s\')\n",
				p_s_matrix, p_s_layout);
			return;
		}

		cs *p_lambda = lambda.p_BlockStructure_to_Sparse();
		// load a sparse matrix lambda

		cs *p_lambda_t = cs_transpose(p_lambda, 0);
		cs *p_lambda_full = cs_add(p_lambda, p_lambda_t, 1, 1);
		cs_spfree(p_lambda_t);
		cs_spfree(p_lambda);
		p_lambda = p_lambda_full;
		// need full AT+A (like AMD, except we require the diagonal to be present here)

		CDebug::Dump_SparseMatrix("mis_0_graph.tga", p_lambda);

		/*int p_rem_test[] = {1, 3, 7, 8, 14};
		cs *p_subgraph = p_Subgraph(p_lambda, p_rem_test, p_rem_test + 5);
		CDebug::Dump_SparseMatrix("mis_1_test-subgraph.tga", p_subgraph);
		cs_spfree(p_subgraph);*/
		// seems to work nicely

		CTimer t;

		std::vector<size_t> mis = CSchurOrdering::/*t_MIS*//*t_MIS_ExStack*//*t_MIS_Parallel*/t_MIS_FirstFit(p_lambda);
		std::sort(mis.begin(), mis.end());

		printf("max independent set is: %d (it took %.2f msec,"
			" %d could be expected in a connected graph)\n",
			int(mis.size()), t.f_Time() * 1e3f, int(p_lambda->n + 1) / 2);

		printf("those are: {%d", int(mis[0]));
		for(size_t i = 1, n = mis.size(); i < n; ++ i)
			printf(", %d", int(mis[i]));
		printf("}\n");

		std::vector<size_t> ordering;
		CSchurOrdering::Complement_VertexSet(ordering, mis, p_lambda->n);
		ordering.insert(ordering.end(), mis.begin(), mis.end());
		std::vector<size_t> inv_ordering(ordering.size());
		for(size_t i = 0, n = ordering.size(); i < n; ++ i)
			inv_ordering[ordering[i]] = i;
		// have the other vertices first, mis vertices last

		cs *p_lambda_perm = cs_symperm(p_lambda, (csi*)&inv_ordering[0], 0);

		CDebug::Dump_SparseMatrix("mis_1_graph-perm.tga", p_lambda_perm);

		CUberBlockMatrix lambda_perm;
		lambda.Permute_UpperTriangular_To(lambda_perm, &inv_ordering[0], inv_ordering.size(), true);
		lambda_perm.Rasterize("mis_2_lambda-perm.tga");

		//printf("max independent set is: %d\n", n_MIS(p_lambda));

		cs_spfree(p_lambda_perm);
		cs_spfree(p_lambda);
	}
} mis_test; /**< @brief test of maximum independent set calculation */

#endif // __TEST_SCHUR_ORDERINGS

#ifdef __TEST_VECTOR_REFS

#include "slam/BlockMatrixBase.h"
#include "eigen/Eigen/Core"
#include <iostream>

class CTestEigenMapConversions {
public:
	CTestEigenMapConversions()
	{
		Run();
	}

	static void Run()
	{
		Eigen::VectorXd vec = Eigen::VectorXd::Ones(3);
		Eigen::Vector3d vec3 = Eigen::Vector3d::Ones() * 3.0;

		//Eigen::Map<Eigen::VectorXd> map(vec); // not ok, Eigen does not support that internally
		//Eigen::Map<Eigen::Vector3d> map3(vec3); // not ok, Eigen does not support that internally
		Eigen::Map<Eigen::VectorXd> map(vec.data(), vec.rows()); // ok
		std::cout << map << std::endl;
		Eigen::Map<Eigen::VectorXd> map_static(vec3.data(), vec3.RowsAtCompileTime); // ok
		std::cout << map_static << std::endl;
		Eigen::Map<Eigen::Vector3d> map3(vec3.data(), vec3.rows()); // ok
		std::cout << map3 << std::endl;
		//Eigen::Map<Eigen::VectorXd> map_from3(map3); // not ok, Eigen does not support that internally
		Eigen::Map<Eigen::VectorXd> map_from3(map3.data(), map3.rows()); // ok
		std::cout << map_from3 << std::endl;
		Eigen::Map<Eigen::VectorXd> map_from3_static(map3.data(), map3.RowsAtCompileTime); // ok
		std::cout << map_from3_static << std::endl;
		Eigen::Map<Eigen::Vector3d> map3_fromX(map.data(), map.rows()); // ok
		std::cout << map3_fromX << std::endl;
		Eigen::Map<Eigen::Vector3d> map3_from_static(map3.data(), map3.RowsAtCompileTime); // ok
		std::cout << map3_from_static << std::endl;

		Eigen::Map<Eigen::VectorXd> map_ref(t_MakeVectorRef<Eigen::Map<Eigen::VectorXd> >(vec)); // ok
		Eigen::Map<Eigen::VectorXd> map_static_ref(t_MakeVectorRef<Eigen::Map<Eigen::VectorXd> >(vec3)); // ok
		Eigen::Map<Eigen::Vector3d> map3_Xref(t_MakeVectorRef<Eigen::Map<Eigen::Vector3d> >(vec)); // ok
		Eigen::Map<Eigen::Vector3d> map3_ref(t_MakeVectorRef<Eigen::Map<Eigen::Vector3d> >(vec3)); // ok
		Eigen::Map<Eigen::VectorXd> map_from3_ref(t_MakeVectorRef<Eigen::Map<Eigen::VectorXd> >(map3)); // ok
		Eigen::Map<Eigen::VectorXd> map_fromX_ref(t_MakeVectorRef<Eigen::Map<Eigen::VectorXd> >(map)); // ok
		Eigen::Map<Eigen::Vector3d> map3_fromX_ref(t_MakeVectorRef<Eigen::Map<Eigen::Vector3d> >(map)); // ok
		Eigen::Vector3d &ref3_fromv3 = t_MakeVectorRef<Eigen::Vector3d&>(vec3); // ok
		Eigen::VectorXd &ref_fromvX = t_MakeVectorRef<Eigen::VectorXd&>(vec); // ok
		//Eigen::VectorXd &ref_fromv3 = t_MakeVectorRef<Eigen::VectorXd&>(vec3); // not ok (expected) // caught by msvc
		//Eigen::Vector3d &ref3_fromvX = t_MakeVectorRef<Eigen::Vector3d&>(vec); // not ok (expected) // caught by msvc
		//Eigen::Vector3d &ref3_fromm3 = t_MakeVectorRef<Eigen::Vector3d&>(map3); // not ok (expected) // caught by msvc
		//Eigen::Vector3d &ref3_frommX = t_MakeVectorRef<Eigen::Vector3d&>(map); // not ok (expected) // caught by msvc
		//Eigen::VectorXd &ref_fromm3 = t_MakeVectorRef<Eigen::VectorXd&>(map3); // not ok (expected) // caught by msvc
		//Eigen::VectorXd &ref_frommX = t_MakeVectorRef<Eigen::VectorXd&>(map); // not ok (expected) // caught by msvc
	}
} run_eigconvtest;

#endif // __TEST_VECTOR_REFS

#ifdef __TEST_ROTATION_AVERAGING

#include "slam/3DSolverBase.h"

/**
 *	@brief rotation averaging test
 *
 *	This is roughly the implementation of Algorithm 1 (A1) in Govindu, Venu Madhav,
 *	"Lie-algebraic averaging for globally consistent motion estimation," CVPR, 2004.
 *	Note that the Algorithm 2 degenerates to Algorithm 1 if there is only a single
 *	camera pair, reobserved many times.
 */
class CRotationAveragingTest {
public:
	CRotationAveragingTest()
	{
		Run();
		exit(0);
	}

	typedef C3DJacobians::TSE3 TPose;

	static TPose t_Random_Pose(double f_translation_scale, double f_rotation_angle)
	{
		Eigen::Vector3d v_random_axis = Eigen::Vector3d::Random();
		v_random_axis.normalize();
		v_random_axis *= f_rotation_angle;
		Eigen::Quaterniond q;
		C3DJacobians::AxisAngle_to_Quat(v_random_axis, q);
		// get a random axis

		Eigen::Vector3d v_random_translation = Eigen::Vector3d::Random();
		v_random_translation *= f_translation_scale;

		return TPose(v_random_translation, q);
	}

	template <class X, class Y>
	struct TPair {
		X first;
		Y second;

		TPair(const X &r_first, const Y &r_second)
			:first(r_first), second(r_second)
		{}

		TPair(const TPair &r_other)
			:first(r_other.first), second(r_other.second)
		{}
	};

	static void Run()
	{
		for(int n_outer_pass = 0; n_outer_pass < 10; ++ n_outer_pass) {
			TPose t_measurement = t_Random_Pose(20, 1.3);
			// make a random measurement

			std::vector<TPair<TPose, TPose> > observations; // will not compile in 32-bit MSVC. just needed to try it quickly, aligned allocator won't help.

			for(int i = 0; i < 100; ++ i) {
				TPose t_noise0 = t_Random_Pose(.5, .1);
				TPose t_noise1 = t_Random_Pose(.5, .1);
				// generate random noise at pose observations of both cameras

				TPose t_observation0 = t_noise0;
				TPose t_observation1 = t_measurement;
				t_observation1 *= t_noise1;
				// calculate absolute observations

				observations.push_back(TPair<TPose, TPose>(t_observation0, t_observation1));

				/*Eigen::Vector6f v_obs0 = t_observation0.v_Exp();
				Eigen::Vector6f v_obs1 = t_observation1.v_Exp();*/
				// exponential map
			}
			// generate a bunch of observations

			TPose t_estimated = TPose::t_Identity();
			for(int n_pass = 0; n_pass < 5; ++ n_pass) {
				Eigen::Vector6d v_residual_se3 = Eigen::Vector6d::Zero();
				for(size_t i = 0, n = observations.size(); i < n; ++ i) {
					TPose t_observation = observations[i].first;
					t_observation.Inverse_Compose(observations[i].second);
					// take a delta observation

					if(!n_pass && !n_outer_pass) {
						double f_error = (t_observation.v_Log() - t_measurement.v_Log()).norm();
						printf("observation error: %f\n", f_error);
					}
					// debug

					TPose t_difference = t_estimated;
					t_difference.Inverse_Compose(t_observation);
					v_residual_se3 += t_difference.v_Log();
				}
				v_residual_se3 /= double(observations.size());
				t_estimated *= TPose(v_residual_se3, TPose::from_se3_vector);
				// update

				printf("residual norm: %f\n", v_residual_se3.norm());
			}

			double f_error = (t_estimated.v_Log() - t_measurement.v_Log()).norm();
			printf("final error: %f\n", f_error);
			// that seems to work well. however, there's no way to get the covariance
		}
	}
} run_rotavg;

#endif // __TEST_ROTATION_AVERAGING

#ifdef __TEST_ERROR_EVAL

#include "slam/ErrorEval.h"

static class CRunMe {
public:
	CRunMe()
	{
		printf("in 3D:\n");
		CErrorEvaluation<false>::Test(); // debug
		printf("\nin 2D:\n");
		CErrorEvaluation<true>::Test(); // debug
		if(rand() > RAND_MAX) {
			CErrorEvaluation<false>::_TyVectorUnalign v;
			C3DJacobians::Pose_Inverse(v, v);
		}
	}
} runme;

#endif // __TEST_ERROR_EVAL

#ifdef __TEST_EIGENSOLVER

void Eigenvalues_UnitTest();

#include <stdlib.h>

static class CMyEigsUnitTest {
public:
	CMyEigsUnitTest()
	{
		Eigenvalues_UnitTest();
		exit(0);
	}
} run_eigs_test;

#endif // __TEST_EIGENSOLVER
