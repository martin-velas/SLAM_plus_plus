/*
								+----------------------------------+
								|                                  |
								|  ***  Homography estimator  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|          Homography.inl          |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __HOMOGRAPHY_ESTIMATION_INLINES_INCLUDED
#define __HOMOGRAPHY_ESTIMATION_INLINES_INCLUDED

/**
 *	@file geometry/Homography.inl
 *	@brief homography estimation inline implementation
 *	@author -tHE SWINe-
 *	@date 2013
 */

/*
 *								=== CHomography::CTransferErrorObjective ===
 */

template <class CDerived0, class CDerived1, class CDerived2>
inline CHomography::CTransferErrorObjective::_TyResidual
	CHomography::CTransferErrorObjective::v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
	const Eigen::MatrixBase<CDerived1> &r_v_template,
	const Eigen::MatrixBase<CDerived2> &r_t_H)
{
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);

	Eigen::Vector3d v_transformed = r_t_H * Eigen::Vector3d(r_v_template(0), r_v_template(1), 1);
	return r_v_image - v_transformed.head<2>() / v_transformed(2);
}

template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
inline CHomography::CTransferErrorObjective::_TyResidual
	CHomography::CTransferErrorObjective::v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
	const Eigen::MatrixBase<CDerived1> &r_v_image,
	const Eigen::MatrixBase<CDerived2> &r_v_template,
	const Eigen::MatrixBase<CDerived3> &r_t_H)
{
	DimensionCheck<_TyJacobian>(r_t_J);
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);

	const double xix = r_v_image(0), xiy = r_v_image(1),
		Xix = r_v_template(0), Xiy = r_v_template(1),
		h11 = r_t_H(0, 0), h12 = r_t_H(0, 1), h13 = r_t_H(0, 2),
		h21 = r_t_H(1, 0), h22 = r_t_H(1, 1), h23 = r_t_H(1, 2),
		h31 = r_t_H(2, 0), h32 = r_t_H(2, 1), h33 = r_t_H(2, 2);

	Eigen::Vector3d v_transformed = r_t_H * Eigen::Vector3d(r_v_template(0), r_v_template(1), 1);
	Eigen::Vector2d v_transformed_dehom = v_transformed.head<2>() / v_transformed(2);

	double f_temp0 = v_transformed(2); // this could be reciprocal but that tends to decrease precision slightly
	r_t_J(0, 0) = Xix / f_temp0;
	r_t_J(1, 0) = Xiy / f_temp0;
	r_t_J(2, 0) = 1 / f_temp0;
	r_t_J(3, 0) = 0.0;
	r_t_J(4, 0) = 0.0;
	r_t_J(5, 0) = 0.0;
	double f_temp1 = f_temp0 * f_temp0;
	double f_temp2 = -v_transformed(0) / f_temp1;
	r_t_J(6, 0) = f_temp2 * Xix;
	r_t_J(7, 0) = f_temp2 * Xiy;
	r_t_J(8, 0) = f_temp2;
	r_t_J(0, 1) = 0.0;
	r_t_J(1, 1) = 0.0;
	r_t_J(2, 1) = 0.0;
	r_t_J(3, 1) = Xix / f_temp0;
	r_t_J(4, 1) = Xiy / f_temp0;
	r_t_J(5, 1) = 1 / f_temp0;
	double f_temp3 = -v_transformed(1) / f_temp1;
	r_t_J(6, 1) = f_temp3 * Xix;
	r_t_J(7, 1) = f_temp3 * Xiy;
	r_t_J(8, 1) = f_temp3;

	return r_v_image - v_transformed_dehom;
	// t_odo - replace common subexpressions
}

/*
 *								=== ~CHomography::CTransferErrorObjective ===
 */

/*
 *								=== CHomography::CSymTransferErrorObjective ===
 */

inline CHomography::CSymTransferErrorObjective::CSymTransferErrorObjective(const Eigen::Matrix3d &r_t_H,
	bool b_will_do_jacobians /*= true*/)
	:m_r_t_H(r_t_H), m_t_H_inv(r_t_H.inverse())
{
	if(b_will_do_jacobians) {
#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
		m_t_JInv = t_Inverse_Jacobian(r_t_H);
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
	}
}

template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
CHomography::CSymTransferErrorObjective::_TyResidual
	CHomography::CSymTransferErrorObjective::v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
	const Eigen::MatrixBase<CDerived1> &r_v_template,
	const Eigen::MatrixBase<CDerived2> &r_t_H,
	const Eigen::MatrixBase<CDerived3> &r_t_H_inv)
{
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);
	DimensionCheck<Eigen::Matrix3d>(r_t_H_inv);

	Eigen::Vector3d v_transformed = r_t_H * Eigen::Vector3d(r_v_template(0), r_v_template(1), 1);
	Eigen::Vector3d v_inv_transformed = r_t_H_inv * Eigen::Vector3d(r_v_image(0), r_v_image(1), 1);
	Eigen::Vector4d v_error;
	v_error.head<2>() = r_v_image - v_transformed.head<2>() / v_transformed(2);
	v_error.tail<2>() = r_v_template - v_inv_transformed.head<2>() / v_inv_transformed(2);

	return v_error;
}

Eigen::Matrix<double, 9, 9> CHomography::CSymTransferErrorObjective::t_Inverse_Jacobian(const Eigen::Matrix3d &r_t_H)
{
	const double h11 = r_t_H(0, 0), h12 = r_t_H(0, 1), h13 = r_t_H(0, 2),
		h21 = r_t_H(1, 0), h22 = r_t_H(1, 1), h23 = r_t_H(1, 2),
		h31 = r_t_H(2, 0), h32 = r_t_H(2, 1), h33 = r_t_H(2, 2);
	double f_temp0 = -h11 * h22 + h12 * h21, f_temp1 = -h11 * h32 + h12 * h31, f_temp2 = h21 * h32 - h22 * h31,
		f_temp3 = h11 * h23 - h13 * h21, f_temp4 = h12 * h23 - h13 * h22, f_temp5 = h11 * h33 - h13 * h31,
		f_temp6 = h21 * h33 - h23 * h31, f_temp7 = h12 * h33 - h13 * h32, f_temp8 = -h22 * h33 + h23 * h32,
		f_temp9 = f_temp8 * h11 - h31 * h12 * h23 + h31 * h13 * h22 + h21 * h12 * h33 - h21 * h13 * h32,
		f_temp10 = f_temp9 * f_temp9;
	Eigen::Matrix<double, 9, 9> JInv;
	JInv <<
		-f_temp8 * f_temp8, -f_temp8 * f_temp7,  f_temp8 * f_temp4, -f_temp8 * f_temp6, -f_temp6 * f_temp7,
		 f_temp6 * f_temp4,  f_temp2 * f_temp8,  f_temp2 * f_temp7, -f_temp2 * f_temp4, -f_temp8 * f_temp6,
		 f_temp8 * f_temp5, -f_temp3 * f_temp8, -f_temp6 * f_temp6,  f_temp6 * f_temp5, -f_temp3 * f_temp6,
		 f_temp2 * f_temp6, -f_temp2 * f_temp5,  f_temp3 * f_temp2,  f_temp2 * f_temp8,  f_temp8 * f_temp1,
		-f_temp8 * f_temp0,  f_temp2 * f_temp6,  f_temp6 * f_temp1, -f_temp6 * f_temp0, -f_temp2 * f_temp2,
		-f_temp2 * f_temp1,  f_temp2 * f_temp0, -f_temp8 * f_temp7, -f_temp7 * f_temp7,  f_temp4 * f_temp7,
		 f_temp8 * f_temp5,  f_temp7 * f_temp5, -f_temp4 * f_temp5,  f_temp8 * f_temp1,  f_temp7 * f_temp1,
		-f_temp4 * f_temp1, -f_temp6 * f_temp7,  f_temp7 * f_temp5, -f_temp3 * f_temp7,  f_temp6 * f_temp5,
		-f_temp5 * f_temp5,  f_temp3 * f_temp5,  f_temp6 * f_temp1, -f_temp5 * f_temp1,  f_temp3 * f_temp1,
		 f_temp2 * f_temp7,  f_temp7 * f_temp1, -f_temp7 * f_temp0, -f_temp2 * f_temp5, -f_temp5 * f_temp1,
		 f_temp5 * f_temp0, -f_temp2 * f_temp1, -f_temp1 * f_temp1,  f_temp1 * f_temp0,  f_temp8 * f_temp4,
		 f_temp4 * f_temp7, -f_temp4 * f_temp4, -f_temp3 * f_temp8, -f_temp3 * f_temp7,  f_temp3 * f_temp4,
		-f_temp8 * f_temp0, -f_temp7 * f_temp0,  f_temp4 * f_temp0,  f_temp6 * f_temp4, -f_temp4 * f_temp5,
		 f_temp3 * f_temp4, -f_temp3 * f_temp6,  f_temp3 * f_temp5, -f_temp3 * f_temp3, -f_temp6 * f_temp0,
		 f_temp5 * f_temp0, -f_temp3 * f_temp0, -f_temp2 * f_temp4, -f_temp4 * f_temp1,  f_temp4 * f_temp0,
		 f_temp3 * f_temp2,  f_temp3 * f_temp1, -f_temp3 * f_temp0,  f_temp2 * f_temp0,  f_temp1 * f_temp0,
		-f_temp0 * f_temp0;
	JInv /= f_temp10;
	// the Jacobian of inverse

	return JInv;
}

#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC)
template <class CDerived0, class CDerived1, class CDerived2, class CDerived3, class CDerived4, class CDerived5>
CHomography::CSymTransferErrorObjective::_TyResidual
	CHomography::CSymTransferErrorObjective::v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
	const Eigen::MatrixBase<CDerived1> &r_v_image,
	const Eigen::MatrixBase<CDerived2> &r_v_template,
	const Eigen::MatrixBase<CDerived3> &r_t_H,
	const Eigen::MatrixBase<CDerived4> &r_t_H_inv,
	const Eigen::MatrixBase<CDerived5> &r_t_JInv)
#else // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
template <class CDerived0, class CDerived1, class CDerived2, class CDerived3, class CDerived4>
CHomography::CSymTransferErrorObjective::_TyResidual
	CHomography::CSymTransferErrorObjective::v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_t_J,
	const Eigen::MatrixBase<CDerived1> &r_v_image,
	const Eigen::MatrixBase<CDerived2> &r_v_template,
	const Eigen::MatrixBase<CDerived3> &r_t_H,
	const Eigen::MatrixBase<CDerived4> &r_t_H_inv)
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC
{
	DimensionCheck<_TyJacobian>(r_t_J);
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);
	DimensionCheck<Eigen::Matrix3d>(r_t_H_inv);
	DimensionCheck<Eigen::Matrix<double, 9, 9> >(r_t_JInv);

	const double xix = r_v_image(0), xiy = r_v_image(1),
		Xix = r_v_template(0), Xiy = r_v_template(1),
		h11 = r_t_H(0, 0), h12 = r_t_H(0, 1), h13 = r_t_H(0, 2),
		h21 = r_t_H(1, 0), h22 = r_t_H(1, 1), h23 = r_t_H(1, 2),
		h31 = r_t_H(2, 0), h32 = r_t_H(2, 1), h33 = r_t_H(2, 2);

	Eigen::Vector4d v_error;

#if defined(SYM_TRANSFER_USE_CHAIN_RULE) || defined(TEST_SYM_TRANSFER_JAC) // use the chain rule (~531 FLOP)
	{
		const Eigen::Matrix3d &t_H_inv = r_t_H_inv;//r_t_H.inverse(); // just rename
		const double hi11 = t_H_inv(0, 0), hi12 = t_H_inv(0, 1), hi13 = t_H_inv(0, 2),
			hi21 = t_H_inv(1, 0), hi22 = t_H_inv(1, 1), hi23 = t_H_inv(1, 2),
			hi31 = t_H_inv(2, 0), hi32 = t_H_inv(2, 1), hi33 = t_H_inv(2, 2);
		/*double f_temp0 = -h11 * h22 + h12 * h21, f_temp1 = -h11 * h32 + h12 * h31, f_temp2 = h21 * h32 - h22 * h31,
			f_temp3 = h11 * h23 - h13 * h21, f_temp4 = h12 * h23 - h13 * h22, f_temp5 = h11 * h33 - h13 * h31,
			f_temp6 = h21 * h33 - h23 * h31, f_temp7 = h12 * h33 - h13 * h32, f_temp8 = -h22 * h33 + h23 * h32,
			f_temp9 = f_temp8 * h11 - h31 * h12 * h23 + h31 * h13 * h22 + h21 * h12 * h33 - h21 * h13 * h32,
			f_temp10 = f_temp9 * f_temp9;
		Eigen::Matrix<double, 9, 9> JInv;
		JInv <<
			-f_temp8 * f_temp8, -f_temp8 * f_temp7,  f_temp8 * f_temp4, -f_temp8 * f_temp6, -f_temp6 * f_temp7,
			 f_temp6 * f_temp4,  f_temp2 * f_temp8,  f_temp2 * f_temp7, -f_temp2 * f_temp4, -f_temp8 * f_temp6,
			 f_temp8 * f_temp5, -f_temp3 * f_temp8, -f_temp6 * f_temp6,  f_temp6 * f_temp5, -f_temp3 * f_temp6,
			 f_temp2 * f_temp6, -f_temp2 * f_temp5,  f_temp3 * f_temp2,  f_temp2 * f_temp8,  f_temp8 * f_temp1,
			-f_temp8 * f_temp0,  f_temp2 * f_temp6,  f_temp6 * f_temp1, -f_temp6 * f_temp0, -f_temp2 * f_temp2,
			-f_temp2 * f_temp1,  f_temp2 * f_temp0, -f_temp8 * f_temp7, -f_temp7 * f_temp7,  f_temp4 * f_temp7,
			 f_temp8 * f_temp5,  f_temp7 * f_temp5, -f_temp4 * f_temp5,  f_temp8 * f_temp1,  f_temp7 * f_temp1,
			-f_temp4 * f_temp1, -f_temp6 * f_temp7,  f_temp7 * f_temp5, -f_temp3 * f_temp7,  f_temp6 * f_temp5,
			-f_temp5 * f_temp5,  f_temp3 * f_temp5,  f_temp6 * f_temp1, -f_temp5 * f_temp1,  f_temp3 * f_temp1,
			 f_temp2 * f_temp7,  f_temp7 * f_temp1, -f_temp7 * f_temp0, -f_temp2 * f_temp5, -f_temp5 * f_temp1,
			 f_temp5 * f_temp0, -f_temp2 * f_temp1, -f_temp1 * f_temp1,  f_temp1 * f_temp0,  f_temp8 * f_temp4,
			 f_temp4 * f_temp7, -f_temp4 * f_temp4, -f_temp3 * f_temp8, -f_temp3 * f_temp7,  f_temp3 * f_temp4,
			-f_temp8 * f_temp0, -f_temp7 * f_temp0,  f_temp4 * f_temp0,  f_temp6 * f_temp4, -f_temp4 * f_temp5,
			 f_temp3 * f_temp4, -f_temp3 * f_temp6,  f_temp3 * f_temp5, -f_temp3 * f_temp3, -f_temp6 * f_temp0,
			 f_temp5 * f_temp0, -f_temp3 * f_temp0, -f_temp2 * f_temp4, -f_temp4 * f_temp1,  f_temp4 * f_temp0,
			 f_temp3 * f_temp2,  f_temp3 * f_temp1, -f_temp3 * f_temp0,  f_temp2 * f_temp0,  f_temp1 * f_temp0,
			-f_temp0 * f_temp0;
		JInv /= f_temp10;
		// the Jacobian of inverse; a bit lengthy, still a lot of common subexpressions in there*/
		// this was precomputed in the constructor

		Eigen::Vector3d v_transformed = r_t_H * Eigen::Vector3d(r_v_template(0), r_v_template(1), 1);
		Eigen::Vector3d v_inv_transformed = t_H_inv * Eigen::Vector3d(r_v_image(0), r_v_image(1), 1);

		v_error.head<2>() = r_v_image - (v_transformed.head<2>() /= v_transformed(2));
		v_error.tail<2>() = r_v_template - (v_inv_transformed.head<2>() /= v_inv_transformed(2));
		// calculate error first

		v_transformed.head<2>() /= -v_transformed(2);
		v_inv_transformed.head<2>() /= -v_inv_transformed(2);
		Eigen::Vector3d v_template_tr_h = Eigen::Vector3d(r_v_template(0), r_v_template(1), 1) / v_transformed(2);
		Eigen::Vector3d v_image_invtr_h = Eigen::Vector3d(r_v_image(0), r_v_image(1), 1) / v_inv_transformed(2);
		Eigen::Vector2d v_template_trx = r_v_template * v_transformed(0),
			v_template_try = r_v_template * v_transformed(1);
		Eigen::Vector2d v_image_invtrx = r_v_image * v_inv_transformed(0),
			v_image_invtry = r_v_image * v_inv_transformed(1);

		r_t_J(0, 0) = v_template_tr_h(0);
		r_t_J(1, 0) = v_template_tr_h(1);
		r_t_J(2, 0) = v_template_tr_h(2);
		r_t_J(3, 0) = 0.0;
		r_t_J(4, 0) = 0.0;
		r_t_J(5, 0) = 0.0;
		r_t_J(6, 0) = v_template_trx(0);
		r_t_J(7, 0) = v_template_trx(1);
		r_t_J(8, 0) = v_transformed(0);
		r_t_J(0, 1) = 0.0;
		r_t_J(1, 1) = 0.0;
		r_t_J(2, 1) = 0.0;
		r_t_J(3, 1) = v_template_tr_h(0);
		r_t_J(4, 1) = v_template_tr_h(1);
		r_t_J(5, 1) = v_template_tr_h(2);
		r_t_J(6, 1) = v_template_try(0);
		r_t_J(7, 1) = v_template_try(1);
		r_t_J(8, 1) = v_transformed(1);

		r_t_J(0, 2) = v_image_invtr_h(0);
		r_t_J(1, 2) = v_image_invtr_h(1);
		r_t_J(2, 2) = v_image_invtr_h(2);
		r_t_J(3, 2) = 0.0;
		r_t_J(4, 2) = 0.0;
		r_t_J(5, 2) = 0.0;
		r_t_J(6, 2) = v_image_invtrx(0);
		r_t_J(7, 2) = v_image_invtrx(1);
		r_t_J(8, 2) = v_inv_transformed(0);
		r_t_J(0, 3) = 0.0;
		r_t_J(1, 3) = 0.0;
		r_t_J(2, 3) = 0.0;
		r_t_J(3, 3) = v_image_invtr_h(0);
		r_t_J(4, 3) = v_image_invtr_h(1);
		r_t_J(5, 3) = v_image_invtr_h(2);
		r_t_J(6, 3) = v_image_invtry(0);
		r_t_J(7, 3) = v_image_invtry(1);
		r_t_J(8, 3) = v_inv_transformed(1);

		r_t_J.template rightCols<2>() = r_t_JInv * r_t_J.template rightCols<2>();
	}
	// try using chain rule for the inverse of H; should make the jacobian shorter
#ifdef TEST_SYM_TRANSFER_JAC
	_TyJacobian J_ChainRule = r_t_J;
#endif // TEST_SYM_TRANSFER_JAC
#endif // SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC

#if !defined(SYM_TRANSFER_USE_CHAIN_RULE) || TEST_SYM_TRANSFER_JAC // use the end-to-end jacobian (~803 FLOP but not sure how well are the common subexpressions eliminated)
	const double f_temp1 = h11 * h22 - h12 * h21,
		f_temp2 = h11 * h22 * h33 - h11 * h23 * h32 - h21 * h12 * h33 +
		h21 * h13 * h32 + h31 * h12 * h23 - h31 * h13 * h22;
	const double f_temp3 = -f_temp1,
		f_temp4 = -h11 * h32 + h12 * h31,
		f_temp5 = h21 * h32 - h22 * h31;
	const double f_temp6 = f_temp5 / f_temp2 * xix +
		f_temp4 / f_temp2 * xiy - f_temp3 / f_temp2,
		f_temp7 = -h11 * h23 + h13 * h21,
		f_temp8  = -h11 * h33 + h13 * h31,
		f_temp9  = h21 * h33 - h23 * h31;
	const double f_temp10 = -f_temp9 / f_temp2 * xix -
		f_temp8 / f_temp2 * xiy + f_temp7 / f_temp2,
		f_temp11 = -h12 * h23 + h13 * h22,
		f_temp12 = -h12 * h33 + h13 * h32,
		f_temp13 = h22 * h33 - h23 * h32;
	const double f_temp14 = f_temp13 / f_temp2 * xix +
		f_temp12 / f_temp2 * xiy - f_temp11 / f_temp2,
		f_temp15 = h31 * Xix + h32 * Xiy + h33,
		f_temp16 = -f_temp11,
		f_temp17 = -f_temp8,
		f_temp18 = -f_temp9;
	const double f_temp19 = -(h11 * Xix + h12 * Xiy + h13) / f_Sqr(f_temp15),
		f_temp20 = -(h21 * Xix + h22 * Xiy + h23) / f_Sqr(f_temp15);

	r_t_J(0, 0) = Xix / f_temp15;
	r_t_J(1, 0) = Xiy / f_temp15;
	r_t_J(2, 0) = 1 / f_temp15;
	r_t_J(3, 0) = 0.0;
	r_t_J(4, 0) = 0.0;
	r_t_J(5, 0) = 0.0;
	r_t_J(6, 0) = f_temp19 * Xix;
	r_t_J(7, 0) = f_temp19 * Xiy;
	r_t_J(8, 0) = f_temp19;
	r_t_J(0, 1) = 0.0;
	r_t_J(1, 1) = 0.0;
	r_t_J(2, 1) = 0.0;
	r_t_J(3, 1) = Xix / f_temp15;
	r_t_J(4, 1) = Xiy / f_temp15;
	r_t_J(5, 1) = 1 / f_temp15;
	r_t_J(6, 1) = f_temp20 * Xix;
	r_t_J(7, 1) = f_temp20 * Xiy;
	r_t_J(8, 1) = f_temp20;
	r_t_J(0, 2) = (-f_Sqr(f_temp13 / f_temp2) * xix - f_temp12 / f_Sqr(f_temp2) * xiy * f_temp13 +
		f_temp11 / f_Sqr(f_temp2) * f_temp13) / f_temp6 - f_temp14 / f_Sqr(f_temp6) * (-f_temp5 /
		f_Sqr(f_temp2) * xix * f_temp13 - h32 / f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) * xiy *
		f_temp13 + h22 / f_temp2 + f_temp3 / f_Sqr(f_temp2) * f_temp13);
	r_t_J(1, 2) = (-f_temp13 / f_Sqr(f_temp2) * xix * f_temp18 - h33 / f_temp2 * xiy - f_temp12 /
		f_Sqr(f_temp2) * xiy * f_temp18 + h23 / f_temp2 + f_temp11 / f_Sqr(f_temp2) * f_temp18) /
		f_temp6 - f_temp14 / f_Sqr(f_temp6) * (-f_temp5 / f_Sqr(f_temp2) * xix * f_temp18 + h31 /
		f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp18 - h21 / f_temp2 + f_temp3 /
		f_Sqr(f_temp2) * f_temp18);
	r_t_J(2, 2) = (-f_temp5 / f_Sqr(f_temp2) * xix * f_temp13 + h32 / f_temp2 * xiy - f_temp12 /
		f_Sqr(f_temp2) * xiy * f_temp5 - h22 / f_temp2 + f_temp11 / f_Sqr(f_temp2) * f_temp5) /
		f_temp6 - f_temp14 / f_Sqr(f_temp6) * (-f_Sqr(f_temp5 / f_temp2) * xix - f_temp4 /
		f_Sqr(f_temp2) * xiy * f_temp5 + f_temp3 / f_Sqr(f_temp2) * f_temp5);
	r_t_J(3, 2) = (-f_temp13 / f_Sqr(f_temp2) * xix * f_temp12 - f_Sqr(f_temp12 / f_temp2) *
		xiy + f_temp11 / f_Sqr(f_temp2) * f_temp12) / f_temp6 - f_temp14 / f_Sqr(f_temp6) *
		(h32 / f_temp2 * xix - f_temp5 / f_Sqr(f_temp2) * xix * f_temp12 - f_temp4 / f_Sqr(f_temp2) *
		xiy * f_temp12 - h12 / f_temp2 + f_temp3 / f_Sqr(f_temp2) * f_temp12);
	r_t_J(4, 2) = (h33 / f_temp2 * xix - f_temp13 / f_Sqr(f_temp2) * xix * f_temp17 - f_temp12 /
		f_Sqr(f_temp2) * xiy * f_temp17 - h13 / f_temp2 + f_temp11 / f_Sqr(f_temp2) * f_temp17) /
		f_temp6 - f_temp14 / f_Sqr(f_temp6) * (-h31 / f_temp2 * xix - f_temp5 / f_Sqr(f_temp2) *
		xix * f_temp17 - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp17 + h11 / f_temp2 + f_temp3 /
		f_Sqr(f_temp2) * f_temp17);
	r_t_J(5, 2) = (-h32 / f_temp2 * xix - f_temp13 / f_Sqr(f_temp2) * xix * f_temp4 - f_temp4 /
		f_Sqr(f_temp2) * xiy * f_temp12 + h12 / f_temp2 + f_temp11 / f_Sqr(f_temp2) * f_temp4) /
		f_temp6 - f_temp14 / f_Sqr(f_temp6) * (-f_temp5 / f_Sqr(f_temp2) * xix * f_temp4 -
		f_Sqr(f_temp4 / f_temp2) * xiy + f_temp3 / f_Sqr(f_temp2) * f_temp4);
	r_t_J(6, 2) = (-f_temp13 / f_Sqr(f_temp2) * xix * f_temp16 - f_temp12 / f_Sqr(f_temp2) *
		xiy * f_temp16 + f_temp11 / f_Sqr(f_temp2) * f_temp16) / f_temp6 - f_temp14 /
		f_Sqr(f_temp6) * (-h22 / f_temp2 * xix - f_temp5 / f_Sqr(f_temp2) * xix * f_temp16 +
		h12 / f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp16 + f_temp3 /
		f_Sqr(f_temp2) * f_temp16);
	r_t_J(7, 2) = (-h23 / f_temp2 * xix - f_temp13 / f_Sqr(f_temp2) * xix * f_temp7 + h13 /
		f_temp2 * xiy - f_temp12 / f_Sqr(f_temp2) * xiy * f_temp7 + f_temp11 / f_Sqr(f_temp2) *
		f_temp7) / f_temp6 - f_temp14 / f_Sqr(f_temp6) * (h21 / f_temp2 * xix - f_temp5 /
		f_Sqr(f_temp2) * xix * f_temp7 - h11 / f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) * xiy *
		f_temp7 + f_temp3 / f_Sqr(f_temp2) * f_temp7);
	r_t_J(8, 2) = (h22 / f_temp2 * xix - f_temp13 / f_Sqr(f_temp2) * xix * f_temp1 - h12 /
		f_temp2 * xiy - f_temp12 / f_Sqr(f_temp2) * xiy * f_temp1 + f_temp11 / f_Sqr(f_temp2) *
		f_temp1) / f_temp6 - f_temp14 / f_Sqr(f_temp6) * (-f_temp5 / f_Sqr(f_temp2) * xix *
		f_temp1 - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp1 + f_temp3 / f_Sqr(f_temp2) * f_temp1);
	r_t_J(0, 3) = (f_temp9 / f_Sqr(f_temp2) * xix * f_temp13 + h33 / f_temp2 * xiy + f_temp8 /
		f_Sqr(f_temp2) * xiy * f_temp13 - h23 / f_temp2 - f_temp7 / f_Sqr(f_temp2) * f_temp13) /
		f_temp6 - f_temp10 / f_Sqr(f_temp6) * (-f_temp5 / f_Sqr(f_temp2) * xix * f_temp13 - h32 /
		f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp13 + h22 / f_temp2 + f_temp3 /
		f_Sqr(f_temp2) * f_temp13);
	r_t_J(1, 3) = (f_temp9 / f_Sqr(f_temp2) * xix * f_temp18 + f_temp8 / f_Sqr(f_temp2) * xiy *
		f_temp18 - f_temp7 / f_Sqr(f_temp2) * f_temp18) / f_temp6 - f_temp10 / f_Sqr(f_temp6) *
		(-f_temp5 / f_Sqr(f_temp2) * xix * f_temp18 + h31 / f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) *
		xiy * f_temp18 - h21 / f_temp2 + f_temp3 / f_Sqr(f_temp2) * f_temp18);
	r_t_J(2, 3) = (f_temp9 / f_Sqr(f_temp2) * xix * f_temp5 - h31 / f_temp2 * xiy + f_temp8 /
		f_Sqr(f_temp2) * xiy * f_temp5 + h21 / f_temp2 - f_temp7 / f_Sqr(f_temp2) * f_temp5) /
		f_temp6 - f_temp10 / f_Sqr(f_temp6) * (-f_Sqr(f_temp5 / f_temp2) * xix - f_temp4 /
		f_Sqr(f_temp2) * xiy * f_temp5 + f_temp3 / f_Sqr(f_temp2) * f_temp5);
	r_t_J(3, 3) = (-h33 / f_temp2 * xix + f_temp9 / f_Sqr(f_temp2) * xix * f_temp12 + f_temp8 /
		f_Sqr(f_temp2) * xiy * f_temp12 + h13 / f_temp2 - f_temp7 / f_Sqr(f_temp2) * f_temp12) /
		f_temp6 - f_temp10 / f_Sqr(f_temp6) * (h32 / f_temp2 * xix - f_temp5 / f_Sqr(f_temp2) *
		xix * f_temp12 - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp12 - h12 / f_temp2 + f_temp3 /
		f_Sqr(f_temp2) * f_temp12);
	r_t_J(4, 3) = (f_temp9 / f_Sqr(f_temp2) * xix * f_temp17 + f_temp8 / f_Sqr(f_temp2) * xiy *
		f_temp17 - f_temp7 / f_Sqr(f_temp2) * f_temp17) / f_temp6 - f_temp10 / f_Sqr(f_temp6) *
		(-h31 / f_temp2 * xix - f_temp5 / f_Sqr(f_temp2) * xix * f_temp17 - f_temp4 / f_Sqr(f_temp2) *
		xiy * f_temp17 + h11 / f_temp2 + f_temp3 / f_Sqr(f_temp2) * f_temp17);
	r_t_J(5, 3) = (h31 / f_temp2 * xix + f_temp9 / f_Sqr(f_temp2) * xix * f_temp4 + f_temp8 /
		f_Sqr(f_temp2) * xiy * f_temp4 - h11 / f_temp2 - f_temp7 / f_Sqr(f_temp2) * f_temp4) /
		f_temp6 - f_temp10 / f_Sqr(f_temp6) * (-f_temp5 / f_Sqr(f_temp2) * xix * f_temp4 -
		f_Sqr(f_temp4 / f_temp2) * xiy + f_temp3 / f_Sqr(f_temp2) * f_temp4);
	r_t_J(6, 3) = (h23 / f_temp2 * xix + f_temp9 / f_Sqr(f_temp2) * xix * f_temp16 - h13 /
		f_temp2 * xiy + f_temp8 / f_Sqr(f_temp2) * xiy * f_temp16 - f_temp7 / f_Sqr(f_temp2) *
		f_temp16) / f_temp6 - f_temp10 / f_Sqr(f_temp6) * (-h22 / f_temp2 * xix - f_temp5 /
		f_Sqr(f_temp2) * xix * f_temp16 + h12 / f_temp2 * xiy - f_temp4 / f_Sqr(f_temp2) * xiy *
		f_temp16 + f_temp3 / f_Sqr(f_temp2) * f_temp16);
	r_t_J(7, 3) = (f_temp9 / f_Sqr(f_temp2) * xix * f_temp7 + f_temp8 / f_Sqr(f_temp2) * xiy *
		f_temp7 - f_Sqr(f_temp7 / f_temp2)) / f_temp6 - f_temp10 / f_Sqr(f_temp6) *
		(h21 / f_temp2 * xix - f_temp5 / f_Sqr(f_temp2) * xix * f_temp7 - h11 / f_temp2 * xiy -
		f_temp4 / f_Sqr(f_temp2) * xiy * f_temp7 + f_temp3 / f_Sqr(f_temp2) * f_temp7);
	r_t_J(8, 3) = (-h21 / f_temp2 * xix + f_temp9 / f_Sqr(f_temp2) * xix * f_temp1 + h11 /
		f_temp2 * xiy + f_temp8 / f_Sqr(f_temp2) * xiy * f_temp1 - f_temp7 / f_Sqr(f_temp2) *
		f_temp1) / f_temp6 - f_temp10 / f_Sqr(f_temp6) * (-f_temp5 / f_Sqr(f_temp2) * xix *
		f_temp1 - f_temp4 / f_Sqr(f_temp2) * xiy * f_temp1 + f_temp3 / f_Sqr(f_temp2) * f_temp1);
#ifdef TEST_SYM_TRANSFER_JAC
	_TyJacobian J_Simple = r_t_J;
	r_t_J = J_ChainRule; // put that one back
#endif // TEST_SYM_TRANSFER_JAC

	v_error = v_Error(r_v_image, r_v_template, r_t_H);
	// did not bother to simplify it but could be easily done from the above
#endif // !SYM_TRANSFER_USE_CHAIN_RULE || TEST_SYM_TRANSFER_JAC

#ifdef TEST_SYM_TRANSFER_JAC
	double f_jac_error = (J_Simple - J_ChainRule).norm();
	_ASSERTE(!(f_jac_error >= 1e-10)); // ignore NaNs

	double f_res_error = (v_error - v_Error(r_v_image, r_v_template, r_t_H)).norm();
	_ASSERTE(!(f_res_error >= 1e-10)); // ignore NaNs
#endif // TEST_SYM_TRANSFER_JAC

	return v_error;
}

/*
 *								=== ~CHomography::CSymTransferErrorObjective ===
 */

/*
 *								=== CHomography::CSampsonErrorObjective ===
 */

/////////////////////////////////////////////////////////////////////////////////
// 
//  Non-linear, robust homography estimation
//  Copyright (C) 2003-11  Manolis Lourakis (lourakis **at** ics.forth.gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
/////////////////////////////////////////////////////////////////////////////////

#ifdef TEST_SAMPSON_JAC
void CHomography::CSampsonErrorObjective::calc2DHomogSampsonErr(const double m1[2],
	const double m2[2], const double h[9], double err[1])
{
	double t1, t2, t3, t4, t6, t7, t8, t10, t12, t13, t14, t15, t17, t18, t19, t20, t21, t22, t23, t24, t26,
		t27, t28, t29, t30, t31, t33, t34, t35, t36, t37, t39, t40, t41, t42, t43, t44, t45, t46, t47, t49,
		t51, t57, t65, t66, t68, t69, t72, t78, t86, t87, t90, t95, t100, t104, t108, t112, t118, t122, t125,
		t126, t129, t139, t141, t144, t150, t153, t161, t167, t174, t193, t199, t201, t202, t213, t219, t220,
		t222, t225, t236, t243, t250, t253, t260, t271, t273, t296, t303, t317, t331, t335, t339, t342, t345,
		t350, t354, t361, t365, t374;
	t1 = m2[0]; t2 = h[6]; t3 = t2*t1; t4 = m1[0]; t6 = h[7]; t7 = t1*t6; t8 = m1[1]; t10 = h[8]; t12 = h[0];
	t13 = t12*t4; t14 = h[1]; t15 = t14*t8; t17 = t3*t4+t7*t8+t1*t10-t13-t15-h[2]; t18 = m2[1]; t19 = t18*t18;
	t20 = t2*t2; t21 = t19*t20; t22 = t18*t2; t23 = h[3]; t24 = t23*t22; t26 = t23*t23; t27 = t6*t6;
	t28 = t19*t27; t29 = t18*t6; t30 = h[4]; t31 = t29*t30; t33 = t30*t30; t34 = t4*t4; t35 = t20*t34;
	t36 = t2*t4; t37 = t6*t8; t39 = 2.0*t36*t37; t40 = t36*t10; t41 = 2.0*t40; t42 = t8*t8; t43 = t42*t27;
	t44 = t37*t10; t45 = 2.0*t44; t46 = t10*t10; t47 = t21-2.0*t24+t26+t28-2.0*t31+t33+t35+t39+t41+t43+t45+t46;
	t49 = t12*t12; t51 = t6*t30; t57 = t20*t2; t65 = t1*t1; t66 = t65*t20; t68 = t65*t57; t69 = t4*t10;
	t72 = t2*t49; t78 = t27*t6; t86 = t65*t78; t87 = t8*t10; t90 = t65*t27; t95 = -2.0*t49*t18*t51-2.0*t3*t12*
		t46-2.0*t1*t57*t12*t34-2.0*t3*t12*t33+t66*t43+2.0*t68*t69+2.0*t72*t69-2.0*t7*t14*t46-2.0*t1*t78*t14*
		t42-2.0*t7*t14*t26+2.0*t86*t87+t90*t35+2.0*t49*t6*t87; t100 = t14*t14; t104 = t100*t2; t108 = t2*t23;
	t112 = t78*t42*t8; t118 = t57*t34*t4; t122 = t10*t26; t125 = t57*t4; t126 = t10*t19; t129 = t78*t8;
	t139 = -2.0*t57*t34*t18*t23+2.0*t100*t6*t87+2.0*t104*t69-2.0*t100*t18*t108+ 4.0*t36*t112+6.0*t43*t35+4.0*
		t118*t37+t35*t28+2.0*t36*t122+2.0*t125*t126+2.0*t129*t126+2.0*t37*t122-2.0*t78*t42*t18*t30+t43*t21;
	t141 = t10*t33; t144 = t46*t18; t150 = t46*t19; t153 = t46*t10; t161 = t27*t27; t167 = 2.0*t36*t141-2.0*
		t144*t108+2.0*t37*t141+t66*t33+t150*t27+t150*t20+4.0*t37*t153+6.0*t43*t46+4.0*t112*t10+t43*t33+t161*
		t42*t19+t43*t26+4.0*t36*t153; t174 = t20*t20; t193 = 6.0*t35*t46+4.0*t10*t118+t35*t33+t35*t26+t174*
		t34*t19+t100*t27*t42+t100*t20*t34+t100*t19*t20+t90*t46+t65*t161*t42+t90*t26+t49*t27*t42+t49*t20*t34+
		t49*t19*t27; t199 = t34*t34; t201 = t12*t23; t202 = t14*t30; t213 = t42*t42; t219 = t66*t46+t100*t26+
		t46*t100+t174*t199-2.0*t201*t202-2.0*t144*t51+t46*t26+t65*t174*t34+t49*t33+t49*t46+t46*t33+t161*t213-
		2.0*t7*t14*t20*t34; t220 = t1*t27; t222 = t36*t8; t225 = t7*t14; t236 = t4*t6*t8; t243 = t3*t12;
	t250 = t46*t46; t253 = t1*t20; t260 = -4.0*t220*t14*t222-4.0*t225*t40-4.0*t220*t15*t10+2.0*t90*t40+2.0*
		t225*t24+2.0*t72*t236-2.0*t3*t12*t27*t42-4.0*t243*t44+2.0*t66*t44+2.0*t243*t31+t250+2.0*t68*t236-4.0*
		t253*t12*t236-4.0*t253*t13*t10; t271 = t4*t20; t273 = t8*t18; t296 = t10*t18; t303 = 2.0*t104*t236-
		2.0*t35*t31+12.0*t35*t44+2.0*t125*t37*t19-4.0*t271*t6*t273*t23+2.0*t36*t37*t26+2.0*t36*t129*t19-4.0*
		t36*t27*t273*t30+2.0*t36*t37*t33+12.0*t36*t43*t10+12.0*t36*t37*t46-4.0*t271*t296*t23+2.0*t36*t126*t27;
	t317 = t18*t14; t331 = t14*t2; t335 = t12*t18; t339 = t220*t18; t342 = t7*t30; t345 = t317*t6; t350 = -4.0*
		t31*t40-2.0*t43*t24+2.0*t37*t126*t20-4.0*t44*t24-4.0*t27*t8*t296*t30-2.0*t253*t317*t30-2.0*t65*t2*t23*
		t6*t30+2.0*t3*t23*t14*t30-2.0*t12*t19*t331*t6+2.0*t335*t331*t30-2.0*t201*t339+2.0*t201*t342+2.0*t201*
		t345+2.0*t86*t222; t354 = 1/(t95+t139+t167+t193+t219+t260+t303+t350); t361 = t22*t4+t29*t8+t296-t23*
		t4-t30*t8-h[5]; t365 = t253*t18-t3*t23-t335*t2+t201+t339-t342-t345+t202; t374 = t66-2.0*t243+t49+t90-
		2.0*t225+t100+t35+t39+t41+t43+t45+t46; err[0] = sqrt((t17*t47*t354-t361*t365*t354)*t17+(-t17*t365*t354+
		t361*t374*t354)*t361);
}

void CHomography::CSampsonErrorObjective::calc2DHomogSampsonErrGrads(const double m1[2],
	const double m2[2], const double h[9], double grads[9])
{
	double t1, t2, t3, t4, t6, t7, t8, t10, t12, t13, t14, t15, t17, t18, t19, t20, t21, t22, t23, t24, t26,
		t27, t28, t29, t30, t31, t33, t34, t35, t36, t37, t39, t40, t41, t42, t43, t44, t45, t46, t47, t48,
		t49, t50, t51, t54, t55, t58, t61, t62, t63, t66, t69, t70, t72, t75, t76, t79, t80, t81, t84, t87,
		t88, t91, t94, t95, t96, t99, t100, t103, t106, t108, t113, t114, t118, t119, t123, t126, t129, t130,
		t133, t134, t139, t141, t149, t150, t153, t158, t159, t161, t162, t167, t173, t176, t180, t191, t204,
		t210, t213, t215, t219, t224, t230, t231, t234, t238, t241, t242, t257, t259, t261, t262, t263, t266,
		t269, t272, t276, t277, t280, t283, t287, t290, t299, t302, t303, t306, t307, t310, t313, t314, t318,
		t319, t322, t323, t329, t332, t333, t338, t343, t346, t350, t353, t354, t357, t361, t365, t366, t368,
		t370, t374, t375, t377, t380, t381, t384, t385, t402, t405, t408, t422, t423, t428, t429, t436, t438,
		t440, t448, t452, t484, t501, t512, t514, t516, t524, t528, t559, t566, t567, t589, t596, t598, t634,
		t643, t672, t674, t695, t696, t698, t703, t712, t722, t741, t742, t745, t748, t756, t764, t777, t784,
		t792, t813, t822, t851, t864, t867, t869, t870, t874, t880, t886, t895, t899, t901, t902, t909, t922,
		t943, t946, t983, t1001, t1020, t1030, t1064, t1067, t1071, t1096, t1118, t1137, t1161, t1187, t1190;
	t1 = m2[0]; t2 = h[6]; t3 = t2*t1; t4 = m1[0]; t6 = h[7]; t7 = t1*t6; t8 = m1[1]; t10 = h[8];
	t12 = h[0]; t13 = t12*t4; t14 = h[1]; t15 = t14*t8; t17 = t3*t4+t7*t8+t1*t10-t13-t15-h[2];
	t18 = m2[1]; t19 = t18*t18; t20 = t2*t2; t21 = t19*t20; t22 = t18*t2; t23 = h[3]; t24 = t23*t22;
	t26 = t23*t23; t27 = t6*t6; t28 = t19*t27; t29 = t18*t6; t30 = h[4]; t31 = t29*t30; t33 = t30*t30;
	t34 = t4*t4; t35 = t20*t34; t36 = t2*t4; t37 = t6*t8; t39 = 2.0*t36*t37; t40 = t36*t10; t41 = 2.0*t40;
	t42 = t8*t8; t43 = t42*t27; t44 = t37*t10; t45 = 2.0*t44; t46 = t10*t10;
	t47 = t21-2.0*t24+t26+t28-2.0*t31+t33+t35+t39+t41+t43+t45+t46; t48 = t17*t47; t49 = t12*t12;
	t50 = t2*t49; t51 = t4*t10; t54 = t49*t18; t55 = t6*t30; t58 = t12*t46; t61 = t20*t2; t62 = t1*t61;
	t63 = t12*t34; t66 = t12*t33; t69 = t1*t1; t70 = t69*t20; t72 = t69*t61; t75 = t61*t34; t76 = t18*t23;
	t79 = t14*t14; t80 = t79*t6; t81 = t8*t10; t84 = t79*t2; t87 = t79*t18; t88 = t2*t23; t91 = t14*t46;
	t94 = t27*t6; t95 = t1*t94; t96 = t14*t42; t99 = 2.0*t50*t51-2.0*t54*t55-2.0*t3*t58-2.0*t62*t63-2.0*
		t3*t66+t70*t43+2.0*t72*t51-2.0*t75*t76+2.0*t80*t81+2.0*t84*t51-2.0*t87*t88-2.0*t7*t91-2.0*t95*t96;
	t100 = t14*t26; t103 = t69*t94; t106 = t69*t27; t108 = t49*t6; t113 = t34*t4; t114 = t61*t113;
	t118 = t94*t42; t119 = t18*t30; t123 = t10*t33; t126 = t10*t26; t129 = t61*t4; t130 = t10*t19;
	t133 = t42*t8; t134 = t94*t133; t139 = -2.0*t7*t100+2.0*t103*t81+t106*t35+2.0*t108*t81+6.0*t43*t35+
		4.0*t114*t37+t35*t28-2.0*t118*t119+t43*t21+2.0*t36*t123+2.0*t36*t126+2.0*t129*t130+4.0*t36*t134+
		2.0*t37*t123; t141 = t94*t8; t149 = t12*t23; t150 = t14*t30; t153 = t46*t18; t158 = t20*t20;
	t159 = t34*t34; t161 = t27*t27; t162 = t42*t42; t167 = 2.0*t141*t130+2.0*t37*t126+t79*t26+t46*t26+t46*
		t33-2.0*t149*t150-2.0*t153*t55-2.0*t153*t88+t158*t159+t162*t161+t79*t46+t49*t46+t49*t33;
	t173 = t46*t10; t176 = t46*t19; t180 = t46*t46; t191 = t43*t33+4.0*t134*t10+6.0*t43*t46+4.0*t37*t173+
		t176*t20+t176*t27+t70*t33+t180+t79*t20*t34+t79*t27*t42+t158*t34*t19+t35*t26+t35*t33+4.0*t114*t10;
	t204 = t49*t19; t210 = t7*t14; t213 = t1*t27; t215 = t36*t8; t219 = t14*t20*t34; t224 = 6.0*t35*t46+
		4.0*t36*t173+t43*t26+t161*t42*t19+t69*t158*t34+t70*t46+t204*t27+t49*t20*t34+t49*t27*t42-4.0*t210*
		t40-4.0*t213*t14*t215-2.0*t7*t219+2.0*t210*t24; t230 = t4*t6; t231 = t230*t8; t234 = t3*t12;
	t238 = t12*t27*t42; t241 = t1*t20; t242 = t13*t10; t257 = t79*t19; t259 = t106*t26+2.0*t106*t40+2.0*
		t103*t215+2.0*t50*t231-4.0*t234*t44-2.0*t3*t238-4.0*t242*t241-4.0*t241*t12*t231+2.0*t31*t234+t69*
		t161*t42+2.0*t70*t44+2.0*t72*t231+t106*t46+t257*t20; t261 = t4*t20; t262 = t10*t18; t263 = t262*t23;
	t266 = t46*t37; t269 = t43*t10; t272 = t33*t37; t276 = t8*t18; t277 = t276*t30; t280 = t141*t19;
	t283 = t37*t26; t287 = t276*t23; t290 = t37*t19; t299 = t15*t10; t302 = -4.0*t261*t263+12.0*t36*t266+
		12.0*t36*t269+2.0*t36*t272-4.0*t36*t27*t277+2.0*t36*t280+2.0*t36*t283-4.0*t261*t6*t287+2.0*t129*
		t290+12.0*t35*t44-2.0*t35*t31+2.0*t84*t231-4.0*t213*t299; t303 = t7*t30; t306 = t18*t14;
	t307 = t306*t6; t310 = t213*t18; t313 = t12*t18; t314 = t14*t2; t318 = t23*t14; t319 = t318*t30;
	t322 = t69*t2; t323 = t23*t6; t329 = t306*t30; t332 = t27*t8; t333 = t262*t30; t338 = t130*t20;
	t343 = t130*t27; t346 = t12*t19; t350 = 2.0*t303*t149+2.0*t149*t307-2.0*t310*t149+2.0*t313*t314*t30+
		2.0*t3*t319-2.0*t322*t323*t30-4.0*t44*t24-2.0*t241*t329-4.0*t332*t333-4.0*t31*t40+2.0*t37*t338-
		2.0*t43*t24+2.0*t36*t343-2.0*t346*t314*t6; t353 = t99+t139+t167+t191+t224+t259+t302+t350;
	t354 = 1/t353; t357 = t29*t8; t361 = t22*t4+t357+t262-t23*t4-t30*t8-h[5]; t365 = t241*t18-t3*t23-t313*
		t2+t149+t310-t303-t307+t150; t366 = t361*t365; t368 = t48*t354-t366*t354; t370 = t17*t365;
	t374 = t70-2.0*t234+t49+t106-2.0*t210+t79+t35+t39+t41+t43+t45+t46; t375 = t361*t374; t377 = -t370*t354+
		t375*t354; t380 = sqrt(t368*t17+t377*t361); t381 = 1/t380; t384 = t353*t353; t385 = 1/t384;
	t402 = t12*t2; t405 = t12*t6; t408 = 2.0*t238+2.0*t12*t20*t34+2.0*t346*t27+2.0*t66+2.0*t58-2.0*t3*t33-
		2.0*t62*t34-2.0*t3*t46-4.0*t313*t55+4.0*t402*t51+4.0*t405*t81; t422 = t19*t2; t423 = t14*t6;
	t428 = t23*t1; t429 = t27*t18; t436 = -2.0*t319+2.0*t3*t31-4.0*t241*t231-4.0*t241*t51-2.0*t3*t43-4.0*t3*
		t44+4.0*t402*t231-2.0*t422*t423+2.0*t22*t150-2.0*t428*t429+2.0*t428*t55+2.0*t318*t29;
	t438 = t385*(t408+t436); t440 = -t22+t23; t448 = t4*t365*t354; t452 = -t3+t12; grads[0] = t381*((-t4*
		t47*t354-t48*t438-t361*t440*t354+t366*t438)*t17-t368*t4+(t448-t17*t440*t354+t370*t438+2.0*t361*t452*
		t354-t375*t438)*t361)/2.0; t484 = 2.0*t14*t27*t42+2.0*t219+2.0*t14*t19*t20+2.0*t100+2.0*t91-2.0*t7*
		t26-2.0*t95*t42-2.0*t7*t46-4.0*t306*t88+4.0*t51*t314+4.0*t423*t81; t501 = t23*t30;
	t512 = -2.0*t149*t30+2.0*t7*t24-2.0*t7*t35-4.0*t215*t213-4.0*t7*t40-4.0*t213*t81+4.0*t314*t231-2.0*
		t241*t119+2.0*t3*t501-2.0*t346*t2*t6+2.0*t313*t2*t30+2.0*t149*t29; t514 = t385*(t484+t512);
	t516 = -t29+t30; t524 = t8*t365*t354; t528 = -t7+t14; grads[1] = t381*((-t8*t47*t354-t48*t514-t361*t516*
		t354+t366*t514)*t17-t368*t8+(t524-t17*t516*t354+t370*t514+2.0*t361*t528*t354-t375*t514)*t361)/2.0;
	grads[2] = -t381*t368; t559 = t10*t23; t566 = 2.0*t43*t23+2.0*t35*t23+2.0*t106*t23+2.0*t79*t23+2.0*t46*
		t23-4.0*t318*t7-2.0*t87*t2-2.0*t75*t18+4.0*t36*t559+4.0*t37*t559-2.0*t153*t2; t567 = t12*t14;
	t589 = t1*t12; t596 = -2.0*t567*t30+2.0*t7*t306*t2-4.0*t261*t357+4.0*t36*t37*t23-4.0*t261*t262-2.0*t43*
		t22-4.0*t37*t262*t2-2.0*t322*t55+2.0*t3*t150-2.0*t589*t429+2.0*t589*t55+2.0*t567*t29;
	t598 = t385*(t566+t596); grads[3] = t381*((2.0*t17*t440*t354-t48*t598+t448-t361*t452*t354+t598*t366)*
		t17+(-t17*t452*t354+t370*t598-t4*t374*t354-t375*t598)*t361-t377*t4)/2.0; t634 = t10*t30;
	t643 = 2.0*t70*t30+2.0*t43*t30+2.0*t30*t35+2.0*t49*t30+2.0*t46*t30-4.0*t3*t12*t30-2.0*t54*t6+4.0*t36*
		t634-2.0*t118*t18+4.0*t37*t634-2.0*t153*t6; t672 = -2.0*t149*t14+2.0*t3*t313*t6-2.0*t35*t29-4.0*t36*
		t332*t18+4.0*t36*t37*t30-4.0*t36*t262*t6-4.0*t332*t262-2.0*t241*t306-2.0*t322*t323+2.0*t3*t318+2.0*
		t313*t314+2.0*t149*t7; t674 = t385*(t643+t672); grads[4] = t381*((2.0*t17*t516*t354-t48*t674+t524-
		t361*t528*t354+t366*t674)*t17+(-t17*t528*t354+t370*t674-t8*t374*t354-t375*t674)*t361-t377*t8)/2.0;
	grads[5] = -t381*t377; t695 = t1*t4; t696 = t47*t354; t698 = t2*t34; t703 = t4*t27; t712 = t36*t6;
	t722 = t20*t113; t741 = -4.0*t703*t277-8.0*t36*t263+24.0*t698*t44-4.0*t698*t31-8.0*t712*t287-2.0*t153*
		t23-8.0*t3*t242+2.0*t7*t306*t23+12.0*t722*t10+4.0*t72*t34+2.0*t322*t46+2.0*t50*t34+2.0*t257*t2+2.0*
		t84*t34+4.0*t75*t19-4.0*t589*t44+2.0*t698*t26; t742 = t79*t4; t745 = t8*t26; t748 = t8*t46;
	t756 = t14*t4; t764 = t49*t4; t777 = t4*t94; t784 = 2.0*t742*t37+2.0*t230*t745+12.0*t230*t748+12.0*t703*
		t42*t10+2.0*t698*t33-4.0*t7*t756*t10+12.0*t698*t46-2.0*t87*t23+2.0*t764*t10-2.0*t589*t46-2.0*t589*
		t33+2.0*t51*t33+2.0*t51*t26-4.0*t3*t329+4.0*t777*t133+2.0*t742*t10+6.0*t261*t290; t792 = t8*t19;
	t813 = t4*t8; t822 = 2.0*t176*t2+2.0*t322*t33+2.0*t51*t28+2.0*t777*t792+6.0*t261*t130+12.0*t698*t43+
		12.0*t722*t37+2.0*t698*t28-6.0*t35*t76-8.0*t234*t231+2.0*t43*t422+2.0*t589*t31+2.0*t106*t51+2.0*
		t103*t813+2.0*t764*t37+4.0*t322*t44-2.0*t589*t43; t851 = t8*t33; t864 = 6.0*t70*t231-6.0*t241*t63+
		2.0*t106*t698+2.0*t322*t43+4.0*t4*t173+4.0*t61*t159-4.0*t7*t314*t34-4.0*t213*t756*t8+6.0*t70*t51+
		2.0*t428*t150-2.0*t346*t423+2.0*t313*t150-2.0*t76*t43+2.0*t230*t851-2.0*t69*t23*t55-4.0*t51*t31+4.0*
		t37*t130*t2-4.0*t37*t263; t867 = t385*(t741+t784+t822+t864); t869 = t18*t4; t870 = t365*t354;
	t874 = 2.0*t3*t18-t428-t313; t880 = t368*t1; t886 = t374*t354; t895 = t377*t18; grads[6] = t381*((t695*
		t696+t17*(2.0*t422-2.0*t76+2.0*t698+2.0*t231+2.0*t51)*t354-t48*t867-t869*t870-t361*t874*t354+t366*
		t867)*t17+t880*t4+(-t695*t870-t17*t874*t354+t370*t867+t869*t886+t361*(2.0*t322-2.0*t589+2.0*t698+
		2.0*t231+2.0*t51)*t354-t375*t867)*t361+t895*t4)/2.0; t899 = t1*t8; t901 = t19*t6; t902 = t6*t42;
	t909 = t1*t14; t922 = t27*t133; t943 = t69*t6; t946 = 2.0*t72*t813-2.0*t909*t35+2.0*t84*t813-6.0*t43*
		t119+6.0*t332*t130+2.0*t35*t901+12.0*t35*t902+12.0*t922*t36+2.0*t902*t21-4.0*t241*t13*t8-4.0*t3*
		t405*t42+2.0*t909*t24-8.0*t210*t215+12.0*t902*t46+2.0*t902*t26+2.0*t108*t42+2.0*t943*t26;
	t983 = 4.0*t103*t42+2.0*t943*t46+2.0*t80*t42-2.0*t54*t30+2.0*t49*t8*t10-2.0*t46*t909-2.0*t909*t26+4.0*
		t114*t8+2.0*t79*t8*t10-2.0*t153*t30+2.0*t81*t33-4.0*t909*t40-8.0*t7*t299+2.0*t176*t6+2.0*t204*t6+
		4.0*t118*t19+2.0*t902*t33; t1001 = t1*t30; t1020 = 12.0*t922*t10+6.0*t106*t81-6.0*t213*t96+2.0*t70*
		t902+2.0*t943*t35+2.0*t149*t306-2.0*t501*t322-2.0*t346*t314+2.0*t149*t1001+2.0*t36*t851+12.0*t36*
		t748+2.0*t81*t21-2.0*t35*t119+12.0*t35*t81+2.0*t129*t792+2.0*t36*t745+4.0*t94*t162; t1030 = t7*t18;
	t1064 = -8.0*t37*t333-4.0*t902*t24-8.0*t712*t277+24.0*t36*t902*t10-4.0*t149*t1030-4.0*t81*t24+4.0*t36*
		t130*t6-4.0*t36*t333+6.0*t36*t332*t19-4.0*t261*t287+2.0*t50*t813+2.0*t70*t81+4.0*t8*t173-4.0*t3*t12*
		t8*t10+2.0*t81*t26+6.0*t106*t215+4.0*t943*t40+2.0*t3*t313*t30; t1067 = t385*(t946+t983+t1020+t1064);
	t1071 = 2.0*t1030-t1001-t306; grads[7] = t381*((t899*t696+t17*(2.0*t901-2.0*t119+2.0*t215+2.0*t902+2.0*
		t81)*t354-t48*t1067-t276*t870-t361*t1071*t354+t366*t1067)*t17+t880*t8+(-t899*t870-t17*t1071*t354+
		t370*t1067+t276*t886+t361*(2.0*t943-2.0*t909+2.0*t215+2.0*t902+2.0*t81)*t354-t375*t1067)*t361+t895*
		t8)/2.0; t1096 = 2.0*t36+2.0*t37+2.0*t10; t1118 = 2.0*t108*t8+2.0*t50*t4+2.0*t72*t4+2.0*t272+12.0*
		t35*t10+12.0*t36*t46+2.0*t280+2.0*t283+2.0*t36*t33+2.0*t36*t26+2.0*t129*t19; t1137 = 2.0*t80*t8+2.0*
		t84*t4+2.0*t103*t8+2.0*t343+12.0*t266+2.0*t338+2.0*t70*t10+2.0*t106*t10+12.0*t269+4.0*t173+2.0*t126+
		2.0*t79*t10; t1161 = 4.0*t134-4.0*t3*t405*t8-4.0*t7*t314*t4+24.0*t36*t44-4.0*t37*t24-4.0*t36*t31+
		2.0*t123+2.0*t49*t10+2.0*t70*t37+12.0*t36*t43+12.0*t35*t37; t1187 = -4.0*t213*t15+2.0*t106*t36-4.0*
		t332*t119+2.0*t37*t21+4.0*t114+2.0*t36*t28-4.0*t261*t76-4.0*t241*t13-4.0*t7*t14*t10-4.0*t262*t55-
		4.0*t262*t88-4.0*t3*t12*t10; t1190 = t385*(t1118+t1137+t1161+t1187); grads[8] = t381*((t1*t47*t354+
		t17*t1096*t354-t48*t1190-t18*t365*t354+t366*t1190)*t17+t880+(-t1*t365*t354+t370*t1190+t18*t374*t354+
		t361*t1096*t354-t375*t1190)*t361+t895)/2.0;
}
#endif // TEST_SAMPSON_JAC

template <class CDerived0, class CDerived1, class CDerived2>
CHomography::CSampsonErrorObjective::_TyResidual
	CHomography::CSampsonErrorObjective::v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
	const Eigen::MatrixBase<CDerived1> &r_v_template,
	const Eigen::MatrixBase<CDerived2> &r_t_H)
{
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);

	// todo - this is from Homest, need to cleanroom it to remove the infectious GPL2
	// see P.D. Sampson, "Fitting Conic Sections to ''very scattered'' Data: An Iterative Refinement
	// of the Bookstein Algorithm", CGIP, Vol. 18, Is. 1, pp. 97-108, Jan. 1982.
	const double m1[] = {r_v_template(0), r_v_template(1)}, m2[] = {r_v_image(0), r_v_image(1)}; // m1 should be template
	const double h[] = {r_t_H(0, 0), r_t_H(0, 1), r_t_H(0, 2), r_t_H(1, 0), // h should be row-major
		r_t_H(1, 1), r_t_H(1, 2), r_t_H(2, 0), r_t_H(2, 1), r_t_H(2, 2)};
	double t1 = m2[0], t2 = h[6];
	double t3 = t2 * t1, t4 = m1[0], t6 = h[7];
	double t7 = t1 * t6, t8 = m1[1], t10 = h[8], t12 = h[0];
	double t13 = t12 * t4, t14 = h[1];
	double t15 = t14 * t8;
	double t17 = t3 * t4 + t7 * t8 + t1 * t10 - t13 - t15 - h[2], t18 = m2[1];
	double t19 = t18 * t18, t20 = t2 * t2;
	double t21 = t19 * t20, t22 = t18 * t2, t23 = h[3];
	double t24 = t23 * t22, t26 = t23 * t23, t27 = t6 * t6;
	double t28 = t19 * t27, t29 = t18 * t6, t30 = h[4];
	double t31 = t29 * t30, t33 = t30 * t30, t34 = t4 * t4;
	double t35 = t20 * t34, t36 = t2 * t4, t37 = t6 * t8;
	double t39 = 2 * t36 * t37, t40 = t36 * t10;
	double t41 = 2 * t40, t42 = t8 * t8;
	double t43 = t42 * t27, t44 = t37 * t10;
	double t45 = 2 * t44, t46 = t10 * t10;
	double t47 = t21 - 2 * t24 + t26 + t28 - 2 * t31 + t33 + t35 + t39 + t41 + t43 + t45 + t46;
	double t49 = t12 * t12, t51 = t6 * t30, t57 = t20 * t2, t65 = t1 * t1;
	double t66 = t65 * t20, t68 = t65 * t57, t69 = t4 * t10, t72 = t2 * t49, t78 = t27 * t6;
	double t86 = t65 * t78, t87 = t8 * t10, t90 = t65 * t27;
	double t95 = -2 * t49 * t18 * t51 - 2 * t3 * t12 * t46 - 2 * t1 * t57 * t12 * t34 - 2 * t3 * t12 * t33 + t66 *
		t43 + 2 * t68 * t69 + 2 * t72 * t69 - 2 * t7 * t14 * t46 - 2 * t1 * t78 * t14 * t42 - 2 * t7 * t14 * t26 +
		2 * t86 * t87 + t90 * t35 + 2 * t49 * t6 * t87, t100 = t14 * t14;
	double t104 = t100 * t2, t108 = t2 * t23, t112 = t78 * t42 * t8, t118 = t57 * t34 * t4, t122 = t10 * t26;
	double t125 = t57 * t4, t126 = t10 * t19, t129 = t78 * t8;
	double t139 = -2 * t57 * t34 * t18 * t23 + 2 * t100 * t6 * t87 + 2 * t104 * t69 - 2 * t100 * t18 * t108 +
		4 * t36 * t112 + 6 * t43 * t35 + 4 * t118 * t37 + t35 * t28 + 2 * t36 * t122 + 2 * t125 * t126 + 2 *
		t129 * t126 + 2 * t37 * t122 - 2 * t78 * t42 * t18 * t30 + t43 * t21;
	double t141 = t10 * t33, t144 = t46 * t18, t150 = t46 * t19, t153 = t46 * t10, t161 = t27 * t27;
	double t167 = 2 * t36 * t141 - 2 * t144 * t108 + 2 * t37 * t141 + t66 * t33 + t150 * t27 + t150 * t20 +
		4 * t37 * t153 + 6 * t43 * t46 + 4 * t112 * t10 + t43 * t33 + t161 * t42 * t19 + t43 * t26 + 4 * t36 * t153;
	double t174 = t20 * t20;
	double t193 = 6 * t35 * t46 + 4 * t10 * t118 + t35 * t33 + t35 * t26 + t174 * t34 * t19 + t100 * t27 * t42 +
		t100 * t20 * t34 + t100 * t19 * t20 + t90 * t46 + t65 * t161 * t42 + t90 * t26 + t49 * t27 * t42 + t49 * t20 * t34 +
		t49 * t19 * t27;
	double t199 = t34 * t34, t201 = t12 * t23, t202 = t14 * t30, t213 = t42 * t42;
	double t219 = t66 * t46 + t100 * t26 + t46 * t100 + t174 * t199 - 2 * t201 * t202 - 2 * t144 * t51 + t46 *
		t26 + t65 * t174 * t34 + t49 * t33 + t49 * t46 + t46 * t33 + t161 * t213 - 2 * t7 * t14 * t20 * t34;
	double t220 = t1 * t27, t222 = t36 * t8, t225 = t7 * t14, t236 = t4 * t6 * t8, t243 = t3 * t12;
	double t250 = t46 * t46, t253 = t1 * t20;
	double t260 = -4 * t220 * t14 * t222 - 4 * t225 * t40 - 4 * t220 * t15 * t10 + 2 * t90 * t40 + 2 *
		t225 * t24 + 2 * t72 * t236 - 2 * t3 * t12 * t27 * t42 - 4 * t243 * t44 + 2 * t66 * t44 + 2 * t243 * t31 +
		t250 + 2 * t68 * t236 - 4 * t253 * t12 * t236 - 4 * t253 * t13 * t10;
	double t271 = t4 * t20, t273 = t8 * t18, t296 = t10 * t18;
	double t303 = 2 * t104 * t236 - 2 * t35 * t31 + 12 * t35 * t44 + 2 * t125 * t37 * t19 - 4 * t271 * t6 *
		t273 * t23 + 2 * t36 * t37 * t26 + 2 * t36 * t129 * t19 - 4 * t36 * t27 * t273 * t30 + 2 * t36 * t37 * t33 +
		12 * t36 * t43 * t10 + 12 * t36 * t37 * t46 - 4 * t271 * t296 * t23 + 2 * t36 * t126 * t27;
	double t317 = t18 * t14, t331 = t14 * t2, t335 = t12 * t18, t339 = t220 * t18, t342 = t7 * t30;
	double t345 = t317 * t6;
	double t350 = -4 * t31 * t40 - 2 * t43 * t24 + 2 * t37 * t126 * t20 - 4 * t44 * t24 - 4 * t27 * t8 *
		t296 * t30 - 2 * t253 * t317 * t30 - 2 * t65 * t2 * t23 * t6 * t30 + 2 * t3 * t23 * t14 * t30 - 2 * t12 * t19 *
		t331 * t6 + 2 * t335 * t331 * t30 - 2 * t201 * t339 + 2 * t201 * t342 + 2 * t201 * t345 + 2 * t86 * t222;
	double t354 = 1 / (t95 + t139 + t167 + t193 + t219 + t260 + t303 + t350);
	double t361 = t22 * t4 + t29 * t8 + t296 - t23 * t4 - t30 * t8 - h[5];
	double t365 = t253 * t18 - t3 * t23 - t335 * t2 + t201 + t339 - t342 - t345 + t202;
	double t374 = t66 - 2 * t243 + t49 + t90 - 2 * t225 + t100 + t35 + t39 + t41 + t43 + t45 + t46;
	_TyResidual v_res;
	v_res(0) = (sqrt((t17 * t47 * t354 - t361 * t365 * t354) * t17 + (-t17 * t365 * t354 +
		t361 * t374 * t354) * t361));
#ifdef TEST_SAMPSON_JAC
	_TyResidual v_err;
	calc2DHomogSampsonErr(m1, m2, h, &v_err(0));
	double f_error = fabs((v_res - v_err)(0));
	_ASSERTE(!(f_error >= 1e-10 * std::max(1.0, fabs(v_err(0))))); // ignore NaNs
#endif // TEST_SAMPSON_JAC
	return v_res;
}

template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
CHomography::CSampsonErrorObjective::_TyResidual
	CHomography::CSampsonErrorObjective::v_Error_Jacobian(Eigen::MatrixBase<CDerived0> &r_v_J,
	const Eigen::MatrixBase<CDerived1> &r_v_image,
	const Eigen::MatrixBase<CDerived2> &r_v_template,
	const Eigen::MatrixBase<CDerived3> &r_t_H)
{
	DimensionCheck<_TyJacobian>(r_v_J);
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);

	// todo - this is from Homest, need to cleanroom it to remove the infectious GPL2
	// see P.D. Sampson, "Fitting Conic Sections to ''very scattered'' Data: An Iterative Refinement
	// of the Bookstein Algorithm", CGIP, Vol. 18, Is. 1, pp. 97-108, Jan. 1982.
	const double m1[] = {r_v_template(0), r_v_template(1)}, m2[] = {r_v_image(0), r_v_image(1)}; // m1 should be template
	const double h[] = {r_t_H(0, 0), r_t_H(0, 1), r_t_H(0, 2), r_t_H(1, 0), // h should be row-major
		r_t_H(1, 1), r_t_H(1, 2), r_t_H(2, 0), r_t_H(2, 1), r_t_H(2, 2)};
	double t1 = m2[0], t2 = h[6];
	double t3 = t2 * t1, t4 = m1[0], t6 = h[7];
	double t7 = t1 * t6, t8 = m1[1], t10 = h[8], t12 = h[0];
	double t13 = t12 * t4, t14 = h[1];
	double t15 = t14 * t8;
	double t17 = t3 * t4 + t7 * t8 + t1 * t10 - t13 - t15 - h[2], t18 = m2[1];
	double t19 = t18 * t18, t20 = t2 * t2;
	double t21 = t19 * t20, t22 = t18 * t2, t23 = h[3];
	double t24 = t23 * t22, t26 = t23 * t23, t27 = t6 * t6;
	double t28 = t19 * t27, t29 = t18 * t6, t30 = h[4];
	double t31 = t29 * t30, t33 = t30 * t30, t34 = t4 * t4;
	double t35 = t20 * t34, t36 = t2 * t4, t37 = t6 * t8;
	double t39 = 2 * t36 * t37, t40 = t36 * t10;
	double t41 = 2 * t40, t42 = t8 * t8;
	double t43 = t42 * t27, t44 = t37 * t10;
	double t45 = 2 * t44, t46 = t10 * t10;
	double t47 = t21 - 2 * t24 + t26 + t28 - 2 * t31 + t33 + t35 + t39 + t41 + t43 + t45 + t46;
	double t48 = t17 * t47, t49 = t12 * t12;
	double t50 = t2 * t49, t51 = t4 * t10, t54 = t49 * t18, t55 = t6 * t30, t58 = t12 * t46, t61 = t20 * t2;
	double t62 = t1 * t61, t63 = t12 * t34, t66 = t12 * t33, t69 = t1 * t1;
	double t70 = t69 * t20, t72 = t69 * t61, t75 = t61 * t34, t76 = t18 * t23, t79 = t14 * t14;
	double t80 = t79 * t6, t81 = t8 * t10, t84 = t79 * t2, t87 = t79 * t18;
	double t88 = t2 * t23, t91 = t14 * t46, t94 = t27 * t6;
	double t95 = t1 * t94, t96 = t14 * t42;
	double t99 = 2 * t50 * t51 - 2 * t54 * t55 - 2 * t3 * t58 - 2 * t62 * t63 - 2 * t3 * t66 + t70 * t43 + 2 *
		t72 * t51 - 2 * t75 * t76 + 2 * t80 * t81 + 2 * t84 * t51 - 2 * t87 * t88 - 2 * t7 * t91 - 2 * t95 * t96;
	double t100 = t14 * t26, t103 = t69 * t94, t106 = t69 * t27, t108 = t49 * t6, t113 = t34 * t4;
	double t114 = t61 * t113, t118 = t94 * t42, t119 = t18 * t30, t123 = t10 * t33, t126 = t10 * t26;
	double t129 = t61 * t4, t130 = t10 * t19, t133 = t42 * t8;
	double t134 = t94 * t133;
	double t139 = -2 * t7 * t100 + 2 * t103 * t81 + t106 * t35 + 2 * t108 * t81 + 6 * t43 * t35 + 4 * t114 *
		t37 + t35 * t28 - 2 * t118 * t119 + t43 * t21 + 2 * t36 * t123 + 2 * t36 * t126 + 2 * t129 * t130 + 4 *
		t36 * t134 + 2 * t37 * t123;
	double t141 = t94 * t8, t149 = t12 * t23, t150 = t14 * t30, t153 = t46 * t18, t158 = t20 * t20;
	double t159 = t34 * t34, t161 = t27 * t27, t162 = t42 * t42;
	double t167 = 2 * t141 * t130 + 2 * t37 * t126 + t79 * t26 + t46 * t26 + t46 * t33 - 2 * t149 * t150 - 2 *
		t153 * t55 - 2 * t153 * t88 + t158 * t159 + t162 * t161 + t79 * t46 + t49 * t46 + t49 * t33;
	double t173 = t46 * t10, t176 = t46 * t19, t180 = t46 * t46;
	double t191 = t43 * t33 + 4 * t134 * t10 + 6 * t43 * t46 + 4 * t37 * t173 + t176 * t20 + t176 * t27 + t70 *
		t33 + t180 + t79 * t20 * t34 + t79 * t27 * t42 + t158 * t34 * t19 + t35 * t26 + t35 * t33 + 4 * t114 * t10;
	double t204 = t49 * t19, t210 = t7 * t14, t213 = t1 * t27, t215 = t36 * t8, t219 = t14 * t20 * t34;
	double t224 = 6 * t35 * t46 + 4 * t36 * t173 + t43 * t26 + t161 * t42 * t19 + t69 * t158 * t34 + t70 * t46 +
		t204 * t27 + t49 * t20 * t34 + t49 * t27 * t42 - 4 * t210 * t40 - 4 * t213 * t14 * t215 - 2 * t7 * t219 + 2 *
		t210 * t24, t230 = t4 * t6;
	double t231 = t230 * t8, t234 = t3 * t12, t238 = t12 * t27 * t42, t241 = t1 * t20, t242 = t13 * t10, t257 = t79 * t19;
	double t259 = t106 * t26 + 2 * t106 * t40 + 2 * t103 * t215 + 2 * t50 * t231 - 4 * t234 * t44 - 2 * t3 *
		t238 - 4 * t242 * t241 - 4 * t241 * t12 * t231 + 2 * t31 * t234 + t69 * t161 * t42 + 2 * t70 * t44 + 2 *
		t72 * t231 + t106 * t46 + t257 * t20;
	double t261 = t4 * t20, t262 = t10 * t18;
	double t263 = t262 * t23, t266 = t46 * t37, t269 = t43 * t10, t272 = t33 * t37, t276 = t8 * t18;
	double t277 = t276 * t30, t280 = t141 * t19, t283 = t37 * t26, t287 = t276 * t23, t290 = t37 * t19, t299 = t15 * t10;
	double t302 = -4 * t261 * t263 + 12 * t36 * t266 + 12 * t36 * t269 + 2 * t36 * t272 - 4 * t36 * t27 *
		t277 + 2 * t36 * t280 + 2 * t36 * t283 - 4 * t261 * t6 * t287 + 2 * t129 * t290 + 12 * t35 * t44 - 2 *
		t35 * t31 + 2 * t84 * t231 - 4 * t213 * t299;
	double t303 = t7 * t30, t306 = t18 * t14;
	double t307 = t306 * t6, t310 = t213 * t18, t313 = t12 * t18, t314 = t14 * t2, t318 = t23 * t14;
	double t319 = t318 * t30, t322 = t69 * t2, t323 = t23 * t6, t329 = t306 * t30, t332 = t27 * t8;
	double t333 = t262 * t30, t338 = t130 * t20, t343 = t130 * t27, t346 = t12 * t19;
	double t350 = 2 * t303 * t149 + 2 * t149 * t307 - 2 * t310 * t149 + 2 * t313 * t314 * t30 + 2 * t3 *
		t319 - 2 * t322 * t323 * t30 - 4 * t44 * t24 - 2 * t241 * t329 - 4 * t332 * t333 - 4 * t31 * t40 + 2 *
		t37 * t338 - 2 * t43 * t24 + 2 * t36 * t343 - 2 * t346 * t314 * t6;
	double t353 = t99 + t139 + t167 + t191 + t224 + t259 + t302 + t350;
	double t354 = 1 / t353, t357 = t29 * t8;
	double t361 = t22 * t4 + t357 + t262 - t23 * t4 - t30 * t8 - h[5];
	double t365 = t241 * t18 - t3 * t23 - t313 * t2 + t149 + t310 - t303 - t307 + t150;
	double t366 = t361 * t365;
	double t368 = t48 * t354 - t366 * t354, t370 = t17 * t365;
	double t374 = t70 - 2 * t234 + t49 + t106 - 2 * t210 + t79 + t35 + t39 + t41 + t43 + t45 + t46;
	double t375 = t361 * t374;
	double t377 = -t370 * t354 + t375 * t354;
	double t380 = sqrt(t368 * t17 + t377 * t361);
	double t381 = 1 / t380, t384 = t353 * t353;
	double t385 = 1 / t384, t402 = t12 * t2, t405 = t12 * t6;
	double t408 = 2 * t238 + 2 * t12 * t20 * t34 + 2 * t346 * t27 + 2 * t66 + 2 * t58 - 2 * t3 * t33 - 2 *
		t62 * t34 - 2 * t3 * t46 - 4 * t313 * t55 + 4 * t402 * t51 + 4 * t405 * t81;
	double t422 = t19 * t2, t423 = t14 * t6, t428 = t23 * t1, t429 = t27 * t18;
	double t436 = -2 * t319 + 2 * t3 * t31 - 4 * t241 * t231 - 4 * t241 * t51 - 2 * t3 * t43 - 4 * t3 *
		t44 + 4 * t402 * t231 - 2 * t422 * t423 + 2 * t22 * t150 - 2 * t428 * t429 + 2 * t428 * t55 + 2 *
		t318 * t29;
	double t438 = t385 * (t408 + t436), t440 = -t22 + t23, t448 = t4 * t365 * t354, t452 = -t3 + t12;
	r_v_J(0) = -t381 * ((-t4 * t47 * t354 - t48 * t438 - t361 * t440 * t354 + t366 * t438) * t17 - t368 *
		t4 + (t448 - t17 * t440 * t354 + t370 * t438 + 2 * t361 * t452 * t354 - t375 * t438) * t361) / 2;
	double t484 = 2 * t14 * t27 * t42 + 2 * t219 + 2 * t14 * t19 * t20 + 2 * t100 + 2 * t91 - 2 * t7 * t26 -
		2 * t95 * t42 - 2 * t7 * t46 - 4 * t306 * t88 + 4 * t51 * t314 + 4 * t423 * t81, t501 = t23 * t30;
	double t512 = -2 * t149 * t30 + 2 * t7 * t24 - 2 * t7 * t35 - 4 * t215 * t213 - 4 * t7 * t40 - 4 *
		t213 * t81 + 4 * t314 * t231 - 2 * t241 * t119 + 2 * t3 * t501 - 2 * t346 * t2 * t6 + 2 * t313 * t2 * t30 +
		2 * t149 * t29;
	double t514 = t385 * (t484 + t512), t516 = -t29 + t30, t524 = t8 * t365 * t354, t528 = -t7 + t14;
	r_v_J(1) = -t381 * ((-t8 * t47 * t354 - t48 * t514 - t361 * t516 * t354 + t366 * t514) * t17 - t368 *
		t8 + (t524 - t17 * t516 * t354 + t370 * t514 + 2 * t361 * t528 * t354 - t375 * t514) * t361) / 2;
	r_v_J(2) = t381 * t368;
	double t559 = t10 * t23, t567 = t12 * t14, t589 = t1 * t12;
	double t566 = 2 * t43 * t23 + 2 * t35 * t23 + 2 * t106 * t23 + 2 * t79 * t23 + 2 * t46 * t23 - 4 *
		t318 * t7 - 2 * t87 * t2 - 2 * t75 * t18 + 4 * t36 * t559 + 4 * t37 * t559 - 2 * t153 * t2;
	double t596 = -2 * t567 * t30 + 2 * t7 * t306 * t2 - 4 * t261 * t357 + 4 * t36 * t37 * t23 - 4 * t261 *
		t262 - 2 * t43 * t22 - 4 * t37 * t262 * t2 - 2 * t322 * t55 + 2 * t3 * t150 - 2 * t589 * t429 + 2 *
		t589 * t55 + 2 * t567 * t29;
	double t598 = t385 * (t566 + t596), t634 = t10 * t30;
	r_v_J(3) = -t381 * ((2 * t17 * t440 * t354 - t48 * t598 + t448 - t361 * t452 * t354 + t598 * t366)
		* t17 + (-t17 * t452 * t354 + t370 * t598 - t4 * t374 * t354 - t375 * t598) * t361 - t377 * t4) / 2;
	double t643 = 2 * t70 * t30 + 2 * t43 * t30 + 2 * t30 * t35 + 2 * t49 * t30 + 2 * t46 * t30 - 4 * t3 *
		t12 * t30 - 2 * t54 * t6 + 4 * t36 * t634 - 2 * t118 * t18 + 4 * t37 * t634 - 2 * t153 * t6;
	double t672 = -2 * t149 * t14 + 2 * t3 * t313 * t6 - 2 * t35 * t29 - 4 * t36 * t332 * t18 + 4 * t36 *
		t37 * t30 - 4 * t36 * t262 * t6 - 4 * t332 * t262 - 2 * t241 * t306 - 2 * t322 * t323 + 2 * t3 * t318 +
		2 * t313 * t314 + 2 * t149 * t7;
	double t674 = t385 * (t643 + t672);
	r_v_J(4) = -t381 * ((2 * t17 * t516 * t354 - t48 * t674 + t524 - t361 * t528 * t354 + t366 * t674)
		* t17 + (-t17 * t528 * t354 + t370 * t674 - t8 * t374 * t354 - t375 * t674) * t361 - t377 * t8) / 2;
	r_v_J(5) = t381 * t377;
	double t695 = t1 * t4, t696 = t47 * t354, t698 = t2 * t34, t703 = t4 * t27, t712 = t36 * t6, t722 = t20 * t113;
	double t741 = -4 * t703 * t277 - 8 * t36 * t263 + 24 * t698 * t44 - 4 * t698 * t31 - 8 * t712 * t287 -
		2 * t153 * t23 - 8 * t3 * t242 + 2 * t7 * t306 * t23 + 12 * t722 * t10 + 4 * t72 * t34 + 2 * t322 *
		t46 + 2 * t50 * t34 + 2 * t257 * t2 + 2 * t84 * t34 + 4 * t75 * t19 - 4 * t589 * t44 + 2 * t698 * t26;
	double t742 = t79 * t4, t745 = t8 * t26, t748 = t8 * t46, t756 = t14 * t4, t764 = t49 * t4, t777 = t4 * t94;
	double t784 = 2 * t742 * t37 + 2 * t230 * t745 + 12 * t230 * t748 + 12 * t703 * t42 * t10 + 2 * t698 *
		t33 - 4 * t7 * t756 * t10 + 12 * t698 * t46 - 2 * t87 * t23 + 2 * t764 * t10 - 2 * t589 * t46 - 2 *
		t589 * t33 + 2 * t51 * t33 + 2 * t51 * t26 - 4 * t3 * t329 + 4 * t777 * t133 + 2 * t742 * t10 + 6 *
		t261 * t290, t792 = t8 * t19, t813 = t4 * t8;
	double t822 = 2 * t176 * t2 + 2 * t322 * t33 + 2 * t51 * t28 + 2 * t777 * t792 + 6 * t261 * t130 +
		12 * t698 * t43 + 12 * t722 * t37 + 2 * t698 * t28 - 6 * t35 * t76 - 8 * t234 * t231 + 2 * t43 * t422 +
		2 * t589 * t31 + 2 * t106 * t51 + 2 * t103 * t813 + 2 * t764 * t37 + 4 * t322 * t44 - 2 * t589 * t43;
	double t851 = t8 * t33;
	double t864 = 6 * t70 * t231 - 6 * t241 * t63 + 2 * t106 * t698 + 2 * t322 * t43 + 4 * t4 * t173 + 4 *
		t61 * t159 - 4 * t7 * t314 * t34 - 4 * t213 * t756 * t8 + 6 * t70 * t51 + 2 * t428 * t150 - 2 * t346 *
		t423 + 2 * t313 * t150 - 2 * t76 * t43 + 2 * t230 * t851 - 2 * t69 * t23 * t55 - 4 * t51 * t31 + 4 *
		t37 * t130 * t2 - 4 * t37 * t263;
	double t867 = t385 * (t741 + t784 + t822 + t864), t869 = t18 * t4, t870 = t365 * t354;
	double t874 = 2 * t3 * t18 - t428 - t313, t880 = t368 * t1, t886 = t374 * t354, t895 = t377 * t18;
	r_v_J(6) = -t381 * ((t695 * t696 + t17 * (2 * t422 - 2 * t76 + 2 * t698 + 2 * t231 + 2 * t51) *
		t354 - t48 * t867 - t869 * t870 - t361 * t874 * t354 + t366 * t867) * t17 + t880 * t4 + (-t695 * t870 - t17 *
		t874 * t354 + t370 * t867 + t869 * t886 + t361 * (2 * t322 - 2 * t589 + 2 * t698 + 2 * t231 + 2 *
		t51) * t354 - t375 * t867) * t361 + t895 * t4) / 2;
	double t899 = t1 * t8, t901 = t19 * t6, t902 = t6 * t42, t909 = t1 * t14, t922 = t27 * t133, t943 = t69 * t6;
	double t946 = 2 * t72 * t813 - 2 * t909 * t35 + 2 * t84 * t813 - 6 * t43 * t119 + 6 * t332 * t130 +
		2 * t35 * t901 + 12 * t35 * t902 + 12 * t922 * t36 + 2 * t902 * t21 - 4 * t241 * t13 * t8 - 4 * t3 *
		t405 * t42 + 2 * t909 * t24 - 8 * t210 * t215 + 12 * t902 * t46 + 2 * t902 * t26 + 2 * t108 * t42 + 2 *
		t943 * t26;
	double t983 = 4 * t103 * t42 + 2 * t943 * t46 + 2 * t80 * t42 - 2 * t54 * t30 + 2 * t49 * t8 * t10 - 2 *
		t46 * t909 - 2 * t909 * t26 + 4 * t114 * t8 + 2 * t79 * t8 * t10 - 2 * t153 * t30 + 2 * t81 * t33 - 4 *
		t909 * t40 - 8 * t7 * t299 + 2 * t176 * t6 + 2 * t204 * t6 + 4 * t118 * t19 + 2 * t902 * t33;
	double t1001 = t1 * t30;
	double t1020 = 12 * t922 * t10 + 6 * t106 * t81 - 6 * t213 * t96 + 2 * t70 * t902 + 2 * t943 * t35 +
		2 * t149 * t306 - 2 * t501 * t322 - 2 * t346 * t314 + 2 * t149 * t1001 + 2 * t36 * t851 + 12 * t36 *
		t748 + 2 * t81 * t21 - 2 * t35 * t119 + 12 * t35 * t81 + 2 * t129 * t792 + 2 * t36 * t745 + 4 * t94 *
		t162, t1030 = t7 * t18;
	double t1064 = -8 * t37 * t333 - 4 * t902 * t24 - 8 * t712 * t277 + 24 * t36 * t902 * t10 - 4 * t149 *
		t1030 - 4 * t81 * t24 + 4 * t36 * t130 * t6 - 4 * t36 * t333 + 6 * t36 * t332 * t19 - 4 * t261 * t287 +
		2 * t50 * t813 + 2 * t70 * t81 + 4 * t8 * t173 - 4 * t3 * t12 * t8 * t10 + 2 * t81 * t26 + 6 * t106 *
		t215 + 4 * t943 * t40 + 2 * t3 * t313 * t30;
	double t1067 = t385 * (t946 + t983 + t1020 + t1064), t1071 = 2 * t1030 - t1001 - t306;
	r_v_J(7) = -t381 * ((t899 * t696 + t17 * (2 * t901 - 2 * t119 + 2 * t215 + 2 * t902 + 2 *
		t81) * t354 - t48 * t1067 - t276 * t870 - t361 * t1071 * t354 + t366 * t1067) * t17 + t880 * t8 + (-t899 *
		t870 - t17 * t1071 * t354 + t370 * t1067 + t276 * t886 + t361 * (2 * t943 - 2 * t909 + 2 * t215 + 2 *
		t902 + 2 * t81) * t354 - t375 * t1067) * t361 + t895 * t8) / 2;
	double t1096 = 2 * t36 + 2 * t37 + 2 * t10;
	double t1118 = 2 * t108 * t8 + 2 * t50 * t4 + 2 * t72 * t4 + 2 * t272 + 12 * t35 * t10 + 12 * t36 *
		t46 + 2 * t280 + 2 * t283 + 2 * t36 * t33 + 2 * t36 * t26 + 2 * t129 * t19;
	double t1137 = 2 * t80 * t8 + 2 * t84 * t4 + 2 * t103 * t8 + 2 * t343 + 12 * t266 + 2 * t338 + 2 *
		t70 * t10 + 2 * t106 * t10 + 12 * t269 + 4 * t173 + 2 * t126 + 2 * t79 * t10;
	double t1161 = 4 * t134 - 4 * t3 * t405 * t8 - 4 * t7 * t314 * t4 + 24 * t36 * t44 - 4 * t37 * t24 - 4 *
		t36 * t31 + 2 * t123 + 2 * t49 * t10 + 2 * t70 * t37 + 12 * t36 * t43 + 12 * t35 * t37;
	double t1187 = -4 * t213 * t15 + 2 * t106 * t36 - 4 * t332 * t119 + 2 * t37 * t21 + 4 * t114 + 2 *
		t36 * t28 - 4 * t261 * t76 - 4 * t241 * t13 - 4 * t7 * t14 * t10 - 4 * t262 * t55 - 4 * t262 * t88 - 4 *
		t3 * t12 * t10;
	double t1190 = t385 * (t1118 + t1137 + t1161 + t1187);
	r_v_J(8) = -t381 * ((t1 * t47 * t354 + t17 * t1096 * t354 - t48 * t1190 - t18 * t365 * t354 + t366 * t1190) *
		t17 + t880 + (-t1 * t365 * t354 + t370 * t1190 + t18 * t374 * t354 + t361 * t1096 * t354 - t375 *
		t1190) * t361 + t895) / 2;
#ifdef TEST_SAMPSON_JAC
	double grads[9];
	calc2DHomogSampsonErrGrads(m1, m2, h, grads);
	_TyJacobian J;
	for(int i = 0; i < 9; ++ i)
		J(i) = -grads[i];
	double f_error = (r_v_J - J).norm();
	_ASSERTE(!(f_error >= 1e-10 * std::max(1.0, J.lpNorm<Eigen::Infinity>()))); // ignore NaNs
#endif // TEST_SAMPSON_JAC

	return v_Error(r_v_image, r_v_template, r_t_H); // todo - eliminate common subexpressions
}

/*
 *								=== ~CHomography::CSampsonErrorObjective ===
 */

/*
 *								=== CHomography::CReprojectionErrorObjective ===
 */

template <class CDerived0, class CDerived1, class CDerived2, class CDerived3>
inline CHomography::CReprojectionErrorObjective::_TyResidual
	CHomography::CReprojectionErrorObjective::v_Error(const Eigen::MatrixBase<CDerived0> &r_v_image,
	const Eigen::MatrixBase<CDerived1> &r_v_template,
	const Eigen::MatrixBase<CDerived2> &r_v_template_opt,
	const Eigen::MatrixBase<CDerived3> &r_t_H)
{
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Vector2d>(r_v_template_opt);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);

	Eigen::Vector3d v_repr = r_t_H * Eigen::Vector3d(r_v_template_opt(0), r_v_template_opt(1), 1);
	Eigen::Vector4d v_error;
	v_error.head<2>() = r_v_image - v_repr.head<2>() / v_repr(2);
	v_error.tail<2>() = r_v_template - r_v_template_opt;

	return v_error;
}

template <class CDerived0, class CDerived1, class CDerived2,
	class CDerived3, class CDerived4, class CDerived5>
inline CHomography::CReprojectionErrorObjective::_TyResidual
	CHomography::CReprojectionErrorObjective::v_Error_Jacobians(Eigen::MatrixBase<CDerived0> &r_t_J_H,
	Eigen::MatrixBase<CDerived1> &r_t_J_T,
	const Eigen::MatrixBase<CDerived2> &r_v_image,
	const Eigen::MatrixBase<CDerived3> &r_v_template,
	const Eigen::MatrixBase<CDerived4> &r_v_template_opt,
	const Eigen::MatrixBase<CDerived5> &r_t_H)
{
	DimensionCheck<_TyJacobian_H>(r_t_J_H);
	DimensionCheck<_TyJacobian_T>(r_t_J_T);
	DimensionCheck<Eigen::Vector2d>(r_v_image);
	DimensionCheck<Eigen::Vector2d>(r_v_template);
	DimensionCheck<Eigen::Vector2d>(r_v_template_opt);
	DimensionCheck<Eigen::Matrix3d>(r_t_H);

	Eigen::Vector3d v_repr = r_t_H * Eigen::Vector3d(r_v_template_opt(0), r_v_template_opt(1), 1);
	Eigen::Vector4d v_error;
	v_error.head<2>() = r_v_image - v_repr.head<2>() / v_repr(2);
	v_error.tail<2>() = r_v_template - r_v_template_opt;
	// repeated code but need v_repr below

	const double /*Imx = r_v_image(0), Imy = r_v_image(1), Tpx = r_v_template(0),
		 Tpy = r_v_template(1),*/ TpOx = r_v_template_opt(0), TpOy = r_v_template_opt(1),
		 h11 = r_t_H(0, 0), h12 = r_t_H(0, 1), h13 = r_t_H(0, 2), h21 = r_t_H(1, 0), h22 = r_t_H(1, 1),
		 h23 = r_t_H(1, 2), h31 = r_t_H(2, 0), h32 = r_t_H(2, 1), h33 = r_t_H(2, 2);

	double f_temp0 = -v_repr(0); //_ASSERTE(fabs(-f_temp0 - (h11*TpOx+h12*TpOy+h13)) < 1e-10);
	double f_temp1 = -v_repr(1); //_ASSERTE(fabs(-f_temp1 - (h21*TpOx+h22*TpOy+h23)) < 1e-10);
	double f_temp2 = v_repr(2); //_ASSERTE(fabs(f_temp2 - (h31*TpOx+h32*TpOy+h33)) < 1e-10);
	double f_temp3 = f_temp2 * f_temp2; //_ASSERTE(fabs(f_temp3 - pow(h31*TpOx+h32*TpOy+h33,2.0)) < 1e-10);
	r_t_J_H(0, 0) = TpOx / f_temp2;
	r_t_J_H(1, 0) = TpOy / f_temp2;
	r_t_J_H(2, 0) = 1 / f_temp2;
	r_t_J_H(3, 0) = 0.0;
	r_t_J_H(4, 0) = 0.0;
	r_t_J_H(5, 0) = 0.0;
	r_t_J_H(6, 0) = f_temp0 / f_temp3 * TpOx;
	r_t_J_H(7, 0) = f_temp0 / f_temp3 * TpOy;
	r_t_J_H(8, 0) = f_temp0 / f_temp3;

	r_t_J_H(0, 1) = 0.0;
	r_t_J_H(1, 1) = 0.0;
	r_t_J_H(2, 1) = 0.0;
	r_t_J_H(3, 1) = TpOx / f_temp2;
	r_t_J_H(4, 1) = TpOy / f_temp2;
	r_t_J_H(5, 1) = 1 / f_temp2;
	r_t_J_H(6, 1) = f_temp1 / f_temp3 * TpOx;
	r_t_J_H(7, 1) = f_temp1 / f_temp3 * TpOy;
	r_t_J_H(8, 1) = f_temp1 / f_temp3;

	//r_t_J_H.rightCols<2>().setZero(); // those are zero; not explicitly stored

	r_t_J_T(0, 0) = h11 / f_temp2 + f_temp0 / f_temp3 * h31;
	r_t_J_T(1, 0) = h12 / f_temp2 + f_temp0 / f_temp3 * h32;
	r_t_J_T(0, 1) = h21 / f_temp2 + f_temp1 / f_temp3 * h31;
	r_t_J_T(1, 1) = h22 / f_temp2 + f_temp1 / f_temp3 * h32;
	r_t_J_T.template rightCols<2>().setIdentity();

	return v_error;
}

/*
 *								=== ~CHomography::CReprojectionErrorObjective ===
 */

/*

the transfer error Jacobians were computed using

syms h11 h12 h13 h21 h22 h23 h31 h32 h33 xix xiy Xix Xiy real
xi = [xix; xiy]
Xi = [Xix; Xiy]
H = [h11 h12 h13; h21 h22 h23; h31 h32 h33]
HXi = H * [Xi; 1];
HXi_dehom = HXi(1:2) / HXi(3)
iHXi = inv(H) * [xi; 1];
iHXi_dehom = iHXi(1:2) / iHXi(3) % f(g(H)) where g(H) is inv(H) and f(Hi) = dehomog(Hi * Xi)

Hvec = [h11 h12 h13 h21 h22 h23 h31 h32 h33]
J = jacobian(HXi_dehom, Hvec)' % asymetric transfer error jacobian
JSym = jacobian([HXi_dehom iHXi_dehom], Hvec)' % symetric transfer error jacobian

ccode(J)
JSym = subexpr(JSym, 'temp1');
temp1 = subexpr(temp1, 'temp0');
fprintf(strrep(ccode(temp0), ';', ';\n'))
fprintf(strrep(ccode(temp1), ';', ';\n'))
fprintf(strrep(ccode(JSym), ';', ';\n'))

the reprojection error Jacobian was computed using

syms h11 h12 h13 h21 h22 h23 h31 h32 h33 Imx Imy Tpx Tpy TpOx TpOy real
Im = [Imx; Imy]
Tp = [Tpx; Tpy]
TpO = [TpOx; TpOy]
H = [h11 h12 h13; h21 h22 h23; h31 h32 h33]

HTpO = H * [TpO; 1];
HTpO_dehom = HTpO(1:2) / HTpO(3)
Reproj = [TpO HTpO_dehom]

Hvec = [h11 h12 h13 h21 h22 h23 h31 h32 h33]
JH = jacobian(Reproj, Hvec)' % asymetric transfer error jacobian
JT = jacobian(Reproj, TpO)'

ccode(JH)
ccode(JT)

the chained symmetric transfer error jacobian was computed using

syms h11 h12 h13 h21 h22 h23 h31 h32 h33 xix xiy Xix Xiy real
xi = [xix; xiy]
Xi = [Xix; Xiy]
H = [h11 h12 h13; h21 h22 h23; h31 h32 h33]
HXi = H * [Xi; 1];
HXi_dehom = HXi(1:2) / HXi(3)
iHxi = inv(H) * [xi; 1];
iHxi_dehom = iHxi(1:2) / iHxi(3) % f(g(H)) where g(H) is inv(H) and f(Hi) = dehomog(Hi * Xi)

Hvec = [h11 h12 h13 h21 h22 h23 h31 h32 h33]
J = jacobian(HXi_dehom, Hvec)'
JSym = jacobian([HXi_dehom iHxi_dehom], Hvec)'
JSym_ = JSym; % remember

len_JSym = strcat(num2str(length(ccode(simple(JSym))) / 1024), ' kB')

ccode(J)
JSym = subexpr(JSym, 'temp1');
temp1 = subexpr(temp1, 'temp0');
fprintf(strrep(ccode(temp0), ';', ';\n'))
fprintf(strrep(ccode(temp1), ';', ';\n'))
fprintf(strrep(ccode(JSym), ';', ';\n'))

% ---- attempt to simplify the symmetric transfer error jacobian using chain rule ----

Hiv = inv(H);
Hiv_vec = [Hiv(1, 1) Hiv(1, 2) Hiv(1, 3) Hiv(2, 1) Hiv(2, 2) Hiv(2, 3) Hiv(3, 1) Hiv(3, 2) Hiv(3, 3)]; % mind the ordering here!
JInv = jacobian(Hiv_vec, Hvec)'
JInv_ = JInv; % remember

syms hi11 hi12 hi13 hi21 hi22 hi23 hi31 hi32 hi33 real
Hi = [hi11 hi12 hi13; hi21 hi22 hi23; hi31 hi32 hi33]
iHxi_ = Hi * [xi; 1];
iHxi_dehom_ = iHxi_(1:2) / iHxi_(3)
Hivec = [hi11 hi12 hi13 hi21 hi22 hi23 hi31 hi32 hi33]
JPrj = jacobian(iHxi_dehom_, Hivec)'
JPrj_ = JPrj; % remember
% note that JPrj is on separate variables

% chain rule as:
%    f(  g(H))' =      f'(  g(H)) *        g'(H)
% proj(inv(H))' =   proj'(inv(H)) *      inv'(H)
%             J =            JPrj *         JInv
%       dx / dH =   dxy / dinv(H) * dinv(H) / dH

JInv = subs(JInv, Hiv(1, 1), hi11);
JInv = subs(JInv, Hiv(1, 2), hi12);
JInv = subs(JInv, Hiv(1, 3), hi13);
JInv = subs(JInv, Hiv(2, 1), hi21);
JInv = subs(JInv, Hiv(2, 2), hi22);
JInv = subs(JInv, Hiv(2, 3), hi23);
JInv = subs(JInv, Hiv(3, 1), hi31);
JInv = subs(JInv, Hiv(3, 2), hi32);
JInv = simple(subs(JInv, Hiv(3, 3), hi33));
% attempr to simplify using subexpressions of the inverse (fails)

JInv = subexpr(JInv, 'temp2')
% reduce common subexpressions

len_JInv = strcat(num2str((length(ccode(simple(temp2))) + length(ccode(simple(JInv)))) / 1024), ' kB')
len_JPrj = strcat(num2str(length(ccode(simple(JPrj))) / 1024), ' kB')

JPrj = subs(JPrj, hi11, Hiv(1, 1));
JPrj = subs(JPrj, hi12, Hiv(1, 2));
JPrj = subs(JPrj, hi13, Hiv(1, 3));
JPrj = subs(JPrj, hi21, Hiv(2, 1));
JPrj = subs(JPrj, hi22, Hiv(2, 2));
JPrj = subs(JPrj, hi23, Hiv(2, 3));
JPrj = subs(JPrj, hi31, Hiv(3, 1));
JPrj = subs(JPrj, hi32, Hiv(3, 2));
JPrj = simple(subs(JPrj, hi33, Hiv(3, 3)));
% substitute the inverse matrix into the projection jacobian

error = simple(JSym_ - [J simple(JInv_ * JPrj)])
% verify

ccode(simple(temp2))
ccode(simple(JInv))
ccode(simple(JPrj_))

*/

#endif // !__HOMOGRAPHY_ESTIMATION_INLINES_INCLUDED
