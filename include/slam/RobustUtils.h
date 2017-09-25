/*
								+----------------------------------+
								|                                  |
								|*** Robust loss function utils ***|
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|          RobustUtils.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __ROBUST_LOSS_FUNCTION_UTILITIES_INCLUDED
#define __ROBUST_LOSS_FUNCTION_UTILITIES_INCLUDED

/**
 *	@file include/slam/RobustUtils.h
 *	@brief robust loss function utilities
 *	@author -tHE SWINe-
 *	@date 2016
 */

/**
 *	@page robustopt Robust Optimization
 *
 *	\ref include/slam/RobustUtils.h adds a few utilities to make robust optimization easy. To enable robust
 *	optimization, the edges need to specify \ref CBaseEdge::Robust in the last template
 *	paramter to \ref CBaseEdgeImpl and to provide a <tt>f_RobustWeight()</tt> function
 *	that takes the error vector as its only argument and returns the robust weight.
 *
 *	This scheme gives the maximum freedom to the user, as the <tt>f_RobustWeight()</tt>
 *	function can do pretty much anything and has access to the observation and the vertices.
 *	However, this is unnecessarily complicated for most optimization tasks.
 *
 *	For the most part, todays optimizers avoid implementing IRLS so the robust
 *	optimization boils down to evaluating a robust weight function with a fixed scale.
 *	This file adds functionality that lets the user specify the robust function
 *	for each edge type (rather than edge instance) using inheritance. For instance,
 *	the \ref CEdgePose2D is defined as:
 *
 *	@code
 *	class CEdgePose2D : public CBaseEdgeImpl<CEdgePose2D,
 *		MakeTypelist(CVertexPose2D, CVertexPose2D), 3> {
 *	@endcode
 *
 *	To make it robust, one needs to performa two essential changes:
 *
 *	@code
 *	class CEdgePose2D : public CBaseEdgeImpl<CEdgePose2D,
 *		MakeTypelist(CVertexPose2D, CVertexPose2D), 3, 3, CBaseEdge::Robust>, // change 1 - specify robust option
 *		public Some_Class_With_Robust_Fun { // change 2 - get implementation of the robust weight
 *
 *		// additionally, change the base edge type if referring to it in the constructors
 *	@endcode
 *
 *	Here, the <tt>Some_Class_With_Robust_Fun</tt> can be replaced e.g. by:
 *
 *	@code
 *	CRobustify_ErrorNorm_Default<double, CHuberLossd>
 *	@endcode
 *
 *	where the first argument is the scale type and the second argument is the type
 *	of the loss function (Huber in double precision). This requires setting the scale
 *	in the constructor and it can be stored inside the edge. It will therefore add
 *	to the memory requirements.
 *
 *	Since the edges are optimized to have minimal size in memory and since the scale
 *	parameters are often constants, we prefer to specify all the constants as
 *	compile-time constants rather than by adding extra payload to all the edges.
 *	This can be done by replacing <tt>Some_Class_With_Robust_Fun</tt> above with:
 *
 *	@code
 *	CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, CHuberLossd>
 *	@endcode
 *
 *	which is equivalent to setting the scale to roughly \f$16 \cdot 1.4826 \approx \frac{593}{25}\f$.
 *	This effectively treats all the observations with error norms above 16 as outliers. The same
 *	effect would be achieved by adding this member function to our \ref CEdgePose2D:
 *
 *	@code
 *	double f_RobustWeight(const Eigen::Vector2d &r_v_error) const
 *	{
 *		CHuberLossd loss;
 *		double scale = 16 * 1.4826;
 *		return loss(r_v_error.norm() / scale);
 *	}
 *	@endcode
 *
 *	In case one wanted to change the arguments of the loss function as well,
 *	it is possible to use:
 *
 *	@code
 *	CRobustify_ErrorNorm_Default<CCTFraction<593, 25>, // 593 / 25 = 16 * 1.4826
 *		CFlyweightLoss<CHuberLossd, CCTFraction<269, 200> > > // 269 / 200 = 1.345
 *	@endcode
 *
 *	In case one wanted to perform IRLS optimization, first either a dummy 1D vertex needs
 *	to be created or another storage for the scale parameter provisioned. Then, it is
 *	possible to use:
 *
 *	@code
 *	CRobustify_ErrorNorm_Default<const double*, CHuberLossd>
 *	@endcode
 *
 *	and in the edge constructor, one passes the (immutable) pointer to double that
 *	will contain the scale parameter throughout the optimization. Note that upon changing
 *	the parameters of the robust function, one needs to notify the nonlinear solver
 *	to correctly relinearize the affected edges, using
 *	\ref CNonlinearSolver_Lambda::Notify_LinearizationChange()
 *	(a similar function was added to most of the solvers in SLAM++).
 *
 *	In case there are different kinds of edges in the system and their error is not
 *	comparable, it might be better to use \f$\chi^2\f$ error as a base for robust weight
 *	calculation instead of norm of error. To do that, it is possible to use
 *	\ref CRobustify_Chi2_Default e.g. like this:
 *
 *	@code
 *	class CEdgePose2D : public CBaseEdgeImpl<CEdgePose2D,
 *		MakeTypelist(CVertexPose2D, CVertexPose2D), 3, 3, CBaseEdge::Robust>, // change 1 - specify robust option
 *		public CRobustify_Chi2_Default<double, CHuberLossd, CEdgePose2D> { // change 2 - get implementation of the robust weight
 *
 *		// additionally, change the base edge type if referring to it in the constructors
 *	@endcode
 *
 *	Note the use of CRTP (Curiously Recurring Template Pattern-you need to specify the edge
 *	type name as the last template argument to \ref CRobustify_Chi2_Default. Also note
 *	that the scale parameter now depends on the covariances of the observations and it
 *	would likely be impractical to enter it as a compile-time constant. This will require
 *	you to specify the scale parameter and pass it to all the edges (you will need to modify
 *	their constructor, etc.).
 *
 *	More complicated use cases require the user to implement her own version of
 *	<tt>f_RobustWeight</tt> similarly as shown above.
 *
 */

/**
 *	@defgroup robust_group Robust Optimization
 *	@brief Utilities for robust optimization.
 *
 *	You may also want to take a look at \ref robustopt.
 */

/** \addtogroup robust_group
 *	@{
 */

#include "geometry/RobustLoss.h" // basic robust kernels

/**
 *	@brief a simple compile-time fraction template
 *
 *	@param[in] _n_numerator is the numerator value
 *	@param[in] _n_denominator is the denominator value
 */
template <const int _n_numerator, const int _n_denominator = 1>
class CCTFraction {
public:
	enum {
		n_numerator = _n_numerator, /**< @brief numerator value */
		n_denominator = _n_denominator /**< @brief denominator value */
	};

public:
	/**
	 *	@brief default constructor; can take assignment of a floating-point value
	 *	@param[in] f_value is the floating point value to set (must be equal to the value of this fraction)
	 */
	CCTFraction(double f_value = double(n_numerator) / n_denominator)
	{
		*this = f_value; // reuse the assignment op
	}

	/**
	 *	@brief default constructor; can take assignment of a floating-point value
	 *	@param[in] f_value is the floating point value to set (must be equal to the value of this fraction)
	 */
	CCTFraction &operator =(double f_value)
	{
		_ASSERTE(fabs(f_value - *this) / std::max(1.0, (double)*this) < 1e-10 || // compare numbers
			(!n_denominator && n_numerator && f_value == *this) || // compare infinities
			(!n_denominator && !n_numerator && f_value != f_value)); // compare NaN
		// this cannot remember a different value than the one set in the args

		return *this;
	}

	/**
	 *	@brief conversion to double
	 *	@return Returns the value of the fraction.
	 */
	inline operator double() const
	{
		return double(n_numerator) / n_denominator;
	}
};

/**
 *	@brief loss function wrapper with zero storage static const parameters
 *
 *	@tparam CLossFunction is loss function type
 *	@tparam CConstArg is constant loss function argument (e.g. a specialization of \ref CCTFraction)
 *	@tparam CScalar is scalar type
 */
template <class CLossFunction, class CConstArg, class CScalar = typename CLossFunction::_TyScalar>
class CFlyweightLoss {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

public:
	/**
	 *	@brief default constructor; can optionally receive the input argument
	 *	@param[in] f_arg0 is the loss function argument (ignored, needs to be equal to the specified constant)
	 */
	CFlyweightLoss(CScalar f_arg0 = CConstArg())
	{
		CConstArg a = f_arg0;
		// test assignment in debug; make sure we're not attempting to use this
		// with different arguments than the ones specified in the template arg list
	}

	/**
	 *	@copydoc CHuberLoss::v_Loss()
	 */
	template <class CVectorType>
	CVectorType v_Loss(CScalar f_error) const
	{
		return CLossFunction(CConstArg()).template v_Loss<CVectorType>(f_error);
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	CScalar operator ()(CScalar f_error) const
	{
		return CLossFunction(CConstArg())(f_error);
	}
};

/**
 *	@brief loss function wrapper with zero storage static const parameters
 *
 *	@tparam CLossFunction is loss function type
 *	@tparam CConstArg0 is the first constant loss function argument (e.g. a specialization of \ref CCTFraction)
 *	@tparam CConstArg1 is the second constant loss function argument
 *	@tparam CScalar is scalar type
 */
template <class CLossFunction, class CConstArg0, class CConstArg1,
	class CScalar = typename CLossFunction::_TyScalar>
class CFlyweightLoss2 {
public:
	typedef CScalar _TyScalar; /**< @brief scalar type */

public:
	/**
	 *	@brief default constructor; can optionally receive the input arguments
	 *
	 *	@param[in] f_arg0 is the first loss function argument 
	 *	@param[in] f_arg1 is the second loss function argument 
	 *
	 *	@note The argument values are ignored. In debug they
	 *		are checked to be equal to the specified constant.
	 */
	CFlyweightLoss2(CScalar f_arg0 = CConstArg0(), CScalar f_arg1 = CConstArg1())
	{
		CConstArg0 a = f_arg0;
		CConstArg1 b = f_arg1;
		// test assignment in debug; make sure we're not attempting to use this
		// with different arguments than the ones specified in the template arg list
	}

	/**
	 *	@copydoc CHuberLoss::v_Loss()
	 */
	template <class CVectorType>
	CVectorType v_Loss(CScalar f_error) const
	{
		return CLossFunction(CConstArg0(), CConstArg1()).template v_Loss<CVectorType>(f_error);
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	CScalar operator ()(CScalar f_error) const
	{
		return CLossFunction(CConstArg0(), CConstArg1())(f_error);
	}
};

/**
 *	@brief loss function wrapper with zero storage static const parameters
 *
 *	@tparam CLossFunction is loss function type
 *	@tparam CConstArg0 is the first constant loss function argument (e.g. a specialization of \ref CCTFraction)
 *	@tparam CConstArg1 is the second constant loss function argument
 *	@tparam CConstArg2 is the third constant loss function argument
 *	@tparam CScalar is scalar type
 */
template <class CLossFunction, class CConstArg0, class CConstArg1,
	class CConstArg2, class CScalar = typename CLossFunction::_TyScalar>
class CFlyweightLoss3 {
public:
	typedef CScalar _TyScalar;

public:
	/**
	 *	@brief default constructor; can optionally receive the input arguments
	 *
	 *	@param[in] f_arg0 is the first loss function argument 
	 *	@param[in] f_arg1 is the second loss function argument 
	 *	@param[in] f_arg2 is the third loss function argument 
	 *
	 *	@note The argument values are ignored. In debug they
	 *		are checked to be equal to the specified constant.
	 */
	CFlyweightLoss3(CScalar f_arg0 = CConstArg0(),
		CScalar f_arg1 = CConstArg1(), CScalar f_arg2 = CConstArg2())
	{
		CConstArg0 a = f_arg0;
		CConstArg1 b = f_arg1;
		CConstArg2 c = f_arg2;
		// test assignment in debug; make sure we're not attempting to use this
		// with different arguments than the ones specified in the template arg list
	}

	/**
	 *	@copydoc CHuberLoss::v_Loss()
	 */
	template <class CVectorType>
	CVectorType v_Loss(CScalar f_error) const
	{
		return CLossFunction(CConstArg0(), CConstArg1(), CConstArg2()).template v_Loss<CVectorType>(f_error);
	}

	/**
	 *	@copydoc CHuberLoss::operator ()
	 */
	CScalar operator ()(CScalar f_error) const
	{
		return CLossFunction(CConstArg0(), CConstArg1(), CConstArg2())(f_error);
	}
};

/**
 *	@brief implementation of robust weight calculation for edges with euclidean error
 *
 *	This implements the <tt>f_RobustWeight()</tt> function that is required by the
 *	robust edges. Its argument is the error vector; its L2 notm is taken and passed
 *	to the specified robust kernel to produce weight for this observation.
 *	This makes sense in systems where all the observations have the same space
 *	where the error is defined. If there are different error spaces, it is better
 *	to use \ref CRobustify_Chi2_Default instead.
 *
 *	The error is divided by a scalar in order to fit scales of different problems.
 *	This scalar can be specified in three basic ways; either as a compile-time
 *	constant via \ref CCTFraction (used in case the scale of the errors is known),
 *	or as a pointer to a number (used in case IRLS is being performed and the scale
 *	changes) or as an ordinary scalar number in case the scale is not constant
 *	but does not change during the optimization.
 *
 *	@tparam CScaleValueType is error scale value type, can be either a value,
 *		a pointer or a \ref CCTFraction specializazion
 *	@tparam CRobustKernel is robust kernel type (it is used with the default
 *		parameters; can use \ref CFlyweightLoss to specify different default parameters)
 */
template <class CScaleValueType, class CRobustKernel>
class CRobustify_ErrorNorm_Default {
protected:
	CScaleValueType m_f_error_scale; /**< @brief error scale value */

public:
	/**
	 *	@brief default constructor; sets the error scale
	 *	@param[in] f_error_scale is error scale value
	 */
	CRobustify_ErrorNorm_Default(CScaleValueType f_error_scale)
		:m_f_error_scale(f_error_scale)
	{}

	/**
	 *	@brief sets the error scale
	 *	@param[in] f_error_scale is the new error scale value
	 */
	void Set_Scale(CScaleValueType f_error_scale)
	{
		m_f_error_scale = f_error_scale;
	}

	/**
	 *	@brief evaluates the robust weight function
	 *	@tparam Derived is derived type if the Eigen matrix expression
	 *	@param[in] r_v_error is error vector
	 *	@return Returns the robust weight.
	 */
	template <class Derived>
	double f_RobustWeight(const Eigen::MatrixBase<Derived> &r_v_error) const
	{
		return CRobustKernel()(r_v_error.norm() / m_f_error_scale);
	}
};

/**
 *	@brief implementation of robust weight calculation for edges with euclidean
 *		error (specialization for compile-time constant scale)
 *
 *	@tparam _n_numerator is error scale value numerator
 *	@tparam _n_denominator is error scale value denominator
 *	@tparam CRobustKernel is robust kernel type (it is used with the default
 *		parameters; can use \ref CFlyweightLoss to specify different default parameters)
 */
template <const int _n_numerator, const int _n_denominator, class CRobustKernel>
class CRobustify_ErrorNorm_Default<CCTFraction<_n_numerator, _n_denominator>, CRobustKernel> {
protected:
	typedef CCTFraction<_n_numerator, _n_denominator> CErrorScale; /**< @brief error scale type */

public:
	/**
	 *	@brief default constructor; optionally allows to set the error scale
	 *	@param[in] f_error_scale is error scale value
	 *		(must match the value of the numerator over denominator)
	 */
	CRobustify_ErrorNorm_Default(double f_error_scale = CErrorScale())
	{
		CErrorScale s = f_error_scale; // just check the assignment
	}

	/**
	 *	@brief evaluates the robust weight function
	 *	@tparam Derived is derived type if the Eigen matrix expression
	 *	@param[in] r_v_error is error vector
	 *	@return Returns the robust weight.
	 */
	template <class Derived>
	double f_RobustWeight(const Eigen::MatrixBase<Derived> &r_v_error) const
	{
		return CRobustKernel()(r_v_error.norm() / CErrorScale());
	}
};

/**
 *	@brief implementation of robust weight calculation for edges with euclidean
 *		error (specialization for a pointer to the global error scale)
 *
 *	@tparam CScaleValueType is error scale value type
 *	@tparam CRobustKernel is robust kernel type (it is used with the default
 *		parameters; can use \ref CFlyweightLoss to specify different default parameters)
 */
template <class CScaleValueType, class CRobustKernel>
class CRobustify_ErrorNorm_Default<const CScaleValueType*, CRobustKernel> {
protected:
	const CScaleValueType *m_p_error_scale; /**< @brief error scale pointer */

public:
	/**
	 *	@brief default constructor; sets the error scale
	 *	@param[in] p_error_scale is pointer to the error scale (must remain allocated)
	 */
	CRobustify_ErrorNorm_Default(const CScaleValueType *p_error_scale)
		:m_p_error_scale(p_error_scale)
	{
		_ASSERTE(p_error_scale);
	}

	/**
	 *	@brief evaluates the robust weight function
	 *	@tparam Derived is derived type if the Eigen matrix expression
	 *	@param[in] r_v_error is error vector
	 *	@return Returns the robust weight.
	 */
	template <class Derived>
	double f_RobustWeight(const Eigen::MatrixBase<Derived> &r_v_error) const
	{
		return CRobustKernel()(r_v_error.norm() / *m_p_error_scale);
	}
};

/**
 *	@brief implementation of robust weight calculation for general edges
 *
 *	This implements the <tt>f_RobustWeight()</tt> function that is required by the
 *	robust edges. Its argument is the error vector; it is used to calculate the
 *	\f$\chi^2\f$ error that is subsequenrly taken and passed to the specified robust
 *	kernel to produce weight for this observation.
 *	This makes sense in systems where different observations have different spaces
 *	where the error is defined or different scales. If the errors are in the same
 *	space and have the same scale, it is possible (and cheaper) to use \ref CRobustify_ErrorNorm_Default.
 *
 *	The error is divided by a scalar in order to fit scales of different problems.
 *	This scalar can be specified in three basic ways; either as a compile-time
 *	constant via \ref CCTFraction (used in case the scale of the errors is known),
 *	or as a pointer to a number (used in case IRLS is being performed and the scale
 *	changes) or as an ordinary scalar number in case the scale is not constant
 *	but does not change during the optimization.
 *
 *	@tparam CScaleValueType is error scale value type, can be either a value,
 *		a pointer or a \ref CCTFraction specializazion
 *	@tparam CRobustKernel is robust kernel type (it is used with the default
 *		parameters; can use \ref CFlyweightLoss to specify different default parameters)
 */
template <class CScaleValueType, class CRobustKernel, class CDerivedEdgeType>
class CRobustify_Chi2_Default {
protected:
	CScaleValueType m_f_error_scale; /**< @brief error scale value */

public:
	/**
	 *	@brief default constructor; sets the error scale
	 *	@param[in] f_error_scale is error scale value
	 */
	CRobustify_Chi2_Default(CScaleValueType f_error_scale)
		:m_f_error_scale(f_error_scale)
	{}

	/**
	 *	@brief sets the error scale
	 *	@param[in] f_error_scale is the new error scale value
	 */
	void Set_Scale(CScaleValueType f_error_scale)
	{
		m_f_error_scale = f_error_scale;
	}

	/**
	 *	@brief evaluates the robust weight function
	 *	@tparam Derived is derived type if the Eigen matrix expression
	 *	@param[in] r_v_error is error vector
	 *	@return Returns the robust weight.
	 */
	template <class Derived>
	double f_RobustWeight(const Eigen::MatrixBase<Derived> &r_v_error) const
	{
		double f_chi2 = r_v_error.dot(((const CDerivedEdgeType*)this)->t_Sigma_Inv() * r_v_error);
		return CRobustKernel()(f_chi2 / m_f_error_scale);
	}
};

/**
 *	@brief implementation of robust weight calculation for edges with euclidean
 *		error (specialization for compile-time constant scale)
 *
 *	@tparam _n_numerator is error scale value numerator
 *	@tparam _n_denominator is error scale value denominator
 *	@tparam CRobustKernel is robust kernel type (it is used with the default
 *		parameters; can use \ref CFlyweightLoss to specify different default parameters)
 */
template <const int _n_numerator, const int _n_denominator, class CRobustKernel, class CDerivedEdgeType>
class CRobustify_Chi2_Default<CCTFraction<_n_numerator, _n_denominator>, CRobustKernel, CDerivedEdgeType> {
protected:
	typedef CCTFraction<_n_numerator, _n_denominator> CErrorScale; /**< @brief error scale type */

public:
	/**
	 *	@brief default constructor; optionally allows to set the error scale
	 *	@param[in] f_error_scale is error scale value
	 *		(must match the value of the numerator over denominator)
	 */
	CRobustify_Chi2_Default(double f_error_scale = CErrorScale())
	{
		CErrorScale s = f_error_scale; // just check the assignment
	}

	/**
	 *	@brief evaluates the robust weight function
	 *	@tparam Derived is derived type if the Eigen matrix expression
	 *	@param[in] r_v_error is error vector
	 *	@return Returns the robust weight.
	 */
	template <class Derived>
	double f_RobustWeight(const Eigen::MatrixBase<Derived> &r_v_error) const
	{
		double f_chi2 = r_v_error.dot(((const CDerivedEdgeType*)this)->t_Sigma_Inv() * r_v_error);
		return CRobustKernel()(f_chi2 / CErrorScale());
	}
};

/**
 *	@brief implementation of robust weight calculation for edges with euclidean
 *		error (specialization for a pointer to the global error scale)
 *
 *	@tparam CScaleValueType is error scale value type
 *	@tparam CRobustKernel is robust kernel type (it is used with the default
 *		parameters; can use \ref CFlyweightLoss to specify different default parameters)
 */
template <class CScaleValueType, class CRobustKernel, class CDerivedEdgeType>
class CRobustify_Chi2_Default<const CScaleValueType*, CRobustKernel, CDerivedEdgeType> {
protected:
	const CScaleValueType *m_p_error_scale; /**< @brief error scale pointer */

public:
	/**
	 *	@brief default constructor; sets the error scale
	 *	@param[in] p_error_scale is pointer to the error scale (must remain allocated)
	 */
	CRobustify_Chi2_Default(const CScaleValueType *p_error_scale)
		:m_p_error_scale(p_error_scale)
	{
		_ASSERTE(p_error_scale);
	}

	/**
	 *	@brief evaluates the robust weight function
	 *	@tparam Derived is derived type if the Eigen matrix expression
	 *	@param[in] r_v_error is error vector
	 *	@return Returns the robust weight.
	 */
	template <class Derived>
	double f_RobustWeight(const Eigen::MatrixBase<Derived> &r_v_error) const
	{
		double f_chi2 = r_v_error.dot(((const CDerivedEdgeType*)this)->t_Sigma_Inv() * r_v_error);
		return CRobustKernel()(f_chi2 / *m_p_error_scale);
	}
};

/** @} */ // end of group

#endif // !__ROBUST_LOSS_FUNCTION_UTILITIES_INCLUDED
