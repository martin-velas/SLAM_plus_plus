/*
								+---------------------------------+
								|                                 |
								| *** Derivatives calculation *** |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2016 |
								|                                 |
								|          Derivatives.h          |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __DIFFERENTIATION_METHODS_INCLUDED
#define __DIFFERENTIATION_METHODS_INCLUDED

/**
 *	@file include/slam/Derivatives.h
 *	@date 2016
 *	@author -tHE SWINe-
 *	@brief calculation of function derivatives
 */

#include "slam/Unused.h"
#include "slam/BlockMatrixFBSUtil.h"

// todo - do automatic differentiation using dual numbers

/**
 *	@brief numerical differentiation functionality
 */
class CNumDiff {
public:
	/**
	 *	@brief wraps a function with argument that requires special composition
	 *		operator to act as a function of an ordinary argument
	 *
	 *	This wraps \f$f(x \oplus a)\f$ in \f$g(a)\f$, where \f$x\f$ is the constant argument
	 *	and \f$a\f$ is the delta applied by numerical differentiation. This changes the
	 *	computation of \f$\frac{df(x \oplus a)}{da}\f$ to \f$\frac{$df(a)}{da}\f$.
	 *
	 *	@tparam CResultVec is type of the result vector
	 *	@tparam CArgVec is type of the argument vector
	 *	@tparam COriginalFun is type of the original function object
	 *	@tparam CComposeFun is type of the composition function object (on the original function argument)
	 */
	template <class CResultVec, class CArgVec, class COriginalFun, class CComposeFun>
	class CWrapFunWithCompositionOp {
	protected:
		COriginalFun m_f; /**< @brief the original function object */
		CComposeFun m_plus; /**< @brief the composition function object */
		const CArgVec &m_r_v_argument; /**< @brief const reference to the argument vector */

	public:
		CWrapFunWithCompositionOp(COriginalFun f, CComposeFun plus, const CArgVec &r_v_argument)
			:m_f(f), m_plus(plus), m_r_v_argument(r_v_argument)
		{}

		template <class Derived>
		inline CResultVec operator ()(const Eigen::MatrixBase<Derived> &r_v_argument) const
		{
			return m_f(m_plus(m_r_v_argument, r_v_argument));
		}
	};

	/**
	 *	@brief a simple addition operator on vectors
	 */
	class CVectorialPlusOp {
	public:
		template <class Derived0, class Derived1>
		inline typename CDeriveMatrixType<Derived0, true>::_TyResult operator ()(const
			Eigen::MatrixBase<Derived0> &r_v_left, const Eigen::MatrixBase<Derived1> &r_v_right) const
		{
			return r_v_left + r_v_right;
		}

		inline double operator ()(double f_x, double f_y) const
		{
			return f_x + f_y;
		}
	};

public:
	/**
	 *	@brief calculates a numerical derivative (Newton's difference quotient) of a function
	 *		\f$f\f$ at a specific value of its argument \f$x\f$; this version assumes that the argument requires
	 *		a vectorial composition operator plus
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam DifferentiatedFun is type of the differentiated function object
	 *	@tparam ComposeFun is type of the composition function object on the argument
	 *
	 *	@param[out] r_t_Jacobian is filled with the Jacobian
	 *	@param[in] r_v_expectation is the value of \f$f(x)\f$ (to save computation)
	 *	@param[in] r_v_argument is the argument value \f$x\f$ to calculate the derivative at
	 *	@param[in] f is the function \f$f\f$ to calculate the derivative of
	 *	@param[in] plus is the composition function on the argument \f$x\f$ (called \f$\text{plus}(x, \epsilon)\f$)
	 *	@param[in] f_epsilon is the value of the change in the argument to calculate the numerical derivative
	 */
	template <class Derived0, class Derived1, class Derived2, class DifferentiatedFun, class ComposeFun>
	static void Evaluate(Eigen::MatrixBase<Derived0> &r_t_Jacobian, const Eigen::MatrixBase<Derived1> &r_v_expectation,
		const Eigen::MatrixBase<Derived2> &r_v_argument, DifferentiatedFun f, ComposeFun plus, double f_epsilon = 1e-9)
	{
		typedef typename CDeriveMatrixType<Derived0, false>::_TyResult _TyJac;
		enum {
			n_jac_rows = _TyJac::RowsAtCompileTime,
			n_jac_cols = _TyJac::ColsAtCompileTime
		};
		typedef typename Eigen::Matrix<double, n_jac_rows, 1> _TyExpectationVec;
		typedef typename Eigen::Matrix<double, n_jac_cols, 1> _TyArgumentVec;
		typedef typename Eigen::Matrix<double, n_jac_cols, n_jac_cols> _TyArgumentMat;
		// guess the types of everything

		DimensionCheck<_TyExpectationVec>(r_v_expectation);
		DimensionCheck<_TyArgumentVec>(r_v_argument);
		// makes sure the dimensions match

		const double f_inv_epsilon = 1 / f_epsilon;
		_TyArgumentVec v_epsilon = _TyArgumentVec::Zero();
		for(int i = 0; i < int(n_jac_cols); ++ i) { // unroll
			v_epsilon(i) = f_epsilon;
			// keep the epsilon vector

			const _TyArgumentVec &r_v_epsilon = v_epsilon; // makes sure the below call does not modify it
			_TyArgumentVec v_delta = plus(r_v_argument, r_v_epsilon);

			r_t_Jacobian.col(i) = (f(v_delta) - r_v_expectation) * f_inv_epsilon;

			v_epsilon(i) = 0;
			// re-zero the epsilon vector (avoids the if(!i) branch
		}
		// calculate the Jacobian
	}

	/**
	 *	@brief calculates a numerical derivative (Newton's difference quotient) of a function
	 *		\f$f\f$ at a specific value of its argument \f$x\f$; this version assumes that the
	 *		argument is plainly additive
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *	@tparam DifferentiatedFun is type of the differentiated function object
	 *
	 *	@param[out] r_t_Jacobian is filled with the Jacobian
	 *	@param[in] r_v_expectation is the value of \f$f(x)\f$ (to save computation)
	 *	@param[in] r_v_argument is the argument value \f$x\f$ to calculate the derivative at
	 *	@param[in] f is the function \f$f\f$ to calculate the derivative of
	 *	@param[in] f_epsilon is the value of the change in the argument to calculate the numerical derivative
	 */
	template <class Derived0, class Derived1, class Derived2, class DifferentiatedFun>
	static inline void Evaluate(Eigen::MatrixBase<Derived0> &r_t_Jacobian,
		const Eigen::MatrixBase<Derived1> &r_v_expectation, const Eigen::MatrixBase<Derived2> &r_v_argument,
		DifferentiatedFun f, double f_epsilon = 1e-9)
	{
		Evaluate(r_t_Jacobian, r_v_expectation, r_v_argument, f, CVectorialPlusOp(), f_epsilon);
		// reuse the function with explicit plus; does not waste computation that way
	}

	/**
	 *	@brief calculates a numerical derivative (Newton's difference quotient) of a function
	 *		\f$f\f$ at a specific value of its argument \f$x\f$; this version assumes that the
	 *		argument is plainly additive, and that the value of \f$f(x)\f$ is not available
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argumentt
	 *	@tparam DifferentiatedFun is type of the differentiated function object
	 *
	 *	@param[out] r_t_Jacobian is filled with the Jacobian
	 *	@param[in] r_v_argument is the argument value \f$x\f$ to calculate the derivative at
	 *	@param[in] f is the function \f$f\f$ to calculate the derivative of
	 *	@param[in] f_epsilon is the value of the change in the argument to calculate the numerical derivative
	 */
	template <class Derived0, class Derived1, class Derived2, class DifferentiatedFun>
	static inline void Evaluate(Eigen::MatrixBase<Derived0> &r_t_Jacobian,
		const Eigen::MatrixBase<Derived2> &r_v_argument, DifferentiatedFun f, double f_epsilon = 1e-9)
	{
		Evaluate(r_t_Jacobian, f(r_v_argument), r_v_argument, f, f_epsilon);
		// evaluate the f(a) in case the caller does not supply it
	}
};

/**
 *	@brief namespace containing utility function for CHoldArgsConstant
 */
namespace const_args_detail {

template <class CResult, class CFun, class T0, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const TSplice &r_a_splice, fbs_ut::CCTSize<0> UNUSED(tag))
{
	return f(r_a_splice, r_a0);
}

template <class CResult, class CFun, class T0, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const TSplice &r_a_splice, fbs_ut::CCTSize<1> UNUSED(tag))
{
	return f(r_a0, r_a_splice);
}

//

template <class CResult, class CFun, class T0, class T1, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const TSplice &r_a_splice, fbs_ut::CCTSize<0> UNUSED(tag))
{
	return f(r_a_splice, r_a0, r_a1);
}

template <class CResult, class CFun, class T0, class T1, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const TSplice &r_a_splice, fbs_ut::CCTSize<1> UNUSED(tag))
{
	return f(r_a0, r_a_splice, r_a1);
}

template <class CResult, class CFun, class T0, class T1, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const TSplice &r_a_splice, fbs_ut::CCTSize<2> UNUSED(tag))
{
	return f(r_a0, r_a1, r_a_splice);
}

//

template <class CResult, class CFun, class T0, class T1, class T2, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const T2 &r_a2,
	const TSplice &r_a_splice, fbs_ut::CCTSize<0> UNUSED(tag))
{
	return f(r_a_splice, r_a0, r_a1, r_a2);
}

template <class CResult, class CFun, class T0, class T1, class T2, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const T2 &r_a2,
	const TSplice &r_a_splice, fbs_ut::CCTSize<1> UNUSED(tag))
{
	return f(r_a0, r_a_splice, r_a1, r_a2);
}

template <class CResult, class CFun, class T0, class T1, class T2, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const T2 &r_a2,
	const TSplice &r_a_splice, fbs_ut::CCTSize<2> UNUSED(tag))
{
	return f(r_a0, r_a1, r_a_splice, r_a2);
}

template <class CResult, class CFun, class T0, class T1, class T2, class TSplice>
inline CResult SpliceArgs(CFun f, const T0 &r_a0, const T1 &r_a1, const T2 &r_a2,
	const TSplice &r_a_splice, fbs_ut::CCTSize<3> UNUSED(tag))
{
	return f(r_a0, r_a1, r_a2, r_a_splice);
}

// todo - will have to make more of those, for various lengths

/*template <class CArgList, const int n_variable_arg>
class CHoldArgsConstant_Base {
public:
	typedef typename CTypelistErase<CArgList, n_variable_arg>::_TyResult _TyConstArgList;
	typedef typename CTypelistItemAt<CArgList, n_variable_arg>::_TyResult _TyVariableArg;
};*/ // unnecessary, can use CChooseType as below

} // ~const_args_detail

template <class CResultType, class CFunctor, class CArgList, const int n_variable_arg>
class CHoldArgsConstant;

template <class CResultType, class CFunctor, class A0, class A1, const int n_variable_arg>
class CHoldArgsConstant<CResultType, CFunctor, MakeTypelist2(A0, A1), n_variable_arg> {
public:
	typedef typename CTypelistItemAt<MakeTypelist2(A0, A1), n_variable_arg>::_TyResult _TyVariableArg;
	typedef typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult T0;

protected:
	CFunctor m_f;
	T0 m_a0;

public:
	CHoldArgsConstant(CFunctor f, T0 a0)
		:m_f(f), m_a0(a0)
	{}

	inline CResultType operator ()(_TyVariableArg free_arg) const
	{
		return const_args_detail::SpliceArgs<CResultType>(m_f, m_a0, free_arg, fbs_ut::CCTSize<n_variable_arg>());
	}
};

template <class CResultType, class CFunctor, class A0, class A1, class A2, const int n_variable_arg>
class CHoldArgsConstant<CResultType, CFunctor, MakeTypelist3(A0, A1, A2), n_variable_arg> {
public:
	typedef typename CTypelistItemAt<MakeTypelist3(A0, A1, A2), n_variable_arg>::_TyResult _TyVariableArg;
	typedef typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult T0;
	typedef typename CChooseType<A2, A1, (n_variable_arg <= 1)>::_TyResult T1;

protected:
	CFunctor m_f;
	T0 m_a0;
	T1 m_a1;

public:
	CHoldArgsConstant(CFunctor f, T0 a0, T1 a1)
		:m_f(f), m_a0(a0), m_a1(a1)
	{}

	inline CResultType operator ()(_TyVariableArg free_arg) const
	{
		return const_args_detail::SpliceArgs<CResultType>(m_f, m_a0, m_a1,
			free_arg, fbs_ut::CCTSize<n_variable_arg>());
	}
};

template <class CResultType, class CFunctor, class A0, class A1, class A2, class A3, const int n_variable_arg>
class CHoldArgsConstant<CResultType, CFunctor, MakeTypelist4(A0, A1, A2, A3), n_variable_arg> {
public:
	typedef typename CTypelistItemAt<MakeTypelist4(A0, A1, A2, A3), n_variable_arg>::_TyResult _TyVariableArg;
	typedef typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult T0;
	typedef typename CChooseType<A2, A1, (n_variable_arg <= 1)>::_TyResult T1;
	typedef typename CChooseType<A3, A2, (n_variable_arg <= 2)>::_TyResult T2;

protected:
	CFunctor m_f;
	T0 m_a0;
	T1 m_a1;
	T2 m_a2;

public:
	CHoldArgsConstant(CFunctor f, T0 a0, T1 a1, T2 a2)
		:m_f(f), m_a0(a0), m_a1(a1), m_a2(a2)
	{}

	inline CResultType operator ()(_TyVariableArg free_arg) const
	{
		return const_args_detail::SpliceArgs<CResultType>(m_f, m_a0, m_a1, m_a2,
			free_arg, fbs_ut::CCTSize<n_variable_arg>());
	}
};

// todo - will have to make more of those, for various lengths, let's say up to eight

template <const int n_variable_arg, class CFunObject, class CResultType, class A0, class A1>
CHoldArgsConstant<CResultType, CFunObject, MakeTypelist2(A0, A1), n_variable_arg>
	HoldArgsConstant(CFunObject f,
	typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult a0,
	CResultType (CFunObject::*p_fun)(A0, A1) = &CFunObject::operator ())
{
	return CHoldArgsConstant<CResultType, CFunObject,
		MakeTypelist2(A0, A1), n_variable_arg>(f, a0);
}
//
template <const int n_variable_arg, class CFunObject, class CResultType, class A0, class A1, class A2>
CHoldArgsConstant<CResultType, CFunObject, MakeTypelist3(A0, A1, A2), n_variable_arg>
	HoldArgsConstant(CFunObject f,
	typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult a0,
	typename CChooseType<A2, A1, (n_variable_arg <= 1)>::_TyResult a1,
	CResultType (CFunObject::*p_fun)(A0, A1, A2) = &CFunObject::operator ())
{
	return CHoldArgsConstant<CResultType, CFunObject,
		MakeTypelist3(A0, A1, A2), n_variable_arg>(f, a0, a1);
}
//
template <const int n_variable_arg, class CFunObject, class CResultType, class A0, class A1, class A2, class A3>
CHoldArgsConstant<CResultType, CFunObject, MakeTypelist4(A0, A1, A2, A3), n_variable_arg>
	HoldArgsConstant(CFunObject f,
	typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult a0,
	typename CChooseType<A2, A1, (n_variable_arg <= 1)>::_TyResult a1,
	typename CChooseType<A3, A2, (n_variable_arg <= 2)>::_TyResult a2,
	CResultType (CFunObject::*p_fun)(A0, A1, A2, A3) = &CFunObject::operator ())
{
	return CHoldArgsConstant<CResultType, CFunObject,
		MakeTypelist4(A0, A1, A2, A3), n_variable_arg>(f, a0, a1);
}
// this requires the caller to also specify the CFunObject::operator () as the last argument but it works
// probably better off to use a typelist as a tag argument
// could go the way of traits as in the OpenCL header using std::binary_function but that's deprecated
// as of C++11 (now there's std::function but that one does not expose argument types for functions of
// three args or more so it is pretty much useless)
// could go backward and infer the template arguments for everything from the arguments to this function
// but then there is an issue with const-ness and pass-by-reference (but should work for vector types)

#if 0 // this yields "sorry, unimplemented: mangling typeof, use decltype instead" (g++ 4.4.7)

#include "../external/TooN_msvc_patch.h"

namespace const_args_detail {

template <const int n_variable_arg, class CFunObj, class CFunction>
class CFunObjTraits;

template <const int n_variable_arg, class F, class R, class A0, class A1, class A2>
class CFunObjTraits<n_variable_arg, F, R (F::*)(A0, A1, A2) const> {
public:
	typedef F _TyFunObj;
	typedef R _TyResult;
	typedef MakeTypelist3(A0, A1, A2) _TyArgs;

	typedef ::CHoldArgsConstant<_TyResult, _TyFunObj, _TyArgs, n_variable_arg> _TyHAC;
};

} // ~const_args_detail

#define FOT(n_variable_arg,CFunObject) \
	typename const_args_detail::CFunObjTraits<n_variable_arg, CFunObject, __typeof__(CFunObject::operator ())>

template <const int n_variable_arg, class CFunObject>
void HoldArgsConstant_dummy(CFunObject f,
	typename CTypelistItemAt<FOT(n_variable_arg, CFunObject)::_TyArgs, 0 + (n_variable_arg == 0)>::_TyResult a0,
	typename CTypelistItemAt<FOT(n_variable_arg, CFunObject)::_TyArgs, 1 + (n_variable_arg <= 1)>::_TyResult a1)
{
}

template <const int n_variable_arg, class CFunObject>
	FOT(n_variable_arg, CFunObject)::_TyHAC HoldArgsConstant(CFunObject f,
	typename CTypelistItemAt<FOT(n_variable_arg, CFunObject)::_TyArgs, 0 + (n_variable_arg == 0)>::_TyResult a0,
	typename CTypelistItemAt<FOT(n_variable_arg, CFunObject)::_TyArgs, 1 + (n_variable_arg <= 1)>::_TyResult a1)
{
	return FOT(n_variable_arg, CFunObject)::_TyHAC(f, a0, a1);
}

#endif // 0

/*template <class CFunction>
class CFunTraits;

template <class R, class A0, class A1, class A2>
class CFunTraits<R (*)(A0, A1, A2)> {
public:
	typedef R _TyResult;
	typedef MakeTypelist3(A0, A1, A2) _TyArgs;
};

template <class CFunObject>
void GetFunTraits(CFunObject f)
{
	typedef CFunTraits<__typeof__(&CFunObject::operator ())> TR;
}*/

//#define HoldArgsConstant_fo(i,f,a0,a1) HoldArgsConstant<(i)>((f),(a0),(a1),&((f).operator ()))

template <const int n_variable_arg, class CResultType, class A0, class A1>
CHoldArgsConstant<CResultType, CResultType (*)(A0, A1), MakeTypelist2(A0, A1), n_variable_arg>
	HoldArgsConstant(CResultType (*p_fun)(A0, A1),
	typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult a0)
{
	return CHoldArgsConstant<CResultType, CResultType (*)(A0, A1),
		MakeTypelist2(A0, A1), n_variable_arg>(p_fun, a0);
}

template <const int n_variable_arg, class CResultType, class A0, class A1, class A2>
CHoldArgsConstant<CResultType, CResultType (*)(A0, A1, A2), MakeTypelist3(A0, A1, A2), n_variable_arg>
	HoldArgsConstant(CResultType (*p_fun)(A0, A1, A2),
	typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult a0,
	typename CChooseType<A2, A1, (n_variable_arg <= 1)>::_TyResult a1)
{
	return CHoldArgsConstant<CResultType, CResultType (*)(A0, A1, A2),
		MakeTypelist3(A0, A1, A2), n_variable_arg>(p_fun, a0, a1);
}

template <const int n_variable_arg, class CResultType, class A0, class A1, class A2, class A3>
CHoldArgsConstant<CResultType, CResultType (*)(A0, A1, A2, A3), MakeTypelist4(A0, A1, A2, A3), n_variable_arg>
	HoldArgsConstant(CResultType (*p_fun)(A0, A1, A2, A3),
	typename CChooseType<A1, A0, (n_variable_arg == 0)>::_TyResult a0,
	typename CChooseType<A2, A1, (n_variable_arg <= 1)>::_TyResult a1,
	typename CChooseType<A3, A2, (n_variable_arg <= 2)>::_TyResult a2)
{
	return CHoldArgsConstant<CResultType, CResultType (*)(A0, A1, A2, A3),
		MakeTypelist4(A0, A1, A2, A3), n_variable_arg>(p_fun, a0, a1, a2);
}

#endif // !__DIFFERENTIATION_METHODS_INCLUDED
