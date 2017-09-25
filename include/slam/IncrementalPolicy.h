/*
								+----------------------------------+
								|                                  |
								|  ***  Incremental policies  ***  |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2013  |
								|                                  |
								|       IncrementalPolicy.h        |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __INCREMENTAL_POLICIES_INCLUDED
#define __INCREMENTAL_POLICIES_INCLUDED

/**
 *	@file include/slam/IncrementalPolicy.h
 *	@brief incremental calculation policies for nonlinear solvers
 *	@author -tHE SWINe-
 *	@date 2013-12-12
 */

//#include "slam/Unused.h" // included from slam/Integer.h
#include "slam/Integer.h"

/**
 *	@page incrementalsolvepolicy Incremental Solving Policies
 *
 *	The early versions of SLAM++ used a simple method of configuration of nonlinear solvers
 *	for incremental solving, simply by passing many scalar arguments to the constructor. However, with
 *	increasing complexity this quickly becomes quite messy and hard to maintain. See for
 *	example CNonlinearSolver_A::CNonlinearSolver_A() constructor.
 *
 *	The old constructors will remain functional, but are deprecated as of now, in favor of
 *	the new ones described below.
 *
 *	To configure a solver for batch solving (such as in Simple SLAM Example), one does:
 *	@code
 *	CNonlinearSolver_Lambda<CSystemType, CLinearSolverType> solver(system);
 *	@endcode
 *	That is quite simple. Now to configure a solver for incremental solving, such as in
 *	Online SLAM Example, one would do:
 *	@code
 *	CNonlinearSolver_FastL<CSystemType, CLinearSolverType> solver(system, solve::Nonlinear(frequency::Every(1)));
 *	@endcode
 *	And that configures the solver to do nonlinear solving at each step. Note that the
 *	code is quite verbose, and you immediately see what it does. You can use any of the following:
 *	@code
 *	SomeSolverType solver(system, solve::Batch()); // solve batch (default)
 *
 *	SomeSolverType solver(system, solve::Nonlinear(frequency::Every(1))); // solve every 1
 *
 *	SomeSolverType solver(system, solve::Nonlinear(frequency::Every(100))); // solve every 100
 *
 *	SomeSolverType solver(system, solve::Nonlinear(frequency::Every(100), 10));
 *	// solve every 100, up to 10 iterations of NLS
 *
 *	SomeSolverType solver(system, solve::Nonlinear(frequency::Every(100), 10, 0.1));
 *	// solve every 100, up to 10 iterations of NLS, 0.1 is residual error threshold
 *
 *	SomeSolverType solver(system, solve::Linear(frequency::Every(1))); // solve every 1, but only single linear step
 *
 *	SomeSolverType solver(system, TIncrementalSolveSetting(
 *		solve::linear, frequency::Every(1), // solve every 1, but only single linear step
 *		solve::nonlinear, frequency::Every(100))); // at the same time, every 100 steps, do nonlinear solve
 *
 *	SomeSolverType solver(system, TIncrementalSolveSetting(
 *		solve::linear, frequency::Every(1),
 *		solve::nonlinear, frequency::Every(100), 10, 0.1));
 *	// full-fledged specification with all the parameters
 *	@endcode
 *
 *	The incremental setting is now nicely packed into TIncrementalSolveSetting data type
 *	so it can be specified at a different place than where the solver is constructed:
 *	@code
 *	TIncrementalSolveSetting incremental_config = solve::Nonlinear(frequency::Every(1));
 *	// specify the incremental config
 *
 *	SomeSolverType solver(system, incremental_config);
 *	// initialize solver with that config
 *	@endcode
 */

/**
 *	@brief frequency setting
 */
struct TIncrementalFreqSetting {
	size_t n_period; /**< @brief period (inverse frequency), in steps */

	/**
	 *	@brief default constructor; sets frequency to 0 (never)
	 */
	inline TIncrementalFreqSetting()
		:n_period(0)
	{}

	/**
	 *	@brief constructor
	 *	@param[in] _n_period is period (inverse frequency; in steps)
	 */
	inline TIncrementalFreqSetting(size_t _n_period)
		:n_period(_n_period)
	{}
};

/**
 *	@brief namespace for frequency setting helper functions
 */
namespace frequency {

/**
 *	@brief gets update setting for update never
 *	@return Returns zero frequency.
 */
inline TIncrementalFreqSetting Never()
{
	return TIncrementalFreqSetting(0);
}

/**
 *	@brief gets update setting for update each N steps
 *	@param[in] n_period is period (inverse frequency; in steps)
 *	@return Returns frequency of 1 / n_period.
 */
inline TIncrementalFreqSetting Every(size_t n_period)
{
	return TIncrementalFreqSetting(n_period);
}

/**
 *	@brief gets update setting for update each step
 *	@return Returns frequency of 1.
 */
inline TIncrementalFreqSetting Always()
{
	return TIncrementalFreqSetting(1);
}

}; // ~frequency

/**
 *	@brief namespace for solver configuration helpers
 */
namespace solve {

/**
 *	@brief tag type for linear solving configuration
 */
struct linear_tag {};

/**
 *	@brief tag type for nonlinear solving configuration
 */
struct nonlinear_tag {};

/**
 *	@brief tag for linear solving configuration
 */
extern const linear_tag linear;

/**
 *	@brief tag for nonlinear solving configuration
 */
extern const nonlinear_tag nonlinear;

};

/**
 *	@brief incremental nonlinear solver setting
 */
struct TIncrementalSolveSetting {
	/**
	 *	@brief constants, stored as enum
	 */
	enum {
		default_NonlinearIteration_Num = 5, /**< @brief default maximum number of nonlinear solver iterations */
		default_NonlinearErrorThresh = 20, /**< @brief default error threshold for nonlinear solver update */
		default_ErrorThresh_Denom = 1 /**< @brief error threshold denominator (for the error threshold) */
	};

	TIncrementalFreqSetting t_linear_freq; /**< @brief linear solving frequency */
	TIncrementalFreqSetting t_nonlinear_freq; /**< @brief nonlinear solving frequency */
	size_t n_max_nonlinear_iteration_num; /**< @brief maximum number of nonlinear solver iterations */
	double f_nonlinear_error_thresh; /**< @brief error threshold for nonlinear solver update */

	/**
	 *	@brief batch constructor
	 */
	inline TIncrementalSolveSetting()
		:n_max_nonlinear_iteration_num(default_NonlinearIteration_Num),
		f_nonlinear_error_thresh(double(default_NonlinearErrorThresh) / default_ErrorThresh_Denom)
	{}

	/**
	 *	@brief linear incremental constructor
	 *
	 *	@param[in] t_tag is tag for tag based dispatch (value unused at runtime)
	 *	@param[in] t_freq is incremental solve frequency settings
	 */
	inline TIncrementalSolveSetting(solve::linear_tag UNUSED(t_tag), TIncrementalFreqSetting t_freq)
		:t_linear_freq(t_freq), n_max_nonlinear_iteration_num(default_NonlinearIteration_Num),
		f_nonlinear_error_thresh(double(default_NonlinearErrorThresh) / default_ErrorThresh_Denom)
	{}

	/**
	 *	@brief nonlinear incremental constructor
	 *
	 *	@param[in] t_tag is tag for tag based dispatch (value unused at runtime)
	 *	@param[in] t_freq is incremental solve frequency settings
	 *	@param[in] _n_max_nonlinear_iteration_num is the maximum number of nonlinear solver iterations
	 *	@param[in] _f_nonlinear_error_thresh is nonlinear solver residual norm threshold
	 */
	inline TIncrementalSolveSetting(solve::nonlinear_tag UNUSED(t_tag), TIncrementalFreqSetting t_freq,
		size_t _n_max_nonlinear_iteration_num = default_NonlinearIteration_Num,
		double _f_nonlinear_error_thresh = double(default_NonlinearErrorThresh) / default_ErrorThresh_Denom)
		:t_nonlinear_freq(t_freq), n_max_nonlinear_iteration_num(_n_max_nonlinear_iteration_num),
		f_nonlinear_error_thresh(_f_nonlinear_error_thresh)
	{}

	/**
	 *	@brief mixed linear/nonlinear incremental constructor
	 *
	 *	@param[in] t_tag0 is tag for tag based dispatch (value unused at runtime)
	 *	@param[in] t_linear_freq is incremental linear solve frequency settings
	 *	@param[in] t_tag1 is tag for tag based dispatch (value unused at runtime)
	 *	@param[in] t_nonlinear_freq is incremental nonlinear solve frequency settings
	 *	@param[in] _n_max_nonlinear_iteration_num is the maximum number of nonlinear solver iterations
	 *	@param[in] _f_nonlinear_error_thresh is nonlinear solver residual norm threshold
	 *
	 *	@note The opposite combination nonlinear first then linear is not allowed.
	 */
	inline TIncrementalSolveSetting(solve::linear_tag UNUSED(t_tag0),
		TIncrementalFreqSetting t_linear_freq, solve::nonlinear_tag UNUSED(t_tag1),
		TIncrementalFreqSetting t_nonlinear_freq,
		size_t _n_max_nonlinear_iteration_num = default_NonlinearIteration_Num,
		double _f_nonlinear_error_thresh = double(default_NonlinearErrorThresh) / default_ErrorThresh_Denom)
		:t_linear_freq(t_linear_freq), t_nonlinear_freq(t_nonlinear_freq),
		n_max_nonlinear_iteration_num(_n_max_nonlinear_iteration_num),
		f_nonlinear_error_thresh(_f_nonlinear_error_thresh)
	{}

	/**
	 *	@brief determines whether this configuration describes incremental solving
	 *	@return Returns true if this configuration describes incremental solving,
	 *		otherwise returns false.
	 */
	inline bool b_IsIncremental() const
	{
		return t_linear_freq.n_period != 0 || t_nonlinear_freq.n_period != 0;
	}

	/**
	 *	@brief determines whether this configuration describes batch solving
	 *	@return Returns true if this configuration describes batch solving,
	 *		otherwise returns false.
	 */
	inline bool b_IsBatch() const
	{
		return !b_IsIncremental();
	}
};

/*

now we can use:

TIncrementalSolveSetting(solve::linear, frequency::Every(1))
// linear solve every step

TIncrementalSolveSetting(solve::nonlinear, frequency::Every(1))
// nonlinear solve every step

TIncrementalSolveSetting(solve::nonlinear, frequency::Every(1), 10)
// nonlinear solve every step, max 10 iterations

TIncrementalSolveSetting(solve::nonlinear, frequency::Every(1), 10, 0.1)
// nonlinear solve every step, max 10 iterations, error threshold 0.1

TIncrementalSolveSetting(solve::linear, frequency::Every(1),
						 solve::nonlinear, frequency::Every(100))
// linear solve every 1, and nonlinear solve every 100

*/

/**
 *	@brief namespace for solver configuration helpers
 */
namespace solve {

/**
 *	@brief gets solver configuration for batch solving
 *	@return Returns solver configuration for batch solving (no incremental updates whatsoever).
 */
inline TIncrementalSolveSetting Batch()
{
	return TIncrementalSolveSetting();
}

/**
 *	@brief gets solver configuration for incremental linear-only solving
 *	@param[in] t_freq is frequency the solving is supposed to take place
 *	@return Returns solver configuration for incremental
 *		linear-only solving with the specified frequency.
 */
inline TIncrementalSolveSetting Linear(TIncrementalFreqSetting t_freq)
{
	return TIncrementalSolveSetting(linear, t_freq);
}

/**
 *	@brief gets solver configuration for incremental nonlinear solving
 *
 *	@param[in] t_freq is frequency the solving is supposed to take place
 *	@param[in] n_max_nonlinear_iteration_num is maximum number of nonlinear
 *		solver iterations (e.g. of Gauss-Newton iterations)
 *	@param[in] f_nonlinear_error_thresh is error threshold for nonlinear solver
 *
 *	@return Returns solver configuration for incremental
 *		nonlinear solving with the specified parameters.
 */
inline TIncrementalSolveSetting Nonlinear(TIncrementalFreqSetting t_freq,
	size_t n_max_nonlinear_iteration_num = TIncrementalSolveSetting::default_NonlinearIteration_Num,
	double f_nonlinear_error_thresh = double(TIncrementalSolveSetting::default_NonlinearErrorThresh) /
	TIncrementalSolveSetting::default_ErrorThresh_Denom)
{
	return TIncrementalSolveSetting(nonlinear, t_freq,
		n_max_nonlinear_iteration_num, f_nonlinear_error_thresh);
}

}; // ~solve

/*

now we can also use

solve::Batch()

solve::Nonlinear(frequency::Every(1))

and so on

*/

/**
 *	@brief namespace for marginal covariance calculation setting helpers
 */
namespace marginals {

/**
 *	@brief marginal covariance preference names
 */
enum {
	do_not_calculate = false, /**< @brief name of choice for not calculating the marginals */
	do_calculate = true /**< @brief name of choice for calculating the marginals */
};

} // ~marginals

/**
 *	@brief matrix part names
 *
 *	@note Do not use binary operations to subtract matrix parts, use n_MPart_Subtract() instead.
 *	@note To test matrix parts, use <tt>(a & mpart_Something) == mpart_Something</tt>
 */
enum EBlockMatrixPart {
	mpart_Nothing = 0, /**< @brief nothing */
	mpart_LastBlock = 1, /**< @brief the last block of the matrix (bottom-right) */
	mpart_Column = 2, /**< @brief a selected block column (which column it is depends on context) */
	mpart_LastColumn = 4, /**< @brief the last block column */
	mpart_Diagonal = 8, /**< @brief the block diagonal */
	mpart_FullMatrix = 1023 /**< @brief the full matrix (idempotent to combination with other flags) */
};

/**
 *	@brief subtracts matrix parts from selected matrix parts
 *
 *	@param[in] a is selected matrix parts
 *	@param[in] b is matrix parts to subtract
 *
 *	@return Returns the selected matrix parts without the subtracted matrix parts.
 */
static inline EBlockMatrixPart n_MPart_Subtract(EBlockMatrixPart a, EBlockMatrixPart b)
{
	if(a == mpart_FullMatrix && b != mpart_FullMatrix)
		return a;
	// full matrix minus anything but full matrix is still a full matrix

	return EBlockMatrixPart(a & ~b);
	// just bitwise subtract

	// if there was more flags that involve other flags, it would be even more complicated
}

/**
 *	@brief setting for incremental marginal covariance calculation
 */
struct TMarginalsComputationPolicy {
	/**
	 *	@brief constants, stored as enum
	 */
	enum {
		supported_MatrixPartFlags =	mpart_Nothing | mpart_LastBlock |
			mpart_LastColumn | mpart_Diagonal /**< @brief combination of supported matrix part types (+ mpart_FullMatrix) */
	};

	bool b_calculate; /**< @brief sets whether marginals will be calculated at all */
	TIncrementalFreqSetting t_increment_freq; /**< @brief sets frequency with which the marginals are calculated */
	EBlockMatrixPart n_incremental_policy; /**< @brief policy, setting behavior after ordinary incremental step */
	EBlockMatrixPart n_relinearize_policy; /**< @brief policy, setting behavior after changing the linearization point */
	EBlockMatrixPart n_cache_miss_policy; /**< @brief policy, setting behavior after user requested a part of covariance matrix, which is not calculated / up to date */

	/**
	 *	@brief default constructor; configures the policy
	 *
	 *	@param[in] b_calculate_marginals is marginals calculation flag (if set the marginals
	 *		can be calculated, if cleared the marginals will not be available and the rest
	 *		of the parameters is ignored)
	 *	@param[in] t_marginals_increment_freq is frequency for marginals calculation
	 *	@param[in] n_incremental_calculation_policy is covariance matrix part that should be
	 *		automatically calculated at each t_marginals_increment_freq steps, after ordinary
	 *		incremental step of the nonlinear solver
	 *	@param[in] n_relinearize_calculation_policy is covariance matrix part that should be
	 *		automatically calculated at each t_marginals_increment_freq steps, after linearization
	 *		point change (or other event preventing incremental calculation) took place in the
	 *		nonlinear solver
	 *	@param[in] n_cache_miss_policy is covariance matrix part that should be calculated
	 *		on demant when user requests a part of covariance matrix, which is not calculated / up
	 *		to date (note that calculation of the requested part is implied)
	 */
	inline TMarginalsComputationPolicy(bool b_calculate_marginals = marginals::do_not_calculate,
		TIncrementalFreqSetting t_marginals_increment_freq = frequency::Never(),
		EBlockMatrixPart n_incremental_calculation_policy = mpart_Nothing,
		EBlockMatrixPart n_relinearize_calculation_policy = mpart_Nothing,
		EBlockMatrixPart n_cache_miss_policy = mpart_Nothing)
		:b_calculate(b_calculate_marginals), t_increment_freq(t_marginals_increment_freq),
		n_incremental_policy(n_incremental_calculation_policy),
		n_relinearize_policy(n_relinearize_calculation_policy),
		n_cache_miss_policy(n_cache_miss_policy)
	{
		_ASSERTE(b_IsSupportedMatrixPart(n_incremental_calculation_policy));
		_ASSERTE(b_IsSupportedMatrixPart(n_relinearize_calculation_policy));
		_ASSERTE(n_cache_miss_policy == mpart_Column ||
			b_IsSupportedMatrixPart(n_cache_miss_policy)); // column selected by query, therefore mpart_Column is permissible
	}

	/**
	 *	@brief changes incremental calculation policy
	 *	@param[in] n_new_policy is covariance matrix part that should be calculated incrementally
	 *	@return Returns reference to this, so that chaining of these commands is possible.
	 */
	inline TMarginalsComputationPolicy &OnIncrement_Calculate(EBlockMatrixPart n_new_policy)
	{
		n_incremental_policy = n_new_policy;
		return *this;
	}

	/**
	 *	@brief changes post-linearization-point-change calculation policy
	 *	@param[in] n_new_policy is covariance matrix part that should be calculated after relinearization
	 *	@return Returns reference to this, so that chaining of these commands is possible.
	 */
	inline TMarginalsComputationPolicy &OnRelinearize_Calculate(EBlockMatrixPart n_new_policy)
	{
		n_relinearize_policy = n_new_policy;
		return *this;
	}

	/**
	 *	@brief changes cache miss calculation policy
	 *	@param[in] n_new_policy is covariance matrix part that should be calculated incrementally
	 *	@return Returns reference to this, so that chaining of these commands is possible.
	 */
	inline TMarginalsComputationPolicy &OnCacheMiss_Calculate(EBlockMatrixPart n_new_policy)
	{
		n_cache_miss_policy = n_new_policy;
		return *this;
	}

	/**
	 *	@brief determines whether a specified matrix part is combination of supported matrix parts
	 *	@param[in] n_part is combination of mpart_* (see EBlockMatrixPart)
	 *	@return Returns true if the specified matrix part is combination
	 *		of supported matrix parts, otherwise returns false.
	 */
	static inline bool b_IsSupportedMatrixPart(EBlockMatrixPart n_part)
	{
		return n_part == mpart_FullMatrix || !(n_part & ~supported_MatrixPartFlags);
	}
};

/*

can have:

CMarginalsComputationPolicy(mpart_LastBlock, mpart_Diagonal, mpart_Nothing)
that calculates the last block incrementally, ater relinearization recalculates only the diagonal
and when querying for something that is not calculated yet, it calculates nothing more than it has to

CMarginalsComputationPolicy(mpart_LastColumn, mpart_FullMatrix, mpart_Nothing)
that calculates the last column incrementally, on relinearization calculates full matrix,
and when querying for something that is not calculated yet, it calculates nothing more than it has to

CMarginalsComputationPolicy(mpart_FullMatrix, mpart_FullMatrix, mpart_Nothing)
alwawys have full matrix at hand, no matter what happens (value of the last arg is not important, can be any)

CMarginalsComputationPolicy(mpart_Nothing, mpart_Nothing, mpart_FullMatrix)
does nothing in the incremental steps, when we access the marginal matrix, it calculates all of it

CMarginalsComputationPolicy(mpart_Nothing, mpart_Nothing, mpart_Column)
similar, but it calculates the selected column

CMarginalsComputationPolicy(mpart_Nothing, mpart_Nothing, mpart_Column + mpart_LastColumn + mpart_Diagonal)
similar, but it calculates the selected column, the last column, and the diagonal

CMarginalsComputationPolicy(mpart_LastColumn + mpart_Diagonal, mpart_LastColumn + mpart_Diagonal, mpart_Nothing)
incrementally maintain the last column and the diagonal so it is always available

there could also be storage policy, which would dictate what is stored

there could be alternate syntax, like:

CMarginalsComputationPolicy policy(marginals::do_calculate, frequency::Every(100));
policy.OnIncrement_Calculate(mpart_LastBlock).
	OnRelinearize_Calculate(mpart_FullMatrix).
	OnCacheMiss_Calculate(mpart_Nothing);

or simply

CMarginalsComputationPolicy policy(marginals::do_not_calculate);

that is maybe nicer to read, probably the same complexity from the point of writing the code

*/

#endif // !__INCREMENTAL_POLICIES_INCLUDED
