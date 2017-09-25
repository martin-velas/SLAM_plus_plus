/*
								+-----------------------------------+
								|                                   |
								| *** CXSparse template wrapper *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2016  |
								|                                   |
								|              cts.hpp              |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __CSPARSE_TEMPLATE_INCLUDED
#define __CSPARSE_TEMPLATE_INCLUDED

/**
 *	@file include/sparse_flops/cts.hpp
 *	@brief C++ template wrapper around CXSparse
 *	@date 2016
 *	@author -tHE SWINe-
 *
 *	This small file adds a template class that wraps cxsparse and allows instantiation
 *	with many different types at once. Note that this only works with C++.
 *
 *	@note In Windows, the CXSparse library does not require modifications, with the exception
 *		of addition of this file. To make this work in Linux, one needs to resolve the missing
 *		explicit casts from <tt>void*</tt> to <tt>double*</tt> or <tt>int*</tt> which "C" does
 *		not require but C++ does. This amounts to replacing expressions of the type
 *		<span class="code-span">`x = values? cs_calloc(...) : NULL ;`</span> to
 *		<span class="code-span">`x = values? (CS_ENTRY*)cs_calloc(...) : (CS_ENTRY*)NULL ;`</span>.
 *		We did this modification, in addition to changing the includes from <tt>"cs.h"</tt>
 *		to <tt>"cxsparse/cs.h"</tt> in order to avoid having to add <tt>"include/cxsparse"</tt>
 *		into the include path of each project that uses this. The fixes were made at:
 *		* cs_add.c, line 14
 *		* cs_multiply.c, line 14
 *		* cs_util.c, line 13
 *	@note There are some issues with using other types than <tt>double</tt>, as CXSparse still
 *		has it hardcoded as a) type for large numbers and b) real data type. Additinal patches
 *		are required in case a type other than <tt>double</tt> is used. This is handled by some
 *		preprocessor hacks and no additional modifications to CXSparse are needed, see
 *		\ref __CSPARSE_TEMPLATE_PATCH_REAL_TYPE.
 *	@note Additionally, some annoying Visual Studio warnings were fixed by adding explicit casts
 *		while taking care of not changing the behavior, at:
 *		* cs_amd.c, line 31 - fixed a warning
 *		* cs_lu.c, line 12 - fixed a warning
 *		* cs_qr.c, line 14 - fixed a warning
 *		* cs_sqr.c, line 84 - fixed a warning in x64 mode
 *	@note This was tested with CXSparse 2.3, changes may be needed to work with a newer / older version.
 */

/**
 *	@def __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
 *	@brief enables CXSparse CS_REAL_TYPE patch, re-defining <tt>double</tt> in a few CXSparse functions below
 *
 *	CSparse uses <tt>double</tt> in place of a real type. Not all the real types may however
 *	be convertible to real, or <tt>double</tt> may be too precise. This patch changes the following
 *	functions to use a new CS_REAL_TYPE type rather than double:
 *
 *	* \ref cs_droptol()
 *	* \ref cs_happly()
 *	* \ref cs_house()
 *	* \ref cs_lu()
 *	* \ref cs_lusol()
 *	* \ref cs_norm()
 *	* \ref cs_qr()
 *	* \ref cs_updown()
 *
 *	Additionally, \ref cs_updown() is reimplemented below, to work with complex numbers.
 */
#define __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#include "csparse/cs.hpp" // the "original", for conversions
#include "slam/TypeList.h" // CEqualType, CChooseType, CEnableIf

#define INSIDE_CSPARSE_TEMPLATE_HPP
#ifndef _CXS_H
#define _CXS_H_WAS_NOT_DEFINED
#define _CXS_H // want to avoid the .c files including cs.h -- rather want them to become templates
#endif // !_CXS_H
#ifndef NCOMPLEX
#define NCOMPLEX_WAS_NOT_DEFINED
#define NCOMPLEX // want NCOMPLEX
#endif // !NCOMPLEX
#ifdef MATLAB_MEX_FILE
#define MATLAB_MEX_WAS_DEFINED
#undef MATLAB_MEX_FILE // don't want MATLAB_MEX_FILE
#endif // MATLAB_MEX_FILE
// handle "C" macros

#ifdef CS_REAL
#undef CS_REAL
#undef CS_IMAG
#undef CS_CONJ
#undef CS_ABS
#endif // CS_REAL
#define CS_REAL(x) (csparse_detail::cxs_real(x))
#define CS_IMAG(x) (csparse_detail::cxs_imag(x))
#define CS_CONJ(x) (csparse_detail::cxs_conj(x))
#define CS_ABS(x) (csparse_detail::cxs_abs(x))
// allow overloading those for complex types

#ifndef CS_MAX
#define CS_MAX(a,b) (((a) > (b)) ? (a) : (b))
#define CS_MIN(a,b) (((a) < (b)) ? (a) : (b))
#define CS_FLIP(i) (-(i)-2)
#define CS_UNFLIP(i) (((i) < 0) ? CS_FLIP(i) : (i))
#define CS_MARKED(w,j) (w [j] < 0)
#define CS_MARK(w,j) { w [j] = CS_FLIP (w [j]) ; }
#define CS_CSC(A) (A && (A->nz == -1))
#define CS_TRIPLET(A) (A && (A->nz >= 0))
#endif // !CS_MAX
// define CSparse macros if not there yet

#ifdef CS_VER
#undef CS_VER
#undef CS_SUBVER
#undef CS_SUBSUB
#undef CS_DATE
#undef CS_COPYRIGHT
#endif // CS_VER
// remove colliding defines, noone uses them anyway

#define CXS_VER 2                    /* CXSparse Version */
#define CXS_SUBVER 3
#define CXS_SUBSUB 0
#define CXS_DATE "Jun 1, 2012"       /* CXSparse release date */
#define CXS_COPYRIGHT "Copyright (c) Timothy A. Davis, 2006-2012"
// define CX version, just to retain the copyright and stuffs

#undef cs
#undef cs_add
#undef cs_cholsol
#undef cs_dupl
#undef cs_entry
#undef cs_lusol
#undef cs_gaxpy
#undef cs_multiply
#undef cs_qrsol
#undef cs_transpose
#undef cs_compress
#undef cs_norm
#undef cs_print
#undef cs_load
#undef cs_calloc
#undef cs_free
#undef cs_realloc
#undef cs_spalloc
#undef cs_spfree
#undef cs_sprealloc
#undef cs_malloc
#undef css
#undef csn
#undef csd
#undef cs_amd
#undef cs_chol
#undef cs_dmperm
#undef cs_droptol
#undef cs_dropzeros
#undef cs_happly
#undef cs_ipvec
#undef cs_lsolve
#undef cs_ltsolve
#undef cs_lu
#undef cs_permute
#undef cs_pinv
#undef cs_pvec
#undef cs_qr
#undef cs_schol
#undef cs_sqr
#undef cs_symperm
#undef cs_usolve
#undef cs_utsolve
#undef cs_updown
#undef cs_sfree
#undef cs_nfree
#undef cs_dfree
#undef cs_counts
#undef cs_cumsum
#undef cs_dfs
#undef cs_etree
#undef cs_fkeep
#undef cs_house
#undef cs_invmatch
#undef cs_maxtrans
#undef cs_post
#undef cs_scc
#undef cs_scatter
#undef cs_tdfs
#undef cs_reach
#undef cs_spsolve
#undef cs_ereach
#undef cs_randperm
#undef cs_leaf
#undef cs_dalloc
#undef cs_done
#undef cs_idone
#undef cs_ndone
#undef cs_ddone
// undefine all the symbols

#include <complex>
#include <limits>

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

/**
 *	@brief complex to int comparison (required by cs_house() which compares CS_ENTRY to integer 0)
 *
 *	@tparam S is scalar type
 *
 *	@param[in] x is the left side value
 *	@param[in] y is the right side value
 *
 *	@return Returns true if the values are equal, otherwise returns false.
 */
template <class S>
inline typename CEnableIf<!CEqualType<S, int>::b_result, bool>::T operator ==(std::complex<S> x, int y)
{
	return x.real() == y && !x.imag();
}

/**
 *	@brief complex to int comparison (required by cs_house() which compares CS_ENTRY to integer 0)
 *
 *	@tparam S is scalar type
 *
 *	@param[in] x is the left side value
 *	@param[in] y is the right side value
 *
 *	@return Returns false if the values are equal, otherwise returns true.
 */
template <class S>
inline typename CEnableIf<!CEqualType<S, int>::b_result, bool>::T operator !=(std::complex<S> x, int y)
{
	return !(x == y);
}

// note that complex<T> to T comparisons are handled by STL
// also note that these functions are disabled for complex<int>, to avoid ambiguity

#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

/**
 *	@brief namespace with internal details of the implementation
 */
namespace csparse_detail {

/**
 *	@brief gets real component of a value
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the real component of the given value.
 */
template <class T>
static inline T cxs_real(T x)
{
	return x;
}

/**
 *	@brief gets imaginary component of a value
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the imaginary component of the given value.
 */
template <class T>
static inline T cxs_imag(T UNUSED(x))
{
	return T(0);
}

/**
 *	@brief gets complex conjugate of a value
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the complex conjugate of the given value.
 */
template <class T>
static inline T cxs_conj(T x)
{
	return x;
}

/**
 *	@brief gets magnitude of a value
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the magnitude of the given value.
 */
template <class T>
static inline T cxs_abs(T x)
{
	return fabs(x);
}

/**
 *	@brief gets real component of a value (specialization for complex numbers)
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the real component of the given value.
 */
template <class T>
static inline T cxs_real(std::complex<T> x)
{
	return std::real(x);
}

/**
 *	@brief gets imaginary component of a value (specialization for complex numbers)
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the imaginary component of the given value.
 */
template <class T>
static inline T cxs_imag(std::complex<T> x)
{
	return std::imag(x);
}

/**
 *	@brief gets complex conjugate of a value (specialization for complex numbers)
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the complex conjugate of the given value.
 */
template <class T>
static inline std::complex<T> cxs_conj(std::complex<T> x)
{
	return std::conj(x);
}

/**
 *	@brief gets magnitude of a value (specialization for complex numbers)
 *	@tparam T is type of the value
 *	@param[in] x is the input value
 *	@return Returns the magnitude of the given value.
 */
template <class T>
static inline T cxs_abs(std::complex<T> x)
{
	return std::abs(x);
}

/**
 *	@brief determines whether a type is a specialization of std::complex
 *	@tparam T is type to be checked for being std::complex
 */
template <class T>
class CIsComplex {
public:
	/**
	 *	@brief result stored as enum
	 */
	enum {
		b_result = false /**< @brief the result */
	};
};

/**
 *	@brief determines whether a type is a specialization of std::complex (specialization for complex types)
 *	@tparam T is type to be checked for being std::complex
 */
template <class T>
class CIsComplex<std::complex<T> > {
public:
	/**
	 *	@brief result stored as enum
	 */
	enum {
		b_result = true /**< @brief the result */
	};
};

/**
 *	@brief static assertion helper
 *	@brief b_expression is expression being asserted
 */
template <const bool b_expression>
class CStaticAssert {};

/**
 *	@brief static assertion helper (specialization for assertion passed)
 */
template <>
class CStaticAssert<true> {
public:
	typedef void INDEX_TYPE_MUST_BE_SIGNED; /**< @brief static assertion tag; index types of CXSparse matrices must be signed integers */
	typedef void REAL_TYPE_MISMATCH; /**< @brief static assertion tag; the CS_REAL_TYPE must match CS_ENTRY in case CS_ENTRY is real, or base type of CS_ENTRY in case it is complex */
	typedef void REAL_TYPE_MUST_NOT_BE_COMPLEX; /**< @brief static assertion tag; CS_REAL_TYPE must not be complex */
};

} // ~csparse_detail

/**
 *	@brief CSparse instantiation template
 *
 *	@tparam CS_ENTRY is scalar data type (default double)
 *	@tparam CS_INT is integer data type (default long)
 *	@tparam CS_REAL_TYPE is real scalar data type (default the same as CS_ENTRY)
 */
template <class CS_ENTRY = double, class CS_INT = csi, class CS_REAL_TYPE = CS_ENTRY>
class CTSparse {
public:
	typedef CS_ENTRY _TyScalar; /**< @brief scalar type */
	typedef CS_INT _TyInt; /**< @brief int type */
	typedef CS_REAL_TYPE _TyRealScalar; /**< @brief real scalar type, in case _TyScalar is a complex type */

	/**
	 *	@brief matrix in compressed-column or triplet form
	 */
	struct cs {
		_TyInt nzmax;     /**< @brief maximum number of entries */
		_TyInt m;         /**< @brief number of rows */
		_TyInt n;         /**< @brief number of columns */
		_TyInt *p;        /**< @brief column pointers (size n+1) or col indices (size nzmax) */
		_TyInt *i;        /**< @brief row indices, size nzmax */
		_TyScalar *x;     /**< @brief numerical values, size nzmax */
		_TyInt nz;        /**< @brief number of entries in triplet matrix, -1 for compressed-col */
	};

	// this is a bit fragile; cs must be defined above, otherwise g++ fails with:
	// cts.hpp:333: error: declaration of "struct CTSparse<CS_ENTRY, CS_INT, CS_REAL_TYPE>::cs"
	// cs.h:44: error: changes meaning of "cs" from "typedef struct cs_sparse cs"

	/**
	 *	@brief properties, stored as enum
	 */
	enum {
		b_is_complex = csparse_detail::CIsComplex<CS_ENTRY>::b_result, /**< @brief determines whether complex type is used for matrix entries */
		b_real_is_complex = csparse_detail::CIsComplex<CS_REAL_TYPE>::b_result, /**< @brief checks whether real type is indeed real */
		b_real_type_same_as_entry_type = CEqualType<_TyScalar, _TyRealScalar>::b_result, /**< @brief determines whether matrix entries type is the same as real type */
		b_has_signed_indices = std::numeric_limits<int>::is_integer && std::numeric_limits<int>::is_signed/*double(CS_INT(-1)) < 0*/, /**< @brief determines whether the index type is a signed one */
		b_is_CSparse_compatible = !b_is_complex &&
			sizeof(::cs) == sizeof(cs) &&
			sizeof(_TyInt) == sizeof(csi) &&
			sizeof(CS_ENTRY) == sizeof(double) /*&&
			CEqualType<CS_ENTRY, double>::b_result*/ /**< @brief "original" CSparse binary compatibility flag */ // do not compare the type, want to allow type puning
	};

	typedef typename csparse_detail::CStaticAssert<b_has_signed_indices>::INDEX_TYPE_MUST_BE_SIGNED CAssert0; /**< @brief compile-type assertion; if this triggers, you are using unsigned type for indices and CXSparse will not work with that */
	typedef typename csparse_detail::CStaticAssert<b_is_complex ||
		b_real_type_same_as_entry_type>::REAL_TYPE_MISMATCH CAssert1; /**< @brief compile-type assertion; in case CS_ENTRY is real, CS_REAL_TYPE must be the same type */
	typedef typename csparse_detail::CStaticAssert<!b_real_is_complex>::REAL_TYPE_MUST_NOT_BE_COMPLEX CAssert2; /**< @brief compile-type assertion; CS_REAL_TYPE must be a real type */

	/**
	 *	@brief symbolic Cholesky, LU, or QR analysis
	 */
	struct css {
		_TyInt *pinv;     /**< @brief inverse row perm. for QR, fill red. perm for Chol */
		_TyInt *q;        /**< @brief fill-reducing column permutation for LU and QR */
		_TyInt *parent;   /**< @brief elimination tree for Cholesky and QR */
		_TyInt *cp;       /**< @brief column pointers for Cholesky, row counts for QR */
		_TyInt *leftmost; /**< @brief leftmost[i] = min(find(A(i,:))), for QR */
		_TyInt m2;        /**< @brief number of rows for QR, after adding fictitious rows */
		double lnz;       /**< @brief number of entries in L for LU or Cholesky; in V for QR */
		double unz;       /**< @brief number of entries in U for LU; in R for QR */
		// note that the two above are supposed to be double (those are used to store some long integers)
	};

	/**
	 *	@brief numeric Cholesky, LU, or QR factorization
	 */
	struct csn {
		cs *L;      /**< @brief L for LU and Cholesky, V for QR */
		cs *U;      /**< @brief U for LU, r for QR, not used for Cholesky */
		_TyInt *pinv;     /**< @brief partial pivoting for LU */
		_TyRealScalar *B;     /**< @brief beta [0..n-1] for QR */ // note that CXSparse does not want this a different type // t_odo - gather those and add a real type (this is intended to not be an imaginary type)
	};

	/**
	 *	@brief cs_dmperm() or cs_scc() output
	 */
	struct csd {
		_TyInt *p;       /**< @brief size m, row permutation */
		_TyInt *q;       /**< @brief size n, column permutation */
		_TyInt *r;       /**< @brief size nb+1, block k is rows r[k] to r[k+1]-1 in A(p,q) */
		_TyInt *s;       /**< @brief size nb+1, block k is cols s[k] to s[k+1]-1 in A(p,q) */
		_TyInt nb;       /**< @brief number of of blocks in fine dmperm decomposition */
		_TyInt rr[5];    /**< @brief coarse row decomposition */
		_TyInt cc[5];    /**< @brief coarse column decomposition */
	};

	/**
	 *	@brief names of the ordering heuristics
	 *	@note This can be used in \ref amd(), \ref schol(),
	 *		\ref cholsol(), \ref sqr(), \ref lusol() and \ref qrsol().
	 */
	enum {
		order_Natural = 0, /**< @brief natural matrix ordering */
		order_AMD_Chol, /**< @brief AMD ordering for Cholesky */
		order_AMD_LU, /**< @brief AMD ordering for LU */
		order_COLAMD_QR /**< @brief COLAMD ordering for QR */
	};

protected:
	/**
	 *	@brief a helper type to effectively disable \ref p_ToSparse() and \ref p_FromSparse().
	 */
	struct CONVERSION_NOT_AVAILABLE {};

	/**
	 *	@brief pointer with permissible conversions, used as a result of <tt>cs_*alloc()</tt>
	 *
	 *	(in "C", <tt>void*</tt> converts to anything, but not in C++: need to fix this)
	 *
	 *	@note This does not implement arithmetics, seems not needed (it is converted to a standard
	 *		pointer as soon as it leaves CXSparse implementation so usets of this class actually
	 *		never directly use this).
	 */
	class CAllocResult {
	protected:
		void *m_p_ptr; /**< value of the pointer */

	public:
		/**
		 *	@brief default constructor; initializes the pointer
		 *	@param[in] p_ptr is value of the pointer
		 */
		CAllocResult(void *p_ptr = 0)
			:m_p_ptr(p_ptr)
		{}

		/**
		 *	@brief logical negation operator
		 *	@return Returns true if this is a null pointer, otherwise returns false.
		 */
		inline bool operator !() const
		{
			return !m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>void</tt> pointer
		 *	@return Returns this pointer, cast to <tt>void*</tt>.
		 */
		inline operator void*()
		{
			return m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>_TyInt</tt> pointer
		 *	@return Returns this pointer, cast to <tt>_TyInt*</tt>.
		 */
		inline operator _TyInt*()
		{
			return (_TyInt*)m_p_ptr;
		}

	protected:
		struct InaccessibleType {}; /**< @brief an inaccessible type */
		typedef typename CChooseType<InaccessibleType*, _TyRealScalar*,
			CEqualType<_TyRealScalar, _TyScalar>::b_result>::_TyResult _TyRealConversionOp; /**< @brief enables or disables conversion to _TyRealScalar*, based on whether _TyRealScalar is the same as _TyScalar or not */

	public:
		/**
		 *	@brief conversion to <tt>_TyScalar</tt> pointer
		 *	@return Returns this pointer, cast to <tt>_TyScalar*</tt>.
		 */
		inline operator _TyScalar*()
		{
			return (_TyScalar*)m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>_TyRealScalar</tt> pointer
		 *	@return Returns this pointer, cast to <tt>_TyRealScalar*</tt>.
		 *	@note This is used in the QR factorization to hold the (always real) Householder coeffs.
		 */
		inline operator _TyRealConversionOp()
		{
			return (_TyRealConversionOp)m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>cs</tt> pointer
		 *	@return Returns this pointer, cast to <tt>cs*</tt>.
		 *	@note This is required by \ref cs_spalloc().
		 */
		inline operator cs*()
		{
			return (cs*)m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>css</tt> pointer
		 *	@return Returns this pointer, cast to <tt>css*</tt>.
		 *	@note This is required by \ref cs_spalloc().
		 */
		inline operator css*()
		{
			return (css*)m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>cs<n/tt> pointer
		 *	@return Returns this pointer, cast to <tt>csn*</tt>.
		 *	@note This is required by \ref cs_spalloc().
		 */
		inline operator csn*()
		{
			return (csn*)m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>csd</tt> pointer
		 *	@return Returns this pointer, cast to <tt>csd*</tt>.
		 *	@note This is required by \ref cs_spalloc().
		 */
		inline operator csd*()
		{
			return (csd*)m_p_ptr;
		}

		/**
		 *	@brief conversion to <tt>long</tt>
		 *	@return Returns this pointer, cast to <tt>long</tt>.
		 *	@note This is required by ternary operator.
		 */
		inline operator long int()
		{
			return (long int)m_p_ptr;
		}
	};

public:
	/* --- conversion to CSparse ------------------------- */

	/**
	 *	@brief determines whether the \ref cs type is compatible with \ref ::cs type
	 *	@return Returns true if it is possible to perform type puning between
	 *		the \ref cs and the \ref ::cs type.
	 */
	static bool b_CSparse_Compatible()
	{
		return b_is_CSparse_compatible;
	}

	/**
	 *	@brief converts a pointer to the \ref cs type to a value that the "original" CSparse can work with
	 *	@param[in] p_cs is pointer to an object of the \ref cs type
	 *	@return Returns the same pointer.
	 *	@note This function is only available in case the two types are the same in memory.
	 */
	static ::cs *p_ToSparse(typename CChooseType<cs,
		CONVERSION_NOT_AVAILABLE, b_is_CSparse_compatible>::_TyResult *p_cs) // can't use CEnableIf as this is not a template
	{
		//if(!b_CSparse_Compatible())
		//	throw std::runtime_error("templated CSparse not compatible with CSparse, conversion attempted");
		return (::cs*)(p_cs);
	}

	/**
	 *	@brief converts a const pointer to the \ref cs type to a value that the "original" CSparse can work with
	 *	@param[in] p_cs is const pointer to an object of the \ref cs type
	 *	@return Returns the same pointer.
	 *	@note This function is only available in case the two types are the same in memory.
	 */
	static ::cs *p_ToSparse(const typename CChooseType<cs,
		CONVERSION_NOT_AVAILABLE, b_is_CSparse_compatible>::_TyResult *p_cs)
	{
		//if(!b_CSparse_Compatible())
		//	throw std::runtime_error("templated CSparse not compatible with CSparse, conversion attempted");
		return (::cs*)(p_cs);
	}

	/**
	 *	@brief converts a pointer of the "original" CSparse to a pointer to the templated \ref cs type
	 *	@param[in] p_cs is pointer to an object of the "original" CSparse \ref ::cs type
	 *	@return Returns the same pointer.
	 *	@note This function is only available in case the two types are the same in memory.
	 */
	static cs *p_FromSparse(typename CChooseType<CONVERSION_NOT_AVAILABLE,
		::cs, !b_is_CSparse_compatible>::_TyResult *p_cs) // note the changed template argument order, this is to avoid forming a tri-graph
	{
		//if(!b_CSparse_Compatible())
		//	throw std::runtime_error("templated CSparse not compatible with CSparse, conversion attempted");
		return (cs*)(p_cs);
	}

	/**
	 *	@brief converts a const pointer of the "original" CSparse to a pointer to the templated \ref cs type
	 *	@param[in] p_cs is const pointer to an object of the "original" CSparse \ref ::cs type
	 *	@return Returns the same pointer.
	 *	@note This function is only available in case the two types are the same in memory.
	 */
	static const cs *p_FromSparse(const typename CChooseType<CONVERSION_NOT_AVAILABLE,
		::cs, !b_is_CSparse_compatible>::_TyResult *p_cs) // note the changed template argument order, this is to avoid forming a tri-graph
	{
		//if(!b_CSparse_Compatible())
		//	throw std::runtime_error("templated CSparse not compatible with CSparse, conversion attempted");
		return (cs*)(p_cs);
	}

	// todo - doc

	/* --- primary CSparse routines and data structures ------------------------- */

	static inline cs *add(const cs *A, const cs *B, _TyScalar alpha, _TyScalar beta) { return TImpl::cs_add(A, B, alpha, beta); }
	static inline _TyInt cholsol(_TyInt order, const cs *A, _TyScalar *b) { return TImpl::cs_cholsol(order, A, b); }
	static inline _TyInt dupl(cs *A) { return TImpl::cs_dupl(A); }
	static inline _TyInt entry(cs *T, _TyInt i, _TyInt j, _TyScalar x) { return TImpl::cs_entry(T, i, j, x); }
	static inline _TyInt lusol(_TyInt order, const cs *A, _TyScalar *b, _TyRealScalar tol) { return TImpl::cs_lusol(order, A, b, tol); }
	static inline _TyInt gaxpy(const cs *A, const _TyScalar *x, _TyScalar *y) { return TImpl::cs_gaxpy(A, x, y); }
	static inline cs *multiply(const cs *A, const cs *B) { return TImpl::cs_multiply(A, B); }
	static inline _TyInt qrsol(_TyInt order, const cs *A, _TyScalar *b) { return TImpl::cs_qrsol(order, A, b); }
	static inline cs *transpose(const cs *A, _TyInt values) { return TImpl::cs_transpose(A, values); }
	static inline cs *compress(const cs *T) { return TImpl::cs_compress(T); }
	static inline _TyScalar norm(const cs *A) { return TImpl::cs_norm(A); }
	//static inline _TyInt print(const cs *A, _TyInt brief) { return TImpl::cs_print(A, brief); } // not implemented
	//static inline cs *load(FILE *f) { return TImpl::cs_load(f); } // not implemented

	/* utilities */
	static inline void *calloc(_TyInt n, size_t size) { return TImpl::cs_calloc(n, size); }
	static inline void *free(void *p) { return TImpl::cs_free(p); }
	static inline void *realloc(void *p, _TyInt n, size_t size, _TyInt *ok) { return TImpl::cs_realloc(p, n, size, ok); }
	static inline void *realloc(void *p, _TyInt n, size_t size, bool &ok) { _TyInt n_ok; void *p_result = TImpl::cs_realloc(p, n, size, &n_ok); ok = !!n_ok; return p_result; }
	static inline cs *spalloc(_TyInt m, _TyInt n, _TyInt nzmax, _TyInt values, _TyInt t) { return TImpl::cs_spalloc(m, n, nzmax, values, t); }
	static inline cs *spfree(cs *A) { return TImpl::cs_spfree(A); }
	static inline _TyInt sprealloc(cs *A, _TyInt nzmax) { return TImpl::cs_sprealloc(A, nzmax); }
	static inline void *malloc(_TyInt n, size_t size) { return TImpl::cs_malloc(n, size); }

	/* --- secondary CSparse routines and data structures ----------------------- */

	/**
	 *	@brief calculate approximate minimum degree (AMD) ordering
	 *
	 *	@param[in] order is ordering type (0: natural, 1: Chol, 2: LU, 3: QR)
	 *	@param[in] A is the matrix for analysis
	 *
	 *	@return Returns a pointer to the ordering vector, the caller is responsible for deleting it.
	 */
	static inline _TyInt *amd(_TyInt order, const cs *A) { return TImpl::cs_amd(order, A); }
	static inline csn *chol(const cs *A, const css *S) { return TImpl::cs_chol(A, S); }
	static inline csd *dmperm(const cs *A, _TyInt seed) { return TImpl::cs_dmperm(A, seed); }
	static inline _TyInt droptol(cs *A, _TyRealScalar tol) { return TImpl::cs_droptol(A, tol); }
	static inline _TyInt dropzeros(cs *A) { return TImpl::cs_dropzeros(A); }
	static inline _TyInt happly(const cs *V, _TyInt i, _TyRealScalar beta, _TyScalar *x) { return TImpl::cs_happly(V, i, beta, x); }
	static inline _TyInt ipvec(const _TyInt *p, const _TyScalar *b, _TyScalar *x, _TyInt n) { return TImpl::cs_ipvec(p, b, x, n); }
	static inline _TyInt lsolve(const cs *L, _TyScalar *x) { return TImpl::cs_lsolve(L, x); }
	static inline _TyInt ltsolve(const cs *L, _TyScalar *x) { return TImpl::cs_ltsolve(L, x); }
	static inline csn *lu(const cs *A, const css *S, _TyRealScalar tol) { return TImpl::cs_lu(A, S, tol); }
	static inline cs *permute(const cs *A, const _TyInt *pinv, const _TyInt *q,
		_TyInt values) { return TImpl::cs_permute(A, pinv, q, values); }
	static inline _TyInt *pinv(const _TyInt *p, _TyInt n) { return TImpl::cs_pinv(p, n); }
	static inline _TyInt pvec(const _TyInt *p, const _TyScalar *b, _TyScalar *x, _TyInt n) { return TImpl::cs_pvec(p, b, x, n); }
	static inline csn *qr(const cs *A, const css *S) { return TImpl::cs_qr(A, S); }

	/**
	 *	@brief symbolic ordering and analysis for Cholesky
	 *
	 *	@param[in] order is ordering type (0: natural, 1: Chol, 2: LU, 3: QR)
	 *	@param[in] A is the matrix for analysis
	 *
	 *	@return Returns symbolic factorization of the given matrix.
	 *
	 *	@note In case order is non-zero, the input matrix is permuted twice (once for
	 *		analysis and then later again for numerical factorization).
	 */
	static inline css *schol(_TyInt order, const cs *A) { return TImpl::cs_schol(order, A); }

	/**
	 *	@brief symbolic ordering and analysis for QR or LU
	 *
	 *	@param[in] order is ordering type (0: natural, 1: Chol, 2: LU, 3: QR)
	 *	@param[in] A is the matrix for analysis
	 *	@param[in] qr is QR flag (nonzero for QR, zero for LU)
	 *
	 *	@return Returns symbolic factorization of the given matrix.
	 *
	 *	@note In case order is non-zero, the input matrix is permuted twice (once for
	 *		analysis and then later again for numerical factorization).
	 */
	static inline css *sqr(_TyInt order, const cs *A, _TyInt qr) { return TImpl::cs_sqr(order, A, qr); }

	static inline cs *symperm(const cs *A, const _TyInt *pinv, _TyInt values) { return TImpl::cs_symperm(A, pinv, values); }
	static inline _TyInt usolve(const cs *U, _TyScalar *x) { return TImpl::cs_usolve(U, x); }
	static inline _TyInt utsolve(const cs *U, _TyScalar *x) { return TImpl::cs_utsolve(U, x); }
	static inline _TyInt updown(cs *L, _TyInt sigma, const cs *C,
		const _TyInt *parent) { return TImpl::cs_updown(L, sigma, C, parent); }

	/* utilities */
	static inline css *sfree(css *S) { return TImpl::cs_sfree(S); }
	static inline csn *nfree(csn *N) { return TImpl::cs_nfree(N); }
	static inline csd *dfree(csd *D) { return TImpl::cs_dfree(D); }

	/* --- tertiary CSparse routines -------------------------------------------- */

	static inline _TyInt *counts(const cs *A, const _TyInt *parent,
		const _TyInt *post, _TyInt ata) { return TImpl::cs_counts(A, parent, post, ata); }
	static inline _TyScalar cumsum(_TyInt *p, _TyInt *c, _TyInt n) { return TImpl::cs_cumsum(p, c, n); }
	static inline _TyInt dfs(_TyInt j, cs *G, _TyInt top, _TyInt *xi,
		_TyInt *pstack, const _TyInt *pinv) { return TImpl::cs_dfs(j, G, top, xi, pstack, pinv); }
	static inline _TyInt *etree(const cs *A, _TyInt ata) { return TImpl::cs_etree(A, ata); }
	static inline _TyInt fkeep(cs *A,
		_TyInt(*p_fkeep)(_TyInt, _TyInt, _TyScalar, void *), void *other) { return TImpl::cs_fkeep(A, p_fkeep, other); }
	static inline _TyScalar house(_TyScalar *x, _TyRealScalar *beta, _TyInt n) { return TImpl::cs_house(x, beta, n); }
	static inline _TyInt *maxtrans(const cs *A, _TyInt seed) { return TImpl::cs_maxtrans(A, seed); }
	static inline _TyInt *post(const _TyInt *parent, _TyInt n) { return TImpl::cs_post(parent, n); }
	static inline csd *scc(cs *A) { return TImpl::cs_scc(A); }
	static inline _TyInt scatter(const cs *A, _TyInt j, _TyScalar beta, _TyInt *w,
		_TyScalar *x, _TyInt mark, cs *C, _TyInt nz) { return TImpl::cs_scatter(A, j, beta, w,x, mark, C, nz); }
	static inline _TyInt tdfs(_TyInt j, _TyInt k, _TyInt *head, const _TyInt *next,
		_TyInt *post, _TyInt *stack) { return TImpl::cs_tdfs(j, k, head, next, post, stack); }
	static inline _TyInt leaf(_TyInt i, _TyInt j, const _TyInt *first,
		_TyInt *maxfirst, _TyInt *prevleaf, _TyInt *ancestor, _TyInt *jleaf) { return TImpl::cs_leaf(i, j, first, maxfirst, prevleaf, ancestor, jleaf); }
	static inline _TyInt reach(cs *G, const cs *B, _TyInt k, _TyInt *xi,
		const _TyInt *pinv) { return TImpl::cs_reach(G, B, k, xi, pinv); }
	static inline _TyInt spsolve(cs *L, const cs *B, _TyInt k, _TyInt *xi,
		_TyScalar *x, const _TyInt *pinv, _TyInt lo) { return TImpl::cs_spsolve(L, B, k, xi, x, pinv, lo); }
	static inline _TyInt ereach(const cs *A, _TyInt k, const _TyInt *parent,
		_TyInt *s, _TyInt *w) { return TImpl::cs_ereach(A, k, parent, s, w); }
	static inline _TyInt *randperm(_TyInt n, _TyInt seed) { return TImpl::cs_randperm(n, seed); }

	/* utilities */
	static inline csd *dalloc(_TyInt m, _TyInt n) { return TImpl::cs_dalloc(m, n); }
	static inline cs *done(cs *C, void *w, void *x, _TyInt ok) { return TImpl::cs_done(C, w, x, ok); }
	static inline _TyInt *idone(_TyInt *p, cs *C, void *w, _TyInt ok) { return TImpl::cs_idone(p, C, w, ok); }
	static inline csn *ndone(csn *N, cs *C, void *w, void *x, _TyInt ok) { return TImpl::cs_ndone(N, C, w, x, ok); }
	static inline csd *ddone(csd *D, cs *C, void *w, _TyInt ok) { return TImpl::cs_ddone(D, C, w, ok); }

protected:
	/**
	 *	@brief the implementation
	 */
	struct TImpl {
		// note that MSVC will (incorrectly) use the cast even in the ternary operator,
		// requiring no changes in CXsparse whatsoever. g++ will only use this for assignments.
#define cs_malloc dummy0(); static CAllocResult cs_malloc
#define cs_calloc dummy1(); static CAllocResult cs_calloc
#define cs_free dummy2(); static CAllocResult cs_free
#define cs_realloc dummy3(); static CAllocResult cs_realloc // make the prototypes static
#define malloc ::malloc
#define calloc ::calloc
#define free ::free
#define realloc ::realloc // make it obvious we refer to the globals
#include "../src/cxsparse/cs_malloc.c"
#undef malloc
#undef calloc
#undef free
#undef realloc // undo
#undef cs_calloc
#undef cs_free
#undef cs_realloc
#undef cs_malloc // need to undef before including the next one(s)

#define cs_scatter dummy10(); static CS_INT cs_scatter
#include "../src/cxsparse/cs_scatter.c"
#undef cs_scatter

/*#define cs_spalloc dummy20(); static cs *cs_spalloc
#define cs_sprealloc dummy21(); static CS_INT cs_sprealloc
#define cs_spfree dummy22(); static cs *cs_spfree
#define cs_nfree dummy23(); static csn *cs_nfree
#define cs_sfree dummy24(); static css *cs_sfree
#define cs_dalloc dummy25(); static csd *cs_dalloc
#define cs_dfree dummy26(); static csd *cs_dfree
#define cs_done dummy27(); static cs *cs_done
#define cs_idone dummy28(); static CS_INT *cs_idone
#define cs_ndone dummy29(); static csn *cs_ndone
#define cs_ddone dummy30(); static csd *cs_ddone
#include "../src/cxsparse/cs_util.c"
#undef cs_spalloc
#undef cs_sprealloc
#undef cs_spfree
#undef cs_nfree
#undef cs_sfree
#undef cs_dalloc
#undef cs_dfree
#undef cs_done
#undef cs_idone
#undef cs_ndone
#undef cs_ddone*/ // won't work here, the functions call each other

	protected:
#define cs_spalloc _cs_spalloc
#define cs_sprealloc _cs_sprealloc
#define cs_spfree _cs_spfree
#define cs_nfree _cs_nfree
#define cs_sfree _cs_sfree
#define cs_dalloc _cs_dalloc
#define cs_dfree _cs_dfree
#define cs_done _cs_done
#define cs_idone _cs_idone
#define cs_ndone _cs_ndone
#define cs_ddone _cs_ddone
//#include "../src/cxsparse/cs_util.c" // make the functions non-static, will implement static wrappers below
#include "cs_util_patched.inl"
#undef cs_spalloc
#undef cs_sprealloc
#undef cs_spfree
#undef cs_nfree
#undef cs_sfree
#undef cs_dalloc
#undef cs_dfree
#undef cs_done
#undef cs_idone
#undef cs_ndone
#undef cs_ddone

	public:
		static inline cs *cs_spalloc(CS_INT m, CS_INT n, CS_INT nzmax, CS_INT values, CS_INT triplet)
		{
			return TImpl()._cs_spalloc(m, n, nzmax, values, triplet);
		}

		static inline CS_INT cs_sprealloc(cs *A, CS_INT nzmax)
		{
			return TImpl()._cs_sprealloc(A, nzmax);
		}

		static inline cs *cs_spfree(cs *A)
		{
			return TImpl()._cs_spfree(A);
		}

		static inline csn *cs_nfree(csn *N)
		{
			return TImpl()._cs_nfree(N);
		}

		static inline css *cs_sfree(css *S)
		{
			return TImpl()._cs_sfree(S);
		}

		static inline csd *cs_dalloc(CS_INT m, CS_INT n)
		{
			return TImpl()._cs_dalloc(m, n);
		}

		static inline csd *cs_dfree(csd *D)
		{
			return TImpl()._cs_dfree(D);
		}

		static inline cs *cs_done(cs *C, void *w, void *x, CS_INT ok)
		{
			return TImpl()._cs_done(C, w, x, ok);
		}

		static inline CS_INT *cs_idone(CS_INT *p, cs *C, void *w, CS_INT ok)
		{
			return TImpl()._cs_idone(p, C, w, ok);
		}

		static inline csn *cs_ndone(csn *N, cs *C, void *w, void *x, CS_INT ok)
		{
			return TImpl()._cs_ndone(N, C, w, x, ok);
		}

		static inline csd *cs_ddone(csd *D, cs *C, void *w, CS_INT ok)
		{
			return TImpl()._cs_ddone(D, C, w, ok);
		}

#define cs_add dummy40(); static cs *cs_add
//#include "../src/cxsparse/cs_add.c"
#include "cs_add_patched.inl"
#undef cs_add

#define cs_amd dummy50(); static CS_INT *cs_amd
#include "../src/cxsparse/cs_amd.c"
#undef cs_amd

#define cs_chol dummy60(); static csn *cs_chol
#include "../src/cxsparse/cs_chol.c"
#undef cs_chol

#define cs_cholsol dummy70(); static CS_INT cs_cholsol
#include "../src/cxsparse/cs_cholsol.c"
#undef cs_cholsol

#define cs_compress dummy80(); static cs *cs_compress
#include "../src/cxsparse/cs_compress.c"
#undef cs_compress

#define cs_counts dummy90(); static CS_INT *cs_counts
#include "../src/cxsparse/cs_counts.c"
#undef cs_counts

#define cs_cumsum dummy100(); static double cs_cumsum
#include "../src/cxsparse/cs_cumsum.c"
#undef cs_cumsum

#define cs_dfs dummy110(); static CS_INT cs_dfs
#include "../src/cxsparse/cs_dfs.c"
#undef cs_dfs

#define cs_dmperm dummy120(); static csd *cs_dmperm
#include "../src/cxsparse/cs_dmperm.c"
#undef cs_dmperm

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_droptol dummy130(); static CS_INT cs_droptol
#include "../src/cxsparse/cs_droptol.c"
#undef cs_droptol
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#define cs_dropzeros dummy140(); static CS_INT cs_dropzeros
#include "../src/cxsparse/cs_dropzeros.c"
#undef cs_dropzeros

#define cs_dupl dummy150(); static CS_INT cs_dupl
#include "../src/cxsparse/cs_dupl.c"
#undef cs_dupl

#define cs_entry dummy160(); static CS_INT cs_entry
#include "../src/cxsparse/cs_entry.c"
#undef cs_entry

#define cs_ereach dummy170(); static CS_INT cs_ereach
#include "../src/cxsparse/cs_ereach.c"
#undef cs_ereach

#define cs_etree dummy180(); static CS_INT *cs_etree
#include "../src/cxsparse/cs_etree.c"
#undef cs_etree

#define cs_fkeep dummy190(); static CS_INT cs_fkeep
#include "../src/cxsparse/cs_fkeep.c"
#undef cs_fkeep

#define cs_gaxpy dummy200(); static CS_INT cs_gaxpy
#include "../src/cxsparse/cs_gaxpy.c"
#undef cs_gaxpy

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_happly dummy210(); static CS_INT cs_happly
#include "../src/cxsparse/cs_happly.c"
#undef cs_happly
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_house dummy220(); static CS_ENTRY cs_house
#include "../src/cxsparse/cs_house.c"
#undef cs_house
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#define cs_ipvec dummy230(); static CS_INT cs_ipvec
#include "../src/cxsparse/cs_ipvec.c"
#undef cs_ipvec

#define cs_leaf dummy240(); static CS_INT cs_leaf
#include "../src/cxsparse/cs_leaf.c"
#undef cs_leaf

/*#define cs_load dummy250(); static cs *cs_print
#include "../src/cxsparse/cs_print.c" // problems with data types
#undef cs_print*/

/*#define cs_load dummy255(); static cs *cs_load
#include "../src/cxsparse/cs_load.c" // problems with data types
#undef cs_load*/

#define cs_lsolve dummy260(); static CS_INT cs_lsolve
#include "../src/cxsparse/cs_lsolve.c"
#undef cs_lsolve

#define cs_ltsolve dummy270(); static CS_INT cs_ltsolve
#include "../src/cxsparse/cs_ltsolve.c"
#undef cs_ltsolve

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_lu dummy280(); static csn *cs_lu
#include "../src/cxsparse/cs_lu.c"
#undef cs_lu
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_lusol dummy290(); static CS_INT cs_lusol
#include "../src/cxsparse/cs_lusol.c"
#undef cs_lusol
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#define cs_maxtrans dummy300(); static CS_INT *cs_maxtrans
#include "../src/cxsparse/cs_maxtrans.c"
#undef cs_maxtrans

#define cs_multiply dummy310(); static cs *cs_multiply
//#include "../src/cxsparse/cs_multiply.c"
#include "cs_multiply_patched.inl"
#undef cs_multiply

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_norm dummy320(); static CS_ENTRY cs_norm
#include "../src/cxsparse/cs_norm.c"
#undef cs_norm
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#define cs_permute dummy330(); static cs *cs_permute
#include "../src/cxsparse/cs_permute.c"
#undef cs_permute

#define cs_pinv dummy340(); static CS_INT *cs_pinv
#include "../src/cxsparse/cs_pinv.c"
#undef cs_pinv

#define cs_post dummy350(); static CS_INT *cs_post
#include "../src/cxsparse/cs_post.c"
#undef cs_post

#define cs_pvec dummy360(); static CS_INT cs_pvec
#include "../src/cxsparse/cs_pvec.c"
#undef cs_pvec

#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define double CS_REAL_TYPE
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_qr dummy370(); static csn *cs_qr
#include "../src/cxsparse/cs_qr.c"
#undef cs_qr
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef double
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#define cs_qrsol dummy380(); static CS_INT cs_qrsol
#include "../src/cxsparse/cs_qrsol.c"
#undef cs_qrsol

#define cs_randperm dummy390(); static CS_INT *cs_randperm
#include "../src/cxsparse/cs_randperm.c"
#undef cs_randperm

#define cs_reach dummy400(); static CS_INT cs_reach
#include "../src/cxsparse/cs_reach.c"
#undef cs_reach

		// cs_scatter is already above (needed by many ops)

#define cs_scc dummy410(); static csd *cs_scc
#include "../src/cxsparse/cs_scc.c"
#undef cs_scc

#define cs_schol dummy420(); static css *cs_schol
#include "../src/cxsparse/cs_schol.c"
#undef cs_schol

#define cs_spsolve dummy430(); static CS_INT cs_spsolve
#include "../src/cxsparse/cs_spsolve.c"
#undef cs_spsolve

#define cs_sqr dummy440(); static css *cs_sqr
#include "../src/cxsparse/cs_sqr.c"
#undef cs_sqr

#define cs_symperm dummy450(); static cs *cs_symperm
#include "../src/cxsparse/cs_symperm.c"
#undef cs_symperm

#define cs_tdfs dummy460(); static CS_INT cs_tdfs
#include "../src/cxsparse/cs_tdfs.c"
#undef cs_tdfs

#define cs_transpose dummy470(); static cs *cs_transpose
#include "../src/cxsparse/cs_transpose.c"
#undef cs_transpose

#ifndef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#define cs_updown dummy480(); static CS_INT cs_updown
#include "../src/cxsparse/cs_updown.c"
#endif // !__CSPARSE_TEMPLATE_PATCH_REAL_TYPE
#undef cs_updown
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
		static CS_INT cs_updown (cs *L, CS_REAL_TYPE sigma, const cs *C, const CS_INT *parent) // this requires patch to work correctly (want it to work with CS_REAL_TYPE rather than double) // also changed the type of sigma! // todo - move this to cs_updown_patch.c to allow a diff to be made
		{
			CS_INT n, p, f, j, *Lp, *Li, *Cp, *Ci ;
			CS_ENTRY *Lx, *Cx, alpha, gamma, w1, w2, *w ;
			CS_REAL_TYPE beta = 1, beta2 = 1, delta ;
		//#ifdef CS_COMPLEX
			CS_ENTRY phase ; // swine - phase is the same as entry type
		//#endif
			if (!CS_CSC (L) || !CS_CSC (C) || !parent) return (0) ;  /* check inputs */
			Lp = L->p ; Li = L->i ; Lx = L->x ; n = L->n ;
			Cp = C->p ; Ci = C->i ; Cx = C->x ;
			if ((p = Cp [0]) >= Cp [1]) return (1) ;        /* return if C empty */
			w = cs_malloc (n, sizeof (CS_ENTRY)) ;          /* get workspace */
			if (!w) return (0) ;                            /* out of memory */
			f = Ci [p] ;
			for ( ; p < Cp [1] ; p++) f = CS_MIN (f, Ci [p]) ;  /* f = min (find (C)) */
			for (j = f ; j != -1 ; j = parent [j]) w [j] = 0 ;  /* clear workspace w */
			for (p = Cp [0] ; p < Cp [1] ; p++) w [Ci [p]] = Cx [p] ; /* w = C */
			for (j = f ; j != -1 ; j = parent [j])          /* walk path f up to root */
			{
				p = Lp [j] ;
				alpha = w [j] / Lx [p] ;                    /* alpha = w(j) / L(j,j) */
				beta2 = beta*beta + sigma*CS_REAL(alpha*CS_CONJ(alpha)) ; // swine - added the missing CS_REAL(), alpha * conj(alpha) will be real
				if (beta2 <= 0) break ;                     /* not positive definite */
				beta2 = sqrt (beta2) ;
				delta = (sigma > 0) ? (beta / beta2) : (beta2 / beta) ;
				gamma = sigma * CS_CONJ(alpha) / (beta2 * beta) ;
				Lx [p] = delta * Lx [p] + ((sigma > 0) ? (gamma * w [j]) : 0) ;
				beta = beta2 ;
		//#ifdef CS_COMPLEX
				if(b_is_complex) { // swine - compile-time constant
					phase = CS_ABS (Lx [p]) / Lx [p] ;  /* phase = abs(L(j,j))/L(j,j)*/
					Lx [p] *= phase ;                   /* L(j,j) = L(j,j) * phase */
				}
		//#endif
				for (p++ ; p < Lp [j+1] ; p++)
				{
					w1 = w [Li [p]] ;
					w [Li [p]] = w2 = w1 - alpha * Lx [p] ;
					Lx [p] = delta * Lx [p] + gamma * ((sigma > 0) ? w1 : w2) ;
		//#ifdef CS_COMPLEX
					if(b_is_complex) { // swine - compile-time constant
						Lx [p] *= phase ;               /* L(i,j) = L(i,j) * phase */
					}
		//#endif
				}
			}
			cs_free (w) ;
			return (beta2 > 0) ;
		}
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

#define cs_usolve dummy490(); static CS_INT cs_usolve
#include "../src/cxsparse/cs_usolve.c"
#undef cs_usolve

#define cs_utsolve dummy500(); static CS_INT cs_utsolve
#include "../src/cxsparse/cs_utsolve.c"
#undef cs_utsolve

		// include implementation of all the functions
	};
};

#undef INSIDE_CSPARSE_TEMPLATE_HPP
#ifdef _CXS_H_WAS_NOT_DEFINED
#undef _CXS_H // undef again, to restore the previous state
#endif // _CXS_H_WAS_NOT_DEFINED
#ifdef NCOMPLEX_WAS_NOT_DEFINED
#undef NCOMPLEX // undef again, to restore the previous state
#endif // NCOMPLEX_WAS_NOT_DEFINED
#ifdef MATLAB_MEX_WAS_DEFINED
#define MATLAB_MEX_FILE // define again, to restore the previous state
#endif // MATLAB_MEX_WAS_DEFINED
// restore "C" macros

#ifdef __CSPARSE_TEMPLATE_UNIT_TESTS

/**
 *	@brief tests the default instantiation of the class by calling every function
 *	@note This is only available if __CSPARSE_TEMPLATE_UNIT_TESTS is defined.
 */
class CTemplateSparseTest {
public:
	typedef CTSparse<> cts; /**< @brief the default instantiation type */
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
	typedef CTSparse<std::complex<double>, long, double> ctsx; /**< @brief complex instantiation type */
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE

	/**
	 *	@brief a helper class to instantiate a dummy callback function for \ref cs_fkeep()
	 *	@tparam CTS is a specialization of \ref CTSparse
	 */
	template <class CTS>
	struct Instantiate {
		/**
		 *	@brief a dummy callback function for \ref cs_fkeep()
		 *
		 *	@param[in] r is row of the element
		 *	@param[in] c is column of the element
		 *	@param[in] f is value of the element
		 *	@param[in] other is pointer to other user-specified data this callback might
		 *		need, equals the value of the third parameter of cs_fkeep()
		 *
		 *	@return Returns nonzero if the nonzero is to be kept, or zero for it to be erased.
		 */
		static typename CTS::_TyInt dummy_fkeep(typename CTS::_TyInt r,
			typename CTS::_TyInt c, typename CTS::_TyScalar f, void *other)
		{
			return 0;
		}
	};

	/**
	 *	@brief default constructor; instantiates the test function
	 */
	CTemplateSparseTest()
	{
		void (*fun)() = &Test<cts>;
		printf("%x\n", (int)(ptrdiff_t)fun);
#ifdef __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
		void (*xfun)() = &Test<ctsx>;
		printf("%x\n", (int)(ptrdiff_t)xfun);
#endif // __CSPARSE_TEMPLATE_PATCH_REAL_TYPE
	}

protected:
	/**
	 *	@brief tries calling all the templated CSparse functions
	 *	@tparam CTS is a specialization of \ref CTSparse
	 *	@note This will crash if called, no memory is allocated.
	 */
	template <class CTS>
	static void Test()
	{
		typename CTS::cs A;
		typename CTS::css S;
		typename CTS::csn N;
		typename CTS::csd D;
		typename CTS::_TyScalar b, x, y;
		typename CTS::_TyRealScalar beta;
		typename CTS::_TyInt ok, p, c, xi, pinv, q, parent, post, head, next,
			stack, first, maxfirst, prevleaf, ancestor, jleaf, w;

		CTS::add(&A, &A, 1.0, 1.0);
		CTS::cholsol(0, &A, &b);
		CTS::dupl(&A);
		CTS::entry(&A, 0, 0, 3.14);
		CTS::lusol(0, &A, &b, 1e-3);
		CTS::gaxpy(&A, &x, &y);
		CTS::multiply(&A, &A);
		CTS::qrsol(0, &A, &b);
		CTS::transpose(&A, 1);
		CTS::compress(&A);
		CTS::norm(&A);

		/* utilities */
		CTS::calloc(1, 8);
		CTS::free(0);
		CTS::realloc(0, 1, 10, &ok);
		CTS::spalloc(100, 100, 100, 1, 0);
		CTS::spfree(&A);
		CTS::sprealloc(&A, 1000);
		CTS::malloc(1, 8);

		/* --- secondary CSparse routines and data structures ----------------------- */

		CTS::amd(0, &A);
		CTS::chol(&A, &S);
		CTS::dmperm(&A, 12345);
		CTS::droptol(&A, 1e-3);
		CTS::dropzeros(&A);
		CTS::happly(&A, 0, 1.0, &x);
		CTS::ipvec(&p, &b, &x, 100);
		CTS::lsolve(&A, &x);
		CTS::ltsolve(&A, &x);
		CTS::lu(&A, &S, 1e-3);
		CTS::permute(&A, &pinv, &q, 1);
		CTS::pinv(&p, 100);
		CTS::pvec(&p, &b, &x, 100);
		CTS::qr(&A, &S);
		CTS::schol(0, &A);
		CTS::sqr(0, &A, 1);
		CTS::symperm(&A, &pinv, 1);
		CTS::usolve(&A, &x);
		CTS::utsolve(&A, &x);
		CTS::updown(&A, +1, &A, &parent);

		/* utilities */
		CTS::sfree(&S);
		CTS::nfree(&N);
		CTS::dfree(&D);

		/* --- tertiary CSparse routines -------------------------------------------- */

		CTS::counts(&A, &parent, &post, 1);
		CTS::cumsum(&p, &c, 100);
		CTS::dfs(0, &A, 0, &xi, &p, &pinv);
		CTS::etree(&A, 1);
		CTS::fkeep(&A, &Instantiate<CTS>::dummy_fkeep, 0);
		CTS::house(&x, &beta, 100);
		CTS::maxtrans(&A, 123456);
		CTS::post(&parent, 100);
		CTS::scc(&A);
		CTS::scatter(&A, 0, beta, &w, &x, 0, &A, 1000);
		CTS::tdfs(0, 0, &head, &next, &post, &stack);
		CTS::leaf(0, 0, &first, &maxfirst, &prevleaf, &ancestor, &jleaf);
		CTS::reach(&A, &A, 0, &xi, &pinv);
		CTS::spsolve(&A, &A, 0, &xi, &x, &pinv, 0);
		CTS::ereach(&A, 0, &parent, &xi, &w);
		CTS::randperm(100, 123456);

		/* utilities */
		CTS::dalloc(100, 100);
		CTS::done(&A, &w, &x, 1);
		CTS::idone(&p, &A, &w, 1);
		CTS::ndone(&N, &A, &w, &x, 1);
		CTS::ddone(&D, &A, &w, 1);
	}
};

#endif // __CSPARSE_TEMPLATE_UNIT_TESTS

#endif // !__CSPARSE_TEMPLATE_INCLUDED
