/*
								+----------------------------------+
								|                                  |
								| ***  Eigenvalue calculation  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|         Eigenvalues.inl          |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file include/slam/Eigenvalues.inl
 *	@brief eigenvalue calculation inlines
 *	@author -tHE SWINe-
 *	@date 2015-11-16
 */

#pragma once
#ifndef __EIGENVALUES_INLINES_INCLUDED
#define __EIGENVALUES_INLINES_INCLUDED

/*
 *	The below code was ported from the ARPACK library (from Fortran).
 *
 *	BSD Software License
 *
 *	Pertains to ARPACK and P_ARPACK
 *
 *	Copyright (c) 1996-2008 Rice University.
 *	Developed by D.C. Sorensen, R.B. Lehoucq, C. Yang, and K. Maschhoff.
 *	All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are
 *	met:
 *
 *	- Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	- Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer listed
 *	  in this license in the documentation and/or other materials
 *	  provided with the distribution.
 *
 *	- Neither the name of the copyright holders nor the names of its
 *	  contributors may be used to endorse or promote products derived from
 *	  this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *	Some of the below code is modified from the rARPACK R package:
 *
 *	Package: rARPACK
 *	Type: Package
 *	Title: R wrapper of ARPACK for large scale eigenvalue/vector problems,
 *		on both dense and sparse matrices
 *	Version: 0.8-0
 *	Date: 2015-08-12
 *	Author: Yixuan Qiu, Jiali Mei and authors of the ARPACK library. See file
 *		AUTHORS for details.
 *	Maintainer: Yixuan Qiu <yixuan.qiu@cos.name>
 *	Description: An R wrapper of the ARPACK library
 *	    (http://www.caam.rice.edu/software/ARPACK/) to solve large scale
 *	    eigenvalue/vector problems. It is typically used to compute a few
 *	    eigenvalues/vectors of an n by n matrix, e.g., the k largest eigenvalues,
 *	    which is usually more efficient than eigen() if k << n. This package
 *	    provides the eigs() function which does the similar job as in Matlab,
 *	    Octave, Python SciPy and Julia. It also provides the svds() function
 *	    to calculate the largest k singular values and corresponding
 *	    singular vectors of a real matrix. Matrices can be given in either dense
 *	    or sparse form.
 *	License: BSD_3_clause + file LICENSE
 *	Copyright: see file COPYRIGHTS
 *	URL: https://github.com/yixuan/rARPACK
 *	BugReports: https://github.com/yixuan/rARPACK/issues
 *	Depends:
 *	    R (>= 3.0.2)
 *	Imports:
 *	    Matrix (>= 1.1-0),
 *	    Rcpp (>= 0.11.5)
 *	LinkingTo: Rcpp, RcppEigen (>= 0.3.2.2.0)
 *
 */

namespace eigenvalues_detail {

template <class T, class CSortModifier, bool b_ascending_sort>
class CSortPred {
protected:
	const T *m_p_array;
	const size_t *m_p_indices_begin;

public:
	CSortPred(const T *p_array, const size_t *p_indices_begin)
		:m_p_array(p_array), m_p_indices_begin(p_indices_begin)
	{}

	bool operator ()(size_t n_index_a, size_t n_index_b)
	{
		if(!b_ascending_sort)
			std::swap(n_index_a, n_index_b);
		// if the sort is descending, swap the indices

		return CSortModifier()(m_p_array[n_index_a]) < CSortModifier()(m_p_array[n_index_b]);
		// comapre what the sort modifier returns
	}
};

template <int n_sort_type, bool b_ascending_sort>
template <class T>
void CSortEigenvalues<n_sort_type, b_ascending_sort>::Sort(std::vector<size_t>
	&r_sorted_sequence, const T *p_eigenvalues, size_t n_eigenvalue_num) // throw(std::bad_alloc)
{
	r_sorted_sequence.resize(n_eigenvalue_num);
	for(size_t i = 0; i < n_eigenvalue_num; ++ i)
		r_sorted_sequence[i] = i;
	// build a sequence of integers

	if(n_eigenvalue_num) {
		std::sort(r_sorted_sequence.begin(), r_sorted_sequence.end(),
			CSortPred<T, _TySortModifier, b_ascending_sort>(p_eigenvalues, &r_sorted_sequence.front()));
	}
	// sort the sequence
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Compute(const Eigen::Ref<const _TyMatrix> &r_A) // throw(std::bad_alloc)
{
	const size_t n = r_A.rows();
	_ASSERTE(n >= 2); // won't work for 1x1 matrices

	enum {
		b_stream = true // streaming / in-place implementation (streaming is probably better for large matrices)
	};

	if(b_stream) { // compile-time const
		m_t_R.resize(n, n);
		//m_t_R.setZero(); // only really need to zero the strictly lower triangle
		m_t_R.template triangularView<Eigen::StrictlyLower>().setZero(); // not sure if this is any faster
		m_t_R.template topRows<1>() = r_A.template topRows<1>(); // or can stream contents in
	} else
		m_t_R = r_A; // can work inplace
	m_t_rot.resize(2, n - 1);

	const _TyScalar f_epsilon = std::numeric_limits<_TyScalar>::epsilon();
	for(size_t i = 1; i < n; ++ i) {
		if(b_stream) // compile-time const
			m_t_R.row(i).tail(n - i + 1) = r_A.row(i).tail(n - i + 1); // stream in more rows

		Eigen::Block<_TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i - 1);
		Eigen::Block<_TyMatrix, 2, 1> v_Tii_Ti1i = m_t_R.template block<2, 1>(i - 1, i - 1);

		_TyScalar r = v_Tii_Ti1i.norm();
		_TyMatrix2 G;
		if(r <= f_epsilon) {
			v_sin_cos = _TyVector2(0, 1);
			G.setIdentity();
		} else {
			v_Tii_Ti1i /= r; // vectorize
			v_sin_cos(0) = v_Tii_Ti1i(1);
			v_sin_cos(1) = v_Tii_Ti1i(0);
			G.col(0) = v_Tii_Ti1i; G(1, 0) = -G(1, 0);
			G.col(1) = v_sin_cos;
		}

		if(b_stream) // compile-time const
			v_Tii_Ti1i = _TyVector2(r, 0);
		else {
			v_Tii_Ti1i(0) = r;
			_ASSERTE(&v_Tii_Ti1i(1) == &m_t_R.col(i - 1).tail(n - i)(0));
			m_t_R.col(i - 1).tail(n - i).setZero(); // could be faster to zero the lower triangle at the beginning / end
		}
		// eliminate the below-diagonal entries

		m_t_R.template middleRows<2>(i - 1).rightCols(n - i) =
			G * m_t_R.template middleRows<2>(i - 1).rightCols(n - i);
		// update rows to the right of the diagonal
	}

	_ASSERTE(Eigen::MatrixXd(m_t_R.template triangularView<Eigen::StrictlyLower>()).norm() == 0);
	// make sure we did not forget anything in the lower triangle

	m_n = n; // computed
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Get_RQ(Eigen::Ref<_TyMatrix> RQ) const // throw(std::bad_alloc)
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	_ASSERTE(RQ.rows() == m_n && RQ.cols() == m_n); // Ref<> can't be resized
	//RQ = m_t_R.template triangularView<Eigen::Upper>(); // is this any faster than copying the whole thing?
	RQ = m_t_R;
	// make a copy of the R matrix

	const size_t n = m_n; // antialiass
	for(size_t i = 1; i < n; ++ i) {
		Eigen::Block<const _TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i - 1);
		_TyMatrix2 G;
		G.col(0) = _TyVector2(v_sin_cos(1), -v_sin_cos(0));
		G.col(1) = v_sin_cos;
		RQ.template middleCols<2>(i - 1).topRows(i + 1) *= G.transpose();
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Get_Q(Eigen::Ref<_TyMatrix> Q) const // throw(std::bad_alloc)
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first
	_ASSERTE(Q.rows() == m_n && Q.cols() == m_n); // Ref<> can't be resized

	Q.setZero(); // arguably faster than setIdentity()
	Q(0, 0) = 1; // only need to initialize the corner
	for(size_t i = 0, n1 = m_n - 1; i < n1; ++ i) { // n != 0 asserted above
		Eigen::Block<const _TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i);
		_TyMatrix2 G;
		G.col(0) = _TyVector2(v_sin_cos(1), -v_sin_cos(0));
		G.col(1) = v_sin_cos;
		Q(i + 1, i + 1) = 1; // stream in parts of the diagonal as the computation progresses
		Q.template middleCols<2>(i).topRows(i + 2) *= G.transpose();
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Apply_YQ(Eigen::Ref<_TyMatrix> Y) const
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const size_t nrow = Y.rows();
	for(size_t i = 0, n1 = m_n - 1; i < n1; ++ i) {
		Eigen::Block<const _TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i);
		_TyMatrix2 G;
		G.col(0) = _TyVector2(v_sin_cos(1), -v_sin_cos(0));
		G.col(1) = v_sin_cos;
		Y.template middleCols<2>(i) *= G.transpose();
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Apply_YQt(Eigen::Ref<_TyMatrix> Y) const // throw(std::bad_alloc) // unused/untested
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const size_t n = m_n; // antialiass
	const _TyScalar *c = &m_t_rot(1, 0 + n - 2),
		*s = &m_t_rot(0, 0 + n - 2); // todo - avoid using pointers
	_TyVector Yi(Y.rows()); // todo - remove
	for(size_t i = n - 1; i > 0; s -= 2, c -= 2) {
		-- i; // here
		Yi = Y.col(i);
		Y.col(i)     = (*c) * Yi - (*s) * Y.col(i + 1);
		Y.col(i + 1) = (*s) * Yi + (*c) * Y.col(i + 1); // todo - use 2x2 matrix multiplication
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Apply_QY(_TyVector &Y) const // unused/untested
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const size_t n = m_n; // antialiass
	for(size_t i = n - 1; i > 0;) {
		-- i; // here
		_TyScalar tmp      = Y[i];
		Y[i]     = m_t_rot(1, i) * tmp - m_t_rot(0, i) * Y[i + 1];
		Y[i + 1] = m_t_rot(0, i) * tmp + m_t_rot(1, i) * Y[i + 1]; // todo - use 2x2 matrix multiplication
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Apply_QY(Eigen::Ref<_TyMatrix> Y) const // throw(std::bad_alloc) // unused/untested
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const size_t n = m_n; // antialiass
	const _TyScalar *c = *c = &m_t_rot(1, 0 + n - 2),
		*s = &m_t_rot(0, 0 + n - 2); // todo - avoid using pointers
	_TyVector Yi(Y.cols()), Yi1(Y.cols());
	for(size_t i = n - 1; i > 0; c -= 2, s -= 2) {
		-- i; // here
		Yi  = Y.row(i);
		Yi1 = Y.row(i + 1);
		Y.row(i)     = (*c) * Yi - (*s) * Yi1;
		Y.row(i + 1) = (*s) * Yi + (*c) * Yi1; // todo - use 2x2 matrix multiplication
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Apply_QtY(_TyVector &Y) const // unused/untested
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const size_t n = m_n; // antialiass
	for(size_t i = 1; i < n; ++ i) {
		_TyScalar tmp = Y[i - 1];
		Y[i - 1] = m_t_rot(1, i - 1) * tmp + m_t_rot(0, i - 1) * Y[i];
		Y[i] = -m_t_rot(0, i - 1) * tmp + m_t_rot(1, i - 1) * Y[i]; // todo - use 2x2 matrix multiplication
	}
}

template <class CScalar>
void CUpperHessenbergQR<CScalar>::Apply_QtY(Eigen::Ref<_TyMatrix> Y) const // throw(std::bad_alloc) // unused/untested
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const _TyScalar *c = &m_t_rot(1, 0),
		*s = &m_t_rot(0, 0); // todo - avoid using pointers
	_TyVector Yi(Y.cols()), Yi1(Y.cols());
	const size_t n = m_n; // antialiass
	for(size_t i = 1; i < n; ++ i, c += 2, s += 2) {
		Yi = Y.row((i - 1));
		Yi1 = Y.row((i - 1) + 1);
		Y.row((i - 1))     = (*c) * Yi + (*s) * Yi1;
		Y.row((i - 1) + 1) = -(*s) * Yi + (*c) * Yi1; // todo - use 2x2 matrix multiplication
	}
}

template <class CScalar>
void CTridiagQR<CScalar>::Compute(const Eigen::Ref<const _TyMatrix> &r_A) // throw(std::bad_alloc)
{
	_ASSERTE(r_A.rows() == r_A.cols() && r_A.cols() >= 2); // won't work for 1x1 matrices

	const size_t n = r_A.rows();
	m_t_R.resize(n, n);
	m_t_rot.resize(2, n - 1);
	// allocate everything

	m_t_R.setZero();
	m_t_R.template topLeftCorner<2, 2>() = r_A.template topLeftCorner<2, 2>();
	/*m_t_R.diagonal() = r_A.diagonal();
	m_t_R.diagonal(1) = r_A.diagonal(-1);
	m_t_R.diagonal(-1) = r_A.diagonal(-1);*/ // do not like copying the diagonals separately like this, not good cache coherency
	// initialize R; the algorithm works inplace (or partially inplace)

	const _TyScalar f_epsilon = std::numeric_limits<_TyScalar>::epsilon();
	for(size_t i = 0, n2 = n - 2; i <= n2; ++ i) {
		Eigen::Block<_TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i);
		Eigen::Block<_TyMatrix, 2, 1> v_Tii_Ti1i = m_t_R.template block<2, 1>(i, i);

		_TyScalar r = v_Tii_Ti1i.norm();
		_TyMatrix2 G;
		if(r <= f_epsilon) {
			v_sin_cos = _TyVector2(0, 1);
			G.setIdentity();
		} else {
			v_Tii_Ti1i /= r; // vectorize
			v_sin_cos(0) = v_Tii_Ti1i(1);
			v_sin_cos(1) = v_Tii_Ti1i(0);
			G.col(0) = v_Tii_Ti1i; G(1, 0) = -G(1, 0);
			G.col(1) = v_sin_cos;
		}

		v_Tii_Ti1i = _TyVector2(r, 0); // eliminate (not needed if R not needed)

		Eigen::Block<_TyMatrix, 2, 1> v_Tii1_Ti1i1 = m_t_R.template block<2, 1>(i, i + 1);
		v_Tii1_Ti1i1 = G * v_Tii1_Ti1i1; // todo - figure out if using G or if using the expanded product is faster
			//_TyVector2(v_sin_cos(1), -v_sin_cos(0)) * v_Tii1_Ti1i1(0) + v_sin_cos * v_Tii1_Ti1i1(1);
		// only one of the two elements needed (the one on the row i + 1) if R not needed

		if(i < n2) {
			m_t_R.template block<2, 1>(i, i + 2) = v_sin_cos * r_A(i + 2, i + 1);//r_A(i + 1, i + 2); // take care to only access lower off-diagonal in A
			// only one of the two elements needed (the one on the row i + 1) if R not needed

			m_t_R.template block<1, 2>(i + 2, i + 1) = r_A.template block<1, 2>(i + 2, i + 1);
			// stream in more data inside the loop (or copy the diagonals above the loop)
		}
	}

	m_n = n;
	// now it is computed
}

template <class CScalar>
void CTridiagQR<CScalar>::Get_RQ(Eigen::Ref<_TyMatrix> RQ) const // throw(std::bad_alloc)
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	_ASSERTE(RQ.rows() == m_n && RQ.cols() == m_n); // Ref<> can't be resized
	RQ.setZero();
	//RQ.diagonal() = m_t_R.diagonal();
	//RQ.diagonal(1) = m_t_R.diagonal(1); // done in the loop already, with better locality of reference
	RQ.template topLeftCorner<2, 1>() = m_t_R.template topLeftCorner<2, 1>(); // only need to initialize the corner
	// get output matrix, partially copy R

	/*RQ = m_t_R;
	Apply_YQ(RQ);
	return RQ;*/
	// this would also work but is a waste of computation

	for(size_t i = 0, n1 = m_n - 1; i < n1; ++ i) { // n != 0 asserted above
		Eigen::Block<const _TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i);
		Eigen::Block<Eigen::Ref<_TyMatrix>, 2, 2> M = RQ.template block<2, 2>(i, i);

		M.col(1) = m_t_R.template block<2, 1>(i, i + 1);
		// stream in parts of the diagonal as the computation progresses

		//_TyScalar f_temp = M(1, 0);
		//M(0, 0) = v_sin_cos(1) * M(0, 0) + v_sin_cos(0) * M(0, 1);
		//M(1, 0) = v_sin_cos(1) * f_temp/*M(1, 0)*/ + v_sin_cos(0) * M(1, 1);
		//M(1, 1) = -v_sin_cos(0) * f_temp/*M(1, 0)*/ + v_sin_cos(1) * M(1, 1);
		//M(0, 1) = M(1, 0); // copy the below-subdiagonal to above-subdiagonal
        // update diagonal and the below-subdiagonal

		//_TyScalar f_temp = _TyVector2(-v_sin_cos(0), v_sin_cos(1)).dot(M.template bottomRows<1>());
		//M.template leftCols<1>() = (_TyVector2(v_sin_cos(1), v_sin_cos(0)) * M.transpose()).transpose(); // not sure if this helps anything
		//M(1, 1) = f_temp;
		//M(0, 1) = M(1, 0); // copy the below-subdiagonal to above-subdiagonal
		// also works

		_TyMatrix2 G;
		G.col(0) = _TyVector2(v_sin_cos(1), -v_sin_cos(0));
		G.col(1) = v_sin_cos;
		M *= G.transpose();
		M(0, 1) = M(1, 0); // this is required, probably has to do with missing a product with one off-diagonal element (would have to make M 3x2 since R is tridiagonal)
		// best choice?
	}

	//RQ.diagonal(1) = RQ.diagonal(-1); // done in the loop already, with better locality of reference
	// copy the below-subdiagonal to above-subdiagonal
}

template <class CScalar>
void CTridiagQR<CScalar>::Get_Q(Eigen::Ref<_TyMatrix> Q) const // throw(std::bad_alloc)
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first
	_ASSERTE(Q.rows() == m_n && Q.cols() == m_n); // Ref<> can't be resized

	Q.setZero(); // arguably faster than setIdentity()
	Q(0, 0) = 1; // only need to initialize the corner
	for(size_t i = 0, n1 = m_n - 1; i < n1; ++ i) { // n != 0 asserted above
		Eigen::Block<const _TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i);
		_TyMatrix2 G;
		G.col(0) = _TyVector2(v_sin_cos(1), -v_sin_cos(0));
		G.col(1) = v_sin_cos;
		Q(i + 1, i + 1) = 1; // stream in parts of the diagonal as the computation progresses
		Q.template middleCols<2>(i).topRows(i + 2) *= G.transpose();
	}
}

template <class CScalar>
void CTridiagQR<CScalar>::Apply_YQ(Eigen::Ref<_TyMatrix> Y) const
{
	_ASSERTE(m_n); // if this triggers, need to call Compute() first

	const size_t nrow = Y.rows();
	for(size_t i = 0, n1 = m_n - 1; i < n1; ++ i) {
		Eigen::Block<const _TyMatrix2X, 2, 1, true> v_sin_cos = m_t_rot.col(i);
		_TyMatrix2 G;
		G.col(0) = _TyVector2(v_sin_cos(1), -v_sin_cos(0));
		G.col(1) = v_sin_cos;
		Y.template middleCols<2>(i) *= G.transpose();
	}
}

template <class CScalar>
void CTridiagEigsSolver<CScalar>::Compute(const Eigen::Ref<const _TyMatrix> &r_A) // throw(std::bad_alloc, std::runtime_error)
{
	_ASSERTE(r_A.rows() >= 2); // won't work for 1x1 matrices
	_ASSERTE(r_A.rows() == r_A.cols()); // the matrix must be square

	const size_t n = r_A.cols();
	m_v_eval = r_A.diagonal();
	_TyVector sub_diag = r_A.diagonal(-1);
	m_v_evec.resize(n, n);
	m_v_evec.setIdentity();

	for(size_t n_start = 0, n_end = n - 1, n_iter = 0;; ++ n_iter) {
		for(size_t i = n_start; i < n_end; ++ i) {
			if(Eigen::internal::isMuchSmallerThan(fabs(sub_diag(i)),
			   fabs(m_v_eval(i)) + fabs(m_v_eval(i + 1))))
				sub_diag(i) = 0;
		}

		while(n_end > 0 && sub_diag(n_end - 1) == 0)
			-- n_end;
		if(!n_end)
			break;
		// find the largest unreduced block

		if(n_iter > 30 * n)
			throw std::runtime_error("CTridiagEigsSolver failed to compute all the eigenvalues");
		// if we spent too many iterations, we give up

		n_start = n_end - 1;
		while(n_start > 0 && sub_diag(n_start - 1) != 0)
			-- n_start;

#if EIGEN_WORLD_VERSION == 3 && EIGEN_MAJOR_VERSION == 2 && EIGEN_MINOR_VERSION > 4
		Eigen::internal::tridiagonal_qr_step/*<Eigen::ColMajor>*/(m_v_eval.data(),
			sub_diag.data(), n_start, n_end, m_v_evec.data(), n);
#else // EIGEN_WORLD_VERSION == 3 && EIGEN_MAJOR_VERSION == 2 && EIGEN_MINOR_VERSION > 4
		Eigen::internal::tridiagonal_qr_step<Eigen::ColMajor>(m_v_eval.data(),
			sub_diag.data(), n_start, n_end, m_v_evec.data(), n);
#endif // EIGEN_WORLD_VERSION == 3 && EIGEN_MAJOR_VERSION == 2 && EIGEN_MINOR_VERSION > 4
	}

	m_n = n; // now computed
}

} // ~eigenvalues_detail

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::CSymEigsSolver(const COpType &r_op,
	size_t n_eigs_num, size_t n_ritz_num)
	:m_r_op(r_op), m_n(r_op.n_Column_Num()), m_n_eigs_num(n_eigs_num),
	m_n_ritz_num((n_ritz_num > m_n)? m_n : n_ritz_num),
	m_n_mat_op_num(0), m_n_restart_num(0),
	m_f_prec(pow(std::numeric_limits<double>::epsilon(), 2.0 / 3))
{
	_ASSERTE(m_n_eigs_num >= 1 && m_n_eigs_num < m_n); // m_n_eigs_num must satisfy 1 <= m_n_eigs_num <= n - 1, n is the size of matrix
	_ASSERTE(m_n_ritz_num > m_n_eigs_num && m_n_ritz_num <= m_n); // m_n_ritz_num must satisfy m_n_eigs_num < m_n_ritz_num <= n, n is the size of matrix
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
template <class Derived0>
void CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Init(const Eigen::MatrixBase<Derived0> &r_v_init_resid) // throw(std::bad_alloc)
{
	_ASSERTE(r_v_init_resid.rows() == m_n && r_v_init_resid.cols() == 1);

	m_t_fac_V.resize(m_n, m_n_ritz_num);
	m_t_fac_H.resize(m_n_ritz_num, m_n_ritz_num);
	m_v_fac_f.resize(m_n);
	m_v_ritz_val.resize(m_n_ritz_num);
	m_t_ritz_vec.resize(m_n_ritz_num, m_n_eigs_num);
	m_ritz_conv.resize(m_n_eigs_num);
	// reset all matrices/vectors to zero

	m_t_fac_V.setZero();
	m_t_fac_H.setZero();
	m_v_fac_f.setZero();
	m_v_ritz_val.setZero();
	m_t_ritz_vec.setZero();
	std::fill(m_ritz_conv.begin(), m_ritz_conv.end(), false);

	m_n_mat_op_num = 0;
	m_n_restart_num = 0;

	double vnorm = r_v_init_resid.norm();
	_ASSERTE(vnorm > std::numeric_limits<double>::epsilon());
	Eigen::VectorXd v = r_v_init_resid;
	v /= vnorm; // putting this in the same line with the initialization changes results slightly; do it later

	Eigen::VectorXd w(m_n);
	m_r_op(v, w);
	++ m_n_mat_op_num;

	m_t_fac_H(0, 0) = v.dot(w);
	m_v_fac_f = w - v * m_t_fac_H(0, 0);
	m_t_fac_V.col(0) = v;
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
void CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Init() // throw(std::bad_alloc)
{
	Eigen::VectorXd init_resid;
	do {
		init_resid = Eigen::VectorXd::Random(m_n).array() - .5;
	} while(init_resid.norm() <= std::numeric_limits<double>::epsilon());
	Init(init_resid);
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
size_t CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Compute(size_t
	n_max_iteration_num /*= 1000*/, double f_tolerance /*= 1e-10*/) // throw(std::bad_alloc, std::runtime_error)
{
	if(!m_t_fac_V.cols())
		Init();
	// in case we have not initialized yet, do it now

	Factorize(1, m_n_ritz_num);
	Get_RitzPairs();
	// the m-step Arnoldi factorization

	size_t n_converged_num = 0;
	for(size_t i = 0; i < n_max_iteration_num; ++ i) {
		n_converged_num = n_Find_Converged(f_tolerance);
		if(n_converged_num >= m_n_eigs_num) {
			_ASSERTE(n_converged_num == m_n_eigs_num);
			// the number of converged Ritz values can't currently be larger than the number of eigenvalues

			break;
		}

		try {
			Restart(n_Adjusted_Eigenvalue_Num(n_converged_num));
		} catch(std::runtime_error&) {
			break; // CTridiagEigsSolver in Get_RitzPairs() failed to converge; will have to do with the eigs we have discovered so far
		}
		++ m_n_restart_num;
	}
	// restarted Arnoldi iteration

	Sort_RitzPairs();
	// sort the results

	return n_converged_num;
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
Eigen::VectorXd CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::v_Eigenvalues() const // throw(std::bad_alloc)
{
	size_t n_converged_num = n_Converged_Num();
	Eigen::VectorXd v_res(n_converged_num);

	if(!n_converged_num)
		return v_res;

	for(size_t i = 0, j = 0; i < m_n_eigs_num; ++ i) {
		if(m_ritz_conv[i]) {
			v_res(j) = m_v_ritz_val(i);
			++ j;
		}
	}

	return v_res;
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
Eigen::MatrixXd CSymEigsSolver<COpType, _n_sort_type,
	_b_ascending_sort>::t_Eigenvectors(size_t n_vector_num /*= SIZE_MAX*/) const // throw(std::bad_alloc)
{
	size_t n_converged_num = n_Converged_Num();
	n_vector_num = std::min(n_vector_num, n_converged_num);

	if(!n_vector_num)
		return Eigen::MatrixXd(m_n, n_vector_num);

	Eigen::MatrixXd t_converged_ritz_vec(m_n_ritz_num, n_vector_num);
	for(size_t i = 0, j = 0; i < m_n_eigs_num && j < n_vector_num; ++ i) {
		if(m_ritz_conv[i]) {
			t_converged_ritz_vec.col(j) = m_t_ritz_vec.col(i);
			++ j;
		}
	}

	return m_t_fac_V * t_converged_ritz_vec;
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
void CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Factorize(size_t n_start, size_t n_end) // throw(std::bad_alloc)
{
	if(n_end <= n_start)
		return;

	m_t_fac_H.rightCols(m_n_ritz_num - n_start).setZero();
	m_t_fac_H.block(n_start, 0, m_n_ritz_num - n_start, n_start).setZero();
	// keep the upper-left k x k submatrix of H and set other elements to 0

	//Eigen::VectorXd w(m_n); // not needed
	Eigen::VectorXd Vf(n_end); // avoid dynamic allocation inside the loop
	for(size_t i = n_start; i < n_end; ++ i) {
		double f_beta = m_v_fac_f.norm();
		m_t_fac_V.col(i).noalias() = m_v_fac_f / f_beta; // The (i+1)-th column

		m_r_op(m_t_fac_V.col(i), m_v_fac_f/*w*/);
		++ m_n_mat_op_num;

		m_t_fac_H(i, i - 1) = f_beta;
		m_t_fac_H(i - 1, i) = f_beta;

		double f_Hii = m_t_fac_V.col(i).dot(m_v_fac_f/*w*/);
		m_t_fac_H(i, i) = f_Hii;

		//m_v_fac_f.noalias() = w - f_beta * m_t_fac_V.col(i - 1) - f_Hii * m_t_fac_V.col(i);
		// CRC32 of the results: 0x6643fadd exact / 0x5d517729 rounded

		m_v_fac_f.noalias() -= m_t_fac_V.col(i - 1) * f_beta;
		m_v_fac_f.noalias() -= m_t_fac_V.col(i) * f_Hii;
		// CRC32 of the results: 0x6643fadd exact / 0x5d517729 rounded

		//m_v_fac_f.noalias() = w - (f_beta * m_t_fac_V.col(i - 1) + f_Hii * m_t_fac_V.col(i));
		// CRC32 of the results: 0xc1d0b190 exact / 0xa986ef51 rounded, fails on Oberwolfach_LFAT5

		//m_v_fac_f.noalias() -= /*w -*/ m_t_fac_V.template middleCols<2>(i - 1) * Eigen::Vector2d(f_beta, f_Hii);
		// CRC32 of the results: 0xc1d0b190 exact / 0xa986ef51 rounded, fails on Oberwolfach_LFAT5

		double v1f = m_v_fac_f.dot(m_t_fac_V.col(0));
		if(v1f > m_f_prec || v1f < -m_f_prec) {
			//Eigen::VectorXd &Vf = w; // reuse w to avoid dynamic allocation inside a loop
			//_ASSERTE(i + 1 <= m_n);
			Vf(0) = v1f; // already calculated this above
			Vf.segment(1, i) = m_t_fac_V.middleCols(1, i).transpose() * m_v_fac_f;
			m_v_fac_f -= m_t_fac_V.leftCols(i + 1) * Vf.head(i + 1);
			//m_v_fac_f -= m_t_fac_V.leftCols(i + 1) * (m_t_fac_V.leftCols(i + 1).transpose() * m_v_fac_f); // needs a temporary anyway, nothing saved
		}
		// make sure f is orthogonal to V; typically the largest absolute value occurs in
		// the first element, i.e., dot(v1, f), so we use this to test the orthogonality
	}
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
void CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Restart(size_t n_start) // throw(std::bad_alloc, std::runtime_error)
{
	if(n_start >= m_n_ritz_num)
		return;

	eigenvalues_detail::CTridiagQR<double> decomp; // reuse storage in the iterations, probably a good thing
	//eigenvalues_detail::CUpperHessenbergQR<double> decomp; // slower and not more precise, just for testing
	Eigen::MatrixXd Q(m_n_ritz_num, m_n_ritz_num);// = Eigen::MatrixXd::Identity(m_n_ritz_num, m_n_ritz_num);
	for(size_t i = n_start; i < m_n_ritz_num; ++ i) {
		m_t_fac_H.diagonal().array() -= m_v_ritz_val(i);
		decomp.Compute(m_t_fac_H); // QR decomposition of H - mu * I (where mu is the shift)
		if(i > n_start)
			decomp.Apply_YQ(Q); // Q = Q * Q_i
		else
			decomp.Get_Q(Q); // Q = Q_1, about half the FLOPS compared to Apply_YQ()
		decomp.Get_RQ(m_t_fac_H); // H = Q'HQ
		m_t_fac_H.diagonal().array() += m_v_ritz_val(i);
		// since QR = H - mu * I, we have H = QR + mu * I and therefore Q'HQ = RQ + mu * I
	}
	// V = VQ, only need to update the first n_start + 1 columns
	// Q has some elements being zero, only the first (m_n_ritz_num - n_start + i)
	// elements of the i-th column of Q are non-zero

	Eigen::MatrixXd Vs(m_n, n_start + 1);
	for(size_t i = 0; i < n_start; ++ i) {
		_ASSERTE(n_start < m_n_ritz_num); // checked above, make sure the line below does not underflow
		size_t nnz = m_n_ritz_num - n_start + i + 1;
		Vs.col(i).noalias() = m_t_fac_V.leftCols(nnz) * Q.col(i).head(nnz);
	}
	Vs.col(n_start).noalias() = m_t_fac_V * Q.col(n_start);
	m_t_fac_V.leftCols(n_start + 1).noalias() = Vs;

	m_v_fac_f = m_v_fac_f * Q(m_n_ritz_num - 1, n_start - 1) +
		m_t_fac_V.col(n_start) * m_t_fac_H(n_start, n_start - 1);
	Factorize(n_start, m_n_ritz_num);

	Get_RitzPairs();
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
size_t CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::n_Find_Converged(double f_tolerance)
{
	_ASSERTE(m_ritz_conv.size() == m_n_eigs_num); // already allocated
	size_t n_converged_num = 0;
	double f_fac_f_norm = m_v_fac_f.norm();
	for(size_t i = 0; i < m_n_eigs_num; ++ i) {
		double f_thresh = f_tolerance * std::max(fabs(m_v_ritz_val(i)), m_f_prec);
		double f_residual = fabs(m_t_ritz_vec.template bottomRows<1>()(i)) * f_fac_f_norm;
		if((m_ritz_conv[i] = (f_residual < f_thresh)))
			++ n_converged_num;
	}
	// converged "wanted" Ritz values

	return n_converged_num;
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
size_t CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::n_Adjusted_Eigenvalue_Num(size_t n_converged_num) const
{
	size_t n_nev_new;
	if(m_n_eigs_num == 1 && m_n_ritz_num >= 6)
		n_nev_new = m_n_ritz_num / 2;
	else if(m_n_eigs_num == 1 && m_n_ritz_num > 2)
		n_nev_new = 2;
	else
		n_nev_new = m_n_eigs_num + std::min(n_converged_num, (m_n_ritz_num - m_n_eigs_num) / 2);
	// adjust n_nev_new, according to dsaup2.f line 677~684 in ARPACK

	return n_nev_new;
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
void CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Get_RitzPairs() // throw(std::bad_alloc, std::runtime_error)
{
	eigenvalues_detail::CTridiagEigsSolver<double> decomp(m_t_fac_H);
	const Eigen::VectorXd &r_v_evals = decomp.v_Eigenvalues();
	const Eigen::MatrixXd &r_t_evecs = decomp.t_Eigenvectors();

	std::vector<size_t> sort_perm;
	eigenvalues_detail::CSortEigenvalues<n_sort_type, b_ascending_sort>::Sort(sort_perm,
		r_v_evals.data(), r_v_evals.rows());

	/*if(SelectionRule == BOTH_ENDS) { // todo - implement this
		std::vector<size_t> perm_copy(sort_perm);
		for(size_t i = 0; i < m_n_ritz_num; ++ i)
			sort_perm[i] = perm_copy[(i & 1)? m_n_ritz_num - 1 - i / 2 : i / 2];
	}*/
	// permute the values to make sure that both the largest and the smallest converge correctly

	for(size_t i = 0; i < m_n_ritz_num; ++ i)
		m_v_ritz_val(i) = r_v_evals(sort_perm[i]);
	for(size_t i = 0; i < m_n_eigs_num; ++ i)
		m_t_ritz_vec.col(i) = r_t_evecs.col(sort_perm[i]);
	// copy the Ritz values and vectors to m_v_ritz_val and m_t_ritz_vec, respectively
}

template <class COpType, int _n_sort_type, bool _b_ascending_sort>
void CSymEigsSolver<COpType, _n_sort_type, _b_ascending_sort>::Sort_RitzPairs() // throw(std::bad_alloc)
{
	std::vector<size_t> sort_perm;
	eigenvalues_detail::CSortEigenvalues<n_sort_type,
		b_ascending_sort>::Sort(sort_perm, m_v_ritz_val.data(), m_n_eigs_num);
	// calculate a sorting permutation

	Eigen::VectorXd v_old_ritz_val = m_v_ritz_val;
	Eigen::MatrixXd t_old_ritz_vec = m_t_ritz_vec;
	std::vector<bool> old_ritz_conv = m_ritz_conv;
	for(size_t i = 0; i < m_n_eigs_num; ++ i) {
		m_v_ritz_val(i) = v_old_ritz_val(sort_perm[i]);
		m_t_ritz_vec.col(i) = t_old_ritz_vec.col(sort_perm[i]);
		m_ritz_conv[i] = old_ritz_conv[sort_perm[i]];
	}
	// copy and permute in O(2n) moves

	// note that sorting this way probably does not damage the
	// Ritz values (no pair is being duplicated or erased)

	// we could use swapping and save memory and do it in swaps
	// (which also amounts to O(2n) global memory accesses)
}

#endif // !__EIGENVALUES_INLINES_INCLUDED
