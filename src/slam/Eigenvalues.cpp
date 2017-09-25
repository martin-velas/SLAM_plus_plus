/*
								+----------------------------------+
								|                                  |
								| ***  Eigenvalue calculation  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|         Eigenvalues.cpp          |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam/Eigenvalues.cpp
 *	@brief eigenvalue calculation
 *	@author -tHE SWINe-
 *	@date 2015-11-16
 */

#include "slam/Eigenvalues.h"
#include "slam/OrderingMagic.h"

/*
 *								=== CGenSparseMatrix_InvProduct ===
 */

CGenSparseMatrix_InvProduct::CGenSparseMatrix_InvProduct(const cs *p_matrix, double f_shift) // throw(std::bad_alloc)
	:m(p_matrix->m), n(p_matrix->n), m_f_shift(f_shift)
{
	// could maybe use Eigen::internal::viewAsEigen(p_matrix);

	std::vector<Eigen::Triplet<double> > triplets;
	triplets.reserve(p_matrix->p[p_matrix->n]);
	for(size_t i = 0; i < n; ++ i) {
		for(size_t p = p_matrix->p[i], e = p_matrix->p[i + 1]; p < e; ++ p) {
			double f_val = p_matrix->x[p];
			size_t n_row = p_matrix->i[p], n_col = i;
			triplets.push_back(Eigen::Triplet<double>(int(n_row), int(n_col), f_val));
		}
	}
	// expand the CSparse matrix to triplets

	SpMat m_mat(int(p_matrix->m), int(p_matrix->n));
	m_mat.setFromTriplets(triplets.begin(), triplets.end());
	// fill the Eigen matrix

	m_mat.makeCompressed();
	// compress the Eigen matrix

	SpMat I((int)m, (int)n);
	I.setIdentity();

	solver.compute(m_mat - I * f_shift);

#ifdef _DEBUG
	//SpMat Q = solver.matrixQ();
	size_t n_fac_nnz = 4 * n/*Q.nonZeros()*/ + solver.matrixR().nonZeros(); // note that counting the NNZ of Q like this is grossly inaccurate
	fprintf(stderr, "debug: the QR factorization has " PRIsize " nonzeros\n", n_fac_nnz);
#endif // _DEBUG
}

/*
 *								=== ~CGenSparseMatrix_InvProduct ===
 */

/*
 *								=== CSymmetricSparseMatrix_InvProduct ===
 */

CSymmetricSparseMatrix_InvProduct::CSymmetricSparseMatrix_InvProduct(const cs *p_matrix, double f_shift) // throw(std::bad_alloc)
	:n(p_matrix->n), m_f_shift(f_shift)
{
	_ASSERTE(p_matrix->m == p_matrix->n);
	// must be square

	SpMat mat(int(p_matrix->m), int(p_matrix->n));
	{
		std::vector<Eigen::Triplet<double> > triplets;
		triplets.reserve(p_matrix->p[p_matrix->n]);
		for(size_t i = 0; i < n; ++ i) {
			for(size_t p = p_matrix->p[i], e = p_matrix->p[i + 1]; p < e; ++ p) {
				double f_val = p_matrix->x[p];
				size_t n_row = p_matrix->i[p], n_col = i;
				triplets.push_back(Eigen::Triplet<double>(int(n_row), int(n_col), f_val));
			}
		}
		// expand the CSparse matrix to triplets

		mat.setFromTriplets(triplets.begin(), triplets.end());
		// fill the Eigen matrix

		mat.makeCompressed();
		// compress the Eigen matrix
	}
	// could maybe use Eigen::internal::viewAsEigen(m_p_matrix); instead (requires eigen update apparently)

	solver.setShift(-f_shift);
	solver.compute(mat);

#ifdef _DEBUG
	size_t n_fac_nnz = ((SpMat)solver.matrixL()).nonZeros(); // type cast required, otherwise it loops forever
	fprintf(stderr, "debug: the LDLT factorization has " PRIsize " nonzeros\n", n_fac_nnz);
#endif // _DEBUG
}

/*
 *								=== ~CSymmetricSparseMatrix_InvProduct ===
 */

/*
 *								=== CSquareSparseMatrix_InvProduct ===
 */

CSquareSparseMatrix_InvProduct::CSquareSparseMatrix_InvProduct(const cs *p_matrix, double f_shift) // throw(std::bad_alloc)
	:n(p_matrix->n), m_f_shift(f_shift)
{
#if 0
	// use ATA ordering with default thresh
#else // 0
	solver.setPivotThreshold(1e-6); // use AT + A ordering with low thresh
#endif // 0
	// could maybe use Eigen::viewAsEigen(p_matrix);

	_ASSERTE(p_matrix->m == p_matrix->n);
	// must be square

	std::vector<Eigen::Triplet<double> > triplets;
	triplets.reserve(p_matrix->p[p_matrix->n]);
	for(size_t i = 0; i < n; ++ i) {
		for(size_t p = p_matrix->p[i], e = p_matrix->p[i + 1]; p < e; ++ p) {
			double f_val = p_matrix->x[p];
			size_t n_row = p_matrix->i[p], n_col = i;
			triplets.push_back(Eigen::Triplet<double>(int(n_row), int(n_col), f_val));
		}
	}
	// expand the CSparse matrix to triplets

	SpMat m_mat((int)n, (int)n);
	m_mat.setFromTriplets(triplets.begin(), triplets.end());
	// fill the Eigen matrix

	m_mat.makeCompressed();
	// compress the Eigen matrix

	SpMat I((int)n, (int)n);
	I.setIdentity();

	solver.compute(m_mat - I * f_shift);

#ifdef _DEBUG
	//size_t n_fac_nnz = ((SpMat)solver.matrixL()).nonZeros() + ((SpMat)solver.matrixU()).nonZeros();
	//fprintf(stderr, "debug: the LU factorization has " PRIsize " nonzeros\n", n_fac_nnz);
	fprintf(stderr, "debug: the LU factorization has finished\n"); // no idea how many nnz, can't tell in eigen
#endif // _DEBUG
}

/*
 *								=== ~CSquareSparseMatrix_InvProduct ===
 */

/*
 *								=== CSquareSparseMatrix_InvProduct2 ===
 */

CSquareSparseMatrix_InvProduct2::CSquareSparseMatrix_InvProduct2(const cs *p_matrix, double UNUSED(f_shift)) // throw(std::bad_alloc)
	:m_temp(p_matrix->n)
{
	_ASSERTE(!f_shift); // not supported here

#if 1
	m_p_symbolic_decomposition = cs_sqr(1, p_matrix, 0); // 1 = ordering A + AT, 2 = ATA
	m_p_factor = cs_lu(p_matrix, m_p_symbolic_decomposition, 1e-6); // tol=1 for ATA ordering, or A+AT with a small tol if the matrix has amostly symmetric nonzero pattern and large enough entries on its diagonal
#else
	m_p_symbolic_decomposition = cs_sqr(2, p_matrix, 0); // 1 = ordering A + AT, 2 = ATA
	m_p_factor = cs_lu(p_matrix, m_p_symbolic_decomposition, 1); // tol=1 for ATA ordering, or A+AT with a small tol if the matrix has amostly symmetric nonzero pattern and large enough entries on its diagonal
#endif
	// do sparse LU

#ifdef _DEBUG
	size_t n_fac_nnz = m_p_factor->L->p[m_p_factor->L->n] + m_p_factor->U->p[m_p_factor->U->n];
	fprintf(stderr, "debug: the LU factorization has " PRIsize " nonzeros\n", n_fac_nnz);
#endif // _DEBUG
}

CSquareSparseMatrix_InvProduct2::~CSquareSparseMatrix_InvProduct2()
{
	cs_sfree(m_p_symbolic_decomposition);
	cs_nfree(m_p_factor);
}

/*
 *								=== ~CSquareSparseMatrix_InvProduct2 ===
 */

/*
 *								=== CPosDefSparseMatrix_InvProduct ===
 */

CPosDefSparseMatrix_InvProduct::CPosDefSparseMatrix_InvProduct(const cs *p_matrix, double UNUSED(f_shift)) // throw(std::bad_alloc)
	:m_temp(p_matrix->n)
{
	_ASSERTE(!f_shift);
	// can't solve for other shifts (could solve for positive shifts, but not for negative ones)

	m_p_symbolic_decomposition = cs_schol(1, p_matrix);
	m_p_factor = cs_chol(p_matrix, m_p_symbolic_decomposition);
	// do sparse cholesky

#ifdef _DEBUG
	size_t n_fac_nnz = m_p_factor->L->p[m_p_factor->L->n];
	fprintf(stderr, "debug: the Cholesky factorization has " PRIsize " nonzeros\n", n_fac_nnz);
#endif // _DEBUG
}

CPosDefSparseMatrix_InvProduct::~CPosDefSparseMatrix_InvProduct()
{
	cs_sfree(m_p_symbolic_decomposition);
	cs_nfree(m_p_factor);
}

/*
 *								=== ~CPosDefSparseMatrix_InvProduct ===
 */

/*
 *								=== CPosDefBlockMatrix_InvProduct ===
 */

CPosDefBlockMatrix_InvProduct::CPosDefBlockMatrix_InvProduct(const CUberBlockMatrix &r_matrix,
	double UNUSED(f_shift)) // throw(std::bad_alloc, std::runtime_error)
	:m_temp(r_matrix.n_Column_Num())
{
	_ASSERTE(r_matrix.b_SymmetricLayout());
	{
		CMatrixOrdering mord;
		const size_t *p_order = mord.p_BlockOrdering(r_matrix, true);
		const size_t *p_inv_order = mord.p_Get_InverseOrdering();
		m_inv_order.insert(m_inv_order.begin(), p_inv_order, p_inv_order + mord.n_Ordering_Size());
	}
	// get ordering, remember it

	CUberBlockMatrix lambda_perm;
	r_matrix.Permute_UpperTriangular_To(lambda_perm, &m_inv_order.front(), m_inv_order.size(), true);
	// permute

	if(!m_R.CholeskyOf(lambda_perm))
		throw std::runtime_error("CPosDefBlockMatrix_InvProduct: matrix not positive definite");
	// factorize
}

/*
 *								=== ~CPosDefBlockMatrix_InvProduct ===
 */

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

std::vector<double> SpSym_Eigenvalues(const cs *p_matrix, size_t n_eig_num,
	bool b_smallest, size_t n_max_iterations, double f_tolerance)
{
	/*size_t n_nnz = p_matrix->p[p_matrix->n], m = p_matrix->m, n = p_matrix->n;
	std::vector<double> eig;
	arma::uvec rowind(n_nnz), colind(n + 1);
	arma::vec values(n_nnz);
	for(size_t i = 0; i < n_nnz; ++ i) {
		rowind[i] = p_matrix->i[i];
		values[i] = p_matrix->x[i];
	}
	for(size_t i = 0; i <= n; ++ i)
		colind[i] = p_matrix->p[i];
	arma::SpMat<double> mat(rowind, colind, values, m, n);
	// convert to armadillo sparse matrix

	arma::vec eigval = arma::eigs_sym(mat, n_eig_num, (b_smallest)? "sm" : "lm");
	// calculate eigenvalues

	eig.insert(eig.end(), &eigval[0], &eigval[0] + eigval.size());
	return eig;*/
	// convert to std::vector
	std::vector<double> eig(n_eig_num, -1);
#if 0 && defined(_DEBUG) // if you want to mock about with this, fine ...
	int nconv = 0;
	if(sizeof(int) == sizeof(csi)) {
		nconv = AREig(&eig[0], /*0,*/ (int)p_matrix->n, (int)p_matrix->p[p_matrix->n], // no eigvec ptr
			(double*)p_matrix->x, (int*)p_matrix->i, (int*)p_matrix->p, 'U', (int)n_eig_num, (b_smallest)? "SM" : "LM");
	} else {
		size_t n_nnz = p_matrix->p[p_matrix->n], m = p_matrix->m, n = p_matrix->n;
		std::vector<int> rowind(n_nnz), colind(n + 1);
		for(size_t i = 0; i < n_nnz; ++ i)
			rowind[i] = (int)p_matrix->i[i];
		for(size_t i = 0; i <= n; ++ i)
			colind[i] = (int)p_matrix->p[i];
		nconv = AREig(&eig[0], /*0,*/ (int)p_matrix->n, (int)p_matrix->p[p_matrix->n], // no eigvec ptr
			(double*)p_matrix->x, (int*)&rowind[0], (int*)&colind[0], 'U', (int)n_eig_num, (b_smallest)? "SM" : "LM");
	}
	eig.resize(nconv); // return only the converged ones?
#endif // _DEBUG

	srand(123456);
	// want to be repeatable

	size_t n_ritz_num = std::min(std::max(2 * n_eig_num + 1, size_t(20)), size_t(p_matrix->n));
	// this seems to be the reasoning Matlab uses (about the twice + 1, and 20 at least)

	if(b_smallest) { // otherwise runs twice
		/*printf("=== R ===\n");
#if defined(_WIN32) || defined(_WIN64)
		system("get_lambda_eigenvalues.bat");
#else // _WIN32 || _WIN64
		system("./get_lambda_eigenvalues.sh");
#endif // _WIN32 || _WIN64
		printf("=== ~R ===\n");*/

		// t_odo - choose matrix decomposition

		/*{
			CPosDefSparseMatrix_InvProduct prod_chol(p_matrix, 0);
			CSymEigsShiftSolver<CPosDefSparseMatrix_InvProduct> solver(prod_chol, n_eig_num, 20); // use the inverse-shift mode
			solver.Init();
			solver.Compute();
			// compute eigenvalues of a symmetric matrix

			Eigen::VectorXd ev = solver.v_Eigenvalues();
			eig.resize(ev.rows());
			if(ev.rows()) {
				Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
				printf("debug: the smallest eigenvalue using CSparse Cholesky: %.10g\n", eig.back());
			} else
				fprintf(stderr, "error: eigenvalues using CSparse Cholesky did not converge\n");
			// copy them to the output vector
		}
		{
			CSquareSparseMatrix_InvProduct2 prod_cslu(p_matrix, 0);
			CSymEigsShiftSolver<CSquareSparseMatrix_InvProduct2> solver(prod_cslu, n_eig_num, 20); // use the inverse-shift mode
			solver.Init();
			solver.Compute();
			// compute eigenvalues of a symmetric matrix

			Eigen::VectorXd ev = solver.v_Eigenvalues();
			eig.resize(ev.rows());
			if(ev.rows()) {
				Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
				printf("debug: the smallest eigenvalue using CSparse LU: %.10g\n", eig.back());
			} else
				fprintf(stderr, "error: eigenvalues using CSparse LU did not converge\n");
			// copy them to the output vector
		}
		{
			CSymmetricSparseMatrix_InvProduct prod_ldlt(p_matrix, 0);
			CSymEigsShiftSolver<CSymmetricSparseMatrix_InvProduct> solver(prod_ldlt, n_eig_num,
				std::min(n_eig_num * 2, size_t(p_matrix->n))); // use the inverse-shift mode
			solver.Init();
			solver.Compute();
			// compute eigenvalues of a symmetric matrix

			Eigen::VectorXd ev = solver.v_Eigenvalues();
			eig.resize(ev.rows());
			/ *if(ev.rows())* / {
				Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
				//printf("debug: the smallest eigenvalue using Eigen LDLT: %.10g\n", eig.back());
			} / *else
				fprintf(stderr, "error: eigenvalues using Eigen LDLT did not converge\n");* /
			// copy them to the output vector
		}
		{
			CSquareSparseMatrix_InvProduct prod_eiglu(p_matrix, 0);
			CSymEigsShiftSolver<CSquareSparseMatrix_InvProduct> solver(prod_eiglu, n_eig_num, 20); // use the inverse-shift mode
			solver.Init();
			solver.Compute();
			// compute eigenvalues of a symmetric matrix

			Eigen::VectorXd ev = solver.v_Eigenvalues();
			eig.resize(ev.rows());
			if(ev.rows()) {
				Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
				printf("debug: the smallest eigenvalue using Eigen LU: %.10g\n", eig.back());
			} else
				fprintf(stderr, "error: eigenvalues using Eigen LU did not converge\n");
			// copy them to the output vector
		}
		{
			CGenSparseMatrix_InvProduct prod_eigqr(p_matrix, 0); // does not seem to finish
			CSymEigsShiftSolver<CGenSparseMatrix_InvProduct> solver(prod_eigqr, n_eig_num, 20); // use the inverse-shift mode
			solver.Init();
			solver.Compute();
			// compute eigenvalues of a symmetric matrix

			Eigen::VectorXd ev = solver.v_Eigenvalues();
			eig.resize(ev.rows());
			if(ev.rows()) {
				Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
				printf("debug: the smallest eigenvalue using Eigen QR: %.10g\n", eig.back());
			} else
				fprintf(stderr, "error: eigenvalues using Eigen QR did not converge\n");
			// copy them to the output vector
		}*/

		CSymmetricSparseMatrix_InvProduct prod_ldlt(p_matrix, 0); // LDLT sparse, suitable for lambdas
		//CGenSparseMatrix_InvProduct prod_qr(p_matrix, 0);
		//CSquareSparseMatrix_InvProduct2 prod(p_matrix, 0);
		CSymEigsShiftSolver<CSymmetricSparseMatrix_InvProduct> solver(prod_ldlt,
			n_eig_num, n_ritz_num); // use the inverse-shift mode
		solver.Init();
		solver.Compute(n_max_iterations, f_tolerance);
		// compute eigenvalues of a symmetric matrix

		Eigen::VectorXd ev = solver.v_Eigenvalues();
		eig.resize(ev.rows());
		if(ev.rows())
			Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
		// copy them to the output vector
	} else {
		CSparseMatrixProduct prod(p_matrix);
		CSymEigsSolver<CSparseMatrixProduct> solver(prod, n_eig_num, n_ritz_num);
		solver.Init();
		solver.Compute(n_max_iterations, f_tolerance);
		// compute eigenvalues of a symmetric matrix

		Eigen::VectorXd ev = solver.v_Eigenvalues();
		eig.resize(ev.rows());
		if(ev.rows())
			Eigen::Map<Eigen::VectorXd, Eigen::DontAlign>(&eig[0], eig.size()) = ev;
		// copy them to the output vector
	}

	return eig;
}

#ifdef __EIGENVALUES_BUILD_UNIT_TESTS

#include "slam/Parser.h"
#include "slam/Debug.h"

/**
 *	@brief unit test for eigenvalue calculation
 *
 *	With the benchmark composed of the following UFLSMC matrices:
 *
 *	@code
 *	symmetric/685_bus.mtx
 *	symmetric/bcsstk04.mtx
 *	symmetric/bcsstk34.mtx
 *	symmetric/ex2.mtx
 *	symmetric/lund_a.mtx
 *	symmetric/mesh2e1.mtx
 *	symmetric/msc00726.mtx
 *	symmetric/nos3.mtx
 *	symmetric/plat362.mtx
 *	symmetric/Si2.mtx
 *	symmetric/USAir97.mtx
 *	pos_def/HB_494_bus.mtx
 *	pos_def/HB_bcsstk01.mtx
 *	pos_def/HB_bcsstk02.mtx
 *	pos_def/HB_bcsstk03.mtx
 *	pos_def/HB_bcsstk04.mtx
 *	pos_def/HB_bcsstk05.mtx
 *	pos_def/HB_bcsstk06.mtx
 *	pos_def/HB_bcsstk07.mtx
 *	pos_def/HB_bcsstk20.mtx
 *	pos_def/HB_bcsstk22.mtx
 *	pos_def/HB_bcsstm02.mtx
 *	pos_def/HB_bcsstm05.mtx
 *	pos_def/HB_bcsstm06.mtx
 *	pos_def/HB_bcsstm07.mtx
 *	pos_def/HB_bcsstm20.mtx
 *	pos_def/HB_bcsstm22.mtx
 *	pos_def/HB_lund_a.mtx
 *	pos_def/HB_lund_b.mtx
 *	pos_def/HB_nos1.mtx
 *	pos_def/HB_nos4.mtx
 *	pos_def/HB_nos5.mtx
 *	pos_def/HB_plat362.mtx
 *	pos_def/Bai_mhdb416.mtx
 *	pos_def/FIDAP_ex5.mtx
 *	pos_def/Pothen_mesh1e1.mtx
 *	pos_def/Pothen_mesh1em1.mtx
 *	pos_def/Pothen_mesh1em6.mtx
 *	pos_def/Pothen_mesh2e1.mtx
 *	pos_def/Pothen_mesh2em5.mtx
 *	pos_def/Pothen_mesh3e1.mtx
 *	pos_def/Pothen_mesh3em5.mtx
 *	pos_def/Oberwolfach_LF10.mtx
 *	pos_def/Oberwolfach_LFAT5.mtx
 *	pos_def/Pajek_Journals.mtx
 *	pos_def/JGD_Trefethen_Trefethen_20b.mtx
 *	pos_def/JGD_Trefethen_Trefethen_20.mtx
 *	pos_def/JGD_Trefethen_Trefethen_150.mtx
 *	pos_def/JGD_Trefethen_Trefethen_200b.mtx
 *	pos_def/JGD_Trefethen_Trefethen_200.mtx
 *	pos_def/JGD_Trefethen_Trefethen_300.mtx
 *	pos_def/JGD_Trefethen_Trefethen_500.mtx
 *	@endcode
 *
 *	Code compiled using VS2008 running on Opteron 2360 SE gives the
 *	following reults (for the largest magnitude eigenvalues tests):
 *
 *	@code
 *	CRC32 of the results: 0x6643fadd exact / 0x5d517729 rounded - with the original code
 *	CRC32 of the results: 0x6643fadd exact / 0x5d517729 rounded - with the optimized tridiagonal qr
 *	CRC32 of the results: 0x74f90197 exact / 0x5951f23b rounded - with (the unmodified) hessenberg qr
 *	CRC32 of the results: 0x7e63bf74 exact / 0x00c60dea rounded - with the optimized hessenberg qr
 *		(introduced by negating the sine, the change is somewhere in Compute())
 *	@endcode
 *
 *	The eigenvalues are almost always within 1e-15 relative error
 *	compared to ones obtained by Matlab R2008a.
 */
class CEigsUnitTest {
protected:
	/**
	 *	@brief CRC calculation template
	 *
	 *	@param TScalar is CRC scalar type (uint16_t for crc16, uint32_t for crc32)
	 *	@param n_polynom is CRC polynom
	 *	@param n_start is CRC starting value
	 *	@param n_final_xor is final reflection value (usually all 0's or all 1's)
	 *
	 *	@note It is possible to use predefined specializations CCrc_16 and CCrc_32,
	 *		implementing standard CRC16 and CRC32 algorithms, respectively.
	 */
	template<class TScalar, const TScalar n_polynom, const TScalar n_start, const TScalar n_final_xor>
	class CCrc {
	public:
		/**
		 *	@brief gets CRC starting value
		 *	@return Returns starting value of CRC.
		 */
		static inline TScalar n_Start()
		{
			return n_start;
		}

		/**
		 *	@brief calculates final xor
		 *
		 *	@param[in] n_prev_crc is either n_Start() (CRC of empty buffer),
		 *		or result of previous call to n_Crc() (when calculating CRC of one or more
		 *		concatenated buffers)
		 *
		 *	@return Returns final value of CRC.
		 */
		static inline TScalar n_Finalize(TScalar n_prev_crc)
		{
			return n_prev_crc ^ n_final_xor;
		}

		/**
		 *	@brief calculates CRC on a stream of data
		 *
		 *	Calculates CRC of n_size bytes from buffer p_data.
		 *
		 *	@param[in] n_size is size of input data, in bytes
		 *	@param[in] p_data is input data buffer, contains n_size bytes
		 *	@param[in] n_prev_crc is either n_Start() (starting new CRC calculation),
		 *		or result of previous call to n_Crc() (when calculating CRC of more
		 *		concatenated buffers)
		 *
		 *	@return Returns CRC value, no data reflection, no final xor.
		 */
		static TScalar n_Crc(size_t n_size, const void *p_data, TScalar n_prev_crc /*= n_start*/) // otherwise ambiguous with the default param
		{
			const uint8_t *_p_data = (const uint8_t*)p_data;

			static TScalar p_crc_table[256];
			static bool b_crc_table_ready = false;
			if(!b_crc_table_ready) {
				b_crc_table_ready = true;
				for(int i = 0; i < 256; ++ i) {
					TScalar n_root = TScalar(i);
					for(int j = 0; j < 8; ++ j) {
						if(n_root & 1)
							n_root = (n_root >> 1) ^ n_polynom;
						else
							n_root >>= 1;
					}
					p_crc_table[i] = n_root;
				}
			}
			// prepare the table

			TScalar n_crc = n_prev_crc;
			for(const uint8_t *p_end = _p_data + n_size; _p_data != p_end; ++ _p_data)
				n_crc = p_crc_table[(n_crc ^ *_p_data) & 0xff] ^ (n_crc >> 8);
			// calculate CRC

			return n_crc;
		}

		/**
		 *	@brief calculates CRC
		 *
		 *	Calculates CRC of n_size bytes from buffer p_data.
		 *
		 *	@param[in] n_size is size of input data, in bytes
		 *	@param[in] p_data is input data buffer, contains n_size bytes
		 *
		 *	@return Returns the final CRC value.
		 */
		static inline TScalar n_Crc(size_t n_size, const void *p_data)
		{
			return n_Finalize(n_Crc(n_size, p_data, n_Start()));
		}
	};

	typedef CCrc<uint16_t, 0xA001, 0xffff, 0xffff> CCrc_16; /**< CRC-16, as defined by ANSI (X3.28) */
	typedef CCrc<uint32_t, 0xedb88320U, 0xffffffffU, 0xffffffffU> CCrc_32; /**< CRC-32, as defined by POSIX */

	/**
	 *	@brief Numerical Recipes' long generator (period 3.138 * 10^57)
	 *	@tparam b_use_virtual_interface is common interface flag (default true)
	 */
	class CNRLongGenerator {
	public:
		/**
		 *	@brief random number generator traits, stored as enum
		 */
		enum {
			b_seedable = true, /**< @brief seed not ignored flag */
			b_64bit_rand = true, /**< @brief 64-bit random number capability flag */
			b_crypto_safe = false, /**< @brief cryptographically safe flag */
			b_fast_floats = false /**< @brief fast (possibly lower quality) floating point random number generation flag */
		};

	protected:
		uint64_t m_n_u; /**< @brief random generator state */
		uint64_t m_n_v; /**< @brief random generator state */
		uint64_t m_n_w; /**< @brief random generator state */

	public:
		/**
		 *	@brief constructor; initializes the random generator
		 */
		CNRLongGenerator(uint64_t n_seed = 0)
		{
			Seed64(n_seed);
		}

		/**
		 *	@copydoc CRandomGeneratorModel::Seed()
		 */
		void Seed(unsigned int n_seed)
		{
			Seed64(n_seed);
		}

		/**
		 *	@copydoc CRandomGeneratorModel::Seed()
		 */
		void Seed64(uint64_t n_seed)
		{
			m_n_v = (uint64_t(0x38ecac5f) << 32) |	0xb3251641U; // 4101842887655102017LL
			m_n_w = 1;
			//_ASSERTE(n_seed != m_n_v); // can happen, don't want this to kill my process in the unlikely event
			m_n_u = n_seed ^ m_n_v;
			n_Rand64();
			m_n_v = m_n_u;
			n_Rand64();
			m_n_w = m_n_v;
			n_Rand64();
		}

		/**
		 *	@brief generates a 64-bit unsigned random number
		 *	@return Returns the generated random number.
		 */
		uint64_t n_Rand64()
		{
			m_n_u = m_n_u * ((uint64_t(0x27bb2ee6U) << 32) | 0x87b0b0fdU)/*2862933555777941757LL*/ +
				((uint64_t(0x61c88646U) << 32) | 0x80b583bfU)/*7046029254386353087LL*/;
			m_n_v ^= m_n_v >> 17;
			m_n_v ^= m_n_v << 31;
			m_n_v ^= m_n_v >> 8;
			m_n_w = 4294957665U * (m_n_w & 0xffffffff) + (m_n_w >> 32);
			uint64_t x = m_n_u ^ (m_n_u << 21);
			x ^= x >> 35;
			x ^= x << 4;
			return (x + m_n_v) ^ m_n_w;
		}

		/**
		 *	@brief generates a 32-bit unsigned random number
		 *	@return Returns the generated random number.
		 */
		uint32_t n_Rand32()
		{
			return uint32_t(n_Rand64());
		}

		/**
		 *	@copydoc CRandomGeneratorModel::f_Rand()
		 */
		double f_Rand()
		{
			int64_t n_rand = n_Rand64() >> 10; // msvc 6.0 does not implement conversion from uint64_t to double
			while(n_rand > (int64_t(1) << 53)) // generates 54-bit numbers, but we're interested only in 1 + 53 zeroes or below
				n_rand = n_Rand64() >> 10; // can try again, the samples are independent
			return double(n_rand) * (1.0 / 9007199254740992.0); // n_rand is [0, 9007199254740992] inclusive
			// more uniformly distributed floats
			// the division is precise, 9007199254740992.0 is a power of two
		}
	};

public:
	static bool Run(bool b_largest_eigenvalues_benchmark = true) // throw(std::bad_alloc)
	{
		std::vector<std::string> matrix_list;
		{
			FILE *p_fr;
			if(!(p_fr = fopen("data/bench_eigs/list.txt", "r"))) {
				fprintf(stderr, "error: failed to open \'%s\'\n", "data/bench_eigs/list.txt");
				return false;
			}
			std::string s_line;
			while(!feof(p_fr)) {
				if(!CParserBase::ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					fprintf(stderr, "error: while reading \'%s\'\n", "data/bench_eigs/list.txt");
					return false;
				}
				CParserBase::TrimSpace(s_line);
				if(!s_line.empty())
					matrix_list.push_back(s_line);
			}
			fclose(p_fr);
		}
		// get a list of matrices

		uint32_t n_crc = CCrc_32::n_Start();
		uint32_t n_crc_approx = CCrc_32::n_Start();

		for(size_t i = 0, n = matrix_list.size(); i < n; ++ i) {
			printf("%s\n", matrix_list[i].c_str());
			// verbose

			CUberBlockMatrix A, sD, sV;
			if(!A.Load_MatrixMarket(("data/bench_eigs/" + matrix_list[i]).c_str(), 1, false)) {
				fprintf(stderr, "warning: failed to load \'%s\'\n", ("data/bench_eigs/" + matrix_list[i]).c_str());
				continue;
			}
			if(!sD.Load_MatrixMarket(("data/bench_eigs/" + matrix_list[i] +
			   ((b_largest_eigenvalues_benchmark)? ".eigvals.mtx" : ".eigvals.sm.mtx")).c_str(), 1, false)) {
				fprintf(stderr, "warning: failed to load \'%s\'\n", ("data/bench_eigs/" +
					matrix_list[i] + ((b_largest_eigenvalues_benchmark)? ".eigvals.mtx" : ".eigvals.sm.mtx")).c_str());
				continue;
			}
			if(!sV.Load_MatrixMarket(("data/bench_eigs/" + matrix_list[i] +
			   ((b_largest_eigenvalues_benchmark)? ".eigvecs.mtx" : ".eigvecs.sm.mtx")).c_str(), 1, false)) {
				fprintf(stderr, "warning: failed to load \'%s\'\n", ("data/bench_eigs/" +
					matrix_list[i] + ((b_largest_eigenvalues_benchmark)? ".eigvecs.mtx" : ".eigvecs.sm.mtx")).c_str());
				continue;
			}
			Eigen::VectorXd D(sD.n_Row_Num());
			Eigen::MatrixXd V;
			sD.Convert_to_Dense(D); // requires allocation by the caller
			sV.Convert_to_Dense(V);
			if(D.rows() != V.cols() || size_t(V.rows()) != A.n_Column_Num() || !A.b_Square()) {
				fprintf(stderr, "warning: dimension mismatch in\'%s\'\n", ("data/bench_eigs/" + matrix_list[i]).c_str());
				continue;
			}
			// load all the matrices

			srand(123456);
			// the eigenvalue calculation starts from a randomized vector, make sure the results are always the same

			size_t n_eig_num = std::min(size_t(D.rows()), A.n_Column_Num() - 1); // somehow this solver cannot recover all of the eigenvalues

			Eigen::VectorXd v_eval;
			Eigen::MatrixXd t_evec;
			if(b_largest_eigenvalues_benchmark) {
				CBlockMatrixProduct prod(A);
				CSymEigsSolver<CBlockMatrixProduct> solver(prod, n_eig_num,
					std::min(std::max(n_eig_num * 2 + 1, size_t(20)), A.n_Column_Num()));
				try {
					solver.Init();
					solver.Compute();
				} catch(std::exception &r_exc) {
					fprintf(stderr, "warning: eigensolver threw an exception: \'%s\'\n", r_exc.what());
					continue;
				}
				// calculate eigenvalues (note that the input matrix must be symmetric!)

				v_eval = solver.v_Eigenvalues();
				t_evec = solver.t_Eigenvectors(v_eval.rows());
			} else {
				cs *p_A;
				if(!(p_A = A.p_Convert_to_Sparse())) {
					fprintf(stderr, "warning: not enough memory to get elemwise sparse form\n");
					continue;
				}
				CGenSparseMatrix_InvProduct prod(p_A, .0);
				CSymEigsShiftSolver<CGenSparseMatrix_InvProduct> solver(prod, n_eig_num,
					std::min(std::max(n_eig_num * 2 + 1, size_t(20)), A.n_Column_Num()));
				try {
					CNRLongGenerator rng(123456);
					Eigen::VectorXd init_resid(A.n_Column_Num()); // explicit seeding doesn't seem to help greatly
					do {
						for(size_t j = 0, m = init_resid.rows(); j < m; ++ j)
							init_resid(j) = rng.f_Rand() - .5;
					} while(init_resid.norm() <= 1e-3);
					// explicit init, use a better quality randomness than libc

					solver.Init(init_resid);
					if(!solver.Compute()) {
						fprintf(stderr, "warning: inverse / shift mode eigensolver found no"
							" eigenvalues, trying to reduce precision\n");

						do {
							for(size_t j = 0, m = init_resid.rows(); j < m; ++ j)
								init_resid(j) = rng.f_Rand() - .5;
						} while(init_resid.norm() <= 1e-3);
						solver.Init(init_resid);

						if(!solver.Compute(10000, 1e-5)) {
							fprintf(stderr, "warning: inverse / shift mode eigensolver found no"
								" eigenvalues, trying to reduce precision even more\n");

							do {
								for(size_t j = 0, m = init_resid.rows(); j < m; ++ j)
									init_resid(j) = rng.f_Rand() - .5;
							} while(init_resid.norm() <= 1e-3);
							solver.Init(init_resid);

							solver.Compute(10000, 1e-2);
						}
						// try to compute with lower tolerance
					}
				} catch(std::exception &r_exc) {
					if(p_A)
						cs_spfree(p_A);
					fprintf(stderr, "warning: inverse / shift mode eigensolver"
						" threw an exception: \'%s\'\n", r_exc.what());
					continue;
				}
				if(p_A)
					cs_spfree(p_A);
				// calculate eigenvalues (note that the input matrix must be symmetric!)

				v_eval = solver.v_Eigenvalues();
				t_evec = solver.t_Eigenvectors(v_eval.rows());
			}
			// get eigenvalues and eigenvectors

			FlipSigns(D, V);

			SortEigvals(D, V);
			// matlab can't sort its eigenvalues correctly if using [V, D] = eigs(M, 20, 'LM')
			// although quite curiously D = eigs(M, 20, 'LM') is sorted just fine

			double f_min_eigenvalue_delta = (D.head(D.rows() - 1) - D.tail(D.rows() - 1)).minCoeff();
			_ASSERTE(f_min_eigenvalue_delta >= 0); // the eigenvalues are sorted, this should be positive
			// calculate minimum eigenvalue delta

			bool b_equal_ev;
			if((b_equal_ev = (f_min_eigenvalue_delta < 1e-5))) {
				fprintf(stderr, "warning: have repeated eigenvalues (%.6e), eigenvector"
					" comparison only approximate\n", f_min_eigenvalue_delta);
			}
			// if there are multiple almost equal eigenvalues, the corresponding eigenvectors might be ordered differently

			if(size_t(v_eval.rows()) < n_eig_num) {
				fprintf(stderr, "warning: only " PRIsize " eigenvalues converged\n", size_t(v_eval.rows()));
				if(!v_eval.rows())
					continue;
				if(b_largest_eigenvalues_benchmark) {
					/*D = D.head(v_eval.rows());
					V = V.leftCols(v_eval.rows());*/ // scrambles the contents (Eigen bug?)
					D.conservativeResize(v_eval.rows());
					V.conservativeResize(V.rows(), v_eval.rows());
				} else {
					/*D = D.tail(v_eval.rows());
					V = V.rightCols(v_eval.rows());*/ // scrambles the contents (Eigen bug?)
					Eigen::VectorXd Dn = D.tail(v_eval.rows());
					Eigen::MatrixXd Vn = V.rightCols(v_eval.rows());
					D = Dn;
					V = Vn;
				}
			}
			// handle cases which did not converge

			if(D.rows() > v_eval.rows()) {
				/*D = D.head(v_eval.rows());
				V = V.leftCols(v_eval.rows());*/ // scrambles the contents (Eigen bug?)
				D.conservativeResize(v_eval.rows());
				V.conservativeResize(V.rows(), v_eval.rows());
			}
			// handle very small matrices

			FlipSigns(v_eval, t_evec);

			double f_eigvals_err = (v_eval - D).norm();
			double f_eigvecs_err = (!b_equal_ev)? f_EigenVector_Difference_Norm(t_evec, V) :
				f_EigenVector_UnorderedDifference_Norm(t_evec, V); // the eigenvectors are complicated
			// calculate errors

			FILE *p_fw;
			if((p_fw = fopen(("data/bench_eigs/" + matrix_list[i] +
			   ((b_largest_eigenvalues_benchmark)? ".eigvecs_cpp.m" : ".eigvecs_cpp.sm.m")).c_str(), "w"))) {
				CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw,
					t_evec, "V = ", ";\n", " %.15g");
				fclose(p_fw);
			}
			if((p_fw = fopen(("data/bench_eigs/" + matrix_list[i] +
			   ((b_largest_eigenvalues_benchmark)? ".eigvals_cpp.m" : ".eigvals_cpp.sm.m")).c_str(), "w"))) {
				CDebug::Print_DenseMatrix_in_MatlabFormat(p_fw,
					v_eval, "D = ", ";\n", " %.15g");
				fclose(p_fw);
			}
			// write the eigenvalues / eigenvectors for a closer inspection in matlab

			printf("eigenvalues residual: %.6e (%.6e), eigenvectors residual: %.6e (%.6e)\n",
				f_eigvals_err, f_eigvals_err / std::max(D.norm(), 1.0),
				f_eigvecs_err, f_eigvecs_err / std::max(V.norm(), 1.0));
			// verbose

			n_crc = CCrc_32::n_Crc(sizeof(double), &f_min_eigenvalue_delta, n_crc);
			n_crc = CCrc_32::n_Crc(sizeof(double), &f_eigvals_err, n_crc);
			double f_tmp = f_eigvals_err / D.norm();
			n_crc = CCrc_32::n_Crc(sizeof(double), &f_tmp, n_crc);
			n_crc = CCrc_32::n_Crc(sizeof(double), &f_eigvecs_err, n_crc);
			f_tmp = f_eigvecs_err / V.norm();
			n_crc = CCrc_32::n_Crc(sizeof(double), &f_tmp, n_crc);
			char p_s_result[256];
			sprintf(p_s_result, "%.8g %.8g %.8g %.8g %.8g", f_min_eigenvalue_delta,
				f_eigvals_err, f_eigvals_err / D.norm(), f_eigvecs_err, f_eigvecs_err / V.norm());
			n_crc_approx = CCrc_32::n_Crc(strlen(p_s_result) * sizeof(char), p_s_result, n_crc_approx);
			// calculate CRC of the exact and rounded results (here the rounding is
			// performed in the most ad-hoc way possible by conversion to a string)
		}

		n_crc = CCrc_32::n_Finalize(n_crc);
		n_crc_approx = CCrc_32::n_Finalize(n_crc_approx);
		// finalize the CRCs

		printf("CRC32 of the results: 0x%08x exact / 0x%08x rounded\n", n_crc, n_crc_approx);
		// print CRC of the results to make the comparison of the implementations easier
		// (just compare a crc instead of comparing all the numbers)

		return true;
	}

	static double f_EigenVector_UnorderedDifference_Norm(const Eigen::MatrixXd &r_t_eigenvectors_a,
		const Eigen::MatrixXd &r_t_eigenvectors_b)
	{
		_ASSERTE(r_t_eigenvectors_a.rows() == r_t_eigenvectors_b.rows() &&
			r_t_eigenvectors_a.cols() == r_t_eigenvectors_b.cols());

		Eigen::VectorXd v_a(r_t_eigenvectors_a.rows()), v_b(r_t_eigenvectors_a.rows());
		for(size_t i = 0, n = r_t_eigenvectors_a.rows(); i < n; ++ i) {
			v_a(i) = (r_t_eigenvectors_a.row(i).array() * r_t_eigenvectors_a.row(i).array()).sum();
			v_b(i) = (r_t_eigenvectors_b.row(i).array() * r_t_eigenvectors_b.row(i).array()).sum();
		}
		// the ordering of the individual eigenvectors is uncertain; just sum all the vectors horizontally

		// the square is needed since the sign ambiguity in the vector direction is unavoidable, see e.g.
		// Hasan, Khader M., et al. "Analytical computation of the eigenvalues and eigenvectors in DT-MRI."
		// Journal of Magnetic Resonance 152.1 (2001): 41-47.

		return sqrt(fabs((v_a - v_b).sum()));
	}

	static double f_EigenVector_Difference_Norm(const Eigen::MatrixXd &r_t_eigenvectors_a,
		const Eigen::MatrixXd &r_t_eigenvectors_b)
	{
		_ASSERTE(r_t_eigenvectors_a.rows() == r_t_eigenvectors_b.rows() &&
			r_t_eigenvectors_a.cols() == r_t_eigenvectors_b.cols());
		double f_squared_norm = 0;
		for(size_t i = 0, n = r_t_eigenvectors_a.cols(); i < n; ++ i) {
			f_squared_norm += std::min((r_t_eigenvectors_a.col(i) - r_t_eigenvectors_b.col(i)).squaredNorm(),
				(r_t_eigenvectors_a.col(i) + r_t_eigenvectors_b.col(i)).squaredNorm());
			// sign ambiguity in the vector direction is unavoidable, see e.g. Hasan, Khader M., et al.
			// "Analytical computation of the eigenvalues and eigenvectors in DT-MRI." Journal of Magnetic
			// Resonance 152.1 (2001): 41-47.
		}
		return sqrt(f_squared_norm);
	}

	static void FlipSigns(Eigen::VectorXd &r_v_eigenvalues, Eigen::MatrixXd &r_t_eigenvectors)
	{
		_ASSERTE(r_v_eigenvalues.rows() == r_t_eigenvectors.cols()); // as many eigenvectors as eigenvalues
		for(size_t i = 0, n = r_v_eigenvalues.rows(); i < n; ++ i) {
			if(r_v_eigenvalues(i) < 0) {
				r_v_eigenvalues(i) = -r_v_eigenvalues(i);
				r_t_eigenvectors.col(i) = -r_t_eigenvectors.col(i); // if the eigenvalue sign is flipped, the eigenvector direction changes
			}
			// if the eigenvalue is negative, the corresponding eigenvector gets flipped
			// (not that it matters, the directions are ambiguous anyways)
		}
	}

	static void SortEigvals(Eigen::VectorXd &r_v_eigenvalues, Eigen::MatrixXd &r_t_eigenvectors) // throw(std::bad_alloc)
	{
		_ASSERTE(r_v_eigenvalues.rows() == r_t_eigenvectors.cols()); // as many eigenvectors as eigenvalues

		std::vector<size_t> perm;
		eigenvalues_detail::CSortEigenvalues<eigen_ByMagnitude, false>::Sort(perm,
			r_v_eigenvalues.data(), r_v_eigenvalues.rows());
		// create a permutation of eigenvalues

		Eigen::VectorXd v_sorted_eigvals = r_v_eigenvalues;
		Eigen::MatrixXd v_sorted_eigvecs = r_t_eigenvectors;
		for(size_t i = 0, n = r_v_eigenvalues.rows(); i < n; ++ i) {
			r_v_eigenvalues(i) = v_sorted_eigvals(perm[i]);
			r_t_eigenvectors.col(i) = v_sorted_eigvecs.col(perm[i]);
		}
	}
};

void Eigenvalues_UnitTest()
{
	CEigsUnitTest::Run(true);
	printf("and now for the smallest eigenvalues ...\n");
	CEigsUnitTest::Run(false);
}

#else // __EIGENVALUES_BUILD_UNIT_TESTS

void Eigenvalues_UnitTest()
{
	fprintf(stderr, "error: __EIGENVALUES_BUILD_UNIT_TESTS not defined\n");
}

#endif // __EIGENVALUES_BUILD_UNIT_TESTS
