/*
								+----------------------------------+
								|                                  |
								| ***  Eigenvalue calculation  *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|          Eigenvalues.h           |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file include/slam/Eigenvalues.h
 *	@brief eigenvalue calculation
 *	@author -tHE SWINe-
 *	@date 2015-11-16
 */

#pragma once
#ifndef __EIGENVALUES_INCLUDED
#define __EIGENVALUES_INCLUDED

/**
 *	@def __EIGENVALUES_BUILD_UNIT_TESTS
 *	@brief if defined, unit tests for the eigensolver are compiled
 *	@note These tests require external data (some matrices from UFLSMC and the ground truth).
 */
#define __EIGENVALUES_BUILD_UNIT_TESTS

#include "slam/BlockMatrix.h"
#include "eigen/Eigen/Core"
#include "eigen/Eigen/Eigenvalues"
#include <complex>
#include <numeric>
#include <vector>

/**
 *	@brief calculates eigenvalues of a sparse symmetric matrix
 *
 *	@param[in] p_matrix is matrix in question (must be square and symmetric)
 *	@param[in] n_eig_num is number of eigenvalues to compute (must be less than the size of the matrix)
 *	@param[in] b_smallest is small eigenvalue flag (if set, an LDLT decomposition of the given matrix
 *		is calculated and the smallest eigenvalues are then recovered using the numerically stable
 *		shift / inverse mode; if cleared then largest (by magnitude) eigenvalues are recovered)
 *
 *	@return Returns a vector of recovered eigenvalues, sorted by magnitude
 *		and in the order depending on b_smallest.
 *
 *	@note This function throws std::bad_alloc.
 */
std::vector<double> SpSym_Eigenvalues(const cs *p_matrix, size_t n_eig_num,
	bool b_smallest = false, size_t n_max_iterations = 1000, double f_tolerance = 1e-10); // throw(std::bad_alloc)

/**
 *	@brief eigenvalue sorting mode
 */
enum {
	eigen_ByMagnitude = 0, /**< @brief sort eigenvalues by magnitude */
	eigen_ByAlgebraic, /**< @brief sort eigenvalues by algebraic value */
	eigen_ByReal, /**< @brief sort eigenvalues by algebraic value of the real component */
	eigen_ByImag  /**< @brief sort eigenvalues by algebraic value of the imaginary component */
};

/**
 *	@brief helper object for calculation of eigenvalues of a block matrix
 */
class CBlockMatrixProduct {
protected:
	const CUberBlockMatrix &m_r_matrix; /**< @brief reference to the matrix */

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *	@param[in] r_matrix is const reference to the matrix to be multiplied
	 *	@note This matrix is only referenced (not copied) and must not
	 *		be deleted while using this object.
	 */
	CBlockMatrixProduct(const CUberBlockMatrix &r_matrix)
		:m_r_matrix(r_matrix)
	{}

	/**
	 *	@copydoc CUberBlockMatrix::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return m_r_matrix.n_Row_Num();
	}

	/**
	 *	@copydoc CUberBlockMatrix::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return m_r_matrix.n_Column_Num();
	}

	/**
	 *	@brief calculates matrix-vector product
	 *
	 *	@tparam Derived0 is type of the vector to be multiplied
	 *	@tparam Derived1 is type of the vector to store the result
	 *
	 *	@param[in] r_v_x is reference to the vector to be multiplied
	 *	@param[in] r_v_y is reference to the vector to store the result
	 *
	 *	@note The two arguments must not point to the same vector.
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Derived0 &r_v_x, Derived1 &r_v_y) const
	{
		_ASSERTE(r_v_x.data() != r_v_y.data());
		r_v_y.setZero();
		m_r_matrix.PostMultiply_Add(r_v_y.data(), r_v_y.rows(), r_v_x.data(), r_v_x.rows());
	}
};

/**
 *	@brief helper object for calculation of eigenvalues of a sparse matrix
 */
class CSparseMatrixProduct {
protected:
	const cs *m_p_matrix; /**< @brief pointer to the matrix */

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *	@param[in] p_matrix is const pointer to the matrix to be multiplied
	 *	@note This matrix is only referenced (not copied) and must not
	 *		be deleted while using this object.
	 */
	CSparseMatrixProduct(const cs *p_matrix)
		:m_p_matrix(p_matrix)
	{}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return m_p_matrix->m;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return m_p_matrix->n;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::operator()
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Derived0 &r_v_x, Derived1 &r_v_y) const
	{
		_ASSERTE((const void*)&r_v_x != (const void*)&r_v_y); // otherwise can't use .noalias()
		r_v_y.setZero();
		cs_gaxpy(m_p_matrix, r_v_x.data(), r_v_y.data());
	}
};

/**
 *	@brief eigenvalue solver for symmetric matrices (guarantees real eigenvalues)
 *
 *	@tparam COpType is matrix operation type (implements matrix multiplication; default \ref CBlockMatrixProduct)
 *	@tparam _n_sort_type is eigenvalue sort type (one of eigen_By*, default \ref eigen_ByMagnitude)
 *	@tparam _b_ascending_sort is eigenvalue sort direction flag (if set - ascending, if cleared - descending)
 *
 *	@note This can be used to recover up to n - 1 eigenvalues and eigenvectors from
 *		a nxn symmetric matrix. To get the low eigenvalues, typically inverting the
 *		matrix and using \ref CSymEigsShiftSolver produces more accurate results
 *		than then using descending ordering (which fails to converge sometimes).
 */
template <class COpType = CBlockMatrixProduct,
	int _n_sort_type = eigen_ByMagnitude, bool _b_ascending_sort = false>
class CSymEigsSolver {
public:
	typedef COpType _TyOperation; /**< @brief matrix operation type */

	/**
	 *	@brief configuration stored as enum
	 */
	enum {
		n_sort_type = _n_sort_type, /**< @brief sorting type (one of eigen_By*) */
		b_ascending_sort = _b_ascending_sort /**< @brief sorting direction (if set - ascending, if cleared - descending) */
	};

protected:
	const COpType &m_r_op; /**< @brief matrix product operator */
	const size_t m_n; /**< @brief dimension of matrix A */
	const size_t m_n_eigs_num; /**< @brief number of eigenvalues requested */
	const size_t m_n_ritz_num; /**< @brief number of Ritz values */
	size_t m_n_mat_op_num; /**< @brief number of matrix operation invokations */
	size_t m_n_restart_num; /**< @brief number of restarting iterations */

	Eigen::MatrixXd m_t_fac_V; /**< @brief V matrix in the Arnoldi factorization */
	Eigen::MatrixXd m_t_fac_H; /**< @brief H matrix in the Arnoldi factorization */
	Eigen::VectorXd m_v_fac_f; /**< @brief residual in the Arnoldi factorization */

	Eigen::VectorXd m_v_ritz_val; /**< @brief Ritz values */
	Eigen::MatrixXd m_t_ritz_vec; /**< @brief Ritz vectors */
	std::vector<bool> m_ritz_conv; /**< @brief Ritz values convergence flags */

	const double m_f_prec; /**< @brief precision parameter used to test convergence, equals epsilon^(2/3) */

public:
	/**
	 *	@brief default constructor; creates an uninitialized solver object
	 *
	 *	@param[in] r_op is reference to a matrix operation object (matrix multiplication;
	 *		this object is only referenced and must not be deleted before calling \ref Compute())
	 *	@param[in] n_eigs_num is the number of requested eigenvalues (must be less than the number
	 *		of columns of the decomposed matrix)
	 *	@param[in] n_ritz_num is the number of Ritz vectors; must be larger than n_eigs_num and
	 *		smaller or equal to the number of columns of the decomposed matrix; higher values
	 *		tend to improve convergence)
	 *
	 *	@note This does not calculate anything; one needs to call Compute().
	 */
	CSymEigsSolver(const COpType &r_op, size_t n_eigs_num, size_t n_ritz_num);

	/**
	 *	@brief initializes the computation with an explicit residual vector
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@param[in] r_v_init_resid is initial residual vector (must have non-zero
	 *		norm and the same dimension as the matrix being decomposed)
	 */
	template <class Derived0>
	void Init(const Eigen::MatrixBase<Derived0> &r_v_init_resid); // throw(std::bad_alloc)

	/**
	 *	@brief initializes the computation with a random residual vector
	 *
	 *	@note The values of the vector are chosen from uniform distribution
	 *		in the [-0.5, 0.5] interval.
	 *	@note This contains randomness. To always obtain the same results,
	 *		call srand() with some constant seed before using this function.
	 */
	void Init(); // throw(std::bad_alloc)

	/**
	 *	@brief computes the eigenvalues
	 *
	 *	@param[in] n_max_iteration_num is maximum number of iterations (default 1000)
	 *	@param[in] f_tolerance is precision threshold for the calculated eigenvalues (default 1e-10)
	 *
	 *	@return Returns the number of converged eigenvalues.
	 *
	 *	@note This function throws std::bad_alloc or std::runtime_error
	 *		if the tridiagonal eigenvalue solver fails.
	 *	@note In case \ref Init() was not called yet, it is called automatically.
	 */
	size_t Compute(size_t n_max_iteration_num = 1000, double f_tolerance = 1e-10); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief counts the converged eigenvalues
	 *	@return Returns the number of the converged eigenvalues.
	 */
	inline size_t n_Converged_Num() const
	{
		return std::accumulate(m_ritz_conv.begin(), m_ritz_conv.end(), size_t(0));
	}

	/**
	 *	@brief gets the number of iterations
	 *	@return Returns the number of iterations used in the computation.
	 */
	inline size_t n_Iteration_Num() const
	{
		return m_n_restart_num;
	}

	/**
	 *	@brief gets the number of matrix operations
	 *	@return Returns the number of matrix operations used in the computation.
	 */
	inline size_t n_MatrixOp_Num() const
	{
		return m_n_mat_op_num;
	}

	/**
	 *	@brief gets the converged eigenvalues
	 *
	 *	@return Returns a vector of eigenvalues.
	 *
	 *	@note There might be less eigenvalues than originally requested
	 *		(the number of converged eigenvalues was returned by \ref Compute()
	 *		and can be also obtained by calling \ref n_Converged_Num()).
	 *	@note This function throws std::bad_alloc.
	 */
	Eigen::VectorXd v_Eigenvalues() const; // throw(std::bad_alloc)

	/**
	 *	@brief gets the converged eigenvectors
	 *
	 *	@param[in] n_vector_num is number of eigenvectors to be returned
	 *		(up to the number of eigenvalues specified in the constructor)
	 *
	 *	@return Returns a matrix of eigenvectors (each eigenvector is a column in this matrix).
	 *
	 *	@note There might be less eigenvectors than originally requested
	 *		(the number of converged eigenvalues was returned by \ref Compute()
	 *		and can be also obtained by calling \ref n_Converged_Num()).
	 *	@note This function throws std::bad_alloc.
	 */
	Eigen::MatrixXd t_Eigenvectors(size_t n_vector_num = SIZE_MAX) const; // throw(std::bad_alloc)

protected:
	/**
	 *	@brief Arnoldi factorization starting from a given step
	 *
	 *	@param[in] n_start is zero-based index of the first ritz value to update
	 *	@param[in] n_end is zero-based index of one past the last ritz value to update
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	void Factorize(size_t n_start, size_t n_end); // throw(std::bad_alloc)

	/**
	 *	@brief implicitly restarted Arnoldi factorization
	 *	@param[in] n_start is zero-based index of the Ritz vector to resume at
	 *	@note This function throws std::bad_alloc and std::runtime_error.
	 */
	void Restart(size_t n_start); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief calculates the number of converged eigenvalues, sets the bit array
	 *	@param[in] f_tolerance is threshold on eigenvalue error
	 *	@return Returns the number of converged eigenvalues.
	 */
	size_t n_Find_Converged(double f_tolerance);

	/**
	 *	@brief calculates the number of adjusted eigenvalues
	 *	@param[in] n_converged_num is number of the eigenvalues converged so far
	 *	@return Returns the number of adjusted eigenvalues for restarting.
	 */
	size_t n_Adjusted_Eigenvalue_Num(size_t n_converged_num) const;

	/**
	 *	@brief retrieve and sort Ritz values and Ritz vectors
	 *	@note This function throws std::bad_alloc or std::runtime_error
	 *		if the tridiagonal eigenvalue solver fails.
	 */
	void Get_RitzPairs(); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief sorts the first <tt>n</tt> Ritz pairs by the specified sort order,
	 *		where <tt>n</tt> is the number of requested eigenvalues
	 *
	 *	@note This is only called once, to return the final results.
	 *	@note \ref CSymEigsShiftSolver inherits from this and using virtual is required
	 *		(alternately, Compute() would have to be overloaded as well).
	 *	@note This function throws std::bad_alloc.
	 */
	virtual void Sort_RitzPairs(); // throw(std::bad_alloc)
};

/**
 *	@brief eigenvalue solver for symmetric matrices in inverse / shift mode
 *
 *	@tparam COpType is matrix operation type (implements matrix inverse multiplication,
 *		possibly with shift; see e.g. \ref CSymmetricSparseMatrix_InvProduct for an example)
 *	@tparam _n_sort_type is eigenvalue sort type (one of eigen_By*, default \ref eigen_ByMagnitude)
 *	@tparam _b_ascending_sort is eigenvalue sort direction flag (if set - ascending, if cleared - descending)
 *
 *	@note This can be used to recover up to n - 1 eigenvalues and eigenvectors from
 *		a nxn symmetric matrix. To get the low eigenvalues, typically inverting the
 *		matrix and using \ref CSymEigsShiftSolver produces more accurate results
 *		than then using descending ordering (which fails to converge sometimes).
 */
template <typename COpType /*= CBlockMatrix_InvProduct*/,
	int n_sort_type = eigen_ByMagnitude, bool b_ascending_sort = false>
class CSymEigsShiftSolver: public CSymEigsSolver<COpType, n_sort_type, b_ascending_sort> {
public:
	typedef CSymEigsSolver<COpType, n_sort_type, b_ascending_sort> _TyBase; /**< @brief base type of eigensolver without shift */

public:
	/**
	 *	@brief default constructor; creates an uninitialized solver object
	 *
	 *	@param[in] r_op is reference to a matrix operation object (matrix inverse multiplication;
	 *		this object is only referenced and must not be deleted before calling \ref Compute())
	 *	@param[in] n_eigs_num is the number of requested eigenvalues (must be less than the number
	 *		of columns of the decomposed matrix)
	 *	@param[in] n_ritz_num is the number of Ritz vectors; must be larger than n_eigs_num and
	 *		smaller or equal to the number of columns of the decomposed matrix; higher values
	 *		tend to improve convergence)
	 *
	 *	@note This does not calculate anything; one needs to call Compute().
	 */
	inline CSymEigsShiftSolver(COpType &r_op, size_t n_eigs_num, size_t n_ritz_num)
		:_TyBase(r_op, n_eigs_num, n_ritz_num)
	{}

	/**
	 *	@brief gets eigenvalue shift
	 *	@return Returns the relative shift of the eigenvalues.
	 */
	inline double f_Shift() const
	{
		return this->m_r_op.f_Shift();
	}

protected:
	/**
	 *	@copydoc CSymEigsSolver::Sort_RitzPairs()
	 */
	virtual void Sort_RitzPairs()
	{
		double f_sigma = this->m_r_op.f_Shift();
		this->m_v_ritz_val.head(this->m_n_eigs_num) = 1.0 / this->m_v_ritz_val.head(this->m_n_eigs_num).array() + f_sigma;
		// First transform back the Ritz values, and then sort
		_TyBase::Sort_RitzPairs();
	}
};

#include "eigen/Eigen/Sparse"
//#include "slam/LinearSolver_Cholmod.h"

/**
 *	@brief helper object for calculation of eigenvalues of a sparse matrix using inverse / shift mode
 */
class CGenSparseMatrix_InvProduct {
protected:
	typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat;
	typedef Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int> > SpQRSolver;

	const size_t m, n;
	double m_f_shift;
	SpQRSolver solver;

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *
	 *	@param[in] p_matrix is const pointer to the matrix to be solved for
	 *	@param[in] f_shift is shift parameter to be applied to the diagonal of the matrix
	 *
	 *	@note This calculates QR decomposition of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	CGenSparseMatrix_InvProduct(const cs *p_matrix, double f_shift); // throw(std::bad_alloc)

	/**
	 *	@brief gets shift
	 *	@return Returns the value of shift.
	 */
	double f_Shift() const
	{
		return m_f_shift;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return n;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return n;
	}

	/**
	 *	@brief calculates inverse matrix product with shift \f$y = (A - shift * I)^{-1} * x\f$
	 *
	 *	@tparam Derived0 is type of the vector to be multiplied
	 *	@tparam Derived1 is type of the vector to store the result
	 *
	 *	@param[in] r_v_x is reference to the vector to be solved for
	 *	@param[in] r_v_y is reference to the vector to store the result
	 *
	 *	@note The two arguments must not point to the same vector.
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Eigen::MatrixBase<Derived0> &r_v_x, Eigen::MatrixBase<Derived1> &r_v_y) const
	{
		_ASSERTE((const void*)&r_v_x != (const void*)&r_v_y); // otherwise can't use .noalias()
		r_v_y.noalias() = solver.solve(r_v_x);
	}

private:
	CGenSparseMatrix_InvProduct(const CGenSparseMatrix_InvProduct &r_other); // no-copy
	CGenSparseMatrix_InvProduct &operator =(const CGenSparseMatrix_InvProduct &r_other); // no-copy
};

/**
 *	@brief helper object for calculation of eigenvalues of a sparse matrix using inverse / shift mode
 */
class CSymmetricSparseMatrix_InvProduct {
protected:
	typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat;
	typedef Eigen::SimplicialLDLT<SpMat> SpLDLTSolver;

	const size_t n;
	const double m_f_shift;
	SpLDLTSolver solver;

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *
	 *	@param[in] p_matrix is const pointer to the matrix to be solved for (must be symmetric)
	 *	@param[in] f_shift is shift parameter to be applied to the diagonal of the matrix
	 *
	 *	@note This calculates LDLT decomposition of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	CSymmetricSparseMatrix_InvProduct(const cs *p_matrix, double f_shift); // throw(std::bad_alloc)

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::f_Shift()
	 */
	double f_Shift() const
	{
		return m_f_shift;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return n;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return n;
	}

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::operator()
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Eigen::MatrixBase<Derived0> &r_v_x, Eigen::MatrixBase<Derived1> &r_v_y) const
	{
		_ASSERTE((const void*)&r_v_x != (const void*)&r_v_y); // otherwise can't use .noalias()
		r_v_y.noalias() = solver.solve(r_v_x);
	}

private:
	CSymmetricSparseMatrix_InvProduct(const CSymmetricSparseMatrix_InvProduct &r_other); // no-copy
	CSymmetricSparseMatrix_InvProduct &operator =(const CSymmetricSparseMatrix_InvProduct &r_other); // no-copy
};

/**
 *	@brief helper object for calculation of eigenvalues of a sparse matrix using inverse / shift mode
 */
class CSquareSparseMatrix_InvProduct {
protected:
	typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat;
#if 0
	typedef Eigen::SparseLU<SpMat, Eigen::COLAMDOrdering<int> > SpLUSolver;
#else // 0
	typedef Eigen::SparseLU<SpMat, Eigen::AMDOrdering<int> > SpLUSolver;
#endif // 0

	const size_t n;
	double m_f_shift;
	SpLUSolver solver;

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *
	 *	@param[in] p_matrix is const pointer to the matrix to be solved for (must be square)
	 *	@param[in] f_shift is shift parameter to be applied to the diagonal of the matrix
	 *
	 *	@note This calculates LU decomposition of the matrix (using Eigen).
	 *	@note This function throws std::bad_alloc.
	 */
	CSquareSparseMatrix_InvProduct(const cs *p_matrix, double f_shift); // throw(std::bad_alloc)

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::f_Shift()
	 */
	double f_Shift() const
	{
		return m_f_shift;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return n;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return n;
	}

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::operator()
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Eigen::MatrixBase<Derived0> &r_v_x, Eigen::MatrixBase<Derived1> &r_v_y) const
	{
		_ASSERTE((const void*)&r_v_x != (const void*)&r_v_y); // otherwise can't use .noalias()
		r_v_y.noalias() = solver.solve(r_v_x);
	}

private:
	CSquareSparseMatrix_InvProduct(const CSquareSparseMatrix_InvProduct &r_other); // no-copy
	CSquareSparseMatrix_InvProduct &operator =(const CSquareSparseMatrix_InvProduct &r_other); // no-copy
};

/**
 *	@brief helper object for calculation of eigenvalues of a sparse matrix using inverse / shift mode
 */
class CSquareSparseMatrix_InvProduct2 {
	css *m_p_symbolic_decomposition;
	csn *m_p_factor;
	mutable std::vector<double> m_temp;

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *
	 *	@param[in] p_matrix is const pointer to the matrix to be solved for (must be square)
	 *	@param[in] f_shift is shift parameter to be applied to the diagonal of the matrix (must be zero)
	 *
	 *	@note This calculates LU decomposition of the matrix (using CSparse).
	 *	@note This function throws std::bad_alloc.
	 */
	CSquareSparseMatrix_InvProduct2(const cs *p_matrix, double UNUSED(f_shift)); // throw(std::bad_alloc)

	/**
	 *	@brief destructor; frees up the decomposition of the matrix
	 */
	~CSquareSparseMatrix_InvProduct2();

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::f_Shift()
	 */
	double f_Shift() const
	{
		return 0;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return m_p_factor->L->m;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return m_p_factor->L->n;
	}

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::operator()
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Derived0 &r_v_x, Derived1 &r_v_y) const
	{
		csi n_col_num = m_p_factor->L->n;
		cs_ipvec(m_p_symbolic_decomposition->pinv, r_v_x.data(), &m_temp[0], n_col_num); // temp = P*x
		cs_lsolve(m_p_factor->L, &m_temp[0]); // x = L\x
		cs_usolve(m_p_factor->U, &m_temp[0]); // x = U\x
		cs_pvec(m_p_symbolic_decomposition->pinv, &m_temp[0], r_v_y.data(), n_col_num); // y = P'*temp
	}

private:
	CSquareSparseMatrix_InvProduct2(const CSquareSparseMatrix_InvProduct2 &r_other); // no-copy
	CSquareSparseMatrix_InvProduct2 &operator =(const CSquareSparseMatrix_InvProduct2 &r_other); // no-copy
};

/**
 *	@brief helper object for calculation of eigenvalues of a block matrix using inverse / shift mode
 */
class CPosDefBlockMatrix_InvProduct {
protected:
	CUberBlockMatrix m_R; /**< @brief factorized matrix */
	std::vector<size_t> m_inv_order; /**< @brief inverse ordering */
	mutable std::vector<double> m_temp; /**< @brief temporary to hold the permuted vector */

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *
	 *	@param[in] p_matrix is const pointer to the matrix to be solved for (must be positive definite)
	 *	@param[in] f_shift is shift parameter to be applied to the diagonal of the matrix (must be zero)
	 *
	 *	@note This calculates Cholesky decomposition of the matrix.
	 *	@note This function throws std::bad_alloc or std::runtime_error
	 *		if the matrix is not positive definite.
	 */
	CPosDefBlockMatrix_InvProduct(const CUberBlockMatrix &r_matrix, double UNUSED(f_shift)); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::f_Shift()
	 */
	double f_Shift() const
	{
		return 0;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return m_R.n_Row_Num();
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return m_R.n_Column_Num();
	}

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::operator()
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Derived0 &r_v_x, Derived1 &r_v_y) const
	{
		const size_t n_elem_num = m_inv_order.size();
		_ASSERTE(m_temp.size() == n_elem_num &&
			r_v_x.rows() == n_elem_num && r_v_y.rows() == n_elem_num);
		double *p_temp = &m_temp.front();
		m_R.InversePermute_LeftHandSide_Vector(p_temp, &r_v_x(0),
			r_v_x.rows(), &m_inv_order.front(), n_elem_num); // temp = iperm(r_v_x)
		m_R.UpperTriangularTranspose_Solve(p_temp, n_elem_num);
		m_R.UpperTriangular_Solve(p_temp, n_elem_num);
		m_R.Permute_LeftHandSide_Vector(&r_v_y(0), p_temp,
			r_v_y.rows(), &m_inv_order.front(), n_elem_num); // r_v_y = perm(temp)
		// ipvec, utsolve, usolve, pvec
	}
};

/**
 *	@brief helper object for calculation of eigenvalues of a sparse matrix using inverse / shift mode
 */
class CPosDefSparseMatrix_InvProduct {
protected:
	css *m_p_symbolic_decomposition;
	csn *m_p_factor;
	mutable std::vector<double> m_temp;

public:
	/**
	 *	@brief default constructor; gets the matrix for multiplication
	 *
	 *	@param[in] p_matrix is const pointer to the matrix to be solved for (must be positive definite)
	 *	@param[in] f_shift is shift parameter to be applied to the diagonal of the matrix (must be zero)
	 *
	 *	@note This calculates Cholesky decomposition of the matrix.
	 *	@note This function throws std::bad_alloc.
	 */
	CPosDefSparseMatrix_InvProduct(const cs *p_matrix, double UNUSED(f_shift)); // throw(std::bad_alloc)

	/**
	 *	@brief destructor; frees up the decomposition of the matrix
	 */
	~CPosDefSparseMatrix_InvProduct();

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::f_Shift()
	 */
	double f_Shift() const
	{
		return 0;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Row_Num()
	 */
	size_t n_Row_Num() const
	{
		return m_p_factor->L->m;
	}

	/**
	 *	@copydoc CBlockMatrixProduct::n_Column_Num()
	 */
	size_t n_Column_Num() const
	{
		return m_p_factor->L->n;
	}

	/**
	 *	@copydoc CGenSparseMatrix_InvProduct::operator()
	 */
	template <class Derived0, class Derived1>
	void operator ()(const Derived0 &r_v_x, Derived1 &r_v_y) const
	{
		csi n_col_num = m_p_factor->L->n;
		cs_ipvec(m_p_symbolic_decomposition->pinv, r_v_x.data(), &m_temp[0], n_col_num); // temp = P*x
		cs_lsolve(m_p_factor->L, &m_temp[0]); // x = L\x
		cs_ltsolve(m_p_factor->L, &m_temp[0]); // x = L'\x
		cs_pvec(m_p_symbolic_decomposition->pinv, &m_temp[0], r_v_y.data(), n_col_num); // y = P'*temp
	}

private:
	CPosDefSparseMatrix_InvProduct(const CPosDefSparseMatrix_InvProduct &r_other); // no-copy
	CPosDefSparseMatrix_InvProduct &operator =(const CPosDefSparseMatrix_InvProduct &r_other); // no-copy
};

/**
 *	@brief namespace with eigenvalue computation helpers
 */
namespace eigenvalues_detail {

struct TGetMagnitude {
	template <class T>
	T operator ()(T x) const
	{
		return fabs(x);
	}

	template <class T>
	T operator ()(const std::complex<T> &x) const
	{
		return x.mag();
	}
};

struct TGetAlgebraic {
	template <class T>
	T operator ()(T x) const
	{
		return x;
	}
};

struct TGetReal {
	template <class T>
	T operator ()(T x) const
	{
		return x;
	}

	template <class T>
	T operator ()(const std::complex<T> &x) const
	{
		return x.real();
	}
};

struct TGetImag {
	template <class T>
	T operator ()(const std::complex<T> &x) const
	{
		return x.imag();
	}
};

template <int n_sort_type, bool b_ascending_sort>
class CSortEigenvalues {
public:
	typedef MakeTypelist(TGetMagnitude, TGetAlgebraic, TGetReal, TGetImag) SortTypes;
	typedef typename CTypelistItemAt<SortTypes, n_sort_type>::_TyResult _TySortModifier;

public:
	template <class T>
	static void Sort(std::vector<size_t> &r_sorted_sequence,
		const T *p_eigenvalues, size_t n_eigenvalue_num); // throw(std::bad_alloc)
};

/**
 *	@brief QR decomposition of an upper Hessenberg matrix
 *	@tparam CScalar is scalar type (default double)
 */
template <class CScalar = double>
class CUpperHessenbergQR {
protected:
	typedef CScalar _TyScalar; /**< @brief scalar type */
	typedef Eigen::Matrix<_TyScalar, Eigen::Dynamic, Eigen::Dynamic> _TyMatrix; /**< @brief general matrix type */
	typedef Eigen::Matrix<_TyScalar, 1, 2> _TyVector2; /**< @brief 2D vector type */
	typedef Eigen::Matrix<_TyScalar, 2, 2> _TyMatrix2; /**< @brief 2x2 matrix type */
	typedef Eigen::Matrix<_TyScalar, 2, Eigen::Dynamic>  _TyMatrix2X; /**< @brief 2xX matrix type */
	typedef Eigen::Matrix<_TyScalar, Eigen::Dynamic, 1> _TyVector; /**< @brief general vector type */

protected:
	size_t m_n; /**< @brief size of the matrix */
	_TyMatrix m_t_R; /**< @brief the tridiagonal R matrix */
	_TyMatrix2X m_t_rot; /**< @brief stacked Givens rotations matrices as pairs of (sin(phi_i), cos(phi_i)) */

	// G_i = [ cos[phi_i]  sin[phi_i]]
	//       [-sin[phi_i]  cos[phi_i]]
	// Q = G_1 * G_2 * ... * G_{m_n-1}

public:
	/**
	 *	@brief default constructor; has no effect
	 *	@note To compute the decomposition, call Compute().
	 */
	inline CUpperHessenbergQR()
		:m_n(0)
	{}

	/**
	 *	@brief constructor; calculates the decomposition
	 *	@param[in] r_A is square matrix to be decomposed, at least 2x2
	 *		(only the upper triangular part is accessed)
	 *	@note This function throws std::bad_alloc.
	 */
	inline CUpperHessenbergQR(const Eigen::Ref<const _TyMatrix> &r_A)
		:m_n(0)
	{
		Compute(r_A);
	}

	/**
	 *	@brief calculates the decomposition
	 *	@param[in] r_A is square matrix to be decomposed, at least 2x2
	 *		(only the upper triangular part is accessed)
	 *	@note This function throws std::bad_alloc.
	 */
	void Compute(const Eigen::Ref<const _TyMatrix> &r_A); // throw(std::bad_alloc)

	/**
	 *	@brief gets the R matrix
	 *	@return Returns a const reference to the R matrix in the QR
	 *		decomposition, which is an upper triangular matrix.
	 */
	inline const _TyMatrix &matrix_R() const
	{
		_ASSERTE(m_n); // if this triggers, need to call Compute() first
		return m_t_R;
	}

	/**
	 *	@brief calculates the RQ matrix
	 *	@param[out] RQ is filled by the product of R and Q, which is an upper Hessenberg matrix
	 *	@note This function throws std::bad_alloc.
	 */
	void Get_RQ(Eigen::Ref<_TyMatrix> RQ) const; // throw(std::bad_alloc)

	/**
	 *	@brief calculates the Q matrix from a series of Givens rotations
	 *	@param[out] Q is filled by the Q matrix, which is an upper triangular matrix
	 *	@note This function throws std::bad_alloc.
	 */
	void Get_Q(Eigen::Ref<_TyMatrix> Q) const; // throw(std::bad_alloc)

	/**
	 *	@brief post-multiplies a matrix Y by Q
	 *	@param[in,out] Y is the matrix to be multiplied by Q from the right
	 */
	void Apply_YQ(Eigen::Ref<_TyMatrix> Y) const;

	/**
	 *	@brief post-multiplies a matrix Y by Q^T
	 *	@param[in,out] Y is the matrix to be multiplied by the transpose of Q from the right
	 *	@note This function throws std::bad_alloc.
	 */
	void Apply_YQt(Eigen::Ref<_TyMatrix> Y) const; // throw(std::bad_alloc) // unused/untested

	/**
	 *	@brief pre-multiplies a vector Y by Q
	 *	@param[in,out] Y is the vector to be multiplied by Q from the left
	 */
	void Apply_QY(_TyVector &Y) const; // unused/untested

	/**
	 *	@brief pre-multiplies a matrix Y by Q
	 *	@param[in,out] Y is the matrix to be multiplied by Q from the left
	 *	@note This function throws std::bad_alloc.
	 */
	void Apply_QY(Eigen::Ref<_TyMatrix> Y) const; // throw(std::bad_alloc) // unused/untested

	/**
	 *	@brief pre-multiplies a vector Y by Q^T
	 *	@param[in,out] Y is the vector to be multiplied by the transpose of Q from the left
	 */
	void Apply_QtY(_TyVector &Y) const; // unused/untested

	/**
	 *	@brief pre-multiplies a matrix Y by Q^T
	 *	@param[in,out] Y is the matrix to be multiplied by the transpose of Q from the left
	 *	@note This function throws std::bad_alloc.
	 */
	void Apply_QtY(Eigen::Ref<_TyMatrix> Y) const; // throw(std::bad_alloc) // unused/untested
};

/**
 *	@brief QR decomposition of a tridiagonal matrix (a special case of upper-Hessenberg QR)
 *	@tparam CScalar is scalar type (default double)
 *	@note It would be better to store the tridiagonal matrix only as 2xn matrix instead on nxn. Could optimize more.
 */
template <typename CScalar = double>
class CTridiagQR {
protected:
	typedef CScalar _TyScalar; /**< @brief scalar type */
	typedef Eigen::Matrix<_TyScalar, Eigen::Dynamic, Eigen::Dynamic> _TyMatrix; /**< @brief general matrix type */
	typedef Eigen::Matrix<_TyScalar, 1, 2> _TyVector2; /**< @brief 2D vector type */
	typedef Eigen::Matrix<_TyScalar, 2, 2> _TyMatrix2; /**< @brief 2x2 matrix type */
	typedef Eigen::Matrix<_TyScalar, 2, Eigen::Dynamic>  _TyMatrix2X; /**< @brief 2xX matrix type */
	typedef Eigen::Matrix<_TyScalar, Eigen::Dynamic, 1> _TyVector; /**< @brief general vector type */

protected:
	size_t m_n; /**< @brief size of the matrix */
	_TyMatrix m_t_R; /**< @brief the tridiagonal R matrix */
	_TyMatrix2X m_t_rot; /**< @brief stacked Givens rotations matrices as pairs of (sin(phi_i), cos(phi_i)) */

	// G_i = [ cos[phi_i]  sin[phi_i]]
	//       [-sin[phi_i]  cos[phi_i]]
	// Q = G_1 * G_2 * ... * G_{m_n-1}

public:
	/**
	 *	@brief default constructor; has no effect
	 *	@note To compute the decomposition, call Compute().
	 */
	inline CTridiagQR()
		:m_n(0)
	{}

	/**
	 *	@brief constructor; calculates the decomposition
	 *	@param[in] r_A is square matrix to be decomposed, at least 2x2 (only
	 *		the major diagonal and one lower minor diagonal are accessed)
	 *	@note This function throws std::bad_alloc.
	 */
	inline CTridiagQR(const Eigen::Ref<const _TyMatrix> &r_A) // throw(std::bad_alloc)
		:m_n(0)
	{
		Compute(r_A);
	}

	/**
	 *	@brief calculates the decomposition
	 *	@param[in] r_A is square matrix to be decomposed, at least 2x2 (only
	 *		the major diagonal and one lower minor diagonal are accessed)
	 *	@note This function throws std::bad_alloc.
	 */
	void Compute(const Eigen::Ref<const _TyMatrix> &r_A); // throw(std::bad_alloc)

	/**
	 *	@brief gets the R matrix
	 *	@return Returns a const reference to the R matrix in the QR
	 *		decomposition, which is an upper triangular matrix.
	 */
	inline const _TyMatrix &matrix_R() const
	{
		_ASSERTE(m_n); // if this triggers, need to call Compute() first
		return m_t_R;
	}

	/**
	 *	@brief calculates the RQ matrix
	 *	@param[out] RQ is filled by the product of R and Q, which is a tridiagonal matrix
	 *	@note This function throws std::bad_alloc.
	 */
	void Get_RQ(Eigen::Ref<_TyMatrix> RQ) const; // throw(std::bad_alloc)

	/**
	 *	@brief calculates the Q matrix from a series of Givens rotations
	 *	@param[out] Q is filled by the Q matrix, which is an upper triangular matrix
	 *	@note This function throws std::bad_alloc.
	 */
	void Get_Q(Eigen::Ref<_TyMatrix> Q) const; // throw(std::bad_alloc)

	/**
	 *	@brief post-multiplies a matrix Y by Q
	 *	@param[in,out] Y is the matrix to be multiplied by Q from the right
	 */
	void Apply_YQ(Eigen::Ref<_TyMatrix> Y) const;
};

/**
 *	@brief eigenvalue decomposition of a tridiagonal matrix
 *	@tparam _TyScalar is scalar type (default double)
 */
template <typename _TyScalar = double>
class CTridiagEigsSolver {
public:
	typedef Eigen::Matrix<_TyScalar, Eigen::Dynamic, Eigen::Dynamic> _TyMatrix; /**< @brief general matrix type */
	typedef Eigen::Matrix<_TyScalar, 1, Eigen::Dynamic> _TyVector; /**< @brief general vector type */

protected:
	size_t m_n; /**< @brief size of the matrix */
	_TyVector m_v_eval; /**< @brief main diagonal elements of the matrix */
	_TyMatrix m_v_evec; /**< @brief eigenvectors */

public:
	/**
	 *	@brief default constructor; has no effect
	 *	@note To compute the decomposition, call Compute().
	 */
	inline CTridiagEigsSolver()
		:m_n(0)
	{}

	/**
	 *	@brief constructor; calculates the decomposition
	 *	@param[in] r_A is square matrix to be decomposed, at least 2x2 (only
	 *		the major diagonal and one lower minor diagonal are accessed)
	 *	@note This function throws std::bad_alloc and std::runtime_error (if not converged).
	 */
	inline CTridiagEigsSolver(const Eigen::Ref<const _TyMatrix> &r_A) // throw(std::bad_alloc, std::runtime_error)
		:m_n(0)
	{
		Compute(r_A);
	}

	/**
	 *	@brief calculates the decomposition
	 *	@param[in] r_A is square matrix to be decomposed, at least 2x2 (only
	 *		the major diagonal and one lower minor diagonal are accessed)
	 *	@note This function throws std::bad_alloc and std::runtime_error (if not converged).
	 */
	void Compute(const Eigen::Ref<const _TyMatrix> &r_A); // throw(std::bad_alloc, std::runtime_error)

	/**
	 *	@brief gets the eigenvalues
	 *	@return Returns a const reference to the vector of eigenvalues.
	 */
	inline const _TyVector &v_Eigenvalues() const
	{
		_ASSERTE(m_n); // need to call Compute() first
		return m_v_eval;
	}

	/**
	 *	@brief gets the eigenvectors
	 *	@return Returns a const reference to the matrix of eigenvectors.
	 */
	inline const _TyMatrix &t_Eigenvectors() const
	{
		_ASSERTE(m_n); // need to call Compute() first
		return m_v_evec;
	}
};

} // ~eigenvalues_detail

#include "slam/Eigenvalues.inl"

#endif // !__EIGENVALUES_INCLUDED
