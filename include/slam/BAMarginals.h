/*
								+----------------------------------+
								|                                  |
								|      ***  BA Marginals  ***      |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2016  |
								|                                  |
								|          BAMarginals.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __BA_MARGINALS_INCLUDED
#define __BA_MARGINALS_INCLUDED

/**
 *	@file include/slam/BAMarginals.h
 *	@brief helper classes for covariance recovery in Schur-complemented systems
 *	@author -tHE SWINe-
 *	@date 2016-02-15
 */

#include "slam/Marginals.h"
#include "slam/MemUsage.h" // PRIsizeB

/**
 *	@def __BA_MARGS_DO_MEMORY_PROFILING
 *	@brief if defined, memory use of the marginals will be profiled
 */
//#define __BA_MARGS_DO_MEMORY_PROFILING

/**
 *	@brief internal functions for Schur complement marginals
 */
namespace sc_margs_detail {

// todo - use wrap3 below

/**
 *	@brief multiply add operation with a sparse matrix and a block vector
 */
class CBlockVectorMAD_Impl {
public:
	struct TInnerContext {
		double *p_dest;
		const double *p_block;

		inline TInnerContext(double *_p_dest, const double *_p_block)
			:p_dest(_p_dest), p_block(_p_block)
		{}
	};

	template <const int n_row_height, class CColumnWidth>
	class CInnerLoop { // not dependent on CBlockSizeList; don't make it an inner class of COuterLoop
	public:
		enum {
			n_column_width = CColumnWidth::n_size
		};

		typedef typename CUberBlockMatrix::CMakeMatrixRef<n_column_width, n_column_width>::_Ty _TyDestMap;
		typedef typename CUberBlockMatrix::CMakeMatrixRef<n_row_height, n_column_width>::_TyConst _TyBlockMap;

	public:
		static inline void Do(TInnerContext t_ctx)
		{
			_TyBlockMap fbs_block(t_ctx.p_block);
			_TyDestMap(t_ctx.p_dest) += fbs_block.transpose().lazyProduct(fbs_block); // mad
		}
	};

	struct TOuterContext {
		double *p_dest;
		const CUberBlockMatrix &r_block_vector;

		inline TOuterContext(double *_p_dest, const CUberBlockMatrix &_r_block_vector)
			:p_dest(_p_dest), r_block_vector(_r_block_vector)
		{}
	};

	template <const int n_column_width, class CBlockSizeList>
	class COuterLoop {
	public:
		static inline void Do(TOuterContext t_ctx)
		{
			_ASSERTE(n_column_width == t_ctx.r_block_vector.n_BlockColumn_Column_Num(0));
			for(size_t i = 0, n = t_ctx.r_block_vector.n_BlockColumn_Block_Num(0); i < n; ++ i) {
				CUberBlockMatrix::_TyMatrixXdRef t_block =
					const_cast<CUberBlockMatrix&>(t_ctx.r_block_vector).t_Block_AtColumn(0, i);
				// hack - need to cast, const_ref does not expose its pointer to data

				size_t n_row_height = t_block.rows();
				_ASSERTE(n_column_width == t_block.cols());

				fbs_ut::CWrap2<CInnerLoop, fbs_ut::CCTSize<n_column_width> >::template
					In_RowHeight_DecisionTree_Given_ColumnWidth<CBlockSizeList,
					n_column_width>(int(n_row_height), TInnerContext(t_ctx.p_dest, t_block.data()/*&t_block(0, 0)*/));
				// mad
			}
			// for each nnz block
		}
	};

/*public:
	template <class CBlockSizeList>
	static void BlockVector_PreMultiplyWithSelfTranspose_Add_FBS(double *p_dest,
		const CUberBlockMatrix &r_block_vector) // g++ is unable to reference TOuterLoop from the outside for some reason
	{
		fbs_ut::CWrap2<COuterLoop, CBlockSizeList>::template In_ColumnWidth_DecisionTree<CBlockSizeList>(
			r_block_vector.n_Column_Num(), TOuterContext(p_dest, r_block_vector));
		// decide over vector width
	}*/
};

template <class CBlockSizeList, class CDestMatrix>
inline void BlockVector_PreMultiplyWithSelfTranspose_Add_FBS(CDestMatrix &r_dest,
	const CUberBlockMatrix &r_block_vector)
{
	_ASSERTE(r_block_vector.n_BlockColumn_Num() == 1); // it is a block column-vector
	_ASSERTE(r_block_vector.n_Column_Num() == r_dest.rows()); // the number of columns in the block vector matches the size of the destination
	_ASSERTE(r_dest.rows() == r_dest.cols()); // the output is square
	_ASSERTE(!(CDestMatrix::Flags & Eigen::RowMajor)); // must be column-major otherwise the conversion to pointer strips data
	//_ASSERTE(CDestMatrix::MaxColsAtCompileTime == Eigen::Dynamic ||
	//	CDestMatrix::MaxColsAtCompileTime == r_dest.cols() ||
	//	r_dest.cols() == 1); // the stride must be tight or there is a single col and it does not matter
	_ASSERTE(r_dest.cols() <= 1 || &r_dest(0, 1) == &r_dest(0, 0) + r_dest.rows()); // the stride must be tight or there is a single col and it does not matter

	// do not zero r_dest

	//CBlockVectorMAD_Impl::template BlockVector_PreMultiplyWithSelfTranspose_Add_FBS<CBlockSizeList>(r_dest.data(), r_block_vector);

	fbs_ut::CWrap2<CBlockVectorMAD_Impl::COuterLoop, CBlockSizeList>::template
		In_ColumnWidth_DecisionTree<CBlockSizeList>(int(r_block_vector.n_Column_Num()),
		CBlockVectorMAD_Impl::TOuterContext(r_dest.data(), r_block_vector));
	// decide over vector width
}

inline void Calculate_UpperTriangularTransposeSolve_Bases(CUberBlockMatrix &S_bases,
	const CUberBlockMatrix &S, /*const*/ cs *p_St, size_t n_column,
	Eigen::MatrixXd &r_workspace, std::vector<size_t> &r_workspace1) // this is inline, to avoid link conflicts
{
	_ASSERTE(S.b_EqualLayout(S_bases)); // will yield a matrix with the same sparsity structure
	//S.CopyLayoutTo(S_bases);

	_ASSERTE(n_column < S.n_BlockColumn_Num()); // make sure the column is inside the matrix

	_ASSERTE(p_St->m == S.n_BlockRow_Num() && p_St->n == S.n_BlockColumn_Num()); // should be the same matrix

	_ASSERTE(sizeof(csi) == sizeof(size_t));
	r_workspace1.resize(2 * p_St->n);
	// alloc workspace

	{
		//cs *p_St = cs_transpose(p_S, 0);
		// need p_St

		/*csi p_col[2] = {0, 1};
		csi n_row = 0;
		cs B;
		B.m = p_St->m;
		B.n = 1;
		B.p = p_col;
		B.i = &n_row;
		B.x = 0;
		B.nzmax = 1;
		B.nz = -1;*/
		// prepare a single entry CSC matrix

		//Eigen::Matrix<double, Eigen::Dynamic, 6> S_dense_basis(S.n_Row_Num(), 6); // todo - implement matrix solving and try to make this row major
		// unit basis matrix

		//for(size_t n_column = 0, n = S.n_BlockColumn_Num(); n_column < n; ++ n_column) { // t_odo - do this in parallel (probably explicit matrix reduction rather than locking)
		{ //size_t n_column = n_column;
			size_t w = S.n_BlockColumn_Column_Num(n_column); // t_odo - FBS it
			//_ASSERTE(w == 6);
			//n_row = n_column;

			//size_t n_first_dep_col = cs_reach(p_St, &B, 0, (csi*)&r_workspace1.front(), 0); // modifies p_St but then puts it back
			size_t n_first_dep_col = cs_dfs(n_column, p_St, p_St->n, (csi*)&r_workspace1.front(),
				(csi*)&r_workspace1.front() + p_St->n, 0); // no need for B // todo - reimplement this directly on block matrices, try to avoid needing the transpose
			size_t n_dep_col_num = p_St->n - n_first_dep_col;
			const size_t *p_dep_col = &r_workspace1[n_first_dep_col];
			for(size_t j = 0; j < n_dep_col_num; ++ j)
				CS_MARK(p_St->p, p_dep_col[j]); // restore col. pointers after calling cs_dfs
			// get the list of columns of S that affect block U_Dinv_{n_column, *}

			r_workspace.resize(S.n_Row_Num(), w);
			// alloc workspace

			Eigen::MatrixXd &S_dense_basis = r_workspace; // just rename
			//Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Aligned>
			//	S_dense_basis(r_workspace.data(), S.n_Row_Num(), w);
			// map the workspace as (column-wise) fixed-size matrix

			S_dense_basis.setZero();
			S_dense_basis.middleRows(S_bases.n_BlockColumn_Base(n_column), w).setIdentity();
			// create a vector of zeros

			for(size_t c = 0; c < w; ++ c) {
				//S.UpperTriangularTranspose_Solve(&U_Dinv_i_permd.col(c)(0), U_Dinv_i_permd.rows(), p_dep_col, n_dep_col_num);
				S.UpperTriangularTranspose_Solve/*_FBS<SC_BlockSizes>*/(&S_dense_basis.col(c)(0),
					S_dense_basis.rows(), p_dep_col, n_dep_col_num);
			}
			// sparse sparse UTTSolve

			for(size_t j = 0; j < n_dep_col_num; ++ j) {
				size_t n_row = p_dep_col[j];
				size_t y = S_bases.n_BlockColumn_Base(n_row);
				size_t h = S_bases.n_BlockColumn_Column_Num(n_row); // those are rows but S_bases is symmetric
#ifdef _DEBUG
				if(j > 0) {
					const size_t r_prev = p_dep_col[j - 1];
					size_t y_prev = S_bases.n_BlockColumn_Base(r_prev);
					size_t h_prev = S_bases.n_BlockColumn_Column_Num(r_prev);
					size_t e_prev = y_prev + h_prev;
					_ASSERTE(S_dense_basis.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
				} else if(y > 0)
					_ASSERTE(S_dense_basis.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
				if(j + 1 == n_dep_col_num)
					_ASSERTE(S_dense_basis.bottomRows(S_dense_basis.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
				// make sure that there are only zeros in between the elements
#endif // _DEBUG
				//_ASSERTE(h == 6);
				//_ASSERTE(S_bases.n_BlockColumn_Column_Num(n_row) == 6); // t_odo - FBS it
				S_bases.t_GetBlock_Log(n_row, n_column, h, w) =
					S_dense_basis.middleRows(S_bases.n_BlockColumn_Base(n_row), h); // this is transposed (transpose the block as well?): each row is a single basis; this only works if the structure of S is symmetric
			}
			// sparse fill the bases matrix
		}

		//S_bases.Rasterize("S_bases.tga", 3); // ...
	}
}

/**
 *	@brief FBS implementation for the upper triangular transpose solve of the sparse bases matrix
 */
class CUTTSolve_Bases_Impl {
public:
	struct TInnerContext {
		CUberBlockMatrix &r_dest;
		const size_t n_column;
		const Eigen::MatrixXd &r_src;
		const size_t n_row;

		inline TInnerContext(CUberBlockMatrix &_r_dest, size_t _n_column,
			const Eigen::MatrixXd &_r_src, size_t _n_row)
			:r_dest(_r_dest), n_column(_n_column), r_src(_r_src),
			n_row(_n_row)
		{}
	};

	template <const int n_row_height, class CColumnWidth>
	class CInnerLoop {
	public:
		enum {
			n_column_width = CColumnWidth::n_size
		};

	public:
		static inline void Do(TInnerContext t_ctx)
		{
			_ASSERTE(t_ctx.r_src.rows() == t_ctx.r_dest.n_Row_Num());
			_ASSERTE(n_column_width == t_ctx.r_dest.n_BlockColumn_Column_Num(t_ctx.n_column));

			Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, n_column_width>, Eigen::Aligned>
				S_dense_basis(t_ctx.r_src.data(), t_ctx.r_src.rows(), n_column_width);
			// map the source as (column-wise) fixed-size matrix

			Eigen::Map<Eigen::Matrix<double, n_row_height,
				n_column_width> > dest_block(t_ctx.r_dest.p_GetBlock_Log(t_ctx.n_row,
				t_ctx.n_column, n_row_height, n_column_width, true, false));
			dest_block = S_dense_basis.template middleRows<n_row_height>(t_ctx.r_dest.n_BlockColumn_Base(t_ctx.n_row));
		}
	};

	struct TOuterContext {
		CUberBlockMatrix &r_S_bases;
		const size_t n_column;
		const CUberBlockMatrix &r_S;
		Eigen::MatrixXd &r_workspace;
		const size_t *p_dep_col;
		const size_t n_dep_num;

		inline TOuterContext(CUberBlockMatrix &_r_S_bases, size_t _n_column,
			const CUberBlockMatrix &_r_S, Eigen::MatrixXd &_r_workspace,
			const size_t *_p_dep_col, size_t _n_dep_num)
			:r_S_bases(_r_S_bases), n_column(_n_column), r_S(_r_S),
			r_workspace(_r_workspace), p_dep_col(_p_dep_col), n_dep_num(_n_dep_num)
		{}
	};

	template <const int n_column_width, class CBlockSizeList>
	class COuterLoop {
	public:
		static inline void Do(TOuterContext t_ctx)
		{
			t_ctx.r_workspace.resize(t_ctx.r_S.n_Row_Num(), n_column_width);
			// alloc workspace

			Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, n_column_width>, Eigen::Aligned>
				S_dense_basis(t_ctx.r_workspace.data(), t_ctx.r_S.n_Row_Num(), n_column_width);
			// map the workspace as (column-wise) fixed-size matrix

			S_dense_basis.setZero();
			S_dense_basis.template middleRows<n_column_width>(t_ctx.r_S.n_BlockColumn_Base(t_ctx.n_column)).setIdentity();
			// create a vector of zeros

			for(size_t c = 0; c < n_column_width; ++ c) { // todo - make a version of UpperTriangularTranspose_Solve_FBS for vectors
				t_ctx.r_S.UpperTriangularTranspose_Solve_FBS<CBlockSizeList>(&S_dense_basis.col(c)(0),
					S_dense_basis.rows(), t_ctx.p_dep_col, t_ctx.n_dep_num);
			}
			// sparse sparse UTTSolve

			for(size_t j = 0; j < t_ctx.n_dep_num; ++ j) {
				size_t n_row = t_ctx.p_dep_col[j];
				size_t h = t_ctx.r_S.n_BlockColumn_Column_Num(n_row); // those are rows but S_bases is symmetric
#ifdef _DEBUG
				size_t y = t_ctx.r_S.n_BlockColumn_Base(n_row);
				if(j > 0) {
					const size_t r_prev = t_ctx.p_dep_col[j - 1];
					size_t y_prev = t_ctx.r_S.n_BlockColumn_Base(r_prev);
					size_t h_prev = t_ctx.r_S.n_BlockColumn_Column_Num(r_prev);
					size_t e_prev = y_prev + h_prev;
					_ASSERTE(S_dense_basis.middleRows(e_prev, y - e_prev).squaredNorm() == 0); // make sure there are zeros between consecutive (nonadjacent) blocks
				} else if(y > 0)
					_ASSERTE(S_dense_basis.topRows(y).squaredNorm() == 0); // make sure there are zeros above the first block
				if(j + 1 == t_ctx.n_dep_num)
					_ASSERTE(S_dense_basis.bottomRows(S_dense_basis.rows() - (y + h)).squaredNorm() == 0); // make sure there are zeros till the end
				// make sure that there are only zeros in between the elements
#endif // _DEBUG
				//_ASSERTE(h == 6);
				//_ASSERTE(S_bases.n_BlockColumn_Column_Num(n_row) == 6); // t_odo - FBS it
				//S_bases.t_GetBlock_Log(n_row, n_column, h, w) =
				//	S_dense_basis.middleRows<6>(S_bases.n_BlockColumn_Base(n_row)); // this is transposed (transpose the block as well?): each row is a single basis; this only works if the structure of S is symmetric

				fbs_ut::CWrap2<CInnerLoop, fbs_ut::CCTSize<n_column_width> >::template
					In_RowHeight_DecisionTree_Given_ColumnWidth<CBlockSizeList,
					n_column_width>(int(h), TInnerContext(t_ctx.r_S_bases,
					t_ctx.n_column, t_ctx.r_workspace, n_row));
				// use SSE to copy stuff around
			}
			// sparse fill the bases matrix
		}
	};
};

template <class CBlockSizeList>
void Calculate_UpperTriangularTransposeSolve_Bases_FBS(CUberBlockMatrix &S_bases,
	const CUberBlockMatrix &S, /*const*/ cs *p_St, size_t n_column,
	Eigen::MatrixXd &r_workspace, std::vector<size_t> &r_workspace1)
{
	_ASSERTE(S.b_EqualLayout(S_bases)); // will yield a matrix with the same sparsity structure
	//S.CopyLayoutTo(S_bases);

	_ASSERTE(n_column < S.n_BlockColumn_Num()); // make sure the column is inside the matrix

	_ASSERTE(p_St->m == S.n_BlockRow_Num() && p_St->n == S.n_BlockColumn_Num()); // should be the same matrix

	_ASSERTE(sizeof(csi) == sizeof(size_t));
	r_workspace1.resize(2 * p_St->n);
	// alloc workspace

	size_t w = S.n_BlockColumn_Column_Num(n_column); // t_odo - FBS it
	//_ASSERTE(w == 6);
	//n_row = n_column;

	//size_t n_first_dep_col = cs_reach(p_St, &B, 0, (csi*)&r_workspace1.front(), 0); // modifies p_St but then puts it back
	size_t n_first_dep_col = cs_dfs(n_column, p_St, p_St->n, (csi*)&r_workspace1.front(),
		(csi*)&r_workspace1.front() + p_St->n, 0); // no need for B // todo - reimplement this directly on block matrices, try to avoid needing the transpose
	size_t n_dep_col_num = p_St->n - n_first_dep_col;
	const size_t *p_dep_col = &r_workspace1[n_first_dep_col];
	for(size_t j = 0; j < n_dep_col_num; ++ j)
		CS_MARK(p_St->p, p_dep_col[j]); // restore col. pointers after calling cs_dfs
	// get the list of columns of S that affect block U_Dinv_{n_column, *}
	// note that this is FBS-independent

	fbs_ut::CWrap2<CUTTSolve_Bases_Impl::COuterLoop, CBlockSizeList>::template
		In_ColumnWidth_DecisionTree<CBlockSizeList>(int(w),
		CUTTSolve_Bases_Impl::TOuterContext(S_bases,
		n_column, S, r_workspace, p_dep_col, n_dep_col_num));
}

} // ~sc_margs_detail

template <class _SC_BlockSizes, class _U_BlockSizes,
	class _V_BlockSizes, class _D_BlockSizes>
class CSchurComplement_Marginals {
public:
	typedef _SC_BlockSizes SC_BlockSizes;
	typedef _SC_BlockSizes A_BlockSizes; // the same
	typedef _U_BlockSizes U_BlockSizes;
	typedef _V_BlockSizes V_BlockSizes;
	typedef _D_BlockSizes D_BlockSizes;
	typedef typename CUniqueTypelist<typename CConcatTypelist<typename
		CConcatTypelist<A_BlockSizes, U_BlockSizes>::_TyResult,
		typename CConcatTypelist<V_BlockSizes,
		D_BlockSizes>::_TyResult>::_TyResult>::_TyResult Lambda_BlockSizes;

protected:
	bool m_b_verbose;

	mutable double m_f_time_Dinv_copy;
	mutable double m_f_time_sp_struct;
	mutable double m_f_time_S_bases;
	mutable double m_f_time_lm_inverse;
	mutable double m_f_time_cam_inverse;

	mutable double m_f_time_lambda_AMD;
	mutable double m_f_time_lambda_perm;
	mutable double m_f_time_lambda_Chol;
	mutable double m_f_time_lambda_recformula;
	mutable double m_f_time_lambda_unperm;

	mutable uint64_t m_n_worst_memory;

public:
	CSchurComplement_Marginals(bool b_verbose = false)
		:m_b_verbose(b_verbose), m_f_time_Dinv_copy(0), m_f_time_sp_struct(0),
		m_f_time_S_bases(0), m_f_time_lm_inverse(0), m_f_time_cam_inverse(0),
		m_f_time_lambda_AMD(0), m_f_time_lambda_perm(0), m_f_time_lambda_Chol(0),
		m_f_time_lambda_recformula(0), m_f_time_lambda_unperm(0), m_n_worst_memory(0)
	{}

	inline bool b_Verbose() const
	{
		return m_b_verbose;
	}

	void Add_LambdaChol_Time(double f_time_AMD, double f_time_perm, double f_time_Chol)
	{
		m_f_time_lambda_AMD += f_time_AMD;
		m_f_time_lambda_perm += f_time_perm;
		m_f_time_lambda_Chol += f_time_Chol;
	}

	void Dump() const
	{
		printf("\trecursive margs took %f sec, out of which:\n",
			m_f_time_lambda_AMD + m_f_time_lambda_perm + m_f_time_lambda_Chol +
			m_f_time_lambda_recformula + m_f_time_lambda_unperm);
		printf("\t\t  amd: %f\n", m_f_time_lambda_AMD);
		printf("\t\t perm: %f\n", m_f_time_lambda_perm);
		printf("\t\t Chol: %f\n", m_f_time_lambda_Chol);
		printf("\t\trform: %f\n", m_f_time_lambda_recformula);
		printf("\t\tunprm: %f\n", m_f_time_lambda_unperm);

		printf("\tSchur margs took %f sec, out of which:\n", m_f_time_Dinv_copy +
			m_f_time_sp_struct + m_f_time_S_bases + m_f_time_lm_inverse);
		printf("\t\tdinit: %f\n", m_f_time_Dinv_copy);
		printf("\t\tstruc: %f\n", m_f_time_sp_struct);
		printf("\t\tbases: %f\n", m_f_time_S_bases);
		printf("\t\t  inv: %f\n", m_f_time_lm_inverse);

		printf("\trecursive inverse of cameras using SC took %f sec\n",
			m_f_time_cam_inverse);
#ifdef __BA_MARGS_DO_MEMORY_PROFILING
		printf("\tSchur marginals took " PRIsizeB "B memory at most\n",
			PRIsizeBparams(m_n_worst_memory));
#endif // __BA_MARGS_DO_MEMORY_PROFILING
	}

	bool Get_CholeskyLambda(CUberBlockMatrix &R,
		CMatrixOrdering &lam_ord, const CUberBlockMatrix &lambda) const
	{
		CTimer t;
		CTimerSampler timer(t);

		lam_ord.p_BlockOrdering(lambda, true); // w.r.t. lambda_perm
		const size_t *p_lam_ord = lam_ord.p_Get_InverseOrdering();
		const size_t n_lam_ord_size = lam_ord.n_Ordering_Size();
		// ordering for lambda

		double f_lambda_amd_time = 0;
		timer.Accum_DiffSample(f_lambda_amd_time);
		m_f_time_lambda_AMD += f_lambda_amd_time;

		CUberBlockMatrix lambda_amd;
		lambda.Permute_UpperTriangular_To(lambda_amd, p_lam_ord, n_lam_ord_size, true);

		double f_lambda_perm_time = 0;
		timer.Accum_DiffSample(f_lambda_perm_time);
		m_f_time_lambda_perm += f_lambda_perm_time;

		/*typedef CConcatTypelist<CConcatTypelist<SC_BlockSizes, U_BlockSizes>::_TyResult,
			CConcatTypelist<V_BlockSizes, D_BlockSizes>::_TyResult>::_TyResult Lambda_BlockSizes;*/

		if(!R.CholeskyOf_FBS<Lambda_BlockSizes>(lambda_amd)) {
			fprintf(stderr, "error: got not pos def when factorizing lambda\n");
			return false;
		}

		double f_lambda_chol_time = 0;
		timer.Accum_DiffSample(f_lambda_chol_time);
		m_f_time_lambda_Chol += f_lambda_chol_time;

		if(m_b_verbose) {
			printf("\tCholesky of lambda took %f sec, out of which:\n",
				f_lambda_chol_time + f_lambda_perm_time + f_lambda_amd_time);
			printf("\t\t  amd: %f\n", f_lambda_amd_time);
			printf("\t\t perm: %f\n", f_lambda_perm_time);
			printf("\t\t Chol: %f, " PRIsize " elem. nnz (needs %.2f MB)\n",
				f_lambda_chol_time, R.n_NonZero_Num(), R.n_Allocation_Size_NoLastPage() / 1048576.0);
		}

		return true;
	}

	/**
	 *	@brief calculates block diagonal of the covariance matrix from a Schur-complemented system
	 *
	 *	@param[out] margs_recursive is filled with the marginals upon return
	 *	@param[in] R is Cholesky factorization of the system matrix
	 *	@param[in] lam_ord is reference to the AMD ordering that was used for R
	 *	@param[in] Dinv is inverse of the (diagonal) landmark matrix
	 *	@param[in] U_Dinv is a product of the top off-diagonal block of the Schur complement and Dinv
	 *	@param[in] b_negative_Dinv_UDinv is negative flag (set if your Dinv and UDinv are negative)
	 *
	 *	@note This function throw std::bad_alloc.
	 */
	void Recursive_Marginals(//CUberBlockMatrix &cam_cov_rec, CUberBlockMatrix &lm_cov_rec,
		CUberBlockMatrix &margs_recursive, const CUberBlockMatrix &R,
		const CMatrixOrdering &lam_ord) const // throw(std::bad_alloc)
	{
		CTimer t;
		CTimerSampler timer(t);

		double f_lambda_recformula_time = 0;

		{
			CUberBlockMatrix margs_ordered;
			CMarginals::Calculate_DenseMarginals_Recurrent_FBS<Lambda_BlockSizes>(margs_ordered, R,
				lam_ord, mpart_Diagonal, false);
			// calculate the thing

			timer.Accum_DiffSample(f_lambda_recformula_time);
			m_f_time_lambda_recformula += f_lambda_recformula_time;

			margs_ordered.Permute_UpperTriangular_To(margs_recursive, lam_ord.p_Get_Ordering(),
				lam_ord.n_Ordering_Size(), false); // no share! the original will be deleted
		}

		double f_lambda_unperm_time = 0;
		timer.Accum_DiffSample(f_lambda_unperm_time);
		m_f_time_lambda_unperm += f_lambda_unperm_time;

		if(m_b_verbose) {
			printf("\trecursive margs took %f sec, out of which:\n", f_lambda_unperm_time +
				f_lambda_recformula_time /*+ f_lambda_chol_time + f_lambda_perm_time + f_lambda_amd_time*/);
			/*printf("\t  amd: %.3f\n", f_lambda_amd_time);
			printf("\t perm: %.3f\n", f_lambda_perm_time);
			printf("\t Chol: %.3f, " PRIsize " elem. nnz (needs %.2f MB)\n",
				f_lambda_chol_time, R.n_NonZero_Num(), R.n_Allocation_Size_NoLastPage() / 1048576.0);*/
			printf("\t\trform: %f\n", f_lambda_recformula_time);
			printf("\t\tunprm: %f\n", f_lambda_unperm_time);
		}

		/*margs_recursive.SliceTo(cam_cov_rec, 0, n_matrix_cut, 0, n_matrix_cut, true);
		margs_recursive.SliceTo(lm_cov_rec, n_matrix_cut, n_size, n_matrix_cut, n_size, true);
		// get the submatrices*/
	}

	/**
	 *	@brief calculates block diagonal of the covariance matrix from a Schur-complemented system
	 *
	 *	@param[out] sp_cam_inv is filled with camera marginals upon return
	 *	@param[in] b_do_cam_marginals is camera marginals flag (if set, sp_cam_inv is filled, otherwise it is empty)
	 *	@param[out] sp_lm_inv is filled with landmark marginals upon return
	 *	@param[in] S is Cholesky factorization of the Schur complement
	 *	@param[in] SC_ord is reference to the fill-reducing ordering that was used to factorize S
	 *		complement (this is *not* the ordering that separates landmarks and poses)
	 *	@param[in] n_SC_ord_size is size of the ordering that was used for Schur complement
	 *	@param[in] Dinv is inverse of the (diagonal) landmark matrix
	 *	@param[in] U_Dinv is a product of the top off-diagonal block of the Schur complement and Dinv
	 *	@param[in] b_negative_Dinv_UDinv is negative flag (set if your Dinv and UDinv are negative)
	 *
	 *	@note This function throw std::bad_alloc.
	 */
	void Schur_Marginals(CUberBlockMatrix &sp_cam_inv, bool b_do_cam_marginals,
		CUberBlockMatrix &sp_lm_inv,
		const CUberBlockMatrix &S, const CMatrixOrdering &SC_ord,
		const CUberBlockMatrix &Dinv, const CUberBlockMatrix &U_Dinv,
		bool b_negative_Dinv_UDinv = false) const // throw(std::bad_alloc)
	{
#ifdef __BA_MARGS_DO_MEMORY_PROFILING
		static bool b_warned = false;
		if(!b_warned) {
			fprintf(stderr, "warning: __BA_MARGS_DO_MEMORY_PROFILING defined! "
				"do not use this run for timing measurements\n");
			b_warned = true;
		}
#endif // __BA_MARGS_DO_MEMORY_PROFILING

		const size_t *p_SC_ord = SC_ord.p_Get_InverseOrdering();
		const size_t n_SC_ord_size = SC_ord.n_Ordering_Size();

		_ASSERTE(n_SC_ord_size == S.n_BlockColumn_Num());
		// if this fails then the ordering is likely for the entire lambda; this is just the
		// ordering to get chol(shur(lambda))

		CTimer t;
		CTimerSampler timer(t);

		sp_lm_inv = Dinv; // start with that

		if(b_negative_Dinv_UDinv)
			sp_lm_inv.Scale_FBS_Parallel<D_BlockSizes>(-1);
		// this needs to be negated; the rest of the product is squared so it cancels out

		double f_diag_init_time = 0;
		timer.Accum_DiffSample(f_diag_init_time);
		m_f_time_Dinv_copy += f_diag_init_time;

		cs *p_S = S.p_BlockStructure_to_Sparse();

		CUberBlockMatrix U_Dinv_perm;
		U_Dinv.PermuteTo(U_Dinv_perm, p_SC_ord, n_SC_ord_size, true, false, true);
		// get a permuted view of U_Dinv (can be shared among the threads)

		cs *p_B = U_Dinv_perm.p_BlockStructure_to_Sparse();
		// grab its structure (can be shared among the threads)

		double f_struct_time = 0;
		timer.Accum_DiffSample(f_struct_time);
		m_f_time_sp_struct += f_struct_time;

		CUberBlockMatrix S_bases;

		std::vector<CUberBlockMatrix> S_bases_thr_list;

		double f_parallel_S_bases_time = 0;
#ifdef __BA_MARGS_DO_MEMORY_PROFILING
		uint64_t n_memory = 0;
#endif // __BA_MARGS_DO_MEMORY_PROFILING

		#pragma omp parallel
		{
#ifdef _OPENMP
			const size_t n_thread_num = omp_get_num_threads();
			const size_t n_thread_id = omp_get_thread_num();
#else // _OPENMP
			static const size_t n_thread_num = 1;
			static const size_t n_thread_id = 0;
#endif // _OPENMP

			#pragma omp master
			{
				S_bases_thr_list.resize(n_thread_num);
			}
			#pragma omp barrier
			// alloc partials in thread 0

			std::vector<size_t> workspace(n_SC_ord_size * 2);
			// alloc thread-private workspace for cs_reach()

			cs *p_St_thr = cs_transpose(p_S, 0);
			// need a private copy of p_St for each thread

			{
				CUberBlockMatrix &S_bases_thr = S_bases_thr_list[n_thread_id];
				// rename

				S.CopyLayoutTo(S_bases_thr);

				Eigen::MatrixXd/*<double, Eigen::Dynamic, 6>*/ S_dense_basis;//(S.n_Row_Num(), 6); // todo - implement matrix solving and try to make this row major
				// unit basis matrix

				const size_t n = S.n_BlockColumn_Num();
				_ASSERTE(n <= INT_MAX);
				const int _n = int(n);
				#pragma omp for schedule(dynamic, 1) // t_odo - dynamic schedule? each column will likely have a different cost (todo - build histograms)
				for(int i = 0; i < _n; ++ i) {
					S_dense_basis.resize(S.n_Row_Num(), S.n_BlockColumn_Column_Num(i)); // handle different block sizes

					sc_margs_detail::Calculate_UpperTriangularTransposeSolve_Bases_FBS<SC_BlockSizes>(S_bases_thr,
						S, p_St_thr, i, S_dense_basis, workspace);
					// use the nice function instead
				}
			}

			#pragma omp barrier
			// wait for all the threads to compute their bases

			#pragma omp master
			{
				S_bases.Swap(S_bases_thr_list.front()); // start with 0
				for(size_t i = 1, n = n_thread_num; i < n; ++ i)
					S_bases_thr_list[i].AddTo(S_bases); // no need for FBS, no two blocks will overlap
				// simple serial reduction in thread 0, could do a parallel one

				std::vector<CUberBlockMatrix> empty;
				S_bases_thr_list.swap(empty);

				timer.Accum_DiffSample(f_parallel_S_bases_time);
				m_f_time_S_bases += f_parallel_S_bases_time;
			}
			// reduce the bases

			#pragma omp barrier
			// synchronize the threads before continuing

			Eigen::Matrix3d lm_i_cov;
			Eigen::Matrix<double, Eigen::Dynamic, 3> U_Dinv_i_permd(U_Dinv.n_Row_Num(), 3);
			// those can stay allocated, the size does not change throughout the algorithm

			const size_t n = Dinv.n_BlockColumn_Num();
			_ASSERTE(n <= INT_MAX);
			const int _n = int(n);
			#pragma omp for schedule(dynamic, 1) // t_odo - dynamic schedule? each column will likely have a different cost (todo - build histograms)
			for(int i = 0; i < _n; ++ i) {
				CUberBlockMatrix U_Dinv_i_perm;
				U_Dinv_perm.SliceTo(U_Dinv_i_perm, 0, U_Dinv_perm.n_BlockRow_Num(), i, i + 1, true);
				// grab a single column of U_Dinv_perm (via reference)

				CUberBlockMatrix SinvT_U_Dinv_i_perm;
				SinvT_U_Dinv_i_perm.ProductOf_FBS<SC_BlockSizes, U_BlockSizes>(S_bases, U_Dinv_i_perm);
				// gets a sparse matrix, the size of U_Dinv_i_perm

#ifdef __BA_MARGS_DO_MEMORY_PROFILING
				size_t n_mem_col = SinvT_U_Dinv_i_perm.n_Allocation_Size_NoLastPage();
				#pragma omp critical
				{
					n_memory = std::max(uint64_t(n_mem_col), n_memory);
				}
#endif // __BA_MARGS_DO_MEMORY_PROFILING

				CUberBlockMatrix::_TyMatrixXdRef t_lminv_ii = sp_lm_inv.t_GetBlock_Log(i, i);
				sc_margs_detail::BlockVector_PreMultiplyWithSelfTranspose_Add_FBS<U_BlockSizes>(t_lminv_ii, SinvT_U_Dinv_i_perm);
			}

			cs_spfree(p_St_thr);
		}
		// calculate block diagonal covariances of only the landmarks

#ifdef __BA_MARGS_DO_MEMORY_PROFILING
		n_memory += CSparseMatrixMemInfo::n_Allocation_Size(p_S) +
			CSparseMatrixMemInfo::n_Allocation_Size(p_B);
		//n_memory += sp_lm_inv.n_Allocation_Size_NoLastPage(); // the marginals themselves // not!
#endif // __BA_MARGS_DO_MEMORY_PROFILING

		cs_spfree(p_S);
		cs_spfree(p_B);

		double f_inverse_time = 0, f_lminv_time = 0;
		timer.Accum_DiffSample(f_inverse_time);
		m_f_time_lm_inverse += f_inverse_time;
		f_lminv_time = f_struct_time + f_diag_init_time + f_parallel_S_bases_time + f_inverse_time;

		if(m_b_verbose) {
			printf("\tSchur margs took %f sec, out of which:\n", f_lminv_time);
			printf("\t\tstruc: %f\n", f_struct_time);
			printf("\t\tbases: %f (%.2f %% sparsity, S has %.2f %%, needed %.2f MB)\n",
				f_parallel_S_bases_time, 100 * double(S_bases.n_Block_Num() * 6 * 6) /
				(S_bases.n_BlockColumn_Num() * S_bases.n_BlockColumn_Num() * 6 * 6),
				100 * double(S.n_Block_Num() * 6 * 6) / (S.n_BlockColumn_Num() *
				S.n_BlockColumn_Num() * 6 * 6), S_bases.n_Allocation_Size_NoLastPage() / 1048576.0);
			printf("\t\tdinit: %f\n", f_diag_init_time);
			printf("\t\t  inv: %f\n", f_inverse_time);
#ifdef __BA_MARGS_DO_MEMORY_PROFILING
			printf("\tSchur margs took " PRIsizeB "B of memory\n", PRIsizeBparams(n_memory));
#endif // __BA_MARGS_DO_MEMORY_PROFILING
		}

		double f_stats_time = 0;
		timer.Accum_DiffSample(f_stats_time);

#ifdef __BA_MARGS_DO_MEMORY_PROFILING
		uint64_t n_memory_cams = 0;
#endif // __BA_MARGS_DO_MEMORY_PROFILING
		if(b_do_cam_marginals) {
			CUberBlockMatrix &rcs_cov = sp_cam_inv; // just rename
			{
				CUberBlockMatrix margs_ordered;
				CMarginals::Calculate_DenseMarginals_Recurrent_FBS<SC_BlockSizes>(margs_ordered, S,
					SC_ord, mpart_Diagonal, false);

				margs_ordered.Permute_UpperTriangular_To(rcs_cov, SC_ord.p_Get_Ordering(),
					SC_ord.n_Ordering_Size(), false); // no share! the original will be deleted
			}

			double f_rcs_inverse_time = 0;
			timer.Accum_DiffSample(f_rcs_inverse_time);
			m_f_time_cam_inverse += f_rcs_inverse_time;

#ifdef __BA_MARGS_DO_MEMORY_PROFILING
			n_memory_cams = rcs_cov.n_Allocation_Size_NoLastPage(); // !! compute even if not in verbose
			for(size_t i = 0, n = rcs_cov.n_BlockColumn_Num(); i < n; ++ i) {
				size_t n_dof = rcs_cov.n_BlockColumn_Column_Num(i);
				_ASSERTE(n_memory_cams >= n_dof * n_dof * sizeof(double)); // make sure the below line does not underflow
				n_memory_cams -= n_dof * n_dof * sizeof(double);
			}
			// do not count the size of marginals we're trying to return! just count the unwanted off-diagonals
#endif // __BA_MARGS_DO_MEMORY_PROFILING
			if(m_b_verbose) {
				printf("\trecursive inverse of camera SC took %f sec (recovered " PRIsize " blocks)\n",
					f_rcs_inverse_time, rcs_cov.n_Block_Num());
#ifdef __BA_MARGS_DO_MEMORY_PROFILING
				printf("\trecursive inverse of camera SC takes " PRIsizeB "B of memory\n",
					PRIsizeBparams(n_memory_cams));
#endif // __BA_MARGS_DO_MEMORY_PROFILING
			}
		} else
			sp_cam_inv.Clear();

#ifdef __BA_MARGS_DO_MEMORY_PROFILING
		m_n_worst_memory = std::max(m_n_worst_memory, n_memory + n_memory_cams);
#endif // __BA_MARGS_DO_MEMORY_PROFILING
	}
};

#endif // !__BA_MARGINALS_INCLUDED
