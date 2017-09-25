/*
								+---------------------------------+
								|                                 |
								| *** Über Block Matrix tests *** |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2016 |
								|                                 |
								|      BlockMatrixTests.cpp       |
								|                                 |
								+---------------------------------+
*/

#include <stdio.h>
#include <map>
#include "slam/BlockMatrixTests.h"

/**
 *	@file src/slam/BlockMatrixTests.cpp
 *	@brief block matrix testing functions and helpers implementation
 *	@author -tHE SWINe-
 *	@date 2016-08-06
 */

/**
 *	@def RuntimeAssert
 *
 *	@brief assertion for the unit tests; prints failures to stdout
 *
 *	@param[in] b_expression is the checked expression
 *	@param[in,out] n_fail_num is the fail counter which is incremented on every failure; care is taken not to overflow INT_MAX
 */
#define RuntimeAssert(b_expression,n_fail_num) do { if(!(b_expression)) { \
	fprintf(stderr, "error: %s line %d: assertion \'%s\' failed\n", __FILE__, __LINE__, #b_expression); \
	if((n_fail_num) < INT_MAX) ++ (n_fail_num); } _ASSERTE(b_expression); } while(false)

/*
 *								=== CTestMatrixFactory ===
 */

cs *CTestMatrixFactory::p_Load_PatternMatrix(const char *p_s_filename) // throw(std::bad_alloc)
{
	CUberBlockMatrix ubm;
	if(!ubm.Load_MatrixMarket(p_s_filename, 1, false)) // note that if it is symmetric, it will be represented in full
		return 0;
	return ubm.p_Convert_to_Sparse(); // will contain values but that's ok
}

cs *CTestMatrixFactory::p_AllocUpper(csi m, csi n, double f_value /*= 1.0*/) // throw(std::bad_alloc)
{
	if(n && m > LONG_MAX / n)
		return 0; // would overflow below
	size_t n_nnz = std::min(m, n) * (std::min(m, n) - 1) / 2 + std::min(m, n) + // the square triangular section
		(n - std::min(m, n)) * m; // the right side if the matrix is wider
	cs *p_matrix = cs_spalloc(m, n, n_nnz, 1, 0);
	if(!p_matrix)
		throw std::bad_alloc();
	csi n_off = 0;
	for(csi i = 0; i < n; ++ i) {
		p_matrix->p[i] = n_off;
		for(csi j = 0, o = std::min(i + 1, m); j < o; ++ j, ++ n_off) {
			p_matrix->i[n_off] = j;
			p_matrix->x[n_off] = f_value;
		}
	}
	p_matrix->p[n] = n_off;
	_ASSERTE(n_off == n_nnz);
	return p_matrix;
}

cs *CTestMatrixFactory::p_AllocLower(csi m, csi n, double f_value /*= 1.0*/) // throw(std::bad_alloc)
{
	if(n && m > LONG_MAX / n)
		return 0; // would overflow below
	size_t n_nnz = std::min(m, n) * (std::min(m, n) - 1) / 2 + std::min(m, n) + // the square triangular section
		(m - std::min(m, n)) * n; // the bottom side if the matrix is narrow (completely filled)
	cs *p_matrix = cs_spalloc(m, n, n_nnz, 1, 0);
	if(!p_matrix)
		throw std::bad_alloc();
	csi n_off = 0;
	for(csi i = 0; i < n; ++ i) {
		p_matrix->p[i] = n_off;
		for(csi j = i; j < m; ++ j, ++ n_off) {
			p_matrix->i[n_off] = j;
			p_matrix->x[n_off] = f_value;
		}
	}
	p_matrix->p[n] = n_off;
	_ASSERTE(n_off == n_nnz);
	return p_matrix;
}

void CTestMatrixFactory::Conform_SparseMatrix_to_BlockMatrix(CUberBlockMatrix &r_dest,
	const cs *p_matrix, size_t n_block_size, bool b_allow_underfilled_last /*= true*/,
	bool b_append_identity_diag_block /*= true*/) // throw(std::bad_alloc, std::runtime_error)
{
	if(!b_allow_underfilled_last && (p_matrix->n % n_block_size || p_matrix->m % n_block_size))
		throw std::runtime_error("matrix cannot be conformed");

	size_t n_block_col_num = (p_matrix->n + n_block_size - 1) / n_block_size;
	size_t n_block_row_num = (p_matrix->m + n_block_size - 1) / n_block_size;

	std::vector<size_t> block_cols_list(n_block_col_num + 1, 0);
	std::vector<size_t> block_rows_list(n_block_row_num + 1, 0);
	for(size_t i = 0, b = n_block_size; i < n_block_col_num; ++ i, b += n_block_size)
		block_cols_list[i + 1] = b;
	for(size_t i = 0, b = n_block_size; i < n_block_row_num; ++ i, b += n_block_size)
		block_rows_list[i + 1] = b;
	// generate block layouts (directly in cumsum)

	{
		CUberBlockMatrix structure(block_rows_list.begin() + 1, block_rows_list.end(),
			block_cols_list.begin() + 1, block_cols_list.end());
		_ASSERTE(structure.n_BlockRow_Num() == p_matrix->m &&
			structure.n_BlockColumn_Num() == p_matrix->n);
		r_dest.Swap(structure);
	}
	// build the structure of the matrix

	if(b_append_identity_diag_block && (p_matrix->n % n_block_size || p_matrix->m % n_block_size)) {
		size_t n_diag = std::min(n_block_col_num, n_block_row_num) - 1;
		CUberBlockMatrix::_TyMatrixXdRef t_last_diag = r_dest.t_GetBlock_Log(n_diag, n_diag,
			r_dest.n_BlockRow_Row_Num(n_diag), r_dest.n_BlockColumn_Column_Num(n_diag), true, false);
		t_last_diag.setIdentity();
	}
	// in case the matrix is made bigger, append an identity block to make sure it is not rank deficient

	/*for(size_t i = 0, n = p_matrix->n; i < n; ++ i) {
		size_t n_width;
		size_t bc = r_dest.n_Find_BlockColumn(i, n_width);
		_ASSERTE(bc != size_t(-1));
		size_t n_col_base = r_dest.n_BlockColumn_Base(bc);

		for(size_t p = p_matrix->p[i], e = p_matrix->p[i + 1]; p < e; ++ p) {
			size_t j = p_matrix->i[p];
			double x = p_matrix->x[p];

			size_t n_height;
			size_t br = r_dest.n_Find_BlockRow(i, n_height);
			_ASSERTE(br != size_t(-1));
			size_t n_row_base = r_dest.n_BlockRow_Base(br);

			CUberBlockMatrix::_TyMatrixXdRef t_block = r_dest.t_GetBlock_Log(br, bc,
				n_height, n_width, true, true);

			t_block(j - n_row_base, i - n_col_base) = x;
		}
	}
	// allocate the blocks in the matrix*/

	std::vector<size_t> workspace;
	if(!r_dest.From_Sparse(0, 0, p_matrix, false, workspace))
		throw std::runtime_error("r_dest.From_Sparse() failed");
	// same thing as the loop above, only faster (does not clear matrix
	// data so the identity diagonal block will stay there)
}

void CTestMatrixFactory::Inflate_PatternMatrix_to_BlockMatrix(CUberBlockMatrix &r_dest,
	const cs *p_matrix, const std::vector<size_t> &r_block_sizes, bool b_symmetric) // throw(std::bad_alloc)
{
	std::vector<size_t> block_cols_list((b_symmetric)? std::max(p_matrix->m, p_matrix->n) : p_matrix->n);
	std::vector<size_t> block_rows_list((b_symmetric)? 0 : p_matrix->m);
	// allocate block layouts

	const size_t n_block_size_num = r_block_sizes.size();
	_ASSERTE(n_block_size_num);

	for(size_t i = 0, n = block_cols_list.size(); i < n; ++ i)
		block_cols_list[i] = r_block_sizes[rand() % n_block_size_num];
	for(size_t i = 0, n = block_rows_list.size(); i < n; ++ i)
		block_rows_list[i] = r_block_sizes[rand() % n_block_size_num];
	// generate random block layouts

	std::partial_sum(block_cols_list.begin(), block_cols_list.end(), block_cols_list.begin());
	block_cols_list.insert(block_cols_list.begin(), 1, size_t(0));
	if(!b_symmetric) {
		std::partial_sum(block_rows_list.begin(), block_rows_list.end(), block_rows_list.begin());
		block_rows_list.insert(block_rows_list.begin(), 1, size_t(0));
	} else
		block_rows_list = block_cols_list;
	block_rows_list.resize(p_matrix->m + 1);
	block_cols_list.resize(p_matrix->n + 1);
	// calculate cumsums and split to two

	{
		CUberBlockMatrix structure(block_rows_list.begin() + 1, block_rows_list.end(),
			block_cols_list.begin() + 1, block_cols_list.end());
		_ASSERTE(structure.n_BlockRow_Num() == p_matrix->m &&
			structure.n_BlockColumn_Num() == p_matrix->n);
		r_dest.Swap(structure);
	}
	// build the of the matrix

	for(size_t i = 0, n = p_matrix->n; i < n; ++ i) {
		for(size_t p = p_matrix->p[i], e = p_matrix->p[i + 1]; p < e; ++ p) {
			size_t j = p_matrix->i[p];
			double x = p_matrix->x[p];

			CUberBlockMatrix::_TyMatrixXdRef t_block = r_dest.t_GetBlock_Log(j, i,
				r_dest.n_BlockRow_Row_Num(j), r_dest.n_BlockColumn_Column_Num(i), true, false);

			t_block.setConstant(x);
			t_block.diagonal() *= 2; // raise the diagonal to be pos def
		}
	}
	// allocate the blocks in the matrix
}

void CTestMatrixFactory::Split_BlockMatrix(std::vector<CUberBlockMatrix> &r_split_matrix,
	CUberBlockMatrix &r_perm_layout, std::vector<size_t> &r_block_row_perm,
	std::vector<size_t> &r_block_col_perm, size_t &r_n_row_ext, size_t &r_n_col_ext,
	const CUberBlockMatrix &r_block_mat) // throw(std::bad_alloc)
{
	std::set<size_t> col_widths, row_heights;

	CUberBlockMatrix block_mat;
	const_cast<CUberBlockMatrix&>(r_block_mat).SliceTo(block_mat,
		r_block_mat.n_BlockRow_Num(), r_block_mat.n_BlockColumn_Num(), true);
	// make a shallow copy so that we can add new rows / cols

	for(size_t i = 0, n = block_mat.n_BlockRow_Num(); i < n; ++ i)
		row_heights.insert(block_mat.n_BlockRow_Row_Num(i));
	for(size_t i = 0, n = block_mat.n_BlockColumn_Num(); i < n; ++ i)
		col_widths.insert(block_mat.n_BlockColumn_Column_Num(i));
	// collect

	std::map<size_t, std::pair<size_t, size_t> > col_ranges, row_ranges;

	r_block_col_perm.clear();
	size_t n_next_elem_col = 0;
	for(std::set<size_t>::const_iterator p_cw_it = col_widths.begin(),
	   p_end_it = col_widths.end(); p_cw_it != p_end_it; ++ p_cw_it) {
		size_t n_col_width = *p_cw_it;

		if(n_next_elem_col % n_col_width) {
			size_t n_pad = n_Align_Up(n_next_elem_col, n_col_width) - n_next_elem_col;
			block_mat.ExtendTo(block_mat.n_Row_Num(), block_mat.n_Column_Num() + n_pad);
			// add a new block column

			r_block_col_perm.push_back(block_mat.n_BlockColumn_Num() - 1);
			n_next_elem_col += n_pad;
			// add it to the ordeting
		}
		// attempt to align everything, in order to be able to use BSR as much as possible

		col_ranges[n_col_width].first = r_block_col_perm.size(); // in blocks

		for(size_t i = 0, n = r_block_mat.n_BlockColumn_Num(); i < n; ++ i) { // only the original ones, not the padding!
			if(n_col_width == block_mat.n_BlockColumn_Column_Num(i)) {
				r_block_col_perm.push_back(i);
				n_next_elem_col += n_col_width;
			}
		}
		// do another pass, concatenate block columns

		col_ranges[n_col_width].second = r_block_col_perm.size(); // in blocks
	}
	_ASSERTE(n_next_elem_col == block_mat.n_Column_Num());

	//row_ranges.clear(); // debug
	r_block_row_perm.clear();
	size_t n_next_elem_row = 0;
	for(std::set<size_t>::const_iterator p_rh_it = row_heights.begin(),
	   p_end_it = row_heights.end(); p_rh_it != p_end_it; ++ p_rh_it) {
		size_t n_row_height = *p_rh_it;

		if(n_next_elem_row % n_row_height) {
			size_t n_pad = n_Align_Up(n_next_elem_row, n_row_height) - n_next_elem_row;
			block_mat.ExtendTo(block_mat.n_Row_Num() + n_pad, block_mat.n_Column_Num());
			// add a new block row

			r_block_row_perm.push_back(block_mat.n_BlockRow_Num() - 1);
			n_next_elem_row += n_pad;
			// add it to the ordeting
		}
		// attempt to align everything, in order to be able to use BSR as much as possible

		row_ranges[n_row_height].first = r_block_row_perm.size(); // in blocks

		for(size_t i = 0, n = r_block_mat.n_BlockRow_Num(); i < n; ++ i) { // only the original ones, not the padding!
			if(n_row_height == block_mat.n_BlockRow_Row_Num(i)) {
				r_block_row_perm.push_back(i);
				n_next_elem_row += n_row_height;
			}
		}
		// do another pass, concatenate block columns

		row_ranges[n_row_height].second = r_block_row_perm.size(); // in blocks
	}
	_ASSERTE(n_next_elem_row == block_mat.n_Row_Num());

	/*size_t n_nr = col_widths.size() * row_heights.size();
	char p_s_filename[256];
	sprintf(p_s_filename, "slice_%02" _PRIsize "_00_orig.tga", n_nr);
	r_block_mat.Rasterize(p_s_filename, 5);
	sprintf(p_s_filename, "slice_%02" _PRIsize "_01_extended.tga", n_nr);
	block_mat.Rasterize(p_s_filename, 5);*/
	// debug - see images

	std::vector<size_t> rperm(r_block_row_perm.size()), cperm(r_block_col_perm.size());
	for(size_t i = 0, n = r_block_row_perm.size(); i < n; ++ i)
		rperm[r_block_row_perm[i]] = i;
	for(size_t i = 0, n = r_block_col_perm.size(); i < n; ++ i)
		cperm[r_block_col_perm[i]] = i;
	// invert the perms

	CUberBlockMatrix perm_mat;
	{
		CUberBlockMatrix rp_temp;
		block_mat.PermuteTo(rp_temp, &rperm.front(), rperm.size(), true, false, true); // reorder rows
		rp_temp.PermuteTo(perm_mat, &cperm.front(), cperm.size(), false, true, true); // reorder cols
	}
	// permute the matrix to have the blocks of the same size in
	// contigouous ranges (as recorded in row_ranges, col_ranges)

	/*sprintf(p_s_filename, "slice_%02" _PRIsize "_02_perm.tga", n_nr);
	perm_mat.Rasterize(p_s_filename, 5);*/
	// debug - see images

	size_t n_block_num = 0;

	r_split_matrix.resize(col_widths.size() * row_heights.size());
	size_t n_idx = 0;
	for(std::set<size_t>::const_iterator p_rh_it = col_widths.begin(),
	   p_end_it = col_widths.end(); p_rh_it != p_end_it; ++ p_rh_it) {
		const size_t n_row_height = *p_rh_it;
		const std::pair<size_t, size_t> row_range = row_ranges[n_row_height];

		if(row_range.first == row_range.second)
			continue; // no nonzero matrices here

		size_t n_first_row = perm_mat.n_BlockRow_Base(row_range.first);
		_ASSERTE(!(n_first_row % n_row_height)); // we even added padding to make this fit

		for(std::set<size_t>::const_iterator p_cw_it = col_widths.begin(),
		   p_end_it = col_widths.end(); p_cw_it != p_end_it; ++ p_cw_it, ++ n_idx) {
			const size_t n_col_width = *p_cw_it;
			const std::pair<size_t, size_t> col_range = col_ranges[n_col_width];

			if(col_range.first == col_range.second)
				continue; // no nonzero matrices here

			size_t n_first_col = perm_mat.n_BlockColumn_Base(col_range.first);
			_ASSERTE(!(n_first_col % n_col_width)); // we even added padding to make this fit

			CUberBlockMatrix &r_dest = r_split_matrix[n_idx];
			// this matrix will only contain n_col_widthxn_row_height blocks

			r_dest.Clear();
			//r_dest.ExtendTo(n_first_row, n_first_col); // t_odo - extend in n_row_height x n_col_width blocks!
			for(size_t r = 0; r < n_first_row;)
				r_dest.ExtendTo(r += n_row_height, 0);
			for(size_t c = 0; c < n_first_col;)
				r_dest.ExtendTo(n_first_row, c += n_col_width);
			size_t n_dest_brow = r_dest.n_BlockRow_Num(), n_dest_bcol = r_dest.n_BlockColumn_Num();
			// upper left blank space

			for(size_t r = row_range.first; r < row_range.second; ++ r) {
				_ASSERTE(perm_mat.n_BlockRow_Row_Num(r) == n_row_height);
				r_dest.ExtendTo(r_dest.n_Row_Num() + n_row_height, r_dest.n_Column_Num());
			}
			for(size_t c = col_range.first; c < col_range.second; ++ c) {
				_ASSERTE(perm_mat.n_BlockColumn_Column_Num(c) == n_col_width);
				r_dest.ExtendTo(r_dest.n_Row_Num(), r_dest.n_Column_Num() + n_col_width);
			}
			// space for the nonzeros

			CUberBlockMatrix slice;
			perm_mat.SliceTo(slice, row_range.first, row_range.second,
				col_range.first, col_range.second, true); // shallow copy
			slice.PasteTo(r_dest, n_dest_brow, n_dest_bcol); // paste, deep copy (could have a shallow copy also, all the data in perm_mat points to r_block_mat); this will improve bandwidth of the BSR GAXPY later

			n_block_num += r_dest.n_Block_Num();

			/*{
				CUberBlockMatrix for_image;
				r_dest.SliceTo(for_image, r_dest.n_BlockRow_Num(), r_dest.n_BlockColumn_Num(), true);
				for_image.ExtendTo(perm_mat.n_Row_Num(), perm_mat.n_Column_Num());
				char p_s_str[256];
				sprintf(p_s_str, "slice_%02" _PRIsize "_03_slice_%03" _PRIsize ".tga", n_nr, n_idx);
				for_image.Rasterize(p_s_str, 5); // debug - see images
			}*/
			// extend the matrix to full size, to make an image which perfectly overlaps with the extended matrix images
		}
	}
	// build the individual slices

	_ASSERTE(n_idx <= r_split_matrix.size());
	r_split_matrix.resize(n_idx);
	// sometimes some block size is not present

	_ASSERTE(n_block_num == r_block_mat.n_Block_Num());
	// make sure the split was conservative (still many ways to go wrong, but better than not checking at all)

	/*rperm.swap(r_block_row_perm);
	cperm.swap(r_block_col_perm);*/ // nope
	block_mat.CopyLayoutTo(r_perm_layout);
	// will need that for permuting the vectors

	r_n_row_ext = block_mat.n_Row_Num() - r_block_mat.n_Row_Num();
	r_n_col_ext = block_mat.n_Column_Num() - r_block_mat.n_Column_Num();
	// how many rows / columns did we have to extend
}

/*
 *								=== ~CTestMatrixFactory ===
 */

/*
 *								=== CUberBlockMatrix_UnitTests ===
 */

int CUberBlockMatrix_UnitTests::Test_ElemwiseTriangularPredicates() // throw(std::bad_alloc)
{
	CMiniProgressIndicator progress;

	int n_fail_num = 0;

	srand(123456);

	for(int n_size_num = 1; n_size_num < 10; ++ n_size_num) {
		std::vector<size_t> block_size_list(n_size_num);
		for(int i = 0; i < n_size_num; ++ i)
			block_size_list[i] = i + 1; // 1x1 blocks as well
		// generate a list of block sizes

		for(int n_outer_pass = 0; n_outer_pass < 10; ++ n_outer_pass) { // try with different random seeds
			cs *p_pattern_matrix = CTestMatrixFactory::p_AllocUpper(1 +
				((n_outer_pass == 1 || n_outer_pass >= 3)? rand() % 50 : 0),
				1 + ((n_outer_pass>= 2)? rand() % 50 : 0));
			// 0: 1x1, 1: nx1, 2: 1xn: 3, 4, ...: nxn

			for(int n_symmetry = 0; n_symmetry < 2; ++ n_symmetry) {
				printf("\rtest triangular pred: %d block sizes, symmetry %d, pass %d  ", n_size_num, n_symmetry, n_outer_pass);

				CUberBlockMatrix block_mat, transp_mat;
				CTestMatrixFactory::Inflate_PatternMatrix_to_BlockMatrix(block_mat,
					p_pattern_matrix, block_size_list, n_symmetry != 0);
				// create a block matrix by randomly inflating the elements to dense blocks

				bool b_1xn = block_mat.n_Row_Num() == 1; // all 1xn matrices are upper triangular
				bool b_nx1 = block_mat.n_Column_Num() == 1; // all nx1 matrices are lower triangular

				RuntimeAssert(!b_1xn || block_mat.b_UpperTriangular(), n_fail_num);
				RuntimeAssert(!b_nx1 || block_mat.b_LowerTriangular(), n_fail_num);

				//block_mat.Rasterize("dbg_upper.tga");

				//RuntimeAssert(n_size_num == 1 || !n_symmetry || b_1xn ||
				//	(b_nx1 && block_mat.n_BlockRow_Row_Num(0) == 1) || !block_mat.b_UpperTriangular(), n_fail_num); // now the diagonal blocks are square and all contain nonzero values
				// not true, the blocks can actually skew the diagonal upwards if there is a lot of wide blocks
				// this is tricky, even if there are two block sizes, they actually might not get used and all the blocks can still be 1x1. this is a stupid test

				RuntimeAssert(n_size_num > 1 || block_mat.b_UpperTriangular(), n_fail_num); // all the blocks are 1x1 so it is now exactly upper triangular

				transp_mat.TransposeOf(block_mat);

				RuntimeAssert(!b_1xn || transp_mat.b_LowerTriangular(), n_fail_num);
				RuntimeAssert(!b_nx1 || transp_mat.b_UpperTriangular(), n_fail_num);

				//RuntimeAssert(n_size_num == 1 || !n_symmetry || b_1xn ||
				//	(b_nx1 && transp_mat.n_BlockColumn_Column_Num(0) == 1) || !transp_mat.b_LowerTriangular(), n_fail_num); // now the diagonal blocks are square and all contain nonzero values
				// not true, the blocks can actually skew the diagonal upwards if there is a lot of wide blocks
				// this is tricky, even if there are two block sizes, they actually might not get used and all the blocks can still be 1x1. this is a stupid test

				RuntimeAssert(n_size_num > 1 || transp_mat.b_LowerTriangular(), n_fail_num); // all the blocks are 1x1 so it is now exactly upper triangular

				block_mat.SetZero();
				transp_mat.SetZero();

				RuntimeAssert(block_mat.b_UpperTriangular(), n_fail_num);
				RuntimeAssert(block_mat.b_LowerTriangular(), n_fail_num);
				RuntimeAssert(transp_mat.b_UpperTriangular(), n_fail_num);
				RuntimeAssert(transp_mat.b_LowerTriangular(), n_fail_num);

				{
					cs *p_U = CTestMatrixFactory::p_AllocUpper(block_mat.n_Row_Num(), block_mat.n_Column_Num());
					std::vector<size_t> workspace;
					block_mat.From_Sparse(0, 0, p_U, false, workspace);
					cs_spfree(p_U);
				}

				RuntimeAssert(block_mat.b_UpperTriangular(), n_fail_num);
				RuntimeAssert(b_nx1 || !block_mat.b_LowerTriangular(), n_fail_num);

				transp_mat.TransposeOf(block_mat);

				RuntimeAssert(b_nx1 || !transp_mat.b_UpperTriangular(), n_fail_num);
				RuntimeAssert(transp_mat.b_LowerTriangular(), n_fail_num);

				for(size_t n_col = 0, n_last_col_with_lower_elems =
				   std::min(block_mat.n_Column_Num(), block_mat.n_Row_Num() - 1);
				   n_col < n_last_col_with_lower_elems; ++ n_col) {
					const size_t n_min_row = n_col + 1;
					const size_t n_max_row = block_mat.n_Row_Num();

					size_t n_col_width = size_t(0xbaadf00d);
					const size_t n_block_col = block_mat.n_Find_BlockColumn(n_col, n_col_width);
					RuntimeAssert(n_block_col != size_t(-1), n_fail_num);
					RuntimeAssert(n_col_width != size_t(0xbaadf00d), n_fail_num);
					const size_t n_col_base = block_mat.n_BlockColumn_Base(n_block_col);
					RuntimeAssert(n_col_base <= n_col, n_fail_num);

					for(size_t n_inner_pass = 0, n_inner_pass_num = 1 + std::min(size_t(10),
					   n_max_row - n_min_row); n_inner_pass < n_inner_pass_num; ++ n_inner_pass) {
						progress();

						CUberBlockMatrix not_upper = block_mat; // not upper triangular matrix

						const size_t n_row = (!n_inner_pass)? n_min_row : n_min_row + rand() % (n_max_row - n_min_row);
						// put a single nonzero below the diagonal

						size_t n_row_height = size_t(0xbaadf00d);
						const size_t n_block_row = not_upper.n_Find_BlockRow(n_row, n_row_height);
						RuntimeAssert(n_block_row != size_t(-1), n_fail_num);
						RuntimeAssert(n_row_height != size_t(0xbaadf00d), n_fail_num);
						const size_t n_row_base = not_upper.n_BlockRow_Base(n_block_row);
						RuntimeAssert(n_row_base <= n_row, n_fail_num);

						not_upper.t_GetBlock_Log(n_block_row, n_block_col, n_row_height,
							n_col_width, true, true)(n_row - n_row_base, n_col - n_col_base) = 12345.0;
						// scatter a single nonzero somewhere below the diagonal

						//not_upper.Rasterize("dbg_notupper.tga");

						RuntimeAssert(b_1xn || !not_upper.b_UpperTriangular(), n_fail_num);
						RuntimeAssert(b_nx1 || !not_upper.b_LowerTriangular(), n_fail_num); // neither, there are the nonzeros

						CUberBlockMatrix not_lower;
						not_lower.TransposeOf(not_upper);

						RuntimeAssert(b_1xn || !not_lower.b_LowerTriangular(), n_fail_num);
						RuntimeAssert(b_nx1 || !not_lower.b_UpperTriangular(), n_fail_num); // neither, there are the nonzeros

						not_upper.SetZero();
						not_upper.t_GetBlock_Log(n_block_row, n_block_col, n_row_height,
							n_col_width, true, true)(n_row - n_row_base, n_col - n_col_base) = 12345.0;
						// scatter a single nonzero somewhere below the diagonal

						RuntimeAssert(!not_upper.b_UpperTriangular(), n_fail_num);
						RuntimeAssert(not_upper.b_LowerTriangular(), n_fail_num); // now there is only a single nonzero in the lower portion

						not_lower.TransposeOf(not_upper);

						RuntimeAssert(!not_lower.b_LowerTriangular(), n_fail_num);
						RuntimeAssert(not_lower.b_UpperTriangular(), n_fail_num); // now there is only a single nonzero in the upper portion
					}
				}
			}

			cs_spfree(p_pattern_matrix);
		}
	}
	printf("\ndone (there were %d fails)\n", n_fail_num);

	return n_fail_num;
}

/*
 *								=== ~CUberBlockMatrix_UnitTests ===
 */
