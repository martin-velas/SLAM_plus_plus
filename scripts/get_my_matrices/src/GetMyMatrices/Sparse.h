#pragma once

#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/Dir.h"
#include "../UberLame_src/Bitmap.h"
#include "../UberLame_src/Integer.h"
#include <limits.h> // INT_MAX
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>

/**
 *	@def MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
 *	@brief if defined, optimize bitmap write routines to access the bitmaps row-wise for better cache reuse
 */
#define MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

/**
 *	@def MATRIX_SUBSAMPLE_RASTERIZE_IN_PARALLEL
 *	@brief if defined, parallelize p_Rasterize() and p_Rasterize_AA() (only has effect if
 *		\ref MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS is also defined)
 */
#define MATRIX_SUBSAMPLE_RASTERIZE_IN_PARALLEL

/**
 *	@brief a very simple COO / CSC sparse matrix storage with no arithmetics operations implemented on it
 */
struct TSparse {
	bool b_compressed;
	size_t m, n; // rows, cols
	size_t *p, *i; // n+1 col ptrs if b_compressed or nnz_max otherwise, row idxs
	double *x; // values
	size_t coo_nnz; // number of nz if !b_compressed or 0 otherwise
	size_t nnz_max; // size of x and i arrays

	TSparse(size_t n_row_num = 0, size_t n_col_num = 0, size_t _nnz_max = 0,
		bool _b_compressed = false, bool b_values = true) // throw(std::bad_alloc)
		:b_compressed(false), m(0), n(0), p(0), i(0), x(0), nnz_max(0), coo_nnz(0) // blank
	{
		Alloc(n_row_num, n_col_num, _nnz_max, _b_compressed, b_values);
	}

	TSparse(const TSparse &r_other) // throw(std::bad_alloc)
		:b_compressed(false), m(0), n(0), p(0), i(0), x(0), nnz_max(0), coo_nnz(0) // blank
	{
		if(r_other.p) { // allocated?
			Alloc(r_other.m, r_other.n, r_other.nnz_max, r_other.b_compressed, r_other.x != 0);

			if(b_compressed) {
				memcpy(p, r_other.p, (r_other.n + 1) * sizeof(size_t));
				size_t nnz = r_other.p[r_other.n];
				memcpy(i, r_other.i, nnz * sizeof(size_t));
				memcpy(x, r_other.x, nnz * sizeof(double));
				coo_nnz = 0;
			} else {
				size_t nnz = r_other.coo_nnz;
				memcpy(p, r_other.p, nnz * sizeof(size_t));
				memcpy(i, r_other.i, nnz * sizeof(size_t));
				memcpy(x, r_other.x, nnz * sizeof(double));
				coo_nnz = nnz;
			}
		}
	}

	TSparse &operator =(const TSparse &r_other) // throw(std::bad_alloc)
	{
		if(&r_other != this) {
			TSparse copy(r_other); // make a copy using copy ctor
			Swap(copy); // swap it
		}
		return *this;
	}

	~TSparse()
	{
		Destroy();
	}

	void Swap(TSparse &r_other)
	{
		std::swap(b_compressed, r_other.b_compressed);
		std::swap(m, r_other.m);
		std::swap(n, r_other.n);
		std::swap(p, r_other.p);
		std::swap(i, r_other.i);
		std::swap(x, r_other.x);
		std::swap(nnz_max, r_other.nnz_max);
		std::swap(coo_nnz, r_other.coo_nnz);
	}

	// loses data
	void Alloc(size_t n_row_num = 0, size_t n_col_num = 0,
		size_t _nnz_max = 0, bool _b_compressed = false, bool b_values = true) // throw(std::bad_alloc)
	{
		if(b_compressed == _b_compressed) {
			if(!b_compressed && _nnz_max <= nnz_max) {
				return; // will fit (nnz_max governs p, i, x)
			} else if(b_compressed && n_col_num <= n && _nnz_max <= nnz_max)
				return; // will fit (nnz_max governs i, x and n governx p)
		}
		Destroy();

		try {
			p = new size_t[(_b_compressed)? n_col_num + 1 : _nnz_max];
			i = new size_t[_nnz_max];
			if(b_values)
				x = new double[_nnz_max];
			else
				x = 0;
		} catch(std::bad_alloc &r_exc) {
			Destroy(); // make sure either all are allocated or all are null
			throw r_exc;
		}
		// alloc

		m = n_row_num;
		n = n_col_num;
		coo_nnz = 0;
		nnz_max = _nnz_max;
		b_compressed = _b_compressed;
		// set pointers
	}

	// keeps data
	void Realloc(size_t n_row_num = 0, size_t n_col_num = 0, size_t _nnz_max = 0) // throw(std::bad_alloc)
	{
		if(n_col_num == SIZE_MAX || _nnz_max == SIZE_MAX)
			throw std::length_error("matrix too big"); // need some headroom
		if(!b_compressed && nnz_max > _nnz_max) {
			m = n_row_num;
			n = n_col_num;
			return;
		}
		stl_ut::CUniquePtr<size_t> new_p, new_i;
		stl_ut::CUniquePtr<double> new_x;
		if(b_compressed && n_col_num > n)
			new_p = stl_ut::CUniquePtr<size_t>(new size_t[n_col_num + 1]);
		if(_nnz_max > nnz_max) {
			if(!b_compressed)
				new_p = stl_ut::CUniquePtr<size_t>(new size_t[_nnz_max]);
			new_i = stl_ut::CUniquePtr<size_t>(new size_t[_nnz_max]);
			if(x)
				new_x = stl_ut::CUniquePtr<double>(new double[_nnz_max]);
		}
		// alloc all the pointers

		if(b_compressed && n_col_num > n) {
			memcpy(new_p.p_Get(), p, sizeof(size_t) * (n + 1)); // copy the old structure
			size_t nnz = p[n];
			for(size_t i = n + 1; i < n_col_num; ++ i)
				new_p.p_Get()[i] = nnz; // add empty columns
			delete[] p;
			p = new_p.p_YieldOwnership();
			n = n_col_num;
		}
		// replace p

		if(_nnz_max > nnz_max) {
			size_t n_old_nnz = (b_compressed)? p[n] : coo_nnz; // still the same value even if reallocated p
			if(!b_compressed) {
				memcpy(new_p.p_Get(), p, sizeof(size_t) * n_old_nnz);
				delete[] p;
				p = new_p.p_YieldOwnership();
			}

			memcpy(new_i.p_Get(), i, sizeof(size_t) * n_old_nnz);
			delete[] i;
			i = new_i.p_YieldOwnership();

			if(x) {
				memcpy(new_x.p_Get(), x, sizeof(double) * n_old_nnz);
				delete[] x;
				x = new_x.p_YieldOwnership();
			}
			nnz_max = _nnz_max;
		}
		// replace i and x
	}

	// only works on triplet matrices
	void Entry(size_t n_row, size_t n_col, double f_value) // throw(std::bad_alloc)
	{
		_ASSERTE(!b_compressed);
		if(nnz_max == coo_nnz)
			Realloc(m, n, coo_nnz * 2);
		_ASSERTE(nnz_max > coo_nnz);
		p[coo_nnz] = n_col;
		i[coo_nnz] = n_row;
		if(x)
			x[coo_nnz] = f_value;
		++ coo_nnz; // !!
	}

	void Destroy()
	{
		if(p)
			delete[] p;
		if(i)
			delete[] i;
		if(x)
			delete[] x;
		p = 0;
		i = 0;
		x = 0;
		m = n = coo_nnz = nnz_max = 0;
	}

	void Clone(TSparse &r_dest) const // throw(std::bad_alloc)
	{
		r_dest.Alloc(m, n, nnz_max, b_compressed, x != 0);
		if(b_compressed) {
			memcpy(r_dest.p, p, (n + 1) * sizeof(size_t));
			memcpy(r_dest.i, i, p[n] * sizeof(size_t));
			if(x)
				memcpy(r_dest.x, x, p[n] * sizeof(double));
		} else {
			memcpy(r_dest.p, p, coo_nnz * sizeof(size_t));
			memcpy(r_dest.i, i, coo_nnz * sizeof(size_t));
			if(x)
				memcpy(r_dest.x, x, coo_nnz * sizeof(size_t));
			r_dest.coo_nnz = coo_nnz; // !!
		}
	}

	void TransposeInplace() // throw(std::bad_alloc)
	{
		if(b_compressed) {
			TSparse temp; // need temporary storage anyway, unable to do it fully inplace
			Transpose(temp);
			temp.Swap(*this);
		} else {
			std::swap(p, i); // just swap row and column indices
			std::swap(m, n); // swap row / col size as well
		}
	}

	void Transpose(TSparse &r_dest) const // throw(std::bad_alloc)
	{
		if(b_compressed) {
			r_dest.Alloc(n, m, nnz_max, true, x != 0);
			// alloc the dest matrix

			std::vector<size_t> row_counts(m, 0);
			for(size_t j = 0, o = p[n]; j < o; ++ j)
				++ row_counts[i[j]];
			// count entries in each column
			if(m) {
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
				std::partial_sum(row_counts.begin(), row_counts.end() - 1,
					stdext::checked_array_iterator<size_t*>(r_dest.p + 2, m + 1 - 2)); // allocated in transpose!
#else // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
				std::partial_sum(row_counts.begin(), row_counts.end() - 1, r_dest.p + 2);
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
			}
			r_dest.p[1] = 0; // prefix with zero for exclusive sum
			// cumsum (exclusive, shifted to the right)

			r_dest.p[0] = 0;
			// the first pointer is always zero

			for(size_t n_col = 0; n_col < n; ++ n_col) {
				for(size_t n_ptr = p[n_col], n_ptr2 = p[n_col + 1]; n_ptr != n_ptr2; ++ n_ptr) {
					size_t n_row = i[n_ptr];
					size_t n_tr_ptr = r_dest.p[n_row + 1];
					++ r_dest.p[n_row + 1];
					r_dest.i[n_tr_ptr] = n_col;
					if(x)
						r_dest.x[n_tr_ptr] = x[n_ptr];
				}
			}
			// write a transpose structure of the matrix
			// note that this sorts the matrix
		} else {
			Clone(r_dest);
			r_dest.TransposeInplace();
			// easy in coo format
		}
	}

	void Sort()
	{
		if(b_compressed) {
			std::vector<size_t> workspace;
			for(size_t j = 0; j < n; ++ j) {
				size_t n_begin = p[j], n_end = p[j + 1];
				Pair_Sort(i + n_begin, (x)? x + n_begin : 0, n_end - n_begin, workspace);
			}
			// sort row/value pairs, the columns are already sorted
		} else {
			Triplet_Sort(p, i, x, coo_nnz);
			// sort column/row/value triplets
		}
	}

	void CompressInplace() // throw(std::bad_alloc)
	{
		Sort(); // this function will guarantee sortedness (sort before checking if it is compressed)

		if(b_compressed)
			return; // another job well done

		stl_ut::CUniquePtr<size_t> col_counts(new size_t[n + 1]);
		size_t *p_new_p = col_counts.p_Get();
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
		std::fill_n(stdext::checked_array_iterator<size_t*>(p_new_p, n + 1), n + 1, size_t(0));
#else // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
		std::fill_n(p_new_p, n + 1, size_t(0));
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
		for(size_t j = 0, o = coo_nnz; j < o; ++ j)
			++ p_new_p[p[j]];
		stl_ut::ExclusiveScan(p_new_p, p_new_p + (n + 1));
		// count entries in each column, cumsum

#ifdef _DEBUG
		for(size_t j = 0; j < n; ++ j) {
			for(size_t k = p_new_p[j], p1 = p_new_p[j], p2 = p_new_p[j + 1]; k < p2; ++ k) {
				_ASSERTE(p[k] == j);
				_ASSERTE(k == p1 || i[k - 1] <= i[k]); // make sure the rows are also sorted
			}
		}
#endif // _DEBUG
		// make sure that the matrix entries are already compressed

		delete[] p;
		p = col_counts.p_YieldOwnership();
		// exchange the pointers

		b_compressed = true;
		coo_nnz = 0;
		// now it is compressed
	}

	void Compress(TSparse &r_dest) const // throw(std::bad_alloc)
	{
		if(b_compressed) {
			r_dest = *this;
			return; // another job well done
		}

		std::vector<size_t> col_counts(n, 0);
		for(size_t j = 0, o = coo_nnz; j < o; ++ j)
			++ col_counts[p[j]];
		// count entries in each column

		r_dest.Alloc(m, n, nnz_max, true, x != 0);
		// alloc destination

		if(n) {
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
			std::partial_sum(col_counts.begin(), col_counts.end() - 1,
				stdext::checked_array_iterator<size_t*>(r_dest.p + 2, n + 1 - 2));
#else // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
			std::partial_sum(col_counts.begin(), col_counts.end() - 1, r_dest.p + 2);
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
		}
		r_dest.p[1] = 0; // prefix with zero for exclusive sum
		// cumsum (exclusive, shifted to the right)

		r_dest.p[0] = 0;
		// the first pointer is always zero

		for(size_t j = 0, o = coo_nnz; j < o; ++ j) {
			size_t n_col = p[j];
			size_t n_idx = r_dest.p[n_col + 1];
			++ r_dest.p[n_col + 1];
			r_dest.i[n_idx] = i[j];
			if(x)
				r_dest.x[n_idx] = x[j];
		}
		// increment the column pointers shifted one right,
		// convert that exclusive sum to an inclusive one

		_ASSERTE(r_dest.p[n] == coo_nnz);
		// make sure we filled it all

		_ASSERTE(stl_ut::b_IsWeaklySorted(r_dest.p, r_dest.p + (n + 1)));
		// must also be ordered

		// note that if there are repeated triplets, this will repeat them also in the compressed form
		// also note that the output matrix is unsorted
	}

	TBmp *p_Rasterize(size_t n_max_bitmap_size = 640, bool b_symmetric = false) const
	{
		_ASSERTE(b_compressed);
		// the matrix needs to be in compressed column form

		const uint32_t n_background = 0xffffffffU,
			n_nonzero = 0xff8080ffU,
			n_nonzero_new = 0xffff0000U,
			n_zero_new = 0xff00ff00U;
		// colors of the individual areas

		size_t mb = m, nb = n;
		double f_scale = double(n_max_bitmap_size) / std::max(m, n);
		if(f_scale < 1) {
			mb = std::max(size_t(1), size_t(m * f_scale));
			nb = std::max(size_t(1), size_t(n * f_scale));
		} else
			f_scale = 1;
		// scale the bitmap down if needed

		if(mb == SIZE_MAX || nb == SIZE_MAX || mb > INT_MAX ||
		   nb > INT_MAX || uint64_t(mb) * nb > INT_MAX)
			return 0;

		TBmp *p_bitmap;

#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
		if(!(p_bitmap = TBmp::p_Alloc(int(mb), int(nb)))) // transposed
			return 0;
#else // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
		if(!(p_bitmap = TBmp::p_Alloc(int(nb), int(mb))))
			return 0;
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

		p_bitmap->Clear(n_background);

#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
#ifdef MATRIX_SUBSAMPLE_RASTERIZE_IN_PARALLEL
		if(n < INT_MAX && this->p[n] >= 10000) {
			const int _n = int(n);
			#pragma omp parallel for schedule(dynamic, 1)
			for(int n_col = 0; n_col < _n; ++ n_col) {
				for(size_t n_p0 = this->p[n_col], n_p1 = this->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
					size_t n_row = this->i[n_p0];
					double f_value = (this->x)? this->x[n_p0] : 1;
					// read the value

					if(f_value != 0)
						p_bitmap->PutPixel(int(n_row * f_scale), int(n_col * f_scale), n_nonzero);
					// draw a dot into the bitmap
				}
			}
		} else
#endif // MATRIX_SUBSAMPLE_RASTERIZE_IN_PARALLEL
		{
			for(size_t n_col = 0; n_col < n; ++ n_col) {
				for(size_t n_p0 = this->p[n_col], n_p1 = this->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
					size_t n_row = this->i[n_p0];
					const double f_one = 1, *p_a_it = (this->x)? this->x + n_p0 : &f_one;
#else // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
			{
				CSparseMatrixIterator p_a_it(this); // can use the simple iterator here
				for(; p_a_it.n_Column() < n; ++ p_a_it) {
					size_t n_col = p_a_it.n_Column();
					size_t n_row = p_a_it.n_Row();
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

					double f_value = *p_a_it;
					// read the value

					if(f_value != 0) {
#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
						size_t n_y = size_t(n_col * f_scale);
						size_t n_x = size_t(n_row * f_scale); // transposed
#else // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
						size_t n_x = size_t(n_col * f_scale);
						size_t n_y = size_t(n_row * f_scale);
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
						p_bitmap->PutPixel(int(n_x), int(n_y), n_nonzero);
						//if(b_symmetric && n_x != n_y)
						//	p_bitmap->PutPixel(int(n_y), int(n_x), n_nonzero);
					}
					// draw a square into the bitmap
				}
			}
		}

		if(b_symmetric) {
			uint32_t *p_buff = p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					uint32_t *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					uint32_t *p_lo = p_buff + (y + x * w);
					*p_up = std::min(*p_up, *p_lo); // choose the darker color
					*p_lo = *p_up; // duplicate the result to the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices

#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
		if((m != n || !b_symmetric) && !p_bitmap->Transpose()) {
			p_bitmap->Delete();
			return 0;
		}
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

		return p_bitmap;
	}

	// modifies p_bmp and returns the modified ptr (equal to the original ptr but the buffer will be reallocated)
	// never fails but if it runs out of memory, may look bad
	TBmp *p_Draw_BlockLayout(TBmp *p_bmp, const std::vector<size_t> &r_block_rows,
		const std::vector<size_t> &r_block_cols, int n_scale_up = 4)
	{
		{
			TBmp *p_large = p_bmp->p_Upscale(p_bmp->n_width * n_scale_up,
				p_bmp->n_height * n_scale_up, TBmp::filter_Nearest);
			if(p_large) {
				std::swap(*p_bmp, *p_large); // use the original pointer to the TBmp struct
				p_large->Delete();
			}
		}

		double f_scale = double(std::max(p_bmp->n_width, p_bmp->n_height)) / std::max(m, n);
		// scale the bitmap down if needed

		TBmp *p_add_one = p_bmp->p_Crop(0, 0, p_bmp->n_width + 1, p_bmp->n_height + 1, 0, 0, -1);
		if(p_add_one) {
			std::swap(*p_bmp, *p_add_one); // use the original pointer to the TBmp struct
			p_add_one->Delete();
		}
		// add one more row/column (after calculating the scale)

		printf("debug: block scale = %f / 256\n", f_scale * 256);
		if(f_scale >= 8.0f / 256) { // generates black frames if below, due to overdraw and rounding
			for(size_t j = 0, o = r_block_rows.size(); j < o; ++ j) {
				p_bmp->DrawLine_AA2(.0f, float(r_block_rows[j] * f_scale),
					float(p_bmp->n_width), float(r_block_rows[j] * f_scale),
					0xffff0000U, float(.5f * min(f_scale, 1.0)));
			}
			for(size_t j = 0, o = r_block_cols.size(); j < o; ++ j) {
				p_bmp->DrawLine_AA2(float(r_block_cols[j] * f_scale), .0f,
					float(r_block_cols[j] * f_scale), float(p_bmp->n_height),
					0xffff0000U, float(.5f * min(f_scale, 1.0)));
			}
		} else {
			float f_pad = 200, f_width = 111;
			p_bmp->DrawLine_AA2(f_pad, f_pad, float(p_bmp->n_width - f_pad),
				float(p_bmp->n_height - f_pad), 0xffff0000U, f_width, 2);
			p_bmp->DrawLine_AA2(float(p_bmp->n_width - f_pad), f_pad, f_pad,
				float(p_bmp->n_height - f_pad), 0xffff0000U, f_width, 2);
			// can't rasterize that (would lead to black frame) but still want to have the file
		}

		return p_bmp;
	}

	TBmp *p_Draw_BlockLayout(const std::vector<size_t> &r_block_rows,
		const std::vector<size_t> &r_block_cols, size_t n_max_bitmap_size = 640 * 4,
		bool b_symmetric = false, int n_scale_up = 4)
	{
		TBmp *p_bitmap;
		if(!(p_bitmap = p_Rasterize_AA(n_max_bitmap_size / n_scale_up, b_symmetric)))
			return 0;
		return p_Draw_BlockLayout(p_bitmap, r_block_rows, r_block_cols, n_scale_up);
	}

	TBmp *p_Rasterize_AA(size_t n_max_bitmap_size = 640, bool b_symmetric = false) const
	{
		_ASSERTE(b_compressed);
		// the matricx needs to be in compressed column form

		const uint32_t n_background = 0x00ffffffU,
			n_nonzero = 0xff8080ffU,
			n_nonzero_new = 0xffff0000U,
			n_zero_new = 0xff00ff00U;
		// colors of the individual areas

		size_t mb = m, nb = n;
		double f_scale = double(n_max_bitmap_size) / std::max(m, n);
		if(f_scale < 1) {
			mb = std::max(size_t(1), size_t(m * f_scale));
			nb = std::max(size_t(1), size_t(n * f_scale));
		} else
			f_scale = 1;
		// scale the bitmap down if needed

		if(mb == SIZE_MAX || nb == SIZE_MAX || mb > INT_MAX ||
		   nb > INT_MAX || uint64_t(mb) * nb > INT_MAX)
			return 0;

		TBmp *p_bitmap;

#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
		if(!(p_bitmap = TBmp::p_Alloc(int(mb), int(nb)))) // tranposed
			return 0;
#else // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
		if(!(p_bitmap = TBmp::p_Alloc(int(nb), int(mb))))
			return 0;
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

		_ASSERTE(sizeof(float) == sizeof(uint32_t));
		std::fill((float*)p_bitmap->p_buffer, ((float*)p_bitmap->p_buffer) + nb * mb, .0f);
		// use as a float buffer

		/*CSparseMatrixShapedIterator p_a_it(this, m, n);
		for(size_t n_col = 0; n_col < n; ++ n_col) {
			for(size_t n_row = 0; n_row < m; ++ n_row, ++ p_a_it) {
				_ASSERTE(n_row == p_a_it.n_Row() && n_col == p_a_it.n_Column());
				// make sure it iterates to the right position*/
#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
#ifdef MATRIX_SUBSAMPLE_RASTERIZE_IN_PARALLEL
		if(n > 16 && n < INT_MAX && this->p[n] >= 10000) {
			size_t n_half_stripe_num = nb / 8 - 1; // want four-pixel stripes with at least four pixel clearance (the blend will overflow up to two pixels - one on the left, one on the right from each stripe), want the last one to be wider than the others
			_ASSERTE(n_half_stripe_num < INT_MAX);
			const int _n_half_stripe_num = int(n_half_stripe_num);
			for(int n_pass = 0; n_pass < 2; ++ n_pass) { // draw even stripes in one pass, odd stripes in the next, this guarantees that two threads never blend over each other's area
				#pragma omp parallel for schedule(dynamic, 1)
				for(int n_stripe = 0; n_stripe < _n_half_stripe_num; ++ n_stripe) { // each thread processes a single stripe
					size_t n_stripe_b = (n / n_half_stripe_num) * n_stripe;
					size_t n_stripe_e = (n_stripe + 1 == _n_half_stripe_num)? n : n_stripe_b + n / n_half_stripe_num;
					_ASSERTE(n_stripe_e >= n_stripe_b + 8);
					if(n_pass)
						n_stripe_b = (n_stripe_b + n_stripe_e) / 2;
					else
						n_stripe_e = (n_stripe_b + n_stripe_e) / 2;
					_ASSERTE(n_stripe_e >= n_stripe_b + 2);
					// begin and end of a stripe

					for(size_t n_col = n_stripe_b; n_col < n_stripe_e; ++ n_col) {
						for(size_t n_p0 = this->p[n_col], n_p1 = this->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
							size_t n_row = this->i[n_p0];
							double f_value = (this->x)? this->x[n_p0] : 1;
							// read the value

							if(f_value != 0)
								PutPixel_AA_Float(p_bitmap, float(n_row * f_scale), float(n_col * f_scale));
							// draw a dot into the bitmap
						}
					}
				}
				// implicit thread synchronization here
			}
		} else
#endif // MATRIX_SUBSAMPLE_RASTERIZE_IN_PARALLEL
		{
			for(size_t n_col = 0; n_col < n; ++ n_col) {
				for(size_t n_p0 = this->p[n_col], n_p1 = this->p[n_col + 1]; n_p0 != n_p1; ++ n_p0) {
					size_t n_row = this->i[n_p0];
					const double f_one = 1, *p_a_it = (this->x)? this->x + n_p0 : &f_one;
#else // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
			{
				CSparseMatrixIterator p_a_it(this); // can use the simple iterator here
				for(; p_a_it.n_Column() < n; ++ p_a_it) {
					size_t n_col = p_a_it.n_Column();
					size_t n_row = p_a_it.n_Row();
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

					double f_value = *p_a_it;
					// read the value

					if(f_value != 0) {
#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
						float f_y = float(n_col * f_scale);
						float f_x = float(n_row * f_scale); // tranposed
#else // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
						float f_x = float(n_col * f_scale);
						float f_y = float(n_row * f_scale);
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
						PutPixel_AA_Float(p_bitmap, /*->PutPixel_AA(*/f_x, f_y/*, n_nonzero*/);
						//if(b_symmetric && f_x != f_y)
						//	PutPixel_AA_Float(p_bitmap, /*->PutPixel_AA(*/f_y, f_x/*, n_nonzero*/);
					}
					// draw a square into the bitmap
				}
			}
		}

		// alpha = 0
		// alpha = 0 + a1 * (1 - 0) = a1
		// alpha = a1 + a2 * (1 - a1) = a1 + a2 - a1a2
		//
		// alpha = 0
		// alpha = 0 + a2 * (1 - 0) = a2
		// alpha = a2 + a1 * (1 - a2) = a2 + a1 - a1a2
		// the order of blending does not make a difference

		if(b_symmetric) {
			float *p_buff = (float*)p_bitmap->p_buffer;
			for(size_t x = 0, w = p_bitmap->n_width, h = p_bitmap->n_height, s = std::min(w, h); x < s; ++ x) {
				for(size_t y = 0; y < x; ++ y) { // only the upper triangle
					_ASSERTE(x < w && y < h);
					float *p_up = p_buff + (x + w * y);
					_ASSERTE(y < w && x < h);
					float *p_lo = p_buff + (y + x * w);
					AlphaBlend_Float(*p_up, 1, *p_lo); // blend the two alphas
					*p_lo = *p_up; // duplicate the result in the other triangle as well
				}
			}
		}
		// if symmetric, transpose the raster to save half the time on very large matrices

		for(float *p_buffer = (float*)p_bitmap->p_buffer, *p_end =
		   p_buffer + nb * mb; p_buffer != p_end; ++ p_buffer) {
			float f_alpha = *p_buffer;
			int n_alpha = std::min(255, std::max(0, int(255 * f_alpha)));
			*(uint32_t*)p_buffer = TBmp::n_Modulate(n_background, 255 - n_alpha) +
				TBmp::n_Modulate(n_nonzero, n_alpha); // integer blend should be enough now, only once per pixel
		}
		// convert float buffer to RGBA

#ifdef MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS
		if((m != n || !b_symmetric) && !p_bitmap->Transpose()) {
			p_bitmap->Delete();
			return 0;
		}
#endif // MATRIX_SUBSAMPLE_RASTERIZE_ROW_MAJOR_ACCESS

		return p_bitmap;
	}

	struct TMMHeader {
		bool b_symmetric, b_binary;
		size_t n_rows, n_cols, n_nnz; // n_nnz contains the number of *stored* nonzeros (about a half if the matrix is symmetric)
		size_t n_full_nnz; // equal to n_nnz if not symmetric, or the about twice that amount (all that are in the full matrix)

		uint64_t n_AllocationSize_CSC(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
		{
			uint64_t n_size_cols = min((min(uint64_t(n_cols),
				UINT64_MAX - 1) + 1), UINT64_MAX / n_idx_size) * n_idx_size;
			uint64_t n_size_rows = min(uint64_t(n_full_nnz), UINT64_MAX / n_idx_size) * n_idx_size;
			uint64_t n_size_data = 0;
			if(!b_binary)
				n_size_data = min(uint64_t(n_full_nnz), UINT64_MAX / n_scalar_size) * n_scalar_size;
			uint64_t n_size = 0;
			n_size = min(n_size, UINT64_MAX - n_size_cols) + n_size_cols;
			n_size = min(n_size, UINT64_MAX - n_size_rows) + n_size_rows;
			n_size = min(n_size, UINT64_MAX - n_size_data) + n_size_data;
			// calculate size (with saturation arithmetics)

			return n_size;
		}

		uint64_t n_AllocationSize_COO(size_t n_idx_size = sizeof(size_t), size_t n_scalar_size = sizeof(double)) const
		{
			size_t n_triplet_size = (b_binary)? 2 * n_idx_size : 2 * n_idx_size + n_scalar_size;
			return min(uint64_t(n_full_nnz), UINT64_MAX / n_triplet_size) * n_triplet_size;
		}
	};

	static bool Peek_MatrixMarket_Header(const char *p_s_filename, TMMHeader &r_t_header)
	{
		bool b_symmetric = false, b_binary = false;
		bool b_had_specifier = false;
		bool b_had_header = false;
		size_t n_rows, n_cols, n_nnz = -1, n_read_nnz = 0, n_full_nnz = 0;

		{
			CUniqueFILE_Ptr p_fr;
			if(!p_fr.Open(p_s_filename, "r"))
				return false;
			// open the matrix market file

			std::string s_line;
			while(!feof(p_fr)) {
				if(!stl_ut::ReadLine(s_line, p_fr))
					return false;
				// read a single line

				size_t n_pos;
				if(!b_had_specifier && (n_pos = s_line.find("%%MatrixMarket")) != std::string::npos) {
					s_line.erase(0, n_pos + strlen("%%MatrixMarket"));
					// get rid of header

					b_binary = s_line.find("pattern") != std::string::npos;
					if(s_line.find("matrix") == std::string::npos ||
					   s_line.find("coordinate") == std::string::npos ||
					   (s_line.find("real") == std::string::npos &&
					   s_line.find("integer") == std::string::npos &&
					   !b_binary)) // integer matrices are not real, but have values and are loadable
						return false;
					// must be matrix coordinate real

					if(s_line.find("general") != std::string::npos)
						b_symmetric = false;
					else if(s_line.find("symmetric") != std::string::npos)
						b_symmetric = true;
					else {
						b_symmetric = false;
						/*//fclose(p_fr);
						return false;*/ // or assume general
					}
					// either general or symmetric

					b_had_specifier = true;
					continue;
				}
				if((n_pos = s_line.find('%')) != std::string::npos)
					s_line.erase(n_pos);
				stl_ut::TrimSpace(s_line);
				if(s_line.empty())
					continue;
				// trim comments, skip empty lines

				if(!b_had_header) {
					if(!b_had_specifier)
						return false;
					// specifier must come before header

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
					if(sscanf_s(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
					   &n_rows, &n_cols, &n_nnz) != 3) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
					   &n_rows, &n_cols, &n_nnz) != 3) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
						return false;
					}
					// read header

					if(n_rows <= SIZE_MAX / max(size_t(1), n_cols) && n_nnz > n_rows * n_cols)
						return false;
					// sanity check (may also fail on big matrices)

					b_had_header = true;
					break;
				}
			}
			if(!b_had_header)
				return false;
			if(b_symmetric) {
				_ASSERTE(b_had_header && b_had_specifier);
				while(!feof(p_fr)) { // only elements follow
					if(!stl_ut::ReadLine(s_line, p_fr))
						return false;
					// read a single line

					stl_ut::TrimSpace(s_line);
					if(s_line.empty() || s_line[0] == '%') // only handles comments at the beginning of the line; comments at the end will simply be ignored 
						continue;
					// trim comments, skip empty lines

					double f_value;
					size_t n_rowi, n_coli;
					do {
						const char *b = s_line.c_str();
						_ASSERTE(*b && !isspace(uint8_t(*b))); // not empty and not space
						for(n_rowi = 0; *b && isdigit(uint8_t(*b)); ++ b) // ignores overflows
							n_rowi = 10 * n_rowi + *b - '0';
						// parse the first number

						if(!*b || !isspace(uint8_t(*b)))
							return false;
						++ b; // skip the first space
						while(*b && isspace(uint8_t(*b)))
							++ b;
						if(!*b || !isdigit(uint8_t(*b)))
							return false;
						// skip space to the second number

						for(n_coli = 0; *b && isdigit(uint8_t(*b)); ++ b) // ignores overflows
							n_coli = 10 * n_coli + *b - '0';
						// parse the second number

						if(!*b) {
							f_value = 1; // a binary matrix?
							break;
						}
						if(!isspace(uint8_t(*b)))
							return false; // bad character
						++ b; // skip the first space
						while(*b && isspace(uint8_t(*b)))
							++ b;
						_ASSERTE(*b); // there must be something since the string sure does not end with space (called TrimSpace() above)
						// skip space to the value

						f_value = 1;//atof(b); // don't care about the values here
					} while(0);
					// read a triplet

					-- n_rowi;
					-- n_coli;
					// indices are 1-based, i.e. a(1,1) is the first element

					if(n_rowi >= n_rows || n_coli >= n_cols || n_read_nnz >= n_nnz)
						return false;
					// make sure it figures

					_ASSERTE(b_symmetric);
					n_full_nnz += (n_rowi != n_coli)? 2 : 1;
					++ n_read_nnz;
					// append the nonzero
				}
				// read the file line by line

				if(ferror(p_fr) || !b_had_header || n_read_nnz != n_nnz)
					return false;
				// make sure no i/o errors occurred, and that the entire matrix was read
			}
			// in case the matrix is symmetric, need to read the indices to see which ones are at the
			// diagonal; does not actually store them, just goes through them quickly; the floats are
			// not parsed
		}

		r_t_header.b_binary = b_binary;
		r_t_header.b_symmetric = b_symmetric;
		r_t_header.n_nnz = n_nnz;
		r_t_header.n_rows = n_rows;
		r_t_header.n_cols = n_cols;

		if(!b_symmetric)
			r_t_header.n_full_nnz = n_nnz;
		else
			r_t_header.n_full_nnz = n_full_nnz;

		return true;
	}

	static uint64_t n_Peek_MatrixMarket_SizeInMemory(const char *p_s_filename)
	{
		TMMHeader t_header;
		if(!Peek_MatrixMarket_Header(p_s_filename, t_header))
			return -1;

		return t_header.n_AllocationSize_CSC(sizeof(*TSparse().p), sizeof(*TSparse().x));
	}

	bool Load_MatrixMarket(const char *p_s_filename) // throw(std::bad_alloc)
	{
		CUniqueFILE_Ptr p_fr;
		if(!p_fr.Open(p_s_filename, "r"))
			return false;
		// open the matrix market file

		TSparse t_dest;

		bool b_symmetric = false, b_binary = false;
		bool b_had_specifier = false;
		bool b_had_header = false;
		size_t n_rows, n_cols, n_nnz = -1, n_read_nnz = 0;
		std::string s_line;
		while(!feof(p_fr)) {
			if(!stl_ut::ReadLine(s_line, p_fr))
				return false;
			// read a single line

			size_t n_pos;
			if(!b_had_specifier && (n_pos = s_line.find("%%MatrixMarket")) != std::string::npos) {
				s_line.erase(0, n_pos + strlen("%%MatrixMarket"));
				// get rid of header

				b_binary = s_line.find("pattern") != std::string::npos;
				if(s_line.find("matrix") == std::string::npos ||
				   s_line.find("coordinate") == std::string::npos ||
				   (s_line.find("real") == std::string::npos &&
				   s_line.find("integer") == std::string::npos &&
				   !b_binary)) // integer matrices are not real, but have values and are loadable
					return false;
				// must be matrix coordinate real

				if(s_line.find("general") != std::string::npos)
					b_symmetric = false;
				else if(s_line.find("symmetric") != std::string::npos)
					b_symmetric = true;
				else {
					b_symmetric = false;
					/*//fclose(p_fr);
					return false;*/ // or assume general
				}
				// either general or symmetric

				b_had_specifier = true;
				continue;
			}
			if((n_pos = s_line.find('%')) != std::string::npos)
				s_line.erase(n_pos);
			stl_ut::TrimSpace(s_line);
			if(s_line.empty())
				continue;
			// trim comments, skip empty lines

			if(!b_had_header) {
				if(!b_had_specifier)
					return false;
				// specifier must come before header

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				if(sscanf_s(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
				   &n_rows, &n_cols, &n_nnz) != 3) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(sscanf(s_line.c_str(), PRIsize " " PRIsize " " PRIsize,
				   &n_rows, &n_cols, &n_nnz) != 3) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					return false;
				}
				// read header

				if(n_rows <= SIZE_MAX / max(size_t(1), n_cols) && n_nnz > n_rows * n_cols)
					return false;
				// sanity check (may also fail on big matrices)

				t_dest.Alloc(n_rows, n_cols, n_nnz, false, b_binary);
				// alloc (erases the old data)

				b_had_header = true;
				break;
			}
		}
		if(!b_had_header)
			return false;
		_ASSERTE(b_had_header && b_had_specifier);
		while(!feof(p_fr)) { // only elements follow
			if(!stl_ut::ReadLine(s_line, p_fr))
				return false;
			// read a single line

			stl_ut::TrimSpace(s_line);
			if(s_line.empty() || s_line[0] == '%') // only handles comments at the beginning of the line; comments at the end will simply be ignored 
				continue;
			// trim comments, skip empty lines

			double f_value;
			size_t n_rowi, n_coli;
#if 0
			const char *b = s_line.c_str();
			n_rowi = strtoul(b, const_cast<char**>(&b), 10);
			n_coli = strtoul(b, const_cast<char**>(&b), 10); // returns 0 or ULONG_MAX on error, that is enough to fail the dimension checks below
			if(*b && isspace(uint8_t(*b))) // no space at the end of the line
				f_value = strtod(b, const_cast<char**>(&b));
			else
				f_value = 1;
			// processing matrix 907: vanHeukelum / cage15 it took 253.688920 to load
#elif 1
			const char *b = s_line.c_str();
			_ASSERTE(*b && !isspace(uint8_t(*b))); // not empty and not space
			for(n_rowi = 0; *b && isdigit(uint8_t(*b)); ++ b) // ignores overflows
				n_rowi = 10 * n_rowi + *b - '0';
			// parse the first number

			if(!*b || !isspace(uint8_t(*b)))
				return false;
			++ b; // skip the first space
			while(*b && isspace(uint8_t(*b)))
				++ b;
			if(!*b || !isdigit(uint8_t(*b)))
				return false;
			// skip space to the second number

			for(n_coli = 0; *b && isdigit(uint8_t(*b)); ++ b) // ignores overflows
				n_coli = 10 * n_coli + *b - '0';
			// parse the second number

			do {
				if(!*b) {
					f_value = 1; // a binary matrix?
					break;
				}
				if(!isspace(uint8_t(*b)))
					return false; // bad character
				++ b; // skip the first space
				while(*b && isspace(uint8_t(*b)))
					++ b;
				_ASSERTE(*b); // there must be something since the string sure does not end with space (called TrimSpace() above)
				// skip space to the value
/*
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				int n = sscanf_s(b, "%lf", &f_value);
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				int n = sscanf(b, "%lf", &f_value);
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(n != 1)
					return false; // failed to parse the float*/
				// processing matrix 907: vanHeukelum / cage15 it took 272.915189 to load

				f_value = atof(b); // processing matrix 907: vanHeukelum / cage15 it took 245.053845 to load
			} while(0);
#else // 1
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
			int n = sscanf_s(s_line.c_str(), PRIsize " " PRIsize " %lf", &n_rowi, &n_coli, &f_value);
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			int n = sscanf(s_line.c_str(), PRIsize " " PRIsize " %lf", &n_rowi, &n_coli, &f_value);
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			if(n < 2) {
				return false;
			} else if(n == 2)
				f_value = 1; // a binary matrix
			// processing matrix 907: vanHeukelum / cage15 it took 331.314893 to load
#endif
			// read a triplet

			-- n_rowi;
			-- n_coli;
			// indices are 1-based, i.e. a(1,1) is the first element

			if(n_rowi >= n_rows || n_coli >= n_cols || n_read_nnz >= n_nnz)
				return false;
			// make sure it figures

			for(int n_pass = 0;; ++ n_pass) {
				t_dest.Entry(n_rowi, n_coli, f_value);
				// add entry

				if(b_symmetric && !n_pass && n_rowi != n_coli)
					std::swap(n_rowi, n_coli);
				else
					break;
				// handle symmetric matrices
			}
			++ n_read_nnz;
			// append the nonzero
		}
		// read the file line by line

		if(ferror(p_fr) || !b_had_header || n_read_nnz != n_nnz)
			return false;
		// make sure no i/o errors occurred, and that the entire matrix was read

#ifdef _DEBUG
		size_t n_elem_size = min(n_read_nnz, SIZE_MAX / ((b_binary)? sizeof(size_t) :
			sizeof(size_t) + sizeof(double))) * ((b_binary)? sizeof(size_t) : sizeof(size_t) + sizeof(double));
		size_t n_cptr_size = min(min(n_cols, SIZE_MAX - 1) + 1, SIZE_MAX / sizeof(size_t)) * sizeof(size_t);
		size_t n_mat_size = min(n_elem_size, SIZE_MAX - n_cptr_size) + n_cptr_size;
		TSparse t_debug;
		if(n_mat_size < 5 * 1048576) {
			t_dest.Compress(t_debug);
			t_debug.Sort();
		}
#endif // _DEBUG
		t_dest.CompressInplace(); // guaranteed sorted, requires 3n + 1 workspace
#ifdef _DEBUG
		if(n_mat_size < 5 * 1048576) {
			_ASSERTE(t_dest.b_compressed && t_debug.b_compressed);
			_ASSERTE(t_dest.m == t_debug.m && t_dest.n == t_debug.n);
			_ASSERTE(!t_dest.coo_nnz && !t_debug.coo_nnz);
			_ASSERTE(t_dest.p[t_dest.n] == t_debug.p[t_debug.n]);
			_ASSERTE(!memcmp(t_dest.p, t_debug.p, t_debug.n * sizeof(size_t)));
			_ASSERTE(!memcmp(t_dest.i, t_debug.i, t_debug.p[t_debug.n] * sizeof(size_t)));
			_ASSERTE(!t_dest.x == !t_debug.x);
			_ASSERTE((!t_dest.x ||/*&&*/ !t_debug.x) ||
				!memcmp(t_dest.x, t_debug.x, t_debug.p[t_debug.n] * sizeof(size_t)));
		}
#endif // _DEBUG
		// compress, make sure it comes out sorted

		t_dest.Swap(*this);
		// overwrite this matrix, only if everything goes well

		return true;
	}

	// todo - reinterpret as cs*

	// utility functions below

	static inline void AlphaBlend_Float(float &r_f_dest, float f_src, float f_alpha)
	{
		//r_f_dest = (r_f_dest * (1 - f_alpha)) + f_src * f_alpha;
		//r_f_dest = r_f_dest * 1 + f_src * f_alpha - r_f_dest * f_alpha;
		//r_f_dest = r_f_dest + (f_src - r_f_dest) * f_alpha;
		r_f_dest = r_f_dest + f_alpha * (f_src - r_f_dest);
	}

	static inline void PutPixel_AA_Float(TBmp *p_bitmap, float f_x, float f_y) // this is likely quite slow
	{
		_ASSERTE(sizeof(float) == sizeof(uint32_t));
		float *p_buffer = (float*)p_bitmap->p_buffer;

		const int n_width = p_bitmap->n_width;
		const int n_height = p_bitmap->n_height;
		int x = int(floor(f_x)), y = int(floor(f_y));
		float f_frac_x = f_x - x;
		float f_frac_y = f_y - y;
		const float f_alpha = 1; // alpha of the incoming pixel
		float f_weight_00 = (f_alpha * (1 - f_frac_x) * (1 - f_frac_y));
		float f_weight_10 = (f_alpha * f_frac_x * (1 - f_frac_y));
		float f_weight_01 = (f_alpha * (1 - f_frac_x) * f_frac_y);
		float f_weight_11 = (f_alpha * f_frac_x * f_frac_y);
		if(x >= 0 && x < n_width && y >= 0 && y < n_height)
			AlphaBlend_Float(p_buffer[x + n_width * y], 1, f_weight_00);
		if(x + 1 >= 0 && x + 1 < n_width && y >= 0 && y < n_height)
			AlphaBlend_Float(p_buffer[(x + 1) + n_width * y], 1, f_weight_10);
		if(x >= 0 && x < n_width && y + 1 >= 0 && y + 1 < n_height)
			AlphaBlend_Float(p_buffer[x + n_width * (y + 1)], 1, f_weight_01);
		if(x + 1 >= 0 && x + 1 < n_width && y + 1 >= 0 && y + 1 < n_height)
			AlphaBlend_Float(p_buffer[(x + 1) + n_width * (y + 1)], 1, f_weight_11);
	}

	static inline void PutPixel_Max(TBmp *p_bitmap, int x, int y, uint32_t n_color)
	{
		uint32_t *p_buffer = p_bitmap->p_buffer;
		const int n_width = p_bitmap->n_width;
		const int n_height = p_bitmap->n_height;
		if(x >= 0 && x < n_width && y >= 0 && y < n_height) {
			uint32_t &r_n_pixel = p_buffer[int(x) + n_width * int(y)];
			if((r_n_pixel & 0xffffff) < (n_color & 0xffffff))
				r_n_pixel = n_color;
		}
	}

	static void Pair_Sort(size_t *i, double *x, size_t n, std::vector<size_t> &r_workspace) // the first array forms keys, the second is sorted alongside with it
	{
		if(x) {
			/*r_workspace.resize(2 * n);
			stl_ut::IOTA(r_workspace.begin(), r_workspace.begin() + n, size_t(0));

			std::sort(r_workspace.begin(), r_workspace.begin() + n,
				stl_ut::CCompare_Indirect<size_t>(i, n));
			// generate a permutation

			size_t *p_indices_inv = &r_workspace[n];
			for(size_t j = 0; j < n; ++ j)
				p_indices_inv[r_workspace[j]] = j;
			// need to invert it :(

			for(size_t j = 0; j < n; ++ j) {
				for(size_t k; (k = p_indices_inv[j]) != j;) {
					std::swap(i[j], i[k]);
					std::swap(x[j], x[k]);
					std::swap(p_indices_inv[j], p_indices_inv[k]);
				}
			}
			// permute inplace (destroys the permutation in the process)*/
			// requires 2n workspace if x != 0 (could be avoided by implementing a triplet zip iterator)
			
			std::sort(stl_ut::p_Make_PairIterator(i, x), stl_ut::p_Make_PairIterator(i, x) + n,
				stl_ut::CCompareFirst());
			// use a fancy pair iterator to sort inplace, compare only indices and not values
		} else
			std::sort(i, i + n);
		// sort pairs
	}

	static void Triplet_Sort(size_t *p, size_t *i, double *x, size_t n) // the first two arrays form keys
	{
		if(x) {
#if !defined(_MSC_VER) || defined(__MWERKS__) || _MSC_VER > 1200
			std::sort(stl_ut::p_Make_PairIterator(stl_ut::p_Make_PairIterator(p, i), x),
				stl_ut::p_Make_PairIterator(stl_ut::p_Make_PairIterator(p, i), x) + n, stl_ut::CCompareFirst());
			// first is (p, i) and second is x ... so compare only first and Bob's your uncle

#else // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200
			// below code requires 2n workspace (could be avoided by implementing a triplet zip iterator)
			std::vector<size_t> indices(n);
			stl_ut::IOTA(indices.begin(), indices.end(), size_t(0));

			std::sort(indices.begin(), indices.end(), stl_ut::CComparePair_Indirect<size_t, size_t>(p, i, n));
			// generate a permutation

			{
				std::vector<size_t> indices_inv(n);
				for(size_t j = 0; j < n; ++ j)
					indices_inv[indices[j]] = j;
				indices.swap(indices_inv);
			}
			// need to invert it :( but still saving some space if x != 0

			if(x) {
				for(size_t j = 0; j < n; ++ j) {
					for(size_t k; (k = indices[j]) != j;) {
						std::swap(p[j], p[k]);
						std::swap(i[j], i[k]);
						std::swap(x[j], x[k]);
						std::swap(indices[j], indices[k]);
					}
				}
			} else { // dead code
				for(size_t j = 0; j < n; ++ j) {
					for(size_t k; (k = indices[j]) != j;) {
						std::swap(p[j], p[k]);
						std::swap(i[j], i[k]);
						std::swap(indices[j], indices[k]);
					}
				}
			}
			// permute inplace (destroys the permutation in the process)
#endif // !_MSC_VER || __MWERKS__ || _MSC_VER > 1200
		} else {
			std::sort(stl_ut::p_Make_PairIterator(p, i), stl_ut::p_Make_PairIterator(p, i) + n);
			// use a fancy pair iterator to sort inplace
		}
	}
};
