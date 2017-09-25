/*
								+--------------------------------+
								|                                |
								| *** Block matrix benchmark *** |
								|                                |
								| Copyright (c) -tHE SWINe- 2012 |
								|                                |
								|          BlockBench.h          |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
#define __UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED

/**
 *	@file include/slam_app/BlockBench.h
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief block matrix benchmark
 *
 *	@date 2012-08-27
 *
 *	Fixed the side-effect bug in matrix multiplication code (was multiplying (2A)'*(A+A')
 *	rather than A'*A). Now all the benchmark results match matlab and themselves.
 *
 *	@date 2012-08-28
 *
 *	Added more functionality to CBlockBench, most importantly writing
 *	results in .csv files for performance analysis.
 *
 *	@date 2012-09-05
 *
 *	Moved unit tests from CBlockMatrixBenchmark to new CBlockMatrixUnitTests.
 *
 *	@date 2013-01-13
 *
 *	Added the __BLOCK_BENCH_G2O ifdef to speed the tests up.
 *
 *	Improved precision of saved times by using the "%lg" format for number smaller than one.
 *
 */

#include "slam/Parser.h" // uses CParserBase::ReadLine() and CParserBase::TrimSpace()
#include "slam/Timer.h"
#include "slam/BlockMatrix.h"

/**
 *	@def __BLOCK_BENCH_DUMP_MATRIX_IMAGES
 *	@brief if defined, some of the benchmarked matrices are written as images (for debugging purposes)
 */
//#define __BLOCK_BENCH_DUMP_MATRIX_IMAGES

/**
 *	@def __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
 *	@brief if defined, some of the benchmarked matrices are written in matlab format (for debugging purposes)
 */
//#define __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

/**
 *	@def __BLOCK_BENCH_GTSAM
 *	@brief if defined, GTSAM's block matrix is also included in tests (not a true block matrix)
 */
//#define __BLOCK_BENCH_GTSAM

/**
 *	@def __BLOCK_BENCH_BLOCK_TYPE_A
 *	@brief if defined, block matrices are built from sparse matrices by dividing matrix area
 *		in blocks and writing any nonzero elements to blocks to where these fall (size of matrix
 *		doesn't change, size of data changes only slightly, depending on sparsity pattern in the
 *		given matrix); the alternative is to replace each nonzero element by a block (size of
 *		matrix changes with size of blocks)
 */
#define __BLOCK_BENCH_BLOCK_TYPE_A

/**
 *	@def __BLOCK_BENCH_CHOLESKY_USE_AMD
 *	@brief if defined, the cholesky benchmark uses the AMD ordering
 */
#define __BLOCK_BENCH_CHOLESKY_USE_AMD

/**
 *	@def __BLOCK_BENCH_CERES
 *	@brief if defined, ceres solver is expected to be included in the build process
 */
//#define __BLOCK_BENCH_CERES

/**
 *	@def __BLOCK_BENCH_G2O
 *	@brief if defined, g2o solver is expected to be included in the build process
 */
//#define __BLOCK_BENCH_G2O

#ifdef __BLOCK_BENCH_G2O
#include "g2o/core/sparse_block_matrix.h"
#endif // __BLOCK_BENCH_G2O

#if defined(_WIN32) || defined(_WIN64)
#if !(defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64))
#ifdef __BLOCK_BENCH_GTSAM
#include <gtsam/base/blockMatrices.h>
#endif // __BLOCK_BENCH_GTSAM
#endif // !(_M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64)
#endif // _WIN32 || _WIN64

#ifdef __BLOCK_BENCH_NISTBLAS
#include "spblas.h"
#include "dbscmml.h"
#endif // __BLOCK_BENCH_NISTBLAS
// test NIST BLAS

#ifdef __BLOCK_BENCH_CERES
#include "ceres/block_sparse_matrix.h"
#endif // __BLOCK_BENCH_CERES
// test Ceres

/**
 *	@brief class, implementing general timing benchmarks and benchmarks,
 *		specialized for block matrices
 */
class CBlockMatrixBenchmark {
protected:
	size_t m_n_pass_num; /**< @brief number of test passes */

	/**
	 *	@brief triplet record
	 */
	struct TTrip {
		size_t n_row; /**< @brief zero-based row index */
		size_t n_col; /**< @brief zero-based column index */
		double f_value; /**< @brief value of the element */
	};

	/**
	 *	@brief matrix in triplet format, as read from .mat file
	 */
	struct TRawMatrix {
		size_t n_row_num; /**< @brief numer of rows */
		size_t n_col_num; /**< @brief numer of columns */
		std::vector<TTrip> data; /**< @brief list of triplet records */
	};

	/**
	 *	@brief raw timing information
	 */
	struct TTimingInfo {
		std::vector<double> sample_list; /**< @brief list of sampled times */
	};

	/**
	 *	@brief cumulative timing information
	 */
	struct TCumTimingInfo {
		double f_span_start; /**< @brief time the last run was started */
		bool b_span_started; /**< @brief span started flag */
		double f_time_total; /**< @brief total cumulative time */
		size_t n_sample_num; /**< @brief number of samples */
	};

	/**
	 *	@brief null-terminated string comparison (for std::map)
	 */
	class CLessCStr {
	public:
		/**
		 *	@brief compares two null-terminated strings
		 *
		 *	@param[in] p_s_a is a null-terminated string
		 *	@param[in] p_s_b is a null-terminated string
		 *
		 *	@return Returns true if p_s_a < p_s_b (lexicographically), otherwise returns false.
		 */
		inline bool operator ()(const char *p_s_a, const char *p_s_b) const
		{
			return strcmp(p_s_a, p_s_b) < 0;
		}
	};

	typedef std::map<const char*, TCumTimingInfo, CLessCStr> CCumTimerMap; /**< @brief map of cumulative timers */
	typedef std::map<const char*, TTimingInfo, CLessCStr> CTimerMap; /**< @brief map of sample-based timers */
	typedef std::map<const char*, double, CLessCStr> CPropsMap; /**< @brief map of test run properties */
	typedef std::set<const char*, CLessCStr> CNameSet; /**< @brief set of field names (timer and property names) */

	/**
	 *	@brief one test run
	 */
	struct TTestRun {
		std::string s_matrix_name; /**< @brief matrix name */
		CTimerMap timer_samples; /**< @brief timer samples */
		CPropsMap quantitative_properties; /**< @brief test / data parameters */
	};

	/**
	 *	@brief ordering function for triplet entries, forms triangle frontline
	 */
	class CCompareTriangleFrontline { // t_odo - test this
	public:
		/**
		 *	@brief compares two triplet entries
		 *
		 *	@param[in] r_a is a triplet entry
		 *	@param[in] r_b is a triplet entry
		 *
		 *	@return Returns true if r_a < r_b, otherwise returns false.
		 */
		inline bool operator ()(const std::pair<size_t, size_t> &r_a,
			const std::pair<size_t, size_t> &r_b) const
		{
			size_t n_min_a = (r_a.first + r_a.second);
			size_t n_min_b = (r_b.first + r_b.second);
			return (n_min_a < n_min_b) || (n_min_a == n_min_b && (r_a.first < r_b.first ||
				(r_a.first == r_b.first && r_a.second < r_b.second)));
		}
	};

	/**
	 *	@brief ordering function for triplet entries, forms rectangular frontline
	 */
	class CCompareRectFrontline { // t_odo - test this
	public:
		/**
		 *	@brief compares two triplet entries
		 *
		 *	@param[in] r_a is a triplet entry
		 *	@param[in] r_b is a triplet entry
		 *
		 *	@return Returns true if r_a < r_b, otherwise returns false.
		 */
		inline bool operator ()(const std::pair<size_t, size_t> &r_a,
			const std::pair<size_t, size_t> &r_b) const
		{
			size_t n_min_a = std::max(r_a.first, r_a.second);
			size_t n_min_b = std::max(r_b.first, r_b.second);
			return (n_min_a < n_min_b) || (n_min_a == n_min_b && (r_a.second < r_b.second ||
				(r_a.second == r_b.second && r_a.first < r_b.first)));
		}
	};

	/**
	 *	@brief ordering function for triplet entries, orders by columns
	 */
	class CComparePairByFirst { // t_odo - write comparator that would create rectangular frontline, appending the matrix in rectangular manner
	public:
		/**
		 *	@brief compares two triplet entries
		 *
		 *	@param[in] r_a is a triplet entry
		 *	@param[in] r_b is a triplet entry
		 *
		 *	@return Returns true if r_a < r_b, otherwise returns false.
		 */
		inline bool operator ()(const std::pair<size_t, size_t> &r_a,
			const std::pair<size_t, size_t> &r_b) const
		{
			return (r_a.first < r_b.first) || (r_a.first == r_b.first && r_a.second < r_b.second);
		}
	};

	/**
	 *	@brief ordering function for triplet entries, orders by rows
	 */
	class CComparePairBySecond {
	public:
		/**
		 *	@brief compares two triplet entries
		 *
		 *	@param[in] r_a is a triplet entry
		 *	@param[in] r_b is a triplet entry
		 *
		 *	@return Returns true if r_a < r_b, otherwise returns false.
		 */
		inline bool operator ()(const std::pair<size_t, size_t> &r_a,
			const std::pair<size_t, size_t> &r_b) const
		{
			return (r_a.second < r_b.second) || (r_a.second == r_b.second && r_a.first < r_b.first);
		}
	};

	CTimer m_timer; /**< @brief timer object */
	CCumTimerMap m_timer_map; /**< @brief map of cumulative timers */
	CNameSet m_label_set; /**< @brief set of timer labels */
	CNameSet m_props_label_set; /**< @brief set of properties labels */
	std::vector<TTestRun> m_test_list; /**< @brief list of tests with samples for analysis */
	bool m_b_test_active; /**< @brief test activity flag (if set, the last entry in m_test_list is the active test) */

	
	/**
	 *	@brief matrix factorization benchmarks
	 *	@tparam COrderingType is ordering function for triplet entries
	 *	@tparam n_block_size is matrix block size for the given benchmark
	 */
	template <const int n_block_size, class COrderingType>
	class CBenchmarkCholesky {
	public:
		/**
		 *	@brief configuration enums
		 */
		enum {
			block_Size = n_block_size /**< @brief preset block size (const at compile-time) */
		};

		/**
		 *	@brief block matrix type (specifically avoids AutoAlign under x86)
		 */
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		typedef Eigen::Matrix<double, block_Size, block_Size> TBlockMatrix;
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		typedef Eigen::Matrix<double, block_Size, block_Size, Eigen::DontAlign> TBlockMatrix; // AutoAlign sometimes aligns matrices (if block_Size is power of two?) and then bat shit happens under x86
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

	protected:
		CBlockMatrixBenchmark *m_p_parent; /**< @brief pointer to the instance of outer class (contains the timers) */
		size_t m_n_matrix_id; /**< @brief matrix id (just a zero-based counter; unused in this test so far) */

		typedef std::map<std::pair<size_t, size_t>, TBlockMatrix, COrderingType> TBlockMap; /**< @brief ordered container for matrix blocks */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_parent is a pointer to the instance of outer class (contains the timers)
		 */
		CBenchmarkCholesky(CBlockMatrixBenchmark *p_parent)
			:m_p_parent(p_parent), m_n_matrix_id(0)
		{}

		/**
		 *	@brief performs testing on one matrix
		 *
		 *	@param[in] r_raw_trip_matrix is the matrix to be tested
		 *	@param[in] r_s_filename is name of file where the matrix is stored (for verbose)
		 *
		 *	@return Returns true on success, false on failure.
		 */
		bool operator ()(const TRawMatrix &r_raw_trip_matrix, const std::string &r_s_filename)
		{
			bool b_have_layout = false;
			size_t n_layout_w, n_layout_h, n_layout_nnz;
			size_t n_layout_bw, n_layout_bh, n_layout_bnnz;
			std::vector<size_t> cols_cumsums;
			std::vector<size_t> rows_cumsums;
			do {
				char p_s_layout_file[256];
				strcpy(p_s_layout_file, r_s_filename.c_str());
				if(strrchr(p_s_layout_file, '.'))
					*(char*)strrchr(p_s_layout_file, '.') = 0;
				strcat(p_s_layout_file, "_block-layout.txt");
				// see if we have block layout

				FILE *p_fr;
				if(!(p_fr = fopen(p_s_layout_file, "r")))
					break;
				if(fscanf(p_fr, PRIsize " x " PRIsize " (" PRIsize ")\n", &n_layout_h,
				   &n_layout_w, &n_layout_nnz) != 3) {
					fclose(p_fr);
					break;
				}
				if(n_layout_h != r_raw_trip_matrix.n_row_num ||
				   n_layout_w != r_raw_trip_matrix.n_col_num ||
				   n_layout_nnz != r_raw_trip_matrix.data.size()) {
					fprintf(stderr, "error: unable to use stored layout: size/nnz mismatch\n");
					fclose(p_fr);
					break;
				}
				if(fscanf(p_fr, PRIsize " x " PRIsize " (" PRIsize ")\n", &n_layout_bh,
				   &n_layout_bw, &n_layout_bnnz) != 3) {
					fclose(p_fr);
					break;
				}
				rows_cumsums.resize(n_layout_bh+1); // first is zero
				cols_cumsums.resize(n_layout_bw+1); // first is zero
				bool b_fail = false;
				for(size_t i = 0, n = rows_cumsums.size(); i < n; ++ i) {
					size_t n_cumsum;
					if(fscanf(p_fr, PRIsize, &n_cumsum) != 1) {
						b_fail = true;
						break;
					}
					rows_cumsums[i] = n_cumsum;
					if(i && rows_cumsums[i] - rows_cumsums[i - 1] != block_Size &&
					   rows_cumsums[i] - rows_cumsums[i - 1] != block_Size + 1) {
						fprintf(stderr, "error: unable to use stored layout: block size " PRIsize " (must be %d or %d)\n",
							rows_cumsums[i] - rows_cumsums[i - 1], block_Size, block_Size + 1);
						b_fail = true;
						break;
					}
				}
				if(b_fail) {
					fclose(p_fr);
					break;
				}
				for(size_t i = 0, n = cols_cumsums.size(); i < n; ++ i) {
					size_t n_cumsum;
					if(fscanf(p_fr, PRIsize, &n_cumsum) != 1) {
						b_fail = true;
						break;
					}
					cols_cumsums[i] = n_cumsum;
					if(i && cols_cumsums[i] - cols_cumsums[i - 1] != block_Size &&
					   cols_cumsums[i] - cols_cumsums[i - 1] != block_Size + 1) {
						fprintf(stderr, "error: unable to use stored layout: block size " PRIsize " (must be %d or %d)\n",
							cols_cumsums[i] - cols_cumsums[i - 1], block_Size, block_Size + 1);
						b_fail = true;
						break;
					}
				}
				if(b_fail) {
					fclose(p_fr);
					break;
				}
				fclose(p_fr);

				if(rows_cumsums.front() != 0 || rows_cumsums.back() != n_layout_h ||
				   cols_cumsums.front() != 0 || cols_cumsums.back() != n_layout_w) {
					fprintf(stderr, "error: unable to use stored layout: corrupt cumsums\n");
					break;
				}

				fprintf(stderr, "debug: using special layout for \'%s\'\n", r_s_filename.c_str());
				b_have_layout = true;
			} while(0);

			size_t n_pass_num = m_p_parent->n_Pass_Num();

			if(r_raw_trip_matrix.n_col_num != r_raw_trip_matrix.n_row_num) {
				fprintf(stderr, "warning: skipping \'%s\': not Cholesky candidate"
					" (" PRIsize " x " PRIsize ")\n", r_s_filename.c_str(),
					r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num);
				return true; // unlikely a candidate for cholesky
			}
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
			if(!b_have_layout && r_raw_trip_matrix.n_col_num % block_Size) {
				fprintf(stderr, "warning: skipping \'%s\': not multiple of block size"
					" (" PRIsize " x " PRIsize ")\n", r_s_filename.c_str(),
					r_raw_trip_matrix.n_col_num, r_raw_trip_matrix.n_col_num);
				return true;
			}
#endif // __BLOCK_BENCH_BLOCK_TYPE_A

			m_p_parent->BeginTest(r_s_filename);
			// begin a test

			m_p_parent->SetTestProperty("matrix-rows", double(r_raw_trip_matrix.n_row_num));
			m_p_parent->SetTestProperty("matrix-cols", double(r_raw_trip_matrix.n_col_num));
			m_p_parent->SetTestProperty("matrix-sparsity", double(r_raw_trip_matrix.data.size()) /
				(r_raw_trip_matrix.n_row_num * r_raw_trip_matrix.n_col_num));
			// set test properties

			std::string s_base_filename = r_s_filename;
			if(s_base_filename.rfind('.') != std::string::npos)
				s_base_filename.erase(s_base_filename.rfind('.'));
			// create base filename

#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
			printf("matrix \'%s\' is %d x %d\n", r_s_filename.c_str(),
				r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num);
			// multiplication debugging

			{
				cs *p_trip = cs_spalloc(r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num, 1, 1, 1);
				for(size_t i = 0, n = r_raw_trip_matrix.data.size(); i < n; ++ i) {
					const TTrip &r_tr = r_raw_trip_matrix.data[i];
					cs_entry(p_trip, r_tr.n_row, r_tr.n_col, r_tr.f_value);
				}
				cs *p_A = cs_compress(p_trip);
				cs_spfree(p_trip);
				// create sparse matrix

				FILE *p_fw;
				std::string s_filename = s_base_filename + "_raw.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "m = ") ||
					   ferror(p_fw)) {
						fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
						DeleteFile(s_filename.c_str());
					}
					fclose(p_fw);
				} else {
					fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
					DeleteFile(s_filename.c_str());
				}
				// save it

				cs_spfree(p_A);
			}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

			for(size_t n_pass = 0; n_pass < n_pass_num; ++ n_pass) {
				m_p_parent->StartTimer("ublock-trip");

				TBlockMap block_list; // blocks and their coordinates
				for(size_t i = 0, n = r_raw_trip_matrix.data.size(); i < n; ++ i) {
					const TTrip &r_tr = r_raw_trip_matrix.data[i];
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					size_t x = r_tr.n_col / block_Size, y = r_tr.n_row / block_Size;
					size_t bx = r_tr.n_col % block_Size, by = r_tr.n_row % block_Size;
					typename TBlockMap::iterator p_block_it;
					if((p_block_it = block_list.find(std::make_pair(x, y))) != block_list.end())
						(*p_block_it).second(by, bx) = r_tr.f_value; // no += to avoid doubling upper diag
					else {
						TBlockMatrix b;
						b.setZero();
						b(by, bx) = r_tr.f_value;
						block_list[std::make_pair(x, y)] = b;
					}
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					size_t x = r_tr.n_col, y = r_tr.n_row; // convert nonzero elements to blocks
					TBlockMatrix b;
					b.setConstant(r_tr.f_value); // or as NIST BLAS, use constant value that doesn't produce denormals and != 1 (isn't identity in multiplication)
					if(block_Size > 1) {
#if 1
						b.diagonal() *= 2; // raise the diagonal to be pos def
						// like this
#else // 1
						for(size_t j = 0; j < block_Size; ++ j)
							b(j, j) += 1; // does not work
						// raise the diagonal to be pos def
						// or like that
#endif // 1
					}
					block_list[std::make_pair(x, y)] = b; // just put it there
#endif // __BLOCK_BENCH_BLOCK_TYPE_A

					if(x > y)
						continue; // has upper diagonal - ok
					// uflcsm contains only lower triangular parts of cholesky candidate matrices,
					// have to mirror the lower diagonal parts (has no influence on full matrices)

					std::swap(x, y);
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					std::swap(bx, by);
					if((p_block_it = block_list.find(std::make_pair(x, y))) != block_list.end())
						(*p_block_it).second(by, bx) = r_tr.f_value; // no += to avoid doubling upper diag
					else {
						TBlockMatrix b;
						b.setZero();
						b(by, bx) = r_tr.f_value;
						block_list[std::make_pair(x, y)] = b;
					}
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					block_list[std::make_pair(x, y)] = b; // just put it there
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
				}
				// creates "triplet form" (sort by columns - by rows is much slower)

				m_p_parent->StopTimer("ublock-trip");

				{
					m_p_parent->StartTimer("ublock-compress");

					CUberBlockMatrix bm((r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size,
						(r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size);
					std::for_each(block_list.begin(), block_list.end(), CAppendBlock<TBlockMatrix, block_Size>(bm));

					m_p_parent->StopTimer("ublock-compress");

					if(b_have_layout) {
						CUberBlockMatrix bm_lay(rows_cumsums.begin() + 1, rows_cumsums.end(),
							cols_cumsums.begin() + 1, cols_cumsums.end());
						if(bm_lay.n_Row_Num() > bm.n_Row_Num() || bm_lay.n_Column_Num() > bm.n_Column_Num()) {
							fprintf(stderr, "error: the provided layout does not fit\n");
							return false;
						}
						if(!bm_lay.b_SymmetricLayout()) {
							fprintf(stderr, "error: the provided layout is not symmetric\n");
							return false;
						}
						// create matrix with layout

						cs *p_elem = bm.p_Convert_to_Sparse();

						if(size_t(p_elem->n) > bm_lay.n_Column_Num())
							p_elem->n = bm_lay.n_Column_Num();
						if(size_t(p_elem->m) > bm_lay.n_Row_Num()) {
							size_t m = p_elem->m = bm_lay.n_Row_Num();
							for(size_t i = 0; i < size_t(p_elem->n); ++ i) {
								for(size_t b = p_elem->p[i], e = p_elem->p[i + 1]; b < e; ++ b) {
									if(size_t(p_elem->i[b]) >= m) {
										p_elem->i[b] = p_elem->i[b - 1];
										p_elem->x[b] = 0;
										// just dup the last value, From_Sparse() doesn't have the additive semantic
									}
								}
							}
							cs_droptol(p_elem, 0); // remove the nulls
						}
						// "trim" the matrix if too big

						std::vector<size_t> work;
						bm_lay.From_Sparse(0, 0, p_elem, false, work);
						cs_spfree(p_elem);
						// convert to sparse and then eat it back to the matrix with the specific layout (wowzer)

						bm.Swap(bm_lay);
						// use this in the tests from now on
					}
					// applies the preloaded layout

#ifdef __BLOCK_BENCH_CHOLESKY_USE_AMD
					{
						cs *p_struct = bm.p_BlockStructure_to_Sparse();
						_ASSERTE(sizeof(size_t) == sizeof(csi));
						size_t *p_order = (size_t*)cs_amd(1, p_struct);
						size_t *p_order_inv = (size_t*)cs_pinv((const csi*)p_order, p_struct->n);
						CUberBlockMatrix perm;
						bm.PermuteTo(perm, p_order_inv, bm.n_BlockColumn_Num(), true, true, false);
						perm.Swap(bm);
						cs_spfree(p_struct);
						cs_free(p_order);
						cs_free(p_order_inv);
					}
					// reorder the matrix using AMD
#endif // __BLOCK_BENCH_CHOLESKY_USE_AMD

					m_p_parent->StartTimer("ublock-etree");

					std::vector<size_t> etree, work;
					bm.Build_EliminationTree(etree, work);

					m_p_parent->StopTimer("ublock-etree");

					m_p_parent->StartTimer("ublock-ereach");

					const size_t n = bm.n_BlockColumn_Num();
					std::vector<size_t> bitfield;
					bitfield.resize(n, 0);
					for(size_t i = 0; i < n; ++ i)
						bm.n_Build_EReach(i, etree, work, bitfield);

					m_p_parent->StopTimer("ublock-ereach");

					{
						m_p_parent->StartTimer("ublock-chol");

						CUberBlockMatrix R;
						bool b_success = R.CholeskyOf(bm, etree, work, bitfield);

						if(b_success)
							m_p_parent->StopTimer("ublock-chol");
						else {
							m_p_parent->CancelTimer("ublock-chol");
							fprintf(stderr, "warning: R.CholeskyOf(%s) failed\n", r_s_filename.c_str());
						}
					}

					typedef typename MakeTypelist_Safe((Eigen::Matrix<double, block_Size, block_Size>)) one_type_list;
					typedef typename MakeTypelist_Safe((Eigen::Matrix<double, block_Size, block_Size>,
						Eigen::Matrix<double, block_Size + 1, block_Size + 1>,
						Eigen::Matrix<double, block_Size, block_Size + 1>,
						Eigen::Matrix<double, block_Size + 1, block_Size>)) two_type_list; // that's correct, instances
					// block size lists with one and two (four actually, two per dimension) different block sizes

					m_p_parent->StartTimer("ublock-chol-FBS");

					CUberBlockMatrix R;
					bool b_success;
					if(b_have_layout) // landmark datasets need layouts, use two_type_list
						b_success = R.CholeskyOf_FBS<two_type_list>(bm, etree, work, bitfield);
					else
						b_success = R.CholeskyOf_FBS<one_type_list>(bm, etree, work, bitfield);

					if(b_success)
						m_p_parent->StopTimer("ublock-chol-FBS");
					else {
						m_p_parent->CancelTimer("ublock-chol-FBS");
						fprintf(stderr, "warning: R.CholeskyOf_FBS(%s) failed\n", r_s_filename.c_str());
					}

					if(!n_pass && b_success) {
						CUberBlockMatrix bm_back;
						R.PreMultiplyWithSelfTransposeTo(bm_back);
						bm.AddTo(bm_back, -1);
						double f_a_diff_ubm = bm_back.f_Norm();
						// calculate difference for UBM

						m_p_parent->SetTestProperty("RTR - A", f_a_diff_ubm);
					}
					// calculate factorization errors in the first pass

					m_p_parent->StartTimer("ublock-to-cs");

					cs *p_m = bm.p_Convert_to_Sparse();

					m_p_parent->StopTimer("ublock-to-cs");

					m_p_parent->StartTimer("cs-etree");

					csi *p_etree = cs_etree(p_m, 0);

					m_p_parent->StopTimer("cs-etree");

					m_p_parent->StartTimer("cs-ereach");

					std::vector<csi> w, s;
					w.resize(p_m->n * 2, 0);
					s.resize(p_m->n * 2, 0);
					for(size_t i = 0, n = p_m->n; i < n; ++ i)
						cs_ereach(p_m, i, p_etree, &s[0], &w[0]);

					m_p_parent->StopTimer("cs-ereach");

					m_p_parent->StartTimer("cs-symbolic");

					css *p_s = cs_schol(0, p_m);

					m_p_parent->StopTimer("cs-symbolic");

					m_p_parent->StartTimer("cs-chol");

					csn *p_f = cs_chol(p_m, p_s);

					if(p_f)
						m_p_parent->StopTimer("cs-chol");
					else {
						m_p_parent->CancelTimer("cs-chol");
						fprintf(stderr, "warning: cs_chol(%s) failed\n", r_s_filename.c_str());
					}

					// big todo - benchmark cholesky with ordering (cs_amd to be fair),
					// measure time for matrix permutation and times for cs_cholsolve and our
					// (involves cs_amd, TranposeUpperDiagTo(), CholeskyOf(), cs_ipvec(),
					// UpperTriangularTranspose_Solve(), UpperTriangular_Solve() and cs_pvec())

					if(p_f && !n_pass) {
						cs *p_cs_r = cs_transpose(p_f->L, 1);
						if(b_success) {
							cs *p_r = R.p_Convert_to_Sparse();
							cs *p_fac_diff = cs_add(p_cs_r, p_r, 1, -1);
							double f_fac_diff = cs_norm(p_fac_diff);
							m_p_parent->SetTestProperty("L_ref - RT", f_fac_diff);

							cs_spfree(p_r);
							cs_spfree(p_fac_diff);
						}

						cs *p_a_back = cs_multiply(p_f->L, p_cs_r);
						cs *p_a_diff = cs_add(p_m, p_a_back, 1, -1);
						double f_a_diff_cs = cs_norm(p_a_diff);
						m_p_parent->SetTestProperty("LLT_ref - A", f_a_diff_cs);

						cs_spfree(p_cs_r);
						cs_spfree(p_a_back);
						cs_spfree(p_a_diff);

					}
					// calculate factorization errors in the first pass

					cs_spfree(p_m);
					free(p_etree);
					cs_sfree(p_s);
					if(p_f)
						cs_nfree(p_f);
					// free everything
				}
			}

			++ m_n_matrix_id;

			m_p_parent->EndTest(r_s_filename);

			return true;
		}
	};

	/**
	 *	@brief matrix allocation time (and other) benchmarks
	 *	@tparam COrderingType is ordering function for triplet entries
	 *	@tparam n_block_size is matrix block size for the given benchmark
	 */
	template <const int n_block_size, class COrderingType>
	class CBenchmarkAllocationTime {
	public:
		/**
		 *	@brief configuration enums
		 */
		enum {
			block_Size = n_block_size /**< @brief preset block size (const at compile-time) */
		};

		/**
		 *	@brief block matrix type (specifically avoids AutoAlign under x86)
		 */
	#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
		typedef Eigen::Matrix<double, block_Size, block_Size> TBlockMatrix;
	#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
		typedef Eigen::Matrix<double, block_Size, block_Size, Eigen::DontAlign> TBlockMatrix; // AutoAlign sometimes aligns matrices (if block_Size is power of two?) and then bat shit happens under x86
	#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

	protected:
		CBlockMatrixBenchmark *m_p_parent; /**< @brief pointer to the instance of outer class (contains the timers) */
		size_t m_n_matrix_id; /**< @brief matrix id (just a zero-based counter) */

		typedef std::map<std::pair<size_t, size_t>, TBlockMatrix, COrderingType> TBlockMap; /**< @brief ordered container for matrix blocks */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_parent is a pointer to the instance of outer class (contains the timers)
		 */
		CBenchmarkAllocationTime(CBlockMatrixBenchmark *p_parent)
			:m_p_parent(p_parent), m_n_matrix_id(0)
		{}

		/**
		 *	@brief performs testing on one matrix
		 *
		 *	@param[in] r_raw_trip_matrix is the matrix to be tested
		 *	@param[in] r_s_filename is name of file where the matrix is stored (for verbose)
		 *
		 *	@return Returns true on success, false on failure.
		 */
		bool operator ()(const TRawMatrix &r_raw_trip_matrix, const std::string &r_s_filename)
		{
			size_t n_pass_num = m_p_parent->n_Pass_Num();

			m_p_parent->BeginTest(r_s_filename);
			// begin a test

			m_p_parent->SetTestProperty("matrix-rows", double(r_raw_trip_matrix.n_row_num));
			m_p_parent->SetTestProperty("matrix-cols", double(r_raw_trip_matrix.n_col_num));
			m_p_parent->SetTestProperty("matrix-sparsity", double(r_raw_trip_matrix.data.size()) /
				(r_raw_trip_matrix.n_row_num * r_raw_trip_matrix.n_col_num));
			// set test properties

			std::string s_base_filename = r_s_filename;
			if(s_base_filename.rfind('.') != std::string::npos)
				s_base_filename.erase(s_base_filename.rfind('.'));
			// create base filename

#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
			printf("matrix \'%s\' is %d x %d\n", r_s_filename.c_str(),
				r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num);
			// multiplication debugging

			{
				cs *p_trip = cs_spalloc(r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num, 1, 1, 1);
				for(size_t i = 0, n = r_raw_trip_matrix.data.size(); i < n; ++ i) {
					const TTrip &r_tr = r_raw_trip_matrix.data[i];
					cs_entry(p_trip, r_tr.n_row, r_tr.n_col, r_tr.f_value);
				}
				cs *p_A = cs_compress(p_trip);
				cs_spfree(p_trip);
				// create sparse matrix

				FILE *p_fw;
				std::string s_filename = s_base_filename + "_raw.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
					if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "m = ") ||
					   ferror(p_fw)) {
						fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
						DeleteFile(s_filename.c_str());
					}
					fclose(p_fw);
				} else {
					fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
					DeleteFile(s_filename.c_str());
				}
				// save it

				cs_spfree(p_A);
			}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

			/*for(size_t n_pass = 0; n_pass < n_pass_num; ++ n_pass) {
				m_p_parent->StartTimer("csparse-trip");

				cs *p_trip = cs_spalloc(r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num, 1, 1, 1);
				for(size_t i = 0, n = r_raw_trip_matrix.data.size(); i < n; ++ i) {
					const TTrip &r_tr = r_raw_trip_matrix.data[i];
					cs_entry(p_trip, r_tr.n_row, r_tr.n_col, r_tr.f_value);
				}

				m_p_parent->StopTimer("csparse-trip");

				m_p_parent->StartTimer("csparse-compress");

				cs *p_A = cs_compress(p_trip);
				cs_spfree(p_trip);

				m_p_parent->StopTimer("csparse-compress");

				m_p_parent->StartTimer("csparse-transpose");

				cs *p_At = cs_transpose(p_A, 1); // A = A^T

				m_p_parent->StopTimer("csparse-transpose");

				m_p_parent->StartTimer("csparse-add");

				cs *p_add1 = 0;
				if(p_A->m == p_A->n)
					p_add1 = cs_add(p_A, p_At, 1, 1); // A + At
				cs *p_add2 = cs_add(p_A, p_A, 1, 1); // A + A
				if(p_add1)
					cs_spfree(p_add1);
				cs_spfree(p_add2);

				m_p_parent->StopTimer("csparse-add");

				m_p_parent->StartTimer("csparse-multiply");

				cs *p_AtA = cs_multiply(p_At, p_A);
				cs_spfree(p_At);
				cs_spfree(p_AtA);

				m_p_parent->StopTimer("csparse-multiply");

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
				if(!n_pass) {
					char p_s_matrix[256];
					sprintf(p_s_matrix, "matrix_bench_%04d_cs.tga", m_n_matrix_id);
					CDebug::Dump_SparseMatrix(p_s_matrix, p_A);
				}
				// debug write matrices to a .tga file
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES

				cs_spfree(p_A);
			}*/
			// test CSparse

			bool b_is_usolvable = (r_raw_trip_matrix.n_row_num == r_raw_trip_matrix.n_col_num);
			bool b_is_utsolvable = (r_raw_trip_matrix.n_row_num == r_raw_trip_matrix.n_col_num);
			bool b_is_lsolvable = (r_raw_trip_matrix.n_row_num == r_raw_trip_matrix.n_col_num);
			bool b_is_ltsolvable = (r_raw_trip_matrix.n_row_num == r_raw_trip_matrix.n_col_num);

			for(size_t n_pass = 0; n_pass < n_pass_num; ++ n_pass) {
				m_p_parent->StartTimer("ublock-trip");

				TBlockMap block_list; // blocks and their coordinates
				for(size_t i = 0, n = r_raw_trip_matrix.data.size(); i < n; ++ i) {
					const TTrip &r_tr = r_raw_trip_matrix.data[i];
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					size_t x = r_tr.n_col / block_Size, y = r_tr.n_row / block_Size;
					size_t bx = r_tr.n_col % block_Size, by = r_tr.n_row % block_Size;
					typename TBlockMap::iterator p_block_it;
					if((p_block_it = block_list.find(std::make_pair(x, y))) != block_list.end())
						(*p_block_it).second(by, bx) += r_tr.f_value; // t_odo - handle block zero-ing // t_odo - use += for the repeated values
					else {
						TBlockMatrix b;
						b.setZero();
						b(by, bx) = r_tr.f_value;
						block_list[std::make_pair(x, y)] = b;
					}
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					//size_t bx = 0, by = 0; // unused
					size_t x = r_tr.n_col, y = r_tr.n_row; // convert nonzero elements to blocks
					TBlockMatrix b;
					b.setConstant(1.1); // same as NIST BLAS, use constant value that doesn't produce denormals and != 1 (isn't identity in multiplication)
					//b(by, bx) = r_tr.f_value; // no
					block_list[std::make_pair(x, y)] = b; // just put it there
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
				}
				// creates "triplet form" (sort by columns - by rows is much slower)

				m_p_parent->StopTimer("ublock-trip");

#if 1
				{
					m_p_parent->StartTimer("ublock-compress");

					CUberBlockMatrix bm((r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size,
						(r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size);
					std::for_each(block_list.begin(), block_list.end(), CAppendBlock<TBlockMatrix, block_Size>(bm));

					b_is_usolvable = b_is_usolvable && bm.b_UpperBlockTriangular();
					b_is_utsolvable = b_is_utsolvable && b_is_usolvable;
					b_is_lsolvable = b_is_usolvable && bm.b_LowerBlockTriangular();
					b_is_ltsolvable = b_is_utsolvable && b_is_usolvable;

					m_p_parent->StopTimer("ublock-compress");

					m_p_parent->StartTimer("ublock-transpose");

					CUberBlockMatrix bmT;
					bm.TransposeTo(bmT); // bm' = bm^T

					m_p_parent->StopTimer("ublock-transpose");

#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
					if(!n_pass) {
						cs *p_A = bm.p_Convert_to_Sparse();

						FILE *p_fw;
						std::string s_filename = s_base_filename + "_ub_bm.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
						if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
						if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
							if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "ub_m = ") ||
							   ferror(p_fw)) {
								fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
								DeleteFile(s_filename.c_str());
							}
							fclose(p_fw);
						} else {
							fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
							DeleteFile(s_filename.c_str());
						}
						// save it

						cs_spfree(p_A);
					}
					if(!n_pass) {
						cs *p_A = bmT.p_Convert_to_Sparse();

						FILE *p_fw;
						std::string s_filename = s_base_filename + "_ub_bmt.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
						if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
						if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
							if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "ub_mt = ") ||
							   ferror(p_fw)) {
								fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
								DeleteFile(s_filename.c_str());
							}
							fclose(p_fw);
						} else {
							fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
							DeleteFile(s_filename.c_str());
						}
						// save it

						cs_spfree(p_A);
					}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

					try {
						m_p_parent->StartTimer("ublock-multiply");

						CUberBlockMatrix mul;
						mul.ProductOf(bmT, bm); // mul = bm' * bm

						m_p_parent->StopTimer("ublock-multiply");

#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
						if(!n_pass) {
							cs *p_A = mul.p_Convert_to_Sparse();

							FILE *p_fw;
							std::string s_filename = s_base_filename + "_ub_mul.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
							if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
							if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
								if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "ub_mtm = ") ||
								   ferror(p_fw)) {
									fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
									DeleteFile(s_filename.c_str());
								}
								fclose(p_fw);
							} else {
								fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
								DeleteFile(s_filename.c_str());
							}
							// save it

							cs_spfree(p_A);
						}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

						/*if(!n_pass) {
							printf("ublock A\'A has %d blocks (%.2f blocks in storage)\n",
								mul.n_Block_Num(), mul.n_Storage_Size() / float(block_Size * block_Size));
						}*/
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-multiply");
						fprintf(stderr, "warning: std::bad_alloc during ublock-multiply\n");
					}

					typedef typename MakeTypelist_Safe((Eigen::Matrix<double, block_Size, block_Size>)) one_type_list;
					typedef typename MakeTypelist_Safe((Eigen::Matrix<double, block_Size, block_Size>,
						Eigen::Matrix<double, block_Size + 1, block_Size + 1>,
						Eigen::Matrix<double, block_Size, block_Size + 1>,
						Eigen::Matrix<double, block_Size + 1, block_Size>)) two_type_list; // that's correct, instances
					// block size lists with one and two (four actually, two per dimension) different block sizes

					try {
						std::vector<double> unit_vector, dest_vector;
						unit_vector.resize(bm.n_Column_Num(), 1.1);
						dest_vector.resize(bm.n_Row_Num(), .0);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("ublock-gaxpy-pre");

						bm.PreMultiply_Add(&dest_vector[0], dest_vector.size(),
							&unit_vector[0], unit_vector.size());

						m_p_parent->StopTimer("ublock-gaxpy-pre");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-gaxpy-pre");
						fprintf(stderr, "warning: std::bad_alloc during ublock-gaxpy-pre\n");
					}

					try {
						std::vector<double> unit_vector, dest_vector;
						unit_vector.resize(bm.n_Row_Num(), 1.1);
						dest_vector.resize(bm.n_Column_Num(), .0);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("ublock-gaxpy-post");

						bm.PostMultiply_Add(&dest_vector[0], dest_vector.size(),
							&unit_vector[0], unit_vector.size());

						m_p_parent->StopTimer("ublock-gaxpy-post");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-gaxpy-post");
						fprintf(stderr, "warning: std::bad_alloc during ublock-gaxpy-post\n");
					}

					try {
						std::vector<double> unit_vector, dest_vector;
						unit_vector.resize(bm.n_Row_Num(), 1.1);
						dest_vector.resize(bm.n_Column_Num(), .0);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("ublock-gaxpy-post-FBS");

						bm.PostMultiply_Add_FBS<one_type_list>(&dest_vector[0], dest_vector.size(),
							&unit_vector[0], unit_vector.size());

						m_p_parent->StopTimer("ublock-gaxpy-post-FBS");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-gaxpy-post-FBS");
						fprintf(stderr, "warning: std::bad_alloc during ublock-gaxpy-post-FBS\n");
					}

					if(b_is_usolvable /*&& bm.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-usolve");

							if(!bm.UpperTriangular_Solve(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-usolve");
								b_is_usolvable = false;
							} else
								m_p_parent->StopTimer("ublock-usolve");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-usolve");
							fprintf(stderr, "warning: std::bad_alloc during ublock-usolve\n");
						}
					}

					if(b_is_usolvable /*&& bm.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-usolve-FBS");

							if(!bm.UpperTriangular_Solve_FBS<one_type_list>(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-usolve-FBS");
								b_is_usolvable = false;
							} else
								m_p_parent->StopTimer("ublock-usolve-FBS");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-usolve-FBS");
							fprintf(stderr, "warning: std::bad_alloc during ublock-usolve-FBS\n");
						}
					}

					if(b_is_utsolvable /*&& bm.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-utsolve");

							if(!bm.UpperTriangularTranspose_Solve(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-utsolve");
								b_is_utsolvable = false;
							} else
								m_p_parent->StopTimer("ublock-utsolve");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-utsolve");
							fprintf(stderr, "warning: std::bad_alloc during ublock-utsolve\n");
						}
					}

					if(b_is_utsolvable /*&& bm.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-utsolve-FBS");

							if(!bm.UpperTriangularTranspose_Solve_FBS<one_type_list>(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-utsolve-FBS");
								b_is_utsolvable = false;
							} else
								m_p_parent->StopTimer("ublock-utsolve-FBS");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-utsolve-FBS");
							fprintf(stderr, "warning: std::bad_alloc during ublock-utsolve-FBS\n");
						}
					}

					if(b_is_lsolvable /*&& bmT.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-lsolve");

							if(!bmT.UpperTriangular_Solve(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-lsolve");
								b_is_lsolvable = false;
							} else
								m_p_parent->StopTimer("ublock-lsolve");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-lsolve");
							fprintf(stderr, "warning: std::bad_alloc during ublock-lsolve\n");
						}
					}

					if(b_is_lsolvable /*&& bmT.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-lsolve-FBS");

							if(!bmT.UpperTriangular_Solve_FBS<one_type_list>(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-lsolve-FBS");
								b_is_lsolvable = false;
							} else
								m_p_parent->StopTimer("ublock-lsolve-FBS");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-lsolve-FBS");
							fprintf(stderr, "warning: std::bad_alloc during ublock-lsolve-FBS\n");
						}
					}

					if(b_is_ltsolvable /*&& bmT.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-ltsolve");

							if(!bmT.UpperTriangularTranspose_Solve(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-ltsolve");
								b_is_ltsolvable = false;
							} else
								m_p_parent->StopTimer("ublock-ltsolve");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-ltsolve");
							fprintf(stderr, "warning: std::bad_alloc during ublock-ltsolve\n");
						}
					}

					if(b_is_ltsolvable /*&& bmT.b_Square()*/) {
						try {
							std::vector<double> unit_vector;
							unit_vector.resize(bm.n_Column_Num(), 1.1);
							// make a dense vector to benchmark operations

							m_p_parent->StartTimer("ublock-ltsolve-FBS");

							if(!bmT.UpperTriangularTranspose_Solve_FBS<one_type_list>(&unit_vector[0], unit_vector.size())) {
								m_p_parent->CancelTimer("ublock-ltsolve-FBS");
								b_is_ltsolvable = false;
							} else
								m_p_parent->StopTimer("ublock-ltsolve-FBS");
						} catch(std::bad_alloc&) {
							m_p_parent->CancelTimer("ublock-ltsolve-FBS");
							fprintf(stderr, "warning: std::bad_alloc during ublock-ltsolve-FBS\n");
						}
					}

					try {
						m_p_parent->StartTimer("ublock-multiply-FBSsingle");

						CUberBlockMatrix mul;
						mul.ProductOf_FBS<one_type_list, one_type_list>(bmT, bm); // mul = bm' * bm
						//bmT.MultiplyToWith_FBS<one_type_list, one_type_list>(mul, bm);

						m_p_parent->StopTimer("ublock-multiply-FBSsingle");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-multiply-FBSsingle");
						fprintf(stderr, "warning: std::bad_alloc during ublock-multiply-FBSsingle\n");
					}

					try {
						m_p_parent->StartTimer("ublock-multiply-FBStwo");

						CUberBlockMatrix mul;
						mul.ProductOf_FBS<two_type_list, two_type_list>(bmT, bm); // mul = bm' * bm
						//bmT.MultiplyToWith_FBS<one_type_list, one_type_list>(mul, bm);

						m_p_parent->StopTimer("ublock-multiply-FBStwo");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-multiply-FBStwo");
						fprintf(stderr, "warning: std::bad_alloc during ublock-multiply-FBStwo\n");
					}

					try {
						m_p_parent->StartTimer("ublock-ata");

						CUberBlockMatrix mul;
						bm.PreMultiplyWithSelfTransposeTo(mul); // mul = bm' * bm, somewhat optimized version

						m_p_parent->StopTimer("ublock-ata");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-ata");
						fprintf(stderr, "warning: std::bad_alloc during ublock-ata\n");
					}

					try {
						m_p_parent->StartTimer("ublock-ata-FBSsingle");

						CUberBlockMatrix mul;
						bm.PreMultiplyWithSelfTransposeTo_FBS_Parallel<one_type_list>(mul); // mul = bm' * bm, somewhat optimized version

						m_p_parent->StopTimer("ublock-ata-FBSsingle");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-ata-FBSsingle");
						fprintf(stderr, "warning: std::bad_alloc during ublock-ata-FBSsingle\n");
					}

					try {
						m_p_parent->StartTimer("ublock-ata-FBStwo");

						CUberBlockMatrix mul;
						bm.PreMultiplyWithSelfTransposeTo_FBS_Parallel<two_type_list>(mul); // mul = bm' * bm, somewhat optimized version

						m_p_parent->StopTimer("ublock-ata-FBStwo");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-ata-FBStwo");
						fprintf(stderr, "warning: std::bad_alloc during ublock-ata-FBStwo\n");
					}

					try {
						m_p_parent->StartTimer("ublock-add");

						if(bm.n_Row_Num() == bm.n_Column_Num())
							bm.AddTo(bmT); // bm' += bm
						bm.AddTo(bm); // bm += bm

						m_p_parent->StopTimer("ublock-add");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-add");
						fprintf(stderr, "warning: std::bad_alloc during ublock-add\n");
					}
					// test addition as last, since it changes matrix values

					try {
						m_p_parent->StartTimer("ublock-add-FBS");

						if(bm.n_Row_Num() == bm.n_Column_Num())
							bm.AddTo_FBS<one_type_list>(bmT); // bm' += bm
						bm.AddTo_FBS<one_type_list>(bm); // bm += bm

						m_p_parent->StopTimer("ublock-add-FBS");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("ublock-add-FBS");
						fprintf(stderr, "warning: std::bad_alloc during ublock-add\n");
					}
					// test addition as last, since it changes matrix values

					if(!n_pass) {
#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
						printf("per ublock-compress:\n");
						bm.Dump_PerfCounters();
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
						char p_s_matrix[256];
						sprintf(p_s_matrix, "matrix_bench_%04d_ubm.tga", m_n_matrix_id);
						if(!bm.Rasterize(p_s_matrix))
							DeleteFile(p_s_matrix);
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES
					}
					// debug write matrices to a .tga file
				}
				// test UberBlockMatrix

				{
					m_p_parent->StartTimer("ublock-compress-str");

#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					size_t n_block_cols = (r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size;
					size_t n_block_rows = (r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size;
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					size_t n_block_cols = r_raw_trip_matrix.n_col_num;
					size_t n_block_rows = r_raw_trip_matrix.n_row_num;
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
					// this way we're faster than g2o even on fragmented matrices
					// note that this is a bit slower on dense matrices than compress without pre-filling structure

					//std::vector<size_t> cumsum_cols(n_block_cols);
					std::vector<size_t> cumsum_rows(n_block_rows);
					/*for(int i = 0; i < n_block_cols; ++ i)
						cumsum_cols[i] = (i + 1) * block_Size;*/
					for(size_t i = 0; i < n_block_rows; ++ i)
						cumsum_rows[i] = (i + 1) * block_Size;

					CUberBlockMatrix bm(cumsum_rows.begin(), cumsum_rows.end(),
						n_block_cols/*cumsum_cols.begin(), cumsum_cols.end()*/); // add cols as they go, submit only count to avoid reallocation
					std::for_each(block_list.begin(), block_list.end(), CAppendBlock<TBlockMatrix, block_Size>(bm));

					m_p_parent->StopTimer("ublock-compress-str");

					if(!n_pass) {
						//printf("there is %d blocks in A\n", block_list.size());
#ifdef __UBER_BLOCK_MATRIX_PERFCOUNTERS
						printf("per ublock-compress-str:\n");
						bm.Dump_PerfCounters();
#endif // __UBER_BLOCK_MATRIX_PERFCOUNTERS

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
						char p_s_matrix[256];
						sprintf(p_s_matrix, "matrix_bench_%04d_ubm_str.tga", m_n_matrix_id);
						if(!bm.Rasterize(p_s_matrix))
							DeleteFile(p_s_matrix);
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES
					}
					// debug write matrices to a .tga file
				}
				// test UberBlockMatrix
#endif // 0

#ifdef __BLOCK_BENCH_G2O
				{
					m_p_parent->StartTimer("g2o-compress");

#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					size_t n_block_cols = (r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size;
					size_t n_block_rows = (r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size;
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					size_t n_block_cols = r_raw_trip_matrix.n_col_num;
					size_t n_block_rows = r_raw_trip_matrix.n_row_num;
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
					std::vector<int> cumsum_cols(n_block_cols);
					std::vector<int> cumsum_rows(n_block_rows);
					_ASSERTE(n_block_cols * block_Size <= INT_MAX);
					for(size_t i = 0; i < n_block_cols; ++ i)
						cumsum_cols[i] = int((i + 1) * block_Size);
					_ASSERTE(n_block_rows * block_Size <= INT_MAX);
					for(size_t i = 0; i < n_block_rows; ++ i)
						cumsum_rows[i] = int((i + 1) * block_Size);
					// build cumsums for matrix structure

					_ASSERTE(n_block_rows <= INT_MAX);
					_ASSERTE(n_block_cols <= INT_MAX);
					g2o::SparseBlockMatrix<> g2o_bm(&cumsum_rows[0], &cumsum_cols[0], int(n_block_rows), int(n_block_cols));
					// initialize matrix structure

					std::for_each(block_list.begin(), block_list.end(), CAppendBlock_g2o<TBlockMatrix>(g2o_bm));
					// fill matrix with blocks

					m_p_parent->StopTimer("g2o-compress");

					m_p_parent->StartTimer("g2o-transpose");

					g2o::SparseBlockMatrix<> *p_g2o_bmT = 0;
					g2o_bm.transpose(p_g2o_bmT); // bm' = bm^T

					m_p_parent->StopTimer("g2o-transpose");

					try {
						m_p_parent->StartTimer("g2o-multiply");

						g2o::SparseBlockMatrix<> *p_mul = 0;
						p_g2o_bmT->multiply(p_mul, &g2o_bm);

						m_p_parent->StopTimer("g2o-multiply");

#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
						if(!n_pass) {
							cs *p_A = cs_spalloc(p_mul->rows(), p_mul->cols(), p_mul->nonZeros(), 1, 0);
							p_mul->fillCCS(p_A->p, p_A->i, p_A->x);
							// convert to c-sparse

							FILE *p_fw;
							std::string s_filename = s_base_filename + "_g2o_mul.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
							if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
							if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
								if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "g2o_mtm = ") ||
								   ferror(p_fw)) {
									fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
									DeleteFile(s_filename.c_str());
								}
								fclose(p_fw);
							} else {
								fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
								DeleteFile(s_filename.c_str());
							}
							// save it

							cs_spfree(p_A);
						}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

						/*if(!n_pass)
							printf("g2o A\'A has %d blocks\n", p_mul->nonZeroBlocks());*/
						delete p_mul;
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("g2o-multiply");
						fprintf(stderr, "warning: std::bad_alloc during g2o-multiply\n");
					}

					try {
						m_p_parent->StartTimer("g2o-add");

						if(g2o_bm.rows() == g2o_bm.cols())
							g2o_bm.add(p_g2o_bmT); // bm' += bm
						g2o::SparseBlockMatrix<> *p_g2o_bm = &g2o_bm;
						g2o_bm.add(p_g2o_bm); // bm += bm

						m_p_parent->StopTimer("g2o-add");
					} catch(std::bad_alloc&) {
						m_p_parent->CancelTimer("g2o-add");
						fprintf(stderr, "warning: std::bad_alloc during g2o-add\n");
					}
					// test add at the end as we change matrix values here

					delete p_g2o_bmT;
				}
				// test g2o
#endif // __BLOCK_BENCH_G2O

#ifdef __BLOCK_BENCH_CERES
				{
					m_p_parent->StartTimer("ceres-compress");

#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					int n_block_cols = int((r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size);
					int n_block_rows = int((r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size);
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					int n_block_cols = int(r_raw_trip_matrix.n_col_num);
					int n_block_rows = int(r_raw_trip_matrix.n_row_num);
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
					_ASSERTE(n_block_rows <= INT_MAX);
					_ASSERTE(n_block_cols <= INT_MAX);
					std::vector<int> cumsum_cols(n_block_cols);
					std::vector<int> cumsum_rows(n_block_rows);
					_ASSERTE(n_block_cols * block_Size <= INT_MAX);
					for(int i = 0; i < n_block_cols; ++ i)
						cumsum_cols[i] = int((i + 1) * block_Size);
					_ASSERTE(n_block_rows * block_Size <= INT_MAX);
					for(int i = 0; i < n_block_rows; ++ i)
						cumsum_rows[i] = int((i + 1) * block_Size);
					// build cumsums for matrix structure

					ceres::internal::CompressedRowBlockStructure *p_struct = new
						ceres::internal::CompressedRowBlockStructure;
					{
						p_struct->cols.resize(n_block_cols);
						for(int i = 0; i < n_block_cols; ++ i) {
							p_struct->cols[i].size = block_Size;
							p_struct->cols[i].position = ((i)? cumsum_cols[i - 1] : 0);
						}
						p_struct->rows.resize(n_block_rows);
						for(int i = 0; i < n_block_rows; ++ i) {
							p_struct->rows[i].block.size = block_Size;
							p_struct->rows[i].block.position = ((i)? cumsum_rows[i - 1] : 0);

							/*p_struct->rows[i].cells.resize(row.cells_size());
							for (int j = 0; j < row.cells_size(); ++j) {
								const CellProto &cell = row.cells(j);
								p_struct->rows[i].cells[j].block_id = cell.block_id();
								p_struct->rows[i].cells[j].position = cell.position();
							}*/
							// Copy the cells within the row.
						}
					}
					// initialize block structure

					size_t n_position = 0;
					for(typename TBlockMap::const_iterator p_block_it = block_list.begin();
					   p_block_it != block_list.end(); ++ p_block_it) {
						const std::pair<size_t, size_t> &coord = (*p_block_it).first;
						const TBlockMatrix &r_block = (*p_block_it).second;
						size_t x = coord.first;
						size_t y = coord.second; // in blocks, not elements
						ceres::internal::Cell cell;
						_ASSERTE(x <= INT_MAX);
						cell.block_id = int(x);
						_ASSERTE(n_position <= INT_MAX);
						cell.position = int(n_position);
						n_position += r_block.rows() * r_block.cols(); // count stuff
						p_struct->rows[y].cells.push_back(cell);
					}
					// initialize cells (need to scatter them)

					ceres::internal::BlockSparseMatrix *p_bm = new ceres::internal::BlockSparseMatrix(p_struct);
					ceres::internal::BlockSparseMatrix &bm = *p_bm;

					{
						const ceres::internal::CompressedRowBlockStructure &bs = *bm.block_structure();
						double *p_data = bm.mutable_values();

						for(typename TBlockMap::const_iterator p_block_it = block_list.begin();
						   p_block_it != block_list.end(); ++ p_block_it) {
							const std::pair<size_t, size_t> &coord = (*p_block_it).first;
							const TBlockMatrix &r_block = (*p_block_it).second;
							size_t x = coord.first;
							size_t y = coord.second; // in blocks, not elements
							const ceres::internal::CompressedRow &row = bs.rows[y];
							for(size_t i = 0, n = row.cells.size(); i < n; ++ i) {
								if(row.cells[i].block_id == x) {
									double *p_block = p_data + row.cells[i].position;
									memcpy(p_block, r_block.data(), r_block.rows() * r_block.cols() * sizeof(double));
									break;
								}
								_ASSERTE(i + 1 != n);
							}
						}
					}
					// specify ceres block matrix bs and data

					m_p_parent->StopTimer("ceres-compress");

					{
						std::vector<double> unit_vector, dest_vector;
						unit_vector.resize(n_block_cols * block_Size, 1.1);
						dest_vector.resize(n_block_rows * block_Size, .0);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("ceres-gaxpy");

						bm.RightMultiply(&unit_vector[0], &dest_vector[0]);

						m_p_parent->StopTimer("ceres-gaxpy");
					}

					{
						std::vector<double> unit_vector, dest_vector;
						unit_vector.resize(n_block_rows * block_Size, 1.1);
						dest_vector.resize(n_block_cols * block_Size, .0);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("ceres-gaxpy-pre");

						bm.LeftMultiply(&unit_vector[0], &dest_vector[0]);

						m_p_parent->StopTimer("ceres-gaxpy-pre");
					}

					delete p_bm;
				}
				// test ceres
#endif // __BLOCK_BENCH_CERES

#if defined(_WIN32) || defined(_WIN64)
#if !(defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64))
#ifdef __BLOCK_BENCH_GTSAM
				if(r_raw_trip_matrix.n_col_num == r_raw_trip_matrix.n_row_num) { // GTSAM only supports symmetric block matrices or vertical (columns are only a single block) block matrices
					m_p_parent->StartTimer("gtsam-compress");

#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					size_t n_block_cols = (r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size;
					size_t n_block_rows = (r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size;
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					size_t n_block_cols = r_raw_trip_matrix.n_col_num;
					size_t n_block_rows = r_raw_trip_matrix.n_row_num;
#endif // __BLOCK_BENCH_BLOCK_TYPE_A
					std::vector<int> sizes_cols(n_block_cols, block_Size);
					// build matrix structure

					bool b_tt_started = false;
					try {
						gtsam::Matrix full_matrix(n_block_cols * block_Size, n_block_rows * block_Size); // full storage (align up to block_Size!)
						gtsam::SymmetricBlockView<gtsam::Matrix> blockMatrix(full_matrix, sizes_cols.begin(),
							sizes_cols.end()); // block matrix
						// initialize matrix structure

						std::for_each(block_list.begin(), block_list.end(), CAppendBlock_GTSAM<TBlockMatrix>(blockMatrix));
						// fill matrix with blocks

						m_p_parent->StopTimer("gtsam-compress");

						m_p_parent->StartTimer("gtsam-transpose");
						b_tt_started = true;

						gtsam::Matrix _transpose;
						_transpose = full_matrix.transpose();

						m_p_parent->StopTimer("gtsam-transpose");

						m_p_parent->StartTimer("gtsam-add");

						if(full_matrix.rows() == full_matrix.cols())
							_transpose += full_matrix;
						full_matrix += full_matrix;

						m_p_parent->StopTimer("gtsam-add");
					} catch(std::bad_alloc&) {
						if(b_tt_started)
							m_p_parent->CancelTimer("gtsam-transpose"); // don't measure this
						else
							m_p_parent->CancelTimer("gtsam-compress-0"); // don't measure this
						fprintf(stderr, "error: GTSAM produced std::bad_alloc on matrix %d\n", m_n_matrix_id);
					}
				}
				// test gtsam (there will likely be less samples)
#endif // __BLOCK_BENCH_GTSAM
#endif // !(_M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64)
#endif // _WIN32 || _WIN64

				{
					m_p_parent->StartTimer("csparse-trip-blocky");
					cs *p_trip = cs_spalloc(r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num, 1, 1, 1);
					{
						for(typename TBlockMap::iterator p_block_it = block_list.begin(),
						   p_end_it = block_list.end(); p_block_it != p_end_it; ++ p_block_it) {
							const std::pair<size_t, size_t> &t_pos = (*p_block_it).first;
							const TBlockMatrix &t_matrix = (*p_block_it).second;

							for(Eigen::DenseIndex r = 0, nr = t_matrix.rows(); r < nr; ++ r) {
								for(Eigen::DenseIndex c = 0, nc = t_matrix.cols(); c < nc; ++ c) {
									cs_entry(p_trip, t_pos.second * block_Size + r,
										t_pos.first * block_Size + c, t_matrix(r, c));
								}
							}
						}
					}
					m_p_parent->StopTimer("csparse-trip-blocky");

					m_p_parent->StartTimer("csparse-compress-blocky");
					cs *p_A = cs_compress(p_trip);
					m_p_parent->StopTimer("csparse-compress-blocky");
					cs_spfree(p_trip);

#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
					if(!n_pass) {
						FILE *p_fw;
						std::string s_filename = s_base_filename + "_blocky.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
						if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
						if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
							if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_A, "blk_m = ") ||
							   ferror(p_fw)) {
								fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
								DeleteFile(s_filename.c_str());
							}
							fclose(p_fw);
						} else {
							fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
							DeleteFile(s_filename.c_str());
						}
						// save it
					}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

					m_p_parent->StartTimer("csparse-transpose-blocky");

					cs *p_At = cs_transpose(p_A, 1); // At = A^T

					m_p_parent->StopTimer("csparse-transpose-blocky");

					{
						std::vector<double> unit_vector, dest_vector;
						unit_vector.resize(p_A->n, 1.1);
						dest_vector.resize(p_A->m, .0);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("csparse-gaxpy");

						cs_gaxpy(p_A, &unit_vector[0], &dest_vector[0]);

						m_p_parent->StopTimer("csparse-gaxpy");
					}

					m_p_parent->StartTimer("csparse-add-blocky");

					cs *p_add1 = 0;
					if(p_A->m == p_A->n)
						p_add1 = cs_add(p_A, p_At, 1, 1); // A + At
					cs *p_add2 = cs_add(p_A, p_A, 1, 1); // A + A
					if(p_add1)
						cs_spfree(p_add1);
					cs_spfree(p_add2);

					m_p_parent->StopTimer("csparse-add-blocky");

					/*if(!n_pass)
						printf("csparse A has %.2f blocks\n", p_A->p[p_A->n] / float(block_Size * block_Size));
					if(!n_pass)
						printf("csparse A\' has %.2f blocks\n", p_At->p[p_At->n] / float(block_Size * block_Size));*/

					m_p_parent->StartTimer("csparse-multiply-blocky");

					cs *p_AtA = cs_multiply(p_At, p_A);
					cs_spfree(p_At);
					//size_t n_AtA_nonzero = (p_AtA)? p_AtA->p[p_AtA->n] : 0; // nnz // unused
					if(p_AtA) {
#ifdef __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT
						if(!n_pass) {
							FILE *p_fw;
							std::string s_filename = s_base_filename + "_cs_mul.m";
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
							if(!fopen_s(&p_fw, s_filename.c_str(), "w")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
							if(p_fw = fopen(s_filename.c_str(), "w")) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
								if(!CDebug::Print_SparseMatrix_in_MatlabFormat(p_fw, p_AtA, "cs_mtm = ") ||
								   ferror(p_fw)) {
									fprintf(stderr, "error: failed to save \'%s\'\n", s_filename.c_str());
									DeleteFile(s_filename.c_str());
								}
								fclose(p_fw);
							} else {
								fprintf(stderr, "error: failed to open \'%s\' for writing\n", s_filename.c_str());
								DeleteFile(s_filename.c_str());
							}
							// save it
						}
#endif // __BLOCK_BENCH_DUMP_MATRICES_IN_MATLAB_FORMAT

						cs_spfree(p_AtA);

						m_p_parent->StopTimer("csparse-multiply-blocky");
					} else {
						m_p_parent->CancelTimer("csparse-multiply-blocky");
						fprintf(stderr, "error: csparse-multiply-blocky triggered std::bad_alloc (or something)\n");
					}

					/*if(!n_pass)
						printf("csparse A\'A has %.2f blocks\n", n_AtA_nonzero / float(block_Size * block_Size));*/

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
					if(!n_pass) {
						char p_s_matrix[256];
						sprintf(p_s_matrix, "matrix_bench_%04d_cs_blk.tga", m_n_matrix_id);
						CDebug::Dump_SparseMatrix(p_s_matrix, p_A);
					}
					// debug write matrices to a .tga file
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES

					cs_spfree(p_A);
				}
				// test csparse with blocks

				{
#ifdef __BLOCK_BENCH_BLOCK_TYPE_A
					//m_p_parent->StartTimer("csparse-trip");

					cs *p_trip = cs_spalloc(r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num, 1, 1, 1);
					for(size_t i = 0, n = r_raw_trip_matrix.data.size(); i < n; ++ i) {
						const TTrip &r_tr = r_raw_trip_matrix.data[i];

						cs_entry(p_trip, r_tr.n_row, r_tr.n_col, r_tr.f_value);
					}

					//m_p_parent->StopTimer("csparse-trip");


					//m_p_parent->StartTimer("csparse-compress");

					cs *p_A = cs_compress(p_trip);
					cs_spfree(p_trip);

					//m_p_parent->StopTimer("csparse-compress");
#else // __BLOCK_BENCH_BLOCK_TYPE_A
					CUberBlockMatrix bm((r_raw_trip_matrix.n_row_num + block_Size - 1) / block_Size,
						(r_raw_trip_matrix.n_col_num + block_Size - 1) / block_Size);
					std::for_each(block_list.begin(), block_list.end(), CAppendBlock<TBlockMatrix, block_Size>(bm));

					cs *p_A = 0;
					if(b_is_usolvable || b_is_utsolvable)
						p_A = bm.p_Convert_to_Sparse_UpperTriangular(); // want upper diag part only
#endif // __BLOCK_BENCH_BLOCK_TYPE_A

					if(b_is_usolvable /*&& p_A->n == p_A->m*/) {
						std::vector<double> unit_vector;
						unit_vector.resize(p_A->n, 1.1);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("csparse-usolve");

						cs_usolve(p_A, &unit_vector[0]);

						m_p_parent->StopTimer("csparse-usolve");
					}

					if(b_is_utsolvable /*&& p_A->n == p_A->m*/) {
						std::vector<double> unit_vector;
						unit_vector.resize(p_A->n, 1.1);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("csparse-utsolve");

						cs_utsolve(p_A, &unit_vector[0]);

						m_p_parent->StopTimer("csparse-utsolve");
					}

#ifndef __BLOCK_BENCH_BLOCK_TYPE_A
					if(b_is_lsolvable || b_is_ltsolvable) {
						CUberBlockMatrix bmT;
						bm.TransposeTo(bmT);
						cs *p_At = bmT.p_Convert_to_Sparse_UpperTriangular(p_A); // want lower diag part only (don't have function, have to transpose twice)
						p_A = cs_transpose(p_At, 1); // the original p_A is freed on the next line, no need to do it before this one
						cs_spfree(p_At);
					}
#endif // !__BLOCK_BENCH_BLOCK_TYPE_A

					if(b_is_lsolvable /*&& p_A->n == p_A->m*/) {
						std::vector<double> unit_vector;
						unit_vector.resize(p_A->n, 1.1);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("csparse-lsolve");

						cs_lsolve(p_A, &unit_vector[0]);

						m_p_parent->StopTimer("csparse-lsolve");
					}

					if(b_is_ltsolvable /*&& p_A->n == p_A->m*/) {
						std::vector<double> unit_vector;
						unit_vector.resize(p_A->n, 1.1);
						// make a dense vector to benchmark operations

						m_p_parent->StartTimer("csparse-ltsolve");

						cs_ltsolve(p_A, &unit_vector[0]);

						m_p_parent->StopTimer("csparse-ltsolve");
					}

					/*m_p_parent->StartTimer("csparse-transpose");

					cs *p_At = cs_transpose(p_A, 1); // A = A^T


					m_p_parent->StopTimer("csparse-transpose");

					m_p_parent->StartTimer("csparse-add");

					cs *p_add1 = 0;

					if(p_A->m == p_A->n)
						p_add1 = cs_add(p_A, p_At, 1, 1); // A + At
					cs *p_add2 = cs_add(p_A, p_A, 1, 1); // A + A
					if(p_add1)
						cs_spfree(p_add1);

					cs_spfree(p_add2);

					m_p_parent->StopTimer("csparse-add");

					m_p_parent->StartTimer("csparse-multiply");


					cs *p_AtA = cs_multiply(p_At, p_A);
					cs_spfree(p_At);
					cs_spfree(p_AtA);

					m_p_parent->StopTimer("csparse-multiply");*/

#ifdef __BLOCK_BENCH_DUMP_MATRIX_IMAGES
					if(!n_pass) {
						char p_s_matrix[256];
						sprintf(p_s_matrix, "matrix_bench_%04d_cs.tga", m_n_matrix_id);

						CDebug::Dump_SparseMatrix(p_s_matrix, p_A);
					}
					// debug write matrices to a .tga file
#endif // __BLOCK_BENCH_DUMP_MATRIX_IMAGES

					cs_spfree(p_A);
				}
				// test CSparse with elements (need elements for *solve() to work)

#if 0 //ndef _DEBUG // want fast results
				bool b_had_n = false;
				for(size_t i = block_list.size() % 2, n = block_list.size(); i < n; i += 2) {
					_ASSERTE(i + 2 <= n);
					if(i + 2 == n)
						b_had_n = true;
					// makes sure the last block was included

					m_p_parent->StartTimer("csparse-trip-incremental");
					cs *p_trip = cs_spalloc(r_raw_trip_matrix.n_row_num, r_raw_trip_matrix.n_col_num, 1, 1, 1);
					{
						typename TBlockMap::iterator p_block_it = block_list.begin();
						for(size_t n_fill_blocks = i + 2; n_fill_blocks > 0; ++ p_block_it, -- n_fill_blocks) {
							_ASSERTE(p_block_it != block_list.end());
							const std::pair<size_t, size_t> &t_pos = (*p_block_it).first;
							const TBlockMatrix &t_matrix = (*p_block_it).second;

							for(int r = 0, nr = t_matrix.rows(); r < nr; ++ r) {
								for(int c = 0, nc = t_matrix.cols(); c < nc; ++ c) {
									double f_value;
									if((f_value = t_matrix(r, c)) != 0)
										cs_entry(p_trip, t_pos.second * block_Size + r, t_pos.first * block_Size + c, f_value);
								}
							}
						}
					}
					if(i + 2 < n)
						m_p_parent->PauseTimer("csparse-trip-incremental");
					else
						m_p_parent->StopTimer("csparse-trip-incremental");

					m_p_parent->StartTimer("csparse-compress-incremental");
					cs *p_A = cs_compress(p_trip);
					if(i + 2 < n)
						m_p_parent->PauseTimer("csparse-compress-incremental");
					else
						m_p_parent->StopTimer("csparse-compress-incremental");
					cs_spfree(p_trip);
					cs_spfree(p_A);
				}
				_ASSERTE(b_had_n); // makes sure the last block was included
#endif // !_DEBUG
				// incremental CSparse - slow slow, don't debug it
			}
			// test block matrices

			++ m_n_matrix_id;

			m_p_parent->EndTest(r_s_filename);

			return true;
		}
	};

	/**
	 *	@brief appends a matrix block to CUberBlockMatrix
	 *
	 *	@tparam TBlockMatrix block matrix type (specifically avoids AutoAlign under x86)
	 *	@tparam block_Size is block size
	 */
	template <class TBlockMatrix, const int block_Size>
	class CAppendBlock {
	protected:
		CUberBlockMatrix &m_r_target; /**< @brief target matrix */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_target is target matrix
		 */
		CAppendBlock(CUberBlockMatrix &r_target)
			:m_r_target(r_target)
		{}

		/**
		 *	@brief appends the matrix block
		 *	@param[in] r_t_block is the matrix block
		 */
		inline void operator ()(const std::pair<const std::pair<size_t, size_t>,
			TBlockMatrix> &r_t_block)
		{
			size_t x = r_t_block.first.first;
			size_t y = r_t_block.first.second;
			const TBlockMatrix &r_t_block_payload = r_t_block.second;
			if(!m_r_target.Append_Block(r_t_block_payload, y * block_Size, x * block_Size)) {
				fprintf(stderr, "error: AppendBlock() failed\n");
				throw std::runtime_error("AppendBlock() failed");
			}
		}
	};

#ifdef __BLOCK_BENCH_G2O

	/**
	 *	@brief appends a matrix block to g2o block matrix
	 *	@tparam TBlockMatrix block matrix type (specifically avoids AutoAlign under x86)
	 */
	template <class TBlockMatrix>
	class CAppendBlock_g2o {
	protected:
		g2o::SparseBlockMatrix<> &m_r_target; /**< @brief target matrix */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_target is target matrix
		 */
		CAppendBlock_g2o(g2o::SparseBlockMatrix<> &r_target)
			:m_r_target(r_target)
		{}

		/**
		 *	@brief appends the matrix block
		 *	@param[in] r_t_block is the matrix block
		 */
		inline void operator ()(const std::pair<const std::pair<size_t, size_t>,
			TBlockMatrix> &r_t_block)
		{
			size_t x = r_t_block.first.first;
			size_t y = r_t_block.first.second;
			_ASSERTE(x <= INT_MAX && y <= INT_MAX);
			const TBlockMatrix &r_t_block_payload = r_t_block.second;
			*m_r_target.block(int(y), int(x), true) = r_t_block_payload; // copy matrix data
		}
	};

#endif // __BLOCK_BENCH_G2O

#if defined(_WIN32) || defined(_WIN64)
#if !(defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64))
#ifdef __BLOCK_BENCH_GTSAM

	/**
	 *	@brief appends a matrix block to GTSAM "block" matrix
	 *	@tparam TBlockMatrix block matrix type (specifically avoids AutoAlign under x86)
	 */
	template <class TBlockMatrix>
	class CAppendBlock_GTSAM {
	protected:
		gtsam::SymmetricBlockView<gtsam::Matrix> &m_r_target; /**< @brief target matrix */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_target is target matrix
		 */
		CAppendBlock_GTSAM(gtsam::SymmetricBlockView<gtsam::Matrix> &r_target)
			:m_r_target(r_target)
		{}

		/**
		 *	@brief appends the matrix block
		 *	@param[in] r_t_block is the matrix block
		 */
		inline void operator ()(const std::pair<const std::pair<int, int>,
			TBlockMatrix> &r_t_block)
		{
			int x = r_t_block.first.first;
			int y = r_t_block.first.second;
			const TBlockMatrix &r_t_block_payload = r_t_block.second;
			m_r_target(y, x) = r_t_block_payload; // copy matrix data
		}
	};

#endif // __BLOCK_BENCH_GTSAM
#endif // !(_M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64)
#endif // _WIN32 || _WIN64

	int m_n_ordering_type; /**< @brief block ordering type for the benchmark (one of order_*) */

public:
	enum {
		order_ColumnMajor = 0, // default
		order_RowMajor,
		order_TriangleOutline,
		order_RectOutline,

		order_MaxOrder // do not move this, has to be the last
	};

	/**
	 *	@brief default constructor
	 *
	 *	@param[in] n_pass_num is number of testing passes (to average-out the noise)
	 *	@param[in] n_ordering_type is block ordering type for the benchmark (one of order_*)
	 */
	CBlockMatrixBenchmark(size_t n_pass_num, int n_ordering_type = order_ColumnMajor);

	/**
	 *	@brief prints behavior of block orderings to stdout
	 */
	static void TestBlockOrderings();

	/**
	 *	@brief prints behavior of a block ordering to stdout
	 *
	 *	@tparam COr is ordering type
	 *
	 *	@param[in] ordering is instance of an ordering object
	 *	@param[in] p_s_or_name is ordering name (for verbose)
	 */
	template <class COr>
	static void TestBlockOrdering(COr ordering, const char *p_s_or_name)
	{
		std::vector<std::pair<size_t, size_t> > coords;
		for(size_t x = 0; x < 10; ++ x) {
			for(size_t y = 0; y < 10; ++ y)
				coords.push_back(std::make_pair(x, y));
		}
		std::sort(coords.begin(), coords.end(), ordering);
		int p_o[10][10];
		for(int i = 0; i < 100; ++ i)
			p_o[coords[i].first][coords[i].second] = i;
		printf("%s\n", p_s_or_name);
		for(int y = 0; y < 10; ++ y) {
			for(int x = 0; x < 10; ++ x)
				printf(" %3d", p_o[x][y]);
			printf("\n");
		}
		printf("\n");
	}

	/**
	 *	@brief runs matrix allocation (and some others) benchmarks
	 *	@tparam n_block_size is matrix block size for the given benchmark
	 *	@param[in] p_s_benchmark is name of a text file, containing
	 *		the filenames of matrices to be tested on
	 *	@return Returns true on success, false on failure.
	 */
	template <const int n_block_size>
	bool Run_AllocationBenchmark(const char *p_s_benchmark)
	{
		printf("Run_AllocationBenchmark() with \'%s\'\n", p_s_benchmark);

		try {
			switch(m_n_ordering_type) {
			case order_TriangleOutline:
				printf("testing with order_TriangleOutline, block_Size = %d\n", n_block_size);
				return Iterate_Matrices(p_s_benchmark, CBenchmarkAllocationTime<n_block_size, CCompareTriangleFrontline>(this));
			case order_RectOutline:
				printf("testing with order_RectOutline, block_Size = %d\n", n_block_size);
				return Iterate_Matrices(p_s_benchmark, CBenchmarkAllocationTime<n_block_size, CCompareRectFrontline>(this));
			default:
			case order_ColumnMajor:
				printf("testing with order_ColumnMajor (default), block_Size = %d\n", n_block_size);
				return Iterate_Matrices(p_s_benchmark, CBenchmarkAllocationTime<n_block_size, CComparePairByFirst>(this));
			case order_RowMajor:
				printf("testing with order_RowMajor, block_Size = %d\n", n_block_size);
				return Iterate_Matrices(p_s_benchmark, CBenchmarkAllocationTime<n_block_size, CComparePairBySecond>(this));
			};
		} catch(std::exception &r_exc) {
			fprintf(stderr, "error: uncaught exception: \'%s\' during"
				" the benchmarks. results invalid.\n", r_exc.what());
			for(typename CCumTimerMap::iterator p_timer_it = m_timer_map.begin(), p_end_it = m_timer_map.end();
			   p_timer_it != p_end_it; ++ p_timer_it) {
				const char *p_s_label = (*p_timer_it).first;
				TCumTimingInfo &r_t_timer = (*p_timer_it).second;
				if(r_t_timer.b_span_started)
					fprintf(stderr, "error: the exception was fired when timer \'%s\' was active\n", p_s_label);
			}
			m_timer_map.clear();
			return false;
		}
	}

	/**
	 *	@brief runs matrix allocation (and some others) benchmarks
	 *	@tparam n_block_size is matrix block size for the given benchmark
	 *	@param[in] p_s_benchmark is name of a text file, containing
	 *		the filenames of matrices to be tested on
	 *	@return Returns true on success, false on failure.
	 */
	template <const int n_block_size>
	bool Run_FactorizationBenchmark(const char *p_s_benchmark)
	{
		printf("Run_FactorizationBenchmark() with \'%s\'\n", p_s_benchmark);

		try {
			printf("testing with order_ColumnMajor (default), block_Size = %d\n", n_block_size);
			return Iterate_Matrices(p_s_benchmark, CBenchmarkCholesky<n_block_size, CComparePairByFirst>(this));
		} catch(std::exception &r_exc) {
			fprintf(stderr, "error: uncaught exception: \'%s\' during"
				" the benchmarks. results invalid.\n", r_exc.what());
			for(typename CCumTimerMap::iterator p_timer_it = m_timer_map.begin(), p_end_it = m_timer_map.end();
			   p_timer_it != p_end_it; ++ p_timer_it) {
				const char *p_s_label = (*p_timer_it).first;
				TCumTimingInfo &r_t_timer = (*p_timer_it).second;
				if(r_t_timer.b_span_started)
					fprintf(stderr, "error: the exception was fired when timer \'%s\' was active\n", p_s_label);
			}
			m_timer_map.clear();
			return false;
		}
	}

	/**
	 *	@brief saves benchmark results to a file
	 *	@param[in] p_s_filename is name of a text file for the results
	 *	@return Returns true on success, false on failure.
	 */
	bool Save_ResultSheet(const char *p_s_filename) const;

	/**
	 *	@brief saves benchmark results to a file
	 *	@param[in] p_s_filename is name of a text file for the results
	 *	@return Returns true on success, false on failure.
	 */
	bool Save_TestBased_ResultSheet(const char *p_s_filename) const;

	/**
	 *	@brief reads in a text file, all at once
	 *
	 *	@param[out] r_s_output is contents of the file
	 *	@param[in] p_s_filename is name of the file to be read
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool ReadFile(std::string &r_s_output, const char *p_s_filename);

	/**
	 *	@brief displays cumulative results to stdout
	 */
	void Show_Cumulative_Results() const;

	/**
	 *	@brief saves sample-based results to a file
	 *	@param[in] p_fw is a file to write results to
	 */
	void Show_SampleBased_Results(FILE *p_fw = stdout) const;

	/**
	 *	@brief saves test-based results to a file
	 *	@param[in] p_fw is a file to write results to
	 *	@todo add timer categories for automatic comparison (or make a function with a comparison table)
	 */
	void Show_TestBased_Results(FILE *p_fw = stdout) const;

	/**
	 *	@brief prints sample-based results to stdout
	 */
	void ShowResults() const;

	/**
	 *	@brief starts a timer
	 *	@param[in] p_s_label is label of the timer
	 *	@note This function throws std::bad_alloc.
	 */
	void StartTimer(const char *p_s_label); // throw(std::bad_alloc)

	/**
	 *	@brief begins a test
	 *	@param[in] r_s_label is name of the test
	 *	@note This function throws std::bad_alloc.
	 */
	void BeginTest(const std::string &r_s_label); // throw(std::bad_alloc)

	/**
	 *	@brief sets test property
	 *	@param[in] p_s_label is name of the test property
	 *	@param[in] f_value is value of the test property
	 *	@note This function throws std::bad_alloc.
	 */
	void SetTestProperty(const char *p_s_label, double f_value); // throw(std::bad_alloc)

	/**
	 *	@brief ends a test
	 *	@param[in] r_s_label is name of the test
	 */
	void EndTest(const std::string &UNUSED(r_s_label));

	/**
	 *	@brief stops timer, generates a sample and updates cumulative time
	 *	@param[in] p_s_label is label of the timer
	 *	@note This function throws std::bad_alloc.
	 */
	void StopTimer(const char *p_s_label);

	/**
	 *	@brief cancels timer (much like stopping it, only no samples are generated)
	 *	@param[in] p_s_label is label of the timer
	 *	@note This function throws std::bad_alloc.
	 */
	void CancelTimer(const char *p_s_label);

	/**
	 *	@brief pauses a running timer (for multi-pass multi-timer loops)
	 *
	 *	This does not generate timer sample but updates cumulative time
	 *	without increasing sample count.
	 *
	 *	@param[in] p_s_label is label of the timer
	 *	@note This function throws std::bad_alloc.
	 *	@note Paused timer can be resumed using StartTimer().
	 */
	void PauseTimer(const char *p_s_label);

	/**
	 *	@brief gets number of test passes
	 *	@return Returns number of test passes.
	 */
	size_t n_Pass_Num() const;

	/**
	 *	@brief iterates matrices from a benchmark file
	 *
	 *	@tparam CFunctor is type of function object for processing matrices (a benchmark function)
	 *
	 *	@param[in] p_s_benchmark is name of a text file, containing
	 *		the filenames of matrices to be tested on
	 *	@param[in] matrix_sink is a benchmark function
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	template <class CFunctor>
	bool Iterate_Matrices(const char *p_s_benchmark, CFunctor matrix_sink) // throw(std::bad_alloc)
	{
		std::string s_bench = p_s_benchmark;
		if(!s_bench.empty() && s_bench[s_bench.length()] != '/' && s_bench[s_bench.length()] != '\\')
			s_bench += '/';
		// make sure there's path separator

		FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fr, (s_bench + "/list.txt").c_str(), "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fr = fopen((s_bench + "/list.txt").c_str(), "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		// open benchmark file

		std::string s_line;
		TRawMatrix raw_trip_matrix; // reuse once-allocated storage
		while(!feof(p_fr)) {
			if(!CParserBase::ReadLine(s_line, p_fr)) {
				fclose(p_fr);
				return false;
			}
			CParserBase::TrimSpace(s_line);
			if(s_line.empty())
				continue;
			s_line = s_bench + s_line;
			// get file with matrix

			printf("testing \'%s\' ...\n", s_line.c_str());
			// verbose

			if(!Load_Matrix(raw_trip_matrix, s_line.c_str())) {
				fprintf(stderr, "warning: failed to load \'%s\'\n", s_line.c_str());
				continue;
			}
			// load the matrix in triplet form

			if(!matrix_sink((const TRawMatrix&)raw_trip_matrix, s_line.c_str())) {
				fprintf(stderr, "warning: test failed on \'%s\'\n", s_line.c_str());
				continue;
			}
		}

		fclose(p_fr);

		return true;
	}

	/**
	 *	@brief loads matrix from a .mat file
	 *
	 *	@param[out] r_trip_matrix is raw triplet matrix (will be overwritten)
	 *	@param[in] p_s_filename is name of file with the matrix
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Load_Matrix(TRawMatrix &r_trip_matrix, const char *p_s_filename);
};

#endif // !__UBER_BLOCK_MATRIX_BENCHMARK_INCLUDED
