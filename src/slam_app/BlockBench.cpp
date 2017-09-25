/*
								+--------------------------------+
								|                                |
								| *** Block matrix benchmark *** |
								|                                |
								| Copyright (c) -tHE SWINe- 2012 |
								|                                |
								|         BlockBench.cpp         |
								|                                |
								+--------------------------------+
*/

/**
 *	@file src/slam_app/BlockBench.cpp
 *	@date 2012
 *	@author -tHE SWINe-
 *	@brief block matrix benchmark
 */

#include "slam_app/BlockBench.h"

/*
 *								=== CBlockMatrixBenchmark ===
 */

CBlockMatrixBenchmark::CBlockMatrixBenchmark(size_t n_pass_num, int n_ordering_type)
	:m_n_pass_num(n_pass_num), m_b_test_active(false), m_n_ordering_type(n_ordering_type)
{
	_ASSERTE(n_ordering_type >= order_ColumnMajor && n_ordering_type < order_MaxOrder);
	m_timer.Reset();
}

void CBlockMatrixBenchmark::TestBlockOrderings()
{
	TestBlockOrdering(CCompareTriangleFrontline(), "CCompareTriangleFrontline");
	TestBlockOrdering(CCompareRectFrontline(), "CCompareRectFrontline");
	TestBlockOrdering(CComparePairByFirst(), "CComparePairByFirst");
	TestBlockOrdering(CComparePairBySecond(), "CComparePairBySecond");
}

bool CBlockMatrixBenchmark::Save_ResultSheet(const char *p_s_filename) const
{
	FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fw, p_s_filename, "w"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fw = fopen(p_s_filename, "w")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open file

	Show_SampleBased_Results(p_fw);
	// write results

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}
	fclose(p_fw);
	// close file

#if defined(_WIN32) || defined(_WIN64)
	std::string s_file;
	if(!ReadFile(s_file, p_s_filename))
		return false;
	std::replace(s_file.begin(), s_file.end(), '.', ',');
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fw, p_s_filename, "wb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fw = fopen(p_s_filename, "wb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	fprintf(p_fw, "%s\n", s_file.c_str());
	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}
	fclose(p_fw);
#endif // _WIN32 || _WIN64
	// in windows, replace '.' to ',' to be usable in excel

	return true;
}

bool CBlockMatrixBenchmark::Save_TestBased_ResultSheet(const char *p_s_filename) const
{
	FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fw, p_s_filename, "w"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fw = fopen(p_s_filename, "w")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open file

	Show_TestBased_Results(p_fw);
	// write results

	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}
	fclose(p_fw);
	// close file

#if defined(_WIN32) || defined(_WIN64)
	std::string s_file;
	if(!ReadFile(s_file, p_s_filename))
		return false;
	std::replace(s_file.begin(), s_file.end(), '.', ',');
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fw, p_s_filename, "wb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fw = fopen(p_s_filename, "wb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	fprintf(p_fw, "%s\n", s_file.c_str());
	if(ferror(p_fw)) {
		fclose(p_fw);
		return false;
	}
	fclose(p_fw);
#endif // _WIN32 || _WIN64
	// in windows, replace '.' to ',' to be usable in excel

	return true;
}

bool CBlockMatrixBenchmark::ReadFile(std::string &r_s_output, const char *p_s_filename)
{
	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	int n_length;
	if(fseek(p_fr, 0, SEEK_END) ||
	   (n_length = ftell(p_fr)) < 0 ||
	   fseek(p_fr, 0, SEEK_SET)) {
		fclose(p_fr);
		return false;
	}
	try {
		r_s_output.resize(n_length);
	} catch(std::bad_alloc&) {
		fclose(p_fr);
		return false;
	}
	if(fread(&r_s_output[0], sizeof(char), n_length, p_fr) != unsigned(n_length)) {
		fclose(p_fr);
		return false;
	}
	fclose(p_fr);
	return true;
}

void CBlockMatrixBenchmark::Show_Cumulative_Results() const
{
	for(CCumTimerMap::const_iterator p_timer_it = m_timer_map.begin(), p_end_it = m_timer_map.end();
	   p_timer_it != p_end_it; ++ p_timer_it) {
		const char *p_s_label = (*p_timer_it).first;
		const TCumTimingInfo &r_t_timer = (*p_timer_it).second;
		_ASSERTE(!r_t_timer.b_span_started);

		printf("timer \'%s\': %g seconds (%d samples)\n", p_s_label, (r_t_timer.n_sample_num)?
			r_t_timer.f_time_total / r_t_timer.n_sample_num : 0, int(r_t_timer.n_sample_num));
	}
}

void CBlockMatrixBenchmark::Show_SampleBased_Results(FILE *p_fw /*= stdout*/) const
{
	_ASSERTE(!m_b_test_active);

	fprintf(p_fw, "test-name");
	for(CNameSet::const_iterator p_lbl_it = m_props_label_set.begin(),
	   p_end_it = m_props_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it)
		fprintf(p_fw, ";%s", *p_lbl_it);
	for(CNameSet::const_iterator p_lbl_it = m_label_set.begin(),
	   p_end_it = m_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it)
		fprintf(p_fw, ";%s", *p_lbl_it);
	fprintf(p_fw, "\n");
	// print all the labels

	for(size_t i = 0, n = m_test_list.size(); i < n; ++ i) {
		const TTestRun &r_test = m_test_list[i];

		size_t n_pos = r_test.s_matrix_name.find_last_of("\\/");
		fprintf(p_fw, "%s", r_test.s_matrix_name.c_str() + ((n_pos == std::string::npos)? 0 : n_pos + 1));
		// print matrix name (filename only)

		for(CNameSet::const_iterator p_lbl_it = m_props_label_set.begin(),
		   p_end_it = m_props_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it) {
			const char *p_s_label = *p_lbl_it;
			CPropsMap::const_iterator p_prop_it = r_test.quantitative_properties.find(p_s_label);
			if(p_prop_it == r_test.quantitative_properties.end())
				fprintf(p_fw, ";");
			else
				fprintf(p_fw, (fabs((*p_prop_it).second) < 1)? ";%g" : ";%f", (*p_prop_it).second);
		}
		// print all the test properties

		for(size_t j = 0, m = 1; j < m; ++ j) { // numbers of lines governed by max sample_list.size() of any label
			if(j) {
				for(size_t k = 0, o = m_props_label_set.size(); k < o; ++ k)
					fprintf(p_fw, ";");
			}
			// print all the colons for all the test props labels (no values on second line and so on)

			const CTimerMap &r_test_samples = r_test.timer_samples;
			for(CNameSet::const_iterator p_lbl_it = m_label_set.begin(),
			   p_end_it = m_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it) {
				const char *p_s_label = *p_lbl_it;
				CTimerMap::const_iterator p_sam_it = r_test_samples.find(p_s_label);
				if(p_sam_it == r_test_samples.end())
					fprintf(p_fw, ";"); // no samples with this label
				else {
					const std::vector<double> &r_sample_list = (*p_sam_it).second.sample_list;
					if(!j && m < r_sample_list.size())
						m = r_sample_list.size();
					if(j < r_sample_list.size())
						fprintf(p_fw, (fabs(r_sample_list[j]) < 1)? ";%g" : ";%f", r_sample_list[j]);
					else
						fprintf(p_fw, ";"); // not enough samples
				}
			}

			fprintf(p_fw, "\n");
		}
		// print all the timers
	}
}

void CBlockMatrixBenchmark::Show_TestBased_Results(FILE *p_fw /*= stdout*/) const
{
	_ASSERTE(!m_b_test_active);

	fprintf(p_fw, "test-name");
	for(CNameSet::const_iterator p_lbl_it = m_props_label_set.begin(),
	   p_end_it = m_props_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it)
		fprintf(p_fw, ";%s", *p_lbl_it);
	fprintf(p_fw, ";samples (min / median / max)");
	for(CNameSet::const_iterator p_lbl_it = m_label_set.begin(),
	   p_end_it = m_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it)
		fprintf(p_fw, ";%s", *p_lbl_it);
	fprintf(p_fw, "\n");
	// print all the labels

	for(size_t i = 0, n = m_test_list.size(); i < n; ++ i) {
		const TTestRun &r_test = m_test_list[i];

		size_t n_pos = r_test.s_matrix_name.find_last_of("\\/");
		fprintf(p_fw, "%s", r_test.s_matrix_name.c_str() + ((n_pos == std::string::npos)? 0 : n_pos + 1));
		// print matrix name (filename only)

		for(CNameSet::const_iterator p_lbl_it = m_props_label_set.begin(),
		   p_end_it = m_props_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it) {
			const char *p_s_label = *p_lbl_it;
			CPropsMap::const_iterator p_prop_it = r_test.quantitative_properties.find(p_s_label);
			if(p_prop_it == r_test.quantitative_properties.end())
				fprintf(p_fw, ";");
			else
				fprintf(p_fw, (fabs((*p_prop_it).second) < 1)? ";%g" : ";%f", (*p_prop_it).second);
		}
		// print all the test properties

		{
			std::vector<size_t> sample_counts;

			const CTimerMap &r_test_samples = r_test.timer_samples;
			for(CNameSet::const_iterator p_lbl_it = m_label_set.begin(),
			   p_end_it = m_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it) {
				const char *p_s_label = *p_lbl_it;
				CTimerMap::const_iterator p_sam_it = r_test_samples.find(p_s_label);
				if(p_sam_it == r_test_samples.end())
					sample_counts.push_back(0); // no samples with this label
				else {
					const std::vector<double> &r_sample_list = (*p_sam_it).second.sample_list;
					sample_counts.push_back(r_sample_list.size());
				}
			}
			// get sample counts for this test and for all the timers

			if(!sample_counts.empty()) {
				std::sort(sample_counts.begin(), sample_counts.end());
				fprintf(p_fw, ";" PRIsize " / " PRIsize " / " PRIsize "", sample_counts[0],
					sample_counts[sample_counts.size() / 2], sample_counts.back());
			} else
				fprintf(p_fw, ";0 / 0 / 0");
		}
		// calculate min / median / max sample counts

		{
			std::vector<size_t> sample_counts;
			std::vector<double> avg_values;
			// for comparisons

			const CTimerMap &r_test_samples = r_test.timer_samples;
			for(CNameSet::const_iterator p_lbl_it = m_label_set.begin(),
			   p_end_it = m_label_set.end(); p_lbl_it != p_end_it; ++ p_lbl_it) {
				const char *p_s_label = *p_lbl_it;
				CTimerMap::const_iterator p_sam_it = r_test_samples.find(p_s_label);
				if(p_sam_it == r_test_samples.end()) {
					sample_counts.push_back(0);
					avg_values.push_back(0);
					fprintf(p_fw, ";"); // no samples with this label
				} else {
					const std::vector<double> &r_sample_list = (*p_sam_it).second.sample_list;
					sample_counts.push_back(r_sample_list.size());

					double f_avg = 0, f_roundoff = 0;
					for(size_t j = 0, m = r_sample_list.size(); j < m; ++ j) {
						double x = r_sample_list[j];
						double y = x - f_roundoff;
						double t = f_avg + y;
						f_roundoff = (t - f_avg) - y; // t and f_avg are similar size, subtraction returns y after rounding
						f_avg = t;
					}
					f_avg /= r_sample_list.size();

					avg_values.push_back(f_avg);

					fprintf(p_fw, (fabs(f_avg) < 1)? ";%.10g" : ";%.10f", f_avg); // the average has higher precision than raw timer samples
				}
			}
			// write all the average samples

			// now it would be a good time to perform pairwise comparison of timers
			// (could employ the expression evaluator to do that or even to calculate
			// compounds - seems like a good idea)

			fprintf(p_fw, "\n");
		}
		// print all the timers
	}
}

void CBlockMatrixBenchmark::ShowResults() const
{
	Show_Cumulative_Results();
}

void CBlockMatrixBenchmark::StartTimer(const char *p_s_label) // throw(std::bad_alloc)
{
	CCumTimerMap::iterator p_map_it = m_timer_map.find(p_s_label);
	if(p_map_it != m_timer_map.end()) {
		TCumTimingInfo &r_t_time = (*p_map_it).second;
		_ASSERTE(!r_t_time.b_span_started);
		r_t_time.f_span_start = m_timer.f_Time(); // discards the span if StopTimer() wasn't called
		r_t_time.b_span_started = true;
	} else {
		TCumTimingInfo t_new;
		t_new.f_span_start = m_timer.f_Time();
		t_new.b_span_started = true;
		t_new.f_time_total = 0;
		t_new.n_sample_num = 0;
		m_timer_map[p_s_label] = t_new;
	}
}

void CBlockMatrixBenchmark::BeginTest(const std::string &r_s_label) // throw(std::bad_alloc)
{
	_ASSERTE(!m_b_test_active);
	if(!m_test_list.empty() && m_test_list.back().s_matrix_name == r_s_label)
		return;
	// this test already running

	m_test_list.resize(m_test_list.size() + 1);
	m_test_list.back().s_matrix_name = r_s_label;
	m_b_test_active = true;
	// add one more test to the end
}

void CBlockMatrixBenchmark::SetTestProperty(const char *p_s_label, double f_value) // throw(std::bad_alloc)
{
	if(m_b_test_active) {
		_ASSERTE(!m_test_list.empty());
		m_props_label_set.insert(p_s_label);
		CPropsMap &r_props_map = m_test_list.back().quantitative_properties;
		r_props_map[p_s_label] = f_value;
	}
}

void CBlockMatrixBenchmark::EndTest(const std::string &UNUSED(r_s_label))
{
	_ASSERTE(!m_test_list.empty() && m_test_list.back().s_matrix_name == r_s_label);
	m_b_test_active = false;
}

void CBlockMatrixBenchmark::StopTimer(const char *p_s_label)
{
	CCumTimerMap::iterator p_map_it = m_timer_map.find(p_s_label);
	if(p_map_it != m_timer_map.end()) {
		TCumTimingInfo &r_t_time = (*p_map_it).second;
		_ASSERTE(r_t_time.b_span_started);
		if(!r_t_time.b_span_started)
			return; // can't reliably count
		double f_time = m_timer.f_Time() - r_t_time.f_span_start; // time sample
		r_t_time.b_span_started = false;
		r_t_time.f_time_total += f_time;
		++ r_t_time.n_sample_num;

		if(m_b_test_active) {
			_ASSERTE(!m_test_list.empty());
			m_label_set.insert(p_s_label);
			CTimerMap &r_timer_map = m_test_list.back().timer_samples;
			r_timer_map[p_s_label].sample_list.push_back(f_time); // collect time samples
		}
	} else
		_ASSERTE(0); // should not happen
}

void CBlockMatrixBenchmark::CancelTimer(const char *p_s_label)
{
	CCumTimerMap::iterator p_map_it = m_timer_map.find(p_s_label);
	if(p_map_it != m_timer_map.end()) {
		TCumTimingInfo &r_t_time = (*p_map_it).second;
		_ASSERTE(r_t_time.b_span_started);
		if(!r_t_time.b_span_started)
			return; // can't reliably count
		//double f_time = m_timer.f_Time() - r_t_time.f_span_start;
		r_t_time.b_span_started = false;
		/*r_t_time.f_time_total += f_time;
		++ r_t_time.n_sample_num;*/
	} else
		_ASSERTE(0); // should not happen
}

void CBlockMatrixBenchmark::PauseTimer(const char *p_s_label)
{
	CCumTimerMap::iterator p_map_it = m_timer_map.find(p_s_label);
	if(p_map_it != m_timer_map.end()) {
		TCumTimingInfo &r_t_time = (*p_map_it).second;
		_ASSERTE(r_t_time.b_span_started);
		if(!r_t_time.b_span_started)
			return; // can't reliably count
		double f_time = m_timer.f_Time() - r_t_time.f_span_start;
		r_t_time.b_span_started = 0;
		r_t_time.f_time_total += f_time;
		//++ r_t_time.n_sample_num; // don't increase sample count yet
	} else
		_ASSERTE(0); // should not happen
}

size_t CBlockMatrixBenchmark::n_Pass_Num() const
{
	return m_n_pass_num;
}

bool CBlockMatrixBenchmark::Load_Matrix(TRawMatrix &r_trip_matrix, const char *p_s_filename)
{
	r_trip_matrix.n_col_num = 0;
	r_trip_matrix.n_row_num = 0;
	r_trip_matrix.data.clear();
	// ...

	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open benchmark file

	bool b_had_header = 0;
	size_t n_rows, n_cols, n_nnz = -1; // initialize to avoid warning

	std::string s_line;
	while(!feof(p_fr)) {
		if(!CParserBase::ReadLine(s_line, p_fr)) {
			fclose(p_fr);
			return false;
		}
		// get file with matrix

		size_t n_pos;
		if((n_pos = s_line.find('%')) != std::string::npos)
			s_line.erase(n_pos);
		CParserBase::TrimSpace(s_line);
		if(s_line.empty())
			continue;
		// trim comments, skip empty lines

		if(!b_had_header) {
			int n_rowsi, n_colsi, n_nnzi;
			if(sscanf(s_line.c_str(), "%d %d %d", &n_rowsi, &n_colsi, &n_nnzi) != 3) {
				fclose(p_fr);
				return false;
			}
			n_rows = n_rowsi;
			n_cols = n_colsi;
			n_nnz = n_nnzi;
			// read header

			if(n_nnz > n_rows * n_cols) {
				fclose(p_fr);
				return false;
			}
			// sanity check (may also fail on big matrices)

			r_trip_matrix.n_col_num = n_cols;
			r_trip_matrix.n_row_num = n_rows;
			r_trip_matrix.data.reserve(n_nnz);
			// alloc the matrix

			b_had_header = true;
		} else {
			TTrip t_trip;
			int n_rowi, n_coli;
			int n = sscanf(s_line.c_str(), "%d %d %lf", &n_rowi, &n_coli, &t_trip.f_value);
			t_trip.n_row = n_rowi;
			t_trip.n_col = n_coli;
			if(n < 2) {
				fclose(p_fr);
				return false;
			} else if(n == 2)
				t_trip.f_value = 1; // a binary matrix
			// read triplet

			-- t_trip.n_row;
			-- t_trip.n_col;
			// indices are 1-based, i.e. a(1,1) is the first element

			if(t_trip.n_row >= r_trip_matrix.n_row_num ||
			   t_trip.n_col >= r_trip_matrix.n_col_num ||
			   r_trip_matrix.data.size() >= n_nnz) {
				fclose(p_fr);
				return false;
			}
			// make sure it figures

			r_trip_matrix.data.push_back(t_trip);
			// append the triplet
		}
	}

	fclose(p_fr);

	return r_trip_matrix.data.size() == n_nnz;
}

/*
 *								=== ~CBlockMatrixBenchmark ===
 */
