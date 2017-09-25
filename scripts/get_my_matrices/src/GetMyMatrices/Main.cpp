#include "../UberLame_src/NewFix.h"
#include "../UberLame_src/CallStack.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <map>
#include <set>
#include "../UberLame_src/proto/HTTP_Download.h"
#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/StdIOUtils.h"
#include "../UberLame_src/Timer.h"
#include "../UberLame_src/Dir.h"
#include "../UberLame_src/RandGen.h"
#include "../UberLame_src/iface/PNGLoad.h"
#include "UFLSMC.h"
#include "FindBlocks.h"

#if defined(_WIN32) || defined(_WIN64)
#ifndef strncasecmp
#define strncasecmp(a,b,n) _strnicmp((a), (b), (n))
#endif // !strncasecmp
#ifndef strcasecmp
#define strcasecmp(a,b) _stricmp((a), (b))
#endif // !strcasecmp
#endif // _WIN32 || _WIN64

class CIsBetween {
protected:
	const size_t m_n_min, m_n_max; // inclusive, exclusive

public:
	CIsBetween(size_t n_min, size_t n_max)
		:m_n_min(n_min), m_n_max(n_max)
	{}

	bool operator ()(size_t n_x) const
	{
		return n_x >= m_n_min && n_x < m_n_max;
	}
};

class CCompareUrl {
protected:
	const std::string &m_r_s_url;

public:
	CCompareUrl(const std::string &r_s_url)
		:m_r_s_url(r_s_url)
	{}

	bool operator ()(const std::pair<std::string, std::string> &r_url_filename) const
	{
		return r_url_filename.first == m_r_s_url;
	}
};

class CCompareToSkipList {
protected:
	const std::string &m_r_s_group;
	const std::string &m_r_s_matrix;

public:
	CCompareToSkipList(const std::string &r_s_group, const std::string &r_s_matrix)
		:m_r_s_group(r_s_group), m_r_s_matrix(r_s_matrix)
	{}

	bool operator ()(const char *p_s_skip_pattern) const
	{
		if(!p_s_skip_pattern || !*p_s_skip_pattern)
			return false;

		const size_t l = strlen(p_s_skip_pattern);
		const char *p_s_slash = strchr(p_s_skip_pattern, '/');
		if(!p_s_slash)
			return false; // need a slash

		bool b_group_match = (l > 2 && *p_s_skip_pattern == '*' && p_s_slash == p_s_skip_pattern + 1) || // either the part before slash is an asterisk
			(m_r_s_group.length() == p_s_slash - p_s_skip_pattern && // length must match the string preceding slash
			std::mismatch(p_s_skip_pattern, p_s_slash, m_r_s_group.begin()).first == p_s_slash); // and it must be the same string

		bool b_matrix_match = !strcmp(p_s_slash + 1, "*") || // either the part after slash is an asterisk
			(m_r_s_matrix == p_s_slash + 1); // or just compare the part after slash

		return b_group_match && b_matrix_match;
	}
};

class CCountFilesSize {
protected:
	uint64_t m_n_size;

public:
	CCountFilesSize()
		:m_n_size(0)
	{}

	bool operator ()(const TFileInfo &r_t_file)
	{
		if(!r_t_file.b_directory)
			m_n_size = min(m_n_size, UINT64_MAX - r_t_file.n_Size64()) + r_t_file.n_Size64(); // saturated add
		return true;
	}

	uint64_t n_Size() const
	{
		return m_n_size;
	}
};

enum {
	max_Threads = 16,
	tag_Folder = 1
};

/*void TrimSpace(std::string &r_s_string)
{
	int b = 0, e = r_s_string.length();
	while(e > 0 && isspace(uint8_t(r_s_string[e - 1])))
		-- e;
	while(b < e && isspace(uint8_t(r_s_string[b])))
		++ b;
	r_s_string.erase(e);
	r_s_string.erase(0, b);
}

bool ReadLine(FILE *p_fr, std::string &r_s_line, int &r_n_cur_line)
{
	while(!feof(p_fr)) {
		r_s_line.erase();
		try {
			for(int c = fgetc(p_fr); c != '\n' && c != EOF; c = fgetc(p_fr))
				r_s_line += c;
		} catch(std::bad_alloc&) {
			return false;
		}
		// read line

		++ r_n_cur_line;
		// line counter for file debugging

		if(r_s_line.find('#') != std::string::npos)
			r_s_line.erase(r_s_line.find('#'));
		// throw away line comment

		TrimSpace(r_s_line);
		// throw away begin / end whitespace

		if(r_s_line.empty())
			continue;
		// skip empty lines

		return true;
	}

	return false;
}*/

bool ReadFile(std::string &r_s_output, const char *p_s_filename)
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
	if(!stl_ut::Resize_To_N(r_s_output, n_length)) {
		fclose(p_fr);
		return false;
	}
	if(fread(&r_s_output[0], sizeof(char), n_length, p_fr) != n_length) {
		fclose(p_fr);
		return false;
	}
	fclose(p_fr);
	return true;
}

bool On_FolderDownload(std::vector<std::string> &dir_download_list,
	std::vector<std::pair<std::string, std::string> > &file_download_list,
	CThreadedHTTPDownloader &r_downloader, std::set<std::string> &folder_fail_list)
{
	_ASSERTE(r_downloader.b_IsAssigned() && r_downloader.n_Tag() == tag_Folder);

	if(!r_downloader.WaitForFinish()) {
		fprintf(stderr, "fatal error: failed to leave critical section\n");
		return false;
	}
	int n_result = r_downloader.n_GetResult(true);
	if(n_result != CHTTPDownloader::down_Success) {
		CPath::Remove_File(r_downloader.p_s_FileName()); // don't leave rubbish ...
		if(folder_fail_list.find(r_downloader.r_s_URL()) != folder_fail_list.end()) {
			fprintf(stderr, "error: failed to download \'%s\'\n", r_downloader.p_s_URL());
			return false;
		} else {
			folder_fail_list.insert(r_downloader.r_s_URL());
			fprintf(stderr, "warning: failed to download \'%s\' (rescheduling)\n", r_downloader.p_s_URL());
			return true;
		}
	}
	// download a directory file (a .HTML)

	std::string s_file;
	if(!ReadFile(s_file, r_downloader.p_s_FileName())) {
		fprintf(stderr, "error: failed to read \'%s\'\n", r_downloader.p_s_FileName());
		CPath::Remove_File(r_downloader.p_s_FileName()); // don't leave rubbish ...
		return true;
	}
	CPath::Remove_File(r_downloader.p_s_FileName()); // don't leave rubbish ...
	// read the file
 
	{
		size_t n_pos = 0;
		while((n_pos = s_file.find("<img src=\"/icons/folder.gif\" "
		   "alt=\"[DIR]\"> <a href=\"", n_pos)) != std::string::npos) {
			size_t n_url_start = n_pos + strlen("<img src=\"/icons/folder.gif\" "
				"alt=\"[DIR]\"> <a href=\"");
			size_t n_url_end = s_file.find('\"', n_url_start + 1);
			n_pos = n_url_end + 1;
			if(n_url_end == std::string::npos) {
				fprintf(stderr, "warning: \'%s\' contains broken link\n", r_downloader.p_s_URL());
				continue;
			}
			// find an URL

			std::string s_new_url = r_downloader.r_s_URL();
			size_t n_index;
			if((n_index = s_new_url.rfind("index.html")) == s_new_url.length() - strlen("index.html") ||
			   (n_index = s_new_url.rfind("index.htm")) == s_new_url.length() - strlen("index.htm") ||
			   (n_index = s_new_url.rfind("index.php")) == s_new_url.length() - strlen("index.php"))
				s_new_url.erase(n_index);
			if(s_new_url.empty() || *(s_new_url.end() - 1) != '/')
				s_new_url += '/';
			s_new_url.insert(s_new_url.end(), s_file.begin() + n_url_start, s_file.begin() + n_url_end);
			// build a new URL

			dir_download_list.push_back(s_new_url);
			// add it to the list ...
		}
	}
	// find all subdirectory downloads
 
	{
		size_t n_pos = 0, e = s_file.length();
		while((n_pos = s_file.find("<img src=\"/icons/", n_pos)) != std::string::npos) {
			n_pos += strlen("<img src=\"/icons/");
			// skip the start of the image ...

			size_t n_type_start = n_pos;
			while(n_pos != e && s_file[n_pos] != '.' && s_file[n_pos] != '\"')
				++ n_pos;
			size_t n_type_end = n_pos;
#ifdef _DEBUG
			std::string s_file_type;
			s_file_type.insert(s_file_type.begin(), s_file.begin() +
				n_type_start, s_file.begin() + n_type_end);
#endif // _DEBUG
			// skip file type ...

			if(s_file.find(".gif\" alt=\"[   ]\"> <a href=\"", n_pos) != n_pos)
				continue;
			// skip the end of the HTML code for the link

			size_t n_url_start = n_pos + strlen(".gif\" alt=\"[   ]\"> <a href=\"");
			size_t n_url_end = s_file.find('\"', n_url_start + 1);
			n_pos = n_url_end + 1;
			if(n_url_end == std::string::npos) {
				fprintf(stderr, "warning: \'%s\' contains broken link\n", r_downloader.p_s_URL());
				continue;
			}
			// find an URL

			std::string s_new_url = r_downloader.r_s_URL();
			size_t n_index;
			if((n_index = s_new_url.rfind("index.html")) == s_new_url.length() - strlen("index.html") ||
			   (n_index = s_new_url.rfind("index.htm")) == s_new_url.length() - strlen("index.htm") ||
			   (n_index = s_new_url.rfind("index.php")) == s_new_url.length() - strlen("index.php"))
				s_new_url.erase(n_index);
			if(s_new_url.empty() || *(s_new_url.end() - 1) != '/')
				s_new_url += '/';
			s_new_url.insert(s_new_url.end(), s_file.begin() + n_url_start, s_file.begin() + n_url_end);
			// build a new URL

			std::string s_filename;
			s_filename.insert(s_filename.end(), s_file.begin() + n_url_start, s_file.begin() + n_url_end);
			// get filename

			file_download_list.push_back(std::make_pair(s_new_url, s_filename));
			// add it to the list ...
		}
		/*n_pos = 0;
		while((n_pos = s_file.find("<img src=\"/icons/unknown.gif\" "
		   "alt=\"[   ]\"> <a href=\"", n_pos)) != std::string::npos) {
			size_t n_url_start = n_pos + strlen("<img src=\"/icons/unknown.gif\" "
				"alt=\"[   ]\"> <a href=\"");
			size_t n_url_end = s_file.find('\"', n_url_start + 1);
			n_pos = n_url_end + 1;
			if(n_url_end == std::string::npos) {
				fprintf(stderr, "warning: \'%s\' contains broken link\n");
				continue;
			}
			// find an URL

			std::string s_new_url = r_downloader.r_s_URL();
			size_t n_index;
			if((n_index = s_new_url.rfind("index.html")) == s_new_url.length() - strlen("index.html") ||
			   (n_index = s_new_url.rfind("index.htm")) == s_new_url.length() - strlen("index.htm") ||
			   (n_index = s_new_url.rfind("index.php")) == s_new_url.length() - strlen("index.php"))
				s_new_url.erase(n_index);
			if(s_new_url.empty() || *(s_new_url.end() - 1) != '/')
				s_new_url += '/';
			s_new_url.insert(s_new_url.end(), s_file.begin() + n_url_start, s_file.begin() + n_url_end);
			// build a new URL

			std::string s_filename;
			s_filename.insert(s_filename.end(), s_file.begin() + n_url_start, s_file.begin() + n_url_end);
			// get filename

			file_download_list.push_back(std::make_pair(s_new_url, s_filename));
			// add it to the list ...
		}*/
	}
	// find all file downloads

	return true;
}

#if defined(_WIN32) || defined(_WIN64)

class CTitleUpdaterThread : public CRunable {
protected:
	CThread m_thread;
	HWND m_h_console;
	bool m_b_run;

public:
	CTitleUpdaterThread()
		:m_b_run(true)
	{
		m_thread.AttachRunable(*this);
		system("title Get My Matrices matrix downloader");
		m_h_console = FindWindow(0, "Get My Matrices matrix downloader");
	}

	bool Start()
	{
		m_b_run = true;
		return m_thread.Start();
	}

	bool Stop()
	{
		m_b_run = false;
		return m_thread.Stop(false);
	}

protected:
	virtual void Run()
	{
		CTimer timer;
		std::string s_title;
		while(m_b_run) {
			double f_time = timer.f_Time();
			uint64_t n_downloaded = CHTTP_Traffic::n_Traffic_Down();
			stl_ut::Format(s_title, "Get My Matrices matrix downloader : downloaded "
				PRIsizeB "B, avg speed: " PRIsizeB "B/sec",
				PRIsizeBparams(n_downloaded), PRIsizeBparams(n_downloaded / f_time));
			SetWindowText(m_h_console, s_title.c_str());
			Sleep(1000);
		}
	}
};

#else // _WIN32 || _WIN64

class CTitleUpdaterThread { // dummy
protected:
public:
	bool Start()
	{
		return true;
	}

	bool Stop()
	{
		return true;
	}
};

#endif // _WIN32 || _WIN64

enum {
	format_Matlab,
	format_MatrixMarket,
	format_RutherfordBoeing
};

void Partition_Test()
{
	size_t p_bss[] = {2, 3, 7, 22};
	const std::set<size_t> bs_set(p_bss, p_bss + sizeof(p_bss) / sizeof(p_bss[0]));

	const size_t p_test_set[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
		12, 100, 123, 200, 220, 230, 1000, 12345, 1U << 31};

	for(size_t n_test = 0; n_test < sizeof(p_test_set) / sizeof(p_test_set[0]); ++ n_test) {
		const size_t n = p_test_set[n_test];

		std::vector<size_t> mults;
		bool b_result = Partition(n, bs_set, &mults);
		bool b_result1 = Partition(n, bs_set);
		_ASSERTE(b_result == b_result1); // make sure it does the same thing with or without
		if(b_result) {
			printf(PRIsize " =", n);
			size_t n_test = 0;
			std::set<size_t>::const_iterator p_bs_it = bs_set.begin();
			bool b_first = true;
			for(size_t i = 0, m = mults.size(); i < m; ++ i, ++ p_bs_it) {
				if(mults[i]) {
					printf(" + " PRIsize " * (" PRIsize ")" + ((b_first)? 2 : 0), mults[i], *p_bs_it);
					b_first = false;
				}
				n_test += mults[i] * *p_bs_it;
			}
			printf("\n");
			_ASSERTE(n_test == n);
		} else
			printf(PRIsize " cannot be written as a sum of the available block sizes\n", n);
	}
}

void BlockLayout_Subdivision_Test()
{
	size_t p_bss[] = {2, 3, 7, 22};
	const std::set<size_t> bs_set(p_bss, p_bss + sizeof(p_bss) / sizeof(p_bss[0]));

	const size_t p_test_block_layout[] = {0, 10, 20, 22, 53, 66, 79, 80};
	std::vector<size_t> block_layout(p_test_block_layout, p_test_block_layout +
		sizeof(p_test_block_layout) / sizeof(p_test_block_layout[0]));

	printf("block column:");
	for(size_t i = 1, n = block_layout.size(); i < n; ++ i) {
		size_t n_base = block_layout[i - 1], n_width = block_layout[i] - n_base;
		printf(" " PRIsize, n_width);
	}
	printf("\n");

	for(size_t i = 1, n = block_layout.size(); i < n; ++ i) {
		size_t n_base = block_layout[i - 1], n_width = block_layout[i] - n_base;
		std::vector<size_t> block_counts;
		if(Partition(n_width, bs_set, &block_counts)) {
			i = n_Subdivide_BlockLayout_Parition(block_layout, i, bs_set, block_counts);
		} else
			fprintf(stderr, "error: parition of block column %d (width %d) failed\n", int(i), int(n_width));
	}
	printf("subdivided c:");
	for(size_t i = 1, n = block_layout.size(); i < n; ++ i) {
		size_t n_base = block_layout[i - 1], n_width = block_layout[i] - n_base;
		printf(" " PRIsize, n_width);
	}
	printf("\n");
}

void BlockLayout_Modification_Test()
{
	TSparse mat;
	mat.Load_MatrixMarket("dest/"/*"FIDAP_ex2"*//*"HB_bcsstk10"*/"HB_lund_a"".mtx");
	std::vector<size_t> block_rows, block_cols;
	TBlockInfo::Load_BlockLayout("dest/"/*"FIDAP_ex2"*//*"HB_bcsstk10"*/"HB_lund_a"".bla", block_rows, block_cols);

	{
		TBmp *p_bmp = mat.p_Draw_BlockLayout(block_rows, block_cols);
		if(p_bmp) {
			CPngCodec::Save_PNG("blocks_orig.png", *p_bmp);
			p_bmp->Delete();
		}
	}

	size_t p_bss[] = {2, 3/*2, 3*//*4, 5*/};
	const std::vector<size_t> bs_vec(p_bss, p_bss + sizeof(p_bss) / sizeof(p_bss[0]));

	if(!Modify_BlockLayout_Greedy(block_rows, block_cols, mat, bs_vec, 10))
		fprintf(stderr, "error: failed to modify the block layout");

	{
		TBmp *p_bmp = mat.p_Draw_BlockLayout(block_rows, block_cols);
		if(p_bmp) {
			CPngCodec::Save_PNG("blocks_improved.png", *p_bmp);
			p_bmp->Delete();
		}
	}
}

int main(int UNUSED(n_arg_num), const char **UNUSED(p_arg_list))
{
	bool b_download_mode = (n_arg_num == 1); // no arguments? update the matrix database
	size_t n_min_size = 0;
	size_t n_max_size = SIZE_MAX;
	size_t n_min_nonzero = 0;
	size_t n_max_nonzero = SIZE_MAX;
	bool b_want_only_real = false;
	bool b_want_only_square = false;
	bool b_want_only_rect = false;
	bool b_want_only_not_real = false;
	bool b_want_only_binary = false;
	bool b_want_only_not_binary = false;
	bool b_want_only_ND = false;
	bool b_want_only_not_ND = false;
	bool b_want_only_pos_def = false;
	bool b_want_only_not_pos_def = false;
	bool b_want_only_num_sym = false;
	bool b_want_only_not_num_sym = false;
	float f_min_pattern_symmetry = 0;
	float f_max_pattern_symmetry = 1;
	const char *p_s_problem_type = 0;
	bool b_list_problems_types_only = false;
	bool b_list_compression_ratios_only = false;
	const char *p_s_list_matrix_names_only = 0; // contains the separator
	int n_preferred_format = format_MatrixMarket;
	const char *p_s_dest_directory = 0;
	size_t n_max_matrix_num = 0;
	const char *p_s_path_to_ufstats_csv = "sparse/matrices/UFstats.csv";
	const char *p_s_uflsmc_root = "";
	const char *p_s_explicit_temp = 0;
	std::vector<const char*> name_list, skip_list;
	size_t n_block_tol = 0;
	bool b_clean_cache = false, b_pre_clean = false;
	const char *p_s_block_layout_folder = 0;
	const char *p_s_block_size_list = 0;
	bool b_generate_block_layour_only = false;
	size_t n_skip_block_anal_num = 0;

	for(int i = 1; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			printf("use: %s\n"
				"or:  %s --download|dl [--dest-directory|-o <path>] [--temp|-tmp <path>]\n"
				"or:  %s --input|-i <path> [options]\n"
				"\nWhere the first two incantations start the download and the second one searches\n"
				"for matrices in the downloaded corpus. The [options] fall into several groups.\n"

				"\nMatrix selection options:\n"
				"--real | -r - selects only real matrices\n"
				"--complex | -c - selects only complex matrices\n"
				"--binary | -b - selects only binary matrices\n"
				"--not-binary | -nb - selects only matrices with values\n"
				"--ND | -nd - selects only 2D/3D problem matrices\n"
				"--not-ND | -nnd - selects matrices not of 2D/3D problems\n"
				"--pos-def | -pd - selects only positive definite matrices\n"
				"--not-pos-def | -npd - selects only other than positive definite matrices\n"
				"--num-symmetric | -ns - selects only numerically symmetric matrices (implies structural\n"
				"                        symmetry also)\n"
				"--not-num-symmetric | -nns - selects only numerically unsymmetric matrices\n"
				"--min-size | -ms <value> - sets minimum seleced matrix size, in rows/cols\n"
				"--max-size | -xs <value> - sets maximum seleced matrix size, in rows/cols\n"
				"--min-nnz | -mnz <value> - sets minimum seleced matrix nnz, in elements\n"
				"--max-nnz | -xnz <value> - sets maximum seleced matrix nnz, in elements\n"
				"--min-pattern-symmetry | -mps <value> - sets minimum pattern symmetry, in percent\n"
				"--max-pattern-symmetry | -xps <value> - sets maximum pattern symmetry, in percent\n"
				"--square | -sq - selects only square matrices\n"
				"--rect | -rr - selects only rectangular matrices\n"
				"--skip | -s <group>/<matrix> - skips matrices by name, use e.g. \'--skip HB/rgg010\',\n"
				"--name | -n <group>/<matrix> - includes matrices by name, use e.g. \'--name HB/rgg010\',\n"
				"                               note that group or matrix can be a Kleene star \'*\',\n"
				"                               --skip takes precedence over --name\n"
				"--problem-type | -t <value> - selects matrices by problem type name (use -lpt to see\n"
				"                              what problem types there are)\n"
				"--skip-first | -sf <number> - skips first <number> matrices (counted in global UFLSMC\n"
				"                              matrix list order)\n"
				"--block-sizes | -bs <sizes[:fill-in-thresh]> - selects matrices which are conformant\n"
				"                                               with the given block sizes (given as a\n"
				"                                               comma-separated list) and optionally\n"
				"                                               limited by maximum block fill-in (given\n"
				"                                               as a fraction of blocks, e.g. 5 for 20%%\n"
				"                                               is the default)\n"
				"\nOutput options:\n"
				"--list-problem-types | -lpt - list the types of problems that the selected matrices\n"
				"                              would fall in (but do not extract any matrices)\n"
				"--list-matrix-names | -lmn <separator> - list the names of the selected matrices as\n"
				"                                         \"group<separator>matrix\" (but do not extract\n"
				"                                         any matrices)\n"
				"--generate-block-layouts | -gbla - generate matrix block layouts (but do not actually\n"
				"                                   extract any matrices)\n"
				"--max-matrices | -mm <value> - sets maximum number of matrices to select (if selected\n"
				"                               more, will randomly choose this amount of matrices)\n"
				"--dest-directory | -o <value> - sets destination directory for the matrices (it is\n"
				"                                created if it doesn't already exist)\n"

				"\nInput options:\n"
				"--input | -i <value> - sets input path (the same path the downloader had set\n"
				"                       as the destination directory)\n"
				"--block-layout-folder | -blaf - sets input path to the folder with matrix block layours\n"

				"\nCache / misc options:\n"
				"--temp | -tmp <value> - sets explicit temporary directory (otherwise uses system temp)\n"
				"--clean-cache | -cc - cleans the decompressed matrix cache\n"
				"--pre-clean | -pc - cleans the entire temp directory before doing anything else\n"
				"--block-tol | -bt <value> - sets block discovery tolerance to 100%%/value nnz discrepancy\n",
				p_arg_list[0], p_arg_list[0], p_arg_list[0]);
			//printf("no help for you, mwuhahahaha! (seriously, read Main.cpp, please)\n");
			return 0;
		} else if(!strcmp(p_arg_list[i], "--download") || !strcmp(p_arg_list[i], "-dl"))
			b_download_mode = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--list-problem-types") || !strcmp(p_arg_list[i], "-lpt")))
			b_list_problems_types_only = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--list-compression-ratios") || !strcmp(p_arg_list[i], "-lcr")))
			b_list_compression_ratios_only = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--generate-block-layouts") || !strcmp(p_arg_list[i], "-gbla")))
			b_generate_block_layour_only = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--real") || !strcmp(p_arg_list[i], "-r")))
			b_want_only_real = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--square") || !strcmp(p_arg_list[i], "-sq"))) {
			b_want_only_square = true;
			if(b_want_only_rect) {
				fprintf(stderr, "error: --square and --rect are mutually exclusive; ignoring the previos --rect\n");
				b_want_only_rect = false;
			}
		} else if(!b_download_mode && (!strcmp(p_arg_list[i], "--rect") || !strcmp(p_arg_list[i], "-rr"))) {
			b_want_only_rect = true;
			if(b_want_only_square) {
				fprintf(stderr, "error: --square and --rect are mutually exclusive; ignoring the previos --square\n");
				b_want_only_square = false;
			}
		} else if(!b_download_mode && (!strcmp(p_arg_list[i], "--complex") || !strcmp(p_arg_list[i], "-c")))
			b_want_only_not_real = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--clean-cache") || !strcmp(p_arg_list[i], "-cc")))
			b_clean_cache = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--pre-clean") || !strcmp(p_arg_list[i], "-pc")))
			b_pre_clean = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--binary") || !strcmp(p_arg_list[i], "-b")))
			b_want_only_binary = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--not-binary") || !strcmp(p_arg_list[i], "-nb")))
			b_want_only_not_binary = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--ND") || !strcmp(p_arg_list[i], "-nd")))
			b_want_only_ND = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--not-ND") || !strcmp(p_arg_list[i], "-nnd")))
			b_want_only_not_ND = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--pos-def") || !strcmp(p_arg_list[i], "-pd")))
			b_want_only_pos_def = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--not-pos-def") || !strcmp(p_arg_list[i], "-npd")))
			b_want_only_not_pos_def = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--num-symmetric") || !strcmp(p_arg_list[i], "-ns")))
			b_want_only_num_sym = true;
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--not-num-symmetric") || !strcmp(p_arg_list[i], "-nns")))
			b_want_only_not_num_sym = true;
		else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return -1;
		} else if(!b_download_mode && (!strcmp(p_arg_list[i], "--input") || !strcmp(p_arg_list[i], "-i")))
			p_s_uflsmc_root = p_arg_list[++ i];
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--min-size") || !strcmp(p_arg_list[i], "-ms")))
			n_min_size = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--max-size") || !strcmp(p_arg_list[i], "-xs")))
			n_max_size = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--min-nnz") || !strcmp(p_arg_list[i], "-mnz")))
			n_min_nonzero = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--max-nnz") || !strcmp(p_arg_list[i], "-xnz")))
			n_max_nonzero = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--max-matrices") || !strcmp(p_arg_list[i], "-mm")))
			n_max_matrix_num = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--min-pattern-symmetry") || !strcmp(p_arg_list[i], "-mps")))
			f_min_pattern_symmetry = float(atof(p_arg_list[++ i]));
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--max-pattern-symmetry") || !strcmp(p_arg_list[i], "-xps")))
			f_max_pattern_symmetry = float(atof(p_arg_list[++ i]));
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--problem-type") || !strcmp(p_arg_list[i], "-t")))
			p_s_problem_type = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--temp") || !strcmp(p_arg_list[i], "-tmp"))
			p_s_explicit_temp = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--dest-directory") || !strcmp(p_arg_list[i], "-o"))
			p_s_dest_directory = p_arg_list[++ i];
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--block-tol") || !strcmp(p_arg_list[i], "-bt")))
			n_block_tol = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--block-sizes") || !strcmp(p_arg_list[i], "-bs")))
			p_s_block_size_list = p_arg_list[++ i];
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--block-layout-folder") || !strcmp(p_arg_list[i], "-blaf")))
			p_s_block_layout_folder = p_arg_list[++ i];
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--skip") || !strcmp(p_arg_list[i], "-s"))) {
			if(!stl_ut::Resize_Add_1More(skip_list, p_arg_list[++ i])) {
				fprintf(stderr, "error: not enough memory\n");
				return -1;
			}
		} else if(!b_download_mode && (!strcmp(p_arg_list[i], "--skip-first") || !strcmp(p_arg_list[i], "-sf")))
			n_skip_block_anal_num = atol(p_arg_list[++ i]);
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--list-matrix-names") || !strcmp(p_arg_list[i], "-lmn")))
			p_s_list_matrix_names_only = p_arg_list[++ i];
		else if(!b_download_mode && (!strcmp(p_arg_list[i], "--name") || !strcmp(p_arg_list[i], "-n"))) {
			if(!stl_ut::Resize_Add_1More(name_list, p_arg_list[++ i])) {
				fprintf(stderr, "error: not enough memory\n");
				return -1;
			}
		} else {
			fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
			return -1;
		}
	}
	// "parse" cmdline

	std::string s_dest_directory = (p_s_dest_directory)? p_s_dest_directory : (b_download_mode)? "." : "benchmark";
	// choose default dest directory based on whether we're downloading or not

	if(!b_download_mode) {
		if(b_generate_block_layour_only && p_s_block_size_list) { // can't do those at the same time
			fprintf(stderr, "error: --generate-block-layouts must be run as a standalone pass,\n"
				"then run again with --block-sizes while making sure to specify\n"
				"--block-layout-folder pointing to the results\n");
			return -1;
		}
		if(p_s_block_size_list && !p_s_block_layout_folder) {
			fprintf(stderr, "error: --block-sizes requites --block-layout-folder to be set\n");
			return -1;
		}
		std::vector<size_t> block_size_list;
		size_t n_block_size_mod_tol = 5;
		std::string s_block_layout_directory;
		if(p_s_block_size_list) {
			const char *b = p_s_block_size_list, *e = p_s_block_size_list +
				strlen(p_s_block_size_list);
			const char *p_s_thresh;
			if((p_s_thresh = strrchr(p_s_block_size_list, ':')))
				n_block_size_mod_tol = atol((e = p_s_thresh) + 1);
			if(b == e || !isdigit(uint8_t(*b))) {
				fprintf(stderr, "error: invalid block size list specifier\n");
				return -1;
			}
			while(b != e) {
				size_t n_bs = 0;
				while(b < e && isdigit(uint8_t(*b))) {
					n_bs = 10 * n_bs + *b - '0';
					++ b;
				}
				// parse an integer (ignores overflows)

				if(!stl_ut::Resize_Add_1More(block_size_list, n_bs)) {
					fprintf(stderr, "error: not enough memory\n");
					return -1;
				}
				// add it to the list

				if(b == e)
					break;
				if(*b != ',' || b + 1 == e || !isdigit(uint8_t(b[1]))) {
					fprintf(stderr, "error: invalid block size list specifier\n");
					return -1;
				}
				++ b; // skip ','
				// handle sequence continuation
			}

			s_block_layout_directory = p_s_block_layout_folder;
			if(!CPath::b_EndsWithSlash(s_block_layout_directory))
				s_block_layout_directory += CDirectory::path_Separator;
			// make sure the destination directory is empty, or ends with slash
		}
		// parse the block size list

		if(b_pre_clean) {
			std::string s_temp;
			if((!p_s_explicit_temp && !CPath::Get_TempDirectory(s_temp)) ||
			   (p_s_explicit_temp && !stl_ut::AppendCStr(s_temp, p_s_explicit_temp))) {
				fprintf(stderr, "error: failed to resolve temp\n");
				return -1;
			}
			printf("debug: temp for this job is \'%s\'\n", s_temp.c_str());

			uint64_t n_size_before;
			{
				CCountFilesSize cfs;
				if(!CDirTraversal::Traverse(s_temp.c_str(), cfs, true))
					n_size_before = -1;
				else
					n_size_before = cfs.n_Size();
			}

			CPath::CRemoveDirTree remover(false, true, false);
			if(!CDirTraversal::Traverse2(s_temp.c_str(), remover, false))
				fprintf(stderr, "error: failed to traverse the temp directory\n");
			if(remover.b_Failed())
				fprintf(stderr, "warning: had some problems\n");

			uint64_t n_size_after;
			{
				CCountFilesSize cfs;
				if(!CDirTraversal::Traverse(s_temp.c_str(), cfs, true))
					n_size_after = -1;
				else
					n_size_after = cfs.n_Size();
			}

			if(n_size_after != uint64_t(-1) && n_size_before != uint64_t(-1)) {
				printf("debug: removed " PRIsizeB "B of temp files (initially " PRIsizeB "B)\n",
					PRIsizeBparams(n_size_before - n_size_after), PRIsizeBparams(n_size_before));
			} else {
				printf("debug: file sizes unknown or imprecies: " PRIsizeB "B before, " PRIsizeB "B after\n",
					PRIsizeBparams(n_size_before), PRIsizeBparams(n_size_after));
			}
		}
		// clean the temp dir (does not work awfully well on linux, due to insufficient access
		// rights if many users are producing temp files, some dirs will fail to recurse)

		if(!CPath::b_EndsWithSlash(s_dest_directory))
			s_dest_directory += CDirectory::path_Separator;
		// make sure the destination directory is empty, or ends with slash

		size_t n_selected_matrix_num = 0;
		std::set<std::string> problem_type_set;
		std::vector<size_t> selected_matrices;
		CUFLSMC uflsmc(p_s_uflsmc_root, p_s_explicit_temp);
		if(!uflsmc.b_Status()) {
			fprintf(stderr, "error: failed to load UFstats.csv from \'%s/%s\'\n",
				p_s_uflsmc_root, p_s_path_to_ufstats_csv);
			return 0;
		}
		// load the collection

		if(b_clean_cache) {
			if(!uflsmc.Remove_Nonreferenced_MatrixFiles(true))
				fprintf(stderr, "warning: had some issues cleaning the cache\n");
			printf("done cleaning, exitting\n");
			return 0;
		}

		if((!b_list_problems_types_only && !p_s_list_matrix_names_only &&
		   !b_list_compression_ratios_only) || p_s_block_size_list) {
			//CPath::Remove_DirTree(s_dest_directory.c_str(), false); // do not pre-clean
			CPath::Make_Directories(s_dest_directory.c_str());
			//system(("del /s /f /q " + s_dest_directory + "* > " + CPath::p_s_NullPath()).c_str());
			// create/clean the dest directory
		}

		for(size_t i = n_skip_block_anal_num, n = uflsmc.n_Matrix_Num(); i < n; ++ i) {
			const CUFLSMC::TMatrixRecord &r_t_mat = uflsmc.t_Matrix_Info(i);

			const std::string &s_group = r_t_mat.s_group, &s_mat_name = r_t_mat.s_name,
				&s_problem_type = r_t_mat.s_kind;
			size_t n_width = r_t_mat.n_cols, n_height = r_t_mat.n_rows, n_nonzero_num = r_t_mat.n_nnz;
			bool b_real = r_t_mat.b_is_real, b_binary = r_t_mat.b_is_binary,
				b_is_ND = r_t_mat.b_is_2D_3D, b_is_positive_definite = r_t_mat.b_is_pos_def,
				b_numerical_symmetry = r_t_mat.b_is_num_symmetric;
			float f_pattern_symmetry = r_t_mat.f_struct_symmetry;
			// get field values

			if(!name_list.empty() && std::find_if(name_list.begin(), name_list.end(),
			   CCompareToSkipList(s_group, s_mat_name)) == name_list.end())
				continue;
			// only include selected matrices

			if(std::find_if(skip_list.begin(), skip_list.end(),
			   CCompareToSkipList(s_group, s_mat_name)) != skip_list.end()) {
				fprintf(stderr, "debug: skipping %s / %s\n", s_group.c_str(), s_mat_name.c_str());
				continue;
			}
			// apply skip list

			if(n_width < n_min_size || n_height < n_min_size ||
			   n_width > n_max_size || n_height > n_max_size)
				continue;
			if(b_want_only_rect && n_width == n_height)
				continue;
			if(b_want_only_square && n_width != n_height)
				continue;
			if(n_nonzero_num < n_min_nonzero || n_nonzero_num > n_max_nonzero)
				continue;
			if((!b_real && b_want_only_real) || (b_real && b_want_only_not_real))
				continue;
			if((!b_binary && b_want_only_binary) || (b_binary && b_want_only_not_binary))
				continue;
			if((!b_is_ND && b_want_only_ND) || (b_is_ND && b_want_only_not_ND))
				continue;
			if((!b_is_positive_definite && b_want_only_pos_def) ||
			   (b_is_positive_definite && b_want_only_not_pos_def))
				continue;
			if((!b_numerical_symmetry && b_want_only_num_sym) ||
			   (b_numerical_symmetry && b_want_only_not_num_sym))
				continue;
			if(f_pattern_symmetry < f_min_pattern_symmetry ||
			   f_pattern_symmetry > f_max_pattern_symmetry)
				continue;
			if(p_s_problem_type && strcasecmp(s_problem_type.c_str(), p_s_problem_type))
				continue;
			// apply the filter

			/*if(n_skip_block_anal_num > 0) {
				-- n_skip_block_anal_num;
				continue;
			}*/
			// skip the matrices which we already checked

			if(p_s_block_size_list || b_list_compression_ratios_only) {
				printf("\r%59s\ranalysing matrix " PRIsize ": %s / %s ", "", i + 1,
					r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str());
				// verbose

				fflush(stdout);
				// if running on the cluster

				std::string s_dest_bla, s_dest_png_bb;
				std::vector<size_t> block_rows, block_cols;
				if(p_s_block_size_list) {
					if(!stl_ut::Format(s_dest_bla, "%s_%s.bla", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str()) ||
					   !stl_ut::Format(s_dest_png_bb, "%s_%s_bb.png", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str())/* ||
					   !stl_ut::Format(s_dest_png_bb_orig, "%s_%s_bbo.png", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str())*/) {
						fprintf(stderr, "error: not enough memory\n");
						continue;
					}
					for(size_t n_pos = 0; (n_pos = s_dest_bla.find_first_of("\\/", n_pos)) != std::string::npos; ++ n_pos)
						s_dest_bla[n_pos] = '_';
					for(size_t n_pos = 0; (n_pos = s_dest_png_bb.find_first_of("\\/", n_pos)) != std::string::npos; ++ n_pos)
						s_dest_png_bb[n_pos] = '_';
					// prepare file names

					if(!TBlockInfo::Load_BlockLayout((s_block_layout_directory + s_dest_bla).c_str(), block_rows, block_cols))
						continue; // no block layout for this matrix
				}

				std::string s_file;
				if(!uflsmc.Get_MatrixFile(s_file, i)) {
					fprintf(stderr, "warning: decompress failed: cleaning up more space\n");
					uflsmc.Remove_Nonreferenced_MatrixFiles(true);
					// try to delete some files left behind

					if(!uflsmc.Get_MatrixFile(s_file, i)) { // i, not selected_matrices[i]!
						fprintf(stderr, "error: failed to decompress matrix " PRIsize " (%s/%s), won't try again\n",
							i, r_t_mat.s_group.c_str(),
							r_t_mat.s_name.c_str());
						continue;
					}
					// try to load again
				}
				CUFLSMC::CUnrefGuard unref(uflsmc, i);
				// a simple guard object to automatically unreference the matrix file

				uint64_t n_matrix_size_csc, n_matrix_size_coo;
				if(0) {
					TSparse::TMMHeader t_header;
					t_header.b_binary = r_t_mat.b_is_binary;
					t_header.b_symmetric = r_t_mat.b_is_struct_symmetric;
					t_header.n_nnz = r_t_mat.n_nnz;
					t_header.n_full_nnz = r_t_mat.n_nnz;
					t_header.n_rows = r_t_mat.n_rows;
					t_header.n_cols = r_t_mat.n_cols;
					n_matrix_size_csc = t_header.n_AllocationSize_CSC();
					n_matrix_size_coo = t_header.n_AllocationSize_COO();
				} else {
					TSparse::TMMHeader t_header;
					if(!TSparse::Peek_MatrixMarket_Header(s_file.c_str(), t_header)) {
						fprintf(stderr, "warning: failed to read the header of \'%s\'\n", s_file.c_str());
						continue;
					}
					n_matrix_size_csc = t_header.n_AllocationSize_CSC();
					n_matrix_size_coo = t_header.n_AllocationSize_COO();
				}
				// don't have to actually read the file to do that

				if(b_list_compression_ratios_only ||
				   n_matrix_size_csc > 2U * 1024 * 1024 * 1024 && n_matrix_size_csc != uint64_t(-1)) {
					if(b_list_compression_ratios_only) {
						printf(PRIsizeB "B CSC, " PRIsizeB "B COO, %.3f:1 compression\n",
							PRIsizeBparams(n_matrix_size_csc),
							PRIsizeBparams(n_matrix_size_coo),
							n_matrix_size_coo / double(n_matrix_size_csc));
					} else {
						printf("(" PRIsizeB "B in CSC / " PRIsizeB "B COO)\ranalysing matrix " PRIsize ": %s / %s ",
							PRIsizeBparams(n_matrix_size_csc),
							PRIsizeBparams(n_matrix_size_coo), i + 1, r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str());
					}
					fflush(stdout); // this stays there for some time
				}
				// verbose - print matrix size if large

				if(b_list_compression_ratios_only)
					continue;
				// just list those and go away

				TSparse mat;
				try {
					if(!mat.Load_MatrixMarket(s_file.c_str())) {
						fprintf(stderr, "warning: failed to load \'%s\'\n", s_file.c_str());
						continue;
					}
				} catch(std::bad_alloc&) {
					fprintf(stderr, "warning: got std::bad_alloc while loading \'%s\'\n", s_file.c_str());
					continue;
				} catch(std::exception &r_exc) {
					fprintf(stderr, "warning: got std::exception while loading "
						"\'%s\' (%s)\n", s_file.c_str(), r_exc.what());
					continue;
				}
				// load the matrix

				unref.Trigger();
				// loaded the matrix, don't need the file anymore

				if(Modify_BlockLayout_Greedy(block_rows, block_cols,
				   mat, block_size_list, n_block_size_mod_tol)) {
					TBlockInfo t_bi(mat, block_rows, block_cols); // use this instead, want to know the fill-in figures and to have a list of blocks
					if(!t_bi.Write_BlockLayout((s_dest_directory + s_dest_bla).c_str())) {
					/*if(!TBlockInfo::Write_BlockLayout((s_dest_directory + s_dest_bla).c_str(),
					   mat, block_rows, block_cols)) {*/
						fprintf(stderr, "warning: failed to save \'%s\'\n",
							(s_dest_directory + s_dest_bla).c_str());
					}
					TBmp *p_bmp;
					if((p_bmp = mat.p_Draw_BlockLayout(block_rows, block_cols))) {
						if(!CPngCodec::Save_PNG((s_dest_directory + s_dest_png_bb).c_str(), *p_bmp, false)) {
							fprintf(stderr, "warning: failed to save \'%s\'\n",
								(s_dest_directory + s_dest_png_bb).c_str());
							CPath::Remove_File((s_dest_directory + s_dest_png_bb).c_str());
						}
						p_bmp->Delete();
					}
				} else
					continue; // failed to modify the block layout, dont select this matrix
				// this matrix works with the block sizes we want; add it among the selected ones
				// (might get pruned) and save its block layout to the dest folder so that we don't
				// have to calculate it twice
			}
			// apply the block size list

			++ n_selected_matrix_num;
			if(b_list_problems_types_only)
				problem_type_set.insert(s_problem_type);
			if(p_s_list_matrix_names_only || (!b_list_problems_types_only && !b_list_compression_ratios_only)) {
				selected_matrices.push_back(i);
				// record index
			}
			// add the matrix to the selection
		}

		printf("filtering finished. selected " PRIsize " matrices\n", n_selected_matrix_num);

		if(n_max_matrix_num && n_selected_matrix_num > n_max_matrix_num) {
			CCLibGenerator<false> rg;
			rg.Seed(int(time(0)));

			for(size_t n = selected_matrices.size(); n > n_max_matrix_num; -- n)
				selected_matrices.erase(selected_matrices.begin() + CUniformIntegerDistribution<size_t>(0, n - 1)(rg));
			n_selected_matrix_num = selected_matrices.size();
			_ASSERTE(n_selected_matrix_num <= n_max_matrix_num); // maybe there was not enough

			printf("randomly selected " PRIsize " matrices\n", n_selected_matrix_num);
		} else if(n_max_matrix_num)
			printf("selected all of " PRIsize " matrices\n", n_selected_matrix_num);

		size_t n_read_fail_num = 0, n_decompress_fail_num = 0, n_anal_fail_num = 0;

		if(p_s_list_matrix_names_only) {
			for(size_t i = 0; i < n_selected_matrix_num; ++ i) {
				const CUFLSMC::TMatrixRecord &r_t_mat = uflsmc.t_Matrix_Info(selected_matrices[i]);
				printf("%s%s%s\n", r_t_mat.s_group.c_str(), p_s_list_matrix_names_only, r_t_mat.s_name.c_str());
			}
			// list the matrix names
		}
		if(b_list_problems_types_only) {
			printf("list of problem types (" PRIsize "):\n", problem_type_set.size());
			for(std::set<std::string>::const_iterator p_type_it = problem_type_set.begin(),
			   p_end_it = problem_type_set.end(); p_type_it != p_end_it; ++ p_type_it) {
				const std::string &r_s_type = *p_type_it;
				printf("\t\'%s\'\n", r_s_type.c_str());
			}
		} else if(!p_s_list_matrix_names_only && !b_list_compression_ratios_only) {
			/*if(selected_matrices.size() < n_selected_matrix_num) {
				fprintf(stderr, "warning: not enough matrices (%d only instead of %d)\n",
					selected_matrices.size(), n_selected_matrix_num);
			}*/ // nonsense, must match
			_ASSERTE(n_selected_matrix_num == selected_matrices.size());
			n_selected_matrix_num = selected_matrices.size(); // !!

			CUniqueFILE_Ptr p_list_file;
			if(!p_list_file.Open((s_dest_directory + "list.txt").c_str(), "w")) {
				fprintf(stderr, "warning: unable to open \'%s\': list of matrices won't be available\n",
					(s_dest_directory + "list.txt").c_str());
			}
			// open list file

			if(n_preferred_format != format_MatrixMarket)
				fprintf(stderr, "warning: will use the matrix market format despite that's not what the user wants\n");

			uint64_t n_total_size = 0;
			for(size_t i = 0; i < n_selected_matrix_num; ++ i) {
				const CUFLSMC::TMatrixRecord &r_t_mat = uflsmc.t_Matrix_Info(selected_matrices[i]);
				printf("\r%59s\rprocessing matrix " PRIsize " (" PRIsize "): %s / %s ",
					"", i + 1, selected_matrices[i] + 1, r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str());
				// verbose

				fflush(stdout);
				// if running on the cluster

				std::string s_dest, s_dest_png, s_dest_png_bb, s_dest_bla;
				if(!stl_ut::Format(s_dest, "%s_%s.mtx", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str()) ||
				   !stl_ut::Format(s_dest_png, "%s_%s.png", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str()) ||
				   !stl_ut::Format(s_dest_png_bb, "%s_%s_bb.png", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str()) ||
				   !stl_ut::Format(s_dest_bla, "%s_%s.bla", r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str())) {
					fprintf(stderr, "error: not enough memory\n");
					continue;
				}
				for(size_t n_pos = 0; (n_pos = s_dest.find_first_of("\\/", n_pos)) != std::string::npos; ++ n_pos)
					s_dest[n_pos] = '_';
				for(size_t n_pos = 0; (n_pos = s_dest_png.find_first_of("\\/", n_pos)) != std::string::npos; ++ n_pos)
					s_dest_png[n_pos] = '_';
				for(size_t n_pos = 0; (n_pos = s_dest_png_bb.find_first_of("\\/", n_pos)) != std::string::npos; ++ n_pos)
					s_dest_png_bb[n_pos] = '_';
				for(size_t n_pos = 0; (n_pos = s_dest_bla.find_first_of("\\/", n_pos)) != std::string::npos; ++ n_pos)
					s_dest_bla[n_pos] = '_';
				// prepare file names (need at least s_dest alawys)

				if(b_generate_block_layour_only) {
					{
						TBmp *p_image;
						if(p_image = CPngCodec::p_Load_PNG((s_dest_directory + s_dest_png).c_str())) {
							p_image->Delete();

							if(p_list_file) // may have failed to open
								fprintf(p_list_file, "%s\n", s_dest.c_str());
							// assume that's ok then

							continue;
						}
					}
					// see if we already processed this matrix and if we did, skip it
				}

				std::string s_file;
				if(!uflsmc.Get_MatrixFile(s_file, selected_matrices[i])) {
					fprintf(stderr, "warning: decompress failed: cleaning up more space\n");
					uflsmc.Remove_Nonreferenced_MatrixFiles(true);
					// try to delete some files left behind

					if(!uflsmc.Get_MatrixFile(s_file, selected_matrices[i])) {
						fprintf(stderr, "error: failed to decompress matrix " PRIsize " (%s/%s), won't try again\n",
							selected_matrices[i], r_t_mat.s_group.c_str(),
							r_t_mat.s_name.c_str());

						++ n_decompress_fail_num;
						continue;
					}
					// try to load again
				}
				CUFLSMC::CUnrefGuard unref(uflsmc, selected_matrices[i]);
				// a simple guard object to automatically unreference the matrix file

				if(!b_generate_block_layour_only) {
					if(!CPath::Copy_File((s_dest_directory + s_dest).c_str(), s_file.c_str(), true)) { // overwrite if exists
						fprintf(stderr, "warning: failed to copy matrix " PRIsize " (%s/%s)\n",
							selected_matrices[i], r_t_mat.s_group.c_str(),
							r_t_mat.s_name.c_str());
						continue;
					}
				}
				if(p_list_file) // may have failed to open
					fprintf(p_list_file, "%s\n", s_dest.c_str());
				uint64_t n_size = TFileInfo(s_file).n_Size64();
				n_total_size = min(n_total_size, UINT64_MAX - n_size) + n_size;

				if(!b_generate_block_layour_only)
					continue;

				//CTimer t;

				uint64_t n_matrix_size_csc, n_matrix_size_coo;
				if(1) { // this is really only verbose; don't spend any time on that
					TSparse::TMMHeader t_header;
					t_header.b_binary = r_t_mat.b_is_binary;
					t_header.b_symmetric = r_t_mat.b_is_struct_symmetric;
					t_header.n_nnz = r_t_mat.n_nnz;
					t_header.n_full_nnz = (t_header.b_symmetric)? r_t_mat.n_nnz * 2 - min(r_t_mat.n_nnz, // guess how many nnz there would be, assuming the diagonal is full
						min(t_header.n_rows, t_header.n_cols)) : r_t_mat.n_nnz;
					t_header.n_rows = r_t_mat.n_rows;
					t_header.n_cols = r_t_mat.n_cols;
					n_matrix_size_csc = t_header.n_AllocationSize_CSC();
					n_matrix_size_coo = t_header.n_AllocationSize_COO();
				} else {
					TSparse::TMMHeader t_header;
					if(!TSparse::Peek_MatrixMarket_Header(s_file.c_str(), t_header)) {
						fprintf(stderr, "warning: failed to read the header of \'%s\'\n", s_file.c_str());
						continue;
					}
					n_matrix_size_csc = t_header.n_AllocationSize_CSC();
					n_matrix_size_coo = t_header.n_AllocationSize_COO();
				}
				// don't have to actually read the file to do that

				if(n_matrix_size_csc > 2U * 1024 * 1024 * 1024 && n_matrix_size_csc != uint64_t(-1)) {
					printf("(" PRIsizeB "B in CSC / " PRIsizeB "B COO)\rprocessing matrix " PRIsize ": %s / %s ",
						PRIsizeBparams(n_matrix_size_csc),
						PRIsizeBparams(n_matrix_size_coo), i + 1, r_t_mat.s_group.c_str(), r_t_mat.s_name.c_str());
					fflush(stdout); // this stays there for some time
				}
				// verbose - print matrix size if large

				TSparse mat;
				try {
					if(!mat.Load_MatrixMarket(s_file.c_str())) {
						++ n_read_fail_num;
						fprintf(stderr, "warning: failed to load \'%s\'\n", s_file.c_str());
						continue;
					}
				} catch(std::bad_alloc&) {
					++ n_read_fail_num;
					fprintf(stderr, "warning: got std::bad_alloc while loading \'%s\'\n", s_file.c_str());
					continue;
				} catch(std::exception &r_exc) {
					++ n_read_fail_num;
					fprintf(stderr, "warning: got std::exception while loading "
						"\'%s\' (%s)\n", s_file.c_str(), r_exc.what());
					continue;
				}
				// load the matrix

				unref.Trigger();
				// loaded the matrix, don't need the file anymore

				/*printf("it took %f to load\n", t.f_Time());
				return 0;*/ // profiling Load_MatrixMarket() somewhat

				try {
					std::vector<size_t> block_rows, block_cols;
					if(n_block_tol)
						Find_BlockLayout_SPARSKIT_Tol(block_rows, block_cols, mat, n_block_tol); // throws
					else
						Find_BlockLayout_SPARSKIT(block_rows, block_cols, mat); // throws
					// find block structure

					TBlockInfo bsi(mat, block_rows, block_cols); // throws
					// gether block information

					std::map<size_t, size_t> block_size_hist;
					bsi.Get_BlockSize_Histogram(block_size_hist); // throws
					// make a histogram out of that

					std::map<size_t, size_t>::const_iterator p_med_it = block_size_hist.begin(),
						p_end_it = block_size_hist.end();
					std::advance(p_med_it, block_size_hist.size() / 2);
					-- p_end_it;
					size_t n_median_bs = (*p_med_it).first;
					size_t n_median_bs_c = (*p_med_it).second;
					size_t n_min_bs = (*block_size_hist.begin()).first;
					size_t n_min_bs_c = (*block_size_hist.begin()).second;
					size_t n_max_bs = (*p_end_it).first;
					size_t n_max_bs_c = (*p_end_it).second;
					printf(": min blk size %d (" PRIvalueMPns "), med %d (" PRIvalueMPns
						"), max %d (" PRIvalueMPns "), blk-sym: %s, %.2f%% blk\n",
						int(n_min_bs), PRIvalueMPparamsExt(n_min_bs_c, 0, 2, false),
						int(n_median_bs), PRIvalueMPparamsExt(n_median_bs_c, 0, 2, false),
						int(n_max_bs), PRIvalueMPparamsExt(n_max_bs_c, 0, 2, false),
						(block_rows == block_cols)? "yes" : "no",
						double(bsi.n_NNZ_BlockSize(1, 3)) / bsi.n_block_elem_nnz/*mat.p[mat.n]*/ * 100); // ratio of the nnz in blocks of area at least 3 nnz
					// display

					if(n_max_bs > 1) { // is there any block structure at all?
						if(!bsi.Write_BlockLayout((s_dest_directory + s_dest_bla).c_str())) {
							fprintf(stderr, "warning: failed to save \'%s\'\n",
								(s_dest_directory + s_dest_bla).c_str());
						}
					}
					// write a .bla file as well

					TBmp *p_bmp;
					const int n_max_bitmap_size = 640;
					if((p_bmp = mat.p_Rasterize_AA(n_max_bitmap_size))) {
						if(!CPngCodec::Save_PNG((s_dest_directory + s_dest_png).c_str(), *p_bmp, false)) {
							fprintf(stderr, "warning: failed to save \'%s\'\n",
								(s_dest_directory + s_dest_png).c_str());
							CPath::Remove_File((s_dest_directory + s_dest_png).c_str());
						}

						if(n_max_bs > 1) { // is there any block structure at all?
							p_bmp = mat.p_Draw_BlockLayout(p_bmp, block_rows, block_cols);

							if(!CPngCodec::Save_PNG((s_dest_directory + s_dest_png_bb).c_str(), *p_bmp, false)) {
								fprintf(stderr, "warning: failed to save \'%s\'\n",
									(s_dest_directory + s_dest_png_bb).c_str());
								CPath::Remove_File((s_dest_directory + s_dest_png_bb).c_str());
							}
						}
						p_bmp->Delete();
					} else {
						fprintf(stderr, "warning: failed to rasterize \'%s\'\n",
							(s_dest_directory + s_dest_png).c_str());
					}
				} catch(std::bad_alloc&) {
					++ n_anal_fail_num;
					fprintf(stderr, "warning: got std::bad_alloc while analysing \'%s\'\n", s_file.c_str());
					continue;
				} catch(std::exception &r_exc) {
					++ n_anal_fail_num;
					fprintf(stderr, "warning: got std::exception while analysing "
						"\'%s\' (%s)\n", s_file.c_str(), r_exc.what());
					continue;
				}
			}
			printf("\ndone. extracted " PRIsizeB "B of data\n", PRIsizeBparams(n_total_size));
			if(n_decompress_fail_num)
				fprintf(stderr, "warning: there were " PRIsize " fails in decompressing the matrix\n", n_decompress_fail_num);
			if(n_read_fail_num)
				fprintf(stderr, "warning: there were " PRIsize " fails in reading the matrix\n", n_read_fail_num);
			if(n_anal_fail_num)
				fprintf(stderr, "warning: there were " PRIsize " fails in analysing the matrix\n", n_anal_fail_num);
			if(!n_decompress_fail_num && !n_read_fail_num && !n_anal_fail_num)
				printf("no issues occured\n");

			if(p_list_file && ferror(p_list_file)) { // may have failed to open
				fprintf(stderr, "warning: i/o error while writing \'%s\': list of matrices won't be available\n",
					(s_dest_directory + "list.txt").c_str());
				p_list_file.Close();
				CPath::Remove_File((s_dest_directory + "list.txt").c_str());
			}
			// finalize the list of files
		}

		return 0;
	} else {
		printf("entering downloader mode ...\n");

		size_t n_fail_num = 0, n_file_num = 0;
		try {
			std::string s_uflsmc = s_dest_directory;
			if(!CPath::b_EndsWithSlash(s_uflsmc))
				s_uflsmc += CPath::path_Separator;
			// get path to where to download the files

			CTitleUpdaterThread ctu;
			ctu.Start();

			CThreadedHTTPDownloader p_downloader[max_Threads];

			std::set<std::string> folder_fail_list;
			std::vector<std::string> dir_download_list;
			std::vector<std::pair<std::string, std::string> > file_download_list;

			//dir_download_list.push_back("http://www.cise.ufl.edu/research/sparse/mat/"); // compressed matlab, can't parse
			//dir_download_list.push_back("http://www.cise.ufl.edu/research/sparse/MM/"); // forbidden now :-/
			//dir_download_list.push_back("http://www.cise.ufl.edu/research/sparse/RB/"); // rutherford-boeing, can't parse
			file_download_list.push_back(std::make_pair(std::string("http://www.cise.ufl."
				"edu/research/sparse/matrices/UFstats.csv"), std::string("UFstats.csv")));
			CPath::Remove_File("sparse/matrices/UFstats.csv"); // redownload this one once in a while ...
			// start downloading the first page and the UFstats.csv file

			/*

			UFstats.csv

				A CSV file is also available with some of this index information (UFstats.csv).
			The first line of the CSV file gives the number of matrices in the collection, and
			the second line gives the LastRevisionDate. Line k+2 in the file lists the following
			statistics for the matrix whose id number is k: Group, Name, nrows, ncols, nnz,
			isReal, isBinary, isND, posdef, pattern_symmetry, numerical_symmetry, and kind.

			*/

			bool b_ufstats_pending = true;
			for(;;) {
				if(dir_download_list.empty() && file_download_list.empty()) {
					for(int i = 0; i < max_Threads; ++ i) {
						if(p_downloader[i].b_IsAssigned() && p_downloader[i].n_Tag() == tag_Folder) {
							if(!On_FolderDownload(dir_download_list, file_download_list,
							   p_downloader[i], folder_fail_list))
								return -1;
						}
					}
					// go through all the downloaders and see if any directory has been downloaded

					if(dir_download_list.empty() && file_download_list.empty() && !b_ufstats_pending)
						break;
					// nothing is downloading anymore ...
				}
				// in case there is nothing to download ...

				CThreadedHTTPDownloader *p_down = CThreadedHTTPDownloader::p_Select(
					p_downloader, p_downloader + max_Threads);
				// select a (possibly) free downloader

				if(p_down->b_IsAssigned() && strcmp(CPath::p_s_Get_Filename(p_down->p_s_FileName()), "UFstats.csv")) { // UFstats.csv handled below
					if(p_down->n_Tag() == tag_Folder) {
						if(!On_FolderDownload(dir_download_list, file_download_list, *p_down, folder_fail_list))
							return -1;
					} else {
						int n_result = p_down->n_GetResult(true);
						if(n_result != CHTTPDownloader::down_Success) {
							fprintf(stderr, "error: failed to download \'%s\'\n", p_down->p_s_URL());
							CPath::Remove_File(p_down->p_s_FileName()); // don't leave rubbish ...
							//return -1; // don't fail
							++ n_fail_num;
						} else {
							printf("downloaded \'%s\' ...\n", p_down->p_s_URL());
							++ n_file_num;
						}
					}
				}

				if(!dir_download_list.empty() && !p_down->b_IsAssigned()) {
					std::string s_file;
					if(!TFileInfo::Get_TempFileName_Dir(s_file, p_s_explicit_temp, "gmm")) {
						fprintf(stderr, "error: not enough memory\n");
						return -1;
					}
					// get temp filename

					if(!p_down->Start_Download(s_file.c_str(), dir_download_list.back().c_str(), tag_Folder)) {
						fprintf(stderr, "error: failed to start downloading \'%s\'\n", dir_download_list.back().c_str());
						return -1;
					} else
						printf("downloading \'%s\' ...\n", dir_download_list.back().c_str());
					dir_download_list.erase(dir_download_list.end() - 1);
					// start downloading it, erase from the list
				} else if(!file_download_list.empty() && !p_down->b_IsAssigned()) {
					_ASSERTE(!file_download_list.empty());

					const std::string &s_filename = file_download_list.back().second;
					const std::string &s_url = file_download_list.back().first;

					std::string s_path = s_url;
					size_t n_pos;
					if(s_filename == "README.txt" && (n_pos = s_path.find("/mat/")) != std::string::npos) {
						s_path.erase(n_pos + 1, 3);
						s_path.insert(n_pos + 1, "MM"); // put readme files in MM rather than in mat
					}
					if((n_pos = s_path.find("/sparse/")) != std::string::npos)
						s_path.erase(0, n_pos + 1);
					s_path.erase(s_path.rfind('/'));
					//CreateDirectory(s_path.c_str(), 0);
					s_path = s_uflsmc + s_path;
					CPath::WeakNormalize(s_path);
					if(!TFileInfo(s_path.c_str()).b_exists)
						CPath::Make_Directories(s_path.c_str());
					_ASSERTE(!CPath::b_EndsWithSlash(s_path)); // deleted it above
					s_path += CPath::path_Separator;
					s_path += s_filename;

					if(TFileInfo(s_path.c_str()).b_exists)
						printf("skipping \'%s\' (already exists) ...\n", s_url.c_str());
					else if(!p_down->Start_Download(s_path.c_str(), s_url)) {
						fprintf(stderr, "error: failed to start downloading \'%s\'\n", s_url.c_str());
						return -1;
					} /*else
						printf("downloading \'%s\' ...\n", s_url.c_str());*/
					file_download_list.erase(file_download_list.end() - 1);
					// start downloading it, erase from the list
				} else {
					_ASSERTE(b_ufstats_pending);

					for(int i = 0; i < max_Threads; ++ i) {
						if(!p_downloader[i].b_IsAssigned())
							continue;
						_ASSERTE(p_downloader[i].n_Tag() != tag_Folder);
						int n_result = p_downloader[i].n_GetResult(true);
						if(n_result != CHTTPDownloader::down_Success) {
							fprintf(stderr, "error: failed to download \'%s\'\n", p_downloader[i].p_s_URL());
							CPath::Remove_File(p_downloader[i].p_s_FileName()); // don't leave rubbish ...
							//return -1; // don't fail
							++ n_fail_num;
						} else {
							printf("downloaded \'%s\' ...\n", p_downloader[i].p_s_URL());
							++ n_file_num;
						}
						
						p_down = &p_downloader[i]; // !!

						if(!strcmp(CPath::p_s_Get_Filename(p_down->p_s_FileName()), "UFstats.csv")) {
							b_ufstats_pending = false;
							if(n_result == CHTTPDownloader::down_Success) {
								CUniqueFILE_Ptr p_fr;
								if(p_fr.Open(p_down->p_s_FileName(), "r")) {
									std::string s_line;
									while(!feof(p_fr)) {
										if(!stl_ut::ReadLine(s_line, p_fr))
											break;
										stl_ut::TrimSpace(s_line);
										if(s_line.empty())
											continue;

										std::vector<std::string> tokens;
										stl_ut::Split(tokens, s_line, ",", -1); // ignore errors here
										if(tokens.size() > 5) { // 5 is just a fudge factor, there are many
											const std::string &r_s_matgroup = tokens[0], r_s_matname = tokens[1];

											{
												std::string s_url = "http://www.cise.ufl.edu/research/sparse/MM/" +
													r_s_matgroup + "/" + r_s_matname + ".tar.gz";
												std::string s_filename = r_s_matname + ".tar.gz";

												file_download_list.push_back(std::make_pair(s_url, s_filename));
											}
											// download matrices

											{
												std::string s_url = "http://www.cise.ufl.edu/research/sparse/mat/" +
													r_s_matgroup + "/README.txt";
												std::string s_filename = "README.txt";

												if(std::find_if(file_download_list.begin(), file_download_list.end(),
												   CCompareUrl(s_url)) == file_download_list.end())
													file_download_list.push_back(std::make_pair(s_url, s_filename));
											}
											// download matrix group readme files
										}
									}
								}
							}
						}

						break;
					}
					// go through all the downloaders and see if any files are pending
				}
			}
			// download tree structure ...

			for(int i = 0; i < max_Threads; ++ i) {
				if(!p_downloader[i].b_IsAssigned())
					continue;
				_ASSERTE(p_downloader[i].n_Tag() != tag_Folder);
				int n_result = p_downloader[i].n_GetResult(true);
				if(n_result != CHTTPDownloader::down_Success) {
					fprintf(stderr, "error: failed to download \'%s\'\n", p_downloader[i].p_s_URL());
					CPath::Remove_File(p_downloader[i].p_s_FileName()); // don't leave rubbish ...
					//return -1; // don't fail
					++ n_fail_num;
				} else {
					printf("downloaded \'%s\' ...\n", p_downloader[i].p_s_URL());
					++ n_file_num;
				}
			}
			// go through all the downloaders and see if any files are pending

			ctu.Stop();
		} catch(std::bad_alloc&) {
			fprintf(stderr, "error: not enough memory\n");
			return -1;
		}

		if(!n_fail_num)
			printf("\ndone. downloaded " PRIsize " files, no error(s) occured\n", n_file_num);
		else
			fprintf(stderr, "\ndone. downloaded " PRIsize " files, " PRIsize " error(s) occured\n", n_file_num, n_fail_num);
	}

	return 0;
}
