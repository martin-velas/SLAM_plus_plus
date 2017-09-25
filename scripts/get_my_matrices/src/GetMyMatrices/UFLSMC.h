#pragma once

#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/Dir.h"
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include "Sparse.h"
#include "Extract_TarGz.h"

#if !defined(_WIN32) && !defined(_WIN64)
#include <time.h>

void Sleep(int n_millisecond_num)
{
	struct timespec tv;
	tv.tv_sec = n_millisecond_num / 1000;
	tv.tv_nsec = (n_millisecond_num % 1000) * 1000000;
	nanosleep(&tv, NULL);
}
#endif // !_WIN32 && !_WIN64

/**
 *	@brief University of Florida Sparse Matrix Collection UFStats.csv parser and matrix loader
 */
class CUFLSMC {
public:
	struct TMatrixRecord {
		std::string s_group;
		std::string s_name;
		size_t n_rows;
		size_t n_cols;
		size_t n_nnz;
		bool b_is_real;
		bool b_is_binary;
		bool b_is_2D_3D;
		bool b_is_pos_def;
		float f_struct_symmetry;
		bool b_is_struct_symmetric;
		bool b_is_num_symmetric;
		std::string s_kind;
	};

	class CUnrefGuard {
	protected:
		CUFLSMC &m_r_uflsmc;
		size_t m_n_matrix_id;

	public:
		CUnrefGuard(CUFLSMC &r_uflsmc, size_t n_matrix_id)
			:m_r_uflsmc(r_uflsmc), m_n_matrix_id(n_matrix_id)
		{}

		~CUnrefGuard()
		{
			Trigger();
		}

		void Trigger()
		{
			if(m_n_matrix_id != size_t(-1)) {
				m_r_uflsmc.Unref_MatrixFile(m_n_matrix_id);
				m_n_matrix_id = size_t(-1);
			}
		}
	};

protected:
	struct TFileRef {
		std::string first;
		CFILE_PtrGuard second;

		TFileRef()
		{}

		TFileRef(const std::string &_first, CFILE_PtrGuard &_second)
			:first(_first), second(_second)
		{}

		TFileRef(const TFileRef &r_other)
			:first(r_other.first), second(const_cast<TFileRef&>(r_other).second)
		{}
	};

	class CRemoveRefFile {
	public:
		void operator ()(TFileRef &r_ref) const
		{
			r_ref.second.Destroy();
			CPath::Remove_File(r_ref.first.c_str());
		}

		inline void operator ()(std::map<size_t, TFileRef>::value_type &r_maptype) const
		{
			(*this)(r_maptype.second);
		}
	};

protected:
	std::string m_s_temp_path;
	std::string m_s_base_path;
	std::string m_s_date;
	std::vector<TMatrixRecord> m_matrix_list;

	mutable std::map<size_t, TFileRef> m_ref_list;

public:
	/**
	 *	@brief default constructor; parses the UFStats.csv file
	 *
	 *	The collection is expected to be in the same format as on the web, i.e.:
	 *
	 *	- sparse
	 *		- mat
	 *			- <groups>
	 *				- <names>.mat
	 *		- matrices
	 *			- UFStats.csv
	 *		- MM
	 *			- <groups>
	 *				- <names>.tar.gz
	 *		- RB
	 *			- <groups>
	 *				- <names>.tar.gz
	 *
	 *	@param[in] p_s_base_parth is null-terminated string, containing the base path
	 *		where the collection was downloaded (in which the "sparse" directory resides)
	 *	@note If this succeeds, b_Status() returns true.
	 */
	CUFLSMC(const char *p_s_base_parth, const char *p_s_explicit_temp = 0)
	{
		if((!p_s_explicit_temp && !CPath::Get_TempDirectory(m_s_temp_path)) ||
		   (p_s_explicit_temp && !stl_ut::AppendCStr(m_s_temp_path, p_s_explicit_temp)))
			return;
		if(!stl_ut::AssignCStr(m_s_base_path, p_s_base_parth))
			return;
		CPath::Drop_TrailingSlash(m_s_base_path);
		if(!Read_UFStats())
			m_matrix_list.clear(); // to mark error
	}

	~CUFLSMC()
	{
		std::for_each(m_ref_list.begin(), m_ref_list.end(), CRemoveRefFile());
		m_ref_list.clear();
	}

	inline bool b_Status() const
	{
		return !m_matrix_list.empty();
	}

	inline size_t n_Matrix_Num() const
	{
		return m_matrix_list.size();
	}

	inline const TMatrixRecord &t_Matrix_Info(size_t n_matrix) const
	{
		_ASSERTE(n_matrix < m_matrix_list.size());
		return m_matrix_list[n_matrix];
	}

	inline const std::vector<TMatrixRecord> &r_Matrix_List() const
	{
		return m_matrix_list;
	}

	bool Remove_Nonreferenced_MatrixFiles(bool b_verbose = false) const
	{
		size_t n_removed_num = 0;
		uint64_t n_size_removed = 0;

		std::string s_temp;
		if(!stl_ut::Assign(s_temp, m_s_temp_path)/*!CPath::Get_TempDirectory(s_temp)*/)
			return false;
		if(!stl_ut::AppendCStr(s_temp, "/_uflsmc/sparse/MM") ||
		   !CPath::WeakNormalize(s_temp) || !CPath::Make_Directories(s_temp.c_str()))
			return false;
		// create a temp directory for the matrices

		CDirectory temp_dir(s_temp.c_str());
		if(!temp_dir.p_s_Name())
			return false;
		TFileInfo t_group_dir;
		if(!temp_dir.Get_FirstFile(t_group_dir))
			return false;

		while(t_group_dir.b_exists) {
			if(t_group_dir.b_directory) {
				CDirectory group_dir(t_group_dir.p_s_Path());
				if(!group_dir.p_s_Name())
					return false;
				TFileInfo t_mat_dir;
				if(!group_dir.Get_FirstFile(t_mat_dir))
					return false;

				bool b_removed_matrices = false;

				while(t_mat_dir.b_exists) {
					if(t_mat_dir.b_directory) {
						CDirectory mat_dir(t_mat_dir.p_s_Path());
						if(!mat_dir.p_s_Name())
							return false;
						TFileInfo t_file;
						if(!mat_dir.Get_FirstFile(t_file))
							return false;

						bool b_failed_to_remove_refs = false;
						while(t_file.b_exists) {
							if(!t_file.b_directory && ((!strncmp(t_file.p_s_FileName(), ".ref-", 5) &&
							   isdigit(uint8_t(t_file.p_s_FileName()[5]))) ||
							   !strcmp(t_file.p_s_FileName(), ".lock"))) { // lock files too
								CUniqueFILE_Ptr lf;
								if(!lf.Open(t_file.p_s_Path(), "w")) { // in linux, an opened file can be deleted, so try writing it first instead
									b_failed_to_remove_refs = true;
									break;
								}
								lf.Close();
								if(!CPath::Remove_File(t_file.p_s_Path())) {
									b_failed_to_remove_refs = true;
									break;
								}
							}

							if(!mat_dir.Get_NextFile(t_file))
								return false;
						}
						// go through all the files in the matrix directory and try to remove them

						if(!b_failed_to_remove_refs) {
							std::string s_filename;
							if((stl_ut::Format(s_filename, "%s%c%s.mtx", t_mat_dir.p_s_Path(), CPath::path_Separator,
							   t_mat_dir.p_s_FileName()) && CPath::b_Exists(s_filename)) ||
							   (stl_ut::Format(s_filename, "%s%c%s%c%s.mtx", t_mat_dir.p_s_Path(), CPath::path_Separator,
							   t_mat_dir.p_s_FileName(), CPath::path_Separator, t_mat_dir.p_s_FileName()) &&
							   CPath::b_Exists(s_filename))) {
								uint64_t n_size = TFileInfo(s_filename).n_Size64();
								n_size_removed = min(n_size_removed, UINT64_MAX - n_size) + n_size;
								if(n_removed_num != SIZE_MAX)
									++ n_removed_num;
								// keep stats

								if(CPath::Remove_File(s_filename.c_str())) {
									b_removed_matrices = true;
									if(b_verbose) {
										printf("debug: deleted unreferenced matrix %s / %s\n",
											t_group_dir.p_s_FileName(), t_mat_dir.p_s_FileName());
									}

									if(CPath::Get_Path(s_filename)) // might be in a subdir
										CPath::Remove_Directory(s_filename.c_str());
									CPath::Remove_Directory(t_mat_dir.p_s_Path());
									// remove the dir the matrix was in as well, ignore errors here (such as user files
									// or if someone just extracted it again)
								} else if(b_verbose) {
									fprintf(stderr, "warning: failed to delete unreferenced matrix %s/%s\n",
										t_group_dir.p_s_FileName(), t_mat_dir.p_s_FileName());
								}
							}
							// remove the .mtx file
						}
					}

					if(!group_dir.Get_NextFile(t_mat_dir))
						return false;
				}
				// for all matrix directories

				if(b_removed_matrices) {
					CPath::Remove_Directory(t_group_dir.p_s_Path());
					// remove the dir the matrix was in as well, ignore errors here (such as user files
					// or if someone just extracted it again)
				}
			}

			if(!temp_dir.Get_NextFile(t_group_dir))
				return false;
		}
		// for all matrix group directories

		if(b_verbose) {
			printf("done. removed " PRIsizeB "B in " PRIsize " files\n",
				PRIsizeBparams(n_size_removed), n_removed_num);
		}
		// verbose

		return true;
	}

	void Unref_MatrixFile(size_t n_matrix) const
	{
		std::map<size_t, TFileRef>::iterator p_ref_it;
		if((p_ref_it = m_ref_list.find(n_matrix)) != m_ref_list.end()) {
			CRemoveRefFile()((*p_ref_it).second);
			m_ref_list.erase(p_ref_it);
		}
		// if we indeed had a reference to this matrix, remove it (might not have
		// one if this was an uncompressed .mtx directly in the collection source)
	}

	bool Get_MatrixFile(std::string &r_s_file, size_t n_matrix) const
	{
		try {
			_ASSERTE(n_matrix < m_matrix_list.size());
			const TMatrixRecord &r_matrix = m_matrix_list[n_matrix];
			r_s_file = m_s_base_path + "/sparse/MM/" + r_matrix.s_group + "/" + r_matrix.s_name + ".mtx";
			if(CPath::WeakNormalize(r_s_file) && TFileInfo(r_s_file.c_str()).b_exists)
				return true;
			// see if an uncompressed mtx exists (an extension to avoid unpacking large files all the time)
			// note that this is inside the collection source (NOT in temp unpacked files,
			// will not be deleted, no need to count references)

			std::string s_archive = m_s_base_path + "/sparse/MM/" + r_matrix.s_group + "/" + r_matrix.s_name + ".tar.gz";
			if(!CPath::WeakNormalize(s_archive) || !TFileInfo(s_archive.c_str()).b_exists)
				return false; // .tar.gz must exist
			// find a .tar.gz

			std::string s_temp;
			if(!stl_ut::Assign(s_temp, m_s_temp_path)/*!CPath::Get_TempDirectory(s_temp)*/)
				return false;
			s_temp += "/_uflsmc/sparse/MM/" + r_matrix.s_group + "/" + r_matrix.s_name;
			if(!CPath::WeakNormalize(s_temp) || !CPath::Make_Directories(s_temp.c_str()))
				return false;
			// create a temp directory for the matrices

			std::string s_lock = s_temp;
			if(!stl_ut::Format(s_lock, "%s%c.lock", s_temp.c_str(), CPath::path_Separator))
				return false;
			FILE *p_fw;
			bool b_had_lock = CPath::b_Exists(s_lock);
			while(!(p_fw = CPath::p_OpenFile(s_lock.c_str(), "w")))
				Sleep(100); // wait for the other thread to finish
			class CClose_Delete {
			protected:
				FILE *m_p_fw;
				std::string m_s_filename;

			public:
				CClose_Delete(FILE *_p_fw, const std::string &r_s_filename)
					:m_p_fw(_p_fw), m_s_filename(r_s_filename)
				{}

				~CClose_Delete()
				{
					if(m_p_fw)
						fclose(m_p_fw);
					CPath::Remove_File(m_s_filename.c_str());
				}
			} guard(p_fw, s_lock); // close the file upon exit
			// open a file for writing

			if(!m_ref_list.count(n_matrix)) {
				std::string s_ref;
				for(int n_refid = 0;; ++ n_refid) {
					if(!stl_ut::Format(s_ref, "%s%c.ref-%d", s_temp.c_str(), CPath::path_Separator, n_refid))
						return false;
					if(CPath::b_Exists(s_ref) && !CPath::Remove_File(s_ref.c_str()))
						continue;
					CUniqueFILE_Ptr p_fw;
					if(!p_fw.Open(s_ref.c_str(), "w"))
						continue; // try with a higher id

					TFileRef &r_ref = m_ref_list[n_matrix];
					r_ref.first.swap(s_ref);
					r_ref.second = p_fw;
					break;
				}
				// keep a list of references to each matrix that we use
			}

			if(!b_had_lock) {
				r_s_file = s_temp + CPath::p_s_Separator() + r_matrix.s_name + ".mtx";
				if(/*CPath::WeakNormalize(r_s_file) &&*/ CPath::b_Exists(r_s_file))
					return true;
				r_s_file = s_temp + CPath::p_s_Separator() + r_matrix.s_name +
					CPath::p_s_Separator() + r_matrix.s_name + ".mtx";
				if(/*CPath::WeakNormalize(r_s_file) &&*/ CPath::b_Exists(r_s_file))
					return true;
			}
			// in case no other thread was there before, don't reuse the file, might be incomplete

			if(!CUnTarGz::Extract(s_archive.c_str(), s_temp.c_str(), m_s_temp_path.c_str())) {
				/*r_s_file = s_temp + CPath::p_s_Separator() + r_matrix.s_name + ".mtx";
				if(CPath::WeakNormalize(r_s_file) && CPath::b_Exists(r_s_file))
					CPath::Remove_File(r_s_file.c_str());
				r_s_file = s_temp + CPath::p_s_Separator() + r_matrix.s_name +
					CPath::p_s_Separator() + r_matrix.s_name + ".mtx";
				if(CPath::WeakNormalize(r_s_file) && CPath::b_Exists(r_s_file))
					CPath::Remove_File(r_s_file.c_str());
				// make sure not to leave rubbish behind!*/

				Remove_RubbishFiles(s_temp, 0); // remove all files
				// remove the rest of the files too

				return false;
			}
			// extract the file to the temp folder, overwriting the other files

			r_s_file = s_temp + CPath::p_s_Separator() + r_matrix.s_name + ".mtx";
			if(/*CPath::WeakNormalize(r_s_file) &&*/ CPath::b_Exists(r_s_file)) {
				Remove_RubbishFiles(s_temp, r_s_file.c_str());
				return true;
			}
			r_s_file = s_temp + CPath::p_s_Separator() + r_matrix.s_name +
				CPath::p_s_Separator() + r_matrix.s_name + ".mtx";
			if(/*CPath::WeakNormalize(r_s_file) &&*/ CPath::b_Exists(r_s_file)) {
				Remove_RubbishFiles(s_temp, r_s_file.c_str());
				return true;
			}
			// see which file was there

			Remove_RubbishFiles(s_temp, 0);
			// remove all files to save space

			// if this happens a lot, could also list the files and pick the shortest filename

			return false; // not found
		} catch(std::bad_alloc&) {
			return false;
		}
	}

#ifdef __UBER_BLOCK_MATRIX_INCLUDED

	bool LoadMatrix(CUberBlockMatrix &r_dest, size_t n_matrix) const
	{
		std::string s_file;
		if(!Get_MatrixFile(s_file, n_matrix))
			return false;
		CUnrefGuard g(*this, n_matrix);
		return r_dest.Load_MatrixMarket(s_out_file.c_str(), 1, true);

#if 0 // old code
		try {
			_ASSERTE(n_matrix < m_matrix_list.size());
			const TMatrixRecord &r_matrix = m_matrix_list[n_matrix];
			std::string s_src = m_s_base_path + "/sparse/MM/" + r_matrix.s_group + CPath::p_s_Separator() + r_matrix.s_name + ".tar.gz";
			CPath::Normalize(s_src);
			// get file name

			std::string s_temp;
			stl_ut::Assign(s_temp, m_s_temp_path)/*CPath::Get_TempDirectory(s_temp)*/;
			s_temp += CPath::path_Separator;
#ifdef __LAME_THREADS_INCLUDED
			char p_s_tid[128];
			stl_ut::Format(p_s_tid, sizeof(p_s_tid), "uflsmc%d", CCurrentThreadHandle::n_Get_Id());
			s_temp += p_s_tid;
#else // __LAME_THREADS_INCLUDED
			s_temp += "uflsmc"; // note that this is not thread safe, the threads would delete each other's work
#endif // __LAME_THREADS_INCLUDED
			if(!CPath::b_Exists(s_temp) && system(("mkdir \"" + s_temp + "\"").c_str()))
				return false;
			s_temp += CPath::path_Separator;
			// create temp directory in temp

#if defined(_WIN32) || defined(_WIN64)
			{
				std::string s_tar_file = s_temp + r_matrix.s_name + ".tar";
				// get .tar filename (7zip extracts .tar.gz first to .tar, then individual files)

				system(("del /s /f /q \"" + s_temp + "*\" > nul").c_str()); // clean the temp directory
				system(("D:\\Progra~1\\7-Zip\\7z.exe e \"-o" + s_temp +
					"\" \"" + s_src + "\" > nul").c_str()); // extract the archive to .tar
				system(("D:\\Progra~1\\7-Zip\\7z.exe e \"-o" + s_temp +
					"\" \"" + s_tar_file +"\" > nul").c_str()); // extract the archive from .tar to separate files
				DeleteFile(s_tar_file.c_str()); // deletess .tar (not .tar.gz)
				// extract the current matrix archive
			}
#else // _WIN32 || _WIN64
#error "unpacking using the tar command on linux not implemented (todo)"
#endif // _WIN32 || _WIN64
			// extract the archive using an external archiver

			std::string s_out_file = s_temp + r_matrix.s_name + ".mtx";
			// get the expected output file name

			if(!TFileInfo(s_out_file.c_str()).b_exists) {
				fprintf(stderr, "error: archive \'%s\' doesn't contain file \'%s\'\n",
					s_src.c_str(), s_out_file.c_str());
				return false;
			}
			// make sure the extracted file is there

			bool b_result = r_dest.Load_MatrixMarket(s_out_file.c_str(), 1, true);

#if defined(_WIN32) || defined(_WIN64)
			system(("del /s /f /q \"" + s_temp + "*\" > nul").c_str());
#else // _WIN32 || _WIN64
			system(("rm -f \"" + s_temp + "*\"").c_str());
#endif // _WIN32 || _WIN64
			// clean the temp directory (there might be more files than just the required one in the archive)

			return b_result;
		} catch(std::bad_alloc&) {
			return false;
		}
#endif // 0
	}

#endif // __UBER_BLOCK_MATRIX_INCLUDED

protected:
	bool Read_UFStats() // throw(std::bad_alloc)
	{
		m_matrix_list.clear();

		CUniqueFILE_Ptr p_fr;
		if(!p_fr.Open((m_s_base_path + "/sparse/matrices/UFstats.csv").c_str(), "r")) // throws
			return false;

		size_t n_matrix_num = size_t(-1);
		std::string s_line;
		std::vector<std::string> fields;
		for(size_t n_line = 0; !feof(p_fr); ++ n_line) {
			if(!stl_ut::ReadLine(s_line, p_fr))
				return false;
			stl_ut::TrimSpace(s_line);
			_ASSERTE(!s_line.empty() || feof(p_fr));
			if(s_line.empty()) {
				_ASSERTE(n_line >= 2);
				return m_matrix_list.size() == n_matrix_num;
			}
			if(!n_line)
				m_matrix_list.reserve(n_matrix_num = atol(s_line.c_str()));
			else if(n_line == 1)
				m_s_date = s_line;
			else {
				if(!stl_ut::Split(fields, s_line, ","))
					return false;
				if(fields.size() != 12) {
					fprintf(stderr, "warning: truncated matrix entry on line " PRIsize "\n", n_line + 1);
					continue;
				}
				TMatrixRecord t_mr;
				t_mr.s_group = fields[0];
				t_mr.s_name = fields[1];
				t_mr.n_rows = atol(fields[2].c_str());
				t_mr.n_cols = atol(fields[3].c_str());
				t_mr.n_nnz = atol(fields[4].c_str());
				t_mr.b_is_real = atoi(fields[5].c_str()) != 0;
				t_mr.b_is_binary = atoi(fields[6].c_str()) != 0;
				t_mr.b_is_2D_3D = atoi(fields[7].c_str()) != 0;
				t_mr.b_is_pos_def = atoi(fields[8].c_str()) != 0;
				t_mr.f_struct_symmetry = float(atof(fields[9].c_str()));
				t_mr.b_is_struct_symmetric = t_mr.f_struct_symmetry == 1;
				t_mr.b_is_num_symmetric = atoi(fields[10].c_str()) != 0;
				t_mr.s_kind = fields[11];
				m_matrix_list.push_back(t_mr);
				// A CSV file is also available with some of this index information (UFstats.csv).
				// The first line of the CSV file gives the number of matrices in the collection, and
				// the second line gives the LastRevisionDate. Line k+2 in the file lists the following
				// statistics for the matrix whose id number is k: Group, Name, nrows, ncols, nnz,
				// isReal, isBinary, isND, posdef, pattern_symmetry, numerical_symmetry, and kind.
			}
		}

		return m_matrix_list.size() == n_matrix_num;
	}

	class CRemoveFilesExcept {
	protected:
		const char *m_p_s_except_file;

	public:
		CRemoveFilesExcept(const char *p_s_except_file)
			:m_p_s_except_file(p_s_except_file)
		{}

		bool operator ()(const TFileInfo &r_t_file) const
		{
			if(r_t_file.b_directory)
				return true;

			const char *p_s_filename = r_t_file.p_s_FileName();
			// get only the file name

			if((!m_p_s_except_file || CPath::n_CompareString(r_t_file.p_s_Path(), m_p_s_except_file)) &&
			   strcmp(p_s_filename, ".lock") &&
			   strncmp(p_s_filename, ".ref-", 5))
				return CPath::Remove_File(r_t_file.p_s_Path());
			// see if this is the file we wanted to keep, the lock file or a ref file
			// if not, delete it

			return true;
		}
	};

	/**
	 *	@brief remove files that are no longer needed, leftover fiels after extraction
	 *		(readme, different versions of the matrix, ...)
	 *
	 *	@param[in] r_s_path is root path where to look for rubbish files
	 *	@param[in] p_s_except_file is null-terminated string containing full path
	 *		name to the file to skip, or null to skip no files
	 *
	 *	@return Returns true on success, false on failure (indicating that some
	 *		files could not be deleted and were left behind).
	 */
	static bool Remove_RubbishFiles(const std::string &r_s_path, const char *p_s_except_file)
	{
		return CDirTraversal::Traverse2(r_s_path.c_str(), CRemoveFilesExcept(p_s_except_file));
	}
};
