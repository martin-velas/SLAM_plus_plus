/*
								+---------------------------------+
								|                                 |
								|   ***   Directory lookup  ***   |
								|                                 |
								|  Copyright  © -tHE SWINe- 2007  |
								|                                 |
								|             Dir.cpp             |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file Dir.cpp
 *	@author -tHE SWINe-
 *	@date 2006
 *	@brief basic directory lookup and file info functions
 *
 *	@date 2007-07-03
 *
 *	passed code revision
 *
 *	added filename allocation, deallocation and copy functions to TFileInfo
 *
 *	added constructors to TFileInfo::TTime
 *
 *	added TFileInfo::b_valid so it's clear wheter call to CDirectory::Get_FirstFile
 *	or CDirectory::Get_NextFile functions failed or wheter there are no more files in
 *	the directory (as was identified by the same return value before)
 *
 *	merged source code of linux CDirectory and win32 CDirectory together as it shares
 *	substantial sections
 *
 *	created Dir.cpp
 *
 *	added CDirTraversal class
 *
 *	@date 2007-07-09
 *
 *	added TFileInfo copy-constructor TFileInfo(const TFileInfo &r_t_other)
 *	changed CDirTraversal's stack to queue so directory traversal is not reversed
 *
 *	@date 2008-02-21
 *
 *	fixed Win32 implementation of CDir::Get_FirstFile where it didn't detect
 *	failure properly and sometimes it could return invalid file data
 *
 *	@date 2008-02-27
 *
 *	fixend inconsistency between linux and windows implementation (in linux, TFileInfo used
 *	to contain bare filenames only, in windows it contained full paths. full paths are now
 *	used in both implementations). note this error made CDirTraversal fail under linux.
 *
 *	@date 2008-03-04
 *
 *	now using Integer.h header
 *
 *	@date 2008-08-08
 *
 *	added \#ifdef for windows 64
 *
 *	@date 2008-10-25
 *
 *	added CDirectory::path_Separator
 *
 *	@date 2008-11-21
 *
 *	added CDirTraversal::Traverse2, enabling file listener to interrupt directory traversal
 *
 *	added parameter b_recurse_subdirectories to both original CDirTraversal::Traverse() and
 *	new CDirTraversal::Traverse2() functions.
 *
 *	cleared-up CDirTraversal class documentation comment, regarding recursive traversal
 *	without recursion (ie. using explicit stack, placed on heap)
 *
 *	@date 2008-12-11
 *
 *	added TFileInfo::n_Size64() for greater conveniency
 *
 *	@date 2008-12-22
 *
 *	removed some g++ warnings
 *
 *	@date 2009-05-23
 *
 *	removed all instances of std::vector::reserve and replaced them by stl_ut::Reserve_*
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 */

#include "NewFix.h"

#include "CallStack.h"
#include <vector>
#include <algorithm>
#include <utility>
#include "Dir.h"
#include "StlUtils.h"

/*
 *								=== TFileInfo::TTime ===
 */

#if defined(_WIN32) || defined(_WIN64)
/*
 *	TFileInfo::TTime::TTime(const SYSTEMTIME &r_t_time)
 *		- (win32) conversion constructor, taking struct SYSTEMTIME as input
 *		- always succeeds
 */
TFileInfo::TTime::TTime(const SYSTEMTIME &r_t_time)
	:n_month(r_t_time.wMonth), n_day(r_t_time.wDay), n_year(r_t_time.wYear),
	n_hour(r_t_time.wHour), n_minute(r_t_time.wMinute), n_second(r_t_time.wSecond)
{}

/*
 *	TFileInfo::TTime::TTime(const FILETIME &r_t_time)
 *		- (win32) conversion constructor, taking struct FILETIME as input
 *		- note FILETIME needs to be converted to SYSTEMTIME first,
 *		  in case conversion fails, all members are set to -1
 */
TFileInfo::TTime::TTime(const FILETIME &r_t_time)
{
	SYSTEMTIME t_time;
	if(!FileTimeToSystemTime(&r_t_time, &t_time)) {
		n_month = n_day = n_year =
			n_hour = n_minute = n_second = -1;
		return;
	}
	n_month = t_time.wMonth;
	n_day = t_time.wDay;
	n_year = t_time.wYear;
	n_hour = t_time.wHour;
	n_minute = t_time.wMinute;
	n_second = t_time.wSecond;
}
#else // _WIN32 || _WIN64
/*
 *	TFileInfo::TTime::TTime(const tm *p_time)
 *		- (unix) conversion constructor, taking struct tm as input
 *		- always succeeds
 */
TFileInfo::TTime::TTime(const tm *p_time)
	:n_month(p_time->tm_mon), n_day(p_time->tm_mday), n_year(p_time->tm_year + 1900),
	n_hour(p_time->tm_hour), n_minute(p_time->tm_min), n_second(p_time->tm_sec)
{}
#endif // _WIN32 || _WIN64

/*
 *	bool TFileInfo::TTime::operator ==(const TTime &r_t_time) const
 *		- equality operator
 *		- returns true in case this is equal to r_t_time, otherwise false
 */
bool TFileInfo::TTime::operator ==(const TTime &r_t_time) const
{
	return n_month == r_t_time.n_month &&
		n_day == r_t_time.n_day &&
		n_year == r_t_time.n_year &&
		n_hour == r_t_time.n_hour &&
		n_minute == r_t_time.n_minute &&
		n_second == r_t_time.n_second;
}

bool TFileInfo::TTime::operator <(const TTime &r_t_time) const
{
	return 
		n_year < r_t_time.n_year || (n_year == r_t_time.n_year && (
			n_month < r_t_time.n_month || (n_month == r_t_time.n_month && (
				n_day < r_t_time.n_day || (n_day == r_t_time.n_day && (
					n_hour < r_t_time.n_hour || (n_hour == r_t_time.n_hour && (
						n_minute < r_t_time.n_minute || (n_minute == r_t_time.n_minute && (
							n_second < r_t_time.n_second
						))
					))
				))
			))
		));
}
/*
 *								=== ~TFileInfo::TTime ===
 */

/*
 *								=== TFileInfo ===
 */

/*TFileInfo::TFileInfo(const char *p_s_filename) // moved to .h
{
	b_valid = SetFilename(p_s_filename) && GetInfo();
	// set filename and get info about the file
}*/

const char *TFileInfo::p_s_FileName() const
{
	size_t n_pos;
#if defined(_WIN32) || defined(_WIN64)
	if((n_pos = s_filename.find_last_of("/\\")) != std::string::npos)
#else // defined(_WIN32) || defined(_WIN64)
	if((n_pos = s_filename.rfind(CDirectory::path_Separator)) != std::string::npos)
#endif // defined(_WIN32) || defined(_WIN64)
		return s_filename.c_str() + n_pos + 1;
	else
		return s_filename.c_str();
}

const char *TFileInfo::p_s_FileName(const char *p_s_path)
{
	_ASSERTE(p_s_path); // may not be null
	const char *p_s_filename;
#if defined(_WIN32) || defined(_WIN64)
	if((p_s_filename = strrchr(p_s_path, '\\'))) { // todo - transform to a loop, declare a macro for filesystem accepting multiple path separators, declare a constant for path separator variants (CDirectory::path_Separator still remains a preferred separator)
		const char *p_s_filename2;
		if((p_s_filename2 = strrchr(p_s_filename, '/')))
			p_s_filename = p_s_filename2;
		return p_s_filename + 1;
	} else if((p_s_filename = strrchr(p_s_path, '/')))
		return p_s_filename + 1;
	else
		return p_s_path;
#else // defined(_WIN32) || defined(_WIN64)
	if((p_s_filename = strrchr(p_s_path, CDirectory::path_Separator)))
		return p_s_filename + 1;
	else
		return p_s_path;
#endif // defined(_WIN32) || defined(_WIN64)
}

const char *TFileInfo::p_s_Extension(const char *p_s_path)
{
	_ASSERTE(p_s_path); // may not be null
	const char *p_s_ext = strrchr(p_s_path, '.');
#if defined(_WIN32) || defined(_WIN64)
	if(p_s_ext && !strchr(p_s_ext, '/') && !strchr(p_s_ext, '\\'))
#else // defined(_WIN32) || defined(_WIN64)
	if(p_s_ext && !strchr(p_s_ext, CDirectory::path_Separator))
#endif // defined(_WIN32) || defined(_WIN64)
		return p_s_ext + 1;
	return "";
}

const char *TFileInfo::p_s_Extension() const
{
	size_t n_pos;
	if((n_pos = s_filename.rfind('.')) != std::string::npos) {
		size_t n_pos2;
#if defined(_WIN32) || defined(_WIN64)
		if((n_pos2 = s_filename.find_first_of("/\\", n_pos)) != std::string::npos)
#else // defined(_WIN32) || defined(_WIN64)
		if((n_pos2 = s_filename.find(CDirectory::path_Separator, n_pos)) != std::string::npos)
#endif // defined(_WIN32) || defined(_WIN64)
			return ""; // there is no '.' after last path separator
		return s_filename.c_str() + n_pos + 1;
	} else
		return ""; // no extension
}

/*
 *	bool TFileInfo::SetFilename(const char *p_s_new_filename)
 *		- sets filename to p_s_new_filename
 *		- returns true on success, false on failure
 *		- note p_s_new_filename can be 0 (sets this filename to 0 as well)
 */
bool TFileInfo::SetFilename(const char *p_s_new_filename)
{
	if(p_s_new_filename) {
		if(!stl_ut::AssignCStr(s_filename, p_s_new_filename))
			return false;

		__STL_UT_TRY {
			s_filename.c_str();
		} __STL_UT_CATCH(std::bad_alloc) {
			return false;
		}
		// make sure c_str() won't throw
	} else
		s_filename.erase();
	// copy filename

	return true;
}

/*
 *	bool TFileInfo::operator =(const TFileInfo &r_t_file_info)
 *		- copy-operator
 *		- returns true on success, false in case there
 *		  was not enough memory to hold filename string
 */
bool TFileInfo::operator =(const TFileInfo &r_t_file_info)
{
	if(!stl_ut::Assign(s_filename, r_t_file_info.s_filename)) {
		b_valid = false;
		return false;
	}
	// copy filename

	b_valid = r_t_file_info.b_valid;
	b_exists = r_t_file_info.b_exists;
	b_directory = r_t_file_info.b_directory;
	n_mode = r_t_file_info.n_mode;
	n_flags = r_t_file_info.n_flags;
	n_size_lo = r_t_file_info.n_size_lo;
	n_size_hi = r_t_file_info.n_size_hi;
	for(int n = 0; n < 3; ++ n)
		p_time[n] = r_t_file_info.p_time[n];
	// copy static data

	return true;
}

bool TFileInfo::Get_TempFileName(std::string &r_s_temp_file_name, const char *p_s_app_id)
{
	_ASSERTE(p_s_app_id && strlen(p_s_app_id));
	// may not be emtpy

#if defined(_WIN32) || defined(_WIN64)
	std::string s_temp_path;
	if(!stl_ut::Resize_To_N(s_temp_path, GetTempPath(0, NULL) + 1) ||
	   !GetTempPathA((DWORD)s_temp_path.size(), &s_temp_path[0])) // size won't exceed DWORD_MAX, since it's read from DWORD
		return false; // something went wrong
	s_temp_path.resize(strlen(s_temp_path.c_str()));
	// get temp path (eg. "c:\windows\temp")

	if(!stl_ut::Resize_To_N(r_s_temp_file_name, s_temp_path.length() + 16 + strlen(p_s_app_id)) ||
	   !GetTempFileNameA(s_temp_path.c_str(), p_s_app_id, 0, &r_s_temp_file_name[0]))
		return false; // something went wrong
	r_s_temp_file_name.resize(strlen(r_s_temp_file_name.c_str()));
	// get temp filename
#else // _WIN32 || _WIN64
	std::string s_tmp;
	if(!CPath::Get_TempDirectory(s_tmp) || // use "proper" temp (e.g. on cluster computers, temp is often directory specific to the job id, like "/tmp/pbs.132048.dm2")
	   !stl_ut::Format(r_s_temp_file_name, "%s/%sXXXXXX", s_tmp.c_str(), p_s_app_id)) // had 8 X's // 2012-07-17 change // t_odo - carry this change to documentation
		return false;														// 2013-11-13 changed template to not include the /tmp folder as that might not be the location of tmp
	// make template

	int n_file;
	if((n_file = mkstemp((char*)r_s_temp_file_name.c_str())) < 0)
		return false;
	close(n_file);
	// create temp file
#endif // _WIN32 || _WIN64

	return true;
}

bool TFileInfo::Get_TempFileName_Dir(std::string &r_s_temp_file_name,
	const char *p_s_directory, const char *p_s_app_id)
{
	if(!p_s_directory)
		return Get_TempFileName(r_s_temp_file_name, p_s_app_id);

	_ASSERTE(p_s_app_id && strlen(p_s_app_id)); // may not be emtpy

#if defined(_WIN32) || defined(_WIN64)
	if(!stl_ut::Resize_To_N(r_s_temp_file_name, strlen(p_s_directory) + 16 + strlen(p_s_app_id)) ||
	   !GetTempFileNameA(p_s_directory, p_s_app_id, 0, &r_s_temp_file_name[0]))
		return false; // something went wrong
	r_s_temp_file_name.resize(strlen(r_s_temp_file_name.c_str()));
	// get temp filename
#else // _WIN32 || _WIN64
	if(!stl_ut::Format(r_s_temp_file_name, "%s/%sXXXXXX", p_s_directory, p_s_app_id)) // had 8 X's // 2012-07-17 change // t_odo - carry this change to documentation
		return false;														// 2013-11-13 changed template to not include the /tmp folder as that might not be the location of tmp
	// make template

	int n_file;
	if((n_file = mkstemp((char*)r_s_temp_file_name.c_str())) < 0)
		return false;
	close(n_file);
	// create temp file
#endif // _WIN32 || _WIN64

	return true;
}

#if defined(_WIN32) || defined(_WIN64)
bool TFileInfo::GetInfo()
{
	const char *p_s_filename = p_s_Path();

	if(!*p_s_filename)
		p_s_filename = ".";
	// an empty path means the current direcotry

	WIN32_FIND_DATAA t_find_data;
	if(!p_s_filename) {
		b_valid = false; // isn't valid
		b_exists = false;
		return true;
	}
	HANDLE h_find;
	if((h_find = FindFirstFileA(p_s_filename, &t_find_data)) == INVALID_HANDLE_VALUE) {
		char p_s_buffer[4] = {0};
		if(GetFullPathNameA(p_s_filename, 4, p_s_buffer, NULL) > 0 && p_s_buffer[3] == 0) {
			if(strlen(p_s_buffer) == 3 && isalpha(p_s_buffer[0]) && p_s_buffer[1] == ':' && p_s_buffer[2] == '\\')
				p_s_filename = p_s_buffer;
		}
		// try getting full path, in case it looks like root directory, use it

		if(strlen(p_s_filename) == 3 && isalpha(p_s_filename[0]) && p_s_filename[1] == ':' && p_s_filename[2] == '\\') {
			memset(&t_find_data, 0, sizeof(t_find_data));
			t_find_data.dwFileAttributes = GetFileAttributesA(p_s_filename);
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
			strncpy_s(t_find_data.cFileName, sizeof(t_find_data.cFileName), p_s_Path(), sizeof(t_find_data.cFileName) / sizeof(t_find_data.cFileName[0]));
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			strncpy(t_find_data.cFileName, p_s_Path(), sizeof(t_find_data.cFileName) / sizeof(t_find_data.cFileName[0]));
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			t_find_data.cFileName[sizeof(t_find_data.cFileName) /
				sizeof(t_find_data.cFileName[0])] = 0; // make sure it's null-terminated
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
			strncpy_s(t_find_data.cAlternateFileName, sizeof(t_find_data.cAlternateFileName), p_s_filename, sizeof(t_find_data.cAlternateFileName) / sizeof(t_find_data.cAlternateFileName[0]));
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			strncpy(t_find_data.cAlternateFileName, p_s_filename, sizeof(t_find_data.cAlternateFileName) / sizeof(t_find_data.cAlternateFileName[0]));
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			t_find_data.cAlternateFileName[sizeof(t_find_data.cAlternateFileName) /
				sizeof(t_find_data.cAlternateFileName[0])] = 0; // make sure it's null-terminated
			// fill WIN32_FIND_DATA

			/*HANDLE h_file;
			if((h_file = CreateFile(p_s_filename, 0, FILE_SHARE_READ | FILE_SHARE_WRITE,
			   NULL, OPEN_EXISTING, 0, NULL)) != INVALID_HANDLE_VALUE) {
			    GetFileTime(h_file, &t_find_data.ftCreationTime,
					&t_find_data.ftLastAccessTime, &t_find_data.ftLastWriteTime);
				CloseHandle(h_file);
			}
			// fill ftCreationTime, ftLastAccessTime and ftLastWriteTime*/
			// roots doesn't have file time under windows

			return GetInfo(t_find_data);
		}
		// this is root directory, FindFirstFile() cannot access those

		b_valid = true;
		b_exists = false; // doesn't exist
		return true;
	}
	FindClose(h_find); // !!
	// find the file

	return GetInfo(t_find_data);
	// use overload
}

bool TFileInfo::GetInfo(const WIN32_FIND_DATAA &r_t_find_data)
{
	//_ASSERTE(!strcmpi(p_s_FileName(), 
	/*if(!stl_ut::AssignCStr(s_filename, r_t_find_data.cFileName)) {
		b_valid = false;
		return false;
	}*/
	// make sure it's the right WIN32_FIND_DATA

	const char *p_s_filename = p_s_Path();
	if(!p_s_filename) {
		b_valid = false;
		return false;
	}
	// get filename

	if(!*p_s_filename)
		p_s_filename = ".";
	// an empty path means the current direcotry

	b_exists = true;
	//_ASSERTE(!_access(p_s_filename, 00)); // it exist, right? // it routinely fails on files with unicode names
	// we have r_t_find_data, file does exist

	if(!_access(p_s_filename, 06))
		n_mode = flag_Read | flag_Write;
	else if(!_access(p_s_filename, 02))
		n_mode = flag_Read;
	else if(!_access(p_s_filename, 04))
		n_mode = flag_Write;
	// what about file mode?

	n_flags = 0;
	if(_access(p_s_filename, 2) == 0)
		n_flags |= (int)TFileInfo::flag_Write;
	if(_access(p_s_filename, 4) == 0)
		n_flags |= (int)TFileInfo::flag_Read | (int)TFileInfo::flag_Execute;
	// get access flags

	b_directory = (r_t_find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
	n_size_lo = r_t_find_data.nFileSizeLow;
	n_size_hi = r_t_find_data.nFileSizeHigh;

	p_time[TFileInfo::time_Creation] = TFileInfo::TTime(r_t_find_data.ftCreationTime);
	p_time[TFileInfo::time_LastAccess] = TFileInfo::TTime(r_t_find_data.ftLastAccessTime);
	p_time[TFileInfo::time_LastWrite] = TFileInfo::TTime(r_t_find_data.ftLastWriteTime);
	if(p_time[TFileInfo::time_Creation].n_year < 0 ||
	   p_time[TFileInfo::time_LastAccess].n_year < 0 ||
	   p_time[TFileInfo::time_LastWrite].n_year < 0) {
		b_valid = false;
		return false;
	}
	// get time

	b_valid = true;
	// file data are valid

	return true;
}
#else // _WIN32 || _WIN64
bool TFileInfo::GetInfo()
{
	const char *p_s_filename = p_s_Path();
	if(!p_s_filename) {
		b_valid = false;
		return false;
	}
	// get file name

	if(!*p_s_filename)
		p_s_filename = ".";
	// an empty path means the current direcotry

	struct stat t_stat;
	if(stat(p_s_filename, &t_stat)) {
		b_exists = false;
		b_valid = true; // doesn't exist
		return true;
	}
	// get file stat

	b_directory = S_ISDIR(t_stat.st_mode) != 0;// == S_IFDIR;
	// is it a directory?

	if(!access(p_s_filename, 00))
		b_exists = true;
	else
		b_exists = false;
	// does it exist?

	if(!access(p_s_filename, 06))
		n_mode = flag_Read | flag_Write;
	else if(!access(p_s_filename, 02))
		n_mode = flag_Read;
	else if(!access(p_s_filename, 04))
		n_mode = flag_Write;
	// what about file mode?

	n_flags = 0;
	if(t_stat.st_mode & S_IROTH)
		n_flags |= (int)TFileInfo::flag_other_Read;
	if(t_stat.st_mode & S_IWOTH)
		n_flags |= (int)TFileInfo::flag_other_Write;
	if(t_stat.st_mode & S_IXOTH)
		n_flags |= (int)TFileInfo::flag_other_Execute;
	//
	if(t_stat.st_mode & S_IRGRP)
		n_flags |= (int)TFileInfo::flag_group_Read;
	if(t_stat.st_mode & S_IWGRP)
		n_flags |= (int)TFileInfo::flag_group_Write;
	if(t_stat.st_mode & S_IXGRP)
		n_flags |= (int)TFileInfo::flag_group_Execute;
	//
	if(t_stat.st_mode & S_IRUSR)
		n_flags |= (int)TFileInfo::flag_owner_Read;
	if(t_stat.st_mode & S_IWUSR)
		n_flags |= (int)TFileInfo::flag_owner_Write;
	if(t_stat.st_mode & S_IXUSR)
		n_flags |= (int)TFileInfo::flag_owner_Execute;
	// get file / directory flags

#if 1 // (sizeof(t_off)) > 32
	n_size_lo = uint64_t(t_stat.st_size) & 0xffffffff;
	n_size_hi = uint64_t(t_stat.st_size) >> 32;
#else // this might prevent some warnings, if (sizeof(t_off)) <= 32
	n_size_lo = t_stat.st_size;
	n_size_hi = 0;
#endif
	p_time[TFileInfo::time_Creation] = TFileInfo::TTime(localtime(&t_stat.st_ctime));
	p_time[TFileInfo::time_LastAccess] = TFileInfo::TTime(localtime(&t_stat.st_atime));
	p_time[TFileInfo::time_LastWrite] = TFileInfo::TTime(localtime(&t_stat.st_mtime));
	// fill output structure

	b_valid = true;
	// file info is valid

	return true;
}
#endif // _WIN32 || _WIN64

/*
 *								=== ~TFileInfo ===
 */

/*
 *								=== CDirectory ===
 */

void CDirectory::Initialize(const char *p_s_dir_name)
{
	_ASSERTE(p_s_dir_name);
	if(!*p_s_dir_name)
		p_s_dir_name = ".";
	// an empty path means the current direcotry

#if defined(_WIN32) || defined(_WIN64)
	m_h_prev_file = 0;
#else // _WIN32 || _WIN64
	m_p_dir = 0;
#endif // _WIN32 || _WIN64

#if defined(_WIN32) || defined(_WIN64)
	int n_extra_chars = (p_s_dir_name[strlen(p_s_dir_name) - 1] == '\\' ||
		p_s_dir_name[strlen(p_s_dir_name) - 1] == '/')? 2 : 3;
	if(m_p_s_dir_name = new(std::nothrow) char[strlen(p_s_dir_name) + n_extra_chars]) {
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		size_t n_size = (strlen(p_s_dir_name) + n_extra_chars) * sizeof(char);
		strcpy_s(m_p_s_dir_name, n_size, p_s_dir_name);
		strcat_s(m_p_s_dir_name, n_size, (n_extra_chars == 2)? "*" : "\\*");
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		strcpy(m_p_s_dir_name, p_s_dir_name);
		strcat(m_p_s_dir_name, (n_extra_chars == 2)? "*" : "\\*");
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		// make sure m_p_s_dir_name ends with \*, i.e. "c:\blah\*" or "e:\*"

		_ASSERTE(path_Separator == '\\');
		for(char *p_s_str = m_p_s_dir_name; *p_s_str; ++ p_s_str) {
			if(*p_s_str == '/')
				*p_s_str = path_Separator;
		}
		// make sure path separator is the right one
	}
#else // _WIN32 || _WIN64
	if((m_p_s_dir_name = new(std::nothrow) char[strlen(p_s_dir_name) + 1])) {
		strcpy(m_p_s_dir_name, p_s_dir_name);
		while(m_p_s_dir_name[strlen(m_p_s_dir_name) - 1] == '\\' ||
		   m_p_s_dir_name[strlen(m_p_s_dir_name) - 1] == '/')
			m_p_s_dir_name[strlen(m_p_s_dir_name) - 1] = 0;
		// make sure m_p_s_dir_name ends with dir name, i.e. "c:\blah" ("blah") or "e:" ("e:")

		_ASSERTE(path_Separator == '/');
		for(char *p_s_str = m_p_s_dir_name; *p_s_str; ++ p_s_str) {
			if(*p_s_str == '\\')
				*p_s_str = path_Separator;
		}
		// make sure path separator is the right one
	}
#endif // _WIN32 || _WIN64
}

/*
 *	CDirectory::~CDirectory()
 *		- default destructor
 */
CDirectory::~CDirectory()
{
	if(m_p_s_dir_name) {
		delete[] m_p_s_dir_name;
		m_p_s_dir_name = 0;
	}
#if !defined(_WIN32) && !defined(_WIN64)
	if(m_p_dir) {
		closedir(m_p_dir);
		m_p_dir = 0;
	}
#else // _WIN32 || _WIN64
	if(m_h_prev_file)
		FindClose(m_h_prev_file);
	// be polite and close handles
#endif // _WIN32 || _WIN64
}

#if defined(_WIN32) || defined(_WIN64)
/*
 *	bool CDirectory::Get_FirstFile(TFileInfo &r_t_file)
 *		- gets first file in the directory, copies it's data to r_t_file
 *		- in case the directory doesn't have any files, r_t_file.b_valid is set to false
 *		- returns true on success, false on failure (possibly inaccesible directory)
 */
bool CDirectory::Get_FirstFile(TFileInfo &r_t_file)
{
	WIN32_FIND_DATAA t_find_data;

	if(m_h_prev_file)
		FindClose(m_h_prev_file);
	// be polite and close handles

	if((m_h_prev_file = FindFirstFileA(m_p_s_dir_name,
	   &t_find_data)) == INVALID_HANDLE_VALUE) {
		r_t_file.SetNoFile();
		return true;
	}
	// no files

	if(!strcmp(".", t_find_data.cFileName) || !strcmp("..", t_find_data.cFileName))
		return Get_NextFile(r_t_file);
	// don't want self '.' and up '..'

	return GetFileInfo(t_find_data, r_t_file);
}

/*
 *	bool CDirectory::Get_NextFile(TFileInfo &r_t_file)
 *		- gets next file in the directory, copies it's data to r_t_file
 *		- in case the directory doesn't have any more files, r_t_file.b_valid is set to false
 *		- returns true on success, false on failure (possibly inaccesible directory)
 *		- in case Get_FirstFile was not called prior calling Get_NextFile, it's called
 *		  automatically. in order to get file list for this folder again, Get_FirstFile
 *		  must be called (r_t_file contents are only written to, not read)
 */
bool CDirectory::Get_NextFile(TFileInfo &r_t_file)
{
	WIN32_FIND_DATAA t_find_data;

	if(!m_h_prev_file)
		return Get_FirstFile(r_t_file);
	while(1) {
		if(!FindNextFileA(m_h_prev_file, &t_find_data)) {
			r_t_file.SetNoFile();
			return true;
		}
		// no more files

		if(!strcmp(".", t_find_data.cFileName) || !strcmp("..", t_find_data.cFileName))
			continue;
		// don't want self '.' and up '..'

		break;
	}

	return GetFileInfo(t_find_data, r_t_file);
}

bool CDirectory::GetFileInfo(const WIN32_FIND_DATAA &r_t_find_data, TFileInfo &r_t_file)
{
	char *p_s_full_name;
	size_t n_length = strlen(r_t_find_data.cFileName) + strlen(m_p_s_dir_name);
	if(!(p_s_full_name = new(std::nothrow) char[n_length])) {
		r_t_file.SetInvalid();
		return false;
	}
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	strcpy_s(p_s_full_name, n_length * sizeof(char), m_p_s_dir_name);
	strcpy_s(p_s_full_name + strlen(p_s_full_name) - 1,
		(n_length - strlen(p_s_full_name) + 1) * sizeof(char), r_t_find_data.cFileName);
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	strcpy(p_s_full_name, m_p_s_dir_name);
	strcpy(p_s_full_name + strlen(p_s_full_name) - 1, r_t_find_data.cFileName);
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	// full filename (overwrite '*' from m_p_s_dir_name -> no extra space for terminating 0)

	if(!r_t_file.SetFilename(p_s_full_name)) {
		delete[] p_s_full_name;
		r_t_file.SetInvalid();
		return false;
	}
	// copy full filename

	delete[] p_s_full_name;
	// cleanup

	return r_t_file.GetInfo(r_t_find_data);
	// update file info

	/*r_t_file.n_flags = 0;
	if(_access(p_s_full_name, 2) == 0)
		r_t_file.n_flags |= (int)TFileInfo::flag_Write;
	if(_access(p_s_full_name, 4) == 0)
		r_t_file.n_flags |= (int)TFileInfo::flag_Read | (int)TFileInfo::flag_Execute;
	// get access flags

	delete[] p_s_full_name;

	r_t_file.b_directory = (r_t_find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
	r_t_file.n_size_lo = r_t_find_data.nFileSizeLow;
	r_t_file.n_size_hi = r_t_find_data.nFileSizeHigh;

	r_t_file.p_time[TFileInfo::time_Creation] = TFileInfo::TTime(r_t_find_data.ftCreationTime);
	r_t_file.p_time[TFileInfo::time_LastAccess] = TFileInfo::TTime(r_t_find_data.ftLastAccessTime);
	r_t_file.p_time[TFileInfo::time_LastWrite] = TFileInfo::TTime(r_t_find_data.ftLastWriteTime);
	if(r_t_file.p_time[TFileInfo::time_Creation].n_year < 0 ||
	   r_t_file.p_time[TFileInfo::time_LastAccess].n_year < 0 ||
	   r_t_file.p_time[TFileInfo::time_LastWrite].n_year < 0) {
		r_t_file.b_valid = false;
		return false;
	}
	// get time

	r_t_file.b_valid = true;
	// file data are valid

	return true;*/
}
#else // _WIN32 || _WIN64
/*
 *	bool Get_FirstFile(TFileInfo &r_t_file)
 *		- gets first file in the directory, copies it's data to r_t_file
 *		- in case the directory doesn't have any files, r_t_file.b_valid is set to false
 *		- returns true on success, false on failure (possibly inaccesible directory)
 */
bool CDirectory::Get_FirstFile(TFileInfo &r_t_file)
{
	if(m_p_dir) {
		closedir(m_p_dir);
		m_p_dir = 0;
	}

	if(!(m_p_dir = opendir(m_p_s_dir_name))) {
		//fprintf(stderr, "error: opendir(%s)\n", m_p_s_dir_name);
		r_t_file.SetInvalid();
		return false;
	}

	return Get_NextFile(r_t_file);
}

/*
 *	bool Get_NextFile(TFileInfo &r_t_file)
 *		- gets next file in the directory, copies it's data to r_t_file
 *		- in case the directory doesn't have any more files, r_t_file.b_valid is set to false
 *		- returns true on success, false on failure (possibly inaccesible directory)
 *		- in case Get_FirstFile was not called prior calling Get_NextFile, it's called
 *		  automatically. in order to get file list for this folder again, Get_FirstFile
 *		  must be called (r_t_file contents are only written to, not read)
 */
bool CDirectory::Get_NextFile(TFileInfo &r_t_file)
{
	if(!m_p_dir)
		return Get_FirstFile(r_t_file);

	for(;;) {
		struct dirent *p_file_record;
		if(!(p_file_record = readdir(m_p_dir))) {
			closedir(m_p_dir);
			m_p_dir = 0;
			r_t_file.SetNoFile();
			return true;
		}
		if(!strcmp(p_file_record->d_name, ".") || !strcmp(p_file_record->d_name, ".."))
			continue;
		// get filename

		char *p_s_full_name;
		if(!(p_s_full_name = new(std::nothrow) char[strlen(p_file_record->d_name) + strlen(m_p_s_dir_name) + 2])) {
			r_t_file.SetInvalid();
			return false;
		}
		strcpy(p_s_full_name, m_p_s_dir_name);
		strcat(p_s_full_name, "/");
		strcat(p_s_full_name, p_file_record->d_name);
		// full filename (overwrite '*' from m_p_s_dir_name -> no extra space for terminating 0)

		if(!r_t_file.SetFilename(p_s_full_name)) {
			delete[] p_s_full_name;
			r_t_file.SetInvalid();
			return false;
		}
		// copy full filename

		delete[] p_s_full_name;
		// don't need this anymore

		return r_t_file.GetInfo();
		// get info about file

		/*if(!r_t_file.SetFilename(p_s_full_name)) {
			delete[] p_s_full_name;
			r_t_file.b_valid = false;
			return false;
		}
		// copy full filename

		delete[] p_s_full_name;

		r_t_file.b_directory = (p_file_record->d_type & DT_DIR) != 0; // todo

		struct stat t_stat;
		if(stat(r_t_file.p_s_Filename(), &t_stat))
			return false;
		// get file stat

		r_t_file.n_flags = 0;
		if(t_stat.st_mode & S_IROTH)
			r_t_file.n_flags |= (int)TFileInfo::flag_other_Read;
		if(t_stat.st_mode & S_IWOTH)
			r_t_file.n_flags |= (int)TFileInfo::flag_other_Write;
		if(t_stat.st_mode & S_IXOTH)
			r_t_file.n_flags |= (int)TFileInfo::flag_other_Execute;
		//
		if(t_stat.st_mode & S_IRGRP)
			r_t_file.n_flags |= (int)TFileInfo::flag_group_Read;
		if(t_stat.st_mode & S_IWGRP)
			r_t_file.n_flags |= (int)TFileInfo::flag_group_Write;
		if(t_stat.st_mode & S_IXGRP)
			r_t_file.n_flags |= (int)TFileInfo::flag_group_Execute;
		//
		if(t_stat.st_mode & S_IRUSR)
			r_t_file.n_flags |= (int)TFileInfo::flag_owner_Read;
		if(t_stat.st_mode & S_IWUSR)
			r_t_file.n_flags |= (int)TFileInfo::flag_owner_Write;
		if(t_stat.st_mode & S_IXUSR)
			r_t_file.n_flags |= (int)TFileInfo::flag_owner_Execute;
		// get flags

#if 1 // (sizeof(t_off)) > 32
		r_t_file.n_size_lo = uint64_t(t_stat.st_size) & 0xffffffff;
		r_t_file.n_size_hi = uint64_t(t_stat.st_size) >> 32;
#else // this might prevent some warnings, if (sizeof(t_off)) <= 32
		r_t_file.n_size_lo = t_stat.st_size;
		r_t_file.n_size_hi = 0;
#endif
		r_t_file.p_time[TFileInfo::time_Creation] = TFileInfo::TTime(localtime(&t_stat.st_ctime));
		r_t_file.p_time[TFileInfo::time_LastAccess] = TFileInfo::TTime(localtime(&t_stat.st_atime));
		r_t_file.p_time[TFileInfo::time_LastWrite] = TFileInfo::TTime(localtime(&t_stat.st_mtime));
		// fill output structure

		return true;*/
	}
}

#endif // _WIN32 || _WIN64

/*
 *								=== ~CDirectory ===
 */

/*
 *								=== CPath ===
 */

#if 0

bool CPath::UnitTests()
{
	{
		std::string s_temp;
		if(!Get_TempDirectory(s_temp))
			return false;
		std::string s_temp_file;
		if(!TFileInfo::Get_TempFileName(s_temp_file))
			return false;
		DeleteFile(s_temp_file.c_str()); // don't leave rubbish
		s_temp_file.erase(s_temp_file.rfind('\\'));
		_ASSERTE(!s_temp_file.compare(s_temp));
		printf("Get_TempDirectory() returns \'%s\'\n", s_temp.c_str());
	}
	// test Get_TempDirectory()

	{
		std::string s_cur;
		if(!Get_CurrentDirectory(s_cur))
			return false;
		printf("Get_CurrentDirectory() returns \'%s\'\n", s_cur.c_str());
		system("mkdir _test_gcwd");
		//system("cd _test_gcwd"); // has no effect
		_chdir((s_cur + "\\_test_gcwd").c_str());
		printf("mkdir _test_gcwd\ncd _test_gcwd\n");
		std::string s_cur2;
		if(!Get_CurrentDirectory(s_cur2))
			return false;
		_ASSERTE(s_cur2 == (s_cur + "\\_test_gcwd"));
		printf("Get_CurrentDirectory() returns \'%s\'\n", s_cur2.c_str());
		//system("cd .."); // has no effect
		_chdir((s_cur + "\\_test_gcwd\\..").c_str());
		printf("cd ..\n");
		std::string s_cur3;
		if(!Get_CurrentDirectory(s_cur3))
			return false;
		_ASSERTE(s_cur3 == s_cur);
		system("rmdir _test_gcwd");
		printf("Get_CurrentDirectory() returns \'%s\'\n", s_cur3.c_str());
	}
	// test Get_CurrentDirectory

#ifdef __CPATH_GET_USER_HOME_DIRECTORY
	{
		std::string s_home;
		if(!Get_UserHomeDirectory(s_home))
			return false;
		printf("Get_UserHomeDirectory() returns \'%s\'\n", s_home.c_str());
	}
#else // __CPATH_GET_USER_HOME_DIRECTORY
	printf("Get_UserHomeDirectory() not compiled\n");
#endif // __CPATH_GET_USER_HOME_DIRECTORY
	// test Get_UserHomeDirectory

	{
		std::string s_norm;
		if(!WeakNormalize(s_norm, "c:\\foo\\bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c:/foo/bar")) // bad slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c:/foo/./bar")) // dot
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c:/foo//bar")) // double slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c://foo//bar")) // more double slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c://foo//bar//")) // even more double slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c:/foo/barbar/../bar")) // double dot
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c:/foo/barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c://foo//barbar//..//bar//.///")) // more slashes along with "complicated" path
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "c:/foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "./foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("foo\\bar"));
		if(!WeakNormalize(s_norm, "../foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("..\\foo\\bar"));
		if(!WeakNormalize(s_norm, "../foo/./barbar/../../../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("..\\..\\bar"));
		if(!WeakNormalize(s_norm, "../foo/./barbar/../../../bar/.."))
			return false;
		_ASSERTE(!s_norm.compare("..\\.."));
		if(!WeakNormalize(s_norm, "../../../../.."))
			return false;
		_ASSERTE(!s_norm.compare("..\\..\\..\\..\\.."));
		if(!WeakNormalize(s_norm, ""))
			return false;
		_ASSERTE(!s_norm.compare(""));
		if(!WeakNormalize(s_norm, "."))
			return false;
		_ASSERTE(!s_norm.compare(""));
		if(!WeakNormalize(s_norm, ".."))
			return false;
		_ASSERTE(!s_norm.compare(".."));
		if(!WeakNormalize(s_norm, "/"))
			return false;
		_ASSERTE(!s_norm.compare(""));
		if(!WeakNormalize(s_norm, "c:/.."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\..")); // can't remove .. due to absolute path
		if(!WeakNormalize(s_norm, "//192.168.0.2/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\.."));
		if(!WeakNormalize(s_norm, "//192.168.0.2\\foo\\bar"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/foo/bar")) // bad slashes
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/foo/./bar")) // dot
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/foo/barbar/../bar")) // double dot
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/foo/barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/./foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/../foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/../foo/./barbar/../../../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\..\\bar"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/../foo/./barbar/../../../bar/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\.."));
		if(!WeakNormalize(s_norm, "//192.168.0.2/../../../../.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\..\\..\\..\\.."));
		if(!WeakNormalize(s_norm, "//192.168.0.2"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2"));
		if(!WeakNormalize(s_norm, "//192.168.0.2/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\.."));
		if(!WeakNormalize(s_norm, "//192.168.0.2/"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2"));
		if(!WeakNormalize(s_norm, "//./COM1"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\COM1"));
		if(!WeakNormalize(s_norm, "//./COM1/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\"));
		if(!WeakNormalize(s_norm, "//./COM1/../.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\.."));
		if(!WeakNormalize(s_norm, "//./../COM1/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\.."));
		if(!WeakNormalize(s_norm, "//./../COM1"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\..\\COM1"));
		if(!WeakNormalize(s_norm, "//?/GLOBALROOT"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\GLOBALROOT"));
		if(!WeakNormalize(s_norm, "//?/GLOBALROOT/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\"));
		if(!WeakNormalize(s_norm, "//?/GLOBALROOT/../.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\.."));
		if(!WeakNormalize(s_norm, "//?//../GLOBALROOT"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\..\\GLOBALROOT"));
		if(!WeakNormalize(s_norm, "//?//../GLOBALROOT/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\.."));
		if(!WeakNormalize(s_norm, "//./c:/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//?/c:/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//?/UNC//192.168.0.2/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!WeakNormalize(s_norm, "//?/UNC/c:/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
	}
	// test WeakNormalize

	// todo - debug the following section and write the rest of the unit tests
	// todo - use comparison, not assertion (or use runtime assert)
	{
		std::string s_norm;
		if(!Normalize(s_norm, "c:\\foo\\bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c:/foo/bar")) // bad slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c:/foo/./bar")) // dot
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c:/foo//bar")) // double slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c://foo//bar")) // more double slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c://foo//bar//")) // even more double slashes
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar\\"));
		if(!Normalize(s_norm, "c:/foo/barbar/../bar")) // double dot
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c:/foo/barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "c://foo//barbar//..//bar//.///")) // more slashes along with "complicated" path
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar\\"));
		if(!Normalize(s_norm, "c:/foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "./foo/./barbar/../bar/."))
			return false;
		std::string s_cwd;
		if(!Get_CurrentDirectory(s_cwd))
			return false;
		_ASSERTE(!s_norm.compare(s_cwd + "\\foo\\bar"));
		if(!Normalize(s_norm, "c:/../foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare(s_cwd + "c:\\foo\\bar")); // WeakNormalize doesn't erase double dots where not applicable (such as at drive root), but Normalize does
		if(!Normalize(s_norm, "c:/../foo/./barbar/../../../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\bar"));
		if(!Normalize(s_norm, "c:/../foo/./barbar/../../../bar/.."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\"));
		if(!Normalize(s_norm, "c:/../../../../.."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\"));
		if(!Normalize(s_norm, ""))
			return false;
		_ASSERTE(!s_norm.compare(""));
		if(!Normalize(s_norm, "."))
			return false;
		_ASSERTE(!s_norm.compare(s_cwd));
		if(!Normalize(s_norm, "c:/.."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\"));
		if(!Normalize(s_norm, "c:/"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\"));
		if(!Normalize(s_norm, "c:/.."))
			return false;
		_ASSERTE(!s_norm.compare("c:\\"));
		if(!Normalize(s_norm, "//192.168.0.2/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\.."));
		if(!Normalize(s_norm, "//192.168.0.2\\foo\\bar"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/foo/bar")) // bad slashes
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/foo/./bar")) // dot
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/foo/barbar/../bar")) // double dot
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/foo/barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/./foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/../foo/./barbar/../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\foo\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/../foo/./barbar/../../../bar/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\..\\bar"));
		if(!Normalize(s_norm, "//192.168.0.2/../foo/./barbar/../../../bar/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\.."));
		if(!Normalize(s_norm, "//192.168.0.2/../../../../.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\..\\..\\..\\..\\.."));
		if(!Normalize(s_norm, "//192.168.0.2"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2"));
		if(!Normalize(s_norm, "//192.168.0.2/."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2"));
		if(!Normalize(s_norm, "//192.168.0.2/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\.."));
		if(!Normalize(s_norm, "//192.168.0.2/"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\"));
		if(!Normalize(s_norm, "//./COM1"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\COM1"));
		if(!Normalize(s_norm, "//./COM1/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\"));
		if(!Normalize(s_norm, "//./COM1/../.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\.."));
		if(!Normalize(s_norm, "//./../COM1/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\.."));
		if(!Normalize(s_norm, "//./../COM1"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\.\\..\\COM1"));
		if(!Normalize(s_norm, "//?/GLOBALROOT"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\GLOBALROOT"));
		if(!Normalize(s_norm, "//?/GLOBALROOT/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\"));
		if(!Normalize(s_norm, "//?/GLOBALROOT/../.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\.."));
		if(!Normalize(s_norm, "//?//../GLOBALROOT"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\..\\GLOBALROOT"));
		if(!Normalize(s_norm, "//?//../GLOBALROOT/.."))
			return false;
		_ASSERTE(!s_norm.compare("\\\\?\\.."));
		if(!Normalize(s_norm, "//./c:/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "//?/c:/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
		if(!Normalize(s_norm, "//?/UNC//192.168.0.2/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("\\\\192.168.0.2\\foo\\bar"));
		if(!Normalize(s_norm, "//?/UNC/c:/foo/bar"))
			return false;
		_ASSERTE(!s_norm.compare("c:\\foo\\bar"));
	}

	return true;
}

#endif // 0

bool CPath::Get_TempDirectory(std::string &r_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	if(!stl_ut::Resize_To_N(r_s_path, GetTempPath(0, NULL) + 1) ||
	   !GetTempPathA((DWORD)r_s_path.size(), &r_s_path[0])) // size won't exceed DWORD_MAX, since it's read from DWORD
		return false; // something went wrong
	_ASSERTE(strlen(r_s_path.c_str()) > 0 && r_s_path[strlen(r_s_path.c_str()) - 1] == '\\'); // the returned string ends with a backslash
	r_s_path.resize(strlen(r_s_path.c_str()) - 1); // cut the backslash here
	// get temp path (eg. "c:\\windows\\temp")
#else // _WIN32 || _WIN64
#if 0 // g++ linker warns about using mktemp(), don't want to use that anymore
	char p_s_temp[256] = "/tmp/fileXXXXXX";
	if(!mktemp(p_s_temp)) // do *not* use mkstemp(), do not want the file to be lying around
		return false;
	_ASSERTE(strrchr(p_s_temp + 1, '/')); // should contain slash
	*(char*)strrchr(p_s_temp + 1, '/') = 0; // erase the last slash (hence the string does not contain it)
	if(!stl_ut::AssignCStr(r_s_path, p_s_temp))
		return false;
	// get temp file name and erase the file name to get the path
#else // 0
	const char *p_s_temp = getenv("TMPDIR"); // environment variable
	// The caller must take care not to modify this string, since that would change the
	// environment of the process. Do not free it either.
	// (e.g. on cluster computers, temp is often directory specific to the job id, like "/tmp/pbs.132048.dm2")

	if(!p_s_temp) {
		if(P_tmpdir)
			p_s_temp = P_tmpdir; // in stdio.h
		else {
			p_s_temp = "/tmp"; // just hope it is there

			/*TFileInfo t_temp_info(p_s_temp);
			if(!t_temp_info.b_exists || !t_temp_info.b_directory)
				return false;*/
			// don't want to depend on Dir.cpp, some apps already use only the header
		}
	}
	// fallbacks if the environment variable is not set

	if(!stl_ut::AssignCStr(r_s_path, p_s_temp))
		return false;
	if(!r_s_path.empty() && r_s_path[r_s_path.length() - 1] == '/')
		r_s_path.erase(r_s_path.end() - 1);
	// get rid of the trailing slash
#endif // 0
#endif // _WIN32 || _WIN64

	_ASSERTE(!b_EndsWithSlash(r_s_path));
	_ASSERTE(b_Is_Normalized(r_s_path));
	_ASSERTE(b_Is_Absolute(r_s_path));
	// make sure there is no slash at the end, and that the path is normalized

	return true;
}

bool CPath::Get_CurrentDirectory(std::string &r_s_path)
{
	if(!stl_ut::Resize_To_N(r_s_path, 8))
		return false;
	for(;;) {
#if defined(_WIN32) || defined(_WIN64)
		_ASSERTE(r_s_path.size() - 1 <= INT_MAX); 
		if(_getcwd(&r_s_path[0], int(r_s_path.size() - 1))) {
#else // _WIN32 || _WIN64
		if(getcwd(&r_s_path[0], r_s_path.size() - 1)) { // different name, same function
#endif // _WIN32 || _WIN64
			r_s_path.resize(strlen(r_s_path.c_str()));
			break;
		}
		// in case the buffer is large enough, trim it

		if(!stl_ut::Resize_To_N(r_s_path, r_s_path.size() * 2))
			return false;
		// grow the buffer
	}

	Drop_TrailingSlash(r_s_path);
	_ASSERTE(b_Is_Normalized(r_s_path));
	_ASSERTE(b_Is_Absolute(r_s_path));
	// make sure there is no slash at the end, and that the path is normalized

	return true;
}

bool CPath::Get_EnvironmentVariable(std::string &r_s_value, bool &r_b_found, const char *p_s_variable)
{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER > 1200
	size_t n_length;
	if(getenv_s(&n_length, 0, 0, p_s_variable))
		return false;
	if(!n_length)
		r_b_found = false;
	else
		r_b_found = true;
	r_s_value.resize(n_length);
	if(n_length) { // only if found
		if(getenv_s(&n_length, &r_s_value[0], n_length, p_s_variable) || // either getenv failed
		   n_length > r_s_value.length()) // or someone changed it under our hands
			return false;
		r_s_value.resize(strlen(r_s_value.c_str())); // contract
	}
	return true;
#else // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
	const char *p_s_env = getenv(p_s_variable);
	r_b_found = p_s_env != 0;
	return stl_ut::AssignCStr(r_s_value, (p_s_env)? p_s_env : "");
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER > 1200
}

bool CPath::Get_ProgramsDirectory(std::string &r_s_path, bool b_alternative /*= false*/)
{
#if defined(_WIN32) || defined(_WIN64)
	bool b_found;
	if(!Get_EnvironmentVariable(r_s_path, b_found, (b_alternative)?
	   "ProgramFiles(x86)" : "ProgramFiles"))
		return false;

#if !(defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64))
	size_t n_pos;
	if(b_found && !b_alternative && (n_pos = r_s_path.find("(x86)")) != std::string::npos) {
		r_s_path.erase(n_pos);
		stl_ut::TrimSpace(r_s_path);
		if(!TFileInfo(r_s_path).b_exists)
			b_found = false;
	}
	// solve the problem with x86 programs not being told where is the x64 program files
#endif // !(_M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64)

	if(!b_found) {
		if(!b_found && (!Get_EnvironmentVariable(r_s_path, b_found, "SystemDrive") ||
		   !stl_ut::AppendCStr(r_s_path, (b_alternative)? "\\Program Files (x86)" : "\\Program Files"))) // guess
			return false;
		if(!b_found && !stl_ut::AssignCStr(r_s_path, (b_alternative)?
		   "C:\\Program Files (x86)" : "C:\\Program Files")) // guess
			return false;

		if(!TFileInfo(r_s_path).b_exists) {
			if(b_alternative)
				return Get_ProgramsDirectory(r_s_path, false); // get the regular one, don't fail
			return false;
		}
		// failed to locate it
	}
	Drop_TrailingSlash(r_s_path); // !!
	return true;
#else // _WIN32 || _WIN64
	return stl_ut::AssignCStr(r_s_path, (b_alternative)? "/usr/bin" : "/bin"); // guess
#endif // _WIN32 || _WIN64
}

#ifdef __CPATH_GET_USER_HOME_DIRECTORY

/**
 *	@brief gets the current logged-on user home directory
 *	@param[out] r_s_path is the path to the user home directory, never ends with a slash
 *	@return Returns true on success, false on failure.
 */
bool CPath::Get_UserHomeDirectory(std::string &r_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	HANDLE h_process = GetCurrentProcess(); // = -1
	HANDLE h_process_real = h_process;
	/*if(!DuplicateHandle(h_process, h_process, h_process,
	   &h_process_real, PROCESS_QUERY_INFORMATION, TRUE, 0))
		return false;*/
	HANDLE h_proc_token;
	if(!OpenProcessToken(h_process, TOKEN_QUERY, &h_proc_token)) {
		//CloseHandle(h_process_real);
		return false;
	}
	// note the handle returned by GetCurrentProcess() doesn't need to be closed

	DWORD n_size = MAX_PATH;
	GetUserProfileDirectoryA(h_proc_token, NULL, &n_size); // fails, but that's ok
	// get size

	if(!stl_ut::Resize_To_N(r_s_path, n_size + 1) ||
	   !GetUserProfileDirectoryA(h_proc_token, &r_s_path[0], &n_size)) {
		CloseHandle(h_proc_token);
		//CloseHandle(h_process_real);
		return false;
	}
	//CloseHandle(h_process_real);
	CloseHandle(h_proc_token);
	// get the directory

	r_s_path.resize(strlen(r_s_path.c_str()));
	// trim it

	_ASSERTE(!b_EndsWithSlash(r_s_path)); // shouldn't have on windows
#else // _WIN32 || _WIN64
	struct passwd *p_pw = getpwuid(getuid());
	if(!p_pw || !stl_ut::Assign_CStr(r_s_path, p_pw->pw_dir))
		return false;
#endif // _WIN32 || _WIN64

	Drop_TrailingSlash(r_s_path);
	_ASSERTE(b_Is_Normalized(r_s_path));
	_ASSERTE(b_Is_Absolute(r_s_path));
	// make sure there is no slash at the end, and that the path is normalized

	return true;
}

#endif // __CPATH_GET_USER_HOME_DIRECTORY

/**
 *	@brief normalizes a path (removes unnecessary dots and slashes, changes slashes
 *		to the OS convention, removes symlinks and drive links and similar)
 *
 *	@param[in,out] r_s_path is the normalized path to a directory, or a file
 *
 *	@return Returns true on success, false on failure.
 *
 *	@note This should only be relied on in simple cases, not involving symlinks and similar.
 *	@note This has a lot of problems. On windows, this function is not a standard API,
 *		and needs to be implemented by hand, with some quirks involved. On linux, this uses
 *		the realpath() function, which may have some memory allocation issues (will
 *		either always work, or always crash).
 *
 *	@todo Try to find perl path source code and see what they use.
 */
bool CPath::Normalize(std::string &r_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	return Canonicalize(r_s_path);
#else // _WIN32 || _WIN64
	std::replace(r_s_path.begin(), r_s_path.end(), '\\', '/');
	// replace back slashes by forward slashes

	char *p_s_path;
	if(!(p_s_path = realpath(r_s_path.c_str(), 0)))
		return false;
	// cannonicalize using realpath. problems follow:

	/*
		Never use this function. It is broken by design since it
		is impossible to determine a suitable size for the output
		buffer. According to POSIX a buffer of size PATH_MAX suf-
		fices, but PATH_MAX need not be a defined constant, and
		may have to be obtained using pathconf(). And asking
		pathconf() does not really help, since on the one hand
		POSIX warns that the result of pathconf() may be huge and
		unsuitable for mallocing memory. And on the other hand
		pathconf() may return -1 to signify that PATH_MAX is not
		bounded.

		The libc4 and libc5 implementation contains a buffer over-
		flow (fixed in libc-5.4.13). Thus, suid programs like
		mount need a private version.
	 */

	// note this could be solved using get / set cwd, but that
	// would require the path to exist and to be accessible

	bool b_result = stl_ut::AssignCStr(r_s_path, p_s_path);
	delete[] p_s_path;
	return b_result;
	// return result
#endif // _WIN32 || _WIN64
}

bool CPath::WeakNormalize(std::string &r_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	std::replace(r_s_path.begin(), r_s_path.end(), '/', '\\');
#else // _WIN32 || _WIN64
	std::replace(r_s_path.begin(), r_s_path.end(), '\\', '/');
#endif // _WIN32 || _WIN64
	// replace slashes by the correct ones

#if defined(_WIN32) || defined(_WIN64)
	if(r_s_path.length() >= 6 && isalpha(uint8_t(r_s_path[4]))) {
		if(!strncmp(r_s_path.c_str(), "\\\\?\\", 4) && r_s_path[5] == ':')
			r_s_path.erase(0, 4);
		else if(!strncmp(r_s_path.c_str(), "\\\\.\\", 4) && r_s_path[5] == ':')
			r_s_path.erase(0, 4);
	}
	// get rid of \\?\ and \\.\ prefixes on drive-letter paths

	if(r_s_path.length() >= 11) {
		if(!strncmp(r_s_path.c_str(), "\\\\?\\UNC\\", 8)) {
			if(isalpha(uint8_t(r_s_path[8])) && r_s_path[9] == ':' && r_s_path[10] == '\\')
				r_s_path.erase(0, 8);
			else if(r_s_path[8] == '\\')
				r_s_path.erase(0, 7);
		}
	}
	// get rid of \\?\UNC on drive-letter and UNC paths
#endif // _WIN32 || _WIN64
	// some path simplifications for windows

#if defined(_WIN32) || defined(_WIN64)
	bool b_uses_namespace = r_s_path.length() >= 4 && r_s_path[0] == '\\' && r_s_path[1] == '\\' &&
		(r_s_path[2] == '.' || r_s_path[2] == '?') && r_s_path[3] == '\\';
	char c_namespace_char = (b_uses_namespace)? r_s_path[2] : 0;
	if(b_uses_namespace)
		r_s_path.erase(0, 4);
#endif // _WIN32 || _WIN64
	// windows use "\\?\" and "\\.\" to denote namespaces, get rid of that

	std::vector<std::string> segment_list;
	const char p_s_separator[] = {path_Separator, 0};
	if(!stl_ut::Split(segment_list, r_s_path, p_s_separator))
		return false;
	// split the path to segments (a pussy way to do it, but less messy)

	try {
#if defined(_WIN32) || defined(_WIN64)
		std::string s_head;
		if(segment_list.size() >= 3 && segment_list[0].empty() && segment_list[1].empty() &&
		   !segment_list[2].empty()) {
			s_head = "\\\\" + segment_list[2];
			segment_list.erase(segment_list.begin(), segment_list.begin() + 3);
			// in case of "\\server\" paths, erase the server
		} else if(!segment_list.empty() && segment_list[0].length() == 2 &&
		   segment_list[0][1] == ':' && isalpha(uint8_t(segment_list[0][0]))) {
			s_head.swap(segment_list[0]);
			segment_list.erase(segment_list.begin());
			// in case of "c:\" paths, erase the drive specifier
		}
#endif // _WIN32 || _WIN64

#if defined(_WIN32) || defined(_WIN64)
		for(size_t i = segment_list.size(); i > 0; -- i) {
#else // _WIN32 || _WIN64
		for(size_t i = segment_list.size(); i > 1; -- i) {
			// linux absolute addresses start with "/", hence first empty segment should be ignored
#endif // _WIN32 || _WIN64
			if(segment_list[i - 1].empty())
				segment_list.erase(segment_list.begin() + (i - 1));
		}
		// in case there is a double slash, replace by a single one

		for(size_t i = segment_list.size(); i > 0; -- i) {
			if(!segment_list[i - 1].compare("."))
				segment_list.erase(segment_list.begin() + (i - 1));
		}
		// drop dots

		for(size_t i = 0, n = segment_list.size(); i < n; ++ i) {
			if(!segment_list[i].compare("..")) {
				if(i > 0 && segment_list[i - 1].compare("..")) { // is there somewhere to go up to?
					segment_list.erase(segment_list.begin() + (i - 1), segment_list.begin() + (i + 1));
					i -= 2; // repeat ...
					n -= 2; // !!
				}
			}
		}
		// drop double dots

#if defined(_WIN32) || defined(_WIN64)
		if(b_uses_namespace) {
			r_s_path = "\\\\";
			r_s_path += c_namespace_char;
			r_s_path += '\\';
			// begin with namespace declaration ("\\?\" or "\\.\")
		} else
			r_s_path.erase();
		if(!s_head.empty()) {
			r_s_path += s_head; // prepend share or drive specification
			if(!segment_list.empty())
				r_s_path += path_Separator;
		}
#else // _WIN32 || _WIN64
		r_s_path.erase();
#endif // _WIN32 || _WIN64
		// clear output string

		for(size_t i = 0, n = segment_list.size(); i < n; ++ i) {
			if(i)
				r_s_path += path_Separator;
			r_s_path += segment_list[i];
		}
	} catch(std::bad_alloc&) {
		return false;
	}

	return true;
}

bool CPath::Join(std::string &r_s_dest, const std::string &r_s_head, const std::string &r_s_tail) // ends with slash only if tail does
{
	if(b_Is_Absolute(r_s_tail))
		return &r_s_dest == &r_s_tail || stl_ut::Assign(r_s_dest, r_s_tail); // tail is dest, or overwrite dest with tail
	// in case tail is an absolute path, discard head

	if(&r_s_dest == &r_s_tail) {
		std::string s_temp_tail;
		return stl_ut::Assign(s_temp_tail, r_s_tail) && Join(r_s_dest, r_s_head, s_temp_tail);
	}
	_ASSERTE(&r_s_dest != &r_s_tail);
	// in case destination string is tail, need to copy it to temp storage first

	const char p_s_slash[] = {path_Separator, 0};
	return (&r_s_head == &r_s_dest || stl_ut::Assign(r_s_dest, r_s_head)) && // head is dest or copy head to dest
		(r_s_head.empty() || b_EndsWithSlash(r_s_head) || stl_ut::AppendCStr(r_s_dest, p_s_slash)) && // head is empty or contains a slash or append slash
		stl_ut::Append(r_s_dest, r_s_tail); // copy tail
	// note this path may be not normalized (contain ".." or "." or double slashes)
}

bool CPath::Join(std::string &r_s_dest, const char *p_s_head, const char *p_s_tail) // ends with slash only if tail does
{
	if(b_Is_Absolute(p_s_tail))
		return r_s_dest.c_str() == p_s_tail || stl_ut::AssignCStr(r_s_dest, p_s_tail); // tail is dest, or overwrite dest with tail
	// in case tail is an absolute path, discard head

	if(r_s_dest.c_str() == p_s_tail) {
		std::string s_temp_tail;
		return stl_ut::AssignCStr(s_temp_tail, p_s_tail) &&
			Join(r_s_dest, p_s_head, s_temp_tail.c_str());
	}
	_ASSERTE(r_s_dest.c_str() != p_s_tail);
	// in case destination string is tail, need to copy it to temp storage first

	const char p_s_slash[] = {path_Separator, 0};
	return (p_s_head == r_s_dest.c_str() || stl_ut::AssignCStr(r_s_dest, p_s_head)) && // head is dest or copy head to dest
		(b_EndsWithSlash(p_s_head) || stl_ut::AppendCStr(r_s_dest, p_s_slash)) && // head contains slash or append slash
		stl_ut::AppendCStr(r_s_dest, p_s_tail); // copy tail
	// note this path may be not normalized (contain ".." or "." or double slashes)
}

bool CPath::To_Relative(std::string &r_s_path, const std::string &r_s_current_dir)
{
	if(!b_Is_Absolute(r_s_current_dir)) {
		std::string s_abs_current;
		return To_Absolute(s_abs_current, r_s_current_dir) && To_Relative(r_s_path, s_abs_current);
	}
	_ASSERTE(b_Is_Absolute(r_s_current_dir));
	// make sure the current directory is absolute

	if(!b_Is_Absolute(r_s_path) && !To_Absolute(r_s_path))
		return false;
	// make sure the path to convert to relative is also absolute

	_ASSERTE(0); // not implemented
	// todo - match the head of the absolute path with current directory,
	// skip matched parts, for every unmatched of r_s_current_dir, there's one double dot,
	// copy the rest of the path after that
	// note r_s_current_dir or r_s_path may contain '.' or '..' or '//' themselves

	return true;
}

bool CPath::To_Absolute(std::string &r_s_path) // ends with slash only if r_s_path does
{
	if(b_Is_Absolute(r_s_path))
		return true;
	// in case it's absolute, do nothing

	std::string s_cwd;
	return Get_CurrentDirectory(s_cwd) && Join(r_s_path, s_cwd, r_s_path);
	// in case it's relative, join with current directory
	// note this path may be not normalized (contain ".." or "." or double slashes)
}

bool CPath::To_Absolute(std::string &r_s_dest, const std::string &r_s_path) // ends with slash only if r_s_path does
{
	return (&r_s_dest == &r_s_path || stl_ut::Assign(r_s_dest, r_s_path)) && To_Absolute(r_s_dest);
}

// note that this simply erases the last component of a filename
// whether the filename actually points to a directory is irrelevant
// note that loosely speaking, <tt>Get_Path("a/b/c/") + Get_Filename("a/b/c/") = "a/b/c/c"</tt>
// however, <tt>Get_Path("a/b/c") + Get_Filename("a/b/c") = "a/b/c"</tt>
bool CPath::Get_Path(std::string &r_s_path, const std::string &r_s_filename) // never ends with slash
{
	size_t n_pos = r_s_filename.find_last_of("\\/");

	if(n_pos == std::string::npos) {
		r_s_path.erase();
		/*try {
			r_s_path = "."; // it was a file in the current directory
		} catch(std::bad_alloc&) {
			return false;
		}*/
		return true;
	}
	// no slash, the whole string is filename (no path)

	while(n_pos > 0 && (r_s_filename[n_pos - 1] == '\\' || r_s_filename[n_pos - 1] == '/'))
		-- n_pos;
	// there might be more of the slashes

	//++ n_pos; // don't (changed on 2014-12-12, the rest of the function does not return either)
	// keep slash

#if !defined(_WIN32) && !defined(_WIN64)
	if(!n_pos)
		n_pos = 1;
	// don't delete the only slash (has a specific meaning in linux)
#endif // !_WIN32 && !_WIN64

	if(&r_s_path == &r_s_filename)
		r_s_path.erase(n_pos);
	else {
		try {
			r_s_path.erase();
			r_s_path.insert(r_s_path.begin(), r_s_filename.begin(), r_s_filename.begin() + n_pos);
		} catch(std::bad_alloc&) {
			return false;
		}
	}
	/*try {
		if(r_s_path.empty())
			r_s_path = "."; // it was a file in the current directory
	} catch(std::bad_alloc&) {
		return false;
	}*/
	return true;
}

// note that this simply returns only the last component of the path
// whether it is a directory or a file does not matter
// note that loosely speaking, <tt>Get_Path("a/b/c/") + Get_Filename("a/b/c/") = "a/b/c/c"</tt>
// however, <tt>Get_Path("a/b/c") + Get_Filename("a/b/c") = "a/b/c"</tt>
bool CPath::Get_Filename(std::string &r_s_name, const std::string &r_s_filename)
{
	/*size_t n_pos = r_s_filename.find_last_of("\\/");

	if(n_pos == std::string::npos || n_pos == r_s_filename.length() - 1) { // fixme
		r_s_name.erase();
		return true;
	}
	// no slash, the whole string is filename

	++ n_pos;
	// delete the slash

	if(&r_s_name == &r_s_filename)
		r_s_name.erase(0, n_pos);
	else {
		try {
			r_s_name.erase();
			r_s_name.insert(r_s_name.begin(), r_s_filename.begin() + n_pos, r_s_filename.end());
		} catch(std::bad_alloc&) {
			return false;
		}
	}*/

	if(&r_s_name == &r_s_filename) {
		std::string s_temp;
		if(!Get_Filename(s_temp, r_s_filename))
			return false;
		std::swap(s_temp, r_s_name);
		return true;
	}
	// may not overlap

	return stl_ut::AssignCStr(r_s_name, p_s_Get_Filename(r_s_name.c_str()));
}

const char *CPath::p_s_Get_Filename(const char *p_s_path)
{
	/*const char *b = p_s_path, *e = p_s_path + strlen(p_s_path);
	while(e > b && *(e - 1) != '/' && *(e - 1) != '\\')
		-- e;
	return e;*/
	const char *p_s_last_slash = p_s_path;
	while(*p_s_path) {
		if((*p_s_path == '/' || *p_s_path == '\\') && p_s_path[1] != 0 && p_s_path[1] != '/' && p_s_path[1] != '\\')
			p_s_last_slash = p_s_path + 1; // not empty and not starting with another slash
		++ p_s_path;
	}
	return p_s_last_slash;
}

bool CPath::Get_Extension(std::string &r_s_extension, const std::string &r_s_filename)
{
	size_t n_pos = r_s_filename.find_last_of("\\/");
	if(n_pos == std::string::npos)
		n_pos = 0; // no slash, the whole string is a filename
	else if(n_pos == r_s_filename.length() - 1) { // fixme
		r_s_extension.erase(); // slash at the end, the whole string is path (no filename, no extension)
		return true;
	}
	size_t n_pos2 = r_s_filename.rfind('.');
	if(n_pos2 == std::string::npos || n_pos2 < n_pos) {
		r_s_extension.erase(); // dot not found, or found before slash
		return true;
	}
	// find the position of the dot

	if(&r_s_extension == &r_s_filename)
		r_s_extension.erase(0, n_pos2);
	else {
		try {
			r_s_extension.erase();
			r_s_extension.insert(r_s_extension.begin(), r_s_filename.begin() + n_pos2, r_s_filename.end());
		} catch(std::bad_alloc&) {
			return false;
		}
	}

	return true;
}

bool CPath::Split(std::string &r_s_path, std::string &r_s_name, const std::string &r_s_filename) // the path does not end with the slash
{
	_ASSERTE(&r_s_path != &r_s_name); // make sure those two do not point to the same string

	size_t n_pos = r_s_filename.find_last_of("\\/");
	if(n_pos == std::string::npos) {
		if(&r_s_filename != &r_s_name &&
		   !stl_ut::Assign(r_s_name, r_s_filename))
			return false;
		r_s_path.erase();
		/*try {
		if(r_s_path.empty())
			r_s_path = "."; // it was a file in the current directory
		} catch(std::bad_alloc&) {
			return false;
		}*/
		// no separator, the whole path is a filename then

		return true;
	}

	size_t n_last_pos = n_pos; // where the filename sparts

	while(n_pos > 0 && (r_s_filename[n_pos - 1] == '\\' || r_s_filename[n_pos - 1] == '/'))
		-- n_pos;
	// there might be more of the slashes

#if !defined(_WIN32) && !defined(_WIN64)
	if(!n_pos)
		n_pos = 1;
	// don't delete the only slash (has a specific meaning in linux)
#endif // !_WIN32 && !_WIN64

	if(&r_s_path != &r_s_filename) {
		try {
			r_s_path.erase();
			r_s_path.insert(r_s_path.begin(), r_s_filename.begin(),
				r_s_filename.begin() + (n_pos /*+ 1*/)); // don't keep the slash (changed on 2014-12-12, the rest of the function does not return either)
		} catch(std::bad_alloc&) {
			return false;
		}
	}
	if(&r_s_name != &r_s_filename) {
		try {
			r_s_name.erase();
			r_s_name.insert(r_s_name.begin(), r_s_filename.begin() +
				(n_last_pos + 1), r_s_filename.end());
		} catch(std::bad_alloc&) {
			return false;
		}
	}
	if(&r_s_path == &r_s_filename)
		r_s_path.erase(n_pos /*+ 1*/); // don't keep the slash (changed on 2014-12-12, the rest of the function does not return either)
	if(&r_s_name == &r_s_filename)
		r_s_name.erase(0, n_last_pos + 1);
	// split the path, mind that some strings may share storage

	/*try {
		if(r_s_path.empty())
			r_s_path = "."; // it was a file in the current directory
	} catch(std::bad_alloc&) {
		return false;
	}*/

	return true;
}

bool CPath::Expand_SystemVariables(std::string &r_s_dest, const std::string &r_s_path)
{
	_ASSERTE(0); // todo
#if defined(_WIN32) || defined(_WIN64)
#else // _WIN32 || _WIN64
#endif // _WIN32 || _WIN64

	return true;
}

bool CPath::b_Is_Absolute(const std::string &r_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	return (r_s_path.length() >= 2 && (r_s_path[0] == '/' || r_s_path[0] == '\\') &&
		(r_s_path[1] == '/' || r_s_path[1] == '\\')) ||
		// double backslash marks UNC address

		(r_s_path.length() >= 3 && isalpha(uint8_t(r_s_path[0])) && r_s_path[1] == ':' &&
		(r_s_path[2] == '/' || r_s_path[2] == '\\')) ||
		// disk designator

		false;// (r_s_path.length() >= 1 && (r_s_path[0] == '/' || r_s_path[0] == '\\'));
		// single backslash (wtf?) // todo - check this
#else // _WIN32 || _WIN64
	return r_s_path.length() >= 1 && r_s_path[0] == '/'; // and that should be it
#endif // _WIN32 || _WIN64
}

bool CPath::Normalize_Case(std::string &r_s_dest, const std::string &r_s_path)
{
	if(&r_s_dest != &r_s_path && !stl_ut::Assign(r_s_dest, r_s_path))
		return false;
	// copy the path, if needed

#if defined(_WIN32) || defined(_WIN64)
	std::replace(r_s_dest.begin(), r_s_dest.end(), '/', '\\');
	// replace slashes for backslashes

	size_t n_begin = 0;
	if(r_s_dest.length() >= 2 && isupper(uint8_t(r_s_dest[0])) && r_s_dest[1] == ':') {
		r_s_dest[0] = tolower(uint8_t(r_s_dest[0]));
		n_begin = 2;
	}
	// in case there is drive letter, make it lowercase

	if(r_s_dest.length() >= 6 && isupper(uint8_t(r_s_dest[4]))) {
		if((!strncmp(r_s_dest.c_str(), "\\\\?\\", 4) && r_s_dest[5] == ':') ||
		   (!strncmp(r_s_dest.c_str(), "\\\\.\\", 4) && r_s_dest[5] == ':')) {
			r_s_dest[4] = tolower(uint8_t(r_s_dest[4]));
			n_begin = 6;
		}
	}
	// in case there is drive letter, make it lowercase (drive letter with "\\?\" or "\\.\" prefix)

	if(r_s_dest.length() >= 11) {
		if(!strncmp(r_s_dest.c_str(), "\\\\?\\", 4) &&
		   toupper(uint8_t(r_s_dest[4])) == 'U' &&
		   toupper(uint8_t(r_s_dest[5])) == 'N' &&
		   toupper(uint8_t(r_s_dest[6])) == 'C' && r_s_dest[7] == '\\') {
		    r_s_dest[4] = 'U';
		    r_s_dest[5] = 'N';
		    r_s_dest[6] = 'C';
			// makes sure 'UNC' is uppercase

			if(isalpha(uint8_t(r_s_dest[8])) && r_s_dest[9] == ':' && r_s_dest[10] == '\\') {
				if(isupper(uint8_t(r_s_dest[8])))
					r_s_dest[8] = tolower(uint8_t(r_s_dest[8]));
				n_begin = 11;
			} else
				n_begin = 7;
			// in case there is drive letter, make it lowercase
		}
	}
	// handle \\?\UNC on drive-letter

	for(size_t i = n_begin, n = r_s_dest.length(); i < n; ++ i)
		r_s_dest[i] = tolower(uint8_t(r_s_dest[i]));
	// transform the rest of the string to lowercase
#else // _WIN32 || _WIN64
	// nothing to be done here ...
#endif // _WIN32 || _WIN64

	return true;
}

const char *CPath::p_s_NullPath()
{
#if defined(_WIN32) || defined(_WIN64)
	const char *p_s_null = "nul";
#else // _WIN32 || _WIN64
	const char *p_s_null = "/dev/null";
#endif // _WIN32 || _WIN64

	return p_s_null;
}

bool CPath::Get_NullPath(std::string &r_s_null_path)
{
	return stl_ut::AssignCStr(r_s_null_path, p_s_NullPath());
}

std::pair<const char*, const char*> CPath::t_ShortFileName(const char *p_s_filename, size_t n_max_length /*= 80*/)
{
	_ASSERTE(p_s_filename);
	_ASSERTE(n_max_length > 4);
	size_t n_len;
	if((n_len = strlen(p_s_filename)) > n_max_length) {
		const char *p_s_end = p_s_filename + n_len;
		const char *p_s_shorter = p_s_filename;
		do {
#if defined(_WIN32) || defined(_WIN64)
			const char *p_s_pattern = "/\\";
			const char *p_s_slash = std::find_first_of(p_s_shorter,
				p_s_end, p_s_pattern, p_s_pattern + 2);
			// windows accept both kinds of slashes, making it messy
#else // _WIN32 || _WIN64
			const char *p_s_slash = std::find(p_s_shorter, p_s_end, (char)CPath::path_Separator);
#endif // _WIN32 || _WIN64
			if(p_s_slash != p_s_end)
				p_s_shorter = p_s_slash + 1;
			else
				break;
			_ASSERTE(p_s_end > p_s_shorter); // will never produce an empty string
		} while(size_t(p_s_end - p_s_shorter) > n_max_length - 4);
		_ASSERTE(p_s_end > p_s_shorter); // will never produce an empty string
		if(size_t(p_s_end - p_s_shorter) <= n_max_length - 4)
			return std::make_pair((const char*)((CPath::path_Separator == '\\')? "...\\" : ".../"), p_s_shorter);
		else { /*if((p_s_end - p_s_shorter) > n_max_length - 4)*/ // always
			_ASSERTE(size_t(p_s_end - p_s_shorter) + 3 >= n_max_length); // make sure it does not underflow
			return std::make_pair((const char*)"...", p_s_shorter + ((p_s_end - p_s_shorter) + 3 - n_max_length));
		}
	} else
		return std::make_pair((const char*)"", p_s_filename);
	// MSVC 60 needs (const char*) before the constant string, otherwise it instantiates std::pair<char[4], char*> 
}

bool CPath::Make_Directory(const char *p_s_path, bool b_skip_if_exists /*= false*/)
{
	if(b_skip_if_exists) {
		TFileInfo t_dir(p_s_path);
		if(t_dir.b_exists && t_dir.b_directory)
			return true; // if already exists, don't try to create it
		if(t_dir.b_exists)
			return false; // there is a file of the same name, won't be able to create the directory
	}
#if defined(_WIN32) || defined(_WIN64)
	return !_mkdir(p_s_path); // direct.h in windows
#else // _WIN32 || _WIN64
	return !mkdir(p_s_path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // sys/stat.h in unix
#endif // _WIN32 || _WIN64
}

bool CPath::Make_Directories(const char *p_s_path)
{
	try {
		std::string s_path_part;
		size_t n_pos = 0;
#if defined(_WIN32) || defined(_WIN64)
		if(//r_s_path.length() >= 4 &&
		   p_s_path[0] == '\\' && p_s_path[1] == '\\' && p_s_path[2] != 0 && p_s_path[3] == '\\')
			n_pos = 4; // skip UNC file name
		else if(/*r_s_path.length() >= 3 &&*/ isalpha(uint8_t(p_s_path[0])) &&
		   p_s_path[1] == ':' && p_s_path[2] == '\\')
			n_pos = 3; // skip drive letter name
#endif // _WIN32 || _WIN64
		const char *e = p_s_path + strlen(p_s_path), *p_s_sep = "/\\", *p_where;
		while((p_where = std::find_first_of(p_s_path + n_pos, e, p_s_sep, p_s_sep + 2)) != e) {
			n_pos = p_where - p_s_path;
			if(n_pos) {
				s_path_part.erase();
				s_path_part.insert(s_path_part.begin(), p_s_path, p_s_path + n_pos);
				if(!Make_Directory(s_path_part.c_str(), true))
					return false;
			}
			++ n_pos; // to skip over the slash for the next round
		}
	} catch(std::bad_alloc&) {
		return false;
	}
	// make all the preceding levels

	return Make_Directory(p_s_path, true); // the last level
}

bool CPath::Remove_Directory(const char *p_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	return !_rmdir(p_s_path); // unistd.h in unix or direct.h in windows
#else // _WIN32 || _WIN64
	return !rmdir(p_s_path); // unistd.h in unix or direct.h in windows
#endif // _WIN32 || _WIN64
}

bool CPath::Recycle(const char *p_s_path) // can be a both file or a dir
{
#if defined(_WIN32) || defined(_WIN64)
	std::string s_double_term;
	if(!stl_ut::Resize_To_N(s_double_term, strlen(p_s_path) + 1))
		return false;
	memcpy(&s_double_term[0], p_s_path, s_double_term.length() * sizeof(char));
	p_s_path = s_double_term.c_str(); // adds the second 0

	SHFILEOPSTRUCTA t_sfo = {0};
	t_sfo.wFunc = FO_DELETE;
	t_sfo.pFrom = p_s_path;
	t_sfo.fFlags = FOF_ALLOWUNDO | FOF_SILENT | FOF_NOCONFIRMATION | FOF_NOERRORUI | FOF_NOCONFIRMMKDIR |
#ifndef FOF_WANTNUKEWARNING
		/*FOF_WANTNUKEWARNING*/0x4000; // FOF_WANTNUKEWARNING not defined everywhere
#else // !FOF_WANTNUKEWARNING
		FOF_WANTNUKEWARNING;
#endif // !FOF_WANTNUKEWARNING
	// prepare structures for recycling rather than deleting

	return !SHFileOperationA(&t_sfo) && !t_sfo.fAnyOperationsAborted;
#else // _WIN32 || _WIN64
	const char *p_s_trash;
#ifdef __APPLE__
	p_s_trash = "~/.Trash";
#else // __APPLE__
	p_s_trash = "~/.trash";
#endif // __APPLE__
	std::string s_cmd;
	return Make_Directory(p_s_trash, true) && // might not exist on linux
		stl_ut::Format(s_cmd, "/bin/mv \"%s\" %s", p_s_path, p_s_trash) && !system(s_cmd.c_str()); // don't use Move_File() as it may fail
#endif // _WIN32 || _WIN64
}

// in windows, this moves to the recycle bin by default. it is considered a bit dangerous to just let it delete everything
bool CPath::Remove_DirTree(const char *p_s_path, bool b_user_confirm,
	bool b_verbose /*= false*/, bool b_recycle /*= true*/) // b_recycle is the last to make sure that it doesn't get changed by accident If wanting to change some other param
{
	if(!p_s_path || !*p_s_path)
		return false; // not empty paths

	std::string s_noslash;
	if(b_EndsWithSlash(p_s_path)) {
		if(!stl_ut::AssignCStr(s_noslash, p_s_path))
			return false;
		Drop_TrailingSlash(s_noslash);
		p_s_path = s_noslash.c_str();
	}
	// fails if ends with slash

	TFileInfo t_dir(p_s_path);
	if(!t_dir.b_Valid() || !t_dir.b_exists || !t_dir.b_directory)
		return false;
	// must exist and be a directory

	CRemoveDirTree remover(b_user_confirm, b_verbose, b_recycle);
	return remover(t_dir) && !remover.b_Failed();
	// will recurse by itself
}

bool CPath::Remove_File(const char *p_s_filename)
{
	return !remove(p_s_filename); // stdio.h
}

bool CPath::Move_File(const char *p_s_new_filename, const char *p_s_old_filename) // may fail to move a file accross disk volumes or other, you may need to copy and delete if this fails
{
	return !rename(p_s_old_filename, p_s_new_filename); // stdio.h
	// if oldname and newname specify different paths and this is supported by the system, the file is moved to the new location
}

bool CPath::Copy_File(const char *p_s_new_filename,
	const char *p_s_old_filename, bool b_force_overwrite /*= false*/)
{
#if defined(_WIN32) || defined(_WIN64)
	return ::CopyFileA(p_s_old_filename, p_s_new_filename, !b_force_overwrite) != 0;
	// file attributes for the existing file are copied to the new file
	// symbolic link behavior-if the source file is a symbolic link, the actual file copied is the target of the symbolic link
	// if the destination file already exists and is a symbolic link, the target of the symbolic link is overwritten by the source file
#else
	if(TFileInfo(p_s_old_filename).b_directory)
		return false; // cp can copy directories, we don't want to allow that here
	std::string s_cmd;
	return stl_ut::Format(s_cmd, "/bin/cp %s --preserve --no-target-directory \"%s\" \"%s\"",
		(b_force_overwrite)? "" : "--no-clobber", p_s_old_filename, p_s_new_filename) && !system(s_cmd.c_str());
	// preserve attributes, treat target as a file and optionally disable 
#endif
}

FILE *CPath::p_OpenFile(const char *p_s_filename, const char *p_s_mode)
{
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	FILE *p_file;
	if(!fopen_s(&p_file, p_s_filename, p_s_mode))
		return p_file;
	return 0; // fail
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	return fopen(p_s_filename, p_s_mode);
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
}

bool CPath::b_EndsWithSlash(const std::string &r_s_path)
{
	if(r_s_path.empty())
		return false;
	char n_slash = r_s_path[r_s_path.length() - 1];
	return n_slash == '/' || n_slash == '\\';
}

bool CPath::b_EndsWithSlash(const char *p_s_path)
{
	if(!*p_s_path)
		return false;
	char n_slash = p_s_path[strlen(p_s_path) - 1];
	return n_slash == '/' || n_slash == '\\'; // todo - modify all the functions - in linux, a backslash shouldn't slip by
}

//static bool b_Is_Normalized(const std::string &r_s_path); // hard to determine; rename to b_is_sane or something like that
//static bool b_Is_Normalized(const char *p_s_path); // todo - decide what that should do (find bad characters? find the opposite slashes? find dots and double dots?)
bool CPath::b_Is_Normalized(const std::string &UNUSED(r_s_path))
{
	return true;
}

void CPath::Drop_TrailingSlash(std::string &r_s_path)
{
	if(r_s_path.empty())
		return;
	char &r_n_back = r_s_path[r_s_path.length() - 1];
	while(r_n_back == '/' || r_n_back == '\\') // there might be more of them
		r_s_path.erase(r_s_path.end() - 1);
}

#if defined(_WIN32) || defined(_WIN64)

bool CPath::Canonicalize(std::string &r_s_path)
{
	// this function is a copy of the one, described at:
	// http://pdh11.blogspot.cz/2009/05/pathcanonicalize-versus-what-it-says-on.html

	char p_s_canonical[MAX_PATH];
	if(!::GetFullPathNameA(r_s_path.c_str(), MAX_PATH, p_s_canonical, NULL))
		return false;
	// note that PathCanonicalize does NOT do what we want here, it's a
	// purely textual operation that eliminates /./ and /../ only

	std::string s_new_path = p_s_canonical;
	if(s_new_path.length() >= 6 && isalpha(uint8_t(s_new_path[4]))) {
		if(!strncmp(s_new_path.c_str(), "\\\\?\\", 4) && s_new_path[5] == ':')
			s_new_path.erase(0, 4);
		else if(!strncmp(s_new_path.c_str(), "\\\\.\\", 4) && s_new_path[5] == ':')
			s_new_path.erase(0, 4);
	}
	// get rid of \\?\ and \\.\ prefixes on drive-letter paths

	if(s_new_path.length() >= 11) {
		if(!strncmp(s_new_path.c_str(), "\\\\?\\UNC\\", 8)) {
			if(isalpha(uint8_t(s_new_path[8])) && s_new_path[9] == ':' && s_new_path[10] == '\\')
				s_new_path.erase(0, 8);
			else if(s_new_path[8] == '\\')
				s_new_path.erase(0, 7);
		}
	}
	// get rid of \\?\UNC on drive-letter and UNC paths

	// anything other than UNC and drive-letter is something we don't understand
	
	if(s_new_path.length() >= 2) {
		if(s_new_path[0] == '\\' && s_new_path[1] == '\\') {
			if(s_new_path[2] == '?' || s_new_path[2] == '.')
				return true; // not understood
			// it's UNC
		} else if(isalpha(uint8_t(s_new_path[0])) && s_new_path[1] == ':') {
			// it's a drive letter, need to potentially unwind substituted letters
			for (;;) {
				char p_s_drive[3];
				p_s_drive[0] = (char)toupper(uint8_t(s_new_path[0]));
				p_s_drive[1] = ':';
				p_s_drive[2] = 0;
				p_s_canonical[0] = 0;
				if(!::QueryDosDeviceA(p_s_drive, p_s_canonical, MAX_PATH))
					break;
				if(!strncmp(p_s_canonical, "\\??\\", 4)) {
					s_new_path = std::string(p_s_canonical + 4) +
						std::string(s_new_path.begin(), s_new_path.begin() + 2);
				} else {
					s_new_path[0] = (char)tolower(uint8_t(s_new_path[0])); // make sure a lowercase letter is used
					break; // not substituted, done
				}
			}

			char p_s_drive[4];
			p_s_drive[0] = (char)toupper(uint8_t(s_new_path[0]));
			p_s_drive[1] = ':';
			p_s_drive[2] = '\\';
			p_s_drive[3] = 0;

			if(::GetDriveTypeA(p_s_drive) == DRIVE_REMOTE) {
				p_s_drive[2] = '\0';
				// QueryDosDevice() and WNetGetConnection() forbid the
				// trailing slash; GetDriveType() requires it

#ifdef __CPATH_NORMALIZE_NETWORK_DRIVE_SUPPORT
				DWORD bufsize = MAX_PATH;
				if(::WNetGetConnectionA(p_s_drive, p_s_canonical, &bufsize) == NO_ERROR)
					s_new_path = std::string(p_s_canonical) + std::string(s_new_path.begin(), s_new_path.begin() + 2);
#else // __CPATH_NORMALIZE_NETWORK_DRIVE_SUPPORT
				break; // can't substitute
#endif // __CPATH_NORMALIZE_NETWORK_DRIVE_SUPPORT
			}
		} else {
			return true;
			// not understood
		}
	} else {
		return true;
		// not understood
	}

	if(::GetLongPathNameA(s_new_path.c_str(), p_s_canonical, MAX_PATH)) // will fail on nonexistent paths
		r_s_path = p_s_canonical; // copy back to dest
	else
		r_s_path.swap(s_new_path); // otherwise use the path as is (possibly with 8.3 names)
	// remove 8.3 names

	std::replace(r_s_path.begin(), r_s_path.end(), '/', '\\');
	// replace forward slashes by backslashes

	return true;
}

#endif // _WIN32 || _WIN64

/*
 *								=== ~CPath ===
 */

/*
 *								=== CPath::CRemoveDirTree ===
 */

CPath::CRemoveDirTree::CRemoveDirTree(bool b_user_confirm,
	bool b_verbose /*= false*/, bool b_recycle /*= true*/) // b_recycle is the last to make sure that it doesn't get changed by accident If wanting to change some other param
	:m_b_user_confirm(b_user_confirm), m_b_verbose(b_verbose),
	m_b_recycle(b_recycle), m_b_fails(false)
{}

bool CPath::CRemoveDirTree::b_Failed() const
{
	return m_b_fails;
}

bool CPath::CRemoveDirTree::b_UserConfirm(const char *p_s_path)
{
	if(!m_b_user_confirm) {
		if(m_b_verbose)
			printf("delete \'%s\'\n", p_s_path);
		return true;
	}
	printf("confirm delete \'%s\' (Yes/No/All): ", p_s_path);
	for(;;) {
		char c = fgetc(stdin);
		switch(tolower(uint8_t(c))) {
		case 'y':
			return true;
		case 'n':
			return false;
		case 'a':
			m_b_user_confirm = false;
			m_b_verbose = true; // echo the rest of the deleted files to the user
			return true;
		}
	}
}

bool CPath::CRemoveDirTree::operator ()(const TFileInfo &r_t_file)
{
	const char *p_s_path = r_t_file.p_s_Path();

	if(!b_UserConfirm(p_s_path)) // user chose not to delete?
		return true;

	if(!r_t_file.b_directory) {
		if(m_b_recycle)
			m_b_fails = !CPath::Recycle(p_s_path) || m_b_fails; // order!
		else
			m_b_fails = !CPath::Remove_File(p_s_path) || m_b_fails; // order!
	}
	// remove files immediately

	if(r_t_file.b_directory) {
		if(!CDirTraversal::Traverse2(p_s_path, *this, false))
			return false;
		// first traverse inside

		if(m_b_recycle)
			m_b_fails = !CPath::Recycle(p_s_path) || m_b_fails; // order!
		else
			m_b_fails = !CPath::Remove_Directory(p_s_path) || m_b_fails; // order!
		// remove the directory once empty
	}
	// handle directories after first recursing inside

	return true;
	// don't fail, saying no on a single file would terminate the deletion upon failing
	// to remove the parent folder and wouldn't proceed to the sibling folders
}

/*
 *								=== ~CPath::CRemoveDirTree ===
 */
